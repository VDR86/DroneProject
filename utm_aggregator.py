"""
utm_aggregator.py
=================

The read-only MONITORING backend for the GridEye. It is the de-weaponized
cousin of the fleet proxy: it only LISTENS. No commands, no spoofing.

  * Listens on ONE UDP port (lightweight_fleet.INGEST_PORT). Every drone - the 10
    lightweight ones and the 2 real SITL ones (fed in via MAVProxy --out) -
    arrives in this single stream, separated by MAVLink system id.
  * Keeps a live table: for each drone, its company, position, altitude, heading,
    speed, and how long since it was last heard.
  * Serves that table as JSON at  http://localhost:8000/api/drones  and serves
    the map page at  http://localhost:8000/  (once utm_map.html exists).

Run AFTER starting lightweight_fleet.py (and optionally the 2 SITL drones):
    python3 utm_aggregator.py
then open http://localhost:8000/ in the browser.
"""

import json
import math
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

import lightweight_fleet as lf  # the scenario's single source of truth

HTTP_PORT = 8000
STALE_S = 4.0   # no telemetry for longer than this -> mark the drone "stale"

EVAL_S = 0.5              # how often the violation check runs
DEPOT_SUPPRESS_M = 200.0  # within this range of a depot, ignore the low-alt rule
STOP_SPEED = 0.6          # m/s at or below this counts as "hovering / stopped"
DELIVERY_RADIUS_M = 120.0 # stopped within this of a delivery point = a drop

# Live state: sysid -> dict of latest values. Guarded by `lock`.
state = {}
lock = threading.Lock()

# Alerts (guarded by `lock`): current violations per drone, plus an edge-event feed.
alerts_state = {}                  # sysid -> {code: message} currently active
notifications = deque(maxlen=200)  # {t, sysid, company, kind, code, text}, newest first

# Deliveries (guarded by `lock`).
delivery_count = 0        # total completed deliveries since start
delivered_ts = {}         # sysid -> epoch of that drone's most recent delivery

# ArduCopter custom_mode number -> readable name (the common ones).
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    16: "POSHOLD", 17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}


# ---------------------------------------------------------------------------
# INGEST: one listener hears the whole fleet by system id
# ---------------------------------------------------------------------------
def ingest(stop):
    conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{lf.INGEST_PORT}")
    print(f"[GridEye] listening for the fleet on UDP {lf.INGEST_PORT}")
    while not stop.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        sid = msg.get_srcSystem()
        mtype = msg.get_type()

        if mtype == "GLOBAL_POSITION_INT":
            company, kind = lf.FLEET.get(sid, ("Unknown", "?"))
            speed = math.hypot(msg.vx, msg.vy) / 100.0  # vx/vy cm/s -> m/s
            with lock:
                d = state.setdefault(sid, {})
                d.update(
                    sysid=sid,
                    company=company,
                    kind=kind,
                    color=lf.COMPANY_COLOR.get(company, "#888888"),
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                    alt_m=round(msg.relative_alt / 1000.0, 1),   # above ground
                    amsl_m=round(msg.alt / 1000.0, 1),           # above sea level
                    hdg=(round(msg.hdg / 100.0, 1) if msg.hdg != 65535 else None),
                    speed_ms=round(speed, 1),
                    last_seen=time.time(),
                )
        elif mtype == "VFR_HUD":
            # Real SITL drones send this; use its groundspeed if we have a record.
            with lock:
                if sid in state:
                    state[sid]["speed_ms"] = round(msg.groundspeed, 1)
        elif mtype == "HEARTBEAT":
            # Flight mode lives in custom_mode (valid when the custom-mode flag is set).
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                mode = COPTER_MODES.get(msg.custom_mode, f"MODE{msg.custom_mode}")
                with lock:
                    state.setdefault(sid, {})["mode"] = mode
        elif mtype == "SYS_STATUS":
            # Battery remaining %, -1 when unknown.
            with lock:
                batt = msg.battery_remaining
                state.setdefault(sid, {})["battery"] = batt if batt >= 0 else None
        elif mtype == "POSITION_TARGET_GLOBAL_INT":
            # Where the drone is currently headed (its active waypoint).
            with lock:
                if sid in state:
                    state[sid]["tgt_lat"] = msg.lat_int / 1e7
                    state[sid]["tgt_lon"] = msg.lon_int / 1e7


# ---------------------------------------------------------------------------
# GEOMETRY + RULES
# ---------------------------------------------------------------------------
EARTH_R = 6_371_000.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def point_in_poly(lat, lon, poly):
    """Ray casting; poly is [(lat, lon), ...]."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        yi, xi = poly[i]
        yj, xj = poly[j]
        if ((yi > lat) != (yj > lat)) and \
           (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def violations(d):
    """Return {code: message} for every rule this drone currently breaks."""
    out = {}
    lat, lon, alt = d.get("lat"), d.get("lon"), d.get("alt_m")
    if lat is None or alt is None:
        return out
    company = d.get("company")

    # 1) above the 400 ft ceiling
    if alt > lf.MAX_ALT_M:
        out["HIGH"] = f"above 400 ft ({alt:.0f} m)"

    # 2) below the 10 m floor — but only in transit (away from any depot)
    if alt < lf.MIN_ALT_M:
        near_depot = any(
            haversine_m(lat, lon, dep[0], dep[1]) < DEPOT_SUPPRESS_M
            for dep in lf.DEPOTS.values()
        )
        if not near_depot:
            out["LOW"] = f"below 10 m ({alt:.0f} m)"

    # 3) inside a no-fly zone
    for z in lf.NO_FLY_ZONES:
        if haversine_m(lat, lon, z["lat"], z["lon"]) < z["radius_m"]:
            out[f"NOFLY:{z['name']}"] = f"in no-fly zone: {z['name']}"

    # 4) outside its company's area of operations
    poly = lf.AREAS_OF_OPERATION.get(company)
    if poly and not point_in_poly(lat, lon, poly):
        out["OUT_AO"] = f"left {company} area"

    return out


def _clear_label(code):
    if code == "HIGH": return "back below ceiling"
    if code == "LOW":  return "back above 10 m"
    if code.startswith("NOFLY:"): return f"cleared no-fly: {code.split(':', 1)[1]}"
    if code == "OUT_AO": return "back inside area"
    return "cleared"


def _emit(sid, code, text, kind):
    company = lf.FLEET.get(sid, ("?", "?"))[0]
    notifications.appendleft({"t": time.time(), "sysid": sid, "company": company,
                              "kind": kind, "code": code, "text": text})


# ---------------------------------------------------------------------------
# EVALUATOR: edge-triggered — fire once on entry, once on clear
# ---------------------------------------------------------------------------
def evaluator(stop):
    global delivery_count
    prev = {}            # sysid -> {code: message} from the last pass
    prev_delivered = set()  # sysids parked at a delivery point last pass
    while not stop.is_set():
        time.sleep(EVAL_S)
        now = time.time()
        with lock:
            snap = {sid: dict(d) for sid, d in state.items()
                    if "lat" in d and "last_seen" in d and (now - d["last_seen"]) < STALE_S}

        cur = {sid: violations(d) for sid, d in snap.items()}

        # Inferred deliveries: a drone stopped near a delivery point (not a depot).
        cur_delivered = set()
        for sid, dd in snap.items():
            if dd.get("speed_ms", 99) > STOP_SPEED:
                continue
            lat, lon = dd["lat"], dd["lon"]
            near_deliv = min((haversine_m(lat, lon, p[0], p[1])
                              for p in lf.DELIVERY_POINTS), default=1e9) < DELIVERY_RADIUS_M
            near_depot = any(haversine_m(lat, lon, dep[0], dep[1]) < DEPOT_SUPPRESS_M
                             for dep in lf.DEPOTS.values())
            if near_deliv and not near_depot:
                cur_delivered.add(sid)

        with lock:
            for sid, v in cur.items():
                old = prev.get(sid, {})
                for code, msg in v.items():
                    if code not in old:
                        _emit(sid, code, msg, "enter")
                for code in old:
                    if code not in v:
                        _emit(sid, code, _clear_label(code), "clear")
            # drones that went stale/vanished: clear whatever they had
            for sid, old in prev.items():
                if sid not in cur:
                    for code in old:
                        _emit(sid, code, _clear_label(code), "clear")
            alerts_state.clear()
            alerts_state.update({sid: v for sid, v in cur.items() if v})

            # newly-arrived deliveries -> count + timestamp (edge-triggered)
            for sid in cur_delivered - prev_delivered:
                delivery_count += 1
                delivered_ts[sid] = now

        prev = cur
        prev_delivered = cur_delivered


# ---------------------------------------------------------------------------
# SNAPSHOT: build the JSON the map polls
# ---------------------------------------------------------------------------
def snapshot():
    now = time.time()
    with lock:
        drones = []
        for d in state.values():
            if "lat" not in d or "last_seen" not in d:
                continue  # heard a heartbeat/battery but no position yet
            age = now - d["last_seen"]
            item = {k: v for k, v in d.items() if k != "last_seen"}
            item["status"] = "active" if age < STALE_S else "stale"
            item["age_s"] = round(age, 1)
            item.setdefault("mode", None)
            item.setdefault("battery", None)
            a = alerts_state.get(d["sysid"], {})
            item["alerts"] = list(a.values())
            item["alert"] = bool(a)
            item["delivered_ts"] = delivered_ts.get(d["sysid"])
            # Flight-plan path: current position -> destination, and (for the
            # delivery fleet) on home to the depot.
            plan = None
            tlat, tlon = d.get("tgt_lat"), d.get("tgt_lon")
            if tlat is not None:
                pos = [d["lat"], d["lon"]]
                depot = lf.DEPOTS.get(d.get("company"))
                if d.get("kind") == "sim" and depot:
                    if haversine_m(tlat, tlon, depot[0], depot[1]) < 120:
                        plan = [pos, [depot[0], depot[1]]]
                    else:
                        plan = [pos, [tlat, tlon], [depot[0], depot[1]]]
                else:
                    plan = [pos, [tlat, tlon]]
            item["plan"] = plan
            drones.append(item)
        notif = [dict(n) for n in list(notifications)[:60]]
    drones.sort(key=lambda x: x["sysid"])

    depots = [
        {"company": c, "lat": ll[0], "lon": ll[1], "color": lf.COMPANY_COLOR[c]}
        for c, ll in lf.DEPOTS.items()
    ]
    active = sum(1 for d in drones if d["status"] == "active")
    by_company = {}
    for d in drones:
        by_company[d["company"]] = by_company.get(d["company"], 0) + 1

    areas = [
        {"company": c, "color": lf.COMPANY_COLOR[c],
         "polygon": [[la, lo] for la, lo in poly],
         "label": [lf.AO_LABEL[c]["lat"], lf.AO_LABEL[c]["lon"]],
         "label_dir": lf.AO_LABEL[c]["dir"]}
        for c, poly in lf.AREAS_OF_OPERATION.items()
    ]

    return {
        "updated": now,
        "center": {"lat": lf.MAP_CENTER[0], "lon": lf.MAP_CENTER[1]},
        "colors": lf.COMPANY_COLOR,
        "counts": {"total": len(drones), "active": active, "by_company": by_company},
        "depots": depots,
        "areas": areas,
        "nofly": lf.NO_FLY_ZONES,
        "drones": drones,
        "notifications": [
            {**n, "ago_s": round(now - n["t"], 1)} for n in notif
        ],
        "alerting": sum(1 for d in drones if d.get("alert")),
        "deliveries": delivery_count,
    }


# ---------------------------------------------------------------------------
# HTTP: serve the JSON API and the map page
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # keep the console quiet

    def _send(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/drones"):
            self._send(json.dumps(snapshot()).encode(), "application/json")
        elif self.path in ("/", "/index.html"):
            try:
                with open("utm_map.html", "rb") as f:
                    self._send(f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(
                    b"<h1>GridEye aggregator is running</h1>"
                    b"<p>Live data: <a href='/api/drones'>/api/drones</a></p>"
                    b"<p>The map page (utm_map.html) isn't in this folder yet.</p>",
                    "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self.end_headers()


def reporter(stop):
    """Print a one-line fleet status every few seconds."""
    while not stop.is_set():
        time.sleep(3.0)
        snap = snapshot()
        c = snap["counts"]
        alerting = snap["alerting"]
        extra = f"  ALERTS: {alerting}" if alerting else ""
        print(f"[GridEye] {c['active']}/{c['total']} active  {c['by_company']}{extra}")


def main():
    stop = threading.Event()
    threading.Thread(target=ingest, args=(stop,), daemon=True).start()
    threading.Thread(target=evaluator, args=(stop,), daemon=True).start()
    threading.Thread(target=reporter, args=(stop,), daemon=True).start()

    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"[GridEye] serving  http://localhost:{HTTP_PORT}/   (api at /api/drones)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        httpd.shutdown()
        print("\n[GridEye] stopped.")


if __name__ == "__main__":
    main()
