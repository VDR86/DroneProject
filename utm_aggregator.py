"""
utm_aggregator.py
=================

The read-only MONITORING backend for the Frisco UTM. It is the de-weaponized
cousin of the fleet proxy: it only LISTENS. No commands, no spoofing.

  * Listens on ONE UDP port (frisco_fleet.INGEST_PORT). Every drone - the 10
    lightweight ones and the 2 real SITL ones (fed in via MAVProxy --out) -
    arrives in this single stream, separated by MAVLink system id.
  * Keeps a live table: for each drone, its company, position, altitude, heading,
    speed, and how long since it was last heard.
  * Serves that table as JSON at  http://localhost:8000/api/drones  and serves
    the map page at  http://localhost:8000/  (once utm_map.html exists).

Run AFTER starting frisco_fleet.py (and optionally the 2 SITL drones):
    python3 utm_aggregator.py
then open http://localhost:8000/ in the browser.
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

import frisco_fleet as ff  # the scenario's single source of truth

HTTP_PORT = 8000
STALE_S = 4.0   # no telemetry for longer than this -> mark the drone "stale"

# Live state: sysid -> dict of latest values. Guarded by `lock`.
state = {}
lock = threading.Lock()


# ---------------------------------------------------------------------------
# INGEST: one listener hears the whole fleet by system id
# ---------------------------------------------------------------------------
def ingest(stop):
    conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{ff.INGEST_PORT}")
    print(f"[utm] listening for the fleet on UDP {ff.INGEST_PORT}")
    while not stop.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        sid = msg.get_srcSystem()
        mtype = msg.get_type()

        if mtype == "GLOBAL_POSITION_INT":
            company, kind = ff.FLEET.get(sid, ("Unknown", "?"))
            speed = math.hypot(msg.vx, msg.vy) / 100.0  # vx/vy cm/s -> m/s
            with lock:
                d = state.setdefault(sid, {})
                d.update(
                    sysid=sid,
                    company=company,
                    kind=kind,
                    color=ff.COMPANY_COLOR.get(company, "#888888"),
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


# ---------------------------------------------------------------------------
# SNAPSHOT: build the JSON the map polls
# ---------------------------------------------------------------------------
def snapshot():
    now = time.time()
    with lock:
        drones = []
        for d in state.values():
            age = now - d["last_seen"]
            item = {k: v for k, v in d.items() if k != "last_seen"}
            item["status"] = "active" if age < STALE_S else "stale"
            item["age_s"] = round(age, 1)
            drones.append(item)
    drones.sort(key=lambda x: x["sysid"])

    depots = [
        {"company": c, "lat": ll[0], "lon": ll[1], "color": ff.COMPANY_COLOR[c]}
        for c, ll in ff.DEPOTS.items()
    ]
    active = sum(1 for d in drones if d["status"] == "active")
    by_company = {}
    for d in drones:
        by_company[d["company"]] = by_company.get(d["company"], 0) + 1

    return {
        "updated": now,
        "center": {"lat": ff.MAP_CENTER[0], "lon": ff.MAP_CENTER[1]},
        "colors": ff.COMPANY_COLOR,
        "counts": {"total": len(drones), "active": active, "by_company": by_company},
        "depots": depots,
        "drones": drones,
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
                    b"<h1>UTM aggregator is running</h1>"
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
        print(f"[utm] {c['active']}/{c['total']} active  {c['by_company']}")


def main():
    stop = threading.Event()
    threading.Thread(target=ingest, args=(stop,), daemon=True).start()
    threading.Thread(target=reporter, args=(stop,), daemon=True).start()

    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"[utm] serving  http://localhost:{HTTP_PORT}/   (api at /api/drones)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        httpd.shutdown()
        print("\n[utm] stopped.")


if __name__ == "__main__":
    main()
