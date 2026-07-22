"""
ui_selftest.py
==============

A DISPLAY fixture for the Frisco UTM dashboard (utm_map.html).

What it is
----------
A tiny web server that serves your real map page and, at /api/drones, a
*scripted, hand-made* snapshot in exactly the shape the real aggregator emits.
It starts in NOMINAL and lets you fire the staged "takeover" on demand from the
keyboard, so you can trigger it exactly when you're looking at the browser:

    Enter   toggle the takeover on/off
    a       force attack (SECURITY)
    c       force clean  (NOMINAL)
    auto    resume the old timed loop (clean for ~10 s, then attack, repeat)
    q       quit

When the takeover is on, three drones flip to GUIDED together (FLEET_GUIDED)
and one more trips SRC_CONFLICT + TELEPORT, so the magenta condition, the
Security panel, the code chips, and the map's target-lock markers all light up.

What it is NOT
--------------
This is a front-end test only. It serves canned JSON to your browser. It does
not speak MAVLink, does not connect to or command any drone, and does not send
anything to the real aggregator. It cannot spoof anything — it only feeds the
webpage a fixed response so you can confirm the display works.

Run
---
Stop the real aggregator first (this uses the same port 8000), then:

    py ui_selftest.py

...and open  http://localhost:8000/  in the browser. Press Enter in the
terminal to fire the takeover and watch the console jump to SECURITY.

Requires frisco_fleet.py in the same folder (only for map geography/colours).
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import frisco_fleet as ff  # reuse the real geography so the map matches

HTTP_PORT = 8000
TAKEOVER_AT = 10.0   # in "auto" mode: seconds into the loop before the attack
LOOP_S = 20.0        # in "auto" mode: scene length before it repeats

# Find the map next to this script, regardless of the launch directory.
HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "utm_map.html")

START = time.time()

# Scene control, driven by the keyboard thread.
#   "manual"  -> hold whatever `attack_on` says (default; fire on demand)
#   "auto"    -> the old timed loop (clean for TAKEOVER_AT, then attack)
CTRL = {"mode": "manual", "attack_on": False}
CTRL_LOCK = threading.Lock()

# Six demo aircraft: (sysid, company, base lat, base lon, cruise alt).
DEMO = [
    (2,  "Wing",    33.150, -96.845, 80),
    (3,  "Wing",    33.145, -96.838, 80),
    (4,  "Wing",    33.158, -96.850, 80),
    (12, "Zipline", 33.120, -96.795, 110),
    (13, "Zipline", 33.132, -96.788, 110),
    (14, "Zipline", 33.112, -96.805, 110),
]

# The staged attack: which drones are flagged, and with what.
# Each entry -> (code, message shown on the drone / in the panel).
TAKEOVER = {
    2:  ("FLEET_GUIDED", "3 drones switched to GUIDED within 0.4s (coordinated takeover?)"),
    3:  ("FLEET_GUIDED", "3 drones switched to GUIDED within 0.4s (coordinated takeover?)"),
    4:  ("FLEET_GUIDED", "3 drones switched to GUIDED within 0.4s (coordinated takeover?)"),
    12: ("SRC_CONFLICT", "telemetry for #12 from 2 sources at once"),
}
EXTRA_12 = ("TELEPORT", "#12 jumped 1100 m in 0.25s (4400 m/s > 45)")


def _orbit(base_lat, base_lon, phase, t):
    """Gentle circular drift so drones move and lay trails."""
    r = 0.004
    w = 0.15
    lat = base_lat + r * math.sin(w * t + phase)
    lon = base_lon + r * math.cos(w * t + phase) * 1.2
    hdg = (math.degrees(w * t + phase) + 90) % 360
    return round(lat, 6), round(lon, 6), round(hdg, 1)


def snapshot():
    now = time.time()
    with CTRL_LOCK:
        mode, attack_on = CTRL["mode"], CTRL["attack_on"]
    if mode == "auto":
        attacking = (now - START) % LOOP_S >= TAKEOVER_AT
    else:                       # manual: fire on demand
        attacking = attack_on

    drones, flagged_sysids, by_code = [], [], {}
    notifications = []

    for i, (sid, company, blat, blon, alt) in enumerate(DEMO):
        lat, lon, hdg = _orbit(blat, blon, i * 1.1, now - START)
        alerts, mode, spoofed = [], "AUTO", False

        if attacking and sid in TAKEOVER:
            spoofed = True
            code, msg = TAKEOVER[sid]
            alerts.append(f"SPOOF — {msg}")
            by_code[code] = by_code.get(code, 0) + 1
            if code == "FLEET_GUIDED":
                mode = "GUIDED"
            if sid == 12:  # give one drone a second detector, for variety
                ecode, emsg = EXTRA_12
                alerts.append(f"SPOOF — {emsg}")
                by_code[ecode] = by_code.get(ecode, 0) + 1
            flagged_sysids.append(sid)

        drones.append({
            "sysid": sid, "company": company, "kind": "sim",
            "color": ff.COMPANY_COLOR[company],
            "lat": lat, "lon": lon, "alt_m": alt, "amsl_m": alt + 215,
            "hdg": hdg, "speed_ms": 12.0, "mode": mode, "battery": 88,
            "status": "active", "age_s": 0.2,
            "alerts": alerts, "alert": bool(alerts), "delivered_ts": None,
        })

    if attacking:
        for sid in sorted(flagged_sysids):
            code, msg = TAKEOVER[sid]
            notifications.append({"t": START + TAKEOVER_AT, "sysid": sid,
                                  "company": "", "kind": "enter",
                                  "code": code, "text": msg})
        notifications.append({"t": START + TAKEOVER_AT, "sysid": 12,
                              "company": "", "kind": "enter",
                              "code": EXTRA_12[0], "text": EXTRA_12[1]})

    depots = [{"company": c, "lat": ll[0], "lon": ll[1],
               "color": ff.COMPANY_COLOR[c]} for c, ll in ff.DEPOTS.items()]
    areas = [{"company": c, "color": ff.COMPANY_COLOR[c],
              "polygon": [[la, lo] for la, lo in poly],
              "label": [ff.AO_LABEL[c]["lat"], ff.AO_LABEL[c]["lon"]],
              "label_dir": ff.AO_LABEL[c]["dir"]}
             for c, poly in ff.AREAS_OF_OPERATION.items()]
    by_company = {}
    for d in drones:
        by_company[d["company"]] = by_company.get(d["company"], 0) + 1

    return {
        "updated": now,
        "center": {"lat": ff.MAP_CENTER[0], "lon": ff.MAP_CENTER[1]},
        "colors": ff.COMPANY_COLOR,
        "counts": {"total": len(drones),
                   "active": len(drones), "by_company": by_company},
        "depots": depots, "areas": areas, "nofly": ff.NO_FLY_ZONES,
        "drones": drones,
        "notifications": [{**n, "ago_s": round(now - n["t"], 1)}
                          for n in notifications],
        "alerting": 0,   # this fixture only demonstrates security alerts
        "deliveries": 3,
        "security": {"flagged_sysids": sorted(flagged_sysids),
                     "flagged": len(flagged_sysids),
                     "active_by_code": by_code},
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/drones"):
            self._send(json.dumps(snapshot()).encode(), "application/json")
        elif self.path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    self._send(f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(b"<h1>put utm_map.html in this folder</h1>",
                           "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self.end_headers()


def _state_line():
    with CTRL_LOCK:
        if CTRL["mode"] == "auto":
            return "mode=AUTO (timed loop)"
        return "mode=MANUAL  attack=" + ("ON — SECURITY" if CTRL["attack_on"]
                                         else "off — NOMINAL")


def keyboard(stop):
    """Read stdin lines and drive the scene. Enter toggles the takeover."""
    for line in iter(input, None):     # blocks per line; ends on EOF/Ctrl+C
        cmd = line.strip().lower()
        with CTRL_LOCK:
            if cmd in ("", "t", "toggle"):
                CTRL["mode"] = "manual"
                CTRL["attack_on"] = not CTRL["attack_on"]
            elif cmd in ("a", "attack"):
                CTRL["mode"] = "manual"; CTRL["attack_on"] = True
            elif cmd in ("c", "clean", "n", "nominal"):
                CTRL["mode"] = "manual"; CTRL["attack_on"] = False
            elif cmd == "auto":
                CTRL["mode"] = "auto"
            elif cmd in ("q", "quit", "exit"):
                stop.set(); break
            else:
                print("[selftest] keys: Enter=toggle  a=attack  c=clean  "
                      "auto=timed  q=quit")
                continue
        print(f"[selftest] {_state_line()}")


def main():
    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    httpd.timeout = 0.5   # so the loop checks the stop flag even when idle
    stop = threading.Event()
    threading.Thread(target=keyboard, args=(stop,), daemon=True).start()

    print(f"[selftest] display fixture on http://localhost:{HTTP_PORT}/")
    print("[selftest] starts NOMINAL. Controls (type in this window):")
    print("[selftest]   Enter = fire/clear the takeover     a = attack")
    print("[selftest]   c = clean     auto = timed loop     q = quit")
    print(f"[selftest] {_state_line()}")
    try:
        while not stop.is_set():
            httpd.handle_request()
    except KeyboardInterrupt:
        pass
    print("\n[selftest] stopped.")


if __name__ == "__main__":
    main()
