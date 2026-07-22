#!/usr/bin/env python3
"""
Frisco UTM — project overview, run tutorial, and launcher
=========================================================

A local, self-contained UAS Traffic Management (UTM) simulation for Frisco, TX.
Ten lightweight simulated delivery drones fly real Frisco geography and emit
normal MAVLink telemetry over UDP; a read-only monitoring backend tracks them,
applies flight rules, and runs a defensive (blue-team) layer that detects
spoofed telemetry and surfaces it on a live map. Everything runs over loopback —
nothing touches a real network or a real aircraft, and nothing in the project
transmits commands or produces a spoof.


WHAT EACH FILE DOES
-------------------
Core system
  frisco_fleet.py   Scenario single-source-of-truth + the simulator. Defines
                    depots, areas of operation, no-fly zones, altitude limits
                    (400 ft ceiling / 10 m floor), colours, map centre, and the
                    fleet roster. Running it launches ten lightweight drones
                    (Wing 2-6, Zipline 12-16) that fly depot->delivery->depot and
                    emit standard MAVLink over UDP. Imported by the other tools.

  utm_aggregator.py The read-only monitoring backend. Binds one raw UDP socket
                    on the ingest port (raw so it can see each datagram's source,
                    which the detector needs), parses MAVLink, keeps a live
                    per-drone table, applies the flight rules, folds in the
                    guardian's spoof alerts, and serves JSON at /api/drones and
                    the map at / on port 8000. Listens only; never commands.

  utm_map.html      The dashboard. A NOMINAL / ADVISORY / SECURITY condition
                    drives the look; a Security panel lists spoofed aircraft and
                    which detectors fired; spoofed drones get a magenta
                    target-lock, flight-rule alerts get an amber ping.

Defensive layer
  utm_guardian.py   The spoof detector. Decoupled from pymavlink and the fleet
                    so it can be unit-tested with plain Python values. Flags six
                    things: GHOST_ID (unknown id), SRC_CONFLICT (one id, two
                    sources), TELEPORT (impossible jump), CLIMB_RATE (impossible
                    climb), VEL_MISMATCH (reported speed vs flown track), and
                    FLEET_GUIDED (coordinated switch to GUIDED — takeover).
                    Returns alert strings only; transmits nothing.

Testing and diagnostics
  test_guardian.py  In-process unit tests for the detector. No network.
  ui_selftest.py    Serves the real map with a scripted canned feed so you can
                    watch every visual state; fires a staged takeover on demand.
                    Browser-only fake JSON — no MAVLink, no drones, no injection.
  ingest_tap.py     Passive read-only listener on the ingest port. Answers "does
                    an attack's telemetry actually reach the monitor?" Run it
                    INSTEAD of the aggregator (they share the port).


SETUP (once)
------------
Needs Python 3 and pymavlink. Keep every file in ONE folder.

    python3 -m venv venv
    source venv/bin/activate        # Windows: venv\\Scripts\\activate
    pip install pymavlink


THE MAIN RUN (the normal system)
--------------------------------
Two terminals, same folder:

    Terminal 1:  python3 run.py fleet       (or: python3 frisco_fleet.py)
    Terminal 2:  python3 run.py monitor     (or: python3 utm_aggregator.py)

Then open http://localhost:8000/ in a browser. Do NOT open utm_map.html as a
file:// path — the aggregator has to serve it over HTTP. A clean run shows ten
moving drones, condition NOMINAL, and an empty Security panel. Raw API is at
http://localhost:8000/api/drones. Ctrl+C in each terminal to stop.

(Optional) The two real ArduPilot SITL drones launch separately and feed the
same ingest port via MAVProxy --out; see the RUN NOTES in frisco_fleet.py.


TESTING THE DETECTOR — THREE LEVELS
-----------------------------------
1. Logic (no network):   python3 run.py test        # expect 8/8 passed
2. Dashboard (visuals):  stop the aggregator, then   python3 run.py selftest
   Open http://localhost:8000/. It starts NOMINAL; drive it from the terminal:
       Enter = toggle takeover   a = attack   c = clean   auto = timed   q = quit
3. End to end:           run the normal system, then your own red-team step, and
   watch the Security panel react to live data.


COVERAGE CHECK WITH THE TAP
---------------------------
The detector only catches what reaches the ingest port. To verify an attack's
traffic gets there, run the tap INSTEAD of the aggregator (shared port):

    stop utm_aggregator.py, then:  python3 run.py tap
    then start the fleet and/or your red-team run

Watch for a NEW source address under a system id that already exists — that's
injection on the wire. If the attack never appears in the tap, the monitor
legitimately can't see it and you've found a coverage gap on that path.


PORTS
-----
UDP 14570 = ingest (fleet -> aggregator/tap).  HTTP 8000 = dashboard.
Only one process can hold each at a time, so the aggregator, ui_selftest.py, and
ingest_tap.py are mutually exclusive — stop one before starting another.


TROUBLESHOOTING
---------------
- "cannot bind UDP 14570" / port in use: another aggregator or the tap already
  holds the ingest port. Stop it and retry.
- Map won't load: open http://localhost:8000/, not the HTML file directly, and
  keep utm_map.html in the same folder as the server that's running.
- ModuleNotFoundError: pymavlink: activate the venv and pip install pymavlink.
- No drones on the map: confirm the fleet is running in its own terminal.
- Security panel never lights up in the self-test: press Enter (or a); it starts
  NOMINAL by design.
"""

