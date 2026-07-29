"""
ui_selftest.py
==============

A DISPLAY fixture for the GridEye dashboard (map.html).

What it is
----------
A tiny web server that serves your real map page and, at /api/drones, a
*scripted, hand-made* snapshot in exactly the shape the real aggregator emits.
The demo fleet is the full 10 aircraft (Wing 2-6, Zipline 12-16), matching the
real system's roster 1:1. It starts in NOMINAL and lets you fire any of
several staged scenarios on demand, driven entirely from the terminal this
process is running in -- there is no on-page control of any kind, so nobody
who can merely load the dashboard in a browser can change what it's showing:


  In the terminal (the only place this is controlled from):
    Enter        toggle the current scenario's takeover on/off
    a            force attack ON (current scenario)
    c            force clean (NOMINAL)
    auto         resume the old timed loop (clean for ~10s, then attack, repeat)
    1            select scenario: fleet hijack (coordinated GUIDED + jumps)
    2            select scenario: ghost aircraft / rogue node (unregistered sysid)
    3            select scenario: telemetry falsification (climb rate or speed)
    4            select scenario: gradual escalation (stages in over ~12s)
    5            select scenario: GPS spoofing (false reported position)
    6            select scenario: combined attack (hijack hits all flight rules)
    q            quit

What it is NOT
--------------
This is a front-end test only. It serves canned JSON to your browser. It does
not speak MAVLink, does not connect to or command any drone, and does not send
anything to the real aggregator. It cannot attack anything — it only feeds the
webpage a fixed response so you can confirm the display works. Importing the
two offset constants from fleet_attack.py is the only thing this file takes
from it; it never imports or runs any of fleet_attack.py's networking code.
Likewise, it borrows lightweight_fleet.py's depots/delivery-points/speed constants
to compute positions, but never opens lightweight_fleet.py's MAVLink connection —
every non-attacked drone flies the exact same depot->delivery->depot route the
real fleet would (same RNG seed per sysid, so the same delivery picks), while
a spoofed or hijacked drone in whichever scenario is running breaks from that
route at an elevated multiple of cruise speed (converging fast on a shared
rally point for a GUIDED takeover, or running away indefinitely for a
TELEPORT) — so "on the real path, cruising normally" vs. "off it, and moving
in an obviously wrong way" is visible on the map itself, not just called out
in the alert text.

Run
---
Stop the real aggregator first (this uses the same port 8000), then:

    py ui_selftest.py

...and open  http://localhost:8000/  in the browser to watch it, then switch
back to this terminal and press 1-4 to pick a scenario, Enter (or a) to fire
it and watch the console react, c to go back to NOMINAL.

Requires lightweight_fleet.py AND fleet_attack.py in the same folder (geography and
the scatter-offset constants — neither is actually connected to).
"""

import json
import math
import os
import random
import threading
import time
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import lightweight_fleet as lf  # reuse the real geography so the map matches

# Reuse fleet_attack.py's own numbers for the staged "jump": that tool fans a
# hijacked fleet out from its real position by this exact (lat, lon) offset in
# degrees when it fires (see ATTACK_LAT_OFFSET_DEG/ATTACK_LON_OFFSET_DEG and
# scatter_fleet() there). Importing the constants means this display fixture's
# fake attack always matches whatever the real tool actually does, instead of
# being a made-up number that could quietly drift out of sync.
from fleet_attack import ATTACK_LAT_OFFSET_DEG, ATTACK_LON_OFFSET_DEG

HTTP_PORT = 8000
TAKEOVER_AT = 10.0   # in "auto" mode: seconds into the loop before the attack

_GRADUAL_STAGES = (0.0, 4.0, 8.0, 12.0)
_GRADUAL_VIEW_BUFFER_S = 4.0   # extra time to actually see the finale before auto cycles back
LOOP_S = TAKEOVER_AT + _GRADUAL_STAGES[-1] + _GRADUAL_VIEW_BUFFER_S  # in "auto" mode: scene length before it repeats

# Find the map (and the off-course compare page) next to this script,
# regardless of the launch directory.
_HERE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(_HERE, "map.html")
# The recorded ATTACKED pathway per sysid: only the positions reported WHILE
# that drone was flagged, starting fresh at the beginning of each incident
# and persisting (frozen) after the alert clears — same behaviour as the
# real aggregator's attack_tracks.
ATTACK_TRACKS = defaultdict(list)
_PREV_ATTACKED_SIDS = set()

# Persistent security notification log — edge-triggered, same shape as the real
# aggregator's notifications deque. Entries are appended once when an anomaly
# first fires and persist (with their original timestamp) after the attack clears
# so the left-panel log doesn't vanish the instant you type 'c'.
_SEC_LOG = deque(maxlen=200)   # {t, sysid, company, kind, code, text}
_PREV_NOTES_KEY = set()        # (sysid, code) pairs active last tick

_delivery_count = 0
_delivered_ts   = {}   # sysid -> float epoch
_delivered_n    = {}   # sysid -> that drone's own completed-delivery count
_at_delivery    = {}   # sysid -> bool

DELIVERY_RADIUS_M = 120.0   # matches aggregator.DELIVERY_RADIUS_M
DEPOT_SUPPRESS_M  = 200.0   # matches aggregator.DEPOT_SUPPRESS_M

START = time.time()

CTRL = {"mode": "manual", "attack_on": False, "scenario": "fleet",
        "attack_started_at": None, "reset_token": 0}
CTRL_LOCK = threading.Lock()

DEMO = [
    (2,  "Wing",    33.150, -96.845, 80),
    (3,  "Wing",    33.145, -96.838, 80),
    (4,  "Wing",    33.158, -96.850, 80),
    (5,  "Wing",    33.152, -96.832, 80),
    (6,  "Wing",    33.140, -96.855, 80),
    (12, "Zipline", 33.120, -96.795, 110),
    (13, "Zipline", 33.132, -96.788, 110),
    (14, "Zipline", 33.112, -96.805, 110),
    (15, "Zipline", 33.126, -96.774, 110),
    (16, "Zipline", 33.108, -96.792, 110),
]

ATTACK_SPEED_MULT = 4.0     # CLIMB_RATE's horizontal component, the scatter runaway
HIJACK_SPEED_MULT = 10.0   # FLEET_GUIDED: a coordinated seizure should look far more urgent than a solo anomaly

SPEED_LIMIT_MS = 30.0

SCATTER_SYSID = 5
_scatter_dm = ATTACK_LAT_OFFSET_DEG * lf.M_PER_DEG_LAT
_scatter_dm_lon = ATTACK_LON_OFFSET_DEG * lf.meters_per_deg_lon(lf.DEPOTS["Wing"][0])
SCATTER_DIST_M = round(math.hypot(_scatter_dm, _scatter_dm_lon))
SCATTER_BEARING_DEG = lf.bearing_deg(
    lf.DEPOTS["Wing"][0], lf.DEPOTS["Wing"][1],
    lf.DEPOTS["Wing"][0] + ATTACK_LAT_OFFSET_DEG, lf.DEPOTS["Wing"][1] + ATTACK_LON_OFFSET_DEG)

RALLY_POINT = (33.157, -96.837)
HIJACK_ORBIT_RADIUS_M = 140.0
HIJACK_ORBIT_PERIOD_S = 12.0

MAP_CENTER_LAT = sum(d[2] for d in DEMO) / len(DEMO)
MAP_CENTER_LON = sum(d[3] for d in DEMO) / len(DEMO)
DEMO_BASE = {sid: (blat, blon) for sid, _co, blat, blon, _alt in DEMO}


def _point_in_polygon(lat, lon, poly):
    """Ray-casting check against a polygon specified as a list of (lat, lon) vertices."""
    if not poly:
        return True
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


def _orbit(base_lat, base_lon, phase, t):
    r = 0.004
    w = 0.15
    lat = base_lat + r * math.sin(w * t + phase)
    lon = base_lon + r * math.cos(w * t + phase) * 1.2
    hdg = (math.degrees(w * t + phase) + 90) % 360
    return round(lat, 6), round(lon, 6), round(hdg, 1)


CLEAN_PATH_STATE = {}
CLEAN_PATH_LOCK = threading.Lock()
_MAX_CATCHUP_TICKS = 2000

DEMO_TIME_MULT = 12.0


def _clean_path_step(state, tgt, speed_mult=1.0):
    d = lf.distance_m(state["lat"], state["lon"], tgt[0], tgt[1])
    state["hdg"] = lf.bearing_deg(state["lat"], state["lon"], tgt[0], tgt[1])
    step = lf.SPEED_MS * speed_mult * DEMO_TIME_MULT * lf.TICK_S
    if d <= step or d < 1.0:
        state["lat"], state["lon"] = tgt
        return True
    frac = step / d
    state["lat"] += (tgt[0] - state["lat"]) * frac
    state["lon"] += (tgt[1] - state["lon"]) * frac
    return False


def _clean_path_tick(state, company, t, speed_mult=1.0):
    cruise = lf.CRUISE_ALT_M[company]
    if state["phase"] in ("to_delivery", "to_depot"):
        if state["rel_alt"] < cruise:
            state["rel_alt"] = min(cruise, state["rel_alt"] + lf.CLIMB_RATE * DEMO_TIME_MULT * lf.TICK_S)
        leg_tgt = state["route"][state["leg_idx"]]
        if _clean_path_step(state, leg_tgt, speed_mult):
            if state["leg_idx"] < len(state["route"]) - 1:
                state["leg_idx"] += 1
            else:
                state["hover_until"] = t + lf.HOVER_S
                state["phase"] = "drop" if state["phase"] == "to_delivery" else "rest"
    elif state["phase"] == "drop":
        if t >= state["hover_until"]:
            state["phase"] = "to_depot"
            state["route"] = lf.return_route(state["route_idx"], lf.DEPOTS[company])
            state["leg_idx"] = 0
    elif state["phase"] == "rest":
        if t >= state["hover_until"]:
            state["route_idx"] = lf.pick_route_idx(state["rng"])
            state["target"] = lf.delivery_point(state["route_idx"])
            state["route"] = lf.outbound_route(state["route_idx"])
            state["leg_idx"] = 0
            state["phase"] = "to_delivery"


def _breakaway_pos(state, now, speed_mult, target_point=None, bearing_deg_fixed=None,
                    hold_s=0.0, escape_bearing_deg=None):
    if state.get("break_since") is None:
        state["break_since"] = now
        state["break_from"] = (state["lat"], state["lon"])
    elapsed = now - state["break_since"]
    dist = speed_mult * lf.SPEED_MS * elapsed
    from_lat, from_lon = state["break_from"]

    if target_point is not None:
        total = lf.distance_m(from_lat, from_lon, target_point[0], target_point[1])
        travel_time = (total / (speed_mult * lf.SPEED_MS)) if speed_mult > 0 else 0.0
        if dist >= total:
            t_at_target = elapsed - travel_time
            if escape_bearing_deg is None:
                ang = 2 * math.pi * (t_at_target / HIJACK_ORBIT_PERIOD_S) + (state.get("sid_phase", 0.0))
                dlat = (HIJACK_ORBIT_RADIUS_M / lf.M_PER_DEG_LAT) * math.sin(ang)
                dlon = (HIJACK_ORBIT_RADIUS_M / lf.meters_per_deg_lon(target_point[0])) * math.cos(ang)
                lat, lon = target_point[0] + dlat, target_point[1] + dlon
                hdg = (math.degrees(ang) + 90) % 360
            elif t_at_target < hold_s:
                lat, lon = target_point
                hdg = lf.bearing_deg(from_lat, from_lon, target_point[0], target_point[1])
            else:
                esc_elapsed = t_at_target - hold_s
                esc_dist = speed_mult * lf.SPEED_MS * esc_elapsed
                th = math.radians(escape_bearing_deg)
                dlat = (esc_dist * math.cos(th)) / lf.M_PER_DEG_LAT
                dlon = (esc_dist * math.sin(th)) / lf.meters_per_deg_lon(target_point[0])
                lat, lon = target_point[0] + dlat, target_point[1] + dlon
                hdg = escape_bearing_deg
        else:
            frac = (dist / total) if total > 0 else 1.0
            lat = from_lat + (target_point[0] - from_lat) * frac
            lon = from_lon + (target_point[1] - from_lon) * frac
            hdg = lf.bearing_deg(from_lat, from_lon, target_point[0], target_point[1])
    else:
        th = math.radians(bearing_deg_fixed)
        dlat = (dist * math.cos(th)) / lf.M_PER_DEG_LAT
        dlon = (dist * math.sin(th)) / lf.meters_per_deg_lon(from_lat)
        lat, lon = from_lat + dlat, from_lon + dlon
        hdg = bearing_deg_fixed

    return round(lat, 6), round(lon, 6), round(hdg, 1)


def _clean_path_pos(sid, company, now, hijacked, speed_mult=1.0,
                     breakaway_bearing_deg=None, breach_target=None,
                     breach_hold_s=0.0, escape_bearing_deg=None):
    with CLEAN_PATH_LOCK:
        state = CLEAN_PATH_STATE.get(sid)
        if state is None:
            depot = lf.DEPOTS[company]
            rng = random.Random(sid * 104729)
            route_idx = lf.pick_route_idx(rng)
            route = lf.outbound_route(route_idx)
            state = {
                "lat": depot[0], "lon": depot[1], "rel_alt": 0.0,
                "hdg": lf.bearing_deg(depot[0], depot[1], route[0][0], route[0][1]),
                "phase": "to_delivery", "hover_until": 0.0,
                "sid": sid, "rng": rng, "route_idx": route_idx, "t": START,
                "break_since": None, "break_from": None,
                "sid_phase": (sid % 7) * 0.9,
            }
            state["target"] = lf.delivery_point(route_idx)
            state["route"] = route
            state["leg_idx"] = 0
            CLEAN_PATH_STATE[sid] = state

        breaking_away = hijacked or (breakaway_bearing_deg is not None) or (breach_target is not None)

        if not breaking_away:
            state["break_since"] = None
            steps = 0
            while state["t"] + lf.TICK_S <= now and steps < _MAX_CATCHUP_TICKS:
                state["t"] += lf.TICK_S
                _clean_path_tick(state, company, state["t"], speed_mult)
                steps += 1
            return (round(state["lat"], 6), round(state["lon"], 6),
                    round(state["hdg"], 1), state["rel_alt"])

        if hijacked:
            target, hold_s, esc_bearing = RALLY_POINT, 0.0, None
        elif breach_target is not None:
            target, hold_s, esc_bearing = breach_target, breach_hold_s, escape_bearing_deg
        else:
            target, hold_s, esc_bearing = None, 0.0, None
        lat, lon, hdg = _breakaway_pos(state, now, speed_mult,
                                        target_point=target,
                                        bearing_deg_fixed=breakaway_bearing_deg,
                                        hold_s=hold_s, escape_bearing_deg=esc_bearing)
        return lat, lon, hdg, state["rel_alt"]


_BATTERY_CYCLE_S = 240.0
_BATTERY_FLOOR_PCT = 22
_BATTERY_CEIL_PCT = 97


def _clean_telemetry(sid, company, t):
    phase = (sid * 37) % 97
    base_speed = 19.5 if company == "Zipline" else 14.0
    speed = base_speed + 2.2 * math.sin(0.07 * t + phase) + (sid % 4) * 0.5
    speed = round(max(3.0, speed), 1)

    frac = ((t + phase * 6.0) % _BATTERY_CYCLE_S) / _BATTERY_CYCLE_S
    battery = round(_BATTERY_CEIL_PCT - (_BATTERY_CEIL_PCT - _BATTERY_FLOOR_PCT) * frac)
    return speed, battery


def _scn_fleet(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    guided_msg = (f"3 drones switched to GUIDED within 0.4s and are converging "
                  f"on a single point at {HIJACK_SPEED_MULT:.0f}x cruise speed")
    for sid in (2, 3, 4):
        overrides[sid] = {"mode": "GUIDED", "attacked": True,
                           "speed_mult": HIJACK_SPEED_MULT,
                           "alerts": [("FLEET_GUIDED", guided_msg)]}
        notes.append((sid, "FLEET_GUIDED", guided_msg, 0.0))
    scatter_msg = (f"#{SCATTER_SYSID} broke off along fleet_attack.py's scatter heading "
                    f"(a {SCATTER_DIST_M} m jump from those constants alone) and is "
                    f"still accelerating away at {ATTACK_SPEED_MULT:.0f}x cruise speed")
    overrides[SCATTER_SYSID] = {"attacked": True, "speed_mult": ATTACK_SPEED_MULT,
                                 "breakaway_bearing_deg": SCATTER_BEARING_DEG,
                                 "alerts": [("TELEPORT", scatter_msg)]}
    notes.append((SCATTER_SYSID, "TELEPORT", scatter_msg, 0.0))
    return overrides, [], notes


def _scn_ghost(now, t_since, attacking):
    overrides, ghosts, notes = {}, [], []
    if not attacking:
        return overrides, ghosts, notes
    ghost_sid = 47
    glat, glon, ghdg = _orbit(33.130, -96.810, 3.0, (now - START) * ATTACK_SPEED_MULT)
    msg = f"sysid {ghost_sid} not in known fleet roster - POSSIBLE COLLISION DETECTED!"
    ghosts.append({
        "sysid": ghost_sid, "company": "UNKNOWN", "kind": "ghost",
        "color": "#ff2e7e",
        "lat": glat, "lon": glon, "alt_m": 95, "amsl_m": 95 + lf.FIELD_ALT_M,
        "hdg": ghdg, "speed_ms": round(9.0 * ATTACK_SPEED_MULT, 1), "mode": "AUTO", "battery": 0,
        "status": "active", "age_s": 0.2, "attacked": True,
        "alerts": [f"ATTACK — {msg}"], "codes": ["GHOST_ID"], "alert": True, "delivered_ts": None,
    })
    notes.append((ghost_sid, "GHOST_ID", msg, 0.0))
    return overrides, ghosts, notes


def _scn_climbvel(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    climb_msg = (f"#3 climbed 300 m in 1.0s (300 m/s > 15) while running "
                 f"{ATTACK_SPEED_MULT:.0f}x cruise speed horizontally")
    overrides[3] = {"attacked": True, "alt_offset_m": 300, "speed_mult": ATTACK_SPEED_MULT,
                     "alerts": [("CLIMB_RATE", climb_msg)]}
    notes.append((3, "CLIMB_RATE", climb_msg, 0.0))
    speed_msg_14 = (f"#14 holding {lf.SPEED_MS * ATTACK_SPEED_MULT:.0f} m/s ground speed "
                    f"({ATTACK_SPEED_MULT:.0f}x cruise, > 45 m/s) with no matching climb "
                    f"— impossible airspeed")
    overrides[14] = {"attacked": True, "speed_mult": ATTACK_SPEED_MULT,
                      "speed_ms": round(lf.SPEED_MS * ATTACK_SPEED_MULT, 1),
                      "alerts": [("CLIMB_RATE", speed_msg_14)]}
    notes.append((14, "CLIMB_RATE", speed_msg_14, 0.0))
    return overrides, [], notes


def _scn_gradual(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes

    if t_since >= _GRADUAL_STAGES[1]:
        msg = (f"#{SCATTER_SYSID} broke off along fleet_attack.py's scatter heading "
               f"(a {SCATTER_DIST_M} m jump from those constants alone) and is "
               f"still accelerating away at {ATTACK_SPEED_MULT:.0f}x cruise speed")
        overrides[SCATTER_SYSID] = {"attacked": True, "speed_mult": ATTACK_SPEED_MULT,
                                     "breakaway_bearing_deg": SCATTER_BEARING_DEG,
                                     "alerts": [("TELEPORT", msg)]}
        notes.append((SCATTER_SYSID, "TELEPORT", msg, _GRADUAL_STAGES[1]))

    if t_since >= _GRADUAL_STAGES[2]:
        msg = (f"#3 climbed 300 m in 1.0s (300 m/s > 15) while running "
               f"{ATTACK_SPEED_MULT:.0f}x cruise speed horizontally")
        overrides[3] = {"attacked": True, "alt_offset_m": 300, "speed_mult": ATTACK_SPEED_MULT,
                         "alerts": [("CLIMB_RATE", msg)]}
        notes.append((3, "CLIMB_RATE", msg, _GRADUAL_STAGES[2]))

    if t_since >= _GRADUAL_STAGES[3]:
        guided_msg = (f"3 drones switched to GUIDED within 0.4s and are converging "
                      f"on a single point at {HIJACK_SPEED_MULT:.0f}x cruise speed")
        for sid in (2, 3, 4):
            entry = overrides.get(sid, {"attacked": True, "alerts": []})
            entry["mode"] = "GUIDED"
            entry["attacked"] = True
            entry["speed_mult"] = HIJACK_SPEED_MULT
            entry["alerts"] = entry.get("alerts", []) + [("FLEET_GUIDED", guided_msg)]
            overrides[sid] = entry
            notes.append((sid, "FLEET_GUIDED", guided_msg, _GRADUAL_STAGES[3]))

    return overrides, [], notes


_GPS_ATTACK_VICTIM_SID = 6
_GPS_ATTACK_WAYPOINTS = [
    lf.DEPOTS["Wing"],          
    (33.172, -96.912),          
    (33.196, -96.905),          
    (33.210, -96.868),          
]
_GPS_ATTACK_REPORTED_HDG = 145.0
_GPS_ATTACK_REPORTED_SPEED = round(lf.SPEED_MS, 1)


def _gps_attack_true_pos(t_since):
    """Walk Wing #6's TRUE position along _GPS_ATTACK_WAYPOINTS at
    ATTACK_SPEED_MULT × cruise, continuing past the last waypoint to send
    the orange drone off the chart."""
    speed = lf.SPEED_MS * ATTACK_SPEED_MULT
    remaining = t_since * speed

    lat, lon = _GPS_ATTACK_WAYPOINTS[0]
    for i in range(1, len(_GPS_ATTACK_WAYPOINTS)):
        wlat, wlon = _GPS_ATTACK_WAYPOINTS[i]
        seg = lf.distance_m(lat, lon, wlat, wlon)
        if remaining <= seg:
            frac = remaining / seg if seg > 0 else 1.0
            out_lat = lat + (wlat - lat) * frac
            out_lon = lon + (wlon - lon) * frac
            hdg = lf.bearing_deg(lat, lon, wlat, wlon)
            return round(out_lat, 6), round(out_lon, 6), round(hdg, 1), i - 1
        remaining -= seg
        lat, lon = wlat, wlon

    last_wlat, last_wlon = _GPS_ATTACK_WAYPOINTS[-1]
    prev_wlat, prev_wlon = _GPS_ATTACK_WAYPOINTS[-2]
    hdg = lf.bearing_deg(prev_wlat, prev_wlon, last_wlat, last_wlon)
    
    extra_dist = remaining  
    th = math.radians(hdg)
    out_lat = last_wlat + (extra_dist * math.cos(th)) / lf.M_PER_DEG_LAT
    out_lon = last_wlon + (extra_dist * math.sin(th)) / lf.meters_per_deg_lon(last_wlat)
    
    return round(out_lat, 6), round(out_lon, 6), round(hdg, 1), len(_GPS_ATTACK_WAYPOINTS) - 1


def _point_in_wing_ao(lat, lon):
    poly = lf.AREAS_OF_OPERATION.get("Wing", [])
    if not poly:
        return True
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


def _point_in_poly(lat, lon, poly):
    if not poly:
        return True
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


def _flight_rule_alerts(company, lat, lon, alt_m):
    out = []
    if alt_m > lf.MAX_ALT_M:
        out.append(("HIGH", f"altitude {alt_m:.0f} m exceeds {lf.MAX_ALT_M:.0f} m ceiling"))
    else:
        depot = lf.DEPOTS.get(company)
        away = depot is None or lf.distance_m(lat, lon, depot[0], depot[1]) > 250.0
        if 1.0 < alt_m < lf.MIN_ALT_M and away:
            out.append(("LOW", f"altitude {alt_m:.0f} m below {lf.MIN_ALT_M:.0f} m floor"))

    poly = lf.AREAS_OF_OPERATION.get(company)
    if poly and not _point_in_poly(lat, lon, poly):
        out.append(("OUT_AO", f"aircraft outside {company} area of operations"))

    for z in lf.NO_FLY_ZONES:
        if lf.distance_m(lat, lon, z["lat"], z["lon"]) <= z["radius_m"]:
            out.append((f"NOFLY:{z['name']}", f'aircraft inside no-fly zone "{z["name"]}"'))

    return out


def _scn_gpsattack(now, t_since, attacking):
    overrides, ghosts, notes = {}, [], []
    if not attacking:
        return overrides, ghosts, notes

    sid = _GPS_ATTACK_VICTIM_SID
    true_lat, true_lon, true_hdg, leg = _gps_attack_true_pos(t_since)
    gap_m = int(lf.distance_m(
        true_lat, true_lon,
        lf.DEPOTS["Wing"][0], lf.DEPOTS["Wing"][1]))
    outside_ao = not _point_in_wing_ao(true_lat, true_lon)

    attack_msg = (f"#{sid} heading ({_GPS_ATTACK_REPORTED_HDG:.0f}°) is opposite to "
                 f"its position-delta vector — GPS spoofing (injected false fix)")
    vel_msg   = (f"#{sid} reports {_GPS_ATTACK_REPORTED_SPEED} m/s but the spoofed "
                 f"position delta implies near-zero ground speed")
    out_msg   = (f"#{sid} true position {gap_m} m from reported, "
                 + ("outside Wing AO — aircraft diverted" if outside_ao
                    else f"leg {leg+1}/{len(_GPS_ATTACK_WAYPOINTS)-1} of spoofed track"))

    alerts = [("GPS_ATTACK", attack_msg), ("VEL_MISMATCH", vel_msg)]
    if outside_ao:
        alerts.append(("OUT_AO", out_msg))

    overrides[sid] = {
        "attacked": True,
        "alerts": alerts,
        "speed_ms": _GPS_ATTACK_REPORTED_SPEED,
        "_reported_hdg": _GPS_ATTACK_REPORTED_HDG,
    }
    for code, msg in alerts:
        notes.append((sid, code, msg, 0.0))

    ghosts.append({
        "sysid": sid, "company": "Wing", "kind": "true_pos",
        "color": "#ff6600",
        "lat": true_lat, "lon": true_lon,
        "alt_m": lf.CRUISE_ALT_M["Wing"],
        "amsl_m": lf.CRUISE_ALT_M["Wing"] + lf.FIELD_ALT_M,
        "hdg": true_hdg,
        "speed_ms": round(lf.SPEED_MS * ATTACK_SPEED_MULT, 1),
        "mode": "AUTO", "battery": 0,
        "status": "active", "age_s": 0.2, "attacked": True,
        "alerts": [f"ATTACK — true pos, leg {leg+1}/{len(_GPS_ATTACK_WAYPOINTS)-1}"
                   + (" — OUTSIDE AO" if outside_ao else "")] + [msg for _, msg in alerts],
        "alert": True, "delivered_ts": None,
    })

    return overrides, ghosts, notes

_COMBINED_BREACHES = [
    (2,  "HIGH",   (33.150, -96.850, 300.0),
     "seized and force-climbed above the 122 m ceiling"),
    (3,  "LOW",    (33.120, -96.840, 5.0),
     "seized and pushed below the 10 m floor out on a leg"),
    (4,  "OUT_AO", (33.150, -96.700, 80.0),
     "seized and flown east out of Wing's area of operations"),
    (14, "NOFLY",  (33.1150, -96.7520, 100.0),
     "seized and flown into a restricted no-fly zone"),
]

ENVELOPE_HOLD_S = 6.0


def _scn_combined(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    for sid, _brk, breach_pos, note in _COMBINED_BREACHES:
        blat, blon, balt = breach_pos
        msg = f"#{sid} {note}"
        escape_bearing = lf.bearing_deg(MAP_CENTER_LAT, MAP_CENTER_LON, blat, blon)
        overrides[sid] = {"attacked": True, "breach_target": (blat, blon),
                           "breach_alt": balt, "hold_s": ENVELOPE_HOLD_S,
                           "escape_bearing_deg": escape_bearing,
                           "speed_mult": HIJACK_SPEED_MULT,
                           "alerts": [("FLEET_GUIDED", msg)]}
        notes.append((sid, "FLEET_GUIDED", msg, 0.0))
    return overrides, [], notes


SCENARIOS = {
    "fleet":     ("Fleet Hijack",          _scn_fleet),
    "ghost":     ("Ghost Aircraft",        _scn_ghost),
    "climbvel":  ("Telemetry Falsification", _scn_climbvel),
    "gradual":   ("Gradual Escalation",    _scn_gradual),
    "gpsattack": ("GPS Spoofing",          _scn_gpsattack),
    "combined":  ("Combined Attack",       _scn_combined),
}
SCENARIO_ORDER = ["fleet", "ghost", "climbvel", "gradual", "gpsattack", "combined"]


def _attack_state(now):
    with CTRL_LOCK:
        mode, attack_on = CTRL["mode"], CTRL["attack_on"]
        started_at = CTRL["attack_started_at"]
    if mode == "auto":
        phase = (now - START) % LOOP_S
        attacking = phase >= TAKEOVER_AT
        t_since = (phase - TAKEOVER_AT) if attacking else 0.0
    else:
        attacking = attack_on
        t_since = (now - started_at) if (attacking and started_at) else 0.0
    return attacking, t_since


def snapshot():
    global _delivery_count, _delivered_ts, _delivered_n, _at_delivery, _PREV_ATTACKED_SIDS
    global _PREV_NOTES_KEY
    now = time.time()
    attacking, t_since = _attack_state(now)
    with CTRL_LOCK:
        scenario_key = CTRL["scenario"]
    _label, scn_fn = SCENARIOS[scenario_key]
    overrides, ghosts, notes = scn_fn(now, t_since, attacking)

    drones, flagged_sysids, by_code = [], [], {}
    cur_attacked_sids = set()
    flight_alerting_sids = set()

    for i, (sid, company, _blat, _blon, _alt) in enumerate(DEMO):
        ov = overrides.get(sid)
        alerts, mode, attacked = [], "AUTO", False
        hijacked = bool(ov and ov.get("mode") == "GUIDED")
        speed_mult = (ov.get("speed_mult", 1.0) if ov else 1.0)
        breakaway_bearing = ov.get("breakaway_bearing_deg") if ov else None
        breach_target = ov.get("breach_target") if ov else None
        breach_hold_s = ov.get("hold_s", 0.0) if ov else 0.0
        escape_bearing = ov.get("escape_bearing_deg") if ov else None
        
        if ov and ov.get("attacked") and not hijacked and breach_target is None \
                and breakaway_bearing is None and sid != _GPS_ATTACK_VICTIM_SID:
            base_lat, base_lon = DEMO_BASE.get(sid, (MAP_CENTER_LAT, MAP_CENTER_LON))
            breakaway_bearing = lf.bearing_deg(MAP_CENTER_LAT, MAP_CENTER_LON,
                                                base_lat, base_lon)
        lat, lon, hdg, rel_alt = _clean_path_pos(sid, company, now, hijacked,
                                                  speed_mult, breakaway_bearing,
                                                  breach_target, breach_hold_s,
                                                  escape_bearing)
        if ov and "_reported_hdg" in ov:
            hdg = ov["_reported_hdg"]

        alt_here = rel_alt
        speed_ms, battery = _clean_telemetry(sid, company, now - START)

        if ov:
            attacked = ov.get("attacked", False)
            mode = ov.get("mode", mode)
            alt_here = rel_alt + ov.get("alt_offset_m", 0)
            if breach_target is not None:
                with CLEAN_PATH_LOCK:
                    st = CLEAN_PATH_STATE.get(sid)
                    break_from = st.get("break_from") if st else None
                    start_alt = st.get("rel_alt") if st else rel_alt
                if break_from is not None:
                    total = lf.distance_m(break_from[0], break_from[1],
                                           breach_target[0], breach_target[1])
                    frac = 1.0 if total <= 0 else min(
                        1.0, lf.distance_m(break_from[0], break_from[1], lat, lon) / total)
                    alt_here = start_alt + frac * (ov.get("breach_alt", start_alt) - start_alt)
            if "speed_ms" in ov:
                speed_ms = ov["speed_ms"]
            if "battery" in ov:
                battery = ov["battery"]
            for code, msg in ov.get("alerts", []):
                alerts.append(f"ATTACK — {msg}")
                by_code[code] = by_code.get(code, 0) + 1
            flagged_sysids.append(sid)
            cur_attacked_sids.add(sid)

        if attacked:
            if sid not in _PREV_ATTACKED_SIDS:
                ATTACK_TRACKS[sid] = []
            ATTACK_TRACKS[sid].append([round(lat, 6), round(lon, 6)])

        plan = None
        depot_ll = lf.DEPOTS[company]
        ps = None
        if not attacked:
            with CLEAN_PATH_LOCK:
                ps = CLEAN_PATH_STATE.get(sid)
            if ps is not None and ps.get("phase") in ("to_delivery", "to_depot"):
                route = ps["route"][ps["leg_idx"]:]
                plan = [[lat, lon]] + [list(p) for p in route]
            elif ps is not None:
                plan = [[lat, lon], list(depot_ll)]

        at_delivery = ps is not None and ps.get("phase") == "drop"
        was_at_delivery = _at_delivery.get(sid, False)
        if at_delivery and not was_at_delivery and not attacked:
            _delivery_count += 1
            _delivered_ts[sid] = now
            _delivered_n[sid] = _delivered_n.get(sid, 0) + 1
        _at_delivery[sid] = at_delivery

        flight_codes = []
        for code, msg in _flight_rule_alerts(company, lat, lon, alt_here):
            alerts.append(msg)
            flight_codes.append(code)
            by_code[code] = by_code.get(code, 0) + 1
        if flight_codes:
            flight_alerting_sids.add(sid)

        drones.append({
            "sysid": sid, "company": company, "kind": "sim",
            "color": lf.COMPANY_COLOR[company],
            "lat": lat, "lon": lon, "alt_m": round(alt_here, 1),
            "amsl_m": round(alt_here + lf.FIELD_ALT_M, 1),
            "hdg": hdg, "speed_ms": speed_ms, "mode": mode, "battery": battery,
            "status": "active", "age_s": 0.2, "attacked": attacked,
            "alerts": alerts, "alert": bool(alerts),
            "codes": [code for code, _ in (ov.get("alerts", []) if ov else [])],
            "flight_codes": flight_codes,
            "delivered_ts": _delivered_ts.get(sid) if not attacked else None,
            "delivered_n": _delivered_n.get(sid, 0),
            "plan": plan,
        })

    ghost_sysids = {g["sysid"] for g in ghosts}
    for g in ghosts:
        if g.get("kind") == "ghost" or g.get("kind") == "true_pos":
            gfc = []
            for code, msg in _flight_rule_alerts(
                    g.get("company"), g["lat"], g["lon"], g.get("alt_m", 0)):
                # For true_pos (orange drone), only include OUT_AO when it's actually outside the AO
                if g.get("kind") == "true_pos" and code == "OUT_AO" and not _point_in_wing_ao(g["lat"], g["lon"]):
                    continue
                g.setdefault("alerts", []).append(msg)
                gfc.append(code)
            if gfc:
                g["flight_codes"] = gfc
                g["alert"] = True
                flight_alerting_sids.add(g["sysid"])
        drones.append(g)
        flagged_sysids.append(g["sysid"])
        gsid = g["sysid"]
        cur_attacked_sids.add(gsid)
        if gsid not in _PREV_ATTACKED_SIDS:
            ATTACK_TRACKS[gsid] = []
        ATTACK_TRACKS[gsid].append([g["lat"], g["lon"]])

    _PREV_ATTACKED_SIDS = cur_attacked_sids

    log_notes = notes
    cur_notes_key = {(sid, code) for sid, code, _msg, _st in log_notes}
    for sid, code, msg, stage_t in log_notes:
        if (sid, code) not in _PREV_NOTES_KEY:
            company = lf.FLEET.get(sid, ("UNKNOWN", "?"))[0]
            _SEC_LOG.appendleft({
                "t": now, "sysid": sid, "company": company,
                "kind": "enter", "code": code, "text": msg,
            })
    for sid, code in _PREV_NOTES_KEY:
        if (sid, code) not in cur_notes_key:
            company = lf.FLEET.get(sid, ("UNKNOWN", "?"))[0]
            _SEC_LOG.appendleft({
                "t": now, "sysid": sid, "company": company,
                "kind": "clear", "code": code, "text": "cleared",
            })
    _PREV_NOTES_KEY.clear()
    _PREV_NOTES_KEY.update(cur_notes_key)

    for sid, code, _msg, _st in notes:
        if sid in ghost_sysids and code not in by_code:
            by_code[code] = by_code.get(code, 0) + 1

    depots = [{"company": c, "lat": ll[0], "lon": ll[1],
               "color": lf.COMPANY_COLOR[c]} for c, ll in lf.DEPOTS.items()]
    areas = [{"company": c, "color": lf.COMPANY_COLOR[c],
              "polygon": [[la, lo] for la, lo in poly],
              "label": [lf.AO_LABEL[c]["lat"], lf.AO_LABEL[c]["lon"]],
              "label_dir": lf.AO_LABEL[c]["dir"]}
             for c, poly in lf.AREAS_OF_OPERATION.items()]
    by_company = {}
    for d in drones:
        by_company[d["company"]] = by_company.get(d["company"], 0) + 1

    return {
        "updated": now,
        "center": {"lat": lf.MAP_CENTER[0], "lon": lf.MAP_CENTER[1]},
        "colors": lf.COMPANY_COLOR,
        "counts": {"total": len(drones),
                   "active": len(drones), "by_company": by_company},
        "depots": depots, "areas": areas, "nofly": lf.NO_FLY_ZONES,
        "drones": drones,
        "notifications": [
            {**n, "ago_s": round(now - n["t"], 1)}
            for n in list(_SEC_LOG)[:60]
        ],
        "alerting": len(flight_alerting_sids),
        "deliveries": _delivery_count,
        "security": {"flagged_sysids": sorted(set(flagged_sysids)),
                     "flagged": len(set(flagged_sysids)),
                     "active_by_code": by_code},
        "scenario": {"key": scenario_key, "label": _label,
                     "t_since_attack": round(t_since, 1)},
        "reset_token": CTRL["reset_token"],
    }


def _apply_ctrl(cmd):
    cmd = cmd.strip().lower()
    with CTRL_LOCK:
        global _SEC_LOG, _PREV_NOTES_KEY
        if cmd in ("", "t", "toggle"):
            CTRL["mode"] = "manual"
            CTRL["attack_on"] = not CTRL["attack_on"]
            CTRL["attack_started_at"] = time.time() if CTRL["attack_on"] else None
        elif cmd in ("a", "attack"):
            CTRL["mode"] = "manual"
            CTRL["attack_on"] = True
            CTRL["attack_started_at"] = time.time()
        elif cmd in ("c", "clean", "n", "nominal"):
            CTRL["mode"] = "manual"
            CTRL["attack_on"] = False
            CTRL["attack_started_at"] = None
            CTRL["reset_token"] += 1
            _SEC_LOG.clear()
            _PREV_NOTES_KEY.clear()
        elif cmd == "auto":
            CTRL["mode"] = "auto"
        elif cmd in ("1", "fleet"):
            CTRL["scenario"] = "fleet"
        elif cmd in ("2", "ghost"):
            CTRL["scenario"] = "ghost"
        elif cmd in ("3", "climbvel", "climb"):
            CTRL["scenario"] = "climbvel"
        elif cmd in ("4", "gradual"):
            CTRL["scenario"] = "gradual"
        elif cmd in ("5", "gpsattack", "gps"):
            CTRL["scenario"] = "gpsattack"
        elif cmd in ("6", "combined", "env"):
            CTRL["scenario"] = "combined"
        else:
            return False
    return True


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
        parsed = urlparse(self.path)
        if parsed.path == "/api/drones":
            self._send(json.dumps(snapshot()).encode(), "application/json")
        elif parsed.path == "/api/track":
            qs = parse_qs(parsed.query)
            try:
                sid = int((qs.get("sysid") or [""])[0])
            except ValueError:
                self._send(json.dumps({"error": "missing/invalid ?sysid="}).encode(),
                           "application/json")
                return
            data = snapshot()
            rec = next((d for d in data["drones"] if d.get("sysid") == sid), None)
            fallback_company = next((c for s, c, *_ in DEMO if s == sid), "Unknown")
            payload = {
                "sysid": sid,
                "company": rec["company"] if rec else fallback_company,
                "color": rec["color"] if rec else lf.COMPANY_COLOR.get(fallback_company, "#888888"),
                "attack_track": list(ATTACK_TRACKS.get(sid, [])),
                "lat": rec["lat"] if rec else None,
                "lon": rec["lon"] if rec else None,
                "alerts": rec["alerts"] if rec else [],
                "attacked": bool(rec and rec.get("attacked")),
            }
            self._send(json.dumps(payload).encode(), "application/json")
        elif parsed.path == "/api/tracks":
            data = snapshot()
            by_sid = {d["sysid"]: d for d in data["drones"]}
            out = []
            for sid, track in ATTACK_TRACKS.items():
                if not track:
                    continue
                rec = by_sid.get(sid)
                fallback_company = next((c for s, c, *_ in DEMO if s == sid), "Unknown")
                out.append({
                    "sysid": sid,
                    "company": rec["company"] if rec else fallback_company,
                    "color": rec["color"] if rec else lf.COMPANY_COLOR.get(fallback_company, "#888888"),
                    "attack_track": list(track),
                    "lat": rec["lat"] if rec else None,
                    "lon": rec["lon"] if rec else None,
                    "alerts": rec.get("alerts", []) if rec else [],
                    "attacked": bool(rec and rec.get("attacked")),
                })
            out.sort(key=lambda x: x["sysid"])
            self._send(json.dumps({"drones": out}).encode(), "application/json")
        elif parsed.path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    html = f.read()
            except FileNotFoundError:
                self._send(b"<h1>put map.html in this folder</h1>",
                           "text/html; charset=utf-8")
                return
            self._send(html, "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self.end_headers()


def _state_line():
    with CTRL_LOCK:
        scenario_label = SCENARIOS[CTRL["scenario"]][0]
        if CTRL["mode"] == "auto":
            return f"mode=AUTO (timed loop)  scenario={scenario_label}"
        return (f"mode=MANUAL  attack=" +
                ("ON — SECURITY" if CTRL["attack_on"] else "off — NOMINAL") +
                f"  scenario={scenario_label}")


def keyboard(stop):
    for line in iter(input, None):
        cmd = line.strip().lower()
        if cmd in ("q", "quit", "exit"):
            stop.set(); break
        if not _apply_ctrl(cmd):
            print("[selftest] keys: Enter=toggle  a=attack  c=clean  auto=timed  "
                  "1=fleet 2=ghost 3=climb+vel 4=gradual 5=gpsattack 6=combined q=quit")
            continue
        print(f"[selftest] {_state_line()}")


def main():
    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    httpd.timeout = 0.5
    stop = threading.Event()
    threading.Thread(target=keyboard, args=(stop,), daemon=True).start()

    print(f"[selftest] display fixture on http://localhost:{HTTP_PORT}/")
    print("[selftest] starts NOMINAL, scenario=fleet. Open the URL above to watch; "
          "control it from THIS terminal only:")
    print("[selftest]   Enter = fire/clear the takeover     a = attack     c = clean")
    print("[selftest]   auto = timed loop     1-6 = pick scenario     q = quit")
    print(f"[selftest] scenarios: " +
          ", ".join(f"{k}={v[0]}" for k, v in SCENARIOS.items()))
    print(f"[selftest] {_state_line()}")
    try:
        while not stop.is_set():
            httpd.handle_request()
    except KeyboardInterrupt:
        pass
    print("\n[selftest] stopped.")


if __name__ == "__main__":
    main()