"""
ui_selftest.py
==============

A DISPLAY fixture for the Frisco UTM dashboard (utm_map.html).

What it is
----------
A tiny web server that serves your real map page and, at /api/drones, a
*scripted, hand-made* snapshot in exactly the shape the real aggregator emits.
The demo fleet is the full 10 aircraft (Wing 2-6, Zipline 12-16), matching the
real system's roster 1:1. It starts in NOMINAL and lets you fire any of
several staged scenarios on demand, two ways:

  In the browser:  a small control bar, bottom-left — CLEAN / AUTO, plus one
                    button per scenario (FLEET / GHOST / CLIMB+VEL / GRADUAL).
  In the terminal:
    Enter        toggle the current scenario's takeover on/off
    a            force attack ON (current scenario)
    c            force clean (NOMINAL)
    auto         resume the old timed loop (clean for ~10s, then attack, repeat)
    1            select scenario: fleet takeover (coordinated GUIDED + jumps)
    2            select scenario: ghost aircraft (unregistered sysid)
    3            select scenario: climb-rate + velocity-mismatch spoof
    4            select scenario: gradual escalation (stages in over ~12s)
    q            quit
Both control paths call the same functions, so they can't drift out of sync —
flip it from the browser and the terminal's next status line agrees, and vice
versa.

SCENARIOS
---------
  fleet     Three drones (Wing #2/#3/#4) flip to GUIDED together and converge
            fast on a shared rally point, then orbit it once they arrive ->
            FLEET_GUIDED. Zipline #12 reports from two sources at once, the
            second one jittering independently near a point 1100 m away ->
            SRC_CONFLICT + TELEPORT. Wing #5 breaks off and runs away
            indefinitely along the same heading fleet_spoof.py's own
            ATTACK_LAT_OFFSET_DEG / ATTACK_LON_OFFSET_DEG vector points ->
            TELEPORT (so this fixture's escape direction always traces back
            to the real red-team tool's constants).

  ghost     An aircraft with a sysid that isn't in the fleet roster at all
            starts transmitting, circling fast and erratically -> GHOST_ID.
            Nothing else moves; this is the "one unexpected blip" case
            rather than a fleet-wide event.

  climbvel  Wing #3 climbs 300 m in one second while also running several
            times cruise speed horizontally -> CLIMB_RATE. Zipline #14
            visibly races ahead of the fleet while reporting a speed to
            match -> VEL_MISMATCH. Two independent single-drone detectors
            firing at once, neither involving a coordinated takeover.

  gradual   The slow-build version: nothing at t=0, then anomalies stage in
            one at a time over ~12 seconds (SRC_CONFLICT, then a TELEPORT
            runaway, then a CLIMB_RATE + speed spike, then a full 3-drone
            FLEET_GUIDED convergence), so you can watch the dashboard's
            condition climb from NOMINAL to ADVISORY to SECURITY instead of
            snapping straight to red.

What it is NOT
--------------
This is a front-end test only. It serves canned JSON to your browser. It does
not speak MAVLink, does not connect to or command any drone, and does not send
anything to the real aggregator. It cannot spoof anything — it only feeds the
webpage a fixed response so you can confirm the display works. Importing the
two offset constants from fleet_spoof.py is the only thing this file takes
from it; it never imports or runs any of fleet_spoof.py's networking code.
Likewise, it borrows frisco_fleet.py's depots/delivery-points/speed constants
to compute positions, but never opens frisco_fleet.py's MAVLink connection —
every non-spoofed drone flies the exact same depot->delivery->depot route the
real fleet would (same RNG seed per sysid, so the same delivery picks), while
a spoofed/hijacked drone in whichever scenario is running breaks from that
route at an elevated multiple of cruise speed (converging fast on a shared
rally point for a GUIDED takeover, or running away indefinitely for a
TELEPORT) — so "on the real path, cruising normally" vs. "off it, and moving
in an obviously wrong way" is visible on the map itself, not just called out
in the alert text.

Run
---
Stop the real aggregator first (this uses the same port 8000), then:

    py ui_selftest.py

...and open  http://localhost:8000/  in the browser. Click a scenario button
(or press 1-4 in the terminal) to pick a scenario, then ATTACK (or Enter) to
fire it and watch the console react; click CLEAN to go back to NOMINAL.

Requires frisco_fleet.py AND fleet_spoof.py in the same folder (geography and
the scatter-offset constants, respectively — neither is actually connected to).
"""

import json
import math
import os
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import frisco_fleet as ff  # reuse the real geography so the map matches

# Reuse fleet_spoof.py's own numbers for the staged "jump": that tool fans a
# hijacked fleet out from its real position by this exact (lat, lon) offset in
# degrees when it fires (see ATTACK_LAT_OFFSET_DEG/ATTACK_LON_OFFSET_DEG and
# scatter_fleet() there). Importing the constants means this display fixture's
# fake attack always matches whatever the real tool actually does, instead of
# being a made-up number that could quietly drift out of sync.
from fleet_spoof import ATTACK_LAT_OFFSET_DEG, ATTACK_LON_OFFSET_DEG

HTTP_PORT = 8000
TAKEOVER_AT = 10.0   # in "auto" mode: seconds into the loop before the attack

# gradual's own stage timings (seconds into an attack). Defined here, before
# LOOP_S, so LOOP_S can be DERIVED from it below rather than being a second,
# separately-maintained number -- the previous version hardcoded LOOP_S=20.0
# while gradual's last stage fired at 12s, and TAKEOVER_AT+attack-window in
# "auto" mode only ever covered ~10s, so the FLEET_GUIDED finale could never
# actually fire in auto mode. Deriving it means that can't silently regress
# again if these stage numbers ever change.
_GRADUAL_STAGES = (0.0, 4.0, 8.0, 12.0)
_GRADUAL_VIEW_BUFFER_S = 4.0   # extra time to actually see the finale before auto cycles back
LOOP_S = TAKEOVER_AT + _GRADUAL_STAGES[-1] + _GRADUAL_VIEW_BUFFER_S  # in "auto" mode: scene length before it repeats

# Find the map next to this script, regardless of the launch directory.
HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "utm_map.html")

START = time.time()

# Scene control, driven by the keyboard thread and the browser control bar.
#   mode        "manual" -> hold whatever attack_on says (fire on demand)
#               "auto"   -> the old timed loop (clean for TAKEOVER_AT, attack)
#   scenario    which staged scenario to run when attacking (see SCENARIOS)
#   attack_started_at  wall-clock time the CURRENT manual attack turned on,
#               used only by the "gradual" scenario to stage its escalation
CTRL = {"mode": "manual", "attack_on": False, "scenario": "fleet",
        "attack_started_at": None}
CTRL_LOCK = threading.Lock()

# Ten demo aircraft — Wing 2-6, Zipline 12-16 — matching the real fleet's
# roster 1:1, so "Wing 5 aircraft / Zipline 5 aircraft" looks the same here
# as it does when frisco_fleet.py + utm_aggregator.py are actually running.
# Position/altitude now come from the real depot->delivery path simulation
# below (same depots, same delivery points, same cruise altitudes as
# frisco_fleet.py) rather than from fixed coordinates here, so this list only
# needs to say which sysid belongs to which company.
# (sysid, company, _unused, _unused, _unused) -- the last three are kept as
# placeholders so the tuple shape doesn't need touching elsewhere.
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

# Every spoofed drone now moves at an elevated multiple of frisco_fleet's own
# SPEED_MS instead of quietly keeping formation -- so "hacked" reads as
# visibly urgent/wrong motion on the map, not just a marker color and some
# alert text next to an otherwise calm, orderly flight track.
SPOOF_SPEED_MULT = 4.0     # VEL_MISMATCH, CLIMB_RATE's horizontal component, the scatter runaway
HIJACK_SPEED_MULT = 10.0   # FLEET_GUIDED: a coordinated seizure should look far more urgent than a solo anomaly

# fleet scenario's 5th flagged drone: instead of a single made-up jump, it now
# breaks off and RUNS AWAY in a straight line, indefinitely, at
# SPOOF_SPEED_MULT x cruise, along the same heading fleet_spoof.py's own
# ATTACK_LAT_OFFSET_DEG/ATTACK_LON_OFFSET_DEG vector points -- so the escape
# direction still traces back to the real red-team tool's constants, even
# though the magnitude is no longer a single fixed jump.
SCATTER_SYSID = 5
_scatter_dm = ATTACK_LAT_OFFSET_DEG * ff.M_PER_DEG_LAT
_scatter_dm_lon = ATTACK_LON_OFFSET_DEG * ff.meters_per_deg_lon(ff.DEPOTS["Wing"][0])
SCATTER_DIST_M = round(math.hypot(_scatter_dm, _scatter_dm_lon))  # the jump fleet_spoof.py's constants alone would produce
SCATTER_BEARING_DEG = ff.bearing_deg(
    ff.DEPOTS["Wing"][0], ff.DEPOTS["Wing"][1],
    ff.DEPOTS["Wing"][0] + ATTACK_LAT_OFFSET_DEG, ff.DEPOTS["Wing"][1] + ATTACK_LON_OFFSET_DEG)

# A hijacked (FLEET_GUIDED) drone breaks off toward this shared point and
# then orbits it once it arrives -- Wing's own AO centroid, not the whole
# map's centre, so a Wing drone (the only company this scenario hijacks)
# doesn't have to cross an unrealistic distance to visibly converge.
RALLY_POINT = (33.157, -96.837)
HIJACK_ORBIT_RADIUS_M = 140.0
HIJACK_ORBIT_PERIOD_S = 12.0

# SRC_CONFLICT's whole premise is "two sources reporting the same aircraft at
# once" -- so instead of only flagging text, we render a second, phantom
# marker under the same sysid at a different spot. These offsets are in
# straight-north metres (dlon=0) purely so the alert text's stated distance
# is exactly what you'd measure on the map. The phantom also jitters fast
# around that offset point (see _phantom_jitter) instead of holding a
# perfectly fixed relative position, so it visibly behaves like a rogue
# signal rather than a second, well-behaved aircraft.
_TELEPORT12_DIST_M = 1100.0                                   # matches the alert text below
_TELEPORT12_OFFSET_DEG = (_TELEPORT12_DIST_M / ff.M_PER_DEG_LAT, 0.0)
_SRC_CONFLICT_PHANTOM_DIST_M = 320.0                           # smaller: no teleport claimed, just a conflicting fix
_SRC_CONFLICT_PHANTOM_OFFSET_DEG = (_SRC_CONFLICT_PHANTOM_DIST_M / ff.M_PER_DEG_LAT, 0.0)


def _orbit(base_lat, base_lon, phase, t):
    """Gentle circular drift — used ONLY for the 'ghost' scenario's intruder
    aircraft, which isn't in frisco_fleet's roster and so has no real depot
    or route to follow. Every other (clean) drone below uses the real
    path-following simulation instead."""
    r = 0.004
    w = 0.15
    lat = base_lat + r * math.sin(w * t + phase)
    lon = base_lon + r * math.cos(w * t + phase) * 1.2
    hdg = (math.degrees(w * t + phase) + 90) % 360
    return round(lat, 6), round(lon, 6), round(hdg, 1)


# ===========================================================================
# REAL PATH SIMULATION FOR CLEAN DRONES
# ---------------------------------------------------------------------------
# frisco_fleet.py's SimDrone flies depot -> delivery -> depot, picking each
# delivery target with random.Random(sysid).choice(DELIVERY_POINTS) and
# advancing at SPEED_MS/CLIMB_RATE/TICK_S. We reuse those exact constants
# and the same per-sysid seed here so a NON-spoofed demo drone traces the
# identical route the real fleet would fly -- same depot, same delivery
# points, same RNG sequence -- instead of a generic decorative circle. This
# only borrows frisco_fleet's geography/constants; it never opens a MAVLink
# connection or touches the network (see module docstring).
# ===========================================================================
CLEAN_PATH_STATE = {}       # sysid -> mutable flight-state dict
CLEAN_PATH_LOCK = threading.Lock()
_MAX_CATCHUP_TICKS = 2000   # safety cap (~500s of simulated flight per call)


def _clean_path_step(state, tgt, speed_mult=1.0):
    """One TICK_S step toward tgt; True once arrived. Mirrors
    SimDrone._step_toward exactly (position/heading only -- vx/vy/vz aren't
    needed for display). speed_mult scales how far it moves this tick, so a
    spoofed drone can visibly race across the map instead of only reporting
    a bigger number."""
    d = ff.distance_m(state["lat"], state["lon"], tgt[0], tgt[1])
    state["hdg"] = ff.bearing_deg(state["lat"], state["lon"], tgt[0], tgt[1])
    step = ff.SPEED_MS * speed_mult * ff.TICK_S
    if d <= step or d < 1.0:
        state["lat"], state["lon"] = tgt
        return True
    frac = step / d
    state["lat"] += (tgt[0] - state["lat"]) * frac
    state["lon"] += (tgt[1] - state["lon"]) * frac
    return False


def _clean_path_tick(state, company, t, speed_mult=1.0):
    """One simulated tick of the same to_delivery -> drop -> to_depot -> rest
    cycle as SimDrone.update(), minus battery (handled separately by
    _clean_telemetry) since only position/altitude/heading are displayed."""
    cruise = ff.CRUISE_ALT_M[company]
    if state["phase"] in ("to_delivery", "to_depot"):
        if state["rel_alt"] < cruise:
            state["rel_alt"] = min(cruise, state["rel_alt"] + ff.CLIMB_RATE * ff.TICK_S)
        tgt = state["target"] if state["phase"] == "to_delivery" else ff.DEPOTS[company]
        if _clean_path_step(state, tgt, speed_mult):
            state["hover_until"] = t + ff.HOVER_S
            state["phase"] = "drop" if state["phase"] == "to_delivery" else "rest"
    elif state["phase"] == "drop":
        if t >= state["hover_until"]:
            state["phase"] = "to_depot"
    elif state["phase"] == "rest":
        if t >= state["hover_until"]:
            state["target"] = state["rng"].choice(ff.DELIVERY_POINTS)
            state["phase"] = "to_delivery"


def _breakaway_pos(state, now, speed_mult, target_point=None, bearing_deg_fixed=None):
    """Move away from state['break_from'] (recorded the first tick this
    fires) at speed_mult x frisco_fleet's own cruise speed, instead of the
    old design where a hijacked/teleported drone just held a fixed spot or a
    fixed offset. Exactly one of target_point or bearing_deg_fixed is given:

    target_point       head straight for that point, then ORBIT it once
                       arrived (used for a FLEET_GUIDED convergence -- a
                       coordinated group should visibly regroup, not freeze).
    bearing_deg_fixed  fly that heading forever, no destination (used for a
                       scatter/TELEPORT runaway -- it never comes back).

    The underlying clean-path state (state['lat']/['lon']/['t']) is left
    completely untouched here, so once this drone stops breaking away the
    real route simply resumes exactly where it was paused."""
    if state.get("break_since") is None:
        state["break_since"] = now
        state["break_from"] = (state["lat"], state["lon"])
    elapsed = now - state["break_since"]
    dist = speed_mult * ff.SPEED_MS * elapsed
    from_lat, from_lon = state["break_from"]

    if target_point is not None:
        total = ff.distance_m(from_lat, from_lon, target_point[0], target_point[1])
        travel_time = (total / (speed_mult * ff.SPEED_MS)) if speed_mult > 0 else 0.0
        if dist >= total:
            orbit_t = elapsed - travel_time
            # per-sid phase so several drones orbiting the same point don't stack
            ang = 2 * math.pi * (orbit_t / HIJACK_ORBIT_PERIOD_S) + (state.get("sid_phase", 0.0))
            dlat = (HIJACK_ORBIT_RADIUS_M / ff.M_PER_DEG_LAT) * math.sin(ang)
            dlon = (HIJACK_ORBIT_RADIUS_M / ff.meters_per_deg_lon(target_point[0])) * math.cos(ang)
            lat, lon = target_point[0] + dlat, target_point[1] + dlon
            hdg = (math.degrees(ang) + 90) % 360
        else:
            frac = (dist / total) if total > 0 else 1.0
            lat = from_lat + (target_point[0] - from_lat) * frac
            lon = from_lon + (target_point[1] - from_lon) * frac
            hdg = ff.bearing_deg(from_lat, from_lon, target_point[0], target_point[1])
    else:
        th = math.radians(bearing_deg_fixed)
        dlat = (dist * math.cos(th)) / ff.M_PER_DEG_LAT
        dlon = (dist * math.sin(th)) / ff.meters_per_deg_lon(from_lat)
        lat, lon = from_lat + dlat, from_lon + dlon
        hdg = bearing_deg_fixed

    return round(lat, 6), round(lon, 6), round(hdg, 1)


def _phantom_jitter(sid, ref_lat, t):
    """Fast, tight wobble for the SRC_CONFLICT phantom marker, so it visibly
    behaves like an unstable rogue signal instead of a second, perfectly
    well-behaved aircraft holding a fixed relative position."""
    r_m = 45.0
    w = 1.3   # much faster than any clean drone's calm cruise turn rate
    ang = w * t + sid * 0.7
    dlat = (r_m / ff.M_PER_DEG_LAT) * math.sin(ang)
    dlon = (r_m / ff.meters_per_deg_lon(ref_lat)) * math.cos(ang)
    return dlat, dlon


def _clean_path_pos(sid, company, now, hijacked, speed_mult=1.0,
                     breakaway_bearing_deg=None):
    """Return (lat, lon, hdg, rel_alt) for a drone at time `now`. State
    persists across requests (module-level, like the real fleet's own
    always-on loop) so the route is continuous rather than recomputed from
    scratch each poll.

    Normally this just advances the same depot->delivery->depot route
    frisco_fleet.SimDrone would fly. Two things instead make it BREAK AWAY,
    at speed_mult x cruise:

    hijacked=True                (FLEET_GUIDED) converge on RALLY_POINT and
                                  orbit it once arrived.
    breakaway_bearing_deg=<deg>  (scatter/TELEPORT) fly that heading forever.

    Either way, the underlying route pauses (doesn't tick) while breaking
    away, and resumes from exactly where it paused once neither applies."""
    with CLEAN_PATH_LOCK:
        state = CLEAN_PATH_STATE.get(sid)
        if state is None:
            rng = random.Random(sid)   # same seed frisco_fleet.SimDrone uses
            depot = ff.DEPOTS[company]
            state = {
                "lat": depot[0], "lon": depot[1], "rel_alt": 0.0,
                "hdg": rng.uniform(0, 360),      # same draw order as SimDrone
                "phase": "to_delivery", "hover_until": 0.0,
                "rng": rng, "t": START, "break_since": None, "break_from": None,
                "sid_phase": (sid % 7) * 0.9,   # spreads out several orbiting drones
            }
            state["target"] = rng.choice(ff.DELIVERY_POINTS)  # 2nd draw, same order
            CLEAN_PATH_STATE[sid] = state

        breaking_away = hijacked or (breakaway_bearing_deg is not None)

        if not breaking_away:
            state["break_since"] = None   # clear any stale anchor for next time
            steps = 0
            while state["t"] + ff.TICK_S <= now and steps < _MAX_CATCHUP_TICKS:
                state["t"] += ff.TICK_S
                _clean_path_tick(state, company, state["t"], speed_mult)
                steps += 1
            return (round(state["lat"], 6), round(state["lon"], 6),
                    round(state["hdg"], 1), state["rel_alt"])

        target = RALLY_POINT if hijacked else None
        lat, lon, hdg = _breakaway_pos(state, now, speed_mult,
                                        target_point=target,
                                        bearing_deg_fixed=breakaway_bearing_deg)
        return lat, lon, hdg, state["rel_alt"]


# Battery drains over a full simulated charge cycle; short on purpose (a real
# fleet cycle would be tens of minutes) so you actually see numbers move
# during a demo instead of every drone reading a frozen 88%.
_BATTERY_CYCLE_S = 240.0
_BATTERY_FLOOR_PCT = 22
_BATTERY_CEIL_PCT = 97


def _clean_telemetry(sid, company, t):
    """Deterministic-but-drone-specific speed/battery for a NON-spoofed
    aircraft, so ten clean drones read as ten different real vehicles rather
    than ten copies of the same canned number. Companies fly at different
    characteristic speeds (Zipline's fixed-wing-style delivery cruise is
    faster than Wing's), each drone wobbles a little around its own cruise
    speed, and battery drains and "recharges" on its own offset schedule."""
    phase = (sid * 37) % 97          # stable per-sysid offset, no two alike
    base_speed = 19.5 if company == "Zipline" else 14.0
    speed = base_speed + 2.2 * math.sin(0.07 * t + phase) + (sid % 4) * 0.5
    speed = round(max(3.0, speed), 1)

    frac = ((t + phase * 6.0) % _BATTERY_CYCLE_S) / _BATTERY_CYCLE_S
    battery = round(_BATTERY_CEIL_PCT - (_BATTERY_CEIL_PCT - _BATTERY_FLOOR_PCT) * frac)
    return speed, battery


# ===========================================================================
# SCENARIOS
# ---------------------------------------------------------------------------
# Each scenario function takes (now, t_since_attack, attacking) and returns:
#   overrides   {sysid: {"mode": str, "spoofed": bool, "alerts": [(code,msg)],
#                          "alt_offset_m": n, "speed_mult": x,
#                          "breakaway_bearing_deg": deg, "phantom_offset_deg":
#                          (dlat, dlon), "speed_ms": n, "battery": n}}
#               — only include the keys you actually want to change.
#               "mode": "GUIDED" makes it break off and converge on
#               RALLY_POINT (see _clean_path_pos); "breakaway_bearing_deg"
#               makes it run away indefinitely along that fixed heading
#               instead. Either way "speed_mult" sets how many multiples of
#               cruise speed it actually moves at while doing so.
#   ghosts      list of extra aircraft dicts to append that aren't in DEMO
#               at all (for GHOST_ID), already positioned.
#   notes       list of (sysid, code, msg, stage_t) tuples to surface in the
#               notifications feed. stage_t is seconds-into-the-attack this
#               note actually starts applying (0.0 for every scenario except
#               "gradual", which staggers its four alerts across ~12s) --
#               it's what gives each notification its own correct timestamp
#               instead of every alert reporting the attack's start time.
# Nothing here touches a network socket; it only decides what the canned
# JSON snapshot should say for this tick.
# ===========================================================================

def _scn_fleet(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    guided_msg = (f"3 drones switched to GUIDED within 0.4s and are converging "
                  f"on a single point at {HIJACK_SPEED_MULT:.0f}x cruise speed")
    for sid in (2, 3, 4):
        overrides[sid] = {"mode": "GUIDED", "spoofed": True,
                           "speed_mult": HIJACK_SPEED_MULT,
                           "alerts": [("FLEET_GUIDED", guided_msg)]}
        notes.append((sid, "FLEET_GUIDED", guided_msg, 0.0))
    src_msg = "telemetry for #12 from 2 sources at once"
    tp_msg = f"#12 jumped {_TELEPORT12_DIST_M:.0f} m in 0.25s (4400 m/s > 45)"
    overrides[12] = {"spoofed": True, "phantom_offset_deg": _TELEPORT12_OFFSET_DEG,
                      "alerts": [("SRC_CONFLICT", src_msg), ("TELEPORT", tp_msg)]}
    notes.append((12, "SRC_CONFLICT", src_msg, 0.0))
    notes.append((12, "TELEPORT", tp_msg, 0.0))
    scatter_msg = (f"#{SCATTER_SYSID} broke off along fleet_spoof.py's scatter heading "
                    f"(a {SCATTER_DIST_M} m jump from those constants alone) and is "
                    f"still accelerating away at {SPOOF_SPEED_MULT:.0f}x cruise speed")
    overrides[SCATTER_SYSID] = {"spoofed": True, "speed_mult": SPOOF_SPEED_MULT,
                                 "breakaway_bearing_deg": SCATTER_BEARING_DEG,
                                 "alerts": [("TELEPORT", scatter_msg)]}
    notes.append((SCATTER_SYSID, "TELEPORT", scatter_msg, 0.0))
    return overrides, [], notes


def _scn_ghost(now, t_since, attacking):
    overrides, ghosts, notes = {}, [], []
    if not attacking:
        return overrides, ghosts, notes
    ghost_sid = 47
    # Same _orbit() shape, just fed sped-up time -- an intruder with no real
    # depot or route should look erratic and fast, not calmly circling.
    glat, glon, ghdg = _orbit(33.130, -96.810, 3.0, (now - START) * SPOOF_SPEED_MULT)
    msg = f"sysid {ghost_sid} not in known fleet roster"
    ghosts.append({
        "sysid": ghost_sid, "company": "UNKNOWN", "kind": "ghost",
        "color": "#ff2e7e",
        "lat": glat, "lon": glon, "alt_m": 95, "amsl_m": 95 + ff.FIELD_ALT_M,
        "hdg": ghdg, "speed_ms": round(9.0 * SPOOF_SPEED_MULT, 1), "mode": "AUTO", "battery": 0,
        "status": "active", "age_s": 0.2, "spoofed": True,
        "alerts": [f"SPOOF — {msg}"], "alert": True, "delivered_ts": None,
    })
    notes.append((ghost_sid, "GHOST_ID", msg, 0.0))
    return overrides, ghosts, notes


def _scn_climbvel(now, t_since, attacking):
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    climb_msg = (f"#3 climbed 300 m in 1.0s (300 m/s > 15) while running "
                 f"{SPOOF_SPEED_MULT:.0f}x cruise speed horizontally")
    overrides[3] = {"spoofed": True, "alt_offset_m": 300, "speed_mult": SPOOF_SPEED_MULT,
                     "alerts": [("CLIMB_RATE", climb_msg)]}
    notes.append((3, "CLIMB_RATE", climb_msg, 0.0))
    vel_reported = round(ff.SPEED_MS * SPOOF_SPEED_MULT)
    vel_msg = f"#14 reports/tracks at {vel_reported} m/s — {SPOOF_SPEED_MULT:.0f}x its normal cruise speed"
    overrides[14] = {"spoofed": True, "speed_ms": float(vel_reported), "speed_mult": SPOOF_SPEED_MULT,
                      "alerts": [("VEL_MISMATCH", vel_msg)]}
    notes.append((14, "VEL_MISMATCH", vel_msg, 0.0))
    return overrides, [], notes


# (seconds into the attack, sysid it touches) -> lets the escalation reuse
# the same per-stage bodies without repeating them. Defined near the top of
# the file now, alongside LOOP_S -- see the comment there.


def _scn_gradual(now, t_since, attacking):
    """Nothing at t=0; anomalies stage in one at a time so the dashboard's
    condition visibly climbs NOMINAL -> ADVISORY -> SECURITY instead of
    snapping straight to red the instant the attack fires."""
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes

    if t_since >= _GRADUAL_STAGES[0]:
        msg = "telemetry for #12 from 2 sources at once"
        overrides[12] = {"spoofed": True,
                          "phantom_offset_deg": _SRC_CONFLICT_PHANTOM_OFFSET_DEG,
                          "alerts": [("SRC_CONFLICT", msg)]}
        notes.append((12, "SRC_CONFLICT", msg, _GRADUAL_STAGES[0]))

    if t_since >= _GRADUAL_STAGES[1]:
        msg = (f"#{SCATTER_SYSID} broke off along fleet_spoof.py's scatter heading "
               f"(a {SCATTER_DIST_M} m jump from those constants alone) and is "
               f"still accelerating away at {SPOOF_SPEED_MULT:.0f}x cruise speed")
        overrides[SCATTER_SYSID] = {"spoofed": True, "speed_mult": SPOOF_SPEED_MULT,
                                     "breakaway_bearing_deg": SCATTER_BEARING_DEG,
                                     "alerts": [("TELEPORT", msg)]}
        notes.append((SCATTER_SYSID, "TELEPORT", msg, _GRADUAL_STAGES[1]))

    if t_since >= _GRADUAL_STAGES[2]:
        msg = (f"#3 climbed 300 m in 1.0s (300 m/s > 15) while running "
               f"{SPOOF_SPEED_MULT:.0f}x cruise speed horizontally")
        overrides[3] = {"spoofed": True, "alt_offset_m": 300, "speed_mult": SPOOF_SPEED_MULT,
                         "alerts": [("CLIMB_RATE", msg)]}
        notes.append((3, "CLIMB_RATE", msg, _GRADUAL_STAGES[2]))

    if t_since >= _GRADUAL_STAGES[3]:
        guided_msg = (f"3 drones switched to GUIDED within 0.4s and are converging "
                      f"on a single point at {HIJACK_SPEED_MULT:.0f}x cruise speed")
        for sid in (2, 3, 4):
            entry = overrides.get(sid, {"spoofed": True, "alerts": []})
            entry["mode"] = "GUIDED"
            entry["spoofed"] = True
            entry["speed_mult"] = HIJACK_SPEED_MULT   # overrides any earlier stage's speed_mult (e.g. #3's climb boost)
            entry["alerts"] = entry.get("alerts", []) + [("FLEET_GUIDED", guided_msg)]
            overrides[sid] = entry
        notes.append((2, "FLEET_GUIDED", guided_msg, _GRADUAL_STAGES[3]))

    return overrides, [], notes


SCENARIOS = {
    "fleet":    ("Fleet takeover",          _scn_fleet),
    "ghost":    ("Ghost aircraft",          _scn_ghost),
    "climbvel": ("Climb + velocity spoof",  _scn_climbvel),
    "gradual":  ("Gradual escalation",      _scn_gradual),
}
SCENARIO_ORDER = ["fleet", "ghost", "climbvel", "gradual"]


def _attack_state(now):
    """Returns (attacking, t_since_attack_start) for the current CTRL."""
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
    now = time.time()
    attacking, t_since = _attack_state(now)
    with CTRL_LOCK:
        scenario_key = CTRL["scenario"]
    _label, scn_fn = SCENARIOS[scenario_key]
    overrides, ghosts, notes = scn_fn(now, t_since, attacking)

    drones, flagged_sysids, by_code = [], [], {}

    for i, (sid, company, _blat, _blon, _alt) in enumerate(DEMO):
        ov = overrides.get(sid)
        alerts, mode, spoofed = [], "AUTO", False
        # A FLEET_GUIDED drone has left its mission -- instead of freezing,
        # it now breaks off and converges fast on RALLY_POINT (see
        # _clean_path_pos / _breakaway_pos).
        hijacked = bool(ov and ov.get("mode") == "GUIDED")
        # A scenario can also make a drone visibly speed up (e.g.
        # VEL_MISMATCH, CLIMB_RATE, the hijack convergence, or a scatter
        # runaway) instead of only changing the number in its telemetry --
        # speed_mult scales how fast it actually moves.
        speed_mult = (ov.get("speed_mult", 1.0) if ov else 1.0)
        # A scatter/TELEPORT drone runs away indefinitely along this fixed
        # heading instead of holding a static offset from its real position.
        breakaway_bearing = ov.get("breakaway_bearing_deg") if ov else None
        lat, lon, hdg, rel_alt = _clean_path_pos(sid, company, now, hijacked,
                                                  speed_mult, breakaway_bearing)
        alt_here = rel_alt
        # Clean baseline: every non-spoofed drone reads its own varied
        # speed/battery instead of an identical canned number, so a spoofed
        # drone's implausible reading actually stands out against it.
        speed_ms, battery = _clean_telemetry(sid, company, now - START)

        if ov:
            spoofed = ov.get("spoofed", False)
            mode = ov.get("mode", mode)
            alt_here = rel_alt + ov.get("alt_offset_m", 0)
            # A scenario can pin speed/battery to a specific suspicious value
            # (e.g. VEL_MISMATCH's #14 claiming an elevated m/s) instead of
            # the drone's normal varied reading.
            if "speed_ms" in ov:
                speed_ms = ov["speed_ms"]
            if "battery" in ov:
                battery = ov["battery"]
            for code, msg in ov.get("alerts", []):
                alerts.append(f"SPOOF — {msg}")
                by_code[code] = by_code.get(code, 0) + 1
            flagged_sysids.append(sid)

        drones.append({
            "sysid": sid, "company": company, "kind": "sim",
            "color": ff.COMPANY_COLOR[company],
            "lat": lat, "lon": lon, "alt_m": round(alt_here, 1),
            "amsl_m": round(alt_here + ff.FIELD_ALT_M, 1),
            "hdg": hdg, "speed_ms": speed_ms, "mode": mode, "battery": battery,
            "status": "active", "age_s": 0.2, "spoofed": spoofed,
            "alerts": alerts, "alert": bool(alerts), "delivered_ts": None,
        })

        # SRC_CONFLICT's whole point is a SECOND source claiming a different
        # position for the same sysid -- so render an actual second marker,
        # not just an alert string next to a single dot. It sits near the
        # drone's real position offset by phantom_offset_deg, PLUS a fast,
        # tight jitter (see _phantom_jitter) so it visibly behaves like an
        # unstable rogue signal instead of a second well-behaved aircraft
        # holding a perfectly fixed relative position.
        if ov and "phantom_offset_deg" in ov:
            plat, plon = ov["phantom_offset_deg"]
            jlat, jlon = _phantom_jitter(sid, lat, now - START)
            drones.append({
                "sysid": sid, "company": company, "kind": "phantom_source",
                "color": ff.COMPANY_COLOR[company],
                "lat": round(lat + plat + jlat, 6), "lon": round(lon + plon + jlon, 6),
                "alt_m": round(alt_here, 1), "amsl_m": round(alt_here + ff.FIELD_ALT_M, 1),
                "hdg": hdg, "speed_ms": speed_ms, "mode": mode, "battery": battery,
                "status": "active", "age_s": 0.2, "spoofed": True,
                "alerts": [f"SPOOF — second source for #{sid} conflicts with the first"],
                "alert": True, "delivered_ts": None,
            })

    ghost_sysids = {g["sysid"] for g in ghosts}
    for g in ghosts:
        drones.append(g)
        flagged_sysids.append(g["sysid"])

    # notes is the single source of truth for both the notification feed and
    # the by_code counts (overrides' alerts were already counted above, so
    # only count a note here if it belongs to a ghost that isn't in overrides).
    # Each note's own stage_t offset (0.0 for every scenario except the
    # staggered "gradual" one) is what lets a later-firing alert show a
    # later, correct timestamp instead of all sharing the attack's start time.
    attack_start = now - t_since
    notifications = [{"t": attack_start + stage_t, "sysid": sid, "company": "",
                       "kind": "enter", "code": code, "text": msg}
                      for sid, code, msg, stage_t in notes]
    for sid, code, msg, _stage_t in notes:
        if sid in ghost_sysids:
            by_code[code] = by_code.get(code, 0) + 1

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
        "security": {"flagged_sysids": sorted(set(flagged_sysids)),
                     "flagged": len(set(flagged_sysids)),
                     "active_by_code": by_code},
        "scenario": {"key": scenario_key, "label": _label,
                     "t_since_attack": round(t_since, 1)},
    }


def _apply_ctrl(cmd):
    """Apply one command to CTRL. Shared by the terminal keyboard() loop and
    the browser's control-bar buttons, so both control paths can never drift
    out of sync with each other."""
    cmd = cmd.strip().lower()
    with CTRL_LOCK:
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
        elif parsed.path == "/api/ctrl":
            # Browser control-bar endpoint: /api/ctrl?cmd=c|a|auto|1|2|3|4
            cmd = parse_qs(parsed.query).get("cmd", [""])[0]
            ok = _apply_ctrl(cmd)
            if ok:
                print(f"[selftest] {_state_line()}  (via browser)")
            with CTRL_LOCK:
                state = dict(CTRL)
            self._send(json.dumps({"ok": ok, **state}).encode(), "application/json")
        elif parsed.path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    html = f.read()
            except FileNotFoundError:
                self._send(b"<h1>put utm_map.html in this folder</h1>",
                           "text/html; charset=utf-8")
                return
            self._send(_inject_demo_controls(html), "text/html; charset=utf-8")
        else:
            self.send_response(404)
            self.end_headers()


# A small floating control bar, injected only by THIS fixture — the real
# utm_map.html file served by utm_aggregator.py in production stays untouched.
# It just calls /api/ctrl, the same endpoint the keyboard commands use.
_DEMO_CONTROLS_HTML = b"""
<div id="demo-ctrl" style="position:fixed; left:18px; bottom:18px; z-index:2500;
  display:flex; flex-direction:column; gap:6px; align-items:flex-start;
  font:600 10px 'IBM Plex Mono',ui-monospace,monospace;">
  <div style="display:flex; gap:6px; align-items:center;
    background:linear-gradient(180deg,#0b1118,#080d13);
    border:1px solid #1b2733; border-radius:20px; padding:6px;
    box-shadow:0 10px 26px -10px rgba(0,0,0,.6);">
    <span style="color:#7d8b9a; padding:0 4px; letter-spacing:1px;">SCENARIO</span>
    <button data-cmd="1" style="cursor:pointer; border:none; border-radius:14px; padding:7px 10px;
      background:#111b25; color:#6fb3ff; font:inherit;">FLEET</button>
    <button data-cmd="2" style="cursor:pointer; border:none; border-radius:14px; padding:7px 10px;
      background:#111b25; color:#6fb3ff; font:inherit;">GHOST</button>
    <button data-cmd="3" style="cursor:pointer; border:none; border-radius:14px; padding:7px 10px;
      background:#111b25; color:#6fb3ff; font:inherit;">CLIMB+VEL</button>
    <button data-cmd="4" style="cursor:pointer; border:none; border-radius:14px; padding:7px 10px;
      background:#111b25; color:#6fb3ff; font:inherit;">GRADUAL</button>
  </div>
  <div style="display:flex; gap:6px; align-items:center;
    background:linear-gradient(180deg,#0b1118,#080d13);
    border:1px solid #1b2733; border-radius:20px; padding:6px;
    box-shadow:0 10px 26px -10px rgba(0,0,0,.6);">
    <span style="color:#7d8b9a; padding:0 4px; letter-spacing:1px;">DEMO</span>
    <button data-cmd="c" style="cursor:pointer; border:none; border-radius:14px; padding:7px 12px;
      background:#111b25; color:#46d17f; font:inherit;">CLEAN</button>
    <button data-cmd="a" style="cursor:pointer; border:none; border-radius:14px; padding:7px 12px;
      background:#111b25; color:#ff2e7e; font:inherit;">ATTACK</button>
    <button data-cmd="auto" style="cursor:pointer; border:none; border-radius:14px; padding:7px 12px;
      background:#111b25; color:#f2b134; font:inherit;">AUTO</button>
  </div>
</div>
<script>
(function(){
  document.querySelectorAll('#demo-ctrl button').forEach(function(btn){
    btn.addEventListener('click', function(){
      fetch('/api/ctrl?cmd=' + encodeURIComponent(btn.dataset.cmd)).then(function(){
        // Refresh right away instead of waiting up to 1s for the page's own
        // setInterval(poll,1000) tick -- otherwise a scenario switch or
        // ATTACK/CLEAN click can leave the PREVIOUS scenario's flags/markers
        // on screen for up to a second, which looks like a stale drone is
        // still (or wrongly) tagged. poll() is a plain top-level function in
        // the main script, so it's already on window by the time this runs.
        if (typeof window.poll === 'function') window.poll();
      });
    });
  });
})();
</script>
"""


def _inject_demo_controls(html_bytes):
    """Append the demo control bar right before </body>, or at the end if the
    page has no </body> tag for some reason."""
    marker = b"</body>"
    if marker in html_bytes:
        return html_bytes.replace(marker, _DEMO_CONTROLS_HTML + marker, 1)
    return html_bytes + _DEMO_CONTROLS_HTML


def _state_line():
    with CTRL_LOCK:
        scenario_label = SCENARIOS[CTRL["scenario"]][0]
        if CTRL["mode"] == "auto":
            return f"mode=AUTO (timed loop)  scenario={scenario_label}"
        return (f"mode=MANUAL  attack=" +
                ("ON — SECURITY" if CTRL["attack_on"] else "off — NOMINAL") +
                f"  scenario={scenario_label}")


def keyboard(stop):
    """Read stdin lines and drive the scene. Enter toggles the takeover.
    'q' quits the process; everything else goes through _apply_ctrl, the same
    function the browser's buttons call."""
    for line in iter(input, None):     # blocks per line; ends on EOF/Ctrl+C
        cmd = line.strip().lower()
        if cmd in ("q", "quit", "exit"):
            stop.set(); break
        if not _apply_ctrl(cmd):
            print("[selftest] keys: Enter=toggle  a=attack  c=clean  auto=timed  "
                  "1=fleet 2=ghost 3=climb+vel 4=gradual  q=quit")
            continue
        print(f"[selftest] {_state_line()}")


def main():
    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    httpd.timeout = 0.5   # so the loop checks the stop flag even when idle
    stop = threading.Event()
    threading.Thread(target=keyboard, args=(stop,), daemon=True).start()

    print(f"[selftest] display fixture on http://localhost:{HTTP_PORT}/")
    print("[selftest] starts NOMINAL, scenario=fleet. Controls: buttons in the "
          "browser, or type here:")
    print("[selftest]   Enter = fire/clear the takeover     a = attack     c = clean")
    print("[selftest]   auto = timed loop     1-4 = pick scenario     q = quit")
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