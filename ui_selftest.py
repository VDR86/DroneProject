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
several staged scenarios on demand, driven entirely from the terminal this
process is running in -- there is no on-page control of any kind, so nobody
who can merely load the dashboard in a browser can change what it's showing:

It also demonstrates the spoof detail tab (utm_compare.html): a flagged
(spoofed) drone gets an "off-course ↗" link — and utm_map.html auto-opens it
— so you can watch its reported track plotted at /compare?sysid=N.

  In the terminal (the only place this is controlled from):
    Enter        toggle the current scenario's takeover on/off
    a            force attack ON (current scenario)
    c            force clean (NOMINAL)
    auto         resume the old timed loop (clean for ~10s, then attack, repeat)
    1            select scenario: fleet takeover (coordinated GUIDED + jumps)
    2            select scenario: ghost aircraft (unregistered sysid)
    3            select scenario: climb-rate + velocity-mismatch spoof
    4            select scenario: gradual escalation (stages in over ~12s)
    5            select scenario: GPS position spoof (false reported position)
    6            select scenario: envelope breach (hijack hits all flight rules)
    q            quit

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

  gpsspoof  Wing #6 is fed a false GPS signal: its REPORTED position follows
            a normal-looking depot->delivery path (right altitude, right speed,
            heading SE toward a delivery, well inside the AO). Its TRUE autopilot
            follows a hard-coded attacker route: NW from the depot (WP1), north
            past the Medical Helipad boundary (WP2), then east to a capture point
            outside Wing's AO (WP3). A second orange marker tracks the true
            position through each waypoint in real time. Three alert codes fire:
            GPS_SPOOF (heading vs. position-delta mismatch), VEL_MISMATCH
            (reported speed vs. near-zero frame delta on the spoofed track), and
            OUT_AO (true position exits Wing's AO after WP2). All three are also
            derivable by utm_guardian.py from real telemetry.

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

...and open  http://localhost:8000/  in the browser to watch it, then switch
back to this terminal and press 1-4 to pick a scenario, Enter (or a) to fire
it and watch the console react, c to go back to NOMINAL.

Requires frisco_fleet.py AND fleet_spoof.py in the same folder (geography and
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

# Find the map (and the off-course compare page) next to this script,
# regardless of the launch directory.
_HERE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(_HERE, "utm_map.html")
COMPARE_HTML_PATH = os.path.join(_HERE, "utm_compare.html")

# The recorded SPOOFED pathway per sysid: only the positions reported WHILE
# that drone was flagged, starting fresh at the beginning of each incident
# and persisting (frozen) after the alert clears — same behaviour as the
# real aggregator's spoof_tracks.
SPOOF_TRACKS = defaultdict(list)
_PREV_SPOOFED_SIDS = set()

# Persistent security notification log — edge-triggered, same shape as the real
# aggregator's notifications deque. Entries are appended once when an anomaly
# first fires and persist (with their original timestamp) after the attack clears
# so the left-panel log doesn't vanish the instant you type 'c'.
_SEC_LOG = deque(maxlen=200)   # {t, sysid, company, kind, code, text}
_PREV_NOTES_KEY = set()        # (sysid, code) pairs active last tick

# Delivery tracking for non-spoofed drones. Detection mirrors the REAL
# aggregator's own rule (utm_aggregator.evaluator): "parked" near a delivery
# point and NOT near a depot, using the drone's actual reported position —
# not an internal phase flag — so tracking is computed fresh every tick and
# can't get stuck partway through an attack.
# _delivery_count  total completed deliveries since startup.
# _delivered_ts    sysid -> epoch of that drone's most recent delivery arrival.
# _at_delivery     sysid -> bool, whether it was "parked at a delivery point"
#                  on the previous snapshot tick (for edge-triggering).
_delivery_count = 0
_delivered_ts   = {}   # sysid -> float epoch
_at_delivery    = {}   # sysid -> bool

# Same distance thresholds utm_aggregator.py uses to infer a completed
# delivery from telemetry alone, so this fixture's "✓ Delivered" behaves
# exactly like the real system's would.
DELIVERY_RADIUS_M = 120.0   # matches utm_aggregator.DELIVERY_RADIUS_M
DEPOT_SUPPRESS_M  = 200.0   # matches utm_aggregator.DEPOT_SUPPRESS_M

START = time.time()

# Scene control, driven entirely by the keyboard thread below.
#   mode        "manual" -> hold whatever attack_on says (fire on demand)
#               "auto"   -> the old timed loop (clean for TAKEOVER_AT, attack)
#   scenario    which staged scenario to run when attacking (see SCENARIOS)
#   attack_started_at  wall-clock time the CURRENT manual attack turned on,
#               used only by the "gradual" scenario to stage its escalation
CTRL = {"mode": "manual", "attack_on": False, "scenario": "fleet",
        "attack_started_at": None, "reset_token": 0}
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

# This fixture's clean drones fly the REAL Frisco depot<->delivery distances
# (2.8-12.8 km) at frisco_fleet's real cruise speed (15 m/s) -- a genuine 3 to
# 14+ MINUTES per leg. That's correct physics but a bad demo: in a normal
# testing session nothing ever visibly delivers, so the counter/badge look
# broken even though they aren't. ui_selftest exists to let you watch every
# UI state quickly, so its own clean-path sim (not the real fleet, and not
# the spoofed/breakaway motion, which already reads fast via its own
# multipliers) runs this many times faster than real life -- distances and
# geography are untouched, only the demo's clock is sped up.
DEMO_TIME_MULT = 12.0


def _clean_path_step(state, tgt, speed_mult=1.0):
    """One TICK_S step toward tgt; True once arrived. Mirrors
    SimDrone._step_toward exactly (position/heading only -- vx/vy/vz aren't
    needed for display). speed_mult scales how far it moves this tick, so a
    spoofed drone can visibly race across the map instead of only reporting
    a bigger number."""
    d = ff.distance_m(state["lat"], state["lon"], tgt[0], tgt[1])
    state["hdg"] = ff.bearing_deg(state["lat"], state["lon"], tgt[0], tgt[1])
    step = ff.SPEED_MS * speed_mult * DEMO_TIME_MULT * ff.TICK_S
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
            state["rel_alt"] = min(cruise, state["rel_alt"] + ff.CLIMB_RATE * DEMO_TIME_MULT * ff.TICK_S)
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
    msg = f"sysid {ghost_sid} not in known fleet roster - POSSIBLE COLLISION DETECTED!"
    ghosts.append({
        "sysid": ghost_sid, "company": "UNKNOWN", "kind": "ghost",
        "color": "#ff2e7e",
        "lat": glat, "lon": glon, "alt_m": 95, "amsl_m": 95 + ff.FIELD_ALT_M,
        "hdg": ghdg, "speed_ms": round(9.0 * SPOOF_SPEED_MULT, 1), "mode": "AUTO", "battery": 0,
        "status": "active", "age_s": 0.2, "spoofed": True,
        "alerts": [f"SPOOF — {msg}"], "codes": ["GHOST_ID"], "alert": True, "delivered_ts": None,
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
            notes.append((sid, "FLEET_GUIDED", guided_msg, _GRADUAL_STAGES[3]))

    return overrides, [], notes


# ---------------------------------------------------------------------------
# GPS SPOOF scenario: hard-coded true waypoint path for Wing #6.
#
# The attacker feeds Wing #6 a false GPS signal so the UTM sees it flying a
# normal clean depot->delivery leg. The drone's REAL autopilot, however, is
# following a completely different set of waypoints chosen by the attacker —
# the route below. This lets the demo show a widening gap between the spoofed
# marker (blue, calm) and a second orange "true position" marker that visibly
# works through a sequence of waypoints before exiting Wing's AO entirely.
#
# Waypoints are (lat, lon) in order. The drone departs from Wing's depot,
# visits each one at SPOOF_SPEED_MULT x cruise, and holds at the final
# capture point once it arrives. Each waypoint is labelled for the script:
#   WP0 → WP1  initial NW diversion (away from Wing's delivery cluster)
#   WP1 → WP2  jog north past the Medical Helipad no-fly boundary
#   WP2 → WP3  final east run to the capture point outside Wing's AO
# ---------------------------------------------------------------------------
_GPS_SPOOF_VICTIM_SID = 6
_GPS_SPOOF_WAYPOINTS = [
    ff.DEPOTS["Wing"],          # WP0 — start (Wing depot, west side)
    (33.172, -96.912),          # WP1 — NW diversion, inside AO but heading wrong
    (33.196, -96.905),          # WP2 — north, approaching Medical Helipad zone
    (33.210, -96.868),          # WP3 — capture point, outside Wing AO (east edge)
]
# Reported (spoofed) heading the attacker injects: points south-east toward a
# plausible Wing delivery, while the drone is actually flying north-west.
_GPS_SPOOF_REPORTED_HDG = 145.0
_GPS_SPOOF_REPORTED_SPEED = round(ff.SPEED_MS, 1)   # normal cruise — looks fine


def _gps_spoof_true_pos(t_since):
    """Walk Wing #6's TRUE position along _GPS_SPOOF_WAYPOINTS at
    SPOOF_SPEED_MULT × cruise, returning (lat, lon, hdg, leg_index)."""
    speed = ff.SPEED_MS * SPOOF_SPEED_MULT
    remaining = t_since * speed   # total metres the drone has covered

    lat, lon = _GPS_SPOOF_WAYPOINTS[0]
    for i in range(1, len(_GPS_SPOOF_WAYPOINTS)):
        wlat, wlon = _GPS_SPOOF_WAYPOINTS[i]
        seg = ff.distance_m(lat, lon, wlat, wlon)
        if remaining <= seg:
            frac = remaining / seg if seg > 0 else 1.0
            out_lat = lat + (wlat - lat) * frac
            out_lon = lon + (wlon - lon) * frac
            hdg = ff.bearing_deg(lat, lon, wlat, wlon)
            return round(out_lat, 6), round(out_lon, 6), round(hdg, 1), i - 1
        remaining -= seg
        lat, lon = wlat, wlon

    # Past the last waypoint — hold at capture point.
    hdg = ff.bearing_deg(_GPS_SPOOF_WAYPOINTS[-2][0], _GPS_SPOOF_WAYPOINTS[-2][1],
                          _GPS_SPOOF_WAYPOINTS[-1][0], _GPS_SPOOF_WAYPOINTS[-1][1])
    return round(lat, 6), round(lon, 6), round(hdg, 1), len(_GPS_SPOOF_WAYPOINTS) - 1


def _point_in_wing_ao(lat, lon):
    """Ray-casting check against Wing's area of operations polygon."""
    poly = ff.AREAS_OF_OPERATION.get("Wing", [])
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


# ===========================================================================
# LIVE FLIGHT-RULE EVALUATOR
# ---------------------------------------------------------------------------
# The real aggregator / utm_guardian.py derives flight-rule alerts geometrically
# from each aircraft's REPORTED telemetry — altitude vs the ceiling/floor,
# position vs its company's area of operations, and position vs every no-fly
# zone. This fixture now runs the SAME checks on the reported position it emits
# for each drone, so a hijacked drone that actually flies out of the envelope
# earns a real flight alert here exactly as a live drone would, instead of the
# alert being hand-scripted per scenario.
#
# Flight-rule codes (HIGH / LOW / OUT_AO / NOFLY:<name>) are DISTINCT from the
# security/spoof codes: their alert strings are NOT "SPOOF —" prefixed, so the
# map routes them to the amber flight-alert channel, not the magenta security
# one, and they never inflate the SECURITY count / spoof toast.
# ===========================================================================
def _point_in_poly(lat, lon, poly):
    """Ray-casting point-in-polygon for any (lat, lon) boundary list."""
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
    """Return [(code, msg)] for every flight-rule breach visible in this
    aircraft's REPORTED telemetry — the same evaluation a live aggregator runs
    off MAVLink. Codes: HIGH, LOW, OUT_AO, NOFLY:<zone>. Never SPOOF-prefixed.

    alt_m is the reported RELATIVE altitude (metres above ground), matching the
    frame the ceiling/floor limits in frisco_fleet.py are expressed in.
    """
    out = []

    # --- altitude envelope ---
    if alt_m > ff.MAX_ALT_M:
        out.append(("HIGH",
                    f"altitude {alt_m:.0f} m exceeds {ff.MAX_ALT_M:.0f} m ceiling"))
    else:
        # The floor is enforced only once the aircraft is out on a leg — not
        # during the normal takeoff/landing climb near its own pad — so ordinary
        # ops don't false-trip LOW. By the time a drone is >250 m from its depot
        # it has long since climbed through the 10 m floor to cruise.
        depot = ff.DEPOTS.get(company)
        away = depot is None or ff.distance_m(lat, lon, depot[0], depot[1]) > 250.0
        if 1.0 < alt_m < ff.MIN_ALT_M and away:
            out.append(("LOW",
                        f"altitude {alt_m:.0f} m below {ff.MIN_ALT_M:.0f} m floor"))

    # --- area of operations (only aircraft with a known company AO) ---
    poly = ff.AREAS_OF_OPERATION.get(company)
    if poly and not _point_in_poly(lat, lon, poly):
        out.append(("OUT_AO",
                    f"aircraft outside {company} area of operations"))

    # --- no-fly zones (circle containment) ---
    for z in ff.NO_FLY_ZONES:
        if ff.distance_m(lat, lon, z["lat"], z["lon"]) <= z["radius_m"]:
            out.append((f"NOFLY:{z['name']}",
                        f'aircraft inside no-fly zone "{z["name"]}"'))

    return out


def _scn_gpsspoof(now, t_since, attacking):
    """GPS position-spoofing demonstration with a hard-coded true waypoint path.

    Wing #6's reported telemetry follows a normal-looking clean depot->delivery
    leg (right altitude, right speed, heading SE toward a plausible delivery,
    well inside the AO). Its TRUE autopilot is flying a completely different
    attacker-chosen route: NW from the depot, north past the Medical Helipad
    boundary, then east to a capture point outside Wing's AO.

    The dashboard shows two markers for sysid 6:
      Blue  (kind="sim")      — what the UTM is being told: normal behaviour.
      Orange (kind="true_pos") — where the aircraft really is, working through
                                 the waypoint sequence in real time.

    Three alert codes fire and the guardian can independently derive all of them
    from real telemetry (see utm_guardian.py):
      GPS_SPOOF    Reported heading (SE) is the opposite of the movement vector
                   implied by the spoofed position deltas (the spoofed position
                   moves very slowly; the heading doesn't match even that).
      VEL_MISMATCH Reported speed (15 m/s) is inconsistent with the frame-to-
                   frame delta of the (stationary / slow-drifting) spoofed track.
      OUT_AO       Once the true position crosses past WP2, it exits Wing's AO
                   polygon — flagged by the guardian's AO check.
    """
    overrides, ghosts, notes = {}, [], []
    if not attacking:
        return overrides, ghosts, notes

    sid = _GPS_SPOOF_VICTIM_SID
    true_lat, true_lon, true_hdg, leg = _gps_spoof_true_pos(t_since)
    gap_m = int(ff.distance_m(
        true_lat, true_lon,
        ff.DEPOTS["Wing"][0], ff.DEPOTS["Wing"][1]))
    outside_ao = not _point_in_wing_ao(true_lat, true_lon)

    spoof_msg = (f"#{sid} heading ({_GPS_SPOOF_REPORTED_HDG:.0f}°) is opposite to "
                 f"its position-delta vector — GPS signal inconsistent")
    vel_msg   = (f"#{sid} reports {_GPS_SPOOF_REPORTED_SPEED} m/s but spoofed "
                 f"position delta implies near-zero ground speed")
    out_msg   = (f"#{sid} true position {gap_m} m from reported, "
                 + ("outside Wing AO — aircraft diverted" if outside_ao
                    else f"leg {leg+1}/{len(_GPS_SPOOF_WAYPOINTS)-1} of attacker route"))

    alerts = [("GPS_SPOOF", spoof_msg), ("VEL_MISMATCH", vel_msg)]
    if outside_ao or t_since > 5.0:   # OUT_AO fires as soon as it's meaningful
        alerts.append(("OUT_AO", out_msg))

    # The REPORTED position stays on the clean depot->delivery path — no
    # breakaway_bearing_deg or hijacked flag — so the spoofed marker appears
    # to behave normally. Only heading and speed are pinned to the fake values.
    overrides[sid] = {
        "spoofed": True,
        "alerts": alerts,
        "speed_ms": _GPS_SPOOF_REPORTED_SPEED,
        # hdg override: the clean_path_pos heading is suppressed in snapshot()
        # below because we inject a fixed reported heading via the override dict.
        "_reported_hdg": _GPS_SPOOF_REPORTED_HDG,
    }
    for code, msg in alerts:
        notes.append((sid, code, msg, 0.0))

    # Orange "true position" ghost marker.
    ghosts.append({
        "sysid": sid, "company": "Wing", "kind": "true_pos",
        "color": "#ff6600",
        "lat": true_lat, "lon": true_lon,
        "alt_m": ff.CRUISE_ALT_M["Wing"],
        "amsl_m": ff.CRUISE_ALT_M["Wing"] + ff.FIELD_ALT_M,
        "hdg": true_hdg,
        "speed_ms": round(ff.SPEED_MS * SPOOF_SPEED_MULT, 1),
        "mode": "AUTO", "battery": 0,
        "status": "active", "age_s": 0.2, "spoofed": True,
        "alerts": [f"SPOOF — true pos, leg {leg+1}/{len(_GPS_SPOOF_WAYPOINTS)-1}"
                   + (" — OUTSIDE AO" if outside_ao else "")],
        "alert": True, "delivered_ts": None,
    })

    return overrides, ghosts, notes


# ---------------------------------------------------------------------------
# ENVELOPE-BREACH scenario: a coordinated hijack that flies each seized drone
# into a DIFFERENT flight-rule violation, so every flight-alert type the
# aggregator can raise — HIGH, LOW, OUT_AO and NOFLY — is exercised at once,
# and always in the "drone is hacked" context. Positions/altitudes are pinned
# deterministically (see the pin_pos override) so all four fire the instant the
# attack starts, instead of depending on a runaway path reaching a boundary.
#
# Each drone is spoofed=True with a FLEET_GUIDED takeover code (so it registers
# as hacked in the Security channel); the geometric evaluator then independently
# adds the flight-rule code from the pinned position, exactly as it would from a
# live hijacked aircraft's telemetry.
# ---------------------------------------------------------------------------
#   sysid, expected flight code, pinned (lat, lon, rel_alt_m), human note
_ENVELOPE_BREACHES = [
    (2,  "HIGH",   (33.150, -96.850, 300.0),
     "seized and force-climbed above the 122 m ceiling"),
    (3,  "LOW",    (33.120, -96.840, 5.0),
     "seized and pushed below the 10 m floor out on a leg"),
    (4,  "OUT_AO", (33.150, -96.700, 80.0),
     "seized and flown east out of Wing's area of operations"),
    (14, "NOFLY",  (33.1150, -96.7520, 100.0),
     "seized and flown into a restricted no-fly zone"),
]


def _scn_envelope(now, t_since, attacking):
    """Hijack four drones and fly each into a distinct flight-envelope breach,
    so HIGH, LOW, OUT_AO and NOFLY all occur together while the drones are
    hacked. See _ENVELOPE_BREACHES for the pinned positions."""
    overrides, notes = {}, []
    if not attacking:
        return overrides, [], notes
    for sid, _brk, pin, note in _ENVELOPE_BREACHES:
        msg = f"#{sid} {note}"
        overrides[sid] = {"spoofed": True, "pin_pos": pin,
                          "alerts": [("FLEET_GUIDED", msg)]}
        notes.append((sid, "FLEET_GUIDED", msg, 0.0))
    return overrides, [], notes


SCENARIOS = {
    "fleet":    ("Fleet takeover",          _scn_fleet),
    "ghost":    ("Ghost aircraft",          _scn_ghost),
    "climbvel": ("Climb + velocity spoof",  _scn_climbvel),
    "gradual":  ("Gradual escalation",      _scn_gradual),
    "gpsspoof": ("GPS position spoof",      _scn_gpsspoof),
    "envelope": ("Envelope breach (all flight rules)", _scn_envelope),
}
SCENARIO_ORDER = ["fleet", "ghost", "climbvel", "gradual", "gpsspoof", "envelope"]


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
    global _delivery_count, _delivered_ts, _at_delivery, _PREV_SPOOFED_SIDS
    global _PREV_NOTES_KEY
    now = time.time()
    attacking, t_since = _attack_state(now)
    with CTRL_LOCK:
        scenario_key = CTRL["scenario"]
    _label, scn_fn = SCENARIOS[scenario_key]
    overrides, ghosts, notes = scn_fn(now, t_since, attacking)

    drones, flagged_sysids, by_code = [], [], {}
    cur_spoofed_sids = set()
    # Flight-rule (advisory) accumulator — kept SEPARATE from the security ones
    # above so a flight breach never inflates the SECURITY count/toast, exactly
    # like the real aggregator's two independent alert channels.
    flight_alerting_sids = set()  # drones with >=1 active flight-rule breach

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
            # A scenario can PIN a hijacked drone to an exact reported position
            # and altitude (deterministic breach placement) instead of relying
            # on the clean-path integrator to fly it there over time. Heading is
            # aimed from the company depot toward the pinned point so the marker
            # arrow still reads sensibly.
            if "pin_pos" in ov:
                plat, plon, palt = ov["pin_pos"]
                dep = ff.DEPOTS.get(company, (plat, plon))
                hdg = ff.bearing_deg(dep[0], dep[1], plat, plon)
                lat, lon, alt_here = plat, plon, palt
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
            cur_spoofed_sids.add(sid)

        if spoofed:
            if sid not in _PREV_SPOOFED_SIDS:
                SPOOF_TRACKS[sid] = []   # fresh incident: start recording clean
            SPOOF_TRACKS[sid].append([round(lat, 6), round(lon, 6)])

        # Flight plan: current position -> active target -> depot (if returning).
        # Mirrors what the real aggregator builds from POSITION_TARGET_GLOBAL_INT.
        # Only included for non-spoofed drones — a hacked drone's declared plan
        # can't be trusted and the map suppresses it anyway.
        plan = None
        depot_ll = ff.DEPOTS[company]
        if not spoofed:
            with CLEAN_PATH_LOCK:
                ps = CLEAN_PATH_STATE.get(sid)
            if ps is not None:
                pos = [lat, lon]
                tgt = ps.get("target")
                cur_phase = ps.get("phase")
                if cur_phase == "to_delivery" and tgt:
                    plan = [pos, list(tgt), list(depot_ll)]
                elif cur_phase in ("to_depot", "drop", "rest"):
                    plan = [pos, list(depot_ll)]

        # Delivery detection — same rule utm_aggregator.py's evaluator uses on
        # real telemetry: "parked" near a delivery point and NOT near a depot.
        # Computed from the drone's actual reported position for EVERY drone,
        # every tick, regardless of spoofed status, so tracking is always
        # fresh and can't get stuck mid-attack (the old version only read the
        # internal flight phase while `not spoofed`, so a drone that was
        # spoofed-but-still-moving on its real route, e.g. climbvel's #3/#14,
        # would freeze its edge-detector and silently drop whatever delivery
        # happened while it was flagged). Only the CREDIT — the counter and
        # delivered_ts, which pops the "✓ Delivered" badge — is withheld while
        # a drone is currently spoofed; a hacked drone's own "dropped it off"
        # signal still can't be trusted, same as before.
        near_deliv = min(
            (ff.distance_m(lat, lon, p[0], p[1]) for p in ff.DELIVERY_POINTS),
            default=1e9) < DELIVERY_RADIUS_M
        near_depot = ff.distance_m(lat, lon, depot_ll[0], depot_ll[1]) < DEPOT_SUPPRESS_M
        at_delivery = near_deliv and not near_depot
        was_at_delivery = _at_delivery.get(sid, False)
        if at_delivery and not was_at_delivery and not spoofed:
            _delivery_count += 1
            _delivered_ts[sid] = now
        _at_delivery[sid] = at_delivery

        # ---- LIVE flight-rule evaluation on the REPORTED telemetry ----
        # Same geometric checks the aggregator applies to every live aircraft.
        # Runs regardless of whether the drone is spoofed: a hijacked drone that
        # actually leaves the envelope (climbs past the ceiling, exits its AO,
        # enters a no-fly zone) now earns a genuine flight alert here, in the
        # exact shape the real system emits. Flight strings are plain (no
        # "SPOOF —" prefix) so the map shows them as amber advisories.
        flight_codes = []
        for code, msg in _flight_rule_alerts(company, lat, lon, alt_here):
            alerts.append(msg)
            flight_codes.append(code)
        if flight_codes:
            flight_alerting_sids.add(sid)

        drones.append({
            "sysid": sid, "company": company, "kind": "sim",
            "color": ff.COMPANY_COLOR[company],
            "lat": lat, "lon": lon, "alt_m": round(alt_here, 1),
            "amsl_m": round(alt_here + ff.FIELD_ALT_M, 1),
            "hdg": hdg, "speed_ms": speed_ms, "mode": mode, "battery": battery,
            "status": "active", "age_s": 0.2, "spoofed": spoofed,
            "alerts": alerts, "alert": bool(alerts),
            "codes": [code for code, _ in (ov.get("alerts", []) if ov else [])],
            "flight_codes": flight_codes,
            "delivered_ts": _delivered_ts.get(sid) if not spoofed else None,
            "plan": plan,
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
                "codes": ["SRC_CONFLICT"],
                "alert": True, "delivered_ts": None,
            })

    ghost_sysids = {g["sysid"] for g in ghosts}
    for g in ghosts:
        # A real intruder aircraft is subject to the same flight rules as the
        # fleet. Evaluate them on its reported position (skip the synthetic
        # true_pos truth-echo, whose divergence is already a security finding).
        if g.get("kind") == "ghost":
            gfc = []
            for code, msg in _flight_rule_alerts(
                    g.get("company"), g["lat"], g["lon"], g.get("alt_m", 0)):
                g.setdefault("alerts", []).append(msg)
                gfc.append(code)
            if gfc:
                g["flight_codes"] = gfc
                g["alert"] = True
                flight_alerting_sids.add(g["sysid"])
        drones.append(g)
        flagged_sysids.append(g["sysid"])
        gsid = g["sysid"]
        cur_spoofed_sids.add(gsid)
        if gsid not in _PREV_SPOOFED_SIDS:
            SPOOF_TRACKS[gsid] = []
        SPOOF_TRACKS[gsid].append([g["lat"], g["lon"]])

    _PREV_SPOOFED_SIDS = cur_spoofed_sids

    # Edge-triggered security log: append a new entry only when a (sysid, code)
    # pair appears that wasn't active last tick. This means each anomaly gets
    # exactly one notification when it fires and the log persists after 'c'.
    # Only SECURITY notes go in the edge-triggered log now: the flight-rule
    # event log was removed from the UI, and because some flight codes (e.g.
    # OUT_AO) also exist in the map's security vocabulary, routing flight notes
    # here would leak them back into the Security tab. Flight alerts surface
    # solely through each drone's live `alerts`/`flight_codes` (the amber
    # Flight-alerts row + marker badge), never the log.
    log_notes = notes
    cur_notes_key = {(sid, code) for sid, code, _msg, _st in log_notes}
    for sid, code, msg, stage_t in log_notes:
        if (sid, code) not in _PREV_NOTES_KEY:
            company = ff.FLEET.get(sid, ("UNKNOWN", "?"))[0]
            _SEC_LOG.appendleft({
                "t": now, "sysid": sid, "company": company,
                "kind": "enter", "code": code, "text": msg,
            })
    # Clear edges: anomalies that just stopped firing get a "cleared" entry.
    for sid, code in _PREV_NOTES_KEY:
        if (sid, code) not in cur_notes_key:
            company = ff.FLEET.get(sid, ("UNKNOWN", "?"))[0]
            _SEC_LOG.appendleft({
                "t": now, "sysid": sid, "company": company,
                "kind": "clear", "code": code, "text": "cleared",
            })
    _PREV_NOTES_KEY.clear()
    _PREV_NOTES_KEY.update(cur_notes_key)

    # by_code was already built in the drone loop above (one entry per override alert).
    # Add any ghost-only codes that aren't in overrides.
    for sid, code, _msg, _st in notes:
        if sid in ghost_sysids and code not in by_code:
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
    """Apply one command to CTRL. The single entry point the terminal
    keyboard() loop uses to change the scene -- this is now the ONLY control
    surface; nothing on the served page can call it."""
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
        elif cmd in ("5", "gpsspoof", "gps"):
            CTRL["scenario"] = "gpsspoof"
        elif cmd in ("6", "envelope", "env"):
            CTRL["scenario"] = "envelope"
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
            # One drone's regular/original plan plus its recorded SPOOFED
            # pathway (only populated while/after it's been flagged) —
            # same shape as the real aggregator's /api/track.
            qs = parse_qs(parsed.query)
            try:
                sid = int((qs.get("sysid") or [""])[0])
            except ValueError:
                self._send(json.dumps({"error": "missing/invalid ?sysid="}).encode(),
                           "application/json")
                return
            data = snapshot()   # also freshens SPOOF_TRACKS for this instant
            rec = next((d for d in data["drones"] if d.get("sysid") == sid), None)
            fallback_company = next((c for s, c, *_ in DEMO if s == sid), "Unknown")
            payload = {
                "sysid": sid,
                "company": rec["company"] if rec else fallback_company,
                "color": rec["color"] if rec else ff.COMPANY_COLOR.get(fallback_company, "#888888"),
                "spoof_track": list(SPOOF_TRACKS.get(sid, [])),
                "lat": rec["lat"] if rec else None,
                "lon": rec["lon"] if rec else None,
                "alerts": rec["alerts"] if rec else [],
                "spoofed": bool(rec and rec.get("spoofed")),
            }
            self._send(json.dumps(payload).encode(), "application/json")
        elif parsed.path == "/api/tracks":
            # Off-course page: every drone that has EVER been flagged in this
            # demo session, each with its recorded spoofed pathway plotted
            # against its regular/original plan. Drones that have never been
            # spoofed don't appear here — their "where it's going" line lives
            # on the main map instead.
            data = snapshot()
            by_sid = {d["sysid"]: d for d in data["drones"]}
            out = []
            for sid, track in SPOOF_TRACKS.items():
                if not track:
                    continue
                rec = by_sid.get(sid)
                fallback_company = next((c for s, c, *_ in DEMO if s == sid), "Unknown")
                out.append({
                    "sysid": sid,
                    "company": rec["company"] if rec else fallback_company,
                    "color": rec["color"] if rec else ff.COMPANY_COLOR.get(fallback_company, "#888888"),
                    "spoof_track": list(track),
                    "lat": rec["lat"] if rec else None,
                    "lon": rec["lon"] if rec else None,
                    "alerts": rec.get("alerts", []) if rec else [],
                    "spoofed": bool(rec and rec.get("spoofed")),
                })
            out.sort(key=lambda x: x["sysid"])
            self._send(json.dumps({"drones": out}).encode(), "application/json")
        elif parsed.path == "/compare":
            try:
                with open(COMPARE_HTML_PATH, "rb") as f:
                    self._send(f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(b"<h1>utm_compare.html not found</h1>"
                           b"<p>Keep it in the same folder as ui_selftest.py.</p>",
                           "text/html; charset=utf-8")
        elif parsed.path in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    html = f.read()
            except FileNotFoundError:
                self._send(b"<h1>put utm_map.html in this folder</h1>",
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
    """Read stdin lines and drive the scene -- the only control surface this
    fixture has. Enter toggles the takeover; 'q' quits the process."""
    for line in iter(input, None):     # blocks per line; ends on EOF/Ctrl+C
        cmd = line.strip().lower()
        if cmd in ("q", "quit", "exit"):
            stop.set(); break
        if not _apply_ctrl(cmd):
            print("[selftest] keys: Enter=toggle  a=attack  c=clean  auto=timed  "
                  "1=fleet 2=ghost 3=climb+vel 4=gradual 5=gpsspoof  q=quit")
            continue
        print(f"[selftest] {_state_line()}")


def main():
    httpd = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    httpd.timeout = 0.5   # so the loop checks the stop flag even when idle
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