"""
lightweight_fleet.py
===============

Scenario geography + the LIGHTWEIGHT simulated drones for the GridEye demo.

Two delivery companies operate over Frisco, TX:
  * Wing     - system IDs 1..6   (1 = real ArduPilot SITL, 2..6 = lightweight)
  * Zipline  - system IDs 11..16 (11 = real ArduPilot SITL, 12..16 = lightweight)

Running THIS file launches only the 10 LIGHTWEIGHT drones (Wing 2..6,
Zipline 12..16). They each fly depot -> delivery -> depot over real Frisco
geography along a direct transit plan drawn at random from THEIR OWN COMPANY's
route pool (ROUTE_LIBRARIES["Wing"] or ROUTE_LIBRARIES["Zipline"]) — the
waypoints themselves are frozen, but WHICH plan a drone flies is re-rolled every
cycle, so drones of the same company swap routes and two of them can be on the
same plan at once. Each company's plans stay inside that company's own area of
operation and fan out to fill it. A drone drops exactly once per trip, on reaching
the final waypoint of its tour (broadcast as a MISSION_COUNT /
MISSION_ITEM_INT sequence, same as a real GCS mission upload) and emit normal
MAVLink (HEARTBEAT + GLOBAL_POSITION_INT) over UDP, so the GridEye aggregator
(built next) cannot tell them apart from the real SITL drones.

The 2 real SITL drones are launched separately (see RUN NOTES at the bottom).

This module is ALSO the single source of truth for the scenario: the aggregator
imports FLEET / DEPOTS / INGEST_PORT from here so everything agrees on who is who.
"""

import math
import os
import random
import threading
import time

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

# ===========================================================================
# SCENARIO CONFIG  (imported by the aggregator too)
# ===========================================================================
# All lightweight drones send their telemetry here; the aggregator listens here.
INGEST_PORT = 14570

# Frisco field elevation (~700 ft) used as the ground reference for altitudes.
FIELD_ALT_M = 215.0

# Company depots (real Frisco coordinates): Wing on the west, Zipline on the east.
DEPOTS = {
    "Wing":    (33.1543, -96.8700),   # west side, near Toyota Stadium area
    "Zipline": (33.1180, -96.7720),   # east side
}

# Cruise heights (metres above ground). Different bands per company so that later
# the GridEye can show altitude separation between the two operators.
CRUISE_ALT_M = {"Wing": 80.0, "Zipline": 110.0}

# Company display colours (used everywhere via the JSON; single source of truth).
# Zipline is yellow so it doesn't clash with the red no-fly zones.
COMPANY_COLOR = {"Wing": "#1E88E5", "Zipline": "#FDD835"}  # blue / yellow

# Where the GridEye map should centre its initial view (central Frisco).
MAP_CENTER = (33.150, -96.823)

# Who is in the fleet: sysid -> (company, kind). "SITL" drones are launched
# separately; "sim" drones are the lightweight ones THIS file flies.
FLEET = {
    1:  ("Wing", "SITL"),
    2:  ("Wing", "sim"), 3: ("Wing", "sim"), 4: ("Wing", "sim"),
    5:  ("Wing", "sim"), 6: ("Wing", "sim"),
    11: ("Zipline", "SITL"),
    12: ("Zipline", "sim"), 13: ("Zipline", "sim"), 14: ("Zipline", "sim"),
    15: ("Zipline", "sim"), 16: ("Zipline", "sim"),
}

# Delivery destinations. These are now the union of every drop-off in the two
# per-company route libraries below (each library's drop-offs fill that
# company's whole area of operation). Kept as a flat list because the aggregator
# imports it to decide when a drone has reached *a* delivery point. Single source
# of truth: it is rebuilt from ROUTE_LIBRARIES at import time, just below the
# libraries, so it can never drift out of sync with the routes.
DELIVERY_POINTS = []  # populated from ROUTE_LIBRARIES after they are defined

# ---------------------------------------------------------------------------
# PER-COMPANY ROUTE LIBRARIES  (one pool per company; each drone draws its own)
# ---------------------------------------------------------------------------
# TWO frozen pools of delivery plans -- one for Wing, one for Zipline -- NOT one
# route per drone. Every drone picks a route at random from ITS OWN company's
# pool each time it leaves the depot, so the assignment is randomised and any
# two drones of the same company may end up flying the same plan (or swap plans
# between cycles) -- exactly like a dispatcher handing out whatever job is next
# off that company's queue. (This is the "different path for each drone"
# behaviour, unchanged -- only WHICH pool a drone draws from is now per-company.)
#
# Each entry is the list of waypoints AFTER the depot:
#
#     [ climb-out, cruise, turn, approach, delivery point ]
#
# These are REAL delivery corridors: two straight legs joined by ONE gentle turn.
# The drone departs the depot on a straight climb-out + cruise leg, makes a single
# course change at 'turn', then flies a straight approach to the drop-off (flown
# LAST). climb-out and cruise lie ON the depot->turn line and approach lies ON the
# turn->delivery line, so the ONLY heading change on the whole path is that single
# dog-leg -- the routes never zig-zag. The drone hovers and drops exactly once, on
# reaching that final waypoint; the trip home is the same plan flown in reverse.
#
# Every vertex of a company's routes lies INSIDE THAT COMPANY'S OWN area of
# operation (with a safety margin), and the delivery points are spread on a grid
# across the whole AO, so each company's routes fan out and FILL its entire area.
# Because a drone only ever draws from its own company's pool, it can never be
# handed a route that leaves its airspace. Generated and validated by
# gen_routes.py (point-in-polygon + edge-margin check on every rounded vertex).
ROUTE_LIBRARIES = {
    "Wing": [
        [  # route 0 — 5 waypoints, ~1.8 km one-way, delivers at (33.1385, -96.8738)
            (33.1535, -96.87), (33.1492, -96.8698), (33.146, -96.8696), (33.1419, -96.8719), (33.1385, -96.8738),
        ],
        [  # route 1 — 5 waypoints, ~2.1 km one-way, delivers at (33.1735, -96.8738)
            (33.1553, -96.8699), (33.1605, -96.8695), (33.1643, -96.8691), (33.1693, -96.8717), (33.1735, -96.8738),
        ],
        [  # route 2 — 5 waypoints, ~2.6 km one-way, delivers at (33.1385, -96.8492)
            (33.1537, -96.8687), (33.1507, -96.8622), (33.1485, -96.8574), (33.143, -96.8529), (33.1385, -96.8492),
        ],
        [  # route 3 — 5 waypoints, ~2.8 km one-way, delivers at (33.1735, -96.8492)
            (33.1551, -96.8687), (33.159, -96.8619), (33.1618, -96.8569), (33.1682, -96.8527), (33.1735, -96.8492),
        ],
        [  # route 4 — 5 waypoints, ~4.5 km one-way, delivers at (33.1385, -96.8247)
            (33.154, -96.8675), (33.1522, -96.8546), (33.1509, -96.8451), (33.1441, -96.8339), (33.1385, -96.8247),
        ],
        [  # route 5 — 5 waypoints, ~4.6 km one-way, delivers at (33.1735, -96.8247)
            (33.1548, -96.8675), (33.1575, -96.8543), (33.1595, -96.8447), (33.1672, -96.8337), (33.1735, -96.8247),
        ],
        [  # route 6 — 5 waypoints, ~5.8 km one-way, delivers at (33.1035, -96.8492)
            (33.1519, -96.8684), (33.1395, -96.8601), (33.1305, -96.854), (33.1156, -96.8514), (33.1035, -96.8492),
        ],
        [  # route 7 — 5 waypoints, ~6.1 km one-way, delivers at (33.2085, -96.8492)
            (33.1569, -96.8684), (33.1702, -96.8601), (33.1799, -96.854), (33.1956, -96.8514), (33.2085, -96.8492),
        ],
        [  # route 8 — 5 waypoints, ~6.5 km one-way, delivers at (33.1385, -96.8002)
            (33.154, -96.8664), (33.1524, -96.8474), (33.1512, -96.8336), (33.1442, -96.8153), (33.1385, -96.8002),
        ],
        [  # route 9 — 5 waypoints, ~6.6 km one-way, delivers at (33.1735, -96.8002)
            (33.1548, -96.8663), (33.1573, -96.8472), (33.1592, -96.8333), (33.1671, -96.8151), (33.1735, -96.8002),
        ],
        [  # route 10 — 5 waypoints, ~6.8 km one-way, delivers at (33.1035, -96.8247)
            (33.1521, -96.8673), (33.1404, -96.853), (33.1319, -96.8426), (33.1163, -96.8328), (33.1035, -96.8247),
        ],
        [  # route 11 — 5 waypoints, ~7.1 km one-way, delivers at (33.2085, -96.8247)
            (33.1567, -96.8673), (33.1693, -96.853), (33.1786, -96.8425), (33.195, -96.8328), (33.2085, -96.8247),
        ],
        [  # route 12 — 5 waypoints, ~8.3 km one-way, delivers at (33.1035, -96.8002)
            (33.1521, -96.8661), (33.1409, -96.846), (33.1326, -96.8313), (33.1166, -96.8142), (33.1035, -96.8002),
        ],
        [  # route 13 — 5 waypoints, ~8.5 km one-way, delivers at (33.2085, -96.8002)
            (33.1566, -96.8661), (33.1689, -96.8459), (33.1778, -96.8311), (33.1947, -96.8141), (33.2085, -96.8002),
        ],
    ],
    "Zipline": [
        [  # route 0 — 5 waypoints, ~2.1 km one-way, delivers at (33.137, -96.766)
            (33.119, -96.772), (33.1243, -96.7718), (33.1281, -96.7717), (33.133, -96.7686), (33.137, -96.766),
        ],
        [  # route 1 — 5 waypoints, ~2.6 km one-way, delivers at (33.103, -96.794)
            (33.1175, -96.7733), (33.1147, -96.7802), (33.1127, -96.7851), (33.1074, -96.79), (33.103, -96.794),
        ],
        [  # route 2 — 5 waypoints, ~2.9 km one-way, delivers at (33.137, -96.794)
            (33.1187, -96.7734), (33.1225, -96.7805), (33.1253, -96.7857), (33.1317, -96.7903), (33.137, -96.794),
        ],
        [  # route 3 — 5 waypoints, ~4.8 km one-way, delivers at (33.103, -96.822)
            (33.1177, -96.7747), (33.1162, -96.7887), (33.1152, -96.799), (33.1085, -96.8116), (33.103, -96.822),
        ],
        [  # route 4 — 5 waypoints, ~5.0 km one-way, delivers at (33.137, -96.822)
            (33.1185, -96.7747), (33.1211, -96.789), (33.123, -96.7994), (33.1307, -96.8118), (33.137, -96.822),
        ],
        [  # route 5 — 5 waypoints, ~5.7 km one-way, delivers at (33.171, -96.766)
            (33.1206, -96.7711), (33.1341, -96.7665), (33.144, -96.7631), (33.1589, -96.7647), (33.171, -96.766),
        ],
        [  # route 6 — 5 waypoints, ~6.0 km one-way, delivers at (33.171, -96.794)
            (33.1205, -96.7737), (33.1334, -96.7823), (33.1429, -96.7886), (33.1583, -96.7916), (33.171, -96.794),
        ],
        [  # route 7 — 5 waypoints, ~7.2 km one-way, delivers at (33.103, -96.85)
            (33.1177, -96.776), (33.1163, -96.797), (33.1153, -96.8123), (33.1085, -96.833), (33.103, -96.85),
        ],
        [  # route 8 — 5 waypoints, ~7.2 km one-way, delivers at (33.171, -96.822)
            (33.1203, -96.775), (33.1325, -96.7904), (33.1414, -96.8016), (33.1577, -96.8128), (33.171, -96.822),
        ],
        [  # route 9 — 5 waypoints, ~7.3 km one-way, delivers at (33.137, -96.85)
            (33.1194, -96.7757), (33.1268, -96.7952), (33.1322, -96.8094), (33.1349, -96.8317), (33.137, -96.85),
        ],
        [  # route 10 — 5 waypoints, ~9.0 km one-way, delivers at (33.171, -96.85)
            (33.1203, -96.7763), (33.1321, -96.7985), (33.1407, -96.8147), (33.1573, -96.8341), (33.171, -96.85),
        ],
        [  # route 11 — 5 waypoints, ~9.5 km one-way, delivers at (33.205, -96.794)
            (33.1225, -96.7725), (33.1456, -96.7752), (33.1625, -96.7772), (33.1859, -96.7865), (33.205, -96.794),
        ],
        [  # route 12 — 5 waypoints, ~10.3 km one-way, delivers at (33.205, -96.822)
            (33.1221, -96.775), (33.1436, -96.7908), (33.1594, -96.8023), (33.1845, -96.8131), (33.205, -96.822),
        ],
        [  # route 13 — 5 waypoints, ~11.6 km one-way, delivers at (33.205, -96.85)
            (33.1226, -96.7754), (33.1468, -96.7933), (33.1645, -96.8063), (33.1868, -96.8303), (33.205, -96.85),
        ],
    ],
}

# Back-compat alias: the combined pool (both companies). Nothing in the fleet
# draws from this any more -- each drone uses its company's list via the helpers
# below -- but keeping the old name defined avoids AttributeErrors if any tool
# still references ROUTE_LIBRARY. It is NOT used for route assignment.
ROUTE_LIBRARY = ROUTE_LIBRARIES["Wing"] + ROUTE_LIBRARIES["Zipline"]

# Rebuild DELIVERY_POINTS (declared above) from the libraries: the last waypoint
# of every route in both pools. The aggregator imports this to detect arrivals,
# so building it from the routes keeps the two in lock-step.
DELIVERY_POINTS = [route[-1] for routes in ROUTE_LIBRARIES.values() for route in routes]

# Areas of operation: one polygon per company, drawn dotted in the company colour.
# Each polygon contains that company's depot AND all delivery points; Wing's leans
# west to its depot, Zipline's leans east to its depot (they overlap in the middle).
# Coordinates are (lat, lon), listed around the boundary.
AREAS_OF_OPERATION = {
    "Wing": [
        (33.226, -96.788), (33.226, -96.862), (33.160, -96.886),
        (33.086, -96.860), (33.086, -96.788),
    ],
    "Zipline": [
        (33.222, -96.864), (33.222, -96.770), (33.150, -96.752),
        (33.086, -96.770), (33.086, -96.864),
    ],
}

# AO label placement: pinned to a fixed corner of each AO — Wing top-right,
# Zipline bottom-left — so the two are diagonal opposites and never overlap.
# 'dir' is which way the text runs from that corner.
AO_LABEL = {
    "Wing":    {"lat": 33.226, "lon": -96.788, "dir": "right"},  # top-right corner
    "Zipline": {"lat": 33.086, "lon": -96.864, "dir": "left"},   # bottom-left corner
}

# Back-compat aliases: some modules refer to these polygons as "geofences".
# They point at the same objects, so either name works and neither can raise
# AttributeError. AREAS_OF_OPERATION / AO_LABEL remain the canonical names.
GEOFENCES = AREAS_OF_OPERATION
GEOFENCE_LABEL = AO_LABEL

# No-fly zones (circles), placed JUST OUTSIDE the AO boundaries so a real drone can
# be flown into them by hand for the demo. Coordinates are illustrative Frisco spots.
NO_FLY_ZONES = [
    {"name": "Medical helipad", "lat": 33.1800, "lon": -96.8950, "radius_m": 800},
    {"name": "School zone",     "lat": 33.1150, "lon": -96.7520, "radius_m": 700},
]

# Altitude limits for the GridEye (metres above ground). 400 ft ceiling, 10 m floor.
MAX_ALT_M = 122.0   # ~400 ft
MIN_ALT_M = 10.0

# Flight model parameters.
SPEED_MS = 15.0      # cruise ground speed
CLIMB_RATE = 4.0     # m/s while climbing to cruise
HOVER_S = 5.0        # pause at each delivery / depot (the "drop")
TICK_S = 0.25        # update + telemetry rate (4 Hz)

# Routing: the waypoints themselves are frozen in ROUTE_LIBRARY above; what is
# randomised is WHICH tour each drone draws, re-rolled every time it leaves the
# depot. Set FLEET_ROUTE_SEED to an int for a repeatable assignment (handy when
# filming the demo); leave it None to get a fresh shuffle every run.
FLEET_ROUTE_SEED = None

M_PER_DEG_LAT = 111_320.0


# ===========================================================================
# GEO HELPERS
# ===========================================================================
def meters_per_deg_lon(lat_deg):
    return 111_320.0 * math.cos(math.radians(lat_deg))


def bearing_deg(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    y = math.sin(d_lon) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def distance_m(lat1, lon1, lat2, lon2):
    dx = (lon2 - lon1) * meters_per_deg_lon((lat1 + lat2) / 2)
    dy = (lat2 - lat1) * M_PER_DEG_LAT
    return math.hypot(dx, dy)


# ===========================================================================
# ROUTE LOOKUP  (keyed on COMPANY + a POOL INDEX, not on sysid)
# ===========================================================================
# Routes belong to a COMPANY's pool, not to any one drone: a (company, index)
# pair is a ticket into ROUTE_LIBRARIES[company], and two drones of the same
# company holding the same ticket fly the same tour. Keeping company + index
# (rather than a copy of the waypoints) in each drone's state is what lets the
# aggregator, the map and ui_selftest all talk about "Wing route 7" and mean the
# same thing. Every helper now takes the company so it draws from the right pool.
def route_count(company):
    return len(ROUTE_LIBRARIES[company])


def pick_route_idx(company, rng):
    """Draw a route at random from THIS COMPANY's pool. No exclusion between
    drones — collisions are the point: two drones of the same company CAN be on
    the same tour."""
    return rng.randrange(len(ROUTE_LIBRARIES[company]))


def delivery_point(company, idx):
    """The single drop-off for this route: its last waypoint."""
    return ROUTE_LIBRARIES[company][idx][-1]


def outbound_route(company, idx):
    """Depot -> delivery: this company's route vertices, delivery point last.
    The caller flies these in order and drops only at the final one."""
    return list(ROUTE_LIBRARIES[company][idx])


def return_route(company, idx, depot):
    """Delivery -> depot: the tour flown in reverse, finishing at the depot."""
    return list(reversed(ROUTE_LIBRARIES[company][idx][:-1])) + [tuple(depot)]


# ===========================================================================
# THE LIGHTWEIGHT SIM DRONE
# ===========================================================================
class SimDrone:
    def __init__(self, sysid, company):
        self.sysid = sysid
        self.company = company
        self.depot = DEPOTS[company]
        self.cruise = CRUISE_ALT_M[company]

        # Its own MAVLink link, stamped with this drone's system id. Created
        # before the first route so we can broadcast that route as a mission.
        self.conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{INGEST_PORT}",
            source_system=sysid, source_component=1, dialect="ardupilotmega")

        # Route assignment is RANDOM and re-rolled every cycle, so drones swap
        # tours and can double up on the same one. Seed from FLEET_ROUTE_SEED
        # (mixed with sysid so the drones don't all draw alike) when a
        # repeatable run is wanted, otherwise from system entropy.
        self.rng = random.Random(None if FLEET_ROUTE_SEED is None
                                 else FLEET_ROUTE_SEED + sysid)

        # State.
        self.lat, self.lon = self.depot
        self.rel_alt = 0.0
        self.phase = "to_delivery"
        self.route_idx = pick_route_idx(self.company, self.rng)
        self.target = delivery_point(self.company, self.route_idx)   # this tour's single drop-off
        self.dest = self.target          # where it's currently headed (for the flight plan)
        self.route = outbound_route(self.company, self.route_idx)
        self.hdg = bearing_deg(self.lat, self.lon, self.route[0][0], self.route[0][1])
        self.leg_idx = 0                 # which waypoint in self.route we're flying to
        self.hover_until = 0.0
        self.vx = self.vy = self.vz = 0.0   # cm/s for telemetry
        self.battery = 100.0                # % — drains flying, tops up at depot
        self._send_route_mission()       # let the aggregator draw this route now

    def _send_route_mission(self):
        """Broadcast the current route as a MISSION_COUNT + MISSION_ITEM_INT
        sequence — the same messages a real GCS uses to display a full
        multi-waypoint mission line (what Mission Planner draws). The
        aggregator listens for these and uses them to draw the real
        multi-waypoint plan instead of a straight guess at the final target."""
        n = len(self.route)
        self.conn.mav.mission_count_send(0, 0, n)
        for seq, (plat, plon) in enumerate(self.route):
            self.conn.mav.mission_item_int_send(
                0, 0, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                1 if seq == self.leg_idx else 0,   # current
                1,                     # autocontinue
                0, 0, 0, 0,            # param1-4 (unused)
                int(plat * 1e7), int(plon * 1e7), self.cruise)
        self._send_mission_current()

    def _send_mission_current(self):
        """Tell the world which waypoint we're actually flying to right now.

        This is the standard MISSION_CURRENT message every real autopilot
        emits as it sequences a mission. It matters here because these tours
        are multi-waypoint plans: without it the map can only draw the WHOLE
        mission, including the legs already flown, so the white flight-plan
        line trails a long spurious loop back to the start of the tour. With
        it the aggregator can trim the plan to the part still ahead."""
        self.conn.mav.mission_current_send(self.leg_idx)

    def _step_toward(self, tgt):
        """Move one tick toward tgt; return True when arrived."""
        d = distance_m(self.lat, self.lon, tgt[0], tgt[1])
        self.hdg = bearing_deg(self.lat, self.lon, tgt[0], tgt[1])
        step = SPEED_MS * TICK_S
        if d <= step or d < 1.0:
            self.lat, self.lon = tgt
            self.vx = self.vy = 0.0
            return True
        frac = step / d
        self.lat += (tgt[0] - self.lat) * frac
        self.lon += (tgt[1] - self.lon) * frac
        th = math.radians(self.hdg)
        self.vx = SPEED_MS * math.cos(th) * 100.0   # north, cm/s
        self.vy = SPEED_MS * math.sin(th) * 100.0   # east, cm/s
        return False

    def update(self, now):
        # Battery: drains while doing anything airborne, recharges at the depot
        # ("rest") as if the drone swapped/charged its pack between deliveries.
        if self.phase == "rest":
            self.battery = min(100.0, self.battery + 3.0)
        else:
            self.battery = max(10.0, self.battery - 0.06)

        if self.phase in ("to_delivery", "to_depot"):
            # Climb toward cruise altitude WHILE flying to the target (drones
            # don't sit still gaining height - they transit and climb together).
            if self.rel_alt < self.cruise:
                self.rel_alt = min(self.cruise, self.rel_alt + CLIMB_RATE * TICK_S)
                self.vz = -CLIMB_RATE * 100.0
            else:
                self.vz = 0.0
            # Fly vertex-by-vertex through the transit plan rather than
            # beelining to the final target, so the drone tracks the whole
            # shape. No pause at any vertex — only the last one is a delivery.
            leg_tgt = self.route[self.leg_idx]
            if self._step_toward(leg_tgt):
                if self.leg_idx < len(self.route) - 1:
                    self.leg_idx += 1          # next vertex, no pause
                    # Tell the map we've sequenced on, so the white flight-plan
                    # line drops the leg we just finished.
                    self._send_mission_current()
                else:
                    # End of the tour: THIS is the one and only delivery.
                    self.hover_until = now + HOVER_S
                    self.phase = "drop" if self.phase == "to_delivery" else "rest"
        elif self.phase == "drop":
            self.vz = 0.0
            if now >= self.hover_until:
                self.phase = "to_depot"
                self.route = return_route(self.company, self.route_idx, self.depot)
                self.leg_idx = 0
                self._send_route_mission()
        elif self.phase == "rest":
            self.vz = 0.0
            if now >= self.hover_until:
                # Fresh draw from THIS company's pool: this drone may pick up
                # the plan another same-company drone just flew, or repeat its own.
                self.route_idx = pick_route_idx(self.company, self.rng)
                self.target = delivery_point(self.company, self.route_idx)
                self.route = outbound_route(self.company, self.route_idx)
                self.leg_idx = 0
                self.phase = "to_delivery"
                self._send_route_mission()

        # Current destination for the flight-plan line: the delivery when
        # outbound, otherwise home to the depot.
        self.dest = self.target if self.phase == "to_delivery" else self.depot

    def send(self, boot_ms, send_hb):
        if send_hb:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                3,  # custom_mode: AUTO-ish (these fly delivery "missions")
                mavutil.mavlink.MAV_STATE_ACTIVE)

            # Battery report. Voltage tracks the % between ~10.5 V and 12.6 V (3S).
            bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
            volt_mv = int(10500 + (self.battery / 100.0) * (12600 - 10500))
            self.conn.mav.sys_status_send(
                bit, bit, bit,      # sensors present / enabled / healthy
                500,                # CPU load 50.0%
                volt_mv, -1,        # battery voltage (mV), current (unknown)
                int(self.battery),  # battery remaining %
                0, 0, 0, 0, 0, 0)

            # Current destination waypoint (drives the flight-plan line on the map).
            if self.dest:
                self.conn.mav.position_target_global_int_send(
                    boot_ms,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000110111111000,     # position only
                    int(self.dest[0] * 1e7), int(self.dest[1] * 1e7), self.cruise,
                    0, 0, 0, 0, 0, 0, 0, 0)

        self.conn.mav.global_position_int_send(
            boot_ms,
            int(self.lat * 1e7), int(self.lon * 1e7),
            int((FIELD_ALT_M + self.rel_alt) * 1000),  # alt AMSL (mm)
            int(self.rel_alt * 1000),                  # relative alt (mm)
            int(self.vx), int(self.vy), int(self.vz),
            int(self.hdg * 100) % 36000)

    def run(self, stop):
        t0 = time.time()
        last_hb = 0.0
        while not stop.is_set():
            now = time.time()
            self.update(now)
            send_hb = (now - last_hb) >= 1.0
            if send_hb:
                last_hb = now
            self.send(int((now - t0) * 1000), send_hb)
            time.sleep(TICK_S)


# ===========================================================================
# LAUNCHER: fly all the lightweight drones
# ===========================================================================
def main():
    sim_ids = [sid for sid, (_c, kind) in FLEET.items() if kind == "sim"]
    drones = [SimDrone(sid, FLEET[sid][0]) for sid in sim_ids]

    print(f"[lightweight_fleet] launching {len(drones)} lightweight drones -> UDP {INGEST_PORT}")
    for d in drones:
        print(f"   sysid {d.sysid:>2}  {d.company:<8} depot {d.depot} cruise {d.cruise:.0f}m")

    stop = threading.Event()
    threads = [threading.Thread(target=d.run, args=(stop,), daemon=True) for d in drones]
    for t in threads:
        t.start()
    print("[lightweight_fleet] flying. Ctrl+C to stop.")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()
        print("\n[lightweight_fleet] stopped.")


if __name__ == "__main__":
    main()

# ===========================================================================
# RUN NOTES
# ===========================================================================
# Lightweight fleet (this file):
#     python3 lightweight_fleet.py
#
# The 2 real SITL drones (separate terminals), homed at each company depot.
# We let MAVProxy run and add --out to the aggregator's ingest port, so the SITL
# drones land in the SAME UDP stream as the lightweight ones (one port, whole
# fleet). Each instance's own MAVProxy still serves an operator GCS too
# (instance 0 on 14550, instance 1 on 14560) for the two Mission Planners later.
#     sim_vehicle.py -v ArduCopter -I0 --sysid 1  --custom-location=33.1543,-96.8700,215,0 --out udp:127.0.0.1:14570
#     sim_vehicle.py -v ArduCopter -I1 --sysid 11 --custom-location=33.1180,-96.7720,215,0 --out udp:127.0.0.1:14570