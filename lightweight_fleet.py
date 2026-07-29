"""
lightweight_fleet.py
===============

Scenario geography + the LIGHTWEIGHT simulated drones for the GridEye demo.

Two delivery companies operate over Frisco, TX:
  * Wing     - system IDs 1..6   (1 = real ArduPilot SITL, 2..6 = lightweight)
  * Zipline  - system IDs 11..16 (11 = real ArduPilot SITL, 12..16 = lightweight)

Running THIS file launches only the 10 LIGHTWEIGHT drones (Wing 2..6,
Zipline 12..16). They each fly depot -> delivery -> depot over real Frisco
landmarks along a rectangular survey plan drawn at random from the shared
ROUTE_LIBRARY — the waypoints themselves are frozen, but WHICH tour a drone
flies is re-rolled every cycle, so drones swap routes and two of them can be
on the same tour at once. A drone drops exactly once per trip, on reaching
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

# Delivery destinations spread across real Frisco landmarks + a bunch of
# scattered residential points, so the fleet fans out across the whole suburb.
DELIVERY_POINTS = [
    (33.1104, -96.8280),  # The Star
    (33.0991, -96.8114),  # Stonebriar Centre
    (33.1509, -96.8318),  # Frisco Square
    (33.2130, -96.8494),  # Omni PGA Resort (north)
    (33.1543, -96.8352),  # Toyota Stadium
    (33.1820, -96.8050),  # scattered NE
    (33.1300, -96.8600),  # scattered SW
    (33.1230, -96.8050),  # scattered SE
    (33.1700, -96.8250),  # scattered N-central
    (33.1400, -96.7950),  # scattered E
    (33.1920, -96.8650),  # scattered NW
    (33.0930, -96.8420),  # scattered S-central
    (33.1650, -96.7850),  # scattered NE-2
    (33.1050, -96.7920),  # scattered SE-2
    (33.2000, -96.8150),  # scattered N
    (33.1250, -96.8900),  # scattered W
]

# ---------------------------------------------------------------------------
# SHARED ROUTE LIBRARY  (the pool every drone draws from)
# ---------------------------------------------------------------------------
# One pool of frozen delivery plans, NOT one route per drone. Every drone picks
# a route from this list at random each time it leaves its depot, so the
# assignment is randomised and any two drones may end up flying the same plan
# (or swap plans between cycles) -- exactly like a real dispatcher handing out
# whatever job is next off the queue.
#
# Each entry is the list of waypoints AFTER the depot:
#
#     [ staging, elbow, box_v1 ... box_vN, delivery point ]
#
# These are REALISTIC RECTANGULAR flight plans, the shape a ground-control
# station actually generates: a straight transit corridor out of shared
# airspace (staging point, then one grid-aligned dog-leg / "elbow"), followed
# by an axis-aligned boustrophedon "lawnmower" survey box over the delivery
# neighbourhood -- parallel legs joined by 90-degree turns -- finishing on the
# drop-off LAST. Frisco's street grid is close to N-S / E-W aligned, so the
# right-angle routing is also what keeps a drone over legal right-of-way. The
# drone turns at every vertex and does not stop at any of them -- it hovers and
# drops exactly once, on reaching that final waypoint. The trip home is the
# same plan flown in reverse.
#
# Because any drone can draw any route, every vertex lies inside BOTH
# companies' areas of operation and every plan ends on a delivery point that
# is valid for both, so a Wing drone flying a route a Zipline drone flew last
# cycle never leaves its own airspace. The depot connector legs are checked
# per company. These were generated and validated by gen_pool.py.
ROUTE_LIBRARY = [
    [  # route 0 — 10 vertices, 9.6 km, delivers at (33.1104, -96.828)
        (33.1348, -96.8264), (33.1189, -96.8264), (33.1189, -96.8221),
        (33.1189, -96.8339), (33.1064, -96.8339), (33.1064, -96.8221),
        (33.1105, -96.8221), (33.1105, -96.8339), (33.1147, -96.8339),
        (33.1147, -96.8221), (33.1104, -96.828),
    ],
    [  # route 1 — 10 vertices, 9.8 km, delivers at (33.0991, -96.8114)
        (33.1297, -96.8189), (33.1076, -96.8189), (33.1076, -96.8173),
        (33.0951, -96.8173), (33.0951, -96.8055), (33.0992, -96.8055),
        (33.0992, -96.8173), (33.1034, -96.8173), (33.1034, -96.8055),
        (33.1076, -96.8055), (33.0991, -96.8114),
    ],
    [  # route 2 — 10 vertices, 7.9 km, delivers at (33.1509, -96.8318)
        (33.153, -96.8281), (33.151, -96.8281), (33.151, -96.8259),
        (33.151, -96.8377), (33.1552, -96.8377), (33.1552, -96.8259),
        (33.1594, -96.8259), (33.1594, -96.8377), (33.1469, -96.8377),
        (33.1469, -96.8259), (33.1509, -96.8318),
    ],
    [  # route 3 — 10 vertices, 10.6 km, delivers at (33.213, -96.8494)
        (33.1809, -96.836), (33.209, -96.836), (33.209, -96.8435),
        (33.2131, -96.8435), (33.2131, -96.8553), (33.2173, -96.8553),
        (33.2173, -96.8435), (33.2215, -96.8435), (33.2215, -96.8553),
        (33.209, -96.8553), (33.213, -96.8494),
    ],
    [  # route 4 — 10 vertices, 7.5 km, delivers at (33.1543, -96.8352)
        (33.1545, -96.8296), (33.1544, -96.8296), (33.1544, -96.8293),
        (33.1544, -96.8411), (33.1586, -96.8411), (33.1586, -96.8293),
        (33.1628, -96.8293), (33.1628, -96.8411), (33.1503, -96.8411),
        (33.1503, -96.8293), (33.1543, -96.8352),
    ],
    [  # route 5 — 10 vertices, 8.6 km, delivers at (33.182, -96.805)
        (33.167, -96.8161), (33.178, -96.8161), (33.178, -96.8109),
        (33.178, -96.7991), (33.1821, -96.7991), (33.1821, -96.8109),
        (33.1863, -96.8109), (33.1863, -96.7991), (33.1905, -96.7991),
        (33.1905, -96.8109), (33.182, -96.805),
    ],
    [  # route 6 — 10 vertices, 4.7 km, delivers at (33.13, -96.86)
        (33.1436, -96.8408), (33.1337, -96.8408), (33.1337, -96.8586),
        (33.1337, -96.8614), (33.1308, -96.8614), (33.1308, -96.8586),
        (33.1318, -96.8586), (33.1318, -96.8614), (33.1327, -96.8614),
        (33.1327, -96.8586), (33.13, -96.86),
    ],
    [  # route 7 — 10 vertices, 8.7 km, delivers at (33.123, -96.805)
        (33.1404, -96.8161), (33.1315, -96.8161), (33.1315, -96.8109),
        (33.119, -96.8109), (33.119, -96.7991), (33.1231, -96.7991),
        (33.1231, -96.8109), (33.1273, -96.8109), (33.1273, -96.7991),
        (33.1315, -96.7991), (33.123, -96.805),
    ],
    [  # route 8 — 10 vertices, 7.9 km, delivers at (33.17, -96.825)
        (33.1616, -96.8251), (33.166, -96.8251), (33.166, -96.8309),
        (33.166, -96.8191), (33.1701, -96.8191), (33.1701, -96.8309),
        (33.1743, -96.8309), (33.1743, -96.8191), (33.1785, -96.8191),
        (33.1785, -96.8309), (33.17, -96.825),
    ],
    [  # route 9 — 10 vertices, 8.2 km, delivers at (33.14, -96.795)
        (33.1481, -96.8116), (33.1485, -96.8116), (33.1485, -96.8009),
        (33.136, -96.8009), (33.136, -96.7891), (33.1401, -96.7891),
        (33.1401, -96.8009), (33.1443, -96.8009), (33.1443, -96.7891),
        (33.1485, -96.7891), (33.14, -96.795),
    ],
    [  # route 10 — 10 vertices, 10.6 km, delivers at (33.093, -96.842)
        (33.1269, -96.8327), (33.1015, -96.8327), (33.1015, -96.8361),
        (33.1015, -96.8479), (33.089, -96.8479), (33.089, -96.8361),
        (33.0931, -96.8361), (33.0931, -96.8479), (33.0973, -96.8479),
        (33.0973, -96.8361), (33.093, -96.842),
    ],
    [  # route 11 — 10 vertices, 7.7 km, delivers at (33.105, -96.792)
        (33.1323, -96.8102), (33.1105, -96.8102), (33.1105, -96.7951),
        (33.104, -96.7951), (33.104, -96.7889), (33.1062, -96.7889),
        (33.1062, -96.7951), (33.1083, -96.7951), (33.1083, -96.7889),
        (33.1105, -96.7889), (33.105, -96.792),
    ],
    [  # route 12 — 10 vertices, 9.2 km, delivers at (33.2, -96.815)
        (33.1751, -96.8206), (33.196, -96.8206), (33.196, -96.8209),
        (33.196, -96.8091), (33.2001, -96.8091), (33.2001, -96.8209),
        (33.2043, -96.8209), (33.2043, -96.8091), (33.2085, -96.8091),
        (33.2085, -96.8209), (33.2, -96.815),
    ],
]

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
# ROUTE LOOKUP  (keyed on a POOL INDEX, not on sysid)
# ===========================================================================
# Routes belong to the fleet, not to any one drone: a route index is just a
# ticket into ROUTE_LIBRARY, and two drones holding the same ticket fly the
# same tour. Keeping the index (rather than a copy of the waypoints) in each
# drone's state is what lets the aggregator, the map and ui_selftest all talk
# about "route 7" and mean the same thing.
def route_count():
    return len(ROUTE_LIBRARY)


def pick_route_idx(rng):
    """Draw a route at random from the shared pool. No exclusion between
    drones — collisions are the point: two drones CAN be on the same tour."""
    return rng.randrange(len(ROUTE_LIBRARY))


def delivery_point(idx):
    """The single drop-off for this route: its last waypoint."""
    return ROUTE_LIBRARY[idx][-1]


def outbound_route(idx):
    """Depot -> delivery: all 16 tour vertices, then the delivery point last.
    The caller flies these in order and drops only at the final one."""
    return list(ROUTE_LIBRARY[idx])


def return_route(idx, depot):
    """Delivery -> depot: the tour flown in reverse, finishing at the depot."""
    return list(reversed(ROUTE_LIBRARY[idx][:-1])) + [tuple(depot)]


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
        self.route_idx = pick_route_idx(self.rng)
        self.target = delivery_point(self.route_idx)   # this tour's single drop-off
        self.dest = self.target          # where it's currently headed (for the flight plan)
        self.route = outbound_route(self.route_idx)
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
        rectangular plan instead of a straight guess at the final target."""
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
            # Fly vertex-by-vertex through the rectangular plan rather than
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
                self.route = return_route(self.route_idx, self.depot)
                self.leg_idx = 0
                self._send_route_mission()
        elif self.phase == "rest":
            self.vz = 0.0
            if now >= self.hover_until:
                # Fresh draw from the shared pool: this drone may pick up the
                # tour another drone just flew, or repeat its own.
                self.route_idx = pick_route_idx(self.rng)
                self.target = delivery_point(self.route_idx)
                self.route = outbound_route(self.route_idx)
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