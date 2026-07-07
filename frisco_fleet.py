"""
frisco_fleet.py
===============

Scenario geography + the LIGHTWEIGHT simulated drones for the Frisco UTM demo.

Two delivery companies operate over Frisco, TX:
  * Wing     - system IDs 1..6   (1 = real ArduPilot SITL, 2..6 = lightweight)
  * Zipline  - system IDs 11..16 (11 = real ArduPilot SITL, 12..16 = lightweight)

Running THIS file launches only the 10 LIGHTWEIGHT drones (Wing 2..6,
Zipline 12..16). They each fly depot -> delivery -> depot routes over real Frisco
landmarks and emit normal MAVLink (HEARTBEAT + GLOBAL_POSITION_INT) over UDP, so
the UTM aggregator (built next) cannot tell them apart from the real SITL drones.

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
# the UTM can show altitude separation between the two operators.
CRUISE_ALT_M = {"Wing": 80.0, "Zipline": 110.0}

# Company display colours (used everywhere via the JSON; single source of truth).
# Zipline is yellow so it doesn't clash with the red no-fly zones.
COMPANY_COLOR = {"Wing": "#1E88E5", "Zipline": "#FDD835"}  # blue / yellow

# Where the UTM map should centre its initial view (central Frisco).
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

# Delivery destinations spread across real Frisco landmarks + a few scattered
# residential points, so the fleet fans out across the whole suburb.
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

# No-fly zones (circles), placed JUST OUTSIDE the AO boundaries so a real drone can
# be flown into them by hand for the demo. Coordinates are illustrative Frisco spots.
NO_FLY_ZONES = [
    {"name": "Medical helipad", "lat": 33.1800, "lon": -96.8950, "radius_m": 800},
    {"name": "School zone",     "lat": 33.1150, "lon": -96.7520, "radius_m": 700},
]

# Altitude limits for the UTM (metres above ground). 400 ft ceiling, 10 m floor.
MAX_ALT_M = 122.0   # ~400 ft
MIN_ALT_M = 10.0

# Flight model parameters.
SPEED_MS = 15.0      # cruise ground speed
CLIMB_RATE = 4.0     # m/s while climbing to cruise
HOVER_S = 5.0        # pause at each delivery / depot (the "drop")
TICK_S = 0.25        # update + telemetry rate (4 Hz)

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
# THE LIGHTWEIGHT SIM DRONE
# ===========================================================================
class SimDrone:
    def __init__(self, sysid, company):
        self.sysid = sysid
        self.company = company
        self.depot = DEPOTS[company]
        self.cruise = CRUISE_ALT_M[company]
        self.rng = random.Random(sysid)   # deterministic-but-varied per drone

        # State.
        self.lat, self.lon = self.depot
        self.rel_alt = 0.0
        self.hdg = self.rng.uniform(0, 360)
        self.phase = "to_delivery"
        self.target = self._pick_delivery()
        self.hover_until = 0.0
        self.vx = self.vy = self.vz = 0.0   # cm/s for telemetry
        self.battery = 100.0                # % — drains flying, tops up at depot

        # Its own MAVLink link, stamped with this drone's system id.
        self.conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{INGEST_PORT}",
            source_system=sysid, source_component=1, dialect="ardupilotmega")

    def _pick_delivery(self):
        return self.rng.choice(DELIVERY_POINTS)

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
            tgt = self.target if self.phase == "to_delivery" else self.depot
            if self._step_toward(tgt):
                self.hover_until = now + HOVER_S
                self.phase = "drop" if self.phase == "to_delivery" else "rest"
        elif self.phase == "drop":
            self.vz = 0.0
            if now >= self.hover_until:
                self.phase = "to_depot"
        elif self.phase == "rest":
            self.vz = 0.0
            if now >= self.hover_until:
                self.target = self._pick_delivery()
                self.phase = "to_delivery"

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

    print(f"[frisco] launching {len(drones)} lightweight drones -> UDP {INGEST_PORT}")
    for d in drones:
        print(f"   sysid {d.sysid:>2}  {d.company:<8} depot {d.depot} cruise {d.cruise:.0f}m")

    stop = threading.Event()
    threads = [threading.Thread(target=d.run, args=(stop,), daemon=True) for d in drones]
    for t in threads:
        t.start()
    print("[frisco] flying. Ctrl+C to stop.")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()
        print("\n[frisco] stopped.")


if __name__ == "__main__":
    main()

# ===========================================================================
# RUN NOTES
# ===========================================================================
# Lightweight fleet (this file):
#     python3 frisco_fleet.py
#
# The 2 real SITL drones (separate terminals), homed at each company depot.
# We let MAVProxy run and add --out to the aggregator's ingest port, so the SITL
# drones land in the SAME UDP stream as the lightweight ones (one port, whole
# fleet). Each instance's own MAVProxy still serves an operator GCS too
# (instance 0 on 14550, instance 1 on 14560) for the two Mission Planners later.
#     sim_vehicle.py -v ArduCopter -I0 --sysid 1  --custom-location=33.1543,-96.8700,215,0 --out udp:127.0.0.1:14570
#     sim_vehicle.py -v ArduCopter -I1 --sysid 11 --custom-location=33.1180,-96.7720,215,0 --out udp:127.0.0.1:14570
