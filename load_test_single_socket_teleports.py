"""
load_test_multisocket.py
================

Controlled MAVLink telemetry load generator for testing aggregator.py.

Usage:
    python3 utm_load_test.py

Make sure:
    - aggregator.py is running
    - UDP ingest port matches lightweight_fleet.INGEST_PORT
"""

import math
import threading
import time
import argparse
import random

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DEFAULT_TARGET = "udpout:127.0.0.1:14570"

DEFAULT_DRONES = 50
DEFAULT_RATE_HZ = 5

CENTER_LAT = 33.125
CENTER_LON = -96.772

RADIUS_M = 15000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

EARTH_R = 6371000.0


def offset_position(lat, lon, north_m, east_m):
    #Convert meter offsets to GPS coordinates.

    d_lat = north_m / EARTH_R
    d_lon = east_m / (EARTH_R * math.cos(math.radians(lat)))

    return (
        lat + math.degrees(d_lat),
        lon + math.degrees(d_lon),
    )


def drone_position(drone_id, elapsed):
    # Teleport to a random point within RADIUS_M of the center.

    angle = random.uniform(0, 2 * math.pi)

    # Uniform distribution over the circle
    radius = RADIUS_M * math.sqrt(random.random())

    north = math.sin(angle) * radius
    east = math.cos(angle) * radius

    return offset_position(
        CENTER_LAT,
        CENTER_LON,
        north,
        east,
    )


# ---------------------------------------------------------------------------
# Drone simulator
# ---------------------------------------------------------------------------

class SimDrone:

    def __init__(self, sysid, connection, rate_hz):
        self.sysid = sysid
        self.master = connection

        self.period = 1.0 / rate_hz
        self.start_time = time.monotonic()
        self.next_send = self.start_time

    def send(self):

        boot_ms = int((time.monotonic() - self.start_time) * 1000)

        lat, lon = drone_position(
            self.sysid,
            boot_ms,
        )

        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            0,
        )

        self.master.mav.global_position_int_send(
            boot_ms,
            int(lat * 1e7),
            int(lon * 1e7),
            30000,
            30000,
            0,
            0,
            0,
            9000,
        )

        self.master.mav.vfr_hud_send(
            10.0,
            10.0,
            90,
            0,
            30,
            0,
        )

        self.master.mav.sys_status_send(
            0,
            0,
            0,
            0,
            12000,
            -1,
            75,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self.next_send += self.period


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--drones",
        type=int,
        default=DEFAULT_DRONES,
        help="number of simulated drones"
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="telemetry updates //s per drone"
    )

    parser.add_argument(
        "--target",
        default=DEFAULT_TARGET,
        help="MAVLink UDP destination"
    )

    args = parser.parse_args()


    print(
        f"""
UTM load test starting

Drones : {args.drones}
Rate   : {args.rate} Hz
Target : {args.target}

Press Ctrl+C to stop.
"""
    )

drones = []

for i in range(args.drones):

    conn = mavutil.mavlink_connection(
        args.target,
        source_system=i,
    )

    drones.append(
        SimDrone(
            i,
            conn,
            args.rate,
        )
    )

last_status = time.monotonic()

while True:

    now = time.monotonic()

    next_deadline = now + 1

    for drone in drones:

        if now >= drone.next_send:
            drone.send()

        if drone.next_send < next_deadline:
            next_deadline = drone.next_send

    if now - last_status >= 5:
        print(f"[load] running {len(drones)} simulated drones")
        last_status = now

    sleep_time = next_deadline - time.monotonic()

    if sleep_time > 0:
        time.sleep(min(sleep_time, 0.01))


if __name__ == "__main__":
    main()