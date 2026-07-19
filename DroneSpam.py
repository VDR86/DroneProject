"""
utm_load_test.py
================

Controlled MAVLink telemetry load generator for testing utm_aggregator.py.

Usage:
    python3 utm_load_test.py

Make sure:
    - utm_aggregator.py is running
    - UDP ingest port matches frisco_fleet.INGEST_PORT
"""

import math
import threading
import time
import argparse

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DEFAULT_TARGET = "udpout:127.0.0.1:14550"

DEFAULT_DRONES = 50
DEFAULT_RATE_HZ = 5

CENTER_LAT = 33.125
CENTER_LON = -96.772

RADIUS_M = 300


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
    #Generate repeatable circular flight paths.
    

    angle = elapsed * 0.15 + drone_id

    north = math.sin(angle) * RADIUS_M
    east = math.cos(angle) * RADIUS_M

    return offset_position(
        CENTER_LAT,
        CENTER_LON,
        north,
        east,
    )


# ---------------------------------------------------------------------------
# Drone simulator
# ---------------------------------------------------------------------------

class SimDrone(threading.Thread):

    def __init__(self, sysid, connection, rate_hz, stop):
        super().__init__(daemon=True)

        self.sysid = sysid
        self.master = connection
        self.rate = rate_hz
        self.stop = stop

        self.start_time = time.time()


    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            0,
        )


    def send_position(self):

        now = time.time() - self.start_time

        lat, lon = drone_position(
            self.sysid,
            now
        )

        alt_mm = 30000       # 30m
        rel_alt_mm = 30000

        self.master.mav.global_position_int_send(
            int(time.time() * 1000),
            int(lat * 1e7),
            int(lon * 1e7),
            alt_mm,
            rel_alt_mm,
            0,
            0,
            0,
            9000,
        )


    def send_vfr(self):

        self.master.mav.vfr_hud_send(
            10.0,
            10.0,
            90,
            0,
            30,
            0
        )


    def send_battery(self):

        self.master.mav.sys_status_send(
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            75,
            0,
            0
        )


    def run(self):

        period = 1.0 / self.rate
        next_send = time.time()

        while not self.stop.is_set():

            now = time.time()

            if now >= next_send:

                self.send_heartbeat()
                self.send_position()
                self.send_vfr()
                self.send_battery()

                next_send += period

            time.sleep(0.001)


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


    stop = threading.Event()

    drones = []

    try:

        for i in range(args.drones):

            sysid = i + 10

            conn = mavutil.mavlink_connection(
                args.target,
                source_system=sysid
            )

            drone = SimDrone(
                sysid,
                conn,
                args.rate,
                stop
            )

            drone.start()

            drones.append(drone)


        while True:
            time.sleep(5)

            print(
                f"[load] running {len(drones)} simulated drones"
            )


    except KeyboardInterrupt:

        print("\nStopping load test...")


    finally:

        stop.set()

        for d in drones:
            d.join(timeout=1)

        print("Done.")


if __name__ == "__main__":
    main()