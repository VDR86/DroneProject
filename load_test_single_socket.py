import time
import math
import random
import argparse

from pymavlink import mavutil


DEFAULT_TARGET = "udpout:127.0.0.1:14570"

DEFAULT_DRONES = 50
DEFAULT_RATE_HZ = 5

CENTER_LAT = 33.125
CENTER_LON = -96.772

RADIUS_M = 15000

EARTH_R = 6371000.0


def offset_position(lat, lon, north, east):

    return (
        lat + math.degrees(north / EARTH_R),
        lon + math.degrees(
            east /
            (EARTH_R * math.cos(math.radians(lat)))
        )
    )


def make_positions(count):

    """
    Generate fixed tracks once.

    No random calculations during the test loop.
    """

    positions = []

    for _ in range(count):

        angle = random.random() * math.tau
        radius = RADIUS_M * math.sqrt(
            random.random()
        )

        north = math.sin(angle) * radius
        east = math.cos(angle) * radius

        positions.append(
            offset_position(
                CENTER_LAT,
                CENTER_LON,
                north,
                east
            )
        )

    return positions


def send_heartbeat(master):

    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        0
    )


def send_position(master, lat, lon, boot_ms):

    master.mav.global_position_int_send(
        boot_ms,
        int(lat * 1e7),
        int(lon * 1e7),
        30000,
        30000,
        0,
        0,
        0,
        9000
    )


def send_vfr(master):

    master.mav.vfr_hud_send(
        10,
        10,
        90,
        0,
        30,
        0
    )


def send_battery(master):

    master.mav.sys_status_send(
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
        0
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--drones",
        type=int,
        default=DEFAULT_DRONES
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ
    )

    parser.add_argument(
        "--target",
        default=DEFAULT_TARGET
    )

    args = parser.parse_args()


    print(
        f"""
UTM optimized load test

Drones : {args.drones}
Rate   : {args.rate} Hz
Target : {args.target}

"""
    )


    # One socket instead of N sockets
    master = mavutil.mavlink_connection(
        args.target,
        source_system=255
    )


    positions = make_positions(
        args.drones
    )


    period = 1 / args.rate


    next_tick = time.monotonic()


    boot = 0


    while True:

        now = time.monotonic()

        if now >= next_tick:

            boot += int(period * 1000)


            for sid, (lat, lon) in enumerate(
                positions
            ):

                master.mav.srcSystem = sid


                send_heartbeat(
                    master
                )

                send_position(
                    master,
                    lat,
                    lon,
                    boot
                )

                send_vfr(
                    master
                )

                send_battery(
                    master
                )


            next_tick += period


        else:

            time.sleep(
                0.001
            )


if __name__ == "__main__":
    main()