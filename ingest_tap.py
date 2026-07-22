"""
ingest_tap.py
=============

A passive, READ-ONLY observer for the Frisco UTM ingest stream.

It binds the same UDP port the aggregator listens on and prints what arrives:
the network source, the MAVLink system id, the message type, and key fields.
Its purpose is to answer one question — *does a given telemetry source actually
reach the monitor's ingest port?* — which is the coverage question worth
checking after a red-team run.

The tell to watch for: a NEW source address appearing under a system id that
already exists (two origins for one drone) is the on-the-wire signature of
injection/impersonation. This tool prints that the instant it happens.

What it does NOT do
-------------------
It never builds or sends a MAVLink message, never commands a vehicle, never
modifies the stream. It only receives and prints — a wiretap on your own
loopback, nothing more.

Port sharing
------------
UDP unicast to 127.0.0.1:<port> is delivered to one listener, so run this
INSTEAD of utm_aggregator.py (they would otherwise compete for datagrams). The
lightweight fleet and any SITL --out senders can keep running — they only send;
this only receives. Typical use:

    1. stop utm_aggregator.py
    2. py ingest_tap.py
    3. start the fleet (and/or your red-team run) and watch what lands here

Run:  py ingest_tap.py
"""

import math
import os
import socket
import time
from collections import defaultdict

os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

import frisco_fleet as ff  # only for INGEST_PORT + the sysid->company registry

REFRESH_S = 2.0  # how often the summary table is reprinted

# ArduCopter custom_mode -> readable name (matches the aggregator's table).
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    16: "POSHOLD", 17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}


class Track:
    """Everything observed about one system id."""
    def __init__(self):
        self.sources = {}     # (host, port) -> [count, last_t]
        self.msgs = 0
        self.mode = None
        self.pos = None       # (lat, lon, alt_m)
        self.last_seen = 0.0


def fmt_src(src):
    return f"{src[0]}:{src[1]}"


def main():
    # Raw socket so we always get the datagram source (see the aggregator note:
    # pymavlink's udpin connection doesn't expose it on this version).
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", ff.INGEST_PORT))
    sock.settimeout(0.5)
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True

    print(f"[tap] passive listener on UDP {ff.INGEST_PORT} — receive-only")
    print("[tap] a NEW source on an existing system id = possible injection\n")

    tracks = defaultdict(Track)
    seen_pairs = set()   # (sysid, src) already announced
    next_print = time.time() + REFRESH_S

    while True:
        try:
            data, src = sock.recvfrom(65535)
        except (socket.timeout, OSError):
            data = None
        now = time.time()

        if data:
            try:
                msgs = parser.parse_buffer(data) or []
            except mavutil.mavlink.MAVError:
                msgs = []
            for msg in msgs:
                mtype = msg.get_type()
                if mtype == "BAD_DATA":
                    continue
                sid = msg.get_srcSystem()
                t = tracks[sid]
                t.msgs += 1
                t.last_seen = now

                if src not in t.sources:
                    t.sources[src] = [0, now]
                    if (sid, src) not in seen_pairs:
                        seen_pairs.add((sid, src))
                        who = ff.FLEET.get(sid, ("Unknown", "?"))[0]
                        tag = "" if sid in ff.FLEET else "  [UNKNOWN SYSID]"
                        extra = ("  [SECOND SOURCE FOR THIS ID]"
                                 if len(t.sources) > 1 else "")
                        print(f"[tap] {time.strftime('%H:%M:%S')}  new source "
                              f"{fmt_src(src)}  ->  sysid {sid} ({who})"
                              f"{tag}{extra}")
                t.sources[src][0] += 1
                t.sources[src][1] = now

                if mtype == "GLOBAL_POSITION_INT":
                    t.pos = (msg.lat / 1e7, msg.lon / 1e7,
                             round(msg.relative_alt / 1000.0, 1))
                elif mtype == "HEARTBEAT":
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                        t.mode = COPTER_MODES.get(msg.custom_mode,
                                                  f"MODE{msg.custom_mode}")

        if now >= next_print:
            next_print = now + REFRESH_S
            print("\n" + "=" * 82)
            print(f"  ingest snapshot @ {time.strftime('%H:%M:%S')}   "
                  f"(port {ff.INGEST_PORT}, receive-only)")
            print("-" * 82)
            print(f"  {'SYSID':<6}{'WHO':<9}{'SOURCE(S)':<26}{'MODE':<8}"
                  f"{'LAST POS':<18}FLAGS")
            for sid in sorted(tracks):
                t = tracks[sid]
                if now - t.last_seen > 6:
                    continue
                who = ff.FLEET.get(sid, ("Unknown", "?"))[0]
                srcs = ",".join(str(s[1]) for s in t.sources)  # show ports
                if len(srcs) > 24:
                    srcs = srcs[:23] + "…"
                mode = t.mode or "—"
                pos = f"{t.pos[0]:.4f},{t.pos[1]:.4f}" if t.pos else "—"
                flags = []
                if sid not in ff.FLEET:
                    flags.append("UNKNOWN-ID")
                if len(t.sources) > 1:
                    flags.append(f"{len(t.sources)}-SOURCES")
                print(f"  {sid:<6}{who:<9}{srcs:<26}{mode:<8}{pos:<18}"
                      f"{' '.join(flags)}")
            print("=" * 82)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[tap] stopped.")
