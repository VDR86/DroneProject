"""
test_guardian.py — in-process unit tests for utm_guardian.Guardian.

No sockets, no MAVLink, no spoofer: each test hand-feeds parsed telemetry
values to observe() and asserts assess() reacts. Run:  python3 test_guardian.py
"""

from utm_guardian import Guardian

KNOWN = {1, 2, 3, 11, 12}


def codes_for(g, sid, now):
    return set(g.assess(now).get(sid, {}))


def test_clean_traffic_is_silent():
    g = Guardian(KNOWN)
    t = 1000.0
    lat, lon, alt = 33.15, -96.87, 80.0
    for _ in range(10):
        # 15 m/s north ~ 1.35e-4 deg/tick at 0.25 s; report the matching speed.
        lat += 15.0 * 0.25 / 111_320.0
        t += 0.25
        g.observe(2, "GLOBAL_POSITION_INT", t=t, lat=lat, lon=lon,
                  alt_m=alt, speed_ms=15.0, src=("127.0.0.1", 5001))
    assert codes_for(g, 2, t) == set(), codes_for(g, 2, t)


def test_ghost_id():
    g = Guardian(KNOWN)
    g.observe(99, "GLOBAL_POSITION_INT", t=1.0, lat=33.1, lon=-96.8,
              alt_m=50, speed_ms=0, src=("127.0.0.1", 6000))
    assert "GHOST_ID" in codes_for(g, 99, 1.0)


def test_src_conflict():
    g = Guardian(KNOWN)
    g.observe(1, "GLOBAL_POSITION_INT", t=1.0, lat=33.1, lon=-96.8,
              alt_m=80, speed_ms=10, src=("127.0.0.1", 5760))
    g.observe(1, "GLOBAL_POSITION_INT", t=1.1, lat=33.1, lon=-96.8,
              alt_m=80, speed_ms=10, src=("10.0.0.5", 40000))  # 2nd origin
    assert "SRC_CONFLICT" in codes_for(g, 1, 1.2)


def test_teleport():
    g = Guardian(KNOWN)
    g.observe(2, "GLOBAL_POSITION_INT", t=1.0, lat=33.10, lon=-96.80,
              alt_m=80, speed_ms=15, src=None)
    # ~1.1 km jump in 0.25 s -> thousands of m/s
    g.observe(2, "GLOBAL_POSITION_INT", t=1.25, lat=33.11, lon=-96.80,
              alt_m=80, speed_ms=15, src=None)
    assert "TELEPORT" in codes_for(g, 2, 1.3)


def test_climb_rate():
    g = Guardian(KNOWN)
    g.observe(2, "GLOBAL_POSITION_INT", t=1.0, lat=33.10, lon=-96.80,
              alt_m=80, speed_ms=0, src=None)
    g.observe(2, "GLOBAL_POSITION_INT", t=1.25, lat=33.10, lon=-96.80,
              alt_m=140, speed_ms=0, src=None)  # +60 m in 0.25 s = 240 m/s
    assert "CLIMB_RATE" in codes_for(g, 2, 1.3)


def test_vel_mismatch_needs_sustain():
    g = Guardian(KNOWN, vel_sustain=3)
    lat, lon, t = 33.10, -96.80, 1.0
    # Aircraft actually stationary, but telemetry claims 20 m/s.
    fired = None
    for i in range(5):
        t += 0.25
        g.observe(2, "GLOBAL_POSITION_INT", t=t, lat=lat, lon=lon,
                  alt_m=80, speed_ms=20.0, src=None)
        if fired is None and "VEL_MISMATCH" in codes_for(g, 2, t):
            fired = i
    assert fired is not None and fired >= 2, fired  # only after the streak


def test_fleet_guided_takeover():
    g = Guardian(KNOWN, guided_fleet_n=3, guided_window_s=3.0)
    # three drones flip to GUIDED within ~0.4 s of each other
    g.observe(1, "HEARTBEAT", t=100.0, mode="AUTO")
    g.observe(2, "HEARTBEAT", t=100.0, mode="AUTO")
    g.observe(3, "HEARTBEAT", t=100.0, mode="AUTO")
    g.observe(1, "HEARTBEAT", t=200.0, mode="GUIDED")
    g.observe(2, "HEARTBEAT", t=200.2, mode="GUIDED")
    g.observe(3, "HEARTBEAT", t=200.4, mode="GUIDED")
    for sid in (1, 2, 3):
        assert "FLEET_GUIDED" in codes_for(g, sid, 201.0), sid


def test_single_guided_is_not_takeover():
    g = Guardian(KNOWN, guided_fleet_n=3)
    g.observe(1, "HEARTBEAT", t=100.0, mode="AUTO")
    g.observe(1, "HEARTBEAT", t=200.0, mode="GUIDED")  # one operator, legit
    assert "FLEET_GUIDED" not in codes_for(g, 1, 201.0)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            print(f"FAIL {fn.__name__}: {e}")
        else:
            passed += 1
            print(f"ok   {fn.__name__}")
    print(f"\n{passed}/{len(tests)} passed")
