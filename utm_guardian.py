"""
utm_guardian.py
===============

Attack-resistance layer for the Frisco UTM aggregator.

Purpose
-------
The aggregator's ingest is a single, plain-UDP MAVLink stream keyed only by
system id. Nothing about that stream is authenticated, so a third party who can
reach the port could:

  * inject fixes for a system id that already exists (impersonation),
  * invent a "ghost" aircraft that was never launched,
  * feed a smooth, plausible track while the real vehicle flies elsewhere, or
  * flip a group of drones into GUIDED at one synchronized instant (takeover).

This module watches the very same telemetry the aggregator already consumes and
raises an alert whenever the data is internally inconsistent (physics it can't
be) or has a suspicious provenance (arriving from the wrong place, or from two
places at once).

Scope
-----
DEFENSIVE ONLY. The Guardian consumes telemetry the aggregator already receives
and returns alert strings. It never transmits, never commands a vehicle, and
contains no knowledge of how an attack is produced. It is a smoke detector, not a
fire.

Design notes
------------
The Guardian is deliberately decoupled from pymavlink and from frisco_fleet so
it can be unit-tested with plain Python values (see test_guardian.py). The
aggregator parses each MAVLink message, hands the fields to `observe()`, and
later folds `assess()` into its own alert table.

Detects nine things: GHOST_ID, SRC_CONFLICT, TELEPORT, CLIMB_RATE,
VEL_MISMATCH, FLEET_GUIDED, SRC_HANDOFF (a source change after the previous
one had already gone quiet), GPS_ATTACK (reported heading inconsistent with the
movement vector implied by consecutive position fixes — the fingerprint of an
injected false position), and OUT_AO (reported position outside the drone's
company area of operations, when ao_polygons/ao_sysid_company are supplied).

Two-phase flow, mirroring the aggregator's own ingest/evaluate split:
  observe()  -- called on every message; cheap; just records history
  assess()   -- called on a timer; runs the detectors over recent history
"""

from collections import defaultdict, deque
import math

# Every security message starts with this token so it is visually distinct from
# the aggregator's flight-rule alerts (altitude, no-fly, area) in the same UI.
SEC_PREFIX = "ATTACK"


# ---------------------------------------------------------------------------
# Geometry helper (self-contained so this module has no project dependencies)
# ---------------------------------------------------------------------------
def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two lat/lon points, in metres."""
    r = 6_371_000.0  # mean Earth radius
    p1, p2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(d_lam / 2) ** 2)
    return 2 * r * math.asin(math.sqrt(a))


def _bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees, 0–360) from point 1 to point 2."""
    d_lon = math.radians(lon2 - lon1)
    y = math.sin(d_lon) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.cos(d_lon))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _angle_diff(a, b):
    """Signed shortest angular difference (a − b) in degrees, in (−180, 180]."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def _point_in_poly(lat, lon, poly):
    """Ray-casting point-in-polygon test. poly is [(lat, lon), ...]."""
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


class Guardian:
    """Stateful attack detector for one UTM aggregator.

    Feed it parsed telemetry via observe(); poll it via assess(). It keeps a
    short rolling history per drone and derives every judgement from that
    history plus the fleet registry it was constructed with.
    """

    def __init__(
        self,
        known_sysids,
        *,
        # --- kinematic limits -------------------------------------------------
        max_speed_ms=45.0,    # Ground speed a fix-to-fix jump may imply before
                              # it's deemed impossible. The sim fleet cruises at
                              # 15 m/s; 45 leaves generous room for real SITL
                              # manoeuvres while still catching teleport jumps
                              # (those imply hundreds/thousands of m/s).
        max_climb_ms=12.0,    # Max believable vertical rate. Sim climbs at
                              # 4 m/s; 12 covers aggressive SITL climbs.
        # --- velocity cross-check --------------------------------------------
        vel_tol_ms=6.0,       # Absolute slack when comparing reported speed to
                              # track-derived speed.
        vel_tol_frac=0.6,     # Fractional slack for the same comparison; the
                              # larger of the two tolerances wins.
        vel_sustain=3,        # Consecutive disagreeing samples required before
                              # VEL_MISMATCH fires (suppresses one-off noise).
        # --- provenance -------------------------------------------------------
        src_window_s=6.0,     # How long a network source stays "current" for
                              # the two-sources-at-once check.
        handoff_hold_s=5.0,   # How long a SRC_HANDOFF alert stays "current"
                              # after it's detected, so the aggregator's
                              # edge-triggered enter/clear logic can see it.
        # --- takeover ---------------------------------------------------------
        guided_window_s=3.0,  # Cluster width for the synchronized-GUIDED test.
        guided_fleet_n=3,     # This many near-simultaneous GUIDED switches
                              # inside guided_window_s counts as a takeover.
        # --- GPS attack / heading consistency ---------------------------------
        hdg_tol_deg=60.0,     # Max acceptable angle between the drone's reported
                              # heading and the bearing implied by consecutive
                              # position fixes. Random GPS scatter and gentle turns
                              # can produce short disagreements; we require a
                              # sustained streak (hdg_sustain samples) before
                              # calling it an attack.
        hdg_sustain=4,        # Consecutive disagreeing samples required.
        hdg_min_dist_m=5.0,   # Don't compare heading when the drone barely moved
                              # (division by near-zero distance is unreliable, and
                              # a hovering drone's heading is arbitrary anyway).
        # --- area-of-operations check ----------------------------------------
        ao_polygons=None,     # Optional {company: [(lat, lon), ...]} mapping.
                              # When supplied, drones outside their company's AO
                              # polygon are flagged OUT_AO. Pass None to skip.
        ao_sysid_company=None,  # {sysid: company} so the check knows which polygon
                              # to test each drone against.
        # --- bookkeeping ------------------------------------------------------
        history=64,           # Fixes retained per drone.
    ):
        self.known = set(known_sysids)
        self.max_speed_ms = max_speed_ms
        self.max_climb_ms = max_climb_ms
        self.vel_tol_ms = vel_tol_ms
        self.vel_tol_frac = vel_tol_frac
        self.vel_sustain = vel_sustain
        self.src_window_s = src_window_s
        self.handoff_hold_s = handoff_hold_s
        self.guided_window_s = guided_window_s
        self.guided_fleet_n = guided_fleet_n
        self.hdg_tol_deg = hdg_tol_deg
        self.hdg_sustain = hdg_sustain
        self.hdg_min_dist_m = hdg_min_dist_m
        self.ao_polygons = ao_polygons or {}
        self.ao_sysid_company = ao_sysid_company or {}

        # --- rolling state, all keyed by system id ---------------------------
        # Recent position fixes: sysid -> deque of (t, lat, lon, alt_m, speed_ms, hdg)
        self._fixes = defaultdict(lambda: deque(maxlen=history))
        # Last time each network source was heard for a sysid: sysid -> {src: t}
        self._srcs = defaultdict(dict)
        # The one source currently "adopted" for a sysid: sysid -> (src, t)
        self._active_src = {}
        # Most recent time a quiet-gap source handoff was detected: sysid -> t
        self._handoff_at = {}
        # Last known flight mode per drone: sysid -> mode string
        self._mode = {}
        # When each drone most recently *entered* GUIDED: sysid -> t
        self._guided_at = {}
        # Running count of disagreeing speed samples: sysid -> int
        self._vel_bad = defaultdict(int)
        # Running count of heading-vs-delta disagreements: sysid -> int
        self._hdg_bad = defaultdict(int)

    # =====================================================================
    # INGEST  -- called for every message; keep it cheap
    # =====================================================================
    def observe(self, sysid, mtype, *, t, lat=None, lon=None, alt_m=None,
                speed_ms=None, hdg=None, mode=None, src=None):
        """Record one parsed MAVLink message.

        Parameters
        ----------
        sysid   : int    MAVLink system id the message claims to come from.
        mtype   : str    Message type, e.g. "GLOBAL_POSITION_INT", "HEARTBEAT".
        t       : float  Receive time (epoch seconds).
        lat,lon : float  Degrees; required for a position fix.
        alt_m   : float  Altitude above ground, metres (optional).
        speed_ms: float  Reported ground speed, m/s (optional).
        mode    : str    Flight-mode name; only used for HEARTBEAT.
        src     : hashable  Network origin (e.g. a (host, port) tuple). When
                            None, the provenance check is skipped for this
                            message only.
        """
        # Provenance: remember every distinct source we hear for this id.
        if src is not None:
            self._srcs[sysid][src] = t
            self._note_src(sysid, src, t)

        if mtype == "GLOBAL_POSITION_INT" and lat is not None and lon is not None:
            self._fixes[sysid].append((t, lat, lon, alt_m, speed_ms, hdg))

        elif mtype == "HEARTBEAT" and mode is not None:
            self._note_mode(sysid, mode, t)

    def _note_src(self, sysid, src, t):
        """Track the one source currently "adopted" for this sysid, and flag
        the case SRC_CONFLICT can't see: the previous source going quiet for
        longer than src_window_s, then a *different* source picking up under
        the same id. (A same-source repeat just refreshes the timestamp; a
        different source that overlaps the old one's window is a live
        conflict and is left to _check_source_conflict instead.)"""
        active = self._active_src.get(sysid)
        if active is None:
            self._active_src[sysid] = (src, t)
            return
        a_src, a_t = active
        if src == a_src:
            self._active_src[sysid] = (src, t)
            return
        if t - a_t > self.src_window_s:
            self._handoff_at[sysid] = t
        self._active_src[sysid] = (src, t)

    def _note_mode(self, sysid, mode, t):
        """Track flight-mode changes, recording the rising edge into GUIDED."""
        prev = self._mode.get(sysid)
        if mode == "GUIDED" and prev != "GUIDED":
            self._guided_at[sysid] = t      # just switched INTO guided
        elif mode != "GUIDED":
            self._guided_at.pop(sysid, None)  # left guided -> forget the edge
        self._mode[sysid] = mode

    # =====================================================================
    # EVALUATION  -- called on a timer; runs the detectors
    # =====================================================================
    def assess(self, now):
        """Return {sysid: {code: message}} for everything currently suspicious.

        Only *current* problems are returned, so the aggregator's existing
        edge-triggered enter/clear notification logic works without change.
        """
        out = defaultdict(dict)

        # Per-drone detectors. Cover every sysid we have ANY state for, not
        # just ones with position fixes -- e.g. a ghost id or a source
        # handoff can show up on HEARTBEAT-only traffic too.
        all_sysids = set(self._fixes) | set(self._active_src) | set(self._mode)
        for sysid in all_sysids:
            self._check_ghost_id(out, sysid)
            self._check_source_conflict(out, sysid, now)
            self._check_src_handoff(out, sysid, now)
            if sysid in self._fixes:
                self._check_kinematics(out, sysid, self._fixes[sysid])
                self._check_ao(out, sysid, self._fixes[sysid])

        # Fleet-wide detector (needs to see all drones at once).
        self._check_fleet_takeover(out)

        return {sysid: dict(codes) for sysid, codes in out.items()}

    # --- individual detectors --------------------------------------------
    def _check_ghost_id(self, out, sysid):
        """An id transmitting positions that was never in the fleet registry."""
        if sysid not in self.known:
            self._flag(out, sysid, "GHOST_ID",
                       f"unknown aircraft id #{sysid} (not in fleet registry)")

    def _check_source_conflict(self, out, sysid, now):
        """One system id arriving from two or more live network sources.

        A genuine drone has exactly one origin. Two at once means something is
        transmitting under an id that isn't its own -- classic impersonation.
        """
        live = [s for s, ts in self._srcs.get(sysid, {}).items()
                if now - ts <= self.src_window_s]
        if len(live) >= 2:
            self._flag(out, sysid, "SRC_CONFLICT",
                       f"telemetry for #{sysid} from {len(live)} sources at once")

    def _check_src_handoff(self, out, sysid, now):
        """A source change that happened AFTER the previous source had
        already gone quiet for longer than src_window_s.

        SRC_CONFLICT only fires while two sources are simultaneously live, so
        it can't see this pattern: real telemetry stops, then -- once it's
        aged out of the live window -- a different source picks up under the
        same id with a plausible, non-teleporting track. This is a lower
        confidence, advisory-level signal (a real drone reconnecting from a
        new client port looks the same on the wire), so it's worth watching
        rather than treating as confirmed impersonation.
        """
        t = self._handoff_at.get(sysid)
        if t is not None and now - t <= self.handoff_hold_s:
            self._flag(out, sysid, "SRC_HANDOFF",
                       f"#{sysid} source changed after {self.src_window_s:.0f}s+ "
                       f"of silence from the prior source (advisory)")

    def _check_kinematics(self, out, sysid, fixes):
        """Physics checks over the two most recent fixes: teleport, climb rate,
        reported-speed-vs-track agreement, and heading-vs-movement-vector."""
        if len(fixes) < 2:
            return
        t0, la0, lo0, al0, _sp0, _h0 = fixes[-2]
        t1, la1, lo1, al1,  sp1,  h1 = fixes[-1]
        dt = t1 - t0
        if dt <= 0:
            return  # duplicate/out-of-order timestamps: nothing to derive

        dist = _haversine_m(la0, lo0, la1, lo1)
        derived_speed = dist / dt

        # TELEPORT: the fix moved further than any aircraft could in dt.
        if derived_speed > self.max_speed_ms:
            self._flag(out, sysid, "TELEPORT",
                       f"#{sysid} jumped {dist:.0f} m in {dt:.2f}s "
                       f"({derived_speed:.0f} m/s > {self.max_speed_ms:.0f})")

        # CLIMB_RATE: altitude changing faster than the airframe physically can.
        if al0 is not None and al1 is not None:
            vclimb = abs(al1 - al0) / dt
            if vclimb > self.max_climb_ms:
                self._flag(out, sysid, "CLIMB_RATE",
                           f"#{sysid} altitude moving {vclimb:.0f} m/s "
                           f"(> {self.max_climb_ms:.0f})")

        # VEL_MISMATCH: the drone's *reported* ground speed should match the
        # speed implied by where it actually moved. An attacker that forges
        # position but leaves the velocity field stale (or the reverse) drifts
        # apart here. Requires a sustained streak so a single AUTO turn or a
        # coarse fix doesn't trip it. Skip when the track speed is itself
        # implausible -- TELEPORT already owns that case.
        if sp1 is not None and derived_speed <= self.max_speed_ms:
            tolerance = max(self.vel_tol_ms,
                            self.vel_tol_frac * max(derived_speed, sp1))
            if abs(sp1 - derived_speed) > tolerance:
                self._vel_bad[sysid] += 1
                if self._vel_bad[sysid] >= self.vel_sustain:
                    self._flag(out, sysid, "VEL_MISMATCH",
                               f"#{sysid} reports {sp1:.0f} m/s but track shows "
                               f"{derived_speed:.0f} m/s")
            else:
                self._vel_bad[sysid] = 0  # agreement resets the streak

        # GPS_ATTACK: the drone's reported heading should roughly agree with
        # the bearing implied by where its position actually moved. A GPS
        # attacker that injects a false position while leaving the drone's own
        # reported heading untouched (or vice versa) produces a sustained
        # angular disagreement that is hard to explain by normal flight.
        #
        # Conditions for skipping:
        #   * No reported heading in this fix (hdg field was 65535 / None).
        #   * The drone barely moved — bearing from near-zero dist is noisy.
        #   * The position itself was flagged as a teleport (that case is
        #     already owned by TELEPORT; don't double-count).
        if (h1 is not None
                and dist >= self.hdg_min_dist_m
                and derived_speed <= self.max_speed_ms):
            delta_bearing = _haversine_m(la0, lo0, la1, lo1)  # nonzero, confirmed
            track_bearing = _bearing_deg(la0, lo0, la1, lo1)
            hdg_diff = abs(_angle_diff(h1, track_bearing))
            if hdg_diff > self.hdg_tol_deg:
                self._hdg_bad[sysid] += 1
                if self._hdg_bad[sysid] >= self.hdg_sustain:
                    self._flag(out, sysid, "GPS_ATTACK",
                               f"#{sysid} heading {h1:.0f}° disagrees with "
                               f"track bearing {track_bearing:.0f}° "
                               f"({hdg_diff:.0f}° > {self.hdg_tol_deg:.0f}°) "
                               f"— possible GPS position attack")
            else:
                self._hdg_bad[sysid] = 0  # agreement resets the streak

    def _check_ao(self, out, sysid, fixes):
        """OUT_AO: the drone's latest reported position is outside its company's
        area of operations polygon.

        This mirrors the aggregator's own AO check but runs inside the Guardian
        so it is independently derived from raw telemetry and feeds the security
        alert pathway. It is only active when ao_polygons and ao_sysid_company
        were supplied at construction time — skipped silently otherwise."""
        if not self.ao_polygons or sysid not in self.ao_sysid_company:
            return
        company = self.ao_sysid_company[sysid]
        poly = self.ao_polygons.get(company)
        if not poly:
            return
        if not fixes:
            return
        _t, lat, lon, *_ = fixes[-1]
        if not _point_in_poly(lat, lon, poly):
            self._flag(out, sysid, "OUT_AO",
                       f"#{sysid} ({company}) reported position outside "
                       f"{company} area of operations")

    def _check_fleet_takeover(self, out):
        """Several drones entering GUIDED inside one narrow window.

        Independent delivery drones, each flown by its own operator, don't all
        switch to GUIDED at the same instant. A tight cluster of GUIDED entries
        is the fingerprint of a single actor seizing the fleet at once.
        """
        # Sort the "entered GUIDED" edges by time, then sweep for any window of
        # guided_window_s that contains at least guided_fleet_n of them.
        events = sorted(self._guided_at.items(), key=lambda kv: kv[1])
        i, n = 0, len(events)
        while i < n:
            j = i
            while j < n and events[j][1] - events[i][1] <= self.guided_window_s:
                j += 1
            cluster = events[i:j]
            if len(cluster) >= self.guided_fleet_n:
                span = cluster[-1][1] - cluster[0][1]
                for sysid, _t in cluster:
                    self._flag(out, sysid, "FLEET_GUIDED",
                               f"{len(cluster)} drones switched to GUIDED within "
                               f"{span:.1f}s (coordinated takeover?)")
            i = j  # non-overlapping sweep: continue past this cluster

    # --- helpers ----------------------------------------------------------
    def _flag(self, out, sysid, code, msg):
        """Record one alert into the result map (no side effects)."""
        out[sysid][code] = f"{SEC_PREFIX} — {msg}"

    # =====================================================================
    # REPORTING
    # =====================================================================
    def summary(self, now):
        """Fleet-level rollup of the *current* alert state, for the JSON API
        and console. Pure: it reflects assess() and nothing accumulated."""
        active = self.assess(now)
        flagged = sorted(active.keys())

        by_code = {}
        for codemap in active.values():
            for code in codemap:
                by_code[code] = by_code.get(code, 0) + 1

        return {
            "flagged_sysids": flagged,
            "flagged": len(flagged),
            "active_by_code": by_code,
        }