# GridEye – Drone Fleet Monitoring & Attack Detection

GridEye is a simulated drone-fleet monitoring system. A fleet of simulated drones streams MAVLink telemetry over UDP to an **aggregator**, which tracks every aircraft, evaluates flight rules (altitude, no-fly zones, areas of operation), and displays the fleet on a live map. A **Guardian** layer watches the same telemetry for signs of spoofing, impersonation, and takeover, and raises security alerts.

> **Scope:** This project is defensive. The Guardian only consumes telemetry and returns alert strings. It never transmits and never commands a vehicle.

---

## Demo

Watch the demo video of the GridEye web interface: [GridEye Demo Video](https://drive.google.com/file/d/1foH3cFJgS4GeDSAXQ1gfDbjtxwATjZ6l/view)

---

## Why this exists

The aggregator's ingest is a single plain-UDP MAVLink stream keyed only by system ID, with no authentication. Anyone who can reach the port could inject fixes for an existing drone, invent a "ghost" aircraft, feed a plausible fake track, or flip several drones into `GUIDED` mode at once. GridEye detects these situations by checking whether the data is physically possible and where it came from.

---

## Features

- **Fleet aggregation:** ingests MAVLink telemetry (`GLOBAL_POSITION_INT`, `HEARTBEAT`) from many drones
- **Live map UI:** `map.html` visualizes the fleet and its alerts
- **Flight-rule alerts:** altitude, no-fly zones, and area-of-operations checks
- **Attack detection (Guardian):** nine detectors, listed below
- **Simulation tooling:** lightweight simulated fleet plus load-test and self-test scripts

### Guardian detectors

| Code | What it catches |
|---|---|
| `GHOST_ID` | Positions from a system ID that isn't in the fleet registry |
| `SRC_CONFLICT` | One system ID arriving from two or more live network sources at once |
| `SRC_HANDOFF` | A different source taking over an ID after the previous one went quiet (advisory) |
| `TELEPORT` | A fix-to-fix jump implying an impossible ground speed |
| `CLIMB_RATE` | Altitude changing faster than the airframe could |
| `VEL_MISMATCH` | Reported speed disagrees with the speed implied by the track |
| `GPS_ATTACK` | Reported heading disagrees with the movement vector between fixes |
| `FLEET_GUIDED` | Several drones switching to `GUIDED` within a narrow time window |
| `OUT_AO` | Position outside the drone's company area of operations |

The Guardian works in two phases, mirroring the aggregator:

1. `observe()` is called for every message and just records history.
2. `assess()` is called on a timer, runs the detectors, and returns only *current* problems so the aggregator's edge-triggered enter/clear alerts work unchanged.

All thresholds (max speed, climb rate, tolerances, sustain counts, windows) are configurable via `Guardian(...)` constructor arguments.

---

## Repository layout

| File | Description |
|---|---|
| `run.py` | Entry point to start the system |
| `aggregator.py` | Receives telemetry, tracks the fleet, evaluates rules, serves the data/API |
| `grideye_guardian.py` | Attack-resistance / anomaly detection layer |
| `lightweight_fleet.py` | Lightweight simulated drone fleet that produces telemetry |
| `fleet_attack.py` | Test harness used to exercise the Guardian's detectors in simulation |
| `load_test_single_socket*.py` | Load tests sending telemetry over a single socket (normal and teleport scenarios) |
| `load_test_multisocket.py` | Load test using multiple sockets |
| `ui_selftest.py` | UI self-test |
| `map.html` | Live map front end |
| `GridEye_Simulations_Runbook.md.pdf` | Runbook for running the simulations |
| `Screenshots/` | Screenshots of the UI |

---

## Getting started

### Prerequisites

- Python 3.9+
- [`pymavlink`](https://pypi.org/project/pymavlink/)
- A modern web browser for the map UI

```bash
git clone https://github.com/VDR86/DroneProject.git
cd DroneProject
pip install pymavlink
```

> Add any other dependencies here (e.g. a `requirements.txt`) once confirmed.

### Run

```bash
# Start the aggregator and UI
python run.py
```

Then open the map UI in your browser (see the runbook PDF for the exact address and port).

For the full step-by-step simulation procedure, see **`GridEye_Simulations_Runbook.md.pdf`**.

### Using the Guardian on its own

The Guardian has no dependency on pymavlink and can be tested with plain Python values:

```python
from grideye_guardian import Guardian
import time

g = Guardian(known_sysids=[1, 2, 3])

g.observe(1, "GLOBAL_POSITION_INT", t=time.time(),
          lat=38.9, lon=-77.3, alt_m=50.0, speed_ms=15.0, hdg=90.0,
          src=("10.0.0.5", 14550))

alerts = g.assess(time.time())   # {sysid: {code: message}}
print(g.summary(time.time()))    # fleet-level rollup
```

---

## Testing

```bash
python load_test_single_socket_normal.py     # baseline load
python load_test_single_socket_teleports.py  # teleport scenario
python load_test_multisocket.py              # multi-socket load
python ui_selftest.py                        # UI self-test
```

---

## Screenshots: Load Test Results

The images in the `Screenshots/` folder (plus the `Screenshot_2026-07-30_18_09_18.png` in the repo root) were captured while running the load test scripts. They show how the aggregator and map UI behave as telemetry volume increases, and how the Guardian responds under load.

The load tests cover three scenarios:

- **Normal traffic** (`load_test_single_socket_normal.py`): a baseline where all telemetry is physically plausible and attacker overhead is minimal.
- **Teleport scenario** (`load_test_single_socket_teleports.py`): randomized positional telemetry to examine movement algorithm performance differences and alert system degradation.
- **Multi-socket** (`load_test_multisocket.py`): telemetry arriving from multiple network sources, to examine network methodology performance differences.


