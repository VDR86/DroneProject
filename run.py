#!/usr/bin/env python3
"""
Frisco UTM — project overview, run tutorial, and launcher
=========================================================

A local, self-contained UAS Traffic Management (UTM) simulation for Frisco, TX.
Ten lightweight simulated delivery drones fly real Frisco geography and emit
normal MAVLink telemetry over UDP; a read-only monitoring backend tracks them,
applies flight rules, and runs a defensive (blue-team) layer that detects
spoofed telemetry and surfaces it on a live map. Everything runs over loopback —
nothing touches a real network or a real aircraft, and nothing in the project
transmits commands or produces a spoof.


WHAT EACH FILE DOES
-------------------
Core system
  frisco_fleet.py    Scenario single-source-of-truth + the simulator. Defines
                     depots, areas of operation, no-fly zones, altitude limits
                     (400 ft ceiling / 10 m floor), colours, map centre, and the
                     fleet roster. Running it launches ten lightweight drones
                     (Wing 2-6, Zipline 12-16) that fly depot->delivery->depot and
                     emit standard MAVLink over UDP. Imported by the other tools.

  utm_aggregator.py  The read-only monitoring backend. Binds one raw UDP socket
                     on the ingest port (raw so it can see each datagram's source,
                     which the detector needs), parses MAVLink, keeps a live
                     per-drone table, applies the flight rules, folds in the
                     guardian's spoof alerts, and serves JSON at /api/drones and
                     the map at / on port 8000. Listens only; never commands.

  utm_map.html       The dashboard. A NOMINAL / ADVISORY / SECURITY condition
                     drives the look; a Security panel lists spoofed aircraft and
                     which detectors fired; spoofed drones get a magenta
                     target-lock, flight-rule alerts get an amber ping.

Defensive layer
  utm_guardian.py    The spoof detector. Decoupled from pymavlink and the fleet
                     so it can be unit-tested with plain Python values. Flags six
                     things: GHOST_ID (unknown id), SRC_CONFLICT (one id, two
                     sources), TELEPORT (impossible jump), CLIMB_RATE (impossible
                     climb), VEL_MISMATCH (reported speed vs flown track), and
                     FLEET_GUIDED (coordinated switch to GUIDED — takeover).
                     Returns alert strings only; transmits nothing.

Testing and diagnostics

  ui_selftest.py     Serves the real map with a scripted canned feed so you can
                     watch every visual state; fires a staged takeover on demand.
                     Browser-only fake JSON — no MAVLink, no drones, no injection.

  ingest_tap.py      Passive read-only listener on the ingest port. Answers "does
                     an attack's telemetry actually reach the monitor?" Run it
                     INSTEAD of the aggregator (they share the port).


SETUP (once)
------------
Needs Python 3 and pymavlink. Keep every file in ONE folder.

    python3 -m venv venv
    source venv/bin/activate        # Windows: venv\\Scripts\\activate
    pip install pymavlink


THE MAIN RUN (the normal system)
--------------------------------
Two terminals, same folder:

    Terminal 1:  python3 run.py fleet       (or: python3 frisco_fleet.py)
    Terminal 2:  python3 run.py monitor     (or: python3 utm_aggregator.py)

Then open http://localhost:8000/ in a browser. Do NOT open utm_map.html as a
file:// path — the aggregator has to serve it over HTTP. A clean run shows ten
moving drones, condition NOMINAL, and an empty Security panel. Raw API is at
http://localhost:8000/api/drones. Ctrl+C in each terminal to stop.

(Optional) The two real ArduPilot SITL drones launch separately and feed the
same ingest port via MAVProxy --out; see the RUN NOTES in frisco_fleet.py.


RUNNING BOTH TOGETHER, EACH IN ITS OWN TERMINAL (python3 run.py)
-----------------------------------------------------------------
Running the launcher with no arguments opens TWO separate terminal windows —
one for the fleet, one for a second process of your choice — and opens the
dashboard for you. It'll prompt:

    1) Normal    — fleet + utm_aggregator.py   (live telemetry, clean by default)
    2) Spoofed   — fleet + ui_selftest.py      (scripted canned attack feed)

Mode 2 gives ui_selftest.py its own terminal window on purpose: it reads
Enter/a/c/auto/q from the keyboard to drive the staged attack, so it needs a
real terminal to type into. Its Security panel content is still the scripted
canned feed described above, not real telemetry from the fleet running
alongside it.

Skip the prompt by naming the mode directly:

    python3 run.py normal      Same as choosing option 1
    python3 run.py spoofed     Same as choosing option 2

Each window runs independently once opened — run.py hands them off and
exits. Stop a process with Ctrl+C in its window, or just close the window.
On macOS this uses Terminal.app; on Linux it tries x-terminal-emulator,
gnome-terminal, konsole, xfce4-terminal, then xterm (whichever is
installed); on Windows it opens a new cmd window. If none of those is
available, it falls back to a plain background process instead (no window,
no keyboard input — fine for utm_aggregator.py, not usable for
ui_selftest.py's staged-attack prompts).


TESTING THE DETECTOR — THREE LEVELS
-----------------------------------
1. Logic (no network):   python3 run.py test        # expect 8/8 passed
2. Dashboard (visuals):  stop the aggregator, then  python3 run.py selftest
   Open http://localhost:8000/. It starts NOMINAL; drive it from the terminal:
       Enter = toggle takeover   a = attack   c = clean   auto = timed   q = quit
3. End to end:           run the normal system, then your own red-team step, and
   watch the Security panel react to live data.


COVERAGE CHECK WITH THE TAP
---------------------------
The detector only catches what reaches the ingest port. To verify an attack's
traffic gets there, run the tap INSTEAD of the aggregator (shared port):

    stop utm_aggregator.py, then:  python3 run.py tap
    then start the fleet and/or your red-team run

Watch for a NEW source address under a system id that already exists — that's
injection on the wire. If the attack never appears in the tap, the monitor
legitimately can't see it and you've found a coverage gap on that path.


PORTS
-----
UDP 14570 = ingest (fleet -> aggregator/tap).  HTTP 8000 = dashboard.
Only one process can hold each at a time, so the aggregator, ui_selftest.py, and
ingest_tap.py are mutually exclusive — stop one before starting another.


TROUBLESHOOTING
---------------
- "cannot bind UDP 14570" / port in use: another aggregator or the tap already
  holds the ingest port. Stop it and retry.
- Map won't load: open http://localhost:8000/, not the HTML file directly, and
  keep utm_map.html in the same folder as the server that's running.
- ModuleNotFoundError: pymavlink: activate the venv and pip install pymavlink.
- No drones on the map: confirm the fleet is running in its own terminal.
- Security panel never lights up in the self-test: press Enter (or a); it starts
  NOMINAL by design.
"""

import json
import os
import platform
import shlex
import shutil
import socket
import subprocess
import sys
import time
import importlib
import urllib.request
import webbrowser

# command -> (module to import + call .main() on, one-line help)
COMMANDS = {
    "fleet":    ("frisco_fleet",   "Launch the 10 lightweight simulated drones"),
    "monitor":  ("utm_aggregator", "Read-only monitor + dashboard on :8000"),
    "test":     ("test_guardian",  "Guardian unit tests, no network (expect 8/8)"),
    "selftest": ("ui_selftest",    "Serve the dashboard with a scripted canned feed"),
    "tap":      ("ingest_tap",     "Passive read-only listener on the ingest port"),
}

# the two "fleet + X" pairings offered by the no-arg launcher
UP_MODES = {
    "1": ("monitor",  "Normal",  "fleet + utm_aggregator.py (live telemetry)"),
    "2": ("selftest", "Spoofed", "fleet + ui_selftest.py (scripted canned attack feed)"),
}
UP_MODE_ALIASES = {"normal": "monitor", "spoofed": "selftest"}


def _usage(msg=None):
    if msg:
        print(f"[run] {msg}\n")
    print(__doc__.strip())
    print("\nUSAGE\n-----")
    print("    python3 run.py            Prompt, then open fleet + monitor (or fleet\n"
          "                              + selftest) each in its own terminal window")
    print("    python3 run.py normal     Skip the prompt: fleet + monitor, separate windows")
    print("    python3 run.py spoofed    Skip the prompt: fleet + selftest, separate windows")
    for name, (_mod, help_text) in COMMANDS.items():
        print(f"    python3 run.py {name:<9} {help_text}")
    print("    python3 run.py check      Offline checks: unit tests + imports + ports")
    print("    python3 run.py smoketest  End-to-end: launches fleet+monitor, asserts, stops")
    print("    python3 run.py --help     Show this reference without launching anything\n")


PORTS = (("udp", 14570, "ingest, used by fleet/aggregator/tap"),
         ("tcp", 8000, "dashboard, used by aggregator/selftest"))


def _port_free(kind, port):
    """Return True if the given port looks free to bind on 127.0.0.1 right
    now. Best-effort: a port can still be grabbed by something else between
    this check and the real bind, but it catches the common case."""
    try:
        if kind == "udp":
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        s.close()
        return True
    except OSError:
        return False


def _check_ports(indent="        "):
    ok = True
    for kind, port, who in PORTS:
        free = _port_free(kind, port)
        state = "free" if free else "IN USE"
        print(f"{indent}{kind.upper()} {port:<6} {state:<7} ({who})")
        ok = ok and free
    return ok

def _run_guardian_tests(indent=""):
    """test_guardian.py has no main() of its own — it runs its tests in its
    own __main__ block. Reproduce that same logic here so both `run.py test`
    and `run.py check` exercise the real, current test functions rather than
    a stale copy of them."""
    import test_guardian as tg
    tests = [v for k, v in sorted(vars(tg).items()) if k.startswith("test_")]
    passed = 0
    ok = True
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            print(f"{indent}FAIL {fn.__name__}: {e}")
            ok = False
        else:
            passed += 1
            print(f"{indent}ok   {fn.__name__}")
    print(f"{indent}{passed}/{len(tests)} passed")
    return ok


def _check():
    """Run everything verifiable with NO network / no other terminal needed:
    the Guardian's unit tests, plus an import/sanity pass over the other
    modules so syntax or missing-dependency errors surface immediately."""
    print("[check] 1/3  Guardian unit tests (test_guardian.py)")
    ok = _run_guardian_tests(indent="        ")

    print("[check] 2/3  Import sanity (fleet / aggregator / selftest / tap)")
    for modname in ("frisco_fleet", "utm_aggregator", "ui_selftest", "ingest_tap"):
        try:
            importlib.import_module(modname)
            print(f"        ok   {modname}")
        except Exception as e:  # noqa: BLE001 - report any import-time failure
            print(f"        FAIL {modname}: {e}")
            ok = False

    print("[check] 3/3  Ports free (14570 UDP, 8000 TCP)")
    ok = _check_ports() and ok

    print("\n[check] " + ("ALL GOOD — ready to run the fleet/monitor for real."
                          if ok else "ISSUES FOUND — see above."))
    return ok


def _smoketest():
    """End-to-end check: launch the REAL fleet + monitor as subprocesses,
    hit the live API, and assert a clean baseline -- 10/10 drones active,
    nothing flagged -- then tear both down. This is the only check here that
    actually exercises the network path; `check` only verifies imports."""
    print("[smoketest] checking ports are free...")
    if not _check_ports(indent="  "):
        print("[smoketest] ABORT: a port is already in use — stop any running "
              "fleet/monitor/tap/selftest and retry.")
        return False

    here = os.path.dirname(os.path.abspath(__file__))
    fleet_p = subprocess.Popen(
        [sys.executable, os.path.join(here, "frisco_fleet.py")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    mon_p = subprocess.Popen(
        [sys.executable, os.path.join(here, "utm_aggregator.py")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    ok = True
    try:
        print("[smoketest] fleet + monitor launched, warming up (4s)...")
        time.sleep(4.0)
        with urllib.request.urlopen("http://localhost:8000/api/drones",
                                    timeout=3) as r:
            data = json.load(r)

        total = data["counts"]["total"]
        active = data["counts"]["active"]
        flagged = data["security"]["flagged"]
        print(f"[smoketest] drones: {active}/{total} active   flagged: {flagged}")

        if total != 10:
            print(f"[smoketest] FAIL: expected 10 drones, saw {total}")
            ok = False
        if active != total:
            print(f"[smoketest] FAIL: expected all drones active, got {active}/{total}")
            ok = False
        if flagged != 0:
            print(f"[smoketest] FAIL: expected a clean baseline, {flagged} flagged")
            ok = False
        if ok:
            print("[smoketest] PASS — clean baseline confirmed end to end.")
    except Exception as e:  # noqa: BLE001 - report any failure, then clean up
        print(f"[smoketest] FAIL: {e}")
        ok = False
    finally:
        for p in (fleet_p, mon_p):
            p.terminate()
        for p in (fleet_p, mon_p):
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        print("[smoketest] fleet + monitor stopped.")
    return ok


def _terminal_launch_cmd(title, script_path, cwd):
    """Return a subprocess.Popen argv list that opens a NEW terminal window
    running `python3 script_path`, or None if no known terminal emulator is
    available on this platform. Best-effort across macOS / Linux / Windows —
    picks whichever emulator is actually installed rather than assuming one."""
    py = sys.executable
    system = platform.system()

    if system == "Darwin":
        inner = f"cd {shlex.quote(cwd)} && {shlex.quote(py)} {shlex.quote(script_path)}"
        escaped = inner.replace("\\", "\\\\").replace('"', '\\"')
        osa = f'tell application "Terminal" to do script "{escaped}"'
        return ["osascript", "-e", osa]

    if system == "Linux":
        # Try common terminal emulators in rough order of likelihood.
        candidates = [
            ("x-terminal-emulator", ["-T", title, "-e", py, script_path]),
            ("gnome-terminal",      ["--title", title, "--", py, script_path]),
            ("konsole",             ["-p", f"tabtitle={title}", "-e", py, script_path]),
            ("xfce4-terminal",      ["--title", title, "-e", f"{py} {script_path}"]),
            ("xterm",               ["-T", title, "-e", py, script_path]),
        ]
        for exe, args in candidates:
            if shutil.which(exe):
                return [exe] + args
        return None

    if system == "Windows":
        # `start` is a cmd.exe builtin, not a real executable — it only
        # parses reliably when the whole line is handed to the shell as a
        # single string (shell=True), with the title explicitly quoted so
        # `start` doesn't mistake it (or a later arg) for the program to run.
        # /k keeps the new console window open after the command finishes.
        safe_title = title.replace('"', "")
        # FIX: We must wrap the executable and script path in an OUTER set of quotes 
        # so cmd.exe doesn't strip the inner quotes when paths have spaces.
        return f'start "{safe_title}" cmd /k ""{py}" "{script_path}""'
    return None


def _launch_in_terminal(title, script_name):
    """Start script_name in its own terminal window if possible, otherwise
    fall back to a plain background subprocess (with a warning, since that
    means no visible window and no stdin — fine for utm_aggregator.py, not
    fine for ui_selftest.py which needs keystrokes)."""
    here = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(here, script_name)
    cmd = _terminal_launch_cmd(title, script_path, here)
    if cmd is None:
        print(f"[run] no terminal emulator found for {title!r} — running it as a "
              f"background process instead (no separate window, no keyboard input).")
        return subprocess.Popen([sys.executable, script_path], cwd=here)
    if isinstance(cmd, str):
        # Windows: a shell command line (needs shell=True for `start` to parse).
        return subprocess.Popen(cmd, cwd=here, shell=True)
    return subprocess.Popen(cmd, cwd=here)


def _prompt_up_mode():
    """Ask which second process should pair with the fleet. Returns a key
    into COMMANDS ("monitor" or "selftest")."""
    print("[run] Choose a mode:")
    for key, (_cmd, label, desc) in UP_MODES.items():
        print(f"    {key}) {label:<8} — {desc}")
    while True:
        choice = input("[run] > ").strip().lower()
        if choice == "":
            choice = "1"
        if choice in UP_MODES:
            return UP_MODES[choice][0]
        if choice in UP_MODE_ALIASES:
            return UP_MODE_ALIASES[choice]
        print("[run] please enter 1 or 2")


def _up(mode=None):
    """Launch the fleet plus a second process, each in its OWN terminal
    window, and open the dashboard. mode selects the second process:
    "monitor" (utm_aggregator.py, the normal live system) or "selftest"
    (ui_selftest.py, the scripted canned attack feed — needs its own
    terminal since it reads Enter/a/c/auto/q from the keyboard). If mode is
    not given, prompts interactively.

    Unlike a single subprocess pair, windows opened this way run
    independently of this script: run.py hands them off and exits, and each
    window is stopped on its own (Ctrl+C inside it, or just closing it)."""
    if mode is None:
        mode = _prompt_up_mode()
    if mode not in ("monitor", "selftest"):
        _usage(f"unknown up mode '{mode}'")
        return 1

    mode_key = "1" if mode == "monitor" else "2"
    second_modname = COMMANDS[mode][0]
    second_label, second_desc = UP_MODES[mode_key][1], UP_MODES[mode_key][2]

    print(f"[run] mode: {second_label} — {second_desc}")
    print("[run] checking ports are free...")
    if not _check_ports(indent="  "):
        print("[run] ABORT: a port is already in use — stop any running "
              "fleet/monitor/tap/selftest and retry.")
        print("[run] (run 'python3 run.py --help' to see individual commands)")
        return 1

    _launch_in_terminal("Frisco UTM - Fleet", "frisco_fleet.py")
    _launch_in_terminal(f"Frisco UTM - {second_label}", f"{second_modname}.py")

    time.sleep(1.5)
    print(f"[run] fleet + {second_modname} launched, each in its own terminal window.")
    try:
        webbrowser.open("http://localhost:8000/")
    except Exception:  # noqa: BLE001 - opening a browser is a nicety, not required
        pass
    print("[run] dashboard: http://localhost:8000/")
    print("[run] to stop, Ctrl+C (or close) each of the two terminal windows.")
    if mode == "selftest":
        print("[run] switch to the ui_selftest window to drive the staged attack "
              "(Enter/a/c/auto/q).")
    return 0


def main():
    if len(sys.argv) == 1:
        sys.exit(_up())

    if sys.argv[1] in ("-h", "--help") and len(sys.argv) == 2:
        _usage()
        sys.exit(0)

    if len(sys.argv) != 2:
        _usage("too many arguments")
        sys.exit(1)

    cmd = sys.argv[1]
    if cmd == "check":
        sys.exit(0 if _check() else 1)
    if cmd == "test":
        sys.exit(0 if _run_guardian_tests() else 1)
    if cmd == "smoketest":
        sys.exit(0 if _smoketest() else 1)
    if cmd in UP_MODE_ALIASES:
        sys.exit(_up(UP_MODE_ALIASES[cmd]))

    if cmd not in COMMANDS:
        _usage(f"unknown command '{cmd}'")
        sys.exit(1)

    modname, _help_text = COMMANDS[cmd]
    try:
        mod = importlib.import_module(modname)
    except ModuleNotFoundError as e:
        if "pymavlink" in str(e):
            print("[run] ModuleNotFoundError: pymavlink — activate the venv and "
                  "run: pip install pymavlink")
            sys.exit(1)
        raise
    mod.main()


if __name__ == "__main__":
    main()