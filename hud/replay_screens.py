#!/usr/bin/env python3
"""Headless-browser screenshots of the HUD replaying REAL dry-run events (Mac).

What it does
------------
For each moment below it starts ``hud/server.py --replay <recording>
--replay-at <sim t>`` (FakeBridge replay mode: the recorded event sequence
from docs/evidence/, held at that sim time, with the current agent speech
post-filter applied to the recorded agent lines) and shoots the page with
headless Chrome at 1920x1080, 1440x900 and 1280x720. One server per shot, so
an alarm flash (shown only for alerts < 10 s old) is always on screen when
the moment is an alarm.

    python3 hud/replay_screens.py                       # -> docs/evidence/v5/hud-*.png
    python3 hud/replay_screens.py --only purge-hold --sizes 1920x1080

Nothing here touches the sim host: it is a Mac-side check of the HUD itself.
The pages carry a REPLAY badge; the camera tile is a placeholder.

Scanner notes (reviewed): the only URLs opened are the local replay
server's, http://127.0.0.1:<port>/..., and loopback_url() refuses anything
else. Both programs it starts are named here, each with an argument list and
no shell: "python3" running hud/server.py, which python_env() makes this
Python (it refuses to start the server otherwise), and Google Chrome at
CHROME. Nothing chooses another program at run time.
"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.parse
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVIDENCE = os.path.join(REPO, "docs", "evidence")
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
SIZES = ("1920x1080", "1440x900", "1280x720")

# name -> (session, sim time, what the shot is for)
MOMENTS = {
    "ready": ("ok1", 344.0, "Spot standing at spawn: SPOT and SPAWN labels apart"),
    "purge-hold": ("ok1", 445.0, "V3: amber PURGE HOLD, 8.5 % at test, progress to <= 8 %"),
    "purge-reached": ("ok1", 449.0, "V3: agent says 8.5 %, HUD shows 8.5 % at test, target reached"),
    "entry-limits": ("inc2", 173.5, "V4: recorded 'It is safe.' as the current post-filter says it"),
    "alarm": ("inc2", 204.3, "P2: flash, trigger reading, 'Stop commanded < 1 ms'"),
    "retreat-complete": ("inc2", 221.0, "P2: header TRIGGER vs now, log top fade, plain tool labels"),
    "completed": ("ok1", 590.0, "P2: permit COMPLETED, 'Walk to entry' instead of walk_to_named"),
}


def loopback_url(url):
    """``url`` if it is http(s)://127.0.0.1, else ValueError. urlopen also
    opens file: and custom schemes (Bandit B310); this tool must only ever
    reach the replay server it started on the loopback interface."""
    parts = urllib.parse.urlsplit(url)
    if parts.scheme not in ("http", "https") or parts.hostname != "127.0.0.1":
        raise ValueError("refusing to open %r: only http(s)://127.0.0.1 is allowed" % url)
    return url


def executable_path(path):
    """``path`` if it is an absolute path to an executable file, else ValueError."""
    if not (isinstance(path, str) and os.path.isabs(path) and os.path.isfile(path)
            and os.access(path, os.X_OK)):
        raise ValueError("not an absolute path to an executable file: %r" % (path,))
    return path


def python_env():
    """The replay server's environment: os.environ with this Python's directory
    first on PATH, so the literal "python3" that starts hud/server.py is this
    interpreter. RuntimeError if the python3 there is another one."""
    path = os.pathsep.join(p for p in (os.path.dirname(sys.executable), os.environ.get("PATH")) if p)
    found = shutil.which("python3", path=path)
    if not found or os.path.realpath(found) != os.path.realpath(sys.executable):
        raise RuntimeError("the python3 next to %s is %s, not this interpreter: run this tool "
                           "with python3" % (sys.executable, found))
    return dict(os.environ, PATH=path)


def wait_ok(proc, timeout):
    """Wait for ``proc`` as subprocess.run(check=True, timeout=...) would: kill it
    and re-raise TimeoutExpired after ``timeout`` s, CalledProcessError if it fails."""
    try:
        code = proc.wait(timeout=timeout)
    except BaseException:  # TimeoutExpired or an interrupt: stop it, as run() does
        proc.kill()
        proc.wait()
        raise
    if code:
        raise subprocess.CalledProcessError(code, proc.args)


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_ready(port, timeout=20.0):
    """The HUD's state once the replay has fast-forwarded (events are in)."""
    deadline = time.monotonic() + timeout
    url = loopback_url("http://127.0.0.1:%d/api/state" % port)
    while time.monotonic() < deadline:
        try:
            # Checked by loopback_url: http(s)://127.0.0.1 only, never file: or another host.
            # nosemgrep: python.lang.security.audit.dynamic-urllib-use-detected.dynamic-urllib-use-detected
            with urllib.request.urlopen(url, timeout=1) as resp:  # nosec B310
                state = json.load(resp)
            if state.get("events_seq") and state.get("gas"):
                return state
        except OSError:
            pass
        time.sleep(0.2)  # nosemgrep: arbitrary-sleep
    raise RuntimeError("HUD did not come up on port %d" % port)


def shoot(name, session, sim_t, size, out_dir, budget_ms, env):
    port = free_port()
    # This Python (env from python_env()) running hud/server.py on repository paths.
    server = subprocess.Popen(["python3", "hud/server.py", "--port", str(port),
                               "--log-level", "warning", "--replay-speech-filter",
                               "--replay-at", str(sim_t),
                               "--replay", os.path.join(EVIDENCE, "events-%s.jsonl" % session),
                               "--replay", os.path.join(EVIDENCE, "hud-audit-%s.csv" % session)],
                              cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        state = wait_ready(port)
        width, height = size.split("x")
        path = os.path.join(out_dir, "hud-%s-%s.png" % (name, size))
        # Chrome at CHROME, fixed flags, a path under --out and the loopback replay URL.
        wait_ok(subprocess.Popen([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                                  "--force-device-scale-factor=1",
                                  "--window-size=%s,%s" % (width, height),
                                  "--virtual-time-budget=%d" % budget_ms, "--screenshot=%s" % path,
                                  loopback_url("http://127.0.0.1:%d/" % port)],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL), timeout=60)
        hold = state.get("purge_hold") or {}
        print("%-40s permit=%-18s alert=%s hold=%s" % (
            os.path.relpath(path, REPO), state["permit"]["state"], state["alert"]["title"],
            hold.get("fraction_remaining") if hold.get("active") else None))
        return path
    finally:
        server.terminate()
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default=os.path.join(EVIDENCE, "v5"))
    parser.add_argument("--only", action="append", choices=sorted(MOMENTS))
    parser.add_argument("--sizes", default=",".join(SIZES))
    parser.add_argument("--budget-ms", type=int, default=4000,
                        help="Chrome --virtual-time-budget (page settle time)")
    args = parser.parse_args(argv)
    try:
        executable_path(CHROME)
        env = python_env()
    except (ValueError, RuntimeError) as err:
        print("cannot take screenshots: %s (needs Google Chrome in /Applications)" % err,
              file=sys.stderr)
        return 2
    os.makedirs(args.out, exist_ok=True)
    for name in args.only or list(MOMENTS):
        session, sim_t, _why = MOMENTS[name]
        for size in args.sizes.split(","):
            shoot(name, session, sim_t, size.strip(), args.out, args.budget_ms, env)
    return 0


if __name__ == "__main__":
    sys.exit(main())
