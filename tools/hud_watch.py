"""Tail a mission JSONL log; snapshot the HUD's /api/state at key storyline moments.

What it is:
    The host-side helper that produced the RUNBOOK's hud-*.json rehearsal
    snapshots (was /opt/spot-demo/tools/hud_watch.py on the sim host,
    copied into the repo 2026-09-27). Pure stdlib. Run it ON the sim host
    (the HUD listens on 127.0.0.1:8080 there; host/run_hud.sh), next to a
    mission or agent run that writes MISSION.jsonl under /opt/spot-demo/logs.

Usage:
    python3 tools/hud_watch.py MISSION.jsonl OUTDIR [timeout_s]

Writes OUTDIR/hud-<moment>.json ({"moment", "trigger", "captured_wall",
"state"}) once each, 1 s after the triggering record (so the HUD has folded
the event in), for:
    fail            first failed atmos_result
    vent            first "Ventilating" ventilation progress record
    authorized      first record with permit ENTRY_AUTHORIZED
    incident        first alarm
    incident-after  retreat_complete / retreat_incomplete
Exits after all five or at the timeout (default 3600 s). ``trigger.wall``
/ ``trigger.wall_t`` echo the record's wall-clock stamps (ISO / unix).
The file may not exist yet when started; it waits for it.

Scanner note (reviewed): the one URL it opens is the HUD's own loopback
endpoint, plain HTTP because the HUD serves no TLS on 127.0.0.1;
loopback_url() refuses anything that is not http(s)://127.0.0.1.
"""
import json
import os
import sys
import time
import urllib.parse
import urllib.request

URL = "http://127.0.0.1:8080/api/state"
MOMENTS = ("fail", "vent", "authorized", "incident", "incident-after")


def loopback_url(url):
    """``url`` if it is http(s)://127.0.0.1, else ValueError. urlopen also
    opens file: and custom schemes (Bandit B310); this helper must only
    ever read the HUD on the loopback interface."""
    parts = urllib.parse.urlsplit(url)
    if parts.scheme not in ("http", "https") or parts.hostname != "127.0.0.1":
        raise ValueError("refusing to open %r: only http(s)://127.0.0.1 is allowed" % url)
    return url


def fetch_state(url=URL, timeout=5):
    """The HUD's /api/state, or {"error": ...} if it cannot be read."""
    try:
        # Checked by loopback_url: http(s)://127.0.0.1 only, never file: or another host.
        # nosemgrep: dynamic-urllib-use-detected
        with urllib.request.urlopen(loopback_url(url), timeout=timeout) as resp:  # nosec B310
            return json.loads(resp.read())
    except Exception as error:  # noqa: BLE001
        return {"error": str(error)}


def moment_for(rec, done):
    """The storyline moment ``rec`` starts, or None if none (or already done)."""
    kind, data = rec.get("kind"), rec.get("data") or {}
    if "fail" not in done and kind == "atmos_result" and not data.get("passed"):
        return "fail"
    if "vent" not in done and kind == "ventilation" and "Ventilating" in (rec.get("message") or ""):
        return "vent"
    if "authorized" not in done and rec.get("permit") == "ENTRY_AUTHORIZED":
        return "authorized"
    if "incident" not in done and kind == "alarm":
        return "incident"
    if "incident-after" not in done and kind in ("retreat_complete", "retreat_incomplete"):
        return "incident-after"
    return None


def snap(name, rec, outdir, done):
    time.sleep(1.0)  # let the HUD receive and fold in the event  # nosemgrep: arbitrary-sleep
    state = fetch_state()
    path = os.path.join(outdir, "hud-%s.json" % name)
    with open(path, "w") as fh:
        json.dump({"moment": name, "trigger": {"kind": rec.get("kind"), "wall": rec.get("wall"),
                                               "wall_t": rec.get("wall_t"),
                                               "message": rec.get("message")},
                   "captured_wall": time.time(), "state": state}, fh, indent=1)
    print("snapshot", name, "<-", rec.get("kind"), (rec.get("message") or "")[:90], flush=True)
    done.add(name)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    log, outdir = argv[0], argv[1]
    timeout = float(argv[2]) if len(argv) > 2 else 3600
    done = set()
    end = time.time() + timeout
    while not os.path.exists(log) and time.time() < end:
        time.sleep(0.2)  # nosemgrep: arbitrary-sleep
    pos = 0
    while time.time() < end and len(done) < len(MOMENTS):
        with open(log) as fh:
            fh.seek(pos)
            lines = fh.readlines()
            pos = fh.tell()
        for line in lines:
            if not line.endswith("\n"):
                pos -= len(line)
                break
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            name = moment_for(rec, done)
            if name is not None:
                snap(name, rec, outdir, done)
        time.sleep(0.2)  # nosemgrep: arbitrary-sleep
    print("watcher done:", sorted(done), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
