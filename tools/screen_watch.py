"""Tail a mission JSONL log; screenshot the sim desktop + HUD at storyline moments.

What it is:
    The trigger half of the presenter dry-run evidence (docs/evidence/,
    2026-09-27); tools/snap_screens.sh does the capturing. Pure stdlib. Run
    it ON the sim host as ubuntu next to an agent or mission run that writes
    MISSION.jsonl under /opt/spot-demo/logs (the agent's --log-path).

Usage:
    python3 tools/screen_watch.py MISSION.jsonl PREFIX [timeout_s]

For each moment below, once, it runs ``bash tools/snap_screens.sh
PREFIX-<moment>`` DELAY seconds after the triggering record (in a thread,
so tailing never stalls) and prints one line per capture:
    probe      first atmos_test_start           +1.5 s  (Spot at the door, probe dwell)
    fail       first failed atmos_result        +1.0 s
    purge-wait first purge_incomplete           +1.0 s  (early re-test refused)
    retest     atmos_test_start after a FAIL    +1.5 s
    authorized permit -> ENTRY_AUTHORIZED       +1.0 s  (re-test passed)
    inside     "Reached interior waypoint 1/"   +0.3 s  (past the door; the presenter's
                                                         spike cue, so before the retreat)
    mid        "Reached interior waypoint 3/"   +0.5 s  (tank centre; runs without a spike)
    alarm      first alarm                      +0.5 s  (HUD incident banner onset)
    retreat    first alarm                      +3.0 s  (guard retreat under way)
    after      retreat_complete/_incomplete     +1.0 s
    completed  permit COMPLETED                 +1.0 s
The HUD PNG of each moment lands ~3 s after the desktop PNG (see
tools/snap_screens.sh). Exits once "after" or "completed" has been captured, or at the timeout
(default 3600 s). The log may not exist yet when started; it waits for it.
"""
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SNAP = os.path.join(HERE, "snap_screens.sh")


def moment_for(rec, seen_fail, done):
    """-> (moment, delay_s) for this record, or None."""
    kind = rec.get("kind")
    data = rec.get("data") or {}
    message = rec.get("message") or ""
    if kind == "atmos_test_start":
        if "probe" not in done:
            return "probe", 1.5
        if seen_fail and "retest" not in done:
            return "retest", 1.5
    if kind == "atmos_result" and data.get("passed") is False and "fail" not in done:
        return "fail", 1.0
    if kind == "purge_incomplete" and "purge-wait" not in done:
        return "purge-wait", 1.0
    if kind == "permit" and "-> ENTRY_AUTHORIZED" in message and "authorized" not in done:
        return "authorized", 1.0
    if kind == "waypoint" and "Reached interior waypoint 1/" in message and "inside" not in done:
        return "inside", 0.3
    if kind == "waypoint" and "Reached interior waypoint 3/" in message and "mid" not in done:
        return "mid", 0.5
    if kind == "alarm" and "alarm" not in done:
        return "alarm", 0.5
    if kind == "alarm" and "retreat" not in done:
        return "retreat", 3.0
    if kind in ("retreat_complete", "retreat_incomplete") and "after" not in done:
        return "after", 1.0
    if kind == "permit" and "-> COMPLETED" in message and "completed" not in done:
        return "completed", 1.0
    return None


def main(argv):
    if len(argv) < 3:
        sys.exit(__doc__)
    log, prefix = argv[1], argv[2]
    timeout = float(argv[3]) if len(argv) > 3 else 3600.0
    done, threads = set(), []
    seen_fail = False

    def snap(name, delay, rec):
        time.sleep(delay)
        t0 = time.time()
        proc = subprocess.run(["bash", SNAP, "%s-%s" % (prefix, name)], capture_output=True, text=True)
        print("moment %-10s <- %-18s t=%s wall_t=%s | capture %.1f s rc=%d | %s"
              % (name, rec.get("kind"), rec.get("t"), rec.get("wall_t"), time.time() - t0,
                 proc.returncode, (rec.get("message") or "")[:80]), flush=True)
        if proc.returncode:
            print(proc.stdout + proc.stderr, flush=True)

    end = time.time() + timeout
    while not os.path.exists(log) and time.time() < end:
        time.sleep(0.2)  # nosemgrep: arbitrary-sleep
    pos = 0
    while time.time() < end and not ({"after", "completed"} & done):
        with open(log) as handle:
            handle.seek(pos)
            lines = handle.readlines()
            pos = handle.tell()
        for line in lines:
            if not line.endswith("\n"):   # partial line: re-read it next time
                pos -= len(line.encode())
                break
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            # One record can start several moments (the alarm: banner onset
            # and the retreat under way).
            hit = moment_for(rec, seen_fail, done)
            while hit:
                done.add(hit[0])
                thread = threading.Thread(target=snap, args=(hit[0], hit[1], rec))
                thread.start()
                threads.append(thread)
                hit = moment_for(rec, seen_fail, done)
            if rec.get("kind") == "atmos_result" and (rec.get("data") or {}).get("passed") is False:
                seen_fail = True
        time.sleep(0.2)  # nosemgrep: arbitrary-sleep
    for thread in threads:
        thread.join()
    print("screen_watch done:", sorted(done), flush=True)


if __name__ == "__main__":
    main(sys.argv)
