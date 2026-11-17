"""Unit tests for tools/screen_watch.py (the dry-run evidence trigger).

Runs on a Mac (``python3 -m unittest discover -s tests``): the trigger
table only, against records shaped like agent/mission.py's audit log; no
screenshots are taken.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import screen_watch  # noqa: E402


def rec(kind, message="", **data):
    return {"kind": kind, "message": message, "data": data}


class TestMomentFor(unittest.TestCase):
    def moments(self, records):
        done, seen_fail, out = set(), False, []
        for r in records:
            hit = screen_watch.moment_for(r, seen_fail, done)
            while hit:                       # like main(): a record may start several
                done.add(hit[0])
                out.append(hit[0])
                hit = screen_watch.moment_for(r, seen_fail, done)
            if r["kind"] == "atmos_result" and r["data"].get("passed") is False:
                seen_fail = True
        return out

    def test_incident_storyline_order(self):
        records = [rec("atmos_test_start"), rec("atmos_result", passed=False),
                   rec("atmos_test_start"), rec("atmos_result", passed=True),
                   rec("permit", "Permit CSE-0427: PRE_ENTRY_TESTING -> ENTRY_AUTHORIZED (test passed)"),
                   rec("waypoint", "At the cleanout door threshold - entering the permit space"),
                   rec("waypoint", "Reached interior waypoint 1/5 (4.6, 0.0)"),
                   rec("alarm", "GAS ALARM: H2S 78.5 ppm >= 10 ppm"),
                   rec("retreat_complete", "Retreat complete")]
        self.assertEqual(self.moments(records),
                         ["probe", "fail", "retest", "authorized", "inside", "alarm", "retreat",
                          "after"])

    def test_purge_gate_and_completed_path(self):
        records = [rec("atmos_test_start"), rec("atmos_result", passed=False),
                   rec("atmos_test_start"), rec("purge_incomplete", "Purge incomplete — wait"),
                   rec("atmos_result", passed=False), rec("atmos_test_start"),
                   rec("waypoint", "Reached interior waypoint 1/5 (4.6, 0.0)"),
                   rec("waypoint", "Reached interior waypoint 2/5 (2.5, 1.6)"),
                   rec("waypoint", "Reached interior waypoint 3/5 (0.0, 0.0)"),
                   rec("permit", "Permit CSE-0428: ENTRY_IN_PROGRESS -> COMPLETED (entrant exited)")]
        self.assertEqual(self.moments(records),
                         ["probe", "fail", "retest", "purge-wait", "inside", "mid", "completed"])

    def test_each_moment_fires_once_and_delays_are_short(self):
        done = {"probe"}
        self.assertIsNone(screen_watch.moment_for(rec("atmos_test_start"), False, done))
        for r in (rec("alarm"), rec("purge_incomplete"), rec("atmos_test_start")):
            hit = screen_watch.moment_for(r, True, set())
            self.assertLessEqual(hit[1], 3.0)


if __name__ == "__main__":
    unittest.main()
