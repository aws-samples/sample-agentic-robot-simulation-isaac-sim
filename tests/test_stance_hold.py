"""Unit tests for sim.stance_hold (the facade's standing heading hold).

Regression (host 2026-09-27): after a walk, the zero-commanded policy crept
~40 deg counter-clockwise within 12 s; at the door that swung the probe out
of reach and the scripted storyline ABORTED. The hold must counter such a
creep on zero commands and never alter a real command.
"""

import math
import unittest

from sim.stance_hold import HOLD_YAW_MAX_RAD_S, HeadingHold, wrap


class TestHeadingHold(unittest.TestCase):
    def setUp(self):
        self.h = HeadingHold()

    def test_real_commands_pass_through_and_release(self):
        self.h.command((0, 0, 0), 1.0)
        self.assertIsNotNone(self.h.yaw)
        for cmd in ((0.3, 0.0, 0.0), (0.0, 0.0, -0.5), (0.0, 0.1, 0.0)):
            self.assertEqual(self.h.command(cmd, 2.0), tuple(float(v) for v in cmd))
            self.assertIsNone(self.h.yaw)

    def test_first_zero_command_latches_without_moving(self):
        self.assertEqual(self.h.command((0, 0, 0), 0.7), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(self.h.yaw, 0.7)

    def test_dead_band(self):
        self.h.command((0, 0, 0), 0.0)
        self.assertEqual(self.h.command((0, 0, 0), math.radians(0.9)), (0.0, 0.0, 0.0))

    def test_ccw_creep_gets_a_clockwise_correction_capped(self):
        self.h.command((0, 0, 0), math.pi)
        _, _, wz = self.h.command((0, 0, 0), math.pi + math.radians(5.0))  # crept CCW 5 deg
        self.assertLess(wz, 0.0)
        self.assertAlmostEqual(wz, -1.5 * math.radians(5.0))
        _, _, wz = self.h.command((0, 0, 0), math.pi + math.radians(40.0))
        self.assertEqual(wz, -HOLD_YAW_MAX_RAD_S)

    def test_wraps_across_pi(self):
        # Latched at +179 deg, now at -179 deg (2 deg CCW across the seam).
        self.h.command((0, 0, 0), math.radians(179.0))
        _, _, wz = self.h.command((0, 0, 0), math.radians(-179.0))
        self.assertLess(wz, 0.0)
        self.assertAlmostEqual(wz, -1.5 * math.radians(2.0), places=6)

    def test_closed_loop_holds_against_the_measured_creep(self):
        # Crude plant: yaw rate = commanded wz + a creep that grows like the
        # host trace (0.02 -> 0.16 rad/s over 12 s). With the hold the
        # heading stays within a few degrees; without it, ~40 deg.
        def run(hold):
            yaw, dt = math.pi, 0.02
            for i in range(int(12.0 / dt)):
                creep = 0.02 + 0.14 * (i * dt / 12.0)
                wz = self.h.command((0, 0, 0), yaw)[2] if hold else 0.0
                yaw = wrap(yaw + (wz + creep) * dt)
            return abs(math.degrees(wrap(yaw - math.pi)))
        self.assertGreater(run(False), 30.0)
        self.h.release()
        self.assertLess(run(True), 7.0)

    def test_unknown_yaw_releases(self):
        self.h.command((0, 0, 0), 1.0)
        self.assertEqual(self.h.command((0, 0, 0), None), (0.0, 0.0, 0.0))
        self.assertIsNone(self.h.yaw)
        self.assertEqual(self.h.command((0, 0, 0), float("nan")), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
