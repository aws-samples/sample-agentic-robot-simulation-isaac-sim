"""Unit tests for sim.battery (the facade's simulated battery) -- stdlib only.

Regression (critic V5, 2026-09-27): the v1 fake discharge (95 %/h, floor 5 %)
turned the HUD battery chip amber after ~44 min and red after ~54 min, i.e.
during a leadership demo on a sim started an hour earlier. The HUD thresholds
(hud/static/app.js) are amber < 30 %, red < 15 %.

    python3 -m unittest tests.test_battery
"""

import unittest

from sim.battery import BATTERY_MODES, DEFAULT_BATTERY_MODE, SimBattery

HUD_AMBER_BELOW = 30.0


class TestSimBattery(unittest.TestCase):
    def test_default_is_demo_and_modes_are_advertised(self):
        self.assertEqual(DEFAULT_BATTERY_MODE, "demo")
        self.assertEqual(SimBattery().mode, "demo")
        self.assertEqual(set(BATTERY_MODES), {"demo", "real", "off"})

    def test_demo_survives_a_two_hour_session_green(self):
        b = SimBattery("demo")
        self.assertEqual(b.pct(0.0), 100.0)
        self.assertGreaterEqual(b.pct(2 * 3600.0), 45.0)
        for hours in (0.5, 1, 2, 3, 6, 24):
            self.assertGreaterEqual(b.pct(hours * 3600.0), HUD_AMBER_BELOW)   # never amber
        self.assertEqual(b.pct(24 * 3600.0), 30.0)                            # the floor

    def test_demo_still_visibly_drains(self):
        b = SimBattery("demo")
        self.assertLess(b.pct(3600.0), 80.0)
        self.assertGreater(b.pct(60.0), b.pct(600.0))

    def test_real_is_about_ninety_minutes(self):
        b = SimBattery("real")
        self.assertAlmostEqual(b.pct(45 * 60.0), 50.0, places=1)
        self.assertAlmostEqual(b.pct(80 * 60.0), 11.11, places=1)
        self.assertEqual(b.pct(90 * 60.0), 5.0)          # floor
        self.assertEqual(b.pct(10 * 3600.0), 5.0)

    def test_off_is_constant(self):
        b = SimBattery("off")
        self.assertEqual({b.pct(t) for t in (0.0, 3600.0, 1e6)}, {100.0})

    def test_reset_recharges_and_restarts_the_drain(self):
        b = SimBattery("real")
        self.assertLess(b.pct(3600.0), 50.0)
        b.reset(3600.0)
        self.assertEqual(b.pct(3600.0), 100.0)
        self.assertAlmostEqual(b.pct(3600.0 + 45 * 60.0), 50.0, places=1)
        self.assertEqual(b.pct(0.0), 100.0)              # before the reset: never above 100

    def test_mode_is_validated(self):
        self.assertEqual(SimBattery(" REAL ").mode, "real")
        with self.assertRaises(ValueError):
            SimBattery("fast")

    def test_describe_mentions_mode_and_floor(self):
        self.assertIn("mode=demo", SimBattery("demo").describe())
        self.assertIn("floor=30", SimBattery("demo").describe())
        self.assertIn("constant", SimBattery("off").describe())


if __name__ == "__main__":
    unittest.main()
