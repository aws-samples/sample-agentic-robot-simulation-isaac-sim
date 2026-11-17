"""Unit tests for sim.gas_field — pure stdlib+numpy, run on a Mac.

Run from the repo root (/Users/.../spot-oil-and-gas-demo):

    python3 -m unittest tests.test_gas_field
    # or
    python3 -m pytest tests/test_gas_field.py
"""

import math
import unittest

from sim.gas_field import (
    BASELINE,
    BODY_FRONT_OFFSET_M,
    CO_ALARM_PPM,
    DEFAULT_RESIDUAL_FRACTION,
    GasField,
    H2S_ALARM_PPM,
    LEL_ALARM_PCT,
    MAX_INJECTED_POCKETS,
    O2_HIGH_PCT,
    O2_LOW_PCT,
    PROBE_LABEL,
    ProbeGeometry,
    UNVENTILATED_HEADSPACE,
)


class TestBaseline(unittest.TestCase):
    def test_clean_outside_tank_exact(self):
        gf = GasField(seed=7)
        # Well outside the radius-6 footprint: exactly baseline w/o noise.
        for (x, y, z) in [(10.0, 10.0, 1.0), (-8.0, 3.0, 0.2), (0.0, 6.5, 0.5)]:
            s = gf.sample(x, y, z, noise=False)
            self.assertEqual(s, dict(BASELINE), msg=str((x, y, z)))

    def test_outside_tank_no_alarms_with_noise(self):
        gf = GasField(seed=7)
        for _ in range(50):
            s = gf.sample(12.0, -9.0, 1.0)
            a = gf.alarms(s)
            self.assertFalse(a["any"], msg=str(s))


class TestPockets(unittest.TestCase):
    def test_h2s_pools_low(self):
        gf = GasField(seed=7)
        cx, cy = gf.pocket_center("h2s")
        low = gf.sample(cx, cy, 0.2, noise=False)["h2s_ppm"]
        high = gf.sample(cx, cy, 1.5, noise=False)["h2s_ppm"]
        self.assertGreater(low, high)
        self.assertGreater(low, 0.0)

    def test_lel_pocket_present_with_o2_depletion(self):
        gf = GasField(seed=7)
        cx, cy = gf.pocket_center("lel")
        s = gf.sample(cx, cy, 0.5, noise=False)
        self.assertGreater(s["lel_pct"], 0.0)
        self.assertLess(s["o2_pct"], BASELINE["o2_pct"])

    def test_pockets_inside_footprint(self):
        gf = GasField(seed=7)
        for p in gf.pockets:
            dx = p["center"][0] - gf.center[0]
            dy = p["center"][1] - gf.center[1]
            self.assertLessEqual((dx * dx + dy * dy) ** 0.5, gf.radius)


class TestInjectSpike(unittest.TestCase):
    def test_spike_raises_readings_and_alarms(self):
        gf = GasField(seed=7)
        # Clean spot outside the tank so only the spike matters.
        x, y, z = 20.0, 20.0, 1.0
        before = gf.sample(x, y, z, noise=False)
        self.assertFalse(gf.alarms(before)["any"])

        gf.inject_spike("co", 100.0, x, y, z, radius=1.5)
        after = gf.sample(x, y, z, noise=False)
        self.assertGreater(after["co_ppm"], before["co_ppm"])
        self.assertAlmostEqual(after["co_ppm"], 100.0, places=6)
        a = gf.alarms(after)
        self.assertTrue(a["co"])
        self.assertTrue(a["any"])

        # Locality: far away the spike contributes ~nothing.
        far = gf.sample(x + 30.0, y, z, noise=False)
        self.assertLess(far["co_ppm"], 1e-6)

    def test_o2_spike_is_signed_delta(self):
        gf = GasField(seed=7)
        x, y, z = -20.0, 15.0, 1.0
        gf.inject_spike("o2", -3.0, x, y, z, radius=1.0)
        s = gf.sample(x, y, z, noise=False)
        self.assertAlmostEqual(s["o2_pct"], BASELINE["o2_pct"] - 3.0, places=6)
        self.assertTrue(gf.alarms(s)["o2"])

    def test_invalid_gas_rejected(self):
        gf = GasField(seed=7)
        with self.assertRaises(ValueError):
            gf.inject_spike("methane", 10.0, 0.0, 0.0, 0.5, radius=1.0)
        with self.assertRaises(ValueError):
            gf.inject_spike("h2s", 10.0, 0.0, 0.0, 0.5, radius=0.0)

    def test_negative_amplitude_only_for_o2(self):
        gf = GasField(seed=7)
        # Signed delta is documented for o2 only; a negative co/h2s/lel
        # spike is a consumer bug and must be rejected loudly.
        for gas in ("co", "h2s", "lel"):
            with self.assertRaises(ValueError, msg=gas):
                gf.inject_spike(gas, -5.0, 0.0, 0.0, 0.5, radius=1.0)
        # o2 accepts both signs.
        gf.inject_spike("o2", -1.0, 30.0, 30.0, 0.5, radius=1.0)
        gf.inject_spike("o2", 1.0, 32.0, 30.0, 0.5, radius=1.0)

    def test_injected_pockets_are_capped(self):
        """T23: a flood of injects cannot grow the pocket list (and the cost of
        every sample) without bound; clear_injected() frees the slots."""
        self.assertEqual(MAX_INJECTED_POCKETS, 16)
        gf = GasField(seed=7)
        builtin = len(gf.pockets)
        for i in range(MAX_INJECTED_POCKETS):
            gf.inject_spike("h2s", 10.0, 20.0 + i, 20.0, 0.5, radius=1.0)
        with self.assertRaisesRegex(ValueError, "at most 16 injected pockets"):
            gf.inject_spike("h2s", 10.0, 0.0, 0.0, 0.5, radius=1.0)
        self.assertEqual(gf.injected_count, MAX_INJECTED_POCKETS)
        self.assertEqual(len(gf.pockets), builtin + MAX_INJECTED_POCKETS)
        self.assertEqual(gf.clear_injected(), MAX_INJECTED_POCKETS)
        gf.inject_spike("h2s", 10.0, 0.0, 0.0, 0.5, radius=1.0)
        small = GasField(seed=7, max_injected=1)
        small.inject_spike("co", 50.0, 20.0, 20.0, 1.0, radius=1.0)
        with self.assertRaises(ValueError):
            small.inject_spike("co", 50.0, 20.0, 20.0, 1.0, radius=1.0)
        with self.assertRaises(ValueError):
            GasField(seed=7, max_injected=-1)


class TestAlarmBoundaries(unittest.TestCase):
    @staticmethod
    def _clean(**overrides):
        s = dict(BASELINE)
        s.update(overrides)
        return s

    def test_o2_boundaries(self):
        # Exactly at the limits: NO alarm (alarm is strictly < low or > high).
        self.assertFalse(GasField.alarms(self._clean(o2_pct=O2_LOW_PCT))["o2"])
        self.assertFalse(GasField.alarms(self._clean(o2_pct=O2_HIGH_PCT))["o2"])
        self.assertTrue(GasField.alarms(self._clean(o2_pct=O2_LOW_PCT - 0.01))["o2"])
        self.assertTrue(GasField.alarms(self._clean(o2_pct=O2_HIGH_PCT + 0.01))["o2"])

    def test_lel_h2s_co_boundaries(self):
        # >= thresholds: exact boundary DOES alarm.
        self.assertTrue(GasField.alarms(self._clean(lel_pct=LEL_ALARM_PCT))["lel"])
        self.assertFalse(GasField.alarms(self._clean(lel_pct=LEL_ALARM_PCT - 0.01))["lel"])
        self.assertTrue(GasField.alarms(self._clean(h2s_ppm=H2S_ALARM_PPM))["h2s"])
        self.assertFalse(GasField.alarms(self._clean(h2s_ppm=H2S_ALARM_PPM - 0.01))["h2s"])
        self.assertTrue(GasField.alarms(self._clean(co_ppm=CO_ALARM_PPM))["co"])
        self.assertFalse(GasField.alarms(self._clean(co_ppm=CO_ALARM_PPM - 0.01))["co"])

    def test_any_flag(self):
        self.assertFalse(GasField.alarms(dict(BASELINE))["any"])
        self.assertTrue(GasField.alarms(self._clean(h2s_ppm=50.0))["any"])


class TestDeterminism(unittest.TestCase):
    def test_same_seed_same_layout_and_samples(self):
        a = GasField(seed=7)
        b = GasField(seed=7)
        self.assertEqual(a.pockets, b.pockets)
        pts = [(0.0, 0.0, 0.3), (1.0, -2.0, 0.5), (3.0, 3.0, 1.0), (5.5, 0.0, 0.2)]
        for (x, y, z) in pts:
            self.assertEqual(a.sample(x, y, z), b.sample(x, y, z))

    def test_different_seed_different_layout(self):
        a = GasField(seed=7)
        b = GasField(seed=8)
        self.assertNotEqual(a.pockets, b.pockets)


class TestWorstCaseAlong(unittest.TestCase):
    def test_worst_case_picks_up_pockets(self):
        gf = GasField(seed=7)
        h2s_c = gf.pocket_center("h2s")
        lel_c = gf.pocket_center("lel")
        path = [(-5.0, -5.0), h2s_c, lel_c, (5.0, 5.0)]
        worst = gf.worst_case_along(path, z=0.2)
        self.assertGreaterEqual(worst["h2s_ppm"],
                                gf.sample(h2s_c[0], h2s_c[1], 0.2, noise=False)["h2s_ppm"])
        self.assertGreaterEqual(worst["lel_pct"],
                                gf.sample(lel_c[0], lel_c[1], 0.2, noise=False)["lel_pct"])
        # o2 worst case is the minimum along the path.
        self.assertLessEqual(worst["o2_pct"], BASELINE["o2_pct"])

    def test_empty_path_is_baseline(self):
        gf = GasField(seed=7)
        self.assertEqual(gf.worst_case_along([], z=0.5), dict(BASELINE))

    def test_o2_enrichment_not_masked_by_min(self):
        gf = GasField(seed=7)
        # Positive o2 spike (enrichment, fire risk) on the path, outside
        # the tank so nothing else contributes.
        x, y, z = 25.0, 25.0, 1.0
        gf.inject_spike("o2", 6.0, x, y, z, radius=1.0)
        worst = gf.worst_case_along([(30.0, 30.0), (x, y)], z=z)
        self.assertGreater(worst["o2_pct"], O2_HIGH_PCT)
        self.assertTrue(gf.alarms(worst)["o2"])

    def test_o2_depletion_still_wins_when_it_alarms_harder(self):
        gf = GasField(seed=7)
        z = 1.0
        gf.inject_spike("o2", 6.0, 25.0, 25.0, z, radius=1.0)   # 26.9 (+3.4)
        gf.inject_spike("o2", -6.0, -25.0, 25.0, z, radius=1.0)  # 14.9 (-4.6)
        worst = gf.worst_case_along([(25.0, 25.0), (-25.0, 25.0)], z=z)
        self.assertAlmostEqual(worst["o2_pct"], BASELINE["o2_pct"] - 6.0,
                               places=6)


# --------------------------------------------------------------------------- #
# Opt-in modes: headspace, forced ventilation, clear_injected, probe.
# --------------------------------------------------------------------------- #

# Scene geometry (sim/build_scene.py): doorway on the wall line at (6, 0)
# facing +X; the storyline probe reaches ~0.5 m inside the opening.
DOOR_PROBE = [(5.5, 0.0, z) for z in (0.2, 0.6, 1.0)]
ROUTE_WAYPOINTS = [(6.0, 0.0), (4.6, 0.0), (2.5, 1.6), (0.0, 0.0),
                   (-2.0, -1.7), (-4.8, 0.0)]
THRESHOLDS = {"lel_pct": LEL_ALARM_PCT, "h2s_ppm": H2S_ALARM_PPM,
              "co_ppm": CO_ALARM_PPM}


def _route(step=0.05):
    pts = []
    for (ax, ay), (bx, by) in zip(ROUTE_WAYPOINTS, ROUTE_WAYPOINTS[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        pts += [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n)]
    return pts + [ROUTE_WAYPOINTS[-1]]


def _storyline_field():
    """The facade's configuration (sim/isaac_app.py)."""
    return GasField(seed=7, headspace=UNVENTILATED_HEADSPACE)


def _any_alarm(samples):
    return any(GasField.alarms(s)["any"] for s in samples)


class TestHeadspace(unittest.TestCase):
    def test_default_field_has_no_headspace(self):
        self.assertEqual(len(GasField(seed=7).pockets), 3)

    def test_headspace_keeps_seed_layout(self):
        plain, story = GasField(seed=7), _storyline_field()
        self.assertEqual(story.pockets[:3], plain.pockets)
        self.assertEqual(story.pocket_center("h2s"), plain.pocket_center("h2s"))

    def test_headspace_is_masked_to_tank(self):
        gf = _storyline_field()
        for (x, y, z) in [(10.0, 10.0, 1.0), (6.5, 0.0, 0.2), (7.0, 0.0, 0.5)]:
            self.assertEqual(gf.sample(x, y, z, noise=False), dict(BASELINE))

    def test_unventilated_door_probe_fails_and_h2s_pools_low(self):
        s02, s06, s10 = _storyline_field().probe(DOOR_PROBE, noise=False)
        self.assertTrue(GasField.alarms(s02)["h2s"], s02)
        # A probe held at 1.0 m alone would have passed.
        self.assertFalse(GasField.alarms(s10)["any"], s10)
        self.assertGreater(s02["h2s_ppm"], s06["h2s_ppm"])
        self.assertGreater(s06["h2s_ppm"], s10["h2s_ppm"])

    def test_bad_headspace_rejected(self):
        with self.assertRaises(ValueError):
            GasField(seed=7, headspace=[("methane_pct", 5.0, 1.0)])
        with self.assertRaises(ValueError):
            GasField(seed=7, headspace=[("h2s_ppm", 5.0, 0.0)])


class TestVentilation(unittest.TestCase):
    def test_default_is_unventilated_and_time_independent(self):
        gf = GasField(seed=7)
        vs = gf.ventilation_state(100.0)
        self.assertFalse(vs["on"])
        self.assertIsNone(vs["t_on"])
        self.assertEqual(vs["fraction_remaining"], 1.0)
        cx, cy = gf.pocket_center("lel")
        raw = gf.sample(cx, cy, 0.5, noise=False)
        self.assertEqual(gf.sample(cx, cy, 0.5, t=0.0, noise=False), raw)
        self.assertEqual(gf.sample(cx, cy, 0.5, t=500.0, noise=False), raw)

    def test_decay_is_exponential_toward_residual(self):
        gf = GasField(seed=7)
        cx, cy = gf.pocket_center("lel")
        raw = gf.sample(cx, cy, 0.5, noise=False)["lel_pct"]
        gf.set_ventilation(True, t=10.0, tau_s=15.0)
        r = DEFAULT_RESIDUAL_FRACTION
        for dt in (0.0, 7.5, 15.0, 45.0):
            expected = r + (1.0 - r) * math.exp(-dt / 15.0)
            vs = gf.ventilation_state(10.0 + dt)
            self.assertTrue(vs["on"])
            self.assertEqual(vs["t_on"], 10.0)
            self.assertEqual(vs["tau_s"], 15.0)
            self.assertAlmostEqual(vs["fraction_remaining"], expected, places=12)
            got = gf.sample(cx, cy, 0.5, t=10.0 + dt, noise=False)["lel_pct"]
            self.assertAlmostEqual(got, raw * expected, places=9)
        # Before t_on nothing has decayed; t=None = as of the last change.
        self.assertEqual(gf.fraction_remaining(5.0), 1.0)
        self.assertEqual(gf.fraction_remaining(None), 1.0)
        # Monotone and bounded below by the residual.
        fr = [gf.fraction_remaining(10.0 + k) for k in range(0, 300, 5)]
        self.assertEqual(fr, sorted(fr, reverse=True))
        self.assertGreaterEqual(min(fr), r)

    def test_o2_depletion_recovers(self):
        gf = GasField(seed=7)
        cx, cy = gf.pocket_center("lel")
        raw = gf.sample(cx, cy, 0.5, noise=False)["o2_pct"]
        gf.set_ventilation(True, t=0.0)
        vented = gf.sample(cx, cy, 0.5, t=150.0, noise=False)["o2_pct"]
        self.assertLess(raw, vented)
        self.assertLess(vented, BASELINE["o2_pct"])

    def test_residual_below_all_thresholds(self):
        for gf in (GasField(seed=7), _storyline_field()):
            gf.set_ventilation(True, t=0.0, tau_s=15.0)
            t = 20 * 15.0  # fraction == residual to ~1e-9
            for i in range(-24, 25):
                for j in range(-24, 25):
                    x, y = i * 0.25, j * 0.25
                    for z in (0.0, 0.2, 0.6, 1.0, 1.5):
                        s = gf.sample(x, y, z, t=t, noise=False)
                        self.assertFalse(gf.alarms(s)["any"], (x, y, z, s))
                        if z >= 0.2:  # probe / body heights: clearly below
                            for key, lim in THRESHOLDS.items():
                                self.assertLess(s[key], 0.5 * lim, (x, y, z, s))
                            self.assertGreater(s["o2_pct"], 20.5, (x, y, z, s))

    def test_door_probe_pass_implies_route_below_thresholds(self):
        # The pre-entry test is only a valid gate if the interior route is
        # already clear by the time the door-mouth probe first passes.
        gf = _storyline_field()
        gf.set_ventilation(True, t=0.0, tau_s=15.0)
        t_pass = None
        for k in range(0, 1200):
            if not _any_alarm(gf.probe(DOOR_PROBE, t=k * 0.1, noise=False)):
                t_pass = k * 0.1
                break
        self.assertIsNotNone(t_pass)
        self.assertGreater(t_pass, 15.0)  # needs real ventilation time
        for z in (0.45, 0.5, 0.6):  # body height while standing
            worst = gf.worst_case_along(_route(), z, t=t_pass)
            self.assertFalse(gf.alarms(worst)["any"], (z, worst))
        # And before ventilation the same route alarms (why entry is denied).
        raw = _storyline_field().worst_case_along(_route(), 0.5)
        self.assertTrue(GasField.alarms(raw)["lel"])
        self.assertTrue(GasField.alarms(raw)["h2s"])

    def test_injected_spikes_do_not_decay(self):
        gf = GasField(seed=7)
        gf.set_ventilation(True, t=0.0)
        gf.inject_spike("h2s", 80.0, 3.0, 0.0, 0.3, radius=1.0)
        s = gf.sample(3.0, 0.0, 0.3, t=600.0, noise=False)
        self.assertGreater(s["h2s_ppm"], 79.9)
        self.assertTrue(gf.alarms(s)["h2s"])

    def test_off_holds_fraction_and_on_resumes_from_it(self):
        gf = GasField(seed=7)
        gf.set_ventilation(True, t=0.0, tau_s=10.0)
        held = gf.fraction_remaining(10.0)
        vs = gf.set_ventilation(False, t=10.0)
        self.assertFalse(vs["on"])
        self.assertIsNone(vs["t_on"])
        self.assertAlmostEqual(gf.fraction_remaining(1000.0), held, places=12)
        gf.set_ventilation(True, t=50.0, tau_s=10.0)
        self.assertAlmostEqual(gf.fraction_remaining(50.0), held, places=12)
        self.assertLess(gf.fraction_remaining(60.0), held)

    def test_repeat_on_is_noop_new_tau_reanchors(self):
        gf = GasField(seed=7)
        gf.set_ventilation(True, t=0.0, tau_s=15.0)
        gf.set_ventilation(True, t=5.0, tau_s=15.0)
        self.assertEqual(gf.ventilation_state(6.0)["t_on"], 0.0)
        f5 = gf.fraction_remaining(5.0)
        gf.set_ventilation(True, t=5.0, tau_s=3.0)
        vs = gf.ventilation_state(5.0)
        self.assertEqual((vs["t_on"], vs["tau_s"]), (5.0, 3.0))
        self.assertAlmostEqual(vs["fraction_remaining"], f5, places=12)

    def test_reset_ventilation(self):
        gf = GasField(seed=7)
        gf.set_ventilation(True, t=0.0)
        gf.reset_ventilation()
        self.assertEqual(gf.ventilation_state(100.0),
                         {"on": False, "t_on": None, "tau_s": 15.0,
                          "fraction_remaining": 1.0,
                          "residual_fraction": DEFAULT_RESIDUAL_FRACTION})
        gf.reset_ventilation(ventilated=True, t=3.0)
        vs = gf.ventilation_state(3.0)
        self.assertTrue(vs["on"])
        self.assertEqual(vs["fraction_remaining"], DEFAULT_RESIDUAL_FRACTION)
        self.assertFalse(_any_alarm(_storyline_field_vented().probe(DOOR_PROBE, t=0.0)))

    def test_invalid_inputs_rejected(self):
        gf = GasField(seed=7)
        with self.assertRaises(ValueError):
            gf.set_ventilation(True, t=0.0, tau_s=0.0)
        with self.assertRaises(ValueError):
            gf.set_ventilation(True, t=float("nan"))
        with self.assertRaises(TypeError):
            gf.sample(0.0, 0.0, 0.5, False)  # old positional noise flag
        with self.assertRaises(ValueError):
            GasField(seed=7, residual_fraction=1.0)


def _storyline_field_vented():
    gf = _storyline_field()
    gf.reset_ventilation(ventilated=True, t=0.0)
    return gf


class TestClearInjected(unittest.TestCase):
    def test_clear_removes_only_injected(self):
        gf = _storyline_field()
        before = gf.pockets
        gf.inject_spike("h2s", 80.0, 20.0, 20.0, 0.3, radius=1.0)
        gf.inject_spike("co", 100.0, 3.0, 0.0, 0.5, radius=1.0)
        self.assertEqual(gf.injected_count, 2)
        self.assertEqual(gf.clear_injected(), 2)
        self.assertEqual(gf.injected_count, 0)
        self.assertEqual(gf.pockets, before)
        self.assertEqual(gf.sample(20.0, 20.0, 0.3, noise=False), dict(BASELINE))
        self.assertEqual(gf.clear_injected(), 0)


class TestProbe(unittest.TestCase):
    def test_probe_heights_h2s_higher_low(self):
        gf = GasField(seed=7)
        cx, cy = gf.pocket_center("h2s")
        low, mid, high = gf.probe([(cx, cy, 0.2), (cx, cy, 0.6), (cx, cy, 1.0)],
                                  noise=False)
        self.assertGreater(low["h2s_ppm"], mid["h2s_ppm"])
        self.assertGreater(mid["h2s_ppm"], high["h2s_ppm"])

    def test_probe_matches_sample_and_is_time_aware(self):
        gf = _storyline_field()
        gf.set_ventilation(True, t=0.0)
        got = gf.probe(DOOR_PROBE, t=30.0, noise=False)
        self.assertEqual(len(got), 3)
        for (x, y, z), s in zip(DOOR_PROBE, got):
            self.assertEqual(s, gf.sample(x, y, z, t=30.0, noise=False))
        self.assertEqual(gf.probe([]), [])

    def test_probe_rejects_bad_points_without_consuming_noise(self):
        a, b = GasField(seed=7), GasField(seed=7)
        for bad in ([(1.0, 2.0)], [(1.0, 2.0, float("inf"))],
                    [(0.0, 0.0, 0.2), ("x", 0.0, 0.2)]):
            with self.assertRaises(ValueError):
                a.probe(bad)
        self.assertEqual(a.sample(1.0, 1.0, 0.5), b.sample(1.0, 1.0, 0.5))


class TestModeDeterminism(unittest.TestCase):
    @staticmethod
    def _script(gf):
        out = [gf.sample(5.0, 0.0, 0.5, t=0.0)]
        gf.set_ventilation(True, t=1.0, tau_s=5.0)
        gf.inject_spike("h2s", 50.0, 4.0, 0.0, 0.3, radius=1.0)
        out += gf.probe(DOOR_PROBE, t=3.0)
        out.append(gf.ventilation_state(3.0))
        out.append(gf.clear_injected())
        out.append(gf.sample(4.0, 0.0, 0.3, t=9.0))
        gf.set_ventilation(False, t=9.0)
        out.append(gf.worst_case_along(_route(0.5), 0.5, t=20.0))
        return out

    def test_same_seed_same_mode_sequence(self):
        self.assertEqual(self._script(_storyline_field()),
                         self._script(_storyline_field()))

    def test_ventilation_does_not_touch_noise_stream(self):
        a, b = GasField(seed=7), GasField(seed=7)
        b.set_ventilation(True, t=0.0)
        for k in range(20):  # outside the tank only noise differs from baseline
            self.assertEqual(a.sample(20.0, 0.0, 0.5),
                             b.sample(20.0, 0.0, 0.5, t=float(k)))


class TestNonFiniteFailSafe(unittest.TestCase):
    """Regression: JSON NaN/Infinity on payload/gas/inject (json.loads
    accepts both) turned every reading NaN, and NaN compared False against
    every threshold, so the facade reported alarm=false everywhere."""

    def test_non_finite_injection_rejected_field_unchanged(self):
        nan, inf = float("nan"), float("inf")
        gf = GasField(seed=7)
        before = gf.sample(3.0, 0.0, 0.3, noise=False)
        for args in (("h2s", nan, 3.0, 0.0, 0.3, 1.0),
                     ("h2s", inf, 3.0, 0.0, 0.3, 1.0),
                     ("co", 100.0, 3.0, 0.0, 0.3, nan),
                     ("co", 100.0, 3.0, 0.0, 0.3, inf),
                     ("lel", 30.0, nan, 0.0, 0.3, 1.0),
                     ("o2", -3.0, 3.0, inf, 0.3, 1.0),
                     ("h2s", 80.0, 3.0, 0.0, nan, 1.0)):
            with self.assertRaises(ValueError, msg=args):
                gf.inject_spike(*args)
        self.assertEqual(gf.injected_count, 0)
        self.assertEqual(gf.sample(3.0, 0.0, 0.3, noise=False), before)

    def test_non_finite_reading_alarms(self):
        for key, flag in (("o2_pct", "o2"), ("lel_pct", "lel"),
                          ("h2s_ppm", "h2s"), ("co_ppm", "co")):
            for bad in (float("nan"), float("inf"), float("-inf")):
                s = dict(BASELINE)
                s[key] = bad
                a = GasField.alarms(s)
                self.assertTrue(a[flag], (key, bad))
                self.assertTrue(a["any"], (key, bad))
        self.assertFalse(GasField.alarms(dict(BASELINE))["any"])


# Storyline stance for the pre-entry test: body center 0.6 m outside the door
# (x = 6.6), facing the tank (-X), so the nose (0.55 m) is at the opening.
DOOR_MOUTH_POSE = (6.6, 0.0, 0.5, math.pi)
# 2 depths (0.5 m, 1.0 m inside the opening along the door normal) x 3 heights.
STORYLINE_PROBE_POINTS = [(6.0 - d, 0.0, z) for d in (0.5, 1.0) for z in (0.2, 0.6, 1.0)]


def _rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


class TestProbeGeometry(unittest.TestCase):
    """Facade gas-probe rules (sim/isaac_app.py payload/gas/probe)."""

    def setUp(self):
        self.g = ProbeGeometry()  # build_scene tank: center (0, 0), r 6, door (6, 0) facing +X

    def test_label_is_honest(self):
        self.assertIn("simulated", PROBE_LABEL)
        self.assertIn("arm-mounted on the real robot", PROBE_LABEL)
        lim = self.g.limits()
        self.assertEqual(lim["label"], PROBE_LABEL)
        self.assertEqual(lim["max_reach_m"], 1.1)
        self.assertEqual(lim["reach_from"], "body_front")
        self.assertTrue(lim["requires_pose"])

    def test_pose_unavailable_rejected(self):
        err = self.g.check(None, (5.5, 0.0, 0.2))
        self.assertIsNotNone(err)
        self.assertIn("pose unavailable", err)

    def test_storyline_points_accepted_from_door_mouth(self):
        for pt in STORYLINE_PROBE_POINTS:
            self.assertIsNone(self.g.check(DOOR_MOUTH_POSE, pt), pt)

    def test_reach_is_from_the_body_front(self):
        # 1.5 m ahead of the body center = 0.95 m ahead of the nose: in reach.
        self.assertIsNone(self.g.check((10.0, 0.0, 0.5, math.pi), (8.5, 0.0, 0.5)))
        # 1.2 m ahead of the nose: out of reach.
        err = self.g.check((10.0, 0.0, 0.5, math.pi), (8.25, 0.0, 0.5))
        self.assertIn("reach", err)
        # Behind the nose (Spot's own position) is in reach.
        self.assertIsNone(self.g.check((10.0, 0.0, 0.5, math.pi), (10.0, 0.0, 0.2)))

    def test_old_three_metre_reach_is_gone(self):
        # The old facade accepted the door-mouth points from 2.5 m away.
        for pt in STORYLINE_PROBE_POINTS:
            self.assertIn("reach", self.g.check((8.0, 0.0, 0.5, math.pi), pt))
        self.assertIn("reach", self.g.check(DOOR_MOUTH_POSE, (-4.8, 0.0, 0.2)))

    def test_one_metre_deep_points_need_the_nose_at_the_opening(self):
        # Documents the envelope the mission must stand in: the 1.0 m-deep
        # points are 1.05 m from the nose at x = 6.6 and 1.15 m at x = 6.7.
        self.assertIsNone(self.g.check((6.64, 0.0, 0.5, math.pi), (5.0, 0.0, 0.2)))
        self.assertIn("reach", self.g.check((6.7, 0.0, 0.5, math.pi), (5.0, 0.0, 0.2)))
        self.assertIsNone(self.g.check((6.7, 0.0, 0.5, math.pi), (5.5, 0.0, 0.2)))

    def test_through_the_shell_beside_the_door_is_occluded(self):
        # From the door mouth, a point just inside the shell beside the jamb.
        err = self.g.check(DOOR_MOUTH_POSE, (5.75, 1.05, 0.3))
        self.assertIn("occluded by tank wall", err)
        # host/e2e_client.py BEHIND_SHELL (y mirrored to the nose's side):
        # occluded, not out of reach, over the door-mouth stances the e2e
        # walk can end in.
        for bx in (6.45, 6.5, 6.6, 6.65, 6.7):
            for by in (-0.2, -0.15, 0.0, 0.15, 0.2):
                for dyaw in (-0.15, -0.1, 0.0, 0.1, 0.15):
                    yaw = math.pi + dyaw
                    side = 1.0 if by + 0.55 * math.sin(yaw) >= 0.0 else -1.0
                    err = self.g.check((bx, by, 0.5, yaw), (5.8, side * 0.9, 0.3))
                    self.assertIn("occluded by tank wall", err or "", (bx, by, dyaw))
        # Standing beside the tank 2 m off the door axis, facing the wall.
        err = self.g.check((6.9, 2.0, 0.5, math.pi), (5.4, 2.0, 0.5))
        self.assertIn("occluded by tank wall", err)

    def test_above_the_lintel_is_occluded(self):
        err = self.g.check(DOOR_MOUTH_POSE, (5.5, 0.0, 2.8))
        self.assertIn("occluded by tank wall", err)

    def test_inside_the_tank_only_reach_applies(self):
        self.assertIsNone(self.g.check((3.0, 0.0, 0.5, 0.0), (4.0, 0.3, 0.2)))
        # From inside, out through the shell beside the door: occluded.
        err = self.g.check((5.0, 2.0, 0.5, 0.0), (6.2, 2.1, 0.5))
        self.assertIn("occluded by tank wall", err)
        # From inside, out through the opening: fine.
        self.assertIsNone(self.g.check((5.3, 0.0, 0.5, 0.0), (6.4, 0.0, 0.5)))

    def test_far_wall_is_not_an_opening(self):
        # A crossing on the far side (x = -6) is laterally centered on the
        # door axis but is solid shell.
        err = self.g.check((-6.6, 0.0, 0.5, 0.0), (-5.5, 0.0, 0.3))
        self.assertIn("occluded by tank wall", err)

    def test_z_range(self):
        self.assertIn("outside", self.g.check(DOOR_MOUTH_POSE, (5.5, 0.0, -0.1)))
        self.assertIn("outside", self.g.check(DOOR_MOUTH_POSE, (5.5, 0.0, 3.5)))

    def test_rotated_door_same_rules(self):
        yaw = math.pi / 2.0  # door at (0, 6) facing +Y
        g = ProbeGeometry(door_xy=(0.0, 6.0), door_normal_yaw=yaw)
        bx, by = _rot(yaw, 6.6, 0.0)
        pose = (bx, by, 0.5, math.pi + yaw)
        for pt in STORYLINE_PROBE_POINTS:
            x, y = _rot(yaw, pt[0], pt[1])
            self.assertIsNone(g.check(pose, (x, y, pt[2])), pt)
        x, y = _rot(yaw, 5.75, 1.05)
        self.assertIn("occluded by tank wall", g.check(pose, (x, y, 0.3)))
        # The +X doorway of the default scene is solid shell here.
        self.assertIn("occluded by tank wall", g.check((6.6, 0.0, 0.5, math.pi), (5.5, 0.0, 0.2)))

    def test_constants_match_build_scene(self):
        # build_scene.py imports pxr (not importable on a Mac): read its text.
        import os
        import re
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "sim", "build_scene.py")
        with open(path) as fh:
            text = fh.read()
        width = float(re.search(r"^DOOR_WIDTH = ([0-9.]+)", text, re.MULTILINE).group(1))
        height = float(re.search(r"^DOOR_HEIGHT = ([0-9.]+)", text, re.MULTILINE).group(1))
        lim = self.g.limits()
        self.assertEqual((lim["door_width_m"], lim["door_height_m"]), (width, height))
        self.assertIn("size=(0.25, jamb_w, jamb_h)", text)  # 0.25 m deep jambs = 2 x half_depth


def _mission_class():
    """agent/mission.py ConfinedSpaceMission (class constants only)."""
    import importlib
    import os
    import sys
    agent_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "agent")
    if agent_dir not in sys.path:
        sys.path.insert(0, agent_dir)
    return importlib.import_module("mission").ConfinedSpaceMission


class TestProbeFromMissionSeat(unittest.TestCase):
    """Cross-module: where agent/mission.py seats Spot for the door test must
    leave all six storyline points inside the facade's probe rules, with
    room for the standing drift measured on the host.

    Regression (host 2026-09-27, scripted storyline): the seat stopped at
    its tolerance edge 0.02 m outside door_mouth_xy (x 6.62), the standing
    body drifted to x 6.65 within 2 s, the facade rejected the 1.0 m points
    ("1.10 m from Spot's body front, beyond the 1.1 m probe reach"), the
    re-seat ended at the same edge and the storyline ABORTED at the first
    test."""

    STANDING_DRIFT_M = 0.04   # measured ~0.03 m outward within 2 s of stopping

    def setUp(self):
        self.M = _mission_class()
        self.g = ProbeGeometry()  # door (6, 0) facing +X, as agent/site_defaults.json

    def _points(self):
        return [(6.0 - d, 0.0, z) for d in self.M.PROBE_DEPTHS_M for z in self.M.PROBE_HEIGHTS_M]

    def test_mission_asks_for_the_six_storyline_points(self):
        self.assertEqual(sorted(self._points()), sorted(STORYLINE_PROBE_POINTS))

    def test_seat_edge_plus_standing_drift_keeps_every_point_in_reach(self):
        M = self.M
        # Coming from outside, walk_to stops PROBE_SEAT_TOL_M short of the
        # seat target (door_mouth_xy moved PROBE_SEAT_LEAD_M toward the door).
        edge = M.DOOR_MOUTH_OFFSET_M - M.PROBE_SEAT_LEAD_M + M.PROBE_SEAT_TOL_M
        for along in (edge, edge + self.STANDING_DRIFT_M):
            for lateral in (-0.03, 0.0, 0.03):
                for yaw_deg in (-M.PROBE_REPOSITION_TOL_DEG, 0.0, M.PROBE_REPOSITION_TOL_DEG):
                    pose = (6.0 + along, lateral, 0.52, math.pi + math.radians(yaw_deg))
                    for pt in self._points():
                        self.assertIsNone(self.g.check(pose, pt), (along, lateral, yaw_deg, pt))

    def test_seat_keeps_the_nose_outside_the_door_plane_and_the_body_outside_the_space(self):
        M = self.M
        edge = M.DOOR_MOUTH_OFFSET_M - M.PROBE_SEAT_LEAD_M + M.PROBE_SEAT_TOL_M
        self.assertGreater(edge - BODY_FRONT_OFFSET_M, 0.0)  # nose short of x = 6.0 at the stop
        target = M.DOOR_MOUTH_OFFSET_M - M.PROBE_SEAT_LEAD_M
        self.assertGreater(target, M.SPACE_MARGIN_M)          # even at the target: not "in space"


if __name__ == "__main__":
    unittest.main()
