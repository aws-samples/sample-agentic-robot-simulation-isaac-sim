"""Unit tests for sim.facade_guard: the simulator facade's input checks and
audit records (hardening pass). Stdlib only, plus hud.ros_bridge to check
that facade records do not trip the HUD's rules; run on a Mac:

    python3 -m unittest tests.test_hardening_sim

sim/isaac_app.py only runs inside the Isaac Sim container, so the rules are
tested here and the call sites are pinned by reading the source
(TestIsaacAppCallSites). Each test docstring names its threat ID
(docs/security/THREAT_MODEL.md).
"""

import ast
import collections
import itertools
import json
import math
import pathlib
import re
import types
import unittest

from hud import ros_bridge as rb
from sim import facade_guard as fg
from sim import launch_args

REPO = pathlib.Path(__file__).resolve().parent.parent
ISAAC_APP = (REPO / "sim" / "isaac_app.py").read_text(encoding="utf-8")


def _function_source(name):
    """Source text of a top-level def in sim/isaac_app.py."""
    tree = ast.parse(ISAAC_APP)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(ISAAC_APP, node)
    raise AssertionError("no def %s in sim/isaac_app.py" % name)


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def _hud_takes(record_text):
    """Feed one facade record to a fresh HudState; return the state."""
    state = rb.HudState()
    state.ingest_event_json(record_text)
    return state


# ---------------------------------------------------------------------------
# T20: cmd_vel / body_pose at the facade
# ---------------------------------------------------------------------------


class TestT20TwistCheck(unittest.TestCase):
    def test_in_envelope_passes_unchanged(self):
        """T20: a command inside the demo envelope reaches the policy as sent."""
        cmd, why = fg.check_twist((0.5, -0.2, 0.0), (0.0, 0.0, -0.3))
        self.assertIsNone(why)
        self.assertEqual(cmd, (0.5, -0.2, -0.3))

    def test_small_overshoot_is_clamped_into_the_envelope(self):
        """T20: up to 2x the envelope is clamped to the agent's own limits."""
        cmd, why = fg.check_twist((1.5, -0.9, 0.0), (0.0, 0.0, 1.9))
        self.assertIsNone(why)
        self.assertEqual(cmd, (1.0, -0.5, 1.0))
        self.assertEqual(fg.CMD_VEL_LIMITS, (1.0, 0.5, 1.0))   # agent/robot_client.py MAX_*

    def test_non_finite_anywhere_is_refused(self):
        """T20: NaN/inf in any Twist field refuses the command (not zeroed, not clamped)."""
        for bad in (math.nan, math.inf, -math.inf):
            for i in range(6):
                values = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1]
                values[i] = bad
                cmd, why = fg.check_twist(values[:3], values[3:])
                self.assertIsNone(cmd, (bad, i))
                self.assertIn("non-finite", why)

    def test_out_of_range_is_refused(self):
        """T20: the threat model's probe (1e9, nan, -1e9) and anything beyond 2x is refused."""
        self.assertIsNone(fg.check_twist((1e9, math.nan, 0.0), (0.0, 0.0, -1e9))[0])
        for linear, angular in (((2.01, 0, 0), (0, 0, 0)), ((0, 1.01, 0), (0, 0, 0)),
                                ((0, 0, 0), (0, 0, -2.5)), ((-1e9, 0, 0), (0, 0, 0))):
            cmd, why = fg.check_twist(linear, angular)
            self.assertIsNone(cmd, (linear, angular))
            self.assertIn("outside", why)

    def test_non_numbers_are_refused(self):
        """T20: booleans, strings and short tuples are malformed, not commands."""
        for linear in ((True, 0, 0), ("1", 0, 0), (0.1, 0.0)):
            self.assertIsNone(fg.check_twist(linear, (0, 0, 0))[0], linear)


class TestT20BodyPoseCheck(unittest.TestCase):
    def test_identity_and_small_offsets_pass(self):
        """T20: a neutral or small body_pose is accepted."""
        self.assertIsNone(fg.check_body_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        half = 0.2 / 2.0   # 0.2 rad of pitch
        self.assertIsNone(fg.check_body_pose((0.0, 0.0, -0.1), (math.cos(half), 0.0,
                                                                 math.sin(half), 0.0)))

    def test_bad_poses_are_refused(self):
        """T20: non-finite, a big height offset, a non-unit quaternion or a big tilt."""
        half = 0.7 / 2.0
        cases = {
            "non-finite": ((0.0, math.nan, 0.0), (1.0, 0.0, 0.0, 0.0)),
            "position.z": ((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)),
            "unit quaternion": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0)),
            "roll": ((0.0, 0.0, 0.0), (math.cos(half), math.sin(half), 0.0, 0.0)),
            "yaw": ((0.0, 0.0, 0.0), (math.cos(half), 0.0, 0.0, math.sin(half))),
        }
        for expected, (position, orientation) in cases.items():
            why = fg.check_body_pose(position, orientation)
            self.assertIsNotNone(why, expected)
            self.assertIn(expected, why)


# ---------------------------------------------------------------------------
# T20: the robot state machine and cmd_vel publishers
# ---------------------------------------------------------------------------


class TestT20ServiceAudit(unittest.TestCase):
    def test_every_estop_and_power_call_is_recorded(self):
        """T20: e-stop / power calls are recorded even when they change nothing."""
        for name in ("power_on", "power_off", "estop/hard", "estop/gentle", "estop/release"):
            kind, message, data, changed = fg.service_event(name, (False, False), (False, False))
            self.assertEqual(kind, "facade_service")
            self.assertEqual(data["service"], name)
            self.assertFalse(changed)
            self.assertIn(name, message)

    def test_estop_release_records_the_change(self):
        """T20: an e-stop release (anyone can call it) is recorded as a change."""
        _kind, message, data, changed = fg.service_event(
            "estop/release", (False, True), (False, False), True, "E-stop released")
        self.assertTrue(changed)
        self.assertTrue(data["was_estopped"])
        self.assertFalse(data["estopped"])
        self.assertIn("was motor power off, e-stop asserted", message)

    def test_other_calls_only_when_power_or_estop_changes(self):
        """T20: stand / cmd_vel are recorded when they power Spot on, not otherwise."""
        self.assertIsNone(fg.service_event("stand", (True, False), (True, False)))
        self.assertIsNone(fg.service_event("cmd_vel", (True, False), (True, False)))
        rec = fg.service_event("cmd_vel", (False, False), (True, False), True,
                               "implicit claim + power-on")
        self.assertIsNotNone(rec)
        self.assertTrue(rec[3])
        self.assertTrue(rec[2]["powered"])

    def test_response_text_is_cleaned(self):
        """T20: a service response copied into a record is printable and bounded."""
        _k, _m, data, _c = fg.service_event("power_on", (False, False), (True, False), True,
                                            "line1\nline2\x1b[31m" + "x" * 500)
        self.assertNotIn("\n", data["response"])
        self.assertNotIn("\x1b", data["response"])
        self.assertLessEqual(len(data["response"]), fg.MAX_TEXT_CHARS + 3)


class TestT20PublisherWatch(unittest.TestCase):
    def setUp(self):
        self.watch = fg.PublisherWatch("/demo_spot/cmd_vel")

    def test_one_publisher_is_quiet(self):
        """T20: the agent alone on cmd_vel produces no record."""
        self.assertIsNone(self.watch.count)
        self.assertIsNone(self.watch.update(["/spot_mission_client_1"]))
        self.assertEqual(self.watch.count, 1)

    def test_second_publisher_is_recorded_once(self):
        """T20: a second cmd_vel publisher is recorded with both names, once."""
        self.watch.update(["/agent"])
        kind, message, data, changed = self.watch.update(["/agent", "/_ros2cli_42"])
        self.assertEqual(kind, "facade_publishers")
        self.assertTrue(data["multiple"])
        self.assertEqual(data["count"], 2)
        self.assertEqual(data["publishers"], ["/_ros2cli_42", "/agent"])
        self.assertTrue(changed)
        self.assertIn("2 publishers on /demo_spot/cmd_vel", message)
        self.assertIsNone(self.watch.update(["/_ros2cli_42", "/agent"]))   # same set: quiet

    def test_set_change_and_return_to_one_are_recorded(self):
        """T20: a different multi-publisher set, then the return to one, are recorded."""
        self.watch.update(["/agent", "/a"])
        self.assertIsNotNone(self.watch.update(["/agent", "/b"]))
        _kind, message, data, _changed = self.watch.update(["/agent"])
        self.assertFalse(data["multiple"])
        self.assertIn("back to 1", message)
        self.assertIsNone(self.watch.update(["/agent"]))

    def test_publisher_names_from_endpoint_info(self):
        """T20: names come from rclpy TopicEndpointInfo fields, cleaned."""
        eps = [types.SimpleNamespace(node_namespace="/", node_name="presenter_gas"),
               types.SimpleNamespace(node_namespace="/demo", node_name="agent\nx")]
        self.assertEqual(fg.publisher_names(eps), ["/demo/agent?x", "/presenter_gas"])


# ---------------------------------------------------------------------------
# Audit records: shape, budget, HUD compatibility (T20, T23, T42)
# ---------------------------------------------------------------------------


class TestFacadeAudit(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.audit = fg.FacadeAudit(clock=self.clock, wall=lambda: 1700000000.0)

    def test_record_shape_matches_the_agent_events(self):
        """T20: records carry sim time, wall time and source "facade"."""
        self.assertTrue(self.audit.record("facade_service", "power_on succeeded",
                                          {"x": math.nan, "t": (1, 2)}, sim_time=12.34567))
        self.assertTrue(self.audit.pending)
        (text,) = self.audit.drain()
        self.assertFalse(self.audit.pending)
        event = json.loads(text)
        self.assertEqual(set(event), {"t", "wall_t", "source", "kind", "message", "data"})
        self.assertEqual(event["source"], "facade")
        self.assertEqual(event["t"], 12.3457)
        self.assertEqual(event["wall_t"], 1700000000.0)
        self.assertEqual(event["data"], {"x": None, "t": [1, 2]})   # strict JSON

    def test_refusal_flood_cannot_starve_applied_records(self):
        """T42/T23: refusals have their own budget; the applied record still goes out."""
        refused = fg.world_control_event("view", None, ValueError("unknown camera"), None)
        passed = sum(self.audit.record(*refused) for _ in range(100))
        self.assertEqual(passed, fg.AUDIT_BUDGETS["noise"][1])
        self.assertTrue(self.audit.record(*fg.service_event("power_on", (False, False),
                                                            (True, False))))
        last = json.loads(self.audit.drain()[-1])
        self.assertEqual(last["data"]["suppressed_before"],
                         {"count": 90, "by_kind": {"world_control/view": 90}})
        self.assertEqual(self.audit.suppressed, 90)

    def test_change_budget_is_bounded_and_refills(self):
        """T42: a flood of applied changes is bounded too, then refills with time."""
        rate, burst = fg.AUDIT_BUDGETS["change"]
        self.assertEqual(sum(self.audit.record("k", "m") for _ in range(burst + 5)), burst)
        self.assertFalse(self.audit.record("k", "m"))
        self.clock.t += 1.0 / rate
        self.assertTrue(self.audit.record("k", "m"))

    def test_pending_queue_is_bounded(self):
        """T42: records never pile up if the sim loop stops draining."""
        audit = fg.FacadeAudit(budgets={"change": (1000.0, 1000), "noise": (1.0, 1)},
                               clock=self.clock, max_pending=5)
        self.assertEqual(sum(audit.record("k", "m") for _ in range(10)), 5)
        self.assertEqual(len(audit.drain()), 5)
        # The safety controls' own backstop holds only if the loop stops draining.
        inject = fg.world_control_event("gas/inject", dict(PRESENTER_INJECT), None, ["/p"])
        n = fg.WORLD_MAX_PENDING + 10
        self.assertEqual(sum(audit.record(*inject) for _ in range(n)), fg.WORLD_MAX_PENDING)
        self.assertEqual(len(audit.drain()), fg.WORLD_MAX_PENDING)

    def test_facade_records_never_flash_or_stage_the_hud(self):
        """T20/T23: facade kinds match no HUD alert, injection or permit rule."""
        records = [
            fg.service_event("estop/hard", (True, False), (False, True)),
            fg.service_event("estop/release", (False, True), (False, False)),
            fg.PublisherWatch("/demo_spot/cmd_vel").update(["/a", "/b"]),
        ]
        for rec in records:
            audit = fg.FacadeAudit(clock=self.clock)
            audit.record(*rec, sim_time=5.0)
            (text,) = audit.drain()
            state = _hud_takes(text)
            self.assertEqual(state.alert_seq, 0, rec[0])
            self.assertIsNone(state._inject_mono, rec[0])
            snap = state.snapshot(include_events=True)
            self.assertEqual(snap["permit"]["state"], "NOT_STARTED")
            self.assertEqual(snap["events"][-1]["source"], "facade")
            self.assertIsNone(rb._ALERT_WORDS.search(rec[0]))
            self.assertIsNone(rb._INJECT_KIND.search(rec[0]))

    def test_clean_text(self):
        """T20/T23: text copied from input is printable and bounded."""
        self.assertEqual(fg.clean_text("a\nb\u202ec"), "a?b?c")
        self.assertEqual(fg.clean_text("x" * 10, limit=4), "xxxx...")


# ---------------------------------------------------------------------------
# T23: simulated-world controls
# ---------------------------------------------------------------------------

PRESENTER_INJECT = {"gas": "h2s", "ppm": 80.0, "x": 14.0, "y": 0.0, "z": 0.3, "radius": 1.0}


def _inject(**changes):
    d = dict(PRESENTER_INJECT)
    d.update(changes)
    return d


class TestT23Inject(unittest.TestCase):
    def test_shipped_senders_are_accepted(self):
        """T23: host/inject_gas.sh and the agent's presenter tool defaults pass."""
        self.assertEqual(fg.validate_inject(dict(PRESENTER_INJECT)), PRESENTER_INJECT)
        agent = {"gas": "H2S", "ppm": 40, "x": 2.5, "y": -1.0, "z": 0.5, "radius": 1.5}
        self.assertEqual(fg.validate_inject(agent)["gas"], "h2s")
        self.assertEqual(fg.validate_inject(_inject(gas="o2", ppm=-3.0))["ppm"], -3.0)
        for gas, amount in (("lel", 30.0), ("co", 100.0)):
            self.assertEqual(fg.validate_inject(_inject(gas=gas, ppm=amount))["ppm"], amount)

    def test_positive_o2_is_refused(self):
        """T23 (M23): an O2 enrichment delta could mask a depleted atmosphere."""
        for ppm in (0.0, 0.5, 5.0):
            with self.assertRaisesRegex(ValueError, "mask a depleted atmosphere"):
                fg.validate_inject(_inject(gas="o2", ppm=ppm))
        with self.assertRaises(ValueError):
            fg.validate_inject(_inject(gas="o2", ppm=-21.0))

    def test_out_of_bounds_values_are_refused(self):
        """T23: amounts, radius, height and distance from the tank are bounded."""
        bad = [_inject(ppm=0.0), _inject(ppm=-5.0), _inject(ppm=501.0),
               _inject(gas="lel", ppm=101.0), _inject(gas="co", ppm=1e6),
               _inject(radius=0.1), _inject(radius=6.0), _inject(z=-0.1), _inject(z=4.5),
               _inject(x=40.0), _inject(x=0.0, y=-25.0)]
        for d in bad:
            with self.assertRaises(ValueError, msg=d):
                fg.validate_inject(d)
        self.assertEqual(fg.validate_inject(_inject(x=20.0), tank_center=(0.0, 0.0),
                                            tank_radius=6.0)["x"], 20.0)

    def test_malformed_payloads_are_refused(self):
        """T23: allowlisted gas and keys, numbers only, finite only."""
        missing = dict(PRESENTER_INJECT)
        del missing["radius"]
        bad = [_inject(gas="methane"), _inject(gas=1), dict(PRESENTER_INJECT, extra=1), missing,
               _inject(ppm="80"), _inject(ppm=True), _inject(x=None), _inject(ppm=10 ** 400)]
        bad += [fg.parse_control_json(json.dumps(PRESENTER_INJECT).replace("80.0", v), "gas/inject")
                for v in ("NaN", "Infinity", "-Infinity")]
        for d in bad:
            with self.assertRaises(ValueError, msg=d):
                fg.validate_inject(d)

    def test_describe_inject(self):
        """T23: the record's detail names the gas, amount, position and size."""
        self.assertEqual(fg.describe_inject(fg.validate_inject(dict(PRESENTER_INJECT))),
                         "H2S 80 ppm at (14.00, 0.00, 0.30), radius 1.00 m")


class TestT23OtherControls(unittest.TestCase):
    def test_ventilation(self):
        """T23: on/off with an optional tau in range, or an exact reset."""
        self.assertEqual(fg.validate_ventilation({"on": True}), {"on": True, "tau_s": None})
        self.assertEqual(fg.validate_ventilation({"on": True, "tau_s": 2}), {"on": True, "tau_s": 2.0})
        self.assertEqual(fg.validate_ventilation({"on": False}), {"on": False, "tau_s": None})
        self.assertEqual(fg.validate_ventilation({"reset": True}), {"reset": True})
        bad = [{"on": True, "tau_s": 0.5},   # the threat model's instant purge
               {"on": True, "tau_s": 1e5}, {"on": True, "tau_s": True}, {"on": "yes"},
               {"on": True, "tau_s": "15"}, {"reset": 1}, {"reset": True, "on": True},
               {"on": True, "fan": 3}, {}]
        for d in bad:
            with self.assertRaises(ValueError, msg=d):
                fg.validate_ventilation(d)
        with self.assertRaises(ValueError):
            fg.validate_ventilation(fg.parse_control_json('{"on": true, "tau_s": NaN}', "x"))

    def test_clear_and_battery(self):
        """T23: clear and battery reset accept only what the shipped senders send."""
        self.assertEqual(fg.validate_clear({}), {})
        self.assertEqual(fg.validate_clear({"clear": "injected"}), {})
        for d in ({"clear": "all"}, {"x": 1}, {"clear": "injected", "y": 2}):
            with self.assertRaises(ValueError, msg=d):
                fg.validate_clear(d)
        self.assertEqual(fg.validate_battery_reset({}), {})
        with self.assertRaises(ValueError):
            fg.validate_battery_reset({"pct": 50})

    def test_payload_parsing_and_size_cap(self):
        """T23/T42: empty means {}, non-objects and oversized payloads are refused."""
        self.assertEqual(fg.parse_control_json("", "gas/clear"), {})
        self.assertEqual(fg.parse_control_json("  ", "gas/clear"), {})
        self.assertEqual(fg.parse_control_json("{}", "gas/clear"), {})
        for text in ("[1]", '"x"', "{bad", "3"):
            with self.assertRaises(ValueError, msg=text):
                fg.parse_control_json(text, "gas/clear")
        big = "{" + " " * fg.MAX_CONTROL_CHARS + "}"
        self.assertIsNone(fg.bounded(big))
        self.assertEqual(fg.bounded("{}"), "{}")
        with self.assertRaisesRegex(ValueError, "over 4096 characters"):
            fg.parse_control_json(fg.bounded(big), "gas/inject")
        with self.assertRaisesRegex(ValueError, "over 4096 characters"):
            fg.control_text(None, "view")


class TestT23WorldControlRecord(unittest.TestCase):
    def test_applied_record(self):
        """T23: an applied control names the control, params, publishers and pockets."""
        kind, message, data, changed = fg.world_control_event(
            "gas/inject", fg.validate_inject(dict(PRESENTER_INJECT)), None, ["/presenter_gas"],
            "H2S 80 ppm", 1)
        self.assertEqual(kind, "world_control")
        self.assertTrue(changed)
        self.assertTrue(data["accepted"])
        self.assertIsNone(data["reason"])
        self.assertEqual(data["publishers"], ["/presenter_gas"])
        self.assertEqual(data["injected_pockets"], 1)
        self.assertTrue(data["simulated"])
        self.assertEqual(message, "SIMULATED gas/inject applied: H2S 80 ppm "
                                  "[publishers: /presenter_gas]")

    def test_refused_record(self):
        """T23: a refused control carries the cleaned reason and counts as noise."""
        kind, message, data, changed = fg.world_control_event(
            "gas/ventilation", None, ValueError("tau_s must be in [2, 3600] s\n, got 0.5"), None)
        self.assertFalse(changed)
        self.assertFalse(data["accepted"])
        self.assertNotIn("\n", data["reason"])
        self.assertIn("refused: tau_s must be", message)
        self.assertIsNone(data["publishers"])

    def test_records_reach_the_hud_without_staging_or_flashing(self):
        """T23: a facade inject record is an audit line, not an injection disclosure."""
        clock = FakeClock()
        for rec in (fg.world_control_event("gas/inject", dict(PRESENTER_INJECT), None, ["/p"]),
                    fg.world_control_event("gas/clear", {}, None, ["/p"]),
                    fg.world_control_event("view", None, ValueError("unknown camera"), None)):
            audit = fg.FacadeAudit(clock=clock)
            self.assertTrue(audit.record(*rec, sim_time=42.0))
            (text,) = audit.drain()
            self.assertEqual(json.loads(text)["t"], 42.0)
            state = _hud_takes(text)
            self.assertEqual(state.alert_seq, 0)
            self.assertIsNone(state._inject_mono)
            self.assertEqual(state.snapshot()["permit"]["state"], "NOT_STARTED")


class TestT23SafetyControlRecords(unittest.TestCase):
    """T23/T42 (M23, M42): the facade audit never collapses a safety-relevant
    world control (inject, clear, ventilation, battery reset) into a count:
    each one that passed the input rate limit gets its own record, applied
    or refused. Only view changes and the other records stay budgeted."""

    def test_the_safety_controls_are_named_and_rate_limited(self):
        """T23: the four controls that change the simulated atmosphere or
        battery are the unbudgeted ones; view is not among them."""
        self.assertEqual(set(fg.SAFETY_CONTROLS),
                         {"gas/inject", "gas/clear", "gas/ventilation", "battery/reset"})
        for control in fg.SAFETY_CONTROLS:
            self.assertIn(control, fg.CONTROL_RATE_LIMITS)
        self.assertNotIn("view", fg.SAFETY_CONTROLS)
        # The backstop holds more than isaac_app can apply between two drains
        # (its two control queues, drained every tick before the audit).
        queue_max = int(re.search(r"^GAS_CMD_QUEUE_MAX = (\d+)", ISAAC_APP, re.M).group(1))
        self.assertGreaterEqual(fg.WORLD_MAX_PENDING, 2 * queue_max)

    def test_view_changes_cannot_hide_a_later_ventilation_change(self):
        """T23/T42 (finding: the 0.5/s change budget was below view's 1/s
        input limit): two minutes of applied view changes use up their own
        budget, and a ventilation change after them is still its own record."""
        clock = FakeClock()
        guard, audit = fg.InputGuard(clock=clock), fg.FacadeAudit(clock=clock)
        view = fg.world_control_event("view", {"camera": "chase", "ui": None}, None, ["/x"])
        published = []
        for _ in range(120):
            clock.t += 1.0
            if guard.allow("view"):
                audit.record(*view, sim_time=clock.t)
            published += audit.drain()
        self.assertGreater(audit.suppressed, 0)            # view changes stay budgeted
        held = audit.suppressed
        vent = fg.world_control_event("gas/ventilation", {"on": True, "tau_s": 2.0}, None, ["/x"])
        self.assertTrue(audit.record(*vent, sim_time=clock.t))
        (text,) = audit.drain()
        record = json.loads(text)
        self.assertEqual((record["kind"], record["data"]["control"]),
                         ("world_control", "gas/ventilation"))
        self.assertEqual(audit.suppressed, held)

    def test_a_flood_at_the_input_limits_loses_no_safety_record(self):
        """T23/T42: isaac_app's path (rate limit in the callback, 64-entry
        queues, apply and drain on the sim thread) under two minutes of every
        control topic at 50 msg/s, half refused, with a 2 s sim-loop stall:
        every safety control that passed the limit has its own record."""
        clock = FakeClock()
        guard, audit = fg.InputGuard(clock=clock), fg.FacadeAudit(clock=clock)
        queue_max = int(re.search(r"^GAS_CMD_QUEUE_MAX = (\d+)", ISAAC_APP, re.M).group(1))
        queues = {"gas": collections.deque(), "ctl": collections.deque()}
        params = {"gas/inject": dict(PRESENTER_INJECT), "gas/clear": {},
                  "gas/ventilation": {"on": True, "tau_s": 30.0}, "battery/reset": {},
                  "view": {"camera": "auto", "ui": None}}
        applied, published, n = collections.Counter(), [], itertools.count()
        for step in range(6000):                       # 120 s at the 50 Hz sim tick
            clock.t = step * 0.02
            for topic in params:
                queue = queues["ctl" if topic in ("view", "battery/reset") else "gas"]
                if guard.allow(topic) and len(queue) < queue_max:
                    queue.append(topic)
            if 60.0 <= clock.t < 62.0:                 # the sim loop stalls
                continue
            for queue in queues.values():
                while queue:
                    topic = queue.popleft()
                    error = ValueError("refused by the bounds") if next(n) % 2 else None
                    audit.record(*fg.world_control_event(
                        topic, None if error else params[topic], error, ["/flood"]),
                        sim_time=clock.t)
                    applied[(topic, error is None)] += 1
            published += [json.loads(text) for text in audit.drain()]
        recorded = collections.Counter((r["data"]["control"], r["data"]["accepted"])
                                       for r in published if r["kind"] == "world_control")
        for key, count in applied.items():
            if key[0] in fg.SAFETY_CONTROLS:
                self.assertEqual(recorded[key], count, key)
        views = [("view", True), ("view", False)]
        self.assertLess(sum(recorded[k] for k in views), sum(applied[k] for k in views))  # budgeted
        held = collections.Counter()
        for record in published:
            held.update((record["data"].get("suppressed_before") or {}).get("by_kind", {}))
        self.assertEqual(set(held), {"world_control/view"})


class TestT23RateLimits(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.guard = fg.InputGuard(clock=self.clock)

    def test_every_queued_topic_has_a_limit(self):
        """T23/T42: each String control topic the facade queues is rate-limited."""
        for topic in ("gas/inject", "gas/clear", "gas/ventilation", "gas/probe", "view",
                      "battery/reset"):
            self.assertIn(topic, fg.CONTROL_RATE_LIMITS)
            self.assertIn('"%s"' % topic.split("/")[-1], ISAAC_APP)

    def test_flood_is_dropped_and_counted(self):
        """T23: a flood of injects is cut to the burst, then the sustained rate."""
        rate, burst = fg.CONTROL_RATE_LIMITS["gas/inject"]
        allowed = sum(self.guard.allow("gas/inject") for _ in range(1000))
        self.assertEqual(allowed, burst)
        self.assertEqual(self.guard.counters()["dropped"]["gas/inject"], 1000 - burst)
        self.clock.t += 1.0 / rate
        self.assertTrue(self.guard.allow("gas/inject"))
        self.assertTrue(self.guard.allow("view"))          # other topics unaffected

    def test_reset_spot_burst_passes(self):
        """T23: host/reset_spot.sh's clear + ventilation + battery + view pass at once."""
        for topic in ("gas/clear", "gas/ventilation", "battery/reset", "view"):
            self.assertTrue(self.guard.allow(topic), topic)
        self.assertTrue(self.guard.allow("gas/clear"))     # agent's clear + reset_tank right after

    def test_counters(self):
        """T23/T42: drops and refusals are counted per topic."""
        self.guard.drop("gas/probe")
        self.guard.reject("view")
        counters = self.guard.counters()
        self.assertEqual(counters["dropped"]["gas/probe"], 1)
        self.assertEqual(counters["rejected"]["view"], 1)
        counters["dropped"]["gas/probe"] = 99                # a copy
        self.assertEqual(self.guard.counters()["dropped"]["gas/probe"], 1)
        with self.assertRaises(KeyError):
            self.guard.allow("not/a/topic")

    def test_token_bucket(self):
        """T23/T42: burst, refill, cap and parameter checks."""
        bucket = fg.TokenBucket(2.0, 3)
        self.assertEqual([bucket.allow(0.0) for _ in range(4)], [True, True, True, False])
        self.assertTrue(bucket.allow(0.5))
        self.assertFalse(bucket.allow(0.5))
        self.assertEqual(sum(bucket.allow(100.0) for _ in range(10)), 3)   # refill capped
        for rate, burst in ((0.0, 1), (1.0, 0.5)):
            with self.assertRaises(ValueError):
                fg.TokenBucket(rate, burst)


# ---------------------------------------------------------------------------
# T21: probe ids
# ---------------------------------------------------------------------------


class TestT21ProbeId(unittest.TestCase):
    def test_contract_ids_are_accepted(self):
        """T21: the agent's token_hex(8) ids and any 1-64 printable ASCII id pass."""
        import secrets
        for value in (secrets.token_hex(8), "a", "x" * 64, "spot_mission_client_4242-7",
                      " ~!{}[]\"'"):
            self.assertTrue(fg.valid_probe_id(value), value)
        self.assertEqual(fg.PROBE_ID_MAX_CHARS, 64)

    def test_out_of_contract_ids_are_refused(self):
        """T21: empty, too long, non-strings and non-printable or non-ASCII ids."""
        for value in ("", "x" * 65, None, 7, ["id"], b"abc", "a\nb", "a\tb", "a\x7fb",
                      "a\x00b", "caf\u00e9", "a\u202eb"):
            self.assertFalse(fg.valid_probe_id(value), repr(value))

    def test_handle_probe_checks_the_id_and_never_reflects_a_bad_one(self):
        """T21: _handle_probe refuses a bad id, replies with "id": null, keeps a good one."""
        body = _function_source("_handle_probe")
        check = body.index("facade_guard.valid_probe_id(req_id)")
        self.assertLess(body.index("req_id = None  # never reflect"), body.index("raise", check))
        self.assertIn('"id": req_id, "samples": samples', body)   # good id echoed
        self.assertIn('facade_guard.parse_control_json(text, "gas/probe")', body)
        self.assertIn("reason = facade_guard.clean_text(error)", body)
        self.assertNotIn("_json_obj", ISAAC_APP)


# ---------------------------------------------------------------------------
# T44: no pass-through to Kit
# ---------------------------------------------------------------------------


class TestT44NoKitPassThrough(unittest.TestCase):
    def test_documented_launches_are_accepted(self):
        """T44: the argument sets the host scripts pass are all known."""
        parser = launch_args.build_parser()
        for argv in ([], ["--headless", "true", "--smoke", "--no-cameras"],   # smoke_test.sh
                     ["--headless", "true", "--no-cameras"],                 # e2e_ros_test.sh
                     ["--headless", "false", "--ui", "presentation"],        # run_sim.sh --gui
                     ["--cameras", "all", "--headless", "false", "--ui", "editor"],
                     ["--gas-initial", "ventilated", "--battery-drain", "off", "--view", "door"]):
            _args, unknown = parser.parse_known_args(argv)
            self.assertIsNone(fg.unknown_args_error(unknown), argv)

    def test_kit_overrides_and_typos_are_refused(self):
        """T44: extension switches, Kit settings and typos never reach Kit."""
        parser = launch_args.build_parser()
        for argv in (["--enable", "omni.kit.livestream.webrtc"],
                     ["--/app/livestream/enabled=true"],
                     ["--/exts/omni.services.transport.server.http/host=0.0.0.0"],
                     ["--headless", "true", "--stauts"], ["extra"]):
            _args, unknown = parser.parse_known_args(argv)
            message = fg.unknown_args_error(unknown)
            self.assertIsNotNone(message, argv)
            self.assertIn("unrecognized arguments", message)
            self.assertIn("passes nothing through to Kit", message)

    def test_message_is_cleaned(self):
        """T44: the refusal message is printable and bounded."""
        self.assertIsNone(fg.unknown_args_error([]))
        self.assertIsNone(fg.unknown_args_error(None))
        message = fg.unknown_args_error(["--x\n" + "y" * 500])
        self.assertNotIn("\n", message)
        self.assertLess(len(message), 250)

    def test_refused_before_simulation_app_starts(self):
        """T44: isaac_app.py calls parser.error before constructing SimulationApp."""
        refuse = ISAAC_APP.index("parser.error(_launch_error)")
        self.assertLess(ISAAC_APP.index("_launch_error = facade_guard.unknown_args_error(_unknown)"),
                        refuse)
        self.assertLess(refuse, ISAAC_APP.index('SimulationApp({"headless": HEADLESS})'))

    def test_isaac_app_parses_with_the_launch_args_parser(self):
        """T44: the parser tested here is the one isaac_app.py uses: it imports
        sim/launch_args.py (which imports only argparse) and builds and runs the
        parser from it before importing SimulationApp, defining no option itself."""
        tree = ast.parse(ISAAC_APP)
        top = [ast.get_source_segment(ISAAC_APP, node) for node in tree.body]
        order = ["import launch_args", "parser = launch_args.build_parser()",
                 "args, _unknown = parser.parse_known_args()", "from isaacsim import SimulationApp"]
        for line in order:
            self.assertIn(line, top)
        self.assertEqual([top.index(line) for line in order], sorted(top.index(line) for line in order))
        names = {getattr(node, "attr", getattr(node, "id", None)) for node in ast.walk(tree)}
        self.assertTrue(names.isdisjoint({"argparse", "ArgumentParser", "add_argument"}))
        module = ast.parse((REPO / "sim" / "launch_args.py").read_text(encoding="utf-8"))
        imported = {alias.name for node in ast.walk(module) if isinstance(node, (ast.Import, ast.ImportFrom))
                    for alias in node.names}
        self.assertEqual(imported, {"argparse"})


# ---------------------------------------------------------------------------
# T41: real-time factor
# ---------------------------------------------------------------------------


class TestT41RealTimeFactor(unittest.TestCase):
    def _tick(self, meter, ticks, wall_dt, sim_dt, wall=0.0, sim=0.0):
        for _ in range(ticks):
            meter.update(wall, sim)
            wall += wall_dt
            sim += sim_dt
        return wall, sim

    def test_none_until_the_first_window(self):
        """T41: no rtf before a full window has been measured."""
        meter = fg.RtfMeter()
        self.assertIsNone(meter.update(0.0, 0.0))
        self.assertIsNone(meter.update(1.9, 1.9))
        self.assertEqual(meter.update(2.0, 2.0), 1.0)

    def test_real_time_and_a_collapse(self):
        """T41: about 1.0 at real time; a collapse to 0.1 shows within one window."""
        meter = fg.RtfMeter()
        wall, sim = self._tick(meter, 300, 0.02, 0.02)          # 6 s at real time
        self.assertAlmostEqual(meter.rtf, 1.0, places=2)
        self._tick(meter, 150, 0.02, 0.002, wall, sim)          # 3 s at 0.1x
        self.assertAlmostEqual(meter.rtf, 0.1, places=2)

    def test_a_stalled_loop_reports_a_near_zero_window(self):
        """T41: a 10 s stall of the sim loop shows up at the next tick."""
        meter = fg.RtfMeter()
        meter.update(0.0, 0.0)
        meter.update(2.0, 2.0)
        self.assertEqual(meter.update(12.0, 2.2), 0.02)

    def test_bad_window_and_backwards_sim_time(self):
        """T41: window validated; sim time never yields a negative rtf."""
        with self.assertRaises(ValueError):
            fg.RtfMeter(0.0)
        meter = fg.RtfMeter(1.0)
        meter.update(0.0, 5.0)
        self.assertEqual(meter.update(1.0, 4.0), 0.0)

    def test_published_in_status_and_updated_every_tick(self):
        """T41: payload/status carries "rtf"; the main loop feeds the meter."""
        self.assertIn('"rtf": rtf_meter.rtf', _function_source("publish_status"))
        self.assertIn("rtf_meter = facade_guard.RtfMeter()", ISAAC_APP)
        loop = ISAAC_APP[ISAAC_APP.index("while RUNNING and simulation_app.is_running():"):]
        self.assertIn("        rtf_meter.update(time.monotonic(), state.sim_time)\n", loop)


# ---------------------------------------------------------------------------
# T42: floods on the control topics
# ---------------------------------------------------------------------------


class TestT42ControlTopicLimits(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.guard = fg.InputGuard(clock=self.clock)

    def _run(self, topic, hz, seconds):
        allowed = 0
        for i in range(int(hz * seconds)):
            self.clock.t = i / hz
            allowed += self.guard.allow(topic)
        return allowed

    def test_motion_topics_are_limited(self):
        """T42: cmd_vel and body_pose have their own limits."""
        self.assertEqual(fg.CONTROL_RATE_LIMITS["cmd_vel"], (100.0, 50))
        self.assertIn("body_pose", fg.CONTROL_RATE_LIMITS)

    def test_cmd_vel_flood_is_bounded(self):
        """T42: a 1 kHz cmd_vel flood for 2 s reaches the facade at about 100 Hz."""
        allowed = self._run("cmd_vel", 1000.0, 2.0)
        self.assertLessEqual(allowed, 50 + 100 * 2 + 1)
        self.assertEqual(self.guard.counters()["dropped"]["cmd_vel"], 2000 - allowed)

    def test_legitimate_rates_are_never_dropped(self):
        """T42: the mission and the guard together (40 Hz) and 2 Hz probes pass."""
        self.assertEqual(self._run("cmd_vel", 40.0, 30.0), 1200)
        self.assertEqual(self._run("gas/probe", 2.0, 30.0), 60)
        self.assertEqual(self.guard.counters()["dropped"], dict.fromkeys(fg.CONTROL_RATE_LIMITS, 0))

    def test_probe_flood_bounds_sim_thread_sampling(self):
        """T42: a probe flood (16 points each, sampled on the sim thread) is cut to 10 Hz."""
        allowed = self._run("gas/probe", 1000.0, 1.0)
        self.assertLessEqual(allowed, 10 + 10 + 1)
        self.assertLessEqual(allowed * 16, 21 * 16)   # samples per second, bounded


# ---------------------------------------------------------------------------
# sim/isaac_app.py call sites (it cannot run here)
# ---------------------------------------------------------------------------


class TestIsaacAppCallSites(unittest.TestCase):
    def test_cmd_vel_is_checked_before_any_claim(self):
        """T20: on_cmd_vel checks the Twist before the implicit claim + power-on."""
        body = _function_source("on_cmd_vel")
        self.assertLess(body.index("facade_guard.check_twist("), body.index("_auto_claim("))
        start = body.index("if cmd is None:")
        refused = body[start:body.index("return", start)]
        self.assertIn("np.zeros(3", refused)
        self.assertIn("state.last_cmd_time = -1.0e9", refused)
        self.assertIn("np.array(cmd,", body)

    def test_body_pose_is_checked(self):
        """T20: on_body_pose refuses malformed poses."""
        self.assertIn("facade_guard.check_body_pose(", _function_source("on_body_pose"))

    def test_every_service_is_audited_and_events_are_published(self):
        """T20: services register through _audited; records go out on payload/events."""
        self.assertIn("node.create_service(Trigger, name, _audited(name, cb))", ISAAC_APP)
        self.assertNotIn("node.create_service(Trigger, name, cb)", ISAAC_APP)
        self.assertIn('pub_events = node.create_publisher(String, "payload/events", EVENTS_QOS)',
                      ISAAC_APP)
        self.assertIn("durability=DurabilityPolicy.TRANSIENT_LOCAL", ISAAC_APP)
        self.assertIn("pub_events.publish(String(data=_record))", ISAAC_APP)
        self.assertIn("node.create_timer(facade_guard.PUBLISHER_WATCH_PERIOD_S, _watch_cmd_vel)",
                      ISAAC_APP)

    def test_string_controls_are_rate_limited_before_queueing(self):
        """T23/T42: _enqueue checks the limit and size before anything is queued."""
        body = _function_source("_enqueue")
        self.assertLess(body.index("inputs.allow(topic)"), body.index("queue.append("))
        self.assertIn("facade_guard.bounded(msg.data)", body)
        self.assertIn('_enqueue(_gas_cmds, _kind, f"gas/{_kind}")', ISAAC_APP)
        self.assertIn('_enqueue(_ctl_cmds, "view", "view")', ISAAC_APP)
        self.assertIn('_enqueue(_ctl_cmds, "battery_reset", "battery/reset")', ISAAC_APP)
        self.assertNotIn("lambda m: _ctl_cmds.append", ISAAC_APP)

    def test_world_controls_are_validated_and_audited(self):
        """T23: every world control is checked by facade_guard and audited both ways."""
        gas_body = _function_source("apply_gas_commands")
        for call in ("facade_guard.validate_inject(", "facade_guard.validate_clear(",
                     "facade_guard.validate_ventilation(", "facade_guard.parse_control_json("):
            self.assertIn(call, gas_body)
        ctl_body = _function_source("apply_control_commands")
        self.assertIn("facade_guard.validate_battery_reset(", ctl_body)
        self.assertIn("facade_guard.control_text(text, control)", ctl_body)
        for body in (gas_body, ctl_body):
            self.assertIn("_audit_control(control, pubs, params=params", body)
            self.assertIn("_audit_control(control, pubs, error=error)", body)
        # The validator runs before the field changes.
        self.assertLess(gas_body.index("validate_inject("), gas_body.index("gas.inject_spike("))

    def test_motion_topics_are_rate_limited_first(self):
        """T42: on_cmd_vel / on_body_pose check the limit before anything else."""
        for name, topic in (("on_cmd_vel", "cmd_vel"), ("on_body_pose", "body_pose")):
            body = _function_source(name)
            code = body[body.index('"""') if '"""' in body else 0:]
            first = code.index('inputs.allow("%s")' % topic)
            self.assertLess(first, code.index("facade_guard.check_"), name)
            self.assertIn('inputs.reject("%s")' % topic, body)

    def test_status_identifies_the_simulator(self):
        """Backend identity contract (T9/M9, which T20-T23 rely on): payload/status
        always carries "simulated": true."""
        self.assertIn('"simulated": True,', _function_source("publish_status"))

    def test_status_reports_drops_and_refusals(self):
        """T42: payload/status carries the per-topic counters and suppressed records."""
        body = _function_source("publish_status")
        self.assertIn("**inputs.counters()", body)
        self.assertIn('"events_suppressed": audit.suppressed', body)
        self.assertIn('"cmd_vel_publishers": cmd_vel_watch.count', body)


if __name__ == "__main__":
    unittest.main()
