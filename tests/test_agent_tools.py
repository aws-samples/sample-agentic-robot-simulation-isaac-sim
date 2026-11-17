"""Unit tests for agent/spot_agent.py tools -- no Bedrock, no ROS, no rclpy.

Also home of ``FakeRobot`` and ``FakeMission``, the in-memory stand-ins for
agent/robot_client.RobotClient (shared contract 2) and
agent/mission.ConfinedSpaceMission (shared contract 3). ``spot_agent.py
--dry-run`` imports them so the model can be exercised end to end without
the simulator.

Run from the repo root:

    python3 -m unittest tests.test_agent_tools -v
    # or
    python3 -m pytest tests/test_agent_tools.py

The Strands tool-spec test runs only where strands-agents is installed and is
skipped otherwise.
"""

import copy
import io
import logging
import math
import os
import sys
import tempfile
import threading
import time
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from agent import spot_agent  # noqa: E402
from agent.prompts import build_system_prompt  # noqa: E402
from agent.spot_agent import (  # noqa: E402
    NO_ROUTE,
    PRESENTER_ENV,
    TOOL_NAMES,
    SpotTools,
    TranscriptCallback,
    _permit_state,
    _test_passed,
)

# Values copied from sim/build_scene.py build_scene() (same as the MISSION
# module's agent/site_defaults.json). Kept inline so these tests do not depend
# on another module's file.
DEFAULT_SITE = {
    "spawn_xy": [14.0, 0.0],
    "spawn_yaw": math.pi,
    "entry_xy": [7.0, 0.0],
    "door_xy": [6.0, 0.0],
    "door_normal_yaw": 0.0,
    "interior_waypoints": [[4.6, 0.0], [2.5, 1.6], [0.0, 0.0], [-2.0, -1.7], [-4.8, 0.0]],
    "tank_center_xy": [0.0, 0.0],
    "tank_radius": 6.0,
}

_BASELINE = {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0, "co_ppm": 0.0}
_GAS_KEY = {"o2": "o2_pct", "lel": "lel_pct", "h2s": "h2s_ppm", "co": "co_ppm"}


def _osha_alarms(sample):
    """OSHA 1910.146 App E alarms, same thresholds as sim/gas_field.py."""
    alarms = {
        "o2": sample["o2_pct"] < 19.5 or sample["o2_pct"] > 23.5,
        "lel": sample["lel_pct"] >= 10.0,
        "h2s": sample["h2s_ppm"] >= 10.0,
        "co": sample["co_ppm"] >= 35.0,
    }
    alarms["any"] = any(alarms.values())
    return alarms


class FakeRobot:
    """In-memory RobotClient (contract 2). Records every call in ``calls``
    and every published event in ``events``. Gas is baseline plus any injected
    spike whose radius covers the robot (a flat disc, not a Gaussian: tests
    want crisp in/out behaviour)."""

    def __init__(self, site=None):
        self.site_dict = copy.deepcopy(site or DEFAULT_SITE)
        self.x, self.y = self.site_dict["spawn_xy"]
        self.yaw = self.site_dict.get("spawn_yaw", 0.0)
        # "simulated": true identifies the Isaac Sim facade (backend contract).
        self.state = {"claimed": False, "powered": False, "standing": False,
                      "standing_up": False, "estopped": False, "battery_pct": 100.0,
                      "sim_time": 0.0, "simulated": True}
        self.calls = []
        self.events = []
        self.spikes = []
        self.telemetry = True      # False: pose/gas/status return None
        self.stand_delay_polls = 0  # status() polls before a stand completes
        self.stand_never = False    # stand accepted but never completes
        self.motion_owner = None    # RobotClient: "retreat" while a retreat owns cmd_vel

    # -- recording helpers (test-only) --
    def _call(self, name, *args):
        self.calls.append((name,) + args)

    def names(self):
        return [c[0] for c in self.calls]

    def called(self, name):
        return [c for c in self.calls if c[0] == name]

    def set_pose(self, x, y, yaw=None):
        self.x, self.y = float(x), float(y)
        if yaw is not None:
            self.yaw = yaw

    # -- contract 2 surface --
    def connect(self, timeout=30):
        self._call("connect", timeout)
        return True

    def claim(self):
        self._call("claim")
        self.state["claimed"] = True
        return True, "Claimed lease"

    def release(self):
        self._call("release")
        self.state["claimed"] = False
        return True, "Released lease"

    def power_on(self):
        self._call("power_on")
        self.state["claimed"] = True
        if self.state["estopped"]:
            return False, "Exception while powering on: e-stop is asserted"
        self.state["powered"] = True
        return True, "Powered on"

    def power_off(self):
        self._call("power_off")
        self.state.update(powered=False, standing=False)
        return True, "Safe power off"

    def stand(self):
        self._call("stand")
        self.state["claimed"] = True
        if not self.state["estopped"]:
            self.state["powered"] = True
        if not self.state["powered"]:
            return False, "Exception while standing: motor power is off (e-stop asserted)"
        if self.stand_delay_polls or self.stand_never:
            self.state["standing_up"] = True
        else:
            self.state["standing"] = True
        return True, "Standing"

    def sit(self):
        self._call("sit")
        self.state.update(standing=False, standing_up=False)
        return True, "Sitting"

    def stop(self):
        self._call("stop")
        return True, "Stopped"

    def estop_hard(self):
        self._call("estop_hard")
        self.state.update(estopped=True, powered=False, standing=False, standing_up=False)
        return True, "Hard e-stop asserted (motor power cut)"

    def estop_gentle(self):
        self._call("estop_gentle")
        self.state.update(estopped=True, powered=False, standing=False, standing_up=False)
        return True, "Gentle e-stop: settled, then motor power cut"

    def estop_release(self):
        self._call("estop_release")
        self.state["estopped"] = False
        return True, "E-stop released"

    def pose(self):
        return (self.x, self.y, self.yaw) if self.telemetry else None

    def gas(self):
        if not self.telemetry:
            return None
        sample = dict(_BASELINE)
        for gas, ppm, sx, sy, _sz, radius in self.spikes:
            if math.hypot(self.x - sx, self.y - sy) <= radius:
                sample[_GAS_KEY[gas]] += ppm
        alarms = _osha_alarms(sample)
        sample["alarms"] = alarms
        sample["alarm"] = alarms["any"]
        return sample

    def status(self):
        if not self.telemetry:
            return None
        if self.state["standing_up"] and not self.stand_never:
            self.stand_delay_polls -= 1
            if self.stand_delay_polls <= 0:
                self.state.update(standing=True, standing_up=False)
        return dict(self.state)

    def site(self):
        return copy.deepcopy(self.site_dict)

    def send_velocity(self, vx, vy, wz):
        self._call("send_velocity", vx, vy, wz)

    def inject_gas(self, gas, ppm, x, y, z, radius):
        self._call("inject_gas", gas, ppm, x, y, z, radius)
        self.spikes.append((gas, ppm, x, y, z, radius))

    def publish_event(self, kind, message, data=None, source="mission"):
        self.events.append({"source": source, "kind": kind, "message": message, "data": data})

    def close(self):
        self._call("close")


class FakeRobotNoSource(FakeRobot):
    """A RobotClient whose publish_event has exactly the contract-2 signature
    (no ``source`` keyword)."""

    def publish_event(self, kind, message, data=None):
        self.events.append({"source": "mission", "kind": kind, "message": message, "data": data})


class FakeMission:
    """In-memory ConfinedSpaceMission (contract 3). ``walk_to`` teleports the
    FakeRobot; arriving in a gas alarm emulates the SafetyGuard (retreat, then
    walk_to returns False)."""

    def __init__(self, robot, site=None, log_path=None):
        self.robot = robot
        self.site = copy.deepcopy(site or robot.site())
        self.log_path = log_path
        self.permit = "NOT_STARTED"
        self.calls = []
        self.raise_on_walk = None     # exception instance to raise from walk_to
        self.walk_ok = True
        self.test_result = None       # override atmospheric_test() return value

    def walk_to(self, x, y, tol=0.25, max_speed=0.5, timeout=60):
        self.calls.append(("walk_to", round(x, 3), round(y, 3), max_speed))
        if self.raise_on_walk is not None:
            raise self.raise_on_walk
        if not self.walk_ok or not self.robot.state["standing"]:
            return False
        self.robot.set_pose(x, y)
        if self.robot.gas()["alarm"]:
            self._guard_retreat()
            return False
        return True

    def _guard_retreat(self):
        self.calls.append(("guard_retreat",))
        self.permit = "ABORTED"
        self.robot.set_pose(*self.site["entry_xy"])
        self.robot.sit()

    def atmospheric_test(self, at_xy, dwell_s=4):
        self.calls.append(("atmospheric_test", tuple(at_xy), dwell_s))
        self.permit = "PRE_ENTRY_TESTING"
        if self.test_result is not None:
            return copy.deepcopy(self.test_result)
        g = self.robot.gas()
        steps = [{"gas": gas, "value": g[_GAS_KEY[gas]], "pass": not g["alarms"][gas]}
                 for gas in ("o2", "lel", "h2s", "co")]
        passed = all(s["pass"] for s in steps)
        if passed:
            self.permit = "ENTRY_AUTHORIZED"
        return {"passed": passed, "steps": steps}

    def run(self, auto_retreat=True):
        self.calls.append(("run", auto_retreat))
        self.robot.stand()
        test = self.atmospheric_test(self.site["entry_xy"])
        if not test["passed"]:
            self.permit = "ABORTED"
            return {"outcome": "ABORTED", "reason": "atmospheric test failed", "test": test}
        self.permit = "ENTRY_IN_PROGRESS"
        for wp in [self.site["door_xy"]] + self.site["interior_waypoints"]:
            self.robot.set_pose(*wp)
            if self.robot.gas()["alarm"] and auto_retreat:
                self._guard_retreat()
                return {"outcome": "ABORTED", "reason": "gas alarm", "at": wp}
        self.robot.set_pose(*self.site["entry_xy"])
        self.robot.sit()
        self.permit = "COMPLETED"
        return {"outcome": "COMPLETED", "test": test}

    def retreat(self):
        self.calls.append(("retreat",))
        self.permit = "ABORTED"
        self.robot.set_pose(*self.site["entry_xy"])
        self.robot.sit()

    def status_snapshot(self):
        return {"permit_state": self.permit, "calls": len(self.calls)}


class FakeFullMission(FakeMission):
    """FakeMission plus the extra step-by-step API of the real agent/mission.py
    (door_mouth_xy, enter, exit_space, new_permit, cancel, close) and its
    walk_to entry gate: targets inside the permit space are refused unless
    the permit is ENTRY_AUTHORIZED / ENTRY_IN_PROGRESS."""

    def __init__(self, robot, site=None, log_path=None):
        super().__init__(robot, site, log_path)
        door = self.site["door_xy"]
        self.door_mouth_xy = (door[0] + 0.6, door[1])  # the door faces +x
        self.closed = False

    def _in_space(self, x, y):
        cx, cy = self.site["tank_center_xy"]
        return math.hypot(x - cx, y - cy) < self.site["tank_radius"] + 0.3

    def walk_to(self, x, y, tol=0.25, max_speed=0.5, timeout=60):
        if self._in_space(x, y):
            if self.permit == "ENTRY_AUTHORIZED":
                self.permit = "ENTRY_IN_PROGRESS"
            elif self.permit != "ENTRY_IN_PROGRESS":
                self.calls.append(("entry_blocked", round(x, 3), round(y, 3)))
                return False
        return super().walk_to(x, y, tol, max_speed, timeout)

    def atmospheric_test(self, at_xy, dwell_s=4):
        # Real semantics: walks to at_xy; only a PRE-ENTRY test can authorize.
        pre_entry = self.permit in ("NOT_STARTED", "PRE_ENTRY_TESTING")
        self.calls.append(("atmospheric_test", tuple(at_xy), dwell_s))
        self.robot.set_pose(*at_xy)
        g = self.robot.gas()
        passed = not g["alarm"]
        if pre_entry:
            self.permit = "ENTRY_AUTHORIZED" if passed else "PRE_ENTRY_TESTING"
        return {"passed": passed, "results": [], "failures": [] if passed else ["alarm"]}

    def enter(self, on_waypoint=None):
        self.calls.append(("enter",))
        if self.permit not in ("ENTRY_AUTHORIZED", "ENTRY_IN_PROGRESS"):
            return False
        if not self.walk_to(*self.site["door_xy"]):
            return False
        for index, (wx, wy) in enumerate(self.site["interior_waypoints"]):
            if not self.walk_to(wx, wy):
                return False
            if on_waypoint is not None:
                on_waypoint(index, wx, wy)
        return True

    def exit_space(self):
        self.calls.append(("exit_space",))
        if self.permit != "ENTRY_IN_PROGRESS":
            return False
        self.robot.set_pose(*self.site["entry_xy"])
        self.permit = "COMPLETED"
        self.robot.sit()
        return True

    def new_permit(self):
        self.calls.append(("new_permit",))
        if self.permit not in ("COMPLETED", "ABORTED", "NOT_STARTED"):
            return False
        self.permit = "NOT_STARTED"
        return True

    def cancel(self, reason="operator cancel"):
        self.calls.append(("cancel", reason))
        if self.permit in ("PRE_ENTRY_TESTING", "ENTRY_AUTHORIZED", "ENTRY_IN_PROGRESS"):
            self.permit = "ABORTED"

    def close(self):
        self.calls.append(("close",))
        self.closed = True

    def log_event(self, kind, message, data=None, source="mission"):
        self.calls.append(("log_event", kind, source))
        return {"kind": kind, "message": message, "data": data, "source": source}

    def status_snapshot(self):
        return {"permit": self.permit, "retreating": False, "calls": len(self.calls)}


class FakeStorylineMission(FakeFullMission):
    """FakeFullMission plus the storyline API of agent/mission.py: permit
    fields and their approach gate, stand_up, SIMULATED ventilation, gas
    clearing, and SafetyGuard incidents (``trip``/``_guard_retreat``)."""

    FIELDS = ("isolation_confirmed", "loto_confirmed", "attendant", "rescue_plan",
              "entry_supervisor")

    def __init__(self, robot, site=None, log_path=None):
        super().__init__(robot, site, log_path)
        self._blank_permit()
        self.vent = {"on": False, "purged": False}
        self.incident_serial = 0
        self.incident = None
        self.self_rights = 0

    def _blank_permit(self):
        self.fields = {"permit_id": "CSE-FAKE", "isolation_confirmed": False,
                       "loto_confirmed": False, "attendant": "", "rescue_plan": "",
                       "entry_supervisor": ""}

    def permit_missing(self):
        return [k for k in self.FIELDS if not self.fields[k]]

    def prepare_permit(self, permit_id=None, isolation_confirmed=None, loto_confirmed=None,
                       attendant=None, rescue_plan=None, source="mission", prepared_by=None,
                       entry_supervisor=None, attestation=None):
        self.calls.append(("prepare_permit", source))
        if self.permit not in ("NOT_STARTED", "PRE_ENTRY_TESTING"):
            return {"ok": False, "complete": False, "missing": self.permit_missing(),
                    "errors": ["permit is %s" % self.permit], "permit": dict(self.fields)}
        attested = isinstance(attestation, dict) and attestation.get("decision") == "confirmed"
        if (isolation_confirmed or loto_confirmed) and not attested:   # like the real mission
            return {"ok": False, "complete": False, "missing": self.permit_missing(),
                    "errors": ["isolation/LOTO need the operator's attestation"],
                    "permit": dict(self.fields)}
        for key, value in (("permit_id", permit_id), ("isolation_confirmed", isolation_confirmed),
                           ("loto_confirmed", loto_confirmed), ("attendant", attendant),
                           ("rescue_plan", rescue_plan), ("entry_supervisor", entry_supervisor)):
            if value is not None:
                self.fields[key] = value
        if attested:
            self.calls.append(("attestation", attestation.get("answer")))
        missing = self.permit_missing()
        return {"ok": True, "complete": not missing, "missing": missing, "errors": [],
                "permit": dict(self.fields)}

    def new_permit(self):
        ok = super().new_permit()
        if ok:
            self._blank_permit()
        return ok

    def approach_refusal(self, x, y):
        if self.permit_missing() and math.hypot(x, y) < 7.5:
            return "the entry permit is incomplete"
        return None

    def walk_to(self, x, y, tol=0.25, max_speed=0.5, timeout=60):
        if self.approach_refusal(x, y):
            self.calls.append(("permit_blocked", round(x, 3), round(y, 3)))
            return False
        return super().walk_to(x, y, tol, max_speed, timeout)

    def atmospheric_test(self, at_xy, dwell_s=4):
        if self.permit_missing():
            return {"passed": False, "results": [], "failures": ["permit incomplete"]}
        return super().atmospheric_test(at_xy, dwell_s)

    def stand_up(self):
        self.calls.append(("stand_up",))
        self.robot.stand()
        return {"ok": True, "self_right_attempts": self.self_rights, "body_z_m": 0.52}

    def start_ventilation(self, tau_s=None, source="mission"):
        self.calls.append(("start_ventilation", tau_s, source))
        self.vent["on"] = True
        return {"ok": True, "on": True, "tau_s": tau_s or 15.0, "purge_estimate_s": 45.0,
                "simulated": True}

    def ventilation_status(self):
        return {"on": self.vent["on"], "purged": self.vent["purged"], "simulated": True,
                "percent_remaining": 8.0 if self.vent["purged"] else 100.0}

    def await_ventilation(self, max_s=None, source="mission"):
        self.calls.append(("await_ventilation", max_s, source))
        self.vent["purged"] = self.vent["on"]
        return dict(self.ventilation_status(), ok=self.vent["purged"], waited_s=max_s)

    def reset_atmosphere(self, source="mission"):
        self.calls.append(("reset_atmosphere", source))
        self.robot.spikes.clear()
        self.vent = {"on": False, "purged": False}
        return {"ok": True}

    def clear_injected_gas(self, source="mission"):
        self.calls.append(("clear_injected_gas", source))
        self.robot.spikes.clear()
        return {"ok": True}

    def run(self, auto_retreat=True, storyline=False):
        self.calls.append(("run_storyline", auto_retreat, storyline))
        return super().run(auto_retreat)

    def trip(self, gas="h2s", reading=42.0, where="waypoint_3"):
        """What the real mission records when its SafetyGuard fires."""
        self.incident_serial += 1
        self.incident = {"found": True, "incident_id": self.incident_serial,
                         "trigger": {"gases_in_alarm": [gas.upper()],
                                     "reading": {"h2s_ppm": reading}},
                         "location": {"nearest_place": where, "inside_permit_space": True},
                         "retreat": {"status": "complete"}, "permit_after": {"state": "ABORTED"}}

    def _guard_retreat(self):
        super()._guard_retreat()
        self.trip()

    def last_incident(self):
        return copy.deepcopy(self.incident) if self.incident else {"found": False}


def setUpModule():
    # The tools log expected failures (injected exceptions) loudly; keep the
    # unittest output readable. Done here, not at import, because
    # spot_agent.py --dry-run imports this module for its fakes.
    logging.getLogger("spot_agent").setLevel(logging.CRITICAL)


def tearDownModule():
    logging.getLogger("spot_agent").setLevel(logging.NOTSET)


class FakeClock:
    """Deterministic clock/sleep pair so wait loops run instantly."""

    def __init__(self):
        self.t = 1000.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


# ---------------------------------------------------------------------------


class ToolTestCase(unittest.TestCase):
    robot_cls = FakeRobot
    mission_cls = FakeMission

    def setUp(self):
        self.robot = self.robot_cls()
        self.mission = self.mission_cls(self.robot)
        self.env = {}
        self.clock = FakeClock()
        self.tools = SpotTools(self.robot, self.mission, self.robot.site(), env=self.env,
                               sleep=self.clock.sleep, clock=self.clock.now)
        # The operator types "y" at the console (agent/operator_console.py)
        # unless a test says otherwise; the model never answers these.
        self.answers = []
        self.tools.pending.responder = lambda item: (self.answers.append(item.action), "y")[1]

    def put(self, place, standing=True):
        """Teleport the robot to a named place (or an (x, y)) and reset records."""
        xy = place if isinstance(place, tuple) else dict(self.tools.places)[place]
        self.robot.set_pose(*xy)
        self.robot.state.update(claimed=True, powered=True, standing=standing)
        self.robot.calls.clear()
        self.robot.events.clear()
        self.mission.calls.clear()

    def walks(self):
        return [(c[1], c[2]) for c in self.mission.calls if c[0] == "walk_to"]

    def xy(self, place):
        return tuple(dict(self.tools.places)[place])


class TestEvents(ToolTestCase):
    def _call_every_tool(self):
        # Safe arguments for every tool; refusals still publish call+result.
        self.put("entry")
        return {
            "get_status": lambda: self.tools.get_status(),
            "get_gas_reading": lambda: self.tools.get_gas_reading(),
            "prepare_permit": lambda: self.tools.prepare_permit(rescue_plan="Standby team"),
            "claim_and_power_on": lambda: self.tools.claim_and_power_on(),
            "stand_up": lambda: self.tools.stand_up(),
            "sit_down": lambda: self.tools.sit_down(),
            "walk_to": lambda: self.tools.walk_to(x=9.0, y=0.0),
            "walk_to_named": lambda: self.tools.walk_to_named(place="entry"),
            "run_atmospheric_test": lambda: self.tools.run_atmospheric_test(),
            "start_ventilation": lambda: self.tools.start_ventilation(),
            "ventilation_status": lambda: self.tools.ventilation_status(),
            "enter_confined_space": lambda: self.tools.enter_confined_space(),
            "full_confined_space_entry": lambda: self.tools.full_confined_space_entry(),
            "retreat_now": lambda: self.tools.retreat_now(),
            "explain_last_incident": lambda: self.tools.explain_last_incident(),
            "emergency_stop": lambda: self.tools.emergency_stop(),
            "inject_gas_demo_spike": lambda: self.tools.inject_gas_demo_spike(),
            "clear_injected_gas": lambda: self.tools.clear_injected_gas(),
        }

    def test_every_tool_publishes_call_and_result_with_agent_source(self):
        calls = self._call_every_tool()
        self.assertEqual(set(calls), set(TOOL_NAMES))
        for name in TOOL_NAMES:
            self.robot.events.clear()
            self.robot.state["standing"] = True
            result = calls[name]()
            self.assertIsInstance(result, dict, name)
            kinds = [(e["kind"], (e["data"] or {}).get("tool")) for e in self.robot.events]
            self.assertIn(("tool_call", name), kinds, name)
            self.assertIn(("tool_result", name), kinds, name)
            self.assertLess(kinds.index(("tool_call", name)), kinds.index(("tool_result", name)))
            for event in self.robot.events:
                self.assertEqual(event["source"], "agent", (name, event))

    def test_tool_result_event_carries_args_result_and_duration(self):
        self.put("spawn")
        self.tools.walk_to_named(place="entry")
        call = next(e for e in self.robot.events if e["kind"] == "tool_call")
        done = next(e for e in self.robot.events if e["kind"] == "tool_result")
        self.assertEqual(call["data"]["args"], {"place": "entry"})
        self.assertTrue(done["data"]["ok"])
        self.assertIn("duration_s", done["data"])
        self.assertEqual(done["data"]["result"]["visited"], ["entry"])

    def test_say_and_operator_events(self):
        self.tools.say("Oxygen is 20.9 percent.")
        self.tools.operator("What is the oxygen level?")
        kinds = [(e["kind"], e["message"], e["source"]) for e in self.robot.events]
        self.assertIn(("say", "Oxygen is 20.9 percent.", "agent"), kinds)
        self.assertIn(("operator", "What is the oxygen level?", "agent"), kinds)

    def test_publish_failure_never_breaks_a_tool(self):
        def boom(*_a, **_k):
            raise RuntimeError("DDS down")
        self.robot.publish_event = boom
        tools = SpotTools(self.robot, self.mission, self.robot.site(), env={})
        self.assertTrue(tools.get_gas_reading()["ok"])


class TestEventsWithoutSourceKwarg(ToolTestCase):
    robot_cls = FakeRobotNoSource

    def test_source_is_tagged_in_data(self):
        self.tools.get_gas_reading()
        self.assertTrue(self.robot.events)
        for event in self.robot.events:
            self.assertEqual(event["data"]["source"], "agent")


class TestStateTools(ToolTestCase):
    def test_claim_and_power_on_calls_claim_then_power_on(self):
        result = self.tools.claim_and_power_on()
        self.assertTrue(result["ok"])
        self.assertEqual(self.robot.names(), ["claim", "power_on"])

    def test_claim_and_power_on_reports_estop(self):
        self.robot.state["estopped"] = True
        result = self.tools.claim_and_power_on()
        self.assertFalse(result["ok"])
        self.assertIn("e-stop", result["power_on"])

    def test_stand_up_waits_until_standing(self):
        self.robot.stand_delay_polls = 5
        result = self.tools.stand_up()
        self.assertTrue(result["ok"])
        self.assertEqual(self.robot.called("stand"), [("stand",)])
        self.assertTrue(self.robot.state["standing"])

    def test_stand_up_times_out_without_hanging(self):
        self.robot.stand_never = True
        result = self.tools.stand_up()
        self.assertFalse(result["ok"])
        self.assertIn("not standing", result["error"])

    def test_sit_down_zero_twist_stop_then_sit(self):
        self.put("entry")
        self.assertTrue(self.tools.sit_down()["ok"])
        self.assertEqual(self.robot.calls, [("send_velocity", 0.0, 0.0, 0.0), ("stop",), ("sit",)])

    def test_get_gas_reading(self):
        result = self.tools.get_gas_reading()
        self.assertTrue(result["ok"])
        self.assertEqual(result["o2_pct"], 20.9)
        self.assertFalse(result["alarm"])
        self.assertEqual(result["alarming_gases"], [])
        self.robot.telemetry = False
        self.assertFalse(self.tools.get_gas_reading()["ok"])

    def test_get_status(self):
        self.put("waypoint_3")
        result = self.tools.get_status()
        self.assertTrue(result["ok"])
        self.assertEqual(result["nearest_place"], "waypoint_3")
        self.assertTrue(result["inside_tank"])
        self.assertEqual(result["mission"]["permit_state"], "NOT_STARTED")
        self.assertFalse(result["entry_authorized"])


class TestMovement(ToolTestCase):
    def test_walk_to_outside_uses_mission_walk_to(self):
        self.put("entry")
        result = self.tools.walk_to(x=10.0, y=-1.0)
        self.assertTrue(result["ok"])
        self.assertEqual(self.walks(), [(10.0, -1.0)])

    def test_walk_to_refuses_into_or_through_the_tank(self):
        self.put("entry")
        for x, y in [(0.0, 0.0), (6.2, 0.0), (-10.0, 0.0), (100.0, 0.0)]:
            result = self.tools.walk_to(x=x, y=y)
            self.assertFalse(result["ok"], (x, y))
            self.assertTrue(result["refused"], (x, y))
        self.put("waypoint_2")
        self.assertTrue(self.tools.walk_to(x=2.0, y=2.0)["refused"])
        self.assertEqual(self.walks(), [])

    def test_walk_to_refuses_when_not_standing(self):
        self.put("entry", standing=False)
        result = self.tools.walk_to(x=10.0, y=0.0)
        self.assertTrue(result["refused"])
        self.assertIn("stand_up", result["error"])
        self.assertEqual(self.walks(), [])

    def test_walk_to_named_entry_from_spawn(self):
        self.put("spawn")
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["ok"])
        self.assertEqual(self.walks(), [self.xy("entry")])

    def test_walk_to_named_normalizes_and_rejects_unknown(self):
        self.put("spawn")
        self.assertTrue(self.tools.walk_to_named(place=" Entry ")["ok"])
        result = self.tools.walk_to_named(place="ladder room")
        self.assertTrue(result["refused"])
        self.assertIn("waypoint_5", result["error"])

    def test_inward_move_requires_authorization(self):
        self.put("entry")
        for place in ("door", "waypoint_1", "waypoint_4"):
            result = self.tools.walk_to_named(place=place)
            self.assertTrue(result["refused"], place)
            self.assertIn("not authorized", result["error"])
        self.assertEqual(self.walks(), [])

    def test_exit_is_allowed_without_authorization_and_retraces_waypoints(self):
        self.put("waypoint_3")
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.walks(), [self.xy("waypoint_2"), self.xy("waypoint_1"),
                                        self.xy("door"), self.xy("entry")])

    def test_far_side_of_tank_has_no_straight_route(self):
        self.put((-10.0, 0.0))
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["refused"])
        self.assertEqual(result["error"], NO_ROUTE)
        self.assertEqual(self.walks(), [])

    def test_gas_alarm_inside_leaves_the_robot_to_the_guard(self):
        self.put("waypoint_1")
        self.robot.spikes.append(("h2s", 50.0, 4.6, 0.0, 0.5, 1.0))
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["refused"])
        self.assertIn("SafetyGuard", result["error"])
        self.assertEqual(self.walks(), [])

    def test_gas_alarm_outside_blocks_entry_but_allows_backing_off(self):
        self.put("entry")
        self.robot.spikes.append(("h2s", 50.0, 7.0, 0.0, 0.5, 1.0))
        self.tools._auth = {"permit_at_auth": None, "t": self.clock.now()}  # even when authorized
        result = self.tools.walk_to_named(place="door")
        self.assertTrue(result["refused"])
        self.assertIn("entry is not allowed", result["error"])
        self.assertTrue(self.tools.enter_confined_space()["refused"])
        self.assertEqual(self.walks(), [])
        self.assertTrue(self.tools.walk_to_named(place="spawn")["ok"])
        self.assertEqual(self.walks(), [self.xy("spawn")])

    def test_motion_refused_while_a_retreat_owns_cmd_vel(self):
        self.put("waypoint_2")
        self.robot.motion_owner = "retreat"
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["refused"])
        self.assertIn("retreat", result["error"])
        self.assertEqual(self.walks(), [])

    def test_motion_refused_on_stale_status(self):
        self.put("spawn")
        self.tools.backend()  # identified as the simulator at startup (await_simulator)
        self.robot.status = lambda: None  # RobotClient returns None when stale
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["refused"])
        self.assertIn("telemetry", result["error"])
        self.assertEqual(self.walks(), [])

    def test_motion_lock_rejects_a_second_concurrent_move(self):
        self.put("spawn")
        self.assertTrue(self.tools._motion_lock.acquire(blocking=False))
        try:
            result = self.tools.walk_to_named(place="entry")
        finally:
            self.tools._motion_lock.release()
        self.assertTrue(result["refused"])
        self.assertEqual(self.walks(), [])


class TestPermitGate(ToolTestCase):
    def test_atmospheric_test_walks_to_entry_then_tests_and_authorizes(self):
        self.put("spawn")
        result = self.tools.run_atmospheric_test()
        self.assertTrue(result["passed"])
        self.assertTrue(result["entry_authorized"])
        self.assertEqual(self.walks(), [self.xy("entry")])
        self.assertIn(("atmospheric_test", self.xy("entry"), 4.0), self.mission.calls)
        permit = [e for e in self.robot.events if e["kind"] == "permit"]
        self.assertTrue(permit and permit[-1]["data"]["entry_authorized"])

    def test_failed_test_blocks_entry(self):
        self.put("entry")
        self.mission.test_result = {"passed": False, "failed": ["h2s"]}
        result = self.tools.run_atmospheric_test()
        self.assertFalse(result["passed"])
        self.assertTrue(self.tools.enter_confined_space()["refused"])
        self.assertTrue(self.tools.walk_to_named(place="door")["refused"])
        self.assertEqual(self.walks(), [])

    def test_unrecognized_test_result_fails_safe(self):
        self.put("entry")
        self.mission.test_result = {"o2": 20.9}
        self.mission.permit = "PRE_ENTRY_TESTING"
        self.assertFalse(self.tools.run_atmospheric_test()["passed"])
        self.assertTrue(self.tools.enter_confined_space()["refused"])

    def test_enter_follows_door_then_every_waypoint_in_order(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.mission.calls.clear()
        result = self.tools.enter_confined_space()
        self.assertTrue(result["ok"], result)
        expected = [self.xy("door")] + [self.xy("waypoint_%d" % i) for i in range(1, 6)]
        self.assertEqual(self.walks(), expected)
        self.assertEqual(len(result["readings"]), 6)
        self.assertEqual(result["peak"]["h2s_ppm"], 0.0)
        # Inside legs are driven slower than outside ones.
        self.assertTrue(all(c[3] < 0.5 for c in self.mission.calls if c[0] == "walk_to"))

    def test_guard_trip_stops_the_entry_leg(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        wp3 = self.xy("waypoint_3")
        self.robot.spikes.append(("h2s", 40.0, wp3[0], wp3[1], 0.5, 0.5))
        self.mission.calls.clear()
        self.robot.calls.clear()
        result = self.tools.enter_confined_space()
        self.assertFalse(result["ok"])
        self.assertTrue(result["guard_tripped"])
        self.assertEqual(self.walks()[-1], wp3)  # nothing issued after the trip
        self.assertIn(("guard_retreat",), self.mission.calls)
        # The agent must not fight the guard's retreat with a stop command.
        self.assertNotIn(("stop",), self.robot.calls)
        # The abort invalidates the authorization: re-entry needs a new test.
        self.robot.spikes.clear()
        self.robot.state["standing"] = True
        self.assertTrue(self.tools.enter_confined_space()["refused"])

    def test_authorization_revoked_by_later_abort(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.mission.permit = "ABORTED"
        result = self.tools.enter_confined_space()
        self.assertTrue(result["refused"])
        self.assertIn("not authorized", result["error"])

    def test_retest_after_abort_reauthorizes(self):
        # A mission that stays ABORTED after a new passing test must not
        # block the agent's fresh authorization.
        self.put("entry")
        self.mission.permit = "ABORTED"
        self.mission.atmospheric_test = lambda at_xy, dwell_s=4: {"passed": True}
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.assertTrue(self.tools.enter_confined_space()["ok"])

    def test_live_alarm_overrides_a_passing_mission_verdict(self):
        self.put("entry")

        def passes_but_gas_arrives(at_xy, dwell_s=4):
            self.robot.spikes.append(("co", 80.0, 7.0, 0.0, 0.5, 1.0))
            return {"passed": True}

        self.mission.atmospheric_test = passes_but_gas_arrives
        result = self.tools.run_atmospheric_test()
        self.assertFalse(result["passed"])
        self.assertFalse(result["entry_authorized"])

    def test_test_in_bad_air_fails_and_test_refused_inside(self):
        self.put("entry")
        self.robot.spikes.append(("co", 80.0, 7.0, 0.0, 0.5, 1.0))
        result = self.tools.run_atmospheric_test()
        self.assertFalse(result["passed"])
        self.assertFalse(result["entry_authorized"])
        self.robot.spikes.clear()
        self.put("waypoint_1")
        self.assertTrue(self.tools.run_atmospheric_test()["refused"])
        self.assertNotIn("atmospheric_test", [c[0] for c in self.mission.calls])

    def test_no_gas_telemetry_blocks_entry(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        real_gas = self.robot.gas
        self.robot.gas = lambda: None
        try:
            result = self.tools.enter_confined_space()
        finally:
            self.robot.gas = real_gas
        self.assertTrue(result["refused"])
        self.assertIn("telemetry", result["error"])


class TestWithRealMissionApi(ToolTestCase):
    """Flows against the extra API the real agent/mission.py exposes."""
    mission_cls = FakeFullMission

    def enter(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        result = self.tools.enter_confined_space()
        self.assertTrue(result["ok"], result)
        return result

    def test_test_is_taken_at_the_door_mouth(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.assertIn(("atmospheric_test", (6.6, 0.0), 4.0), self.mission.calls)
        self.assertEqual(self.mission.permit, "ENTRY_AUTHORIZED")

    def test_enter_uses_mission_enter_and_samples_each_waypoint(self):
        result = self.enter()
        self.assertIn(("enter",), self.mission.calls)
        self.assertEqual([r["place"] for r in result["readings"]],
                         ["waypoint_%d" % i for i in range(1, 6)])
        self.assertEqual(result["visited"][0], "door")
        self.assertEqual(self.mission.permit, "ENTRY_IN_PROGRESS")

    def test_exit_uses_exit_space_and_closes_the_permit(self):
        self.enter()
        self.mission.calls.clear()
        result = self.tools.walk_to_named(place="entry")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.mission.calls, [("exit_space",)])  # no ad-hoc walk_to legs
        self.assertEqual(result["permit"], "COMPLETED")
        self.robot.state["standing"] = True
        self.assertTrue(self.tools.enter_confined_space()["refused"])  # permit closed

    def test_partial_outward_move_inside_is_refused(self):
        self.enter()
        result = self.tools.walk_to_named(place="waypoint_2")
        self.assertTrue(result["refused"])
        self.assertIn("exits completely", result["error"])

    def test_new_permit_is_issued_before_a_retest(self):
        self.enter()
        self.tools.walk_to_named(place="entry")  # exit: permit COMPLETED, robot sat
        self.robot.state["standing"] = True
        result = self.tools.run_atmospheric_test()
        self.assertTrue(result["passed"], result)
        self.assertIn(("new_permit",), self.mission.calls)
        self.assertEqual(self.mission.permit, "ENTRY_AUTHORIZED")

    def test_mission_gate_still_holds_if_the_agent_gate_is_bypassed(self):
        self.put("entry")
        self.tools._auth = {"permit_at_auth": "NOT_STARTED", "t": self.clock.now()}  # agent-side bug
        result = self.tools.enter_confined_space()
        self.assertFalse(result["ok"])
        self.assertEqual(result["permit"], "NOT_STARTED")
        self.assertEqual(self.robot.pose()[:2], (7.0, 0.0))  # never moved in

    def test_full_entry_renews_a_closed_permit(self):
        self.put("spawn")
        self.mission.permit = "ABORTED"
        self.assertTrue(self.tools.full_confined_space_entry()["ok"])
        # The operator's entry confirmation is audited (log_event) before the run.
        self.assertEqual([c[0] for c in self.mission.calls if c[0] != "log_event"][:2],
                         ["new_permit", "run"])


class TestMissionTools(ToolTestCase):
    def test_full_entry_calls_mission_run(self):
        self.put("spawn")
        result = self.tools.full_confined_space_entry()
        self.assertTrue(result["ok"])
        self.assertIn(("run", True), self.mission.calls)
        self.assertEqual(result["summary"]["outcome"], "COMPLETED")
        self.assertEqual(result["permit"], "COMPLETED")

    def test_full_entry_refused_when_inside(self):
        self.put("waypoint_2")
        self.assertTrue(self.tools.full_confined_space_entry()["refused"])
        self.assertNotIn(("run", True), self.mission.calls)

    def test_retreat_now_calls_mission_retreat(self):
        self.put("waypoint_2")
        result = self.tools.retreat_now()
        self.assertTrue(result["ok"])
        self.assertIn(("retreat",), self.mission.calls)
        self.assertEqual(result["permit"], "ABORTED")

    def test_retreat_works_even_while_a_move_holds_the_lock(self):
        self.put("waypoint_2")
        self.tools._motion_lock.acquire()
        try:
            self.assertTrue(self.tools.retreat_now()["ok"])
        finally:
            self.tools._motion_lock.release()


class TestEmergencyStop(ToolTestCase):
    def test_refuses_without_confirm(self):
        self.put("waypoint_2")
        for kwargs in ({}, {"confirm": False}, {"confirm": "yes"}):
            result = self.tools.emergency_stop(**kwargs)
            self.assertFalse(result["ok"], kwargs)
            self.assertTrue(result["refused"], kwargs)
        self.assertEqual(self.robot.called("estop_hard"), [])

    def test_estops_with_confirm(self):
        # Inside the permit space (T8): stop now, ask the operator, then cut power.
        self.put("waypoint_2")
        result = self.tools.emergency_stop(confirm=True)
        self.assertTrue(result["ok"])
        names = self.robot.names()
        self.assertEqual(names[:2], ["send_velocity", "stop"])
        self.assertLess(names.index("stop"), names.index("estop_hard"))
        self.assertEqual(self.answers, ["hard_estop_inside"])
        self.assertTrue(self.robot.state["estopped"])
        # Outside it acts at once, without asking.
        self.put("spawn")
        self.robot.state["estopped"] = False
        self.assertTrue(self.tools.emergency_stop(confirm=True)["ok"])
        self.assertEqual(self.robot.names()[:2], ["estop_hard", "send_velocity"])
        self.assertEqual(self.answers, ["hard_estop_inside"])

    def test_works_while_a_move_holds_the_lock(self):
        self.tools._motion_lock.acquire()
        try:
            self.assertTrue(self.tools.emergency_stop(confirm=True)["ok"])
        finally:
            self.tools._motion_lock.release()


class TestGasInjection(ToolTestCase):
    def test_refuses_without_presenter_env(self):
        for value in (None, "0", "true", "yes"):
            self.env.clear()
            if value is not None:
                self.env[PRESENTER_ENV] = value
            result = self.tools.inject_gas_demo_spike()
            self.assertTrue(result["refused"], value)
            self.assertIn(PRESENTER_ENV, result["error"])
        self.assertEqual(self.robot.called("inject_gas"), [])

    def test_injects_at_robot_pose_by_default(self):
        self.env[PRESENTER_ENV] = "1"
        self.put("waypoint_3")
        result = self.tools.inject_gas_demo_spike(gas="H2S", ppm=40.0)
        self.assertTrue(result["ok"])
        self.assertEqual(self.robot.called("inject_gas"),
                         [("inject_gas", "h2s", 40.0, 0.0, 0.0, 0.5, 1.5)])
        self.assertTrue(self.tools.get_gas_reading()["alarm"])

    def test_reports_link_down(self):
        self.env[PRESENTER_ENV] = "1"
        self.robot.inject_gas = lambda *a: False  # RobotClient: not connected
        self.assertFalse(self.tools.inject_gas_demo_spike()["ok"])

    def test_validates_arguments(self):
        self.env[PRESENTER_ENV] = "1"
        self.assertTrue(self.tools.inject_gas_demo_spike(gas="methane")["refused"])
        self.assertTrue(self.tools.inject_gas_demo_spike(gas="h2s", ppm=-5.0)["refused"])
        self.assertTrue(self.tools.inject_gas_demo_spike(gas="h2s", radius=50.0)["refused"])
        self.assertTrue(self.tools.inject_gas_demo_spike(gas="o2", ppm=-3.0)["ok"])


class TestFailureSafety(ToolTestCase):
    def test_exception_in_tool_sends_zero_twist_and_stop(self):
        self.put("spawn")
        self.mission.raise_on_walk = RuntimeError("odometry lost")
        result = self.tools.walk_to_named(place="entry")
        self.assertFalse(result["ok"])
        self.assertTrue(result["robot_stopped"])
        self.assertIn("odometry lost", result["error"])
        self.assertIn(("send_velocity", 0.0, 0.0, 0.0), self.robot.calls)
        self.assertIn(("stop",), self.robot.calls)
        kinds = [e["kind"] for e in self.robot.events]
        self.assertIn("halt", kinds)
        self.assertEqual(kinds[-1], "tool_result")
        self.assertFalse(self.tools._motion_lock.locked())  # lock released after the error

    def test_failed_walk_halts_robot(self):
        self.put("spawn")
        self.mission.walk_ok = False
        result = self.tools.walk_to_named(place="entry")
        self.assertFalse(result["ok"])
        self.assertIn(("stop",), self.robot.calls)

    def test_ask_turns_agent_errors_into_a_halt(self):
        class BrokenAgent:
            def __call__(self, _text, limits=None):
                raise ConnectionError("bedrock unreachable")

            def cancel(self):
                pass

        self.put("entry")
        with _quiet_stdout():
            ok = spot_agent.ask(BrokenAgent(), self.tools, "status?", turn_timeout=5.0)
        self.assertFalse(ok)
        self.assertIn(("stop",), self.robot.calls)
        self.assertIn("error", [e["kind"] for e in self.robot.events])

    def test_ask_times_out_cancels_and_halts(self):
        class SlowAgent:
            def __init__(self):
                self.cancelled = threading.Event()

            def __call__(self, _text, limits=None):
                self.cancelled.wait(5.0)

            def cancel(self):
                self.cancelled.set()

        agent = SlowAgent()
        started = time.monotonic()
        with _quiet_stdout():
            ok = spot_agent.ask(agent, self.tools, "walk", turn_timeout=0.3)
        self.assertFalse(ok)
        self.assertTrue(agent.cancelled.is_set())
        self.assertLess(time.monotonic() - started, 3.0)
        self.assertIn(("stop",), self.robot.calls)


class _quiet_stdout:
    def __enter__(self):
        self._saved, sys.stdout = sys.stdout, io.StringIO()

    def __exit__(self, *_exc):
        sys.stdout = self._saved


class TestTranscriptCallback(unittest.TestCase):
    def test_streams_text_and_publishes_assistant_messages_only(self):
        said, out = [], io.StringIO()
        cb = TranscriptCallback(said.append, out=out)
        cb(data="Checking ")
        cb(data="the gas.")
        cb(event={"contentBlockStart": {"start": {"toolUse": {"name": "get_gas_reading"}}}})
        cb(message={"role": "assistant", "content": [{"text": "Checking the gas."},
                                                     {"toolUse": {"name": "get_gas_reading"}}]})
        cb(message={"role": "user", "content": [{"toolResult": {"content": [{"text": "{}"}]}}]})
        cb(message={"role": "assistant", "content": [{"text": "  "}]})
        cb(result=object())
        self.assertEqual(said, ["Checking the gas."])
        self.assertIn("Checking the gas.", out.getvalue())
        self.assertIn("[tool] get_gas_reading", out.getvalue())


class TestSpokenLanguage(unittest.TestCase):
    """Dry-run critic V4 + P2: regulatory wording and no tool names aloud."""

    # Verbatim model replies from docs/evidence/transcript-inc2-incident.txt
    # and the ok1 HUD transcript (06-hud-purge-incomplete.png).
    INC2 = ("It is safe. All four gases passed: oxygen 20.8 percent, LEL 0.7 percent, hydrogen "
            "sulfide 3.6 ppm, and carbon monoxide 0.8 ppm, all well within limits, and entry is "
            "now authorized on permit CSE-0427.")
    OK1 = ("I recommend waiting briefly with ventilation_status before the next re-test.")

    def test_system_prompt_forbids_absolute_safety_claims(self):
        import re
        prompt = build_system_prompt(DEFAULT_SITE)
        flat = " ".join(prompt.split())
        self.assertRegex(flat, r"[Nn]ever say or imply that the tank, the atmosphere, the entry "
                               r"or the robot is safe")
        self.assertIn('never answer "is it safe" with a yes', flat)
        self.assertIn('"the readings are within permissible entry limits"', flat)
        self.assertIn("all clear", flat)
        # The prompt itself never models an absolute claim for the model to copy.
        for bad in ("It is safe", "it's safe", "safe to enter now", "The tank is safe"):
            self.assertNotIn(bad, prompt)
        # Every remaining "safe" in the prompt sits in a prohibition, the
        # certification disclaimer, or a comparison ("safer").
        for match in re.finditer(r"[^.]*\bsafe\b[^.]*\.", flat):
            sentence = match.group(0)
            self.assertTrue(re.search(r"\b(never|not|Regulatory)\b", sentence, re.I), sentence)
        self.assertIn("Never say tool names or parameter names aloud", flat)

    def test_it_is_safe_is_rewritten_to_regulatory_language(self):
        import re
        spoken, flags = spot_agent.sanitize_speech(self.INC2)
        self.assertTrue(spoken.startswith("The readings are within permissible entry limits."),
                        spoken)
        self.assertIsNone(re.search(r"\bsafe\b", spoken, re.I), spoken)
        self.assertIn("hydrogen sulfide 3.6 ppm", spoken)          # the facts are untouched
        self.assertEqual([f["rule"] for f in flags], ["absolute_safety_claim"])
        for claim, expect in (
                ("The tank is now safe to enter.", "The readings are within permissible entry"),
                ("Everything's safe.", "The readings are within permissible entry limits."),
                ("The space is completely safe for workers.", "The readings are within"),
                ("Readings say it is safe to enter.", "Readings say the readings are within"),
                ("All clear.", "Within permissible entry limits.")):
            out, got = spot_agent.sanitize_speech(claim)
            self.assertTrue(out.startswith(expect), (claim, out))
            self.assertIsNone(re.search(r"\bsafe\b|all clear", out, re.I), out)
            self.assertTrue(got)

    def test_benign_words_pass_and_bare_safe_is_flagged_not_changed(self):
        for text in ("The SafetyGuard stopped the robot safely.",
                     "For a gas alarm, retreating is usually safer than an e-stop."):
            self.assertEqual(spot_agent.sanitize_speech(text), (text, []))
        disclaimer = "Spot is not intrinsically safe certified."
        out, flags = spot_agent.sanitize_speech(disclaimer)
        self.assertEqual(out, disclaimer)
        self.assertEqual([f["rule"] for f in flags], ["safe_word"])

    def test_tool_names_become_plain_words(self):
        spoken, flags = spot_agent.sanitize_speech(self.OK1)
        self.assertEqual(spoken, "I recommend waiting briefly with the ventilation status check "
                                 "before the next re-test.")
        self.assertEqual(flags[0]["rule"], "tool_name")
        for name in TOOL_NAMES:
            out, _ = spot_agent.sanitize_speech("Next I will use %s()." % name)
            self.assertNotIn(name, out)
            self.assertNotIn("_", out)
        self.assertEqual(set(spot_agent.SPOKEN_TOOL_LABELS), set(TOOL_NAMES))

    def test_negated_hedge_is_left_as_said_not_inverted(self):
        # Review of V4: "I can't say it is safe" is the regulatory answer the
        # prompt asks for; rewriting its "it is safe" produced "I can't say
        # the readings are within permissible entry limits" right before
        # "... but the readings are within permissible entry limits".
        for text in ("I can't say it is safe, but the readings are within permissible entry "
                     "limits.", "I cannot say that the tank is safe.", "No one can say it is safe.",
                     "I would not say it\u2019s safe to enter yet."):
            out, flags = spot_agent.sanitize_speech(text)
            self.assertEqual(out, text)
            self.assertIn("negated_safety_claim", [f["rule"] for f in flags])
        out, _ = spot_agent.sanitize_speech("There is no alarm, so it is safe.")
        self.assertEqual(out, "There is no alarm, so the readings are within permissible entry "
                              "limits.")

    def test_safety_claims_about_any_subject_are_rewritten(self):
        import re
        for claim, expect in (("Entry is now safe.", "Entry is now within permissible entry limits."),
                              ("Conditions are safe, so I will proceed.",
                               "Conditions are within permissible entry limits, so I will proceed."),
                              ("The air is safe to breathe.",
                               "The readings are within permissible entry limits."),
                              ("Readings are safe.", "Readings are within permissible entry limits.")):
            out, flags = spot_agent.sanitize_speech(claim)
            self.assertEqual(out, expect)
            self.assertIsNone(re.search(r"\bsafe\b", out, re.I), out)
            self.assertIn("absolute_safety_claim", [f["rule"] for f in flags])
        # The robot's own state is not an atmosphere claim: flagged, not garbled.
        out, flags = spot_agent.sanitize_speech("Spot is safe and standing at the entry point.")
        self.assertEqual(out, "Spot is safe and standing at the entry point.")
        self.assertIn("robot_safe_claim", [f["rule"] for f in flags])
        for text in ("This is a safe place to stand.", "It is safe to say the test passed."):
            self.assertEqual(spot_agent.sanitize_speech(text)[0], text)

    def test_tool_call_arguments_are_not_spoken(self):
        out, _ = spot_agent.sanitize_speech("I ran walk_to_named(place=entry), then "
                                            "ventilation_status(wait_s=30).")
        self.assertEqual(out, "I ran a walk to a named place, then the ventilation status check.")

    def test_purge_hold_text_quotes_the_missions_rounded_percent(self):
        # fraction 0.0825 is the 4-digit copy of the gate's 0.08251 (8.3 %);
        # re-rounding it gives 8.2 while the mission and the HUD show 8.3.
        hold = {"fraction_remaining": 0.0825, "percent_remaining": 8.3, "purge_target": 0.08,
                "eta_s": 1.2}
        text = spot_agent._purge_hold_text(hold)
        self.assertIn("8.3 percent of the unventilated contamination remains", text)
        self.assertNotIn("8.2", text)
        hold.pop("percent_remaining")                     # an older mission: the fraction
        self.assertIn("8.2 percent", spot_agent._purge_hold_text(hold))

    def test_say_publishes_the_filtered_text_and_keeps_the_original_for_audit(self):
        robot = FakeRobot()
        tools = SpotTools(robot, FakeMission(robot), DEFAULT_SITE, env={})
        with self.assertLogs("spot_agent", level="WARNING"):
            published = tools.say(self.INC2)
        say = [e for e in robot.events if e["kind"] == "say"][-1]
        self.assertEqual(say["message"], published)
        self.assertNotIn("It is safe", say["message"])
        audit = say["data"]["speech_filter"]
        self.assertTrue(audit["rewritten"])
        self.assertEqual(audit["original"], self.INC2)
        self.assertEqual(audit["rules"], ["absolute_safety_claim"])
        tools.say("Oxygen 20.9 percent.")
        clean = [e for e in robot.events if e["kind"] == "say"][-1]
        self.assertEqual((clean["message"], clean["data"]), ("Oxygen 20.9 percent.", None))

    def test_console_transcript_shows_the_filtered_text(self):
        said, out = [], io.StringIO()
        cb = TranscriptCallback(said.append, out=out)
        for delta in ("It is sa", "fe. Oxygen 20.8 percent. I'll use ventila", "tion_status."):
            cb(data=delta)
        cb(message={"role": "assistant", "content": [{"text": "It is safe. Oxygen 20.8 percent. "
                                                               "I'll use ventilation_status."}]})
        text = out.getvalue()
        self.assertNotIn("It is safe", text)
        self.assertNotIn("ventilation_status", text)
        self.assertIn("The readings are within permissible entry limits. Oxygen 20.8 percent.",
                      text)
        self.assertEqual(len(said), 1)   # SpotTools.say filters (and audits) what is published


    def test_a_cut_off_reply_is_printed_not_glued_to_the_next_one(self):
        # Buffering per text block (for the filter) must not hold a reply
        # that never completes (Bedrock error, timeout, Ctrl-C).
        import types
        said, out = [], io.StringIO()
        cb = TranscriptCallback(said.append, out=out)
        cb(data="Oxygen 20.8 percent, and the H2S")
        cb(force_stop=True, force_stop_reason="ThrottlingException")
        self.assertEqual(out.getvalue(), "agent> Oxygen 20.8 percent, and the H2S\n")
        cb(data="Stale")
        spot_agent._flush_transcript(types.SimpleNamespace(callback_handler=cb))
        cb(init_event_loop=True)
        cb(data="Next reply.")
        cb(message={"role": "assistant", "content": [{"text": "Next reply."}]})
        self.assertIn("agent> Stale\nagent> Next reply.\n", out.getvalue())
        self.assertEqual(said, ["Next reply."])
        spot_agent._flush_transcript(object())   # an agent without the handler: no-op


class TestReplConsole(unittest.TestCase):
    """P2: the guard's 5 s INFO lines overwrote the operator> prompt."""

    def record(self, source, kind, level=logging.INFO, name="mission"):
        return logging.LogRecord(name, level, __file__, 1, "[%s] %s: %s",
                                 (source, kind, "O2 20.9 %"), None)

    def test_guard_monitor_lines_are_dropped_everything_else_prints(self):
        f = spot_agent.QuietGuardConsole()
        self.assertFalse(f.filter(self.record("guard", "monitor")))
        for record in (self.record("guard", "alarm"), self.record("guard", "retreat_start"),
                       self.record("mission", "atmos_test"), self.record("agent", "ventilation"),
                       self.record("guard", "monitor", level=logging.WARNING),
                       self.record("guard", "monitor", name="spot_agent")):
            self.assertTrue(f.filter(record), record.getMessage())

    def test_installed_on_the_console_handler_and_idempotent(self):
        stream = io.StringIO()
        logger = logging.getLogger("test_repl_console_root")
        logger.propagate = False
        handler = logging.StreamHandler(stream)
        logger.addHandler(handler)
        try:
            self.assertEqual(spot_agent.quiet_guard_console(logger), 1)
            self.assertEqual(spot_agent.quiet_guard_console(logger), 1)
            self.assertEqual(len(handler.filters), 1)
            mission_log = logging.getLogger("test_repl_console_root.child")
            mission_log.setLevel(logging.INFO)
            handler.handle(self.record("guard", "monitor"))
            handler.handle(self.record("guard", "alarm"))
            self.assertNotIn("monitor", stream.getvalue())
            self.assertIn("[guard] alarm", stream.getvalue())
        finally:
            logger.removeHandler(handler)


class TestToolMetadata(unittest.TestCase):
    def test_every_tool_has_a_docstring_covering_its_parameters(self):
        import inspect
        tools = SpotTools(FakeRobot(), FakeMission(FakeRobot()), DEFAULT_SITE, env={})
        for fn in tools.tool_functions():
            doc = inspect.getdoc(fn) or ""
            self.assertTrue(doc.strip(), fn.__name__)
            params = list(inspect.signature(fn).parameters)
            if params:
                self.assertIn("Args:", doc, fn.__name__)
            for param in params:
                self.assertIn("%s:" % param, doc, (fn.__name__, param))

    def test_strands_builds_tool_specs(self):
        try:
            from strands import tool
        except ImportError:
            self.skipTest("strands-agents not installed")
        tools = SpotTools(FakeRobot(), FakeMission(FakeRobot()), DEFAULT_SITE, env={})
        specs = {t.tool_name: t.tool_spec for t in (tool(fn) for fn in tools.tool_functions())}
        self.assertEqual(list(specs), list(TOOL_NAMES))
        walk = specs["walk_to"]["inputSchema"]["json"]
        self.assertEqual(sorted(walk["required"]), ["x", "y"])
        self.assertEqual(walk["properties"]["x"]["type"], "number")
        estop = specs["emergency_stop"]["inputSchema"]["json"]
        self.assertFalse(estop["properties"]["confirm"]["default"])
        self.assertNotIn("required", estop)
        for name, spec in specs.items():
            self.assertTrue(spec["description"].strip(), name)
            self.assertNotIn("self", spec["inputSchema"]["json"].get("properties", {}), name)

    def test_strands_tool_call_path_coerces_and_publishes(self):
        try:
            from strands import tool
        except ImportError:
            self.skipTest("strands-agents not installed")
        robot = FakeRobot()
        tools = SpotTools(robot, FakeMission(robot), DEFAULT_SITE, env={})
        wrapped = tool(tools.emergency_stop)
        self.assertTrue(wrapped(confirm=False)["refused"])  # direct call still works
        self.assertEqual(robot.called("estop_hard"), [])


class TestReviewRegressions(ToolTestCase):
    """Bugs found in the adversarial review, one test each (contract-only mission)."""

    def test_guard_trip_revokes_authorization_even_if_the_permit_state_is_unchanged(self):
        # Contract-only mission already ABORTED: a guard trip leaves it ABORTED,
        # so "permit changed since the test" alone cannot catch the abort.
        self.put("entry")
        self.mission.permit = "ABORTED"
        self.mission.atmospheric_test = lambda at_xy, dwell_s=4: {"passed": True}
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        wp3 = self.xy("waypoint_3")
        self.robot.spikes.append(("h2s", 40.0, wp3[0], wp3[1], 0.5, 0.5))
        self.assertFalse(self.tools.enter_confined_space()["ok"])
        self.robot.spikes.clear()
        self.robot.state["standing"] = True
        result = self.tools.enter_confined_space()
        self.assertTrue(result.get("refused"), result)

    def test_stale_test_must_be_repeated_before_entry(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.clock.t += spot_agent.AUTH_MAX_AGE_S + 1.0
        result = self.tools.enter_confined_space()
        self.assertTrue(result.get("refused"), result)
        self.assertEqual(self.walks(), [])
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.assertTrue(self.tools.enter_confined_space()["ok"])

    def test_sit_down_refused_while_a_retreat_runs(self):
        self.put("waypoint_2")
        self.robot.motion_owner = "retreat"
        result = self.tools.sit_down()
        self.assertTrue(result.get("refused"), result)
        self.assertEqual(self.robot.calls, [])  # not even a stop: it would stall the retreat

    def test_sit_down_inside_stops_but_stays_standing(self):
        self.put("waypoint_2")
        result = self.tools.sit_down()
        self.assertTrue(result.get("refused"), result)
        self.assertIn(("stop",), self.robot.calls)
        self.assertEqual(self.robot.called("sit"), [])
        self.assertTrue(self.robot.state["standing"])

    def test_stand_up_refused_while_a_retreat_runs(self):
        self.put("waypoint_2")
        self.robot.motion_owner = "retreat"
        self.assertTrue(self.tools.stand_up().get("refused"))
        self.assertEqual(self.robot.called("stand"), [])

    def test_full_entry_refused_when_the_leg_to_entry_crosses_the_tank(self):
        self.put((-10.0, 0.0))
        result = self.tools.full_confined_space_entry()
        self.assertTrue(result.get("refused"), result)
        self.assertNotIn(("run", True), self.mission.calls)

    def test_retreat_from_the_far_side_does_not_walk_through_the_tank(self):
        self.put((-10.0, 0.0))
        result = self.tools.retreat_now()
        self.assertTrue(result.get("refused"), result)
        self.assertNotIn(("retreat",), self.mission.calls)
        self.assertIn(("stop",), self.robot.calls)

    def test_retreat_refused_fast_when_estopped(self):
        self.put("waypoint_2")
        self.robot.state.update(estopped=True, powered=False, standing=False)
        result = self.tools.retreat_now()
        self.assertTrue(result.get("refused"), result)
        self.assertIn("e-stop", result["error"])
        self.assertNotIn(("retreat",), self.mission.calls)

    def test_retreat_stands_a_sitting_robot_first(self):
        self.put("waypoint_2", standing=False)
        self.assertTrue(self.tools.retreat_now()["ok"])
        self.assertIn(("stand",), self.robot.calls)
        self.assertIn(("retreat",), self.mission.calls)

    def test_alarm_detected_from_values_when_flags_are_missing(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.robot.gas = lambda: {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 15.0, "co_ppm": 0.0,
                                  "alarms": {}, "alarm": False}
        self.assertTrue(self.tools.get_gas_reading()["alarm"])
        self.assertEqual(self.tools.get_gas_reading()["alarming_gases"], ["h2s"])
        self.assertTrue(self.tools.enter_confined_space().get("refused"))
        self.assertEqual(self.walks(), [])

    def test_ask_timeout_waits_for_the_in_flight_tool_then_halts(self):
        # Strands' cancel() does not interrupt a running tool: a mission
        # walk_to keeps sending cmd_vel at 20 Hz, so halting while it still
        # runs is a no-op (and the next instruction would hit Strands'
        # ConcurrencyException).
        finished = {}

        class BusyToolAgent:
            def __call__(self, _text, limits=None):
                time.sleep(0.6)  # a tool that ignores cancel()
                finished["t"] = time.monotonic()

            def cancel(self):
                pass

        stops = []
        real_stop = self.robot.stop
        self.robot.stop = lambda: (stops.append(time.monotonic()), real_stop())[1]
        with _quiet_stdout():
            ok = spot_agent.ask(BusyToolAgent(), self.tools, "walk", turn_timeout=0.05)
        self.assertFalse(ok)
        self.assertIn("t", finished)  # ask() did not return while the tool still ran
        self.assertTrue(stops and stops[-1] >= finished["t"], (stops, finished))

    def test_agent_error_does_not_stall_a_running_retreat(self):
        class BrokenAgent:
            def __call__(self, _text, limits=None):
                raise ConnectionError("bedrock throttled")

            def cancel(self):
                pass

        self.put("waypoint_2")
        self.robot.motion_owner = "retreat"
        with _quiet_stdout():
            spot_agent.ask(BrokenAgent(), self.tools, "status?", turn_timeout=5.0)
        self.assertEqual(self.robot.called("stop"), [])


class TestReviewRegressionsRealMissionApi(ToolTestCase):
    """Bugs found in the adversarial review against the real mission's API."""
    mission_cls = FakeFullMission

    def test_agent_veto_holds_even_though_the_mission_authorized(self):
        # The mission's own test passes (and it moves to ENTRY_AUTHORIZED),
        # then gas arrives before the agent's live check: the agent vetoes.
        # Once the alarm clears the veto must still hold until a new test.
        self.put("entry")
        mission_test = self.mission.atmospheric_test

        def passes_then_gas_arrives(at_xy, dwell_s=4):
            result = mission_test(at_xy, dwell_s)
            self.robot.spikes.append(("co", 80.0, at_xy[0], at_xy[1], 0.5, 1.0))
            return result

        self.mission.atmospheric_test = passes_then_gas_arrives
        self.assertFalse(self.tools.run_atmospheric_test()["passed"])
        self.assertEqual(self.mission.permit, "ENTRY_AUTHORIZED")
        self.robot.spikes.clear()
        result = self.tools.enter_confined_space()
        self.assertTrue(result.get("refused"), result)
        self.assertNotIn(("enter",), self.mission.calls)

    def test_return_to_the_nearest_interior_place_is_allowed(self):
        self.put("entry")
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.assertTrue(self.tools.walk_to_named(place="waypoint_2")["ok"])
        # A leg toward waypoint_3 stopped 0.8 m short of it (timeout, say).
        wp2, wp3 = self.xy("waypoint_2"), self.xy("waypoint_3")
        k = 0.8 / math.hypot(wp3[0] - wp2[0], wp3[1] - wp2[1])
        self.put((wp3[0] + (wp2[0] - wp3[0]) * k, wp3[1] + (wp2[1] - wp3[1]) * k))
        result = self.tools.walk_to_named(place="waypoint_3")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.walks(), [wp3])

    def test_failed_exit_points_to_retreat_now(self):
        self.put("waypoint_2")
        self.mission.permit = "ABORTED"  # e.g. an incomplete retreat left it inside
        result = self.tools.walk_to_named(place="entry")
        self.assertFalse(result["ok"])
        self.assertIn("retreat_now", result["error"])

    def test_injection_is_disclosed_in_the_mission_audit_log(self):
        self.env[PRESENTER_ENV] = "1"
        self.put("waypoint_3")
        self.assertTrue(self.tools.inject_gas_demo_spike(gas="h2s", ppm=40.0)["ok"])
        self.assertIn(("log_event", "gas_injected", "agent"), self.mission.calls)


class TestStorylineTools(ToolTestCase):
    """The storyline tools against a fake with the real mission's extra API."""
    mission_cls = FakeStorylineMission

    def complete_permit(self):
        result = self.tools.prepare_permit(permit_id="CSE-0417",
                                           rescue_plan="Standby rescue team at the door",
                                           request_attestation=True)
        self.assertTrue(result["complete"], result)
        return result

    def test_no_approach_until_the_permit_is_complete(self):
        self.put("spawn")
        for call in (lambda: self.tools.walk_to_named(place="entry"),
                     lambda: self.tools.run_atmospheric_test(),
                     lambda: self.tools.full_confined_space_entry(),
                     lambda: self.tools.walk_to(x=6.5, y=3.0)):
            result = call()
            self.assertTrue(result.get("refused"), result)
            self.assertIn("permit", result["error"])
        self.assertEqual(self.walks(), [])
        partial = self.tools.prepare_permit(permit_id="CSE-0417", rescue_plan="Standby rescue team")
        self.assertTrue(partial["ok"])
        self.assertFalse(partial["complete"])
        self.assertIn("isolation", partial["message"])
        refused = self.tools.walk_to_named(place="entry")
        self.assertIn("lockout/tagout (LOTO)", refused["error"])
        self.assertTrue(self.tools.prepare_permit(request_attestation=True)["complete"])
        self.assertIn(("attestation", "y"), self.mission.calls)
        self.assertTrue(self.tools.walk_to_named(place="entry")["ok"])
        self.assertIn(("prepare_permit", "agent"), self.mission.calls)
        # Leaving the tank area never needs the permit.
        self.mission._blank_permit()
        self.assertTrue(self.tools.walk_to_named(place="spawn")["ok"])

    def test_prepare_permit_starts_a_new_permit_after_a_closed_one(self):
        self.put("entry")
        self.mission.permit = "ABORTED"
        result = self.tools.prepare_permit(rescue_plan="Standby team B")
        self.assertTrue(result["new_permit_started"])
        self.assertIn(("new_permit",), self.mission.calls)
        self.assertEqual(self.mission.fields["rescue_plan"], "Standby team B")

    def test_fail_ventilate_retest_pass_enter(self):
        self.put("entry")
        self.complete_permit()
        self.robot.spikes.append(("h2s", 40.0, 6.6, 0.0, 0.5, 0.5))  # unventilated air at the door
        first = self.tools.run_atmospheric_test()
        self.assertFalse(first["passed"])
        self.assertEqual(first["permit"], "PRE_ENTRY_TESTING")
        self.assertIn("start_ventilation", first["next_step"])
        self.assertTrue(self.tools.enter_confined_space().get("refused"))
        started = self.tools.start_ventilation(tau_s=10.0)
        self.assertTrue(started["ok"])
        self.assertIn(("start_ventilation", 10.0, "agent"), self.mission.calls)
        status = self.tools.ventilation_status(wait_s=30.0)
        self.assertTrue(status["purged"])
        self.assertIn(("await_ventilation", 30.0, "agent"), self.mission.calls)
        self.robot.spikes.clear()  # the fake's stand-in for the purged atmosphere
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        self.assertTrue(self.tools.enter_confined_space()["ok"])

    def test_ventilation_arguments_are_validated(self):
        for tau in (0.5, 500.0, float("nan")):
            self.assertTrue(self.tools.start_ventilation(tau_s=tau).get("refused"), tau)
        self.assertNotIn("start_ventilation", [c[0] for c in self.mission.calls])
        self.assertFalse(self.tools.ventilation_status()["purged"])  # report now, no wait
        self.assertNotIn("await_ventilation", [c[0] for c in self.mission.calls])

    def test_stand_up_uses_the_mission_and_reports_a_self_right(self):
        self.mission.self_rights = 1
        result = self.tools.stand_up()
        self.assertTrue(result["ok"])
        self.assertIn("self-righted", result["message"])
        self.assertIn(("stand_up",), self.mission.calls)

    def test_guard_trip_during_a_tool_puts_the_incident_in_its_result(self):
        self.put("entry")
        self.complete_permit()
        self.assertTrue(self.tools.run_atmospheric_test()["passed"])
        wp3 = self.xy("waypoint_3")
        self.robot.spikes.append(("h2s", 40.0, wp3[0], wp3[1], 0.5, 0.5))
        result = self.tools.enter_confined_space()
        self.assertFalse(result["ok"])
        self.assertTrue(result["guard_tripped"])
        self.assertTrue(result["incident"]["found"])
        self.assertEqual(result["incident"]["trigger"]["gases_in_alarm"], ["H2S"])
        self.assertIn("SafetyGuard tripped", result["incident_note"])
        done = [e for e in self.robot.events if e["kind"] == "tool_result"][-1]
        self.assertIn("incident", done["data"]["result"])
        self.assertIsNone(self.tools.pending_incident_prompt())  # already told

    def test_incident_between_tool_calls_rides_on_the_next_tool_result(self):
        self.put("entry")
        self.mission.trip()  # fired while the model was mid-turn, between two tools
        first = self.tools.get_gas_reading()
        self.assertTrue(first["incident"]["found"])
        self.assertNotIn("incident", self.tools.get_gas_reading())

    def test_incident_between_turns_leads_the_next_prompt(self):
        prompts = []

        class RecordingAgent:
            def __call__(self, text, limits=None):
                prompts.append(text)

            def cancel(self):
                pass

        self.put("entry")
        self.mission.trip(reading=37.5)  # guard fired while the model was idle
        with _quiet_stdout():
            self.assertTrue(spot_agent.ask(RecordingAgent(), self.tools, "Status?", 5.0))
            self.assertTrue(spot_agent.ask(RecordingAgent(), self.tools, "And now?", 5.0))
        self.assertTrue(prompts[0].startswith("[Automatic safety notice"))
        self.assertIn("37.5", prompts[0])
        self.assertTrue(prompts[0].endswith("Operator instruction: Status?"))
        self.assertEqual(prompts[1], "And now?")  # surfaced once only
        self.assertIn("incident_notice", [e["kind"] for e in self.robot.events])
        operator = [e["message"] for e in self.robot.events if e["kind"] == "operator"]
        self.assertEqual(operator, ["Status?", "And now?"])  # the HUD shows what was typed

    def test_explain_last_incident(self):
        result = self.tools.explain_last_incident()
        self.assertTrue(result["ok"])
        self.assertFalse(result["found"])
        self.mission.trip()
        result = self.tools.explain_last_incident()
        self.assertTrue(result["found"])
        self.assertEqual(result["retreat"]["status"], "complete")
        self.assertIsNone(self.tools.pending_incident_prompt())  # explaining counts as told

    def test_clear_injected_gas_is_presenter_gated(self):
        self.put("entry")
        self.robot.spikes.append(("h2s", 40.0, 7.0, 0.0, 0.5, 1.0))
        refused = self.tools.clear_injected_gas()
        self.assertTrue(refused["refused"])
        self.assertIn(PRESENTER_ENV, refused["error"])
        self.assertTrue(self.robot.spikes)
        self.env[PRESENTER_ENV] = "1"
        self.assertTrue(self.tools.clear_injected_gas()["ok"])
        self.assertEqual(self.robot.spikes, [])
        self.assertIn(("clear_injected_gas", "agent"), self.mission.calls)
        self.put("waypoint_2")
        self.assertTrue(self.tools.clear_injected_gas(reset_tank=True)["refused"])
        self.put("entry")
        self.assertTrue(self.tools.clear_injected_gas(reset_tank=True)["tank_reset"])
        self.assertIn(("reset_atmosphere", "agent"), self.mission.calls)

    def test_full_entry_can_run_the_ventilation_storyline(self):
        self.put("spawn")
        self.complete_permit()
        self.assertTrue(self.tools.full_confined_space_entry(ventilate_if_needed=True)["ok"])
        self.assertIn(("run_storyline", True, True), self.mission.calls)


class TestAgentStorylineWithRealMission(unittest.TestCase):
    """The agent tools driving the REAL mission + SafetyGuard on the
    kinematic robot of tests/test_mission.py (scaled clock): the full
    storyline, then a presenter spike injected mid-turn during the entry."""

    def setUp(self):
        from tests import test_mission as tm
        import mission as real_mission
        self.tm = tm
        self.clock = tm.ScaledClock(20.0)
        self.robot = tm.FakeRobot(self.clock, gas_fn3=tm.unventilated_tank, probe_enabled=True,
                                  standing=False)
        self.tmp = tempfile.TemporaryDirectory()
        self.mission = real_mission.ConfinedSpaceMission(
            self.robot, self.robot.site(), os.path.join(self.tmp.name, "mission.jsonl"),
            clock=self.clock, dwell_s=3.0, require_permit=True)
        self.env = {PRESENTER_ENV: "1"}
        self.tools = SpotTools(self.robot, self.mission, self.robot.site(), env=self.env,
                               sleep=self.clock.sleep, clock=self.clock.now)
        self.tools.pending.responder = lambda item: "y"   # the operator at the console

    def tearDown(self):
        if self.mission._retreating:
            self.mission.cancel("test teardown")
        self.mission.close()
        self.tmp.cleanup()

    def test_storyline_then_a_mid_turn_spike_is_explained_from_the_tool_result(self):
        tools, robot = self.tools, self.robot
        self.assertTrue(tools.stand_up()["ok"])
        self.assertTrue(tools.walk_to_named(place="entry").get("refused"))  # no permit yet
        self.assertTrue(tools.prepare_permit(
            permit_id="CSE-0417", rescue_plan="Standby rescue team at the door",
            request_attestation=True)["complete"])

        first = tools.run_atmospheric_test(dwell_s=3.0)
        self.assertFalse(first["passed"])
        self.assertEqual(first["permit"], "PRE_ENTRY_TESTING")
        readings = first["readings_in_osha_order"]
        self.assertEqual([r["gas"] for r in readings], ["O2", "LEL", "H2S", "CO"])
        self.assertEqual(readings[2]["worst_at_height_m"], 0.2)
        self.assertFalse(readings[2]["passed"])
        self.assertIn("probe", first["method"])
        self.assertTrue(tools.enter_confined_space().get("refused"))

        self.assertTrue(tools.start_ventilation(tau_s=3.0)["ok"])
        self.assertTrue(tools.ventilation_status(wait_s=60.0)["purged"])
        second = tools.run_atmospheric_test(dwell_s=3.0)
        self.assertTrue(second["passed"], second)

        # The presenter injects an H2S spike at Spot while enter_confined_space runs.
        stop = threading.Event()

        def presenter():
            while not stop.is_set():
                x, y, _ = robot.true_pose()
                if math.hypot(x, y) < 3.2:  # past waypoint 1, heading for waypoint 2
                    tools.inject_gas_demo_spike(gas="h2s", ppm=80.0, radius=1.0)
                    return
                self.clock.sleep(0.05)

        watcher = threading.Thread(target=presenter, daemon=True)
        watcher.start()
        result = tools.enter_confined_space()
        stop.set()
        watcher.join(timeout=5.0)
        self.assertFalse(result["ok"])
        incident = result["incident"]
        self.assertTrue(incident["found"])
        self.assertEqual(incident["trigger"]["gases_in_alarm"], ["H2S"])
        self.assertGreaterEqual(incident["trigger"]["reading"]["h2s_ppm"], 10.0)
        self.assertEqual(incident["retreat"]["status"], "complete")  # waited for the retreat
        self.assertEqual(incident["permit_after"]["state"], "ABORTED")
        self.assertEqual(incident["staged_injection"][0]["kind"], "gas_injected")
        self.assertIn("SafetyGuard tripped", result["incident_note"])
        x, y, _ = robot.true_pose()
        self.assertFalse(self.tm.in_tank(x, y))
        self.assertEqual(self.mission.permit, "ABORTED")
        self.assertIsNone(tools.pending_incident_prompt())
        self.assertNotIn("incident", tools.get_status())  # told once, not on every tool
        self.assertEqual(tools.explain_last_incident()["incident_id"], incident["incident_id"])
        done = [e for e in robot.events if e["kind"] == "tool_result"
                and e["data"]["tool"] == "enter_confined_space"][-1]
        self.assertIn("incident", done["data"]["result"])

        # Re-entry needs a new permit: the closed one is replaced by a blank one.
        self.assertTrue(tools.stand_up()["ok"])
        again = tools.run_atmospheric_test()
        self.assertTrue(again.get("refused"), again)
        self.assertIn("not complete", again["error"])
        robot.true_pose()
        self.assertGreater(robot.min_clearance, 0.05, robot.worst_pose)


class TestHelpersAndPrompt(unittest.TestCase):
    def test_permit_state_accepts_likely_snapshot_shapes(self):
        self.assertEqual(_permit_state({"permit_state": "aborted"}), "ABORTED")
        self.assertEqual(_permit_state({"permit": {"state": "COMPLETED"}}), "COMPLETED")
        self.assertEqual(_permit_state({"state": "ENTRY_AUTHORIZED"}), "ENTRY_AUTHORIZED")
        self.assertIsNone(_permit_state({"pose": [1, 2]}))
        self.assertIsNone(_permit_state(None))

    def test_test_passed_is_explicit_only(self):
        self.assertTrue(_test_passed({"passed": True}))
        self.assertFalse(_test_passed({"passed": False, "ok": True}))
        self.assertTrue(_test_passed({"result": "PASS"}))
        self.assertFalse(_test_passed({"o2_pct": 20.9}))
        self.assertFalse(_test_passed(None))

    def test_system_prompt_carries_rules_and_site(self):
        prompt = build_system_prompt(DEFAULT_SITE)
        for needle in ("1910.146", "oxygen first", "SafetyGuard", "ATEX", "confirm",
                       "presenter", "waypoint_5 (-4.8, 0.0)", "entry (7.0, 0.0)"):
            self.assertIn(needle, prompt)

    def test_cli_defaults(self):
        args = spot_agent.build_arg_parser().parse_args([])
        self.assertEqual(args.model, "us.anthropic.claude-sonnet-5")
        self.assertEqual(args.region, "us-west-2")
        self.assertEqual(args.namespace, "/demo_spot")
        self.assertFalse(args.dry_run)
        self.assertIsNone(args.once)



class TestAdversarialReviewAgent(unittest.TestCase):
    """Agent-level regressions from the mission_agent adversarial review,
    against the REAL mission + SafetyGuard on the kinematic test robot."""

    def setUp(self):
        from tests import test_mission as tm
        import mission as real_mission
        self.tm, self.real_mission = tm, real_mission
        self.clock = tm.ScaledClock(20.0)
        self.tmp = tempfile.TemporaryDirectory()
        self.missions = []

    def tearDown(self):
        for m in self.missions:
            if m._retreating:
                m.cancel("test teardown")
            m.close()
        self.tmp.cleanup()

    def make(self, robot):
        m = self.real_mission.ConfinedSpaceMission(
            robot, robot.site(), os.path.join(self.tmp.name, "mission.jsonl"),
            clock=self.clock, dwell_s=3.0, require_permit=True)
        self.missions.append(m)
        tools = SpotTools(robot, m, robot.site(), env={}, sleep=self.clock.sleep,
                          clock=self.clock.now)
        tools.pending.responder = lambda item: "y"   # the operator at the console
        return m, tools

    def test_guard_watches_from_agent_startup_without_any_tool_call(self):
        # Agent restarted while Spot stood inside the tank: no tool has run
        # yet, and the gas goes bad. The guard must still pull Spot out.
        state = {"h2s": 0.0}
        tm = self.tm
        robot = tm.FakeRobot(self.clock, start=(2.5, 1.6, 0.0), gas_fn=lambda x, y: dict(
            tm.BASELINE, h2s_ppm=state["h2s"] if tm.in_tank(x, y) else 0.0))
        mission, _tools = self.make(robot)
        state["h2s"] = 40.0
        deadline = self.clock.now() + 120.0
        while (not robot.of_kind("retreat_complete") and not robot.of_kind("retreat_incomplete")
               and self.clock.now() < deadline):
            self.clock.sleep(0.25)
        self.assertEqual(len(robot.of_kind("alarm")), 1)
        self.assertEqual(len(robot.of_kind("retreat_complete")), 1)
        x, y, _ = robot.true_pose()
        self.assertFalse(tm.in_tank(x, y))
        self.assertGreater(robot.min_clearance, 0.05, robot.worst_pose)

    def test_probe_failure_at_the_door_never_authorizes_entry(self):
        # The body sensor at the door mouth reads the clean air outside an
        # unventilated tank; without the probe the test must not pass.
        tm = self.tm
        robot = tm.FakeRobot(self.clock, gas_fn3=tm.unventilated_tank, probe_enabled=False)
        _mission, tools = self.make(robot)
        self.assertTrue(tools.prepare_permit(
            permit_id="CSE-0417", rescue_plan="Standby rescue team at the door",
            request_attestation=True)["complete"])
        result = tools.run_atmospheric_test(dwell_s=3.0)
        self.assertFalse(result["passed"], result)
        self.assertFalse(result["entry_authorized"])
        self.assertEqual(result["permit"], "PRE_ENTRY_TESTING")
        self.assertIn("probe", result["next_step"])
        self.assertNotIn("start_ventilation", result["next_step"])
        self.assertTrue(tools.enter_confined_space().get("refused"))
        self.assertTrue(tools.walk_to_named(place="door").get("refused"))
        self.assertFalse(any(tm.in_tank(x, y) for _, x, y, _ in robot.trajectory))


class TestProbeAndPurgeTools(ToolTestCase):
    """Critic round (P1-3 / P1-4 wording): the six-point simulated probe and
    the purge gate, as the model sees them."""
    mission_cls = FakeStorylineMission

    PURGE = ("Purge incomplete — wait: the gases read within limits at the door, but an earlier "
             "test on this permit failed and 17.6 % of the unventilated contamination remains "
             "(entry needs <= 8 %); keep ventilating about 18 s more, then re-test")

    def test_purge_incomplete_says_wait_not_ventilate_and_does_not_authorize(self):
        self.put("entry")
        self.tools.prepare_permit(permit_id="CSE-0417", rescue_plan="Standby rescue team at the door",
                                  request_attestation=True)
        results = [{"gas": g, "value": v, "unit": u, "limit": "x", "passed": True,
                    "worst_z_m": 0.2, "worst_inside_m": 1.0, "by_height": {"0.20": v}}
                   for g, v, u in (("o2", 20.8, "%"), ("lel", 0.9, "% LEL"),
                                   ("h2s", 8.4, "ppm"), ("co", 0.0, "ppm"))]
        forced = {"passed": False, "purge_incomplete": True, "sampled_inside": True,
                  "method": "probe", "results": results, "failures": [self.PURGE],
                  "purge": {"message": self.PURGE, "wait_s": 18.0}}

        def purge_incomplete_test(at_xy, dwell_s=4):   # what agent/mission.py returns
            self.mission.calls.append(("atmospheric_test", tuple(at_xy), dwell_s))
            self.mission.permit = "PRE_ENTRY_TESTING"
            return copy.deepcopy(forced)

        self.mission.atmospheric_test = purge_incomplete_test
        result = self.tools.run_atmospheric_test()
        self.assertFalse(result["passed"])
        self.assertFalse(result["entry_authorized"])
        self.assertTrue(result["purge_incomplete"])
        self.assertEqual(result["wait_s"], 18.0)
        self.assertIn("Purge incomplete, wait", result["next_step"])
        self.assertIn("ventilation_status", result["next_step"])
        self.assertIn("18 s more", result["next_step"])
        self.assertNotIn("start_ventilation", result["next_step"])
        self.assertIn("six points", result["method"])
        self.assertIn("simulated; arm-mounted on the real robot", result["method"])
        self.assertEqual(result["readings_in_osha_order"][2]["worst_inside_opening_m"], 1.0)
        self.assertTrue(self.tools.enter_confined_space().get("refused"))

    def test_purge_hold_result_is_exactly_the_missions_event_data(self):
        # Dry-run critic V3: the HUD said 7 % while the agent said 8.5 %. The
        # tool result now carries the mission's purge_hold dict unchanged,
        # and next_step spells out the same numbers, rounded like the HUD.
        self.put("entry")
        self.tools.prepare_permit(permit_id="CSE-0428", rescue_plan="on-site standby team",
                                  request_attestation=True)
        hold = {"fraction_remaining": 0.0847, "purge_target": 0.08, "eta_s": 1.7,
                "percent_remaining": 8.5, "fraction_source": "sim", "ventilation_on": True,
                "target_reachable": True, "prior_failure": True, "simulated": True}
        results = [{"gas": g, "value": v, "unit": u, "limit": "x", "passed": True}
                   for g, v, u in (("o2", 20.7, "%"), ("lel", 0.9, "% LEL"),
                                   ("h2s", 4.6, "ppm"), ("co", 0.7, "ppm"))]
        forced = {"passed": False, "purge_incomplete": True, "sampled_inside": True,
                  "method": "probe", "results": results, "failures": ["purge"],
                  "purge": {"message": "Purge incomplete — wait", "wait_s": 1.7},
                  "purge_hold": hold}

        def held_test(at_xy, dwell_s=4):
            self.mission.permit = "PRE_ENTRY_TESTING"
            return copy.deepcopy(forced)

        self.mission.atmospheric_test = held_test
        result = self.tools.run_atmospheric_test()
        self.assertEqual(result["purge_hold"], hold)
        self.assertFalse(result["entry_authorized"])
        for needle in ("8.5 percent of the unventilated contamination remains",
                       "entry needs 8 percent or less", "about 2 seconds more",
                       "within permissible entry limits"):
            self.assertIn(needle, result["next_step"])
        self.assertNotIn("7 percent", result["next_step"])

    def test_probe_is_labelled_honestly_and_the_purge_gate_is_explained(self):
        doc = SpotTools.run_atmospheric_test.__doc__
        self.assertIn("extendable gas probe, simulated; arm-mounted on", doc)
        self.assertIn("six points", doc)
        self.assertIn("purge", doc)
        prompt = build_system_prompt(DEFAULT_SITE)
        for needle in ("simulated gas probe", "arm-mounted on the real robot", "one meter inside",
                       "six points", "purge incomplete", "repositions at the door and retries once"):
            self.assertIn(needle, prompt)
        self.assertNotIn("about half a meter inside the opening, sampling", prompt)


if __name__ == "__main__":
    unittest.main()
