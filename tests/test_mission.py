"""Unit tests for agent/mission.py, agent/robot_client.py, agent/run_mission.py.

Runs on a Mac without ROS (``python3 -m unittest discover -s tests`` from the
repo root). The robot is ``FakeRobot``: a unicycle model that integrates the
last ``send_velocity`` command with the facade's 0.125 s watchdog, a 0.15 s
velocity lag, 5 % speed loss and a small yaw drift (so the heading
controller has something to correct), a 4 Hz gas feed from a pluggable field,
and a collision checker against the real scene geometry from
sim/build_scene.py (pipe runs, door jambs, tank shell, ladder, barrels) that
models Spot's 1.1 x 0.5 m body as three 0.27 m circles.

Time runs on a ``ScaledClock`` (virtual seconds = wall seconds x speed) shared
by the mission, the SafetyGuard and the fake, so a 2-minute mission takes a
few seconds of wall time with every thread still running concurrently.

Storyline support (all opt-in, legacy tests see the old fake): a 3-D gas
field (``gas_fn3``) whose deviations from baseline decay under SIMULATED
forced ventilation exactly like sim/gas_field.py (fraction_remaining =
residual + (anchor - residual) * exp(-t / tau)), the facade's probe
(``probe_enabled``; 3 m reach), clear/reset controls, ``pose3`` body height
and a fall-after-stand model (``falls_on_stand``) for the self-right retry.
``field=`` plugs in the real sim/gas_field.GasField instead.
``arm_reach_m=`` adds the facade's P1-4 probe rules as rejections with an
error reply (RobotClient's "probe rejected by the simulator: ..." text):
no pose, a point beyond ``arm_reach_m`` (horizontal) from the nose, or a
point inside the tank whose straight line from the body crosses the shell
outside the 1.3 x 1.3 m doorway ("occluded by tank wall");
``force_probe_error`` rejects every request with that error.
"""

import ast
import json
import math
import os
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENT_DIR = os.path.join(REPO, "agent")
SIM_DIR = os.path.join(REPO, "sim")
for _path in (AGENT_DIR, SIM_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import mission as mission_mod  # noqa: E402
import robot_client  # noqa: E402
import run_mission  # noqa: E402
from mission import (ABORTED, COMPLETED, ENTRY_AUTHORIZED, NOT_STARTED,  # noqa: E402
                     PRE_ENTRY_TESTING, ConfinedSpaceMission, Permit, SafetyGuard,
                     alarm_reasons, gas_passes, normalize_site, wrap_angle)

# Tight GIL hand-off so threads on a scaled clock stay responsive.
sys.setswitchinterval(0.0005)

BASELINE = {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0, "co_ppm": 0.0}
SITE = robot_client.load_site_defaults()
STANDOFF_X = SITE["door_xy"][0] + ConfinedSpaceMission.SIT_STANDOFF_M  # 8.0


# SPOT_TEST_SLOWDOWN=N divides every ScaledClock's speed by N (default 1). The
# fakes, the mission and the SafetyGuard run real threads on virtual time, so on a
# slow or emulated machine they can fall behind a 20x clock and miss virtual
# deadlines that hold on a laptop; a slower clock gives them the same headroom.
TEST_SLOWDOWN = max(1.0, float(os.environ.get("SPOT_TEST_SLOWDOWN", "1") or 1))


class ScaledClock:
    """Virtual time running ``speed`` x faster than the wall clock (divided by
    SPOT_TEST_SLOWDOWN)."""

    def __init__(self, speed):
        self.speed = float(speed) / TEST_SLOWDOWN
        self._t0 = time.monotonic()

    def now(self):
        return (time.monotonic() - self._t0) * self.speed

    def sleep(self, seconds):
        if seconds > 0.0:
            time.sleep(seconds / self.speed)


# --- scene obstacles (numbers from sim/build_scene.py) ----------------------
PIPES = [((-2.0, -0.7), (-2.0, 5.5), 0.12), ((2.5, -5.3), (2.5, 0.6), 0.12)]
JAMBS = [(5.875, 6.125, 0.65, 0.87), (5.875, 6.125, -0.87, -0.65)]
LADDER = (-5.78, -5.72, -0.28, 0.28)
BARRELS = [((9.5, 3.0), 0.30), ((10.3, 3.6), 0.30)]
BODY_OFFSETS = (-0.30, 0.0, 0.30)
BODY_RADIUS = 0.27


def _seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _box_dist(p, box):
    x0, x1, y0, y1 = box
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def _obstacle_dist(p):
    d = min(_seg_dist(p, a, b) - r for a, b, r in PIPES)
    d = min([d] + [_box_dist(p, j) for j in JAMBS] + [_box_dist(p, LADDER)])
    d = min([d] + [math.hypot(p[0] - c[0], p[1] - c[1]) - r for c, r in BARRELS])
    rc = math.hypot(p[0], p[1])
    if rc > 1e-9:
        nx, ny = p[0] / rc * 6.0, p[1] / rc * 6.0
        in_door_gap = nx > 0.0 and abs(ny) < 0.65
        if not in_door_gap:
            d = min(d, max(0.0, abs(rc - 6.0) - 0.05))
    return d


def body_clearance(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return min(_obstacle_dist((x + o * c, y + o * s)) for o in BODY_OFFSETS) - BODY_RADIUS


def in_tank(x, y):
    return math.hypot(x, y) < 6.0 + 0.3


def unventilated_tank(x, y, z):
    """Raw (unventilated) tank for the storyline tests, shaped like the
    facade's headspace (sim/gas_field.py UNVENTILATED_HEADSPACE: H2S pooled
    low, ~7.5 % LEL, O2 ~19.9 %) plus an LEL pocket on the interior route.
    Door probe (5.5, 0): H2S 51 / 13.5 / 3.6 ppm at 0.2 / 0.6 / 1.0 m; body
    height inside: ~18 ppm H2S and up to ~26 % LEL. Outside: baseline."""
    reading = dict(BASELINE)
    if math.hypot(x, y) >= 6.0:
        return reading
    reading["h2s_ppm"] = 100.0 * math.exp(-z / 0.3)
    reading["lel_pct"] = (8.0 * math.exp(-z / 3.0) + 25.0 * math.exp(
        -((x - 1.0) ** 2 + (y - 0.8) ** 2) / (2 * 1.2 ** 2)) * math.exp(-z / 2.0))
    reading["o2_pct"] = 20.9 - 1.0 * math.exp(-z / 3.0)
    return reading


def osha_alarms(reading):
    alarms = {
        "o2": reading["o2_pct"] < 19.5 or reading["o2_pct"] > 23.5,
        "lel": reading["lel_pct"] >= 10.0,
        "h2s": reading["h2s_ppm"] >= 10.0,
        "co": reading["co_ppm"] >= 35.0,
    }
    alarms["any"] = any(alarms.values())
    return alarms


class FakeRobot:
    """Kinematic RobotClient stand-in (see module docstring)."""

    WATCHDOG_S = 0.125
    TAU_S = 0.15
    GAS_PERIOD_S = 0.25
    PROBE_REACH_M = 3.0        # sim/isaac_app.py PROBE_MAX_REACH_M
    STAND_Z, SIT_Z, FALLEN_Z = 0.52, 0.20, 0.12
    FALL_DELAY_S = 0.8         # host F1: fell within ~2 s of the stand

    def __init__(self, clock, start=(14.0, 0.0, math.pi), gas_fn=None,
                 speed_scale=0.95, yaw_bias=0.02, standing=True, gas_fn3=None,
                 probe_enabled=False, field=None, residual=0.04, arm_reach_m=None):
        self.clock = clock
        self.x, self.y, self.yaw = start
        self.v = self.w = self.u = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = -1.0e9
        self.t = clock.now()
        self.lock = threading.RLock()
        self.gas_fn = gas_fn or (lambda x, y: dict(BASELINE))
        self.gas_fn3 = gas_fn3          # raw (unventilated) field f(x, y, z)
        self.field = field              # optional sim/gas_field.GasField
        self.probe_enabled = probe_enabled
        self.arm_reach_m = arm_reach_m  # None: the old 3 m body-centre reach only
        self.force_probe_error = None   # str: the facade rejects every probe with it
        self.probe_rejections = []      # (t, error) for every rejected request
        self.probes = []                # (t, points)
        self.last_probe_error = None
        self.vent = {"on": False, "t_on": None, "tau_s": 15.0, "anchor": 1.0,
                     "residual": residual}
        self.falls_on_stand = 0         # upcoming stands that end in a fall
        self.fall_at = None
        self.fallen = False
        self.speed_scale = speed_scale
        self.yaw_bias = yaw_bias
        self.standing = standing
        self.stuck = False
        self.pose_ok = True
        self.telemetry = True
        self.owner = None
        self.spikes = []
        self.commands = []     # (t, vx, vy, wz, owner, accepted)
        self.calls = []        # (t, service)
        self.events = []
        self.trajectory = []   # (t, x, y, yaw) every 0.05 s
        self.min_clearance = float("inf")
        self.worst_pose = None
        self._gas_slot = None
        self._gas_cache = None
        self.first_alarm_sample_t = None
        self._last_traj_t = -1.0
        self.battery_pct = 100.0
        self.guard_beats = []  # payload/guard heartbeats
        self.odom_facade = True  # odometry_sample(): published by the facade alone
        self.odom_samples = []   # odometry_sample(): served first, one per call

    # -- kinematics --------------------------------------------------------
    def _integrate(self):
        now = self.clock.now()
        if self.fall_at is not None and now >= self.fall_at:
            self.fall_at, self.fallen = None, True
        while self.t < now:
            dt = min(0.01, now - self.t)
            active = (self.standing and not self.stuck and not self.fallen
                      and (self.t - self.cmd_t) <= self.WATCHDOG_S)
            vx_c, vy_c, wz_c = self.cmd if active else (0.0, 0.0, 0.0)
            alpha = min(1.0, dt / self.TAU_S)
            self.v += (vx_c * self.speed_scale - self.v) * alpha
            self.u += (vy_c * self.speed_scale - self.u) * alpha     # lateral (holonomic)
            self.w += (wz_c - self.w) * alpha
            rate = self.w + (self.yaw_bias if abs(self.v) > 0.05 else 0.0)
            self.yaw = math.atan2(math.sin(self.yaw + rate * dt), math.cos(self.yaw + rate * dt))
            self.x += (self.v * math.cos(self.yaw) - self.u * math.sin(self.yaw)) * dt
            self.y += (self.v * math.sin(self.yaw) + self.u * math.cos(self.yaw)) * dt
            self.t += dt
            clearance = body_clearance(self.x, self.y, self.yaw)
            if clearance < self.min_clearance:
                self.min_clearance, self.worst_pose = clearance, (self.x, self.y, self.yaw)
            if self.t - self._last_traj_t >= 0.05:
                self._last_traj_t = self.t
                self.trajectory.append((self.t, self.x, self.y, self.yaw))

    def true_pose(self):
        with self.lock:
            self._integrate()
            return (self.x, self.y, self.yaw)

    # -- RobotClient API ---------------------------------------------------
    def connect(self, timeout=30):
        return True

    def close(self):
        pass

    def _svc(self, name):
        with self.lock:
            self._integrate()
            self.calls.append((self.clock.now(), name))
            if name == "stand":
                self.standing = True
                if self.falls_on_stand > 0 and not self.fallen:
                    self.falls_on_stand -= 1
                    self.fall_at = self.clock.now() + self.FALL_DELAY_S
            elif name == "self_right":
                self.fallen, self.fall_at, self.standing = False, None, True
            elif name == "sit":
                self.standing = False
                self.cmd_t = -1.0e9
            elif name == "stop":
                self.cmd = (0.0, 0.0, 0.0)
                self.cmd_t = -1.0e9
            return True, "fake %s ok" % name

    def claim(self):
        return self._svc("claim")

    def release(self):
        return self._svc("release")

    def power_on(self):
        return self._svc("power_on")

    def power_off(self):
        return self._svc("power_off")

    def stand(self):
        return self._svc("stand")

    def sit(self):
        return self._svc("sit")

    def stop(self):
        return self._svc("stop")

    def self_right(self):
        return self._svc("self_right")

    def estop_hard(self):
        return self._svc("estop/hard")

    def estop_release(self):
        return self._svc("estop/release")

    def pose(self):
        with self.lock:
            self._integrate()
            return (self.x, self.y, self.yaw) if self.pose_ok else None

    ODOM_PERIOD_S = 0.02       # the facade's odometry is 50 Hz

    def odometry_sample(self):
        """RobotClient.odometry_sample() stand-in: the true pose numbered on
        the 50 Hz odometry grid, from the facade alone unless
        ``odom_facade`` is False; ``odom_samples`` are served first."""
        with self.lock:
            self._integrate()
            if self.odom_samples:
                return self.odom_samples.pop(0)
            if not self.pose_ok:
                return None
            return {"seq": math.floor(self.clock.now() / self.ODOM_PERIOD_S),
                    "pose": (self.x, self.y, self.yaw), "facade": self.odom_facade,
                    "why": None if self.odom_facade else "2 publishers on odometry"}

    def pose3(self):
        with self.lock:
            self._integrate()
            if not self.pose_ok:
                return None
            return (self.x, self.y, self._body_z(), self.yaw)

    def _body_z(self):
        if self.fallen:
            return self.FALLEN_Z
        return self.STAND_Z if self.standing else self.SIT_Z

    def sim_time(self):
        return self.clock.now()

    # -- SIMULATED gas controls (same semantics as sim/gas_field.py) -----
    def _fraction(self, t):
        v = self.vent
        if not v["on"]:
            return v["anchor"]
        r = v["residual"]
        return r + (v["anchor"] - r) * math.exp(-max(0.0, t - v["t_on"]) / v["tau_s"])

    def ventilation_state(self):
        now = self.clock.now()
        if self.field is not None:
            return self.field.ventilation_state(now)
        v = self.vent
        return {"on": v["on"], "t_on": v["t_on"], "tau_s": v["tau_s"],
                "fraction_remaining": self._fraction(now), "residual_fraction": v["residual"]}

    def set_ventilation(self, on, tau_s=None):
        with self.lock:
            now = self.clock.now()
            if self.field is not None:
                tau = self.field.ventilation_state()["tau_s"] if tau_s is None else tau_s
                self.field.set_ventilation(bool(on), now, tau_s=tau)
                return True
            tau = self.vent["tau_s"] if tau_s is None else float(tau_s)
            current = self._fraction(now)
            if on:
                if not (self.vent["on"] and tau == self.vent["tau_s"]):
                    self.vent.update(on=True, t_on=now, tau_s=tau, anchor=current)
            else:
                self.vent.update(on=False, t_on=None, anchor=current)
            return True

    def reset_ventilation(self):
        with self.lock:
            if self.field is not None:
                self.field.reset_ventilation(ventilated=False, t=self.clock.now())
            else:
                self.vent.update(on=False, t_on=None, tau_s=15.0, anchor=1.0)
            return True

    def clear_gas(self):
        with self.lock:
            if self.field is not None:
                self.field.clear_injected()
            self.spikes = []
            return True

    def _reading3(self, x, y, z):
        now = self.clock.now()
        if self.field is not None:
            reading = self.field.sample(x, y, z, t=now, noise=False)
        else:
            raw = self.gas_fn3(x, y, z) if self.gas_fn3 is not None else self.gas_fn(x, y)
            frac = self._fraction(now)
            reading = {k: BASELINE[k] + (raw[k] - BASELINE[k]) * frac for k in BASELINE}
            for gas, ppm, sx, sy, radius in self.spikes:
                key = mission_mod.GAS_KEYS[gas]
                d2 = (x - sx) ** 2 + (y - sy) ** 2
                reading[key] += ppm * math.exp(-d2 / (2.0 * radius * radius))
        alarms = osha_alarms(reading)
        reading["alarms"], reading["alarm"] = alarms, alarms["any"]
        return reading

    def _reading(self, x, y):
        reading = self._reading3(x, y, self._body_z())
        reading["ventilation"] = self.ventilation_state()
        reading["injected_pockets"] = (self.field.injected_count if self.field is not None
                                       else len(self.spikes))
        return reading

    def probe(self, points, timeout=3.0):
        """Facade probe stand-in: samples at [x, y, z] (normalized like
        RobotClient.probe), None when disabled or out of reach."""
        with self.lock:
            self._integrate()
            self.probes.append((self.clock.now(), [list(p) for p in points]))
            if not self.probe_enabled:
                self.last_probe_error = "probe disabled on this fake"
                return None
            error = self._facade_probe_error(points)
            if error is not None:
                self.probe_rejections.append((self.clock.now(), error))
                self.last_probe_error = "probe rejected by the simulator: " + error
                return None
            for px, py, _pz in points:
                if math.hypot(px - self.x, py - self.y) > self.PROBE_REACH_M:
                    self.last_probe_error = "point beyond the 3.0 m probe reach"
                    return None
            out = []
            for px, py, pz in points:
                reading = self._reading3(px, py, pz)
                reading.pop("alarm")
                out.append(dict(reading, x=px, y=py, z=pz))
            self.last_probe_error = None
            return out

    NOSE_M = 0.55              # body centre -> front of the body
    DOOR_HALF_WIDTH_M = 0.65   # 1.3 m clear opening
    DOOR_HEIGHT_M = 1.3

    def _facade_probe_error(self, points):
        """The facade's P1-4 probe rules (see module docstring), or None."""
        if self.force_probe_error:
            return self.force_probe_error
        if self.arm_reach_m is None:
            return None
        if not self.pose_ok:
            return "robot pose unavailable"
        nx = self.x + self.NOSE_M * math.cos(self.yaw)
        ny = self.y + self.NOSE_M * math.sin(self.yaw)
        bz = self._body_z()
        for px, py, pz in points:
            if math.hypot(px - nx, py - ny) > self.arm_reach_m:
                return ("point (%.2f, %.2f) is beyond the %.1f m probe reach from the body's front"
                        % (px, py, self.arm_reach_m))
            if math.hypot(px, py) < 6.0 <= math.hypot(self.x, self.y):
                dx, dy = px - self.x, py - self.y
                a = dx * dx + dy * dy
                b = 2.0 * (self.x * dx + self.y * dy)
                c = self.x * self.x + self.y * self.y - 36.0
                t = (-b - math.sqrt(max(0.0, b * b - 4.0 * a * c))) / (2.0 * a)
                cx, cy, cz = self.x + t * dx, self.y + t * dy, bz + t * (pz - bz)
                if not (cx > 0.0 and abs(cy) <= self.DOOR_HALF_WIDTH_M and cz <= self.DOOR_HEIGHT_M):
                    return "occluded by tank wall"
        return None

    def gas(self):
        with self.lock:
            if not self.telemetry:
                return None
            self._integrate()
            now = self.clock.now()
            slot = math.floor(now / self.GAS_PERIOD_S)  # 4 Hz publish grid
            if slot != self._gas_slot:
                self._gas_slot = slot
                self._gas_cache = self._reading(self.x, self.y)
                if self._gas_cache["alarm"] and self.first_alarm_sample_t is None:
                    self.first_alarm_sample_t = slot * self.GAS_PERIOD_S
            return dict(self._gas_cache)

    def status(self):
        # "simulated": true is how the facade identifies itself (backend
        # identity contract); the agent commands nothing without it.
        return {"claimed": True, "powered": True, "standing": self.standing,
                "standing_up": False, "estopped": False, "battery_pct": self.battery_pct,
                "sim_time": self.clock.now(), "simulated": True}

    def site(self, timeout=None):
        return robot_client.load_site_defaults()

    def set_motion_owner(self, owner):
        with self.lock:
            self.owner = owner

    def send_velocity(self, vx, vy=0.0, wz=0.0, owner=None):
        with self.lock:
            self._integrate()
            now = self.clock.now()
            moving = (vx, vy, wz) != (0.0, 0.0, 0.0)
            accepted = not (self.owner is not None and owner != self.owner and moving)
            self.commands.append((now, vx, vy, wz, owner, accepted))
            if accepted:
                self.cmd, self.cmd_t = (vx, vy, wz), now
            return accepted

    def inject_gas(self, gas, ppm, x, y, z, radius):
        with self.lock:
            self.spikes.append((gas, float(ppm), float(x), float(y), float(radius)))
            if self.field is not None:
                self.field.inject_spike(gas, float(ppm), float(x), float(y), float(z), float(radius))
        return True

    def publish_event(self, kind, message, data=None, source="mission"):
        event = robot_client.make_event(kind, message, data, source=source, t=self.clock.now())
        with self.lock:
            self.events.append(event)
        return event

    def publish_guard(self, beat):
        with self.lock:
            self.guard_beats.append(dict(beat))
        return True

    # -- test helpers ------------------------------------------------------
    def kinds(self):
        return [e["kind"] for e in self.events]

    def of_kind(self, kind):
        return [e for e in self.events if e["kind"] == kind]

    def accepted(self):
        return [c for c in self.commands if c[5]]

    def last_call(self, name):
        times = [t for t, n in self.calls if n == name]
        return times[-1] if times else None


def read_jsonl(path):
    with open(path, "r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


class MissionTestBase(unittest.TestCase):
    SPEED = 20.0

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log_path = os.path.join(self.tmp.name, "mission.jsonl")
        self.clock = ScaledClock(self.SPEED)
        self.missions = []

    def tearDown(self):
        for m in self.missions:
            if m._retreating:
                m.cancel("test teardown")
            m.close()
        self.tmp.cleanup()

    def make(self, robot, **kwargs):
        # These tests predate the permit-fields gate (TestPermitGate covers it)
        # and exercise the motion/guard logic with it off unless they opt in.
        kwargs.setdefault("require_permit", False)
        m = ConfinedSpaceMission(robot, robot.site(), self.log_path, clock=self.clock, **kwargs)
        self.missions.append(m)
        return m

    def assert_stopped(self, robot):
        accepted = robot.accepted()
        self.assertTrue(accepted, "no commands were sent")
        self.assertEqual(accepted[-1][1:4], (0.0, 0.0, 0.0), "last accepted twist is not zero")
        self.assertIsNotNone(robot.last_call("stop"))
        self.assertGreaterEqual(robot.last_call("stop"), accepted[-1][0] - 1e-9)

    def assert_collision_free(self, robot, margin=0.05):
        robot.true_pose()
        self.assertGreater(robot.min_clearance, margin,
                           "body clearance %.3f m at pose %s" % (robot.min_clearance,
                                                                  robot.worst_pose))


# ---------------------------------------------------------------------------
# Thresholds and site geometry
# ---------------------------------------------------------------------------


class TestThresholds(unittest.TestCase):
    def test_constants_match_sim_gas_field(self):
        try:
            import gas_field
        except ImportError as error:  # numpy missing
            self.skipTest("sim/gas_field.py not importable: %s" % error)
        self.assertEqual(mission_mod.O2_LOW_PCT, gas_field.O2_LOW_PCT)
        self.assertEqual(mission_mod.O2_HIGH_PCT, gas_field.O2_HIGH_PCT)
        self.assertEqual(mission_mod.LEL_ALARM_PCT, gas_field.LEL_ALARM_PCT)
        self.assertEqual(mission_mod.H2S_ALARM_PPM, gas_field.H2S_ALARM_PPM)
        self.assertEqual(mission_mod.CO_ALARM_PPM, gas_field.CO_ALARM_PPM)

    def test_alarm_reasons_agree_with_gas_field_alarms(self):
        try:
            import gas_field
        except ImportError as error:
            self.skipTest("sim/gas_field.py not importable: %s" % error)
        field = gas_field.GasField(seed=7)
        for i in range(400):
            x, y, z = -6 + (i % 20) * 0.6, -6 + (i // 20) * 0.6, 0.1 + (i % 7) * 0.1
            sample = field.sample(x, y, z)
            self.assertEqual(bool(alarm_reasons(sample)), gas_field.GasField.alarms(sample)["any"],
                             "disagreement at %s: %s" % ((x, y, z), sample))

    def test_boundaries(self):
        self.assertTrue(gas_passes("o2", 19.5))
        self.assertFalse(gas_passes("o2", 19.49))
        self.assertTrue(gas_passes("o2", 23.5))
        self.assertFalse(gas_passes("o2", 23.51))
        self.assertTrue(gas_passes("lel", 9.99))
        self.assertFalse(gas_passes("lel", 10.0))
        self.assertFalse(gas_passes("h2s", 10.0))
        self.assertTrue(gas_passes("co", 34.9))
        self.assertFalse(gas_passes("co", 35.0))
        self.assertFalse(gas_passes("h2s", float("nan")))
        self.assertEqual(alarm_reasons(BASELINE), [])
        self.assertTrue(alarm_reasons({"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0}))  # CO missing
        flagged = dict(BASELINE, alarm=True, alarms={"h2s": True, "any": True})
        self.assertIn("facade alarm", alarm_reasons(flagged)[0])


_ARITHMETIC = {ast.Add: lambda a, b: a + b, ast.Sub: lambda a, b: a - b,
               ast.Mult: lambda a, b: a * b, ast.Div: lambda a, b: a / b}


def _constant(node, env):
    """Value of a constant expression: literals (ast.literal_eval), names of
    constants already evaluated, math.pi, + - * / and unary minus. Anything
    else raises ValueError; the scene code is parsed, never executed."""
    if isinstance(node, ast.Name):
        if node.id in env:
            return env[node.id]
        raise ValueError(node.id)
    if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
            and node.value.id == "math" and node.attr == "pi"):
        return math.pi
    if isinstance(node, ast.BinOp) and type(node.op) in _ARITHMETIC:
        return _ARITHMETIC[type(node.op)](_constant(node.left, env), _constant(node.right, env))
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        value = _constant(node.operand, env)
        return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, (ast.Tuple, ast.List)):
        items = [_constant(item, env) for item in node.elts]
        return tuple(items) if isinstance(node, ast.Tuple) else items
    return ast.literal_eval(node)


def _build_scene_return():
    """Evaluate build_scene()'s returned dict literal without importing pxr
    (constant expressions only; the scene code is never executed)."""
    with open(os.path.join(SIM_DIR, "build_scene.py"), "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read())
    env = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)):
            try:
                env[node.targets[0].id] = _constant(node.value, env)
            except (ValueError, TypeError, SyntaxError, ZeroDivisionError):
                pass
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "build_scene")
    ret = [n for n in ast.walk(fn) if isinstance(n, ast.Return) and isinstance(n.value, ast.Dict)][-1]
    out = {}
    for key, value in zip(ret.value.keys, ret.value.values):
        try:
            out[key.value] = _constant(value, env)
        except (ValueError, TypeError, SyntaxError, ZeroDivisionError):
            pass   # e.g. camera_keepouts (a comprehension): not compared here
    return out, env


class TestSite(unittest.TestCase):
    def test_defaults_match_build_scene(self):
        scene, consts = _build_scene_return()

        def close(a, b):
            self.assertEqual(len(a), len(b))
            for u, v in zip(a, b):
                self.assertAlmostEqual(float(u), float(v), places=6)

        close(SITE["spawn_xy"], scene["spawn_xy"])
        self.assertAlmostEqual(SITE["spawn_yaw"], scene["spawn_yaw"], places=9)
        close(SITE["entry_xy"], scene["entry_xy"])
        close(SITE["door_xy"], scene.get("door_xy", scene.get("doorway_xy")))
        normal = scene.get("door_normal_yaw",
                           consts["DOOR_SEGMENT"] * 2.0 * math.pi / consts["NUM_SEGMENTS"])
        self.assertAlmostEqual(SITE["door_normal_yaw"], normal, places=9)
        self.assertEqual(len(SITE["interior_waypoints"]), len(scene["interior_waypoints"]))
        for a, b in zip(SITE["interior_waypoints"], scene["interior_waypoints"]):
            close(a, b)
        close(SITE["tank_center_xy"], scene.get("tank_center_xy", (0.0, 0.0)))
        self.assertAlmostEqual(SITE["tank_radius"], scene.get("tank_radius", consts["TANK_RADIUS"]))

    def test_normalize_accepts_doorway_xy_and_fills_missing(self):
        site = normalize_site({"doorway_xy": (6.0, 0.0), "entry_xy": [7, 0]})
        self.assertEqual(site["door_xy"], (6.0, 0.0))
        self.assertEqual(len(site["interior_waypoints"]), 5)
        with self.assertRaises(ValueError):
            normalize_site(dict(SITE, tank_radius=-1))
        with self.assertRaises(ValueError):
            normalize_site(dict(SITE, interior_waypoints=[]))

    def test_planned_route_is_collision_free(self):
        # Straight segments between consecutive route points, body aligned
        # with the segment - the geometric premise walk_to relies on.
        route = [tuple(SITE["entry_xy"]), tuple(SITE["door_xy"])] + [
            tuple(p) for p in SITE["interior_waypoints"]]
        for (ax, ay), (bx, by) in zip(route, route[1:]):
            yaw = math.atan2(by - ay, bx - ax)
            for i in range(101):
                t = i / 100.0
                x, y = ax + (bx - ax) * t, ay + (by - ay) * t
                self.assertGreater(body_clearance(x, y, yaw), 0.05, "segment clash at %s" % ((x, y),))


# ---------------------------------------------------------------------------
# robot_client without ROS
# ---------------------------------------------------------------------------


class TestRobotClientOffline(unittest.TestCase):
    def test_imports_and_degrades_without_connection(self):
        client = robot_client.RobotClient()
        self.assertEqual(client.namespace, "/demo_spot")
        ok, message = client.claim()
        self.assertFalse(ok)
        self.assertIn("not connected", message)
        self.assertIsNone(client.pose())
        self.assertIsNone(client.gas())
        self.assertIsNone(client.status())
        self.assertFalse(client.send_velocity(0.5, 0.0, 0.0))
        self.assertFalse(client.inject_gas("h2s", 80, 0, 0, 0.3, 2.0))
        self.assertEqual(client.site(timeout=0), SITE)
        event = client.publish_event("note", "hello", {"x": float("nan")}, source="agent")
        # P2-9: every event carries t (sim time when known, else wall) AND wall_t.
        self.assertEqual(set(event), {"t", "wall_t", "source", "kind", "message", "data"})
        self.assertAlmostEqual(event["wall_t"], time.time(), delta=5.0)
        self.assertIsNone(event["data"]["x"])
        client.close()  # no-op when never connected

    def test_connect_fails_cleanly_without_rclpy(self):
        try:
            import rclpy  # noqa: F401
            self.skipTest("rclpy installed; offline behaviour not testable here")
        except ImportError:
            pass
        client = robot_client.RobotClient()
        start = time.monotonic()
        self.assertFalse(client.connect(timeout=0.2))
        self.assertLess(time.monotonic() - start, 1.0)

    def test_helpers(self):
        self.assertEqual(robot_client.clamp_twist(5.0, -5.0, float("nan")),
                         (robot_client.MAX_VX, -robot_client.MAX_VY, 0.0))
        self.assertAlmostEqual(robot_client.yaw_from_quaternion(
            math.cos(0.5), 0.0, 0.0, math.sin(0.5)), 1.0)
        with self.assertRaises(ValueError):
            robot_client.make_event("k", "m", source="llm")


# ---------------------------------------------------------------------------
# walk_to controller
# ---------------------------------------------------------------------------


class TestWalkTo(MissionTestBase):
    def test_reaches_targets_ahead_and_behind(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        self.assertTrue(m.walk_to(10.0, 0.0))
        x, y, _ = robot.true_pose()
        self.assertLess(math.hypot(x - 10.0, y), 0.25 + 0.05)
        self.assert_stopped(robot)
        self.assertTrue(m.walk_to(12.0, 1.5))  # behind Spot: turn in place first
        x, y, _ = robot.true_pose()
        self.assertLess(math.hypot(x - 12.0, y - 1.5), 0.30)
        self.assert_stopped(robot)
        # Commands were streamed at ~20 Hz (watchdog is 0.125 s).
        moving = [c[0] for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)]
        gaps = [b - a for a, b in zip(moving, moving[1:])]
        self.assertLess(sorted(gaps)[len(gaps) // 2], 0.08)
        self.assert_collision_free(robot)

    def test_timeout_returns_false_stops_and_publishes_event(self):
        robot = FakeRobot(self.clock)
        robot.stuck = True
        m = self.make(robot)
        t0 = self.clock.now()
        self.assertFalse(m.walk_to(10.0, 0.0, timeout=3.0))
        elapsed = self.clock.now() - t0
        self.assertGreaterEqual(elapsed, 3.0)
        self.assertLess(elapsed, 4.5)
        self.assertEqual(len(robot.of_kind("walk_timeout")), 1)
        self.assert_stopped(robot)

    def test_lost_odometry_aborts_leg(self):
        robot = FakeRobot(self.clock)
        robot.pose_ok = False
        m = self.make(robot)
        self.assertFalse(m.walk_to(10.0, 0.0, timeout=30.0))
        self.assertEqual(len(robot.of_kind("pose_lost")), 1)
        self.assert_stopped(robot)

    def test_refuses_to_enter_without_authorization(self):
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi))
        m = self.make(robot)
        self.assertFalse(m.walk_to(4.6, 0.0))
        self.assertEqual(len(robot.of_kind("entry_blocked")), 1)
        self.assertEqual(robot.accepted(), [])

    def test_exception_mid_walk_leaves_robot_stopped(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        original = m._compute_command
        calls = [0]

        def flaky(*args):
            calls[0] += 1
            if calls[0] > 20:
                raise RuntimeError("injected controller bug")
            return original(*args)

        m._compute_command = flaky
        with self.assertRaises(RuntimeError):
            m.walk_to(10.0, 0.0)
        self.assert_stopped(robot)


# ---------------------------------------------------------------------------
# Atmospheric test and permit gating
# ---------------------------------------------------------------------------


def lel_at_door_mouth(x, y):
    reading = dict(BASELINE)
    if 6.0 < x < 6.9 and abs(y) < 1.0:
        reading["lel_pct"] = 14.0
    return reading


class TestAtmosphericTest(MissionTestBase):
    def test_osha_order_and_authorization(self):
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi))
        m = self.make(robot)
        result = m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=3.0)
        self.assertTrue(result["passed"], result)
        tests = robot.of_kind("atmos_test")
        self.assertEqual([e["data"]["gas"] for e in tests], ["o2", "lel", "h2s", "co"])
        self.assertTrue(all(e["data"]["passed"] for e in tests))
        self.assertTrue(all("PASS" in e["message"] for e in tests))
        # Sequential phases: O2 is judged before LEL, LEL before H2S/CO.
        t = [e["t"] for e in tests]
        self.assertGreater(t[1] - t[0], 0.7)
        self.assertGreater(t[2] - t[1], 0.7)
        self.assertLessEqual(t[2], t[3])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        x, _, _ = robot.true_pose()
        self.assertAlmostEqual(x, m.door_mouth_xy[0], delta=0.3)

    def test_failing_pre_entry_test_blocks_entry(self):
        robot = FakeRobot(self.clock, gas_fn=lel_at_door_mouth)
        m = self.make(robot, dwell_s=3.0)
        summary = m.run()
        self.assertEqual(summary["outcome"], ABORTED)
        self.assertEqual(summary["permit"], ABORTED)
        self.assertEqual(summary["abort_cause"], "entry_denied")
        self.assertTrue(summary["aborted_as_designed"])
        self.assertEqual(run_mission.exit_code_for(summary), 2)
        verdicts = {e["data"]["gas"]: e["data"]["passed"] for e in robot.of_kind("atmos_test")}
        self.assertEqual(verdicts, {"o2": True, "lel": False, "h2s": True, "co": True})
        self.assertEqual(robot.of_kind("entry_denied")[0]["kind"], "entry_denied")
        self.assertFalse(any(in_tank(x, y) for _, x, y, _ in robot.trajectory),
                         "robot entered the space without authorization")
        self.assertNotIn("ENTRY_AUTHORIZED",
                         [e["data"]["state"] for e in robot.of_kind("permit")])
        x, y, _ = robot.true_pose()
        self.assertLess(math.hypot(x - 7.0, y), 0.4)
        self.assertEqual(robot.calls[-1][1], "sit")
        self.assert_collision_free(robot)

        # Atmosphere ventilated: the same mission object re-runs under a new
        # permit (from the entry point, facing away from the door) and completes.
        first_permit = summary["permit_id"]
        robot.gas_fn = lambda x, y: dict(BASELINE)
        time.sleep(1.05)  # permit ids have 1 s resolution
        again = m.run()
        self.assertEqual(again["outcome"], COMPLETED, again)
        self.assertNotEqual(again["permit_id"], first_permit)
        self.assert_collision_free(robot)


# ---------------------------------------------------------------------------
# Full mission: clean run and H2S excursion
# ---------------------------------------------------------------------------


def h2s_pocket_on_leg_2(x, y):
    """H2S pocket sitting on the wp1 (2.5, 1.6) -> wp2 (0, 0) leg."""
    reading = dict(BASELINE)
    reading["h2s_ppm"] = 30.0 * math.exp(-((x - 0.9) ** 2 + (y - 0.6) ** 2) / (2 * 0.6 ** 2))
    return reading


class TestFullMission(MissionTestBase):
    def test_clean_run_completes(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot, dwell_s=3.0)
        summary = m.run()
        self.assertEqual(summary["outcome"], COMPLETED, summary)
        self.assertEqual(summary["permit"], COMPLETED)
        self.assertEqual(summary["waypoints_reached"], 5)
        self.assertEqual(run_mission.exit_code_for(summary), 0)
        states = [e["data"]["state"] for e in robot.of_kind("permit")]
        self.assertEqual(states, ["PRE_ENTRY_TESTING", "ENTRY_AUTHORIZED",
                                  "ENTRY_IN_PROGRESS", "COMPLETED"])
        for wx, wy in SITE["interior_waypoints"]:
            self.assertLess(min(math.hypot(x - wx, y - wy) for _, x, y, _ in robot.trajectory),
                            0.3, "waypoint (%s, %s) not visited" % (wx, wy))
        x, y, _ = robot.true_pose()
        # Exited facing away from the tank: it steps out to the sit standoff
        # (door + SIT_STANDOFF_M) so the next stand-up cannot shove it into the door.
        self.assertLess(math.hypot(x - STANDOFF_X, y), 0.35)
        self.assertEqual(robot.calls[-1][1], "sit")
        self.assert_collision_free(robot)
        self.assertEqual(robot.of_kind("alarm"), [])
        # Audit log: every event mirrored, summary last.
        records = read_jsonl(self.log_path)
        self.assertEqual([r["kind"] for r in records], robot.kinds())
        self.assertEqual(records[-1]["kind"], "summary")
        self.assertEqual(records[-1]["data"]["outcome"], COMPLETED)
        self.assertEqual([r["seq"] for r in records], list(range(1, len(records) + 1)))

    def test_h2s_spike_mid_entry_triggers_retreat(self):
        robot = FakeRobot(self.clock, gas_fn=h2s_pocket_on_leg_2)
        m = self.make(robot, dwell_s=3.0)
        summary = m.run()
        self.assertEqual(summary["outcome"], ABORTED, summary)
        self.assertEqual(summary["permit"], ABORTED)
        self.assertEqual(summary["abort_cause"], "atmosphere")
        self.assertTrue(summary["aborted_as_designed"])
        self.assertTrue(summary["retreat"]["ok"])
        self.assertEqual(summary["waypoints_reached"], 2)
        self.assertEqual(run_mission.exit_code_for(summary), 2)
        self.assertGreaterEqual(summary["trigger_reading"]["h2s_ppm"], 10.0)

        # Guard reaction: forward motion stops within 0.5 s of the first
        # alarm sample on the 4 Hz feed.
        t_alarm = robot.first_alarm_sample_t
        self.assertIsNotNone(t_alarm)
        after = [c for c in robot.accepted() if c[0] >= t_alarm]
        t_stop = next(c[0] for c in after if c[1] <= 0.0)
        self.assertLess(t_stop - t_alarm, 0.5)
        pose_alarm = min(robot.trajectory, key=lambda p: abs(p[0] - t_alarm))
        deepest = min(math.hypot(x, y) for t, x, y, _ in robot.trajectory if t >= t_alarm)
        self.assertGreater(deepest, math.hypot(pose_alarm[1], pose_alarm[2]) - 0.3,
                           "kept advancing into the pocket after the alarm")

        # Back out through the door to the entry point, then sit.
        after_traj = [(x, y) for t, x, y, _ in robot.trajectory if t >= t_alarm]
        self.assertTrue(any(5.8 < x < 6.2 and abs(y) < 0.4 for x, y in after_traj),
                        "did not pass back through the doorway")
        x, y, _ = robot.true_pose()
        # Exited facing away from the tank: it steps out to the sit standoff
        # (door + SIT_STANDOFF_M) so the next stand-up cannot shove it into the door.
        self.assertLess(math.hypot(x - STANDOFF_X, y), 0.6)
        self.assertEqual(robot.calls[-1][1], "sit")
        self.assert_collision_free(robot)

        # Audit trail: alarm (guard) -> permit ABORTED with the reading ->
        # retreat_start -> retreat_complete -> summary.
        records = read_jsonl(self.log_path)
        kinds = [r["kind"] for r in records]
        i_alarm = kinds.index("alarm")
        aborted = next(r for r in records if r["kind"] == "permit" and r["data"]["state"] == ABORTED)
        self.assertGreater(records.index(aborted), i_alarm)
        self.assertEqual(records[i_alarm]["source"], "guard")
        self.assertGreaterEqual(aborted["data"]["reading"]["h2s_ppm"], 10.0)
        self.assertLess(kinds.index("retreat_start"), kinds.index("retreat_complete"))
        self.assertEqual(kinds[-1], "summary")

    def test_cli_injection_at_waypoint_exits_2(self):
        # The CLI requires the permit, so only a probe test can authorize entry.
        robot = FakeRobot(self.clock, probe_enabled=True)
        args = run_mission.build_parser().parse_args(
            ["--inject-h2s-at-waypoint", "2", "--log", self.log_path, "--dwell", "3",
             "--rehearsal-auto-confirm"])
        code = run_mission.execute(args, robot, clock=self.clock)
        self.assertEqual(code, 2)
        self.assertEqual(len(robot.spikes), 1)
        _, ppm, sx, sy, _ = robot.spikes[0]
        self.assertLess(math.hypot(sx - 0.0, sy - 0.0), 0.3)  # waypoint 2 = tank center
        kinds = [r["kind"] for r in read_jsonl(self.log_path)]
        self.assertLess(kinds.index("demo_inject"), kinds.index("alarm"))
        self.assertIn("retreat_complete", kinds)
        self.assertEqual(kinds[-1], "summary")
        self.assert_collision_free(robot)

    def test_rehearsal_mode_logs_alarm_but_completes(self):
        robot = FakeRobot(self.clock, gas_fn=h2s_pocket_on_leg_2)
        m = self.make(robot, dwell_s=3.0)
        summary = m.run(auto_retreat=False)
        self.assertEqual(summary["outcome"], COMPLETED)
        self.assertGreaterEqual(summary["alarms_ignored"], 1)
        self.assertTrue(robot.of_kind("alarm_ignored"))
        self.assertFalse(summary["auto_retreat"])
        # Rehearsal is scoped to that run: the guard retreats again afterwards.
        self.assertTrue(m.auto_retreat)


# ---------------------------------------------------------------------------
# SafetyGuard
# ---------------------------------------------------------------------------


class TestSafetyGuard(MissionTestBase):
    SPEED = 5.0

    def test_guard_reacts_within_half_second_and_latches(self):
        state = {"h2s": 0.0}
        robot = FakeRobot(self.clock, start=(0.0, 0.0, 0.0),
                          gas_fn=lambda x, y: dict(BASELINE, h2s_ppm=state["h2s"]))
        fired = []
        guard = SafetyGuard(robot, lambda r, why: fired.append((self.clock.now(), r, why)),
                            clock=self.clock)
        guard.start()
        try:
            self.clock.sleep(1.0)
            self.assertEqual(fired, [])
            state["h2s"] = 25.0
            self.clock.sleep(1.5)
            self.assertEqual(len(fired), 1, "guard must fire exactly once per excursion")
            latency = fired[0][0] - robot.first_alarm_sample_t
            self.assertGreaterEqual(latency, 0.0)
            self.assertLess(latency, 0.5)
            self.assertIn("H2S", fired[0][2][0])
            state["h2s"] = 0.0          # excursion clears -> latch re-opens
            self.clock.sleep(0.75)
            state["h2s"] = 25.0
            self.clock.sleep(0.75)
            self.assertEqual(len(fired), 2)
        finally:
            guard.stop()

    def test_telemetry_loss_fires_and_disarmed_guard_is_silent(self):
        robot = FakeRobot(self.clock, start=(0.0, 0.0, 0.0))
        fired, silent = [], []
        guard = SafetyGuard(robot, lambda r, why: fired.append((r, why)), stale_s=1.0,
                            clock=self.clock)
        quiet = SafetyGuard(robot, lambda r, why: silent.append(why), stale_s=1.0,
                            armed=lambda: False, clock=self.clock)
        guard.start()
        quiet.start()
        try:
            robot.telemetry = False
            self.clock.sleep(1.6)
            self.assertEqual(len(fired), 1)
            self.assertIsNone(fired[0][0])
            self.assertIn("telemetry lost", fired[0][1][0])
            self.assertEqual(silent, [])
        finally:
            guard.stop()
            quiet.stop()

    def test_guard_retreats_robot_driven_in_by_agent(self):
        # The "LLM" drove Spot inside with raw cmd_vel: no permit, no trail.
        # The guard must still pull it out, and ignore the agent meanwhile.
        state = {"h2s": 0.0}
        robot = FakeRobot(self.clock, start=(4.0, 0.0, math.pi),
                          gas_fn=lambda x, y: dict(BASELINE, h2s_ppm=state["h2s"] if in_tank(x, y) else 0.0))
        m = self.make(robot)
        m._ensure_guard()
        self.clock.sleep(0.5)
        state["h2s"] = 40.0
        deadline = self.clock.now() + 3.0
        while not m._retreating and self.clock.now() < deadline:
            self.clock.sleep(0.05)
        self.assertTrue(m._retreating, "guard did not start a retreat")
        self.assertFalse(robot.send_velocity(0.5, 0.0, 0.0), "agent command not blocked")
        self.assertFalse(m.walk_to(0.0, 0.0))
        self.assertTrue(m._wait_retreat(timeout=30.0))
        self.assertTrue(m.status_snapshot()["retreat_ok"])
        x, y, _ = robot.true_pose()
        # Exited facing away from the tank: it steps out to the sit standoff
        # (door + SIT_STANDOFF_M) so the next stand-up cannot shove it into the door.
        self.assertLess(math.hypot(x - STANDOFF_X, y), 0.6)
        self.assertIsNone(robot.owner, "motion ownership not released")
        self.assert_collision_free(robot)


# ---------------------------------------------------------------------------
# Regressions from the adversarial review
# ---------------------------------------------------------------------------


class TestReviewRegressions(MissionTestBase):
    def test_failed_retest_withdraws_authorization(self):
        # Authorized, then a re-test before entry fails: the permit must not
        # stay ENTRY_AUTHORIZED (walk_to / the agent would still let Spot in).
        state = {"lel": 0.0}
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi),
                          gas_fn=lambda x, y: dict(BASELINE, lel_pct=state["lel"]))
        m = self.make(robot)
        self.assertTrue(m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        state["lel"] = 15.0
        self.assertFalse(m.atmospheric_test(dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ABORTED)
        # T22 (M22): the SafetyGuard is armed while the permit is
        # ENTRY_AUTHORIZED, so the alarm itself aborts the authorization
        # (and backs Spot off) before the failed re-test would withdraw it.
        self.assertEqual(m.status_snapshot()["abort_cause"], "atmosphere")
        self.assertTrue(robot.of_kind("alarm"))
        m._wait_retreat()
        state["lel"] = 0.0
        self.clock.sleep(0.5)
        self.assertFalse(m.walk_to(*m.door_xy))
        self.assertTrue(robot.of_kind("entry_blocked"))
        self.assertFalse(any(in_tank(x, y) for _, x, y, _ in robot.trajectory))

    def test_leg_through_the_tank_needs_authorization(self):
        # A target beyond the tank is "outside", but the straight leg to it
        # runs through the permit space.
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi))
        m = self.make(robot)
        self.assertFalse(m.walk_to(-7.0, 0.0, timeout=10.0))
        self.assertEqual(len(robot.of_kind("entry_blocked")), 1)
        self.assertEqual([c for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)], [])

    def test_unplanned_straight_exit_from_deep_inside_is_refused(self):
        robot = FakeRobot(self.clock, start=(-2.0, -1.7, 0.0))
        m = self.make(robot)
        self.assertFalse(m.walk_to(7.0, 0.0, timeout=10.0))  # would cross pipe run B
        self.assertEqual(len(robot.of_kind("walk_refused")), 1)
        self.assertEqual([c for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)], [])

    def test_retreat_from_outside_never_approaches_the_tank(self):
        # From spawn it must not walk 7 m toward the door; from beside the
        # shell a straight line to entry_xy would cut through the tank wall.
        for start in ((14.0, 0.0, math.pi), (0.0, 6.8, 0.0)):
            robot = FakeRobot(self.clock, start=start)
            m = self.make(robot)
            self.assertTrue(m.retreat(reason="operator"), start)
            x, y, _ = robot.true_pose()
            self.assertLess(math.hypot(x - start[0], y - start[1]), 0.05, start)
            self.assertEqual([c for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)], [])
            self.assertTrue(robot.of_kind("retreat_complete"))
            self.assertEqual(robot.calls[-1][1], "sit")
        # Just outside the door it still backs off to the entry point.
        robot = FakeRobot(self.clock, start=(6.5, 0.0, math.pi))
        m = self.make(robot)
        self.assertTrue(m.retreat(reason="operator"))
        x, y, _ = robot.true_pose()
        self.assertLess(math.hypot(x - 7.0, y), 0.6)

    def test_sits_at_standoff_when_the_door_is_behind(self):
        # Host regression (2026-09-27): sat at entry_xy facing away from the
        # tank, the next stand-up shoved Spot backward into the door frame.
        robot = FakeRobot(self.clock, start=(6.8, 0.0, 0.0))
        m = self.make(robot)
        self.assertTrue(m._sit_if_outside())
        x, y, _ = robot.true_pose()
        self.assertLess(math.hypot(x - STANDOFF_X, y), 0.3)
        self.assertEqual(robot.calls[-1][1], "sit")
        self.assertTrue(all(c[1] >= 0.0 for c in robot.accepted()), "drove toward the door")
        # Facing the door (the shove points away from the tank): sit in place.
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi))
        m = self.make(robot)
        self.assertTrue(m._sit_if_outside())
        self.assertEqual([c for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)], [])
        self.assertEqual(robot.calls[-1][1], "sit")

    def test_leg_halted_by_a_retreat_that_ends_within_one_tick(self):
        # The retreat is instant (Spot already at entry_xy) and clears _halt
        # before the mission loop wakes; the leg must still stop, not keep
        # driving a sitting robot toward the door.
        robot = FakeRobot(self.clock, start=(6.85, 0.0, math.pi))
        m = self.make(robot)
        m.CONTROL_HZ = 1.0  # widen the tick so the retreat fits inside it
        with m._lock:
            m._permit = ENTRY_AUTHORIZED
            # The door check (T19) has just passed: this test is about the leg.
            m._door_check = {"t": m._now(), "permit_id": m.permit_id, "passed": True}
        result = {}
        walker = threading.Thread(target=lambda: result.update(ok=m.walk_to(*m.door_xy, timeout=20)))
        walker.start()
        self.clock.sleep(0.3)
        t_retreat = self.clock.now()
        self.assertTrue(m.retreat(reason="operator"))
        walker.join(timeout=10.0)
        self.assertFalse(walker.is_alive())
        self.assertFalse(result["ok"])
        late = [c for c in robot.accepted()
                if c[0] > t_retreat and c[4] == "mission" and c[1:4] != (0.0, 0.0, 0.0)]
        self.assertEqual(late, [], "mission leg kept driving after the retreat")
        self.assertLess(self.clock.now() - t_retreat, 5.0)


class TestRobotClientMotionOwner(unittest.TestCase):
    def test_owner_check_and_publish_are_atomic(self):
        # set_motion_owner("retreat") must not slip in between a non-owner's
        # owner check and its publish (the stale twist would go out after
        # the retreat's zero twist).
        import types
        client = robot_client.RobotClient()
        blocked = []

        class Publisher:
            def publish(self, _msg):
                taker = threading.Thread(target=client.set_motion_owner, args=("retreat",))
                taker.start()
                taker.join(timeout=0.1)
                blocked.append(taker.is_alive())
                self.taker = taker

        pub = Publisher()
        client._node = GraphNode()  # "connected" without ROS
        identify_facade(client)
        client._pub_cmd = pub
        client._msg = {"Twist": lambda: types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0), angular=types.SimpleNamespace(z=0.0))}
        try:
            self.assertTrue(client.send_velocity(0.3, 0.0, 0.0, owner="mission"))
            pub.taker.join(timeout=1.0)
            self.assertEqual(blocked, [True], "set_motion_owner ran during a publish")
            self.assertEqual(client.motion_owner, "retreat")
            self.assertFalse(client.send_velocity(0.3, 0.0, 0.0, owner="mission"))
            self.assertTrue(client.send_velocity(0.0, 0.0, 0.0, owner="mission"))
        finally:
            client._node = None


# ---------------------------------------------------------------------------
# Storyline: permit fields, probe test, ventilation, incidents, self-right
# ---------------------------------------------------------------------------

DOOR_PROBE = [(5.5, 0.0, z) for z in (0.2, 0.6, 1.0)]


# What agent/operator_console.py records when the operator types "y" (T5:
# isolation and LOTO are only ever recorded from such an attestation).
ATTESTATION = {"action": "permit_attestation", "decision": "confirmed", "answer": "y",
               "mode": "responder", "operator": {"user": "test-operator", "ssm_session": None}}


def complete_permit(m, **overrides):
    fields = dict(permit_id="CSE-TEST-%d" % id(m),
                  rescue_plan="Standby rescue team at the door (test)")
    fields.update(overrides)
    m.prepare_permit(**fields)
    result = m.prepare_permit(isolation_confirmed=True, loto_confirmed=True,
                              attendant=mission_mod.ATTENDANT_ROLE,
                              entry_supervisor=mission_mod.ENTRY_SUPERVISOR_ROLE,
                              attestation=ATTESTATION)
    assert result["complete"], result
    return result


def moving(robot):
    return [c for c in robot.accepted() if c[1:4] != (0.0, 0.0, 0.0)]


def permit_states(robot):
    return [e["data"]["state"] for e in robot.of_kind("permit") if "state" in (e["data"] or {})]


class TestPermitGate(MissionTestBase):
    def test_no_approach_until_every_field_is_recorded(self):
        # With the permit required only a probe test can authorize entry.
        robot = FakeRobot(self.clock, probe_enabled=True)
        m = self.make(robot, require_permit=True)
        self.assertFalse(m.walk_to(*m.entry_xy))
        self.assertEqual(len(robot.of_kind("permit_incomplete")), 1)
        self.assertEqual(moving(robot), [])
        self.assertTrue(m.walk_to(12.0, 1.5))  # open ground well away from the tank is fine

        partial = m.prepare_permit(permit_id="CSE-0042", rescue_plan="Standby team at the door")
        self.assertTrue(partial["ok"])
        self.assertFalse(partial["complete"])
        self.assertEqual(partial["missing"], ["isolation_confirmed", "loto_confirmed", "attendant",
                                              "entry_supervisor"])
        self.assertFalse(m.walk_to(*m.entry_xy))
        self.assertIn("LOTO", robot.of_kind("permit_incomplete")[-1]["message"])

        bad = m.prepare_permit(isolation_confirmed="yes")  # a string is not a confirmation
        self.assertFalse(bad["ok"])
        self.assertIn("isolation_confirmed", bad["missing"])
        # T5: True without the operator's console attestation is refused too.
        unattested = m.prepare_permit(isolation_confirmed=True, loto_confirmed=True)
        self.assertFalse(unattested["ok"])
        self.assertIn("loto_confirmed", unattested["missing"])

        done = m.prepare_permit(isolation_confirmed=True, loto_confirmed=True,
                                attendant=mission_mod.ATTENDANT_ROLE,
                                entry_supervisor=mission_mod.ENTRY_SUPERVISOR_ROLE,
                                attestation=ATTESTATION)
        self.assertTrue(done["complete"], done)
        self.assertEqual(m.permit_id, "CSE-0042")
        self.assertTrue(done["permit"]["issued_at"])
        prepared = robot.of_kind("permit_prepared")
        self.assertEqual(len(prepared), 4)
        self.assertEqual(prepared[-1]["data"]["permit"]["attendant"], "attendant")
        self.assertTrue(prepared[-1]["data"]["permit"]["loto_confirmed"])
        self.assertEqual(prepared[-1]["data"]["permit"]["attestation"]["answer"], "y")
        self.assertTrue(m.walk_to(*m.entry_xy))

        # Once authorized, the permit fields are frozen.
        self.assertTrue(m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        self.assertFalse(m.prepare_permit(rescue_plan="another plan")["ok"])
        # Permit transitions carry the Permit dict for the HUD checklist.
        last = robot.of_kind("permit")[-1]["data"]
        self.assertEqual(last["permit"]["permit_id"], "CSE-0042")
        self.assert_collision_free(robot)

    def test_leaving_the_zone_needs_no_permit(self):
        robot = FakeRobot(self.clock, start=(7.2, 0.0, 0.0))  # at the entry, facing away
        m = self.make(robot, require_permit=True)
        self.assertFalse(m.walk_to(6.6, 0.0))
        self.assertTrue(m.walk_to(10.0, 0.0))
        self.assertEqual(len(robot.of_kind("permit_incomplete")), 1)

    def test_run_stops_before_any_motion_without_a_permit(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot, require_permit=True, dwell_s=3.0)
        m.prepare_permit(rescue_plan="Standby team at the door")
        summary = m.run()
        self.assertEqual(summary["outcome"], "FAILED")
        self.assertIn("incomplete", summary["reason"])
        self.assertEqual(run_mission.exit_code_for(summary), 1)
        self.assertEqual(robot.calls, [])      # not even claimed or stood up
        self.assertEqual(moving(robot), [])
        self.assertEqual(summary["permit"], NOT_STARTED)

    def test_a_test_in_place_cannot_authorize_without_a_permit(self):
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi))
        m = self.make(robot, require_permit=True)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertFalse(result["passed"])
        self.assertIn("permit", result["failures"][0])
        self.assertEqual(m.permit, NOT_STARTED)
        self.assertEqual(robot.of_kind("atmos_test"), [])

    def test_new_permit_starts_blank(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertTrue(m.retreat(reason="test"))  # outside: closes the permit, no motion
        self.assertEqual(m.permit, ABORTED)
        self.assertTrue(m.new_permit())
        self.assertEqual(m.permit_missing(),
                         ["isolation_confirmed", "loto_confirmed", "attendant", "rescue_plan",
                          "entry_supervisor"])
        self.assertFalse(robot.of_kind("permit")[-1]["data"]["permit"]["complete"])

    def test_permit_dataclass(self):
        p = Permit("CSE-1", True, 1, "A", "plan")
        self.assertEqual(p.missing(), ["loto_confirmed", "entry_supervisor"])  # 1 is not True
        self.assertFalse(p.as_dict()["complete"])


class TestProbeAtmosphericTest(MissionTestBase):
    def test_probe_inside_the_opening_finds_pooled_h2s_the_body_sensor_misses(self):
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi), gas_fn3=unventilated_tank,
                          probe_enabled=True)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=3.0)
        self.assertFalse(result["passed"])
        self.assertEqual(result["method"], "probe")
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)  # entry denied, re-test possible
        self.assertEqual(len(robot.of_kind("entry_denied")), 1)
        self.assertNotIn(ABORTED, permit_states(robot))
        # The body sensor at the door mouth reads clean air (outside the tank).
        self.assertEqual(robot.gas()["h2s_ppm"], 0.0)

        tests = robot.of_kind("atmos_test")
        self.assertEqual([e["data"]["gas"] for e in tests], ["o2", "lel", "h2s", "co"])
        verdicts = {e["data"]["gas"]: e["data"]["passed"] for e in tests}
        self.assertEqual(verdicts, {"o2": True, "lel": True, "h2s": False, "co": True})
        t = [e["t"] for e in tests]
        self.assertGreater(t[1] - t[0], 0.7)
        self.assertGreater(t[2] - t[1], 0.7)
        h2s = tests[2]["data"]
        self.assertEqual(h2s["worst_z_m"], 0.2)
        self.assertAlmostEqual(h2s["value"], 100.0 * math.exp(-0.2 / 0.3), delta=0.5)
        self.assertEqual(sorted(h2s["by_height"]), ["0.20", "0.60", "1.00"])
        self.assertLess(h2s["by_height"]["1.00"], 10.0)  # a single 1 m probe would have passed
        self.assertIn("worst at 0.2 m", h2s["message"])

        # Every probe request: the three heights 0.5 m inside the opening.
        self.assertGreaterEqual(len(robot.probes), 3)
        for _t, points in robot.probes:
            for (x, y, z), (ex, ey, ez) in zip(points, DOOR_PROBE):
                self.assertAlmostEqual(x, ex, places=6)
                self.assertAlmostEqual(y, ey, places=6)
                self.assertAlmostEqual(z, ez, places=6)
            # zip() above stops at DOOR_PROBE's 3 points: every request must
            # also carry the 1.0 m station (6 points, the 0.5 m ones first).
            self.assertEqual(len(points), 6)
            for (x, y, z), (ex, ey, ez) in zip(points[3:], [(5.0, 0.0, z) for z in (0.2, 0.6, 1.0)]):
                self.assertAlmostEqual(x, ex, places=6)
                self.assertAlmostEqual(y, ey, places=6)
                self.assertAlmostEqual(z, ez, places=6)
        rows = robot.of_kind("atmos_probe")[0]["data"]["samples"]
        # P1-3: six points - 0.5 and 1.0 m inside the opening x three heights.
        self.assertEqual([(r["inside_m"], r["z"]) for r in rows],
                         [(0.5, 0.2), (0.5, 0.6), (0.5, 1.0), (1.0, 0.2), (1.0, 0.6), (1.0, 1.0)])
        self.assertEqual([r["z"] for r in rows[:3]], [0.2, 0.6, 1.0])
        self.assertGreater(rows[0]["h2s_ppm"], 50.0)
        self.assertNotIn("alarm", robot.of_kind("atmos_probe")[0]["data"])  # no HUD flash
        self.assertEqual(robot.probes and [n for _, n in robot.calls if n != "stop"], [])

    def test_each_gas_is_judged_on_its_worst_height(self):
        def layered(x, y, z):
            reading = dict(BASELINE)
            if math.hypot(x - 5.5, y) < 0.1:
                if abs(z - 0.6) < 1e-6:
                    reading["o2_pct"] = 24.0   # enrichment only at 0.6 m
                if abs(z - 1.0) < 1e-6:
                    reading["lel_pct"] = 12.0  # vapour only at 1.0 m
                if abs(z - 0.2) < 1e-6:
                    reading["co_ppm"] = 40.0   # CO only at 0.2 m
            return reading

        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), gas_fn3=layered,
                          probe_enabled=True)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertFalse(result["passed"])
        by_gas = {e["gas"]: e for e in result["results"]}
        self.assertEqual([e["gas"] for e in result["results"]], ["o2", "lel", "h2s", "co"])
        self.assertEqual((by_gas["o2"]["passed"], by_gas["o2"]["value"], by_gas["o2"]["worst_z_m"]),
                         (False, 24.0, 0.6))
        self.assertEqual((by_gas["lel"]["passed"], by_gas["lel"]["worst_z_m"]), (False, 1.0))
        self.assertTrue(by_gas["h2s"]["passed"])
        self.assertEqual((by_gas["co"]["passed"], by_gas["co"]["worst_z_m"]), (False, 0.2))

    def test_probe_unavailable_falls_back_to_the_body_sensor_and_says_so(self):
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi), gas_fn=lel_at_door_mouth)
        m = self.make(robot)
        result = m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=3.0)
        self.assertEqual(result["method"], "body")
        self.assertIn("probe", result["probe_unavailable"])
        unavailable = robot.of_kind("probe_unavailable")
        self.assertEqual(len(unavailable), 1)
        self.assertIn("body gas sensor", unavailable[0]["message"])
        self.assertEqual([e["data"]["gas"] for e in robot.of_kind("atmos_test")],
                         ["o2", "lel", "h2s", "co"])
        self.assertFalse(result["passed"])
        self.assertTrue(all(e["data"]["method"] == "body" for e in robot.of_kind("atmos_test")))


class TestVentilation(MissionTestBase):
    def test_start_await_and_status(self):
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), gas_fn3=unventilated_tank)
        m = self.make(robot)
        before = m.ventilation_status()
        self.assertFalse(before["on"])
        self.assertFalse(before["purged"])
        self.assertEqual(before["source"], "sim")
        started = m.start_ventilation(tau_s=3.0)
        self.assertTrue(started["ok"])
        self.assertTrue(robot.vent["on"])
        waited = m.await_ventilation(max_s=60.0)
        self.assertTrue(waited["ok"], waited)
        # residual 0.04 + 0.96 e^-t/3 reaches the e^-3 target after ~9 s.
        self.assertGreater(waited["waited_s"], 7.5)
        self.assertLess(waited["waited_s"], 12.0)
        self.assertLessEqual(waited["fraction_remaining"], 0.04 + 0.96 * math.exp(-3.0) + 1e-6)
        kinds = [e["message"] for e in robot.of_kind("ventilation")]
        self.assertTrue(any("SIMULATED" in k for k in kinds))
        self.assertTrue(any("Purge target reached" in k for k in kinds))
        self.assertTrue(any("Ventilating" in k for k in kinds))  # progress events
        self.assertEqual(moving(robot), [])

    def test_await_is_bounded(self):
        robot = FakeRobot(self.clock, gas_fn3=unventilated_tank)
        m = self.make(robot)
        self.assertFalse(m.await_ventilation(max_s=5.0)["ok"])  # never switched on
        m.start_ventilation(tau_s=600.0)
        t0 = self.clock.now()
        result = m.await_ventilation(max_s=2.0)
        self.assertFalse(result["ok"])
        self.assertIn("within", result["error"])
        self.assertLess(self.clock.now() - t0, 4.0)

    def test_clear_and_reset_are_logged_as_simulation_controls(self):
        robot = FakeRobot(self.clock, gas_fn3=unventilated_tank)
        m = self.make(robot)
        robot.inject_gas("h2s", 80.0, 0.0, 0.0, 0.3, 1.0)
        m.start_ventilation(tau_s=3.0)
        self.assertTrue(m.clear_injected_gas()["ok"])
        self.assertEqual(robot.spikes, [])
        self.assertIn("PRESENTER DEMO", robot.of_kind("gas_cleared")[0]["message"])
        self.assertTrue(m.reset_atmosphere()["ok"])
        self.assertFalse(robot.vent["on"])
        self.assertEqual(robot.vent["anchor"], 1.0)
        self.assertIn("SIMULATION CONTROL", robot.of_kind("sim_control")[0]["message"])


class TestStoryline(MissionTestBase):
    def storyline_robot(self, **kwargs):
        kwargs.setdefault("gas_fn3", unventilated_tank)
        return FakeRobot(self.clock, probe_enabled=True, **kwargs)

    def check_storyline_completed(self, robot, summary):
        self.assertEqual(summary["outcome"], COMPLETED, summary)
        self.assertEqual(run_mission.exit_code_for(summary), 0)
        self.assertTrue(summary["storyline"])
        # Fail, pass, then the door re-test just before crossing (T19).
        self.assertEqual([e["data"]["passed"] for e in robot.of_kind("atmos_result")],
                         [False, True, True])
        self.assertEqual(permit_states(robot), ["PRE_ENTRY_TESTING", "ENTRY_AUTHORIZED",
                                                "ENTRY_IN_PROGRESS", COMPLETED])
        kinds = robot.kinds()
        i_denied = kinds.index("entry_denied")
        i_vent = kinds.index("ventilation")
        i_retest = [i for i, k in enumerate(kinds) if k == "atmos_test_start"][1]
        self.assertLess(i_denied, i_vent)
        self.assertLess(i_vent, i_retest)
        self.assertEqual(robot.of_kind("alarm"), [])
        # Nothing entered the tank before the passing re-test authorized it.
        t_auth = next(e["t"] for e in robot.of_kind("permit")
                      if e["data"].get("state") == ENTRY_AUTHORIZED)
        self.assertFalse(any(in_tank(x, y) for t, x, y, _ in robot.trajectory if t < t_auth))
        self.assertEqual(len(summary["atmospheric_tests"]), 3)
        self.assertEqual(len(robot.of_kind("door_check")), 1)
        self.assertTrue(robot.ventilation_state()["on"])  # ventilation stays on during the entry
        self.assertEqual(robot.calls[-1][1], "sit")
        self.assert_collision_free(robot)

    def test_fail_ventilate_retest_pass_enter_complete(self):
        robot = self.storyline_robot()
        m = self.make(robot, require_permit=True, dwell_s=3.0)
        complete_permit(m)
        summary = m.run(storyline=True, vent_tau_s=3.0, vent_max_s=60.0, reset_atmosphere=True)
        self.check_storyline_completed(robot, summary)
        # Two permit_prepared: the permit number and plan, then the attestation (T5).
        self.assertEqual(robot.kinds()[:4], ["permit_prepared", "permit_prepared", "mission_start",
                                             "sim_control"])
        self.assertEqual(m.last_incident()["found"], False)

    def test_storyline_with_the_sim_gas_field(self):
        try:
            import gas_field
        except ImportError as error:
            self.skipTest("sim/gas_field.py not importable: %s" % error)
        if not hasattr(gas_field, "UNVENTILATED_HEADSPACE"):
            self.skipTest("sim/gas_field.py predates the ventilation storyline")
        field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
        robot = self.storyline_robot(gas_fn3=None, field=field)
        m = self.make(robot, require_permit=True, dwell_s=3.0)
        complete_permit(m)
        summary = m.run(storyline=True, vent_tau_s=3.0, vent_max_s=60.0)
        self.check_storyline_completed(robot, summary)

    def test_cli_storyline_excursion_aborts_as_designed_and_explains_the_incident(self):
        robot = self.storyline_robot()
        captured = []

        class Capturing(ConfinedSpaceMission):
            def __init__(inner, *a, **k):
                super().__init__(*a, **k)
                captured.append(inner)

        args = run_mission.build_parser().parse_args(
            ["--storyline", "--vent-tau", "3", "--inject-h2s-at-waypoint", "2",
             "--inject-radius", "1.0", "--log", self.log_path, "--dwell", "3",
             "--rehearsal-auto-confirm"])
        with mock.patch.object(run_mission, "ConfinedSpaceMission", Capturing):
            code = run_mission.execute(args, robot, clock=self.clock)
        self.assertEqual(code, 2)
        records = read_jsonl(self.log_path)
        kinds = [r["kind"] for r in records]
        self.assertEqual(kinds[0], "permit_prepared")
        self.assertIn("demo default values", records[0]["data"]["prepared_by"])
        self.assertLess(kinds.index("demo_inject"), kinds.index("alarm"))
        self.assertIn("retreat_complete", kinds)
        summary = records[-1]["data"]
        self.assertEqual(summary["outcome"], ABORTED)
        self.assertTrue(summary["aborted_as_designed"])
        # Fail, pass, then the door re-test just before crossing (T19).
        self.assertEqual([t["passed"] for t in summary["atmospheric_tests"]], [False, True, True])

        incident = captured[0].last_incident()
        self.assertTrue(incident["found"])
        self.assertEqual(incident["incident_id"], 1)
        self.assertEqual(captured[0].incident_serial, 1)
        self.assertEqual(incident["trigger"]["gases_in_alarm"], ["H2S"])
        self.assertGreaterEqual(incident["trigger"]["reading"]["h2s_ppm"], 10.0)
        self.assertTrue(incident["location"]["inside_permit_space"])
        self.assertEqual(incident["retreat"]["status"], "complete")
        self.assertEqual(incident["permit_after"]["state"], ABORTED)
        self.assertEqual(incident["staged_injection"][0]["kind"], "demo_inject")
        self.assertLess(incident["guard"]["stop_command_s"], 0.5)
        self.assertIsNotNone(incident["time"]["wall_utc"])
        self.assert_collision_free(robot)


class TestSelfRight(MissionTestBase):
    def test_fall_after_stand_self_rights_then_stands_again(self):
        robot = FakeRobot(self.clock, standing=False)
        robot.falls_on_stand = 1
        m = self.make(robot)
        self.assertTrue(m.prepare())
        names = [n for _, n in robot.calls]
        self.assertEqual(names, ["claim", "power_on", "stand", "self_right", "stand"])
        self.assertEqual(len(robot.of_kind("self_right")), 1)
        self.assertIn("appears to have fallen", robot.of_kind("self_right")[0]["message"])
        self.assertEqual(robot.of_kind("robot_fallen"), [])
        self.assertGreater(robot.pose3()[2], 0.3)
        self.assertEqual(moving(robot), [])

    def test_gives_up_after_two_attempts_without_moving(self):
        robot = FakeRobot(self.clock, standing=False)
        robot.falls_on_stand = 10
        m = self.make(robot, dwell_s=3.0)
        summary = m.run()
        self.assertEqual(summary["outcome"], "FAILED")
        self.assertEqual(run_mission.exit_code_for(summary), 1)
        names = [n for _, n in robot.calls]
        self.assertEqual(names.count("self_right"), 2)
        self.assertEqual(names.count("stand"), 3)
        self.assertEqual(len(robot.of_kind("robot_fallen")), 1)
        self.assertEqual(moving(robot), [])


class _Pub:
    """Captures publish() payloads; optional reply hook for the probe."""

    def __init__(self, on_publish=None):
        self.sent = []
        self.on_publish = on_publish

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))
        if self.on_publish is not None:
            self.on_publish(self.sent[-1])


FACADE_NODE = ("/demo_spot", "spot_facade")   # sim/isaac_app.py: one node publishes everything
GRAPH_TOPICS = ("odometry", "payload/status", "payload/gas", "payload/gas/probe_result",
                "payload/site")


class GraphNode:
    """Stub rclpy node for RobotClient's publisher census: ``publishers``
    maps a topic (without the namespace) to (namespace, node) pairs, one
    per publisher, as ``get_publishers_info_by_topic`` reports them. By
    default the facade's one node publishes every facade topic."""

    def __init__(self, **publishers):
        self.publishers = {name: [FACADE_NODE] for name in GRAPH_TOPICS}
        self.publishers.update({name.replace("__", "/"): list(nodes)
                                for name, nodes in publishers.items()})

    def set(self, topic, *nodes):
        self.publishers[topic] = list(nodes)

    def get_publishers_info_by_topic(self, topic):
        name = topic.split("/demo_spot/", 1)[-1]
        return [types.SimpleNamespace(node_namespace=ns, node_name=node)
                for ns, node in self.publishers.get(name, [])]

    def count_publishers(self, topic):
        return len(self.get_publishers_info_by_topic(topic))


def identify_facade(client):
    """The facade's first payload/status, published by the one node that
    also publishes payload/gas and odometry: the backend is the simulator."""
    client._on_status(types.SimpleNamespace(data=json.dumps({"simulated": True})))
    assert client.backend()["simulator"], client.backend()
    return client


def _connected_client():
    client = robot_client.RobotClient()
    client._node = GraphNode()  # "connected" without ROS; the facade alone on the graph
    client._msg = {"String": lambda data: types.SimpleNamespace(data=data)}
    return identify_facade(client)


class TestRobotClientStoryline(unittest.TestCase):
    def test_offline_controls_fail_fast(self):
        client = robot_client.RobotClient()
        self.assertIsNone(client.pose3())
        self.assertIsNone(client.gas_age())
        self.assertFalse(client.set_ventilation(True, 15.0))
        self.assertFalse(client.reset_ventilation())
        self.assertFalse(client.clear_gas())
        start = time.monotonic()
        self.assertIsNone(client.probe(DOOR_PROBE))
        self.assertLess(time.monotonic() - start, 0.1)
        self.assertIn("not connected", client.last_probe_error)
        self.assertEqual(client.recent_presenter_events(), [])

    def test_probe_points_validation(self):
        self.assertEqual(robot_client.probe_points([(5.5, 0, 0.2)]), [[5.5, 0.0, 0.2]])
        for bad in ([], [(1, 2)], [(1, 2, 3.5)], [(1, 2, float("nan"))], [(0, 0, 0.5)] * 17,
                    "abc"):
            with self.assertRaises(ValueError, msg=bad):
                robot_client.probe_points(bad)

    def test_parse_probe_result(self):
        pts = [[5.5, 0.0, z] for z in (0.2, 0.6, 1.0)]
        reply = {"id": "a", "simulated": True, "samples": [
            {"point": p, "o2_pct": 20.0, "lel_pct": 7.0, "h2s_ppm": 51.0 - 20 * i,
             "co_ppm": 0.0, "alarms": {"h2s": i < 2}} for i, p in enumerate(pts)]}
        samples, error = robot_client.parse_probe_result(reply, pts)
        self.assertIsNone(error)
        self.assertEqual([s["z"] for s in samples], [0.2, 0.6, 1.0])
        self.assertEqual(samples[0]["h2s_ppm"], 51.0)
        self.assertTrue(samples[0]["alarms"]["h2s"])
        del reply["samples"][1]["point"]  # falls back to the requested point
        self.assertEqual(robot_client.parse_probe_result(reply, pts)[0][1]["z"], 0.6)
        samples, error = robot_client.parse_probe_result(
            {"id": "a", "error": "point beyond reach", "samples": []}, pts)
        self.assertIsNone(samples)
        self.assertIn("beyond the probe's reach", error)   # a fixed phrase (T6), not the raw text
        self.assertIsNone(robot_client.parse_probe_result({"samples": reply["samples"][:2]}, pts)[0])
        broken = {"samples": [dict(reply["samples"][0], h2s_ppm=None)] + reply["samples"][1:]}
        self.assertIsNone(robot_client.parse_probe_result(broken, pts)[0])

    def test_probe_round_trip_matches_the_request_id(self):
        client = _connected_client()

        def reply(request):
            def later():
                time.sleep(0.02)
                stray = {"id": "someone-else", "samples": []}
                client._on_probe_result(types.SimpleNamespace(data=json.dumps(stray)))
                answer = {"id": request["id"], "sim_time": 1.0, "simulated": True, "samples": [
                    {"point": p, "o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 1.0 + p[2],
                     "co_ppm": 0.0, "alarms": {}} for p in request["points"]]}
                client._on_probe_result(types.SimpleNamespace(data=json.dumps(answer)))
            threading.Thread(target=later, daemon=True).start()

        client._pub_probe = _Pub(reply)
        try:
            samples = client.probe(DOOR_PROBE, timeout=2.0)
            self.assertEqual([s["z"] for s in samples], [0.2, 0.6, 1.0])
            self.assertAlmostEqual(samples[2]["h2s_ppm"], 2.0)
            self.assertIsNone(client.last_probe_error)
            self.assertEqual(client._probe_pending, {})
            request = client._pub_probe.sent[0]
            self.assertEqual(request["points"], [list(map(float, p)) for p in DOOR_PROBE])
            # No reply: bounded wait, then None with a reason.
            client._pub_probe = _Pub()
            start = time.monotonic()
            self.assertIsNone(client.probe(DOOR_PROBE, timeout=0.2))
            self.assertLess(time.monotonic() - start, 1.0)
            self.assertIn("no probe reply", client.last_probe_error)
            self.assertEqual(client._probe_pending, {})
        finally:
            client._node = None

    def test_ventilation_and_clear_payloads(self):
        client = _connected_client()
        client._pub_vent, client._pub_clear = _Pub(), _Pub()
        try:
            self.assertTrue(client.set_ventilation(True, 15))
            self.assertTrue(client.set_ventilation(False))
            self.assertTrue(client.reset_ventilation())
            self.assertFalse(client.set_ventilation(True, 0.01))  # outside the facade range
            self.assertEqual(client._pub_vent.sent, [{"on": True, "tau_s": 15.0}, {"on": False},
                                                     {"reset": True}])
            self.assertTrue(client.clear_gas())
            self.assertEqual(len(client._pub_clear.sent), 1)
        finally:
            client._node = None

    def test_presenter_disclosures_are_remembered(self):
        client = robot_client.RobotClient()
        own = {"t": 1.0, "source": "mission", "kind": "alarm", "message": "x", "data": None}
        staged = {"t": time.time(), "source": "presenter", "kind": "demo_inject",
                  "message": "PRESENTER DEMO: simulated H2S pocket", "data": {"gas": "h2s"}}
        for event in (own, staged):
            client._on_events(types.SimpleNamespace(data=json.dumps(event)))
        # Typed (T6): an allow-listed kind and data, no free-text message.
        self.assertEqual(client.recent_presenter_events(),
                         [{"source": "presenter", "kind": "demo_inject", "data": {"gas": "h2s"},
                           "t": staged["t"]}])
        self.assertEqual(client.recent_presenter_events(max_age_s=-1.0), [])



# ---------------------------------------------------------------------------
# Adversarial review (mission_agent), one regression per bug found
# ---------------------------------------------------------------------------


class TeleportingSelfRight(FakeRobot):
    """self_right as the facade does it (host/reset_spot.sh): the policy
    restarts from its default state, which puts Spot back at spawn."""

    def self_right(self):
        with self.lock:
            self._integrate()
            self.x, self.y, self.yaw = 14.0, 0.0, math.pi
            self.v = self.w = self.u = 0.0
        return super().self_right()


class TestAdversarialReview(MissionTestBase):
    def test_body_sensor_fallback_cannot_authorize_entry(self):
        # Probe broken (old sim, timeout): the body sensor at the door mouth
        # is outside the tank and reads clean air over an unventilated tank.
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi), gas_fn3=unventilated_tank)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=1.5)
        self.assertEqual(result["method"], "body")
        self.assertTrue(all(e["passed"] for e in result["results"]))  # clean air outside...
        self.assertFalse(result["passed"])                              # ...is not a test of the space
        self.assertFalse(result["sampled_inside"])
        self.assertIn("did not sample inside the opening", "; ".join(result["failures"]))
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        self.assertEqual(len(robot.of_kind("entry_denied")), 1)
        self.assertFalse(m.walk_to(*m.door_xy))
        self.assertFalse(any(in_tank(x, y) for _, x, y, _ in robot.trajectory))
        # With the probe working again the same permit is authorized on a
        # vented tank; a later re-test without the probe withdraws it.
        robot.probe_enabled = True
        robot.gas_fn3 = lambda x, y, z: dict(BASELINE)
        self.assertTrue(m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        robot.probe_enabled = False
        self.assertFalse(m.atmospheric_test(at_xy=m.door_mouth_xy, dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ABORTED)

    def test_storyline_without_the_probe_denies_entry_without_ventilating(self):
        robot = FakeRobot(self.clock, gas_fn3=unventilated_tank)  # probe disabled
        m = self.make(robot, require_permit=True, dwell_s=3.0)
        complete_permit(m)
        summary = m.run(storyline=True, vent_tau_s=3.0, vent_max_s=60.0)
        self.assertEqual(summary["outcome"], ABORTED, summary)
        self.assertEqual(summary["abort_cause"], "entry_denied")
        self.assertIn("did not sample inside the opening", summary["reason"])
        self.assertEqual(len(summary["atmospheric_tests"]), 1)
        self.assertEqual(robot.of_kind("ventilation"), [])
        self.assertFalse(any(in_tank(x, y) for _, x, y, _ in robot.trajectory))

    def test_test_away_from_the_door_cannot_authorize_entry(self):
        robot = FakeRobot(self.clock, gas_fn3=unventilated_tank, probe_enabled=True)  # at spawn
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertEqual(result["probe"]["at"], "robot")
        self.assertFalse(result["passed"])
        self.assertIn("not at the door mouth", "; ".join(result["failures"]))
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        self.assertFalse(m.enter())
        self.assertFalse(any(in_tank(x, y) for _, x, y, _ in robot.trajectory))

    def test_cli_default_injection_leaves_spot_sitting_in_clean_air(self):
        robot = FakeRobot(self.clock, probe_enabled=True)
        args = run_mission.build_parser().parse_args(
            ["--inject-h2s-at-waypoint", "0", "--log", self.log_path, "--dwell", "3",
             "--rehearsal-auto-confirm"])
        self.assertEqual(run_mission.execute(args, robot, clock=self.clock), 2)
        self.assertEqual(robot.calls[-1][1], "sit")
        x, y, _ = robot.true_pose()
        self.assertFalse(in_tank(x, y))
        self.assertFalse(robot.gas()["alarm"], (x, y, robot.gas()))

    def test_stand_up_starts_the_guard(self):
        robot = FakeRobot(self.clock, standing=False)
        m = self.make(robot)
        self.assertFalse(m.guard.running)
        self.assertTrue(m.stand_up()["ok"])
        self.assertTrue(m.guard.running)

    def _alarm_inside(self, events, clear_with_mission=False, pockets=True):
        state = {"h2s": 0.0}
        robot = FakeRobot(self.clock, start=(2.5, 1.6, 0.0), gas_fn=lambda x, y: dict(
            BASELINE, h2s_ppm=state["h2s"] if in_tank(x, y) else 0.0))
        if pockets:
            # The simulator reports an injected pocket (far away, no effect on
            # the readings): with a disclosure, the label "staged" needs it (T7).
            robot.spikes.append(("h2s", 1.0, 100.0, 100.0, 0.5))
        robot.recent_presenter_events = lambda max_age_s=300.0: [dict(e) for e in events]
        m = self.make(robot, auto_retreat=False)  # log the alarm, no motion
        if clear_with_mission:
            self.assertTrue(m.clear_injected_gas()["ok"])
        m.start_guard()
        state["h2s"] = 40.0
        deadline = self.clock.now() + 5.0
        while not robot.of_kind("alarm") and self.clock.now() < deadline:
            self.clock.sleep(0.05)
        self.assertTrue(robot.of_kind("alarm"))
        return m.last_incident()

    def test_cleared_presenter_spike_is_not_blamed_for_a_later_alarm(self):
        inject = {"t": 1.0, "source": "presenter", "kind": "demo_inject", "message": "x",
                  "data": {"gas": "h2s", "ppm": 80.0, "x": 2.5, "y": 1.6, "z": 0.3, "radius": 1.0}}
        clear = {"t": 2.0, "source": "presenter", "kind": "demo_clear", "message": "y", "data": {}}
        live = self._alarm_inside([inject])
        self.assertEqual(live["staged_injection"][0]["gas"], "h2s")  # not cleared: disclosed
        # T7: a disclosure without injected pockets in the gas stream is a genuine alarm.
        self.assertIsNone(self._alarm_inside([inject], pockets=False)["staged_injection"])
        self.assertIsNone(self._alarm_inside([inject, clear])["staged_injection"])
        self.assertIsNone(self._alarm_inside([inject], clear_with_mission=True)["staged_injection"])

    def test_self_right_relocation_is_disclosed(self):
        robot = TeleportingSelfRight(self.clock, start=(8.0, 0.0, 0.0), standing=False)
        robot.falls_on_stand = 1
        m = self.make(robot)
        out = m.stand_up()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["self_right_attempts"], 1)
        self.assertAlmostEqual(out["relocated_to"][0], 14.0, delta=0.3)
        self.assertTrue(any("SIMULATION" in e["message"] for e in robot.of_kind("self_right")))


# ---------------------------------------------------------------------------
# Critic round: two-depth probe, purge gate, probe rejections, summary
# ventilation, wall_t (P1-3, P1-6, P2-9)
# ---------------------------------------------------------------------------


def _sim_gas_field():
    try:
        import gas_field
    except ImportError as error:
        raise unittest.SkipTest("sim/gas_field.py not importable: %s" % error)
    if not hasattr(gas_field, "UNVENTILATED_HEADSPACE"):
        raise unittest.SkipTest("sim/gas_field.py predates the ventilation storyline")
    return gas_field


ROUTE_Z = tuple(round(0.1 + 0.1 * i, 2) for i in range(12))   # 0.1 .. 1.2 m


def route_points(m, step=0.1):
    """Every point Spot's body passes through on the planned entry: door
    mouth -> door -> the interior waypoints, sampled every ``step`` m."""
    legs = [m.door_mouth_xy, m.door_xy] + list(m.waypoints)
    pts = []
    for a, b in zip(legs, legs[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        pts.extend((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n))
    pts.append(tuple(legs[-1]))
    return pts


def route_exceedances(field, t, pts):
    """OSHA App. E exceedances anywhere on the route at z 0.1-1.2 m (noiseless)."""
    over = []
    for x, y in pts:
        for z in ROUTE_Z:
            reading = field.sample(x, y, z, t=t, noise=False)
            reasons = alarm_reasons(reading)
            if reasons:
                over.append(((round(x, 2), round(y, 2), z), reasons))
    return over


class TestTwoDepthProbe(MissionTestBase):
    def test_door_test_samples_six_points_and_judges_the_worst(self):
        # A pocket only 1.0 m inside the opening: the 0.5 m station reads
        # clean, so a one-depth probe would have authorized entry.
        def deep_pocket(x, y, z):
            reading = dict(BASELINE)
            if abs(x - 5.0) < 0.05 and abs(y) < 0.05 and z < 0.4:
                reading["h2s_ppm"] = 14.0
            return reading

        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), gas_fn3=deep_pocket,
                          probe_enabled=True, arm_reach_m=1.1)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertFalse(result["passed"])
        self.assertTrue(result["sampled_inside"])
        expected = [(x, y, z) for x in (5.5, 5.0) for y in (0.0,) for z in (0.2, 0.6, 1.0)]
        for _t, points in robot.probes:
            self.assertEqual(len(points), 6)
            for got, want in zip(points, expected):
                for a, b in zip(got, want):
                    self.assertAlmostEqual(a, b, places=6)
        self.assertEqual(robot.probe_rejections, [])   # all six within the 1.1 m arm reach
        h2s = next(e for e in result["results"] if e["gas"] == "h2s")
        self.assertFalse(h2s["passed"])
        self.assertEqual((h2s["worst_inside_m"], h2s["worst_z_m"]), (1.0, 0.2))
        self.assertEqual(h2s["by_point"]["1.00@0.20"], 14.0)
        self.assertEqual(h2s["by_point"]["0.50@0.20"], 0.0)
        self.assertIn("1.0 m inside the opening", h2s["message"])
        info = result["probe"]
        self.assertEqual(info["depths_m"], [0.5, 1.0])
        self.assertEqual(info["points"], 6)
        self.assertEqual(info["points_xy"], [[5.5, 0.0], [5.0, 0.0]])
        self.assertIn("simulated; arm-mounted on the real robot", info["stand_in"])
        start = robot.of_kind("atmos_test_start")[0]["message"]
        self.assertIn("extendable gas probe (simulated)", start)
        self.assertIn("6 points", start)
        probe_event = robot.of_kind("atmos_probe")[0]["data"]
        self.assertEqual(len(probe_event["samples"]), 6)
        self.assertEqual(probe_event["depths_m"], [0.5, 1.0])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)

    def test_test_away_from_the_door_still_samples_three_heights_at_spot(self):
        robot = FakeRobot(self.clock, probe_enabled=True)   # at spawn
        m = self.make(robot)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertEqual(result["probe"]["at"], "robot")
        self.assertEqual(result["probe"]["points"], 3)
        self.assertIsNone(result["probe"]["depths_m"])
        self.assertEqual([r["inside_m"] for r in result["probe"]["rows"]], [None, None, None])


class TestProbeRejection(MissionTestBase):
    def test_out_of_reach_from_the_entry_point_repositions_and_retries_once(self):
        # From entry_xy the 1.0 m station is 1.45 m from the nose: rejected.
        robot = FakeRobot(self.clock, start=(7.0, 0.0, math.pi), probe_enabled=True,
                          arm_reach_m=1.1)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        self.assertTrue(result["sampled_inside"])
        self.assertTrue(result["probe"]["repositioned"])
        self.assertEqual(len(robot.probe_rejections), 1)
        self.assertIn("beyond the 1.1 m probe reach", robot.probe_rejections[0][1])
        kinds = robot.kinds()
        self.assertLess(kinds.index("probe_rejected"), kinds.index("probe_retry"))
        self.assertLess(kinds.index("probe_retry"), kinds.index("atmos_result"))
        self.assertNotIn("probe_unavailable", kinds)
        rejected = robot.of_kind("probe_rejected")[0]
        self.assertIn("repositioning Spot at the door mouth", rejected["message"])
        self.assertIn("beyond", rejected["data"]["error"])
        x, y, yaw = robot.true_pose()
        along = (x - m.door_mouth_xy[0]) * m.door_outward[0] + (y - m.door_mouth_xy[1]) * m.door_outward[1]
        self.assertLessEqual(along, 0.045)            # seated: the 1.0 m station within reach
        self.assertGreater(x - FakeRobot.NOSE_M, m.door_xy[0])   # nose never passed the door plane
        self.assertLess(abs(y), 0.05)
        self.assertLess(abs(mission_mod.wrap_angle(yaw - math.pi)), math.radians(6.0))
        self.assertTrue(all(e["method"] == "probe" for e in result["results"]))
        self.assertFalse(any(in_tank(px, py) for _, px, py, _ in robot.trajectory))
        self.assert_collision_free(robot)

    def test_persistent_rejection_fails_the_test_with_a_clear_event(self):
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), probe_enabled=True)
        robot.force_probe_error = "occluded by tank wall"
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertFalse(result["passed"])
        self.assertFalse(result["sampled_inside"])
        self.assertEqual(len(robot.of_kind("probe_rejected")), 1)       # retried exactly once
        self.assertEqual(len(robot.probe_rejections), 2)
        self.assertEqual(robot.of_kind("probe_retry"), [])
        unavailable = robot.of_kind("probe_unavailable")
        self.assertEqual(len(unavailable), 1)
        self.assertIn("rejected again after repositioning", unavailable[0]["message"])
        text = "; ".join(result["failures"])
        self.assertIn("rejected by the simulator even after repositioning", text)
        self.assertIn("occluded by tank wall", text)
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        self.assertIn("Entry NOT authorized", robot.of_kind("entry_denied")[0]["message"])
        self.assertFalse(m.walk_to(*m.door_xy))

    def test_rejection_away_from_the_door_never_moves_spot_toward_the_tank(self):
        robot = FakeRobot(self.clock, probe_enabled=True)   # at spawn, 7.4 m from the door mouth
        robot.force_probe_error = "robot pose unavailable"
        m = self.make(robot)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertEqual(robot.of_kind("probe_rejected"), [])
        self.assertEqual(moving(robot), [])
        self.assertIn("pose unavailable", result["probe_unavailable"])

    def test_seated_stations_pass_the_facades_probe_geometry(self):
        # Interface check against sim/gas_field.ProbeGeometry (the facade's
        # P1-4 rules) when present: seated at the door mouth (within the
        # mission's seat window) every station passes; from entry_xy the
        # 1.0 m ones are out of reach, which is what the retry handles.
        gas_field = _sim_gas_field()
        geometry_cls = getattr(gas_field, "ProbeGeometry", None)
        if geometry_cls is None:
            self.skipTest("sim/gas_field.py has no ProbeGeometry")
        m = self.make(FakeRobot(self.clock))
        geometry = geometry_cls(tank_center=m.center_xy, tank_radius=m.radius, door_xy=m.door_xy,
                                door_normal_yaw=m.site["door_normal_yaw"])
        points = [(x, y, z) for x, y in m.probe_stations_xy for z in m.PROBE_HEIGHTS_M]
        mx, my = m.door_mouth_xy
        for along in (-0.05, 0.0, 0.04):
            for lateral in (-0.03, 0.03):
                for yaw_err in (-4.0, 4.0):
                    pose = (mx + along, my + lateral, FakeRobot.STAND_Z,
                            math.pi + math.radians(yaw_err))
                    for point in points:
                        self.assertIsNone(geometry.check(pose, point), (pose, point))
        entry = (m.entry_xy[0], m.entry_xy[1], FakeRobot.STAND_Z, math.pi)
        errors = [geometry.check(entry, p) for p in points]
        self.assertTrue(all(e is None for e in errors[:3]))
        self.assertTrue(all(e and "reach" in e for e in errors[3:]))
        _samples, reason = robot_client.parse_probe_result(
            {"id": "a", "error": errors[3], "samples": []}, points)
        self.assertTrue(ConfinedSpaceMission._probe_rejection(reason))

    def test_storyline_with_arm_reach_needs_no_reposition(self):
        gas_field = _sim_gas_field()
        field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
        robot = FakeRobot(self.clock, probe_enabled=True, field=field, arm_reach_m=1.1)
        m = self.make(robot, require_permit=True, dwell_s=3.0)
        complete_permit(m)
        summary = m.run(storyline=True, vent_tau_s=3.0, vent_max_s=60.0)
        self.assertEqual(summary["outcome"], COMPLETED, summary["reason"])
        self.assertEqual(robot.probe_rejections, [])
        self.assertEqual(robot.of_kind("probe_rejected"), [])
        # Fail, pass, then the door re-test just before crossing (T19).
        self.assertEqual([t["passed"] for t in summary["atmospheric_tests"]], [False, True, True])
        auth = next(e for e in robot.of_kind("permit") if e["data"].get("state") == ENTRY_AUTHORIZED)
        self.assertLessEqual(field.fraction_remaining(auth["t"]), m.purge_target + 1e-9)
        self.assertTrue(summary["ventilation"]["on"])
        self.assert_collision_free(robot)

    def test_robot_client_rejection_text_is_recognised(self):
        samples, error = robot_client.parse_probe_result(
            {"id": "a", "error": "occluded by tank wall", "samples": []}, DOOR_PROBE)
        self.assertIsNone(samples)
        self.assertTrue(ConfinedSpaceMission._probe_rejection(error))
        self.assertFalse(ConfinedSpaceMission._probe_rejection("no probe reply within 3.0 s"))


class TestSeatedInProbeEnvelope(MissionTestBase):
    """Presenter dry run 2026-09-27 (GUI, agent REPL, re-test after the
    purge): Spot had stood at (6.583, -0.033), yaw 175 deg for the 45 s
    purge (stance hold: +-0.1 deg), 0.147 m from the seat target (tol
    0.14: the ~3 cm standing drift after the first seat). The re-test's
    seat check therefore walked a 1 cm hop from standstill; the policy's
    stop transient then carried the body 7 cm outward (nose 6.10), the
    facade rejected the 1.0 m points (1.10 m), the re-seat hop chased a
    target 0.2 m away (yaw 175 -> 114 deg), the second probe was rejected
    and entry stayed denied. The door probe worked from where Spot stood
    (1.0 m points 1.04 m from the nose): a Spot already inside the probe
    envelope must not be moved."""

    DRY_RUN_POSE = (6.583, -0.033, math.radians(175.0))

    def test_dry_run_pose_is_seated_without_moving(self):
        robot = FakeRobot(self.clock, start=self.DRY_RUN_POSE, probe_enabled=True,
                          arm_reach_m=1.1, yaw_bias=0.0)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        lead = m.DOOR_MOUTH_OFFSET_M - m.PROBE_SEAT_LEAD_M
        target = (m.door_xy[0] + m.door_outward[0] * lead, m.door_xy[1] + m.door_outward[1] * lead)
        self.assertGreater(math.hypot(self.DRY_RUN_POSE[0] - target[0],
                                      self.DRY_RUN_POSE[1] - target[1]), m.PROBE_SEAT_TOL_M)
        result = m.atmospheric_test(m.door_mouth_xy, dwell_s=1.5)
        self.assertEqual(moving(robot), [])                    # no re-seat hop
        self.assertEqual(robot.probe_rejections, [])
        self.assertTrue(result["sampled_inside"])
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(robot.of_kind("probe_rejected"), [])

    def test_envelope_checks_reach_heading_lateral_and_door_plane(self):
        m = self.make(FakeRobot(self.clock))
        self.assertTrue(m._probe_seated(self.DRY_RUN_POSE))
        self.assertTrue(m._probe_seated((6.6, 0.0, math.pi)))
        self.assertFalse(m._probe_seated(None))
        self.assertFalse(m._probe_seated((6.66, 0.0, math.pi)))         # 1.0 m points 1.11 m away
        self.assertFalse(m._probe_seated((6.62, 0.0, math.pi)))         # 1.07 m: inside reach, not the margin
        self.assertFalse(m._probe_seated((6.58, 0.0, math.radians(165.0))))   # 15 deg off the door axis
        self.assertFalse(m._probe_seated((6.58, 0.2, math.pi)))         # off the door axis
        self.assertFalse(m._probe_seated((6.5, 0.0, math.pi)))          # nose past the door plane
        self.assertFalse(m._probe_seated((7.0, 0.0, math.pi)))          # entry_xy: 1.45 m

    def test_envelope_uses_the_sites_advertised_probe_rules(self):
        robot = FakeRobot(self.clock)
        site = dict(robot.site(), probe={"max_reach_m": 1.3, "body_front_offset_m": 0.55})
        m = ConfinedSpaceMission(robot, site, self.log_path, clock=self.clock, require_permit=False)
        self.missions.append(m)
        self.assertTrue(m._probe_seated((6.75, 0.0, math.pi)))           # 1.2 m, within 1.3 - margin

    def test_rejection_retry_still_reseats_from_inside_the_envelope(self):
        # The envelope only skips the pre-test hop: after a facade rejection
        # the retry re-seats as before (the pose evidently does not work).
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), probe_enabled=True)
        robot.force_probe_error = "occluded by tank wall"
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        m.atmospheric_test(m.door_mouth_xy, dwell_s=1.5)
        self.assertEqual(len(robot.of_kind("probe_rejected")), 1)
        self.assertTrue(moving(robot))                                   # the retry walked
        seats = [e["data"] for e in robot.of_kind("probe_seat")]
        self.assertEqual([(s["decision"], s["force"]) for s in seats],
                         [("in_envelope", False), ("step", True)])

    def test_short_reseat_strafes_and_squares_up_to_the_door(self):
        # Leadership rehearsal 2026-09-28 lr3-rep1: 0.152 m from the seat
        # target at yaw 169 deg; the unicycle hop swung Spot to 94 deg, the
        # probe was rejected and later re-seats ended facing away (entry
        # denied for the session). The step strafes and holds the heading.
        robot = FakeRobot(self.clock, start=(6.57, -0.079, math.radians(169.0)),
                          probe_enabled=True, arm_reach_m=1.1, yaw_bias=0.0)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(m.door_mouth_xy, dwell_s=1.5)
        seat = [e["data"] for e in robot.of_kind("probe_seat")]
        self.assertEqual([s["decision"] for s in seat], ["step"])
        self.assertTrue(seat[0]["ok"])
        self.assertTrue(any(abs(c[2]) > 0.0 for c in moving(robot)))       # used linear.y
        yaws = [math.degrees(abs(wrap_angle(p[3] - math.pi))) for p in robot.trajectory]
        self.assertLess(max(yaws), 12.0)                                      # never swung round
        self.assertLess(yaws[-1], m.SEAT_STEP_YAW_TOL_DEG + 1.0)              # squared up
        self.assertEqual(robot.of_kind("turn_refused"), [])
        self.assertEqual(robot.probe_rejections, [])
        self.assertTrue(result["sampled_inside"])
        self.assertTrue(result["passed"], result["failures"])

    def test_a_longer_hop_in_keeps_facing_the_door(self):
        # A 0.34 m re-seat like the same session's, facing the door: Spot
        # 0.33 m farther out than the seat target (6.44, 0).
        robot = FakeRobot(self.clock, start=(6.77, -0.08, math.radians(172.0)),
                          probe_enabled=True, arm_reach_m=1.1, yaw_bias=0.0)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertTrue(m._seat_at_door_mouth())                        # the pre-test seat
        x, y, yaw = robot.true_pose()
        self.assertEqual([e["data"]["decision"] for e in robot.of_kind("probe_seat")], ["step"])
        self.assertLess(abs(wrap_angle(yaw - math.pi)), math.radians(m.SEAT_STEP_YAW_TOL_DEG + 1.0))
        self.assertTrue(m._probe_seated((x, y, yaw)))                  # probe works from here
        self.assertGreater(x, 6.5)                                      # smallest move: nose outside
        yaws = [math.degrees(abs(wrap_angle(p[3] - math.pi))) for p in robot.trajectory]
        self.assertLess(max(yaws), 10.0)

    def test_every_seat_decision_is_logged(self):
        # Rehearsal audit: skip (in_envelope / at_target) vs hop (walk).
        robot = FakeRobot(self.clock, start=self.DRY_RUN_POSE, probe_enabled=True,
                          arm_reach_m=1.1, yaw_bias=0.0)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        m.atmospheric_test(m.door_mouth_xy, dwell_s=1.5)
        seat = robot.of_kind("probe_seat")
        self.assertEqual(len(seat), 1)
        self.assertEqual(seat[0]["data"]["decision"], "in_envelope")
        self.assertAlmostEqual(seat[0]["data"]["dist_m"], 0.147, delta=0.01)
        self.assertEqual(len(seat[0]["data"]["pose"]), 3)
        far = FakeRobot(self.clock, start=(12.0, 0.0, math.pi), probe_enabled=True)
        m2 = self.make(far, require_permit=True)
        complete_permit(m2)
        m2.atmospheric_test(m2.door_mouth_xy, dwell_s=1.5)
        self.assertEqual([e["data"]["decision"] for e in far.of_kind("probe_seat")], ["walk"])


class TestPurgeGate(MissionTestBase):
    def vented_robot(self, residual=0.04):
        return FakeRobot(self.clock, start=(6.6, 0.0, math.pi), gas_fn3=unventilated_tank,
                         probe_enabled=True, arm_reach_m=1.1, residual=residual)

    def test_retest_after_a_fail_waits_for_the_purge_target(self):
        robot = self.vented_robot()
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertFalse(m.atmospheric_test(dwell_s=1.5)["passed"])      # unventilated: FAIL
        # Door probe clean (H2S 51 x 0.12 = 6 ppm at 0.2 m) but 12 % remains.
        robot.vent.update(on=True, t_on=self.clock.now(), tau_s=1.0e6, anchor=0.12)
        early = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(all(e["passed"] for e in early["results"]))
        self.assertFalse(early["passed"])
        self.assertTrue(early["purge_incomplete"])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        refusal = robot.of_kind("purge_incomplete")
        self.assertEqual(len(refusal), 1)
        message = refusal[0]["message"]
        self.assertTrue(message.startswith("Purge incomplete — wait"), message)
        self.assertIn("an earlier test on this permit failed", message)
        self.assertIn("12.0 %", message)
        self.assertIn("<= 8 %", message)
        self.assertIn("wait for the purge", robot.of_kind("entry_denied")[-1]["message"])
        self.assertFalse(m.walk_to(*m.door_xy))
        # At the target: authorized.
        robot.vent.update(anchor=0.07)
        late = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(late["passed"], late["failures"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)

    def test_purge_hold_event_is_the_single_source_for_hud_and_agent(self):
        # Dry-run critic V3: the HUD showed 7 % while the agent said 8.5 %.
        # The hold is now one structured event whose numbers equal the
        # purge gate's and the test result's.
        robot = self.vented_robot()
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertFalse(m.atmospheric_test(dwell_s=1.5)["passed"])
        robot.vent.update(on=True, t_on=self.clock.now(), tau_s=20.0, anchor=0.0847)
        held = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(held["purge_incomplete"])
        events = robot.of_kind("purge_hold")
        self.assertEqual(len(events), 1)
        hold = events[0]["data"]
        self.assertEqual(hold, held["purge_hold"])
        for key in ("fraction_remaining", "purge_target", "eta_s"):
            self.assertIn(key, hold)
        self.assertEqual(hold["fraction_remaining"], round(held["purge"]["fraction_remaining"], 4))
        self.assertEqual(hold["purge_target"], m.purge_target)
        self.assertEqual(hold["eta_s"], held["purge"]["wait_s"])
        self.assertEqual(hold["fraction_source"], "sim")
        self.assertTrue(hold["ventilation_on"])
        # Rounded once, by the gate: the hold quotes the gate's own value
        # (re-rounding the 4-digit fraction flipped 8.0/8.1 at a 0.05 % edge).
        self.assertEqual(hold["percent_remaining"], held["purge"]["percent_remaining"])
        self.assertEqual(hold["percent_remaining"],
                         round(100.0 * held["purge"]["fraction_remaining"], 1))
        # The event text rounds like the purge_incomplete message (0.1 %).
        pct = "%.1f %%" % hold["percent_remaining"]
        self.assertIn(pct, events[0]["message"])
        self.assertIn(pct, robot.of_kind("purge_incomplete")[0]["message"])
        self.assertIn("<= 8 %", events[0]["message"])
        self.assertTrue(events[0]["message"].startswith("PURGE HOLD"))
        # Published right after the explanation, before the probe rows the
        # HUD marks as held.
        kinds = [e["kind"] for e in robot.events]
        self.assertEqual(kinds.index("purge_hold"), kinds.index("purge_incomplete") + 1)
        self.assertLess(kinds.index("purge_hold"), len(kinds) - 1 - kinds[::-1].index("atmos_probe"))

    def test_no_purge_hold_event_when_a_test_fails_on_gas(self):
        robot = self.vented_robot()
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertFalse(result["passed"])
        self.assertNotIn("purge_hold", result)
        self.assertEqual(robot.of_kind("purge_hold"), [])

    def test_refusal_says_how_long_to_wait_and_when_ventilation_is_off(self):
        robot = self.vented_robot()
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertFalse(m.atmospheric_test(dwell_s=1.5)["passed"])
        robot.vent.update(on=False, t_on=None, anchor=0.15)        # vented, then switched off
        off = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(off["purge_incomplete"])
        self.assertIn("forced ventilation is OFF", off["purge"]["message"])
        robot.vent.update(on=True, t_on=self.clock.now(), tau_s=20.0, anchor=0.15)
        on = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(on["purge_incomplete"])
        # 0.04 + 0.11 e^(-t/20) reaches 0.08 about 20 ln(0.11/0.04) = 20 s after t_on.
        self.assertGreater(on["purge"]["wait_s"], 12.0)
        self.assertLess(on["purge"]["wait_s"], 20.5)
        self.assertIn("keep ventilating about", on["purge"]["message"])

    def test_first_test_in_a_tank_being_ventilated_is_gated_too(self):
        # No earlier FAIL: an agent that vents first and tests at ~17 %
        # remaining must not get an authorization either.
        robot = self.vented_robot()
        robot.vent.update(on=True, t_on=self.clock.now(), tau_s=1.0e6, anchor=0.17)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(all(e["passed"] for e in result["results"]))
        self.assertTrue(result["purge_incomplete"])
        self.assertIn("the tank is being ventilated", result["purge"]["message"])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)

    def test_gas_feed_dropout_does_not_make_a_vented_tank_read_untouched(self):
        # Regression: the host vented the tank to 15 % (not this mission), and
        # payload/gas went stale during the test while the probe still worked.
        # ventilation_status() then fell back to the (never-commanded) local
        # state, so the tank read "untouched" and the door-only pass at 15 %
        # authorized entry although the route still alarms below ~9 %.
        robot = self.vented_robot()
        robot.vent.update(on=True, t_on=self.clock.now(), tau_s=1.0e6, anchor=0.15)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertEqual(m.ventilation_status()["source"], "sim")   # a block is seen once
        robot.telemetry = False                                      # payload/gas stale now
        result = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(all(e["passed"] for e in result["results"]))
        self.assertFalse(result["passed"])
        self.assertTrue(result["purge_incomplete"])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        self.assertIn("last ventilation report", result["purge"]["message"])
        self.assertNotIn("forced ventilation is OFF", result["purge"]["message"])
        # T12 (M12): a last-seen block at the target does not authorize
        # either: without a current simulator feed nothing measures the tank
        # now. It is reported, labelled as the last report.
        robot.telemetry = True
        robot.vent.update(anchor=0.07)
        m.ventilation_status()
        robot.telemetry = False
        stale = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(all(e["passed"] for e in stale["results"]))
        self.assertFalse(stale["passed"])
        self.assertTrue(stale["purge_incomplete"])
        self.assertAlmostEqual(stale["purge"]["last_seen_fraction"], 0.07, places=3)
        self.assertFalse(stale["purge_hold"]["current_feed"])
        self.assertEqual(stale["purge_hold"]["fraction_source"], "last_seen")
        self.assertIn("no current simulator ventilation feed", stale["purge"]["message"])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        # The current feed back at the target: authorized.
        robot.telemetry = True
        self.assertTrue(m.atmospheric_test(dwell_s=1.5)["passed"])
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)

    def test_untouched_clean_tank_and_a_ventilated_start_are_not_gated(self):
        clean = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), probe_enabled=True)
        m = self.make(clean, require_permit=True)
        complete_permit(m)
        self.assertTrue(m.atmospheric_test(dwell_s=1.5)["passed"])        # frac 1.0, never vented
        started_vented = self.vented_robot()                             # --gas-initial ventilated
        started_vented.vent.update(on=True, t_on=self.clock.now(), anchor=0.04)
        m2 = self.make(started_vented, require_permit=True)
        complete_permit(m2)
        self.assertTrue(m2.atmospheric_test(dwell_s=1.5)["passed"])

    def test_unreachable_target_is_reported_not_waited_for(self):
        robot = self.vented_robot(residual=0.1)
        m = self.make(robot)
        m.start_ventilation(tau_s=2.0)
        t0 = self.clock.now()
        waited = m.await_ventilation(max_s=60.0)
        self.assertFalse(waited["ok"])
        self.assertIn("cannot be reached", waited["error"])
        self.assertLess(self.clock.now() - t0, 5.0)

    def test_purge_target_is_configurable_and_validated(self):
        robot = self.vented_robot()
        self.assertEqual(self.make(robot).purge_target, 0.08)
        self.assertEqual(self.make(robot, purge_target=0.05).purge_target, 0.05)
        for bad in (0.0, 1.0, -0.1, float("nan"), "x"):
            with self.assertRaises(ValueError):
                ConfinedSpaceMission(robot, robot.site(), None, clock=self.clock, purge_target=bad)

    def test_cli_purge_target_reaches_the_mission(self):
        captured = []

        class Capturing(ConfinedSpaceMission):
            def __init__(inner, *a, **k):
                super().__init__(*a, **k)
                captured.append(inner)

        self.assertIsNone(run_mission.build_parser().parse_args([]).purge_target)
        args = run_mission.build_parser().parse_args(
            ["--purge-target", "0.05", "--no-loto", "--log", self.log_path])
        robot = FakeRobot(self.clock)
        with mock.patch.object(run_mission, "ConfinedSpaceMission", Capturing):
            run_mission.execute(args, robot, clock=self.clock)   # stops at the permit gate
        self.assertEqual(captured[0].purge_target, 0.05)

    def test_a_local_estimate_is_shown_but_never_authorizes(self):
        """T12 (M12): without a simulator ventilation feed the purge is
        estimated from the commanded time and reported, labelled as an
        estimate, but the estimate never counts as purged and a passing
        door test in a ventilated, earlier-failed tank is refused."""
        class NoVentFeed(FakeRobot):
            def gas(self):
                reading = super().gas()
                if reading is not None:
                    reading.pop("ventilation", None)
                return reading

        robot = NoVentFeed(self.clock, start=(6.6, 0.0, math.pi), gas_fn3=unventilated_tank,
                           probe_enabled=True, arm_reach_m=1.1)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        self.assertFalse(m.atmospheric_test(dwell_s=1.5)["passed"])      # unventilated: FAIL
        self.assertFalse(m.ventilation_status()["purged"])
        m.start_ventilation(tau_s=3.0)
        status = m.ventilation_status()
        self.assertEqual(status["source"], "local")
        self.assertIsNone(status["fraction_remaining"])
        self.assertGreater(status["fraction_estimate"], 0.5)
        waited = m.await_ventilation(max_s=60.0)
        self.assertFalse(waited["ok"], waited)
        self.assertFalse(waited["purged"])
        self.assertTrue(waited["estimate_reached"])
        self.assertIn("ESTIMATE", waited["error"])
        # 0.04 + 0.96 e^(-t/3) <= 0.08 at t = 3 ln 24 = 9.5 s.
        self.assertGreater(waited["waited_s"], 8.5)
        self.assertLess(waited["waited_s"], 12.0)
        self.assertLessEqual(waited["fraction_estimate"], 0.08 + 1e-9)
        vent_events = robot.of_kind("ventilation")
        self.assertFalse(any((e["data"] or {}).get("purged") for e in vent_events))
        self.assertTrue(vent_events[-1]["message"].startswith("Purge NOT confirmed"))
        # The door probe now reads within limits (the fake field did decay),
        # but the estimate does not authorize: shown, labelled, refused.
        test = m.atmospheric_test(dwell_s=1.5)
        self.assertTrue(all(e["passed"] for e in test["results"]), test["failures"])
        self.assertFalse(test["passed"])
        self.assertTrue(test["purge_incomplete"])
        self.assertLessEqual(test["purge"]["fraction_estimate"], 0.08 + 1e-9)
        self.assertIn("ESTIMATE from the commanded ventilation time", test["purge"]["message"])
        self.assertIn("no current simulator ventilation feed", test["purge"]["message"])
        self.assertEqual(test["purge_hold"]["fraction_source"], "estimate")
        self.assertFalse(test["purge_hold"]["current_feed"])
        self.assertEqual(m.permit, PRE_ENTRY_TESTING)
        self.assertFalse(m.walk_to(*m.door_xy))


class TestPurgeGateQuantitative(MissionTestBase):
    """P1-3, quantitatively, with the real sim/gas_field.GasField (seed 7 +
    the facade's unventilated headspace) decaying under forced ventilation:
    entry is never authorized while any point of the planned route at
    z 0.1-1.2 m exceeds an OSHA App. E limit."""

    def field_at_fraction(self, gas_field, fraction, tau=15.0):
        field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
        r = field.residual_fraction
        field.set_ventilation(True, 0.0, tau_s=tau)
        return field, tau * math.log((1.0 - r) / (fraction - r))

    def test_decision_rule_over_the_whole_purge(self):
        gas_field = _sim_gas_field()
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        pts = route_points(m)
        probe = [(x, y, z) for x, y in m.probe_stations_xy for z in m.PROBE_HEIGHTS_M]
        field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
        field.set_ventilation(True, 0.0, tau_s=15.0)
        probe_only_unsafe, authorized = [], []
        for i in range(0, 481):                       # 0-120 s every 0.25 s
            t = 0.25 * i
            probe_ok = all(not alarm_reasons(field.sample(x, y, z, t=t, noise=False))
                           for x, y, z in probe)
            gate_ok = field.fraction_remaining(t) <= m.purge_target + 1e-9
            over = route_exceedances(field, t, pts)
            if probe_ok and gate_ok:
                authorized.append(t)
                self.assertEqual(over, [], "authorized at t=%.2f s with the route over a limit" % t)
            elif probe_ok and over:
                probe_only_unsafe.append((t, field.fraction_remaining(t)))
        self.assertTrue(authorized)                   # not vacuous: entry does become possible
        # The finding reproduced: the 6-point door probe alone passes while
        # floor-level H2S on the route still alarms - the purge gate is needed.
        self.assertTrue(probe_only_unsafe)
        self.assertGreater(max(f for _t, f in probe_only_unsafe), 0.18)

    def test_purge_target_sits_below_the_route_safe_fraction(self):
        gas_field = _sim_gas_field()
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        pts = route_points(m, step=0.2)

        def safe(fraction):
            field, t = self.field_at_fraction(gas_field, fraction)
            return not route_exceedances(field, t, pts)

        lo, hi = 0.041, 0.5                            # bisect the largest route-safe fraction
        self.assertTrue(safe(lo))
        self.assertFalse(safe(hi))
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if safe(mid) else (lo, mid)
        self.assertLess(m.purge_target, lo)            # 0.08 < ~0.092
        self.assertGreater(lo - m.purge_target, 0.005)

    def test_mission_never_authorizes_while_the_route_exceeds(self):
        gas_field = _sim_gas_field()
        field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi), field=field, probe_enabled=True,
                          arm_reach_m=1.1)
        m = self.make(robot, require_permit=True)
        complete_permit(m)
        pts = route_points(m, step=0.2)
        first = m.atmospheric_test(dwell_s=1.5)                  # unventilated tank: FAIL
        self.assertFalse(first["passed"])
        self.assertTrue(route_exceedances(field, self.clock.now(), pts))
        self.assertTrue(m.start_ventilation(tau_s=15.0)["ok"])
        tests = []
        t0 = self.clock.now()
        while m.permit == PRE_ENTRY_TESTING and self.clock.now() - t0 < 150.0:
            t_start = self.clock.now()
            result = m.atmospheric_test(dwell_s=1.5)
            gases_ok = bool(result["results"]) and all(e["passed"] for e in result["results"])
            tests.append((t_start, gases_ok, result["passed"], bool(result.get("purge_incomplete"))))
        self.assertEqual(m.permit, ENTRY_AUTHORIZED)
        t_auth = next(e["t"] for e in robot.of_kind("permit")
                      if e["data"].get("state") == ENTRY_AUTHORIZED)
        self.assertEqual(route_exceedances(field, t_auth, pts), [])
        self.assertLessEqual(field.fraction_remaining(t_auth), m.purge_target + 1e-9)
        # Every earlier test that read within limits at the door was refused
        # by the purge gate, and at least one of them while the route alarmed.
        refused = [t for t in tests[:-1] if t[1]]
        self.assertTrue(refused)
        self.assertTrue(all(t[3] and not t[2] for t in refused))
        self.assertTrue(any(route_exceedances(field, t[0], pts) for t in refused))
        self.assertTrue(robot.of_kind("purge_incomplete"))


class TestSummaryVentilationAndWallTime(MissionTestBase):
    def test_summary_reports_the_simulators_ventilation_not_the_commanded_state(self):
        # P1-6: an excursion run in a tank vented earlier (not by this mission).
        robot = FakeRobot(self.clock)
        robot.set_ventilation(True, 2.0)
        robot.inject_gas("h2s", 30.0, 0.9, 0.6, 0.3, 0.6)   # a release on leg 2 (never decays)
        m = self.make(robot, dwell_s=3.0)
        summary = m.run()
        self.assertEqual(summary["outcome"], ABORTED, summary["reason"])
        self.assertTrue(summary["ventilation"]["on"])
        self.assertEqual(summary["ventilation"]["source"], "sim")
        self.assertLess(summary["ventilation"]["fraction_remaining"], 0.1)
        self.assertFalse(summary["ventilation_commanded"]["on"])
        self.assertFalse(summary["ventilation"]["commanded"]["on"])

    def test_summary_falls_back_to_the_last_block_the_guard_saw(self):
        robot = FakeRobot(self.clock, start=(6.6, 0.0, math.pi))
        robot.set_ventilation(True, 2.0)
        m = self.make(robot)
        m.start_guard()
        self.clock.sleep(0.5)
        robot.telemetry = False
        report = m._vent_report()
        self.assertEqual(report["source"], "sim_last_seen")
        self.assertTrue(report["on"])

    def test_records_carry_sim_time_and_wall_time(self):
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        before = time.time()
        m.log_event("note", "hello")
        m.close()
        record = read_jsonl(self.log_path)[-1]
        self.assertAlmostEqual(record["t"], self.clock.now(), delta=5.0)   # sim (fake clock) time
        self.assertGreaterEqual(record["wall_t"], before - 0.01)
        self.assertLessEqual(record["wall_t"], time.time() + 0.01)
        self.assertTrue(record["wall"].startswith(time.strftime("%Y-%m-%d", time.gmtime(record["wall_t"]))))
        self.assertEqual(robot.events[-1]["wall_t"], record["wall_t"])
        self.assertEqual(robot_client.make_event("k", "m", wall_t=12.5, t=3.0)["wall_t"], 12.5)

    def test_a_bogus_publisher_wall_t_cannot_make_event_logging_raise(self):
        # Regression: fromtimestamp(1e20) raised OverflowError inside
        # _event, the path that also logs guard alarms and retreats.
        robot = FakeRobot(self.clock)
        m = self.make(robot)
        with mock.patch.object(robot, "publish_event",
                               return_value={"t": 1.0, "wall_t": 1.0e20}):
            m.log_event("note", "hello")
        m.close()
        record = read_jsonl(self.log_path)[-1]
        self.assertEqual(record["message"], "hello")
        self.assertLess(abs(record["wall_t"] - time.time()), 60.0)


if __name__ == "__main__":
    unittest.main()
