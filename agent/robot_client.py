"""Thread-safe rclpy wrapper around the Spot facade (shared contract 2).

What this is
------------
``RobotClient`` is the only object the mission (agent/mission.py) and the LLM
agent (agent/spot_agent.py) use to talk to Spot. It speaks the spot_ros2
names under one namespace (default ``/demo_spot``), the names the Isaac Sim
facade publishes (the ``payload/*`` topics are this demo's custom payload
namespace). It commands only a backend identified as the simulator (see
"Backend identity" below): a real robot's ``spot_ros2`` driver is refused,
with no override. A real robot needs its own safety and security review.

    subscribes  odometry (nav_msgs/Odometry), payload/gas, payload/status,
                payload/site (std_msgs/String JSON; site is latched:
                reliable + TRANSIENT_LOCAL, depth 1), payload/gas/probe_result
                (probe replies), payload/events (VOLATILE: only to notice
                presenter disclosures from host/inject_gas.sh)
    publishes   cmd_vel (geometry_msgs/Twist), payload/gas/inject,
                payload/gas/clear, payload/gas/ventilation, payload/gas/probe
                (String JSON; SIMULATED gas controls, see sim/isaac_app.py),
                payload/guard (String JSON, reliable + VOLATILE, depth 10: the
                SafetyGuard heartbeat at 2 Hz, see agent/mission.py),
                payload/events (String JSON; reliable + TRANSIENT_LOCAL
                depth 100 so a late-joining HUD still gets recent history,
                and because a TRANSIENT_LOCAL publisher is compatible with
                both VOLATILE and TRANSIENT_LOCAL subscribers)
    calls       std_srvs/Trigger: claim release power_on power_off stand sit
                stop self_right estop/hard estop/gentle estop/release

How it runs
-----------
rclpy is imported lazily inside ``connect()``, so this module imports (and its
pure helpers unit-test) on a Mac without ROS. Inside the ros:jazzy agent
container::

    robot = RobotClient()              # namespace="/demo_spot"
    if not robot.connect(timeout=30):  # odometry flowing + services up
        ...
    ok, msg = robot.stand()
    robot.send_velocity(0.3, 0.0, 0.0) # must be re-sent at >= 8 Hz
    robot.close()

A private rclpy Context + SingleThreadedExecutor spin in a daemon thread, so
the client never collides with other rclpy users in the same process and all
getters are safe to call from any thread. rclpy's own SIGINT handler is
disabled: on Ctrl-C the caller must still be able to send a zero twist and
call ``stop`` before the context goes away.

Safety behaviour (why these choices)
------------------------------------
* Getters return ``None`` for stale data (pose > 1 s, gas > 2 s, status > 5 s
  by default) instead of an old value: a controller acting on a stale pose is
  driving blind, and a safety guard reading an old gas sample is not
  monitoring. Pass ``max_age=None`` to get the last value regardless.
* ``send_velocity`` clamps to conservative limits, rejects NaN, and honours a
  motion *owner*: while the mission's retreat owns motion
  (``set_motion_owner("retreat")``), commands from anyone else (e.g. an LLM
  tool call) are dropped. A zero twist is always allowed through - stopping
  is never unsafe.
* Every service call has a bounded timeout and returns ``(False, reason)``
  instead of raising or hanging when the robot is unreachable.
* ``probe(points)`` is request/response by id over two topics; it always
  returns within ``timeout`` (None on timeout, rejection or no link), so an
  atmospheric test can fall back to the body sensor instead of hanging.
* Typed, allow-listed fields only (T6): payload/status, payload/gas, probe
  replies and presenter disclosures are reduced to known keys with checked
  types as they arrive (``filter_status``, ``filter_gas``,
  ``filter_presenter_event``); probe errors and Trigger replies become a
  fixed set of phrases (``probe_error_text``, ``trigger_text``). No raw
  string from the robot side reaches the model, the console or the logs;
  ``clean_text`` strips control, escape and invisible format characters
  from the rest. Records the facade publishes on payload/events (source
  "facade") are never taken for presenter disclosures.
* One publisher per safety topic (T21, T22): a 1 Hz census (rclpy's
  ``get_publishers_info_by_topic``) refuses gas readings, probe replies and
  the site message while more than one participant publishes payload/gas,
  payload/gas/probe_result or payload/site, and it is read again as each
  payload/gas sample and probe reply arrives (and when the reply is
  accepted): exactly one publisher, the node that identified the
  simulator, else the data is refused. Probe ids are random
  (``secrets.token_hex``); a reply must match a pending request, and a
  second reply to the same request refuses the probe. Odometry must be
  finite and physically plausible (speed and turn rate between samples);
  after a jump ``pose()`` returns None for ODOM_HOLD_S, so every motion
  loop holds zero velocity. Each odometry sample is numbered and checked
  as it arrives (one publisher, the node that identified the simulator);
  ``odometry_sample()`` returns the latest with that verdict, so the
  mission can declare a retreat complete only on two such samples (T22).
  A received site must match agent/site_defaults.json within
  SITE_TOLERANCE_M, else it is refused and the reviewed defaults are used.
* Backend identity: the simulator is identified by ``payload/status``
  carrying ``"simulated": true`` (``BackendIdentity``), and only when that
  status's one publisher is the node that alone publishes payload/gas and
  odometry and is named as the facade's is (``FACADE_NODE``,
  /demo_spot/spot_facade, sim/isaac_app.py). On any other backend every
  command is refused (motion, probe, power, stand, sit, self-right, e-stop
  release, the simulated gas controls) and the reason is returned. Until
  the simulator has been identified nothing at all is sent, stops included
  (T15): to a real driver a zero twist or a ``stop`` claims the lease and
  can power the robot on. Once identified, stopping (zero twist, ``stop``,
  the e-stops, ``power_off``) is never refused. A later status without the
  flag or a second payload/status publisher is a CONFLICT: still refused as
  above, except the stops, the retreat's own motion, its first stand and
  its sit once outside (owner "retreat" while the retreat holds motion), so
  one forged message cannot strand Spot inside. This sample supports the
  simulator only: there is no override. The flag and node names are
  self-declared; only DDS isolation and SROS2 authenticate them.

Simulated gas controls (sim/isaac_app.py facade, "SIMULATED gas controls")
------------------------------------------------------------------------
    payload/gas/ventilation  {"on": bool, "tau_s": float?} | {"reset": true}
    payload/gas/clear        any payload: removes every injected pocket
    payload/gas/probe        {"id": str, "points": [[x, y, z], ...]} (1..16
                             points, z 0..3 m; the facade's extendable gas
                             probe, simulated; arm-mounted on the real robot:
                             body pose required, <= 1.1 m horizontally from
                             the body's front, and only through the doorway
                             into the tank, else an "error" reply such as
                             "occluded by tank wall")
    payload/gas/probe_result {"id", "samples": [{"point", "o2_pct",
                             "lel_pct", "h2s_ppm", "co_ppm", "alarms"}],
                             "sim_time", "ventilation", "simulated": true}
                             or {"id", "error", "samples": []}
    payload/gas              adds "ventilation" {on, t_on, tau_s,
                             fraction_remaining, residual_fraction} and
                             "injected_pockets"
"""

import collections
import json
import logging
import math
import os
import re
import secrets
import threading
import time
import unicodedata

LOG = logging.getLogger("robot_client")

SITE_DEFAULTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "site_defaults.json")

# Contract 1: consumers fall back to site_defaults.json if the latched site
# message has not arrived within 10 s of connecting.
SITE_WAIT_S = 10.0

TRIGGER_SERVICES = (
    "claim", "release", "power_on", "power_off", "stand", "sit", "stop",
    "self_right", "estop/hard", "estop/gentle", "estop/release",
)

# Per-service timeouts. Stop/e-stop must answer fast or not at all: a caller
# waiting 10 s on a hung stop call is a caller not doing anything else safe.
SERVICE_TIMEOUT_S = {
    "stop": 3.0, "estop/hard": 3.0, "estop/gentle": 3.0, "estop/release": 3.0,
    "claim": 5.0, "release": 5.0, "power_on": 10.0, "power_off": 10.0,
    "stand": 10.0, "sit": 10.0, "self_right": 10.0,
}

EVENT_SOURCES = ("mission", "agent", "guard")

# Conservative command limits for a walking demo in a cluttered space (real
# Spot tops out ~1.6 m/s; nothing in this scenario needs more than 0.5).
MAX_VX = 1.0
MAX_VY = 0.5
MAX_WZ = 1.0

DEFAULT_POSE_MAX_AGE_S = 1.0     # odometry is 50 Hz
DEFAULT_GAS_MAX_AGE_S = 2.0      # payload/gas is 4 Hz
DEFAULT_STATUS_MAX_AGE_S = 5.0   # payload/status is 2 Hz

# Probe request limits, same as the facade (sim/isaac_app.py PROBE_*): a
# request outside them would only come back as an error.
PROBE_MAX_POINTS = 16
PROBE_Z_RANGE_M = (0.0, 3.0)
DEFAULT_PROBE_TIMEOUT_S = 3.0
VENT_TAU_RANGE_S = (2.0, 3600.0)   # facade (sim/isaac_app.py): tau_s below 2 s is refused
PRESENTER_SOURCE = "presenter"   # host/inject_gas.sh disclosure events
GAS_READING_KEYS = ("o2_pct", "lel_pct", "h2s_ppm", "co_ppm")

# Backend identity (interface contract): the Isaac Sim facade's
# payload/status carries "simulated": true. Anything else is not the
# simulator, and nothing here commands it. Until the simulator has been
# identified not even a stop is sent (T15: a zero twist or a stop claims a
# real driver's lease); once it has, stopping is never refused.
SIMULATED_KEY = "simulated"
BACKEND_UNKNOWN = "unknown"
BACKEND_SIMULATOR = "simulator"
BACKEND_NOT_SIMULATOR = "not_simulator"
# Identified as the simulator, then contradicted (a status without the flag,
# or a second payload/status publisher): only stops and the retreat remain.
BACKEND_CONFLICT = "conflict"
STOP_SERVICES = ("stop", "estop/hard", "estop/gentle", "power_off")
STOP_METHODS = ("stop", "estop_hard", "estop_gentle", "power_off")
# In a backend identity conflict these pass only for the retreat (owner
# "retreat" while it holds motion): its first stand, its sit outside (T9).
RETREAT_SERVICES = RETREAT_METHODS = ("stand", "sit")
SIMULATOR_ONLY_TEXT = ("this sample commands only the Isaac Sim simulator; a real robot needs "
                       "its own safety and security review")
CONFLICT_TEXT = ("forward motion, entry and every other command are held; only stopping and the "
                 "retreat out of the permit space still work")
BACKEND_WAIT_S = 10.0
# The facade's one node publishes all three (sim/isaac_app.py "spot_facade"):
# the identifying payload/status must come from the node that alone publishes
# the gas stream and the odometry the agent relies on, and that node must be
# the facade's by name and namespace (T9: rclpy.create_node("spot_facade",
# namespace=ROS_NAMESPACE) with ROS_NAMESPACE "/demo_spot").
IDENTITY_TOPICS = ("payload/status", "payload/gas", "odometry")
FACADE_NODE_NAME = "spot_facade"
FACADE_NAMESPACE = "/demo_spot"
FACADE_NODE = "%s/%s" % (FACADE_NAMESPACE, FACADE_NODE_NAME)   # as _publishers() names it
UNKNOWN_NODE_MARK = "_NODE_NAME_UNKNOWN_"   # rmw: endpoint seen, node info not yet

# Allow-lists for robot-side data (T6). Anything else is dropped.
STATUS_BOOL_KEYS = ("claimed", "powered", "standing", "standing_up", "estopped", SIMULATED_KEY)
STATUS_NUM_KEYS = ("battery_pct", "sim_time", "rtf")
STATUS_INT_KEYS = ("cmd_vel_publishers",)
ALARM_KEYS = ("o2", "lel", "h2s", "co", "any")
VENTILATION_KEYS = ("t_on", "tau_s", "fraction_remaining", "residual_fraction")
PRESENTER_KINDS = ("demo_inject", "demo_clear", "demo_reset")
PRESENTER_GASES = ("h2s", "lel", "o2", "co")
PRESENTER_NUM_KEYS = ("ppm", "x", "y", "z", "radius")
TEXT_MAX = 2000

# One publisher per safety topic (T21, T22) and plausible odometry (T22).
SINGLE_PUBLISHER_TOPICS = ("payload/gas", "payload/gas/probe_result", "payload/site")
CENSUS_TOPICS = SINGLE_PUBLISHER_TOPICS + ("payload/status", "odometry")
CENSUS_PERIOD_S = 1.0
PROBE_DUPLICATE_WINDOW_S = 0.15  # after the first reply: time for a competing one to show
ODOM_MAX_SPEED_MPS = 3.0         # real Spot tops out near 1.6 m/s
ODOM_MAX_YAW_RATE = 4.0          # rad/s
ODOM_MIN_DT_S = 0.02             # 50 Hz odometry; a burst of queued samples is not a jump
ODOM_HOLD_S = 2.0                # no pose for this long after an implausible sample
SITE_TOLERANCE_M = 0.05
SITE_YAW_TOLERANCE = 0.01        # rad
SITE_XY_KEYS = ("spawn_xy", "entry_xy", "door_xy", "tank_center_xy")
SITE_YAW_KEYS = ("spawn_yaw", "door_normal_yaw")
# Terminal control sequences (CSI, OSC, other ESC sequences, 8-bit CSI).
_ESCAPE_SEQUENCE = re.compile(
    r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)?|\x1b[@-Z\\-_]|\x9b[0-?]*[ -/]*[@-~]")


# ---------------------------------------------------------------------------
# Pure helpers (no ROS; unit-tested on the Mac)
# ---------------------------------------------------------------------------


def clean_text(value, limit=TEXT_MAX, newlines=False):
    """Printable copy of ``value`` for the console, the logs and the model:
    terminal escape sequences, C0/C1 controls, DEL and invisible Unicode
    format characters (bidi overrides, zero-width, tag characters) are
    removed; other whitespace becomes a space (newlines kept only with
    ``newlines``). Truncated to ``limit`` characters (None: no limit)."""
    text = _ESCAPE_SEQUENCE.sub("", str(value))
    out = []
    for ch in text:
        if ch == "\n" and newlines:
            out.append(ch)
            continue
        category = unicodedata.category(ch)
        if category[0] == "Z" or ch in "\t\r\n\x0b\x0c":
            out.append(" ")
        elif category[0] != "C":       # Cc controls, Cf format, Cs, Co, Cn
            out.append(ch)
    text = "".join(out)
    return text if limit is None else text[:limit]


def _bool_or_none(value):
    return value if isinstance(value, bool) else None


def _num_or_none(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def filter_status(raw):
    """payload/status -> typed dict of the allow-listed keys only, or None."""
    if not isinstance(raw, dict):
        return None
    out = {}
    for key in STATUS_BOOL_KEYS:
        if isinstance(raw.get(key), bool):
            out[key] = raw[key]
    for key in STATUS_NUM_KEYS:
        value = _num_or_none(raw.get(key))
        if value is not None:
            out[key] = value
        elif key in raw and raw[key] is None:
            out[key] = None            # e.g. rtf before the facade's first window
    for key in STATUS_INT_KEYS:
        value = raw.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
            out[key] = value
    return out


def filter_alarms(raw):
    """The facade's per-gas alarm flags: known keys, real booleans only."""
    if not isinstance(raw, dict):
        return {}
    return {key: raw[key] for key in ALARM_KEYS if isinstance(raw.get(key), bool)}


def filter_ventilation(raw):
    if not isinstance(raw, dict):
        return None
    out = {"on": raw.get("on") is True}
    for key in VENTILATION_KEYS:
        out[key] = _num_or_none(raw.get(key))
    return out


def filter_gas(raw):
    """payload/gas -> typed dict (the four readings, alarm flags,
    ventilation block, injected_pockets, times), or None. A missing or
    non-numeric reading is left out, which the SafetyGuard treats as an
    alarm (fail safe)."""
    if not isinstance(raw, dict):
        return None
    out = {}
    for key in GAS_READING_KEYS:
        value = _num_or_none(raw.get(key))
        if value is not None:
            out[key] = value
    if "alarms" in raw:
        out["alarms"] = filter_alarms(raw.get("alarms"))
    if isinstance(raw.get("alarm"), bool):
        out["alarm"] = raw["alarm"]
    vent = filter_ventilation(raw.get("ventilation"))
    if vent is not None:
        out["ventilation"] = vent
    pockets = raw.get("injected_pockets")
    if isinstance(pockets, int) and not isinstance(pockets, bool) and pockets >= 0:
        out["injected_pockets"] = pockets
    for key in ("sim_time", "wall_t", "t"):
        value = _num_or_none(raw.get(key))
        if value is not None:
            out[key] = value
    if isinstance(raw.get(SIMULATED_KEY), bool):
        out[SIMULATED_KEY] = raw[SIMULATED_KEY]
    return out


def filter_presenter_event(event):
    """A presenter disclosure (host/inject_gas.sh) reduced to an allow-listed
    kind and typed data, or None for anything else (other sources such as
    the facade's own "facade" records, unknown kinds, malformed data)."""
    if not isinstance(event, dict) or event.get("source") != PRESENTER_SOURCE:
        return None
    kind = event.get("kind")
    if kind not in PRESENTER_KINDS:
        return None
    raw = event.get("data") if isinstance(event.get("data"), dict) else {}
    data = {}
    gas = raw.get("gas")
    if isinstance(gas, str) and gas.lower() in PRESENTER_GASES:
        data["gas"] = gas.lower()
    for key in PRESENTER_NUM_KEYS:
        value = _num_or_none(raw.get(key))
        if value is not None:
            data[key] = value
    out = {"source": PRESENTER_SOURCE, "kind": kind, "data": data}
    for key in ("t", "wall_t"):
        value = _num_or_none(event.get(key))
        if value is not None:
            out[key] = value
    return out


def probe_error_text(raw):
    """The facade's probe error -> one of a fixed set of phrases."""
    text = str(raw or "").lower()
    if "occluded" in text:
        return "occluded by tank wall"
    if "reach" in text:
        return "point beyond the probe's reach from the body's front"
    if "pose" in text:
        return "robot pose unavailable"
    if "outside" in text and " z" in " " + text:
        return "probe point height outside the allowed range"
    if "finite" in text:
        return "non-finite probe request"
    if any(word in text for word in ("id must", "points must", "each point", "json")):
        return "malformed probe request"
    return "probe request refused"


class ClientRefusal(str):
    """A refusal generated by this code (e.g. ``backend_refusal``), never by
    the robot. Marked by type, not by its wording, so a robot reply that
    imitates the wording is still mapped to a fixed phrase (T6)."""


def trigger_text(service, ok, raw):
    """A Trigger reply -> a fixed phrase (the robot's own text never
    reaches the model). Only this client's own refusals (``ClientRefusal``)
    pass through."""
    if ok:
        return "%s accepted" % service
    if isinstance(raw, ClientRefusal):
        return raw
    text = str(raw or "")
    low = text.lower()
    if "e-stop" in low or "estop" in low:
        reason = "the e-stop is asserted"
    elif "timeout" in low or "timed out" in low:
        reason = "no reply in time"
    elif "unavailable" in low:
        reason = "the service is unavailable"
    elif "not connected" in low:
        reason = "not connected to the robot"
    elif "power" in low:
        reason = "motor power is off"
    elif "lease" in low or "claim" in low:
        reason = "the robot is not claimed"
    else:
        reason = "refused by the robot"
    return "%s refused: %s" % (service, reason)


def _pair(value):
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def site_mismatch(site, reference, tol_m=SITE_TOLERANCE_M, tol_yaw=SITE_YAW_TOLERANCE):
    """Why ``site`` differs from the reviewed ``reference`` geometry
    (agent/site_defaults.json) by more than the tolerances, or None. The
    keys compared are the route and permit-space geometry; anything else
    (e.g. the facade's informational "probe" block) is not compared."""
    if not isinstance(site, dict):
        return "the site is not a JSON object"
    door = site.get("door_xy", site.get("doorway_xy"))
    for key in SITE_XY_KEYS:
        got = _pair(door if key == "door_xy" else site.get(key))
        want = _pair(reference.get(key))
        if got is None or want is None or math.hypot(got[0] - want[0], got[1] - want[1]) > tol_m:
            return "%s %r differs from the reviewed %r" % (key, site.get(key), reference.get(key))
    for key in SITE_YAW_KEYS:
        got, want = _num_or_none(site.get(key)), _num_or_none(reference.get(key))
        if got is None or want is None or abs(math.atan2(math.sin(got - want),
                                                         math.cos(got - want))) > tol_yaw:
            return "%s %r differs from the reviewed %r" % (key, site.get(key), reference.get(key))
    radius, want = _num_or_none(site.get("tank_radius")), _num_or_none(reference.get("tank_radius"))
    if radius is None or want is None or abs(radius - want) > tol_m:
        return "tank_radius %r differs from the reviewed %r" % (site.get("tank_radius"), want)
    got, want = site.get("interior_waypoints"), reference.get("interior_waypoints")
    if not isinstance(got, (list, tuple)) or len(got) != len(want or []):
        return "interior_waypoints differ from the reviewed route"
    for a, b in zip(got, want):
        a, b = _pair(a), _pair(b)
        if a is None or b is None or math.hypot(a[0] - b[0], a[1] - b[1]) > tol_m:
            return "interior_waypoints differ from the reviewed route"
    return None


def load_site_defaults(path=SITE_DEFAULTS_PATH):
    """Return the site dict from agent/site_defaults.json (a fresh copy)."""
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def yaw_from_quaternion(w, x, y, z):
    """Yaw (rad) of a WXYZ quaternion, same formula as sim/isaac_app.py."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _finite_or_zero(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) else 0.0


def clamp_twist(vx, vy, wz):
    """Clamp a body-frame twist to the demo limits; NaN/inf/garbage -> 0."""
    vx, vy, wz = _finite_or_zero(vx), _finite_or_zero(vy), _finite_or_zero(wz)
    return (max(-MAX_VX, min(MAX_VX, vx)),
            max(-MAX_VY, min(MAX_VY, vy)),
            max(-MAX_WZ, min(MAX_WZ, wz)))


def sanitize_json(obj):
    """Make ``obj`` strict-JSON safe: tuples -> lists, NaN/inf -> None.

    The HUD parses events in a browser; Python's json happily emits NaN,
    which JSON.parse rejects, so strip it at the source.
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): sanitize_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize_json(v) for v in obj]
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    return str(obj)


def probe_points(points):
    """Validate probe points -> list of [x, y, z] floats, or raise ValueError.

    Same limits as the facade (1..16 points, finite, z within 0..3 m), so
    a bad request fails here, at once, instead of costing a round trip.
    """
    try:
        pts = [[float(v) for v in p] for p in points]
    except (TypeError, ValueError):
        raise ValueError("probe points must be [[x, y, z], ...]") from None
    if not 1 <= len(pts) <= PROBE_MAX_POINTS:
        raise ValueError("probe needs 1..%d points, got %d" % (PROBE_MAX_POINTS, len(pts)))
    for p in pts:
        if len(p) != 3 or not all(math.isfinite(v) for v in p):
            raise ValueError("each probe point must be 3 finite numbers, got %r" % (p,))
        if not PROBE_Z_RANGE_M[0] <= p[2] <= PROBE_Z_RANGE_M[1]:
            raise ValueError("probe z %.2f outside %s m" % (p[2], PROBE_Z_RANGE_M))
    return pts


def parse_probe_result(result, points):
    """payload/gas/probe_result dict -> (samples, error).

    ``samples`` is a list (request order) of dicts with ``x``, ``y``, ``z``,
    the four gas keys (floats) and ``alarms`` (dict, possibly empty); it is
    None when the reply is an error, empty, incomplete or malformed, and
    ``error`` then says why. A sample without its own ``point`` takes the
    requested point at the same index.
    """
    if not isinstance(result, dict):
        return None, "malformed probe reply"
    if result.get("error"):
        # agent/mission.py keys its reposition-and-retry on this wording.
        return None, "probe rejected by the simulator: %s" % probe_error_text(result["error"])
    raw = result.get("samples")
    if not isinstance(raw, list) or len(raw) != len(points):
        return None, "probe reply has %s samples for %d points" % (
            len(raw) if isinstance(raw, list) else "no", len(points))
    samples = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            return None, "malformed probe sample %d" % i
        point = item.get("point")
        if not (isinstance(point, (list, tuple)) and len(point) == 3):
            point = points[i]
        sample = {}
        try:
            sample["x"], sample["y"], sample["z"] = (float(v) for v in point)
            for key in GAS_READING_KEYS:
                value = float(item[key])
                if not math.isfinite(value):
                    raise ValueError(key)
                sample[key] = value
        except (KeyError, TypeError, ValueError):
            return None, "probe sample %d lacks a finite gas reading" % i
        sample["alarms"] = filter_alarms(item.get("alarms"))
        samples.append(sample)
    return samples, None


class BackendIdentity:
    """Which robot backend answers on payload/status (thread-safe, sticky).

    Before identification every command is refused, stops included (fail
    closed, T15). A payload/status carrying ``"simulated": true``
    identifies the simulator; with ``require_origin`` (RobotClient) only
    when ``origin`` says that its one publisher is the node that alone
    publishes payload/gas and odometry (an identification that fails this
    is not latched: a later status may still identify). A status without
    the flag before identification marks the backend NOT the simulator for
    the rest of the process.

    Once identified the latch is positive: a later status without the flag,
    or a second payload/status publisher (``contradict``), does not silently
    turn the backend into "not the simulator" - that would refuse the
    retreat and strand Spot inside on one forged message. It is a CONFLICT
    for the rest of the process: forward commands are refused as before,
    stopping and the retreat (``backend_refusal(..., retreat=True)``) still
    work. A missing or stale status changes nothing.

    The flag is self-declared on an unauthenticated topic; the publisher
    checks narrow who can make the claim, they do not authenticate it.
    """

    def __init__(self, require_origin=False):
        self._lock = threading.Lock()
        self._require_origin = bool(require_origin)
        self._state = BACKEND_UNKNOWN
        self._reason = "no payload/status has identified the robot backend yet"
        self._node = None      # the identifying publisher (never in snapshot(): graph text, T6)

    def observe(self, status, origin=None):
        """Update from one payload/status dict (anything else is ignored).
        ``origin`` ({"ok", "node", "why"}) says who publishes it. Returns
        ``snapshot()``."""
        if isinstance(status, dict):
            with self._lock:
                flagged = status.get(SIMULATED_KEY) is True
                if self._state == BACKEND_UNKNOWN:
                    if not flagged:
                        self._state = BACKEND_NOT_SIMULATOR
                        self._reason = ('a payload/status message without "simulated": true was '
                                        "received, so the robot backend is not the simulator")
                    elif not self._require_origin:
                        self._state = BACKEND_SIMULATOR
                        self._reason = ('payload/status carries "simulated": true '
                                        "(the Isaac Sim facade)")
                    elif isinstance(origin, dict) and origin.get("ok") and origin.get("node"):
                        self._state, self._node = BACKEND_SIMULATOR, origin["node"]
                        self._reason = ('payload/status carries "simulated": true from the one '
                                        "node that also alone publishes payload/gas and odometry "
                                        "(the Isaac Sim facade)")
                    else:
                        why = origin.get("why") if isinstance(origin, dict) else None
                        self._reason = ('a payload/status says "simulated": true, but %s, so it '
                                        "does not identify the simulator"
                                        % (why or "its publisher could not be checked"))
                elif self._state == BACKEND_SIMULATOR:
                    if not flagged:
                        self._conflict_locked('a later payload/status message without '
                                              '"simulated": true was received')
                    elif isinstance(origin, dict) and origin.get("ok") is False:
                        self._conflict_locked(origin.get("why") or "its publisher changed")
        return self.snapshot()

    def contradict(self, why):
        """A census contradicts an identified simulator (e.g. a second
        payload/status publisher): CONFLICT. No effect in any other state."""
        with self._lock:
            if self._state == BACKEND_SIMULATOR:
                self._conflict_locked(why)
        return self.snapshot()

    def _conflict_locked(self, why):
        self._state = BACKEND_CONFLICT
        self._reason = ("backend identity conflict: the robot was identified as the simulator, "
                        "then %s" % why)

    def node(self):
        """The publisher that identified the simulator (a ``namespace/node``
        name), or None. Kept out of ``snapshot()``: it is graph text."""
        with self._lock:
            return self._node if self._state in (BACKEND_SIMULATOR, BACKEND_CONFLICT) else None

    def snapshot(self):
        """{"backend": unknown|simulator|not_simulator|conflict, "simulator",
        "conflict", "reason"}. "simulator" is True only while commands are
        allowed."""
        with self._lock:
            return {"backend": self._state, "simulator": self._state == BACKEND_SIMULATOR,
                    "conflict": self._state == BACKEND_CONFLICT, "reason": self._reason}


def backend_refusal(identity, action, retreat=False, stop=False):
    """None when ``identity`` (a ``BackendIdentity.snapshot()``) is the
    simulator, or when ``retreat`` or ``stop`` and the identity is in
    conflict (a backend identified as the simulator whose identity is now in
    doubt still lets Spot be stopped and brought out); otherwise why
    ``action`` is refused. A stop (``stop``: a zero twist, a stop or e-stop
    service, power off) is refused too while the backend has never been
    identified as the simulator (T15): on a real driver it claims the lease
    and can power the robot on, so nothing at all is sent to it."""
    if isinstance(identity, dict):
        if identity.get("simulator") is True:
            return None
        if identity.get("backend") == BACKEND_CONFLICT:
            if retreat or stop:
                return None
            return ClientRefusal("Not %s: %s; %s." % (action, identity.get("reason"),
                                                      CONFLICT_TEXT))
    reason = identity.get("reason") if isinstance(identity, dict) else None
    return ClientRefusal("Not %s: %s; %s." % (action, reason or "the robot backend is not identified",
                                              SIMULATOR_ONLY_TEXT))


# The operator's login and SSM session id (agent/operator_console.py
# operator_identity, always under this key) are kept in the mission JSONL
# only, never published on payload/events (DDS, the HUD).
OPERATOR_KEY = "operator"


def without_operator_identity(data):
    """A copy of ``data`` for payload/events and for the model (tool
    results, T37, T38): every ``"operator"`` identity (a dict, or None
    before an answer) removed at any depth. Other values, such as the cause
    string "operator", are kept."""
    if isinstance(data, dict):
        return {key: without_operator_identity(value) for key, value in data.items()
                if not (key == OPERATOR_KEY and (value is None or isinstance(value, dict)))}
    if isinstance(data, (list, tuple)):
        return [without_operator_identity(value) for value in data]
    return data


def make_event(kind, message, data=None, source="mission", t=None, wall_t=None):
    """Build a payload/events record: {"t","wall_t","source","kind","message","data"}.

    ``t`` is the caller's timestamp (RobotClient passes sim time when
    payload/status carries it) and falls back to wall time; ``wall_t`` is
    always the unix wall time (s) at which the event was built, so an audit
    consumer can order sim-time events against host/presenter events.
    ``data`` loses the operator's identity (``without_operator_identity``).
    """
    if source not in EVENT_SOURCES:
        raise ValueError("source must be one of %s, got %r" % (EVENT_SOURCES, source))
    wall = time.time() if wall_t is None else float(wall_t)
    return {
        "t": float(wall if t is None else t),
        "wall_t": round(wall, 3),
        "source": source,
        "kind": str(kind),
        "message": str(message),
        "data": sanitize_json(without_operator_identity(data)) if data is not None else None,
    }


# ---------------------------------------------------------------------------
# RobotClient
# ---------------------------------------------------------------------------


class RobotClient:
    """rclpy client for the /demo_spot facade (see module docstring)."""

    def __init__(self, namespace="/demo_spot", node_name=None):
        ns = "/" + str(namespace).strip("/")
        self.namespace = ns if ns != "/" else ""
        self._node_name = node_name or "spot_mission_client_%d" % os.getpid()

        self._lock = threading.Lock()        # latest-message state
        self._pub_lock = threading.Lock()    # publishers used from many threads
        self._conn_lock = threading.Lock()   # connect()/close()
        self._stop_spin = threading.Event()

        self._ctx = None
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._clients = {}
        self._pub_cmd = None
        self._pub_inject = None
        self._pub_clear = None
        self._pub_vent = None
        self._pub_probe = None
        self._pub_events = None
        self._pub_guard = None
        self._msg = {}                       # lazily imported ROS msg classes
        self._connect_t = None               # monotonic time connect() started

        self._pose = None                    # (x, y, yaw)
        self._pose_z = None                  # body height (odometry position.z)
        self._pose_rx = None
        self._gas = None
        self._gas_rx = None
        self._gas_refusal = None             # why the last payload/gas sample was dropped
        self._status = None
        self._status_rx = None
        self._site = None
        self._site_warned = False
        self._bad_json_warned = set()

        self._probe_pending = {}             # id -> {"event", "result"}
        self.last_probe_error = None         # why the last probe() returned None
        self._presenter_events = collections.deque(maxlen=20)  # (rx monotonic, event)
        self._pub_counts = {}                # topic -> publishers seen by the census
        self._pub_nodes = {}                 # topic -> their namespace/node names (graph text)
        self._multi_warned = set()
        self._odom_prev = None               # (x, y, yaw, rx) of the previous sample
        self._odom_seq = 0                   # odometry samples stored so far
        self._odom_refusal = "no odometry sample yet"  # receipt check of the latest (T22)
        self._odom_bad_until = 0.0           # monotonic: pose() is None until then
        self.odom_violations = 0
        self.site_refusal = None             # why a received site message was refused

        self._motion_owner = None
        # From payload/status and the publisher census (see module docstring).
        self._backend = BackendIdentity(require_origin=True)

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #

    @property
    def connected(self):
        return self._node is not None and not self._stop_spin.is_set()

    def _topic(self, name):
        return "%s/%s" % (self.namespace, name)

    def connect(self, timeout=30.0):
        """Create the node and wait until odometry flows and ``claim`` is up.

        Returns True when both are seen within ``timeout`` seconds, False
        otherwise (including when rclpy is not installed). Safe to call again
        to keep waiting; the node is created only once.
        """
        with self._conn_lock:
            if self._node is None and not self._create_node():
                return False
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                have_odom = self._pose_rx is not None
            claim_cli = self._clients.get("claim")
            services_up = claim_cli is not None and claim_cli.service_is_ready()
            if have_odom and services_up:
                LOG.info("connected to %s (odometry + services up)", self.namespace)
                return True
            if time.monotonic() >= deadline:
                LOG.error("connect: timed out after %.1fs (odometry=%s, services=%s) "
                          "- is sim/isaac_app.py running and ROS_DOMAIN_ID matching?",
                          timeout, have_odom, services_up)
                return False
            time.sleep(0.1)  # nosemgrep: arbitrary-sleep

    def _create_node(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
        except ImportError as error:
            LOG.error("connect: ROS 2 Python packages unavailable (%s); run inside "
                      "the ros:jazzy agent container", error)
            return False

        self._msg = {"Twist": Twist, "String": String, "Trigger": Trigger}
        ctx = Context()
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(context=ctx, signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init(context=ctx)
        node = rclpy.create_node(self._node_name, context=ctx)

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        events_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Heartbeat (HUD contract): reliable, VOLATILE - a late joiner must not
        # see an old "monitoring" beat - depth 10.
        guard_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)

        node.create_subscription(Odometry, self._topic("odometry"), self._on_odom, 10)
        node.create_subscription(String, self._topic("payload/gas"), self._on_gas, 10)
        node.create_subscription(String, self._topic("payload/status"), self._on_status, 10)
        node.create_subscription(String, self._topic("payload/site"), self._on_site, latched)
        node.create_subscription(String, self._topic("payload/gas/probe_result"),
                                 self._on_probe_result, 10)
        # VOLATILE on purpose: only live presenter disclosures, never the
        # TRANSIENT_LOCAL history of an earlier run.
        node.create_subscription(String, self._topic("payload/events"), self._on_events, 50)
        self._pub_cmd = node.create_publisher(Twist, self._topic("cmd_vel"), 10)
        self._pub_inject = node.create_publisher(String, self._topic("payload/gas/inject"), 10)
        self._pub_clear = node.create_publisher(String, self._topic("payload/gas/clear"), 10)
        self._pub_vent = node.create_publisher(String, self._topic("payload/gas/ventilation"), 10)
        self._pub_probe = node.create_publisher(String, self._topic("payload/gas/probe"), 10)
        self._pub_events = node.create_publisher(String, self._topic("payload/events"), events_qos)
        self._pub_guard = node.create_publisher(String, self._topic("payload/guard"), guard_qos)
        self._clients = {name: node.create_client(Trigger, self._topic(name))
                         for name in TRIGGER_SERVICES}

        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        self._ctx, self._node, self._executor = ctx, node, executor
        self._census()
        node.create_timer(CENSUS_PERIOD_S, self._census)   # runs on the executor thread
        self._stop_spin.clear()
        self._connect_t = time.monotonic()
        self._spin_thread = threading.Thread(
            target=self._spin, name="robot_client_spin", daemon=True)
        self._spin_thread.start()
        return True

    def _spin(self):
        while not self._stop_spin.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as error:  # context shut down or transient rcl error
                if self._ctx is None or not self._ctx.ok():
                    break
                LOG.warning("executor spin error: %s", error)
                time.sleep(0.1)  # nosemgrep: arbitrary-sleep

    def close(self):
        """Send a final zero twist, stop spinning, and tear down the node."""
        with self._conn_lock:
            if self._node is None:
                return
            try:
                self._publish_twist(0.0, 0.0, 0.0)
            except Exception:
                pass
            self._stop_spin.set()
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=2.0)
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            try:
                self._node.destroy_node()
            except Exception:
                pass
            try:
                self._ctx.try_shutdown()
            except Exception:
                pass
            self._node = self._executor = self._ctx = None
            self._clients = {}

    # ------------------------------------------------------------------ #
    # Subscription callbacks (executor thread)
    # ------------------------------------------------------------------ #

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (float(p.x), float(p.y), float(p.z), float(q.w), float(q.x), float(q.y),
                  float(q.z))
        now = time.monotonic()
        if not all(math.isfinite(v) for v in values):
            self._odom_violation("non-finite odometry", now)
            return
        pose = (values[0], values[1], yaw_from_quaternion(*values[3:]))
        prev = self._odom_prev
        self._odom_prev = (pose[0], pose[1], pose[2], now)
        if prev is not None:
            dt = max(now - prev[3], ODOM_MIN_DT_S)
            speed = math.hypot(pose[0] - prev[0], pose[1] - prev[1]) / dt
            turn = abs(math.atan2(math.sin(pose[2] - prev[2]), math.cos(pose[2] - prev[2]))) / dt
            if speed > ODOM_MAX_SPEED_MPS or turn > ODOM_MAX_YAW_RATE:
                self._odom_violation("implausible odometry: %.1f m/s, %.1f rad/s between samples"
                                     % (speed, turn), now)
        # Who publishes odometry as this sample arrives (T22): exactly one
        # node, the one that identified the simulator. Kept with the sample
        # for odometry_sample(); pose() does not depend on it.
        refusal = self._receipt_refusal("odometry")
        if refusal is None and self._backend.node() is None:
            refusal = "the simulator is not identified"
        with self._lock:
            self._pose, self._pose_z, self._pose_rx = pose, values[2], now
            self._odom_seq += 1
            self._odom_refusal = refusal

    def _odom_violation(self, why, now):
        """Hold: no pose for ODOM_HOLD_S, so every motion loop stops (T22)."""
        with self._lock:
            first = now >= self._odom_bad_until
            self._odom_bad_until = now + ODOM_HOLD_S
            self.odom_violations += 1
        if first:
            LOG.warning("%s on %s: holding (no pose for %.0f s)", why, self._topic("odometry"),
                        ODOM_HOLD_S)

    def _publishers(self, name):
        """Sorted ``namespace/node`` names, one per publisher of ``name``,
        from rclpy's publisher info (``get_publishers_info_by_topic``); None
        when the graph cannot be read. The names are graph text, chosen by
        whoever creates the node: cleaned, and never passed to the model.
        The count is recorded for ``extra_publishers``."""
        node = self._node
        if node is None:
            return None
        try:
            infos = node.get_publishers_info_by_topic(self._topic(name))
            names = []
            for info in infos:
                ns = str(getattr(info, "node_namespace", "") or "").rstrip("/")
                names.append(clean_text("%s/%s" % (ns, getattr(info, "node_name", "") or "?"),
                                        96))
        except Exception as error:   # graph query failed: keep the last count
            LOG.debug("get_publishers_info_by_topic(%s) failed: %s", name, error)
            return None
        names.sort()
        with self._lock:
            self._pub_counts[name] = len(names)
            self._pub_nodes[name] = tuple(names)
        return names

    def _census(self):
        """Count the publishers of the census topics (once a second); a
        second payload/status publisher, or another node publishing it,
        contradicts an identified simulator (T9)."""
        for name in CENSUS_TOPICS:
            self._publishers(name)
        with self._lock:
            status = self._pub_nodes.get("payload/status")
        self._check_status_publishers(status)

    def _check_status_publishers(self, status):
        """Conflict when an identified simulator's payload/status no longer
        has exactly its one identifying publisher. ``status``: the census
        names (None: unreadable, no change)."""
        node = self._backend.node()
        if node is None or status is None or list(status) == [node]:
            return
        if len(status) != 1:
            why = "payload/status had %d publishers, not one" % len(status)
        else:
            why = "payload/status was published by another node than the one that identified it"
        before = self._backend.snapshot()["backend"]
        identity = self._backend.contradict(why)
        if before != identity["backend"]:
            LOG.error("%s; %s", identity["reason"], CONFLICT_TEXT)

    def _receipt_refusal(self, name):
        """Why data that just arrived on ``name`` is refused, or None. The
        census is read again at this moment (T21), not only once a second:
        exactly one publisher, and once the simulator is identified, the
        node that identified it. rclpy does not say which publisher sent a
        message, so this sees the publishers present when it arrives."""
        names = self._publishers(name)
        if names is None:
            return "the publishers of %s could not be read" % name
        if len(names) != 1:
            return "%d publishers on %s" % (len(names), name)
        facade = self._backend.node()
        if facade is not None and names[0] != facade:
            return ("%s is published by a node other than the one that identified the simulator"
                    % name)
        return None

    def _status_origin(self):
        """Who publishes payload/status, payload/gas and odometry now:
        {"ok", "node", "why"}. ``ok`` only when each has exactly one
        publisher and it is one and the same, known node, and that node is
        the simulator's facade by name and namespace (``FACADE_NODE``, T9)."""
        nodes = {name: self._publishers(name) for name in IDENTITY_TOPICS}
        if any(names is None for names in nodes.values()):
            return {"ok": False, "node": None, "why": "the ROS graph could not be read"}
        for name in IDENTITY_TOPICS:
            if len(nodes[name]) != 1:
                return {"ok": False, "node": None,
                        "why": "%s has %d publishers, not exactly one" % (name, len(nodes[name]))}
        node = nodes["payload/status"][0]
        if UNKNOWN_NODE_MARK in node:
            return {"ok": False, "node": None, "why": "its publisher's node is not known yet"}
        if any(nodes[name][0] != node for name in IDENTITY_TOPICS):
            return {"ok": False, "node": None,
                    "why": "payload/status, payload/gas and odometry are not published by one node"}
        if node != FACADE_NODE:
            # A fixed phrase: the graph's node name never reaches the model (T6).
            return {"ok": False, "node": None,
                    "why": "their one publisher is not the simulator's facade node (%s)"
                    % FACADE_NODE}
        return {"ok": True, "node": node, "why": None}

    def extra_publishers(self, name):
        """True while more than one participant publishes ``name`` (one of
        SINGLE_PUBLISHER_TOPICS): its data is refused (T21, T22)."""
        with self._lock:
            count = self._pub_counts.get(name, 0)
        if count > 1:
            if name not in self._multi_warned:
                self._multi_warned.add(name)
                LOG.error("%d publishers on %s: refusing its data (a second publisher can forge "
                          "it)", count, self._topic(name))
            return True
        self._multi_warned.discard(name)
        return False

    def _parse(self, topic, raw):
        try:
            data = json.loads(raw)
            if isinstance(data, dict):
                return data
        except (TypeError, ValueError):
            pass
        if topic not in self._bad_json_warned:  # warn once, don't flood the log
            self._bad_json_warned.add(topic)
            LOG.warning("ignoring malformed JSON on %s: %.120r", topic, raw)
        return None

    def _on_gas(self, msg):
        data = filter_gas(self._parse("payload/gas", msg.data))
        if data is None:
            return
        # Checked as it arrives (T21): a sample while a second node publishes
        # payload/gas, or from another node than the facade's, is dropped and
        # gas() refuses until a sample arrives with the facade alone again.
        refusal = self._receipt_refusal("payload/gas")
        with self._lock:
            self._gas_refusal = refusal
            if refusal is None:
                self._gas, self._gas_rx = data, time.monotonic()
        if refusal is None:
            self._multi_warned.discard("payload/gas receipt")
        elif "payload/gas receipt" not in self._multi_warned:
            self._multi_warned.add("payload/gas receipt")
            LOG.error("refusing payload/gas samples: %s (a second publisher can forge them)",
                      refusal)

    def _on_status(self, msg):
        data = self._parse("payload/status", msg.data)
        if data is None:
            return
        state = self._backend.snapshot()["backend"]
        origin = None
        if state == BACKEND_UNKNOWN and data.get(SIMULATED_KEY) is True:
            origin = self._status_origin()          # identification: who sent it (T9)
        before = state
        identity = self._backend.observe(data, origin)
        if state == BACKEND_SIMULATOR:
            self._check_status_publishers(self._publishers("payload/status"))
            identity = self._backend.snapshot()
        if identity["backend"] != before:
            level = logging.INFO if identity["simulator"] else logging.ERROR
            LOG.log(level, "robot backend: %s", identity["reason"])
        with self._lock:
            self._status, self._status_rx = filter_status(data), time.monotonic()

    def _on_probe_result(self, msg):
        data = self._parse("payload/gas/probe_result", msg.data)
        if data is None:
            return
        req_id = data.get("id")
        if not isinstance(req_id, str):
            return
        with self._lock:
            if req_id not in self._probe_pending:
                return            # not a request of ours: ignored
        refusal = self._receipt_refusal("payload/gas/probe_result")   # as it arrives (T21)
        with self._lock:
            slot = self._probe_pending.get(req_id)
            if slot is None:
                return
            if slot["result"] is not None:
                slot["duplicates"] += 1   # a second responder: the probe is refused
                return
            slot["result"] = data
            slot["refusal"] = refusal
            slot["event"].set()

    def _on_events(self, msg):
        raw = msg.data
        if PRESENTER_SOURCE not in raw:  # cheap pre-filter: our own events echo back
            return
        try:
            event = json.loads(raw)
        except (TypeError, ValueError):
            return
        # Presenter disclosures only, typed; the facade's own records
        # (source "facade") and everything else are ignored.
        event = filter_presenter_event(event)
        if event is not None:
            with self._lock:
                self._presenter_events.append((time.monotonic(), event))

    def _on_site(self, msg):
        data = self._parse("payload/site", msg.data)
        if data is not None:
            with self._lock:
                self._site = data
            LOG.info("site geometry received on %s", self._topic("payload/site"))

    # ------------------------------------------------------------------ #
    # Thread-safe getters
    # ------------------------------------------------------------------ #

    @staticmethod
    def _fresh(rx, max_age):
        return rx is not None and (max_age is None or time.monotonic() - rx <= max_age)

    def _odom_held(self):
        return time.monotonic() < self._odom_bad_until

    def pose(self, max_age=DEFAULT_POSE_MAX_AGE_S):
        """(x, y, yaw) in the odom frame, or None if missing/stale or held
        after an implausible sample."""
        with self._lock:
            if self._odom_held():
                return None
            return self._pose if self._fresh(self._pose_rx, max_age) else None

    def pose3(self, max_age=DEFAULT_POSE_MAX_AGE_S):
        """(x, y, z, yaw) in the odom frame (z = body height from
        odometry position.z), or None if missing/stale."""
        with self._lock:
            if self._odom_held() or not self._fresh(self._pose_rx, max_age) \
                    or self._pose_z is None:
                return None
            x, y, yaw = self._pose
            return (x, y, self._pose_z, yaw)

    def odometry_sample(self, max_age=DEFAULT_POSE_MAX_AGE_S):
        """The latest odometry sample, {"seq", "pose": (x, y, yaw),
        "facade", "why"}, or None while ``pose()`` would be None (missing,
        stale, or held after an implausible sample). ``seq`` numbers the
        samples as they arrive; ``facade`` is True only when, as this one
        arrived, odometry had exactly one publisher and it was the node that
        identified the simulator (``why`` says why not). Two samples with
        different ``seq`` are two messages, so one forged or stale pose
        cannot stand for two (T22). rclpy does not say which publisher sent
        a message: a publisher that sends and leaves between graph reads is
        not seen."""
        with self._lock:
            if self._odom_held() or not self._fresh(self._pose_rx, max_age):
                return None
            seq, pose, refusal = self._odom_seq, self._pose, self._odom_refusal
        return {"seq": seq, "pose": pose, "facade": refusal is None, "why": refusal}

    def gas_age(self):
        """Seconds since the last payload/gas message arrived, or None."""
        with self._lock:
            return None if self._gas_rx is None else time.monotonic() - self._gas_rx

    def recent_presenter_events(self, max_age_s=300.0):
        """Presenter disclosure events (source "presenter", e.g. kind
        "demo_inject" from host/inject_gas.sh) received in the last
        ``max_age_s`` seconds, oldest first."""
        now = time.monotonic()
        with self._lock:
            return [dict(ev) for rx, ev in self._presenter_events if now - rx <= max_age_s]

    def gas(self, max_age=DEFAULT_GAS_MAX_AGE_S):
        """Latest payload/gas dict (copy), or None if missing/stale, if
        more than one participant publishes payload/gas, or if the last
        sample was refused as it arrived (``_receipt_refusal``)."""
        if self.extra_publishers("payload/gas"):
            return None
        with self._lock:
            if self._gas_refusal is not None:
                return None
            return dict(self._gas) if self._fresh(self._gas_rx, max_age) else None

    def status(self, max_age=DEFAULT_STATUS_MAX_AGE_S):
        """Latest payload/status dict (copy), or None if missing/stale."""
        with self._lock:
            return dict(self._status) if self._fresh(self._status_rx, max_age) else None

    def backend(self):
        """Backend identity from payload/status (``BackendIdentity.snapshot()``)."""
        return self._backend.snapshot()

    def wait_for_backend(self, timeout=BACKEND_WAIT_S):
        """Wait up to ``timeout`` s for payload/status to identify the
        backend; returns ``backend()`` (still "unknown" on a timeout)."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            identity = self.backend()
            if identity["backend"] != BACKEND_UNKNOWN or time.monotonic() >= deadline:
                return identity
            time.sleep(0.1)  # nosemgrep: arbitrary-sleep

    def _refusal(self, action, retreat=False, stop=False):
        return backend_refusal(self.backend(), action, retreat=retreat, stop=stop)

    def sim_time(self):
        """Best-effort current sim time: last status sim_time plus wall time
        elapsed since it arrived (RTF ~1, extrapolation capped at 1 s).
        None when no recent status carries sim_time."""
        with self._lock:
            status, rx = self._status, self._status_rx
        if not self._fresh(rx, DEFAULT_STATUS_MAX_AGE_S) or "sim_time" not in status:
            return None
        try:
            return float(status["sim_time"]) + min(time.monotonic() - rx, 1.0)
        except (TypeError, ValueError):
            return None

    def site(self, timeout=None):
        """Latched site geometry (contract 1) or agent/site_defaults.json.

        Blocks until the latched message arrives or until ``SITE_WAIT_S``
        after connect() started (``timeout`` overrides the remaining wait).
        Missing keys in a received site are filled from the defaults.
        """
        if timeout is None:
            if self._connect_t is None:
                timeout = 0.0
            else:
                timeout = max(0.0, SITE_WAIT_S - (time.monotonic() - self._connect_t))
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                site = dict(self._site) if self._site is not None else None
            if site is not None or time.monotonic() >= deadline:
                break
            time.sleep(0.1)  # nosemgrep: arbitrary-sleep
        defaults = load_site_defaults()
        if site is not None:
            # Reviewed geometry only (T22): a forged or shifted site is refused.
            refusal = ("more than one publisher on %s" % self._topic("payload/site")
                       if self.extra_publishers("payload/site") else site_mismatch(site, defaults))
            if refusal:
                if self.site_refusal != refusal:
                    LOG.error("refusing the received site (%s); using the reviewed %s", refusal,
                              SITE_DEFAULTS_PATH)
                self.site_refusal = refusal
                return defaults
        if site is None:
            if not self._site_warned:
                self._site_warned = True
                LOG.warning("no %s received; using %s", self._topic("payload/site"),
                            SITE_DEFAULTS_PATH)
            return defaults
        missing = [k for k in defaults if k not in site]
        if missing:
            LOG.warning("site message lacks %s; filled from site_defaults.json", missing)
            for key in missing:
                site[key] = defaults[key]
        return site

    # ------------------------------------------------------------------ #
    # Trigger services
    # ------------------------------------------------------------------ #

    def _trigger(self, name, timeout=None, owner=None):
        if not self.connected:
            return False, "not connected to robot (connect() first)"
        # A stop passes once the simulator has been identified, never before
        # (T15); in a conflict only the stops and the retreat's stand and sit
        # pass (T9).
        retreat = (owner == "retreat" and name in RETREAT_SERVICES
                   and self._motion_owner == "retreat")
        refusal = self._refusal("calling %s" % name, retreat=retreat, stop=name in STOP_SERVICES)
        if refusal:
            return False, refusal
        timeout = float(SERVICE_TIMEOUT_S.get(name, 10.0) if timeout is None else timeout)
        client = self._clients.get(name)
        if client is None:
            return False, "unknown service %r" % name
        if not client.wait_for_service(timeout_sec=min(2.0, timeout)):
            return False, "service %s unavailable" % self._topic(name)
        try:
            future = client.call_async(self._msg["Trigger"].Request())
        except Exception as error:
            return False, "call %s failed: %s" % (name, error)
        # Poll instead of spinning: the executor thread completes the future.
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                try:
                    client.remove_pending_request(future)
                except Exception:
                    pass
                return False, "timeout after %.1fs waiting for %s" % (timeout, name)
            time.sleep(0.01)  # nosemgrep: arbitrary-sleep
        try:
            result = future.result()
        except Exception as error:
            return False, "call %s raised: %s" % (name, error)
        if result is None:
            return False, "call %s returned no response" % name
        ok = bool(result.success)
        LOG.debug("%s replied %r", name, clean_text(result.message, 200))
        return ok, trigger_text(name, ok, result.message)

    def claim(self):
        return self._trigger("claim")

    def release(self):
        return self._trigger("release")

    def power_on(self):
        return self._trigger("power_on")

    def power_off(self):
        return self._trigger("power_off")

    def stand(self, owner=None):
        """``owner`` "retreat": the retreat's first step, which a backend
        identity conflict does not refuse while the retreat holds motion."""
        return self._trigger("stand", owner=owner)

    def sit(self, owner=None):
        """``owner`` "retreat": the sit once a retreat has ended outside, which
        a backend identity conflict does not refuse while the retreat holds
        motion."""
        return self._trigger("sit", owner=owner)

    def stop(self):
        return self._trigger("stop")

    def self_right(self):
        return self._trigger("self_right")

    def estop_hard(self):
        return self._trigger("estop/hard")

    def estop_gentle(self):
        return self._trigger("estop/gentle")

    def estop_release(self):
        return self._trigger("estop/release")

    # ------------------------------------------------------------------ #
    # Commands / publications
    # ------------------------------------------------------------------ #

    @property
    def motion_owner(self):
        return self._motion_owner

    def set_motion_owner(self, owner):
        """Reserve cmd_vel for ``owner`` (None releases). Used by the
        mission's retreat so no other caller can steer Spot mid-escape.
        Taken under the publish lock: once this returns, no twist from a
        non-owner that was checked earlier can still go out after it."""
        with self._pub_lock:
            self._motion_owner = owner

    def _publish_twist_locked(self, vx, vy, wz):
        if self._pub_cmd is None:
            return False
        msg = self._msg["Twist"]()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self._pub_cmd.publish(msg)
        return True

    def _publish_twist(self, vx, vy, wz):
        with self._pub_lock:
            return self._publish_twist_locked(vx, vy, wz)

    def send_velocity(self, vx, vy=0.0, wz=0.0, owner=None):
        """Publish one cmd_vel (valid ~0.125 s on the robot; re-send at >= 8 Hz).

        Returns False if not connected, if another owner holds motion, or
        if the backend is not the simulator. A zero twist passes once the
        simulator has been identified (also in a later conflict), never
        before (T15: on a real driver any cmd_vel claims the lease and powers
        the motors). In a backend identity conflict only the retreat's
        motion passes (``owner`` "retreat" while it owns motion).
        """
        vx, vy, wz = clamp_twist(vx, vy, wz)
        retreat = owner == "retreat" and self._motion_owner == "retreat"
        stop = (vx, vy, wz) == (0.0, 0.0, 0.0)
        if self._refusal("stopping the robot" if stop else "moving the robot", retreat=retreat,
                         stop=stop):
            return False
        # Owner check and publish are atomic w.r.t. set_motion_owner.
        with self._pub_lock:
            holder = self._motion_owner
            if holder is not None and owner != holder and (vx, vy, wz) != (0.0, 0.0, 0.0):
                return False
            if not self.connected:
                return False
            return self._publish_twist_locked(vx, vy, wz)

    def inject_gas(self, gas, ppm, x, y, z, radius):
        """Operator-console stand-in: add a gas pocket in the sim
        (payload/gas/inject JSON {"gas","ppm","x","y","z","radius"})."""
        if not self.connected or self._pub_inject is None:
            return False
        if self._refusal("injecting simulated gas"):
            return False
        if str(gas).lower() == "o2" and not -20.9 <= _finite_or_zero(ppm) < 0.0:
            LOG.error("inject_gas: the facade accepts O2 depletion only, got %r", ppm)
            return False
        payload = sanitize_json({"gas": str(gas), "ppm": float(ppm), "x": float(x),
                                 "y": float(y), "z": float(z), "radius": float(radius)})
        msg = self._msg["String"](data=json.dumps(payload))
        with self._pub_lock:
            self._pub_inject.publish(msg)
        return True

    def _publish_json(self, publisher, payload):
        if not self.connected or publisher is None:
            return False
        msg = self._msg["String"](data=json.dumps(sanitize_json(payload)))
        with self._pub_lock:
            publisher.publish(msg)
        return True

    def set_ventilation(self, on, tau_s=None):
        """SIMULATED forced ventilation on/off (payload/gas/ventilation
        {"on", "tau_s"}). Off holds the current level in the sim; use
        reset_ventilation() to return to the unventilated start. False if
        not connected or tau_s is out of range."""
        if self._refusal("controlling the simulated ventilation"):
            return False
        payload = {"on": bool(on)}
        if tau_s is not None:
            tau = _finite_or_zero(tau_s)
            if not VENT_TAU_RANGE_S[0] <= tau <= VENT_TAU_RANGE_S[1]:
                LOG.error("set_ventilation: tau_s %r outside %s s", tau_s, VENT_TAU_RANGE_S)
                return False
            payload["tau_s"] = tau
        return self._publish_json(self._pub_vent, payload)

    def reset_ventilation(self):
        """SIMULATED: back to the sim's --gas-initial atmosphere
        (payload/gas/ventilation {"reset": true})."""
        if self._refusal("resetting the simulated atmosphere"):
            return False
        return self._publish_json(self._pub_vent, {"reset": True})

    def clear_gas(self):
        """SIMULATED: remove every injected gas pocket (payload/gas/clear).
        Built-in pockets are untouched."""
        if self._refusal("clearing simulated gas"):
            return False
        return self._publish_json(self._pub_clear, {"clear": "injected"})

    def probe(self, points, timeout=DEFAULT_PROBE_TIMEOUT_S):
        """Sample the gas field at [[x, y, z], ...] via the facade's
        extendable gas probe (simulated; arm-mounted on the real robot).

        Returns a list of samples in request order (see
        ``parse_probe_result``), or None on timeout, rejection, bad input or
        no link; ``last_probe_error`` then says why. Never blocks longer
        than ``timeout`` seconds.
        """
        try:
            pts = probe_points(points)
        except ValueError as error:
            self.last_probe_error = str(error)
            return None
        if not self.connected or self._pub_probe is None:
            self.last_probe_error = "not connected to robot"
            return None
        refusal = self._refusal("using the gas probe")
        if refusal:
            self.last_probe_error = refusal
            return None
        if self.extra_publishers("payload/gas/probe_result"):
            self.last_probe_error = ("more than one publisher on payload/gas/probe_result: probe "
                                     "replies refused")
            return None
        req_id = secrets.token_hex(8)   # unpredictable (T21); matched against pending only
        slot = {"event": threading.Event(), "result": None, "duplicates": 0, "refusal": None}
        with self._lock:
            self._probe_pending[req_id] = slot  # registered before publishing: no lost reply
        try:
            if not self._publish_json(self._pub_probe, {"id": req_id, "points": pts}):
                self.last_probe_error = "probe request not published"
                return None
            if not slot["event"].wait(max(0.0, float(timeout))):
                self.last_probe_error = "no probe reply within %.1f s" % float(timeout)
                return None
            time.sleep(PROBE_DUPLICATE_WINDOW_S)   # let a competing reply show itself  # nosemgrep: arbitrary-sleep
        finally:
            with self._lock:
                self._probe_pending.pop(req_id, None)
        if slot["duplicates"]:
            self.last_probe_error = ("more than one reply to probe request %s: probe readings "
                                     "refused" % req_id)
            return None
        # The census when the reply arrived and again now (T21): the request,
        # id included, is in clear on payload/gas/probe, so a reply is only
        # taken while the facade's node alone publishes the reply topic.
        for when, refusal in (("when the reply arrived", slot["refusal"]),
                              ("when it was accepted",
                               self._receipt_refusal("payload/gas/probe_result"))):
            if refusal:
                self.last_probe_error = ("%s %s: probe readings refused (another publisher can "
                                         "forge the reply)" % (refusal, when))
                return None
        samples, error = parse_probe_result(slot["result"], pts)
        self.last_probe_error = error
        return samples

    def publish_guard(self, beat):
        """Publish one payload/guard heartbeat ({"state", "robot_inside",
        "seq", "wall_t", "checks", "check_age_s"}; agent/mission.py
        guard_heartbeat). The three contract states pass through; anything
        else is sent as "stalled", never as a healthy state (T16). False if
        not connected."""
        state = beat.get("state")
        age = beat.get("check_age_s")
        checks = beat.get("checks")
        return self._publish_json(self._pub_guard, {
            "state": state if state in ("monitoring", "standby", "stalled") else "stalled",
            "robot_inside": bool(beat.get("robot_inside")), "seq": int(beat.get("seq", 0)),
            "wall_t": float(beat.get("wall_t", time.time())),
            "checks": int(checks) if isinstance(checks, int) and not isinstance(checks, bool) else None,
            "check_age_s": (round(float(age), 3) if isinstance(age, (int, float))
                            and not isinstance(age, bool) and math.isfinite(age) else None)})

    def publish_event(self, kind, message, data=None, source="mission"):
        """Publish one payload/events record and return it (dict).

        ``t`` is sim time when payload/status is flowing, else wall time;
        ``wall_t`` is always unix wall time. The dict is returned even when
        not connected, so callers can still write it to their audit log.
        """
        t = self.sim_time()
        event = make_event(kind, message, data, source=source, t=t)
        if self.connected and self._pub_events is not None:
            try:
                msg = self._msg["String"](data=json.dumps(event))
                with self._pub_lock:
                    self._pub_events.publish(msg)
            except Exception as error:
                LOG.warning("publish_event failed: %s", error)
        return event
