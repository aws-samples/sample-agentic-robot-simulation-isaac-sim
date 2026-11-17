"""Input checks and audit records for the simulator facade (pure stdlib).

What it is
----------
The checks ``sim/isaac_app.py`` applies to what arrives over DDS before it
reaches the walking policy or the simulated world, and the audit records the
facade publishes about it on ``payload/events``. No Isaac, rclpy or numpy
imports, so everything here unit-tests on a Mac::

    python3 -m unittest tests.test_hardening_sim

isaac_app.py keeps only the call sites; the rules live here.

Threats (docs/security/THREAT_MODEL.md, 2.3.3)
----------------------------------------------
T20  ``cmd_vel`` and ``body_pose`` must be finite and inside the demo
     envelope (:func:`check_twist`, :func:`check_body_pose`); a refused
     command stops Spot and claims nothing. Every e-stop and power service
     call, and any other call or command that changes motor power or the
     e-stop, gets an audit record (:func:`service_event`), as does a second
     publisher on ``cmd_vel`` (:class:`PublisherWatch`). No lease secrets:
     the sample has no in-code authentication (owner decision 2).
T21  Probe ids follow the shared contract (:func:`valid_probe_id`): 1-64
     printable characters, echoed back in the reply; anything else is
     refused (the agent's unpredictable ids and its match on a pending
     request are the agent package's half).
T23  Every simulated-world control (``payload/gas/inject``, ``clear``,
     ``ventilation``, ``payload/battery/reset``, ``payload/view``) is
     rate-limited (:class:`InputGuard`), size-capped, checked against
     allowlists and bounds (``validate_*``), and produces one audit record,
     applied or refused, with sim time and the topic's publishers
     (:func:`world_control_event`).
T41  The real-time factor is published in ``payload/status``
     (:class:`RtfMeter`), so the agent and the HUD can see a collapse that
     would stretch the wall-clock mission budgets.
T42  Every incoming control topic, ``cmd_vel`` and ``body_pose`` included,
     has a per-topic rate limit checked before parsing (:class:`InputGuard`,
     :data:`CONTROL_RATE_LIMITS`), so a flood costs the sim loop at most a
     bounded amount of work; drops and refusals are counted for
     ``payload/status``.
T44  No remote-access add-on exists in sim/: SimulationApp gets only
     "headless" and only the ROS 2 bridge extension is enabled. The one way
     in was the command line (host/run_sim.sh passes unknown arguments
     through, and isaac_app.py ignored them); :func:`unknown_args_error`
     now refuses them before Kit starts.

Audit records
-------------
Same shape as ``agent/robot_client.py`` ``make_event``, with source
``"facade"``: ``{"t": <sim time>, "wall_t", "source", "kind", "message",
"data"}``. Kinds: ``facade_service``, ``facade_publishers``,
``world_control``. No kind contains
"inject", "probe" or one of the HUD's alert words (``hud/ros_bridge.py``
``_INJECT_KIND``, ``_ALERT_WORDS``), so a facade record never stages an
alarm or fires the HUD's full-screen flash on its own. :class:`FacadeAudit`
bounds how many records the facade publishes, so a flood of view changes,
refused input or service calls becomes a count rather than a flood of events
(T42). The safety-relevant world controls (:data:`SAFETY_CONTROLS`: gas
inject, clear, ventilation, battery reset) are never collapsed into a count:
each one that passed its input rate limit, applied or refused, gets its own
record (T23); the input limits bound how many that can be.
"""

import collections
import json
import math
import threading
import time

EVENT_SOURCE = "facade"
MAX_TEXT_CHARS = 200        # a reason or name copied into a record or a log line
MAX_MESSAGE_CHARS = 300
MAX_NAMES = 8               # publisher names kept in one record


def clean_text(value, limit=MAX_TEXT_CHARS):
    """Printable, bounded copy of ``value`` for a log line or audit record.

    Every non-printable character (newline, escape, bidi control) becomes
    "?", and the text is cut at ``limit`` characters ("..." marks a cut).
    """
    text = str(value)
    out = "".join(ch if ch.isprintable() else "?" for ch in text[:limit])
    return out + "..." if len(text) > limit else out


def sanitize(obj, _depth=0):
    """Strict-JSON copy of ``obj``: tuples become lists, non-finite floats
    None, strings :func:`clean_text`; at most 32 items per level, 5 levels."""
    if obj is None or isinstance(obj, (bool, int)):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if _depth >= 5:
        return None
    if isinstance(obj, dict):
        return {clean_text(k, 64): sanitize(v, _depth + 1) for k, v in list(obj.items())[:32]}
    if isinstance(obj, (list, tuple)):
        return [sanitize(v, _depth + 1) for v in list(obj)[:32]]
    return clean_text(obj)


def make_event(kind, message, data=None, sim_time=None, wall_t=None):
    """One ``payload/events`` record (agent/robot_client.py make_event shape).

    ``t`` is the facade's sim time, or the wall time when no finite sim
    time is given; ``wall_t`` is always unix wall time.
    """
    wall = time.time() if wall_t is None else float(wall_t)
    finite_sim = (isinstance(sim_time, (int, float)) and not isinstance(sim_time, bool)
                  and math.isfinite(sim_time))
    return {
        "t": round(float(sim_time if finite_sim else wall), 4),
        "wall_t": round(wall, 3),
        "source": EVENT_SOURCE,
        "kind": str(kind),
        "message": clean_text(message, MAX_MESSAGE_CHARS),
        "data": sanitize(data) if data is not None else None,
    }


class TokenBucket:
    """``rate_hz`` sustained, ``burst`` at once. ``allow(now)`` spends one
    token if there is one; ``now`` is a monotonic time in seconds."""

    def __init__(self, rate_hz, burst):
        self.rate = float(rate_hz)
        self.burst = float(burst)
        if not (self.rate > 0.0 and self.burst >= 1.0):
            raise ValueError("TokenBucket needs rate_hz > 0 and burst >= 1")
        self.tokens = self.burst
        self._t = None

    def allow(self, now):
        now = float(now)
        if self._t is not None:
            self.tokens = min(self.burst, self.tokens + max(0.0, now - self._t) * self.rate)
        self._t = now
        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True
        return False


# ---------------------------------------------------------------------------
# Audit records (payload/events)
# ---------------------------------------------------------------------------

# Budget name -> (records per second sustained, burst). "change": something
# was applied or changed (a view change, motor power, the e-stop, the set of
# cmd_vel publishers). "noise": refusals and calls that changed nothing.
# host/reset_spot.sh sends four controls and a self_right within a second, a
# mission a handful of service calls: legitimate use stays inside the bursts.
AUDIT_BUDGETS = {"change": (0.5, 20), "noise": (0.2, 10)}
AUDIT_MAX_PENDING = 64
# T23: world controls that change the simulated atmosphere or battery. Their
# world_control records, applied or refused, are in no budget: each is
# published, so an undisclosed purge can never hide in a count. CONTROL_RATE_LIMITS
# bounds them (2.5 records/s for the four together), and isaac_app.py applies
# them from two 64-entry queues and drains this audit on the same sim-loop
# tick, so at most 128 are pending at once; WORLD_MAX_PENDING is only a
# backstop for a loop that stops draining.
SAFETY_CONTROLS = ("gas/inject", "gas/clear", "gas/ventilation", "battery/reset")
WORLD_MAX_PENDING = 256


def safety_control(kind, data):
    """True for the world_control record of a :data:`SAFETY_CONTROLS` control."""
    return kind == "world_control" and isinstance(data, dict) \
        and data.get("control") in SAFETY_CONTROLS


class FacadeAudit:
    """Thread-safe, rate-bounded queue of facade audit records (JSON text).

    ROS callbacks (executor thread) and the sim thread call :meth:`record`;
    the sim loop publishes :meth:`drain` on ``payload/events``. The two
    budgets of :data:`AUDIT_BUDGETS` are separate, so a flood of refused
    input cannot use up the budget that records what was actually applied.
    A record over budget is counted, not queued: :attr:`suppressed` (in
    ``payload/status``) and a ``suppressed_before`` block on the next record
    that is published, by kind (world_control records by control).
    A :data:`SAFETY_CONTROLS` record is in no budget and has its own
    :data:`WORLD_MAX_PENDING` backstop, so other records cannot crowd it out.
    """

    def __init__(self, budgets=None, clock=time.monotonic, wall=time.time,
                 max_pending=AUDIT_MAX_PENDING, max_world_pending=WORLD_MAX_PENDING):
        budgets = AUDIT_BUDGETS if budgets is None else budgets
        self._buckets = {name: TokenBucket(rate, burst) for name, (rate, burst) in budgets.items()}
        self._lock = threading.Lock()
        self._clock, self._wall = clock, wall
        self._max_pending = int(max_pending)
        self._max_world = int(max_world_pending)
        self._pending = collections.deque()
        self._counts = collections.Counter()  # pending records per class ("world", "other")
        self._held = collections.Counter()   # suppressed since the last published record
        self.suppressed = 0

    def record(self, kind, message, data=None, changed=True, sim_time=None):
        """Queue one record; False if it was suppressed (over budget)."""
        with self._lock:
            world = safety_control(kind, data)
            cls = "world" if world else "other"
            if world:
                ok = self._counts[cls] < self._max_world
            else:
                ok = (self._counts[cls] < self._max_pending
                      and self._buckets["change" if changed else "noise"].allow(self._clock()))
            if not ok:
                label = str(kind)
                if kind == "world_control" and isinstance(data, dict) and data.get("control"):
                    label = "%s/%s" % (kind, clean_text(data["control"], 32))
                self._held[label] += 1
                self.suppressed += 1
                return False
            data = dict(data or {})
            if self._held:
                data["suppressed_before"] = {"count": sum(self._held.values()),
                                             "by_kind": dict(self._held)}
            text = json.dumps(make_event(kind, message, data, sim_time, self._wall()),
                              allow_nan=False)
            self._held.clear()
            self._pending.append(text)
            self._counts[cls] += 1
            return True

    @property
    def pending(self):
        return bool(self._pending)

    def drain(self):
        """Every queued record (JSON text), oldest first; empties the queue."""
        with self._lock:
            out = list(self._pending)
            self._pending.clear()
            self._counts.clear()
        return out


# ---------------------------------------------------------------------------
# T20: motion commands, the robot state machine, cmd_vel publishers
# ---------------------------------------------------------------------------

# The demo envelope (vx m/s, vy m/s, wz rad/s): agent/robot_client.py
# MAX_VX / MAX_VY / MAX_WZ, which clamp the agent's own commands. The facade
# clamps a small overshoot into it and refuses anything beyond twice it
# (2 m/s is above a real Spot's ~1.6 m/s top speed: not a command, garbage).
CMD_VEL_LIMITS = (1.0, 0.5, 1.0)
CMD_VEL_REFUSE_FACTOR = 2.0
# body_pose (the v1 stub logs it and applies nothing): a body height offset
# and a small tilt about the stance, as spot_ros2 applies them on the next
# stand.
BODY_POSE_MAX_Z_M = 0.2
BODY_POSE_MAX_ANGLE_RAD = 0.5


def _finite_numbers(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in values)


def check_twist(linear, angular):
    """``(vx, vy, wz)`` for the policy, or ``(None, reason)``.

    ``linear`` and ``angular`` are the Twist's (x, y, z). Contract
    (docs/facade-contract-v1.md): only linear.x, linear.y and angular.z
    move Spot, but a non-finite value in any field makes the message
    malformed. Each used component is clamped into :data:`CMD_VEL_LIMITS`;
    one beyond :data:`CMD_VEL_REFUSE_FACTOR` times its limit refuses the
    whole command.
    """
    linear, angular = tuple(linear), tuple(angular)
    if len(linear) != 3 or len(angular) != 3 or not _finite_numbers(linear + angular):
        return None, "non-finite or malformed Twist"
    out = []
    for name, value, limit in zip(("linear.x", "linear.y", "angular.z"),
                                  (linear[0], linear[1], angular[2]), CMD_VEL_LIMITS):
        hard = limit * CMD_VEL_REFUSE_FACTOR
        if abs(value) > hard:
            return None, "%s %.3g outside +/-%g" % (name, value, hard)
        out.append(max(-limit, min(limit, float(value))))
    return tuple(out), None


def check_body_pose(position, orientation):
    """None if a body_pose is acceptable, else the reason.

    ``position`` = (x, y, z), ``orientation`` = (w, x, y, z). Every field
    finite, |z| <= :data:`BODY_POSE_MAX_Z_M`, a unit quaternion (norm within
    5 %) whose roll, pitch and yaw are each within
    :data:`BODY_POSE_MAX_ANGLE_RAD`.
    """
    position, orientation = tuple(position), tuple(orientation)
    if len(position) != 3 or len(orientation) != 4 or not _finite_numbers(position + orientation):
        return "non-finite or malformed Pose"
    if abs(position[2]) > BODY_POSE_MAX_Z_M:
        return "position.z %.3g outside +/-%g m" % (position[2], BODY_POSE_MAX_Z_M)
    norm = math.sqrt(sum(v * v for v in orientation))
    if not 0.95 <= norm <= 1.05:
        return "orientation is not a unit quaternion (norm %.3g)" % norm
    w, x, y, z = (v / norm for v in orientation)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    for name, angle in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
        if abs(angle) > BODY_POSE_MAX_ANGLE_RAD:
            return "%s %.3g rad outside +/-%g" % (name, angle, BODY_POSE_MAX_ANGLE_RAD)
    return None


# Services whose every call is recorded. Any other call (stand, sit,
# self_right, ...) or cmd_vel's implicit power-on is recorded when it changes
# motor power or the e-stop.
AUDITED_SERVICES = frozenset(("power_on", "power_off", "estop/hard", "estop/gentle",
                              "estop/release"))


def service_event(name, before, after, success=True, response=""):
    """``(kind, message, data, changed)`` for a state-machine call, or None.

    ``before`` / ``after`` are ``(powered, estopped)`` around the call.
    rclpy hands a service callback no caller identity, so the record names
    the call, its outcome and the state change, not who made it.
    """
    before = tuple(bool(v) for v in before)
    after = tuple(bool(v) for v in after)
    changed = before != after
    if name not in AUDITED_SERVICES and not changed:
        return None

    def describe(powered, estopped):
        return "motor power %s, e-stop %s" % ("on" if powered else "off",
                                             "asserted" if estopped else "released")
    message = "%s %s: %s" % (name, "succeeded" if success else "refused", describe(*after))
    if changed:
        message += " (was %s)" % describe(*before)
    data = {"service": str(name), "success": bool(success), "response": clean_text(response),
            "powered": after[0], "estopped": after[1],
            "was_powered": before[0], "was_estopped": before[1], "changed": changed}
    return "facade_service", message, data, changed


PUBLISHER_WATCH_PERIOD_S = 1.0


def publisher_names(endpoints):
    """Sorted ``namespace/node`` names, one per publisher, from rclpy
    ``TopicEndpointInfo``-like objects (``node_namespace``, ``node_name``)."""
    names = []
    for ep in endpoints:
        ns = str(getattr(ep, "node_namespace", "") or "").rstrip("/")
        names.append(clean_text("%s/%s" % (ns, getattr(ep, "node_name", "") or "?"), 96))
    return sorted(names)


class PublisherWatch:
    """Notice more than one publisher on a command topic (T20).

    :meth:`update` takes the current publisher names (one per DDS writer)
    and returns an audit record ``(kind, message, data, changed)`` when the
    topic gains a second publisher, when the set changes while there are
    several, and when it is back to one or none; otherwise None.
    :attr:`count` is the latest count (``payload/status``), None until the
    first update.
    """

    KIND = "facade_publishers"

    def __init__(self, topic):
        self.topic = str(topic)
        self.count = None
        self._reported = None   # the multi-publisher set last recorded

    def update(self, names):
        names = sorted(str(n) for n in names)
        self.count = len(names)
        shown = _shown(names)
        data = {"topic": self.topic, "count": len(names), "publishers": names[:MAX_NAMES]}
        if len(names) >= 2:
            if tuple(names) == self._reported:
                return None
            self._reported = tuple(names)
            data["multiple"] = True
            return (self.KIND, "%d publishers on %s (%s): any of them can drive Spot"
                    % (len(names), self.topic, shown), data, True)
        if self._reported is not None:
            self._reported = None
            data["multiple"] = False
            return (self.KIND, "%s is back to %d publisher(s)%s"
                    % (self.topic, len(names), " (%s)" % shown if names else ""), data, True)
        return None


def _shown(names, n=3):
    return ", ".join(names[:n]) + (", ..." if len(names) > n else "")


# ---------------------------------------------------------------------------
# T21: probe request ids
# ---------------------------------------------------------------------------

# Contract: the agent sends secrets.token_hex(8) (16 characters); the facade
# accepts 1-64 printable ASCII characters (space to tilde) and echoes the id.
PROBE_ID_MAX_CHARS = 64


def valid_probe_id(value):
    """True for a str of 1..PROBE_ID_MAX_CHARS printable ASCII characters."""
    return (isinstance(value, str) and 1 <= len(value) <= PROBE_ID_MAX_CHARS
            and all(" " <= ch <= "~" for ch in value))


# ---------------------------------------------------------------------------
# Per-topic rate limits (T23 world controls, T42 every control topic)
# ---------------------------------------------------------------------------

# Topic (relative to the namespace, without "payload/") -> (messages per
# second sustained, burst). Human-paced controls get human-paced limits:
# host/reset_spot.sh sends clear + ventilation reset + battery + view at once,
# inject_gas.sh one control per run. Probes: the mission sends one per 0.5 s.
# cmd_vel: the mission's motion loop and the guard's retreat each publish at
# 20 Hz (agent/mission.py CONTROL_HZ); 100 Hz leaves room for both. Messages
# over the limit are dropped, not applied: the 0.125 s watchdog still holds
# (and then zeroes) the last accepted command.
CONTROL_RATE_LIMITS = {
    "cmd_vel": (100.0, 50),
    "body_pose": (10.0, 10),
    "gas/inject": (1.0, 4),
    "gas/clear": (0.5, 3),
    "gas/ventilation": (0.5, 3),
    "gas/probe": (10.0, 10),
    "battery/reset": (0.5, 2),
    "view": (1.0, 4),
}


class InputGuard:
    """Per-topic token buckets plus drop and refusal counters (thread-safe).

    :meth:`allow` runs in the ROS callback, before anything is parsed or
    queued; a message over its topic's limit is dropped and counted.
    :meth:`drop` counts other drops (a full queue), :meth:`reject` input
    that was refused after parsing. :meth:`counters` -> ``{"dropped": {...},
    "rejected": {...}}`` for ``payload/status``.
    """

    def __init__(self, limits=None, clock=time.monotonic):
        limits = CONTROL_RATE_LIMITS if limits is None else limits
        self._buckets = {topic: TokenBucket(rate, burst) for topic, (rate, burst) in limits.items()}
        self._clock = clock
        self._lock = threading.Lock()
        self._dropped = dict.fromkeys(limits, 0)
        self._rejected = dict.fromkeys(limits, 0)

    def allow(self, topic):
        with self._lock:
            if self._buckets[topic].allow(self._clock()):
                return True
            self._dropped[topic] += 1
            return False

    def drop(self, topic):
        with self._lock:
            self._dropped[topic] += 1

    def reject(self, topic):
        with self._lock:
            self._rejected[topic] += 1

    def counters(self):
        with self._lock:
            return {"dropped": dict(self._dropped), "rejected": dict(self._rejected)}


# ---------------------------------------------------------------------------
# T23: simulated-world controls
# ---------------------------------------------------------------------------

# Longest control payload accepted (a 16-point probe request is ~600
# characters); longer ones are refused without being parsed or kept.
MAX_CONTROL_CHARS = 4096
# Ventilation time constant. The lower bound is the shortest any shipped
# client uses (host/e2e_client.py, the mission's VENT_TAU_RANGE_S): the
# threat model's 0.5 s "instant purge" is refused.
VENT_TAU_RANGE_S = (2.0, 3600.0)
# Injected spikes. gas -> (lowest, highest, unit). O2 is a signed delta, and
# only depletion is accepted: a positive delta laid over a depleted area
# would read normal and mask the hazard (M23).
INJECT_GASES = {
    "h2s": (0.0, 500.0, "ppm"),
    "co": (0.0, 1500.0, "ppm"),
    "lel": (0.0, 100.0, "% LEL"),
    "o2": (-20.9, 0.0, "% O2 points"),
}
INJECT_KEYS = ("gas", "ppm", "x", "y", "z", "radius")
INJECT_RADIUS_RANGE_M = (0.2, 5.0)    # agent/spot_agent.py inject_gas_demo_spike
INJECT_Z_RANGE_M = (0.0, 4.0)         # tank floor to roof (sim/build_scene.py TANK_HEIGHT)
INJECT_MARGIN_M = 15.0                # beyond the shell: covers the spawn 8 m out


def bounded(text):
    """``text`` if it is within :data:`MAX_CONTROL_CHARS`, else None (the
    queue then holds no oversized payload; :func:`control_text` refuses it)."""
    return text if isinstance(text, str) and len(text) <= MAX_CONTROL_CHARS else None


def control_text(text, control):
    if text is None:
        raise ValueError("payload/%s: payload missing or over %d characters"
                         % (control, MAX_CONTROL_CHARS))
    return text


def parse_control_json(text, control):
    """The JSON object of a control payload; empty text is ``{}``."""
    text = control_text(text, control)
    if not text.strip():
        return {}
    d = json.loads(text)
    if not isinstance(d, dict):
        raise ValueError("payload/%s expects a JSON object" % control)
    return d


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s must be a number" % name)
    try:
        number = float(value)
    except OverflowError:
        number = math.inf
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _check_keys(d, allowed, control):
    unknown = sorted(str(k) for k in set(d) - set(allowed))
    if unknown:
        raise ValueError("payload/%s: unknown key(s) %s" % (control, clean_text(", ".join(unknown), 80)))


def validate_inject(d, tank_center=(0.0, 0.0), tank_radius=6.0):
    """Normalized ``{"gas", "ppm", "x", "y", "z", "radius"}`` or ValueError.

    Exactly the six keys; ``gas`` one of :data:`INJECT_GASES` with its
    amount inside the range (O2: depletion only); radius, height and the
    distance from the tank centre bounded. The pocket count is capped by
    ``GasField.inject_spike`` itself.
    """
    _check_keys(d, INJECT_KEYS, "gas/inject")
    missing = [k for k in INJECT_KEYS if k not in d]
    if missing:
        raise ValueError("payload/gas/inject: missing %s" % ", ".join(missing))
    gas = d["gas"].strip().lower() if isinstance(d["gas"], str) else None
    if gas not in INJECT_GASES:
        raise ValueError("gas must be one of %s" % ", ".join(INJECT_GASES))
    ppm, x, y, z, radius = (_number(d[k], k) for k in INJECT_KEYS[1:])
    lo, hi, unit = INJECT_GASES[gas]
    if gas == "o2":
        if not lo <= ppm < 0.0:
            raise ValueError("o2 is a depletion delta in [%g, 0) %s; a positive delta could "
                             "mask a depleted atmosphere" % (lo, unit))
    elif not lo < ppm <= hi:
        raise ValueError("%s amount must be in (%g, %g] %s" % (gas, lo, hi, unit))
    if not INJECT_RADIUS_RANGE_M[0] <= radius <= INJECT_RADIUS_RANGE_M[1]:
        raise ValueError("radius must be in [%g, %g] m" % INJECT_RADIUS_RANGE_M)
    if not INJECT_Z_RANGE_M[0] <= z <= INJECT_Z_RANGE_M[1]:
        raise ValueError("z must be in [%g, %g] m" % INJECT_Z_RANGE_M)
    dist = math.hypot(x - float(tank_center[0]), y - float(tank_center[1]))
    limit = float(tank_radius) + INJECT_MARGIN_M
    if dist > limit:
        raise ValueError("spike centre is %.1f m from the tank centre, beyond %.1f m" % (dist, limit))
    return {"gas": gas, "ppm": ppm, "x": x, "y": y, "z": z, "radius": radius}


def describe_inject(spike):
    return "%s %g %s at (%.2f, %.2f, %.2f), radius %.2f m" % (
        spike["gas"].upper(), spike["ppm"], INJECT_GASES[spike["gas"]][2],
        spike["x"], spike["y"], spike["z"], spike["radius"])


def validate_clear(d):
    """``{}`` (host/reset_spot.sh, inject_gas.sh) or ``{"clear": "injected"}``
    (agent/robot_client.py); anything else is refused."""
    if d and not (len(d) == 1 and d.get("clear") == "injected"):
        raise ValueError('payload/gas/clear expects {} or {"clear": "injected"}')
    return {}


def validate_ventilation(d):
    """``{"reset": True}`` or ``{"on": bool, "tau_s": float or None}``."""
    if "reset" in d:
        if len(d) != 1 or d["reset"] is not True:
            raise ValueError('payload/gas/ventilation reset expects exactly {"reset": true}')
        return {"reset": True}
    _check_keys(d, ("on", "tau_s"), "gas/ventilation")
    if not isinstance(d.get("on"), bool):
        raise ValueError('expects {"on": true|false, "tau_s": optional} or {"reset": true}')
    tau = d.get("tau_s")
    if tau is not None:
        tau = _number(tau, "tau_s")
        if not VENT_TAU_RANGE_S[0] <= tau <= VENT_TAU_RANGE_S[1]:
            raise ValueError("tau_s must be in [%g, %g] s, got %g" % (VENT_TAU_RANGE_S + (tau,)))
    return {"on": d["on"], "tau_s": tau}


def validate_battery_reset(d):
    """Only an empty payload or ``{}`` (host/reset_spot.sh)."""
    if d:
        raise ValueError("payload/battery/reset expects an empty payload or {}")
    return {}


def world_control_event(control, params=None, error=None, publishers=None, detail=None,
                        injected_pockets=None):
    """``(kind, message, data, changed)``: the audit record for one world
    control the facade applied (``error`` None) or refused (T23).

    ``publishers`` are the topic's publishers when the control arrived: the
    candidates for who sent it (DDS gives no per-message sender here).
    """
    accepted = error is None
    reason = None if accepted else clean_text(error)
    who = " [publishers: %s]" % _shown(publishers) if publishers else ""
    if accepted:
        message = "SIMULATED %s applied%s%s" % (control, ": %s" % detail if detail else "", who)
    else:
        message = "SIMULATED %s refused: %s%s" % (control, reason, who)
    data = {"control": str(control), "accepted": accepted, "reason": reason, "params": params,
            "publishers": list(publishers)[:MAX_NAMES] if publishers is not None else None,
            "injected_pockets": injected_pockets, "simulated": True}
    return "world_control", message, data, accepted


# ---------------------------------------------------------------------------
# T44: nothing on the command line reaches Kit
# ---------------------------------------------------------------------------


def unknown_args_error(unknown):
    """The parser error for arguments isaac_app.py does not define, or None.

    isaac_app.py forwards nothing to Kit. An extension switch or a Kit
    setting override (``--enable <extension>``, ``--/path=value``) could
    turn on a remote viewer or another network listener, bound to every
    interface under ``--network=host``. host/run_sim.sh accepts only its own
    options and isaac_app.py's; refusing anything else here as well, before
    SimulationApp starts, covers any other way of launching the app and keeps
    such add-ons off unless the code itself enables them.
    """
    if not unknown:
        return None
    return ("unrecognized arguments: %s (isaac_app.py passes nothing through to Kit; see --help)"
            % clean_text(" ".join(str(a) for a in unknown), 120))


# ---------------------------------------------------------------------------
# T41: real-time factor
# ---------------------------------------------------------------------------

RTF_WINDOW_S = 2.0


class RtfMeter:
    """Real-time factor (sim seconds per wall second) over the last complete
    window of at least ``window_s`` wall seconds.

    :meth:`update` takes a monotonic wall time and the sim time, once per
    sim-loop tick; :attr:`rtf` is None until the first window completes.
    A stalled loop shows up at the next tick as a window with little sim
    time in it. Rounded to 0.001.
    """

    def __init__(self, window_s=RTF_WINDOW_S):
        self.window_s = float(window_s)
        if not self.window_s > 0.0:
            raise ValueError("window_s must be > 0")
        self.rtf = None
        self._anchor = None

    def update(self, wall, sim):
        wall, sim = float(wall), float(sim)
        if self._anchor is None:
            self._anchor = (wall, sim)
            return self.rtf
        wall0, sim0 = self._anchor
        if wall - wall0 >= self.window_s:
            self.rtf = round(max(0.0, sim - sim0) / (wall - wall0), 3)
            self._anchor = (wall, sim)
        return self.rtf
