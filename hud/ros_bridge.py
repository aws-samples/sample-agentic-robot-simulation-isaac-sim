"""Data layer for the presenter HUD: ROS 2 topics -> thread-safe HUD state.

What it is
----------
Three pieces, all importable on a Mac WITHOUT rclpy (rclpy is imported lazily
inside ``RosBridge.start``), so the unit tests and ``--fake`` mode need only
the stdlib + Pillow:

* ``HudState`` -- thread-safe store of the latest telemetry: payload/status,
  payload/gas (+ its ``ventilation`` block, normalized), the latest
  payload/gas/probe_result (door-mouth probe: up to 8 points, e.g. the
  mission's 2 depths x 3 heights), payload/site
  (latched), odometry pose + a downsampled path trail, the latest camera
  frame as JPEG, bounded per-source buffers of payload/events entries (each
  stamped with the HUD's view at receipt, for the audit export; see
  SAFETY_BUFFER), per-topic ingest counters, the permit
  state + pre-entry checklist (isolation, LOTO, attendant, rescue plan)
  derived from those events, a de-duplicated "alert" that drives the
  full-screen alarm banner (with the triggering reading and retreat note),
  and a flag for every simulator atmosphere change (the facade's own
  world_control records) that no presenter disclosure accompanies (M23).
* ``RosBridge`` -- rclpy subscriber node under /demo_spot feeding HudState
  from the shared-contract-5 topics (payload/status, payload/gas,
  payload/gas/probe_result, payload/events, payload/site, odometry,
  camera/frontleft/image) and the agent's SafetyGuard heartbeat
  (payload/guard, T16: GUARD DOWN / GUARD NOT RUNNING on the page).
* ``FakeBridge`` -- deterministic synthetic scenario (walk to the door,
  atmospheric tests in OSHA order, entry, H2S alarm just after t = 20 s,
  guard retreat, permit ABORTED) driven through the SAME ``HudState.ingest_*``
  methods the ROS callbacks use, so tests exercise the real ingest path.
  Its replay mode, ``ReplayBridge``, re-publishes a RECORDED dry run
  (docs/evidence/events-*.jsonl + the session's HUD audit CSV) the same way.

Read-only by design: the HUD creates no publishers and no service clients.
Nothing in the HUD can command the robot, so no HUD failure can ever leave
the robot moving -- stopping the robot stays the job of the mission's
SafetyGuard and the agent, which own the command path.

Permit state and alarms come from payload/events (the sim facade knows
nothing about permits). Conventions the MISSION/AGENT modules should follow
(documented in hud/README.md):
  * permit transitions: any event whose ``data`` carries
    ``permit_state`` (or ``state`` / ``to``) equal to one of PERMIT_STATES;
  * full-screen flash: event ``kind`` containing alarm/abort/estop/retreat/
    hazard/excursion/conflict (but not complete/cleared/released/...), or
    ``data.alarm`` true, or a permit transition to ABORTED, or a rising edge
    of payload/gas ``alarm`` or payload/status ``estopped`` -- the last three
    need no cooperation from other modules. One flash per incident: signals
    less than ALERT_DEDUPE_S apart (and a gas alarm that stays on) merge,
    except an E-stop or a failed retreat, which always flash anew;
  * permit checklist: the Permit dict nested as ``data.permit`` (any event),
    or the same keys at the top level of a ``kind`` containing "permit"
    (see PERMIT_CHECKLIST for the accepted key names); absent = unknown;
  * probe results: payload/gas/probe_result, or an event whose ``kind``
    contains "probe" with the same JSON in ``data`` (see normalize_probe).

Transport: Fast DDS is pinned to UDPv4 (``FASTDDS_BUILTIN_TRANSPORTS``,
set by hud/Dockerfile and defaulted here before rclpy loads) because its
shared-memory transport silently drops samples between containers running
as different uids (Isaac runs as uid 1234) -- see host/run_sim.sh.
"""

import collections
import io
import itertools
import json
import logging
import math
import os
import re
import threading
import time

from PIL import Image, ImageDraw, ImageFont

log = logging.getLogger("hud.bridge")

# ---------------------------------------------------------------------------
# Constants shared with the frontend (sent in every snapshot)
# ---------------------------------------------------------------------------

DEFAULT_NAMESPACE = "/demo_spot"
DEFAULT_CAMERA_TOPIC = "camera/frontleft/image"
PROBE_TOPIC = "payload/gas/probe_result"
# T16 interface contract: the agent publishes its SafetyGuard heartbeat at
# 2 Hz for as long as it runs (std_msgs/String JSON, reliable, VOLATILE,
# depth 10): {"state": "monitoring" | "standby" | "stalled", "robot_inside":
# bool, "seq": int, "wall_t": float}, plus "checks" (completed gas
# evaluations, advanced only by the guard's check loop) and "check_age_s"
# (seconds since the last one). "monitoring" = the guard is armed and its
# checks are completing; "stalled" = its thread is gone or no check has
# completed for over 2 s (GUARD STALLED, an alarm). GUARD DOWN: Spot was last
# reported inside and no heartbeat for more than GUARD_TIMEOUT_S; GUARD NOT
# RUNNING: none ever seen.
GUARD_TOPIC = "payload/guard"
GUARD_STATES = ("monitoring", "standby", "stalled")
GUARD_TIMEOUT_S = 3.0
GUARD_PERIOD_S = 0.5        # the contract's 2 Hz (FakeBridge publishes at this rate)

PERMIT_STATES = (
    "NOT_STARTED",
    "PRE_ENTRY_TESTING",
    "ENTRY_AUTHORIZED",
    "ENTRY_IN_PROGRESS",
    "COMPLETED",
    "ABORTED",
)

# OSHA 1910.146 Appendix E. Duplicated from sim/gas_field.py on purpose: the
# HUD container does not ship sim/. These only draw threshold lines; the
# per-gas alarm booleans shown on the gauges come from payload/gas itself.
OSHA_THRESHOLDS = {
    "o2": {"key": "o2_pct", "unit": "%", "low": 19.5, "high": 23.5},
    "lel": {"key": "lel_pct", "unit": "%", "alarm_at": 10.0},
    "h2s": {"key": "h2s_ppm", "unit": "ppm", "alarm_at": 10.0},
    "co": {"key": "co_ppm", "unit": "ppm", "alarm_at": 35.0},
}

EVENT_BUFFER = 200          # newest routine/unverified events in the page's log (task contract)
# T42: a flood under one name must not push another's records out of the
# HUD or its audit export. Each allowlisted source has its own bounded
# buffer for safety records (alarms, permit transitions, retreats, test
# verdicts, presenter disclosures, the facade's records;
# HudState._record_class) and another for the rest; unverified sources share
# one small buffer, and the HUD's own records have two: payload/gas alarm
# edges, and undisclosed-change flags (M23). The facade's records that
# changed nothing (refusals) have a buffer apart from what it applied.
# Evictions are counted.
SAFETY_BUFFER = 100         # per allowlisted source
ROUTINE_BUFFER = 200        # per allowlisted source
LOCAL_BUFFER = 100          # each of the HUD's own two buffers
UNVERIFIED_BUFFER = 50      # every unverified source together
PROTECTED_RECORDS = ("safety", "local")   # never trimmed from the page before routine lines
# T42: per-topic ingest limits (sustained messages/s, burst) applied in
# RosBridge before a message is parsed; the excess is dropped and counted
# (snapshot "ingest", audit headers, the page's INGEST LIMITED badge).
# Observed peaks: payload/events 7/s in the dry runs, gas 4 Hz, status
# 2 Hz, odometry 50 Hz, camera 10 Hz. payload/events allows a burst for
# the TRANSIENT_LOCAL history (EVENT_HISTORY_DEPTH) a restarted HUD gets.
INGEST_LIMITS = {
    "payload/status": (10.0, 20),
    "payload/gas": (20.0, 40),
    "payload/gas/probe_result": (5.0, 10),
    "payload/events": (400.0, 800),
    "payload/site": (2.0, 5),
    "payload/guard": (10.0, 20),
    "odometry": (150.0, 300),
    "camera": (30.0, 60),
}
# T42: payload/events is limited a second time, after parsing, per declared
# source (25 msg/s, burst 250 each, at most SOURCE_BUCKETS_MAX distinct names;
# any further name shares one bucket). The per-topic limit above only bounds
# the parsing work, so a flood under one name, an unverified one included, no
# longer starves the guard's and the mission's records. Sources are
# self-declared (T3): a flood under an allowlisted name still drops that
# name's own records, counted, and a flood above the per-topic limit drops
# everyone's, because rclpy does not say who sent a message.
SOURCE_EVENT_LIMIT = (25.0, 250)
SOURCE_BUCKETS_MAX = 16
TRAIL_MAX_POINTS = 1500     # ~150 m of path at 0.1 m spacing
TRAIL_MIN_STEP_M = 0.10     # skip odometry jitter; 50 Hz odom would bloat it
# Consecutive odometry poses farther apart than this are a teleport (reset
# to spawn), not walking (<= ~2 m/s at 10+ Hz): the path trail restarts.
TRAIL_TELEPORT_M = 1.5
# An incident stays open while alert signals (or alarm-true gas samples) keep
# arriving less than ALERT_DEDUPE_S apart: gas edge + guard event + ABORTED +
# the guard's 5 s "monitor" echoes during the retreat = ONE flash.
ALERT_DEDUPE_S = 5.0
# Titles that flash again even inside an open incident (new, worse news).
ESTOP_TITLE = "E-STOP ENGAGED"
RETREAT_FAILED_TITLE = "RETREAT INCOMPLETE"
ESCALATION_TITLES = (ESTOP_TITLE, RETREAT_FAILED_TITLE)
MAX_JSON_CHARS = 256_000    # refuse absurd payloads instead of choking the UI
MAX_MESSAGE_CHARS = 2000
MAX_DATA_CHARS = 4000
GAS_ORDER = ("o2", "lel", "h2s", "co")   # OSHA sampling order

# Pre-entry permit checklist (API RP 2016 / OSHA 1910.146 flavored). Each row:
# (key, label, accepted data keys). The mission's Permit dict is read from
# data.permit (any event) or from the top level of a 'permit' event.
PERMIT_CHECKLIST = (
    ("isolation", "Isolation", ("isolation", "isolation_confirmed", "isolated",
                                "energy_isolation", "isolation_ok")),
    ("loto", "LOTO", ("loto", "loto_confirmed", "lockout_tagout", "lockout", "loto_ok")),
    ("attendant", "Attendant", ("attendant", "attendant_name", "attendant_id")),
    ("rescue_plan", "Rescue plan", ("rescue_plan", "rescue", "rescue_service",
                                    "rescue_team", "rescue_plan_name")),
)
_NEGATIVE_TEXT = frozenset(("", "false", "no", "n", "none", "null", "pending", "tbd", "unset",
                            "not set", "not confirmed", "unconfirmed", "missing"))

# Banner note: shown once a retreat has actually started (never for the
# rehearsal mode with auto-retreat off). While it runs the note says it is
# under way; only retreat_complete upgrades it to the past tense, and a
# failed retreat (retreat_incomplete, the mission's retreat_failed after its
# last retry, or its "Retreat software fault" error) drops it.
RETREAT_NOTE = "Spot retreated automatically — no human entered the space"
COMMAND_RETREAT_NOTE = "Spot retreated on command — no human entered the space"
RETREATING_NOTE = "Spot is retreating automatically — no human entered the space"
COMMAND_RETREATING_NOTE = "Spot is retreating on command — no human entered the space"
_DONE_NOTES = (RETREAT_NOTE, COMMAND_RETREAT_NOTE)
# A retreat_complete this soon after a flash annotates it; >= agent/mission.py
# RETREAT_TIMEOUT_S (240 s) so a long retreat's completion is never lost.
ALERT_ANNOTATE_S = 300.0
# Presenter-staged alarms (interface contract): an alarm is labelled staged
# only if an injection disclosure was received (the HUD's own, or the
# guard's staged_by_presenter list) AND payload/gas reports
# injected_pockets > 0 at the time. Either alone, or a simulator that
# reports no count, is a genuine alarm. A disclosure counts for this long
# (agent/mission.py remembers presenter events for 300 s too).
STAGED_WINDOW_S = 300.0
STAGED_LATE_S = 10.0        # an injection disclosed this soon AFTER a flash still stages it
# Injection disclosures from an allowlisted source: host/inject_gas.sh
# (presenter demo_inject), the agent's presenter tool (agent gas_injected),
# run_mission.py's harness (mission demo_inject). Clears and a presenter
# reset (which clears the pockets first) end the disclosure.
_INJECT_KIND = re.compile(r"inject", re.I)
_INJECT_CLEAR_KINDS = ("demo_clear", "gas_cleared", "sim_control", "demo_reset")
# agent/mission.py _retreat_worker's exception path: "Retreat software fault: ..."
_RETREAT_FAULT = re.compile(r"^\s*retreat\b", re.I)
AUTOMATIC_CAUSES = ("atmosphere", "telemetry")   # agent/mission.py SafetyGuard causes

# T3: payload/events carries no message authentication (none by design:
# owner decision; SROS2 is the gate before a real robot, M45), so the HUD
# makes provenance visible instead. These are the sources this sample's
# publishers declare. An event from any other source is buffered, shown
# and exported marked "unverified"; it may still flash an alarm (failing
# safe), but it never changes the permit, checklist, purge hold, probe
# table, staging, retreat note or session. "allowlisted" means only that
# the self-declared source is one of these, not that it was proven.
EVENT_SOURCES = (
    "mission",     # agent/mission.py, agent/run_mission.py (robot_client's default)
    "guard",       # agent/mission.py SafetyGuard and the retreat it runs
    "agent",       # agent/spot_agent.py replies and tool calls
    "presenter",   # host/inject_gas.sh and host/reset_spot.sh disclosures
    "operator",    # host/inject_gas.sh ventilation on/off
    "facade",      # sim/isaac_app.py's own audit records (sim/facade_guard.py)
)
LOCAL_SOURCE = "hud"   # records the HUD writes itself; never accepted from DDS
SOURCE_ALLOWLISTED, SOURCE_UNVERIFIED, SOURCE_LOCAL = "allowlisted", "unverified", "local"
UNVERIFIED_TITLE = "UNVERIFIED ALARM"
# T3/T23: the simulator facade publishes only these record kinds (what it did
# with a world control, a power or e-stop call, the cmd_vel publisher set). They
# report; they are not instructions, so a record under the facade's name is
# kept as a protected safety record but changes no HUD state (permit, alarm
# label, staging, probe, session); an alarm claim in one flashes as
# UNVERIFIED ALARM, since the facade never sends one. Any other kind under
# its name is unverified.
FACADE_SOURCE = "facade"
FACADE_KINDS = ("world_control", "facade_service", "facade_publishers")


def source_check(source, kind=None):
    """'allowlisted' for a source in EVENT_SOURCES (exact match), else
    'unverified'. Under the facade's name only FACADE_KINDS are allowlisted."""
    if source not in EVENT_SOURCES:
        return SOURCE_UNVERIFIED
    if source == FACADE_SOURCE and kind is not None and kind not in FACADE_KINDS:
        return SOURCE_UNVERIFIED
    return SOURCE_ALLOWLISTED


# M23: an atmosphere change the facade applied (its world_control record)
# should come with a disclosure from an allowlisted, non-facade publisher:
# host/inject_gas.sh (demo_inject, demo_clear, ventilation), host/reset_spot.sh
# (demo_reset, sent after its self_right, up to ~12 s after the controls),
# agent/mission.py (ventilation, sim_control, gas_cleared), the agent's
# presenter tools (gas_injected) and run_mission.py's harness (demo_inject).
# A change with no matching disclosure within DISCLOSURE_WINDOW_S, before or
# after it, is flagged: the HUD writes its own undisclosed_change record and
# counts it in the snapshot ("world"). One disclosure covers one change of
# each control it names. Nothing here is proof (T3): a host process can still
# send both a control and a matching disclosure.
ATMOSPHERE_CONTROLS = ("gas/inject", "gas/clear", "gas/ventilation")
DISCLOSURE_WINDOW_S = 15.0
WORLD_PENDING_MAX = 256     # changes awaiting a disclosure; an overflow is flagged at once
UNDISCLOSED_KIND = "undisclosed_change"
_CLEAR_DISCLOSURES = ("demo_clear", "gas_cleared", "sim_control", "demo_reset")


def world_change(ev):
    """``(control, params, publishers)`` for a facade record of an applied
    atmosphere change (ATMOSPHERE_CONTROLS), else None."""
    data = ev.get("data") or {}
    if (ev.get("source") != FACADE_SOURCE or ev.get("kind") != "world_control"
            or data.get("accepted") is not True or data.get("control") not in ATMOSPHERE_CONTROLS):
        return None
    params = data.get("params") if isinstance(data.get("params"), dict) else {}
    publishers = data.get("publishers") if isinstance(data.get("publishers"), list) else None
    return data["control"], params, publishers


def _same_number(a, b):
    a, b = _finite(a), _finite(b)
    return a is None or b is None or math.isclose(a, b, rel_tol=1e-3, abs_tol=1e-3)


def discloses(ev, control, params):
    """True if allowlisted event ``ev`` (not the facade's) discloses an
    applied ``control`` with ``params`` (M23). It must name the same change:
    the same gas for an injection, the same on/off and time constant for
    ventilation, a reset for a reset. The mission's purge progress lines
    (ventilation events with ``waited_s``) report, they do not disclose."""
    kind = (ev.get("kind") or "").lower()
    data = ev.get("data") or {}
    if ev.get("source") == FACADE_SOURCE or data.get("ok") is False:
        return False
    if control == "gas/inject":
        gas = data.get("gas")
        return (bool(_INJECT_KIND.search(kind)) and data.get("published") is not False
                and (not isinstance(gas, str) or gas.strip().lower() == params.get("gas"))
                and _same_number(data.get("ppm"), params.get("ppm")))
    if control == "gas/clear":
        return kind in _CLEAR_DISCLOSURES and data.get("cleared") is not False
    if control != "gas/ventilation":
        return False
    if params.get("reset") is True:
        return ((kind == "ventilation" and data.get("reset") is True)
                or (kind == "sim_control" and data.get("ventilation_reset") is not False)
                or (kind == "demo_reset" and data.get("reset_ventilation") is True))
    return (kind == "ventilation" and "waited_s" not in data and "reset" not in data
            and isinstance(data.get("on"), bool) and data["on"] is params.get("on")
            and _same_number(data.get("tau_s"), params.get("tau_s")))

# Scalar data keys the HUD derives state from; kept when big data is truncated.
_KEEP_ON_TRUNCATE = ("permit_state", "state", "to", "permit_id", "alarm", "auto_retreat",
                     "pass", "passed", "ok", "tool", "cause", "action", "gas", "value",
                     "purge_incomplete", "duration_s",
                     "o2_pct", "lel_pct", "h2s_ppm", "co_ppm") + tuple(
                         key for _k, _l, keys in PERMIT_CHECKLIST for key in keys)
# Small nested dicts kept on truncation: the Permit dict and the trigger reading.
_KEEP_NESTED_ON_TRUNCATE = ("permit", "reading")
_NESTED_KEEP_MAX_CHARS = 1500
# agent/robot_client.py publishes payload/events reliable + TRANSIENT_LOCAL,
# depth 100, so a (re)started HUD receives recent history on match. Events
# arriving this soon after subscribing are treated as that replay: they set
# the permit and fill the log but never fire the full-screen flash.
EVENT_HISTORY_DEPTH = 100
EVENT_REPLAY_WINDOW_S = 5.0
# An event is stamped with the latest payload/gas sample only if it is this fresh.
RECEIPT_GAS_MAX_AGE_S = 5.0
# A facade probe reply this soon after the mission's aggregated probe event
# is a late per-request answer and must not replace the verdict table.
PROBE_EVENT_HOLD_S = 3.0

# Purge hold (agent/mission.py purge gate): the mission's purge_hold event
# ({fraction_remaining, purge_target, eta_s}) is the single source of truth
# for the amber PURGE HOLD panel; an older mission's purge_incomplete event
# (same numbers as fraction_remaining / purge_target_fraction / wait_s) is
# accepted as a fallback. The hold ends when a re-test starts, the permit
# leaves PRE_ENTRY_TESTING, a new permit opens, or the demo is reset.
PURGE_HOLD_KIND = "purge_hold"
PURGE_FALLBACK_KIND = "purge_incomplete"


def _is_presenter_reset(ev):
    """True for host/reset_spot.sh's disclosure event (presenter /
    demo_reset): the HUD starts a fresh run (HudState._new_session),
    unless an alarm or retreat is under way (HudState._reset_blocker)."""
    return ((ev.get("kind") or "").lower() == "demo_reset"
            and ev.get("source") == "presenter")

# Kind keywords that fire the full-screen flash (see module docstring).
_ALERT_WORDS = re.compile(r"alarm|abort|e[-_]?stop|retreat|hazard|excursion|conflict", re.I)
# Matched after removing "incomplete": the mission's "retreat_incomplete" is a
# failure and must still flash. No lookbehind so app.js can use the same rule
# on browsers without regex lookbehind support.
_CLEAR_WORDS = re.compile(r"complete|done|clear|resolved|finished|end\b|ok\b|release|reset",
                          re.I)
_INCOMPLETE = re.compile(r"incomplete", re.I)

# Copied from sim/build_scene.py (last-resort fallback only; the latched
# payload/site topic, then agent/site_defaults.json, take precedence).
BUILTIN_SITE = {
    "spawn_xy": [14.0, 0.0],
    "spawn_yaw": math.pi,
    "entry_xy": [7.0, 0.0],
    "door_xy": [6.0, 0.0],
    "door_normal_yaw": 0.0,
    "interior_waypoints": [[4.6, 0.0], [2.5, 1.6], [0.0, 0.0], [-2.0, -1.7], [-4.8, 0.0]],
    "tank_center_xy": [0.0, 0.0],
    "tank_radius": 6.0,
}

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SITE_DEFAULT_CANDIDATES = (
    os.path.join(_REPO_ROOT, "agent", "site_defaults.json"),
    "/workspace/agent/site_defaults.json",
)


class BridgeUnavailable(RuntimeError):
    """rclpy (or a ROS message package) cannot be imported in this process."""


# ---------------------------------------------------------------------------
# Small pure helpers (unit-tested directly)
# ---------------------------------------------------------------------------


def sanitize(value, depth=0):
    """Make a parsed JSON value safe to re-serialize for browsers.

    Python's json accepts and emits NaN/Infinity, but browsers' JSON.parse
    rejects them -- one NaN gas reading would silently freeze the HUD -- so
    non-finite floats become None. Also bounds nesting and sizes.
    """
    if depth > 8:
        return None
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k)[:64]: sanitize(v, depth + 1) for k, v in list(value.items())[:64]}
    if isinstance(value, (list, tuple)):
        return [sanitize(v, depth + 1) for v in value[:512]]
    return str(value)


def _finite(value, default=None):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def _xy(value, name):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError("site.%s must be [x, y], got %r" % (name, value))
    x, y = _finite(value[0]), _finite(value[1])
    if x is None or y is None:
        raise ValueError("site.%s has non-numeric coordinates: %r" % (name, value))
    return [x, y]


def guard_heartbeat(raw):
    """The contract fields of a payload/guard heartbeat, or None if any is
    missing or malformed. A malformed heartbeat is not proof that the guard
    is alive (fail closed)."""
    if not isinstance(raw, dict):
        return None
    state, inside, seq, wall = (raw.get(k) for k in ("state", "robot_inside", "seq", "wall_t"))
    if state not in GUARD_STATES or not isinstance(inside, bool):
        return None
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        return None
    if isinstance(wall, bool) or not isinstance(wall, (int, float)) or not math.isfinite(wall):
        return None
    return {"state": state, "robot_inside": inside, "seq": seq, "wall_t": float(wall)}


def normalize_site(raw):
    """Validate a site dict (topic payload or defaults file); raise ValueError.

    Accepts build_scene's ``doorway_xy`` as an alias for ``door_xy`` so a
    defaults file copied straight from build_scene output still works.
    """
    if not isinstance(raw, dict):
        raise ValueError("site must be a JSON object")
    door = raw.get("door_xy") if raw.get("door_xy") is not None else raw.get("doorway_xy")
    site = {
        "tank_center_xy": _xy(raw.get("tank_center_xy"), "tank_center_xy"),
        "tank_radius": _finite(raw.get("tank_radius")),
        "entry_xy": _xy(raw.get("entry_xy"), "entry_xy"),
        "door_xy": _xy(door, "door_xy"),
        "interior_waypoints": [
            _xy(p, "interior_waypoints[%d]" % i)
            for i, p in enumerate(raw.get("interior_waypoints") or [])
        ],
    }
    if site["tank_radius"] is None or site["tank_radius"] <= 0:
        raise ValueError("site.tank_radius must be a positive number")
    if raw.get("spawn_xy") is not None:
        site["spawn_xy"] = _xy(raw["spawn_xy"], "spawn_xy")
    for key in ("spawn_yaw", "door_normal_yaw"):
        if _finite(raw.get(key)) is not None:
            site[key] = _finite(raw.get(key))
    return site


def load_site_defaults(path=None):
    """Return (site, source_label): defaults file if readable, else builtin."""
    candidates = [path] if path else list(SITE_DEFAULT_CANDIDATES)
    for candidate in candidates:
        if not candidate or not os.path.isfile(candidate):
            continue
        try:
            with open(candidate, "r", encoding="utf-8") as handle:
                return normalize_site(json.load(handle)), "defaults"
        except (OSError, ValueError) as error:
            log.warning("site defaults %s unusable (%s); trying next", candidate, error)
    return normalize_site(BUILTIN_SITE), "builtin"


def gas_alarm_text(sample):
    """Human-readable list of gases in alarm, e.g. 'H2S 12.6 ppm (>= 10 ppm)'."""
    alarms = sample.get("alarms") if isinstance(sample.get("alarms"), dict) else {}
    parts = []
    for gas, spec in OSHA_THRESHOLDS.items():
        value = _finite(sample.get(spec["key"]))
        if not alarms.get(gas) or value is None:
            continue
        if gas == "o2":
            limit = "%.1f-%.1f %%" % (spec["low"], spec["high"])
            parts.append("O2 %.1f %% (safe %s)" % (value, limit))
        else:
            parts.append("%s %.1f %s (>= %g %s)" % (
                gas.upper(), value, spec["unit"], spec["alarm_at"], spec["unit"]))
    return "; ".join(parts) if parts else "atmospheric alarm"


def gas_exceeds(gas, value):
    """True if ``value`` is past the OSHA App. E limit for ``gas``."""
    spec = OSHA_THRESHOLDS[gas]
    if gas == "o2":
        return value < spec["low"] or value > spec["high"]
    return value >= spec["alarm_at"]


_GAS_ALIASES = {gas: (OSHA_THRESHOLDS[gas]["key"], gas) for gas in GAS_ORDER}
_NESTED_GAS_KEYS = ("reading", "gas", "sample", "values", "concentrations")


def gas_values(obj):
    """{o2_pct, lel_pct, h2s_ppm, co_ppm} found in ``obj`` (flat, or nested
    under reading/gas/sample/values), plus ``alarms`` flags if present.
    Returns {} when no gas value is present. Accepts o2 / o2_pct aliases."""
    if not isinstance(obj, dict):
        return {}
    src = obj
    if not any(k in obj for keys in _GAS_ALIASES.values() for k in keys
               if not isinstance(obj.get(k), (dict, str))):
        for key in _NESTED_GAS_KEYS:
            if isinstance(obj.get(key), dict):
                src = obj[key]
                break
    out = {}
    for gas, keys in _GAS_ALIASES.items():
        for key in keys:
            value = _finite(src.get(key)) if not isinstance(src.get(key), bool) else None
            if value is not None:
                out[OSHA_THRESHOLDS[gas]["key"]] = value
                break
    if out:
        flags = src.get("alarms") if isinstance(src.get("alarms"), dict) else obj.get("alarms")
        if isinstance(flags, dict):
            out["alarms"] = {g: bool(flags.get(g)) for g in GAS_ORDER}
    return out


def _fmt_gas(gas, value):
    spec = OSHA_THRESHOLDS[gas]
    if gas == "o2":
        return "O2 %.1f %% (safe %.1f-%.1f %%)" % (value, spec["low"], spec["high"])
    return "%s %.1f %s (limit < %g %s)" % (gas.upper(), value, spec["unit"],
                                           spec["alarm_at"], spec["unit"])


def trigger_reading_text(sample):
    """The gases over their limit in ``sample`` (threshold OR alarm flag),
    e.g. 'H2S 25.3 ppm (limit < 10 ppm)'. None if nothing is over."""
    values = gas_values(sample)
    flags = values.get("alarms") or {}
    parts = []
    for gas in GAS_ORDER:
        value = values.get(OSHA_THRESHOLDS[gas]["key"])
        if value is not None and (gas_exceeds(gas, value) or flags.get(gas)):
            parts.append(_fmt_gas(gas, value))
    return "; ".join(parts) or None


def event_gas_values(data):
    """Gas values an event itself carries (monitor readings, alarm/retreat
    'reading', or an atmos_test {gas, value} entry); {} if none."""
    if not isinstance(data, dict):
        return {}
    values = gas_values(data)
    values.pop("alarms", None)
    if not values and data.get("gas") in GAS_ORDER and _finite(data.get("value")) is not None:
        values = {OSHA_THRESHOLDS[data["gas"]]["key"]: _finite(data["value"])}
    return values


# -- ventilation (payload/gas "ventilation") ---------------------------------

_VENT_ACTIVE_KEYS = ("active", "on", "running", "enabled", "forced")
_VENT_MODE_KEYS = ("mode", "state", "status", "phase")
_VENT_FRACTION_KEYS = ("fraction_remaining", "remaining_fraction", "fraction", "remaining",
                       "pocket_fraction", "pocket_scale", "scale", "amplitude", "level", "factor")
_VENT_PCT_KEYS = ("percent_remaining", "remaining_pct", "pct_remaining")
_VENT_OFF = re.compile(r"raw|unvent|^off$|none|idle|stop|inactive|natural|static|disabled", re.I)
_VENT_ON = re.compile(r"vent|forced|^on$|running|active|purg|blow|decay|clearing", re.I)


def _fraction(value, percent=False):
    f = _finite(value) if not isinstance(value, bool) else None
    if f is None:
        return None
    if percent or 1.0 < f <= 100.0:
        f /= 100.0
    return max(0.0, min(1.0, f))


def normalize_ventilation(raw):
    """payload/gas ``ventilation`` -> {"state": "on"|"off"|"unknown", "mode",
    "fraction_remaining" (0..1 of the built-in pocket strength), "tau_s",
    "residual", "elapsed_s"}; None if the field is absent.

    Tolerant on purpose (the sim module owns the exact shape): a bool, a mode
    string ('raw', 'ventilating', ...), a bare fraction, or a dict with
    any of active/mode/fraction_remaining/tau_s/residual/elapsed_s.
    """
    if raw is None:
        return None
    out = {"state": "unknown", "mode": None, "fraction_remaining": None, "tau_s": None,
           "residual": None, "elapsed_s": None}
    active = None
    if isinstance(raw, bool):
        active = raw
    elif isinstance(raw, str):
        out["mode"] = raw[:32]
    elif isinstance(raw, (int, float)):
        out["fraction_remaining"] = _fraction(raw)
    elif isinstance(raw, dict):
        for key in _VENT_ACTIVE_KEYS:
            if isinstance(raw.get(key), bool):
                active = raw[key]
                break
        for key in _VENT_MODE_KEYS:
            if isinstance(raw.get(key), str):
                out["mode"] = raw[key][:32]
                break
        for key in _VENT_FRACTION_KEYS:
            if _fraction(raw.get(key)) is not None:
                out["fraction_remaining"] = _fraction(raw.get(key))
                break
        else:
            for key in _VENT_PCT_KEYS:
                if _fraction(raw.get(key), percent=True) is not None:
                    out["fraction_remaining"] = _fraction(raw.get(key), percent=True)
                    break
        out["tau_s"] = _finite(raw.get("tau_s", raw.get("tau")))
        out["residual"] = _fraction(raw.get("residual", raw.get("residual_fraction")))
        out["elapsed_s"] = _finite(raw.get("elapsed_s", raw.get("since_s", raw.get("elapsed"))))
    else:
        return None
    if active is not None:
        out["state"] = "on" if active else "off"
    elif out["mode"]:
        out["state"] = ("off" if _VENT_OFF.search(out["mode"]) else
                        "on" if _VENT_ON.search(out["mode"]) else "unknown")
    elif out["fraction_remaining"] is not None and out["fraction_remaining"] < 0.999:
        out["state"] = "on"
    return out


def purge_hold_from(kind, data):
    """{fraction_remaining, purge_target, eta_s, percent_remaining,
    ventilation_on} from a mission ``purge_hold`` event (or, for a mission
    that predates it, ``purge_incomplete``: fraction_remaining /
    purge_target_fraction / wait_s); None for any other event. Fractions
    are 0..1 of the unventilated contamination; values are passed through
    unrounded so the HUD shows exactly what the mission (and the agent) use."""
    if kind not in (PURGE_HOLD_KIND, PURGE_FALLBACK_KIND) or not isinstance(data, dict):
        return None

    def frac(key):
        value = data.get(key)
        f = _finite(value) if not isinstance(value, bool) else None
        return None if f is None or not 0.0 <= f <= 1.0 else f

    target = frac("purge_target")
    if target is None:
        target = frac("purge_target_fraction")
    remaining = frac("fraction_remaining")
    if remaining is None and kind == PURGE_FALLBACK_KIND:
        remaining = frac("last_seen_fraction")
        if remaining is None:
            remaining = frac("fraction_estimate")
    if target is None and remaining is None:
        return None
    eta = data.get("eta_s", data.get("wait_s"))
    eta = _finite(eta) if not isinstance(eta, bool) else None
    vent = data.get("ventilation") if isinstance(data.get("ventilation"), dict) else {}
    on = data.get("ventilation_on", vent.get("on"))
    # The mission's own rounded percentage wins (it is the number in its
    # purge message and the agent's answer): re-rounding the 4-digit
    # fraction can differ by 0.1 (0.08251 -> 0.0825 -> 8.2, mission 8.3).
    pct = data.get("percent_remaining")
    pct = _finite(pct) if not isinstance(pct, bool) else None
    if pct is None or not 0.0 <= pct <= 100.0:
        pct = round(100.0 * remaining, 1) if remaining is not None else None
    source = data.get("fraction_source")
    # Not the simulator's current reading (a last report or the mission's
    # commanded-time estimate): shown, labelled, and never what authorizes
    # entry (T12). An older mission that sends neither key reads as current.
    estimate = (source in ("last_seen", "estimate")
                or data.get("current_feed") is False)
    return {"fraction_remaining": remaining, "purge_target": target,
            "eta_s": eta if eta is None or eta >= 0 else None,
            "percent_remaining": pct,
            "ventilation_on": on if isinstance(on, bool) else None,
            "estimate": estimate}


# -- door-mouth probe (payload/gas/probe_result) -----------------------------

_HEIGHT_KEYS = ("z", "z_m", "height", "height_m", "h", "probe_z")
_ROW_LIST_KEYS = ("samples", "readings", "heights", "levels", "by_height", "profile",
                  "results", "rows")
_XY_KEYS = ("xy", "at_xy", "probe_xy", "position", "pos")
_INSIDE_KEYS = ("inside_m", "insertion_m", "depth_m", "reach_m", "offset_m")
_VERDICT_KEYS = ("passed", "pass", "ok")
_HEIGHT_IN_KEY = re.compile(r"-?\d+(?:\.\d+)?")
MAX_PROBE_ROWS = 8


def _point_of(row):
    """(x, y, z) from the facade's ``"point": [x, y, z]``, or None."""
    point = row.get("point")
    if isinstance(point, (list, tuple)) and len(point) == 3:
        xyz = tuple(_finite(v) if not isinstance(v, bool) else None for v in point)
        if None not in xyz:
            return xyz
    return None


def _height_of(row, fallback=None):
    for key in _HEIGHT_KEYS:
        value = _finite(row.get(key)) if not isinstance(row.get(key), bool) else None
        if value is not None:
            return value
    point = _point_of(row)
    return point[2] if point is not None else fallback


def _probe_rows(raw):
    """[(height or None, row dict)] from the accepted layouts: a list of
    rows, a dict keyed by height ('0.2' / '0.2m'), or parallel lists
    (heights=[...] + readings=[...])."""
    heights_list = raw.get("heights") if isinstance(raw.get("heights"), list) else None
    if heights_list and not all(isinstance(h, (int, float)) and not isinstance(h, bool)
                                for h in heights_list):
        heights_list = None
    for key in _ROW_LIST_KEYS:
        value = raw.get(key)
        if isinstance(value, list) and value and all(isinstance(r, dict) for r in value):
            fallback = heights_list if heights_list and len(heights_list) == len(value) else None
            return [(_height_of(r, _finite(fallback[i]) if fallback else None), r)
                    for i, r in enumerate(value)]
        if isinstance(value, dict) and value and all(isinstance(r, dict) for r in value.values()):
            rows = []
            for name, r in value.items():
                match = _HEIGHT_IN_KEY.search(str(name))
                rows.append((_height_of(r, float(match.group()) if match else None), r))
            return rows
    return []


def _worst(gas, values):
    if gas == "o2":  # whichever side is closer to (or past) its limit
        spec = OSHA_THRESHOLDS["o2"]
        return min(values, key=lambda v: min(v - spec["low"], spec["high"] - v))
    return max(values)


def _row_xy(row):
    """(x, y) of a probe row: its "point", or its own x/y keys; else None."""
    point = _point_of(row)
    if point is not None:
        return point[0], point[1]
    x, y = _finite(row.get("x")), _finite(row.get("y"))
    return (x, y) if x is not None and y is not None else None


def _row_inside(row):
    for key in _INSIDE_KEYS:
        value = row.get(key)
        if not isinstance(value, bool) and _finite(value) is not None:
            return _finite(value)
    return None


def _sort_probe_rows(rows):
    """Ascending height, then nearest depth first (unknown depth last)."""
    rows.sort(key=lambda r: (r["z"] is None, r["z"] if r["z"] is not None else 0.0,
                             r["inside_m"] is None, r["inside_m"] or 0.0))


def normalize_probe(raw):
    """A probe result -> {"rows": [{"z", "inside_m", "x", "y", o2_pct..,
    "alarms": {..}}] (ascending height, then depth; one row per sampled
    point), "worst": {o2_pct.., "alarms"}, "any_alarm", "complete",
    "verdict" (publisher's bool or None), "xy", "points_xy", "inside_m"
    (nearest depth), "depths_m", "t", "error"};
    None if there are neither per-height gas values nor an error. Alarm
    colouring is the HUD's own check against OSHA App. E, OR'ed with any
    per-gas alarm flags in the payload. A single flat reading is accepted as
    a one-row probe. The facade's shape (sim/isaac_app.py) is
    {"id", "samples": [{"point": [x, y, z], o2_pct.., "alarms"}], "sim_time"}
    or {"id", "error", "samples": []} for a rejected request."""
    if not isinstance(raw, dict):
        return None
    pairs = _probe_rows(raw)
    if not pairs and gas_values(raw):
        pairs = [(_height_of(raw), raw)]
    points = [pt for pt in (_point_of(r) for _h, r in pairs) if pt is not None]
    rows = []
    for height, row in pairs[:MAX_PROBE_ROWS]:
        values = gas_values(row)
        flags = values.pop("alarms", {})
        if not values:
            continue
        xy = _row_xy(row)
        inside = _row_inside(row)
        out = {"z": round(height, 3) if height is not None else None,
               "inside_m": round(inside, 2) if inside is not None else None,
               "x": round(xy[0], 3) if xy else None, "y": round(xy[1], 3) if xy else None}
        alarms = {}
        for gas in GAS_ORDER:
            key = OSHA_THRESHOLDS[gas]["key"]
            value = values.get(key)
            out[key] = round(value, 3) if value is not None else None
            alarms[gas] = value is not None and (gas_exceeds(gas, value) or bool(flags.get(gas)))
        out["alarms"] = alarms
        rows.append(out)
    error = raw.get("error") if isinstance(raw.get("error"), str) and raw["error"].strip() else None
    t = _finite(raw.get("t", raw.get("sim_time")))
    if not rows:
        if error is None:
            return None
        return {"rows": [], "worst": None, "any_alarm": False, "complete": False,
                "verdict": None, "xy": None, "points_xy": [], "inside_m": None,
                "depths_m": None, "t": t, "error": error[:200]}
    _sort_probe_rows(rows)
    worst, worst_alarms = {}, {}
    for gas in GAS_ORDER:
        key = OSHA_THRESHOLDS[gas]["key"]
        values = [r[key] for r in rows if r[key] is not None]
        worst[key] = _worst(gas, values) if values else None
        worst_alarms[gas] = any(r["alarms"][gas] for r in rows)
    worst["alarms"] = worst_alarms
    verdict = None
    for key in _VERDICT_KEYS:
        if isinstance(raw.get(key), bool):
            verdict = raw[key]
            break
    xy = None
    for key in _XY_KEYS:
        value = raw.get(key)
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            x, y = _finite(value[0]), _finite(value[1])
            if x is not None and y is not None:
                xy = [round(x, 3), round(y, 3)]
                break
        if isinstance(value, dict) and _finite(value.get("x")) is not None \
                and _finite(value.get("y")) is not None:
            xy = [round(_finite(value["x"]), 3), round(_finite(value["y"]), 3)]
            break
    if xy is None and points:   # facade: centroid of the sampled points
        xy = [round(sum(p[0] for p in points) / len(points), 3),
              round(sum(p[1] for p in points) / len(points), 3)]
    inside = _row_inside(raw)
    depths = sorted({r["inside_m"] for r in rows if r["inside_m"] is not None})
    if isinstance(raw.get("depths_m"), list):
        listed = [_finite(d) for d in raw["depths_m"] if not isinstance(d, bool)]
        depths = sorted({round(d, 2) for d in listed if d is not None} | set(depths))
    if inside is None and depths:
        inside = depths[0]
    points_xy = []
    for r in rows:
        if r["x"] is not None and [r["x"], r["y"]] not in points_xy:
            points_xy.append([r["x"], r["y"]])
    return {
        "rows": rows,
        "worst": worst,
        "any_alarm": any(worst_alarms.values()),
        "complete": all(worst[OSHA_THRESHOLDS[g]["key"]] is not None for g in GAS_ORDER),
        "verdict": verdict,
        "xy": xy,
        "points_xy": points_xy,
        "inside_m": inside,
        "depths_m": depths or None,
        "t": t,
        "error": error[:200] if error else None,
    }


def inside_distance(xy, site):
    """How far ``xy`` is inside the doorway plane (m, positive = toward the
    tank centre), or None without a site / if the point is outside."""
    if not xy or not site:
        return None
    (dx, dy), (cx, cy) = site["door_xy"], site["tank_center_xy"]
    nx, ny = cx - dx, cy - dy
    norm = math.hypot(nx, ny)
    if norm < 1e-6:
        return None
    depth = ((xy[0] - dx) * nx + (xy[1] - dy) * ny) / norm
    return round(depth, 2) if depth > 0 else None


# -- permit checklist --------------------------------------------------------

# Checklist row -> the agent/mission.py Permit field named in its ``missing``.
_MISSION_FIELD = {"isolation": "isolation_confirmed", "loto": "loto_confirmed",
                  "attendant": "attendant", "rescue_plan": "rescue_plan"}


def permit_details(ev):
    """{field: raw value} for the permit fields an event carries (only keys
    actually present), plus 'permit_id'. Reads data.permit (the mission's
    Permit dict, any event kind) and, for kinds containing 'permit', the
    top level of data (which wins)."""
    data = ev.get("data") if isinstance(ev.get("data"), dict) else {}
    sources = []
    if isinstance(data.get("permit"), dict):
        sources.append((data["permit"], ("permit_id", "id")))
    if "permit" in (ev.get("kind") or "").lower():
        sources.append((data, ("permit_id",)))
    found = {}
    nested = data.get("permit") if isinstance(data.get("permit"), dict) else None
    if nested is not None and isinstance(nested.get("missing"), list):
        # The mission's own gate verdict: the checklist must never disagree
        # with what actually lets Spot approach the entry.
        found["_gate"] = {"missing": [str(m) for m in nested["missing"]][:16],
                          "complete": nested["complete"]
                          if isinstance(nested.get("complete"), bool) else None}
    for src, id_keys in sources:
        for key in id_keys:
            if isinstance(src.get(key), (str, int)) and not isinstance(src.get(key), bool) \
                    and str(src[key]).strip():
                found["permit_id"] = str(src[key])[:64]
                break
        for field, _label, keys in PERMIT_CHECKLIST:
            for key in keys:
                if key in src:
                    found[field] = src[key]
                    break
    return found


def checklist_item(value, present=True):
    """One checklist row -> {"ok": True|False|None, "text": str|None}.
    Absent = unknown (None, rendered '—'); null/False/'' = not set (False)."""
    if not present:
        return {"ok": None, "text": None}
    if isinstance(value, dict):
        ok = next((value[k] for k in ("confirmed", "ok", "done", "verified", "value")
                   if isinstance(value.get(k), bool)), None)
        text = next((value[k] for k in ("name", "by", "text", "description", "plan", "who")
                     if isinstance(value.get(k), str) and value[k].strip()), None)
        if ok is None:
            ok = text is not None
        return {"ok": bool(ok), "text": (text or ("Confirmed" if ok else "Not confirmed"))[:80]}
    if value is None:
        return {"ok": False, "text": "Not set"}
    if isinstance(value, bool):
        return {"ok": value, "text": "Confirmed" if value else "Not confirmed"}
    if isinstance(value, (int, float)):
        return {"ok": bool(value), "text": "Confirmed" if value else "Not confirmed"}
    text = str(value).strip()
    if text.lower() in _NEGATIVE_TEXT:
        return {"ok": False, "text": text or "Not set"}
    return {"ok": True, "text": text[:80]}


def build_checklist(fields, gate=None):
    """Checklist rows (+ summary) from the stored raw permit fields.

    ``gate`` ({"missing": [...], "complete": bool|None}, from the mission's
    Permit dict) is authoritative when present: a row is a check mark iff
    the mission does not list its field as missing, so the HUD never shows a
    cross while the mission lets Spot approach (or the reverse)."""
    rows = []
    for key, label, _keys in PERMIT_CHECKLIST:
        item = checklist_item(fields.get(key), key in fields)
        if gate is not None:
            ok = _MISSION_FIELD[key] not in gate["missing"]
            if item["ok"] is not ok:
                text = item["text"] if ok and item["text"] not in (None, "Not set",
                                                                   "Not confirmed") else None
                item = {"ok": ok, "text": text or ("Confirmed" if ok else "Not set")}
        rows.append(dict(item, key=key, label=label))
    known = [r["ok"] for r in rows if r["ok"] is not None]
    complete = bool(known) and len(known) == len(rows) and all(known)
    if gate is not None and gate.get("complete") is not None:
        complete = gate["complete"]
    return rows, {"confirmed": sum(1 for ok in known if ok), "total": len(rows),
                  "complete": complete}


def compute_alarms(sample):
    """OSHA App. E alarms for a sample dict (used by FakeBridge only)."""
    o2 = sample["o2_pct"]
    alarms = {
        "o2": o2 < OSHA_THRESHOLDS["o2"]["low"] or o2 > OSHA_THRESHOLDS["o2"]["high"],
        "lel": sample["lel_pct"] >= OSHA_THRESHOLDS["lel"]["alarm_at"],
        "h2s": sample["h2s_ppm"] >= OSHA_THRESHOLDS["h2s"]["alarm_at"],
        "co": sample["co_ppm"] >= OSHA_THRESHOLDS["co"]["alarm_at"],
    }
    alarms["any"] = any(alarms.values())
    return alarms


# sensor_msgs/Image encoding -> (PIL mode, PIL raw decoder mode, bytes/pixel)
_ENCODINGS = {
    "rgb8": ("RGB", "RGB", 3),
    "bgr8": ("RGB", "BGR", 3),
    "rgba8": ("RGB", "RGBX", 4),
    "bgra8": ("RGB", "BGRX", 4),
    "mono8": ("L", "L", 1),
}
_BILINEAR = getattr(Image, "Resampling", Image).BILINEAR
# T32 (M32): the largest camera frame the HUD decodes. Any DDS publisher can
# send one, and Pillow decodes, resizes and re-encodes it in the HUD
# container, so a frame is checked before Pillow sees it. The simulator
# publishes 640x480 rgb8 (sim/sensors.py create_spot_cameras, 921,600 bytes);
# these caps leave room for a 1920x1080 frame at 4 bytes a pixel with some
# row padding. A refused frame is counted ("rejected", camera).
CAMERA_MAX_WIDTH = 1920
CAMERA_MAX_HEIGHT = 1080
CAMERA_MAX_BYTES = 8 * 1024 * 1024


def camera_frame_error(width, height, encoding, step, nbytes):
    """Why a sensor_msgs/Image cannot be decoded or is over the caps, or None.

    Cheap (no pixel is read): used in the ROS callback before the frame is
    queued, and again by :func:`encode_ros_image`."""
    spec = _ENCODINGS.get(str(encoding or "").lower())
    if spec is None:
        return "unsupported image encoding %r" % (str(encoding)[:32],)
    try:
        width, height, step, nbytes = int(width), int(height), int(step), int(nbytes)
    except (TypeError, ValueError):
        return "non-integer image size"
    if not (0 < width <= CAMERA_MAX_WIDTH and 0 < height <= CAMERA_MAX_HEIGHT):
        return "image size %dx%d outside 1..%dx%d" % (width, height, CAMERA_MAX_WIDTH,
                                                       CAMERA_MAX_HEIGHT)
    if nbytes > CAMERA_MAX_BYTES:
        return "image buffer %d bytes > %d" % (nbytes, CAMERA_MAX_BYTES)
    row = width * spec[2]
    step = step or row
    if step < row:
        return "step %d < row bytes %d" % (step, row)
    if step * height > nbytes:
        return "image buffer %d bytes < %d expected" % (nbytes, step * height)
    return None


def encode_ros_image(data, width, height, encoding, step=0, max_width=800, quality=70):
    """sensor_msgs/Image fields -> (jpeg_bytes, out_width, out_height).

    Honors ``step`` (row stride, which may include padding) and downsamples
    to ``max_width`` first: a 1280x720 RTX frame at full size would cost the
    SSM tunnel ~150 KB per frame for no visible gain on a HUD tile.
    Raises ValueError for unsupported encodings, truncated buffers and
    frames over the caps (:func:`camera_frame_error`, T32).
    """
    error = camera_frame_error(width, height, encoding, step, len(data))
    if error:
        raise ValueError(error)
    mode, rawmode, bpp = _ENCODINGS[str(encoding).lower()]
    width, height = int(width), int(height)
    step = int(step) or width * bpp
    img = Image.frombytes(mode, (width, height), data, "raw", rawmode, step, 1)
    if max_width and width > max_width:
        img = img.resize((int(max_width), max(1, round(height * max_width / width))), _BILINEAR)
    out = io.BytesIO()
    img.save(out, format="JPEG", quality=int(quality))
    return out.getvalue(), img.width, img.height


class _RateLimitedLog:
    """Log a recurring problem at most once per ``period`` s per key.

    A bad publisher can emit 50 malformed messages a second; without this the
    one useful line scrolls away under thousands of copies.
    """

    def __init__(self, period=10.0, logger=None):
        self._period = period
        self._last = {}
        self._lock = threading.Lock()
        self._log = logger or log

    def warning(self, key, msg, *args):
        now = time.monotonic()
        with self._lock:
            if now - self._last.get(key, -1e9) < self._period:
                return
            self._last[key] = now
        self._log.warning(msg, *args)


class _TokenBucket:
    """Per-topic ingest limit (T42): ``rate`` messages/s sustained, bursts up
    to ``burst``. Used from the one executor thread."""

    def __init__(self, rate, burst, clock=time.monotonic):
        self._rate, self._burst, self._clock = float(rate), float(burst), clock
        self._tokens, self._t = float(burst), clock()

    def take(self):
        now = self._clock()
        self._tokens = min(self._burst, self._tokens + (now - self._t) * self._rate)
        self._t = now
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False


# ---------------------------------------------------------------------------
# HudState
# ---------------------------------------------------------------------------


class HudState:
    """Thread-safe latest-value store + event ring buffer.

    Producers (ROS executor thread, camera worker, FakeBridge thread) call
    ``ingest_*``; the asyncio server reads ``snapshot()`` / ``events_since()``.
    Stored dicts are replaced wholesale, never mutated, so snapshots can share
    references without copying. Listeners are notified (outside the lock) on
    every new event or alert so the WebSocket can push them immediately.
    """

    def __init__(self, mode="ros", wall_clock=time.time, monotonic=time.monotonic):
        self._wall = wall_clock
        self._mono = monotonic
        self._lock = threading.Lock()
        self._mode = mode
        self._bridge_ok = False
        self._bridge_note = None
        self._status = None
        self._gas = None
        self._gas_alarm = False
        self._ventilation = None
        self._probe = None
        self._pose = None
        self._site = None
        self._trail = collections.deque(maxlen=TRAIL_MAX_POINTS)
        self._trail_epoch = 0
        self._buckets = {}         # (record class, source) -> bounded deque (T42)
        self._event_seq = 0
        self._dropped = collections.Counter()    # topic -> over the ingest limit (RosBridge)
        self._rejected = collections.Counter()   # topic -> malformed or oversize
        self._evicted = collections.Counter()    # buffer -> pushed out by newer records
        self._permit_fields = {}   # raw checklist values seen for the current permit
        self._permit_gate = None   # the mission's missing/complete verdict (authoritative)
        self._purge = None         # purge hold from the mission's purge_hold event (see purge_hold_from)
        checklist, summary = build_checklist({})
        self._permit = {"state": "NOT_STARTED", "since_wall": None, "permit_id": None,
                        "message": None, "checklist": checklist, "checklist_summary": summary}
        self._alert = {"seq": 0, "title": None, "reason": None, "source": None,
                       "t_wall": None, "reading": None, "note": None, "reaction_s": None,
                       "staged": False, "unverified": False}
        self._retreat_mono = None  # a retreat started (allowlisted event) and has not ended
        self._alert_claim = False  # a signal of the current incident claimed a disclosure
        self._inject_mono = None   # latest gas-injection disclosure (allowlisted source)
        self._injected = None      # payload/gas injected_pockets (None = not reported)
        self._alert_touch = None   # monotonic time of the latest alert signal
        self._alert_reading_src = None   # 'gas' | 'alarm' | 'event' (see _raise_alert)
        self._estopped = None      # last payload/status estopped (None = unknown)
        self._camera = None  # (jpeg, width, height)
        self._camera_seq = 0
        self._rx = {k: None for k in ("status", "gas", "odometry", "events", "site", "camera",
                                      "probe", "guard")}
        self._heartbeat = None     # last valid payload/guard heartbeat (T16)
        # M23: applied atmosphere changes awaiting a disclosure, the recent
        # disclosures, and how many changes went undisclosed (the last one).
        self._world_pending = collections.deque()
        self._world_overflow = []   # pushed out of a full pending list: flagged at once
        self._disclosures = collections.deque(maxlen=WORLD_PENDING_MAX)
        self._world_undisclosed = 0
        self._world_last = None
        self._listeners = {}
        self._listener_ids = itertools.count(1)
        self._warn = _RateLimitedLog()

    # -- listeners ---------------------------------------------------------

    def add_listener(self, callback):
        """Register ``callback()`` (no args) for new events/alerts; returns a token."""
        with self._lock:
            token = next(self._listener_ids)
            self._listeners[token] = callback
        return token

    def remove_listener(self, token):
        with self._lock:
            self._listeners.pop(token, None)

    def _notify(self):
        with self._lock:
            callbacks = list(self._listeners.values())
        for callback in callbacks:
            try:
                callback()
            except Exception:  # a dead client's loop must not break ingest
                pass

    # -- bridge metadata ----------------------------------------------------

    def set_bridge_info(self, ok, note=None):
        with self._lock:
            self._bridge_ok = bool(ok)
            self._bridge_note = note

    # -- ingest (called from producer threads) -----------------------------

    def _parse(self, topic, text):
        if not isinstance(text, str) or len(text) > MAX_JSON_CHARS:
            self._warn.warning(topic + ":size", "%s: payload missing or > %d chars; dropped",
                               topic, MAX_JSON_CHARS)
            self.count_rejected(topic)
            return None
        try:
            return sanitize(json.loads(text))
        except ValueError as error:
            self._warn.warning(topic + ":json", "%s: invalid JSON (%s); dropped", topic, error)
            self.count_rejected(topic)
            return None

    def count_dropped(self, topic):
        """One message over ``topic``'s ingest limit was dropped (T42)."""
        with self._lock:
            self._dropped[topic] += 1

    def count_rejected(self, topic):
        with self._lock:
            self._rejected[topic] += 1

    def ingest_status_json(self, text):
        status = self._parse("payload/status", text)
        if not isinstance(status, dict):
            return
        estopped = status.get("estopped")
        with self._lock:
            # Only a seen False -> True edge flashes (not a HUD started mid-E-stop).
            estop_edge = estopped is True and self._estopped is False
            if isinstance(estopped, bool):
                self._estopped = estopped
            self._status = status
            self._rx["status"] = self._mono()
        if estop_edge:
            self._raise_alert(ESTOP_TITLE, "The robot reports its E-stop engaged", "status")

    def ingest_gas_json(self, text):
        sample = self._parse("payload/gas", text)
        if not isinstance(sample, dict):
            return
        alarms = sample.get("alarms") if isinstance(sample.get("alarms"), dict) else {}
        alarm = bool(sample.get("alarm") or alarms.get("any"))
        ventilation = normalize_ventilation(sample.get("ventilation"))
        injected = sample.get("injected_pockets")
        with self._lock:
            if isinstance(injected, int) and not isinstance(injected, bool) and injected >= 0:
                self._injected = injected   # the facade's own count of staged pockets
                if injected > 0:
                    self._stage_late(self._mono())
            self._purge_progress(ventilation)
            previous = self._gas_alarm
            rising = alarm and not previous
            self._gas_alarm = alarm
            self._gas = sample
            self._ventilation = ventilation
            self._rx["gas"] = self._mono()
            if alarm and not rising:
                # A gas alarm that stays on keeps the incident open, so the
                # guard's periodic monitor echoes don't re-flash the screen.
                self._alert_touch = self._mono()
        if alarm != previous:
            # The simulator's own alarm edges go in the audit export too, in
            # the HUD's buffer, so no publisher flood can evict them (T42).
            values = gas_values(sample)
            values.pop("alarms", None)
            self._add_local("gas_alarm_on" if alarm else "gas_alarm_off",
                            "payload/gas alarm: " + gas_alarm_text(sample) if alarm
                            else "payload/gas alarm cleared", dict(values, alarm=alarm))
        if rising:
            self._raise_alert("ATMOSPHERIC ALARM", gas_alarm_text(sample), "gas",
                              reading=trigger_reading_text(sample))
        self._settle_world()   # payload/gas (4 Hz) is the HUD's tick for the M23 window

    def ingest_guard_json(self, text):
        """payload/guard: the agent's SafetyGuard heartbeat (T16, see GUARD_TOPIC)."""
        raw = self._parse(GUARD_TOPIC, text)
        heartbeat = guard_heartbeat(raw)
        if heartbeat is None:
            if raw is not None:
                self._warn.warning("guard:schema", "%s: not a guard heartbeat; ignored", GUARD_TOPIC)
                self.count_rejected(GUARD_TOPIC)
            return
        with self._lock:
            self._heartbeat = heartbeat
            self._rx["guard"] = self._mono()

    def _pose_inside(self):
        """The HUD's own odometry inside the tank shell? None if unknown (lock held)."""
        if not self._pose or not self._site:
            return None
        (cx, cy), radius = self._site["tank_center_xy"], self._site["tank_radius"]
        return math.hypot(self._pose["x"] - cx, self._pose["y"] - cy) < radius

    def _guard_view(self, now):
        """The guard's status for the page (lock held; T16).

        ``status``: monitoring / standby (a heartbeat <= GUARD_TIMEOUT_S old),
        stalled (fresh, but the guard reports that its checks have stopped
        completing; an alarm wherever Spot is), not_armed (fresh, but standby
        while Spot is inside), down (stale and
        Spot last reported inside), not_running (none ever, or stale with
        Spot outside), or not_recorded (replay: the recordings predate the
        heartbeat). With no fresh heartbeat the HUD's own odometry also
        counts as "inside" (fail closed). ``alarm``: show it in red.
        """
        heartbeat, rx = self._heartbeat, self._rx["guard"]
        pose_inside = self._pose_inside()
        if heartbeat is None:
            inside = pose_inside
            status = "not_recorded" if self._mode == "replay" else "not_running"
        elif now - rx > GUARD_TIMEOUT_S:
            inside = heartbeat["robot_inside"] or pose_inside is True
            status = "down" if inside else "not_running"
        else:
            inside = heartbeat["robot_inside"]
            if heartbeat["state"] == "stalled":
                status = "stalled"
            else:
                status = ("not_armed" if inside and heartbeat["state"] != "monitoring"
                          else heartbeat["state"])
        alarm = (status in ("down", "not_armed", "stalled")
                 or (status == "not_running" and inside is True))
        return {"status": status, "alarm": alarm, "robot_inside": inside,
                "state": heartbeat["state"] if heartbeat else None}

    def ingest_probe_json(self, text):
        """payload/gas/probe_result (JSON, see normalize_probe)."""
        raw = self._parse(PROBE_TOPIC, text)
        probe = normalize_probe(raw)
        if probe is None:
            if raw is not None:
                self._warn.warning("probe:schema", "%s: no per-height gas values found; dropped",
                                   PROBE_TOPIC)
            return
        self._set_probe(probe, "topic")

    def _set_probe(self, probe, source):
        """Store a probe result. The mission's aggregated per-height result
        (an event, with its verdict) is not overwritten by a per-request
        facade reply that arrives just after it."""
        now = self._mono()
        with self._lock:
            last = self._probe
            if (source == "topic" and last is not None and last["source"] == "event"
                    and now - self._rx["probe"] < PROBE_EVENT_HOLD_S):
                return
            probe = self._with_depths(probe)
            # The mission refused entry on readings that are all within the
            # limits because the purge is not done: a hold, not a failure.
            held = (source == "event" and probe.get("verdict") is False
                    and not probe.get("any_alarm") and bool(self._purge)
                    and self._purge.get("active"))
            self._probe = dict(probe, source=source, rx_wall=round(self._wall(), 3), held=held)
            self._rx["probe"] = now

    def _with_depths(self, probe):
        """Fill each row's depth inside the doorway (and the probe's nearest
        depth / depth list) from the site geometry when the publisher did
        not send them (the facade's probe_result carries points only)."""
        if not self._site or not probe.get("rows"):
            if probe.get("inside_m") is None:
                probe = dict(probe, inside_m=inside_distance(probe.get("xy"), self._site))
            return probe
        rows = []
        for row in probe["rows"]:
            if row.get("inside_m") is None and row.get("x") is not None:
                row = dict(row, inside_m=inside_distance([row["x"], row["y"]], self._site))
            rows.append(row)
        _sort_probe_rows(rows)
        depths = sorted({r["inside_m"] for r in rows if r.get("inside_m") is not None}
                        | set(probe.get("depths_m") or []))
        inside = probe.get("inside_m")
        if inside is None:
            inside = depths[0] if depths else inside_distance(probe.get("xy"), self._site)
        return dict(probe, rows=rows, depths_m=depths or None, inside_m=inside)

    def ingest_site_json(self, text):
        raw = self._parse("payload/site", text)
        if raw is None:
            return
        try:
            self.set_site(normalize_site(raw), "topic")
        except ValueError as error:
            self._warn.warning("payload/site:schema", "payload/site rejected: %s", error)

    def set_site(self, site, source):
        """Install a validated site; defaults never override a topic site."""
        with self._lock:
            if source != "topic" and self._site and self._site.get("source") == "topic":
                return
            self._site = dict(site, source=source)
            self._rx["site"] = self._mono()

    def has_site(self):
        with self._lock:
            return self._site is not None

    def ingest_pose(self, x, y, z=0.0, yaw=0.0, speed=0.0):
        x, y = _finite(x), _finite(y)
        if x is None or y is None:
            return
        pose = {"x": round(x, 3), "y": round(y, 3), "z": round(_finite(z, 0.0), 3),
                "yaw": round(_finite(yaw, 0.0), 4), "speed": round(_finite(speed, 0.0), 3)}
        with self._lock:
            prev = self._pose
            self._pose = pose
            self._rx["odometry"] = self._mono()
            if prev is not None and math.hypot(x - prev["x"], y - prev["y"]) > TRAIL_TELEPORT_M:
                # reset_spot.sh teleports Spot to spawn: not a walked path
                # (the next run's ready screen drew a line door -> spawn).
                self._trail.clear()
                self._trail_epoch += 1
            last = self._trail[-1] if self._trail else None
            if last is None or math.hypot(x - last[0], y - last[1]) >= TRAIL_MIN_STEP_M:
                self._trail.append((round(x, 2), round(y, 2)))

    def reset_trail(self):
        with self._lock:
            self._trail.clear()
            self._trail_epoch += 1

    def set_camera_jpeg(self, jpeg, width, height):
        with self._lock:
            self._camera = (jpeg, int(width), int(height))
            self._camera_seq += 1
            self._rx["camera"] = self._mono()

    def camera_jpeg(self):
        """(jpeg_bytes, seq) of the latest frame, or (None, 0)."""
        with self._lock:
            if self._camera is None:
                return None, 0
            return self._camera[0], self._camera_seq

    def ingest_event_json(self, text, replay=False):
        raw = self._parse("payload/events", text)
        if raw is None:
            return
        self.add_event(raw, replay=replay)

    def add_event(self, raw, replay=False):
        """Normalize, buffer, derive permit/alert, notify listeners.

        ``replay=True`` (TRANSIENT_LOCAL history delivered when the HUD
        subscribes) updates the permit and the log but never flashes: those
        alarms may be minutes old.
        """
        ev = self._normalize_event(raw)
        check = source_check(ev["source"], ev["kind"])
        listed = check == SOURCE_ALLOWLISTED
        # The facade's records report and are kept, but drive no HUD state (T3).
        trusted = listed and ev["source"] != FACADE_SOURCE
        permit = self._permit_from_event(ev)          # as claimed; applied only if trusted
        details = permit_details(ev) if trusted else {}
        data = ev.get("data") or {}
        probe = (normalize_probe(data) if trusted and data and "probe" in ev["kind"].lower()
                 else None)
        reset = trusted and _is_presenter_reset(ev)
        with self._lock:
            self._event_seq += 1
            ev["seq"] = self._event_seq
            ev["rx_wall"] = round(self._wall(), 3)
            prev_state = self._permit["state"]
            prev_id = self._permit["permit_id"]
            blocker = self._reset_blocker() if reset and not replay else None
            if trusted:
                self._apply_permit(ev, permit, details)
                self._apply_purge(ev, permit, prev_id, reset_ok=reset and blocker is None)
                self._track_retreat(ev)
            if reset and not replay and blocker is None:
                self._new_session(ev)
            ev["hud"] = self._receipt_stamp(replay, check, self._retreat_by(ev) if trusted else None)
            if blocker is not None:
                ev["hud"]["reset_refused"] = blocker
            record = self._record_class(ev, permit) if listed else SOURCE_UNVERIFIED
            self._store(ev, (record, ev["source"] if listed else None)
                        + (("no-change",) if listed and self._facade_no_change(ev) else ()))
            self._rx["events"] = self._mono()
            if not replay:   # history: its disclosures may have left with their writers
                self._note_world(ev, trusted)
        if probe is not None:
            self._set_probe(probe, "event")
        if not replay:
            kind = ev["kind"].lower()
            if trusted and _INJECT_KIND.search(kind) and data.get("published") is not False:
                self._note_injection()
            elif trusted and kind in _INJECT_CLEAR_KINDS:
                with self._lock:
                    self._inject_mono = None
            self._settle_world()
            reading_src = data.get("reading") if isinstance(data.get("reading"), dict) else data
            reading = trigger_reading_text(reading_src)
            src = "alarm" if "alarm" in ev["kind"].lower() else "event"
            note = self._retreat_note(ev) if trusted else None
            # agent/mission.py's guard alarm: detection -> zero-twist time, and
            # whether the guard heard a presenter injection disclosure.
            extra = ({"reaction_s": _finite(data.get("stop_command_s")),
                      "staged": bool(data.get("staged_by_presenter"))} if trusted else {})
            title = self._alert_title(ev) if trusted else UNVERIFIED_TITLE
            if self._is_alert_event(ev):
                self._raise_alert(title, ev["message"] or ev["kind"], ev["source"],
                                  reading=reading, note=note, reading_src=src, extra=extra,
                                  unverified=not trusted)
            elif permit == "ABORTED" and prev_state != "ABORTED":
                self._raise_alert("ENTRY ABORTED" if trusted else UNVERIFIED_TITLE,
                                  ev["message"] or "permit aborted", ev["source"],
                                  reading=reading, note=note, reading_src=src, extra=extra,
                                  unverified=not trusted)
            elif note is not None:
                self._annotate_alert(note)
        self._notify()
        return ev

    def _add_local(self, kind, message, data, bucket=("local", None)):
        """Buffer a record the HUD writes itself (source "hud"); no permit or
        alert logic runs on it."""
        ev = {"t": None, "wall_t": None, "source": LOCAL_SOURCE, "kind": kind,
              "message": message[:MAX_MESSAGE_CHARS], "data": sanitize(data)}
        with self._lock:
            self._event_seq += 1
            ev["seq"] = self._event_seq
            ev["rx_wall"] = round(self._wall(), 3)
            ev["hud"] = self._receipt_stamp(False, SOURCE_LOCAL)
            self._store(ev, bucket)
        self._notify()

    # -- M23: undisclosed atmosphere changes ---------------------------------

    def _note_world(self, ev, trusted):
        """Match an applied facade atmosphere change with a disclosure seen
        within DISCLOSURE_WINDOW_S, or a disclosure with a waiting change;
        whatever is unmatched waits (lock held). Each side is scanned once,
        when it arrives, so a flood costs O(WORLD_PENDING_MAX) per event. A
        full pending list hands its oldest change to the next settle, which
        flags it: a flood never drops one unflagged."""
        now = self._mono()
        change = world_change(ev)
        if change is not None:
            control, params, publishers = change
            for d in self._disclosures:
                if (control not in d["used"] and now - d["mono"] <= DISCLOSURE_WINDOW_S
                        and discloses(d["ev"], control, params)):
                    d["used"].add(control)
                    return
            if len(self._world_pending) >= WORLD_PENDING_MAX:
                self._world_overflow.append(dict(self._world_pending.popleft(), overflow=True))
            self._world_pending.append({"mono": now, "control": control, "params": params,
                                        "publishers": publishers, "seq": ev["seq"],
                                        "rx_wall": ev["rx_wall"]})
            return
        kind = ev["kind"].lower()
        if not trusted or not (_INJECT_KIND.search(kind) or kind in _CLEAR_DISCLOSURES
                               or kind == "ventilation"):
            return
        disclosure = {"mono": now, "ev": ev, "used": set()}
        keep = collections.deque()
        for waiting in self._world_pending:
            control = waiting["control"]
            if (control not in disclosure["used"] and now - waiting["mono"] <= DISCLOSURE_WINDOW_S
                    and discloses(ev, control, waiting["params"])):
                disclosure["used"].add(control)
            else:
                keep.append(waiting)
        self._world_pending = keep
        self._disclosures.append(disclosure)

    def _settle_world(self):
        """Flag every change whose window passed with no disclosure (and any
        overflow) as the HUD's own undisclosed_change record."""
        with self._lock:
            flags, self._world_overflow = self._world_overflow, []
            now = self._mono()
            pending, seen = self._world_pending, self._disclosures
            while pending and now - pending[0]["mono"] > DISCLOSURE_WINDOW_S:
                flags.append(pending.popleft())
            while seen and now - seen[0]["mono"] > DISCLOSURE_WINDOW_S:
                seen.popleft()
            for change in flags:
                self._world_undisclosed += 1
                self._world_last = {k: change[k] for k in ("control", "params", "publishers",
                                                           "seq", "rx_wall")}
        for change in flags:
            params = ", ".join("%s=%s" % (k, v) for k, v in sorted(change["params"].items()))
            who = change["publishers"] or []
            who = " [publishers: %s]" % ", ".join(map(str, who[:3])) if who else ""
            self._add_local(
                UNDISCLOSED_KIND,
                "SIMULATED %s applied (%s) with no disclosure from an allowlisted publisher "
                "within %.0f s%s" % (change["control"], params or "no parameters",
                                     DISCLOSURE_WINDOW_S, who),
                {"control": change["control"], "params": change["params"],
                 "publishers": change["publishers"], "facade_seq": change["seq"],
                 "window_s": DISCLOSURE_WINDOW_S, "overflow": bool(change.get("overflow"))},
                bucket=("local", "world"))

    def _world_view(self):
        """The undisclosed-change counters for the snapshot (lock held)."""
        return {"undisclosed": self._world_undisclosed, "pending": len(self._world_pending),
                "last": self._world_last, "window_s": DISCLOSURE_WINDOW_S}

    @staticmethod
    def _facade_no_change(ev):
        """A facade record of something that changed nothing: a refused world
        control or a service call with no power or e-stop change. The facade
        records every refused safety control (sim/facade_guard.py), so these
        get their own buffer and a refusal flood cannot evict what was
        applied (T23, T42)."""
        data = ev.get("data") or {}
        return ev.get("source") == FACADE_SOURCE and (data.get("accepted") is False
                                                      or data.get("changed") is False)

    @staticmethod
    def _record_class(ev, permit):
        """'safety' for the records an incident review needs (alarms, permit
        transitions, retreats, test verdicts, presenter disclosures), else
        'routine'. The guard's 5 s monitor readings are routine."""
        kind = (ev.get("kind") or "").lower()
        data = ev.get("data") or {}
        if kind == "monitor":
            return "routine"
        if ev.get("source") == FACADE_SOURCE:
            return "safety"     # only FACADE_KINDS are allowlisted under its name
        safety = (HudState._is_alert_event(ev) or permit is not None or "retreat" in kind
                  or bool(_INJECT_KIND.search(kind)) or kind in _INJECT_CLEAR_KINDS
                  or kind in (PURGE_HOLD_KIND, PURGE_FALLBACK_KIND, "entry_denied")
                  or any(isinstance(data.get(k), bool) for k in ("pass", "passed")))
        return "safety" if safety else "routine"

    def _store(self, ev, bucket):
        """Append to its bounded buffer, counting an eviction (lock held)."""
        queue = self._buckets.get(bucket)
        if queue is None:
            cap = {"safety": SAFETY_BUFFER, "routine": ROUTINE_BUFFER, "local": LOCAL_BUFFER,
                   SOURCE_UNVERIFIED: UNVERIFIED_BUFFER}[bucket[0]]
            queue = self._buckets[bucket] = collections.deque(maxlen=cap)
        if len(queue) == queue.maxlen:
            self._evicted["/".join(b for b in bucket if b)] += 1
        ev["hud"]["record"] = bucket[0]
        queue.append(ev)

    def _merged(self, classes=None):
        """Buffered events (of ``classes``, or all), oldest first (lock held)."""
        out = [ev for key, queue in self._buckets.items()
               if classes is None or key[0] in classes for ev in queue]
        out.sort(key=lambda ev: ev["seq"])
        return out

    def _page_events(self):
        """The page's log (lock held): every protected record, plus the
        newest EVENT_BUFFER others. A flood of routine or unverified events
        never pushes an alarm, permit or retreat record off the page."""
        others = self._merged(("routine", SOURCE_UNVERIFIED))[-EVENT_BUFFER:]
        out = self._merged(PROTECTED_RECORDS) + others
        out.sort(key=lambda ev: ev["seq"])
        return out

    def _ingest_view(self):
        """Counters for the page and the audit headers (lock held)."""
        protected = sum(n for key, n in self._evicted.items()
                        if key.split("/")[0] in PROTECTED_RECORDS + (SOURCE_UNVERIFIED,))
        return {"dropped": dict(self._dropped), "rejected": dict(self._rejected),
                "evicted": dict(self._evicted),
                "flood_total": sum(self._dropped.values()) + sum(self._rejected.values())
                + protected}

    def ingest_counts(self):
        with self._lock:
            return self._ingest_view()

    def _reset_blocker(self):
        """Why a presenter reset must not clear the screen now, or None
        (lock held). A reset while the gas alarm is on or a retreat is
        under way would hide a live incident (T3), and a forged reset is
        exactly that, so it is refused; the event is still logged."""
        if self._gas_alarm:
            return "gas alarm active"
        if self._retreat_mono is not None and self._mono() - self._retreat_mono < ALERT_ANNOTATE_S:
            return "retreat in progress"
        return None

    def _track_retreat(self, ev):
        """Start/end of a retreat from an allowlisted event (lock held)."""
        kind = (ev.get("kind") or "").lower()
        data = ev.get("data") or {}
        if (("retreat" in kind and (_CLEAR_WORDS.search(kind) or "failed" in kind))
                or self._is_retreat_fault(ev)):
            self._retreat_mono = None     # retreat_complete / _incomplete / _failed / software fault
        elif kind == "retreat_start" or (data.get("auto_retreat") is True
                                         and self._is_alert_event(ev) and kind != "monitor"):
            self._retreat_mono = self._mono()

    def _new_session(self, ev):
        """host/reset_spot.sh (presenter demo_reset): Spot is back at spawn
        for a new run, so the screen starts fresh - permit NOT STARTED with
        an empty checklist, no probe table, no path trail
        (lock held). The mission log and the audit export keep every event;
        the alert keeps its sequence number (no flash), and the incident
        strip hides because the permit is no longer ABORTED. Rehearsal
        2026-09-28: the HUD still showed the previous run's ABORTED permit
        and trigger banner at the next run's start."""
        self._permit_fields = {}
        self._permit_gate = None
        checklist, summary = build_checklist({})
        self._permit = {"state": "NOT_STARTED", "since_wall": ev["rx_wall"], "permit_id": None,
                        "message": "Reset by the presenter: ready for a new permit",
                        "checklist": checklist, "checklist_summary": summary}
        self._probe = None             # (a purge hold was already released: _apply_purge)
        self._trail.clear()
        self._trail_epoch += 1

    def _apply_permit(self, ev, permit, details):
        """Update permit state/id/checklist from one event (lock held)."""
        data = ev.get("data") or {}
        new_id = details.get("permit_id")
        if new_id is None and permit is not None and data.get("permit_id") not in (None, ""):
            new_id = str(data["permit_id"])[:64]
        current = self._permit
        fields_changed = False
        if new_id and current["permit_id"] and new_id != current["permit_id"]:
            self._permit_fields = {}   # a new permit: its checklist starts unknown
            self._permit_gate = None
            fields_changed = True
        for key, _label, _keys in PERMIT_CHECKLIST:
            if key in details:
                self._permit_fields[key] = details[key]
                fields_changed = True
        if "_gate" in details:
            self._permit_gate = details["_gate"]
            fields_changed = True
        if permit is None and not fields_changed and not new_id:
            return
        checklist, summary = build_checklist(self._permit_fields, self._permit_gate)
        self._permit = {
            "state": permit or current["state"],
            "since_wall": (ev["rx_wall"] if permit is not None and permit != current["state"]
                           else current["since_wall"]),
            "permit_id": new_id or current["permit_id"],
            "message": (ev["message"] or None) if permit is not None else current["message"],
            "checklist": checklist,
            "checklist_summary": summary,
        }

    def _apply_purge(self, ev, permit, prev_id, reset_ok=False):
        """Open, refine or release the purge hold from one event (lock held).
        ``reset_ok``: this is a presenter reset the HUD accepted."""
        kind = (ev.get("kind") or "").lower()
        data = ev.get("data") or {}
        hold = purge_hold_from(kind, data)
        if hold is not None:
            if kind == PURGE_FALLBACK_KIND and self._purge and self._purge.get("active") \
                    and self._purge.get("kind") == PURGE_HOLD_KIND:
                return   # the structured event already set this hold
            self._purge = dict(hold, kind=kind, active=True, t=ev.get("t"),
                               rx_wall=ev["rx_wall"], permit_id=self._permit["permit_id"],
                               reached=False, reached_fraction=None, released=None)
            self._purge_progress(self._ventilation)
            return
        if not self._purge or not self._purge.get("active"):
            return
        reason = None
        if kind == "atmos_test_start":
            reason = "retest"
        elif kind == "demo_reset" and reset_ok:
            reason = "reset"
        elif permit is not None and permit != "PRE_ENTRY_TESTING":
            reason = "permit " + permit
        elif self._permit["permit_id"] != prev_id and prev_id is not None:
            reason = "new permit"
        elif kind == "ventilation" and data.get("purged") is True:
            frac = _fraction(data.get("fraction_remaining"))
            self._purge = dict(self._purge, reached=True,
                               reached_fraction=frac if frac is not None
                               else self._purge.get("reached_fraction"))
        if reason is not None:
            self._purge = dict(self._purge, active=False, released=reason)

    def _purge_progress(self, ventilation):
        """Mark an active hold 'reached' once the simulator's live
        ventilation fraction is at or below the hold's target (lock held)."""
        hold = self._purge
        if not hold or not hold.get("active") or not isinstance(ventilation, dict):
            return
        frac, target = ventilation.get("fraction_remaining"), hold.get("purge_target")
        if frac is not None and target is not None and frac <= target + 1e-9 \
                and not hold.get("reached"):
            self._purge = dict(hold, reached=True, reached_fraction=round(frac, 4))

    def _receipt_stamp(self, replay, check=SOURCE_ALLOWLISTED, retreat=None):
        """The HUD's view when an event arrived (lock held): permit state
        after applying it, the source check (T3), the HUD's own retreat
        classification (automatic / commanded, from the event's cause, never
        from its source), and the latest payload/gas sample if it is fresh.
        Replayed history gets no gas stamp (those events may be minutes old)."""
        stamp = {"permit_state": self._permit["state"], "replay": bool(replay), "gas": None,
                 "source_check": check, "retreat": retreat}
        rx = self._rx["gas"]
        age = None if rx is None else self._mono() - rx
        if not replay and self._gas is not None and age is not None \
                and age <= RECEIPT_GAS_MAX_AGE_S:
            values = gas_values(self._gas)
            values.pop("alarms", None)
            stamp.update(gas=values or None, gas_alarm=self._gas_alarm, gas_age_s=round(age, 2))
        return stamp

    @staticmethod
    def _retreat_by(ev):
        """'automatic' or 'commanded' for an event that starts, runs or ends a
        retreat; None otherwise and in rehearsal mode (auto-retreat off).
        From the mission's structured fields only: the guard's cause
        (atmosphere / telemetry) or its alarm's auto_retreat flag. The
        self-declared source never makes a retreat automatic (T3)."""
        kind = (ev.get("kind") or "").lower()
        data = ev.get("data") or {}
        if data.get("auto_retreat") is False:
            return None
        if not ("retreat" in kind or data.get("auto_retreat") is True
                or data.get("action") == "retreat"):
            return None
        automatic = data.get("cause") in AUTOMATIC_CAUSES or data.get("auto_retreat") is True
        return "automatic" if automatic else "commanded"

    @staticmethod
    def _retreat_note(ev):
        """Banner note for events showing a retreat has started or finished;
        None otherwise (incl. a failed retreat and rehearsal mode)."""
        kind = (ev.get("kind") or "").lower()
        by = HudState._retreat_by(ev)
        if by is None or "incomplete" in kind or "failed" in kind:
            return None
        automatic = by == "automatic"
        if "retreat" in kind and _CLEAR_WORDS.search(kind):   # retreat_complete / _done
            return RETREAT_NOTE if automatic else COMMAND_RETREAT_NOTE
        return RETREATING_NOTE if automatic else COMMAND_RETREATING_NOTE

    @staticmethod
    def _is_retreat_fault(ev):
        """The mission's retreat worker died ("Retreat software fault: ..."):
        the retreat did not finish, and it emits no retreat_incomplete."""
        return ((ev.get("kind") or "").lower() == "error"
                and ev.get("source") in ("guard", "mission")
                and bool(_RETREAT_FAULT.search(ev.get("message") or "")))

    @staticmethod
    def _normalize_event(raw):
        if not isinstance(raw, dict):
            return {"t": None, "wall_t": None, "source": "unknown", "kind": "raw",
                    "message": str(raw)[:MAX_MESSAGE_CHARS], "data": None}
        data = raw.get("data")
        if data is not None and not isinstance(data, dict):
            data = {"value": data}
        if data is not None:
            text = json.dumps(data)
            if len(text) > MAX_DATA_CHARS:
                # Keep the keys permit/alert/PASS-FAIL are derived from, so a
                # big payload can't silently drop a permit transition, the
                # permit checklist, or the reading that triggered a retreat.
                keep = {k: data[k] for k in _KEEP_ON_TRUNCATE
                        if data.get(k) is None and k in data
                        or isinstance(data.get(k), (str, bool, int, float))}
                for k in _KEEP_NESTED_ON_TRUNCATE:
                    if isinstance(data.get(k), dict) \
                            and len(json.dumps(data[k])) <= _NESTED_KEEP_MAX_CHARS:
                        keep[k] = data[k]
                data = dict(keep, _truncated=True, preview=text[:MAX_DATA_CHARS])
        return {
            "t": _finite(raw.get("t")),
            # Publisher's unix wall time (agent/robot_client.make_event);
            # None from publishers that predate it.
            "wall_t": _finite(raw.get("wall_t")),
            "source": str(raw.get("source") or "unknown")[:32],
            "kind": str(raw.get("kind") or "info")[:64],
            "message": str(raw.get("message") or "")[:MAX_MESSAGE_CHARS],
            "data": data,
        }

    @staticmethod
    def _permit_from_event(ev):
        data = ev.get("data") or {}
        nested = data.get("permit") if isinstance(data.get("permit"), dict) else {}
        for src in (data, nested):   # top level wins over the nested Permit dict
            for key in ("permit_state", "state", "to"):
                value = src.get(key)
                if isinstance(value, str) and value.upper() in PERMIT_STATES:
                    return value.upper()
        return None

    @staticmethod
    def _is_alert_event(ev):
        data = ev.get("data") or {}
        if data.get("alarm") is True or HudState._is_retreat_fault(ev):
            return True
        kind = ev.get("kind") or ""
        return bool(_ALERT_WORDS.search(kind)) and not _CLEAR_WORDS.search(
            _INCOMPLETE.sub("", kind))

    @staticmethod
    def _alert_title(ev):
        kind = (ev.get("kind") or "").lower()
        data = ev.get("data") or {}
        if re.search(r"e[-_]?stop", kind):
            return ESTOP_TITLE
        if "incomplete" in kind or kind == "retreat_failed" or HudState._is_retreat_fault(ev):
            return RETREAT_FAILED_TITLE
        if "abort" in kind:
            return "ENTRY ABORTED"
        if data.get("auto_retreat") is False or "ignored" in kind:
            return "ALARM: AUTO-RETREAT OFF"   # mission rehearsal mode
        if HudState._retreat_by(ev) == "automatic":
            return "SAFETY GUARD: RETREAT"     # from the cause, not the source (T3)
        return "ALARM"

    def _raise_alert(self, title, reason, source, reading=None, note=None, reading_src="gas",
                     extra=None, unverified=False):
        """One flash per incident; later signals of the same incident only
        refine the text.

        ``reading`` is the triggering reading ('H2S 25.3 ppm (limit < 10
        ppm)'): the first one seen wins, except that the guard's own alarm
        event (``reading_src`` 'alarm', the sample it acted on) replaces a
        reading taken from the payload/gas edge. ``note`` (the retreat note)
        is added whenever a signal carries one. ``extra`` may set
        ``reaction_s`` (first value wins) and ``staged``: the guard heard a
        disclosure, which stages the alert only together with the facade's
        injected_pockets > 0 (see STAGED_WINDOW_S; sticky once set).
        ``unverified``: raised by an event from a source outside
        EVENT_SOURCES (T3). Such a claim never refines or prolongs a
        verified incident, and a verified signal after an unverified
        incident is new news: it flashes anew and carries nothing over.

        The incident stays open while signals keep arriving less than
        ALERT_DEDUPE_S apart (gas samples in alarm count, see ingest_gas_json),
        so the guard's 5 s monitor echoes during a retreat never re-cover the
        screen. An E-stop or a failed retreat is new, worse news and always
        flashes. The gas topic's rising edge usually lands first (4 Hz) with an
        auto-generated reason; the guard's event text is more informative, so
        an event may replace a 'gas' reason but not another event's.
        """
        now_wall, now = self._wall(), self._mono()
        with self._lock:
            last = self._alert
            incident_open = (last["t_wall"] is not None and self._alert_touch is not None
                             and now - self._alert_touch < ALERT_DEDUPE_S)
            same_kind = bool(last.get("unverified")) == bool(unverified)
            if incident_open and unverified and not same_kind:
                return
            self._alert_touch = now
            escalation = (title in ESCALATION_TITLES and title != last["title"]) or not same_kind
            claimed = bool((extra or {}).get("staged"))
            if incident_open and not escalation:
                update = {}
                if last["source"] == "gas" and source != "gas":
                    update.update(title=title, reason=reason, source=source)
                if reading and (last["reading"] is None or
                                (self._alert_reading_src == "gas" and reading_src == "alarm")):
                    update["reading"] = reading
                    self._alert_reading_src = reading_src
                if note and not (last["note"] in _DONE_NOTES and note not in _DONE_NOTES):
                    update["note"] = note   # never downgrade 'retreated' to 'retreating'
                if extra and extra.get("reaction_s") is not None and last["reaction_s"] is None:
                    update["reaction_s"] = extra["reaction_s"]
                self._alert_claim = self._alert_claim or claimed
                if not last["staged"] and self._staged(now, self._alert_claim):
                    update["staged"] = True
                if update:
                    self._alert = dict(last, **update)
                return
            carried = last["reading"] if incident_open and same_kind else None
            self._alert_claim = claimed or (incident_open and same_kind and self._alert_claim)
            staged = self._staged(now, self._alert_claim)
            if incident_open:
                staged = staged or last["staged"]
            if reading:
                self._alert_reading_src = reading_src
            elif not carried:
                self._alert_reading_src = None
            reaction = (extra or {}).get("reaction_s")
            if reaction is None and incident_open:
                reaction = last["reaction_s"]
            self._alert = {"seq": last["seq"] + 1, "title": title, "reason": reason,
                           "source": source, "t_wall": round(now_wall, 3),
                           "reading": reading or carried, "note": note,
                           "reaction_s": reaction, "staged": staged,
                           "unverified": bool(unverified)}
        self._notify()

    def _staged(self, now, claimed=False):
        """Is this alarm presenter-staged by the interface contract? (lock
        held) Both must hold: a disclosure was received less than
        STAGED_WINDOW_S ago and not cleared since (``claimed``: the guard's
        alarm says it heard one), AND the facade's payload/gas reports
        injected_pockets > 0. A simulator that reports no count, a count
        with no disclosure, or a disclosure with no pocket: genuine."""
        disclosed = claimed or (self._inject_mono is not None
                                and now - self._inject_mono < STAGED_WINDOW_S)
        return disclosed and self._injected is not None and self._injected > 0

    def _note_injection(self):
        """An injection disclosure event: start the staging window, and stage
        an alert raised just before it (the gas edge can beat the event)."""
        now = self._mono()
        with self._lock:
            self._inject_mono = now
            self._stage_late(now)

    def _stage_late(self, now):
        """Stage the latest alert when the missing half of the contract (a
        disclosure, or a payload/gas count > 0) arrives within STAGED_LATE_S
        after it: DDS does not order payload/gas against payload/events
        (lock held)."""
        last = self._alert
        if (last["t_wall"] is not None and not last["staged"]
                and self._wall() - last["t_wall"] <= STAGED_LATE_S
                and self._staged(now, self._alert_claim)):
            self._alert = dict(last, staged=True)

    def _annotate_alert(self, note):
        """Add a retreat note to the latest alert without re-flashing (e.g.
        retreat_complete arriving after the incident window closed)."""
        with self._lock:
            last = self._alert
            if last["t_wall"] is None or last["note"] == note \
                    or (last["note"] in _DONE_NOTES and note not in _DONE_NOTES) \
                    or self._wall() - last["t_wall"] > ALERT_ANNOTATE_S:
                return
            self._alert = dict(last, note=note)

    # -- readers (called from the server) ----------------------------------

    @property
    def alert_seq(self):
        with self._lock:
            return self._alert["seq"]

    def events(self):
        """Every buffered event (oldest first), for the audit export."""
        with self._lock:
            return self._merged()

    def events_since(self, seq):
        with self._lock:
            out = []
            for queue in self._buckets.values():
                for ev in reversed(queue):
                    if ev["seq"] <= seq:
                        break
                    out.append(ev)
        out.sort(key=lambda ev: ev["seq"])
        return out

    def snapshot(self, include_events=False, include_trail=False):
        """JSON-ready dict. Events/trail are opt-in: the WebSocket streams
        those incrementally, so the 4 Hz diffs stay small."""
        self._settle_world()
        now = self._mono()
        with self._lock:
            camera = self._camera
            snap = {
                "server": {"mode": self._mode, "bridge_ok": self._bridge_ok,
                           "bridge_note": self._bridge_note,
                           "wall_time": round(self._wall(), 3)},
                "permit": dict(self._permit),
                "status": self._status,
                "gas": self._gas,
                "ventilation": self._ventilation,
                "probe": self._probe,
                "purge_hold": self._purge,
                "pose": self._pose,
                "site": self._site,
                "camera": {"seq": self._camera_seq,
                           "width": camera[1] if camera else None,
                           "height": camera[2] if camera else None},
                "link": {k: (round(now - t, 1) if t is not None else None)
                         for k, t in self._rx.items()},
                "alert": dict(self._alert),
                "thresholds": OSHA_THRESHOLDS,
                "events_seq": self._event_seq,
                "trail_epoch": self._trail_epoch,
                "ingest": self._ingest_view(),
                "guard": self._guard_view(now),
                "world": self._world_view(),
            }
            if include_events:
                snap["events"] = self._page_events()
            if include_trail:
                snap["trail"] = [list(p) for p in self._trail]
        return snap


# ---------------------------------------------------------------------------
# RosBridge (real topics)
# ---------------------------------------------------------------------------


def check_ros_available():
    """Raise BridgeUnavailable with an actionable message if rclpy is missing."""
    try:
        import rclpy  # noqa: F401
        from nav_msgs.msg import Odometry  # noqa: F401
        from sensor_msgs.msg import Image as _RosImage  # noqa: F401
        from std_msgs.msg import String  # noqa: F401
    except ImportError as error:
        raise BridgeUnavailable(
            "ROS 2 Python packages not importable (%s). Run inside the HUD "
            "container (hud/Dockerfile, ros:jazzy) or pass --fake." % error) from error


class RosBridge:
    """Subscribes to the contract topics in a private rclpy context.

    A private ``Context`` (not the global one) keeps start/stop idempotent and
    lets stop() tear everything down without touching any other rclpy user in
    the process. Camera frames are only *referenced* in the executor callback
    and encoded on a separate worker thread, so a slow JPEG encode can never
    delay odometry/gas/event callbacks.
    """

    def __init__(self, state, namespace=DEFAULT_NAMESPACE, camera_topic=DEFAULT_CAMERA_TOPIC,
                 camera_fps=5.0, camera_max_width=800, jpeg_quality=70,
                 site_defaults_path=None, site_timeout_s=10.0):
        self.state = state
        self.namespace = namespace
        self.camera_topic = camera_topic
        self.camera_period = 1.0 / max(0.5, min(float(camera_fps), 5.0))  # contract: <= 5 fps
        self.camera_max_width = int(camera_max_width)
        self.jpeg_quality = int(jpeg_quality)
        self.site_defaults_path = site_defaults_path
        self.site_timeout_s = float(site_timeout_s)
        self._stop = threading.Event()
        self._cam_wake = threading.Event()
        self._cam_lock = threading.Lock()
        self._cam_pending = None
        self._threads = []
        self._ctx = self._node = self._executor = None
        self._started_mono = None
        self._warn = _RateLimitedLog()
        self._clock = time.monotonic   # the ingest limits' clock (tests replace it)

    def start(self):
        os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
        check_ros_available()
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image as RosImage
        from std_msgs.msg import String

        self._ctx = Context()
        # Same pattern as agent/robot_client.py: uvicorn owns SIGINT/SIGTERM
        # and the lifespan hook shuts this private context down, so rclpy's
        # own handler must not tear it down underneath the spin thread.
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(context=self._ctx, signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init(context=self._ctx)
        self._node = rclpy.create_node("presenter_hud", namespace=self.namespace,
                                       context=self._ctx)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # Matches agent/robot_client.py's publisher (reliable + TRANSIENT_LOCAL,
        # depth 100): a HUD restarted mid-mission gets the recent history and
        # therefore the current permit state instead of showing NOT STARTED.
        events_qos = QoSProfile(depth=EVENT_HISTORY_DEPTH, history=QoSHistoryPolicy.KEEP_LAST,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node, st = self._node, self.state
        node.create_subscription(String, "payload/status", self._guard(
            "payload/status", lambda m: st.ingest_status_json(m.data)), 10)
        node.create_subscription(String, "payload/gas", self._guard(
            "payload/gas", lambda m: st.ingest_gas_json(m.data)), 10)
        # Reliable + volatile (depth 10) matches both a volatile and a
        # TRANSIENT_LOCAL reliable publisher; one message per probe.
        node.create_subscription(String, PROBE_TOPIC, self._guard(
            PROBE_TOPIC, lambda m: st.ingest_probe_json(m.data)), 10)
        node.create_subscription(String, "payload/events", self._guard(
            "payload/events", self._events_by_source()), events_qos)
        node.create_subscription(String, "payload/site", self._guard(
            "payload/site", lambda m: st.ingest_site_json(m.data)), latched)
        # T16 contract QoS: reliable, VOLATILE (a dead agent's last heartbeat
        # must not be replayed as if it were alive), depth 10.
        guard_qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                               reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.VOLATILE)
        node.create_subscription(String, GUARD_TOPIC, self._guard(
            GUARD_TOPIC, lambda m: st.ingest_guard_json(m.data)), guard_qos)
        # Best-effort subscribers match both reliable and best-effort
        # publishers and never apply backpressure to the 50 Hz odom stream.
        node.create_subscription(Odometry, "odometry",
                                 self._guard("odometry", self._on_odometry), qos_profile_sensor_data)
        node.create_subscription(RosImage, self.camera_topic,
                                 self._guard("camera", self._on_image), qos_profile_sensor_data)

        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._started_mono = time.monotonic()
        self._stop.clear()
        for target, name in ((self._spin, "hud-ros-spin"), (self._work, "hud-camera-worker")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        self.state.set_bridge_info(True, "rclpy node %s/presenter_hud (%s)" % (
            self.namespace, os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")))
        log.info("ROS bridge up: namespace=%s camera=%s ROS_DOMAIN_ID=%s transports=%s",
                 self.namespace, self.camera_topic, os.environ.get("ROS_DOMAIN_ID", "0"),
                 os.environ.get("FASTDDS_BUILTIN_TRANSPORTS"))

    def _in_replay_window(self):
        """True while TRANSIENT_LOCAL history is expected (just after start)."""
        return (self._started_mono is not None
                and time.monotonic() - self._started_mono < EVENT_REPLAY_WINDOW_S)

    def _guard(self, topic, fn):
        """Wrap a subscription callback: drop and count what is over the
        topic's ingest limit before any parsing (T42), and never let one
        malformed message raise into (and kill) the executor spin loop."""
        limit = _TokenBucket(*INGEST_LIMITS[topic], clock=lambda: self._clock())

        def callback(msg):
            if not limit.take():
                self.state.count_dropped(topic)
                self._warn.warning("limit:" + topic, "%s: over %.0f msg/s; dropping the excess "
                                   "(counted on the HUD)", topic, INGEST_LIMITS[topic][0])
                return
            try:
                fn(msg)
            except Exception as error:
                self._warn.warning("cb:" + topic, "%s callback failed: %r", topic, error)
        return callback

    def _events_by_source(self):
        """payload/events callback: parse (bounded), then a per-source ingest
        limit (T42) before the event reaches HudState."""
        buckets = {}
        st = self.state

        def callback(msg):
            raw = st._parse("payload/events", msg.data)
            if raw is None:
                return
            # The same name _normalize_event will store.
            name = str((raw.get("source") if isinstance(raw, dict) else None) or "unknown")[:32]
            if name not in buckets and len(buckets) >= SOURCE_BUCKETS_MAX:
                name = "*other*"
            bucket = buckets.get(name)
            if bucket is None:
                bucket = buckets[name] = _TokenBucket(*SOURCE_EVENT_LIMIT, clock=lambda: self._clock())
            if not bucket.take():
                st.count_dropped("payload/events")
                self._warn.warning("limit:source", "payload/events: source %r over %.0f msg/s; "
                                   "dropping the excess (counted on the HUD)", name, SOURCE_EVENT_LIMIT[0])
                return
            st.add_event(raw, replay=self._in_replay_window())
        return callback

    def _on_odometry(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        self.state.ingest_pose(p.x, p.y, p.z, yaw, math.hypot(v.x, v.y))

    def _on_image(self, msg):
        error = camera_frame_error(msg.width, msg.height, msg.encoding, msg.step, len(msg.data))
        if error:   # T32: never queued, never decoded; counted
            self.state.count_rejected("camera")
            self._warn.warning("camera:refused", "camera frame refused: %s", error)
            return
        with self._cam_lock:
            self._cam_pending = msg  # keep only the newest; older frames are moot
        self._cam_wake.set()

    def _spin(self):
        try:
            while not self._stop.is_set() and self._ctx.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as error:  # includes ExternalShutdownException on Ctrl-C
            if not self._stop.is_set():
                log.error("ROS spin loop ended unexpectedly: %r", error)
                self.state.set_bridge_info(False, "ROS spin loop stopped: %r" % (error,))

    def _work(self):
        """Camera encode at <= camera_fps + one-shot site-defaults fallback."""
        last_encode = 0.0
        while not self._stop.is_set():
            self._cam_wake.wait(timeout=0.5)
            if (not self.state.has_site()
                    and time.monotonic() - self._started_mono >= self.site_timeout_s):
                site, source = load_site_defaults(self.site_defaults_path)
                log.warning("no payload/site within %.0f s; using %s site layout",
                            self.site_timeout_s, source)
                self.state.set_site(site, source)
            delay = last_encode + self.camera_period - time.monotonic()
            if delay > 0 and self._stop.wait(delay):
                return
            self._cam_wake.clear()
            with self._cam_lock:
                msg, self._cam_pending = self._cam_pending, None
            if msg is None:
                continue
            last_encode = time.monotonic()
            try:
                jpeg, w, h = encode_ros_image(msg.data, msg.width, msg.height, msg.encoding,
                                              msg.step, self.camera_max_width, self.jpeg_quality)
                self.state.set_camera_jpeg(jpeg, w, h)
            except ValueError as error:
                self.state.count_rejected("camera")
                self._warn.warning("camera:encode", "camera frame dropped: %s", error)

    def stop(self):
        """Idempotent, bounded-time teardown (never hangs server shutdown)."""
        self._stop.set()
        self._cam_wake.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        try:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=1.0)
            if self._node is not None:
                self._node.destroy_node()
            if self._ctx is not None and self._ctx.ok():
                self._ctx.try_shutdown()
        except Exception as error:
            log.warning("ROS bridge teardown: %r", error)
        self._executor = self._node = self._ctx = None
        self.state.set_bridge_info(False, "stopped")


# ---------------------------------------------------------------------------
# FakeBridge (synthetic scenario for --fake and tests)
# ---------------------------------------------------------------------------


def _along(poly, dist):
    """Point + heading at arc length ``dist`` along polyline ``poly``."""
    yaw = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-9:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        if dist <= seg:
            f = dist / seg
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw
        dist -= seg
    return poly[-1][0], poly[-1][1], yaw


def _poly_len(poly):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poly, poly[1:]))


class FakeBridge:
    """Deterministic scripted scenario on the BUILTIN_SITE geometry.

    ``tick(t)`` is a pure function of scenario time t (plus emit-once event
    bookkeeping), so tests call it directly with ``realtime=False``; with
    ``realtime=True`` a thread calls it at 20 Hz and loops every PERIOD_S.
    Timeline: permit opened with an empty checklist (0 s) -> isolation,
    LOTO, attendant, rescue plan confirmed (0.6 s) -> stand (1 s) -> forced
    ventilation starts (3 s; tau 1.5 s, compressed for the fake) -> walk to
    entry (3-10 s) -> 3-height door-mouth probe on payload/gas/probe_result
    (10.5 s) -> O2/LEL/H2S/CO tests from the probe (11-13 s) -> authorize +
    enter (14-15 s) -> injected H2S spike from 20 s (alarm ~20.15 s) -> guard
    retreat + permit ABORTED (20.2-20.5 s) -> back at the entry (~26 s) -> sit.
    Ventilation is shown only; the fake's gas values do not depend on it.
    """

    PERIOD_S = 45.0
    TICK_S = 0.05
    WALK_SPEED = 1.0       # fake runs faster than the real 0.5 m/s cap
    INSIDE_SPEED = 0.8
    WALK_T = (3.0, 10.0)
    ENTER_T = 15.0
    SPIKE_T = 20.0
    ABORT_T = 20.5
    VENT_T = 3.0
    VENT_TAU_S = 1.5
    VENT_RESIDUAL = 0.05
    PROBE_T = 10.5
    PROBE_INSIDE_M = 0.5                 # nearest depth (agent/mission.py PROBE_DEPTHS_M[0])
    PROBE_DEPTHS_M = (0.5, 1.0)          # agent/mission.py PROBE_DEPTHS_M
    PROBE_HEIGHTS_M = (0.2, 0.6, 1.0)
    PERMIT_ID = "CS-0417 (synthetic)"
    CAMERA_LABEL = "SYNTHETIC FRAME - FAKE MODE"

    def __init__(self, state=None, realtime=True, speed=1.0, loop=True, camera_fps=5.0):
        self._init_runtime(state if state is not None else HudState(mode="fake"),
                           realtime, speed, loop, camera_fps)
        site = BUILTIN_SITE
        self._spawn = tuple(site["spawn_xy"])
        self._entry = tuple(site["entry_xy"])
        self._forward = [self._entry, tuple(site["door_xy"])] + [tuple(p) for p in site["interior_waypoints"]]
        d_abort = self.INSIDE_SPEED * (self.ABORT_T - self.ENTER_T)
        ax, ay, _ = _along(self._forward, d_abort)
        visited, travelled = [], 0.0
        for a, b in zip(self._forward, self._forward[1:]):
            if travelled >= d_abort:
                break
            visited.append(a)
            travelled += math.hypot(b[0] - a[0], b[1] - a[1])
        self._retreat = [(ax, ay)] + list(reversed(visited))
        self._retreat_end_t = self.ABORT_T + _poly_len(self._retreat) / self.INSIDE_SPEED
        sx, sy, _ = self.pose_at(self.SPIKE_T)
        self._spike_xy = (sx, sy)
        self._probe_raw = self._build_probe()
        self._probe_worst = normalize_probe(self._probe_raw)["worst"]
        self._timeline = self._build_timeline()
        self._reset()

    def _init_runtime(self, state, realtime, speed, loop, camera_fps):
        """Thread/rate/font plumbing shared with ReplayBridge."""
        self.state = state
        self._realtime = realtime
        self._speed = float(speed)
        self._loop = loop
        self._camera_period = 1.0 / max(0.5, min(float(camera_fps), 5.0))
        self._stop = threading.Event()
        self._thread = None
        try:
            self._font = ImageFont.load_default(size=22)  # Pillow >= 10.1
        except TypeError:
            self._font = ImageFont.load_default()

    # -- scenario model (pure functions of t) --------------------------------

    def pose_at(self, t):
        """(x, y, yaw) of the scripted robot at scenario time t."""
        w0, w1 = self.WALK_T
        if t < w0:
            return self._spawn[0], self._spawn[1], math.pi
        if t < w1:
            return _along([self._spawn, self._entry], (t - w0) * self.WALK_SPEED)
        if t < self.ENTER_T:
            return self._entry[0], self._entry[1], math.pi
        if t < self.ABORT_T:
            return _along(self._forward, (t - self.ENTER_T) * self.INSIDE_SPEED)
        return _along(self._retreat, (t - self.ABORT_T) * self.INSIDE_SPEED)

    def gas_at(self, t, x, y):
        """Payload/gas-shaped sample: clean outside, mild inside, H2S spike at 20 s."""
        r = math.hypot(x - BUILTIN_SITE["tank_center_xy"][0], y - BUILTIN_SITE["tank_center_xy"][1])
        depth = max(0.0, min(1.0, (BUILTIN_SITE["tank_radius"] - r) / BUILTIN_SITE["tank_radius"]))
        inside = r < BUILTIN_SITE["tank_radius"]
        sample = {
            "o2_pct": 20.9 + 0.03 * math.sin(1.3 * t) - (0.6 * depth if inside else 0.0),
            "lel_pct": (1.5 + 2.5 * depth + 0.2 * math.sin(0.9 * t)) if inside else 0.0,
            "h2s_ppm": (1.0 + 3.0 * depth + 0.2 * math.sin(1.7 * t)) if inside else 0.0,
            "co_ppm": (2.0 + 3.0 * depth + 0.3 * math.sin(0.7 * t)) if inside else 0.0,
        }
        if t >= self.SPIKE_T:
            # Stand-in for the operator-console payload/gas/inject spike.
            amp = 30.0 * (1.0 - math.exp(-(t - self.SPIKE_T) / 0.5))
            d2 = (x - self._spike_xy[0]) ** 2 + (y - self._spike_xy[1]) ** 2
            sample["h2s_ppm"] += amp * math.exp(-d2 / (2.0 * 1.2 ** 2))
        sample = {k: round(v, 2) for k, v in sample.items()}
        sample["alarms"] = compute_alarms(sample)
        sample["alarm"] = sample["alarms"]["any"]
        sample["ventilation"] = self.ventilation_at(t)
        # The facade's own count of presenter pockets (sim/isaac_app.py).
        sample["injected_pockets"] = 1 if t >= self.SPIKE_T else 0
        return sample

    def ventilation_at(self, t):
        """payload/gas 'ventilation' block (the shape documented in README)."""
        active = t >= self.VENT_T
        elapsed = max(0.0, t - self.VENT_T)
        fraction = (self.VENT_RESIDUAL + (1.0 - self.VENT_RESIDUAL)
                    * math.exp(-elapsed / self.VENT_TAU_S)) if active else 1.0
        return {"active": active, "mode": "ventilating" if active else "raw",
                "fraction_remaining": round(fraction, 3), "tau_s": self.VENT_TAU_S,
                "residual": self.VENT_RESIDUAL, "elapsed_s": round(elapsed, 2)}

    def _build_probe(self):
        """The mission's door-mouth probe result: the (simulated) extendable
        gas probe at PROBE_DEPTHS_M inside the doorway x three heights, six
        points (H2S pooled low)."""
        door = BUILTIN_SITE["door_xy"]
        samples = []
        for j, depth in enumerate(self.PROBE_DEPTHS_M):
            x, y = door[0] - depth, door[1]   # door faces +x: inside is -x
            for i, z in enumerate(self.PROBE_HEIGHTS_M):
                samples.append({"point": [x, y, z], "inside_m": depth,
                                "o2_pct": round(20.8 + 0.05 * i, 2),
                                "lel_pct": round(0.6 - 0.15 * i + 0.1 * j, 2),
                                "h2s_ppm": round(1.8 - 0.7 * i - 0.2 * j, 2),
                                "co_ppm": round(2.0 - 0.5 * i, 2)})
        return {"t": self.PROBE_T, "xy": [door[0] - self.PROBE_INSIDE_M, door[1]],
                "inside_m": self.PROBE_INSIDE_M, "depths_m": list(self.PROBE_DEPTHS_M),
                "samples": samples, "ventilation": self.ventilation_at(self.PROBE_T)}

    def _build_timeline(self):
        def atmo(t, gas, label, unit, limit):
            def make():
                value = self._probe_worst[OSHA_THRESHOLDS[gas]["key"]]
                ok = not self._probe_worst["alarms"][gas]
                return ("mission", "atmospheric_test",
                        "%s %.1f %s -- %s (%s; worst of 6 probe points)"
                        % (label, value, unit, "PASS" if ok else "FAIL", limit),
                        {"gas": gas, "value": value, "pass": ok})
            return (t, make)

        def guard_alarm():
            t = self.SPIKE_T + 0.2
            x, y, _ = self.pose_at(t)
            sample = self.gas_at(t, x, y)
            return ("guard", "gas_alarm",
                    "%s -- ANY-alarm rule: retreat along the entry path" % gas_alarm_text(sample),
                    {"gas": "h2s", "value": sample["h2s_ppm"], "threshold": 10.0,
                     "action": "retreat", "alarm": True, "auto_retreat": True,
                     "reading": sample, "stop_command_s": 0.02})

        def fixed(t, source, kind, message, data=None):
            return (t, lambda: (source, kind, message, data))

        permit_id = self.PERMIT_ID
        empty = {"permit_id": permit_id, "isolation": False, "loto": False,
                 "attendant": None, "rescue_plan": None}
        ready = {"permit_id": permit_id, "isolation": "Blinded at inlet and outlet (synthetic)",
                 "loto": "Mixer and pumps locked out, tag 0417 (synthetic)",
                 "attendant": "attendant on station at the entry (synthetic)",
                 "rescue_plan": "Standby rescue team at the manway (synthetic)"}
        return [
            fixed(0.0, "mission", "permit", "Scenario start: permit opened (fake mode)",
                  {"permit_state": "NOT_STARTED", "permit_id": permit_id, "permit": empty}),
            fixed(0.6, "mission", "permit_prepared",
                  "Permit prepared: isolation, LOTO, attendant and rescue plan confirmed",
                  {"permit": ready}),
            fixed(1.0, "agent", "say", "I'll run the pre-entry sequence: stand, approach the "
                  "cleanout door, and test the atmosphere before anyone commits to entry."),
            fixed(1.2, "agent", "tool_call", "stand()", {"tool": "stand", "args": {}}),
            fixed(2.8, "mission", "info", "Standing. Walking to the entry point (7.0, 0.0)."),
            fixed(3.0, "agent", "tool_call", "walk_to(x=7.0, y=0.0)",
                  {"tool": "walk_to", "args": {"x": 7.0, "y": 0.0}}),
            fixed(3.1, "presenter", "ventilation",
                  "PRESENTER DEMO: forced ventilation started (simulated pocket decay)",
                  {"active": True}),
            fixed(10.0, "mission", "permit", "At the doorway: extending the simulated gas probe "
                  "%s m inside (2 depths x 3 heights)"
                  % " and ".join("%.1f" % d for d in self.PROBE_DEPTHS_M),
                  {"permit_state": "PRE_ENTRY_TESTING"}),
            atmo(11.0, "o2", "O2", "%", "safe 19.5-23.5 %"),
            atmo(12.0, "lel", "LEL", "%", "limit < 10 %"),
            atmo(13.0, "h2s", "H2S", "ppm", "limit < 10 ppm"),
            atmo(13.2, "co", "CO", "ppm", "limit < 35 ppm"),
            fixed(13.8, "agent", "say", "All four readings are inside OSHA limits at the opening. "
                  "Authorizing entry; monitoring continues the whole time I'm inside."),
            fixed(14.0, "mission", "permit", "Entry authorized",
                  {"permit_state": "ENTRY_AUTHORIZED"}),
            fixed(15.0, "mission", "permit", "Entering through the cleanout doorway",
                  {"permit_state": "ENTRY_IN_PROGRESS"}),
            fixed(15.1, "agent", "tool_call", "follow_waypoints(count=5)",
                  {"tool": "follow_waypoints", "args": {"count": 5}}),
            fixed(self.SPIKE_T, "presenter", "demo_inject",
                  "PRESENTER DEMO: simulated H2S pocket injected at (%.1f, %.1f)" % self._spike_xy,
                  {"gas": "h2s", "ppm": 30.0, "x": round(self._spike_xy[0], 3),
                   "y": round(self._spike_xy[1], 3), "radius": 1.2}),
            (self.SPIKE_T + 0.2, guard_alarm),
            fixed(self.ABORT_T, "mission", "permit", "Permit aborted: H2S above the 10 ppm limit",
                  {"permit_state": "ABORTED"}),
            fixed(21.0, "agent", "say", "The safety guard pulled us out on H2S above 10 ppm. "
                  "That's the right call -- retreating the way we came in."),
            fixed(self._retreat_end_t, "mission", "retreat_complete",
                  "Retreat complete: robot is outside the space at the entry point.",
                  {"cause": "atmosphere"}),
            fixed(self._retreat_end_t + 1.0, "mission", "info",
                  "Sitting. No person entered the space.", {"entrants": 0}),
        ]

    # -- driving HudState ----------------------------------------------------

    def _reset(self):
        self._next_event = 0
        self._last_t = -1.0
        self._last_pub = {"status": -1e9, "gas": -1e9, "pose": -1e9, "camera": -1e9,
                          "guard": -1e9}
        self._probe_sent = False
        self._guard_seq = 0

    def tick(self, t):
        """Advance the scripted scenario to time t (seconds) and publish."""
        st = self.state
        if t < self._last_t:  # loop wrapped: start a fresh run
            self._reset()
            st.reset_trail()
        self._last_t = t
        if not st.has_site():
            st.ingest_site_json(json.dumps(BUILTIN_SITE))
        while self._next_event < len(self._timeline) and self._timeline[self._next_event][0] <= t:
            ev_t, make = self._timeline[self._next_event]
            self._next_event += 1
            source, kind, message, data = make()
            st.ingest_event_json(json.dumps({"t": round(ev_t, 2), "wall_t": round(st._wall(), 3),
                                             "source": source, "kind": kind,
                                             "message": message, "data": data}))
        if not self._probe_sent and t >= self.PROBE_T:
            self._probe_sent = True
            st.ingest_probe_json(json.dumps(self._probe_raw))
        x, y, yaw = self.pose_at(t)
        moving = (self.WALK_T[0] <= t < self.WALK_T[1]) or (self.ENTER_T <= t < self._retreat_end_t)
        if t - self._last_pub["pose"] >= 0.05:
            self._last_pub["pose"] = t
            st.ingest_pose(x, y, 0.52, yaw, (self.WALK_SPEED if t < self.WALK_T[1] else
                                              self.INSIDE_SPEED) if moving else 0.0)
        if t - self._last_pub["gas"] >= 0.25:
            self._last_pub["gas"] = t
            st.ingest_gas_json(json.dumps(self.gas_at(t, x, y)))
        if t - self._last_pub["status"] >= 0.5:
            self._last_pub["status"] = t
            sitting_again = t >= self._retreat_end_t + 1.0
            st.ingest_status_json(json.dumps({
                "claimed": t >= 1.0, "powered": t >= 1.0,
                "standing": 2.6 <= t and not sitting_again,
                "standing_up": 1.0 <= t < 2.6, "estopped": False,
                "battery_pct": round(100.0 - 0.02 * t, 2), "sim_time": round(t, 3),
            }))
        if t - self._last_pub["camera"] >= self._camera_period:
            self._last_pub["camera"] = t
            self._publish_camera(t, x, y)
        if t - self._last_pub["guard"] >= GUARD_PERIOD_S:
            # The agent's SafetyGuard heartbeat (T16 contract): armed while inside.
            self._last_pub["guard"] = t
            self._guard_seq += 1
            centre, radius = BUILTIN_SITE["tank_center_xy"], BUILTIN_SITE["tank_radius"]
            inside = math.hypot(x - centre[0], y - centre[1]) < radius
            st.ingest_guard_json(json.dumps({"state": "monitoring" if inside else "standby",
                                             "robot_inside": inside, "seq": self._guard_seq,
                                             "wall_t": round(st._wall(), 3)}))

    def _publish_camera(self, t, x, y):
        """Synthetic 640x360 frame pushed through the real rgb8 encode path."""
        w, h = 640, 360
        img = Image.new("RGB", (w, h), (16, 19, 22))
        draw = ImageDraw.Draw(img)
        door = BUILTIN_SITE["door_xy"]
        dist = math.hypot(x - door[0], y - door[1])
        if x > door[0]:  # outside: draw the tank wall + doorway growing as we approach
            half = int(max(18, min(150, 260 / max(dist, 0.5))))
            draw.rectangle([0, h // 2 - 2 * half, w, h // 2 + half], fill=(70, 34, 22))
            draw.rectangle([w // 2 - half, h // 2 - half, w // 2 + half, h // 2 + half], fill=(4, 4, 5))
        else:
            draw.text((24, h // 2), "TANK INTERIOR (unlit)", fill=(90, 96, 102), font=self._font)
        draw.text((24, 20), self.CAMERA_LABEL, fill=(230, 232, 234), font=self._font)
        draw.text((24, h - 44), "T+%5.1f s  pose (%.1f, %.1f)"
                  % (t, round(x, 1) + 0.0, round(y, 1) + 0.0),   # never "-0.0"
                  fill=(160, 168, 175), font=self._font)
        jpeg, jw, jh = encode_ros_image(img.tobytes(), w, h, "rgb8", w * 3)
        self.state.set_camera_jpeg(jpeg, jw, jh)

    def start(self):
        self.state.set_bridge_info(True, "synthetic scenario (--fake); no simulator connected")
        if self._realtime and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="hud-fake", daemon=True)
            self._thread.start()
        log.info("fake bridge up (realtime=%s speed=%.1f loop=%s)",
                 self._realtime, self._speed, self._loop)

    def _run(self):
        t0 = time.monotonic()
        while not self._stop.wait(self.TICK_S):
            t = (time.monotonic() - t0) * self._speed
            if self._loop:
                t %= self.PERIOD_S
            try:
                self.tick(t)
            except Exception:
                log.exception("fake scenario tick failed at t=%.2f", t)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.state.set_bridge_info(False, "stopped")


# ---------------------------------------------------------------------------
# FakeBridge replay mode (recorded demo -> HudState)
# ---------------------------------------------------------------------------

_CLEAN_AIR = {"o2_pct": 20.9, "lel_pct": 0.0, "h2s_ppm": 0.0, "co_ppm": 0.0}
_GAS_KEYS = tuple(OSHA_THRESHOLDS[g]["key"] for g in GAS_ORDER)
REPLAY_RESIDUAL = 0.04      # sim/gas_field.py ventilation residual (the logs carry it too)


def _full_reading(obj):
    """{o2_pct, lel_pct, h2s_ppm, co_ppm} if ``obj`` carries all four, else None."""
    values = gas_values(obj) if isinstance(obj, dict) else {}
    values.pop("alarms", None)
    return {k: values[k] for k in _GAS_KEYS} if all(k in values for k in _GAS_KEYS) else None


def _record(raw, origin):
    """A recorded event -> {t, wall_t, source, kind, message, data, gas, origin}, or None."""
    wall = _finite(raw.get("wall_t"))
    if wall is None or not raw.get("kind") or raw.get("kind") == "summary":
        return None
    data = raw.get("data")
    return {"t": _finite(raw.get("t")), "wall_t": wall, "source": str(raw.get("source") or "unknown"),
            "kind": str(raw["kind"]), "message": str(raw.get("message") or ""),
            "data": data if isinstance(data, dict) or data is None else {"value": data},
            "gas": raw.get("gas"), "origin": origin}


def _read_replay_file(path):
    """Records from one recording: a mission audit log / HUD /api/audit.jsonl
    (.jsonl), or a HUD /api/audit.csv export (.csv: adds the agent transcript
    and the payload/gas sample the HUD held when each event arrived)."""
    import csv
    out = []
    name = os.path.basename(path)
    if path.lower().endswith(".csv"):
        with open(path, "r", encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                try:
                    data = json.loads(row.get("data_json") or "null")
                except ValueError:
                    data = None
                rec = _record(dict(row, data=data), name)
                if rec is None:
                    continue
                gas = {k: _finite(row.get(k)) for k in _GAS_KEYS}
                rec["gas"] = gas if all(v is not None for v in gas.values()) else None
                out.append(rec)
        return out
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rec = _record(json.loads(line), name)
            except (ValueError, AttributeError):
                continue
            if rec is not None:
                out.append(rec)
    return out


def load_replay_records(paths, margin_s=20.0):
    """Merge recordings into one wall-time-ordered event list.

    The first file is the primary recording (normally the mission's
    docs/evidence/events-*.jsonl) and sets the time window; records from
    later files (e.g. the HUD audit CSV of the same session, which also
    holds the agent's speech and tool calls) are added when they fall
    inside that window (+- ``margin_s``) and are not already present."""
    if not paths:
        raise ValueError("replay needs at least one recording")
    records = _read_replay_file(paths[0])
    if not records:
        raise ValueError("no replayable events in %s" % paths[0])
    lo = min(r["wall_t"] for r in records) - margin_s
    hi = max(r["wall_t"] for r in records) + margin_s
    seen = {(round(r["wall_t"], 2), r["source"], r["kind"], r["message"][:120]) for r in records}
    for path in paths[1:]:
        for rec in _read_replay_file(path):
            if not lo <= rec["wall_t"] <= hi:
                continue
            key = (round(rec["wall_t"], 2), rec["source"], rec["kind"], rec["message"][:120])
            if key in seen:
                if rec["gas"] is not None:   # a duplicate still carries the HUD's gas sample
                    records.append(dict(rec, kind="_gas_sample", message="", data=None))
                continue
            seen.add(key)
            records.append(rec)
    records.sort(key=lambda r: r["wall_t"])
    return records


class ReplayBridge(FakeBridge):
    """FakeBridge replay mode: re-publishes a RECORDED demo (the real event
    sequence of a dry run, e.g. docs/evidence/events-ok1.jsonl, optionally
    plus that session's HUD audit CSV) through the same HudState.ingest_*
    paths the ROS callbacks use, so the HUD can be checked on a Mac against
    what really happened -- no simulator.

    The recordings hold events, not telemetry, so pose, gas, ventilation
    and robot status are RECONSTRUCTED from them: pose from atmos_test_start
    at_xy, waypoint / alarm / retreat poses and exit/retreat paths (linear in
    between); gas from guard monitor / alarm readings and the CSV's
    payload/gas samples (held between samples, clean air outside); the
    ventilation block from the ventilation events and the blocks the
    mission logged (exponential decay toward the residual between them).
    Events keep their recorded ``t`` / ``wall_t``. The replay clock is wall
    time since the first record; ``start_at`` is a SIM time (as shown on the
    HUD, e.g. 444.6), and ``hold=True`` freezes the scene there while the
    telemetry keeps flowing (screenshots). The page shows a REPLAY badge and
    the camera tile a placeholder: nothing here is live.
    """

    LEAD_S = 2.0
    CAMERA_LABEL = "REPLAY OF RECORDED EVENTS - NO CAMERA"

    def __init__(self, paths, state=None, realtime=True, speed=1.0, start_at=None, hold=False,
                 camera_fps=5.0, say_filter=None, site=None):
        self._now_tau = 0.0
        # The HUD's wall clock runs on the RECORDED timeline, so permit
        # "for m:ss", probe ages and the alarm flash's freshness match what
        # the room saw at that moment (an alarm 17 s before the replay point
        # shows as the header summary, not as a fresh full-screen flash).
        self._init_runtime(state if state is not None
                           else HudState(mode="replay", wall_clock=self.recorded_wall),
                           realtime, speed, False, camera_fps)
        self.paths = list(paths)
        self._say_filter = say_filter
        self._site_raw = dict(site or BUILTIN_SITE)
        self.records = load_replay_records(self.paths)
        self._wall0 = self.records[0]["wall_t"] - self.LEAD_S
        offsets = sorted(r["t"] - r["wall_t"] for r in self.records
                         if r["t"] is not None and r["t"] < 1e8)
        self._sim_offset = offsets[len(offsets) // 2] if offsets else None
        for rec in self.records:
            rec["tau"] = rec["wall_t"] - self._wall0
        self.duration = self.records[-1]["tau"] + 3.0
        self._build_tracks()
        self._start_tau = self.tau_of_sim(start_at) if start_at is not None else 0.0
        self._hold = bool(hold)
        self._reset()

    # -- time --------------------------------------------------------------

    def recorded_wall(self):
        """Unix time on the recording's own timeline at the replay position."""
        return self._wall0 + self._now_tau

    def tau_of_sim(self, sim_t):
        """Replay-clock time of a recorded SIM time (clamped to the recording)."""
        if self._sim_offset is None:
            return max(0.0, float(sim_t))
        tau = float(sim_t) - self._sim_offset - self._wall0
        return max(0.0, min(self.duration, tau))

    def sim_time(self, tau):
        if self._sim_offset is None:
            return round(tau, 3)
        return round(self._wall0 + tau + self._sim_offset, 3)

    # -- reconstructed tracks -------------------------------------------------

    def _build_tracks(self):
        site = self._site_raw
        spawn = tuple(site.get("spawn_xy") or BUILTIN_SITE["spawn_xy"])
        door = tuple(site.get("door_xy") or BUILTIN_SITE["door_xy"])
        yaw0 = _finite(site.get("spawn_yaw"), math.pi)
        poses = [(0.0, spawn[0], spawn[1], yaw0)]
        gas = [(0.0, dict(_CLEAN_AIR))]    # event-carried readings + clean-air resets
        hud_samples = []                   # the CSV's payload/gas-at-receipt samples
        incidents, alarm_tau = [], None    # [alarm, retreat end] windows
        vent = [(0.0, {"on": False, "tau_s": None, "fraction": 1.0, "residual": REPLAY_RESIDUAL})]
        standing = [(0.0, False)]
        injected = [(0.0, 0)]
        pending_path = None

        def here():
            return poses[-1][1], poses[-1][2]

        def put(tau, x, y, yaw=None):
            poses.append((tau, float(x), float(y), yaw))

        def walk_path(tau_end, path):
            """Spread ``path`` points between the last pose and tau_end by distance."""
            tau0, x0, y0 = poses[-1][0], poses[-1][1], poses[-1][2]
            pts = [(float(p[0]), float(p[1])) for p in path if _xy_ok(p)]
            legs, total, last = [], 0.0, (x0, y0)
            for p in pts:
                total += math.hypot(p[0] - last[0], p[1] - last[1])
                legs.append((total, p))
                last = p
            for dist, p in legs[:-1] if legs else []:
                put(tau0 + (tau_end - tau0) * (dist / total if total > 1e-6 else 1.0), *p)

        def vent_now(tau):
            return _vent_eval(vent[-1], tau)

        def vent_block(tau, block):
            frac = _finite(block.get("fraction_remaining"))
            on = block.get("on") if isinstance(block.get("on"), bool) else vent[-1][1]["on"]
            tau_s = _finite(block.get("tau_s")) or vent[-1][1]["tau_s"]
            resid = _finite(block.get("residual_fraction"), vent[-1][1]["residual"])
            vent.append((tau, {"on": on, "tau_s": tau_s,
                               "fraction": frac if frac is not None else vent_now(tau),
                               "residual": resid}))

        for rec in self.records:
            tau, kind, data = rec["tau"], rec["kind"], rec["data"] or {}
            if rec["gas"] is not None:
                hud_samples.append((tau, dict(rec["gas"])))
            if kind == "_gas_sample":
                continue
            reading = _full_reading(data.get("reading")) or (
                _full_reading(data) if kind in ("monitor", "alarm") else None)
            if reading is not None:
                gas.append((tau, reading))
            if kind == "alarm" and alarm_tau is None:
                alarm_tau = tau
            elif kind in ("retreat_complete", "retreat_incomplete") and alarm_tau is not None:
                incidents.append((alarm_tau, tau))
                alarm_tau = None
            for block in (data.get("ventilation"),
                          (data.get("reading") or {}).get("ventilation")
                          if isinstance(data.get("reading"), dict) else None):
                if isinstance(block, dict):
                    vent_block(tau, block)
            for block in (data, data.get("reading")):
                # The guard's alarm carries the count in the reading it acted on.
                if isinstance(block, dict) and isinstance(block.get("injected_pockets"), int):
                    injected.append((tau, block["injected_pockets"]))
                    break
            if kind == "ventilation" and isinstance(data.get("on"), bool):
                vent_block(tau, data)
            elif kind == "demo_inject":
                injected.append((tau, injected[-1][1] + 1))
            elif kind in ("demo_clear", "gas_cleared"):
                injected.append((tau, 0))
                gas.append((tau, dict(_CLEAN_AIR)))
            elif kind == "demo_reset":
                injected.append((tau, 0))
                gas.append((tau, dict(_CLEAN_AIR)))
                if data.get("reset_ventilation"):
                    vent.append((tau, {"on": False, "tau_s": None, "fraction": 1.0,
                                       "residual": REPLAY_RESIDUAL}))
                if "spawn" in rec["message"].lower():
                    put(tau, *here(), poses[-1][3])
                    put(tau + 0.05, spawn[0], spawn[1], yaw0)
            elif kind == "robot" and data.get("service") in ("stand", "sit") and data.get("ok"):
                standing.append((tau, data["service"] == "stand"))
                put(tau, *here(), poses[-1][3])
            elif kind == "atmos_test_start" and _xy_ok(data.get("at_xy")):
                x, y = data["at_xy"][:2]
                put(tau, x, y, math.atan2(door[1] - y, door[0] - x))
            elif kind == "permit" and str(data.get("state") or data.get("permit_state") or "").upper() \
                    in ("ENTRY_IN_PROGRESS",):
                put(tau, *here(), poses[-1][3])
            elif kind == "waypoint":
                pose = data.get("pose") if _xy_ok(data.get("pose")) else None
                xy = pose or (data.get("xy") if _xy_ok(data.get("xy")) else None)
                if xy is not None:
                    yaw = _finite(pose[2]) if pose is not None and len(pose) > 2 else None
                    put(tau, xy[0], xy[1], yaw)
            elif kind in ("alarm", "retreat_start") and _xy_ok(data.get("pose")):
                p = data["pose"]
                put(tau, p[0], p[1], _finite(p[2]) if len(p) > 2 else None)
                if kind == "retreat_start" and isinstance(data.get("path"), list):
                    pending_path = data["path"]
            elif kind == "exit_start" and isinstance(data.get("path"), list):
                put(tau, *here(), poses[-1][3])   # stood still until the exit began
                pending_path = data["path"]
            elif kind == "retreat_complete" or (
                    kind == "permit" and str(data.get("state") or "").upper() == "COMPLETED"):
                if pending_path:
                    walk_path(tau, pending_path)
                end = data.get("pose") if _xy_ok(data.get("pose")) else (
                    pending_path[-1] if pending_path and _xy_ok(pending_path[-1]) else None)
                if end is not None:
                    put(tau, end[0], end[1], _finite(end[2]) if len(end) > 2 else None)
                pending_path = None
                gas.append((tau + 0.3, dict(_CLEAN_AIR)))   # the body sensor is outside again
        if alarm_tau is not None:
            incidents.append((alarm_tau, float("inf")))
        # A HUD sample is the latest payload/gas when an event ARRIVED, so it
        # can predate a reading the event itself carries (the alarm burst):
        # use one only away from event readings and outside an incident,
        # where the guard's own readings (in alarm until the retreat ends)
        # are the record. Otherwise a stale clean sample would end the alarm
        # and the next reading would re-flash the screen.
        for tau, sample in hud_samples:
            if any(abs(tau - t) < 1.0 for t, _r in gas[1:]) or \
                    any(a - 0.5 <= tau <= b for a, b in incidents):
                continue
            gas.append((tau, sample))
        gas.sort(key=lambda item: item[0])
        self._poses, self._gas_track, self._vent_track = poses, gas, vent
        self._standing, self._injected_track = standing, injected

    @staticmethod
    def _at(track, tau):
        """Latest (tau_k, value) with tau_k <= tau (tracks are time-ordered)."""
        best = track[0]
        for item in track:
            if item[0] > tau:
                break
            best = item
        return best

    def pose_at(self, tau):
        """(x, y, yaw, speed) reconstructed at replay time ``tau``."""
        poses = self._poses
        prev = poses[0]
        heading = prev[3] if prev[3] is not None else math.pi
        for nxt in poses[1:]:
            if nxt[0] > tau:
                dt = nxt[0] - prev[0]
                dist = math.hypot(nxt[1] - prev[1], nxt[2] - prev[2])
                if dist > 1e-3 and dt > 1e-6:
                    f = max(0.0, min(1.0, (tau - prev[0]) / dt))
                    return (prev[1] + f * (nxt[1] - prev[1]), prev[2] + f * (nxt[2] - prev[2]),
                            math.atan2(nxt[2] - prev[2], nxt[1] - prev[1]), dist / dt)
                return prev[1], prev[2], (prev[3] if prev[3] is not None else heading), 0.0
            if math.hypot(nxt[1] - prev[1], nxt[2] - prev[2]) > 1e-3:
                heading = math.atan2(nxt[2] - prev[2], nxt[1] - prev[1])
            if nxt[3] is not None:
                heading = nxt[3]
            prev = nxt
        return prev[1], prev[2], (prev[3] if prev[3] is not None else heading), 0.0

    def ventilation_at(self, tau):
        _k, block = self._at(self._vent_track, tau)
        frac = _vent_eval(self._at(self._vent_track, tau), tau)
        return {"on": bool(block["on"]), "tau_s": block["tau_s"], "fraction_remaining": round(frac, 4),
                "residual_fraction": block["residual"], "mode": "ventilating" if block["on"] else "raw"}

    def gas_at(self, tau, x=None, y=None):
        sample = dict(self._at(self._gas_track, tau)[1])
        sample = {k: round(float(v), 3) for k, v in sample.items()}
        sample["alarms"] = compute_alarms(sample)
        sample["alarm"] = sample["alarms"]["any"]
        sample["ventilation"] = self.ventilation_at(tau)
        sample["injected_pockets"] = self._at(self._injected_track, tau)[1]
        return sample

    # -- driving HudState ----------------------------------------------------

    def _reset(self):
        super()._reset()
        self._pub_wall = {"status": -1e9, "gas": -1e9, "pose": -1e9, "camera": -1e9}
        self._trail_tau = 0.0

    def _due(self, key, tau, period):
        """Publish when scenario time OR wall time advanced by ``period``
        (a held replay keeps the telemetry fresh)."""
        now = time.monotonic()
        if tau - self._last_pub[key] >= period or now - self._pub_wall[key] >= period:
            self._last_pub[key], self._pub_wall[key] = tau, now
            return True
        return False

    def tick(self, tau):
        st = self.state
        if tau < self._last_t:
            return   # no looping, no rewinding: a replay only moves forward
        self._last_t = tau
        if not st.has_site():
            st.ingest_site_json(json.dumps(self._site_raw))
        records = self.records
        while self._next_event < len(records) and records[self._next_event]["tau"] <= tau:
            rec = records[self._next_event]
            self._next_event += 1
            self._now_tau = rec["tau"]
            if rec["kind"] == "_gas_sample":
                continue
            message = rec["message"]
            if rec["kind"] == "say" and self._say_filter is not None:
                message = self._say_filter(message)
            st.ingest_event_json(json.dumps({"t": rec["t"], "wall_t": rec["wall_t"],
                                             "source": rec["source"], "kind": rec["kind"],
                                             "message": message, "data": rec["data"]}))
        self._now_tau = tau
        if tau - self._trail_tau > 0.5:
            # A jump (start_at, or a slow host): walk the path in between so
            # the map's "path walked" trail is complete.
            step = self._trail_tau + 0.25
            while step < tau:
                px, py, pyaw, _v = self.pose_at(step)
                st.ingest_pose(px, py, 0.52, pyaw, 0.0)
                step += 0.25
        self._trail_tau = tau
        x, y, yaw, speed = self.pose_at(tau)
        if self._due("pose", tau, 0.05):
            st.ingest_pose(x, y, 0.52, yaw, speed)
        if self._due("gas", tau, 0.25):
            st.ingest_gas_json(json.dumps(self.gas_at(tau)))
        if self._due("status", tau, 0.5):
            standing = self._at(self._standing, tau)[1]
            st.ingest_status_json(json.dumps({
                "claimed": True, "powered": True, "standing": standing, "standing_up": False,
                "estopped": False, "battery_pct": round(max(30.0, 97.0 - tau / 240.0), 1),
                "sim_time": self.sim_time(tau)}))
        if self._due("camera", tau, self._camera_period):
            self._publish_camera(self.sim_time(tau), x, y)

    def start(self):
        self.state.set_bridge_info(True, "replay of recorded events (%s); no simulator connected"
                                   % ", ".join(os.path.basename(p) for p in self.paths))
        if self._realtime and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="hud-replay", daemon=True)
            self._thread.start()
        log.info("replay bridge up: %s (start tau=%.1f s, hold=%s, speed=%.1f)",
                 ", ".join(self.paths), self._start_tau, self._hold, self._speed)

    def _run(self):
        t0 = time.monotonic()
        while not self._stop.wait(self.TICK_S):
            tau = self._start_tau if self._hold else min(
                self.duration, self._start_tau + (time.monotonic() - t0) * self._speed)
            try:
                self.tick(tau)
            except Exception:
                log.exception("replay tick failed at tau=%.2f", tau)


def _xy_ok(value):
    return (isinstance(value, (list, tuple)) and len(value) >= 2
            and _finite(value[0]) is not None and _finite(value[1]) is not None)


def _vent_eval(item, tau):
    """Ventilation fraction at ``tau`` from keyframe (tau_k, block): decays
    toward the residual while on (sim/gas_field.py model), else held."""
    tau_k, block = item
    frac = block["fraction"] if block["fraction"] is not None else 1.0
    if block["on"] and block["tau_s"]:
        r = block["residual"] if block["residual"] is not None else REPLAY_RESIDUAL
        frac = r + (frac - r) * math.exp(-max(0.0, tau - tau_k) / block["tau_s"])
    return max(0.0, min(1.0, frac))
