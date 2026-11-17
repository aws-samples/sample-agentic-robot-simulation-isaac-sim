"""Permit-required confined-space entry mission for Spot (shared contract 3).

What this is
------------
``ConfinedSpaceMission`` scripts an OSHA 1910.146-style entry of the crude
tank built by sim/build_scene.py, and ``SafetyGuard`` is the deterministic
(no LLM) watchdog that pulls Spot out on ANY gas alarm, whoever is driving.
The mission only touches the robot through the RobotClient API
(agent/robot_client.py) - any object with the same methods works, which is
how tests/test_mission.py runs it on a Mac against a kinematic fake.

Permit states::

    NOT_STARTED -> PRE_ENTRY_TESTING -> ENTRY_AUTHORIZED -> ENTRY_IN_PROGRESS
                                                         -> COMPLETED | ABORTED
    (ABORTED is reachable from every non-terminal state)

Entry permit fields (``Permit``, ``prepare_permit()``): permit id,
isolation confirmed, LOTO confirmed, attendant, rescue plan. With
``require_permit=True`` (the default) Spot will not approach the entry -
``walk_to`` refuses any target within ``tank_radius + APPROACH_MARGIN_M`` of
the tank, and any leg that dips into that zone - and no atmospheric test can
authorize entry until every field is recorded. ``new_permit()`` starts a
blank permit (each entry needs its own).

Scripted run (``run()``): permit check -> claim/power/stand (body-height
check, bounded self-right retry) -> walk to entry_xy -> face the door ->
atmospheric test (O2 -> LEL -> H2S/CO, each logged PASS/FAIL) -> authorize ->
enter via door_xy and the interior waypoints -> survey dwell -> exit back
along the visited path -> COMPLETED -> sit (after stepping straight out to
SIT_STANDOFF_M from the door if it is behind Spot: the facade's stand-up
shoves Spot backward, see ``_step_to_standoff``). ``run(storyline=True)``
keeps a failed test as "entry denied" (permit stays PRE_ENTRY_TESTING),
starts SIMULATED forced ventilation, waits for the purge and re-tests
before entering; without it a failed test aborts the permit.

Atmospheric test: Spot stands with its nose at the opening (door_mouth_xy)
and the extendable gas probe (simulated; arm-mounted on the real robot)
samples six points: PROBE_DEPTHS_M (0.5 and 1.0 m) inside the opening along
the door normal, each at PROBE_HEIGHTS_M (H2S pools low). Each gas is judged
in OSHA order on its worst reading across points and samples. A facade
rejection (out of reach, occluded by the tank wall, no pose) re-seats Spot
at the door mouth and retries once; without a working probe the body sensor
is used and an explicit ``probe_unavailable`` event says so; such a test
(or one taken away from the door) can deny entry but never authorize it,
because from outside the tank the body sensor never samples the space.
Purge gate: after a failed atmosphere on the permit, or once the simulator
reports the tank ventilated, a passing test authorizes entry only at
ventilation fraction_remaining <= purge_target (default 0.08); otherwise
``purge_incomplete`` says how long to wait (the door probe alone passes
while floor-level H2S deeper inside still alarms), and ``purge_hold``
carries the numbers ({fraction_remaining, purge_target, eta_s}) that the
HUD and the agent both quote.

Step-by-step use (the LLM agent): ``prepare_permit()``, ``prepare()``,
``stand_up()``, ``walk_to()``, ``face()``, ``face_door()``,
``atmospheric_test()``, ``start_ventilation()``, ``ventilation_status()``,
``await_ventilation()``, ``enter()``, ``exit_space()``, ``retreat()``,
``new_permit()``, ``last_incident()``, ``status_snapshot()``, ``close()``.
There are no hidden globals; every piece of state lives on the instance.

Why the safety logic looks like this
------------------------------------
* ``SafetyGuard`` runs in its own thread at 20 Hz (contract: >= 4 Hz; the
  facade publishes gas at 4 Hz, so a new sample is seen within 50 ms). It is
  armed while an entry is authorized or in progress (ENTRY_AUTHORIZED,
  ENTRY_IN_PROGRESS), whenever a fresh pose puts Spot inside the permit
  space, and whenever there is no fresh pose (unknown counts as inside, so a
  withheld pose cannot disarm it, T22); not while a retreat owns the robot.
  An LLM driving raw cmd_vel into the tank gets the same protection as the
  scripted mission. It fires on the rising
  edge of any OSHA App. E exceedance (O2 < 19.5 or > 23.5 %, LEL >= 10 %,
  H2S >= 10 ppm, CO >= 35 ppm), on the facade's own alarm flag, and on loss
  of gas telemetry for 5 s (continuous monitoring is a permit condition).
* On alarm the guard thread itself zeroes cmd_vel and sets the halt flag
  before anything else (forward motion stops within one 50 ms control tick),
  then a retreat thread takes cmd_vel ownership (other senders are dropped)
  and reverses the visited path. The first retreat leg is driven in REVERSE
  when the last waypoint is behind Spot: that backs away from the hazard
  immediately and avoids turning in place mid-segment, where clearance to
  the pipe runs was never verified. Later legs turn in place only at planned
  waypoints. Spot never turns in place within 0.9 m of the doorway (the jambs
  are 0.65 m from the door axis; Spot's half-diagonal is 0.6 m) - it backs
  out instead, and a straight run through the door is never split by a stop.
* The permit flips to ABORTED the moment the retreat starts, with the
  triggering reading, because the entry is no longer supervised; retreat
  completion is logged separately.
* A retreat leg that loses odometry (or times out) is not abandoned: Spot
  holds zero velocity until the pose returns and the leg is retried, up to
  RETREAT_LEG_RETRIES times within the RETREAT_TIMEOUT_S budget
  (``retreat_hold`` events).
* Guard heartbeat: while the mission object lives, ``payload/guard``
  carries {"state": "monitoring" | "standby" | "stalled", "robot_inside",
  "seq", "wall_t", "checks", "check_age_s"} at 2 Hz (HUD contract).
  ``checks`` counts the SafetyGuard's completed gas evaluations (only its
  check loop advances it) and ``check_age_s`` is the wall-clock age of the
  last one (T16). "stalled" = the guard was started but its last completed
  evaluation is older than GUARD_STALL_S, or its thread is gone: a loop
  stuck inside a check is not reported as monitoring. "monitoring" = it
  evaluates and is armed (or its retreat is running).
* Holds (T22 and the facade's own health fields): every motion loop but a
  retreat holds zero velocity while ``drive_hold_reason()`` names a
  reason - a second cmd_vel publisher on the bus (payload/status
  ``cmd_vel_publishers`` > 1) or a degraded simulator (``rtf`` below
  RTF_MIN for RTF_DEGRADED_S) - and no entry starts meanwhile. The retreat
  is never held: stopping it would strand Spot inside. The site geometry must match
  agent/site_defaults.json (robot_client.site_mismatch) or the mission
  does not start.
* Door gate (T19): crossing into the space with an ENTRY_AUTHORIZED permit
  first runs ``door_check()``: the battery must be at or above
  ``battery_floor_pct``, forced ventilation must still be running (with a
  current simulator feed) if the authorizing test depended on it, the
  audit log must still be writable, and a short atmospheric re-test at the
  door mouth must pass. Any failure withdraws the authorization (permit
  ABORTED). Ventilation switching off after the test also withdraws an
  authorization, or retreats Spot if it is already inside.
* ``halt_motion()`` is the operator's stop: every mission motion loop
  (walk, turn, test, scripted run) ends and Spot stays standing where it
  is, but a running retreat is left alone; ``cancel()`` remains the hard
  stop of everything.
* Every motion loop ends in a zero twist + ``stop`` call in a ``finally``
  block, so exceptions, timeouts and Ctrl-C never leave Spot walking.
* Robot-side data is typed before use (robot_client.filter_gas /
  filter_status / filter_presenter_event, T6): the guard, the tests and the
  records see only allow-listed keys with checked types, and every audit
  message and string field is stripped of control and escape characters.
  An alarm is labelled presenter-staged only when a presenter disclosure
  was received AND the simulator's payload/gas reported
  ``injected_pockets > 0`` at the time of the alarm (T7); otherwise it is a
  genuine alarm. ``gas_sentence``, ``verdict_sentence`` and
  ``incident_sentence`` generate the spoken gas, verdict and incident
  sentences from the readings in code (T10).
* Simulator only: ``backend()`` identifies the robot backend from
  payload/status (``"simulated": true``, robot_client.BackendIdentity).
  On any other backend every command is refused with the reason (an
  ``backend_refused`` event), and a guard alarm never drives an autonomous
  retreat on geometry nobody surveyed. Until the simulator has been
  identified nothing is sent, not even a zero twist or a stop (T15: on a
  real driver either claims the lease and can power the robot on); once it
  has, stopping is always allowed. There is no override. Once identified, a
  contradiction (a status without the flag, a second payload/status
  publisher) is a backend identity CONFLICT, recorded once as a
  ``backend_conflict`` event: forward motion and entry hold, while the
  stops, the guard's retreat and ``retreat()`` still bring Spot out.

Controller (``walk_to``): proportional heading control on odometry at 20 Hz
wall. Turn in place while |heading error| > 25 deg (until < 10 deg, to avoid
chatter at the threshold); otherwise drive forward at a speed that tapers
toward the goal (0.8 * distance, floor 0.12 m/s, scaled by cos(error)) with
yaw correction. Waypoints are pre-planned obstacle-free, so there is no
obstacle avoidance. Odometry in the sim is the world frame; on the real robot
the route would need to be expressed in its odom frame.

Audit log: JSONL, one record per event (``seq``, ``t`` = sim time when the
facade reports it, ``wall_t`` = unix wall time and ``wall`` = the same as UTC
ISO 8601, permit id and state, ``source``, ``kind``, ``message``, ``data``,
``prev_sha256``), ending with a ``summary`` record; every record is also
published on /demo_spot/payload/events via ``robot.publish_event``, less
the operator's login and SSM session id, which stay in the file. Each
record carries the SHA-256 of the previous line of the file (the first one
of a new file: 64 zeros), so an edited, inserted or deleted record breaks
the chain (``verify_audit_log``); records appended by a later run continue
the chain. The chain head (the file's record count and the SHA-256 of its
last record) goes to the operator console (``on_audit_head``) after the
session, mission-start, operator-confirmation and summary records and when
the log is closed, as one ``[audit head]`` line; with Session Manager
logging on, the session transcript in CloudWatch Logs keeps those lines off
the host, and ``python3 mission.py --verify LOG --heads-from TRANSCRIPT``
checks the file against them: truncation, a rewritten record and a
rewritten chain are reported (T38). The default file is under the
repository's ``logs`` directory, which the agent container mounts from the
host (``default_log_path``). The log fails closed: a log that cannot be
opened stops the mission at construction, and once a write fails no test
authorizes entry, no entry starts, an authorized entry is withdrawn and an
entry in progress retreats.
Permits record roles (``attendant``, ``entry supervisor``, optionally with
a roster id such as "attendant 2"), never personal names (T37).
``mission_start`` records the deployed commit (``SPOT_DEMO_VERSION``, set by
host/run_agent.sh from the checkout's VERSION file; "unknown" without it)
and the run's ``attribution`` (``unattributed_run``, set by the caller from
--unattributed-run; T38). Event kinds:
mission_start, robot, permit, permit_prepared, permit_incomplete,
atmos_test_start, atmos_test (one per gas, in OSHA order), atmos_probe
(per-point rows for the HUD), probe_rejected, probe_retry, probe_unavailable,
purge_incomplete, purge_hold, atmos_result, entry_denied, entry_blocked,
ventilation, sim_control, self_right, robot_fallen, storyline, waypoint,
monitor, alarm, alarm_ignored, retreat_start, retreat_complete,
retreat_incomplete, walk_timeout, pose_lost, walk_refused, turn_refused,
exit_start, survey, demo_inject, gas_injected, gas_cleared, cancelled,
error, summary. Permit events carry the Permit dict as ``data.permit`` (the
HUD checklist reads it). ``last_incident()`` rebuilds the latest SafetyGuard
incident (trigger reading, time, retreat outcome, any disclosed staged
injection) from the in-memory copy of these records.
"""

import argparse
import collections
import dataclasses
import datetime
import hashlib
import inspect
import json
import logging
import math
import os
import re
import sys
import threading
import time
import traceback
from typing import Optional

_AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
if _AGENT_DIR not in sys.path:
    # The sibling robot_client module (pure helpers only; rclpy stays lazy)
    # resolves whether this file runs as a script or as agent.mission.
    sys.path.insert(0, _AGENT_DIR)

from operator_console import AUDIT_HEAD_PREFIX, attribution, operator_identity  # noqa: E402
from robot_client import (BACKEND_CONFLICT, CONFLICT_TEXT, RETREAT_METHODS,  # noqa: E402
                          STOP_METHODS,
                          BackendIdentity, backend_refusal,
                          clean_text, filter_alarms, filter_gas, filter_presenter_event,
                          filter_status, probe_error_text, site_mismatch, trigger_text,
                          without_operator_identity)

LOG = logging.getLogger("mission")

# ---------------------------------------------------------------------------
# Permit states
# ---------------------------------------------------------------------------

NOT_STARTED = "NOT_STARTED"
PRE_ENTRY_TESTING = "PRE_ENTRY_TESTING"
ENTRY_AUTHORIZED = "ENTRY_AUTHORIZED"
ENTRY_IN_PROGRESS = "ENTRY_IN_PROGRESS"
COMPLETED = "COMPLETED"
ABORTED = "ABORTED"
PERMIT_STATES = (NOT_STARTED, PRE_ENTRY_TESTING, ENTRY_AUTHORIZED,
                 ENTRY_IN_PROGRESS, COMPLETED, ABORTED)
TERMINAL_STATES = (COMPLETED, ABORTED)

_TRANSITIONS = {
    NOT_STARTED: (PRE_ENTRY_TESTING, ABORTED),
    PRE_ENTRY_TESTING: (ENTRY_AUTHORIZED, ABORTED),
    ENTRY_AUTHORIZED: (ENTRY_IN_PROGRESS, ABORTED),
    ENTRY_IN_PROGRESS: (COMPLETED, ABORTED),
    COMPLETED: (),
    ABORTED: (),
}

# ---------------------------------------------------------------------------
# OSHA 1910.146 Appendix E thresholds. The simulator's copy lives in
# sim/gas_field.py (GasField.alarms); it cannot be imported here because the
# agent runs in a different container. tests/test_mission.py asserts that the
# two copies stay identical.
# ---------------------------------------------------------------------------

O2_LOW_PCT = 19.5
O2_HIGH_PCT = 23.5
LEL_ALARM_PCT = 10.0
H2S_ALARM_PPM = 10.0
CO_ALARM_PPM = 35.0

OSHA_TEST_ORDER = ("o2", "lel", "h2s", "co")
GAS_KEYS = {"o2": "o2_pct", "lel": "lel_pct", "h2s": "h2s_ppm", "co": "co_ppm"}
GAS_UNITS = {"o2": "%", "lel": "% LEL", "h2s": "ppm", "co": "ppm"}
GAS_LABELS = {"o2": "O2", "lel": "LEL", "h2s": "H2S", "co": "CO"}
GAS_LIMITS = {"o2": "19.5-23.5 %", "lel": "< 10 % LEL", "h2s": "< 10 ppm", "co": "< 35 ppm"}

SITE_KEYS = ("spawn_xy", "spawn_yaw", "entry_xy", "door_xy", "door_normal_yaw",
             "interior_waypoints", "tank_center_xy", "tank_radius")

# Permit fields every entry needs before Spot approaches (API RP 2016 /
# OSHA 1910.146(f) flavoured; the demo records them, it does not verify them).
# Isolation, LOTO, the attendant and the entry supervisor come only from an
# operator attestation typed at the console (agent/operator_console.py),
# never from a tool argument.
PERMIT_FIELD_LABELS = {
    "permit_id": "permit id",
    "isolation_confirmed": "isolation confirmed",
    "loto_confirmed": "LOTO confirmed",
    "attendant": "attendant",
    "rescue_plan": "rescue plan",
    "entry_supervisor": "entry supervisor",
}
ATTENDANT_ROLE = "attendant"
ENTRY_SUPERVISOR_ROLE = "entry supervisor"
# A role may carry a roster id ("attendant 2", "attendant A-7"); never a name.
_ROSTER_ID = re.compile(r"^(?:[A-Z]{1,3}-?)?[0-9]{1,6}$")

# Audit log location and integrity (T38).
LOG_DIR = os.path.join(os.path.dirname(_AGENT_DIR), "logs")   # /workspace/logs in the container
GENESIS_SHA256 = "0" * 64
# What the JSONL is (T38): stated in mission_start and the agent's session record.
RECORD_NOTICE = ("Demonstration record of a SIMULATED confined-space entry (Isaac Sim): not a "
                 "regulatory permit record (OSHA 1910.146(f)) and not evidence that any real "
                 "space was tested or entered")


# The deployed commit (M30): host/run_agent.sh passes the contents of
# $SPOT_DEMO_DIR/VERSION in this variable; every run records it.
VERSION_ENV = "SPOT_DEMO_VERSION"
VERSION_MAX = 80


def deployed_version(env=None):
    """The deployed commit from SPOT_DEMO_VERSION (printable, bounded), or
    "unknown" when it is absent or blank."""
    env = os.environ if env is None else env
    text = " ".join(clean_text(env.get(VERSION_ENV) or "", VERSION_MAX).split())
    return text or "unknown"


def default_log_path(prefix="mission"):
    """logs/<prefix>-<utc>.jsonl under the repository (the agent container
    mounts the host's logs directory there)."""
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
    return os.path.join(LOG_DIR, "%s-%s.jsonl" % (prefix, stamp))


def _line_sha256(line):
    return hashlib.sha256(line.encode("utf-8")).hexdigest()


def _chain_state(path):
    """(records, SHA-256 of the last record) of ``path``, so a run that
    appends continues the chain and its head counts every record in the
    file; (0, GENESIS_SHA256) for a missing or empty file. Blank lines are
    not records (as in ``verify_audit_log``)."""
    count, last = 0, None
    try:
        with open(path, "rb") as handle:
            for raw in handle:
                if raw.strip():
                    count += 1
                    last = raw
    except FileNotFoundError:
        return 0, GENESIS_SHA256
    if last is None:
        return 0, GENESIS_SHA256
    return count, hashlib.sha256(last.rstrip(b"\n")).hexdigest()


# The chain head on the operator console (T38): one line, AUDIT_HEAD_PREFIX
# then the log's file name and <records>:<sha256 of the last record>, printed
# after these records (and when the log is closed: the end of the session).
# With Session Manager logging on, the session transcript in CloudWatch Logs
# keeps every such line, a copy off the host that host users cannot rewrite;
# verify_audit_log(path, expect_heads) checks a log against them.
AUDIT_HEAD_AFTER = {"session": "session start", "mission_start": "mission start",
                    "operator_confirmation": "operator confirmation", "summary": "mission end"}
AUDIT_HEAD_CLOSED = "session end"
_HEAD = re.compile(r"^(\d{1,12}):([0-9a-f]{64})$")
_HEAD_LINE = re.compile(re.escape(AUDIT_HEAD_PREFIX.strip())
                        + r" (\S+) (\d{1,12}):([0-9a-f]{64})\b")


def _head_name(path):
    """The log's file name as audit head lines carry it: printable, no spaces."""
    return "_".join(clean_text(os.path.basename(str(path)), 200).split()) or "-"


def audit_head_line(path, records, sha256, after, failed=False):
    """The console line for a chain head (see AUDIT_HEAD_AFTER)."""
    return "%s%s %d:%s after %s%s" % (
        AUDIT_HEAD_PREFIX, _head_name(path), int(records), sha256, after,
        " (AUDIT LOG WRITE FAILED: later records are not in the file)" if failed else "")


def parse_head(text):
    """(records, sha256) from "<records>:<sha256>"; ValueError otherwise."""
    match = _HEAD.match(str(text).strip().lower())
    if not match:
        raise ValueError("a head is <records>:<64 hex digits of SHA-256>, got %r" % (text,))
    return int(match.group(1)), match.group(2)


def audit_heads_in(text, path=None):
    """Every "<records>:<sha256>" in ``text`` on an audit head line, in order,
    duplicates dropped; only lines naming ``path``'s file when given. Works
    on a session transcript or a CloudWatch Logs export of one."""
    name = _head_name(path) if path is not None else None
    heads = []
    for match in _HEAD_LINE.finditer(str(text)):
        head = "%s:%s" % (int(match.group(2)), match.group(3))
        if (name is None or match.group(1) == name) and head not in heads:
            heads.append(head)
    return heads


def verify_audit_log(path, expect_heads=None):
    """Check the record chain of a mission JSONL and, optionally, the chain
    heads taken from the session log (T38).

    Returns {"ok", "records", "first_bad_line" (1-based, None when the chain
    holds), "reason", "head" ("<records>:<sha256>" of the file's last
    record), "findings"}. ``findings`` lists, for the chain and for each
    expected head ("<records>:<sha256>"), what is wrong:
      * "rewritten": a record's prev_sha256 does not match the line before
        it (a record was edited, inserted or removed without rewriting the
        chain after it), or a line is not JSON;
      * "truncated": the file has fewer records than an expected head;
      * "mismatch": the record at an expected head's count has another
        SHA-256 (the chain was rewritten from there on, or this is another
        log).
    ``ok`` is True only when there is no finding."""
    prev = GENESIS_SHA256
    hashes = []
    first_bad, reason = None, None
    with open(path, "r", encoding="utf-8") as handle:
        for number, raw in enumerate(handle, start=1):
            line = raw.rstrip("\n")
            if not line.strip():
                continue
            if first_bad is None:
                try:
                    record = json.loads(line)
                except ValueError:
                    first_bad, reason = number, "not JSON"
                else:
                    if not isinstance(record, dict) or record.get("prev_sha256") != prev:
                        first_bad, reason = number, "prev_sha256 does not match the previous line"
            prev = _line_sha256(line)
            hashes.append(prev)
    findings = []
    if first_bad is not None:
        findings.append({"kind": "rewritten", "line": first_bad,
                         "message": "line %d: %s: a record at or before it was changed, inserted "
                                    "or removed" % (first_bad, reason)})
    checked = set()
    for expected in expect_heads or ():
        count, sha256 = parse_head(expected)
        head = "%d:%s" % (count, sha256)
        if head in checked:      # mission end and session end often print the same head
            continue
        checked.add(head)
        if count > len(hashes):
            findings.append({"kind": "truncated", "head": head,
                             "message": "the session log shows %d records; the file has %d: "
                                        "records were removed from the end, or the file was "
                                        "replaced" % (count, len(hashes))})
        elif count and hashes[count - 1] != sha256:
            findings.append({"kind": "mismatch", "head": head,
                             "message": "record %d has SHA-256 %s, not %s as in the session log: "
                                        "the chain was rewritten, or this is another log"
                                        % (count, hashes[count - 1], sha256)})
        elif not count and sha256 != GENESIS_SHA256:
            findings.append({"kind": "mismatch", "head": head,
                             "message": "a head of 0 records is the genesis value"})
    return {"ok": not findings, "records": len(hashes), "first_bad_line": first_bad,
            "reason": reason, "head": "%d:%s" % (len(hashes), hashes[-1] if hashes else
                                                 GENESIS_SHA256),
            "findings": findings}


def role_label(value, role):
    """``role`` itself, or ``role`` plus a roster id ("attendant 2",
    "attendant A-7"), normalized; None for anything else (a name)."""
    text = " ".join(str(value).split())
    if text.lower() == role:
        return role
    if text.lower().startswith(role + " "):
        rest = text[len(role) + 1:].upper()
        if _ROSTER_ID.match(rest):
            return "%s %s" % (role, rest)
    return None
CONFIRMED = "confirmed"   # operator_console decision for a "y" answer
PERMIT_ID_MAX = 40
PERMIT_TEXT_MAX = 200
SITE_DEFAULTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "site_defaults.json")


def _num(value):
    """float(value) if it is a finite number, else None."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def gas_passes(gas, value):
    """True if ``value`` is inside the OSHA App. E acceptable range.

    Boundary semantics match sim/gas_field.py: O2 exactly 19.5 or 23.5 is
    acceptable; LEL/H2S/CO exactly at the threshold is an alarm.
    """
    value = _num(value)
    if value is None:
        return False  # no reading is never "safe"
    if gas == "o2":
        return O2_LOW_PCT <= value <= O2_HIGH_PCT
    if gas == "lel":
        return value < LEL_ALARM_PCT
    if gas == "h2s":
        return value < H2S_ALARM_PPM
    if gas == "co":
        return value < CO_ALARM_PPM
    raise ValueError("unknown gas %r" % (gas,))


# Units for one-line messages ("LEL 12.0 % >= 10 %" reads better than "% LEL").
_MSG_UNITS = {"o2": "%", "lel": "%", "h2s": "ppm", "co": "ppm"}


def _exceedance_text(gas, value):
    label, unit = GAS_LABELS[gas], _MSG_UNITS[gas]
    if gas == "o2":
        side = "below 19.5" if value < O2_LOW_PCT else "above 23.5"
        return "%s %.1f %s %s %%" % (label, value, unit, side)
    limit = {"lel": LEL_ALARM_PCT, "h2s": H2S_ALARM_PPM, "co": CO_ALARM_PPM}[gas]
    return "%s %.1f %s >= %g %s" % (label, value, unit, limit, unit)


def alarm_reasons(reading):
    """Human-readable reasons ``reading`` (a payload/gas dict) is an alarm.

    Empty list = acceptable atmosphere. Fail-safe: a missing or non-numeric
    value counts as an alarm, and the facade's own alarm flag is honoured
    even if our thresholds disagree (defence in depth).
    """
    reasons = []
    for gas in OSHA_TEST_ORDER:
        value = _num(reading.get(GAS_KEYS[gas]))
        if value is None:
            reasons.append("%s reading missing" % GAS_LABELS[gas])
        elif not gas_passes(gas, value):
            reasons.append(_exceedance_text(gas, value))
    flags = filter_alarms(reading.get("alarms"))
    if not reasons and (reading.get("alarm") is True or flags.get("any") is True):
        # Known flag names only: a flag key is never free text in a reason.
        which = [k for k in OSHA_TEST_ORDER if flags.get(k) is True]
        reasons.append("facade alarm flag set (%s)" % (",".join(which) or "any"))
    return reasons


def format_reading(reading):
    """One-line gauge text, e.g. 'O2 20.9 % | LEL 0.0 % | H2S 0.0 ppm | CO 0.0 ppm'."""
    parts = []
    for gas in OSHA_TEST_ORDER:
        value = _num(reading.get(GAS_KEYS[gas]))
        parts.append("%s %s %s" % (GAS_LABELS[gas],
                                   "?" if value is None else "%.1f" % value, _MSG_UNITS[gas]))
    return " | ".join(parts)


def _worst_value(gas, values):
    """Worst of ``values`` for ``gas``: the max for LEL/H2S/CO; for O2
    whichever side is closer to (or past) its limit."""
    if gas == "o2":
        lo, hi = min(values), max(values)
        return lo if (lo - O2_LOW_PCT) <= (O2_HIGH_PCT - hi) else hi
    return max(values)


# Spoken words for the code-generated sentences (T10): numbers and verdicts
# come from the readings here, never from the model's paraphrase.
_SPOKEN_GAS = {"o2": "oxygen", "lel": "LEL", "h2s": "hydrogen sulfide", "co": "carbon monoxide"}
_SPOKEN_UNIT = {"o2": "percent", "lel": "percent", "h2s": "ppm", "co": "ppm"}
_SPOKEN_LIMIT = {"o2": "19.5 to 23.5 percent", "lel": "below 10 percent",
                 "h2s": "below 10 ppm", "co": "below 35 ppm"}


def _spoken_list(items):
    items = [i for i in items if i]
    if len(items) <= 1:
        return "".join(items)
    return ", ".join(items[:-1]) + " and " + items[-1]


def gas_sentence(reading, where="at the robot"):
    """One sentence for a payload/gas reading, generated from its values."""
    if not isinstance(reading, dict):
        return "There is no current gas reading."
    parts, alarming = [], []
    for gas in OSHA_TEST_ORDER:
        value = _num(reading.get(GAS_KEYS[gas]))
        if value is None:
            parts.append("no %s reading" % _SPOKEN_GAS[gas])
            alarming.append(gas)
            continue
        parts.append("%s %.1f %s" % (_SPOKEN_GAS[gas], value, _SPOKEN_UNIT[gas]))
        if not gas_passes(gas, value):
            alarming.append(gas)
    text = "%s %s" % (_spoken_list(parts), where)
    text = text[:1].upper() + text[1:]
    if alarming:
        return "%s; %s %s in alarm (limits: %s)." % (
            text, _spoken_list(_SPOKEN_GAS[g] for g in alarming),
            "is" if len(alarming) == 1 else "are",
            _spoken_list("%s %s" % (_SPOKEN_GAS[g], _SPOKEN_LIMIT[g]) for g in alarming))
    return "%s; no gas is in alarm." % text


def verdict_sentence(test):
    """The spoken verdict of an atmospheric_test() result, from its values."""
    if not isinstance(test, dict):
        return "No atmospheric test result is available."
    results = [e for e in test.get("results") or [] if isinstance(e, dict)]
    readings = _spoken_list("%s %.1f %s" % (_SPOKEN_GAS.get(e.get("gas"), e.get("gas")),
                                            float(e["value"]), _SPOKEN_UNIT.get(e.get("gas"), ""))
                            for e in results if _num(e.get("value")) is not None)
    failed = [e for e in results if e.get("passed") is False]
    if test.get("passed") is True:
        return ("Atmospheric test passed: %s at the worst sampled points; the readings are "
                "within permissible entry limits." % readings)
    if failed:
        bad = []
        for e in failed:
            gas, value = e.get("gas"), _num(e.get("value"))
            height = _num(e.get("worst_z_m"))
            if value is None:
                bad.append("no %s reading" % _SPOKEN_GAS.get(gas, gas))
            else:
                bad.append("%s %.1f %s%s against a limit of %s" % (
                    _SPOKEN_GAS.get(gas, gas), value, _SPOKEN_UNIT.get(gas, ""),
                    " at %.1f meters" % height if height is not None else "",
                    _SPOKEN_LIMIT.get(gas, "?")))
        return ("Atmospheric test failed: %s; the readings exceed the permissible entry limits, "
                "so entry is not authorized." % _spoken_list(bad))
    if test.get("purge_incomplete"):
        hold = test.get("purge_hold") if isinstance(test.get("purge_hold"), dict) else {}
        pct, target = _num(hold.get("percent_remaining")), _num(hold.get("purge_target"))
        return ("The readings are within permissible entry limits at the door (%s), but the "
                "ventilation purge target is not reached yet%s, so entry is not authorized." % (
                    readings, ": %.1f percent of the contamination remains and entry needs %g "
                    "percent or less" % (pct, round(100.0 * target, 1))
                    if pct is not None and target is not None else ""))
    if test.get("sampled_inside") is False:
        return ("The gas probe did not sample inside the opening, so this test cannot authorize "
                "entry.")
    failures = [str(f) for f in test.get("failures") or []]
    return ("The atmospheric test did not authorize entry%s."
            % (": " + clean_text(failures[0], 300) if failures else ""))


_RETREAT_SPOKEN = {"complete": "the guard stopped the robot and walked it out, and the retreat "
                               "completed",
                   "INCOMPLETE": "the guard stopped the robot but the retreat did NOT complete: "
                                 "operator attention is needed",
                   "in progress": "the guard stopped the robot and the retreat is still running",
                   "not started": "the guard stopped the robot"}


def incident_sentence(report):
    """The spoken account of a ``last_incident()`` report, from its values."""
    if not isinstance(report, dict) or not report.get("found"):
        return "No SafetyGuard incident is recorded in this session."
    trigger = report.get("trigger") if isinstance(report.get("trigger"), dict) else {}
    reading = trigger.get("reading") if isinstance(trigger.get("reading"), dict) else {}
    gases = [g.lower() for g in trigger.get("gases_in_alarm") or [] if isinstance(g, str)]
    if trigger.get("telemetry_lost"):
        what = "gas telemetry was lost (continuous monitoring is a permit condition)"
    elif gases:
        what = _spoken_list("%s read %s against a limit of %s" % (
            _SPOKEN_GAS.get(g, g), "%.1f %s" % (reading[GAS_KEYS[g]], _SPOKEN_UNIT[g])
            if _num(reading.get(GAS_KEYS.get(g, ""))) is not None else "an unknown value",
            _SPOKEN_LIMIT.get(g, "?")) for g in gases if g in GAS_KEYS)
    else:
        what = "the facade's gas alarm flag was set"
    location = report.get("location") if isinstance(report.get("location"), dict) else {}
    where = ""
    if location:
        where = " with Spot %s the permit space near %s" % (
            "inside" if location.get("inside_permit_space") else "outside",
            location.get("nearest_place") or "an unknown place")
    retreat = report.get("retreat") if isinstance(report.get("retreat"), dict) else {}
    status = str(retreat.get("status") or "not started")
    reaction = _RETREAT_SPOKEN.get(status, "the automatic retreat was %s" % status)
    text = ("SafetyGuard incident %s: %s%s; %s; the permit is aborted, so re-entry needs a new "
            "permit and a new atmospheric test." % (report.get("incident_id") or "?", what, where,
                                                     reaction))
    if report.get("staged_injection"):
        text += (" The simulator reported an injected gas pocket and a presenter disclosure was "
                 "received: this was a presenter-staged simulated spike.")
    else:
        text += " No staged injection was active at the time: treat it as a genuine alarm."
    return text


def _utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


@dataclasses.dataclass
class Permit:
    """Entry permit fields recorded before Spot may approach the entry.

    ``issued_at`` (UTC ISO 8601) is stamped when the last missing field is
    recorded. The booleans count only when exactly True: a "yes" string or
    1 is not a confirmation. ``attestation`` is the operator's console
    record (typed answer, time, operator) behind isolation, LOTO, the
    attendant and the entry supervisor.
    """

    permit_id: str
    isolation_confirmed: bool = False
    loto_confirmed: bool = False
    attendant: str = ""
    rescue_plan: str = ""
    issued_at: Optional[str] = None
    entry_supervisor: str = ""
    attestation: Optional[dict] = None

    def missing(self):
        """Names of the fields still missing, in checklist order."""
        out = []
        if not str(self.permit_id or "").strip():
            out.append("permit_id")
        if self.isolation_confirmed is not True:
            out.append("isolation_confirmed")
        if self.loto_confirmed is not True:
            out.append("loto_confirmed")
        if not str(self.attendant or "").strip():
            out.append("attendant")
        if not str(self.rescue_plan or "").strip():
            out.append("rescue_plan")
        if not str(self.entry_supervisor or "").strip():
            out.append("entry_supervisor")
        return out

    @property
    def complete(self):
        return not self.missing()

    def as_dict(self):
        out = dataclasses.asdict(self)
        out["missing"] = self.missing()
        out["complete"] = not out["missing"]
        return out


def describe_missing(fields):
    """'LOTO confirmed and rescue plan' style text for missing permit fields."""
    names = [PERMIT_FIELD_LABELS.get(f, f) for f in fields]
    if len(names) <= 1:
        return "".join(names)
    return ", ".join(names[:-1]) + " and " + names[-1]


def wrap_angle(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _jsonable(obj):
    """Strict-JSON-safe copy (tuples -> lists, NaN/inf -> None), with
    control and escape characters stripped from every string (T10)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {clean_text(k, 200): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, str):
        return clean_text(obj, None)
    if obj is None or isinstance(obj, (bool, int)):
        return obj
    return clean_text(obj, None)


# ---------------------------------------------------------------------------
# Site geometry
# ---------------------------------------------------------------------------


def load_site_defaults(path=SITE_DEFAULTS_PATH):
    """agent/site_defaults.json (values copied from sim/build_scene.py)."""
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _xy(value, name):
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        raise ValueError("site %s must be [x, y], got %r" % (name, value)) from None
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("site %s is not finite: %r" % (name, value))
    return (x, y)


def normalize_site(site):
    """Validate a contract-1 site dict; fill missing keys from the defaults.

    Accepts build_scene's ``doorway_xy`` spelling for ``door_xy``. Returns
    plain tuples/floats. Raises ValueError on malformed geometry, because a
    wrong route inside a tank is worse than not starting.
    """
    src = dict(site or {})
    if "door_xy" not in src and "doorway_xy" in src:
        src["door_xy"] = src["doorway_xy"]
    missing = [k for k in SITE_KEYS if k not in src]
    if missing:
        defaults = load_site_defaults()
        LOG.warning("site lacks %s; using agent/site_defaults.json values", missing)
        for key in missing:
            src[key] = defaults[key]
    waypoints = [_xy(p, "interior_waypoints[%d]" % i)
                 for i, p in enumerate(src["interior_waypoints"])]
    if not waypoints:
        raise ValueError("site interior_waypoints is empty")
    radius = _num(src["tank_radius"])
    if radius is None or radius <= 0.0:
        raise ValueError("site tank_radius must be > 0, got %r" % (src["tank_radius"],))
    for key in ("spawn_yaw", "door_normal_yaw"):
        if _num(src[key]) is None:
            raise ValueError("site %s must be a finite number" % key)
    return {
        "spawn_xy": _xy(src["spawn_xy"], "spawn_xy"),
        "spawn_yaw": float(src["spawn_yaw"]),
        "entry_xy": _xy(src["entry_xy"], "entry_xy"),
        "door_xy": _xy(src["door_xy"], "door_xy"),
        "door_normal_yaw": float(src["door_normal_yaw"]),
        "interior_waypoints": waypoints,
        "tank_center_xy": _xy(src["tank_center_xy"], "tank_center_xy"),
        "tank_radius": radius,
    }


def _takes_keyword(fn, name):
    """True if callable ``fn`` accepts keyword argument ``name``."""
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False
    return name in params or any(p.kind == p.VAR_KEYWORD for p in params.values())


class RealClock:
    """Wall clock. Tests inject a faster clock with the same two methods."""

    @staticmethod
    def now():
        return time.monotonic()

    @staticmethod
    def sleep(seconds):
        if seconds > 0.0:
            time.sleep(seconds)


# ---------------------------------------------------------------------------
# SafetyGuard
# ---------------------------------------------------------------------------


class SafetyGuard:
    """Deterministic gas watchdog: polls ``robot.gas()`` in its own thread.

    ``on_alarm(reading, reasons)`` is called once per excursion (rising edge
    of "armed AND alarm"); the latch re-opens when the alarm clears or on
    ``reset()``. ``reading`` is None when the trigger is telemetry loss.

    Args:
        robot: RobotClient-like; only ``gas()`` is used.
        on_alarm: callback, runs in the guard thread - keep it short.
        period_s: poll period; must be <= 0.25 s (>= 4 Hz contract).
        stale_s: gas() returning None this long while armed is an alarm.
        armed: optional zero-arg predicate; default always armed.
        on_sample: optional ``on_sample(reading, armed)`` per poll.
        clock: object with now()/sleep() (default wall clock).
    """

    def __init__(self, robot, on_alarm, period_s=0.05, stale_s=5.0, armed=None,
                 on_sample=None, clock=None):
        if not 0.0 < float(period_s) <= 0.25:
            raise ValueError("SafetyGuard period_s must be in (0, 0.25] (>= 4 Hz)")
        self.robot = robot
        self.on_alarm = on_alarm
        self.period_s = float(period_s)
        self.stale_s = float(stale_s)
        self._armed_fn = armed
        self._on_sample = on_sample
        self._clock = clock or RealClock()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latched = False
        self._none_since = None
        self._armed = False
        self._last_reading = None
        self._last_reasons = []
        self._fires = 0
        self._polls = 0
        # Evidence that the loop evaluates (T16): completed checks and when
        # the last one finished, in wall-clock monotonic time (a stuck check
        # is a real-time stall whatever ``clock`` says).
        self._checks = 0
        self._checked_at = None
        self._started_at = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return
        self._stop.clear()
        with self._lock:
            self._started_at = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="safety_guard", daemon=True)
        self._thread.start()

    def stop(self, timeout=1.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def reset(self):
        """Re-open the latch (next alarm while armed fires again)."""
        with self._lock:
            self._latched = False
            self._none_since = None

    def snapshot(self):
        """State, with ``checks`` (completed evaluations) and ``check_age_s``
        (wall-clock seconds since the last one finished, or since start()
        if none has; None if never started)."""
        with self._lock:
            since = self._checked_at if self._checked_at is not None else self._started_at
            return {
                "running": self.running,
                "armed": self._armed,
                "latched": self._latched,
                "fires": self._fires,
                "polls": self._polls,
                "checks": self._checks,
                "check_age_s": None if since is None else max(0.0, time.monotonic() - since),
                "last_reasons": list(self._last_reasons),
                "last_reading": dict(self._last_reading) if self._last_reading else None,
            }

    def check_once(self):
        """One evaluation step. Returns the reasons if it fired, else [].
        Only a step that completes (alarm callback included) counts as a
        check in ``snapshot()``."""
        reasons = self._evaluate()
        with self._lock:
            self._checks += 1
            self._checked_at = time.monotonic()
        return reasons

    def _evaluate(self):
        now = self._clock.now()
        try:
            reading = filter_gas(self.robot.gas())  # typed: allow-listed keys only (T6)
        except Exception as error:  # a comms error is "no reading", not a crash
            LOG.warning("guard: gas() raised %s", error)
            reading = None
        try:
            armed = True if self._armed_fn is None else bool(self._armed_fn())
        except Exception as error:  # unknown -> assume armed (fail-safe)
            LOG.warning("guard: armed predicate raised %s", error)
            armed = True
        if self._on_sample is not None and reading is not None:
            try:
                self._on_sample(reading, armed)
            except Exception:
                LOG.exception("guard: on_sample callback failed")

        reasons = []
        with self._lock:
            self._polls += 1
            self._armed = armed
            if reading is None:
                if not armed:
                    self._none_since = None
                elif self._none_since is None:
                    self._none_since = now
                elif now - self._none_since >= self.stale_s:
                    reasons = ["gas telemetry lost for %.1f s (continuous monitoring "
                               "is a permit condition)" % (now - self._none_since)]
            else:
                self._none_since = None
                self._last_reading = reading
                reasons = alarm_reasons(reading)
            if not reasons:
                self._latched = False  # excursion over: next one fires again
                return []
            if not armed or self._latched:
                return []
            self._latched = True
            self._fires += 1
            self._last_reasons = list(reasons)
        try:
            self.on_alarm(reading, reasons)
        except Exception:
            LOG.exception("guard: on_alarm callback failed")
        return reasons

    def _loop(self):
        next_t = self._clock.now()
        while not self._stop.is_set():
            try:
                self.check_once()
            except Exception:  # never let the watchdog die silently
                LOG.exception("guard: check failed")
            next_t += self.period_s
            delay = next_t - self._clock.now()
            if delay < -self.period_s:
                next_t = self._clock.now()  # fell behind: don't burst-catch-up
                delay = 0.0
            self._clock.sleep(max(0.0, delay))


# ---------------------------------------------------------------------------
# ConfinedSpaceMission
# ---------------------------------------------------------------------------


class ConfinedSpaceMission:
    """Scripted / step-by-step confined-space entry (see module docstring).

    Args:
        robot: RobotClient (or compatible fake).
        site: contract-1 site dict (``robot.site()``); validated by
            ``normalize_site``.
        log_path: JSONL audit log path (appended); None disables the file.
        clock: object with now()/sleep(); default wall clock.
        dwell_s: atmospheric-test dwell used by ``run()``.
        auto_retreat: default for the guard's response (``run()`` overrides).
        require_permit: refuse to approach the entry, and never authorize
            entry, until ``prepare_permit()`` has recorded every Permit
            field. Default True; False only for tests of the motion logic.
        require_probe: only a test whose every gas was read by the gas
            probe INSIDE the opening can authorize (or confirm) entry; a
            body-sensor or away-from-the-door reading taken outside the
            tank never samples the space. Default: same as require_permit.
        purge_target: ventilation fraction_remaining a ventilated (or
            earlier-failed) tank must reach before a passing test can
            authorize entry. Default VENT_PURGE_TARGET_FRACTION (0.08).
    """

    # Controller / behaviour tuning. Class attributes so tests or the agent
    # can override per instance without touching the code.
    CONTROL_HZ = 20.0            # cmd_vel rate; facade watchdog is 0.125 s
    TURN_IN_PLACE_DEG = 25.0     # contract: turn in place above this error
    TURN_RESUME_DEG = 10.0       # ...until below this (hysteresis)
    K_TURN = 1.2                 # rad/s per rad, turning in place
    K_YAW = 1.5                  # rad/s per rad, yaw correction while driving
    MAX_WZ = 0.8
    MAX_WZ_DRIVE = 0.6
    MIN_TURN_WZ = 0.25           # below this the policy barely rotates
    K_V = 0.8                    # m/s per m of remaining distance (taper)
    MIN_V = 0.12
    REVERSE_MAX_SPEED = 0.3      # backing up is slower and less stable
    POSE_LOST_S = 2.0
    NEAR_DOOR_M = 0.9            # no turning in place this close to the door
    UNPLANNED_EXIT_M = 1.5       # walk_to straight out only from this close to the door
    DOOR_MOUTH_OFFSET_M = 0.6    # test point: Spot's nose (0.55 m) at the opening
    SIT_STANDOFF_M = 2.0         # sit >= this far from the door when it is behind Spot
    SPACE_MARGIN_M = 0.3         # "inside" = within tank_radius + margin
    STAND_TIMEOUT_S = 15.0
    SURVEY_DWELL_S = 2.0
    MONITOR_EVENT_PERIOD_S = 5.0  # audit cadence for readings (design: no gap > 30 s)
    RETREAT_TIMEOUT_S = 240.0
    RETREAT_ARRIVE_TOL_M = 0.6
    RETREAT_LEG_RETRIES = 3      # a leg that loses its pose (or times out) is retried
    RETREAT_POSE_WAIT_S = 10.0   # ...after holding zero velocity this long for the pose
    # A guard retreat that ends incomplete while the alarm persists is tried
    # again this many times, after this pause (zero velocity), and every
    # failure is shown to the operator (T17).
    RETREAT_RETRIES = 2
    RETREAT_RETRY_PAUSE_S = 5.0
    # "Retreat complete" needs two distinct fresh odometry samples from the
    # identified facade node, both outside at the retreat's end (T22).
    RETREAT_CONFIRM_S = 3.0
    HEARTBEAT_PERIOD_S = 0.5     # payload/guard at 2 Hz (wall clock)
    # The heartbeat says "stalled" once the guard's last completed gas
    # evaluation is older than this (wall clock; it runs every 50 ms, T16).
    GUARD_STALL_S = 2.0
    STOPPED_TEXT = "stopped by the operator at the console"
    RTF_MIN = 0.5                # simulator real-time factor below this...
    RTF_DEGRADED_S = 3.0         # ...for this long: a degraded simulator, hold and no entry
    DOOR_RETEST_DWELL_S = 1.5    # the re-test at the door just before crossing
    DOOR_CHECK_MAX_AGE_S = 20.0  # a passed door check covers a crossing this soon after
    ENTRY_BATTERY_FLOOR_PCT = 30.0  # no entry below this charge (sim demo battery floor: 30 %)
    ATMOS_SAMPLE_PERIOD_S = 0.25  # payload/gas rate
    GUARD_PERIOD_S = 0.05
    GAS_STALE_S = 5.0
    WAYPOINT_TOL_M = 0.25
    CRUISE_SPEED = 0.5
    # Permit gate: no approach within tank_radius + this until the permit is
    # complete. 1.5 m keeps entry_xy (1 m from the shell) inside the zone and
    # the sit standoff (2 m from the door) outside it.
    APPROACH_MARGIN_M = 1.5
    # Gas probe (facade payload/gas/probe): an extendable gas probe,
    # SIMULATED in this demo (arm-mounted on the real robot; Spot Arm reach
    # is ~0.98 m, and the facade rejects points beyond its simulated reach
    # or behind the tank wall).
    PROBE_LABEL = "extendable gas probe (simulated; arm-mounted on the real robot)"
    PROBE_INSIDE_M = 0.5            # nearest sample depth inside the opening (compat alias)
    PROBE_DEPTHS_M = (0.5, 1.0)     # sample depths inside the opening, along the door normal
    PROBE_HEIGHTS_M = (0.2, 0.6, 1.0)  # H2S pools low, so the lowest height matters most
    PROBE_TIMEOUT_S = 3.0
    PROBE_PERIOD_S = 0.5            # probe requests per test phase: one per this period
    PROBE_DOOR_RANGE_M = 1.5        # Spot this close to door_mouth_xy -> probe the opening
    # Seating at the door mouth for a door test. The facade's reach is
    # measured from the nose (0.55 m ahead of the body centre): at
    # door_mouth_xy the 1.0 m station is 1.05 m from it, so Spot must stop
    # within a few cm of door_mouth_xy. It walks toward a point
    # PROBE_SEAT_LEAD_M nearer the door and stops PROBE_SEAT_TOL_M short of
    # it: coming from outside, that is ~0.02 m inside door_mouth_xy (nose
    # ~3 cm outside the door plane), with the heading error still small (no
    # turn in place by the door), and the nose never passes the door plane.
    # The window is narrow by design: nose outside the plane (x >= 6.0) and
    # the 1.0 m points within the 1.1 m reach (nose x <= ~6.1). Host
    # 2026-09-27 with lead 0.10: seated 0.02 m outside door_mouth_xy, the
    # standing body drifted ~3 cm further out within 2 s, the 1.0 m points
    # hit 1.10 m, the re-seat stopped at the same tolerance edge, drifted
    # again, and the storyline ABORTED. Lead 0.16 keeps ~4 cm for that drift
    # (tests/test_gas_field.py TestProbeFromMissionSeat checks the envelope).
    PROBE_SEAT_LEAD_M = 0.16
    PROBE_SEAT_TOL_M = 0.14
    # Already seated (no pre-test hop): every door station within the
    # facade's reach minus this margin from the nose, facing the door, near
    # its axis, nose outside the door plane. Presenter dry run 2026-09-27:
    # after the purge wait Spot stood 0.147 m from the seat target (the
    # standing drift), the re-test hopped 1 cm, the stop transient carried
    # it 7 cm outward and the probe was rejected twice; it had been able to
    # probe from where it stood (1.04 m). Reach/nose offset come from the
    # site's advertised "probe" block when present.
    PROBE_REACH_DEFAULT_M = 1.1         # sim/gas_field.py PROBE_MAX_REACH_M
    PROBE_FRONT_OFFSET_DEFAULT_M = 0.55  # body centre -> front of the body
    PROBE_SEATED_REACH_MARGIN_M = 0.04  # room for the ~3 cm standing drift
    PROBE_SEATED_MAX_YAW_DEG = 10.0
    PROBE_SEATED_MAX_LATERAL_M = 0.15
    PROBE_REPOSITION_TOL_DEG = 4.0  # after a rejection: face the door this precisely
    # Short seat corrections strafe instead of steering (``_seat_step``).
    # Leadership rehearsal 2026-09-28 (lr3-rep1, GUI, agent REPL): Spot stood
    # 0.152 m from the seat target at yaw 169 deg (11 deg off the door axis,
    # so not "seated"); the unicycle hop to a target that close chased its
    # bearing, swung the body 169 -> 94 deg, the facade rejected the probe,
    # face() may not turn 69 deg that close to the door, and the next two
    # re-seats ended facing AWAY from the door (yaw -6 / -9 deg): entry stayed
    # denied for the rest of the session. Spot is holonomic (cmd_vel
    # linear.y, docs/facade-contract-v1.md), so hops up to SEAT_STEP_MAX_M
    # move in the body frame while holding / squaring the door heading.
    SEAT_STEP_MAX_M = 0.5
    SEAT_STEP_TOL_M = 0.06
    # 6, not 4: at 4 deg the MIN_TURN_WZ nudges overshot back and forth for
    # the whole timeout (host, lr7-clean: 184 deg after the step, 12 s),
    # which pushed the early re-test past the purge-hold window. The probe
    # envelope allows 10 deg.
    SEAT_STEP_YAW_TOL_DEG = 6.0
    SEAT_STEP_MAX_V = 0.2        # m/s
    SEAT_STEP_MIN_V = 0.08       # below this the policy barely steps
    SEAT_STEP_K_V = 1.0          # m/s per m
    SEAT_STEP_K_YAW = 1.2        # rad/s per rad
    SEAT_STEP_MAX_WZ = 0.4
    SEAT_STEP_MAX_YAW_FIX_DEG = 45.0  # beyond this: hold the heading, no big turn at the door
    SEAT_STEP_TIMEOUT_S = 8.0
    # Stand verification (host F1: a re-stand after a sit fell within ~2 s).
    UPRIGHT_MIN_Z_M = 0.3           # standing body height is ~0.5 m
    STAND_SETTLE_S = 2.5
    SELF_RIGHT_ATTEMPTS = 2
    SELF_RIGHT_WAIT_S = 3.0
    SELF_RIGHT_RELOCATE_M = 1.0     # moved more than this during self_right: the sim teleported it
    # SIMULATED forced ventilation (facade payload/gas/ventilation).
    VENT_DEFAULT_TAU_S = 15.0       # facade default (sim/gas_field.py DEFAULT_VENT_TAU_S)
    VENT_TAU_RANGE_S = (2.0, 600.0)
    # Purge target: entry after ventilation needs the sim's fraction_remaining
    # (1.0 = the unventilated tank) at or below this. 0.08 keeps every point
    # of the interior route at z 0.1-1.2 m below every OSHA alarm in the
    # facade's seed-7 field with the unventilated headspace (the door probe
    # alone passes from ~0.19; tests/test_mission.py checks both numbers).
    VENT_PURGE_TARGET_FRACTION = 0.08
    VENT_ASSUMED_RESIDUAL = 0.04    # sim/gas_field.py default; local estimate only
    VENT_PURGE_TAUS = 3.0           # estimate shown before the sim confirms a command
    VENT_PROGRESS_S = 5.0
    VENT_MAX_WAIT_S = 600.0
    VENT_CONFIRM_S = 2.0            # the sim's gas block confirms a command within ~1 sample
    INCIDENT_RECORDS = 2000         # audit records kept in memory for last_incident()

    def __init__(self, robot, site, log_path, clock=None, dwell_s=4.0, auto_retreat=True,
                 require_permit=True, require_probe=None, purge_target=None,
                 battery_floor_pct=None, new_log=False):
        self.robot = robot
        self.site = normalize_site(site)
        mismatch = site_mismatch(self.site, load_site_defaults())
        if mismatch:
            # Reviewed geometry only (T22): a wrong route inside a tank is
            # worse than not starting.
            raise ValueError("site geometry is not the reviewed agent/site_defaults.json: %s"
                             % mismatch)
        self.log_path = log_path
        self.dwell_s = float(dwell_s)
        self.auto_retreat = bool(auto_retreat)
        self.require_permit = bool(require_permit)
        self.require_probe = self.require_permit if require_probe is None else bool(require_probe)
        target = self.VENT_PURGE_TARGET_FRACTION if purge_target is None else _num(purge_target)
        if target is None or not 0.0 < target < 1.0:
            raise ValueError("purge_target must be a fraction in (0, 1), got %r" % (purge_target,))
        self.purge_target = target
        floor = self.ENTRY_BATTERY_FLOOR_PCT if battery_floor_pct is None else _num(battery_floor_pct)
        if floor is None or not 0.0 <= floor <= 100.0:
            raise ValueError("battery_floor_pct must be 0-100, got %r" % (battery_floor_pct,))
        self.battery_floor_pct = floor
        self._clock = clock or RealClock()

        s = self.site
        self.entry_xy = s["entry_xy"]
        self.door_xy = s["door_xy"]
        self.center_xy = s["tank_center_xy"]
        self.radius = s["tank_radius"]
        self.waypoints = list(s["interior_waypoints"])
        ox, oy = self.door_xy[0] - self.center_xy[0], self.door_xy[1] - self.center_xy[1]
        norm = math.hypot(ox, oy)
        if norm < 1e-6:  # degenerate: door at the center; trust the site's normal
            ox, oy, norm = math.cos(s["door_normal_yaw"]), math.sin(s["door_normal_yaw"]), 1.0
        self.door_outward = (ox / norm, oy / norm)
        self.door_mouth_xy = (self.door_xy[0] + self.door_outward[0] * self.DOOR_MOUTH_OFFSET_M,
                              self.door_xy[1] + self.door_outward[1] * self.DOOR_MOUTH_OFFSET_M)
        # Probe stations: PROBE_DEPTHS_M inside the opening, along the door
        # normal (probe_xy = the nearest, kept for older consumers).
        self.probe_stations_xy = [
            (self.door_xy[0] - self.door_outward[0] * d, self.door_xy[1] - self.door_outward[1] * d)
            for d in self.PROBE_DEPTHS_M]
        self.probe_xy = self.probe_stations_xy[0]
        rules = site.get("probe") if isinstance(site, dict) else None
        rules = rules if isinstance(rules, dict) else {}
        reach, front = _num(rules.get("max_reach_m")), _num(rules.get("body_front_offset_m"))
        self.probe_reach_m = reach if reach is not None and reach > 0 else self.PROBE_REACH_DEFAULT_M
        self.probe_front_m = (front if front is not None and front >= 0
                              else self.PROBE_FRONT_OFFSET_DEFAULT_M)

        self._lock = threading.RLock()         # mission state
        self._log_lock = threading.Lock()      # audit file + seq
        self._motion_lock = threading.Lock()   # one mission motion at a time
        self._halt = threading.Event()         # set by retreat/cancel: motion loops exit
        self._cancel = threading.Event()
        self._retreat_done = threading.Event()
        self._retreat_done.set()
        # Bumped by every retreat/cancel. ``_halt`` is cleared when a retreat
        # ends, so a mission loop that sleeps through a whole (fast) retreat
        # would never see it; comparing epochs catches that.
        self._motion_epoch = 0
        self._operator_halt_epoch = None       # epoch of the last halt_motion()
        self._rtf_low_since = None             # monotonic (mission clock) rtf went below RTF_MIN
        self._hold_cache = (-1.0e9, None)      # (when, reason) of the last drive_hold_reason()
        self._run_epoch = None                 # epoch when the scripted run started
        self._gate = callable(getattr(robot, "set_motion_owner", None))

        self._backend = BackendIdentity()      # for robot links without backend()
        self._conflict_recorded = False        # the backend_conflict event is written once
        # confirm_action(action, summary, details) -> (ok, record): the
        # operator's console confirmation (set by the agent). None = no
        # console, so actions that need one (self_right inside the permit
        # space) are refused.
        self.confirm_action = None
        # notify_operator(text): a console notice for the operator (set by
        # the agent or run_mission.py); every call is also logged as ERROR.
        self.notify_operator = None
        self._gas_hold_since = None            # when motion began holding for missing gas (T17)
        self._seq = 0
        self._records = collections.deque(maxlen=self.INCIDENT_RECORDS)
        self._incident_serial = 0              # SafetyGuard alarms, never reset
        self._cleared_injections = set()       # presenter disclosures whose pockets we cleared
        self._vent = {"on": False, "tau_s": None, "t_on": None}  # what we last commanded
        self._vent_seen = None                 # (latest payload/gas ventilation block, when)
        self._log_file = None
        self._log_error_reported = False
        self._log_failed = False               # sticky: a write failed (fail closed, T38)
        self._prev_sha256 = GENESIS_SHA256
        self._chain_records = 0                # records in the file (the chain head's count)
        # on_audit_head(line): the chain head on the operator console (set by
        # the agent or run_mission.py; T38). unattributed_run: the operator's
        # --unattributed-run, recorded in mission_start's attribution.
        self.on_audit_head = None
        self.unattributed_run = False
        if log_path:
            directory = os.path.dirname(os.path.abspath(log_path))
            os.makedirs(directory, exist_ok=True)
            if new_log:
                # A run's own new log (the default path, new_log=True): created
                # here, exclusively, or the mission does not start. Anything
                # already at that path, a file or a link that someone could
                # have planted with a forged but well-chained history, is never
                # adopted (FileExistsError, an OSError; T38).
                self._log_file = open(log_path, "x", encoding="utf-8", buffering=1)  # nosemgrep: open-never-closed
            else:
                self._chain_records, self._prev_sha256 = _chain_state(log_path)
                # Append to a log the operator named: an audit trail is never
                # overwritten by the next run, and its chain continues.
                # Raises (OSError) if it cannot be opened: the mission does not start.
                self._log_file = open(log_path, "a", encoding="utf-8", buffering=1)  # nosemgrep: open-never-closed

        self._reset_permit_state()
        self.guard = SafetyGuard(robot, self._on_alarm, period_s=self.GUARD_PERIOD_S,
                                 stale_s=self.GAS_STALE_S, armed=self._guard_armed,
                                 on_sample=self._on_sample, clock=self._clock)
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self._heartbeat_seq = 0

    def _reset_permit_state(self):
        with self._lock:
            self.permit_id = "CSE-" + datetime.datetime.now(
                datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
            self._permit = NOT_STARTED
            self._retreating = False
            self._retreat_ok = None
            self._abort_reason = None
            self._abort_cause = None
            self._trigger_reading = None
            self._trail = []
            self._exiting = False
            self._last_test = None
            self._alarms = []
            self._alarms_ignored = 0
            self._gas_extremes = {}
            self._last_monitor_t = -1.0e9
            self._waypoints_reached = 0
            self._outcome = None
            self._last_pose = getattr(self, "_last_pose", None)
            self._t_start = None
            self._tests = []
            self._purge_applied = False       # the last purge-gate check applied (vented / prior fail)
            self._auth_vent_required = False  # the authorizing test depended on ventilation
            self._vent_lost_handled = False
            self._door_check = None
            self.entry_permit = Permit(permit_id=self.permit_id)

    # ------------------------------------------------------------------ #
    # Small helpers
    # ------------------------------------------------------------------ #

    @property
    def permit(self):
        return self._permit

    def _now(self):
        return self._clock.now()

    def _pose(self):
        try:
            pose = self.robot.pose()
        except Exception as error:
            LOG.warning("pose() raised %s", error)
            pose = None
        if pose is None:
            return None
        pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        self._last_pose = pose
        return pose

    def _status(self):
        try:
            return filter_status(self.robot.status())
        except Exception as error:
            LOG.warning("status() raised %s", error)
            return None

    def backend(self):
        """Backend identity (robot_client.BackendIdentity snapshot): the
        robot link's own when it has ``backend()``, else built here from the
        payload/status dicts it returns. The first time it is in conflict
        (identified as the simulator, then contradicted) a
        ``backend_conflict`` event is recorded."""
        identity = None
        fn = getattr(self.robot, "backend", None)
        if callable(fn):
            try:
                identity = fn()
            except Exception as error:
                LOG.warning("backend() raised %s", error)
                identity = None
        if not isinstance(identity, dict):
            identity = self._backend.observe(self._status())
        if identity.get("backend") == BACKEND_CONFLICT and not self._conflict_recorded:
            with self._lock:
                first, self._conflict_recorded = not self._conflict_recorded, True
            if first:
                self._event("backend_conflict", "BACKEND IDENTITY CONFLICT: %s; %s. Operator "
                            "attention required." % (identity.get("reason"), CONFLICT_TEXT),
                            {"backend": identity, "robot_inside": self._inside_now()})
        return identity

    @property
    def simulator(self):
        """True only once the backend is identified as the simulator."""
        return self.backend().get("simulator") is True

    def backend_refusal(self, action, source="mission", retreat=False):
        """None on the simulator (and, with ``retreat``, in a backend
        identity conflict); otherwise logs a ``backend_refused`` event and
        returns why ``action`` is refused (fail closed, no override)."""
        identity = self.backend()
        refusal = backend_refusal(identity, action, retreat=retreat)
        if refusal:
            self._event("backend_refused", refusal, {"action": action, "backend": identity}, source)
        return refusal

    def drive_hold_reason(self):
        """Why motion must hold right now, or None: a second cmd_vel publisher
        (another driver on the bus) or a degraded simulator (payload/status
        rtf below RTF_MIN for RTF_DEGRADED_S). Re-read at most 4 times a s."""
        now = self._now()
        when, reason = self._hold_cache
        if now - when < 0.25:
            return reason
        status = self._status() or {}
        reason = None
        identity = self.backend()
        if identity.get("backend") == BACKEND_CONFLICT:
            reason = "%s, so the robot holds (the retreat still runs)" % identity.get("reason")
        pubs = status.get("cmd_vel_publishers")
        if isinstance(pubs, int) and pubs > 1 and reason is None:
            reason = ("%d publishers on cmd_vel: another driver is on the bus, so the robot holds"
                      % pubs)
        rtf = status.get("rtf")
        if isinstance(rtf, float) and rtf < self.RTF_MIN:
            if self._rtf_low_since is None:
                self._rtf_low_since = now
            elif now - self._rtf_low_since >= self.RTF_DEGRADED_S and reason is None:
                reason = ("the simulator runs at %.2f x real time (below %.1f for %.0f s): "
                          "degraded, so the robot holds and no entry starts"
                          % (rtf, self.RTF_MIN, now - self._rtf_low_since))
        else:
            self._rtf_low_since = None
        self._hold_cache = (now, reason)
        return reason

    def _call(self, name, retreat=False):
        """Call a RobotClient trigger method; never raises. Anything but a
        stop is refused unless the backend is the simulator, and a stop too
        until the backend has been identified as the simulator (T15).
        ``retreat`` (only from a running retreat: its first stand, its sit
        outside) lets a stand or sit through a backend identity conflict too
        (T9)."""
        retreat = bool(retreat) and name in RETREAT_METHODS and self._retreating
        refusal = backend_refusal(self.backend(), "calling %s" % name, retreat=retreat,
                                  stop=name in STOP_METHODS)
        if refusal:
            return False, refusal
        try:
            fn = getattr(self.robot, name)
            result = fn(owner="retreat") if retreat and _takes_keyword(fn, "owner") else fn()
            ok, message = result
            return bool(ok), trigger_text(name, bool(ok), message)
        except Exception as error:
            LOG.warning("%s raised %r", name, error)
            return False, "%s failed (exception in the robot link)" % name

    def in_space(self, x, y):
        """True if (x, y) is inside the permit space (tank + margin)."""
        return math.hypot(x - self.center_xy[0], y - self.center_xy[1]) < self.radius + self.SPACE_MARGIN_M

    def _inside_now(self, pose=None):
        """Spot inside the permit space, or in an entry, or its pose unknown
        (fail closed: treated as inside)."""
        with self._lock:
            if self._permit == ENTRY_IN_PROGRESS:
                return True
        pose = pose or self._pose() or self._last_pose
        return pose is None or self.in_space(pose[0], pose[1])

    def _confirm_physical(self, action, summary, details=None):
        """(ok, record) from ``confirm_action``; no console (None) or a
        failing hook is a refusal."""
        fn = self.confirm_action
        if not callable(fn):
            return False, {"action": action, "decision": "no console"}
        try:
            ok, record = fn(action, summary, details)
        except Exception as error:
            LOG.exception("confirm_action failed")
            return False, {"action": action, "decision": "error: %r" % (error,)}
        return bool(ok), record if isinstance(record, dict) else {}

    def _near_door(self, x, y):
        return math.hypot(x - self.door_xy[0], y - self.door_xy[1]) < self.NEAR_DOOR_M

    def _segment_enters_space(self, a, b):
        """True if the straight leg a -> b passes inside the permit space."""
        cx, cy = self.center_xy
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 < 1e-12 else max(0.0, min(1.0, ((cx - a[0]) * dx + (cy - a[1]) * dy) / seg2))
        return self.in_space(a[0] + t * dx, a[1] + t * dy)

    def _clear_of_space(self, x, y):
        """Outside the permit space and at least as far from the doorway as
        the entry point: nothing left to retreat from."""
        if self.in_space(x, y):
            return False
        entry_gap = math.hypot(self.entry_xy[0] - self.door_xy[0], self.entry_xy[1] - self.door_xy[1])
        return (math.hypot(x - self.door_xy[0], y - self.door_xy[1])
                >= entry_gap - self.WAYPOINT_TOL_M)

    def _ensure_guard(self):
        if not self.guard.running:
            self.guard.start()
        self._ensure_heartbeat()

    def robot_inside(self):
        """Spot in the permit space by its (last known) pose, or in an entry."""
        with self._lock:
            if self._permit == ENTRY_IN_PROGRESS:
                return True
        pose = self._pose() or self._last_pose
        return pose is not None and self.in_space(pose[0], pose[1])

    def guard_heartbeat(self):
        """The next payload/guard heartbeat (HUD contract): "stalled" when
        the guard was started but has not completed a gas evaluation within
        GUARD_STALL_S or its thread is gone, with ``checks`` and
        ``check_age_s`` as the evidence (T16)."""
        snap = self.guard.snapshot()
        with self._lock:
            retreating = self._retreating
            self._heartbeat_seq += 1
            seq = self._heartbeat_seq
        age = snap["check_age_s"]
        stalled = age is not None and (not snap["running"] or age > self.GUARD_STALL_S)
        monitoring = snap["running"] and (snap["armed"] or retreating)
        return {"state": "stalled" if stalled else "monitoring" if monitoring else "standby",
                "robot_inside": bool(self.robot_inside()), "seq": seq,
                "wall_t": round(time.time(), 3), "checks": snap["checks"],
                "check_age_s": None if age is None else round(age, 3)}

    def _ensure_heartbeat(self):
        publish = self._robot_fn("publish_guard")
        if publish is None or (self._heartbeat_thread is not None
                               and self._heartbeat_thread.is_alive()):
            return
        self._heartbeat_stop.clear()

        def beat():
            while not self._heartbeat_stop.is_set():
                try:
                    self.backend()        # records a backend identity conflict promptly
                    publish(self.guard_heartbeat())
                except Exception as error:   # never let telemetry stop the beat
                    LOG.warning("guard heartbeat failed: %s", error)
                self._heartbeat_stop.wait(self.HEARTBEAT_PERIOD_S)

        self._heartbeat_thread = threading.Thread(target=beat, name="guard_heartbeat",
                                                  daemon=True)
        self._heartbeat_thread.start()

    def start_guard(self):
        """Start the SafetyGuard now (idempotent). Callers that hand Spot to
        an operator or LLM call this up front: otherwise the guard would
        only start with the first mission motion, and a Spot already inside
        the tank (e.g. after an agent restart) would be unwatched."""
        self._ensure_guard()

    def _pose3(self):
        """(x, y, z, yaw) from robot.pose3(), or None (no method / no data)."""
        fn = getattr(self.robot, "pose3", None)
        if not callable(fn):
            return None
        try:
            pose = fn()
        except Exception as error:
            LOG.warning("pose3() raised %s", error)
            return None
        if pose is None:
            return None
        try:
            return tuple(float(v) for v in pose[:4])
        except (TypeError, ValueError, IndexError):
            return None

    def _approaches_entry(self, start, target):
        """True if driving start -> target goes within the approach zone
        (tank_radius + APPROACH_MARGIN_M of the tank center) - at the target,
        or anywhere along the leg closer than Spot already is. Leaving the
        zone is never an approach."""
        cx, cy = self.center_xy
        zone = self.radius + self.APPROACH_MARGIN_M
        if math.hypot(target[0] - cx, target[1] - cy) < zone:
            return True
        if start is None:
            return False
        dx, dy = target[0] - start[0], target[1] - start[1]
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            return False
        t = max(0.0, min(1.0, ((cx - start[0]) * dx + (cy - start[1]) * dy) / seg2))
        if t <= 1e-3:
            return False  # the leg only moves away from the tank
        closest = math.hypot(start[0] + t * dx - cx, start[1] + t * dy - cy)
        return closest < zone and closest < math.hypot(start[0] - cx, start[1] - cy) - 1e-3

    def permit_missing(self):
        """Permit fields still missing ([] when complete or not required)."""
        if not self.require_permit:
            return []
        with self._lock:
            return self.entry_permit.missing()

    def permit_ready(self):
        return not self.permit_missing()

    def approach_refusal(self, x, y):
        """Why walk_to(x, y) would be refused by the permit gate, or None."""
        missing = self.permit_missing()
        if not missing:
            return None
        start = self._pose() or self._last_pose
        if not self._approaches_entry(start, (float(x), float(y))):
            return None
        return ("the entry permit is incomplete (missing %s); Spot does not approach the "
                "entry until the rescue plan is recorded and the operator has attested isolation, "
                "LOTO and the attendant at the console" % describe_missing(missing))

    def _place_name(self, x, y):
        """Nearest named place (entry, door, waypoint_N, spawn) to (x, y)."""
        places = [("spawn", self.site["spawn_xy"]), ("entry", self.entry_xy),
                  ("door", self.door_xy)]
        places += [("waypoint_%d" % (i + 1), wp) for i, wp in enumerate(self.waypoints)]
        name, xy = min(places, key=lambda p: math.hypot(x - p[1][0], y - p[1][1]))
        return name, math.hypot(x - xy[0], y - xy[1])

    # ------------------------------------------------------------------ #
    # Audit log / events
    # ------------------------------------------------------------------ #

    def _event(self, kind, message, data=None, source="mission"):
        """Publish on payload/events and append to the JSONL audit log. The
        operator's login and SSM session id go to the JSONL only (T37)."""
        data = _jsonable(data) if data is not None else None
        public = without_operator_identity(data) if data is not None else None
        message = clean_text(message, None)
        # Publish and write under one lock so payload/events and the audit
        # file carry events in the same order, whichever thread emits them.
        with self._log_lock:
            published = None
            try:
                try:
                    published = self.robot.publish_event(kind, message, public, source=source)
                except TypeError:  # RobotClient-like without the source keyword
                    published = self.robot.publish_event(kind, message, public)
            except Exception as error:
                LOG.warning("publish_event(%s) failed: %s", kind, error)
            t = published.get("t") if isinstance(published, dict) else None
            wall_t = _num(published.get("wall_t")) if isinstance(published, dict) else None
            if wall_t is None or not 0.0 < wall_t < 1.0e11:
                # Missing, or a value fromtimestamp() would raise on: this
                # path also logs guard/retreat events, so it must not throw.
                wall_t = time.time()
            record = {
                "t": t if t is not None else wall_t,
                "wall_t": round(wall_t, 3),
                "wall": datetime.datetime.fromtimestamp(wall_t, datetime.timezone.utc).isoformat(
                    timespec="milliseconds"),
                "permit_id": self.permit_id,
                "permit": self._permit,
                "source": source,
                "kind": kind,
                "message": message,
                "data": data,
            }
            self._seq += 1
            record["seq"] = self._seq
            record["prev_sha256"] = self._prev_sha256
            self._records.append(record)
            newly_failed = False
            head = None
            if self._log_file is not None:
                line = json.dumps(record)
                try:
                    self._log_file.write(line + "\n")
                    self._log_file.flush()
                    self._prev_sha256 = _line_sha256(line)
                    self._chain_records += 1
                except (OSError, ValueError) as error:
                    newly_failed = not self._log_failed
                    self._log_failed = True
                    if not self._log_error_reported:
                        self._log_error_reported = True
                        LOG.error("audit log write failed (%s): %s", self.log_path, error)
                if kind in AUDIT_HEAD_AFTER:
                    head = self._head_line_locked(AUDIT_HEAD_AFTER[kind])
        LOG.info("[%s] %s: %s", source, kind, message)
        if head is not None:
            self._print_audit_head(head)
        if newly_failed:
            self._audit_failed()
        return record

    def _head_line_locked(self, after):
        """The console line for the current chain head (``_log_lock`` held)."""
        return audit_head_line(self.log_path, self._chain_records, self._prev_sha256, after,
                               failed=self._log_failed)

    def audit_head(self):
        """{"log", "records", "sha256"}: the chain head of the audit log (the
        file's record count and the SHA-256 of its last record; T38)."""
        with self._log_lock:
            return {"log": self.log_path, "records": self._chain_records,
                    "sha256": self._prev_sha256}

    def _print_audit_head(self, line):
        """Hand a chain head line to the console (``on_audit_head``); never
        raises (T38)."""
        fn = self.on_audit_head
        if callable(fn):
            try:
                fn(line)
            except Exception:
                LOG.exception("on_audit_head failed")

    def _audit_failed(self):
        """The audit log can no longer be written (fail closed, T38): an
        authorized entry is withdrawn, an entry in progress retreats; tests
        no longer authorize and no run starts (see the callers)."""
        with self._lock:
            permit = self._permit
        reason = "the audit log can no longer be written (%s)" % self.log_path
        if permit == ENTRY_AUTHORIZED:
            self._withdraw_authorization(reason)
        elif permit == ENTRY_IN_PROGRESS and self.auto_retreat:
            message = "AUDIT LOG LOST during the entry: " + reason
            if self._begin_retreat(message, None, "audit"):
                threading.Thread(target=self._retreat_worker, args=(message, None, "audit", "mission"),
                                 name="retreat", daemon=True).start()

    def log_event(self, kind, message, data=None, source="mission"):
        """Public: add a record to the audit log and payload/events (e.g. the
        agent's reasoning, or the demo harness disclosing an injection)."""
        return self._event(kind, message, data, source)

    def _set_permit(self, new, reason, data=None, source="mission"):
        with self._lock:
            old = self._permit
            if new == old:
                return True
            if new not in _TRANSITIONS[old]:
                LOG.warning("illegal permit transition %s -> %s ignored", old, new)
                return False
            self._permit = new
            fields = self.entry_permit.as_dict()
        payload = {"state": new, "previous": old, "permit_id": self.permit_id, "reason": reason,
                   "permit": fields}
        payload.update(data or {})
        self._event("permit", "Permit %s: %s -> %s (%s)" % (self.permit_id, old, new, reason),
                    payload, source)
        return True

    # ------------------------------------------------------------------ #
    # Motion primitives
    # ------------------------------------------------------------------ #

    def _send(self, vx, vy, wz, owner):
        """cmd_vel through both gates: mission-level (retreat/cancel) and the
        robot's motion owner. Zero twists go through once the backend has
        been identified as the simulator, never before (T15: on a real
        driver any cmd_vel claims the lease and powers the motors)."""
        moving = (vx, vy, wz) != (0.0, 0.0, 0.0)
        if moving and (self._cancel.is_set() or
                       (owner != "retreat" and (self._retreating or self._halt.is_set()))):
            return False
        if backend_refusal(self.backend(), "moving the robot" if moving else "stopping the robot",
                           retreat=owner == "retreat", stop=not moving):
            return False
        try:
            if self._gate:
                return bool(self.robot.send_velocity(vx, vy, wz, owner=owner))
            return self.robot.send_velocity(vx, vy, wz) is not False
        except Exception as error:
            LOG.warning("send_velocity failed: %s", error)
            return False

    def safe_stop(self):
        """Zero twist + stop service. Safe to call from any thread, any time."""
        self._send(0.0, 0.0, 0.0, "retreat" if self._retreating else "mission")
        return self._call("stop")

    def _tick(self, next_tick):
        """Sleep until the next control tick; returns the following deadline."""
        period = 1.0 / self.CONTROL_HZ
        next_tick += period
        delay = next_tick - self._now()
        if delay < -period:  # fell behind (GC, slow call): resync, no burst
            next_tick = self._now()
            delay = 0.0
        self._clock.sleep(max(0.0, delay))
        return next_tick

    def _interrupted(self, owner, epoch=None):
        if self._cancel.is_set():
            return "cancelled"
        if owner != "retreat" and (self._halt.is_set() or
                                   (epoch is not None and epoch != self._motion_epoch)):
            return "halted"
        return None

    def _preempted(self, epoch):
        """True if a retreat/cancel started since ``epoch`` was taken."""
        return self._halt.is_set() or self._cancel.is_set() or epoch != self._motion_epoch

    def _end_motion(self, result, owner):
        """Zero twist + stop, unless a running retreat owns cmd_vel (a stop
        from here would stall its escape)."""
        if result != "halted" or not self._retreating:
            self._send(0.0, 0.0, 0.0, owner)
            self._call("stop")

    def _compute_command(self, dist, err, turning, reverse, max_speed):
        """Heading controller step -> (vx, wz, turning)."""
        abs_err = abs(err)
        if turning:
            if abs_err < math.radians(self.TURN_RESUME_DEG):
                turning = False
        elif abs_err > math.radians(self.TURN_IN_PLACE_DEG):
            turning = True
        if turning:
            wz = math.copysign(min(self.MAX_WZ, max(self.MIN_TURN_WZ, self.K_TURN * abs_err)), err)
            return 0.0, wz, True
        cap = min(max_speed, self.REVERSE_MAX_SPEED) if reverse else max_speed
        speed = min(cap, max(self.MIN_V, self.K_V * dist)) * max(0.0, math.cos(err))
        wz = max(-self.MAX_WZ_DRIVE, min(self.MAX_WZ_DRIVE, self.K_YAW * err))
        return (-speed if reverse else speed), wz, False

    def _goto(self, x, y, tol, max_speed, timeout, owner, allow_reverse=None, epoch=None):
        """Closed-loop drive to (x, y). Returns 'reached' | 'timeout' |
        'pose_lost' | 'halted' | 'cancelled'. Always ends stopped unless a
        retreat has taken over (then the retreat owns cmd_vel). ``epoch``
        (default: now) - a retreat/cancel after it halts a non-retreat leg."""
        max_speed = max(0.05, min(float(max_speed), 1.0))
        epoch = self._motion_epoch if epoch is None else epoch
        start = self._now()
        deadline = start + float(timeout)
        last_pose_t = start
        reverse = None
        turning = False
        result = "timeout"
        next_tick = start
        try:
            while True:
                interrupted = self._interrupted(owner, epoch)
                if interrupted:
                    result = interrupted
                    break
                now = self._now()
                if now >= deadline:
                    result = "timeout"
                    break
                # A retreat is never held: stopping it would strand Spot inside.
                hold = self.drive_hold_reason() if owner != "retreat" else None
                pose = self._pose() if hold is None else None
                if pose is None:
                    self._send(0.0, 0.0, 0.0, owner)  # never drive blind
                    if now - last_pose_t > self.POSE_LOST_S:
                        result = "held" if hold else "pose_lost"
                        self._last_hold = hold
                        break
                else:
                    last_pose_t = now
                    gas_hold = (self._gas_hold_reason(pose, (x, y)) if owner != "retreat"
                                else None)
                    if gas_hold:
                        self._send(0.0, 0.0, 0.0, owner)   # no gas reading: hold (T17)
                        self._note_gas_hold(gas_hold, pose, owner)
                        next_tick = self._tick(next_tick)
                        continue
                    self._end_gas_hold(resumed=True)
                    px, py, yaw = pose
                    dist = math.hypot(x - px, y - py)
                    if dist <= tol:
                        result = "reached"
                        break
                    bearing = math.atan2(y - py, x - px)
                    if reverse is None:
                        # Direction is chosen once per leg so it cannot flip-flop.
                        behind = abs(wrap_angle(bearing - yaw)) > math.pi / 2.0
                        may_reverse = (self._near_door(px, py) if allow_reverse is None
                                       else bool(allow_reverse))
                        reverse = behind and may_reverse
                    heading = yaw + (math.pi if reverse else 0.0)
                    vx, wz, turning = self._compute_command(
                        dist, wrap_angle(bearing - heading), turning, reverse, max_speed)
                    self._send(vx, 0.0, wz, owner)
                next_tick = self._tick(next_tick)
        finally:
            self._end_gas_hold(resumed=False)
            self._end_motion(result, owner)
        return result

    def _turn_to(self, yaw, tol, timeout, owner):
        """Turn in place to ``yaw``; same result strings as ``_goto``."""
        epoch = self._motion_epoch
        start = self._now()
        deadline = start + float(timeout)
        last_pose_t = start
        result = "timeout"
        next_tick = start
        try:
            while True:
                interrupted = self._interrupted(owner, epoch)
                if interrupted:
                    result = interrupted
                    break
                now = self._now()
                if now >= deadline:
                    break
                hold = self.drive_hold_reason() if owner != "retreat" else None
                pose = self._pose() if hold is None else None
                if pose is None:
                    self._send(0.0, 0.0, 0.0, owner)
                    if now - last_pose_t > self.POSE_LOST_S:
                        result = "held" if hold else "pose_lost"
                        self._last_hold = hold
                        break
                else:
                    last_pose_t = now
                    gas_hold = self._gas_hold_reason(pose) if owner != "retreat" else None
                    if gas_hold:
                        self._send(0.0, 0.0, 0.0, owner)   # no gas reading: hold (T17)
                        self._note_gas_hold(gas_hold, pose, owner)
                        next_tick = self._tick(next_tick)
                        continue
                    self._end_gas_hold(resumed=True)
                    err = wrap_angle(yaw - pose[2])
                    if abs(err) <= tol:
                        result = "reached"
                        break
                    wz = math.copysign(min(self.MAX_WZ, max(self.MIN_TURN_WZ, self.K_TURN * abs(err))), err)
                    self._send(0.0, 0.0, wz, owner)
                next_tick = self._tick(next_tick)
        finally:
            self._end_gas_hold(resumed=False)
            self._end_motion(result, owner)
        return result

    def _gas_hold_reason(self, pose, target=None):
        """Why motion must hold for missing gas telemetry, or None (T17): no
        current gas reading (``gas()`` None: payload/gas stale beyond its
        freshness limit, or refused as it arrived) while Spot is in the
        permit space or the move approaches it. The retreat never asks: it
        is never held, and stopping never is."""
        inside = self.in_space(pose[0], pose[1])
        if not inside and (target is None or not self._moves_toward_space(pose, target)):
            return None
        if self._gas_now() is not None:
            return None
        return ("no current gas reading (payload/gas stale or refused) %s"
                % ("inside the permit space" if inside else "while approaching the permit space"))

    def _moves_toward_space(self, start, target):
        """True if the leg start -> target comes closer to the tank than Spot
        already is, within the approach zone (tank_radius +
        APPROACH_MARGIN_M). Moving away is never an approach."""
        cx, cy = self.center_xy
        dx, dy = target[0] - start[0], target[1] - start[1]
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            return False
        t = max(0.0, min(1.0, ((cx - start[0]) * dx + (cy - start[1]) * dy) / seg2))
        closest = math.hypot(start[0] + t * dx - cx, start[1] + t * dy - cy)
        return (closest < self.radius + self.APPROACH_MARGIN_M
                and closest < math.hypot(start[0] - cx, start[1] - cy) - 1e-3)

    def _note_gas_hold(self, reason, pose, owner):
        """Record a hold for missing gas once, when it begins (T17)."""
        with self._lock:
            first = self._gas_hold_since is None
            if first:
                self._gas_hold_since = self._now()
        if first:
            self._event("gas_hold", "Motion HELD: %s. Spot holds zero velocity until a fresh gas "
                        "sample arrives from the simulator (stops and the guard's retreat are "
                        "never held)." % reason,
                        {"reason": reason, "pose": [round(v, 3) for v in pose], "owner": owner})

    def _end_gas_hold(self, resumed):
        """End a hold for missing gas: recorded when a fresh sample lets the
        motion resume; a motion that ends while held just clears it."""
        with self._lock:
            since, self._gas_hold_since = self._gas_hold_since, None
        if since is not None and resumed:
            held = self._now() - since
            self._event("gas_hold_released", "Fresh gas sample: motion resumes after %.1f s held"
                        % held, {"held_s": round(held, 2)})

    def _leg_timeout(self, target, speed):
        pose = self._last_pose
        dist = math.hypot(target[0] - pose[0], target[1] - pose[1]) if pose else 10.0
        return max(30.0, 4.0 * dist / max(speed, 0.1) + 15.0)

    def _navigate(self, x, y, tol, max_speed, timeout, owner, allow_reverse=None,
                  label="walk", source="mission", epoch=None):
        status = self._goto(x, y, tol, max_speed, timeout, owner, allow_reverse, epoch)
        if status in ("timeout", "pose_lost", "held"):
            kind = {"timeout": "walk_timeout", "pose_lost": "pose_lost", "held": "walk_held"}[status]
            text = ("timed out after %.0f s" % timeout if status == "timeout"
                    else "lost odometry for > %.1f s" % self.POSE_LOST_S if status == "pose_lost"
                    else "held: %s" % getattr(self, "_last_hold", None))
            self._event(kind, "%s to (%.2f, %.2f) %s; robot stopped" % (label, x, y, text),
                        {"target": [x, y], "pose": self._last_pose, "timeout_s": timeout,
                         "label": label}, source)
        return status

    # ------------------------------------------------------------------ #
    # Public step-by-step API
    # ------------------------------------------------------------------ #

    def prepare_permit(self, permit_id=None, isolation_confirmed=None, loto_confirmed=None,
                       attendant=None, rescue_plan=None, source="mission", prepared_by=None,
                       entry_supervisor=None, attestation=None):
        """Record entry-permit fields; ``None`` keeps a field's current value.

        Allowed only before entry is authorized (NOT_STARTED or
        PRE_ENTRY_TESTING) and not during a retreat; a closed permit needs
        ``new_permit()`` first. The booleans must be real booleans (True =
        confirmed), and True is accepted only together with ``attestation``,
        the operator's console record whose ``decision`` is "confirmed"
        (agent/operator_console.py): a tool argument alone never confirms
        isolation or LOTO. Returns {"ok" (recorded without errors),
        "complete", "permit" (Permit dict), "missing", "errors",
        "permit_state"}.
        """
        errors = []
        with self._lock:
            state = self._permit
            if state not in (NOT_STARTED, PRE_ENTRY_TESTING) or self._retreating:
                why = ("a retreat is in progress" if self._retreating else
                       "permit %s is %s; %s" % (self.permit_id, state,
                                                "start a new permit first" if state in TERMINAL_STATES
                                                else "fields cannot change once entry is authorized"))
                return {"ok": False, "complete": self.entry_permit.complete, "error": why,
                        "permit": self.entry_permit.as_dict(),
                        "missing": self.entry_permit.missing(), "errors": [why],
                        "permit_state": state}
            current = self.entry_permit
            new_id = current.permit_id
            if permit_id is not None:
                text = " ".join(str(permit_id).split())
                if not text or len(text) > PERMIT_ID_MAX or not text.isprintable():
                    errors.append("permit_id must be 1-%d printable characters" % PERMIT_ID_MAX)
                else:
                    new_id = text

            attested = isinstance(attestation, dict) and attestation.get("decision") == CONFIRMED

            def flag(value, name, keep):
                if value is None:
                    return keep
                if isinstance(value, bool):
                    if value and not attested:
                        errors.append("%s is recorded only from the operator's attestation at the "
                                      "console, not from a tool argument" % name)
                        return keep
                    return value
                errors.append("%s must be true or false, got %r" % (name, value))
                return False

            def text_field(value, name, keep):
                if value is None:
                    return keep
                text = " ".join(clean_text(value, None).split())
                if len(text) > PERMIT_TEXT_MAX:
                    errors.append("%s is longer than %d characters" % (name, PERMIT_TEXT_MAX))
                    return keep
                return text

            def role_field(value, name, role, keep):
                if value is None:
                    return keep
                label = role_label(value, role)
                if label is None:
                    # Roles, never personal names, in the permit record (T37).
                    errors.append("%s must be a role label such as %r or %r (a roster id), not a "
                                  "name" % (name, role, role + " 2"))
                    return keep
                return label

            updated = Permit(
                permit_id=new_id,
                isolation_confirmed=flag(isolation_confirmed, "isolation_confirmed",
                                         current.isolation_confirmed),
                loto_confirmed=flag(loto_confirmed, "loto_confirmed", current.loto_confirmed),
                attendant=role_field(attendant, "attendant", ATTENDANT_ROLE, current.attendant),
                rescue_plan=text_field(rescue_plan, "rescue_plan", current.rescue_plan),
                issued_at=current.issued_at,
                entry_supervisor=role_field(entry_supervisor, "entry_supervisor",
                                            ENTRY_SUPERVISOR_ROLE, current.entry_supervisor),
                attestation=(_jsonable(attestation) if attested else current.attestation))
            if updated.complete:
                updated.issued_at = current.issued_at if current.complete else _utc_now()
            else:
                updated.issued_at = None
            self.entry_permit = updated
            self.permit_id = new_id
            fields = updated.as_dict()
        missing = fields["missing"]
        if missing:
            message = ("Permit %s fields recorded; INCOMPLETE - missing %s. Spot will not "
                       "approach the entry." % (new_id, describe_missing(missing)))
        else:
            message = ("Permit %s complete: isolation and LOTO attested by the operator at the "
                       "console as %s, %s stationed, rescue plan: %s."
                       % (new_id, fields["entry_supervisor"], fields["attendant"],
                          fields["rescue_plan"]))
        if errors:
            message += " Rejected: " + "; ".join(errors) + "."
        self._event("permit_prepared", message,
                    {"permit": fields, "permit_id": new_id, "permit_state": state,
                     "complete": not missing, "missing": missing, "errors": errors,
                     "prepared_by": prepared_by or source}, source)
        return {"ok": not errors, "complete": not missing, "permit": fields, "missing": missing,
                "errors": errors, "permit_state": state}

    def prepare(self):
        """claim -> power_on -> stand_up() (stand, wait, body-height check
        with bounded self-right). -> bool"""
        self._ensure_guard()
        if self.backend_refusal("preparing the robot (claim, power on, stand)"):
            return False
        status = self._status()
        if status and status.get("estopped"):
            self._event("robot", "E-stop is asserted; release it before the mission", status)
            return False
        for name in ("claim", "power_on"):
            ok, message = self._call(name)
            self._event("robot", "%s: %s - %s" % (name, "ok" if ok else "FAILED", message),
                        {"service": name, "ok": ok, "message": message})
            if not ok:
                return False
        return self.stand_up()["ok"]

    def stand_up(self):
        """stand -> wait until standing -> check the body height (odometry z).

        If Spot is down after the stand (z < UPRIGHT_MIN_Z_M; host F1: a
        re-stand after a sit fell backward within ~2 s), call self_right and
        stand again, at most SELF_RIGHT_ATTEMPTS times, logging each step.
        Services only - never cmd_vel. Returns {"ok", "self_right_attempts",
        "body_z_m", "error"?}.
        """
        self._ensure_guard()
        out = {"ok": False, "self_right_attempts": 0, "body_z_m": None}
        if self._retreating:
            out["error"] = "a retreat is in progress"
            return out
        refusal = self.backend_refusal("standing the robot up")
        if refusal:
            out["error"] = refusal
            return out
        while True:
            if self._cancel.is_set():
                out["error"] = "cancelled"
                return out
            ok, message = self._call("stand")
            self._event("robot", "stand: %s - %s" % ("ok" if ok else "FAILED", message),
                        {"service": "stand", "ok": ok, "message": message})
            if not ok:
                out["error"] = message
                return out
            if not self._wait_standing():
                out["error"] = "stand did not complete"
                return out
            z, has_height = self._settled_body_z()
            out["body_z_m"] = None if z is None else round(z, 3)
            if z is None:
                if has_height:
                    self._event("robot", "Body-height check skipped: no odometry height")
                out["ok"] = True
                return out
            if z >= self.UPRIGHT_MIN_Z_M:
                if out["self_right_attempts"]:
                    self._event("robot", "Upright after self-right: body height %.2f m" % z,
                                {"body_z_m": z, "self_right_attempts": out["self_right_attempts"]})
                out["ok"] = True
                return out
            attempt = out["self_right_attempts"] + 1
            if attempt > self.SELF_RIGHT_ATTEMPTS:
                self._event("robot_fallen", "Spot is still down (body height %.2f m) after %d "
                            "self-right attempts; not moving - operator attention required"
                            % (z, self.SELF_RIGHT_ATTEMPTS),
                            {"body_z_m": z, "self_right_attempts": self.SELF_RIGHT_ATTEMPTS})
                out["error"] = "robot fallen (body height %.2f m)" % z
                return out
            out["self_right_attempts"] = attempt
            before = self._pose()
            if self._inside_now(before):
                confirmed, record = self._confirm_physical(
                    "self_right_inside",
                    "SELF-RIGHT INSIDE THE PERMIT SPACE: Spot is down (body height %.2f m) inside "
                    "the tank. Roll it over in the confined space (attempt %d/%d)?"
                    % (z, attempt, self.SELF_RIGHT_ATTEMPTS),
                    {"body_z_m": z, "attempt": attempt, "pose": before})
                if not confirmed:
                    self._event("self_right", "self_right inside the permit space NOT done: the "
                                "operator did not confirm it (%s); Spot stays down - operator "
                                "attention required" % record.get("decision", "no console"),
                                {"body_z_m": z, "attempt": attempt, "ok": False,
                                 "confirmation": record})
                    out["error"] = "self_right inside the permit space was not confirmed"
                    return out
            ok, message = self._call("self_right")
            self._event("self_right", "Body height %.2f m after stand (< %.2f m): Spot appears to "
                        "have fallen. self_right attempt %d/%d: %s - %s; then stand again"
                        % (z, self.UPRIGHT_MIN_Z_M, attempt, self.SELF_RIGHT_ATTEMPTS,
                           "ok" if ok else "FAILED", message),
                        {"body_z_m": z, "attempt": attempt, "ok": ok, "message": message})
            if not ok:
                out["error"] = "self_right failed: %s" % message
                return out
            self._clock.sleep(self.SELF_RIGHT_WAIT_S)
            after = self._pose()
            if before is not None and after is not None:
                moved = math.hypot(after[0] - before[0], after[1] - before[1])
                if moved > self.SELF_RIGHT_RELOCATE_M:
                    # The facade's self_right restarts the policy from its
                    # default state, which puts Spot back at spawn: say so
                    # instead of passing it off as an in-place recovery.
                    out["relocated_m"] = round(moved, 2)
                    out["relocated_to"] = [round(after[0], 2), round(after[1], 2)]
                    # kind self_right, not sim_control: the HUD treats
                    # sim_control as "injected gas cleared".
                    self._event("self_right", "SIMULATION: self_right re-initialized Spot at "
                                "(%.1f, %.1f), %.1f m from where it fell (the simulator restarts "
                                "the walking policy from its default pose; a real Spot "
                                "self-rights in place)" % (after[0], after[1], moved),
                                {"from": list(before[:2]), "to": list(after[:2]),
                                 "moved_m": round(moved, 2), "simulated": True})

    def _settled_body_z(self):
        """(z, has_height): body height after STAND_SETTLE_S (the host fall
        happened within ~2 s of the stand). z None = unknown."""
        if not callable(getattr(self.robot, "pose3", None)):
            return None, False
        end = self._now() + self.STAND_SETTLE_S
        z = None
        while not self._cancel.is_set():
            pose = self._pose3()
            if pose is not None:
                z = pose[2]
            if self._now() >= end:
                break
            self._clock.sleep(0.1)
        return z, True

    def _wait_standing(self):
        start = self._now()
        saw_status = False
        while self._now() - start < self.STAND_TIMEOUT_S:
            if self._cancel.is_set():
                return False
            status = self._status()
            if status is not None:
                saw_status = True
                if status.get("estopped"):
                    self._event("robot", "E-stop asserted while standing up", status)
                    return False
                if status.get("standing") and not status.get("standing_up"):
                    return True
            elif not saw_status and self._now() - start > 3.0:
                # No payload/status (e.g. the real driver): give the stand
                # command the ~2 s it takes on hardware and carry on.
                self._clock.sleep(2.0)
                return True
            self._clock.sleep(0.1)
        self._event("robot", "stand did not complete within %.0f s" % self.STAND_TIMEOUT_S,
                    self._status())
        return False

    def walk_to(self, x, y, tol=0.25, max_speed=0.5, timeout=60):
        """Drive to (x, y) on odometry; False on timeout/refusal/halt.

        Refuses to enter the permit space - a target inside it, or a straight
        leg from outside that passes through it (e.g. a point beyond the
        tank) - unless entry is authorized, and refuses while a retreat owns
        the robot. With an incomplete permit (``require_permit``) it also
        refuses any approach to the entry: a target within tank_radius +
        APPROACH_MARGIN_M, or a leg that dips into that zone
        (``permit_incomplete`` event). Retreats and exits do not use this
        gate, so an incomplete permit can never trap Spot inside.
        """
        self._ensure_guard()
        x, y = float(x), float(y)
        if self._cancel.is_set():
            return False
        if self.backend_refusal("moving the robot"):
            return False
        if self._retreating:
            self._event("walk_refused", "Retreat in progress; walk_to(%.2f, %.2f) refused" % (x, y))
            return False
        target_inside = self.in_space(x, y)
        start = self._pose() or self._last_pose
        missing = self.permit_missing()
        if missing and self._approaches_entry(start, (x, y)):
            with self._lock:
                fields = self.entry_permit.as_dict()
            self._event("permit_incomplete",
                        "Not approaching the entry: permit %s is incomplete (missing %s). "
                        "Record isolation, LOTO, attendant and rescue plan first."
                        % (self.permit_id, describe_missing(missing)),
                        {"target": [x, y], "missing": missing, "permit": fields})
            return False
        crosses = (start is not None and not self.in_space(start[0], start[1]) and
                   self._segment_enters_space(start, (x, y)))
        if target_inside or crosses:
            with self._lock:
                permit = self._permit
            hold = self.drive_hold_reason() if permit == ENTRY_AUTHORIZED else None
            if hold:
                self._event("entry_blocked", "Not crossing into the permit space: %s" % hold,
                            {"target": [x, y], "permit": permit, "hold": hold})
                return False
            if permit == ENTRY_AUTHORIZED and not self._door_check_fresh() \
                    and not self.door_check():
                self._event("entry_blocked", "Refusing to cross into the permit space: the check "
                            "at the door did not pass (permit %s)" % self._permit,
                            {"target": [x, y], "permit": self._permit})
                return False
            if permit == ENTRY_AUTHORIZED and self._set_permit(
                    ENTRY_IN_PROGRESS, "entrant crossing into the permit space"):
                pass
            elif permit != ENTRY_IN_PROGRESS:
                self._event("entry_blocked",
                            "Refusing to drive into the permit space: permit is %s "
                            "(a passing atmospheric test must authorize entry)" % permit,
                            {"target": [x, y], "permit": permit})
                return False
        elif (start is not None and self.in_space(start[0], start[1]) and
              math.hypot(start[0] - self.door_xy[0], start[1] - self.door_xy[1])
              > self.UNPLANNED_EXIT_M):
            # A straight line from deep inside to a point outside was never
            # planned (it crosses the pipe runs or the shell).
            self._event("walk_refused", "Not driving straight out of the permit space from "
                        "(%.2f, %.2f); use exit_space() or retreat() to leave along the route"
                        % (start[0], start[1]), {"target": [x, y], "pose": list(start)})
            return False
        if not self._motion_lock.acquire(blocking=False):
            self._event("walk_refused", "Another motion command is in progress")
            return False
        try:
            status = self._navigate(x, y, tol, max_speed, timeout, owner="mission")
        finally:
            self._motion_lock.release()
        if status == "reached" and target_inside:
            with self._lock:
                if self._permit == ENTRY_IN_PROGRESS and not self._exiting:
                    self._trail.append((x, y))
        return status == "reached"

    def face(self, yaw, tol_deg=5.0, timeout=20.0):
        """Turn in place to ``yaw`` (rad). Refused within NEAR_DOOR_M of the
        doorway, where a turning Spot would sweep into the jambs."""
        self._ensure_guard()
        if self._cancel.is_set() or self._retreating:
            return False
        if self.backend_refusal("turning the robot"):
            return False
        pose = self._pose()
        if pose is None:
            return False
        tol = math.radians(tol_deg)
        err = abs(wrap_angle(yaw - pose[2]))
        if err <= tol:
            return True
        # Small corrections sweep Spot's corners only a few cm; a big turn
        # this close to the doorway would swing them into the jambs.
        if self._near_door(pose[0], pose[1]) and err > math.radians(self.TURN_IN_PLACE_DEG):
            self._event("turn_refused", "Not turning %.0f deg in place within %.1f m of the "
                        "doorway" % (math.degrees(err), self.NEAR_DOOR_M), {"pose": pose, "yaw": yaw})
            return False
        if not self._motion_lock.acquire(blocking=False):
            return False
        try:
            status = self._turn_to(float(yaw), tol, timeout, owner="mission")
        finally:
            self._motion_lock.release()
        return status == "reached"

    def face_door(self, tol_deg=15.0):
        """Turn to look at the doorway from wherever Spot stands. The loose
        default tolerance is deliberate: the next leg's heading controller
        trims the residual error while walking, without a turn in place."""
        pose = self._pose()
        if pose is None:
            return False
        return self.face(math.atan2(self.door_xy[1] - pose[1], self.door_xy[0] - pose[0]),
                         tol_deg=tol_deg)

    def atmospheric_test(self, at_xy=None, dwell_s=4.0, *, withdraw_on_fail=True):
        """Pre-entry (or spot-check) atmospheric test in OSHA order.

        Walks to ``at_xy`` first if given, then dwells ``dwell_s`` split over
        three phases - O2, then LEL, then H2S + CO. Near the door mouth the
        extendable gas probe (simulated; arm-mounted on the real robot)
        samples SIX points: PROBE_DEPTHS_M (0.5 and 1.0 m) inside the
        opening along the door normal, each at PROBE_HEIGHTS_M; elsewhere it
        samples the three heights at Spot's own position. Each gas is judged
        on its worst reading across points and samples and logged as its own
        ``atmos_test`` event; ``atmos_probe`` carries the per-point rows.

        If the facade REJECTS a door-probe request (out of reach, occluded by
        the tank wall, no pose), Spot re-seats at door_mouth_xy facing the
        door and retries once (``probe_rejected``, then ``probe_retry``); a
        second rejection fails the test. A rejection or timeout falls back to
        the body sensor and a ``probe_unavailable`` event says so; with
        ``require_probe`` only probe samples inside the opening can
        authorize (or confirm) entry (``sampled_inside`` in the result).

        Purge gate: once this permit has had a failed atmosphere, or while
        the simulator reports the tank under (or after) forced ventilation,
        a passing test authorizes entry only when ventilation_status() shows
        fraction_remaining <= purge_target. Otherwise it is logged as
        ``purge_incomplete`` ("Purge incomplete - wait ...", with the time
        still needed) and the permit stays PRE_ENTRY_TESTING. Why: the door
        probe only reads the doorway; in the facade's field it passes from
        ~19 % remaining while floor-level H2S deeper inside still alarms.

        Pre-entry (NOT_STARTED / PRE_ENTRY_TESTING): needs a complete permit
        (when required); a full pass moves the permit to ENTRY_AUTHORIZED; a
        failure is logged as ``entry_denied`` and leaves the permit in
        PRE_ENTRY_TESTING, so the test can be repeated (e.g. after
        ventilation). A failing re-test while ENTRY_AUTHORIZED (not yet
        entered) withdraws the authorization: the permit is ABORTED and a new
        one is needed (unless ``withdraw_on_fail`` is False: the door check
        decides itself). Returns {"passed", "results", "failures", "at_xy",
        "method", "probe", "purge" (when the purge gate refused), ...}.
        """
        self._ensure_guard()
        dwell_s = max(0.75, float(dwell_s))
        epoch = self._motion_epoch
        with self._lock:
            permit = self._permit
        pre_entry = permit in (NOT_STARTED, PRE_ENTRY_TESTING)
        result = {"passed": False, "at_xy": None, "dwell_s": dwell_s,
                  "order": list(OSHA_TEST_ORDER), "results": [], "failures": [],
                  "pre_entry": pre_entry, "method": None, "probe": None}
        refusal = self.backend_refusal("running the atmospheric test")
        if refusal:
            result["failures"].append(refusal)
            return result
        missing = self.permit_missing() if pre_entry else []
        if missing:
            text = "permit %s incomplete (missing %s)" % (self.permit_id, describe_missing(missing))
            result["failures"].append(text)
            self._event("permit_incomplete", "Atmospheric test not started: %s. Record the "
                        "permit fields first." % text,
                        {"missing": missing, "permit": self.entry_permit.as_dict()})
            return result
        if permit == NOT_STARTED:
            self._set_permit(PRE_ENTRY_TESTING, "pre-entry atmospheric testing")
        if at_xy is not None:
            ax, ay = float(at_xy[0]), float(at_xy[1])
            pose = self._pose()
            if math.hypot(ax - self.door_mouth_xy[0], ay - self.door_mouth_xy[1]) < 1e-3:
                if not self._seat_at_door_mouth():
                    result["failures"].append("could not reach the test point")
            elif pose is None or math.hypot(ax - pose[0], ay - pose[1]) > 0.3:
                if not self.walk_to(ax, ay):
                    result["failures"].append("could not reach the test point")
        pose = self._pose() or self._last_pose
        where = [pose[0], pose[1]] if pose else (list(at_xy) if at_xy is not None else None)
        result["at_xy"] = where
        if not result["failures"]:
            points, probe_info = self._probe_plan(pose)
            result["probe"] = probe_info
            heights = "/".join("%g" % z for z in self.PROBE_HEIGHTS_M)
            how = ("extendable gas probe (simulated) %s m inside the opening at %s m (%d points)"
                   % (" and ".join("%g" % d for d in self.PROBE_DEPTHS_M), heights, len(points))
                   if probe_info and probe_info["at"] == "door" else
                   "extendable gas probe (simulated) at Spot's position at %s m" % heights
                   if probe_info else "body gas sensor")
            self._event("atmos_test_start",
                        "Atmospheric test at (%.2f, %.2f): O2 -> LEL -> H2S/CO "
                        "(OSHA 1910.146 order), %.1f s dwell, %s" % (where[0], where[1], dwell_s, how)
                        if where else "Atmospheric test: O2 -> LEL -> H2S/CO, " + how,
                        {"at_xy": where, "order": list(OSHA_TEST_ORDER), "dwell_s": dwell_s,
                         "probe": probe_info})
            probe_state = {"points": points, "reason": None,
                           "at": probe_info["at"] if probe_info else None,
                           "retried": False, "gave_up": False}
            phase_samples = {}
            for phase in (("o2",), ("lel",), ("h2s", "co")):
                samples = self._collect_phase(dwell_s / 3.0, epoch, probe_state)
                if samples is None:
                    result["failures"].append("test interrupted")
                    break
                for gas in phase:
                    phase_samples[gas] = samples
                    entry = self._judge(gas, samples)
                    result["results"].append(entry)
                    self._event("atmos_test", entry["message"], entry)
                    if not entry["passed"]:
                        result["failures"].append(entry["message"])
            methods = {e["method"] for e in result["results"]}
            result["method"] = (methods.pop() if len(methods) == 1 else
                                "probe+body" if methods else None)
            if probe_state["reason"]:
                result["probe_unavailable"] = probe_state["reason"]
            if probe_info:
                probe_info["rows"] = self._probe_rows(phase_samples)
                probe_info["repositioned"] = probe_state["retried"]
            # Only the probe inside the opening samples the space. From the
            # door mouth the body sensor is outside the tank and reads clean
            # air whatever is inside (host F2), so such a test can deny entry
            # but never authorize or confirm it.
            inside = (probe_info is not None and probe_info["at"] == "door"
                      and all(e["method"] == "probe" for e in result["results"]))
            result["sampled_inside"] = inside
            if probe_state["gave_up"] and result["results"]:
                result["failures"].append(
                    "the gas probe was rejected by the simulator even after repositioning at the "
                    "door mouth (%s); a test without the probe inside the opening cannot "
                    "authorize entry" % probe_state["reason"])
            elif (self.require_probe and not inside and result["results"]
                    and permit in (NOT_STARTED, PRE_ENTRY_TESTING, ENTRY_AUTHORIZED)):
                why = ("gas probe unavailable: %s" % probe_state["reason"] if probe_state["reason"]
                       else "gas probe unavailable" if probe_info is None
                       else "Spot is not at the door mouth")
                result["failures"].append(
                    "the gas probe did not sample inside the opening (%s); a reading taken "
                    "outside the tank cannot authorize entry" % why)
        result["passed"] = len(result["results"]) == len(OSHA_TEST_ORDER) and not result["failures"]
        with self._lock:
            permit = self._permit
        if pre_entry and result["passed"]:
            missing = self.permit_missing()
            purge = None if missing else self._purge_refusal()
            if missing:
                result["passed"] = False
                result["failures"].append("permit incomplete (missing %s)" % describe_missing(missing))
            elif self._log_failed:
                result["passed"] = False
                result["failures"].append("the audit log can no longer be written, so entry "
                                          "cannot be authorized")
            elif purge is not None:
                result["passed"] = False
                result["purge_incomplete"] = True
                result["purge"] = purge
                result["failures"].append(purge["message"])
                self._event("purge_incomplete", purge["message"], purge)
                # The structured hold the HUD's PURGE HOLD panel and the
                # agent's tool result both quote (one set of numbers).
                hold = self._purge_hold(purge)
                result["purge_hold"] = hold
                self._event("purge_hold", self._purge_hold_message(hold), hold)
            elif not self._authorize(result):
                # The permit moved on under us (cancel/retreat): no authorization.
                result["passed"] = False
                result["failures"].append("permit is %s; entry cannot be authorized" % self._permit)
        verdict = ("PASS - all four gases within OSHA 1910.146 App. E limits" if result["passed"]
                   else "FAIL - " + "; ".join(result["failures"]))
        if result["probe"] and result["probe"].get("rows"):
            probe = result["probe"]
            self._event("atmos_probe", "Probe readings by point (worst per gas): " + "; ".join(
                "%s%.1f m: %s" % ("%.1f m in @ " % row["inside_m"] if row.get("inside_m") else "",
                                  row["z"], format_reading(row)) for row in probe["rows"]),
                {"samples": probe["rows"], "passed": result["passed"],
                 "xy": probe["xy"], "inside_m": probe.get("inside_m"),
                 "depths_m": probe.get("depths_m"), "points_xy": probe.get("points_xy"),
                 "at": probe["at"], "stand_in": self.PROBE_LABEL, "simulated": True})
        self._event("atmos_result", "Atmospheric test " + verdict, result)
        with self._lock:
            self._last_test = result
            self._tests.append({"passed": result["passed"], "method": result["method"],
                                "failures": list(result["failures"]),
                                "atmos_failed": any(not e["passed"] for e in result["results"]),
                                "purge_incomplete": bool(result.get("purge_incomplete")),
                                "worst": {e["gas"]: e["value"] for e in result["results"]}})
        if not result["passed"] and (pre_entry or (permit == ENTRY_AUTHORIZED and withdraw_on_fail)):
            note = (("; permit stays PRE_ENTRY_TESTING - wait for the purge and re-test"
                     if result.get("purge_incomplete") else
                     "; permit stays PRE_ENTRY_TESTING - ventilate and re-test")
                    if pre_entry and permit == PRE_ENTRY_TESTING else "")
            self._event("entry_denied", "Entry NOT authorized: " + "; ".join(result["failures"]) + note,
                        result)
            if permit == ENTRY_AUTHORIZED and withdraw_on_fail:
                # A failed re-test before entry withdraws the earlier
                # authorization; the permit cannot go back to testing, so it
                # is cancelled and a new permit is needed.
                reason = ("entry authorization withdrawn - atmospheric re-test failed: "
                          + "; ".join(result["failures"]))
                with self._lock:
                    if self._abort_cause is None:
                        self._abort_cause, self._abort_reason = "entry_denied", reason
                        self._trigger_reading = {"atmospheric_test": result["results"]}
                self._set_permit(ABORTED, reason, {"results": result["results"]})
        return result

    def _purge_refusal(self):
        """Why a passing test must not authorize entry yet (purge not done),
        as {"message", "ventilation", "wait_s", ...}, or None.

        Applies once this permit has had a failed atmosphere, or when the
        simulator reports forced ventilation on (or a fraction below 1.0,
        i.e. the tank was ventilated): then the simulator's CURRENT
        fraction_remaining must be <= purge_target. An untouched,
        never-failed tank is judged by the probe alone.

        With no current simulator feed (payload/gas stale, or without the
        simulator's ventilation block, at this instant) such a tank is not
        authorized at all (fail closed, T12): neither the estimate from the
        commanded ventilation time nor the last ventilation block the
        mission saw is a measurement of the tank now. Both are still
        reported, labelled for what they are. The last block also decides
        whether the tank was ventilated, so a tank the presenter or host
        vented does not read "untouched" just because gas telemetry dropped
        out during the test."""
        with self._lock:
            prior_fail = any(t.get("atmos_failed") for t in self._tests)
            seen = self._vent_seen
        status = self.ventilation_status()
        frac = status["fraction_remaining"]
        seen_on, seen_frac, seen_age = False, None, None
        current = status["source"] == "sim"
        if current:
            vented = bool(status["on"]) or (frac is not None and frac < 1.0 - 1e-6)
            purged = status["purged"]
        else:
            vented = bool(status["on"])
            purged = False      # nothing measures the tank now: no authorization (T12)
            if seen is not None:
                block, t_seen = seen
                seen_on = bool(block.get("on"))
                seen_frac = _num(block.get("fraction_remaining"))
                seen_age = round(max(0.0, self._now() - t_seen), 1)
                if seen_on or (seen_frac is not None and seen_frac < 1.0 - 1e-6):
                    vented = True
        with self._lock:
            self._purge_applied = bool(prior_fail or vented)
        if not (prior_fail or vented) or purged:
            return None
        on_now = bool(status["on"]) or seen_on
        estimate = status.get("fraction_estimate")
        # The percentage is rounded ONCE, here: the message, the purge_hold
        # event, the HUD and the agent all quote this one value.
        judged = next((f for f in (frac, seen_frac, _num(estimate)) if f is not None), None)
        percent = round(100.0 * judged, 1) if judged is not None else None
        if frac is not None:
            level = "%.1f %% of the unventilated contamination remains" % percent
        elif seen_frac is not None:
            level = ("%.1f %% of the unventilated contamination remained at the simulator's last "
                     "ventilation report, %.0f s ago (no current simulator feed)"
                     % (percent, seen_age))
        elif estimate is not None:
            level = ("about %.0f %% of the unventilated contamination remains by an ESTIMATE from "
                     "the commanded ventilation time (not a measurement: no simulator "
                     "ventilation feed)" % (100.0 * estimate))
        else:
            level = "no ventilation progress is available"
        if not status["target_reachable"]:
            advice = ("the purge target is below the simulator's ventilation residual, so it "
                      "cannot be reached; entry stays denied")
        elif not current:
            advice = ("there is no current simulator ventilation feed, and neither an estimate nor "
                      "an earlier report authorizes entry: entry stays denied until payload/gas "
                      "carries the simulator's ventilation block again and it shows the target "
                      "reached")
        elif not on_now:
            advice = ("forced ventilation is OFF: start it (start_ventilation), wait for the "
                      "purge (ventilation_status), then re-test")
        elif status["remaining_s"] is not None:
            advice = "keep ventilating about %.0f s more, then re-test" % status["remaining_s"]
        else:
            advice = "keep ventilating, then re-test"
        why = ("an earlier test on this permit failed" if prior_fail else
               "the tank is being ventilated" if on_now else "the tank was ventilated")
        message = ("Purge incomplete — wait: the gases read within limits at the door, but %s "
                   "and %s (entry needs <= %.0f %%); %s"
                   % (why, level, 100.0 * self.purge_target, advice))
        return {"message": message, "prior_failure": prior_fail,
                "purge_target_fraction": self.purge_target, "fraction_remaining": frac,
                "percent_remaining": percent,
                "fraction_estimate": estimate, "wait_s": status["remaining_s"],
                "last_seen_fraction": seen_frac, "last_seen_age_s": seen_age,
                "current_feed": current, "ventilation": status, "simulated": True}

    def _authorize(self, result):
        """ENTRY_AUTHORIZED after a passing pre-entry test; remembers whether
        the authorization depends on forced ventilation (T19)."""
        with self._lock:
            self._auth_vent_required = self._purge_applied
            self._vent_lost_handled = False
            self._door_check = None
        return self._set_permit(ENTRY_AUTHORIZED, "pre-entry atmospheric test passed",
                                {"results": result["results"],
                                 "ventilation_required": self._auth_vent_required})

    def _battery_refusal(self):
        """Why the charge is too low for an entry (payload/status battery_pct
        below battery_floor_pct, or missing: fail closed), or None."""
        status = self._status()
        pct = _num(status.get("battery_pct")) if isinstance(status, dict) else None
        if pct is None:
            return ("no battery reading in payload/status (an entry needs at least %g %%)"
                    % self.battery_floor_pct)
        if pct < self.battery_floor_pct:
            return ("the battery is at %.0f %%, below the %g %% an entry needs to get back out"
                    % (pct, self.battery_floor_pct))
        return None

    def _withdraw_authorization(self, reason, source="mission", data=None):
        """An ENTRY_AUTHORIZED permit whose conditions changed: ABORTED."""
        with self._lock:
            if self._permit != ENTRY_AUTHORIZED:
                return False
            if self._abort_cause is None:
                self._abort_cause, self._abort_reason = "conditions_changed", reason
        self._event("entry_denied", "Entry NOT authorized any more: %s" % reason, data, source)
        return self._set_permit(ABORTED, "entry authorization withdrawn - %s" % reason, data, source)

    def door_check(self, source="mission"):
        """Re-check just before crossing the door with an ENTRY_AUTHORIZED
        permit (T19): battery, ventilation (when the authorization depended
        on it), the audit log, then a short re-test at the door mouth. Any
        failure withdraws the authorization. -> bool"""
        with self._lock:
            permit, vent_required = self._permit, self._auth_vent_required
        if permit != ENTRY_AUTHORIZED:
            return False
        reasons = []
        battery = self._battery_refusal()
        if battery:
            reasons.append(battery)
        if vent_required:
            vent = self.ventilation_status()
            if vent["source"] != "sim":
                reasons.append("the authorization depends on forced ventilation and there is no "
                               "current simulator ventilation feed")
            elif not vent["on"]:
                reasons.append("forced ventilation stopped after the atmospheric test")
        if getattr(self, "_log_failed", False):
            reasons.append("the audit log can no longer be written")
        if reasons:
            self._withdraw_authorization("; ".join(reasons), source, {"reasons": reasons})
            return False
        epoch = self._motion_epoch
        for attempt in (1, 2):
            test = self.atmospheric_test(at_xy=self.door_mouth_xy,
                                         dwell_s=self.DOOR_RETEST_DWELL_S, withdraw_on_fail=False)
            with self._lock:
                still = self._permit == ENTRY_AUTHORIZED
            if not still:
                return False             # a retreat or a cancel took over meanwhile
            if test["passed"]:
                break
            if self._preempted(epoch):
                return False             # an operator stop: neither a retry nor a withdrawal
            exceeded = any(e.get("passed") is False for e in test["results"])
            if exceeded or attempt == 2:
                self._withdraw_authorization("the re-test at the door did not pass: "
                                             + "; ".join(test["failures"]), source)
                return False
            # Nothing exceeded, the measurement failed (e.g. one lost probe
            # reply): re-test once before giving up on the authorization.
            self._event("door_check", "Door re-test could not measure (%s); re-testing once"
                        % "; ".join(test["failures"]), {"retry": True,
                                                         "failures": test["failures"]}, source)
        with self._lock:
            self._door_check = {"t": self._now(), "permit_id": self.permit_id, "passed": True}
        self._event("door_check", "Door re-test passed just before crossing (battery, %s "
                    "and the atmosphere re-checked)" % ("ventilation" if vent_required
                                                         else "no ventilation dependency"),
                    {"results": test["results"], "ventilation_required": vent_required},
                    source)
        return True

    def _door_check_fresh(self):
        with self._lock:
            check = self._door_check
            return bool(check and check.get("passed") and check.get("permit_id") == self.permit_id
                        and self._now() - check["t"] <= self.DOOR_CHECK_MAX_AGE_S)

    def _purge_hold(self, purge):
        """``purge_hold`` event data (also ``atmospheric_test()["purge_hold"]``)
        from a ``_purge_refusal()`` dict: {"fraction_remaining",
        "purge_target", "eta_s", ...}.

        This is the single source of truth for the hold: the HUD's amber
        PURGE HOLD panel and the agent's tool result show exactly these
        values, so the screen and the spoken answer cannot disagree.
        ``fraction_remaining`` is the value the purge gate judged: the
        simulator's ventilation block, else its last report, else the
        commanded-time estimate (``fraction_source`` says which)."""
        frac, source = _num(purge.get("fraction_remaining")), "sim"
        if frac is None:
            frac, source = _num(purge.get("last_seen_fraction")), "last_seen"
        if frac is None:
            frac, source = _num(purge.get("fraction_estimate")), "estimate"
        if frac is None:
            source = None
        vent = purge.get("ventilation") if isinstance(purge.get("ventilation"), dict) else {}
        eta = _num(purge.get("wait_s"))
        # The gate's own rounded percentage when it gave one (rounded once,
        # in _purge_refusal); never a re-rounding of the 4-digit fraction.
        percent = _num(purge.get("percent_remaining"))
        if percent is None and frac is not None:
            percent = round(100.0 * frac, 1)
        return {
            "fraction_remaining": round(frac, 4) if frac is not None else None,
            "purge_target": self.purge_target,
            "eta_s": round(eta, 1) if eta is not None else None,
            "percent_remaining": percent,
            "fraction_source": source,
            # False: no current simulator feed; the fraction is a last
            # report or an estimate, shown but never authorizing (T12).
            "current_feed": bool(purge.get("current_feed")),
            "ventilation_on": bool(vent.get("on")),
            "target_reachable": vent.get("target_reachable") is not False,
            "prior_failure": bool(purge.get("prior_failure")),
            "simulated": True,
        }

    @staticmethod
    def _purge_hold_message(hold):
        """One line for the purge_hold event, rounded like the HUD (0.1 %)."""
        level = ("%.1f %% of the unventilated contamination remains" % hold["percent_remaining"]
                 if hold["percent_remaining"] is not None else "ventilation progress unknown")
        eta = (", about %.0f s more" % hold["eta_s"]
               if hold["eta_s"] is not None and hold["eta_s"] > 0 else "")
        return ("PURGE HOLD - waiting for ventilation to reach target: %s; entry needs <= %s %%%s"
                % (level, "%g" % round(100.0 * hold["purge_target"], 1), eta))

    def _depth_inside(self, x, y):
        """How far (x, y) is inside the doorway plane along the door normal
        (m), or None when it is not inside."""
        depth = ((self.door_xy[0] - x) * self.door_outward[0]
                 + (self.door_xy[1] - y) * self.door_outward[1])
        return round(depth, 3) if depth > 1e-6 else None

    def _probe_plan(self, pose):
        """(points, info) for the gas probe from ``pose``, or (None, None)
        with a ``probe_unavailable`` event when the robot has no probe.

        Near the door mouth (and outside the space) the probe samples
        PROBE_DEPTHS_M inside the opening x PROBE_HEIGHTS_M (depth-major:
        the 0.5 m heights first); anywhere else the heights at Spot's own
        position."""
        if not callable(getattr(self.robot, "probe", None)):
            self._event("probe_unavailable", "Gas probe unavailable (the robot link has no probe): "
                        "testing with the body gas sensor instead",
                        {"reason": "no probe", "fallback": "body sensor"})
            return None, None
        if pose is None:
            self._event("probe_unavailable", "Gas probe not used (no odometry to aim it): testing "
                        "with the body gas sensor instead",
                        {"reason": "no pose", "fallback": "body sensor"})
            return None, None
        px, py = pose[0], pose[1]
        at_door = (not self.in_space(px, py) and
                   math.hypot(px - self.door_mouth_xy[0], py - self.door_mouth_xy[1])
                   <= self.PROBE_DOOR_RANGE_M)
        stations = list(self.probe_stations_xy) if at_door else [(px, py)]
        points = [[sx, sy, z] for sx, sy in stations for z in self.PROBE_HEIGHTS_M]
        info = {"xy": [round(stations[0][0], 3), round(stations[0][1], 3)],
                "points_xy": [[round(sx, 3), round(sy, 3)] for sx, sy in stations],
                "heights_m": list(self.PROBE_HEIGHTS_M),
                "depths_m": list(self.PROBE_DEPTHS_M) if at_door else None,
                "points": len(points),
                "at": "door" if at_door else "robot",
                "inside_m": self.PROBE_DEPTHS_M[0] if at_door else None,
                "stand_in": self.PROBE_LABEL, "simulated": True}
        return points, info

    @staticmethod
    def _probe_rejection(reason):
        """True if ``reason`` (RobotClient.last_probe_error) says the facade
        answered with an error - out of reach, occluded, no pose - rather
        than a timeout or a dead link: repositioning may fix that."""
        return "rejected by the simulator" in str(reason or "").lower()

    def _probe_once(self, points):
        """robot.probe(points) -> (samples, reason, rejected); samples None
        on failure. Samples carry ``method`` "probe" and their depth inside
        the opening (``inside_m``, None at Spot's own position)."""
        try:
            samples = self.robot.probe(points, timeout=self.PROBE_TIMEOUT_S)
        except Exception as error:
            return None, "probe raised %r" % (error,), False
        if not isinstance(samples, list) or not samples:
            reason = str(getattr(self.robot, "last_probe_error", None) or "no probe reply")
            if self._probe_rejection(reason):
                # Fixed phrases only (T6), whatever the link passed through.
                reason = "probe rejected by the simulator: %s" % probe_error_text(
                    reason.split("rejected by the simulator:", 1)[-1])
            else:
                reason = clean_text(reason, 300)
            return None, reason, self._probe_rejection(reason)
        out = []
        for s in samples:
            if not isinstance(s, dict):
                continue
            x, y, z = _num(s.get("x")), _num(s.get("y")), _num(s.get("z"))
            depth = self._depth_inside(x, y) if x is not None and y is not None else None
            sample = {"x": x, "y": y, "z": z, "alarms": filter_alarms(s.get("alarms")),
                      "method": "probe", "inside_m": depth}
            for key in GAS_KEYS.values():
                value = _num(s.get(key))
                if value is not None:
                    sample[key] = value
            out.append(sample)
        return out, None, False

    def _probe_seated(self, pose):
        """True if the door probe can run from ``pose`` (x, y, yaw) without
        moving: facing the door within PROBE_SEATED_MAX_YAW_DEG, body centre
        within PROBE_SEATED_MAX_LATERAL_M of the door axis, nose outside the
        door plane, and every probe station within the probe reach minus
        PROBE_SEATED_REACH_MARGIN_M of the nose (horizontal, like the
        facade's check)."""
        if pose is None:
            return False
        ox, oy = self.door_outward
        facing = math.atan2(-oy, -ox)
        if abs(wrap_angle(pose[2] - facing)) > math.radians(self.PROBE_SEATED_MAX_YAW_DEG):
            return False
        dx, dy = pose[0] - self.door_xy[0], pose[1] - self.door_xy[1]
        if abs(dy * ox - dx * oy) > self.PROBE_SEATED_MAX_LATERAL_M:
            return False
        nx = pose[0] + self.probe_front_m * math.cos(pose[2])
        ny = pose[1] + self.probe_front_m * math.sin(pose[2])
        if (nx - self.door_xy[0]) * ox + (ny - self.door_xy[1]) * oy < 0.0:
            return False
        limit = self.probe_reach_m - self.PROBE_SEATED_REACH_MARGIN_M
        return all(math.hypot(nx - sx, ny - sy) <= limit for sx, sy in self.probe_stations_xy)

    def _seat_at_door_mouth(self, timeout=60.0, force=False):
        """Stand at door_mouth_xy precisely enough for the door probe (see
        PROBE_SEAT_LEAD_M). Already within the probe envelope
        (``_probe_seated``) or the seat tolerance: no move, unless ``force``
        (the re-seat after a facade rejection). Within SEAT_STEP_MAX_M: a
        holonomic step that squares up to the door (``_seat_step``); farther:
        a walk. Every decision is logged (``probe_seat``: at_target,
        in_envelope, step, walk). -> bool (False: refused / timed out /
        halted)."""
        lead = self.DOOR_MOUTH_OFFSET_M - self.PROBE_SEAT_LEAD_M
        tx = self.door_xy[0] + self.door_outward[0] * lead
        ty = self.door_xy[1] + self.door_outward[1] * lead
        facing = math.atan2(-self.door_outward[1], -self.door_outward[0])
        pose = self._pose()
        dist = math.hypot(tx - pose[0], ty - pose[1]) if pose is not None else None
        if not force and dist is not None and dist <= self.PROBE_SEAT_TOL_M:
            decision = "at_target"
        elif not force and self._probe_seated(pose):
            decision = "in_envelope"
        elif dist is not None and dist <= self.SEAT_STEP_MAX_M:
            decision = "step"
        else:
            decision = "walk"
        ok = True
        if decision == "step":
            # Forced (the facade rejected this pose): all the way to the target.
            ok = self._seat_step(tx, ty, facing, envelope_ok=not force)
        elif decision == "walk":
            ok = self.walk_to(tx, ty, tol=self.PROBE_SEAT_TOL_M, timeout=timeout)
        after = self._pose() if decision in ("step", "walk") else pose
        # Audit only (the HUD hides it): rehearsals count skips vs moves.
        self._event("probe_seat", "Probe seat: %s (%s m from the seat target%s)%s"
                    % (decision, "%.3f" % dist if dist is not None else "?",
                       ", forced after a rejection" if force else "",
                       "" if ok else " - did not reach it"),
                    {"decision": decision, "dist_m": round(dist, 3) if dist is not None else None,
                     "tol_m": self.PROBE_SEAT_TOL_M, "force": bool(force), "ok": bool(ok),
                     "pose": [round(v, 3) for v in pose] if pose is not None else None,
                     "pose_after": [round(v, 3) for v in after] if after is not None else None,
                     "target_xy": [round(tx, 3), round(ty, 3)]})
        return ok

    def _seat_step(self, x, y, yaw, envelope_ok=True):
        """Short holonomic move to (x, y): body-frame vx/vy toward it while
        wz squares the heading to ``yaw`` (errors above
        SEAT_STEP_MAX_YAW_FIX_DEG are held, not turned: no big turn at the
        door). Stops translating once in the probe envelope
        (``_probe_seated``; unless not ``envelope_ok``: the re-seat after a
        rejection) and ends when squared up there or at the target.
        Never steers toward the target, so a hop of a few cm cannot swing
        Spot round (see SEAT_STEP_MAX_M). -> True once within
        PROBE_SEAT_TOL_M or in the envelope (a timeout there still counts)."""
        self._ensure_guard()
        if self._cancel.is_set() or self._retreating:
            return False
        if not self._motion_lock.acquire(blocking=False):
            self._event("walk_refused", "Another motion command is in progress")
            return False
        epoch = self._motion_epoch
        start = self._now()
        deadline = start + self.SEAT_STEP_TIMEOUT_S
        last_pose_t = start
        next_tick = start
        result = "timeout"
        max_fix = math.radians(self.SEAT_STEP_MAX_YAW_FIX_DEG)
        yaw_tol = math.radians(self.SEAT_STEP_YAW_TOL_DEG)
        try:
            while True:
                interrupted = self._interrupted("mission", epoch)
                if interrupted:
                    result = interrupted
                    break
                now = self._now()
                if now >= deadline:
                    break
                hold = self.drive_hold_reason()
                pose = self._pose() if hold is None else None
                if pose is None:
                    self._send(0.0, 0.0, 0.0, "mission")
                    if now - last_pose_t > self.POSE_LOST_S:
                        result = "held" if hold else "pose_lost"
                        self._last_hold = hold
                        break
                else:
                    last_pose_t = now
                    gas_hold = self._gas_hold_reason(pose, (x, y))
                    if gas_hold:
                        self._send(0.0, 0.0, 0.0, "mission")   # no gas reading: hold (T17)
                        self._note_gas_hold(gas_hold, pose, "mission")
                        next_tick = self._tick(next_tick)
                        continue
                    self._end_gas_hold(resumed=True)
                    px, py, pyaw = pose
                    dx, dy = x - px, y - py
                    dist = math.hypot(dx, dy)
                    err = wrap_angle(yaw - pyaw)
                    if abs(err) > max_fix:
                        err = 0.0
                    # Done once squared up at the target, or anywhere in the
                    # probe envelope (the smallest move; nose stays outside).
                    placed = dist <= self.SEAT_STEP_TOL_M or (envelope_ok and self._probe_seated(pose))
                    if placed and abs(err) <= yaw_tol:
                        result = "reached"
                        break
                    vx = vy = 0.0
                    if not placed:
                        speed = min(self.SEAT_STEP_MAX_V,
                                    max(self.SEAT_STEP_MIN_V, self.SEAT_STEP_K_V * dist))
                        c, s = math.cos(pyaw), math.sin(pyaw)
                        vx = speed * (c * dx + s * dy) / dist
                        vy = speed * (-s * dx + c * dy) / dist
                    wz = max(-self.SEAT_STEP_MAX_WZ, min(self.SEAT_STEP_MAX_WZ,
                                                         self.SEAT_STEP_K_YAW * err))
                    if abs(err) <= yaw_tol:
                        wz = 0.0
                    elif abs(wz) < self.MIN_TURN_WZ and vx == 0.0 and vy == 0.0:
                        wz = math.copysign(self.MIN_TURN_WZ, err)
                    self._send(vx, vy, wz, "mission")
                next_tick = self._tick(next_tick)
        finally:
            self._end_gas_hold(resumed=False)
            self._end_motion(result, "mission")
            self._motion_lock.release()
        if result not in ("reached", "timeout"):
            return False
        pose = self._pose()
        return pose is not None and (math.hypot(x - pose[0], y - pose[1]) <= self.PROBE_SEAT_TOL_M
                                     or (envelope_ok and self._probe_seated(pose)))

    def _reposition_for_probe(self, reason, epoch):
        """After a rejected door probe: re-seat Spot at door_mouth_xy facing
        the door (the facade measures reach from the nose and checks the
        line through the opening). -> True/False (moved), None if a
        retreat/cancel interrupted."""
        short = str(reason).split("rejected by the simulator:", 1)[-1].strip()
        self._event("probe_rejected", "Gas probe request rejected by the simulator (%s): "
                    "repositioning Spot at the door mouth and retrying once" % short,
                    {"error": short, "reason": reason, "action": "reposition_and_retry",
                     "door_mouth_xy": list(self.door_mouth_xy), "pose": self._last_pose,
                     "stand_in": self.PROBE_LABEL, "simulated": True})
        ok = self._seat_at_door_mouth(timeout=30.0, force=True)
        if self._preempted(epoch):
            return None
        pose = self._pose()
        if ok and pose is not None:
            ok = self.face(math.atan2(self.door_xy[1] - pose[1], self.door_xy[0] - pose[0]),
                           tol_deg=self.PROBE_REPOSITION_TOL_DEG, timeout=15.0)
        if self._preempted(epoch):
            return None
        return bool(ok)

    def _collect_phase(self, duration, epoch, probe_state):
        """Samples for one test phase: probe requests every PROBE_PERIOD_S
        (all points each time) while the probe works. A facade rejection of
        a door probe re-seats Spot and retries once per test (the phase
        dwell restarts after the re-seat); after a probe failure, the body
        sensor for the rest of the test (one ``probe_unavailable`` event).
        None if a retreat/cancel interrupts."""
        end = self._now() + duration
        samples = []
        rest = duration
        while probe_state["points"] is not None:
            if self._preempted(epoch):
                return None
            readings, reason, rejected = self._probe_once(probe_state["points"])
            if (readings is None and rejected and probe_state.get("at") == "door"
                    and not probe_state.get("retried")):
                probe_state["retried"] = True
                moved = self._reposition_for_probe(reason, epoch)
                if moved is None:
                    return None
                readings, reason, rejected = self._probe_once(probe_state["points"])
                if readings is not None:
                    self._event("probe_retry", "Gas probe accepted after repositioning at the "
                                "door mouth", {"ok": True, "moved": moved, "pose": self._last_pose,
                                               "simulated": True})
                end = self._now() + duration
            if readings is None:
                if rejected and probe_state.get("retried"):
                    probe_state["gave_up"] = True
                    reason = "rejected again after repositioning at the door mouth: %s" % reason
                probe_state["points"], probe_state["reason"] = None, reason
                self._event("probe_unavailable", "Gas probe unavailable (%s): continuing the test "
                            "with the body gas sensor" % reason,
                            {"reason": reason, "fallback": "body sensor",
                             "rejected": bool(rejected)})
                # The rest of the phase, and at least ~1 s of samples when a
                # probe timeout used the phase up.
                rest = max(end - self._now(), 4 * self.ATMOS_SAMPLE_PERIOD_S)
                break
            samples.extend(readings)
            if self._now() >= end:
                return samples
            self._clock.sleep(min(self.PROBE_PERIOD_S, max(0.0, end - self._now())))
        body = self._collect_gas(rest, epoch)
        if body is None:
            return None
        return samples + [dict(s, method="body") for s in body]

    @staticmethod
    def _probe_rows(phase_samples):
        """Per-point rows (worst value per gas, from that gas's own phase)
        for the HUD's probe panel: [{"inside_m", "z", "x", "y", o2_pct,
        lel_pct, h2s_ppm, co_ppm}], nearest depth first, then by height."""
        points = {}
        for samples in phase_samples.values():
            for s in samples:
                z = _num(s.get("z"))
                if s.get("method") != "probe" or z is None:
                    continue
                x, y = _num(s.get("x")), _num(s.get("y"))
                key = (round(x, 3) if x is not None else None,
                       round(y, 3) if y is not None else None, round(z, 3))
                points.setdefault(key, _num(s.get("inside_m")))
        rows = []
        for (x, y, z), depth in points.items():
            row = {"inside_m": round(depth, 2) if depth is not None else None, "z": z,
                   "x": x, "y": y}
            for gas, samples in phase_samples.items():
                values = [_num(s.get(GAS_KEYS[gas])) for s in samples
                          if s.get("method") == "probe" and _num(s.get("z")) is not None
                          and abs(float(s["z"]) - z) < 1e-6
                          and (x is None or abs((_num(s.get("x")) or 0.0) - x) < 1e-3)
                          and (y is None or abs((_num(s.get("y")) or 0.0) - y) < 1e-3)]
                values = [v for v in values if v is not None]
                if values:
                    row[GAS_KEYS[gas]] = round(_worst_value(gas, values), 3)
            rows.append(row)
        rows.sort(key=lambda r: (r["inside_m"] is None, r["inside_m"] or 0.0, r["z"]))
        return rows

    def _collect_gas(self, duration, epoch=None):
        """Samples of payload/gas over ``duration`` s; None if interrupted."""
        samples = []
        end = self._now() + duration
        epoch = self._motion_epoch if epoch is None else epoch
        while True:
            if self._preempted(epoch):
                return None
            try:
                reading = filter_gas(self.robot.gas())
            except Exception as error:
                LOG.warning("gas() raised %s", error)
                reading = None
            if reading is not None:
                samples.append(reading)
            if self._now() >= end:
                return samples
            self._clock.sleep(self.ATMOS_SAMPLE_PERIOD_S)

    @staticmethod
    def _judge(gas, samples):
        """Verdict for one gas over all samples (every probe point): worst
        value, the height (and depth inside the opening) it was found at,
        and the worst per height."""
        key, unit, label = GAS_KEYS[gas], GAS_UNITS[gas], GAS_LABELS[gas]
        pairs = [(v, s) for v, s in ((_num(s.get(key)), s) for s in samples) if v is not None]
        values = [v for v, _s in pairs]
        facade_alarm = any(bool((s.get("alarms") or {}).get(gas))
                           for s in samples if isinstance(s.get("alarms"), dict))
        methods = {s.get("method", "body") for s in samples}
        method = methods.pop() if len(methods) == 1 else ("probe+body" if methods else "body")
        entry = {"gas": gas, "key": key, "unit": unit, "limit": GAS_LIMITS[gas],
                 "samples": len(values), "facade_alarm": facade_alarm, "method": method,
                 "worst_z_m": None, "worst_inside_m": None, "by_height": None, "by_point": None}
        if not values:
            entry.update(value=None, min=None, max=None, passed=False,
                         message="%s FAIL - no sensor data (limit %s)" % (label, GAS_LIMITS[gas]))
            return entry
        lo, hi = min(values), max(values)
        worst = _worst_value(gas, values)
        worst_sample = next(s for v, s in pairs if v == worst)
        by_height, by_point = {}, {}
        for v, s in pairs:
            z = _num(s.get("z"))
            if z is not None and s.get("method") == "probe":
                by_height.setdefault("%.2f" % z, []).append(v)
                depth = _num(s.get("inside_m"))
                if depth is not None:
                    by_point.setdefault("%.2f@%.2f" % (depth, z), []).append(v)
        if by_height:
            entry["by_height"] = {z: round(_worst_value(gas, vs), 3) for z, vs in sorted(by_height.items())}
            wz = _num(worst_sample.get("z"))
            is_probe = worst_sample.get("method") == "probe"
            entry["worst_z_m"] = round(wz, 3) if wz is not None and is_probe else None
            wd = _num(worst_sample.get("inside_m"))
            entry["worst_inside_m"] = round(wd, 2) if wd is not None and is_probe else None
        if by_point:
            # "depth@height" (m) -> worst value there
            entry["by_point"] = {p: round(_worst_value(gas, vs), 3) for p, vs in sorted(by_point.items())}
        passed = all(gas_passes(gas, v) for v in values) and not facade_alarm
        if entry["worst_z_m"] is None:
            where = ""
        elif by_point and len(by_point) > len(by_height):
            where = (", worst at %.1f m height, %.1f m inside the opening; %d probe points"
                     % (entry["worst_z_m"], entry["worst_inside_m"] or 0.0, len(by_point)))
        else:
            where = ", worst at %.1f m of %d heights" % (entry["worst_z_m"], len(by_height))
        entry.update(value=round(worst, 3), min=round(lo, 3), max=round(hi, 3), passed=passed,
                     message="%s %.1f %s %s (limit %s%s, %d samples)"
                             % (label, worst, _MSG_UNITS[gas], "PASS" if passed else "FAIL",
                                GAS_LIMITS[gas], where, len(values)))
        return entry

    def enter(self, on_waypoint=None):
        """Walk through door_xy then every interior waypoint (needs an
        authorized or in-progress permit). ``on_waypoint(index, x, y)`` runs
        after each interior waypoint is reached. -> bool"""
        with self._lock:
            permit = self._permit
        if permit not in (ENTRY_AUTHORIZED, ENTRY_IN_PROGRESS):
            self._event("entry_blocked", "enter() refused: permit is %s" % permit)
            return False
        if self.backend_refusal("entering the permit space"):
            return False
        if not self.walk_to(*self.door_xy):
            return False
        self._event("waypoint", "At the cleanout door threshold - entering the permit space",
                    {"index": -1, "name": "door", "xy": self.door_xy})
        total = len(self.waypoints)
        for index, (wx, wy) in enumerate(self.waypoints):
            if not self.walk_to(wx, wy):
                return False
            with self._lock:
                self._waypoints_reached = max(self._waypoints_reached, index + 1)
            pose = self._pose() or (wx, wy, 0.0)
            self._event("waypoint", "Reached interior waypoint %d/%d (%.1f, %.1f)"
                        % (index + 1, total, wx, wy),
                        {"index": index, "xy": [wx, wy], "pose": pose})
            if on_waypoint is not None:
                try:
                    on_waypoint(index, pose[0], pose[1])
                except Exception as error:
                    LOG.exception("on_waypoint hook failed")
                    self._event("error", "on_waypoint hook failed: %r" % (error,))
        return True

    def exit_space(self):
        """Normal exit: retrace the visited path out through the door to
        entry_xy, close the permit (COMPLETED) and sit. -> bool"""
        if self.backend_refusal("moving the robot out of the permit space"):
            return False
        if not self._motion_lock.acquire(blocking=False):
            return False
        try:
            with self._lock:
                if self._permit != ENTRY_IN_PROGRESS or self._retreating:
                    return False
                self._exiting = True
                epoch = self._motion_epoch
            path = self._outward_path(self._pose())
            self._event("exit_start", "Exiting via the visited path (%d legs)" % len(path),
                        {"path": path})
            for point in path:
                status = self._navigate(point[0], point[1], self.WAYPOINT_TOL_M, self.CRUISE_SPEED,
                                        self._leg_timeout(point, self.CRUISE_SPEED),
                                        owner="mission", label="exit", epoch=epoch)
                if status != "reached":
                    return False
                self._pop_trail_to(point)
        finally:
            self._motion_lock.release()
        if not self._set_permit(COMPLETED, "entrant exited the permit space; permit closed"):
            with self._lock:  # aborted meanwhile (retreat/cancel): not a completed entry
                self._exiting = False
            return False
        with self._lock:
            self._exiting = False
            self._trail = []
        self._sit_if_outside()
        return True

    def _pop_trail_to(self, point):
        with self._lock:
            while self._trail and self._trail[-1] != point:
                self._trail.pop()

    def _outward_path(self, pose):
        """Points from ``pose`` back out to entry_xy along the visited path.

        Empty when Spot is already outside and clear of the doorway: a
        "retreat" must never walk it back toward (or, from beside the shell,
        straight through) the tank, and a stale trail is ignored once outside.
        """
        pose = pose or self._last_pose
        px, py = (pose[0], pose[1]) if pose else self.entry_xy
        with self._lock:
            trail = list(self._trail)
            exiting = self._exiting
        tol = self.WAYPOINT_TOL_M
        if not self.in_space(px, py):
            if self._clear_of_space(px, py):
                return []
            trail = []  # just outside the door: back straight off to entry_xy
        if trail:
            if exiting and math.hypot(px - trail[-1][0], py - trail[-1][1]) > tol:
                trail = trail[:-1]  # already left trail[-1] heading outward
            path = list(reversed(trail))
        elif (self.in_space(px, py) and
              math.hypot(px - self.door_xy[0], py - self.door_xy[1]) > self.UNPLANNED_EXIT_M):
            # Driven in without a recorded trail (e.g. raw cmd_vel from the
            # agent): chain back from the nearest planned point. Best effort -
            # the first segment was never planned.
            chain = [self.door_xy] + self.waypoints
            nearest = min(range(len(chain)),
                          key=lambda i: math.hypot(px - chain[i][0], py - chain[i][1]))
            path = list(reversed(chain[:nearest + 1]))
        else:
            path = []
        path.append(self.entry_xy)
        while len(path) > 1 and math.hypot(path[0][0] - px, path[0][1] - py) <= tol:
            path.pop(0)
        return self._skip_straight_door(path, (px, py))

    def _skip_straight_door(self, path, start):
        """Don't stop in the doorway when the route runs straight through it:
        a stop there followed by a turn would sweep Spot into the jambs."""
        out = []
        for i, point in enumerate(path):
            is_door = math.hypot(point[0] - self.door_xy[0], point[1] - self.door_xy[1]) < 1e-6
            if is_door and i + 1 < len(path):
                prev = out[-1] if out else start
                nxt = path[i + 1]
                a = math.atan2(point[1] - prev[1], point[0] - prev[0])
                b = math.atan2(nxt[1] - point[1], nxt[0] - point[0])
                if abs(wrap_angle(b - a)) < math.radians(15.0):
                    continue
            out.append(point)
        return out

    def _sit_if_outside(self):
        pose = self._pose() or self._last_pose
        if pose is not None and self.in_space(pose[0], pose[1]):
            self._event("robot", "Not sitting inside the permit space; holding position")
            return False
        # After a guard/operator retreat only: one that backed out of an
        # excursion near the door must not sit down in the pocket's tail.
        fresh_air = self._retreating and self._body_in_alarm()
        if pose is not None and not self._step_to_standoff(pose, fresh_air=fresh_air):
            return False
        if self._retreating and self._body_in_alarm():
            gas = self._gas_now() or {}
            self._event("robot", "Gas still in alarm where Spot sits down outside the permit space "
                        "(%s); operator attention required" % format_reading(gas),
                        {"reading": gas, "pose": self._pose() or self._last_pose})
        # A retreat's sit outside passes a backend identity conflict (T9).
        ok, message = self._call("sit", retreat=self._retreating)
        self._event("robot", "sit: %s - %s" % ("ok" if ok else "FAILED", message),
                    {"service": "sit", "ok": ok, "message": message})
        return ok

    def _gas_now(self):
        try:
            return filter_gas(self.robot.gas())
        except Exception:
            return None

    def _body_in_alarm(self):
        """True if the latest body gas reading is an OSHA alarm (False when
        there is no reading: no telemetry is not a reason to move)."""
        gas = self._gas_now()
        return bool(gas) and bool(alarm_reasons(gas))

    def _step_to_standoff(self, pose, fresh_air=False):
        """Before sitting with the doorway BEHIND Spot and closer than
        SIT_STANDOFF_M, walk straight out along the door axis to that
        standoff. Why: standing up from a sit shoves Spot backward (the
        facade's scripted stand-up moved it 0.3-0.65 m on the host), and an
        exit or retreat leaves Spot at entry_xy facing away from the tank, so
        the next stand backed its hind legs into the door frame and it fell.
        Facing the door (e.g. after backing off an entry denial) the shove
        points away from the tank, so it sits where it is - unless
        ``fresh_air`` (a retreat that backed out of an excursion near the
        door and still reads an alarm in the pocket's tail): then it backs
        out to the standoff too. -> False only if a retreat/cancel
        interrupted the step (then it must not sit)."""
        px, py, yaw = pose
        bearing = math.atan2(self.door_xy[1] - py, self.door_xy[0] - px)
        gap = math.hypot(px - self.door_xy[0], py - self.door_xy[1])
        facing_door = abs(wrap_angle(bearing - yaw)) <= math.pi / 2.0
        if gap >= self.SIT_STANDOFF_M - self.WAYPOINT_TOL_M or (facing_door and not fresh_air):
            return True
        sx = self.door_xy[0] + self.door_outward[0] * self.SIT_STANDOFF_M
        sy = self.door_xy[1] + self.door_outward[1] * self.SIT_STANDOFF_M
        owner = "retreat" if self._retreating else "mission"
        status = self._navigate(sx, sy, self.WAYPOINT_TOL_M, self.CRUISE_SPEED,
                                self._leg_timeout((sx, sy), self.CRUISE_SPEED), owner,
                                allow_reverse=True, label="standoff")
        return status not in ("halted", "cancelled")

    def _dwell(self, seconds, epoch=None):
        """Stand still for ``seconds``; False if a retreat/cancel interrupts."""
        end = self._now() + seconds
        epoch = self._motion_epoch if epoch is None else epoch
        while self._now() < end:
            if self._preempted(epoch):
                return False
            self._clock.sleep(0.05)
        return True

    # ------------------------------------------------------------------ #
    # SIMULATED atmosphere controls (facade payload/gas/*)
    # ------------------------------------------------------------------ #

    def _robot_fn(self, name):
        fn = getattr(self.robot, name, None)
        return fn if callable(fn) else None

    def _set_ventilation(self, on, tau_s, source):
        fn = self._robot_fn("set_ventilation")
        if fn is None:
            self._event("ventilation", "Forced ventilation control unavailable on this robot link",
                        {"on": False, "ok": False, "simulated": True}, source)
            return False
        try:
            return fn(bool(on), tau_s) is not False
        except Exception as error:
            LOG.warning("set_ventilation failed: %s", error)
            return False

    def _purge_estimate_s(self, tau):
        """Seconds from the unventilated tank to the purge target at time
        constant ``tau`` (facade default residual assumed)."""
        r = self.VENT_ASSUMED_RESIDUAL
        if self.purge_target <= r:
            return self.VENT_PURGE_TAUS * tau
        return tau * math.log((1.0 - r) / (self.purge_target - r))

    def start_ventilation(self, tau_s=None, source="mission"):
        """Switch SIMULATED forced ventilation on (built-in contamination
        decays with time constant ``tau_s`` toward a small residual).
        Returns {"ok", "on", "tau_s", "purge_estimate_s", "simulated"}."""
        tau = self.VENT_DEFAULT_TAU_S if tau_s is None else _num(tau_s)
        lo, hi = self.VENT_TAU_RANGE_S
        if tau is None or not lo <= tau <= hi:
            return {"ok": False, "on": False, "error": "tau_s must be %g-%g s" % (lo, hi)}
        refusal = self.backend_refusal("starting the simulated ventilation", source)
        if refusal:
            return {"ok": False, "on": False, "error": refusal}
        ok = self._set_ventilation(True, tau, source)
        purge = self._purge_estimate_s(tau)
        if ok:
            with self._lock:
                self._vent = {"on": True, "tau_s": tau, "t_on": self._now()}
        self._event("ventilation",
                    "Forced ventilation ON (SIMULATED blower at the cleanout door; time constant "
                    "%.0f s, purge target in about %.0f s)" % (tau, purge) if ok else
                    "Forced ventilation command FAILED (robot link down)",
                    {"on": ok, "ok": ok, "tau_s": tau, "purge_estimate_s": round(purge, 1),
                     "simulated": True}, source)
        return {"ok": ok, "on": ok, "tau_s": tau, "purge_estimate_s": round(purge, 1),
                "simulated": True}

    def stop_ventilation(self, source="mission"):
        """Switch SIMULATED ventilation off (the sim holds the current level)."""
        refusal = self.backend_refusal("stopping the simulated ventilation", source)
        if refusal:
            return {"ok": False, "on": False, "error": refusal}
        ok = self._set_ventilation(False, None, source)
        if ok:
            with self._lock:
                self._vent = {"on": False, "tau_s": self._vent.get("tau_s"), "t_on": None}
        self._event("ventilation", "Forced ventilation OFF (simulated)" if ok else
                    "Ventilation-off command FAILED", {"on": False, "ok": ok, "simulated": True},
                    source)
        return {"ok": ok, "on": False}

    def _note_vent_block(self, reading):
        """Remember the latest payload/gas "ventilation" block (for the
        summary when the feed is momentarily stale)."""
        block = reading.get("ventilation") if isinstance(reading, dict) else None
        if isinstance(block, dict):
            with self._lock:
                self._vent_seen = (dict(block), self._now())
        return block if isinstance(block, dict) else None

    def ventilation_status(self):
        """Ventilation state and purge progress. Prefers the facade's
        payload/gas "ventilation" block (fraction_remaining: 1.0 = the
        unventilated tank); without it, estimates the fraction from the
        time since start_ventilation() (anchored at the unventilated 1.0 and
        the facade's default residual, so the estimate errs high).
        ``purged`` = the simulator's current fraction_remaining at or below
        the purge target (``purge_target``, default
        VENT_PURGE_TARGET_FRACTION). The estimate is reported as
        ``fraction_estimate`` (``estimate_reached`` when it is at the
        target) but never sets ``purged``: a timer is not a measurement
        and never authorizes entry (T12)."""
        try:
            gas = filter_gas(self.robot.gas())
        except Exception:
            gas = None
        block = self._note_vent_block(gas)
        with self._lock:
            local = dict(self._vent)
        target = self.purge_target
        frac = resid = estimate = None
        if block is not None:
            on = bool(block.get("on"))
            tau = _num(block.get("tau_s"))
            frac = _num(block.get("fraction_remaining"))
            resid = _num(block.get("residual_fraction"))
            t_on = _num(block.get("t_on"))
            sim_fn = self._robot_fn("sim_time")
            sim_t = None
            if sim_fn is not None:
                try:
                    sim_t = _num(sim_fn())
                except Exception:
                    sim_t = None
            elapsed = (sim_t - t_on) if (on and t_on is not None and sim_t is not None) else None
        else:
            on, tau = bool(local["on"]), local["tau_s"]
            elapsed = None
        pending = False
        if (block is not None and not on and local["on"] and local["t_on"] is not None
                and self._now() - local["t_on"] < self.VENT_CONFIRM_S):
            # Commanded on a moment ago; the sim's next gas sample confirms it.
            on, pending, tau = True, True, local["tau_s"]
        if elapsed is None and on and local["t_on"] is not None:
            elapsed = self._now() - local["t_on"]
        reachable = resid is None or resid < target
        estimate_reached = False
        if pending:
            purged, remaining = False, (self.VENT_PURGE_TAUS * tau if tau else None)
        elif frac is not None:
            purged = frac <= target + 1e-9
            remaining = (0.0 if purged else
                         tau * math.log((frac - resid) / (target - resid))
                         if on and tau and resid is not None and reachable and frac > resid
                         else None)
        else:
            r = self.VENT_ASSUMED_RESIDUAL
            full = tau * math.log((1.0 - r) / (target - r)) if tau and target > r else None
            if on and tau and elapsed is not None:
                estimate = r + (1.0 - r) * math.exp(-max(0.0, elapsed) / tau)
            # An estimate from the commanded time, not a measurement (T12).
            purged = False
            estimate_reached = bool(estimate is not None and estimate <= target + 1e-9)
            remaining = (max(0.0, full - elapsed) if on and full is not None and elapsed is not None
                         else None)
        return {
            "on": on, "tau_s": tau, "fraction_remaining": frac,
            "percent_remaining": round(100.0 * frac, 1) if frac is not None else None,
            "fraction_estimate": round(estimate, 4) if estimate is not None else None,
            "estimate_reached": estimate_reached,
            "residual_fraction": resid, "purge_target_fraction": target,
            "target_reachable": reachable,
            "elapsed_s": round(elapsed, 1) if elapsed is not None else None,
            "purged": purged, "remaining_s": round(remaining, 1) if remaining is not None else None,
            "source": "sim" if block is not None else "local", "simulated": True,
            "pending_confirmation": pending,
        }

    def await_ventilation(self, max_s=None, source="mission"):
        """Wait (at most ``max_s``, capped at VENT_MAX_WAIT_S) until the purge
        target is reached, logging progress every VENT_PROGRESS_S. Returns
        ventilation_status() plus "ok" and "waited_s". Never moves Spot."""
        limit = self.VENT_MAX_WAIT_S if max_s is None else max(0.0, min(float(max_s),
                                                                         self.VENT_MAX_WAIT_S))
        start = last = self._now()
        with self._lock:
            start_epoch = self._motion_epoch
        while True:
            status = self.ventilation_status()
            waited = round(self._now() - start, 1)
            with self._lock:
                halted = (self._operator_halt_epoch is not None
                          and self._operator_halt_epoch > start_epoch)
            if halted:
                return dict(status, ok=False, waited_s=waited, error=self.STOPPED_TEXT)
            if status["purged"]:
                self._event("ventilation", "Purge target reached (<= %.0f %%)%s: ready for the "
                            "re-test (simulated atmosphere)" % (
                                100.0 * self.purge_target,
                                " - %.1f %% of the unventilated contamination remains"
                                % status["percent_remaining"]
                                if status["percent_remaining"] is not None else ""),
                            dict(status, waited_s=waited), source)
                return dict(status, ok=True, waited_s=waited)
            if status["estimate_reached"]:
                # Only the commanded-time estimate says so (T12): labelled,
                # and never reported as the purge target reached.
                error = ("by an ESTIMATE from the commanded ventilation time, about %.0f %% of the "
                         "unventilated contamination remains, but there is no current simulator "
                         "ventilation feed: an estimate is not a measurement and does not "
                         "authorize entry" % (100.0 * status["fraction_estimate"]))
                self._event("ventilation", "Purge NOT confirmed: " + error,
                            dict(status, waited_s=waited), source)
                return dict(status, ok=False, waited_s=waited, error=error)
            if not status["target_reachable"]:
                error = ("the purge target (%.0f %%) is below the simulator's ventilation residual "
                         "(%.0f %%); it cannot be reached" % (100.0 * self.purge_target,
                                                              100.0 * status["residual_fraction"]))
                self._event("ventilation", "Purge target unreachable: " + error,
                            dict(status, waited_s=waited), source)
                return dict(status, ok=False, waited_s=waited, error=error)
            if not status["on"]:
                return dict(status, ok=False, waited_s=waited, error="ventilation is off")
            if self._cancel.is_set():
                return dict(status, ok=False, waited_s=waited, error="cancelled")
            now = self._now()
            if now - start >= limit:
                self._event("ventilation", "Purge target not reached within %.0f s" % limit,
                            dict(status, waited_s=waited), source)
                return dict(status, ok=False, waited_s=waited,
                            error="purge not complete within %.0f s" % limit)
            if now - last >= self.VENT_PROGRESS_S:
                last = now
                self._event("ventilation", "Ventilating (simulated): %s, about %s to the purge "
                            "target (<= %.0f %%)"
                            % ("%.0f %% of the contamination remains" % status["percent_remaining"]
                               if status["percent_remaining"] is not None else "purging",
                               "%.0f s" % status["remaining_s"] if status["remaining_s"] is not None
                               else "? s", 100.0 * self.purge_target),
                            dict(status, waited_s=waited), source)
            self._clock.sleep(0.25)

    def reset_atmosphere(self, source="mission"):
        """SIMULATION CONTROL (demo reset): remove injected pockets and put
        the tank back to the simulator's initial atmosphere
        (payload/gas/clear + payload/gas/ventilation {"reset": true})."""
        refusal = self.backend_refusal("resetting the simulated atmosphere", source)
        if refusal:
            return {"ok": False, "cleared": False, "ventilation_reset": False, "error": refusal}
        results = {}
        for name in ("clear_gas", "reset_ventilation"):
            fn = self._robot_fn(name)
            try:
                results[name] = fn is not None and fn() is not False
            except Exception as error:
                LOG.warning("%s failed: %s", name, error)
                results[name] = False
        ok = all(results.values())
        if results["clear_gas"]:
            self._forget_presenter_injections()
        with self._lock:
            self._vent = {"on": False, "tau_s": None, "t_on": None}
            self._vent_seen = None     # a pre-reset block must not vouch for the purge
        self._event("sim_control", "SIMULATION CONTROL (demo reset): injected gas pockets cleared "
                    "and the tank atmosphere reset to the simulator's initial state%s"
                    % ("" if ok else " - NOT CONFIRMED (%s)" % ", ".join(
                        k for k, v in results.items() if not v)),
                    {"ok": ok, "cleared": results["clear_gas"],
                     "ventilation_reset": results["reset_ventilation"], "simulated": True}, source)
        return {"ok": ok, "cleared": results["clear_gas"],
                "ventilation_reset": results["reset_ventilation"]}

    def clear_injected_gas(self, source="mission"):
        """Presenter control: remove every injected (staged) gas pocket."""
        refusal = self.backend_refusal("clearing simulated gas", source)
        if refusal:
            return {"ok": False, "error": refusal}
        fn = self._robot_fn("clear_gas")
        try:
            ok = fn is not None and fn() is not False
        except Exception as error:
            LOG.warning("clear_gas failed: %s", error)
            ok = False
        if ok:
            self._forget_presenter_injections()
        self._event("gas_cleared", "PRESENTER DEMO: injected (simulated) gas pockets cleared" if ok
                    else "Clearing injected gas FAILED (robot link down)",
                    {"ok": ok, "simulated": True}, source)
        return {"ok": ok}

    # ------------------------------------------------------------------ #
    # Guard integration and retreat
    # ------------------------------------------------------------------ #

    def _guard_armed(self):
        """Armed while an entry is authorized or in progress, while a fresh
        pose puts Spot in the permit space, and whenever there is no fresh
        pose (unknown is treated as inside: a withheld or implausible pose
        cannot disarm it, T22); not while a retreat owns the robot."""
        with self._lock:
            if self._retreating:
                return False
            if self._permit in (ENTRY_AUTHORIZED, ENTRY_IN_PROGRESS):
                return True
        pose = self._pose()
        return pose is None or self.in_space(pose[0], pose[1])

    def _on_sample(self, reading, armed):
        self.backend()        # the guard thread records a backend identity conflict (T9)
        block = self._note_vent_block(reading)
        if block is not None and block.get("on") is False:
            self._ventilation_stopped()
        with self._lock:
            active = armed or self._retreating
            if not active:
                return
            for gas in OSHA_TEST_ORDER:
                key = GAS_KEYS[gas]
                value = _num(reading.get(key))
                if value is None:
                    continue
                lo, hi = self._gas_extremes.get(key, (value, value))
                self._gas_extremes[key] = (min(lo, value), max(hi, value))
            now = self._now()
            due = now - self._last_monitor_t >= self.MONITOR_EVENT_PERIOD_S
            if due:
                self._last_monitor_t = now
        if due:
            self._event("monitor", format_reading(reading), reading, source="guard")

    def _ventilation_stopped(self):
        """Guard thread: forced ventilation reads OFF. If the authorization
        depended on it, withdraw it, or retreat when the entry has started."""
        with self._lock:
            if not self._auth_vent_required or self._vent_lost_handled:
                return
            permit = self._permit
            if permit not in (ENTRY_AUTHORIZED, ENTRY_IN_PROGRESS):
                return
            self._vent_lost_handled = True
        reason = "forced ventilation stopped after the atmospheric test"
        if permit == ENTRY_AUTHORIZED:
            self._withdraw_authorization(reason, "guard")
            return
        message = "VENTILATION STOPPED during the entry: %s" % reason
        if self.auto_retreat and self._begin_retreat(message, None, "ventilation"):
            self._event("ventilation_lost", message + "; retreating", {"permit": permit}, "guard")
            threading.Thread(target=self._retreat_worker,
                             args=(message, None, "ventilation", "guard"),
                             name="retreat", daemon=True).start()

    def _on_alarm(self, reading, reasons):
        """SafetyGuard callback (guard thread): stop first, then log, then retreat."""
        t_detect = self._now()
        message = "GAS ALARM: " + "; ".join(reasons)
        cause = "telemetry" if reading is None else "atmosphere"
        started = self.auto_retreat and self._begin_retreat(message, reading, cause)
        try:
            self._record_alarm(message, reading, reasons, started, t_detect)
        finally:
            # A claimed retreat runs whatever happens to the records: a claim
            # without its thread would block every motion for good.
            if started:
                self._start_retreat_thread(message, reading, cause)

    def _record_alarm(self, message, reading, reasons, started, t_detect):
        """The alarm's audit records (the guard thread; keep it short)."""
        t_stop = self._now()
        pose = self._last_pose
        with self._lock:
            self._alarms.append({"reasons": list(reasons), "reading": reading, "pose": pose})
            incident_id = self._incident_serial + 1
        # Presenter-staged only with a disclosure AND injected pockets in the
        # simulator's own gas stream at this moment (T7, HUD contract).
        pockets = reading.get("injected_pockets") if isinstance(reading, dict) else None
        pockets = pockets if isinstance(pockets, int) and not isinstance(pockets, bool) else None
        staged = self._recent_presenter_injections() if pockets else []
        age = None
        age_fn = self._robot_fn("gas_age")
        if age_fn is not None:
            try:
                age = _num(age_fn())
            except Exception:
                age = None
        self._event("alarm", message, {
            "reasons": reasons, "reading": reading, "pose": pose,
            "auto_retreat": self.auto_retreat, "incident_id": incident_id,
            # Guard detection -> zero-twist command; plus how old the reading was.
            "stop_command_s": round(t_stop - t_detect, 4) if started else None,
            "reading_age_s": round(age, 3) if age is not None else None,
            "injected_pockets": pockets, "staged_by_presenter": staged}, source="guard")
        with self._lock:
            self._incident_serial = incident_id  # after the record exists: readers see both
        if not self.auto_retreat:
            with self._lock:
                self._alarms_ignored += 1
            self._event("alarm_ignored", "Auto-retreat DISABLED (rehearsal mode): continuing "
                        "despite the alarm", {"reasons": reasons}, source="guard")

    def _start_retreat_thread(self, message, reading, cause):
        threading.Thread(target=self._retreat_worker, args=(message, reading, cause, "guard"),
                         name="retreat", daemon=True).start()

    @staticmethod
    def _presenter_key(event):
        return (event.get("t"), str(event.get("kind")),
                json.dumps(event.get("data"), sort_keys=True, default=str))

    def _live_presenter_injections(self, max_age_s=300.0):
        """Presenter injection disclosures (host/inject_gas.sh) the robot
        link heard recently whose pockets have not been cleared since - by a
        presenter clear/reset (demo_clear, demo_reset) or by this mission's
        clear_injected_gas()/reset_atmosphere(). A cleared spike cannot have
        caused a later alarm, and the incident report must not say it did.
        [] when the link does not listen for them."""
        fn = self._robot_fn("recent_presenter_events")
        if fn is None:
            return []
        try:
            events = fn(max_age_s)
        except Exception:
            return []
        with self._lock:
            cleared = set(self._cleared_injections)
        live = []
        for event in events or []:
            event = filter_presenter_event(event)   # allow-listed kinds, typed data (T6)
            if event is None:
                continue
            if event["kind"] in ("demo_clear", "demo_reset"):
                live = []  # every pocket injected before it is gone
            elif event["kind"] == "demo_inject" and self._presenter_key(event) not in cleared:
                live.append(event)
        return live

    def _forget_presenter_injections(self):
        """After this mission cleared the injected pockets: remember which
        disclosures are now stale (bounded by the link's 20-event memory)."""
        fn = self._robot_fn("recent_presenter_events")
        try:
            events = fn() if fn is not None else []
        except Exception:
            events = []
        typed = [filter_presenter_event(e) for e in events or []]
        keys = {self._presenter_key(e) for e in typed if e and e["kind"] == "demo_inject"}
        with self._lock:
            self._cleared_injections = keys

    def _recent_presenter_injections(self, max_age_s=300.0):
        """Structured summary of ``_live_presenter_injections`` (last 3)."""
        out = []
        injections = self._live_presenter_injections(max_age_s)
        for event in injections[-3:]:
            # Structured fields only: this text reaches the LLM's context, and
            # anyone on the ROS network can publish a "presenter" event.
            data = event.get("data") if isinstance(event.get("data"), dict) else {}
            item = {"kind": "demo_inject", "source": "presenter"}
            gas = data.get("gas")
            if isinstance(gas, str) and gas.lower() in GAS_KEYS:
                item["gas"] = gas.lower()
            for key in ("ppm", "x", "y", "z", "radius"):
                value = _num(data.get(key))
                if value is not None:
                    item[key] = round(value, 3)
            out.append(item)
        return out

    @property
    def incident_serial(self):
        """Count of SafetyGuard alarms since this mission object was created."""
        with self._lock:
            return self._incident_serial

    def last_incident(self):
        """The latest SafetyGuard incident, rebuilt from the audit records:
        trigger reading and reasons, time, where Spot was, the guard's
        reaction, the retreat outcome, the permit outcome, and any staged
        (presenter) injection disclosed before it. {"found": False} if none."""
        with self._log_lock:
            records = list(self._records)
        alarms = [i for i, r in enumerate(records) if r["kind"] == "alarm"]
        if not alarms:
            return {"found": False,
                    "message": "No SafetyGuard alarm is recorded in this session's audit log."}
        i = alarms[-1]
        alarm = records[i]
        data = alarm.get("data") or {}
        after = records[i + 1:]
        reading = data.get("reading") if isinstance(data.get("reading"), dict) else None
        trigger = {"message": alarm["message"], "reasons": data.get("reasons") or []}
        if reading:
            trigger["reading"] = {GAS_KEYS[g]: round(v, 2) for g in OSHA_TEST_ORDER
                                  for v in [_num(reading.get(GAS_KEYS[g]))] if v is not None}
            trigger["gases_in_alarm"] = [GAS_LABELS[g] for g in OSHA_TEST_ORDER
                                         if not gas_passes(g, reading.get(GAS_KEYS[g]))]
            trigger["limits"] = {GAS_LABELS[g]: GAS_LIMITS[g] for g in OSHA_TEST_ORDER}
        else:
            trigger["gases_in_alarm"] = []
            trigger["telemetry_lost"] = True
        location = None
        pose = data.get("pose")
        if isinstance(pose, (list, tuple)) and len(pose) >= 2:
            name, dist = self._place_name(float(pose[0]), float(pose[1]))
            location = {"x": round(float(pose[0]), 2), "y": round(float(pose[1]), 2),
                        "inside_permit_space": self.in_space(float(pose[0]), float(pose[1])),
                        "nearest_place": name, "distance_to_place_m": round(dist, 2)}
        if data.get("auto_retreat") is False:
            retreat = {"status": "disabled (rehearsal mode: auto-retreat off)"}
        else:
            retreat = {"status": "not started"}
        permit_after = None
        for r in after:
            rdata = r.get("data") or {}
            if r["kind"] == "alarm":
                break
            if r["kind"] == "retreat_start":
                retreat = {"status": "in progress", "message": r["message"],
                           "legs": len(rdata.get("path") or [])}
            elif r["kind"] in ("retreat_complete", "retreat_incomplete"):
                retreat.update(status="complete" if r["kind"] == "retreat_complete" else "INCOMPLETE",
                               message=r["message"], duration_s=rdata.get("duration_s"),
                               final_pose=rdata.get("pose"))
            elif (r["kind"] == "permit" and rdata.get("state") == ABORTED and permit_after is None):
                permit_after = {"state": ABORTED, "reason": rdata.get("reason")}
        with self._lock:
            retreating = self._retreating
            state_now, id_now = self._permit, self.permit_id
        if retreating and retreat["status"] == "not started":
            retreat["status"] = "in progress"
        if permit_after is None and id_now == alarm.get("permit_id"):
            permit_after = {"state": state_now}
        staged = []
        pockets = data.get("injected_pockets")
        if isinstance(pockets, int) and pockets > 0:
            # Only while the simulator reported injected pockets (T7): a
            # disclosure alone does not make a genuine alarm "staged".
            staged = [{"kind": r["kind"], "message": r["message"], "data": r.get("data")}
                      for r in records[:i] if r["kind"] in ("demo_inject", "gas_injected")
                      and r.get("permit_id") == alarm.get("permit_id")][-1:]
            staged += data.get("staged_by_presenter") or []
        t = alarm.get("t")
        report = {
            "found": True,
            "incident_id": data.get("incident_id"),
            "kind": "gas telemetry lost" if reading is None else "gas alarm",
            "time": {"sim_time_s": t if isinstance(t, (int, float)) and t < 1.0e8 else None,
                     "wall_utc": alarm.get("wall")},
            "permit_id": alarm.get("permit_id"),
            "permit_state_at_alarm": alarm.get("permit"),
            "trigger": trigger,
            "location": location,
            "guard": {"auto_retreat": data.get("auto_retreat"),
                      "stop_command_s": data.get("stop_command_s"),
                      "reading_age_s": data.get("reading_age_s"),
                      "guard_poll_hz": round(1.0 / self.GUARD_PERIOD_S),
                      "gas_feed_hz": round(1.0 / self.ATMOS_SAMPLE_PERIOD_S)},
            "retreat": retreat,
            "permit_after": permit_after,
            "staged_injection": staged or None,
            "injected_pockets_at_alarm": pockets,
            "simulated": self.simulator,
            "next_steps": ("The permit is closed. Re-entry needs a new permit and a new "
                           "atmospheric test once the cause of the excursion is dealt with."),
        }
        report["spoken_summary"] = incident_sentence(report)
        return report

    def _begin_retreat(self, reason, reading, cause):
        """Claim the retreat and stop forward motion immediately. False if a
        retreat is already running. Deliberately does no logging/IO beyond
        the zero twist so the stop is as fast as possible."""
        with self._lock:
            if self._retreating:
                return False
            self._retreating = True
            self._motion_epoch += 1
            self._retreat_ok = None
            self._retreat_done.clear()
            self._abort_reason = reason
            self._abort_cause = cause
            self._trigger_reading = reading
        self._halt.set()
        if self._gate:
            try:
                self.robot.set_motion_owner("retreat")
            except Exception as error:
                LOG.warning("set_motion_owner failed: %s", error)
        self._send(0.0, 0.0, 0.0, "retreat")
        return True

    def retreat(self, reason="retreat requested", reading=None, cause="operator"):
        """Emergency exit: stop, reverse the visited path out through the door
        to entry_xy, sit, permit ABORTED. Blocks until done. If a retreat is
        already running (e.g. the guard's), waits for it. -> bool (True when
        Spot is back at entry_xy outside the permit space)."""
        self._ensure_guard()
        if not self._begin_retreat(reason, reading, cause):
            self._wait_retreat()
            return bool(self._retreat_ok)
        return self._retreat_worker(reason, reading, cause, "mission")

    def _retreat_worker(self, reason, reading, cause, source):
        ok = False
        attempt = 1
        try:
            ok = self._retreat_body(reason, reading, cause, source)
            while not ok and self._retreat_failed(attempt, reason, cause, source):
                attempt += 1
                ok = self._retreat_body(reason, reading, cause, source, attempt=attempt)
        except Exception as error:
            LOG.exception("retreat failed")
            self._event("error", "Retreat software fault: %r" % (error,), source=source)
            self._send(0.0, 0.0, 0.0, "retreat")
            self._call("stop")
        finally:
            self._end_retreat(ok)
        return ok

    def _retreat_failed(self, attempt, reason, cause, source):
        """After a retreat attempt that did not bring Spot out: tell the
        operator (console notice and a payload/events record), and for a
        guard retreat whose alarm persists pause at zero velocity and return
        True to try again, at most RETREAT_RETRIES times (T17)."""
        reading = self._gas_now()
        persists = reading is None or bool(alarm_reasons(reading))
        refused = backend_refusal(self.backend(), "retreating", retreat=True)
        retry = (source == "guard" and cause in ("atmosphere", "telemetry") and persists
                 and not refused and attempt <= self.RETREAT_RETRIES
                 and not self._cancel.is_set())
        pose = self._pose() or self._last_pose
        where = "(%.2f, %.2f)" % (pose[0], pose[1]) if pose else "an unknown position"
        if retry:
            text = ("RETREAT FAILED (attempt %d of %d): Spot is at %s and the %s persists; "
                    "retrying in %.0f s. Operator attention required."
                    % (attempt, self.RETREAT_RETRIES + 1, where,
                       "alarm" if reading is not None else "gas telemetry loss",
                       self.RETREAT_RETRY_PAUSE_S))
        else:
            why = ("the robot backend is not the simulator" if refused else
                   "the retreat was cancelled" if self._cancel.is_set() else
                   "this retreat is not retried automatically"
                   if source != "guard" or cause not in ("atmosphere", "telemetry") else
                   "the alarm has cleared, so it is not retried" if not persists else
                   "the alarm persists and no retry is left")
            text = ("RETREAT FAILED%s: Spot is at %s; %s. Operator attention required: "
                    "recover Spot by the site's rescue plan."
                    % (" after %d attempts" % attempt if attempt > 1 else "", where, why))
        self._event("retreat_retry" if retry else "retreat_failed", text,
                    {"attempt": attempt, "attempts_max": self.RETREAT_RETRIES + 1,
                     "retry": retry, "alarm_persists": persists, "pose": pose,
                     "reason": reason, "cause": cause}, source)
        self._notify(text)
        if not retry:
            return False
        end = self._now() + self.RETREAT_RETRY_PAUSE_S
        while self._now() < end and not self._cancel.is_set():
            self._send(0.0, 0.0, 0.0, "retreat")
            self._clock.sleep(0.1)
        return not self._cancel.is_set()

    def _notify(self, text):
        """A console notice for the operator (``notify_operator``), also
        logged at ERROR so it reaches the console without the hook."""
        LOG.error("OPERATOR ATTENTION: %s", text)
        fn = self.notify_operator
        if callable(fn):
            try:
                fn(text)
            except Exception:
                LOG.exception("notify_operator failed")

    def _retreat_body(self, reason, reading, cause, source, attempt=1):
        start = self._now()
        with self._lock:
            permit = self._permit
        if permit not in TERMINAL_STATES:
            self._set_permit(ABORTED, reason, {"cause": cause, "reading": reading}, source)
        self._call("stop")
        pose = self._pose() or self._last_pose
        # In a backend identity conflict the retreat still runs (T9): the
        # simulator was identified, and the way out must not be lost.
        refusal = self.backend_refusal("retreating (an autonomous walk on unsurveyed geometry)",
                                       source, retreat=True)
        if refusal:
            # Never identified as the simulator: nothing is sent to it, not
            # even a stop (T15), and it is never driven autonomously.
            self._event("retreat_incomplete", "Retreat NOT driven: %s Nothing was sent to the "
                        "robot, not even a stop - operator attention required" % refusal,
                        {"pose": pose, "duration_s": round(self._now() - start, 2),
                         "reason": reason, "cause": cause, "backend_refused": True}, source)
            return False
        not_standing = self._stand_for_retreat(source)
        if not_standing:
            self._send(0.0, 0.0, 0.0, "retreat")
            self._event("retreat_incomplete", "Retreat NOT driven: Spot is not standing and could "
                        "not stand up (%s); Spot stays where it is - operator attention required"
                        % not_standing,
                        {"pose": pose, "duration_s": round(self._now() - start, 2),
                         "reason": reason, "cause": cause, "not_standing": not_standing}, source)
            return False
        path = self._outward_path(pose)
        self._event("retreat_start",
                    "RETREAT: %s. Reversing the visited path out through the door "
                    "(%d legs)%s." % (reason, len(path), "" if attempt == 1 else
                                      ", attempt %d of %d" % (attempt, self.RETREAT_RETRIES + 1)),
                    {"reason": reason, "cause": cause, "reading": reading, "pose": pose,
                     "path": path, "attempt": attempt}, source)
        with self._lock:
            self._exiting = True  # a second trigger mid-retreat keeps heading out
        hold = self.drive_hold_reason()
        if hold:
            # Holds stop every other motion; the escape from the hazard goes on.
            self._event("retreat_hold", "Retreat continues despite a motion hold: %s" % hold,
                        {"hold": hold, "override": True}, source)
        deadline = start + self.RETREAT_TIMEOUT_S
        for index, point in enumerate(path):
            status = self._retreat_leg(index, point, deadline, source)
            if status != "reached":
                # Don't improvise a shortcut through unplanned geometry.
                break
            self._pop_trail_to(point)
        at_entry, samples, why = self._outside_confirmed(path)
        pose = tuple(samples[-1]["pose"]) if samples else (self._pose() or self._last_pose)
        near_entry = at_entry and self._near_entry(pose[0], pose[1])
        if at_entry:
            self._sit_if_outside()
        self._event("retreat_complete" if at_entry else "retreat_incomplete",
                    ("Retreat complete: Spot is outside the permit space%s (two fresh odometry "
                     "samples from the simulator agree)"
                     % (" at the entry point" if near_entry else ", clear of the doorway")
                     if at_entry else "Retreat INCOMPLETE: Spot stopped at %s (%s) - operator "
                     "attention required" % (list(pose) if pose else "unknown", why)),
                    {"pose": pose, "duration_s": round(self._now() - start, 2),
                     "reason": reason, "cause": cause,
                     "confirmed_by_seq": [sample["seq"] for sample in samples] or None,
                     "unconfirmed": None if at_entry else why}, source)
        return at_entry

    def _stand_for_retreat(self, source):
        """The retreat's first step: stand Spot up if payload/status says it
        is not standing (the facade ignores cmd_vel unless standing). Passes
        a backend identity conflict like the retreat's motion (T9). Returns
        why Spot cannot walk out, or None (standing, or no status: walk)."""
        status = self._status()
        if not isinstance(status, dict) or status.get("standing") is not False:
            return None      # a stand on a standing Spot would interrupt the walk
        if status.get("estopped"):
            return "the e-stop is asserted; a human must release it"
        ok, message = self._call("stand", retreat=True)
        self._event("robot", "Retreat: Spot is not standing, so it stands up first: %s - %s"
                    % ("ok" if ok else "FAILED", message),
                    {"service": "stand", "ok": ok, "message": message, "retreat": True}, source)
        if not ok:
            return message
        if not self._wait_standing():
            return "the stand did not complete"
        return None

    def _near_entry(self, x, y):
        return math.hypot(x - self.entry_xy[0], y - self.entry_xy[1]) <= self.RETREAT_ARRIVE_TOL_M

    def _retreat_end(self, x, y, path):
        """(x, y) is where a retreat along ``path`` may end: outside the
        permit space, at the entry point (or, with no path, clear of the
        doorway)."""
        return (not self.in_space(x, y) and
                (self._near_entry(x, y) or (not path and self._clear_of_space(x, y))))

    def _outside_confirmed(self, path):
        """(ok, samples, why): two odometry samples with different
        ``seq``, each fresh and, as it arrived, from the node that identified
        the simulator alone, both at the retreat's end (``_retreat_end``),
        within RETREAT_CONFIRM_S. A fresh facade sample anywhere else ends
        the wait: one forged or stale pose cannot declare a retreat complete
        (T22). No sample source on the robot link: not confirmed."""
        fn = self._robot_fn("odometry_sample")
        if fn is None:
            return False, [], "this robot link gives no odometry samples to confirm the position"
        end = self._now() + self.RETREAT_CONFIRM_S
        first, why = None, "no fresh odometry sample within %.0f s" % self.RETREAT_CONFIRM_S
        while not self._cancel.is_set():
            try:
                sample = fn()
            except Exception as error:
                LOG.warning("odometry_sample() raised %s", error)
                sample = None
            if isinstance(sample, dict):
                try:
                    x, y = float(sample["pose"][0]), float(sample["pose"][1])
                    seq = sample["seq"]
                except (KeyError, TypeError, ValueError, IndexError):
                    x = y = seq = None
                if seq is None or not (math.isfinite(x) and math.isfinite(y)):
                    first, why = None, "a malformed odometry sample"
                elif sample.get("facade") is not True:
                    first = None
                    why = ("odometry not from the simulator's node alone (%s)"
                           % (sample.get("why") or "publisher not checked"))
                elif not self._retreat_end(x, y, path):
                    return False, [sample], ("a fresh odometry sample puts Spot at (%.2f, %.2f), "
                                             "not outside at the end of the retreat" % (x, y))
                elif first is None:
                    first = sample
                elif seq != first["seq"]:
                    return True, [first, sample], None
            if self._now() >= end:
                if first is not None:
                    why = ("no second fresh odometry sample confirmed the first within %.0f s"
                           % self.RETREAT_CONFIRM_S)
                return False, [first] if first else [], why
            self._clock.sleep(0.02)
        return False, [], "cancelled"

    def _retreat_leg(self, index, point, deadline, source):
        """One retreat leg; on a pose gap (or a timed-out leg) hold zero
        velocity until the pose returns and retry, at most
        RETREAT_LEG_RETRIES times within ``deadline``."""
        attempt = 0
        while True:
            remaining = deadline - self._now()
            if remaining <= 0.0:
                return "timeout"
            status = self._navigate(point[0], point[1], self.WAYPOINT_TOL_M, self.CRUISE_SPEED,
                                    min(remaining, self._leg_timeout(point, self.REVERSE_MAX_SPEED)),
                                    owner="retreat",
                                    # First leg: back straight up the segment just driven.
                                    allow_reverse=True if index == 0 else None,
                                    label="retreat", source=source)
            if (status not in ("pose_lost", "timeout", "held") or attempt >= self.RETREAT_LEG_RETRIES
                    or self._cancel.is_set() or deadline - self._now() <= 0.0):
                return status
            attempt += 1
            self._event("retreat_hold", "Retreat leg %d to (%.2f, %.2f): %s; holding zero velocity "
                        "and resuming (retry %d/%d)" % (index + 1, point[0], point[1],
                                                        {"pose_lost": "odometry lost",
                                                         "held": "motion held"}.get(
                                                             status, "leg timed out"), attempt,
                                                        self.RETREAT_LEG_RETRIES),
                        {"leg": index, "target": list(point), "status": status, "retry": attempt,
                         "pose": self._last_pose}, source)
            self._hold_for_pose(min(self.RETREAT_POSE_WAIT_S, max(0.0, deadline - self._now())))

    def _hold_for_pose(self, max_s):
        """Zero velocity until a fresh pose arrives (True) or ``max_s`` pass."""
        end = self._now() + max_s
        while not self._cancel.is_set():
            self._send(0.0, 0.0, 0.0, "retreat")
            if self._pose() is not None:
                return True
            if self._now() >= end:
                return False
            self._clock.sleep(0.1)
        return False

    def _end_retreat(self, ok):
        with self._lock:
            self._retreating = False
            self._retreat_ok = bool(ok)
            self._exiting = False
            if ok:
                self._trail = []
        if self._gate:
            try:
                self.robot.set_motion_owner(None)
            except Exception as error:
                LOG.warning("set_motion_owner(None) failed: %s", error)
        self._halt.clear()
        if ok:
            # Outside again: a future excursion (new permit) must fire. After
            # a failed retreat the latch stays closed until the alarm clears,
            # so the guard doesn't restart a doomed retreat 20 times a second.
            self.guard.reset()
        self._retreat_done.set()

    def retreat_budget_s(self):
        """Longest a retreat may run, with its retries (T17)."""
        attempt = (self.RETREAT_TIMEOUT_S + self.STAND_TIMEOUT_S + self.RETREAT_CONFIRM_S
                   + self.RETREAT_RETRY_PAUSE_S)
        return (self.RETREAT_RETRIES + 1) * attempt

    def _wait_retreat(self, timeout=None):
        """Block until an in-progress retreat finishes (or cancel)."""
        limit = time.monotonic() + (self.retreat_budget_s() + 30.0 if timeout is None else timeout)
        while not self._retreat_done.wait(0.1):
            if self._cancel.is_set() or time.monotonic() > limit:
                return False
        return True

    def halt_motion(self, reason="operator stop", source="mission"):
        """The operator's stop (console /stop, Ctrl-C): every mission motion
        loop - walk, turn, atmospheric test, scripted run - ends at its next
        tick and Spot stays standing where it is, so it can still be walked
        out; nothing is cancelled for good. A running retreat (the guard's
        escape) is left alone."""
        with self._lock:
            self._motion_epoch += 1
            self._operator_halt_epoch = self._motion_epoch
            retreating = self._retreating
        if not retreating:
            self._send(0.0, 0.0, 0.0, "mission")
            self._call("stop")
        self._event("halt", "Operator stop (%s): %s" % (
            reason, "the running retreat continues" if retreating
            else "mission motion halted, Spot stopped where it is"),
            {"reason": reason, "retreat_running": retreating}, source)
        return not retreating

    def _halted_by_operator(self):
        with self._lock:
            halt, run = self._operator_halt_epoch, self._run_epoch
        return halt is not None and (run is None or halt > run)

    def exit_safely(self, reason="operator interrupt"):
        """For a process about to exit (Ctrl-C, SIGTERM): finish a running
        retreat, or start one if Spot is inside the permit space, so the
        SafetyGuard does not end with Spot still inside (T16); otherwise
        cancel. A second interrupt during the retreat (KeyboardInterrupt)
        is the caller's to turn into cancel(). Returns True when Spot is
        outside."""
        with self._lock:
            retreating = self._retreating
        if retreating or self.robot_inside():
            self.retreat(reason="process exiting: %s" % reason, cause="operator")
        else:
            self.cancel(reason)
        return not self.robot_inside()

    def cancel(self, reason="operator cancel"):
        """Stop everything now (Ctrl-C / SIGTERM path). Motion loops,
        including a retreat, exit; Spot is left stopped where it is."""
        self._cancel.set()
        self._halt.set()
        with self._lock:
            self._motion_epoch += 1
        self._send(0.0, 0.0, 0.0, "retreat")
        self._call("stop")
        with self._lock:
            permit = self._permit
            if self._abort_cause is None:
                self._abort_cause, self._abort_reason = "cancelled", reason
        if permit in (PRE_ENTRY_TESTING, ENTRY_AUTHORIZED, ENTRY_IN_PROGRESS):
            self._set_permit(ABORTED, reason)
        self._event("cancelled", "Mission cancelled: %s; robot stopped" % reason)

    def new_permit(self):
        """Start a fresh permit after COMPLETED/ABORTED (Spot must be outside)."""
        with self._lock:
            if self._permit not in TERMINAL_STATES + (NOT_STARTED,) or self._retreating:
                return False
        pose = self._pose() or self._last_pose
        if pose is not None and self.in_space(pose[0], pose[1]):
            return False
        self._cancel.clear()
        self._halt.clear()
        self._reset_permit_state()
        self.guard.reset()
        self._event("permit", "New permit %s issued (NOT_STARTED)%s" % (
                        self.permit_id, "; isolation, LOTO, attendant and rescue plan must be "
                        "recorded before Spot approaches" if self.require_permit else ""),
                    {"state": NOT_STARTED, "permit_id": self.permit_id,
                     "permit": self.entry_permit.as_dict()})
        return True

    # ------------------------------------------------------------------ #
    # Scripted run
    # ------------------------------------------------------------------ #

    def run(self, auto_retreat=True, on_waypoint=None, storyline=False, vent_tau_s=None,
            vent_max_s=None, max_retests=2, reset_atmosphere=False, confirm_entry=None):
        """Full scripted entry. Returns the summary dict (also the last
        audit-log record). ``auto_retreat=False`` is a REHEARSAL mode: the
        guard still logs every alarm but the route continues.

        ``storyline=True``: a failed pre-entry test is "entry denied" (the
        permit stays PRE_ENTRY_TESTING); SIMULATED forced ventilation starts
        (``vent_tau_s``), the purge is awaited (``vent_max_s``) and the test
        repeated up to ``max_retests`` times before entry. Ventilation stays
        on during the entry. ``reset_atmosphere=True`` first clears injected
        pockets and resets the simulated tank to its initial state (logged as
        a SIMULATION CONTROL event). Needs a complete permit when required.

        ``confirm_entry(test_result) -> (ok, record)`` is the operator's
        console confirmation, asked once the test has authorized entry and
        before Spot crosses the door; a refusal (or no answer) aborts the
        permit without entering (abort cause ``not_confirmed``).
        """
        refusal = self.backend_refusal("running the scripted mission")
        if refusal:
            return self._finish("FAILED", refusal)
        if self._log_failed:
            return self._finish("FAILED", "the audit log can no longer be written")
        with self._lock:
            terminal = self._permit in TERMINAL_STATES
        if terminal and not self.new_permit():
            return self._finish("FAILED", "previous permit closed and a new one could not be "
                                "issued (retreat running or Spot inside the space)")
        options = {"storyline": bool(storyline), "vent_tau_s": vent_tau_s,
                   "vent_max_s": vent_max_s, "max_retests": max(1, int(max_retests)),
                   "reset_atmosphere": bool(reset_atmosphere), "confirm_entry": confirm_entry}
        # Rehearsal mode (auto_retreat=False) is scoped to this run: the
        # guard's default for later step-by-step use stays as it was.
        previous_auto_retreat = self.auto_retreat
        self.auto_retreat = bool(auto_retreat)
        try:
            return self._run_guarded(on_waypoint, options)
        finally:
            self.auto_retreat = previous_auto_retreat

    def _run_guarded(self, on_waypoint, options=None):
        options = options or {"storyline": False}
        self._ensure_guard()
        self._t_start = self._now()
        self._run_options = options
        with self._lock:
            self._run_epoch = self._motion_epoch
        identity = operator_identity()
        self._event("mission_start",
                    "Confined-space entry mission starting (permit %s, auto-retreat %s%s)"
                    % (self.permit_id, "ON" if self.auto_retreat else "OFF - REHEARSAL",
                       ", storyline: test -> ventilate -> re-test" if options.get("storyline") else ""),
                    {"site": self.site, "auto_retreat": self.auto_retreat,
                     "dwell_s": self.dwell_s, "door_mouth_xy": self.door_mouth_xy,
                     "probe_xy": self.probe_xy, "probe_stations_xy": self.probe_stations_xy,
                     "purge_target_fraction": self.purge_target,
                     "options": {k: v for k, v in options.items() if k != "confirm_entry"},
                     "permit": self.entry_permit.as_dict(), "operator": identity,
                     "attribution": attribution(identity, self.unattributed_run),
                     "version": deployed_version(), "record_notice": RECORD_NOTICE})
        try:
            return self._run_sequence(on_waypoint, options)
        except KeyboardInterrupt:
            # Stop the run's motion but leave a retreat (or the caller's
            # retreat on the way out, T16) possible: no cancel() here.
            self.halt_motion("operator interrupt")
            raise
        except Exception as error:
            LOG.exception("mission fault")
            self._event("error", "Mission software fault: %r" % (error,),
                        {"traceback": traceback.format_exc(limit=6)})
            if self._retreating:
                self._wait_retreat()
            else:
                self.safe_stop()
                pose = self._pose() or self._last_pose
                if (self.auto_retreat and not self._cancel.is_set() and pose is not None
                        and self.in_space(pose[0], pose[1])):
                    try:
                        self.retreat(reason="software fault: %r" % (error,), cause="fault")
                    except Exception:
                        LOG.exception("fault retreat failed")
                        self.safe_stop()
            return self._finish("FAILED", "software fault: %r" % (error,))

    def _run_sequence(self, on_waypoint, options=None):
        options = options or {"storyline": False}
        missing = self.permit_missing()
        if missing:
            text = "permit %s incomplete (missing %s)" % (self.permit_id, describe_missing(missing))
            self._event("permit_incomplete", "Mission not started: %s. Record isolation, LOTO, "
                        "attendant and rescue plan first." % text,
                        {"missing": missing, "permit": self.entry_permit.as_dict()})
            return self._finish("FAILED", text)
        # An operator stop (halt_motion) ends the run at the next step boundary
        # too, not only inside a motion loop: nothing starts after it.
        if options.get("storyline") and options.get("reset_atmosphere"):
            self.reset_atmosphere()
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        if not self.prepare():
            if self._halted_by_operator():
                return self._finish("FAILED", self.STOPPED_TEXT)
            return self._finish("FAILED", "robot not ready (claim/power_on/stand failed)")
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        if not self.walk_to(*self.entry_xy):
            return self._after_motion_failure("could not reach the entry point")
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        if not self.face_door():
            return self._after_motion_failure("could not turn to face the door")
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        if options.get("storyline"):
            test = self._storyline_pre_entry(options)
        else:
            test = self.atmospheric_test(at_xy=self.door_mouth_xy, dwell_s=self.dwell_s)
        if not test["passed"]:
            if self._cancel.is_set():
                return self._finish("FAILED", "cancelled by operator")
            if self._halted_by_operator():
                return self._finish("FAILED", self.STOPPED_TEXT)
            reason = ("entry denied - pre-entry atmosphere unacceptable%s: " % (
                          " after forced ventilation and %d re-test(s)" % (len(self._tests) - 1)
                          if options.get("storyline") and len(self._tests) > 1 else "")
                      + "; ".join(test["failures"]))
            with self._lock:
                self._abort_cause, self._abort_reason = "entry_denied", reason
                self._trigger_reading = {"atmospheric_test": test["results"]}
            self._set_permit(ABORTED, reason, {"results": test["results"]})
            # Back off to the entry point (reverses: no turning at the door).
            self.walk_to(*self.entry_xy)
            self._sit_if_outside()
            return self._finish(ABORTED, reason)
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        confirm_entry = options.get("confirm_entry")
        if callable(confirm_entry):
            try:
                confirmed, record = confirm_entry(test)
            except Exception as error:
                LOG.exception("confirm_entry failed")
                confirmed, record = False, {"decision": "error: %r" % (error,)}
            record = record if isinstance(record, dict) else {}
            if self._halted_by_operator():
                # Stopped while the operator was asked: no back-off motion.
                return self._finish("FAILED", self.STOPPED_TEXT)
            if not confirmed:
                reason = ("entry not confirmed by the operator at the console (%s)"
                          % record.get("decision", "no answer"))
                with self._lock:
                    self._abort_cause, self._abort_reason = "not_confirmed", reason
                self._set_permit(ABORTED, reason, {"confirmation": record})
                self.walk_to(*self.entry_xy)
                self._sit_if_outside()
                return self._finish(ABORTED, reason)
        epoch = self._motion_epoch
        if not self.enter(on_waypoint=on_waypoint):
            return self._after_motion_failure("could not complete the interior route")
        self._event("survey", "Reached the far end of the permit space; surveying for %.0f s"
                    % self.SURVEY_DWELL_S, {"pose": self._last_pose})
        if not self._dwell(self.SURVEY_DWELL_S, epoch):
            return self._after_motion_failure("survey interrupted")
        if self._halted_by_operator():
            return self._finish("FAILED", self.STOPPED_TEXT)
        if not self.exit_space():
            return self._after_motion_failure("could not exit the permit space")
        return self._finish(COMPLETED, "entry completed; entrant exited; permit closed")

    def _storyline_pre_entry(self, options):
        """Storyline steps 2-4: test the unventilated tank (expected to
        FAIL -> entry denied, permit stays PRE_ENTRY_TESTING), start forced
        ventilation, wait for the purge, re-test (bounded). Returns the last
        test result."""
        test = self.atmospheric_test(at_xy=self.door_mouth_xy, dwell_s=self.dwell_s)
        if self._cancel.is_set() or self._halted_by_operator():
            return test
        with self._lock:
            state = self._permit
        if test["passed"]:
            self._event("storyline", "Initial test PASSED although the tank was expected to be "
                        "unventilated (simulator started ventilated?); forced ventilation still "
                        "runs during the entry", {"passed": True})
        elif state != PRE_ENTRY_TESTING:
            return test  # could not test at all (e.g. did not reach the test point)
        elif test.get("sampled_inside") is False and self.require_probe:
            # Ventilation cannot fix a test that never sampled the space.
            self._event("storyline", "Entry denied: the gas probe did not sample inside the "
                        "opening; not ventilating - the probe must work before any re-test",
                        {"passed": False, "failures": test["failures"]})
            return test
        else:
            self._event("storyline", "Entry denied on the unventilated tank: starting forced "
                        "ventilation, then re-testing", {"passed": False,
                                                         "failures": test["failures"]})
        vent = self.start_ventilation(options.get("vent_tau_s"))
        if test["passed"] or not vent["ok"]:
            return test
        for attempt in range(1, options["max_retests"] + 1):
            if attempt == 1:
                self.await_ventilation(options.get("vent_max_s"))
            elif test.get("purge_incomplete"):
                self._event("storyline", "Re-test %d read within limits but the purge target was "
                            "not reached; waiting for the purge before re-test %d"
                            % (attempt - 1, attempt), {"attempt": attempt})
                if not self.await_ventilation(options.get("vent_max_s"))["ok"] \
                        and self._cancel.is_set():
                    break
            else:
                extra = vent["tau_s"]
                self._event("storyline", "Re-test %d did not pass; ventilating %.0f s more before "
                            "re-test %d" % (attempt - 1, extra, attempt), {"attempt": attempt})
                if not self._dwell(extra):
                    break
            if self._cancel.is_set() or self._halted_by_operator():
                break
            test = self.atmospheric_test(at_xy=self.door_mouth_xy, dwell_s=self.dwell_s)
            with self._lock:
                state = self._permit
            if test["passed"] or state != PRE_ENTRY_TESTING:
                break
        return test

    def _after_motion_failure(self, what):
        if self._cancel.is_set():
            return self._finish("FAILED", "cancelled by operator")
        if self._halted_by_operator() and not self._retreating:
            # The operator stopped it: no automatic retreat, Spot stays put.
            return self._finish("FAILED", self.STOPPED_TEXT)
        with self._lock:
            retreat_started = self._retreating or self._abort_cause in (
                "atmosphere", "telemetry", "operator", "fault", "ventilation", "audit")
            withdrawn = self._abort_cause in ("conditions_changed", "entry_denied")
        if withdrawn and not retreat_started:
            # The door check withdrew the authorization: Spot never crossed.
            self.walk_to(*self.entry_xy)
            self._sit_if_outside()
            return self._finish(ABORTED, self._abort_reason or what)
        if retreat_started:
            self._wait_retreat()
            return self._finish(ABORTED, self._abort_reason or what)
        # Genuine navigation fault (timeout / lost odometry).
        self.safe_stop()
        pose = self._pose() or self._last_pose
        if pose is not None and self.in_space(pose[0], pose[1]) and self.auto_retreat:
            self.retreat(reason="navigation fault: %s" % what, cause="fault")
            return self._finish(ABORTED, "navigation fault: %s" % what)
        return self._finish("FAILED", what)

    def _vent_report(self):
        """Ventilation for the summary as the SIMULATOR reports it: the
        latest payload/gas block (so an excursion in a tank vented earlier,
        or started ventilated, does not read "off" just because this
        mission never commanded the blower). Falls back to the last block
        the guard saw, then to what this mission commanded."""
        status = self.ventilation_status()
        if status["source"] != "sim":
            with self._lock:
                seen = self._vent_seen
            if seen is not None:
                block, t_seen = seen
                frac = _num(block.get("fraction_remaining"))
                status = {"on": bool(block.get("on")), "tau_s": _num(block.get("tau_s")),
                          "fraction_remaining": frac,
                          "percent_remaining": round(100.0 * frac, 1) if frac is not None else None,
                          "residual_fraction": _num(block.get("residual_fraction")),
                          "purge_target_fraction": self.purge_target,
                          "purged": frac is not None and frac <= self.purge_target + 1e-9,
                          "source": "sim_last_seen", "age_s": round(self._now() - t_seen, 1),
                          "simulated": True}
            else:
                status = dict(status, source="commanded")
        with self._lock:
            status["commanded"] = dict(self._vent)
        return status

    def _finish(self, outcome, reason):
        pose = self._pose() or self._last_pose
        vent = self._vent_report()
        outside = pose is not None and not self.in_space(pose[0], pose[1])
        with self._lock:
            permit = self._permit
        if outcome != COMPLETED and permit not in TERMINAL_STATES + (NOT_STARTED,):
            self._set_permit(ABORTED, reason)
        with self._lock:
            cause = self._abort_cause
            retreat_ok = self._retreat_ok
            summary = {
                "record": "summary",
                "outcome": outcome,
                "reason": reason,
                "permit": self._permit,
                "permit_id": self.permit_id,
                "abort_cause": cause,
                "aborted_as_designed": bool(
                    outcome == ABORTED and outside and
                    (cause in ("entry_denied", "not_confirmed", "conditions_changed")
                     or (cause in ("atmosphere", "ventilation") and retreat_ok))),
                "auto_retreat": self.auto_retreat,
                "trigger_reading": self._trigger_reading,
                "retreat": {"triggered": retreat_ok is not None, "ok": retreat_ok},
                "atmospheric_test": self._last_test,
                "atmospheric_tests": list(self._tests),
                "storyline": bool(getattr(self, "_run_options", {}).get("storyline")),
                "permit_fields": self.entry_permit.as_dict(),
                "ventilation": vent,
                "ventilation_commanded": dict(self._vent),
                "waypoints_total": len(self.waypoints),
                "waypoints_reached": self._waypoints_reached,
                "alarms": len(self._alarms),
                "alarms_ignored": self._alarms_ignored,
                "gas_extremes_in_space": {k: {"min": v[0], "max": v[1]}
                                          for k, v in self._gas_extremes.items()},
                "final_pose": pose,
                "outside_permit_space": outside,
                "duration_s": (round(self._now() - self._t_start, 2)
                               if self._t_start is not None else None),
                "log_path": self.log_path,
            }
            self._outcome = outcome
        self._event("summary", "Mission %s: %s" % (outcome, reason), summary)
        return summary

    # ------------------------------------------------------------------ #
    # Introspection / teardown
    # ------------------------------------------------------------------ #

    def status_snapshot(self):
        """Thread-safe view for the agent/HUD."""
        pose = self._last_pose
        with self._lock:
            snapshot = {
                "permit": self._permit,
                "permit_id": self.permit_id,
                "outcome": self._outcome,
                "auto_retreat": self.auto_retreat,
                "retreating": self._retreating,
                "retreat_ok": self._retreat_ok,
                "abort_reason": self._abort_reason,
                "abort_cause": self._abort_cause,
                "trigger_reading": self._trigger_reading,
                "pose": list(pose) if pose else None,
                "inside_permit_space": bool(pose and self.in_space(pose[0], pose[1])),
                "trail": [list(p) for p in self._trail],
                "waypoints_total": len(self.waypoints),
                "waypoints_reached": self._waypoints_reached,
                "last_atmospheric_test": self._last_test,
                "alarms": len(self._alarms),
                "alarms_ignored": self._alarms_ignored,
                "entry_xy": list(self.entry_xy),
                "door_xy": list(self.door_xy),
                "door_mouth_xy": list(self.door_mouth_xy),
                "probe_xy": list(self.probe_xy),
                "probe_stations_xy": [list(p) for p in self.probe_stations_xy],
                "purge_target_fraction": self.purge_target,
                "require_permit": self.require_permit,
                "permit_fields": self.entry_permit.as_dict(),
                "ventilation_commanded": dict(self._vent),
                "atmospheric_tests": len(self._tests),
                "incident_serial": self._incident_serial,
            }
        snapshot["guard"] = self.guard.snapshot()
        return snapshot

    def close(self):
        """Stop the heartbeat and guard threads and close the audit log.
        Does not move Spot. The heartbeat stops first, so its last beat is
        not a "stalled" one for a guard stopped on purpose."""
        self._heartbeat_stop.set()
        thread = self._heartbeat_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self.guard.stop()
        head = None
        with self._log_lock:
            if self._log_file is not None:
                head = self._head_line_locked(AUDIT_HEAD_CLOSED)   # the end of the session (T38)
                try:
                    self._log_file.close()
                except OSError:
                    pass
                self._log_file = None
        if head is not None:
            self._print_audit_head(head)


# ---------------------------------------------------------------------------
# CLI: check a mission JSONL against the chain heads in the session log (T38)
# ---------------------------------------------------------------------------

def main(argv=None):
    """``python3 mission.py --verify LOG [--expect-head N:SHA256 ...]
    [--heads-from FILE ...]``: check LOG's record chain and every expected
    head; ``--heads-from`` reads the ``[audit head]`` lines that name LOG's
    file from a session transcript or a CloudWatch Logs export of it.
    Prints one line per finding and the file's own head. Exit codes: 0 no
    finding, 1 a finding, 2 the log or a head file cannot be read, a head
    is malformed, or no head was given or found."""
    parser = argparse.ArgumentParser(
        prog="mission.py",
        description="Check a mission JSONL audit log against the chain heads that the operator "
                    "console printed ([audit head] lines in the Session Manager session log).")
    parser.add_argument("--verify", required=True, metavar="LOG", help="the mission JSONL")
    parser.add_argument("--expect-head", action="append", default=[], metavar="N:SHA256",
                        help="a head from the session log: record count and SHA-256 of the "
                             "last record (repeatable)")
    parser.add_argument("--heads-from", action="append", default=[], metavar="FILE",
                        help="a session transcript or CloudWatch Logs export: every "
                             "[audit head] line naming LOG's file is checked (repeatable)")
    parser.add_argument("--chain-only", action="store_true",
                        help="check the chain alone (no head: truncation and a rewritten "
                             "chain then go unnoticed)")
    args = parser.parse_args(argv)
    heads = []
    try:
        for text in args.expect_head:
            count, sha256 = parse_head(text)
            heads.append("%d:%s" % (count, sha256))
        for name in args.heads_from:
            with open(name, "r", encoding="utf-8", errors="replace") as handle:
                found = audit_heads_in(handle.read(), args.verify)
            print("%s: %d audit head line(s) for %s" % (name, len(found), _head_name(args.verify)))
            heads.extend(h for h in found if h not in heads)
        if not heads and not args.chain_only:
            print("no head to check against: pass --expect-head or --heads-from (or "
                  "--chain-only, which cannot see truncation or a rewritten chain)")
            return 2
        result = verify_audit_log(args.verify, expect_heads=heads)
    except (OSError, ValueError) as error:
        print("cannot verify: %s" % (error,))
        return 2
    for finding in result["findings"]:
        print("%s: %s" % (finding["kind"].upper(), finding["message"]))
    print("%s: %d record(s), head %s; %d head(s) checked: %s"
          % (args.verify, result["records"], result["head"], len(heads),
             "OK" if result["ok"] else "%d finding(s)" % len(result["findings"])))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
