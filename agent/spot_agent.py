"""Natural-language operator agent for the simulated Spot confined-space demo.

What it is
----------
A Strands Agents SDK agent on Amazon Bedrock (default model: the one pinned
in ``DEFAULT_MODEL_ID``, region ``us-west-2``). Its tools are thin
wrappers over the two shared modules' public APIs (shared contracts 2 and 3):

  * ``RobotClient`` (agent/robot_client.py): claim/power/stand/sit/stop/estop
    triggers, pose(), gas(), status(), site(), send_velocity(), inject_gas(),
    publish_event().
  * ``ConfinedSpaceMission`` (agent/mission.py): walk_to(), atmospheric_test(),
    run(), retreat(), status_snapshot(). Its deterministic SafetyGuard retreats
    the robot on ANY gas alarm no matter who is driving, including this agent.

Beyond the contracts, a few members of the real modules are used when present
(``getattr``), each with a contract-only fallback: mission ``door_mouth_xy``
(test point), ``enter()`` (per-waypoint audit events), ``exit_space()`` (see
``_exit_space``), ``new_permit()``, ``cancel()``, ``close()``,
``prepare_permit()`` / ``permit_missing()`` / ``approach_refusal()`` (permit
fields gate), ``start_guard()`` (SafetyGuard running from startup),
``stand_up()`` (body-height check + self-right retry),
``start_ventilation()`` / ``ventilation_status()`` / ``await_ventilation()``
/ ``reset_atmosphere()`` / ``clear_injected_gas()`` (SIMULATED atmosphere),
``incident_serial`` / ``last_incident()`` (SafetyGuard incidents), and the
RobotClient ``motion_owner`` ("retreat" while a retreat owns cmd_vel).

Storyline (see agent/prompts.py): prepare_permit -> stand -> run the probe
test at the door (the unventilated tank FAILS: entry denied, the permit stays
in pre-entry testing) -> start_ventilation -> ventilation_status (can wait for
the purge) -> re-test (PASS) -> enter -> exit. When the SafetyGuard trips
during a tool call, that tool's result carries ``incident`` (trigger reading,
time, place, retreat outcome, staged injection) after waiting for the retreat
to finish, so the model narrates it; an incident that happens between turns
is prepended to the next instruction as an automatic safety notice.

Every tool call, every tool result, every operator instruction and every model
text reply is published to ``/demo_spot/payload/events`` with source="agent"
so the HUD can show the full transcript.

Robot-side text never reaches the model raw (T6): status, gas and probe data
are typed allow-lists (agent/robot_client.py), Trigger replies are fixed
phrases, and every string in a tool result, an event or a console line is
stripped of control, escape and invisible format characters (T10). Tool
results that carry readings also carry ``spoken_summary``, a sentence
generated in code from the values (gas, test verdict, incident) that the
model is told to say as written, and the console prints it too.

How it runs (ros:jazzy container, see agent/Dockerfile)
-------------------------------------------------------
    python3 spot_agent.py                      # interactive REPL
    python3 spot_agent.py --once "Run the full confined space entry"
    python3 spot_agent.py --dry-run [--once ...]
        # The REAL mission + SafetyGuard on the kinematic FakeRobot from
        # tests/test_mission.py and the sim's gas model (sim/gas_field.py, or
        # a look-alike): no ROS, no Isaac Sim, real time. It still calls
        # Bedrock (needs AWS credentials with Bedrock access). Rehearses the
        # storyline, including a presenter spike mid-entry.
    Options: --model <bedrock model or inference-profile id>  --region <aws region>
             --namespace /demo_spot  --log-path <mission JSONL audit log>
             --connect-timeout 30  --turn-timeout 900  --verbose  --unattributed-run

Attribution (T38): the agent does not start when the login that started the
session (SUDO_USER, passed by host/run_agent.sh as SPOT_OPERATOR_LOGIN) is a
shared account (ssm-user, ubuntu, root) or unknown, and says how to connect
as your own account; --unattributed-run starts anyway and the session
record says the run is unattributed. The audit log's chain head is printed
as an ``[audit head]`` line at session start, after every operator
confirmation, at mission start and end and at session end, so the Session
Manager session log keeps a copy off the host (agent/mission.py --verify).

Ctrl-C (or SIGTERM from ``docker stop``, or SIGHUP when the session drops)
cancels the model turn and halts the mission's motion; if Spot is inside the
permit space, or a retreat is running, the agent first brings Spot out (up
to EXIT_RETREAT_WAIT_S, with /stop, /estop and /status still read), then
sends a zero twist plus ``stop`` and exits without a traceback. A second
Ctrl-C cancels the retreat and leaves Spot where it is. ``quit`` is refused
while Spot is inside, and end of input keeps the process (and the
SafetyGuard) up until Spot is out, so quitting can never strand Spot
half-way out of the tank.

Safety design (why the tools look like this)
--------------------------------------------
The system prompt tells the model the rules, but the tools ENFORCE them, so a
confused or manipulated model still cannot break them:
  * Entry gate: nothing moves inward past the door unless a passing
    agent-run atmospheric test (mission verdict AND a clean live reading) is
    on record and nothing has invalidated it since: an alarm, a SafetyGuard
    trip, a later abort/completion, or AUTH_MAX_AGE_S before entry starts.
    The mission's own ENTRY_AUTHORIZED state alone is not enough, so the
    agent's veto holds; the mission's walk_to gate still applies underneath.
  * Named-place moves follow the pre-planned chain spawn -> entry -> door ->
    waypoint_1..N one segment at a time, because only those segments are
    verified collision-free (pipes at 0.28-0.52 m that Spot can neither step
    over nor pass under). Free walk_to(x, y) refuses targets in or through the
    tank.
    full_confined_space_entry and retreat_now refuse a start point from which
    the mission's straight leg to entry_xy would cross the tank.
  * While a gas alarm is active or a retreat runs, the SafetyGuard owns the
    robot: motion, stand and sit tools refuse, and halts (other than Ctrl-C
    and exit) leave the retreat alone rather than stall it. sit_down never
    sits the robot inside the tank (it could not be walked out).
  * Permit gate: moves toward the entry, the atmospheric test and entry are
    refused until the mission's permit records the permit id, isolation, LOTO,
    attendant and rescue plan (the mission's walk_to gate holds underneath).
  * emergency_stop requires confirm=True; inject_gas_demo_spike and
    clear_injected_gas require SPOT_DEMO_PRESENTER=1 in the environment.
  * Operator confirmation (agent/operator_console.py): no tool acts on a
    confirmation the model supplies. Isolation, LOTO and the attendant are
    attested by the operator typing y at the console; entering the permit
    space (enter_confined_space, full_confined_space_entry), an e-stop of
    either mode the model asks for inside the space (the gentle one also
    cuts motor power once Spot settles), self_right inside the space and
    e-stop release all wait (up to CONFIRM_TTL_S) for the operator's y/N,
    default deny, and the answer, time and operator go to the mission JSONL.
    Stopping never waits: the robot is halted before the e-stop question
    (either mode, inside the permit space) is asked.
  * Simulator only: the agent starts only once payload/status identifies
    the Isaac Sim backend ("simulated": true), and every tool that would
    command the robot refuses, with the reason, on any other backend.
    Until the simulator is identified nothing is sent at all, not even a
    stop, and the SafetyGuard has not started (T15: to a real driver a
    zero twist or a stop claims the lease and can power the robot on);
    once it is, stopping is never refused. No flag or environment variable
    overrides this (not SPOT_SKIP_SIM_CHECK, not SPOT_DEMO_PRESENTER). In a
    backend identity conflict (identified, then contradicted) the stops,
    retreat_now and /retreat still work; everything else refuses.
  * Any exception inside a tool sends a zero twist + stop before the error is
    returned to the model; the facade's 0.125 s cmd_vel watchdog is the final
    backstop if this process dies.
  * Tools run strictly one at a time (SequentialToolExecutor): Strands'
    default executor runs parallel tool calls concurrently, which for a robot
    would mean two motion loops fighting over cmd_vel.

Strands Agents API used here -- verified against the installed
strands-agents 1.57.1 source (pip install into a scratch venv, 2026-09-27;
github.com/strands-agents/sdk-python), not guessed:
  * ``strands.Agent(model=, tools=, system_prompt=, callback_handler=,
    tool_executor=, retry_strategy=)``                 strands/agent/agent.py
  * ``strands.models.BedrockModel(model_id=, region_name=, max_tokens=,
    boto_client_config=)``                             strands/models/bedrock.py
  * ``strands.tool(fn)`` builds the tool spec from the signature and the
    Google-style docstring (``Args:`` section); works on bound methods and on
    functools.wraps wrappers (inspect.signature follows __wrapped__, drops self).
    Non-string return values are json.dumps'ed into the tool result.
                                                       strands/tools/decorator.py
  * ``callback_handler(**kwargs)`` receives ``data`` (text delta),
    ``reasoningText``, ``event`` (raw stream chunk; tool starts arrive as
    ``event.contentBlockStart.start.toolUse``), ``message`` (each complete
    message) and ``result``.                           strands/types/_events.py
  * ``agent(prompt, limits={"turns": N})`` -> AgentResult (``str()`` is the
    final text); ``agent.cancel()`` is thread-safe.   strands/agent/agent.py
  * ``strands.tools.executors.SequentialToolExecutor``, ``strands.ModelRetryStrategy``.
agent/requirements.lock now pins strands-agents 1.62.0; the session budget's
metrics (``event_loop_metrics.cycle_count``, ``accumulated_usage``) and
``limits`` keys (``turns``, ``output_tokens``, ``total_tokens``; any other key
is a TypeError) were re-read in that version's source (``SessionBudget``).
The default model is ``DEFAULT_MODEL_ID``, the one place it is set: the US
cross-Region inference profile of Claude Sonnet 5, verified ACTIVE in
us-west-2 via ``aws bedrock list-inference-profiles`` (2026-09-27). Claude
Sonnet 5 has no dated snapshot: its model id is complete as it stands, a
fixed model version rather than a "latest Sonnet" alias. Any other
``--model`` is logged loudly as untested (the prompt rules were measured on
the default only). Sonnet 5 rejects sampling parameters (temperature/top_p),
so none are set. An Amazon Bedrock guardrail (Strands ``BedrockModel``
guardrail_id / guardrail_version, trace enabled) is **on by default**: its id
and version come from ``--guardrail-id`` / ``--guardrail-version`` or from
``SPOT_GUARDRAIL_ID`` / ``SPOT_GUARDRAIL_VERSION``, which
``infra/bedrock-guardrail.yaml`` outputs, and the agent refuses to start with
neither unless the operator passes ``--no-guardrail``, which warns loudly and
is recorded in the session record. The guardrail is probabilistic: a second
layer behind the typed tool results, the facade's limits and the code-rendered
confirmations, not a replacement for them.

Nothing in this module imports rclpy or strands at import time: the unit tests
(tests/test_agent_tools.py) run on a Mac with neither installed.
"""

import argparse
import functools
import inspect
import json
import logging
import math
import os
import re
import signal
import sys
import threading
import time
from typing import Optional

log = logging.getLogger("spot_agent")

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(AGENT_DIR)
if AGENT_DIR not in sys.path:
    # Sibling modules (prompts, robot_client, mission) resolve both when run as
    # a script inside the container and when imported as agent.spot_agent.
    sys.path.insert(0, AGENT_DIR)

from mission import (ATTENDANT_ROLE, ENTRY_SUPERVISOR_ROLE, RECORD_NOTICE,  # noqa: E402
                     default_log_path, deployed_version, gas_sentence, incident_sentence,
                     verdict_sentence)
from operator_console import attribution, attribution_refusal, operator_identity  # noqa: E402
from operator_console import Console, LineReader, PendingActions  # noqa: E402
from operator_console import (CONFIRM_MAX_PENDING, CONFIRM_RETRY_S, DENIED,  # noqa: E402
                              NOT_ASKED, quote_agent_text)
from operator_console import (AGENT_PREFIX, ATTENTION_PREFIX, READINGS_PREFIX,  # noqa: E402
                              REASONING_PREFIX, TOOL_PREFIX, neutralise_console_markers)
from prompts import build_system_prompt  # noqa: E402
from robot_client import (BACKEND_CONFLICT, BACKEND_UNKNOWN, BackendIdentity,  # noqa: E402
                          backend_refusal,
                          clean_text, filter_status, trigger_text, without_operator_identity)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# The one pinned default model (T11): a fixed model version (Claude Sonnet 5
# has no dated snapshot; this id does not move to a newer model).
DEFAULT_MODEL_ID = "us.anthropic.claude-sonnet-5"
_MODEL_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9.:/_-]{2,200}$")   # profile id, model id or ARN
DEFAULT_REGION = "us-west-2"

# The Amazon Bedrock guardrail is on by default (T11, M11). A guardrail id is
# account-specific, so a sample cannot ship one: the id and version come from
# the flags or from these variables, which infra/bedrock-guardrail.yaml's
# outputs fill in, and the agent refuses to start with neither unless the
# operator asks for that with --no-guardrail.
GUARDRAIL_ID_ENV = "SPOT_GUARDRAIL_ID"
GUARDRAIL_VERSION_ENV = "SPOT_GUARDRAIL_VERSION"
_GUARDRAIL_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9:/_-]{2,200}$")       # guardrail id or ARN
_GUARDRAIL_VERSION = re.compile(r"^(DRAFT|[1-9][0-9]{0,7})$")
NO_GUARDRAIL_WARNING = (
    "NO BEDROCK GUARDRAIL (--no-guardrail): the model's input and output are not screened by "
    "Amazon Bedrock. The deterministic gates still hold (the facade's limits, the typed robot "
    "data and the operator's confirmations), but the probabilistic layer is off and this session "
    "records that it was. Deploy infra/bedrock-guardrail.yaml and set %s / %s to turn it on."
    % (GUARDRAIL_ID_ENV, GUARDRAIL_VERSION_ENV))
NO_GUARDRAIL_REFUSAL = (
    "A Bedrock guardrail is required and none is configured: pass --guardrail-id with "
    "--guardrail-version, or set %s and %s (infra/bedrock-guardrail.yaml creates the guardrail "
    "and outputs both), or run with --no-guardrail to accept the risk for this session."
    % (GUARDRAIL_ID_ENV, GUARDRAIL_VERSION_ENV))
DEFAULT_NAMESPACE = "/demo_spot"
PRESENTER_ENV = "SPOT_DEMO_PRESENTER"
EVENT_SOURCE = "agent"

OUTSIDE_SPEED = 0.5      # m/s; facade verified: 0.5 m/s cmd_vel for 6 s -> 2.89 m
INSIDE_SPEED = 0.35      # slower in the dark, over sludge, between pipe runs
WALK_TOL = 0.25          # m; mission.walk_to default arrival tolerance
SPOT_CLEARANCE = 0.6     # m; Spot is ~1.1 m long, so keep its center this far off walls
SPACE_MARGIN = 0.3       # m; "in the permit space" = within tank_radius + this
                         # (same rule as ConfinedSpaceMission.in_space; the door counts)
APPROACH_MARGIN = 1.5    # m; approaching = coming within tank_radius + this
                         # (same as ConfinedSpaceMission.APPROACH_MARGIN_M)
SITE_LIMIT = 18.0        # m from tank center; the ground slab is 40 x 40 m
AT_PLACE = 0.5           # m; closer than this counts as "at" a named place
ON_SEGMENT = 0.4         # m; lateral slack for "already on this route segment"
STAND_TIMEOUT_S = 8.0    # facade stand-up blend is ~1.6 s, then the policy settles
AUTH_MAX_AGE_S = 600.0   # a passing test older than this is repeated before entry starts
CANCEL_GRACE_S = 600.0   # after a turn timeout, max wait for the in-flight tool to return
# OSHA 1910.146 App. E limits, same as sim/gas_field.py: a second check on the
# values in case the facade's alarm flags are missing or renamed.
O2_LOW_PCT, O2_HIGH_PCT = 19.5, 23.5
LEL_LIMIT_PCT, H2S_LIMIT_PPM, CO_LIMIT_PPM = 10.0, 10.0, 35.0
INCIDENT_WAIT_S = 240.0  # a tool that tripped the guard waits this long for the retreat
VENT_WAIT_MAX_S = 180.0  # ventilation_status(wait_s) cap
VENT_TAU_RANGE_S = (2.0, 120.0)
O2_DEPLETION_RANGE = (-20.9, 0.0)   # facade: O2 spikes deplete only, [-20.9, 0)
MAX_TURNS = 30           # per-instruction model-turn cap: no runaway tool loops
# Per-session model budget (T39, --session-max-turns/--session-max-output-tokens/
# --session-max-input-tokens): about ten full instructions. Once reached,
# instructions are refused; the console commands and the SafetyGuard do not
# use the model and keep working. Input tokens dominate the cost of a long
# session (every turn re-sends the system prompt, the tool specs and up to
# 40 messages of history, Strands' default sliding window): 6 M is about 240
# turns at a full window of some 25,000 tokens, an estimate, not measured.
SESSION_MAX_TURNS = 300
SESSION_MAX_OUTPUT_TOKENS = 250000
SESSION_MAX_INPUT_TOKENS = 6000000
MAX_TOKENS = 8192        # per model response; replies are short, headroom for thinking
GAS_NAMES = ("h2s", "lel", "o2", "co")
# Tools that command the robot: refused unless the backend is the simulator.
# sit_down and retreat_now stop the robot first and then check (stopping is
# never refused once the simulator has been identified, T15); read-only
# tools, the permit record and the e-stop are not gated.
BACKEND_GATED_TOOLS = frozenset((
    "claim_and_power_on", "stand_up", "walk_to", "walk_to_named", "run_atmospheric_test",
    "start_ventilation", "enter_confined_space", "full_confined_space_entry",
    "inject_gas_demo_spike", "clear_injected_gas"))
NO_ROUTE = ("No safe straight path from here onto the planned route. Move the robot to "
            "open ground near the entry point first.")

# Model-visible tools, in a fixed order: a deterministic tool list keeps the
# model's view identical across runs (and is a prerequisite for prompt caching).
TOOL_NAMES = (
    "get_status",
    "get_gas_reading",
    "prepare_permit",
    "claim_and_power_on",
    "stand_up",
    "sit_down",
    "walk_to",
    "walk_to_named",
    "run_atmospheric_test",
    "start_ventilation",
    "ventilation_status",
    "enter_confined_space",
    "full_confined_space_entry",
    "retreat_now",
    "explain_last_incident",
    "emergency_stop",
    "inject_gas_demo_spike",
    "clear_injected_gas",
)

PERMIT_FIELD_TEXT = {"permit_id": "the permit number", "isolation_confirmed": "isolation",
                     "loto_confirmed": "lockout/tagout (LOTO)", "attendant": "the attendant",
                     "rescue_plan": "the rescue plan", "entry_supervisor": "the entry supervisor"}
ESTOP_MODES = ("hard", "gentle")
GAS_TEXT = {"o2": "O2", "lel": "LEL", "h2s": "H2S", "co": "CO"}

# Spoken-output post-filter (``sanitize_speech``). The model's replies are
# read aloud to executives and shown on the HUD. Two things never go out as
# spoken text: an absolute safety claim ("It is safe.", dry run inc2) --
# a test only shows readings within permissible entry limits at the points
# and time sampled -- and raw tool identifiers ("with ventilation_status",
# dry run ok1). The prompt forbids both; this filter is the backstop.
ENTRY_LIMITS_TEXT = "the readings are within permissible entry limits"
SPOKEN_TOOL_LABELS = {
    "get_status": "a status check",
    "get_gas_reading": "a gas reading",
    "prepare_permit": "the permit record",
    "claim_and_power_on": "the power-on step",
    "stand_up": "the stand-up command",
    "sit_down": "the sit-down command",
    "walk_to": "a short walk command",
    "walk_to_named": "a walk to a named place",
    "run_atmospheric_test": "the atmospheric test",
    "start_ventilation": "forced ventilation",
    "ventilation_status": "the ventilation status check",
    "enter_confined_space": "the entry sequence",
    "full_confined_space_entry": "the full entry sequence",
    "retreat_now": "an immediate retreat",
    "explain_last_incident": "the incident report",
    "emergency_stop": "the emergency stop",
    "inject_gas_demo_spike": "the presenter's simulated gas spike",
    "clear_injected_gas": "the simulated gas clear",
}
_SAFE_QUALIFIER = r"(?:now\s+|all\s+|completely\s+|perfectly\s+|totally\s+|fully\s+|entirely\s+)?"
# "I can't say it is safe" IS the regulatory hedge: rewriting its "it is
# safe" would invert it ("I can't say the readings are within permissible
# entry limits"). A claim right after a negated say/claim verb is left as
# said and flagged ("negated_safety_claim").
_NEGATED_CLAIM = re.compile(
    r"(?:\b(?:not|never|cannot)|n['\u2019]t|\b(?:no\s+one|nobody)\s+(?:can|could|would|will))\s+"
    r"(?:\w+\s+){0,2}?(?:say|claim|call|declare|confirm|guarantee|promise|state|tell\s+you|"
    r"assume|consider|describe|certify)(?:\s+[\w'\u2019]+){0,3}\s*$", re.I)
# The robot's own condition is not an atmosphere claim: "Spot is safe" is
# flagged, not turned into "Spot is within permissible entry limits".
_ROBOT_SUBJECT = re.compile(r"\b(?:spot|robot|dog)\s*$", re.I)


def _negated(prefix):
    return bool(_NEGATED_CLAIM.search(prefix))


def _negated_or_robot(prefix):
    return _negated(prefix) or bool(_ROBOT_SUBJECT.search(prefix))


_SPEECH_RULES = (
    # "It is safe." / "The tank is now safe to enter" / "Everything's safe"
    (re.compile(r"\b(?:it|this|that|everything|the\s+(?:tank|space|atmosphere|air|area|entry|"
                r"environment|conditions?))\s*(?:is|are|looks|seems|'s|\u2019s)\s+" + _SAFE_QUALIFIER
                + r"safe(?!\s+to\s+(?:say|assume))(?:\s+(?:to\s+(?:enter|go\s+in|proceed|breathe|"
                r"work(?:\s+in)?)|for\s+(?:entry|people|workers|personnel|the\s+robot)))?\b", re.I),
     ENTRY_LIMITS_TEXT, "absolute_safety_claim", _negated),
    (re.compile(r"\bsafe\s+(?:to\s+(?:enter|go\s+in)|for\s+entry)\b", re.I),
     "within permissible entry limits", "safe_to_enter", _negated),
    (re.compile(r"\b(?:completely|perfectly|totally|entirely|100\s*%)\s+safe\b", re.I),
     "within permissible entry limits", "absolute_safety_claim", _negated),
    # Any other predicative claim, whatever the subject: "Entry is now
    # safe." / "Conditions are safe, so ..." (not "is a safe place").
    (re.compile(r"\b(is|are|was|were|be|looks|seems|remains|stays|becomes)\s+((?:(?:now|still|"
                r"also|definitely|absolutely|fully)\s+)*)safe\b(?=\s*(?:[.,;:!?)]|$)|\s+(?:and|but|"
                r"so|now|again|here|there|inside|once|until|while|because)\b)", re.I),
     lambda m: "%s %swithin permissible entry limits" % (m.group(1), m.group(2)),
     "absolute_safety_claim", _negated_or_robot),
    (re.compile(r"\bsafe\s+(?:atmosphere|air|conditions|levels?)\b", re.I),
     "acceptable entry conditions", "safe_atmosphere", _negated),
    (re.compile(r"\ball[\s-]+clear\b", re.I), "within permissible entry limits", "all_clear",
     _negated),
)
# A tool name with its call syntax: "walk_to_named(place=entry)" must not
# leave "(place=entry)" behind (parameter names are not said aloud either).
_TOOL_WORD = re.compile(r"`?\b(%s)\b(?:\([^()\n]{0,120}\))?`?" % "|".join(
    sorted(SPOKEN_TOOL_LABELS, key=len, reverse=True)))
_BARE_SAFE = re.compile(r"\bsafe\b", re.I)
_SENTENCE_START = re.compile(r"(?:^|[.!?]\s+|\n\s*)$")

INCIDENT_NOTE = (
    "The SafetyGuard tripped (during or just before this action). Tell the operator now, plainly "
    "and briefly: "
    "which gas alarmed and its reading versus the limit, where the robot was, that the guard "
    "stopped the robot and walked it out (and whether the retreat completed), whether it was a "
    "presenter-staged simulated spike, and that the permit is aborted so re-entry needs a new "
    "permit and a new atmospheric test. Use only the values in 'incident'.")
INCIDENT_PREAMBLE = (
    "[Automatic safety notice from the robot system - not typed by the operator] While you were "
    "waiting for the operator, the SafetyGuard tripped. Say this first, as written: %s\n"
    "Incident details from the audit log (typed values only): %s\nThen respond to the "
    "operator's instruction below.")


# ---------------------------------------------------------------------------
# Small pure helpers
# ---------------------------------------------------------------------------

def _json_default(obj):
    """json.dumps fallback: numpy scalars/arrays -> Python, anything else -> str."""
    for attr in ("item", "tolist"):
        fn = getattr(obj, attr, None)
        if callable(fn):
            try:
                return fn()
            except Exception:  # pragma: no cover - exotic objects
                pass
    return str(obj)


def _clean_strings(value):
    if isinstance(value, str):
        return clean_text(value, None, newlines=True)
    if isinstance(value, dict):
        return {clean_text(k, 200): _clean_strings(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_clean_strings(v) for v in value]
    return value


def _jsonable(value):
    """Round-trip through JSON so tool results and event payloads never carry
    numpy scalars, tuples or objects that would break json.dumps downstream
    (in RobotClient.publish_event or in the Strands tool-result encoder);
    every string loses its control and escape characters (T10)."""
    return _clean_strings(json.loads(json.dumps(value, default=_json_default)))


def _xy(value):
    """(x, y) floats from a 2+ element sequence, or None."""
    if value is None:
        return None
    try:
        return (float(value[0]), float(value[1]))
    except (TypeError, ValueError, IndexError, KeyError):
        return None


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point_segment_distance(p, a, b):
    """Shortest distance from point p to segment a-b (2-D)."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        return _dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2))
    return _dist(p, (ax + t * dx, ay + t * dy))


def _limit_breaches(gas):
    """Gases whose value in a payload/gas dict is outside the OSHA limits."""
    def value(key):
        v = gas.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            return None
        return float(v)

    o2, lel, h2s, co = (value(k) for k in ("o2_pct", "lel_pct", "h2s_ppm", "co_ppm"))
    out = []
    if o2 is not None and not O2_LOW_PCT <= o2 <= O2_HIGH_PCT:
        out.append("o2")
    if lel is not None and lel >= LEL_LIMIT_PCT:
        out.append("lel")
    if h2s is not None and h2s >= H2S_LIMIT_PPM:
        out.append("h2s")
    if co is not None and co >= CO_LIMIT_PPM:
        out.append("co")
    return out


def _alarming_gases(gas):
    """Gases in alarm by the facade's flags OR by their values."""
    alarms = gas.get("alarms") if isinstance(gas.get("alarms"), dict) else {}
    flagged = {k for k in GAS_NAMES if alarms.get(k) is True}   # known gas names only (T6)
    return sorted(flagged | set(_limit_breaches(gas)))


def _gas_alarm(gas):
    """True if a payload/gas dict reports any OSHA alarm: the facade's flags,
    or the values themselves (a missing or renamed flag must not hide a breach)."""
    if not isinstance(gas, dict):
        return False
    alarms = gas.get("alarms") if isinstance(gas.get("alarms"), dict) else {}
    return bool(gas.get("alarm") is True or alarms.get("any") is True or _alarming_gases(gas))


def _gas_summary(gas):
    """Compact, rounded view of a payload/gas dict for the model and the HUD."""
    out = {}
    for key in ("o2_pct", "lel_pct", "h2s_ppm", "co_ppm"):
        if isinstance(gas.get(key), (int, float)):
            out[key] = round(float(gas[key]), 2)
    out["alarming_gases"] = _alarming_gases(gas)
    out["alarm"] = _gas_alarm(gas)
    return out


def _permit_state(snapshot):
    """Best-effort permit state (upper-case str) from mission.status_snapshot().

    INTEGRATION ASSUMPTION: contract 3 names the permit states but not the
    snapshot key, so accept the likely spellings ("permit_state", "permit",
    "state", or a nested {"permit": {"state": ...}}).
    """
    if not isinstance(snapshot, dict):
        return None
    for key in ("permit_state", "permit", "state"):
        value = snapshot.get(key)
        if isinstance(value, dict):
            value = value.get("state")
        if isinstance(value, str) and value:
            return value.upper()
    return None


def _test_passed(result):
    """True only if an atmospheric_test() result explicitly says it passed.

    INTEGRATION ASSUMPTION: contract 3 says atmospheric_test returns a dict but
    not its keys. An unrecognized shape counts as FAILED (fail safe): step-wise
    entry is then blocked, while full_confined_space_entry (mission.run) keeps
    working because the mission gates itself.
    """
    if not isinstance(result, dict):
        return False
    for key in ("passed", "pass", "all_pass", "all_passed", "ok"):
        if isinstance(result.get(key), bool):
            return result[key]
    for key in ("result", "verdict", "outcome"):
        if isinstance(result.get(key), str):
            return result[key].strip().upper() in ("PASS", "PASSED")
    return False


def _accepts_kwarg(fn, name):
    """Whether callable ``fn`` accepts keyword ``name`` (or **kwargs)."""
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return True  # builtins/mocks: assume yes, _emit still guards the call
    return name in params or any(p.kind == p.VAR_KEYWORD for p in params.values())


def _describe_fields(fields):
    names = [PERMIT_FIELD_TEXT.get(f, f) for f in fields]
    if len(names) <= 1:
        return "".join(names)
    return ", ".join(names[:-1]) + " and " + names[-1]


def _osha_readings(test):
    """Compact per-gas view of a mission atmospheric_test() result, in
    OSHA order, for the model to read out."""
    out = []
    for entry in (test or {}).get("results") or []:
        if not isinstance(entry, dict):
            continue
        out.append({"gas": GAS_TEXT.get(entry.get("gas"), entry.get("gas")),
                    "worst": entry.get("value"), "unit": entry.get("unit"),
                    "limit": entry.get("limit"), "passed": entry.get("passed"),
                    "worst_at_height_m": entry.get("worst_z_m"),
                    "worst_inside_opening_m": entry.get("worst_inside_m"),
                    "by_height_m": entry.get("by_height")})
    return out


def sanitize_speech(text):
    """Post-filter one spoken reply: (text, flags).

    Rewrites absolute safety claims into regulatory language ("It is safe."
    -> "The readings are within permissible entry limits.") and raw tool
    identifiers into plain words ("ventilation_status" -> "the ventilation
    status check"). ``flags`` lists every rule that fired ({"rule",
    "match"}); a remaining bare "safe" (e.g. "not intrinsically safe
    certified") is flagged but left as said, and so is a claim the speaker
    negates ("I can't say it is safe": rewriting it would invert it). Pure and deterministic, so the
    console, the HUD transcript and the tests all see the same result."""
    if not isinstance(text, str) or not text:
        return text, []
    flags = []

    def rewrite(pattern, replacement, rule, source, keep=None):
        def sub(match):
            prefix = source[:match.start()]
            if keep is not None and keep(prefix):
                flags.append({"rule": "negated_safety_claim" if _negated(prefix)
                              else "robot_safe_claim", "match": match.group(0)})
                return match.group(0)
            flags.append({"rule": rule, "match": match.group(0)})
            out = replacement(match) if callable(replacement) else replacement
            if _SENTENCE_START.search(prefix):
                out = out[:1].upper() + out[1:]
            return out
        return pattern.sub(sub, source)

    for pattern, replacement, rule, keep in _SPEECH_RULES:
        text = rewrite(pattern, replacement, rule, text, keep)
    text = rewrite(_TOOL_WORD, lambda m: SPOKEN_TOOL_LABELS[m.group(1)], "tool_name", text)
    for match in _BARE_SAFE.finditer(text):
        flags.append({"rule": "safe_word", "match": match.group(0)})
    return text, flags


def _purge_hold_text(hold):
    """The mission's purge_hold numbers in the words the model should say,
    rounded exactly like the HUD's PURGE HOLD panel (0.1 %, target as set).

    The percentage is the mission's own ``percent_remaining`` (the number
    in its purge message and on the HUD), not a re-rounding of the 4-digit
    ``fraction_remaining``: 0.08251 is stored as 0.0825, which re-rounds to
    8.2 while the mission and the HUD say 8.3."""
    if not isinstance(hold, dict):
        return ""
    frac, target, eta = (hold.get(k) for k in ("fraction_remaining", "purge_target", "eta_s"))
    pct = hold.get("percent_remaining")
    if not isinstance(pct, (int, float)) or isinstance(pct, bool) or not math.isfinite(pct):
        pct = (100.0 * frac if isinstance(frac, (int, float)) and not isinstance(frac, bool)
               else None)
    parts = []
    if pct is not None:
        parts.append("%.1f percent of the unventilated contamination remains" % pct)
    if isinstance(target, (int, float)) and not isinstance(target, bool):
        parts.append("entry needs %g percent or less" % round(100.0 * target, 1))
    if isinstance(eta, (int, float)) and not isinstance(eta, bool) and eta > 0:
        parts.append("about %.0f seconds more" % eta)
    if not parts:
        return ""
    return ("Purge hold, the values shown on the HUD: " + ", ".join(parts)
            + ". Quote these numbers, not a later reading.")


def _refuse(reason, **extra):
    """Uniform refusal result: the model gets the reason to relay in one sentence."""
    out = {"ok": False, "refused": True, "error": reason}
    out.update(extra)
    return out


def _decision_text(record):
    """A confirmation record's decision in words for the model: "denied",
    "expired", or, for a request that was not asked (OWASP ASI09), why."""
    if record.get("decision") == NOT_ASKED:
        return "not asked again: %s" % record.get("reason")
    return record.get("decision")


def _short(value, limit=160):
    text = json.dumps(value, default=_json_default, sort_keys=True)
    return text if len(text) <= limit else text[: limit - 3] + "..."


def agent_tool(motion=False):
    """Mark a SpotTools method as a model-visible tool.

    Every call is routed through SpotTools._invoke, which publishes the call
    and the result as HUD events and turns any exception into "zero twist +
    stop, then report the error". ``motion=True`` tools also take the motion
    lock so two movement commands can never drive the robot at once.
    functools.wraps keeps the signature and docstring that strands.tool reads.
    """
    def decorate(method):
        @functools.wraps(method)
        def wrapper(self, *args, **kwargs):
            return self._invoke(method, args, kwargs, motion)

        wrapper.is_agent_tool = True
        return wrapper

    return decorate


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

class SpotTools:
    """The agent's tool surface over a RobotClient-like and a mission-like object.

    Deliberately independent of Strands: tests call these methods directly with
    fakes, and ``tool_functions()`` hands the bound methods to ``strands.tool``.
    """

    def __init__(self, robot, mission, site, env=None, sleep=time.sleep, clock=time.monotonic,
                 pending=None):
        self.robot = robot
        self.mission = mission
        # Physical actions waiting for the operator's y/N at the console; the
        # model's requests are rate-limited (OWASP ASI09, see _confirm).
        self.pending = pending if pending is not None else PendingActions(
            retry_after_s=CONFIRM_RETRY_S, max_pending=CONFIRM_MAX_PENDING)
        self.site = dict(site or {})
        self._env = os.environ if env is None else env
        self._sleep = sleep
        self._clock = clock
        self._motion_lock = threading.Lock()
        self._auth = None  # set by a passing run_atmospheric_test
        self._backend_id = BackendIdentity()  # for missions without backend()
        self._publish_with_source = _accepts_kwarg(robot.publish_event, "source")
        self.places, self.first_inside = self._build_places()
        self._incident_reported = self._incident_serial()  # last incident the model was told about
        # The SafetyGuard watches from the moment the backend is identified
        # as the simulator (here, or in await_simulator), not from the
        # model's first movement tool: Spot may already be inside the tank
        # (e.g. this process restarted mid-entry). Not before (T15): on any
        # other backend its alarm would send a stop.
        self._guard_started = False
        self.start_guard()
        if hasattr(mission, "confirm_action"):
            # self_right inside the permit space asks the operator through us.
            mission.confirm_action = self._confirm
        if hasattr(mission, "notify_operator"):
            # A failed retreat is shown on the console at once (T17).
            mission.notify_operator = self.operator_notice
        self._last_test = None   # (clock, atmospheric_test result) of the last agent-run test
        self._console_retreat = None  # thread of a retreat started from the console
        self.console_note = None      # callable(text): code-generated sentences on the console
        self.console_alert = None     # callable(text): operator-attention notices on the console
        self.session_unusable = False  # a model turn never finished (see ask())

    # -- plumbing ----------------------------------------------------------

    def tool_functions(self):
        """Bound tool methods in TOOL_NAMES order (for strands.tool wrapping)."""
        return [getattr(self, name) for name in TOOL_NAMES]

    def _emit(self, kind, message, data=None):
        """Publish one HUD event; never lets a telemetry failure break a tool.

        Contract 2 lists publish_event(kind, message, data) while the agent
        spec says source="agent"; agent/robot_client.py accepts source=. For
        a RobotClient-like object without it, tag the source inside data.
        """
        message = clean_text(message, None, newlines=True)
        log.info("[%s] %s", kind, message)
        try:
            # The operator's login and SSM session id stay in the JSONL (T37).
            payload = without_operator_identity(_jsonable(data)) if data is not None else None
            if self._publish_with_source:
                self.robot.publish_event(kind, message, payload, source=EVENT_SOURCE)
            else:
                payload = dict(payload or {})
                payload.setdefault("source", EVENT_SOURCE)
                self.robot.publish_event(kind, message, payload)
        except Exception:
            log.warning("publish_event(%s) failed", kind, exc_info=True)

    def _audit(self, kind, message, data=None):
        """Add a record to the mission's JSONL audit log (when it has one)."""
        fn = self._mission_fn("log_event")
        if fn is None:
            return
        try:
            fn(kind, message, data, source=EVENT_SOURCE)
        except Exception:
            log.warning("mission.log_event(%s) failed", kind, exc_info=True)

    def _confirm(self, action, summary, details=None, limit=True):
        """(ok, record): register ``action`` as a pending physical action and
        wait (up to the pending registry's expiry) for the operator's y/N at
        the console. Default deny; the record (typed answer, times, operator)
        goes to the HUD and the mission JSONL. A rehearsal auto-confirmation
        is refused unless the backend is the simulator, and nothing is shown
        on the console then. With ``limit`` (every request the model's tools
        make; not the operator's own /release) a request over the pending
        registry's limits is not asked at all (decision "not_asked", with the
        reason): the same action within CONFIRM_RETRY_S of the operator's
        refusal or expiry, or a third question while CONFIRM_MAX_PENDING
        wait (confirmation fatigue, OWASP ASI09)."""
        throttled = self.pending.throttle(action) if limit else None
        if throttled:
            item = self.pending.decline(action, summary, details, throttled)
        elif self.pending.auto_confirm and not self.backend().get("simulator"):
            refused = "auto-confirm refused: the backend is not the simulator"
            item = self.pending.decline(action, summary, details, refused, status=DENIED,
                                        mode="console", answer=refused)
        else:
            item = self.pending.request(action, summary, details)
            data = {"action": action, "action_id": item.id, "summary": summary,
                    "expires_in_s": round(item.expires_at - item.requested, 1)}
            self._emit("confirmation_requested", "Waiting for the operator at the console: %s"
                       % summary, data)
            self._audit("confirmation_requested", "Operator confirmation requested: %s" % summary,
                        data)
            self.pending.wait(item)
        record = item.record()
        ok = record["decision"] == "confirmed"
        if record["decision"] == NOT_ASKED:
            text = "Not asking the operator to confirm %s: %s" % (action, record.get("reason"))
        else:
            text = "Operator %s %s (answer %r%s)" % (
                "CONFIRMED" if ok else "did not confirm", action, record.get("answer"),
                ", rehearsal auto-confirm" if record.get("mode") == "rehearsal_auto_confirm"
                else "")
        self._emit("operator_confirmation", text, record)
        self._audit("operator_confirmation", text, record)
        return ok, record

    def say(self, text):
        """Publish a model text reply for the HUD transcript (kind="say"),
        after the spoken-output post-filter (``sanitize_speech``). A rewrite
        or flag is logged and carried in ``data.speech_filter`` (with the
        original text, for the audit log). Returns the published text."""
        spoken, flags = sanitize_speech(text)
        if isinstance(spoken, str):
            spoken = clean_text(spoken, None, newlines=True)   # no escapes on the HUD (T10)
        data = None
        if flags:
            data = {"speech_filter": {"rewritten": spoken != text,
                                      "rules": sorted({f["rule"] for f in flags}),
                                      "matches": [f["match"] for f in flags][:20],
                                      "original": text if isinstance(text, str) else None}}
            # A rewrite is a model slip worth a WARNING; a flag alone (the
            # mandated "not intrinsically safe certified" disclaimer, "I
            # can't say it is safe") is routine and stays at INFO.
            log.log(logging.WARNING if spoken != text else logging.INFO,
                    "speech post-filter %s: %s", "rewrote" if spoken != text else "flagged",
                    ", ".join("%s %r" % (f["rule"], f["match"]) for f in flags))
        self._emit("say", spoken, data)
        return spoken

    def operator_notice(self, text):
        """Print a mission notice that needs the operator (a failed
        retreat) on the console, from any thread (T17)."""
        write = self.console_alert if callable(self.console_alert) else self.console_note
        if callable(write):
            try:
                write(clean_text(text, 500))
            except Exception:
                log.debug("console alert failed", exc_info=True)

    def operator(self, text):
        """Publish the human operator's instruction (kind="operator") and
        keep it in the mission JSONL: the record shows what the operator
        actually typed next to what the agent did."""
        text = clean_text(text, 2000)
        self._emit("operator", text)
        self._audit("operator", text)

    def start_guard(self):
        """Start the mission's SafetyGuard (and its heartbeat) once the
        backend has been identified as the simulator (T15); idempotent.
        True when it has been started."""
        if self._guard_started:
            return True
        if self._stop_refusal() is not None:
            return False
        start_guard = self._mission_fn("start_guard")
        if start_guard is not None:
            start_guard()
        self._guard_started = True
        return True

    def _stop_refusal(self, action="stopping the robot"):
        """Why not even a stop may be sent, or None: the backend has never
        been identified as the simulator (T15; on a real driver a zero twist
        or a stop claims the lease and can power the robot on). Once it has
        been, stopping is never refused, in a backend conflict too."""
        return backend_refusal(self.backend(), action, stop=True)

    def halt_motion(self, reason, force=False):
        """Zero twist, then stop. Each step is independent: a failing
        send_velocity must not prevent the stop trigger (and vice versa).
        True when the robot was stopped.

        Unless ``force`` (Ctrl-C, process exit), a running retreat is left
        alone: stopping it would only stall the SafetyGuard's escape. On a
        backend never identified as the simulator nothing is sent (T15)."""
        refusal = self._stop_refusal()
        if refusal:
            self._emit("halt", "Not stopping the robot (%s): %s" % (reason, refusal),
                       {"reason": reason, "skipped": "backend not identified"})
            return False
        if not force and self._retreat_in_progress():
            self._emit("halt", "Not stopping the robot (%s): a retreat has control of it." % reason,
                       {"reason": reason, "skipped": "retreat in progress"})
            return False
        try:
            self.robot.send_velocity(0.0, 0.0, 0.0)
        except Exception:
            log.exception("halt_motion: send_velocity failed")
        try:
            self.robot.stop()
        except Exception:
            log.exception("halt_motion: stop failed")
        self._emit("halt", "Robot stopped: %s" % reason, {"reason": reason})
        return True

    def backend(self):
        """Backend identity: the mission's (real mission) or one built here
        from the robot's payload/status."""
        fn = self._mission_fn("backend")
        if fn is not None:
            try:
                identity = fn()
                if isinstance(identity, dict):
                    return identity
            except Exception:
                log.warning("mission.backend() failed", exc_info=True)
        try:
            status = self.robot.status()
        except Exception:
            status = None
        return self._backend_id.observe(status)

    def _backend_refusal(self, action="commanding the robot", retreat=False):
        refusal = backend_refusal(self.backend(), action, retreat=retreat)
        return _refuse(refusal, backend=self.backend()) if refusal else None

    def _invoke(self, method, args, kwargs, motion):
        name = method.__name__
        try:
            bound = inspect.signature(method).bind(self, *args, **kwargs)
            bound.apply_defaults()
            call_args = {k: v for k, v in bound.arguments.items() if k != "self"}
        except TypeError:
            call_args = {"args": list(args), "kwargs": kwargs}
        call_args = _jsonable(call_args)
        self._emit("tool_call", "%s(%s)" % (name, ", ".join(
            "%s=%s" % (k, _short(v, 40)) for k, v in call_args.items())),
            {"tool": name, "args": call_args})

        started = self._clock()
        locked = False
        serial_before = self._incident_serial()
        try:
            if motion:
                locked = self._motion_lock.acquire(blocking=False)
            refusal = self._backend_refusal() if name in BACKEND_GATED_TOOLS else None
            if refusal:
                result = refusal
            elif motion and not locked:
                result = _refuse("Another movement command is still running. Wait for it, "
                                 "or use retreat_now or emergency_stop.")
            else:
                result = method(self, *args, **kwargs)
        except Exception as exc:
            log.exception("tool %s raised", name)
            stopped = self.halt_motion("%s failed with %s" % (name, type(exc).__name__))
            result = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc),
                      "robot_stopped": stopped}
        finally:
            if locked:
                self._motion_lock.release()

        if not isinstance(result, dict):
            result = {"ok": True, "result": result}
        if name != "explain_last_incident":
            self._attach_incident(result, serial_before)
        # The operator's login and SSM session id (a confirmation record,
        # the permit's attestation) stay in the JSONL: never sent to the
        # model, as never on payload/events (T37, T38).
        result = without_operator_identity(_jsonable(result))
        self._note_summaries(result)
        elapsed = round(self._clock() - started, 2)
        self._emit("tool_result", "%s -> %s (%.1f s)" % (
            name, "ok" if result.get("ok") else "failed: %s" % result.get("error", "?"), elapsed),
            {"tool": name, "ok": bool(result.get("ok")), "result": result, "duration_s": elapsed})
        return result

    def _note_summaries(self, result):
        """Print the code-generated sentences of a tool result on the console
        (the operator sees the true numbers whatever the model says)."""
        note = self.console_note
        if not callable(note):
            return
        texts = [result.get("spoken_summary")]
        incident = result.get("incident")
        if isinstance(incident, dict):
            texts.append(incident.get("spoken_summary"))
        for text in texts:
            if isinstance(text, str) and text:
                try:
                    note(text)
                except Exception:
                    log.debug("console note failed", exc_info=True)

    # -- mission extras, permit and incidents ------------------------------

    def _mission_fn(self, name):
        fn = getattr(self.mission, name, None)
        return fn if callable(fn) else None

    def _incident_serial(self):
        """The mission's SafetyGuard alarm counter, or None if it has none."""
        try:
            serial = getattr(self.mission, "incident_serial", None)
        except Exception:
            return None
        return serial if isinstance(serial, int) and not isinstance(serial, bool) else None

    def _wait_for_retreat(self, wait_s):
        deadline = self._clock() + wait_s
        while self._retreat_in_progress() and self._clock() < deadline:
            self._sleep(0.25)

    def _incident_report(self, wait_s=0.0):
        """mission.last_incident(), after waiting (bounded) for a running
        retreat so the report carries its outcome."""
        if wait_s > 0.0:
            self._wait_for_retreat(wait_s)
        fn = self._mission_fn("last_incident")
        if fn is not None:
            try:
                report = fn()
                if isinstance(report, dict):
                    if report.get("found") and not report.get("spoken_summary"):
                        report["spoken_summary"] = incident_sentence(report)
                    return report
            except Exception:
                log.warning("mission.last_incident() failed", exc_info=True)
        snap = self._snapshot()
        return {"found": bool(snap.get("abort_reason")), "abort_reason": snap.get("abort_reason"),
                "trigger_reading": snap.get("trigger_reading"), "retreat_ok": snap.get("retreat_ok"),
                "permit": _permit_state(snap)}

    def _attach_incident(self, result, serial_before):
        """Put any SafetyGuard incident the model has not been told about
        into ``result``. One that tripped during this tool is reported after
        its retreat has played out (bounded wait), so the outcome is in it."""
        serial_after = self._incident_serial()
        if serial_after is None:
            return
        if serial_after == serial_before and (result.get("guard_tripped") or result.get("gas_alarm")):
            # The guard halts motion first and logs the alarm a moment later
            # (its own thread), so the tool can return in between.
            self._wait_for_retreat(INCIDENT_WAIT_S)
            serial_after = self._incident_serial()
        if serial_after == self._incident_reported:
            return
        tripped_here = serial_after != serial_before
        result["incident"] = self._incident_report(wait_s=INCIDENT_WAIT_S if tripped_here else 0.0)
        result["incident_note"] = INCIDENT_NOTE
        self._incident_reported = serial_after

    def pending_incident_prompt(self):
        """Notice text for a SafetyGuard incident the model has not been told
        about (it happened between turns), or None. Marks it as told."""
        serial = self._incident_serial()
        if serial is None or serial == self._incident_reported:
            return None
        self._incident_reported = serial
        report = self._incident_report()
        if not report.get("found"):
            return None
        self._emit("incident_notice", "Surfacing SafetyGuard incident %s to the agent"
                   % report.get("incident_id"), {"incident_id": report.get("incident_id")})
        report = without_operator_identity(_jsonable(report))   # not to the model (T37, T38)
        return INCIDENT_PREAMBLE % (report.get("spoken_summary") or incident_sentence(report),
                                    json.dumps(report, sort_keys=True))

    def _permit_missing(self):
        """Missing permit fields ([] = complete), or None when the mission
        has no permit record (contract-only mission: no gate here)."""
        fn = self._mission_fn("permit_missing")
        if fn is None:
            return None
        try:
            return list(fn() or [])
        except Exception:
            log.warning("mission.permit_missing() failed", exc_info=True)
            return None

    def _permit_refusal(self):
        missing = self._permit_missing()
        if not missing:
            return None
        return _refuse("The entry permit is not complete: %s %s not recorded. Record the permit "
                       "number and the rescue plan with prepare_permit, and have the operator "
                       "attest isolation, LOTO and the attendant at the console (prepare_permit "
                       "with request_attestation); the robot does not approach the entry until "
                       "the permit is complete."
                       % (_describe_fields(missing), "is" if len(missing) == 1 else "are"),
                       missing=missing)

    # -- site geometry -----------------------------------------------------

    def _build_places(self):
        """Ordered named-place chain and the index of the first place that
        counts as entry (the doorway). Adjacent places are joined by the only
        segments known to be collision-free."""
        places = []
        for name, key in (("spawn", "spawn_xy"), ("entry", "entry_xy")):
            xy = _xy(self.site.get(key))
            if xy:
                places.append((name, xy))
        first_inside = len(places)
        door = _xy(self.site.get("door_xy") or self.site.get("doorway_xy"))
        if door:
            places.append(("door", door))
        for i, wp in enumerate(self.site.get("interior_waypoints") or [], start=1):
            xy = _xy(wp)
            if xy:
                places.append(("waypoint_%d" % i, xy))
        return places, first_inside

    def _place_index(self, name):
        for i, (place, _xy_) in enumerate(self.places):
            if place == name:
                return i
        return None

    def _tank(self):
        center = _xy(self.site.get("tank_center_xy")) or (0.0, 0.0)
        try:
            radius = float(self.site.get("tank_radius", 6.0))
        except (TypeError, ValueError):
            radius = 6.0
        return center, radius

    def _in_space(self, xy):
        """Inside the permit space (tank footprint plus margin; the doorway
        counts as inside, matching the mission)."""
        center, radius = self._tank()
        return _dist(xy, center) < radius + SPACE_MARGIN

    def _here(self):
        pose = self.robot.pose()
        if pose is None:
            return None
        return _xy(pose)

    def _nearest_place(self, here):
        return min(range(len(self.places)), key=lambda i: _dist(here, self.places[i][1]))

    def _start_index(self, here):
        """Place to head for first, or None if no straight, collision-free
        first leg exists from here.

        Inside the tank only the door and waypoints are candidates; outside,
        only spawn and entry (plus the door when the robot is right at it).
        A plain "nearest place" could be a waypoint on the far side of the
        tank wall.
        """
        n = len(self.places)
        has_door = self.first_inside < n and self.places[self.first_inside][0] == "door"
        if self._in_space(here):
            candidates = range(self.first_inside, n)
        elif has_door and _dist(here, self.places[self.first_inside][1]) <= AT_PLACE:
            return self.first_inside
        else:
            candidates = range(0, min(self.first_inside, n))
        if len(candidates) == 0:
            return None
        best = min(candidates, key=lambda i: _dist(here, self.places[i][1]))
        if not self._in_space(here) and self._leg_clips_tank(here, self.places[best][1]):
            return None  # e.g. on the far side of the tank: the first leg would clip it
        return best

    def _leg_clips_tank(self, a, b):
        """True if a straight, unplanned leg a -> b (longer than AT_PLACE)
        passes closer to the tank than Spot's clearance allows."""
        center, radius = self._tank()
        return (_dist(a, b) > AT_PLACE
                and _point_segment_distance(center, a, b) < radius + SPOT_CLEARANCE)

    def _route(self, target, here):
        """(start_index, [place indices to visit]) along the place chain, or
        (None, None) when there is no safe first leg from here.

        Starts from the best reachable place; skips it when the robot is
        already at it or already on the segment leading toward the target.
        """
        start = self._start_index(here)
        if start is None:
            return None, None
        if start == target:
            at = _dist(here, self.places[start][1]) <= AT_PLACE
            return start, ([] if at else [target])
        step = 1 if target > start else -1
        nxt = start + step
        first = start
        if _dist(here, self.places[start][1]) <= AT_PLACE or _point_segment_distance(
                here, self.places[start][1], self.places[nxt][1]) <= ON_SEGMENT:
            first = nxt
        return start, list(range(first, target + step, step))

    # -- live state --------------------------------------------------------

    def _standing(self):
        status = self.robot.status()
        return bool(status and status.get("standing"))

    def _snapshot(self):
        try:
            snapshot = self.mission.status_snapshot()
        except Exception:
            log.warning("mission.status_snapshot() failed", exc_info=True)
            return {}
        return snapshot if isinstance(snapshot, dict) else {}

    def _mission_state(self):
        return _permit_state(self._snapshot())

    def _retreat_in_progress(self):
        """True while the mission's retreat (SafetyGuard or retreat_now) owns
        the robot: RobotClient then drops everyone else's non-zero twists
        (motion owner "retreat") and the snapshot says "retreating". Motion
        tools refuse instead of silently doing nothing."""
        return (getattr(self.robot, "motion_owner", None) == "retreat"
                or bool(self._snapshot().get("retreating")))

    def _renew_permit_if_closed(self):
        """A COMPLETED or ABORTED permit can never authorize another entry
        (OSHA: a new entry needs a new permit). Ask the mission for a fresh
        one before re-testing. Returns a refusal reason, or None."""
        state = self._mission_state()
        renew = getattr(self.mission, "new_permit", None)
        if state in ("COMPLETED", "ABORTED") and callable(renew):
            if not renew():
                return ("The previous permit is %s and the mission would not issue a new one; "
                        "the robot must be outside and no retreat may be running." % state)
            self._auth = None
        return None

    def _revoke(self, reason, message=None):
        if self._auth is not None:
            self._auth = None
            self._emit("permit", message or "Entry authorization withdrawn: %s" % reason,
                       {"entry_authorized": False, "reason": reason})

    def _entry_authorized(self):
        """True only for a passing agent-run test that nothing has invalidated
        since: a SafetyGuard trip, a later abort or completion of the permit,
        or (before the entry has started) an age over AUTH_MAX_AGE_S.

        The mission's own ENTRY_AUTHORIZED state is deliberately NOT enough:
        the mission authorizes on its samples alone, and the agent's veto (a
        live alarm right after the test, a stale test, a guard trip) must
        still hold once that alarm has cleared.
        """
        if self._auth is None:
            return False
        state = self._mission_state()
        if state in ("ABORTED", "COMPLETED") and state != self._auth.get("permit_at_auth"):
            self._revoke("permit is now %s" % state)
            return False
        here = self._here()
        started = state == "ENTRY_IN_PROGRESS" or (here is not None and self._in_space(here))
        tested_at = self._auth.get("t")
        if not started and (tested_at is None or self._clock() - tested_at > AUTH_MAX_AGE_S):
            self._revoke("the atmospheric test is more than %.0f minutes old"
                         % (AUTH_MAX_AGE_S / 60.0))
            return False
        return True

    def _motion_precheck(self, inward):
        """Common refusals for agent-initiated movement; None means go.

        During a gas alarm the robot may still move AWAY from the tank while
        it is outside (the guard is not armed there and backing off is the
        safe direction); inward moves and any move from inside are refused,
        because inside the permit space the SafetyGuard owns the robot.
        """
        status = self.robot.status()
        if status is None:
            return _refuse("No current status telemetry from the robot, so it will not be moved.")
        if not status.get("standing"):
            return _refuse("The robot is not standing. Call stand_up first.")
        if self._retreat_in_progress():
            return _refuse("A retreat is in progress and has control of the robot. Wait for it "
                           "to finish.")
        hold = self._drive_hold()
        if hold:
            return _refuse("Not moving the robot: %s. Tell the operator." % hold, hold=hold)
        gas = self.robot.gas()
        if _gas_alarm(gas):
            here = self._here()
            if here is None or self._in_space(here):
                return _refuse("A gas alarm is active inside the permit space, so the SafetyGuard "
                               "has control of the robot. Use retreat_now if it is not already "
                               "retreating.", gas=_gas_summary(gas))
            if inward:
                self._revoke("gas alarm at the entry")
                return _refuse("A gas alarm is active, so entry is not allowed; a new "
                               "atmospheric test is needed once it clears.",
                               gas=_gas_summary(gas))
        if inward:
            if gas is None:
                return _refuse("No gas telemetry, so the atmosphere cannot be monitored. "
                               "Entry is not allowed.")
            if not self._entry_authorized():
                return _refuse("Entry is not authorized. Run the atmospheric test at the "
                               "entry point first; it must pass O2, LEL, H2S and CO, and it "
                               "must be repeated after an alarm, an abort, or %.0f minutes."
                               % (AUTH_MAX_AGE_S / 60.0))
        return None

    def _drive_hold(self):
        """The mission's drive-hold reason (another cmd_vel publisher, a
        degraded simulator), or None."""
        fn = self._mission_fn("drive_hold_reason")
        try:
            return fn() if fn is not None else None
        except Exception:
            log.warning("mission.drive_hold_reason() failed", exc_info=True)
            return None

    def _walk(self, index):
        """One pre-planned segment via the mission's closed-loop walk_to."""
        name, (x, y) = self.places[index]
        inside = index >= self.first_inside
        speed = INSIDE_SPEED if inside else OUTSIDE_SPEED
        here = self._here()
        dist = _dist(here, (x, y)) if here else 10.0
        timeout = max(30.0, 2.5 * dist / speed + 10.0)
        return bool(self.mission.walk_to(x, y, tol=WALK_TOL, max_speed=speed, timeout=timeout))

    def _walk_route(self, indices, inward, sample_gas=False):
        """Walk a list of place indices, re-checking gas and the permit before
        every segment. Returns a result dict."""
        visited, readings = [], []
        for index in indices:
            name = self.places[index][0]
            gas = self.robot.gas()
            hold = self._gas_hold(gas, index, inward)
            if hold:
                return {"ok": False, "gas_hold": True, "visited": visited, "readings": readings,
                        "error": hold, "permit": self._mission_state()}
            if inward and _gas_alarm(gas):
                self._revoke("gas alarm during the entry")
                return {"ok": False, "gas_alarm": True, "visited": visited, "readings": readings,
                        "error": "Gas alarm before reaching %s, so the entry stopped there." % name,
                        "gas": _gas_summary(gas), "permit": self._mission_state()}
            if inward and index >= self.first_inside and not self._entry_authorized():
                # The robot is parked at the previous place (that leg finished),
                # so there is nothing to halt; most likely the guard aborted.
                return {"ok": False, "visited": visited, "readings": readings,
                        "error": "Entry authorization lost before %s (permit %s)."
                                 % (name, self._mission_state())}
            permit_before = self._mission_state()
            if not self._walk(index):
                return self._leg_failure(name, permit_before, visited, readings,
                                         inward and index >= self.first_inside)
            visited.append(name)
            if sample_gas:
                gas = self.robot.gas()
                if gas:
                    readings.append(dict(place=name, **_gas_summary(gas)))
        return {"ok": True, "visited": visited, "readings": readings}

    def _gas_hold(self, gas, index, inward):
        """Why the next segment must not start, or None (T17): no current
        gas reading (``gas()`` None) while Spot is in the permit space (or
        its pose is unknown), the move is inward, or the segment approaches
        the tank. The robot is stopped between segments, so it holds there;
        retreat_now and the stops still work."""
        if gas is not None:
            return None
        here = self._here()
        target = self.places[index][1]
        center, radius = self._tank()
        closest = _point_segment_distance(center, here, target) if here is not None else None
        approaching = (closest is not None and closest < radius + APPROACH_MARGIN
                       and closest < _dist(here, center) - 1e-3)
        if not (inward or here is None or self._in_space(here) or approaching):
            return None
        text = ("No current gas reading before %s, so the robot holds where it is: it does not "
                "move in or toward the permit space without gas telemetry. Ask again once the "
                "gas feed is back; retreat_now and the stops still work." % self.places[index][0])
        data = {"place": self.places[index][0], "inward": bool(inward)}
        if self._mission_fn("log_event") is not None:
            self._audit("gas_hold", text, data)      # the JSONL, and payload/events with it
        else:
            self._emit("gas_hold", text, data)
        return text

    def _leg_failure(self, name, permit_before, visited, readings, inward=False):
        """Result for a mission move that returned False. If the SafetyGuard
        tripped (alarm, retreat running, or the permit just flipped to
        ABORTED) it owns the robot and a stop from here would only stall its
        retreat; otherwise (timeout, refusal) make sure nothing is creeping.

        The entry authorization is withdrawn after a trip, and after any
        failed inward move that leaves the robot outside (a guard that
        walked it back out cannot always be told apart from other failures,
        e.g. with a mission that was already ABORTED): re-entry needs a new
        test. A robot still inside keeps it, to finish or exit."""
        gas = self.robot.gas()
        permit_after = self._mission_state()
        snap = self._snapshot()
        if (snap.get("abort_cause") in ("conditions_changed", "entry_denied")
                and permit_after == "ABORTED" and not self._retreat_in_progress()):
            # The mission's door check withdrew the authorization (T19).
            self._revoke("the check at the door withdrew the entry authorization")
            return {"ok": False, "visited": visited, "readings": readings, "guard_tripped": False,
                    "entry_withdrawn": True,
                    "error": "Entry authorization withdrawn at the door before crossing: %s. A new "
                             "permit and a new atmospheric test are needed."
                             % snap.get("abort_reason"),
                    "permit": permit_after}
        tripped = (_gas_alarm(gas) or self._retreat_in_progress()
                   or (permit_after == "ABORTED" != permit_before))
        if not tripped:
            self.halt_motion("could not reach %s" % name)
        here = self._here()
        if tripped or (inward and (here is None or not self._in_space(here))):
            self._revoke("the entry was interrupted" if not tripped
                         else "the SafetyGuard aborted the entry")
        return {"ok": False, "visited": visited, "readings": readings,
                "guard_tripped": tripped,
                "error": ("The SafetyGuard aborted the move to %s and is retreating." % name
                          if tripped else "Could not reach %s (permit %s)." % (name, permit_after)),
                "gas": _gas_summary(gas) if gas else None,
                "permit": permit_after}

    # -- operator console (no model) ---------------------------------------

    def robot_inside(self):
        """Spot in the permit space by its (last known) pose, or in an entry."""
        if self._mission_state() == "ENTRY_IN_PROGRESS":
            return True
        here = self._here()
        if here is not None:
            return self._in_space(here)
        return bool(self._snapshot().get("inside_permit_space"))

    def console_stop(self, reason="operator console stop"):
        """Stop the robot without the model: pending confirmations are
        denied and mission motion halts with Spot standing where it is (so it
        can still be walked out); a running retreat continues. Returns the
        message for the console."""
        self.pending.deny_all(reason)
        fn = self._mission_fn("halt_motion")
        try:
            if fn is not None:
                fn(reason, source=EVENT_SOURCE)
            else:
                self.halt_motion(reason)
        except Exception:
            log.exception("console stop failed; sending a plain halt")
            self.halt_motion(reason)
        if self._retreat_in_progress():
            return "A retreat is running and continues (/estop cuts motor power)."
        return "Robot stopped: standing, motor power on."

    def retreating(self):
        """A retreat is running, or one started from the console has not ended."""
        thread = self._console_retreat
        return self._retreat_in_progress() or bool(thread is not None and thread.is_alive())

    def start_retreat(self, reason="operator console /retreat"):
        """Bring Spot out along its path in a background thread, without the
        model (the same checks as retreat_now). Returns the console message."""
        if self.retreating():
            return "A retreat is already running."
        if not self.robot_inside():
            return "Spot is not inside the permit space; there is nothing to retreat from."
        self._emit("console", "Operator console: retreat requested (%s)." % reason,
                   {"command": "retreat", "reason": reason})
        retreat = type(self).retreat_now.__wrapped__   # the tool's body, without the model

        def run():
            try:
                result = retreat(self)
                self._emit("console", "Console retreat finished: %s"
                           % ("ok" if result.get("ok") else result.get("error")),
                           {"command": "retreat", "result": result})
            except Exception:
                log.exception("console retreat failed")
                self.halt_motion("console retreat failed")

        self._console_retreat = threading.Thread(target=run, name="console-retreat", daemon=True)
        self._console_retreat.start()
        return "Retreat started: Spot walks out along its path."

    def console_estop(self, hard=False):
        """Operator-commanded e-stop from the console: immediate, never
        waits for a confirmation (gentle by default). Nothing is sent to a
        backend never identified as the simulator (T15)."""
        refusal = self._stop_refusal("e-stopping the robot")
        if refusal:
            return "E-stop not sent: %s" % refusal
        fn = getattr(self.robot, "estop_hard" if hard else "estop_gentle", None)
        ok, msg = fn() if callable(fn) else (False, "not available on this robot link")
        try:
            self.robot.send_velocity(0.0, 0.0, 0.0)
        except Exception:
            log.exception("console e-stop: zero twist failed")
        self._revoke("operator e-stop at the console")
        self._emit("console", "Operator console: %s e-stop - %s" % (
            "hard" if hard else "gentle", "ok" if ok else "FAILED"), {"command": "estop",
                                                                     "hard": hard, "ok": bool(ok)})
        return "%s e-stop %s." % ("Hard" if hard else "Gentle", "asserted" if ok else
                                  "FAILED: %s" % trigger_text("estop", ok, msg))

    def status_text(self):
        """One code-generated status block for the console (/status)."""
        snap = self._snapshot()
        identity = self.backend()
        pose = self._here()
        if identity.get("simulator"):
            backend = "Isaac Sim simulator"
        elif identity.get("backend") == BACKEND_CONFLICT:
            backend = ("IDENTITY CONFLICT (%s): only stopping and /retreat work"
                       % identity.get("reason"))
        else:
            backend = "NOT the simulator (%s)" % identity.get("reason")
        lines = ["Backend: %s." % backend]
        lines.append("Permit %s: %s." % (snap.get("permit_id") or "?",
                                         self._mission_state() or "unknown"))
        if pose is not None:
            name = self.places[self._nearest_place(pose)][0] if self.places else "?"
            lines.append("Spot at (%.2f, %.2f), %s, nearest %s." % (
                pose[0], pose[1], "INSIDE the permit space" if self._in_space(pose)
                else "outside the permit space", name))
        else:
            lines.append("Spot position unknown (no odometry).")
        gas = self.robot.gas()
        if gas:
            summary = _gas_summary(gas)
            lines.append("Gas at the robot: O2 %s %%, LEL %s %%, H2S %s ppm, CO %s ppm; %s." % (
                summary.get("o2_pct", "?"), summary.get("lel_pct", "?"),
                summary.get("h2s_ppm", "?"), summary.get("co_ppm", "?"),
                "ALARM: %s" % ", ".join(g.upper() for g in summary["alarming_gases"])
                if summary["alarm"] else "no gas alarm"))
        else:
            lines.append("Gas: no current reading.")
        guard = snap.get("guard") or {}
        lines.append("SafetyGuard: %s, %s, %s alarm(s) so far; retreat %s." % (
            "running" if guard.get("running") else "NOT RUNNING",
            "armed" if guard.get("armed") else "not armed", guard.get("fires", "?"),
            "RUNNING" if self.retreating() else "not running"))
        status = self.robot.status() or {}
        if isinstance(status.get("battery_pct"), (int, float)):
            lines.append("Battery %.0f %%%s." % (status["battery_pct"],
                                                 ", E-STOPPED" if status.get("estopped") else ""))
        hold = self._drive_hold()
        if hold:
            lines.append("MOTION HELD: %s." % hold)
        lines.append("Pending confirmations: %d. Model session: %s." % (
            len(self.pending.pending()), "UNUSABLE (restart the agent)" if self.session_unusable
            else "ok"))
        return "\n".join(lines)

    # -- read-only tools ---------------------------------------------------

    @agent_tool()
    def get_status(self) -> dict:
        """Get the robot's state, position, nearest named place and permit state.

        Use this before acting when you are unsure whether the robot is powered,
        standing, e-stopped, inside the tank, or authorized to enter.
        """
        status = filter_status(self.robot.status())   # allow-listed, typed fields (T6)
        pose = self.robot.pose()
        here = _xy(pose)
        out = {"ok": status is not None, "robot": status}
        if status is None:
            out["error"] = "No status telemetry from the robot yet."
        if pose is not None:
            out["pose"] = {"x": round(float(pose[0]), 2), "y": round(float(pose[1]), 2),
                           "yaw_deg": round(math.degrees(float(pose[2])), 1)}
        if here is not None and self.places:
            i = self._nearest_place(here)
            out["nearest_place"] = self.places[i][0]
            out["distance_to_nearest_place_m"] = round(_dist(here, self.places[i][1]), 2)
            out["inside_tank"] = self._in_space(here)
        try:
            out["mission"] = self.mission.status_snapshot()
        except Exception as exc:
            out["mission"] = {"error": "%s: %s" % (type(exc).__name__, exc)}
        out["entry_authorized"] = self._entry_authorized()
        return out

    @agent_tool()
    def get_gas_reading(self) -> dict:
        """Get the latest four-gas reading at the robot's body: oxygen percent,
        LEL percent, hydrogen sulfide ppm, carbon monoxide ppm, and which gases
        are in alarm by OSHA 1910.146 Appendix E limits.
        """
        gas = self.robot.gas()
        if gas is None:
            return {"ok": False, "error": "No gas telemetry received yet."}
        out = {"ok": True}
        out.update(_gas_summary(gas))
        here = self._here()
        if here is not None:
            out["at"] = {"x": round(here[0], 2), "y": round(here[1], 2)}
        out["spoken_summary"] = gas_sentence(gas)
        return out

    @agent_tool()
    def prepare_permit(self, permit_id: Optional[str] = None, rescue_plan: Optional[str] = None,
                       request_attestation: bool = False) -> dict:
        """Record the entry permit: the permit number and the rescue plan the operator gives
        you. Energy isolation, lockout/tagout (LOTO), the attendant at the entry and the entry
        supervisor are attested by the operator at the console, not by you: once the operator
        says they are in place, set request_attestation to true and the console asks the
        operator to type y or n. Only that typed answer records them; you cannot answer for the
        operator. The permit records roles, never a person's name. Fields you leave out keep
        their current value. A closed permit (completed or aborted) is replaced by a new blank
        one first. The robot does not approach the entry until the permit is complete.

        Args:
            permit_id: Permit number, for example "CSE-0417".
            rescue_plan: The rescue plan or rescue service, in a few words, without names.
            request_attestation: True to ask the operator at the console to attest isolation, LOTO and the attendant.
        """
        fn = self._mission_fn("prepare_permit")
        if fn is None:
            return _refuse("This mission backend keeps no permit record.")
        if self._retreat_in_progress():
            return _refuse("A retreat is in progress; record the permit once it has finished.")
        renewed = self._mission_state() in ("COMPLETED", "ABORTED")
        refusal = self._renew_permit_if_closed()
        if refusal:
            return _refuse(refusal)
        errors = []
        result = None
        if permit_id is not None or rescue_plan is not None:
            result = fn(permit_id=permit_id, rescue_plan=rescue_plan, source=EVENT_SOURCE,
                        prepared_by="agent: permit number and rescue plan as stated by the "
                                    "operator")
            errors += list(result.get("errors") or [])
        attestation = None
        if request_attestation is True:
            fields = (result or {}).get("permit") or self._snapshot().get("permit_fields") or {}
            # The permit number and the rescue plan are the model's text:
            # quoted, one line, bounded (OWASP ASI09).
            summary = ("PERMIT ATTESTATION for permit %s. You, the operator at this console, "
                       "attest as the %s that: energy and line isolation is in place; "
                       "lockout/tagout is in place; an %s is stationed at the cleanout door. "
                       "Rescue plan on record: %s. Roles are recorded, not names. Text in "
                       "double quotes was given to the agent."
                       % (quote_agent_text(fields["permit_id"]) if fields.get("permit_id")
                          else "?", ENTRY_SUPERVISOR_ROLE, ATTENDANT_ROLE,
                          quote_agent_text(fields["rescue_plan"]) if fields.get("rescue_plan")
                          else "none yet"))
            confirmed, record = self._confirm("permit_attestation", summary,
                                              {"permit_id": fields.get("permit_id")})
            attestation = _decision_text(record)
            if confirmed:
                result = fn(isolation_confirmed=True, loto_confirmed=True,
                            attendant=ATTENDANT_ROLE, entry_supervisor=ENTRY_SUPERVISOR_ROLE,
                            attestation=record, source=EVENT_SOURCE,
                            prepared_by="operator at the console (%s)" % ENTRY_SUPERVISOR_ROLE)
                errors += list(result.get("errors") or [])
        if result is None:
            missing = self._permit_missing() or []
            result = {"ok": True, "complete": not missing, "missing": missing,
                      "permit": self._snapshot().get("permit_fields")}
        missing = list(result.get("missing") or [])
        out = {"ok": not errors, "complete": bool(result.get("complete")),
               "missing": missing, "permit": result.get("permit"),
               "new_permit_started": renewed, "permit_state": self._mission_state()}
        if attestation is not None:
            out["attestation"] = ("recorded: the operator confirmed at the console"
                                  if attestation == "confirmed" else
                                  "NOT recorded: the operator did not confirm at the console (%s)"
                                  % attestation)
        if errors:
            out["error"] = "; ".join(errors)
        elif missing:
            out["message"] = ("Recorded. Still missing: %s. The robot will not approach the "
                              "entry until they are recorded." % _describe_fields(missing))
        else:
            out["message"] = "Permit complete; the robot may approach the entry for testing."
        return out

    # -- robot state tools -------------------------------------------------

    @agent_tool()
    def claim_and_power_on(self) -> dict:
        """Claim the robot's control lease and power on its motors.

        Fails if the e-stop is asserted; a human must release the e-stop first.
        """
        ok_claim, msg_claim = self.robot.claim()
        ok_power, msg_power = self.robot.power_on()
        return {"ok": bool(ok_claim and ok_power),
                "claim": trigger_text("claim", ok_claim, msg_claim),
                "power_on": trigger_text("power_on", ok_power, msg_power)}

    def _stand_and_wait(self):
        """robot.stand(), then poll status until standing. -> result dict."""
        ok, msg = self.robot.stand()
        msg = trigger_text("stand", ok, msg)
        if not ok:
            return {"ok": False, "error": msg}
        deadline = self._clock() + STAND_TIMEOUT_S
        while not self._standing():
            if self._clock() >= deadline:
                return {"ok": False, "message": msg,
                        "error": "Stand accepted but the robot was not standing after %.0f s."
                                 % STAND_TIMEOUT_S}
            self._sleep(0.2)
        return {"ok": True, "message": msg, "standing": True}

    @agent_tool(motion=True)
    def stand_up(self) -> dict:
        """Stand the robot up and wait until it is standing (a few seconds).

        Claims and powers on automatically if needed.
        """
        if self._retreat_in_progress():
            # The facade's scripted stand-up blend would interrupt the walk out.
            return _refuse("A retreat is in progress and has control of the robot. Wait for it "
                           "to finish.")
        stand_fn = self._mission_fn("stand_up")
        if stand_fn is None:
            return self._stand_and_wait()
        # The mission also checks the body height and self-rights (bounded)
        # if Spot went down after the stand.
        result = stand_fn()
        result = result if isinstance(result, dict) else {"ok": bool(result)}
        out = {"ok": bool(result.get("ok")), "standing": bool(result.get("ok")),
               "self_right_attempts": result.get("self_right_attempts", 0),
               "body_height_m": result.get("body_z_m")}
        if not out["ok"]:
            out["error"] = result.get("error") or "The robot did not stand up."
        elif out["self_right_attempts"]:
            out["message"] = ("The robot fell after standing; it self-righted (%d attempt%s) and "
                              "is standing now." % (out["self_right_attempts"],
                                                    "" if out["self_right_attempts"] == 1 else "s"))
            if result.get("relocated_to"):
                out["relocated_to"] = result["relocated_to"]
                out["message"] += (" In the simulation the self-right put it back at (%.1f, %.1f); "
                                   "say so, it did not recover in place." % tuple(result["relocated_to"]))
        return out

    @agent_tool(motion=True)
    def sit_down(self) -> dict:
        """Stop any motion and sit the robot down where it is, outside the tank.

        Inside the tank the robot is stopped but stays standing, because a
        sitting robot cannot be walked out by the SafetyGuard. Refused while
        a retreat is running (it sits the robot itself once outside).
        """
        if self._retreat_in_progress():
            # No stop either: it would stall the retreat mid-escape.
            return _refuse("A retreat is in progress and has control of the robot; it sits the "
                           "robot itself once it is outside.")
        refusal = self._stop_refusal()
        if refusal:
            return _refuse(refusal, backend=self.backend())   # nothing sent (T15)
        self.robot.send_velocity(0.0, 0.0, 0.0)
        self.robot.stop()
        refusal = self._backend_refusal("sitting the robot down")
        if refusal:
            refusal["robot_stopped"] = True
            return refusal
        here = self._here()
        if here is None or self._in_space(here):
            return _refuse("The robot has been stopped but stays standing inside the tank, so the "
                           "SafetyGuard can always walk it out. Exit first, then sit.",
                           robot_stopped=True)
        ok, msg = self.robot.sit()
        return {"ok": bool(ok), "message": trigger_text("sit", ok, msg)}

    # -- movement tools ----------------------------------------------------

    @agent_tool(motion=True)
    def walk_to(self, x: float, y: float) -> dict:
        """Walk in a straight line to a point on open ground OUTSIDE the tank.

        Refuses targets inside the tank, paths that would cross the tank wall,
        and any use while the robot is inside the tank. Prefer walk_to_named.

        Args:
            x: Target x in meters (odom frame).
            y: Target y in meters (odom frame).
        """
        target = (float(x), float(y))
        if not all(math.isfinite(v) for v in target):
            return _refuse("Coordinates must be finite numbers.")
        here = self._here()
        if here is None:
            return _refuse("No odometry from the robot, so it cannot be moved safely.")
        center, radius = self._tank()
        if _dist(target, center) > SITE_LIMIT:
            return _refuse("That point is off the site; stay within %.0f meters of the tank."
                           % SITE_LIMIT)
        if self._in_space(here):
            return _refuse("The robot is inside the tank. Use walk_to_named or retreat_now; "
                           "free moves inside could hit the pipe runs.")
        keep_out = radius + SPOT_CLEARANCE
        if _dist(target, center) < keep_out or _point_segment_distance(center, here, target) < keep_out:
            return _refuse("That path enters or crosses the tank. Use walk_to_named with the "
                           "entry, door or waypoint names instead.")
        approach = self._mission_fn("approach_refusal")
        reason = approach(target[0], target[1]) if approach is not None else None
        if reason:
            return _refuse("Not moving there: %s. Record the permit with prepare_permit first."
                           % reason)
        refusal = self._motion_precheck(inward=False)
        if refusal:
            return refusal
        timeout = max(30.0, 2.5 * _dist(here, target) / OUTSIDE_SPEED + 10.0)
        ok = bool(self.mission.walk_to(target[0], target[1], tol=WALK_TOL,
                                       max_speed=OUTSIDE_SPEED, timeout=timeout))
        if not ok:
            self.halt_motion("could not reach (%.1f, %.1f)" % target)
        return {"ok": ok, "target": {"x": target[0], "y": target[1]},
                **({} if ok else {"error": "Could not reach the target."})}

    @agent_tool(motion=True)
    def walk_to_named(self, place: str) -> dict:
        """Walk to a named place along the pre-planned route, one safe segment
        at a time: spawn, entry, door, then waypoint_1 up to the last waypoint.

        Moving to the door or any waypoint from outside counts as entering the
        confined space and requires a passing atmospheric test. From inside the
        tank the robot either goes deeper along the route or exits completely:
        walking to entry or spawn from inside exits through the door, closes
        the permit and sits the robot at the entry point.

        Args:
            place: One of "spawn", "entry", "door", or "waypoint_1" ... "waypoint_N".
        """
        name = str(place).strip().lower().replace(" ", "_").replace("-", "_")
        index = self._place_index(name)
        if index is None:
            return _refuse("Unknown place %r. Valid places: %s."
                           % (place, ", ".join(p for p, _ in self.places)))
        here = self._here()
        if here is None:
            return _refuse("No odometry from the robot, so it cannot be moved safely.")
        start, indices = self._route(index, here)
        if indices is None:
            return _refuse(NO_ROUTE)
        if not indices:
            return {"ok": True, "message": "Already at %s." % name, "visited": []}
        # index == start: back onto the nearest route point, e.g. after a leg
        # stopped short inside. That is not an exit, so it needs the permit.
        inward = index >= start and index >= self.first_inside
        if not inward and self._in_space(here):
            if index >= self.first_inside:
                return _refuse("Inside the tank the robot only goes deeper along the route or "
                               "exits completely. Use walk_to_named entry to exit, or retreat_now.")
            refusal = self._motion_precheck(inward=False)
            if refusal:
                return refusal
            return self._exit_space(indices, name)
        entry_index = self._place_index("entry")
        if entry_index is not None and index >= entry_index and not self._in_space(here):
            refusal = self._permit_refusal()  # approaching the entry needs the permit
            if refusal:
                return refusal
        refusal = self._motion_precheck(inward=inward)
        if refusal:
            return refusal
        if inward and not self._in_space(here) and any(i >= self.first_inside for i in indices):
            # Walking to the door or a waypoint from outside is an entry too.
            route = [self.places[i][0] for i in indices]
            confirmed, record = self._confirm("entry", self._entry_summary(route),
                                              {"route": route})
            if not confirmed:
                return _refuse("The operator did not confirm the entry at the console (%s); the "
                               "robot has not moved." % _decision_text(record),
                               confirmation=record)
        result = self._walk_route(indices, inward=inward, sample_gas=inward)
        result["destination"] = name
        return result

    def _exit_space(self, indices, destination):
        """Leave the permit space through the door.

        Uses the mission's exit_space() when it has one: it retraces the
        recorded trail and closes the permit. Driving out with walk_to legs
        instead would append the outbound points to the mission's trail, and
        a guard retreat triggered mid-exit would then reverse that trail and
        head deeper into the tank first. Plain walk_to legs remain the
        fallback for a contract-only mission object.
        """
        exit_fn = getattr(self.mission, "exit_space", None)
        if not callable(exit_fn):
            result = self._walk_route(indices, inward=False)
            result["destination"] = destination
            return result
        permit_before = self._mission_state()
        if not exit_fn():
            result = self._leg_failure("the entry point", permit_before, [], [])
            if not result["guard_tripped"]:
                # exit_space() only closes an in-progress permit; retreat()
                # brings the robot out whatever the permit state.
                result["error"] += " Use retreat_now to bring the robot out along its path."
            return result
        # A normal exit is not a withdrawal: the HUD shows this line under
        # COMPLETED (leadership rehearsal 2026-09-28).
        self._revoke("exited the confined space",
                     "Entry complete: Spot exited the confined space and the permit is closed.")
        note = ("" if destination == "entry"
                else " Stand up and walk again to continue to %s." % destination)
        return {"ok": True, "visited": ["entry"], "destination": destination,
                "permit": self._mission_state(),
                "message": "Exited through the door; the permit is closed and the robot sat "
                           "down at the entry point." + note}

    @agent_tool(motion=True)
    def run_atmospheric_test(self, dwell_s: float = 4.0) -> dict:
        """Run the pre-entry atmospheric test at the cleanout door, from outside.

        Walks to the entry point first if needed. With its nose at the opening the
        robot extends its gas probe (an extendable gas probe, simulated; arm-mounted on
        the real robot) 0.5 and 1.0 meters inside and samples at 0.2, 0.6 and 1.0 meters
        at each depth, six points, in the OSHA order O2, then LEL, then H2S and CO; each
        gas is judged on its worst point. Entry is authorized only if every gas passes;
        after a failed test, or while the tank is ventilated, the ventilation purge
        target must also be reached. A failed test denies entry but can be repeated,
        for example after ventilation. Needs a complete permit. A closed permit from an
        earlier entry is replaced by a new one first.

        Args:
            dwell_s: Total test dwell in seconds, split across the three phases, 2 to 30. Default 4.
        """
        dwell = min(30.0, max(2.0, float(dwell_s)))
        entry_index = self._place_index("entry")
        if entry_index is None:
            return _refuse("The site has no entry point defined.")
        entry_xy = self.places[entry_index][1]
        here = self._here()
        if here is None:
            return _refuse("No odometry from the robot, so it cannot be moved safely.")
        if self._in_space(here):
            return _refuse("The pre-entry test is done from outside the tank. Exit first; use "
                           "get_gas_reading for a reading in place.")
        refusal = self._motion_precheck(inward=False)
        if refusal:
            return refusal
        # A new entry needs a new permit: renew a closed one before walking,
        # so the permit gate below sees the permit this test is for.
        refusal = self._renew_permit_if_closed()
        if refusal:
            return _refuse(refusal)
        refusal = self._permit_refusal()
        if refusal:
            return refusal
        walked = []
        if _dist(here, entry_xy) > AT_PLACE:
            _start, indices = self._route(entry_index, here)
            if indices is None:
                return _refuse(NO_ROUTE)
            moved = self._walk_route(indices, inward=False)
            if not moved["ok"]:
                return moved
            walked = moved["visited"]
        # The mission tests with Spot's nose at the opening (door_mouth_xy) and
        # walks the last few decimeters itself; entry_xy for a contract-only mission.
        test_xy = _xy(getattr(self.mission, "door_mouth_xy", None)) or entry_xy
        result = self.mission.atmospheric_test(test_xy, dwell_s=dwell)
        self._last_test = (self._clock(), result)
        gas_now = self.robot.gas()
        # Defence in depth: the mission's verdict AND a clean live reading.
        passed = _test_passed(result) and gas_now is not None and not _gas_alarm(gas_now)
        if passed:
            self._auth = {"permit_at_auth": self._mission_state(), "t": self._clock()}
            self._emit("permit", "Atmospheric test passed; entry authorized.",
                       {"entry_authorized": True})
        else:
            self._revoke("atmospheric test did not pass")
        out = {"ok": True, "passed": passed, "entry_authorized": passed,
               "order": "O2, LEL, H2S and CO", "walked": walked, "test": result,
               "gas_now": _gas_summary(gas_now) if gas_now else None,
               "permit": self._mission_state()}
        if _test_passed(result) and not passed:
            # The agent's veto: a live alarm (or no live reading) after a pass.
            out["spoken_summary"] = ("The test read within limits, but %s, so entry is not "
                                     "authorized." % (gas_sentence(gas_now).rstrip(".")
                                                      if gas_now else "there is no live reading"))
        else:
            out["spoken_summary"] = verdict_sentence(result)
        if isinstance(result, dict) and result.get("results"):
            out["readings_in_osha_order"] = _osha_readings(result)
            out["method"] = {"probe": "extendable gas probe (simulated; arm-mounted on the real "
                                      "robot) at six points inside the opening",
                             "body": "body gas sensor (the probe was unavailable)",
                             "probe+body": "gas probe, then the body sensor after a probe "
                                           "failure"}.get(result.get("method"), result.get("method"))
        if not passed and out["permit"] == "PRE_ENTRY_TESTING":
            if isinstance(result, dict) and result.get("purge_incomplete"):
                purge = result.get("purge") if isinstance(result.get("purge"), dict) else {}
                hold = result.get("purge_hold") if isinstance(result.get("purge_hold"), dict) else None
                out["purge_incomplete"] = True
                out["wait_s"] = purge.get("wait_s")
                if hold is not None:
                    # Exactly the mission's purge_hold event: the HUD's PURGE
                    # HOLD panel shows these same numbers.
                    out["purge_hold"] = hold
                hold_text = _purge_hold_text(hold)
                out["next_step"] = ("Purge incomplete, wait: the readings are within permissible "
                                    "entry limits at the door but the ventilation purge target is "
                                    "not reached, so entry is not authorized yet. "
                                    + (hold_text + " " if hold_text else "")
                                    + "Wait with ventilation_status (wait_s), then re-test. "
                                    + str(purge.get("message") or ""))
            elif isinstance(result, dict) and result.get("sampled_inside") is False:
                # Ventilating cannot fix a test that never sampled the space.
                out["next_step"] = ("Entry denied: the gas probe did not sample inside the "
                                    "opening, so this test cannot authorize entry. Tell the "
                                    "operator the probe is unavailable; do not suggest that "
                                    "ventilation will fix it.")
            else:
                out["next_step"] = ("Entry denied; the permit stays in pre-entry testing. "
                                    "Recommend forced ventilation (start_ventilation), wait for "
                                    "the purge (ventilation_status), then re-test.")
        return out

    @agent_tool()
    def start_ventilation(self, tau_s: Optional[float] = None) -> dict:
        """Start forced ventilation of the tank through the cleanout door (a SIMULATED
        blower in this demo). The unventilated contamination then decays toward a small
        residual over a few time constants; follow it with ventilation_status and run
        the atmospheric test again once the purge is done. Ventilation keeps running
        during the entry.

        Args:
            tau_s: Optional purge time constant in seconds, 2 to 120 (demo tuning). Default 15.
        """
        tau = None
        if tau_s is not None:
            tau = float(tau_s)
            if not (math.isfinite(tau) and VENT_TAU_RANGE_S[0] <= tau <= VENT_TAU_RANGE_S[1]):
                return _refuse("tau_s must be %g to %g seconds." % VENT_TAU_RANGE_S)
        fn = self._mission_fn("start_ventilation")
        if fn is not None:
            result = fn(tau, source=EVENT_SOURCE)
        else:
            vent = getattr(self.robot, "set_ventilation", None)
            if not callable(vent):
                return _refuse("Ventilation control is not available on this robot link.")
            ok = vent(True, tau) is not False
            self._emit("ventilation", "Forced ventilation %s (simulated)" % ("ON" if ok else "FAILED"),
                       {"on": ok, "tau_s": tau, "simulated": True})
            result = {"ok": ok, "on": ok, "tau_s": tau}
        out = dict(result)
        if out.get("ok"):
            out["message"] = ("Forced ventilation is running (simulated). The purge takes about "
                              "%s; then re-test." % ("%.0f seconds" % out["purge_estimate_s"]
                                                    if out.get("purge_estimate_s") else "a minute"))
        else:
            out.setdefault("error", "The ventilation command was not sent (robot link down).")
        return out

    @agent_tool()
    def ventilation_status(self, wait_s: float = 0.0) -> dict:
        """Report forced-ventilation progress (simulated): whether it is on, what share of
        the unventilated contamination remains, and whether the purge target is reached.
        Optionally wait for the purge to finish first. The robot does not move.

        Args:
            wait_s: Seconds to wait for the purge before reporting, 0 to 180. Default 0, report now.
        """
        wait = float(wait_s)
        wait = min(VENT_WAIT_MAX_S, max(0.0, wait if math.isfinite(wait) else 0.0))
        await_fn = self._mission_fn("await_ventilation")
        status_fn = self._mission_fn("ventilation_status")
        if wait > 0.0 and await_fn is not None:
            status = await_fn(max_s=wait, source=EVENT_SOURCE)
        elif status_fn is not None:
            status = status_fn()
        else:
            gas = self.robot.gas()
            block = gas.get("ventilation") if isinstance(gas, dict) else None
            if not isinstance(block, dict):
                return {"ok": False, "error": "No ventilation state from the simulator."}
            status = dict(block, source="sim", simulated=True)
        out = {"ok": True}
        out.update(status or {})
        out["ok"] = bool(status is not None) and out.get("error") is None
        return out

    @agent_tool(motion=True)
    def enter_confined_space(self) -> dict:
        """Enter the tank through the door and follow every interior waypoint to
        the far wall by the ladder, sampling gas at each waypoint. The
        SafetyGuard stays in charge: on any alarm it retreats the robot and this
        entry stops. Requires a passing atmospheric test first. Does not exit;
        use walk_to_named("entry") to come back out.
        """
        if len(self.places) <= self.first_inside:
            return _refuse("The site has no door or interior waypoints defined.")
        here = self._here()
        if here is None:
            return _refuse("No odometry from the robot, so it cannot be moved safely.")
        refusal = self._permit_refusal() if not self._in_space(here) else None
        if refusal:
            return refusal
        refusal = self._motion_precheck(inward=True)
        if refusal:
            return refusal
        last = len(self.places) - 1
        start, indices = self._route(last, here)
        if indices is None:
            return _refuse(NO_ROUTE)
        if not indices:
            return {"ok": True, "message": "Already at the far waypoint.", "visited": []}
        if not self._in_space(here):
            confirmed, record = self._confirm(
                "entry", self._entry_summary([self.places[i][0] for i in indices]),
                {"route": [self.places[i][0] for i in indices]})
            if not confirmed:
                return _refuse("The operator did not confirm the entry at the console (%s); the "
                               "robot has not moved." % _decision_text(record),
                               confirmation=record)
        self._emit("entry", "Entering the confined space.",
                   {"route": [self.places[i][0] for i in indices]})
        enter_fn = getattr(self.mission, "enter", None)
        if callable(enter_fn) and start < self.first_inside:
            result = self._enter_with_mission(enter_fn, [i for i in indices if i < self.first_inside])
        else:
            result = self._walk_route(indices, inward=True, sample_gas=True)
        if result["readings"]:
            result["peak"] = {
                "h2s_ppm": max(r.get("h2s_ppm", 0.0) for r in result["readings"]),
                "lel_pct": max(r.get("lel_pct", 0.0) for r in result["readings"]),
                "co_ppm": max(r.get("co_ppm", 0.0) for r in result["readings"]),
                "o2_pct_min": min(r.get("o2_pct", 20.9) for r in result["readings"]),
            }
        return result

    def _test_text(self, test):
        """'PASS: O2 20.8 %, LEL 0.7 % LEL, ...' for an atmospheric_test() result."""
        if not isinstance(test, dict):
            return "no atmospheric test on record"
        parts = []
        for entry in test.get("results") or []:
            if isinstance(entry, dict) and isinstance(entry.get("value"), (int, float)):
                parts.append("%s %.1f %s" % (GAS_TEXT.get(entry.get("gas"), entry.get("gas")),
                                             float(entry["value"]), entry.get("unit") or ""))
        verdict = "PASS" if _test_passed(test) else "FAIL"
        return "%s%s" % (verdict, (": " + ", ".join(parts) + " (worst sampled points)")
                         if parts else "")

    def _entry_summary(self, route, test=None):
        """Code-rendered text of an entry the operator is asked to confirm."""
        if test is None and self._last_test is not None:
            age = self._clock() - self._last_test[0]
            latest = "%s, %.0f s ago" % (self._test_text(self._last_test[1]), age)
        else:
            latest = self._test_text(test)
        permit_id = self._snapshot().get("permit_id")
        return ("ENTRY INTO THE PERMIT SPACE: Spot walks through the cleanout door and follows %s "
                "(permit %s, state %s). Latest atmospheric test: %s. The door is re-tested and "
                "the battery and ventilation re-checked just before crossing."
                % (", ".join(route) if route else "the interior waypoints",
                   quote_agent_text(permit_id) if permit_id else "?",   # the model's text (ASI09)
                   self._mission_state(), latest))

    def _enter_with_mission(self, enter_fn, outside_indices):
        """Walk the outside part of the route, then let mission.enter() drive
        door -> waypoints: it emits the per-waypoint audit events the HUD and
        the permit summary count, and its on_waypoint hook samples gas."""
        moved = self._walk_route(outside_indices, inward=False)
        if not moved["ok"]:
            return moved
        readings = []

        def on_waypoint(index, _x, _y):
            gas = self.robot.gas()
            if gas:
                readings.append(dict(place="waypoint_%d" % (index + 1), **_gas_summary(gas)))

        permit_before = self._mission_state()
        ok = enter_fn(on_waypoint=on_waypoint)
        reached = [r["place"] for r in readings]
        visited = moved["visited"] + (["door"] + reached if (ok or reached) else [])
        if not ok:
            return self._leg_failure("the far waypoint", permit_before, visited, readings,
                                     inward=True)
        return {"ok": True, "visited": visited, "readings": readings,
                "permit": self._mission_state()}

    @agent_tool(motion=True)
    def full_confined_space_entry(self, ventilate_if_needed: bool = False) -> dict:
        """Run the complete scripted permit-required entry: stand, walk to the
        entry point, atmospheric test in OSHA order, authorize, enter and follow
        the waypoints with continuous monitoring, exit back through the door,
        and sit. Takes a few minutes. Retreats automatically on any alarm.
        Needs a complete permit (prepare_permit).

        Args:
            ventilate_if_needed: If true, a failed first test starts forced ventilation, waits for the purge and re-tests before entering; if false, a failed test ends the entry. Default false.
        """
        here = self._here()
        if here is not None and self._in_space(here):
            return _refuse("The robot is already inside the tank. Bring it out first with "
                           "walk_to_named entry or retreat_now.")
        entry = self._place_index("entry")
        if here is not None and entry is not None and self._leg_clips_tank(
                here, self.places[entry][1]):
            # mission.run() walks straight to entry_xy first.
            return _refuse(NO_ROUTE)
        if _gas_alarm(self.robot.gas()):
            return _refuse("A gas alarm is active; the SafetyGuard has control of the robot.")
        if self._retreat_in_progress():
            return _refuse("A retreat is in progress; wait for it to finish.")
        refusal = self._renew_permit_if_closed()
        if refusal:
            return _refuse(refusal)
        refusal = self._permit_refusal()
        if refusal:
            return refusal
        self._revoke("full mission run manages its own permit")
        kwargs = {"auto_retreat": True}
        if ventilate_if_needed is True and _accepts_kwarg(self.mission.run, "storyline"):
            kwargs["storyline"] = True
        route = [p for p, _xy_ in self.places[self.first_inside:]]
        if _accepts_kwarg(self.mission.run, "confirm_entry"):
            # Asked once the run's own test has authorized entry, at the door.
            kwargs["confirm_entry"] = lambda test: self._confirm(
                "entry", self._entry_summary(route, test), {"route": route})
        else:
            confirmed, record = self._confirm(
                "entry", "FULL SCRIPTED ENTRY: stand, test the atmosphere at the door and, if it "
                "passes, enter and follow %s, then exit." % ", ".join(route), {"route": route})
            if not confirmed:
                return _refuse("The operator did not confirm the entry at the console (%s); "
                               "nothing was done." % _decision_text(record),
                               confirmation=record)
        summary = self.mission.run(**kwargs)
        return {"ok": True, "summary": summary, "permit": self._mission_state()}

    @agent_tool()
    def retreat_now(self) -> dict:
        """Abort the entry: retrace the visited path back out through the door
        to the entry point, then sit. Use on any doubt about safety inside.
        """
        # Not motion-locked on purpose: retreat must work even while another
        # movement is in progress (the mission's retreat overrides it, the same
        # way the SafetyGuard does).
        self._revoke("retreat ordered")
        # A backend identity conflict still lets Spot be brought out (T9).
        refusal = self._backend_refusal("walking the robot out (retreat)", retreat=True)
        if refusal:
            refusal["robot_stopped"] = self.halt_motion(
                "retreat refused: the backend is not the simulator")
            return refusal
        status = self.robot.status()
        if status and status.get("estopped"):
            # The retreat's cmd_vel would be ignored for its whole 240 s budget.
            return _refuse("The e-stop is asserted, so the robot cannot walk out. A human must "
                           "release the e-stop first.")
        here = self._here()
        entry = self._place_index("entry")
        if (here is not None and not self._in_space(here) and entry is not None
                and self._leg_clips_tank(here, self.places[entry][1])):
            # From outside, mission.retreat() walks straight to entry_xy.
            self.halt_motion("retreat would cross the tank")
            return _refuse("The robot is already outside the tank, on a side where walking back "
                           "to the entry point would cross the tank. It has been stopped instead.",
                           robot_stopped=True)
        if (status and not status.get("standing") and not self._retreat_in_progress()
                and here is not None and self._in_space(here)
                and not self.backend().get("conflict")):
            # The facade ignores cmd_vel unless standing. The mission's
            # stand_up also self-rights (bounded) if Spot goes down. In a
            # backend identity conflict stand_up is refused; the mission's
            # retreat stands Spot up itself as its first step (T9).
            stand_fn = self._mission_fn("stand_up")
            stood = stand_fn() if stand_fn is not None else self._stand_and_wait()
            stood = stood if isinstance(stood, dict) else {"ok": bool(stood)}
            if not stood.get("ok"):
                return {"ok": False, "error": "Could not stand the robot up to retreat: %s"
                                              % stood.get("error", "stand failed")}
        result = self.mission.retreat()
        out = {"ok": result is not False, "result": result, "permit": self._mission_state()}
        if result is False:
            out["error"] = ("The retreat did not get the robot back to the entry point; operator "
                            "attention is needed.")
        return out

    @agent_tool()
    def explain_last_incident(self) -> dict:
        """Get the most recent SafetyGuard incident from the audit log: which gas
        alarmed and its reading versus the limit, when and where it happened, how fast
        the guard stopped the robot, how the retreat ended, the permit outcome, and
        whether the gas was a presenter-staged simulated spike. Use it whenever you are
        asked what happened, then explain it plainly.
        """
        report = self._incident_report()
        serial = self._incident_serial()
        if serial is not None:
            self._incident_reported = serial
        out = {"ok": True}
        out.update(report)
        if not report.get("found"):
            out["message"] = "No SafetyGuard incident has been recorded in this session."
        return out

    @agent_tool()
    def emergency_stop(self, confirm: bool = False, mode: str = "hard") -> dict:
        """E-stop the robot. mode "hard" cuts motor power at once: the robot drops where it
        stands and cannot walk out on its own until a person releases the e-stop. mode "gentle"
        lets it settle before the power is cut. Inside the permit space either mode also needs
        the operator's confirmation at the console; the robot is stopped while it waits. For a
        gas alarm inside, the SafetyGuard's retreat is usually better than either.

        Args:
            confirm: Must be true to act; the operator asked for an e-stop.
            mode: "hard" (default) or "gentle".
        """
        if confirm is not True:
            return _refuse("Not e-stopped: confirmation required. Ask the operator to confirm, "
                           "then call emergency_stop with confirm set to true.")
        mode = str(mode).strip().lower()
        if mode not in ESTOP_MODES:
            return _refuse("mode must be one of %s." % ", ".join(ESTOP_MODES))
        refusal = self._stop_refusal("e-stopping the robot")
        if refusal:
            return _refuse(refusal, backend=self.backend())   # nothing sent (T15)
        record = None
        here = self._here()
        inside = (here is None or self._in_space(here)
                  or self._mission_state() == "ENTRY_IN_PROGRESS")
        if inside:
            # Both modes end with motor power cut (the gentle one once Spot has
            # settled), which leaves Spot inside and ends a retreat (T8).
            # Stopping never waits: halt now, then ask before cutting power.
            # A running retreat is not halted (it would stall the escape), and
            # neither the question nor the refusal may then say Spot is
            # stopped: it is walking out (T8, T10).
            retreating = self._retreat_in_progress()
            stopped = self.halt_motion("%s e-stop requested inside the permit space" % mode)
            question = {
                "hard": "HARD E-STOP INSIDE THE PERMIT SPACE: cut motor power now? Spot drops "
                        "where it stands",
                "gentle": "GENTLE E-STOP INSIDE THE PERMIT SPACE: let Spot settle, then cut motor "
                          "power? Spot stays where it settles"}[mode]
            meanwhile = ("It has been stopped meanwhile." if stopped else
                         "A retreat is walking it out of the permit space meanwhile; it has "
                         "not been stopped." if retreating else
                         "It could not be stopped meanwhile.")
            confirmed, record = self._confirm(
                "%s_estop_inside" % mode, question + " and cannot walk out until a person "
                "releases the e-stop. " + meanwhile,
                {"pose": list(here) if here else None, "mode": mode})
            if not confirmed:
                now = ("The robot is stopped with its power on, so the SafetyGuard can still "
                       "walk it out." if stopped else
                       "The robot was NOT stopped: a retreat is walking it out of the permit "
                       "space, with its power on." if retreating else
                       "The robot could not be stopped; its power is on.")
                if record.get("decision") == NOT_ASKED:
                    now += " The operator's /estop at the console e-stops at once."
                return _refuse("The operator did not confirm the %s e-stop inside the permit "
                               "space (%s). %s" % (mode, _decision_text(record), now),
                               robot_stopped=stopped, retreat_in_progress=retreating,
                               confirmation=record)
        if mode == "gentle":
            gentle = getattr(self.robot, "estop_gentle", None)
            ok, msg = gentle() if callable(gentle) else (False, "no gentle e-stop on this link")
        else:
            ok, msg = self.robot.estop_hard()
        msg = trigger_text("estop/%s" % mode, ok, msg)
        try:
            self.robot.send_velocity(0.0, 0.0, 0.0)
        except Exception:
            log.exception("emergency_stop: zero twist failed")
        self._revoke("emergency stop")
        out = {"ok": bool(ok), "mode": mode, "message": msg}
        if record is not None:
            out["confirmation"] = record
        return out

    def release_estop(self):
        """Release the e-stop after the operator confirms at the console
        (console /release; not a model tool). Simulator only."""
        refusal = self._backend_refusal("releasing the e-stop")
        if refusal:
            return refusal
        confirmed, record = self._confirm(
            "estop_release", "RELEASE THE E-STOP: the robot can be powered and driven again "
            "afterwards. Release it?", limit=False)   # the operator's own /release
        if not confirmed:
            return _refuse("E-stop not released: the operator did not confirm (%s)."
                           % _decision_text(record), confirmation=record)
        ok, msg = self.robot.estop_release()
        return {"ok": bool(ok), "message": trigger_text("estop/release", ok, msg),
                "confirmation": record}

    @agent_tool()
    def inject_gas_demo_spike(self, gas: str = "h2s", ppm: float = 40.0,
                              x: Optional[float] = None, y: Optional[float] = None,
                              z: float = 0.5, radius: float = 1.5) -> dict:
        """PRESENTER ONLY: inject a simulated gas spike to demonstrate the alarm
        and automatic retreat. Only call when the presenter explicitly asks.

        Args:
            gas: "h2s", "lel", "o2" or "co".
            ppm: Peak amount: ppm for h2s and co, percentage points for lel; for o2 a
                negative percentage-point change (depletion only, down to -20.9).
            x: Spike center x in meters; default is the robot's current position.
            y: Spike center y in meters; default is the robot's current position.
            z: Spike center height in meters. Default 0.5, about body height.
            radius: Spike size (one sigma) in meters, 0.2 to 5. Default 1.5.
        """
        if self._env.get(PRESENTER_ENV) != "1":
            return _refuse("Gas injection is presenter-only and is disabled in this session "
                           "(%s is not set to 1)." % PRESENTER_ENV)
        gas_name = str(gas).strip().lower()
        if gas_name not in GAS_NAMES:
            return _refuse("gas must be one of %s." % ", ".join(GAS_NAMES))
        amount, z, radius = float(ppm), float(z), float(radius)
        if not math.isfinite(amount) or (gas_name != "o2" and amount <= 0.0):
            return _refuse("ppm must be a positive number (only o2 may be negative).")
        if gas_name == "o2" and not O2_DEPLETION_RANGE[0] <= amount < O2_DEPLETION_RANGE[1]:
            # The facade accepts O2 depletion only: an enrichment would mask a
            # real depletion in the simulated field (T23).
            return _refuse("For o2, ppm must be a depletion: %g to below %g percentage points."
                           % O2_DEPLETION_RANGE)
        if not 0.2 <= radius <= 5.0 or not 0.0 <= z <= 3.0:
            return _refuse("radius must be 0.2 to 5 meters and z 0 to 3 meters.")
        if x is None or y is None:
            here = self._here() or self._tank()[0]
            x = here[0] if x is None else x
            y = here[1] if y is None else y
        x, y = float(x), float(y)
        if self.robot.inject_gas(gas_name, amount, x, y, z, radius) is False:
            return {"ok": False, "error": "Injection not sent: the robot link is down."}
        injected = {"gas": gas_name, "ppm": amount, "x": round(x, 2), "y": round(y, 2),
                    "z": z, "radius": radius}
        # Disclose it in the mission's JSONL audit log too, so the permit
        # record never presents a staged spike as a real atmosphere reading.
        disclose = getattr(self.mission, "log_event", None)
        if callable(disclose):
            try:
                disclose("gas_injected", "PRESENTER DEMO: simulated %s spike injected at "
                         "(%.1f, %.1f)" % (gas_name.upper(), x, y), injected, source=EVENT_SOURCE)
            except Exception:
                log.warning("mission.log_event(gas_injected) failed", exc_info=True)
        return {"ok": True, "injected": injected}

    @agent_tool()
    def clear_injected_gas(self, reset_tank: bool = False) -> dict:
        """PRESENTER ONLY: remove the simulated gas spikes injected for the demo. With
        reset_tank true, also put the simulated tank back to its unventilated starting
        atmosphere for the next demo run (only while the robot is outside). Only call
        when the presenter explicitly asks.

        Args:
            reset_tank: Also reset the simulated tank atmosphere to its unventilated start. Default false.
        """
        if self._env.get(PRESENTER_ENV) != "1":
            return _refuse("Clearing gas is presenter-only and is disabled in this session "
                           "(%s is not set to 1)." % PRESENTER_ENV)
        if reset_tank is True:
            here = self._here()
            if (here is None or self._in_space(here) or self._retreat_in_progress()
                    or self._mission_state() == "ENTRY_IN_PROGRESS"):
                return _refuse("Not resetting the tank atmosphere while the robot is inside, "
                               "entering or retreating; bring it out first.")
            reset_fn = self._mission_fn("reset_atmosphere")
            if reset_fn is not None:
                result = reset_fn(source=EVENT_SOURCE)
            else:
                steps = [getattr(self.robot, n, None) for n in ("clear_gas", "reset_ventilation")]
                if not all(callable(f) for f in steps):
                    return _refuse("This robot link cannot reset the simulated atmosphere.")
                ok = all(f() is not False for f in steps)
                result = {"ok": ok}
            self._revoke("the simulated tank atmosphere was reset")
            out = {"ok": bool(result.get("ok")), "cleared": True, "tank_reset": True,
                   "message": "Injected spikes cleared and the simulated tank is back to its "
                              "unventilated start; a new permit and test are needed."}
        else:
            clear_fn = self._mission_fn("clear_injected_gas")
            if clear_fn is not None:
                result = clear_fn(source=EVENT_SOURCE)
            else:
                fn = getattr(self.robot, "clear_gas", None)
                if not callable(fn):
                    return _refuse("This robot link cannot clear injected gas.")
                result = {"ok": fn() is not False}
            out = {"ok": bool(result.get("ok")), "cleared": True, "tank_reset": False,
                   "message": "Injected (simulated) gas spikes cleared."}
        if not out["ok"]:
            out["error"] = "The command was not sent (robot link down)."
        return out


# ---------------------------------------------------------------------------
# Strands wiring (imported lazily so tests never need the SDK)
# ---------------------------------------------------------------------------

class TranscriptCallback:
    """Strands callback_handler: print text, reasoning and tool starts to the
    console, and publish each complete assistant text as a kind="say" event.

    Reply text is buffered and printed through ``sanitize_speech`` when the
    text block ends (a tool starts or the message completes), not streamed
    delta by delta: a phrase the post-filter rewrites can span deltas, and
    the console must show what the HUD shows. Reasoning is buffered the same
    way. Every line of model text starts with a fixed prefix (``agent> ``,
    ``agent (reasoning)> ``) and the console's own line markers inside it are
    made inert (``neutralise_console_markers``), so a model line can never
    read as a ``[readings]`` note, an ``[OPERATOR ATTENTION]`` notice, a
    confirmation question, the prompt or a log line (T10)."""

    def __init__(self, on_say, out=None):
        self._on_say = on_say
        self._out = out or sys.stdout
        self._dim = hasattr(self._out, "isatty") and self._out.isatty()
        self._pending = []
        self._reasoning = []
        self._bol = True          # the last write ended a line

    def _write(self, text, dim=False):
        text = clean_text(text, None, newlines=True)   # an echoed escape cannot forge a line
        if not text:
            return
        self._bol = text.endswith("\n")
        if dim and self._dim:
            text = "\033[2m%s\033[0m" % text
        self._out.write(text)
        self._out.flush()

    def _newline(self):
        if not self._bol:
            self._write("\n")

    def _write_model(self, text, prefix, dim=False):
        """Model text from the start of a line, every line after ``prefix``."""
        text = neutralise_console_markers(clean_text(text, None, newlines=True)).strip("\n")
        if not text.strip():
            return
        self._newline()
        self._write("\n".join(prefix + line if line.strip() else prefix.rstrip()
                              for line in text.split("\n")), dim=dim)

    def _flush_reasoning(self):
        if self._reasoning:
            text, self._reasoning = "".join(self._reasoning), []
            self._write_model(text, REASONING_PREFIX, dim=True)

    def _flush_text(self):
        self._flush_reasoning()
        if self._pending:
            text, self._pending = "".join(self._pending), []
            self._write_model(sanitize_speech(text)[0], AGENT_PREFIX)

    def flush(self):
        """Print any reply text still buffered: a turn that ends on an error,
        a timeout or Ctrl-C sends no completed message, and the fragment
        must not be glued to the start of the next reply."""
        if self._pending or self._reasoning:
            self._flush_text()
            self._newline()

    def __call__(self, **kwargs):
        if kwargs.get("force_stop") or kwargs.get("init_event_loop"):
            self.flush()   # Strands: turn aborted / a new turn starts
        text = kwargs.get("data")
        if isinstance(text, str) and text:
            self._flush_reasoning()
            self._pending.append(text)
        reasoning = kwargs.get("reasoningText")
        if isinstance(reasoning, str) and reasoning:
            if self._pending:
                self._flush_text()
            self._reasoning.append(reasoning)
        event = kwargs.get("event")
        if isinstance(event, dict):
            start = (event.get("contentBlockStart") or {}).get("start") or {}
            tool_use = start.get("toolUse")
            if isinstance(tool_use, dict):
                self._flush_text()
                self._newline()
                # The tool name comes from the model: one line, inert (T10).
                name = neutralise_console_markers(clean_text(tool_use.get("name", "?"), 64))
                self._write("%s%s\n" % (TOOL_PREFIX, name), dim=True)
        if "result" in kwargs:
            self._flush_text()
        message = kwargs.get("message")
        if isinstance(message, dict) and message.get("role") == "assistant":
            self._flush_text()
            said = " ".join(block["text"].strip() for block in message.get("content") or []
                            if isinstance(block, dict) and isinstance(block.get("text"), str)
                            and block["text"].strip())
            if said:
                self._newline()
                self._on_say(said)


class QuietGuardConsole(logging.Filter):
    """Console filter for the interactive REPL: drops the SafetyGuard's
    routine INFO lines (mission logger, "[guard] monitor: O2 ..." every 5 s
    while Spot is inside). They printed over the "operator>" prompt while
    the operator was typing (dry run 2026-09-27). Nothing is lost: the
    mission still publishes every guard event on payload/events (HUD) and
    writes it to its JSONL audit log, which never go through logging. Guard
    alarms, retreats, permit changes and every WARNING/ERROR still print."""

    def filter(self, record):
        if record.name != "mission" or record.levelno > logging.INFO:
            return True
        args = record.args if isinstance(record.args, tuple) else ()
        return not (len(args) >= 2 and args[0] == "guard" and args[1] == "monitor")


class CleanConsoleLog(logging.Filter):
    """Strip control, escape and invisible format characters from every
    console log line, and make the console's own markers in it inert (T10,
    T38): robot-side or model text in a log message cannot move the cursor,
    recolour, forge a console line or print an "[audit head]" line."""

    def filter(self, record):
        try:
            message = record.getMessage()
        except Exception:
            return True
        record.msg = neutralise_console_markers(clean_text(message, None, newlines=True))
        record.args = None
        return True


def clean_console_log(logger=None):
    """Attach CleanConsoleLog to every handler of ``logger`` (default root)."""
    handlers = (logger or logging.getLogger()).handlers
    for handler in handlers:
        if not any(isinstance(f, CleanConsoleLog) for f in handler.filters):
            handler.addFilter(CleanConsoleLog())
    return len(handlers)


def quiet_guard_console(logger=None):
    """Attach QuietGuardConsole to every handler of ``logger`` (default: the
    root logger, i.e. the stderr console from basicConfig). Returns the
    number of handlers filtered."""
    handlers = (logger or logging.getLogger()).handlers
    for handler in handlers:
        if not any(isinstance(f, QuietGuardConsole) for f in handler.filters):
            handler.addFilter(QuietGuardConsole())
    return len(handlers)


def resolve_guardrail(args, env=None):
    """``(guardrail_id, guardrail_version, source)`` for this session, or
    ValueError if the combination is unusable or no guardrail is configured
    (T11, M11).

    The guardrail is on by default. The flags win over the environment, so a
    presenter can point one session at another guardrail without changing the
    host; ``--no-guardrail`` is the only way to run without one, it cannot be
    combined with an id, and the caller warns and records it. ``source`` is
    ``"flag"``, ``"environment"`` or ``"disabled"``."""
    env = os.environ if env is None else env
    flag_id = str(getattr(args, "guardrail_id", None) or "").strip()
    flag_version = str(getattr(args, "guardrail_version", None) or "").strip()
    disabled = bool(getattr(args, "no_guardrail", False))
    if disabled and (flag_id or flag_version):
        raise ValueError("--no-guardrail cannot be combined with --guardrail-id "
                         "or --guardrail-version")
    if bool(flag_id) != bool(flag_version):
        raise ValueError("--guardrail-id and --guardrail-version go together")
    if flag_id:
        return _checked_guardrail(flag_id, flag_version, "flag", "--guardrail-id",
                                  "--guardrail-version")
    env_id = str(env.get(GUARDRAIL_ID_ENV, "") or "").strip()
    env_version = str(env.get(GUARDRAIL_VERSION_ENV, "") or "").strip()
    if env_id and not disabled:
        if not env_version:
            raise ValueError("%s is set without %s; they go together"
                             % (GUARDRAIL_ID_ENV, GUARDRAIL_VERSION_ENV))
        return _checked_guardrail(env_id, env_version, "environment",
                                  GUARDRAIL_ID_ENV, GUARDRAIL_VERSION_ENV)
    if disabled:
        return None, None, "disabled"
    raise ValueError(NO_GUARDRAIL_REFUSAL)


def _checked_guardrail(guardrail_id, guardrail_version, source, id_name, version_name):
    """Validate the shape of a guardrail id and version, naming where they
    came from so the operator can fix the right flag or variable."""
    if not _GUARDRAIL_ID.match(guardrail_id):
        raise ValueError("%s must be a Bedrock guardrail id or ARN" % id_name)
    if not _GUARDRAIL_VERSION.match(guardrail_version):
        raise ValueError("%s must be DRAFT or a version number, e.g. 1" % version_name)
    return guardrail_id, guardrail_version, source


def model_settings(model_id, guardrail_id=None, guardrail_version=None):
    """(settings for BedrockModel, warnings) for the chosen model and the
    optional guardrail, or ValueError for an unusable combination (T11)."""
    model_id = str(model_id or "").strip()
    if not _MODEL_ID.match(model_id):
        raise ValueError("--model must be a Bedrock model id, inference-profile id or ARN")
    if bool(guardrail_id) != bool(guardrail_version):
        raise ValueError("--guardrail-id and --guardrail-version go together")
    warnings = []
    if model_id != DEFAULT_MODEL_ID:
        warnings.append("UNTESTED MODEL %s: the prompt rules and the demo were measured only on "
                        "%s; evaluate it before a demo, and check the IAM policy and the "
                        "model's lifecycle" % (model_id, DEFAULT_MODEL_ID))
    settings = {"model_id": model_id}
    if guardrail_id:
        settings.update(
            guardrail_id=str(guardrail_id), guardrail_version=str(guardrail_version),
            guardrail_trace="enabled",
            # The PROMPT_ATTACK filter fires only on text the request marks as the
            # user's, and nothing else does: with Converse that is a guardContent
            # block, and guardrail_latest_message is what makes BedrockModel wrap
            # the last user text message in one (read in the strands-agents 1.62.0
            # source, _format_request_messages / _find_last_user_text_message_index;
            # it skips toolResult turns). It also keeps the system prompt, which is
            # full of the safety instructions a prompt attack imitates, out of the
            # evaluation. Tool results are not screened by the guardrail: M6's typed
            # allowlist covers that path instead.
            guardrail_latest_message=True,
            # A blocked instruction replaces that whole user turn, so say what
            # happened rather than leaving the default "[User input redacted.]": the
            # turn may also have carried a between-turn incident notice, and the
            # deterministic guard, not the model, is what retreats the robot.
            guardrail_redact_input_message=(
                "[The operator's instruction was blocked by the Bedrock guardrail and is "
                "not shown. Nothing was commanded. Any safety action already taken stands.]"),
        )
    return settings, warnings


def build_agent(spot_tools, model_id=DEFAULT_MODEL_ID, region=DEFAULT_REGION, out=None,
                guardrail_id=None, guardrail_version=None):
    """Construct the Strands Agent for ``spot_tools`` (see module docstring
    for the verified API references)."""
    from botocore.config import Config as BotocoreConfig
    from strands import Agent, ModelRetryStrategy, tool
    from strands.models import BedrockModel
    from strands.tools.executors import SequentialToolExecutor

    settings, warnings = model_settings(model_id, guardrail_id, guardrail_version)
    for warning in warnings:
        log.warning("%s", warning)
    model = BedrockModel(
        region_name=region,
        max_tokens=MAX_TOKENS,
        # Fail fast on a dead network instead of hanging in front of an
        # audience: short connect timeout, bounded read timeout, few retries.
        boto_client_config=BotocoreConfig(
            connect_timeout=10, read_timeout=90,
            retries={"max_attempts": 2, "mode": "standard"}),
        **settings
    )
    return Agent(
        model=model,
        tools=[tool(fn) for fn in spot_tools.tool_functions()],
        system_prompt=build_system_prompt(spot_tools.site),
        callback_handler=TranscriptCallback(spot_tools.say, out=out),
        tool_executor=SequentialToolExecutor(),
        # Default is 6 attempts with backoff growing to 64 s; 3 attempts with
        # delays capped at 8 s surface a throttled/broken model call quickly.
        retry_strategy=ModelRetryStrategy(max_attempts=3, initial_delay=2, max_delay=8),
    )


def _flush_transcript(agent):
    """Show reply text a cut-off turn left in the TranscriptCallback buffer."""
    flush = getattr(getattr(agent, "callback_handler", None), "flush", None)
    if callable(flush):
        try:
            flush()
        except Exception:
            log.debug("transcript flush failed", exc_info=True)


CONSOLE_HELP = """Console commands (they work at any time and never go to the model):
  /stop or stop  stop the robot now: motion halted, power on (a running retreat continues)
  /retreat       bring Spot out of the permit space along its path
  /estop         gentle e-stop (settle, then cut motor power); /estop hard cuts power at once
  /status        robot, permit, gas and SafetyGuard state
  /release       release the e-stop (asks for confirmation)
  /help          this list
  quit           exit (refused while Spot is inside the permit space)"""
BUSY_TEXT = ("(The robot is acting on the last instruction. /stop stops it, /retreat brings it out, "
             "/status shows its state, /help lists the console commands.)\n")
UNUSABLE_TEXT = ("The model session is unusable: an instruction never finished. Console commands "
                 "still work (/stop, /retreat, /status); restart the agent for a new model "
                 "session.")
# A guard retreat may be retried (agent/mission.py RETREAT_RETRIES, T17):
# wait for all of its attempts before exiting.
EXIT_RETREAT_WAIT_S = 900.0
SESSION_BUDGET_TEXT = ("Session model budget reached (%s): this instruction was not sent to the "
                       "model, and no further one will be. Console commands still work (/stop, "
                       "/retreat, /estop, /status) and the SafetyGuard keeps monitoring; restart "
                       "the agent for a new session.")


class SessionBudget:
    """Per-session model budget (T39): model turns (Strands event-loop
    cycles), output tokens and input tokens over the life of one Agent,
    read from its ``event_loop_metrics`` (``cycle_count`` and
    ``accumulated_usage``, both cumulative across invocations; field names
    as in strands-agents 1.62.0, the version agent/requirements.lock pins,
    telemetry/metrics.py and types/event_loop.py). ``ask()`` refuses an
    instruction once any cap is reached, and passes each one the Strands
    per-invocation ``limits`` cut to what is left (``turns``,
    ``output_tokens``, and ``total_tokens`` for what is left of both token
    caps: Strands 1.62.0 has no input-token limit). Input tokens are counted
    as Strands counts a prompt (``_total_prompt_tokens``): ``inputTokens``,
    plus the cache-read and cache-write counters when the provider reports
    them on top of it, as Bedrock does. Where the agent exposes no metrics,
    an instruction counts as its whole turn cap (fail closed). Nothing here
    touches the robot."""

    def __init__(self, max_turns=SESSION_MAX_TURNS, max_output_tokens=SESSION_MAX_OUTPUT_TOKENS,
                 max_input_tokens=SESSION_MAX_INPUT_TOKENS):
        for name, value in (("max_turns", max_turns), ("max_output_tokens", max_output_tokens),
                            ("max_input_tokens", max_input_tokens)):
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError("the session %s must be a positive integer, got %r"
                                 % (name.replace("_", " "), value))
        self.max_turns, self.max_output_tokens = max_turns, max_output_tokens
        self.max_input_tokens = max_input_tokens
        self.turns = 0
        self.output_tokens = 0
        self.input_tokens = 0
        self.reported = False      # the one audit record when the budget is first hit

    @staticmethod
    def _count(value):
        return value if isinstance(value, int) and not isinstance(value, bool) else None

    @classmethod
    def _input_tokens(cls, usage):
        """Prompt tokens in a Strands ``Usage`` dict, or None."""
        tokens = cls._count(usage.get("inputTokens"))
        if tokens is None:
            return None
        out, total = cls._count(usage.get("outputTokens")), cls._count(usage.get("totalTokens"))
        if out is not None and total is not None and tokens + out == total:
            return tokens            # cache tokens, if any, are inside inputTokens
        return tokens + sum(cls._count(usage.get(key)) or 0
                            for key in ("cacheReadInputTokens", "cacheWriteInputTokens"))

    @classmethod
    def _metrics(cls, agent, result=None):
        """(cycle_count, output_tokens, input_tokens) from the agent's
        metrics; None each when not available."""
        metrics = getattr(agent, "event_loop_metrics", None)
        if metrics is None:
            metrics = getattr(result, "metrics", None)
        cycles = getattr(metrics, "cycle_count", None)
        usage = getattr(metrics, "accumulated_usage", None)
        if not isinstance(usage, dict):
            return cls._count(cycles), None, None
        return cls._count(cycles), cls._count(usage.get("outputTokens")), cls._input_tokens(usage)

    def _update_tokens(self, out, inp):
        if out is not None:
            self.output_tokens = max(self.output_tokens, out)
        if inp is not None:
            self.input_tokens = max(self.input_tokens, inp)

    def refresh(self, agent):
        """Catch up with the agent's cumulative metrics (e.g. a turn that
        finished after its instruction timed out)."""
        cycles, out, inp = self._metrics(agent)
        if cycles is not None:
            self.turns = max(self.turns, cycles)
        self._update_tokens(out, inp)

    def observe(self, agent, result=None, requested_turns=MAX_TURNS):
        """After an instruction: update from the metrics, or count the
        instruction's whole turn cap when there are none."""
        cycles, out, inp = self._metrics(agent, result)
        if cycles is None:
            self.turns += requested_turns
        else:
            self.turns = max(self.turns, cycles)
        self._update_tokens(out, inp)

    def exhausted(self):
        """Why the session's budget is spent, or None."""
        if self.turns >= self.max_turns:
            return "%d of %d model turns used" % (self.turns, self.max_turns)
        if self.output_tokens >= self.max_output_tokens:
            return "%d of %d output tokens used" % (self.output_tokens, self.max_output_tokens)
        if self.input_tokens >= self.max_input_tokens:
            return "%d of %d input tokens used" % (self.input_tokens, self.max_input_tokens)
        return None

    def limits(self):
        """Strands per-invocation limits: the per-instruction turn cap, and
        the caps cut to what is left of the session (token caps are soft:
        checked at turn boundaries)."""
        left_out = max(1, self.max_output_tokens - self.output_tokens)
        left_in = max(1, self.max_input_tokens - self.input_tokens)
        return {"turns": max(1, min(MAX_TURNS, self.max_turns - self.turns)),
                "output_tokens": left_out, "total_tokens": left_out + left_in}

    def snapshot(self):
        return {"turns": self.turns, "max_turns": self.max_turns,
                "output_tokens": self.output_tokens,
                "max_output_tokens": self.max_output_tokens,
                "input_tokens": self.input_tokens, "max_input_tokens": self.max_input_tokens}


class ConsoleCommands:
    """REPL commands that bypass the model (T18, T40): the operator can stop,
    e-stop, bring Spot out and read its state while the model is busy,
    throttled or gone. Commands that need a confirmation (/release) run in a
    background thread so the console's main thread can ask for it."""

    STOP = ("/stop", "stop")

    def __init__(self, tools, write, agent=None):
        self.tools = tools
        self.write = write
        self.agent = agent

    def handle(self, line):
        """Run ``line`` if it is a console command; True if it was one."""
        cmd = " ".join(str(line).strip().lower().split())
        tools = self.tools
        if cmd in self.STOP:
            if self.agent is not None:
                try:
                    self.agent.cancel()      # no further tool calls from this turn
                except Exception:
                    log.debug("agent.cancel() failed", exc_info=True)
            text = tools.console_stop("operator console stop")
        elif cmd in ("/estop", "/estop gentle"):
            text = tools.console_estop(hard=False)
        elif cmd == "/estop hard":
            text = tools.console_estop(hard=True)
        elif cmd == "/retreat":
            text = tools.start_retreat("operator console /retreat")
        elif cmd == "/status":
            text = tools.status_text()
        elif cmd == "/release":
            threading.Thread(target=self._release, name="console-release", daemon=True).start()
            text = "Asking for confirmation to release the e-stop..."
        elif cmd in ("/help", "help", "?"):
            text = CONSOLE_HELP
        else:
            return False
        self.write(text + "\n")
        return True

    def _release(self):
        result = self.tools.release_estop()
        self.write(("E-stop released.\n" if result.get("ok") else
                    "E-stop not released: %s\n" % result.get("error")))


def quit_refusal(tools):
    """Why ``quit`` must wait (Spot inside the permit space or a retreat
    running), or None."""
    if tools.retreating():
        return ("A retreat is running: wait for it to finish (/status), then quit. "
                "Ctrl-C twice cancels it and leaves Spot where it is.")
    if tools.robot_inside():
        return ("Spot is inside the permit space: type /retreat to bring it out (or /stop to "
                "stop it), then quit. Quitting now would leave it with no SafetyGuard.")
    return None


EXIT_WAIT_TEXT = ("(Exiting: waiting for the retreat to finish. /stop, /estop and /status still "
                  "work; instructions are no longer sent to the model.)\n")


def exit_safely(tools, reason, write=print, keep_monitoring=False, sleep=time.sleep,
                clock=time.monotonic, timeout_s=EXIT_RETREAT_WAIT_S, console=None,
                commands=None):
    """Before the agent exits: deny pending confirmations, halt mission
    motion, and finish a running retreat or start one if Spot is inside
    (T16: Ctrl-C, SIGTERM, a hang-up, end of input or the end of --once).
    With ``keep_monitoring`` (end of input) the process stays up, the
    SafetyGuard and heartbeat with it, while Spot is still inside. Returns
    True when Spot is outside. ``console`` keeps answering confirmations the
    retreat may ask for (a self-right inside) while this waits, and with
    ``commands`` (``ConsoleCommands``) it keeps reading the console commands
    too, so /stop, /estop and /status work during the wait (T18)."""
    tools.pending.deny_all(reason)
    tools.console_stop(reason)
    if not tools.retreating() and tools.robot_inside():
        write("Spot is inside the permit space: bringing it out before exiting (%s). Ctrl-C "
              "again cancels the retreat and leaves Spot where it is." % reason)
        write(tools.start_retreat("agent exiting: %s" % reason))
    deadline = clock() + timeout_s
    announced = False
    reading = console is not None and commands is not None
    while tools.retreating() and clock() < deadline:
        if not announced:
            write("Waiting for the retreat to finish before exiting. /stop, /estop and /status "
                  "still work; Ctrl-C again cancels the retreat.")
            announced = True
        if console is not None and console.service_pending():
            continue
        if not reading:
            sleep(0.25)
            continue
        line = console.poll(0.25)
        if line is not None and line.strip() and not commands.handle(line):
            console.write(EXIT_WAIT_TEXT)
    announced = False
    while keep_monitoring and tools.robot_inside():
        if not announced:
            write("Spot is still inside the permit space: the SafetyGuard keeps monitoring and "
                  "this process stays up. Stop it only after Spot has been recovered.")
            tools._emit("console", "Agent kept running: Spot is still inside the permit space "
                        "(%s)." % reason, {"reason": reason})
            announced = True
        sleep(1.0)
    return not tools.robot_inside()


def ask(agent, spot_tools, text, turn_timeout, console=None, commands=None, budget=None):
    """Run one operator instruction. Returns True on a normal finish.

    With ``budget`` (``SessionBudget``, T39) an instruction is refused,
    without calling the model, once the session's budget is spent: the
    console says so every time, and one ``session_budget`` record goes to
    the HUD and the mission JSONL.

    While the turn runs, the main thread (``console``) shows every pending
    physical action on the operator's terminal and reads y/N
    (agent/operator_console.py): the model never answers them. It also
    reads the console commands (``commands``: /stop, /retreat, /status, ...)
    so the operator can stop or bring Spot out without the model.

    The model turn runs in a daemon worker thread so the main thread stays
    responsive to Ctrl-C and can enforce ``turn_timeout``. Ctrl-C cancels
    the turn, stops the robot (a running retreat continues) and re-raises
    KeyboardInterrupt so the caller can exit safely (``exit_safely``).

    On a timeout the turn is cancelled, but Strands' ``Agent.cancel()`` does
    not interrupt a tool that is already running (strands/agent/agent.py:
    it stops "before tool execution" / "after tool execution"). A mission
    walk_to/run keeps sending cmd_vel at 20 Hz, so a halt now would be
    overridden within 50 ms, and a new instruction would hit Strands'
    ConcurrencyException. So wait (up to CANCEL_GRACE_S; /stop and Ctrl-C
    still stop the robot at once) for the in-flight tool to return. If it
    has not returned by then, the mission's motion is halted (T40) and the
    model session is marked unusable: console commands keep working.
    """
    spot_tools.operator(text)
    if budget is not None:
        budget.refresh(agent)
        spent = budget.exhausted()
        if spent:
            message = SESSION_BUDGET_TEXT % spent
            if not budget.reported:
                budget.reported = True
                log.error("%s", message)
                spot_tools._emit("session_budget", message, budget.snapshot())
                spot_tools._audit("session_budget", message, budget.snapshot())
            print(message)
            return False
    limits = budget.limits() if budget is not None else {"turns": MAX_TURNS}
    try:
        notice = spot_tools.pending_incident_prompt()
    except Exception:
        log.warning("pending incident check failed", exc_info=True)
        notice = None
    prompt = text if notice is None else "%s\n\nOperator instruction: %s" % (notice, text)
    box = {}

    def work():
        try:
            box["result"] = agent(prompt, limits=limits)
        except Exception as exc:  # Bedrock/network/SDK errors end the turn, not the process
            box["error"] = exc

    worker = threading.Thread(target=work, name="spot-agent-turn", daemon=True)
    worker.start()
    deadline = time.monotonic() + turn_timeout
    timed_out = stuck = False
    try:
        while worker.is_alive():
            if console is not None and console.service_pending():
                continue
            line = console.poll(0.25) if console is not None else worker.join(0.25)
            if line is not None and console is not None:
                if commands is None or not commands.handle(line):
                    console.write(BUSY_TEXT)
            if not worker.is_alive():
                break
            now = time.monotonic()
            if not timed_out and now > deadline:
                timed_out = True
                deadline = now + CANCEL_GRACE_S
                log.error("instruction timed out after %g s; cancelling", turn_timeout)
                agent.cancel()
                spot_tools._emit("error", "Instruction timed out after %g s; cancelled after the "
                                 "current robot action." % turn_timeout)
                print("\nInstruction timed out: waiting for the current robot action to finish "
                      "(/stop stops the robot now).")
            elif timed_out and now > deadline:
                log.error("the cancelled turn is still running after %g s", CANCEL_GRACE_S)
                stuck = True
                break
    except KeyboardInterrupt:
        agent.cancel()
        spot_tools.console_stop("operator pressed Ctrl-C")
        _flush_transcript(agent)
        raise
    _flush_transcript(agent)
    if budget is not None and not stuck:
        budget.observe(agent, box.get("result"), limits["turns"])
    if stuck:
        # A tool is still running: stop the mission's motion loops so it
        # cannot keep driving, and stop sending instructions to the model.
        spot_tools.console_stop("a model turn did not finish")
        spot_tools.session_unusable = True
        spot_tools._emit("error", "Model unavailable: an instruction did not finish; mission "
                         "motion halted. Console commands still work.")
        print(UNUSABLE_TEXT)
        return False
    if timed_out:
        spot_tools.halt_motion("instruction timed out")
        return False
    print()
    if "error" in box:
        exc = box["error"]
        log.error("agent turn failed: %s: %s", type(exc).__name__, exc)
        spot_tools.halt_motion("agent error")
        spot_tools._emit("error", "Agent error: %s: %s" % (type(exc).__name__, exc))
        print("Agent error: %s: %s" % (type(exc).__name__, exc))
        return False
    stop_reason = getattr(box.get("result"), "stop_reason", None)
    if stop_reason not in (None, "end_turn"):
        log.warning("agent turn ended with stop_reason=%s", stop_reason)
    return True


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Natural-language operator agent for the simulated Spot confined-space demo")
    parser.add_argument("--once", metavar="INSTRUCTION", help="run one instruction, then exit")
    parser.add_argument("--dry-run", action="store_true",
                        help="use FakeRobot/FakeMission from tests (no ROS); still calls Bedrock")
    parser.add_argument("--model", default=DEFAULT_MODEL_ID,
                        help="Bedrock model / inference-profile id (default: the pinned, tested "
                             "%s; any other is logged as untested)" % DEFAULT_MODEL_ID)
    parser.add_argument("--guardrail-id", default=None,
                        help="Amazon Bedrock guardrail id (default: %s; needs "
                             "--guardrail-version and bedrock:ApplyGuardrail)" % GUARDRAIL_ID_ENV)
    parser.add_argument("--guardrail-version", default=None,
                        help="version of --guardrail-id, e.g. 1 or DRAFT (default: %s)"
                             % GUARDRAIL_VERSION_ENV)
    parser.add_argument("--no-guardrail", action="store_true",
                        help="run without a Bedrock guardrail: the agent otherwise refuses to "
                             "start with none configured. Warns loudly and is recorded in the "
                             "session record")
    parser.add_argument("--region", default=DEFAULT_REGION, help="AWS region for Bedrock")
    parser.add_argument("--namespace", default=DEFAULT_NAMESPACE, help="robot ROS namespace")
    parser.add_argument("--log-path", default=None,
                        help="mission JSONL audit log (default logs/mission-<utc>.jsonl in the "
                             "mounted logs directory; the agent does not start without one)")
    parser.add_argument("--connect-timeout", type=float, default=30.0,
                        help="seconds to wait for robot telemetry")
    parser.add_argument("--turn-timeout", type=float, default=900.0,
                        help="max seconds per instruction (a full entry takes minutes)")
    parser.add_argument("--session-max-turns", type=int, default=SESSION_MAX_TURNS,
                        help="model turns for the whole session (default %d); once used, "
                             "instructions are refused, console commands still work"
                             % SESSION_MAX_TURNS)
    parser.add_argument("--session-max-output-tokens", type=int,
                        default=SESSION_MAX_OUTPUT_TOKENS,
                        help="model output tokens for the whole session (default %d)"
                             % SESSION_MAX_OUTPUT_TOKENS)
    parser.add_argument("--session-max-input-tokens", type=int,
                        default=SESSION_MAX_INPUT_TOKENS,
                        help="model input (prompt) tokens for the whole session, cached ones "
                             "included (default %d)" % SESSION_MAX_INPUT_TOKENS)
    parser.add_argument("--verbose", action="store_true", help="debug logging")
    parser.add_argument("--rehearsal-auto-confirm", action="store_true",
                        help="AUTOMATED REHEARSAL ONLY: answer every operator confirmation with "
                             "yes (simulator backend only; each one is logged as auto-confirmed)")
    parser.add_argument("--unattributed-run", action="store_true",
                        help="start although the login that started this session is a shared "
                             "account (ssm-user, ubuntu, root) or unknown; the session record "
                             "and mission_start then record the run as unattributed (T38)")
    return parser


def _make_backend(args):
    """(robot, mission, site) for the real ROS stack or the dry-run fakes; None on failure."""
    if args.dry_run:
        return _make_dry_run_backend(args)
    try:
        from mission import ConfinedSpaceMission
        from robot_client import RobotClient
    except ImportError as exc:
        log.error("cannot import the ROS robot stack (%s); run inside the ros:jazzy "
                  "container or use --dry-run", exc)
        return None
    robot = RobotClient(namespace=args.namespace)
    if not robot.connect(timeout=args.connect_timeout):
        log.error("no telemetry from %s within %.0f s: is the sim running and is "
                  "ROS_DOMAIN_ID/FASTDDS transport matching?", args.namespace, args.connect_timeout)
        robot.close()
        return None
    site = robot.site()  # latched topic, or agent/site_defaults.json after ~10 s
    try:
        mission = ConfinedSpaceMission(robot, site, args.log_path, require_permit=True,
                                       new_log=getattr(args, "new_log", False))
    except (OSError, ValueError) as exc:
        # Fail closed (T38): no writable audit log, or not the reviewed site.
        log.error("cannot start the mission (%s); the agent does not start", exc)
        robot.close()
        return None
    return robot, mission, site


def _make_dry_run_backend(args):
    """The real mission + SafetyGuard on the kinematic FakeRobot from
    tests/test_mission.py, in real time, with the sim's gas model
    (sim/gas_field.py GasField + unventilated headspace) or, if that is not
    importable, the tests' look-alike field. No ROS, no Isaac Sim."""
    for path in (REPO_ROOT, os.path.join(REPO_ROOT, "sim")):
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        from mission import ConfinedSpaceMission, RealClock
        from tests.test_mission import FakeRobot as KinematicRobot, unventilated_tank
    except ImportError as exc:
        log.error("--dry-run needs tests/test_mission.py next to agent/: %s", exc)
        return None
    field = None
    try:
        import gas_field
        if hasattr(gas_field, "UNVENTILATED_HEADSPACE"):
            field = gas_field.GasField(seed=7, headspace=gas_field.UNVENTILATED_HEADSPACE)
    except ImportError:
        field = None
    robot = KinematicRobot(RealClock(), standing=False, probe_enabled=True,
                           gas_fn3=None if field is not None else unventilated_tank, field=field)
    site = robot.site()
    print("DRY RUN: kinematic fake robot and %s (no ROS, no Isaac Sim); Bedrock is still called."
          % ("the sim's gas model" if field is not None else "a look-alike gas field"))
    try:
        mission = ConfinedSpaceMission(robot, site, args.log_path, require_permit=True,
                                       new_log=getattr(args, "new_log", False))
    except (OSError, ValueError) as exc:
        log.error("cannot start the mission (%s); the agent does not start", exc)
        return None
    return robot, mission, site


def session_data(args):
    """The agent's ``session`` record (mission JSONL): the model, who runs
    this session (T38: the account, the invoking login and an SSM session id
    when the launcher passes them; kept off payload/events) and whether the
    run is attributed (``attribution``: --unattributed-run, a shared login),
    the deployed commit (M30), the session's model budget (T39) and what the
    record is (``RECORD_NOTICE``: a demonstration record, not a permit
    record)."""
    identity = operator_identity()
    return {"model": args.model, "region": args.region, "dry_run": args.dry_run,
            "default_model": args.model == DEFAULT_MODEL_ID,
            "guardrail": ({"id": args.guardrail_id, "version": args.guardrail_version,
                           "source": getattr(args, "guardrail_source", None)}
                          if args.guardrail_id else {"disabled": True}),
            "operator": identity,
            "attribution": attribution(identity, getattr(args, "unattributed_run", False)),
            "rehearsal_auto_confirm": bool(args.rehearsal_auto_confirm),
            "session_budget": {"max_turns": getattr(args, "session_max_turns", None),
                               "max_output_tokens": getattr(args, "session_max_output_tokens",
                                                            None),
                               "max_input_tokens": getattr(args, "session_max_input_tokens",
                                                           None)},
            "version": deployed_version(), "record_notice": RECORD_NOTICE}


def await_simulator(tools, timeout_s=10.0, sleep=time.sleep, clock=time.monotonic):
    """Wait (bounded) until payload/status identifies the backend. Returns
    None on the simulator, once the SafetyGuard has been started, else the
    refusal text: the agent then does not start (no model session against
    anything but the simulator) and has sent the robot nothing, not even a
    stop (T15)."""
    deadline = clock() + max(0.0, float(timeout_s))
    while True:
        identity = tools.backend()
        if identity.get("backend") != BACKEND_UNKNOWN or clock() >= deadline:
            refusal = backend_refusal(identity, "starting the agent")
            if refusal is None:
                tools.start_guard()
            return refusal
        sleep(0.2)


def _make_console(pending):
    """The operator console on this terminal: select-based line input on
    stdin's file descriptor, or blocking input() where stdin has none."""
    try:
        reader = LineReader(sys.stdin.fileno())
    except (AttributeError, OSError, ValueError):
        return Console(pending, lambda _timeout=None: input(), _console_write, blocking=True)
    return Console(pending, reader.readline, _console_write, drain=reader.drain)


def _console_write(text):
    sys.stdout.write(text)
    sys.stdout.flush()


_SIGNALS = []   # names of the signals received, newest last


def _sigterm_to_interrupt(signum, _frame):
    # docker stop sends SIGTERM and a dropped SSM session SIGHUP: take the
    # same safe path as Ctrl-C (exit_safely brings Spot out first).
    try:
        _SIGNALS.append(signal.Signals(signum).name)
    except ValueError:
        _SIGNALS.append(str(signum))
    raise KeyboardInterrupt


def _sigint(signum, frame):
    _SIGNALS.append("SIGINT")
    raise KeyboardInterrupt


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", stream=sys.stderr)
    if not args.verbose:
        for noisy in ("botocore", "urllib3", "strands"):
            logging.getLogger(noisy).setLevel(logging.WARNING)
        if not args.once:
            quiet_guard_console()   # REPL: keep the 5 s guard lines off the prompt
    clean_console_log()   # after the quiet filter, which reads the record's arguments
    # Ctrl-C must always raise KeyboardInterrupt (a launcher may have left
    # SIGINT ignored); docker stop's SIGTERM and a hang-up (SSM session
    # loss) take the same safe path: Spot is brought out before the exit.
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigterm_to_interrupt)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _sigterm_to_interrupt)
    try:
        # The guardrail is on by default: resolved from the flags or the
        # environment before anything connects, and the agent does not start
        # with none unless --no-guardrail says so (T11, M11).
        (args.guardrail_id, args.guardrail_version,
         args.guardrail_source) = resolve_guardrail(args)
        model_settings(args.model, args.guardrail_id, args.guardrail_version)
        budget = SessionBudget(args.session_max_turns, args.session_max_output_tokens,
                               args.session_max_input_tokens)
    except ValueError as exc:
        log.error("%s", exc)
        return 2
    if args.guardrail_source == "disabled":
        log.warning("%s", NO_GUARDRAIL_WARNING)
    # No unattributed record without saying so (T38): refused before anything
    # connects or calls the model, unless the operator passed --unattributed-run.
    refusal = attribution_refusal(operator_identity(), args.unattributed_run, "The agent")
    if refusal:
        log.error("%s", refusal)
        return 2
    if args.unattributed_run:
        log.warning("UNATTRIBUTED RUN (--unattributed-run): the session record and mission_start "
                    "record that this session does not name its operator")
    # The default log is this session's own new file, created exclusively (T38).
    args.new_log = args.log_path is None
    if args.log_path is None:
        args.log_path = default_log_path("mission")
    if not args.log_path:
        log.error("an audit log is required (--log-path PATH); the agent does not start")
        return 2

    backend = _make_backend(args)
    if backend is None:
        return 2
    robot, mission, site = backend
    # The chain head on this console, and with it in the Session Manager
    # session log off the host (T38); the attribution goes into mission_start.
    mission.on_audit_head = lambda line: _console_write(line + "\n")
    mission.unattributed_run = bool(args.unattributed_run)
    tools = SpotTools(robot, mission, site)
    refusal = await_simulator(tools, timeout_s=min(args.connect_timeout, 10.0))
    if refusal:
        log.error("%s", refusal)
        print(refusal)
        tools._emit("session", "Agent did not start: %s" % refusal, {"backend": tools.backend()})
        for fn in (getattr(mission, "close", None), robot.close):
            if callable(fn):
                try:
                    fn()
                except Exception:
                    log.exception("close failed")
        return 2
    if args.rehearsal_auto_confirm:
        log.warning("REHEARSAL AUTO-CONFIRM: every operator confirmation is answered yes and "
                    "logged as auto-confirmed (simulator only); never use it for a demo")
        tools.pending.auto_confirm = True
    console = _make_console(tools.pending)
    tools.console_note = lambda text: _console_write("%s%s\n" % (READINGS_PREFIX, text))
    tools.console_alert = lambda text: _console_write("\n%s%s\n" % (ATTENTION_PREFIX, text))
    commands = ConsoleCommands(tools, _console_write)
    console.on_command = commands.handle
    agent = None
    exit_code = 0
    try:
        try:
            agent = build_agent(tools, args.model, args.region,
                                guardrail_id=args.guardrail_id,
                                guardrail_version=args.guardrail_version)
        except Exception as exc:
            log.error("could not build the agent (%s: %s); is strands-agents installed?",
                      type(exc).__name__, exc)
            return 2
        mode = ", dry run" if args.dry_run else ""
        tools._emit("session", "Agent online (%s%s)." % (args.model, mode),
                    {"model": args.model, "region": args.region, "dry_run": args.dry_run,
                     # Whether the guardrail is on, not which one: the HUD's
                     # events carry no identifier that is not already public.
                     "guardrail": bool(args.guardrail_id)})
        tools._audit("session", "Agent online (%s%s)." % (args.model, mode), session_data(args))
        commands.agent = agent
        if args.once:
            exit_code = 0 if ask(agent, tools, args.once, args.turn_timeout, console,
                                 commands, budget) else 1
            exit_safely(tools, "end of the one-shot instruction", console=console,
                        commands=commands)
        else:
            print("Spot confined-space agent (%s). Type an instruction; /help lists the console "
                  "commands; 'quit' or Ctrl-D to exit." % args.model)
            eof = False
            while True:
                try:
                    line = console.prompt("operator> ").strip()
                except EOFError:
                    # SSM session lost or input closed: nobody is at the console.
                    exit_safely(tools, "end of input on the operator console", keep_monitoring=True,
                                console=console, commands=commands)
                    eof = True
                    break
                if line.lower() in ("quit", "exit"):
                    refusal = quit_refusal(tools)
                    if refusal:
                        print(refusal)
                        continue
                    break
                if commands.handle(line) or not line:
                    continue
                if tools.session_unusable:
                    print(UNUSABLE_TEXT)
                    continue
                ask(agent, tools, line, args.turn_timeout, console, commands, budget)
            if not eof:
                exit_safely(tools, "operator quit", console=console, commands=commands)
    except KeyboardInterrupt:
        reason = _SIGNALS[-1] if _SIGNALS else "interrupt"
        print("\nInterrupted (%s): stopping the model and the robot." % reason)
        if agent is not None:
            try:
                agent.cancel()
            except Exception:
                log.debug("agent.cancel() failed", exc_info=True)
        try:
            exit_safely(tools, reason, console=console, commands=commands)
        except KeyboardInterrupt:
            # The operator's explicit choice: everything stops, Spot stays put.
            print("\nSecond interrupt: cancelling the retreat; Spot stays where it is.")
            cancel = getattr(mission, "cancel", None)
            if callable(cancel):
                try:
                    cancel("operator interrupted the agent twice")
                except Exception:
                    log.exception("mission.cancel() failed")
        exit_code = 130
    finally:
        tools.pending.deny_all("agent exiting")
        tools.halt_motion("agent exiting", force=True)
        tools._emit("session", "Agent offline.")
        for name, fn in (("mission.close", getattr(mission, "close", None)),
                         ("robot.close", robot.close)):
            if callable(fn):
                try:
                    fn()
                except Exception:
                    log.exception("%s() failed", name)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
