"""Operator-console confirmations for physical actions (threat model option D).

What this is
------------
The rule: a confirmation that releases a physical action never comes from
the model's arguments. A tool that wants one (permit attestations, entry
into the permit space, a model-initiated hard e-stop inside the space,
``self_right`` inside the space, e-stop release) registers a
``PendingAction`` (single-use id, ~60 s expiry) with ``PendingActions`` and
waits. The REPL's main thread, not the model, prints the pending action on
the operator's terminal and reads ``y``/``N`` (``Console.service_pending``);
anything but ``y``/``yes``, no answer, or the expiry is a denial. The typed
answer, the time and the operator (``$USER``, the login that started the
session before ``sudo -iu``, and an SSM session id, when the environment
carries them) form the record that the caller writes to the mission JSONL. Stopping never waits for a confirmation.

The agent's own requests are bounded (confirmation fatigue, OWASP ASI09):
an action the operator refused or let expire is not asked again for
CONFIRM_RETRY_S, at most CONFIRM_MAX_PENDING questions wait at once
(``throttle``; a request over either limit is decided ``not_asked`` without
being shown), and model-chosen text inside a question is quoted on one line,
length-bounded, with the console's own line markers made inert
(``quote_agent_text``).

``--rehearsal-auto-confirm`` (agent and scripted mission) answers every
pending action itself, for automated rehearsals: callers allow it only once
the simulator backend is identified, and every auto-confirmation is
recorded as such (``mode: "rehearsal_auto_confirm"``) and logged at WARNING.

Attribution (T38): the agent and ``run_mission.py`` do not start when the
invoking login is a shared account (``SHARED_LOGINS``) or unknown, so a
mission record never names only a shared account without saying so
(``attribution_refusal``); ``--unattributed-run`` starts anyway and the
session and mission-start records say the run is unattributed
(``attribution``). This is not authentication: the login is whatever the
environment says.

Pure Python (no ROS, no Strands): tests/test_hardening_agent.py drives it
with an injected reader.
"""

import logging
import os
import re
import secrets
import select
import threading
import time
import unicodedata

LOG = logging.getLogger("operator_console")

CONFIRM_TTL_S = 60.0
YES_ANSWERS = ("y", "yes")
ANSWER_MAX = 40
# Confirmation fatigue (OWASP ASI09): the agent's requests are rate-limited
# (``PendingActions(retry_after_s=..., max_pending=...)``). An action the
# operator refused or let expire is not asked again for CONFIRM_RETRY_S, and
# no more than CONFIRM_MAX_PENDING questions wait at once. Model-chosen text
# in a question is quoted on one line, at most AGENT_TEXT_MAX characters.
CONFIRM_RETRY_S = 30.0
CONFIRM_MAX_PENDING = 2
AGENT_TEXT_MAX = 120

PENDING, CONFIRMED, DENIED, EXPIRED = "pending", "confirmed", "denied", "expired"
NOT_ASKED = "not_asked"        # rate-limited: decided without asking (ASI09)

# The console's own line prefixes (agent/spot_agent.py: main's notes and
# notices, TranscriptCallback's tool lines; format_request below). Model
# text never carries one (T10): every line of a model reply starts with
# AGENT_PREFIX, and neutralise_console_markers() makes the markers inside it
# inert, so a model line cannot read as one of the console's own.
AGENT_PREFIX = "agent> "
REASONING_PREFIX = "agent (reasoning)> "
READINGS_PREFIX = "[readings] "
ATTENTION_PREFIX = "[OPERATOR ATTENTION] "
TOOL_PREFIX = "[tool] "
# The audit log's chain head (agent/mission.py audit_head_line, T38): with
# Session Manager logging on, these lines reach CloudWatch Logs as they are
# printed, an off-host copy of the head that host users cannot rewrite.
AUDIT_HEAD_PREFIX = "[audit head] "
QUESTION_HEADER = "=== OPERATOR CONFIRMATION REQUIRED"
_CONSOLE_MARKER = re.compile(
    r"\[\s*(readings|operator\s+attention|tool|y\s*/\s*n|audit\s+head)\s*\]"   # [readings] ... [y/N]
    r"|(={2,})"                                                     # === question header ===
    r"|\b(operator|agent)(\s*\([^()\n]{0,24}\))?\s*>"                # operator> / agent> lines
    r"|(OPERATOR CONFIRMATION REQUIRED)", re.I)
MODE_CONSOLE = "console"
MODE_AUTO = "rehearsal_auto_confirm"
MODE_TEST = "responder"
MODE_LIMIT = "rate_limit"

# Environment variables that may carry the operator's identity. USER/LOGNAME
# are the account the agent runs as (on the documented procedure the shared
# ubuntu); the invoking login is the account that ran ``sudo -iu ubuntu``
# (sudo keeps it in SUDO_USER; host/run_agent.sh passes it as
# SPOT_OPERATOR_LOGIN): the operator's own with Session Manager "Run As"
# per-operator accounts, ssm-user by default (T38). The SSM variables only
# exist if the launcher passes them.
USER_ENV = ("USER", "LOGNAME")
LOGIN_ENV = ("SPOT_OPERATOR_LOGIN", "SUDO_USER")
SSM_ENV = ("SSM_SESSION_ID", "AWS_SSM_SESSION_ID")


def _printable(text, limit):
    """Bounded, printable one-line copy of ``text``: control and format
    characters dropped, any whitespace run becomes one space."""
    kept = []
    for ch in str(text):
        major = unicodedata.category(ch)[0]
        if major == "Z" or ch in "\t\n\r":
            kept.append(" ")
        elif major != "C":
            kept.append(ch)
    return " ".join("".join(kept).split())[:limit]


def neutralise_console_markers(text):
    """``text`` with the console's own line markers made inert (T10):
    "[readings]", "[OPERATOR ATTENTION]", "[tool]", "[y/N]" and "[audit
    head]" (T38) lose their brackets, a run of "=" becomes one,
    "operator>" and "agent>" lose the ">", and "OPERATOR CONFIRMATION
    REQUIRED" is lower-cased."""
    def inert(match):
        word, equals, who, aside, header = match.groups()
        if word is not None:
            return "(%s)" % word
        if equals is not None:
            return "="
        if who is not None:
            return "%s%s:" % (who, aside or "")
        return header.lower()

    return _CONSOLE_MARKER.sub(inert, str(text))


class InertConsoleMarkers(logging.Filter):
    """Make the console's own markers inert in every log line (T10, T38).

    Log lines reach the same terminal, and with Session Manager logging the same
    off-host session log, as the console's own lines, and they carry model and
    robot-side text (the agent logs every reply as "[say] ..."). Without this, such
    text could print an "[audit head]" line that ``mission.py --verify`` would read
    as a chain head, or a line that looks like a console notice."""

    def filter(self, record):
        try:
            message = record.getMessage()
        except Exception:
            return True
        record.msg, record.args = neutralise_console_markers(message), None
        return True


def inert_console_markers(logger=None):
    """Attach InertConsoleMarkers to every handler of ``logger`` (default root)."""
    for handler in (logger or logging.getLogger()).handlers:
        if not any(isinstance(f, InertConsoleMarkers) for f in handler.filters):
            handler.addFilter(InertConsoleMarkers())


def quote_agent_text(value, limit=AGENT_TEXT_MAX):
    """Model-chosen text (a permit number, a rescue plan) for a console
    question (OWASP ASI09): one line, at most ``limit`` characters, the
    console's line markers made inert and double quotes turned into single
    ones, inside double quotes, so it can neither look like the console's
    own question nor close its quotes."""
    text = neutralise_console_markers(_printable(value, 4 * int(limit))).replace('"', "'")
    if len(text) > limit:
        text = text[:max(0, int(limit) - 3)].rstrip() + "..."
    return '"%s"' % text


def operator_identity(env=None):
    """{"user", "login", "ssm_session"} from the environment (None when
    absent): the account, the invoking login (T38) and the SSM session id.
    Callers keep it in the mission JSONL only (``without_operator_identity``
    strips it from payload/events)."""
    env = os.environ if env is None else env

    def first(names):
        for name in names:
            value = env.get(name)
            if value:
                return _printable(value, 64) or None
        return None

    return {"user": first(USER_ENV), "login": first(LOGIN_ENV), "ssm_session": first(SSM_ENV)}


# Logins that name no person (T38): Session Manager's default account, the
# demo account every operator works as, and root. An invoking login that is
# one of these, or none at all, makes the record unattributed.
SHARED_LOGINS = ("ssm-user", "ubuntu", "root")
UNATTRIBUTED_FLAG = "--unattributed-run"


def shared_login(identity):
    """True when ``identity`` (``operator_identity()``) has no invoking
    login, or one of SHARED_LOGINS (T38)."""
    login = (identity or {}).get("login")
    return not login or str(login).strip().lower() in SHARED_LOGINS


def attribution(identity, unattributed_run=False):
    """The ``attribution`` of a session or mission-start record (T38):
    ``attributed`` only when the invoking login names an operator and the
    operator did not pass --unattributed-run. Booleans only: the login
    itself stays in ``operator``, which payload/events never carries."""
    shared = shared_login(identity)
    flag = bool(unattributed_run)
    return {"attributed": not (shared or flag), "unattributed_run": flag, "shared_login": shared}


def attribution_refusal(identity, unattributed_run, program):
    """None, or the console text with which ``program`` refuses to start
    (T38): the invoking login is a shared account or unknown and the
    operator did not pass --unattributed-run. Not authentication: a refusal
    to write an unattributed record without saying so."""
    if unattributed_run or not shared_login(identity):
        return None
    login = (identity or {}).get("login")
    who = ("the shared account %r" % str(login).strip().lower() if login
           else "not known (no SUDO_USER or SPOT_OPERATOR_LOGIN in the environment)")
    return ("%s did not start: the login that started this session is %s, so the mission record "
            "would not say who ran it (T38). Connect to the host as your own account (Session "
            "Manager Run As: your IAM role or user carries the SSMSessionRunAs tag naming your OS "
            "account, which an administrator creates with host/add_operator.sh), run "
            "'sudo -iu ubuntu', and start it again from that shell. To run anyway and record the "
            "run as unattributed, pass %s." % (program, who, UNATTRIBUTED_FLAG))


def _wall():
    return round(time.time(), 3)


class PendingAction:
    """One physical action waiting for the operator (see module docstring)."""

    def __init__(self, action, summary, details, ttl_s, now):
        self.id = secrets.token_hex(4)
        self.action = str(action)
        self.summary = str(summary)
        self.details = details
        self.requested = now
        self.requested_wall = _wall()
        self.expires_at = now + float(ttl_s)
        self.status = PENDING
        self.answer = None
        self.answered_wall = None
        self.mode = None
        self.operator = None
        self.prompted = False
        self.reason = None       # why it was decided without asking (``decline``)

    def record(self):
        """The audit record (mission JSONL ``operator_confirmation`` data)."""
        out = {"action_id": self.id, "action": self.action, "summary": self.summary,
               "decision": self.status, "answer": self.answer, "mode": self.mode,
               "operator": self.operator, "requested_wall": self.requested_wall,
               "answered_wall": self.answered_wall}
        if self.reason is not None:
            out["reason"] = self.reason
        return out


class PendingActions:
    """Thread-safe registry of pending physical actions.

    ``responder`` (callable(item) -> answer text) answers synchronously in
    ``wait()``: tests use it; ``auto_confirm`` is ``--rehearsal-auto-confirm``.
    ``retry_after_s`` and ``max_pending`` (None: no limit) are what
    ``throttle()`` enforces for a caller that asks on someone else's behalf
    (the agent's model, OWASP ASI09).
    """

    def __init__(self, ttl_s=CONFIRM_TTL_S, clock=time.monotonic, auto_confirm=False, env=None,
                 retry_after_s=None, max_pending=None):
        self.ttl_s = float(ttl_s)
        self._clock = clock
        self._env = env
        self.auto_confirm = bool(auto_confirm)
        self.responder = None
        self.retry_after_s = retry_after_s
        self.max_pending = max_pending
        self._cond = threading.Condition()
        self._items = {}
        self._refused_at = {}    # action -> when the operator last refused it or let it expire

    def request(self, action, summary, details=None):
        with self._cond:
            item = PendingAction(action, summary, details, self.ttl_s, self._clock())
            self._items[item.id] = item
            self._cond.notify_all()
            return item

    def _decide_locked(self, item, answer, mode, refusal=True):
        if item.status != PENDING:
            return item
        if self._clock() > item.expires_at:
            item.status = EXPIRED
        else:
            text = _printable("" if answer is None else answer, ANSWER_MAX)
            item.answer = text
            item.status = CONFIRMED if text.lower() in YES_ANSWERS else DENIED
        item.mode = mode
        item.answered_wall = _wall()
        item.operator = operator_identity(self._env)
        if refusal and item.status in (DENIED, EXPIRED):
            self._refused_at[item.action] = self._clock()
        self._items.pop(item.id, None)
        self._cond.notify_all()
        return item

    def throttle(self, action):
        """Why a new request for ``action`` is not to be asked (OWASP ASI09,
        confirmation fatigue), or None: ``max_pending`` questions are
        already waiting, or the operator refused this action (or let it
        expire) less than ``retry_after_s`` ago."""
        with self._cond:
            if self.max_pending is not None:
                waiting = sum(1 for i in self._items.values() if i.status == PENDING)
                if waiting >= self.max_pending:
                    return ("%d confirmation%s already waiting for the operator"
                            % (waiting, " is" if waiting == 1 else "s are"))
            when = self._refused_at.get(str(action))
            if self.retry_after_s is not None and when is not None:
                ago = self._clock() - when
                if ago < self.retry_after_s:
                    return ("the operator refused this request or let it expire %.0f s ago; it is "
                            "not asked again for another %.0f s"
                            % (ago, self.retry_after_s - ago))
        return None

    def decline(self, action, summary, details=None, reason="", status=NOT_ASKED,
                mode=MODE_LIMIT, answer=None):
        """An item decided at once without asking anyone: never pending and
        never shown on the console. ``status`` NOT_ASKED for a rate-limited
        request; the record carries ``reason``."""
        with self._cond:
            item = PendingAction(action, summary, details, 0.0, self._clock())
            item.status, item.mode, item.reason = status, mode, str(reason)
            item.answer = None if answer is None else _printable(answer, ANSWER_MAX)
            item.answered_wall = item.requested_wall
            item.operator = operator_identity(self._env)
            return item

    def answer(self, action_id, answer, mode=MODE_CONSOLE):
        """Decide ``action_id`` once (single use). Returns the item, or None
        for an unknown id or one already decided or expired."""
        with self._cond:
            item = self._items.get(action_id)
            if item is None:
                return None
            return self._decide_locked(item, answer, mode)

    def wait(self, item):
        """Block until ``item`` is decided or expires; returns its status."""
        if self.auto_confirm:
            LOG.warning("REHEARSAL AUTO-CONFIRM: %s (%s)", item.action, item.summary)
            with self._cond:
                self._decide_locked(item, "y", MODE_AUTO)
        elif callable(self.responder):
            text = self.responder(item)
            with self._cond:
                self._decide_locked(item, text, MODE_TEST)
        with self._cond:
            while item.status == PENDING:
                remaining = item.expires_at - self._clock()
                if remaining <= 0.0:
                    item.status = EXPIRED
                    item.answered_wall = _wall()
                    item.operator = operator_identity(self._env)
                    self._refused_at[item.action] = self._clock()
                    self._items.pop(item.id, None)
                    break
                self._cond.wait(min(remaining, 0.25))
            return item.status

    def next_unprompted(self):
        """The oldest pending action the console has not shown yet (marked
        shown), or None."""
        with self._cond:
            for item in sorted(self._items.values(), key=lambda i: i.requested):
                if not item.prompted and item.status == PENDING:
                    item.prompted = True
                    return item
            return None

    def pending(self):
        with self._cond:
            return [i for i in self._items.values() if i.status == PENDING]

    def deny_all(self, reason="console closed"):
        """Deny everything still pending (e.g. at exit or end of input). Not
        the operator's refusal: it does not hold back the next request."""
        with self._cond:
            for item in list(self._items.values()):
                self._decide_locked(item, reason, MODE_CONSOLE, refusal=False)


class LineReader:
    """Line input from a file descriptor with a timeout (select + os.read).

    One reader for every line the console takes - the instruction prompt,
    y/N answers and the /commands - so type-ahead is never split between
    Python's buffered stdin and a raw read. ``readline(timeout)`` returns
    the line without its newline, None when nothing arrived in time, and
    raises EOFError at end of input."""

    def __init__(self, fd=0):
        self.fd = fd
        self._buf = b""
        self._eof = False

    def readline(self, timeout=None):
        deadline = None if timeout is None else time.monotonic() + max(0.0, float(timeout))
        while b"\n" not in self._buf:
            if self._eof:
                if self._buf:
                    line, self._buf = self._buf, b""
                    return line.decode("utf-8", "replace")
                raise EOFError
            wait = None if deadline is None else max(0.0, deadline - time.monotonic())
            ready, _w, _x = select.select([self.fd], [], [], wait)
            if not ready:
                return None
            chunk = os.read(self.fd, 4096)
            if not chunk:
                self._eof = True
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8", "replace").rstrip("\r")

    def drain(self):
        """Discard everything typed so far (buffered or already readable), so
        a confirmation counts only an answer typed after its question.
        Returns the discarded complete lines (the console re-runs any that
        are console commands, e.g. a queued /stop)."""
        while not self._eof:
            ready, _w, _x = select.select([self.fd], [], [], 0)
            if not ready:
                break
            chunk = os.read(self.fd, 4096)
            if not chunk:
                self._eof = True
            self._buf += chunk
        lines = [ln.decode("utf-8", "replace").rstrip("\r") for ln in self._buf.split(b"\n")[:-1]]
        self._buf = b""
        return lines


def is_console_command(line):
    """A console command (/stop, /retreat, ...) rather than an answer."""
    text = str(line or "").strip()
    return text.startswith("/") or text.lower() == "stop"


def format_request(item):
    """The text the console prints for a pending action."""
    seconds = max(0, int(round(item.expires_at - time.monotonic())))
    return ("\n%s (%s, id %s) ===\n%s\n"
            "Type y to confirm; anything else, or no answer within %d s, refuses. [y/N] "
            % (QUESTION_HEADER, item.action, item.id, item.summary,
               seconds or int(CONFIRM_TTL_S)))


def ask_on_terminal(pending, item, readline, write, clock=time.monotonic, drain=None):
    """Print ``item`` and read the operator's answer (bounded by the item's
    expiry). Returns the decided item. ``readline(timeout)`` follows
    ``LineReader.readline``; EOF denies. ``drain()`` first discards any
    type-ahead, so a "y" typed before the question cannot answer it."""
    if callable(drain):
        drain()
    write(format_request(item))
    try:
        line = readline(max(0.0, item.expires_at - clock()))
    except EOFError:
        line = "end of input"
    decided = pending.answer(item.id, "" if line is None else line)
    if decided is None:          # expired meanwhile: wait() records it
        write("\nNot confirmed (expired).\n")
        return item
    write("Confirmed.\n" if decided.status == CONFIRMED else "Refused (%s).\n" % decided.status)
    return decided


def terminal_confirmer(readline, write, auto_confirm=False, allow_auto=None, ttl_s=CONFIRM_TTL_S,
                       env=None, drain=None):
    """``confirm(action, summary, details) -> (ok, record)`` for a single
    thread (the scripted mission): asks on the terminal right away.
    ``allow_auto()`` must be true for an auto-confirmation (the simulator
    backend identified); otherwise the request is denied."""
    pending = PendingActions(ttl_s=ttl_s, env=env)

    def confirm(action, summary, details=None):
        item = pending.request(action, summary, details)
        if auto_confirm:
            if callable(allow_auto) and allow_auto():
                pending.auto_confirm = True
                pending.wait(item)
                pending.auto_confirm = False
            else:
                pending.answer(item.id, "auto-confirm refused: backend not the simulator")
        else:
            ask_on_terminal(pending, item, readline, write, drain=drain)
            pending.wait(item)
        return item.status == CONFIRMED, item.record()

    return confirm


class Console:
    """The operator's terminal on the REPL's main thread: pending
    confirmations first, then the instruction prompt and, while a model
    turn runs, the console commands (``poll``). ``blocking`` marks a
    ``readline`` that cannot time out (no select on this stdin): ``poll``
    then only waits."""

    def __init__(self, pending, readline, write, blocking=False, drain=None):
        self.pending = pending
        self.readline = readline
        self.write = write
        self.blocking = bool(blocking)
        self.drain = drain
        self.on_command = None   # callable(line): a console command typed as the answer
        self._eof = False

    def service_pending(self):
        """Show and decide the oldest unshown pending action. True if one
        was handled (the caller loops again at once)."""
        item = self.pending.next_unprompted()
        if item is None:
            return False
        # Type-ahead never answers the question; queued console commands
        # (a /stop typed a moment ago) still run.
        stale = self.drain() if callable(self.drain) else None
        for line in stale or []:
            if is_console_command(line) and callable(self.on_command):
                self.on_command(line.strip())
        decided = ask_on_terminal(self.pending, item, self.readline, self.write)
        answer = (decided.answer or "").strip()
        if callable(self.on_command) and is_console_command(answer):
            self.on_command(answer)   # e.g. /stop at the prompt: refused AND stopped
        return True

    def poll(self, timeout):
        """A line typed within ``timeout`` s, or None. End of input is
        remembered and raised by the next ``prompt``."""
        if self._eof or self.blocking:
            time.sleep(max(0.0, timeout))
            return None
        try:
            return self.readline(timeout)
        except EOFError:
            self._eof = True
            return None

    def prompt(self, text):
        """Print ``text`` and wait for one line, showing any pending action
        that arrives meanwhile (e.g. from a console /retreat); EOFError at
        end of input."""
        if self._eof:
            raise EOFError
        self.write(text)
        while True:
            if self.service_pending():
                self.write(text)
                continue
            try:
                line = self.readline(None if self.blocking else 0.25)
            except EOFError:
                self._eof = True
                raise
            if line is not None:
                return line
