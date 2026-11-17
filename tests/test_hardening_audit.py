"""Tests for the attributable, off-host-anchored audit trail (threat model T38, M38).

Every test names T38 in its docstring. What they cover:

* the agent and run_mission.py refuse to start on a shared or unknown operator
  login, unless --unattributed-run, which the session and mission-start records
  then carry (agent/operator_console.py ``attribution_refusal``);
* the chain head on the operator console (``[audit head]`` lines) and the check
  of a log against those lines (agent/mission.py ``verify_audit_log``, ``main``);
* host/add_operator.sh, run against a fake host (stand-ins for getent, useradd,
  usermod, userdel, passwd, visudo, chown and id first on PATH): no account,
  password or sudoers file of the machine running the tests is touched;
* the Session Manager preferences, the session-log template and the
  instance-role statements under docs/iam and infra, read as files.

Nothing here calls AWS, starts a container or listens on the network. Every
subprocess argv is a list of string literals; the variable parts go in the
environment.

Run from the repo root:

    python3 -m unittest tests.test_hardening_audit -v
"""

import contextlib
import hashlib
import io
import json
import logging
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

try:
    import yaml
except ImportError:          # the CI lock has PyYAML; a bare checkout may not
    yaml = None

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENT_DIR = os.path.join(REPO, "agent")
for _path in (REPO, AGENT_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from tests import test_agent_tools as ta  # noqa: E402
from tests import test_mission as tm  # noqa: E402

import mission as mission_mod  # noqa: E402
import operator_console  # noqa: E402
import robot_client  # noqa: E402
import run_mission  # noqa: E402
from agent import spot_agent  # noqa: E402

BASH = shutil.which("bash")
LOGIN_VARS = ("SPOT_OPERATOR_LOGIN", "SUDO_USER")


def _read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as handle:
        return handle.read()


def _flat(text):
    return " ".join(text.split())


@contextlib.contextmanager
def operator_login(login):
    """os.environ with exactly this invoking login (None: no login at all)."""
    with mock.patch.dict(os.environ):
        for name in LOGIN_VARS:
            os.environ.pop(name, None)
        if login is not None:
            os.environ["SPOT_OPERATOR_LOGIN"] = login
        yield


def line_hashes(path):
    """SHA-256 of each record (non-blank line) of a JSONL, in order."""
    with open(path, "r", encoding="utf-8") as handle:
        return [hashlib.sha256(line.rstrip("\n").encode("utf-8")).hexdigest()
                for line in handle if line.strip()]


def heads_of(lines):
    """[(records, sha256, after)] from console text."""
    found = re.findall(r"^\[audit head\] \S+ (\d+):([0-9a-f]{64}) after ([a-z ]+?)(?: \(|$)",
                       lines, re.M)
    return [(int(n), sha, after) for n, sha, after in found]


class MissionCase(unittest.TestCase):
    """The real mission and SafetyGuard on the kinematic fake (scaled clock)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log_path = os.path.join(self.tmp.name, "mission-t38.jsonl")
        self.clock = tm.ScaledClock(20.0)
        self.missions = []

    def tearDown(self):
        for m in self.missions:
            if m._retreating:
                m.cancel("test teardown")
            m.close()

    def make(self, robot, **kwargs):
        kwargs.setdefault("require_permit", True)
        m = mission_mod.ConfinedSpaceMission(robot, robot.site(), self.log_path,
                                             clock=self.clock, **kwargs)
        self.missions.append(m)
        return m

    def scripted(self, *extra, login="op-alice"):
        """run_mission.execute() on the kinematic fake; (exit code, console text, robot)."""
        robot = tm.FakeRobot(self.clock, probe_enabled=True)
        args = run_mission.build_parser().parse_args(
            ["--log", self.log_path, "--dwell", "3", "--rehearsal-auto-confirm"] + list(extra))
        out = []
        with operator_login(login), contextlib.redirect_stdout(io.StringIO()):
            code = run_mission.execute(args, robot, clock=self.clock, write=out.append)
        return code, "".join(out), robot


# ---------------------------------------------------------------------------
# Fail closed on a shared identity
# ---------------------------------------------------------------------------


class TestT38SharedLoginIsRefused(MissionCase):
    def test_shared_or_missing_logins_are_unattributed(self):
        """T38 (M38): the invoking login is shared when it is ssm-user,
        ubuntu, root (any case) or missing; the operator's own is not, and
        add_operator.sh refuses exactly the same shared names."""
        for login in ("ssm-user", "ubuntu", "root", "SSM-User", " ubuntu ", "", None):
            self.assertTrue(operator_console.shared_login({"login": login}), login)
        self.assertTrue(operator_console.shared_login(None))
        self.assertFalse(operator_console.shared_login({"login": "op-alice"}))
        shared = re.search(r'^SHARED="([^"]*)"', _read("host", "add_operator.sh"), re.M).group(1)
        self.assertEqual(sorted(shared.split()), sorted(operator_console.SHARED_LOGINS))

    def test_the_refusal_says_how_to_connect(self):
        """T38 (M38): the refusal names the shared account (never a person),
        says to connect as your own account and run sudo -iu ubuntu, and
        names --unattributed-run; with the flag, or an own login, there is
        none."""
        text = operator_console.attribution_refusal({"login": "ssm-user"}, False, "The agent")
        flat = _flat(text)
        for part in ("The agent did not start", "'ssm-user'", "your own account",
                     "SSMSessionRunAs", "host/add_operator.sh", "sudo -iu ubuntu",
                     "--unattributed-run"):
            self.assertIn(part, flat)
        self.assertIn("not known", operator_console.attribution_refusal({}, False, "x"))
        self.assertIsNone(operator_console.attribution_refusal({"login": "ssm-user"}, True, "x"))
        self.assertIsNone(operator_console.attribution_refusal({"login": "op-alice"}, False, "x"))

    def test_run_mission_refuses_before_it_connects(self):
        """T38 (M38): run_mission.py with a shared or missing login exits 1
        with the refusal before any robot connection; an own login, or
        --unattributed-run, goes on to connect."""
        cases = (("ssm-user", [], False), (None, [], False), ("root", [], False),
                 ("op-alice", [], True), ("ssm-user", ["--unattributed-run"], True))
        for login, argv, connects in cases:
            with self.subTest(login=login, argv=argv), operator_login(login), \
                    mock.patch.object(run_mission.signal, "signal"), \
                    mock.patch.object(run_mission.logging, "basicConfig"), \
                    mock.patch.object(robot_client.RobotClient, "connect",
                                      return_value=False) as connect, \
                    self.assertLogs("run_mission", level="ERROR") as logged:
                self.assertEqual(run_mission.main(argv), run_mission.EXIT_FAILURE)
            self.assertEqual(connect.called, connects)
            refused = any("did not start" in line for line in logged.output)
            self.assertEqual(refused, not connects, logged.output)

    def test_the_agent_refuses_before_any_backend_or_model(self):
        """T38 (M38): spot_agent.main with a shared or missing login exits 2
        with the refusal before the robot link or the model is set up; an
        own login, or --unattributed-run, gets past the check."""
        cases = (("ubuntu", [], False), (None, [], False), ("op-alice", [], True),
                 ("ssm-user", ["--unattributed-run"], True))
        for login, argv, past in cases:
            with self.subTest(login=login, argv=argv), operator_login(login), \
                    mock.patch.object(spot_agent.signal, "signal"), \
                    mock.patch.object(spot_agent.logging, "basicConfig"), \
                    mock.patch.object(spot_agent, "quiet_guard_console"), \
                    mock.patch.object(spot_agent, "clean_console_log"), \
                    mock.patch.object(spot_agent, "_make_backend", return_value=None) as backend, \
                    mock.patch.object(spot_agent, "build_agent") as build, \
                    self.assertLogs(spot_agent.log.name, level="ERROR") as logged:
                spot_agent.log.error("marker")       # assertLogs needs one record either way
                # --no-guardrail: the guardrail gate comes first, and this tests the
                # attribution gate behind it (T11).
                self.assertEqual(
                    spot_agent.main(argv + ["--log-path", self.log_path, "--no-guardrail"]), 2)
            self.assertEqual(backend.called, past)
            build.assert_not_called()
            self.assertEqual(any("did not start" in line for line in logged.output), not past)

    def test_the_records_carry_the_attribution(self):
        """T38 (M38): mission_start and the agent's session record carry
        ``attribution``: attributed with an own login, unattributed (and
        why) with --unattributed-run on a shared login. It holds no login, so
        payload/events may carry it; the login itself never goes there."""
        code, _text, robot = self.scripted("--unattributed-run", login="ssm-user")
        self.assertEqual(code, run_mission.EXIT_COMPLETED)
        start = next(r for r in tm.read_jsonl(self.log_path) if r["kind"] == "mission_start")
        self.assertEqual(start["data"]["attribution"],
                         {"attributed": False, "unattributed_run": True, "shared_login": True})
        self.assertEqual(start["data"]["operator"]["login"], "ssm-user")
        published = next(e for e in robot.events if e["kind"] == "mission_start")
        self.assertEqual(published["data"]["attribution"], start["data"]["attribution"])
        self.assertNotIn("operator", published["data"])
        os.remove(self.log_path)
        self.assertEqual(self.scripted(login="op-alice")[0], run_mission.EXIT_COMPLETED)
        start = next(r for r in tm.read_jsonl(self.log_path) if r["kind"] == "mission_start")
        self.assertEqual(start["data"]["attribution"],
                         {"attributed": True, "unattributed_run": False, "shared_login": False})
        parser = spot_agent.build_arg_parser()
        with operator_login("ssm-user"):
            session = spot_agent.session_data(parser.parse_args(["--unattributed-run"]))
        self.assertEqual(session["attribution"],
                         {"attributed": False, "unattributed_run": True, "shared_login": True})
        with operator_login("op-alice"):
            session = spot_agent.session_data(parser.parse_args([]))
        self.assertTrue(session["attribution"]["attributed"])
        self.assertFalse(run_mission.build_parser().parse_args([]).unattributed_run)
        self.assertFalse(parser.parse_args([]).unattributed_run)


# ---------------------------------------------------------------------------
# The chain head on the operator console
# ---------------------------------------------------------------------------


class TestT38DefaultLogIsCreatedNew(MissionCase):
    """T38 (M38): the default log path is predictable to the second, and the logs
    directory is writable by the operators and the containers, so a run's default
    log is a file the run creates itself, exclusively. A file or link already at
    that path, which could carry a forged but correctly chained history that the
    run's heads would then vouch for, is never adopted: the run does not start."""

    def forged(self):
        prev, lines = mission_mod.GENESIS_SHA256, []
        for seq in range(1, 4):
            line = json.dumps({"seq": seq, "kind": "note", "message": "FORGED by a planter",
                               "prev_sha256": prev})
            lines.append(line)
            prev = hashlib.sha256(line.encode("utf-8")).hexdigest()
        return "".join(line + "\n" for line in lines)

    def default_run(self):
        """run_mission.execute() with no --log, its default path being self.log_path."""
        robot = tm.FakeRobot(self.clock, probe_enabled=True)
        args = run_mission.build_parser().parse_args(["--dwell", "3", "--rehearsal-auto-confirm"])
        with operator_login("op-alice"), contextlib.redirect_stdout(io.StringIO()), \
                mock.patch.object(run_mission, "default_log_path", return_value=self.log_path):
            return run_mission.execute(args, robot, clock=self.clock, write=lambda _: None)

    def test_a_file_planted_at_the_default_path_is_never_adopted(self):
        planted = self.forged()
        with open(self.log_path, "w", encoding="utf-8") as handle:
            handle.write(planted)
        with self.assertLogs("run_mission", level="ERROR"):
            self.assertEqual(self.default_run(), run_mission.EXIT_FAILURE)
        with open(self.log_path, encoding="utf-8") as handle:
            self.assertEqual(handle.read(), planted)        # nothing appended

    def test_a_link_planted_at_the_default_path_is_not_followed(self):
        target = os.path.join(self.tmp.name, "elsewhere.jsonl")
        with open(target, "w", encoding="utf-8") as handle:
            handle.write(self.forged())
        os.symlink(target, self.log_path)
        before = os.stat(target).st_size
        with self.assertLogs("run_mission", level="ERROR"):
            self.assertEqual(self.default_run(), run_mission.EXIT_FAILURE)
        self.assertEqual(os.stat(target).st_size, before)

    def test_a_new_default_log_starts_its_own_chain(self):
        self.assertEqual(self.default_run(), 0)
        with open(self.log_path, encoding="utf-8") as handle:
            first = json.loads(handle.readline())
        self.assertEqual(first["prev_sha256"], mission_mod.GENESIS_SHA256)
        self.assertTrue(mission_mod.verify_audit_log(self.log_path)["ok"])

    def test_a_log_the_operator_names_still_continues_its_chain(self):
        with open(self.log_path, "w", encoding="utf-8") as handle:
            handle.write(self.forged())
        with self.assertRaises(FileExistsError):
            self.make(tm.FakeRobot(self.clock), new_log=True)
        m = self.make(tm.FakeRobot(self.clock))              # an explicit path appends
        self.assertEqual(m.audit_head()["records"], 3)

    def test_both_programs_create_their_default_log_new(self):
        """The agent and the scripted mission pass new_log for their default path only."""
        for name in ("spot_agent.py", "run_mission.py"):
            source = _read("agent", name)
            self.assertIn("new_log", source, name)
        agent = _read("agent", "spot_agent.py")
        self.assertIn("args.new_log = args.log_path is None", agent)
        self.assertEqual(agent.count('new_log=getattr(args, "new_log", False)'), 2)
        self.assertIn("new_log = args.log is None", _read("agent", "run_mission.py"))


class TestT38ChainHeadOnTheConsole(MissionCase):
    def test_the_scripted_mission_prints_every_head(self):
        """T38 (M38): run_mission.py prints one [audit head] line after each
        operator confirmation, at mission start, at mission end and at
        session end (the log closed), each the record count and the SHA-256
        of the last record in the file at that moment."""
        code, text, _robot = self.scripted()
        self.assertEqual(code, run_mission.EXIT_COMPLETED)
        heads = heads_of(text)
        afters = [after for _n, _sha, after in heads]
        self.assertEqual(afters.count("operator confirmation"), 2)   # attestation, entry
        self.assertIn("mission start", afters)
        self.assertEqual(afters[-2:], ["mission end", "session end"])
        hashes = line_hashes(self.log_path)
        records = tm.read_jsonl(self.log_path)
        kind_of = {"operator confirmation": "operator_confirmation", "mission start":
                   "mission_start", "mission end": "summary"}
        for count, sha, after in heads:
            self.assertEqual(hashes[count - 1], sha, (count, after))
            if after in kind_of:
                self.assertEqual(records[count - 1]["kind"], kind_of[after], (count, after))
        self.assertEqual(heads[-1][:2], (len(hashes), hashes[-1]))
        for line in text.splitlines():
            if line.startswith(operator_console.AUDIT_HEAD_PREFIX):
                self.assertIn(os.path.basename(self.log_path), line)

    def test_a_run_appended_to_a_log_counts_the_whole_file(self):
        """T38 (M38): a run that appends to an existing log continues the
        chain, and its heads count every record in the file."""
        self.scripted()
        before = len(line_hashes(self.log_path))
        _code, text, _robot = self.scripted()
        heads = heads_of(text)
        self.assertGreater(heads[0][0], before)
        self.assertEqual(heads[-1][0], len(line_hashes(self.log_path)))
        self.assertTrue(mission_mod.verify_audit_log(
            self.log_path, ["%d:%s" % (n, sha) for n, sha, _a in heads])["ok"])

    def test_the_agent_prints_a_head_after_each_confirmation(self):
        """T38 (M38): the agent's confirmations (here the permit
        attestation) and its session record are followed by an [audit head]
        line, through the mission's on_audit_head hook."""
        robot = tm.FakeRobot(self.clock)
        m = self.make(robot)
        lines = []
        m.on_audit_head = lines.append
        tools = spot_agent.SpotTools(robot, m, robot.site(), env={}, sleep=self.clock.sleep,
                                     clock=self.clock.now)
        tools.pending.responder = lambda item: "y"
        tools._audit("session", "Agent online", {"operator": None})
        tools.prepare_permit(permit_id="CSE-0419", rescue_plan="Standby team",
                             request_attestation=True)
        heads = heads_of("\n".join(lines))
        self.assertEqual([a for _n, _s, a in heads], ["session start", "operator confirmation"])
        records = tm.read_jsonl(self.log_path)
        count, sha, _after = heads[-1]
        self.assertEqual(records[count - 1]["kind"], "operator_confirmation")
        self.assertEqual(line_hashes(self.log_path)[count - 1], sha)
        self.assertEqual(m.audit_head()["records"], len(records))

    def test_the_agent_prints_the_head_at_session_end(self):
        """T38 (M38): spot_agent.main hooks the console to the mission, so
        closing the log at the end of the session prints its head, even on
        the refusal path (here a backend that never identifies itself)."""
        robot = tm.FakeRobot(self.clock, start=(0.4, 0.3, 0.0), standing=False)
        robot.status = lambda: None        # like a real driver: never identified as the simulator
        robot.gas = lambda: None
        m = self.make(robot)
        written = []
        with operator_login("op-alice"), \
                mock.patch.object(spot_agent, "_make_backend",
                                  return_value=(robot, m, robot.site())), \
                mock.patch.object(spot_agent.signal, "signal"), \
                mock.patch.object(spot_agent.logging, "basicConfig"), \
                mock.patch.object(spot_agent, "_console_write", written.append), \
                ta._quiet_stdout():
            self.assertEqual(spot_agent.main(["--connect-timeout", "0.2", "--log-path",
                                              self.log_path, "--no-guardrail"]), 2)
        heads = heads_of("".join(written))
        self.assertEqual([a for _n, _s, a in heads], ["session end"])

    def test_a_failed_write_is_said_in_the_head(self):
        """T38 (M38): once the audit log cannot be written, the head line
        says so; its count and SHA-256 stay those of the last record on
        disk."""
        robot = tm.FakeRobot(self.clock)
        m = self.make(robot)
        lines = []
        m.on_audit_head = lines.append
        m.log_event("note", "written")
        on_disk = m.audit_head()

        def disk_full(_text):
            raise OSError(28, "No space left on device")

        m._log_file.write = disk_full
        m.log_event("operator_confirmation", "not written", {"decision": "confirmed"})
        self.assertIn("AUDIT LOG WRITE FAILED", lines[-1])
        self.assertIn("%d:%s" % (on_disk["records"], on_disk["sha256"]), lines[-1])

    def test_model_text_cannot_imitate_a_head_line(self):
        """T38 / T10 (M38): "[audit head]" is one of the console's own
        markers, so model text that contains it is made inert."""
        text = operator_console.neutralise_console_markers("[audit head] mission.jsonl 3:" + "a" * 64)
        self.assertFalse(text.startswith(operator_console.AUDIT_HEAD_PREFIX))
        self.assertEqual(mission_mod.audit_heads_in(text), [])
        self.assertTrue(operator_console.quote_agent_text("[ Audit  Head ] x").startswith('"(Audit'))

    def test_log_lines_cannot_carry_a_head_line_either(self):
        """T38 / T10 (M38): every model reply is also logged ("[say] <text>"), and
        mission events carry robot-side and presenter text into the log. Those lines
        reach the same terminal and session log, so both programs' log filters make
        the markers inert: no head can be read back from them."""
        forged = "[audit head] mission-20261030-101500.jsonl 9:%s after operator confirmation" % ("b" * 64)
        cases = (("spot_agent", spot_agent.CleanConsoleLog(), "[%s] %s", ("say", forged)),
                 ("mission", operator_console.InertConsoleMarkers(), "[%s] %s: %s",
                  ("presenter", "note", forged)))
        for name, log_filter, fmt, args in cases:
            with self.subTest(name):
                stream = io.StringIO()
                handler = logging.StreamHandler(stream)
                handler.setFormatter(logging.Formatter("%(asctime)s INFO %(name)s: %(message)s"))
                handler.addFilter(log_filter)
                logger = logging.getLogger("t38-log-filter-%s" % name)
                logger.addHandler(handler)
                logger.setLevel(logging.INFO)
                logger.propagate = False
                try:
                    logger.info(fmt, *args)
                finally:
                    logger.removeHandler(handler)
                output = stream.getvalue()
                self.assertIn("mission-20261030-101500.jsonl", output)
                self.assertEqual(mission_mod.audit_heads_in(output), [], output)
        self.assertIn("inert_console_markers()", _read("agent", "run_mission.py"))


# ---------------------------------------------------------------------------
# Checking a log against the session log
# ---------------------------------------------------------------------------


class TestT38VerifyAgainstTheSessionLog(MissionCase):
    RUN = None    # (console text, log lines) of one scripted run, shared by the tests

    def setUp(self):
        super().setUp()
        if TestT38VerifyAgainstTheSessionLog.RUN is None:
            _code, console, _robot = self.scripted()
            with open(self.log_path, encoding="utf-8") as handle:
                TestT38VerifyAgainstTheSessionLog.RUN = (console, handle.read().splitlines())
        self.console, self.lines = TestT38VerifyAgainstTheSessionLog.RUN
        self.heads = ["%d:%s" % (n, sha) for n, sha, _a in heads_of(self.console)]
        self.rewrite(self.lines)

    def rewrite(self, lines):
        with open(self.log_path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

    def kinds(self):
        return [f["kind"] for f in mission_mod.verify_audit_log(self.log_path,
                                                                self.heads)["findings"]]

    def test_an_intact_log_matches_every_head(self):
        """T38 (M38): the log as written matches every head the console
        printed, and reports its own head."""
        result = mission_mod.verify_audit_log(self.log_path, self.heads)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["head"], self.heads[-1])
        self.assertEqual(result["findings"], [])

    def test_truncation_is_reported(self):
        """T38 (M38): records removed from the end (the chain itself still
        holds) are reported as TRUNCATED against the later heads; so is a
        file replaced by a fresh, shorter chain."""
        self.rewrite(self.lines[:-3])
        self.assertTrue(mission_mod.verify_audit_log(self.log_path)["ok"])   # no head: unseen
        self.assertIn("truncated", self.kinds())
        os.remove(self.log_path)
        m = self.make(tm.FakeRobot(self.clock))
        m.log_event("note", "a fresh file")
        m.close()
        self.assertIn("truncated", self.kinds())

    def test_a_rewritten_record_is_reported(self):
        """T38 (M38): a record edited in place breaks the chain at the next
        line (REWRITTEN, with the line); the last record edited breaks no
        link but no longer matches the final head (MISMATCH)."""
        lines = list(self.lines)
        lines[4] = lines[4].replace('"kind": ', '"kind":  ')
        self.rewrite(lines)
        result = mission_mod.verify_audit_log(self.log_path, self.heads)
        self.assertEqual(result["findings"][0]["kind"], "rewritten")
        self.assertEqual(result["first_bad_line"], 6)
        lines = list(self.lines)
        lines[-1] = lines[-1].replace("COMPLETED", "ABORTED")
        self.rewrite(lines)
        self.assertEqual(self.kinds(), ["mismatch"])

    def test_a_rewritten_chain_is_a_mismatch(self):
        """T38 (M38): a record changed and every later prev_sha256
        recomputed passes the chain check alone, but not the heads printed
        after it (MISMATCH)."""
        records = [json.loads(line) for line in self.lines]
        records[10]["message"] = "rewritten"
        prev, lines = mission_mod.GENESIS_SHA256, []
        for record in records:
            record["prev_sha256"] = prev
            line = json.dumps(record)
            lines.append(line)
            prev = hashlib.sha256(line.encode("utf-8")).hexdigest()
        self.rewrite(lines)
        self.assertTrue(mission_mod.verify_audit_log(self.log_path)["ok"])
        kinds = self.kinds()
        self.assertIn("mismatch", kinds)
        self.assertNotIn("rewritten", kinds)

    def test_heads_are_read_from_a_session_log_export(self):
        """T38 (M38): the heads come from a session transcript or a
        CloudWatch Logs export of the streamed session (JSON inside JSON),
        only those naming this log's file, without duplicates."""
        events = [{"message": json.dumps({"sessionData": [line]})}
                  for line in self.console.splitlines()]
        events.append({"message": json.dumps({"sessionData": [
            "[audit head] other.jsonl 1:" + "b" * 64 + " after session start"]})})
        export = json.dumps({"events": events})
        self.assertEqual(mission_mod.audit_heads_in(export, self.log_path),
                         list(dict.fromkeys(self.heads)))
        self.assertEqual(mission_mod.audit_heads_in(self.console, "/elsewhere/other.jsonl"), [])
        with self.assertRaises(ValueError):
            mission_mod.parse_head("12:xyz")
        self.assertEqual(mission_mod.parse_head(" 12:" + "A" * 64), (12, "a" * 64))

    def test_the_cli_exit_codes(self):
        """T38 (M38): python3 mission.py --verify exits 0 when every head
        matches, 1 on a finding (printed by kind), and 2 with no head to
        check (unless --chain-only) or a malformed one."""
        session = os.path.join(self.tmp.name, "session.txt")
        with open(session, "w", encoding="utf-8") as handle:
            handle.write(self.console)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(mission_mod.main(["--verify", self.log_path,
                                               "--heads-from", session]), 0)
            self.assertEqual(mission_mod.main(["--verify", self.log_path]), 2)
            self.assertEqual(mission_mod.main(["--verify", self.log_path, "--chain-only"]), 0)
            self.assertEqual(mission_mod.main(["--verify", self.log_path,
                                               "--expect-head", "3:abc"]), 2)
            self.rewrite(self.lines[:-1])
            self.assertEqual(mission_mod.main(["--verify", self.log_path,
                                               "--expect-head", self.heads[-1]]), 1)
        self.assertIn("TRUNCATED:", out.getvalue())

    @unittest.skipUnless(BASH, "needs bash")
    def test_the_cli_runs_as_a_script(self):
        """T38 (M38): agent/mission.py runs as a script, with the Python
        standard library alone, as a reviewer would run it."""
        session = os.path.join(self.tmp.name, "session.txt")
        with open(session, "w", encoding="utf-8") as handle:
            handle.write(self.console)
        env = {"PATH": os.environ.get("PATH", ""), "LOG": self.log_path, "HEADS": session,
               "MISSION_PY": os.path.join(AGENT_DIR, "mission.py")}
        done = subprocess.run(["bash", "-c", 'exec python3 -I "$MISSION_PY" --verify "$LOG" '
                               '--heads-from "$HEADS"'], env=env, cwd=REPO, capture_output=True,
                              text=True, timeout=120)
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertIn("OK", done.stdout)


# ---------------------------------------------------------------------------
# host/add_operator.sh on a fake host
# ---------------------------------------------------------------------------

# A passwd database in $FAKE_ACCOUNTS (JSON) for the account stand-ins below.
FAKE_ACCOUNT_TOOL = r'''#!/usr/bin/env python3
import json, os, sys
db_path = os.environ["FAKE_ACCOUNTS"]
tool = os.path.basename(sys.argv[0])
args = sys.argv[1:]
with open(os.environ["FAKE_CALLS"], "a") as fh:
    fh.write(json.dumps([tool] + args) + "\n")
with open(db_path) as fh:
    db = json.load(fh)
def save():
    with open(db_path, "w") as fh:
        json.dump(db, fh)
name = args[-1] if args else ""
if tool == "getent":
    acct = db.get(name)
    if args[:1] != ["passwd"] or acct is None:
        sys.exit(2)
    print("%s:x:%d:%d:%s:%s:%s" % (name, acct["uid"], acct["uid"], acct["gecos"],
                                   "/home/" + name, acct["shell"]))
elif tool == "useradd":
    if name in db:
        sys.exit(9)
    gecos = args[args.index("--comment") + 1] if "--comment" in args else ""
    shell = args[args.index("--shell") + 1] if "--shell" in args else "/bin/sh"
    db[name] = {"uid": 2000 + len(db), "gecos": gecos, "shell": shell, "pw": "L"}
    save()
elif tool == "usermod":
    if name not in db:
        sys.exit(6)
    if "--lock" in args:
        db[name]["pw"] = "L"
    save()
elif tool == "userdel":
    if name not in db:
        sys.exit(6)
    del db[name]
    save()
elif tool == "passwd":
    print("%s %s 10/30/2026 0 99999 7 -1" % (name, db[name]["pw"]))
'''

# visudo -cf FILE: a sudoers file of comments and plain rules parses, unless
# FAKE_VISUDO_FAIL=cf; visudo -c (the whole configuration) fails with FAKE_VISUDO_FAIL=c.
# Each call records the file's mode, to show the check ran before the install.
FAKE_VISUDO = r'''#!/usr/bin/env python3
import json, os, re, stat, sys
args = sys.argv[1:]
record = ["visudo"] + args
if args[:1] == ["-cf"]:
    path = args[1]
    record.append("%o" % stat.S_IMODE(os.stat(path).st_mode))
with open(os.environ["FAKE_CALLS"], "a") as fh:
    fh.write(json.dumps(record) + "\n")
fail = os.environ.get("FAKE_VISUDO_FAIL", "")
if args[:1] == ["-cf"]:
    with open(path) as fh:
        lines = [l for l in fh.read().splitlines() if l.strip() and not l.startswith("#")]
    rule = re.compile(r'^[a-z][a-z0-9_-]* ALL=\([a-z][a-z0-9_-]*\) NOPASSWD: /\S+ ""$')
    sys.exit(1 if fail == "cf" or not all(rule.match(l) for l in lines) else 0)
sys.exit(1 if fail == "c" else 0)
'''

FAKE_SH = {
    "id": 'if [ "${1:-}" = -u ]; then echo "${FAKE_UID:-0}"; else exec /usr/bin/id "$@"; fi\n',
    "chown": 'echo "[\\"chown\\", \\"$1\\", \\"$2\\"]" >> "$FAKE_CALLS"\n',
}

SEED_ACCOUNTS = {
    "root": {"uid": 0, "gecos": "root", "shell": "/bin/bash", "pw": "P"},
    "ubuntu": {"uid": 1000, "gecos": "Ubuntu", "shell": "/bin/bash", "pw": "P"},
    "ssm-user": {"uid": 1001, "gecos": "", "shell": "/bin/sh", "pw": "L"},
    "daemon": {"uid": 1, "gecos": "daemon", "shell": "/usr/sbin/nologin", "pw": "L"},
}


@unittest.skipUnless(BASH, "needs bash")
class TestT38AddOperator(unittest.TestCase):
    """T38 (M38): host/add_operator.sh, against stand-ins for the account tools."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="spot-add-operator-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        self.sudoers = os.path.join(self.tmp, "sudoers.d")
        os.makedirs(self.bin)
        os.makedirs(self.sudoers)
        self.accounts = os.path.join(self.tmp, "accounts.json")
        with open(self.accounts, "w", encoding="utf-8") as handle:
            json.dump(SEED_ACCOUNTS, handle)
        self.calls_path = os.path.join(self.tmp, "calls.log")
        for tool in ("getent", "useradd", "usermod", "userdel", "passwd"):
            self.add_fake(tool, FAKE_ACCOUNT_TOOL)
        self.add_fake("visudo", FAKE_VISUDO)
        for name, body in FAKE_SH.items():
            self.add_fake(name, "#!/bin/sh\n" + body)
        self.env = {
            "PATH": os.pathsep.join([self.bin, os.path.dirname(BASH), os.environ.get("PATH", "")]),
            "HOME": self.tmp, "FAKE_ACCOUNTS": self.accounts, "FAKE_CALLS": self.calls_path,
            "SPOT_SUDOERS_DIR": self.sudoers, "SCRIPT": os.path.join(REPO, "host",
                                                                      "add_operator.sh"),
        }

    def add_fake(self, name, body):
        path = os.path.join(self.bin, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)
        os.chmod(path, 0o755)

    def run_script(self, *args, **env):
        """bash host/add_operator.sh ARGS: the arguments go in A1..A3 and N."""
        merged = dict(self.env, N=str(len(args)))
        for i, value in enumerate(args, start=1):
            merged["A%d" % i] = value
        merged.update(env)
        return subprocess.run(
            ["bash", "-c", 'case "$N" in 0) exec bash "$SCRIPT" ;; 1) exec bash "$SCRIPT" "$A1" ;; '
                           '2) exec bash "$SCRIPT" "$A1" "$A2" ;; '
                           '*) exec bash "$SCRIPT" "$A1" "$A2" "$A3" ;; esac'],
            env=merged, cwd=REPO, capture_output=True, text=True, timeout=60,
            stdin=subprocess.DEVNULL)

    def calls(self, tool=None):
        if not os.path.exists(self.calls_path):
            return []
        with open(self.calls_path, encoding="utf-8") as handle:
            calls = [json.loads(line) for line in handle if line.strip()]
        return [c for c in calls if tool is None or c[0] == tool]

    def db(self):
        with open(self.accounts, encoding="utf-8") as handle:
            return json.load(handle)

    def dropin(self, name="op-alice"):
        return os.path.join(self.sudoers, "spot-operator-" + name)

    def test_add_creates_the_account_and_the_one_sudo_rule(self):
        """T38 (M38): add creates the account with no password (locked) and
        installs a sudoers file, mode 0440 and owned by root, that allows
        only `sudo -iu ubuntu` (ubuntu's login shell, no arguments); it was
        checked with visudo -cf at mode 0440 before it was installed."""
        done = self.run_script("op-alice")
        self.assertEqual(done.returncode, 0, done.stderr)
        account = self.db()["op-alice"]
        self.assertEqual((account["gecos"], account["shell"], account["pw"]),
                         ("spot-demo operator", "/bin/bash", "L"))
        (useradd,) = self.calls("useradd")
        self.assertIn("--create-home", useradd)
        self.assertFalse({"-p", "--password"} & set(useradd))
        with open(self.dropin(), encoding="utf-8") as handle:
            rules = [line for line in handle.read().splitlines() if not line.startswith("#")]
        self.assertEqual(rules, ['op-alice ALL=(ubuntu) NOPASSWD: /bin/bash ""'])
        self.assertEqual(stat.S_IMODE(os.stat(self.dropin()).st_mode), 0o440)
        checked = self.calls("visudo")
        self.assertEqual([c[1] for c in checked], ["-cf", "-c"])
        self.assertEqual(checked[0][-1], "440")
        self.assertTrue(any(c[1] == "root:root" for c in self.calls("chown")))
        self.assertEqual([n for n in os.listdir(self.sudoers)], ["spot-operator-op-alice"])
        self.assertIn("SSMSessionRunAs=op-alice", done.stderr)

    def test_add_is_idempotent(self):
        """T38 (M38): a second add creates nothing and rewrites nothing."""
        self.assertEqual(self.run_script("op-alice").returncode, 0)
        before = os.stat(self.dropin()).st_mtime_ns
        done = self.run_script("op-alice")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual(len(self.calls("useradd")), 1)
        self.assertEqual(len([c for c in self.calls("visudo") if c[1] == "-cf"]), 1)
        self.assertEqual(os.stat(self.dropin()).st_mtime_ns, before)
        self.assertIn("exists already", done.stderr)
        self.assertEqual(self.run_script("status", "op-alice").returncode, 0)

    def test_names_that_are_not_plain_user_names_are_refused(self):
        """T38 (M38): only a plain POSIX user name is accepted; the shared
        accounts and an existing account the script did not create are
        refused, and nothing is created or written for any of them."""
        bad = ("Alice", "1alice", "-alice", "al.ice", "al/ice", "al ice", "a" * 33, "",
               "alice\n", "al$ice", "alice;id")
        for name in bad:
            with self.subTest(name=name):
                done = self.run_script(name)
                self.assertEqual(done.returncode, 2, done.stderr)
        for name in ("root", "ubuntu", "ssm-user"):
            with self.subTest(name=name):
                done = self.run_script(name)
                self.assertEqual(done.returncode, 2)
                self.assertIn("shared account", done.stderr)
        done = self.run_script("daemon")
        self.assertEqual(done.returncode, 1)
        self.assertIn("not created by this script", done.stderr)
        self.assertEqual(self.calls("useradd") + self.calls("usermod"), [])
        self.assertEqual(os.listdir(self.sudoers), [])
        self.assertEqual(self.db(), SEED_ACCOUNTS)

    def test_a_rule_visudo_rejects_is_never_installed(self):
        """T38 (M38): when visudo -cf rejects the file nothing is installed
        (and no temporary file is left); when the whole configuration no
        longer parses, the file is removed again."""
        for fail in ("cf", "c"):
            with self.subTest(fail=fail):
                done = self.run_script("op-alice", FAKE_VISUDO_FAIL=fail)
                self.assertEqual(done.returncode, 1, done.stderr)
                self.assertEqual(os.listdir(self.sudoers), [])

    def test_remove_undoes_exactly_what_add_did(self):
        """T38 (M38): remove deletes the sudoers file and the account it
        created (and its home); it refuses an account the script did not
        create, and is a no-op when there is nothing to remove."""
        self.assertEqual(self.run_script("op-alice").returncode, 0)
        done = self.run_script("remove", "op-alice")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertNotIn("op-alice", self.db())
        self.assertEqual(self.calls("userdel")[-1], ["userdel", "--remove", "op-alice"])
        self.assertEqual(os.listdir(self.sudoers), [])
        self.assertEqual(self.run_script("remove", "op-alice").returncode, 0)
        self.assertEqual(self.run_script("status", "op-alice").returncode, 1)
        done = self.run_script("remove", "daemon")
        self.assertEqual(done.returncode, 1)
        self.assertIn("daemon", self.db())

    def test_dry_run_and_arguments(self):
        """T38 (M38): --dry-run prints the changes and makes none; status
        reports a changed sudoers file; bad arguments and a non-root caller
        are refused before anything changes."""
        done = self.run_script("op-alice", "--dry-run")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("useradd --create-home", done.stdout)
        self.assertIn('op-alice ALL=(ubuntu) NOPASSWD: /bin/bash ""', done.stdout)
        self.assertNotIn("op-alice", self.db())
        self.assertEqual(os.listdir(self.sudoers), [])
        self.assertEqual(self.run_script("op-alice").returncode, 0)
        os.chmod(self.dropin(), 0o640)
        status = self.run_script("status", "op-alice")
        self.assertEqual(status.returncode, 1)
        self.assertIn("changed", status.stdout)
        for args in ((), ("remove",), ("op-bob", "--force"), ("status", "op-bob", "--dry-run"),
                     ("op-bob", "--dry-run", "x")):
            with self.subTest(args=args):
                self.assertEqual(self.run_script(*args).returncode, 2)
        done = self.run_script("op-bob", FAKE_UID="1000")
        self.assertEqual(done.returncode, 2)
        self.assertIn("run as root", done.stderr)
        self.assertNotIn("op-bob", self.db())


# ---------------------------------------------------------------------------
# Session Manager preferences, the session-log template, the instance role
# ---------------------------------------------------------------------------

LOG_GROUP = "/spot-demo/session-manager"
LOG_GROUP_ARN = "arn:aws:logs:us-west-2:<ACCOUNT_ID>:log-group:" + LOG_GROUP
KEY_ARN = "arn:aws:kms:us-west-2:<ACCOUNT_ID>:key/<SESSION_LOG_KEY_ID>"


def _actions(statement):
    actions = statement["Action"]
    return [actions] if isinstance(actions, str) else list(actions)


def _resources(statement):
    resources = statement["Resource"]
    return [resources] if isinstance(resources, str) else list(resources)


def _load_template():
    """infra/session-logging.yaml, with the CloudFormation short forms
    (!Ref, !Sub, !GetAtt) read as {"Ref": ...} / {"Fn::Sub": ...}."""

    class CfnLoader(yaml.SafeLoader):
        pass

    def short_form(loader, suffix, node):
        if isinstance(node, yaml.ScalarNode):
            value = loader.construct_scalar(node)
        elif isinstance(node, yaml.SequenceNode):
            value = loader.construct_sequence(node)
        else:
            value = loader.construct_mapping(node)
        if suffix == "GetAtt" and isinstance(value, str):
            value = value.split(".", 1)
        return {"Ref": value} if suffix == "Ref" else {"Fn::" + suffix: value}

    CfnLoader.add_multi_constructor("!", short_form)
    loader = CfnLoader(_read("infra", "session-logging.yaml"))
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()


class TestT38SessionManagerPreferences(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads(_read("docs", "iam", "session-preferences.json"))
        self.inputs = self.doc["inputs"]

    def test_run_as_each_operator_with_no_default(self):
        """T38 (M38): the preferences are a Standard_Stream Session document
        with Run As on and no default user, so a principal without the
        SSMSessionRunAs tag gets no session (it never falls back to
        ssm-user or a shared account)."""
        self.assertEqual(self.doc["schemaVersion"], "1.0")
        self.assertEqual(self.doc["sessionType"], "Standard_Stream")
        self.assertIs(self.inputs["runAsEnabled"], True)
        self.assertEqual(self.inputs["runAsDefaultUser"], "")

    def test_sessions_stream_to_the_encrypted_log_group(self):
        """T38 (M38): session data is streamed (as it happens, not at the
        end of the session) to the template's log group, which must be
        encrypted; no S3 bucket; an idle timeout of 1 to 60 minutes."""
        self.assertEqual(self.inputs["cloudWatchLogGroupName"], LOG_GROUP)
        self.assertIs(self.inputs["cloudWatchEncryptionEnabled"], True)
        self.assertIs(self.inputs["cloudWatchStreamingEnabled"], True)
        self.assertEqual(self.inputs["s3BucketName"], "")
        self.assertTrue(1 <= int(self.inputs["idleSessionTimeout"]) <= 60)

    def test_the_readme_says_how_to_apply_and_roll_back(self):
        """T38 (M38): docs/iam/README.md says the preferences are account-
        and Region-wide, how to apply them (update-document or
        create-document on SSM-SessionManagerRunShell), that each operator
        needs the SSMSessionRunAs tag and an account from add_operator.sh,
        and how to roll back."""
        text = _flat(_read("docs", "iam", "README.md"))
        for part in ("account- and Region-wide", "dedicated account",
                     "aws ssm update-document --region us-west-2 --name SSM-SessionManagerRunShell",
                     "aws ssm create-document", "SSMSessionRunAs", "host/add_operator.sh",
                     "To roll back", "previous-preferences.json", "infra/session-logging.yaml",
                     "<SESSION_LOG_KEY_ID>"):
            self.assertIn(part, text)


@unittest.skipUnless(yaml, "needs PyYAML (tests/requirements-ci.lock)")
class TestT38RunAsTagGuardrail(unittest.TestCase):
    def test_nobody_it_is_attached_to_can_set_the_run_as_tag(self):
        """T38, T29 (M38): the Run As tag decides whose account a shell runs as, and so
        whose name the mission record carries. docs/iam/operator-runas-guardrail.json
        grants nothing and denies setting or removing SSMSessionRunAs on any user or
        role and passing it as a session tag; the IAM README says to attach it to every
        principal that can open a shell."""
        doc = json.loads(_read("docs", "iam", "operator-runas-guardrail.json"))
        statements = doc["Statement"]
        self.assertTrue(statements)
        self.assertEqual({st["Effect"] for st in statements}, {"Deny"})
        actions = sorted(a for st in statements for a in _actions(st))
        self.assertEqual(actions, ["iam:TagRole", "iam:TagUser", "iam:UntagRole", "iam:UntagUser",
                                   "sts:TagSession"])
        for st in statements:
            self.assertEqual(st["Resource"], "*")
            self.assertEqual(st["Condition"], {"ForAnyValue:StringEquals": {"aws:TagKeys": "SSMSessionRunAs"}})
        readme = _flat(_read("docs", "iam", "README.md"))
        self.assertIn("[`operator-runas-guardrail.json`](operator-runas-guardrail.json) | Every principal that can open a shell", readme)
        self.assertIn("attach [`operator-runas-guardrail.json`](operator-runas-guardrail.json) to every operator and administrator principal", readme)


class TestT38SessionLogTemplate(unittest.TestCase):
    def setUp(self):
        self.template = _load_template()
        self.resources = self.template["Resources"]

    def of_type(self, kind):
        (name,) = [n for n, r in self.resources.items() if r["Type"] == kind]
        return name, self.resources[name]

    def test_the_key_serves_cloudwatch_logs_for_this_log_group_only(self):
        """T38 (M38): a rotated symmetric KMS key whose policy lets the
        account administer it through IAM and lets CloudWatch Logs in this
        Region use it only with this log group's ARN as encryption context;
        no wildcard principal."""
        _name, key = self.of_type("AWS::KMS::Key")
        props = key["Properties"]
        self.assertIs(props["EnableKeyRotation"], True)
        self.assertEqual(props["KeySpec"], "SYMMETRIC_DEFAULT")
        statements = props["KeyPolicy"]["Statement"]
        self.assertEqual(len(statements), 2)
        for statement in statements:
            self.assertNotEqual(statement["Principal"], "*")
            self.assertNotIn("*", statement["Principal"].values())
        logs = next(s for s in statements if "Service" in s["Principal"])
        self.assertEqual(logs["Principal"]["Service"], {"Fn::Sub": "logs.${AWS::Region}.amazonaws.com"})
        self.assertEqual(logs["Condition"]["StringEquals"]["kms:EncryptionContext:aws:logs:arn"],
                         {"Fn::Sub": "arn:${AWS::Partition}:logs:${AWS::Region}:${AWS::AccountId}"
                                     ":log-group:${LogGroupName}"})
        account = next(s for s in statements if "AWS" in s["Principal"])
        self.assertEqual(account["Principal"]["AWS"],
                         {"Fn::Sub": "arn:${AWS::Partition}:iam::${AWS::AccountId}:root"})

    def test_the_log_group_is_encrypted_kept_and_retained(self):
        """T38 (M38): the log group uses the key, keeps data at least a year,
        and both survive a stack deletion (they hold the audit trail); its
        default name is the one the preferences and the role policy use."""
        key_name, key = self.of_type("AWS::KMS::Key")
        _name, group = self.of_type("AWS::Logs::LogGroup")
        props = group["Properties"]
        self.assertEqual(props["KmsKeyId"], {"Fn::GetAtt": [key_name, "Arn"]})
        self.assertEqual(props["RetentionInDays"], {"Ref": "RetentionInDays"})
        retention = self.template["Parameters"]["RetentionInDays"]
        self.assertGreaterEqual(min(retention["AllowedValues"]), 365)
        self.assertGreaterEqual(retention["Default"], 365)
        self.assertEqual(self.template["Parameters"]["LogGroupName"]["Default"], LOG_GROUP)
        for resource in (key, group):
            self.assertEqual(resource["DeletionPolicy"], "Retain")
            self.assertEqual(resource["UpdateReplacePolicy"], "Retain")
        self.assertTrue({"LogGroupName", "LogGroupArn", "KeyArn"} <= set(self.template["Outputs"]))


class TestT38InstanceRoleSessionLogs(unittest.TestCase):
    def setUp(self):
        doc = json.loads(_read("docs", "iam", "instance-role-policy.json"))
        self.statements = doc["Statement"]

    def with_prefix(self, prefix):
        return [s for s in self.statements if any(a.startswith(prefix) for a in _actions(s))]

    def test_logs_only_on_the_session_log_group(self):
        """T38 (M38): the SSM Agent's documented CloudWatch Logs actions and
        no others: create streams and put events in the session log group's
        streams, describe its streams, and DescribeLogGroups, which has no
        resource-level permission (Resource "*")."""
        logs = self.with_prefix("logs:")
        actions = sorted(a for s in logs for a in _actions(s))
        self.assertEqual(actions, ["logs:CreateLogStream", "logs:DescribeLogGroups",
                                   "logs:DescribeLogStreams", "logs:PutLogEvents"])
        for statement in logs:
            self.assertEqual(statement["Effect"], "Allow")
            if _actions(statement) == ["logs:DescribeLogGroups"]:
                self.assertEqual(statement["Resource"], "*")
            else:
                for resource in _resources(statement):
                    self.assertTrue(resource == LOG_GROUP_ARN or
                                    resource.startswith(LOG_GROUP_ARN + ":"), resource)
        put = next(s for s in logs if "logs:PutLogEvents" in _actions(s))
        self.assertEqual(_resources(put), [LOG_GROUP_ARN + ":log-stream:*"])

    def test_the_key_only_through_cloudwatch_logs(self):
        """T38 (M38): the KMS permissions are those CloudWatch Logs needs to
        write to the encrypted group, on the one key, only through CloudWatch
        Logs, and GenerateDataKey/Decrypt only for this log group."""
        kms = self.with_prefix("kms:")
        self.assertEqual(sorted(a for s in kms for a in _actions(s)),
                         ["kms:Decrypt", "kms:DescribeKey", "kms:GenerateDataKey"])
        for statement in kms:
            self.assertEqual(statement["Resource"], KEY_ARN)
            self.assertEqual(statement["Condition"]["StringEquals"]["kms:ViaService"],
                             "logs.us-west-2.amazonaws.com")
            if "kms:Decrypt" in _actions(statement):
                self.assertEqual(
                    statement["Condition"]["StringEquals"]["kms:EncryptionContext:aws:logs:arn"],
                    LOG_GROUP_ARN)


    def test_no_policy_document_repeats_a_key(self):
        """T38, T27, T29: a JSON object with the same key twice (two "StringEquals"
        blocks, say) parses without error but keeps only the last, silently dropping
        a condition. Every shipped policy document has each key once."""
        def unique(pairs):
            keys = [key for key, _ in pairs]
            self.assertEqual(len(keys), len(set(keys)), keys)
            return dict(pairs)
        iam = os.path.join(REPO, "docs", "iam")
        names = sorted(n for n in os.listdir(iam) if n.endswith(".json"))
        self.assertIn("instance-role-policy.json", names)
        for name in names:
            with open(os.path.join(iam, name), encoding="utf-8") as handle:
                json.load(handle, object_pairs_hook=unique)

# ---------------------------------------------------------------------------
# Scripts and documentation
# ---------------------------------------------------------------------------


class TestT38ScriptsAndDocs(unittest.TestCase):
    def test_the_unattended_rehearsal_passes_the_flag(self):
        """T38 (M38): tools/rehearse_rep.sh runs the agent with no operator
        (as root through sudo), so it passes --unattributed-run; the smoke
        and e2e tests start neither the agent nor the scripted mission."""
        rehearse = _read("tools", "rehearse_rep.sh")
        (call,) = [line for line in rehearse.splitlines()
                   if "run_agent.sh" in line and not line.lstrip().startswith("#")]
        self.assertIn("--unattributed-run", call)
        for script in ("smoke_test.sh", "e2e_ros_test.sh"):
            text = _read("host", script)
            self.assertNotIn("run_agent.sh", text)
            self.assertNotIn("run_mission", text)

    def test_the_runbook_makes_per_operator_access_the_procedure(self):
        """T38 (M38): the runbook has the administrator's steps (preferences,
        template, tags, add_operator.sh), connects operators as themselves
        before sudo -iu ubuntu, explains the head lines and how to check a
        log against the session log, and states exactly who can rename or
        delete a log in the sticky logs directory."""
        text = _flat(_read("docs", "RUNBOOK.md"))
        for part in ("Once per account: one OS account per operator", "infra/session-logging.yaml",
                     "SSMSessionRunAs=<their OS account>", "host/add_operator.sh <os-account>",
                     "docs/iam/session-preferences.json", "a shell as your own OS account",
                     "--unattributed-run", "**Audit head lines.**",
                     "**Check a log against the session log**",
                     "python3 agent/mission.py --verify",
                     "--heads-from ~/spot-demo-logs/<instance-id>/session-<session-id>.json",
                     "aws ssm describe-sessions --region us-west-2 --state History",
                     "TRUNCATED", "REWRITTEN", "MISMATCH",
                     "renamed or deleted only by the file's owner, the directory's owner or root",
                     "one-shot containers that write there through the group too (uid 10003",
                     "tmux -L <your account>"):
            self.assertIn(part, text)

    def test_session_exports_are_kept_outside_the_checkout(self):
        """T36, T38: the session export (the operator's whole console transcript) and
        the saved Session Manager preferences (log group, bucket and key names) are
        written outside the checkout, like fetched logs, and .gitignore keeps stray
        copies at the repository root out of a commit."""
        runbook = _read("docs", "RUNBOOK.md")
        (export,) = [l for l in runbook.splitlines() if "--output json >" in l]
        self.assertIn("> ~/spot-demo-logs/", export)
        iam = _read("docs", "iam", "README.md")
        (saved,) = [l for l in iam.splitlines() if "--query Content --output text >" in l]
        self.assertIn("> ~/", saved)
        ignored = _read(".gitignore").splitlines()
        for pattern in ("/session*.json", "/previous-preferences.json"):
            self.assertIn(pattern, ignored)

    def test_the_readme_describes_operator_access_and_the_log(self):
        """T38 (M38): the README lists --unattributed-run for both programs,
        add_operator.sh, the [audit head] lines and the verify command."""
        text = _flat(_read("README.md"))
        self.assertEqual(text.count("| `--unattributed-run` | off |"), 2)
        for part in ("| `add_operator.sh` |", "`[audit head]`",
                     "python3 agent/mission.py --verify LOG --heads-from SESSION_LOG",
                     "as your own OS account"):
            self.assertIn(part, text)


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    unittest.main()
