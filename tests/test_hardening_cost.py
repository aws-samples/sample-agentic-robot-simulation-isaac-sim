"""Tests for the cost and availability guardrails (threat model 2.3.3, T39).

Each class docstring names the threat it covers. The CloudFormation template is
parsed and its resources checked; nothing is deployed. host/idle_stop.sh runs
against a fake host: a temporary directory of stand-ins for docker, pgrep,
systemctl, shutdown, sudo and chown, first on PATH, and temporary paths for the
unit files, the installed copy, the state and the log. scripts/start-host.sh runs
against a fake AWS CLI and a fake sleep. No AWS call is made, nothing is powered
off and nothing listens on the network.

Every subprocess call here has an argv of string literals only; the variable parts
travel in the environment or the working directory.

Run from the repo root:

    python3 -m unittest tests.test_hardening_cost -v
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

try:
    import yaml
except ImportError:   # in tests/requirements-ci.lock; the template tests skip without it
    yaml = None

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASH = shutil.which("bash")
if REPO not in sys.path:
    sys.path.insert(0, REPO)


def _read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as handle:
        return handle.read()


def _flat(text):
    return " ".join(text.split())


# --- the template ------------------------------------------------------------------

def _cfn_load(text):
    """The template as plain data; each short-form tag becomes {"Fn::X": value} ("Ref" for !Ref)."""
    class Loader(yaml.SafeLoader):
        pass

    def construct(loader, suffix, node):
        if isinstance(node, yaml.ScalarNode):
            value = loader.construct_scalar(node)
        elif isinstance(node, yaml.SequenceNode):
            value = loader.construct_sequence(node, deep=True)
        else:
            value = loader.construct_mapping(node, deep=True)
        if suffix == "GetAtt" and isinstance(value, str):
            value = value.split(".", 1)
        return {("Ref" if suffix == "Ref" else "Fn::" + suffix): value}

    Loader.add_multi_constructor("!", construct)
    loader = Loader(text)   # what yaml.load does, with the SafeLoader subclass spelled out
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()


@unittest.skipUnless(yaml, "needs PyYAML (tests/requirements-ci.lock)")
class TestT39CostGuardrailsTemplate(unittest.TestCase):
    """T39 (M39): infra/cost-guardrails.yaml caps and watches the host's cost.

    An account-wide AWS Budgets COST budget (no filter, so Bedrock, EC2, EBS and
    the rest all count; credits and refunds excluded) emails at 80 % and 100 % of
    actual and 100 % of forecast spend; an alarm stops the instance with the EC2
    stop action after its CPU stayed below a threshold for the idle window; an
    instance-check alarm reboots it; a system-check alarm notifies, and recovers
    only when asked, because the g6 family supports no recover action. The alarm
    topic is encrypted with a customer managed key only this stack's alarms may
    use. Every resource is deleted with the stack. Not covered: deploying it
    (cfn-lint and checkov were run; no account was touched).
    """

    def setUp(self):
        self.doc = _cfn_load(_read("infra", "cost-guardrails.yaml"))
        self.res = self.doc["Resources"]

    def of_type(self, kind):
        return {name: r["Properties"] for name, r in self.res.items() if r["Type"] == kind}

    def test_parameters_are_the_instance_the_budget_and_the_email(self):
        """T39: the three required parameters, with the instance id validated by CloudFormation."""
        params = self.doc["Parameters"]
        self.assertEqual(params["InstanceId"]["Type"], "AWS::EC2::Instance::Id")
        self.assertEqual(params["MonthlyBudgetUSD"]["Type"], "Number")
        self.assertNotIn("Default", params["MonthlyBudgetUSD"])          # the adopter chooses
        self.assertNotIn("Default", params["NotificationEmail"])
        email = re.compile(params["NotificationEmail"]["AllowedPattern"])
        self.assertTrue(email.match("ops@example.com"))
        self.assertFalse(email.match("ops@example.com, other@example.com"))
        # Period 300 s x periods may not exceed one day (PutMetricAlarm).
        self.assertLessEqual(params["IdlePeriods"]["MaxValue"] * 300, 86400)
        self.assertEqual(params["SystemCheckAction"]["Default"], "notify")

    def test_the_budget_is_the_whole_accounts_cost_on_actual_and_forecast(self):
        """T39: the budget has no cost filter (Bedrock counts) and alerts on actual and forecast spend."""
        (budget,) = self.of_type("AWS::Budgets::Budget").values()
        data = budget["Budget"]
        self.assertEqual(data["BudgetType"], "COST")
        self.assertEqual(data["TimeUnit"], "MONTHLY")
        self.assertEqual(data["BudgetLimit"], {"Amount": {"Ref": "MonthlyBudgetUSD"}, "Unit": "USD"})
        self.assertNotIn("CostFilters", data)
        self.assertNotIn("FilterExpression", data)
        self.assertEqual(data["CostTypes"], {"IncludeCredit": False, "IncludeRefund": False})
        alerts = sorted((n["Notification"]["NotificationType"], n["Notification"]["Threshold"])
                        for n in budget["NotificationsWithSubscribers"])
        self.assertEqual(alerts, [("ACTUAL", 80), ("ACTUAL", 100), ("FORECASTED", 100)])
        for item in budget["NotificationsWithSubscribers"]:
            self.assertEqual(item["Notification"]["ThresholdType"], "PERCENTAGE")
            self.assertEqual(item["Subscribers"], [{"SubscriptionType": "EMAIL",
                                                    "Address": {"Ref": "NotificationEmail"}}])

    def alarm(self, metric):
        (alarm,) = [a for a in self.of_type("AWS::CloudWatch::Alarm").values() if a["MetricName"] == metric]
        self.assertEqual(alarm["Namespace"], "AWS/EC2")
        self.assertEqual(alarm["Dimensions"], [{"Name": "InstanceId", "Value": {"Ref": "InstanceId"}}])
        return alarm

    def test_the_status_alarms_ignore_missing_data(self):
        """The status-check alarms keep the CloudWatch guide's TreatMissingData: missing."""
        for metric in ("StatusCheckFailed_Instance", "StatusCheckFailed_System"):
            with self.subTest(metric=metric):
                self.assertEqual(self.alarm(metric)["TreatMissingData"], "missing")

    def test_the_idle_alarm_counts_a_stopped_host_as_not_idle(self):
        """T39, measured in an account (2026-11-01): with TreatMissingData missing, the
        periods while the host was stopped were skipped, so the low datapoints of its last
        idle hour and one boot datapoint filled the window, and the alarm stopped the host
        82 s after it was started again. notBreaching counts a stopped period as not idle,
        so a started host gets the whole idle window first."""
        self.assertEqual(self.alarm("CPUUtilization")["TreatMissingData"], "notBreaching")

    def test_the_idle_alarm_stops_the_instance(self):
        """T39: low average CPU for every period of the idle window takes the EC2 stop action."""
        alarm = self.alarm("CPUUtilization")
        self.assertEqual(alarm["Statistic"], "Average")
        self.assertEqual(alarm["Period"], 300)
        self.assertEqual(alarm["EvaluationPeriods"], {"Ref": "IdlePeriods"})
        self.assertEqual(alarm["DatapointsToAlarm"], {"Ref": "IdlePeriods"})
        self.assertEqual(alarm["ComparisonOperator"], "LessThanThreshold")
        self.assertEqual(alarm["Threshold"], {"Ref": "IdleCpuPercent"})
        self.assertEqual(alarm["AlarmActions"],
                         [{"Fn::Sub": "arn:${AWS::Partition}:automate:${AWS::Region}:ec2:stop"},
                          {"Ref": "AlarmTopic"}])
        self.assertNotIn("terminate", json.dumps(self.res))

    def test_status_checks_reboot_and_recover_only_when_asked(self):
        """T39: the instance check reboots after 3 minutes; the system check notifies after 2, and
        takes the recover action only under SystemCheckAction=recover (g6 has no recover support)."""
        reboot = self.alarm("StatusCheckFailed_Instance")
        self.assertEqual((reboot["Statistic"], reboot["Period"], reboot["EvaluationPeriods"],
                          reboot["DatapointsToAlarm"], reboot["ComparisonOperator"], reboot["Threshold"]),
                         ("Minimum", 60, 3, 3, "GreaterThanThreshold", 0))
        self.assertEqual(reboot["AlarmActions"][0],
                         {"Fn::Sub": "arn:${AWS::Partition}:automate:${AWS::Region}:ec2:reboot"})
        system = self.alarm("StatusCheckFailed_System")
        self.assertEqual((system["Statistic"], system["Period"], system["EvaluationPeriods"],
                          system["DatapointsToAlarm"]), ("Minimum", 60, 2, 2))
        condition, recover, notify = system["AlarmActions"]["Fn::If"]
        self.assertEqual(condition, "RecoverOnSystemCheck")
        self.assertEqual(recover, [{"Fn::Sub": "arn:${AWS::Partition}:automate:${AWS::Region}:ec2:recover"},
                                   {"Ref": "AlarmTopic"}])
        self.assertEqual(notify, [{"Ref": "AlarmTopic"}])
        self.assertEqual(self.doc["Conditions"]["RecoverOnSystemCheck"],
                         {"Fn::Equals": [{"Ref": "SystemCheckAction"}, "recover"]})

    def test_the_alarm_topic_is_encrypted_and_only_this_stacks_alarms_publish(self):
        """T39: a customer managed key with rotation (CloudWatch cannot use alias/aws/sns), and
        key and topic policies limited to this account's alarms named after the stack."""
        (key,) = self.of_type("AWS::KMS::Key").values()
        self.assertTrue(key["EnableKeyRotation"])
        (topic,) = self.of_type("AWS::SNS::Topic").values()
        self.assertEqual(topic["KmsMasterKeyId"], {"Ref": "AlarmTopicKey"})
        source = {"StringEquals": {"aws:SourceAccount": {"Ref": "AWS::AccountId"}},
                  "ArnLike": {"aws:SourceArn": {"Fn::Sub": "arn:${AWS::Partition}:cloudwatch:${AWS::Region}:"
                                                           "${AWS::AccountId}:alarm:${AWS::StackName}-*"}}}
        statements = key["KeyPolicy"]["Statement"]
        service = [s for s in statements if s["Principal"] != {"AWS": {"Fn::Sub": "arn:${AWS::Partition}:iam::"
                                                                              "${AWS::AccountId}:root"}}]
        self.assertEqual(len(service), 1)
        self.assertEqual(service[0]["Principal"], {"Service": "cloudwatch.amazonaws.com"})
        self.assertEqual(service[0]["Action"], ["kms:Decrypt", "kms:GenerateDataKey*"])
        self.assertEqual(service[0]["Condition"], source)
        (policy,) = self.of_type("AWS::SNS::TopicPolicy").values()
        (publish,) = policy["PolicyDocument"]["Statement"]
        self.assertEqual((publish["Principal"], publish["Action"]),
                         ({"Service": "cloudwatch.amazonaws.com"}, "sns:Publish"))
        self.assertEqual(publish["Condition"], source)
        for props in self.of_type("AWS::CloudWatch::Alarm").values():
            self.assertTrue(props["AlarmName"]["Fn::Sub"].startswith("${AWS::StackName}-"))
        (subscription,) = self.of_type("AWS::SNS::Subscription").values()
        self.assertEqual((subscription["Protocol"], subscription["Endpoint"]),
                         ("email", {"Ref": "NotificationEmail"}))

    def test_everything_is_deleted_with_the_stack_and_named_in_the_outputs(self):
        """T39: no resource outlives the stack; the instance is never a resource of it."""
        for name, resource in self.res.items():
            with self.subTest(resource=name):
                self.assertEqual(resource.get("DeletionPolicy"), "Delete")
                self.assertEqual(resource.get("UpdateReplacePolicy"), "Delete")
                self.assertFalse(resource["Type"].startswith("AWS::EC2::"))
        self.assertEqual(self.of_type("AWS::KMS::Key")["AlarmTopicKey"]["PendingWindowInDays"], 7)
        self.assertEqual(set(self.doc["Outputs"]),
                         {"BudgetName", "IdleStopAlarmName", "InstanceCheckRebootAlarmName",
                          "SystemCheckAlarmName", "AlarmTopicArn", "AlarmTopicKeyArn"})


# --- the host-side idle stop ---------------------------------------------------------

FAKE_DOCKER = r'''#!/usr/bin/env python3
"""Fake docker: `ps` prints $FAKE_DOCKER_PS (space-separated names); rc $FAKE_DOCKER_RC."""
import os, sys
with open(os.environ["FAKE_CALLS"], "a") as fh:
    fh.write("docker " + " ".join(sys.argv[1:]) + "\n")
rc = int(os.environ.get("FAKE_DOCKER_RC", "0"))
if sys.argv[1:2] == ["ps"] and rc == 0:
    for name in os.environ.get("FAKE_DOCKER_PS", "").split():
        print(name)
sys.exit(rc)
'''

FAKE_PGREP = r'''#!/usr/bin/env python3
"""Fake pgrep -f PATTERN over $FAKE_PROCS ("pid cmdline" lines); rc $FAKE_PGREP_RC if set."""
import os, re, sys
args = sys.argv[1:]
if os.environ.get("FAKE_PGREP_RC"):
    sys.exit(int(os.environ["FAKE_PGREP_RC"]))
assert args[0] == "-f", args
pattern = re.compile(args[1])
hits = []
for line in os.environ.get("FAKE_PROCS", "").splitlines():
    pid, _, cmd = line.partition(" ")
    if pattern.search(cmd):
        hits.append(pid)
print("\n".join(hits))
sys.exit(0 if hits else 1)
'''

FAKE_SH = {
    "sudo": 'while [ $# -gt 0 ]; do case "$1" in -u) shift 2 ;; -i|-n|-E|--) shift ;; *) break ;; esac; done\n'
            'exec "$@"\n',
    "chown": 'echo "chown $*" >> "$FAKE_CALLS"\n',
    "shutdown": 'echo "shutdown $*" >> "$FAKE_CALLS"\n',
    # systemctl: records each call; the timer is "enabled and active" from enable to disable.
    "systemctl": 'echo "systemctl $*" >> "$FAKE_CALLS"\n'
                 'case "$1" in\n'
                 '  enable) touch "$FAKE_TIMER_ON" ;;\n'
                 '  disable) rm -f "$FAKE_TIMER_ON" ;;\n'
                 '  is-enabled|is-active) [ -e "$FAKE_TIMER_ON" ] ;;\n'
                 'esac\n',
}

SESSION = "4242 /usr/bin/ssm-session-worker ssm-user-0a1b2c3d4e5f6a7b8 i-0123456789abcdef0"
RUN_COMMAND = "4343 /usr/bin/ssm-document-worker 5e1c0a4b-1111-2222-3333-444455556666 i-0123456789abcdef0"
AGENT = "5151 /opt/agent-venv/bin/python3 /workspace/agent/spot_agent.py --log-path /workspace/logs/demo.jsonl"
MISSION = "5252 /opt/agent-venv/bin/python3 /workspace/agent/run_mission.py --storyline"
BACKGROUND = "1 /sbin/init\n700 /usr/bin/dockerd -H fd://\n900 /usr/bin/amazon-ssm-agent\n" \
             "1200 /usr/bin/dcvserver -d --service"


@unittest.skipUnless(BASH, "needs bash")
class _IdleStopHost(unittest.TestCase):
    """A fake host for host/idle_stop.sh: stand-ins first on PATH, every path temporary."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="spot-idle-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        self.logs = os.path.join(self.tmp, "demo", "logs")
        self.units = os.path.join(self.tmp, "systemd")
        self.state = os.path.join(self.tmp, "run", "spot-idle-stop")
        self.installed = os.path.join(self.tmp, "sbin", "spot-idle-stop")
        self.idle_log = os.path.join(self.tmp, "var-log", "spot-idle-stop.log")
        self.calls = os.path.join(self.tmp, "calls.log")
        for d in (self.bin, self.logs):
            os.makedirs(d)
        self.add_fake("docker", FAKE_DOCKER)
        self.add_fake("pgrep", FAKE_PGREP)
        for name, body in FAKE_SH.items():
            self.add_fake(name, "#!/bin/sh\n" + body)
        self.env = {
            "PATH": os.pathsep.join([self.bin, os.path.dirname(BASH), os.environ.get("PATH", "")]),
            "HOME": self.tmp,
            "FAKE_CALLS": self.calls,
            "FAKE_TIMER_ON": os.path.join(self.tmp, "timer-on"),
            "FAKE_PROCS": BACKGROUND,
            "SPOT_DEMO_DIR": os.path.join(self.tmp, "demo"),
            "SPOT_IDLE_UNIT_DIR": self.units,
            "SPOT_IDLE_BIN": self.installed,
            "SPOT_IDLE_STATE_DIR": self.state,
            "SPOT_IDLE_LOG": self.idle_log,
        }

    def add_fake(self, name, body):
        path = os.path.join(self.bin, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)
        os.chmod(path, 0o700)

    def idle(self, args="", **env):
        """bash host/idle_stop.sh ARGS (split on spaces, never globbed)."""
        merged = dict(self.env, IDLE_ARGS=args)
        merged.update(env)
        return subprocess.run(["bash", "-c", 'set -f; exec bash host/idle_stop.sh $IDLE_ARGS'],
                              cwd=REPO, env=merged, capture_output=True, text=True, timeout=60,
                              stdin=subprocess.DEVNULL)

    def host_calls(self):
        if not os.path.exists(self.calls):
            return ""
        with open(self.calls, encoding="utf-8") as handle:
            return handle.read()

    def set_last_active(self, seconds_ago):
        """Record the last activity SECONDS_AGO, and forget the host calls so far."""
        if os.path.exists(self.calls):
            os.remove(self.calls)
        os.makedirs(self.state, exist_ok=True)
        with open(os.path.join(self.state, "last-active"), "w", encoding="utf-8") as handle:
            handle.write("%d 2026-10-30T09:00:00Z\n" % (int(time.time()) - seconds_ago))

    def last_active_age(self):
        with open(os.path.join(self.state, "last-active"), encoding="utf-8") as handle:
            return int(time.time()) - int(handle.read().split()[0])

    def write_log(self, name, records, age_s=0):
        path = os.path.join(self.logs, name)
        with open(path, "w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record) + "\n")
        stamp = time.time() - age_s
        os.utime(path, (stamp, stamp))
        return path


def _record(kind, permit, seq=1):
    """A record with the top-level keys in the order agent/mission.py writes them."""
    return {"t": 12.5, "wall_t": 1793000000.0, "wall": "2026-10-30T09:00:00.000+00:00",
            "permit_id": "CSE-0427", "permit": permit, "source": "mission", "kind": kind,
            "message": "m", "data": {"permit": {"supervisor": "entry supervisor"}},
            "seq": seq, "prev_sha256": "0" * 64}


class TestT39IdleStopInstall(_IdleStopHost):
    """T39 (M39): host/idle_stop.sh apply/remove/status, in the style of imds_guard.sh.

    apply installs a root-owned copy of the script, a oneshot service and a 5-minute
    timer, enables the timer and prints every change; it is idempotent; --dry-run
    changes nothing; remove deletes exactly what apply added and keeps the power-off
    log; status reports both states. Not covered: systemd itself running the timer,
    and the InstanceInitiatedShutdownBehavior attribute (an AWS-side check, RUNBOOK).
    """

    def test_apply_installs_a_root_owned_copy_and_a_timer(self):
        """T39: apply writes the copy and both units, then enables the timer."""
        result = self.idle("apply --minutes 45")
        self.assertEqual(result.returncode, 0, result.stderr)
        with open(self.installed, encoding="utf-8") as handle:
            self.assertEqual(handle.read(), _read("host", "idle_stop.sh"))
        self.assertEqual(os.stat(self.installed).st_mode & 0o777, 0o755)
        with open(os.path.join(self.units, "spot-idle-stop.service"), encoding="utf-8") as handle:
            service = handle.read()
        self.assertIn("Type=oneshot\n", service)
        self.assertIn("TimeoutStartSec=120\n", service)    # a stuck check is killed (T39)
        self.assertIn("Environment=SPOT_IDLE_MINUTES=45\n", service)
        self.assertIn("Environment=SPOT_LOG_DIR=%s\n" % self.logs, service)
        self.assertIn("ExecStart=/bin/bash %s check\n" % self.installed, service)
        with open(os.path.join(self.units, "spot-idle-stop.timer"), encoding="utf-8") as handle:
            timer = handle.read()
        for line in ("OnBootSec=5min", "OnUnitActiveSec=5min", "WantedBy=timers.target"):
            self.assertIn(line + "\n", timer)
        calls = self.host_calls()
        self.assertIn("systemctl daemon-reload\n", calls)
        self.assertIn("systemctl enable --now spot-idle-stop.timer\n", calls)
        self.assertIn("chown root:root %s\n" % self.installed, calls)
        for path in (self.installed, "spot-idle-stop.service", "spot-idle-stop.timer"):
            self.assertIn(path, result.stderr)                      # every change is printed
        self.assertIn("InstanceInitiatedShutdownBehavior", result.stderr)

    def test_apply_is_idempotent(self):
        """T39: a second apply rewrites nothing."""
        self.assertEqual(self.idle("apply").returncode, 0)
        second = self.idle("apply")
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual(second.stderr.count("unchanged"), 3)
        self.assertNotIn("writing", second.stderr)

    def test_dry_run_prints_and_changes_nothing(self):
        """T39: --dry-run shows the units and commands and touches nothing."""
        result = self.idle("apply --dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("would write %s/spot-idle-stop.timer" % self.units, result.stdout)
        self.assertIn("  OnUnitActiveSec=5min", result.stdout)
        self.assertIn("would run: systemctl enable --now spot-idle-stop.timer", result.stdout)
        self.assertFalse(os.path.exists(self.units))
        self.assertFalse(os.path.exists(self.installed))
        self.assertEqual(self.host_calls(), "")

    def test_remove_deletes_exactly_what_apply_added(self):
        """T39: remove disables the timer and deletes the units, the copy and the state; the
        power-off log stays."""
        self.assertEqual(self.idle("apply").returncode, 0)
        self.set_last_active(10)
        os.makedirs(os.path.dirname(self.idle_log))
        with open(self.idle_log, "w", encoding="utf-8") as handle:
            handle.write("earlier power-off\n")
        before = set(os.listdir(self.tmp)) - {"timer-on", "calls.log"}
        result = self.idle("remove")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("systemctl disable --now spot-idle-stop.timer\n", self.host_calls())
        self.assertEqual(os.listdir(self.units), [])
        self.assertFalse(os.path.exists(self.installed))
        self.assertFalse(os.path.exists(self.state))
        self.assertTrue(os.path.exists(self.idle_log))
        self.assertEqual(set(os.listdir(self.tmp)) - {"calls.log"}, before)
        again = self.idle("remove")
        self.assertEqual(again.returncode, 0)
        self.assertIn("nothing to remove", again.stderr)

    def test_status_reports_both_states(self):
        """T39: status exits 1 before apply and after remove, 0 while applied, and says what is
        keeping the host in use and when the installed copy is out of date."""
        before = self.idle("status")
        self.assertEqual(before.returncode, 1)
        self.assertIn("NOT APPLIED", before.stdout)
        self.assertEqual(self.idle("apply --minutes 90").returncode, 0)
        applied = self.idle("status", FAKE_PROCS=BACKGROUND + "\n" + SESSION)
        self.assertEqual(applied.returncode, 0, applied.stderr)
        self.assertIn("APPLIED (powers the host off after 90 idle minutes)", applied.stdout)
        self.assertIn("in use: a Session Manager session (pid 4242)", applied.stdout)
        self.assertNotIn("differs", applied.stdout)
        with open(self.installed, "a", encoding="utf-8") as handle:
            handle.write("# an older deploy\n")
        self.assertIn("differs from %s/host/idle_stop.sh: run apply again" % REPO, self.idle("status").stdout)
        self.assertEqual(self.idle("remove").returncode, 0)
        self.assertEqual(self.idle("status").returncode, 1)

    def test_bad_arguments(self):
        """T39: unknown actions or options, and idle times outside 15-1440 minutes, exit 2."""
        for args in ("", "enable", "apply --now", "status --dry-run", "check --minutes 30",
                     "apply --minutes", "apply --minutes 5", "apply --minutes 2000",
                     "apply --minutes 1e3", "apply --minutes -60"):
            with self.subTest(args=args):
                self.assertEqual(self.idle(args).returncode, 2)
        self.assertEqual(self.idle("check", SPOT_IDLE_MINUTES="10").returncode, 2)
        self.assertEqual(self.idle("apply", SPOT_LOG_DIR="/opt/spot demo/logs").returncode, 2)
        self.assertEqual(self.idle("apply", SPOT_IDLE_BIN="/usr/local/sbin/../bin/x").returncode, 2)
        self.assertFalse(os.path.exists(self.units))
        self.assertNotIn("shutdown", self.host_calls())


class TestT39IdleStopCheck(_IdleStopHost):
    """T39 (M39): host/idle_stop.sh check powers off only after the whole idle time with no
    Session Manager session or Run Command, no demo container, no agent or mission process
    and no open mission, and logs why; it never powers off when a check cannot run.

    The open-mission records are written by the real agent/mission.py in one test, so the
    pattern the script matches follows the code that writes them. Not covered: real
    ssm-session-worker and docker processes on the host; shutdown stopping the instance.
    """

    def check(self, procs="", containers="", **env):
        return self.idle("check", FAKE_PROCS=BACKGROUND + ("\n" + procs if procs else ""),
                         FAKE_DOCKER_PS=containers, **env)

    def assert_not_stopped(self, result):
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("shutdown", self.host_calls())

    def test_idle_for_the_whole_window_powers_off_and_logs_why(self):
        """T39: idle past the window -> shutdown -P now, with the reason in the log file."""
        self.set_last_active(61 * 60)
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertRegex(self.host_calls(), r"(?m)^shutdown -P now spot idle stop: idle for at least 60 minutes$")
        with open(self.idle_log, encoding="utf-8") as handle:
            line = handle.read()
        self.assertRegex(line, r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ powering off: idle for 61 minutes, at least 60 "
                               r"\(no Session Manager session or Run Command, no demo container, no agent or "
                               r"mission process and no open mission since 2026-10-30T09:00:00Z\)\n$")
        self.assertIn("powering off", result.stderr)                # and the journal

    def test_idle_within_the_window_or_first_seen_does_not_power_off(self):
        """T39: idle for less than the window, or no activity recorded since boot, keeps it up."""
        self.set_last_active(59 * 60)
        result = self.check()
        self.assert_not_stopped(result)
        self.assertIn("idle for 59 of 60 minutes", result.stderr)
        shutil.rmtree(self.state)                                   # /run is emptied at boot
        result = self.check()
        self.assert_not_stopped(result)
        self.assertIn("the idle clock starts now", result.stderr)
        self.assertLess(self.last_active_age(), 5)
        self.set_last_active(-3600)                                 # a clock that went back
        self.assert_not_stopped(self.check())
        self.assertLess(abs(self.last_active_age()), 5)

    def test_the_window_follows_the_unit(self):
        """T39: SPOT_IDLE_MINUTES (written into the unit by apply) sets the window."""
        self.set_last_active(61 * 60)
        self.assert_not_stopped(self.check(SPOT_IDLE_MINUTES="90"))
        self.set_last_active(91 * 60)
        self.check(SPOT_IDLE_MINUTES="90")
        self.assertIn("shutdown -P now", self.host_calls())

    def test_any_activity_keeps_the_host_up_and_restarts_the_clock(self):
        """T39: a Session Manager session, a Run Command, each demo container, the agent or a
        mission process each keep the host up and reset the idle clock."""
        cases = [("a Session Manager session (pid 4242)", {"procs": SESSION}),
                 ("a Run Command in progress (pid 4343)", {"procs": RUN_COMMAND}),
                 ("an agent or mission process (pid 5151)", {"procs": AGENT}),
                 ("an agent or mission process (pid 5252)", {"procs": MISSION})]
        cases += [("container %s is running" % name, {"containers": "other " + name})
                  for name in ("spot-sim", "spot-agent", "spot-mission", "spot-hud", "spot-rec")]
        for reason, kwargs in cases:
            with self.subTest(reason=reason):
                self.set_last_active(5 * 3600)
                result = self.check(**kwargs)
                self.assert_not_stopped(result)
                self.assertIn("in use, not stopping: " + reason, result.stderr)
                self.assertLess(self.last_active_age(), 5)
        self.set_last_active(5 * 3600)
        self.check(containers="buildkit spot-simulator")            # other containers are not the demo
        self.assertIn("shutdown -P now", self.host_calls())

    def test_a_check_that_cannot_run_never_powers_off(self):
        """T39: docker or pgrep missing or failing counts as in use (it cannot tell)."""
        for name, env in (("docker fails", {"FAKE_DOCKER_RC": "1"}),
                          ("pgrep fails", {"FAKE_PGREP_RC": "3"})):
            with self.subTest(name):
                self.set_last_active(5 * 3600)
                result = self.check(**env)
                self.assert_not_stopped(result)
                self.assertIn("cannot tell", result.stderr)
        os.remove(os.path.join(self.bin, "docker"))
        self.set_last_active(5 * 3600)
        path = os.pathsep.join([self.bin, os.path.dirname(BASH), "/usr/bin", "/bin"])
        if not any(os.access(os.path.join(d, "docker"), os.X_OK) for d in path.split(os.pathsep)):
            result = self.check(PATH=path)
            self.assert_not_stopped(result)
            self.assertIn("cannot tell: docker not found", result.stderr)

    def test_a_log_swapped_after_the_listing_is_not_read(self):
        """T39, T28: the logs directory is shared with the containers, which could swap a
        listed log for a FIFO, a device or a link before the root check reads it. A file
        that is no longer a regular file is not opened: the check says it cannot tell,
        reads nothing through a link, and does not power off."""
        fifo = os.path.join(self.logs, "demo-1.jsonl")
        os.mkfifo(fifo)
        secret = os.path.join(self.tmp, "root-only.txt")
        with open(secret, "w", encoding="utf-8") as handle:
            handle.write("not for the check\n")
        link = os.path.join(self.logs, "demo-2.jsonl")
        os.symlink(secret, link)
        # find(1) saw a regular file; by the time the check reads it, it is not one.
        self.add_fake("find", '#!/bin/sh\nprintf \'%s\\0\' "$FAKE_SWAPPED"\n')
        for path in (fifo, link):
            with self.subTest(os.path.basename(path)):
                self.set_last_active(5 * 3600)
                started = time.monotonic()
                result = self.check(FAKE_SWAPPED=path)
                self.assertLess(time.monotonic() - started, 20)
                self.assert_not_stopped(result)
                self.assertIn("cannot tell: %s is no longer a regular file" % path, result.stderr)

    def test_a_read_that_does_not_end_is_cut_off(self):
        """T39, T28: a read that would not end (a file swapped between the type check
        and the read) is killed after SPOT_IDLE_READ_TIMEOUT and counts as cannot tell."""
        self.write_log("demo-1.jsonl", [_record("summary", "COMPLETED")])
        self.add_fake("head", "#!/bin/sh\nexec sleep 30\n")
        self.set_last_active(5 * 3600)
        started = time.monotonic()
        result = self.check(SPOT_IDLE_READ_TIMEOUT="1")
        self.assertLess(time.monotonic() - started, 20)
        self.assert_not_stopped(result)
        self.assertIn("within 1s", result.stderr)

    def test_a_log_larger_than_the_cap_is_not_held(self):
        """T39, T28: at most SPOT_IDLE_READ_MAX_BYTES of a log are read; a larger one
        (or a FIFO that keeps feeding data) is cannot tell, never idle."""
        path = self.write_log("demo-1.jsonl", [_record("summary", "COMPLETED", seq) for seq in range(50)])
        self.set_last_active(5 * 3600)
        result = self.check(SPOT_IDLE_READ_MAX_BYTES="512")
        self.assert_not_stopped(result)
        self.assertIn("cannot tell: %s is larger than 512 bytes" % path, result.stderr)

    def test_an_open_mission_keeps_the_host_up(self):
        """T39: a recent mission log whose last record has an open permit, or whose last
        mission_start has no summary after it, keeps the host up; a closed or stale one does not."""
        for permit in ("PRE_ENTRY_TESTING", "ENTRY_AUTHORIZED", "ENTRY_IN_PROGRESS"):
            with self.subTest(permit=permit):
                path = self.write_log("demo-1.jsonl", [_record("waypoint", permit)])
                self.set_last_active(5 * 3600)
                result = self.check()
                self.assert_not_stopped(result)
                self.assertIn("open mission in %s (its last record has an open permit)" % path, result.stderr)
        self.write_log("demo-1.jsonl", [_record("mission_start", "NOT_STARTED"),
                                        _record("permit", "NOT_STARTED", 2)])
        self.set_last_active(5 * 3600)
        result = self.check()
        self.assert_not_stopped(result)
        self.assertIn("mission_start with no summary after it", result.stderr)
        # Closed: a summary after the last mission_start, and a terminal permit.
        self.write_log("demo-1.jsonl", [_record("mission_start", "NOT_STARTED"),
                                        _record("summary", "COMPLETED", 2)])
        self.set_last_active(5 * 3600)
        self.check()
        self.assertIn("shutdown -P now", self.host_calls())

    def test_a_stale_open_mission_does_not_block_forever(self):
        """T39: an open log not written within the idle window (a crashed run with nothing
        left running) no longer keeps the host up."""
        self.write_log("demo-crashed.jsonl", [_record("waypoint", "ENTRY_IN_PROGRESS")], age_s=2 * 3600)
        self.set_last_active(5 * 3600)
        self.check()
        self.assertIn("shutdown -P now", self.host_calls())

    def test_the_open_permit_pattern_matches_what_mission_py_writes(self):
        """T39: records written by agent/mission.py's own _event are read as open and closed
        correctly (the idle stop and the audit log format stay in step)."""
        from tests import test_mission as tm
        import mission as mission_mod
        path = os.path.join(self.logs, "mission-real.jsonl")
        clock = tm.ScaledClock(20.0)
        robot = tm.FakeRobot(clock)
        mission = mission_mod.ConfinedSpaceMission(robot, robot.site(), path, clock=clock,
                                                   require_permit=False)
        self.addCleanup(mission.close)

        def verdict():
            self.set_last_active(5 * 3600)
            result = self.check()
            return "shutdown -P now" not in self.host_calls(), result.stderr

        mission._event("probe", "a record before any entry")
        self.assertEqual(verdict()[0], False)                      # NOT_STARTED: closed
        with mission._lock:
            mission._permit = mission_mod.ENTRY_IN_PROGRESS
        mission._event("waypoint", "inside")
        kept, stderr = verdict()
        self.assertTrue(kept, stderr)
        self.assertIn("its last record has an open permit", stderr)
        with mission._lock:
            mission._permit = mission_mod.COMPLETED
        mission._event("mission_start", "a run starts")
        kept, stderr = verdict()
        self.assertTrue(kept, stderr)
        self.assertIn("mission_start with no summary after it", stderr)
        mission._event("summary", "Mission COMPLETED", {"record": "summary"})
        self.assertEqual(verdict()[0], False)

    def test_dry_run_never_powers_off(self):
        """T39: check --dry-run says what it would do and changes nothing."""
        self.set_last_active(5 * 3600)
        result = self.idle("check --dry-run", FAKE_PROCS=BACKGROUND)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("would power off: idle for 300 minutes", result.stdout)
        self.assertNotIn("shutdown", self.host_calls())
        self.assertFalse(os.path.exists(self.idle_log))


# --- the start helper ------------------------------------------------------------------

FAKE_AWS = r'''#!/usr/bin/env python3
"""Fake AWS CLI: start-instances fails with $FAKE_START_ERROR $FAKE_START_FAILS times, then
succeeds; describe-instances prints a type and zone; wait succeeds."""
import os, sys
args = sys.argv[1:]
with open(os.environ["FAKE_AWS_LOG"], "a") as log:
    log.write(" ".join(args) + "\n")
if "start-instances" in args:
    counter = os.environ["FAKE_AWS_COUNTER"]
    n = int(open(counter).read()) if os.path.exists(counter) else 0
    open(counter, "w").write(str(n + 1))
    if n < int(os.environ.get("FAKE_START_FAILS", "0")):
        sys.stderr.write("\nAn error occurred (%s) when calling the StartInstances operation: %s\n"
                         % (os.environ.get("FAKE_START_ERROR", "InsufficientInstanceCapacity"),
                            "We currently do not have sufficient g6.4xlarge capacity in the "
                            "Availability Zone you requested (us-west-2a)."))
        sys.exit(254)
    print("pending")
elif "describe-instances" in args:
    print("g6.4xlarge\tus-west-2a")
'''


@unittest.skipUnless(BASH, "needs bash")
class TestT39StartHostRetriesCapacity(unittest.TestCase):
    """T39 (M39): scripts/start-host.sh starts the instance, retrying only
    InsufficientInstanceCapacity with a doubling wait capped at 300 s, then waits for the
    status checks; after the last attempt it prints the Capacity Reservation commands and
    that a reservation bills while active. Not covered: the real EC2 API's error text
    (the fake prints the documented error code)."""

    INSTANCE = "i-0123456789abcdef0"

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="spot-start-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        bindir = os.path.join(self.tmp, "bin")
        os.makedirs(bindir)
        for name, body in (("aws", FAKE_AWS), ("sleep", '#!/bin/sh\necho "sleep $*" >> "$FAKE_AWS_LOG"\n')):
            with open(os.path.join(bindir, name), "w", encoding="utf-8") as handle:
                handle.write(body)
            os.chmod(os.path.join(bindir, name), 0o700)
        self.log = os.path.join(self.tmp, "aws.log")
        self.env = {k: v for k, v in os.environ.items() if not k.startswith(("SPOT_", "AWS_"))}
        self.env.update(PATH=os.pathsep.join([bindir, os.path.dirname(BASH), os.environ.get("PATH", "")]),
                        FAKE_AWS_LOG=self.log, FAKE_AWS_COUNTER=os.path.join(self.tmp, "starts"),
                        SPOT_SIM_INSTANCE=self.INSTANCE)

    def start(self, **env):
        merged = dict(self.env)
        merged.update(env)
        return subprocess.run(["bash", "scripts/start-host.sh"], cwd=REPO, env=merged,
                              capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL)

    def calls(self):
        if not os.path.exists(self.log):
            return []
        with open(self.log, encoding="utf-8") as handle:
            return handle.read().splitlines()

    def test_starts_and_waits_for_the_status_checks(self):
        """T39: one start, then the running and status-ok waiters."""
        result = self.start()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertEqual(len([c for c in calls if "start-instances" in c]), 1)
        self.assertIn("ec2 wait instance-running --region us-west-2 --instance-ids %s" % self.INSTANCE, calls)
        self.assertIn("ec2 wait instance-status-ok --region us-west-2 --instance-ids %s" % self.INSTANCE, calls)
        self.assertFalse([c for c in calls if c.startswith("sleep")])

    def test_capacity_errors_are_retried_with_a_doubling_wait(self):
        """T39: InsufficientInstanceCapacity is retried: waits of 30, 60, 120, 240, then 300 s."""
        result = self.start(FAKE_START_FAILS="5", SPOT_START_ATTEMPTS="6")
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertEqual([c for c in calls if c.startswith("sleep")],
                         ["sleep 30", "sleep 60", "sleep 120", "sleep 240", "sleep 300"])
        self.assertEqual(len([c for c in calls if "start-instances" in c]), 6)
        self.assertIn("start accepted on attempt 6", result.stdout)
        self.assertIn("ec2 wait instance-status-ok --region us-west-2 --instance-ids %s" % self.INSTANCE, calls)

    def test_after_the_last_attempt_it_prints_the_reservation_advice(self):
        """T39: still no capacity -> exit 3 with the create-capacity-reservation command for the
        instance's own type and zone, and the warning that it bills while active."""
        result = self.start(FAKE_START_FAILS="99", SPOT_START_ATTEMPTS="3", SPOT_START_BACKOFF="0")
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertEqual(len([c for c in self.calls() if "start-instances" in c]), 3)
        advice = _flat(result.stderr.replace("\\\n", " "))
        self.assertIn("no g6.4xlarge capacity in us-west-2a after 3 attempts", advice)
        self.assertIn("aws ec2 create-capacity-reservation --region us-west-2 --instance-type g6.4xlarge "
                      "--instance-platform Linux/UNIX --availability-zone us-west-2a --instance-count 1 "
                      "--instance-match-criteria open --end-date-type limited", advice)
        self.assertIn("bills at the On-Demand rate", advice)
        self.assertFalse([c for c in self.calls() if " wait " in " %s " % c])

    def test_other_errors_are_not_retried(self):
        """T39: a permission or state error stops at once with exit 1."""
        result = self.start(FAKE_START_FAILS="1", FAKE_START_ERROR="UnauthorizedOperation")
        self.assertEqual(result.returncode, 1)
        self.assertIn("not a capacity error, not retried", result.stderr)
        self.assertEqual(len([c for c in self.calls() if "start-instances" in c]), 1)

    def test_bad_settings_make_no_aws_call(self):
        """T39: a malformed instance id, Region, attempt count or wait exits 2 before any call."""
        for env in ({"SPOT_SIM_INSTANCE": "i-0;id"}, {"AWS_REGION": "us west 2"},
                    {"SPOT_START_ATTEMPTS": "0"}, {"SPOT_START_ATTEMPTS": "21"},
                    {"SPOT_START_BACKOFF": "301"}, {"SPOT_START_BACKOFF": "-1"}):
            with self.subTest(env=env):
                self.assertEqual(self.start(**env).returncode, 2)
        self.assertEqual(self.calls(), [])


# --- the documented steps -------------------------------------------------------------------

class TestT39CostDocs(unittest.TestCase):
    """T39 (M39, documented part): the RUNBOOK makes the guardrails and the idle stop
    first-time setup steps and a Capacity Reservation a required step for a scheduled
    live demo (create, check, cancel, and that it bills while active); the README's cost
    and Cleanup sections and the IAM README's host baseline point to them; infra/README.md
    describes each template (the session logging one is T38's)."""

    def setUp(self):
        self.runbook = _flat(_read("docs", "RUNBOOK.md"))
        self.readme = _flat(_read("README.md"))
        self.iam = _flat(_read("docs", "iam", "README.md"))
        self.infra = _read("infra", "README.md")

    def test_first_time_setup_deploys_the_guardrails_and_applies_the_idle_stop(self):
        """T39: both are setup steps, with the shutdown-behaviour check."""
        setup = self.runbook[self.runbook.index("### Once per host"):self.runbook.index("## 1. Start the instance")]
        self.assertIn("aws cloudformation deploy --template-file infra/cost-guardrails.yaml", setup)
        self.assertIn("sudo bash idle_stop.sh apply", setup)
        self.assertIn("--attribute instanceInitiatedShutdownBehavior", setup)

    def test_a_scheduled_demo_reserves_capacity(self):
        """T39: the reservation is required for a scheduled demo, with create, check and cancel
        commands and the billing warning."""
        section = self.runbook[self.runbook.index("## Capacity for a scheduled demo"):]
        section = section[:section.index(" ## ")]
        for needle in ("aws ec2 create-capacity-reservation", "--instance-match-criteria open",
                       "--end-date-type limited", "aws ec2 describe-capacity-reservations",
                       "aws ec2 cancel-capacity-reservation", "bills while it is active",
                       "Required for any scheduled live demo"):
            self.assertIn(needle, section)
        self.assertIn("scripts/start-host.sh", self.runbook)

    def test_readme_cost_and_cleanup(self):
        """T39: the cost section names the guardrails and the reservation; Cleanup cancels the
        reservation, removes the idle stop and deletes the guardrail stack."""
        cost = self.readme[self.readme.index("## AWS resources and cost"):
                           self.readme.index("## Known limitations and honest caveats")]
        for needle in ("infra/cost-guardrails.yaml", "host/idle_stop.sh", "Capacity Reservation"):
            self.assertIn(needle, cost)
        cleanup = self.readme[self.readme.index("## Cleanup"):self.readme.index("## Contributing")]
        for needle in ("aws ec2 cancel-capacity-reservation", "aws cloudformation delete-stack",
                       "idle_stop.sh remove"):
            self.assertIn(needle, cleanup)

    def test_iam_baseline_no_longer_leaves_it_to_the_adopter(self):
        """T39: the host baseline points to the shipped guardrails and sets the shutdown behaviour."""
        self.assertIn("infra/cost-guardrails.yaml", self.iam)
        self.assertIn("host/idle_stop.sh", self.iam)
        self.assertIn("| Shutdown behaviour | `stop`", self.iam)

    def test_infra_readme_covers_each_template(self):
        """T39, T38: each template's section says what it creates, its parameters, and how to
        deploy, check and delete it."""
        for heading in ("## Cost guardrails (`cost-guardrails.yaml`)",
                        "## Session logging (`session-logging.yaml`)"):
            self.assertIn("\n%s\n" % heading, self.infra)
        session = _flat(self.infra[self.infra.index("## Session logging"):])
        for needle in ("SessionLogKey", "SessionLogGroup", "LogGroupName", "RetentionInDays",
                       "aws cloudformation deploy", "--template-file infra/session-logging.yaml",
                       "aws logs describe-log-groups", "aws cloudformation delete-stack",
                       "DeletionPolicy: Retain"):
            self.assertIn(needle, session)
        flat = _flat(self.infra)
        for needle in ("Optional sample infrastructure", "InstanceId", "MonthlyBudgetUSD",
                       "NotificationEmail", "aws cloudformation deploy", "describe-stacks",
                       "aws cloudformation delete-stack", "uvx cfn-lint@1.59.0", "uvx checkov@3.4.0"):
            self.assertIn(needle, flat)


if __name__ == "__main__":
    unittest.main()
