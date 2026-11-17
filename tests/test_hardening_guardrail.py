"""Tests for the Amazon Bedrock guardrail the agent applies (threat model T6, T10,
T11, M11).

The CloudFormation template is parsed and its policies checked; nothing is deployed
and no AWS call is made. The rest reads the shipped IAM policy and the documents, so
that the guardrail cannot become opt-in again without a test failing. How the agent
resolves and records the guardrail is covered by ``TestModelPinAndGuardrail`` in
tests/test_hardening_agent.py.

Run from the repo root:

    python3 -m unittest tests.test_hardening_guardrail -v
"""

import json
import os
import re
import unittest

try:
    import yaml
except ImportError:   # in tests/requirements-ci.lock; the template tests skip without it
    yaml = None

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as handle:
        return handle.read()


def _flat(text):
    return " ".join(text.split())


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
class TestT11GuardrailTemplate(unittest.TestCase):
    """T6, T10, T11 (M11): infra/bedrock-guardrail.yaml is the guardrail the agent
    applies, with four policies, an immutable version and an optional alarm on
    interventions.

    PROMPT_ATTACK is an input-side filter, so its OutputStrength must be NONE.
    VIOLENCE and MISCONDUCT are deliberately LOW: the demo's own language is about
    lethal atmospheres, injury and rescue, and a higher strength blocks its subject
    matter. Not covered: deploying the stack or exercising a policy against a real
    prompt (cfn-lint and checkov were run; no account was touched).
    """

    def setUp(self):
        self.doc = _cfn_load(_read("infra", "bedrock-guardrail.yaml"))
        self.res = self.doc["Resources"]

    def of_type(self, kind):
        return {name: r["Properties"] for name, r in self.res.items() if r["Type"] == kind}

    def filters(self):
        guardrail, = self.of_type("AWS::Bedrock::Guardrail").values()
        return {f["Type"]: f for f in guardrail["ContentPolicyConfig"]["FiltersConfig"]}

    def test_one_guardrail_and_one_immutable_version(self):
        """T11: exactly one guardrail, and a version of it, so a session is screened
        by the reviewed policy and never by DRAFT."""
        guardrails = self.of_type("AWS::Bedrock::Guardrail")
        versions = self.of_type("AWS::Bedrock::GuardrailVersion")
        self.assertEqual((len(guardrails), len(versions)), (1, 1))
        (name, _guardrail), = guardrails.items()
        version, = versions.values()
        self.assertEqual(version["GuardrailIdentifier"], {"Fn::GetAtt": [name, "GuardrailId"]})
        for resource in self.res.values():
            self.assertEqual(resource.get("DeletionPolicy"), "Delete")

    def test_prompt_attack_is_filtered_hard_on_the_input_only(self):
        """T6, T10, T11: the operator's instruction is the untrusted text that
        reaches the model, so PROMPT_ATTACK is HIGH; it is input-side, so its
        output strength is NONE."""
        prompt_attack = self.filters()["PROMPT_ATTACK"]
        self.assertEqual((prompt_attack["InputStrength"], prompt_attack["OutputStrength"]),
                         ("HIGH", "NONE"))

    def test_the_harm_filters_are_set_for_a_confined_space_demo(self):
        """T11: hate, insults and sexual content are filtered hard both ways;
        violence and misconduct are LOW on purpose, with the reason in the file,
        because the demo talks about atmospheres that kill and about rescue."""
        filters = self.filters()
        for kind in ("HATE", "INSULTS", "SEXUAL"):
            self.assertEqual((filters[kind]["InputStrength"], filters[kind]["OutputStrength"]),
                             ("HIGH", "HIGH"), kind)
        for kind in ("VIOLENCE", "MISCONDUCT"):
            self.assertEqual((filters[kind]["InputStrength"], filters[kind]["OutputStrength"]),
                             ("LOW", "LOW"), kind)
        template = _flat(_read("infra", "bedrock-guardrail.yaml"))
        self.assertIn("LOW on purpose", template)

    def test_real_robot_operation_is_a_denied_topic(self):
        """T9, T43, T11: a second, probabilistic layer in front of the
        deterministic simulator-only gate, with examples of the ask."""
        guardrail, = self.of_type("AWS::Bedrock::Guardrail").values()
        topic, = guardrail["TopicPolicyConfig"]["TopicsConfig"]
        self.assertEqual((topic["Name"], topic["Type"]), ("RealRobotOperation", "DENY"))
        definition = _flat(topic["Definition"]).lower()
        for needle in ("real", "physical robot", "simulation-only"):
            self.assertIn(needle, definition)
        self.assertGreaterEqual(len(topic["Examples"]), 3)

    def test_profanity_and_sensitive_information_policies(self):
        """T37, T11: the managed profanity list (BR8) and no custom word list;
        personal data is masked and a credential in a prompt is blocked (BR9)."""
        guardrail, = self.of_type("AWS::Bedrock::Guardrail").values()
        word_policy = guardrail["WordPolicyConfig"]
        self.assertEqual([entry["Type"] for entry in word_policy["ManagedWordListsConfig"]],
                         ["PROFANITY"])
        self.assertNotIn("WordsConfig", word_policy)
        pii = {entry["Type"]: entry["Action"]
               for entry in guardrail["SensitiveInformationPolicyConfig"]["PiiEntitiesConfig"]}
        for kind in ("NAME", "ADDRESS", "EMAIL", "PHONE", "USERNAME"):
            self.assertEqual(pii[kind], "ANONYMIZE", kind)
        for kind in ("PASSWORD", "AWS_ACCESS_KEY", "AWS_SECRET_KEY"):
            self.assertEqual(pii[kind], "BLOCK", kind)

    def test_a_blocked_turn_tells_the_operator_what_happened(self):
        """T10, T11: the blocked-input and blocked-output messages say that the
        instruction or the reply was blocked and that the robot's state does not
        depend on it, rather than leaving the service's default wording."""
        guardrail, = self.of_type("AWS::Bedrock::Guardrail").values()
        blocked_input = _flat(guardrail["BlockedInputMessaging"])
        blocked_output = _flat(guardrail["BlockedOutputsMessaging"])
        self.assertIn("blocked", blocked_input)
        self.assertIn("has not changed", blocked_input)
        self.assertIn("blocked", blocked_output)
        for message in (blocked_input, blocked_output):
            self.assertLessEqual(len(message), 500)   # the service's limit

    def test_every_deployment_reports_on_interventions(self):
        """BR10, T11: the alarm and the dashboard are created unconditionally, so a
        guardrail is never deployed with nothing watching it. Neither carries a
        Condition; the topic only decides whether the alarm also notifies."""
        alarms = self.of_type("AWS::CloudWatch::Alarm")
        dashboards = self.of_type("AWS::CloudWatch::Dashboard")
        self.assertEqual((len(alarms), len(dashboards)), (1, 1))
        for name in list(alarms) + list(dashboards):
            self.assertNotIn("Condition", self.res[name], name)

    def test_the_alarm_watches_the_guardrails_own_metric(self):
        """BR10, T11: InvocationsIntervened for this guardrail and version; no data
        point is the normal state, so missing data does not breach. The topic is
        attached only when one was given, and an alarm with no actions is still an
        alarm."""
        (_name, alarm), = self.of_type("AWS::CloudWatch::Alarm").items()
        self.assertEqual((alarm["Namespace"], alarm["MetricName"], alarm["Statistic"]),
                         ("AWS/Bedrock/Guardrails", "InvocationsIntervened", "Sum"))
        self.assertEqual(alarm["TreatMissingData"], "notBreaching")
        self.assertEqual(alarm["ComparisonOperator"], "GreaterThanOrEqualToThreshold")
        self.assertEqual(sorted(d["Name"] for d in alarm["Dimensions"]),
                         ["GuardrailArn", "GuardrailVersion"])
        self.assertIn("WantInterventionAlarm", self.doc["Conditions"])
        self.assertEqual(alarm["AlarmActions"],
                         {"Fn::If": ["WantInterventionAlarm", [{"Ref": "AlarmTopicArn"}],
                                     {"Ref": "AWS::NoValue"}]})

    def test_the_dashboard_body_is_valid_json_that_reports_the_interventions(self):
        """BR10, T11: the dashboard renders to valid JSON and reports interventions
        against calls, interventions split by policy type, the latency the guardrail
        adds, and how to read an intervention. It stays inside the 50 metrics the
        CloudWatch free tier allows a dashboard."""
        (_name, dash), = self.of_type("AWS::CloudWatch::Dashboard").items()
        sub = dash["DashboardBody"]["Fn::Sub"]
        body, params = sub[0], sub[1]
        self.assertEqual(sorted(params), ["AlarmArn", "Arn", "Ver"])
        rendered = re.sub(r"\$\{([A-Za-z0-9:]+)\}", "X", body)
        doc = json.loads(rendered)          # CloudFormation would reject invalid JSON
        widgets = doc["widgets"]
        titles = [w["properties"].get("title", "") for w in widgets]
        self.assertTrue(any("intervention" in t.lower() for t in titles), titles)
        self.assertTrue(any("by policy" in t.lower() for t in titles), titles)
        self.assertTrue(any("latency" in t.lower() for t in titles), titles)
        metric_lines = sum(len(w["properties"]["metrics"])
                           for w in widgets if w["type"] == "metric")
        self.assertLessEqual(metric_lines, 50)
        self.assertGreaterEqual(metric_lines, 4)
        # Every guardrail policy type the stack configures has its own series.
        by_policy, = [w for w in widgets
                      if "by policy" in w["properties"].get("title", "").lower()]
        policies = {m[3] for m in by_policy["properties"]["metrics"]}
        self.assertEqual(policies, {"ContentPolicy", "TopicPolicy", "WordPolicy",
                                    "SensitiveInformationPolicy"})
        # The note says what a guardrail is not, so nobody reads the dashboard as a
        # safety instrument (T43).
        note, = [w for w in widgets if w["type"] == "text"]
        markdown = note["properties"]["markdown"]
        self.assertIn("not a safety function", markdown)
        self.assertIn("mission log", markdown)

    def test_the_outputs_are_what_the_host_and_the_role_need(self):
        """T11, T27: the id and version the host sets for the agent, and the ARN the
        instance-role statement needs."""
        outputs = self.doc["Outputs"]
        self.assertEqual(sorted(outputs), ["DashboardName", "GuardrailArn", "GuardrailId",
                                           "GuardrailVersion", "InterventionAlarmName"])
        for name, env in (("GuardrailId", "SPOT_GUARDRAIL_ID"),
                          ("GuardrailVersion", "SPOT_GUARDRAIL_VERSION")):
            self.assertIn(env, outputs[name]["Description"])
        self.assertIn("instance-role-policy.json", _flat(outputs["GuardrailArn"]["Description"]))


class TestT11GuardrailIsShippedOnByDefault(unittest.TestCase):
    """T11, T27 (M11): the instance role ships the permission the guardrail needs,
    the host wrapper passes the two variables into the container, and the documents
    say the guardrail is on by default rather than opt-in."""

    def setUp(self):
        self.policy = json.loads(_read("docs", "iam", "instance-role-policy.json"))
        self.iam = _flat(_read("docs", "iam", "README.md"))
        self.readme = _flat(_read("README.md"))
        self.runbook = _read("docs", "RUNBOOK.md")
        self.infra = _read("infra", "README.md")
        self.run_agent = _read("host", "run_agent.sh")

    def test_the_instance_role_may_apply_only_the_one_guardrail(self):
        """T11, T27: ApplyGuardrail is in the shipped policy, on one guardrail ARN,
        not on a wildcard."""
        statement, = [s for s in self.policy["Statement"]
                      if s["Sid"] == "ApplyTheOneGuardrail"]
        self.assertEqual(statement["Action"], "bedrock:ApplyGuardrail")
        self.assertEqual(statement["Effect"], "Allow")
        self.assertTrue(statement["Resource"].startswith("arn:aws:bedrock:us-west-2:"))
        self.assertTrue(statement["Resource"].endswith(":guardrail/<GUARDRAIL_ID>"))
        self.assertIn("bedrock:ApplyGuardrail", self.iam)
        self.assertIn("guardrail is on by default", self.iam)

    def test_the_host_wrapper_passes_the_guardrail_into_the_container(self):
        """T11: SPOT_GUARDRAIL_ID and SPOT_GUARDRAIL_VERSION reach the agent, and
        only when they are set, so an unset variable does not become an empty one."""
        for env in ("SPOT_GUARDRAIL_ID", "SPOT_GUARDRAIL_VERSION"):
            self.assertIn('${%s:+-e %s="$%s"}' % (env, env, env), self.run_agent)

    def test_the_documents_say_on_by_default_and_how_to_deploy_it(self):
        """T11: the README flag table, the runbook setup step and the infra README
        agree that the guardrail is on by default, name the template and the two
        variables, and say that --no-guardrail is the only way past it."""
        self.assertIn("**On by default**", self.readme)
        self.assertIn("`--no-guardrail` | off", self.readme)
        setup = self.runbook[self.runbook.index("### Once per account: the Bedrock guardrail"):
                             self.runbook.index("### Once per host")]
        for needle in ("--template-file infra/bedrock-guardrail.yaml", "SPOT_GUARDRAIL_ID",
                       "SPOT_GUARDRAIL_VERSION", "--no-guardrail", "ApplyTheOneGuardrail"):
            self.assertIn(needle, setup)
        self.assertIn("\n## Bedrock guardrail (`bedrock-guardrail.yaml`)\n", self.infra)
        section = _flat(self.infra[self.infra.index("## Bedrock guardrail"):
                                   self.infra.index("## Cost guardrails")])
        for needle in ("on by default", "refuses to start", "not a safety function",
                       "GuardrailName", "AlarmTopicArn", "InterventionThreshold",
                       "aws cloudformation deploy", "aws bedrock get-guardrail",
                       "aws cloudformation delete-stack"):
            self.assertIn(needle, section)

    def test_the_cleanup_deletes_the_guardrail_stack_last(self):
        """T11: the guardrail stack is in Cleanup, with the reason it goes last."""
        cleanup = self.readme[self.readme.index("## Cleanup"):
                              self.readme.index("## Contributing")]
        self.assertIn("delete-stack --stack-name spot-demo-bedrock-guardrail", cleanup)
        self.assertIn("Bedrock guardrail stack, last", cleanup)


if __name__ == "__main__":
    unittest.main()
