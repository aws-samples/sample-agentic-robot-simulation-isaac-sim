"""Tests for the documented baseline that adopters copy: the IAM policy files and the
release files. Each class docstring names the threat-model entry it covers.

Run from the repo root:

    python3 -m unittest tests.test_hardening_docs -v
"""

import json
import os
import re
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _policy(name):
    with open(os.path.join(REPO, "docs", "iam", name), encoding="utf-8") as handle:
        return json.load(handle)


def _actions(statement):
    actions = statement["Action"]
    return [actions] if isinstance(actions, str) else list(actions)


class TestT27InstanceRolePolicy(unittest.TestCase):
    """T27 and T26 (M27, M26 in part): the shipped instance-role policy is least privilege.

    Bedrock invoke on one inference profile, its foundation model only through that
    profile, the DCV licence bucket; no Secrets Manager, no wildcard action, no
    wildcard resource on Bedrock or S3 beyond the licence objects.
    """

    def setUp(self):
        self.doc = _policy("instance-role-policy.json")

    def test_no_secret_reads_and_no_wildcard_actions(self):
        for statement in self.doc["Statement"]:
            self.assertEqual(statement["Effect"], "Allow")
            for action in _actions(statement):
                self.assertNotIn("*", action)
                self.assertFalse(action.startswith("secretsmanager:"), action)

    def test_bedrock_is_one_profile_its_model_through_it_and_one_guardrail(self):
        bedrock = [s for s in self.doc["Statement"]
                   if any(a.startswith("bedrock:") for a in _actions(s))]
        self.assertEqual(len(bedrock), 3)
        profile, model, guardrail = bedrock
        self.assertEqual(profile["Resource"],
                         "arn:aws:bedrock:us-west-2:<ACCOUNT_ID>:inference-profile/"
                         "us.anthropic.claude-sonnet-5")
        self.assertEqual(model["Condition"]["StringEquals"]["bedrock:InferenceProfileArn"],
                         profile["Resource"])
        for statement in (profile, model):
            self.assertEqual(sorted(_actions(statement)),
                             ["bedrock:InvokeModel", "bedrock:InvokeModelWithResponseStream"])
        # The guardrail is on by default (threat model T11), so applying it is a shipped
        # permission, on one guardrail and not a wildcard.
        self.assertEqual(_actions(guardrail), ["bedrock:ApplyGuardrail"])
        self.assertEqual(guardrail["Resource"],
                         "arn:aws:bedrock:us-west-2:<ACCOUNT_ID>:guardrail/<GUARDRAIL_ID>")

    def test_only_the_dcv_licence_bucket(self):
        s3 = [s for s in self.doc["Statement"] if any(a.startswith("s3:") for a in _actions(s))]
        self.assertEqual([s["Resource"] for s in s3], ["arn:aws:s3:::dcv-license.us-west-2/*"])
        self.assertEqual(_actions(s3[0]), ["s3:GetObject"])

    def test_trust_policy_is_ec2_only(self):
        (statement,) = _policy("instance-role-trust-policy.json")["Statement"]
        self.assertEqual(statement["Principal"], {"Service": "ec2.amazonaws.com"})
        self.assertEqual(statement["Action"], "sts:AssumeRole")


class TestT29PresenterPolicy(unittest.TestCase):
    """T29 (M29): presenters get port forwarding to the tagged sim host and nothing else."""

    def setUp(self):
        self.doc = _policy("presenter-policy.json")

    def test_only_session_actions(self):
        actions = {a for s in self.doc["Statement"] for a in _actions(s)}
        self.assertEqual(actions, {"ssm:StartSession", "ssm:TerminateSession", "ssm:ResumeSession"})

    def test_start_session_needs_the_tag_and_the_port_forwarding_document(self):
        starts = [s for s in self.doc["Statement"] if _actions(s) == ["ssm:StartSession"]]
        resources = sorted(s["Resource"] for s in starts)
        self.assertEqual(resources, ["arn:aws:ec2:us-west-2:<ACCOUNT_ID>:instance/*",
                                     "arn:aws:ssm:us-west-2:<ACCOUNT_ID>:document/SpotDemo-PortForward"])
        instance = next(s for s in starts if ":instance/" in s["Resource"])
        self.assertEqual(instance["Condition"]["StringEquals"], {"ssm:resourceTag/spot-demo": "sim-host"})
        self.assertEqual(instance["Condition"]["BoolIfExists"],
                         {"ssm:SessionDocumentAccessCheck": "true"})

    def test_sessions_are_the_callers_own(self):
        manage = next(s for s in self.doc["Statement"] if "ssm:TerminateSession" in _actions(s))
        self.assertTrue(manage["Resource"].endswith(":session/${aws:userid}-*"))


class TestT29PortForwardingDocument(unittest.TestCase):
    """T29 and T1 (M29, M1): the shipped port-forwarding document lets presenters reach
    only DCV (8443) and the HUD (8080), on the same local port, and nothing else; it
    is otherwise AWS-StartPortForwardingSession."""

    def test_only_the_two_demo_ports(self):
        doc = _policy("port-forwarding-document.json")
        self.assertEqual(doc["sessionType"], "Port")
        self.assertEqual(doc["properties"], {"portNumber": "{{ portNumber }}", "type": "LocalPortForwarding",
                                             "localPortNumber": "{{ localPortNumber }}"})
        for name in ("portNumber", "localPortNumber"):
            pattern = re.compile(doc["parameters"][name]["allowedPattern"])
            for port in ("8443", "8080"):
                self.assertTrue(pattern.match(port), (name, port))
            for port in ("22", "80", "443", "8081", "18080", "84430", "0"):
                self.assertFalse(pattern.match(port), (name, port))


class TestT35ReleaseFiles(unittest.TestCase):
    """T35 (M35): the files a public aws-samples repository needs are present."""

    def test_release_files(self):
        for name in ("LICENSE", "NOTICE", "CODE_OF_CONDUCT.md", "CONTRIBUTING.md"):
            self.assertTrue(os.path.isfile(os.path.join(REPO, name)), name)
        with open(os.path.join(REPO, "LICENSE"), encoding="utf-8") as handle:
            self.assertTrue(handle.read().startswith("MIT No Attribution"))
        with open(os.path.join(REPO, "CONTRIBUTING.md"), encoding="utf-8") as handle:
            text = handle.read()
        self.assertIn("aws.amazon.com/security/vulnerability-reporting", text)
        self.assertIn("Please do **not** create a public github issue", text)


class TestT43NoticeAndScope(unittest.TestCase):
    """T43 (M43, documented part): the README opens with the non-production and
    not-a-safety-system notice, states that only the simulator is supported, and
    turns the real-robot roadmap into gates rather than steps. The code gate that
    refuses a non-simulator backend is tested in tests/test_hardening_agent.py."""

    def setUp(self):
        with open(os.path.join(REPO, "README.md"), encoding="utf-8") as handle:
            text = re.sub(r"(?m)^> ?", "", handle.read())   # unwrap blockquotes
        self.readme = " ".join(text.split())

    def test_notice_comes_first(self):
        head = self.readme[:self.readme.index("## What this is")]
        self.assertIn("This is sample code, for non-production usage.", head)
        self.assertIn("not a safety system", head)
        self.assertIn("it refuses to command a real robot", head)

    def test_roadmap_is_gates_not_steps(self):
        roadmap = self.readme[self.readme.index("## Roadmap to the real robot"):
                              self.readme.index("## Security")]
        self.assertIn("**Not supported by this sample.**", roadmap)
        self.assertIn("not steps to follow", roadmap)
        self.assertNotIn("--namespace /<spot_name>", roadmap)

    def test_security_and_cleanup_sections(self):
        for heading in ("## Security", "## Cleanup"):
            self.assertIn(heading, self.readme)
        cleanup = self.readme[self.readme.index("## Cleanup"):self.readme.index("## Contributing")]
        for item in ("terminate-instances", "delete-security-group", "delete-role",
                     "delete-secret", "~/spot-demo-logs"):
            self.assertIn(item, cleanup)


class TestT32KernelHeadersAreNotShipped(unittest.TestCase):
    """T32 (M32): both images remove linux-libc-dev, which carried every High and
    Critical advisory left in them and has no fix in Ubuntu 24.04. Nothing in the
    images compiles (Python is installed as wheels only)."""

    def test_both_images_purge_the_kernel_headers(self):
        for name in ("agent", "hud"):
            with open(os.path.join(REPO, name, "Dockerfile"), encoding="utf-8") as handle:
                text = handle.read()
            with self.subTest(image=name):
                self.assertIn("dpkg --purge --force-depends linux-libc-dev", text)
                self.assertIn("--only-binary :all:", text)


class TestT35ThirdPartyLicensesMatchTheLocks(unittest.TestCase):
    """T35 (M35): THIRD-PARTY-LICENSES lists every package the images install, at the
    version the lock files pin, so a dependency change cannot leave it stale."""

    def test_every_locked_package_is_listed_at_its_version(self):
        with open(os.path.join(REPO, "THIRD-PARTY-LICENSES"), encoding="utf-8") as handle:
            listed = dict(re.findall(r"^\| `([^`]+)` \| ([^ |]+) \|", handle.read(), re.M))
        for lock in ("agent/requirements.lock", "hud/requirements.lock"):
            with open(os.path.join(REPO, lock), encoding="utf-8") as handle:
                pinned = re.findall(r"^([A-Za-z0-9_.-]+)==([^ \\\n]+)", handle.read(), re.M)
            self.assertTrue(pinned, lock)
            for name, version in pinned:
                with self.subTest(lock=lock, package=name):
                    self.assertEqual(listed.get(name), version)


class TestT33CiDefinitions(unittest.TestCase):
    """T33 and T32 (M33, M32 in part): both CI definitions keep their safeguards. The GitHub
    workflow pins every action to a full commit SHA, runs with a read-only token and never
    on pull_request_target; both CIs install only hash-checked locks, and both run the
    dependency audit on the image locks, the GitHub workflow also on a weekly schedule."""

    def setUp(self):
        # Comments dropped and shell line continuations joined, so one command is one line.
        def load(*path):
            with open(os.path.join(REPO, *path), encoding="utf-8") as handle:
                text = re.sub(r"(?m)^\s*#.*\n", "", handle.read())
            return re.sub(r"\s*\\\n\s*", " ", text)
        self.github = load(".github", "workflows", "ci.yml")
        self.gitlab = load(".gitlab-ci.yml")

    def test_actions_are_pinned_and_the_token_is_read_only(self):
        uses = re.findall(r"uses: (\S+)", self.github)
        self.assertTrue(uses)
        for ref in uses:
            self.assertRegex(ref, r"@[0-9a-f]{40}$")
        self.assertIn("permissions:\n  contents: read\n", self.github)
        self.assertNotIn("pull_request_target", self.github)
        self.assertEqual(self.github.count("persist-credentials: false"),
                         self.github.count("actions/checkout@"))

    def test_only_hash_checked_installs(self):
        for name, text in (("github", self.github), ("gitlab", self.gitlab)):
            installs = re.findall(r"pip install[^\n]*", text)
            with self.subTest(ci=name):
                self.assertEqual(len(installs), 2)
                for line in installs:
                    self.assertIn("--require-hashes --no-deps --only-binary :all:", line)
                    self.assertRegex(line, r"-r tests/requirements-(ci|audit)\.lock'?$")

    def test_the_dependency_audit_covers_the_image_locks(self):
        audit = ("pip-audit --strict --disable-pip --require-hashes "
                 "-r agent/requirements.lock -r hud/requirements.lock")
        for name, text in (("github", self.github), ("gitlab", self.gitlab)):
            with self.subTest(ci=name):
                self.assertIn(audit, text)
        self.assertRegex(self.github, r"schedule:\n\s+- cron: \"[0-9 *]+\"")
        self.assertIn('$CI_PIPELINE_SOURCE == "schedule"', self.gitlab)
        with open(os.path.join(REPO, "tests", "requirements-audit.lock"), encoding="utf-8") as handle:
            lock = handle.read()
        self.assertRegex(lock, r"(?m)^pip-audit==[0-9.]+ \\$")

    def test_both_cis_scan_the_code_and_the_working_tree_for_secrets(self):
        """T33: a contributor's merge request passes a code scanner and a secret scanner
        in both CIs, and neither is fetched unpinned. bandit comes from the hash-checked
        audit lock; gitleaks is a release tarball, so its checksum is verified before it
        runs and the version in the URL is the version the checksum line names."""
        for name, text in (("github", self.github), ("gitlab", self.gitlab)):
            with self.subTest(ci=name):
                # The shipped code is gated at Medium, the tests at High: every Medium
                # this repository has is in tests/ and none is High.
                self.assertIn("bandit -q -r . --exclude ./tests,./.audit "
                              "--severity-level medium", text)
                self.assertIn("bandit -q -r tests --severity-level high", text)
                self.assertIn("gitleaks dir . --no-banner --redact", text)
                # Pinned: a 64-hex digest, checked with sha256sum before the binary runs.
                digests = re.findall(r"([0-9a-f]{64})  /tmp/gitleaks\.tar\.gz", text)
                self.assertEqual(len(digests), 1)
                self.assertIn("sha256sum -c -", text)
                urls = re.findall(r"https://github\.com/gitleaks/gitleaks/releases/"
                                  r"download/v([0-9.]+)/gitleaks_([0-9.]+)_linux_x64\.tar\.gz",
                                  text)
                self.assertEqual(len(urls), 1)
                self.assertEqual(urls[0][0], urls[0][1])   # tag and filename agree
                # The tarball is verified before anything is extracted or run.
                self.assertLess(text.index("sha256sum -c -"), text.index("gitleaks dir ."))
        with open(os.path.join(REPO, "tests", "requirements-audit.lock"), encoding="utf-8") as handle:
            lock = handle.read()
        self.assertRegex(lock, r"(?m)^bandit==[0-9.]+ \\$")

    def test_every_ci_command_parses_as_one_string(self):
        """Both CI files are valid YAML whose every command is a string. An unquoted
        ": " inside a command (pip's ``--only-binary :all: -r``) makes YAML read it as a
        mapping, and GitLab then rejects the whole pipeline before any job starts."""
        try:
            import yaml
        except ImportError:   # in tests/requirements-ci.lock, so CI always has it
            self.skipTest("needs PyYAML")
        with open(os.path.join(REPO, ".gitlab-ci.yml"), encoding="utf-8") as handle:
            gitlab = yaml.safe_load(handle)
        with open(os.path.join(REPO, ".github", "workflows", "ci.yml"), encoding="utf-8") as handle:
            github = yaml.safe_load(handle)
        commands = 0
        for name, job in gitlab.items():
            if not isinstance(job, dict) or "script" not in job:
                continue
            for key in ("before_script", "script", "after_script"):
                for item in job.get(key, []):
                    with self.subTest(ci="gitlab", job=name, key=key, item=item):
                        self.assertIsInstance(item, str)
                        commands += 1
        for name, job in github["jobs"].items():
            for step in job["steps"]:
                if "run" in step:
                    with self.subTest(ci="github", job=name, step=step.get("name")):
                        self.assertIsInstance(step["run"], str)
                        commands += 1
        self.assertGreater(commands, 10)

    def test_gitlab_gives_the_checkout_normal_modes_before_the_tests(self):
        """GitLab's Docker executor checks out under umask 0000, which leaves every file
        world-writable and fails the T28 mode check; the tests job removes group and
        other write before anything else runs."""
        try:
            import yaml
        except ImportError:
            self.skipTest("needs PyYAML")
        with open(os.path.join(REPO, ".gitlab-ci.yml"), encoding="utf-8") as handle:
            tests = yaml.safe_load(handle)["tests"]
        self.assertEqual(tests["before_script"][0], "chmod -R go-w .")


if __name__ == "__main__":
    unittest.main()
