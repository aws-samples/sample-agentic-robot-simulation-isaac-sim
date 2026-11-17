"""Tests for the hardening of the Mac-side scripts in scripts/.

Each class docstring names the threat-model entry (docs/security/THREAT_MODEL.md)
it covers. The scripts run in a throwaway git repository against a local fake of
the AWS CLI that records its arguments and the request files it is given and
answers every command with Success: no AWS call is made and this checkout is
never modified.

Run from the repo root:

    python3 -m unittest tests.test_hardening_scripts -v
"""

import base64
import glob
import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHIPPED = ("sim", "host", "agent", "hud", "tests", "tools")

FAKE_AWS = r'''#!/usr/bin/env python3
import os, shutil, sys
args = sys.argv[1:]
with open(os.environ["FAKE_AWS_LOG"], "a") as log:
    log.write("\t".join(args) + "\n")
for a in args:
    if a.startswith("file://"):
        n = len(os.listdir(os.environ["FAKE_AWS_REQS"]))
        shutil.copy(a[len("file://"):], os.path.join(os.environ["FAKE_AWS_REQS"], "%04d.json" % n))
if "send-command" in args:
    print("cid-1")
elif "Status" in args:
    print("Success")
elif "StandardOutputContent" in args:
    print(os.environ.get("FAKE_AWS_STDOUT", "remote output"))
'''


@unittest.skipUnless(shutil.which("bash") and shutil.which("git") and shutil.which("shasum"),
                     "needs bash, git and shasum")
class _ScriptInTempRepo(unittest.TestCase):
    """A throwaway git repo holding a copy of scripts/ and one file per shipped dir."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.repo = os.path.join(self.tmp, "repo")
        shutil.copytree(os.path.join(REPO, "scripts"), os.path.join(self.repo, "scripts"))
        for d in SHIPPED:
            os.makedirs(os.path.join(self.repo, d))
            with open(os.path.join(self.repo, d, "keep.txt"), "w") as handle:
                handle.write(d + "\n")
        subprocess.run(["git", "init", "-q"], cwd=self.repo, check=True)
        subprocess.run(["git", "add", "-A"], cwd=self.repo, check=True)
        subprocess.run(["git", "-c", "user.email=t@example.com", "-c", "user.name=t",
                        "commit", "-q", "-m", "init"], cwd=self.repo, check=True)
        self.commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self.repo, check=True,
                                     capture_output=True, text=True).stdout.strip()
        bindir = os.path.join(self.tmp, "bin")
        os.makedirs(bindir)
        with open(os.path.join(bindir, "aws"), "w") as handle:
            handle.write(FAKE_AWS)
        os.chmod(os.path.join(bindir, "aws"), 0o755)
        self.log = os.path.join(self.tmp, "aws.log")
        self.reqs = os.path.join(self.tmp, "reqs")
        os.makedirs(self.reqs)
        self.path = bindir + os.pathsep + os.environ.get("PATH", "")

    def run_script(self, name, *args, **env):
        full = {k: v for k, v in os.environ.items() if not k.startswith(("SPOT_", "AWS_"))}
        full.update(PATH=self.path, FAKE_AWS_LOG=self.log, FAKE_AWS_REQS=self.reqs, **env)
        return subprocess.run(["bash", os.path.join(self.repo, "scripts", name), *args],
                              capture_output=True, text=True, env=full, timeout=120)

    def requests(self):
        out = []
        for path in sorted(glob.glob(os.path.join(self.reqs, "*.json"))):
            with open(path) as handle:
                out.append(json.load(handle))
        return out


class TestT30DeployProvenanceAndPaths(_ScriptInTempRepo):
    """T30 (M30): deploy-to-host.sh ships a named commit and validates every path.

    It refuses a dirty tree unless --allow-dirty, records the commit in VERSION,
    refuses install paths that are malformed or under a system directory, builds
    every SSM request with a JSON encoder, stages in a random 0700 directory that
    must not exist yet, and wipes tools/ like every other shipped dir.
    """

    INSTANCE = "i-0123456789abcdef0"

    def test_refuses_a_dirty_tree(self):
        with open(os.path.join(self.repo, "tools", "scratch.sh"), "w") as handle:
            handle.write("echo hi\n")
        result = self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=self.INSTANCE)
        self.assertEqual(result.returncode, 1)
        self.assertIn("uncommitted or untracked", result.stderr)
        self.assertEqual(self.requests(), [])

    def test_allow_dirty_is_recorded_as_dirty(self):
        with open(os.path.join(self.repo, "tools", "scratch.sh"), "w") as handle:
            handle.write("echo hi\n")
        result = self.run_script("deploy-to-host.sh", "--allow-dirty", SPOT_SIM_INSTANCE=self.INSTANCE)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("%s-dirty" % self.commit, result.stdout)

    def test_rejects_bad_install_dirs_before_any_aws_call(self):
        for bad in ("/", "/opt", "/usr", "/usr/local/spot", "/etc/spot", "/var/lib/spot",
                    "/opt/a b", "/opt/../etc", "/opt/./x", "/opt/x;rm -rf /", "opt/spot",
                    "/opt/spot%s", "/opt/$(id)"):
            with self.subTest(dir=bad):
                result = self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=self.INSTANCE,
                                         SPOT_DEMO_DIR=bad)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("SPOT_DEMO_DIR", result.stderr)
        self.assertFalse(os.path.exists(self.log))

    def test_rejects_a_bad_instance_id(self):
        for bad in ("i-0;id", "i-XYZ", "my-host", "i-0123 i-0456"):
            with self.subTest(instance=bad):
                result = self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=bad)
                self.assertEqual(result.returncode, 2)
        self.assertFalse(os.path.exists(self.log))

    def test_requests_are_valid_json_with_a_random_0700_stage(self):
        result = self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=self.INSTANCE)
        self.assertEqual(result.returncode, 0, result.stderr)
        reqs = self.requests()
        self.assertGreaterEqual(len(reqs), 3)
        for req in reqs:
            self.assertEqual(req["InstanceIds"], [self.INSTANCE])
            self.assertEqual(req["DocumentName"], "AWS-RunShellScript")
        stage_cmds = reqs[0]["Parameters"]["commands"]
        self.assertEqual(stage_cmds[0], "umask 077")
        match = re.fullmatch(r"mkdir -m 0700 '(/tmp/spot-demo-deploy\.[0-9a-f]{32})'", stage_cmds[1])
        self.assertIsNotNone(match, stage_cmds)
        stage = match.group(1)
        for req in reqs[1:-1]:
            self.assertTrue(req["Parameters"]["commands"][0].endswith(">> '%s/payload.b64'" % stage))
        unpack = reqs[-1]["Parameters"]["commands"]
        # T30: the install directory itself, and recursively only what was shipped.
        self.assertIn("sudo chown ubuntu:ubuntu '/opt/spot-demo'", unpack)
        self.assertNotIn("sudo chown -R ubuntu:ubuntu '/opt/spot-demo'", unpack)
        chown = next(c for c in unpack if c.startswith("sudo chown -R "))
        self.assertEqual(chown, "sudo chown -R ubuntu:ubuntu " + " ".join(
            "'/opt/spot-demo/%s'" % d for d in SHIPPED) + " '/opt/spot-demo/VERSION'")
        self.assertIn("rm -rf '%s'" % stage, unpack)
        wipe = next(c for c in unpack if c.startswith("sudo rm -rf "))
        for d in SHIPPED:
            self.assertIn("'/opt/spot-demo/%s'" % d, wipe)
        self.assertNotIn("logs", wipe)

    def test_the_stage_name_differs_per_run(self):
        names = set()
        for _ in range(2):
            self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=self.INSTANCE)
        for req in self.requests():
            for cmd in req["Parameters"]["commands"]:
                names.update(re.findall(r"/tmp/spot-demo-deploy\.[0-9a-f]{32}", cmd))
        self.assertEqual(len(names), 2)

    def test_payload_carries_version_and_only_committed_files(self):
        result = self.run_script("deploy-to-host.sh", SPOT_SIM_INSTANCE=self.INSTANCE)
        self.assertEqual(result.returncode, 0, result.stderr)
        chunks = [r["Parameters"]["commands"][0] for r in self.requests()[1:-1]]
        b64 = "".join(re.match(r"printf '%s' '([A-Za-z0-9+/=]*)' >> ", c).group(1) for c in chunks)
        tgz = os.path.join(self.tmp, "payload.tgz")
        with open(tgz, "wb") as handle:
            handle.write(base64.b64decode(b64))
        listing = subprocess.run(["tar", "tzf", "payload.tgz"], cwd=self.tmp, check=True,
                                 capture_output=True, text=True).stdout.split()
        self.assertIn("VERSION", listing)
        self.assertEqual(sorted(set(p.split("/")[0] for p in listing)), sorted(("VERSION",) + SHIPPED))
        out = subprocess.run(["tar", "xzOf", "payload.tgz", "VERSION"], cwd=self.tmp, check=True,
                             capture_output=True, text=True).stdout
        self.assertIn("commit=%s\n" % self.commit, out)


class TestT36FetchDestinationOutsideTheCheckout(_ScriptInTempRepo):
    """T36 (M36) and T37 (M37): fetch-log.sh keeps operator data out of the repo.

    The default destination is ~/spot-demo-logs/<instance id>, not <repo>/logs; the
    host-side temp file has a random name and is created 0600; every input is
    checked against a plain pattern before any AWS call.
    """

    INSTANCE = "i-0123456789abcdef0"

    def test_default_destination_is_outside_the_checkout(self):
        with open(os.path.join(REPO, "scripts", "fetch-log.sh")) as handle:
            text = handle.read()
        self.assertIn('DEST="${FETCH_DEST:-$HOME/spot-demo-logs/$INSTANCE_ID}"', text)
        self.assertNotIn("$REPO_ROOT/logs", text)

    def test_pack_step_uses_a_private_random_temp_file(self):
        result = self.run_script("fetch-log.sh", SPOT_SIM_INSTANCE=self.INSTANCE,
                                 HOME=self.tmp, FAKE_AWS_STDOUT="NO-LOGDIR")
        self.assertEqual(result.returncode, 0, result.stderr)
        pack = self.requests()[0]["Parameters"]["commands"]
        tar = next(c for c in pack if c.startswith("tar czf"))
        self.assertRegex(tar, r"> /tmp/spot-logs-[0-9a-f]{32}\.b64$")
        self.assertLess(pack.index("umask 077"), pack.index(tar))

    def test_rejects_bad_inputs_before_any_aws_call(self):
        for env in ({"SPOT_SIM_INSTANCE": "i-0;id"},
                    {"SPOT_SIM_INSTANCE": self.INSTANCE, "SPOT_DEMO_DIR": "/opt/x y"},
                    {"SPOT_SIM_INSTANCE": self.INSTANCE, "AWS_REGION": "us-west-2;id"}):
            with self.subTest(env=env):
                result = self.run_script("fetch-log.sh", HOME=self.tmp, **env)
                self.assertEqual(result.returncode, 2, result.stderr)
        result = self.run_script("fetch-log.sh", "../../etc/*.txt", HOME=self.tmp,
                                 SPOT_SIM_INSTANCE=self.INSTANCE)
        self.assertEqual(result.returncode, 2)
        self.assertFalse(os.path.exists(self.log))


class TestT37SsmrunLeavesNothingOnTheHost(_ScriptInTempRepo):
    """T37 (M37): ssmrun.sh runs the script from a fresh mktemp directory on the host
    and removes it, output included, after the run; the request is JSON-encoded."""

    INSTANCE = "i-0123456789abcdef0"

    def test_request_cleans_up_its_directory(self):
        script = os.path.join(self.tmp, "job.sh")
        with open(script, "w") as handle:
            handle.write("echo hello\n")
        result = self.run_script("ssmrun.sh", script, "120", SPOT_SIM_INSTANCE=self.INSTANCE,
                                 FAKE_AWS_STDOUT="hello")
        self.assertEqual(result.returncode, 0, result.stderr)
        req = self.requests()[0]
        self.assertEqual(req["TimeoutSeconds"], 120)
        (cmd,) = req["Parameters"]["commands"]
        self.assertTrue(cmd.startswith("J=$(mktemp -d) && "))
        self.assertIn('rm -rf "$J"; exit $rc', cmd)
        self.assertNotIn("/tmp/job-", cmd)
        b64 = re.search(r"echo ([A-Za-z0-9+/=]+) \| base64 -d", cmd).group(1)
        self.assertEqual(base64.b64decode(b64), b"echo hello\n")

    def test_rejects_a_bad_instance_or_timeout(self):
        script = os.path.join(self.tmp, "job.sh")
        with open(script, "w"):
            pass
        for env, args in (({"SPOT_SIM_INSTANCE": "i-0;id"}, [script]),
                          ({"SPOT_SIM_INSTANCE": self.INSTANCE}, [script, "10;id"])):
            with self.subTest(env=env, args=args):
                result = self.run_script("ssmrun.sh", *args, **env)
                self.assertEqual(result.returncode, 2)
        self.assertFalse(os.path.exists(self.log))


class TestT2ConnectHudKeepsThePort(_ScriptInTempRepo):
    """T2 (M2): the HUD answers only localhost:<its port>, so connect-hud.sh refuses a
    local port that differs from the HUD's and never opens the tunnel."""

    def test_mismatched_ports_are_refused(self):
        result = self.run_script("connect-hud.sh", SPOT_SIM_INSTANCE="i-0123456789abcdef0",
                                 LOCAL_PORT="18080")
        self.assertEqual(result.returncode, 2)
        self.assertIn("must equal REMOTE_PORT", result.stderr)
        self.assertFalse(os.path.exists(self.log))

    def test_matching_ports_open_the_tunnel(self):
        result = self.run_script("connect-hud.sh", SPOT_SIM_INSTANCE="i-0123456789abcdef0")
        self.assertEqual(result.returncode, 0, result.stderr)
        with open(self.log) as handle:
            self.assertIn("portNumber=8080,localPortNumber=8080", handle.read())


if __name__ == "__main__":
    unittest.main()
