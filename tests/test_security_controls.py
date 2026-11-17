"""Tests for the controls the threat model recorded as present at 63c05ef, before the
hardening pass, kept so that a later change cannot quietly remove them.

One class per register entry whose control could be checked without ROS, Isaac Sim,
AWS or the network, plus one class for T43 that pins the README disclosures. Each
class docstring names the threat ID. The controls added by the hardening pass are
tested in tests/test_hardening_{agent,docs,host,hud,scripts,sim}.py and
tests/test_threshold_copies.py; section 4.1 of docs/security/THREAT_MODEL.md maps
every control to its tests.

Not covered here:
  - The no-ingress security group is built by hand from docs/iam/README.md; there is
    no IaC, so nothing in the repository can check it.

Shell scripts run against a local fake of the AWS CLI that only records its
arguments: no AWS call is made. Other script checks read the file text.

Run from the repo root:

    python3 -m unittest tests.test_security_controls -v

Needs: fastapi, httpx (TestClient), pillow (same as tests/test_hud.py).
"""

import ast
import csv
import io
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from agent import spot_agent  # noqa: E402
from agent.spot_agent import SpotTools, TranscriptCallback, sanitize_speech  # noqa: E402
from hud import server  # noqa: E402

try:  # `discover -s tests` puts tests/ on sys.path; `-m unittest tests.x` does not
    from test_agent_tools import FakeMission, FakeRobot
except ImportError:  # pragma: no cover - depends on how the suite is launched
    from tests.test_agent_tools import FakeMission, FakeRobot

rb = server.ros_bridge


def _read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as handle:
        return handle.read()


def _hud_client(state=None):
    """TestClient around a FakeBridge that is never ticked (no synthetic events).

    Requests carry the browser's Host, localhost:8080: the HUD answers no other (T2)."""
    bridge = rb.FakeBridge(state=state or rb.HudState(), realtime=False)
    return TestClient(server.create_app(bridge), base_url="http://localhost:8080")


# Records every call (argv) and returns success; `exec aws ...` ends the script.
FAKE_AWS = r'''#!/usr/bin/env python3
import json, os, sys
with open(os.environ["FAKE_AWS_LOG"], "a") as log:
    log.write(json.dumps(sys.argv[1:]) + "\n")
'''


class _FakeAwsScript(unittest.TestCase):
    """Run a scripts/*.sh helper with a fake `aws` first on PATH."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="secctl-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        bindir = os.path.join(self.tmp, "bin")
        os.makedirs(bindir)
        aws = os.path.join(bindir, "aws")
        with open(aws, "w") as handle:
            handle.write(FAKE_AWS)
        os.chmod(aws, 0o755)
        self.log = os.path.join(self.tmp, "calls.jsonl")
        self.path = bindir + os.pathsep + os.environ.get("PATH", "")

    def run_script(self, name, **env):
        full = {k: v for k, v in os.environ.items() if k not in ("SPOT_SIM_INSTANCE", "LOCAL_PORT")}
        full.update(PATH=self.path, FAKE_AWS_LOG=self.log, **env)
        return subprocess.run(["bash", os.path.join(REPO, "scripts", name)], env=full,
                              capture_output=True, text=True, timeout=30)

    def calls(self):
        import json
        if not os.path.exists(self.log):
            return []
        with open(self.log) as handle:
            return [json.loads(line) for line in handle]

    @staticmethod
    def opt(argv, name):
        return argv[argv.index(name) + 1]


# ---------------------------------------------------------------------------
# T1 (partial)
# ---------------------------------------------------------------------------


class TestT1DcvOnlyThroughSsmTunnel(_FakeAwsScript):
    """T1: DCV console reached only through an IAM-gated SSM port forward.

    Present since 63c05ef: scripts/connect-dcv.sh opens an SSM port-forwarding session
    to the instance's 8443 and needs an explicit instance id; it does not fetch the DCV
    password itself. The hardening pass added host/configure_dcv.sh (loopback-only
    listeners, QUIC off, one viewer) and the 8443/8080-only port-forwarding document;
    those are tested in tests/test_hardening_host.py and tests/test_hardening_docs.py.
    """

    def test_tunnel_is_an_ssm_port_forward_to_8443(self):
        result = self.run_script("connect-dcv.sh", SPOT_SIM_INSTANCE="i-0123456789abcdef0",
                                 AWS_REGION="us-west-2")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.calls()
        self.assertEqual(argv[:2], ["ssm", "start-session"])
        self.assertEqual(self.opt(argv, "--target"), "i-0123456789abcdef0")
        self.assertEqual(self.opt(argv, "--document-name"), "AWS-StartPortForwardingSession")
        self.assertEqual(self.opt(argv, "--parameters"), "portNumber=8443,localPortNumber=8443")

    def test_never_reads_the_password_secret(self):
        result = self.run_script("connect-dcv.sh", SPOT_SIM_INSTANCE="i-0123456789abcdef0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse([c for c in self.calls() if "secretsmanager" in c])
        # The script prints how to fetch it; the value never passes through it.
        self.assertIn("get-secret-value", result.stdout)

    def test_refuses_without_an_instance_id(self):
        result = self.run_script("connect-dcv.sh")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])


# ---------------------------------------------------------------------------
# T2 (partial)
# ---------------------------------------------------------------------------


class TestT2HudLoopbackOriginAndReadOnly(_FakeAwsScript):
    """T2: the HUD binds loopback by default, checks WebSocket Origin, and is read-only.

    --host defaults to 127.0.0.1, the image CMD passes it, run_hud.sh publishes no
    port; only GET routes plus /ws; no ROS publishers. Since the hardening pass the
    HUD answers only the loopback Host names on its own port and /ws needs an exact
    loopback Origin (a missing one is refused); the Host allowlist, the caps and the
    bind check are tested in tests/test_hardening_hud.py. By owner decision there is
    no token, so a process on the host can still read the HUD (T24).
    """

    def test_cli_default_bind_is_loopback(self):
        self.assertEqual(server.parse_args([]).host, "127.0.0.1")

    def test_image_and_wrapper_keep_loopback(self):
        dockerfile = _read("hud", "Dockerfile")
        cmd = [line for line in dockerfile.splitlines() if line.startswith("CMD ")]
        self.assertEqual(len(cmd), 1)
        self.assertIn('"--host", "127.0.0.1"', cmd[0])
        run_hud = _read("host", "run_hud.sh")
        run = run_hud[run_hud.index("docker run -d --name"):]
        run = run[:run.index(">/dev/null")]
        self.assertNotRegex(run, r"\s(-p|--publish)\b")
        # Nothing after the image name: the CMD (and its --host) is not overridden.
        self.assertTrue(run.rstrip().endswith('"$IMAGE"'), run)
        for rel in ("hud", "host"):
            for root, _dirs, files in os.walk(os.path.join(REPO, rel)):
                for name in files:
                    if name.endswith((".py", ".sh", "Dockerfile", ".md")):
                        with open(os.path.join(root, name), encoding="utf-8") as handle:
                            self.assertNotRegex(handle.read(), r"\b0\.0\.0\.0\b", name)

    def test_ssm_tunnel_targets_the_loopback_hud_port(self):
        result = self.run_script("connect-hud.sh", SPOT_SIM_INSTANCE="i-0123456789abcdef0")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.calls()
        self.assertEqual(self.opt(argv, "--document-name"), "AWS-StartPortForwardingSession")
        self.assertEqual(self.opt(argv, "--parameters"), "portNumber=8080,localPortNumber=8080")
        self.assertNotEqual(self.run_script("connect-hud.sh").returncode, 0)

    def _ws_accepted(self, headers):
        with _hud_client() as client:
            try:
                with client.websocket_connect("ws://localhost:8080/ws", headers=headers) as ws:
                    return ws.receive_json()["type"] == "hello"
            except WebSocketDisconnect:
                return False

    def test_cross_origin_websocket_refused(self):
        for origin in ("http://evil.example", "http://localhost:9999", "null",
                       "http://localhost.evil.example:8080", "file://"):
            self.assertFalse(self._ws_accepted({"origin": origin}), origin)
        self.assertFalse(self._ws_accepted({}))   # no Origin: refused since M2

    def test_same_origin_websocket_served(self):
        self.assertTrue(self._ws_accepted({"origin": "http://localhost:8080"}))
        self.assertTrue(self._ws_accepted({"origin": "HTTP://LOCALHOST:8080"}))
        self.assertTrue(self._ws_accepted({"origin": "http://127.0.0.1:8080"}))

    def test_only_reads_are_routed(self):
        app = server.create_app(rb.FakeBridge(state=rb.HudState(), realtime=False))
        for route in app.routes:
            methods = getattr(route, "methods", None)
            if methods:   # HTTP routes; the /static mount and /ws have none
                self.assertLessEqual(set(methods), {"GET", "HEAD"}, route.path)
        with _hud_client() as client:
            for path in ("/api/state", "/api/audit.csv", "/api/audit.jsonl"):
                for method in ("post", "put", "delete"):
                    self.assertEqual(getattr(client, method)(path).status_code, 405, (method, path))
            for path in ("/docs", "/redoc", "/openapi.json"):
                self.assertEqual(client.get(path).status_code, 404, path)

    def test_hud_never_publishes_or_calls_ros(self):
        hud_dir = os.path.join(REPO, "hud")
        for name in sorted(os.listdir(hud_dir)):
            if not name.endswith(".py"):
                continue
            tree = ast.parse(_read("hud", name), filename=name)
            called = {node.func.attr for node in ast.walk(tree)
                      if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
            for forbidden in ("create_publisher", "create_client", "create_service",
                              "create_action_client"):
                self.assertNotIn(forbidden, called, "%s calls %s" % (name, forbidden))


# ---------------------------------------------------------------------------
# T4 (fixed)
# ---------------------------------------------------------------------------


class TestT4UntrustedTextInHudAndCsv(unittest.TestCase):
    """T4: LLM and ROS text cannot become script in the HUD or a formula in the CSV.

    Present since 63c05ef: the frontend sets text with textContent only, every
    response carries a CSP with script-src 'self', and text cells in the CSV export go
    through audit.safe_cell. The CSP header and the full-row CSV check had no test
    before this file; refused requests carrying the same headers are tested in
    tests/test_hardening_hud.py.
    """

    FORMULA = ("=", "+", "-", "@", "\t", "\r")

    def test_csp_and_nosniff_on_every_response(self):
        with _hud_client() as client:
            for path in ("/", "/api/state", "/static/app.js", "/api/audit.csv", "/no-such-page"):
                resp = client.get(path)
                csp = resp.headers.get("content-security-policy", "")
                directives = {d.strip().split()[0]: d.strip().split()[1:]
                              for d in csp.split(";") if d.strip()}
                self.assertEqual(directives.get("script-src"), ["'self'"], path)
                self.assertEqual(directives.get("default-src"), ["'self'"], path)
                self.assertEqual(directives.get("frame-ancestors"), ["'none'"], path)
                self.assertEqual(directives.get("base-uri"), ["'none'"], path)
                self.assertNotIn("unsafe-inline", csp, path)
                self.assertNotIn("unsafe-eval", csp, path)
                self.assertEqual(resp.headers.get("x-content-type-options"), "nosniff", path)

    def test_page_has_no_inline_script_or_handlers(self):
        html = _read("hud", "static", "index.html")
        scripts = re.findall(r"<script\b[^>]*>", html, re.I)
        self.assertEqual(scripts, ['<script src="/static/app.js" defer>'])
        bodies = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.I | re.S)
        self.assertEqual([b.strip() for b in bodies], [""])       # no inline body
        self.assertNotRegex(html, r"\son[a-z]+\s*=")              # no onclick= etc.
        self.assertNotIn("javascript:", html)

    def test_frontend_has_no_code_or_html_sinks(self):
        js = _read("hud", "static", "app.js")
        self.assertNotRegex(js, r"\.(inner|outer)HTML\b")
        for sink in ("insertAdjacentHTML(", "document.write", "eval(", "new Function",
                     "javascript:", "createContextualFragment"):
            self.assertNotIn(sink, js)
        self.assertNotRegex(js, r"setAttribute\(\s*['\"]on")
        self.assertNotRegex(js, r"set(Timeout|Interval)\(\s*['\"`]")

    def test_hostile_event_exports_no_formula_cell(self):
        state = rb.HudState()
        hostile = "=HYPERLINK(\"http://x\",\"y\")"
        for prefix in self.FORMULA:
            state.add_event({"source": prefix + "src", "kind": prefix + "kind",
                             "message": prefix + hostile, "t": prefix + "1",
                             "wall_t": prefix + "2",
                             "data": {"permit_state": prefix + "AUTHORIZED",
                                      "o2_pct": prefix + "cmd|' /C calc'!A0"}})
        with _hud_client(state) as client:
            resp = client.get("/api/audit.csv")
        self.assertEqual(resp.status_code, 200)
        self.assertTrue(resp.headers["content-disposition"].startswith("attachment;"))
        rows = list(csv.DictReader(io.StringIO(resp.content.decode("utf-8"))))
        self.assertEqual(len(rows), len(self.FORMULA))
        numeric = ("seq", "t", "wall_t", "o2_pct", "lel_pct", "h2s_ppm", "co_ppm")
        for row in rows:
            for column, cell in row.items():
                if column in numeric:
                    if cell != "":
                        float(cell)   # a number, never text a spreadsheet evaluates
                    continue
                self.assertFalse(cell.startswith(self.FORMULA), (column, cell))
            self.assertIn(row["permit_state"], set(rb.PERMIT_STATES) | {""})
            self.assertTrue(row["message"].startswith("'"))

    def test_jsonl_download_is_an_attachment(self):
        with _hud_client() as client:
            resp = client.get("/api/audit.jsonl")
        self.assertEqual(resp.headers["content-type"], "application/x-ndjson")
        self.assertTrue(resp.headers["content-disposition"].startswith("attachment;"))
        self.assertEqual(resp.headers["x-content-type-options"], "nosniff")


# ---------------------------------------------------------------------------
# T10 (partial)
# ---------------------------------------------------------------------------


class TestT10SpeechPostFilter(unittest.TestCase):
    """T10: the denylist post-filter rewrites absolute safety claims and tool names.

    Present since 63c05ef: agent/spot_agent.py sanitize_speech, applied by say()
    before the HUD event and by TranscriptCallback before the console. Paraphrases
    ("no hazard is present"), Unicode look-alikes and wrong numbers still pass it
    (threat model T10); the escape-sequence stripping and the code-generated
    sentences added by the hardening pass are tested in tests/test_hardening_agent.py.
    """

    CLAIMS = ("It is safe.", "The tank is now safe to enter.", "Entry is now safe.",
              "The atmosphere is completely safe for workers.", "All clear, go ahead.",
              "Conditions are safe, so I will proceed.")

    def test_absolute_claims_become_regulatory_wording(self):
        for claim in self.CLAIMS:
            spoken, flags = sanitize_speech(claim)
            self.assertNotRegex(spoken, r"(?i)\bsafe\b|all clear", claim)
            self.assertIn("within permissible entry limits", spoken.lower(), claim)
            self.assertTrue(flags, claim)

    def test_tool_identifiers_are_not_spoken(self):
        spoken, flags = sanitize_speech('Next I call walk_to_named(place="entry") and '
                                        "ventilation_status.")
        for name in spot_agent.SPOKEN_TOOL_LABELS:
            self.assertNotIn(name, spoken)
        self.assertNotIn("place=", spoken)
        self.assertEqual({f["rule"] for f in flags}, {"tool_name"})

    def test_negated_claim_is_kept_and_flagged(self):
        text = "I can't say it is safe."
        spoken, flags = sanitize_speech(text)
        self.assertEqual(spoken, text)   # rewriting it would invert the meaning
        self.assertIn("negated_safety_claim", {f["rule"] for f in flags})

    def test_hud_event_carries_the_filtered_text(self):
        robot = FakeRobot()
        tools = SpotTools(robot, FakeMission(robot), robot.site(), env={},
                          sleep=lambda _s: None, clock=lambda: 0.0)
        robot.events.clear()
        published = tools.say("It is safe. Calling run_atmospheric_test now.")
        self.assertNotRegex(published, r"(?i)\bsafe\b|run_atmospheric_test")
        (event,) = [e for e in robot.events if e["kind"] == "say"]
        self.assertEqual(event["message"], published)
        self.assertEqual(event["source"], "agent")
        audit = event["data"]["speech_filter"]
        self.assertTrue(audit["rewritten"])
        self.assertIn("It is safe.", audit["original"])   # the audit keeps what was said

    def test_console_shows_the_filtered_text(self):
        out, said = io.StringIO(), []
        callback = TranscriptCallback(said.append, out=out)
        for chunk in ("It is ", "safe. I used ", "get_gas_reading."):
            callback(data=chunk)
        callback(message={"role": "assistant",
                          "content": [{"text": "It is safe. I used get_gas_reading."}]})
        console = out.getvalue()
        self.assertNotIn("It is safe", console)
        self.assertNotIn("get_gas_reading", console)
        self.assertIn("within permissible entry limits", console)
        self.assertEqual(len(said), 1)   # say() then filters what the HUD gets


# ---------------------------------------------------------------------------
# T28 (partial)
# ---------------------------------------------------------------------------


class TestT28ContainerBoundaryPieces(unittest.TestCase):
    """T28: the pieces of the container boundary that the repo itself controls.

    Present since 63c05ef: the X grant is the uid-scoped +si:localuser:#1234 form,
    never a blanket `xhost +`; the sim's docker run does not override the image user
    or add privileges; the HUD mounts agent/ read-only; deploy-to-host.sh chowns the
    tree to ubuntu:ubuntu and the shipped files are not world-writable. The hardening
    pass added unprivileged users, dropped capabilities, read-only roots and minimal
    mounts for the other containers and opt-in desktop changes; those are tested in
    tests/test_hardening_host.py.
    """

    SHIPPED = ("sim", "host", "agent", "hud", "tests", "tools")   # deploy-to-host.sh:41

    def _scripts(self):
        for rel in ("host", "tools", "scripts"):
            for name in sorted(os.listdir(os.path.join(REPO, rel))):
                if name.endswith(".sh"):
                    yield name, _read(rel, name)

    def test_x_access_is_granted_to_one_uid_only(self):
        grants = []
        for name, text in self._scripts():
            for line in text.splitlines():
                code = line.strip()
                if code.startswith("#"):   # full-line comments ("#1234" is not one)
                    continue
                # An xhost access change: `xhost +...` / `xhost -...` (quoted or not).
                for match in re.finditer(r"\bxhost\s+[\"']?([+-][^\s\"']*)", code):
                    grants.append((name, match.group(1)))
        self.assertTrue(grants, "run_sim.sh --gui grants X access somewhere")
        self.assertEqual(set(grants), {("run_sim.sh", "+si:localuser:#$ISAAC_UID")})
        self.assertIn("ISAAC_UID=1234", _read("host", "run_sim.sh"))

    def _sim_docker_run(self):
        text = _read("host", "run_sim.sh")
        start = text.index("SIM_DOCKER=(docker run")
        return text[start:text.index("\n\n", start)]

    def test_sim_container_gets_no_extra_privilege(self):
        run = self._sim_docker_run()
        for flag in ("--user", " -u ", "--privileged", "--cap-add", "--pid=host",
                     "--ipc=host", "--userns", "docker.sock", "--security-opt"):
            self.assertNotIn(flag, run)

    def test_hud_mounts_agent_code_read_only(self):
        run_hud = _read("host", "run_hud.sh")
        mounts = re.findall(r'-v\s+"([^"]+)"', run_hud)
        self.assertEqual(mounts, ["$DEMO_DIR/agent:/workspace/agent:ro"])

    def test_deploy_hands_the_tree_to_ubuntu(self):
        deploy = _read("scripts", "deploy-to-host.sh")
        # The install directory itself, and recursively only the shipped tree (T30).
        self.assertIn("sudo chown ubuntu:ubuntu '$REMOTE_DIR'", deploy)
        self.assertIn("sudo chown -R ubuntu:ubuntu ${WIPE[*]} '$REMOTE_DIR/VERSION'", deploy)
        ship = re.search(r"SHIP_DIRS=\(([^)]*)\)", deploy).group(1).split()
        self.assertEqual(tuple(ship), self.SHIPPED)

    def test_shipped_files_are_not_world_writable(self):
        # The deploy tars this working tree and root's tar keeps its modes.
        loose = []
        for rel in self.SHIPPED:
            for root, dirs, files in os.walk(os.path.join(REPO, rel)):
                dirs[:] = [d for d in dirs if d != "__pycache__"]
                for name in dirs + files:
                    path = os.path.join(root, name)
                    if os.lstat(path).st_mode & stat.S_IWOTH:
                        loose.append(os.path.relpath(path, REPO))
        self.assertEqual(loose, [])


# ---------------------------------------------------------------------------
# T43 (open; the disclosure is pinned, not a control)
# ---------------------------------------------------------------------------


class TestT43RealRobotDisclosure(unittest.TestCase):
    """T43: the README keeps its simulation-only and not-certified disclosures.

    A disclosure, not a control. The hardening pass added the non-production and
    not-a-safety-system notice (tests/test_hardening_docs.py TestT43NoticeAndScope)
    and the simulator-only gate in the agent (tests/test_hardening_agent.py).
    """

    def setUp(self):
        self.readme = " ".join(_read("README.md").split())

    def test_simulation_only_is_stated_up_front(self):
        top = self.readme[:self.readme.index("## Contents")]
        self.assertIn("this is a simulation only", top)
        self.assertIn("Nothing has run on a real robot", top)
        self.assertIn("Base Spot is not ATEX or IECEx certified", top)

    def test_scope_of_claims_keeps_the_hazard_caveats(self):
        self.assertIn("**Simulation only.** Nothing has run against the real `spot_ros2` "
                      "driver or on hardware.", self.readme)
        self.assertIn("certified equipment or an inerted, gas-free tank, and a site hazard "
                      "review", self.readme)
        self.assertIn("**Permit fields are recorded, not verified.**", self.readme)

    def test_agent_default_targets_the_simulator(self):
        self.assertEqual(spot_agent.build_arg_parser().parse_args([]).namespace, "/demo_spot")


# ---------------------------------------------------------------------------
# T44 (partial)
# ---------------------------------------------------------------------------


class TestT44NoRemoteViewerEnabled(unittest.TestCase):
    """T44: the shipped launch enables no Isaac livestream and no Foxglove bridge.

    sim/isaac_app.py builds SimulationApp with only "headless" and enables only
    isaacsim.ros2.bridge. Since the hardening pass it also refuses unknown command-line
    arguments before Kit starts (tests/test_hardening_sim.py TestT44NoKitPassThrough),
    and run_sim.sh checks for off-loopback listeners after READY
    (tests/test_hardening_host.py TestT44NothingListensOffLoopback).
    """

    STREAMING = re.compile(r"livestream|webrtc|streamsdk|foxglove|omni\.services\.streaming",
                           re.I)

    def test_isaac_app_enables_only_the_ros2_bridge(self):
        tree = ast.parse(_read("sim", "isaac_app.py"))
        enabled, app_configs = [], []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
            if name == "enable_extension":
                self.assertIsInstance(node.args[0], ast.Constant)
                enabled.append(node.args[0].value)
            if name == "SimulationApp":
                self.assertIsInstance(node.args[0], ast.Dict)
                app_configs.append([k.value for k in node.args[0].keys])
        self.assertEqual(enabled, ["isaacsim.ros2.bridge"])
        self.assertEqual(app_configs, [["headless"]])

    def test_no_streaming_or_bridge_listener_in_launch_code(self):
        for rel in ("sim", "host", "tools", "scripts", "agent", "hud"):
            for root, dirs, files in os.walk(os.path.join(REPO, rel)):
                dirs[:] = [d for d in dirs if d != "__pycache__"]
                for name in files:
                    if name.endswith((".py", ".sh", "Dockerfile", ".txt", ".json")):
                        with open(os.path.join(root, name), encoding="utf-8") as handle:
                            text = handle.read()
                        self.assertIsNone(self.STREAMING.search(text),
                                          os.path.relpath(os.path.join(root, name), REPO))


if __name__ == "__main__":
    unittest.main()
