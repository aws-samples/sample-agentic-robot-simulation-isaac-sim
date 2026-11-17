"""Tests for the host-wrapper, container and dependency hardening (threat model 2.3.3).

Each class docstring names the threat it covers. Most tests run the real host
scripts against a fake host: a temporary directory of stand-ins for docker, sudo,
stat and the other host commands, placed first on PATH. The fake docker records
every invocation's argv and keeps a small container table, so a test can check the
exact `docker run` options a wrapper uses and what it does when another container
is running. No container is started, no AWS call is made and nothing listens on the
network. The rest read the Dockerfiles, lock files and scripts as text.

Run from the repo root:

    python3 -m unittest tests.test_hardening_host -v
"""

import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest

import defusedxml.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASH = shutil.which("bash")


def _bash_first(path):
    """``path`` with BASH's directory in front, so the literal "bash" a test runs is BASH."""
    return os.pathsep.join((os.path.dirname(BASH), path))


def _read(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as handle:
        return handle.read()


def _scripts(*dirs):
    for rel in dirs:
        for name in sorted(os.listdir(os.path.join(REPO, rel))):
            if name.endswith(".sh"):
                yield "%s/%s" % (rel, name), _read(rel, name)


def _docker_runs(text):
    """Every `docker run` command in a script, continuation lines joined, comments dropped."""
    lines = [line for line in text.splitlines() if not line.lstrip().startswith("#")]
    joined = re.sub(r"\\\n", " ", "\n".join(lines))
    return [line[m.start():] for line in joined.splitlines()
            for m in re.finditer(r"\bdocker run\s+[-\"]", line)]


def _env(argv):
    """The -e NAME=VALUE pairs of one recorded docker argv."""
    out = {}
    for i, arg in enumerate(argv):
        if arg == "-e" and i + 1 < len(argv) and "=" in argv[i + 1]:
            name, value = argv[i + 1].split("=", 1)
            out[name] = value
    return out


def _mounts(argv):
    return [argv[i + 1] for i, arg in enumerate(argv) if arg == "-v" and i + 1 < len(argv)]


# Stand-ins for the host commands. Each is small and records what it was asked.
FAKE_DOCKER = r'''#!/usr/bin/env python3
"""Fake docker: records argv to $FAKE_DOCKER_LOG and keeps a container table."""
import json, os, sys, time
args = sys.argv[1:]
log = os.environ["FAKE_DOCKER_LOG"]
state_path = os.environ["FAKE_DOCKER_STATE"]
with open(log, "a") as fh:
    fh.write(json.dumps(args) + "\n")
try:
    with open(state_path) as fh:
        state = json.load(fh)
except (OSError, ValueError):
    state = None
def save():
    # Write a new file and rename it over the table, so a concurrent call (run_sim.sh's
    # foreground READY watcher) reads the old or the new table, never a half-written one
    # that it would re-seed empty, dropping the container that was just run.
    tmp = "%s.%d" % (state_path, os.getpid())
    with open(tmp, "w") as fh:
        json.dump(state, fh)
    os.replace(tmp, state_path)
if state is None:   # first call of a test: seed the running containers once
    state = {name: {"args": []} for name in os.environ.get("FAKE_DOCKER_PS", "").split()}
    save()
cmd = args[0] if args else ""
if cmd == "run":
    name = args[args.index("--name") + 1] if "--name" in args else "anon%d" % len(state)
    if name in state and os.environ.get("FAKE_DOCKER_CONFLICT") == "1":
        sys.exit(125)
    state[name] = {"args": args}
    save()
    if "-d" in args:
        print("f00dfeed" * 8)
    else:
        if "--smoke" in args:
            env = dict(a.split("=", 1) for a in args if a.startswith("SPOT_RUN_ID="))
            print("SMOKE-OK steps=1 run_id=%s" % env.get("SPOT_RUN_ID", ""))
        time.sleep(float(os.environ.get("FAKE_DOCKER_FG_SLEEP", "0")))
    sys.exit(int(os.environ.get("FAKE_DOCKER_RUN_RC", "0")))
if cmd == "ps":
    print("\n".join(sorted(state)))
    sys.exit(0)
if cmd == "inspect":
    name = args[-1]
    if name not in state and not name.startswith("f00d"):
        sys.exit(1)
    fmt = args[args.index("-f") + 1] if "-f" in args else ""
    if ".Config.Env" in fmt:
        run = state.get(name, {}).get("args", [])
        print("\n".join(run[i + 1] for i, a in enumerate(run) if a == "-e"))
    else:
        print("true" if "Running" in fmt else "2026-10-29T00:00:00Z")
    sys.exit(0)
if cmd == "top":
    print("PID")
    for pid in os.environ.get("FAKE_DOCKER_TOP", "").split():
        print(pid)
    sys.exit(0)
if cmd == "logs":
    name = [a for a in args[1:] if not a.startswith("-")][-1]
    run = state.get(name, {}).get("args", [])
    ids = [a.split("=", 1)[1] for a in run if a.startswith("SPOT_RUN_ID=")]
    if ids:
        print("FACADE-READY ns=/demo_spot run_id=%s" % ids[-1])
    sys.exit(0)
if cmd in ("rm", "stop"):
    for name in [a for a in args[1:] if not a.startswith("-") and not a.isdigit()]:
        state.pop(name, None)
    save()
    sys.exit(0)
sys.exit(0)  # image inspect, pull, build, exec, ...
'''

FAKE_STAT = r'''#!/usr/bin/env python3
"""Fake GNU stat: -c FORMAT with %u %g %U %G %s %a, from os.stat (env overrides)."""
import os, pwd, grp, sys
args = sys.argv[1:]
fmt = args[args.index("-c") + 1]
paths = [a for i, a in enumerate(args) if not a.startswith("-") and args[i - 1] != "-c"]
rc = 0
for path in paths:
    try:
        st = os.lstat(path)
    except OSError:
        rc = 1
        continue
    uid = int(os.environ.get("FAKE_STAT_UID", st.st_uid))
    gid = int(os.environ.get("FAKE_STAT_GID", st.st_gid))
    def uname(u):
        try:
            return pwd.getpwuid(u).pw_name
        except KeyError:
            return str(u)
    def gname(g):
        try:
            return grp.getgrgid(g).gr_name
        except KeyError:
            return str(g)
    out = fmt
    for key, val in (("%u", uid), ("%g", gid), ("%U", uname(uid)), ("%G", gname(gid)),
                     ("%s", st.st_size), ("%a", "%o" % (st.st_mode & 0o7777))):
        out = out.replace(key, str(val))
    print(out)
sys.exit(rc)
'''

FAKE_SH = {
    # sudo [-u USER] [-i] [--] CMD...: run CMD as the test user.
    "sudo": 'while [ $# -gt 0 ]; do case "$1" in -u) shift 2 ;; -i|-n|-E|--) shift ;; *) break ;; esac; done\n'
            'exec "$@"\n',
    # timeout [--kill-after=N] DURATION CMD...
    "timeout": 'while [ $# -gt 0 ]; do case "$1" in --*) shift ;; *) shift; break ;; esac; done\nexec "$@"\n',
    "chown": 'echo "chown $*" >> "$FAKE_CALLS"\n',
    "sysctl": 'case "$1" in -n) echo 99999999 ;; *) echo "sysctl $*" >> "$FAKE_CALLS" ;; esac\n',
    "curl": "exit 0\n",
    # ss -Hltnup: the listening sockets the test sets.
    "ss": 'printf "%b" "${FAKE_SS:-}"\n',
    "setsid": 'exec "$@"\n',
    "nohup": 'exec "$@"\n',
    "systemctl": 'echo "systemctl $*" >> "$FAKE_CALLS"\n',
    "pkill": 'echo "pkill $*" >> "$FAKE_CALLS"\n',
    # getent passwd USER -> a passwd line whose home is the test's temp HOME; no dcv
    # account (host/imds_guard.sh lets an installed DCV server through), unless a test
    # adds one: as root, as in CI, the line would give dcv uid 0, which the guard refuses.
    "getent": '[ "$2" = dcv ] && exit 2\necho "$2:x:$(id -u):$(id -g)::$HOME:/bin/bash"\n',
    # iptables / ip6tables: the IMDS guard reads as applied unless FAKE_IMDS_GUARD=absent
    # (TestT26ImdsGuard replaces them with a stateful fake).
    "iptables": '[ "${FAKE_IMDS_GUARD:-applied}" = applied ]\n',
    "ip6tables": '[ "${FAKE_IMDS_GUARD:-applied}" = applied ]\n',
    # xhost: record changes; with no argument, print the access list.
    "xhost": 'if [ $# -gt 0 ]; then echo "xhost $*" >> "$FAKE_CALLS"; else printf "%b\\n" "$FAKE_XHOST_LIST"; fi\n',
}


@unittest.skipUnless(BASH, "needs bash")
class FakeHost(unittest.TestCase):
    """Base class: a temporary demo dir and a PATH of stand-ins for the host commands."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="spot-hardening-")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = os.path.join(self.tmp, "bin")
        self.demo = os.path.join(self.tmp, "demo")
        self.cache = os.path.join(self.tmp, "isaac-cache")
        for d in (self.bin, os.path.join(self.demo, "sim"), self.cache):
            os.makedirs(d)
        self.docker_log = os.path.join(self.tmp, "docker.log")
        self.calls = os.path.join(self.tmp, "calls.log")
        self.add_fake("docker", FAKE_DOCKER)
        self.add_fake("stat", FAKE_STAT)
        for name, body in FAKE_SH.items():
            self.add_fake(name, "#!/bin/sh\n" + body)
        self.env = {
            "PATH": self.bin + os.pathsep + os.environ.get("PATH", ""),
            "HOME": self.tmp,
            "FAKE_DOCKER_LOG": self.docker_log,
            "FAKE_DOCKER_STATE": os.path.join(self.tmp, "docker-state.json"),
            "FAKE_CALLS": self.calls,
            "SPOT_DEMO_DIR": self.demo,
            "ISAAC_CACHE_ROOT": self.cache,
            "SPOT_SKIP_SIM_CHECK": "1",
            "ACCEPT_EULA": "Y",
            "SPOT_GUI_USER": os.environ.get("USER") or "nobody",
            "FAKE_XHOST_LIST": "access control enabled, only authorized clients can connect\\n"
                               "SI:localuser:%s\\nSI:localuser:#1234" % (os.environ.get("USER") or "nobody"),
        }

    def add_fake(self, name, body):
        path = os.path.join(self.bin, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)
        os.chmod(path, 0o755)

    def run_script(self, rel, *args, timeout=60, **env):
        merged = dict(self.env)
        merged.update(env)
        for key in [k for k, v in merged.items() if v is None]:
            del merged[key]
        # The literal "bash" is BASH: no stand-in is named bash, and BASH is the first
        # bash on the rest of PATH.
        self.assertEqual(shutil.which("bash", path=merged["PATH"]), BASH)
        return subprocess.run(["bash", os.path.join(REPO, rel), *args], env=merged,
                              capture_output=True, text=True, timeout=timeout,
                              stdin=subprocess.DEVNULL)

    def sim_lib(self, *call, stdin=""):
        """Run one host/sim_lib.sh function on the fake host; ``call`` is its name and
        arguments. The shell code is a fixed literal and ``call`` reaches it as
        positional parameters, so no test value is ever parsed as shell code. PATH
        keeps the stand-ins first, then BASH's directory."""
        env = dict(self.env, PATH=self.bin + os.pathsep + _bash_first(os.environ.get("PATH", "")))
        return subprocess.run(["bash", "-c", 'source host/sim_lib.sh; "$@"', "bash", *call], cwd=REPO,
                              env=env, input=stdin, capture_output=True, text=True, timeout=30)

    def docker_calls(self, verb=None):
        if not os.path.exists(self.docker_log):
            return []
        with open(self.docker_log, encoding="utf-8") as handle:
            calls = [json.loads(line) for line in handle if line.strip()]
        return [c for c in calls if verb is None or (c and c[0] == verb)]

    def runs(self):
        return self.docker_calls("run")

    def mutating_calls(self):
        """docker calls that start, stop or remove a container (ps and inspect only read)."""
        return [c for c in self.docker_calls() if c and c[0] in ("run", "rm", "stop", "kill", "restart")]

    def host_calls(self):
        if not os.path.exists(self.calls):
            return ""
        with open(self.calls, encoding="utf-8") as handle:
            return handle.read()


# Every wrapper that starts a DDS participant, with arguments that reach `docker run`.
WRAPPERS = (
    ("host/run_sim.sh", ("--headless", "true")),
    ("host/run_agent.sh", ("--once", "status")),
    ("host/run_hud.sh", ()),
    ("host/inject_gas.sh", ("h2s", "40")),
    ("host/reset_spot.sh", ()),
    ("host/set_view.sh", ("chase",)),
)


class TestT25DdsIsolation(FakeHost):
    """T25: every container joins DDS on this host's loopback only, off domain 0.

    host/sim_lib.sh spot_dds_args gives every wrapper the same options:
    ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-42}, FASTDDS_BUILTIN_TRANSPORTS=UDPv4 and a
    read-only Fast DDS profile that binds every RTPS socket to 127.0.0.1, and does not
    set ROS_AUTOMATIC_DISCOVERY_RANGE. Measured on the GPU host (2026-11-01) with
    Isaac Sim 6.1's Fast DDS 2.14.6 and ros:jazzy's: the profile alone keeps every
    socket on loopback, and LOCALHOST on top of it opens two send sockets on 0.0.0.0
    and the host's address, which the sim's listener check (T44) refuses; LOCALHOST
    alone listens on every interface. The profile's initial peers keep discovery on
    this host.
    """

    def _assert_contract(self, argv, domain="42"):
        env = _env(argv)
        self.assertNotIn("ROS_AUTOMATIC_DISCOVERY_RANGE", env, argv)
        self.assertEqual(env.get("ROS_DOMAIN_ID"), domain, argv)
        self.assertEqual(env.get("FASTDDS_BUILTIN_TRANSPORTS"), "UDPv4", argv)
        profile = os.path.join(self.demo, "dds", "fastdds-loopback.xml")
        self.assertIn(profile + ":/etc/spot/fastdds-loopback.xml:ro", _mounts(argv))
        self.assertEqual(env.get("FASTRTPS_DEFAULT_PROFILES_FILE"), "/etc/spot/fastdds-loopback.xml")
        self.assertEqual(env.get("FASTDDS_DEFAULT_PROFILES_FILE"), "/etc/spot/fastdds-loopback.xml")

    def test_every_wrapper_container_gets_the_contract(self):
        for rel, args in WRAPPERS:
            with self.subTest(rel):
                os.remove(self.docker_log) if os.path.exists(self.docker_log) else None
                result = self.run_script(rel, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                (argv,) = self.runs()
                self._assert_contract(argv)

    def test_e2e_and_smoke_containers_get_the_contract(self):
        """T41 too: the test sims and the e2e client run on the test domain, 43, not
        the demo's 42, so an agent started during a test finds no facade."""
        result = self.run_script("host/e2e_ros_test.sh", FAKE_DOCKER_FG_SLEEP="3")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        runs = self.runs()
        self.assertEqual(len(runs), 2, runs)          # the sim, then the e2e client
        for argv in runs:
            self._assert_contract(argv, domain="43")
        self.assertIn("/e2e_client.py", runs[1][-1])
        os.remove(self.docker_log)
        result = self.run_script("host/smoke_test.sh")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        (argv,) = self.runs()
        self._assert_contract(argv, domain="43")

    def test_a_test_domain_equal_to_the_demos_is_refused(self):
        for rel in ("host/e2e_ros_test.sh", "host/smoke_test.sh"):
            with self.subTest(rel):
                result = self.run_script(rel, SPOT_TEST_DOMAIN_ID="42")
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn("must differ from the demo's ROS_DOMAIN_ID", result.stderr)
                self.assertEqual(self.runs(), [])

    def test_domain_follows_the_environment(self):
        result = self.run_script("host/set_view.sh", "auto", ROS_DOMAIN_ID="17")
        self.assertEqual(result.returncode, 0, result.stderr)
        self._assert_contract(self.runs()[0], domain="17")

    def test_bad_domain_fails_closed_before_any_container(self):
        for value in ("abc", "300", "042", "-1", "4 2"):
            with self.subTest(value):
                result = self.run_script("host/inject_gas.sh", "clear", ROS_DOMAIN_ID=value)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("not a ROS 2 domain id", result.stderr)
        self.assertEqual(self.runs(), [])

    def test_profile_binds_every_socket_to_loopback(self):
        result = self.run_script("host/set_view.sh", "auto")
        self.assertEqual(result.returncode, 0, result.stderr)
        path = os.path.join(self.demo, "dds", "fastdds-loopback.xml")
        self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o644)
        ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(path).getroot()
        (transport,) = root.findall("f:transport_descriptors/f:transport_descriptor", ns)
        self.assertEqual(transport.find("f:type", ns).text, "UDPv4")
        self.assertEqual([a.text for a in transport.findall("f:interfaceWhiteList/f:address", ns)],
                         ["127.0.0.1"])
        (participant,) = root.findall("f:participant", ns)
        self.assertEqual(participant.get("is_default_profile"), "true")
        rtps = participant.find("f:rtps", ns)
        self.assertEqual(rtps.find("f:useBuiltinTransports", ns).text, "false")
        addresses = [a.text for a in rtps.iter("{%s}address" % ns["f"])]
        self.assertEqual(set(addresses), {"127.0.0.1"})
        self.assertEqual(len(addresses), 3)   # user data, discovery unicast, initial peer
        self.assertNotIn("Multicast", ET.tostring(rtps, encoding="unicode"))

    def test_profile_is_rewritten_only_when_it_differs(self):
        self.assertEqual(self.run_script("host/set_view.sh", "auto").returncode, 0)
        path = os.path.join(self.demo, "dds", "fastdds-loopback.xml")
        os.utime(path, (1, 1))
        self.assertEqual(self.run_script("host/set_view.sh", "auto").returncode, 0)
        self.assertEqual(os.stat(path).st_mtime, 1)   # same content: untouched
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("<profiles/>\n")             # loosened by hand
        self.assertEqual(self.run_script("host/set_view.sh", "auto").returncode, 0)
        self.assertIn("127.0.0.1", _read(path))

    def test_every_docker_run_in_host_and_tools_uses_the_contract(self):
        found = 0
        for name, text in _scripts("host", "tools"):
            for cmd in _docker_runs(text):
                found += 1
                # SPOT_ARGS (spot_container_args) carries SPOT_DDS_ARGS; the sim uses it directly.
                self.assertRegex(cmd, r'"\$\{SPOT_(DDS_)?ARGS\[@\]\}"', "%s: %s" % (name, cmd))
        self.assertEqual(found, 9)   # sim, agent, HUD, 3 one-shots, e2e, spot-rec, Playwright

    def test_nothing_defaults_to_domain_0(self):
        for rel in ("host", "tools", "agent", "hud", "sim"):
            for root, dirs, files in os.walk(os.path.join(REPO, rel)):
                dirs[:] = [d for d in dirs if d != "__pycache__"]
                for name in files:
                    if name.endswith((".sh", ".py", "Dockerfile")):
                        found = re.search(r"ROS_DOMAIN_ID:-0\b|ROS_DOMAIN_ID=0\b", _read(root, name))
                        self.assertIsNone(found, os.path.join(root, name))

    def test_dockerfile_usage_comments_show_the_contract(self):
        for rel in ("agent/Dockerfile", "hud/Dockerfile"):
            usage = "\n".join(line for line in _read(rel).splitlines() if line.startswith("#"))
            for needle in ("ROS_DOMAIN_ID=42", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
                           "/etc/spot/fastdds-loopback.xml:ro"):
                self.assertIn(needle, usage, rel)
            self.assertNotIn("ROS_AUTOMATIC_DISCOVERY_RANGE", usage, rel)


def _sim_lib_uids():
    text = _read("host", "sim_lib.sh")
    return {name: int(value) for name, value in
            re.findall(r"^(SPOT_[A-Z]+_UID)=(\d+)\b", text, re.M)}


def _run_bash_function(name, script, stdin="", env=None):
    """Run one function from a script, defined alone (like tests/test_sensors.py does).

    The shell code is a fixed literal: the function's source, read from the
    repository script, and its name reach it as positional parameters, so nothing
    is spliced into the code; the literal "bash" is BASH (its directory first on PATH)."""
    src = _read(*script.split("/"))
    func = re.search(r"^%s\(\) \{\n.*?^\}\n" % re.escape(name), src, re.S | re.M).group(0)
    env = dict(os.environ, **(env or {}))
    env["PATH"] = _bash_first(os.environ.get("PATH", ""))
    return subprocess.run(["bash", "-c", 'log() { echo "LOG $*"; }\neval "$1"\n"$2"', "bash", func, name],
                          input=stdin, capture_output=True, text=True, timeout=30, env=env)


class TestT28ContainersRunUnprivileged(FakeHost):
    """T28: every container but the sim runs as its own non-root uid, unprivileged.

    The agent and HUD images set USER 10001 / 10002 and the wrappers pass the uid from
    host/sim_lib.sh again, with --cap-drop=ALL, no-new-privileges and a read-only root
    filesystem (tmpfs /tmp); every mount a container does not write is read-only; the
    agent and spot-rec write logs/ through its group, not as ubuntu; spot-rec no longer
    mounts the code; the sim mounts only sim/, read-only, and under --gui only its
    display's socket, read-only, and refuses an X server open to anyone but ubuntu and
    uid 1234. Not covered: the sim keeps the image defaults for capabilities (see
    run_sim.sh), and GDM autologin (TestDesktopSettingsAreOptIn).
    """

    KIND_UID = {"agent": 10001, "hud": 10002, "oneshot": 10003}

    def _assert_unprivileged(self, argv, kind, read_only=True):
        uid = self.KIND_UID[kind]
        self.assertEqual(argv[argv.index("--user") + 1], "%d:%d" % (uid, uid), argv)
        self.assertIn("--cap-drop=ALL", argv)
        self.assertEqual(argv[argv.index("--security-opt") + 1], "no-new-privileges")
        for flag in ("--privileged", "--cap-add", "--pid=host", "--ipc=host", "--userns"):
            self.assertNotIn(flag, " ".join(argv))
        self.assertNotIn("docker.sock", " ".join(argv))
        if read_only:
            self.assertIn("--read-only", argv)
            self.assertEqual(argv[argv.index("--tmpfs") + 1], "/tmp:rw,nosuid,nodev,noexec,size=64m")

    def test_uids_are_distinct_and_neither_root_nor_ubuntu(self):
        uids = _sim_lib_uids()
        self.assertEqual(uids, {"SPOT_SIM_UID": 1234, "SPOT_AGENT_UID": 10001,
                                "SPOT_HUD_UID": 10002, "SPOT_ONESHOT_UID": 10003})
        self.assertEqual(len(set(uids.values())), 4)
        self.assertFalse({0, 1000} & set(uids.values()))
        self.assertIn("ISAAC_UID=%d" % uids["SPOT_SIM_UID"], _read("host", "run_sim.sh"))

    def test_images_set_the_same_non_root_user_last(self):
        uids = _sim_lib_uids()
        for rel, uid in (("agent/Dockerfile", uids["SPOT_AGENT_UID"]),
                         ("hud/Dockerfile", uids["SPOT_HUD_UID"])):
            lines = [line.strip() for line in _read(rel).splitlines()]
            users = [i for i, line in enumerate(lines) if line.startswith("USER ")]
            self.assertEqual(len(users), 1, rel)
            self.assertEqual(lines[users[0]], "USER %d:%d" % (uid, uid), rel)
            runs = [i for i, line in enumerate(lines) if line.startswith("RUN ")]
            self.assertGreater(users[0], max(runs), rel)   # nothing runs as root after it
            self.assertIn("useradd --uid %d" % uid, _read(rel))

    def test_agent_hud_and_one_shots_drop_privileges(self):
        for rel, args, kind in (("host/run_agent.sh", ("--once", "status"), "agent"),
                                ("host/run_agent.sh", ("--mission",), "agent"),
                                ("host/run_hud.sh", (), "hud"),
                                ("host/inject_gas.sh", ("clear",), "oneshot"),
                                ("host/reset_spot.sh", (), "oneshot"),
                                ("host/set_view.sh", ("door",), "oneshot")):
            with self.subTest(rel=rel, args=args):
                os.remove(self.docker_log) if os.path.exists(self.docker_log) else None
                result = self.run_script(rel, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                (argv,) = self.runs()
                self._assert_unprivileged(argv, kind)

    def test_agent_writes_only_logs_through_its_group(self):
        result = self.run_script("host/run_agent.sh", "--once", "status")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        logs = os.path.join(self.demo, "logs")
        self.assertEqual(argv[argv.index("--group-add") + 1], str(os.stat(logs).st_gid))
        self.assertTrue(os.stat(logs).st_mode & stat.S_IWGRP)
        writable = [m for m in _mounts(argv) if not m.endswith(":ro")]
        self.assertEqual(writable, [logs + ":/workspace/logs"])

    def test_hud_and_e2e_client_mount_nothing_writable(self):
        self.assertEqual(self.run_script("host/run_hud.sh").returncode, 0)
        result = self.run_script("host/e2e_ros_test.sh", FAKE_DOCKER_FG_SLEEP="3")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        hud, _sim, client = self.runs()
        for argv in (hud, client):
            self.assertTrue(all(m.endswith(":ro") for m in _mounts(argv)), _mounts(argv))
        self._assert_unprivileged(client, "oneshot")

    def test_sim_mounts_only_sim_read_only(self):
        result = self.run_script("host/run_sim.sh", "--headless", "true")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        mounts = _mounts(argv)
        self.assertIn(os.path.join(self.demo, "sim") + ":/workspace/sim:ro", mounts)
        self.assertNotIn(self.demo + ":/workspace", mounts)
        writable = [m.split(":")[0] for m in mounts if not m.endswith(":ro")]
        self.assertTrue(writable and all(w.startswith(self.cache + os.sep) for w in writable), writable)

    def test_gui_mounts_one_display_socket_read_only(self):
        result = self.run_script("host/run_sim.sh", "--gui", SPOT_GUI_DISPLAY=":0")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        self.assertEqual(_env(argv)["DISPLAY"], ":0")
        self.assertIn("/tmp/.X11-unix/X0:/tmp/.X11-unix/X0:ro", _mounts(argv))
        self.assertNotIn("/tmp/.X11-unix:/tmp/.X11-unix:rw", _mounts(argv))
        self.assertIn("xhost +si:localuser:#1234", self.host_calls())

    def test_gui_refuses_an_x_server_open_to_others(self):
        user = self.env["SPOT_GUI_USER"]
        for listing in ("access control disabled, clients can connect from any host",
                        "access control enabled, only authorized clients can connect\\nLOCAL:",
                        "access control enabled, only authorized clients can connect\\n"
                        "SI:localuser:%s\\nINET:192.0.2.7" % user):
            with self.subTest(listing):
                result = self.run_script("host/run_sim.sh", "--gui", SPOT_GUI_DISPLAY=":0",
                                         FAKE_XHOST_LIST=listing)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("lets in more than", result.stderr)
        self.assertEqual(self.runs(), [])

    def test_x_access_list_parser(self):
        ok = "access control enabled, only authorized clients can connect\nSI:localuser:ubuntu\n" \
             "SI:localuser:#1234\n"
        env = {"GUI_USER": "ubuntu", "ISAAC_UID": "1234"}
        self.assertEqual(_run_bash_function("x_acl_unexpected", "host/run_sim.sh", ok, env).returncode, 0)
        # The Amazon DCV server adds its own account to the console it streams
        # (seen on the GPU host, 2026-11-01); no container runs as that account.
        with_dcv = ok + "SI:localuser:dcv\n"
        self.assertEqual(_run_bash_function("x_acl_unexpected", "host/run_sim.sh", with_dcv, env).returncode, 0)
        for bad in ("", "access control disabled, clients can connect from any host\n",
                    ok + "SI:localuser:mallory\n", ok + "SI:localuser:#1000\n", ok + "LOCAL:\n",
                    ok + "SI:localuser:dcvx\n", ok + "SI:localuser:#994\n"):
            with self.subTest(bad):
                result = _run_bash_function("x_acl_unexpected", "host/run_sim.sh", bad, env)
                self.assertEqual(result.returncode, 1, bad)
                self.assertTrue(result.stdout.strip())

    def test_spot_rec_never_mounts_the_code(self):
        (run,) = [cmd for cmd in _docker_runs(_read("tools", "rehearse_rep.sh")) if "spot-rec" in cmd]
        self.assertIn('"${SPOT_ARGS[@]}"', run)
        self.assertIn('--group-add "$LOG_GID"', run)
        self.assertEqual(re.findall(r'-v "([^"]+)"', run),
                         ["$D/tools/odom_rec.py:/demo/tools/odom_rec.py:ro", "$L:/demo/logs"])
        self.assertIn("spot_container_args oneshot", _read("tools", "rehearse_rep.sh"))

    def test_playwright_runs_as_the_one_shot_uid(self):
        screens = os.path.join(self.demo, "logs", "screens")
        os.makedirs(screens)
        self.run_script("tools/snap_screens.sh", "t28", SPOT_SCREENS_DIR=screens)
        (argv,) = self.runs()
        self._assert_unprivileged(argv, "oneshot", read_only=False)
        self.assertEqual(argv[argv.index("--group-add") + 1], str(os.stat(screens).st_gid))
        self.assertEqual([m for m in _mounts(argv) if not m.endswith(":ro")], [screens + ":/out"])


DIGEST = re.compile(r"^[a-z0-9][a-z0-9./_:-]*@sha256:[0-9a-f]{64}$")


def _sim_lib_images():
    text = _read("host", "sim_lib.sh")
    out = {}
    for name, value in re.findall(r'^(SPOT_[A-Z]+_IMAGE)="([^"]+)"', text, re.M):
        out[name] = value.split(":-", 1)[1].rstrip("}") if value.startswith("${") else value
    return out


def _lock(rel):
    """{name: (version, [hashes])} from a uv --generate-hashes lock file."""
    pins, current = {}, None
    for line in _read(rel).splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = re.match(r"^([A-Za-z0-9._-]+)==([^ \\]+)", line)
        if match:
            current = match.group(1).lower()
            pins[current] = (match.group(2), [])
            continue
        hashed = re.match(r"^\s+--hash=sha256:([0-9a-f]{64})", line)
        if hashed and current:
            pins[current][1].append(hashed.group(1))
            continue
        raise AssertionError("unexpected lock line in %s: %r" % (rel, line))
    return pins


def _version(text):
    return tuple(int(part) for part in re.findall(r"\d+", text)[:3])


class TestT31T32PinnedImagesAndLockedDependencies(FakeHost):
    """T31, T32: images are pinned by digest and Python packages by version and hash.

    host/sim_lib.sh holds every image reference (ros:jazzy-ros-base, isaac-sim 6.1.0,
    Playwright 1.49.0) as an index digest; both Dockerfiles use the same ROS digest; an
    unpinned override is refused. agent/ and hud/requirements.lock pin every package,
    dependencies included, with hashes; the images install only the lock
    (--require-hashes --no-deps --only-binary :all:) and then remove pip. pillow is
    13.x (T32) and mcp the fixed 1.30 line. Not covered: the trivy result itself (the
    scan output is evidence, threat-model 4.2), and the npm package npx fetches.
    """

    def test_every_image_in_sim_lib_is_pinned(self):
        images = _sim_lib_images()
        self.assertEqual(sorted(images), ["SPOT_ISAAC_IMAGE", "SPOT_PLAYWRIGHT_IMAGE", "SPOT_ROS_IMAGE"])
        for name, ref in images.items():
            self.assertRegex(ref, DIGEST, name)
        self.assertIn("isaac-sim:6.1.0@", images["SPOT_ISAAC_IMAGE"])
        self.assertIn("ros:jazzy-ros-base@", images["SPOT_ROS_IMAGE"])
        self.assertIn("playwright:v1.49.0-noble@", images["SPOT_PLAYWRIGHT_IMAGE"])

    def test_dockerfiles_use_the_same_ros_digest(self):
        ros = _sim_lib_images()["SPOT_ROS_IMAGE"]
        # ECR Public's mirror of the official image (ACAT non-ecr-docker-image); its index
        # digest equals Docker Hub's for the same tag (threat-model evidence image-digests.txt).
        self.assertTrue(ros.startswith("public.ecr.aws/docker/library/ros:jazzy-ros-base@sha256:"), ros)
        for rel in ("agent/Dockerfile", "hud/Dockerfile"):
            froms = [line.split()[1] for line in _read(rel).splitlines() if line.startswith("FROM ")]
            self.assertEqual(froms, [ros], rel)

    def test_no_script_names_an_unpinned_image(self):
        for name, text in _scripts("host", "tools"):
            for cmd in _docker_runs(text):
                for literal in ("ros:jazzy", "isaac-sim:", "playwright:", ":latest"):
                    self.assertNotIn(literal, cmd, "%s: %s" % (name, cmd))

    def test_wrappers_run_the_pinned_images(self):
        images = _sim_lib_images()
        for rel, args, image in (("host/run_sim.sh", (), images["SPOT_ISAAC_IMAGE"]),
                                 ("host/inject_gas.sh", ("clear",), images["SPOT_ROS_IMAGE"]),
                                 ("host/reset_spot.sh", (), images["SPOT_ROS_IMAGE"]),
                                 ("host/set_view.sh", ("auto",), images["SPOT_ROS_IMAGE"])):
            with self.subTest(rel):
                os.remove(self.docker_log) if os.path.exists(self.docker_log) else None
                result = self.run_script(rel, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                (argv,) = self.runs()
                self.assertIn(image, argv)

    def test_unpinned_overrides_are_refused(self):
        for rel, args, env in (("host/run_sim.sh", (), {"ISAAC_IMAGE": "nvcr.io/nvidia/isaac-sim:6.1.0"}),
                               ("host/set_view.sh", ("auto",), {"SPOT_ROS_IMAGE": "ros:jazzy-ros-base"}),
                               ("host/e2e_ros_test.sh", (), {"ROS_IMAGE": "ros:jazzy"}),
                               ("host/inject_gas.sh", ("clear",),
                                {"SPOT_ROS_IMAGE": "ros@sha256:" + "0" * 63})):
            with self.subTest(rel):
                result = self.run_script(rel, *args, **env)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("not pinned by digest", result.stderr)
        self.assertEqual(self.runs(), [])

    def test_locks_pin_every_package_with_hashes(self):
        for pkg in ("agent", "hud"):
            pins = _lock("%s/requirements.lock" % pkg)
            self.assertGreater(len(pins), 10, pkg)
            for name, (version, hashes) in pins.items():
                self.assertTrue(hashes, "%s %s has no hash" % (pkg, name))
            wanted = [re.split(r"[<>=!~ ]", line, 1)[0].lower() for line in
                      _read(pkg, "requirements.txt").splitlines()
                      if line.strip() and not line.startswith("#")]
            for name in wanted:
                self.assertIn(name, pins, "%s: %s missing from the lock" % (pkg, name))

    def test_locked_versions_close_the_known_advisories(self):
        hud, agent = _lock("hud/requirements.lock"), _lock("agent/requirements.lock")
        self.assertEqual(_version(hud["pillow"][0])[0], 13)            # T32: >= 13.0.0, < 14
        self.assertIn("pillow>=13.0.0,<14", _read("hud", "requirements.txt"))
        self.assertGreaterEqual(_version(agent["mcp"][0]), (1, 30, 0))
        self.assertLess(_version(agent["mcp"][0]), (2,))
        strands = _version(agent["strands-agents"][0])
        self.assertTrue((1, 57) <= strands < (2,), strands)
        for pins in (hud, agent):
            self.assertNotIn("numpy", pins)       # apt's numpy stays (ROS packages need it)

    def test_dockerfiles_install_only_the_lock(self):
        for pkg in ("agent", "hud"):
            text = re.sub(r"(?m)^#.*\n", "", _read(pkg, "Dockerfile"))
            self.assertNotIn("requirements.txt", text)
            install = re.search(r"bin/pip install([^\n]*\\\n)*[^\n]*", text).group(0)
            for flag in ("--require-hashes", "--no-deps", "--only-binary :all:",
                         "-r /workspace/%s/requirements.lock" % pkg):
                self.assertIn(flag, install, pkg)
            self.assertIn("-m pip uninstall -y -q pip", text)


class TestT35LicenceAndTelemetryAreTheOperatorsChoice(FakeHost):
    """T35: run_sim.sh no longer accepts the NVIDIA licence or telemetry for the adopter.

    The sim starts only when the operator has set ACCEPT_EULA=Y themselves; telemetry
    consent is off unless PRIVACY_CONSENT=Y (then not passed at all, plus
    OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1). smoke_test.sh and e2e_ros_test.sh check
    the same up front. Not covered: what the Isaac image does with PRIVACY_CONSENT
    unset (NVIDIA documents it as the opt-out), and the LICENSE/NOTICE files (README).
    """

    def test_no_licence_no_sim(self):
        for value in (None, "", "y", "yes", "N"):
            with self.subTest(value):
                result = self.run_script("host/run_sim.sh", ACCEPT_EULA=value)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("does not accept it", result.stderr)
                self.assertIn("ACCEPT_EULA=Y", result.stderr)
        self.assertEqual(self.docker_calls(), [])    # not even the `docker rm -f spot-sim`

    def test_telemetry_is_off_by_default(self):
        result = self.run_script("host/run_sim.sh", PRIVACY_CONSENT=None)
        self.assertEqual(result.returncode, 0, result.stderr)
        env = _env(self.runs()[0])
        self.assertEqual(env["ACCEPT_EULA"], "Y")
        self.assertNotIn("PRIVACY_CONSENT", env)
        self.assertEqual(env["OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA"], "1")
        self.assertIn("telemetry: off", result.stderr)

    def test_telemetry_only_on_explicit_opt_in(self):
        result = self.run_script("host/run_sim.sh", PRIVACY_CONSENT="Y")
        self.assertEqual(result.returncode, 0, result.stderr)
        env = _env(self.runs()[0])
        self.assertEqual(env["PRIVACY_CONSENT"], "Y")
        self.assertNotIn("OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA", env)
        bad = self.run_script("host/run_sim.sh", PRIVACY_CONSENT="sure")
        self.assertEqual(bad.returncode, 2, bad.stderr)

    def test_smoke_and_e2e_check_the_licence_first(self):
        for rel in ("host/smoke_test.sh", "host/e2e_ros_test.sh"):
            with self.subTest(rel):
                result = self.run_script(rel, ACCEPT_EULA=None)
                self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.docker_calls(), [])

    def test_no_script_sets_consent_for_the_operator(self):
        for name, text in _scripts("host", "tools"):
            for cmd in _docker_runs(text):
                self.assertNotIn("PRIVACY_CONSENT=Y", cmd, name)


FAKE_GSETTINGS = r'''#!/usr/bin/env python3
"""Fake gsettings get|set|reset SCHEMA KEY [VALUE] over a JSON file."""
import json, os, sys
db_path = os.environ["FAKE_GSETTINGS_DB"]
db = json.load(open(db_path))
verb, schema, key = sys.argv[1:4]
name = schema + " " + key
with open(os.environ["FAKE_CALLS"], "a") as fh:
    fh.write("gsettings " + " ".join(sys.argv[1:]) + "\n")
if name not in db:
    sys.exit("No such schema or key: " + name)
if verb == "get":
    print(db[name])
elif verb == "set":
    db[name] = sys.argv[4]
json.dump(db, open(db_path, "w"))
'''

ORIGINAL_SETTINGS = {
    "org.gnome.desktop.screensaver lock-enabled": "true",
    "org.gnome.desktop.session idle-delay": "uint32 300",
    "org.gnome.settings-daemon.plugins.power idle-dim": "true",
    "com.ubuntu.update-notifier show-apport-crashes": "true",
    "org.gnome.desktop.notifications show-banners": "true",
    "org.gnome.shell.extensions.dash-to-dock dock-fixed": "true",
    "org.gnome.shell.extensions.dash-to-dock autohide": "false",
    "org.gnome.shell.extensions.dash-to-dock intellihide": "false",
}
ORIGINAL_GDM = "# GDM configuration storage\n[daemon]\n# AutomaticLoginEnable = true\n" \
               "WaylandEnable=false\n\n[security]\n"


class TestDesktopSettingsAreOptIn(FakeHost):
    """T28 (desktop part): run_sim.sh --gui no longer changes host security settings.

    GDM autologin, screen lock, idle blank and dim, banners and crash pop-ups are changed
    only by the explicit --prepare-desktop step, which prints every change, records the
    originals first (once; the first record wins) and changes nothing else.
    --restore-desktop puts the recorded originals back and removes the record. --gui on
    a desktop that was never prepared changes no setting and only warns. Not covered:
    the real GDM restart and gsettings on a GNOME host.
    """

    def setUp(self):
        super().setUp()
        self.add_fake("gsettings", FAKE_GSETTINGS)
        self.add_fake("dbus-run-session",
                      '#!/bin/sh\nwhile [ "$1" != -- ]; do shift; done; shift; exec "$@"\n')
        self.db = os.path.join(self.tmp, "gsettings.json")
        with open(self.db, "w", encoding="utf-8") as handle:
            json.dump(ORIGINAL_SETTINGS, handle)
        self.gdm = os.path.join(self.tmp, "custom.conf")
        with open(self.gdm, "w", encoding="utf-8") as handle:
            handle.write(ORIGINAL_GDM)
        self.state = os.path.join(self.tmp, "desktop-state")
        self.env.update(FAKE_GSETTINGS_DB=self.db, SPOT_GDM_CONF=self.gdm,
                        SPOT_DESKTOP_STATE_DIR=self.state, SPOT_GUI_DISPLAY=":0")
        self.marker = os.path.join(self.tmp, ".config", "gnome-initial-setup-done")

    def settings(self):
        with open(self.db, encoding="utf-8") as handle:
            return json.load(handle)

    def prepare(self):
        result = self.run_script("host/run_sim.sh", "--prepare-desktop")
        self.assertEqual(result.returncode, 0, result.stderr)
        return result

    def test_gui_on_an_unprepared_desktop_changes_nothing(self):
        result = self.run_script("host/run_sim.sh", "--gui")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("was not prepared", result.stderr)
        self.assertEqual(self.settings(), ORIGINAL_SETTINGS)
        self.assertEqual(_read(self.gdm), ORIGINAL_GDM)
        self.assertNotIn("gsettings set", self.host_calls())
        self.assertNotIn("systemctl", self.host_calls())
        self.assertFalse(os.path.exists(self.marker))
        self.assertEqual(len(self.runs()), 1)

    def test_gui_without_a_desktop_points_to_prepare(self):
        import glob
        if glob.glob("/tmp/.X11-unix/X*"):
            self.skipTest("this machine has an X server socket")
        result = self.run_script("host/run_sim.sh", "--gui", SPOT_GUI_DISPLAY=None)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("--prepare-desktop", result.stderr)
        self.assertEqual(_read(self.gdm), ORIGINAL_GDM)
        self.assertEqual(self.mutating_calls(), [])   # the running sim is left alone

    def test_prepare_prints_records_then_changes(self):
        result = self.prepare()
        for needle in ("AutomaticLogin=%s" % self.env["SPOT_GUI_USER"], "lock-enabled -> false",
                       "idle-delay -> 0", "show-banners -> false", "show-apport-crashes -> false",
                       "--restore-desktop"):
            self.assertIn(needle, result.stderr)
        self.assertEqual(_read(self.state, "gdm-custom.conf"), ORIGINAL_GDM)
        recorded = {}
        for line in _read(self.state, "gsettings").splitlines():
            schema, key, value = line.split(None, 2)
            recorded[schema + " " + key] = value
        self.assertEqual(recorded, ORIGINAL_SETTINGS)
        gdm = _read(self.gdm)
        self.assertIn("[daemon]\nAutomaticLoginEnable=true\nAutomaticLogin=%s\n" % self.env["SPOT_GUI_USER"], gdm)
        self.assertNotIn("# AutomaticLoginEnable", gdm)
        self.assertIn("[security]", gdm)
        now = self.settings()
        self.assertEqual(now["org.gnome.desktop.screensaver lock-enabled"], "false")
        self.assertEqual(now["org.gnome.desktop.session idle-delay"], "0")
        self.assertEqual(now["org.gnome.shell.extensions.dash-to-dock dock-fixed"], "true")  # per launch only
        self.assertTrue(os.path.exists(self.marker))
        self.assertNotIn("systemctl restart gdm3", self.host_calls())   # the desktop was up
        self.assertEqual(self.docker_calls(), [])

    def test_a_second_prepare_keeps_the_first_originals(self):
        self.prepare()
        self.prepare()
        self.assertIn("org.gnome.desktop.screensaver lock-enabled true", _read(self.state, "gsettings"))
        self.assertEqual(_read(self.state, "gdm-custom.conf"), ORIGINAL_GDM)

    def test_restore_puts_the_recorded_originals_back(self):
        self.prepare()
        result = self.run_script("host/run_sim.sh", "--restore-desktop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.settings(), ORIGINAL_SETTINGS)
        self.assertEqual(_read(self.gdm), ORIGINAL_GDM)
        self.assertFalse(os.path.exists(self.marker))
        self.assertFalse(os.path.exists(self.state))
        self.assertIn("next reboot", result.stderr)

    def test_restore_without_a_record_changes_nothing(self):
        result = self.run_script("host/run_sim.sh", "--restore-desktop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("nothing to restore", result.stderr)
        self.assertEqual(_read(self.gdm), ORIGINAL_GDM)

    def test_dock_follows_the_ui_mode_only_once_prepared(self):
        self.prepare()
        self.assertEqual(self.run_script("host/run_sim.sh", "--gui").returncode, 0)
        self.assertEqual(self.settings()["org.gnome.shell.extensions.dash-to-dock autohide"], "true")
        self.assertEqual(self.run_script("host/run_sim.sh", "--editor").returncode, 0)
        self.assertEqual(self.settings()["org.gnome.shell.extensions.dash-to-dock autohide"], "false")

    def test_actions_take_no_other_arguments(self):
        for args in (("--prepare-desktop", "--gui"), ("--restore-desktop", "--headless", "true"),
                     ("--status", "--background")):
            with self.subTest(args):
                result = self.run_script("host/run_sim.sh", *args)
                self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(_read(self.gdm), ORIGINAL_GDM)
        self.assertEqual(self.docker_calls(), [])


class TestT41SimIsNotStoppedMidMission(FakeHost):
    """T41: wrappers that stop or replace the sim refuse while a mission depends on it.

    run_sim.sh (a launch replaces any running sim, and the new --stop), smoke_test.sh,
    e2e_ros_test.sh and tools/rehearse_rep.sh refuse while a spot-agent or
    spot-mission container runs, unless --force is given. The smoke and e2e sims run on
    their own DDS domain and remove only their own sim at exit. run_sim.sh accepts only its
    own options and isaac_app.py's (kept equal to sim/launch_args.py here), and refuses
    anything else before it touches the running sim, so a typo of --status no longer
    reaches `docker rm -f spot-sim`. Not covered: the mission's wall-clock budgets
    under a real-time-factor collapse (agent package).
    """

    def test_launch_refuses_while_an_agent_or_mission_runs(self):
        for name in ("spot-agent", "spot-mission"):
            with self.subTest(name):
                state = self.env["FAKE_DOCKER_STATE"]
                os.remove(state) if os.path.exists(state) else None
                result = self.run_script("host/run_sim.sh", "--background", FAKE_DOCKER_PS=name)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("refusing to replace the running sim", result.stderr)
                self.assertIn(name, result.stderr)
                self.assertEqual(self.mutating_calls(), [])

    def _rm_own_sim(self, sim_args):
        with open(self.env["FAKE_DOCKER_STATE"], "w", encoding="utf-8") as handle:
            json.dump({"spot-sim": {"args": sim_args}}, handle)
        return self.sim_lib("spot_rm_own_sim", "run-1")

    def test_a_test_removes_only_its_own_sim(self):
        """smoke_test.sh and e2e_ros_test.sh remove spot-sim at exit only if it is
        their own launch (its SPOT_RUN_ID), never a sim started since."""
        result = self._rm_own_sim(["run", "--name", "spot-sim", "-e", "SPOT_RUN_ID=run-2"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.mutating_calls(), [])
        result = self._rm_own_sim(["run", "--name", "spot-sim", "-e", "SPOT_RUN_ID=run-1"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.mutating_calls(), [["rm", "-f", "spot-sim"]])
        for rel in ("host/e2e_ros_test.sh", "host/smoke_test.sh"):
            with self.subTest(rel):
                self.assertNotIn("docker rm -f spot-sim", _read(*rel.split("/")))
                self.assertIn("spot_rm_own_sim", _read(*rel.split("/")))

    def test_force_replaces_it_anyway_and_says_so(self):
        result = self.run_script("host/run_sim.sh", "--force", FAKE_DOCKER_PS="spot-agent")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--force", result.stderr)
        self.assertEqual([c[0] for c in self.mutating_calls()], ["rm", "run"])
        self.assertNotIn("--force", self.runs()[0])   # consumed, not passed to isaac_app.py

    def test_stop_refuses_while_a_mission_runs(self):
        result = self.run_script("host/run_sim.sh", "--stop", FAKE_DOCKER_PS="spot-sim spot-mission")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.mutating_calls(), [])
        result = self.run_script("host/run_sim.sh", "--stop", "--force")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([c[:1] for c in self.mutating_calls()], [["stop"], ["rm"]])

    def test_stop_without_an_agent(self):
        result = self.run_script("host/run_sim.sh", "--stop", FAKE_DOCKER_PS="spot-sim spot-hud")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["stop", "-t", "10", "spot-sim"], self.docker_calls())
        os.remove(self.docker_log)
        result = self.run_script("host/run_sim.sh", "--stop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("no spot-sim container", result.stdout)

    def test_unknown_or_kit_arguments_are_refused_before_the_sim_is_touched(self):
        for args in (("--stauts",), ("--status", "--force"), ("--headles", "true"),
                     ("--/app/window/width=640",), ("--enable", "omni.kit.anything"),
                     ("--ext-folder", "/tmp"), ("--cameras",), ("--cameras", "--smoke"),
                     ("--scene=",), ("stray",), ("--prepare-desktop", "--force")):
            with self.subTest(args):
                result = self.run_script("host/run_sim.sh", *args, FAKE_DOCKER_PS="spot-sim")
                self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.mutating_calls(), [])

    def test_isaac_app_options_pass_through(self):
        result = self.run_script("host/run_sim.sh", "--headless", "true", "--smoke", "--no-cameras",
                                 "--cameras=all", "--camera-hz", "5")
        self.assertEqual(result.returncode, 0, result.stderr)
        argv = self.runs()[0]
        tail = argv[argv.index("/workspace/sim/isaac_app.py") + 1:]
        self.assertEqual(tail, ["--headless", "true", "--smoke", "--no-cameras", "--cameras=all",
                                "--camera-hz", "5"])

    def test_the_fake_docker_table_is_never_seen_half_written(self):
        # The foreground READY watcher reads the table while `docker run` writes it; a
        # half-written read re-seeded it empty and the watcher polled past the test timeout.
        state = self.env["FAKE_DOCKER_STATE"]
        env = dict(self.env)   # its PATH finds the fake docker first
        for _ in range(20):
            with open(state, "w", encoding="utf-8") as handle:
                json.dump({"spot-hud": {"args": []}}, handle)
            run = subprocess.Popen(["docker", "run", "--name", "spot-sim", "img"], env=env,
                                   stdout=subprocess.DEVNULL)
            while run.poll() is None:
                with open(state, encoding="utf-8") as handle:
                    self.assertIn("spot-hud", json.load(handle))
            self.assertEqual(run.returncode, 0)
            with open(state, encoding="utf-8") as handle:
                self.assertEqual(sorted(json.load(handle)), ["spot-hud", "spot-sim"])
        self.assertEqual([n for n in os.listdir(os.path.dirname(state))
                          if n.startswith("docker-state.json.")], [])

    def test_allowlist_matches_isaac_app_argparse(self):
        import ast
        flags, values = set(), set()
        for node in ast.walk(ast.parse(_read("sim", "launch_args.py"))):
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "add_argument":
                names = [a.value for a in node.args if isinstance(a, ast.Constant)]
                action = [k.value.value for k in node.keywords if k.arg == "action"]
                (flags if action == ["store_true"] else values).update(names)
        text = _read("host", "run_sim.sh")
        self.assertEqual(set(re.search(r'^SIM_APP_FLAGS="([^"]*)"', text, re.M).group(1).split()), flags)
        self.assertEqual(set(re.search(r'^SIM_APP_VALUE_OPTS="([^"]*)"', text, re.M).group(1).split()),
                         values)

    def test_smoke_and_e2e_refuse_while_an_agent_runs(self):
        for rel in ("host/smoke_test.sh", "host/e2e_ros_test.sh"):
            with self.subTest(rel):
                state = self.env["FAKE_DOCKER_STATE"]
                os.remove(state) if os.path.exists(state) else None
                result = self.run_script(rel, FAKE_DOCKER_PS="spot-sim spot-agent")
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("refusing to replace the running sim", result.stderr)
                self.assertEqual(self.mutating_calls(), [])

    def test_smoke_with_force_passes_it_to_run_sim(self):
        result = self.run_script("host/smoke_test.sh", "--force", FAKE_DOCKER_PS="spot-agent")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(self.runs()), 1)

    def test_rehearsal_checks_before_it_removes_the_agent(self):
        """T41 and T16: the rehearsal checks for a running agent first, and with
        --force stops it with docker stop (the agent brings Spot out), never rm -f."""
        text = _read("tools", "rehearse_rep.sh")
        guard = text.index('spot_guard_sim_stop "$FORCE"')
        self.assertLess(guard, text.index("docker stop -t 900 spot-agent"))
        self.assertNotIn("docker rm -f spot-agent", text)

    def test_no_script_documents_docker_rm_as_the_stop(self):
        for name, text in _scripts("host"):
            for line in text.splitlines():
                if "docker rm -f spot-sim" in line and line.lstrip().startswith(("#", "log", "echo")):
                    self.assertIn("used to", line, "%s: %s" % (name, line))


SS_LOOPBACK_ONLY = (
    'tcp LISTEN 0 4096 127.0.0.53%lo:53 0.0.0.0:* users:(("systemd-resolve",pid=541,fd=15))\\n'
    'tcp LISTEN 0 10 127.0.0.1:8443 0.0.0.0:* users:(("dcvserver",pid=1200,fd=9))\\n'
    'tcp LISTEN 0 10 [::1]:8443 [::]:* users:(("dcvserver",pid=1200,fd=10))\\n'
    'tcp LISTEN 0 2048 127.0.0.1:8080 0.0.0.0:* users:(("python3",pid=3001,fd=7))\\n'
    'udp UNCONN 0 0 127.0.0.1:17910 0.0.0.0:* users:(("kit",pid=4242,fd=80))\\n'
    'udp UNCONN 0 0 10.0.1.5%ens5:68 0.0.0.0:* users:(("systemd-network",pid=600,fd=22))\\n')


class TestT44NothingListensOffLoopback(FakeHost):
    """T44: no remote viewer is enabled, and the sim launch checks that none appeared.

    run_sim.sh has no remote-viewer option and refuses Kit settings and extension flags
    (TestT41SimIsNotStoppedMidMission). After READY, in both launch modes, it lists listening
    sockets (ss -ltnup) and fails when anything listens off loopback on the DCV, HUD,
    Kit, remote-viewer or this domain's DDS ports, or when any process other than the
    SSM agent, DCV or the DHCP client listens off loopback; a listener of the sim stops
    the sim. Not covered: whether Kit opens a port on the real image (the GPU host).
    """

    def _launch(self, ss, **env):
        return self.run_script("host/run_sim.sh", "--background", FAKE_SS=ss, **env)

    def test_parser_drops_only_loopback(self):
        sample = ("tcp LISTEN 0 1 127.0.0.53%lo:53 0.0.0.0:*\n"
                  "tcp LISTEN 0 1 [::1]:8443 [::]:*\n"
                  "udp UNCONN 0 0 [::ffff:127.0.0.1]:17910 *:*\n"
                  'tcp LISTEN 0 1 *:8443 *:* users:(("dcvserver",pid=12,fd=9))\n'
                  'tcp LISTEN 0 1 0.0.0.0:22 0.0.0.0:* users:(("sshd",pid=900,fd=3))\n'
                  'tcp LISTEN 0 1 [fe80::1%ens5]:8211 [::]:* users:(("kit",pid=42,fd=9))\n')
        out = self.sim_lib("spot_nonloopback_listeners", "-", stdin=sample)
        self.assertEqual(out.stdout.splitlines(), ["tcp *:8443 dcvserver 12", "tcp 0.0.0.0:22 sshd 900",
                                                   "tcp [fe80::1%ens5]:8211 kit 42"])

    def test_loopback_only_host_is_ready(self):
        result = self._launch(SS_LOOPBACK_ONLY)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("run_sim: READY", result.stdout)
        self.assertIn("listeners: none off loopback", result.stderr)

    def test_dcv_on_every_interface_fails_but_keeps_the_sim(self):
        ss = SS_LOOPBACK_ONLY + 'tcp LISTEN 0 10 *:8443 *:* users:(("dcvserver",pid=1200,fd=11))\\n'
        result = self._launch(ss)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("*:8443 (dcvserver", result.stderr)
        self.assertEqual([c[0] for c in self.mutating_calls()], ["rm", "run"])   # not removed after

    def test_a_sim_listener_stops_the_sim(self):
        ss = SS_LOOPBACK_ONLY + 'tcp LISTEN 0 5 0.0.0.0:49100 0.0.0.0:* users:(("kit",pid=4242,fd=91))\\n'
        result = self._launch(ss, FAKE_DOCKER_TOP="4242 4243")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("stopping the sim", result.stderr)
        self.assertEqual(self.mutating_calls()[-1], ["rm", "-f", "spot-sim"])

    def test_a_foreground_launch_is_checked_too(self):
        """The foreground mode (the smoke and end-to-end tests use it) runs the same
        check from a watcher once this run is READY, and a sim listener stops the sim."""
        ss = SS_LOOPBACK_ONLY + 'tcp LISTEN 0 5 0.0.0.0:49100 0.0.0.0:* users:(("kit",pid=4242,fd=91))\\n'
        result = self.run_script("host/run_sim.sh", FAKE_SS=ss, FAKE_DOCKER_TOP="4242",
                                 FAKE_DOCKER_FG_SLEEP="4")
        self.assertIn("0.0.0.0:49100 (kit pid 4242)", result.stderr)
        self.assertIn("stopping the sim", result.stderr)
        self.assertEqual(self.mutating_calls()[-1], ["rm", "-f", "spot-sim"])
        os.remove(self.docker_log)
        result = self.run_script("host/run_sim.sh", FAKE_SS=SS_LOOPBACK_ONLY, FAKE_DOCKER_FG_SLEEP="4")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("listeners: none off loopback", result.stderr)

    def test_dds_off_loopback_fails(self):
        ss = SS_LOOPBACK_ONLY + 'udp UNCONN 0 0 0.0.0.0:17911 0.0.0.0:* users:(("python3",pid=5000,fd=4))\\n'
        result = self._launch(ss)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("DDS port", result.stderr)

    def test_other_listeners_fail_unless_accepted(self):
        ss = SS_LOOPBACK_ONLY + 'tcp LISTEN 0 128 0.0.0.0:22 0.0.0.0:* users:(("sshd",pid=900,fd=3))\\n'
        result = self._launch(ss)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('SPOT_LISTEN_ALLOW="sshd"', result.stderr)
        self.assertEqual(self._launch(ss, SPOT_LISTEN_ALLOW="sshd").returncode, 0)

    def test_run_sim_offers_no_remote_viewer_option(self):
        text = _read("host", "run_sim.sh")
        options = set(re.findall(r"^\s+(--[a-z-]+)(?:\|--[a-z-]+)*\)", text, re.M))
        self.assertEqual(options, {"--gui", "--editor", "--background", "--force", "--status"})


class TestT26OnlyTheAgentAsksIMDS(FakeHost):
    """T26 (container part): every container but the agent sets AWS_EC2_METADATA_DISABLED.

    The sim, HUD, presenter one-shots, the e2e client, spot-rec and the Playwright
    helper get AWS_EC2_METADATA_DISABLED=true, so SDK clients in them never fetch the
    role's credentials; the agent, which calls Bedrock, does not. This stops SDK
    clients only; host/imds_guard.sh (TestT26ImdsGuard) and the role policy are the
    real control.
    """

    def test_every_container_but_the_agent(self):
        cases = (("host/run_sim.sh", (), True), ("host/run_hud.sh", (), True),
                 ("host/inject_gas.sh", ("clear",), True), ("host/reset_spot.sh", (), True),
                 ("host/set_view.sh", ("auto",), True),
                 ("host/run_agent.sh", ("--once", "x"), False),
                 ("host/run_agent.sh", ("--mission",), False))
        for rel, args, disabled in cases:
            with self.subTest(rel=rel, args=args):
                os.remove(self.docker_log) if os.path.exists(self.docker_log) else None
                self.assertEqual(self.run_script(rel, *args).returncode, 0)
                env = _env(self.runs()[0])
                if disabled:
                    self.assertEqual(env.get("AWS_EC2_METADATA_DISABLED"), "true")
                else:
                    self.assertNotIn("AWS_EC2_METADATA_DISABLED", env)
                    self.assertIn("AWS_REGION", env)

    def test_e2e_client_spot_rec_and_playwright(self):
        result = self.run_script("host/e2e_ros_test.sh", FAKE_DOCKER_FG_SLEEP="3")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for argv in self.runs():
            self.assertEqual(_env(argv).get("AWS_EC2_METADATA_DISABLED"), "true")
        screens = os.path.join(self.demo, "logs", "screens")
        os.makedirs(screens)
        os.remove(self.docker_log)
        self.run_script("tools/snap_screens.sh", "t26", SPOT_SCREENS_DIR=screens)
        self.assertEqual(_env(self.runs()[0]).get("AWS_EC2_METADATA_DISABLED"), "true")
        rec = _read("tools", "rehearse_rep.sh")
        self.assertIn("spot_container_args oneshot", rec)   # spot-rec: the one-shot options

    def test_the_comment_says_it_is_not_the_real_control(self):
        for rel in (("host", "sim_lib.sh"), ("host", "run_sim.sh")):
            text = _read(*rel)
            self.assertIn("AWS_EC2_METADATA_DISABLED", text)
            self.assertRegex(text, r"(?s)stops\s+(#\s+)?SDK clients only")
            self.assertIn("real control", text)


SAMPLE_DCV_CONF = """[license]
# a comment
[connectivity]
web-port=8443
web-listen-endpoints=['0.0.0.0:8443']
enable-quic-frontend=true
idle-timeout=0

[session-management]
create-session = true

[session-management/automatic-console-session]
owner="ubuntu"
#max-concurrent-clients=4

[display]
target-fps=30
"""


class TestT1DcvOnLoopbackOnly(FakeHost):
    """T1 (M1): host/configure_dcv.sh keeps DCV on loopback with one viewer.

    apply records /etc/dcv/dcv.conf once and sets, per the DCV admin guide's parameter
    reference: web-listen-endpoints and quic-listen-endpoints on 127.0.0.1 and ::1,
    enable-quic-frontend=false, create-session=true, the console owner and
    max-concurrent-clients=1, keeping every other line; restore puts the original back
    byte for byte. check fails when anything listens on TCP or UDP 8443 off loopback,
    and run_sim.sh --gui refuses to start then. Not covered: DCV itself reading the
    file (needs the DCV host); the per-event password rotation (RUNBOOK).
    """

    def setUp(self):
        super().setUp()
        self.conf = os.path.join(self.tmp, "dcv.conf")
        with open(self.conf, "w", encoding="utf-8") as handle:
            handle.write(SAMPLE_DCV_CONF)
        self.env["SPOT_DCV_CONF"] = self.conf

    def dcv(self, *args, **env):
        return self.run_script("host/configure_dcv.sh", *args, **env)

    def sections(self, text):
        out, section = {}, None
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("["):
                section = line[1:-1]
            elif line and not line.startswith("#") and "=" in line:
                key, value = (part.strip() for part in line.split("=", 1))
                self.assertNotIn((section, key), out, "duplicate key %s %s" % (section, key))
                out[(section, key)] = value
        return out

    def test_print_sets_the_keys_and_keeps_the_rest(self):
        result = self.dcv("print")
        self.assertEqual(result.returncode, 0, result.stderr)
        keys = self.sections(result.stdout)
        loop = "['127.0.0.1:8443', '[::1]:8443']"
        self.assertEqual(keys[("connectivity", "web-listen-endpoints")], loop)
        self.assertEqual(keys[("connectivity", "quic-listen-endpoints")], loop)
        self.assertEqual(keys[("connectivity", "enable-quic-frontend")], "false")
        self.assertEqual(keys[("session-management", "create-session")], "true")
        console = "session-management/automatic-console-session"
        self.assertEqual(keys[(console, "max-concurrent-clients")], "1")
        self.assertEqual(keys[(console, "owner")], '"%s"' % self.env["SPOT_GUI_USER"])
        self.assertEqual(keys[("connectivity", "idle-timeout")], "0")
        self.assertEqual(keys[("display", "target-fps")], "30")
        self.assertIn("# a comment", result.stdout)
        self.assertIn("#max-concurrent-clients=4", result.stdout)
        self.assertEqual(_read(self.conf), SAMPLE_DCV_CONF)          # print writes nothing

    def test_apply_records_once_and_restore_puts_it_back(self):
        self.assertEqual(self.dcv("apply").returncode, 0)
        applied = _read(self.conf)
        self.assertEqual(_read(self.conf + ".spot-demo.orig"), SAMPLE_DCV_CONF)
        self.assertIn("enable-quic-frontend=false", applied)
        second = self.dcv("apply")
        self.assertIn("unchanged", second.stderr)
        self.assertEqual(_read(self.conf), applied)
        self.assertEqual(_read(self.conf + ".spot-demo.orig"), SAMPLE_DCV_CONF)   # first record wins
        self.assertNotIn("systemctl", self.host_calls())
        result = self.dcv("restore", "--restart")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(_read(self.conf), SAMPLE_DCV_CONF)
        self.assertFalse(os.path.exists(self.conf + ".spot-demo.orig"))
        self.assertIn("systemctl restart dcvserver", self.host_calls())
        self.assertIn("nothing to restore", self.dcv("restore").stderr)

    def test_apply_without_a_file_and_restore_removes_it(self):
        os.remove(self.conf)
        self.assertEqual(self.dcv("apply").returncode, 0)
        self.assertIn("max-concurrent-clients=1", _read(self.conf))
        self.assertTrue(os.path.exists(self.conf + ".spot-demo.absent"))
        self.assertEqual(self.dcv("restore").returncode, 0)
        self.assertFalse(os.path.exists(self.conf))

    def test_check_fails_on_any_off_loopback_8443(self):
        ok = ('tcp LISTEN 0 10 127.0.0.1:8443 0.0.0.0:* users:(("dcvserver",pid=1,fd=9))\\n'
              'tcp LISTEN 0 10 [::1]:8443 [::]:* users:(("dcvserver",pid=1,fd=10))\\n'
              'tcp LISTEN 0 10 0.0.0.0:18443 0.0.0.0:* users:(("other",pid=2,fd=3))\\n')
        self.assertEqual(self.dcv("check", FAKE_SS=ok).returncode, 0)
        for bad in ('tcp LISTEN 0 10 *:8443 *:* users:(("dcvserver",pid=1,fd=9))\\n',
                    'tcp LISTEN 0 10 [::]:8443 [::]:* users:(("dcvserver",pid=1,fd=9))\\n',
                    'udp UNCONN 0 0 0.0.0.0:8443 0.0.0.0:* users:(("dcvserver",pid=1,fd=11))\\n',
                    'tcp LISTEN 0 10 10.0.1.5:8443 0.0.0.0:* users:(("nc",pid=7,fd=3))\\n'):
            with self.subTest(bad):
                result = self.dcv("check", FAKE_SS=ok + bad)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("off loopback", result.stderr)
        self.add_fake("ss", "#!/bin/sh\nexit 1\n")
        self.assertEqual(self.dcv("check").returncode, 2)            # cannot list: fail closed

    def test_bad_arguments(self):
        for args in ((), ("apply", "--now"), ("check", "--restart"), ("enable",), ("apply", "--restart", "x")):
            with self.subTest(args):
                self.assertEqual(self.dcv(*args).returncode, 2)
        self.assertEqual(_read(self.conf), SAMPLE_DCV_CONF)

    def test_gui_refuses_while_dcv_is_reachable(self):
        result = self.run_script("host/run_sim.sh", "--gui", SPOT_GUI_DISPLAY=":0",
                                 FAKE_SS='tcp LISTEN 0 10 *:8443 *:* users:(("dcvserver",pid=1,fd=9))\\n')
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("refusing --gui", result.stderr)
        self.assertEqual(self.mutating_calls(), [])


FAKE_IPTABLES = r'''#!/usr/bin/env python3
"""Fake iptables/ip6tables: -N -F -X -A -I -C -D -S -L over a JSON rule table."""
import json, os, sys
tool = os.path.basename(sys.argv[0])
path = os.path.join(os.environ["FAKE_XT_DIR"], tool + ".json")
try:
    table = json.load(open(path))
except (OSError, ValueError):
    table = {"OUTPUT": [], "INPUT": [], "FORWARD": []}
with open(os.environ["FAKE_CALLS"], "a") as fh:
    fh.write(tool + " " + " ".join(sys.argv[1:]) + "\n")
args = [a for a in sys.argv[1:] if a != "-w"]
op, chain, spec = args[0], (args[1] if len(args) > 1 else None), args[2:]
def save():
    json.dump(table, open(path, "w"))
if op == "-N":
    if chain in table: sys.exit("chain exists")
    table[chain] = []
elif op == "-F":
    if chain not in table: sys.exit(1)
    table[chain] = []
elif op == "-X":
    refs = [r for rules in table.values() for r in rules if r[-1] == chain]
    if chain not in table or table[chain] or refs: sys.exit("cannot delete")
    del table[chain]
elif op == "-A":
    table[chain].append(spec)
elif op == "-I":
    pos = int(spec[0]) - 1 if spec and spec[0].isdigit() else 0
    table[chain].insert(pos, spec[1:] if spec and spec[0].isdigit() else spec)
elif op == "-C":
    sys.exit(0 if chain in table and spec in table[chain] else 1)
elif op == "-D":
    if chain not in table or spec not in table[chain]: sys.exit(1)
    table[chain].remove(spec)
elif op == "-L":
    sys.exit(0 if chain in table else 1)
elif op == "-S":
    print("-N " + chain)
    for rule in table.get(chain, []):
        print("-A %s %s" % (chain, " ".join(rule)))
    sys.exit(0)
else:
    sys.exit("unsupported " + op)
save()
'''


class TestT26LaunchesNeedTheImdsGuard(FakeHost):
    """T26 (M26): every launch wrapper refuses to start a container while
    host/imds_guard.sh's rules are not in place (a reboot drops them), before the
    running sim is touched; SPOT_IMDS_GUARD=off starts it anyway and says what that
    costs. Not covered: the guard's rules against the host's real iptables."""

    def test_no_guard_no_launch(self):
        for rel, args in WRAPPERS:
            with self.subTest(rel):
                os.remove(self.docker_log) if os.path.exists(self.docker_log) else None
                result = self.run_script(rel, *args, FAKE_IMDS_GUARD="absent")
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("the IMDS guard is not in place", result.stderr)
                self.assertIn("imds_guard.sh apply", result.stderr)
                self.assertEqual(self.mutating_calls(), [])

    def test_off_launches_with_a_warning(self):
        result = self.run_script("host/run_hud.sh", FAKE_IMDS_GUARD="absent", SPOT_IMDS_GUARD="off")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("SPOT_IMDS_GUARD=off", result.stderr)
        self.assertIn("take the instance role's credentials", result.stderr)
        self.assertEqual(len(self.runs()), 1)


class TestT26ImdsGuard(FakeHost):
    """T26 (host part): host/imds_guard.sh lets only root, the agent's uid and, where it
    is installed, the DCV server's account reach IMDS.

    apply adds a SPOT-IMDS chain (RETURN for uid 0, SPOT_AGENT_UID 10001 and the dcv
    account's uid if there is one, REJECT for every other uid) and one OUTPUT jump for 169.254.169.254 (and fd00:ec2::254 with
    ip6tables); it is idempotent, and remove deletes exactly what apply added, leaving
    other rules alone. The uids come from host/sim_lib.sh, where each container has its
    own. Not run anywhere: rule generation and argument handling are tested against a
    fake iptables only; the rules are not persistent across a reboot.
    """

    def setUp(self):
        super().setUp()
        self.xt = os.path.join(self.tmp, "xt")
        os.makedirs(self.xt)
        self.env["FAKE_XT_DIR"] = self.xt
        for tool in ("iptables", "ip6tables"):
            self.add_fake(tool, FAKE_IPTABLES)
            with open(os.path.join(self.xt, tool + ".json"), "w", encoding="utf-8") as handle:
                json.dump({"OUTPUT": [["-p", "udp", "--dport", "53", "-j", "ACCEPT"]],
                           "INPUT": [], "FORWARD": [], "DOCKER-USER": [["-j", "RETURN"]]}, handle)
        self.before = {tool: self.table(tool) for tool in ("iptables", "ip6tables")}
        self.add_fake("getent", "#!/bin/sh\nexit 2\n")   # no dcv account unless a test adds one

    def table(self, tool):
        with open(os.path.join(self.xt, tool + ".json"), encoding="utf-8") as handle:
            return json.load(handle)

    def guard(self, *args):
        return self.run_script("host/imds_guard.sh", *args)

    def test_apply_lets_only_root_and_the_agent_through(self):
        result = self.guard("apply")
        self.assertEqual(result.returncode, 0, result.stderr)
        uid = str(_sim_lib_uids()["SPOT_AGENT_UID"])
        for tool, dest in (("iptables", "169.254.169.254/32"), ("ip6tables", "fd00:ec2::254/128")):
            table = self.table(tool)
            self.assertEqual(table["OUTPUT"][0], ["-d", dest, "-j", "SPOT-IMDS"])
            self.assertEqual(table["OUTPUT"][1:], self.before[tool]["OUTPUT"])
            self.assertEqual(table["SPOT-IMDS"], [
                ["-m", "owner", "--uid-owner", "0", "-j", "RETURN"],
                ["-m", "owner", "--uid-owner", uid, "-j", "RETURN"],
                ["-p", "tcp", "-j", "REJECT", "--reject-with", "tcp-reset"],
                ["-j", "REJECT"]])
            self.assertEqual(table["DOCKER-USER"], self.before[tool]["DOCKER-USER"])

    def test_refused_uids_are_every_other_container(self):
        uids = _sim_lib_uids()
        allowed = {"0", str(uids["SPOT_AGENT_UID"])}
        self.assertEqual(self.guard("apply").returncode, 0)
        returned = {r[3] for r in self.table("iptables")["SPOT-IMDS"] if r[-1] == "RETURN"}
        self.assertEqual(returned, allowed)
        for name in ("SPOT_SIM_UID", "SPOT_HUD_UID", "SPOT_ONESHOT_UID"):
            self.assertNotIn(str(uids[name]), returned)
        self.assertNotIn("1000", returned)
        header = _read("host", "imds_guard.sh")
        self.assertIn("--uid-owner %d -j RETURN" % uids["SPOT_AGENT_UID"], header)

    def _fake_dcv_account(self, uid):
        self.add_fake("getent", '#!/bin/sh\n[ "$1 $2" = "passwd dcv" ] || exit 2\n'
                                'echo "dcv:x:%s:%s:DCV server:/var/lib/dcv:/usr/sbin/nologin"\n' % (uid, uid))

    def test_the_dcv_server_is_let_through_when_installed(self):
        """Measured on the GPU host (2026-11-01): the Amazon DCV server runs as the system
        account dcv and, on EC2, reads its licence through the instance role; with only
        root and the agent let through it retries IMDS for ever and never listens. When
        the dcv account exists, its uid is let through as well, and nobody else."""
        self._fake_dcv_account(988)
        result = self.guard("apply")
        self.assertEqual(result.returncode, 0, result.stderr)
        uid = str(_sim_lib_uids()["SPOT_AGENT_UID"])
        for tool in ("iptables", "ip6tables"):
            self.assertEqual(self.table(tool)["SPOT-IMDS"], [
                ["-m", "owner", "--uid-owner", "0", "-j", "RETURN"],
                ["-m", "owner", "--uid-owner", uid, "-j", "RETURN"],
                ["-m", "owner", "--uid-owner", "988", "-j", "RETURN"],
                ["-p", "tcp", "-j", "REJECT", "--reject-with", "tcp-reset"],
                ["-j", "REJECT"]])
        self.assertIn("the DCV server (uid 988)", result.stderr)

    def test_a_dcv_uid_shared_with_a_container_or_ubuntu_is_refused(self):
        uids = _sim_lib_uids()
        for uid in (uids["SPOT_SIM_UID"], uids["SPOT_HUD_UID"], uids["SPOT_ONESHOT_UID"], 1000, 0):
            with self.subTest(uid=uid):
                self._fake_dcv_account(uid)
                result = self.guard("apply")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("FATAL", result.stderr)
                self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, self.before)

    def test_apply_is_idempotent(self):
        self.assertEqual(self.guard("apply").returncode, 0)
        once = {tool: self.table(tool) for tool in ("iptables", "ip6tables")}
        self.assertEqual(self.guard("apply").returncode, 0)
        self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, once)

    def test_remove_deletes_exactly_what_apply_added(self):
        self.assertEqual(self.guard("apply").returncode, 0)
        self.assertEqual(self.guard("remove").returncode, 0)
        self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, self.before)
        self.assertEqual(self.guard("remove").returncode, 0)        # nothing left to remove
        self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, self.before)

    def test_status_reports_both_states(self):
        result = self.guard("status")
        self.assertEqual(result.returncode, 1)
        self.assertIn("NOT APPLIED", result.stdout)
        self.guard("apply")
        result = self.guard("status")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("APPLIED", result.stdout)
        self.assertIn("--uid-owner 10001 -j RETURN", result.stdout)

    def test_dry_run_prints_and_changes_nothing(self):
        result = self.guard("apply", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("iptables -w -I OUTPUT 1 -d 169.254.169.254/32 -j SPOT-IMDS", result.stdout)
        self.assertIn("ip6tables -w -A SPOT-IMDS -j REJECT", result.stdout)
        self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, self.before)

    def test_bad_arguments(self):
        for args in ((), ("enable",), ("apply", "--force"), ("status", "--dry-run"), ("apply", "--dry-run", "x")):
            with self.subTest(args):
                self.assertEqual(self.guard(*args).returncode, 2)
        self.assertEqual({tool: self.table(tool) for tool in ("iptables", "ip6tables")}, self.before)


class TestT23PresenterGasRefusesWhatTheSimRefuses(FakeHost):
    """T23 (presenter console): inject_gas.sh refuses O2 enrichment and extreme tau.

    The sim refuses positive O2 deltas (they would mask a depletion) and ventilation
    tau_s below 2 s; inject_gas.sh now refuses O2 amounts outside [-20.9, 0) and tau_s
    outside 2-3600 s itself, before any container starts. The sim-side checks are the
    sim package's.
    """

    def test_o2_must_be_a_depletion(self):
        for amount in ("5", "0", "0.0", "-21", "-20.95", "80"):
            with self.subTest(amount):
                result = self.run_script("host/inject_gas.sh", "o2", amount)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("depletion", result.stderr)
        self.assertEqual(self.runs(), [])
        for amount in ("-20.9", "-3", "-0.1"):
            with self.subTest(amount):
                self.assertEqual(self.run_script("host/inject_gas.sh", "o2", amount).returncode, 0)

    def test_tau_bounds(self):
        for tau in ("1.9", "0", "3600.5", "9999"):
            with self.subTest(tau):
                result = self.run_script("host/inject_gas.sh", "vent", "on", tau)
                self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.runs(), [])
        for tau in ("2", "15", "3600"):
            with self.subTest(tau):
                self.assertEqual(self.run_script("host/inject_gas.sh", "vent", "on", tau).returncode, 0)
        self.assertEqual(self.run_script("host/inject_gas.sh", "vent", "on").returncode, 0)

    def test_help_text_no_longer_offers_enrichment(self):
        header = _read("host", "inject_gas.sh").split("set -euo pipefail")[0].lower()
        self.assertNotIn("signed", header)
        self.assertNotIn("enrich", header)


class TestT16T38AgentContainerStopsCleanlyAndNamesTheOperator(FakeHost):
    """T16 (M16) and T5/T38 (M5, M38): run_agent.sh gives the agent a 900 s stop
    timeout, so `docker stop` lets it bring Spot out before it exits, and passes the
    operator's user name (and an SSM session id when one is set) into the container,
    so the mission log names who answered each console confirmation."""

    def test_stop_timeout_and_operator_identity(self):
        result = self.run_script("host/run_agent.sh", "--once", "status", USER="presenter1",
                                 SSM_SESSION_ID="presenter1-0abc")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        # The stop timeout covers the agent's own exit wait (retreat retries included).
        wait = re.search(r"^EXIT_RETREAT_WAIT_S = ([0-9.]+)$", _read("agent", "spot_agent.py"), re.M)
        self.assertGreaterEqual(float(argv[argv.index("--stop-timeout") + 1]), float(wait.group(1)))
        self.assertIn("USER=presenter1", argv)
        self.assertIn("SSM_SESSION_ID=presenter1-0abc", argv)
        self.assertFalse(any(a.startswith("SPOT_OPERATOR_LOGIN=") for a in argv))

    def test_the_login_that_ran_sudo_is_passed(self):
        """T38: after `sudo -iu ubuntu`, SUDO_USER names the login that ran it (with
        Session Manager Run As per operator, the operator's own account); the agent
        records it as the operator's login."""
        result = self.run_script("host/run_agent.sh", "--once", "status", USER="ubuntu",
                                 SUDO_USER="op-alice")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        self.assertIn("SPOT_OPERATOR_LOGIN=op-alice", argv)

    def test_no_session_variable_when_none_is_set(self):
        result = self.run_script("host/run_agent.sh", "--mission", USER="presenter1")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        self.assertFalse([a for a in argv if a.startswith("SSM_SESSION_ID=")])


    def test_the_deployed_commit_goes_into_the_container(self):
        with open(os.path.join(self.demo, "VERSION"), "w") as handle:
            handle.write("commit=0123456789abcdef0123456789abcdef01234567\ndeployed_utc=2026-10-30T00:00:00Z\n")
        result = self.run_script("host/run_agent.sh", "--once", "status")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        self.assertIn("SPOT_DEMO_VERSION=0123456789abcdef0123456789abcdef01234567", argv)

    def test_an_unknown_version_when_nothing_was_deployed(self):
        result = self.run_script("host/run_agent.sh", "--once", "status")
        self.assertEqual(result.returncode, 0, result.stderr)
        (argv,) = self.runs()
        self.assertIn("SPOT_DEMO_VERSION=unknown", argv)

    def test_an_existing_container_is_never_killed_by_the_hint(self):
        text = _read("host/run_agent.sh")
        self.assertNotIn("docker rm -f $NAME", text)


class TestT43E2eClientChecksTheBackend(unittest.TestCase):
    """T43: host/e2e_client.py drives /demo_spot directly; before its first command it
    waits for a payload/status that says "simulated": true and stops otherwise, so
    pointed at anything but the simulator it sends nothing."""

    def test_the_check_comes_before_any_command(self):
        text = _read("host", "e2e_client.py")
        main = text[text.index("def main():"):]
        check = main.index('check("backend is the simulator"')
        for command in ("c.call(", "c.drive(", "c.probe(", "publish("):
            if command in main:
                self.assertLess(check, main.index(command), command)
        self.assertIn('c.status.get("simulated") is True', main)


class TestT36WindowOnlyCapture(FakeHost):
    """T36 (M36): snap_screens.sh captures the Isaac Sim window, never the whole desktop
    unless SPOT_SNAP_ROOT=1 asks for it, and refuses when it finds no such window.
    Not covered: xwininfo and ImageMagick against the real DCV console (the GPU host)."""

    TREE = (r'xwininfo: Window id: 0x1e6 (the root window) (has no name)\n'
            r'     0x2400003 \"Terminal\": (\"gnome-terminal\" \"Terminal\")  800x600+0+0  +0+0\n'
            r'     0x3a00007 \"Isaac Sim 6.1.0\": (\"kit\" \"Kit\")  1920x1080+0+0  +0+0\n')

    def setUp(self):
        super().setUp()
        self.add_fake("pgrep", "#!/bin/sh\necho 4242\n")
        self.add_fake("xwininfo", '#!/bin/sh\nprintf "%b" "${FAKE_XWININFO:-}"\n')
        self.add_fake("import", '#!/bin/sh\necho "import $*" >> "$FAKE_CALLS"\n')
        self.screens = os.path.join(self.demo, "logs", "screens")
        os.makedirs(self.screens)

    def snap(self, **env):
        return self.run_script("tools/snap_screens.sh", "t36", SPOT_SCREENS_DIR=self.screens,
                               SPOT_SNAP_HUD="0", **env)

    def test_only_the_isaac_window(self):
        result = self.snap(FAKE_XWININFO=self.TREE)
        self.assertIn("import -silent -window 0x3a00007 ", self.host_calls())
        self.assertNotIn("-window root", self.host_calls())
        self.assertNotIn("no window named", result.stderr)

    def test_no_isaac_window_means_no_capture(self):
        result = self.snap(FAKE_XWININFO=self.TREE.split("     0x3a")[0])
        self.assertEqual(result.returncode, 1)
        self.assertIn("not capturing the whole desktop", result.stderr)
        self.assertNotIn("import", self.host_calls())

    def test_the_whole_desktop_only_on_request(self):
        self.snap(FAKE_XWININFO=self.TREE, SPOT_SNAP_ROOT="1")
        self.assertIn("import -silent -window root ", self.host_calls())


class TestT28SharedLogsDirectoryWrites(FakeHost):
    """T28 and T36 (M28, M36): containers write logs/ and logs/screens through the
    directory's group, so the host scripts never follow a name a container planted
    there. The shared directory is sticky; snap_screens.sh writes each capture into a
    private directory and moves it in with `mv -n`, so a planted symlink is neither
    followed nor replaced; tools/rehearse_rep.sh, which runs as root, creates its files
    with O_EXCL (noclobber), refuses a name that already exists as a file or a link,
    and writes through descriptors it opened before any container that writes logs/
    runs; spot_new_log refuses any existing entry under the log's name. Not run on
    the host: rehearse_rep.sh is checked by reading it."""

    def test_the_shared_directory_is_sticky(self):
        shared = os.path.join(self.tmp, "shared")
        os.makedirs(shared)
        result = self.sim_lib("spot_shared_dir_gid", shared)
        self.assertEqual(result.returncode, 0, result.stderr)
        mode = os.stat(shared).st_mode
        self.assertTrue(mode & stat.S_ISVTX)
        self.assertTrue(mode & stat.S_IWGRP)

    def test_a_planted_link_is_not_followed_by_a_capture(self):
        self.add_fake("pgrep", "#!/bin/sh\necho 4242\n")
        self.add_fake("xwininfo", '#!/bin/sh\nprintf "%b" "$FAKE_XWININFO"\n')
        self.add_fake("import", '#!/bin/sh\nfor a; do out="$a"; done; echo captured > "$out"\n')
        screens = os.path.join(self.demo, "logs", "screens")
        os.makedirs(screens)
        victim = os.path.join(self.tmp, "victim.txt")
        with open(victim, "w", encoding="utf-8") as handle:
            handle.write("untouched\n")
        os.symlink(victim, os.path.join(screens, "sim-t28.png"))
        result = self.run_script("tools/snap_screens.sh", "t28", SPOT_SCREENS_DIR=screens,
                                 SPOT_SNAP_HUD="0", FAKE_XWININFO=TestT36WindowOnlyCapture.TREE)
        self.assertEqual(result.returncode, 1)
        self.assertIn("already exists; not overwritten", result.stderr)
        with open(victim, encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "untouched\n")
        self.assertTrue(os.path.islink(os.path.join(screens, "sim-t28.png")))

    def test_a_planted_directory_link_or_fifo_is_refused(self):
        """A link to a directory under a capture's name is refused (mv would move the
        capture into it), and spot_new_log refuses a FIFO under the log's name
        (noclobber alone would open it)."""
        self.add_fake("pgrep", "#!/bin/sh\necho 4242\n")
        self.add_fake("xwininfo", '#!/bin/sh\nprintf "%b" "$FAKE_XWININFO"\n')
        self.add_fake("import", '#!/bin/sh\nfor a; do out="$a"; done; echo captured > "$out"\n')
        screens = os.path.join(self.demo, "logs", "screens")
        os.makedirs(screens)
        elsewhere = os.path.join(self.tmp, "elsewhere")
        os.makedirs(elsewhere)
        os.symlink(elsewhere, os.path.join(screens, "sim-t28d.png"))
        result = self.run_script("tools/snap_screens.sh", "t28d", SPOT_SCREENS_DIR=screens,
                                 SPOT_SNAP_HUD="0", FAKE_XWININFO=TestT36WindowOnlyCapture.TREE)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(os.listdir(elsewhere), [])
        logs = os.path.join(self.demo, "logs")
        os.mkfifo(os.path.join(logs, "smoke-run-9.log"))
        result = self.sim_lib("spot_new_log", "smoke", "run-9")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("cannot create a new log file", result.stderr)

    def test_rehearsal_creates_its_files_exclusively(self):
        text = _read("tools", "rehearse_rep.sh")
        self.assertIn("\nset -C\n", text)
        self.assertIn('exec 4> "$TIM"', text)
        self.assertIn('if [ -e "$f" ] || [ -L "$f" ]', text)
        self.assertNotIn("chown", text)
        self.assertNotRegex(text, r'script -qfc[^\n]*\n[^\n]*"\$TR" <')   # script writes no file by name
        self.assertIn('/dev/null < "$FIFO" >&5', text)
        # Every root-written file is opened before any container that writes logs/ runs.
        start_rec = text.index("docker run -d --name spot-rec")
        for opened in ('exec 4> "$TIM"', 'exec 5> "$TR"', 'exec 6> "$SCR"'):
            self.assertLess(text.index(opened), start_rec, opened)
        self.assertLess(text.index("docker rm spot-agent spot-rec"), text.index('exec 4> "$TIM"'))
        self.assertIn(">&6 2>&1", text)


if __name__ == "__main__":
    unittest.main()
