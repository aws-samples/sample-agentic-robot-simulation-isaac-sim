# Agentic robot simulation with NVIDIA Isaac Sim: confined-space entry scenario

> **Sample code. Not for production use, and not a safety system.** This is sample code,
> for non-production usage. You should work with your security and legal teams to meet
> your organizational security, regulatory and compliance requirements before
> deployment. The safety logic here shows how such logic could be structured; it is not a
> protective function, has not been assessed under any functional-safety standard, and
> must not be relied on to protect people, equipment or a site. The sample supports the
> Isaac Sim simulator only: it refuses to command a real robot.
>
> This repository and any related content are provided for informational purposes only.
> You are responsible for making your own independent assessment of the information and
> referenced content provided, including testing, securing and optimizing any
> suggestions. This information and referenced content create no warranties,
> representations, commitments or assurances from AWS. AWS takes no responsibility and
> assumes no liability for any information or content referenced. This repository and
> related sample code are **not intended for use in real robots, in the physical
> world**. You should consult with all necessary experts, including but not limited to
> workplace and robotic safety engineers, and take all necessary safety measures when
> you independently consider using real robots.

## What this is

This repo is a demo of a Spot quadruped robot doing a permit-required
confined-space entry into a crude-oil storage tank, so that no person has to
go inside. The robot, the tank and the gas are simulated in NVIDIA Isaac Sim
6.1 on an AWS EC2 GPU instance. An AI operator agent (Strands Agents with
Claude Sonnet 5 on Amazon Bedrock) takes plain-English instructions,
prepares the permit, tests the air through the tank door, ventilates, enters
and explains what happened. Deterministic safety code, not the language
model, decides when entry is allowed and pulls the robot out if the gas
readings go over the limits. A browser dashboard (the HUD) shows the permit,
gas readings, map, camera and audit log to the audience.

**The robot is not the point, and it is replaceable.** The agent never talks to a
robot SDK. It talks to a ROS 2 facade whose topics, services and actions follow the
open source `spot_ros2` contract ([`docs/facade-contract-v1.md`](docs/facade-contract-v1.md)),
so **any robot with a ROS 2 interface can take its place** by publishing the same
topics: the quadruped here is simply the one NVIDIA Isaac Sim ships as an asset. What
is worth copying is the pattern — an LLM agent that proposes, deterministic code that
decides, and an operator who confirms — not the choice of robot.

**Status (2026-09-27):** rehearsed end to end in simulation, including a
five-repetition leadership rehearsal. **Honest caveats:** this is a
simulation only. All gas readings, the gas probe and the ventilation blower
are simulated. Base Spot is not ATEX or IECEx certified for explosive
atmospheres. Nothing has run on a real robot or against the real `spot_ros2`
driver. The topic and service names follow `spot_ros2` so that the design
can be compared with a real driver, but the agent refuses to command any
backend other than this simulator: a real robot needs its own safety case
and security review ([Roadmap](#roadmap-to-the-real-robot)).

## Contents

1. [Start here](#start-here)
2. [How it fits together](#how-it-fits-together)
3. [Repository layout](#repository-layout)
4. [The demo storyline](#the-demo-storyline)
5. [Quick start](#quick-start)
6. [ROS 2 interface](#ros-2-interface)
7. [Testing](#testing)
8. [Verified results](#verified-results)
9. [AWS resources and cost](#aws-resources-and-cost)
10. [Known limitations and honest caveats](#known-limitations-and-honest-caveats)
11. [Roadmap to the real robot](#roadmap-to-the-real-robot)
12. [Security](#security)
13. [Cleanup](#cleanup)
14. [Contributing](#contributing)

## Start here

| I am a... | Read this first | Then |
|---|---|---|
| Leader or presenter | [`docs/LEADERSHIP-BRIEF.md`](docs/LEADERSHIP-BRIEF.md): the 8-minute script, what to say, the caveats to say out loud, and the never-say list | [`docs/RUNBOOK.md`](docs/RUNBOOK.md), section "Presenter cheat sheet" |
| HSE reviewer | [The demo storyline](#the-demo-storyline) below, then the "Honest caveats" in [`docs/LEADERSHIP-BRIEF.md`](docs/LEADERSHIP-BRIEF.md) | [`docs/evidence/README.md`](docs/evidence/README.md) (screenshots and logs behind every claim); `agent/prompts.py` (the regulatory wording the agent must use) |
| Operator running the demo | [`docs/RUNBOOK.md`](docs/RUNBOOK.md): steps 0-9 from the Mac, with measured timings | [`host/gui_notes.md`](host/gui_notes.md) (GUI/DCV troubleshooting) |
| Engineer | [`docs/design.md`](docs/design.md) (why Isaac Sim; note that its architecture diagram is the original plan, see [caveats](#known-limitations-and-honest-caveats)) and [`docs/facade-contract-v1.md`](docs/facade-contract-v1.md) (the `spot_ros2` names the sim copies) | [`docs/capability-matrix.md`](docs/capability-matrix.md), [`docs/isaac-api-notes.md`](docs/isaac-api-notes.md), [`hud/README.md`](hud/README.md) |
| New contributor | This README: [Repository layout](#repository-layout), [Testing](#testing), [Contributing](#contributing) | Run the unit tests on your Mac, then the HUD in fake mode (no AWS needed) |
| Checking a number | [`docs/evidence/README.md`](docs/evidence/README.md) | [Verified results](#verified-results) below lists each number with its source |

## How it fits together

Everything runs on one EC2 GPU host. The presenter's Mac reaches it only
through AWS Systems Manager (SSM); the instance has no inbound ports.

```
 Presenter's Mac
 +--------------------------------------------------------------------------------+
 |  Browser 1: Isaac Sim desktop (NICE DCV)  https://localhost:8443  connect-dcv.sh |
 |  Browser 2: presenter HUD                 http://localhost:8080   connect-hud.sh |
 |  Terminals: host shells A and B (aws ssm start-session), deploy-to-host.sh,      |
 |             fetch-log.sh                                                        |
 +--------------------------------------------------------------------------------+
            |
            |  AWS Systems Manager only: port forwards, shell sessions, Run Command
            |  (security group has no ingress rules; no SSH, no public ports)
            v
 EC2 g6.4xlarge (1x NVIDIA L4, Ubuntu 24.04, us-west-2)   code deployed at /opt/spot-demo
 +--------------------------------------------------------------------------------+
 |  container spot-sim   (nvcr.io/nvidia/isaac-sim:6.1.0, started by run_sim.sh)   |
 |    sim/isaac_app.py: Spot + bundled RL walking policy, tank scene, gas model,   |
 |    ROS 2 node "spot_facade" that mimics spot_ros2 under /demo_spot              |
 +--------------------------------------------------------------------------------+
            ^                         ^                            ^
            |   ROS 2 DDS over the host network (UDPv4), topics + Trigger services
            |   under /demo_spot      |                            |
 +----------+-------------+  +--------+-------------------+  +-----+----------------+
 | spot-agent             |  | spot-hud                   |  | presenter console    |
 | (ros:jazzy-ros-base)   |  | (ros:jazzy-ros-base)       |  | inject_gas.sh,       |
 | run_agent.sh           |  | run_hud.sh                 |  | reset_spot.sh,       |
 | agent/spot_agent.py    |  | hud/server.py              |  | set_view.sh          |
 |  (LLM operator)        |  |  127.0.0.1:8080            |  | (one-shot ros:jazzy  |
 | agent/mission.py       |  |  read-only: no publishers, |  |  containers)         |
 |  permit + SafetyGuard  |  |  no service clients        |  |                      |
 +----------+-------------+  +----------------------------+  +----------------------+
            |
            |  HTTPS, EC2 instance role via IMDSv2 (no keys in the container)
            v
 Amazon Bedrock, model us.anthropic.claude-sonnet-5 (us-west-2)
```

**Why the names match `spot_ros2`.** The agent only talks to the robot through
`agent/robot_client.py`, which uses `spot_ros2` topic and service names
(`cmd_vel`, `odometry`, `claim`, `stand`, `estop/hard`, ...) under one
namespace, so the simulated interface is faithful to the real driver's:

```
 this sample:  agent  ->  /demo_spot/*  ->  spot_facade inside Isaac Sim  ->  simulated Spot
```

It does not support a real robot: the agent refuses to command anything but
this simulator, with no switch to turn that off. What a separate project
would need first is listed in
[Roadmap to the real robot](#roadmap-to-the-real-robot).

The `payload/*` topics (gas, probe, ventilation, site, events) are this
demo's own additions. They have no `spot_ros2` equivalent, so a real gas
detector and probe will need their own integration. See
[Roadmap to the real robot](#roadmap-to-the-real-robot).

## Repository layout

```
spot-oil-and-gas-demo/
├── sim/            Isaac Sim app: Spot, the tank scene, the simulated gas field and the
│                   ROS 2 facade. Runs inside the Isaac Sim container on the EC2 host.
├── agent/          The operator side: LLM agent (Strands + Bedrock), scripted no-LLM
│                   mission, permit state machine, SafetyGuard, ROS 2 robot client.
├── hud/            Read-only presenter dashboard (FastAPI + plain JS). Live on the host,
│                   or fake/replay mode on a Mac with no ROS.
├── host/           Scripts you run ON the EC2 host: start/stop the containers, presenter
│                   console (gas, reset, camera), smoke and end-to-end tests, GUI notes.
├── scripts/        Scripts you run on the Mac: SSM tunnels (DCV, HUD), deploy the repo to
│                   the host, pull logs back, run a local script on the host.
├── tools/          Rehearsal and evidence helpers: odometry recorder, screenshot and HUD
│                   snapshot watchers, a scripted rehearsal rep, a run analyzer.
├── infra/          Optional CloudFormation for the adopter: cost guardrails (budget, idle
│                   stop and status-check alarms) and the encrypted session-log group.
├── tests/          946 unit tests that run on a Mac without ROS, Isaac Sim or Bedrock.
├── docs/           Design, runbook, leadership brief, capability matrix, spot_ros2 contract,
│   │               Isaac Sim API notes.
│   └── evidence/   Screenshots, transcripts, logs and analyses behind the published numbers.
│                   Some files are also test fixtures and HUD replay input.
├── .gitignore      Python caches, local venvs, and operator data (logs/, screens/, .env)
└── README.md       This file.
```

Local-only folders you may see in a working copy: `.evidence-archive/`
(gitignored older screenshots) and `.ruff_cache/` (linter cache; it ignores
itself).

**Where each piece runs**

| Folder | Runs on | How |
|---|---|---|
| `sim/` | EC2 host | Inside container `spot-sim` (Isaac Sim 6.1.0), launched by `host/run_sim.sh`. The pure-Python modules also import on a Mac for tests. |
| `agent/` | EC2 host | Inside container `spot-agent` or `spot-mission` (`ros:jazzy-ros-base` + venv), launched by `host/run_agent.sh`. `--dry-run` and tests work on a Mac from a repo checkout. |
| `hud/` | EC2 host (live) or Mac (fake/replay) | Container `spot-hud`, launched by `host/run_hud.sh`; viewed in the presenter's browser through `scripts/connect-hud.sh`. |
| `host/` | EC2 host | From `/opt/spot-demo/host`, as `ubuntu` in an SSM shell or the DCV terminal. |
| `scripts/` | Mac | Needs the AWS CLI, credentials for the sim account and the Session Manager plugin. |
| `tools/` | Mostly EC2 host | `rh_analyze.py` also runs on a Mac on logs pulled with `scripts/fetch-log.sh`. |
| `infra/` | Your AWS account | Optional CloudFormation stacks an administrator reviews and deploys from the Mac (see [`infra/README.md`](infra/README.md)); nothing deploys them for you. |
| `tests/` | Mac | `python3 -m unittest discover -s tests` from the repo root. Also shipped to the host (the agent image needs one file). |
| `docs/` | Anywhere | Documentation only. `docs/evidence/` files are read by tests and the HUD replay mode. |

---

### `sim/` — the simulated robot, tank and gas

**What it is for.** Everything on the simulation side. One process,
`sim/isaac_app.py`, starts Isaac Sim 6.1 with Spot and Isaac's bundled
flat-terrain RL walking policy, builds a procedural oil-tank scene, runs a
simulated gas field, and exposes a ROS 2 facade that behaves like
`spot_ros2` under the namespace `/demo_spot`. The other five files are
modules it imports. Four of them are plain Python/numpy so they can be unit
tested on a Mac.

**Where it runs.** On the EC2 host inside the Isaac Sim container (image
`nvcr.io/nvidia/isaac-sim:6.1.0`, container name `spot-sim`,
`--network=host`, `--gpus all`). `host/run_sim.sh` mounts `/opt/spot-demo`
at `/workspace` and runs `/isaac-sim/python.sh /workspace/sim/isaac_app.py`.
On a Mac, only `gas_field.py`, `battery.py`, `stance_hold.py` and the top
half of `sensors.py` are imported, by the unit tests.

| File | What it does |
|---|---|
| `isaac_app.py` | The only entrypoint (about 1850 lines). Creates the Isaac `SimulationApp`, enables the bundled ROS 2 Jazzy bridge, spawns Spot at `/World/Spot`, builds the scene and gas field, and creates the ROS 2 node `spot_facade` in `/demo_spot` with all topics and Trigger services. Runs a lease/power/stand/sit/e-stop state machine that mirrors `spot_ros2` defaults, a scripted stand-up that hands over to the RL policy, a 0.125 s `cmd_vel` watchdog and real-time pacing (10 physics steps per tick, 50 ticks/s). Prints stdout markers the host scripts wait for, such as `FACADE-READY ... run_id=<id>`, `PERF rtf=...` every 5 s, and `SMOKE-OK`. |
| `build_scene.py` | Builds the tank out of simple USD shapes under `/World/ConfinedSpace`: 6 m radius, 4 m tall, 24 wall segments, a 1.30 x 1.30 m cleanout door facing +X, a visual-only cone roof, sludge patches, a ladder, two pipe runs that force a weaving route, barrels and a pipe rack outside, and the lights. Returns the site layout: spawn (14, 0) facing the tank, entry point (7, 0), door (6, 0), tank centre (0, 0), and 5 interior waypoints. Its docstring explains why Spot (about 1.10 x 0.50 m) fits the door. Container only (imports `pxr`). |
| `gas_field.py` | The simulated atmosphere (numpy only, no Isaac). Deterministic gas pockets (seed 7): H2S pooled low, flammables (LEL), oxygen depletion, limited to the tank. An optional unventilated headspace (the storyline's starting state) and forced ventilation that decays toward a 4 % residual (time constant 15 s by default). Runtime-injected pockets for the staged spike. OSHA 1910.146 Appendix E alarm limits: O2 below 19.5 % or above 23.5 %, LEL at or above 10 %, H2S at or above 10 ppm, CO at or above 35 ppm; non-numeric readings count as an alarm. `ProbeGeometry` enforces the probe rules: 1.1 m horizontal reach from the body front, height 0-3 m, a known body pose, and line of sight only through the doorway. The docstring states that the values are illustrative, not derived from any real tank. |
| `sensors.py` | The visual layer. Top half (pure Python, tested on a Mac): camera list parsing, frame skipping, rotations, the presenter view parser and the `CameraDirector` logic for the `auto`, `chase`, `interior`, `overhead` and `door` views. Bottom half (container only): up to 5 body cameras (frontleft, frontright, left, right, back) published to ROS 2 through OmniGraph camera helpers, Spot's nose lights that fade out away from the tank, and the unpublished GUI chase camera. |
| `battery.py` | A simulated battery percentage that drains linearly with sim time. Modes: `demo` (default, 25 %/h, never below 30 %), `real` (100 % over 90 min, floor 5 %), `off` (always 100 %). Nothing actually consumes energy. Reset by `payload/battery/reset`. |
| `launch_args.py` | `build_parser()`: `isaac_app.py`'s command line, in a module that imports only `argparse`, so the tests import the real parser and `run_sim.sh`'s allowlist is checked against it (T44). |
| `stance_hold.py` | `HeadingHold`: while Spot stands with a zero command, it holds the body heading with a small yaw correction (1 deg dead band, capped at 0.4 rad/s). It fixes a measured slow turn of the standing policy after a walk, which used to push the door probe out of reach and abort the storyline. |

**Entrypoint and flags** (normally passed through `host/run_sim.sh`):

| Flag | Default | Meaning |
|---|---|---|
| `--headless true\|false` | `true` | `false` shows the GUI on the DCV desktop (use `run_sim.sh --gui` or `--editor`). Takes a value; a bare `--headless` fails. |
| `--smoke` | off | About 600 physics steps with automatic claim, power and stand, then `SMOKE-OK` and exit 0 (exit 1 on failure). |
| `--cameras all\|none\|<list>` | `frontleft,frontright` | Body cameras to publish. `--no-cameras` equals `none`. `--camera-hz` (default 10). |
| `--render-hz` | 10 | Stage render rate. 20 Hz drops the GUI below real time. |
| `--gas-initial raw\|ventilated` | `raw` | `raw` = unventilated tank (the storyline start); `ventilated` rehearses the entry alone. |
| `--ui presentation\|editor` | `presentation` | GUI only: fullscreen viewport, or the full Isaac editor. |
| `--view auto\|chase\|interior\|overhead\|door` | `auto` | Initial presenter camera. |
| `--gui-res WxH` | `1280x720` | GUI window size (160-3840 per side). |
| `--battery-drain demo\|real\|off` | `demo` | See `battery.py`. |
| `--device cpu\|cuda` | `cpu` | PhysX device. |
| `--fabric true\|false` | `true` | Faster PhysX-to-Fabric path instead of USD write-back. |
| `--realtime true\|false` | `true` | Pace the sim to the wall clock. |
| `--actuators fast\|newton` | `fast` | `fast` = PD on PhysX joint drives; `newton` = the stock actuators (much slower in profiling). |
| `--scene <usd>` | - | Accepted but ignored (see caveats). |

Environment: `SPOT_RUN_ID` (per-launch id echoed on the readiness lines;
set by `run_sim.sh`) and `SPOT_PROFILE_AT` (start a 2 s profile at that sim
time). Unknown flags are refused before Isaac Sim starts (T44).

---

### `agent/` — the operator agent, mission logic and safety guard

**What it is for.** The software that drives the simulated Spot through the
permit-required confined-space entry. There are two front ends that share
the same libraries:

- `spot_agent.py`: a natural-language operator (Strands Agents on Amazon
  Bedrock). The presenter types instructions; the model picks tools.
- `run_mission.py`: a scripted mission with no LLM. The fallback if Bedrock
  is slow or unavailable, and the reproducible baseline.

Both use `robot_client.py` (the only code that talks to the robot) and
`mission.py` (the permit state machine and the deterministic SafetyGuard).

**Where it runs.** On the EC2 host inside the `spot-agent` image (built from
`agent/Dockerfile`, based on `ros:jazzy-ros-base`), started by
`host/run_agent.sh` with `--network=host` next to the sim. AWS credentials
come from the EC2 instance role; no keys are baked in. The pure-Python parts
import on a Mac without ROS (ROS and Strands are imported lazily), which is
how the unit tests and `--dry-run` work.

| File | What it does |
|---|---|
| `spot_agent.py` | The LLM operator: a REPL (`operator>` prompt) or a one-shot `--once` run. Builds a Strands agent on Bedrock (default model `us.anthropic.claude-sonnet-5`, region `us-west-2`) that runs tools strictly one at a time. Exposes 18 tools: `get_status`, `get_gas_reading`, `prepare_permit`, `claim_and_power_on`, `stand_up`, `sit_down`, `walk_to`, `walk_to_named`, `run_atmospheric_test`, `start_ventilation`, `ventilation_status`, `enter_confined_space`, `full_confined_space_entry`, `retreat_now`, `explain_last_incident`, `emergency_stop`, `inject_gas_demo_spike`, `clear_injected_gas`. Enforces safety gates in code (listed under [The demo storyline](#the-demo-storyline)). A speech post-filter (`sanitize_speech`) rewrites absolute "safe" or "all clear" claims and spoken tool names before anything is printed. |
| `run_mission.py` | Scripted mission CLI. Records the permit, then stand, approach, face the door, test O2, LEL, then H2S/CO with the simulated probe at 6 points, enter through the door and interior waypoints with continuous monitoring, exit and sit. `--storyline` adds the fail, ventilate, re-test, enter sequence. Prints one JSON summary line. |
| `mission.py` | `ConfinedSpaceMission`: permit states `NOT_STARTED -> PRE_ENTRY_TESTING -> ENTRY_AUTHORIZED -> ENTRY_IN_PROGRESS -> COMPLETED` or `ABORTED`; a 20 Hz walk controller with a permit gate (no approach within tank radius + 1.5 m until the permit fields are complete); the 6-point door-probe test judged per gas on its worst point; simulated ventilation with a purge gate (8 % contamination remaining); enter, exit, retreat; and a JSONL audit log mirrored to `payload/events`. `SafetyGuard`: a separate 20 Hz watchdog (see the storyline). |
| `robot_client.py` | `RobotClient`: the only object that talks to Spot, over ROS 2 topics and `std_srvs/Trigger` services using `spot_ros2`-style names. Thread-safe. Treats stale data as missing (pose older than 1 s, gas 2 s, status 5 s). Clamps velocities (\|vx\| <= 1.0, \|vy\| <= 0.5, \|wz\| <= 1.0 m/s or rad/s) and rejects NaN. Gives the retreat exclusive control of motion (a zero command always passes). Every service call has a bounded timeout and returns `(ok, message)` instead of raising. Reads the latched site layout, falling back to `site_defaults.json` after 10 s. |
| `prompts.py` | The system prompt: site description, storyline, SafetyGuard and incident-narration rules, e-stop confirmation, presenter-only controls, spoken style (1-3 short sentences, no tool names), regulatory wording ("within permissible entry limits", never "safe"), and forbidden claims (no ATEX/IECEx or intrinsically-safe claims, no named operators, simulated readings are not field measurements). `build_system_prompt(site)` appends the live named places. |
| `site_defaults.json` | Fallback site layout in metres (spawn, entry, door, 5 waypoints, tank centre and radius), used when the latched `payload/site` topic has not arrived within 10 s. Also read by the HUD. |
| `Dockerfile` | Builds `spot-agent`: `ros:jazzy-ros-base` plus a venv at `/opt/agent-venv` (with system site packages, so `rclpy` and numpy still work) holding `strands-agents` and `boto3`, installed from `requirements.lock` with `--require-hashes`. Uses UDPv4 DDS (shared memory drops samples between containers with different users). The entrypoint runs `spot_agent.py` as PID 1. |
| `requirements.txt`, `requirements.lock` | `requirements.txt` holds the accepted ranges (`strands-agents>=1.57,<2`, `boto3>=1.35,<2`, `mcp>=1.30,<2`) and is only the input to the lock; nothing installs from it. `requirements.lock` pins every package, dependencies included, to one version with its hashes, the image installs it with `--require-hashes`, and CI audits it and tests the same versions from `tests/requirements-ci.lock`. `rclpy` comes from the ROS image and numpy from apt. |

**`spot_agent.py` flags**

| Flag | Default | Meaning |
|---|---|---|
| (none) | - | Interactive REPL. Needs a TTY. `quit` or Ctrl-D exits, except while Spot is inside the permit space. Console commands (below) work at any time. |
| `--once "<instruction>"` | - | Run one instruction and exit (a new session each time, so the permit does not carry over). |
| `--dry-run` | off | Kinematic fake robot and the sim's gas model plus the real mission and SafetyGuard; no ROS or Isaac. Still calls Bedrock. Works in the image and from a full repo checkout. |
| `--model` | `us.anthropic.claude-sonnet-5` | Bedrock model or inference profile id. The default is the one the rehearsals were run with; any other is logged as untested, because the demo's narration rules live in the prompt. |
| `--guardrail-id` / `--guardrail-version` | `SPOT_GUARDRAIL_ID` / `SPOT_GUARDRAIL_VERSION` | The Amazon Bedrock guardrail applied to the operator's instruction and the model's reply. **On by default**: deploy [`infra/bedrock-guardrail.yaml`](infra/bedrock-guardrail.yaml) and set those two variables from its outputs, or pass the flags for one session. The role needs `bedrock:ApplyGuardrail` on it, which the shipped policy has (see [`docs/iam/`](docs/iam/README.md)). |
| `--no-guardrail` | off | Run with no guardrail. The agent otherwise refuses to start when none is configured. It warns loudly, and the session record says the guardrail was off. The deterministic controls (the facade's limits, the typed robot data, the simulator-only check and the operator's confirmations) do not depend on the guardrail. |
| `--region` | `us-west-2` | Bedrock region. |
| `--namespace` | `/demo_spot` | Robot namespace. Only the simulator's own `/demo_spot` facade is ever identified, so any other value means the agent refuses to start. |
| `--log-path` | `logs/mission-<utc>.jsonl` in the mounted logs directory | Audit log (in the container, `/workspace/logs`, which is `/opt/spot-demo/logs` on the host). The agent does not start if it cannot write it. Each line carries the SHA-256 of the line before, so an edit shows. The console prints the chain head (record count and SHA-256 of the last record) as `[audit head]` lines, which the Session Manager session log keeps off the host; `python3 agent/mission.py --verify LOG --heads-from SESSION_LOG` checks a log against them ([runbook](docs/RUNBOOK.md)). |
| `--connect-timeout` / `--turn-timeout` | 30 s / 900 s | Robot connection wait; time limit per instruction. |
| `--session-max-turns` / `--session-max-output-tokens` / `--session-max-input-tokens` | 300 / 250000 / 6000000 | Model budget for the whole session. Once spent, instructions are refused without calling the model; console commands, stops, `/retreat` and the SafetyGuard keep working. A restart starts a new budget. |
| `--verbose` | off | More logging. |
| `--rehearsal-auto-confirm` | off | Automated rehearsal only (`tools/rehearse_rep.sh`): answers every console confirmation with yes, against the simulator only, and logs each one as auto-confirmed. |
| `--unattributed-run` | off | Start although the login that started the session is a shared account (`ssm-user`, `ubuntu`, `root`) or unknown; without it the agent refuses and says how to connect as your own account. The session and mission-start records then say the run names no operator. |

Exit codes: 0 ok; 1 the `--once` turn failed or timed out; 2 the robot
connection (or the `--dry-run` fakes) or the Strands agent could not be set
up; 130 on Ctrl-C or `docker stop`. Environment:
`SPOT_DEMO_PRESENTER=1` enables the two gas tools for the presenter's own
session only.

**What the operator confirms at the console.** The model cannot attest or
release anything physical on its own. When it needs one of the actions
below, the console prints what it is about to do and asks `y/N`; anything
but `y` or `yes`, or no answer within 60 s, refuses it, and the answer, the
time and the operator (`USER`, the login that ran `sudo -iu ubuntu`, and an SSM
session id if one is set) go into the mission log, and the console then prints
the log's chain head (`[audit head]`):

* the permit attestations (energy isolation, lockout-tagout, attendant and
  entry supervisor at the entry);
* entering the permit space;
* an e-stop the model asks for while Spot is inside (hard or gentle; both end
  with motor power off; motion halts at once and only the power cut waits),
  `self_right` inside, and releasing an e-stop.

Stopping never waits for a confirmation. These console commands work at any
time, also while the model is busy, and never go to the model:

| Command | What it does |
|---|---|
| `/stop` or `stop` | Halt motion now (power stays on; a running retreat continues) |
| `/retreat` | Bring Spot out of the permit space along its path |
| `/estop` | Gentle e-stop (settle, then cut motor power); `/estop hard` cuts power at once |
| `/status` | Robot, permit, gas and SafetyGuard state |
| `/release` | Release the e-stop (asks for confirmation) |
| `/help` | The list |

The console's own lines and the model's are told apart by their start: every
line of the model's reply starts with `agent> `, and the model's text cannot
produce the console's markers (`[readings]`, `[OPERATOR ATTENTION]`, the
confirmation header). A request the operator refused, or let expire, is not
asked again for 30 s, and at most two questions wait at once; anything else the
model asks for meanwhile is refused unasked. Stops, `/retreat` and `/status`
also work while the agent waits to exit for a retreat to finish.

**Simulator only.** The agent and the scripted mission command a backend
only after its `payload/status` has identified it as this simulator. Against
anything else they send nothing at all, not even a stop, and say why; the
SafetyGuard starts only once the simulator is identified. There is no flag to
change that.

**`run_mission.py` flags**

| Flag | Default | Meaning |
|---|---|---|
| `--storyline` | off | Test (expected FAIL), ventilate, wait for the purge, re-test, enter, exit. Without it, a failed test aborts. |
| `--no-gas-reset` | off | Storyline only: do not clear injected gas and reset the tank first. |
| `--vent-tau S` / `--vent-wait S` / `--purge-target F` / `--max-retests N` | the sim's 15 / 180 / 0.08 / 2 | Ventilation tuning: time constant, max purge wait, fraction remaining needed for entry, re-tests before entry is denied. |
| `--inject-h2s-at-waypoint N` | - | Stage an H2S pocket at interior waypoint N (0-based; logged openly as `demo_inject`). `--inject-ppm` (80), `--inject-radius` (1.0 m), `--inject-z` (0.3 m). |
| `--no-auto-retreat` | off | Rehearsal only: alarms are logged but Spot keeps going. Never use in the demo. |
| `--log PATH` | `logs/mission-<utc>.jsonl` in the mounted logs directory | Audit log, appended and hash-chained like the agent's, with the same `[audit head]` console lines. |
| `--unattributed-run` | off | Start although the login that started the session is a shared account or unknown; `mission_start` then records the run as unattributed. Without it the run is refused before anything connects. |
| `--permit-id`, `--attendant`, `--rescue-plan`, `--no-isolation`, `--no-loto` | generated `CSE-<utc>` id, labelled demo values, isolation and LOTO confirmed | Permit fields. `--no-isolation` or `--no-loto` leaves one unconfirmed to show that Spot will not move. |
| `--namespace`, `--connect-timeout`, `--dwell`, `-v` | `/demo_spot`, 30, 4 s | Connection and pacing (only the `/demo_spot` facade is ever identified as the simulator). |

Exit codes: 0 permit COMPLETED; 2 ABORTED as designed (entry denied, or a
retreat on an alarm); 1 any failure.

---

### `hud/` — the presenter dashboard

**What it is for.** A read-only web dashboard for the audience, laid out for
a 1920x1080 projector. It shows the permit state and checklist, four gas
gauges with the OSHA thresholds (O2, LEL, H2S, CO), ventilation and purge
hold, the door-probe table, a top-down tank map with Spot's path, the
front-left camera, status chips (lease, power, stance, e-stop, battery,
speed, sim time, sim link), the mission log, and a full-screen alarm flash. The footer
always says the run is a simulation and the gas, ventilation and probe are
simulated. **By design it creates no ROS publishers or service clients, so it
cannot command the robot.** More detail: [`hud/README.md`](hud/README.md).

**Where it runs.** Three ways:
1. Live, on the EC2 host in container `spot-hud` (built from
   `hud/Dockerfile`), bound to `127.0.0.1:8080`. The presenter opens it on
   the Mac at `http://localhost:8080` through `scripts/connect-hud.sh`.
2. Fake mode on a Mac: a 45-second synthetic scenario, no ROS or AWS.
3. Replay mode on a Mac: replays a recorded run from `docs/evidence/`.

| File | What it does |
|---|---|
| `server.py` | FastAPI app and CLI. Routes: `GET /`, `/static/*`, `GET /api/state` (full snapshot), `GET /api/camera.jpg`, `GET /api/audit.jsonl` and `/api/audit.csv` (audit downloads), `WS /ws` (state pushes). Rejects cross-origin WebSocket connections and sets strict security headers. Picks the live, fake or replay data source from the flags. |
| `ros_bridge.py` | The data layer. `HudState` is the thread-safe store (permit and checklist derivation, purge hold, alarm de-duplication, staged-injection disclosure). `RosBridge` is the ROS 2 node `presenter_hud` (subscriptions only). `FakeBridge` is the synthetic scenario. `ReplayBridge` rebuilds a run from recorded events. Holds copies of the OSHA thresholds and the site layout, because the HUD container does not ship `sim/`. |
| `audit.py` | Formats the audit downloads (JSONL, and CSV with a formula-injection guard because messages include model output). Covers only the events this HUD saw, within bounded per-source buffers that keep every alarm, permit and retreat record through a flood; the authoritative log is the mission JSONL on the host. |
| `replay_screens.py` | Mac-only tool: starts replay mode at named moments and takes headless Google Chrome screenshots (Chrome from `/Applications`) at 1920x1080, 1440x900 and 1280x720. |
| `Dockerfile` | `ros:jazzy-ros-base` plus a venv at `/opt/hud-venv` with FastAPI, uvicorn, websockets and Pillow, installed from `requirements.lock` with `--require-hashes`. Code is baked in, so rebuild (`run_hud.sh --build`) after changing `hud/`. |
| `requirements.txt`, `requirements.lock` | `requirements.txt` holds the accepted ranges for `fastapi`, `uvicorn`, `websockets` and `pillow` and is only the input to the lock. `requirements.lock` pins every package with its hashes; the image and the laptop instructions install it with `--require-hashes`, and CI audits it and tests the same versions from `tests/requirements-ci.lock`. No numpy; `rclpy` comes from the ROS image. |
| `README.md` | Detailed HUD documentation: panels, purge hold, topic and QoS contract, event conventions, HTTP/WS API, audit logs, troubleshooting. |
| `static/index.html` | Page skeleton: top bar (alarm strip, incident summary, SYNTHETIC DATA / REPLAY badge, connection status), three columns, footer disclosures, alarm overlay. |
| `static/app.js` | Browser logic (plain JS, no build step, no CDN, works offline over the tunnel). WebSocket with polling fallback; marks data older than 3 s as STALE; inserts all event and model text as text, never as HTML. |
| `static/style.css` | Dark control-room palette so red alarms stand out; scales to 1440x900, 1280x720 or a DCV window; state is never shown by colour alone. |

**Flags** (`python3 hud/server.py ...`):

| Flag | Default | Meaning |
|---|---|---|
| (none) | - | Live ROS mode (the container default). |
| `--fake`, `--fake-speed N` | off, 1.0 | Synthetic 45 s scenario. |
| `--replay FILE` (repeatable) | - | Replay a mission `.jsonl`, optionally plus its HUD audit `.csv`. |
| `--replay-at T` / `--replay-from T` / `--replay-speed N` | - / - / 1.0 | Jump to sim time T and hold / jump and play / speed. |
| `--replay-speech-filter` | off | Pass replayed agent lines through the agent's speech filter (repo checkout only). |
| `--host`, `--port` | `127.0.0.1`, `8080` | Bind address. |
| `--namespace` | `/demo_spot` | Robot namespace. |
| `--camera-topic`, `--camera-fps`, `--camera-width` | `camera/frontleft/image`, 5, 800 | Camera tile source and limits. |
| `--site-defaults`, `--site-timeout` | `agent/site_defaults.json`, 10 s | Site fallback. |
| `--log-level` | `info` | Server log level. |

---

### `host/` — scripts that run on the EC2 host

**What it is for.** Starting and stopping the three long-lived containers
(`spot-sim`, `spot-hud`, `spot-agent` or `spot-mission`), the presenter
console (stage gas, reset, change camera), the headless smoke test, the
end-to-end ROS 2 test, and the GUI operator notes.

**Where it runs.** On the EC2 host from `/opt/spot-demo/host` (deployed by
`scripts/deploy-to-host.sh`), as `ubuntu`: in an SSM shell opened as your own
OS account (Session Manager Run As, [`docs/iam/`](docs/iam/README.md)) and then
`sudo -iu ubuntu`, or the DCV desktop terminal, where the agent and the scripted
mission refuse to start without `--unattributed-run` (DCV logs in as `ubuntu`, so
the record could not name you). All containers use `--network=host`
and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.

| File | What it does |
|---|---|
| `run_sim.sh` | Starts the Isaac Sim container `spot-sim` (GPU, host network, image pinned by digest), mounting the warm Isaac caches from `/opt/isaac-cache` and only `sim/` (read-only). It refuses to start unless you have set `ACCEPT_EULA=Y` yourself after reading the NVIDIA licence; NVIDIA data collection stays off unless you set `PRIVACY_CONSENT=Y`. Raises the kernel's UDP receive buffers to 8 MiB (runtime only) so camera frames are not dropped. Its own flags: `--background` (detached; writes `logs/sim-<run_id>.log`; returns only once this launch prints `FACADE-READY`, then fails if anything listens off loopback that should not), `--gui` (presentation mode on the DCV desktop; refuses while DCV listens off loopback), `--editor` (full Isaac editor), `--status` (is the running sim ready?), `--stop`, `--force`, and the opt-in, once-per-host `--prepare-desktop` (GDM autologin for `ubuntu`, screen lock, idle blanking, banners and crash pop-ups off; it prints each change and records the originals first) with `--restore-desktop` to undo it. Only `sim/isaac_app.py`'s own options are passed through; anything else is refused before a running sim is touched. Launching or `--stop` refuses while the agent or a mission is running, unless `--force`. |
| `sim_lib.sh` | Shared helpers sourced by the other scripts: new run ids, per-run log files with a `*-latest.log` symlink, and a readiness check that reads `docker logs spot-sim` of the current container (so a stale log line can never count as ready). |
| `run_hud.sh` | Starts the HUD container `spot-hud` (builds the image if missing or with `--build`), mounts `agent/` read-only for the site fallback, and waits up to 30 s for `/api/state`. Warns (does not fail) if no ready sim is running. `--stop` removes it. |
| `run_agent.sh` | Runs the operator agent in `spot-agent`: REPL, `--once "..."`, `--dry-run`, `--log-path ...`. With `--mission` it runs `run_mission.py` instead (container `spot-mission`). Refuses to start unless the running sim has printed `FACADE-READY` (skip with `SPOT_SKIP_SIM_CHECK=1`). Mounts `/opt/spot-demo/logs` at `/workspace/logs`. Forwards `SPOT_DEMO_PRESENTER`. Passes no AWS keys. `--build` rebuilds the image first. |
| `inject_gas.sh` | Presenter console for the simulated atmosphere. `bash inject_gas.sh` stages an 80 ppm H2S pocket (1 m radius, 0.3 m high) at Spot's current position. `bash inject_gas.sh <h2s\|lel\|o2\|co> AMOUNT [RADIUS_M] [Z_M]` for other gases; `clear` removes every injected pocket; `vent on [tau_s]`, `vent off`, `vent reset` control ventilation. Every action also publishes a disclosure event so the HUD and log mark it as staged. The reaction comes from the SafetyGuard, not from this script. |
| `reset_spot.sh` | Resets between runs without restarting the sim: clears injected gas, optionally un-ventilates the tank (`--reset-ventilation`, alias `--raw`), recharges the simulated battery, puts the camera back on `auto`, and calls `self_right`, which in this facade teleports Spot to spawn, standing. The HUD starts a fresh run. |
| `set_view.sh` | GUI only. `bash set_view.sh auto\|chase\|interior\|overhead\|door` switches the presenter camera; `bash set_view.sh --ui editor\|presentation` switches the layout without a restart. |
| `idle_stop.sh` | Opt-in idle stop (`sudo bash idle_stop.sh apply [--minutes N]`, `remove`, `status`): a systemd timer that powers the host off, which stops it, after an idle window (default 60 minutes) with no Session Manager session, demo container, agent or mission process or open mission; it never powers off when it cannot tell. See the RUNBOOK, "Once per host". |
| `add_operator.sh` | One OS account per operator for Session Manager Run As (`sudo bash add_operator.sh NAME`, `remove NAME`, `status NAME`): no password, and a sudoers rule, checked with `visudo`, that allows only `sudo -iu ubuntu`, so the agent records the operator's own login. Refuses names that are not plain user names, the shared accounts and accounts it did not create. |
| `smoke_test.sh` | Headless smoke test: runs the sim with `--headless true --smoke --no-cameras` under a timeout (default 1200 s) and passes only if this run printed `SMOKE-OK`. Runs on the test DDS domain (`SPOT_TEST_DOMAIN_ID`, default 43), not the demo's, and afterwards removes `spot-sim` if it is still this run's. |
| `e2e_ros_test.sh` | End-to-end ROS 2 test: starts a headless sim, waits for readiness, then runs `e2e_client.py` in a ROS container, both on the test DDS domain (default 43). Exit code is the client's. Stops any running sim, and at exit removes its own sim only. |
| `e2e_client.py` | The e2e checks, driving the facade the way the agent does: discovery, real-time factor, claim/power/stand, first stand-up drift, a `cmd_vel` walk, the watchdog, gas baseline and injected alarm, e-stop and recovery, three sit/stand/walk cycles, ventilation fields, gas clear, the probe rules in `payload/site`, a walk to the door, probe accept/reject cases, ventilation and reset. Prints `E2E OK\|FAILED: passed/total`. |
| `gui_notes.md` | Operator notes for GUI mode on the DCV desktop: what `--gui` changes and why, presentation mode, camera views, lighting, battery modes, how to check the screen without seeing it, a performance table and troubleshooting. |

---

### `scripts/` — scripts that run on the Mac

**What it is for.** Reaching the EC2 host only through AWS Systems Manager:
no SSH, no S3, no inbound ports.

**Where it runs.** On the presenter's or developer's Mac. Needs the AWS CLI
with credentials for the sim account and the Session Manager plugin (for
the tunnels). Commands sent with Run Command execute on the host as root.

| File | What it does |
|---|---|
| `connect-dcv.sh` | Port-forwards the host's NICE DCV port 8443 to `https://localhost:8443` (self-signed certificate). Log in as `ubuntu`. It prints the command that reads the password from your Secrets Manager secret (`SPOT_DCV_SECRET_ID`, default `spot-demo/dcv-password`); it does not fetch it for you. Needs `SPOT_SIM_INSTANCE`. |
| `connect-hud.sh` | Port-forwards the host's HUD port 8080 to `http://localhost:8080` (plain HTTP inside the encrypted tunnel). Needs `SPOT_SIM_INSTANCE`; `LOCAL_PORT` and `REMOTE_PORT` override the ports. |
| `deploy-to-host.sh` | Ships `sim/ host/ agent/ hud/ tests/ tools/` at the current commit (`git archive` of HEAD; it refuses uncommitted changes unless you pass `--allow-dirty`) to `/opt/spot-demo` on the host over SSM Run Command: base64-encodes and sends in chunks to a fresh private staging directory, verifies size and SHA-256, replaces all six directories, writes the commit to `/opt/spot-demo/VERSION`, and sets ownership to `ubuntu`. Needs `SPOT_SIM_INSTANCE`; `SPOT_DEMO_DIR` must be a plain path at least two levels deep and not under a system directory. |
| `fetch-log.sh` | The reverse: pulls `/opt/spot-demo/logs/<PATTERN>` from the host into `~/spot-demo-logs/<instance id>/` on the Mac, outside the checkout so operator data cannot be committed (override with `FETCH_DEST`; default pattern `*.jsonl`; `.log`, `.csv`, `.json`, `.txt`, `.png` also allowed; one optional subfolder such as `screens/*.png`, flattened on arrival), with size and SHA-256 checks. |
| `start-host.sh` | Starts the instance and waits for its status checks; on `InsufficientInstanceCapacity` it retries with a doubling wait (6 attempts by default), then prints the Capacity Reservation command for the instance's type and Availability Zone. Needs `SPOT_SIM_INSTANCE`. |
| `ssmrun.sh` | `scripts/ssmrun.sh <script-file> [timeout-seconds]`: runs a local script on the host as root and prints the status and the last 40 lines of output. |

---

### `tools/` — rehearsal and evidence helpers

**What it is for.** Recording a run, capturing screenshots and HUD
snapshots at storyline moments, driving one scripted rehearsal rep, and
summarizing a run into one line of numbers. These produced the files and
numbers under `docs/evidence/` and in the runbook.

**Where it runs.** Mostly on the EC2 host from `/opt/spot-demo/tools`.
`rh_analyze.py` also runs on a Mac.

| File | What it does |
|---|---|
| `odom_rec.py` | `python3 odom_rec.py OUT.jsonl [duration_s]`: records odometry (up to 10 Hz) and gas readings to JSONL. Needs `rclpy`, so run it in a `ros:jazzy-ros-base` container. |
| `rh_analyze.py` | `python3 tools/rh_analyze.py MISSION.jsonl ODOM.jsonl [LABEL]`: one JSON line per run with outcome, test results, alarm and retreat latencies, path length, minimum door clearance (Spot's body box against the door jambs), minimum body height and final position. |
| `hud_watch.py` | Tails a mission log and saves the HUD's `/api/state` about 1 s after key moments (fail, ventilation, authorized, incident, after the incident). |
| `screen_watch.py` | Tails a mission log and calls `snap_screens.sh` at storyline moments (probe, fail, purge wait, re-test, authorized, inside, mid-tank, alarm, retreat, completed). Has unit tests. |
| `snap_screens.sh` | `bash tools/snap_screens.sh NAME`: captures the Isaac Sim window on the DCV desktop (ImageMagick; the whole desktop only with `SPOT_SNAP_ROOT=1`) and the HUD (headless Chromium in a Playwright container) to `/opt/spot-demo/logs/screens/`. GUI mode only. The HUD shot lands about 3 s after the desktop shot. |
| `rehearse_rep.sh` | `bash /opt/spot-demo/tools/rehearse_rep.sh incident\|clean LABEL` (as root, sim and HUD already up): resets, starts the odometry recorder, runs the agent REPL on a pseudo-terminal and types the cheat-sheet prompts. In incident mode the H2S spike must come from a second shell running `inject_gas.sh`. |

---

### `tests/` — unit tests

**What it is for.** Fast checks of the logic that does not need a GPU: 20
`unittest` modules with 946 test methods. They use in-memory fakes (a
kinematic fake robot, a sped-up clock, fake ROS modules, a fake AWS CLI)
instead of ROS 2, Isaac Sim or Bedrock.

**Where it runs.** On a Mac or Linux from the repo root, with the packages of
`tests/requirements-ci.lock` (installed with hashes as in
[CONTRIBUTING.md](CONTRIBUTING.md#running-the-checks-locally)), plus bash and
`shasum`/`sha256sum`. Without `strands-agents`, 2 tests skip. Two files are also
runtime dependencies: `agent/Dockerfile` copies `test_agent_tools.py` into
the agent image, and `spot_agent.py --dry-run` imports the fake robot from
`test_mission.py`.

| File | Tests | Covers |
|---|---|---|
| `test_agent_tools.py` | 111 | The agent's tools: permit gate, movement rules, mission tools, e-stop confirmation, presenter gas gating, failure handling, the speech filter (no "safe", no tool names aloud), REPL console, system prompt content, CLI defaults. |
| `test_hud.py` | 112 | HUD server and bridge: every HTTP route and the WebSocket, permit checklist, ventilation, probe table, incident banner, audit exports, frontend source checks, the ROS subscription set and QoS, replay mode on `docs/evidence/`, and `scripts/fetch-log.sh` against a fake AWS CLI. |
| `test_mission.py` | 93 | Permit state machine, SafetyGuard, purge gate, robot client offline behaviour and `run_mission.py`, using a fake robot with the watchdog, lag, tank collisions, a 4 Hz gas feed and the probe rules. Includes the rehearsal regressions. |
| `test_gas_field.py` | 58 | Gas model: pockets, injected spikes, OSHA alarm boundaries, determinism, ventilation decay, non-numeric fail-safe, probe reach and door occlusion, door size matching `build_scene.py`. |
| `test_sensors.py` | 46 | Camera parsing, frame skip, camera and light aim, light fade, chase-camera clamp, `CameraDirector`, presentation UI settings, the view command parser, and `run_sim.sh`'s layout helper against a stub. |
| `test_battery.py` | 8 | Battery modes; `demo` never drops below 30 %. |
| `test_stance_hold.py` | 7 | Heading hold: pass-through, latch, dead band, capped correction. |
| `test_screen_watch.py` | 3 | Screenshot moment triggers. |
| `test_hardening_agent.py` | 129 | The agent and mission controls added for the threat model: the simulator-only gate and backend identity, operator-console confirmations, the door re-test, the guard heartbeat, stale-gas holds and retreat retries, the audit-log chain and identity, the session model budget. |
| `test_hardening_host.py` | 99 | The host scripts against fake Docker, `ss`, `iptables` and X tools: DDS isolation, container users and capabilities, the IMDS guard and the launch check, listener checks, the desktop opt-in, pinned images, the test domain, window-only captures. |
| `test_hardening_sim.py` | 65 | The simulator facade's bounds, rate limits and records, the real-time factor, and the refusal of unknown Kit arguments, without Isaac Sim. |
| `test_hardening_hud.py` | 54 | The HUD's Host and Origin allowlists and caps, source provenance, undisclosed-change flags, flood behaviour, the guard heartbeat, camera-frame caps and the scanner fixes. |
| `test_security_controls.py` | 30 | The controls the threat model recorded at its baseline, kept so that a change cannot quietly remove them. |
| `test_hardening_docs.py` | 19 | The IAM policy documents, the release files, the README notice, the kernel-header purge, the third-party licence list and the CI safeguards. |
| `test_hardening_scripts.py` | 14 | The Mac-side scripts against a fake AWS CLI: deploy provenance and paths, fetch destinations, SSM request encoding. |
| `test_threshold_copies.py` | 12 | Every copy of the OSHA thresholds and of the site layout equals the simulator's. |
| `test_hardening_audit.py` | 43 | The audit trail's attribution and its anchor off the host: the Session Manager preferences and session-log template, the instance role's session-log statements, `host/add_operator.sh`, the refusal to start a shared-account run, the chain-head console lines and `mission.py --verify`. |
| `test_hardening_guardrail.py` | 14 | The Amazon Bedrock guardrail the agent applies: the CloudFormation template's four policies, its immutable version, the always-created intervention alarm and dashboard (whose body is parsed as JSON) and its outputs; the `bedrock:ApplyGuardrail` statement in the shipped instance policy; the host wrapper's pass-through; and the documents that say it is on by default. |
| `test_hardening_cost.py` | 36 | The cost guardrails: the CloudFormation template, `host/idle_stop.sh` against a fake systemd, Docker and `pgrep`, `scripts/start-host.sh`'s capacity retries against a fake AWS CLI, and the setup, capacity and Cleanup documentation. |
| `test_check_citations.py` | 6 | The threat model's citation checker. |
| `test_process_calls.py` | 4 | Every process the repository's Python starts is named in the source: a literal program, a literal script for Python and literal inline code for `sh -c`, no shell and no `os.system`; computed values are arguments only. |

Not covered by unit tests: `sim/isaac_app.py` and `sim/build_scene.py`
(they need Isaac Sim; the host smoke and e2e tests exercise them).

---

### `docs/` — documentation

**What it is for.** The written record of the project. Nothing here runs.

**Where it runs.** Read anywhere. The runbook's commands are split between
the Mac (`aws`, `scripts/*.sh`) and the host (`host/*.sh`).

| File | What it covers |
|---|---|
| `design.md` | Why Isaac Sim 6.1 on EC2 was chosen over a MuJoCo stack (it ships a Spot walking policy, the Spot model, sensors and a ROS 2 bridge), the original architecture plan, the regulator-grade scenario with OSHA thresholds, capability scope, the host inventory (instance, security group, IAM role, image sizes) and the "Next" list. |
| `RUNBOOK.md` | The operator procedure, steps 0-9: Mac setup, start the instance, deploy, open a host shell, start the sim (headless or GUI), start the HUD, drive the storyline (agent REPL, scripted fallback, scripted excursion), presenter console, between runs, stop. Then measured rehearsal tables, the presenter cheat sheet, the leadership rehearsal results, fixes found in rehearsal and 12 known limitations. |
| `LEADERSHIP-BRIEF.md` | One-page presenter script for the 8-minute leadership demo: clock, beat, what to type, what to say; the honest caveats to say out loud; the never-say list. |
| `capability-matrix.md` | About 40 Spot capabilities: the real `spot_ros2` interface name, how the Isaac facade would provide it, difficulty and whether the demo needs it. Plus findings about related open-source projects. |
| `facade-contract-v1.md` | `spot_ros2` names copied from its source with file and line references: namespace and frame prefix, `cmd_vel` and `body_pose`, the 0.125 s command duration, odometry and joint topics, Trigger services, leg joint order, and the driver's lease, power and e-stop behaviour. |
| `isaac-api-notes.md` | Isaac Sim 6.1.0 API notes checked against the container: bundled ROS 2 distributions, the Spot policy runner, the ROS 2 bridge and `rclpy` patterns, the asset root, the camera helper pattern, and gotchas. |
| `security/THREAT_MODEL.md` | The [threat model](docs/security/THREAT_MODEL.md): scope, data flows and trust boundaries, OWASP and STRIDE analysis, an STPA safety view, the threat and mitigation registers, and the pre-publication review checklist. |

---

### `docs/evidence/` — the proof behind the numbers

**What it is for.** Screenshots, transcripts and logs from the presenter
dry run on 2026-09-27 (GUI mode, agent REPL on Bedrock, spike fired from a
second shell) and the leadership rehearsal (`v5/`). Three dry-run sessions:
`inc1` (before a probe-seating fix; needed a fallback prompt), `inc2` (the
cheat-sheet storyline with a staged spike, ABORTED as designed) and `ok1`
(early re-tests, purge gate, entry, exit, COMPLETED).

**Where it runs.** Produced on the host with the `tools/` helpers and pulled
to the Mac with `scripts/fetch-log.sh`. Read by `tests/test_hud.py` and by
the HUD replay mode. **Do not rename `events-{ok1,inc2}.jsonl` or
`hud-audit-{ok1,inc2}.csv`: the HUD replay tests open them by name.**
(`tests/test_agent_tools.py` also quotes replies from
`transcript-inc2-incident.txt` verbatim, but does not read the file.)

| File | What it is |
|---|---|
| `README.md` | Index and narrative: what was run, measured numbers, fixes found, and a per-file table of what each image shows. |
| `01-...png` to `14-...png` | 14 screenshots: ready, door probe, test FAIL, purge incomplete, inside, alarm onset, far end, retreat, retreat complete, permit COMPLETED, and the 1080p layout fix. Desktop shots are the whole DCV desktop; HUD shots are headless Chromium at 1920x1080. |
| `transcript-{inc1-before-fix,inc2-incident,ok1-completed}.txt` | REPL transcripts: operator prompts, tool calls and results, model answers. |
| `events-{inc1,inc2,ok1}.jsonl` | Mission audit logs, one JSON record per event (sim time, wall time, permit, source, kind, message, data). HUD replay fixtures. |
| `hud-audit-{inc1,inc2,ok1}.csv` | HUD audit exports (recorded before the hardening pass, in its older column layout), including the presenter's staged-spike disclosures. |
| `screen-moments-{inc1,inc2,ok1}.txt` | Which log record triggered each screenshot. |
| `rh-analyze.txt` | `tools/rh_analyze.py` output per session (path length, door clearance, ventilation wait, alarm-to-retreat-start time, retreat time, body height, final pose). |
| `perf-gui.txt` | Every PERF, VIEWPORT, CAMERAS and FACADE-READY line from both GUI sim launches. |
| `v5/lr-01...lr-09-*-{sim,hud}.png` | Leadership rehearsal frames (17 files; `lr-05` has only the sim frame): ready, door probe FAIL, PURGE HOLD, re-test authorized, past the door, incident banner, retreat complete, mid-tank, clean COMPLETED. |
| `v5/lr-reps-summary.jsonl` | One line per final rehearsal session: turn times, test results, purge-hold percentage, alarm and retreat latencies, whether the model said "safe". |
| `v5/lr7-rep1-*`, `v5/lr8-clean-*` | Audit log and transcript of one incident rep and of the clean COMPLETED run. |
| `v5/sim-{before,after}-*.png`, `v5/sim-ui-editor-toggle.png` | Earlier frames: editor layout versus presentation mode. |

## The demo storyline

The flow is modelled on OSHA 29 CFR 1910.146 (permit-required confined
spaces; Appendix E atmospheric limits) and the tank-entry practice described
in API RP 2016. It is a demonstration, not a compliance claim.

| # | Beat | What happens |
|---|---|---|
| 0 | Reset | `reset_spot.sh --reset-ventilation`: Spot stands at spawn (14, 0) facing the tank; the tank is unventilated; the HUD shows NOT STARTED. |
| 1 | Permit | The operator asks for permit CSE-0427 with attendant, isolation, lockout-tagout and rescue plan. The fields are recorded (not verified). Until they are complete, code refuses any approach within tank radius + 1.5 m. |
| 2 | Pre-entry test fails | Spot stops outside the 1.30 x 1.30 m cleanout door and reaches in with the simulated extendable gas probe: 0.5 m and 1.0 m inside the door, each at 0.2, 0.6 and 1.0 m high, in the order O2, then LEL, then H2S/CO. Each gas is judged on its worst point against the OSHA limits. Unventilated, H2S reads about 51-52 ppm at 0.2 m: FAIL, entry denied. |
| 3 | Forced ventilation and purge hold | Simulated forced ventilation starts (contamination decays with a 15 s time constant toward a 4 % residual). If the operator asks for an early re-test, the door readings may already be within limits, but entry is still held until the purge target (8 % remaining) is met: the HUD shows an amber PURGE HOLD and the permit stays PRE_ENTRY_TESTING. |
| 4 | Re-test passes | At or below 8 % remaining, the re-test passes (H2S about 4 ppm): ENTRY_AUTHORIZED. In the agent, an authorization is only valid for 600 s and only if nothing has invalidated it. |
| 5 | Entry | Spot walks through the door and along 5 fixed interior waypoints around the pipe runs, reading the air the whole way. The GUI camera cuts inside the tank. |
| 6 | Staged H2S excursion | The presenter runs `bash inject_gas.sh` in a second shell when the HUD map shows Spot at waypoint 1. The pocket is announced on the HUD and in the log as staged. |
| 7 | SafetyGuard retreat | The deterministic SafetyGuard fires on the alarm, stops Spot, takes over motion, aborts the permit and backs Spot out along the path it came in, without turning in place near the door. |
| 8 | Agent explains | The incident is attached to the agent's tool result; the model explains what happened in plain words: staged spike, reading, retreat, permit aborted, re-entry needs a new permit and a new test. |
| 9 | Reset | `reset_spot.sh --reset-ventilation` for the next run. For a calm ending instead of beats 6-8, the operator says "Walk back out to the entry point." and the permit ends COMPLETED. |

**What is deterministic code and what is the language model**

| Decision | Who makes it | Where |
|---|---|---|
| Permit fields complete before approaching the tank | Code | `agent/mission.py` |
| Pass or fail of each gas against the OSHA limits | Code | `agent/mission.py`, `sim/gas_field.py` |
| Purge gate (8 % contamination remaining) | Code | `agent/mission.py` |
| Entry needs a fresh (600 s), passing, agent-run test | Code | `agent/spot_agent.py` |
| Route: named places only in the chain spawn, entry, door, waypoint_1..N, one segment at a time; free walks may not enter or cross the tank | Code | `agent/spot_agent.py`, `agent/mission.py` |
| Motion refused while a gas alarm or retreat is active | Code | `agent/spot_agent.py` |
| SafetyGuard: 20 Hz watchdog, armed while entry is authorized or under way, whenever Spot is inside, and whenever the pose is unknown; fires on any exceedance, on the sim's alarm flag, or after 5 s without gas data | Code | `agent/mission.py` |
| Velocity limits, motion ownership during a retreat, stale-data rejection | Code | `agent/robot_client.py` |
| E-stop requires explicit confirmation; staged-gas tools need `SPOT_DEMO_PRESENTER=1` | Code | `agent/spot_agent.py` |
| Rewriting "safe" or "all clear" and spoken tool names out of replies | Code | `agent/spot_agent.py` (`sanitize_speech`) |
| Which tool to call next, in what order, and what to say | Language model | Bedrock `us.anthropic.claude-sonnet-5`, guided by `agent/prompts.py` |

The model proposes and explains; code enforces and cannot be overridden by
the model. `run_mission.py --storyline` runs the same sequence with no model
at all.

## Quick start

For the presenter, from the Mac. This is the short version; the full
procedure, timings and fallbacks are in [`docs/RUNBOOK.md`](docs/RUNBOOK.md).

**Once:** install the AWS CLI v2 and the Session Manager plugin, and get
credentials for the sim account.

```bash
# 0. Mac: point the scripts at the sim host
export SPOT_SIM_INSTANCE=<instance-id> AWS_REGION=us-west-2

# 1. Mac: start the instance (and stop it again in step 11: it bills while running)
aws ec2 start-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE
aws ec2 wait instance-status-ok --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE

# 2. Mac: deploy, only if the repo changed (~1 min, SHA-256 checked)
scripts/deploy-to-host.sh

# 3. Mac, terminal 1: DCV desktop tunnel. It prints how to read the password.
scripts/connect-dcv.sh              # browse https://localhost:8443, user ubuntu

# 4. Mac, terminal 2: host shell A, as your own OS account (Run As; set up once per
#    account in docs/iam/README.md). The agent refuses a shared login such as ssm-user.
aws ssm start-session --region us-west-2 --target $SPOT_SIM_INSTANCE
sudo -iu ubuntu
cd /opt/spot-demo/host

#    Host, shell A, once per host: keep DCV on loopback, prepare the console desktop
#    (both are opt-in and reversible; see docs/RUNBOOK.md, step 0)
sudo bash configure_dcv.sh apply --restart
bash run_sim.sh --prepare-desktop
#    Host, shell A, after every boot: only root and the agent reach the instance
#    metadata service (every launch below refuses until this is in place)
sudo bash imds_guard.sh apply
#    Host, shell A, once per host, optional: power off after 60 idle minutes (check the
#    shutdown behaviour first, and deploy the cost guardrails: docs/RUNBOOK.md, step 0)
sudo bash idle_stop.sh apply

# 5. Host, shell A: if agent/ changed in step 2, rebuild the agent image
docker build -f /opt/spot-demo/agent/Dockerfile -t spot-agent:latest /opt/spot-demo

# 6. Host, shell A: start the sim in GUI presentation mode (READY in ~40-65 s warm).
#    Read the NVIDIA Isaac Sim licence first: ACCEPT_EULA=Y is your acceptance.
export ACCEPT_EULA=Y
bash run_sim.sh --background --gui
bash run_sim.sh --status            # READY / NOT READY / NOT RUNNING
bash set_view.sh auto               # optional: auto | chase | interior | overhead | door

# 7. Host, shell A: start the HUD before the agent (add --build if hud/ changed in step 2)
bash run_hud.sh
#    Mac, terminal 3:
scripts/connect-hud.sh              # browse http://localhost:8080, F11 for full screen

# 8. Host, shell A: reset, then start the agent REPL
bash reset_spot.sh --reset-ventilation
bash run_agent.sh --log-path /workspace/logs/demo-$(date -u +%Y%m%dT%H%M%S).jsonl
```

At the `operator>` prompt, type the four cheat-sheet prompts:

```text
Prepare permit CSE-0427: the attendant is at the entry, isolation and lockout-tagout are in place, rescue plan: on-site standby team. Then run the pre-entry atmospheric test at the tank.
Start ventilation, wait 30 seconds, then re-test. Don't wait for the purge.
Wait for the purge target, then re-test.
Proceed with the confined space entry.
```

The console asks you to confirm the permit attestations after the first
prompt and the entry after the last: type `y`. Skip the third prompt if the
HUD already shows ENTRY AUTHORIZED. Type `quit` to leave the REPL once Spot
is out; `/stop`, `/retreat` and `/status` work at any time.

```bash
# 9. Host, shell B (a second SSM session, same as step 4): the presenter console
bash /opt/spot-demo/host/inject_gas.sh          # stage the H2S spike when the HUD shows waypoint 1
bash /opt/spot-demo/host/inject_gas.sh clear    # remove injected gas

# 10. Between runs (either shell)
bash reset_spot.sh --reset-ventilation

# 11. Stop everything and stop paying for the GPU
bash run_hud.sh --stop; docker stop spot-agent spot-mission 2>/dev/null; bash run_sim.sh --stop   # docker stop, not rm -f: the agent brings Spot out first
exit                                                                          # leave the host
aws ec2 stop-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE   # Mac
```

**If Bedrock is slow or unavailable**, run the same story with no model:

```bash
bash run_agent.sh --mission --storyline --log /workspace/logs/story-$(date -u +%Y%m%dT%H%M%S).jsonl
```

**Try the HUD without AWS** (Mac, from the repo root):

```bash
python3 -m venv .venv-hud    # the same hash-locked packages as the HUD image
.venv-hud/bin/pip install --require-hashes --no-deps --only-binary :all: -r hud/requirements.lock
.venv-hud/bin/python hud/server.py --fake       # synthetic scenario at http://127.0.0.1:8080
.venv-hud/bin/python hud/server.py --replay docs/evidence/events-ok1.jsonl \
  --replay docs/evidence/hud-audit-ok1.csv --replay-at 450   # a recorded run
```

**Pull the logs back** to `~/spot-demo-logs/<instance id>/` on the Mac:

```bash
scripts/fetch-log.sh 'demo-2026*.jsonl'
```

## ROS 2 interface

The sim publishes everything under the namespace `/demo_spot` from the node
`spot_facade`. DDS runs over the host network on `ROS_DOMAIN_ID` 42 (override
with `ROS_DOMAIN_ID`; the smoke and end-to-end tests use 43, `SPOT_TEST_DOMAIN_ID`), with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and a
Fast DDS profile that binds every socket to 127.0.0.1 and sends discovery only
to 127.0.0.1, so no other machine can join the graph.
`ROS_AUTOMATIC_DISCOVERY_RANGE` is not set: `LOCALHOST` alone leaves the
sockets on every interface (a peer that names the host as a static peer can
join), and on top of the profile it opens two send sockets off loopback. All `payload/*` messages are
`std_msgs/String` carrying JSON. The full contract is in
[`hud/README.md`](hud/README.md) and [`docs/facade-contract-v1.md`](docs/facade-contract-v1.md).

**`spot_ros2`-style names** (the part the real driver also has):

| Name | Type | Sim direction | Notes |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | subscribes | `linear.x`, `linear.y`, `angular.z` only. Implicitly claims and powers on (like `spot_ros2` defaults). 0.125 s watchdog: re-send at 8 Hz or more. |
| `body_pose` | `geometry_msgs/Pose` | subscribes | Stub: logged, not applied. |
| `odometry` | `nav_msgs/Odometry` | publishes, 50 Hz | Frame `odom` to `demo_spot/body`. Simulator ground truth, not an estimate. |
| `joint_states` | `sensor_msgs/JointState` | publishes, 50 Hz | The 12 `spot_ros2` leg joint names. |
| `/tf` (global) | `tf2_msgs/TFMessage` | publishes | `odom` to `demo_spot/body` only. |
| `camera/<name>/image`, `camera/<name>/camera_info` | `sensor_msgs/Image` (rgb8, 640x480), `CameraInfo` | publishes, 10 Hz | Default cameras `frontleft`, `frontright`. The HUD shows `frontleft`. |
| Services `claim`, `release`, `power_on`, `power_off`, `stand`, `sit`, `stop`, `self_right`, `estop/hard`, `estop/gentle`, `estop/release` | `std_srvs/Trigger` | serves | `self_right` here teleports Spot to spawn, standing (used by `reset_spot.sh`). |

**Demo-only `payload/*` topics** (no `spot_ros2` equivalent):

| Name | Sim direction | Rate / QoS | Content |
|---|---|---|---|
| `payload/status` | publishes | 2 Hz | `claimed`, `powered`, `standing`, `standing_up`, `estopped`, `battery_pct`, `sim_time`, `simulated: true` (the backend identity the agent requires), `rtf`, input counts and `cmd_vel_publishers` |
| `payload/gas` | publishes | 4 Hz, and after any gas command | O2 %, LEL %, H2S ppm, CO ppm at the body, alarms, ventilation state, injected pockets |
| `payload/gas/probe` / `payload/gas/probe_result` | subscribes / publishes | one reply per request | Request `{id, points: [[x,y,z], ...]}` (1-16 points); reply with one reading per point, or an error if any point breaks the probe rules. Marked `simulated: true`. |
| `payload/gas/inject` | subscribes | - | `{gas, ppm, x, y, z, radius}`: stage a pocket (presenter). |
| `payload/gas/clear` | subscribes | - | Remove all injected pockets. |
| `payload/gas/ventilation` | subscribes | - | `{on, tau_s}` or `{reset: true}`. |
| `payload/site` | publishes once | latched (RELIABLE + TRANSIENT_LOCAL, depth 1) | Spawn, entry, door, waypoints, tank centre and radius, and the probe rules (1.1 m reach from the body front, 0-3 m height, line of sight). Subscribers must also ask for TRANSIENT_LOCAL. |
| `payload/view` | subscribes | - | GUI camera `{camera: ...}` and layout `{ui: ...}`. |
| `payload/battery/reset` | subscribes | - | Battery back to 100 %. |
| `payload/events` | publishes its own records (source `facade`) | RELIABLE + TRANSIENT_LOCAL | Audit events `{t, wall_t, source, kind, message, data}` published by the agent and mission (`mission`, `agent`, `guard`), the simulator's facade (every world control applied or refused) and the presenter scripts (`presenter`); read by the HUD. |

## Testing

**On a Mac (no ROS, Isaac Sim or Bedrock needed):**

```bash
python3 -m unittest discover -s tests          # all 21 modules, 965 tests
python3 -m unittest tests.test_hud -v          # one module
python3 -m unittest tests.test_battery tests.test_gas_field tests.test_sensors tests.test_stance_hold   # sim logic only
```

Most recent full local run (2026-11-13, Python 3.10, macOS): 965 tests in
about 500 s, all passing, 2 skipped (`strands-agents` not installed). On a
slow or shared machine set `SPOT_TEST_SLOWDOWN=2` (CI does) to slow the
mission tests' scaled clock.

**On the EC2 host (needs the GPU and the Isaac container):**

| Test | Command | What it proves |
|---|---|---|
| Smoke | `bash /opt/spot-demo/host/smoke_test.sh` | The sim boots headless, claims, powers on, stands and steps physics about 600 times, then prints `SMOKE-OK`. |
| End to end | `bash /opt/spot-demo/host/e2e_ros_test.sh` (~2.5 min, stops any running sim) | The ROS 2 facade behaves the way the agent expects: services, walking, watchdog, e-stop and recovery, re-stand cycles, gas alarms, ventilation, the probe rules. Last result in the runbook: `E2E OK: 44/44` (2 runs). |

What no test covers yet: the real `spot_ros2` driver or hardware, and a
streaming DCV viewer during the GUI timing runs.

## Verified results

All numbers come from simulation runs on the one EC2 host. They describe how
the demo behaves, not field performance. The real-time factor (RTF) is sim
time divided by wall time, so 1.00 means the sim keeps up with real time.

| Result | Number | Source |
|---|---|---|
| Scripted storyline | 5/5 COMPLETED; 145.95-146.85 s; path 36.1-36.3 m; minimum body-to-door-jamb clearance 0.209-0.245 m | `docs/RUNBOOK.md` |
| Scripted H2S excursion | 3/3 ABORTED as designed (exit 2); injection to guard alarm 33-117 ms; retreat 23.8-24.4 s | `docs/RUNBOOK.md` |
| Unventilated first test | FAIL on H2S 51.3-51.8 ppm at 0.2 m height (O2, LEL, CO pass) | `docs/RUNBOOK.md`, `docs/evidence/events-*.jsonl` |
| Leadership rehearsal, 5 incident reps | PURGE HOLD at 11.0-11.4 % remaining; alarm at H2S 77.4-78.8 ppm; stop command 0.1 ms after detection; alarm to retreat start 12-23 ms; retreat 11.31-14.86 s; the model explained the spike as staged 5/5 | `docs/evidence/v5/lr-reps-summary.jsonl` |
| Leadership rehearsal, all 7 sessions (5 incident reps, 2 clean runs) | The model never said "safe"; 0 probe rejections; both clean runs ended COMPLETED through all 5 waypoints | `docs/evidence/v5/lr-reps-summary.jsonl` |
| Presenter dry run, incident session `inc2` (GUI) | Guard alarm at H2S 78.5 ppm; alarm to retreat start 14 ms; retreat 12.1 s; permit ABORTED | `docs/evidence/README.md`, `docs/evidence/rh-analyze.txt` |
| Presenter dry run, startup times | Sim ready 35.6 s (2/2 warm launches); HUD up 2.7 s; reset 2.9 s | `docs/evidence/README.md` |
| Agent-driven session (3 turns) | 122 s end to end; guard alarm 92 ms after the presenter's spike; retreat 14.1 s | `docs/RUNBOOK.md` |
| Model skipped the purge-hold beat on its own | 3 of 20 agent sessions | `docs/RUNBOOK.md`, `docs/LEADERSHIP-BRIEF.md` |
| Facade end-to-end test | `E2E OK: 44/44` (2 runs), RTF 1.00; first stand-up moved 0.078 m | `docs/RUNBOOK.md` |
| Headless RTF | 0.96-1.04 over 338 five-second samples (none below 0.95); front-left camera 10.0 Hz at 640x480 | `docs/RUNBOOK.md` |
| GUI presentation-mode RTF | 394 samples: min 0.95, median 1.00, max 1.06; render 36 ms median, 50 ms max | `docs/RUNBOOK.md` |
| GUI dry-run RTF | min 0.95, median 1.00, max 1.04; render 26-27 ms median, 41 ms max | `docs/evidence/README.md`, raw lines in `docs/evidence/perf-gui.txt` |
| Unit tests | 946 test methods in 20 files | `tests/` |

The raw host logs behind the `docs/RUNBOOK.md` tables stay on the host
(`/opt/spot-demo/logs/`); `docs/evidence/` holds the files that are in the
repo.

## AWS resources and cost

| Resource | Value |
|---|---|
| Region | `us-west-2` |
| Sim host | One EC2 g6.4xlarge (1x NVIDIA L4, 64 GB RAM), Ubuntu 24.04, us-west-2a, IMDSv2, 200 GB gp3 encrypted volume |
| Network | A security group with **no inbound rules** and no public IP. All access is SSM: port forwarding (`AWS-StartPortForwardingSession`, or for presenters the 8443/8080-only `SpotDemo-PortForward` in [`docs/iam/`](docs/iam/README.md)) (DCV 8443, HUD 8080), shell sessions, and `AWS-RunShellScript` (deploy, fetch logs). The HUD listens on the host's loopback only. |
| IAM | An instance role with SSM core, read access to the DCV license bucket, Bedrock invoke on one inference profile, `us.anthropic.claude-sonnet-5`, `bedrock:ApplyGuardrail` on the one guardrail, and, for Session Manager logging, writing to the one session log group and using its key only through CloudWatch Logs. It reads no secrets. Containers get credentials from the role via IMDSv2; no keys are passed in. |
| Secrets (names only; never put values in the repo) | One Secrets Manager secret holding the DCV login password for `ubuntu`, read by the operator on the Mac (`scripts/connect-dcv.sh`), never by the host. Pulling the Isaac Sim image needs no NGC key. |
| Model | Amazon Bedrock `us.anthropic.claude-sonnet-5`, called from us-west-2. It is a cross-Region inference profile: Bedrock may serve a request in another US Region. The prompts carry the operator's instructions, permit fields (roles, not names) and robot telemetry. Amazon Bedrock does not store prompts or responses unless you turn on model invocation logging. |
| Guardrail | One Amazon Bedrock guardrail, from [`infra/bedrock-guardrail.yaml`](infra/bedrock-guardrail.yaml), applied to the operator's latest instruction and the model's reply, on by default. Content filters (prompt attack at HIGH on the input; violence and misconduct deliberately LOW, because the demo's own subject is lethal atmospheres and rescue), a denied topic for real-robot operation, the managed profanity list, and sensitive-information masking with credentials blocked. It is probabilistic and is not a safety function. A CloudWatch dashboard (`<stack>-guardrail`) and an alarm on `InvocationsIntervened` are created with it, so every intervention is reported; pass an SNS topic to have the alarm notify as well. |
| Container images on the host | `nvcr.io/nvidia/isaac-sim:6.1.0` (32.3 GB), `ros:jazzy-ros-base` (1.32 GB), `spot-agent:latest` (1.43 GB), `spot-hud:latest` (1.37 GB) |
| Host paths | `/opt/spot-demo` (deployed code, `logs/`), `/opt/isaac-cache` (warm Isaac caches) |

**What costs money.** On-Demand list prices for us-west-2, from the AWS
Price List API on 2026-09-27, re-checked 2026-10-30 and 2026-11-12, unchanged each
time. Re-check them before quoting a number.

| Item | When it bills | List price |
|---|---|---|
| g6.4xlarge instance | Only while it is running. | $1.3232 per hour |
| 200 GB gp3 volume | All the time, **including while the instance is stopped** (`docs/RUNBOOK.md` step 9). | $0.08 per GB-month, about $16 per month |
| Bedrock model calls | Per request (tokens), only while the agent runs. | See Bedrock pricing for Claude Sonnet 5 |
| On-Demand Capacity Reservation (scheduled demos only) | While it is active, **whether the instance runs or not**; nothing extra while the instance runs in it. | The g6.4xlarge On-Demand rate |
| Cost guardrail stack (optional) | Monthly: one KMS key and three CloudWatch alarms; the budget is free. | $1 per month for the key, plus the alarms (CloudWatch pricing) |

The main cost is forgetting to stop the instance. Stop it after every
session (Quick start step 11).

**Model guardrail (shipped, on by default).**
`infra/bedrock-guardrail.yaml` creates the Amazon Bedrock guardrail the agent
applies to the operator's instruction and the model's reply; deploy it once per
account (`docs/RUNBOOK.md`, step 0) and set `SPOT_GUARDRAIL_ID` and
`SPOT_GUARDRAIL_VERSION` on the host from its outputs. Bedrock bills guardrail
evaluation by text unit, which is small for a demo's short turns.

**Cost guardrails (optional, shipped).** Two controls stop a forgotten host
and alarm spend; both are documented setup steps (`docs/RUNBOOK.md`, step 0,
"Once per host") that you review and turn on:

* `infra/cost-guardrails.yaml`, a CloudFormation stack: an AWS Budgets alert
  on the whole account's cost (Bedrock included) at 80 % and 100 % of actual
  and 100 % of forecast spend, an alarm that stops the instance after 2 hours
  of low CPU, and status-check alarms. See [`infra/README.md`](infra/README.md).
* `host/idle_stop.sh apply`, on the host: a systemd timer that powers the
  host off (an instance-initiated shutdown, which stops it while its
  shutdown behaviour is `stop`, the default) after 60 minutes with no Session
  Manager session, no demo container, no agent and no open mission.

Budget data refreshes up to three times a day, so the budget warns rather
than stops; a sim left running keeps the host up under both controls.

**Check the instance before a demo.** Check the instance state well ahead
of time and again just before you present, and allow time for a start plus
a sim warm-up (READY in about 40-65 s for the GUI on warm caches,
`docs/RUNBOOK.md` step 4b):

```bash
aws ec2 describe-instances --region us-west-2 --instance-ids $SPOT_SIM_INSTANCE \
  --query 'Reservations[].Instances[].State.Name' --output text
```

**Account automation can stop the instance.** Some accounts run cleanup
automation that stops idle instances. Ask your account owner whether yours
does, and how to exempt the sim host. A restart can also fail with
g6.4xlarge capacity errors (`InsufficientInstanceCapacity`); before a live
session, start the instance at least 15 minutes early and confirm the sim
reaches READY. `scripts/start-host.sh` retries a capacity error with a
growing wait. For a **scheduled** live demo, reserve the capacity with an
On-Demand Capacity Reservation for the demo window; it is a required step
(`docs/RUNBOOK.md`, "Capacity for a scheduled demo"), and the reservation
bills while it is active, whether the instance runs or not.

## Known limitations and honest caveats

**Scope of the claims**

1. **Simulation only.** Nothing has run against the real `spot_ros2` driver
   or on hardware.
2. **Simulated gas.** The gas field, the probe, the blower and the battery
   are models. The gas values are illustrative and not derived from any
   real tank. The spike is staged by the presenter and disclosed on screen.
3. **Not certified.** Base Spot is not ATEX or IECEx certified or
   intrinsically safe. A real entry into a tank with flammable vapour needs
   certified equipment or an inerted, gas-free tank, and a site hazard
   review. Say "within permissible entry limits", never "safe", and do not
   name real operators as users.
4. **Permit fields are recorded, not verified.** The operator attests
   isolation, lockout-tagout and the attendant at the console, and the
   demo checks that the rescue plan is filled in, but nothing checks that
   any of it is true. The permit records roles, never names.
5. **No arm model.** The extendable gas probe stands in for an arm-mounted
   probe: no arm kinematics, collisions or motion time. The sim allows 1.1 m
   reach from the body front; the runbook notes the real Spot Arm reaches
   about 0.98 m.
6. **Perfect localization.** Odometry is the simulator's ground-truth pose.
   The real robot's pose will be an estimate.
7. **Smaller interface than planned.** The facade publishes odometry, joint
   states, `/tf`, `payload/*` and body cameras, plus 11 Trigger services.
   There is no depth, lidar, hand camera, PTZ, IR, arm, GraphNav,
   `status/*` or lease topics, although `docs/design.md` and
   `docs/capability-matrix.md` list many of these as must-haves.

**Behaviour to know before a live demo**

8. The model does not always stop at the purge hold (3 of 20 sessions); skip
   the third prompt when the HUD already shows ENTRY AUTHORIZED.
9. If Spot ends up turned more than 45 deg away within 0.9 m of the door,
   there is no automatic recovery: say "Walk to the entry point." and
   re-test.
10. GUI timing was measured without a DCV viewer actually streaming.
11. `run_sim.sh --prepare-desktop` changes host desktop security settings
    (autologin, screen lock off) and, when the `ubuntu` desktop is not up yet,
    restarts `gdm3` and `dcvserver`, which drops connected DCV viewers. Undo it
    with `--restore-desktop` when the host is no longer a demo host.
12. The HUD audit export only covers the events it saw, within its buffer
    limits; the authoritative log is the mission JSONL in `/opt/spot-demo/logs/`.

**Known rough edges in the code**

13. `isaac_app.py` refuses unknown or misspelled flags (so does `run_sim.sh`,
    before a running sim is touched), `--scene` is a no-op, and `--headless`
    needs a value (`true` or `false`).
14. Waypoint numbering differs: `run_mission.py --inject-h2s-at-waypoint` is
    0-based; the agent's `waypoint_1..waypoint_5` are 1-based.
15. Timestamps mix clocks: ROS message stamps are wall time, while the
    watchdog, publish rates and battery run on sim time. Presenter events may
    carry wall time; order across sources by `wall_t`.
16. The OSHA thresholds and site layout are copied into `hud/ros_bridge.py`
    (the HUD container does not ship `sim/`), so they can drift from
    `sim/gas_field.py` and `sim/build_scene.py`.

**Docs that disagree with the code** (the code is right)

17. `docs/design.md`'s architecture diagram is the original plan: a separate
    ROS 2 Humble facade container, Foxglove, Nova Sonic voice and IoT Core.
    None of those were built; the facade runs inside the Isaac Sim process on
    its bundled Jazzy, and the agent and HUD containers are
    `ros:jazzy-ros-base`.
18. `docs/design.md` and `docs/capability-matrix.md` say the `cmd_vel`
    watchdog is 0.25 s; the code and `spot_ros2` use 0.125 s.
    `docs/design.md` says the e2e test has 40 checks; the newer runbook
    reports 44/44.
19. `docs/RUNBOOK.md` step 2 and `hud/README.md` say deploy ships five
    folders; it also ships `tools/`.
20. `docs/evidence/README.md` lists some older v5 screenshots that now live
    only in the local, gitignored `.evidence-archive/`, and gives 355 PERF
    samples for `perf-gui.txt`, which holds 532 PERF lines (min, median and
    max match).
21. Dates labelled 2026-09-28 (the leadership rehearsal) are UTC; the same
    work is dated 2026-09-27 in US Pacific time.

## Roadmap to the real robot

**Not supported by this sample.** The agent refuses to command anything but
this simulator, and there is no switch to turn that off. The list below is
what a separate project would need before a real Spot is powered on near a
tank. It is a list of conditions, not steps to follow, and each one needs
people qualified to sign it off.

1. **A safety case.** A hazard analysis of the deployed system (STPA or
   HAZOP), a functional-safety assessment (for example IEC 61508 or
   ISO 13849) of every function relied on for protection, an independent gas
   interlock, and the site's confined-space programme under
   29 CFR 1910.146 (entry supervisor, attendant, rescue). The SafetyGuard in
   this repository is a demonstration, not a protective function.
2. **A human-held stop.** A vendor E-Stop endpoint held by the
   attendant, registered and tested before power-on, independent of the
   agent and of the network path the agent uses.
3. **A supervised guard.** The gas guard and the retreat running as a
   separately supervised process, or on the robot, so that stopping or
   crashing the agent cannot end monitoring.
4. **Hazardous areas.** Base Spot is not ATEX or IECEx certified: only an
   inerted or gas-free tank, or certified equipment, with a hazardous-area
   classification by a competent person.
5. **Real instruments.** A real gas detector and probe with calibration,
   bump-test and health inputs, in place of the simulated `payload/gas` and
   `payload/gas/probe`.
6. **A secured robot network.** SROS2 in Enforce mode for every node, the
   robot's credentials held in a secrets store and passed only to the driver
   for a least-privilege robot user, and the robot vendor's own network and
   credential hardening applied to the robot.
7. **A surveyed site.** A site map anchored to the robot's own localization,
   with a personnel zone around the entry and the attendant, in place of the
   simulator's coordinates.
8. **Reviews.** A security review of the real-robot system and a legal review
   of the NVIDIA, robot-vendor and model-provider terms for that use.

The threat model (`docs/security/THREAT_MODEL.md`) lists these as the gates
before any real-robot run.

## Security

* **Threat model.** [`docs/security/THREAT_MODEL.md`](docs/security/THREAT_MODEL.md)
  covers the data flows, trust boundaries, every threat found and its status.
* **Host baseline.** [`docs/iam/`](docs/iam/README.md) holds least-privilege
  IAM policies for the instance role and for presenters, and the host
  settings to apply: no inbound rules, no public IP, IMDSv2 with hop
  limit 1. Its Session Manager preferences run every shell as the
  operator's own OS account (`host/add_operator.sh`) and stream it to an
  encrypted CloudWatch Logs log group (`infra/session-logging.yaml`), so the
  mission log names its operator and its chain head has a copy off the host.
* **Reach.** The HUD and the DCV desktop are reached only through SSM port
  forwarding and listen on the host's loopback. Never open 8443 or 8080 to a
  network. ROS 2 discovery is limited to the host
  (a loopback-only Fast DDS profile), but the bus has no
  authentication: any process on the host can read and publish on it, so
  run nothing else on the sim host. `run_sim.sh --background` fails if
  anything listens off loopback that should not; a host with `sshd` on every
  interface fails it until you stop `sshd` (on Ubuntu 24.04 systemd holds its
  socket: `sudo systemctl disable --now ssh.socket ssh.service`; Session Manager
  needs no SSH).
* **Containers.** Each container runs as its own unprivileged user (agent
  10001, HUD 10002, one-shots 10003, Isaac Sim 1234) with capabilities
  dropped, `no-new-privileges` and a read-only root filesystem (except the
  Isaac Sim container, which keeps the GPU runtime's defaults). Only the
  agent may use the instance role: `host/imds_guard.sh apply` makes the
  kernel refuse the metadata service to every other user, and every launch
  refuses to start a container until those rules are in place (they do not
  survive a reboot; `SPOT_IMDS_GUARD=off` skips the check and says so). Images
  are pinned by digest and Python packages by hash.
* **Reporting a vulnerability.** See [CONTRIBUTING.md](CONTRIBUTING.md#security-issue-notifications).
  Please do not open a public issue.

## Cleanup

Stop the instance after every session (Quick start step 11): it bills while
running. When you are done with the demo, delete everything it created, in
this order:

1. Fetched logs on the Mac: `rm -rf ~/spot-demo-logs`. They hold operator
   chat and mission records.
2. The EC2 instance, with its EBS volume (the volume bills while the
   instance is stopped) and any snapshots or AMIs made from it; any
   On-Demand Capacity Reservation made for a demo (it bills while it is
   active, even with the instance stopped or gone); and the cost guardrail
   stack, if you deployed it (its budget, alarms, topic and key;
   `infra/README.md`). If you keep the instance for something else instead,
   undo the host changes: `sudo bash /opt/spot-demo/host/idle_stop.sh remove`
   and the undo steps in `docs/RUNBOOK.md`, step 0.
3. The security group, and the instance profile, the instance role and its
   inline policy.
4. The Secrets Manager secret holding the DCV password.
5. The Session Manager preferences, the `SSMSessionRunAs` tags and the
   session-logging stack, whose log group and KMS key are retained when the
   stack is deleted ([`docs/iam/README.md`](docs/iam/README.md#session-manager-one-os-account-per-operator-and-session-logs-off-the-host),
   "To roll back"); any other CloudWatch log groups or S3 prefixes you created
   for Session Manager logging; and the AWS Budgets alert if it was only for
   this demo.
6. The Bedrock guardrail stack, last: with the guardrail gone, a session
   started without `--no-guardrail` fails at its first model call
   (`infra/README.md`).

```bash
aws ec2 describe-capacity-reservations --filters Name=state,Values=active \
  --query 'CapacityReservations[].[CapacityReservationId,InstanceType,AvailabilityZone]'
aws ec2 cancel-capacity-reservation --capacity-reservation-id <cr-id>
aws ec2 terminate-instances --instance-ids <instance-id>
aws cloudformation delete-stack --stack-name spot-demo-guardrails
aws ec2 delete-security-group --group-id <security-group-id>
aws iam remove-role-from-instance-profile --instance-profile-name <profile> --role-name <role>
aws iam delete-instance-profile --instance-profile-name <profile>
aws iam delete-role-policy --role-name <role> --policy-name <policy>
aws iam detach-role-policy --role-name <role> \
  --policy-arn arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore
aws iam delete-role --role-name <role>
aws secretsmanager delete-secret --secret-id <dcv-password-secret>
aws cloudformation delete-stack --stack-name spot-demo-bedrock-guardrail
```

`terminate-instances` deletes the root volume only if it was created with
`DeleteOnTermination` (the default for the root volume). Check with
`aws ec2 describe-volumes` that nothing is left.

## Contributing

- **Start with [CONTRIBUTING.md](CONTRIBUTING.md):** the checks to run,
  the files whose changes need a second reviewer, and how to report a
  security issue. Work on a branch cut from `main`.
- **Commit messages.** Conventional Commits, as in the history:
  `feat(sim): ...`, `fix: ...`, `perf(sim): ...`, `docs: ...`, in the
  imperative mood.
- **Never commit secrets.** No AWS keys, DCV password or NGC key in code,
  docs, logs or screenshots. Secrets stay in Secrets Manager and are
  referenced by name only; the containers use the instance role.
- **Evidence images.** Keep new screenshots at most 1600 px wide and small:
  the dry-run set (`01`-`14`) is 1600 px wide and under 400 KB each, the
  `v5/` frames up to about 650 KB. Retired images go to the gitignored
  `.evidence-archive/`, not the repo.
- **Do not move test fixtures.** `docs/evidence/events-{ok1,inc2}.jsonl`
  and `hud-audit-{ok1,inc2}.csv` are opened by name in `tests/test_hud.py`.
- **Before committing,** run `python3 -m unittest discover -s tests`. After
  changing `sim/`, also run `host/smoke_test.sh` and `host/e2e_ros_test.sh`
  on the host. After changing `agent/` or `hud/`, rebuild their images on
  the host.
- **Keep the language rules.** In code, prompts, UI and docs: "within
  permissible entry limits", never "safe"; no certification claims; no named
  operators; readings are simulated.
- **Keep copies in sync.** If you change the OSHA thresholds or the site
  layout in `sim/`, update `hud/ros_bridge.py` and `agent/site_defaults.json`
  too, and update the docs when a measured number changes.
