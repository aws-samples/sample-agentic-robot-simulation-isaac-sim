# Spot confined-space-entry demo — design

Status: rehearsed end to end in simulation, 2026-09-27. Published as a simulation-only sample:
the agent refuses to command anything but this simulator, and a real robot would need a separate
project (README, "Roadmap to the real robot"); this note keeps the original plan for the record. Full host
rehearsal: facade e2e 40/40 (re-stand fixed), scripted storyline (raw test FAIL -> ventilation ->
re-test PASS -> entry -> exit) COMPLETED 5/5, scripted H2S excursion ABORTED with guard retreat
3/3, agent-driven storyline on Bedrock Sonnet 5 with a presenter spike mid-entry: guard retreat and
a spoken incident explanation. RTF 0.96-1.04 throughout. Numbers: `docs/RUNBOOK.md`.

## Decision: Isaac Sim 6.1 on EC2

Chosen over the MuJoCo stack (strands-robots + Menagerie + Playground-trained policy) because
Isaac ships a working Spot flat-terrain locomotion policy, the Spot USD (plus an arm variant),
RTX camera/lidar sensors, and a ROS 2 Humble/Jazzy bridge — the walking-gait risk that dominated
the MuJoCo path disappears. Verified on our host: `spot_standalone.py` walks headless in the
`nvcr.io/nvidia/isaac-sim:6.1.0` container (open-loop course, 0.53 m / 27° end-pose drift over
one circuit). Trade-off: Isaac Sim itself is proprietary (EULA acceptance; no license key), and
its bundled assets carry NVIDIA's "Additional Software and Materials License" — fine for a demo,
re-check before public redistribution. No NGC API key is required to pull the container.

## Architecture

This was the design at the start. As built, nothing listens beyond the host's
loopback except through the two SSM port forwards below; there is no Foxglove
bridge, Isaac livestream or other remote viewer, and adding one would need its
own review (threat model T44).

```
Mac (presenter)
  └─ SSM tunnels only (no inbound ports on the instance)
       ├─ NICE DCV https://localhost:8443  → full GUI (Isaac viewport)
       └─ HUD      http://localhost:8080   → presenter dashboard

EC2 g6.4xlarge (L4 24 GB, 64 GB RAM, Ubuntu 24.04, us-west-2)
  ├─ Isaac Sim 6.1.0 container (--network=host, headless or DCV-visible)
  │    ├─ Spot USD + flat-terrain policy  (cmd_vel → joint targets)
  │    ├─ Confined-space scene (tank floor + API 650-style cleanout door)
  │    ├─ ROS 2 bridge (OmniGraph): cameras, depth, lidar, odom/TF, joint_states
  │    └─ Gas field plugin: analytic concentration field sampled at body pose
  ├─ ROS 2 Humble container: spot_msgs + "spot facade" node
  │    └─ implements the spot_ros2 topic/service/action surface verbatim
  │       (see docs/capability-matrix.md → "names the facade must implement")
  ├─ Agent: Strands Agents + Bedrock (Claude); tools call ROS 2 (facade), not the SDK
  │    └─ optional voice: Amazon Nova Sonic (pattern from aws-samples Go2 repo)
  └─ Telemetry: gas readings → IoT Core/dashboard (phase 2)

Real robot later: replace the facade+Isaac pair with spot_ros2 (mock_enable:=False).
Contract test: run spot_ros2 with mock_enable:=True, diff `ros2 topic|service|action list -t`
against the facade.
```

## Scenario (regulator-grade script)

Scene: crude-tank floor entered through a ~4×4 ft flush-type cleanout door (API 650 style) — a
round 24″ shell manway is NOT physically plausible for Spot (1.1 × 0.5 × 0.61–0.70 m standing)
and an HSE audience knows it. Dark interior, internal ladder/baffles, sludge patches, heavier-
than-air H2S pooling low.

1. Permit brief on HUD (permit no., isolation/LOTO, attendant). Gate: fields populated before motion.
2. Approach cleanout door; exterior visual/thermal-stand-in scan.
3. Arm presents gas probe at the opening; log **O2 → LEL → H2S/CO in that order**
   (OSHA 1910.146 App. E thresholds: O2 19.5–23.5 %, LEL <10 %, H2S <10 ppm, CO <35 ppm).
4. Enter; continuous telemetry + camera streams, timestamped (no gap >30 s).
5. **Injected H2S spike** (operator console, not scripted for the agent): agent detects the
   excursion, states rationale, autonomously retreats, permit flips to Aborted, alert fires.
6. Post-entry: exportable atmospheric log (audit trail).
7. Narration: "no human entered the space." Disclose that base Spot is not ATEX-certified
   (unclassified/Zone-2-with-mitigations only); do not name operators (bp/Shell/etc.) —
   unverified for confined-space use specifically.

## Capability scope (from docs/capability-matrix.md)

Must-have: cmd_vel walk + 0.25 s watchdog, stand/sit/stop, power/lease/e-stop state machine,
trajectory/navigate_to, odom/TF/joint_states, status/*, 5 body cams + hand cam, depth, RTX lidar
(velodyne/points), PTZ, gas sensor (custom `payload/` namespace), IR stand-in (false-color from
per-prim temperature), safety gate on dangerous actions.
Arm: unstow/pose/gripper for the probe-presentation beat (M); grasping out of scope.
GraphNav: YAML waypoint graph behind the same service names (real map recorded on the robot later).

## Repo findings that shaped this

- `spot_ros2`: real-robot driver, no physics; its `mock_enable:=True` mode defines the facade contract.
- `strands-spot`: 8 SDK clients only; advertised graph_nav/manipulation/docking fail at runtime; SDK-direct so it can't drive a ROS facade.
- `strands-labs/robots`: MuJoCo Spot (Menagerie spot_arm.xml), no locomotion policy, Isaac backend pinned to 6.0.1; useful as agent-tool pattern.
- `aws-samples/sample-ros2-with-agentic-control-on-aws`: Unitree Go2, but its Nova Sonic voice agent, Bedrock vision tools, client-swap pattern (`client_registry.set_client`), Foxglove-over-SSM, and CDK stacks port over.

## Host inventory

- EC2 g6.4xlarge (NVIDIA L4 23 GB, driver 595.91.07), us-west-2a, a security group with no ingress, IMDSv2, 200 GB gp3 encrypted (42 GB used, 2026-09-27).
- IAM: an instance role with SSM core + the DCV license bucket; Bedrock invoke of `us.anthropic.claude-sonnet-5` from the agent container works via the instance role (verified 2026-09-27).
- Docker + nvidia-container-toolkit. Images: `nvcr.io/nvidia/isaac-sim:6.1.0` (32.3 GB), `ros:jazzy-ros-base` (1.32 GB), `spot-agent:latest` (1.43 GB, `agent/Dockerfile`), `spot-hud:latest` (1.37 GB, `hud/Dockerfile`). Both demo images use a `--system-site-packages` venv (pip into the system Python fails on Ubuntu's RECORD-less typing_extensions 4.10).
- `/opt/spot-demo` = `sim/ host/ agent/ hud/ tests/ tools/` shipped by `scripts/deploy-to-host.sh` (SSM chunks + sha256), plus `logs/` (mission JSONL audit logs, mounted into the agent container at `/workspace/logs`).
- Host scripts: `run_sim.sh` (Isaac container `spot-sim`), `run_hud.sh` (container `spot-hud`, 127.0.0.1:8080), `run_agent.sh` (agent REPL / `--once` / `--mission`), `reset_spot.sh` (clear injected gas, optional `--reset-ventilation`, self_right: back to spawn, standing), `inject_gas.sh` (presenter gas pocket / `clear` / `vent on|off|reset`), `e2e_ros_test.sh` (40 facade checks incl. 3 re-stand cycles and the gas controls).
- NICE DCV console session (`ubuntu`, in the `docker` group), reachable only via `scripts/connect-dcv.sh`; HUD via `scripts/connect-hud.sh` (both SSM port forwards).
- Isaac caches persisted at `/opt/isaac-cache` (mount into every container run). Sim start to `FACADE-READY`: 36 s warm.
- Measured 2026-09-27 (default cameras frontleft+frontright): RTF 0.98-1.02 (0.96-1.04 over the full rehearsal), camera/frontleft/image 10.0 Hz 640x480 (fx=fy=336), latched `payload/site` received by late joiners, e2e 40/40. Operator steps and all measured mission numbers: `docs/RUNBOOK.md`.

## Next

Done 2026-09-27 (was Next 1-3): simulated forced ventilation (built-in pockets decay toward a 4 %
residual, tau 15 s) and a door-mouth gas probe 0.5 m inside the opening at 0.2/0.6/1.0 m, so the
unventilated test fails and the ventilated re-test passes; re-stand after sit (policy runtime reset
at the stand-up handover plus a stiff scripted rise over the feet); `payload/gas/clear` for
injected pockets; permit fields gate (isolation, LOTO, attendant, rescue plan) and HUD audit export.

1. Real `spot_ros2` / hardware: nothing verified yet (arm probe, GraphNav, e-stop semantics).
2. Probe realism: no reach limit without a body pose, and the probe can sample through the wall; the
   door probe would pass at ~19 % contamination while the floor inside still reads 11-16 ppm H2S
   (the storyline waits for 3 tau, 9 %). Consider requiring the purge target before any re-test.
3. Audit clocks: event `t` is sim time for mission/agent events and wall time for presenter
   scripts; the HUD CSV mixes both (order by `rx_wall_utc`).
4. Isaac GUI demo run over DCV with the HUD side by side (not timed); exterior scan; GraphNav YAML.
