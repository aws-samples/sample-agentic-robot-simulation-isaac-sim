# Isaac Sim 6.1.0 API notes (verified against the real container)

Every snippet below was extracted verbatim from `nvcr.io/nvidia/isaac-sim:6.1.0` on the EC2 sim
host on 2026-09-27, via read-only `docker run --entrypoint bash` inspection.
Do not trust memory of Isaac Sim 4.x APIs — 6.1.0 changed the Spot policy API completely.

## 0. Container facts

- OS inside image: **Ubuntu 24.04.3 LTS (noble)** (`/etc/os-release`).
- Bundled internal ROS 2 distros: **humble AND jazzy**, at
  `/isaac-sim/exts/isaacsim.ros2.core/{humble,jazzy}/` (each contains `lib/`, `rclpy/`, `share/`).
  Backend libs: `/isaac-sim/exts/isaacsim.ros2.core/bin/libisaacsim.ros2.core.{humble,jazzy}.so`.
- From `isaacsim.ros2.core/docs/Overview.md` (verbatim, line 106):
  > At startup, the extension detects the configured ROS 2 distribution from the `ROS_DISTRO`
  > environment variable. The standard Isaac Sim launchers set this variable automatically for
  > bundled ROS 2 libraries when `ROS_DISTRO` is unset. If `ROS_DISTRO` remains unset at extension
  > startup, the extension falls back to the Ubuntu default (Humble for 22.04, **Jazzy for 24.04**)
  > and loads the bundled internal ROS 2 libraries.
  So in this container, default internal distro = **jazzy** (Ubuntu 24.04). No manual
  `LD_LIBRARY_PATH` needed when using the standard launchers (`./python.sh`); source a system ROS 2
  only for custom message packages / custom RMW.
- Setting (from `isaacsim.ros2.core/config/extension.toml`):
  `exts."isaacsim.ros2.bridge".ros_distro = "system_default"`.
- ROS 2 pub/sub is active **only during simulation playback** (bridge Overview.md).

## 1. Spot policy example — `spot_standalone.py`

Path: `/isaac-sim/standalone_examples/api/isaacsim.robot.policy.examples/spot_standalone.py`
(166 lines; helper `command_path.py`, 248 lines, same dir).

**CRITICAL API CHANGE:** the extension `isaacsim.robot.policy.examples` is at version 7.0.0 and
**removed `SpotFlatTerrainPolicy` / `PolicyController`**. The old names still import but are
migration shims in `_legacy.py` that **raise on construction**. The shim's own migration text
(verbatim from `_legacy.py`):

```python
"from isaacsim.robot.policy.examples import RobotPolicyRunner, get_spot_spec",
"""runner = RobotPolicyRunner(
    get_spot_spec(), prim_path=prim_path, position=position, orientation=orientation
)""",
"runner.step(dt, command)",
```

### Exact imports used by spot_standalone.py (verbatim)

```python
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Select simulation engine and device.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda", help="Simulation device")
parser.add_argument("--engine", type=str, choices=["physx", "newton"], default="physx", help="Physics engine")

args, unknown = parser.parse_known_args()
extra_args = [f"--/exts/isaacsim.core.simulation_manager/default_engine={args.engine}"]
if args.engine == "newton":
    extra_args.extend(["--enable", "isaacsim.physics.newton", "--enable", "isaacsim.physics.newton.tensors"])
simulation_app = SimulationApp({"headless": False, "extra_args": extra_args})

import carb
import numpy as np
import omni.timeline
from command_path import TraveledPath, author_command_path, phase_boundary_frames, report_tracking
from isaacsim.core.experimental.utils.stage import define_prim, set_stage_units, set_stage_up_axis
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import PolicyEnvConfig, RobotPolicyRunner, get_spot_spec
from isaacsim.storage.native import get_assets_root_path
```

Headless: pass `{"headless": True}` to `SimulationApp` (example ships with `False`).
`SimulationApp` MUST be constructed before any other isaacsim/omni import.

### World / ground / physics scene creation (verbatim)

```python
set_stage_up_axis("Z")
set_stage_units(meters_per_unit=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# spawn physics scene
# TODO: physics scene should be created by simulation manager
define_prim("/World/PhysicsScene", "PhysicsScene")

SimulationManager.set_physics_sim_device(args.device)
```

Note: no `World` class — 6.1 standalone examples use `SimulationManager` +
`omni.timeline` directly (the 4.x `omni.isaac.core World` pattern is gone from these examples).

### Robot construction (verbatim)

```python
spec = get_spot_spec()
spot = RobotPolicyRunner(spec, prim_path="/World/Spot")
spot.spawn()

timing = PolicyEnvConfig.from_file(spec.engines[args.engine].env_config_path).timing
frame_dt = timing.render_interval * timing.physics_dt
RenderingManager.set_dt(frame_dt)
SimulationManager.set_physics_dt(spot.physics_dt)
```

### Per-physics-step command application (verbatim)

```python
first_step = True
reset_needed = False
policy_failed = False

def on_physics_step(step_size: float, context: object) -> None:
    global first_step, reset_needed, policy_failed
    if policy_failed:
        return
    try:
        if first_step:
            spot.restart_from_default_state(base_command)
            first_step = False
        elif reset_needed:
            reset_needed = False
            first_step = True
        else:
            spot.step(step_size, base_command)
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        policy_failed = True
        carb.log_error(f"spot_standalone: policy deployment failed, stopping: {error}")

base_command = np.zeros(3, dtype=np.float32)   # [vx, vy, yaw_rate], body frame

_physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()
    if SimulationManager.is_simulating():
        base_command = frame_commands[i]        # set command per app frame
        ...
    else:
        reset_needed = True
timeline.stop()
SimulationManager.deregister_callback(_physics_callback_id)
spot.close()
simulation_app.close()
```

Command schedule shape in the example: `COMMAND_PHASES = [(frames, [vx, vy, yaw_rate]), ...]`,
e.g. `(55, [1.5, 0.0, 0.0])` forward, `(30, [0.0, 1.0, 0.0])` strafe left, `(68, [1.2, 0.0, 1.4])`
arc left. One command held per app frame.

### RobotPolicyRunner API surface (from `runtime.py`, extension source)

```python
class RobotPolicyRunner:
    def __init__(self, spec: PolicySpec, *, prim_path: str,
                 position: Sequence[float] | None = None,
                 orientation: Sequence[float] | None = None,      # WXYZ
                 training_engine: str | None = None,               # "physx"/"newton"
                 task_state_provider: Callable[[Articulation], object] | None = None) -> None
    # properties: articulation -> Articulation | None; engine -> str | None
    #             physics_dt -> float; decimation -> int; env_config -> PolicyEnvConfig
    def spawn(self) -> Articulation
    def initialize(self) -> None
    def restart_from_default_state(self, command=None) -> None
    def reset(self) -> None
    def step(self, dt: float, command: Sequence[float] | None = None) -> None
    def close(self) -> None
```

`spot.articulation.get_world_poses()` returns `(positions, orientations)` tensors
(`.numpy().reshape(-1)`; orientation is WXYZ quaternion) — used by `command_path.TraveledPath.record`.

### Bundled spot spec asset handling (verbatim from `bundled/spot.py`)

```python
assets_root_path = get_assets_root_path()
policy_dir = f"{assets_root_path}/Isaac/Samples/Policies/Spot_Policies"
return PolicySpec(
    name="spot_flat_terrain",
    engines={
        "physx": PolicyArtifact.from_files(
            f"{policy_dir}/physx_policy.pt",
            f"{policy_dir}/physx_env.yaml",
            f"{policy_dir}/spot_physx_IO_descriptors.yaml",
            model_sha256="37084fd7d48ffc23100b025b3ffc92bc00507a5be6f676b005781d579d834b8d",
        ),
        "newton": PolicyArtifact.from_files(... newton_policy.pt / newton_env.yaml ...),
    },
    usd_path=f"{assets_root_path}/Isaac/Robots/BostonDynamics/spot/spot.usd",
    default_spawn_orientation=(1.0, 0.0, 0.0, 0.0),
)
```

Observation is 48-wide, action 12-wide joint positions, action scale 0.2 — all derived from the
exported IO descriptor at deploy time (docstring of `bundled/spot.py`).
`PolicySpec` fields: `engines, usd_path, name, default_spawn_position,
default_spawn_orientation, zero_targets_on_initialize, binding`. Bundled getters take any field as
kwarg override, e.g. `get_spot_spec(default_spawn_position=(x, y, z))`.

### command_path.py helper (same dir; must sit next to the script or be on sys.path)

Exports `integrate_command_phases`, `phase_boundary_frames`, `author_command_path` (draws green
commanded course + yellow waypoint cones via `UsdGeom.BasisCurves`/`UsdGeom.Cone`),
`TraveledPath` (`.record(articulation, mark=False)`, `.last_pose`), `report_tracking`.
Pure USD authoring — no physics dependencies; reusable for path visualization.

## 2. ROS 2 bridge in standalone Python

Examples dir `/isaac-sim/standalone_examples/api/isaacsim.ros2.bridge/`:
`camera_manual.py, camera_noise.py, camera_periodic.py, camera_rclpy_async.py,
carter_multiple_robot_navigation.py, carter_stereo.py, clock.py, moveit.py, rtx_lidar.py,
rtx_lidar_rclpy.py, rtx_radar.py, subscriber.py`.

### Extension enable + rclpy pattern (verbatim from clock.py)

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": True})
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
from isaacsim.core.simulation_manager import SimulationManager
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
import rclpy                     # imported ONLY AFTER the extension is enabled
from rosgraph_msgs.msg import Clock
rclpy.init()
```

Then plain rclpy: `node = rclpy.create_node("isaac_sim_clock")`,
`node.create_subscription(Clock, topic, cb, 1)`, and inside the main loop
`rclpy.spin_once(node, timeout_sec=0.0)` after each `simulation_app.update()`.
Sim setup in these examples: `stage_utils.set_stage_units(meters_per_unit=1.0)`;
`SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")`; `app_utils.play()`.

### Clock publishing via OmniGraph (verbatim core of clock.py)

```python
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "sim_time"),
        ],
    },
)
```

### Subscriber-in-a-Node pattern (subscriber.py)

`class Subscriber(Node)` subclassing `rclpy.node.Node`; builds scene with
`from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane`
(`GroundPlane("/World/GroundPlane")` is the simple ground-plane one-liner);
`self.create_subscription(Empty, "move_cube", self.move_cube_callback, 10)`; main loop calls
`simulation_app.update()` then `rclpy.spin_once(self, timeout_sec=0.0)`;
`self.cube.set_world_poses(positions=...)` applies data; `app_utils.is_playing()` gates actions.

## 3. Asset root

- `get_assets_root_path()` from `isaacsim.storage.native` reads carb setting
  `/persistent/isaac/asset_root/default` (`nucleus.py:34`), verifies reachability, returns URL.
- Container default (from `isaacsim.storage.native/config/extension.toml`, verbatim):
  `persistent.isaac.asset_root.default = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"`
  (assets are streamed from that S3 cloud bucket; needs outbound HTTPS, 5 s timeout / 3 retries).
- Environment USDs actually referenced by shipped 6.1.0 examples (all relative to assets root):
  - `/Isaac/Environments/Grid/default_environment.usd` (spot_standalone.py uses this)
  - `/Isaac/Environments/Simple_Warehouse/full_warehouse.usd`
  - `/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd` (camera_periodic.py)
  - Props under `/Isaac/Environments/Simple_Warehouse/Props/` (e.g. `SM_PaletteA_01.usd`,
    `S_TrafficCone.usd`, `S_WetFloorSign.usd`) and `/Isaac/Environments/Hospital/Props/`.
- Robot USD: `/Isaac/Robots/BostonDynamics/spot/spot.usd`.
- Spot policies: `/Isaac/Samples/Policies/Spot_Policies/{physx_policy.pt,physx_env.yaml,spot_physx_IO_descriptors.yaml,...}`.

## 4. Camera → ROS 2 quick pattern

There is NO `isaacsim.sensors.camera` python extension in 6.1.0 (only `isaacsim.sensors.camera.ui`,
`isaacsim.sensors.experimental.rtx`, etc.; no `api/isaacsim.sensors.camera` standalone dir). The
shipped camera pattern is plain USD camera + OmniGraph `ROS2CameraHelper` (camera_periodic.py):

```python
camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim("/Camera", "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim)
xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim.GetFocalLengthAttr().Set(24)
# OmniGraph (evaluator "push", GRAPH_PIPELINE_STAGE_ONDEMAND):
#   ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")  width=1280 height=720
#       inputs:cameraPrim = [usdrt.Sdf.Path("/Camera")]
#   ("cameraHelperRgb",  "isaacsim.ros2.bridge.ROS2CameraHelper")  type="rgb"  topicName="rgb"
#   ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper") topicName="camera_info"
#   ("cameraHelperDepth","isaacsim.ros2.bridge.ROS2CameraHelper")  type="depth" topicName="depth"
# then: og.Controller.evaluate_sync(ros_camera_graph)
```

## 5. Gotchas for code-writing agents

1. `SimulationApp({...})` first, all isaacsim/omni imports after.
2. Do NOT use `SpotFlatTerrainPolicy` — it raises. Use `RobotPolicyRunner(get_spot_spec(), prim_path=...)`.
3. Enable `isaacsim.ros2.bridge` via `app_utils.enable_extension(...)` + `simulation_app.update()`
   BEFORE `import rclpy`.
4. Set `SimulationManager.set_physics_dt(spot.physics_dt)` from the runner, and render dt from
   `PolicyEnvConfig.from_file(spec.engines[engine].env_config_path).timing`
   (`frame_dt = timing.render_interval * timing.physics_dt`).
5. Command vector is 3-wide float32 `[vx, vy, yaw_rate]` in body frame; step the policy only from
   the `IsaacEvents.POST_PHYSICS_STEP` callback; first callback must call
   `restart_from_default_state(command)`.
6. Physics callbacks must not raise — wrap in try/except (the shipped example does).
7. Assets stream from the public S3 asset root; the EC2 host needs outbound HTTPS or a local
   asset pack with `/persistent/isaac/asset_root/default` overridden.
8. ROS pub/sub only works while the timeline is playing.
