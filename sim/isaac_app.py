"""Isaac Sim 6.1 standalone entrypoint for the Spot confined-space demo.

What it is:
    The single simulation-side process. It loads Spot with the bundled flat-terrain
    policy (Isaac Sim 6.1 RobotPolicyRunner API), builds the demo environment via
    sim/build_scene.py, instantiates the GasField from sim/gas_field.py, and exposes
    a spot_ros2-compatible ROS 2 facade (topics/services named per
    docs/facade-contract-v1.md) under the /demo_spot namespace.

How it runs (inside the Isaac Sim 6.1.0 container ONLY):
    /isaac-sim/python.sh /workspace/sim/isaac_app.py [--headless true|false]
        [--smoke] [--cameras all|none|<list>] [--no-cameras] [--scene <usd path>]
        [--gas-initial raw|ventilated] [--ui presentation|editor]
        [--view auto|chase|interior|overhead|door] [--battery-drain demo|real|off]

    Unknown arguments are refused (exit 2) before Kit starts: nothing is
    passed through to Kit (sim/facade_guard.py unknown_args_error, T44).

    --headless    default true; passes {"headless": ...} to SimulationApp.
                  false = GUI (host/run_sim.sh --gui puts it on the DCV
                  desktop): the viewport shows the presenter camera
                  (sim/sensors.py CameraDirector, --view / payload/view).
    --ui          GUI only. presentation (default): the Kit window goes
                  fullscreen with every editor panel, the main menu and the
                  viewport menubar / axis gizmo hidden -- the Kit F11 path
                  (action omni.kit.ui.actions/toggle_fullscreen: app window
                  set_fullscreen + /app/window/hideUi). editor: the stock
                  Isaac Sim editor layout (host/run_sim.sh --editor), window
                  maximized. Switch at runtime with payload/view {"ui": ...}
                  (host/set_view.sh --ui editor|presentation).
    --view        initial presenter camera (default auto; see payload/view).
    --gui-res     GUI viewport render resolution WxH (default 1280x720),
                  independent of the window size, so RTF does not depend on
                  the DCV client's screen.
    --battery-drain  simulated battery (sim/battery.py): demo (default,
                  25 %/h, floor 30 %: green for a whole session), real (~90 min
                  full to empty, floor 5 %) or off (100 %). Reset to 100 % on
                  payload/battery/reset (host/reset_spot.sh).
    --smoke       run ~600 physics steps, print odom pose + a gas sample every
                  100 steps, then print the exact marker "SMOKE-OK" and exit 0
                  (auto claims/powers/stands so the policy actually runs);
                  any failure exits 1 without printing the marker.
    --cameras     body cameras to publish (sim/sensors.py): "all", "none" or a
                  comma list of frontleft,frontright,left,right,back
                  (default frontleft,frontright). 640x480 rgb8 at --camera-hz.
    --no-cameras  alias for --cameras none (smoke/e2e tests use it).
    --render-hz   stage render rate (default 10; GUI viewport fps too). Cameras
                  still publish at --camera-hz (frameSkipCount).
    --scene       optional USD environment override forwarded to build_scene.
    --actuators   fast (default): PD on the PhysX joint drives; newton: stock
                  explicit Newton actuator models (see "Actuator mode" below).
    --gas-initial raw (default, storyline start): tank unventilated -- seed-7
                  pockets plus the unventilated headspace (sim/gas_field.py
                  UNVENTILATED_HEADSPACE), so a door-mouth probe fails.
                  ventilated: start already at the ventilation residual
                  (quick rehearsals of the entry itself).

Facade surface (namespace /demo_spot):
    subs:  cmd_vel (Twist, 0.125 s sim-time watchdog = contract cmd_duration;
             on a zero or expired command the standing policy holds its
             heading, sim/stance_hold.py; non-finite or beyond 2x the demo
             envelope = refused and Spot stops, sim/facade_guard.py),
           body_pose (Pose, v1 stub), and (String JSON, SIMULATED gas controls,
           applied on the sim thread at the next 20 ms tick; each String
           control is rate-limited, size-capped and checked against
           allowlists and bounds in sim/facade_guard.py, and every world
           control applied or refused gets a payload/events record):
           payload/gas/inject {"gas","ppm","x","y","z","radius"} (O2:
             depletion deltas only; at most 16 injected pockets),
           payload/gas/clear ({} or {"clear": "injected"}: remove every
             injected spike),
           payload/gas/ventilation {"on": bool, "tau_s": 2..3600 optional} or
             {"reset": true} (back to the --gas-initial state),
           payload/gas/probe {"id": str (1-64 printable ASCII, echoed in
             the reply), "points": [[x,y,z], ...]} -- an
             extendable gas probe (simulated; arm-mounted on the real
             robot): refused unless the body pose is known, every point is
             within 1.1 m (horizontal) of the body's front and the straight
             body->point line crosses the tank shell only inside the
             1.3 x 1.3 m doorway (sim/gas_field.py ProbeGeometry),
           payload/view (String: {"camera": "auto|chase|interior|overhead|door"}
             and/or {"ui": "presentation|editor"}, or a bare view name;
             presenter camera / GUI layout, GUI only; host/set_view.sh),
           payload/battery/reset (String, empty or {}: simulated battery
             back to 100 %; host/reset_spot.sh)
    srvs:  claim release power_on power_off stand sit stop self_right
           estop/hard estop/gentle estop/release   (all std_srvs/Trigger)
    pubs:  odometry (nav_msgs/Odometry, 50 Hz, odom -> demo_spot/body),
           joint_states (sensor_msgs/JointState, 50 Hz, 12 spot_ros2 joint names),
           /tf (odom -> demo_spot/body), payload/status (String JSON, 2 Hz;
           "simulated": true marks this backend as the simulator; also "rtf", per-topic "dropped" / "rejected" input counts,
           "events_suppressed", "cmd_vel_publishers"),
           payload/gas (String JSON + OSHA alarm field, 4 Hz; also sent right
           after any gas control; keys o2_pct lel_pct h2s_ppm co_ppm alarms
           alarm ventilation{on,t_on,tau_s,fraction_remaining,
           residual_fraction} injected_pockets),
           payload/gas/probe_result (String JSON, one per probe request:
           {"id","samples":[{"point","o2_pct","lel_pct","h2s_ppm","co_ppm",
           "alarms"}],"sim_time","wall_t","ventilation","simulated": true,
           "probe": <label>,"body_xy"} or {"id","error","samples": [],
           "sim_time","wall_t","simulated","probe"} when rejected),
           payload/site (String JSON, latched: reliable + TRANSIENT_LOCAL,
           depth 1, published once at startup — shared contract 1; its
           informational "probe" block states the probe rules),
           camera/<name>/image (sensor_msgs/Image rgb8) + camera/<name>/camera_info
           (OmniGraph ROS2CameraHelper, not rclpy; see sim/sensors.py),
           payload/events (String JSON, source "facade", reliable +
           TRANSIENT_LOCAL depth 100: audit records for e-stop / power calls,
           motor-power changes, extra cmd_vel publishers and every world
           control applied or refused; sim/facade_guard.py)

State machine mirrors spot_ros2 at default parameters (facade-contract-v1.md
section 5, get_lease_on_action=True): stand/sit/self_right and cmd_vel
implicitly claim the lease and power on first (power-on silently fails while
e-stopped, so the action itself then fails); power_on/power_off/stop claim
only. On smoke success the process prints the exact marker "SMOKE-OK"
(host/smoke_test.sh greps for it) and exits 0; failures exit 1.

Stdout markers for the host scripts: "RUN-ID run_id=<id>" at start,
"FACADE-READY ... run_id=<id>" once ROS is live, "SMOKE-OK ... run_id=<id>";
<id> is $SPOT_RUN_ID (set per launch by host/run_sim.sh).
"""

import collections
import inspect
import json
import time
import math
import cProfile
import io
import pstats
import signal
import threading
import sys
import os

# Sibling sim/ modules. sensors.py is pure Python at import time, so the
# camera list is validated here, before the ~1 min Kit startup: a typo in
# --cameras fails in under a second instead of after the scene loads.
_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
import launch_args  # noqa: E402  (pure stdlib: this script's command line, T44)

parser = launch_args.build_parser()
args, _unknown = parser.parse_known_args()

import sensors  # noqa: E402
import facade_guard  # noqa: E402  (pure: input checks + audit records, T20-T44)
from battery import SimBattery  # noqa: E402

# T44: nothing on the command line reaches Kit. host/run_sim.sh accepts only
# its own options and this script's, and this second check covers any other
# launcher: a Kit extension switch or setting override could turn on a remote
# viewer bound to every interface, so unknown arguments are refused here,
# before SimulationApp starts.
_launch_error = facade_guard.unknown_args_error(_unknown)
if _launch_error:
    parser.error(_launch_error)

HEADLESS = args.headless == "true"
try:
    GUI_VIEWPORT_RESOLUTION = tuple(int(v) for v in args.gui_res.lower().split("x"))
    if len(GUI_VIEWPORT_RESOLUTION) != 2 or not all(160 <= v <= 3840 for v in GUI_VIEWPORT_RESOLUTION):
        raise ValueError(args.gui_res)
except ValueError:
    parser.error(f"--gui-res: expected WxH within 160..3840, got {args.gui_res!r}")
try:
    CAMERAS = () if args.no_cameras else sensors.parse_camera_list(args.cameras)
except ValueError as _cam_err:
    parser.error(f"--cameras: {_cam_err}")
# 10 Hz in both modes. Measured on the L4 host (2 cameras): headless RTF
# 0.95-1.03 at 10 Hz; GUI RTF 0.78-0.94 at 20 Hz (each GUI render ~32 ms on
# top of ~460 ms/s of physics), so a smoother viewport costs real time. A
# higher --render-hz still publishes cameras at --camera-hz because
# sensors.create_spot_cameras derives frameSkipCount from the ratio.
RENDER_HZ = float(args.render_hz) if args.render_hz else 10.0

# Run id: a per-launch nonce host/run_sim.sh passes as SPOT_RUN_ID. It is
# printed on the RUN-ID, FACADE-READY and SMOKE-OK lines so the host scripts
# wait for THIS launch's readiness line (docker logs of the current
# container), never a stale line left in an old log file.
RUN_ID = "".join(ch for ch in os.environ.get("SPOT_RUN_ID", "") if ch.isalnum() or ch in "-_.")[:64] \
    or f"adhoc-{int(time.time())}-{os.getpid()}"
print(f"RUN-ID run_id={RUN_ID}", flush=True)

# SimulationApp MUST be constructed before any other isaacsim/omni import.
# Only pass options we know SimulationApp accepts.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import carb  # noqa: E402
import carb.settings  # noqa: E402

# The camera helpers run with useSystemTime=True (wall-clock stamps, like
# odometry). Because this app steps physics itself (SimulationManager.step,
# not the timeline), the sim->system time table the helper consults is
# empty, so the plugin warns "getSystemTimeAtTime: no data found ...,
# returning current system time" on EVERY frame (measured 1741 lines in 90 s
# with 2 cameras) -- and the fallback it describes is exactly the stamp we
# want (verified: image stamp within 6 ms of receive time). Raise only that
# plugin's channel to error so real errors stay visible in the log.
carb.settings.get_settings().set_string(
    "/log/channels/isaacsim.core.simulation_manager.plugin", "error")
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.utils.stage import define_prim, set_stage_units, set_stage_up_axis  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents  # noqa: E402
from isaacsim.robot.policy.examples import PolicyEnvConfig, RobotPolicyRunner, get_spot_spec  # noqa: E402
from pxr import PhysxSchema  # noqa: E402

# Enable the ROS 2 bridge BEFORE importing rclpy (bundled jazzy on Ubuntu 24.04).
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402
import rclpy.executors  # noqa: E402
from geometry_msgs.msg import Pose, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

# Sibling sim/ modules (sys.path set up before SimulationApp, above).
try:
    import build_scene as build_scene_mod  # noqa: E402
except Exception as _bs_err:  # module may not be delivered yet; fall back
    build_scene_mod = None
    carb.log_warn(f"isaac_app: sim/build_scene.py unavailable ({_bs_err}); "
                  "falling back to the Grid environment")
from gas_field import (  # noqa: E402
    DEFAULT_VENT_TAU_S,
    PROBE_LABEL,
    UNVENTILATED_HEADSPACE,
    GasField,
    ProbeGeometry,
)
from stance_hold import HeadingHold  # noqa: E402

# ---------------------------------------------------------------------------
# Constants (docs/facade-contract-v1.md + shared conventions)
# ---------------------------------------------------------------------------

ROS_NAMESPACE = "/demo_spot"
TF_PREFIX = "demo_spot/"
ODOM_FRAME = "odom"                    # shared convention: odom -> demo_spot/body
BODY_FRAME = TF_PREFIX + "body"

CONTRACT_JOINT_NAMES = [
    "front_left_hip_x", "front_left_hip_y", "front_left_knee",
    "front_right_hip_x", "front_right_hip_y", "front_right_knee",
    "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee",
    "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
]

# Default stand angles, contract order (noarm_squat.cpp:22).
STAND_POSE = np.array(
    [0.12, 0.72, -1.45, -0.12, 0.72, -1.45, 0.12, 0.72, -1.45, -0.12, 0.72, -1.45],
    dtype=np.float32,
)
# v1 folded "sit" pose, contract order (fixed target array; approximates BD sit).
SIT_POSE = np.array(
    [0.35, 1.30, -2.70, -0.35, 1.30, -2.70, 0.35, 1.30, -2.70, -0.35, 1.30, -2.70],
    dtype=np.float32,
)
# Stand-up = rise vertically over the feet. SIT_POSE puts every foot ~6 cm
# AHEAD of its hip; the policy's default stance has the front feet 4 cm and
# the hind feet 14 cm BEHIND theirs. The v1 joint-space blend between the two
# therefore dragged the loaded feet: it shoved Spot ~0.2 m backward and left
# it crouched (z 0.35 m) and pitched (-0.28 rad) at the policy handover,
# which then fell (measured on the host). Now each foot keeps its x (hip
# frame, sagittal plane) while its z goes to the default-stance height,
# mapped to hip_y / knee with 2-link IK; the policy takes over from a level
# stance and re-places the feet itself.
LEG_UPPER_M = 0.3205             # Spot upper leg (hip_y -> knee)
LEG_LOWER_M = 0.37               # Spot lower leg (knee -> foot)
KNEE_MIN_RAD = -2.75             # inside the knee joint limit (-2.79)
SIT_BLEND_SECONDS = 1.5
STAND_BLEND_SECONDS = 1.5
STAND_SETTLE_SECONDS = 0.4       # minimum hold at the stance before handover
STAND_SETTLE_MAX_SECONDS = 2.5   # hand over anyway after this (logged)
# The policy's PD gains are soft (it holds its stance with offset targets):
# plain targets sag ~0.6 rad under gravity, unevenly front/hind, so the body
# pitched and tipped during the rise. Like the real robot's stand behaviour,
# the scripted stand runs stiff position control -- PhysX drive gains scaled
# up -- and the policy's own gains are restored exactly at handover (or on
# any other exit from the stand-up). An integral term removes what sag is left.
STAND_KP_SCALE = 5.0             # x policy stiffness during the scripted stand
STAND_KD_SCALE = 3.0             # x policy damping during the scripted stand
STAND_KI = 4.0                   # 1/s, integral gain on (path target - q)
STAND_I_LIMIT_RAD = 1.0          # per-joint clamp on the integral offset
STAND_HANDOVER_TOL_RAD = 0.05    # max |q - stand goal| for the handover
STAND_HANDOVER_TILT_RAD = 0.10   # max |roll|, |pitch| for the handover

CMD_VEL_WATCHDOG_S = 0.125  # contract cmd_duration default (spot_ros2.py:383)
ODOM_PERIOD_S = 1.0 / 50.0
STATUS_PERIOD_S = 1.0 / 2.0
GAS_PERIOD_S = 1.0 / 4.0
SMOKE_STEPS = 600
SMOKE_PRINT_EVERY = 100
# Diagnostics-only profiling state (wall clock never drives physics).
PERF_PERIOD_WALL_S = 5.0
perf = {"wall": time.monotonic(), "sim": 0.0}
perf_frames = [0]
perf_t = {"update": 0.0, "spin": 0.0, "policy": 0.0, "cb": 0.0}
perf_steps = [0]
perf_renders = [0]
# T41: the real-time factor in payload/status ("rtf"; the PERF line logs it
# every 5 s), so the agent and the HUD can see a collapse.
rtf_meter = facade_guard.RtfMeter()
# Simulated battery (sim/battery.py): linear drain of sim time, reset on
# payload/battery/reset. v1 drained 95 %/h (amber after ~44 min, red after
# ~54 min -- mid-demo); the default "demo" mode is 25 %/h, floor 30 %.
battery = SimBattery(args.battery_drain)
print(f"BATTERY {battery.describe()}", flush=True)

# Isaac-asset dof-name aliases -> contract names.
# VERIFY-ON-HOST: actual dof names of /Isaac/Robots/BostonDynamics/spot/spot.usd in 6.1
# (older Isaac Spot assets used fl_hx/fl_hy/fl_kn style; newer may use full BD names).
JOINT_ALIASES = {
    "fl_hx": "front_left_hip_x", "fl_hy": "front_left_hip_y", "fl_kn": "front_left_knee",
    "fr_hx": "front_right_hip_x", "fr_hy": "front_right_hip_y", "fr_kn": "front_right_knee",
    "hl_hx": "rear_left_hip_x", "hl_hy": "rear_left_hip_y", "hl_kn": "rear_left_knee",
    "hr_hx": "rear_right_hip_x", "hr_hy": "rear_right_hip_y", "hr_kn": "rear_right_knee",
    "rl_hx": "rear_left_hip_x", "rl_hy": "rear_left_hip_y", "rl_kn": "rear_left_knee",
    "rr_hx": "rear_right_hip_x", "rr_hy": "rear_right_hip_y", "rr_kn": "rear_right_knee",
}

# OSHA alarm thresholds live in gas_field.py (GasField.alarms) — single source.


# ---------------------------------------------------------------------------
# Facade state
# ---------------------------------------------------------------------------

class FacadeState:
    """Mutable robot/facade state shared between ROS callbacks, the physics
    callback, and the render loop (all run on the main thread in standalone)."""

    def __init__(self):
        self.claimed = False
        self.powered = False
        self.standing = False
        self.estopped = False
        self.sim_time = 0.0            # accumulated physics time (s)
        self.step_count = 0            # physics steps taken
        self.base_command = np.zeros(3, dtype=np.float32)   # [vx, vy, yaw_rate]
        self.last_cmd_time = -1.0e9    # sim time of last cmd_vel
        self.first_step = True
        self.request_restart = False   # self_right: re-init policy default state
        self.policy_failed = False
        self.sit_start_time = None
        self.sit_start_positions = None   # isaac dof order
        # Scripted stand-up (like the real robot's stand behaviour): blend
        # joint targets from the current pose to the policy's default stance,
        # then hand over to the RL policy, which was trained from a standing
        # start and cannot recover from the folded sit pose on its own.
        self.stand_pose = None            # isaac dof order, captured after first reset
        self.standing_up = False
        self.stand_start_time = None
        self.stand_start_positions = None
        self.stand_offset = None          # integral compensation (isaac dof order)
        self.legs = None                  # [(hip_x, hip_y, knee) isaac idx] x 4
        self.stand_path = None            # (feet0, goal_feet, goal q) or None
        self.policy_gains = None          # (kp, kd) numpy, saved before scaling
        self.gains_stiff = False
        self.stand_gains_pending = False
        self.hold_positions = None        # isaac dof order; estop/hard hold
        self.body_pose_msg_count = 0
        self.last_pose = None          # (np pos[3], np quat_wxyz[4])
        self.joint_map = None          # contract idx -> isaac dof idx (or None)
        self.joint_map_warned = False


state = FacadeState()
# Stance heading hold while the policy stands on a zero command (the policy
# otherwise creeps into a ~40 deg turn after a walk; sim/stance_hold.py).
heading_hold = HeadingHold()
# gas (GasField) is constructed after build_scene so it can be centered on the
# scene's actual tank footprint.


# ---------------------------------------------------------------------------
# Articulation access adapters
# ---------------------------------------------------------------------------
# The recon notes only evidence articulation.get_world_poses(); dof accessors
# below follow the isaacsim.core.experimental Articulation naming and are
# probed defensively.
# VERIFY-ON-HOST: dof_names / get_dof_positions / get_dof_velocities /
# set_dof_position_targets exist under these names on the 6.1 experimental
# Articulation returned by RobotPolicyRunner.spawn().

def _to_numpy(data):
    if data is None:
        return None
    if hasattr(data, "numpy"):
        data = data.numpy()
    return np.asarray(data, dtype=np.float32).reshape(-1)


def art_dof_names(art):
    for attr in ("dof_names", "joint_names"):
        try:
            names = getattr(art, attr, None)
            if callable(names):
                names = names()
            if names is not None and len(names) > 0:
                return [str(n) for n in names]
        except Exception:
            continue
    return None


def art_dof_positions(art):
    for attr in ("get_dof_positions", "get_joint_positions"):
        fn = getattr(art, attr, None)
        if fn is not None:
            try:
                return _to_numpy(fn())
            except Exception:
                continue
    return None


def art_dof_velocities(art):
    for attr in ("get_dof_velocities", "get_joint_velocities"):
        fn = getattr(art, attr, None)
        if fn is not None:
            try:
                return _to_numpy(fn())
            except Exception:
                continue
    return None


def art_set_dof_position_targets(art, targets):
    arr = np.asarray(targets, dtype=np.float32).reshape(1, -1)
    for attr in ("set_dof_position_targets", "set_joint_position_targets"):
        fn = getattr(art, attr, None)
        if fn is not None:
            try:
                fn(arr)
                return True
            except Exception:
                continue
    return False


def build_joint_map(dof_names):
    """Map contract joint index -> isaac dof index. Never assumes ordering."""
    contract_to_isaac = [None] * len(CONTRACT_JOINT_NAMES)
    contract_index = {n: i for i, n in enumerate(CONTRACT_JOINT_NAMES)}
    for isaac_idx, raw in enumerate(dof_names):
        base = raw.split("/")[-1].split(":")[-1].strip().lower()
        canonical = JOINT_ALIASES.get(base, base)
        idx = contract_index.get(canonical)
        if idx is not None:
            contract_to_isaac[idx] = isaac_idx
    return contract_to_isaac


def contract_pose_to_isaac(pose_contract, joint_map, n_dofs):
    out = np.zeros(n_dofs, dtype=np.float32)
    for c_idx, i_idx in enumerate(joint_map):
        if i_idx is not None and i_idx < n_dofs:
            out[i_idx] = pose_contract[c_idx]
    return out


# ---------------------------------------------------------------------------
# Stage / scene / robot setup
# ---------------------------------------------------------------------------

set_stage_up_axis("Z")
set_stage_units(meters_per_unit=1.0)

# The app owns the physics scene; build_scene builds only environment prims.
# VERIFY-ON-HOST: build_scene.py must not define a second PhysicsScene.
define_prim("/World/PhysicsScene", "PhysicsScene")
SimulationManager.set_physics_sim_device(args.device)

def _fallback_scene():
    """Minimal Grid environment (verbatim spot_standalone.py pattern) so the
    app and --smoke still run when sim/build_scene.py is absent or fails."""
    from isaacsim.storage.native import get_assets_root_path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("isaac_app: could not find Isaac Sim assets root")
    else:
        prim = define_prim("/World/Ground", "Xform")
        prim.GetReferences().AddReference(
            assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")
    # Defaults consistent with the GasField default tank (center (0,0), r=6):
    # spawn outside the footprint facing the entry (+Y), entry at the wall,
    # no interior waypoints.
    return {"spawn_xy": (0.0, -9.0), "spawn_yaw": math.pi / 2.0,
            "entry_xy": (0.0, -6.0), "interior_waypoints": []}


if build_scene_mod is None:
    scene_info = _fallback_scene()
else:
    # Signature is inspected (not probed via TypeError) so an internal
    # TypeError inside build_scene() cannot trigger a second, duplicate build.
    try:
        _bs_params = inspect.signature(build_scene_mod.build_scene).parameters
        _accepts_scene_usd = ("scene_usd" in _bs_params or any(
            p.kind is inspect.Parameter.VAR_KEYWORD for p in _bs_params.values()))
    except (TypeError, ValueError):
        _accepts_scene_usd = False
    if _accepts_scene_usd:
        scene_info = build_scene_mod.build_scene(scene_usd=args.scene)
    else:
        if args.scene:
            carb.log_warn("isaac_app: build_scene() does not accept scene_usd; --scene ignored")
        scene_info = build_scene_mod.build_scene()

spawn_xy = tuple(scene_info.get("spawn_xy", (0.0, -9.0)))
spawn_yaw = float(scene_info.get("spawn_yaw", 0.0))
entry_xy = tuple(scene_info.get("entry_xy", (0.0, -6.0)))
interior_waypoints = [tuple(w) for w in scene_info.get("interior_waypoints", [])]
tank_center_xy = tuple(scene_info.get("tank_center_xy", (0.0, 0.0)))
tank_radius = float(scene_info.get("tank_radius", 6.0))
# The doorway is where the entry route crosses the shell. build_scene reports
# it (doorway_xy) plus its outward normal; if a scene lacks them, derive both
# from the geometry that is always present (entry point outside the door,
# tank center) so the site contract never ships a guessed constant.
door_xy = tuple(scene_info.get("doorway_xy", entry_xy))
door_normal_yaw = float(scene_info.get(
    "door_normal_yaw",
    math.atan2(door_xy[1] - tank_center_xy[1], door_xy[0] - tank_center_xy[0])))
carb.log_info(
    f"isaac_app: scene ready — spawn={spawn_xy} entry={entry_xy} "
    f"waypoints={len(interior_waypoints)}"
)


def site_payload():
    """Shared contract 1 (/demo_spot/payload/site): plain JSON lists/floats
    so non-ROS consumers (HUD, agent fallback defaults file) parse it as-is."""
    def xy(p):
        return [round(float(p[0]), 4), round(float(p[1]), 4)]
    return {
        "spawn_xy": xy(spawn_xy),
        "spawn_yaw": round(spawn_yaw, 6),
        "entry_xy": xy(entry_xy),
        "door_xy": xy(door_xy),
        "door_normal_yaw": round(door_normal_yaw, 6),
        "interior_waypoints": [xy(w) for w in interior_waypoints],
        "tank_center_xy": xy(tank_center_xy),
        "tank_radius": round(tank_radius, 4),
        # Informational (consumers that validate the site ignore it): the
        # rules payload/gas/probe applies, so a client can aim within them.
        "probe": probe_geom.limits(),
    }


# Center the gas model on the scene's tank (GasField defaults match the
# build_scene tank: center (0,0), radius 6 m). The facade adds the
# unventilated headspace (opt-in in GasField) so the storyline's pre-entry
# probe fails until forced ventilation has run; see sim/gas_field.py.
GAS_INITIAL_VENTILATED = args.gas_initial == "ventilated"
gas = GasField(seed=7, center=tank_center_xy, radius=tank_radius,
               headspace=UNVENTILATED_HEADSPACE)
gas.reset_ventilation(ventilated=GAS_INITIAL_VENTILATED, t=0.0)
# Gas-probe reach / line-of-sight rules against the same scene geometry
# (door size from build_scene when it loaded; 1.30 x 1.30 m otherwise).
probe_geom = ProbeGeometry(
    tank_center=tank_center_xy, tank_radius=tank_radius, door_xy=door_xy,
    door_normal_yaw=door_normal_yaw,
    door_width=getattr(build_scene_mod, "DOOR_WIDTH", 1.30),
    door_height=getattr(build_scene_mod, "DOOR_HEIGHT", 1.30))
print(f"PROBE {PROBE_LABEL}: reach {probe_geom.max_reach} m (horizontal) from the body front "
      f"({probe_geom.front_offset} m ahead of the body center), body pose required, "
      f"line of sight through the {2 * probe_geom.half_width:.2f} x {probe_geom.door_height:.2f} m "
      "doorway only", flush=True)
print(f"GAS initial={args.gas_initial} headspace=on "
      f"residual_fraction={gas.residual_fraction} tau_default={DEFAULT_VENT_TAU_S}s", flush=True)

# ---------------------------------------------------------------------------
# Actuator mode (--actuators)
# ---------------------------------------------------------------------------
# newton (stock 6.1): runtime._build_actuators() turns the Spot env config's
#   DelayedPDActuator (8 hips) + RemotizedPDActuator (4 knees) groups into ONE
#   Newton Actuator PER DOF (utils/newton_actuators.py loops spec.joints), and
#   ArticulationActuators.step_actuators() runs all 12 on EVERY PHYSICS_PRE_STEP
#   (not per policy tick), each doing 5 warp.launch() calls (delay read, PD,
#   clamp, scatter-add, delay write) = 60 Python-packed launches per 2 ms step.
#   Measured: 8.6 of 9.6 s cProfile time per 2 s of sim.
# fast: the runtime's own path for ImplicitActuator groups. _build_actuators()
#   returns None, so no actuator callback exists and PhysX runs the same PD
#   (stiffness/damping that initialize_articulation() already wrote to the
#   drives) inside its solver; we only author the torque limits the Newton
#   models would have clamped to. Fidelity deltas vs newton: implicit solver
#   PD instead of explicit torque; no 4-step (8 ms) target delay (training
#   sampled delay in [0, 4] steps, so 0 is in-distribution); the knee's
#   position-dependent limit (lookup: >=71 N*m over -2.2..-0.6 rad, 113 N*m at
#   the stand angle) becomes a constant evaluated at the default knee angle.
import isaacsim.robot.policy.examples.runtime as _policy_runtime  # noqa: E402
from omni.physics.core import get_physics_simulation_interface  # noqa: E402

_newton_build_actuators = _policy_runtime._build_actuators
_PHYSX_DRIVE_CLASSES = {"IdealPDActuator", "DelayedPDActuator", "RemotizedPDActuator"}


def _build_actuators_as_physx_drives(articulation, env_config, policy_joint_names):
    """Drop-in for runtime._build_actuators: keep PD on the PhysX drives."""
    specs = env_config.actuator_model_specs(policy_joint_names)
    unsupported = sorted({s.class_name for s in specs} - _PHYSX_DRIVE_CLASSES)
    if unsupported:  # DC-motor / actuator-net curves have no PhysX drive equivalent
        carb.log_warn(f"isaac_app: --actuators fast cannot map {unsupported}; using Newton actuators")
        return _newton_build_actuators(articulation, env_config, policy_joint_names)
    dof_names = [str(n) for n in articulation.dof_names]
    props = env_config.joint_properties(policy_joint_names)
    default_pos = dict(zip(props.joint_names, props.default_positions))
    indices, limits = [], []
    for s in specs:
        for i, joint in enumerate(s.joints):
            if s.class_name == "RemotizedPDActuator":
                lut = np.asarray(s.joint_parameter_lookup, dtype=np.float64)  # (angle, ratio, torque)
                limit = float(np.interp(default_pos[joint], lut[:, 0], lut[:, 2]))
            else:
                limit = float(s.effort_limit[i])
            indices.append(dof_names.index(joint))
            limits.append(limit)
    if indices:
        # Same call + flush pattern as runtime.initialize_articulation().
        articulation.set_dof_max_efforts(limits, dof_indices=indices)
        get_physics_simulation_interface().flush_changes()
    print(f"ACTUATORS mode=fast (PhysX drives) device={SimulationManager.get_physics_sim_device()} "
          f"max_effort={dict(zip((dof_names[i] for i in indices), (round(v, 1) for v in limits)))}",
          flush=True)
    return None  # runtime treats None as "every group uses an engine drive"


if args.actuators == "fast":
    # Module-global lookup at call time (runtime._build_runtime), so patching
    # before the first restart_from_default_state() is sufficient.
    # VERIFY-ON-HOST: PERF policy_ms/step and cb_ms/tick drop; SMOKE odom z stays ~0.5 m.
    _policy_runtime._build_actuators = _build_actuators_as_physx_drives
else:
    print(f"ACTUATORS mode=newton device={SimulationManager.get_physics_sim_device()}", flush=True)

spec = get_spot_spec()
# VERIFY-ON-HOST: 0.7 m spawn height clears the ground for the default policy init.
# Face the doorway at spawn: build_scene's spawn_yaw -> WXYZ yaw quaternion
# (RobotPolicyRunner orientation kwarg per docs/isaac-api-notes.md, runtime.py).
spot = RobotPolicyRunner(spec, prim_path="/World/Spot",
                         position=(spawn_xy[0], spawn_xy[1], 0.7),
                         orientation=(math.cos(spawn_yaw / 2.0), 0.0, 0.0,
                                      math.sin(spawn_yaw / 2.0)))
spot.spawn()

timing = PolicyEnvConfig.from_file(spec.engines["physx"].env_config_path).timing
RenderingManager.set_dt(timing.render_interval * timing.physics_dt)
SimulationManager.set_physics_dt(spot.physics_dt)
# Fabric mode turns off /physics/updateToUsd (+velocities/forces). Measured on
# the L4 host: with USD write-back every 2 ms step cost ~8.5 ms of wall time.
# Rendering then reads poses from Fabric, updated on render ticks only.
USE_FABRIC = args.fabric == "true"
if USE_FABRIC:
    SimulationManager.enable_fabric(True)


# ---------------------------------------------------------------------------
# Visuals: body cameras, nose spotlight, chase camera (sim/sensors.py)
# ---------------------------------------------------------------------------
# Replaces the v1 fixed overwatch camera (/World/BodyCamera on camera/rgb):
# cameras now ride on the body link under spot_ros2 topic names. Everything
# here is non-critical: a failure logs and the facade keeps running.

_stage = omni.usd.get_context().get_stage()
if CAMERAS:
    try:
        sensors.create_spot_cameras(
            _stage, "/World/Spot", CAMERAS, rate_hz=args.camera_hz, render_hz=RENDER_HZ,
            namespace=ROS_NAMESPACE, frame_prefix=TF_PREFIX,
            log=lambda m: print(m, flush=True))
    except Exception as error:  # noqa: BLE001
        print(f"CAMERAS FAILED (continuing without video): {error}", flush=True)
else:
    print("CAMERAS enabled=none", flush=True)

payload_lights = None
try:
    sensors.add_nose_spotlight(_stage, "/World/Spot")
    # Off out in the sunlight, on near the tank (sensors.payload_light_scale).
    payload_lights = sensors.PayloadLights(_stage, "/World/Spot", tank_center_xy, tank_radius)
except Exception as error:  # noqa: BLE001
    carb.log_warn(f"isaac_app: nose spotlight failed (continuing): {error}")

chase_cam = None
try:
    chase_cam = sensors.ChaseCamera(_stage, tank_center_xy, tank_radius,
                                    door_normal_yaw=door_normal_yaw, view=args.view,
                                    keepouts=scene_info.get("camera_keepouts"))
except Exception as error:  # noqa: BLE001
    carb.log_warn(f"isaac_app: chase camera failed (continuing): {error}")


_ui_state = {"mode": None, "report_at": None}


def _ui_report():
    """One line, a few seconds after a UI mode change: what actually took
    effect (set_fullscreen is applied by the window system asynchronously)."""
    _ui_state["report_at"] = None
    try:
        import omni.appwindow

        w = omni.appwindow.get_default_app_window()
        print(f"UI check mode={_ui_state['mode']} fullscreen={w.is_fullscreen()} "
              f"hideUi={carb.settings.get_settings().get('/app/window/hideUi')} "
              f"window={w.get_width()}x{w.get_height()}", flush=True)
    except Exception as error:  # noqa: BLE001 - diagnostics
        print(f"UI check failed: {error}", flush=True)


def _set_ui_mode(mode):
    """GUI only: presentation = fullscreen viewport only; editor = the stock
    Isaac editor layout. Cosmetic: logs and returns on any failure.

    Uses Kit's own F11 path, verified in the 6.1.0 container:
    extscache/omni.kit.ui.actions-1.0.5+00c488ae/omni/kit/ui/actions/actions.py
    on_fullscreen(new_state) saves the workspace (ui.Workspace.dump_workspace),
    calls omni.appwindow.get_default_app_window().set_fullscreen() and sets
    /app/window/hideUi, and restores the workspace on the way back. With
    hideUi the viewport window fills the main window without title bar and
    hides its Menubar and Axis layers
    (extscache/omni.kit.viewport.window-110.0.1+00c488ae/omni/kit/viewport/
    window/window.py __hide_ui, docs/Settings.md). A fullscreen X window also
    covers the GNOME top bar and dock. If that action is not registered, the
    same two calls are made directly (omni.appwindow _appwindow.pyi
    set_fullscreen / is_fullscreen; omni.ui _ui.pyi Workspace).
    hideUi leaves one overlay: the stage-units badge ("m", bottom left) of
    omni.usd.metrics.assembler.ui, hidden via its per-viewport setting
    /app/viewport/<viewport id>/hud/metrics/assembler/visible
    (extscache/omni.usd.metrics.assembler.ui-110.1.1+110.1.0.cp312.u7f4/omni/
    metrics/assembler/ui/constants.py, tools.py get_per_viewport_setting_path).
    """
    if HEADLESS or mode not in sensors.UI_MODES:
        return
    try:
        import omni.appwindow
        import omni.kit.actions.core
        from omni import ui

        settings = carb.settings.get_settings()
        registry = omni.kit.actions.core.get_action_registry()
        want = mode == "presentation"
        path = "action"
        for key, value in sensors.KIT_UI_SETTINGS.items():  # before the action: no Esc exit
            settings.set(key, value)
        if registry is not None and registry.get_action("omni.kit.ui.actions", "toggle_fullscreen"):
            registry.execute_action("omni.kit.ui.actions", "toggle_fullscreen", want)
        else:
            path = "direct"
            omni.appwindow.get_default_app_window().set_fullscreen(want)
            settings.set("/app/window/hideUi", want)
        try:
            import omni.kit.viewport.utility as viewport_utility
            vp = viewport_utility.get_active_viewport()
            if vp is not None:
                settings.set_bool(f"/app/viewport/{vp.id}/hud/metrics/assembler/visible", not want)
        except Exception as error:  # noqa: BLE001 - cosmetic
            print(f"UI units badge toggle failed (continuing): {error}", flush=True)
        _ui_state["mode"] = mode
        _ui_state["report_at"] = time.monotonic() + 5.0  # fullscreen lands asynchronously
        app_window = omni.appwindow.get_default_app_window()
        if not want:
            # Leaving fullscreen restores the pre-fullscreen 1440x863 window
            # (host 2026-09-27); the editor wants the whole desktop.
            app_window.maximize_window()
        shown = sorted(str(getattr(w, "title", "?")) for w in ui.Workspace.get_windows()
                       if getattr(w, "visible", False))
        print(f"UI mode={mode} via={path} fullscreen={app_window.is_fullscreen()} "
              f"hideUi={settings.get('/app/window/hideUi')} "
              f"window={app_window.get_width()}x{app_window.get_height()} "
              f"visible_windows={shown}", flush=True)
    except Exception as error:  # noqa: BLE001 - cosmetic
        print(f"UI mode={mode} failed (continuing with the current layout): {error}", flush=True)


def _configure_viewport():
    """GUI: show the presenter camera, pin the render resolution and apply
    --ui. Headless: stop rendering the hidden 1280x720 viewport nobody can
    see (measured on the host: the headless app still has an active viewport
    with updates_enabled=True), which frees the render budget the body
    cameras need."""
    try:
        import omni.kit.viewport.utility as viewport_utility
        viewport = viewport_utility.get_active_viewport()
        if viewport is None:
            print("VIEWPORT none", flush=True)
            return
        if HEADLESS:
            viewport.updates_enabled = False
            print("VIEWPORT headless: hidden viewport rendering disabled", flush=True)
        else:
            if chase_cam is not None:
                viewport.camera_path = sensors.ChaseCamera.PATH
            # Pin the render resolution: by default it follows the window,
            # so maximizing Kit on a large DCV client would multiply the
            # per-render cost and pull the sim below real time mid-demo.
            try:
                viewport.resolution = GUI_VIEWPORT_RESOLUTION
            except Exception as error:  # noqa: BLE001
                print(f"VIEWPORT resolution pin failed (continuing): {error}", flush=True)
            print(f"VIEWPORT gui: camera={viewport.camera_path} "
                  f"resolution={getattr(viewport, 'resolution', '?')} "
                  f"view={chase_cam.view if chase_cam is not None else 'none'}", flush=True)
    except Exception as error:  # noqa: BLE001 - cosmetic
        print(f"VIEWPORT configure failed (continuing): {error}", flush=True)
    _set_ui_mode(args.ui)


# ---------------------------------------------------------------------------
# ROS 2 facade
# ---------------------------------------------------------------------------

rclpy.init()
node = rclpy.create_node("spot_facade", namespace=ROS_NAMESPACE)

pub_odom = node.create_publisher(Odometry, "odometry", 10)
pub_joints = node.create_publisher(JointState, "joint_states", 10)
pub_tf = node.create_publisher(TFMessage, "/tf", 10)  # TF is global, not namespaced
pub_status = node.create_publisher(String, "payload/status", 10)
pub_gas = node.create_publisher(String, "payload/gas", 10)
pub_probe = node.create_publisher(String, "payload/gas/probe_result", 10)

# Site geometry, latched (shared contract 1): reliable + TRANSIENT_LOCAL,
# depth 1, published once. DDS keeps the sample for late joiners, so the
# mission/agent/HUD can start in any order after the sim. Subscribers must
# request TRANSIENT_LOCAL too or they only see samples sent after they join.
SITE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub_site = node.create_publisher(String, "payload/site", SITE_QOS)
_site_json = json.dumps(site_payload())
pub_site.publish(String(data=_site_json))
print(f"SITE published {ROS_NAMESPACE}/payload/site {_site_json}", flush=True)

# The facade's own audit records (source "facade", sim/facade_guard.py): same
# record shape and QoS as agent/robot_client.py (reliable + TRANSIENT_LOCAL,
# depth 100), so the HUD and the audit export see them. Queued from any
# thread, published from the sim loop, rate-bounded by FacadeAudit.
EVENTS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub_events = node.create_publisher(String, "payload/events", EVENTS_QOS)
audit = facade_guard.FacadeAudit()
# Per-topic rate limits and drop / refusal counters for every incoming
# control topic, reported in payload/status (T23, T42).
inputs = facade_guard.InputGuard()


def _audit(record):
    if record is not None:
        audit.record(*record, sim_time=state.sim_time)


def _publishers_of(topic):
    """Publisher names on a topic (relative to the namespace), or None."""
    try:
        return facade_guard.publisher_names(node.get_publishers_info_by_topic(topic))
    except Exception:  # noqa: BLE001 - attribution only, never fatal
        return None


def _auto_claim(power=False):
    """spot_ros2 default get_lease_on_action=True: actions claim (and power
    on) before running; claim/power failures are ignored and the wrapped
    command surfaces the real error (facade-contract-v1.md section 5).
    Power-on silently fails while e-stopped (no e-stop authority)."""
    state.claimed = True
    if power and not state.estopped:
        state.powered = True


def on_cmd_vel(msg):
    # Twist semantics per contract: linear.x, linear.y, angular.z only; each
    # message is a one-shot command kept alive by the watchdog window.
    # velocity_cmd is in the ClaimAndPowerDecorator set -> implicit claim+power.
    if not inputs.allow("cmd_vel"):  # T42: over the rate limit = dropped, counted
        return
    cmd, why = facade_guard.check_twist((msg.linear.x, msg.linear.y, msg.linear.z),
                                        (msg.angular.x, msg.angular.y, msg.angular.z))
    if cmd is None:
        # T20 fail closed: a non-finite or out-of-range command never reaches
        # the policy, stops Spot (as srv_stop does) and claims nothing.
        inputs.reject("cmd_vel")
        state.base_command = np.zeros(3, dtype=np.float32)
        state.last_cmd_time = -1.0e9
        node.get_logger().warning(f"cmd_vel rejected (Spot stopped): {why}",
                                  throttle_duration_sec=5.0)
        return
    before = (state.powered, state.estopped)
    _auto_claim(power=True)
    _audit(facade_guard.service_event("cmd_vel", before, (state.powered, state.estopped),
                                      True, "implicit claim + power-on"))
    state.base_command = np.array(cmd, dtype=np.float32)
    state.last_cmd_time = state.sim_time


def on_body_pose(msg):
    # v1 stub: the real driver applies position.z + orientation via mobility
    # params on the next stand. We only log (and refuse malformed poses, T20).
    if not inputs.allow("body_pose"):  # T42
        return
    p, o = msg.position, msg.orientation
    why = facade_guard.check_body_pose((p.x, p.y, p.z), (o.w, o.x, o.y, o.z))
    if why:
        inputs.reject("body_pose")
        node.get_logger().warning(f"body_pose rejected: {why}", throttle_duration_sec=5.0)
        return
    state.body_pose_msg_count += 1
    if state.body_pose_msg_count <= 3:
        node.get_logger().info(
            f"body_pose received (v1 stub, not applied): z={msg.position.z:.3f}")


# Simulated gas controls. ROS callbacks run on the executor thread, while the
# sim thread samples the field for payload/gas, so they only enqueue here;
# apply_gas_commands() runs them on the sim thread each tick (<= 20 ms of sim
# time later), which also stamps them with the physics clock.
GAS_CMD_QUEUE_MAX = 64
PROBE_MAX_POINTS = 16
# Probe reach / occlusion / pose rules: probe_geom (sim/gas_field.py
# ProbeGeometry): an extendable gas probe (simulated; arm-mounted on the
# real robot) reaching 1.1 m horizontally from the body's front, only
# through the doorway opening, and only with a known body pose.
# Queue items: (kind, raw JSON text or None if oversized, publishers or None);
# deque ops are thread-safe.
_gas_cmds = collections.deque()


def _enqueue(queue, kind, topic):
    """ROS callback for a String control: rate limit, cap the queue and the
    payload size, then queue it for the sim thread (T23, T42). World
    controls also record the topic's publishers while the sender exists."""
    def _cb(msg):
        if not inputs.allow(topic):
            return
        if len(queue) >= GAS_CMD_QUEUE_MAX:
            inputs.drop(topic)
            node.get_logger().error(f"payload/{topic} dropped: {GAS_CMD_QUEUE_MAX} commands pending",
                                    throttle_duration_sec=5.0)
            return
        publishers = None if kind == "probe" else _publishers_of(f"payload/{topic}")
        queue.append((kind, facade_guard.bounded(msg.data), publishers))
    return _cb


node.create_subscription(Twist, "cmd_vel", on_cmd_vel, 10)
node.create_subscription(Pose, "body_pose", on_body_pose, 10)
for _kind in ("inject", "clear", "ventilation", "probe"):
    node.create_subscription(String, f"payload/gas/{_kind}",
                             _enqueue(_gas_cmds, _kind, f"gas/{_kind}"), 10)

# Presenter / sim controls (String), applied on the sim thread like the gas
# controls: payload/view (camera + GUI layout), payload/battery/reset.
_ctl_cmds = collections.deque()
node.create_subscription(String, "payload/view", _enqueue(_ctl_cmds, "view", "view"), 10)
node.create_subscription(String, "payload/battery/reset",
                         _enqueue(_ctl_cmds, "battery_reset", "battery/reset"), 10)


# --- Trigger services (semantics mirror docs/facade-contract-v1.md) ---------

def _begin_sit():
    state.standing = False
    state.standing_up = False
    state.sit_start_time = state.sim_time
    state.sit_start_positions = art_dof_positions(spot.articulation)


def srv_claim(_req, res):
    if state.claimed:
        res.success, res.message = True, "We already claimed the lease"
    else:
        state.claimed = True
        res.success, res.message = True, "Claimed lease"
    return res


def srv_release(_req, res):
    state.claimed = False
    res.success, res.message = True, "Released lease"
    return res


def srv_power_on(_req, res):
    _auto_claim()  # passive decorator set: claim only (contract section 5)
    if state.estopped:
        res.success, res.message = False, "Exception while powering on: e-stop is asserted"
    elif state.powered:
        res.success, res.message = True, "Was already powered on"
    else:
        state.powered = True
        res.success, res.message = True, "Powered on"
    return res


def srv_power_off(_req, res):
    # safe_power_off: sit first, then cut motor power (claim-only decoration).
    _auto_claim()
    if state.standing:
        _begin_sit()
    state.powered = False
    res.success, res.message = True, "Safe power off (sat down, motors off)"
    return res


def srv_stand(_req, res):
    _auto_claim(power=True)  # decorated action: implicit claim + power_on
    if not state.powered:
        # power_on failed silently (e-stop asserted); the action surfaces it.
        res.success, res.message = False, "Exception while standing: motor power is off (e-stop asserted)"
    else:
        state.sit_start_time = None
        state.hold_positions = None
        state.base_command = np.zeros(3, dtype=np.float32)
        if not state.standing and not state.standing_up:
            _p = art_dof_positions(spot.articulation) if spot.articulation is not None else None
            if _p is None or state.stand_pose is None:
                state.standing = True  # no pose data yet: enable the policy directly
            else:
                state.stand_start_positions = np.array(_p, dtype=np.float32).copy()
                state.stand_offset = np.zeros_like(state.stand_start_positions)
                state.stand_path = plan_stand_path(state.stand_start_positions)
                state.stand_gains_pending = True  # applied on the sim thread
                state.stand_start_time = state.sim_time
                state.standing_up = True  # policy takes over when the blend completes
        res.success, res.message = True, "Standing"
    return res


def srv_sit(_req, res):
    _auto_claim(power=True)  # decorated action: implicit claim + power_on
    if not state.powered:
        res.success, res.message = False, "Exception while sitting: motor power is off (e-stop asserted)"
    else:
        _begin_sit()
        res.success, res.message = True, "Sitting"
    return res


def srv_stop(_req, res):
    _auto_claim()  # claim only, no power change (contract section 5)
    state.base_command = np.zeros(3, dtype=np.float32)
    state.last_cmd_time = -1.0e9
    res.success, res.message = True, "Stopped"
    return res


def srv_self_right(_req, res):
    _auto_claim(power=True)  # decorated action: implicit claim + power_on
    if not state.powered:
        res.success, res.message = False, "Exception while self-righting: motor power is off (e-stop asserted)"
    else:
        # VERIFY-ON-HOST: restart_from_default_state may also reset the root
        # pose to the spawn pose (acceptable for an explicit recovery action).
        state.request_restart = True
        state.standing = True
        state.standing_up = False  # a stand-up in progress is superseded
        state.sit_start_time = None
        res.success, res.message = True, "Self-right requested (policy re-initialized)"
    return res


def srv_estop_hard(_req, res):
    state.estopped = True
    state.powered = False
    state.standing = False
    state.standing_up = False
    state.sit_start_time = None
    state.hold_positions = art_dof_positions(spot.articulation)  # hold, no policy
    state.base_command = np.zeros(3, dtype=np.float32)
    res.success, res.message = True, "Hard e-stop asserted (motor power cut)"
    return res


def srv_estop_gentle(_req, res):
    state.estopped = True
    if state.standing:
        _begin_sit()   # settle...
    state.powered = False  # ...then cut
    state.base_command = np.zeros(3, dtype=np.float32)
    res.success, res.message = True, "Gentle e-stop asserted (settled, motor power cut)"
    return res


def srv_estop_release(_req, res):
    state.estopped = False
    state.hold_positions = None
    res.success, res.message = True, "E-stop released (motors stay off until next power_on/action)"
    return res


def _audited(name, cb):
    """T20: record e-stop / power calls and any motor-power or e-stop change."""
    def _cb(req, res):
        before = (state.powered, state.estopped)
        res = cb(req, res)
        _audit(facade_guard.service_event(name, before, (state.powered, state.estopped),
                                          res.success, res.message))
        return res
    return _cb


for name, cb in [
    ("claim", srv_claim), ("release", srv_release),
    ("power_on", srv_power_on), ("power_off", srv_power_off),
    ("stand", srv_stand), ("sit", srv_sit), ("stop", srv_stop),
    ("self_right", srv_self_right),
    ("estop/hard", srv_estop_hard), ("estop/gentle", srv_estop_gentle),
    ("estop/release", srv_estop_release),
]:
    node.create_service(Trigger, name, _audited(name, cb))

# T20: a second publisher on cmd_vel can drive Spot as well as the agent
# (last writer wins). Polled on the executor thread (a wall-clock node timer).
cmd_vel_watch = facade_guard.PublisherWatch(f"{ROS_NAMESPACE}/cmd_vel")


def _watch_cmd_vel():
    names = _publishers_of("cmd_vel")
    if names is not None:
        _audit(cmd_vel_watch.update(names))


node.create_timer(facade_guard.PUBLISHER_WATCH_PERIOD_S, _watch_cmd_vel)


# ---------------------------------------------------------------------------
# Leg kinematics for the scripted sit / stand (sagittal 2-link model)
# ---------------------------------------------------------------------------

def leg_fk(hip_y, knee):
    """Foot (x forward, z up) relative to the hip for hip_y / knee angles."""
    a = LEG_UPPER_M * math.sin(hip_y) + LEG_LOWER_M * math.sin(hip_y + knee)
    b = LEG_UPPER_M * math.cos(hip_y) + LEG_LOWER_M * math.cos(hip_y + knee)
    return -a, -b


def leg_ik(x, z):
    """Inverse of leg_fk (knee bent backward); unreachable targets clamp."""
    l1, l2 = LEG_UPPER_M, LEG_LOWER_M
    d = min(max(math.hypot(x, z), abs(l1 - l2) + 1e-3), l1 + l2 - 1e-3)
    cos_knee = (l1 * l1 + l2 * l2 - d * d) / (2.0 * l1 * l2)
    knee = -(math.pi - math.acos(min(1.0, max(-1.0, cos_knee))))
    cos_hip = (l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d)
    hip_y = math.atan2(-x, -z) + math.acos(min(1.0, max(-1.0, cos_hip)))
    return hip_y, max(knee, KNEE_MIN_RAD)


def set_stand_gains(stiff):
    """Scale the PhysX drive gains for the scripted stand (stiff=True) or
    restore the policy's own gains (stiff=False). Idempotent; never raises."""
    if stiff == state.gains_stiff:
        return
    art = spot.articulation
    try:
        if state.policy_gains is None:
            kp, kd = art.get_dof_gains()
            state.policy_gains = (_to_numpy(kp).reshape(1, -1).copy(), _to_numpy(kd).reshape(1, -1).copy())
            print(f"GAINS policy kp={np.round(state.policy_gains[0], 1).tolist()} "
                  f"kd={np.round(state.policy_gains[1], 2).tolist()}", flush=True)
        kp, kd = state.policy_gains
        if stiff:
            art.set_dof_gains(stiffnesses=kp * STAND_KP_SCALE, dampings=kd * STAND_KD_SCALE)
        else:
            art.set_dof_gains(stiffnesses=kp, dampings=kd)
        get_physics_simulation_interface().flush_changes()
        state.gains_stiff = stiff
    except Exception as error:  # noqa: BLE001 - fall back to the soft gains
        carb.log_warn(f"isaac_app: set_stand_gains({stiff}) failed: {error}")
        if not stiff:
            state.gains_stiff = False


def _ensure_legs():
    """Per-leg isaac dof indices (hip_x, hip_y, knee); None until the joint
    map and the captured default stance are available."""
    if state.legs is not None:
        return state.legs
    _ensure_joint_map()
    if state.joint_map is None or state.stand_pose is None:
        return None
    legs = []
    for leg in ("front_left", "front_right", "rear_left", "rear_right"):
        idx = [state.joint_map[CONTRACT_JOINT_NAMES.index(f"{leg}_{j}")]
               for j in ("hip_x", "hip_y", "knee")]
        if any(i is None or i >= len(state.stand_pose) for i in idx):
            return None
        legs.append(tuple(idx))
    state.legs = legs
    return legs


def feet_of(q):
    return [leg_fk(float(q[hy]), float(q[kn])) for (_hx, hy, kn) in state.legs]


def plan_stand_path(q0):
    """(feet0, goal_feet, goal): keep each foot's hip-relative x, raise the
    body to the default-stance foot height; hip_x goes to its default.
    None if the leg map is unavailable (then the v1 joint blend is used)."""
    if _ensure_legs() is None or len(q0) != len(state.stand_pose):
        return None
    feet0 = feet_of(q0)
    goal_feet = [(x0, z_default) for (x0, _z0), (_xd, z_default)
                 in zip(feet0, feet_of(state.stand_pose))]
    goal = state.stand_pose.copy()
    for (_hx, hy, kn), (x, z) in zip(state.legs, goal_feet):
        goal[hy], goal[kn] = leg_ik(x, z)
    return feet0, goal_feet, goal


def foot_path_targets(q0, feet0, goal_feet, goal, alpha):
    """Joint targets at blend fraction alpha: each foot moves on a straight
    line from feet0 to goal_feet (hip frame); every other dof (hip_x)
    blends linearly from q0 to goal."""
    out = (1.0 - alpha) * q0 + alpha * goal
    for (_hx, hy, kn), (x0, z0), (x1, z1) in zip(state.legs, feet0, goal_feet):
        out[hy], out[kn] = leg_ik(x0 + alpha * (x1 - x0), z0 + alpha * (z1 - z0))
    return out


def body_tilt():
    """(roll, pitch) of the body in rad, or None."""
    _pos, quat = read_body_pose()
    if quat is None:
        return None
    w, x, y, z = (float(v) for v in quat)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


# ---------------------------------------------------------------------------
# Physics-step callback (must never raise)
# ---------------------------------------------------------------------------

def _ensure_joint_map():
    if state.joint_map is not None:
        return
    names = art_dof_names(spot.articulation)
    if names is None:
        return
    state.joint_map = build_joint_map(names)
    missing = [CONTRACT_JOINT_NAMES[i] for i, m in enumerate(state.joint_map) if m is None]
    if missing and not state.joint_map_warned:
        state.joint_map_warned = True
        carb.log_warn(f"isaac_app: unmapped contract joints {missing}; dof names={names}")


def _apply_passive_targets():
    """Policy is off: hold (estop/hard) or blend toward the folded sit pose."""
    art = spot.articulation
    if art is None:
        return
    _ensure_joint_map()
    if state.joint_map is None:
        return
    positions = art_dof_positions(art)
    if positions is None:
        return
    n = positions.shape[0]
    if state.hold_positions is not None:
        targets = state.hold_positions[:n]
    else:
        fold = contract_pose_to_isaac(SIT_POSE, state.joint_map, n)
        if state.sit_start_time is not None and state.sit_start_positions is not None:
            alpha = min(1.0, (state.sim_time - state.sit_start_time) / SIT_BLEND_SECONDS)
            targets = (1.0 - alpha) * state.sit_start_positions[:n] + alpha * fold
        else:
            targets = fold
    art_set_dof_position_targets(art, targets)


def _apply_stand_up_targets(step_size):
    """Blend from the pose at the stand request to the default stance (with
    integral compensation for the soft gains), hold until the joints are at
    the stance, then hand over to the policy (state.standing = True)."""
    art = spot.articulation
    elapsed = state.sim_time - state.stand_start_time
    q = art_dof_positions(art) if art is not None else None
    if q is None or state.stand_pose is None or state.stand_start_positions is None:
        state.standing_up, state.standing = False, True  # nothing to blend; hand over
        return
    n = min(len(state.stand_pose), len(state.stand_start_positions), len(q))
    alpha = min(1.0, elapsed / STAND_BLEND_SECONDS)
    alpha = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep: no jerk at either end
    if state.stand_path is not None and n == len(state.stand_pose):
        feet0, goal_feet, goal = state.stand_path
        base = foot_path_targets(state.stand_start_positions[:n], feet0, goal_feet, goal, alpha)
    else:
        goal = state.stand_pose[:n]
        base = (1.0 - alpha) * state.stand_start_positions[:n] + alpha * goal
    offset = state.stand_offset[:n]
    offset += STAND_KI * step_size * (base - q[:n])
    np.clip(offset, -STAND_I_LIMIT_RAD, STAND_I_LIMIT_RAD, out=offset)
    art_set_dof_position_targets(art, base + offset)
    err = float(np.max(np.abs(goal - q[:n])))
    settled = False
    tilt = None
    if elapsed >= STAND_BLEND_SECONDS + STAND_SETTLE_SECONDS and err <= STAND_HANDOVER_TOL_RAD:
        tilt = body_tilt()
        settled = tilt is not None and max(abs(tilt[0]), abs(tilt[1])) <= STAND_HANDOVER_TILT_RAD
    if settled or elapsed >= STAND_BLEND_SECONDS + STAND_SETTLE_MAX_SECONDS:
        # Policy handover. The controller still holds the feedback (previous
        # action, observation history) from the last time it ran -- spawn,
        # or the walk before the last sit. initialize() on an already-built
        # runner is the public wrapper of _reset_runtime()
        # (isaacsim.robot.policy.examples runtime.py:350, :519): controller
        # + actuators + tick reset from the CURRENT articulation state, no
        # teleport. Then one policy step writes targets before the next
        # physics tick, as restart_from_default_state does.
        zero = np.zeros(3, dtype=np.float32)
        set_stand_gains(False)
        spot.initialize()
        spot.step(step_size, zero)
        state.standing_up = False
        state.standing = True
        state.base_command = zero
        pos, _ = read_body_pose()
        tilt = tilt or body_tilt() or (float("nan"), float("nan"))
        print(f"STAND handover sim_time={state.sim_time:.2f} after={elapsed:.2f}s "
              f"max_joint_err={err:.3f}rad settled={settled} roll={tilt[0]:+.3f} pitch={tilt[1]:+.3f} "
              f"z={float(pos[2]) if pos is not None else float('nan'):.3f}", flush=True)


def on_physics_step(step_size, _context):
    _cb0 = time.monotonic()
    try:
        _on_physics_step(step_size)
    finally:
        perf_t["cb"] += time.monotonic() - _cb0


def _on_physics_step(step_size):
    try:
        # Time/step accounting first: publishers and the smoke guard are
        # driven by sim_time/step_count and must keep advancing even after a
        # policy failure disables stepping.
        state.sim_time += step_size
        state.step_count += 1
        perf_steps[0] += 1
        if state.policy_failed:
            return
        if state.first_step or state.request_restart:
            state.first_step = False
            state.request_restart = False
            heading_hold.release()
            # last_pose is refreshed by the 50 Hz odometry read, so until then
            # it is the PRE-teleport pose: the stance hold latched that
            # heading and turned Spot at spawn back to where the previous run
            # left it, e.g. facing away from the tank after an exit or a
            # retreat (leadership rehearsal 2026-09-28: 180 -> ~10 deg within
            # 5 s of reset_spot.sh). No pose until the next read.
            state.last_pose = None
            set_stand_gains(False)  # no-op unless a stand-up was interrupted
            spot.restart_from_default_state(np.zeros(3, dtype=np.float32))
            return
        if state.stand_pose is None and spot.articulation is not None:
            _p = art_dof_positions(spot.articulation)
            if _p is not None:
                state.stand_pose = np.array(_p, dtype=np.float32).copy()

        if state.standing_up:
            heading_hold.release()
            if not state.powered or state.estopped:
                state.standing_up = False
            else:
                if state.stand_gains_pending:
                    state.stand_gains_pending = False
                    set_stand_gains(True)
                _apply_stand_up_targets(step_size)
                return
        if state.gains_stiff:  # any exit from the stand-up (sit, e-stop, handover)
            set_stand_gains(False)

        policy_active = state.powered and state.standing and not state.estopped
        if policy_active:
            fresh = (state.sim_time - state.last_cmd_time) <= CMD_VEL_WATCHDOG_S
            if state.claimed and fresh:
                cmd = state.base_command
            else:
                cmd = np.zeros(3, dtype=np.float32)  # watchdog: zero the command
            # Zero command: hold the heading (stance hold; sim/stance_hold.py).
            q = state.last_pose[1] if state.last_pose is not None else None
            yaw = (quat_yaw(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                   if q is not None else None)
            cmd = np.array(heading_hold.command(cmd, yaw), dtype=np.float32)
            _pt0 = time.monotonic()
            spot.step(step_size, cmd)
            perf_t["policy"] += time.monotonic() - _pt0
        else:
            heading_hold.release()
            _apply_passive_targets()
    except Exception as error:  # noqa: BLE001 — physics callbacks must not raise
        state.policy_failed = True
        carb.log_error(f"isaac_app: physics-step failure, policy disabled: {error}")


physics_cb_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)


# ---------------------------------------------------------------------------
# Publishing helpers (called from the render loop, driven by sim time)
# ---------------------------------------------------------------------------

def quat_yaw(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_body_pose():
    try:
        positions, orientations = spot.articulation.get_world_poses()
        pos = _to_numpy(positions)[:3]
        quat = _to_numpy(orientations)[:4]  # WXYZ
        state.last_pose = (pos, quat)
        return pos, quat
    except Exception:
        return state.last_pose if state.last_pose is not None else (None, None)


_prev_odom = {"t": None, "pos": None, "yaw": None}


def publish_odometry_and_tf(stamp):
    pos, quat = read_body_pose()
    if pos is None:
        return
    w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    yaw = quat_yaw(w, x, y, z)

    # Twist via sim-time finite difference, rotated into the body (child) frame.
    vx_b = vy_b = vz = yaw_rate = 0.0
    if _prev_odom["t"] is not None:
        dt = state.sim_time - _prev_odom["t"]
        if dt > 1.0e-6:
            dp = (pos - _prev_odom["pos"]) / dt
            dyaw = yaw - _prev_odom["yaw"]
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            yaw_rate = dyaw / dt
            c, s = math.cos(yaw), math.sin(yaw)
            vx_b = c * dp[0] + s * dp[1]
            vy_b = -s * dp[0] + c * dp[1]
            vz = float(dp[2])
    _prev_odom["t"] = state.sim_time
    _prev_odom["pos"] = pos.copy()
    _prev_odom["yaw"] = yaw

    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = ODOM_FRAME
    odom.child_frame_id = BODY_FRAME
    odom.pose.pose.position.x = float(pos[0])
    odom.pose.pose.position.y = float(pos[1])
    odom.pose.pose.position.z = float(pos[2])
    odom.pose.pose.orientation.w = w
    odom.pose.pose.orientation.x = x
    odom.pose.pose.orientation.y = y
    odom.pose.pose.orientation.z = z
    odom.twist.twist.linear.x = float(vx_b)
    odom.twist.twist.linear.y = float(vy_b)
    odom.twist.twist.linear.z = vz
    odom.twist.twist.angular.z = float(yaw_rate)
    pub_odom.publish(odom)

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = ODOM_FRAME
    tf.child_frame_id = BODY_FRAME
    tf.transform.translation.x = float(pos[0])
    tf.transform.translation.y = float(pos[1])
    tf.transform.translation.z = float(pos[2])
    tf.transform.rotation.w = w
    tf.transform.rotation.x = x
    tf.transform.rotation.y = y
    tf.transform.rotation.z = z
    msg = TFMessage()
    msg.transforms = [tf]
    pub_tf.publish(msg)


def publish_joint_states(stamp):
    _ensure_joint_map()
    if state.joint_map is None:
        return
    positions = art_dof_positions(spot.articulation)
    velocities = art_dof_velocities(spot.articulation)
    if positions is None:
        return
    js = JointState()
    js.header.stamp = stamp
    js.name = list(CONTRACT_JOINT_NAMES)
    js.position = [
        float(positions[m]) if m is not None and m < positions.shape[0] else 0.0
        for m in state.joint_map
    ]
    if velocities is not None:
        js.velocity = [
            float(velocities[m]) if m is not None and m < velocities.shape[0] else 0.0
            for m in state.joint_map
        ]
    pub_joints.publish(js)


def battery_pct():
    return battery.pct(state.sim_time)


def publish_status():
    pub_status.publish(String(data=json.dumps({
        # Backend identity (shared contract): the agent commands a backend
        # only if its payload/status says it is this simulator.
        "simulated": True,
        "claimed": state.claimed,
        "powered": state.powered,
        "standing": state.standing,
        "standing_up": state.standing_up,
        "estopped": state.estopped,
        "battery_pct": battery_pct(),
        "sim_time": round(state.sim_time, 4),
        "rtf": rtf_meter.rtf,   # T41: sim s per wall s, last 2 s window; None at first
        # T42: per-topic input over the rate limit or a full queue
        # ("dropped") and input refused after parsing ("rejected"), since
        # start; audit records held back by FacadeAudit's budget.
        **inputs.counters(),
        "events_suppressed": audit.suppressed,
        "cmd_vel_publishers": cmd_vel_watch.count,   # T20; None until polled
    })))


def ventilation_payload():
    vs = gas.ventilation_state(state.sim_time)
    for key in ("t_on", "tau_s", "fraction_remaining"):
        if vs[key] is not None:
            vs[key] = round(float(vs[key]), 4)
    return vs


def gas_sample_with_alarm():
    pos, _ = read_body_pose()
    if pos is None:
        pos = np.array([spawn_xy[0], spawn_xy[1], 0.5], dtype=np.float32)
    # GasField.sample keys (verified in sim/gas_field.py): o2_pct, lel_pct,
    # h2s_ppm, co_ppm. OSHA thresholds are owned by GasField.alarms.
    sample = gas.sample(float(pos[0]), float(pos[1]), float(pos[2]), t=state.sim_time)
    alarms = GasField.alarms(sample)
    sample["alarms"] = alarms          # per-gas booleans + "any"
    sample["alarm"] = alarms["any"]    # convenience OR for the demo UI
    sample["ventilation"] = ventilation_payload()
    sample["injected_pockets"] = gas.injected_count
    return sample


def publish_gas():
    pub_gas.publish(String(data=json.dumps(gas_sample_with_alarm())))


def _handle_ventilation(v):
    """Apply a ventilation control already checked by facade_guard.validate_ventilation."""
    if v.get("reset"):
        gas.reset_ventilation(ventilated=GAS_INITIAL_VENTILATED, t=state.sim_time)
        return f"reset to --gas-initial {args.gas_initial}"
    tau = v["tau_s"] if v["tau_s"] is not None else gas.ventilation_state()["tau_s"]
    gas.set_ventilation(v["on"], state.sim_time, tau_s=tau)
    return f"on={v['on']} tau_s={tau}"


def _audit_control(control, publishers, params=None, error=None, detail=None):
    """T23: one payload/events record per world control, applied or refused."""
    if error is not None:
        inputs.reject(control)
    _audit(facade_guard.world_control_event(control, params, error, publishers, detail,
                                            gas.injected_count))


def _probe_body_pose():
    """(x, y, z, yaw) of the body for aiming the probe, or None if the
    facade has never read a pose (the request is then refused)."""
    pos, quat = read_body_pose()
    if pos is None or quat is None:
        return None
    return (float(pos[0]), float(pos[1]), float(pos[2]),
            quat_yaw(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))


def _handle_probe(text):
    """Answer one probe request on payload/gas/probe_result -- always, so a
    requester waiting on its id never hangs (errors carry "error").

    The probe is an extendable gas probe (simulated; arm-mounted on the real
    robot). A request is refused as a whole (no samples) if the body pose is
    unknown, or if ANY point is outside the z range, beyond 1.1 m from the
    body's front, or behind the tank shell outside the doorway opening
    ("occluded by tank wall"); see sim/gas_field.py ProbeGeometry.

    T21 (contract): the id is echoed back unchanged; an id that is not 1-64
    printable ASCII characters is refused and the reply carries "id": null."""
    req_id = None
    try:
        d = facade_guard.parse_control_json(text, "gas/probe")
        req_id = d.get("id")
        if not facade_guard.valid_probe_id(req_id):
            req_id = None  # never reflect an id outside the contract
            raise ValueError(f"id must be 1-{facade_guard.PROBE_ID_MAX_CHARS} printable "
                             "ASCII characters")
        points = d.get("points")
        if not isinstance(points, list) or not 1 <= len(points) <= PROBE_MAX_POINTS:
            raise ValueError(f"points must be a list of 1..{PROBE_MAX_POINTS} [x, y, z]")
        pts = []
        for p in points:
            if not isinstance(p, (list, tuple)) or len(p) != 3:
                raise ValueError(f"each point must be [x, y, z], got {p!r}")
            x, y, z = (float(v) for v in p)
            if not all(math.isfinite(v) for v in (x, y, z)):
                raise ValueError(f"non-finite point {p!r}")
            pts.append((x, y, z))
        body = _probe_body_pose()
        for pt in pts:
            why = probe_geom.check(body, pt)
            if why:
                raise ValueError(why)
        samples = []
        for (x, y, z), s in zip(pts, gas.probe(pts, t=state.sim_time)):
            s["alarms"] = GasField.alarms(s)
            samples.append({"point": [round(x, 4), round(y, 4), round(z, 4)], **s})
        result = {"id": req_id, "samples": samples, "sim_time": round(state.sim_time, 4),
                  "wall_t": round(time.time(), 3), "ventilation": ventilation_payload(),
                  "simulated": True, "probe": PROBE_LABEL,
                  "body_xy": [round(body[0], 3), round(body[1], 3)]}
        node.get_logger().info(
            f"gas probe {req_id} ({PROBE_LABEL}): " + ", ".join(
                f"({s['point'][0]:.2f},{s['point'][1]:.2f}) z={s['point'][2]:.2f} "
                f"H2S={s['h2s_ppm']:.1f} LEL={s['lel_pct']:.1f}"
                f"{' ALARM' if s['alarms']['any'] else ''}" for s in samples))
    except Exception as error:  # noqa: BLE001 - reported to the requester
        reason = facade_guard.clean_text(error)  # may quote the request: printable, bounded
        result = {"id": req_id if isinstance(req_id, str) else None, "error": reason,
                  "samples": [], "sim_time": round(state.sim_time, 4),
                  "wall_t": round(time.time(), 3), "simulated": True, "probe": PROBE_LABEL}
        node.get_logger().error(f"payload/gas/probe rejected ({PROBE_LABEL}): {reason}")
    pub_probe.publish(String(data=json.dumps(result)))


def apply_control_commands():
    """Drain payload/view and payload/battery/reset on the sim thread."""
    while _ctl_cmds:
        kind, text, pubs = _ctl_cmds.popleft()
        control = "view" if kind == "view" else "battery/reset"
        try:
            if kind == "battery_reset":
                params = facade_guard.validate_battery_reset(
                    facade_guard.parse_control_json(text, control))
                battery.reset(state.sim_time)
                node.get_logger().info(f"payload/battery/reset applied at sim_time={state.sim_time:.2f}: "
                                       f"100 % ({battery.describe()})")
                _audit_control(control, pubs, params=params, detail="battery back to 100 %")
                continue
            camera, ui_mode = sensors.parse_view_command(facade_guard.control_text(text, control))
            if camera is not None and chase_cam is not None:
                chase_cam.set_view(camera)
            if ui_mode is not None:
                _set_ui_mode(ui_mode)
            print(f"VIEW camera={chase_cam.view if chase_cam is not None else 'none'} "
                  f"ui={_ui_state['mode'] or ('headless' if HEADLESS else args.ui)} "
                  f"sim_time={state.sim_time:.2f}{' (headless: no viewport)' if HEADLESS else ''}",
                  flush=True)
            _audit_control(control, pubs, params={"camera": camera, "ui": ui_mode},
                           detail=f"camera {camera or 'unchanged'}, ui {ui_mode or 'unchanged'}")
        except Exception as error:  # noqa: BLE001 - bad presenter input must not stop the sim
            node.get_logger().error(f"payload/{control} rejected: {facade_guard.clean_text(error)}")
            _audit_control(control, pubs, error=error)


def apply_gas_commands():
    """Drain queued gas controls on the sim thread. Returns True if the
    field changed (the caller then publishes payload/gas immediately)."""
    changed = False
    while _gas_cmds:
        kind, text, pubs = _gas_cmds.popleft()
        if kind == "probe":
            _handle_probe(text)
            continue
        control = f"gas/{kind}"
        try:
            # Allowlists and bounds: sim/facade_guard.py (T23).
            d = facade_guard.parse_control_json(text, control)
            if kind == "inject":
                params = facade_guard.validate_inject(d, tank_center_xy, tank_radius)
                gas.inject_spike(params["gas"], params["ppm"], params["x"], params["y"],
                                 params["z"], params["radius"])
                what = facade_guard.describe_inject(params)
            elif kind == "clear":
                params = facade_guard.validate_clear(d)
                what = f"removed {gas.clear_injected()} injected pocket(s)"
            else:  # ventilation
                params = facade_guard.validate_ventilation(d)
                what = _handle_ventilation(params)
            changed = True
            node.get_logger().info(f"payload/{control} applied at sim_time={state.sim_time:.2f}: {what}")
            _audit_control(control, pubs, params=params, detail=what)
        except Exception as error:  # noqa: BLE001 - bad operator input must not stop the sim
            node.get_logger().error(f"payload/{control} rejected: {facade_guard.clean_text(error)}")
            _audit_control(control, pubs, error=error)
    return changed


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

RUNNING = True


def _sigint_handler(_signum, _frame):
    global RUNNING
    RUNNING = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)

if args.smoke:
    # Smoke mode drives the state machine itself so the policy runs.
    state.claimed = True
    state.powered = True
    state.standing = True
    carb.log_info("isaac_app: SMOKE mode — auto claim/power_on/stand")

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()
# After the first update the (hidden or GUI) viewport certainly exists.
_configure_viewport()
if chase_cam is not None:
    chase_cam.update((spawn_xy[0], spawn_xy[1], 0.5), spawn_yaw, 0.0)

# SimulationManager.set_device() only reconfigures physics scenes that are
# already registered (broadphase MBP + GPU dynamics off for cpu). The early
# call above ran before the scene existed, so re-apply now and report the
# effective PhysX scene settings instead of assuming them.
SimulationManager.set_device(args.device)
try:
    for _prim in omni.usd.get_context().get_stage().Traverse():
        if _prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            _api = PhysxSchema.PhysxSceneAPI(_prim)
            print(f"PHYSICS-SCENE {_prim.GetPath()} device={args.device} "
                  f"gpu_dynamics={_api.GetEnableGPUDynamicsAttr().Get()} "
                  f"broadphase={_api.GetBroadphaseTypeAttr().Get()} "
                  f"solver={_api.GetSolverTypeAttr().Get()} fabric={USE_FABRIC}", flush=True)
except Exception as _scene_err:  # diagnostics only
    print(f"PHYSICS-SCENE report failed: {_scene_err}", flush=True)

# ROS callbacks run on a background executor thread: draining with
# spin_once() on the sim thread cost ~1 ms per call (measured), which at
# 50 ticks/s would eat most of the real-time budget. Callbacks only assign
# plain attributes / append to lists, which the GIL keeps consistent with
# the physics callback reading them.
ros_executor = rclpy.executors.SingleThreadedExecutor()
ros_executor.add_node(node)
ros_thread = threading.Thread(target=ros_executor.spin, name="ros-executor", daemon=True)
ros_thread.start()

# Stdout marker host scripts wait on (carb.log_info does not reach stdout).
print(f"FACADE-READY node=spot_facade namespace={ROS_NAMESPACE} run_id={RUN_ID}", flush=True)

# Loop structure (docs/isaac-api-notes.md, SimulationManager.step /
# RenderingManager.render in 6.1): physics is stepped directly, PHYSICS_STEPS_PER_TICK
# steps per tick, and the stage is rendered only every RENDER_PERIOD_S of sim
# time. A full simulation_app.update() per 10 physics steps capped the sim at
# ~0.2x real time on the L4 host because every update renders.
PHYSICS_STEPS_PER_TICK = 10
PROFILE_AT = float(os.environ["SPOT_PROFILE_AT"]) if os.environ.get("SPOT_PROFILE_AT") else None
prof_state = {"p": None, "t_end": 0.0, "done": False}
REALTIME = args.realtime == "true"
RENDER_PERIOD_S = 1.0 / RENDER_HZ
UPDATE_CHASE_CAM = (not HEADLESS) and chase_cam is not None  # nobody sees it headless
next_odom = next_status = next_gas = next_render = 0.0
perf["wall"] = time.monotonic()  # start the RTF window at play time
pace_wall0, pace_sim0 = time.monotonic(), state.sim_time
smoke_next_print = SMOKE_PRINT_EVERY
smoke_frame_guard = 0
exit_code = 0

try:
    while RUNNING and simulation_app.is_running():
        render_due = state.sim_time >= next_render
        # Optional one-shot profiler (diagnostics): SPOT_PROFILE_AT=<sim seconds>.
        if PROFILE_AT is not None and state.sim_time >= PROFILE_AT and prof_state["p"] is None:
            prof_state["p"] = cProfile.Profile()
            prof_state["t_end"] = state.sim_time + 2.0
            prof_state["p"].enable()
        elif prof_state["p"] is not None and not prof_state["done"] and state.sim_time >= prof_state["t_end"]:
            prof_state["p"].disable()
            prof_state["done"] = True
            _buf = io.StringIO()
            pstats.Stats(prof_state["p"], stream=_buf).sort_stats("tottime").print_stats(25)
            print("PROFILE-BEGIN\n" + _buf.getvalue() + "\nPROFILE-END", flush=True)
        _t0 = time.monotonic()
        if USE_FABRIC and render_due:
            # Only the last step before a render needs its results in Fabric.
            SimulationManager.step(steps=PHYSICS_STEPS_PER_TICK - 1)
            SimulationManager.step(steps=1, update_fabric=True)
        else:
            SimulationManager.step(steps=PHYSICS_STEPS_PER_TICK)
        _t1 = time.monotonic()
        rendered = False
        if render_due:
            # Advance on a fixed grid (not sim_time + period) so tick
            # granularity doesn't stretch the period (was 8.2 Hz for 10).
            next_render = max(next_render + RENDER_PERIOD_S, state.sim_time - RENDER_PERIOD_S)
            if UPDATE_CHASE_CAM and state.last_pose is not None:
                # last_pose is refreshed by the 50 Hz odometry publisher, so
                # the chase camera costs no extra articulation read.
                _cp, _cq = state.last_pose
                chase_cam.update(_cp, quat_yaw(*(float(q) for q in _cq)), state.sim_time)
            if payload_lights is not None and state.last_pose is not None:
                payload_lights.update(state.last_pose[0])   # both modes: body cameras see it
            RenderingManager.render()
            rendered = True
        _t2 = time.monotonic()
        perf_t["update"] += _t1 - _t0
        perf_t["spin"] += _t2 - _t1  # now: render time (kept key for the PERF line)
        perf_frames[0] += 1
        if rendered:
            perf_renders[0] += 1

        if _gas_cmds and apply_gas_commands():
            next_gas = state.sim_time  # publish the changed field this tick
        if _ctl_cmds:
            apply_control_commands()
            next_status = state.sim_time  # a battery reset shows up at once
        if audit.pending:
            for _record in audit.drain():
                pub_events.publish(String(data=_record))
        if _ui_state["report_at"] is not None and time.monotonic() >= _ui_state["report_at"]:
            _ui_report()
        stamp = node.get_clock().now().to_msg()
        if state.sim_time >= next_odom:
            next_odom = state.sim_time + ODOM_PERIOD_S
            publish_odometry_and_tf(stamp)
            publish_joint_states(stamp)
        if state.sim_time >= next_status:
            next_status = state.sim_time + STATUS_PERIOD_S
            publish_status()
        if state.sim_time >= next_gas:
            next_gas = state.sim_time + GAS_PERIOD_S
            publish_gas()

        # Real-time pacing: never run ahead of the wall clock, so a presenter's
        # "walk 2 m" means 2 m in about 4 s. Wall time paces only; it never
        # drives physics (which advances in fixed dt steps).
        if REALTIME:
            ahead = (state.sim_time - pace_sim0) - (time.monotonic() - pace_wall0)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5:  # fell behind: re-anchor instead of sprinting
                pace_wall0, pace_sim0 = time.monotonic(), state.sim_time
        rtf_meter.update(time.monotonic(), state.sim_time)

        # Real-time-factor readout (wall clock used for diagnostics only).
        _now = time.monotonic()
        if _now - perf["wall"] >= PERF_PERIOD_WALL_S:
            _dw = _now - perf["wall"]
            _ticks = max(perf_frames[0], 1)
            print(f"PERF rtf={(state.sim_time - perf['sim']) / _dw:.2f} "
                  f"ticks/s={perf_frames[0] / _dw:.1f} renders/s={perf_renders[0] / _dw:.1f} "
                  f"sim_time={state.sim_time:.1f}s "
                  f"ms/tick step={1e3 * perf_t['update'] / _ticks:.1f} "
                  f"render_ms/render={1e3 * perf_t['spin'] / max(perf_renders[0], 1):.1f} "
                  f"phys_steps/tick={perf_steps[0] / _ticks:.1f} "
                  f"policy_ms/step={1e3 * perf_t['policy'] / max(perf_steps[0], 1):.2f} "
                  f"cb_ms/tick={1e3 * perf_t['cb'] / _ticks:.1f}", flush=True)
            perf_t.update(update=0.0, spin=0.0, policy=0.0, cb=0.0)
            perf_steps[0] = 0
            perf_renders[0] = 0
            perf.update(wall=_now, sim=state.sim_time)
            perf_frames[0] = 0

        if args.smoke:
            smoke_frame_guard += 1
            if state.policy_failed:  # fail fast; do NOT print the OK marker
                print("SMOKE FAILED: physics-step failure at step "
                      f"{state.step_count} (see carb log)", flush=True)
                exit_code = 1
                break
            if state.step_count >= smoke_next_print:
                smoke_next_print += SMOKE_PRINT_EVERY
                pos, quat = read_body_pose()
                if pos is not None:
                    yaw = quat_yaw(*(float(q) for q in quat))
                    pose_txt = (f"x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} "
                                f"yaw={yaw:.3f}")
                else:
                    pose_txt = "unavailable"
                print(f"SMOKE step={state.step_count} odom: {pose_txt} "
                      f"gas: {json.dumps(gas_sample_with_alarm())}", flush=True)
            if state.step_count >= SMOKE_STEPS:
                # Exact success marker grepped by host/smoke_test.sh. Failure
                # paths must never print a string containing "SMOKE-OK".
                print(f"SMOKE-OK steps={state.step_count} "
                      f"sim_time={state.sim_time:.2f}s run_id={RUN_ID}", flush=True)
                break
            if smoke_frame_guard > 20000:  # safety: sim never started stepping
                print("SMOKE FAILED: physics never reached "
                      f"{SMOKE_STEPS} steps (got {state.step_count})", flush=True)
                exit_code = 1
                break

    if args.smoke and exit_code == 0 and state.step_count < SMOKE_STEPS:
        # Loop ended early (window closed / SIGTERM) before the run finished.
        print(f"SMOKE FAILED: exited early at {state.step_count} steps", flush=True)
        exit_code = 1
except KeyboardInterrupt:
    pass
finally:
    try:
        timeline.stop()
    except Exception:
        pass
    try:
        SimulationManager.deregister_callback(physics_cb_id)
    except Exception:
        pass
    try:
        spot.close()
    except Exception:
        pass
    try:
        ros_executor.shutdown(timeout_sec=1.0)
    except Exception:
        pass
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    simulation_app.close()

sys.exit(exit_code)
