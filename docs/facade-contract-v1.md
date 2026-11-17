# Spot ROS 2 Facade Contract — v1

Extracted verbatim from real `spot_ros2` sources (shallow clone at `/tmp/spot-repos/spot_ros2`, github.com/rai-opensource/spot_ros2, checked 2026-09-27). All names below are as declared in code, not READMEs.

## Namespacing and frame prefix

- The driver and all companion nodes are launched inside namespace `<spot_name>` (`spot_driver/launch/spot_driver.launch.py:85-166`; `spot_name` launch arg declared at line 277, default `""`). All topics/services below are therefore `/<spot_name>/<name>`.
- TF/frame prefix: if the `frame_prefix` parameter is unset, it defaults to `<spot_name>/` (Python: `spot_driver/spot_driver/spot_ros2.py:501-503`; wrapper: `spot_wrapper/spot_wrapper/wrapper.py:409`; C++: `RclcppParameterInterface::getFramePrefixWithDefaultFallback`, `spot_driver/src/interfaces/rclcpp_parameter_interface.cpp:312-315` — `robot_name + "/"`).

## 1. v1 subset — topics

### Subscriptions (Python driver node, `spot_driver/spot_driver/spot_ros2.py:626-627`)

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | Callback `cmd_velocity_callback` (line 2720) uses `linear.x`, `linear.y`, `angular.z` only. Sent with `cmd_duration` TTL. |
| `body_pose` | `geometry_msgs/msg/Pose` | Callback `body_pose_callback` (line 2729): only `position.z` and `orientation` are used; applied as `BodyControlParams.base_offset_rt_footprint` via mobility params (takes effect on next stand). |

**Command timeout constant:** ROS parameter `cmd_duration`, declared with default **`0.125`** seconds (`spot_ros2.py:383`, read at line 489, passed at line 2726). Wrapper signature default matches: `velocity_cmd(..., cmd_duration: float = 0.125)` — "Default is 125ms (assuming 10Hz command rate)" (`spot_wrapper/spot_wrapper/wrapper.py:1288-1310`). Each Twist is a one-shot command valid for `cmd_duration`; publish faster than 8 Hz or the robot stops.

### Publications (C++ state publisher node `spot_state_publisher`, `spot_driver/src/robot_state/state_middleware_handle.cpp:24-36`)

| Topic | Type | Source constant |
|---|---|---|
| `joint_states` | `sensor_msgs/msg/JointState` | `kJointStatesTopic` |
| `odometry` | `nav_msgs/msg/Odometry` | `kOdomTopic` |
| `odometry/twist` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `kOdomTwistTopic` |

### Odometry frames (`spot_driver/src/conversions/robot_state.cpp:213-244`)

- `header.frame_id` = `<prefix>vision` if `preferred_odom_frame == "vision"`, else `<prefix>odom` (lines 231-240).
- `child_frame_id` = `<prefix>body` (line 243).
- Parameter `preferred_odom_frame`: valid values `{"odom", "vision"}`, **default `"odom"`** (`spot_driver/include/spot_driver/interfaces/parameter_interface_base.hpp:69-71`; declared in `rclcpp_parameter_interface.cpp:30,228-235`). Twist selects `velocity_of_body_in_vision` vs `velocity_of_body_in_odom` accordingly (robot_state.cpp:193-203).

### TF frames published

The state publisher republishes Spot's full `transforms_snapshot` frame tree, every frame prefixed `<spot_name>/` (`getTf`, `robot_state.cpp:137-183`). Key frames (canonical BD names, see also the known-frames list in `spot_driver/src/object_sync/object_synchronizer.cpp:50-90`): `odom`, `vision`, `body`, `flat_body`, `gpe`, `base_link`, leg link frames (`front_left_hip`, `front_left_upper_leg`, `front_left_lower_leg`, … per leg), camera frames. TF root is re-rooted to parameter `tf_root`: valid `{"odom", "vision", "body"}`, **default `"odom"`** (`parameter_interface_base.hpp:70-72`; inversion logic robot_state.cpp:174-178).

## 2. v1 subset — Trigger services

All `std_srvs/srv/Trigger`, created in `create_trigger_services()` via `TriggerServiceWrapper` (`spot_ros2.py:1285-1348`; wrapper class in `spot_driver/spot_driver/ros_helpers.py:338`). Names verbatim:

`claim`, `release`, `stop`, `self_right`, `sit`, `stand`, `rollover`, `power_on`, `power_off`, `estop/hard`, `estop/gentle`, `estop/release`, `undock`, `spot_check`, `stop_dance`
(arm-only, out of v1 scope: `arm_stow`, `arm_unstow`, `arm_carry`, `open_gripper`, `close_gripper`; plus `take_lease` at `spot_ros2.py:1079`).

Mapping (spot_ros2.py:1316-1330): `stand`→`SpotWrapper.stand`, `power_off`→`SpotWrapper.safe_power_off`, `estop/hard`→`assertEStop(severe=True)` (cuts motor power immediately), `estop/gentle`→`assertEStop(severe=False)` (settle-then-cut), `estop/release`→`disengageEStop`. Note in-code TODO at line 1326: estop calls flagged as possibly non-functional. Response: `success: bool`, `message: str` passed straight through from the wrapper (`ros_helpers.py:377`).

## 3. The 12 leg joints (spot_description URDF)

From `spot_description/spot_description/urdf/spot_macro.xacro` (revolute joints, lines 85-546), each named `${tf_prefix}<joint>` where xacro arg `tf_prefix` defaults to `""` (`spot.urdf.xacro:15`) and is set to the robot namespace by the launch pipeline. Canonical index order from `spot_ros2_control/include/spot_ros2_control/spot_joint_map.hpp:41-46` (`kJointNameToIndexWithoutArm`):

```
0  front_left_hip_x     1  front_left_hip_y     2  front_left_knee
3  front_right_hip_x    4  front_right_hip_y    5  front_right_knee
6  rear_left_hip_x      7  rear_left_hip_y      8  rear_left_knee
9  rear_right_hip_x    10  rear_right_hip_y    11  rear_right_knee
```

(With arm, indices 12-18 continue: `arm_sh0 arm_sh1 arm_el0 arm_el1 arm_wr0 arm_wr1 arm_f1x`.)

Default standing joint angles (same order, hip_x, hip_y, knee per leg) from `spot_ros2_control/examples/noarm_squat.cpp:21-24`:

```
0.12, 0.72, -1.45,   -0.12, 0.72, -1.45,   0.12, 0.72, -1.45,   -0.12, 0.72, -1.45
```

## 4. status/* topics (names + spot_msgs types; payloads deferred to phase 2)

C++ state publisher (`state_middleware_handle.cpp:27-34`):

| Topic | Type |
|---|---|
| `status/feet` | `spot_msgs/msg/FootStateArray` |
| `status/estop` | `spot_msgs/msg/EStopStateArray` |
| `status/wifi` | `spot_msgs/msg/WiFiState` |
| `status/battery_states` | `spot_msgs/msg/BatteryStateArray` |
| `status/power_states` | `spot_msgs/msg/PowerState` |
| `status/system_faults` | `spot_msgs/msg/SystemFaultState` |
| `status/behavior_faults` | `spot_msgs/msg/BehaviorFaultState` |
| `status/end_effector_force` | `geometry_msgs/msg/Vector3Stamped` |

C++ lease node (`spot_driver/src/lease/lease_middleware_handle.cpp:10`): `status/leases` → `spot_msgs/msg/LeaseArray` (also published by the Python node at `spot_ros2.py:531`).

Python driver node (`spot_ros2.py:614-624`): `status/has_arm` → `std_msgs/msg/Bool` (latched/TRANSIENT_LOCAL), `status/metrics` → `spot_msgs/msg/Metrics`, `status/feedback` → `spot_msgs/msg/Feedback`, `status/mobility_params` → `spot_msgs/msg/MobilityParams`.

## 5. State-machine preconditions enforced by the real driver

Sources: `spot_wrapper/spot_wrapper/wrapper.py`, `wrapper_helpers.py` (`ClaimAndPowerDecorator`), `spot_leash.py`, `spot_ros2.py`.

- **Lease/claim before any command.** Commands go to the BD SDK with `lease=None` (`_robot_command`, wrapper.py:1072-1102) — the SDK's lease wallet must hold a lease. Without a claim, the SDK raises (e.g. LeaseUseError) and the wrapper returns `(False, str(exception))`; there is no explicit "not claimed" check in the driver.
- **Auto-claim on action (default ON).** ROS parameter `get_lease_on_action` **defaults to `True`** (`spot_ros2.py:379`). With it, `SpotLeashContext.bind` (spot_leash.py:95-104) wraps these wrapper methods with `ClaimAndPowerDecorator` so each call first calls `claim()` **then `power_on()`** (wrapper_helpers.py:44-56, claim/power failures ignored — the wrapped command surfaces the real error): `stop, self_right, sit, simple_stand, stand, battery_change_pose, velocity_cmd, trajectory_cmd, execute_dance, _robot_command, _manipulation_request, arm_surface_contact_command` (wrapper.py:527-543). "Passive" set (claim only, **no** power-on): `stop, power_on, safe_power_off, toggle_power` (wrapper.py:545-554). Net effect at defaults: calling `stand` or publishing `cmd_vel` implicitly claims and powers on.
- **`claim`** (wrapper.py:964-986): idempotent — returns `(True, "We already claimed the lease")` if already held. If `start_estop` parameter is true (ROS default `False`, spot_ros2.py:385) and robot not powered, it also takes the software e-stop endpoint (`resetEStop`).
- **`power_on`** (wrapper.py:1253-1272): no-op returning `(True, "Was already powered on")` if already on; acquires e-stop first if `start_estop` and no keepalive; requires e-stop authority — otherwise `(False, "Exception while powering on: <err>")`.
- **`stand` / `sit`** (wrapper.py:1137-1214): no explicit precondition check in the wrapper ("If the e-stop is enabled, and the motor power is enabled, stand the robot up"); motors-off/no-lease failures come back as `(False, <SDK exception string>)`.
- **`rollover`** (`battery_change_pose`, wrapper.py:1216-1228): the one explicit guard — requires `is_sitting`, else returns `(False, "Call sit before trying to roll over")`.
- **`power_off`** = `safe_power_off` (wrapper.py:1230-1237): issues `safe_power_off_command()` — robot sits first, then cuts motor power.
- **`release`** (wrapper.py:1059-1066): returns lease and releases e-stop keepalive.
- **e-stop** (wrapper.py:1004-1030): `estop/hard` cuts motor power immediately (`keepalive.stop()`); `estop/gentle` settles then cuts (`settle_then_cut()`); `estop/release` re-allows (`keepalive.allow()`); asserting creates the keepalive if missing.
- **Optional startup sequence:** parameters `auto_claim`, `auto_power_on`, `auto_stand` (all default `False`, spot_ros2.py:374-376) chain claim→power_on→stand at node startup (spot_ros2.py:1072-1077).
- **Mock mode:** with no `spot_wrapper`, all Trigger services return `success=True, message="Mock spot success"` (ros_helpers.py:359-362) and `cmd_vel`/`body_pose` are logged no-ops.
