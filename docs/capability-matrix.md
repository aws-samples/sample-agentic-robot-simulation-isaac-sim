# Spot capability matrix: real driver vs Isaac Sim 6.1 facade

Sources: shallow clones in `/tmp/spot-repos`, taken 2026-09-27 from the default branches. Names come from the code, not the READMEs:
`spot_ros2/spot_driver/spot_driver/spot_ros2.py`, `spot_driver/src/robot_state/state_middleware_handle.cpp`,
`spot_driver/src/api/spot_image_sources.cpp`, `spot_driver/src/lease/lease_manager.cpp`, `spot_msgs/`, `spot_description/urdf/`;
`strands-spot/strands_spot/use_spot.py`; `robots/strands_robots/registry/robots.json`, `docs/robots/mobile.md`, `docs/ros2/*`;
`sample-ros2-with-agentic-control-on-aws/voice_agent/tools.py`, `go2_ros2_client.py`, `sim_dog.py`, `deployment/lib/*.ts`.

Conventions: every name below is relative to the `/<spot_name>/` namespace (the launch arg `spot_name`). TF and joint names use the prefix `<spot_name>/`.
Unless noted, services are `std_srvs/Trigger`, and `spot_msgs` types are shown without the package. Difficulty: S is a stub or shim, M is a few days of work, L needs a controller, a policy, or a new sim subsystem.
The "Must" column marks what the oil and gas confined-space inspection demo needs.

Assumption to check before relying on the Isaac column: this plan expects Isaac Sim 6.1 to ship Spot USD assets (`Robots/BostonDynamics/spot/spot.usd`, plus an arm variant) and the `SpotFlatTerrainPolicy` example in `isaacsim.robot.policy.examples`. Update 2026-09-27: `spot_standalone.py` in `isaacsim.robot.policy.examples` was run headless in `nvcr.io/nvidia/isaac-sim:6.1.0` on a g6.4xlarge (L4). The policy walks: after one open-loop course it ended 0.53 m / 27 deg from the commanded end pose. The arm variant is still unverified.

## Capability table

| Capability | Source repo(s) | Real-Spot interface | Isaac Sim 6.1 realization | Diff | Must |
|---|---|---|---|---|---|
| Velocity walk | spot_ros2, strands-spot (`robot_command.velocity_command`), robots (`RosBridgedRobot.drive`), go2 (`move`, `turn_degrees`) | sub `cmd_vel` `geometry_msgs/Twist` (body frame; the command expires after `cmd_duration`=0.25 s) | Spot USD plus the flat-terrain RL policy, which maps Twist to 12 joint targets through an articulation PD. The facade must add the 0.25 s watchdog that zeroes velocity | M | Y |
| Stop | spot_ros2, go2 (`stop_moving`) | `stop` | Zero the command, and the policy holds stance | S | Y |
| Stand / sit | all four | `stand`, `sit` (SDK `synchro_stand_command` / `synchro_sit_command`) | Scripted PD pose blend to stand or crouch joint targets, with the policy disabled while sitting | S-M | Y |
| Self-right, rollover | spot_ros2, strands-spot | `self_right`, `rollover` | Fake: reset the base pose upright, then blend to sit. Not physically faithful | S | N |
| Power on/off | spot_ros2, strands-spot (`power.power_on`) | `power_on`, `power_off` | State flag. While off, set joint gains to 0 so the robot goes limp; publish `status/power_states` | S | Y |
| Lease | spot_ros2, strands-spot | `claim`, `release`, `take_lease`; `acquire_lease` (`AcquireLease`), `return_lease` (`ReturnLease`), `claim_leases`/`release_leases` (lease_manager); pub `status/leases` `LeaseArray` | Bookkeeping shim that returns success, and rejects motion when no lease is held so agent errors match the real robot | S | Y |
| E-stop | spot_ros2 | `estop/hard`, `estop/gentle`, `estop/release`; pub `status/estop` `EStopStateArray` | Hard: cut the torques. Gentle: sit, then power off. Latch until release. (The upstream code carries a TODO saying these calls are broken on real hardware, so test them on the robot) | S | Y |
| Body pose / stand height | spot_ros2 | sub `body_pose` `geometry_msgs/Pose`; `set_stand_height` `SetStandHeight` (±0.15 m); pub `status/mobility_params` | Policy command offsets (the flat policy has no height input): IK-shift the hip targets, or train a height-conditioned policy | M | Y (crouch under pipes) |
| Locomotion hint / stairs / max velocity / obstacle params | spot_ros2 | `locomotion_mode` `SetLocomotion`, `stairs_mode` `SetStairsMode`, `max_velocity` `SetVelocity`; `robot_command` action (full bosdyn `MobilityParams`, obstacle params); `trajectory` action `disable_obstacle_avoidance` | Store the values. Clamp `cmd_vel` to max_velocity. Stairs needs a rough-terrain policy (Isaac Lab); obstacle avoidance needs a costmap-based velocity filter | M-L | Partial (max_vel Y) |
| Go-to-pose | spot_ros2 | action `trajectory` `Trajectory` (PoseStamped + duration) | Pure-pursuit or Nav2 controller on the sim `odometry`, sending `cmd_vel` to the policy | M | Y |
| Raw robot command | spot_ros2, strands-spot | action/service `robot_command` (`bosdyn_msgs` RobotCommand) | Parse the subset used (stand, sit, velocity, arm stow/unstow) and reject the rest with a clear message | M | N |
| Odometry / TF | spot_ros2 | pub `odometry` `nav_msgs/Odometry`, `odometry/twist`; TF root `odom` or `vision`; frames `body`, `odom`, `vision`, `gpe`, `flat_body`, camera frames | Isaac ROS 2 OmniGraph: odometry plus TF publisher. Alias `vision` to the sim ground truth. `robot_state_publisher` with spot_description xacro | S | Y |
| Joint states | spot_ros2, robots (sim-bridge `joint_states`) | pub `joint_states` (`<prefix>front_left_hip_x`…, `arm_sh0`…`arm_f1x`) | OmniGraph articulation-state publisher, renaming the USD joints to the spot_ros2 names | S | Y |
| Feet / faults / wifi / metrics / feedback | spot_ros2 | `status/feet` `FootStateArray`, `status/system_faults`, `status/behavior_faults`, `status/wifi`, `status/metrics`, `status/feedback`, `status/has_arm` `std_msgs/Bool`; `clear_behavior_fault` | Feet from contact sensors. Everything else is a static or templated publisher. Inject faults for demo scenarios | S | N (has_arm Y) |
| Battery / power state | spot_ros2 | `status/battery_states` `BatteryStateArray`, `status/power_states` `PowerState` | Simulated discharge model, which also drives the return-to-dock story | S | Y |
| 5 body cameras | spot_ros2, strands-spot (`image.get_image_from_sources`), go2 (`describe_scene`) | `camera/{frontleft,frontright,left,right,back}/image`, `/camera_info`, `/compressed` | 5 Isaac Camera prims at the calibrated fisheye extrinsics (pinhole approximation), published with ROS 2 camera-helper OmniGraph | M | Y |
| Hand camera | spot_ros2 | `camera/hand/image` (+ `camera_info`) on frame `hand_color_image_sensor`; `get/set_gripper_camera_parameters` | Camera prim on the arm wrist link | S | Y |
| Depth / registered depth | spot_ros2 | `depth/<cam>/image`, `depth_registered/<cam>/image` (+ `camera_info`) | Isaac depth annotator per camera. For registered depth, render depth from the RGB camera pose | M | Y |
| Stitched front view | spot_ros2 | `virtual_camera/image` | Run the real `image_stitcher` node on the sim images unchanged | S | N |
| Point cloud / lidar (EAP) | spot_ros2, go2 (`lidar_processor`) | `velodyne/points` `PointCloud2` (`use_velodyne`) | RTX Lidar, VLP-16 profile, on the URDF `velodyne_cage`/`lidar_mount` frame | M | Y |
| Spot CAM PTZ | spot_ros2 | `list_ptz`, `get_ptz_position`, `set_ptz_position` `SetPtzPosition`, `initialize_lens`, `list_cameras`; LED, audio, logpoint services | PTZ = a camera prim on 2 revolute joints plus focal-length zoom. The rest are stubs | M | Y |
| Spot CAM IR / thermal | (driver exposes logpoints only) | `store_logpoint`/`retrieve_logpoint`… | **Not faithful.** Stand-in: a false-colour temperature render from per-prim `temperature` USD attributes (a custom shader or annotator), published as `mono16` | L | Y (hot-spot story) |
| Gas / H2S sensor payload | (none) | none upstream (custom payload) | Stand-in: a gas-concentration field in the scene, sampled at the body pose, on a custom topic | S | Y |
| Arm joint / pose / velocity | spot_ros2 | sub `arm_joint_commands` `JointState`, `arm_pose_commands` `PoseStamped`, `arm_velocity_commands`; `get_inverse_kinematic_solutions`; action `arm_surface_contact` | Spot-with-arm USD with articulation position control. Pose commands go through Lula/RMPflow with a generated robot description | M-L | N (nice) |
| Arm stow / unstow / carry | spot_ros2, strands-spot (README only) | `arm_stow`, `arm_unstow`, `arm_carry` | Named joint presets | S | N |
| Gripper | spot_ros2 | `open_gripper`, `close_gripper`, `set_gripper_angle` `SetGripperAngle` (0-90°) | Finger joint target | S | N |
| Grasp / manipulation | spot_ros2, strands-spot (README only) | action `manipulation` `Manipulation` (ManipulationApiRequest); `override_grasp_or_carry`; `SpotGraspingPipeline` action defined but not served | Pick-by-pixel: back-project the pixel with depth, then RMPflow reach and a surface gripper | L | N |
| GraphNav localization / maps | spot_ros2 | `graph_nav_upload_graph`, `graph_nav_clear_graph`, `graph_nav_set_localization`, `graph_nav_get_localization_pose`, `list_graph`; action `navigate_to` `NavigateTo`(waypoint_id); pub `graph_nav/body_pose` | **Not faithful** (the vendor's map format is proprietary). Stand-in: a YAML waypoint graph with named waypoints and poses in the scene frame. `navigate_to` then runs pure pursuit or Nav2 over the waypoints. For real Spot, record a matching Autowalk map | M | Y |
| Docking | spot_ros2 | `dock` `Dock`(dock_id), `undock` | Fiducial prim at the dock pose. Dock = navigate to a pre-dock pose, then sit. Recharge the battery model | M | Y |
| Autowalk / mission | none of the repos | not exposed in spot_ros2 | Agent-level waypoint list over `navigate_to` | M | Y |
| World objects / fiducials | spot_ros2 | `list_world_objects`, `mutate_world_objects` | Return the ground-truth poses of AprilTag prims | S | N |
| Choreography / dance | spot_ros2 | action `execute_dance`, `upload_*`, `list_all_*`, `stop_dance` | Unsupported stub | S | N |
| Spot Check | spot_ros2 | `spot_check` | Stub returning OK | S | N |
| Voice (speech to speech) | go2 (Nova Sonic `amazon.nova-2-sonic-v1:0`, `BidiAgent`) | n/a (agent side) | Port unchanged; the agent is robot-agnostic | S | Y |
| Scene VLM | go2 (`describe_scene`, `recall_scene`, `compare_scenes`; Bedrock Claude Sonnet) | camera topic | Point the tools at `camera/hand/image` or the PTZ image | S | Y |
| Object detect / find | go2 (`get_detections`, `find_object`, `start_tracking` via `coco_detector`) | camera topic | Reuse the detector, retrained for gauges, valves, leaks | M | Y |
| Dangerous-action confirm | go2 (`request_`/`confirm_dangerous_action`, 60 s), robots (`/cmd_vel` operator gate) | n/a | Built: a 60 s single-use console `y/N` confirmation (`agent/operator_console.py`) for the permit attestations, entry, `estop/release`, `self_right` and an e-stop (hard or gentle) the model asks for inside the permit space | S | Y |
| Telemetry UI | go2 (Foxglove bridge `:9876`, Lichtblick, SSM tunnel) | n/a | Not built: the HUD replaced it. A bridge would be an unauthenticated listener and needs its own review (threat model T44) | S | N |
| Cloud video | go2 (KVS stream plus `kvs_describe` Lambda) | n/a | KVS producer on the PTZ or hand topic | M | N |
| Cloud deploy | go2 CDK: Network, EC2 (GPU host), KVS, Guardrail, AgentCore Runtime | n/a | Reuse the stacks. Move EC2 to a g6e with Isaac. AgentCore dials the bridge IP | M | Y |
| Sim/real switch | go2 (`client_registry.set_client`, `main_sim.py`), robots (`Robot("spot")` MuJoCo, `mode="real"`) | n/a | The same ROS 2 names in both, so switching changes only `ROS_DOMAIN_ID` or the endpoint | S | Y |

## Findings that change the plan
- **spot_ros2 already ships a mock mode** (`mock_enable`, `mock_has_arm`) that brings up the node without a robot, and `spot_ros2_control` has a mock hardware interface. Run these to capture the exact interface list (`ros2 interface`, `ros2 service list -t`) and use it as the facade's contract test.
- **strands-spot connects only 8 SDK clients**: robot_command, robot_state, power, lease, image, estop, time_sync, directory. Its README advertises `graph_nav`, `manipulation`, `docking`, `arm_stow`, and `gripper_command`, but those raise `Unknown service` or `AttributeError` in the code. Only stand, sit, self_right, velocity_command, power, state, and image actually work. It talks to the SDK directly, so it cannot drive a ROS facade. Use a ROS-based tool instead (robots' `use_ros` or `RosBridgedRobot`).
- **strands-labs/robots has Spot as sim-only**: a MuJoCo Menagerie `spot_arm.xml` with no real driver and no locomotion policy. Its Isaac backend targets Isaac Sim 6.0.1, and its ROS bridge publishes only `joint_states` and `image_raw`. It gates `/cmd_vel` and `/e_stop` behind operator approval by default.
- **The Go2 sample's sim mode animates the gait**: `sim_dog.py` drives the base kinematically. Isaac with an RL policy is a real upgrade over it. Nav2 and SLAM are launched but no agent tool calls them.
