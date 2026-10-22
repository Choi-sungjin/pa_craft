# craft_planning

`planning_master` is a C++17 ROS 2 node that sequences base navigation, Cartesian
arm goals, gripper commands, carousel slots and waits. Startup is `IDLE` with all
axes held. The supplied `config/missions.yaml` contains `missions: {}`: loading
the package never starts an example mission or invents physical target positions.

## Architecture

| File/module | Responsibility |
| --- | --- |
| `src/planning_master.cpp` | ROS I/O, TF, input snapshots, asynchronous work, commands and diagnostics |
| `StateMachine` | Readiness, deadlines, bounded calculation retries, completion and cancellation |
| `PathPlanner` | Static maps, footprint inflation, native A*, dynamic obstacles and collision checks |
| `ArmPlanner` | Headless URDF FK/IK with Pinocchio and a measured joint seed |
| `planning_config.cpp` | Required typed ROS parameters and strict mission YAML parsing |

`configure()` validates parameters, loads the fixed map, URDF and mission file,
and configures the modules. `connect()` creates ROS interfaces, TF and the timer.
`planning_loop()` checks readiness, collects calculations, advances one step,
issues commands and reports state. Path/IK calculations run outside ROS callbacks;
only one is in flight. Cancellation/replacement changes a generation so old
results are discarded. Only calculations may retry, never physical actions.

The installed executable is C++. `scripts/planning_master.py` remains legacy
source and is not installed as the executable. Python `src/path_planner.py` and
`src/kant_astar_arc.py` remain importable references with regression tests. Native
C++ planning does not execute KANT `PASS`/`STOP_TURN`/`ALIGN` metadata or its
corner-fillet/speed-profile algorithm. Provenance remains in `third_party/`.
The bundled `v75_ws` IK header and URDF are reused directly; no external
`/ik_target`/`/ik_solution` bridge or RViz demo is needed at runtime.

## ROS inputs

Types use the ROS `package/msg/Type` convention.

| Topic | Type | Meaning |
| --- | --- | --- |
| `/planning/mission` | `std_msgs/msg/String` | Configured mission name or control command below |
| `/planning/target_base_pose` | `geometry_msgs/msg/PoseStamped` | Replace work with one navigation step |
| `/planning/target_tcp_pose` | `geometry_msgs/msg/PoseStamped` | Replace work with one TCP step |
| `/perception/state/odom` | `nav_msgs/msg/Odometry` | Fresh base pose in `map_frame` |
| `/control/arm_joints` | `sensor_msgs/msg/JointState` | Default measured, inverse-calibrated URDF joints; configurable with `joint_state_topic` |
| `/perception/lidar/obstacle` | `geometry_msgs/msg/PoseArray` | LiDAR points; required for navigation by default |
| `/perception/vision/d435/obstacle` | `geometry_msgs/msg/PoseArray` | Additional vision obstacle points |
| `/perception/vision/d435/knob_observations` | `craft_perception/msg/KnobObservationArray` | Depth-valid knob targets |
| `/perception/vision/d435/puck_observations` | `craft_perception/msg/PuckObservationArray` | Axis-valid puck targets |
| `/perception/vision/object_pose` | `geometry_msgs/msg/PoseStamped` | Optional generic observation source |
| `/perception/vision/d435/object_detection` | `geometry_msgs/msg/PoseArray` | Generic object positions |
| `/control/arm_lm_state`, `/control/wheel_state`, `/control/clock_state` | `std_msgs/msg/Int32` | 0 idle, 1 using, 2 fault |
| `/control/{arm,lm,wheel,clock,gripper}_target_reached` | `std_msgs/msg/Bool` | Target acknowledgements/completion |
| `/control/clock_slot` | `std_msgs/msg/Int32` | Measured slot; 0 means unavailable |
| `/control/planning_held` | `std_msgs/msg/Bool` | Hold/release acknowledgement |
| `/control/tracking_path` | `nav_msgs/msg/Path` | Control's resampled path for collision validation |

## ROS outputs

| Topic | Type | Meaning |
| --- | --- | --- |
| `/planning/target_path` | `nav_msgs/msg/Path` | Navigation path; empty on stop/failure |
| `/planning/odom` | `nav_msgs/msg/Odometry` | Fresh validated odometry forwarded to control |
| `/planning/target_v` | `std_msgs/msg/Float64` | Base speed [m/s]; nonzero only after tracking-path validation |
| `/planning/target_arm_joints` | `sensor_msgs/msg/JointState` | Slide [m], shoulder/elbow/wrist pitch/tool roll [rad] |
| `/planning/gripper_command` | `std_msgs/msg/Bool` | true closes, false opens |
| `/planning/target_clock_slot` | `std_msgs/msg/Int32` | Slot 1..12 |
| `/planning/hold` | `std_msgs/msg/Bool` | Periodic all-axis hold level and heartbeat |
| `/planning/state` | `std_msgs/msg/String` | Transient-local FSM state |
| `/planning/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | State, step, generation and readiness/fault reason |

Joint order is `slide_joint`, `shoulder_joint`, `elbow_joint`, `wrist_pitch_joint`,
`tool_roll_joint`. Seeds require these measured joints within URDF limits;
RViz's interpolated `/joint_states` is not hardware feedback. An independent
encoder source can override `joint_state_topic` to `/arm/odom`; the current arm
encoder sensor stub does not publish measurements.

## Configuration, maps and coordinates

Supply the complete [config/planning.yaml](config/planning.yaml). All application
parameters are required; types/ranges and unknown overrides are checked at
startup. Relative `map_file`, `ik.urdf` and `mission_file` paths resolve against
the installed package share. Maps, URDF and missions are loaded at startup.

Base goals and odometry must use `map_frame` at the chassis axle center. The node
does not relabel `odom` as `map` or subscribe to SLAM `/map`. Fixed-map geometry
and localization must share a world origin. TCP goals use `ik.base_frame`
(`base_link`), with `tool_tcp` as the default tip. Lengths are metres, rotations
are radians/quaternions and timeouts are seconds. Perception targets/obstacles
use TF at acquisition timestamps. Missing camera/base/map transforms prevent
readiness; TF is not guessed.

Maps support binary STL (metres, Z-up), text blocks, or trinary ROS YAML with PGM
or PNG. STL unions horizontal slices and preserves XY world origin. Text uses
an integer `block_size/res` ratio; its first row is the top. ROS YAML supplies
resolution and origin translation/yaw. Unknown pixels, transparency and outside
map positions are blocked; raw images and ASCII STL are rejected. Inflation uses
`r_robot + margin`, conservatively including occupied cell area. Dynamic points
are inflated in a private snapshot without modifying the static map.

Native eight-direction A* includes heading-change/proximity costs, forbids
diagonal corner cutting, and shortens routes only with collision-free lines of
sight. Requested start/goal XY is preserved. `max_cells`, `max_expansions` and
sampling checks bound work. Control resamples paths into a spline; its matching
`/control/tracking_path` must also pass collision checks before driving, and new
obstacle inputs trigger another check.

## Missions and commands

This is a complete **format example** for a separate mission file. Replace its
positions/orientations with calibrated, reachable field goals before selecting
it through `mission_file`. The supplied defaults do not enable this example.

```yaml
missions:
  MAIN_MISSION:
    - id: approach
      type: navigate
      frame: map
      pose: [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    - id: grasp_pose
      type: tcp
      source: d435_knob
      class_id: 0
      frame: base_link
      pose: [0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0]
    - {id: close, type: gripper, close: true}
    - {id: store, type: clock, slot: 1}
    - {id: settle, type: wait, duration_s: 1.0}
```

Step IDs must be unique per mission. Unknown keys/types, invalid slots, frames,
poses and non-finite values fail parsing. Pose is `[x,y,z,qx,qy,qz,qw]` with a
nonzero quaternion. `source` defaults to `fixed`; alternatives are `d435_knob`,
`d435_puck`, `object_pose`, `object_detection`. The highest-confidence fresh
matching target is transformed into the step frame. Configured XYZ is an offset
in that frame; the configured quaternion supplies orientation. Target selection
is frozen when calculation begins. `class_id: -1` accepts any class.

Send the configured name on `/planning/mission` to start it; repeated names are
deduplicated. `INIT` clears the remembered name without cancelling active motion.
`STOP`/`CANCEL` cancel work, clear the queue and hold every axis; an existing
fault stays latched. `RESET` clears the planning fault into `CANCELLED`, discards
work and holds; it never resumes a command. Hardware faults and missing inputs
must still be resolved before a new goal.

## Execution and limits

States are `IDLE`, `WAITING`, `COMPUTING`, `EXECUTING`, `SUCCEEDED`, `FAULT`,
`CANCELLED`. Each step requests hold=true and waits for its fresh true
acknowledgement, then requests hold=false and waits for a fresh released
acknowledgement before sending a target. Missing/true/stale heartbeat clears
control targets, commands zero wheel speed and holds measured actuator positions.
Release cannot revive old targets; stop does not open the gripper. Explicit
readiness/computation/execution deadlines, acquisition ages and steady-clock
reception watchdogs fail closed.
Control's `serial_feedback_timeout_s` also holds all axes before valid OpenCR
feedback or when either board stream expires; stale board feedback reports fault.

Completion requires post-command false then fresh true feedback, stable for the
debounce interval. Navigation also checks actual goal distance and optional yaw;
clock checks measured slot and idle state. Bool feedback lacks goal IDs, so
arbitrarily delayed old false/true pairs cannot be fully correlated. Gripper
position completion does not prove that an object was grasped.

Pure Pursuit has no in-place-turn input. Excessive initial heading error is
rejected; `goal_heading_required=true` also rejects incompatible final approach
headings and checks arrival yaw. Default arrival is position-only. Arm IK checks
URDF limits and pose tolerances, with no self/environment collision checking or
motor trajectory profiling. OpenCR/DYNAMIXEL owns motor profiles. Calibrate the
footprint, wheel geometry, motor directions/offsets, LM pulley/gearing/travel and
gripper limits/current thresholds. Default arm/LM/gripper mappings remain
unconfirmed in control; these are not hardware-ready calibration values.

## Build and verification

Provide ROS 2, Pinocchio's CMake package (ROS package `pinocchio`), yaml-cpp,
libpng and declared dependencies. Planning uses `craft_perception` message types,
so its existing OpenCV/ncnn dependencies also apply; see its
[build instructions](../craft_perception/README.md#빌드와-검증). From the repository
root, with `ROS_DISTRO` set for the installed distribution:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to craft_planning craft_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run craft_planning planning_master --ros-args --params-file \
  "$(ros2 pkg prefix craft_planning)/share/craft_planning/config/planning.yaml"
colcon test --packages-select craft_planning craft_control --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  CRAFT_CONTROL_BINARY="$(ros2 pkg prefix craft_control)/lib/craft_control/control_master" \
  python3 src/craft_control/test/test_planning_watchdog_integration.py
```

Use the Pinocchio package built for the active ROS distribution. The verified
Lyrical build uses `ros-lyrical-pinocchio` 4.1.0 and its matching native dependencies;
the initial pip `pin` dependency combination crashed during ROS node construction.

Tests cover native map/FK/IK/FSM/configuration behavior, ROS scheduling with
synthetic feedback, and control heartbeat/hold/recovery using disposable PTYs.
They do not verify physical motor calibration or complete hardware bringup.
