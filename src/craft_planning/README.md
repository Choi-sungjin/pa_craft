# craft_planning

`planning_master` is a C++17 ROS 2 node that sequences base navigation, Cartesian
arm goals, gripper commands, carousel slots and waits. Startup is `Idle` with all
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
only one is in flight. Cancellation/fault changes a generation so old
results are discarded. Only calculations may retry, never physical actions.

The installed executable is C++. `scripts/planning_master.py` forwards to the C++ executable. Python `src/path_planner.py` and
`src/kant_astar_arc.py` remain importable references with regression tests. Native
C++ planning does not execute KANT `PASS`/`STOP_TURN`/`ALIGN` metadata or its
corner-fillet/speed-profile algorithm. Provenance remains in `third_party/`.
The bundled `v75_ws` IK header and URDF are reused directly; no external
`/ik_target`/`/ik_solution` bridge or RViz demo is needed at runtime.

## ROS inputs

Types use the ROS `package/msg/Type` convention.

| Topic | Type | Meaning |
| --- | --- | --- |
| `/planning/mission` | `std_msgs/msg/String` | Optional manual input of a configured mission name |
| `/perception/vision/d435/apriltag` | `std_msgs/msg/Int32MultiArray` | Floor/wall tag IDs mapped to configured missions |
| `/planning/target_base_pose` | `geometry_msgs/msg/PoseStamped` | Queue one navigation task |
| `/planning/target_tcp_pose` | `geometry_msgs/msg/PoseStamped` | Queue one TCP task |
| `/perception/state/odom` | `nav_msgs/msg/Odometry` | Fresh base pose in `map_frame` |
| `/control/arm_joints` | `sensor_msgs/msg/JointState` | Default measured, inverse-calibrated URDF joints; configurable with `joint_state_topic` |
| `/perception/lidar/obstacle` | `geometry_msgs/msg/PoseArray` | LiDAR points; required for navigation by default |
| `/perception/vision/d435/obstacle` | `geometry_msgs/msg/PoseArray` | Additional vision obstacle points |
| `/perception/vision/d435/knob_observations` | `craft_perception/msg/KnobObservationArray` | Depth-valid knob targets |
| `/perception/vision/d435/puck_observations` | `craft_perception/msg/PuckObservationArray` | Axis-valid puck targets |
| `/perception/vision/object_pose` | `geometry_msgs/msg/PoseStamped` | Optional generic observation source |
| `/perception/vision/d435/object_detection` | `geometry_msgs/msg/PoseArray` | Generic object positions |
| `/control/task_result` | `craft_interfaces/msg/TaskResult` | ID-correlated aggregate result and controller matrix |
| `/control/status` | `craft_interfaces/msg/ControlStatus` | Ready/fault/held/measured stop status |
| `/planning/command` | `std_msgs/msg/String` | START, CANCEL, STOP, RESET |

## ROS outputs

| Topic | Type | Meaning |
| --- | --- | --- |
| `/planning/task_command` | `craft_interfaces/msg/TaskCommand` | Task ID, operation, revision and existing path/joints/slot/gripper/wait payload |
| `/planning/odom` | `nav_msgs/msg/Odometry` | Fresh validated odometry forwarded to control |
| `/planning/hold` | `std_msgs/msg/Bool` | Periodic all-axis hold level and watchdog heartbeat |
| `/planning/state` | `std_msgs/msg/String` | Idle, Ready, Run.Phase or Fault |
| `/planning/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Current task ID, queue size, path revision and fault reason |

Planning consumes no controller-specific completion Booleans. ControlMaster distributes
commands internally and publishes one aggregate result per task on `/control/task_result`.
Controller diagnostics can still be inspected independently.

Joint order is `slide_joint`, `shoulder_joint`, `elbow_joint`, `wrist_pitch_joint`,
`tool_roll_joint`. Seeds require these measured joints within URDF limits;
RViz's interpolated `/joint_states` is not hardware feedback. Arm encoder
feedback comes from the control side; point `joint_state_topic` at its topic.

IK branch choice: a TCP pose can have several in-limit joint solutions (elbow up/down).
`ArmPlanner` solves from the seed and from `max_restarts` fixed-seed random starts,
keeps every distinct solution, and returns the one with the least elbow load; loads
within 1 mm count as equal and then the solution nearest the seed wins. The URDF has
no inertias, so the load is the gravity moment about the elbow axis with equal masses
at the wrist pitch, tool roll and TCP origins (base +z up). Control interpolates in
joint space, so a far branch means a large swing; the measured puck poses have only
one in-limit solution.

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
`d435_puck`, `object_pose`, `object_detection`. For `d435_puck`, `class_id` is the
AprilTag ID printed on the puck underside. The highest-confidence fresh
matching target is transformed into the step frame. Configured XYZ is an offset
in that frame; the configured quaternion supplies orientation. Target selection
is frozen when calculation begins. `class_id: -1` accepts any class.

After `Ready`, send `START` on `/planning/command`, then show a configured floor/wall
mission tag or submit a goal/mission name manually. `tag_registry_file` points at the
shared `craft_perception/config/tag_registry.yaml`. Each tag ID has exactly one value:
`{mission: NAME}` or `{puck: NUMBER}`. Both nodes load that table into a hash map;
Planning uses mission entries and Perception uses puck entries. The table is empty until
the official allocation is available. Requests while Idle/Ready/Fault/Stopping are rejected without faulting.
New valid work is appended to a bounded FIFO (64 pending steps), not preempted.
Mission step labels are distinct from runtime task IDs. IDs are allocated on enqueue
from a system-clock epoch seed and increase across RESET; never reuse an ID in a run.
Repeated mission names are deduplicated until fault reset; direct pose requests
are deduplicated by type, frame, stamp and payload (last 1024 requests). Give an intentional
repeat a new acquisition/request stamp. Rejected requests are logged and not queued.

## Lifecycle and ownership

- `Idle`: map/IK/FSM configuration has loaded; wait for ControlMaster ready + stopped,
  fresh measured joints/odometry and required LiDAR. Initialization timeout enters Fault.
- `Ready`: configured and available; wait for explicit `START`.
- `Run.AwaitRequest`: hold and wait for queued work.
- `Run.PrepareTask`: release control and wait for current task inputs/observation.
- `Run.Compute`: path/IK calculation or validation of simple tasks. Bounded computation retries only.
- `Run.Execute`: await ControlMaster's result for the dispatched task ID.
- `Run.Complete`: record success through published state, clear current task, return to AwaitRequest.
- `Run.Stopping`: request stop; wait for fresh ControlStatus stopped and (if dispatched)
  matching TaskResult STOPPED. CANCEL returns to AwaitRequest, STOP returns to Ready.
- `Fault`: hold, discard pending tasks/calculation results, retain the original phase and reason.
  RESET waits for measured stop, reconfigures the planners, returns through Idle to Ready.
  It never automatically starts Run or resumes an old task.

All phases live under Run; there is no phase outside Run. Timeouts use monotonic time.
Completion and physical-stop criteria live in Control/controllers. Planning only verifies
IDs, message freshness and response deadlines. Faults clear the queue. Both cancel and
operation stop clear pending work. Lifecycle commands are accepted only on `/planning/command`.

During navigation, PathPlanner regenerates a path from the current pose/obstacle snapshot
every `replan_period_s` (default 0.2 s, at most one calculation in flight). Valid updates
preserve task ID and increase revision while staying in Run.Execute. Failed/expired replans
caused by a temporarily blocked route pause wheel speed and retry without dropping the task
or queue. Invalid input, resource-limit and expired computations still fault. Control follows
the planner's segments without spline overshoot.
PathPlanner owns collision checking; Planning does not implement another collision algorithm.
The previous valid path remains active while each calculation is running. A no-route result
pauses it until a later calculation succeeds. The period and compute timeout bound update
latency and must be tuned to actual robot speed and sensing latency.

Control turns the differential drive in place when the path direction differs by more than
`wheel_heading_tolerance`, then resumes Pure Pursuit. Optional `goal_heading_required` also
uses this mode for final alignment. Rotation speed is `wheel_rotate_angular_speed`. Default
arrival is position-only. Arm IK validates limits/pose, not self/environment collisions. Calibrate
motor mappings before operation: default arm/LM/gripper confirmations remain false.
Invalid startup configuration files fail node construction; runtime initialization and
operation timeouts are reported through Fault.

```bash
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: START}'
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: CANCEL}'
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: STOP}'
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: RESET}'
```

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
