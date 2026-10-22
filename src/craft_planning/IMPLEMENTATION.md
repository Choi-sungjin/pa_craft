# C++ planning integration contract

## Objective and module boundaries

`planning_master` owns configure/connect, ROS I/O, input snapshots, asynchronous
work and scheduling. `StateMachine`, `ArmPlanner` and `PathPlanner` are ordinary
C++ classes. The existing Python KANT implementation and its tests remain as a
reference; the installed master uses the native C++ static-grid A* planner.
The supplied Pinocchio IK core and URDF are reused without the RViz demo node.

Common helper names follow the other masters: `param()` and `positive_param()`
read configuration, and `is_fresh_input()` checks ROS acquisition timestamps as
in perception. `Sample::observe()` records input and its receive time, and
`Sample::fresh()` checks the receive watchdog, following control's watchdog API.

| Module | Responsibility | Dependencies |
| --- | --- | --- |
| state-machine | Readiness, compute/execution deadlines, debounced completion, cancellation | C++ standard library |
| path-planner | Fixed map loading, footprint inflation, A*, dynamic point obstacles, path validation | ROS message types, yaml-cpp, PNG |
| arm-planner | URDF FK/IK with an explicit measured joint seed | Supplied v75 IK core, Pinocchio |
| planning-master | Typed configuration, mission steps, TF, watchdogs, communication | All three modules |

Build order: independent module tests, master integration, control heartbeat,
ROS integration tests, review. C++17; fail invalid configuration at startup.

## Execution contract

- Startup is IDLE and holds motion. No example mission starts automatically.
- Existing base/TCP goal topics create one-step requests; YAML defines named
  missions made of navigate/TCP/gripper/clock/wait steps. Repeated mission names
  do not restart a mission. INIT is informational; STOP/CANCEL cancel, RESET
  clears the fault latch. A fault requires explicit RESET before new goals.
- FSM states: IDLE, WAITING, COMPUTING, EXECUTING, SUCCEEDED, FAULT, CANCELLED.
- Readiness, computation, execution, sensor and feedback deadlines are explicit
  positive YAML parameters. Timeouts use a steady clock. Physical actions are
  never automatically retried; only failed computations have bounded retries.
- A generation invalidates asynchronous results after cancellation/replacement.
  At most one computation is in flight. Calculations run outside ROS callbacks.
- TCP goals are base_link poses (metres/quaternions); IK requires fresh measured
  JointState, not the RViz model's interpolated state. Current five-joint names
  and limits must match the control contract.
  The default source is `/control/arm_joints`; `/arm/odom` can be configured when
  a separate measured joint encoder source is available.
- Base goals/odom/static map use one frame. Perception observations are
  transformed at their acquisition timestamp before becoming explicit goals.
  Observation offsets/orientations and mission order are configured, not guessed.
- LiDAR points and vision obstacles are transformed into the map frame and
  checked against the footprint. The controller's resampled tracking path must
  also pass collision checking before a nonzero speed is sent. Unsupported
  initial/final orientation requirements stop rather than claim success.
- Completion Booleans must observe a post-command false before true and remain
  fresh/stable. Wheel completion also checks goal distance; clock completion
  also checks the slot and idle state. Legacy Bool topics lack goal IDs, so
  arbitrary delayed previous false/true pairs cannot be rigorously correlated.
- `/planning/hold` (Bool, periodic) is both a level and a heartbeat. Control
  acknowledges through `/control/planning_held`. Missing/true/stale heartbeat
  clears targets and holds current positions; release never resumes old targets.
  Master waits for a fresh released acknowledgement before sending commands.
- Control separately watches each board's serial receive deadline. Missing
  feedback keeps startup held; expired previously valid feedback reports fault.
  It stops transmitting to a stale board so firmware can hold its own measured
  position rather than continually receiving an old host-side position.
- Stop sends zero speed, an empty path and hold=true. Gripper is held, not
  automatically opened. Firmware fault handling remains authoritative.

## Verification and boundaries

Build with ROS Lyrical sourced, Pinocchio available via CMAKE_PREFIX_PATH:

```bash
colcon build --base-paths src --packages-select craft_planning craft_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select craft_planning craft_control --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover FSM deadlines/cancellation/old completion, native map obstacles and
frames, real FK-to-IK round trips, control hold recovery, and a ROS run with
synthetic sensor/control inputs. Tests must not open serial ports or start the
full hardware bringup. User-added nested v75 workspace and removed old paths are
preserved. No hardware calibration, invented physical mission geometry, or
automatic real-robot operation is part of this change.

### Verified on 2026-10-22

Both native packages built in Release against ROS Lyrical and its Pinocchio 4.1.0
distribution. Build/install artifacts and extracted dependency packages stayed
under `/tmp`; system packages and the source workspace's install tree were not
modified. The initial pip Pinocchio dependency combination crashed during ROS
node construction; using the matching ROS dependency distribution resolved it.

| Check | Result |
| --- | --- |
| Planning C++ FSM/path/IK/mission-parser tests | 55 passed |
| Control C++ controller/watchdog/calibration tests | 36 passed |
| Original Python path-planner regressions | 17 passed |
| Real C++ master with synthetic ROS peers | 5 passed |
| Control with two disposable PTY boards | Passed |
| Installed default YAML, STL map and URDF startup/shutdown | Passed |

Master runtime tests cover tracking-path acknowledgement, old completion rejection,
native TCP IK, stale odometry/fault/reset, actual footprint collision, observed
target missions, gripper/carousel/wait sequencing, mission deduplication and STOP.
Empty obstacle arrays require no TF lookup; actual obstacle points still require
successful transformation. A runtime regression covers both cases.
The PTY test covers every-axis hold/release, stale planning heartbeat, independent
serial feedback loss, firmware-watchdog handoff, recovery, measured joint output,
unconfirmed calibration and a paused ROS clock.

The existing perception inference dependency (ncnn) was unavailable in this
environment. Planning compiled against a temporary interface-only package
generated from the **unchanged original perception `.msg` files**. The complete
perception inference build and real robot operation were not verified.

ROS APIs are checked against installed Lyrical headers; references:
https://github.com/ros2/rclcpp/tree/rolling/rclcpp/include/rclcpp
https://github.com/ros2/geometry2/tree/rolling/tf2_ros/include/tf2_ros
