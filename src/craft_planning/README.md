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
| `PathPlanner` | Static maps, footprint inflation, A*+arc (C++ port), dynamic obstacles and collision checks |
| `ArmPlanner` | Headless URDF FK/IK with Pinocchio and a measured joint seed |
| `planning_config.cpp` | Required typed ROS parameters and strict mission YAML parsing |

`configure()` validates parameters, loads the fixed map, URDF and mission file,
and configures the modules. `connect()` creates ROS interfaces, TF and the timer.
`planning_loop()` checks readiness, collects calculations, advances one step,
issues commands and reports state. Path/IK calculations run outside ROS callbacks;
only one is in flight. Cancellation/fault changes a generation so old
results are discarded. Only calculations may retry, never physical actions.

The installed executable is C++. `scripts/planning_master.py` forwards to the C++ executable. Python `src/path_planner.py` and
`src/astar_arc.py` remain importable references with regression tests. The C++
`PathPlanner` is a port of `astar_arc.py` (same costs, heap tie order and fillet search;
waypoints matched the Python planner on 40/40 random arena3_clean starts, ~24 ms vs ~560 ms).
Provenance remains in `third_party/`.
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
`r_robot + margin` on the A*+arc ESDF (centre distance minus half a cell). Dynamic points
are inflated in a private snapshot without modifying the static map.

A*+arc search: state (cell, heading E/N/W/S) from the robot's heading rounded to 90 degrees; one
cell forward costs `res * (1 + proximity)`, a 90 degree turn in place costs `lam`. Corners get
the largest collision-free fillet (bisection, `r_hi_global`); `R >= r_min` is `PASS`, else
`STOP_TURN` (also start, 180 degree reversals and the goal). With `goal_heading_required` the
last `approach_len` is straight (`ALIGN`). `PathPlan` carries the waypoints, per-pose speed
limits and `stops` (pose indices of intermediate `STOP_TURN` corners). A start just inside the
margin leaves straight to the nearest free cell within 0.10 m while `r_robot` is clear.
Requested start/goal XY is preserved. `max_cells`, `max_expansions` and sampling checks
bound work.

## Global / local paths (`path_planner.mode: global_local`)

The default since 2026-11-07. A Navigate step plans a **global path** once: 8-connected A* on the same C-space
(`r_robot + margin`, proximity cost) to the goal (or to the approach entry `approach_len` before it when
`goal_heading_required`), then line-of-sight pruning to an any-angle polyline. It is planned again only when the
driven path is blocked (pause, `block_wait_s`, replan) or a local path could not be renewed before its end.

The global polyline is then smoothed once, as a whole, like `astar8_spline.smooth_curve` (clearance band pad +
0.05 m, curvature above min(a_lat_max / v_ref^2, w_max / v_ref) pulled in, 600 steps, every point within
`local_max_dev` (0.20 m) of the polyline) and natural-cubic-splined at 0.02 m: that is the global path (2026-11-08;
before, each local window was smoothed on its own, so the local path left the global one).

The robot drives a **local path**: the samples of the global path in `[ego - local_behind, ego + local_ahead]`
(1 m behind, 4 m ahead = 5 m), cut from it unchanged, so a renewal never moves the path under the robot. After
`local_update` (2 m) of progress along the global path, planning_master sends the next window as UPDATE_PATH of the
same task; the last one ends exactly at the goal. A local path that ends short of the global end counts as a
segment: driven to its end without a renewal, the rest is planned from there.

Each window carries a **speed profile** (`TaskCommand.speed_limit`, per pose): min(v_ref, sqrt(a_lat_max / k),
w_max / k, and both wheels within `max_wheel` once the turn is added, v (1 + |k| wheel_separation / 2) /
wheel_radius <= max_wheel), then `a_lon_max` up from `v_min` at the window start and down to `v_min`, which it holds
over the last `goal_tolerance` (where control stops), never below `v_min`. The wheel boards follow steps at once
(Profile Acceleration at its maximum), so this profile is the driving ramp. The window starts `local_behind` behind
the robot, so a renewal does not slow a moving robot down. Control drives min(cruise_speed, the profile at its
nearest pose), speeding up at most `max_accel` from its last command (resume, end of an in-place turn).

### Driving up to a box (`/planning/approach_box`, 2026-11-08)

`/planning/approach_box` (std_msgs/String, a `box_approach.names` entry: L1, L2, L3) queues a Navigate step to that
box (`demo/corner/run_corner.py --box L2`). `PathPlanner::plan_box` makes one goal per face: `box_approach.gap` from
the LiDAR front face (`front_face_x` ahead of base_link) to the face, on the face normal through the box centre,
heading into the box. A face is approachable only when that goal, the approach entry `approach_len` behind it and the
line between are clear of the padding (inflated map + obstacles), the box's own padding aside: within padding of that
approach line, cells blocked only by the target box's padding are free (`BoxApproach`; the box itself, walls and
obstacles still block), so the goal may sit closer to the box than the padding (the upper cam gap, the final stop).
Renewals and the blocked-path check while driving use the same rule. Each approachable face is planned as a normal
global_local goal with the heading required, and the shortest global path wins. With the heading required the
approach is the straight `approach_len` along the goal heading at any angle (it used to round to a quarter turn), so
a box at any yaw works; the upper camera's gap only holds square to a face. The step drives at `box_approach.speed`
with the goal heading required; planning logs the chosen face and publishes its goal on `/planning/approach_goal`.
The planning.yaml box is the known position: the step plans to the stage points of its approachable faces at once,
and the perceived box (`/perception/box/pose`) takes over at the stage point. Phases of a box step:

1. Stage: `plan_box` with the known box and `stage_lead`: for every approachable face an ordinary path (no heading)
   to its stage point, `approach_len + stage_lead` behind the face goal on its normal (moved toward the entry in
   5 cm steps when in the padding); the shortest is driven at `speed`, arriving within `stage_goal_tolerance`.
   Stage points sit 0.43-0.58 m from the face (LiDAR front). (One path straight into the face could hairpin at the
   approach entry; Pure Pursuit cut it and turned back and forth for 40 s, 2026-11-08.)
2. Perceive: at the stage point the robot stands still `settle_s`, then takes a `/perception/box/pose` received
   after that (map frame, orientation = yaw of a face normal; `box_detection_timeout_s` in rates.yaml, at most
   `max_offset` from the known position). None within the timeout fails the step.
3. Dock: with the perceived box a new task drives the face normal itself, from
   the point on it nearest the robot to the face goal, at `dock_speed`, `goal_tolerance` = `dock_goal_tolerance` and
   `tolerance_rad` = `yaw_tolerance_deg`: control turns toward its lookahead point first, Pure Pursuit pulls the
   robot onto the normal (near the end the target stays lookahead ahead on the extended line) and the arrival
   counts along the line; it turns square at the goal. A dock or back-out that stays blocked `block_wait_s` fails
   (no global replan off the normal).
4. Settle: at the face goal the robot stands still `settle_s` (camera shake, the gap filter), then is checked against
   the goal of the same face of the box as perceived after settling: lateral offset and heading error within
   `lateral_tolerance` / `yaw_tolerance_deg` complete the step. `/planning/box_alignment` reports each check
   ([lateral m, yaw rad, forward m, aligned 0/1, correction]).
5. Reverse (correction parking): out of tolerance, the robot backs out `reverse_distance` straight along the normal
   (a new task, speed `-reverse_speed`, still facing the box), perceives again (2.), docks again and settles
   again; after `max_corrections` corrections the step fails. Blocked behind (padding, an obstacle), it backs
   out the longest clear distance down to `reverse_min_distance` in 5 cm steps; a blocked back-out or dock line waits
   and fails after `block_wait_s`.

The box detection itself is not part of planning (any node publishing `/perception/box/pose` works);
`demo/corner/run_corner.py --box L2 --fake-detect [--fake-offset DX DY DYAW]` stands in for it.

Global and local outputs pass a repair step (samples in a blocked cell corner, where a line-of-sight path grazes
the padding, are pushed out half a cell along the clearance gradient) so `collision_free` agrees with them.
Jetson (Orin Nano Super, 초레지 0.02 m, pad 0.293 m): global + first local 11-13 ms, a local renewal 4-9 ms;
figure `tools/path_compare/out/global_local.png`.

`mode: arc` keeps the previous behaviour (A*+arc, whole path, STOP_TURN segments). Comparison on 초레지:
`tools/path_compare` (8-connected + spline + curve: 7-10 % shorter, 7-15 % faster than A*+arc).

## Arena boxes in the A* map

`src/box_detector.py` (Python, installed as `craft_planning.box_detector`) finds the 205 x 205 mm boxes
(start boxes S1-S6, destinations L1-L3 of the project2 brief) in a ROS YAML map, at that map's resolution,
which is also the A* grid (YAML maps are not resampled). Hits on straight runs longer than 0.32 m are walls;
the rest is clustered, fragments merge while they fit one footprint, and a fixed 205 mm square is fitted to the
outline (yaw within +-15 degrees, 0.2 hit per degree). Box insides show as free in these maps, so with one face
seen the nearest brief position decides the side. `--write <base>` copies the map with every box drawn as its
full square outline, so A* and the global localizer see the unseen faces too.

```bash
/usr/bin/python3 src/box_detector.py ~/maps/arena5_legend_arena_raw.yaml --png boxes.png --write ~/maps/arena5_legend_arena
```

`~/maps/arena5_legend_arena` (corner_demo default) is the arena-frame map (`tools/mapmaker`, pipeline/make_arena_map.py)
with the boxes completed; `_raw` is the plain render. On it the boxes sit 57-275 mm from the brief positions.

## Puck graspable check

`grasp_region.hpp`, no IK. On every `/perception/vision/d435/puck_observations` message the nearest puck
is taken to `ik.base_frame` (centre and axis through TF) and judged: alignment first (lying: long axis
within `grasp_region.axis_tolerance_deg` of chassis forward, 180 degree symmetric; standing: any direction),
then the centre against that type's region `[x_min, x_max, y_min, y_max, z_min, z_max]` shrunk by `margin`.
Regions are measured by experiment for an approach along the long axis; min >= max means not measured and
never graspable. `confirm_count` consecutive graspable checks confirm it. Diagnostics: `puck_graspable`,
`puck_grasp_reason`, `puck_grasp_confirmed`, `puck_base_link` (use the last one while measuring regions).

`plan_stop()` picks the stop along the approach (chassis forward) that minimises
`J = w_box / (d_short - min_box_gap) + w_grasp * ((s - good_plane) / sigma)^2`: `d_short` is the shortest
distance between the front-face segment (LM guide plane) and the detected box square, `s` the front face
to puck centre distance, `sigma` half the region length in x. The box barrier is infinite at the 3 cm safety
gap; the grasp check must hold at the stop. 1 mm grid plus golden section (tens of microseconds).

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
      source: d435_puck
      class_id: 0
      frame: base_link
      pose: [0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0]
    - {id: close, type: gripper, close: true}
    - {id: store, type: clock, slot: 1}
    - {id: settle, type: wait, duration_s: 1.0}
```

Step IDs must be unique per mission. Unknown keys/types, invalid slots, frames,
poses and non-finite values fail parsing. Pose is `[x,y,z,qx,qy,qz,qw]` with a
nonzero quaternion. `source` defaults to `fixed`; alternatives are
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

A Navigate path goes to Control one segment at a time: up to the first `STOP_TURN` corner.
Control reports that segment SUCCEEDED; Planning stays in Run.Execute, plans the rest from
there and sends it as a new task ID. While driving, every `replan_period_s` Planning only
checks the rest of the segment against the current obstacles (~5 ms); it replans (at most one
calculation in flight, ~25-50 ms on arena3_clean) only when that is blocked, because the start
heading is rounded to 90 degrees and a replan inside a fillet would change the plan under the
robot. Such updates preserve task ID and increase revision. Failed/expired replans
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

## Mission (`mission_master`, 2026-11-12)

The dice memory mission (발제 2026-11-04, v5) runs one level above `planning_master`:
`ros2 launch craft_bringup craft.launch.py mission:=true`, then
`ros2 topic pub -1 /mission/command std_msgs/msg/String "{data: START}"` (also `STOP`, `RESET`); `/mission/state` shows
the state and the action being run. Settings: [config/mission.yaml](config/mission.yaml).

| File | What it holds |
| --- | --- |
| `mission_logic.hpp/.cpp` | barcode -> sort order, counts -> groups of L1-L3 (ties R, Y, G), 12 pucks -> allocation (Y one shape), dice spots |
| `mission_fsm.hpp/.cpp` | `Idle -> ReadBarcode -> WatchSignal -> Collect -> Allocate -> Deliver -> Verify -> Return -> Done`, any -> `Fault`; one `MissionAction` at a time, no ROS |
| `mission_master.cpp` | ROS node: perception inputs, drives through `/planning/approach_box` and `/planning/target_base_pose`, `signal.json` / `allocation.json` / `mission.log` under `log_dir` |

Collect picks every puck of every start box into the clock (12 slots) before the allocation is made, so R/G cannot take
what Y needs. A fault (unregistered barcode, invalid signal, incomplete survey, no allocation, a failed or timed-out
action) sends planning `STOP` and stays in `Fault` until `RESET` or a new `START`.

**Skeletons** (`run_skeleton` in `mission_master.cpp`: logged as `[SKELETON]`, done after `skeleton_action_s`):
`observe_pucks`, `pick_puck`, `place_puck`, `verify_location`. The pick must report the puck id it took and check
`/control/gripper_grasped` (control latches a grasp at >= 0.1 A for 0.1 s; a close that ends without it held nothing).
Until then `assume_unknown_picks: true` hands the surveyed ids out to the slots so the sequence can be walked through.

`box_approach.square_boxes` (L1-L3): these boxes are not turned, so they take the map yaw and skip the second stage
drive + LiDAR measurement of a box step.
