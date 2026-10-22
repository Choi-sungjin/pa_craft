# Wheel and clock targets

The control node consumes these planning inputs:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/planning/target_v` | `std_msgs/Float64` | Signed chassis forward speed, m/s |
| `/planning/target_path` | `nav_msgs/Path` | Ordered points from the current position toward the goal |
| `/planning/odom` | `nav_msgs/Odometry` | Chassis pose at the wheel axle center |
| `/planning/target_clock_slot` | `std_msgs/Int32` | Target storage slot, 1 through 12 |
| `/planning/hold` | `std_msgs/Bool` | Periodic hold/release heartbeat for every axis |

The C++ planning master publishes `/planning/hold` every scheduling tick. Control
starts held, acknowledges `/control/planning_held`, and accepts targets only after
the acknowledgement is false. `true`, a heartbeat gap longer than
`planning_watchdog_timeout_s` (default 0.5 s), or a board fault clears all stored
and pending targets. Release needs new targets; it never resumes an old one.
These deadlines use a steady clock, including when ROS simulation time pauses.
Standalone control clients must supply this heartbeat too. The explicit
`require_planning_heartbeat=false` compatibility setting permits operation before
the first heartbeat; after one is received the watchdog remains enforced.

Each board also has a `serial_feedback_timeout_s` receive deadline (default
0.5 s). No first sample keeps control held. Once a board has reported feedback,
stale feedback is reported as state 2 with completion false. Stale-board command
transmission stops so the existing firmware watchdog can hold using its own live
motor measurements. Healthy boards continue receiving hold commands.

`/control/arm_joints` (`sensor_msgs/JointState`) supplies the planning IK seed:
`[slide_joint, shoulder_joint, elbow_joint, wrist_pitch_joint, tool_roll_joint]`,
with units `[m, rad, rad, rad, rad]`, `frame_id=base_link`. It is emitted only for
fresh healthy arm feedback with both arm and LM mappings confirmed and valid
calibration. Values invert the configured motor offsets, directions and gearing;
they are measured feedback, not the previous command or RViz interpolation.

Physical dimensions, limits, tracking tolerances, and clock geometry are defined
in `config/control.yaml`. `ControlMaster` reads and validates the ROS parameters,
then passes them to `ArmController`, `LmController`, `GripperController`, `ClockController`,
`LongitudinalController`, and `LateralController`. Controllers do not read YAML
or declare ROS parameters themselves.

## Arm, LM and gripper responsibilities

`ArmController` handles four rotational joints only: shoulder, elbow, wrist_pitch,
and tool_roll. Its limits, joint names, directions and offsets each contain four
values in YAML. `LmController` handles the slide joint independently, converting
linear displacement in metres to motor radians:

```text
lm_motor_angle = lm_motor_offset + lm_motor_direction * lm_gear_ratio * position_m / lm_pulley_radius
```

LM motor limits and calibration use separate `lm_*` parameters, including
`lm_mapping_confirmed`. Zero pulley radius and gear ratio disable LM targets and
hold its measured motor angle; they do not disable the rotational arm controller.
LM limits retain the former slide channel's values, in motor radians, pending
hardware calibration. `GripperController` remains independent.

The existing planning topic `/planning/target_arm_joints` still carries the full
five-joint IK result: slide [m], shoulder, elbow, wrist_pitch, tool_roll [rad].
Every planning input follows the same flow. The subscription callback only keeps
the newest message pending; `control_loop()` validates it, then stores it with a
setter that does no validation, or clears the target when validation fails:

```text
validate_arm_target      -> set_arm_target      | clear_arm_target
validate_lm_target       -> set_lm_target       | clear_lm_target
validate_gripper_target  -> set_gripper_target  | (ignored)
validate_wheel_target    -> set_wheel_target    | clear_wheel_target
validate_path_target     -> set_path_target     | clear_path_target
validate_odom            -> lateral_.set_odom   | lateral_.clear_odom
validate_clock_target    -> set_clock_target    | clear_clock_target
```

The manipulator JointState is split into LM and four rotational positions first.
An invalid target means the previous target is no longer trustworthy, so the
axis **stops** (holds its current position, or zero wheel speed) until planning
sends a new valid target. An empty path is planning's "no route" result and also
stops tracking. Arm, LM and gripper use independent confirmation flags; while a
mapping is unconfirmed its targets are ignored. The clock slot is applied after
fresh feedback, because the shortest turn depends on the current angle. Pending
messages are consumed once, rather than reapplying targets on every timer tick.
Planning remains responsible for coordinating these independently controlled axes.

Every controller uses the same method names:

| Role | Method |
| --- | --- |
| Configuration | `configure(...)` (validated once; failure stops the node) |
| Target validation | `is_valid_target(...)` (gripper open/close needs none) |
| Store / release target | `set_target(...)` / `clear_target()` |
| Motor command | `compute_command(feedback)` |
| Arrival | `is_target_reached(feedback)` |
| Derived state | `get_*()` (e.g. `ClockController::get_current_slot`) |

Odometry is feedback, not a target: `LateralController::is_valid_odom`,
`set_odom` and `clear_odom`.

Configuration (limits, mappings, tolerances) is validated once in the
ControlMaster constructor; an invalid value stops the node. Controllers therefore
keep no "configured" flags and do not re-check configuration at run time.

Only the transport is shared. Feedback positions are stored separately as
`lm_angle_`, four `arm_joints_`, and `gripper_angle_`; commands are assembled in
the existing OpenCR order: slide, shoulder, elbow, wrist_pitch, tool_roll, gripper.

## Differential drive

Path and odometry must have the same nonempty `header.frame_id`. Transform them in
planning before publishing if necessary. Pose coordinates follow ROS convention:
forward +x, left +y, counterclockwise positive yaw. Pose orientations in the path
are not used; this controller tracks positions, without a final heading maneuver.

Incoming path points are joined by a natural cubic spline parameterized by
cumulative chord length and resampled every `path_spacing` (the first and last
points are kept exactly). The resampled path is tracked and republished on
`/control/tracking_path` for visualization; an empty path is published when the
target is cleared. Sharp corners may overshoot slightly between points.

The longitudinal controller limits `target_v`. Pure Pursuit
selects a path point at least `lookahead_distance` away, starting at the nearest
path point; it uses the last available point if the remaining path is shorter.
With that point transformed into the chassis frame `(x, y)`:

```text
curvature = 2*y / (x*x + y*y)
omega = limited_v * curvature
left  = limited_v / wheel_radius - omega * wheel_separation / (2*wheel_radius)
right = limited_v / wheel_radius + omega * wheel_separation / (2*wheel_radius)
```

Both wheel commands are scaled by the same factor if either exceeds `max_wheel`.
Commands are wheel angular velocities in rad/s. Wheel centers are 0.288 m apart;
wheel radius is 0.033 m. Tune lookahead and endpoint tolerance on the robot.

Empty/invalid paths, missing/invalid odometry, differing frames, and endpoint
arrival stop the wheels. The next valid path uses
the most recently received `target_v`; planning should publish zero when canceling
a driving task. Negative target speeds pursue path points behind the chassis.
Pure Pursuit does not generate an in-place turn at zero linear speed or when no
point lies in the requested direction of travel.

The host owns path tracking, differential-drive conversion, and `max_v` /
`max_wheel` limits. OpenCR firmware owns DYNAMIXEL configuration and must set a
nonzero `Profile Acceleration(108)` in Velocity Control Mode. The DYNAMIXEL then
owns the acceleration profile, velocity PID, feedforward, and motor-level
tracking. Keeping an additional host acceleration ramp would duplicate that
profile and make the resulting acceleration depend on two independent layers.

## Clock storage

The default motor angle convention is **0 rad = slot 12, positive motor rotation
is counterclockwise and increases slot numbers, adjacent slots differ by 30 motor
degrees**. No additional gear conversion is applied. The angle command is chosen
once, from fresh feedback, as the nearest equivalent of the requested slot:

```text
slot_motor_angle = clock_zero_angle + clock_direction * (slot % clock_slot_count) * clock_slot_angle
revolution_angle = clock_slot_count * clock_slot_angle
delta = remainder(slot_motor_angle - current_motor_angle, revolution_angle)
target_motor_angle = current_motor_angle + delta
```

`clock_slot_count=12`, `clock_slot_angle=pi/6`, `clock_zero_angle=0`, and
`clock_direction=1` are supplied by YAML through `ControlMaster`. The final slot
(`clock_slot_count`, normally 12) is the zero-reference slot. Slot spacing is in
motor radians, so it can account for gearing if needed. `clock_direction` must
be +1 or -1 and tolerance must be smaller than half the configured slot spacing.

For example, from 0 rad at slot 12, slot 7 commands -5*pi/6 rad (150 degrees
clockwise). An exact half-turn may use either direction. Repeating the same slot
does not recalculate or change the latched target angle during motion.

OpenCR must accept and report **unwrapped motor angles in radians**, including
negative angles and angles above 2*pi, and track the commanded absolute angle.
The implementation is in `firmware/wheel_clock`; see `firmware/README.md` for
hardware configuration and builds. Feedback lists the channels shared with the
command first, in the same order, then the board-specific state:

```text
host -> OpenCR: wheel_l,wheel_r,clock_motor_angle
OpenCR -> host: wheel_l_feedback,wheel_r_feedback,clock_motor_angle,wheel_state,clock_state
```

The control node reports on each new valid serial sample:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/control/clock_slot` | `std_msgs/Int32` | Aligned slot 1..12 when idle; 0 while moving, between slots, or faulted |
| `/control/clock_target_reached` | `std_msgs/Bool` | A target exists, its command was sent, motor is idle, and angle is within tolerance |
| `/control/clock_state` | `std_msgs/Int32` | OpenCR state: 0 idle, 1 using, 2 fault |

`clock_angle_tolerance` defaults to 2 degrees. A clock target waits for fresh
feedback, is validated and stored there, and immediately publishes `false` on
`/control/clock_target_reached`. An invalid slot clears the target and holds position. Before any target, completion is false even if aligned.

Planning owns sequencing and puck inventory. It should record the pending target
slot, wait for `/control/clock_target_reached=true` **and** `/control/clock_slot` matching that
target, and then issue the next clock command or arm loading/retrieval target.
These reports are live feedback, not latched inventory records. No arm interlock
is imposed by control. The current planning skeleton does not yet implement this
mission sequence or produce a target speed.

## Target reached topics

Each axis reports arrival as `std_msgs/Bool`, published on fresh feedback after a
successful OpenCR write. A newly stored target immediately publishes `false`, so a
previous `true` is never mistaken for completion of the new target.

| Topic | True when |
| --- | --- |
| `/control/arm_target_reached` | All four rotational motor angles within `arm_angle_tolerance` of the target motor angles |
| `/control/lm_target_reached` | LM motor angle within `lm_angle_tolerance` of the target motor angle |
| `/control/gripper_target_reached` | Open: within `gripper_angle_tolerance` of the open angle. Close: within tolerance of the close angle, or grasped |
| `/control/wheel_target_reached` | Path endpoint within `wheel_goal_tolerance` of odometry (same frame) |
| `/control/clock_target_reached` | See Clock storage |

Planning republishes the same path each tick, so only a changed path resets
`/control/wheel_target_reached`. Every motor tolerance is 2 degrees. All tolerances come from `config/control.yaml`, are
validated as positive by ControlMaster, and are stored in each controller at
configuration, so every controller exposes `is_target_reached(feedback)`.
Clock completion additionally requires `clock_state` idle, checked by ControlMaster.
Targets outside motor limits are not expected: planning accounts for them in IK.

## Verification

```bash
colcon build --packages-select craft_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select craft_control --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover clock shortest turns and alignment, completion/fault handling, wheel
speed limits, straight/curved/reverse paths, coordinate transforms, goal
arrival, and invalid input cancellation.
