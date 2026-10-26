# Wheel and clock targets

ControlMaster receives `/planning/task_command` (`craft_interfaces/TaskCommand`),
`/planning/odom` and the periodic `/planning/hold` heartbeat. It distributes each task:

| Kind | Participants |
| --- | --- |
| NAVIGATE | Wheel (longitudinal + lateral) |
| MANIPULATOR | Arm + LM, validated together before either target is applied |
| GRIPPER | Gripper |
| CLOCK | Clock |
| WAIT | ControlMaster monotonic timer |

`/control/task_result` (`TaskResult`) carries task ID, path revision, aggregate status,
reason, and five matrix columns `[Arm, LM, Wheel, Clock, Gripper]`. UNUSED columns do
not gate completion. Active participants must all report arrival and settle before
SUCCEEDED. Failure stops the participating work and holds all axes. A stopped task is
reported STOPPED only after measured settling, not merely because hold was requested.
Results repeat every control tick; duplicate EXECUTE requests do not re-execute.
Retired task IDs cannot be reused. Updates require the active navigation ID, an increasing
revision, the same goal/frame, and a valid path. They do not reset the task execution timeout.
New independent tasks are rejected while one is running/stopping; the FIFO lives in Planning.

`/control/status` reports ready, fault, held, stopped and reason every tick. Ready requires
fresh board feedback and confirmed arm/LM/gripper calibration. Stopped requires successful
writes, fresh idle-board feedback, stable measured actuator positions over `settle_time_s`,
and both measured wheel speeds within `wheel_stop_tolerance_rad_s`. Position stability uses
`stop_position_tolerance_rad`. Planning consumes these aggregate reports only.

Control starts held. Planning waits for held=false before dispatching a task. A true/missing/
stale heartbeat (`planning_watchdog_timeout_s`) or board fault clears targets and holds current
positions. Release cannot revive work. Task/serial/odometry deadlines use monotonic time,
including when `/clock` pauses. The optional `require_planning_heartbeat=false` permits operation
before the first heartbeat; once received, heartbeat expiry is always enforced.

Each board also has a `serial_feedback_timeout_s` receive deadline (default
0.5 s). No first sample keeps control held. Once a board has reported feedback,
stale feedback is reported as state 2 with completion false. Stale-board command
transmission stops so the existing firmware watchdog can hold using its own live
motor measurements. Healthy boards continue receiving hold commands.

`/control/arm_joints` (`sensor_msgs/JointState`) supplies the planning IK seed:
`[slide_joint, shoulder_joint, elbow_joint, wrist_pitch_joint, tool_roll_joint]`,
with units `[m, rad, rad, rad, rad]`, `frame_id=base_link`. It is emitted only for
fresh healthy arm feedback with both arm and LM mappings confirmed and valid
calibration. Values invert the configured motor offsets, directions and measured LM conversion;
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
lm_motor_angle = lm_motor_offset + lm_motor_direction * position_m / lm_meters_per_rad
```

LM motor limits and calibration use separate `lm_*` parameters, including
`lm_mapping_confirmed`. A zero `lm_meters_per_rad` disables LM targets and
holds its measured motor angle; this does not disable the rotational arm controller.
`lm_meters_per_rad` is measured travel divided by motor rotation in radians; separate
pulley radius and gear ratio parameters are unnecessary. LM calibration uses
108 mm travel over -141.4 to 808.1 degrees, with 249.4 degrees as slide zero. `GripperController` remains independent.

Direction settings assume hardware Reverse OFF. With URDF +Y forward and +Z up
at the reference pose, `arm_joint_directions` is `[+1, +1, -1, +1]` in
shoulder, elbow, wrist_pitch, tool_roll order; `lm_motor_direction` is `+1`
(URDF `slide_joint` axis is -X, so + moves toward the LM motor, same as motor raw +).
These arm signs were inferred from the reported physical motions at the reference
pose and still require hardware validation with calibrated offsets. Arm firmware
preserves raw motor signs; apply the URDF sign correction only in host YAML.
Wheel signs are applied in firmware: left `+1`, right `-1`. Clock uses firmware
`+1`, host `clock_direction: +1`, and host zero angle 314.6 degrees (slot 12).
Gripper closes toward 229 degrees (5 degrees inside the 234-degree hard stop) from its
94-degree open pose; its opposite closed endpoint is -53.1 degrees (limit -48.1).
Grasp is detected by current (0.1 A for 0.1 s), not by reaching the close target. It uses open/close targets rather than an IK sign.

Arm zero offsets and wrist pitch limits remain uncalibrated. Mapping confirmation
flags and both firmware `kHardwareConfirmed` flags remain false. No board upload
or physical motion validation has been performed for these settings.

`TaskCommand.joints` carries the five-joint IK result: slide [m], then shoulder,
elbow, wrist_pitch and tool_roll [rad]. ControlMaster validates the complete manipulator
command before setting Arm and LM targets; invalid or uncalibrated requests are rejected.
The callback queues commands and the control loop applies them once. ControlMaster aggregates
independent controller results; Planning owns mission ordering. Legacy controller-specific
planning target subscriptions have been removed to prevent bypassing the task protocol.

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
forward +x, left +y, counterclockwise positive yaw. Large path-heading errors and optional
final arrival-yaw errors use an in-place differential turn before path tracking continues.

Incoming planner segments are subdivided to at most `path_spacing`, preserving all
vertices and avoiding spline overshoot outside the checked path. `/control/tracking_path`
remains a visualization output. Path endpoint position and optional requested goal yaw are
checked in LateralController; `wheel_heading_tolerance` is configured in control YAML.

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
the speed carried in the task command; STOP or the hold heartbeat cancels motion. Negative target speeds pursue path points behind the chassis.
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
OpenCR -> host: wheel_l_feedback,wheel_r_feedback,clock_motor_angle,wheel_state,clock_state,
                imu_ok,gz
```

Each sample also publishes `/control/wheel/odom` (wheel dead reckoning with twist covariance) and,
when `imu_ok=1`, `/control/imu/data` in `imu_link` (OpenCR board-frame yaw rate only, minus
`imu_gyro_z_bias`; no orientation or acceleration). `odom->base_link` comes from robot_localization
(`craft_perception/config/ekf.yaml`); set `publish_odom_tf: true` only when running without it.

The control node reports on each new valid serial sample:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/control/clock_slot` | `std_msgs/Int32` | Aligned slot 1..12 when idle; 0 while moving, between slots, or faulted |
| `/control/clock_target_reached` | `std_msgs/Bool` | A target exists, its command was sent, motor is idle, and angle is within tolerance |
| `/control/clock_state` | `std_msgs/Int32` | OpenCR state: 0 idle, 1 using, 2 fault |

`clock_angle_tolerance` defaults to 2 degrees. A clock target waits for fresh
feedback, is validated and stored there, and immediately publishes `false` on
`/control/clock_target_reached`. An invalid slot clears the target and holds position. Before any target, completion is false even if aligned.

Planning owns sequencing and puck inventory. It waits for the matching aggregate task
result before advancing. ClockController checks the target angle and ControlMaster checks
fresh idle/settled board feedback. Diagnostic slot/reached topics are not task acknowledgements.

## Target reached topics

Each axis reports arrival as `std_msgs/Bool`, published on fresh feedback after a
successful OpenCR write. These Booleans are diagnostic only. Task ID correlation and completion aggregation use TaskResult.

| Topic | True when |
| --- | --- |
| `/control/arm_target_reached` | All four rotational motor angles within `arm_angle_tolerance` of the target motor angles |
| `/control/lm_target_reached` | LM motor angle within `lm_angle_tolerance` of the target motor angle |
| `/control/gripper_target_reached` | Open: within `gripper_angle_tolerance` of the open angle. Close: within tolerance of the close angle, or grasped |
| `/control/wheel_target_reached` | Path endpoint within `wheel_goal_tolerance` of odometry (same frame) |
| `/control/clock_target_reached` | See Clock storage |

A new path revision resets
`/control/wheel_target_reached`. Arrival tolerances are configured per controller. All tolerances come from `config/control.yaml`, are
validated as positive by ControlMaster, and are stored in each controller at
configuration, so every controller exposes `is_target_reached(feedback)`.
Clock completion additionally requires `clock_state` idle, checked by ControlMaster.
Targets outside motor limits are not expected: planning accounts for them in IK.

## Verification

```bash
colcon build --packages-up-to craft_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select craft_control --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover clock shortest turns and alignment, completion/fault handling, wheel
speed limits, straight/curved/reverse paths, coordinate transforms, goal
arrival, and invalid input cancellation.
