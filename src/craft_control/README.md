# Wheel and clock targets

The control node consumes these planning inputs:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/planning/target_v` | `std_msgs/Float64` | Signed chassis forward speed, m/s |
| `/planning/target_path` | `nav_msgs/Path` | Ordered points from the current position toward the goal |
| `/planning/odom` | `nav_msgs/Odometry` | Chassis pose at the wheel axle center |
| `/planning/target_clock_slot` | `std_msgs/Int32` | Target storage slot, 1 through 12 |

Physical dimensions, limits, tracking tolerances, and clock geometry are defined
in `config/control.yaml`. `ControlMaster` reads and validates the ROS parameters,
then passes them to `ArmController`, `GripperController`, `ClockController`,
`LongitudinalController`, and `LateralController`. Controllers do not read YAML
or declare ROS parameters themselves.

## Differential drive

Path and odometry must have the same nonempty `header.frame_id`. Transform them in
planning before publishing if necessary. Pose coordinates follow ROS convention:
forward +x, left +y, counterclockwise positive yaw. Pose orientations in the path
are not used; this controller tracks positions, without a final heading maneuver.

The longitudinal controller limits `target_v` and its acceleration. Pure Pursuit
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
arrival stop the wheels and reset the velocity ramp. The next valid path uses
the most recently received `target_v`; planning should publish zero when canceling
a driving task. Negative target speeds pursue path points behind the chassis.
Pure Pursuit does not generate an in-place turn at zero linear speed or when no
point lies in the requested direction of travel.

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
Firmware is outside this change. The serial format remains:

```text
host -> OpenCR: wheel_l,wheel_r,clock_motor_angle
OpenCR -> host: wheel_state,clock_state,wheel_l_feedback,wheel_r_feedback,clock_motor_angle
```

The control node reports on each new valid serial sample:

| Topic | Message | Meaning |
| --- | --- | --- |
| `/clock_slot` | `std_msgs/Int32` | Aligned slot 1..12 when idle; 0 while moving, between slots, or faulted |
| `/clock_target_reached` | `std_msgs/Bool` | A target exists, its command was sent, motor is idle, and angle is within tolerance |
| `/clock_state` | `std_msgs/Int32` | OpenCR state: 0 idle, 1 using, 2 fault |

`clock_angle_tolerance` defaults to 1 degree. A valid new target immediately
publishes `false` on `/clock_target_reached`, and waits for fresh feedback before
choosing its angle. Before any target, completion is false even if aligned.

Planning owns sequencing and puck inventory. It should record the pending target
slot, wait for `/clock_target_reached=true` **and** `/clock_slot` matching that
target, and then issue the next clock command or arm loading/retrieval target.
These reports are live feedback, not latched inventory records. No arm interlock
is imposed by control. The current planning skeleton does not yet implement this
mission sequence or produce a target speed.

## Verification

```bash
colcon build --packages-select craft_control --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select craft_control --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover clock shortest turns and alignment, completion/fault handling, wheel
acceleration limits, straight/curved/reverse paths, coordinate transforms, goal
arrival, and invalid input cancellation.
