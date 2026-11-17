# EXECUTE is idempotent. UPDATE_PATH preserves the task ID and advances revision.
uint8 EXECUTE=0
uint8 UPDATE_PATH=1
uint8 STOP=2
uint8 NAVIGATE=0
uint8 MANIPULATOR=1
uint8 GRIPPER=2
uint8 CLOCK=3
uint8 WAIT=4
std_msgs/Header header
uint64 task_id
uint64 revision
uint8 operation
uint8 kind
nav_msgs/Path path
sensor_msgs/JointState joints
bool require_goal_heading
# NAVIGATE: [m/s]; negative drives the path in reverse (the robot keeps its heading, e.g. backing out of a box).
float64 speed
# NAVIGATE: speed profile per path pose [m/s] (planning: curvature, wheel speed limit, acceleration); the robot drives
# min(speed, the profile at its nearest pose). Empty = speed everywhere.
float64[] speed_limit
int32 clock_slot
bool close
# WAIT: wait duration. NAVIGATE + continuous_tracking: whole-task deadline [s], 0 = configured deadline,
# positive values at most 3600 s. Updates do not reset the deadline.
float64 duration_s
# NAVIGATE: fine in-place alignment [rad] before moving off and at the goal heading; 0 = control.yaml.
# MANIPULATOR: arrival tolerance of the LM and the four arm motors [motor rad]; 0 = control.yaml (0.5 deg).
# Larger only for moves whose end needs no precision (via poses, 2026-11-07); at most 5 deg.
float64 tolerance_rad
# MANIPULATOR: Position I gain of the shoulder, elbow, wrist_pitch and tool_roll motors from this move on (written
# before its targets; 0..16383, XM430 units); empty = the board's power-up gains (firmware config.h kPositionIGains)
# (2026-11-07: lower I for moves against gravity).
uint16[] position_i_gains
# NAVIGATE: arrival distance [m]; 0 = control.yaml wheel_goal_tolerance (a box dock: 1 cm).
float64 goal_tolerance
# NAVIGATE: base-frame reference x and path lookahead [m]; both zero use the legacy speed table.
float64 tracking_reference_x
float64 lookahead_distance
# Camera distance longitudinal control: latest acquisition and remaining gap [m].
bool camera_distance_control
float64 camera_gap_m
float64 camera_target_gap_m
builtin_interfaces/Time camera_observation_stamp

# NAVIGATE: follow bends while translating, without automatic in-place realignment. Default false preserves
# ordinary tasks. If the path lies 90 deg or more behind the travel heading, hold still instead of turning.
# Does not bypass obstacle holds, watchdogs, speed limits or final requested heading alignment.
bool continuous_tracking
