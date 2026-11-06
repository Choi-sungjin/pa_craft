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
float64 speed
int32 clock_slot
bool close
float64 duration_s
# MANIPULATOR: arrival tolerance of the LM and the four arm motors [motor rad]; 0 = control.yaml (0.5 deg).
# Larger only for moves whose end needs no precision (via poses, 2026-11-07); at most 5 deg.
float64 tolerance_rad
# MANIPULATOR: Position I gain of the shoulder, elbow, wrist_pitch and tool_roll motors from this move on (written
# before its targets; 0..16383, XM430 units); empty = the board's power-up gains (firmware config.h kPositionIGains)
# (2026-11-07: lower I for moves against gravity).
uint16[] position_i_gains
