// OpenCR #2. All command/feedback positions are motor radians (LM is not metres).
// USB RX: LM,shoulder,elbow,wrist_pitch,tool_roll,gripper\n
// USB TX: same six positions,arm_lm_state,gripper_current_a\n (50 Hz)
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<6> motor_board(
    motor_bus, kAxes, kHardwareConfirmed, kDxlBaud, kGripperCurrentAPerTick);

void setup() { motor_board.begin(); }
void loop() { motor_board.tick(); }
