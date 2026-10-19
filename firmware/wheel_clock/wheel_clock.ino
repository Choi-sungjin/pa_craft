// OpenCR #1. USB RX: wheel_l [rad/s],wheel_r [rad/s],clock [unwrapped rad]\n
// USB TX: wheel_l,wheel_r,clock,wheel_state,clock_state\n (50 Hz)
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<3> motor_board(motor_bus, kAxes, kHardwareConfirmed, kDxlBaud);

void setup() { motor_board.begin(); }
void loop() { motor_board.tick(); }
