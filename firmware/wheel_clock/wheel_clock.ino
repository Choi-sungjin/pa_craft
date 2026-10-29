// OpenCR #1. USB RX: wheel_l [rad/s],wheel_r [rad/s],clock [unwrapped rad]\n
// USB TX: wheel_l,wheel_r,clock,wheel_state,clock_state,imu_ok,gx,gy,gz,ax,ay,az\n (50 Hz)
// IMU values: raw board-frame means (CraftImu.h); the host removes the gyro offset and finds the
// mounting from gravity while the robot stands still.
#include <CraftImu.h>
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<3> motor_board(motor_bus, kAxes, kHardwareConfirmed, kDxlBaud);
cIMU imu;
craft::ImuFeedback imu_feedback;

void printImu() { imu_feedback.print(imu); }

void setup() {
  imu.begin();  // starts the sensor; its own gyro zeroing (unused here) blocks up to 5 s
  motor_board.setFeedbackExtension(printImu);
  motor_board.begin();
}

void loop() {
  if (imu.update()) imu_feedback.add(imu);  // paced internally at 200 Hz; nonzero on a new sample
  motor_board.tick();
}
