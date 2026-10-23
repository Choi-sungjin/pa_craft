// OpenCR #1. USB RX: wheel_l [rad/s],wheel_r [rad/s],clock [unwrapped rad]\n
// USB TX: wheel_l,wheel_r,clock,wheel_state,clock_state,imu_ok,gz\n (50 Hz)
// gz = board-frame yaw rate [rad/s], the only IMU value the EKF uses. Mounting (upside down)
// is handled on the host by the base_link->imu_link TF, not here.
#include <IMU.h>
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<3> motor_board(motor_bus, kAxes, kHardwareConfirmed, kDxlBaud);
cIMU imu;

void printImu() {
  // imu_ok=0 (and gz=0) when the sensor did not start; the host then skips /control/imu/data.
  Serial.print(','); Serial.print(imu.bConnected ? 1 : 0);
  Serial.print(','); Serial.print(imu.SEN.gyroADC[2] * kGyroRadSPerLsb, 6);
}

void setup() {
  imu.begin();  // gyro zero calibration (up to 5 s): keep the robot still at power-up
  motor_board.setFeedbackExtension(printImu);
  motor_board.begin();
}

void loop() {
  imu.update();  // paced internally at 200 Hz
  motor_board.tick();
}
