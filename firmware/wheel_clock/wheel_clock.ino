// OpenCR #1. USB RX: wheel_l [rad/s],wheel_r [rad/s],clock [unwrapped rad]\n
// USB TX: wheel_l,wheel_r,clock,wheel_state,clock_state,imu_ok,gx,gy,gz,ax,ay,az\n (50 Hz)
// IMU values are in the OpenCR board frame (gyro rad/s, accel m/s^2). Mounting (upside down)
// is handled on the host by the base_link->imu_link TF, not here.
#include <IMU.h>
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<3> motor_board(motor_bus, kAxes, kHardwareConfirmed, kDxlBaud);
cIMU imu;

void printImu() {
  // imu_ok=0 (and zeros) when the sensor did not start; the host then skips /imu/data.
  Serial.print(','); Serial.print(imu.bConnected ? 1 : 0);
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(imu.SEN.gyroADC[i] * kGyroRadSPerLsb, 6); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(imu.SEN.accADC[i] * kAccelMS2PerLsb, 6); }
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
