// OpenCR #2. All command/feedback positions are motor radians (LM is not metres).
// USB RX: LM,shoulder,elbow,wrist_pitch,tool_roll,gripper\n
// USB TX: same six positions,arm_lm_state,gripper_current_a,imu_ok,gx,gy,gz,ax,ay,az\n (50 Hz)
// The board is fixed to the chassis, so its IMU is a second chassis yaw-rate source (CraftImu.h).
#include <CraftImu.h>
#include "config.h"

Dynamixel2Arduino motor_bus(Serial3, 84);
craft::MotorBoard<6> motor_board(
    motor_bus, kAxes, kHardwareConfirmed, kDxlBaud, kGripperCurrentAPerTick);
cIMU imu;
craft::ImuFeedback imu_feedback;

void printImu() { imu_feedback.print(imu); }

void setup() {
  motor_board.setPositionIGains(kPositionIGains);
  motor_board.setFeedbackExtension(printImu);
  motor_board.begin();  // torque on first so the arm holds during the IMU start-up
  imu.begin();  // starts the sensor; its own gyro zeroing (unused here) blocks up to 5 s, no feedback
}

void loop() {
  if (imu.update()) imu_feedback.add(imu);  // paced internally at 200 Hz; nonzero on a new sample
  motor_board.tick();
}
