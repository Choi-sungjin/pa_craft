#pragma once

// OpenCR on-board IMU feedback, appended by both boards: ",imu_ok,gx,gy,gz,ax,ay,az".
// Not included by CraftMotorFirmware.h; the host-side firmware test uses fakes/IMU.h.
#include <IMU.h>

namespace craft {

// Boards before 2020 (ours: 2018) carry an MPU-9250, later ones an ICM-20648; the OpenCR IMU library
// sets both to ±2000 dps and ±2 g. Gyro factor as in the official TurtleBot3 firmware.
constexpr double kGyroRadSPerLsb = 0.0010642;   // datasheet 16.4 LSB/(deg/s) at ±2000 dps -> rad/s
constexpr double kAccelGPerLsb = 1.0 / 16384.0;  // 32768 LSB / 2 g

// Raw IMU samples averaged over one feedback period (four at 200 Hz), board frame, as floats.
// Not the library's gyroADC: that one keeps its power-up zero in whole LSBs and zeroes |x| <= 3 LSB,
// so up to ~1 LSB of bias (0.0011 rad/s, ~3.7 deg/min) stays hidden at standstill and leaks in
// while driving. The gyro here still carries the chip's zero-rate offset; the host measures and
// removes it while the robot stands still (craft_perception imu_processor). The accelerometer is
// never zeroed either, so it holds gravity: the host uses it to tell standing from accelerating.
// gyroRAW and accADC share the axes (the library's orientation maps x, y, z to indices 0, 1, 2).
class ImuFeedback {
public:
  // After every imu.update() that returned a new sample.
  void add(const cIMU &imu) {
    for (int i = 0; i < 3; ++i) {
      gyro_sum_[i] += imu.SEN.gyroRAW[i];
      accel_sum_[i] += imu.SEN.accADC[i];
    }
    ++count_;
  }

  // ",imu_ok,gx,gy,gz,ax,ay,az": gyro [rad/s], accel [g]. Repeats the last mean when no new sample
  // arrived; imu_ok=0 and zeros when the sensor did not start.
  void print(const cIMU &imu) {
    if (count_ > 0) {
      for (int i = 0; i < 3; ++i) {
        gyro_[i] = gyro_sum_[i] / count_ * kGyroRadSPerLsb;
        accel_[i] = accel_sum_[i] / count_ * kAccelGPerLsb;
        gyro_sum_[i] = accel_sum_[i] = 0;
      }
      count_ = 0;
    }
    Serial.print(','); Serial.print(imu.bConnected ? 1 : 0);
    for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(gyro_[i], 6); }
    for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(accel_[i], 4); }
  }

private:
  double gyro_sum_[3] = {}, accel_sum_[3] = {};
  double gyro_[3] = {}, accel_[3] = {};
  unsigned count_ = 0;
};

}  // namespace craft
