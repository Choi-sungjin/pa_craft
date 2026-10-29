#pragma once
#include <stdint.h>

// Host-side stand-in for the OpenCR IMU library: only the fields CraftImu.h reads.
struct cIMUDevice {
  bool bConnected = true;
  int16_t gyroRAW[3] = {};
  int16_t accADC[3] = {};
};
struct cIMU {
  bool bConnected = true;
  cIMUDevice SEN;
};
