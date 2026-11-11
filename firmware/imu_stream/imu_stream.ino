// OpenCR on-board IMU stream for bench checks; replaces wheel_clock on the wheel board temporarily.
// No DYNAMIXEL bus: wheels and clock stay unpowered. Back to driving: resetwheel.
// USB TX (100 Hz): t_ms,imu_ok,
//   acc_raw x3,gyro_raw x3,mag_raw x3   sensor LSB, no calibration/filter
//   acc_g x3                            acc ADC (calibrated) x 8/32768
//   gyro_dps x3                         gyro ADC (zeroed, 4-tap mean, |LSB|<=3 -> 0 in the library) x 2000/32768
//   roll,pitch,yaw                      Madgwick [deg], yaw drifts (no magnetometer)
#include <IMU.h>

constexpr uint32_t kPeriodMs = 10;

cIMU imu;
uint32_t last_ms = 0;

void printTriple(const int16_t *v) {
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(v[i]); }
}

void printFloat(float v, int digits) {
  Serial.print(','); Serial.print(v, digits);
}

void setup() {
  Serial.begin(115200);
  imu.begin();  // gyro zero calibration (up to 5 s): keep the robot still at power-up
}

void loop() {
  imu.update();  // paced internally at 200 Hz
  const uint32_t now = millis();
  if (now - last_ms < kPeriodMs) return;
  last_ms = now;
  Serial.print(now); Serial.print(','); Serial.print(imu.bConnected ? 1 : 0);
  printTriple(imu.accRaw); printTriple(imu.gyroRaw); printTriple(imu.magRaw);
  printFloat(imu.ax, 4); printFloat(imu.ay, 4); printFloat(imu.az, 4);
  printFloat(imu.gx, 3); printFloat(imu.gy, 3); printFloat(imu.gz, 3);
  printFloat(imu.rpy[0], 2); printFloat(imu.rpy[1], 2); printFloat(imu.rpy[2], 2);
  Serial.print('\n');
}
