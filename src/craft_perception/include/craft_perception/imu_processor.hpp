#pragma once

#include <array>
#include <cstddef>
#include <optional>

namespace craft_perception {

using Vector3 = std::array<double, 3>;

// One OpenCR on-board IMU, any mounting. While the robot stands still it takes the board's up axis
// from gravity (accelerometer) and the gyro bias, then reports the chassis yaw rate
// up . (gyro - bias): a tilted board needs no per-board setting, and the horizontal heading of the
// board does not matter for a yaw rate.
class BoardImu {
public:
  // calibration_samples: consecutive still samples before the first estimate.
  // tracking_gain: per still sample, how fast bias and up follow afterwards (0 freezes them).
  // still_rate: [rad/s] a sample further than this from the bias (before calibration: from the
  // window mean, since the raw gyro still carries the chip offset) is never taken as still.
  bool configure(int calibration_samples, double tracking_gain, double still_rate);
  // gyro [rad/s] raw (chip offset included), accel [g], both board frame; still = wheels stopped.
  void update(const Vector3 &gyro, const Vector3 &accel, bool still);
  // Clears the calibration (sensor lost); the next still window calibrates again.
  void reset();
  bool calibrated() const { return calibrated_; }
  const Vector3 &up() const { return up_; }
  const Vector3 &bias() const { return bias_; }
  // Chassis yaw rate [rad/s] from the newest sample, counter-clockwise positive; empty until calibrated.
  std::optional<double> yaw_rate() const;

private:
  int calibration_samples_ = 0;
  double tracking_gain_ = 0.0, still_rate_ = 0.0;
  int count_ = 0, outliers_ = 0;
  Vector3 gyro_sum_{}, accel_sum_{};
  Vector3 up_{}, bias_{}, gyro_{};
  bool calibrated_ = false, have_sample_ = false;
};

struct YawRate {
  double value;       // [rad/s]
  double variance;    // [(rad/s)^2]
  std::size_t board;  // the board it came from
  bool disagree;      // the other board has differed for disagreement_samples outputs in a row
};

struct ImuProcessorConfig {
  std::size_t primary;            // board used whenever it is available; the other is the backup
  int calibration_samples;        // unbroken still samples before a board reports
  double tracking_gain;           // per still sample afterwards; 0 freezes bias and up
  double still_rate;              // [rad/s]
  double variance;                // [(rad/s)^2] of one board
  double max_disagreement;        // [rad/s] boards further apart than this
  double max_disagreement_ratio;  //   + this share of |yaw rate| disagree (monitor only)
  double max_age_s;               // a board with no accepted sample this recent is unavailable
  double max_yaw_rate;            // [rad/s] above it a sample is impossible for the chassis: dropped
  double max_yaw_accel;           // [rad/s^2] with jump_noise: largest change between two samples
  double jump_noise;              // [rad/s]
  int disagreement_samples;       // outputs in a row before a disagreement is reported
  int frozen_samples;             // identical raw gyro samples in a row: board frozen (and as many
                                  // changing ones to come back)
};

// Both chassis-fixed OpenCR IMUs (0 = wheel_clock, 1 = lm_arm) -> one chassis yaw rate.
// The primary board is used whenever it is available (calibrated, fresh accepted sample); the backup
// only while it is not. The two are never mixed: with two sensors a disagreement shows that one is
// wrong but not which, so it is only reported. For that check each board's rate is carried to the
// same time along its own last slope (they sample up to 20 ms apart).
// Per board, a sample is dropped when its yaw rate is beyond max_yaw_rate, or when it jumps from the
// last accepted one by more than max_yaw_accel * dt + jump_noise unless the previous raw sample
// already showed the same level (a real fast change is then accepted one sample late, a one-sample
// spike never). With the wheels still, a rate below still_rate is reported as 0 (zero-velocity update).
// A board whose raw gyro repeats exactly for frozen_samples samples is frozen (stuck chip or IMU library:
// the firmware sends a noisy 4-sample float mean, which never repeats on a live sensor): it is
// unavailable, so the backup takes over, and it does not feed its own bias tracking.
class ImuProcessor {
public:
  static constexpr std::size_t kBoards = 2;
  bool configure(const ImuProcessorConfig &config);
  // One raw sample of a board at time t [s]; returns true when this sample completed its calibration.
  bool update(std::size_t board, double t, const Vector3 &gyro, const Vector3 &accel, bool wheels_still);
  // Yaw rate at time t from the primary, or the backup while the primary is unavailable; empty when
  // neither is. One call per output: it counts how long the boards have disagreed.
  std::optional<YawRate> select(double t, bool wheels_still);
  bool available(std::size_t board, double t) const;
  // Learned on top of the board's own calibration (LocalizationMonitor): rate = scale * rate + bias.
  void set_correction(std::size_t board, double bias, double scale);
  std::size_t primary() const { return config_.primary; }
  const BoardImu &board(std::size_t board) const { return boards_.at(board); }
  // Newest accepted yaw rate of a board (after the range and jump checks).
  std::optional<double> accepted(std::size_t board) const { return accepted_.at(board); }
  // Samples dropped by the range or jump check, per board.
  std::size_t dropped(std::size_t board) const { return dropped_.at(board); }
  bool frozen(std::size_t board) const { return frozen_.at(board); }

private:
  ImuProcessorConfig config_{};
  std::array<BoardImu, kBoards> boards_;
  std::array<std::optional<double>, kBoards> accepted_, accepted_at_, raw_, raw_at_;
  std::array<double, kBoards> slope_{};  // [rad/s^2] between the last two accepted samples
  std::array<double, kBoards> correction_bias_{}, correction_scale_{1.0, 1.0};
  std::array<std::size_t, kBoards> dropped_{};
  std::array<std::optional<Vector3>, kBoards> last_raw_;
  std::array<int, kBoards> repeats_{}, changes_{};
  std::array<bool, kBoards> frozen_{};
  int disagreements_ = 0;
};

}  // namespace craft_perception
