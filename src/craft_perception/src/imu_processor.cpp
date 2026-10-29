#include "craft_perception/imu_processor.hpp"

#include <algorithm>
#include <cmath>

namespace craft_perception {
namespace {
double norm(const Vector3 &v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

bool finite(const Vector3 &v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }

// Calibration: this many outliers in a row (real motion) restart the still window; fewer (a spike) are
// skipped.
constexpr int kCalibrationOutliers = 3;

// A still accelerometer reads gravity alone, 1 g; far from it the robot is accelerating or tilting.
bool gravity_only(const Vector3 &accel) {
  const double g = norm(accel);
  return g > 0.9 && g < 1.1;
}

Vector3 unit(const Vector3 &v) {
  const double n = norm(v);
  return {v[0] / n, v[1] / n, v[2] / n};
}
}  // namespace

bool BoardImu::configure(int calibration_samples, double tracking_gain, double still_rate) {
  if (calibration_samples < 1 || !std::isfinite(tracking_gain) || tracking_gain < 0.0 ||
      tracking_gain >= 1.0 || !std::isfinite(still_rate) || still_rate <= 0.0) {
    return false;
  }
  calibration_samples_ = calibration_samples;
  tracking_gain_ = tracking_gain;
  still_rate_ = still_rate;
  reset();
  return true;
}

void BoardImu::reset() {
  count_ = outliers_ = 0;
  gyro_sum_ = accel_sum_ = up_ = bias_ = gyro_ = Vector3{};
  calibrated_ = have_sample_ = false;
}

void BoardImu::update(const Vector3 &gyro, const Vector3 &accel, bool still) {
  if (!finite(gyro) || !finite(accel)) return;
  gyro_ = gyro;
  have_sample_ = true;
  // The raw gyro carries the chip's zero-rate offset (up to ~0.09 rad/s), so before calibration a
  // sample counts as still by its spread around the window mean, not by its magnitude.
  // Afterwards the bias-corrected magnitude decides.
  Vector3 reference{};
  if (calibrated_) {
    reference = bias_;
  } else if (count_ > 0) {
    for (int i = 0; i < 3; ++i) reference[i] = gyro_sum_[i] / count_;
  } else {
    reference = gyro;
  }
  const Vector3 rate{gyro[0] - reference[0], gyro[1] - reference[1], gyro[2] - reference[2]};
  const bool usable = still && gravity_only(accel) && norm(rate) < still_rate_;
  if (!calibrated_) {
    if (!usable) {
      // With the wheels still, a few outliers in a row are spikes: skip them, keep the window.
      if (still && gravity_only(accel) && ++outliers_ < kCalibrationOutliers) return;
      count_ = outliers_ = 0;  // the wheels moved or the gyro keeps turning: restart the window
      gyro_sum_ = accel_sum_ = Vector3{};
      return;
    }
    outliers_ = 0;
    for (int i = 0; i < 3; ++i) {
      gyro_sum_[i] += gyro[i];
      accel_sum_[i] += accel[i];
    }
    if (++count_ < calibration_samples_) return;
    for (int i = 0; i < 3; ++i) bias_[i] = gyro_sum_[i] / count_;
    up_ = unit(accel_sum_);
    calibrated_ = true;
    return;
  }
  if (usable && tracking_gain_ > 0.0) {
    const Vector3 measured_up = unit(accel);
    Vector3 up;
    for (int i = 0; i < 3; ++i) {
      bias_[i] += tracking_gain_ * (gyro[i] - bias_[i]);
      up[i] = up_[i] + tracking_gain_ * (measured_up[i] - up_[i]);
    }
    up_ = unit(up);
  }
}

std::optional<double> BoardImu::yaw_rate() const {
  if (!calibrated_ || !have_sample_) return std::nullopt;
  double rate = 0.0;
  for (int i = 0; i < 3; ++i) rate += up_[i] * (gyro_[i] - bias_[i]);
  return rate;
}

bool ImuProcessor::configure(const ImuProcessorConfig &config) {
  if (config.primary >= kBoards || config.calibration_samples > 100000 || !std::isfinite(config.variance) || config.variance <= 0.0 ||
      !std::isfinite(config.max_disagreement) || config.max_disagreement < 0.0 ||
      !std::isfinite(config.max_disagreement_ratio) || config.max_disagreement_ratio < 0.0 ||
      !std::isfinite(config.max_age_s) || config.max_age_s <= 0.0 ||
      !std::isfinite(config.max_yaw_rate) || config.max_yaw_rate <= 0.0 ||
      !std::isfinite(config.max_yaw_accel) || config.max_yaw_accel <= 0.0 ||
      !std::isfinite(config.jump_noise) || config.jump_noise < 0.0 || config.disagreement_samples < 1 ||
      config.frozen_samples < 2) {
    return false;
  }
  for (auto &board : boards_) {
    if (!board.configure(config.calibration_samples, config.tracking_gain, config.still_rate)) return false;
  }
  config_ = config;
  accepted_ = accepted_at_ = raw_ = raw_at_ = {};
  dropped_ = {};
  slope_ = {};
  last_raw_ = {};
  repeats_ = changes_ = {};
  frozen_ = {};
  disagreements_ = 0;
  return true;
}

bool ImuProcessor::update(std::size_t board, double t, const Vector3 &gyro, const Vector3 &accel,
                          bool wheels_still) {
  const bool was_calibrated = boards_.at(board).calibrated();
  const bool repeated = last_raw_[board] && *last_raw_[board] == gyro;
  last_raw_[board] = gyro;
  repeats_[board] = repeated ? repeats_[board] + 1 : 0;
  changes_[board] = repeated ? 0 : changes_[board] + 1;
  if (repeats_[board] + 1 >= config_.frozen_samples) frozen_[board] = true;
  if (frozen_[board] && changes_[board] >= config_.frozen_samples) frozen_[board] = false;
  if (frozen_[board]) return false;  // no calibration, tracking or acceptance from a stuck sensor
  boards_[board].update(gyro, accel, wheels_still);
  const auto rate = boards_[board].yaw_rate();
  if (!rate) return false;
  const auto within = [this, t](double from, std::optional<double> at, double rate) {
    const double dt = at ? std::max(0.0, t - *at) : 0.0;
    return std::abs(rate - from) <= config_.max_yaw_accel * dt + config_.jump_noise;
  };
  const bool in_range = std::abs(*rate) <= config_.max_yaw_rate;
  const bool continuous = !accepted_[board] || within(*accepted_[board], accepted_at_[board], *rate);
  const bool confirmed = raw_[board] && within(*raw_[board], raw_at_[board], *rate);
  if (in_range && (continuous || confirmed)) {
    const double dt = accepted_at_[board] ? t - *accepted_at_[board] : 0.0;
    slope_[board] = accepted_[board] && dt > 0.0 ? std::clamp(
      (*rate - *accepted_[board]) / dt, -config_.max_yaw_accel, config_.max_yaw_accel) : 0.0;
    accepted_[board] = rate;
    accepted_at_[board] = t;
  } else {
    ++dropped_[board];
  }
  raw_[board] = rate;
  raw_at_[board] = t;
  return boards_[board].calibrated() && !was_calibrated;
}

void ImuProcessor::set_correction(std::size_t board, double bias, double scale) {
  correction_bias_.at(board) = bias;
  correction_scale_.at(board) = scale;
}

bool ImuProcessor::available(std::size_t board, double t) const {
  return !frozen_.at(board) && accepted_at_.at(board) && t - *accepted_at_[board] <= config_.max_age_s;
}

std::optional<YawRate> ImuProcessor::select(double t, bool wheels_still) {
  const std::size_t backup = 1 - config_.primary;
  if (available(0, t) && available(1, t)) {  // monitor only, compared at the same time t
    const auto at_t = [this, t](std::size_t i) {
      return *accepted_[i] + slope_[i] * std::max(0.0, t - *accepted_at_[i]);
    };
    const double a = at_t(0), b = at_t(1);
    const bool apart = std::abs(a - b) > config_.max_disagreement +
      config_.max_disagreement_ratio * std::abs(a + b) / 2.0;
    disagreements_ = apart ? disagreements_ + 1 : 0;
  } else {
    disagreements_ = 0;
  }
  const std::size_t source = available(config_.primary, t) ? config_.primary : backup;
  if (!available(source, t)) return std::nullopt;
  YawRate rate{correction_scale_[source] * *accepted_[source] + correction_bias_[source], config_.variance, source,
               disagreements_ >= config_.disagreement_samples};
  // Zero-velocity update: with the wheels still, a small rate is residual bias, not rotation, so
  // heading cannot creep while stopped. A larger one (carried, or a fault) passes through.
  if (wheels_still && std::abs(rate.value) < config_.still_rate) rate.value = 0.0;
  return rate;
}

}  // namespace craft_perception
