#include "craft_perception/localization_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace craft_perception {
namespace {
double wrap(double a) { return std::remainder(a, 2.0 * M_PI); }

// d^T S^-1 d for a symmetric 3x3 S; a singular S rejects (no information to accept with).
std::optional<double> mahalanobis2(const std::array<double, 3> &d, const Covariance3 &s) {
  const double a = s[0], b = s[1], c = s[2], e = s[4], f = s[5], i = s[8];
  const double det = a * (e * i - f * f) - b * (b * i - f * c) + c * (b * f - e * c);
  if (!std::isfinite(det) || det <= 1e-18) return std::nullopt;
  const Covariance3 inv{(e * i - f * f) / det, (c * f - b * i) / det, (b * f - c * e) / det,
                        (c * f - b * i) / det, (a * i - c * c) / det, (b * c - a * f) / det,
                        (b * f - c * e) / det, (b * c - a * f) / det, (a * e - b * b) / det};
  double m = 0.0;
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) m += d[r] * inv[r * 3 + k] * d[k];
  }
  return m;
}
}  // namespace

const char *to_string(LocalizationState state) {
  switch (state) {
    case LocalizationState::kOk: return "ok";
    case LocalizationState::kDeadReckoning: return "dead_reckoning";
    case LocalizationState::kRelocalize: return "relocalize";
    case LocalizationState::kLost: return "lost";
  }
  return "lost";
}

bool LocalizationMonitor::configure(const LocalizationMonitorConfig &c) {
  for (double v : {c.rejection_sigma, c.dead_reckoning_m, c.bias_time_constant_s, c.min_turn, c.straight_rate,
                   c.turn_rate, c.min_bias_segment_s}) {
    if (!std::isfinite(v) || v <= 0.0) return false;
  }
  for (double v : {c.stable_s, c.bias_limit, c.scale_gain, c.scale_limit, c.settle_s}) {
    if (!std::isfinite(v) || v < 0.0) return false;
  }
  if (c.scale_gain > 1.0 || c.scale_limit >= 1.0 || c.straight_rate >= c.turn_rate || c.misfit_checks < 1) return false;
  config_ = c;
  return true;
}

bool LocalizationMonitor::slam(double t, const Pose2 &slam, const Covariance3 &slam_cov, const Pose2 &ekf,
                               const Covariance3 &ekf_cov) {
  Covariance3 s;
  for (int k = 0; k < 9; ++k) s[k] = slam_cov[k] + ekf_cov[k];
  const auto m2 = mahalanobis2({slam.x - ekf.x, slam.y - ekf.y, wrap(slam.yaw - ekf.yaw)}, s);
  const bool accepted = m2 && *m2 <= config_.rejection_sigma * config_.rejection_sigma &&
                        misfits_ < config_.misfit_checks;
  if (accepted) {
    dead_reckoning_ = 0.0;
    state_ = LocalizationState::kOk;
    if (!stable_since_) stable_since_ = t;
  } else {
    stable_since_.reset();
    if (state_ == LocalizationState::kOk) state_ = LocalizationState::kDeadReckoning;
  }
  return accepted;
}

void LocalizationMonitor::scan_fits(bool fits) {
  misfits_ = fits ? 0 : misfits_ + 1;
  if (misfits_ >= config_.misfit_checks) {
    stable_since_.reset();
    if (state_ == LocalizationState::kOk) state_ = LocalizationState::kDeadReckoning;
  }
}

void LocalizationMonitor::moved(double distance) {
  if (state_ == LocalizationState::kOk || !std::isfinite(distance)) return;
  dead_reckoning_ += std::abs(distance);
  if (state_ == LocalizationState::kDeadReckoning && dead_reckoning_ > config_.dead_reckoning_m) {
    state_ = LocalizationState::kRelocalize;
  }
}

void LocalizationMonitor::relocalized(bool ok) {
  if (state_ != LocalizationState::kRelocalize) return;
  if (ok) {
    state_ = LocalizationState::kDeadReckoning;  // corrected; SLAM still has to agree again
    dead_reckoning_ = 0.0;
    misfits_ = 0;  // the relocalized pose fits the map by construction
  } else {
    state_ = LocalizationState::kLost;
  }
}

bool LocalizationMonitor::learning(double t) const {
  return state_ == LocalizationState::kOk && stable_since_ && t - *stable_since_ >= config_.stable_s;
}

void LocalizationMonitor::yaw(double t, double map_odom_yaw, double odom_yaw, double odom_yaw_rate,
                              std::size_t board) {
  if (last_map_odom_) {
    map_odom_ += wrap(map_odom_yaw - *last_map_odom_);
    odom_ += wrap(odom_yaw - *last_odom_);
  }
  last_t_ = t;
  last_map_odom_ = map_odom_yaw;
  last_odom_ = odom_yaw;
  if (!learning(t) || board > 1) {
    straight_t_.reset();
    turn_odom_.reset();
    return;
  }
  const double rate = std::abs(odom_yaw_rate);
  // Bias: on a straight segment, map->odom yaw changing at r means the IMU misses r rad/s.
  if (rate < config_.straight_rate) {
    if (!straight_t_) {
      straight_t_ = t;
      straight_map_odom_ = map_odom_;
    } else if (t - *straight_t_ >= config_.min_bias_segment_s) {
      const double dt = t - *straight_t_;
      const double drift = (map_odom_ - *straight_map_odom_) / dt;
      bias_[board] = std::clamp(bias_[board] + drift * std::min(1.0, dt / config_.bias_time_constant_s),
                                -config_.bias_limit, config_.bias_limit);
      straight_t_ = t;
      straight_map_odom_ = map_odom_;
    }
  } else {
    straight_t_.reset();
  }
  // Scale: over a whole turn (+ settle_s for SLAM to catch up) the true turn is the odom turn plus
  // what SLAM added on top; their ratio is the remaining scale error.
  if (rate > config_.turn_rate) {
    if (!turn_odom_) {
      turn_odom_ = odom_;
      turn_map_odom_ = map_odom_;
      turn_board_ = board;
    }
    turn_end_.reset();
  } else if (turn_odom_) {
    if (!turn_end_) {
      turn_end_ = t;
    } else if (t - *turn_end_ >= config_.settle_s) {
      const double turned = odom_ - *turn_odom_;
      if (std::abs(turned) >= config_.min_turn && turn_board_ == board) {
        const double ratio = (turned + map_odom_ - *turn_map_odom_) / turned;
        scale_[board] = std::clamp(scale_[board] * (1.0 + config_.scale_gain * (ratio - 1.0)),
                                   1.0 - config_.scale_limit, 1.0 + config_.scale_limit);
      }
      turn_odom_.reset();
    }
  }
}

}  // namespace craft_perception
