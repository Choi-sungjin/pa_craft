#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace craft_perception {

struct Pose2 {
  double x, y, yaw;
};
using Covariance3 = std::array<double, 9>;  // row-major x, y, yaw

struct LocalizationMonitorConfig {
  double rejection_sigma;       // SLAM pose further than this (Mahalanobis) from ekf_global: rejected
  double dead_reckoning_m;      // travel on IMU + wheels alone before a wide relocalization
  double stable_s;              // SLAM accepted this long in a row before the IMU corrections learn
  double bias_time_constant_s;  // the bias correction follows the measured drift this slowly
  double bias_limit;            // [rad/s] |bias correction| cap
  double scale_gain;            // share of one turn's scale error taken
  double scale_limit;           // |scale correction - 1| cap
  double min_turn;              // [rad] shorter turns do not update the scale
  double settle_s;              // after a turn, wait this long for SLAM yaw before measuring it
  double straight_rate;         // [rad/s] below it the robot drives straight (bias segments)
  double turn_rate;             // [rad/s] above it the robot turns (scale segments)
  double min_bias_segment_s;    // straight time before a drift rate is measured
  int misfit_checks;            // scan-to-map checks in a row below the score before dead reckoning
};

enum class LocalizationState { kOk, kDeadReckoning, kRelocalize, kLost };
const char *to_string(LocalizationState state);

// Watches the EKF localization (localization:=ekf) from outside robot_localization:
// - Each SLAM pose is checked against ekf_global the way its pose0_rejection_threshold does
//   (Mahalanobis, SLAM + EKF covariance). Rejected poses mean the robot runs on IMU + wheels alone
//   (dead reckoning); past dead_reckoning_m of travel a wide relocalization is requested, and only
//   its failure is final (kLost).
// - SLAM and the EKF can also be wrong together (slam_toolbox only searches near the odometry
//   prediction, so after a push both stay off): the scan is scored against SLAM's map at the EKF pose
//   now and then, and misfit_checks low scores in a row count as rejection too.
// - While SLAM has been accepted for stable_s, the yaw SLAM adds on top of the IMU (map->odom yaw)
//   is the IMU's remaining error: its rate on straight segments corrects the bias, its share of a
//   turn corrects the scale. Both are capped and learned per board, for the board in use.
class LocalizationMonitor {
public:
  bool configure(const LocalizationMonitorConfig &config);
  // One SLAM pose and ekf_global's pose at the same time; returns whether it is accepted.
  bool slam(double t, const Pose2 &slam, const Covariance3 &slam_cov, const Pose2 &ekf, const Covariance3 &ekf_cov);
  // One scan-to-map check at the EKF pose (ScanRelocalizer::score >= min_score).
  void scan_fits(bool fits);
  // Travel [m] since the previous call (wheel odometry).
  void moved(double distance);
  // Result of a requested relocalization: success restarts the dead-reckoning budget, failure is kLost.
  void relocalized(bool ok);
  // One ekf_global/ekf_local pair: map->odom yaw and odom yaw [rad], odom yaw rate, IMU board in use.
  void yaw(double t, double map_odom_yaw, double odom_yaw, double odom_yaw_rate, std::size_t board);
  LocalizationState state() const { return state_; }
  double dead_reckoning() const { return dead_reckoning_; }
  // IMU corrections for a board: rate_out = scale * rate + bias.
  double bias(std::size_t board) const { return bias_.at(board); }
  double scale(std::size_t board) const { return scale_.at(board); }

private:
  bool learning(double t) const;
  LocalizationMonitorConfig config_{};
  LocalizationState state_ = LocalizationState::kOk;
  double dead_reckoning_ = 0.0;
  std::optional<double> stable_since_;
  int misfits_ = 0;  // scan-to-map checks in a row below the score
  std::array<double, 2> bias_{0.0, 0.0}, scale_{1.0, 1.0};
  // Unwrapped yaw tracks and the segments measured on them.
  std::optional<double> last_t_, last_map_odom_, last_odom_;
  double map_odom_ = 0.0, odom_ = 0.0;
  std::optional<double> straight_t_, straight_map_odom_;
  std::optional<double> turn_odom_, turn_map_odom_, turn_end_;
  std::size_t turn_board_ = 0;
};

}  // namespace craft_perception
