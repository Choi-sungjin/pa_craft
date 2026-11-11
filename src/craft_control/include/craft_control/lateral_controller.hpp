#pragma once

#include <array>
#include <limits>
#include <optional>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace craft_control
{

class LateralController
{
public:
  // All values are validated positive by ControlMaster except skid_steer_factor; heading tolerance is
  // in radians. wheel_track: left/right wheel center distance, wheelbase: front/rear axle distance [m].
  // skid_steer_factor <= 0 uses the geometric estimate 1 + (wheelbase / wheel_track)^2.
  // tracking_gain scales the Pure Pursuit turn command only (under steer compensation; not the in-place turn).
  void configure(
    double wheel_radius, double wheel_track, double wheelbase, double skid_steer_factor,
    double lookahead, double goal_tolerance, double path_spacing,
    double heading_tolerance = 0.5, double rotate_angular_speed = 0.6, double tracking_gain = 1.0);
  // 4WD skid steer: chassis yaw rate = wheel_radius * (wr - wl) / effective_separation().
  double effective_separation() const {return wheel_separation_;}
  // Nonempty frame_id, finite position, nonzero quaternion.
  bool is_valid_odom(const nav_msgs::msg::Odometry & odom) const;
  // Nonempty path in one nonempty frame with finite positions.
  bool is_valid_target(const nav_msgs::msg::Path & path) const;
  // Store odometry/path after validation succeeds; no validation here. Both update "my node", the path
  // sample the robot is at: nearest within +-kNodeWindow samples of the previous one, so a path that
  // overlaps itself (U-turn) never switches branches; re-searched globally only when that is lookahead away.
  void set_odom(const nav_msgs::msg::Odometry & odom);
  void clear_odom();
  // Subdivides planner segments to at most path_spacing without changing path geometry.
  void set_target(const nav_msgs::msg::Path & path, bool require_goal_heading = false);
  void clear_target();
  // Resampled path being tracked (empty when there is no target).
  const nav_msgs::msg::Path & target_path() const;
  // False while turning in place: the lookahead target is more than heading_tolerance off (hysteresis:
  // turning stops below half of it), or a required goal heading is still off at the goal.
  bool can_track(double target_v) const;
  // Same-frame endpoint arrival, including heading tolerance when requested.
  bool is_target_reached() const;
  // Pure Pursuit toward the sample lookahead along the path from my node: omega = tracking_gain * v * curvature.
  // Only the rotational wheel-speed contribution; with a finite hint it turns in place when needed.
  std::array<double, 2> compute_command(
    double linear_speed, double target_v_hint = std::numeric_limits<double>::quiet_NaN()) const;

  static constexpr std::size_t kNodeWindow = 3;

private:
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path path_;
  std::vector<double> arc_;  // cumulative length along path_ [m]
  std::size_t node_ = 0;
  mutable bool rotating_ = false;  // hysteresis state of the in-place turn, per travel direction
  mutable int rotating_direction_ = 0;
  void update_node();
  std::size_t target_index() const;
  bool needs_rotation(double target_v) const;
  bool at_goal() const;
  std::optional<double> get_curvature(double target_v) const;
  std::optional<double> heading_error(double target_v) const;
  std::array<double, 2> wheel_delta(double omega) const;
  bool require_goal_heading_ = false;
  double heading_tolerance_ = 0.5;
  double rotate_angular_speed_ = 0.6;
  double tracking_gain_ = 1.0;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;  // effective (skid steer) separation
  double lookahead_ = 0.0;
  double goal_tolerance_ = 0.0;
  double path_spacing_ = 0.0;
  bool have_odom_ = false;
};

}  // namespace craft_control
