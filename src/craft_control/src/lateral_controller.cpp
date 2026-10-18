#include "craft_control/lateral_controller.hpp"

#include <cmath>
#include <limits>

namespace craft_control
{

bool LateralController::set_params(
  double wheel_radius, double wheel_separation, double lookahead, double goal_tolerance)
{
  if (!std::isfinite(wheel_radius) || wheel_radius <= 0.0 ||
    !std::isfinite(wheel_separation) || wheel_separation <= 0.0 ||
    !std::isfinite(lookahead) || lookahead <= 0.0 ||
    !std::isfinite(goal_tolerance) || goal_tolerance <= 0.0)
  {
    return false;
  }
  wheel_radius_ = wheel_radius;
  wheel_separation_ = wheel_separation;
  lookahead_ = lookahead;
  goal_tolerance_ = goal_tolerance;
  configured_ = true;
  return true;
}

bool LateralController::set_odom(const nav_msgs::msg::Odometry & odom)
{
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  have_odom_ = !odom.header.frame_id.empty() && std::isfinite(p.x) && std::isfinite(p.y) &&
    std::isfinite(norm) && norm > 1e-9;
  if (!have_odom_) {
    return false;
  }
  odom_ = odom;
  auto & normalized = odom_.pose.pose.orientation;
  normalized.x /= norm;
  normalized.y /= norm;
  normalized.z /= norm;
  normalized.w /= norm;
  return true;
}

bool LateralController::set_path(const nav_msgs::msg::Path & path)
{
  path_ = nav_msgs::msg::Path{};  // invalid/empty paths cancel the previous path
  if (path.header.frame_id.empty() || path.poses.empty()) {
    return false;
  }
  for (const auto & pose : path.poses) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
    {
      return false;
    }
  }
  path_ = path;
  return true;
}

std::optional<double> LateralController::get_curvature(double target_v) const
{
  if (!configured_ || !have_odom_ || path_.poses.empty() ||
    path_.header.frame_id != odom_.header.frame_id || !std::isfinite(target_v))
  {
    return std::nullopt;
  }
  const auto & p = odom_.pose.pose.position;
  const auto & goal = path_.poses.back().pose.position;
  if (std::hypot(goal.x - p.x, goal.y - p.y) <= goal_tolerance_) {
    return std::nullopt;
  }
  const auto & q = odom_.pose.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path_.poses.size(); ++i) {
    const auto & point = path_.poses[i].pose.position;
    const double distance = std::hypot(point.x - p.x, point.y - p.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  std::optional<double> curvature;
  for (std::size_t i = nearest; i < path_.poses.size(); ++i) {
    const auto & point = path_.poses[i].pose.position;
    const double dx = point.x - p.x;
    const double dy = point.y - p.y;
    const double x = c * dx + s * dy;
    const double y = -s * dx + c * dy;
    // Reverse targets pursue points behind the chassis. A zero target never turns in place.
    if ((target_v < 0.0 ? -x : x) <= 1e-9) {
      continue;
    }
    const double distance = std::hypot(x, y);
    curvature = 2.0 * (y / distance) / distance;
    if (distance >= lookahead_) {
      break;
    }
  }
  return curvature;
}

bool LateralController::can_track(double target_v) const
{
  return get_curvature(target_v).has_value();
}

std::array<double, 2> LateralController::get_wheel_speed(double linear_speed) const
{
  const auto curvature = get_curvature(linear_speed);
  if (!curvature) {
    return {0.0, 0.0};
  }
  const double omega = linear_speed * *curvature;
  const double wheel_delta = omega * wheel_separation_ / (2.0 * wheel_radius_);
  return {-wheel_delta, wheel_delta};
}

}  // namespace craft_control
