#include "craft_perception/state_estimator.hpp"
#include <cmath>

namespace craft_perception {
bool StateEstimator::is_valid_input(const nav_msgs::msg::Odometry &input) const {
  const auto &p = input.pose.pose.position;
  const auto &q = input.pose.pose.orientation;
  const auto &v = input.twist.twist.linear;
  const auto &w = input.twist.twist.angular;
  const double values[] = {p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z};
  for (const double value : values) if (!std::isfinite(value)) return false;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (input.header.frame_id.empty() || std::abs(norm - 1.0) > 0.01) return false;
  return true;
}

nav_msgs::msg::Odometry StateEstimator::compute_result(const nav_msgs::msg::Odometry &input) const {
  return input;
}
}
