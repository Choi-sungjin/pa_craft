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

nav_msgs::msg::Odometry StateEstimator::compute_result(const nav_msgs::msg::Odometry &input,
    const geometry_msgs::msg::Transform &map_from_odom, const std::string &map_frame) const {
  const auto &a = map_from_odom.rotation;
  const auto &t = map_from_odom.translation;
  const auto &p = input.pose.pose.position;
  const auto &b = input.pose.pose.orientation;
  // Rotate p by a: v' = v + 2w(u x v) + 2u x (u x v), u = (a.x, a.y, a.z).
  const double cx = a.y * p.z - a.z * p.y, cy = a.z * p.x - a.x * p.z, cz = a.x * p.y - a.y * p.x;
  auto output = input;
  output.header.frame_id = map_frame;
  output.pose.pose.position.x = t.x + p.x + 2 * (a.w * cx + a.y * cz - a.z * cy);
  output.pose.pose.position.y = t.y + p.y + 2 * (a.w * cy + a.z * cx - a.x * cz);
  output.pose.pose.position.z = t.z + p.z + 2 * (a.w * cz + a.x * cy - a.y * cx);
  auto &q = output.pose.pose.orientation;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return output;
}
}
