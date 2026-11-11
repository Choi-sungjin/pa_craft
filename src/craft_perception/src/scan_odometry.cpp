#include "craft_perception/scan_odometry.hpp"

#include <cmath>

namespace craft_perception {

Pose2 compose(const Pose2 &a, const Pose2 &b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, std::remainder(a.yaw + b.yaw, 2.0 * M_PI)};
}

Pose2 between(const Pose2 &a, const Pose2 &b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw), dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, std::remainder(b.yaw - a.yaw, 2.0 * M_PI)};
}

bool ScanOdometry::configure(const ScanOdometryConfig &c) {
  for (double v : {c.grid_resolution, c.grid_half_extent, c.keyframe_m, c.keyframe_rad}) {
    if (!std::isfinite(v) || v <= 0.0) return false;
  }
  if (c.grid_half_extent < 10 * c.grid_resolution || !matcher_.configure(c.matcher)) return false;
  config_ = c;
  has_keyframe_ = false;
  return true;
}

bool ScanOdometry::set_keyframe(const std::vector<std::pair<double, double>> &points) {
  const int n = static_cast<int>(std::round(2.0 * config_.grid_half_extent / config_.grid_resolution));
  std::vector<int8_t> grid(static_cast<std::size_t>(n) * n, 0);
  for (const auto &[x, y] : points) {
    const int u = static_cast<int>(std::floor((x + config_.grid_half_extent) / config_.grid_resolution));
    const int v = static_cast<int>(std::floor((y + config_.grid_half_extent) / config_.grid_resolution));
    if (u >= 0 && v >= 0 && u < n && v < n) grid[static_cast<std::size_t>(v) * n + u] = 100;
  }
  has_keyframe_ = matcher_.set_map(n, n, config_.grid_resolution, -config_.grid_half_extent,
                                   -config_.grid_half_extent, grid);
  pose_ = {0.0, 0.0, 0.0};
  return has_keyframe_;
}

std::optional<ScanOdometry::Step> ScanOdometry::update(const std::vector<std::pair<double, double>> &points,
                                                       const Pose2 &predicted) {
  if (!has_keyframe_) {
    set_keyframe(points);
    return std::nullopt;
  }
  const auto found = matcher_.match(points, compose(pose_, predicted));
  if (!found) {  // lost: the next scan is matched against this one
    set_keyframe(points);
    return std::nullopt;
  }
  const Step step{between(pose_, found->pose), found->score};
  pose_ = found->pose;
  if (std::hypot(pose_.x, pose_.y) > config_.keyframe_m || std::fabs(pose_.yaw) > config_.keyframe_rad) {
    set_keyframe(points);
  }
  return step;
}

}  // namespace craft_perception
