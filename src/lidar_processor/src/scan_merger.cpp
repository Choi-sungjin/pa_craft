#include "lidar_processor/scan_merger.hpp"
#include <cmath>
#include <limits>

namespace lidar_processing {
using sensor_msgs::msg::LaserScan;
bool ScanMerger::configure(const MergerConfig &c) {
  if (c.target_frame.empty() || c.bins < 2 || !std::isfinite(c.range_max) || c.range_max <= 0 ||
      !std::isfinite(c.max_age) || c.max_age < 0 ||
      !std::isfinite(c.front_fov_deg) || c.front_fov_deg < 0 || c.front_fov_deg > 360 ||
      !std::isfinite(c.rear_fov_deg) || c.rear_fov_deg < 0 || c.rear_fov_deg > 360 ||
      !std::isfinite(c.fov_near_m) || c.fov_near_m < 0 ||
      !std::isfinite(c.body_radius_m) || c.body_radius_m < 0 || c.body_radius_m >= c.range_max) return false;
  config_ = c;
  return true;
}
bool ScanMerger::is_valid_input(const LaserScan &s) {
  return !s.header.frame_id.empty() && std::isfinite(s.angle_min) &&
    std::isfinite(s.angle_increment) && (s.ranges.empty() || s.angle_increment != 0) &&
    std::isfinite(s.range_min) && std::isfinite(s.range_max) &&
    s.range_min >= 0 && s.range_max >= s.range_min;
}
std::optional<LaserScan> ScanMerger::merge(const LaserScan &front, const Pose2D &fp,
  const LaserScan *rear, const Pose2D *rp) const {
  const auto valid_pose = [](const Pose2D &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
  };
  if (!is_valid_input(front) || !valid_pose(fp)) return std::nullopt;
  LaserScan out;
  out.header.stamp = front.header.stamp;
  out.header.frame_id = config_.target_frame;
  out.angle_min = -M_PI;
  out.angle_increment = 2.0 * M_PI / config_.bins;
  out.angle_max = out.angle_min + out.angle_increment * (config_.bins - 1);
  out.scan_time = front.scan_time;
  out.range_min = 0.0;
  out.range_max = config_.range_max;
  out.ranges.assign(config_.bins, std::numeric_limits<float>::quiet_NaN());
  add_scan(front, fp, config_.front_fov_deg, out);
  if (rear && rp && is_valid_input(*rear) && valid_pose(*rp)) {
    const double age = std::fabs((double(front.header.stamp.sec) - rear->header.stamp.sec) +
      (double(front.header.stamp.nanosec) - rear->header.stamp.nanosec) * 1e-9);
    if (age <= config_.max_age) add_scan(*rear, *rp, config_.rear_fov_deg, out);
  }
  return out;
}
// 한 LiDAR 의 점을 FOV 로 거른 뒤 base_link 기준 bin 에 넣음
void ScanMerger::add_scan(const LaserScan & s, const Pose2D & p, double fov_deg, LaserScan & out) const
{
  const double half_fov = fov_deg * M_PI / 360.0;
  // FOV 중심 = 로봇 중앙 → LiDAR 방향(바깥쪽)을 LiDAR 자기 좌표 각도로 바꾼 값
  const double fov_center = std::atan2(p.y, p.x) - p.yaw;
  const double range_max = out.range_max;
  const double fov_near = config_.fov_near_m;
  const double body_r = config_.body_radius_m;
  const double c = std::cos(p.yaw), sn = std::sin(p.yaw);
  for (size_t i = 0; i < s.ranges.size(); ++i) {
    const float r = s.ranges[i];
    if (!std::isfinite(r) || r < s.range_min || r > s.range_max) {continue;}
    const double a = s.angle_min + i * s.angle_increment;           // LiDAR 기준 각도
    const double lx = r * std::cos(a), ly = r * std::sin(a);
    const double x = p.x + c * lx - sn * ly;                           // base_link 좌표
    const double y = p.y + sn * lx + c * ly;
    const double d = std::hypot(x, y);                                 // 로봇 중앙에서 거리
    if (d > range_max || d < body_r) {continue;}                      // 너무 멀거나 몸체(기둥) 안쪽
    // FOV 밖이면서 로봇 가까이(기둥·몸체 거리)인 점만 버림 → 옆쪽 먼 장애물은 남김
    const bool out_of_fov = std::fabs(std::remainder(a - fov_center, 2.0 * M_PI)) > half_fov;
    if (out_of_fov && d < fov_near) {continue;}
    int k = static_cast<int>(std::floor((std::atan2(y, x) - out.angle_min) / out.angle_increment));
    if (k < 0) {k = 0;}
    if (k >= config_.bins) {k = config_.bins - 1;}
    if (!(out.ranges[k] <= d)) {out.ranges[k] = static_cast<float>(d);}  // 비었거나 더 멀면 교체
  }
}

}
