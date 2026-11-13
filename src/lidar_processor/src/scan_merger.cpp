#include "lidar_processor/scan_merger.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_processing {
using sensor_msgs::msg::LaserScan;
bool ScanMerger::configure(const MergerConfig &c) {
  if (c.target_frame.empty() || c.bins < 2 || !std::isfinite(c.range_max) || c.range_max <= 0 ||
      !std::isfinite(c.max_age) || c.max_age < 0 ||
      !std::isfinite(c.front_fov_deg) || c.front_fov_deg < 0 || c.front_fov_deg > 360 ||
      !std::isfinite(c.rear_fov_deg) || c.rear_fov_deg < 0 || c.rear_fov_deg > 360 ||
      !std::isfinite(c.fov_near_m) || c.fov_near_m < 0 || !std::isfinite(c.body_center_x) ||
      !std::isfinite(c.body_radius_m) || c.body_radius_m < 0 || c.body_radius_m >= c.range_max ||
      !std::isfinite(c.pose_time_offset) || std::fabs(c.pose_time_offset) > 1.0) return false;
  if (c.floor_roi.enabled) {
    if (!std::isfinite(c.floor_roi.step_deg) || c.floor_roi.step_deg <= 0) return false;
    for (const auto *s : {&c.floor_roi.front_left, &c.floor_roi.front_right, &c.floor_roi.rear_left, &c.floor_roi.rear_right}) {
      if (!std::isfinite(s->clear_deg) || s->clear_deg < 0 || s->clear_deg > 90) return false;
      for (const double r : s->max_range_m) {if (!std::isfinite(r) || r < 0) return false;}
    }
  }
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
  const LaserScan *rear, const Pose2D *rp, const Motion *motion) const {
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
  const auto &roi = config_.floor_roi;
  if (motion && (!std::isfinite(motion->v) || !std::isfinite(motion->w))) motion = nullptr;
  add_scan(front, fp, config_.front_fov_deg, roi.enabled ? &roi.front_left : nullptr,
           roi.enabled ? &roi.front_right : nullptr, out, motion, -config_.pose_time_offset);
  if (rear && rp && is_valid_input(*rear) && valid_pose(*rp)) {
    const double front_after_rear = (double(front.header.stamp.sec) - rear->header.stamp.sec) +
      (double(front.header.stamp.nanosec) - rear->header.stamp.nanosec) * 1e-9;
    const double age = std::fabs(front_after_rear);
    if (age <= config_.max_age) {
      add_scan(*rear, *rp, config_.rear_fov_deg, roi.enabled ? &roi.rear_left : nullptr,
               roi.enabled ? &roi.rear_right : nullptr, out, motion, front_after_rear - config_.pose_time_offset);
    }
  }
  return out;
}
// 한 LiDAR 의 점을 FOV 로 거른 뒤 base_link 기준 bin 에 넣음
void ScanMerger::add_scan(const LaserScan & s, const Pose2D & p, double fov_deg,
  const FloorRoiSide * left, const FloorRoiSide * right, LaserScan & out,
  const Motion * motion, double reference_after_stamp) const
{
  // Time between two beams: the driver's, else the sweep time over the beams, else 0 (all at the stamp).
  const double beam_dt = s.time_increment > 0 ? s.time_increment :
    (s.scan_time > 0 && s.ranges.size() > 1 ? s.scan_time / (s.ranges.size() - 1) : 0.0);
  const double half_fov = fov_deg * M_PI / 360.0;
  // FOV 중심 = 몸체 중앙(LiDAR 가운데) → LiDAR 방향(바깥쪽)을 LiDAR 자기 좌표 각도로 바꾼 값
  const double bx = config_.body_center_x;
  const double fov_center = (std::atan2(p.y, p.x - bx) - p.yaw) * (p.flipped ? -1.0 : 1.0);
  const double range_max = out.range_max;
  const double fov_near = config_.fov_near_m;
  const double body_r = config_.body_radius_m;
  const double c = std::cos(p.yaw), sn = std::sin(p.yaw);
  const double flip = p.flipped ? -1.0 : 1.0;  // upside down: scan angles turn the other way
  for (size_t i = 0; i < s.ranges.size(); ++i) {
    const float r = s.ranges[i];
    if (!std::isfinite(r) || r < s.range_min || r > s.range_max) {continue;}
    const double a = s.angle_min + i * s.angle_increment;           // LiDAR 기준 각도
    const double lx = r * std::cos(a), ly = flip * r * std::sin(a);
    const double x = p.x + c * lx - sn * ly;                           // base_link 좌표
    const double y = p.y + sn * lx + c * ly;
    const double d = std::hypot(x, y);                                 // base_link(회전 중심)에서 거리
    const double body_d = std::hypot(x - bx, y);                       // 몸체 중앙에서 거리
    if (d > range_max || body_d < body_r) {continue;}                 // 너무 멀거나 몸체(기둥) 안쪽
    // FOV 밖이면서 몸체 가까이(기둥·몸체 거리)인 점만 버림 → 옆쪽 먼 장애물은 남김
    const bool out_of_fov = std::fabs(std::remainder(a - fov_center, 2.0 * M_PI)) > half_fov;
    if (out_of_fov && body_d < fov_near) {continue;}
    // Floor ROI: angle of the point from this LiDAR's outward axis, in target_frame, per side.
    if (left && right) {
      const double outward = std::atan2(p.y, p.x - bx);
      const double dev = std::remainder(std::atan2(y - p.y, x - p.x) - outward, 2.0 * M_PI);
      const FloorRoiSide & side = (y - p.y) >= 0 ? *left : *right;
      const double deg = std::fabs(dev) * 180.0 / M_PI;
      if (deg > side.clear_deg && deg <= 90.0) {
        const auto bin = static_cast<size_t>((deg - side.clear_deg) / config_.floor_roi.step_deg);
        if (bin >= side.max_range_m.size() || r > side.max_range_m[bin]) {continue;}
      }
    }
    // Where the point lies seen from the robot at the reference time. The beam was measured before_stamp before
    // the scan stamp; from then to the reference time (dt later) the robot went v * dt forward and turned w * dt,
    // so the point is that much further back and turned the other way. The filters above use it as measured.
    double mx = x, my = y, md = d;
    if (motion) {
      const double before_stamp = beam_dt * (config_.first_beam_latest ? i : s.ranges.size() - 1 - i);
      const double dt = std::clamp(reference_after_stamp + before_stamp, -0.5, 0.5);
      const double th = -motion->w * dt, ct = std::cos(th), st = std::sin(th);
      mx = ct * x - st * y - motion->v * dt;
      my = st * x + ct * y;
      md = std::hypot(mx, my);
      if (md > range_max) {continue;}
    }
    int k = static_cast<int>(std::floor((std::atan2(my, mx) - out.angle_min) / out.angle_increment));
    if (k < 0) {k = 0;}
    if (k >= config_.bins) {k = config_.bins - 1;}
    if (!(out.ranges[k] <= md)) {out.ranges[k] = static_cast<float>(md);}  // 비었거나 더 멀면 교체
  }
}

}
