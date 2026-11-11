#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace lidar_processing {
// A LiDAR in target_frame. flipped: mounted upside down (its z axis points down), so seen from above its
// scan angles turn clockwise: a point at scan angle a lies at yaw - a instead of yaw + a.
struct Pose2D { double x = 0, y = 0, yaw = 0; bool flipped = false; };
// TF translation + rotation (quaternion) -> Pose2D. yaw is the LiDAR x axis direction; the LiDAR z axis
// must be vertical (up or down: mounts are level, upright or upside down).
inline Pose2D pose_from_transform(double x, double y, double qx, double qy, double qz, double qw) {
  return {x, y, std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)), 1 - 2 * (qx * qx + qy * qy) < 0};
}
// Floor ROI for one side (left/right in target_frame) of one LiDAR, angles from its outward axis: up to clear_deg
// every point is kept; from clear_deg to 90 deg in step_deg bins a point is kept only within max_range_m[bin] of
// the LiDAR (beams there dip to the floor further out). Beyond the table up to 90 deg: dropped. Empty: no ROI.
struct FloorRoiSide {
  double clear_deg = 90.0;
  std::vector<double> max_range_m;
};
struct FloorRoi {
  bool enabled = false;
  double step_deg = 5.0;
  FloorRoiSide front_left, front_right, rear_left, rear_right;
};
struct MergerConfig {
  std::string target_frame = "base_link";
  int bins = 720;
  double range_max = 8.0, max_age = 0.2;
  double front_fov_deg = 200.0, rear_fov_deg = 200.0;
  double fov_near_m = 0.30, body_radius_m = 0.25;
  // Body (LiDAR midpoint) x in target_frame [m]: base_link sits on the turning center, the body filter
  // and FOV directions stay centred on the chassis.
  double body_center_x = 0.0;
  FloorRoi floor_roi;
};
// Pure processing module: the caller owns subscriptions, TF and publishing.
class ScanMerger {
public:
  bool configure(const MergerConfig &config);
  static bool is_valid_input(const sensor_msgs::msg::LaserScan &scan);
  std::optional<sensor_msgs::msg::LaserScan> merge(
    const sensor_msgs::msg::LaserScan &front, const Pose2D &front_pose,
    const sensor_msgs::msg::LaserScan *rear = nullptr, const Pose2D *rear_pose = nullptr) const;
private:
  void add_scan(const sensor_msgs::msg::LaserScan &scan, const Pose2D &pose, double fov_deg,
                const FloorRoiSide *left, const FloorRoiSide *right, sensor_msgs::msg::LaserScan &out) const;
  MergerConfig config_;
};
}
