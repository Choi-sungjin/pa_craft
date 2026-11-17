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
  // Motion compensation (merge() with a Motion): every beam is moved to where it would have been measured from the
  // robot pose of one moment, the reference time = the front scan stamp - pose_time_offset.
  // A sweep takes scan_time (0.1 s) and the rear sweep is up to max_age older or newer than the front one: without
  // it a robot turning at 0.6 rad/s gets a scan bent 3.4 deg within a sweep and up to 6.9 deg between the two.
  // first_beam_latest: the scan stamp is the end of the sweep and index 0 is its last beam (coin_d4 driver with
  // reverse: false; measured 2026-11-12 from tour bags: wall directions of scan sectors against the yaw rate say
  // the beams run counter-clockwise in base_link in time on both upside-down LiDARs = against the scan index).
  // pose_time_offset [s]: the TF pose looked up at the scan stamp is this much older than the stamp
  // (robot_localization transform_time_offset 0.05 in ekf.yaml), so the scan is made for that moment.
  bool first_beam_latest = true;
  double pose_time_offset = 0.0;
};
// Robot motion over the sweeps, in target_frame: forward speed [m/s] and yaw rate [rad/s], taken as constant.
struct Motion { double v = 0, w = 0; };
// Pure processing module: the caller owns subscriptions, TF and publishing.
class ScanMerger {
public:
  bool configure(const MergerConfig &config);
  static bool is_valid_input(const sensor_msgs::msg::LaserScan &scan);
  std::optional<sensor_msgs::msg::LaserScan> merge(
    const sensor_msgs::msg::LaserScan &front, const Pose2D &front_pose,
    const sensor_msgs::msg::LaserScan *rear = nullptr, const Pose2D *rear_pose = nullptr,
    const Motion *motion = nullptr) const;  // motion == nullptr: beams laid down as measured (as before)
private:
  void add_scan(const sensor_msgs::msg::LaserScan &scan, const Pose2D &pose, double fov_deg,
                const FloorRoiSide *left, const FloorRoiSide *right, sensor_msgs::msg::LaserScan &out,
                const Motion *motion, double reference_after_stamp) const;
  MergerConfig config_;
};
}
