// Upper-cam box geometry, C++ port of demo/corner/uppercam_square_far20_estimator.py (square_far20) and its
// helpers in uppercam_fast5_box_range.py / uppercam_box_range.py, plus the estimator's filter and the LiDAR face
// fit (uppercam_lidar_geometry.py). Same steps and constants; the RANSAC draws come from std::mt19937 instead of
// numpy, so single frames can differ where the Python result itself depends on the draw (2026-11-12 comparison on
// boxgap_1108_1236 with uppercam_geometry_check: same detections, holdout MAE 1.41 vs 1.43 mm).
#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace uppercam
{

constexpr double kSide = 0.205;        // box top square [m]
constexpr double kBoxHeight = 0.070;   // box top over the floor [m]
constexpr double kCamHeight = 0.6229;  // colour lens over the floor [m] (user's triangulation 2026-11-07)
constexpr double kLaserX = 0.19067;    // URDF front_laser x, the calibration's reference
constexpr const char * kMethod = "square_far20_spec_reimplementation_v1";

// base_link <- camera optical frame as the estimator uses it: URDF joint, turned 1.5 deg CCW, y 32.2 mm,
// height floor + kCamHeight. up = base z in the camera frame.
struct CameraMount
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  Eigen::Vector3d up;
};
CameraMount load_camera_mount(const std::string & urdf_path);

struct Geometry
{
  double center_x{}, center_y{}, box_yaw_rad{}, near_x{}, lateral_m{};
  int far_inliers{}, side_inliers{};
  double fit_cost{}, raw_gap_mm{}, plane_rmse_mm{}, top_area_m2{};
  std::vector<std::string> flags;
};

// One aligned depth frame [m] (CV_64F or CV_32F, 0 = no depth) and its 3x3 intrinsics -> geometry.
// Throws std::runtime_error with the Python messages (e.g. "no level blob of box-top size at box-top height").
Geometry square_far20_geometry(const cv::Mat & depth_m, const Eigen::Matrix3d & K, const CameraMount & mount);

// fit_square(): robust fit of the 205 mm square's far edge and sides to boundary points in base_link.
Geometry fit_square(const std::vector<Eigen::Vector2d> & points, const Eigen::Vector3d & initial);

// ---- estimator state (estimate_from_geometry, SelectedPostprocessor, LatestGate, gap calibration) ------------
struct Calibration
{
  std::vector<double> x, y;  // input_knots_mm -> gap_knots_mm, both strictly increasing
  double apply(double raw, bool & extrapolated) const;
};
Calibration load_calibration(const std::string & json_path);  // uppercam_selected_calibration.json

class LatestGate  // uppercam_trigger_filter.py
{
public:
  struct Item { double value; long index; double time; bool extrapolated; };
  struct Out { bool valid{false}; std::string state{"invalid"}; double value{}; long index{}; bool extrapolated{}; };
  Out update(double value, long index, double t, bool extrapolated);
  void reset() { anchor_.reset(); pending_.reset(); history_.clear(); }
  const std::deque<Item> & history() const { return history_; }
private:
  bool close(const Item & a, const Item & b) const;
  double jump_mm_{8.0}, max_rate_mm_s_{120.0}, max_gap_s_{0.3};
  std::optional<Item> anchor_, pending_;
  std::deque<Item> history_;
};

struct Estimate  // the fields of the Python result dict the bridge reads
{
  bool valid{false};             // always false here: square_observation_policy decides
  std::string state{"invalid"};  // invalid | warmup | outlier | geometry_unverified
  std::string error;             // geometry failure message, empty if none
  std::optional<double> gap_mm, raw_gap_mm, lateral_m, box_yaw_rad, source_stamp_s;
  std::optional<long> source_frame_index;
  double capture_stamp_s{}, processing_ms{};
  long frame_index{};
  bool extrapolated{false}, current_extrapolated{false};
  std::optional<Geometry> geometry;
};

class SquareFar20Estimator
{
public:
  explicit SquareFar20Estimator(Calibration calibration) : calibration_(std::move(calibration)) {}
  // Capture order; geometry is nullopt with error set when square_far20_geometry threw.
  Estimate estimate(const std::optional<Geometry> & geo, const std::string & error, long frame_index, double stamp_s);
  void reset() { gate_.reset(); }
private:
  Calibration calibration_;
  LatestGate gate_;
};

// ---- bridge contract (uppercam_bridge_utils.py) ----------------------------------------------------------------
struct Policy { bool valid; std::string reason; double uncertainty_m; };
Policy square_observation_policy(const Estimate & e, bool enabled);
bool acquisition_valid(double stamp_s, double now_s, double request_s, double timeout);
// (x, y, heading) of the box centre in the frame of robot_pose from the calibrated camera measurement.
Eigen::Vector3d camera_box_in_map(double gap_m, double lateral_m, double yaw_relative, const Eigen::Vector3d & robot_pose,
                                  double front_face_x, double side);

// ---- LiDAR face (uppercam_lidar_geometry.py) ---------------------------------------------------------------------
struct FaceFit
{
  Eigen::Vector2d box_center_map, face_normal_map;
  double box_yaw, gap_m, lateral_m, yaw_relative, rmse_m;
  int inliers;
};
std::optional<FaceFit> fit_box_face(const std::vector<Eigen::Vector2d> & points_map, const Eigen::Vector3d & expected_box,
                                    const Eigen::Vector3d & robot_pose, double side, double front_face_x);

struct FaceEnds
{
  bool ok{false};
  std::string reason;
  Eigen::Vector2d face_centre_map{}, box_center_map{};
  double width_m{};
};
FaceEnds observe_face_endpoints(const std::vector<double> & ranges, const std::vector<double> & angles,
                                const Eigen::Matrix3d & rotation, const Eigen::Vector3d & origin, const FaceFit & fit,
                                double side, double sensor_min, double sensor_max);

// ---- Box finder (2026-11-13) --------------------------------------------------------------------------------------
// The map has no start boxes, so a start box shows up as LiDAR points off the map (/perception/lidar/obstacle).
// Among those within radius of where planning expects the box: the cluster (points within cluster_gap of each other)
// nearest that place whose extent fits one box, fitted with the side x side square: yaw from the bounding rectangle
// that fits its points best (1 deg steps over a quarter turn, the one nearest the expected yaw mod 90 deg), centre
// from the faces seen (side / 2 behind the face facing the robot; the middle where both faces of an axis are seen).
struct FoundBox { double x, y, yaw; int points; double extent_u, extent_v; };
std::optional<FoundBox> find_box(const std::vector<Eigen::Vector2d> & points_map, const Eigen::Vector3d & expected,
                                 const Eigen::Vector2d & robot, double side, double radius = 0.6,
                                 double cluster_gap = 0.05);

}  // namespace uppercam
