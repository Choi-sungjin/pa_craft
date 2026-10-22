#pragma once
#include "craft_perception/perception_types.hpp"
#include <optional>

namespace craft_perception {
struct PuckAxisConfig {
  double grow = 2.0;               // ring outer size / box size
  double plane_threshold_m = 0.004;  // RANSAC inlier distance
  double min_height_m = 0.008, max_height_m = 0.08;  // puck points above the shelf plane
  double standing_height_m = 0.045;  // p95 height above this -> standing
  double min_depth_m = 0.1, max_depth_m = 3.0;
  int min_points = 30, ransac_iterations = 100;
};

// Port of puck_axis_live.py (donut method, same idea as KnobDepth):
// ring = grown box - box -> RANSAC shelf plane; box points above it = puck;
// standing -> axis = plane normal, lying -> 2D PCA major axis on the plane.
class PuckAxis {
public:
  bool configure(const PuckAxisConfig &config);
  bool is_valid_input(
    const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &intrinsics) const;
  std::optional<PuckResult> compute_result(
    const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &intrinsics) const;
private:
  PuckAxisConfig config_;
};
}
