#pragma once
#include "craft_perception/perception_types.hpp"
#include <optional>

namespace craft_perception {
class KnobDepth {
public:
  bool configure(double grow, double tolerance_m, int minimum_points);
  bool is_valid_input(
    const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &intrinsics) const;
  std::optional<DepthResult> compute_result(
    const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &intrinsics) const;
private:
  double grow_ = 0, tolerance_m_ = 0;
  int minimum_points_ = 0;
};
}
