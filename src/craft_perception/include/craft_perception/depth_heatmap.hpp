#pragma once
#include "craft_perception/perception_types.hpp"

namespace craft_perception {
class DepthHeatmap {
public:
  bool configure(double near_m, double far_m);
  bool is_valid_input(const DepthImage &depth) const;
  cv::Mat compute_result(const DepthImage &depth) const;
private:
  double near_m_ = 0, far_m_ = 0;
};
}
