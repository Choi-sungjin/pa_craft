#pragma once
#include "craft_perception/perception_types.hpp"
#include <vector>

namespace craft_perception {
// YOLO boxes of obstacle classes + aligned depth -> nearest surface of each obstacle
// within range. Uses a low depth quantile of the whole box, not the center median.
class ObstacleDetector {
public:
  bool configure(const std::vector<int> &class_ids, double maximum_range_m, double near_quantile);
  bool is_valid_input(const DepthImage &depth, const Intrinsics &intrinsics) const;
  std::vector<ObjectPoint> compute_result(const DepthImage &depth,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
private:
  std::vector<int> class_ids_;
  double maximum_range_m_ = 0, near_quantile_ = 0;
};
}
