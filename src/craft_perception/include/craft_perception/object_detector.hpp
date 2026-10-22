#pragma once
#include "craft_perception/perception_types.hpp"
#include <vector>

namespace craft_perception {
// YOLO boxes + aligned depth -> 3D object positions (median depth of the box center).
class ObjectDetector {
public:
  bool configure(double inner_fraction);  // (0,1]: center part of the box used for depth
  bool is_valid_input(const DepthImage &depth, const Intrinsics &intrinsics) const;
  std::vector<ObjectPoint> compute_result(const DepthImage &depth,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
private:
  double inner_fraction_ = 0;
};
}
