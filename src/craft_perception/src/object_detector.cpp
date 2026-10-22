#include "craft_perception/object_detector.hpp"
#include "geometry_utils.hpp"
#include <cmath>

namespace craft_perception {
using namespace detail;
bool ObjectDetector::configure(double inner_fraction) {
  if (!std::isfinite(inner_fraction) || inner_fraction <= 0 || inner_fraction > 1) return false;
  inner_fraction_ = inner_fraction;
  return true;
}

bool ObjectDetector::is_valid_input(const DepthImage &depth, const Intrinsics &intrinsics) const {
  return depth.is_valid() && is_valid_intrinsics(intrinsics);
}

std::vector<ObjectPoint> ObjectDetector::compute_result(const DepthImage &depth,
  const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const {
  std::vector<ObjectPoint> output;
  for (const auto &box : boxes) {
    const auto roi = clipped(scaled(box.box, inner_fraction_), depth.raw.size());
    if (roi.empty()) continue;
    const auto z = depth_quantile(depth, roi, 0.5);
    if (!z) continue;
    output.push_back({box, deproject(box.box.x + box.box.width / 2,
                                     box.box.y + box.box.height / 2, *z, intrinsics)});
  }
  return output;
}
}
