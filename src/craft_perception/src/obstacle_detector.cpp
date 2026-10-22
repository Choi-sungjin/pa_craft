#include "craft_perception/obstacle_detector.hpp"
#include "geometry_utils.hpp"
#include <algorithm>
#include <cmath>

namespace craft_perception {
using namespace detail;
bool ObstacleDetector::configure(
  const std::vector<int> &class_ids, double maximum_range_m, double near_quantile) {
  if (class_ids.empty() || !std::isfinite(maximum_range_m) || maximum_range_m <= 0 ||
      !std::isfinite(near_quantile) || near_quantile < 0 || near_quantile > 1) return false;
  for (const int id : class_ids) if (id < 0) return false;
  class_ids_ = class_ids; maximum_range_m_ = maximum_range_m; near_quantile_ = near_quantile;
  return true;
}

bool ObstacleDetector::is_valid_input(const DepthImage &depth, const Intrinsics &intrinsics) const {
  return depth.is_valid() && is_valid_intrinsics(intrinsics);
}

std::vector<ObjectPoint> ObstacleDetector::compute_result(const DepthImage &depth,
  const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const {
  std::vector<ObjectPoint> output;
  for (const auto &box : boxes) {
    if (std::find(class_ids_.begin(), class_ids_.end(), box.class_id) == class_ids_.end()) continue;
    const auto roi = clipped(box.box, depth.raw.size());
    if (roi.empty()) continue;
    const auto z = depth_quantile(depth, roi, near_quantile_);
    if (!z || *z > maximum_range_m_) continue;
    output.push_back({box, deproject(box.box.x + box.box.width / 2,
                                     box.box.y + box.box.height / 2, *z, intrinsics)});
  }
  return output;
}
}
