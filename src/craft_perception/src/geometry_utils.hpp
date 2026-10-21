#pragma once
#include "craft_perception/perception_types.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace craft_perception::detail {
inline cv::Rect clipped(const cv::Rect2f &box, cv::Size size) {
  if (!std::isfinite(box.x) || !std::isfinite(box.y) || !std::isfinite(box.width) ||
      !std::isfinite(box.height) || box.width <= 0 || box.height <= 0) return {};
  const auto bounded = box & cv::Rect2f(0, 0, size.width, size.height);
  if (bounded.empty()) return {};
  const int x1 = std::floor(bounded.x), y1 = std::floor(bounded.y);
  const int x2 = std::min(size.width, int(std::ceil(bounded.x + bounded.width)));
  const int y2 = std::min(size.height, int(std::ceil(bounded.y + bounded.height)));
  return {x1, y1, x2 - x1, y2 - y1};
}

// Same center, each side scaled by factor.
inline cv::Rect2f scaled(const cv::Rect2f &box, double factor) {
  const float width = box.width * factor, height = box.height * factor;
  return {box.x + box.width / 2 - width / 2, box.y + box.height / 2 - height / 2, width, height};
}

inline bool is_valid_intrinsics(const Intrinsics &in) {
  return std::isfinite(in.fx) && std::isfinite(in.fy) && in.fx > 0 && in.fy > 0 &&
    std::isfinite(in.cx) && std::isfinite(in.cy);
}

inline cv::Vec3d deproject(double x, double y, double z, const Intrinsics &in) {
  return {(x - in.cx) / in.fx * z, (y - in.cy) / in.fy * z, z};
}

// q in [0,1] of the valid (finite, >0) depth inside roi [m]; nullopt without depth.
inline std::optional<double> depth_quantile(const DepthImage &depth, const cv::Rect &roi, double q) {
  std::vector<double> valid;
  valid.reserve(roi.area());
  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const double z = depth.at(y, x);
      if (std::isfinite(z) && z > 0) valid.push_back(z);
    }
  }
  if (valid.empty()) return {};
  const auto nth = valid.begin() + std::lround(q * (valid.size() - 1));
  std::nth_element(valid.begin(), nth, valid.end());
  return *nth;
}
}  // namespace craft_perception::detail
