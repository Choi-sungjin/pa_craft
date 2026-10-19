#pragma once
#include <opencv2/core.hpp>
#include <cmath>

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
}
