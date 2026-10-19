#pragma once
#include <opencv2/core.hpp>
#include <optional>

namespace craft_perception {
class KnobLever {
public:
  bool is_valid_input(const cv::Mat &bgr, const cv::Rect2f &box) const;
  std::optional<double> compute_result(const cv::Mat &bgr, const cv::Rect2f &box) const;
};
}
