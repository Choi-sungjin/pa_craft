#pragma once
#include <opencv2/core.hpp>
#include <optional>

namespace craft_perception {
// Lever direction inside a knob box, image-plane rad: 0 up, clockwise. The knob box center is
// the pivot; the lever points to its end farther from the pivot. Two ways to get lever pixels:
class KnobLever {
public:
  // hsv: red pixels, the brighter (Otsu) part is the lever. bgr = whole image, box = knob box.
  bool is_valid_input(const cv::Mat &bgr, const cv::Rect2f &box) const;
  std::optional<double> compute_result(const cv::Mat &bgr, const cv::Rect2f &box) const;
  // yolo_seg: lever pixels already segmented. CV_8U mask (non-zero = lever) the size of the knob crop.
  bool is_valid_input(const cv::Mat &lever_mask) const;
  std::optional<double> compute_result(const cv::Mat &lever_mask) const;
private:
  // Largest blob -> PCA axis -> end farther from the crop center -> clock angle.
  std::optional<double> angle(const cv::Mat &mask, bool bound_area) const;
};
}
