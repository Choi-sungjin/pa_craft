#include "craft_perception/depth_heatmap.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace craft_perception {
bool DepthHeatmap::configure(double near_m, double far_m) {
  if (!std::isfinite(near_m) || !std::isfinite(far_m) || near_m < 0 || far_m <= near_m) {
    return false;
  }
  near_m_ = near_m; far_m_ = far_m;
  return true;
}

bool DepthHeatmap::is_valid_input(const DepthImage &depth) const { return depth.is_valid(); }

cv::Mat DepthHeatmap::compute_result(const DepthImage &depth) const {
  cv::Mat hsv(depth.raw.size(), CV_8UC3, cv::Scalar(0, 0, 0)), bgr;
  for (int y = 0; y < depth.raw.rows; ++y) for (int x = 0; x < depth.raw.cols; ++x) {
    const float z = depth.at(y, x);
    if (std::isfinite(z) && z > 0) hsv.at<cv::Vec3b>(y, x) = {
      static_cast<uint8_t>(120 * std::clamp((z - near_m_) / (far_m_ - near_m_), 0.0, 1.0)), 255, 255};
  }
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}
}
