#pragma once
#include <opencv2/core.hpp>
#include <cmath>

namespace craft_perception {
struct Detection {
  cv::Rect2f box;
  int class_id;
  float score;
};
struct Intrinsics { double fx, fy, cx, cy; };

// A cheap immutable view. Production frames retain Z16 and convert only pixels
// actually used by a module; CV_32F is also accepted for recorded/test inputs.
struct DepthImage {
  cv::Mat raw;
  double unit_m = 1.0;
  cv::Mat owner; // keeps a shared snapshot allocation alive when raw is an external view
  bool is_valid() const {
    return !raw.empty() && (raw.type() == CV_16UC1 || raw.type() == CV_32FC1) &&
           std::isfinite(unit_m) && unit_m > 0;
  }
  double at(int y, int x) const {
    return (raw.type() == CV_16UC1 ? double(raw.at<uint16_t>(y, x)) :
                                   double(raw.at<float>(y, x))) * unit_m;
  }
};
// One YOLO box placed in the camera optical frame [m].
struct ObjectPoint {
  Detection detection;
  cv::Vec3d position;
};
struct DepthResult {
  cv::Vec3d position;
  bool normal_valid = false;
  cv::Vec3d normal = {0, 0, 0};
  double plane_rms_m = 0;
};
struct PuckResult {
  bool standing = false;
  cv::Vec3d center, axis, normal;  // optical frame; axis is unit, normal faces the camera
  double height_m = 0, plane_rms_m = 0, ring_inlier_ratio = 0;
  int points = 0;
};
}  // namespace craft_perception
