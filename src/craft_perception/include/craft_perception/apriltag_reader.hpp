#pragma once
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif
#include <array>
#include <string>
#include <vector>

namespace craft_perception {
struct AprilTagDetection {
  int id = -1;
  cv::Point2f center;
  cv::Rect2f box;
  std::array<cv::Point2f, 4> corners{};  // TL, TR, BR, BL of the tag image
};
// AprilTags sorted left -> right, including image geometry for puck association.
class AprilTagReader {
public:
  bool configure(const std::string &family);  // tag16h5 | tag25h9 | tag36h10 | tag36h11
  bool is_valid_input(const cv::Mat &bgr) const;
  std::vector<AprilTagDetection> compute_result(const cv::Mat &bgr) const;
  // Same, searching only inside rois (image px, e.g. YOLO puck boxes): each is grown by `grow` about its
  // center, clipped to the image and upscaled by `upscale` (1 = none) before detection. Much cheaper than
  // the full frame (upper-cam bag 2026-11-11: 0.36 vs 12.1 ms/frame). A tag found from two overlapping
  // crops is kept once. Corners/center/box are in full-image px.
  std::vector<AprilTagDetection> compute_result(const cv::Mat &bgr, const std::vector<cv::Rect2f> &rois,
    double grow = 1.4, int upscale = 1) const;
private:
  std::vector<AprilTagDetection> detect(const cv::Mat &gray, cv::Point2f offset, float scale) const;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};
}
