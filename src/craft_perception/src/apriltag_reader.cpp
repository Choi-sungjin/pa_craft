#include "craft_perception/apriltag_reader.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <map>

namespace craft_perception {
bool AprilTagReader::configure(const std::string &family) {
  static const std::map<std::string, int> families = {
    {"tag16h5", cv::aruco::DICT_APRILTAG_16h5}, {"tag25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"tag36h10", cv::aruco::DICT_APRILTAG_36h10}, {"tag36h11", cv::aruco::DICT_APRILTAG_36h11}};
  const auto found = families.find(family);
  if (found == families.end()) return false;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(found->second));
  parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
#else
  dictionary_ = cv::aruco::getPredefinedDictionary(found->second);
  parameters_ = cv::aruco::DetectorParameters::create();
#endif
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  return true;
}

bool AprilTagReader::is_valid_input(const cv::Mat &bgr) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3;
}

std::vector<AprilTagDetection> AprilTagReader::compute_result(const cv::Mat &bgr) const {
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::ArucoDetector(*dictionary_, *parameters_).detectMarkers(gray, corners, ids);
#else
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, parameters_);
#endif
  std::vector<AprilTagDetection> output;
  for (size_t i = 0; i < ids.size(); ++i) {
    cv::Point2f center;
    for (const auto &point : corners[i]) center += point * 0.25f;
    output.push_back({ids[i], center, cv::Rect2f(cv::boundingRect(corners[i]))});
  }
  std::sort(output.begin(), output.end(), [](const auto &a, const auto &b) {
    return a.center.x < b.center.x;
  });
  return output;
}
}  // namespace craft_perception
