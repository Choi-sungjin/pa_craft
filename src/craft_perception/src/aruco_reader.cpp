#include "craft_perception/aruco_reader.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <stdexcept>

namespace craft_perception {
bool ArucoReader::configure(int dictionary) {
  if (dictionary < 0 || dictionary > cv::aruco::DICT_APRILTAG_36h11) {
    return false;
  }
  dictionary_ = cv::aruco::getPredefinedDictionary(dictionary);
  parameters_ = cv::aruco::DetectorParameters::create();
  return true;
}

bool ArucoReader::is_valid_input(const cv::Mat &bgr) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3;
}

std::vector<int> ArucoReader::compute_result(const cv::Mat &bgr) const {
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, parameters_);
  std::vector<std::pair<float, int>> ordered;
  for (size_t i = 0; i < ids.size(); ++i) {
    float center_x = 0;
    for (const auto &point : corners[i]) center_x += point.x / 4;
    ordered.emplace_back(center_x, ids[i]);
  }
  std::sort(ordered.begin(), ordered.end());
  ids.clear();
  for (const auto &item : ordered) ids.push_back(item.second);
  return ids;
}
}  // namespace craft_perception
