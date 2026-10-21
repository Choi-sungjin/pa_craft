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
  dictionary_ = cv::aruco::getPredefinedDictionary(found->second);
  parameters_ = cv::aruco::DetectorParameters::create();
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  return true;
}

bool AprilTagReader::is_valid_input(const cv::Mat &bgr) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3;
}

std::vector<int> AprilTagReader::compute_result(const cv::Mat &bgr) const {
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
