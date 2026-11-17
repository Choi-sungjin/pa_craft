#include "craft_perception/apriltag_reader.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
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

// gray (a crop of the image, upscaled by scale) -> detections in full-image px (crop origin at offset).
std::vector<AprilTagDetection> AprilTagReader::detect(const cv::Mat &gray, cv::Point2f offset, float scale) const {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::ArucoDetector(*dictionary_, *parameters_).detectMarkers(gray, corners, ids);
#else
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, parameters_);
#endif
  std::vector<AprilTagDetection> output;
  for (size_t i = 0; i < ids.size(); ++i) {
    for (auto &point : corners[i]) point = point / scale + offset;
    cv::Point2f center;
    for (const auto &point : corners[i]) center += point * 0.25f;
    AprilTagDetection detection{ids[i], center, cv::Rect2f(cv::boundingRect(corners[i]))};
    for (size_t k = 0; k < 4 && k < corners[i].size(); ++k) detection.corners[k] = corners[i][k];
    output.push_back(detection);
  }
  return output;
}

namespace {
void sort_left_to_right(std::vector<AprilTagDetection> &tags) {
  std::sort(tags.begin(), tags.end(), [](const auto &a, const auto &b) { return a.center.x < b.center.x; });
}
}  // namespace

std::vector<AprilTagDetection> AprilTagReader::compute_result(const cv::Mat &bgr) const {
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  auto output = detect(gray, {0, 0}, 1.0f);
  sort_left_to_right(output);
  return output;
}

std::vector<AprilTagDetection> AprilTagReader::compute_result(const cv::Mat &bgr,
  const std::vector<cv::Rect2f> &rois, double grow, int upscale) const {
  std::vector<AprilTagDetection> output;
  if (!(grow >= 1.0) || upscale < 1) return output;
  const cv::Rect image(0, 0, bgr.cols, bgr.rows);
  cv::Mat gray, scaled;
  for (const auto &roi : rois) {
    const cv::Point2f c(roi.x + roi.width / 2, roi.y + roi.height / 2);
    const float w = roi.width * static_cast<float>(grow), h = roi.height * static_cast<float>(grow);
    const cv::Rect area = cv::Rect(cv::Point(static_cast<int>(std::floor(c.x - w / 2)), static_cast<int>(std::floor(c.y - h / 2))),
      cv::Point(static_cast<int>(std::ceil(c.x + w / 2)), static_cast<int>(std::ceil(c.y + h / 2)))) & image;
    if (area.width < 4 || area.height < 4) continue;
    cv::cvtColor(bgr(area), gray, cv::COLOR_BGR2GRAY);
    if (upscale > 1) cv::resize(gray, scaled, cv::Size(), upscale, upscale, cv::INTER_CUBIC);
    for (const auto &tag : detect(upscale > 1 ? scaled : gray, cv::Point2f(area.tl()), static_cast<float>(upscale))) {
      const bool seen = std::any_of(output.begin(), output.end(), [&tag](const AprilTagDetection &o) {
        return o.id == tag.id && cv::norm(o.center - tag.center) < 3.0;
      });
      if (!seen) output.push_back(tag);
    }
  }
  sort_left_to_right(output);
  return output;
}
}  // namespace craft_perception
