#include "craft_perception/knob_lever.hpp"
#include "geometry_utils.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace craft_perception {
using detail::clipped;
bool KnobLever::is_valid_input(const cv::Mat &bgr, const cv::Rect2f &box) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3 && !clipped(box, bgr.size()).empty();
}

std::optional<double> KnobLever::compute_result(const cv::Mat &bgr, const cv::Rect2f &box) const {
  const auto roi = clipped(box, bgr.size());
  cv::Mat hsv, low, high, red;
  cv::cvtColor(bgr(roi), hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 90, 0), cv::Scalar(10, 255, 255), low);
  cv::inRange(hsv, cv::Scalar(165, 90, 0), cv::Scalar(179, 255, 255), high);
  cv::bitwise_or(low, high, red);
  if (cv::countNonZero(red) < 30) return {};
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat values(cv::countNonZero(red), 1, CV_8U);
  int index = 0;
  for (int y = 0; y < red.rows; ++y) for (int x = 0; x < red.cols; ++x) {
    if (red.at<uint8_t>(y, x)) values.at<uint8_t>(index++, 0) = channels[2].at<uint8_t>(y, x);
  }
  cv::Mat unused, bright, mask;
  const double threshold = cv::threshold(values, unused, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::compare(channels[2], threshold, bright, cv::CMP_GT);
  cv::bitwise_and(red, bright, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));
  return angle(mask, true);
}

bool KnobLever::is_valid_input(const cv::Mat &lever_mask) const {
  return !lever_mask.empty() && lever_mask.dims == 2 && lever_mask.type() == CV_8UC1;
}

std::optional<double> KnobLever::compute_result(const cv::Mat &lever_mask) const {
  return angle(lever_mask, false);  // a learned mask needs no red-background area guard
}

std::optional<double> KnobLever::angle(const cv::Mat &mask, bool bound_area) const {
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  if (count < 2) return {};
  int largest = 1;
  for (int i = 2; i < count; ++i) if (stats.at<int>(i, cv::CC_STAT_AREA) >
      stats.at<int>(largest, cv::CC_STAT_AREA)) largest = i;
  const int area = stats.at<int>(largest, cv::CC_STAT_AREA);
  const double fraction = static_cast<double>(area) / mask.total();
  // hsv: too small = noise, too big = red background. Any mask: PCA needs a few points.
  if (area < 3 || (bound_area && (fraction < 0.02 || fraction > 0.5))) return {};
  std::vector<cv::Point> locations;
  cv::findNonZero(labels == largest, locations);
  cv::Mat points(locations.size(), 2, CV_64F);
  for (size_t i = 0; i < locations.size(); ++i) {
    points.at<double>(i, 0) = locations[i].x;
    points.at<double>(i, 1) = locations[i].y;
  }
  cv::PCA pca(points, cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Vec2d direction(pca.eigenvectors.at<double>(0, 0), pca.eigenvectors.at<double>(0, 1));
  double minimum = 1e100, maximum = -1e100;
  for (const auto &p : locations) {
    const double projection = direction.dot(cv::Vec2d(p.x - mask.cols / 2.0, p.y - mask.rows / 2.0));
    minimum = std::min(minimum, projection); maximum = std::max(maximum, projection);
  }
  if (maximum < -minimum) direction = -direction;
  double angle = std::atan2(direction[0], -direction[1]);
  if (angle < 0) angle += 2 * CV_PI;
  return angle; // radians, image clock convention: 0 up, pi/2 right
}

}
