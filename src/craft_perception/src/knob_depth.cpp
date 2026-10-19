#include "craft_perception/knob_depth.hpp"
#include "geometry_utils.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace craft_perception {
using detail::clipped;
bool KnobDepth::configure(double grow, double tolerance_m, int minimum_points) {
  if (!std::isfinite(grow) || grow <= 1 || !std::isfinite(tolerance_m) ||
      tolerance_m <= 0 || minimum_points < 3) return false;
  grow_ = grow; tolerance_m_ = tolerance_m; minimum_points_ = minimum_points;
  return true;
}

bool KnobDepth::is_valid_input(
  const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &in) const {
  return depth.is_valid() && std::isfinite(in.fx) && std::isfinite(in.fy) &&
    in.fx > 0 && in.fy > 0 && std::isfinite(in.cx) && std::isfinite(in.cy) &&
    !clipped(box, depth.raw.size()).empty();
}

std::optional<DepthResult> KnobDepth::compute_result(
  const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &in) const {
  const cv::Point2f center(box.x + box.width / 2, box.y + box.height / 2);
  const auto inner = clipped({center.x - box.width / 4, center.y - box.height / 4,
                              box.width / 2, box.height / 2}, depth.raw.size());
  std::vector<float> valid;
  for (int y = inner.y; y < inner.y + inner.height; ++y) {
    for (int x = inner.x; x < inner.x + inner.width; ++x) {
      const float z = depth.at(y, x);
      if (std::isfinite(z) && z > 0) valid.push_back(z);
    }
  }
  if (valid.empty()) return {};
  const size_t mid = valid.size() / 2;
  std::nth_element(valid.begin(), valid.begin() + mid, valid.end());
  double z = valid[mid];
  if (valid.size() % 2 == 0) z = (z + *std::max_element(valid.begin(), valid.begin() + mid)) / 2;
  DepthResult result{{(center.x - in.cx) / in.fx * z, (center.y - in.cy) / in.fy * z, z}};
  const auto outer = clipped({float(center.x - box.width * grow_ / 2), float(center.y - box.height * grow_ / 2),
                              float(box.width * grow_), float(box.height * grow_)}, depth.raw.size());
  std::vector<cv::Vec3d> points;
  for (int y = outer.y; y < outer.y + outer.height; ++y) {
    for (int x = outer.x; x < outer.x + outer.width; ++x) {
      if (box.contains(cv::Point2f(x, y))) continue;
      const float d = depth.at(y, x);
      if (std::isfinite(d) && d > 0 && std::abs(d - z) < tolerance_m_) {
        points.push_back({(x - in.cx) / in.fx * d, (y - in.cy) / in.fy * d, d});
      }
    }
  }
  if (static_cast<int>(points.size()) < minimum_points_) return result;
  cv::Vec3d mean(0, 0, 0);
  for (const auto &p : points) mean += p;
  mean /= static_cast<double>(points.size());
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const auto &p : points) { const auto d = p - mean; covariance += d * d.t(); }
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors);
  // A line or a single repeated point does not define a plane.
  if (eigenvalues.at<double>(1) < 1e-12) return result;
  result.normal = {eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1),
                   eigenvectors.at<double>(2, 2)};
  if (result.normal.dot(mean) > 0) result.normal = -result.normal;
  result.plane_rms_m = std::sqrt(std::max(0.0, eigenvalues.at<double>(2)) / points.size());
  result.normal_valid = true;
  return result;
}

}
