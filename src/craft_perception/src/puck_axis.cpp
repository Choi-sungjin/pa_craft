#include "craft_perception/puck_axis.hpp"
#include "geometry_utils.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace craft_perception {
using namespace detail;
namespace {
constexpr size_t kMinRingPoints = 50;

// Least-squares plane through points; normal faces the camera (origin).
void fit_plane(const std::vector<cv::Vec3d> &points, cv::Vec3d &normal, cv::Vec3d &mean, double &rms) {
  mean = {0, 0, 0};
  for (const auto &p : points) mean += p;
  mean /= static_cast<double>(points.size());
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const auto &p : points) { const auto d = p - mean; covariance += d * d.t(); }
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors);
  normal = {eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1), eigenvectors.at<double>(2, 2)};
  if (normal.dot(mean) > 0) normal = -normal;
  rms = std::sqrt(std::max(0.0, eigenvalues.at<double>(2)) / points.size());
}

// numpy.percentile default (linear interpolation).
double percentile(std::vector<double> values, double q) {
  std::sort(values.begin(), values.end());
  const double index = q * (values.size() - 1);
  const size_t low = static_cast<size_t>(index);
  const size_t high = std::min(low + 1, values.size() - 1);
  return values[low] + (values[high] - values[low]) * (index - low);
}
}  // namespace

bool PuckAxis::configure(const PuckAxisConfig &c) {
  const double values[] = {c.grow, c.plane_threshold_m, c.min_height_m, c.max_height_m,
                           c.standing_height_m, c.min_depth_m, c.max_depth_m};
  for (const double value : values) if (!std::isfinite(value) || value <= 0) return false;
  if (c.grow <= 1 || c.min_height_m >= c.max_height_m || c.min_depth_m >= c.max_depth_m ||
      c.min_points < 3 || c.ransac_iterations < 1) return false;
  config_ = c;
  return true;
}

bool PuckAxis::is_valid_input(
  const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &intrinsics) const {
  return depth.is_valid() && is_valid_intrinsics(intrinsics) && !clipped(box, depth.raw.size()).empty();
}

std::optional<PuckResult> PuckAxis::compute_result(
  const DepthImage &depth, const cv::Rect2f &box, const Intrinsics &in) const {
  const auto outer = clipped(scaled(box, config_.grow), depth.raw.size());
  std::vector<cv::Vec3d> ring, inside;
  for (int y = outer.y; y < outer.y + outer.height; ++y) {
    for (int x = outer.x; x < outer.x + outer.width; ++x) {
      const double z = depth.at(y, x);
      if (!(z > config_.min_depth_m && z < config_.max_depth_m)) continue;
      const bool in_box = x >= box.x && x < box.x + box.width && y >= box.y && y < box.y + box.height;
      (in_box ? inside : ring).push_back(deproject(x, y, z, in));
    }
  }
  if (ring.size() < kMinRingPoints) return {};

  // Fixed seed: same frame -> same plane, like the script's default_rng(0) per run.
  std::mt19937 rng(0);
  std::uniform_int_distribution<size_t> pick(0, ring.size() - 1);
  std::vector<char> best, current(ring.size());
  size_t best_count = 0;
  for (int i = 0; i < config_.ransac_iterations; ++i) {
    const size_t a = pick(rng);
    size_t b = pick(rng), c = pick(rng);
    if (a == b || a == c || b == c) continue;
    auto n = (ring[b] - ring[a]).cross(ring[c] - ring[a]);
    const double norm = cv::norm(n);
    if (norm < 1e-9) continue;
    n /= norm;
    size_t count = 0;
    for (size_t k = 0; k < ring.size(); ++k) {
      current[k] = std::abs((ring[k] - ring[a]).dot(n)) < config_.plane_threshold_m;
      count += current[k];
    }
    if (count > best_count) { best_count = count; best = current; }
  }
  if (best_count < 3) return {};
  std::vector<cv::Vec3d> inliers;
  inliers.reserve(best_count);
  for (size_t k = 0; k < ring.size(); ++k) if (best[k]) inliers.push_back(ring[k]);
  PuckResult result;
  cv::Vec3d plane_point;
  fit_plane(inliers, result.normal, plane_point, result.plane_rms_m);
  result.ring_inlier_ratio = static_cast<double>(best_count) / ring.size();
  const auto &n = result.normal;

  std::vector<cv::Vec3d> foot;
  std::vector<double> heights;
  for (const auto &p : inside) {
    const double h = (p - plane_point).dot(n);
    if (h > config_.min_height_m && h < config_.max_height_m) {
      foot.push_back(p - h * n);  // puck points dropped onto the shelf
      heights.push_back(h);
    }
  }
  if (static_cast<int>(foot.size()) < config_.min_points) return {};
  cv::Vec3d center(0, 0, 0);
  for (const auto &p : foot) center += p;
  center /= static_cast<double>(foot.size());
  result.height_m = percentile(heights, 0.95);
  result.points = static_cast<int>(foot.size());
  result.standing = result.height_m > config_.standing_height_m;
  if (result.standing) {
    result.axis = n;
  } else {
    cv::Vec3d e1 = n.cross(cv::Vec3d(1, 0, 0));
    if (cv::norm(e1) < 1e-6) e1 = n.cross(cv::Vec3d(0, 1, 0));
    e1 /= cv::norm(e1);
    const cv::Vec3d e2 = n.cross(e1);
    cv::Matx22d scatter = cv::Matx22d::zeros();
    for (const auto &p : foot) {
      const cv::Vec2d a((p - center).dot(e1), (p - center).dot(e2));
      scatter += a * a.t();
    }
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(cv::Mat(scatter), eigenvalues, eigenvectors);  // descending: row 0 = major axis
    result.axis = eigenvectors.at<double>(0, 0) * e1 + eigenvectors.at<double>(0, 1) * e2;
    if (result.axis[0] < 0) result.axis = -result.axis;  // 180 deg symmetric: point image right
  }
  result.center = center + n * (result.height_m / 2);
  return result;
}
}
