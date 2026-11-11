#include "craft_perception/scan_relocalizer.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>

namespace craft_perception {

bool ScanRelocalizer::configure(const ScanRelocalizerConfig &c) {
  for (double v : {c.window_m, c.window_rad, c.step_m, c.step_rad, c.sigma_m}) {
    if (!std::isfinite(v) || v <= 0.0) return false;
  }
  if (!std::isfinite(c.min_score) || c.min_score <= 0.0 || c.min_score > 1.0 || c.max_points < 10 ||
      c.occupied_threshold < 1 || c.occupied_threshold > 100 || c.step_m > c.window_m || c.step_rad > c.window_rad) {
    return false;
  }
  config_ = c;
  return true;
}

bool ScanRelocalizer::set_map(int width, int height, double resolution, double origin_x, double origin_y,
                              const std::vector<int8_t> &data) {
  if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
      data.size() != static_cast<std::size_t>(width) * height) {
    return false;
  }
  // distanceTransform measures to the nearest zero pixel: walls are 0, free and unknown 255.
  cv::Mat free(height, width, CV_8U);
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      free.at<uint8_t>(v, u) = data[v * width + u] >= config_.occupied_threshold ? 0 : 255;
    }
  }
  if (cv::countNonZero(free) == width * height) return false;  // no wall yet
  cv::Mat distance;
  cv::distanceTransform(free, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  const double k = -0.5 * (resolution * resolution) / (config_.sigma_m * config_.sigma_m);
  cv::Mat squared = distance.mul(distance) * k;
  cv::exp(squared, likelihood_);
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  return true;
}

ScanRelocalizer::Result ScanRelocalizer::search(const std::vector<std::pair<double, double>> &points,
                                                const Pose2 &center, double wm, double wr, double sm,
                                                double sr) const {
  const int nm = static_cast<int>(std::round(wm / sm)), nr = static_cast<int>(std::round(wr / sr));
  Result best{center, -1.0};
  std::vector<float> u(points.size()), v(points.size());
  for (int ia = -nr; ia <= nr; ++ia) {
    const double yaw = center.yaw + ia * sr;
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (std::size_t i = 0; i < points.size(); ++i) {  // cell coordinates at the center position
      u[i] = static_cast<float>((center.x + c * points[i].first - s * points[i].second - origin_x_) / resolution_);
      v[i] = static_cast<float>((center.y + s * points[i].first + c * points[i].second - origin_y_) / resolution_);
    }
    for (int ix = -nm; ix <= nm; ++ix) {
      const float du = static_cast<float>(ix * sm / resolution_);
      for (int iy = -nm; iy <= nm; ++iy) {
        const float dv = static_cast<float>(iy * sm / resolution_);
        double sum = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
          const int cu = static_cast<int>(std::floor(u[i] + du)), cv_ = static_cast<int>(std::floor(v[i] + dv));
          if (cu >= 0 && cv_ >= 0 && cu < likelihood_.cols && cv_ < likelihood_.rows) {
            sum += likelihood_.at<float>(cv_, cu);
          }
        }
        const double score = sum / points.size();
        if (score > best.score) best = {{center.x + ix * sm, center.y + iy * sm, yaw}, score};
      }
    }
  }
  return best;
}

std::optional<double> ScanRelocalizer::score(const std::vector<std::pair<double, double>> &points,
                                             const Pose2 &pose) const {
  if (likelihood_.empty() || points.size() < 10) return std::nullopt;
  std::vector<std::pair<double, double>> used;
  const std::size_t stride = (points.size() + config_.max_points - 1) / config_.max_points;
  for (std::size_t i = 0; i < points.size(); i += stride) used.push_back(points[i]);
  return search(used, pose, 0.0, 0.0, config_.step_m, config_.step_rad).score;
}

std::optional<ScanRelocalizer::Result> ScanRelocalizer::match(const std::vector<std::pair<double, double>> &points,
                                                              const Pose2 &guess) const {
  if (likelihood_.empty() || points.size() < 10) return std::nullopt;
  std::vector<std::pair<double, double>> used;
  const std::size_t stride = (points.size() + config_.max_points - 1) / config_.max_points;
  for (std::size_t i = 0; i < points.size(); i += stride) used.push_back(points[i]);
  const auto coarse = search(used, guess, config_.window_m, config_.window_rad, config_.step_m, config_.step_rad);
  const auto fine = search(used, coarse.pose, config_.step_m, config_.step_rad, config_.step_m / 4.0,
                           config_.step_rad / 4.0);
  if (fine.score < config_.min_score) return std::nullopt;
  return fine;
}

}  // namespace craft_perception
