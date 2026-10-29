#pragma once

#include "craft_perception/localization_monitor.hpp"
#include <opencv2/core.hpp>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace craft_perception {

struct ScanRelocalizerConfig {
  double window_m;        // search +-window_m in x and y around the guess
  double window_rad;      // and +-window_rad in yaw
  double step_m;          // coarse grid; a second pass refines to a quarter of it
  double step_rad;
  double sigma_m;         // a scan point scores exp(-d^2 / 2 sigma^2), d = distance to the nearest wall
  double min_score;       // mean point score needed to accept the best pose
  int max_points;         // scan points used (evenly subsampled)
  int occupied_threshold; // occupancy grid value [0, 100] counted as a wall
};

// Wide scan-to-map matching for a lost EKF pose: scores every pose in a window around the guess by
// how close the scan endpoints fall to the walls of SLAM's own map (likelihood field), coarse then
// fine. Only the map built so far is the reference; there is no external landmark.
class ScanRelocalizer {
public:
  struct Result {
    Pose2 pose;
    double score;  // mean point score in [0, 1]
  };
  bool configure(const ScanRelocalizerConfig &config);
  // nav_msgs/OccupancyGrid content; the map must not be rotated (slam_toolbox's is not).
  bool set_map(int width, int height, double resolution, double origin_x, double origin_y,
               const std::vector<int8_t> &data);
  bool has_map() const { return !likelihood_.empty(); }
  // Scan endpoints in base_link [m]; empty when no pose reaches min_score.
  std::optional<Result> match(const std::vector<std::pair<double, double>> &points, const Pose2 &guess) const;
  // Mean point score of the scan at one pose: does it still fit the map where the EKF thinks we are?
  std::optional<double> score(const std::vector<std::pair<double, double>> &points, const Pose2 &pose) const;
  double min_score() const { return config_.min_score; }

private:
  // Best pose on a grid of half-widths (wm, wr) and steps (sm, sr) around center.
  Result search(const std::vector<std::pair<double, double>> &points, const Pose2 &center, double wm, double wr,
                double sm, double sr) const;
  ScanRelocalizerConfig config_{};
  cv::Mat likelihood_;  // CV_32F, per cell
  double resolution_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0;
};

}  // namespace craft_perception
