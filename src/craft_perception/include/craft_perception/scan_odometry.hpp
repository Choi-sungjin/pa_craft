#pragma once

#include "craft_perception/scan_relocalizer.hpp"
#include <optional>
#include <utility>
#include <vector>

namespace craft_perception {

struct ScanOdometryConfig {
  ScanRelocalizerConfig matcher;  // window/step around the prediction, sigma, min_score, points
  double grid_resolution;         // keyframe grid cell [m]
  double grid_half_extent;        // the keyframe grid covers +-this around the keyframe pose [m]
  double keyframe_m;              // a new keyframe after this much travel from the current one [m]
  double keyframe_rad;            // or this much turn [rad]
};

// LiDAR odometry with ScanRelocalizer as the matcher: each scan is matched against a keyframe scan
// (rasterized into a small likelihood grid) around the predicted pose, so drift only grows between
// keyframes, not scan to scan. Endpoint matching, so a merged virtual scan (two LiDARs) is fine.
class ScanOdometry {
public:
  struct Step {
    Pose2 delta;   // motion since the previous scan, in the previous base frame
    double score;  // matcher score in [0, 1]
  };
  bool configure(const ScanOdometryConfig &config);
  // Scan endpoints in base_link [m]; predicted: expected motion since the previous scan in its base frame.
  // Empty on the first scan and when the match fails; both restart from a keyframe at this scan.
  std::optional<Step> update(const std::vector<std::pair<double, double>> &points, const Pose2 &predicted);
  void reset() { has_keyframe_ = false; }

private:
  bool set_keyframe(const std::vector<std::pair<double, double>> &points);
  ScanOdometryConfig config_{};
  ScanRelocalizer matcher_;
  bool has_keyframe_ = false;
  Pose2 pose_{0.0, 0.0, 0.0};  // current scan pose in the keyframe frame
};

// a then b (b in a's frame), and b seen from a.
Pose2 compose(const Pose2 &a, const Pose2 &b);
Pose2 between(const Pose2 &a, const Pose2 &b);

}  // namespace craft_perception
