// ROS map (YAML + PGM) helpers: occupancy, clearance, global scan-to-map search (C++ port of demo/corner/map_tools.py).
// No ROS includes, so the search can be checked on synthetic scans. Poses are map-frame (x, y, yaw); scan points
// are base_link (x, y) metres.
#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace map_tools {

using Pt = std::array<double, 2>;
using Pose = std::array<double, 3>;        // x, y, yaw
using Scored = std::array<double, 4>;      // score, x, y, yaw

inline double wrap(double a) { return std::remainder(a, 2 * M_PI); }

// numpy.arange(start, stop, step)
inline std::vector<double> arange(double start, double stop, double step) {
  std::vector<double> v;
  const auto n = static_cast<long>(std::ceil((stop - start) / step));
  for (long i = 0; i < n; ++i) v.push_back(start + i * step);
  return v;
}

// A nav2/slam_toolbox map: trinary PGM, row 0 at the top, origin = lower-left corner of the image.
struct MapGrid {
  double res, ox, oy;
  int h, w;
  cv::Mat occupied, free, dist;  // CV_8U, CV_8U, CV_32F [m] to the nearest occupied cell

  explicit MapGrid(const std::string & yaml_path) {
    const auto meta = YAML::LoadFile(yaml_path);
    const auto image = (std::filesystem::path(yaml_path).parent_path() / meta["image"].as<std::string>()).string();
    cv::Mat img = cv::imread(image, cv::IMREAD_UNCHANGED);
    if (img.empty()) throw std::runtime_error("map image not found: " + meta["image"].as<std::string>());
    if (img.channels() != 1) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    img.convertTo(img, CV_64F);
    res = meta["resolution"].as<double>();
    ox = meta["origin"][0].as<double>();
    oy = meta["origin"][1].as<double>();
    h = img.rows;
    w = img.cols;
    const cv::Mat occupancy = (255.0 - img) / 255.0;  // negate 0
    const double occ_t = meta["occupied_thresh"] ? meta["occupied_thresh"].as<double>() : 0.65;
    const double free_t = meta["free_thresh"] ? meta["free_thresh"].as<double>() : 0.196;
    occupied = occupancy >= occ_t;
    free = occupancy <= free_t - 1e-6;
    cv::Mat not_occ = occupied == 0;
    not_occ.convertTo(not_occ, CV_8U, 1.0 / 255);
    cv::distanceTransform(not_occ, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    dist *= res;
  }

  Pt to_xy(double col, double row) const { return {ox + (col + 0.5) * res, oy + (h - row - 0.5) * res}; }

  // map (x, y) of free cells at least `clearance` m from any wall, on a `step` m grid (row-major, as numpy).
  std::vector<Pt> free_cells(double clearance, double step) const {
    const int k = std::max(1, static_cast<int>(std::lround(step / res)));
    std::vector<Pt> out;
    for (int r = 0; r < h; r += k)
      for (int c = 0; c < w; c += k)
        if (free.at<uchar>(r, c) && dist.at<float>(r, c) >= clearance) out.push_back(to_xy(c, r));
    return out;
  }
};

struct Localization {
  std::optional<Pose> pose;
  double score = 0, second = 0;
  std::optional<Pose> second_pose;
  bool ambiguous = true;
};

// Likelihood-field matcher: a scan point scores exp(-d^2 / 2 sigma^2), d = distance to the nearest wall.
class ScanMatcher {
public:
  explicit ScanMatcher(const MapGrid & grid, double sigma = 0.05) : g_(grid) {
    cv::Mat d2 = grid.dist.mul(grid.dist) / (-2 * sigma * sigma);
    cv::exp(d2, lik_);
  }

  // Mean point score of pts (base_link) at pose.
  double score(const Pose & p, const std::vector<Pt> & pts) const {
    const double c = std::cos(p[2]), s = std::sin(p[2]);
    double sum = 0;
    for (const auto & q : pts) {
      const double x = p[0] + q[0] * c - q[1] * s, y = p[1] + q[0] * s + q[1] * c;
      const int col = static_cast<int>(std::floor((x - g_.ox) / g_.res));
      const int row = static_cast<int>(std::floor(g_.h - (y - g_.oy) / g_.res));
      if (col >= 0 && row >= 0 && col < g_.w && row < g_.h) sum += lik_.at<float>(row, col);
    }
    return static_cast<float>(sum / pts.size());
  }

  // Best distinct poses over the free map (or only within radius of near): [(score, x, y, yaw)], best first.
  // 6 cm / 3 deg: 6 s on the Jetson for arena3_clean (Python), synthetic scans <= 1.1 cm after refine.
  std::vector<Scored> global_search(const std::vector<Pt> & pts, double clearance = 0.18, double step = 0.06,
                                    double angle_step_deg = 3.0, size_t top = 6, std::optional<Pt> near = {},
                                    double radius = 0.5, std::optional<double> yaw = {},
                                    double yaw_span_deg = 20.0) const {
    auto cells = g_.free_cells(clearance, step);
    if (near) {
      std::vector<Pt> kept;
      for (const auto & c : cells)
        if (std::hypot(c[0] - (*near)[0], c[1] - (*near)[1]) <= radius) kept.push_back(c);
      cells = kept;
      if (cells.empty()) return {};
    }
    // yaw: only headings within yaw_span_deg of it (the robot is placed facing a fixed way, user 2026-11-14)
    std::vector<double> yaws;
    if (!yaw)
      for (double d : arange(0.0, 360.0, angle_step_deg)) yaws.push_back(d * M_PI / 180);
    else
      for (double d : arange(-yaw_span_deg, yaw_span_deg + 1e-9, angle_step_deg)) yaws.push_back(*yaw + d * M_PI / 180);
    const size_t n = cells.size();
    std::vector<float> scores(yaws.size() * n);
    for (size_t i = 0; i < yaws.size(); ++i)
      for (size_t j = 0; j < n; ++j) scores[i * n + j] = score({cells[j][0], cells[j][1], yaws[i]}, pts);
    std::vector<size_t> order(scores.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scores[a] < scores[b]; });
    std::reverse(order.begin(), order.end());
    std::vector<Scored> picks;
    for (size_t flat : order) {  // non-maximum suppression: distinct by 0.3 m or 20 deg
      const size_t i = flat / n, j = flat % n;
      const double x = cells[j][0], y = cells[j][1], a = yaws[i];
      bool distinct = true;
      for (const auto & p : picks)
        distinct &= std::hypot(x - p[1], y - p[2]) > 0.3 || std::abs(wrap(a - p[3])) > 20 * M_PI / 180;
      if (distinct) {
        picks.push_back({scores[flat], x, y, a});
        if (picks.size() == top) break;
      }
    }
    return picks;
  }

  // Local grid search around pose, coarse then fine -> (score, x, y, yaw).
  Scored refine(const Pose & pose, const std::vector<Pt> & pts, double span = 0.06, double step = 0.01,
                double span_deg = 4.0, double step_deg = 0.5) const {
    double x0 = pose[0], y0 = pose[1], yaw0 = pose[2];
    Scored best{};
    const double stages[2][4] = {{span, step, span_deg, step_deg}, {step, step / 4, step_deg, step_deg / 4}};
    for (const auto & st : stages) {
      const auto d = arange(-st[0], st[0] + 1e-9, st[1]);
      const auto a = arange(-st[2], st[2] + 1e-9, st[3]);
      bool first = true;
      for (double dx : d)          // meshgrid(indexing='ij').ravel(): x, then y, then yaw; argmax keeps the first
        for (double dy : d)
          for (double da : a) {
            const Pose p{x0 + dx, y0 + dy, yaw0 + da * M_PI / 180};
            const double s = score(p, pts);
            if (first || s > best[0]) best = {s, p[0], p[1], p[2]};
            first = false;
          }
      x0 = best[1]; y0 = best[2]; yaw0 = best[3];
    }
    return best;
  }

  // Global search + refinement. near: only candidates within radius of it compete (the robot was placed there).
  Localization localize(const std::vector<Pt> & pts, double clearance = 0.18, double ambiguity = 0.92,
                        std::optional<Pt> near = {}, double radius = 0.5, std::optional<double> yaw = {}) const {
    const auto cands = global_search(pts, clearance, 0.06, 3.0, 6, near, radius, yaw);
    Localization out;
    if (cands.empty()) return out;
    std::vector<Scored> refined;
    for (size_t i = 0; i < std::min<size_t>(3, cands.size()); ++i)
      refined.push_back(refine({cands[i][1], cands[i][2], cands[i][3]}, pts));
    std::sort(refined.rbegin(), refined.rend());  // descending, tuple order as Python's sorted(reverse=True)
    const auto best = refined[0];
    // the runner-up must be a different place, not the best one found again after refinement
    std::vector<Scored> others(refined.begin() + 1, refined.end());
    others.insert(others.end(), cands.begin() + std::min<size_t>(3, cands.size()), cands.end());
    std::optional<Scored> runner;
    for (const auto & r : others)
      if (std::hypot(r[1] - best[1], r[2] - best[2]) > 0.3 || std::abs(wrap(r[3] - best[3])) > 20 * M_PI / 180)
        if (!runner || r > *runner) runner = r;
    out.pose = Pose{best[1], best[2], best[3]};
    out.score = best[0];
    out.second = runner ? (*runner)[0] : 0.0;
    if (runner) out.second_pose = Pose{(*runner)[1], (*runner)[2], (*runner)[3]};
    out.ambiguous = out.second > ambiguity * best[0];
    return out;
  }

private:
  const MapGrid & g_;
  cv::Mat lik_;  // CV_32F
};

}  // namespace map_tools
