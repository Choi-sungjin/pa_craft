#include "craft_perception/lidar_obstacle_filter.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <unordered_map>

namespace craft_perception {

bool LidarObstacleFilter::configure(const LidarObstacleFilterConfig &c) {
  if (!std::isfinite(c.wall_margin_m) || c.wall_margin_m < 0 || !std::isfinite(c.cell_m) || c.cell_m <= 0 ||
      c.persist_frames < 1 || c.neighbor_cells < 0 || c.min_cluster_points < 1 ||
      c.occupied_threshold < 1 || c.occupied_threshold > 100 || !std::isfinite(c.memory_s) || c.memory_s < 0) {
    return false;
  }
  config_ = c;
  history_.clear();
  memory_.clear();
  return true;
}

void LidarObstacleFilter::set_map(const nav_msgs::msg::OccupancyGrid &map) {
  const auto &q = map.info.origin.orientation;
  if (std::fabs(q.x) > 1e-6 || std::fabs(q.y) > 1e-6 || std::fabs(q.z) > 1e-6 || map.info.resolution <= 0 ||
      map.data.size() != static_cast<size_t>(map.info.width) * map.info.height || map.data.empty()) {
    return;
  }
  width_ = static_cast<int>(map.info.width);
  height_ = static_cast<int>(map.info.height);
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  // distanceTransform measures to the nearest zero pixel: walls (occupied or unknown) are 0, free space 255.
  cv::Mat free(height_, width_, CV_8U);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int v = map.data[static_cast<size_t>(y) * width_ + x];
      free.at<uint8_t>(y, x) = (v < 0 || v >= config_.occupied_threshold) ? 0 : 255;
    }
  }
  cv::Mat dist;
  cv::distanceTransform(free, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  distance_.assign(dist.begin<float>(), dist.end<float>());
  for (auto &d : distance_) d *= static_cast<float>(resolution_);
}

double LidarObstacleFilter::wall_distance(double x, double y) const {
  if (distance_.empty()) return 1e9;
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return 0.0;
  return distance_[static_cast<size_t>(cy) * width_ + cx];
}

std::vector<bool> LidarObstacleFilter::filter(const std::vector<std::array<double, 2>> &points) {
  std::vector<bool> keep(points.size(), false);
  std::vector<std::array<int, 2>> cell(points.size());
  std::unordered_set<int64_t> now;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &p = points[i];
    if (!std::isfinite(p[0]) || !std::isfinite(p[1])) continue;
    if (has_map() && wall_distance(p[0], p[1]) <= config_.wall_margin_m) continue;  // 1. a known wall
    cell[i] = {static_cast<int>(std::floor(p[0] / config_.cell_m)), static_cast<int>(std::floor(p[1] / config_.cell_m))};
    keep[i] = true;
    now.insert(key(cell[i][0], cell[i][1]));
  }
  // 2. Persistence over the previous persist_frames - 1 frames (not enough history yet: nothing passes).
  const auto seen_near = [this](const std::unordered_set<int64_t> &frame, int cx, int cy) {
      const int n = config_.neighbor_cells;
      for (int dy = -n; dy <= n; ++dy) {
        for (int dx = -n; dx <= n; ++dx) {if (frame.count(key(cx + dx, cy + dy))) return true;}
      }
      return false;};
  const size_t needed = static_cast<size_t>(config_.persist_frames - 1);
  for (size_t i = 0; i < points.size(); ++i) {
    if (!keep[i]) continue;
    if (history_.size() < needed) {keep[i] = false; continue;}
    for (size_t k = history_.size() - needed; k < history_.size() && keep[i]; ++k) {
      keep[i] = seen_near(history_[k], cell[i][0], cell[i][1]);
    }
  }
  history_.push_back(std::move(now));
  while (history_.size() > needed) history_.pop_front();
  // 3. Clusters of kept cells (8-connected), sized by this frame's kept points.
  std::unordered_map<int64_t, int> count;
  for (size_t i = 0; i < points.size(); ++i) {if (keep[i]) ++count[key(cell[i][0], cell[i][1])];}
  std::unordered_map<int64_t, int> label;
  std::vector<int> size;
  for (const auto &start : count) {
    if (label.count(start.first)) continue;
    const int id = static_cast<int>(size.size());
    size.push_back(0);
    std::vector<int64_t> stack{start.first};
    label[start.first] = id;
    while (!stack.empty()) {
      const int64_t k = stack.back(); stack.pop_back();
      size[id] += count[k];
      const int cx = key_x(k), cy = key_y(k);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int64_t m = key(cx + dx, cy + dy);
          if (count.count(m) && !label.count(m)) {label[m] = id; stack.push_back(m);}
        }
      }
    }
  }
  for (size_t i = 0; i < points.size(); ++i) {
    if (keep[i] && size[label[key(cell[i][0], cell[i][1])]] < config_.min_cluster_points) keep[i] = false;
  }
  return keep;
}

std::vector<std::array<double, 2>> LidarObstacleFilter::remember(
    double stamp, const std::array<double, 2> &origin, const std::vector<std::array<double, 2>> &ray_ends,
    const std::vector<std::array<double, 2>> &points, const std::vector<bool> &keep) {
  std::vector<std::array<double, 2>> out;
  if (config_.memory_s <= 0) return out;
  const auto cell_of = [this](double v) { return static_cast<int>(std::floor(v / config_.cell_m)); };
  std::unordered_set<int64_t> now;
  for (size_t i = 0; i < points.size() && i < keep.size(); ++i) {
    if (!keep[i]) continue;
    const int64_t k = key(cell_of(points[i][0]), cell_of(points[i][1]));
    now.insert(k);
    memory_[k] = {stamp, 0};
  }
  // Cells a ray crosses before its return (the last two cells, the return's and the one before, are not crossed:
  // a beam grazing the obstacle edge must not clear it).
  std::unordered_set<int64_t> crossed;
  if (!memory_.empty()) {
    for (const auto &end : ray_ends) {
      const double dx = end[0] - origin[0], dy = end[1] - origin[1], length = std::hypot(dx, dy);
      if (!std::isfinite(length)) continue;
      const int steps = static_cast<int>(length / (config_.cell_m / 2)) - 3;
      for (int s = 1; s <= steps; ++s) {
        const double t = s * (config_.cell_m / 2) / length;
        const int64_t k = key(cell_of(origin[0] + t * dx), cell_of(origin[1] + t * dy));
        if (memory_.count(k)) crossed.insert(k);
      }
    }
  }
  for (auto it = memory_.begin(); it != memory_.end();) {
    auto &cell = it->second;
    if (!now.count(it->first)) cell.free_frames = crossed.count(it->first) ? cell.free_frames + 1 : 0;
    if (stamp - cell.seen > config_.memory_s || cell.free_frames >= config_.persist_frames) {
      it = memory_.erase(it);
      continue;
    }
    if (!now.count(it->first)) {
      out.push_back({(key_x(it->first) + 0.5) * config_.cell_m, (key_y(it->first) + 0.5) * config_.cell_m});
    }
    ++it;
  }
  return out;
}

}  // namespace craft_perception
