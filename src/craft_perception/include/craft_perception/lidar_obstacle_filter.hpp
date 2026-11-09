#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace craft_perception {

// Raw LiDAR obstacle points (one per beam) -> points worth avoiding. Points are in the map frame.
//  1. Map walls: a point within wall_margin_m of an occupied or unknown map cell (or off the map) is a wall
//     the map already has, seen a few cm off by localization or map resolution: dropped.
//  2. Persistence: the point's cell (cell_m) must have had a surviving point within neighbor_cells in each of
//     the previous persist_frames - 1 frames (a one-frame flicker never passes; neighbor_cells allows a slow
//     mover: 2 cells of 5 cm at 10 Hz = 1 m/s).
//  3. Clusters: 8-connected kept cells with fewer than min_cluster_points points this frame are dropped.
//  4. Memory (remember()): a kept cell stays an obstacle after it leaves the field of view (or falls in the floor
//     cut beside the chassis), until scan rays pass through it in persist_frames frames in a row (seen free), or
//     memory_s after it was last kept, or clear_memory(). 0 = no memory.
struct LidarObstacleFilterConfig {
  double wall_margin_m = 0.06;
  double cell_m = 0.05;
  int persist_frames = 3;
  int neighbor_cells = 2;
  int min_cluster_points = 3;
  int occupied_threshold = 65;  // map values >= this (and unknown -1) count as wall
  double memory_s = 0.0;        // [s]
};

class LidarObstacleFilter {
public:
  bool configure(const LidarObstacleFilterConfig &config);
  void set_map(const nav_msgs::msg::OccupancyGrid &map);  // axis-aligned maps only (rotated origin: ignored)
  bool has_map() const { return !distance_.empty(); }
  // keep[i] for points[i] (map frame). Advances the persistence history by one frame.
  std::vector<bool> filter(const std::vector<std::array<double, 2>> &points);
  // After filter() on the same frame: stamp [s], scan origin and every return of the scan (rays, any range) in the
  // map frame. Returns the remembered obstacle cells (centres) that no kept point of this frame covers.
  std::vector<std::array<double, 2>> remember(double stamp, const std::array<double, 2> &origin,
                                              const std::vector<std::array<double, 2>> &ray_ends,
                                              const std::vector<std::array<double, 2>> &points,
                                              const std::vector<bool> &keep);
  void reset() { history_.clear(); memory_.clear(); }
  void clear_memory() { memory_.clear(); }  // a new drive starts without the previous one's obstacles
  double wall_distance(double x, double y) const;  // [m]; 0 off the map

private:
  static int64_t key(int cx, int cy) {
    return static_cast<int64_t>((static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) | static_cast<uint32_t>(cy));
  }
  static int key_x(int64_t k) { return static_cast<int32_t>(static_cast<uint64_t>(k) >> 32); }
  static int key_y(int64_t k) { return static_cast<int32_t>(static_cast<uint64_t>(k) & 0xffffffffu); }
  LidarObstacleFilterConfig config_;
  std::vector<float> distance_;  // [m] per map cell to the nearest wall cell
  int width_ = 0, height_ = 0;
  double resolution_ = 0, origin_x_ = 0, origin_y_ = 0;
  std::deque<std::unordered_set<int64_t>> history_;  // surviving cells of previous frames, newest last
  struct Remembered { double seen; int free_frames; };
  std::unordered_map<int64_t, Remembered> memory_;   // obstacle cells (cell_m grid), last kept stamp
};

}  // namespace craft_perception
