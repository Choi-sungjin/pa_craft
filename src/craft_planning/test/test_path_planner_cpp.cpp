#include "craft_planning/path_planner.hpp"

#include <gtest/gtest.h>
#include <png.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <string>
#include <unistd.h>
#include <vector>

namespace
{
using craft_planning::PathPlanner;
using craft_planning::PathPlannerConfig;
namespace fs = std::filesystem;

geometry_msgs::msg::PoseStamped pose(double x, double y, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}

nav_msgs::msg::Odometry odom(double x, double y, const std::string & frame = "map")
{
  nav_msgs::msg::Odometry result;
  result.header.frame_id = frame;
  result.pose.pose = pose(x, y).pose;
  return result;
}

class NativePathPlanner : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static unsigned counter = 0;
    dir = fs::temp_directory_path() /
      ("craft_path_planner_" + std::to_string(getpid()) + "_" + std::to_string(counter++));
    fs::create_directories(dir);
    config.res = config.block_size = 0.1;
    config.r_robot = 0.04;
    config.margin = 0.01;
  }

  void TearDown() override {fs::remove_all(dir);}

  fs::path text_map(const std::vector<std::string> & rows)
  {
    const auto filename = dir / "map.txt";
    std::ofstream out(filename);
    for (const auto & row : rows) {out << row << '\n';}
    config.map_file = filename.string();
    return filename;
  }

  void open_map()
  {
    std::vector<std::string> rows(20, std::string(20, '.'));
    rows.front() = rows.back() = std::string(20, '#');
    for (auto & row : rows) {row.front() = row.back() = '#';}
    text_map(rows);
  }

  fs::path stl_map()
  {
    using Point = std::array<float, 3>;
    using Triangle = std::array<Point, 3>;
    std::vector<Triangle> triangles;
    const auto wall = [&triangles](float ax, float ay, float bx, float by) {
        const Point a{ax, ay, 0}, b{bx, by, 0}, c{bx, by, 0.4f}, d{ax, ay, 0.4f};
        triangles.push_back({a, b, c});
        triangles.push_back({a, c, d});
      };
    wall(-1, -0.5f, 1.5f, -0.5f);
    wall(1.5f, -0.5f, 1.5f, 1.5f);
    wall(1.5f, 1.5f, -1, 1.5f);
    wall(-1, 1.5f, -1, -0.5f);
    wall(0.22f, -0.5f, 0.22f, 0.5f);
    const auto filename = dir / "offset.stl";
    std::ofstream out(filename, std::ios::binary);
    std::array<char, 80> header{};
    out.write(header.data(), header.size());
    const auto u32 = [&out](uint32_t value) {
        for (int byte = 0; byte < 4; ++byte) {out.put(static_cast<char>(value >> (8 * byte)));}
      };
    u32(static_cast<uint32_t>(triangles.size()));
    for (const auto & triangle : triangles) {
      u32(0); u32(0); u32(0);
      for (const auto & vertex : triangle) {
        for (const auto value : vertex) {
          uint32_t bits;
          std::memcpy(&bits, &value, sizeof(bits));
          u32(bits);
        }
      }
      out.put(0); out.put(0);
    }
    return filename;
  }

  fs::path yaml_map(const std::string & image, const std::string & origin = "[0, 0, 0]")
  {
    const auto filename = dir / "map.yaml";
    std::ofstream(filename) << "image: " << image << "\nresolution: 0.1\norigin: " << origin <<
      "\nnegate: 0\nfree_thresh: 0.196\noccupied_thresh: 0.65\nmode: trinary\n";
    config.map_file = filename.string();
    return filename;
  }

  fs::path dir;
  PathPlannerConfig config;
};

TEST_F(NativePathPlanner, StraightRoutePreservesEndpointsOrderAndFrames)
{
  open_map();
  PathPlanner planner;
  planner.configure(config);
  const auto result = planner.plan(odom(0.35, 0.35), pose(1.63, 0.36));
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_GE(result.path.poses.size(), 2u);
  EXPECT_EQ(result.path.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, 0.35);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 1.63);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y, 0.36);
  EXPECT_TRUE(planner.collision_free(result.path));
  for (const auto & sample : result.path.poses) {
    EXPECT_EQ(sample.header.frame_id, "map");
    EXPECT_NEAR(std::hypot(sample.pose.orientation.z, sample.pose.orientation.w), 1.0, 1e-10);
  }
}

TEST_F(NativePathPlanner, RoutesAroundInflatedWallAndRejectsBlockedRoute)
{
  std::vector<std::string> rows(20, std::string(20, '.'));
  for (int y = 4; y < 20; ++y) {rows[y][10] = '#';}
  text_map(rows);
  PathPlanner planner;
  planner.configure(config);
  auto route = planner.plan(odom(0.35, 0.35), pose(1.65, 0.35));
  ASSERT_TRUE(route.ok) << route.error;
  double ymax = 0;
  for (const auto & p : route.path.poses) {ymax = std::max(ymax, p.pose.position.y);}
  EXPECT_GT(ymax, 1.6);
  EXPECT_TRUE(planner.collision_free(route.path));
  for (auto & row : rows) {row[10] = '#';}
  text_map(rows);
  planner.configure(config);
  route = planner.plan(odom(0.35, 0.35), pose(1.65, 0.35));
  EXPECT_FALSE(route.ok);
  EXPECT_TRUE(route.retryable);
  EXPECT_TRUE(route.path.poses.empty());
  EXPECT_FALSE(route.error.empty());
}

TEST_F(NativePathPlanner, FootprintAndMapBoundaryBlockUnsafeEndpoints)
{
  open_map();
  config.r_robot = 0.21;
  config.margin = 0.03;
  PathPlanner planner;
  planner.configure(config);
  EXPECT_TRUE(planner.plan(odom(0.2, 0.5), pose(1.5, 1.5)).retryable);
  EXPECT_FALSE(planner.plan(odom(-0.1, 0.5), pose(1.5, 1.5)).retryable);
  EXPECT_TRUE(planner.plan(odom(0.5, 0.5), pose(1.95, 1.5)).retryable);
}

TEST_F(NativePathPlanner, FrameAndFiniteValidationNeverReturnsOldRoute)
{
  open_map();
  PathPlanner planner;
  planner.configure(config);
  ASSERT_TRUE(planner.plan(odom(0.35, 0.35), pose(1.65, 1.65)).ok);
  auto result = planner.plan(odom(0.35, 0.35, "odom"), pose(1.65, 1.65));
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.path.poses.empty());
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), pose(1.65, 1.65, "")).ok);
  auto invalid = pose(1.65, 1.65);
  invalid.pose.orientation.w = 0;
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), invalid).ok);
  invalid = pose(1.65, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), invalid).ok);
}

TEST_F(NativePathPlanner, DynamicObstacleSnapshotDoesNotMutateStaticMap)
{
  open_map();
  PathPlanner planner;
  planner.configure(config);
  const auto original = planner.plan(odom(0.35, 0.35), pose(1.65, 0.35));
  ASSERT_TRUE(original.ok) << original.error;
  geometry_msgs::msg::PoseArray obstacles;
  obstacles.header.frame_id = "map";
  obstacles.poses.push_back(pose(0.95, 0.35).pose);
  EXPECT_FALSE(planner.collision_free(original.path, obstacles));
  const auto detour = planner.plan(odom(0.35, 0.35), pose(1.65, 0.35), obstacles);
  ASSERT_TRUE(detour.ok) << detour.error;
  EXPECT_TRUE(planner.collision_free(detour.path, obstacles));
  EXPECT_TRUE(planner.collision_free(original.path));
  obstacles.header.frame_id = "camera";
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), pose(1.65, 0.35), obstacles).ok);
  EXPECT_FALSE(planner.collision_free(original.path, obstacles));
}

TEST_F(NativePathPlanner, CollisionChecksSegmentsNotOnlyPathSamples)
{
  open_map();
  PathPlanner planner;
  planner.configure(config);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {pose(0.35, 0.35), pose(1.65, 0.35)};
  geometry_msgs::msg::PoseArray obstacles;
  obstacles.header.frame_id = "map";
  obstacles.poses.push_back(pose(0.917, 0.35).pose);
  EXPECT_FALSE(planner.collision_free(path, obstacles));
  EXPECT_FALSE(planner.collision_free(nav_msgs::msg::Path{}));
}

TEST_F(NativePathPlanner, DoesNotCutDiagonalCorners)
{
  text_map({"#####", "##..#", "#.#.#", "#..##", "#####"});
  config.r_robot = 0;
  config.margin = 0;
  PathPlanner planner;
  planner.configure(config);
  EXPECT_FALSE(planner.plan(odom(0.15, 0.15), pose(0.35, 0.35)).ok);
}

TEST_F(NativePathPlanner, BinaryStlPreservesOffsetOriginAndDetoursAroundWall)
{
  config.map_file = stl_map().string();
  config.res = 0.05;
  config.r_robot = 0.10;
  config.margin = 0.02;
  PathPlanner planner;
  planner.configure(config);
  const auto result = planner.plan(odom(-0.4, 0), pose(0.9, 0));
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, -0.4);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.9);
  double ymax = 0;
  for (const auto & p : result.path.poses) {ymax = std::max(ymax, p.pose.position.y);}
  EXPECT_GT(ymax, 0.6);
  EXPECT_TRUE(planner.collision_free(result.path));
  config.stl_slice_heights.assign(1, 0.5);
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
}

TEST_F(NativePathPlanner, RosPgmYamlHonorsRotatedOriginAndUnknownPixels)
{
  std::ofstream image(dir / "map.pgm");
  image << "P2\n# a ROS map\n20 20\n255\n";
  for (int row = 0; row < 20; ++row) {
    for (int col = 0; col < 20; ++col) {image << (col == 10 ? 205 : 255) << ' ';}
    image << '\n';
  }
  image.close();
  yaml_map("map.pgm", "[10, 20, 1.5707963267948966]");
  PathPlanner planner;
  planner.configure(config);
  ASSERT_TRUE(planner.plan(odom(9.65, 20.35), pose(8.35, 20.35)).ok);
  EXPECT_FALSE(planner.plan(odom(9.65, 20.35), pose(9.65, 21.65)).ok);
}

TEST_F(NativePathPlanner, TransparentPngPixelsAreUnknownAndBlocked)
{
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = image.height = 20;
  image.format = PNG_FORMAT_RGBA;
  std::vector<unsigned char> rgba(20 * 20 * 4, 255);
  for (int y = 0; y < 20; ++y) {rgba[(y * 20 + 10) * 4 + 3] = 0;}
  ASSERT_NE(png_image_write_to_file(&image, (dir / "map.png").c_str(), 0, rgba.data(), 0, nullptr), 0);
  yaml_map("map.png");
  PathPlanner planner;
  planner.configure(config);
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), pose(1.65, 0.35)).ok);
}

TEST_F(NativePathPlanner, BinarySixteenBitPgmDoesNotRoundUnknownPixelsIntoFreeSpace)
{
  std::ofstream image(dir / "map.pgm", std::ios::binary);
  image << "P5\n20 20\n1000\n";
  for (int y = 0; y < 20; ++y) {
    for (int x = 0; x < 20; ++x) {
      const int value = x == 10 ? 799 : 1000;
      image.put(static_cast<char>(value >> 8));
      image.put(static_cast<char>(value & 255));
    }
  }
  image.close();
  const auto yaml = yaml_map("map.pgm");
  std::ofstream(yaml) << "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
    "free_thresh: 0.2\noccupied_thresh: 0.65\nnegate: 0\n";
  PathPlanner planner;
  planner.configure(config);
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), pose(1.65, 0.35)).ok);
}

TEST_F(NativePathPlanner, RejectsMalformedMapsAndBoundedWork)
{
  open_map();
  PathPlanner planner;
  config.max_cells = 10;
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
  config.max_cells = 2000000;
  config.max_expansions = 1;
  planner.configure(config);
  EXPECT_FALSE(planner.plan(odom(0.35, 0.35), pose(1.65, 1.65)).ok);
  config.res = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
  config.res = 0.1;
  config.map_file = (dir / "broken.stl").string();
  std::ofstream(config.map_file) << "solid ascii\nendsolid\n";
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
}

TEST_F(NativePathPlanner, InvalidReconfigurationPreservesSnapshotAndConcurrentQueriesAreIndependent)
{
  open_map();
  PathPlanner planner;
  planner.configure(config);
  auto invalid = config;
  invalid.res = 0;
  EXPECT_THROW(planner.configure(invalid), std::invalid_argument);
  auto first = std::async(std::launch::async, [&planner] {
      return planner.plan(odom(0.35, 0.35), pose(1.65, 0.35));
    });
  auto second = std::async(std::launch::async, [&planner] {
      geometry_msgs::msg::PoseArray obstacles;
      obstacles.header.frame_id = "map";
      obstacles.poses.push_back(pose(0.95, 0.35).pose);
      return planner.plan(odom(0.35, 0.35), pose(1.65, 0.35), obstacles);
    });
  ASSERT_TRUE(first.get().ok);
  ASSERT_TRUE(second.get().ok);
  EXPECT_TRUE(planner.plan(odom(0.35, 0.35), pose(1.65, 0.35)).ok);
}

TEST_F(NativePathPlanner, ExcessiveSamplingRequestFailsWithoutAllocatingUnboundedPath)
{
  open_map();
  config.arc_sample = std::numeric_limits<double>::min();
  PathPlanner planner;
  planner.configure(config);
  // arc_sample only drives fillet collision checks (path samples are 2 cm): plan around a corner.
  const auto result = planner.plan(odom(0.35, 0.35), pose(1.65, 1.65));
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST_F(NativePathPlanner, PackagedMazePlansWithinOneMazeAndRejectsCrossingPartition)
{
  config = PathPlannerConfig{};
  config.map_file = std::string(CRAFT_PLANNING_SOURCE_DIR) + "/maps/maze.stl";
  PathPlanner planner;
  planner.configure(config);
  const auto within = planner.plan(odom(-1.9, 0.5), pose(1.9, 2.85));
  ASSERT_TRUE(within.ok) << within.error;
  EXPECT_TRUE(planner.collision_free(within.path));
  EXPECT_FALSE(planner.plan(odom(-1.9, 0.5), pose(-1.9, -2.85)).ok);
}
// global_local: 6 x 3 m room, a wall from the top down to y = 0.9 at x = 3 (the route goes under it).
class GlobalLocalPlanner : public NativePathPlanner
{
protected:
  void SetUp() override
  {
    NativePathPlanner::SetUp();
    std::vector<std::string> rows(30, std::string(60, '.'));
    rows.front() = rows.back() = std::string(60, '#');
    for (auto & row : rows) {row.front() = row.back() = '#';}
    for (int r = 0; r <= 20; ++r) {rows[r][30] = '#';}
    text_map(rows);
    config.mode = "global_local";
    config.local_behind = 1.0; config.local_ahead = 4.0; config.local_update = 2.0; config.local_max_dev = 0.2;
    planner.configure(config);
  }
  static double length(const nav_msgs::msg::Path & path)
  {
    double total = 0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position, & b = path.poses[i].pose.position;
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return total;
  }
  static double distance_to(const nav_msgs::msg::Path & path, double x, double y)
  {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position, & b = path.poses[i].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
      const double t = len2 > 0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
      best = std::min(best, std::hypot(x - a.x - t * dx, y - a.y - t * dy));
    }
    return best;
  }
  static geometry_msgs::msg::Point at(const nav_msgs::msg::Path & path, double s)
  {
    double total = 0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position, & b = path.poses[i].pose.position;
      const double step = std::hypot(b.x - a.x, b.y - a.y);
      if (total + step >= s) {
        auto p = a; const double t = step > 0 ? (s - total) / step : 0; p.x += t * (b.x - a.x); p.y += t * (b.y - a.y);
        return p;
      }
      total += step;
    }
    return path.poses.back().pose.position;
  }
  PathPlanner planner;
};

TEST_F(GlobalLocalPlanner, GlobalOnceLocalWindowCutFromIt)
{
  const auto plan = planner.plan(odom(0.35, 2.65), pose(5.65, 2.65));
  ASSERT_TRUE(plan.ok) << plan.error;
  ASSERT_GE(plan.global.poses.size(), 2u);
  EXPECT_TRUE(plan.stops.empty());
  EXPECT_TRUE(planner.collision_free(plan.global));
  EXPECT_TRUE(planner.collision_free(plan.path));
  EXPECT_GT(length(plan.global), 6.0);   // down under the wall and up again (straight line 5.3 m)
  EXPECT_LE(length(plan.path), config.local_ahead + config.local_max_dev + 0.1);
  EXPECT_NEAR(plan.path.poses.front().pose.position.x, 0.35, 1e-9);
  EXPECT_NEAR(plan.path.poses.front().pose.position.y, 2.65, 1e-9);
  EXPECT_EQ(plan.path.poses.size(), plan.speed_limit.size());
  for (const auto & q : plan.path.poses) {  // samples of the global path itself
    EXPECT_LT(distance_to(plan.global, q.pose.position.x, q.pose.position.y), 1e-6);
  }
  EXPECT_NEAR(craft_planning::path_progress(plan.global, plan.path.poses.back().pose.position), config.local_ahead, 0.3);
}

// Via points (run_tour.py, 2026-11-12): the goal is 0.5 m from the start, the route still goes under the wall to the
// via point in the other half and comes back, as one path without a stop.
TEST_F(GlobalLocalPlanner, RouteGoesThroughTheViaPointsInOrder)
{
  EXPECT_LT(length(planner.plan(odom(1.0, 2.0), pose(1.5, 2.0)).global), 0.6);
  geometry_msgs::msg::Point far;
  far.x = 5.0; far.y = 2.0;
  const std::vector<geometry_msgs::msg::Point> via{far};
  const auto plan = planner.plan(odom(1.0, 2.0), pose(1.5, 2.0), {}, false, nullptr, &via);
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_TRUE(plan.stops.empty());
  EXPECT_TRUE(planner.collision_free(plan.global));
  EXPECT_GT(length(plan.global), 7.0);
  EXPECT_LT(distance_to(plan.global, 5.0, 2.0), 1.0);  // the smoothing rounds the turn at the via point
  EXPECT_NEAR(plan.global.poses.back().pose.position.x, 1.5, 1e-9);
  far.x = 3.0; far.y = 2.0;  // in the wall
  const std::vector<geometry_msgs::msg::Point> blocked{far};
  EXPECT_FALSE(planner.plan(odom(1.0, 2.0), pose(1.5, 2.0), {}, false, nullptr, &blocked).ok);
}

// A start a little inside r_robot (stopped at the padding edge, grid rounding) escapes; deeper in still fails
// (user 2026-11-13, sim 04_goal_blocked_replan looped on "Start is in an obstacle").
TEST_F(GlobalLocalPlanner, StartJustInsideThePaddingEscapes)
{
  config.margin = 0.0; config.r_robot = 0.10;  // as planning.yaml: margin 0, 1 cell = 0.1 m here
  planner.configure(config);
  geometry_msgs::msg::PoseArray obstacles;
  obstacles.header.frame_id = "map";
  obstacles.poses.push_back(pose(1.15, 1.15).pose);  // diagonal cell: start clearance 0.091 (r_robot - 0.009)
  const auto plan = planner.plan(odom(1.05, 1.05), pose(1.05, 2.05), obstacles);
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_NEAR(plan.path.poses.front().pose.position.x, 1.05, 1e-9);
  EXPECT_NEAR(plan.path.poses.front().pose.position.y, 1.05, 1e-9);
  obstacles.poses.back() = pose(1.15, 1.05).pose;  // next cell: clearance 0.05, 5 cm inside: still refused
  const auto deep = planner.plan(odom(1.05, 1.05), pose(1.05, 2.05), obstacles);
  EXPECT_FALSE(deep.ok);
  EXPECT_NE(deep.error.find("Start is in an obstacle"), std::string::npos) << deep.error;
}

TEST_F(GlobalLocalPlanner, RenewedWindowKeepsThePathUnderTheRobotAndReachesTheGoal)
{
  const auto first = planner.plan(odom(0.35, 2.65), pose(5.65, 2.65));
  ASSERT_TRUE(first.ok) << first.error;
  // The robot drove the first local path for local_update.
  const auto here = at(first.path, config.local_update);
  const auto next = planner.local_path(first.global, odom(here.x, here.y));
  ASSERT_TRUE(next.ok) << next.error;
  EXPECT_TRUE(planner.collision_free(next.path));
  const double s_ego = craft_planning::path_progress(first.global, here);
  EXPECT_NEAR(craft_planning::path_progress(first.global, next.path.poses.front().pose.position),
    s_ego - config.local_behind, 0.25);
  // Around the robot (behind it to 0.25 m ahead, the control lookahead) the new path lies on the old one.
  for (const auto & q : next.path.poses) {
    const double dx = q.pose.position.x - here.x, dy = q.pose.position.y - here.y;
    if (std::hypot(dx, dy) < 0.25) {EXPECT_LT(distance_to(first.path, q.pose.position.x, q.pose.position.y), 0.01);}
  }
  for (const auto & q : next.path.poses) {
    EXPECT_LT(distance_to(first.global, q.pose.position.x, q.pose.position.y), 1e-6);
  }
  // Near the end the window is clipped at the global end and finishes exactly at the goal.
  const auto late = at(first.global, length(first.global) - 1.0);
  const auto last = planner.local_path(first.global, odom(late.x, late.y));
  ASSERT_TRUE(last.ok) << last.error;
  EXPECT_DOUBLE_EQ(last.path.poses.back().pose.position.x, 5.65);
  EXPECT_DOUBLE_EQ(last.path.poses.back().pose.position.y, 2.65);
}

TEST_F(GlobalLocalPlanner, RejectsUnknownModeAndLocalPathInArcMode)
{
  config.mode = "spline";
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
  config.mode = "arc";
  planner.configure(config);
  const auto arc = planner.plan(odom(0.35, 2.65), pose(5.65, 2.65));
  ASSERT_TRUE(arc.ok);
  EXPECT_TRUE(arc.global.poses.empty());
  EXPECT_FALSE(planner.local_path(arc.path, odom(0.35, 2.65)).ok);
}

TEST_F(GlobalLocalPlanner, SpeedProfileKeepsWheelsAndAccelerationInLimits)
{
  const auto first = planner.plan(odom(0.35, 2.65), pose(5.65, 2.65));
  ASSERT_TRUE(first.ok) << first.error;
  const auto late = at(first.global, length(first.global) - 1.0);
  const auto last = planner.local_path(first.global, odom(late.x, late.y));
  ASSERT_TRUE(last.ok) << last.error;
  for (const auto * plan : {&first, &last}) {
    const auto & poses = plan->path.poses;
    const auto & v = plan->speed_limit;
    ASSERT_EQ(poses.size(), v.size());
    EXPECT_DOUBLE_EQ(v.front(), config.v_min);
    double turn_max = 0;
    for (std::size_t i = 0; i < v.size(); ++i) {
      EXPECT_GE(v[i], config.v_min);
      EXPECT_LE(v[i], std::min(config.v_ref, config.max_wheel * config.wheel_radius) + 1e-12);
      if (i >= 3 && i + 3 < v.size()) {  // curvature over +-3 samples, as the planner takes it
        const auto & a = poses[i - 3].pose.position, & b = poses[i].pose.position, & c = poses[i + 3].pose.position;
        const double ax = b.x - a.x, ay = b.y - a.y, bx = c.x - b.x, by = c.y - b.y;
        const double k = 2 * std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by)) / (std::hypot(ax, ay) + std::hypot(bx, by));
        EXPECT_LE(v[i] * (1 + k * config.wheel_separation / 2) / config.wheel_radius, config.max_wheel + 1e-9) << i;
        turn_max = std::max(turn_max, k);
      }
      if (i > 0) {
        const auto & a = poses[i - 1].pose.position, & b = poses[i].pose.position;
        const double room = 2 * config.a_lon_max * std::hypot(b.x - a.x, b.y - a.y) + 1e-9;
        EXPECT_LE(v[i] * v[i], v[i - 1] * v[i - 1] + room) << i;
        EXPECT_LE(v[i - 1] * v[i - 1], v[i] * v[i] + room) << i;
      }
    }
    if (plan == &first) {EXPECT_GT(turn_max, 0.5);}  // the route bends under the wall: the wheel limit is exercised
  }
  // The window that ends at the goal creeps over the last stop_distance.
  EXPECT_DOUBLE_EQ(last.speed_limit.back(), config.v_min);
  double from_end = 0;
  for (std::size_t i = last.path.poses.size() - 1; i > 0 && from_end <= config.stop_distance - 0.03; --i) {
    EXPECT_DOUBLE_EQ(last.speed_limit[i], config.v_min);
    const auto & a = last.path.poses[i - 1].pose.position, & b = last.path.poses[i].pose.position;
    from_end += std::hypot(b.x - a.x, b.y - a.y);
  }
}
// Box approach: 6 x 3 m room (row 0 = top), a 0.2 m box at (4.5, 1.5) and a wall 0.4 m east of it, so the east face
// cannot be approached (its approach line would cross the wall).
class BoxApproach : public NativePathPlanner
{
protected:
  void SetUp() override
  {
    NativePathPlanner::SetUp();
    std::vector<std::string> rows(30, std::string(60, '.'));
    rows.front() = rows.back() = std::string(60, '#');
    for (auto & row : rows) {row.front() = row.back() = '#';}
    for (int r = 14; r <= 15; ++r) {rows[r][44] = rows[r][45] = '#';}  // box: x 4.4..4.6, y 1.4..1.6
    for (int r = 8; r <= 21; ++r) {rows[r][49] = '#';}                 // wall: x 4.9..5.0, y 0.8..2.2
    text_map(rows);
    config.mode = "global_local";
    config.r_robot = 0.15;  // padding 0.16 m: one 0.1 m cell around every wall (0.05 m would not reach a neighbour)
    planner.configure(config);
  }
  // Heading of the last approach_len of a path and its largest sideways deviation from that straight line.
  static std::pair<double, double> approach_line(const nav_msgs::msg::Path & path, double length)
  {
    const auto & end = path.poses.back().pose.position;
    std::size_t i = path.poses.size() - 1;
    double along = 0;
    while (i > 0 && along < length) {
      const auto & a = path.poses[i - 1].pose.position, & b = path.poses[i].pose.position;
      along += std::hypot(b.x - a.x, b.y - a.y); --i;
    }
    const auto & begin = path.poses[i].pose.position;
    const double yaw = std::atan2(end.y - begin.y, end.x - begin.x);
    double off = 0;
    for (std::size_t k = i; k < path.poses.size(); ++k) {
      const auto & p = path.poses[k].pose.position;
      off = std::max(off, std::abs(-std::sin(yaw) * (p.x - begin.x) + std::cos(yaw) * (p.y - begin.y)));
    }
    return {yaw, off};
  }
  PathPlanner planner;
};

TEST_F(BoxApproach, PicksAnApproachableFaceAndEndsAlongItsNormal)
{
  const craft_planning::BoxTarget box{4.5, 1.5, M_PI / 2, 0.2, 0.15};
  const auto plan = planner.plan_box(odom(0.5, 0.5), box);
  ASSERT_TRUE(plan.ok) << plan.error;
  const craft_planning::BoxApproach approach{box, plan.goal.pose};  // the goal is inside the box padding (0.16 m)
  EXPECT_TRUE(planner.collision_free(plan.global, {}, &approach));
  EXPECT_LT(std::cos(plan.face_normal), 0.9);   // not the east face (normal 0): the wall blocks its approach line
  // The goal: standoff from the face along its normal, heading into the box; the path ends exactly there.
  const double reach = 0.1 + 0.15;
  EXPECT_NEAR(plan.goal.pose.position.x, 4.5 + reach * std::cos(plan.face_normal), 1e-9);
  EXPECT_NEAR(plan.goal.pose.position.y, 1.5 + reach * std::sin(plan.face_normal), 1e-9);
  EXPECT_NEAR(plan.global.poses.back().pose.position.x, plan.goal.pose.position.x, 1e-9);
  EXPECT_NEAR(plan.global.poses.back().pose.position.y, plan.goal.pose.position.y, 1e-9);
  // The last approach_len is straight along the face normal, into the box.
  const auto [yaw, off] = approach_line(plan.global, config.approach_len - 0.02);
  EXPECT_NEAR(std::remainder(yaw - (plan.face_normal + M_PI), 2 * M_PI), 0.0, 0.02);
  EXPECT_LT(off, 0.005);
}

TEST_F(BoxApproach, AnyBoxYawGetsAStraightApproachAlongItsFace)
{
  // A box turned 30 deg (only the target matters for the approach): the approach is not rounded to a quarter turn.
  const craft_planning::BoxTarget box{1.5, 1.5, 30 * M_PI / 180, 0.2, 0.15};
  const auto plan = planner.plan_box(odom(0.5, 0.5), box);
  ASSERT_TRUE(plan.ok) << plan.error;
  const double turns = std::remainder(plan.face_normal - box.yaw, M_PI / 2);
  EXPECT_NEAR(turns, 0.0, 1e-9);  // one of the four face normals
  const auto [yaw, off] = approach_line(plan.global, config.approach_len - 0.02);
  EXPECT_NEAR(std::remainder(yaw - (plan.face_normal + M_PI), 2 * M_PI), 0.0, 0.02);
  EXPECT_LT(off, 0.005);
}

TEST_F(BoxApproach, TheBoxPaddingDoesNotBlockItsOwnApproach)
{
  // Standoff 5 cm: every goal is in the box's padding (0.16 m), which the approach ignores; the box still blocks.
  const craft_planning::BoxTarget box{4.5, 1.5, M_PI / 2, 0.2, 0.05};
  const auto plan = planner.plan_box(odom(0.5, 0.5), box);
  ASSERT_TRUE(plan.ok) << plan.error;
  const craft_planning::BoxApproach approach{box, plan.goal.pose};
  EXPECT_FALSE(planner.collision_free(plan.global));            // the plain padding check stops at the box
  EXPECT_TRUE(planner.collision_free(plan.global, {}, &approach));
  const auto here = plan.global.poses[plan.global.poses.size() / 2].pose.position;
  const auto renewed = planner.local_path(plan.global, odom(here.x, here.y), {}, &approach);
  ASSERT_TRUE(renewed.ok) << renewed.error;
  // A goal inside the box itself is still refused.
  nav_msgs::msg::Path into = plan.global;
  auto inside = into.poses.back();
  inside.pose.position.x = 4.5; inside.pose.position.y = 1.5;
  into.poses.push_back(inside);
  EXPECT_FALSE(planner.collision_free(into, {}, &approach));
}

TEST_F(BoxApproach, WithAStageLeadThePathEndsAtTheStagePoint)
{
  // stage_lead 0.2: an ordinary path to approach_len + 0.2 behind the face goal on its normal; the face goal is the
  // same one as without (the dock goes there).
  const craft_planning::BoxTarget box{4.5, 1.5, M_PI / 2, 0.2, 0.15};
  const auto full = planner.plan_box(odom(0.5, 0.5), box);
  const auto staged = planner.plan_box(odom(0.5, 0.5), box, {}, nullptr, 0.2);
  ASSERT_TRUE(full.ok && staged.ok) << full.error << staged.error;
  EXPECT_NEAR(staged.face_normal, full.face_normal, 1e-9);
  const double back = config.approach_len + 0.2, heading = staged.face_normal + M_PI;
  const auto & end = staged.global.poses.back().pose.position;
  EXPECT_NEAR(end.x, staged.goal.pose.position.x - back * std::cos(heading), 1e-9);
  EXPECT_NEAR(end.y, staged.goal.pose.position.y - back * std::sin(heading), 1e-9);
  EXPECT_TRUE(planner.collision_free(staged.global));
  // Only one face allowed (a correction): that face or none.
  const double west = M_PI;
  const auto only = planner.plan_box(odom(0.5, 0.5), box, {}, &west, 0.2);
  ASSERT_TRUE(only.ok) << only.error;
  EXPECT_NEAR(std::remainder(only.face_normal - west, 2 * M_PI), 0.0, 1e-9);
}

TEST_F(BoxApproach, AStagePointInThePaddingMovesUpTowardTheEntry)
{
  // South face: goal y 1.25, entry y 0.95, stage point (lead 0.2) y 0.75. An obstacle at y 0.62 puts the stage point in
  // the padding: it moves up (5 cm steps) but stays behind the entry.
  const craft_planning::BoxTarget box{4.5, 1.5, M_PI / 2, 0.2, 0.15};
  geometry_msgs::msg::PoseArray obstacles; obstacles.header.frame_id = "map";
  geometry_msgs::msg::Pose o; o.position.x = 4.5; o.position.y = 0.62; o.orientation.w = 1.0;
  obstacles.poses.push_back(o);
  const double south = -M_PI / 2;
  const auto plan = planner.plan_box(odom(0.5, 0.5), box, obstacles, &south, 0.2);
  ASSERT_TRUE(plan.ok) << plan.error;
  const auto & end = plan.global.poses.back().pose.position;
  EXPECT_NEAR(end.x, 4.5, 1e-9);
  EXPECT_GT(end.y, 0.75 + 1e-6);
  EXPECT_LE(end.y, 0.95 + 1e-9);
}

TEST_F(BoxApproach, WallsStillRuleFacesOut)
{
  // Standoff 0.25 m: the east goal (x 4.85) is in the wall's padding (wall x 4.9..5.0); the box padding aside, the
  // other faces are fine, and the east face is reported as the one in the padding.
  const auto plan = planner.plan_box(odom(0.5, 0.5), craft_planning::BoxTarget{4.5, 1.5, M_PI / 2, 0.2, 0.25});
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_LT(std::cos(plan.face_normal), 0.9);
  // Off the map on every side: no face at all.
  const auto none = planner.plan_box(odom(0.5, 0.5), craft_planning::BoxTarget{3.0, 1.5, M_PI / 2, 0.2, 3.0});
  EXPECT_FALSE(none.ok);
  EXPECT_NE(none.error.find("No approachable box face"), std::string::npos) << none.error;
}

}  // namespace

TEST(PathProgress, RepeatVisitsUseTheRequestedArcInterval)
{
  nav_msgs::msg::Path route;
  for (const auto xy : std::vector<std::pair<double,double>>{{0,0},{1,0},{1,1},{0,1},{0,0},{1,0}}) {
    geometry_msgs::msg::PoseStamped pose; pose.pose.position.x=xy.first; pose.pose.position.y=xy.second;
    route.poses.push_back(pose);
  }
  geometry_msgs::msg::Point p; p.x=.5;
  EXPECT_NEAR(craft_planning::path_progress(route,p),.5,1e-9);
  EXPECT_NEAR(craft_planning::path_progress(route,p,4.0,5.0),4.5,1e-9);
  p.x=0;
  EXPECT_NEAR(craft_planning::path_progress(route,p,3.8,4.2),4.0,1e-9);
  EXPECT_NEAR(craft_planning::path_length(route),5.0,1e-9);
  EXPECT_THROW(craft_planning::path_progress(route,p,2.,1.),std::invalid_argument);
}

TEST_F(GlobalLocalPlanner, ExplicitProgressReachesTheEndOfAClosedRoute)
{
  nav_msgs::msg::Path route; route.header.frame_id="map";
  for (const auto xy : std::vector<std::pair<double,double>>{{.6,.6},{1.6,.6},{1.6,1.6},{.6,1.6},{.6,.6}}) {
    auto q=pose(xy.first,xy.second); route.poses.push_back(q);
  }
  const double total=craft_planning::path_length(route);
  const auto end=planner.local_path(route,odom(.6,.65),{},nullptr,total-.05);
  ASSERT_TRUE(end.ok)<<end.error;
  EXPECT_LT(length(end.path),config.local_behind+.15);
  EXPECT_NEAR(end.path.poses.back().pose.position.x,.6,1e-9);
  EXPECT_NEAR(end.path.poses.back().pose.position.y,.6,1e-9);
}
