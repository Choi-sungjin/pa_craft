#include "craft_planning/path_planner.hpp"

#include <gtest/gtest.h>
#include <png.h>

#include <array>
#include <cmath>
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
  EXPECT_FALSE(planner.plan(odom(0.2, 0.5), pose(1.5, 1.5)).ok);
  EXPECT_FALSE(planner.plan(odom(-0.1, 0.5), pose(1.5, 1.5)).ok);
  EXPECT_FALSE(planner.plan(odom(0.5, 0.5), pose(1.95, 1.5)).ok);
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
  const auto result = planner.plan(odom(0.35, 0.35), pose(1.65, 0.35));
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
}  // namespace
