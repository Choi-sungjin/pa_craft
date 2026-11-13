#include "craft_perception/yolo_detector.hpp"
#include "craft_perception/object_detector.hpp"
#include <random>
#include "craft_perception/puck_axis.hpp"
#include "craft_perception/puck_tag_axis.hpp"
#include "craft_perception/d435_perception.hpp"
#include "craft_perception/apriltag_reader.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/imu_processor.hpp"
#include "craft_perception/localization_monitor.hpp"
#include "craft_perception/scan_relocalizer.hpp"
#include "craft_perception/scan_odometry.hpp"
#include "craft_perception/obstacle_detector.hpp"
#include "craft_perception/lidar_obstacle_detector.hpp"
#include "craft_perception/lidar_obstacle_filter.hpp"
#include "craft_perception/lidar_perception.hpp"
#include "craft_perception/traffic_light.hpp"
#include "craft_perception/barcode_reader.hpp"
#ifdef CRAFT_WITH_ZXING
#include <ZXing/BitMatrix.h>
#include <ZXing/MultiFormatWriter.h>
#endif
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <limits>

using namespace craft_perception;

TEST(ModuleContract, RejectsInvalidConfigurationWithoutThrowing) {
  AprilTagReader apriltag;
  EXPECT_FALSE(apriltag.configure("DICT_4X4_50"));
  LidarObstacleDetector obstacle;
  EXPECT_FALSE(obstacle.configure(3.0, 0));
  YoloDetector detector;
  DetectorConfig config;
  config.model_dir = "/does-not-exist";
  config.confidence = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(detector.configure(config));
  EXPECT_FALSE(ObjectDetector().configure(0));
  EXPECT_FALSE(ObstacleDetector().configure({}, 1.0, 0.1));
  EXPECT_FALSE(ObstacleDetector().configure({1}, 1.0, 1.5));
  PuckAxisConfig puck;
  puck.min_height_m = puck.max_height_m;
  EXPECT_FALSE(PuckAxis().configure(puck));
  D435Config d435;  // nonpositive puck tag box scale, nothing to load
  d435.use_yolo = d435.use_apriltag = d435.use_puck = false;
  d435.obstacle_class_ids = {1};
  d435.puck_tag_box_scale = 0;
  EXPECT_FALSE(D435Perception().configure(d435));
  d435.puck_tag_box_scale = 1;
  EXPECT_TRUE(D435Perception().configure(d435));
}

TEST(ModuleContract, SeparatesInvalidInputsFromValidEmptyObservations) {
  cv::Mat gray(10, 10, CV_8UC1), bgr(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));
  YoloDetector detector;
  AprilTagReader apriltag;
  EXPECT_FALSE(detector.is_valid_input(gray));
  EXPECT_FALSE(apriltag.is_valid_input(gray));
  EXPECT_TRUE(detector.is_valid_input(bgr));
  EXPECT_TRUE(apriltag.is_valid_input(bgr));
  sensor_msgs::msg::LaserScan scan;
  LidarObstacleDetector obstacle;
  ASSERT_TRUE(obstacle.configure(3.0, 1));
  EXPECT_FALSE(obstacle.is_valid_input(scan));
  scan.header.frame_id = "laser";
  ASSERT_TRUE(obstacle.is_valid_input(scan));
  EXPECT_TRUE(obstacle.compute_result(scan).poses.empty());
}

TEST(Yolo, LetterboxRestorationAndClassAwareNms) {
  auto transform = letterbox(cv::Mat(360, 640, CV_8UC3), 640);
  ASSERT_EQ(transform.top, 140);
  ASSERT_EQ(transform.left, 0);
  ncnn::Mat output(3, 9); output.fill(0.f);
  for (int i = 0; i < 3; ++i) {
    output.row(0)[i] = 100; output.row(1)[i] = 240;
    output.row(2)[i] = 40; output.row(3)[i] = 20;
  }
  output.row(4)[0] = 0.9f;
  output.row(4)[1] = 0.8f;
  output.row(5)[2] = 0.85f;
  const auto result = decode_yolo(output, transform, 5, 0.5, 0.45, 100);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].class_id, 0);
  EXPECT_EQ(result[1].class_id, 1);
  EXPECT_FLOAT_EQ(result[0].box.x, 80);
  EXPECT_FLOAT_EQ(result[0].box.y, 90);
}

TEST(Yolo, RejectsWrongLayoutAndNonFiniteBoxes) {
  auto transform = letterbox(cv::Mat(360, 640, CV_8UC3), 640);
  EXPECT_THROW(decode_yolo(ncnn::Mat(6, 300), transform, 5, 0.5, 0.45, 100), std::runtime_error);
  ncnn::Mat output(1, 9); output.fill(0.f);
  output.row(4)[0] = 0.9;
  output.row(0)[0] = std::numeric_limits<float>::quiet_NaN();
  output.row(2)[0] = output.row(3)[0] = 20;
  EXPECT_TRUE(decode_yolo(output, transform, 5, 0.5, 0.45, 100).empty());
}

TEST(AprilTag, SortsIdsByImagePosition) {
  cv::Mat image(220, 420, CV_8UC1, cv::Scalar(255)), left, right, bgr;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(dictionary, 17, 120, left);
  cv::aruco::generateImageMarker(dictionary, 3, 120, right);
#else
  cv::aruco::drawMarker(dictionary, 17, 120, left);
  cv::aruco::drawMarker(dictionary, 3, 120, right);
#endif
  left.copyTo(image(cv::Rect(30, 50, 120, 120)));
  right.copyTo(image(cv::Rect(260, 50, 120, 120)));
  cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  AprilTagReader module;
  ASSERT_TRUE(module.configure("tag36h11"));
  ASSERT_TRUE(module.is_valid_input(bgr));
  const auto result = module.compute_result(bgr);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, 17);
  EXPECT_EQ(result[1].id, 3);
  EXPECT_LT(result[0].center.x, result[1].center.x);
  EXPECT_NEAR(result[0].box.width, 120, 2);
}

TEST(SharedMemory, OwnsSnapshotRejectsOverwrittenSlotAndBusyWriter) {
  const std::string name = "/craft_perception_test_" + std::to_string(getpid());
  int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  struct Cleanup { int fd; std::string name; ~Cleanup() {close(fd); shm_unlink(name.c_str());} } cleanup{fd, name};
  const size_t payload = 2 * 2 * 5, size = payload + sizeof(uint64_t);
  ASSERT_EQ(ftruncate(fd, size * 4), 0);
  std::vector<uint8_t> bytes(size, 0);
  std::fill(bytes.begin(), bytes.begin() + 12, 42);
  uint16_t depth[4] = {250, 250, 250, 250};
  std::memcpy(bytes.data() + 12, depth, sizeof(depth));
  uint64_t sequence = 7;
  std::memcpy(bytes.data() + payload, &sequence, sizeof(sequence));
  ASSERT_EQ(pwrite(fd, bytes.data(), size, 0), static_cast<ssize_t>(size));
  SharedFrameReader reader(name, true);
  auto frame = reader.snapshot({0, 2, 2, 7, 100, 1000000});
  ASSERT_TRUE(frame);
  EXPECT_FLOAT_EQ(frame->depth.at(0, 0), 0.25);
  EXPECT_EQ(frame->bgr.at<cv::Vec3b>(0, 0)[0], 42);
  EXPECT_FALSE(reader.snapshot({0, 2, 2, 6, 100, 1000000}));
  ASSERT_EQ(flock(fd, LOCK_EX | LOCK_NB), 0);
  EXPECT_FALSE(reader.snapshot({0, 2, 2, 8, 101, 1000000}));
  flock(fd, LOCK_UN);
  EXPECT_EQ(reader.snapshot({0, 2, 2, 7, 100, 1000000}).get(), frame.get());
  EXPECT_EQ(reader.snapshots(), 1u);
  EXPECT_EQ(reader.bytes_copied(), payload);
  EXPECT_EQ(reader.cache_hits(), 1u);
  EXPECT_EQ(frame->depth.raw.type(), CV_16UC1);
  bytes[0] = 99; pwrite(fd, bytes.data(), size, 0);
  EXPECT_EQ(frame->bgr.at<cv::Vec3b>(0, 0)[0], 42);
  EXPECT_FALSE(reader.snapshot({0, 2, 2})); // old wire layout
}

TEST(Feedback, KeepsFramesAndRejectsInvalidOdom) {
  nav_msgs::msg::Odometry input;
  EXPECT_FALSE(StateEstimator().is_valid_input(input));
  input.header.frame_id = "odom"; input.pose.pose.orientation.w = 1;
  ASSERT_TRUE(StateEstimator().is_valid_input(input));
  EXPECT_EQ(StateEstimator().compute_result(input).header.frame_id, "odom");
  input.pose.pose.position.x = 1.0;
  input.pose.pose.orientation.z = std::sin(CV_PI / 4); input.pose.pose.orientation.w = std::cos(CV_PI / 4);
  geometry_msgs::msg::Transform map_from_odom;  // +2 m in y, then yaw +90 deg
  map_from_odom.translation.y = 2.0;
  map_from_odom.rotation.z = std::sin(CV_PI / 4); map_from_odom.rotation.w = std::cos(CV_PI / 4);
  const auto in_map = StateEstimator().compute_result(input, map_from_odom, "map");
  EXPECT_EQ(in_map.header.frame_id, "map");
  EXPECT_NEAR(in_map.pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(in_map.pose.pose.position.y, 3.0, 1e-9);
  EXPECT_NEAR(in_map.pose.pose.orientation.z, 1.0, 1e-9);  // yaw 180 deg
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser"; scan.angle_min = 0; scan.angle_increment = CV_PI / 2;
  scan.range_min = 0.1; scan.range_max = 10;
  scan.ranges = {1, 2, std::numeric_limits<float>::infinity(), 9};
  LidarObstacleDetector module;
  ASSERT_TRUE(module.configure(3, 1));
  ASSERT_TRUE(module.is_valid_input(scan));
  const auto points = module.compute_result(scan);
  ASSERT_EQ(points.poses.size(), 2u);
  EXPECT_EQ(points.header.frame_id, "laser");
  EXPECT_NEAR(points.poses[0].position.x, 1, 1e-6);
  EXPECT_NEAR(points.poses[1].position.y, 2, 1e-6);
}

TEST(LidarPerception, MergesNearestBinsAndRejectsStaleRear) {
  LidarPerception module;
  lidar_processing::MergerConfig config;
  ASSERT_TRUE(module.configure(config, 3.0, 1));
  sensor_msgs::msg::LaserScan front;
  front.header.frame_id = "laser_front";
  front.header.stamp.sec = 10;
  front.angle_increment = 1e-6;
  front.range_min = 0.1;
  front.range_max = 8;
  front.ranges = {2, 1, std::numeric_limits<float>::quiet_NaN()};
  auto rear = front;
  rear.header.frame_id = "laser_rear";
  rear.ranges = {1};
  const lidar_processing::Pose2D fp{0.15, 0, 0}, rp{-0.15, 0, CV_PI};
  auto merged = module.merge(front, fp, &rear, &rp);
  ASSERT_TRUE(merged);
  EXPECT_EQ(merged->ranges.size(), 720u);
  EXPECT_EQ(merged->header.frame_id, "base_link");
  EXPECT_EQ(merged->header.stamp, front.header.stamp);
  const auto points = module.detect_obstacles(*merged);
  ASSERT_TRUE(points);
  ASSERT_EQ(points->poses.size(), 2u);
  for (const auto &pose : points->poses) {
    EXPECT_NEAR(std::hypot(pose.position.x, pose.position.y), 1.15, 1e-5);
  }
  rear.header.stamp.sec = 9;
  merged = module.merge(front, fp, &rear, &rp);
  ASSERT_TRUE(merged);
  EXPECT_EQ(module.detect_obstacles(*merged)->poses.size(), 1u);
  front.header.frame_id.clear();
  EXPECT_FALSE(module.merge(front, fp, nullptr, nullptr));
}

TEST(LidarPerception, UpsideDownLidarMirrorsItsScanAngles) {
  // Roll 180 deg (q = x 1): flipped, still facing forward. Flipped about the left-right axis instead
  // (Ry 180, q = y 1): flipped and facing backward, i.e. roll 180 + yaw 180.
  const auto rolled = lidar_processing::pose_from_transform(0.15, 0.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(rolled.flipped);
  EXPECT_NEAR(rolled.yaw, 0.0, 1e-12);
  const auto pitched = lidar_processing::pose_from_transform(0.15, 0.0, 0.0, 1.0, 0.0, 0.0);
  EXPECT_TRUE(pitched.flipped);
  EXPECT_NEAR(std::fabs(pitched.yaw), CV_PI, 1e-12);
  EXPECT_FALSE(lidar_processing::pose_from_transform(0.15, 0.0, 0.0, 0.0, 1.0, 0.0).flipped);  // yaw 180 only
  // A return at scan angle +60 deg, 1 m: left of an upright LiDAR, right of an upside-down one.
  LidarPerception module;
  ASSERT_TRUE(module.configure(lidar_processing::MergerConfig{}, 3.0, 1));
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_front";
  scan.angle_min = CV_PI / 3;
  scan.angle_increment = 1e-6;
  scan.range_min = 0.1;
  scan.range_max = 8;
  scan.ranges = {1.0};
  for (const bool flipped : {false, true}) {
    const auto merged = module.merge(scan, {0.15, 0.0, 0.0, flipped}, nullptr, nullptr);
    ASSERT_TRUE(merged);
    const auto points = module.detect_obstacles(*merged);
    ASSERT_TRUE(points);
    ASSERT_EQ(points->poses.size(), 1u);
    EXPECT_NEAR(points->poses[0].position.x, 0.65, 0.01);
    EXPECT_NEAR(points->poses[0].position.y, flipped ? -0.866 : 0.866, 0.01);
  }
}

TEST(LidarPerception, FiltersBodyAndNearPointsOutsideFovButKeepsFarPoints) {
  LidarPerception module;
  lidar_processing::MergerConfig config;
  config.body_radius_m = 0.05;
  ASSERT_TRUE(module.configure(config, 3.0, 1));
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_front";
  scan.angle_min = CV_PI;
  scan.angle_increment = 1e-6;
  scan.range_min = 0.1;
  scan.range_max = 8;
  scan.ranges = {0.15, 0.35, 1.0};
  const auto merged = module.merge(scan, {0.15, 0, 0}, nullptr, nullptr);
  ASSERT_TRUE(merged);
  const auto points = module.detect_obstacles(*merged);
  ASSERT_TRUE(points);
  ASSERT_EQ(points->poses.size(), 1u);
  EXPECT_NEAR(points->poses[0].position.x, -0.85, 1e-4);
}

TEST(LidarPerception, BodyFilterFollowsTheBodyCenterNotBaseLink) {
  // base_link on the turning center, body (LiDAR midpoint) 26 mm ahead: a point 0.26 m ahead of
  // base_link is only 0.234 m from the body center, inside the 0.25 m body radius.
  LidarPerception module;
  lidar_processing::MergerConfig config;
  config.body_center_x = 0.026;
  ASSERT_TRUE(module.configure(config, 3.0, 1));
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_front";
  scan.angle_increment = 1e-6;
  scan.range_min = 0.05;
  scan.range_max = 8;
  const auto count = [&](float range) {
    scan.ranges = {range};
    const auto merged = module.merge(scan, {0.1697, 0, 0}, nullptr, nullptr);
    return merged ? module.detect_obstacles(*merged)->poses.size() : 99u;
  };
  EXPECT_EQ(count(0.26f - 0.1697f), 0u);
  EXPECT_EQ(count(0.28f - 0.1697f), 1u);
  config.body_center_x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(module.configure(config, 3.0, 1));
}

TEST(LidarPerception, RejectsInvalidMergerConfiguration) {
  LidarPerception module;
  lidar_processing::MergerConfig config;
  config.bins = 0;
  EXPECT_FALSE(module.configure(config, 3.0, 1));
  config.bins = 720;
  config.front_fov_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(module.configure(config, 3.0, 1));
  config.front_fov_deg = 200;
  config.max_age = -1;
  EXPECT_FALSE(module.configure(config, 3.0, 1));
}

TEST(SharedMemory, HandlesOddPixelsResizeAndProducerReplacement) {
  const std::string name = "/craft_perception_resize_" + std::to_string(getpid());
  int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  struct Cleanup { int &fd; std::string name; ~Cleanup() {close(fd); shm_unlink(name.c_str());} } cleanup{fd, name};
  const auto write_frame = [&fd](int width, uint64_t sequence, uint16_t raw) {
    std::vector<uint8_t> data(width * 5 + sizeof(uint64_t), 42);
    for (int x = 0; x < width; ++x) std::memcpy(data.data() + width * 3 + x * 2, &raw, sizeof(raw));
    std::memcpy(data.data() + width * 5, &sequence, sizeof(sequence));
    EXPECT_EQ(flock(fd, LOCK_EX), 0);
    EXPECT_EQ(ftruncate(fd, data.size() * 4), 0);
    EXPECT_EQ(pwrite(fd, data.data(), data.size(), 0), static_cast<ssize_t>(data.size()));
    flock(fd, LOCK_UN);
  };
  DepthImage retained;
  {
    SharedFrameReader reader(name, true);
    write_frame(1, 1, 250);
    auto odd = reader.snapshot({0, 1, 1, 1, 100, 1000000});
    ASSERT_TRUE(odd);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(odd->depth.raw.data) % alignof(uint16_t), 0u);
    EXPECT_DOUBLE_EQ(odd->depth.at(0, 0), 0.25);
    retained = odd->depth;
    write_frame(20, 2, 500);
    auto resized = reader.snapshot({0, 20, 1, 2, 101, 1000000});
    ASSERT_TRUE(resized);
    EXPECT_DOUBLE_EQ(resized->depth.at(0, 19), 0.5);
    EXPECT_DOUBLE_EQ(odd->depth.at(0, 0), 0.25);
    ASSERT_EQ(shm_unlink(name.c_str()), 0);
    close(fd);
    fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    ASSERT_GE(fd, 0);
    write_frame(2, 1, 750);
    auto restarted = reader.snapshot({0, 2, 1, 1, 102, 1000000});
    ASSERT_TRUE(restarted);
    EXPECT_DOUBLE_EQ(restarted->depth.at(0, 1), 0.75);
    EXPECT_EQ(reader.snapshots(), 3u);
  }
  // A retained depth view owns its snapshot even after the reader was destroyed.
  EXPECT_DOUBLE_EQ(retained.at(0, 0), 0.25);
}

TEST(Objects, BoxCenterMedianAndNearestObstacleSurface) {
  cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(2.0));
  depth(cv::Rect(40, 40, 20, 20)).setTo(1.0);  // object face
  depth(cv::Rect(30, 30, 4, 4)).setTo(0.5);    // near edge inside the box
  const Intrinsics in{100, 100, 50, 50};
  const std::vector<Detection> boxes{{{30, 30, 40, 40}, 2, 0.9f, {}}, {{0, 0, 10, 10}, 0, 0.8f, {}}};
  ObjectDetector objects;
  ASSERT_TRUE(objects.configure(0.5));
  ASSERT_TRUE(objects.is_valid_input({depth, 1.0, {}}, in));
  const auto points = objects.compute_result({depth, 1.0, {}}, boxes, in);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].position[2], 1.0, 1e-6);
  EXPECT_NEAR(points[0].position[0], 0.0, 1e-6);
  EXPECT_EQ(points[0].detection.class_id, 2);
  ObstacleDetector obstacles;
  ASSERT_TRUE(obstacles.configure({2}, 1.5, 0.0));
  const auto near = obstacles.compute_result({depth, 1.0, {}}, boxes, in);
  ASSERT_EQ(near.size(), 1u);  // class 0 is not an obstacle class
  EXPECT_NEAR(near[0].position[2], 0.5, 1e-6);
  ASSERT_TRUE(obstacles.configure({2}, 0.4, 0.0));
  EXPECT_TRUE(obstacles.compute_result({depth, 1.0, {}}, boxes, in).empty());
  EXPECT_FALSE(objects.is_valid_input({depth, 1.0, {}}, {0, 100, 50, 50}));
}

namespace {
// Shelf at z=0.5 facing the camera, puck block 2 cm (lying) or 6 cm (standing) closer.
cv::Mat shelf_with_puck(float height, cv::Rect puck) {
  cv::Mat depth(120, 160, CV_32FC1, cv::Scalar(0.5));
  depth(puck).setTo(0.5 - height);
  return depth;
}
}

TEST(Puck, LyingPuckAxisFollowsLongSideOnShelf) {
  const Intrinsics in{200, 200, 80, 60};
  const auto depth = shelf_with_puck(0.02f, {60, 52, 40, 16});  // long side along image x
  PuckAxis module;
  ASSERT_TRUE(module.configure({}));
  const cv::Rect2f box(58, 50, 44, 20);
  ASSERT_TRUE(module.is_valid_input({depth, 1.0, {}}, box, in));
  const auto result = module.compute_result({depth, 1.0, {}}, box, in);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->standing);
  EXPECT_NEAR(result->normal[2], -1, 1e-6);
  EXPECT_NEAR(result->height_m, 0.02, 1e-3);
  EXPECT_GT(result->axis[0], 0.99);
  EXPECT_NEAR(result->center[2], 0.49, 1e-3);
}

TEST(Puck, StandingPuckUsesShelfNormalAndEmptyRingFails) {
  const Intrinsics in{200, 200, 80, 60};
  const auto depth = shelf_with_puck(0.06f, {70, 50, 20, 20});
  PuckAxis module;
  ASSERT_TRUE(module.configure({}));
  const auto result = module.compute_result({depth, 1.0, {}}, {68, 48, 24, 24}, in);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->standing);
  EXPECT_NEAR(result->axis[2], -1, 1e-6);
  cv::Mat empty(120, 160, CV_32FC1, cv::Scalar(0));
  EXPECT_FALSE(module.compute_result({empty, 1.0, {}}, {68, 48, 24, 24}, in));
}

namespace {
// Optical frame of the top D435 (43.6 deg down): world up, horizontal forward, right.
const cv::Vec3d kUp = cv::normalize(cv::Vec3d(0.0, -0.7234, -0.6904));
const cv::Vec3d kRight(1, 0, 0);
const cv::Vec3d kForward = kUp.cross(kRight);
const Intrinsics kTagCamera{615, 615, 320, 240};

// One 16 mm tag on a face: normal toward the camera, tag up (y) along `up_in_face`.
PuckTagAxis::Corners project_tag(const cv::Vec3d &center, const cv::Vec3d &normal, const cv::Vec3d &up_in_face) {
  const cv::Vec3d y = cv::normalize(up_in_face), x = y.cross(normal);
  const double h = 0.008;
  const double offsets[4][2] = {{-h, h}, {h, h}, {h, -h}, {-h, -h}};
  PuckTagAxis::Corners corners;
  for (int k = 0; k < 4; ++k) {
    const cv::Vec3d p = center + x * offsets[k][0] + y * offsets[k][1];
    corners[k] = {float(kTagCamera.fx * p[0] / p[2] + kTagCamera.cx), float(kTagCamera.fy * p[1] / p[2] + kTagCamera.cy)};
  }
  return corners;
}

// Visible faces of a 30x30x60 cuboid with long axis `axis`, the cross section's sides along `side`.
// turns: per visible tag, how many 90-degree turns it is stuck on with (tags go on in any orientation).
std::vector<PuckTagAxis::Corners> cuboid_tags(const cv::Vec3d &center, cv::Vec3d axis, cv::Vec3d side,
  const std::vector<int> &turns = {}) {
  axis = cv::normalize(axis);
  side = cv::normalize(side);
  const cv::Vec3d other = axis.cross(side);
  std::vector<PuckTagAxis::Corners> tags;
  for (const double sign : {1.0, -1.0}) {
    const auto end = axis * sign;
    const auto turned = [&](const cv::Vec3d &n, const cv::Vec3d &up) {  // tag up turned k * 90 deg in the face
      const int k = tags.size() < turns.size() ? turns[tags.size()] : 0;
      cv::Vec3d u = up;
      for (int i = 0; i < k; ++i) u = n.cross(u);
      return u;
    };
    if ((center + end * 0.03).dot(end) < 0) tags.push_back(project_tag(center + end * 0.03, end, turned(end, side)));
    for (const auto &n : {side * sign, other * sign}) {
      if ((center + n * 0.015).dot(n) < 0) tags.push_back(project_tag(center + n * 0.015, n, turned(n, axis)));
    }
  }
  return tags;
}

// Bounding box [px] of the projected 30x30x60 cuboid: what a tight YOLO box around the puck would be.
cv::Rect2f cuboid_box(const cv::Vec3d &center, cv::Vec3d axis, cv::Vec3d side) {
  axis = cv::normalize(axis);
  side = cv::normalize(side - axis * side.dot(axis));
  const cv::Vec3d other = axis.cross(side);
  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  for (const double a : {-0.03, 0.03}) for (const double b : {-0.015, 0.015}) for (const double c : {-0.015, 0.015}) {
    const cv::Vec3d p = center + axis * a + side * b + other * c;
    const float u = float(kTagCamera.fx * p[0] / p[2] + kTagCamera.cx), v = float(kTagCamera.fy * p[1] / p[2] + kTagCamera.cy);
    x0 = std::min(x0, u); y0 = std::min(y0, v); x1 = std::max(x1, u); y1 = std::max(y1, v);
  }
  return {x0, y0, x1 - x0, y1 - y0};
}

double line_angle_deg(const cv::Vec3d &a, const cv::Vec3d &b) {
  return std::acos(std::min(1.0, std::abs(cv::normalize(a).dot(cv::normalize(b))))) * 180 / M_PI;
}
}  // namespace

TEST(PuckTag, StandingCuboidFromTopAndSideTags) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const cv::Vec3d center = cv::Vec3d(0.02, 0.0, 0.0) + kForward * 0.35 - kUp * 0.1;
  const auto tags = cuboid_tags(center, kUp, cv::normalize(kRight + kForward * 0.3));
  ASSERT_GE(tags.size(), 2u);
  const auto result = module.compute_result(tags, kTagCamera);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->standing);
  EXPECT_LT(line_angle_deg(result->axis, kUp), 1.0);
  EXPECT_GT(result->axis.dot(kUp), 0);
  EXPECT_LT(cv::norm(result->center - center), 0.002);
  EXPECT_EQ(result->tags, static_cast<int>(tags.size()));
}

TEST(PuckTag, LyingCuboidYawedFromThreeTags) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const double yaw = 30 * M_PI / 180;
  const cv::Vec3d axis = kRight * std::cos(yaw) + kForward * std::sin(yaw);
  const cv::Vec3d center = kForward * 0.35 - kUp * 0.1;
  const auto tags = cuboid_tags(center, axis, kUp);
  ASSERT_EQ(tags.size(), 3u);
  const auto result = module.compute_result(tags, kTagCamera);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->standing);
  EXPECT_LT(line_angle_deg(result->axis, axis), 1.0);
  EXPECT_GT(result->axis[0], 0);
  EXPECT_LT(cv::norm(result->center - center), 0.002);
  EXPECT_NEAR(result->height_m, 0.03, 1e-9);
}

TEST(PuckTag, TagsStuckOnInAnyOrientation) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const double yaw = 30 * M_PI / 180;
  const cv::Vec3d lying = kRight * std::cos(yaw) + kForward * std::sin(yaw);
  const cv::Vec3d center = kForward * 0.35 - kUp * 0.1;
  // Every mix of 0/90/180/270-degree turns on the three visible faces gives the same axis.
  for (int a = 0; a < 4; ++a) for (int b = 0; b < 4; ++b) for (int c = 0; c < 4; ++c) {
    const auto tags = cuboid_tags(center, lying, kUp, {a, b, c});
    ASSERT_EQ(tags.size(), 3u);
    const auto result = module.compute_result(tags, kTagCamera);
    ASSERT_TRUE(result) << a << b << c;
    EXPECT_FALSE(result->standing);
    EXPECT_LT(line_angle_deg(result->axis, lying), 1.0) << a << b << c;
  }
  // Standing, top + side tags, side tag turned 90 deg (long axis along its x).
  const cv::Vec3d stand = cv::Vec3d(0.02, 0.0, 0.0) + kForward * 0.35 - kUp * 0.1;
  const auto result = module.compute_result(cuboid_tags(stand, kUp, cv::normalize(kRight + kForward * 0.3), {1, 1, 3}),
    kTagCamera);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->standing);
  EXPECT_LT(line_angle_deg(result->axis, kUp), 1.0);
}

TEST(PuckTag, SingleTagNeedsCylinderShape) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  // Lying cylinder, end face toward the camera.
  const cv::Vec3d axis = cv::normalize(kForward + kRight * 0.4);
  const cv::Vec3d center = kForward * 0.35 - kUp * 0.1;
  const std::vector<PuckTagAxis::Corners> tags{project_tag(center - axis * 0.03, -axis, kUp)};
  EXPECT_FALSE(module.compute_result(tags, kTagCamera));  // cuboid end or side face?
  const auto result = module.compute_result(tags, kTagCamera, PuckShape::kCylinder);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->standing);
  EXPECT_LT(line_angle_deg(result->axis, axis), 1.0);
  EXPECT_LT(cv::norm(result->center - center), 0.002);
}

TEST(PuckTag, LyingCuboidSurvivesCornerNoise) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const double yaw = 50 * M_PI / 180;
  const cv::Vec3d axis = kRight * std::cos(yaw) + kForward * std::sin(yaw);
  const cv::Vec3d center = kForward * 0.35 - kUp * 0.1;
  std::mt19937 rng(1);
  std::normal_distribution<float> noise(0.f, 0.3f);  // px, about AprilTag corner refinement
  int solved = 0;
  for (int trial = 0; trial < 50; ++trial) {
    auto tags = cuboid_tags(center, axis, kUp);
    for (auto &tag : tags) for (auto &corner : tag) corner += cv::Point2f(noise(rng), noise(rng));
    const auto result = module.compute_result(tags, kTagCamera);
    if (!result) continue;
    ++solved;
    EXPECT_LT(line_angle_deg(result->axis, axis), 5.0);
    EXPECT_FALSE(result->standing);
  }
  // Tags turned at random: which face is the 30x30 end comes only from the tag positions (PnP of a
  // 16 mm tag), and 0.3 px corner noise makes that close in ~1 of 5 trials -> no answer (never a wrong one).
  // Was 45/50 when side tags had a fixed axis. Depth tag positions (D435) would make this firmer.
  EXPECT_GE(solved, 35);
}

TEST(PuckTag, DisagreeingTagsAndBadConfigFail) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const cv::Vec3d center = kForward * 0.35 - kUp * 0.1;
  // Two tags on faces meeting at 40 deg: no puck has that (faces are 0 or 90 deg apart), whichever
  // face or tag turn each one is taken to be.
  const cv::Vec3d n1 = -kForward, n2 = cv::normalize(-kForward * std::cos(40 * M_PI / 180) + kUp * std::sin(40 * M_PI / 180));
  const std::vector<PuckTagAxis::Corners> tags{
    project_tag(center + n1 * 0.015, n1, kRight),
    project_tag(center + n2 * 0.015 + kRight * 0.02, n2, kRight)};
  EXPECT_FALSE(module.compute_result(tags, kTagCamera));
  PuckTagAxisConfig bad;
  bad.up = {0, 0, 0};
  EXPECT_FALSE(PuckTagAxis().configure(bad));
}

TEST(Seg, MaskFollowsPrototypesThroughLetterbox) {
  // 640x360 image letterboxed into 640 (top pad 140); one prototype plane: +1 left, -1 right.
  const auto transform = letterbox(cv::Mat(360, 640, CV_8UC3, cv::Scalar(0, 0, 0)), 640);
  cv::Mat protos(160, 160, CV_32F, cv::Scalar(-1));
  protos(cv::Rect(0, 0, 80, 160)).setTo(1);
  const float coefficient = 2;
  const auto mask = decode_mask(&coefficient, 1, protos.ptr<float>(), 160 * 160, 1, 160, 160,
                                transform, 640, {300, 100, 40, 20});
  ASSERT_EQ(mask.size(), cv::Size(40, 20));
  EXPECT_EQ(mask.at<uint8_t>(10, 5), 255);   // x=305 < 320 -> positive plane
  EXPECT_EQ(mask.at<uint8_t>(10, 35), 0);    // x=335 > 320
}

TEST(BoardImu, UprightBoardsReportGzMinusBiasWhateverTheirHeading) {
  // Both boards upright; lm_arm turned ~180 deg about the vertical, which leaves gz unchanged.
  for (double heading : {0.0, 3.05}) {
    BoardImu imu;
    ASSERT_TRUE(imu.configure(10, 0.0, 0.05));
    const Vector3 bias{0.004, -0.003, 0.002};
    const Vector3 gravity{0.0, 0.0, 1.0};  // a still accelerometer reads +1 g up
    for (int i = 0; i < 10; ++i) {
      EXPECT_FALSE(imu.yaw_rate());
      imu.update(bias, gravity, true);
    }
    ASSERT_TRUE(imu.calibrated());
    // Chassis turns left at 0.5 rad/s; x and y gyro (here from the heading) do not enter the yaw rate.
    imu.update({bias[0] + 0.01 * std::cos(heading), bias[1] + 0.01 * std::sin(heading), bias[2] + 0.5},
               gravity, false);
    ASSERT_TRUE(imu.yaw_rate());
    EXPECT_NEAR(*imu.yaw_rate(), 0.5, 1e-9);
  }
}

TEST(BoardImu, CalibratesRawGyroWithLargeChipOffset) {
  // The firmware sends the raw gyro: a 0.08 rad/s zero-rate offset is larger than still_rate.
  BoardImu imu;
  ASSERT_TRUE(imu.configure(4, 0.0, 0.05));
  const Vector3 up{0.0, 0.0, 1.0};
  for (double noise : {0.002, -0.001, 0.001, -0.002}) {
    imu.update({0.01, -0.03, 0.08 + noise}, up, true);
  }
  ASSERT_TRUE(imu.calibrated());
  EXPECT_NEAR(imu.bias()[2], 0.08, 1e-12);
  imu.update({0.01, -0.03, 0.08 + 0.5}, up, false);
  EXPECT_NEAR(*imu.yaw_rate(), 0.5, 1e-12);
}

TEST(BoardImu, CalibrationNeedsAnUnbrokenStillWindowAndRejectsMotion) {
  BoardImu imu;
  ASSERT_TRUE(imu.configure(3, 0.0, 0.05));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update({0, 0, 0}, up, true);
  imu.update({0, 0, 0}, up, true);
  imu.update({0, 0, 0}, up, false);   // wheels moved: window restarts
  imu.update({0, 0, 0.2}, up, true);  // turning while the wheels report still (carried): not still
  imu.update({0, 0, 0}, {0, 0, 1.3}, true);  // accelerating: not gravity alone
  imu.update({0, 0, 0}, up, true);
  imu.update({0, 0, 0}, up, true);
  EXPECT_FALSE(imu.calibrated());
  imu.update({0, 0, 0}, up, true);
  EXPECT_TRUE(imu.calibrated());
  imu.reset();
  EXPECT_FALSE(imu.calibrated());
  EXPECT_FALSE(imu.yaw_rate());
}

TEST(BoardImu, TracksBiasOnlyWhileStill) {
  BoardImu imu;
  ASSERT_TRUE(imu.configure(1, 0.1, 0.05));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update({0, 0, 0}, up, true);
  for (int i = 0; i < 200; ++i) {
    imu.update({0, 0, 0.01}, up, true);  // bias drifted to 0.01 rad/s
  }
  EXPECT_NEAR(imu.bias()[2], 0.01, 1e-6);
  EXPECT_NEAR(*imu.yaw_rate(), 0.0, 1e-6);
  for (int i = 0; i < 200; ++i) {
    imu.update({0, 0, 0.3}, up, false);  // turning: bias must not follow
  }
  EXPECT_NEAR(imu.bias()[2], 0.01, 1e-6);
  EXPECT_NEAR(*imu.yaw_rate(), 0.29, 1e-6);
}

TEST(BoardImu, RejectsInvalidConfigurationAndNonFiniteSamples) {
  BoardImu imu;
  EXPECT_FALSE(imu.configure(0, 0.0, 0.05));
  EXPECT_FALSE(imu.configure(10, 1.0, 0.05));
  EXPECT_FALSE(imu.configure(10, -0.1, 0.05));
  EXPECT_FALSE(imu.configure(10, 0.0, 0.0));
  ASSERT_TRUE(imu.configure(1, 0.0, 0.05));
  imu.update({std::nan(""), 0, 0}, {0, 0, 1}, true);
  EXPECT_FALSE(imu.calibrated());
}

// max_yaw_rate 2.5, max_yaw_accel 3.2, jump_noise 0.1: at 20 ms a sample may move 0.164 rad/s.
static ImuProcessorConfig imu_config(int disagreement_samples) {
  return {0, 1, 0.0, 0.05, 1e-4, 0.02, 0.08, 0.1, 2.5, 3.2, 0.1, disagreement_samples, 10};
}

TEST(ImuProcessor, PrimaryUnlessUnavailableAndZeroVelocityUpdate) {
  ImuProcessor imu;
  auto bad = imu_config(1);
  bad.primary = 2;
  EXPECT_FALSE(imu.configure(bad));
  bad = imu_config(0);
  EXPECT_FALSE(imu.configure(bad));
  ASSERT_TRUE(imu.configure(imu_config(1)));
  const Vector3 up{0.0, 0.0, 1.0};
  EXPECT_FALSE(imu.select(0.0, true));                        // nothing calibrated yet
  EXPECT_TRUE(imu.update(0, 0.0, {0, 0, 0.01}, up, true));    // calibrates on its first still sample
  EXPECT_TRUE(imu.update(1, 0.0, {0, 0, -0.02}, up, true));
  imu.update(0, 0.02, {0, 0, 0.03}, up, true);
  imu.update(1, 0.02, {0, 0, -0.02}, up, true);
  EXPECT_EQ(imu.select(0.02, true)->value, 0.0);              // wheels still: 0.02 residual -> 0
  const auto moving = imu.select(0.02, false);
  EXPECT_NEAR(moving->value, 0.02, 1e-12);                    // the primary alone, never a mean
  EXPECT_EQ(moving->board, 0u);
  EXPECT_NEAR(moving->variance, 1e-4, 1e-12);
  imu.update(1, 0.5, {0, 0, 0.08}, up, false);                // primary silent for 0.48 s
  const auto backup = imu.select(0.5, false);
  EXPECT_EQ(backup->board, 1u);
  EXPECT_NEAR(backup->value, 0.1, 1e-12);
  imu.update(0, 0.52, {0, 0, 0.11}, up, false);               // primary back
  EXPECT_EQ(imu.select(0.52, false)->board, 0u);
  EXPECT_FALSE(imu.select(5.0, false));                       // both stale
}

TEST(ImuProcessor, DisagreementIsReportedButNeverSwitchesBoards) {
  // Backup stuck at a constant 0.033 rad/s while the primary reads 0: reported after 3 outputs.
  ImuProcessor imu;
  ASSERT_TRUE(imu.configure(imu_config(3)));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update(0, 0.0, {0, 0, 0}, up, true);
  imu.update(1, 0.0, {0, 0, 0}, up, true);
  for (int i = 1; i <= 5; ++i) {
    const double t = i * 0.02;
    imu.update(0, t, {0, 0, 0}, up, false);
    imu.update(1, t, {0, 0, 0.033}, up, false);
    const auto rate = imu.select(t, false);
    EXPECT_EQ(rate->board, 0u);
    EXPECT_EQ(rate->value, 0.0);
    EXPECT_EQ(rate->disagree, i >= 3);
  }
}

TEST(ImuProcessor, DropsImpossibleRatesAndOneSampleSpikes) {
  ImuProcessor imu;
  ASSERT_TRUE(imu.configure(imu_config(3)));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update(0, 0.0, {0, 0, 0}, up, true);
  imu.update(0, 0.02, {0, 0, 0.05}, up, false);
  EXPECT_NEAR(*imu.accepted(0), 0.05, 1e-12);
  imu.update(0, 0.04, {0, 0, 3.0}, up, false);      // beyond max_yaw_rate
  imu.update(0, 0.06, {0, 0, 0.05}, up, false);
  imu.update(0, 0.08, {0, 0, 1.5}, up, false);      // one-sample spike: +1.45 in 20 ms
  EXPECT_NEAR(*imu.accepted(0), 0.05, 1e-12);
  imu.update(0, 0.10, {0, 0, 0.06}, up, false);     // back: continuous with the last accepted
  EXPECT_NEAR(*imu.accepted(0), 0.06, 1e-12);
  EXPECT_EQ(imu.dropped(0), 2u);
}

TEST(ImuProcessor, AcceptsARealFastChangeOneSampleLate) {
  ImuProcessor imu;
  ASSERT_TRUE(imu.configure(imu_config(3)));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update(0, 0.0, {0, 0, 0}, up, true);
  imu.update(0, 0.02, {0, 0, 0.6}, up, false);      // faster than the Waffle Pi limit: held once
  EXPECT_NEAR(*imu.accepted(0), 0.0, 1e-12);
  imu.update(0, 0.04, {0, 0, 0.6}, up, false);      // repeated: real, accepted
  EXPECT_NEAR(*imu.accepted(0), 0.6, 1e-12);
}

TEST(BoardImu, CalibrationSkipsSpikesButRestartsOnMotion) {
  BoardImu imu;
  ASSERT_TRUE(imu.configure(4, 0.0, 0.05));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update({0, 0, 0.08}, up, true);
  imu.update({0, 0, 2.0}, up, true);   // one spike: skipped
  imu.update({0, 0, 0.08}, up, true);
  imu.update({0, 0, 0.08}, up, true);
  imu.update({0, 0, 0.08}, up, true);
  ASSERT_TRUE(imu.calibrated());
  EXPECT_NEAR(imu.bias()[2], 0.08, 1e-12);  // the spike never entered the mean
  BoardImu turning;
  ASSERT_TRUE(turning.configure(4, 0.0, 0.05));
  turning.update({0, 0, 0.08}, up, true);
  for (int i = 0; i < 3; ++i) turning.update({0, 0, 0.5}, up, true);  // keeps turning: restart
  for (int i = 0; i < 3; ++i) turning.update({0, 0, 0.5}, up, true);
  EXPECT_FALSE(turning.calibrated());
}

TEST(ImuProcessor, BoardsAreComparedAtTheSameTime) {
  // Ramping at 3.2 rad/s^2, lm_arm's newest sample is 20 ms older (0.336 vs wheel_clock's 0.40).
  // Carried along its slope it is 0.40 too: no disagreement, although 0.064 > 0.02 + 8 % of 0.37.
  ImuProcessor imu;
  ASSERT_TRUE(imu.configure(imu_config(1)));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update(0, 0.0, {0, 0, 0}, up, true);
  imu.update(1, 0.0, {0, 0, 0}, up, true);
  imu.update(1, 0.96, {0, 0, 0.272}, up, false);
  imu.update(0, 0.98, {0, 0, 0.336}, up, false);
  imu.update(1, 0.98, {0, 0, 0.336}, up, false);
  imu.update(0, 1.0, {0, 0, 0.40}, up, false);
  const auto aligned = imu.select(1.0, false);
  EXPECT_FALSE(aligned->disagree);
  EXPECT_NEAR(aligned->value, 0.40, 1e-9);
}

static LocalizationMonitorConfig monitor_config() {
  return {3.0, 0.3, 3.0, 20.0, 0.005, 0.3, 0.05, 0.5, 2.0, 0.05, 0.2, 2.0, 2};
}

TEST(LocalizationMonitor, RejectionDeadReckoningRelocalizationAndLost) {
  LocalizationMonitor m;
  ASSERT_TRUE(m.configure(monitor_config()));
  const Covariance3 cov{0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};  // 1 cm, 1 cm, 0.57 deg each
  EXPECT_TRUE(m.slam(0.0, {1.0, 1.0, 0.0}, cov, {1.02, 1.0, 0.0}, cov));   // 2 cm: 1.4 sigma
  EXPECT_FALSE(m.slam(0.2, {1.0, 1.0, 0.0}, cov, {1.10, 1.0, 0.0}, cov));  // 10 cm: 7 sigma
  EXPECT_EQ(m.state(), LocalizationState::kDeadReckoning);
  m.moved(0.2);
  EXPECT_EQ(m.state(), LocalizationState::kDeadReckoning);
  m.moved(0.15);  // 0.35 m > 0.3 m
  EXPECT_EQ(m.state(), LocalizationState::kRelocalize);
  m.relocalized(true);
  EXPECT_EQ(m.state(), LocalizationState::kDeadReckoning);
  EXPECT_EQ(m.dead_reckoning(), 0.0);
  m.moved(0.31);
  m.relocalized(false);
  EXPECT_EQ(m.state(), LocalizationState::kLost);
  EXPECT_TRUE(m.slam(9.0, {1.0, 1.0, 0.0}, cov, {1.0, 1.0, 0.0}, cov));    // SLAM and EKF agree again
  EXPECT_EQ(m.state(), LocalizationState::kOk);
  // Both wrong together: they agree, but the scan no longer fits the map twice in a row.
  m.scan_fits(false);
  EXPECT_EQ(m.state(), LocalizationState::kOk);
  m.scan_fits(false);
  EXPECT_EQ(m.state(), LocalizationState::kDeadReckoning);
  EXPECT_FALSE(m.slam(10.0, {1.0, 1.0, 0.0}, cov, {1.0, 1.0, 0.0}, cov));  // agreeing is not enough now
  m.scan_fits(true);
  EXPECT_TRUE(m.slam(11.0, {1.0, 1.0, 0.0}, cov, {1.0, 1.0, 0.0}, cov));
}

TEST(LocalizationMonitor, LearnsImuBiasAndScaleFromSlamYawOnlyWhenStable) {
  LocalizationMonitor m;
  ASSERT_TRUE(m.configure(monitor_config()));
  const Covariance3 cov{0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};
  m.slam(0.0, {0, 0, 0}, cov, {0, 0, 0}, cov);
  // Straight for 20 s while SLAM adds 0.002 rad/s on top of the IMU: learning starts after 3 s.
  double t = 0.0, mo = 0.0;
  for (; t < 20.0; t += 0.02) {
    mo += 0.002 * 0.02;
    m.yaw(t, mo, 0.0, 0.0, 0);
  }
  EXPECT_GT(m.bias(0), 0.0005);
  EXPECT_LT(m.bias(0), 0.002);
  EXPECT_EQ(m.bias(1), 0.0);  // only the board in use
  // A 90 deg turn the IMU under-reports by 3 %: SLAM adds 0.047 rad; scale moves 30 % of the way.
  double odom = 0.0;
  for (int i = 0; i < 157; ++i, t += 0.02) {
    odom += 0.5 * 0.02;
    mo += 0.03 * 0.5 * 0.02;
    m.yaw(t, mo, odom, 0.5, 0);
  }
  for (int i = 0; i < 150; ++i, t += 0.02) m.yaw(t, mo, odom, 0.0, 0);
  EXPECT_NEAR(m.scale(0), 1.009, 0.001);
  // Rejected SLAM: no learning.
  const double before = m.scale(0);
  m.slam(t, {1, 0, 0}, cov, {0, 0, 0}, cov);
  for (int i = 0; i < 157; ++i, t += 0.02) {
    odom += 0.01;
    mo += 0.001;
    m.yaw(t, mo, odom, 0.5, 0);
  }
  for (int i = 0; i < 150; ++i, t += 0.02) m.yaw(t, mo, odom, 0.0, 0);
  EXPECT_EQ(m.scale(0), before);
}

// 4 x 3 m room, 5 cm cells, with a box so the scan has a unique fit.
static std::vector<int8_t> room(int &w, int &h) {
  w = 80;
  h = 60;
  std::vector<int8_t> grid(w * h, 0);
  for (int u = 0; u < w; ++u) grid[u] = grid[(h - 1) * w + u] = 100;
  for (int v = 0; v < h; ++v) grid[v * w] = grid[v * w + w - 1] = 100;
  for (int u = 34; u < 50; ++u) {
    for (int v = 24; v < 36; ++v) grid[v * w + u] = 100;
  }
  return grid;
}

static std::vector<std::pair<double, double>> cast(const std::vector<int8_t> &grid, int w, int h, const Pose2 &pose) {
  std::vector<std::pair<double, double>> points;
  for (int k = 0; k < 360; ++k) {
    const double a = k * M_PI / 180.0;
    for (double r = 0.1; r < 8.0; r += 0.01) {
      const double x = pose.x + r * std::cos(pose.yaw + a), y = pose.y + r * std::sin(pose.yaw + a);
      const int u = static_cast<int>(x / 0.05), v = static_cast<int>(y / 0.05);
      if (u < 0 || v < 0 || u >= w || v >= h) break;
      if (grid[v * w + u] >= 65) {
        points.emplace_back(r * std::cos(a), r * std::sin(a));
        break;
      }
    }
  }
  return points;
}

TEST(ScanRelocalizer, FindsThePoseInAWideWindowOrFails) {
  ScanRelocalizer r;
  const ScanRelocalizerConfig config{0.3, 15.0 * M_PI / 180.0, 0.02, M_PI / 180.0, 0.05, 0.5, 180, 65};
  ASSERT_TRUE(r.configure(config));
  int w = 0, h = 0;
  const auto grid = room(w, h);
  ASSERT_TRUE(r.set_map(w, h, 0.05, 0.0, 0.0, grid));
  const Pose2 truth{1.2, 1.0, 0.1};
  const auto points = cast(grid, w, h, truth);
  const auto found = r.match(points, {truth.x + 0.2, truth.y - 0.15, truth.yaw - 0.14});  // 25 cm, 8 deg off
  ASSERT_TRUE(found);
  EXPECT_NEAR(found->pose.x, truth.x, 0.03);
  EXPECT_NEAR(found->pose.y, truth.y, 0.03);
  EXPECT_NEAR(found->pose.yaw, truth.yaw, 0.02);
  EXPECT_GT(found->score, 0.8);
  EXPECT_GT(*r.score(points, truth), 0.8);
  // 60 cm off along x: walls parallel to x still fit, so the score drops but not to zero.
  EXPECT_LT(*r.score(points, {truth.x + 0.6, truth.y, truth.yaw}), *r.score(points, truth) - 0.2);
  // A scan that fits nowhere (the robot is really elsewhere, out of the window): failure.
  std::vector<std::pair<double, double>> wrong;
  for (int k = 0; k < 360; ++k) wrong.emplace_back(0.4 * std::cos(k * M_PI / 180.0), 0.4 * std::sin(k * M_PI / 180.0));
  EXPECT_FALSE(r.match(wrong, truth));
  EXPECT_FALSE(r.set_map(w, h, 0.05, 0.0, 0.0, std::vector<int8_t>(w * h, 0)));  // no wall
}

TEST(ScanOdometry, TracksMotionThroughKeyframesDespiteASlippingPrediction) {
  int w = 0, h = 0;
  const auto grid = room(w, h);
  ScanOdometry odom;
  const ScanOdometryConfig config{{0.08, 3.0 * M_PI / 180.0, 0.01, 0.5 * M_PI / 180.0, 0.03, 0.4, 360, 50},
                                  0.02, 6.0, 0.3, 15.0 * M_PI / 180.0};
  ASSERT_TRUE(odom.configure(config));
  Pose2 truth{0.6, 0.6, 0.0}, estimate = truth;
  EXPECT_FALSE(odom.update(cast(grid, w, h, truth), {0, 0, 0}));  // first scan: keyframe only
  // 1.2 m forward, a 90 deg left turn, 0.45 m on (stopping short of the box): several keyframes. The prediction carries the true
  // turn (IMU) but the wheels claim 4 cm more travel per scan than happened (slip).
  int steps = 0;
  auto drive = [&](const Pose2 &move, int n) {
    for (int i = 0; i < n; ++i, ++steps) {
      truth = compose(truth, move);
      const auto step = odom.update(cast(grid, w, h, truth), {move.x + 0.04, move.y, move.yaw});
      ASSERT_TRUE(step) << "scan " << steps;
      EXPECT_GT(step->score, 0.6);
      EXPECT_NEAR(step->delta.x, move.x, 0.012);
      estimate = compose(estimate, step->delta);
    }
  };
  drive({0.03, 0.0, 0.0}, 40);
  drive({0.0, 0.0, 3.0 * M_PI / 180.0}, 30);
  drive({0.03, 0.0, 0.0}, 15);
  EXPECT_NEAR(estimate.x, truth.x, 0.03);
  EXPECT_NEAR(estimate.y, truth.y, 0.03);
  EXPECT_NEAR(estimate.yaw, truth.yaw, 0.02);
  // Sideways slide (skid steer slip): seen as lateral motion.
  truth = compose(truth, {0.0, 0.03, 0.0});
  const auto slide = odom.update(cast(grid, w, h, truth), {0, 0, 0});
  ASSERT_TRUE(slide);
  EXPECT_NEAR(slide->delta.y, 0.03, 0.012);
  // A scan from elsewhere: no step, and the next scan restarts from it.
  std::vector<std::pair<double, double>> wrong;
  for (int k = 0; k < 360; ++k) wrong.emplace_back(0.4 * std::cos(k * M_PI / 180.0), 0.4 * std::sin(k * M_PI / 180.0));
  EXPECT_FALSE(odom.update(wrong, {0, 0, 0}));
  EXPECT_FALSE(odom.configure({config.matcher, 0.02, 0.1, 0.3, 0.2}));  // grid smaller than 10 cells
}

TEST(ImuProcessor, FrozenPrimaryHandsOverToTheBackupAndComesBack) {
  ImuProcessor imu;
  ASSERT_TRUE(imu.configure(imu_config(3)));
  const Vector3 up{0.0, 0.0, 1.0};
  imu.update(0, 0.0, {0, 0, 0.01}, up, true);
  imu.update(1, 0.0, {0, 0, -0.02}, up, true);
  double t = 0.0;
  auto step = [&](double primary_z, double backup_z) {
    t += 0.02;
    imu.update(0, t, {0.001, 0.002, primary_z}, up, false);
    imu.update(1, t, {0.001, 0.002, backup_z}, up, false);
    return imu.select(t, false);
  };
  // Live and noisy: never frozen, the primary is used.
  for (int i = 0; i < 20; ++i) EXPECT_EQ(step(0.31 + 0.001 * (i % 3), 0.28 + 0.001 * (i % 2))->board, 0u);
  // The primary repeats its raw sample exactly: frozen after 10, the backup takes over.
  std::optional<YawRate> rate;
  for (int i = 0; i < 9; ++i) rate = step(0.5, 0.28 + 0.001 * (i % 2));
  EXPECT_EQ(rate->board, 0u);
  rate = step(0.5, 0.28);
  EXPECT_TRUE(imu.frozen(0));
  EXPECT_EQ(rate->board, 1u);
  EXPECT_NEAR(rate->value, 0.30, 1e-9);
  // Changing again: back after 10 changing samples.
  for (int i = 0; i < 9; ++i) rate = step(0.31 + 0.001 * (i % 3), 0.28 + 0.001 * (i % 2));
  EXPECT_EQ(rate->board, 1u);
  rate = step(0.315, 0.281);  // differs from the 0.312 before it
  EXPECT_FALSE(imu.frozen(0));
  EXPECT_EQ(rate->board, 0u);
}

namespace {
using P2 = std::vector<std::array<double, 2>>;
// 2 m x 2 m, 5 cm cells, origin (0, 0): a wall along x = 1.0 m (column 20), the rest free.
nav_msgs::msg::OccupancyGrid wall_map() {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = map.info.height = 40;
  map.info.resolution = 0.05;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(40 * 40, 0);
  for (int y = 0; y < 40; ++y) map.data[y * 40 + 20] = 100;
  return map;
}
P2 blob(double x, double y) { return {{x, y}, {x + 0.02, y}, {x, y + 0.02}, {x + 0.02, y + 0.02}}; }
size_t kept(const std::vector<bool> &keep) { return static_cast<size_t>(std::count(keep.begin(), keep.end(), true)); }
}  // namespace

TEST(LidarObstacleFilter, PersistentObjectsPassFlickersAndSinglePointsDoNot) {
  craft_perception::LidarObstacleFilter filter;
  ASSERT_TRUE(filter.configure({}));  // 3 frames, 3 points, no map
  // A 4-point object: dropped while history builds up, kept from the third frame.
  EXPECT_EQ(kept(filter.filter(blob(0.5, 0.5))), 0u);
  EXPECT_EQ(kept(filter.filter(blob(0.5, 0.5))), 0u);
  EXPECT_EQ(kept(filter.filter(blob(0.5, 0.5))), 4u);
  // One noise point next to it in one frame only: dropped, the object stays.
  auto frame = blob(0.5, 0.5); frame.push_back({1.5, 1.5});
  auto keep = filter.filter(frame);
  EXPECT_EQ(kept(keep), 4u);
  EXPECT_FALSE(keep.back());
  // A single persistent point: persists but is a cluster of one.
  for (int i = 0; i < 3; ++i) keep = filter.filter({{1.5, 1.5}});
  EXPECT_EQ(kept(keep), 0u);
  // A slow mover (4 cm per frame) still persists.
  filter.reset();
  for (int i = 0; i < 3; ++i) keep = filter.filter(blob(0.2 + 0.04 * i, 0.2));
  EXPECT_EQ(kept(keep), 4u);
}

TEST(LidarObstacleFilter, MapWallsAndOffMapPointsAreDropped) {
  craft_perception::LidarObstacleFilter filter;
  ASSERT_TRUE(filter.configure({}));
  filter.set_map(wall_map());
  ASSERT_TRUE(filter.has_map());
  EXPECT_NEAR(filter.wall_distance(0.525, 1.0), 0.50, 1e-6);  // cell centre to wall cell centre: 10 cells
  EXPECT_EQ(filter.wall_distance(-0.1, 1.0), 0.0);  // off the map counts as wall
  P2 frame = blob(1.05, 1.0);   // the wall itself, seen a few cm off: within 6 cm
  auto object = blob(0.6, 1.0);  // 0.4 m from the wall
  frame.insert(frame.end(), object.begin(), object.end());
  std::vector<bool> keep;
  for (int i = 0; i < 3; ++i) keep = filter.filter(frame);
  EXPECT_EQ(kept(keep), 4u);
  for (size_t i = 0; i < 4; ++i) EXPECT_FALSE(keep[i]);
  for (size_t i = 4; i < 8; ++i) EXPECT_TRUE(keep[i]);
}

TEST(LidarObstacleFilter, RememberedCellsStayUntilRaysSeeThemFreeOrTheyExpire) {
  craft_perception::LidarObstacleFilter filter;
  craft_perception::LidarObstacleFilterConfig c;
  c.memory_s = 30.0;
  ASSERT_TRUE(filter.configure(c));
  const std::array<double, 2> robot{0.0, 0.0};
  const auto frame = [&](double t, const P2 &points, const P2 &rays) {
    return filter.remember(t, robot, rays, points, filter.filter(points));};
  // Kept in the third frame (persistence), then out of view: no points, no rays through it.
  for (int i = 0; i < 3; ++i) EXPECT_TRUE(frame(0.1 * i, blob(1.0, 0.0), blob(1.0, 0.0)).empty());
  auto remembered = frame(0.3, {}, {});
  ASSERT_FALSE(remembered.empty());
  for (const auto &p : remembered) EXPECT_NEAR(std::hypot(p[0] - 1.01, p[1] - 0.01), 0.0, 0.05);
  // Rays that end on it (the object seen again) do not clear it; a ray beside it does not either.
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(frame(0.4 + 0.1 * i, {}, {{1.0, 0.0}, {1.0, 0.5}}).empty());
  // Rays through it to a wall behind: free in persist_frames frames in a row -> cleared.
  const P2 through{{2.0, 0.0}, {2.0, 0.02}, {2.0, 0.04}};
  EXPECT_FALSE(frame(1.0, {}, through).empty());
  EXPECT_FALSE(frame(1.1, {}, through).empty());
  EXPECT_TRUE(frame(1.2, {}, through).empty());
  // Never seen free again: dropped memory_s after it was last kept.
  filter.reset();
  for (int i = 0; i < 3; ++i) frame(0.1 * i, blob(1.0, 0.0), {});
  EXPECT_FALSE(frame(29.0, {}, {}).empty());
  EXPECT_TRUE(frame(30.3, {}, {}).empty());
  // A new drive clears it.
  filter.reset();
  for (int i = 0; i < 3; ++i) frame(0.1 * i, blob(1.0, 0.0), {});
  ASSERT_FALSE(frame(0.3, {}, {}).empty());
  filter.clear_memory();
  EXPECT_TRUE(frame(0.4, {}, {}).empty());
  // memory_s 0: nothing remembered.
  c.memory_s = 0; ASSERT_TRUE(filter.configure(c));
  for (int i = 0; i < 3; ++i) frame(0.1 * i, blob(1.0, 0.0), {});
  EXPECT_TRUE(frame(0.3, {}, {}).empty());
}

TEST(LidarObstacleFilter, RejectsBadConfiguration) {
  craft_perception::LidarObstacleFilter filter;
  craft_perception::LidarObstacleFilterConfig c;
  c.cell_m = 0; EXPECT_FALSE(filter.configure(c));
  c = {}; c.persist_frames = 0; EXPECT_FALSE(filter.configure(c));
  c = {}; c.min_cluster_points = 0; EXPECT_FALSE(filter.configure(c));
  c = {}; c.wall_margin_m = -1; EXPECT_FALSE(filter.configure(c));
  c = {}; c.memory_s = -1; EXPECT_FALSE(filter.configure(c));
}

#include "lidar_processor/scan_merger.hpp"

TEST(ScanMergerFloorRoi, ClearSectorKeepsAllEdgeBinsKeepNearPointsPerSide) {
  // One LiDAR at x 0.155 facing +x (outward), not flipped; points at the given angle/range from it.
  lidar_processing::MergerConfig c;
  c.front_fov_deg = 180; c.fov_near_m = 10; c.body_radius_m = 0.1; c.body_center_x = 0.01275;
  c.floor_roi.enabled = true;
  c.floor_roi.front_left = {75.0, {0.88, 0.93, 0.67}};
  c.floor_roi.front_right = {65.0, {1.37, 1.40, 0.81, 0.73, 0.53}};
  lidar_processing::ScanMerger merger;
  ASSERT_TRUE(merger.configure(c));
  const auto kept = [&](double deg, double range) {
      sensor_msgs::msg::LaserScan s;
      s.header.frame_id = "laser"; s.range_min = 0.05f; s.range_max = 8.0f;
      s.angle_min = static_cast<float>(deg * M_PI / 180); s.angle_increment = 0.01f;
      s.ranges = {static_cast<float>(range)};
      const auto out = merger.merge(s, {0.155, 0.0, 0.0, false});
      return std::any_of(out->ranges.begin(), out->ranges.end(), [](float r) {return std::isfinite(r);});};
  EXPECT_TRUE(kept(30, 3.0));    // inside the clear sector: any range
  EXPECT_TRUE(kept(74, 3.0));
  EXPECT_TRUE(kept(80, 0.5));    // left 80-85 bin: up to 0.93 m
  EXPECT_FALSE(kept(80, 1.5));
  EXPECT_TRUE(kept(-70, 1.2));   // right 70-75 bin: up to 1.40 m
  EXPECT_FALSE(kept(-70, 1.6));
  EXPECT_TRUE(kept(-60, 3.0));   // right clear sector ends at 65 deg
  EXPECT_FALSE(kept(-89, 0.6));  // right 85-90 bin: up to 0.53 m
  c.floor_roi.front_left.clear_deg = 95;  // out of range
  EXPECT_FALSE(merger.configure(c));
  c.floor_roi.enabled = false;
  ASSERT_TRUE(merger.configure(c));
  EXPECT_TRUE(kept(80, 1.5));    // ROI off: the plain 180 deg field of view
}

// ---- traffic light ----------------------------------------------------------------------------
namespace {
Detection light(int class_id, float score = 0.9f) { return {cv::Rect2f(0, 0, 10, 10), class_id, score, {}}; }
// 30 fps, 1 s on / 1 s off per color in `seq` (0=R 1=Y 2=G). `drop` frames with no detection,
// `flip` frames with the wrong color, a spurious 1-frame blink in every OFF phase.
TrafficLightCounter run_sequence(const std::string &seq, bool noisy) {
  TrafficLightCounter counter;
  EXPECT_TRUE(counter.configure({}));
  double t = 0;
  const double dt = 1.0 / 30;
  for (char c : seq) {
    const int id = c == 'R' ? 0 : c == 'Y' ? 1 : 2;
    for (int i = 0; i < 30; ++i, t += dt) {
      std::vector<Detection> frame{light(id)};
      if (noisy && i % 7 == 3) frame.clear();                    // missed detections
      if (noisy && i % 11 == 5) frame = {light((id + 1) % 3)};   // wrong color
      if (noisy && i % 13 == 6) frame.push_back(light(id, 0.2f)); // low score, ignored
      counter.update(t, frame);
    }
    for (int i = 0; i < 30; ++i, t += dt) {
      std::vector<Detection> frame;
      if (noisy && i == 15) frame = {light(0)};  // false positive while dark
      counter.update(t, frame);
    }
  }
  for (int i = 0; i < 90; ++i, t += dt) counter.update(t, {});  // 3 s dark: end of the sequence
  return counter;
}
}

TEST(TrafficLight, CountsCleanSequence) {
  const auto counter = run_sequence("RRYGGRYYGRRG", false);
  ASSERT_EQ(counter.blinks().size(), 12u);
  const auto c = counter.counts();
  EXPECT_EQ(c.red(), 5);
  EXPECT_EQ(c.yellow(), 3);
  EXPECT_EQ(c.green(), 4);
  EXPECT_TRUE(counter.valid());
  EXPECT_TRUE(counter.complete());
  EXPECT_TRUE(counter.rejected().empty());
}

TEST(TrafficLight, FiltersMissedWrongAndSpuriousDetections) {
  const std::string seq = "GYRRGYGGRYRR";
  const auto counter = run_sequence(seq, true);
  ASSERT_EQ(counter.blinks().size(), 12u);
  for (size_t i = 0; i < seq.size(); ++i) {
    EXPECT_EQ(std::string(to_string(counter.blinks()[i].color)), std::string(1, seq[i])) << i;
  }
  EXPECT_TRUE(counter.valid());
  EXPECT_FALSE(counter.rejected().empty());  // the 1-frame false positives were rejected
}

TEST(TrafficLight, RejectsInvalidConfigAndRuleViolations) {
  TrafficLightCounter counter;
  TrafficLightConfig bad;
  bad.yellow_class_id = bad.red_class_id;
  EXPECT_FALSE(counter.configure(bad));
  bad = {};
  bad.max_gap_s = 0;
  EXPECT_FALSE(counter.configure(bad));
  bad = {};
  bad.end_off_s = bad.max_gap_s;
  EXPECT_FALSE(counter.configure(bad));
}

TEST(TrafficLight, WrongCycleIsDiscardedAndRecounted) {
  // 7 reds breaks the 1..6 rule even though the total is 12: dropped at the end of the cycle.
  auto counter = run_sequence("RRRRRRRYYGGG", false);
  EXPECT_FALSE(counter.complete());
  EXPECT_EQ(counter.discarded_cycles(), 1);
  EXPECT_TRUE(counter.blinks().empty());
  // 11 blinks, then a full cycle on the same counter: the second one is accepted.
  TrafficLightCounter again;
  ASSERT_TRUE(again.configure({}));
  double t = 0;
  const double dt = 1.0 / 30;
  const auto play = [&](const std::string &seq) {
    for (char c : seq) {
      const int id = c == 'R' ? 0 : c == 'Y' ? 1 : 2;
      for (int i = 0; i < 30; ++i, t += dt) again.update(t, {light(id)});
      for (int i = 0; i < 30; ++i, t += dt) again.update(t, {});
    }
    for (int i = 0; i < 90; ++i, t += dt) again.update(t, {});
  };
  play("RRYGGRYYGRR");  // 11
  EXPECT_FALSE(again.complete());
  EXPECT_EQ(again.discarded_cycles(), 1);
  play("RRYGGRYYGRRG");
  EXPECT_TRUE(again.complete());
  EXPECT_EQ(again.counts().total(), 12);
  // Not complete until the dark period: 12 blinks alone are not enough.
  TrafficLightCounter pending;
  ASSERT_TRUE(pending.configure({}));
  t = 0;
  for (char c : std::string("RRYGGRYYGRRG")) {
    const int id = c == 'R' ? 0 : c == 'Y' ? 1 : 2;
    for (int i = 0; i < 30; ++i, t += dt) pending.update(t, {light(id)});
    for (int i = 0; i < 30; ++i, t += dt) pending.update(t, {});
  }
  EXPECT_EQ(pending.blinks().size(), 12u);
  EXPECT_FALSE(pending.complete());
}

// ---- traffic light placement (발제 v5) ---------------------------------------------------------
namespace {
TrafficLightCounts counts_of(const std::string &seq) {
  TrafficLightCounts c;
  for (char ch : seq) ++c.count[ch == 'R' ? 0 : ch == 'Y' ? 1 : 2];
  return c;
}
// spec "RC RB GC ..." : R/G color, C cylinder / B box(cuboid); ids 0.. in order.
std::vector<PuckInfo> pucks_of(const std::string &spec) {
  std::vector<PuckInfo> out;
  for (std::size_t i = 0; i + 1 < spec.size(); i += 3) {
    out.push_back({static_cast<int>(out.size()), spec[i] == 'R' ? PuckColor::kRed : PuckColor::kGreen,
                   spec[i + 1] == 'C' ? PuckShape::kCylinder : PuckShape::kCuboid});
  }
  return out;
}
std::vector<int> sorted(std::vector<int> v) { std::sort(v.begin(), v.end()); return v; }
}

TEST(TrafficLightPuzzle, BarcodeAndDice) {
  EXPECT_EQ(parse_barcode("P2-01"), SortOrder::kAscending);
  EXPECT_EQ(parse_barcode("P2-03"), SortOrder::kAscending);
  EXPECT_EQ(parse_barcode("P2-02"), SortOrder::kDescending);
  EXPECT_EQ(parse_barcode("P2-04"), SortOrder::kDescending);
  EXPECT_FALSE(parse_barcode("P2-05").has_value());
  for (int n = 1; n <= 6; ++n) EXPECT_EQ(dice_dots(n).size(), static_cast<std::size_t>(n));
  const auto six = dice_dots(6);  // left and right columns, no middle column
  for (const auto &d : six) EXPECT_NE(d[0], 0.0);
  const auto two = dice_dots(2);  // top-left / bottom-right diagonal
  EXPECT_DOUBLE_EQ(two[0][0], -0.06); EXPECT_DOUBLE_EQ(two[0][1], 0.06);
  EXPECT_DOUBLE_EQ(two[1][0], 0.06); EXPECT_DOUBLE_EQ(two[1][1], -0.06);
}

TEST(TrafficLightPuzzle, Q1Ascending) {
  // 발제 05 Q1: R G R R G R G G Y G Y G, P2-01 -> R4 Y2 G6 -> L1 Y(2), L2 R(4), L3 G(6).
  const auto counts = counts_of("RGRRGRGGYGYG");
  ASSERT_EQ(counts.red(), 4); ASSERT_EQ(counts.yellow(), 2); ASSERT_EQ(counts.green(), 6);
  // red 6 (cuboid 3, cylinder 3), green 6
  const auto pucks = pucks_of("RB RC RB RC RB RC GB GC GB GC GB GC ");
  std::string error;
  const auto plan = solve_traffic_light(counts, SortOrder::kAscending, pucks, &error);
  ASSERT_TRUE(plan) << error;
  EXPECT_EQ(plan->locations[0].group, LightColor::kYellow);
  EXPECT_EQ(plan->locations[1].group, LightColor::kRed);
  EXPECT_EQ(plan->locations[2].group, LightColor::kGreen);
  EXPECT_EQ(plan->yellow_shape, PuckShape::kCuboid);
  EXPECT_EQ(plan->locations[0].puck_ids, (std::vector<int>{0, 2}));        // 2 red cuboids
  EXPECT_EQ(sorted(plan->locations[1].puck_ids), (std::vector<int>{1, 3, 4, 5}));
  EXPECT_EQ(sorted(plan->locations[2].puck_ids), (std::vector<int>{6, 7, 8, 9, 10, 11}));
}

TEST(TrafficLightPuzzle, SpecExampleAndShapeFallback) {
  // 발제 04 example: red 8, green 4; 3 same-shape reds go to Y -> R5 Y3 G4.
  // Only cylinders have 3 reds: the cuboid-first choice must fall back.
  const auto pucks = pucks_of("RB RB RC RC RC RB RB RB GB GC GB GC ");
  TrafficLightCounts counts; counts.count = {5, 3, 4};
  // reds: cuboid 5 (0,1,5,6,7), cylinder 3 (2,3,4) -> cuboid also has >= 3: cuboid chosen, ids 0,1,5.
  auto plan = solve_traffic_light(counts, SortOrder::kDescending, pucks);
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->yellow_shape, PuckShape::kCuboid);
  // descending: R5, G4, Y3
  EXPECT_EQ(plan->locations[0].group, LightColor::kRed);
  EXPECT_EQ(plan->locations[1].group, LightColor::kGreen);
  EXPECT_EQ(plan->locations[2].group, LightColor::kYellow);
  EXPECT_EQ(plan->locations[2].puck_ids, (std::vector<int>{0, 1, 5}));
  // Now only 2 red cuboids: Y must be cylinders.
  const auto fewer = pucks_of("RB RC RC RC RC RB RC RC GB GC GB GC ");
  plan = solve_traffic_light(counts, SortOrder::kAscending, fewer);
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->yellow_shape, PuckShape::kCylinder);
  EXPECT_EQ(plan->locations[0].group, LightColor::kYellow);  // ascending: Y3, G4, R5
  EXPECT_EQ(plan->locations[0].puck_ids, (std::vector<int>{1, 2, 3}));
}

TEST(TrafficLightPuzzle, TiesKeepRygAndMixedColorYellow) {
  // R4 Y4 G4 with red 6 / green 6: Y = 2 red + 2 green of one shape.
  const auto pucks = pucks_of("RB RB RC RC RC RC GC GB GB GC GC GC ");
  TrafficLightCounts counts; counts.count = {4, 4, 4};
  for (const auto order : {SortOrder::kAscending, SortOrder::kDescending}) {
    std::string error;
    const auto plan = solve_traffic_light(counts, order, pucks, &error);
    ASSERT_TRUE(plan) << error;
    EXPECT_EQ(plan->locations[0].group, LightColor::kRed);
    EXPECT_EQ(plan->locations[1].group, LightColor::kYellow);
    EXPECT_EQ(plan->locations[2].group, LightColor::kGreen);
    EXPECT_EQ(plan->yellow_shape, PuckShape::kCuboid);  // red cuboid 2, green cuboid 2
    EXPECT_EQ(sorted(plan->locations[1].puck_ids), (std::vector<int>{0, 1, 7, 8}));
  }
}

TEST(TrafficLightPuzzle, ReportsImpossibleProblems) {
  TrafficLightCounts counts; counts.count = {4, 4, 4};
  std::string error;
  // Y needs 2 red + 2 green of one shape; red cuboid 1 / green cylinder 1 -> no shape fits.
  EXPECT_FALSE(solve_traffic_light(counts, SortOrder::kAscending,
    pucks_of("RB RC RC RC RC RC GC GB GB GB GB GB "), &error));
  EXPECT_NE(error.find("no shape"), std::string::npos);
  EXPECT_FALSE(solve_traffic_light(counts, SortOrder::kAscending,
    pucks_of("RB RB RB GB GB GB GB GB GB GB GB GB "), &error));  // 3 red < R4
  counts.count = {7, 1, 4};
  EXPECT_FALSE(solve_traffic_light(counts, SortOrder::kAscending,
    pucks_of("RB RB RB RB RB RB RB RB GB GB GB GB "), &error));  // R > 6
}

TEST(TrafficLightPuzzle, SurveyCollectsTwelveThenSolves) {
  PuckSurvey survey;
  EXPECT_FALSE(survey.add({7, 0, PuckColor::kRed, PuckShape::kCuboid, 0.9f, {}}));  // no box 7
  for (int box = 1; box <= 6; ++box) {
    for (int slot = 0; slot < 2; ++slot) {
      if (box == 6 && slot == 1) continue;
      EXPECT_TRUE(survey.add({box, slot, box <= 3 ? PuckColor::kRed : PuckColor::kGreen,
                              slot ? PuckShape::kCylinder : PuckShape::kCuboid, 0.6f, {}}));
    }
  }
  EXPECT_FALSE(survey.complete());
  EXPECT_EQ(survey.missing_ids(), (std::vector<int>{11}));
  EXPECT_FALSE(survey.pucks());
  survey.add({6, 1, PuckColor::kGreen, PuckShape::kCuboid, 0.4f, {}});
  survey.add({6, 1, PuckColor::kGreen, PuckShape::kCylinder, 0.8f, {}});  // better look wins
  ASSERT_TRUE(survey.complete());
  const auto pucks = *survey.pucks();
  EXPECT_EQ(pucks[11].shape, PuckShape::kCylinder);
  TrafficLightCounts counts; counts.count = {4, 4, 4};  // red 6 / green 6, Y = 2+2 one shape
  EXPECT_TRUE(solve_traffic_light(counts, SortOrder::kAscending, pucks));
}

TEST(TrafficLightPuzzle, LoadingAndReverification) {
  PuckSurvey survey;
  survey.reset();
  for (int box = 1; box <= 6; ++box) {
    for (int slot = 0; slot < 2; ++slot) {
      survey.add({box, slot, box <= 3 ? PuckColor::kRed : PuckColor::kGreen,
                  slot ? PuckShape::kCylinder : PuckShape::kCuboid, 0.9f, {}});
    }
  }
  EXPECT_FALSE(survey.all_loaded());
  for (int id = 0; id < 12; ++id) EXPECT_TRUE(survey.load(id, 11 - id));
  EXPECT_FALSE(survey.load(3, 5));            // already loaded
  EXPECT_TRUE(survey.all_loaded());
  EXPECT_EQ(survey.carrier_slot(0), 11);

  TrafficLightCounts counts; counts.count = {4, 4, 4};
  const auto plan = solve_traffic_light(counts, SortOrder::kAscending, *survey.pucks());
  ASSERT_TRUE(plan);
  // Perfect placement, pucks permuted within each location and 5 mm off: passes.
  std::array<std::vector<PlacedPuck>, 3> seen;
  for (int l = 0; l < 3; ++l) {
    const auto &loc = plan->locations[l];
    for (std::size_t k = 0; k < loc.dots.size(); ++k) {
      seen[l].push_back({loc.puck_ids[(k + 1) % loc.puck_ids.size()], {loc.dots[k][0] + 0.005, loc.dots[k][1]}});
    }
  }
  EXPECT_TRUE(verify_placement(*plan, seen).ok());
  // 15 mm off: that dot is empty and the puck is an extra.
  auto shifted = seen;
  shifted[0][0].offset[0] += 0.010;
  auto check = verify_placement(*plan, shifted);
  EXPECT_FALSE(check.ok());
  EXPECT_EQ(check.locations[0].empty_dots, 1);
  EXPECT_EQ(check.locations[0].extra_pucks, 1);
  EXPECT_TRUE(check.locations[1].ok);
  // Two pucks swapped between L1 and L3: wrong group at both, nothing duplicated.
  auto swapped = seen;
  std::swap(swapped[0][0].id, swapped[2][0].id);
  check = verify_placement(*plan, swapped);
  EXPECT_EQ(check.locations[0].wrong_ids.size(), 1u);
  EXPECT_EQ(check.locations[2].missing_ids.size(), 1u);
  EXPECT_TRUE(check.duplicate_ids.empty());
  // A puck never seen.
  auto lost = seen;
  const int gone = lost[1].back().id;
  lost[1].pop_back();
  check = verify_placement(*plan, lost);
  EXPECT_EQ(check.unseen_ids, (std::vector<int>{gone}));
}

TEST(Barcode, ConfirmsAfterThreeReadsAndRejectsUnregistered) {
  BarcodeReader reader;
  EXPECT_FALSE(reader.update({"P2-03"}));
  EXPECT_FALSE(reader.update({"P2-02"}));  // one misread never wins
  EXPECT_FALSE(reader.update({"P2-03"}));
  EXPECT_FALSE(reader.confirmed());
  EXPECT_TRUE(reader.update({"P2-03"}));
  EXPECT_EQ(reader.order(), SortOrder::kAscending);
  EXPECT_FALSE(reader.update({"P2-02"}));  // locked until reset
  reader.reset();
  for (int i = 0; i < 3; ++i) reader.update({"HELLO"});
  EXPECT_TRUE(reader.confirmed());
  EXPECT_FALSE(reader.order().has_value());  // unregistered -> mission error
}

#ifdef CRAFT_WITH_ZXING
TEST(Barcode, DecodesRenderedCode128) {
  ASSERT_TRUE(barcode_decoder_available());
  for (const std::string text : {"P2-01", "P2-04"}) {
    const auto bits = ZXing::MultiFormatWriter(ZXing::BarcodeFormat::Code128).setMargin(10).encode(text, 300, 80);
    cv::Mat symbol(bits.height(), bits.width(), CV_8U);
    for (int y = 0; y < bits.height(); ++y) {
      for (int x = 0; x < bits.width(); ++x) symbol.at<uint8_t>(y, x) = bits.get(x, y) ? 0 : 255;
    }
    cv::Mat scene(360, 640, CV_8UC3, cv::Scalar(120, 130, 140));  // camera-sized frame, symbol pasted in
    cv::Mat colour;
    cv::cvtColor(symbol, colour, cv::COLOR_GRAY2BGR);
    colour.copyTo(scene(cv::Rect(150, 120, colour.cols, colour.rows)));
    const auto texts = decode_code128(scene);
    ASSERT_EQ(texts.size(), 1u) << text;
    EXPECT_EQ(texts[0], text);
  }
  EXPECT_TRUE(decode_code128(cv::Mat(360, 640, CV_8UC3, cv::Scalar(90, 90, 90))).empty());
}
#endif

TEST(TrafficLightPuzzle, AssociatesTagsWithPuckBoxes) {
  const auto box = [](float x, float y, float w, float h, int cls, float score) {
    return Detection{cv::Rect2f(x, y, w, h), cls, score, {}};
  };
  const std::vector<Detection> yolo{
    box(100, 100, 40, 60, 1, 0.9f),   // red prism, tag 3
    box(200, 100, 40, 40, 2, 0.8f),   // green cylinder, tag 7 on its top face
    box(300, 100, 40, 60, 3, 0.9f),   // green prism: two different tags inside -> ambiguous
    box(400, 100, 40, 40, 0, 0.9f),   // red cylinder seen from the side: no tag -> not identified
    box(90, 90, 200, 200, 1, 0.9f),   // large box around the first two: smaller boxes win
    box(500, 100, 40, 60, 1, 0.2f)};  // below min_score
  const std::vector<PuckTag> tags{{3, {120, 120}}, {3, {125, 150}},  // two faces of one cuboid
    {7, {220, 120}}, {5, {310, 110}}, {6, {330, 150}}, {13, {420, 120}},  // 13 = location tag
    {9, {520, 130}}};
  const auto pucks = associate_pucks(yolo, tags);
  ASSERT_EQ(pucks.size(), 2u);
  EXPECT_EQ(pucks[0].id, 3);
  EXPECT_EQ(pucks[0].color, PuckColor::kRed);
  EXPECT_EQ(pucks[0].shape, PuckShape::kCuboid);
  EXPECT_EQ(pucks[1].id, 7);
  EXPECT_EQ(pucks[1].color, PuckColor::kGreen);
  EXPECT_EQ(pucks[1].shape, PuckShape::kCylinder);
  PuckSurvey survey;
  for (const auto &p : pucks) EXPECT_TRUE(survey.add(p.id, p.color, p.shape, p.score));
  EXPECT_EQ(survey.missing_ids().size(), 10u);
}

TEST(AprilTag, SearchesOnlyInsideRoisWithFullImageCoordinates) {
  cv::Mat image(220, 420, CV_8UC1, cv::Scalar(255)), left, right, bgr;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(dictionary, 5, 60, left);
  cv::aruco::generateImageMarker(dictionary, 9, 60, right);
#else
  cv::aruco::drawMarker(dictionary, 5, 60, left);
  cv::aruco::drawMarker(dictionary, 9, 60, right);
#endif
  left.copyTo(image(cv::Rect(40, 60, 60, 60)));
  right.copyTo(image(cv::Rect(300, 80, 60, 60)));
  cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  AprilTagReader module;
  ASSERT_TRUE(module.configure("tag36h11"));
  const auto full = module.compute_result(bgr);
  ASSERT_EQ(full.size(), 2u);
  // A box around the left tag only (grown 1.4x so the white margin is in the crop).
  const auto one = module.compute_result(bgr, {cv::Rect2f(30, 50, 80, 80)}, 1.4, 1);
  ASSERT_EQ(one.size(), 1u);
  EXPECT_EQ(one[0].id, 5);
  for (int k = 0; k < 4; ++k) EXPECT_NEAR(cv::norm(one[0].corners[k] - full[0].corners[k]), 0.0, 0.5);
  // Overlapping boxes see the same tag twice: kept once. Upscaled crops map back to image px.
  const auto twice = module.compute_result(bgr, {cv::Rect2f(30, 50, 80, 80), cv::Rect2f(35, 55, 80, 80)}, 1.4, 2);
  ASSERT_EQ(twice.size(), 1u);
  EXPECT_NEAR(cv::norm(twice[0].center - full[0].center), 0.0, 0.5);
  // No boxes or boxes away from tags: nothing; bad grow/upscale: nothing.
  EXPECT_TRUE(module.compute_result(bgr, {}, 1.4, 1).empty());
  EXPECT_TRUE(module.compute_result(bgr, {cv::Rect2f(180, 10, 40, 40)}, 1.4, 1).empty());
  EXPECT_TRUE(module.compute_result(bgr, {cv::Rect2f(30, 50, 80, 80)}, 0.5, 1).empty());
  EXPECT_TRUE(module.compute_result(bgr, {cv::Rect2f(30, 50, 80, 80)}, 1.4, 0).empty());
}

namespace {
std::array<cv::Point2f, 4> square_tag(cv::Point2f c, float side) {
  return {c + cv::Point2f(-side / 2, -side / 2), c + cv::Point2f(side / 2, -side / 2),
          c + cv::Point2f(side / 2, side / 2), c + cv::Point2f(-side / 2, side / 2)};
}
std::array<cv::Point2f, 4> oblique_tag(cv::Point2f c, float side) {  // seen from below: top edge short
  return {c + cv::Point2f(-side / 4, -side / 4), c + cv::Point2f(side / 4, -side / 4),
          c + cv::Point2f(side / 2, side / 4), c + cv::Point2f(-side / 2, side / 4)};
}
}  // namespace

TEST(PuckShape, FaceOnFromTagCorners) {
  const PuckShapeRule rule;
  EXPECT_TRUE(tag_face_on(square_tag({50, 50}, 10), rule));
  EXPECT_FALSE(tag_face_on(oblique_tag({50, 50}, 10), rule));
  EXPECT_FALSE(tag_face_on(std::array<cv::Point2f, 4>{}, rule));  // unknown corners: never face-on
}

TEST(PuckShape, TagsOverrideYoloCylinderWhenTheyProveACuboid) {
  const auto det = [](float x, float w, float h, int cls) { return Detection{cv::Rect2f(x, 100, w, h), cls, 0.85f, {}}; };
  const PuckClassMap classes;  // 0 red cylinder, 1 red prism, 2 green cylinder, 3 green prism
  const std::vector<Detection> yolo{
    det(100, 30, 45, 0),   // YOLO: red cylinder; two tags (two faces of one cuboid) -> cuboid
    det(200, 22, 44, 0),   // red cylinder, long box, one face-on tag -> cuboid (its 30x60 side)
    det(300, 30, 32, 0),   // red cylinder, near-square box, one face-on tag -> could be an end: keep
    det(400, 22, 44, 0),   // red cylinder, long box, one oblique tag -> keep
    det(500, 30, 45, 1)};  // red prism, no tag -> not identified
  const std::vector<PuckTag> tags{
    {1, {110, 110}, square_tag({110, 110}, 8)}, {1, {120, 135}, oblique_tag({120, 135}, 8)},
    {2, {211, 122}, square_tag({211, 122}, 8)},
    {3, {315, 116}, square_tag({315, 116}, 8)},
    {4, {411, 122}, oblique_tag({411, 122}, 8)}};
  const auto pucks = associate_pucks(yolo, tags, classes);
  ASSERT_EQ(pucks.size(), 4u);
  EXPECT_EQ(pucks[0].id, 1);
  EXPECT_EQ(pucks[0].tags, 2);
  EXPECT_EQ(pucks[0].shape, PuckShape::kCuboid);
  EXPECT_TRUE(pucks[0].shape_from_tags);
  EXPECT_EQ(pucks[1].shape, PuckShape::kCuboid);
  EXPECT_TRUE(pucks[1].shape_from_tags);
  EXPECT_EQ(pucks[2].shape, PuckShape::kCylinder);
  EXPECT_FALSE(pucks[2].shape_from_tags);
  EXPECT_EQ(pucks[3].shape, PuckShape::kCylinder);
  EXPECT_FALSE(pucks[3].shape_from_tags);
  for (const auto &p : pucks) EXPECT_EQ(p.color, PuckColor::kRed);
  PuckShapeRule off;
  off.enabled = false;
  const auto plain = associate_pucks(yolo, tags, classes, 0.4f, off);
  EXPECT_EQ(plain[0].shape, PuckShape::kCylinder);
  EXPECT_FALSE(plain[0].shape_from_tags);
}

TEST(PuckShape, DepthNormalDecidesFaceOnWhenKnown) {
  const PuckShapeRule rule;
  PuckTag tag{2, {50, 50}, oblique_tag({50, 50}, 8), cv::Vec3d(0, 0, 0.5), cv::Vec3d(0, 0, -1)};
  EXPECT_TRUE(tag_face_on(tag, rule));   // corners say oblique, the depth plane faces the camera
  tag.normal = cv::Vec3d(std::sin(CV_PI / 4), 0, -std::cos(CV_PI / 4));
  tag.corners = square_tag({50, 50}, 8);
  EXPECT_FALSE(tag_face_on(tag, rule));  // 45 deg off the line of sight
  // Depth positions of the tags become the puck position.
  const std::vector<Detection> yolo{{cv::Rect2f(30, 30, 40, 40), 2, 0.9f, {}}};
  PuckTag a{6, {45, 45}, square_tag({45, 45}, 6), cv::Vec3d(0.01, 0, 0.5), cv::Vec3d(0, 0, -1)};
  PuckTag b{6, {55, 55}, oblique_tag({55, 55}, 6), cv::Vec3d(0.03, 0.02, 0.52), cv::Vec3d(1, 0, 0)};
  const auto pucks = associate_pucks(yolo, {a, b});
  ASSERT_EQ(pucks.size(), 1u);
  ASSERT_TRUE(pucks[0].position);
  EXPECT_NEAR((*pucks[0].position)[0], 0.02, 1e-9);
  EXPECT_NEAR((*pucks[0].position)[2], 0.51, 1e-9);
  EXPECT_EQ(pucks[0].shape, PuckShape::kCylinder);  // green box: red_only keeps the YOLO shape
  PuckShapeRule all;
  all.red_only = false;
  EXPECT_EQ(associate_pucks(yolo, {a, b}, {}, 0.4f, all)[0].shape, PuckShape::kCuboid);  // two tags -> cuboid
}

TEST(PuckShape, TagDepthFitsTheTagPlane) {
  const Intrinsics in{200, 200, 100, 100};
  const auto plane = [&](double tilt_deg) {  // plane z = 0.5 + tan(tilt) * X, as an aligned depth image
    cv::Mat z(200, 200, CV_32FC1);
    const double t = std::tan(tilt_deg * CV_PI / 180);
    for (int v = 0; v < 200; ++v) for (int u = 0; u < 200; ++u) z.at<float>(v, u) = 0.5 / (1 - t * (u - in.cx) / in.fx);
    return DepthImage{z, 1.0, {}};
  };
  const auto quad = square_tag({100, 100}, 20);
  const auto flat = tag_depth(plane(0), quad, in);
  ASSERT_TRUE(flat);
  EXPECT_NEAR(flat->center[2], 0.5, 1e-3);
  EXPECT_NEAR(flat->view_angle_deg, 0.0, 1.0);
  EXPECT_LT(flat->normal[2], 0);  // toward the camera
  const auto tilted = tag_depth(plane(30), quad, in);
  ASSERT_TRUE(tilted);
  EXPECT_NEAR(tilted->view_angle_deg, 30.0, 1.5);
  // No depth or too few valid pixels: nothing.
  EXPECT_FALSE(tag_depth(DepthImage{cv::Mat::zeros(200, 200, CV_32FC1), 1.0, {}}, quad, in));
  EXPECT_FALSE(tag_depth(plane(0), quad, Intrinsics{0, 0, 0, 0}));
}

TEST(PuckShape, SurveyKeepsTagConfirmedShape) {
  PuckSurvey survey;
  survey.add(4, PuckColor::kRed, PuckShape::kCylinder, 0.9f);        // YOLO only
  survey.add(4, PuckColor::kRed, PuckShape::kCuboid, 0.6f, true);    // tags: wins despite the lower score
  EXPECT_EQ(survey.sighting(4)->shape, PuckShape::kCuboid);
  EXPECT_TRUE(survey.sighting(4)->shape_confirmed);
  survey.add(4, PuckColor::kRed, PuckShape::kCylinder, 0.99f);       // YOLO only again: ignored
  EXPECT_EQ(survey.sighting(4)->shape, PuckShape::kCuboid);
  survey.add(4, PuckColor::kRed, PuckShape::kCuboid, 0.8f, true);    // better confirmed look replaces
  EXPECT_FLOAT_EQ(survey.sighting(4)->score, 0.8f);
}

TEST(PuckTag, YoloBoxSettlesASingleCuboidTag) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const cv::Size image(640, 480);
  const cv::Vec3d side = cv::normalize(kRight + kForward * 0.3);
  // Standing cuboid, only its top (end) tag read.
  const cv::Vec3d stand = kForward * 0.35 - kUp * 0.3;  // inside the 640x480 image
  const std::vector<PuckTagAxis::Corners> top{project_tag(stand + kUp * 0.03, kUp, side)};
  EXPECT_FALSE(module.compute_result(top, kTagCamera, PuckShape::kCuboid));  // end or side? no box: no answer
  const auto standing = module.compute_result(top, kTagCamera, PuckShape::kCuboid, cuboid_box(stand, kUp, side), image);
  ASSERT_TRUE(standing);
  EXPECT_TRUE(standing->standing);
  EXPECT_LT(line_angle_deg(standing->axis, kUp), 2.0);
  EXPECT_GT(standing->box_iou, 0.9);
  // Lying cuboid yawed 30 deg, only its top (side) tag read, stuck on turned 90 deg: the box also tells which
  // of the tag's two axes is the long one.
  const double yaw = 30 * M_PI / 180;
  const cv::Vec3d lying_axis = kRight * std::cos(yaw) + kForward * std::sin(yaw);
  const cv::Vec3d lie = kForward * 0.35 - kUp * 0.315;
  const std::vector<PuckTagAxis::Corners> face{project_tag(lie + kUp * 0.015, kUp, kUp.cross(lying_axis))};
  const auto lying = module.compute_result(face, kTagCamera, PuckShape::kCuboid, cuboid_box(lie, lying_axis, kUp), image);
  ASSERT_TRUE(lying);
  EXPECT_FALSE(lying->standing);
  EXPECT_LT(line_angle_deg(lying->axis, lying_axis), 3.0);
}

TEST(PuckTag, YoloBoxCutByTheImageEdgeOrNotFittingIsNotTrusted) {
  PuckTagAxis module;
  ASSERT_TRUE(module.configure({}));
  const cv::Vec3d side = cv::normalize(kRight + kForward * 0.3);
  const cv::Vec3d stand = kForward * 0.35 - kUp * 0.3;  // inside the 640x480 image
  const std::vector<PuckTagAxis::Corners> top{project_tag(stand + kUp * 0.03, kUp, side)};
  const auto box = cuboid_box(stand, kUp, side);
  // Same box but the image ends inside it: treated as cut, so back to tags only (one tag: no answer).
  EXPECT_FALSE(module.compute_result(top, kTagCamera, PuckShape::kCuboid, box,
    cv::Size(int(box.x + box.width), 480)));
  // A box no reading of the tag can fill (three times too wide): every reading below min_box_iou.
  cv::Rect2f wide = box;
  wide.x -= box.width; wide.width *= 3;
  EXPECT_FALSE(module.compute_result(top, kTagCamera, PuckShape::kCuboid, wide, cv::Size(640, 480)));
  // Two tags (standing, top + side): the box agrees with the tags, the answer stays the same.
  const auto tags = cuboid_tags(stand, kUp, side);
  const auto plain = module.compute_result(tags, kTagCamera);
  const auto boxed = module.compute_result(tags, kTagCamera, PuckShape::kCuboid, box, cv::Size(640, 480));
  ASSERT_TRUE(plain && boxed);
  EXPECT_LT(line_angle_deg(plain->axis, boxed->axis), 0.5);
  EXPECT_EQ(plain->box_iou, -1);
}

TEST(PuckShape, EveryBoxKeepsBoxesWithoutAUsableId) {
  const std::vector<Detection> yolo{{cv::Rect2f(0, 0, 40, 60), 1, 0.9f, {}}, {cv::Rect2f(100, 0, 40, 60), 3, 0.9f, {}},
    {cv::Rect2f(200, 0, 40, 60), 2, 0.9f, {}}, {cv::Rect2f(300, 0, 40, 60), 0, 0.9f, {}}};
  const std::vector<PuckTag> tags{{3, {20, 20}}, {9, {20, 40}},  // red prism: two different ids
    {5, {120, 20}}, {5, {220, 20}}};                             // the same id on two green pucks
  EXPECT_EQ(associate_pucks(yolo, tags).size(), 1u);  // survey view: one green id-5 box only
  const auto every = associate_pucks(yolo, tags, {}, 0.4f, {}, true);
  ASSERT_EQ(every.size(), 4u);
  EXPECT_EQ(every[0].id, -1);                 // ambiguous ids
  EXPECT_EQ(every[0].tag_corners.size(), 2u);  // but both tags kept for the axis
  EXPECT_EQ(every[0].shape, PuckShape::kCuboid);
  EXPECT_EQ(every[1].id, 5);
  EXPECT_EQ(every[2].id, 5);
  EXPECT_EQ(every[3].id, -1);                 // no tag at all
  EXPECT_EQ(every[3].tags, 0);
}

// ---- traffic light: pixels color source -----------------------------------------------------
namespace {
// A 30x70 tower on a grey frame: white cap on top (like ours), one lamp lit as a clipped white core
// with a saturated ring of `ring_bgr` around it at `center_y` (0 = top of the tower).
cv::Mat tower_frame(bool lit, double center_y, cv::Scalar ring_bgr) {
  cv::Mat frame(360, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  const cv::Rect tower(300, 100, 30, 70);
  frame(tower).setTo(cv::Scalar(40, 40, 40));
  frame(cv::Rect(300, 100, 30, 12)).setTo(cv::Scalar(250, 250, 250));  // white cap: top 17 % of the box
  if (lit) {
    const cv::Point c(315, 100 + static_cast<int>(center_y * 70));
    cv::circle(frame, c, 14, ring_bgr, cv::FILLED);
    cv::circle(frame, c, 11, cv::Scalar(255, 255, 255), cv::FILLED);  // core ~25 % of the box, as on our tower
  }
  return frame;
}
PixelColorConfig our_pixels() {  // perception.yaml uppercam.traffic_light.pixels
  PixelColorConfig c;
  c.cap_fraction = 0.2; c.on_fraction = 0.10; c.use_ring_hue = false; c.red_cut = 0.49; c.yellow_cut = 0.72;
  return c;
}
cv::Scalar bgr_of_hue(int hue) {  // OpenCV hue 0..179, full saturation and value
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 230)), bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto v = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(v[0], v[1], v[2]);
}
}

TEST(TrafficLight, PixelColorOfOurTower) {
  const auto c = our_pixels();
  const cv::Rect2f box(300, 100, 30, 70);
  // Unlit: the white cap alone stays below on_fraction, whatever color the model would say.
  EXPECT_FALSE(measure_light_color(tower_frame(false, 0, {}), box, c).has_value());
  // Lamps where ours sit (cap left out): R 0.36-0.40, Y 0.58-0.63, G 0.80-0.85. Red's ring hue 14 is inside
  // yellow's hue range on our camera: the position decides.
  EXPECT_EQ(measure_light_color(tower_frame(true, 0.38, bgr_of_hue(14)), box, c), LightColor::kRed);
  EXPECT_EQ(measure_light_color(tower_frame(true, 0.60, bgr_of_hue(17)), box, c), LightColor::kYellow);
  EXPECT_EQ(measure_light_color(tower_frame(true, 0.82, bgr_of_hue(32)), box, c), LightColor::kGreen);
  // With the cap in (cap_fraction 0) the unlit cap alone reads as a lit lamp at a low on_fraction.
  PixelColorConfig with_cap = c;
  with_cap.cap_fraction = 0;
  EXPECT_EQ(measure_light_color(tower_frame(false, 0, {}), box, with_cap), LightColor::kRed);
}

TEST(TrafficLight, PixelsSourceIgnoresTheModelColor) {
  TrafficLightConfig config;
  config.red_class_id = 1; config.yellow_class_id = 2; config.green_class_id = 3; config.off_class_id = 4;
  config.color_source = ColorSource::kPixels;
  config.pixels = our_pixels();
  TrafficLightCounter counter;
  ASSERT_TRUE(counter.configure(config));
  const auto tower = [](int cls, float score) { return Detection{cv::Rect2f(300, 100, 30, 70), cls, score, {}}; };
  const auto dark = tower_frame(false, 0, {});
  const auto green = tower_frame(true, 0.82, bgr_of_hue(32));
  double t = 0;
  const double dt = 1.0 / 15;
  // 1 s: the model says sign_yellow 0.9 on an unlit tower (2026-11-12 false blinks) -> nothing counted.
  for (int i = 0; i < 15; ++i, t += dt) counter.update(t, {tower(2, 0.9f)}, dark);
  for (int i = 0; i < 15; ++i, t += dt) counter.update(t, {tower(4, 0.8f)}, dark);
  EXPECT_TRUE(counter.blinks().empty());
  EXPECT_TRUE(counter.rejected().empty());
  // 1 s lit green, the model calling it sign_off 0.6: counted as green from the pixels.
  for (int i = 0; i < 15; ++i, t += dt) counter.update(t, {tower(4, 0.6f)}, green);
  for (int i = 0; i < 15; ++i, t += dt) counter.update(t, {tower(4, 0.8f)}, dark);
  ASSERT_EQ(counter.blinks().size(), 1u);
  EXPECT_EQ(counter.blinks()[0].color, LightColor::kGreen);
  // No frame given: nothing is lit.
  TrafficLightCounter blind;
  ASSERT_TRUE(blind.configure(config));
  for (int i = 0; i < 15; ++i) blind.update(i * dt, {tower(3, 0.9f)});
  blind.flush();
  EXPECT_TRUE(blind.blinks().empty());
}
