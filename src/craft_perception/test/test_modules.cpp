#include "craft_perception/yolo_detector.hpp"
#include "craft_perception/object_detector.hpp"
#include "craft_perception/puck_axis.hpp"
#include "craft_perception/d435_perception.hpp"
#include "craft_perception/c310_perception.hpp"
#include "craft_perception/apriltag_reader.hpp"
#include "craft_perception/knob_depth.hpp"
#include "craft_perception/knob_lever.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/obstacle_detector.hpp"
#include "craft_perception/lidar_obstacle_detector.hpp"
#include "craft_perception/lidar_perception.hpp"
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
  KnobDepth depth;
  EXPECT_FALSE(depth.configure(1.0, 0.03, 50));
  EXPECT_FALSE(depth.configure(2.0, std::numeric_limits<double>::quiet_NaN(), 50));
  EXPECT_FALSE(depth.configure(2.0, 0.03, 2));
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
  D435Config d435;  // knob id outside the class list, nothing to load
  d435.use_yolo = d435.use_apriltag = d435.use_puck = false;
  d435.obstacle_class_ids = {1};
  d435.knob_class_id = 5;
  EXPECT_FALSE(D435Perception().configure(d435));
  d435.knob_class_id = 0;
  EXPECT_TRUE(D435Perception().configure(d435));
  C310Config c310;
  c310.use_knob_lever = false;
  c310.knob_class_id = -1;
  EXPECT_FALSE(C310Perception().configure(c310));
}

TEST(ModuleContract, SeparatesInvalidInputsFromValidEmptyObservations) {
  cv::Mat gray(10, 10, CV_8UC1), bgr(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));
  YoloDetector detector;
  AprilTagReader apriltag;
  EXPECT_FALSE(detector.is_valid_input(gray));
  EXPECT_FALSE(apriltag.is_valid_input(gray));
  EXPECT_FALSE(KnobLever().is_valid_input(bgr, {20, 20, 5, 5}));
  EXPECT_TRUE(detector.is_valid_input(bgr));
  EXPECT_TRUE(apriltag.is_valid_input(bgr));
  KnobDepth depth;
  EXPECT_FALSE(depth.is_valid_input({gray, 1.0, {}}, {0, 0, 5, 5}, {100, 100, 5, 5}));
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

TEST(Depth, FitsFrontPlaneInMetresAndFacesCamera) {
  cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(0.25));
  KnobDepth module;
  ASSERT_TRUE(module.configure(2.0, 0.03, 50));
  ASSERT_TRUE(module.is_valid_input({depth, 1.0, {}}, {40, 40, 20, 20}, {100, 100, 50, 50}));
  const auto result = module.compute_result({depth, 1.0, {}}, {40, 40, 20, 20}, {100, 100, 50, 50});
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->position[2], 0.25, 1e-6);
  EXPECT_NEAR(result->position[0], 0, 1e-6);
  ASSERT_TRUE(result->normal_valid);
  EXPECT_NEAR(result->normal[2], -1, 1e-6);
  EXPECT_NEAR(result->plane_rms_m, 0, 1e-6);
}

TEST(Depth, MissingDepthAndInvalidIntrinsicsDoNotInventPositions) {
  cv::Mat empty(50, 50, CV_32FC1, cv::Scalar(0));
  KnobDepth module;
  ASSERT_TRUE(module.configure(2.0, 0.03, 50));
  ASSERT_TRUE(module.is_valid_input({empty, 1.0, {}}, {10, 10, 20, 20}, {100, 100, 25, 25}));
  EXPECT_FALSE(module.compute_result({empty, 1.0, {}}, {10, 10, 20, 20}, {100, 100, 25, 25}));
  empty.setTo(0.25);
  EXPECT_FALSE(module.is_valid_input({empty, 1.0, {}}, {10, 10, 20, 20}, {0, 100, 25, 25}));
}

TEST(Lever, RightLeverIsClockwiseQuarterTurn) {
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 0, 100));
  cv::rectangle(image, cv::Rect(45, 44, 45, 12), cv::Scalar(0, 0, 255), cv::FILLED);
  KnobLever module;
  ASSERT_TRUE(module.is_valid_input(image, {0, 0, 100, 100}));
  const auto angle = module.compute_result(image, {0, 0, 100, 100});
  ASSERT_TRUE(angle);
  EXPECT_NEAR(*angle, CV_PI / 2, 0.05);
  EXPECT_FALSE(module.compute_result(cv::Mat::zeros(100, 100, CV_8UC3), {0, 0, 100, 100}));
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

TEST(Depth, RawZ16UsesScaleWithoutAFullFrameConversion) {
  DepthImage input{cv::Mat(100, 100, CV_16UC1, cv::Scalar(500)), 0.0005, {}};
  KnobDepth module;
  ASSERT_TRUE(module.configure(2.0, 0.03, 50));
  ASSERT_TRUE(module.is_valid_input(input, {40, 40, 20, 20}, {100, 100, 50, 50}));
  auto result = module.compute_result(input, {40, 40, 20, 20}, {100, 100, 50, 50});
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->position[2], 0.25, 1e-6);
  EXPECT_TRUE(result->normal_valid);
  EXPECT_NEAR(result->normal[2], -1, 1e-6);
  EXPECT_EQ(input.raw.type(), CV_16UC1);
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

TEST(Lever, SegMaskUsesSameAngleConvention) {
  cv::Mat mask(100, 100, CV_8U, cv::Scalar(0));
  cv::rectangle(mask, cv::Rect(45, 44, 45, 12), cv::Scalar(255), cv::FILLED);  // center -> right
  KnobLever module;
  ASSERT_TRUE(module.is_valid_input(mask));
  const auto angle = module.compute_result(mask);
  ASSERT_TRUE(angle);
  EXPECT_NEAR(*angle, CV_PI / 2, 0.05);
  EXPECT_FALSE(module.compute_result(cv::Mat::zeros(100, 100, CV_8U)));
  EXPECT_FALSE(module.is_valid_input(cv::Mat(10, 10, CV_8UC3)));
}

TEST(Lever, C310RejectsSegWithoutPrototypesOrLeverClass) {
  C310Config c310;
  c310.use_knob_lever = false;  // configuration checks only, no model load
  c310.lever_source = "yolo_seg";
  c310.lever_class_id = 4;
  EXPECT_FALSE(C310Perception().configure(c310));  // detect model (no proto blob)
  c310.yolo.proto_blob = "out1";
  EXPECT_TRUE(C310Perception().configure(c310));
  c310.lever_class_id = c310.knob_class_id;
  EXPECT_FALSE(C310Perception().configure(c310));
  c310.lever_source = "color";
  EXPECT_FALSE(C310Perception().configure(c310));
  c310.lever_source = "hsv";
  EXPECT_TRUE(C310Perception().configure(c310));
}
