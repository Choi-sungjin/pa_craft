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
#include "craft_perception/imu_processor.hpp"
#include "craft_perception/localization_monitor.hpp"
#include "craft_perception/scan_relocalizer.hpp"
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

// Board frame of a board tilted by `tilt` about its x axis, then turned `heading` about the vertical:
// returns the board-frame vector of chassis-frame v.
static Vector3 to_board(const Vector3 &v, double heading, double tilt) {
  const double ch = std::cos(heading), sh = std::sin(heading), ct = std::cos(tilt), st = std::sin(tilt);
  const Vector3 h{ch * v[0] + sh * v[1], -sh * v[0] + ch * v[1], v[2]};
  return {h[0], ct * h[1] + st * h[2], -st * h[1] + ct * h[2]};
}

TEST(BoardImu, UprightTurnedAndFlippedBoardsAllReportChassisYawRate) {
  const Vector3 gravity{0.0, 0.0, 1.0};  // a still accelerometer reads +1 g up
  // wheel_clock upright; lm_arm turned ~180 deg and tilted 4 deg; an upside-down board for the old mounting.
  const std::pair<double, double> mountings[] = {{0.0, 0.0}, {3.05, 0.07}, {0.3, M_PI}};
  for (const auto & [heading, tilt] : mountings) {
    BoardImu imu;
    ASSERT_TRUE(imu.configure(10, 0.0, 0.05));
    const Vector3 bias{0.004, -0.003, 0.002};
    for (int i = 0; i < 10; ++i) {
      EXPECT_FALSE(imu.yaw_rate());
      imu.update(bias, to_board(gravity, heading, tilt), true);
    }
    ASSERT_TRUE(imu.calibrated());
    const auto rate = to_board({0.0, 0.0, 0.5}, heading, tilt);  // chassis turns left at 0.5 rad/s
    imu.update({rate[0] + bias[0], rate[1] + bias[1], rate[2] + bias[2]}, to_board(gravity, heading, tilt), false);
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
  return {0, 1, 0.0, 0.05, 1e-4, 0.02, 0.08, 0.1, 2.5, 3.2, 0.1, disagreement_samples};
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

