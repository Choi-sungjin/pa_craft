#include "craft_perception/object_detector.hpp"
#include "craft_perception/aruco_reader.hpp"
#include "craft_perception/knob_depth.hpp"
#include "craft_perception/knob_lever.hpp"
#include "craft_perception/depth_heatmap.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/state_estimator.hpp"
#include "craft_perception/obstacle_detector.hpp"
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
  ArucoReader aruco;
  EXPECT_FALSE(aruco.configure(-1));
  KnobDepth depth;
  EXPECT_FALSE(depth.configure(1.0, 0.03, 50));
  EXPECT_FALSE(depth.configure(2.0, std::numeric_limits<double>::quiet_NaN(), 50));
  EXPECT_FALSE(depth.configure(2.0, 0.03, 2));
  DepthHeatmap heatmap;
  EXPECT_FALSE(heatmap.configure(0.3, 0.2));
  ObstacleDetector obstacle;
  EXPECT_FALSE(obstacle.configure(3.0, 0));
  ObjectDetector detector;
  DetectorConfig config;
  config.model_dir = "/does-not-exist";
  config.confidence = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(detector.configure(config));
}

TEST(ModuleContract, SeparatesInvalidInputsFromValidEmptyObservations) {
  cv::Mat gray(10, 10, CV_8UC1), bgr(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));
  ObjectDetector detector;
  ArucoReader aruco;
  EXPECT_FALSE(detector.is_valid_input(gray));
  EXPECT_FALSE(aruco.is_valid_input(gray));
  EXPECT_FALSE(KnobLever().is_valid_input(bgr, {20, 20, 5, 5}));
  EXPECT_TRUE(detector.is_valid_input(bgr));
  EXPECT_TRUE(aruco.is_valid_input(bgr));
  KnobDepth depth;
  EXPECT_FALSE(depth.is_valid_input({gray, 1.0, {}}, {0, 0, 5, 5}, {100, 100, 5, 5}));
  DepthHeatmap heatmap;
  EXPECT_FALSE(heatmap.is_valid_input({gray, 1.0, {}}));
  sensor_msgs::msg::LaserScan scan;
  ObstacleDetector obstacle;
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

TEST(Aruco, SortsIdsByImagePosition) {
  cv::Mat image(220, 420, CV_8UC1, cv::Scalar(255)), left, right, bgr;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::drawMarker(dictionary, 17, 120, left);
  cv::aruco::drawMarker(dictionary, 3, 120, right);
  left.copyTo(image(cv::Rect(30, 50, 120, 120)));
  right.copyTo(image(cv::Rect(260, 50, 120, 120)));
  cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  ArucoReader module;
  ASSERT_TRUE(module.configure(cv::aruco::DICT_4X4_50));
  ASSERT_TRUE(module.is_valid_input(bgr));
  EXPECT_EQ(module.compute_result(bgr), (std::vector<int>{17, 3}));
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

TEST(Heatmap, MissingBlackNearRedFarBlue) {
  cv::Mat depth(1, 3, CV_32FC1);
  depth.at<float>(0, 0) = 0; depth.at<float>(0, 1) = 0.2; depth.at<float>(0, 2) = 0.3;
  DepthHeatmap module;
  ASSERT_TRUE(module.configure(0.2, 0.3));
  ASSERT_TRUE(module.is_valid_input({depth, 1.0, {}}));
  const auto image = module.compute_result({depth, 1.0, {}});
  EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
  EXPECT_EQ(image.at<cv::Vec3b>(0, 1), cv::Vec3b(0, 0, 255));
  EXPECT_EQ(image.at<cv::Vec3b>(0, 2), cv::Vec3b(255, 0, 0));
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
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser"; scan.angle_min = 0; scan.angle_increment = CV_PI / 2;
  scan.range_min = 0.1; scan.range_max = 10;
  scan.ranges = {1, 2, std::numeric_limits<float>::infinity(), 9};
  ObstacleDetector module;
  ASSERT_TRUE(module.configure(3, 1));
  ASSERT_TRUE(module.is_valid_input(scan));
  const auto points = module.compute_result(scan);
  ASSERT_EQ(points.poses.size(), 2u);
  EXPECT_EQ(points.header.frame_id, "laser");
  EXPECT_NEAR(points.poses[0].position.x, 1, 1e-6);
  EXPECT_NEAR(points.poses[1].position.y, 2, 1e-6);
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
  DepthHeatmap heatmap;
  ASSERT_TRUE(heatmap.configure(0.25, 0.3));
  ASSERT_TRUE(heatmap.is_valid_input(input));
  auto heat = heatmap.compute_result(input);
  EXPECT_EQ(heat.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));
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
