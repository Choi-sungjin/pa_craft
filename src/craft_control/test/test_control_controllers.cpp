#include <cmath>
#include <limits>
#include <utility>

#include "gtest/gtest.h"
#include "craft_control/arm_controller.hpp"
#include "craft_control/clock_controller.hpp"
#include "craft_control/lateral_controller.hpp"
#include "craft_control/lm_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 0.033;
constexpr double kSeparation = 0.288;

nav_msgs::msg::Odometry odom(double x = 0.0, double y = 0.0, double yaw = 0.0)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  return msg;
}

nav_msgs::msg::Path path(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = "odom";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    msg.poses.push_back(pose);
  }
  return msg;
}

craft_control::LateralController lateral()
{
  craft_control::LateralController controller;
  EXPECT_TRUE(controller.set_params(kRadius, kSeparation, 0.3, 0.03));
  EXPECT_TRUE(controller.set_odom(odom()));
  return controller;
}
}  // namespace

TEST(ArmController, ValidationDoesNotStoreOrChangeTargets)
{
  craft_control::ArmController controller;
  const craft_control::ArmIkJointNames names = {
    "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"};
  const std::vector<std::string> joint_names(names.begin(), names.end());
  const std::vector<double> positions = {0.1, 0.2, 0.3, 0.4};
  EXPECT_FALSE(controller.is_valid_target(joint_names, positions));
  ASSERT_TRUE(controller.set_angle_limits({-10, -10, -10, -10}, {10, 10, 10, 10}));
  ASSERT_TRUE(controller.set_actuator_mapping(
    names, {1, 1, 1, 1}, {0, 0, 0, 0}));

  const craft_control::ArmJoints feedback{};
  const auto hold = controller.compute_arm_command(feedback);
  ASSERT_TRUE(controller.is_valid_target(joint_names, positions));
  EXPECT_EQ(controller.compute_arm_command(feedback), hold);

  controller.set_target_arm_pose({0.1, 0.2, 0.3, 0.4});
  const auto command = controller.compute_arm_command(feedback);
  EXPECT_DOUBLE_EQ(command[0], 0.1);  // Rotational arm positions need no LM calibration.
  EXPECT_DOUBLE_EQ(command[1], 0.2);
  EXPECT_FALSE(controller.is_valid_target(joint_names, {0.1}));
  EXPECT_EQ(controller.compute_arm_command(feedback), command);
  controller.clear_target();
  EXPECT_EQ(controller.compute_arm_command(feedback), hold);
}

TEST(ArmController, RejectsWrongNamesAndNonFinitePositions)
{
  craft_control::ArmController controller;
  const craft_control::ArmIkJointNames names = {
    "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"};
  ASSERT_TRUE(controller.set_actuator_mapping(
    names, {1, 1, 1, 1}, {0, 0, 0, 0}));
  std::vector<std::string> joint_names(names.begin(), names.end());
  std::vector<double> positions(4, 0.0);
  ASSERT_TRUE(controller.is_valid_target(joint_names, positions));
  joint_names[0] = "wrong_joint";
  EXPECT_FALSE(controller.is_valid_target(joint_names, positions));
  joint_names[0] = names[0];
  positions[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.is_valid_target(joint_names, positions));
}

TEST(ArmController, AppliesOnlyRotationalMappingAndLimits)
{
  craft_control::ArmController controller;
  ASSERT_TRUE(controller.set_angle_limits({-1, -1, -1, -1}, {1, 1, 1, 1}));
  ASSERT_TRUE(controller.set_actuator_mapping(
    {"shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"},
    {-1, 1, 1, 1}, {0.1, 0, 0, 0}));
  controller.set_target_arm_pose({0.2, 0.3, 2.0, -2.0});
  const auto command = controller.compute_arm_command({0, 0, 0, 0});
  EXPECT_NEAR(command[0], -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(command[1], 0.3);
  EXPECT_DOUBLE_EQ(command[2], 1.0);
  EXPECT_DOUBLE_EQ(command[3], -1.0);
}

TEST(LmController, ConvertsLinearDisplacementUsingItsOwnMappingAndLimits)
{
  craft_control::LmController controller;
  ASSERT_TRUE(controller.configure("slide_joint", -5.0, 5.0, -1.0, 0.2, 0.01, 2.0));
  EXPECT_DOUBLE_EQ(controller.compute_lm_command(0.4), 0.4);
  ASSERT_TRUE(controller.is_valid_target("slide_joint", 0.01));
  controller.set_target_position(0.01);
  EXPECT_NEAR(controller.compute_lm_command(0.4), -1.8, 1e-12);
  controller.set_target_position(1.0);
  EXPECT_DOUBLE_EQ(controller.compute_lm_command(0.4), -5.0);
  controller.clear_target();
  EXPECT_DOUBLE_EQ(controller.compute_lm_command(0.4), 0.4);
}

TEST(LmController, PlaceholderCalibrationHoldsAndRejectsTargets)
{
  craft_control::LmController controller;
  ASSERT_TRUE(controller.configure("slide_joint", -1.0, 1.0, 1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", 0.1));
  EXPECT_DOUBLE_EQ(controller.compute_lm_command(0.3), 0.3);
}

TEST(LmController, RejectsWrongNamesInvalidCalibrationAndNonFiniteTargets)
{
  craft_control::LmController controller;
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 0, 0, 0.01, 1));
  EXPECT_FALSE(controller.configure("slide_joint", 1, -1, 1, 0, 0.01, 1));
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 1, 0, 0, 1));
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 1, 0, -0.01, 1));
  ASSERT_TRUE(controller.configure("slide_joint", -1, 1, 1, 0, 0.01, 1));
  EXPECT_FALSE(controller.is_valid_target("shoulder_joint", 0.1));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", std::numeric_limits<double>::infinity()));
}

TEST(ClockController, HoldsBeforeTargetAndNeverReportsUncommandedCompletion)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  EXPECT_DOUBLE_EQ(controller.compute_clock_command(1.2), 1.2);
  EXPECT_FALSE(controller.is_target_reached(0.0, 0));
  EXPECT_EQ(controller.get_current_slot(0.0), 12);
}

TEST(ClockController, TwelveToSevenUsesClockwiseShortestTurn)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  ASSERT_TRUE(controller.set_target_slot(7, 0.0));
  EXPECT_NEAR(controller.compute_clock_command(0.0), -5.0 * kPi / 6.0, 1e-12);
  EXPECT_EQ(controller.get_current_slot(-5.0 * kPi / 6.0), 7);
}

TEST(ClockController, AllSlotPairsUseAtMostHalfATurnAndPreserveWinding)
{
  for (int from = 1; from <= 12; ++from) {
    for (int to = 1; to <= 12; ++to) {
      craft_control::ClockController controller;
      ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
      const double current = 4.0 * kPi + (from % 12) * kPi / 6.0;
      ASSERT_TRUE(controller.set_target_slot(to, current));
      const double target = controller.compute_clock_command(current);
      EXPECT_LE(std::abs(target - current), kPi + 1e-12);
      EXPECT_EQ(controller.get_current_slot(target), to);
      EXPECT_TRUE(controller.is_target_reached(target, 0));
    }
  }
}

TEST(ClockController, SlotBoundaryCrossingUsesShortTurnAndRepeatedTargetStaysFixed)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  const double current = 11.0 * kPi / 6.0;
  ASSERT_TRUE(controller.set_target_slot(1, current));
  const double target = 13.0 * kPi / 6.0;
  EXPECT_NEAR(controller.compute_clock_command(current), target, 1e-12);
  ASSERT_TRUE(controller.set_target_slot(1, current - 4.0));
  EXPECT_NEAR(controller.compute_clock_command(current), target, 1e-12);
}

TEST(ClockController, CompletionRequiresTargetAngleAndIdleState)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  ASSERT_TRUE(controller.set_target_slot(3, 0.0));
  EXPECT_FALSE(controller.is_target_reached(0.0, 0));
  EXPECT_FALSE(controller.is_target_reached(kPi / 2.0, 1));
  EXPECT_FALSE(controller.is_target_reached(kPi / 2.0, 2));
  EXPECT_TRUE(controller.is_target_reached(kPi / 2.0 + 0.005, 0));
  EXPECT_FALSE(controller.is_target_reached(kPi / 2.0 + 0.02, 0));
  EXPECT_EQ(controller.get_current_slot(kPi / 12.0), 0);
  EXPECT_EQ(controller.get_current_slot(kPi / 2.0 + 0.005), 3);
}

TEST(ClockController, RejectsInvalidSlotsTolerancesAndNonFiniteFeedback)
{
  craft_control::ClockController controller;
  EXPECT_FALSE(controller.set_target_slot(1, 0.0));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.0));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, kPi / 12.0));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, std::numeric_limits<double>::infinity()));
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  EXPECT_FALSE(controller.set_target_slot(0, 0.0));
  EXPECT_FALSE(controller.set_target_slot(13, 0.0));
  EXPECT_FALSE(controller.set_target_slot(1, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(controller.get_current_slot(std::numeric_limits<double>::infinity()), 0);
}

TEST(ClockController, UsesConfiguredSlotGeometryZeroAndDirection)
{
  craft_control::ClockController controller;
  // Eight slots, 90 motor degrees per slot, reversed direction, and a shifted zero.
  ASSERT_TRUE(controller.configure(8, kPi / 2.0, 0.3, -1.0, 0.02));
  EXPECT_EQ(controller.get_current_slot(0.3), 8);
  ASSERT_TRUE(controller.set_target_slot(7, 0.3));
  const double target = 0.3 + kPi / 2.0;
  EXPECT_NEAR(controller.compute_clock_command(0.3), target, 1e-12);
  EXPECT_EQ(controller.get_current_slot(target), 7);
  EXPECT_TRUE(controller.is_target_reached(target + 0.015, 0));
  EXPECT_FALSE(controller.is_target_reached(target + 0.025, 0));
  EXPECT_FALSE(controller.set_target_slot(9, 0.3));
}

TEST(ClockController, RejectsInvalidGeometry)
{
  craft_control::ClockController controller;
  EXPECT_FALSE(controller.configure(1, kPi / 6.0, 0.0, 1.0, 0.01));
  EXPECT_FALSE(controller.configure(12, 0.0, 0.0, 1.0, 0.01));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 0.0, 0.01));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 2.0, 0.01));
  EXPECT_FALSE(controller.configure(
    12, kPi / 6.0, std::numeric_limits<double>::quiet_NaN(), 1.0, 0.01));
  EXPECT_FALSE(controller.configure(
    12, std::numeric_limits<double>::infinity(), 0.0, 1.0, 0.01));
}

TEST(WheelControl, StraightPathKeepsWheelsEqualAndLongitudinalLimitsSpeed)
{
  auto controller = lateral();
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {1, 0}, {2, 0}})));
  ASSERT_TRUE(controller.can_track(0.2));
  const auto turn = controller.get_wheel_speed(0.2);
  EXPECT_DOUBLE_EQ(turn[0], 0.0);
  EXPECT_DOUBLE_EQ(turn[1], 0.0);
  craft_control::LongitudinalController longitudinal;
  longitudinal.set_params(kRadius, 0.26);
  longitudinal.set_target_v(1.0);
  const auto limited = longitudinal.get_wheel_speed();
  EXPECT_NEAR(limited[0] * kRadius, 0.26, 1e-12);
  EXPECT_DOUBLE_EQ(limited[0], limited[1]);
  longitudinal.stop();
  EXPECT_DOUBLE_EQ(longitudinal.get_wheel_speed()[0], 0.0);
}

TEST(WheelControl, LeftAndRightPathsProduceOppositeDifferentialSpeeds)
{
  auto controller = lateral();
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {1, 1}, {2, 2}})));
  const auto left = controller.get_wheel_speed(0.2);
  EXPECT_NEAR(left[0], -0.2 * kSeparation / (2.0 * kRadius), 1e-12);
  EXPECT_NEAR(left[1], -left[0], 1e-12);
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {1, -1}, {2, -2}})));
  const auto right = controller.get_wheel_speed(0.2);
  EXPECT_NEAR(right[0], -left[0], 1e-12);
  EXPECT_NEAR(right[1], -left[1], 1e-12);
}

TEST(WheelControl, TransformsWorldPathUsingOdometryYaw)
{
  auto controller = lateral();
  ASSERT_TRUE(controller.set_odom(odom(3.0, 4.0, kPi / 2.0)));
  ASSERT_TRUE(controller.set_path(path({{3, 4}, {2, 5}, {1, 6}})));
  const auto speeds = controller.get_wheel_speed(0.2);
  EXPECT_NEAR(speeds[0], -0.2 * kSeparation / (2.0 * kRadius), 1e-12);
  EXPECT_GT(speeds[1], 0.0);
}

TEST(WheelControl, ReversePathPursuesPointsBehindTheRobot)
{
  auto controller = lateral();
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {-1, 1}, {-2, 2}})));
  EXPECT_FALSE(controller.can_track(0.2));
  EXPECT_TRUE(controller.can_track(-0.2));
  const auto speeds = controller.get_wheel_speed(-0.2);
  EXPECT_GT(speeds[0], 0.0);
  EXPECT_LT(speeds[1], 0.0);
  const auto stopped = controller.get_wheel_speed(0.0);
  EXPECT_DOUBLE_EQ(stopped[0], 0.0);
  EXPECT_DOUBLE_EQ(stopped[1], 0.0);
}

TEST(WheelControl, MissingPathMismatchedFramesAndGoalArrivalDisableTracking)
{
  auto controller = lateral();
  EXPECT_FALSE(controller.can_track(0.2));
  auto target = path({{0, 0}, {1, 0}});
  target.header.frame_id = "map";
  ASSERT_TRUE(controller.set_path(target));
  EXPECT_FALSE(controller.can_track(0.2));
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {1, 0}})));
  ASSERT_TRUE(controller.set_odom(odom(0.99, 0.0)));
  EXPECT_FALSE(controller.can_track(0.2));
}

TEST(WheelControl, InvalidInputsCancelPreviousTracking)
{
  auto controller = lateral();
  auto target = path({{0, 0}, {1, 0}});
  ASSERT_TRUE(controller.set_path(target));
  ASSERT_TRUE(controller.can_track(0.2));
  target.poses[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.set_path(target));
  EXPECT_FALSE(controller.can_track(0.2));
  ASSERT_TRUE(controller.set_path(path({{0, 0}, {1, 0}})));
  auto invalid_odom = odom();
  invalid_odom.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(controller.set_odom(invalid_odom));
  EXPECT_FALSE(controller.can_track(0.2));
  EXPECT_FALSE(controller.set_params(kRadius, 0.0, 0.3, 0.03));
}
