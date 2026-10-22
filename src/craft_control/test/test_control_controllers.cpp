#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "gtest/gtest.h"
#include "craft_control/arm_controller.hpp"
#include "craft_control/clock_controller.hpp"
#include "craft_control/gripper_controller.hpp"
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

// Pure Pursuit wheel delta for a target at chassis-frame (x, y): first resampled point >= lookahead.
double expected_wheel_delta(const craft_control::LateralController & controller, double v)
{
  for (const auto & pose : controller.target_path().poses) {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    const double d = std::hypot(x, y);
    if (x > 0.0 && d >= 0.31) {
      return v * (2.0 * y / (d * d)) * kSeparation / (2.0 * kRadius);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

craft_control::LateralController lateral()
{
  craft_control::LateralController controller;
  controller.configure(kRadius, kSeparation, 0.31, 0.03, 0.02);  // lookahead off the 2 cm grid
  controller.set_odom(odom());
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
  ASSERT_TRUE(controller.configure(
    {-10, -10, -10, -10}, {10, 10, 10, 10},
    names, {1, 1, 1, 1}, {0, 0, 0, 0}, 0.01));

  const craft_control::ArmJoints feedback{};
  const auto hold = controller.compute_command(feedback);
  ASSERT_TRUE(controller.is_valid_target(joint_names, positions));
  EXPECT_EQ(controller.compute_command(feedback), hold);

  controller.set_target({0.1, 0.2, 0.3, 0.4});
  const auto command = controller.compute_command(feedback);
  EXPECT_DOUBLE_EQ(command[0], 0.1);  // Rotational arm positions need no LM calibration.
  EXPECT_DOUBLE_EQ(command[1], 0.2);
  EXPECT_FALSE(controller.is_valid_target(joint_names, {0.1}));
  EXPECT_EQ(controller.compute_command(feedback), command);
  controller.clear_target();
  EXPECT_EQ(controller.compute_command(feedback), hold);
}

TEST(ArmController, RejectsWrongNamesAndNonFinitePositions)
{
  craft_control::ArmController controller;
  const craft_control::ArmIkJointNames names = {
    "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"};
  ASSERT_TRUE(controller.configure(
    {-1, -1, -1, -1}, {1, 1, 1, 1}, names, {1, 1, 1, 1}, {0, 0, 0, 0}, 0.01));
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
  ASSERT_TRUE(controller.configure(
    {-1, -1, -1, -1}, {1, 1, 1, 1},
    {"shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"},
    {-1, 1, 1, 1}, {0.1, 0, 0, 0}, 0.01));
  controller.set_target({0.2, 0.3, 2.0, -2.0});
  const auto command = controller.compute_command({0, 0, 0, 0});
  EXPECT_NEAR(command[0], -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(command[1], 0.3);
  EXPECT_DOUBLE_EQ(command[2], 1.0);
  EXPECT_DOUBLE_EQ(command[3], -1.0);
}

TEST(LmController, ConvertsLinearDisplacementUsingItsOwnMappingAndLimits)
{
  craft_control::LmController controller;
  ASSERT_TRUE(controller.configure("slide_joint", -5.0, 5.0, -1.0, 0.2, 0.01, 2.0, 0.01));
  EXPECT_DOUBLE_EQ(controller.compute_command(0.4), 0.4);
  ASSERT_TRUE(controller.is_valid_target("slide_joint", 0.01));
  controller.set_target(0.01);
  EXPECT_NEAR(controller.compute_command(0.4), -1.8, 1e-12);
  controller.set_target(1.0);
  EXPECT_DOUBLE_EQ(controller.compute_command(0.4), -5.0);
  controller.clear_target();
  EXPECT_DOUBLE_EQ(controller.compute_command(0.4), 0.4);
}

TEST(LmController, PlaceholderCalibrationHoldsAndRejectsTargets)
{
  craft_control::LmController controller;
  ASSERT_TRUE(controller.configure("slide_joint", -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.01));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", 0.1));
  EXPECT_DOUBLE_EQ(controller.compute_command(0.3), 0.3);
}

TEST(LmController, RejectsWrongNamesInvalidCalibrationAndNonFiniteTargets)
{
  craft_control::LmController controller;
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 0, 0, 0.01, 1, 0.01));
  EXPECT_FALSE(controller.configure("slide_joint", 1, -1, 1, 0, 0.01, 1, 0.01));
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 1, 0, 0, 1, 0.01));
  EXPECT_FALSE(controller.configure("slide_joint", -1, 1, 1, 0, -0.01, 1, 0.01));
  ASSERT_TRUE(controller.configure("slide_joint", -1, 1, 1, 0, 0.01, 1, 0.01));
  EXPECT_FALSE(controller.is_valid_target("shoulder_joint", 0.1));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(controller.is_valid_target("slide_joint", std::numeric_limits<double>::infinity()));
}

TEST(GripperController, HoldsUntilTargetAndSelectsOpenCloseAngles)
{
  craft_control::GripperController controller;
  ASSERT_TRUE(controller.configure(0.0, 2.0, 0.5, 1.5, 0.0, 0.0, 0.05));
  EXPECT_DOUBLE_EQ(controller.compute_command(0.8), 0.8);
  EXPECT_DOUBLE_EQ(controller.compute_command(3.0), 2.0);
  controller.set_target(true);
  EXPECT_DOUBLE_EQ(controller.compute_command(0.8), 1.5);
  controller.set_target(false);
  EXPECT_DOUBLE_EQ(controller.compute_command(0.8), 0.5);
  EXPECT_FALSE(controller.configure(2.0, 0.0, 0.5, 1.5, 0.0, 0.0, 0.05));
}

TEST(ArmController, TargetReachedRequiresAllJointsWithinTolerance)
{
  craft_control::ArmController controller;
  ASSERT_TRUE(controller.configure(
    {-10, -10, -10, -10}, {10, 10, 10, 10},
    {"shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"},
    {1, -1, 1, 1}, {0.5, 0, 0, 0}, 0.01));
  EXPECT_FALSE(controller.is_target_reached({0, 0, 0, 0}));  // no target
  controller.set_target({0.1, 0.2, 0.3, 0.4});
  EXPECT_TRUE(controller.is_target_reached({0.605, -0.2, 0.3, 0.4}));  // motor space
  EXPECT_FALSE(controller.is_target_reached({0.6, -0.2, 0.3, 0.42}));
  controller.clear_target();
  EXPECT_FALSE(controller.is_target_reached({0.6, -0.2, 0.3, 0.4}));
}

TEST(LmController, TargetReachedComparesMotorAngle)
{
  craft_control::LmController controller;
  ASSERT_TRUE(controller.configure("slide_joint", -5.0, 5.0, -1.0, 0.2, 0.01, 2.0, 0.01));
  EXPECT_FALSE(controller.is_target_reached(0.2));  // no target
  controller.set_target(0.01);  // -1.8 motor rad
  EXPECT_TRUE(controller.is_target_reached(-1.795));
  EXPECT_FALSE(controller.is_target_reached(-1.7));
}

TEST(GripperController, CloseCompletesOnAngleOrGrasp)
{
  craft_control::GripperController controller;
  ASSERT_TRUE(controller.configure(0.0, 2.0, 0.5, 1.5, 1.0, 0.1, 0.05));
  EXPECT_FALSE(controller.is_target_reached(0.5));  // no target
  controller.set_target(false);
  EXPECT_TRUE(controller.is_target_reached(0.52));
  controller.set_target(true);
  EXPECT_FALSE(controller.is_target_reached(1.0));
  EXPECT_TRUE(controller.is_target_reached(1.48));
  controller.update_current_feedback(1.2, 0.2);  // object stops the jaws before close_angle
  EXPECT_TRUE(controller.is_target_reached(1.0));
  controller.set_target(false);  // grasp does not complete an open command
  EXPECT_FALSE(controller.is_target_reached(1.0));
}

TEST(ClockController, HoldsBeforeTargetAndNeverReportsUncommandedCompletion)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  EXPECT_DOUBLE_EQ(controller.compute_command(1.2), 1.2);
  EXPECT_FALSE(controller.is_target_reached(0.0));
  EXPECT_EQ(controller.get_current_slot(0.0), 12);
}

TEST(ClockController, TwelveToSevenUsesClockwiseShortestTurn)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  controller.set_target(7, 0.0);
  EXPECT_NEAR(controller.compute_command(0.0), -5.0 * kPi / 6.0, 1e-12);
  EXPECT_EQ(controller.get_current_slot(-5.0 * kPi / 6.0), 7);
}

TEST(ClockController, AllSlotPairsUseAtMostHalfATurnAndPreserveWinding)
{
  for (int from = 1; from <= 12; ++from) {
    for (int to = 1; to <= 12; ++to) {
      craft_control::ClockController controller;
      ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
      const double current = 4.0 * kPi + (from % 12) * kPi / 6.0;
      controller.set_target(to, current);
      const double target = controller.compute_command(current);
      EXPECT_LE(std::abs(target - current), kPi + 1e-12);
      EXPECT_EQ(controller.get_current_slot(target), to);
      EXPECT_TRUE(controller.is_target_reached(target));
    }
  }
}

TEST(ClockController, SlotBoundaryCrossingUsesShortTurnAndRepeatedTargetStaysFixed)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  const double current = 11.0 * kPi / 6.0;
  controller.set_target(1, current);
  const double target = 13.0 * kPi / 6.0;
  EXPECT_NEAR(controller.compute_command(current), target, 1e-12);
  controller.set_target(1, current - 4.0);
  EXPECT_NEAR(controller.compute_command(current), target, 1e-12);
}

TEST(ClockController, CompletionRequiresTargetAngleWithinTolerance)
{
  craft_control::ClockController controller;
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  controller.set_target(3, 0.0);
  EXPECT_FALSE(controller.is_target_reached(0.0));
  EXPECT_TRUE(controller.is_target_reached(kPi / 2.0 + 0.005));
  EXPECT_FALSE(controller.is_target_reached(kPi / 2.0 + 0.02));
  EXPECT_EQ(controller.get_current_slot(kPi / 12.0), 0);
  EXPECT_EQ(controller.get_current_slot(kPi / 2.0 + 0.005), 3);
}

TEST(ClockController, ValidatesSlotsAndTolerancesAndClearsTarget)
{
  craft_control::ClockController controller;
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.0));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, kPi / 12.0));
  EXPECT_FALSE(controller.configure(12, kPi / 6.0, 0.0, 1.0, std::numeric_limits<double>::infinity()));
  ASSERT_TRUE(controller.configure(12, kPi / 6.0, 0.0, 1.0, 0.01));
  EXPECT_TRUE(controller.is_valid_target(1));
  EXPECT_TRUE(controller.is_valid_target(12));
  EXPECT_FALSE(controller.is_valid_target(0));
  EXPECT_FALSE(controller.is_valid_target(13));
  controller.set_target(3, 0.0);
  controller.clear_target();
  EXPECT_DOUBLE_EQ(controller.compute_command(0.2), 0.2);  // hold current angle
  EXPECT_FALSE(controller.is_target_reached(kPi / 2.0));
}

TEST(ClockController, UsesConfiguredSlotGeometryZeroAndDirection)
{
  craft_control::ClockController controller;
  // Eight slots, 90 motor degrees per slot, reversed direction, and a shifted zero.
  ASSERT_TRUE(controller.configure(8, kPi / 2.0, 0.3, -1.0, 0.02));
  EXPECT_EQ(controller.get_current_slot(0.3), 8);
  controller.set_target(7, 0.3);
  const double target = 0.3 + kPi / 2.0;
  EXPECT_NEAR(controller.compute_command(0.3), target, 1e-12);
  EXPECT_EQ(controller.get_current_slot(target), 7);
  EXPECT_TRUE(controller.is_target_reached(target + 0.015));
  EXPECT_FALSE(controller.is_target_reached(target + 0.025));
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
  controller.set_target(path({{0, 0}, {1, 0}, {2, 0}}));
  ASSERT_TRUE(controller.can_track(0.2));
  const auto turn = controller.compute_command(0.2);
  EXPECT_DOUBLE_EQ(turn[0], 0.0);
  EXPECT_DOUBLE_EQ(turn[1], 0.0);
  craft_control::LongitudinalController longitudinal;
  longitudinal.configure(kRadius, 0.26);
  longitudinal.set_target(1.0);
  const auto limited = longitudinal.compute_command();
  EXPECT_NEAR(limited[0] * kRadius, 0.26, 1e-12);
  EXPECT_DOUBLE_EQ(limited[0], limited[1]);
  longitudinal.clear_target();
  EXPECT_DOUBLE_EQ(longitudinal.compute_command()[0], 0.0);
}

TEST(WheelControl, LeftAndRightPathsProduceOppositeDifferentialSpeeds)
{
  auto controller = lateral();
  controller.set_target(path({{0, 0}, {1, 1}, {2, 2}}));
  const auto left = controller.compute_command(0.2);
  EXPECT_NEAR(left[0], -expected_wheel_delta(controller, 0.2), 1e-9);
  EXPECT_LT(left[0], 0.0);
  EXPECT_NEAR(left[1], -left[0], 1e-12);
  controller.set_target(path({{0, 0}, {1, -1}, {2, -2}}));
  const auto right = controller.compute_command(0.2);
  EXPECT_NEAR(right[0], -left[0], 1e-12);
  EXPECT_NEAR(right[1], -left[1], 1e-12);
}

TEST(WheelControl, TransformsWorldPathUsingOdometryYaw)
{
  auto controller = lateral();
  controller.set_odom(odom(3.0, 4.0, kPi / 2.0));
  controller.set_target(path({{3, 4}, {2, 5}, {1, 6}}));
  const auto speeds = controller.compute_command(0.2);
  // Same 45-degree-left geometry as LeftAndRightPaths, seen from the rotated chassis.
  auto reference = lateral();
  reference.set_target(path({{0, 0}, {1, 1}, {2, 2}}));
  EXPECT_NEAR(speeds[0], reference.compute_command(0.2)[0], 1e-9);
  EXPECT_LT(speeds[0], 0.0);
  EXPECT_GT(speeds[1], 0.0);
}

TEST(WheelControl, ReversePathPursuesPointsBehindTheRobot)
{
  auto controller = lateral();
  controller.set_target(path({{0, 0}, {-1, 1}, {-2, 2}}));
  EXPECT_FALSE(controller.can_track(0.2));
  EXPECT_TRUE(controller.can_track(-0.2));
  const auto speeds = controller.compute_command(-0.2);
  EXPECT_GT(speeds[0], 0.0);
  EXPECT_LT(speeds[1], 0.0);
  const auto stopped = controller.compute_command(0.0);
  EXPECT_DOUBLE_EQ(stopped[0], 0.0);
  EXPECT_DOUBLE_EQ(stopped[1], 0.0);
}

TEST(WheelControl, GoalReachedNeedsSameFramePathEndpointWithinTolerance)
{
  auto controller = lateral();
  EXPECT_FALSE(controller.is_target_reached());  // no path
  controller.set_target(path({{0.5, 0.0}, {1.0, 0.0}}));
  EXPECT_FALSE(controller.is_target_reached());
  controller.set_odom(odom(0.98, 0.0));
  EXPECT_TRUE(controller.is_target_reached());
  EXPECT_FALSE(controller.can_track(0.1));
  auto other = path({{1.0, 0.0}});
  other.header.frame_id = "map";
  controller.set_target(other);
  EXPECT_FALSE(controller.is_target_reached());
}

TEST(WheelControl, MissingPathMismatchedFramesAndGoalArrivalDisableTracking)
{
  auto controller = lateral();
  EXPECT_FALSE(controller.can_track(0.2));
  auto target = path({{0, 0}, {1, 0}});
  target.header.frame_id = "map";
  controller.set_target(target);
  EXPECT_FALSE(controller.can_track(0.2));
  controller.set_target(path({{0, 0}, {1, 0}}));
  controller.set_odom(odom(0.99, 0.0));
  EXPECT_FALSE(controller.can_track(0.2));
}

TEST(WheelControl, ClearingPathOrOdometryStopsTracking)
{
  auto controller = lateral();
  controller.set_target(path({{0, 0}, {1, 0}}));
  ASSERT_TRUE(controller.can_track(0.2));
  controller.clear_target();
  EXPECT_FALSE(controller.can_track(0.2));
  EXPECT_FALSE(controller.is_target_reached());
  controller.set_target(path({{0, 0}, {1, 0}}));
  controller.clear_odom();
  EXPECT_FALSE(controller.can_track(0.2));
  controller.set_odom(odom());
  EXPECT_TRUE(controller.can_track(0.2));
}

TEST(WheelControl, ResamplesPathAlongCubicSplineAtFixedSpacing)
{
  auto controller = lateral();
  // Uneven input spacing on a straight line: output is evenly spaced and stays on the line.
  controller.set_target(path({{0, 0}, {0.05, 0}, {0.9, 0}, {1.0, 0}}));
  const auto & straight = controller.target_path();
  ASSERT_EQ(straight.poses.size(), 51u);
  EXPECT_EQ(straight.header.frame_id, "odom");
  for (std::size_t i = 0; i < straight.poses.size(); ++i) {
    EXPECT_NEAR(straight.poses[i].pose.position.x, 0.02 * static_cast<double>(i), 1e-9);
    EXPECT_NEAR(straight.poses[i].pose.position.y, 0.0, 1e-9);
  }

  // Curved input: spline passes through every input point and keeps the exact endpoints.
  controller.set_target(path({{0, 0}, {1, 0.5}, {2, 0}, {2, 0}, {3, -0.5}}));
  const auto & curve = controller.target_path();
  EXPECT_DOUBLE_EQ(curve.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(curve.poses.back().pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(curve.poses.back().pose.position.y, -0.5);
  // Equal arc-length spacing (chord ~= arc at 2 cm); only the final gap may be shorter.
  for (std::size_t i = 1; i + 1 < curve.poses.size(); ++i) {
    const auto & a = curve.poses[i - 1].pose.position;
    const auto & b = curve.poses[i].pose.position;
    EXPECT_NEAR(std::hypot(b.x - a.x, b.y - a.y), 0.02, 1e-4);
  }
  for (const auto & point : {std::pair{1.0, 0.5}, std::pair{2.0, 0.0}}) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & pose : curve.poses) {
      nearest = std::min(nearest, std::hypot(
        pose.pose.position.x - point.first, pose.pose.position.y - point.second));
    }
    EXPECT_LT(nearest, 0.02);
  }

  // A single distinct point is kept as is; clearing empties the tracked path.
  controller.set_target(path({{1, 0}, {1, 0}}));
  ASSERT_EQ(controller.target_path().poses.size(), 1u);
  controller.clear_target();
  EXPECT_TRUE(controller.target_path().poses.empty());
}

TEST(WheelControl, ValidatesVelocityPathAndOdometry)
{
  craft_control::LongitudinalController longitudinal;
  EXPECT_TRUE(longitudinal.is_valid_target(-0.1));
  EXPECT_FALSE(longitudinal.is_valid_target(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(longitudinal.is_valid_target(std::numeric_limits<double>::infinity()));

  auto controller = lateral();
  auto target = path({{0, 0}, {1, 0}});
  EXPECT_TRUE(controller.is_valid_target(target));
  EXPECT_FALSE(controller.is_valid_target(path({})));
  target.poses[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.is_valid_target(target));
  target = path({{0, 0}, {1, 0}});
  target.poses[1].header.frame_id = "map";
  EXPECT_FALSE(controller.is_valid_target(target));
  target.header.frame_id.clear();
  EXPECT_FALSE(controller.is_valid_target(target));

  EXPECT_TRUE(controller.is_valid_odom(odom()));
  auto invalid_odom = odom();
  invalid_odom.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(controller.is_valid_odom(invalid_odom));
  invalid_odom = odom(std::numeric_limits<double>::infinity());
  EXPECT_FALSE(controller.is_valid_odom(invalid_odom));
  invalid_odom = odom();
  invalid_odom.header.frame_id.clear();
  EXPECT_FALSE(controller.is_valid_odom(invalid_odom));
}
