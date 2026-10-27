#include "craft_planning/arm_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>

namespace craft_planning {
namespace {

class ArmPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    ArmPlannerConfig config;
    config.urdf = PLANNING_TEST_URDF;
    planner.configure(config);
    seed.name = planner.joint_names();
    seed.position = {0.0, 0.0, 0.0, 0.0, 0.0};
  }

  ArmPlanner planner;
  sensor_msgs::msg::JointState seed;
};

TEST_F(ArmPlannerTest, ForwardUsesTheUrdfTcpInBaseCoordinates) {
  const auto pose = planner.forward(seed);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(pose.pose.position.x, 0.12765, 1e-9);
  EXPECT_NEAR(pose.pose.position.y, 0.415477, 1e-9);  // TCP 0.118 (mid usable grip)
  EXPECT_NEAR(pose.pose.position.z, 0.306837, 1e-9);  // shoulder axis 114 mm
}

TEST_F(ArmPlannerTest, SolvesAReachablePoseFromADifferentSeed) {
  auto desired = seed;
  desired.position = {0.005, 0.2, -0.4, 0.3, 0.1};
  const auto goal = planner.forward(desired);

  const auto result = planner.plan(goal, seed);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.error.empty());
  EXPECT_EQ(result.joints.name, planner.joint_names());
  ASSERT_EQ(result.joints.position.size(), 5u);
  const auto reached = planner.forward(result.joints);
  const double dx = reached.pose.position.x - goal.pose.position.x;
  const double dy = reached.pose.position.y - goal.pose.position.y;
  const double dz = reached.pose.position.z - goal.pose.position.z;
  EXPECT_LT(std::sqrt(dx * dx + dy * dy + dz * dz), 0.0001);
  const auto &a = reached.pose.orientation;
  const auto &b = goal.pose.orientation;
  const double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
  EXPECT_LT(2.0 * std::acos(std::clamp(dot, 0.0, 1.0)), 0.001);
}

TEST_F(ArmPlannerTest, PrefersTheBranchWithTheLeastElbowLoad) {
  // Two in-limit branches reach this pose; the seed sits on the one that holds the distal links far
  // out from the elbow (lever 0.516 m vs 0.007 m), so the planner must leave it for the other.
  auto heavy = seed;
  heavy.position = {0.0, 0.2251, 2.6189, -0.2694, 0.1};
  const auto result = planner.plan(planner.forward(heavy), heavy);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.joints.position[1], 1.2142, 1e-3);
  EXPECT_NEAR(result.joints.position[2], -0.7826, 1e-3);
  EXPECT_NEAR(result.joints.position[3], 2.1430, 1e-3);
}

TEST_F(ArmPlannerTest, AcceptsReorderedNamedSeedAndReturnsControlOrder) {
  const auto goal = planner.forward(seed);
  std::reverse(seed.name.begin(), seed.name.end());
  std::reverse(seed.position.begin(), seed.position.end());
  const auto result = planner.plan(goal, seed);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.joints.name, planner.joint_names());
}

TEST_F(ArmPlannerTest, ReturnsFailureForAnUnreachablePose) {
  auto goal = planner.forward(seed);
  goal.pose.position.x = 10.0;
  const auto result = planner.plan(goal, seed);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
  EXPECT_TRUE(result.joints.position.empty());
}

TEST_F(ArmPlannerTest, RejectsGoalFramesThatNeedAnExternalTransform) {
  auto goal = planner.forward(seed);
  goal.header.frame_id = "camera_color_optical_frame";
  const auto result = planner.plan(goal, seed);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST_F(ArmPlannerTest, RejectsZeroAndNonFiniteQuaternions) {
  auto goal = planner.forward(seed);
  goal.pose.orientation = geometry_msgs::msg::Quaternion();
  goal.pose.orientation.w = 0.0;
  EXPECT_FALSE(planner.plan(goal, seed).ok);
  goal.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planner.plan(goal, seed).ok);
}

TEST_F(ArmPlannerTest, NormalizesANonzeroQuaternion) {
  auto goal = planner.forward(seed);
  goal.pose.orientation.w = 2.0;
  EXPECT_TRUE(planner.plan(goal, seed).ok);
}

TEST_F(ArmPlannerTest, RejectsNonFiniteGoalPositions) {
  auto goal = planner.forward(seed);
  goal.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(planner.plan(goal, seed).ok);
}

TEST_F(ArmPlannerTest, RejectsMissingUnknownAndDuplicateSeedNames) {
  const auto goal = planner.forward(seed);
  auto invalid = seed;
  invalid.name.pop_back();
  invalid.position.pop_back();
  EXPECT_FALSE(planner.plan(goal, invalid).ok);
  invalid = seed;
  invalid.name[0] = "unknown_joint";
  EXPECT_FALSE(planner.plan(goal, invalid).ok);
  invalid = seed;
  invalid.name[0] = invalid.name[1];
  EXPECT_FALSE(planner.plan(goal, invalid).ok);
}

TEST_F(ArmPlannerTest, RejectsSeedValuesOutsideUrdfLimitsOrNonFinite) {
  const auto goal = planner.forward(seed);
  seed.position[0] = 1.0;
  EXPECT_FALSE(planner.plan(goal, seed).ok);
  EXPECT_THROW(planner.forward(seed), std::invalid_argument);
  seed.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planner.plan(goal, seed).ok);
}

TEST_F(ArmPlannerTest, RepeatedRequestsAreDeterministic) {
  auto desired = seed;
  desired.position = {0.005, 0.2, -0.4, 0.3, 0.1};
  const auto goal = planner.forward(desired);
  const auto first = planner.plan(goal, seed);
  const auto second = planner.plan(goal, seed);
  ASSERT_TRUE(first.ok) << first.error;
  ASSERT_TRUE(second.ok) << second.error;
  EXPECT_EQ(first.joints.position, second.joints.position);
}

TEST_F(ArmPlannerTest, ConcurrentRequestsHaveIndependentScratchState) {
  auto desired = seed;
  desired.position = {0.005, 0.2, -0.4, 0.3, 0.1};
  const auto goal = planner.forward(desired);
  auto first = std::async(std::launch::async, [&] { return planner.plan(goal, seed); });
  auto second = std::async(std::launch::async, [&] { return planner.plan(goal, seed); });
  const auto a = first.get();
  const auto b = second.get();
  ASSERT_TRUE(a.ok) << a.error;
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(a.joints.position, b.joints.position);
}

TEST(ArmPlannerConfiguration, RejectsUnconfiguredUse) {
  ArmPlanner planner;
  EXPECT_FALSE(planner.plan(geometry_msgs::msg::PoseStamped(),
    sensor_msgs::msg::JointState()).ok);
  EXPECT_THROW(planner.forward(sensor_msgs::msg::JointState()), std::logic_error);
}

TEST(ArmPlannerConfiguration, RejectsInvalidSettingsAndUnexpectedChains) {
  ArmPlanner planner;
  ArmPlannerConfig config;
  config.urdf = PLANNING_TEST_URDF;
  config.position_tolerance = 0.0;
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
  config.position_tolerance = 0.0001;
  config.tip_frame = "shoulder_link";
  EXPECT_THROW(planner.configure(config), std::invalid_argument);
}

}  // namespace
}  // namespace craft_planning
