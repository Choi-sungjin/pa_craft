#include "craft_planning/grasp_region.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using craft_planning::GraspConfirm;
using craft_planning::GraspRegionConfig;
using craft_planning::PuckInBase;
using craft_planning::check_graspable;
constexpr double kPi = 3.14159265358979323846;

GraspRegionConfig config()
{
  GraspRegionConfig c;
  c.lying = {0.20, 0.30, -0.03, 0.03, 0.00, 0.10};
  c.standing = {0.22, 0.28, -0.02, 0.02, 0.02, 0.12};
  c.margin = 0.005;
  c.axis_tolerance = 10 * kPi / 180;
  c.confirm_count = 3;
  return c;
}

PuckInBase puck(double x, double y, double z, double axis_deg, bool standing = false)
{
  PuckInBase p;
  p.center.x = x; p.center.y = y; p.center.z = z;
  p.axis.x = std::cos(axis_deg * kPi / 180); p.axis.y = std::sin(axis_deg * kPi / 180);
  if (standing) {p.axis.x = p.axis.y = 0; p.axis.z = 1;}
  p.standing = standing;
  return p;
}
}  // namespace

TEST(GraspRegion, AlignedLyingPuckInsideTheRegionIsGraspable)
{
  const auto check = check_graspable(config(), puck(0.25, 0.0, 0.05, 3));
  EXPECT_TRUE(check.graspable) << check.reason;
  // The long axis is 180 degree symmetric.
  EXPECT_TRUE(check_graspable(config(), puck(0.25, 0.0, 0.05, 177)).graspable);
  EXPECT_TRUE(check_graspable(config(), puck(0.25, 0.0, 0.05, -177)).graspable);
}

TEST(GraspRegion, AlignmentIsCheckedBeforeTheRegion)
{
  // Inside the region but 30 degrees off: rejected for alignment, the region is not even judged.
  auto check = check_graspable(config(), puck(0.25, 0.0, 0.05, 30));
  EXPECT_FALSE(check.graspable);
  EXPECT_EQ(check.reason.rfind("not aligned", 0), 0u) << check.reason;
  // Outside the region and off axis: still reported as not aligned first.
  check = check_graspable(config(), puck(0.50, 0.2, 0.05, 30));
  EXPECT_EQ(check.reason.rfind("not aligned", 0), 0u) << check.reason;
  // Aligned but outside: now the region decides.
  check = check_graspable(config(), puck(0.50, 0.0, 0.05, 0));
  EXPECT_FALSE(check.graspable);
  EXPECT_NE(check.reason.find("aligned, lying centre x"), std::string::npos) << check.reason;
}

TEST(GraspRegion, MarginShrinksEveryFace)
{
  EXPECT_TRUE(check_graspable(config(), puck(0.294, 0.0, 0.05, 0)).graspable);
  EXPECT_FALSE(check_graspable(config(), puck(0.296, 0.0, 0.05, 0)).graspable);
  EXPECT_FALSE(check_graspable(config(), puck(0.25, 0.026, 0.05, 0)).graspable);
  EXPECT_FALSE(check_graspable(config(), puck(0.25, 0.0, 0.003, 0)).graspable);
}

TEST(GraspRegion, StandingPuckUsesItsOwnRegionAndAnyApproachDirection)
{
  EXPECT_TRUE(check_graspable(config(), puck(0.25, 0.0, 0.05, 0, true)).graspable);
  // Inside the lying region but outside the narrower standing one.
  EXPECT_FALSE(check_graspable(config(), puck(0.21, 0.0, 0.05, 0, true)).graspable);
  EXPECT_TRUE(check_graspable(config(), puck(0.21, 0.0, 0.05, 0)).graspable);
}

TEST(GraspRegion, UnmeasuredRegionIsNeverGraspable)
{
  auto c = config();
  c.standing = {};
  const auto check = check_graspable(c, puck(0.25, 0.0, 0.05, 0, true));
  EXPECT_FALSE(check.graspable);
  EXPECT_EQ(check.reason, "standing grasp region not measured");
  EXPECT_TRUE(check_graspable(c, puck(0.25, 0.0, 0.05, 0)).graspable);
}

TEST(GraspRegion, VerticalAxisOnALyingPuckAndNonFiniteCentreAreRejected)
{
  auto p = puck(0.25, 0.0, 0.05, 0);
  p.axis.x = p.axis.y = 0; p.axis.z = 1;
  EXPECT_FALSE(check_graspable(config(), p).graspable);
  p = puck(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.05, 0);
  EXPECT_FALSE(check_graspable(config(), p).graspable);
}

TEST(GraspRegion, ConfirmNeedsConsecutiveGraspableChecks)
{
  GraspConfirm confirm;
  const auto yes = check_graspable(config(), puck(0.25, 0.0, 0.05, 0));
  const auto no = check_graspable(config(), puck(0.25, 0.0, 0.05, 30));
  EXPECT_FALSE(confirm.update(yes, 3));
  EXPECT_FALSE(confirm.update(yes, 3));
  EXPECT_FALSE(confirm.update(no, 3));  // restarts
  EXPECT_FALSE(confirm.update(yes, 3));
  EXPECT_FALSE(confirm.update(yes, 3));
  EXPECT_TRUE(confirm.update(yes, 3));
  confirm.reset();
  EXPECT_EQ(confirm.count(), 0);
}

namespace
{
GraspRegionConfig stop_config()
{
  auto c = config();  // standing region x 0.22..0.28 (centre in base_link)
  c.front_face_x = 0.15; c.front_face_width = 0.30; c.good_plane = 0.10; c.min_box_gap = 0.03;
  c.w_box = 0.003; c.w_grasp = 1.0;
  return c;
}
// Aligned box straight ahead: d_short = gap - a, s = s0 - a. Newton on J'(a) = 0.
double newton_advance(const GraspRegionConfig & c, double gap, double s0, double sigma)
{
  double a = s0 - c.good_plane;
  for (int i = 0; i < 100; ++i) {
    const double d = gap - a - c.min_box_gap, e = s0 - a - c.good_plane;
    const double g = c.w_box / (d * d) - 2 * c.w_grasp * e / (sigma * sigma);
    const double h = 2 * c.w_box / (d * d * d) + 2 * c.w_grasp / (sigma * sigma);
    a -= g / h;
  }
  return a;
}
}  // namespace

TEST(StopPlan, AlignedBoxMatchesTheAnalyticOptimum)
{
  const auto c = stop_config();
  // Box face 0.25 m ahead of the front face, puck 4 cm inside the face, standing (any direction).
  craft_planning::BoxInBase box{c.front_face_x + 0.25 + 0.1025, 0.0, 0.0};
  const auto p = puck(c.front_face_x + 0.25 + 0.04, 0.0, 0.05, 0, true);
  const auto plan = craft_planning::plan_stop(c, p, box);
  ASSERT_TRUE(plan.feasible) << plan.reason;
  EXPECT_NEAR(plan.advance, newton_advance(c, 0.25, 0.29, 0.03), 1e-5);
  EXPECT_NEAR(plan.box_gap, 0.25 - plan.advance, 1e-9);
  // The box term pulls the stop back from the good plane (puck would sit at s = 0.10 without it).
  EXPECT_GT(plan.puck_s, c.good_plane);
  EXPECT_TRUE(check_graspable(c, puck(p.center.x - plan.advance, 0.0, 0.05, 0, true)).graspable);
}

TEST(StopPlan, MinimumGapIsHardAndARotatedBoxCornerCountsFirst)
{
  auto c = stop_config();
  c.w_box = 0.0;  // grasp term alone would put the puck on the good plane...
  c.min_box_gap = 0.08;
  craft_planning::BoxInBase box{c.front_face_x + 0.25 + 0.1025, 0.0, 0.0};
  const auto p = puck(c.front_face_x + 0.25 + 0.04, 0.0, 0.05, 0, true);
  auto plan = craft_planning::plan_stop(c, p, box);
  ASSERT_TRUE(plan.feasible) << plan.reason;
  EXPECT_NEAR(plan.box_gap, 0.08, 1e-3);  // ...but the safety gap stops it just outside 8 cm
  EXPECT_GT(plan.box_gap, 0.08);
  c = stop_config();
  const auto aligned = craft_planning::plan_stop(c, p, box);
  box.yaw = 30 * kPi / 180;  // a corner now sticks out towards the robot
  plan = craft_planning::plan_stop(c, p, box);
  ASSERT_TRUE(plan.feasible) << plan.reason;
  EXPECT_LT(plan.advance, aligned.advance);
  EXPECT_LT(craft_planning::front_face_to_box(c, box), 0.25);
}

TEST(StopPlan, InfeasibleCasesExplainWhy)
{
  const auto c = stop_config();
  craft_planning::BoxInBase box{c.front_face_x + 0.25 + 0.1025, 0.0, 0.0};
  // Lying puck off axis: alignment fails before anything else.
  auto plan = craft_planning::plan_stop(c, puck(0.45, 0.0, 0.05, 30), box);
  EXPECT_FALSE(plan.feasible);
  EXPECT_EQ(plan.reason.rfind("not aligned", 0), 0u) << plan.reason;
  // Sideways out of the region: advancing cannot fix y.
  plan = craft_planning::plan_stop(c, puck(0.45, 0.10, 0.05, 0, true), box);
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.reason.find("centre y"), std::string::npos) << plan.reason;
  // Puck deep inside the box: the region needs the front face closer than min_box_gap.
  plan = craft_planning::plan_stop(c, puck(c.front_face_x + 0.25 + 0.15, 0.0, 0.05, 0, true), box);
  EXPECT_FALSE(plan.feasible);
  // Not measured yet.
  plan = craft_planning::plan_stop(config(), puck(0.45, 0.0, 0.05, 0, true), box);
  EXPECT_FALSE(plan.feasible);
  EXPECT_NE(plan.reason.find("not configured"), std::string::npos);
}

TEST(GraspRegion, ValidationRejectsBadConfiguration)
{
  auto c = config();
  EXPECT_NO_THROW(craft_planning::validate(c));
  c.margin = -0.01;
  EXPECT_THROW(craft_planning::validate(c), std::invalid_argument);
  c = config(); c.confirm_count = 0;
  EXPECT_THROW(craft_planning::validate(c), std::invalid_argument);
  c = config(); c.lying[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(craft_planning::validate(c), std::invalid_argument);
  c = config(); c.axis_tolerance = 2.0;
  EXPECT_THROW(craft_planning::validate(c), std::invalid_argument);
  c = config(); c.w_box = -1;
  EXPECT_THROW(craft_planning::validate(c), std::invalid_argument);
}
