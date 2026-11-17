#include "craft_planning/planning_config.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace craft_planning {
namespace {

TEST(BoxDockGoal, LeftShiftForEveryHeadingWithoutChangingForwardGap) {
  const BoxTarget box{1.476, 1.159, M_PI / 2, 0.205, 0.233};  // S1, 213 mm front + 20 mm gap
  for (const double normal : {0.0, M_PI / 2, M_PI, -M_PI / 2, 0.37}) {
    const auto before = box_dock_goal(box, normal, 0.0);
    const auto after = box_dock_goal(box, normal, 0.0015);
    const double heading = after[2], dx = after[0] - before[0], dy = after[1] - before[1];
    EXPECT_NEAR(-std::sin(heading) * dx + std::cos(heading) * dy, 0.0015, 1e-14);
    EXPECT_NEAR(std::cos(heading) * dx + std::sin(heading) * dy, 0.0, 1e-14);
    EXPECT_DOUBLE_EQ(before[2], after[2]);
    // Repeated LiDAR/camera updates use the observation again, never the already shifted goal.
    EXPECT_EQ(after, box_dock_goal(box, normal, 0.0015));
    const auto right = box_dock_goal(box, normal, -0.0015);
    EXPECT_NEAR(right[0] - before[0], -dx, 1e-14);
    EXPECT_NEAR(right[1] - before[1], -dy, 1e-14);
  }
  const auto before = box_dock_goal(box, M_PI / 2, 0.0);
  const auto after = box_dock_goal(box, M_PI / 2, 0.0015);
  EXPECT_NEAR(before[0], 1.476, 1e-14);
  EXPECT_NEAR(after[0], 1.4775, 1e-14);  // heading -90 deg: robot left is map +x
  EXPECT_NEAR(before[1], 1.4945, 1e-14);
  EXPECT_NEAR(after[1], before[1], 1e-14);
}

std::map<std::string, std::vector<PlanningStep>> parse(const std::string &yaml) {
  char name[] = "/tmp/craft-missions-XXXXXX";
  const int fd = mkstemp(name);
  if (fd < 0) throw std::runtime_error("cannot create mission test file");
  close(fd);
  struct Cleanup {
    const char *path;
    ~Cleanup() {std::remove(path);}
  } cleanup{name};
  std::ofstream file(name);
  file << yaml;
  file.close();
  return parse_missions(name, "map", "base_link");
}

TEST(PlanningMissions, AcceptsAnExplicitlyEmptyMissionSet) {
  EXPECT_TRUE(parse("missions: {}\n").empty());
}

TEST(PlanningMissions, LoadsOrderedStepsForAllSupportedKinds) {
  const auto missions = parse(R"(
missions:
  MAIN_MISSION:
    - {id: approach, type: navigate, pose: [1, 2, 0, 0, 0, 0, 1], frame: map}
    - {id: grasp, type: tcp, pose: [0, 0, 0.02, 0, 0, 0, 1], source: d435_puck, class_id: 0}
    - {id: close, type: gripper, close: true}
    - {id: slot, type: clock, slot: 12}
    - {id: settle, type: wait, duration_s: 0.5}
)");
  ASSERT_EQ(missions.size(), 1u);
  const auto &steps = missions.at("MAIN_MISSION");
  ASSERT_EQ(steps.size(), 5u);
  EXPECT_EQ(steps[0].kind, StepKind::Navigate);
  EXPECT_EQ(steps[0].goal.header.frame_id, "map");
  EXPECT_EQ(steps[0].goal.pose.position.x, 1.0);
  EXPECT_EQ(steps[1].kind, StepKind::Tcp);
  EXPECT_EQ(steps[1].goal.header.frame_id, "base_link");
  EXPECT_EQ(steps[1].source, "d435_puck");
  EXPECT_EQ(steps[1].class_id, 0);
  EXPECT_EQ(steps[1].goal.pose.position.z, 0.02);
  EXPECT_EQ(steps[2].kind, StepKind::Gripper);
  EXPECT_TRUE(steps[2].close);
  EXPECT_EQ(steps[3].kind, StepKind::Clock);
  EXPECT_EQ(steps[3].slot, 12);
  EXPECT_EQ(steps[4].kind, StepKind::Wait);
  EXPECT_EQ(steps[4].duration_s, 0.5);
}

TEST(PlanningMissions, AcceptsEveryObservationSourceWithAnExplicitOffsetAndOrientation) {
  for (const auto &source : {"fixed", "d435_puck", "object_pose", "object_detection"}) {
    const auto missions = parse(std::string("missions: {M: [{id: target, type: tcp, source: ") +
      source + ", pose: [0, 0, 0, 0, 0, 0, 2]}]}");
    const auto &step = missions.at("M").front();
    EXPECT_EQ(step.source, source);
    EXPECT_DOUBLE_EQ(step.goal.pose.orientation.w, 1.0);
  }
}

TEST(PlanningMissions, RejectsUnknownTypesSourcesAndFields) {
  for (const auto &text : {
      "missions: {M: [{id: a, type: fly}]}",
      "missions: {M: [{id: a, type: tcp, source: camera, pose: [0,0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: tcp, source: d435_knob, pose: [0,0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: wait, duration_s: 1, typo: 2}]}",
      "missions: {M: [{id: a, type: gripper, close: true, pose: [0,0,0,0,0,0,1]}]}",
      "missions: {}, typo: true"})
  {
    EXPECT_THROW(parse(text), std::invalid_argument) << text;
  }
}

TEST(PlanningMissions, RejectsMissingOrMalformedRequiredFields) {
  for (const auto &text : {
      "{}", "missions: []", "missions: {M: {id: a}}",
      "missions: {M: [{type: wait, duration_s: 1}]}",
      "missions: {M: [{id: a, type: tcp, source: d435_puck}]}",
      "missions: {M: [{id: a, type: gripper}]}",
      "missions: {M: [{id: a, type: gripper, close: [true]}]}",
      "missions: {M: [{id: a, type: gripper, close: 'true'}]}",
      "missions: {M: [{id: a, type: clock, slot: 1.5}]}",
      "missions: {M: [{id: a, type: clock, slot: '1'}]}",
      "missions: {M: [{id: 1, type: wait, duration_s: 0}]}",
      "missions: {M: [{id: a, type: tcp, pose: ['0',0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: wait, duration_s: hello}]}"})
  {
    EXPECT_THROW(parse(text), std::invalid_argument) << text;
  }
}

TEST(PlanningMissions, RejectsWrongFramesInvalidPosesAndClassFilters) {
  for (const auto &text : {
      "missions: {M: [{id: a, type: navigate, frame: odom, pose: [0,0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: tcp, frame: map, pose: [0,0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: tcp, pose: [0,0,0]}]}",
      "missions: {M: [{id: a, type: tcp, pose: [0,0,0,0,0,0,0]}]}",
      "missions: {M: [{id: a, type: tcp, pose: [.nan,0,0,0,0,0,1]}]}",
      "missions: {M: [{id: a, type: tcp, pose: [0,0,0,0,0,.inf,1]}]}",
      "missions: {M: [{id: a, type: tcp, pose: [0,0,0,0,0,0,1], class_id: -2}]}"})
  {
    EXPECT_THROW(parse(text), std::invalid_argument) << text;
  }
}

TEST(PlanningMissions, RejectsOutOfRangeSlotsAndWaits) {
  for (const auto &text : {
      "missions: {M: [{id: a, type: clock, slot: 0}]}",
      "missions: {M: [{id: a, type: clock, slot: 13}]}",
      "missions: {M: [{id: a, type: wait, duration_s: -1}]}",
      "missions: {M: [{id: a, type: wait, duration_s: .inf}]}"})
  {
    EXPECT_THROW(parse(text), std::invalid_argument) << text;
  }
  EXPECT_EQ(parse("missions: {M: [{id: a, type: wait, duration_s: 0}]}")
    .at("M").front().duration_s, 0.0);
}

TEST(PlanningMissions, RejectsDuplicateMissionNamesAndStepIdentifiers) {
  EXPECT_THROW(parse("missions: {M: [], M: []}"), std::invalid_argument);
  EXPECT_THROW(parse(R"(missions: {M: [
    {id: duplicate, type: wait, duration_s: 0},
    {id: duplicate, type: wait, duration_s: 1}]})"), std::invalid_argument);
  EXPECT_THROW(parse("missions: {M: [{id: a, id: b, type: wait, duration_s: 0}]}"),
    std::invalid_argument);
}

TEST(PlanningMissions, ReportsMissingAndMalformedFilesAsConfigurationErrors) {
  EXPECT_THROW(parse_missions("/does/not/exist/craft_missions.yaml", "map", "base_link"),
    std::invalid_argument);
  EXPECT_THROW(parse("missions: {"), std::invalid_argument);
}

}  // namespace
}  // namespace craft_planning
