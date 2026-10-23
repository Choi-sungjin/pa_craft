#include "craft_planning/planning_config.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace craft_planning {
namespace {

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
    - {id: grasp, type: tcp, pose: [0, 0, 0.02, 0, 0, 0, 1], source: d435_knob, class_id: 0}
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
  EXPECT_EQ(steps[1].source, "d435_knob");
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
  for (const auto &source : {"fixed", "d435_knob", "d435_puck", "object_pose", "object_detection"}) {
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
      "missions: {M: [{id: a, type: tcp, source: d435_knob}]}",
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
