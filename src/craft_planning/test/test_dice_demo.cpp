#include "craft_planning/dice_demo.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace craft_planning::dice;

namespace {

ControlParams control() {
  return load_control(std::string(CRAFT_PLANNING_SOURCE_DIR) + "/../craft_control/config/control.yaml");
}
DiceParams dice() {return load_dice(std::string(CRAFT_PLANNING_SOURCE_DIR) + "/config/dice_demo.yaml");}

}  // namespace

TEST(DiceDemo, PlaceholderCompiles) {
  const auto p = dice();
  const Kinematics k(PLANNING_TEST_URDF, control(), p);
  EXPECT_EQ(dice_spots(6, p).size(), 6u);
}
