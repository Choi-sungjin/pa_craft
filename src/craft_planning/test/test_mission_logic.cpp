#include "craft_planning/mission_logic.hpp"
#include <gtest/gtest.h>
#include <set>
using namespace craft_planning::mission;

// n pucks: ids from first, colour and shape as given
static void add(std::vector<Puck> &pucks, int n, PuckColor color, PuckShape shape) {
  for (int i = 0; i < n; ++i) pucks.push_back({static_cast<int>(pucks.size()), color, shape});
}
static const auto R = PuckColor::Red, G = PuckColor::Green;
static const auto CUB = PuckShape::Cuboid, CYL = PuckShape::Cylinder;

TEST(MissionLogic, BarcodeOrder) {
  EXPECT_EQ(order_from_barcode("P2-01"), SortOrder::Ascending);
  EXPECT_EQ(order_from_barcode("P2-03"), SortOrder::Ascending);
  EXPECT_EQ(order_from_barcode("P2-02"), SortOrder::Descending);
  EXPECT_EQ(order_from_barcode("P2-04"), SortOrder::Descending);
  EXPECT_FALSE(order_from_barcode("P2-05"));
  EXPECT_FALSE(order_from_barcode(""));
}

TEST(MissionLogic, SignalCheck) {
  EXPECT_TRUE(check_signal({5, 3, 4}).empty());
  EXPECT_FALSE(check_signal({5, 3, 3}).empty());   // 11 blinks
  EXPECT_FALSE(check_signal({7, 1, 4}).empty());   // 7 of one colour
  EXPECT_FALSE(check_signal({6, 0, 6}).empty());   // a colour missing
  const auto c = count_sequence({0, 2, 0, 0, 2, 0, 2, 2, 1, 2, 1, 2});  // the public example R G R R G R G G Y G Y G
  ASSERT_TRUE(c);
  EXPECT_EQ(c->red, 4); EXPECT_EQ(c->yellow, 2); EXPECT_EQ(c->green, 6);
  EXPECT_FALSE(count_sequence({0, 3}));
}

TEST(MissionLogic, LocationGroupsSortedWithTieOrder) {
  using A = std::array<Group, 3>;
  // Q1: R4 Y2 G6, ascending -> Y, R, G
  EXPECT_EQ(location_groups({4, 2, 6}, SortOrder::Ascending), (A{Group::Yellow, Group::Red, Group::Green}));
  EXPECT_EQ(location_groups({4, 2, 6}, SortOrder::Descending), (A{Group::Green, Group::Red, Group::Yellow}));
  // ties keep R -> Y -> G in both directions
  EXPECT_EQ(location_groups({4, 4, 4}, SortOrder::Ascending), (A{Group::Red, Group::Yellow, Group::Green}));
  EXPECT_EQ(location_groups({4, 4, 4}, SortOrder::Descending), (A{Group::Red, Group::Yellow, Group::Green}));
  EXPECT_EQ(location_groups({5, 2, 5}, SortOrder::Descending), (A{Group::Red, Group::Green, Group::Yellow}));
  EXPECT_EQ(location_groups({3, 6, 3}, SortOrder::Ascending), (A{Group::Red, Group::Green, Group::Yellow}));
}

TEST(MissionLogic, AllocationBriefExample) {
  // 8 red (3 cylinders among them), 4 green, R5 Y3 G4: Y takes the three red cylinders.
  std::vector<Puck> pucks;
  add(pucks, 5, R, CUB); add(pucks, 3, R, CYL); add(pucks, 2, G, CUB); add(pucks, 2, G, CYL);
  const auto a = allocate(pucks, {5, 3, 4});
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.yellow_shape, CUB);  // 5 red cuboids cover Y too: cuboid comes first
  EXPECT_EQ(a.of(Group::Yellow), (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(a.of(Group::Red).size(), 5u); EXPECT_EQ(a.of(Group::Green).size(), 4u);
}

TEST(MissionLogic, AllocationUsesEveryPuckOnceAndKeepsTheRules) {
  std::vector<Puck> pucks;
  add(pucks, 2, R, CUB); add(pucks, 4, R, CYL); add(pucks, 5, G, CUB); add(pucks, 1, G, CYL);
  // R3 Y4 G5: Y = 3 red + 1 green of one shape -> only cylinders (4 red, 1 green)
  const auto a = allocate(pucks, {3, 4, 5});
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.yellow_shape, CYL);
  std::set<int> all;
  for (Group g : {Group::Red, Group::Yellow, Group::Green}) for (int id : a.of(g)) EXPECT_TRUE(all.insert(id).second);
  EXPECT_EQ(all.size(), 12u);
  for (int id : a.of(Group::Red)) EXPECT_EQ(pucks[id].color, R);
  for (int id : a.of(Group::Green)) EXPECT_EQ(pucks[id].color, G);
  for (int id : a.of(Group::Yellow)) EXPECT_EQ(pucks[id].shape, CYL);
  EXPECT_EQ(a.of(Group::Red).size(), 3u); EXPECT_EQ(a.of(Group::Yellow).size(), 4u);
}

TEST(MissionLogic, AllocationImpossibleCasesAreErrors) {
  std::vector<Puck> pucks;
  add(pucks, 3, R, CUB); add(pucks, 3, R, CYL); add(pucks, 3, G, CUB); add(pucks, 3, G, CYL);
  // R1 needs 5 red in Y, G6 needs more greens than there are
  EXPECT_FALSE(allocate(pucks, {1, 5, 6}).ok);
  // R2 Y6 G4: Y = 4 red + 2 green of one shape, but only 3 red of each shape
  const auto a = allocate(pucks, {2, 6, 4});
  EXPECT_FALSE(a.ok); EXPECT_FALSE(a.error.empty());
  EXPECT_FALSE(allocate(pucks, {4, 4, 3}).ok);  // 11 blinks
  pucks.pop_back();
  EXPECT_FALSE(allocate(pucks, {4, 4, 4}).ok);  // 11 pucks
  pucks.push_back(pucks.front());
  EXPECT_FALSE(allocate(pucks, {4, 4, 4}).ok);  // an id twice
}

TEST(MissionLogic, DiceSpots) {
  for (int face = 1; face <= 6; ++face) {
    const auto spots = dice_spots(face);
    EXPECT_EQ(spots.size(), static_cast<std::size_t>(face));
    std::set<std::array<int, 2>> distinct(spots.begin(), spots.end());
    EXPECT_EQ(distinct.size(), spots.size());
    for (const auto &s : spots) {EXPECT_LE(std::abs(s[0]), 1); EXPECT_LE(std::abs(s[1]), 1);}
    // centre only on the odd faces; every face is point symmetric
    EXPECT_EQ(distinct.count({0, 0}), static_cast<std::size_t>(face % 2));
    for (const auto &s : spots) EXPECT_TRUE(distinct.count({-s[0], -s[1]}));
  }
  EXPECT_THROW(dice_spots(0), std::invalid_argument);
  EXPECT_THROW(dice_spots(7), std::invalid_argument);
}
