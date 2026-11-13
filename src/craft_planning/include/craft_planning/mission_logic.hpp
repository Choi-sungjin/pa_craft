#pragma once
// Dice memory mission (발제 2026-11-04, v5): the decisions that need no robot.
//   barcode -> sort order, blink counts -> which group goes to L1/L2/L3, 12 pucks -> which puck goes where,
//   dice face -> grid spots. Pure functions; mission_fsm.hpp sequences them, mission_master.cpp runs them.
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace craft_planning::mission {

enum class Group {Red, Yellow, Green};       // signal colour = the group a location gets
enum class PuckColor {Red, Green};           // PuckSurvey.RED / GREEN
enum class PuckShape {Cuboid, Cylinder};     // PuckSurvey.CUBOID / CYLINDER
enum class SortOrder {Ascending, Descending};

const char *name(Group g);
const char *name(PuckColor c);
const char *name(PuckShape s);
const char *name(SortOrder o);

// P2-01 / P2-03 = ascending (fewest blinks first), P2-02 / P2-04 = descending. Anything else: not registered.
std::optional<SortOrder> order_from_barcode(const std::string &code);

struct SignalCounts {
  int red = 0, yellow = 0, green = 0;
  int of(Group g) const {return g == Group::Red ? red : g == Group::Yellow ? yellow : green;}
};

// 12 blinks, every colour 1..6 times. Empty = valid, else why not.
std::string check_signal(const SignalCounts &counts);
// Counts of a blink sequence (TrafficLight.sequence: 0 red, 1 yellow, 2 green). Empty on any other value.
std::optional<SignalCounts> count_sequence(const std::vector<uint8_t> &sequence);

// The groups in L1, L2, L3 order: sorted by count, equal counts keep R -> Y -> G.
std::array<Group, 3> location_groups(const SignalCounts &counts, SortOrder order);

struct Puck {
  int id = -1;
  PuckColor color = PuckColor::Red;
  PuckShape shape = PuckShape::Cuboid;
};

struct Allocation {
  bool ok = false;
  std::string error;                  // why no allocation exists (ok = false)
  PuckShape yellow_shape = PuckShape::Cuboid;
  std::array<std::vector<int>, 3> ids;  // puck ids of the Red, Yellow, Green group, ascending
  const std::vector<int> &of(Group g) const {return ids[static_cast<int>(g)];}
};

// Every puck once: R = red pucks, G = green pucks, Y = one shape (either colour). The whole allocation is decided
// before anything is picked, so R/G never take the pucks Y needs. Lowest ids first; cuboid before cylinder when
// both shapes work for Y. Needs exactly 12 distinct pucks and a valid signal.
Allocation allocate(const std::vector<Puck> &pucks, const SignalCounts &counts);

// Dice face 1..6 on the 3 x 3 grid: [x, y] in grid steps (-1, 0, +1; 60 mm), map +x right, +y up. 2 and 3 run
// top-left to bottom-right, 6 is the left and the right column (a quarter turn of the whole face does not matter:
// user, 2026-11-12). Throws invalid_argument outside 1..6.
std::vector<std::array<int, 2>> dice_spots(int face);

}  // namespace craft_planning::mission
