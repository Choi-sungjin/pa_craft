#include "craft_planning/mission_logic.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace craft_planning::mission {

const char *name(Group g) {return g == Group::Red ? "R" : g == Group::Yellow ? "Y" : "G";}
const char *name(PuckColor c) {return c == PuckColor::Red ? "red" : "green";}
const char *name(PuckShape s) {return s == PuckShape::Cuboid ? "cuboid" : "cylinder";}
const char *name(SortOrder o) {return o == SortOrder::Ascending ? "ascending" : "descending";}

std::optional<SortOrder> order_from_barcode(const std::string &code) {
  if (code == "P2-01" || code == "P2-03") return SortOrder::Ascending;
  if (code == "P2-02" || code == "P2-04") return SortOrder::Descending;
  return std::nullopt;
}

std::string check_signal(const SignalCounts &c) {
  const int total = c.red + c.yellow + c.green;
  if (total != 12) return "signal has " + std::to_string(total) + " blinks, not 12";
  for (Group g : {Group::Red, Group::Yellow, Group::Green})
    if (c.of(g) < 1 || c.of(g) > 6)
      return std::string("signal ") + name(g) + " count " + std::to_string(c.of(g)) + " is outside 1..6";
  return {};
}

std::optional<SignalCounts> count_sequence(const std::vector<uint8_t> &sequence) {
  SignalCounts c;
  for (uint8_t v : sequence) {
    if (v == 0) ++c.red; else if (v == 1) ++c.yellow; else if (v == 2) ++c.green; else return std::nullopt;
  }
  return c;
}

std::array<Group, 3> location_groups(const SignalCounts &counts, SortOrder order) {
  std::array<Group, 3> groups{Group::Red, Group::Yellow, Group::Green};  // the tie order
  std::stable_sort(groups.begin(), groups.end(), [&](Group a, Group b) {
    return order == SortOrder::Ascending ? counts.of(a) < counts.of(b) : counts.of(a) > counts.of(b);
  });
  return groups;
}

Allocation allocate(const std::vector<Puck> &pucks, const SignalCounts &counts) {
  Allocation out;
  if (const auto error = check_signal(counts); !error.empty()) {out.error = error; return out;}
  if (pucks.size() != 12) {out.error = std::to_string(pucks.size()) + " pucks registered, not 12"; return out;}
  std::set<int> seen;
  for (const auto &p : pucks) {
    if (p.id < 0 || p.id > 11) {out.error = "puck id " + std::to_string(p.id) + " is outside 0..11"; return out;}
    if (!seen.insert(p.id).second) {out.error = "puck id " + std::to_string(p.id) + " registered twice"; return out;}
  }
  // ids by colour and shape, ascending
  std::vector<int> by[2][2];
  for (const auto &p : pucks) by[static_cast<int>(p.color)][static_cast<int>(p.shape)].push_back(p.id);
  for (auto &colour : by) for (auto &ids : colour) std::sort(ids.begin(), ids.end());
  const int reds = by[0][0].size() + by[0][1].size(), greens = by[1][0].size() + by[1][1].size();
  // Every puck is used once, so how many of each colour Y takes is fixed by the counts.
  const int yellow_red = reds - counts.red, yellow_green = greens - counts.green;
  if (yellow_red < 0 || yellow_green < 0) {
    out.error = "R=" + std::to_string(counts.red) + " G=" + std::to_string(counts.green) + " needs more than the " +
      std::to_string(reds) + " red / " + std::to_string(greens) + " green pucks there are";
    return out;
  }
  for (int shape = 0; shape < 2; ++shape) {
    if (static_cast<int>(by[0][shape].size()) < yellow_red || static_cast<int>(by[1][shape].size()) < yellow_green) continue;
    std::vector<int> yellow(by[0][shape].begin(), by[0][shape].begin() + yellow_red);
    yellow.insert(yellow.end(), by[1][shape].begin(), by[1][shape].begin() + yellow_green);
    const std::set<int> taken(yellow.begin(), yellow.end());
    std::vector<int> red, green;
    for (const auto &p : pucks) {
      if (taken.count(p.id)) continue;
      (p.color == PuckColor::Red ? red : green).push_back(p.id);
    }
    std::sort(red.begin(), red.end()); std::sort(green.begin(), green.end()); std::sort(yellow.begin(), yellow.end());
    out.ok = true; out.yellow_shape = static_cast<PuckShape>(shape);
    out.ids = {red, yellow, green};
    return out;
  }
  out.error = "Y needs " + std::to_string(yellow_red) + " red + " + std::to_string(yellow_green) +
    " green pucks of one shape: cuboids " + std::to_string(by[0][0].size()) + " red / " +
    std::to_string(by[1][0].size()) + " green, cylinders " + std::to_string(by[0][1].size()) + " red / " +
    std::to_string(by[1][1].size()) + " green";
  return out;
}

std::vector<std::array<int, 2>> dice_spots(int face) {
  switch (face) {
    case 1: return {{0, 0}};
    case 2: return {{-1, 1}, {1, -1}};
    case 3: return {{-1, 1}, {0, 0}, {1, -1}};
    case 4: return {{-1, 1}, {1, 1}, {-1, -1}, {1, -1}};
    case 5: return {{-1, 1}, {1, 1}, {0, 0}, {-1, -1}, {1, -1}};
    case 6: return {{-1, 1}, {-1, 0}, {-1, -1}, {1, 1}, {1, 0}, {1, -1}};
  }
  throw std::invalid_argument("dice face must be 1..6");
}

}  // namespace craft_planning::mission
