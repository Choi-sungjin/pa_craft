#pragma once
#include "plan.hpp"
#include <optional>
#include <string>
#include <vector>

namespace puck::motion {
constexpr double SHOULDER_EARLY_FRACTION = 0.;  // user 2026-11-16: the others first, then the shoulder alone
struct Move { Pose target; double seconds, tolerance; bool intermediate; };
struct Prepared { std::vector<Move> moves; std::string reason; };
bool candidate(const std::string &name);
Prepared prepare_motion(const std::string &name, const std::optional<Pose> &start,
                        const Pose &target, double seconds, double tolerance,
                        bool held = false, bool box_known = true);
}
