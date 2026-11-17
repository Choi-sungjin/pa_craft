#pragma once
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace puck::execute {
// Regenerate plan.py's standing trajectory after a camera correction. The
// returned descent includes the grasp as its last target; nullopt means the
// corrected point cannot be planned.
std::optional<std::pair<std::vector<double>, std::vector<std::vector<double>>>>
replan(double side_mm, double ahead_mm, const std::string &out,
       const std::string &plan_file, std::optional<double> roll = std::nullopt,
       double shelf_mm = 0.0);
}
