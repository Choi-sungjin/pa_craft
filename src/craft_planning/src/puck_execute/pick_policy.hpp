// Port of demo/puck_pick/pick_policy.py: two-puck collection policy. Colour/shape do not break posture/distance priority.
#pragma once

#include <nlohmann/json.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace puck::policy {

struct ValueError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Candidate {
  std::string cls;
  double score = 0, ahead = 0, lateral = 0;
  std::string posture;
  std::optional<double> height;
  double distance = 0;
  bool operator==(const Candidate &o) const;
  nlohmann::ordered_json record() const;
};

std::vector<std::string> splitlines(const std::string &text);

std::vector<Candidate> candidates(const std::string &text, double ahead_bias = 0., double lateral_bias = 0.,
                                  double face_x = 257.67, double laser_y = .02, double min_score = .4);

Candidate select(const std::vector<Candidate> &found, std::optional<int> expected = std::nullopt);

}  // namespace puck::policy
