#include "craft_perception/tag_registry.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <stdexcept>

namespace craft_perception {
namespace {
int integer(const YAML::Node &node, const std::string &context) {
  if (!node.IsScalar() || node.Tag() == "tag:yaml.org,2002:str") {
    throw std::invalid_argument(context + " must be an integer");
  }
  const auto value = node.as<int64_t>();
  if (value < 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(context + " must be a nonnegative int32");
  }
  return static_cast<int>(value);
}
}  // namespace

TagRegistry load_tag_registry(const std::string &filename) {
  YAML::Node root;
  try { root = YAML::LoadFile(filename); }
  catch (const std::exception &e) {
    throw std::invalid_argument("cannot load tag registry " + filename + ": " + e.what());
  }
  if (!root.IsMap() || root.size() != 1 || !root["tags"] || !root["tags"].IsMap()) {
    throw std::invalid_argument("tag registry must contain only a tags mapping");
  }
  TagRegistry result;
  for (const auto &item : root["tags"]) {
    const int tag_id = integer(item.first, "tag ID");
    const auto target = item.second;
    if (!target.IsMap() || target.size() != 1) {
      throw std::invalid_argument("tag " + std::to_string(tag_id) +
        " must contain exactly one of mission or puck");
    }
    TagTarget parsed;
    if (target["mission"]) {
      if (!target["mission"].IsScalar()) {
        throw std::invalid_argument("tag mission must be a nonempty string");
      }
      parsed.role = TagRole::Mission;
      parsed.mission = target["mission"].as<std::string>();
      if (parsed.mission.empty()) throw std::invalid_argument("tag mission must be a nonempty string");
    } else if (target["puck"]) {
      parsed.role = TagRole::Puck;
      parsed.puck_id = integer(target["puck"], "puck number");
    } else {
      throw std::invalid_argument("tag " + std::to_string(tag_id) +
        " must contain exactly one of mission or puck");
    }
    if (!result.emplace(tag_id, std::move(parsed)).second) {
      throw std::invalid_argument("duplicate tag ID " + std::to_string(tag_id));
    }
  }
  return result;
}

}  // namespace craft_perception
