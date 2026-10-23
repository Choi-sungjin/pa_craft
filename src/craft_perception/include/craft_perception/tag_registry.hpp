#pragma once

#include <string>
#include <unordered_map>

namespace craft_perception {

enum class TagRole { Mission, Puck };

struct TagTarget {
  TagRole role = TagRole::Puck;
  std::string mission;
  int puck_id = -1;
};

using TagRegistry = std::unordered_map<int, TagTarget>;

// File format: tags: {<tag id>: {mission: <name>} | {puck: <number>}}
// Unknown fields, duplicate IDs and invalid values throw std::invalid_argument.
TagRegistry load_tag_registry(const std::string &filename);

}  // namespace craft_perception
