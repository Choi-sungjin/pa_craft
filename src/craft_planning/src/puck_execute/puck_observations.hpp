// Port of demo/puck_pick/puck_observations.py: C++ detector output -> craft_perception PuckObservationArray (display only).
#pragma once

#include <craft_perception/msg/puck_observation_array.hpp>

#include <string>

namespace puck {

craft_perception::msg::PuckObservationArray upper_observations(const std::string &text, const std::string &pose_path,
                                                               double ahead_bias = 0., double lateral_bias = 0.);
craft_perception::msg::PuckObservationArray grip_observations(const std::string &text);

}  // namespace puck
