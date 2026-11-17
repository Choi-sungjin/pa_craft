#pragma once
#include "geometry.hpp"
namespace puck::execute {
Steps steps_from_plan(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps lying_steps_from_plan(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps load_tail(const Pose &lift,std::optional<Pose> start);
Steps pick_steps(const std::string &prefix);
std::pair<Path,Pose> dice_line(double lm,const std::string &row,std::optional<Pose> q0_raw=std::nullopt);
Steps dice_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps unload_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps bigbro_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps gripcal_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps gripzero_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
}
