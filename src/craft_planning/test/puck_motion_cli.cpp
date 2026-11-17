#include "motion.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <optional>
int main() {
  using nlohmann::json;
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      auto in=json::parse(line);std::optional<puck::Pose> start;
      if(in.contains("start")&&!in["start"].is_null())start=in["start"].get<puck::Pose>();
      auto result=puck::motion::prepare_motion(in.at("name").get<std::string>(),start,in.at("target").get<puck::Pose>(),in.value("seconds",2.),in.value("tolerance",.01),in.value("held",false),in.value("box_known",true));
      json moves=json::array();for(const auto &m:result.moves)moves.push_back({{"target",m.target},{"seconds",m.seconds},{"tolerance",m.tolerance},{"intermediate",m.intermediate}});
      std::cout<<json({{"moves",moves},{"reason",result.reason}}).dump()<<std::endl;
    } catch(const std::exception &e) {std::cout<<json({{"error",e.what()}}).dump()<<std::endl;}
  }
}
