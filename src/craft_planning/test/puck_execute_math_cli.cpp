#include "plan.hpp"
#include "teach.hpp"
#include "pick_policy.hpp"
#include "puck_observations.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::ordered_json;
using namespace puck;
json observations(const craft_perception::msg::PuckObservationArray &a) {
  json out = {{"frame", a.header.frame_id}, {"items", json::array()}};
  for (const auto &o : a.observations) out["items"].push_back({{"label", o.detection.label}, {"confidence", o.detection.confidence}, {"standing", o.standing}, {"axis_valid", o.axis_valid}, {"center", {o.center.x,o.center.y,o.center.z}}, {"axis", {o.axis.x,o.axis.y,o.axis.z}}, {"height_m",o.height_m}, {"puck_id",o.puck_id}});
  return out;
}
int main() {
  json in; std::cin >> in; json out;
  try {
    auto mode = in.at("mode").get<std::string>();
    if (mode == "teach") {out["target"] = teach::raw_to_target(in.at("raw").get<std::vector<double>>()); out["raw"] = teach::target_to_raw(out["target"].get<std::vector<double>>()); out["limits"] = teach::motor_limits();}
    else if (mode == "plan") {
      Chain ch(urdf_path()); JointMap q = in.at("q").get<JointMap>(); auto t = ch.fk(q); out["fk"] = json::array();
      for (int i=0;i<4;++i) {out["fk"].push_back(json::array()); for(int j=0;j<4;++j) out["fk"][i].push_back(t(i,j));}
      for (const auto &n : Chain::NAMES) out["limits"][n] = ch.limits(n);
      auto v = in.at("tcp").get<std::vector<double>>(); out["ik"] = ik_top_down(ch,{v.at(0),v.at(1),v.at(2)},in.value("tilt",0.));
    } else if (mode == "policy") {
      auto c = policy::candidates(in.at("text"),in.value("ahead_bias",0.),in.value("lateral_bias",0.),in.value("face_x",257.67),in.value("laser_y",.02),in.value("min_score",.4)); out["candidates"] = json::array(); for (const auto &p:c) out["candidates"].push_back(p.record());
      try {out["selected"] = policy::select(c,in.contains("expected") ? std::optional<int>(in["expected"].get<int>()) : std::nullopt).record();} catch (const policy::ValueError &e) {out["error"] = e.what();}
    } else if (mode == "mod") out["value"] = py_mod(in.at("a"), in.at("b"));
    else if (mode == "paths") out = {{"here",here().string()},{"ws",ws().string()},{"control",control_yaml().string()},{"urdf",urdf_path().string()}};
    else if (mode == "upper") out = observations(upper_observations(in.at("text"),in.at("pose"),in.value("ahead_bias",0.),in.value("lateral_bias",0.)));
    else if (mode == "grip") out = observations(grip_observations(in.at("text")));
    else throw std::runtime_error("unknown mode");
    std::cout << out.dump() << '\n';
  } catch(const std::exception &e) {std::cerr << e.what() << '\n'; return 1;}
}
