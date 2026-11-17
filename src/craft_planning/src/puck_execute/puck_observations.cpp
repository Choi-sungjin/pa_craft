#include "puck_observations.hpp"

#include "pick_policy.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace puck {

namespace {
const std::string NUMBER = R"(([+-]?\d+(?:\.\d+)?))";
bool ends_with(const std::string &s, const std::string &t) {
  return s.size() >= t.size() && s.compare(s.size() - t.size(), t.size(), t) == 0;
}
}  // namespace

craft_perception::msg::PuckObservationArray upper_observations(const std::string &text, const std::string &pose_path,
                                                               double ahead_bias, double lateral_bias) {
  craft_perception::msg::PuckObservationArray out;
  out.header.frame_id = "base_link";
  std::ifstream f(pose_path);
  std::string first;
  if (!f || !std::getline(f, first)) throw std::runtime_error("cannot read " + pose_path);
  std::istringstream in(first);
  std::vector<double> p;
  for (double x; in >> x;) p.push_back(x);
  const double floor = p.at(12), laser_x = p.at(13), laser_y = p.at(14), face = p.at(16), sign = p.at(18);
  static const std::regex head("PUCK (\\S+) " + NUMBER + " ahead " + NUMBER + " lateral " + NUMBER + " top " + NUMBER);
  static const std::regex state(R"( posture (standing|lying|unknown)\b)");
  for (const auto &line : policy::splitlines(text)) {
    std::smatch m;
    if (!std::regex_search(line, m, head, std::regex_constants::match_continuous)) continue;
    const std::string label = m[1];
    craft_perception::msg::PuckObservation o;
    o.puck_id = -1;
    std::smatch s;
    const bool has_state = std::regex_search(line, s, state);
    const std::string posture = ends_with(label, "cylinder") ? "standing" : has_state ? std::string(s[1]) : "unknown";
    o.standing = posture == "standing";
    o.detection.label = label + (posture == "unknown" ? "_posture_unknown" : "");
    o.detection.confidence = static_cast<float>(std::stod(m[2]));
    o.axis_valid = true;  // position measured; posture explicitly unknown in the label
    o.center.x = laser_x + face + (std::stod(m[3]) - ahead_bias) / 1000;
    o.center.y = laser_y + (std::stod(m[4]) - lateral_bias) / 1000 / sign;
    o.center.z = floor + std::stod(m[5]) / 1000;
    out.observations.push_back(o);
  }
  return out;
}

craft_perception::msg::PuckObservationArray grip_observations(const std::string &text) {
  craft_perception::msg::PuckObservationArray out;
  out.header.frame_id = "camera_wrist_color_optical_frame";
  static const std::regex yaw_re(": (LYING )?YAW " + NUMBER + " deg CENTRE " + NUMBER + " " + NUMBER + " " + NUMBER + " mm");
  std::smatch m;
  if (!std::regex_search(text, m, yaw_re)) return out;
  craft_perception::msg::PuckObservation o;
  o.puck_id = -1;
  o.axis_valid = true;
  o.standing = !m[1].matched;
  o.detection.label = text.find("VIZ_SHAPE cylinder") != std::string::npos ? "unknown_cylinder" : "unknown_prism";
  static const std::regex frames_re(R"((\d+)/(\d+) frames)");
  std::smatch fr;
  o.detection.confidence = 0.f;
  if (std::regex_search(text, fr, frames_re) && std::stoi(fr[2]))
    o.detection.confidence = static_cast<float>(static_cast<double>(std::stoi(fr[1])) / std::stoi(fr[2]));
  o.center.x = std::stod(m[3]) / 1000;
  o.center.y = std::stod(m[4]) / 1000;
  o.center.z = std::stod(m[5]) / 1000;
  static const std::regex axis_re("VIZ_AXIS " + NUMBER + " " + NUMBER + " " + NUMBER);
  std::smatch ax;
  if (std::regex_search(text, ax, axis_re)) {
    o.axis.x = std::stod(ax[1]);
    o.axis.y = std::stod(ax[2]);
    o.axis.z = std::stod(ax[3]);
  }
  o.height_m = o.standing ? .060 : .030;
  out.observations.push_back(o);
  return out;
}

}  // namespace puck
