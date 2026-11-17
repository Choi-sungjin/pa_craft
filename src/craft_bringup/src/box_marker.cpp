// Box detections -> RViz squares (/perception/box/markers, visualization_msgs/MarkerArray, map frame).
//   grey   : the box planning asked for (/planning/box_observation_request: expected_box, box_side)
//   green  : the last valid LiDAR detection (/perception/box/observation, source lidar)
//   blue   : the last valid upper-cam detection (/perception/uppercam/observation)
//   dots   : every valid detection centre since the last request (green LiDAR, blue camera): a wall taken for the
//            box shows up as dots away from the box
//   text   : box id, source and valid / why not, above the expected box
//   names  : every box of planning.yaml box_approach (S1..S6, L1..L3) at its place, once a second
// An invalid detection removes that source's square. Works on a bag replay too:
//   ros2 run craft_bringup box_marker   (with ros2 bag play --clock: --ros-args -p use_sim_time:=true)
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "craft_interfaces/msg/box_observation.hpp"
#include "craft_interfaces/msg/box_observation_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using craft_interfaces::msg::BoxObservation;
using craft_interfaces::msg::BoxObservationRequest;
using geometry_msgs::msg::Point;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
  ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

const ColorRGBA GREY = rgba(0.6f, 0.6f, 0.6f);
const std::map<std::string, ColorRGBA> COLOR = {{"lidar", rgba(0.1f, 0.85f, 0.2f)},
                                                {"uppercam", rgba(0.15f, 0.45f, 1.0f)}};
const std::map<std::string, int> ID = {{"expected", 0}, {"lidar", 1}, {"uppercam", 2},
                                       {"text", 3}, {"lidar_dots", 4}, {"uppercam_dots", 5}};

// {name: (x, y)} from planning.yaml box_approach (names + [x, y, yaw] per name).
void walk(const YAML::Node & d, std::map<std::string, std::pair<double, double>> & found) {
  if (d.IsMap()) {
    if (d["names"] && d["names"].IsSequence())
      for (const auto & n : d["names"]) {
        const auto name = n.as<std::string>();
        const auto v = d[name];
        if (v && v.IsSequence() && v.size() >= 2) found[name] = {v[0].as<double>(), v[1].as<double>()};
      }
    for (const auto & kv : d) walk(kv.second, found);
  } else if (d.IsSequence()) {
    for (const auto & v : d) walk(v, found);
  }
}

std::map<std::string, std::pair<double, double>> box_places() {
  std::map<std::string, std::pair<double, double>> found;
  walk(YAML::LoadFile((ament_index_cpp::get_package_share_path("craft_planning") / "config" / "planning.yaml").string()), found);
  return found;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q) { return 2 * std::atan2(q.z, q.w); }

std::vector<Point> square(double x, double y, double yaw, double side, double z) {
  const double c = std::cos(yaw), s = std::sin(yaw), h = side / 2;
  const double corners[5][2] = {{h, h}, {-h, h}, {-h, -h}, {h, -h}, {h, h}};
  std::vector<Point> out;
  for (const auto & ab : corners) {
    Point p;
    p.x = x + c * ab[0] - s * ab[1];
    p.y = y + s * ab[0] + c * ab[1];
    p.z = z;
    out.push_back(p);
  }
  return out;
}

std::string fmt(const char * f, double a, double b, double c) {
  char buf[96];
  std::snprintf(buf, sizeof buf, f, a, b, c);
  return buf;
}

}  // namespace

class BoxMarker : public rclcpp::Node {
public:
  BoxMarker() : Node("box_marker") {
    pub_ = create_publisher<MarkerArray>("/perception/box/markers", 10);
    subs_.push_back(create_subscription<BoxObservationRequest>(
        "/planning/box_observation_request", 10,
        [this](BoxObservationRequest::ConstSharedPtr r) { on_request(*r); }));
    obs_subs_.push_back(create_subscription<BoxObservation>(
        "/perception/box/observation", 10, [this](BoxObservation::ConstSharedPtr m) { on_obs(*m, "lidar"); }));
    obs_subs_.push_back(create_subscription<BoxObservation>(
        "/perception/uppercam/observation", 10,
        [this](BoxObservation::ConstSharedPtr m) { on_obs(*m, "uppercam"); }));
    int i = 0;
    for (const auto & [name, xy] : box_places()) {  // std::map: sorted by name, as Python's sorted()
      Marker m;
      m.ns = "box_names";
      m.id = i++;
      m.type = Marker::TEXT_VIEW_FACING;
      m.action = Marker::ADD;
      m.text = name;
      m.header.frame_id = "map";
      m.pose.position.x = xy.first;
      m.pose.position.y = xy.second;
      m.pose.position.z = 0.25;
      m.pose.orientation.w = 1.0;
      m.scale.z = 0.12;
      m.color = rgba(1.0f, 0.85f, 0.2f);
      names_.markers.push_back(m);
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { pub_->publish(names_); });  // RViz may join late
  }

private:
  Marker marker(const std::string & name, const std_msgs::msg::Header & header, int kind = Marker::LINE_STRIP) {
    Marker m;
    m.ns = "box";
    m.id = ID.at(name);
    m.type = kind;
    m.action = Marker::ADD;
    m.header.frame_id = header.frame_id.empty() ? "map" : header.frame_id;
    m.header.stamp = header.stamp;
    m.pose.orientation.w = 1.0;
    return m;
  }

  void on_request(const BoxObservationRequest & req) {
    if (req.source == "off") return;  // the end of a box step: keep what was seen last
    if (req.box_side > 0) side_ = req.box_side;
    if (req.box_id != box_id_) {
      dots_.clear();
      status_.clear();
    }
    box_id_ = req.box_id;
    const auto & p = req.expected_box.position;
    auto m = marker("expected", req.header);
    m.scale.x = 0.01;
    m.color = GREY;
    m.points = square(p.x, p.y, yaw_of(req.expected_box.orientation), side_, 0.01);
    expected_ = {p.x, p.y};
    MarkerArray out;
    out.markers = {m, text(req.header)};
    pub_->publish(out);
  }

  Marker text(const std_msgs::msg::Header & header) {
    auto m = marker("text", header, Marker::TEXT_VIEW_FACING);
    m.pose.position.x = expected_.first;
    m.pose.position.y = expected_.second;
    m.pose.position.z = 0.35;
    m.scale.z = 0.08;
    m.color = rgba(1.0f, 1.0f, 1.0f);
    m.text = box_id_;
    for (const auto & [k, v] : status_) m.text += "\n" + k + ": " + v;
    return m;
  }

  void on_obs(const BoxObservation & obs, const std::string & source) {
    auto box = marker(source, obs.header);
    const auto & p = obs.box_pose.position;
    const bool valid = obs.valid && std::isfinite(p.x) && std::isfinite(p.y) && !(p.x == 0.0 && p.y == 0.0);
    auto & dots = dots_[source];
    if (valid) {
      const double yaw = yaw_of(obs.box_pose.orientation);
      box.scale.x = 0.015;
      box.color = COLOR.at(source);
      box.points = square(p.x, p.y, yaw, side_, 0.03);
      Point d;
      d.x = p.x; d.y = p.y; d.z = 0.03;
      dots.push_back(d);
      if (dots.size() > 300) dots.erase(dots.begin(), dots.end() - 300);
      status_[source] = fmt("ok (%.3f, %.3f) %.0f deg", p.x, p.y, yaw * 180.0 / M_PI);
    } else {
      box.action = Marker::DELETE;
      status_[source] = "invalid: " + obs.reason;
    }
    auto d = marker(source + "_dots", obs.header, Marker::POINTS);
    d.scale.x = d.scale.y = 0.02;
    d.color = COLOR.at(source);
    d.points = dots;
    if (d.points.empty()) d.action = Marker::DELETE;
    MarkerArray out;
    out.markers = {box, d, text(obs.header)};
    pub_->publish(out);
  }

  double side_ = 0.205;  // until a request says (planning.yaml box_approach side)
  std::string box_id_;
  std::map<std::string, std::string> status_;
  std::map<std::string, std::vector<Point>> dots_;
  std::pair<double, double> expected_{0.0, 0.0};
  MarkerArray names_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<BoxObservationRequest>::SharedPtr> subs_;
  std::vector<rclcpp::Subscription<BoxObservation>::SharedPtr> obs_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxMarker>());
  rclcpp::shutdown();
}
