// Detected pucks -> flat, map-fixed RViz footprints and explicit attribute labels.
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "craft_perception/msg/puck_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using craft_perception::msg::PuckObservation;
using craft_perception::msg::PuckObservationArray;
using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

using Vec3 = std::array<double, 3>;
using XY = std::pair<double, double>;

bool contains(const std::string & s, const char * sub) { return s.find(sub) != std::string::npos; }

std::pair<std::string, std::string> attributes(const std::string & label) {
  std::string color = "unknown";
  std::stringstream ss(label);
  std::string part;
  std::vector<std::string> parts;
  while (std::getline(ss, part, '_')) parts.push_back(part);
  for (const char * v : {"red", "green"}) {
    bool hit = false;
    for (const auto & p : parts) hit |= p == v;
    if (hit) { color = v; break; }
  }
  const std::string shape = contains(label, "cylinder") ? "cylinder"
                            : (contains(label, "prism") || contains(label, "cuboid")) ? "prism" : "unknown";
  return {color, shape};
}

Vec3 rotate(const geometry_msgs::msg::Quaternion & q, const Vec3 & v) {
  const double x = v[0], y = v[1], z = v[2];
  const double tx = 2 * (q.y * z - q.z * y), ty = 2 * (q.z * x - q.x * z), tz = 2 * (q.x * y - q.y * x);
  return {x + q.w * tx + q.y * tz - q.z * ty, y + q.w * ty + q.z * tx - q.x * tz, z + q.w * tz + q.x * ty - q.y * tx};
}

std::vector<XY> outline(const std::string & shape, const std::string & posture) {
  // Nominal dimensions: diameter/width 30 mm, length/height 60 mm.
  if (shape == "unknown" || posture == "unknown") return {{0.02, 0}, {0, .02}, {-.02, 0}, {0, -.02}, {.02, 0}};
  if (shape == "cylinder") {
    std::vector<XY> pts;
    if (posture == "standing") {
      for (int i = 0; i < 41; ++i) pts.push_back({.015 * std::cos(i * 2 * M_PI / 40), .015 * std::sin(i * 2 * M_PI / 40)});
      return pts;
    }
    for (const auto & [cx, start] : {XY{.015, -M_PI / 2}, XY{-.015, M_PI / 2}})
      for (int i = 0; i < 21; ++i)
        pts.push_back({cx + .015 * std::cos(start + i * M_PI / 20), .015 * std::sin(start + i * M_PI / 20)});
    pts.push_back(pts.front());
    return pts;
  }
  const double half = posture == "lying" ? .03 : .015;
  return {{half, .015}, {-half, .015}, {-half, -.015}, {half, -.015}, {half, .015}};
}

Point pt(double x, double y, double z) {
  Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

std::vector<Marker> markers_for(const std::string & source, int index, const std::string & frame,
                                const PuckObservation & obs, const Vec3 & center, const Vec3 & axis, double z,
                                double lifetime, const std::optional<std::string> & color_label) {
  auto [color, shape] = attributes(obs.detection.label);
  if (color == "unknown" && color_label) color = attributes(*color_label).first;
  const std::string posture = contains(obs.detection.label, "posture_unknown") ? "unknown"
                              : obs.standing ? "standing" : "lying";
  const bool angle_known = std::hypot(axis[0], axis[1]) > .1;
  const double yaw = angle_known ? std::atan2(axis[1], axis[0]) : 0.;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const std::map<std::string, Vec3> rgbs = {{"red", {1., .12, .10}}, {"green", {.1, .9, .25}}, {"unknown", {.65, .65, .65}}};
  const auto & rgb = rgbs.at(color);
  std::vector<Point> points;
  for (const auto & [x, y] : outline(shape, posture)) points.push_back(pt(center[0] + c * x - s * y, center[1] + s * x + c * y, z));

  Marker fill;
  fill.ns = source + "/footprint"; fill.id = index; fill.type = Marker::TRIANGLE_LIST; fill.action = Marker::ADD;
  fill.header.frame_id = frame; fill.pose.orientation.w = 1.;
  fill.scale.x = fill.scale.y = fill.scale.z = 1.;
  fill.color.r = rgb[0]; fill.color.g = rgb[1]; fill.color.b = rgb[2]; fill.color.a = .85;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    fill.points.push_back(pt(center[0], center[1], z));
    fill.points.push_back(points[i]);
    fill.points.push_back(points[i + 1]);
  }
  Marker edge;
  edge.ns = source + "/outline"; edge.id = index; edge.type = Marker::LINE_STRIP; edge.action = Marker::ADD;
  edge.header.frame_id = frame; edge.pose.orientation.w = 1.; edge.scale.x = .002;
  edge.color.r = edge.color.g = edge.color.b = edge.color.a = 1.; edge.points = points;
  Marker label;
  label.ns = source + "/label"; label.id = index; label.type = Marker::TEXT_VIEW_FACING; label.action = Marker::ADD;
  label.header.frame_id = frame; label.pose.orientation.w = 1.;
  label.pose.position = pt(center[0], center[1] + .045, z + .01);
  label.scale.z = .022; label.color.r = label.color.g = label.color.b = label.color.a = 1.;
  const std::string identity = (obs.tag_valid && obs.puck_id >= 0 ? "#" + std::to_string(obs.puck_id) : "@" + std::to_string(index));
  char conf[16];
  std::snprintf(conf, sizeof conf, "%.0f%%", obs.detection.confidence * 100.0);
  label.text = identity + " " + posture + " | " + color + " " + shape + "\n" + source + " " + conf;
  if (posture == "lying" && !angle_known) label.text += " | yaw unknown";
  const auto life = rclcpp::Duration::from_seconds(lifetime);
  for (auto * m : {&fill, &edge, &label}) m->lifetime = life;
  return {fill, edge, label};
}

struct Item {
  int index;
  PuckObservation obs;
  Vec3 center, axis;
};

struct View {
  std::vector<Item> items;
  int64_t received;
  double ttl;
};

}  // namespace

class PuckMarker : public rclcpp::Node {
public:
  PuckMarker() : Node("puck_marker"), tf_(get_clock()) {
    listener_ = std::make_unique<tf2_ros::TransformListener>(tf_, this, false);
    frame_ = declare_parameter("fixed_frame", std::string("map"));
    z_ = declare_parameter("plane_z", .04);
    live_ttl_ = declare_parameter("live_timeout_s", 3.);
    shot_ttl_ = declare_parameter("snapshot_timeout_s", 30.);
    if (std::min(live_ttl_, shot_ttl_) <= 0) throw std::invalid_argument("marker timeouts must be positive");
    pub_ = create_publisher<MarkerArray>("/perception/pucks/markers", rclcpp::QoS(1).transient_local());
    const std::vector<std::pair<std::string, std::string>> sources = {
        {"uppercam", "/perception/vision/uppercam/puck_survey_observations"},
        {"gripcam", "/perception/vision/d435/puck_survey_observations"},
        {"picker_upper", "/perception/pucks/picker_upper"},
        {"picker_grip", "/perception/pucks/picker_grip"},
    };
    for (const auto & [source, topic] : sources)
      subs_.push_back(create_subscription<PuckObservationArray>(
          topic, 10, [this, s = source](PuckObservationArray::ConstSharedPtr m) { receive(s, *m); }));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { refresh(); });
  }

private:
  void receive(const std::string & source, const PuckObservationArray & msg) {
    pending_.erase(source);
    if (!msg.observations.empty()) pending_[source] = {msg, now().nanoseconds()};
    views_.erase(source);  // a new empty/invalid observation must clear old detections
  }

  static bool live(int64_t now, int64_t received, double ttl) {
    return !((now - received) * 1e-9 > ttl || now < received);
  }

  void refresh() {
    const int64_t now_ns = now().nanoseconds();
    for (auto it = pending_.begin(); it != pending_.end();) {
      const auto & [source, entry] = *it;
      const auto & [msg, received] = entry;
      const double ttl = source.rfind("picker_", 0) == 0 ? shot_ttl_ : live_ttl_;
      if (!live(now_ns, received, ttl)) {
        views_.erase(source);
        it = pending_.erase(it);
        continue;
      }
      std::string frame = msg.header.frame_id;
      if (frame == "camera_color_optical_frame") frame = "camera_wrist_color_optical_frame";  // project D435 is the gripper camera
      std::optional<geometry_msgs::msg::Transform> transform;
      if (frame != frame_) {
        try {
          transform = tf_.lookupTransform(frame_, frame, rclcpp::Time(msg.header.stamp)).transform;
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "puck display waiting for %s -> %s: %s",
                               frame.c_str(), frame_.c_str(), e.what());
          ++it;
          continue;  // retry at the acquisition stamp; never place an untransformed point at the origin
        }
      }
      std::vector<Item> items;
      for (size_t i = 0; i < msg.observations.size(); ++i) {
        const auto & o = msg.observations[i];
        if (!o.axis_valid) continue;  // these messages have no measured 3D centre
        Vec3 center{o.center.x, o.center.y, o.center.z}, axis{o.axis.x, o.axis.y, o.axis.z};
        bool finite = true;
        for (double v : center) finite &= std::isfinite(v);
        for (double v : axis) finite &= std::isfinite(v);
        if (!finite) continue;
        if (transform) {
          center = rotate(transform->rotation, center);
          const auto & t = transform->translation;
          center = {center[0] + t.x, center[1] + t.y, center[2] + t.z};
          axis = rotate(transform->rotation, axis);
        }
        items.push_back({static_cast<int>(i), o, center, axis});
      }
      views_[source] = {items, received, ttl};
      it = pending_.erase(it);
    }
    MarkerArray out;
    Marker clear;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);
    auto fresh = [&](const std::string & name) -> std::vector<Item> {
      const auto v = views_.find(name);
      if (v == views_.end()) return {};
      const double age = (now_ns - v->second.received) * 1e-9;
      return 0 <= age && age <= v->second.ttl ? v->second.items : std::vector<Item>{};
    };
    const auto upper = fresh("picker_upper");
    const auto detailed = fresh("picker_grip");
    for (auto it = views_.begin(); it != views_.end();) {
      const auto & [source, view] = *it;
      if (!live(now_ns, view.received, view.ttl)) {
        it = views_.erase(it);
        continue;
      }
      for (const auto & item : view.items) {
        const auto shape = attributes(item.obs.detection.label).second;
        if (source == "picker_upper") {
          bool replaced = false;
          for (const auto & d : detailed)
            replaced |= attributes(d.obs.detection.label).second == shape &&
                        std::hypot(d.center[0] - item.center[0], d.center[1] - item.center[1]) < .06;
          if (replaced) continue;  // the confirmed gripper view replaces this coarse footprint
        }
        std::optional<std::string> match;
        if (source == "picker_grip" && !upper.empty()) {
          const Item * best = nullptr;
          double best_d = 0;
          for (const auto & v : upper) {
            const double dd = std::hypot(v.center[0] - item.center[0], v.center[1] - item.center[1]);
            if (!best || dd < best_d) { best = &v; best_d = dd; }
          }
          if (attributes(best->obs.detection.label).second == shape && best_d < .06) match = best->obs.detection.label;
        }
        for (auto & m : markers_for(source, item.index, frame_, item.obs, item.center, item.axis, z_, view.ttl, match))
          out.markers.push_back(std::move(m));
      }
      ++it;
    }
    pub_->publish(out);
  }

  std::string frame_;
  double z_, live_ttl_, shot_ttl_;
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::map<std::string, std::pair<PuckObservationArray, int64_t>> pending_;
  std::map<std::string, View> views_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<PuckObservationArray>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuckMarker>());
  rclcpp::shutdown();
}
