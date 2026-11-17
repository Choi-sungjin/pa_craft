// Odometry -> persistent point trail for RViz (RViz's Odometry display only draws arrows/axes and drops them).
// Every pose at least min_spacing from the last kept one is added; the whole trail goes out as one
// visualization_msgs/Marker POINTS in the odometry's frame (latched, so a late or re-toggled RViz gets it all).
// In a map-fixed RViz the trail moves with map->odom: it shows odometry, drift included.
//   ros2 run craft_bringup odom_trail --ros-args -p odom_topic:=/odometry/local -p trail_topic:=/odometry/local/trail
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using visualization_msgs::msg::Marker;

class OdomTrail : public rclcpp::Node {
public:
  OdomTrail() : Node("odom_trail") {
    const auto odom_topic = declare_parameter("odom_topic", std::string("/odometry/local"));
    const auto trail_topic = declare_parameter("trail_topic", std::string("/odometry/local/trail"));
    spacing_ = declare_parameter("min_spacing", 0.02);                        // [m]
    max_points_ = declare_parameter("max_points", int64_t{100000});           // oldest dropped beyond this
    const double period = 1.0 / declare_parameter("publish_hz", 5.0);
    marker_.ns = "odom_trail";
    marker_.id = 0;
    marker_.type = Marker::POINTS;
    marker_.action = Marker::ADD;
    marker_.scale.x = marker_.scale.y = declare_parameter("point_size", 0.03);
    marker_.color.r = 42.0 / 255;
    marker_.color.g = 120.0 / 255;
    marker_.color.b = 214.0 / 255;
    marker_.color.a = 1.0;
    marker_.pose.orientation.w = 1.0;
    pub_ = create_publisher<Marker>(trail_topic, rclcpp::QoS(1).transient_local());
    // EKF publishes RELIABLE; a best-effort subscription matches either.
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { publish(); });
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg) {
    const auto & p = msg.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    if (!marker_.header.frame_id.empty() && msg.header.frame_id != marker_.header.frame_id)
      marker_.points.clear();  // a different odometry frame: start over
    marker_.header.frame_id = msg.header.frame_id;
    if (!marker_.points.empty()) {
      const auto & last = marker_.points.back();
      if (std::hypot(p.x - last.x, p.y - last.y) < spacing_) return;
    }
    marker_.points.push_back(p);
    if (static_cast<int64_t>(marker_.points.size()) > max_points_)
      marker_.points.erase(marker_.points.begin(), marker_.points.end() - max_points_);
    changed_ = true;
  }

  void publish() {
    if (!changed_) return;
    marker_.header.stamp = now();
    pub_->publish(marker_);
    changed_ = false;
  }

  double spacing_;
  int64_t max_points_;
  bool changed_ = false;
  Marker marker_;
  rclcpp::Publisher<Marker>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTrail>());
  rclcpp::shutdown();
}
