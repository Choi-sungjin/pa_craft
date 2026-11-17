// Global localization at start-up (craft.launch.py with slam_map:= or localization:=amcl): run_corner.py's step 1-2
// as a node (C++ port of demo/corner/global_localize.py).
//
// The robot standing still anywhere in the saved map: ~1 s of /lidar/merged is matched over the whole map
// (map_tools ScanMatcher), the pose goes out on /initialpose, and it is done once map->base_link agrees
// (< 10 cm, < 5 deg). --near X Y (craft.launch.py: initial_x/initial_y): only within --radius of it, and a fit there
// must score >= 0.85 (run_corner.py's --near rule). An ambiguous fit (runner-up within 8 %), too few points or no
// agreement: tried again every --retry_s (move or turn the robot a little meanwhile).
//
// /perception/localization/global (std_msgs/String, latched):
//   "searching"                      not localized yet (waiting for the scan / TF, or retrying)
//   "ok x y yaw_deg score"           localized; mission_master waits for this before its first drive
//   "retry: <why>"                   the last try failed, trying again
// A message on /perception/localization/global_request (std_msgs/Empty) runs it again.
#include <chrono>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "map_tools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

using map_tools::Pose;
using map_tools::Pt;
using sensor_msgs::msg::LaserScan;

namespace {

// as run_corner.py: the merged scan is in base_link, drop the robot itself
constexpr double BODY_CENTER_X = 0.06075;  // 0.01275 + 0.048 (base_link moved back 2026-11-15)
constexpr double BODY_R = 0.27;

double yaw_of(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

double wall() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }

std::string fmt(const char * f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

std::string near_text(const std::optional<Pt> & near) {
  return near ? fmt("(%g, %g)", (*near)[0], (*near)[1]) : "None";
}

struct Args {
  std::string map;
  double retry_s = 3.0, radius = 0.5;
  std::optional<Pt> near;
  std::optional<double> yaw;  // [deg]
};

Args parse(int argc, char ** argv) {  // argparse.parse_known_args: unknown (ROS) arguments are left alone
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](int n = 1) {
      if (i + n >= argc) throw std::invalid_argument(k + " needs " + std::to_string(n) + " value(s)");
      return std::string(argv[++i]);
    };
    if (k == "--map") a.map = next();
    else if (k == "--retry_s") a.retry_s = std::stod(next());
    else if (k == "--radius") a.radius = std::stod(next());
    else if (k == "--yaw") a.yaw = std::stod(next());
    else if (k == "--near") { const double x = std::stod(next(2)); a.near = Pt{x, std::stod(next())}; }
    else if (k == "-h" || k == "--help") {
      std::cout << "usage: global_localize --map MAP_YAML [--retry_s S] [--near X Y] [--radius M] [--yaw DEG]\n";
      std::exit(0);
    }
  }
  if (a.map.empty()) throw std::invalid_argument("the following arguments are required: --map");
  if (a.map.rfind("~/", 0) == 0 && std::getenv("HOME")) a.map = std::string(std::getenv("HOME")) + a.map.substr(1);
  return a;
}

}  // namespace

class GlobalLocalize {
public:
  GlobalLocalize(rclcpp::Node::SharedPtr node, const map_tools::ScanMatcher & matcher, const Args & args)
      : n_(node), matcher_(matcher), retry_s_(args.retry_s), near_(args.near), radius_(args.radius),
        tf_(node->get_clock()) {
    if (args.yaw) yaw_ = *args.yaw * M_PI / 180;
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_, node, false);
    scan_sub_ = n_->create_subscription<LaserScan>("/lidar/merged", rclcpp::SensorDataQoS(),
                                                   [this](LaserScan::SharedPtr m) { scans_.push_back(m); });
    req_sub_ = n_->create_subscription<std_msgs::msg::Empty>("/perception/localization/global_request", 10,
                                                             [this](std_msgs::msg::Empty::ConstSharedPtr) { request(); });
    initial_ = n_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    status_ = n_->create_publisher<std_msgs::msg::String>("/perception/localization/global",
                                                          rclcpp::QoS(1).transient_local());
    exec_.add_node(n_);
  }

  void run() {
    say("searching");
    while (rclcpp::ok()) {
      if (!wanted_) {
        spin(0.5);
        continue;
      }
      if (!spin(5.0, [this] { return !scans_.empty() && slam().has_value(); })) {
        RCLCPP_INFO(n_->get_logger(), "waiting for /lidar/merged and map->base_link (slam_toolbox) ...");
        continue;
      }
      const auto why = attempt();
      if (!why) {
        wanted_ = false;
        continue;
      }
      say("retry: " + *why);
      spin(retry_s_);
    }
  }

private:
  void request() {
    wanted_ = true;
    say("searching");
  }

  void say(const std::string & text) {
    std_msgs::msg::String m;
    m.data = text;
    status_->publish(m);
    RCLCPP_INFO(n_->get_logger(), "%s", text.c_str());
  }

  std::optional<Pose> slam() {
    try {
      const auto t = tf_.lookupTransform("map", "base_link", tf2::TimePointZero);
      return Pose{t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  bool spin(double sec, const std::function<bool()> & until = [] { return false; }) {
    const double end = wall() + sec;
    while (wall() < end && rclcpp::ok() && !until()) exec_.spin_once(std::chrono::milliseconds(50));
    return until();
  }

  std::optional<std::vector<Pt>> scan_points(double sec = 1.0, size_t keep = 240) {
    scans_.clear();
    spin(sec);
    std::vector<Pt> pts;
    for (const auto & m : scans_)
      for (size_t i = 0; i < m->ranges.size(); ++i) {
        const double r = m->ranges[i], a = m->angle_min + i * static_cast<double>(m->angle_increment);
        if (!std::isfinite(r) || r <= 0.05 || r >= 7.5) continue;
        const double x = r * std::cos(a), y = r * std::sin(a);
        if (std::hypot(x - BODY_CENTER_X, y) > BODY_R) pts.push_back({x, y});
      }
    if (scans_.empty() || pts.empty()) return std::nullopt;
    // ponytail: seeded mt19937 sample, not numpy's PCG64 choice: a different (equally random) 240-point subset
    std::mt19937 rng(0);
    std::shuffle(pts.begin(), pts.end(), rng);
    pts.resize(std::min(keep, pts.size()));
    return pts;
  }

  void send_initial(const Pose & pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.frame_id = "map";
    m.header.stamp = n_->get_clock()->now();
    m.pose.pose.position.x = pose[0];
    m.pose.pose.position.y = pose[1];
    m.pose.pose.orientation.z = std::sin(pose[2] / 2);
    m.pose.pose.orientation.w = std::cos(pose[2] / 2);
    m.pose.covariance[0] = m.pose.covariance[7] = 0.0025;
    m.pose.covariance[35] = 0.003;
    initial_->publish(m);
  }

  bool agrees(const Pose & pose, double dist = 0.10, double ang = 5 * M_PI / 180) {
    const auto s = slam();
    return s && std::hypot((*s)[0] - pose[0], (*s)[1] - pose[1]) < dist &&
           std::abs(map_tools::wrap((*s)[2] - pose[2])) < ang;
  }

  // One localization; nullopt when done, else why not.
  std::optional<std::string> attempt() {
    const auto pts = scan_points();
    if (!pts || pts->size() < 60) return "too few scan points";
    const double t0 = wall();
    const auto res = matcher_.localize(*pts, 0.18, 0.92, near_, radius_, yaw_);
    if (!res.pose) return near_ ? fmt("no fit within %g m of %s", radius_, near_text(near_).c_str()) : "no fit";
    const auto & pose = *res.pose;
    RCLCPP_INFO(n_->get_logger(), "fit (%+.3f, %+.3f, %+.1f deg) score %.2f, runner-up %.2f, %.1f s", pose[0], pose[1],
                pose[2] * 180 / M_PI, res.score, res.second, wall() - t0);
    if (near_ && res.score < 0.85)
      // Near the hint something always fits: a true fit scores 0.94-1.00, a robot elsewhere ~0.70 (run_corner.py).
      return fmt("best fit near %s scores only %.2f: is the robot in the start zone?", near_text(near_).c_str(), res.score);
    if (res.ambiguous) {
      const auto & p2 = *res.second_pose;
      return fmt("ambiguous: runner-up %.2f at (%+.2f, %+.2f) vs %.2f; turn or move the robot a little", res.second,
                 p2[0], p2[1], res.score);
    }
    for (int i = 0; i < 3; ++i) {
      send_initial(pose);
      if (spin(5.0, [&] { return agrees(pose); })) {
        const auto s = *slam();
        say(fmt("ok %.3f %.3f %.1f %.2f", s[0], s[1], s[2] * 180 / M_PI, res.score));
        return std::nullopt;
      }
    }
    const auto s = slam();
    return fmt("slam_toolbox did not settle on (%+.2f, %+.2f) (its pose %s)", pose[0], pose[1],
               s ? fmt("(%.3f, %.3f, %.3f)", (*s)[0], (*s)[1], (*s)[2]).c_str() : "None");
  }

  rclcpp::Node::SharedPtr n_;
  const map_tools::ScanMatcher & matcher_;
  double retry_s_;
  std::optional<Pt> near_;
  double radius_;
  std::optional<double> yaw_;
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<LaserScan::SharedPtr> scans_;
  bool wanted_ = true;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr req_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

int main(int argc, char ** argv) {
  Args args;
  try {
    args = parse(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "global_localize: error: " << e.what() << "\n";
    return 2;
  }
  const map_tools::MapGrid grid(args.map);
  const map_tools::ScanMatcher matcher(grid);
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("global_localize");
    GlobalLocalize(node, matcher, args).run();
  }
  rclcpp::shutdown();
}
