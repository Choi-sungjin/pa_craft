// control_master: planning targets -> motor_controller
// sub: /planning/target_path, /planning/odom, /planning/arm_target_pose, /planning/clock_target_pose
// out: motor_controller (wheel_speed, arm_target_pose, clock_target_pose) - not a ROS topic

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "craft_control/lateral_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"

constexpr double kDt = 0.02;  // control period [s]

class ControlMaster : public rclcpp::Node
{
public:
  ControlMaster()
  : Node("control_master")
  {
    // wheel: TurtleBot3 Waffle Pi, XM430-W210-T
    const double wheel_radius = declare_parameter("wheel_radius", 0.033);  // [m]
    const double max_v = declare_parameter("max_v", 0.26);                 // [m/s]
    const double max_acc_v = declare_parameter("max_acc_v", 0.5);          // [m/s^2]
    max_wheel_ = declare_parameter("max_wheel", 7.8);                      // [rad/s]
    if (wheel_radius <= 0.0 || max_v <= 0.0 || max_acc_v <= 0.0 || max_wheel_ <= 0.0) {
      throw std::runtime_error("wheel_radius, max_v, max_acc_v, max_wheel must be > 0");
    }
    longitudinal_.set_params(wheel_radius, max_v, max_acc_v);

    target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/target_path", 10, [this](const nav_msgs::msg::Path & msg) {lateral_.set_path(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/planning/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {lateral_.set_odom(msg);});
    arm_target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/arm_target_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {arm_target_pose_ = msg;});
    clock_target_pose_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/planning/clock_target_pose", 10,
      [this](const std_msgs::msg::Float64 & msg) {clock_target_pose_ = msg.data;});
    timer_ = create_wall_timer(std::chrono::duration<double>(kDt), [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    const auto t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : kDt;  // real dt
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    longitudinal_.set_target_v(0.0);  // TODO: target v from path
    auto v = longitudinal_.get_wheel_speed(dt);
    auto w = lateral_.get_wheel_speed();
    double wl = v[0] + w[0];  // [rad/s]
    double wr = v[1] + w[1];
    // over wheel limit: scale both by the same ratio to keep curvature
    const double k = std::max({1.0, std::abs(wl) / max_wheel_, std::abs(wr) / max_wheel_});
    std::array<double, 2> wheel_speed{wl / k, wr / k};
    (void)wheel_speed;
    // TODO: send wheel_speed, arm_target_pose_, clock_target_pose_ to motor_controller (serial)
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
  double max_wheel_ = 7.8;
  std::optional<rclcpp::Time> last_t_;
  geometry_msgs::msg::PoseStamped arm_target_pose_;
  double clock_target_pose_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_target_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr clock_target_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlMaster>());
  rclcpp::shutdown();
  return 0;
}
