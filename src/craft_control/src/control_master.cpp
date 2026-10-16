// control_master: planning targets -> motor_controller
// sub: /planning/target_path, /planning/odom, /planning/arm_target_pose, /planning/clock_target_pose
// out: motor_controller (wheel_speed, arm_target_pose, clock_target_pose) - not a ROS topic

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "craft_control/lateral_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"

using namespace std::chrono_literals;

class ControlMaster : public rclcpp::Node
{
public:
  ControlMaster()
  : Node("control_master")
  {
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
    timer_ = create_wall_timer(20ms, [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    longitudinal_.set_target_v(0.0);  // TODO: target v from path
    auto v = longitudinal_.get_wheel_speed();
    auto w = lateral_.get_wheel_speed();
    std::array<double, 2> wheel_speed{v[0] + w[0], v[1] + w[1]};
    (void)wheel_speed;
    // TODO: send wheel_speed, arm_target_pose_, clock_target_pose_ to motor_controller (serial)
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
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
