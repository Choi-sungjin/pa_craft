// wheel_encoder_processor: Wheel encoder tick -> /wheel/odom
// pub: /wheel/odom

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class WheelEncoderProcessor : public rclcpp::Node
{
public:
  WheelEncoderProcessor()
  : Node("wheel_encoder_processor")
  {
    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    // TODO: read encoder tick, integrate odometry
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEncoderProcessor>());
  rclcpp::shutdown();
  return 0;
}
