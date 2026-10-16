// arm_encoder_processor: Arm encoder raw -> /arm/odom
// pub: /arm/odom

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class ArmEncoderProcessor : public rclcpp::Node
{
public:
  ArmEncoderProcessor()
  : Node("arm_encoder_processor")
  {
    arm_odom_pub_ = create_publisher<sensor_msgs::msg::JointState>("/arm/odom", 10);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    // TODO: read arm encoder raw
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmEncoderProcessor>());
  rclcpp::shutdown();
  return 0;
}
