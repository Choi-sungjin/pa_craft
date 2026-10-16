// c310_processor: Logitech C310 -> image for object_pose_estimator
// pub: /vision/c310_image

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class C310Processor : public rclcpp::Node
{
public:
  C310Processor()
  : Node("c310_processor")
  {
    vision_c310_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/c310_image", 10);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    // TODO: grab frame from C310
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vision_c310_image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<C310Processor>());
  rclcpp::shutdown();
  return 0;
}
