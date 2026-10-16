// object_detector: RealSense image -> detected objects (pucks)
// sub: /vision/image
// pub: /vision/object_detection

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector()
  : Node("object_detector")
  {
    vision_object_detection_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/vision/object_detection", 10);
    vision_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/vision/image", 10, [this](const sensor_msgs::msg::Image & msg) {on_vision_image(msg);});
  }

private:
  void on_vision_image(const sensor_msgs::msg::Image & msg)
  {
    (void)msg;
    // TODO
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr vision_object_detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
