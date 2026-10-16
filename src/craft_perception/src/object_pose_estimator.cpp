// object_pose_estimator: C310 image -> object pose
// sub: /vision/c310_image
// pub: /vision/object_pose

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

class ObjectPoseEstimator : public rclcpp::Node
{
public:
  ObjectPoseEstimator()
  : Node("object_pose_estimator")
  {
    vision_object_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vision/object_pose", 10);
    vision_c310_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/vision/c310_image", 10, [this](const sensor_msgs::msg::Image & msg) {on_vision_c310_image(msg);});
  }

private:
  void on_vision_c310_image(const sensor_msgs::msg::Image & msg)
  {
    (void)msg;
    // TODO
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_object_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_c310_image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPoseEstimator>());
  rclcpp::shutdown();
  return 0;
}
