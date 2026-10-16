// obstacle_detector: merged scan -> obstacles
// sub: /lidar/merged
// pub: /lidar/obstacle

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    lidar_obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/lidar/obstacle", 10);
    lidar_merged_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar/merged", 10, [this](const sensor_msgs::msg::LaserScan & msg) {on_lidar_merged(msg);});
  }

private:
  void on_lidar_merged(const sensor_msgs::msg::LaserScan & msg)
  {
    (void)msg;
    // TODO
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lidar_obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_merged_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
