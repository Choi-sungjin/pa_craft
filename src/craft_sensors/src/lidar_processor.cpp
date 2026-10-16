// lidar_processor: LiDAR x2 scan -> merged scan
// sub: /lidar/front/scan, /lidar/rear/scan
// pub: /lidar/merged

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarProcessor : public rclcpp::Node
{
public:
  LidarProcessor()
  : Node("lidar_processor")
  {
    lidar_merged_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/merged", 10);
    lidar_front_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar/front/scan", 10, [this](const sensor_msgs::msg::LaserScan & msg) {on_lidar_front_scan(msg);});
    lidar_rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar/rear/scan", 10, [this](const sensor_msgs::msg::LaserScan & msg) {on_lidar_rear_scan(msg);});
  }

private:
  void on_lidar_front_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    (void)msg;
    // TODO
  }

  void on_lidar_rear_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    (void)msg;
    // TODO
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_merged_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_front_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_rear_scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProcessor>());
  rclcpp::shutdown();
  return 0;
}
