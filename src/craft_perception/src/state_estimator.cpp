// state_estimator: wheel odom + IMU + lidar odom fusion -> /state/odom
// sub: /wheel/odom, /imu/data, /lidar/odom
// pub: /state/odom

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator()
  : Node("state_estimator")
  {
    state_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/state/odom", 10);
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {on_wheel_odom(msg);});
    imu_data_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, [this](const sensor_msgs::msg::Imu & msg) {on_imu_data(msg);});
    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {on_lidar_odom(msg);});
  }

private:
  void on_wheel_odom(const nav_msgs::msg::Odometry & msg)
  {
    (void)msg;
    // TODO
  }

  void on_imu_data(const sensor_msgs::msg::Imu & msg)
  {
    (void)msg;
    // TODO
  }

  void on_lidar_odom(const nav_msgs::msg::Odometry & msg)
  {
    (void)msg;
    // TODO
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
