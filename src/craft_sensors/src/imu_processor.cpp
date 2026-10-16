// imu_processor: IMU raw -> /imu/data
// pub: /imu/data

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuProcessor : public rclcpp::Node
{
public:
  ImuProcessor()
  : Node("imu_processor")
  {
    imu_data_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    // TODO: read IMU raw
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuProcessor>());
  rclcpp::shutdown();
  return 0;
}
