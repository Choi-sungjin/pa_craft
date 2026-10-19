// mission_fsm: mission state machine (page 2 of craft_data_flowchart.drawio)
// sub: /vision/aruco, /control/arm_lm_state
// pub: /planning/mission

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MissionFsm : public rclcpp::Node
{
public:
  MissionFsm()
  : Node("mission_fsm")
  {
    mission_pub_ = create_publisher<std_msgs::msg::String>("/planning/mission", 10);
    aruco_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/vision/aruco", 10, [this](const std_msgs::msg::Int32MultiArray & msg) {on_aruco(msg);});
    arm_lm_state_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/control/arm_lm_state", 10, [this](const std_msgs::msg::Int32 & msg) {on_arm_lm_state(msg);});
    timer_ = create_wall_timer(100ms, [this]() {mission_loop();});
  }

private:
  enum class State { INIT, QR_RECOGNITION, MAIN_MISSION, SUB_MISSION, DONE };

  void on_aruco(const std_msgs::msg::Int32MultiArray & msg)
  {
    (void)msg;
    // TODO: decide mission order (Main -> Sub / Sub -> Main)
  }

  void on_arm_lm_state(const std_msgs::msg::Int32 & msg)  // 0 idle, 1 using, 2 fault
  {
    (void)msg;
    // TODO: count loaded pucks / knob turned, advance state
  }

  void mission_loop()
  {
    std_msgs::msg::String msg;
    msg.data = to_string(state_);
    mission_pub_->publish(msg);
  }

  static std::string to_string(State s)
  {
    switch (s) {
      case State::INIT: return "INIT";
      case State::QR_RECOGNITION: return "QR_RECOGNITION";
      case State::MAIN_MISSION: return "MAIN_MISSION";
      case State::SUB_MISSION: return "SUB_MISSION";
      case State::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  State state_ = State::INIT;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_lm_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionFsm>());
  rclcpp::shutdown();
  return 0;
}
