// ik_goal: 목표를 한 번 보내는 도구 (ros2 topic pub 의 YAML 입력 대신 사용)
//   ros2 run v75_arm_ik ik_goal X Y Z PITCH ROLL          팔만 (base_link 기준 mm, deg) -> /ik_goal
//   ros2 run v75_arm_ik ik_goal X Y Z                     팔만, 위치만
//   ros2 run v75_arm_ik ik_goal --grasp X Y Z [ROLL]       수직 하향 잡기 (world 기준 mm, deg) -> /grasp_goal
//                                                         앞뒤는 바퀴, 좌우는 LM, 높이는 팔
//   ros2 run v75_arm_ik ik_goal --knob [ANGLE]            카메라 정렬 노브 돌리기 시나리오 (목표 각도 deg, 기본 90)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv) {
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  bool grasp = false, knob = false;
  std::vector<std::string> v;
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--grasp") grasp = true;
    else if (args[i] == "--knob") knob = true;
    else v.push_back(args[i]);
  }
  if (knob && v.empty()) v.push_back("90");
  const bool ok = knob ? v.size() == 1 : grasp ? (v.size() == 3 || v.size() == 4) : (v.size() == 5 || v.size() == 3);
  if (!ok) {
    std::fprintf(stderr,
                 "사용법:\n"
                 "  ros2 run v75_arm_ik ik_goal X Y Z [PITCH ROLL]     팔만 (base_link 기준 mm, deg)\n"
                 "  ros2 run v75_arm_ik ik_goal --grasp X Y Z [ROLL]   수직 하향 잡기 (world 기준 mm, deg)\n"
                 "  ros2 run v75_arm_ik ik_goal --knob [ANGLE]         카메라 정렬 노브 돌리기 (목표 deg, 기본 90)\n");
    rclcpp::shutdown();
    return 1;
  }
  auto node = std::make_shared<rclcpp::Node>("ik_goal_sender");
  auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      knob ? "knob_task" : grasp ? "grasp_goal" : "ik_goal", 10);
  std_msgs::msg::Float64MultiArray msg;
  for (const auto& s : v) msg.data.push_back(std::atof(s.c_str()));

  // ik_rviz_node 와 연결될 때까지 최대 3초 대기
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (pub->get_subscription_count() == 0) {
    RCLCPP_ERROR(node->get_logger(), "구독하는 노드가 없음. ik_demo.launch.py 가 실행 중인지 확인");
    rclcpp::shutdown();
    return 2;
  }
  pub->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  RCLCPP_INFO(node->get_logger(), "%s 로 보냄: %zu개 값", knob ? "/knob_task" : grasp ? "/grasp_goal" : "/ik_goal", msg.data.size());
  rclcpp::shutdown();
  return 0;
}
