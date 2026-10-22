// ik_rviz_node: 목표를 받아 IK를 풀고, 결과를 /joint_states 와 TF로 보내 RViz의 로봇을 움직인다.
//
// 좌표계
//   world      : 로봇 시작 위치의 base_link (바닥에 고정). 노브 받침대(knob_station)가 여기에 고정됨
//   base_link  : 로봇. 바퀴 이동은 world -> base_link 의 y 이동으로 표현 (이 노드가 TF 발행)
//
// 입력
//   /grasp_goal  std_msgs/Float64MultiArray  [x, y, z] 또는 [x, y, z, roll]  (world 기준 mm, deg)
//                수직 하향 잡기: 앞뒤(y)는 바퀴, 좌우(x)는 LM 가이드, 높이(z)는 팔, 그리퍼는 항상 아래(pitch -90)
//                물체가 로봇 기준 y = y_ref 에 오도록 로봇이 움직인 뒤 팔이 내려감
//   /ik_goal     std_msgs/Float64MultiArray  [x, y, z, pitch, roll] 또는 [x, y, z]  (base_link 기준 mm, deg)
//                팔만 움직임 (바퀴 이동 없음)
//   /ik_target   geometry_msgs/PoseStamped    (base_link 기준 m, 쿼터니언)
//   /knob_task   std_msgs/Float64MultiArray  [목표 노브 각도 deg]  카메라 정렬 노브 돌리기 시나리오 (아래)
// 출력
//   /joint_states            로봇 관절,  /station/joint_states  노브 관절
//   TF world -> base_link    바퀴 이동
//   /ik_solution             IK 체인 관절의 해 (m, rad)
//   /ik_target_marker        목표 표시 (초록 = 성공, 빨강 = 도달 불가)
//
// 카메라 정렬 노브 돌리기 시나리오 (/knob_task)
//   1) 정렬   : 바퀴로 노브를 y_ref 에 맞추고, LM 으로 카메라 광축을 노브 위에 맞춤 (그리퍼 수직 하향, roll 0)
//   2) 축 추출: 그 자세의 카메라 광축 = 노브 축 (실물에서는 영상 중앙 정렬로 얻는 값)
//   3) LM 이동: LM 가이드만 카메라-그리퍼 간격만큼 옆으로 -> 그리퍼 축이 노브 축 위에
//   4) 하강   : 팔로 잡는 높이까지 내리고 집게 닫기
//   5) 돌리기 : roll 로 노브를 목표 각도까지
//   6) 놓기   : 집게 열고 정렬 높이로 올라감
//
// 노브 잡기 시뮬레이션 (표시용)
//   집는 점이 노브 축 위(반경 knob_grab_radius, 높이 0~knob_grab_height)에 있고 비틀기 축이 노브 축과
//   나란하면 잡음 -> 집게가 닫히고, 이후 roll 을 바꾸면 노브가 같은 각도만큼 돈다 (노브 한계에서 멈춤).
//   노브에서 벗어나면 집게가 열린다.

#include "v75_arm_ik/arm_ik_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#ifndef DEFAULT_URDF
#define DEFAULT_URDF ""
#endif

using namespace v75;
using std::placeholders::_1;

namespace {
double smoothstep(double s) {
  s = std::clamp(s, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}
}  // namespace

class IkRvizNode : public rclcpp::Node {
 public:
  IkRvizNode() : Node("ik_rviz_node"), rng_(42) {
    std::string share;
    try {
      share = ament_index_cpp::get_package_share_directory("v75_robot_description");
    } catch (const std::exception&) {
    }
    const std::string urdf =
        declare_parameter<std::string>("urdf", share.empty() ? DEFAULT_URDF : share + "/urdf/v75_robot.urdf");
    const std::string station_urdf = declare_parameter<std::string>(
        "station_urdf", share.empty() ? "" : share + "/urdf/knob_station.urdf");
    const std::string tip = declare_parameter<std::string>("tip", "tool_tcp");
    const std::string base = declare_parameter<std::string>("base", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    move_time_ = declare_parameter<double>("move_time", 1.5);
    const double rate = declare_parameter<double>("rate", 30.0);
    y_ref_ = declare_parameter<double>("y_ref", -0.110);
    const std::string knob_joint = declare_parameter<std::string>("knob_joint", "knob_joint");
    const std::string roll_joint = declare_parameter<std::string>("roll_joint", "tool_roll_joint");
    jaw_joints_ = declare_parameter<std::vector<std::string>>(
        "jaw_joints", std::vector<std::string>{"jaw_upper_joint", "jaw_lower_joint"});
    jaw_open_ = declare_parameter<double>("jaw_open", -0.021);
    jaw_grip_ = declare_parameter<double>("jaw_grip", -0.0018);
    grab_radius_ = declare_parameter<double>("knob_grab_radius", 0.006);
    grab_height_ = declare_parameter<double>("knob_grab_height", 0.04);
    grab_angle_ = declare_parameter<double>("knob_grab_angle", 10.0) * kDeg;
    obs_tcp_z_ = declare_parameter<double>("observe_tcp_z", 0.130);  // 정렬 단계 끝점 높이 (world z)
    grasp_tcp_z_ = declare_parameter<double>("grasp_tcp_z", 0.116);  // 잡는 높이 (world z)
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");

    arm_ = buildArm(urdf, tip, base);
    const int n = arm_.model.nq;
    q_now_ = q_from_ = q_goal_ = VectorXd::Zero(n);
    for (int i = 0; i < n; ++i)
      if (arm_.chain_joints[i] == roll_joint) roll_idx_ = i;
    if (!station_urdf.empty()) setupKnob(station_urdf, knob_joint);
    jaw_now_ = jaw_from_ = jaw_goal_ = jaw_open_;

    RCLCPP_INFO(get_logger(), "URDF: %s", urdf.c_str());
    RCLCPP_INFO(get_logger(), "기준 %s -> 끝점 %s, IK 관절 %d개", arm_.base_name.c_str(), arm_.tip_name.c_str(), n);
    for (int i = 0; i < n; ++i)
      RCLCPP_INFO(get_logger(), "  %-18s [%8.2f, %8.2f] %s", arm_.chain_joints[i].c_str(),
                  toUser(arm_, i, arm_.model.lowerPositionLimit[i]),
                  toUser(arm_, i, arm_.model.upperPositionLimit[i]), arm_.prismatic[i] ? "mm" : "deg");
    if (has_knob_)
      RCLCPP_INFO(get_logger(), "노브 '%s' (world): 축 (%.1f, %.1f, %.1f) mm, 범위 [%.1f, %.1f] deg",
                  knob_name_.c_str(), knob_pos_w_.x() * 1000, knob_pos_w_.y() * 1000, knob_pos_w_.z() * 1000,
                  knob_lo_ / kDeg, knob_hi_ / kDeg);
    RCLCPP_INFO(get_logger(), "수직 하향 잡기 기준 위치 y_ref = %.1f mm (base_link)", y_ref_ * 1000);

    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    station_js_pub_ = create_publisher<sensor_msgs::msg::JointState>("station/joint_states", 10);
    sol_pub_ = create_publisher<sensor_msgs::msg::JointState>("ik_solution", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "ik_target_marker", rclcpp::QoS(1).transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    goal_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "ik_goal", 10, std::bind(&IkRvizNode::onGoalArray, this, _1));
    grasp_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "grasp_goal", 10, std::bind(&IkRvizNode::onGraspGoal, this, _1));
    knob_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "knob_task", 10, std::bind(&IkRvizNode::onKnobTask, this, _1));
    if (arm_.model.existFrame(camera_frame_)) {
      cam_frame_id_ = arm_.model.getFrameId(camera_frame_);
      pin::Data dd(arm_.model);
      forward(arm_, dd, VectorXd::Zero(n));
      pin::updateFramePlacement(arm_.model, dd, cam_frame_id_);
      tcp_to_cam_ = dd.oMf[arm_.tip].actInv(dd.oMf[cam_frame_id_]);
      RCLCPP_INFO(get_logger(), "카메라 '%s': 끝점 기준 (%.1f, %.1f, %.1f) mm", camera_frame_.c_str(),
                  tcp_to_cam_.translation().x() * 1000, tcp_to_cam_.translation().y() * 1000,
                  tcp_to_cam_.translation().z() * 1000);
    } else {
      RCLCPP_WARN(get_logger(), "카메라 프레임 '%s' 없음: /knob_task 사용 불가", camera_frame_.c_str());
    }
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "ik_target", 10, std::bind(&IkRvizNode::onGoalPose, this, _1));

    move_start_ = now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&IkRvizNode::tick, this));
    RCLCPP_INFO(get_logger(), "준비 완료. 예) ros2 run v75_arm_ik ik_goal --knob 90  /  ik_goal --grasp 80 -140 116 0");
  }

 private:
  // 노브 받침대 URDF에서 노브 축 위치/방향/한계를 읽는다 (station 루트 = world)
  void setupKnob(const std::string& urdf, const std::string& knob_joint) {
    pin::Model m;
    try {
      pin::urdf::buildModel(urdf, m);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "노브 받침대 URDF 로드 실패 (%s): %s", urdf.c_str(), e.what());
      return;
    }
    if (!m.existJointName(knob_joint)) {
      RCLCPP_WARN(get_logger(), "노브 관절 '%s' 없음: 노브 잡기 시뮬레이션 끔", knob_joint.c_str());
      return;
    }
    pin::Data d(m);
    pin::forwardKinematics(m, d, pin::neutral(m));
    const pin::JointIndex j = m.getJointId(knob_joint);
    const std::string sn = m.joints[j].shortname();
    int col = 2;
    if (sn.find("RX") != std::string::npos) col = 0;
    else if (sn.find("RY") != std::string::npos) col = 1;
    knob_pos_w_ = d.oMi[j].translation();
    knob_axis_w_ = d.oMi[j].rotation().col(col);
    const int iq = m.joints[j].idx_q();
    knob_lo_ = m.lowerPositionLimit[iq];
    knob_hi_ = m.upperPositionLimit[iq];
    knob_name_ = knob_joint;
    knob_now_ = knob_from_ = knob_goal_ = std::clamp(0.0, knob_lo_, knob_hi_);
    has_knob_ = true;
  }

  // 팔만 움직임 (base_link 기준)
  void onGoalArray(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const auto& d = msg->data;
    if (d.size() != 5 && d.size() != 3) {
      RCLCPP_WARN(get_logger(), "/ik_goal 은 [x,y,z,pitch,roll] (mm,deg) 또는 [x,y,z] 이어야 함 (받은 개수 %zu)", d.size());
      return;
    }
    const bool pos_only = d.size() == 3;
    const double pitch = pos_only ? 0.0 : d[3] * kDeg, roll = pos_only ? 0.0 : d[4] * kDeg;
    solveAndMove(makeTarget(d[0] / 1000.0, d[1] / 1000.0, d[2] / 1000.0, pitch, roll), pos_only, base_now_);
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto& p = msg->pose.position;
    const auto& o = msg->pose.orientation;
    const Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);
    solveAndMove(pin::SE3(quat.normalized().toRotationMatrix(), Vector3d(p.x, p.y, p.z)), false, base_now_);
  }

  // 수직 하향 잡기 (world 기준): 바퀴로 y 맞추고 팔은 y_ref 에서 수직으로 내려감
  void onGraspGoal(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const auto& d = msg->data;
    if (d.size() != 3 && d.size() != 4) {
      RCLCPP_WARN(get_logger(), "/grasp_goal 은 [x,y,z] 또는 [x,y,z,roll] (world 기준 mm, deg) 이어야 함");
      return;
    }
    const double xw = d[0] / 1000.0, yw = d[1] / 1000.0, zw = d[2] / 1000.0;
    const double roll = d.size() == 4 ? d[3] * kDeg : 0.0;
    const double base_goal = yw - y_ref_;  // 물체가 로봇 기준 y_ref 에 오도록
    RCLCPP_INFO(get_logger(), "잡기 목표 (world) (%.1f, %.1f, %.1f) mm, roll %.1f deg -> 바퀴 %+.1f mm 이동",
                xw * 1000, yw * 1000, zw * 1000, roll / kDeg, (base_goal - base_now_) * 1000);
    solveAndMove(makeTarget(xw, y_ref_, zw, -M_PI / 2, roll), false, base_goal);
  }

  // target 은 목표 로봇 위치(base_goal)에서의 base_link 기준
  void solveAndMove(const pin::SE3& target, bool pos_only, double base_goal) {
    if (!queue_.empty()) {
      queue_.clear();
      RCLCPP_WARN(get_logger(), "진행 중이던 시나리오 취소");
    }
    IkParams params;
    params.pos_only = pos_only;
    const IkResult r = solveIk(arm_, target, q_now_, params, rng_);
    publishMarker(target, r.ok, base_goal);

    double p, rl, yaw;
    decompose(target.rotation(), p, rl, yaw);
    const Vector3d t = target.translation() * 1000.0;
    if (!r.ok) {
      RCLCPP_WARN(get_logger(),
                  "도달 불가: (%.1f, %.1f, %.1f) mm (base_link), pitch %.1f, roll %.1f deg -> 오차 %.1f mm / %.1f deg. "
                  "로봇은 움직이지 않음",
                  t.x(), t.y(), t.z(), p / kDeg, rl / kDeg, r.pos_err * 1000, r.rot_err / kDeg);
      return;
    }
    std::string s;
    char buf[96];
    for (int i = 0; i < arm_.model.nq; ++i) {
      std::snprintf(buf, sizeof(buf), "\n  %-18s %8.2f %s", arm_.chain_joints[i].c_str(), toUser(arm_, i, r.q[i]),
                    arm_.prismatic[i] ? "mm" : "deg");
      s += buf;
    }
    RCLCPP_INFO(get_logger(), "해 찾음 (%.2f ms): base_link 기준 (%.1f, %.1f, %.1f) mm, pitch %.1f, roll %.1f deg%s",
                r.ms, t.x(), t.y(), t.z(), p / kDeg, rl / kDeg, s.c_str());

    sensor_msgs::msg::JointState sol;
    sol.header.stamp = now();
    sol.name = arm_.chain_joints;
    sol.position.assign(r.q.data(), r.q.data() + r.q.size());
    sol_pub_->publish(sol);

    // 진행 중이던 동작은 현재 값에서 이어서 시작
    q_from_ = q_now_;
    q_goal_ = r.q;
    base_from_ = base_now_;
    base_goal_ = base_goal;
    jaw_from_ = jaw_now_;
    knob_from_ = knob_now_;
    updateKnobGrasp(r.q, base_goal);
    in_segment_ = false;
    move_start_ = now();
  }

  // ---------------------------------------------------------------------------
  // 카메라 정렬 노브 돌리기 시나리오
  // ---------------------------------------------------------------------------
  struct Segment {
    std::string msg;
    double base, jaw, knob, duration;
    VectorXd q;
    std::function<void()> on_start;
  };

  int jointIndex(const std::string& name) const {
    for (int i = 0; i < arm_.model.nq; ++i)
      if (arm_.chain_joints[i] == name) return i;
    return -1;
  }

  void onKnobTask(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!has_knob_ || cam_frame_id_ == 0 || roll_idx_ < 0) {
      RCLCPP_WARN(get_logger(), "/knob_task: 노브 또는 카메라 프레임이 없어 실행 불가");
      return;
    }
    const double target = std::clamp((msg->data.empty() ? 90.0 : msg->data[0]) * kDeg, knob_lo_, knob_hi_);
    const int slide = jointIndex("slide_joint");
    const pin::Model& m = arm_.model;
    const double dur = move_time_;

    // 1) 정렬: 카메라 광축이 노브 축 위에 오도록 끝점 위치 계산 (그리퍼 수직 하향, roll 0)
    const double base1 = knob_pos_w_.y() - y_ref_;            // 바퀴: 노브를 y_ref 에
    pin::SE3 tcp1 = makeTarget(0, y_ref_, obs_tcp_z_, -M_PI / 2, 0.0);
    const Vector3d cam_off = tcp1.rotation() * tcp_to_cam_.translation();  // 끝점 -> 카메라 (base 기준)
    tcp1.translation() = Vector3d(knob_pos_w_.x() - cam_off.x(), y_ref_ - cam_off.y(), obs_tcp_z_);
    IkParams prm;
    IkResult r1 = solveIk(arm_, tcp1, q_now_, prm, rng_);
    if (!r1.ok) {
      RCLCPP_WARN(get_logger(), "정렬 자세 도달 불가 (끝점 %.1f, %.1f, %.1f mm). observe_tcp_z 를 낮추거나 노브 위치 확인",
                  tcp1.translation().x() * 1000, tcp1.translation().y() * 1000, tcp1.translation().z() * 1000);
      return;
    }
    // 2) 축 추출: 정렬 자세에서 카메라 광축이 지나는 점 (world)
    pin::Data dd(m);
    forward(arm_, dd, r1.q);
    pin::updateFramePlacement(m, dd, cam_frame_id_);
    pin::updateFramePlacement(m, dd, arm_.base);
    const pin::SE3 cam_b = dd.oMf[arm_.base].actInv(dd.oMf[cam_frame_id_]);
    const Vector3d axis_w = cam_b.translation() + Vector3d(0, base1, 0);
    // 3) LM 만 이동: 그리퍼 축을 카메라가 보던 축으로
    VectorXd q3 = r1.q;
    q3[slide] += cam_off.x();
    if (q3[slide] < m.lowerPositionLimit[slide] || q3[slide] > m.upperPositionLimit[slide]) {
      RCLCPP_WARN(get_logger(), "LM 이동 후 위치 %.1f mm 가 LM 범위 밖", q3[slide] * 1000);
      return;
    }
    // 4) 하강
    const pin::SE3 tcp4 = makeTarget(axis_w.x(), y_ref_, grasp_tcp_z_, -M_PI / 2, 0.0);
    IkResult r4 = solveIk(arm_, tcp4, q3, prm, rng_);
    if (!r4.ok) {
      RCLCPP_WARN(get_logger(), "잡는 높이 %.1f mm 도달 불가", grasp_tcp_z_ * 1000);
      return;
    }
    if (std::abs(r4.q[slide] - q3[slide]) > 1e-4)
      RCLCPP_WARN(get_logger(), "하강 중 LM 이 %.2f mm 움직임 (예상 0)", (r4.q[slide] - q3[slide]) * 1000);
    // 5) 돌리기: 위에서 내려다보면 비틀기와 노브 방향이 반대
    VectorXd q5 = r4.q;
    q5[roll_idx_] = r4.q[roll_idx_] - (target - knob_now_);
    if (q5[roll_idx_] < m.lowerPositionLimit[roll_idx_] || q5[roll_idx_] > m.upperPositionLimit[roll_idx_]) {
      RCLCPP_WARN(get_logger(), "roll %.1f deg 가 관절 범위 밖", q5[roll_idx_] / kDeg);
      return;
    }
    // 6) 놓고 올라가기
    VectorXd q6 = r1.q;
    q6[slide] = q3[slide];
    q6[roll_idx_] = q5[roll_idx_];

    const double k0 = knob_now_;
    const double dx = cam_off.x() * 1000;
    queue_.clear();
    queue_.push_back({"1) 정렬: 바퀴 + LM 으로 카메라 광축을 노브 위에", base1, jaw_open_, k0, dur, r1.q,
                      [this] { grasped_ = false; }});
    queue_.push_back({"2) 축 추출: 노브 축 = (" + fmt(axis_w.x()) + ", " + fmt(axis_w.y()) + ") mm (world)", base1,
                      jaw_open_, k0, 0.6, r1.q, [this, axis_w] { publishAxisMarker(axis_w); }});
    queue_.push_back({"3) LM 이동 " + fmt(dx / 1000) + " mm: 그리퍼 축을 노브 축 위로", base1, jaw_open_, k0, dur, q3,
                      nullptr});
    queue_.push_back({"4) 하강 + 집게 닫기", base1, jaw_grip_, k0, dur, r4.q, [this, k0] {
                        grasped_ = true;
                        knob_ref_ = k0;
                      }});
    queue_.push_back({"5) 돌리기: 노브 " + fmt(k0 / kDeg / 1000) + " -> " + fmt(target / kDeg / 1000) + " deg", base1,
                      jaw_grip_, target, dur, q5, nullptr});
    queue_.push_back({"6) 놓고 올라가기", base1, jaw_open_, target, dur, q6, [this] { grasped_ = false; }});
    RCLCPP_INFO(get_logger(), "노브 돌리기 시나리오 시작 (목표 %.1f deg)", target / kDeg);
    startNextSegment();
  }

  static std::string fmt(double v_m) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", v_m * 1000);
    return b;
  }

  void startNextSegment() {
    if (queue_.empty()) return;
    Segment sg = queue_.front();
    queue_.pop_front();
    RCLCPP_INFO(get_logger(), "%s", sg.msg.c_str());
    if (sg.on_start) sg.on_start();
    q_from_ = q_now_;
    q_goal_ = sg.q;
    base_from_ = base_now_;
    base_goal_ = sg.base;
    jaw_from_ = jaw_now_;
    jaw_goal_ = sg.jaw;
    knob_from_ = knob_now_;
    knob_goal_ = sg.knob;
    seg_time_ = sg.duration;
    move_start_ = now();
    in_segment_ = true;
  }

  void publishAxisMarker(const Vector3d& axis_w) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker c;
    c.header.frame_id = world_frame_;
    c.header.stamp = now();
    c.ns = "knob_axis";
    c.id = 0;
    c.type = visualization_msgs::msg::Marker::CYLINDER;
    c.action = visualization_msgs::msg::Marker::ADD;
    c.pose.position.x = axis_w.x();
    c.pose.position.y = axis_w.y();
    c.pose.position.z = knob_pos_w_.z() + 0.06;
    c.pose.orientation.w = 1.0;
    c.scale.x = c.scale.y = 0.003;
    c.scale.z = 0.12;
    c.color.a = 0.9;
    c.color.r = 1.0;
    c.color.g = 0.6;
    c.color.b = 0.0;
    arr.markers.push_back(c);
    marker_pub_->publish(arr);
  }

  // 목표 자세에서 노브를 잡는지 판단하고, 집게/노브 목표값을 정한다 (world 기준 비교)
  void updateKnobGrasp(const VectorXd& q_goal, double base_goal) {
    if (!has_knob_ || roll_idx_ < 0) return;
    pin::Data data(arm_.model);
    const pin::SE3 tcp_b = forward(arm_, data, q_goal);
    const Vector3d tcp_w = tcp_b.translation() + Vector3d(0.0, base_goal, 0.0);  // 바퀴 이동은 y 평행이동
    const Vector3d d = tcp_w - knob_pos_w_;
    const double h = d.dot(knob_axis_w_);
    const double radial = (d - h * knob_axis_w_).norm();
    const double align = tcp_b.rotation().col(1).dot(knob_axis_w_);  // base 와 world 는 회전 없음
    const bool on_knob = radial < grab_radius_ && h > 0.0 && h < grab_height_ && std::abs(align) > std::cos(grab_angle_);
    const double roll_goal = q_goal[roll_idx_];

    if (on_knob && !grasped_) {
      grasped_ = true;
      roll_ref_ = roll_goal;
      knob_ref_ = knob_now_;
      jaw_goal_ = jaw_grip_;
      knob_goal_ = knob_now_;
      RCLCPP_INFO(get_logger(), "노브 잡음 (축에서 %.1f mm, 높이 %.1f mm). 이제 roll 을 바꾸면 노브가 돈다",
                  radial * 1000, h * 1000);
    } else if (on_knob && grasped_) {
      const double sign = align > 0 ? 1.0 : -1.0;  // 위에서 내려다보면 비틀기와 노브 방향이 반대
      const double want = knob_ref_ + sign * (roll_goal - roll_ref_);
      knob_goal_ = std::clamp(want, knob_lo_, knob_hi_);
      jaw_goal_ = jaw_grip_;
      if (std::abs(want - knob_goal_) > 1e-6)
        RCLCPP_WARN(get_logger(), "노브 한계 [%.0f, %.0f] deg: 노브는 %.1f deg 에서 멈춤 (요청 %.1f deg)",
                    knob_lo_ / kDeg, knob_hi_ / kDeg, knob_goal_ / kDeg, want / kDeg);
      else
        RCLCPP_INFO(get_logger(), "노브 회전 -> %.1f deg", knob_goal_ / kDeg);
    } else if (!on_knob && grasped_) {
      grasped_ = false;
      jaw_goal_ = jaw_open_;
      knob_goal_ = knob_now_;
      RCLCPP_INFO(get_logger(), "노브 놓음 (노브 %.1f deg 에 유지)", knob_now_ / kDeg);
    } else {
      jaw_goal_ = jaw_open_;
      knob_goal_ = knob_now_;
    }
  }

  void tick() {
    const double T = std::max(1e-3, in_segment_ ? seg_time_ : move_time_);
    const double s = std::clamp((now() - move_start_).seconds() / T, 0.0, 1.0);
    // 바퀴가 먼저(앞 40%), 팔은 전체 구간에 걸쳐 움직임
    const double wb = smoothstep(s / 0.4);
    const double w = smoothstep(s);
    base_now_ = base_from_ + (base_goal_ - base_from_) * wb;
    q_now_ = q_from_ + (q_goal_ - q_from_) * w;
    knob_now_ = knob_from_ + (knob_goal_ - knob_from_) * w;
    const double wj = jaw_goal_ > jaw_from_ ? smoothstep((s - 0.7) / 0.3) : smoothstep(s / 0.3);
    jaw_now_ = jaw_from_ + (jaw_goal_ - jaw_from_) * wj;
    const auto stamp = now();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = arm_.base_name;
    tf.transform.translation.y = base_now_;
    tf.transform.rotation.w = 1.0;
    tf_->sendTransform(tf);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    for (const auto& name : arm_.all_joints) {
      js.name.push_back(name);
      const auto it = std::find(arm_.chain_joints.begin(), arm_.chain_joints.end(), name);
      double v = 0.0;
      if (it != arm_.chain_joints.end()) v = q_now_[it - arm_.chain_joints.begin()];
      else if (std::find(jaw_joints_.begin(), jaw_joints_.end(), name) != jaw_joints_.end()) v = jaw_now_;
      js.position.push_back(v);
    }
    js_pub_->publish(js);

    if (has_knob_) {
      sensor_msgs::msg::JointState ks;
      ks.header.stamp = stamp;
      ks.name.push_back(knob_name_);
      ks.position.push_back(knob_now_);
      station_js_pub_->publish(ks);
    }

    if (s >= 1.0 && in_segment_) {
      in_segment_ = false;
      if (!queue_.empty()) startNextSegment();
      else RCLCPP_INFO(get_logger(), "시나리오 완료 (노브 %.1f deg)", knob_now_ / kDeg);
    }
  }

  // 목표 표시 (world 기준으로 그림)
  void publishMarker(const pin::SE3& target_b, bool ok, double base_goal) {
    visualization_msgs::msg::MarkerArray arr;
    const Vector3d p = target_b.translation() + Vector3d(0.0, base_goal, 0.0);
    const Eigen::Quaterniond q(target_b.rotation());

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = world_frame_;
    sphere.header.stamp = now();
    sphere.ns = "ik_target";
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = p.x();
    sphere.pose.position.y = p.y();
    sphere.pose.position.z = p.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
    sphere.color.a = 0.9;
    sphere.color.r = ok ? 0.1 : 0.9;
    sphere.color.g = ok ? 0.8 : 0.1;
    sphere.color.b = 0.2;
    arr.markers.push_back(sphere);

    visualization_msgs::msg::Marker arrow = sphere;  // 접근 방향 (끝점 +y)
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    const Eigen::Quaterniond qa = q * Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitZ()));
    arrow.pose.orientation.w = qa.w();
    arrow.pose.orientation.x = qa.x();
    arrow.pose.orientation.y = qa.y();
    arrow.pose.orientation.z = qa.z();
    arrow.scale.x = 0.06;
    arrow.scale.y = arrow.scale.z = 0.006;
    arr.markers.push_back(arrow);
    marker_pub_->publish(arr);
  }

  Arm arm_;
  std::mt19937 rng_;
  VectorXd q_now_, q_from_, q_goal_;
  double move_time_ = 1.5, y_ref_ = -0.110;
  rclcpp::Time move_start_;
  int roll_idx_ = -1;
  std::string world_frame_;

  // 시나리오
  std::deque<Segment> queue_;
  bool in_segment_ = false;
  double seg_time_ = 1.0, obs_tcp_z_ = 0.130, grasp_tcp_z_ = 0.116;
  std::string camera_frame_;
  pin::FrameIndex cam_frame_id_ = 0;
  pin::SE3 tcp_to_cam_ = pin::SE3::Identity();

  // 바퀴 (world 에서 base_link 의 y 위치)
  double base_now_ = 0, base_from_ = 0, base_goal_ = 0;

  // 노브 / 집게
  bool has_knob_ = false, grasped_ = false;
  std::string knob_name_;
  Vector3d knob_pos_w_ = Vector3d::Zero(), knob_axis_w_ = Vector3d::UnitZ();
  double knob_lo_ = 0, knob_hi_ = 0;
  double knob_now_ = 0, knob_from_ = 0, knob_goal_ = 0, knob_ref_ = 0, roll_ref_ = 0;
  std::vector<std::string> jaw_joints_;
  double jaw_open_ = -0.021, jaw_grip_ = -0.0018, jaw_now_ = 0, jaw_from_ = 0, jaw_goal_ = 0;
  double grab_radius_ = 0.006, grab_height_ = 0.04, grab_angle_ = 10.0 * kDeg;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_, station_js_pub_, sol_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_sub_, grasp_sub_, knob_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IkRvizNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ik_rviz_node"), "시작 실패: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
