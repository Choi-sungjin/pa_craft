// control_master: planning targets -> 2x OpenCR (serial)
// sub: /planning/target_path, /planning/odom, /planning/target_arm_joints
//      target_arm_joints: JointState from URDF IK, positions [rad] or [m]
// pub: /arm_state, /clock_state (Int32: 0 idle, 1 using, 2 fault, every tick)
// param: config/control.yaml (OpenCR ports, arm motor limits)
// serial (ASCII CSV, '\n' terminated, OpenCR must stream rx on its own):
//   arm OpenCR   tx: arm_j0,...,arm_j4,lm
//                rx: arm_state,arm_j0,...,arm_j4,lm[,gripper_current_a]
//   clock OpenCR tx: wheel_l,wheel_r,clock
//                rx: clock_state,wheel_l,wheel_r,clock

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "craft_control/arm_planner.hpp"
#include "craft_control/clock_planner.hpp"
#include "craft_control/gripper_planner.hpp"
#include "craft_control/lateral_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"

constexpr double kDt = 0.02;  // control period [s]

class OpencrSerial
{
public:
  explicit OpencrSerial(const std::string & port)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open OpenCR serial: " + port);
    }
    termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    cfsetspeed(&tty, B115200);  // OpenCR USB CDC ignores baud, kept for USB-UART adapters
    tty.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd_, TCSANOW, &tty);
  }
  ~OpencrSerial() {::close(fd_);}
  OpencrSerial(const OpencrSerial &) = delete;
  OpencrSerial & operator=(const OpencrSerial &) = delete;

  std::vector<std::string> read_lines()
  {
    char buf[256];
    ssize_t n;
    while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
      rx_buf_.append(buf, n);
    }
    std::vector<std::string> lines;
    size_t pos;
    while ((pos = rx_buf_.find('\n')) != std::string::npos) {
      lines.push_back(rx_buf_.substr(0, pos));
      rx_buf_.erase(0, pos + 1);
    }
    if (rx_buf_.size() > 1024) {  // garbage without '\n'
      rx_buf_.clear();
    }
    return lines;
  }

  bool write_values(const std::vector<double> & values)
  {
    std::string line;
    for (double x : values) {
      char b[32];
      std::snprintf(b, sizeof(b), "%.4f,", x);
      line += b;
    }
    line.back() = '\n';
    return ::write(fd_, line.data(), line.size()) == static_cast<ssize_t>(line.size());
  }

private:
  int fd_ = -1;
  std::string rx_buf_;
};

// "1.0,2,3\r" -> {1, 2, 3}; false on junk
static bool parse_csv(const std::string & line, std::vector<double> & v)
{
  const char * p = line.c_str();
  char * end;
  while (true) {
    double x = std::strtod(p, &end);
    if (end == p) {
      break;
    }
    v.push_back(x);
    p = (*end == ',') ? end + 1 : end;
  }
  return *end == '\0' || *end == '\r';
}

class ControlMaster : public rclcpp::Node
{
public:
  ControlMaster()
  : Node("control_master")
  {
    const auto min_angle = load_arm_param("arm_min_angle");
    const auto max_angle = load_arm_param("arm_max_angle");
    const auto arm_ik_joint_names = declare_parameter<std::vector<std::string>>(
      "arm_ik_joint_names", std::vector<std::string>{
        "slide_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"});
    const auto arm_motor_channels = declare_parameter<std::vector<int>>(
      "arm_motor_channels", std::vector<int>{5, 0, 1, 2, 3});
    const auto arm_joint_directions = load_arm_ik_param("arm_joint_directions");
    const auto arm_joint_offsets = load_arm_ik_param("arm_joint_offsets");
    arm_ik_mapping_confirmed_ = declare_parameter("arm_ik_mapping_confirmed", false);
    const auto lm_linear_joint_name = declare_parameter<std::string>("lm_linear_joint_name", "slide_joint");
    const double lm_pulley_radius = declare_parameter("lm_pulley_radius", 0.0);
    const double lm_gear_ratio = declare_parameter("lm_gear_ratio", 0.0);
    const int gripper_motor_channel = declare_parameter("gripper_motor_channel", 4);
    const double gripper_open_angle = declare_parameter("gripper_open_angle", 0.0);
    const double gripper_close_angle = declare_parameter("gripper_close_angle", 0.0);
    const double grasp_current_threshold_a = declare_parameter("grasp_current_threshold_a", 0.0);
    const double grasp_current_debounce = declare_parameter("grasp_current_debounce", 0.0);
    gripper_mapping_confirmed_ = declare_parameter("gripper_mapping_confirmed", false);
    if (arm_ik_joint_names.size() != craft_control::kArmIkJoints ||
      arm_motor_channels.size() != craft_control::kArmIkJoints)
    {
      throw std::runtime_error("arm_ik_joint_names and arm_motor_channels need 5 entries");
    }
    for (std::size_t i = 0; i < craft_control::kArmMotors; ++i) {
      if (!std::isfinite(min_angle[i]) || !std::isfinite(max_angle[i]) || min_angle[i] >= max_angle[i]) {
        throw std::runtime_error("bad arm limit at motor " + std::to_string(i) + " (config/control.yaml)");
      }
    }
    for (std::size_t i = 0; i < craft_control::kArmIkJoints; ++i) {
      if (!std::isfinite(arm_joint_directions[i]) || !std::isfinite(arm_joint_offsets[i]) ||
        std::abs(arm_joint_directions[i]) != 1.0)
      {
        throw std::runtime_error("bad arm calibration at IK joint " + std::to_string(i) + " (config/control.yaml)");
      }
    }
    arm_planner_.set_angle_limits(min_angle, max_angle);
    craft_control::ArmIkJointNames ik_joint_name_array;
    craft_control::ArmIkMotorChannels arm_motor_channel_array;
    std::copy(arm_ik_joint_names.begin(), arm_ik_joint_names.end(), ik_joint_name_array.begin());
    for (std::size_t i = 0; i < craft_control::kArmIkJoints; ++i) {
      if (arm_motor_channels[i] < 0 || arm_motor_channels[i] >= static_cast<int>(craft_control::kArmMotors)) {
        throw std::runtime_error("arm_motor_channels entries must be in [0, 5]");
      }
      arm_motor_channel_array[i] = static_cast<std::size_t>(arm_motor_channels[i]);
    }
    if (gripper_motor_channel < 0 || gripper_motor_channel >= static_cast<int>(craft_control::kArmMotors)) {
      throw std::runtime_error("gripper_motor_channel must be in [0, 5]");
    }
    if (std::find(arm_motor_channel_array.begin(), arm_motor_channel_array.end(),
      static_cast<std::size_t>(gripper_motor_channel)) != arm_motor_channel_array.end())
    {
      throw std::runtime_error("gripper_motor_channel overlaps an arm IK motor channel");
    }
    if (!arm_planner_.set_actuator_mapping(
      ik_joint_name_array, arm_motor_channel_array, arm_joint_directions, arm_joint_offsets,
      lm_linear_joint_name, lm_pulley_radius, lm_gear_ratio))
    {
      throw std::runtime_error("invalid arm IK joint or motor mapping in config/control.yaml");
    }
    gripper_motor_channel_ = static_cast<std::size_t>(gripper_motor_channel);
    if (!gripper_planner_.configure(
      min_angle[gripper_motor_channel_], max_angle[gripper_motor_channel_],
      gripper_open_angle, gripper_close_angle, grasp_current_threshold_a, grasp_current_debounce))
    {
      throw std::runtime_error("invalid gripper angle limits or open/close targets");
    }
    if (!arm_ik_mapping_confirmed_) {
      RCLCPP_WARN(get_logger(), "arm IK-to-motor mapping is unconfirmed; IK targets will be ignored");
    }
    if (!gripper_mapping_confirmed_) {
      RCLCPP_WARN(get_logger(), "gripper mapping is unconfirmed; gripper commands will be ignored");
    }
    if (!gripper_planner_.grasp_detection_configured()) {
      RCLCPP_WARN(get_logger(), "gripper current-based grasp detection is disabled until thresholds are calibrated");
    }

    // wheel: TurtleBot3 Waffle Pi, XM430-W210-T
    const double wheel_radius = declare_parameter("wheel_radius", 0.033);  // [m]
    const double max_v = declare_parameter("max_v", 0.26);                 // [m/s]
    const double max_acc_v = declare_parameter("max_acc_v", 0.5);          // [m/s^2]
    max_wheel_ = declare_parameter("max_wheel", 7.8);                      // [rad/s]
    if (wheel_radius <= 0.0 || max_v <= 0.0 || max_acc_v <= 0.0 || max_wheel_ <= 0.0) {
      throw std::runtime_error("wheel_radius, max_v, max_acc_v, max_wheel must be > 0");
    }
    longitudinal_.set_params(wheel_radius, max_v, max_acc_v);

    arm_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("arm_port"));
    clock_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("clock_port"));

    arm_state_pub_ = create_publisher<std_msgs::msg::Int32>("/arm_state", 10);
    gripper_grasped_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_grasped", 10);
    clock_state_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_state", 10);
    target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/target_path", 10, [this](const nav_msgs::msg::Path & msg) {lateral_.set_path(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/planning/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {lateral_.set_odom(msg);});
    arm_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/planning/target_arm_joints", 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_arm_ik_solution(msg);});
    gripper_target_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/gripper_command", 10, [this](const std_msgs::msg::Bool & msg) {
        if (!gripper_mapping_confirmed_) {
          return;
        }
        if (!gripper_planner_.set_target(msg.data)) {
          RCLCPP_WARN(get_logger(), "gripper planner is not configured");
        }
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(kDt), [this]() {on_timer();});
  }

private:
  craft_control::ArmJoints load_arm_param(const std::string & name)
  {
    const auto v = declare_parameter<std::vector<double>>(name);
    if (v.size() != craft_control::kArmMotors) {
      throw std::runtime_error(name + " needs 6 values (config/control.yaml)");
    }
    craft_control::ArmJoints a;
    std::copy(v.begin(), v.end(), a.begin());
    return a;
  }

  craft_control::ArmIkJointValues load_arm_ik_param(const std::string & name)
  {
    const auto values = declare_parameter<std::vector<double>>(name);
    if (values.size() != craft_control::kArmIkJoints) {
      throw std::runtime_error(name + " needs 5 entries (config/control.yaml)");
    }
    craft_control::ArmIkJointValues result;
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  bool read_arm()
  {
    bool updated = false;
    for (const auto & line : arm_serial_->read_lines()) {
      std::vector<double> v;
      if (!parse_csv(line, v) ||
        (v.size() != 1 + craft_control::kArmMotors && v.size() != 2 + craft_control::kArmMotors))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad arm OpenCR line: %s", line.c_str());
        continue;
      }
      if (!std::all_of(v.begin(), v.begin() + 1 + craft_control::kArmMotors, [](double value) {
        return std::isfinite(value);
      }))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "non-finite arm OpenCR feedback: %s", line.c_str());
        continue;
      }
      arm_state_ = static_cast<int>(v[0]);
      std::copy(v.begin() + 1, v.begin() + 1 + craft_control::kArmMotors, arm_joints_.begin());
      have_arm_fb_ = true;
      have_gripper_current_fb_ = v.size() == 2 + craft_control::kArmMotors &&
        std::isfinite(v.back());
      if (have_gripper_current_fb_) {
        gripper_current_a_ = v.back();
      }
      updated = true;
    }
    return updated;
  }

  void read_clock()
  {
    for (const auto & line : clock_serial_->read_lines()) {
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad clock OpenCR line: %s", line.c_str());
        continue;
      }
      clock_state_ = static_cast<int>(v[0]);
      // v[1], v[2]: wheel feedback (unused, odom comes from /planning/odom)
      clock_angle_ = v[3];
      have_clock_fb_ = true;
    }
  }

  craft_control::ArmJoints get_arm_pose() const
  {
    return arm_joints_;
  }

  void on_arm_ik_solution(const sensor_msgs::msg::JointState & msg)
  {
    if (!arm_ik_mapping_confirmed_) {
      return;
    }
    if (msg.name.empty() || msg.name.size() != msg.position.size()) {
      RCLCPP_WARN(get_logger(), "ignoring invalid arm IK JointState");
      return;
    }
    if (!arm_planner_.set_target_arm_pose(msg.name, msg.position)) {
      RCLCPP_WARN(get_logger(), "ignoring arm IK solution: joint mapping or LM calibration is incomplete");
    }
  }

  void on_timer()
  {
    const auto t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : kDt;  // real dt
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    const bool have_new_arm_feedback = read_arm();
    read_clock();
    if (have_new_arm_feedback && have_gripper_current_fb_) {
      gripper_planner_.update_current_feedback(gripper_current_a_, dt);
      std_msgs::msg::Bool grasped_msg;
      grasped_msg.data = gripper_planner_.is_grasped();
      gripper_grasped_pub_->publish(grasped_msg);
    } else if (have_new_arm_feedback && gripper_planner_.grasp_detection_configured()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "grasp detection needs gripper_current_a in arm feedback");
    }

    // no command before first feedback of each OpenCR, so motors never jump from 0
    if (have_arm_fb_) {
      const auto current_pose = get_arm_pose();
      auto arm_command = arm_planner_.compute_arm_command(current_pose);
      arm_command[gripper_motor_channel_] = gripper_planner_.compute_gripper_command(
        current_pose[gripper_motor_channel_]);
      if (!arm_serial_->write_values({arm_command.begin(), arm_command.end()})) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "arm OpenCR write failed");
      }
      std_msgs::msg::Int32 msg;
      msg.data = arm_planner_.get_arm_status(arm_state_);
      arm_state_pub_->publish(msg);
    }

    if (have_clock_fb_) {
      longitudinal_.set_target_v(0.0);  // TODO: target v from path
      auto v = longitudinal_.get_wheel_speed(dt);
      auto w = lateral_.get_wheel_speed();
      const double wl = v[0] + w[0];  // [rad/s]
      const double wr = v[1] + w[1];
      // over wheel limit: scale both by the same ratio to keep curvature
      const double k = std::max({1.0, std::abs(wl) / max_wheel_, std::abs(wr) / max_wheel_});
      const double clock_pose = clock_planner_.get_clock_pose(clock_angle_);
      if (!clock_serial_->write_values({wl / k, wr / k, clock_pose})) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "clock OpenCR write failed");
      }
      std_msgs::msg::Int32 msg;
      msg.data = clock_planner_.get_clock_status(clock_state_);
      clock_state_pub_->publish(msg);
    }
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
  double max_wheel_ = 7.8;
  std::optional<rclcpp::Time> last_t_;
  craft_control::ArmPlanner arm_planner_;
  craft_control::GripperPlanner gripper_planner_;
  craft_control::ClockPlanner clock_planner_;

  std::unique_ptr<OpencrSerial> arm_serial_;
  std::unique_ptr<OpencrSerial> clock_serial_;
  craft_control::ArmJoints arm_joints_{};
  bool arm_ik_mapping_confirmed_ = false;
  bool gripper_mapping_confirmed_ = false;
  std::size_t gripper_motor_channel_ = 4;
  double gripper_current_a_ = 0.0;
  bool have_gripper_current_fb_ = false;
  int arm_state_ = 0;
  bool have_arm_fb_ = false;
  double clock_angle_ = 0.0;
  int clock_state_ = 0;
  bool have_clock_fb_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_grasped_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlMaster>());
  rclcpp::shutdown();
  return 0;
}
