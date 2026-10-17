// control_master: planning targets -> 2x OpenCR (serial)
// sub: /planning/target_path, /planning/odom, /planning/target_arm_joints
//      target_arm_joints: JointState from URDF IK, positions [rad] or [m]
// pub: /arm_state, /clock_wheel_state (Int32: 0 idle, 1 using, 2 fault, every tick)
// param: config/control.yaml (OpenCR ports, arm motor limits)
// serial (ASCII CSV, '\n' terminated, OpenCR must stream rx on its own):
//   arm OpenCR   tx: slide,shoulder,elbow,wrist_pitch,tool_roll,gripper
//                rx: arm_state,slide,shoulder,elbow,wrist_pitch,tool_roll,gripper[,gripper_current_a]
//   clock+wheel OpenCR tx: wheel_l,wheel_r,clock
//                         rx: clock_wheel_state,wheel_l,wheel_r,clock

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
constexpr std::size_t kGripperMotorChannel = craft_control::kArmIkJoints;

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
    const auto arm_min_angle = load_array_param<craft_control::kArmIkJoints>("arm_min_angle");
    const auto arm_max_angle = load_array_param<craft_control::kArmIkJoints>("arm_max_angle");
    const auto arm_ik_joint_names = declare_parameter<std::vector<std::string>>(
      "arm_ik_joint_names", std::vector<std::string>{
        "slide_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"});
    const auto arm_joint_directions = load_array_param<craft_control::kArmIkJoints>("arm_joint_directions");
    const auto arm_joint_offsets = load_array_param<craft_control::kArmIkJoints>("arm_joint_offsets");
    arm_ik_mapping_confirmed_ = declare_parameter("arm_ik_mapping_confirmed", false);
    const auto lm_linear_joint_name = declare_parameter<std::string>("lm_linear_joint_name", "slide_joint");
    const double lm_pulley_radius = declare_parameter("lm_pulley_radius", 0.0);
    const double lm_gear_ratio = declare_parameter("lm_gear_ratio", 0.0);
    const double gripper_min_angle = declare_parameter("gripper_min_angle", 0.0);
    const double gripper_max_angle = declare_parameter("gripper_max_angle", 0.0);
    const double gripper_open_angle = declare_parameter("gripper_open_angle", 0.0);
    const double gripper_close_angle = declare_parameter("gripper_close_angle", 0.0);
    const double grasp_current_threshold_a = declare_parameter("grasp_current_threshold_a", 0.0);
    const double grasp_current_debounce = declare_parameter("grasp_current_debounce", 0.0);
    gripper_mapping_confirmed_ = declare_parameter("gripper_mapping_confirmed", false);

    // Validate the dynamic vector length before copying into the fixed-size array.
    if (arm_ik_joint_names.size() != craft_control::kArmIkJoints)
    {
      throw std::runtime_error("arm_ik_joint_names needs 5 entries");
    }
    if (!arm_planner_.set_angle_limits(arm_min_angle, arm_max_angle)) {
      throw std::runtime_error("invalid arm angle limits in config/control.yaml");
    }

    craft_control::ArmIkJointNames ik_joint_name_array;
    std::copy(arm_ik_joint_names.begin(), arm_ik_joint_names.end(), ik_joint_name_array.begin());
    if (!arm_planner_.set_actuator_mapping(
      ik_joint_name_array, arm_joint_directions, arm_joint_offsets,
      lm_linear_joint_name, lm_pulley_radius, lm_gear_ratio))
    {
      throw std::runtime_error("invalid arm IK joint or motor mapping in config/control.yaml");
    }
    if (!gripper_planner_.configure(
      gripper_min_angle, gripper_max_angle,
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

    const auto wheel_radius = load_positive_param("wheel_radius");
    const auto max_v = load_positive_param("max_v");
    const auto max_acc_v = load_positive_param("max_acc_v");
    max_wheel_ = load_positive_param("max_wheel");
    longitudinal_.set_params(wheel_radius, max_v, max_acc_v);

    arm_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("arm_port"));
    clock_wheel_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("clock_wheel_port"));

    arm_state_pub_ = create_publisher<std_msgs::msg::Int32>("/arm_state", 10);
    gripper_grasped_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_grasped", 10);
    clock_wheel_state_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_wheel_state", 10);
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
    timer_ = create_wall_timer(std::chrono::duration<double>(kDt), [this]() {control_loop();});
  }

private:
  template<std::size_t N>
  std::array<double, N> load_array_param(const std::string & name)
  {
    const auto values = declare_parameter<std::vector<double>>(name);
    if (values.size() != N) {
      throw std::runtime_error(name + " needs " + std::to_string(N) + " values (config/control.yaml)");
    }
    std::array<double, N> result;
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  double load_positive_param(const std::string & name)
  {
    const auto value = declare_parameter<double>(name, 0.0);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(name + " must be finite and > 0 (config/control.yaml)");
    }
    return value;
  }

  bool read_arm_feedback()
  {
    bool feedback_updated = false;
    for (const auto & line : arm_serial_->read_lines()) {
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 2 + craft_control::kArmMotors)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad arm OpenCR line: %s", line.c_str());
        continue;
      }
      if (!std::all_of(v.begin(), v.end(), [](double value) {
        return std::isfinite(value);
      }))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "non-finite arm OpenCR feedback: %s", line.c_str());
        continue;
      }
      arm_state_ = static_cast<int>(v[0]);
      std::copy(v.begin() + 1, v.begin() + 1 + craft_control::kArmMotors, arm_joints_.begin());
      have_arm_feedback_ = true;
      gripper_current_a_ = v.back();
      feedback_updated = true;
    }
    return feedback_updated;
  }

  void read_clock_wheel_feedback()
  {
    for (const auto & line : clock_wheel_serial_->read_lines()) {
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad clock+wheel OpenCR line: %s", line.c_str());
        continue;
      }
      clock_wheel_state_ = static_cast<int>(v[0]);
      // v[1], v[2]: wheel feedback (unused, odom comes from /planning/odom)
      clock_angle_ = v[3];
      have_clock_wheel_feedback_ = true;
    }
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

  void control_loop()
  {
    const auto t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : kDt;  // real dt
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    const bool arm_feedback_updated = read_arm_feedback();
    read_clock_wheel_feedback();
    if (have_arm_feedback_) {
      if (arm_feedback_updated) {
        gripper_planner_.update_current_feedback(gripper_current_a_, dt);
        std_msgs::msg::Bool grasped_msg;
        grasped_msg.data = gripper_planner_.is_grasped();
        gripper_grasped_pub_->publish(grasped_msg);
      }

      const auto arm_motor_commands = arm_planner_.compute_arm_command(arm_joints_);
      const double gripper_motor_command = gripper_planner_.compute_gripper_command(
        arm_joints_[kGripperMotorChannel]);
      craft_control::ArmJoints opencr_commands{};
      std::copy(arm_motor_commands.begin(), arm_motor_commands.end(), opencr_commands.begin());
      opencr_commands[kGripperMotorChannel] = gripper_motor_command;
      if (!arm_serial_->write_values({opencr_commands.begin(), opencr_commands.end()})) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "arm OpenCR write failed");
      }
      std_msgs::msg::Int32 msg;
      msg.data = arm_planner_.get_arm_status(arm_state_);
      arm_state_pub_->publish(msg);
    }

    if (have_clock_wheel_feedback_) {
      longitudinal_.set_target_v(0.0);  // TODO: target v from path
      auto v = longitudinal_.get_wheel_speed(dt);
      auto w = lateral_.get_wheel_speed();
      const double wl = v[0] + w[0];  // [rad/s]
      const double wr = v[1] + w[1];
      // over wheel limit: scale both by the same ratio to keep curvature
      const double k = std::max({1.0, std::abs(wl) / max_wheel_, std::abs(wr) / max_wheel_});
      const double clock_pose = clock_planner_.get_clock_pose(clock_angle_);
      if (!clock_wheel_serial_->write_values({wl / k, wr / k, clock_pose})) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "clock+wheel OpenCR write failed");
      }
      std_msgs::msg::Int32 msg;
      msg.data = clock_planner_.get_clock_wheel_status(clock_wheel_state_);
      clock_wheel_state_pub_->publish(msg);
    }
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
  double max_wheel_ = 0.0;
  std::optional<rclcpp::Time> last_t_;
  craft_control::ArmPlanner arm_planner_;
  craft_control::GripperPlanner gripper_planner_;
  craft_control::ClockPlanner clock_planner_;

  std::unique_ptr<OpencrSerial> arm_serial_;
  std::unique_ptr<OpencrSerial> clock_wheel_serial_;
  craft_control::ArmJoints arm_joints_{};
  bool arm_ik_mapping_confirmed_ = false;
  bool gripper_mapping_confirmed_ = false;
  double gripper_current_a_ = 0.0;
  int arm_state_ = 0;
  bool have_arm_feedback_ = false;
  double clock_angle_ = 0.0;
  int clock_wheel_state_ = 0;
  bool have_clock_wheel_feedback_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_grasped_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_wheel_state_pub_;
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
