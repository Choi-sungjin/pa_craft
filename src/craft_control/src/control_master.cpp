// control_master: planning targets -> 2x OpenCR (serial)
// sub: /planning/target_path, /planning/odom, /planning/target_arm_joints
//      /planning/target_v (Float64, m/s), /planning/target_clock_slot (Int32, 1..12)
//      target_arm_joints: JointState from URDF IK, positions [rad] or [m]
// pub: /arm_state, /wheel_state, /clock_state (Int32: 0 idle, 1 using, 2 fault, every tick)
//      /clock_slot (Int32: 1..12, 0 between slots), /clock_target_reached (Bool)
// param: config/control.yaml (OpenCR ports, arm motor limits)
// serial (ASCII CSV, '\n' terminated, OpenCR must stream rx on its own):
//   arm OpenCR   tx: slide,shoulder,elbow,wrist_pitch,tool_roll,gripper
//                rx: arm_state,slide,shoulder,elbow,wrist_pitch,tool_roll,gripper[,gripper_current_a]
//   clock+wheel OpenCR tx: wheel_l,wheel_r,clock
//                         rx: wheel_state,clock_state,wheel_l,wheel_r,clock

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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

#include "craft_control/arm_controller.hpp"
#include "craft_control/clock_controller.hpp"
#include "craft_control/gripper_controller.hpp"
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
    if (!arm_controller_.set_angle_limits(arm_min_angle, arm_max_angle)) {
      throw std::runtime_error("invalid arm angle limits in config/control.yaml");
    }

    craft_control::ArmIkJointNames ik_joint_name_array;
    std::copy(arm_ik_joint_names.begin(), arm_ik_joint_names.end(), ik_joint_name_array.begin());
    if (!arm_controller_.set_actuator_mapping(
      ik_joint_name_array, arm_joint_directions, arm_joint_offsets,
      lm_linear_joint_name, lm_pulley_radius, lm_gear_ratio))
    {
      throw std::runtime_error("invalid arm IK joint or motor mapping in config/control.yaml");
    }
    if (!gripper_controller_.configure(
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
    if (!gripper_controller_.grasp_detection_configured()) {
      RCLCPP_WARN(get_logger(), "gripper current-based grasp detection is disabled until thresholds are calibrated");
    }

    wheel_radius_ = load_positive_param("wheel_radius");
    const auto wheel_separation = load_positive_param("wheel_separation");
    const auto max_v = load_positive_param("max_v");
    const auto max_acc_v = load_positive_param("max_acc_v");
    max_wheel_ = load_positive_param("max_wheel");
    const auto lookahead_distance = load_positive_param("lookahead_distance");
    const auto wheel_goal_tolerance = load_positive_param("wheel_goal_tolerance");
    longitudinal_.set_params(wheel_radius_, max_v, max_acc_v);
    if (!lateral_.set_params(
      wheel_radius_, wheel_separation, lookahead_distance, wheel_goal_tolerance))
    {
      throw std::runtime_error("invalid lateral controller parameters");
    }
    clock_slot_count_ = declare_parameter<int>("clock_slot_count");
    const auto clock_slot_angle = load_positive_param("clock_slot_angle");
    const auto clock_zero_angle = declare_parameter<double>("clock_zero_angle");
    const auto clock_direction = declare_parameter<double>("clock_direction");
    const auto clock_angle_tolerance = load_positive_param("clock_angle_tolerance");
    if (!clock_controller_.configure(
      clock_slot_count_, clock_slot_angle, clock_zero_angle, clock_direction, clock_angle_tolerance))
    {
      throw std::runtime_error("invalid clock geometry or angle tolerance in config/control.yaml");
    }

    arm_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("arm_port"));
    clock_wheel_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("clock_wheel_port"));

    arm_state_pub_ = create_publisher<std_msgs::msg::Int32>("/arm_state", 10);
    gripper_grasped_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_grasped", 10);
    wheel_state_pub_ = create_publisher<std_msgs::msg::Int32>("/wheel_state", 10);
    clock_state_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_state", 10);
    clock_slot_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_slot", 10);
    clock_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/clock_target_reached", 10);
    target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/target_path", 10, [this](const nav_msgs::msg::Path & msg) {
        if (!lateral_.set_path(msg) && !msg.poses.empty()) {
          RCLCPP_WARN(get_logger(), "ignoring invalid wheel path: check coordinates and frame_id");
        }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/planning/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {
        if (!lateral_.set_odom(msg)) {
          RCLCPP_WARN(get_logger(), "ignoring invalid wheel odometry");
        }
      });
    wheel_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/planning/target_v", 10,
      [this](const std_msgs::msg::Float64 & msg) {update_wheel_target(msg);});
    clock_target_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/planning/target_clock_slot", 10,
      [this](const std_msgs::msg::Int32 & msg) {update_clock_target(msg);});
    arm_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/planning/target_arm_joints", 10,
      [this](const sensor_msgs::msg::JointState & msg) {update_arm_target(msg);});
    gripper_target_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/gripper_command", 10, [this](const std_msgs::msg::Bool & msg) {
        if (!gripper_mapping_confirmed_) {
          return;
        }
        if (!gripper_controller_.set_target(msg.data)) {
          RCLCPP_WARN(get_logger(), "gripper controller is not configured");
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
    const auto lines = arm_serial_->read_lines();
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) { // reverse order: newest first
      const auto & line = *it;
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 2 + craft_control::kArmMotors) // 2: arm_state + gripper_current_a
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
      arm_joints_[0] = v[1];  // slide
      arm_joints_[1] = v[2];  // shoulder
      arm_joints_[2] = v[3];  // elbow
      arm_joints_[3] = v[4];  // wrist pitch
      arm_joints_[4] = v[5];  // tool roll
      arm_joints_[5] = v[6];  // gripper
      gripper_current_a_ = v[7];

      have_arm_feedback_ = true; // 정상 arm 피드백을 한 번이라도 받은 적이 있는가
      return true;
    }
    return false;
  }

  bool read_clock_wheel_feedback()
  {
    const auto lines = clock_wheel_serial_->read_lines();
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) { // reverse order: newest first
      const auto & line = *it;
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 5) { // 5: wheel_state, clock_state, wheel_l, wheel_r, clock
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad clock+wheel OpenCR line: %s", line.c_str());
        continue;
      }
      if (!std::all_of(v.begin(), v.end(), [](double value) {
        return std::isfinite(value);
      }))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "non-finite clock+wheel OpenCR feedback: %s", line.c_str());
        continue;
      }
      wheel_state_ = static_cast<int>(v[0]);
      clock_state_ = static_cast<int>(v[1]);
      wheel_left_feedback_ = v[2];
      wheel_right_feedback_ = v[3];
      clock_angle_ = v[4];

      have_clock_wheel_feedback_ = true; // 정상 clock & wheel 피드백을 한 번이라도 받은 적이 있는가
      return true;
    }
    return false;
  }

  void update_arm_target(const sensor_msgs::msg::JointState & msg)
  {
    if (!arm_ik_mapping_confirmed_) {
      return;
    }
    if (msg.name.empty() || msg.name.size() != msg.position.size()) {
      RCLCPP_WARN(get_logger(), "ignoring invalid arm IK JointState");
      return;
    }
    if (!arm_controller_.set_target_arm_pose(msg.name, msg.position)) {
      RCLCPP_WARN(get_logger(), "ignoring arm IK solution: joint mapping or LM calibration is incomplete");
    }
  }

  void update_wheel_target(const std_msgs::msg::Float64 & msg)
  {
    if (!std::isfinite(msg.data)) {
      RCLCPP_WARN(get_logger(), "ignoring non-finite wheel target velocity");
      return;
    }
    wheel_target_v_ = msg.data;
  }

  void update_clock_target(const std_msgs::msg::Int32 & msg)
  {
    if (msg.data < 1 || msg.data > clock_slot_count_) {
      RCLCPP_WARN(get_logger(), "ignoring clock target: slot must be 1..%d", clock_slot_count_);
      return;
    }
    // Wait for a fresh position before choosing the nearest equivalent motor angle.
    pending_clock_slot_ = msg.data;
    clock_target_sent_ = false;
    std_msgs::msg::Bool reached;
    reached.data = false;
    clock_target_reached_pub_->publish(reached);
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
    const bool clock_wheel_feedback_updated = read_clock_wheel_feedback();
    if (clock_wheel_feedback_updated && pending_clock_slot_) {
      if (clock_controller_.set_target_slot(*pending_clock_slot_, clock_angle_)) {
        pending_clock_slot_.reset();
      }
    }
    if (have_arm_feedback_) {
      if (arm_feedback_updated) {
        gripper_controller_.update_current_feedback(gripper_current_a_, dt);
        std_msgs::msg::Bool grasped_msg;
        grasped_msg.data = gripper_controller_.is_grasped();
        gripper_grasped_pub_->publish(grasped_msg);
      }

      const auto arm_motor_commands = arm_controller_.compute_arm_command(arm_joints_);
      const double gripper_motor_command = gripper_controller_.compute_gripper_command(
        arm_joints_[kGripperMotorChannel]);
      craft_control::ArmJoints opencr_commands{};
      std::copy(arm_motor_commands.begin(), arm_motor_commands.end(), opencr_commands.begin());
      opencr_commands[kGripperMotorChannel] = gripper_motor_command;
      if (!arm_serial_->write_values({opencr_commands.begin(), opencr_commands.end()})) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "arm OpenCR write failed");
      }
      std_msgs::msg::Int32 msg;
      msg.data = arm_controller_.get_arm_status(arm_state_);
      arm_state_pub_->publish(msg);
    }

    if (have_clock_wheel_feedback_) {
      if (lateral_.can_track(wheel_target_v_)) {
        longitudinal_.set_target_v(wheel_target_v_);
      } else {
        longitudinal_.stop();  // no valid path/odom, or already at the path goal
      }
      const auto v = longitudinal_.get_wheel_speed(dt);
      const double linear_speed = wheel_radius_ * (v[0] + v[1]) / 2.0;
      const auto w = lateral_.get_wheel_speed(linear_speed);
      const double wl = v[0] + w[0];  // [rad/s]
      const double wr = v[1] + w[1];
      // over wheel limit: scale both by the same ratio to keep curvature
      const double k = std::max({1.0, std::abs(wl) / max_wheel_, std::abs(wr) / max_wheel_});
      const double clock_pose = clock_controller_.get_clock_pose(clock_angle_);
      const bool command_sent = clock_wheel_serial_->write_values({wl / k, wr / k, clock_pose});
      if (!command_sent) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "clock+wheel OpenCR write failed");
      }
      if (command_sent && !pending_clock_slot_) {
        clock_target_sent_ = true;
      }
      std_msgs::msg::Int32 wheel_state_msg;
      wheel_state_msg.data = wheel_state_;
      wheel_state_pub_->publish(wheel_state_msg);
      std_msgs::msg::Int32 clock_state_msg;
      clock_state_msg.data = clock_controller_.get_clock_status(clock_state_);
      clock_state_pub_->publish(clock_state_msg);
      if (clock_wheel_feedback_updated) {
        std_msgs::msg::Int32 slot_msg;
        slot_msg.data = clock_state_ == 0 ? clock_controller_.get_current_slot(clock_angle_) : 0;
        clock_slot_pub_->publish(slot_msg);
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = clock_target_sent_ && !pending_clock_slot_ && command_sent &&
          clock_controller_.is_complete(clock_angle_, clock_state_);
        clock_target_reached_pub_->publish(reached_msg);
      }
    }
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
  double max_wheel_ = 0.0;
  double wheel_radius_ = 0.0;
  double wheel_target_v_ = 0.0;
  std::optional<rclcpp::Time> last_t_;
  craft_control::ArmController arm_controller_;
  craft_control::GripperController gripper_controller_;
  craft_control::ClockController clock_controller_;

  std::unique_ptr<OpencrSerial> arm_serial_;
  std::unique_ptr<OpencrSerial> clock_wheel_serial_;
  craft_control::ArmJoints arm_joints_{};
  bool arm_ik_mapping_confirmed_ = false;
  bool gripper_mapping_confirmed_ = false;
  double gripper_current_a_ = 0.0;
  int arm_state_ = 0;
  bool have_arm_feedback_ = false;
  double wheel_left_feedback_ = 0.0;
  double wheel_right_feedback_ = 0.0;
  double clock_angle_ = 0.0;
  int wheel_state_ = 0;
  int clock_state_ = 0;
  bool have_clock_wheel_feedback_ = false;
  std::optional<int> pending_clock_slot_;
  int clock_slot_count_ = 0;
  bool clock_target_sent_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_grasped_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wheel_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_slot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clock_target_reached_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr clock_target_sub_;
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
