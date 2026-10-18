// control_master: planning targets -> 2x OpenCR (serial)
// sub: /planning/target_path, /planning/odom, /planning/target_arm_joints
//      /planning/target_v (Float64, m/s), /planning/target_clock_slot (Int32, 1..12)
//      target_arm_joints: JointState from URDF IK, positions [rad] or [m]
// pub: /arm_lm_state, /wheel_state, /clock_state (Int32: 0 idle, 1 using, 2 fault, every tick)
//      /clock_slot (Int32: 1..12, 0 between slots), /clock_target_reached (Bool)
//      /arm_target_reached, /lm_target_reached, /gripper_target_reached, /wheel_target_reached (Bool)
//      reached topics publish false immediately when a new target is stored
// param: config/control.yaml (OpenCR ports, arm motor limits)
// serial (ASCII CSV, '\n' terminated, OpenCR must stream rx on its own):
//   shared channels first (same order as tx), board-specific state/current last
//   arm+LM OpenCR      tx: slide,shoulder,elbow,wrist_pitch,tool_roll,gripper
//                      rx: slide,shoulder,elbow,wrist_pitch,tool_roll,gripper,arm_lm_state,gripper_current_a
//   clock+wheel OpenCR tx: wheel_l,wheel_r,clock
//                      rx: wheel_l,wheel_r,clock,wheel_state,clock_state

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
#include "craft_control/lm_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"

constexpr double kDt = 0.02;  // control period [s], TODO: check computing time and OpenCR serial latency

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
    // Load parameters: arm, gripper, lm guide
    const auto arm_min_angle = load_array_param<craft_control::kArmIkJoints>("arm_min_angle");
    const auto arm_max_angle = load_array_param<craft_control::kArmIkJoints>("arm_max_angle");
    const auto arm_ik_joint_names = declare_parameter<std::vector<std::string>>(
      "arm_ik_joint_names", std::vector<std::string>{
        "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "tool_roll_joint"});
    const auto arm_joint_directions = load_array_param<craft_control::kArmIkJoints>("arm_joint_directions");
    const auto arm_joint_offsets = load_array_param<craft_control::kArmIkJoints>("arm_joint_offsets");
    arm_ik_mapping_confirmed_ = declare_parameter("arm_ik_mapping_confirmed", false);
    lm_linear_joint_name_ = declare_parameter<std::string>("lm_linear_joint_name", "slide_joint");
    lm_mapping_confirmed_ = declare_parameter("lm_mapping_confirmed", false);
    const double lm_min_angle = declare_parameter<double>("lm_min_angle");
    const double lm_max_angle = declare_parameter<double>("lm_max_angle");
    const double lm_motor_direction = declare_parameter<double>("lm_motor_direction");
    const double lm_motor_offset = declare_parameter<double>("lm_motor_offset");
    const double lm_pulley_radius = declare_parameter("lm_pulley_radius", 0.0);
    const double lm_gear_ratio = declare_parameter("lm_gear_ratio", 0.0);
    const double gripper_min_angle = declare_parameter("gripper_min_angle", 0.0);
    const double gripper_max_angle = declare_parameter("gripper_max_angle", 0.0);
    const double gripper_open_angle = declare_parameter("gripper_open_angle", 0.0);
    const double gripper_close_angle = declare_parameter("gripper_close_angle", 0.0);
    const double grasp_current_threshold_a = declare_parameter("grasp_current_threshold_a", 0.0);
    const double grasp_current_debounce = declare_parameter("grasp_current_debounce", 0.0);
    gripper_mapping_confirmed_ = declare_parameter("gripper_mapping_confirmed", false);
    const double arm_angle_tolerance = load_positive_param("arm_angle_tolerance");
    const double lm_angle_tolerance = load_positive_param("lm_angle_tolerance");
    const double gripper_angle_tolerance = load_positive_param("gripper_angle_tolerance");

    // Validate parameters: arm, gripper, lm guide
    if (arm_ik_joint_names.size() != craft_control::kArmIkJoints)
    {
      throw std::runtime_error("arm_ik_joint_names needs 4 rotational joint entries");
    }
    craft_control::ArmIkJointNames ik_joint_name_array;
    std::copy(arm_ik_joint_names.begin(), arm_ik_joint_names.end(), ik_joint_name_array.begin());
    if (!arm_controller_.configure(
      arm_min_angle, arm_max_angle, ik_joint_name_array, arm_joint_directions, arm_joint_offsets,
      arm_angle_tolerance))
    {
      throw std::runtime_error("invalid arm limits or IK joint/motor mapping in config/control.yaml");
    }
    if (std::find(arm_ik_joint_names.begin(), arm_ik_joint_names.end(), lm_linear_joint_name_) !=
      arm_ik_joint_names.end() || !lm_controller_.configure(
        lm_linear_joint_name_, lm_min_angle, lm_max_angle, lm_motor_direction, lm_motor_offset,
        lm_pulley_radius, lm_gear_ratio, lm_angle_tolerance))
    {
      throw std::runtime_error("invalid LM limits, mapping or calibration in config/control.yaml");
    }
    if (!gripper_controller_.configure(
      gripper_min_angle, gripper_max_angle,
      gripper_open_angle, gripper_close_angle, grasp_current_threshold_a, grasp_current_debounce,
      gripper_angle_tolerance))
    {
      throw std::runtime_error("invalid gripper angle limits or open/close targets");
    }
    if (!arm_ik_mapping_confirmed_) {
      RCLCPP_WARN(get_logger(), "arm IK-to-motor mapping is unconfirmed; rotational arm targets will be ignored");
    }
    if (!lm_mapping_confirmed_) {
      RCLCPP_WARN(get_logger(), "LM mapping is unconfirmed; linear targets will be ignored");
    }
    if (!gripper_mapping_confirmed_) {
      RCLCPP_WARN(get_logger(), "gripper mapping is unconfirmed; gripper commands will be ignored");
    }
    if (!gripper_controller_.grasp_detection_configured()) {
      RCLCPP_WARN(get_logger(), "gripper current-based grasp detection is disabled until thresholds are calibrated");
    }

    // Load parameters: wheel, clock
    wheel_radius_ = load_positive_param("wheel_radius");
    const auto wheel_separation = load_positive_param("wheel_separation");
    const auto max_v = load_positive_param("max_v");
    max_wheel_ = load_positive_param("max_wheel");
    const auto lookahead_distance = load_positive_param("lookahead_distance");
    const auto wheel_goal_tolerance = load_positive_param("wheel_goal_tolerance");
    clock_slot_count_ = declare_parameter<int>("clock_slot_count");
    const auto clock_slot_angle = load_positive_param("clock_slot_angle");
    const auto clock_zero_angle = declare_parameter<double>("clock_zero_angle");
    const auto clock_direction = declare_parameter<double>("clock_direction");
    const auto clock_angle_tolerance = load_positive_param("clock_angle_tolerance");

    // Validate parameters: wheel, clock
    longitudinal_.configure(wheel_radius_, max_v);
    lateral_.configure(wheel_radius_, wheel_separation, lookahead_distance, wheel_goal_tolerance);
    if (!clock_controller_.configure(
      clock_slot_count_, clock_slot_angle, clock_zero_angle, clock_direction, clock_angle_tolerance))
    {
      throw std::runtime_error("invalid clock geometry or angle tolerance in config/control.yaml");
    }

    // OpenCR serial ports
    arm_lm_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("arm_lm_port"));
    clock_wheel_serial_ = std::make_unique<OpencrSerial>(declare_parameter<std::string>("clock_wheel_port"));

    // ROS2 publishers and subscribers
    arm_lm_state_pub_ = create_publisher<std_msgs::msg::Int32>("/arm_lm_state", 10);
    gripper_grasped_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_grasped", 10);
    wheel_state_pub_ = create_publisher<std_msgs::msg::Int32>("/wheel_state", 10);
    clock_state_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_state", 10);
    clock_slot_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_slot", 10);
    clock_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/clock_target_reached", 10);
    arm_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/arm_target_reached", 10);
    lm_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/lm_target_reached", 10);
    gripper_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_target_reached", 10);
    wheel_target_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/wheel_target_reached", 10);
    target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/target_path", 10,
      [this](const nav_msgs::msg::Path & msg) {pending_path_ = msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/planning/odom", 10,
      [this](const nav_msgs::msg::Odometry & msg) {pending_odom_ = msg;});
    wheel_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/planning/target_v", 10,
      [this](const std_msgs::msg::Float64 & msg) {pending_wheel_target_velocity_ = msg.data;});
    clock_target_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/planning/target_clock_slot", 10,
      [this](const std_msgs::msg::Int32 & msg) {pending_clock_slot_ = msg.data;});
    arm_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/planning/target_arm_joints", 10,
      [this](const sensor_msgs::msg::JointState & msg) {pending_manipulator_target_ = msg;});
    gripper_target_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/gripper_command", 10,
      [this](const std_msgs::msg::Bool & msg) {pending_gripper_target_ = msg.data;});

    // Control loop timer
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

  static void publish_bool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub, bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  double load_positive_param(const std::string & name)
  {
    const auto value = declare_parameter<double>(name, 0.0);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(name + " must be finite and > 0 (config/control.yaml)");
    }
    return value;
  }

  bool read_arm_lm_feedback()
  {
    const auto lines = arm_lm_serial_->read_lines();

    for (auto it = lines.rbegin(); it != lines.rend(); ++it) { // reverse order: newest first
      const auto & line = *it;
      std::vector<double> v;

      // Validate the line: must be CSV, correct number of values, all finite
      if (!parse_csv(line, v) || v.size() != 8) // LM + four arm motors + gripper + state + current
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad arm+LM OpenCR line: %s", line.c_str());
        continue;
      }
      if (!std::all_of(v.begin(), v.end(), [](double value) {
        return std::isfinite(value);
      }))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "non-finite arm+LM OpenCR feedback: %s", line.c_str());
        continue;
      }

      // Parse the line: slide, shoulder, elbow, wrist_pitch, tool_roll, gripper, arm_lm_state, gripper_current_a
      lm_angle_ = v[0];
      arm_joints_[0] = v[1];  // shoulder
      arm_joints_[1] = v[2];  // elbow
      arm_joints_[2] = v[3];  // wrist pitch
      arm_joints_[3] = v[4];  // tool roll
      gripper_angle_ = v[5];
      arm_lm_state_ = static_cast<int>(v[6]);
      gripper_current_a_ = v[7];

      have_arm_lm_feedback_ = true; // 정상 arm+LM 피드백을 한 번이라도 받은 적이 있는가
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

      // Validate the line: must be CSV, contain five values, and contain only finite numbers.
      if (!parse_csv(line, v) || v.size() != 5) { // 5: wheel_l, wheel_r, clock, wheel_state, clock_state
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

      // Parse the line: wheel_l, wheel_r, clock, wheel_state, clock_state.
      wheel_left_feedback_ = v[0];
      wheel_right_feedback_ = v[1];
      clock_angle_ = v[2];
      wheel_state_ = static_cast<int>(v[3]);
      clock_state_ = static_cast<int>(v[4]);

      have_clock_wheel_feedback_ = true; // 정상 clock & wheel 피드백을 한 번이라도 받은 적이 있는가
      return true;
    }
    return false;
  }

  bool validate_arm_target(const sensor_msgs::msg::JointState & msg)
  {
    // Validate the mapping and incoming joint target.
    if (!arm_ik_mapping_confirmed_) {
      return false;
    }
    if (!arm_controller_.is_valid_target(msg.name, msg.position)) {
      RCLCPP_WARN(get_logger(), "invalid rotational arm joint target: holding position");
      return false;
    }
    return true;
  }

  void set_arm_target(const sensor_msgs::msg::JointState & msg)
  {
    // Store the validated target; motor conversion and transmission happen in control_loop().
    craft_control::ArmIkJointValues target;
    std::copy(msg.position.begin(), msg.position.end(), target.begin());
    arm_controller_.set_target(target);
    publish_bool(arm_target_reached_pub_, false);
  }

  void clear_arm_target()
  {
    // Invalid target: stop at the current position until planning sends a new target.
    arm_controller_.clear_target();
    publish_bool(arm_target_reached_pub_, false);
  }

  bool validate_lm_target(const std::string & joint_name, double position)
  {
    // Validate only the LM target; its calibration does not gate the rotational arm.
    if (!lm_mapping_confirmed_) {
      return false;
    }
    if (!lm_controller_.is_valid_target(joint_name, position)) {
      RCLCPP_WARN(get_logger(), "invalid LM target or incomplete LM calibration: holding position");
      return false;
    }
    return true;
  }

  void set_lm_target(double position)
  {
    // Store the validated linear target; motor-angle conversion happens in control_loop().
    lm_controller_.set_target(position);
    publish_bool(lm_target_reached_pub_, false);
  }

  void clear_lm_target()
  {
    // Invalid target: stop at the current position until planning sends a new target.
    lm_controller_.clear_target();
    publish_bool(lm_target_reached_pub_, false);
  }

  bool validate_gripper_target()
  {
    // Open/close is always well-formed; configure() succeeded in the constructor.
    return gripper_mapping_confirmed_;
  }

  void set_gripper_target(bool close)
  {
    // Store the validated target; angle selection and clamping happen in control_loop().
    gripper_controller_.set_target(close);
    publish_bool(gripper_target_reached_pub_, false);
  }

  bool validate_wheel_target(double velocity)
  {
    if (!longitudinal_.is_valid_target(velocity)) {
      RCLCPP_WARN(get_logger(), "non-finite wheel target velocity: stopping");
      return false;
    }
    return true;
  }

  void set_wheel_target(double velocity)
  {
    // Store the validated target; wheel-speed conversion happens in control_loop().
    wheel_target_v_ = velocity;
  }

  void clear_wheel_target()
  {
    // Invalid target: stop until planning sends a new velocity.
    wheel_target_v_ = 0.0;
  }

  bool validate_path_target(const nav_msgs::msg::Path & path)
  {
    // An empty path is planning's "no route" result: stop without a warning.
    if (path.poses.empty()) {
      return false;
    }
    if (!lateral_.is_valid_target(path)) {
      RCLCPP_WARN(get_logger(), "invalid wheel path (coordinates or frame_id): stopping");
      return false;
    }
    return true;
  }

  void set_path_target(const nav_msgs::msg::Path & path)
  {
    // Store the validated path. Planning republishes the same path every tick; only a changed
    // path resets arrival.
    lateral_.set_target(path);
    if (path.poses != last_path_poses_) {
      last_path_poses_ = path.poses;
      publish_bool(wheel_target_reached_pub_, false);
    }
  }

  void clear_path_target()
  {
    // Invalid or empty path: stop until planning sends a new path.
    lateral_.clear_target();
    if (!last_path_poses_.empty()) {
      last_path_poses_.clear();
      publish_bool(wheel_target_reached_pub_, false);
    }
  }

  bool validate_odom(const nav_msgs::msg::Odometry & odom)
  {
    if (!lateral_.is_valid_odom(odom)) {
      RCLCPP_WARN(get_logger(), "invalid wheel odometry: stopping");
      return false;
    }
    return true;
  }

  bool validate_clock_target(int slot)
  {
    if (!clock_controller_.is_valid_target(slot)) {
      RCLCPP_WARN(get_logger(), "invalid clock target: slot must be 1..%d; holding position", clock_slot_count_);
      return false;
    }
    return true;
  }

  void set_clock_target(int slot)
  {
    // Store the validated slot; the shortest turn is chosen from fresh feedback.
    clock_controller_.set_target(slot, clock_angle_);
    clock_target_sent_ = false;
    publish_bool(clock_target_reached_pub_, false);
  }

  void clear_clock_target()
  {
    // Invalid target: stop at the current position until planning sends a new slot.
    clock_controller_.clear_target();
    clock_target_sent_ = false;
    publish_bool(clock_target_reached_pub_, false);
  }

  void control_loop()
  {
    const auto t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : kDt;  // real dt
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    // Planning callbacks keep only the newest message; validate and store targets here.
    if (pending_manipulator_target_) {
      const auto & msg = *pending_manipulator_target_;
      // Existing planning JointState: LM [m], then four rotational joints [rad].
      if (msg.name.size() == 1 + craft_control::kArmIkJoints &&
        msg.name.size() == msg.position.size() && msg.name[0] == lm_linear_joint_name_)
      {
        sensor_msgs::msg::JointState arm_target;
        arm_target.name.assign(msg.name.begin() + 1, msg.name.end());
        arm_target.position.assign(msg.position.begin() + 1, msg.position.end());

        // Validate arm target pose, then store it.
        if (validate_arm_target(arm_target)) {
          set_arm_target(arm_target);
        } else if (arm_ik_mapping_confirmed_) {
          clear_arm_target();
        }

        // Validate LM target pose, then store it independently.
        if (validate_lm_target(msg.name[0], msg.position[0])) {
          set_lm_target(msg.position[0]);
        } else if (lm_mapping_confirmed_) {
          clear_lm_target();
        }
      } else {
        RCLCPP_WARN(get_logger(), "invalid manipulator JointState (expected LM then four arm joints): holding position");
        if (arm_ik_mapping_confirmed_) {
          clear_arm_target();
        }
        if (lm_mapping_confirmed_) {
          clear_lm_target();
        }
      }
      pending_manipulator_target_.reset();
    }

    // Validate gripper target, then store it. An unconfirmed mapping ignores gripper commands.
    if (pending_gripper_target_) {
      if (validate_gripper_target()) {
        set_gripper_target(*pending_gripper_target_);
      }
      pending_gripper_target_.reset();
    }

    // Validate wheel target velocity, then store it. Invalid input stops the wheels.
    if (pending_wheel_target_velocity_) {
      if (validate_wheel_target(*pending_wheel_target_velocity_)) {
        set_wheel_target(*pending_wheel_target_velocity_);
      } else {
        clear_wheel_target();
      }
      pending_wheel_target_velocity_.reset();
    }

    // Validate path and odometry, then store them. Invalid input stops path tracking.
    if (pending_path_) {
      if (validate_path_target(*pending_path_)) {
        set_path_target(*pending_path_);
      } else {
        clear_path_target();
      }
      pending_path_.reset();
    }
    if (pending_odom_) {
      if (validate_odom(*pending_odom_)) {
        lateral_.set_odom(*pending_odom_);
      } else {
        lateral_.clear_odom();
      }
      pending_odom_.reset();
    }

    const bool arm_lm_feedback_updated = read_arm_lm_feedback();
    const bool clock_wheel_feedback_updated = read_clock_wheel_feedback();
    // Validate clock slot, then store it once fresh feedback fixes the shortest turn.
    if (clock_wheel_feedback_updated && pending_clock_slot_) {
      if (validate_clock_target(*pending_clock_slot_)) {
        set_clock_target(*pending_clock_slot_);
      } else {
        clear_clock_target();
      }
      pending_clock_slot_.reset();
    }
    if (have_arm_lm_feedback_) {
      if (arm_lm_feedback_updated) {
        gripper_controller_.update_current_feedback(gripper_current_a_, dt);
        std_msgs::msg::Bool grasped_msg;
        grasped_msg.data = gripper_controller_.is_grasped();
        gripper_grasped_pub_->publish(grasped_msg);
      }

      const auto arm_motor_commands = arm_controller_.compute_command(arm_joints_);
      const double lm_motor_command = lm_controller_.compute_command(lm_angle_);
      const double gripper_motor_command = gripper_controller_.compute_command(gripper_angle_);
      // Shared OpenCR transport only: independent controllers, fixed hardware channel order.
      if (!arm_lm_serial_->write_values({
          lm_motor_command, arm_motor_commands[0], arm_motor_commands[1],
          arm_motor_commands[2], arm_motor_commands[3], gripper_motor_command}))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "arm+LM OpenCR write failed");
      } else if (arm_lm_feedback_updated) {
        // Fresh feedback against the targets just sent; planning sequences the axes.
        publish_bool(arm_target_reached_pub_, arm_controller_.is_target_reached(arm_joints_));
        publish_bool(lm_target_reached_pub_, lm_controller_.is_target_reached(lm_angle_));
        publish_bool(gripper_target_reached_pub_, gripper_controller_.is_target_reached(gripper_angle_));
      }
      std_msgs::msg::Int32 msg;
      msg.data = arm_lm_state_;
      arm_lm_state_pub_->publish(msg);
    }

    if (have_clock_wheel_feedback_) {
      if (lateral_.can_track(wheel_target_v_)) {
        longitudinal_.set_target(wheel_target_v_);
      } else {
        longitudinal_.clear_target();  // no valid path/odom, or already at the path goal
      }
      // Host converts planning targets and applies safety limits only. OpenCR firmware configures
      // DYNAMIXEL Profile Acceleration and owns velocity PID/feedforward and motor-level tracking.
      const auto v = longitudinal_.compute_command();
      const double linear_speed = wheel_radius_ * (v[0] + v[1]) / 2.0;
      const auto w = lateral_.compute_command(linear_speed);
      const double wl = v[0] + w[0];  // [rad/s]
      const double wr = v[1] + w[1];
      // over wheel limit: scale both by the same ratio to keep curvature
      const double k = std::max({1.0, std::abs(wl) / max_wheel_, std::abs(wr) / max_wheel_});
      const double clock_motor_command = clock_controller_.compute_command(clock_angle_);
      const bool command_sent = clock_wheel_serial_->write_values({wl / k, wr / k, clock_motor_command});
      if (!command_sent) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "clock+wheel OpenCR write failed");
      }
      if (command_sent && !pending_clock_slot_) {
        clock_target_sent_ = true;
      }
      publish_bool(wheel_target_reached_pub_, lateral_.is_target_reached());
      std_msgs::msg::Int32 wheel_state_msg;
      wheel_state_msg.data = wheel_state_;
      wheel_state_pub_->publish(wheel_state_msg);
      std_msgs::msg::Int32 clock_state_msg;
      clock_state_msg.data = clock_state_;
      clock_state_pub_->publish(clock_state_msg);
      if (clock_wheel_feedback_updated) {
        std_msgs::msg::Int32 slot_msg;
        slot_msg.data = clock_state_ == 0 ? clock_controller_.get_current_slot(clock_angle_) : 0;
        clock_slot_pub_->publish(slot_msg);
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = clock_target_sent_ && !pending_clock_slot_ && command_sent &&
          clock_state_ == 0 && clock_controller_.is_target_reached(clock_angle_);
        clock_target_reached_pub_->publish(reached_msg);
      }
    }
  }

  craft_control::LongitudinalController longitudinal_;
  craft_control::LateralController lateral_;
  double max_wheel_ = 0.0;
  double wheel_radius_ = 0.0;
  double wheel_target_v_ = 0.0;
  std::vector<geometry_msgs::msg::PoseStamped> last_path_poses_;
  std::optional<sensor_msgs::msg::JointState> pending_manipulator_target_;
  std::optional<double> pending_wheel_target_velocity_;
  std::optional<bool> pending_gripper_target_;
  std::optional<nav_msgs::msg::Path> pending_path_;
  std::optional<nav_msgs::msg::Odometry> pending_odom_;
  std::optional<rclcpp::Time> last_t_;
  craft_control::ArmController arm_controller_;
  craft_control::LmController lm_controller_;
  craft_control::GripperController gripper_controller_;
  craft_control::ClockController clock_controller_;

  std::unique_ptr<OpencrSerial> arm_lm_serial_;
  std::unique_ptr<OpencrSerial> clock_wheel_serial_;
  craft_control::ArmJoints arm_joints_{};
  bool arm_ik_mapping_confirmed_ = false;
  std::string lm_linear_joint_name_;
  bool lm_mapping_confirmed_ = false;
  double lm_angle_ = 0.0;
  double gripper_angle_ = 0.0;
  bool gripper_mapping_confirmed_ = false;
  double gripper_current_a_ = 0.0;
  int arm_lm_state_ = 0;
  bool have_arm_lm_feedback_ = false;
  double wheel_left_feedback_ = 0.0;
  double wheel_right_feedback_ = 0.0;
  double clock_angle_ = 0.0;
  int wheel_state_ = 0;
  int clock_state_ = 0;
  bool have_clock_wheel_feedback_ = false;
  std::optional<int> pending_clock_slot_;
  int clock_slot_count_ = 0;
  bool clock_target_sent_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_lm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_grasped_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wheel_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_slot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clock_target_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_target_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lm_target_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_target_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheel_target_reached_pub_;
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
