// control_master: planning targets -> 2x OpenCR (serial)
// sub: /planning/target_path, /planning/odom
// pub: /arm_state, /clock_state (Int32: 0 idle, 1 using, 2 fault, every tick)
// param: config/control.yaml (OpenCR ports, arm motor limits)
// serial (ASCII CSV, '\n' terminated, OpenCR must stream rx on its own):
//   arm OpenCR   tx: arm_j0,...,arm_j4,lm
//                rx: arm_state,arm_j0,...,arm_j4,lm
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
#include "std_msgs/msg/int32.hpp"

#include "craft_control/arm_planner.hpp"
#include "craft_control/clock_planner.hpp"
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
    const auto max_vel = load_arm_param("arm_max_vel");
    for (std::size_t i = 0; i < craft_control::kArmMotors; ++i) {
      if (min_angle[i] >= max_angle[i] || max_vel[i] <= 0.0) {
        throw std::runtime_error("bad arm limit at motor " + std::to_string(i) + " (config/control.yaml)");
      }
    }
    arm_planner_.set_limits(min_angle, max_angle, max_vel);

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
    clock_state_pub_ = create_publisher<std_msgs::msg::Int32>("/clock_state", 10);
    target_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/target_path", 10, [this](const nav_msgs::msg::Path & msg) {lateral_.set_path(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/planning/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {lateral_.set_odom(msg);});
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

  void read_arm()
  {
    for (const auto & line : arm_serial_->read_lines()) {
      std::vector<double> v;
      if (!parse_csv(line, v) || v.size() != 1 + craft_control::kArmMotors) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "bad arm OpenCR line: %s", line.c_str());
        continue;
      }
      arm_state_ = static_cast<int>(v[0]);
      std::copy(v.begin() + 1, v.end(), arm_joints_.begin());
      have_arm_fb_ = true;
    }
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

  void on_timer()
  {
    const auto t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : kDt;  // real dt
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    read_arm();
    read_clock();

    // no command before first feedback of each OpenCR, so motors never jump from 0
    if (have_arm_fb_) {
      const auto arm_pose = arm_planner_.get_arm_pose(arm_joints_, kDt);
      if (!arm_serial_->write_values({arm_pose.begin(), arm_pose.end()})) {
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
  craft_control::ClockPlanner clock_planner_;

  std::unique_ptr<OpencrSerial> arm_serial_;
  std::unique_ptr<OpencrSerial> clock_serial_;
  craft_control::ArmJoints arm_joints_{};
  int arm_state_ = 0;
  bool have_arm_fb_ = false;
  double clock_angle_ = 0.0;
  int clock_state_ = 0;
  bool have_clock_fb_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlMaster>());
  rclcpp::shutdown();
  return 0;
}
