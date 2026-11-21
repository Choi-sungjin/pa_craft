// Mission adapter for the calibrated executor. No direct control/hold publisher.
// Request: '<token> <box> <slot> <collect|find|lie2|lie5> [remaining]'; result: '<token> <OK|FAILED> <reason>'.
// Only one child runs; STOP cancels it and stops planning. Output is retained per request.
// executor_path defaults to the installed puck_execute binary.
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;
using std_msgs::msg::String;
using Clock = std::chrono::steady_clock;

namespace {

fs::path home() {
  const char * h = std::getenv("HOME");
  return h ? fs::path(h) : fs::path("/tmp");
}

bool all_of(const std::string & s, int (*pred)(int)) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [pred](unsigned char c) { return pred(c) != 0; });
}

String text(const std::string & s) {
  String m;
  m.data = s;
  return m;
}

}  // namespace

class Picker : public rclcpp::Node {
public:
  Picker() : Node("puck_picker") {
    fs::path prefix;
    ament_index_cpp::get_package_prefix("craft_planning", prefix);
    const auto default_exe = prefix / "lib" / "craft_planning" / "puck_execute";
    executor_path_ = declare_parameter("executor_path", default_exe.string());
    log_dir_ = declare_parameter("log_dir", (home() / "craft_mission_logs" / "picks").string());
    timeout_ = declare_parameter("timeout_s", 600.0);
    results_ = create_publisher<String>("/mission/pick_result", 10);
    command_ = create_publisher<String>("/planning/command", 10);
    request_sub_ = create_subscription<String>("/mission/pick_request", 10,
                                               [this](String::ConstSharedPtr m) { request(m->data); });
    cancel_sub_ = create_subscription<String>("/mission/pick_cancel", 10, [this](String::ConstSharedPtr) { cancel(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { poll(); });
  }

  ~Picker() override {
    cancel();
    if (child_ <= 0) return;
    const auto until = Clock::now() + std::chrono::seconds(3);
    while (Clock::now() < until && !reap(WNOHANG)) usleep(20000);
    if (child_ > 0) {
      ::kill(-child_, SIGKILL);
      reap(0);
    }
  }

private:
  void result(const std::string & token, bool ok, const std::string & reason) {
    results_->publish(text(token + (ok ? " OK " : " FAILED ") + reason));
  }

  void request(const std::string & data) {
    std::istringstream ss(data);
    std::vector<std::string> f;
    for (std::string w; ss >> w;) f.push_back(w);
    std::string token = f.empty() ? "invalid" : f[0];
    try {
      if (f.size() != 4 && f.size() != 5) throw std::invalid_argument("expected token box slot mode [remaining]");
      token = f[0];
      const std::string & box = f[1], & slot = f[2], & mode = f[3];
      std::optional<int> remaining;
      if (f.size() == 5) {
        if (!all_of(f[4], ::isdigit)) throw std::invalid_argument("invalid literal for int(): '" + f[4] + "'");
        remaining = std::stoi(f[4]);
        if (*remaining != 1 && *remaining != 2) throw std::invalid_argument("remaining puck count must be 1 or 2");
      }
      const bool slot_ok = all_of(slot, ::isdigit) && slot.size() < 4 && std::stoi(slot) >= 1 && std::stoi(slot) <= 12;
      if (!all_of(token, ::isdigit) || !all_of(box, ::isalnum) || !slot_ok ||
          (mode != "collect" && mode != "find" && mode != "lie2" && mode != "lie5"))
        throw std::invalid_argument("invalid pick request");
      if (child_ > 0) throw std::invalid_argument("picker is busy");
      if (!fs::is_regular_file(executor_path_)) throw std::invalid_argument("executor missing: " + executor_path_);

      std::vector<std::string> args;
      args.insert(args.end(), {executor_path_, "--planning", "--box", box, "--slot", slot, "--auto"});
      if (mode == "collect") args.push_back("--collect");
      else if (mode == "find") args.push_back("--find");
      else args.insert(args.end(), {"--lie", "--cell", mode.substr(mode.size() - 1), "--pick"});
      if (mode == "collect" && remaining) args.insert(args.end(), {"--expected-pucks", std::to_string(*remaining)});

      fs::create_directories(log_dir_);
      log_path_ = (fs::path(log_dir_) / ("pick_" + token + ".log")).string();
      const int log_fd = ::open(log_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
      if (log_fd < 0) throw std::runtime_error("cannot open " + log_path_);
      std::vector<char *> argv;
      for (auto & a : args) argv.push_back(a.data());
      argv.push_back(nullptr);
      const pid_t pid = ::fork();
      if (pid == 0) {  // child: own session (so SIGINT/SIGKILL reach its whole group), output to the log
        ::setsid();
        const int null_fd = ::open("/dev/null", O_RDONLY);
        ::dup2(null_fd, 0);
        ::dup2(log_fd, 1);
        ::dup2(log_fd, 2);
        ::execv(argv[0], argv.data());
        _exit(127);
      }
      ::close(log_fd);
      if (pid < 0) throw std::runtime_error("fork failed");
      child_ = pid;
      token_ = token;
      started_ = Clock::now();
      cancelled_ = false;
      stop_at_.reset();
      RCLCPP_INFO(get_logger(), "%s: %s", data.c_str(), log_path_.c_str());
    } catch (const std::exception & e) {
      result(token, false, e.what());
    }
  }

  void cancel() {
    if (child_ <= 0 || cancelled_) return;
    cancelled_ = true;
    stop_at_ = Clock::now();
    ::kill(-child_, SIGINT);
    if (rclcpp::ok()) command_->publish(text("STOP"));
  }

  // true once the child has exited; sets code_ like Popen.returncode (negative = killed by that signal).
  bool reap(int flags) {
    int status = 0;
    if (::waitpid(child_, &status, flags) != child_) return false;
    code_ = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
    child_ = -1;
    return true;
  }

  void poll() {
    if (child_ <= 0) return;
    const auto now = Clock::now();
    if (std::chrono::duration<double>(now - started_).count() > timeout_) cancel();
    if (stop_at_ && std::chrono::duration<double>(now - *stop_at_).count() > 3) ::kill(-child_, SIGKILL);
    if (!reap(WNOHANG)) return;
    result(token_, code_ == 0 && !cancelled_,
           "exit=" + std::to_string(code_) + ", cancelled=" + (cancelled_ ? "True" : "False") + ", log=" + log_path_);
  }

  std::string executor_path_, log_dir_, token_, log_path_;
  double timeout_;
  pid_t child_ = -1;
  int code_ = 0;
  bool cancelled_ = false;
  Clock::time_point started_;
  std::optional<Clock::time_point> stop_at_;
  rclcpp::Publisher<String>::SharedPtr results_, command_;
  rclcpp::Subscription<String>::SharedPtr request_sub_, cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<Picker>();
    rclcpp::spin(node);
  }  // ~Picker: cancel the child, wait 3 s, then kill
  rclcpp::shutdown();
}
