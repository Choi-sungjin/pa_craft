// mission_master: runs the dice memory mission (mission_fsm.hpp) on top of planning_master.
//   in : /mission/command (START, STOP, RESET), the barcode, the blink count and the puck survey of perception,
//        /planning/state
//   out: /mission/state, /planning/command, /planning/approach_box, /planning/leave_box, /planning/target_base_pose, the perception resets,
//        <log_dir>/mission_<time>/{signal.json, allocation.json, mission.log}
// Driving (approach_box, go_home) goes through planning_master. Looking at the pucks, picking, placing and verifying
// are SKELETONS here (run_skeleton): they log what they would do and succeed after skeleton_action_s.
#include "craft_planning/mission_fsm.hpp"

#include <craft_perception/msg/puck_survey.hpp>
#include <craft_perception/msg/traffic_light.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

using namespace craft_planning::mission;
using Kind = MissionAction::Kind;
using std_msgs::msg::String;

namespace {

double seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <class T, class F> std::string json_list(const T &values, F item) {
  std::string out = "[";
  for (auto it = values.begin(); it != values.end(); ++it) out += (it == values.begin() ? "" : ", ") + item(*it);
  return out + "]";
}
std::string quoted(const std::string &text) {
  std::string out = "\"";
  for (char c : text) {if (c == '"' || c == '\\') out += '\\'; out += c;}
  return out + "\"";
}

class MissionMaster : public rclcpp::Node {
public:
  MissionMaster() : Node("mission_master") {
    MissionConfig c;
    c.start_boxes = declare_parameter<std::vector<std::string>>("start_boxes");
    const auto locations = declare_parameter<std::vector<std::string>>("locations");
    if (locations.size() != 3) throw std::invalid_argument("locations must name three boxes (L1, L2, L3 order)");
    std::copy(locations.begin(), locations.end(), c.locations.begin());
    c.pucks_per_box = declare_parameter<int>("pucks_per_box");
    c.barcode_timeout_s = declare_parameter<double>("barcode_timeout_s");
    c.signal_timeout_s = declare_parameter<double>("signal_timeout_s");
    c.home_still_s = declare_parameter<double>("home_still_s");
    c.verify_after_all = declare_parameter<bool>("verify_after_all");
    c.assume_unknown_picks = declare_parameter<bool>("assume_unknown_picks");
    c.drive_only = declare_parameter<bool>("drive_only");
    c.action_timeout_s[name(Kind::ApproachBox)] = declare_parameter<double>("approach_timeout_s");
    c.action_timeout_s[name(Kind::GoHome)] = declare_parameter<double>("go_home_timeout_s");
    c.action_timeout_s[name(Kind::LeaveBox)] = declare_parameter<double>("leave_timeout_s");
    // Before the FSM is built: it copies the config. [x, y, yaw_deg] where the upper cam reads the barcode; [] = read it where the robot stands.
    // Default []: a ROS params file cannot give an empty list a type ("[]" -> "Contains no value", node dies);
    // leave barcode_view out of mission.yaml to read the barcode where the robot stands (2026-11-14).
    barcode_view_ = declare_parameter<std::vector<double>>("barcode_view", std::vector<double>{});
    if (!barcode_view_.empty() && (barcode_view_.size() != 3 || !std::isfinite(barcode_view_[0]) ||
        !std::isfinite(barcode_view_[1]) || !std::isfinite(barcode_view_[2]))) {
      throw std::invalid_argument("barcode_view must be [] or [x, y, yaw_deg] in the map frame");
    }
    c.view_barcode = !barcode_view_.empty();
    c.action_timeout_s[name(Kind::ViewBarcode)] = c.action_timeout_s[name(Kind::GoHome)];  // go_home_timeout_s
    config_ = c;
    fsm_ = MissionFsm(config_);  // validates
    home_ = declare_parameter<std::vector<double>>("home");
    if (home_.size() != 3 || !std::isfinite(home_[0]) || !std::isfinite(home_[1]) || !std::isfinite(home_[2])) {
      throw std::invalid_argument("home must be [x, y, yaw_deg] in the map frame");
    }
    map_frame_ = declare_parameter<std::string>("map_frame");
    reset_perception_ = declare_parameter<bool>("reset_perception_on_start");
    // craft.launch.py sets it when global_localize runs (slam_map:=): no drive before it reports "ok".
    wait_localization_ = declare_parameter<bool>("wait_global_localization", false);
    skeleton_action_s_ = declare_parameter<double>("skeleton_action_s");
    log_dir_ = declare_parameter<std::string>("log_dir");
    if (!log_dir_.empty() && log_dir_[0] == '~') log_dir_ = std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + log_dir_.substr(1);
    const double hz = declare_parameter<double>("loop_hz");
    if (!(hz > 0) || skeleton_action_s_ < 0) throw std::invalid_argument("loop_hz must be > 0, skeleton_action_s >= 0");

    const auto latched = rclcpp::QoS(1).transient_local();
    state_pub_ = create_publisher<String>("/mission/state", latched);
    planning_command_pub_ = create_publisher<String>("/planning/command", 10);
    approach_pub_ = create_publisher<String>("/planning/approach_box", 10);
    leave_pub_ = create_publisher<String>("/planning/leave_box", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/planning/target_base_pose", 10);
    signal_reset_pub_ = create_publisher<std_msgs::msg::Empty>("/perception/traffic_light/reset", 10);
    survey_reset_pub_ = create_publisher<std_msgs::msg::Empty>("/perception/puck_survey/reset", 10);

    subs_.push_back(create_subscription<String>("/mission/command", 10,
      [this](const String &m) {command(m.data);}));
    subs_.push_back(create_subscription<String>("/perception/vision/uppercam/barcode", latched,
      [this](const String &m) {inputs_.barcode = m.data;}));
    subs_.push_back(create_subscription<craft_perception::msg::TrafficLight>(
      "/perception/vision/uppercam/traffic_light", latched, [this](const craft_perception::msg::TrafficLight &m) {
        inputs_.signal_complete = m.complete; inputs_.signal_valid = m.valid;
        inputs_.counts = {m.red, m.yellow, m.green}; inputs_.sequence = m.sequence;
      }));
    subs_.push_back(create_subscription<craft_perception::msg::PuckSurvey>(
      "/perception/vision/d435/puck_survey", latched, [this](const craft_perception::msg::PuckSurvey &m) {
        inputs_.survey.clear();
        for (std::size_t i = 0; i < m.ids.size() && i < m.colors.size() && i < m.shapes.size(); ++i) {
          inputs_.survey.push_back({m.ids[i], m.colors[i] == m.RED ? PuckColor::Red : PuckColor::Green,
                                    m.shapes[i] == m.CUBOID ? PuckShape::Cuboid : PuckShape::Cylinder});
        }
      }));
    subs_.push_back(create_subscription<String>("/planning/state", 10, [this](const String &m) {planning(m.data);}));
    subs_.push_back(create_subscription<String>("/perception/localization/global", latched,
      [this](const String &m) {localized_ = m.data.rfind("ok", 0) == 0; localization_ = m.data;}));
    // The gripper latches a grasp when its current stays >= grasp_current_threshold_a (control.yaml: 0.1 A for
    // 0.1 s). A close that ends without it closed on nothing: the pick skeleton must check this.
    subs_.push_back(create_subscription<std_msgs::msg::Bool>("/control/gripper_grasped", 10,
      [this](const std_msgs::msg::Bool &m) {gripper_grasped_ = m.data;}));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this] {loop();});
    publish_state();
  }

private:
  void command(const std::string &text) {
    if (text == "START") {
      if (!fsm_.start(seconds())) {RCLCPP_WARN(get_logger(), "START ignored: the mission is running"); return;}
      open_log();
      note("mission started");
    } else if (text == "STOP") {
      if (fsm_.running()) fsm_.fail("stopped by the operator");
    } else if (text == "RESET") {
      if (fsm_.running()) {RCLCPP_WARN(get_logger(), "RESET ignored: STOP the mission first"); return;}
      fsm_ = MissionFsm(config_); fault_handled_ = false;
    } else {
      RCLCPP_WARN(get_logger(), "unknown /mission/command '%s' (START, STOP, RESET)", text.c_str());
    }
  }

  // /planning/state: Run.Complete lasts one planning loop, so it is latched here, not polled.
  void planning(const std::string &state) {
    planning_state_ = state;
    if (!request_sent_) return;
    if (state == "Run.Complete") request_complete_ = true;
    else if (state.rfind("Run.", 0) == 0 && state != "Run.AwaitRequest") request_busy_ = true;
    else if (state == "Run.AwaitRequest" && request_busy_) request_complete_ = true;
  }
  void send_planning(const std::string &text) {String m; m.data = text; planning_command_pub_->publish(m);}

  void begin(const MissionAction &a, double now) {
    action_at_ = now; request_sent_ = request_busy_ = request_complete_ = false; start_sent_at_ = -1e9;
    note("action: " + a.describe());
    switch (a.kind) {
      case Kind::ObservePucks: case Kind::PickPuck: case Kind::PlacePuck: case Kind::VerifyLocation:
        RCLCPP_WARN(get_logger(), "[SKELETON] %s: not implemented, reported as done after %.1f s",
                    a.describe().c_str(), skeleton_action_s_);
        break;
      case Kind::ResetPerception:
        if (reset_perception_) {
          signal_reset_pub_->publish(std_msgs::msg::Empty()); survey_reset_pub_->publish(std_msgs::msg::Empty());
          inputs_ = MissionInputs{};  // what was latched before the reset is not this run's
        }
        break;
      default: break;
    }
  }

  // Runs the action in flight; true once it is over (result in ok / reason).
  bool run(const MissionAction &a, double now, bool &ok, std::string &reason) {
    ok = true;
    switch (a.kind) {
      case Kind::ResetPerception: return true;
      case Kind::HoldStill: return now - action_at_ >= a.seconds;
      case Kind::ApproachBox: case Kind::LeaveBox: case Kind::GoHome: case Kind::ViewBarcode:
        return run_drive(a, now, ok, reason);
      case Kind::ObservePucks: case Kind::PickPuck: case Kind::PlacePuck: case Kind::VerifyLocation:
        return run_skeleton(a, now, ok, reason);
      case Kind::None: return true;
    }
    return true;
  }

  // A drive through planning_master: START it if it is only Ready, send the request once it awaits one, done on
  // Run.Complete, failed on Fault.
  bool run_drive(const MissionAction &a, double now, bool &ok, std::string &reason) {
    if (planning_state_ == "Fault") {ok = false; reason = "planning is in Fault"; return true;}
    if (!request_sent_) {
      if (wait_localization_ && !localized_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s: waiting for global localization (%s)",
                             a.describe().c_str(), localization_.empty() ? "no status yet" : localization_.c_str());
        return false;
      }
      if (planning_state_ == "Ready" && now - start_sent_at_ >= 1.0) {send_planning("START"); start_sent_at_ = now;}
      if (planning_state_ != "Run.AwaitRequest") return false;
      if (a.kind == Kind::ApproachBox) {
        String m; m.data = a.box; approach_pub_->publish(m);
      } else if (a.kind == Kind::LeaveBox) {
        String m; m.data = a.box; leave_pub_->publish(m);
      } else {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = map_frame_; goal.header.stamp = this->now();
        const auto &pose = a.kind == Kind::ViewBarcode ? barcode_view_ : home_;
        goal.pose.position.x = pose[0]; goal.pose.position.y = pose[1];
        const double yaw = pose[2] * M_PI / 180.0;
        goal.pose.orientation.z = std::sin(yaw / 2); goal.pose.orientation.w = std::cos(yaw / 2);
        goal_pub_->publish(goal);
      }
      request_sent_ = true;
      return false;
    }
    return request_complete_;
  }

  // SKELETON (2026-11-12): the camera -> pick link and the arm sequences are not connected yet.
  //   ObservePucks:   gripper D435 survey at the box -> which puck ids stand on it and where (needs the camera TF).
  //   PickPuck:       graspable check (grasp_region), arm pick, close, gripper_grasped_ must be true, load into the
  //                   clock slot; report the puck id taken (action_done puck_id; -1 here = not known).
  //   PlacePuck:      clock to the slot, take the puck out, put it down on the grid spot (demo/puck_pick sis3).
  //   VerifyLocation: observe the location again, record ids and positions from that observation.
  bool run_skeleton(const MissionAction &, double now, bool &ok, std::string &) {
    ok = true;
    return now - action_at_ >= skeleton_action_s_;
  }

  void loop() {
    const double now = seconds();
    const auto before = fsm_.state();
    if (fsm_.busy()) {
      bool ok = true; std::string reason;
      if (run(fsm_.action(), now, ok, reason)) fsm_.action_done(ok, reason);
    }
    const auto next = fsm_.tick(inputs_, now);
    if (next.kind != Kind::None) begin(next, now);
    if (fsm_.state() != before) transition(before);
    write_records();
    if (fsm_.state() == MissionState::Fault && !fault_handled_) {
      fault_handled_ = true;
      send_planning("STOP");  // safe stop: planning holds every axis
      RCLCPP_ERROR(get_logger(), "mission fault: %s", fsm_.reason().c_str());
      note("FAULT " + fsm_.reason());
    }
    publish_state();
  }

  void transition(MissionState before) {
    note(std::string(name(before)) + " -> " + name(fsm_.state()));
    if (fsm_.state() == MissionState::Done) note("mission done");
  }
  // signal.json once the signal is accepted, allocation.json once the allocation is made (several states can pass
  // in one tick, so by what the mission holds, not by the transition).
  void write_records() {
    const auto state = fsm_.state();
    const bool decided = state != MissionState::Idle && state != MissionState::ReadBarcode &&
      state != MissionState::WatchSignal && state != MissionState::Fault;
    if (decided && !signal_written_) {signal_written_ = true; write_signal();}
    if (!fsm_.placements().empty() && !allocation_written_) {allocation_written_ = true; write_allocation();}
  }

  void publish_state() {
    String m; m.data = fsm_.label();
    if (fsm_.state() == MissionState::Fault) m.data += ": " + fsm_.reason();
    if (m.data != last_state_) {last_state_ = m.data; state_pub_->publish(m); RCLCPP_INFO(get_logger(), "%s", m.data.c_str());}
  }

  void open_log() {
    run_dir_.clear(); fault_handled_ = false; signal_written_ = allocation_written_ = false;
    if (log_dir_.empty()) return;
    char stamp[32]; const std::time_t t = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    std::error_code error;
    const auto dir = std::filesystem::path(log_dir_) / (std::string("mission_") + stamp);
    std::filesystem::create_directories(dir, error);
    if (error) {RCLCPP_ERROR(get_logger(), "no mission log: %s: %s", dir.c_str(), error.message().c_str()); return;}
    run_dir_ = dir.string();
  }
  void note(const std::string &text) {
    if (run_dir_.empty()) return;
    char stamp[32]; const std::time_t t = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%H:%M:%S", std::localtime(&t));
    std::ofstream(run_dir_ + "/mission.log", std::ios::app) << stamp << " " << text << "\n";
  }
  void write_signal() {
    if (run_dir_.empty()) return;
    static const char *colors[] = {"R", "Y", "G"};
    const auto &c = fsm_.counts();
    std::ofstream(run_dir_ + "/signal.json")
      << "{\n  \"barcode\": " << quoted(fsm_.barcode()) << ",\n  \"order\": " << quoted(fsm_.order() ? name(*fsm_.order()) : "none")
      << ",\n  \"sequence\": " << json_list(fsm_.sequence(), [](uint8_t v) {return quoted(colors[v]);})
      << ",\n  \"counts\": {\"R\": " << c.red << ", \"Y\": " << c.yellow << ", \"G\": " << c.green << "}"
      << ",\n  \"locations\": {" << (fsm_.order() ? location_json() : std::string()) << "}\n}\n";  // drive_only: no order

  }
  std::string location_json() const {
    std::string out;
    for (int i = 0; i < 3; ++i) {
      out += std::string(i ? ", " : "") + quoted(config_.locations[i]) + ": {\"group\": " +
        quoted(name(fsm_.groups()[i])) + ", \"count\": " + std::to_string(fsm_.counts().of(fsm_.groups()[i])) + "}";
    }
    return out;
  }
  void write_allocation() {
    if (run_dir_.empty()) return;
    std::ostringstream pucks;
    for (std::size_t i = 0; i < inputs_.survey.size(); ++i) {
      const auto &p = inputs_.survey[i];
      pucks << (i ? ",\n    " : "    ") << "{\"id\": " << p.id << ", \"color\": " << quoted(name(p.color))
            << ", \"shape\": " << quoted(name(p.shape)) << "}";
    }
    std::ostringstream placed;
    for (std::size_t i = 0; i < fsm_.placements().size(); ++i) {
      const auto &p = fsm_.placements()[i];
      placed << (i ? ",\n    " : "    ") << "{\"location\": " << quoted(p.location) << ", \"group\": "
             << quoted(name(p.group)) << ", \"puck\": " << p.puck_id << ", \"slot\": " << p.slot
             << ", \"spot\": [" << p.spot[0] << ", " << p.spot[1] << "]}";
    }
    std::ofstream(run_dir_ + "/allocation.json")
      << "{\n  \"locations\": {" << location_json() << "},\n  \"yellow_shape\": "
      << quoted(name(fsm_.allocation().yellow_shape)) << ",\n  \"pucks\": [\n" << pucks.str()
      << "\n  ],\n  \"placements\": [\n" << placed.str() << "\n  ],\n  \"assume_unknown_picks\": "
      << (config_.assume_unknown_picks ? "true" : "false") << "\n}\n";
  }

  MissionConfig config_;
  MissionFsm fsm_;
  MissionInputs inputs_;
  std::vector<double> home_, barcode_view_;
  std::string map_frame_, log_dir_, run_dir_, planning_state_, last_state_;
  bool reset_perception_ = true, fault_handled_ = false, gripper_grasped_ = false;
  bool wait_localization_ = false, localized_ = false;
  std::string localization_;
  bool signal_written_ = false, allocation_written_ = false;
  bool request_sent_ = false, request_busy_ = false, request_complete_ = false;
  double skeleton_action_s_ = 0.5, action_at_ = 0, start_sent_at_ = -1e9;
  rclcpp::Publisher<String>::SharedPtr state_pub_, planning_command_pub_, approach_pub_, leave_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr signal_reset_pub_, survey_reset_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MissionMaster>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_master"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
