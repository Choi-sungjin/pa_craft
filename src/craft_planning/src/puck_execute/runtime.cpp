#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/parameter_client.hpp>
#include <ctime>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
namespace puck::execute {

Runner::Runner(bool planned) : Node("puck_pick_execute"), planned_(planned), task_id_(std::time(nullptr)) {
  puck_pubs_["uppercam"] = create_publisher<craft_perception::msg::PuckObservationArray>("/perception/pucks/picker_upper", 10);
  puck_pubs_["d435"] = create_publisher<craft_perception::msg::PuckObservationArray>("/perception/pucks/picker_grip", 10);
  for (const auto &[role, prefix] : std::vector<std::pair<std::string, std::string>>{
           {"uppercam", "/perception/uppercam"}, {"d435", "/vision/d435"}}) {
    camera_ack_[role] = false;
    camera_pause_[role] = create_publisher<std_msgs::msg::Bool>(prefix + "/pick_pause", 10);
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(prefix + "/pick_paused", 10,
      [this, role](const std_msgs::msg::Bool &m) { std::lock_guard<std::mutex> lock(mutex_); camera_ack_[role] = m.data; }));
  }
  timers_.push_back(create_wall_timer(200ms, [this] {
    std::optional<std::string> role;
    { std::lock_guard<std::mutex> lock(mutex_); role = camera_leased_; }
    if (role) { std_msgs::msg::Bool m; m.data = true; camera_pause_.at(*role)->publish(m); }
  }));
  lifecycle_ = create_publisher<std_msgs::msg::String>("/planning/command", 10);
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/planning/state", state_qos,
    [this](const std_msgs::msg::String &m) { std::lock_guard<std::mutex> lock(mutex_); planning_state_ = m.data; }));
  cmd_ = create_publisher<TaskCommand>(planned_ ? "/planning/pick_step" : "/planning/task_command", 10);
  if (!planned_) {
    hold_ = create_publisher<std_msgs::msg::Bool>("/planning/hold", 10);
    timers_.push_back(create_wall_timer(100ms, [this] { std_msgs::msg::Bool m; m.data = hold_value_; hold_->publish(m); }));
  }
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/control/arm_joints", 10,
    [this](const sensor_msgs::msg::JointState &m) { std::lock_guard<std::mutex> lock(mutex_); joints_ = m; }));
  subscriptions_.push_back(create_subscription<TaskResult>(planned_ ? "/planning/pick_step_result" : "/control/task_result", 10,
    [this](const TaskResult &m) { std::lock_guard<std::mutex> lock(mutex_); result_ = m; }));
  subscriptions_.push_back(create_subscription<craft_interfaces::msg::ControlStatus>("/control/status", 10,
    [this](const craft_interfaces::msg::ControlStatus &m) { std::lock_guard<std::mutex> lock(mutex_); status_ = m; }));
  subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>("/control/gripper_grasped", 10,
    [this](const std_msgs::msg::Bool &m) { std::lock_guard<std::mutex> lock(mutex_); grasped_ = m.data; }));
}

uint64_t Runner::send(TaskCommand command) {
  if (planned_) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (true) {
      if (is_interrupted()) throw std::runtime_error("interrupted");
      auto state = planning_state();
      if (state && *state == "Run.AwaitRequest") break;
      if ((state && (*state == "Fault" || *state == "Ready")) || std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("planning cannot accept a pick step: " + state.value_or("None"));
      std::this_thread::sleep_for(20ms);
    }
  }
  command.header.stamp = now();
  command.task_id = ++task_id_;
  command.revision = 1;
  command.operation = TaskCommand::EXECUTE;
  { std::lock_guard<std::mutex> lock(mutex_); result_.reset(); }
  cmd_->publish(command);
  return command.task_id;
}

std::pair<std::string, std::string> Runner::wait(uint64_t id, double timeout_s, uint64_t revision) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < until) {
    if (is_interrupted()) { stop(); return {"STOPPED", "interrupted"}; }
    std::optional<TaskResult> result;
    { std::lock_guard<std::mutex> lock(mutex_); result = result_; }
    if (result && result->task_id == id && (!revision || result->revision == revision) &&
        result->status != TaskResult::RUNNING && result->status != TaskResult::STOPPING) {
      static const std::map<unsigned, std::string> names{{0,"RUNNING"},{1,"SUCCEEDED"},{2,"FAILED"},{3,"STOPPING"},{4,"STOPPED"},{5,"REJECTED"}};
      auto it = names.find(result->status);
      return {it == names.end() ? std::to_string(result->status) : it->second, result->reason};
    }
    std::this_thread::sleep_for(50ms);
  }
  return {"TIMEOUT", "no terminal result"};
}

std::pair<std::string, std::string> Runner::stream_arm(const std::vector<std::string> &names,
    const std::vector<double> &start, const std::vector<std::vector<double>> &targets,
    double waypoint_seconds) {
  using Clock=std::chrono::steady_clock;
  if(planned_ || targets.empty() || names.size()!=5 || start.size()!=5 ||
      !std::isfinite(waypoint_seconds)||waypoint_seconds<=0. ||
      std::any_of(targets.begin(),targets.end(),[](const auto &target){return target.size()!=5;}))
    return {"FAILED","invalid nominal escape stream"};
  auto parameters=std::make_shared<rclcpp::AsyncParametersClient>(this,"/control_master");
  if(!parameters->wait_for_service(1s))
    return {"FAILED","cannot check control_master STREAM setting"};
  auto response=parameters->get_parameters({"arm_stream_direct_ms"});
  if(response.wait_for(1s)!=std::future_status::ready)
    return {"FAILED","control_master STREAM setting timed out"};
  const auto settings=response.get();
  if(settings.size()!=1||settings.front().get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER||
      settings.front().as_int()<10||settings.front().as_int()>500)
    return {"FAILED","OpenCR STREAM is disabled in control_master"};
  const auto stream_ms=settings.front().as_int();
  const double lead=stream_ms/1000.;
  constexpr double tolerance=M_PI/180.;
  std::cout<<"   OpenCR STREAM,"<<stream_ms<<", targets at 20 Hz\n";
  const double spacing=waypoint_seconds;
  double first_segment_seconds=spacing;
  auto initial=start;
  auto interpolate=[&](double seconds) {
    const double sample=seconds<=first_segment_seconds?seconds/first_segment_seconds:
      1.+(seconds-first_segment_seconds)/spacing;
    const double bounded=std::clamp(sample,0.,static_cast<double>(targets.size()));
    const auto index=std::min(static_cast<std::size_t>(bounded),targets.size()-1);
    const auto &a=index==0?initial:targets[index-1];
    const auto &b=targets[index];
    auto q=a;
    for(std::size_t axis=0;axis<q.size();++axis)q[axis]+=(bounded-index)*(b[axis]-a[axis]);
    return q;
  };
  auto measured_pose=[&]() -> std::optional<std::vector<double>> {
    const auto msg=joints();
    if(!msg||msg->name.size()!=msg->position.size())return std::nullopt;
    const auto age=now().nanoseconds()-rclcpp::Time(msg->header.stamp).nanoseconds();
    if(age<0||age>500000000LL)return std::nullopt;
    std::vector<double> measured;
    for(const auto &name:names) {
      auto joint=std::find(msg->name.begin(),msg->name.end(),name);
      if(joint==msg->name.end())return std::nullopt;
      const double value=msg->position[joint-msg->name.begin()];
      if(!std::isfinite(value))return std::nullopt;
      measured.push_back(value);
    }
    return measured;
  };
  auto tracking_error=[&](const std::vector<double> &measured,const std::vector<double> &q,bool starting) -> std::string {
    for(std::size_t axis=0;axis<q.size();++axis) {
      const double current=measured[axis];
      if(axis==0&&starting) {
        if(!pick_start_lm_ok(current,q[axis]))return names[axis]+" outside symmetric nominal start range";
        continue;
      }
      const double error=std::abs(current-q[axis]);
      const double threshold=axis==0?.015:(starting?pick_start_joint_tolerance_rad:10.*tolerance);
      if(!(error<=threshold)) {
        const double scale=axis==0?1000.:180./M_PI;
        std::ostringstream message;
        message<<names[axis]<<" error "<<(current-q[axis])*scale
          <<(axis==0?" mm":" deg")<<", limit "<<threshold*scale;
        return message.str();
      }
    }
    return {};
  };
  // Keep both the path clock and its lookahead target within half the stop tolerance.
  auto pace_to_feedback=[&](const std::vector<double> &measured,double from,double to) {
    auto close_enough=[&](double time) {
      const auto q=interpolate(time);
      for(std::size_t axis=0;axis<q.size();++axis)
        if(std::abs(measured[axis]-q[axis])>(axis==0?.0075:5.*tolerance))return false;
      return true;
    };
    if(!close_enough(from))return from;
    if(close_enough(to))return to;
    for(int i=0;i<10;++i) {
      const double middle=(from+to)*.5;
      if(close_enough(middle))from=middle;
      else to=middle;
    }
    return from;
  };
  const auto measured=measured_pose();
  if(!measured)return {"FAILED","no fresh arm feedback before the stream"};
  if(auto error=tracking_error(*measured,start,true);!error.empty())
    return {"FAILED","arm left nominal tolerance: "+error};
  initial=*measured;
  // Join the recorded path from measured feedback with all axes in the same segment.
  const double angular_rate=std::min(50.*.229*2.*M_PI/60.,7.5*tolerance/spacing);
  first_segment_seconds=std::max(spacing,std::abs(targets.front()[0]-initial[0])/.028);
  for(std::size_t axis=1;axis<5;++axis)
    first_segment_seconds=std::max(first_segment_seconds,std::abs(targets.front()[axis]-initial[axis])/angular_rate);
  const double duration=first_segment_seconds+spacing*(targets.size()-1);
  TaskCommand point;
  point.kind=TaskCommand::MANIPULATOR;
  point.header.frame_id="arm_stream";  // Select STREAM from the opening EXECUTE onward.
  point.tolerance_rad=tolerance;
  point.duration_s=lead;
  point.joints.name=names;
  point.joints.position=interpolate(pace_to_feedback(initial,0.,std::min(.05,duration)));
  auto id=send(point);
  uint64_t revision=1;
  auto result_for=[&] {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_&&result_->task_id==id?result_:std::nullopt;
  };
  const auto ack_limit=Clock::now()+1s;
  while(!result_for()&&Clock::now()<ack_limit&&!is_interrupted())std::this_thread::sleep_for(10ms);
  if(!result_for())return {"FAILED","no acknowledgement for nominal escape stream"};
  auto previous_tick=Clock::now();
  auto last_progress=previous_tick;
  double elapsed=0.;
  bool pacing_reported=false;
  auto next=previous_tick+50ms;
  for(;;) {
    std::this_thread::sleep_until(next);
    if(is_interrupted())return {"STOPPED","interrupted"};
    const auto tick=Clock::now();
    const double dt=std::chrono::duration<double>(tick-previous_tick).count();
    previous_tick=tick;
    const auto result=result_for();
    if(result&&result->status!=TaskResult::RUNNING&&result->status!=TaskResult::SUCCEEDED)
      return {"FAILED",result->reason};
    const auto tracking=measured_pose();
    if(!tracking)return {"FAILED","stale arm feedback during nominal escape"};
    if(auto error=tracking_error(*tracking,interpolate(elapsed),false);!error.empty())
      return {"FAILED","coordinated stream at "+std::to_string(elapsed)+" s: "+error};
    const double requested=std::min(elapsed+dt,duration);
    const double paced=pace_to_feedback(*tracking,elapsed,requested);
    if(paced<requested-1e-6&&!pacing_reported) {
      std::cout<<"   STREAM slowing to match joint feedback\n";
      pacing_reported=true;
    }
    if(paced>elapsed+1e-6)last_progress=tick;
    else if(tick-last_progress>=5s)return {"FAILED","STREAM made no path progress for 5 s"};
    elapsed=paced;
    const bool final=elapsed>=duration;
    point.joints.position=interpolate(pace_to_feedback(*tracking,elapsed,std::min(elapsed+lead,duration)));
    point.duration_s=lead;
    if(result&&result->status==TaskResult::SUCCEEDED&&result->revision==revision) {
      id=send(point);
      revision=1;
    } else {
      point.header.stamp=now();
      point.task_id=id;
      point.revision=++revision;
      point.operation=TaskCommand::UPDATE_PATH;
      cmd_->publish(point);
    }
    if(final)return wait(id,5.,revision);
    next=std::max(next+50ms,tick+50ms);
  }
}

// shift_m (re-dock only): sideways from this box's last dock [m], + left; planning reads it from tracking_reference_x.
bool Runner::drive(const std::string &box, bool reverse, double shift_m) {
  TaskCommand c; c.kind = TaskCommand::NAVIGATE; c.header.frame_id = box; c.speed = reverse ? -0.28 : 0.0;
  if (!reverse) c.tracking_reference_x = shift_m;
  auto [state, reason] = wait(send(c), 180.0);
  std::cout << "   -> " << state << ": " << reason << '\n';
  return state == "SUCCEEDED";
}

void Runner::stop() {
  if (planned_) { std_msgs::msg::String m; m.data = "STOP"; lifecycle_->publish(m); }
  else { TaskCommand c; c.operation = TaskCommand::STOP; c.task_id = 0; cmd_->publish(c); hold_value_ = true; }
}
void Runner::set_hold(bool value) { hold_value_ = value; }
void Runner::lease_camera(const std::string &role) {
  auto pub = camera_pause_.at(role);
  if (!planned_) return;
  // A fresh puck_execute may not have discovered the camera driver yet (2026-11-16 14:25: driver up 85 s, failed at
  // once): wait up to 5 s for its pick_pause subscription before calling it missing.
  const auto found_by = std::chrono::steady_clock::now() + 5s;
  while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < found_by && !is_interrupted())
    std::this_thread::sleep_for(50ms);
  if (!pub->get_subscription_count()) throw std::runtime_error(role + ": project camera driver with pick_pause support is not running");
  { std::lock_guard<std::mutex> lock(mutex_); camera_ack_.at(role) = false; camera_leased_ = role; }
  std_msgs::msg::Bool m; m.data = true; pub->publish(m);
  const auto until = std::chrono::steady_clock::now() + 5s;
  while (true) {
    if (is_interrupted()) { release_camera(role); throw std::runtime_error("interrupted"); }
    { std::lock_guard<std::mutex> lock(mutex_); if (camera_ack_.at(role)) return; }
    if (std::chrono::steady_clock::now() >= until) {
      release_camera(role);
      throw std::runtime_error(role + ": camera release acknowledgement timed out");
    }
    std::this_thread::sleep_for(20ms);
  }
}
void Runner::release_camera(const std::string &role) {
  if (!planned_) return;
  { std::lock_guard<std::mutex> lock(mutex_); camera_leased_.reset(); }
  std_msgs::msg::Bool m; m.data = false; camera_pause_.at(role)->publish(m);
}
void Runner::publish_observation(const std::string &role, craft_perception::msg::PuckObservationArray message) { puck_pubs_.at(role)->publish(message); }
size_t Runner::observation_subscribers(const std::string &role) const { return puck_pubs_.at(role)->get_subscription_count(); }
std::optional<std::string> Runner::planning_state() const { std::lock_guard<std::mutex> lock(mutex_); return planning_state_; }
std::optional<craft_interfaces::msg::ControlStatus> Runner::status() const { std::lock_guard<std::mutex> lock(mutex_); return status_; }
std::optional<sensor_msgs::msg::JointState> Runner::joints() const { std::lock_guard<std::mutex> lock(mutex_); return joints_; }
std::optional<bool> Runner::grasped() const { std::lock_guard<std::mutex> lock(mutex_); return grasped_; }
}  // namespace puck::execute
