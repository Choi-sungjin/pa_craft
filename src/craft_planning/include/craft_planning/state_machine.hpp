#ifndef CRAFT_PLANNING__STATE_MACHINE_HPP_
#define CRAFT_PLANNING__STATE_MACHINE_HPP_

#include <cstddef>
#include <optional>
#include <string>

namespace craft_planning
{

enum class PlanningState {Idle, Waiting, Computing, Executing, Succeeded, Fault, Cancelled};
enum class StepKind {Navigate, Tcp, Gripper, Clock, Wait};

struct StateMachineConfig
{
  double ready_timeout_s = 5.0;
  double compute_timeout_s = 3.0;
  double execution_timeout_s = 30.0;
  double feedback_timeout_s = 0.5;
  double completion_debounce_s = 0.1;
  int max_compute_retries = 1;
};

struct FsmInput
{
  bool ready = false;
  bool healthy = true;
  bool feedback_fresh = true;
  bool compute_done = false;
  bool compute_ok = false;
  bool completed = false;
  std::string error;
};

struct FsmOutput
{
  bool start_compute = false;
  bool send_command = false;
  bool stop = false;
};

// The caller supplies monotonic seconds and owns ROS messages and asynchronous jobs.
// Capture generation() when starting a job and discard results of older generations.
class StateMachine
{
public:
  void configure(const StateMachineConfig & config);
  void start(StepKind kind, double now);
  FsmOutput tick(const FsmInput & input, double now);
  void cancel(double now);
  void fail(const std::string & reason, double now);

  PlanningState state() const {return state_;}
  const std::string & reason() const {return reason_;}
  std::size_t generation() const {return generation_;}
  StepKind kind() const {return kind_;}

private:
  void check_time(double now);
  bool active() const;

  StateMachineConfig config_;
  PlanningState state_ = PlanningState::Idle;
  StepKind kind_ = StepKind::Wait;
  std::string reason_;
  std::size_t generation_ = 0;
  int compute_retries_ = 0;
  double state_since_ = 0.0;
  std::optional<double> last_now_;
  std::optional<double> completed_since_;
};

// Bool feedback has no goal identifier. A post-arm false followed by a fresh true
// prevents a cached true from completing a new command; it cannot correlate queued
// feedback from different commands without a goal acknowledgement in the protocol.
class CompletionGate
{
public:
  void arm();
  void observe(bool value, double now);
  bool complete(double now, double freshness_timeout_s) const;

private:
  bool armed_ = false;
  bool saw_false_ = false;
  bool value_ = false;
  std::optional<double> observed_at_;
};

}  // namespace craft_planning

#endif  // CRAFT_PLANNING__STATE_MACHINE_HPP_
