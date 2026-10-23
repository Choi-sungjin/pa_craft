#pragma once
#include <cstddef>
#include <optional>
#include <string>
namespace craft_planning {
enum class SystemState {Idle, Ready, Run, Fault};
enum class RunPhase {AwaitRequest, PrepareTask, Compute, Execute, Complete, Stopping};
enum class StepKind {Navigate, Tcp, Gripper, Clock, Wait};
struct StateMachineConfig {
  double initialization_timeout_s = 10, ready_timeout_s = 5;
  double compute_timeout_s = 3, execution_timeout_s = 30;
  double feedback_timeout_s = 0.5, stop_timeout_s = 3;
  int max_compute_retries = 1;
};
struct FsmInput {
  bool ready = false, healthy = true, feedback_fresh = true;
  bool compute_done = false, compute_ok = false, completed = false, stopped = false;
  std::string error;
};
struct FsmOutput { bool start_compute = false, send_command = false, stop = false; };
class StateMachine {
public:
  void configure(const StateMachineConfig &config);
  void initialize(double now);
  bool begin_run(double now);
  bool start(StepKind kind, double now);
  bool retry_compute(double now);
  void request_stop(bool end_run, double now);
  bool reset(bool stopped, double now);
  void fail(const std::string &reason, double now);
  FsmOutput tick(const FsmInput &input, double now);
  SystemState state() const {return state_;}
  std::optional<RunPhase> phase() const {return phase_;}
  const std::string &reason() const {return reason_;}
  std::size_t generation() const {return generation_;}
  bool active() const {return state_ == SystemState::Run;}
  std::string label() const;
private:
  void check_time(double now);
  void enter(RunPhase phase, double now);
  StateMachineConfig config_;
  SystemState state_ = SystemState::Idle;
  std::optional<RunPhase> phase_;
  std::optional<double> last_now_;
  std::string reason_;
  double since_ = 0;
  std::size_t generation_ = 0;
  int retries_ = 0;
  bool end_run_ = false;
};
}
