#include "craft_planning/state_machine.hpp"
#include <cmath>
#include <stdexcept>
namespace craft_planning {
void StateMachine::configure(const StateMachineConfig &c) {
  for (double t : {c.initialization_timeout_s, c.ready_timeout_s, c.compute_timeout_s, c.execution_timeout_s,
                   c.feedback_timeout_s, c.stop_timeout_s})
    if (!std::isfinite(t) || t <= 0) throw std::invalid_argument("invalid FSM timeout");
  if (c.max_compute_retries < 0) throw std::invalid_argument("invalid retry count");
  config_ = c;
}
void StateMachine::check_time(double now) {
  if (!std::isfinite(now) || (last_now_ && now < *last_now_)) throw std::logic_error("nonmonotonic FSM clock");
  last_now_ = now;
}
void StateMachine::initialize(double now) {
  check_time(now); state_ = SystemState::Idle; phase_.reset(); since_ = now;
  reason_.clear(); ++generation_;
}
bool StateMachine::begin_run(double now) {
  check_time(now);
  if (state_ != SystemState::Ready) return false;
  state_ = SystemState::Run; enter(RunPhase::AwaitRequest, now); return true;
}
void StateMachine::enter(RunPhase phase, double now) {phase_ = phase; since_ = now;}
bool StateMachine::start(StepKind, double now) {
  check_time(now);
  if (phase_ != RunPhase::AwaitRequest || state_ != SystemState::Run) return false;
  ++generation_; retries_ = 0; enter(RunPhase::PrepareTask, now); return true;
}
bool StateMachine::retry_compute(double now) {
  check_time(now);
  if (state_ != SystemState::Run || phase_ != RunPhase::Compute) return false;
  ++generation_; since_ = now; return true;
}
void StateMachine::request_stop(bool end_run, double now) {
  check_time(now);
  if (state_ != SystemState::Run) return;
  end_run_ = end_run_ || end_run;
  if (phase_ == RunPhase::Stopping) return; // repeated STOP cannot extend timeout
  ++generation_; enter(RunPhase::Stopping, now);
}
bool StateMachine::reset(bool stopped, double now) {
  if (state_ != SystemState::Fault || !stopped) return false;
  end_run_ = false; initialize(now); return true;
}
void StateMachine::fail(const std::string &reason, double now) {
  check_time(now);
  if (state_ == SystemState::Fault) return; // preserve original cause
  reason_ = label() + ": " + reason;
  state_ = SystemState::Fault; phase_.reset(); since_ = now; ++generation_;
}
FsmOutput StateMachine::tick(const FsmInput &i, double now) {
  check_time(now);
  if (state_ == SystemState::Fault) return {false, false, true};
  if (state_ == SystemState::Idle) {
    if (i.ready && i.stopped) {state_ = SystemState::Ready; since_ = now;}
    else if (now - since_ >= config_.initialization_timeout_s) fail("initialization timeout: " + i.error, now);
    return {false, false, true};
  }
  if (!i.healthy || !i.feedback_fresh) {
    fail(i.error.empty() ? "module/feedback fault" : i.error, now); return {false, false, true};
  }
  if (state_ == SystemState::Ready) return {false, false, true};
  const double elapsed = now - since_;
  switch (*phase_) {
    case RunPhase::AwaitRequest: return {false, false, true};
    case RunPhase::PrepareTask:
      if (elapsed >= config_.ready_timeout_s) fail("task preparation timeout: " + i.error, now);
      else if (i.ready) {enter(RunPhase::Compute, now); return {true, false, false};}
      break;
    case RunPhase::Compute:
      if (elapsed >= config_.compute_timeout_s) fail("compute timeout", now);
      else if (i.compute_done) {
        if (i.compute_ok) {enter(RunPhase::Execute, now); return {false, true, false};}
        if (retries_++ < config_.max_compute_retries) {
          ++generation_; enter(RunPhase::Compute, now); return {true, false, false};
        }
        fail("calculation failed: " + i.error, now);
      }
      break;
    case RunPhase::Execute:
      if (elapsed >= config_.execution_timeout_s) fail("execution timeout", now);
      else if (i.completed) enter(RunPhase::Complete, now);
      break;
    case RunPhase::Complete: enter(RunPhase::AwaitRequest, now); break;
    case RunPhase::Stopping:
      if (i.stopped) {
        if (end_run_) {state_ = SystemState::Ready; phase_.reset();}
        else enter(RunPhase::AwaitRequest, now);
        end_run_ = false;
      } else if (elapsed >= config_.stop_timeout_s) fail("stop acknowledgement timeout", now);
      return {false, false, true};
  }
  return {false, false, state_ == SystemState::Fault || phase_ == RunPhase::Complete};
}
std::string StateMachine::label() const {
  switch (state_) {
    case SystemState::Idle: return "Idle";
    case SystemState::Ready: return "Ready";
    case SystemState::Fault: return "Fault";
    case SystemState::Run: break;
  }
  switch (*phase_) {
    case RunPhase::AwaitRequest: return "Run.AwaitRequest";
    case RunPhase::PrepareTask: return "Run.PrepareTask";
    case RunPhase::Compute: return "Run.Compute";
    case RunPhase::Execute: return "Run.Execute";
    case RunPhase::Complete: return "Run.Complete";
    case RunPhase::Stopping: return "Run.Stopping";
  }
  return "Fault";
}
}
