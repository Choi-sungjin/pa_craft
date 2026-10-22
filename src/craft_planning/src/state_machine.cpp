#include "craft_planning/state_machine.hpp"

#include <cmath>
#include <stdexcept>

namespace craft_planning
{
namespace
{

void require_time(double now, const std::optional<double> & previous)
{
  if (!std::isfinite(now) || (previous && now < *previous)) {
    throw std::invalid_argument("time must be finite and monotonic");
  }
}

bool positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool StateMachine::active() const
{
  return state_ == PlanningState::Waiting || state_ == PlanningState::Computing ||
         state_ == PlanningState::Executing;
}

void StateMachine::check_time(double now)
{
  require_time(now, last_now_);
  last_now_ = now;
}

void StateMachine::configure(const StateMachineConfig & config)
{
  if (!positive(config.ready_timeout_s) || !positive(config.compute_timeout_s) ||
    !positive(config.execution_timeout_s) || !positive(config.feedback_timeout_s) ||
    !std::isfinite(config.completion_debounce_s) || config.completion_debounce_s < 0.0 ||
    config.max_compute_retries < 0)
  {
    throw std::invalid_argument("invalid state machine timeouts, debounce or retry count");
  }
  if (active()) {
    throw std::logic_error("cannot configure an active state machine");
  }
  config_ = config;
}

void StateMachine::start(StepKind kind, double now)
{
  if (active() || state_ == PlanningState::Fault) {
    throw std::logic_error("cancel the active or faulted step before starting another");
  }
  check_time(now);
  kind_ = kind;
  state_ = PlanningState::Waiting;
  state_since_ = now;
  compute_retries_ = 0;
  completed_since_.reset();
  reason_.clear();
  ++generation_;
}

void StateMachine::cancel(double now)
{
  check_time(now);
  state_ = PlanningState::Cancelled;
  state_since_ = now;
  completed_since_.reset();
  reason_ = "cancelled";
  ++generation_;
}

void StateMachine::fail(const std::string & reason, double now)
{
  check_time(now);
  state_ = PlanningState::Fault;
  state_since_ = now;
  completed_since_.reset();
  reason_ = reason.empty() ? "planning failed" : reason;
  ++generation_;
}

FsmOutput StateMachine::tick(const FsmInput & input, double now)
{
  check_time(now);
  if (!active()) {
    return {false, false, state_ != PlanningState::Idle};
  }
  if (!input.healthy || (state_ == PlanningState::Executing && !input.feedback_fresh)) {
    fail(input.error.empty() ?
      (input.healthy ? "control feedback stale" : "control fault") : input.error, now);
    return {false, false, true};
  }

  const double elapsed = now - state_since_;
  switch (state_) {
    case PlanningState::Waiting:
      if (elapsed >= config_.ready_timeout_s) {
        fail("ready timeout", now);
        return {false, false, true};
      }
      if (input.ready) {
        const bool compute = kind_ == StepKind::Navigate || kind_ == StepKind::Tcp;
        if (!compute && !input.feedback_fresh) {
          break;
        }
        state_since_ = now;
        state_ = compute ? PlanningState::Computing : PlanningState::Executing;
        return {compute, !compute, false};
      }
      break;
    case PlanningState::Computing:
      if (elapsed >= config_.compute_timeout_s) {
        fail("compute timeout", now);
        return {false, false, true};
      }
      if (input.compute_done) {
        if (input.compute_ok) {
          if (!input.feedback_fresh) {
            fail(input.error.empty() ? "control feedback stale" : input.error, now);
            return {false, false, true};
          }
          state_ = PlanningState::Executing;
          state_since_ = now;
          return {false, true, false};
        }
        if (compute_retries_ < config_.max_compute_retries) {
          ++compute_retries_;
          ++generation_;
          state_since_ = now;
          return {true, false, false};
        }
        fail(input.error.empty() ? "computation failed" : input.error, now);
        return {false, false, true};
      }
      break;
    case PlanningState::Executing:
      if (elapsed >= config_.execution_timeout_s) {
        fail("execution timeout", now);
        return {false, false, true};
      }
      if (!input.completed) {
        completed_since_.reset();
      } else {
        if (!completed_since_) {
          completed_since_ = now;
        }
        if (now - *completed_since_ >= config_.completion_debounce_s) {
          state_ = PlanningState::Succeeded;
          state_since_ = now;
          return {false, false, true};
        }
      }
      break;
    default:
      break;
  }
  return {};
}

void CompletionGate::arm()
{
  armed_ = true;
  saw_false_ = false;
  value_ = false;
  observed_at_.reset();
}

void CompletionGate::observe(bool value, double now)
{
  require_time(now, observed_at_);
  if (!armed_) {
    return;
  }
  observed_at_ = now;
  value_ = value;
  saw_false_ = saw_false_ || !value;
}

bool CompletionGate::complete(double now, double freshness_timeout_s) const
{
  require_time(now, observed_at_);
  if (!positive(freshness_timeout_s)) {
    throw std::invalid_argument("completion freshness timeout must be finite and positive");
  }
  return armed_ && saw_false_ && value_ && observed_at_ &&
         now - *observed_at_ <= freshness_timeout_s;
}

}  // namespace craft_planning
