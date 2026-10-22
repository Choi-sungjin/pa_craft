#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "craft_planning/state_machine.hpp"

namespace craft_planning
{

TEST(StateMachine, NavigationWaitsForReadinessThenComputesAndCommandsOnce)
{
  StateMachine fsm;
  fsm.start(StepKind::Navigate, 0.0);
  EXPECT_EQ(fsm.state(), PlanningState::Waiting);
  EXPECT_FALSE(fsm.tick({}, 0.1).start_compute);

  FsmInput input;
  input.ready = true;
  EXPECT_TRUE(fsm.tick(input, 0.2).start_compute);
  EXPECT_EQ(fsm.state(), PlanningState::Computing);
  EXPECT_FALSE(fsm.tick(input, 0.3).start_compute);

  input.compute_done = true;
  input.compute_ok = true;
  EXPECT_TRUE(fsm.tick(input, 0.4).send_command);
  EXPECT_EQ(fsm.state(), PlanningState::Executing);
  EXPECT_FALSE(fsm.tick(input, 0.5).send_command);
  EXPECT_EQ(fsm.kind(), StepKind::Navigate);
}

TEST(StateMachine, ImmediateStepsSkipComputation)
{
  for (const auto kind : {StepKind::Gripper, StepKind::Clock, StepKind::Wait}) {
    StateMachine fsm;
    fsm.start(kind, 0.0);
    FsmInput input;
    input.ready = true;
    const auto output = fsm.tick(input, 0.1);
    EXPECT_TRUE(output.send_command);
    EXPECT_FALSE(output.start_compute);
    EXPECT_EQ(fsm.state(), PlanningState::Executing);
  }
}

TEST(StateMachine, CompletionMustRemainTrueForDebounce)
{
  StateMachine fsm;
  fsm.start(StepKind::Gripper, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  input.completed = true;
  EXPECT_FALSE(fsm.tick(input, 0.2).stop);
  input.completed = false;
  fsm.tick(input, 0.25);
  input.completed = true;
  EXPECT_FALSE(fsm.tick(input, 0.3).stop);
  EXPECT_TRUE(fsm.tick(input, 0.41).stop);
  EXPECT_EQ(fsm.state(), PlanningState::Succeeded);
  EXPECT_TRUE(fsm.tick(input, 0.5).stop);
}

TEST(StateMachine, WaitingTimesOutWithoutReadiness)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  EXPECT_TRUE(fsm.tick({}, 5.0).stop);
  EXPECT_EQ(fsm.state(), PlanningState::Fault);
  EXPECT_NE(fsm.reason().find("ready"), std::string::npos);
}

TEST(StateMachine, ComputeTimeoutDoesNotStartConcurrentRetry)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  const auto output = fsm.tick(input, 3.1);
  EXPECT_TRUE(output.stop);
  EXPECT_FALSE(output.start_compute);
  EXPECT_EQ(fsm.state(), PlanningState::Fault);
  EXPECT_NE(fsm.reason().find("compute"), std::string::npos);
}

TEST(StateMachine, FailedComputationRetriesOnlyConfiguredNumberOfTimes)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  const auto generation = fsm.generation();
  input.compute_done = true;
  input.error = "unreachable pose";
  EXPECT_TRUE(fsm.tick(input, 0.2).start_compute);
  EXPECT_GT(fsm.generation(), generation);
  EXPECT_TRUE(fsm.tick(input, 0.3).stop);
  EXPECT_EQ(fsm.state(), PlanningState::Fault);
  EXPECT_EQ(fsm.reason(), "unreachable pose");
}

TEST(StateMachine, ExecutionTimeoutPrecedesCompletion)
{
  StateMachine fsm;
  fsm.start(StepKind::Clock, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  input.completed = true;
  const auto output = fsm.tick(input, 30.1);
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(fsm.state(), PlanningState::Fault);
  EXPECT_NE(fsm.reason().find("execution"), std::string::npos);
}

TEST(StateMachine, FaultAndStaleFeedbackPrecedeCompletion)
{
  for (const bool hardware_fault : {false, true}) {
    StateMachine fsm;
    fsm.start(StepKind::Clock, 0.0);
    FsmInput input;
    input.ready = true;
    fsm.tick(input, 0.1);
    input.completed = true;
    fsm.tick(input, 0.2);
    input.healthy = !hardware_fault;
    input.feedback_fresh = hardware_fault;
    input.error = hardware_fault ? "clock fault" : "clock feedback stale";
    EXPECT_TRUE(fsm.tick(input, 0.4).stop);
    EXPECT_EQ(fsm.state(), PlanningState::Fault);
    EXPECT_EQ(fsm.reason(), input.error);
  }
}

TEST(StateMachine, MissingFeedbackCanWaitBeforeExecution)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  FsmInput input;
  input.feedback_fresh = false;
  EXPECT_FALSE(fsm.tick(input, 0.1).stop);
  input.ready = true;
  EXPECT_TRUE(fsm.tick(input, 0.2).start_compute);
  EXPECT_FALSE(fsm.tick(input, 0.3).stop);
}

TEST(StateMachine, ImmediateCommandWaitsForFreshFeedbackEvenIfReady)
{
  StateMachine fsm;
  fsm.start(StepKind::Clock, 0.0);
  FsmInput input;
  input.ready = true;
  input.feedback_fresh = false;
  EXPECT_FALSE(fsm.tick(input, 0.1).send_command);
  EXPECT_EQ(fsm.state(), PlanningState::Waiting);
  input.feedback_fresh = true;
  EXPECT_TRUE(fsm.tick(input, 0.2).send_command);
}

TEST(StateMachine, ComputationCannotDispatchCommandWithStaleFeedback)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  input.compute_done = true;
  input.compute_ok = true;
  input.feedback_fresh = false;
  const auto output = fsm.tick(input, 0.2);
  EXPECT_FALSE(output.send_command);
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(fsm.state(), PlanningState::Fault);
}

TEST(StateMachine, CancelInvalidatesComputationAndIgnoresLateResults)
{
  StateMachine fsm;
  fsm.start(StepKind::Tcp, 0.0);
  FsmInput input;
  input.ready = true;
  fsm.tick(input, 0.1);
  const auto generation = fsm.generation();
  fsm.cancel(0.2);
  EXPECT_GT(fsm.generation(), generation);
  input.compute_done = true;
  input.compute_ok = true;
  const auto output = fsm.tick(input, 0.3);
  EXPECT_TRUE(output.stop);
  EXPECT_FALSE(output.send_command);
  EXPECT_EQ(fsm.state(), PlanningState::Cancelled);
  fsm.start(StepKind::Clock, 0.4);
  EXPECT_EQ(fsm.state(), PlanningState::Waiting);
}

TEST(StateMachine, FaultRequiresExplicitCancellationBeforeRestart)
{
  StateMachine fsm;
  fsm.start(StepKind::Clock, 0.0);
  EXPECT_THROW(fsm.start(StepKind::Tcp, 0.1), std::logic_error);
  fsm.fail("board fault", 0.2);
  EXPECT_THROW(fsm.start(StepKind::Tcp, 0.3), std::logic_error);
  EXPECT_TRUE(fsm.tick({}, 0.4).stop);
  fsm.cancel(0.5);
  EXPECT_NO_THROW(fsm.start(StepKind::Tcp, 0.6));
}

TEST(StateMachine, InvalidConfigurationAndNonmonotonicTimeAreRejected)
{
  StateMachine fsm;
  StateMachineConfig config;
  config.execution_timeout_s = 0.0;
  EXPECT_THROW(fsm.configure(config), std::invalid_argument);
  config = StateMachineConfig{};
  config.completion_debounce_s = -1.0;
  EXPECT_THROW(fsm.configure(config), std::invalid_argument);
  config = StateMachineConfig{};
  config.max_compute_retries = -1;
  EXPECT_THROW(fsm.configure(config), std::invalid_argument);
  EXPECT_THROW(fsm.start(StepKind::Wait, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  fsm.start(StepKind::Wait, 2.0);
  EXPECT_THROW(fsm.tick({}, 1.0), std::invalid_argument);
  EXPECT_THROW(fsm.tick({}, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(CompletionGate, IgnoresOldTrueUntilFalseThenFreshTrue)
{
  CompletionGate gate;
  gate.arm();
  gate.observe(true, 0.0);
  EXPECT_FALSE(gate.complete(0.1, 0.5));
  gate.observe(false, 0.2);
  EXPECT_FALSE(gate.complete(0.2, 0.5));
  gate.observe(true, 0.3);
  EXPECT_TRUE(gate.complete(0.4, 0.5));
  EXPECT_FALSE(gate.complete(0.9, 0.5));
}

TEST(CompletionGate, RearmingAndFalseFeedbackClearCompletion)
{
  CompletionGate gate;
  gate.arm();
  gate.observe(false, 0.0);
  gate.observe(true, 0.1);
  EXPECT_TRUE(gate.complete(0.2, 0.5));
  gate.observe(false, 0.3);
  EXPECT_FALSE(gate.complete(0.3, 0.5));
  gate.observe(true, 0.4);
  gate.arm();
  EXPECT_FALSE(gate.complete(0.5, 0.5));
  gate.observe(true, 0.6);
  EXPECT_FALSE(gate.complete(0.6, 0.5));
}

TEST(CompletionGate, RejectsInvalidTimesAndFreshnessTimeouts)
{
  CompletionGate gate;
  gate.arm();
  EXPECT_THROW(gate.observe(false, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  gate.observe(false, 1.0);
  EXPECT_THROW(gate.observe(true, 0.0), std::invalid_argument);
  EXPECT_THROW(gate.complete(0.0, 0.5), std::invalid_argument);
  EXPECT_THROW(gate.complete(1.0, 0.0), std::invalid_argument);
}

}  // namespace craft_planning
