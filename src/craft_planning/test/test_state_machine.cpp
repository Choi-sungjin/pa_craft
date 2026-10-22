#include "craft_planning/state_machine.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace craft_planning;
static void ready(StateMachine &f) {
  f.initialize(0); FsmInput i; i.ready = i.stopped = true; f.tick(i, .1);
  ASSERT_EQ(f.state(), SystemState::Ready); ASSERT_FALSE(f.phase());
  ASSERT_TRUE(f.begin_run(.2));
}
TEST(StateMachine, LifecycleAndEveryTaskUsesCompute) {
  for (auto kind : {StepKind::Navigate, StepKind::Tcp, StepKind::Clock, StepKind::Gripper, StepKind::Wait}) {
    StateMachine f; ready(f);
    ASSERT_TRUE(f.start(kind, .3));
    EXPECT_EQ(f.label(), "Run.PrepareTask");
    FsmInput i; i.ready = true;
    EXPECT_TRUE(f.tick(i,.4).start_compute); EXPECT_EQ(f.phase(),RunPhase::Compute);
    i.compute_done = i.compute_ok = true;
    EXPECT_TRUE(f.tick(i,.5).send_command); EXPECT_EQ(f.phase(),RunPhase::Execute);
    EXPECT_FALSE(f.tick(i,.6).send_command);
    i.completed = true; f.tick(i,.7); EXPECT_EQ(f.phase(),RunPhase::Complete);
    f.tick(i,.8); EXPECT_EQ(f.phase(),RunPhase::AwaitRequest);
  }
}
TEST(StateMachine, ActiveTaskCannotBePreemptedAndStopWaitsForControl) {
  StateMachine f; ready(f); f.start(StepKind::Navigate,.3);
  EXPECT_FALSE(f.start(StepKind::Tcp,.4));
  const auto generation = f.generation(); f.request_stop(false,.5);
  EXPECT_GT(f.generation(),generation); EXPECT_EQ(f.phase(),RunPhase::Stopping);
  f.tick({},.6); EXPECT_EQ(f.phase(),RunPhase::Stopping);
  FsmInput i; i.stopped = true; f.tick(i,.7); EXPECT_EQ(f.phase(),RunPhase::AwaitRequest);
  f.request_stop(true,.8); f.tick(i,.9); EXPECT_EQ(f.state(),SystemState::Ready);
}
TEST(StateMachine, FaultLatchedUntilResetAndFreshInitialization) {
  StateMachine f; ready(f); FsmInput i; i.healthy = false; i.error = "board";
  f.tick(i,.3); EXPECT_EQ(f.state(),SystemState::Fault);
  const auto reason = f.reason(); f.fail("later",.4); EXPECT_EQ(f.reason(),reason);
  EXPECT_FALSE(f.begin_run(.5)); EXPECT_FALSE(f.reset(false,.6));
  EXPECT_TRUE(f.reset(true,.7)); EXPECT_EQ(f.state(),SystemState::Idle);
  EXPECT_FALSE(f.phase());
}
TEST(StateMachine, TimeoutsAndComputeRetryDoNotRetryMotion) {
  StateMachine f; f.initialize(0); f.tick({},9.9); EXPECT_EQ(f.state(),SystemState::Idle);
  f.tick({},10); EXPECT_EQ(f.state(),SystemState::Fault);
  StateMachine g; ready(g); g.start(StepKind::Navigate,.3);
  FsmInput i; i.ready = true; g.tick(i,.4); i.compute_done = true;
  EXPECT_TRUE(g.tick(i,.5).start_compute);
  g.tick(i,.6); EXPECT_EQ(g.state(),SystemState::Fault);
  StateMachine h; ready(h); h.request_stop(false,.3); h.request_stop(false,2);
  h.tick({},3.4); EXPECT_EQ(h.state(),SystemState::Fault);
  EXPECT_THROW(h.tick({},1),std::logic_error);
}
TEST(StateMachine, RouteRetryRefreshesOnlyTheComputeDeadline) {
  StateMachine f; ready(f); f.start(StepKind::Navigate,.3);
  FsmInput i; i.ready = true; f.tick(i,.4);
  const auto generation = f.generation();
  EXPECT_TRUE(f.retry_compute(3.3));
  EXPECT_GT(f.generation(), generation);
  EXPECT_EQ(f.state(), SystemState::Run);
  EXPECT_EQ(f.phase(), RunPhase::Compute);
  f.tick({}, 6.2); EXPECT_EQ(f.state(), SystemState::Run);
}
