#include "map_server/cost_values.hpp"
#include "nav_executor/motion_fsm.hpp"
#include "nav_executor/recovery_controller.hpp"
#include "nav_executor/safety_monitor.hpp"
#include "nav_executor/task_manager.hpp"
#include <gtest/gtest.h>
using namespace nav_executor;
namespace {
geometry_msgs::msg::PoseStamped pose(double x = 0, double y = 0) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1;
  return p;
}
nav_msgs::msg::Path path() {
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.poses = {pose(), pose(2, 0)};
  return p;
}
TaskManager::Clock::time_point at(double seconds) {
  return TaskManager::Clock::time_point{} +
         std::chrono::duration_cast<TaskManager::Clock::duration>(
             std::chrono::duration<double>(seconds));
}
} // namespace
TEST(MotionFSM, HoldSpinAndRecoveryNeverResumeAnOldRoute) {
  MotionFSM f;
  EXPECT_EQ(f.state(), MotionState::IDLE);
  f.consume(MotionEvent::PATH_ACCEPTED);
  EXPECT_EQ(f.state(), MotionState::FOLLOW);
  f.consume(MotionEvent::GOAL_REACHED);
  EXPECT_EQ(f.state(), MotionState::HOLD);
  f.consume(MotionEvent::SPIN_REQUESTED);
  EXPECT_EQ(f.state(), MotionState::PREPARE_SPIN);
  f.consume(MotionEvent::PATH_ACCEPTED);
  EXPECT_EQ(f.state(), MotionState::PREPARE_SPIN);
  f.consume(MotionEvent::SPIN_READY);
  EXPECT_EQ(f.state(), MotionState::SPIN);
  f.consume(MotionEvent::SPIN_RELEASED);
  EXPECT_EQ(f.state(), MotionState::IDLE);
  f.consume(MotionEvent::PATH_ACCEPTED);
  f.consume(MotionEvent::STALLED);
  EXPECT_EQ(f.state(), MotionState::ESCAPE);
  f.consume(MotionEvent::SPIN_REQUESTED);
  EXPECT_EQ(f.state(), MotionState::ESCAPE);
  f.consume(MotionEvent::HAZARD);
  EXPECT_EQ(f.state(), MotionState::HAZARD_RECOVERY);
  f.consume(MotionEvent::RECOVERY_SUCCEEDED);
  EXPECT_EQ(f.state(), MotionState::IDLE);
  f.consume(MotionEvent::STOP);
  EXPECT_EQ(f.state(), MotionState::IDLE);
}
TEST(SupervisorTask,
     RecoverySuspendsDispatchAndRejectsOldResultsWithoutChangingGoal) {
  TaskManager t;
  ASSERT_TRUE(t.setGoal(pose(2, 0)));
  const auto goal = t.identity().goal_id;
  auto request = t.nextRequest(true, at(1));
  ASSERT_TRUE(request);
  ASSERT_TRUE(t.accept({request->identity, path(), ""}, at(2)));
  const auto old = t.identity();
  ASSERT_TRUE(t.beginMotionOverride());
  EXPECT_FALSE(t.nextRequest(true, at(5)));
  EXPECT_FALSE(t.isPathConfirmed(old));
  EXPECT_FALSE(t.accept({old, path(), ""}, at(5)));
  ASSERT_TRUE(t.endMotionOverride(ReplanReason::RECOVERY_COMPLETED));
  auto next = t.nextRequest(true, at(6));
  ASSERT_TRUE(next);
  EXPECT_EQ(next->identity.goal_id, goal);
  EXPECT_GT(next->identity.plan_generation, old.plan_generation);
  EXPECT_EQ(t.lastReplanReason(), ReplanReason::RECOVERY_COMPLETED);
}
TEST(SupervisorTask, NewGoalPreemptsRecoveryAndCancellationRejectsLateResults) {
  TaskManager t;
  t.setGoal(pose(2, 0));
  auto old = t.nextRequest(true, at(1));
  ASSERT_TRUE(old);
  t.beginMotionOverride();
  t.setGoal(pose(3, 0));
  EXPECT_FALSE(t.motionOverride());
  auto next = t.nextRequest(true, at(2));
  ASSERT_TRUE(next);
  EXPECT_GT(next->identity.goal_id, old->identity.goal_id);
  EXPECT_FALSE(t.accept({old->identity, path(), ""}, at(3)));
  t.cancel();
  EXPECT_FALSE(t.accept({next->identity, path(), ""}, at(4)));
  EXPECT_EQ(t.state(), TaskState::IDLE);
}
TEST(SupervisorTask, TerminalRecoveryFailureDoesNotRetryAutomatically) {
  TaskManager t;
  t.setGoal(pose(2, 0));
  t.abort("No safe direction");
  EXPECT_EQ(t.state(), TaskState::FAILED);
  EXPECT_FALSE(t.nextRequest(true, at(1000)));
  ASSERT_TRUE(t.replan());
  EXPECT_TRUE(t.nextRequest(true, at(1001)));
}
TEST(SafetyMonitor, PhysicalStallNeedsFreshSustainedCommandAndResetsOnMotion) {
  SafetyMonitorConfig config;
  config.stall_duration = .5;
  SafetyMonitor m(config);
  geometry_msgs::msg::Twist measured, command;
  command.linear.y = .3;
  EXPECT_FALSE(m.observe(pose(), measured, command, at(1)));
  EXPECT_FALSE(m.observe(pose(), measured, command, at(1.2)));
  EXPECT_FALSE(m.observe(pose(), measured, command, at(1.4)));
  EXPECT_TRUE(m.observe(pose(), measured, command, at(1.6)));
  measured.linear.y = .2;
  EXPECT_FALSE(m.observe(pose(), measured, command, at(1.8)));
  measured.linear.y = 0;
  EXPECT_FALSE(m.observe(pose(), measured, command, at(2)));
  EXPECT_FALSE(m.observe(pose(), measured, command,
                         at(5))); // stale gap cannot count as stuck
  command.linear.y = 0;
  command.angular.z = .5;
  EXPECT_FALSE(m.observe(pose(), measured, command, at(5.2)));
}
TEST(Recovery, EntireSweptBodyIncludesOffCenterObstaclesUnknownAndBoundaries) {
  map_server::Costmap2D g(100, 100, .05, -2.5, -2.5, map_server::FREE_SPACE);
  EXPECT_TRUE(sweptFootprintSafe(g, 0, 0, 1, 0, .3));
  unsigned int x, y;
  ASSERT_TRUE(g.worldToMap(.5, .22, x, y));
  g.setCost(x, y, map_server::LETHAL_OBSTACLE);
  EXPECT_FALSE(sweptFootprintSafe(g, 0, 0, 1, 0, .3));
  g.setCost(x, y, map_server::NO_INFORMATION);
  EXPECT_FALSE(sweptFootprintSafe(g, 0, 0, 1, 0, .3));
  g.setCost(x, y, map_server::FREE_SPACE);
  EXPECT_FALSE(sweptFootprintSafe(g, 0, 0, 2.4, 0, .3));
}
TEST(Recovery, SelectsSafeOmnidirectionalEscapeAndStopsWhenSurrounded) {
  map_server::Costmap2D g(100, 100, .05, -2.5, -2.5, map_server::FREE_SPACE);
  RecoveryController recovery;
  auto target = recovery.select(g, pose(), .2, false);
  ASSERT_TRUE(target);
  EXPECT_LT(target->x, 0);
  EXPECT_TRUE(sweptFootprintSafe(g, 0, 0, target->x, target->y, .23));
  for (unsigned int y = 0; y < 100; ++y)
    for (unsigned int x = 0; x < 100; ++x)
      g.setCost(x, y, map_server::LETHAL_OBSTACLE);
  EXPECT_FALSE(recovery.select(g, pose(), .2, true));
}
