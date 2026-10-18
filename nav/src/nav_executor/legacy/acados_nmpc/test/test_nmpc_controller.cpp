#include "nav_executor/mpc/geometry.hpp"
#include "nav_executor/mpc_controller.hpp"
#include "nav_executor/route_monitor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace
{
using Vector = Eigen::Vector2d;
namespace mpc = nav_executor::mpc;
class NmpcTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void configure(std::vector<rclcpp::Parameter> parameters = {})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    node = std::make_shared<rclcpp::Node>("test_nmpc", options);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    controller = std::make_unique<nav_executor::MpcController>();
    controller->initialize(node, tf, &map, "odom", "base_link_fake", 0.15);
  }
  nav_msgs::msg::Path path(std::vector<Vector> points = {{2, 2}, {4, 2}},
                           std::string frame = "odom")
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = frame;
    for (const auto &p : points)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.orientation.w = 1;
      result.poses.push_back(pose);
    }
    return result;
  }
  geometry_msgs::msg::PoseStamped pose(Vector p = {2, 2}, double yaw = 0)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "odom";
    result.pose.position.x = p.x();
    result.pose.position.y = p.y();
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    result.pose.orientation = tf2::toMsg(q);
    return result;
  }
  map_server::Costmap2D map{200, 200, 0.05, 0, 0, map_server::FREE_SPACE};
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav_executor::MpcController> controller;
};

TEST_F(NmpcTest, StraightPathProducesBoundedAcceleration)
{
  configure();
  controller->setPlan(path());
  const auto cmd = controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{});
  EXPECT_GT(cmd.twist.linear.x, 0.01);
  EXPECT_LE(std::hypot(cmd.twist.linear.x, cmd.twist.linear.y), 0.1003);
  EXPECT_NEAR(cmd.twist.linear.y, 0, 1e-3);
  EXPECT_EQ(cmd.header.frame_id, "base_link_fake");
  EXPECT_TRUE(controller->feedback().valid);
}
TEST_F(NmpcTest, TransformedPathAndFakeFrameCommand)
{
  configure();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.transform.translation.x = 1;
  transform.transform.rotation.w = 1;
  tf->setTransform(transform, "test", true);
  controller->setPlan(path({{1, 2}, {3, 2}}, "map"));
  const auto cmd =
      controller->computeVelocityCommands(pose({2, 2}, M_PI / 2), geometry_msgs::msg::Twist{});
  EXPECT_NEAR(cmd.twist.linear.x, 0, 1e-3);
  EXPECT_LT(cmd.twist.linear.y, -0.01);
}
TEST_F(NmpcTest, DiagonalSpeedIsLimitedInsideOptimization)
{
  configure();
  controller->setPlan(path({{2, 2}, {5, 5}}));
  geometry_msgs::msg::Twist measured;
  auto p = pose();
  for (int k = 0; k < 25; ++k)
  {
    geometry_msgs::msg::TwistStamped cmd;
    try
    {
      cmd = controller->computeVelocityCommands(p, measured);
    }
    catch (const std::exception &error)
    {
      FAIL() << "cycle=" << k << " " << error.what();
    }
    EXPECT_LE(std::hypot(cmd.twist.linear.x, cmd.twist.linear.y), 2.0003);
    EXPECT_LE(
        std::hypot(cmd.twist.linear.x - measured.linear.x, cmd.twist.linear.y - measured.linear.y),
        0.1003);
    controller->notifyPublished(cmd.twist, 0);
    measured = cmd.twist;
    p.pose.position.x += measured.linear.x * 0.05;
    p.pose.position.y += measured.linear.y * 0.05;
  }
}
TEST_F(NmpcTest, SinglePointGoalDoesNotThrowOrDivideByZero)
{
  configure();
  controller->setPlan(path({{2, 2}}));
  const auto cmd = controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{});
  EXPECT_LT(std::hypot(cmd.twist.linear.x, cmd.twist.linear.y), 0.01);
}
TEST_F(NmpcTest, OccupiedFootprintIsRejectedBeforeSolve)
{
  configure();
  controller->setPlan(path());
  map.setCost(40, 40, map_server::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}
TEST_F(NmpcTest, UnknownPolicyIsExplicit)
{
  configure({rclcpp::Parameter("MPC.allow_unknown", true)});
  controller->setPlan(path());
  map.setCost(40, 40, map_server::NO_INFORMATION);
  EXPECT_NO_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}));
  controller.reset();
  configure();
  controller->setPlan(path());
  EXPECT_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}
TEST_F(NmpcTest, GeneratedDimensionsCannotBeOverriddenAtRuntime)
{
  EXPECT_THROW(configure({rclcpp::Parameter("MPC.horizon", 10)}), std::invalid_argument);
}
TEST_F(NmpcTest, StaleSnapshotIsRejected)
{
  configure();
  controller->setPlan(path());
  auto snapshot = std::make_shared<map_server::CostmapSnapshot>();
  snapshot->grid = map;
  snapshot->current = false;
  controller->setSnapshot(snapshot);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}
TEST_F(NmpcTest, LatestMapRevalidationRejectsNewObstacle)
{
  configure();
  controller->setPlan(path());
  controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{});
  map_server::CostmapSnapshot snapshot;
  snapshot.current = true;
  snapshot.grid = map;
  snapshot.grid.setCost(40, 40, map_server::LETHAL_OBSTACLE);
  EXPECT_THROW(controller->validateLatestMap(snapshot), std::runtime_error);
}
TEST_F(NmpcTest, PublishedHistoryDoesNotReplaceMeasuredVelocity)
{
  configure();
  controller->setPlan(path());
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  controller->notifyPublished(command, 0);
  EXPECT_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}
TEST_F(NmpcTest, ResetRevokesPlanAndDetour)
{
  configure();
  controller->setPlan(path());
  controller->reset();
  EXPECT_FALSE(controller->hasFreshDetour());
  EXPECT_THROW(controller->computeVelocityCommands(pose(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}

TEST_F(NmpcTest, ReachesGoalWithBrakingInAnIdealClosedLoop)
{
  configure();
  controller->setPlan(path({{2, 2}, {3, 2}}));
  auto current = pose();
  geometry_msgs::msg::Twist measured;
  double maximum_solve = 0;
  bool stopped = false;
  for (int k = 0; k < 180; ++k)
  {
    geometry_msgs::msg::TwistStamped cmd;
    try
    {
      cmd = controller->computeVelocityCommands(current, measured);
    }
    catch (const std::exception &error)
    {
      FAIL() << "cycle=" << k << " " << error.what();
    }
    controller->notifyPublished(cmd.twist, 0);
    measured = cmd.twist;
    current.pose.position.x += measured.linear.x * 0.05;
    current.pose.position.y += measured.linear.y * 0.05;
    maximum_solve = std::max(maximum_solve, controller->feedback().solve_ms);
    if (std::hypot(current.pose.position.x - 3, current.pose.position.y - 2) < 0.10 &&
        std::hypot(measured.linear.x, measured.linear.y) < 0.05)
    {
      stopped = true;
      break;
    }
  }
  EXPECT_TRUE(stopped);
  RecordProperty("maximum_solve_ms", maximum_solve);
}
TEST_F(NmpcTest, AvoidsAStaticObstacleInAnIdealClosedLoop)
{
  configure();
  controller->setPlan(path({{2, 2}, {5, 2}}));
  // A compact obstacle requires a lateral detour, rather than stopping on the line.
  for (unsigned int y = 38; y <= 42; ++y)
  {
    for (unsigned int x = 58; x <= 62; ++x)
    {
      map.setCost(x, y, 0);
    }
  }
  auto current = pose();
  geometry_msgs::msg::Twist measured;
  double deviation = 0;
  bool passed = false;
  for (int k = 0; k < 180; ++k)
  {
    geometry_msgs::msg::TwistStamped cmd;
    try
    {
      cmd = controller->computeVelocityCommands(current, measured);
    }
    catch (const std::exception &error)
    {
      FAIL() << "cycle=" << k << " " << error.what();
    }
    controller->notifyPublished(cmd.twist, 0);
    measured = cmd.twist;
    const Vector before(current.pose.position.x, current.pose.position.y);
    current.pose.position.x += measured.linear.x * 0.05;
    current.pose.position.y += measured.linear.y * 0.05;
    const Vector after(current.pose.position.x, current.pose.position.y);
    ASSERT_TRUE(mpc::trajectoryClear(map, {before, after}, 0.20, false));
    deviation = std::max(deviation, std::abs(current.pose.position.y - 2));
    if (current.pose.position.x > 3.6)
    {
      passed = true;
      break;
    }
  }
  EXPECT_TRUE(passed);
  EXPECT_GT(deviation, 0.25);
}
TEST_F(NmpcTest, RuntimeTuningIsRejectedRatherThanSilentlyIgnored)
{
  configure();
  const auto result = node->set_parameters({rclcpp::Parameter("MPC.max_speed", 1.0)});
  ASSERT_EQ(result.size(), 1u);
  EXPECT_FALSE(result[0].successful);
}

TEST(Geometry, ConservativeCoverageIncludesEveryOccupiedCellCorner)
{
  map_server::Costmap2D map(100, 100, 0.05, 0, 0);
  for (unsigned int y = 2; y < 90; y += 4)
  {
    for (unsigned int x = 2; x < 90; x += 4)
    {
      map.setCost(x, y, 0);
    }
  }
  const auto circles = mpc::coverObstacles(map, false, 0.2, 128);
  ASSERT_LE(circles.size(), 128u);
  for (unsigned int y = 2; y < 90; y += 4)
  {
    for (unsigned int x = 2; x < 90; x += 4)
    {
      for (int dx : {0, 1})
      {
        for (int dy : {0, 1})
        {
          const Vector corner((x + dx) * 0.05, (y + dy) * 0.05);
          bool covered = false;
          for (const auto &circle : circles)
          {
            covered |= (corner - circle.center).norm() <= circle.radius - 0.2 + 1e-9;
          }
          EXPECT_TRUE(covered);
        }
      }
    }
  }
}
TEST(Geometry, SweepDetectsObstacleBetweenSparsePredictedPoints)
{
  map_server::Costmap2D map(100, 100, 0.05, 0, 0);
  map.setCost(50, 40, 0);
  EXPECT_FALSE(mpc::trajectoryClear(map, {{2, 2}, {3, 2}}, 0.2, false));
  EXPECT_FALSE(mpc::trajectoryClear(map, {{0.05, 2}, {1, 2}}, 0.2, false));
}
TEST(Geometry, SharpCornerHasBackwardBrakingEnvelope)
{
  const auto profile = mpc::makeSpeedProfile({{1, 1}, {2, 1}, {2, 2}}, 2, 1, 2, 2);
  ASSERT_EQ(profile.size(), 3u);
  EXPECT_DOUBLE_EQ(profile[1].speed, 0);
  EXPECT_DOUBLE_EQ(profile[2].speed, 0);
  const auto before = mpc::sampleReference(profile, 0.99);
  EXPECT_LE(before.speed, std::sqrt(2 * 2 * 0.01) + 1e-8);
}
TEST(Geometry, ProjectionDoesNotJumpToLaterCrossing)
{
  const auto profile = mpc::makeSpeedProfile({{1, 1}, {3, 3}, {1, 3}, {3, 1}}, 2, 1, 2, 2);
  EXPECT_LT(mpc::projectProgress(profile, {2, 2}, 0, 2), 2.0);
}
TEST_F(NmpcTest, LocalWaitingStillTriggersStallReplanning)
{
  nav_executor::RouteMonitorConfig config;
  config.maximum_sample_gap = 2;
  config.stall_duration = 1;
  nav_executor::RouteMonitor monitor(config);
  const auto now = nav_executor::TaskManager::Clock::now();
  EXPECT_FALSE(
      monitor.observe(path(), {1, 1}, pose(), geometry_msgs::msg::Twist{}, now, false, true));
  const auto event = monitor.observe(path(), {1, 1}, pose(), geometry_msgs::msg::Twist{},
                                     now + std::chrono::milliseconds(1100), false, true);
  ASSERT_TRUE(event);
  EXPECT_EQ(event->reason, nav_executor::ReplanReason::EXECUTION_STALLED);
}
} // namespace
