#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "map_server/cost_values.hpp"
#include "nav_executor/mpc_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <gtest/gtest.h>

class MpcControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void configure(bool allow_unknown = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("MPC.allow_unknown", allow_unknown)});
    node_ = std::make_shared<rclcpp::Node>("test_mpc_controller", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    controller_.initialize(node_, tf_, &map_, "odom", "base_link_fake", 0.15);
  }

  nav_msgs::msg::Path straightPath(const std::string &frame = "odom")
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    for (double x : {2.0, 4.0})
    {
      geometry_msgs::msg::PoseStamped point;
      point.header.frame_id = frame;
      point.pose.position.x = x;
      point.pose.position.y = 2.0;
      point.pose.orientation.w = 1.0;
      path.poses.push_back(point);
    }
    return path;
  }

  geometry_msgs::msg::PoseStamped robot(double yaw = 0.0)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 2.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
  }

  map_server::Costmap2D map_{200, 200, 0.05, 0.0, 0.0, map_server::FREE_SPACE};
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav_executor::MpcController controller_;
};

TEST_F(MpcControllerTest, StraightPathAcceleratesTowardGoal)
{
  configure();
  controller_.setPlan(straightPath());
  const auto cmd = controller_.computeVelocityCommands(robot(), geometry_msgs::msg::Twist{});
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_LE(cmd.twist.linear.x, 0.101);
  EXPECT_NEAR(cmd.twist.linear.y, 0.0, 1e-5);
  EXPECT_EQ(cmd.header.frame_id, "base_link_fake");
}

TEST_F(MpcControllerTest, MapPathTransformsAndVelocityUsesFakeBaseFrame)
{
  configure();
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "odom";
  map_to_odom.child_frame_id = "map";
  map_to_odom.transform.rotation.w = 1.0;
  tf_->setTransform(map_to_odom, "test", true);
  controller_.setPlan(straightPath("map"));
  const auto cmd =
      controller_.computeVelocityCommands(robot(M_PI / 2.0), geometry_msgs::msg::Twist{});
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, 1e-5);
  EXPECT_LT(cmd.twist.linear.y, 0.0);
}

TEST_F(MpcControllerTest, LethalCellUnderFootprintRejectsTrajectory)
{
  configure();
  map_.setCost(40, 40, map_server::LETHAL_OBSTACLE);
  controller_.setPlan(straightPath());
  EXPECT_THROW(controller_.computeVelocityCommands(robot(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}

TEST_F(MpcControllerTest, UnknownPolicyMatchesPlannerConfiguration)
{
  map_.setCost(40, 40, map_server::NO_INFORMATION);
  configure(true);
  controller_.setPlan(straightPath());
  EXPECT_NO_THROW(controller_.computeVelocityCommands(robot(), geometry_msgs::msg::Twist{}));
  controller_.reset();
  node_.reset();
  configure(false);
  controller_.setPlan(straightPath());
  EXPECT_THROW(controller_.computeVelocityCommands(robot(), geometry_msgs::msg::Twist{}),
               std::runtime_error);
}

namespace
{
using Point = Eigen::Vector2d;
nav_msgs::msg::Path makePath(const std::vector<Point> &points)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "odom";
  for (const auto &point : points)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.orientation.w = 1.0;
    result.poses.push_back(pose);
  }
  return result;
}
double distanceToPath(const Point &p, const std::vector<Point> &points)
{
  double best = INFINITY;
  for (size_t i = 1; i < points.size(); ++i)
  {
    const Point d = points[i] - points[i - 1];
    const double t = std::clamp((p - points[i - 1]).dot(d) / d.squaredNorm(), 0.0, 1.0);
    best = std::min(best, (p - points[i - 1] - t * d).norm());
  }
  return best;
}
} // namespace

TEST_F(MpcControllerTest, LaggedPlantStopsAccuratelyWithoutEarlyGoalCutoff)
{
  configure();
  controller_.setPlan(makePath({{2, 2}, {6, 2}}));
  Point p(2, 2), velocity(0, 0);
  double overshoot = 0, maximum_time = 0;
  bool reached = false;
  for (int k = 0; k < 500; ++k)
  {
    auto pose = robot();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = velocity.x();
    measured.linear.y = velocity.y();
    const auto began = std::chrono::steady_clock::now();
    const auto command = controller_.computeVelocityCommands(pose, measured);
    maximum_time =
        std::max(maximum_time,
                 std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
                     .count());
    const Point u(command.twist.linear.x, command.twist.linear.y);
    EXPECT_LE(u.norm(), 1.4001);
    // Slight model mismatch: actual response 0.25s vs predicted 0.20s.
    const double beta = std::exp(-0.05 / 0.25);
    p += 0.25 * (1 - beta) * velocity + (0.05 - 0.25 * (1 - beta)) * u;
    velocity = beta * velocity + (1 - beta) * u;
    overshoot = std::max(overshoot, p.x() - 6);
    if ((p - Point(6, 2)).norm() < 0.05 && velocity.norm() < 0.03)
    {
      reached = true;
      break;
    }
  }
  EXPECT_TRUE(reached);
  EXPECT_LT(overshoot, 0.08);
  RecordProperty("maximum_compute_ms", maximum_time);
  RecordProperty("maximum_overshoot_m", overshoot);
}

TEST_F(MpcControllerTest, RecoversFromLateralOffsetInsteadOfAdvancingReferenceAway)
{
  configure();
  controller_.setPlan(makePath({{2, 2.4}, {6, 2.4}}));
  Point p(2, 2), velocity(0, 0);
  bool recovered = false;
  for (int k = 0; k < 100; ++k)
  {
    auto pose = robot();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = velocity.x();
    measured.linear.y = velocity.y();
    const auto command = controller_.computeVelocityCommands(pose, measured);
    const Point u(command.twist.linear.x, command.twist.linear.y);
    const double beta = std::exp(-0.05 / 0.25);
    p += 0.25 * (1 - beta) * velocity + (0.05 - 0.25 * (1 - beta)) * u;
    velocity = beta * velocity + (1 - beta) * u;
    if (std::abs(p.y() - 2.4) < 0.08)
    {
      recovered = true;
      break;
    }
  }
  EXPECT_TRUE(recovered);
}

TEST_F(MpcControllerTest, TightHairpinTracksWithLagAndBrakesBeforeTurn)
{
  configure();
  std::vector<Point> points{{2, 2}, {3, 2}, {4, 2}};
  for (int i = 1; i <= 60; ++i)
  {
    const double angle = -M_PI / 2 + i * M_PI / 60;
    points.emplace_back(4 + 0.65 * std::cos(angle), 2.65 + 0.65 * std::sin(angle));
  }
  points.emplace_back(3, 3.3);
  points.emplace_back(2, 3.3);
  controller_.setPlan(makePath(points));
  Point p(2, 2), velocity(0, 0);
  double maximum_error = 0, speed_in_turn = 0;
  bool reached = false;
  for (int k = 0; k < 800; ++k)
  {
    auto pose = robot();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = velocity.x();
    measured.linear.y = velocity.y();
    const auto command = controller_.computeVelocityCommands(pose, measured);
    const Point u(command.twist.linear.x, command.twist.linear.y);
    const double beta = std::exp(-0.05 / 0.25);
    p += 0.25 * (1 - beta) * velocity + (0.05 - 0.25 * (1 - beta)) * u;
    velocity = beta * velocity + (1 - beta) * u;
    maximum_error = std::max(maximum_error, distanceToPath(p, points));
    if (p.x() > 4.1)
    {
      speed_in_turn = std::max(speed_in_turn, velocity.norm());
    }
    if ((p - points.back()).norm() < 0.05 && velocity.norm() < 0.03)
    {
      reached = true;
      break;
    }
  }
  EXPECT_TRUE(reached);
  EXPECT_LT(maximum_error, 0.20);
  EXPECT_LT(speed_in_turn, 0.85);
  RecordProperty("maximum_cross_track_error_m", maximum_error);
  RecordProperty("maximum_turn_speed_mps", speed_in_turn);
}

TEST_F(MpcControllerTest, OvershotGoalProducesReverseCorrectionAndReturns)
{
  configure();
  controller_.setPlan(makePath({{2, 2}, {3, 2}}));
  auto pose = robot();
  pose.pose.position.x = 3.15;
  Point p(3.15, 2), velocity(0.25, 0);
  bool reversed = false, reached = false;
  // A new plan starts at the current location; retain a nearby segment ending at the overshot goal.
  controller_.setPlan(makePath({{2.8, 2}, {3, 2}}));
  for (int k = 0; k < 300; ++k)
  {
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = velocity.x();
    measured.linear.y = velocity.y();
    const auto command = controller_.computeVelocityCommands(pose, measured);
    const Point u(command.twist.linear.x, command.twist.linear.y);
    reversed |= u.x() < -0.01;
    const double beta = std::exp(-0.05 / 0.25);
    p += 0.25 * (1 - beta) * velocity + (0.05 - 0.25 * (1 - beta)) * u;
    velocity = beta * velocity + (1 - beta) * u;
    if ((p - Point(3, 2)).norm() < 0.05 && velocity.norm() < 0.03)
    {
      reached = true;
      break;
    }
  }
  EXPECT_TRUE(reversed);
  EXPECT_TRUE(reached);
}

TEST_F(MpcControllerTest, MovingHandoffOutsideInscribedPolygonRemainsFeasible)
{
  configure();
  controller_.setPlan(makePath({{2, 2}, {6, 2}}));
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 1.4 * std::cos(M_PI / 8);
  measured.linear.y = 1.4 * std::sin(M_PI / 8);
  const auto command = controller_.computeVelocityCommands(robot(), measured);
  EXPECT_LE(std::hypot(command.twist.linear.x, command.twist.linear.y), 1.4001);
}
