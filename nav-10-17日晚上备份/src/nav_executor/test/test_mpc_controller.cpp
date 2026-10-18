#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include "map_server/cost_values.hpp"
#include "nav_executor/mpc_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MpcControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions options;
    options.auto_initialize_logging(false);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void configure(bool allow_unknown = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("MPC.allow_unknown", allow_unknown)});
    node_ = std::make_shared<rclcpp::Node>("test_mpc_controller", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    controller_.initialize(node_, tf_, &map_, "odom", "base_link_fake", 0.15);
  }

  nav_msgs::msg::Path straightPath(const std::string & frame = "odom")
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    for (double x : {2.0, 4.0}) {
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
  const auto cmd = controller_.computeVelocityCommands(
    robot(M_PI / 2.0), geometry_msgs::msg::Twist{});
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
