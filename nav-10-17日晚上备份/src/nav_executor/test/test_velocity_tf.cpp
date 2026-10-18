#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "nav_executor/path_executor.hpp"
#include "tf2/exceptions.h"

class VelocityTfTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options; options.auto_initialize_logging(false);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  nav_msgs::msg::Odometry odometry() {
    nav_msgs::msg::Odometry m;
    m.header.stamp = rclcpp::Time(10, 0);
    m.child_frame_id = "aft_mapped"; m.twist.twist.linear.x = 1;
    return m;
  }
  geometry_msgs::msg::TransformStamped transform(int nanosec) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_link_fake"; t.child_frame_id = "aft_mapped";
    t.header.stamp = rclcpp::Time(9, nanosec); t.transform.rotation.w = 1;
    return t;
  }
};
TEST_F(VelocityTfTest, WaitsForTheExactOdometryStampWhenTfArrivesLater) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>());
  buffer.setUsingDedicatedThread(true);
  buffer.setTransform(transform(990000000), "test");
  std::atomic<bool> published{false};
  std::thread broadcaster([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto next = transform(0); next.header.stamp = rclcpp::Time(10, 0);
    buffer.setTransform(next, "test"); published = true;
  });
  try {
    const auto speed = nav_executor::transformOdometryVelocity(
      buffer, "base_link_fake", odometry(), .1);
    EXPECT_TRUE(published); EXPECT_DOUBLE_EQ(speed.linear.x, 1);
  } catch (...) {broadcaster.join(); throw;}
  broadcaster.join();
}
TEST_F(VelocityTfTest, AStaleTransformStillFailsInsteadOfUsingLatest) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>());
  buffer.setUsingDedicatedThread(true);
  buffer.setTransform(transform(0), "test");
  EXPECT_THROW(nav_executor::transformOdometryVelocity(
    buffer, "base_link_fake", odometry(), .02), tf2::TransformException);
}
TEST_F(VelocityTfTest, SameFrameNeedsNoTransform) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>());
  auto m = odometry(); m.child_frame_id = "base_link_fake";
  EXPECT_DOUBLE_EQ(nav_executor::transformOdometryVelocity(
    buffer, "base_link_fake", m, .05).linear.x, 1);
}
