#include "nav_executor/mpc_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

using Point = Eigen::Vector2d;
class TrackingProbe : public rclcpp::Node
{
public:
  TrackingProbe() : Node("nav_executor") {}
  void initialize()
  {
    scenario_ = declare_parameter("scenario", std::string("straight"));
    output_ = declare_parameter("result_file", std::string("/tmp/swerve-tracking-result.json"));
    tolerance_ = declare_parameter("xy_goal_tolerance", 0.05);
    stop_speed_ = declare_parameter("stopped_velocity_tolerance", 0.03);
    // The unused executor settings in config/mpc.yaml are intentionally ignored here.
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    controller_.initialize(shared_from_this(), tf_, &map_, "odom", "base_link", 0.2);
    command_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { observe(*msg); });
  }
  bool done() const { return done_; }
  bool passed() const { return passed_; }
  void timeout() { finish(false, "wall timeout"); }

private:
  double crossTrack(const Point &p) const
  {
    double best = INFINITY;
    for (size_t i = 1; i < path_.size(); ++i)
    {
      const Point delta = path_[i] - path_[i - 1];
      const double ratio =
          std::clamp((p - path_[i - 1]).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
      best = std::min(best, (p - path_[i - 1] - ratio * delta).norm());
    }
    return best;
  }
  void setPath(const std::vector<Point> &points)
  {
    path_ = points;
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    for (const auto &p : points)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    controller_.setPlan(path);
  }
  void observe(const nav_msgs::msg::Odometry &msg)
  {
    if (done_)
    {
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (stamp < 2 || stamp - last_control_ < 0.0499)
    {
      return;
    }
    last_control_ = stamp;
    const Point p(msg.pose.pose.position.x, msg.pose.pose.position.y);
    const double speed = std::hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    if (!started_)
    {
      anchor_ = p;
      start_time_ = stamp;
      started_ = true;
      if (scenario_ == "hairpin")
      {
        std::vector<Point> points{p, p + Point(1, 0), p + Point(2, 0)};
        for (int i = 1; i <= 60; ++i)
        {
          const double angle = -M_PI / 2 + i * M_PI / 60;
          points.push_back(p + Point(2 + 0.65 * std::cos(angle), 0.65 + 0.65 * std::sin(angle)));
        }
        points.push_back(p + Point(1, 1.3));
        points.push_back(p + Point(0, 1.3));
        setPath(points);
      }
      else if (scenario_ == "recovery")
      {
        setPath({p + Point(0, 0.4), p + Point(4, 0.4)});
      }
      else
      {
        setPath({p, p + Point(4, 0)});
      }
    }
    if (scenario_ == "overshoot" && !short_goal_ && p.x() - anchor_.x() > 1.5 && speed > 0.8)
    {
      setPath({p - Point(0.2, 0), p + Point(0.1, 0)});
      short_goal_ = true;
      RCLCPP_INFO(get_logger(), "Injected nearby goal while moving %.3fm/s", speed);
    }
    final_error_ = (p - path_.back()).norm();
    final_speed_ = speed;
    maximum_error_ = std::max(maximum_error_, crossTrack(p));
    maximum_x_ = std::max(maximum_x_, p.x() - anchor_.x());
    maximum_overshoot_ = std::max(maximum_overshoot_, p.x() - path_.back().x());
    if (scenario_ == "recovery" && std::abs(p.y() - path_.front().y()) < 0.08 && recovery_time_ < 0)
    {
      recovery_time_ = stamp - start_time_;
    }
    if (scenario_ == "hairpin" && p.x() > anchor_.x() + 2.1)
    {
      turn_speed_ = std::max(turn_speed_, speed);
    }
    if (final_error_ < tolerance_ && speed <= stop_speed_ &&
        (scenario_ != "overshoot" || short_goal_))
    {
      command_->publish(geometry_msgs::msg::Twist{});
      if (hold_since_ < 0)
      {
        hold_since_ = stamp;
      }
      if (stamp - hold_since_ >= 0.3)
      {
        bool success = true;
        if (scenario_ == "hairpin")
        {
          success = maximum_error_ < 0.25 && maximum_x_ > 2.4 && turn_speed_ < 0.95;
        }
        if (scenario_ == "straight")
        {
          success = maximum_overshoot_ < 0.10;
        }
        if (scenario_ == "recovery")
        {
          success = recovery_time_ >= 0 && recovery_time_ < 4.0;
        }
        if (scenario_ == "overshoot")
        {
          success = reversed_;
        }
        finish(success, success ? "passed" : "metric threshold");
      }
      return;
    }
    hold_since_ = -1;
    if (stamp - start_time_ > 45)
    {
      finish(false, "simulation timeout");
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose = msg.pose.pose;
    try
    {
      const auto began = std::chrono::steady_clock::now();
      const auto result = controller_.computeVelocityCommands(pose, msg.twist.twist);
      max_compute_ms_ = std::max(max_compute_ms_, std::chrono::duration<double, std::milli>(
                                                      std::chrono::steady_clock::now() - began)
                                                      .count());
      const double yaw = tf2::getYaw(pose.pose.orientation);
      const double world_x =
          std::cos(yaw) * result.twist.linear.x - std::sin(yaw) * result.twist.linear.y;
      if (short_goal_ && world_x < -0.01)
      {
        reversed_ = true;
      }
      command_->publish(result.twist);
      ++commands_;
    }
    catch (const std::exception &error)
    {
      finish(false, error.what());
    }
  }
  void finish(bool success, const std::string &reason)
  {
    if (done_)
    {
      return;
    }
    done_ = true;
    passed_ = success;
    command_->publish(geometry_msgs::msg::Twist{});
    std::ofstream output(output_);
    output << "{\"scenario\":\"" << scenario_ << "\",\"success\":" << (success ? "true" : "false")
           << ",\"reason\":\"" << reason << "\",\"final_error_m\":" << final_error_
           << ",\"final_speed_mps\":" << final_speed_
           << ",\"maximum_cross_track_m\":" << maximum_error_
           << ",\"maximum_overshoot_m\":" << maximum_overshoot_
           << ",\"maximum_compute_ms\":" << max_compute_ms_
           << ",\"maximum_turn_speed_mps\":" << turn_speed_
           << ",\"recovery_time_s\":" << recovery_time_ << ",\"commands\":" << commands_
           << ",\"reversed\":" << (reversed_ ? "true" : "false") << "}\n";
    RCLCPP_INFO(get_logger(), "%s: %s, error %.4fm speed %.4fm/s max_compute %.2fms",
                scenario_.c_str(), reason.c_str(), final_error_, final_speed_, max_compute_ms_);
  }
  nav_executor::MpcController controller_;
  map_server::Costmap2D map_{600, 600, 0.05, -10, -10};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  std::string scenario_, output_;
  std::vector<Point> path_;
  Point anchor_{0, 0};
  bool started_{false}, done_{false}, passed_{false}, short_goal_{false}, reversed_{false};
  double last_control_{0}, start_time_{0}, hold_since_{-1}, tolerance_{0.05}, stop_speed_{0.03};
  double final_error_{0}, final_speed_{0}, maximum_error_{0}, maximum_x_{0}, maximum_overshoot_{0};
  double max_compute_ms_{0}, turn_speed_{0}, recovery_time_{-1};
  int commands_{0};
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackingProbe>();
  node->initialize();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  while (rclcpp::ok() && !node->done() && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!node->done())
  {
    node->timeout();
  }
  const bool passed = node->passed();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
