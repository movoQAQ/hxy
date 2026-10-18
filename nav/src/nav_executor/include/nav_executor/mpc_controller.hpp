#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "map_server/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <Eigen/Core>

namespace nav_executor
{
// Tracks tdt's geometric nav_msgs/Path. Odometry supplies measured velocity;
// speed references are generated from path arclength, not inferred timestamps.
class MpcController
{
public:
  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<tf2_ros::Buffer> tf,
                  map_server::Costmap2D *costmap, std::string global_frame, std::string base_frame,
                  double robot_radius);
  void setPlan(const nav_msgs::msg::Path &path);
  void reset();
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &measured_speed);

private:
  struct Sample
  {
    Eigen::Vector2d point;
    Eigen::Vector2d tangent;
    double speed;
  };
  struct PathPoint
  {
    Eigen::Vector2d point;
    double arc;
    double speed{0.0};
  };
  void buildSpeedProfile(std::vector<PathPoint> &path) const;
  Sample sample(double arc, const std::vector<PathPoint> &path) const;
  std::vector<PathPoint> transformPlan(const geometry_msgs::msg::PoseStamped &pose) const;
  bool rolloutClear(const std::vector<Eigen::Vector2d> &predicted) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  map_server::Costmap2D *costmap_{nullptr};
  nav_msgs::msg::Path plan_;
  std::string global_frame_, base_frame_;
  bool allow_unknown_{false};
  double robot_radius_{0.15};
  bool have_last_command_{false};
  double progress_{0.0};
  double dt_{0.05}, max_speed_{1.4}, max_acc_{0.8}, max_lateral_acc_{0.6};
  double max_brake_{0.8}, response_time_{0.2}, direction_rate_{1.2};
  double curvature_window_{0.2}, recovery_distance_{0.3}, recovery_gain_{1.5};
  double max_recovery_speed_{0.6}, goal_speed_gain_{1.2}, terminal_weight_{5.0};
  double r_delta_{0.3}, projection_backtrack_{0.5};
  double q_along_{5.0}, q_cross_{20.0}, r_speed_{1.5}, delay_{0.08};
  int horizon_{30}, max_working_set_recalculations_{500};
  Eigen::Vector2d last_command_{0.0, 0.0};
};
} // namespace nav_executor
