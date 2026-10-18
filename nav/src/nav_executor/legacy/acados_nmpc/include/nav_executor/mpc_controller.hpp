#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "map_server/costmap_2d.hpp"
#include "map_server/costmap_client.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace nav_executor
{
struct MpcFeedback
{
  bool valid{false}, detouring{false}, converged{false}, restored{false};
  int solver_status{-1};
  double progress{0.0}, solve_ms{0.0}, total_ms{0.0};
  uint64_t map_version{0};
};

// The only active backend is the generated Acados discrete omnidirectional NMPC.
class MpcController
{
public:
  MpcController();
  ~MpcController();
  MpcController(const MpcController &) = delete;
  MpcController &operator=(const MpcController &) = delete;
  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<tf2_ros::Buffer> tf,
                  map_server::Costmap2D *costmap, std::string global_frame, std::string base_frame,
                  double robot_radius);
  void setPlan(const nav_msgs::msg::Path &path);
  void reset();
  void setSnapshot(std::shared_ptr<const map_server::CostmapSnapshot> snapshot);
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &measured_speed);
  // Called by the sole publisher after the actual publish operation.
  void notifyPublished(const geometry_msgs::msg::Twist &command, double frame_yaw);
  void validateLatestMap(const map_server::CostmapSnapshot &snapshot) const;
  const MpcFeedback &feedback() const;
  bool hasFreshDetour() const;
  bool hasFreshPlan() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace nav_executor
