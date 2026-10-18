// Costmap -> tdt YAstar -> MinimumSnap -> nav_msgs::Path.
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "map_server/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"



namespace topology_global_planner::grid
{

class GridPlanner
{
public:
  struct Options
  {
    double tolerance{0.2};
    double cost_field_radius{0.6};
    double cost_weight{1.0};
    bool smooth{true};
    std::string smoothing_backend{"osqp_corridor"};
    double smoothing_dt{0.1};
    double smoothing_max_speed{3.0};
    double smoothing_max_acc{1.0};
    double smoothing_time_limit{10.0};
    int smoothing_iterations{6};
    double smoothing_corridor_range{2.5};
    bool allow_unknown{true};
    bool use_final_approach_orientation{true};
  };

  struct SmoothingReport
  {
    bool attempted{false};
    bool accepted{false};
    std::string backend;
    std::string reason{"disabled or single point"};
    size_t control_points{0};
    size_t output_points{0};
  };
  const SmoothingReport & lastSmoothingReport() const {return smoothing_report_;}

  GridPlanner(
    map_server::Costmap2D * costmap,
    const std::string & global_frame,
    rclcpp::Clock::SharedPtr clock,
    Options options);

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker, bool exact_goal = false);

private:
  map_server::Costmap2D * costmap_;
  std::string global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  Options options_;
  SmoothingReport smoothing_report_;


};

}  // namespace topology_global_planner::grid