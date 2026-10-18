#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "nav_executor/task_manager.hpp"
#include <optional>
namespace nav_executor {
struct SafetyMonitorConfig {
  double stall_duration{3.0}, minimum_command_speed{0.1},
      minimum_displacement{0.08};
  double maximum_measured_speed{0.05}, maximum_sample_gap{0.3};
};
// Physical non-movement only; route deviation and blockage belong to
// RouteMonitor.
class SafetyMonitor {
public:
  explicit SafetyMonitor(SafetyMonitorConfig config = {});
  bool observe(const geometry_msgs::msg::PoseStamped &pose,
               const geometry_msgs::msg::Twist &measured,
               const geometry_msgs::msg::Twist &command,
               TaskManager::Clock::time_point now);
  void reset();

private:
  SafetyMonitorConfig config_;
  std::optional<TaskManager::Clock::time_point> since_, previous_;
  double x_{0}, y_{0};
};
} // namespace nav_executor
