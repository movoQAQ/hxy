#include "nav_executor/safety_monitor.hpp"
#include <cmath>
#include <stdexcept>
namespace nav_executor {
SafetyMonitor::SafetyMonitor(SafetyMonitorConfig config) : config_(config) {
  for (double v : {config.stall_duration, config.minimum_command_speed,
                   config.minimum_displacement, config.maximum_measured_speed,
                   config.maximum_sample_gap})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid physical stall threshold");
}
void SafetyMonitor::reset() {
  since_.reset();
  previous_.reset();
}
bool SafetyMonitor::observe(const geometry_msgs::msg::PoseStamped &pose,
                            const geometry_msgs::msg::Twist &measured,
                            const geometry_msgs::msg::Twist &command,
                            TaskManager::Clock::time_point now) {
  const auto &p = pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(measured.linear.x) || !std::isfinite(measured.linear.y) ||
      !std::isfinite(command.linear.x) || !std::isfinite(command.linear.y)) {
    reset();
    return false;
  }
  if (previous_ && (now <= *previous_ ||
                    std::chrono::duration<double>(now - *previous_).count() >
                        config_.maximum_sample_gap))
    reset();
  previous_ = now;
  if (std::hypot(command.linear.x, command.linear.y) <
          config_.minimum_command_speed ||
      std::hypot(measured.linear.x, measured.linear.y) >
          config_.maximum_measured_speed) {
    since_.reset();
    return false;
  }
  if (!since_ ||
      std::hypot(p.x - x_, p.y - y_) >= config_.minimum_displacement) {
    since_ = now;
    x_ = p.x;
    y_ = p.y;
  }
  return std::chrono::duration<double>(now - *since_).count() >=
         config_.stall_duration;
}
} // namespace nav_executor
