#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "map_server/costmap_2d.hpp"
#include <optional>
namespace nav_executor {
struct RecoveryConfig {
  double distance{0.35}, search_radius{1.0}, speed{0.15}, timeout{6.0},
      acceleration{0.3};
  double margin{0.03}, angular_speed{0.5}, angular_acceleration{0.5},
      stop_speed{0.05};
};
// Uses a circumscribed disc: the complete footprint is contained at every yaw.
// Unknown space, out-of-map space and intersecting lethal cells are forbidden.
bool sweptFootprintSafe(const map_server::Costmap2D &grid, double ax, double ay,
                        double bx, double by, double radius);
struct RecoveryTarget {
  double x, y;
};
class RecoveryController {
public:
  explicit RecoveryController(RecoveryConfig config = {});
  std::optional<RecoveryTarget>
  select(const map_server::Costmap2D &grid,
         const geometry_msgs::msg::PoseStamped &pose, double radius,
         bool hazard) const;
  const RecoveryConfig &config() const { return config_; }

private:
  RecoveryConfig config_;
};
} // namespace nav_executor
