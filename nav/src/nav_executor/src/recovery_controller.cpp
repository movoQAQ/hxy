#include "nav_executor/recovery_controller.hpp"
#include "map_server/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace nav_executor {
bool sweptFootprintSafe(const map_server::Costmap2D &g, double ax, double ay,
                        double bx, double by, double radius) {
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) ||
      !std::isfinite(by) || !std::isfinite(radius) || radius <= 0 ||
      g.getSizeInCellsX() == 0 || g.getResolution() <= 0)
    return false;
  const double r = radius + g.getResolution() * std::sqrt(0.5);
  const double minx = std::min(ax, bx) - r, maxx = std::max(ax, bx) + r;
  const double miny = std::min(ay, by) - r, maxy = std::max(ay, by) + r;
  unsigned int x0, y0, x1, y1;
  if (!g.worldToMap(minx, miny, x0, y0) || !g.worldToMap(maxx, maxy, x1, y1))
    return false;
  const double dx = bx - ax, dy = by - ay, length2 = dx * dx + dy * dy;
  for (unsigned int y = y0; y <= y1; ++y)
    for (unsigned int x = x0; x <= x1; ++x) {
      const auto c = g.getCost(x, y);
      if (c != map_server::LETHAL_OBSTACLE && c != map_server::NO_INFORMATION)
        continue;
      double wx, wy;
      g.mapToWorld(x, y, wx, wy);
      const double t =
          length2 > 1e-12
              ? std::clamp(((wx - ax) * dx + (wy - ay) * dy) / length2, 0., 1.)
              : 0.;
      if (std::hypot(wx - ax - t * dx, wy - ay - t * dy) <= r)
        return false;
    }
  return true;
}
RecoveryController::RecoveryController(RecoveryConfig config)
    : config_(config) {
  for (double v :
       {config.distance, config.search_radius, config.speed, config.timeout,
        config.acceleration, config.margin, config.angular_speed,
        config.angular_acceleration, config.stop_speed})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid recovery/spin limits");
  if (config.distance > config.search_radius || config.speed > 0.5 ||
      config.search_radius > 3.0)
    throw std::invalid_argument(
        "Recovery must be a bounded, low-speed local motion");
}
std::optional<RecoveryTarget>
RecoveryController::select(const map_server::Costmap2D &g,
                           const geometry_msgs::msg::PoseStamped &pose,
                           double radius, bool hazard) const {
  const auto &p = pose.pose.position;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  std::optional<RecoveryTarget> best;
  double best_score = -INFINITY;
  // Reverse/sideways directions are preferred for a stuck omnidirectional
  // chassis.
  const double offsets[] = {M_PI,          M_PI / 2, -M_PI / 2, 3 * M_PI / 4,
                            -3 * M_PI / 4, 0,        M_PI / 4,  -M_PI / 4};
  for (double distance = config_.distance;
       distance <= (hazard ? config_.search_radius : config_.distance) + 1e-6;
       distance += 0.15)
    for (double a : offsets) {
      const double x = p.x + distance * std::cos(yaw + a),
                   y = p.y + distance * std::sin(yaw + a);
      if (!sweptFootprintSafe(g, p.x, p.y, x, y, radius + config_.margin))
        continue;
      unsigned int mx, my;
      if (!g.worldToMap(x, y, mx, my))
        continue;
      // Larger tdt values mean more clearance. Prefer short maneuvers at equal
      // clearance.
      const double score =
          static_cast<double>(g.getCost(mx, my)) - 5 * distance;
      if (score > best_score) {
        best_score = score;
        best = RecoveryTarget{x, y};
      }
    }
  return best;
}
} // namespace nav_executor
