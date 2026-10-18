#include "nav_executor/mpc/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav_executor::mpc
{
Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d &value)
{
  return {std::cos(yaw) * value.x() - std::sin(yaw) * value.y(),
          std::sin(yaw) * value.x() + std::cos(yaw) * value.y()};
}

std::vector<Circle> coverObstacles(const map_server::Costmap2D &map, bool allow_unknown,
                                   double robot_radius, int capacity)
{
  if (capacity < 1)
  {
    throw std::invalid_argument("OBSTACLE_OVERFLOW: invalid capacity");
  }
  // Merge occupied cell squares into conservative block circles. Every occupied
  // cell is retained; larger blocks trade passage width for a bounded NLP size.
  for (unsigned int block = std::min(2u, std::max(map.getSizeInCellsX(), map.getSizeInCellsY()));
       block > 0 && block <= std::max(map.getSizeInCellsX(), map.getSizeInCellsY()); block *= 2)
  {
    std::vector<Circle> circles;
    for (unsigned int by = 0; by < map.getSizeInCellsY(); by += block)
    {
      for (unsigned int bx = 0; bx < map.getSizeInCellsX(); bx += block)
      {
        unsigned int minx = map.getSizeInCellsX(), miny = map.getSizeInCellsY(), maxx = 0, maxy = 0;
        bool occupied = false;
        for (unsigned int y = by; y < std::min(by + block, map.getSizeInCellsY()); ++y)
        {
          for (unsigned int x = bx; x < std::min(bx + block, map.getSizeInCellsX()); ++x)
          {
            const auto cost = map.getCost(x, y);
            if (cost == map_server::LETHAL_OBSTACLE ||
                (!allow_unknown && cost == map_server::NO_INFORMATION))
            {
              occupied = true;
              minx = std::min(minx, x);
              maxx = std::max(maxx, x);
              miny = std::min(miny, y);
              maxy = std::max(maxy, y);
            }
          }
        }
        if (occupied)
        {
          const double res = map.getResolution();
          const Eigen::Vector2d center(map.getOriginX() + 0.5 * (minx + maxx + 1) * res,
                                       map.getOriginY() + 0.5 * (miny + maxy + 1) * res);
          circles.push_back(
              {center, 0.5 * res * std::hypot(maxx - minx + 1, maxy - miny + 1) + robot_radius});
        }
      }
    }
    if (static_cast<int>(circles.size()) <= capacity)
    {
      return circles;
    }
    if (block > std::max(map.getSizeInCellsX(), map.getSizeInCellsY()) / 2)
    {
      break;
    }
  }
  throw std::runtime_error("OBSTACLE_OVERFLOW: complete coverage exceeds capacity");
}

bool trajectoryClear(const map_server::Costmap2D &map, const std::vector<Eigen::Vector2d> &points,
                     double radius, bool allow_unknown)
{
  if (map.getSizeInCellsX() < 3 || points.empty() || radius <= 0)
  {
    return false;
  }
  const double res = map.getResolution();
  const double xmin = map.getOriginX() + radius, ymin = map.getOriginY() + radius;
  const double xmax = map.getOriginX() + map.getSizeInCellsX() * res - radius;
  const double ymax = map.getOriginY() + map.getSizeInCellsY() * res - radius;
  auto clear = [&](const Eigen::Vector2d &p)
  {
    if (!p.allFinite() || p.x() < xmin || p.x() > xmax || p.y() < ymin || p.y() > ymax)
    {
      return false;
    }
    unsigned int mx, my;
    if (!map.worldToMap(p.x(), p.y(), mx, my))
    {
      return false;
    }
    const auto center = map.getCost(mx, my);
    if (center == map_server::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
    const int cells = static_cast<int>(std::ceil(radius / res)) + 1;
    for (int y = std::max(0, static_cast<int>(my) - cells);
         y <= std::min(static_cast<int>(map.getSizeInCellsY()) - 1, static_cast<int>(my) + cells);
         ++y)
    {
      for (int x = std::max(0, static_cast<int>(mx) - cells);
           x <= std::min(static_cast<int>(map.getSizeInCellsX()) - 1, static_cast<int>(mx) + cells);
           ++x)
      {
        const auto cost = map.getCost(x, y);
        if (cost != map_server::LETHAL_OBSTACLE &&
            (allow_unknown || cost != map_server::NO_INFORMATION))
        {
          continue;
        }
        // Exact circle versus closed cell square, not a center-only check.
        const double dx = std::max(
            {map.getOriginX() + x * res - p.x(), 0.0, p.x() - (map.getOriginX() + (x + 1) * res)});
        const double dy = std::max(
            {map.getOriginY() + y * res - p.y(), 0.0, p.y() - (map.getOriginY() + (y + 1) * res)});
        if (dx * dx + dy * dy <= radius * radius)
        {
          return false;
        }
      }
    }
    return true;
  };
  if (!clear(points.front()))
  {
    return false;
  }
  for (size_t i = 1; i < points.size(); ++i)
  {
    if (!points[i].allFinite())
    {
      return false;
    }
    const int steps =
        std::max(1, static_cast<int>(std::ceil((points[i] - points[i - 1]).norm() / (res * 0.5))));
    // The controller adds a half-sample margin to cover the continuous sweep.
    for (int j = 1; j <= steps; ++j)
    {
      if (!clear(points[i - 1] + (points[i] - points[i - 1]) * (static_cast<double>(j) / steps)))
      {
        return false;
      }
    }
  }
  return true;
}

std::vector<PathPoint> makeSpeedProfile(const std::vector<Eigen::Vector2d> &points,
                                        double max_speed, double lateral_acc, double accel,
                                        double brake)
{
  std::vector<PathPoint> path;
  for (const auto &point : points)
  {
    if (!point.allFinite())
    {
      throw std::invalid_argument("NO_REFERENCE: nonfinite path");
    }
    if (!path.empty() && (point - path.back().position).norm() < 1e-5)
    {
      continue;
    }
    const double arc = path.empty() ? 0 : path.back().arc + (point - path.back().position).norm();
    path.push_back({point, arc, max_speed});
  }
  if (path.empty())
  {
    throw std::runtime_error("NO_REFERENCE: empty path");
  }
  for (size_t i = 1; i + 1 < path.size(); ++i)
  {
    const Eigen::Vector2d before = path[i].position - path[i - 1].position;
    const Eigen::Vector2d after = path[i + 1].position - path[i].position;
    const double angle =
        std::abs(std::atan2(before.x() * after.y() - before.y() * after.x(), before.dot(after)));
    const double curvature = angle / std::max(0.02, 0.5 * (before.norm() + after.norm()));
    path[i].speed = std::min(max_speed, std::sqrt(lateral_acc / std::max(1e-6, curvature)));
    // A geometric discontinuity cannot be taken at finite speed exactly.
    if (angle > 0.7)
    {
      path[i].speed = 0.0;
    }
  }
  path.back().speed = 0.0;
  for (size_t i = path.size() - 1; i > 0; --i)
  {
    path[i - 1].speed =
        std::min(path[i - 1].speed, std::sqrt(path[i].speed * path[i].speed +
                                              2 * brake * (path[i].arc - path[i - 1].arc)));
  }
  for (size_t i = 1; i < path.size(); ++i)
  {
    path[i].speed = std::min(path[i].speed, std::sqrt(path[i - 1].speed * path[i - 1].speed +
                                                      2 * accel * (path[i].arc - path[i - 1].arc)));
  }
  return path;
}

Reference sampleReference(const std::vector<PathPoint> &path, double arc)
{
  if (path.size() == 1)
  {
    return {path.front().position, Eigen::Vector2d(1, 0), 0, 0};
  }
  arc = std::clamp(arc, 0.0, path.back().arc);
  auto upper = std::upper_bound(path.begin(), path.end(), arc,
                                [](double a, const PathPoint &b) { return a < b.arc; });
  const size_t i = std::clamp<size_t>(upper - path.begin(), 1, path.size() - 1);
  const double length = path[i].arc - path[i - 1].arc, ratio = (arc - path[i - 1].arc) / length;
  const Eigen::Vector2d tangent = (path[i].position - path[i - 1].position) / length;
  // Interpolate v^2 to preserve the braking envelope along a segment.
  const double speed = std::sqrt(std::max(0.0, (1 - ratio) * std::pow(path[i - 1].speed, 2) +
                                                   ratio * std::pow(path[i].speed, 2)));
  return {path[i - 1].position + tangent * (arc - path[i - 1].arc), tangent, speed, arc};
}

double projectProgress(const std::vector<PathPoint> &path, const Eigen::Vector2d &p,
                       double previous, double search_distance)
{
  if (path.size() == 1)
  {
    return 0;
  }
  double best = std::numeric_limits<double>::infinity(), arc = previous;
  for (size_t i = 1; i < path.size(); ++i)
  {
    const Eigen::Vector2d d = path[i].position - path[i - 1].position;
    const double t = std::clamp((p - path[i - 1].position).dot(d) / d.squaredNorm(), 0.0, 1.0);
    const double candidate = path[i - 1].arc + t * d.norm();
    if (candidate < previous - 0.1 || candidate > previous + search_distance)
    {
      continue;
    }
    const double dist = (p - path[i - 1].position - t * d).squaredNorm();
    if (dist < best)
    {
      best = dist;
      arc = candidate;
    }
  }
  return std::max(previous, arc);
}

double pathDistance(const std::vector<PathPoint> &path, const Eigen::Vector2d &p)
{
  double best = (p - path.front().position).norm();
  for (size_t i = 1; i < path.size(); ++i)
  {
    const Eigen::Vector2d d = path[i].position - path[i - 1].position;
    const double t = std::clamp((p - path[i - 1].position).dot(d) / d.squaredNorm(), 0.0, 1.0);
    best = std::min(best, (p - path[i - 1].position - t * d).norm());
  }
  return best;
}
} // namespace nav_executor::mpc
