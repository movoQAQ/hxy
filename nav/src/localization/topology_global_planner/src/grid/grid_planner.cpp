#include "topology_global_planner/grid/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>
#include "map_server/cost_values.hpp"
#include "topology_global_planner/grid/safe_corner_cut.hpp"
#include "yastar.hpp"
#include "minimumSnap.hpp"

namespace topology_global_planner::grid
{
GridPlanner::GridPlanner(map_server::Costmap2D * costmap,
  const std::string & global_frame, rclcpp::Clock::SharedPtr clock, Options options)
: costmap_(costmap), global_frame_(global_frame), clock_(std::move(clock)), options_(options)
{
  const auto positive = [](double x) {return std::isfinite(x) && x > 0;};
  if (!costmap_ || !clock_ || global_frame_.empty() ||
    !std::isfinite(options.tolerance) || options.tolerance < 0 ||
    !std::isfinite(options.cost_field_radius) || options.cost_field_radius < 0 ||
    !positive(options.cost_weight) || !positive(options.smoothing_dt) ||
    !positive(options.smoothing_max_speed) || !positive(options.smoothing_max_acc) ||
    !positive(options.smoothing_time_limit) || !positive(options.smoothing_corridor_range) ||
    options.smoothing_iterations < 1 ||
    (options.smoothing_backend != "close" && options.smoothing_backend != "osqp_corridor"))
  {throw std::invalid_argument("tdt planner: invalid configuration");}
}

nav_msgs::msg::Path GridPlanner::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel_checker, bool exact_goal)
{
  smoothing_report_ = {};
  const auto check_cancel = [&]() {
      if (cancel_checker && cancel_checker()) {throw std::runtime_error("Planning cancelled");}
    };
  check_cancel();
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw std::invalid_argument("tdt planner: frame mismatch");
  }
  map_server::Costmap2D snapshot(*costmap_);
  const int nx = snapshot.getSizeInCellsX(), ny = snapshot.getSizeInCellsY();
  const double resolution = snapshot.getResolution();
  const double ox = snapshot.getOriginX(), oy = snapshot.getOriginY();
  unsigned int sx, sy, gx, gy;
  if (nx < 3 || ny < 3 ||
    !snapshot.worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
    !snapshot.worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {throw std::runtime_error("tdt planner: invalid map or endpoints outside map");}

  // Costmap storage already uses tdt encoding. Resolve unknown policy in this local copy.
  MinimumSnap::Map map = Eigen::Map<const MinimumSnap::Map>(snapshot.getCharMap(), ny, nx);
  for (int i = 0; i < map.size(); ++i) {
    if (map.data()[i] == map_server::NO_INFORMATION) {
      map.data()[i] = options_.allow_unknown ? map_server::FREE_SPACE : map_server::LETHAL_OBSTACLE;
    }
    // The inscribed band is occupied by the robot centre, even though tdt accepts nonzero cells.
    if (map.data()[i] == map_server::INSCRIBED_INFLATED_OBSTACLE) {
      map.data()[i] = map_server::LETHAL_OBSTACLE;
    }
  }
  // Preserve collision data at the start; the costmap producer clears the footprint.
  if (map(sy, sx) == 0) {throw std::runtime_error("tdt planner: occupied start");}
  // Work in cell coordinates for tdt. This avoids float precision loss at large map origins.
  const Eigen::Vector2f first((start.pose.position.x - ox) / resolution,
    (start.pose.position.y - oy) / resolution);
  const Eigen::Vector2f requested((goal.pose.position.x - ox) / resolution,
    (goal.pose.position.y - oy) / resolution);
  YAstar astar;
  astar.setMap(nx, ny, map.data());
  astar.setMapping(1.f);
  astar.setOriginPos(0.f, 0.f);
  astar.setCostWeight(options_.cost_weight);
  astar.setCostField(options_.cost_field_radius / resolution, [](float d) {
      return 1.f / std::max(d, 0.1f);
    });
  // Check every touched cell (including diagonal corners) against the same snapshot.
  const auto point_free = [&](const Eigen::Vector2f & p) {
      return p.allFinite() && p.x() >= 0 && p.y() >= 0 && p.x() < nx && p.y() < ny &&
             map(static_cast<int>(p.y()), static_cast<int>(p.x())) != 0;
    };
  const auto line_free = [&](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
      if (!point_free(a) || !point_free(b)) {return false;}
      int x = std::floor(a.x()), y = std::floor(a.y());
      const int ex = std::floor(b.x()), ey = std::floor(b.y());
      const double dx = b.x() - a.x(), dy = b.y() - a.y();
      const int step_x = dx > 0 ? 1 : -1, step_y = dy > 0 ? 1 : -1;
      const double inf = std::numeric_limits<double>::infinity();
      double tx = dx == 0 ? inf : ((dx > 0 ? x + 1 : x) - a.x()) / dx;
      double ty = dy == 0 ? inf : ((dy > 0 ? y + 1 : y) - a.y()) / dy;
      const double dtx = dx == 0 ? inf : 1 / std::abs(dx);
      const double dty = dy == 0 ? inf : 1 / std::abs(dy);
      const auto cell_free = [&](int cx, int cy) {
          return cx >= 0 && cy >= 0 && cx < nx && cy < ny && map(cy, cx) != 0;
        };
      while (x != ex || y != ey) {
        if (std::abs(tx - ty) < 1e-9) {
          if (!cell_free(x + step_x, y) || !cell_free(x, y + step_y)) {return false;}
          x += step_x; y += step_y; tx += dtx; ty += dty;
        } else if (tx < ty) {x += step_x; tx += dtx;}
        else {y += step_y; ty += dty;}
        if (!cell_free(x, y)) {return false;}
      }
      return true;
    };
  const auto valid = [&](const std::vector<Eigen::Vector2f> & points) {
      if (points.empty() || !point_free(points.front())) {return false;}
      for (size_t i = 1; i < points.size(); ++i) {
        if (!line_free(points[i - 1], points[i])) {return false;}
      }
      return true;
    };
  std::vector<Eigen::Vector2f> candidates{requested};
  if (!exact_goal && options_.tolerance > 0) {
    const double radius = options_.tolerance / resolution;
    for (int y = std::max(0, static_cast<int>(std::floor(requested.y() - radius)));
      y <= std::min(ny - 1, static_cast<int>(std::ceil(requested.y() + radius))); ++y)
    {
      for (int x = std::max(0, static_cast<int>(std::floor(requested.x() - radius)));
        x <= std::min(nx - 1, static_cast<int>(std::ceil(requested.x() + radius))); ++x)
      {
        Eigen::Vector2f p(x + 0.5f, y + 0.5f);
        if ((p - requested).norm() <= radius && point_free(p)) {candidates.push_back(p);}
      }
    }
    std::stable_sort(candidates.begin() + 1, candidates.end(), [&](const auto & a, const auto & b) {
        return (a - requested).squaredNorm() < (b - requested).squaredNorm();
      });
  }
  std::vector<Eigen::Vector2f> points;
  for (const auto & target : candidates) {
    check_cancel();
    if (!point_free(target)) {continue;}
    if ((target - first).norm() < 1e-5f) {points = {first}; break;}
    auto raw = astar.search(first, target, cancel_checker);
    check_cancel();
    // YAstar returns only start on failure and emits cell corners on success.
    if (raw.empty() || (raw.back() - Eigen::Vector2f(std::floor(target.x()),
      std::floor(target.y()))).norm() > 1e-3f) {continue;}
    for (auto & p : raw) {p += Eigen::Vector2f(0.5f, 0.5f);}
    raw.insert(raw.begin(), first);
    raw.push_back(target);
    if (!valid(raw)) {continue;}
    points = std::move(raw); break;
  }
  if (points.empty()) {throw std::runtime_error("tdt planner: no collision-free reachable goal");}
  // Remove duplicates before time allocation; zero-length segments are invalid.
  points.erase(std::unique(points.begin(), points.end(), [](const auto & a, const auto & b) {
      return (a - b).norm() < 1e-5f;
    }), points.end());
  if (options_.smooth && points.size() > 1) {
    smoothing_report_.attempted = true;
    // Preserve obstacle turns but discard centimetre-scale YAstar stair steps.
    // At 5 cm map resolution the former half-cell tolerance kept every 2.5 cm
    // lateral variation, which the interpolating closed-form solver reproduced
    // as visible waves. Visibility still forces a split before an obstacle.
    const float waypoint_tolerance_cells =
      std::max(0.5f, static_cast<float>(0.1 / resolution));
    std::vector<bool> keep(points.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<size_t, size_t>> spans{{0, points.size() - 1}};
    while (!spans.empty()) {
      check_cancel();
      const auto [begin, end] = spans.back(); spans.pop_back();
      if (end <= begin + 1) {continue;}
      const auto delta = (points[end] - points[begin]).eval();
      const float length2 = delta.squaredNorm();
      float deviation = 0; size_t split = (begin + end) / 2;
      for (size_t i = begin + 1; i < end; ++i) {
        const float t = length2 > 0 ? std::clamp((points[i] - points[begin]).dot(delta) /
          length2, 0.f, 1.f) : 0.f;
        const float d = (points[i] - points[begin] - t * delta).norm();
        if (d > deviation) {deviation = d; split = i;}
      }
      if (deviation > waypoint_tolerance_cells || !line_free(points[begin], points[end])) {
        keep[split] = true; spans.emplace_back(begin, split); spans.emplace_back(split, end);
      }
    }
    std::vector<Eigen::Vector2f> controls;
    for (size_t i = 0; i < points.size(); ++i) {if (keep[i]) {controls.push_back(points[i]);}}
    // A global RDP split around an obstacle can retain tiny local stair steps.
    // Remove only points whose neighbouring chord remains clear and close.
    for (size_t i = 1; i + 1 < controls.size();) {
      const auto chord = (controls[i + 1] - controls[i - 1]).eval();
      const float length2 = chord.squaredNorm();
      const float ratio = length2 > 0.f ? std::clamp(
        (controls[i] - controls[i - 1]).dot(chord) / length2, 0.f, 1.f) : 0.f;
      const float deviation = (controls[i] - controls[i - 1] - ratio * chord).norm();
      if (deviation <= waypoint_tolerance_cells &&
        line_free(controls[i - 1], controls[i + 1])) {
        controls.erase(controls.begin() + i);
        if (i > 1) {--i;}
      } else {++i;}
    }
    // Avoid carrying raster stair steps into an unobstructed straight connection.
    if (line_free(points.front(), points.back())) {controls = {points.front(), points.back()};}
    smoothing_report_.control_points = controls.size();
    const auto controls_grid = controls;
    {
      MinimumSnap smoother;
      // Match the unmodified tdt main.cpp demo conditions.
      smoother.setOrder(6); smoother.setMaxDx(3);
      smoother.setDt(options_.smoothing_dt);
      smoother.setTL(options_.smoothing_time_limit);
      // Original SFC uses +origin, collision checks use -origin. A zero origin is
      // the common supported convention; do not pass the robot position as origin.
      // SFC also assumes an obstacle border instead of checking array bounds.
      constexpr int border = 2;
      MinimumSnap::Map smoothing_map = MinimumSnap::Map::Zero(ny + 2 * border, nx + 2 * border);
      smoothing_map.block(border, border, ny, nx) = map;
      const Eigen::Vector2f padding(border, border);
      for (auto & point : controls) {point = ((point + padding) * resolution).eval();}
      smoother.setMap(smoothing_map, resolution, 0.f, 0.f);
      MinimumSnap::SolveInput input;
      input.setPath(controls);
      input.setInitVel(Eigen::Vector2f::Zero());
      input.setMaxSpeed(options_.smoothing_max_speed);
      input.setMaxAcc(options_.smoothing_max_acc);
      input.setNormTime(true);
      input.setCollisionCheckIter(options_.smoothing_iterations);
      input.setMaxCorridorRange(options_.smoothing_corridor_range);
      input.setCorridorShrink(0.f);
      input.setBackend(options_.smoothing_backend == "close" ?
        MinimumSnap::Backend::Close : MinimumSnap::Backend::OSQPCorridor);
      check_cancel();
      auto accept = [&](MinimumSnap::SolveOutput result, const std::string & backend) {
          smoothing_report_.backend = backend;
          if (!result.success) {smoothing_report_.reason = "solver failed or curve collided"; return false;}
          for (auto & point : result.path) {point = (point / resolution - padding).eval();}
          if (result.path.size() < 2 ||
            (result.path.front() - points.front()).norm() > 1e-3f ||
            (result.path.back() - points.back()).norm() > 1e-3f)
          {smoothing_report_.reason = "endpoint mismatch"; return false;}
          if (!valid(result.path)) {smoothing_report_.reason = "strict collision check failed"; return false;}
          points = std::move(result.path);
          smoothing_report_.accepted = true;
          smoothing_report_.reason = "accepted";
          smoothing_report_.output_points = points.size();
          return true;
        };
      auto result = smoother.solve(input);
      check_cancel();
      bool accepted = accept(std::move(result), options_.smoothing_backend);
      if (!accepted && options_.smoothing_backend == "osqp_corridor") {
        // Reuse tdt's equality-constrained polynomial optimizer when the corridor
        // ADMM solve fails. This is still a smoothing solve, with the same order,
        // timing and endpoint constraints; accept only a checked collision-free curve.
        RCLCPP_INFO(rclcpp::get_logger("tdt_planner"),
          "OSQP corridor smoothing rejected (%s); retrying tdt closed-form smoothing",
          smoothing_report_.reason.c_str());
        input.setBackend(MinimumSnap::Backend::Close);
        check_cancel();
        auto retry = smoother.solve(input);
        check_cancel();
        accepted = accept(std::move(retry), "close_retry");
      }
      if (accepted) {
        RCLCPP_INFO(rclcpp::get_logger("tdt_planner"),
          "MinimumSnap accepted: backend=%s controls=%zu samples=%zu",
          smoothing_report_.backend.c_str(), controls.size(), points.size());
      } else {
        const std::string rejection = smoothing_report_.reason;
        // This backup uses the same supercover collision check and leaves
        // narrow corners untouched when a local cut would cross an obstacle.
        const auto & source = valid(controls_grid) ? controls_grid : points;
        auto backup = safeCornerCut(source, line_free, valid,
          std::max(1.f, static_cast<float>(0.1 / resolution)), check_cancel);
        if (backup.rounded_corners > 0) {
          points = std::move(backup.path);
          smoothing_report_.backend = "safe_corner_cut";
          smoothing_report_.accepted = true;
          smoothing_report_.reason = "MinimumSnap rejected: " + rejection;
          smoothing_report_.output_points = points.size();
          RCLCPP_WARN(rclcpp::get_logger("tdt_planner"),
            "MinimumSnap rejected: %s; safe corner-cut backup accepted: corners=%zu samples=%zu",
            rejection.c_str(), backup.rounded_corners, points.size());
        }
        if (!smoothing_report_.accepted) {
          RCLCPP_WARN(rclcpp::get_logger("tdt_planner"),
            "MinimumSnap and safe corner-cut backup rejected: %s (controls=%zu); using checked YAstar path",
            rejection.c_str(), controls.size());
        }
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ox + points[i].x() * resolution;
    pose.pose.position.y = oy + points[i].y() * resolution;
    const auto delta = points.size() == 1 ? Eigen::Vector2f::Zero().eval() :
      (i + 1 < points.size() ? (points[i + 1] - points[i]).eval() :
      (points[i] - points[i - 1]).eval());
    const double yaw = std::atan2(delta.y(), delta.x());
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    path.poses.push_back(pose);
  }
  if (!options_.use_final_approach_orientation) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  } else if (points.size() == 1) {path.poses.back().pose.orientation = start.pose.orientation;}
  check_cancel();
  return path;
}
}  // namespace topology_global_planner::grid
