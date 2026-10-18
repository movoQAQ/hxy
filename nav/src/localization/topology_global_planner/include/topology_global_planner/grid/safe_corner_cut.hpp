#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Core>

namespace topology_global_planner::grid
{
struct SafeCornerCutResult
{
  std::vector<Eigen::Vector2f> path;
  size_t rounded_corners{0};
};

SafeCornerCutResult safeCornerCut(
  const std::vector<Eigen::Vector2f> & input,
  const std::function<bool(const Eigen::Vector2f &, const Eigen::Vector2f &)> & line_free,
  const std::function<bool(const std::vector<Eigen::Vector2f> &)> & valid,
  float max_step, const std::function<void()> & check_cancel = {});
}  // namespace topology_global_planner::grid
