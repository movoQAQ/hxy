#include "topology_global_planner/grid/safe_corner_cut.hpp"

#include <algorithm>
#include <cmath>

namespace topology_global_planner::grid
{
SafeCornerCutResult safeCornerCut(
  const std::vector<Eigen::Vector2f> & input,
  const std::function<bool(const Eigen::Vector2f &, const Eigen::Vector2f &)> & line_free,
  const std::function<bool(const std::vector<Eigen::Vector2f> &)> & valid,
  float max_step, const std::function<void()> & check_cancel)
{
  SafeCornerCutResult result{input, 0};
  if (input.size() < 3 || !std::isfinite(max_step) || max_step <= 0 || !valid(input)) {
    return result;
  }
  for (int round = 0; round < 2; ++round) {
    if (check_cancel) {check_cancel();}
    std::vector<Eigen::Vector2f> next;
    next.reserve(2 * result.path.size());
    next.push_back(result.path.front());
    for (size_t i = 1; i + 1 < result.path.size(); ++i) {
      if (check_cancel) {check_cancel();}
      const auto incoming = (result.path[i] - result.path[i - 1]).eval();
      const auto outgoing = (result.path[i + 1] - result.path[i]).eval();
      const float turn = std::abs(std::atan2(
        incoming.x() * outgoing.y() - incoming.y() * outgoing.x(),
        incoming.dot(outgoing)));
      if (turn < .03f) {next.push_back(result.path[i]); continue;}
      const Eigen::Vector2f before =
        (result.path[i] * .75f + result.path[i - 1] * .25f).eval();
      const Eigen::Vector2f after =
        (result.path[i] * .75f + result.path[i + 1] * .25f).eval();
      if (line_free(before, after)) {
        next.push_back(before);
        next.push_back(after);
        ++result.rounded_corners;
      } else {next.push_back(result.path[i]);}
    }
    next.push_back(result.path.back());
    if (!valid(next)) {return {input, 0};}
    result.path = std::move(next);
  }
  if (result.rounded_corners == 0) {return result;}
  std::vector<Eigen::Vector2f> sampled{result.path.front()};
  for (size_t i = 1; i < result.path.size(); ++i) {
    if (check_cancel) {check_cancel();}
    const auto delta = (result.path[i] - result.path[i - 1]).eval();
    const int steps = std::max(1, static_cast<int>(std::ceil(delta.norm() / max_step)));
    for (int k = 1; k <= steps; ++k) {
      sampled.push_back((result.path[i - 1] + delta * (static_cast<float>(k) / steps)).eval());
    }
  }
  if (!valid(sampled)) {return {input, 0};}
  result.path = std::move(sampled);
  return result;
}
}  // namespace topology_global_planner::grid
