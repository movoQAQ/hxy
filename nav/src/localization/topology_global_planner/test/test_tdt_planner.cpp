#include <gtest/gtest.h>
#include <cmath>
#include "topology_global_planner/grid/grid_planner.hpp"
#include "topology_global_planner/grid/safe_corner_cut.hpp"
#include "map_server/costmap_layers.hpp"
#include "map_server/footprint.hpp"

using topology_global_planner::grid::GridPlanner;
namespace {
double maxHeadingChange(const nav_msgs::msg::Path & path) {
  double maximum = 0, previous_yaw = 0;
  bool have_previous = false;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    if (std::hypot(dx, dy) < 1e-4) {continue;}
    const double yaw = std::atan2(dy, dx);
    if (have_previous) {maximum = std::max(maximum, std::abs(std::remainder(yaw - previous_yaw, 2 * M_PI)));}
    previous_yaw = yaw; have_previous = true;
  }
  return maximum;
}
int headingReversals(const nav_msgs::msg::Path & path) {
  int reversals = 0, last_sign = 0;
  double previous_yaw = 0;
  bool have_previous = false;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    if (std::hypot(b.x - a.x, b.y - a.y) < 1e-4) {continue;}
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    if (have_previous) {
      const double change = std::remainder(yaw - previous_yaw, 2 * M_PI);
      const int sign = change > .012 ? 1 : change < -.012 ? -1 : 0;
      if (sign && last_sign && sign != last_sign) {++reversals;}
      if (sign) {last_sign = sign;}
    }
    previous_yaw = yaw; have_previous = true;
  }
  return reversals;
}
geometry_msgs::msg::PoseStamped pose(double x, double y) {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1;
  return p;
}
GridPlanner planner(map_server::Costmap2D & map, GridPlanner::Options options = {}) {
  return GridPlanner(&map, "map", std::make_shared<rclcpp::Clock>(), options);
}
}
TEST(CostmapEncoding, EveryPenaltyRoundTrips) {
  EXPECT_EQ(map_server::LETHAL_OBSTACLE, 0);
  EXPECT_EQ(map_server::FREE_SPACE, 255);
  for (int i = 0; i < 256; ++i) {
    EXPECT_EQ(map_server::fromObstacleCost(map_server::toObstacleCost(i)), i);
  }
}
TEST(CostmapEncoding, OccupancyBoundaryAndInflation) {
  nav_msgs::msg::OccupancyGrid occupancy;
  occupancy.header.frame_id = "map"; occupancy.info.width = 21; occupancy.info.height = 21;
  occupancy.info.resolution = 0.1; occupancy.info.origin.orientation.w = 1;
  occupancy.data.assign(441, 0); occupancy.data[10 * 21 + 10] = 100; occupancy.data[0] = -1;
  auto map = map_server::occupancyToCostmap(occupancy, true);
  EXPECT_EQ(map.getCost(10, 10), 0); EXPECT_EQ(map.getCost(1, 1), 255);
  EXPECT_EQ(map.getCost(0, 0), map_server::NO_INFORMATION);
  map_server::InflationOptions options; options.radius = .5; options.inscribed_radius = .15;
  map_server::inflate(map, options);
  EXPECT_EQ(map.getCost(10, 10), 0);
  EXPECT_EQ(map.getCost(11, 10), map_server::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_LT(map.getCost(12, 10), map.getCost(14, 10));
  EXPECT_LT(map_server::toObstacleCost(map.getCost(14, 10)),
    map_server::toObstacleCost(map.getCost(12, 10)));
}
TEST(TdtPlanner, SmoothStraightPathPreservesMetricEndpointsAndOrientation) {
  map_server::Costmap2D map(50, 50, .1, -2, -2);
  GridPlanner::Options options; options.tolerance = 0; options.use_final_approach_orientation = false;
  const auto start = pose(-1.45, -.45), goal = pose(1.45, .85);
  auto p = planner(map, options);
  auto path = p.plan(start, goal, {});
  ASSERT_TRUE(p.lastSmoothingReport().accepted) << p.lastSmoothingReport().reason;
  ASSERT_GE(path.poses.size(), 11u); // original tdt samples ten intervals per polynomial segment
  EXPECT_NEAR(path.poses.front().pose.position.x, start.pose.position.x, 1e-4);
  EXPECT_NEAR(path.poses.back().pose.position.y, goal.pose.position.y, 1e-4);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w, 1);
  EXPECT_EQ(map.getCost(0, 0), 255);
}
TEST(TdtPlanner, RejectsNoPathBoundsOccupiedStartAndCancellation) {
  map_server::Costmap2D map(30, 30, .1, 0, 0);
  for (unsigned int y = 0; y < 30; ++y) {map.setCost(15, y, 0);}
  GridPlanner::Options options; options.smooth = false; options.tolerance = 0;
  auto p = planner(map, options);
  EXPECT_THROW(p.plan(pose(.25, .25), pose(2.25, 2.25), {}), std::runtime_error);
  EXPECT_THROW(p.plan(pose(-.01, .25), pose(.45, .45), {}), std::runtime_error);
  EXPECT_THROW(p.plan(pose(.25, .25), pose(3, .45), {}), std::runtime_error);
  EXPECT_THROW(p.plan(pose(.25, .25), pose(.45, .45), [](){return true;}), std::runtime_error);
  int calls = 0;
  EXPECT_THROW(p.plan(pose(.25, .25), pose(2.25, 2.25), [&](){return ++calls > 4;}), std::runtime_error);
  map.setCost(2, 2, 0);
  EXPECT_THROW(p.plan(pose(.25, .25), pose(.45, .45), {}), std::runtime_error);
}
TEST(TdtPlanner, UnknownPolicyAndExactConnectorGoals) {
  map_server::Costmap2D map(30, 30, .1, 0, 0);
  for (unsigned int y = 0; y < 30; ++y) {map.setCost(15, y, map_server::NO_INFORMATION);}
  GridPlanner::Options options; options.smooth = false; options.tolerance = 0;
  options.allow_unknown = false;
  EXPECT_THROW(planner(map, options).plan(pose(.25, .25), pose(2.25, 2.25), {}), std::runtime_error);
  options.allow_unknown = true;
  EXPECT_FALSE(planner(map, options).plan(pose(.25, .25), pose(2.25, 2.25), {}).poses.empty());
  map.setCost(10, 10, 0); options.tolerance = .25;
  auto p = planner(map, options);
  EXPECT_FALSE(p.plan(pose(.25, .25), pose(1.05, 1.05), {}).poses.empty());
  EXPECT_THROW(p.plan(pose(.25, .25), pose(1.05, 1.05), {}, true), std::runtime_error);
}
TEST(TdtPlanner, ObstacleDetourAndSingleCell) {
  map_server::Costmap2D map(40, 40, .1, 0, 0);
  for (unsigned int y = 0; y < 25; ++y) {map.setCost(20, y, 0);}
  auto p = planner(map);
  auto path = p.plan(pose(.55, .55), pose(3.55, .55), {});
  ASSERT_TRUE(p.lastSmoothingReport().accepted) << p.lastSmoothingReport().reason;
  ASSERT_GT(path.poses.size(), 2u);
  GridPlanner::Options raw_options; raw_options.smooth = false;
  const auto raw = planner(map, raw_options).plan(pose(.55, .55), pose(3.55, .55), {});
  EXPECT_LT(maxHeadingChange(path), .35);
  EXPECT_LT(maxHeadingChange(path), maxHeadingChange(raw));
  RecordProperty("raw_max_heading_change_radians", std::to_string(maxHeadingChange(raw)));
  RecordProperty("smooth_max_heading_change_radians", std::to_string(maxHeadingChange(path)));
  // Independent geometric check of every curve chord across the wall's x slab.
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    double begin = 0, end = 1;
    if (std::abs(dx) < 1e-9) {
      if (a.x <= 2.00001 || a.x >= 2.09999) {continue;}
    } else {
      double enter = (2.00001 - a.x) / dx, exit = (2.09999 - a.x) / dx;
      if (enter > exit) {std::swap(enter, exit);}
      begin = std::max(begin, enter); end = std::min(end, exit);
    }
    if (begin <= end) {
      EXPECT_GE(std::min(a.y + begin * (b.y - a.y), a.y + end * (b.y - a.y)), 2.5 - 1e-5);
    }
  }
  double max_y = 0;
  for (const auto & point : path.poses) {
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(point.pose.position.x, point.pose.position.y, x, y));
    EXPECT_NE(map.getCost(x, y), 0); max_y = std::max(max_y, point.pose.position.y);
  }
  EXPECT_GT(max_y, 2.5);
  EXPECT_FALSE(p.plan(pose(.51, .51), pose(.54, .54), {}).poses.empty());
  EXPECT_EQ(p.plan(pose(.51, .51), pose(.51, .51), {}).poses.size(), 1u);
}
TEST(CostmapEncoding, FootprintReturnsMostDangerousTraversability) {
  map_server::Costmap2D map(20, 20, .1, 0, 0);
  map_server::Footprint f(4);
  f[0].x = -.1; f[0].y = -.1; f[1].x = .1; f[1].y = -.1;
  f[2].x = .1; f[2].y = .1; f[3].x = -.1; f[3].y = .1;
  map_server::FootprintCollisionChecker<map_server::Costmap2D *> checker(&map);
  EXPECT_EQ(checker.footprintCostAtPose(.55, .55, 0, f), 255);
  map.setCost(4, 4, 0);
  EXPECT_EQ(checker.footprintCostAtPose(.55, .55, 0, f), 0);
}
TEST(TdtPlanner, SfcCoordinatesStayInBoundsForReverseAndOffsetRoutes) {
  // Previously controls were relative to start and SFC received a negative
  // origin. Reverse routes then indexed a negative cell in _getBoundSquare2.
  for (const double origin : {0., -1000.}) {
    map_server::Costmap2D map(40, 40, .1, origin, origin);
    for (unsigned int y = 0; y < 25; ++y) {map.setCost(20, y, 0);}
    auto p = planner(map);
    auto path = p.plan(pose(origin + 3.55, origin + .55),
      pose(origin + .55, origin + .55), {});
    ASSERT_TRUE(p.lastSmoothingReport().accepted) << p.lastSmoothingReport().reason;
    ASSERT_FALSE(path.poses.empty());
    EXPECT_NEAR(path.poses.front().pose.position.x, origin + 3.55, 1e-4);
    EXPECT_NEAR(path.poses.back().pose.position.x, origin + .55, 1e-4);
    for (const auto & point : path.poses) {
      unsigned int x, y;
      ASSERT_TRUE(map.worldToMap(point.pose.position.x, point.pose.position.y, x, y));
      EXPECT_NE(map.getCost(x, y), 0);
    }
  }
}
TEST(TdtPlanner, SfcExpansionAtFreeMapEdgesHasAnObstacleBorder) {
  map_server::Costmap2D map(20, 20, .1, 0, 0);
  // A turn with no obstacle at the physical array edges: SFC must not expand
  // beyond its allocation even with the unchanged demo's 2.5m corridor range.
  for (unsigned int y = 3; y < 17; ++y) {map.setCost(10, y, 0);}
  auto p = planner(map);
  auto path = p.plan(pose(.05, .05), pose(1.95, 1.95), {});
  ASSERT_TRUE(p.lastSmoothingReport().attempted);
  ASSERT_FALSE(path.poses.empty());
}

TEST(TdtPlanner, KeepsCheckedSearchPathIfNeitherSmootherFitsANarrowTurn) {
  map_server::Costmap2D map(40, 40, .1, 0, 0, 0);
  for (unsigned int x = 2; x <= 25; ++x) {map.setCost(x, 5, 255);}
  for (unsigned int y = 5; y <= 30; ++y) {map.setCost(25, y, 255);}
  GridPlanner::Options options; options.smoothing_iterations = 1;
  auto p = planner(map, options);
  const auto path = p.plan(pose(.25, .55), pose(2.55, 3.05), {});
  EXPECT_TRUE(p.lastSmoothingReport().attempted);
  EXPECT_FALSE(p.lastSmoothingReport().accepted);
  options.smooth = false;
  const auto raw = planner(map, options).plan(pose(.25, .55), pose(2.55, 3.05), {});
  ASSERT_EQ(path.poses.size(), raw.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.poses[i].pose.position.x, raw.poses[i].pose.position.x);
    EXPECT_DOUBLE_EQ(path.poses[i].pose.position.y, raw.poses[i].pose.position.y);
  }
}
TEST(TdtPlanner, SmoothsTurnsAcrossMapResolutions) {
  for (const double resolution : {.05, .2}) {
    const unsigned int count = std::lround(4 / resolution);
    map_server::Costmap2D map(count, count, resolution, 0, 0);
    for (unsigned int y = 0; y < std::lround(2.5 / resolution); ++y) {
      map.setCost(std::lround(2 / resolution), y, 0);
    }
    auto p = planner(map);
    const auto path = p.plan(pose(.55, .55), pose(3.55, .55), {});
    ASSERT_TRUE(p.lastSmoothingReport().accepted) << "resolution=" << resolution << ": " <<
      p.lastSmoothingReport().reason;
    EXPECT_LT(maxHeadingChange(path), .4);
  }
}
TEST(TdtPlanner, ClosedFormDetourDoesNotFollowRasterWaves) {
  map_server::Costmap2D map(160, 120, .05, 0, 0);
  for (unsigned int y = 0; y < 120; ++y) {
    for (unsigned int x = 0; x < 160; ++x) {
      const double dx = (x + .5) * .05 - 4.0;
      const double dy = (y + .5) * .05 - 3.0;
      if (dx * dx / (1.0 * 1.0) + dy * dy / (.8 * .8) < 1.0) {
        map.setCost(x, y, map_server::LETHAL_OBSTACLE);
      }
    }
  }
  auto p = planner(map);
  const auto path = p.plan(pose(.5, 1.0), pose(7.5, 5.0), {});
  ASSERT_TRUE(p.lastSmoothingReport().accepted) << p.lastSmoothingReport().reason;
  EXPECT_LE(p.lastSmoothingReport().control_points, 4u);
  RecordProperty("controls", static_cast<int>(p.lastSmoothingReport().control_points));
  RecordProperty("heading_reversals", headingReversals(path));
  EXPECT_LE(headingReversals(path), 2);
}
TEST(TdtPlanner, SafeCornerCutBackupRoundsOnlyClearTurns) {
  using Point = Eigen::Vector2f;
  const std::vector<Point> input{{0, 0}, {1, 0}, {1, 1}};
  const auto clear = [](const Point &, const Point &) {return true;};
  const auto valid = [](const std::vector<Point> & points) {return points.size() >= 2;};
  const auto rounded = topology_global_planner::grid::safeCornerCut(
    input, clear, valid, .1f);
  EXPECT_GT(rounded.rounded_corners, 0u);
  ASSERT_GT(rounded.path.size(), input.size());
  EXPECT_TRUE(rounded.path.front().isApprox(input.front()));
  EXPECT_TRUE(rounded.path.back().isApprox(input.back()));
  for (size_t i = 1; i < rounded.path.size(); ++i) {
    EXPECT_LE((rounded.path[i] - rounded.path[i - 1]).norm(), .101f);
  }
  // A one-cell L corridor allows the axis-aligned route but blocks a cut.
  const auto corridor = [](const Point & a, const Point & b) {
      return std::abs(a.x() - b.x()) < 1e-6f || std::abs(a.y() - b.y()) < 1e-6f;
    };
  const auto unchanged = topology_global_planner::grid::safeCornerCut(
    input, corridor, valid, .1f);
  EXPECT_EQ(unchanged.rounded_corners, 0u);
  ASSERT_EQ(unchanged.path.size(), input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    EXPECT_TRUE(unchanged.path[i].isApprox(input[i]));
  }
}
