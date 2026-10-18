#pragma once
#include "map_server/costmap_2d.hpp"
#include <Eigen/Core>
#include <vector>

namespace nav_executor::mpc
{
struct Circle
{
  Eigen::Vector2d center;
  double radius;
};
struct PathPoint
{
  Eigen::Vector2d position;
  double arc{0}, speed{0};
};
struct Reference
{
  Eigen::Vector2d position, tangent;
  double speed{0}, arc{0};
};

std::vector<Circle> coverObstacles(const map_server::Costmap2D &map, bool allow_unknown,
                                   double robot_radius, int capacity);
bool trajectoryClear(const map_server::Costmap2D &map, const std::vector<Eigen::Vector2d> &points,
                     double radius, bool allow_unknown);
std::vector<PathPoint> makeSpeedProfile(const std::vector<Eigen::Vector2d> &points,
                                        double max_speed, double lateral_acc, double accel,
                                        double brake);
Reference sampleReference(const std::vector<PathPoint> &path, double arc);
double projectProgress(const std::vector<PathPoint> &path, const Eigen::Vector2d &position,
                       double previous, double search_distance);
double pathDistance(const std::vector<PathPoint> &path, const Eigen::Vector2d &position);
Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d &value);
} // namespace nav_executor::mpc
