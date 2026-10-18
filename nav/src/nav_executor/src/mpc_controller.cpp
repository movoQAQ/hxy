#include "nav_executor/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <qpOASES.hpp>

namespace nav_executor
{
namespace
{
double cross(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
  return a.x() * b.y() - a.y() * b.x();
}

Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d &v)
{
  return {std::cos(yaw) * v.x() - std::sin(yaw) * v.y(),
          std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};
}
} // namespace

void MpcController::initialize(const rclcpp::Node::SharedPtr &node,
                               std::shared_ptr<tf2_ros::Buffer> tf, map_server::Costmap2D *costmap,
                               std::string global_frame, std::string base_frame,
                               double robot_radius)
{
  node_ = node;
  tf_ = std::move(tf);
  costmap_ = costmap;
  global_frame_ = std::move(global_frame);
  base_frame_ = std::move(base_frame);
  robot_radius_ = robot_radius;
  allow_unknown_ = node->declare_parameter("MPC.allow_unknown", false);
  dt_ = node->declare_parameter("MPC.dt", 0.05);
  horizon_ = node->declare_parameter("MPC.horizon", 30);
  max_speed_ = node->declare_parameter("MPC.max_speed", 1.4);
  max_acc_ = node->declare_parameter("MPC.max_acc", 0.8);
  max_lateral_acc_ = node->declare_parameter("MPC.max_lateral_acc", 0.6);
  q_along_ = node->declare_parameter("MPC.q_along", 5.0);
  q_cross_ = node->declare_parameter("MPC.q_cross", 20.0);
  r_speed_ = node->declare_parameter("MPC.r_speed", 1.5);
  delay_ = node->declare_parameter("MPC.control_delay", 0.08);
  max_working_set_recalculations_ =
      node->declare_parameter("MPC.max_working_set_recalculations", 500);
  max_brake_ = node->declare_parameter("MPC.max_brake", 0.8);
  response_time_ = node->declare_parameter("MPC.response_time", 0.2);
  direction_rate_ = node->declare_parameter("MPC.max_direction_rate", 1.2);
  curvature_window_ = node->declare_parameter("MPC.curvature_window", 0.2);
  recovery_distance_ = node->declare_parameter("MPC.recovery_distance", 0.3);
  recovery_gain_ = node->declare_parameter("MPC.recovery_gain", 1.5);
  max_recovery_speed_ = node->declare_parameter("MPC.max_recovery_speed", 0.6);
  goal_speed_gain_ = node->declare_parameter("MPC.goal_speed_gain", 1.2);
  terminal_weight_ = node->declare_parameter("MPC.terminal_weight", 5.0);
  r_delta_ = node->declare_parameter("MPC.r_delta_velocity", 0.3);
  projection_backtrack_ = node->declare_parameter("MPC.projection_backtrack", 0.5);
  for (const double value : {max_brake_, response_time_, direction_rate_, curvature_window_,
                             recovery_distance_, recovery_gain_, max_recovery_speed_,
                             goal_speed_gain_, terminal_weight_, r_delta_, projection_backtrack_})
  {
    if (!std::isfinite(value) || value <= 0)
    {
      throw std::invalid_argument("Invalid MPC tracking/braking configuration");
    }
  }
  if (!node_ || !tf_ || !costmap_ || global_frame_.empty() || base_frame_.empty() ||
      !std::isfinite(dt_) || dt_ <= 0 || dt_ > 0.2 || horizon_ < 1 || horizon_ > 40 ||
      !std::isfinite(max_speed_) || max_speed_ <= 0 || !std::isfinite(max_acc_) || max_acc_ <= 0 ||
      !std::isfinite(max_lateral_acc_) || max_lateral_acc_ <= 0 || !std::isfinite(q_along_) ||
      q_along_ <= 0 || !std::isfinite(q_cross_) || q_cross_ <= 0 || !std::isfinite(r_speed_) ||
      r_speed_ <= 0 || !std::isfinite(delay_) || delay_ < 0 || delay_ > 0.2 ||
      !std::isfinite(robot_radius_) || robot_radius_ <= 0 || max_working_set_recalculations_ < 1)
  {
    throw std::invalid_argument("Invalid MPC configuration");
  }
}

void MpcController::setPlan(const nav_msgs::msg::Path &path)
{
  if (path.poses.empty() || path.header.frame_id.empty())
  {
    throw std::invalid_argument("MPC requires a nonempty framed path");
  }
  reset();
  plan_ = path;
}

void MpcController::reset()
{
  plan_.poses.clear();
  progress_ = 0;
  last_command_.setZero();
  have_last_command_ = false;
}

std::vector<MpcController::PathPoint>
MpcController::transformPlan(const geometry_msgs::msg::PoseStamped &pose) const
{
  geometry_msgs::msg::TransformStamped transform;
  const bool change_frame = plan_.header.frame_id != global_frame_;
  if (change_frame)
  {
    transform = tf_->lookupTransform(global_frame_, plan_.header.frame_id, tf2::TimePointZero);
  }
  std::vector<PathPoint> path;
  path.reserve(plan_.poses.size() + 1);
  for (const auto &waypoint : plan_.poses)
  {
    geometry_msgs::msg::PoseStamped converted = waypoint;
    converted.header.frame_id = plan_.header.frame_id;
    if (change_frame)
    {
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(converted, transformed, transform);
      converted = std::move(transformed);
    }
    Eigen::Vector2d point(converted.pose.position.x, converted.pose.position.y);
    if (!point.allFinite())
    {
      throw std::runtime_error("Invalid transformed MPC path");
    }
    if (!path.empty() && (point - path.back().point).norm() < 1e-5)
    {
      continue;
    }
    const double arc = path.empty() ? 0.0 : path.back().arc + (point - path.back().point).norm();
    path.push_back({point, arc});
  }
  if (path.size() == 1)
  {
    const Eigen::Vector2d current(pose.pose.position.x, pose.pose.position.y);
    path.insert(path.begin(), {current, 0.0});
    path.back().arc = (path.back().point - current).norm();
  }
  if (path.size() < 2)
  {
    throw std::runtime_error("MPC path has no usable points");
  }
  buildSpeedProfile(path);
  return path;
}

void MpcController::buildSpeedProfile(std::vector<PathPoint> &path) const
{
  auto pointAt = [&](double arc) -> Eigen::Vector2d
  {
    arc = std::clamp(arc, 0.0, path.back().arc);
    auto upper =
        std::upper_bound(path.begin(), path.end(), arc,
                         [](double value, const PathPoint &item) { return value < item.arc; });
    const size_t i = std::clamp<size_t>(upper - path.begin(), 1, path.size() - 1);
    const double ds = path[i].arc - path[i - 1].arc;
    if (ds < 1e-8)
    {
      return path[i].point;
    }
    return path[i - 1].point + (path[i].point - path[i - 1].point) * ((arc - path[i - 1].arc) / ds);
  };
  for (size_t i = 0; i < path.size(); ++i)
  {
    const double arc = path[i].arc;
    const Eigen::Vector2d before = path[i].point - pointAt(arc - curvature_window_);
    const Eigen::Vector2d after = pointAt(arc + curvature_window_) - path[i].point;
    double curvature = 0.0;
    if (before.norm() > 1e-5 && after.norm() > 1e-5)
    {
      const double angle = std::abs(std::atan2(cross(before, after), before.dot(after)));
      curvature = angle / std::max(0.01, 0.5 * (before.norm() + after.norm()));
    }
    const double curve_speed = std::sqrt(max_lateral_acc_ / std::max(curvature, 1e-6));
    const double steering_speed = direction_rate_ / std::max(curvature, 1e-6);
    const double remaining = path.back().arc - arc;
    path[i].speed =
        std::min({max_speed_, curve_speed, steering_speed, goal_speed_gain_ * remaining});
  }
  const double braking = std::min(max_brake_, max_acc_);
  // Propagate every upcoming bend/goal limit backward, including actuator delay.
  for (size_t i = path.size() - 1; i > 0; --i)
  {
    const double ds = path[i].arc - path[i - 1].arc;
    const double end = path[i].speed;
    const double distance_speed = std::sqrt(end * end + 2 * braking * ds);
    path[i - 1].speed = std::min(path[i - 1].speed, distance_speed);
  }
}

MpcController::Sample MpcController::sample(double arc, const std::vector<PathPoint> &path) const
{
  const double s = std::clamp(arc, 0.0, path.back().arc);
  auto upper = std::upper_bound(path.begin(), path.end(), s, [](double value, const PathPoint &item)
                                { return value < item.arc; });
  const size_t i = std::clamp<size_t>(upper - path.begin(), 1, path.size() - 1);
  const auto &a = path[i - 1];
  const auto &b = path[i];
  const double ds = b.arc - a.arc;
  if (ds < 1e-8)
  {
    return {b.point, Eigen::Vector2d(1.0, 0.0), 0.0};
  }
  const Eigen::Vector2d tangent = (b.point - a.point) / ds;
  const double ratio = (s - a.arc) / ds;
  const double speed =
      std::sqrt(std::max(0.0, (1 - ratio) * a.speed * a.speed + ratio * b.speed * b.speed));
  return {a.point + tangent * (s - a.arc), tangent,
          std::min(speed, goal_speed_gain_ * (path.back().arc - s))};
}

bool MpcController::rolloutClear(const std::vector<Eigen::Vector2d> &predicted) const
{
  const map_server::Costmap2D map(*costmap_);
  if (map.getSizeInCellsX() == 0 || predicted.empty())
  {
    return false;
  }
  const double resolution = map.getResolution();
  const double check_radius = robot_radius_;
  for (size_t i = 1; i < predicted.size(); ++i)
  {
    const auto &a = predicted[i - 1];
    const auto &b = predicted[i];
    const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm() / (resolution * 0.5))));
    for (int j = 0; j <= steps; ++j)
    {
      const Eigen::Vector2d point = a + (b - a) * (static_cast<double>(j) / steps);
      unsigned int mx, my;
      if (!map.worldToMap(point.x(), point.y(), mx, my))
      {
        return false;
      }
      const auto center_cost = map.getCost(mx, my);
      if (center_cost == map_server::INSCRIBED_INFLATED_OBSTACLE ||
          (!allow_unknown_ && center_cost == map_server::NO_INFORMATION))
      {
        return false;
      }
      const int cells = static_cast<int>(std::ceil(check_radius / resolution));
      for (int y = std::max(0, static_cast<int>(my) - cells);
           y <= std::min(static_cast<int>(map.getSizeInCellsY()) - 1, static_cast<int>(my) + cells);
           ++y)
      {
        for (int x = std::max(0, static_cast<int>(mx) - cells);
             x <=
             std::min(static_cast<int>(map.getSizeInCellsX()) - 1, static_cast<int>(mx) + cells);
             ++x)
        {
          const double cx = map.getOriginX() + (x + 0.5) * resolution;
          const double cy = map.getOriginY() + (y + 0.5) * resolution;
          if (std::hypot(cx - point.x(), cy - point.y()) >
              check_radius + resolution * std::sqrt(0.5))
          {
            continue;
          }
          const auto cost = map.getCost(x, y);
          if (cost == map_server::LETHAL_OBSTACLE ||
              (!allow_unknown_ && cost == map_server::NO_INFORMATION))
          {
            return false;
          }
        }
      }
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped
MpcController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::Twist &measured_speed)
{
  if (plan_.poses.empty())
  {
    throw std::runtime_error("MPC has no path");
  }
  if (pose.header.frame_id != global_frame_)
  {
    throw std::runtime_error("MPC pose frame mismatch");
  }
  const auto path = transformPlan(pose);
  const Eigen::Vector2d p(pose.pose.position.x, pose.pose.position.y);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const Eigen::Vector2d measured_world =
      rotate(yaw, {measured_speed.linear.x, measured_speed.linear.y});
  if (!p.allFinite() || !measured_world.allFinite() || !std::isfinite(yaw) ||
      !std::isfinite(measured_speed.angular.z))
  {
    throw std::runtime_error("Invalid MPC odometry state");
  }
  // Reacquire nearby earlier segments after a bend overshoot. A bounded arc
  // window prevents jumping to a distant branch at a self-intersection.
  double best_distance = INFINITY, best_arc = progress_;
  const double forward_window = std::max(0.6, measured_world.norm() * 0.4);
  for (size_t i = 1; i < path.size(); ++i)
  {
    const auto segment = path[i].point - path[i - 1].point;
    const double length2 = segment.squaredNorm();
    if (length2 < 1e-10)
    {
      continue;
    }
    const double ratio = std::clamp((p - path[i - 1].point).dot(segment) / length2, 0.0, 1.0);
    const double candidate = path[i - 1].arc + ratio * std::sqrt(length2);
    if (candidate < progress_ - projection_backtrack_ || candidate > progress_ + forward_window)
    {
      continue;
    }
    const double distance = (p - path[i - 1].point - ratio * segment).squaredNorm();
    if (distance < best_distance)
    {
      best_distance = distance;
      best_arc = candidate;
    }
  }
  progress_ = best_arc;
  const auto nearest = sample(progress_, path);
  const Eigen::Vector2d normal(-nearest.tangent.y(), nearest.tangent.x());
  const double lateral_error = (p - nearest.point).dot(normal);
  const double recovery_scale =
      1.0 / (1.0 + std::pow(std::sqrt(best_distance) / recovery_distance_, 2));
  const int nv = 2 * horizon_;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::VectorXd reference(nv), feedforward(nv), initial(nv), initial_velocity(nv);
  const Eigen::Vector2d delayed_position = p + measured_world * delay_;
  const double beta = std::exp(-dt_ / response_time_);
  double ref_arc = progress_;
  for (int k = 0; k < horizon_; ++k)
  {
    auto at = sample(ref_arc, path);
    // Preview the speed required after the response delay, rather than arriving
    // at a bend before requesting its lower speed.
    const auto ahead = sample(ref_arc + at.speed * (delay_ + response_time_), path);
    const double speed = std::min(at.speed, ahead.speed) * recovery_scale;
    ref_arc = std::min(path.back().arc, ref_arc + speed * dt_);
    at = sample(ref_arc, path);
    reference.segment<2>(2 * k) = at.point;
    const Eigen::Vector2d stage_normal(-at.tangent.y(), at.tangent.x());
    const double correction =
        std::clamp(-recovery_gain_ * lateral_error, -max_recovery_speed_, max_recovery_speed_) *
        std::exp(-k * dt_);
    feedforward.segment<2>(2 * k) = at.tangent * speed + stage_normal * correction;
    const double decay = std::pow(beta, k + 1);
    initial.segment<2>(2 * k) = delayed_position + response_time_ * (1 - decay) * measured_world;
    initial_velocity.segment<2>(2 * k) = decay * measured_world;
    const double terminal = k + 1 == horizon_ ? terminal_weight_ : 1.0;
    Q.block<2, 2>(2 * k, 2 * k) = terminal * (q_along_ * at.tangent * at.tangent.transpose() +
                                              q_cross_ * stage_normal * stage_normal.transpose());
    for (int j = 0; j <= k; ++j)
    {
      const double power = std::pow(beta, k - j);
      B.block<2, 2>(2 * k, 2 * j) =
          (dt_ - response_time_ * (1 - beta) * power) * Eigen::Matrix2d::Identity();
      V.block<2, 2>(2 * k, 2 * j) = (1 - beta) * power * Eigen::Matrix2d::Identity();
    }
  }
  Eigen::Vector2d previous = have_last_command_ ? last_command_ : measured_world;
  if (previous.norm() > max_speed_)
  {
    previous *= max_speed_ / previous.norm();
  }
  // A handoff may measure a velocity outside the inscribed polygon even when
  // its norm is below max_speed. Project only the *initial command anchor*;
  // keep the measured plant velocity unchanged in the response prediction.
  double anchor_scale = 1.0;
  for (int i = 0; i < 8; ++i)
  {
    const double angle = (i + 0.5) * 2 * M_PI / 8;
    const double projection = std::cos(angle) * previous.x() + std::sin(angle) * previous.y();
    if (projection > max_speed_ * std::cos(M_PI / 8))
    {
      anchor_scale = std::min(anchor_scale, max_speed_ * std::cos(M_PI / 8) / projection);
    }
  }
  previous *= anchor_scale;
  Eigen::MatrixXd D = Eigen::MatrixXd::Identity(nv, nv);
  for (int k = 1; k < horizon_; ++k)
  {
    D.block<2, 2>(2 * k, 2 * (k - 1)) = -Eigen::Matrix2d::Identity();
  }
  Eigen::VectorXd offset = Eigen::VectorXd::Zero(nv);
  offset.head<2>() = -previous;
  Eigen::MatrixXd H =
      2.0 * (B.transpose() * Q * B + r_speed_ * V.transpose() * V + r_delta_ * D.transpose() * D);
  H.diagonal().array() += 1e-6;
  const Eigen::VectorXd g = 2.0 * (B.transpose() * Q * (initial - reference) +
                                   r_speed_ * V.transpose() * (initial_velocity - feedforward) +
                                   r_delta_ * D.transpose() * offset);
  // Inscribed octagons constrain Euclidean speed/command acceleration inside
  // the QP. No norm clipping is allowed after solving.
  const int directions = 8;
  const bool constrain_heading = previous.norm() > 0.15;
  const int nc = 2 * directions * horizon_ + (constrain_heading ? 3 : 0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nv);
  Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nc, -qpOASES::INFTY);
  Eigen::VectorXd ubA = Eigen::VectorXd::Zero(nc);
  const double polygon = std::cos(M_PI / directions);
  for (int k = 0; k < horizon_; ++k)
  {
    for (int direction = 0; direction < directions; ++direction)
    {
      const double angle = (direction + 0.5) * 2 * M_PI / directions;
      const Eigen::RowVector2d axis(std::cos(angle), std::sin(angle));
      const int speed_row = 2 * directions * k + direction;
      A.block<1, 2>(speed_row, 2 * k) = axis;
      ubA[speed_row] = max_speed_ * polygon;
      const int rate_row = speed_row + directions;
      A.block<1, 2>(rate_row, 2 * k) = axis;
      if (k)
      {
        A.block<1, 2>(rate_row, 2 * (k - 1)) = -axis;
      }
      const double anchor = k ? 0.0 : axis.dot(previous);
      // Use the conservative braking/acceleration envelope for every direction.
      ubA[rate_row] = anchor + std::min(max_acc_, max_brake_) * dt_ * polygon;
    }
  }
  if (constrain_heading)
  {
    const Eigen::Vector2d along = previous.normalized(), across(-along.y(), along.x());
    const double turn = std::tan(std::min(0.5, direction_rate_ * dt_));
    A.block<1, 2>(nc - 3, 0) = (across - turn * along).transpose();
    A.block<1, 2>(nc - 2, 0) = (-across - turn * along).transpose();
    A.block<1, 2>(nc - 1, 0) = -along.transpose();
  }
  // qpOASES accepts row-major dense arrays.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Hr = H, Ar = A;
  std::vector<qpOASES::real_t> h(Hr.data(), Hr.data() + Hr.size());
  std::vector<qpOASES::real_t> a(Ar.data(), Ar.data() + Ar.size());
  std::vector<qpOASES::real_t> grad(g.data(), g.data() + g.size());
  std::vector<qpOASES::real_t> lower(nv, -max_speed_), upper(nv, max_speed_);
  std::vector<qpOASES::real_t> lower_a(lbA.data(), lbA.data() + lbA.size());
  std::vector<qpOASES::real_t> upper_a(ubA.data(), ubA.data() + ubA.size());
  qpOASES::QProblem problem(nv, nc);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  problem.setOptions(options);
  qpOASES::int_t work = max_working_set_recalculations_;
  const auto status = problem.init(h.data(), grad.data(), a.data(), lower.data(), upper.data(),
                                   lower_a.data(), upper_a.data(), work);
  if (status != qpOASES::SUCCESSFUL_RETURN)
  {
    throw std::runtime_error("MPC qpOASES solve failed: " +
                             std::to_string(static_cast<int>(status)));
  }
  std::vector<qpOASES::real_t> solution(nv);
  if (problem.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN)
  {
    throw std::runtime_error("MPC solution unavailable");
  }
  std::vector<Eigen::Vector2d> predicted{p, delayed_position};
  Eigen::Vector2d position = delayed_position, velocity = measured_world;
  Eigen::Vector2d first_command = Eigen::Vector2d::Zero();
  for (int k = 0; k < horizon_; ++k)
  {
    const Eigen::Vector2d u(solution[2 * k], solution[2 * k + 1]);
    if (!u.allFinite() || u.norm() > max_speed_ + 1e-4)
    {
      throw std::runtime_error("MPC invalid velocity");
    }
    position += response_time_ * (1 - beta) * velocity + (dt_ - response_time_ * (1 - beta)) * u;
    velocity = beta * velocity + (1 - beta) * u;
    predicted.push_back(position);
    if (!k)
    {
      first_command = u;
    }
  }
  // Validate a conservative braking tail, not just a horizon ending while moving.
  for (int k = 0; velocity.norm() > 0.01 && k < 200; ++k)
  {
    position += velocity * dt_;
    const double norm = velocity.norm();
    velocity *= std::max(0.0, norm - std::min(max_acc_, max_brake_) * dt_) / norm;
    predicted.push_back(position);
  }
  if (!rolloutClear(predicted))
  {
    throw std::runtime_error("MPC predicted path intersects costmap");
  }
  last_command_ = first_command;
  have_last_command_ = true;
  const auto command_fake = rotate(-yaw, last_command_);
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now();
  command.header.frame_id = base_frame_;
  command.twist.linear.x = command_fake.x();
  command.twist.linear.y = command_fake.y();
  command.twist.angular.z = 0.0; // fake_baselink currently discards angular.z.
  return command;
}
} // namespace nav_executor
