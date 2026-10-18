#include "nav_executor/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <qpOASES.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_executor
{
namespace
{
double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{return a.x() * b.y() - a.y() * b.x();}

Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d & v)
{return {std::cos(yaw) * v.x() - std::sin(yaw) * v.y(),
    std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};}
}  // namespace

void MpcController::initialize(const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf, map_server::Costmap2D * costmap,
  std::string global_frame, std::string base_frame, double robot_radius)
{
  node_ = node; tf_ = std::move(tf); costmap_ = costmap;
  global_frame_ = std::move(global_frame); base_frame_ = std::move(base_frame);
  robot_radius_ = robot_radius;
  allow_unknown_ = node->declare_parameter("MPC.allow_unknown", false);
  dt_ = node->declare_parameter("MPC.dt", 0.05);
  horizon_ = node->declare_parameter("MPC.horizon", 10);
  max_speed_ = node->declare_parameter("MPC.max_speed", 2.0);
  max_acc_ = node->declare_parameter("MPC.max_acc", 2.0);
  max_lateral_acc_ = node->declare_parameter("MPC.max_lateral_acc", 1.0);
  q_along_ = node->declare_parameter("MPC.q_along", 3.0);
  q_cross_ = node->declare_parameter("MPC.q_cross", 12.0);
  r_speed_ = node->declare_parameter("MPC.r_speed", 1.5);
  delay_ = node->declare_parameter("MPC.control_delay", 0.05);
  max_working_set_recalculations_ = node->declare_parameter("MPC.max_working_set_recalculations", 200);
  if (!node_ || !tf_ || !costmap_ || global_frame_.empty() || base_frame_.empty() ||
    !std::isfinite(dt_) || dt_ <= 0 || dt_ > 0.2 || horizon_ < 1 || horizon_ > 40 ||
    !std::isfinite(max_speed_) || max_speed_ <= 0 ||
    !std::isfinite(max_acc_) || max_acc_ <= 0 ||
    !std::isfinite(max_lateral_acc_) || max_lateral_acc_ <= 0 ||
    !std::isfinite(q_along_) || q_along_ <= 0 ||
    !std::isfinite(q_cross_) || q_cross_ <= 0 ||
    !std::isfinite(r_speed_) || r_speed_ <= 0 ||
    !std::isfinite(delay_) || delay_ < 0 || delay_ > 0.2 ||
    !std::isfinite(robot_radius_) || robot_radius_ <= 0 ||
    max_working_set_recalculations_ < 1)
  {throw std::invalid_argument("Invalid MPC configuration");}
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty())
  {throw std::invalid_argument("MPC requires a nonempty framed path");}
  reset(); plan_ = path;
}

void MpcController::reset()
{plan_.poses.clear(); progress_ = 0; last_command_.setZero(); have_last_command_ = false;}

std::vector<MpcController::PathPoint> MpcController::transformPlan(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TransformStamped transform;
  const bool change_frame = plan_.header.frame_id != global_frame_;
  if (change_frame) {
    transform = tf_->lookupTransform(global_frame_, plan_.header.frame_id, tf2::TimePointZero);
  }
  std::vector<PathPoint> path;
  path.reserve(plan_.poses.size() + 1);
  for (const auto & waypoint : plan_.poses) {
    geometry_msgs::msg::PoseStamped converted = waypoint;
    converted.header.frame_id = plan_.header.frame_id;
    if (change_frame) {
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(converted, transformed, transform);
      converted = std::move(transformed);
    }
    Eigen::Vector2d point(converted.pose.position.x, converted.pose.position.y);
    if (!point.allFinite()) {throw std::runtime_error("Invalid transformed MPC path");}
    if (!path.empty() && (point - path.back().point).norm() < 1e-5) {continue;}
    const double arc = path.empty() ? 0.0 : path.back().arc + (point - path.back().point).norm();
    path.push_back({point, arc});
  }
  if (path.size() == 1) {
    const Eigen::Vector2d current(pose.pose.position.x, pose.pose.position.y);
    path.insert(path.begin(), {current, 0.0});
    path.back().arc = (path.back().point - current).norm();
  }
  if (path.size() < 2 || path.back().arc < 1e-5)
  {throw std::runtime_error("MPC path has no usable length");}
  return path;
}

MpcController::Sample MpcController::sample(double arc, const std::vector<PathPoint> & path) const
{
  const double s = std::clamp(arc, 0.0, path.back().arc);
  auto upper = std::upper_bound(path.begin(), path.end(), s,
    [](double value, const PathPoint & item) {return value < item.arc;});
  size_t i = std::clamp<size_t>(static_cast<size_t>(upper - path.begin()), 1, path.size() - 1);
  const auto & a = path[i - 1]; const auto & b = path[i];
  const double ds = b.arc - a.arc;
  const Eigen::Vector2d tangent = (b.point - a.point) / ds;
  const Eigen::Vector2d point = a.point + tangent * (s - a.arc);
  double curvature = 0.0;
  // Estimate curvature at nearby vertices, including an A* right-angle fallback.
  for (size_t vertex : {i - 1, i}) {
    if (vertex == 0 || vertex + 1 >= path.size()) {continue;}
    const auto before = (path[vertex].point - path[vertex - 1].point).normalized();
    const auto after = (path[vertex + 1].point - path[vertex].point).normalized();
    const double turn = std::abs(std::atan2(cross(before, after), before.dot(after)));
    const double scale = std::max(0.05,
      0.5 * (path[vertex + 1].arc - path[vertex - 1].arc));
    if (std::abs(s - path[vertex].arc) < std::max(0.2, ds))
    {curvature = std::max(curvature, turn / scale);}
  }
  const double remaining = path.back().arc - s;
  const double turn_speed = std::sqrt(max_lateral_acc_ / std::max(curvature, 1e-6));
  const double stop_speed = std::sqrt(2.0 * max_acc_ * remaining);
  const double speed = std::min({max_speed_, turn_speed, stop_speed});
  return {point, tangent, speed};
}

bool MpcController::rolloutClear(const std::vector<Eigen::Vector2d> & predicted) const
{
  const map_server::Costmap2D map(*costmap_);
  if (map.getSizeInCellsX() == 0 || predicted.empty()) {return false;}
  const double resolution = map.getResolution();
  const double check_radius = robot_radius_;
  for (size_t i = 1; i < predicted.size(); ++i) {
    const auto & a = predicted[i - 1]; const auto & b = predicted[i];
    const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm() / (resolution * 0.5))));
    for (int j = 0; j <= steps; ++j) {
      const Eigen::Vector2d point = a + (b - a) * (static_cast<double>(j) / steps);
      unsigned int mx, my;
      if (!map.worldToMap(point.x(), point.y(), mx, my)) {return false;}
      const auto center_cost = map.getCost(mx, my);
      if (center_cost == map_server::INSCRIBED_INFLATED_OBSTACLE ||
        (!allow_unknown_ && center_cost == map_server::NO_INFORMATION)) {return false;}
      const int cells = static_cast<int>(std::ceil(check_radius / resolution));
      for (int y = std::max(0, static_cast<int>(my) - cells);
        y <= std::min(static_cast<int>(map.getSizeInCellsY()) - 1, static_cast<int>(my) + cells); ++y)
      {
        for (int x = std::max(0, static_cast<int>(mx) - cells);
          x <= std::min(static_cast<int>(map.getSizeInCellsX()) - 1, static_cast<int>(mx) + cells); ++x)
        {
          const double cx = map.getOriginX() + (x + 0.5) * resolution;
          const double cy = map.getOriginY() + (y + 0.5) * resolution;
          if (std::hypot(cx - point.x(), cy - point.y()) > check_radius + resolution * std::sqrt(0.5))
          {continue;}
          const auto cost = map.getCost(x, y);
          if (cost == map_server::LETHAL_OBSTACLE ||
            (!allow_unknown_ && cost == map_server::NO_INFORMATION)) {return false;}
        }
      }
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & measured_speed)
{
  if (plan_.poses.empty()) {throw std::runtime_error("MPC has no path");}
  if (pose.header.frame_id != global_frame_) {throw std::runtime_error("MPC pose frame mismatch");}
  const auto path = transformPlan(pose);
  const Eigen::Vector2d p(pose.pose.position.x, pose.pose.position.y);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const Eigen::Vector2d measured_world = rotate(yaw,
    {measured_speed.linear.x, measured_speed.linear.y});
  if (!p.allFinite() || !measured_world.allFinite() || !std::isfinite(yaw) ||
    !std::isfinite(measured_speed.angular.z))
  {throw std::runtime_error("Invalid MPC odometry state");}
  // Projection advances monotonically within one accepted plan.
  double best_distance = INFINITY, best_arc = progress_;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto segment = path[i].point - path[i - 1].point;
    const double length2 = segment.squaredNorm();
    const double ratio = std::clamp((p - path[i - 1].point).dot(segment) / length2, 0.0, 1.0);
    const double candidate = path[i - 1].arc + ratio * std::sqrt(length2);
    if (candidate + 0.05 < progress_) {continue;}
    const double distance = (p - path[i - 1].point - ratio * segment).squaredNorm();
    if (distance < best_distance) {best_distance = distance; best_arc = candidate;}
  }
  progress_ = std::max(progress_, best_arc);
  const int nv = 2 * horizon_;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::VectorXd reference(nv), feedforward(nv), initial(nv);
  double ref_arc = progress_;
  for (int k = 0; k < horizon_; ++k) {
    auto at = sample(ref_arc, path);
    ref_arc = std::min(path.back().arc, ref_arc + std::max(0.1, at.speed) * dt_);
    at = sample(ref_arc, path);
    reference.segment<2>(2 * k) = at.point;
    feedforward.segment<2>(2 * k) = at.tangent * at.speed;
    initial.segment<2>(2 * k) = p + measured_world * delay_;
    const Eigen::Vector2d normal(-at.tangent.y(), at.tangent.x());
    Q.block<2, 2>(2 * k, 2 * k) = q_along_ * at.tangent * at.tangent.transpose() +
      q_cross_ * normal * normal.transpose();
    for (int j = 0; j <= k; ++j) {B.block<2, 2>(2 * k, 2 * j).setIdentity();
      B.block<2, 2>(2 * k, 2 * j) *= dt_;}
  }
  Eigen::MatrixXd H = 2.0 * (B.transpose() * Q * B + r_speed_ * Eigen::MatrixXd::Identity(nv, nv));
  H.diagonal().array() += 1e-8;
  const Eigen::VectorXd g = 2.0 * (B.transpose() * Q * (initial - reference) - r_speed_ * feedforward);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::VectorXd lbA(nv), ubA(nv);
  Eigen::Vector2d previous = have_last_command_ ? last_command_ : measured_world;
  previous.x() = std::clamp(previous.x(), -max_speed_, max_speed_);
  previous.y() = std::clamp(previous.y(), -max_speed_, max_speed_);
  for (int k = 0; k < horizon_; ++k) {
    for (int axis = 0; axis < 2; ++axis) {
      const int row = 2 * k + axis;
      A(row, row) = 1.0;
      if (k > 0) {A(row, row - 2) = -1.0;}
      const double anchor = k == 0 ? previous[axis] : 0.0;
      lbA[row] = anchor - max_acc_ * dt_;
      ubA[row] = anchor + max_acc_ * dt_;
    }
  }
  // qpOASES accepts row-major dense arrays.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Hr = H, Ar = A;
  std::vector<qpOASES::real_t> h(Hr.data(), Hr.data() + Hr.size());
  std::vector<qpOASES::real_t> a(Ar.data(), Ar.data() + Ar.size());
  std::vector<qpOASES::real_t> grad(g.data(), g.data() + g.size());
  std::vector<qpOASES::real_t> lower(nv, -max_speed_), upper(nv, max_speed_);
  std::vector<qpOASES::real_t> lower_a(lbA.data(), lbA.data() + lbA.size());
  std::vector<qpOASES::real_t> upper_a(ubA.data(), ubA.data() + ubA.size());
  qpOASES::QProblem problem(nv, nv);
  qpOASES::Options options; options.setToMPC(); options.printLevel = qpOASES::PL_NONE;
  problem.setOptions(options);
  qpOASES::int_t work = max_working_set_recalculations_;
  const auto status = problem.init(h.data(), grad.data(), a.data(), lower.data(), upper.data(),
    lower_a.data(), upper_a.data(), work);
  if (status != qpOASES::SUCCESSFUL_RETURN)
  {throw std::runtime_error("MPC qpOASES solve failed: " + std::to_string(static_cast<int>(status)));}
  std::vector<qpOASES::real_t> solution(nv);
  if (problem.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN)
  {throw std::runtime_error("MPC solution unavailable");}
  std::vector<Eigen::Vector2d> predicted;
  predicted.reserve(horizon_ + 1);
  predicted.push_back(p);
  predicted.push_back(p + measured_world * delay_);
  Eigen::Vector2d first_command = Eigen::Vector2d::Zero();
  for (int k = 0; k < horizon_; ++k) {
    Eigen::Vector2d u(solution[2 * k], solution[2 * k + 1]);
    if (!u.allFinite()) {throw std::runtime_error("MPC produced nonfinite velocity");}
    if (u.norm() > max_speed_) {u *= max_speed_ / u.norm();}
    predicted.push_back(predicted.back() + u * dt_);
    if (k == 0) {first_command = u;}
  }
  if (!rolloutClear(predicted)) {throw std::runtime_error("MPC predicted path intersects costmap");}
  last_command_ = first_command;
  have_last_command_ = true;
  const auto command_fake = rotate(-yaw, last_command_);
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now(); command.header.frame_id = base_frame_;
  command.twist.linear.x = command_fake.x();
  command.twist.linear.y = command_fake.y();
  command.twist.angular.z = 0.0;  // fake_baselink currently discards angular.z.
  return command;
}
}  // namespace nav_executor
