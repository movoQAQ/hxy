#include "nav_executor/mpc_controller.hpp"
#include "nav_executor/mpc/geometry.hpp"
#include "solver_constants.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
extern "C"
{
#include "acados_solver_omni_mpc.h"
}

namespace nav_executor
{
namespace
{
using Clock = std::chrono::steady_clock;
namespace g = mpc::generated;
using Vector = Eigen::Vector2d;
using Params = std::array<double, g::ParameterCount>;
Vector limit(Vector v, double maximum)
{
  const double norm = v.norm();
  return norm > maximum ? (v * (maximum / norm)).eval() : v;
}
double rho(const Vector &v) { return std::sqrt(v.squaredNorm() + 1e-6) - 1e-3; }
} // namespace

class MpcController::Impl
{
public:
  ~Impl() { destroy(); }
  void destroy()
  {
    if (capsule)
    {
      omni_mpc_acados_free(capsule);
      omni_mpc_acados_free_capsule(capsule);
      capsule = nullptr;
    }
  }
  double parameter(const std::string &name, double fallback, bool zero = false)
  {
    const auto owner = node.lock();
    if (!owner)
    {
      throw std::runtime_error("INVALID_INPUT: controller node expired");
    }
    const double value = owner->declare_parameter("MPC." + name, fallback);
    if (!std::isfinite(value) || (zero ? value < 0 : value <= 0))
    {
      throw std::invalid_argument("INVALID_CONFIG: MPC." + name);
    }
    return value;
  }
  void initialize(const rclcpp::Node::SharedPtr &n, std::shared_ptr<tf2_ros::Buffer> t,
                  map_server::Costmap2D *map, std::string frame, std::string base, double radius)
  {
    if (capsule || !n || !t || !map || frame.empty() || base.empty() || !std::isfinite(radius) ||
        radius <= 0)
    {
      throw std::invalid_argument("INVALID_CONFIG: NMPC initialization");
    }
    node = n;
    clock = n->get_clock();
    logger = n->get_logger();
    tf = std::move(t);
    costmap = map;
    global_frame = std::move(frame);
    base_frame = std::move(base);
    allow_unknown = n->declare_parameter("MPC.allow_unknown", false);
    allow_detours = n->has_parameter("MPC.allow_detours")
                        ? n->get_parameter("MPC.allow_detours").as_bool()
                        : n->declare_parameter("MPC.allow_detours", true);
    const double dt = parameter("dt", g::Dt);
    const int horizon = n->declare_parameter("MPC.horizon", g::Horizon);
    const int capacity = n->declare_parameter("MPC.max_obstacles", g::MaxObstacles);
    const double frequency = n->has_parameter("controller_frequency")
                                 ? n->get_parameter("controller_frequency").as_double()
                                 : 1.0 / g::Dt;
    if (std::abs(dt - g::Dt) > 1e-9 || horizon != g::Horizon || capacity != g::MaxObstacles ||
        !std::isfinite(frequency) || std::abs(frequency * g::Dt - 1.0) > 1e-6)
    {
      throw std::invalid_argument(
          "SOLVER_MANIFEST_MISMATCH: dt/horizon/obstacles/control frequency");
    }
    max_speed = parameter("max_speed", 2.0);
    max_change = parameter("max_acc", 2.0);
    accel = parameter("max_accel", 2.0);
    brake = parameter("max_brake", 2.0);
    tracking_acc = parameter("max_tracking_acc", 2.0);
    lateral_acc = parameter("max_lateral_acc", 1.0);
    delay = parameter("control_delay", 0.05, true);
    if (delay > 0.2)
    {
      throw std::invalid_argument("INVALID_CONFIG: control_delay > 0.2s");
    }
    q_cross = parameter("q_cross", 12.0);
    q_along = parameter("q_along", 3.0);
    r_speed = parameter("r_speed", 1.5);
    r_delta = parameter("r_delta_velocity", 1.0, true);
    r_brake = parameter("r_brake", 0.0, true);
    r_direction = parameter("r_direction", 0.0, true);
    q_obstacle = parameter("q_obstacle", 30.0, true);
    soft_distance = parameter("soft_distance", 0.25);
    terminal_weight = parameter("terminal_weight", 3.0);
    terminal_stop = parameter("terminal_stop", 8.0);
    tolerance = parameter("constraint_tolerance", 0.0002);
    if (tolerance > 0.001)
    {
      throw std::invalid_argument("INVALID_CONFIG: constraint_tolerance > 0.001");
    }
    budget_ms = parameter("solver_budget_ms", 25.0);
    if (budget_ms > 40.0)
    {
      throw std::invalid_argument("INVALID_CONFIG: solver budget > 40ms");
    }
    max_deviation = parameter("max_deviation", 1.0);
    if (!allow_detours)
    {
      max_deviation = std::min(max_deviation, 0.05);
    }
    safety_radius = radius + parameter("safety_margin", 0.05, true);
    capsule = omni_mpc_acados_create_capsule();
    if (!capsule)
    {
      throw std::runtime_error("SOLVER_CREATE_FAILED: capsule");
    }
    const int status = omni_mpc_acados_create(capsule);
    if (status != 0)
    {
      omni_mpc_acados_free_capsule(capsule);
      capsule = nullptr;
      throw std::runtime_error("SOLVER_CREATE_FAILED: " + std::to_string(status));
    }
    config = omni_mpc_acados_get_nlp_config(capsule);
    dims = omni_mpc_acados_get_nlp_dims(capsule);
    input = omni_mpc_acados_get_nlp_in(capsule);
    output = omni_mpc_acados_get_nlp_out(capsule);
    solver = omni_mpc_acados_get_nlp_solver(capsule);
    // Leave a scheduling and final-iteration reserve before the hard elapsed-time gate.
    double timeout = 0.8 * budget_ms / 1000.0;
    ocp_nlp_solver_opts_set(config, omni_mpc_acados_get_nlp_opts(capsule), "timeout_max_time",
                            &timeout);
    parameter_guard = n->add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &values)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &value : values)
          {
            if (value.get_name().rfind("MPC.", 0) == 0)
            {
              result.successful = false;
              result.reason = "NMPC parameters are startup-only; edit config/mpc.yaml and restart";
              break;
            }
          }
          return result;
        });
    RCLCPP_INFO(n->get_logger(),
                "Active controller: Acados omni NMPC N=%d dt=%.3f, old QP archived", g::Horizon,
                g::Dt);
  }
  void reset()
  {
    plan.poses.clear();
    progress = 0;
    warm.clear();
    last_published.setZero();
    have_published = false;
    last_result.clear();
    last_check.clear();
    feedback = {};
    snapshot.reset();
    published_at = {};
    last_solve_at = {};
    if (capsule)
    {
      omni_mpc_acados_reset(capsule, 1, 1, 0, 0);
    }
  }
  std::vector<mpc::PathPoint> pathInFrame()
  {
    geometry_msgs::msg::TransformStamped transform;
    const bool change = plan.header.frame_id != global_frame;
    if (change)
    {
      transform = tf->lookupTransform(global_frame, plan.header.frame_id, tf2::TimePointZero);
    }
    std::vector<Vector> points;
    for (const auto &waypoint : plan.poses)
    {
      geometry_msgs::msg::PoseStamped point = waypoint;
      point.header.frame_id = plan.header.frame_id;
      if (change)
      {
        geometry_msgs::msg::PoseStamped converted;
        tf2::doTransform(point, converted, transform);
        point = converted;
      }
      points.emplace_back(point.pose.position.x, point.pose.position.y);
    }
    return mpc::makeSpeedProfile(points, max_speed, lateral_acc, accel, brake);
  }
  std::vector<Vector> brakingTail(Vector point, Vector velocity) const
  {
    std::vector<Vector> tail{point};
    // Use the slower of vector-change and scalar braking limits.
    const double step = std::min(brake, max_change) * g::Dt;
    const int count = static_cast<int>(std::ceil(velocity.norm() / step)) + 1;
    if (count > 2000)
    {
      throw std::runtime_error("INVALID_CONFIG: excessive braking tail");
    }
    for (int k = 0; k < count && velocity.norm() > 1e-8; ++k)
    {
      // Conservatively retain the incoming speed for a full sample before braking.
      point += velocity * g::Dt;
      tail.push_back(point);
      velocity = limit(velocity, std::max(0.0, velocity.norm() - step));
    }
    return tail;
  }
  void validate(const map_server::Costmap2D &map, const std::vector<Vector> &check) const
  {
    // The extra quarter cell covers gaps between samples no more than res/2 apart.
    if (!mpc::trajectoryClear(map, check, safety_radius + map.getResolution() * 0.25,
                              allow_unknown))
    {
      throw std::runtime_error("COLLISION_OR_OUTSIDE_MAP: prediction/delay/braking tail");
    }
  }
  Params parameters(const mpc::Reference &ref, const Vector &measured,
                    const map_server::Costmap2D &map, bool first, bool terminal, bool stopping,
                    const std::vector<mpc::Circle> &circles) const
  {
    Params p{};
    p[g::REF_X] = ref.position.x();
    p[g::REF_Y] = ref.position.y();
    p[g::TANGENT_X] = ref.tangent.x();
    p[g::TANGENT_Y] = ref.tangent.y();
    p[g::VREF] = ref.speed;
    p[g::Q_CROSS] = q_cross;
    p[g::Q_ALONG] = q_along;
    p[g::R_SPEED] = r_speed;
    p[g::R_DELTA] = r_delta;
    p[g::R_BRAKE] = r_brake;
    p[g::R_DIRECTION] = r_direction;
    p[g::Q_OBSTACLE] = q_obstacle;
    p[g::TERMINAL_WEIGHT] = terminal_weight;
    p[g::TERMINAL_STOP] = terminal && stopping ? terminal_stop : 0.0;
    p[g::MEASURED_X] = measured.x();
    p[g::MEASURED_Y] = measured.y();
    p[g::TRACK_ACTIVE] = first ? 1 : 0;
    p[g::MAX_SPEED] = max_speed;
    p[g::MAX_CHANGE] = max_change;
    p[g::MAX_ACCEL] = accel;
    p[g::MAX_BRAKE] = brake;
    p[g::MAX_TRACKING_ACC] = tracking_acc;
    const double radius = safety_radius + map.getResolution() * 0.25;
    p[g::MIN_X] = map.getOriginX() + radius;
    p[g::MAX_X] = map.getOriginX() + map.getSizeInCellsX() * map.getResolution() - radius;
    p[g::MIN_Y] = map.getOriginY() + radius;
    p[g::MAX_Y] = map.getOriginY() + map.getSizeInCellsY() * map.getResolution() - radius;
    p[g::SOFT_DISTANCE] = soft_distance;
    p[g::BRAKE_GATE] = stopping || ref.speed < measured.norm() - 0.05 ? 0.0 : 1.0;
    for (size_t j = 0; j < circles.size(); ++j)
    {
      const int offset = g::OBSTACLES + 4 * j;
      p[offset] = circles[j].center.x();
      p[offset + 1] = circles[j].center.y();
      p[offset + 2] = circles[j].radius;
      p[offset + 3] = 1;
    }
    return p;
  }
  std::vector<Vector> seed(const Vector &initial, const Vector &measured,
                           const std::vector<mpc::Reference> &reference,
                           const std::vector<mpc::Circle> &obstacles)
  {
    std::vector<Vector> best;
    double best_score = INFINITY;
    const auto now = Clock::now();
    int shift = 1;
    if (last_solve_at.time_since_epoch().count())
    {
      shift = std::max(1, static_cast<int>(std::round(
                              std::chrono::duration<double>(now - last_solve_at).count() / g::Dt)));
    }
    const bool use_warm = warm.size() == g::Horizon && shift < 5;
    for (double offset : {0.0, 0.6, -0.6, 1.0, -1.0})
    {
      if (!allow_detours && offset != 0)
      {
        continue;
      }
      Vector pos = initial, previous = have_published ? last_published : measured;
      std::vector<Vector> controls;
      double score = 0;
      for (int k = 0; k < g::Horizon; ++k)
      {
        const auto &ref = reference[k + 1];
        const Vector normal(-ref.tangent.y(), ref.tangent.x());
        const Vector target =
            ref.position + normal * offset * std::sin(M_PI * (k + 1) / g::Horizon);
        Vector desired = limit(ref.tangent * ref.speed + 2.0 * (target - pos), max_speed);
        if (use_warm && offset == 0)
        {
          desired = warm[std::min(k + shift, g::Horizon - 1)];
        }
        const Vector u = previous + limit(desired - previous, max_change * g::Dt);
        controls.push_back(u);
        pos += u * g::Dt;
        previous = u;
        score += 0.01 * (pos - ref.position).squaredNorm();
        for (const auto &obstacle : obstacles)
        {
          const double intrusion =
              std::max(0.0, obstacle.radius + 0.05 - (pos - obstacle.center).norm());
          score += 1000 * intrusion * intrusion;
        }
      }
      if (score < best_score)
      {
        best_score = score;
        best = std::move(controls);
      }
    }
    return best;
  }
  geometry_msgs::msg::TwistStamped compute(const geometry_msgs::msg::PoseStamped &pose,
                                           const geometry_msgs::msg::Twist &speed)
  {
    feedback = {};
    const auto began = Clock::now();
    if (!capsule || plan.poses.empty() || pose.header.frame_id != global_frame)
    {
      throw std::runtime_error("INVALID_INPUT: solver/path/frame");
    }
    // The executor supplies coherent metadata; direct unit callers still use a locked grid copy.
    auto current_snapshot = std::exchange(snapshot, {});
    const map_server::Costmap2D map = current_snapshot ? current_snapshot->grid : *costmap;
    if (current_snapshot && !current_snapshot->current)
    {
      throw std::runtime_error("STALE_INPUT: costmap");
    }
    const Vector actual(pose.pose.position.x, pose.pose.position.y);
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const Vector measured = mpc::rotate(yaw, {speed.linear.x, speed.linear.y});
    if (!actual.allFinite() || !measured.allFinite() || !std::isfinite(yaw) ||
        measured.norm() > max_speed + tolerance)
    {
      throw std::runtime_error("INVALID_INPUT: pose/velocity outside model envelope");
    }
    const auto now = Clock::now();
    if (last_solve_at.time_since_epoch().count() &&
        std::chrono::duration<double>(now - last_solve_at).count() > 0.2)
    {
      warm.clear();
      have_published = false;
    }
    const auto ros_now = clock->now();
    if (last_ros_stamp.nanoseconds() > 0 && ros_now < last_ros_stamp)
    {
      warm.clear();
      have_published = false;
    }
    last_ros_stamp = ros_now;
    auto path = pathInFrame();
    progress = mpc::projectProgress(path, actual, progress, max_speed * 0.2 + 0.5);
    if (mpc::pathDistance(path, actual) > max_deviation + tolerance)
    {
      throw std::runtime_error("PATH_DEVIATION: outside local planning tube");
    }
    const Vector initial = actual + measured * delay;
    validate(map, {actual, initial});
    const double coverage_radius = safety_radius + map.getResolution() * 0.25;
    const auto obstacles =
        mpc::coverObstacles(map, allow_unknown, coverage_radius, g::MaxObstacles);
    double arc = progress;
    std::vector<mpc::Reference> refs;
    refs.push_back(mpc::sampleReference(path, arc));
    double achievable = measured.norm();
    for (int k = 0; k < g::Horizon; ++k)
    {
      const auto at = mpc::sampleReference(path, arc);
      achievable = std::clamp(at.speed, std::max(0.0, achievable - brake * g::Dt),
                              achievable + accel * g::Dt);
      arc = std::min(path.back().arc, arc + std::max(0.05, achievable) * g::Dt);
      auto next = mpc::sampleReference(path, arc);
      next.speed = std::min(next.speed, achievable);
      refs.push_back(next);
    }
    const bool stopping =
        path.back().arc - progress <= std::max(0.2, measured.norm() * g::Horizon * g::Dt + 0.5);
    const Vector memory = have_published ? last_published : measured;
    if ((memory - measured).norm() > (max_change + tracking_acc) * g::Dt + 2 * tolerance)
    {
      throw std::runtime_error(
          "TRACKING_DIVERGENCE: measured/command constraint intersection empty");
    }
    auto guess = seed(initial, measured, refs, obstacles);
    std::array<double, 4> state{initial.x(), initial.y(), memory.x(), memory.y()};
    ocp_nlp_constraints_model_set(config, dims, input, output, 0, "lbx", state.data());
    ocp_nlp_constraints_model_set(config, dims, input, output, 0, "ubx", state.data());
    for (int k = 0; k <= g::Horizon; ++k)
    {
      auto p = parameters(refs[k], measured, map, k == 0, k == g::Horizon, stopping, obstacles);
      // At identical measured/command anchors the first vector constraint is
      // exactly the same constraint; keep only the stricter one in the NLP.
      if (k == 0 && (memory - measured).norm() < 1e-9 && tracking_acc >= max_change)
      {
        p[g::TRACK_ACTIVE] = 0;
      }
      omni_mpc_acados_update_params(capsule, k, p.data(), p.size());
      ocp_nlp_out_set(config, dims, output, input, k, "x", state.data());
      if (k < g::Horizon)
      {
        double u[2] = {guess[k].x(), guess[k].y()};
        ocp_nlp_out_set(config, dims, output, input, k, "u", u);
        state[0] += g::Dt * u[0];
        state[1] += g::Dt * u[1];
        state[2] = u[0];
        state[3] = u[1];
      }
    }
    const auto solve_began = Clock::now();
    const int status = omni_mpc_acados_solve(capsule);
    feedback.solve_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - solve_began).count();
    last_solve_at = Clock::now();
    if (feedback.solve_ms > budget_ms)
    {
      warm.clear();
      throw std::runtime_error("DEADLINE_EXCEEDED: NMPC solve");
    }
    feedback.solver_status = status;
    feedback.converged = status == 0;
    // A bounded SQP iterate is usable only after all independent physical checks.
    if (status != 0 && status != 2 && status != 7)
    {
      int iterations = 0;
      ocp_nlp_get(solver, "sqp_iter", &iterations);
      double stat = 0, ineq = 0;
      ocp_nlp_get(solver, "res_stat", &stat);
      ocp_nlp_get(solver, "res_ineq", &ineq);
      warm.clear();
      throw std::runtime_error("SOLVE_FAILED: Acados status=" + std::to_string(status) +
                               " iterations=" + std::to_string(iterations) +
                               " ms=" + std::to_string(feedback.solve_ms) +
                               " stat=" + std::to_string(stat) + " ineq=" + std::to_string(ineq));
    }
    double res_eq = 0, res_ineq = 0;
    ocp_nlp_get(solver, "res_eq", &res_eq);
    ocp_nlp_get(solver, "res_ineq", &res_ineq);
    if (!std::isfinite(res_eq) || !std::isfinite(res_ineq) || res_eq > tolerance)
    {
      throw std::runtime_error("CONSTRAINT_VIOLATION: solver residual");
    }
    std::vector<Vector> controls, positions{initial};
    Vector previous = memory;
    Vector raw_position = initial;
    double deviation = 0;
    for (int k = 0; k < g::Horizon; ++k)
    {
      double raw[2];
      ocp_nlp_out_get(config, dims, output, k, "u", raw);
      const Vector raw_u(raw[0], raw[1]);
      if (!raw_u.allFinite())
      {
        throw std::runtime_error("NUMERICAL_FAILURE: nonfinite control");
      }
      raw_position += raw_u * g::Dt;
      double raw_state[4];
      ocp_nlp_out_get(config, dims, output, k + 1, "x", raw_state);
      const Eigen::Map<Eigen::Vector4d> solved(raw_state);
      const Eigen::Vector4d integrated(raw_position.x(), raw_position.y(), raw_u.x(), raw_u.y());
      if (!solved.allFinite() || (solved - integrated).lpNorm<Eigen::Infinity>() > tolerance)
      {
        throw std::runtime_error("CONSTRAINT_VIOLATION: independent dynamics check");
      }
      // Restore only small numerical violations. The entire changed trajectory is
      // re-integrated and rechecked below; an unsafe or materially different NLP
      // iterate is never made publishable by an unverified output clamp.
      Vector u = raw_u;
      for (int iteration = 0; iteration < 20; ++iteration)
      {
        u = limit(u, max_speed);
        u = previous + limit(u - previous, max_change * g::Dt);
        if (k == 0)
        {
          u = measured + limit(u - measured, tracking_acc * g::Dt);
        }
        const double low = std::max(0.0, rho(previous) - brake * g::Dt);
        const double high = rho(previous) + accel * g::Dt;
        const double desired = std::clamp(rho(u), low, high);
        const double norm = std::sqrt(std::max(0.0, (desired + 1e-3) * (desired + 1e-3) - 1e-6));
        if (u.norm() > 1e-10)
        {
          u *= norm / u.norm();
        }
        else if (norm > 0 && previous.norm() > 1e-10)
        {
          u = previous.normalized() * norm;
        }
      }
      if ((u - raw_u).norm() > 0.005)
      {
        throw std::runtime_error(
            "CONSTRAINT_VIOLATION: excessive feasibility restoration k=" + std::to_string(k) +
            " correction=" + std::to_string((u - raw_u).norm()) +
            " status=" + std::to_string(status));
      }
      feedback.restored = feedback.restored || (u - raw_u).norm() > 1e-8;
      if (u.norm() > max_speed + tolerance ||
          (u - previous).norm() > max_change * g::Dt + tolerance ||
          rho(u) - rho(previous) > accel * g::Dt + tolerance ||
          rho(previous) - rho(u) > brake * g::Dt + tolerance ||
          (k == 0 && (u - measured).norm() > tracking_acc * g::Dt + tolerance))
      {
        throw std::runtime_error("CONSTRAINT_VIOLATION: independent velocity check");
      }
      controls.push_back(u);
      positions.push_back(positions.back() + u * g::Dt);
      previous = u;
      for (const auto &obstacle : obstacles)
      {
        if ((positions.back() - obstacle.center).norm() < obstacle.radius - tolerance)
        {
          throw std::runtime_error("CONSTRAINT_VIOLATION: independent obstacle check");
        }
      }
      deviation = std::max(deviation, mpc::pathDistance(path, positions.back()));
      if (deviation > max_deviation + tolerance)
      {
        throw std::runtime_error("PATH_DEVIATION: predicted tube exit");
      }
    }
    last_check = {actual};
    last_check.insert(last_check.end(), positions.begin(), positions.end());
    const auto tail = brakingTail(positions.back(), controls.back());
    last_check.insert(last_check.end(), tail.begin() + 1, tail.end());
    validate(map, last_check);
    feedback.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
    if (status != 0)
    {
      RCLCPP_DEBUG(logger, "NMPC accepted feasible SQP iterate: status=%d solve_ms=%.3f", status,
                   feedback.solve_ms);
    }
    feedback.valid = true;
    feedback.detouring = allow_detours && deviation > 0.15;
    feedback.progress = progress;
    feedback.map_version = current_snapshot ? current_snapshot->version : 0;
    warm = controls;
    last_result = positions;
    candidate = controls.front();
    candidate_yaw = yaw;
    const Vector fake = mpc::rotate(-yaw, candidate);
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = clock->now();
    command.header.frame_id = base_frame;
    command.twist.linear.x = fake.x();
    command.twist.linear.y = fake.y();
    return command;
  }
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard;
  rclcpp::Node::WeakPtr node;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Logger logger{rclcpp::get_logger("nmpc")};
  std::shared_ptr<tf2_ros::Buffer> tf;
  map_server::Costmap2D *costmap{nullptr};
  std::shared_ptr<const map_server::CostmapSnapshot> snapshot;
  std::string global_frame, base_frame;
  nav_msgs::msg::Path plan;
  bool allow_unknown{false}, allow_detours{true}, have_published{false};
  double max_speed{}, max_change{}, accel{}, brake{}, tracking_acc{}, lateral_acc{}, delay{};
  double q_cross{}, q_along{}, r_speed{}, r_delta{}, r_brake{}, r_direction{}, q_obstacle{},
      soft_distance{};
  double terminal_weight{}, terminal_stop{}, tolerance{}, budget_ms{}, max_deviation{},
      safety_radius{}, progress{0};
  Vector last_published{0, 0}, candidate{0, 0};
  double candidate_yaw{0};
  std::vector<Vector> warm, last_result, last_check;
  Clock::time_point last_solve_at{}, published_at{};
  rclcpp::Time last_ros_stamp{0, 0, RCL_ROS_TIME};
  MpcFeedback feedback;
  omni_mpc_solver_capsule *capsule{nullptr};
  ocp_nlp_config *config{nullptr};
  ocp_nlp_dims *dims{nullptr};
  ocp_nlp_in *input{nullptr};
  ocp_nlp_out *output{nullptr};
  ocp_nlp_solver *solver{nullptr};
};

MpcController::MpcController() : impl_(std::make_unique<Impl>()) {}
MpcController::~MpcController() = default;
void MpcController::initialize(const rclcpp::Node::SharedPtr &node,
                               std::shared_ptr<tf2_ros::Buffer> tf, map_server::Costmap2D *map,
                               std::string frame, std::string base, double radius)
{
  impl_->initialize(node, std::move(tf), map, std::move(frame), std::move(base), radius);
}
void MpcController::reset() { impl_->reset(); }
void MpcController::setPlan(const nav_msgs::msg::Path &path)
{
  if (path.poses.empty() || path.header.frame_id.empty())
  {
    throw std::invalid_argument("NO_REFERENCE: empty path/frame");
  }
  for (const auto &waypoint : path.poses)
  {
    const auto &p = waypoint.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        (!waypoint.header.frame_id.empty() && waypoint.header.frame_id != path.header.frame_id))
    {
      throw std::invalid_argument("NO_REFERENCE: invalid position/frame");
    }
  }
  impl_->reset();
  impl_->plan = path;
}
void MpcController::setSnapshot(std::shared_ptr<const map_server::CostmapSnapshot> snapshot)
{
  impl_->snapshot = std::move(snapshot);
}
geometry_msgs::msg::TwistStamped
MpcController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::Twist &speed)
{
  return impl_->compute(pose, speed);
}
void MpcController::notifyPublished(const geometry_msgs::msg::Twist &command, double yaw)
{
  impl_->last_published = mpc::rotate(yaw, {command.linear.x, command.linear.y});
  impl_->have_published = true;
  impl_->published_at = Clock::now();
}
void MpcController::validateLatestMap(const map_server::CostmapSnapshot &snapshot) const
{
  if (!snapshot.current)
  {
    throw std::runtime_error("STALE_INPUT: latest map");
  }
  impl_->validate(snapshot.grid, impl_->last_check);
}
const MpcFeedback &MpcController::feedback() const { return impl_->feedback; }
bool MpcController::hasFreshPlan() const
{
  return impl_->feedback.valid &&
         std::chrono::duration<double>(Clock::now() - impl_->published_at).count() <= 0.15;
}
bool MpcController::hasFreshDetour() const { return hasFreshPlan() && impl_->feedback.detouring; }
} // namespace nav_executor
