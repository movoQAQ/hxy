#include "nav_executor/path_executor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "nav_executor/mpc_controller.hpp"
#include "nav_executor/recovery_controller.hpp"
#include "nav_executor/safety_monitor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_executor {
bool finiteTwist(const geometry_msgs::msg::Twist &speed) {
  return std::isfinite(speed.linear.x) && std::isfinite(speed.linear.y) &&
         std::isfinite(speed.linear.z) && std::isfinite(speed.angular.x) &&
         std::isfinite(speed.angular.y) && std::isfinite(speed.angular.z);
}

geometry_msgs::msg::Twist transformRobotVelocity(
    const geometry_msgs::msg::Twist &speed,
    const geometry_msgs::msg::TransformStamped &base_from_child) {
  if (!finiteTwist(speed)) {
    throw std::invalid_argument("Odometry velocity is not finite");
  }
  tf2::Transform transform;
  tf2::fromMsg(base_from_child.transform, transform);
  const tf2::Vector3 angular =
      transform.getBasis() *
      tf2::Vector3(speed.angular.x, speed.angular.y, speed.angular.z);
  const tf2::Vector3 linear =
      transform.getBasis() *
          tf2::Vector3(speed.linear.x, speed.linear.y, speed.linear.z) +
      transform.getOrigin().cross(angular);
  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x();
  result.linear.y = linear.y();
  result.linear.z = linear.z();
  result.angular.x = angular.x();
  result.angular.y = angular.y();
  result.angular.z = angular.z();
  if (!finiteTwist(result)) {
    throw std::runtime_error("Invalid transformed odometry velocity");
  }
  return result;
}

geometry_msgs::msg::Twist transformOdometryVelocity(
    tf2_ros::Buffer &buffer, const std::string &base_frame,
    const nav_msgs::msg::Odometry &odometry, double wait_timeout) {
  if (!std::isfinite(wait_timeout) || wait_timeout < 0.0 ||
      base_frame.empty() || odometry.child_frame_id.empty()) {
    throw std::invalid_argument("Invalid odometry velocity transform request");
  }
  if (odometry.child_frame_id == base_frame) {
    return odometry.twist.twist;
  }
  const auto transform = buffer.lookupTransform(
      base_frame, odometry.child_frame_id, odometry.header.stamp,
      rclcpp::Duration::from_seconds(wait_timeout));
  return transformRobotVelocity(odometry.twist.twist, transform);
}

namespace {
bool finitePose(const geometry_msgs::msg::Pose &pose) {
  const auto &p = pose.position;
  const auto &q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
}

const char *stateName(ExecutionState state) {
  switch (state) {
  case ExecutionState::IDLE:
    return "IDLE";
  case ExecutionState::RUNNING:
    return "RUNNING";
  case ExecutionState::STOPPED:
    return "STOPPED";
  case ExecutionState::SUCCEEDED:
    return "SUCCEEDED";
  case ExecutionState::FAILED:
    return "FAILED";
  }
  return "UNKNOWN";
}

struct GoalTolerances {
  double xy{0.3};
  double yaw{3.14};
  double stopped_speed{0.05};
};
} // namespace

class PathExecutor::Impl {
public:
  Impl(const rclcpp::Node::SharedPtr &node,
       std::shared_ptr<map_server::CostmapClient> costmap,
       std::function<bool()> global_ready, Failure failure, Success success,
       Confirmation confirmation, Observation observation, Recovery recovery)
      : node_(node), costmap_(std::move(costmap)),
        global_ready_(std::move(global_ready)), failure_(std::move(failure)),
        success_(std::move(success)), confirmation_(std::move(confirmation)),
        observation_(std::move(observation)),
        recovery_callback_(std::move(recovery)), logger_(node->get_logger()),
        clock_(node->get_clock()) {
    frequency_ = node->declare_parameter("controller_frequency", 20.0);
    prepare_spin_timeout_ =
        node->declare_parameter("spin.prepare_timeout", 3.0);
    if (!std::isfinite(prepare_spin_timeout_) || prepare_spin_timeout_ <= 0)
      throw std::invalid_argument("Invalid spin preparation timeout");
    odom_timeout_ = node->declare_parameter("odom_timeout", 0.5);
    tf_timeout_ = node->declare_parameter("tf_timeout", 0.5);
    control_timeout_ = node->declare_parameter("control_timeout", 0.2);
    braking_acceleration_ =
        node->declare_parameter("safety.braking_acceleration", 2.0);
    hazard_threshold_ =
        node->declare_parameter("recovery.hazard_threshold", 20);
    if (!std::isfinite(braking_acceleration_) || braking_acceleration_ <= 0 ||
        hazard_threshold_ < 2 || hazard_threshold_ > 254)
      throw std::invalid_argument("Invalid braking/hazard threshold");
    tolerances_.xy = node->declare_parameter("xy_goal_tolerance", 0.05);
    check_yaw_ = node->declare_parameter("goal_check_yaw", false);
    check_angular_ =
        node->declare_parameter("goal_check_angular_velocity", false);
    goal_confirmations_ = node->declare_parameter("goal_confirmations", 3);
    tolerances_.yaw = node->declare_parameter("yaw_goal_tolerance", 3.14);
    tolerances_.stopped_speed =
        node->declare_parameter("stopped_velocity_tolerance", 0.03);
    const auto odom_topic =
        node->declare_parameter("odom_topic", std::string("/Odometry"));
    const auto cmd_topic =
        node->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    for (double v :
         {frequency_, odom_timeout_, tf_timeout_, control_timeout_,
          tolerances_.xy, tolerances_.yaw, tolerances_.stopped_speed})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument("Invalid executor limits");
    if (frequency_ > 1000 || goal_confirmations_ < 1 || odom_topic.empty() ||
        cmd_topic.empty())
      throw std::invalid_argument(
          "Invalid executor frequency, confirmations or topics");
    SafetyMonitorConfig safety;
    safety.stall_duration =
        node->declare_parameter("safety.stall_duration", 3.0);
    safety.minimum_command_speed =
        node->declare_parameter("safety.minimum_command_speed", 0.1);
    safety.minimum_displacement =
        node->declare_parameter("safety.minimum_displacement", 0.08);
    safety.maximum_measured_speed =
        node->declare_parameter("safety.maximum_measured_speed", 0.05);
    safety.maximum_sample_gap =
        node->declare_parameter("safety.maximum_sample_gap", 0.3);
    safety_ = std::make_unique<SafetyMonitor>(safety);
    RecoveryConfig limits;
    limits.distance = node->declare_parameter("recovery.distance", 0.35);
    limits.search_radius =
        node->declare_parameter("recovery.search_radius", 1.0);
    limits.speed = node->declare_parameter("recovery.speed", 0.15);
    limits.timeout = node->declare_parameter("recovery.timeout", 6.0);
    limits.acceleration = node->declare_parameter("recovery.acceleration", 0.3);
    limits.margin = node->declare_parameter("recovery.margin", 0.03);
    limits.angular_speed = node->declare_parameter("spin.angular_speed", 0.5);
    limits.angular_acceleration =
        node->declare_parameter("spin.angular_acceleration", 0.5);
    limits.stop_speed = node->declare_parameter("spin.stop_speed", 0.05);
    recovery_ = std::make_unique<RecoveryController>(limits);
    footprint_radius_ = costmap_->settings().robot_radius;
    for (const auto &p : costmap_->getRobotFootprint())
      footprint_radius_ = std::max(footprint_radius_, std::hypot(p.x, p.y));
    cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
        cmd_topic, rclcpp::QoS(1));
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = node->create_publisher<std_msgs::msg::String>(
        "nav_executor/execution_state", qos);
    error_pub_ = node->create_publisher<std_msgs::msg::String>(
        "nav_executor/execution_error", qos);
    motion_pub_ = node->create_publisher<std_msgs::msg::String>(
        "nav_executor/motion_state", qos);
    auto safety_input = [this](bool emergency,
                               std_msgs::msg::Bool::ConstSharedPtr msg) {
      if (emergency)
        emergency_stop_ = msg->data;
      else
        chassis_fault_ = msg->data;
      if (msg->data)
        fail(emergency ? "Emergency stop" : "Chassis fault");
    };
    emergency_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "nav_executor/emergency_stop", 10,
        [safety_input](std_msgs::msg::Bool::ConstSharedPtr msg) {
          safety_input(true, msg);
        });
    fault_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "nav_executor/chassis_fault", 10,
        [safety_input](std_msgs::msg::Bool::ConstSharedPtr msg) {
          safety_input(false, msg);
        });
    controller_.initialize(node, costmap_->getTfBuffer(),
                           costmap_->getCostmap(), costmap_->getGlobalFrameID(),
                           costmap_->getBaseFrameID(),
                           costmap_->settings().robot_radius);
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const rclcpp::Time stamp(msg->header.stamp, clock_->get_clock_type());
          if (msg->child_frame_id.empty() || stamp.nanoseconds() <= 0 ||
              !finiteTwist(msg->twist.twist) ||
              (odom_ && stamp < rclcpp::Time(odom_->header.stamp,
                                             clock_->get_clock_type()))) {
            odom_.reset();
            fail("Invalid or out-of-order odometry");
            return;
          }
          odom_ = *msg;
          odom_received_ = TaskManager::Clock::now();
        });
    timer_ = node->create_wall_timer(
        std::chrono::duration<double>(1. / frequency_), [this]() { tick(); });
    publishZero();
    setState(ExecutionState::IDLE, "");
    publishMotionState();
    RCLCPP_INFO(logger_,
                "Motion FSM initialized; controller: original qpOASES MPC");
  }
  ~Impl() {
    timer_.reset();
    publishZero();
  }
  void publishMotionState() {
    std_msgs::msg::String msg;
    msg.data = MotionFSM::name(fsm_.state());
    motion_pub_->publish(msg);
  }
  void event(MotionEvent e) {
    const auto old = fsm_.state();
    fsm_.consume(e);
    if (old != fsm_.state())
      publishMotionState();
  }
  // Every normal, spin and recovery command leaves through this single
  // publisher.
  void publishCommand(const geometry_msgs::msg::Twist &command) {
    last_command_ = command;
    cmd_pub_->publish(command);
  }
  void publishZero() { publishCommand(geometry_msgs::msg::Twist{}); }
  void setState(ExecutionState state, const std::string &reason) {
    state_ = state;
    std_msgs::msg::String msg;
    msg.data = stateName(state);
    state_pub_->publish(msg);
    msg.data = reason;
    error_pub_->publish(msg);
  }
  void stop(const std::string &reason) {
    path_.reset();
    hold_pose_.reset();
    recovery_target_.reset();
    spin_requested_ = false;
    distance_remaining_ = -1.;
    goal_hits_ = 0;
    last_goal_stamp_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    safety_->reset();
    publishZero();
    controller_.reset();
    event(MotionEvent::STOP);
    setState(ExecutionState::STOPPED, reason);
  }
  void fail(const std::string &reason) {
    const bool recovering = fsm_.overridesPath(), had_path = path_.has_value();
    path_.reset();
    hold_pose_.reset();
    recovery_target_.reset();
    spin_requested_ = false;
    goal_hits_ = 0;
    safety_->reset();
    publishZero();
    event(MotionEvent::STOP);
    setState(ExecutionState::FAILED, reason);
    if (recovering && recovery_callback_)
      recovery_callback_(identity_, false, false, reason);
    else if (had_path)
      failure_(identity_, reason);
  }
  bool acceptPath(const nav_msgs::msg::Path &path, PlanIdentity identity) {
    if (fsm_.overridesPath() || !confirmation_(identity))
      return false;
    stop("Replacing execution path");
    identity_ = identity;
    try {
      if (path.poses.empty() || path.header.frame_id.empty())
        throw std::invalid_argument("Empty path/frame");
      for (const auto &p : path.poses)
        if (!finitePose(p.pose) || (!p.header.frame_id.empty() &&
                                    p.header.frame_id != path.header.frame_id))
          throw std::invalid_argument("Invalid path pose/frame");
      controller_.setPlan(path);
      path_ = path;
      event(MotionEvent::PATH_ACCEPTED);
      setState(ExecutionState::RUNNING, "");
      return true;
    } catch (const std::exception &e) {
      setState(ExecutionState::FAILED, e.what());
      failure_(identity_, e.what());
      return false;
    }
  }
  void requestSpin(bool requested) {
    if (requested == spin_requested_)
      return;
    spin_requested_ = requested;
    spin_started_ = TaskManager::Clock::now();
    if (fsm_.recovering())
      return; // Recovery owns control until completion.
    path_.reset();
    safety_->reset();
    controller_.reset();
    publishZero();
    event(requested ? MotionEvent::SPIN_REQUESTED : MotionEvent::SPIN_RELEASED);
  }
  void requestEscape() {
    if (fsm_.state() == MotionState::FOLLOW)
      startRecovery(false);
  }
  void startRecovery(bool hazard) {
    path_.reset();
    recovery_target_.reset();
    safety_->reset();
    publishZero();
    controller_.reset();
    recovery_started_ = TaskManager::Clock::now();
    event(hazard ? MotionEvent::HAZARD : MotionEvent::STALLED);
    if (recovery_callback_)
      recovery_callback_(identity_, true, false,
                         hazard ? "HAZARD" : "PHYSICAL_STALL");
  }
  void checkStamp(const builtin_interfaces::msg::Time &stamp, double timeout,
                  const std::string &description,
                  bool allow_zero = false) const {
    const rclcpp::Time time(stamp, clock_->get_clock_type());
    if (allow_zero && time.nanoseconds() == 0)
      return;
    const double age = (clock_->now() - time).seconds();
    if (time.nanoseconds() <= 0 || age > timeout || age < -.1)
      throw std::runtime_error(description + " timestamp is stale or invalid");
  }
  geometry_msgs::msg::Twist readSpeed() {
    if (!odom_ || std::chrono::duration<double>(TaskManager::Clock::now() -
                                                odom_received_)
                          .count() > odom_timeout_)
      throw std::runtime_error("Odometry missing or receive timeout");
    checkStamp(odom_->header.stamp, odom_timeout_, "Odometry");
    return transformOdometryVelocity(*costmap_->getTfBuffer(),
                                     costmap_->getBaseFrameID(), *odom_,
                                     std::min(tf_timeout_, .05));
  }
  bool safeSweep(const geometry_msgs::msg::PoseStamped &pose, double x,
                 double y) {
    auto *grid = costmap_->getCostmap();
    std::lock_guard<map_server::Costmap2D::mutex_t> lock(*grid->getMutex());
    return sweptFootprintSafe(*grid, pose.pose.position.x, pose.pose.position.y,
                              x, y, footprint_radius_);
  }
  geometry_msgs::msg::Twist limitOverride(geometry_msgs::msg::Twist command) {
    const auto &c = recovery_->config();
    const double max_step = c.acceleration / frequency_;
    const double dx = command.linear.x - last_command_.linear.x,
                 dy = command.linear.y - last_command_.linear.y;
    const double scale =
        std::hypot(dx, dy) > max_step ? max_step / std::hypot(dx, dy) : 1.;
    command.linear.x = last_command_.linear.x + scale * dx;
    command.linear.y = last_command_.linear.y + scale * dy;
    command.angular.z = std::clamp(
        command.angular.z,
        last_command_.angular.z - c.angular_acceleration / frequency_,
        last_command_.angular.z + c.angular_acceleration / frequency_);
    return command;
  }
  void checkedPublish(const geometry_msgs::msg::Twist &command,
                      const geometry_msgs::msg::PoseStamped &pose) {
    if (!finiteTwist(command) || emergency_stop_ || chassis_fault_ ||
        !global_ready_() || !costmap_->isCurrent())
      throw std::runtime_error("Final command safety gate rejected inputs");
    readSpeed();
    checkStamp(pose.header.stamp, tf_timeout_, "Robot TF", true);
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double horizon = .2 + std::hypot(command.linear.x, command.linear.y) /
                                    (2 * braking_acceleration_);
    const double dx =
        (std::cos(yaw) * command.linear.x - std::sin(yaw) * command.linear.y) *
        horizon;
    const double dy =
        (std::sin(yaw) * command.linear.x + std::cos(yaw) * command.linear.y) *
        horizon;
    if (!safeSweep(pose, pose.pose.position.x + dx, pose.pose.position.y + dy))
      throw std::runtime_error("Command swept footprint intersects obstacle, "
                               "unknown space or map boundary");
    publishCommand(command);
  }
  void recoveryTick(const geometry_msgs::msg::PoseStamped &pose,
                    const geometry_msgs::msg::Twist &speed) {
    const auto &c = recovery_->config();
    if (std::chrono::duration<double>(TaskManager::Clock::now() -
                                      recovery_started_)
            .count() > c.timeout)
      throw std::runtime_error(
          "Recovery timed out; stopped without a safe escape");
    if (!recovery_target_) {
      if (std::hypot(speed.linear.x, speed.linear.y) > c.stop_speed) {
        publishZero();
        return;
      }
      auto *grid = costmap_->getCostmap();
      std::lock_guard<map_server::Costmap2D::mutex_t> lock(*grid->getMutex());
      recovery_target_ =
          recovery_->select(*grid, pose, footprint_radius_,
                            fsm_.state() == MotionState::HAZARD_RECOVERY);
      if (!recovery_target_)
        throw std::runtime_error("No safe swept-footprint recovery direction");
    }
    const double dx = recovery_target_->x - pose.pose.position.x,
                 dy = recovery_target_->y - pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < .05) {
      publishZero();
      if (std::hypot(speed.linear.x, speed.linear.y) > c.stop_speed)
        return;
      recovery_target_.reset();
      event(MotionEvent::RECOVERY_SUCCEEDED);
      setState(ExecutionState::STOPPED,
               "Recovery completed; waiting for new generation");
      if (recovery_callback_)
        recovery_callback_(identity_, false, true, "Recovery completed");
      if (spin_requested_)
        event(MotionEvent::SPIN_REQUESTED);
      return;
    }
    if (!safeSweep(pose, recovery_target_->x, recovery_target_->y))
      throw std::runtime_error("Recovery corridor became unsafe");
    const double yaw = tf2::getYaw(pose.pose.orientation),
                 v = std::min(c.speed, distance);
    geometry_msgs::msg::Twist command;
    command.linear.x = (std::cos(yaw) * dx + std::sin(yaw) * dy) * v / distance;
    command.linear.y =
        (-std::sin(yaw) * dx + std::cos(yaw) * dy) * v / distance;
    checkedPublish(limitOverride(command), pose);
  }
  void tick() {
    if (!path_ && !fsm_.overridesPath() && fsm_.state() != MotionState::HOLD) {
      publishZero();
      return;
    }
    if (path_ && !confirmation_(identity_)) {
      stop("TaskManager revoked path");
      return;
    }
    try {
      if (emergency_stop_ || chassis_fault_)
        throw std::runtime_error("Emergency stop or chassis fault");
      if (!global_ready_() || !costmap_->isCurrent())
        throw std::runtime_error(
            "Global or local costmap unavailable or not current");
      geometry_msgs::msg::PoseStamped pose;
      if (!costmap_->getRobotPose(pose))
        throw std::runtime_error("Robot pose TF unavailable");
      checkStamp(pose.header.stamp, tf_timeout_, "Robot TF", true);
      const auto speed = readSpeed();
      if (fsm_.recovering()) {
        recoveryTick(pose, speed);
        return;
      }
      if (!safeSweep(pose, pose.pose.position.x, pose.pose.position.y)) {
        if (path_) {
          startRecovery(true);
          return;
        }
        throw std::runtime_error("Current footprint unsafe; motion rejected");
      }
      if (path_ && fsm_.state() == MotionState::FOLLOW) {
        auto *grid = costmap_->getCostmap();
        std::lock_guard<map_server::Costmap2D::mutex_t> lock(*grid->getMutex());
        unsigned int x, y;
        if (grid->worldToMap(pose.pose.position.x, pose.pose.position.y, x,
                             y) &&
            grid->getCost(x, y) < hazard_threshold_) {
          startRecovery(true);
          return;
        }
      }
      if (fsm_.state() == MotionState::PREPARE_SPIN) {
        if (std::chrono::duration<double>(TaskManager::Clock::now() -
                                          spin_started_)
                .count() > prepare_spin_timeout_)
          throw std::runtime_error(
              "Spin preparation timed out before measured stopping");
        publishZero();
        if (std::hypot(speed.linear.x, speed.linear.y) <
                recovery_->config().stop_speed &&
            std::abs(speed.angular.z) < recovery_->config().stop_speed)
          event(MotionEvent::SPIN_READY);
        return;
      }
      if (fsm_.state() == MotionState::SPIN) {
        geometry_msgs::msg::Twist command;
        command.angular.z = recovery_->config().angular_speed;
        checkedPublish(limitOverride(command), pose);
        return;
      }
      if (fsm_.state() == MotionState::HOLD) {
        if (!hold_pose_) {
          publishZero();
          return;
        }
        const double dx = hold_pose_->pose.position.x - pose.pose.position.x;
        const double dy = hold_pose_->pose.position.y - pose.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (distance < std::min(.03, tolerances_.xy * .5)) {
          publishZero();
          return;
        }
        const double yaw = tf2::getYaw(pose.pose.orientation),
                     v = std::min(recovery_->config().speed, distance);
        geometry_msgs::msg::Twist command;
        command.linear.x =
            (std::cos(yaw) * dx + std::sin(yaw) * dy) * v / distance;
        command.linear.y =
            (-std::sin(yaw) * dx + std::cos(yaw) * dy) * v / distance;
        checkedPublish(limitOverride(command), pose);
        return;
      }
      if (!path_) {
        publishZero();
        return;
      }
      if (safety_->observe(pose, speed, last_command_,
                           TaskManager::Clock::now())) {
        startRecovery(false);
        return;
      }
      const auto path_tf = costmap_->getTfBuffer()->lookupTransform(
          costmap_->getGlobalFrameID(), path_->header.frame_id,
          tf2::TimePointZero);
      checkStamp(path_tf.header.stamp, tf_timeout_, "Path TF", true);
      auto goal = path_->poses.back();
      goal.header.frame_id = path_->header.frame_id;
      goal.header.stamp = builtin_interfaces::msg::Time{};
      goal = costmap_->getTfBuffer()->transform(goal,
                                                costmap_->getGlobalFrameID());
      if (!finitePose(pose.pose) || !finitePose(goal.pose))
        throw std::runtime_error("Invalid robot/goal pose");
      distance_remaining_ =
          std::hypot(pose.pose.position.x - goal.pose.position.x,
                     pose.pose.position.y - goal.pose.position.y);
      const double yaw_error =
          std::remainder(tf2::getYaw(pose.pose.orientation) -
                             tf2::getYaw(goal.pose.orientation),
                         2 * M_PI);
      const bool reached =
          distance_remaining_ <= tolerances_.xy &&
          (!check_yaw_ || std::abs(yaw_error) <= tolerances_.yaw);
      const bool stopped = std::hypot(speed.linear.x, speed.linear.y) <=
                               tolerances_.stopped_speed &&
                           (!check_angular_ || std::abs(speed.angular.z) <=
                                                   tolerances_.stopped_speed);
      if (reached && stopped) {
        publishZero();
        const rclcpp::Time stamp(odom_->header.stamp, clock_->get_clock_type());
        if (stamp > last_goal_stamp_) {
          ++goal_hits_;
          last_goal_stamp_ = stamp;
        }
        if (goal_hits_ >= goal_confirmations_) {
          if (active_hold_)
            hold_pose_ = pose;
          else
            hold_pose_.reset();
          path_.reset();
          controller_.reset();
          event(MotionEvent::GOAL_REACHED);
          setState(ExecutionState::SUCCEEDED, "");
          success_(identity_);
        }
        return;
      }
      goal_hits_ = 0;
      const auto began = TaskManager::Clock::now();
      const auto command = controller_.computeVelocityCommands(pose, speed);
      if (std::chrono::duration<double>(TaskManager::Clock::now() - began)
              .count() > control_timeout_)
        throw std::runtime_error("MPC computation deadline exceeded");
      tf2::Transform local_from_path;
      tf2::fromMsg(path_tf.transform, local_from_path);
      auto path_from_local = path_tf;
      path_from_local.header.frame_id = path_->header.frame_id;
      path_from_local.child_frame_id = costmap_->getGlobalFrameID();
      path_from_local.transform = tf2::toMsg(local_from_path.inverse());
      geometry_msgs::msg::PoseStamped route_pose;
      tf2::doTransform(pose, route_pose, path_from_local);
      observation_(identity_, route_pose, command.twist);
      if (!path_ || !confirmation_(identity_) ||
          fsm_.state() != MotionState::FOLLOW) {
        publishZero();
        return;
      }
      checkStamp(path_tf.header.stamp, tf_timeout_, "Path TF", true);
      checkedPublish(command.twist, pose);
    } catch (const std::exception &e) {
      fail(e.what());
    }
  }
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<map_server::CostmapClient> costmap_;
  std::function<bool()> global_ready_;
  Failure failure_;
  Success success_;
  Confirmation confirmation_;
  Observation observation_;
  Recovery recovery_callback_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  bool active_hold_{false};
  bool check_yaw_{false}, check_angular_{false}, spin_requested_{false},
      emergency_stop_{false}, chassis_fault_{false};
  int goal_hits_{0}, goal_confirmations_{3};
  rclcpp::Time last_goal_stamp_{0, 0, RCL_ROS_TIME};
  double frequency_, odom_timeout_, tf_timeout_, control_timeout_,
      footprint_radius_, braking_acceleration_, distance_remaining_{-1.0};
  int hazard_threshold_{20};
  double prepare_spin_timeout_{3.0};
  GoalTolerances tolerances_;
  MpcController controller_;
  MotionFSM fsm_;
  std::unique_ptr<SafetyMonitor> safety_;
  std::unique_ptr<RecoveryController> recovery_;
  std::optional<geometry_msgs::msg::PoseStamped> hold_pose_;
  std::optional<RecoveryTarget> recovery_target_;
  geometry_msgs::msg::Twist last_command_;
  std::optional<nav_msgs::msg::Path> path_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  TaskManager::Clock::time_point odom_received_{}, recovery_started_{},
      spin_started_{};
  PlanIdentity identity_;
  ExecutionState state_{ExecutionState::IDLE};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, error_pub_,
      motion_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_,
      fault_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

PathExecutor::PathExecutor(
    const rclcpp::Node::SharedPtr &node,
    std::shared_ptr<map_server::CostmapClient> local_costmap,
    std::function<bool()> global_map_ready, Failure failure, Success success,
    Confirmation confirmation, Observation observation, Recovery recovery)
    : impl_(std::make_unique<Impl>(
          node, std::move(local_costmap), std::move(global_map_ready),
          std::move(failure), std::move(success), std::move(confirmation),
          std::move(observation), std::move(recovery))) {}
PathExecutor::~PathExecutor() = default;
bool PathExecutor::acceptPath(const nav_msgs::msg::Path &path,
                              PlanIdentity identity) {
  return impl_->acceptPath(path, identity);
}
void PathExecutor::stop(const std::string &reason) { impl_->stop(reason); }

ExecutionState PathExecutor::state() const { return impl_->state_; }
MotionState PathExecutor::motionState() const { return impl_->fsm_.state(); }
void PathExecutor::setHoldMode(bool active) { impl_->active_hold_ = active; }
void PathExecutor::requestSpin(bool requested) {
  impl_->requestSpin(requested);
}
void PathExecutor::requestEscape() { impl_->requestEscape(); }
double PathExecutor::distanceRemaining() const {
  return impl_->distance_remaining_;
}
} // namespace nav_executor
