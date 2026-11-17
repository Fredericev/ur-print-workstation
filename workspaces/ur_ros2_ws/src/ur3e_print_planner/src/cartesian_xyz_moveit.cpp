#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ur_klipper_interfaces/action/manual_stepper_move.hpp>
#include "printhead_collision_geometry.hpp"

namespace
{

std::vector<std::string> parseAxes(const std::string & value)
{
  std::vector<std::string> axes;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
    std::transform(item.begin(), item.end(), item.begin(), ::tolower);
    if (!item.empty())
    {
      if (item != "x" && item != "y" && item != "z")
      {
        throw std::runtime_error("axes may only contain x, y, z");
      }
      axes.push_back(item);
    }
  }
  if (axes.empty())
  {
    throw std::runtime_error("axes cannot be empty");
  }
  return axes;
}

void installHeightTrialCollisionGeometry(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & end_effector_link,
  const double surface_z,
  const double minimum_x, const double maximum_x,
  const double minimum_y, const double maximum_y,
  const double tcp_x, const double tcp_y, const double tcp_z)
{
  if (!std::isfinite(surface_z) || !std::isfinite(minimum_x) ||
      !std::isfinite(maximum_x) || !std::isfinite(minimum_y) ||
      !std::isfinite(maximum_y) || minimum_x >= maximum_x ||
      minimum_y >= maximum_y)
  {
    throw std::runtime_error("invalid table-height trial collision geometry");
  }
  const double tcp_length = std::hypot(tcp_x, tcp_y, tcp_z);
  if (!std::isfinite(tcp_length) || tcp_length < 0.01 || tcp_length > 0.5 ||
      std::abs(tcp_x) > 0.5 || std::abs(tcp_y) > 0.5 || std::abs(tcp_z) > 0.5)
  {
    throw std::runtime_error("height trial requires a finite, measured tool0-to-nozzle TCP within 500 mm");
  }
  moveit::planning_interface::PlanningSceneInterface scene;
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "base_link";
  table.id = "height_trial_table_forbidden_region";
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_shape.dimensions = {maximum_x - minimum_x, maximum_y - minimum_y, 2.0};
  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = (minimum_x + maximum_x) / 2.0;
  table_pose.position.y = (minimum_y + maximum_y) / 2.0;
  table_pose.position.z = surface_z - 1.0;
  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene.applyCollisionObject(table))
  {
    throw std::runtime_error("MoveIt rejected the height-trial tabletop guard");
  }

  // Bounds of the represented STEP solids in the candidate tool0 mount.
  // Missing physical parts, the mount and the selected nozzle end state still
  // require verification before this close-to-table motion is authorized.
  moveit_msgs::msg::CollisionObject head;
  head.header.frame_id = end_effector_link;
  head.id = "height_trial_printhead_box";
  shape_msgs::msg::SolidPrimitive head_shape;
  head_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  head_shape.dimensions = {
    ur3e_print_planner::kPrintheadBoxSizeX,
    ur3e_print_planner::kPrintheadBoxSizeY,
    ur3e_print_planner::kPrintheadBoxSizeZ,
  };
  geometry_msgs::msg::Pose head_pose;
  head_pose.orientation.w = 1.0;
  head_pose.position.x = ur3e_print_planner::kPrintheadBoxCenterX;
  head_pose.position.y = ur3e_print_planner::kPrintheadBoxCenterY;
  head_pose.position.z = ur3e_print_planner::kPrintheadBoxCenterZ;
  head.primitives.push_back(head_shape);
  head.primitive_poses.push_back(head_pose);
  head.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene.applyCollisionObject(head) ||
      !move_group.attachObject(head.id, end_effector_link,
        {end_effector_link, "flange", "wrist_3_link"}))
  {
    throw std::runtime_error("MoveIt could not attach the height-trial printhead box");
  }

  // A measured tip may extend beyond the represented CAD box, so guard its
  // full tool0-to-tip span and the tip itself.
  moveit_msgs::msg::CollisionObject nozzle;
  nozzle.header.frame_id = end_effector_link;
  nozzle.id = "height_trial_measured_nozzle_guard";
  shape_msgs::msg::SolidPrimitive shaft;
  shaft.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  shaft.dimensions = {tcp_length, 0.004};
  geometry_msgs::msg::Pose shaft_pose;
  shaft_pose.position.x = tcp_x / 2.0;
  shaft_pose.position.y = tcp_y / 2.0;
  shaft_pose.position.z = tcp_z / 2.0;
  // A ROS cylinder is aligned with its local +Z axis.
  const double nx = tcp_x / tcp_length;
  const double ny = tcp_y / tcp_length;
  const double nz = tcp_z / tcp_length;
  if (nz < -0.999999)
  {
    shaft_pose.orientation.x = 1.0;
  }
  else
  {
    const double inverse_norm = 1.0 / std::sqrt(2.0 * (1.0 + nz));
    shaft_pose.orientation.x = -ny * inverse_norm;
    shaft_pose.orientation.y = nx * inverse_norm;
    shaft_pose.orientation.w = (1.0 + nz) * inverse_norm;
  }
  nozzle.primitives.push_back(shaft);
  nozzle.primitive_poses.push_back(shaft_pose);
  shape_msgs::msg::SolidPrimitive tip;
  tip.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  tip.dimensions = {0.004};
  geometry_msgs::msg::Pose tip_pose;
  tip_pose.orientation.w = 1.0;
  tip_pose.position.x = tcp_x;
  tip_pose.position.y = tcp_y;
  tip_pose.position.z = tcp_z;
  nozzle.primitives.push_back(tip);
  nozzle.primitive_poses.push_back(tip_pose);
  nozzle.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene.applyCollisionObject(nozzle) ||
      !move_group.attachObject(nozzle.id, end_effector_link,
        {end_effector_link, "flange", "wrist_3_link"}))
  {
    throw std::runtime_error("MoveIt could not attach the measured height-trial nozzle guard");
  }
  RCLCPP_INFO(node->get_logger(), "height_trial_collision_model_v4_geometry_bound installed");
}

void applyAxis(geometry_msgs::msg::Pose & pose, const std::string & axis, const double distance)
{
  if (axis == "x")
  {
    pose.position.x += distance;
  }
  else if (axis == "y")
  {
    pose.position.y += distance;
  }
  else if (axis == "z")
  {
    pose.position.z += distance;
  }
}

bool envIsYes(const char * name)
{
  const char * value = std::getenv(name);
  return value != nullptr && std::string(value) == "YES";
}

void loadMatchingRobotDescriptions(const rclcpp::Node::SharedPtr & node)
{
  const auto local_urdf = node->get_parameter("robot_description").as_string();
  const auto local_srdf = node->get_parameter("robot_description_semantic").as_string();
  if (!local_urdf.empty() && !local_srdf.empty())
  {
    return;
  }

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(std::chrono::seconds(5)))
  {
    throw std::runtime_error(
      "MoveIt parameter service is unavailable; start MoveIt before running this test");
  }

  auto future = client->get_parameters(
    {"robot_description", "robot_description_semantic"});
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
  {
    throw std::runtime_error("timed out while loading the robot model from /move_group");
  }

  const auto descriptions = future.get();
  if (
    descriptions.size() != 2 ||
    descriptions[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    descriptions[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    descriptions[0].as_string().empty() || descriptions[1].as_string().empty())
  {
    throw std::runtime_error("/move_group did not provide a complete URDF and SRDF model");
  }

  const auto results = node->set_parameters({
    rclcpp::Parameter("robot_description", descriptions[0].as_string()),
    rclcpp::Parameter("robot_description_semantic", descriptions[1].as_string()),
  });
  if (
    results.size() != 2 || !results[0].successful || !results[1].successful)
  {
    throw std::runtime_error("failed to apply the matching MoveIt URDF and SRDF model");
  }
  RCLCPP_INFO(node->get_logger(), "Loaded matching URDF and SRDF from /move_group.");
}

std::string actionNameForController(const std::string & controller_name)
{
  return "/" + controller_name + "/follow_joint_trajectory";
}

bool isSupportedTrajectoryController(const std::string & controller_name)
{
  return controller_name == "passthrough_trajectory_controller" ||
         controller_name == "scaled_joint_trajectory_controller";
}

std::string competingController(const std::string & controller_name)
{
  if (controller_name == "passthrough_trajectory_controller")
  {
    return "scaled_joint_trajectory_controller";
  }
  if (controller_name == "scaled_joint_trajectory_controller")
  {
    return "passthrough_trajectory_controller";
  }
  throw std::runtime_error("unsupported trajectory controller: " + controller_name);
}

void switchTrajectoryController(
  const rclcpp::Node::SharedPtr & node, const std::string & controller_name)
{
  if (!isSupportedTrajectoryController(controller_name))
  {
    throw std::runtime_error("unsupported trajectory controller: " + controller_name);
  }

  using SwitchController = controller_manager_msgs::srv::SwitchController;
  auto client = node->create_client<SwitchController>("/controller_manager/switch_controller");
  if (!client->wait_for_service(std::chrono::seconds(5)))
  {
    throw std::runtime_error("/controller_manager/switch_controller service is not available");
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = {controller_name};
  request->deactivate_controllers = {competingController(controller_name)};
  request->strictness = request->BEST_EFFORT;

  RCLCPP_INFO(
    node->get_logger(), "Switching active trajectory controller to %s",
    controller_name.c_str());
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    throw std::runtime_error("timed out switching trajectory controller");
  }
  if (!future.get()->ok)
  {
    throw std::runtime_error("controller_manager refused to switch to " + controller_name);
  }
}

builtin_interfaces::msg::Duration secondsToDuration(const double seconds)
{
  builtin_interfaces::msg::Duration msg;
  const double clamped = std::max(0.001, seconds);
  msg.sec = static_cast<int32_t>(std::floor(clamped));
  msg.nanosec = static_cast<uint32_t>((clamped - static_cast<double>(msg.sec)) * 1.0e9);
  return msg;
}

double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

double maxAbsDelta(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size())
  {
    throw std::runtime_error("joint vector sizes do not match");
  }
  double result = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
  {
    result = std::max(result, std::abs(a[i] - b[i]));
  }
  return result;
}

class ProgramState
{
public:
  explicit ProgramState(const rclcpp::Node::SharedPtr & node)
  {
    auto qos = rclcpp::QoS(10).reliable().transient_local();
    sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/io_and_status_controller/robot_program_running", qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        seen_.store(true);
        running_.store(msg->data);
      });
  }

  bool waitUntilRunning(
    const rclcpp::Node::SharedPtr &, const double timeout_s,
    const double stable_s = 0.5)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      if (seen_.load() && running_.load())
      {
        if (stable_since == std::chrono::steady_clock::time_point{})
        {
          stable_since = std::chrono::steady_clock::now();
        }
        if (
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stable_since).count() >= stable_s)
        {
          return true;
        }
      }
      else
      {
        stable_since = std::chrono::steady_clock::time_point{};
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  bool isRunning() const
  {
    return seen_.load() && running_.load();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> seen_{false};
  std::atomic<bool> running_{false};
};

class ActualState
{
public:
  explicit ActualState(const rclcpp::Node::SharedPtr & node)
  {
    joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(20),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        positions_.clear();
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
        {
          positions_[msg->name[i]] = msg->position[i];
        }
        joint_seen_ = true;
      });

    auto tcp_qos = rclcpp::QoS(10).reliable().transient_local();
    tcp_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tcp_pose_broadcaster/pose", tcp_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        tcp_pose_ = *msg;
        tcp_seen_ = true;
      });
  }

  bool waitForJoints(const std::vector<std::string> & joint_names, const double timeout_s)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      try
      {
        positions(joint_names);
        return true;
      }
      catch (const std::exception &)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    return false;
  }

  std::vector<double> positions(const std::vector<std::string> & joint_names) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joint_seen_)
    {
      throw std::runtime_error("no /joint_states received");
    }
    std::vector<double> result;
    result.reserve(joint_names.size());
    for (const auto & name : joint_names)
    {
      const auto it = positions_.find(name);
      if (it == positions_.end())
      {
        throw std::runtime_error("missing joint in /joint_states: " + name);
      }
      result.push_back(it->second);
    }
    return result;
  }

  bool waitNear(
    const std::vector<std::string> & joint_names, const std::vector<double> & target,
    const double tolerance_rad, const double timeout_s)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      const auto actual = positions(joint_names);
      if (maxAbsDelta(actual, target) <= tolerance_rad)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  geometry_msgs::msg::PoseStamped tcpPose() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tcp_seen_)
    {
      throw std::runtime_error("no /tcp_pose_broadcaster/pose received");
    }
    return tcp_pose_;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_sub_;
  mutable std::mutex mutex_;
  std::map<std::string, double> positions_;
  geometry_msgs::msg::PoseStamped tcp_pose_;
  bool joint_seen_{false};
  bool tcp_seen_{false};
};

class GuardedTrajectoryExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using Watchdog = std::function<void()>;

  GuardedTrajectoryExecutor(
    const rclcpp::Node::SharedPtr & node, const std::string & controller_name)
  : node_(node), controller_name_(controller_name)
  {
    if (!isSupportedTrajectoryController(controller_name_))
    {
      throw std::runtime_error("unsupported trajectory controller: " + controller_name_);
    }
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, actionNameForController(controller_name_));
  }

  void prepare(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectory::_points_type & planned_points,
    ActualState & actual_state)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      throw std::runtime_error(controller_name_ + " action server is not available");
    }
    if (planned_points.empty())
    {
      throw std::runtime_error("no planned points to execute");
    }
    if (!actual_state.waitForJoints(joint_names, 5.0))
    {
      throw std::runtime_error("did not receive current joint states before execution");
    }
  }

  void execute(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectory::_points_type & planned_points,
    ActualState & actual_state, const double max_segment_delta_rad,
    const double max_joint_speed_rad_s, const double min_segment_duration_s,
    const double goal_tolerance_rad, const double goal_velocity_tolerance_rad_s,
    const bool smooth_execution, const bool include_velocity_setpoints,
    const Watchdog & watchdog = Watchdog{})
  {
    if (smooth_execution && controller_name_ == "scaled_joint_trajectory_controller")
    {
      throw std::runtime_error(
        "refusing smooth_execution on scaled_joint_trajectory_controller; use "
        "passthrough_trajectory_controller for smooth whole-trajectory execution");
    }
    prepare(joint_names, planned_points, actual_state);
    checkWatchdog(watchdog);

    if (smooth_execution)
    {
      executeSmooth(
        joint_names, planned_points, actual_state, max_segment_delta_rad, max_joint_speed_rad_s,
        min_segment_duration_s, goal_tolerance_rad, goal_velocity_tolerance_rad_s,
        include_velocity_setpoints, watchdog);
      return;
    }

    std::vector<double> previous_target = actual_state.positions(joint_names);
    size_t segment_index = 0;

    for (size_t point_index = 1; point_index < planned_points.size(); ++point_index)
    {
      const auto & planned = planned_points[point_index].positions;
      if (planned.size() != joint_names.size())
      {
        throw std::runtime_error("planned point size does not match joint count");
      }

      const double point_delta = maxAbsDelta(previous_target, planned);
      const size_t subdivisions = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(point_delta / max_segment_delta_rad)));

      const std::vector<double> start = previous_target;
      for (size_t sub = 1; sub <= subdivisions; ++sub)
      {
        const double ratio = static_cast<double>(sub) / static_cast<double>(subdivisions);
        std::vector<double> target;
        target.reserve(planned.size());
        for (size_t i = 0; i < planned.size(); ++i)
        {
          target.push_back(start[i] + (planned[i] - start[i]) * ratio);
        }
        sendSegment(joint_names, target, actual_state, max_joint_speed_rad_s,
          min_segment_duration_s, goal_tolerance_rad, goal_velocity_tolerance_rad_s,
          ++segment_index, watchdog);
        previous_target = target;
      }
    }
  }

private:
  void checkWatchdog(const Watchdog & watchdog) const
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("ROS shutdown requested during trajectory execution");
    }
    if (watchdog)
    {
      watchdog();
    }
  }

  void cancelGoalNoThrow(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      return;
    }
    try
    {
      RCLCPP_WARN(node_->get_logger(), "Canceling the active UR trajectory goal.");
      auto cancel_future = client_->async_cancel_goal(goal_handle);
      cancel_future.wait_for(std::chrono::seconds(2));
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not cancel the UR trajectory goal: %s", ex.what());
    }
  }

  template<typename ResultFutureT>
  void waitForResultOrCancel(
    ResultFutureT & result_future, const GoalHandle::SharedPtr & goal_handle,
    const std::chrono::steady_clock::time_point & deadline, const Watchdog & watchdog,
    const std::string & timeout_message)
  {
    try
    {
      while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
      {
        checkWatchdog(watchdog);
        if (std::chrono::steady_clock::now() >= deadline)
        {
          throw std::runtime_error(timeout_message);
        }
      }
      checkWatchdog(watchdog);
    }
    catch (...)
    {
      cancelGoalNoThrow(goal_handle);
      throw;
    }
  }

  void interruptibleSleep(
    const std::chrono::duration<double> & duration, const Watchdog & watchdog)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
    while (std::chrono::steady_clock::now() < deadline)
    {
      checkWatchdog(watchdog);
      const auto remaining = deadline - std::chrono::steady_clock::now();
      const auto quantum = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::min(remaining, quantum));
    }
  }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> buildGuardedPoints(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectory::_points_type & planned_points,
    const std::vector<double> & actual_start, const double max_segment_delta_rad,
    const double max_joint_speed_rad_s, const double min_segment_duration_s,
    const bool include_velocity_setpoints, double & total_duration_s)
  {
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> guarded_points;
    std::vector<double> previous_target = actual_start;
    total_duration_s = 0.0;

    for (size_t point_index = 1; point_index < planned_points.size(); ++point_index)
    {
      const auto & planned = planned_points[point_index].positions;
      if (planned.size() != actual_start.size())
      {
        throw std::runtime_error("planned point size does not match joint count");
      }

      const double point_delta = maxAbsDelta(previous_target, planned);
      const size_t subdivisions = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(point_delta / max_segment_delta_rad)));

      const std::vector<double> start = previous_target;
      for (size_t sub = 1; sub <= subdivisions; ++sub)
      {
        const double ratio = static_cast<double>(sub) / static_cast<double>(subdivisions);
        std::vector<double> target;
        target.reserve(planned.size());
        for (size_t i = 0; i < planned.size(); ++i)
        {
          target.push_back(start[i] + (planned[i] - start[i]) * ratio);
        }

        const double segment_delta = maxAbsDelta(previous_target, target);
        const double duration = std::max(min_segment_duration_s, segment_delta / max_joint_speed_rad_s);
        total_duration_s += duration;

        trajectory_msgs::msg::JointTrajectoryPoint guarded_point;
        guarded_point.positions = target;
        guarded_point.time_from_start = secondsToDuration(total_duration_s);
        guarded_points.push_back(guarded_point);
        previous_target = target;
      }
    }

    if (include_velocity_setpoints)
    {
      fillVelocitySetpoints(joint_names, actual_start, max_joint_speed_rad_s, guarded_points);
    }

    return guarded_points;
  }

  void fillVelocitySetpoints(
    const std::vector<std::string> & joint_names, const std::vector<double> & actual_start,
    const double max_joint_speed_rad_s,
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points)
  {
    const size_t joint_count = joint_names.size();
    for (auto & point : points)
    {
      point.velocities.assign(joint_count, 0.0);
      point.accelerations.clear();
    }
    if (points.size() < 2)
    {
      return;
    }

    for (size_t point_index = 0; point_index + 1 < points.size(); ++point_index)
    {
      const auto & before_positions = point_index == 0 ?
        actual_start : points[point_index - 1].positions;
      const auto & after_positions = points[point_index + 1].positions;
      const double before_time_s = point_index == 0 ?
        0.0 : durationToSeconds(points[point_index - 1].time_from_start);
      const double after_time_s = durationToSeconds(points[point_index + 1].time_from_start);
      const double span_s = after_time_s - before_time_s;
      if (span_s <= 0.0)
      {
        continue;
      }

      for (size_t joint_index = 0; joint_index < joint_count; ++joint_index)
      {
        const double raw_velocity =
          (after_positions[joint_index] - before_positions[joint_index]) / span_s;
        points[point_index].velocities[joint_index] =
          std::clamp(raw_velocity, -max_joint_speed_rad_s, max_joint_speed_rad_s);
      }
    }

    points.back().velocities.assign(joint_count, 0.0);
  }

  void addGoalTolerances(
    FollowJointTrajectory::Goal & goal, const std::vector<std::string> & joint_names,
    const double goal_tolerance_rad, const double goal_velocity_tolerance_rad_s)
  {
    goal.goal_tolerance.clear();
    for (const auto & name : joint_names)
    {
      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = name;
      tolerance.position = goal_tolerance_rad;
      tolerance.velocity = goal_velocity_tolerance_rad_s;
      tolerance.acceleration = 1.0;
      goal.goal_tolerance.push_back(tolerance);
    }
    goal.goal_time_tolerance = secondsToDuration(2.0);
  }

  void executeSmooth(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectory::_points_type & planned_points,
    ActualState & actual_state, const double max_segment_delta_rad,
    const double max_joint_speed_rad_s, const double min_segment_duration_s,
    const double goal_tolerance_rad, const double goal_velocity_tolerance_rad_s,
    const bool include_velocity_setpoints, const Watchdog & watchdog)
  {
    const auto actual_start = actual_state.positions(joint_names);
    double total_duration_s = 0.0;
    auto guarded_points = buildGuardedPoints(
      joint_names, planned_points, actual_start, max_segment_delta_rad, max_joint_speed_rad_s,
      min_segment_duration_s, include_velocity_setpoints, total_duration_s);
    if (guarded_points.empty())
    {
      throw std::runtime_error("smooth guarded trajectory contains no points");
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points = guarded_points;
    addGoalTolerances(goal, joint_names, goal_tolerance_rad, goal_velocity_tolerance_rad_s);

    const auto & final_target = guarded_points.back().positions;
    RCLCPP_INFO(
      node_->get_logger(),
      "Sending one smooth guarded trajectory through %s: %zu points, duration %.3f s, final delta %.6f rad",
      controller_name_.c_str(), guarded_points.size(), total_duration_s,
      maxAbsDelta(actual_start, final_target));

    const auto start_time = std::chrono::steady_clock::now();
    auto goal_future = client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      throw std::runtime_error("timed out sending smooth guarded trajectory goal");
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      throw std::runtime_error("smooth guarded trajectory goal was rejected");
    }

    auto result_future = client_->async_get_result(goal_handle);
    const auto result_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(total_duration_s + 5.0));
    waitForResultOrCancel(
      result_future, goal_handle, result_deadline, watchdog,
      "timed out waiting for smooth guarded trajectory result");

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      throw std::runtime_error(
        "smooth guarded trajectory failed: " + wrapped_result.result->error_string);
    }

    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    const auto desired = std::chrono::duration<double>(total_duration_s);
    if (elapsed < desired)
    {
      interruptibleSleep(desired - elapsed, watchdog);
    }
    if (!actual_state.waitNear(joint_names, final_target, goal_tolerance_rad, 3.0))
    {
      const auto after = actual_state.positions(joint_names);
      std::ostringstream error;
      error << "robot did not physically reach smooth guarded final target; max error "
            << maxAbsDelta(after, final_target) << " rad";
      throw std::runtime_error(error.str());
    }
  }

  void sendSegment(
    const std::vector<std::string> & joint_names, const std::vector<double> & target,
    ActualState & actual_state, const double max_joint_speed_rad_s,
    const double min_segment_duration_s, const double goal_tolerance_rad,
    const double goal_velocity_tolerance_rad_s, const size_t segment_index,
    const Watchdog & watchdog)
  {
    const auto actual = actual_state.positions(joint_names);
    const double delta = maxAbsDelta(actual, target);
    const double duration = std::max(min_segment_duration_s, delta / max_joint_speed_rad_s);

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target;
    point.velocities.assign(target.size(), 0.0);
    point.time_from_start = secondsToDuration(duration);
    goal.trajectory.points.push_back(point);
    addGoalTolerances(goal, joint_names, goal_tolerance_rad, goal_velocity_tolerance_rad_s);

    RCLCPP_INFO(
      node_->get_logger(), "Guarded segment %zu: max actual->target delta %.6f rad, duration %.3f s",
      segment_index, delta, duration);

    const auto start_time = std::chrono::steady_clock::now();
    auto goal_future = client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      throw std::runtime_error("timed out sending guarded segment goal");
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      throw std::runtime_error("guarded segment goal was rejected");
    }

    auto result_future = client_->async_get_result(goal_handle);
    const auto result_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(duration + 4.0));
    waitForResultOrCancel(
      result_future, goal_handle, result_deadline, watchdog,
      "timed out waiting for guarded segment result");
    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      throw std::runtime_error(
        "guarded segment failed: " + wrapped_result.result->error_string);
    }
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    const auto desired = std::chrono::duration<double>(duration);
    if (elapsed < desired)
    {
      interruptibleSleep(desired - elapsed, watchdog);
    }
    if (!actual_state.waitNear(joint_names, target, goal_tolerance_rad, 2.0))
    {
      const auto after = actual_state.positions(joint_names);
      std::ostringstream error;
      error << "robot did not physically reach guarded segment target; max error "
            << maxAbsDelta(after, target) << " rad";
      throw std::runtime_error(error.str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string controller_name_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

class KlipperActionClient
{
public:
  using ManualStepperMove = ur_klipper_interfaces::action::ManualStepperMove;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ManualStepperMove>;
  using Trigger = std_srvs::srv::Trigger;
  using Watchdog = std::function<void()>;

  explicit KlipperActionClient(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    action_client_ = rclcpp_action::create_client<ManualStepperMove>(
      node_, "/klipper/manual_stepper_move");
    refresh_client_ = node_->create_client<Trigger>("/klipper/refresh");
    disable_client_ = node_->create_client<Trigger>("/klipper/disable_stepper");
    emergency_client_ = node_->create_client<Trigger>("/klipper/emergency_stop");
  }

  void refresh()
  {
    const auto response = callTrigger(
      refresh_client_, "/klipper/refresh", std::chrono::seconds(8));
    if (!response->success)
    {
      throw std::runtime_error("ROS Klipper bridge is not ready: " + response->message);
    }
    RCLCPP_INFO(node_->get_logger(), "ROS Klipper bridge ready: %s", response->message.c_str());
  }

  void sendMove(
    const std::string & operation_id, const std::string & stepper,
    const double target_position, const double speed, const double acceleration,
    const bool set_position_before_move, const double initial_position,
    const bool disable_after, const Watchdog & watchdog)
  {
    if (active_)
    {
      throw std::runtime_error("a Klipper move is already active");
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      throw std::runtime_error("ROS Klipper manual-stepper action server is unavailable");
    }

    ManualStepperMove::Goal goal;
    goal.operation_id = operation_id;
    goal.stepper = stepper;
    goal.target_position = target_position;
    goal.speed = speed;
    goal.acceleration = acceleration;
    goal.set_position_before_move = set_position_before_move;
    goal.initial_position = initial_position;
    goal.start_delay_s = 0.0;
    goal.disable_after = disable_after;

    auto state = std::make_shared<FeedbackState>();
    rclcpp_action::Client<ManualStepperMove>::SendGoalOptions options;
    options.feedback_callback =
      [state](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const ManualStepperMove::Feedback> feedback) {
        state->phase.store(feedback->phase);
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->detail = feedback->detail;
        }
      };

    RCLCPP_INFO(
      node_->get_logger(),
      "Requesting Klipper move through ROS bridge: stepper=%s target=%.6f speed=%.6f accel=%.6f",
      stepper.c_str(), target_position, speed, acceleration);
    auto goal_future = action_client_->async_send_goal(goal, options);
    const auto accept_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (goal_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
      checkWatchdog(watchdog);
      if (std::chrono::steady_clock::now() >= accept_deadline)
      {
        throw std::runtime_error("timed out waiting for the ROS Klipper bridge to accept the move");
      }
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      throw std::runtime_error("ROS Klipper bridge rejected the manual-stepper move");
    }
    auto move = std::make_shared<ActiveMove>();
    move->operation_id = operation_id;
    move->state = state;
    move->handle = goal_handle;
    move->result_future = action_client_->async_get_result(goal_handle);
    active_ = move;

    try
    {
      const auto dispatch_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (
        state->phase.load() <
        static_cast<uint8_t>(ManualStepperMove::Feedback::PHASE_DISPATCHED))
      {
        checkWatchdog(watchdog);
        if (move->result_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
          const auto wrapped = move->result_future.get();
          const auto message = wrapped.result ? wrapped.result->message :
            std::string("the bridge returned no result");
          throw std::runtime_error(
                  "Klipper move ended before dispatch: " + message);
        }
        if (std::chrono::steady_clock::now() >= dispatch_deadline)
        {
          throw std::runtime_error("timed out waiting for Klipper DISPATCHED feedback");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      active_->dispatched = true;
      any_motion_dispatched_ = true;
      RCLCPP_INFO(
        node_->get_logger(), "Klipper move accepted and dispatched by the ROS bridge: %s",
        feedbackDetail(state).c_str());
    }
    catch (...)
    {
      cancelActiveAndEmergencyNoThrow();
      throw;
    }
  }

  void ensureActiveHealthy() const
  {
    if (!active_)
    {
      return;
    }
    if (active_->result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return;
    }
    const auto wrapped = active_->result_future.get();
    if (
      wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped.result || !wrapped.result->success)
    {
      const auto message = wrapped.result ? wrapped.result->message :
        std::string("the bridge returned no result");
      throw std::runtime_error("Klipper move failed while the UR was active: " + message);
    }
  }

  void waitForActiveCompletion(const Watchdog & watchdog)
  {
    if (!active_)
    {
      throw std::runtime_error("there is no active Klipper move to wait for");
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(315);
    while (active_->result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
      checkWatchdog(watchdog);
      if (std::chrono::steady_clock::now() >= deadline)
      {
        cancelActiveAndEmergencyNoThrow();
        throw std::runtime_error("timed out waiting for Klipper physical motion completion");
      }
    }
    const auto wrapped = active_->result_future.get();
    if (
      wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped.result || !wrapped.result->success)
    {
      const auto message = wrapped.result ? wrapped.result->message :
        std::string("the bridge returned no result");
      throw std::runtime_error("Klipper move failed: " + message);
    }
    RCLCPP_INFO(
      node_->get_logger(), "Klipper physical move completed: %s",
      wrapped.result->message.c_str());
    active_.reset();
  }

  bool hasActiveDispatchedMotion() const
  {
    return active_ && active_->dispatched;
  }

  bool anyMotionWasDispatched() const
  {
    return any_motion_dispatched_;
  }

  void cancelActiveAndEmergencyNoThrow()
  {
    if (active_ && active_->handle)
    {
      try
      {
        auto cancel_future = action_client_->async_cancel_goal(active_->handle);
        cancel_future.wait_for(std::chrono::seconds(2));
      }
      catch (const std::exception & ex)
      {
        RCLCPP_ERROR(node_->get_logger(), "Could not cancel the Klipper ROS action: %s", ex.what());
      }
    }
    if (hasActiveDispatchedMotion())
    {
      emergencyStopNoThrow();
    }
    active_.reset();
  }

  void disableNoThrow()
  {
    try
    {
      const auto response = callTrigger(
        disable_client_, "/klipper/disable_stepper", std::chrono::seconds(5));
      if (!response->success)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Klipper stepper disable was rejected: %s",
          response->message.c_str());
      }
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not disable the Klipper stepper: %s", ex.what());
    }
  }

  void emergencyStopNoThrow()
  {
    try
    {
      const auto response = callTrigger(
        emergency_client_, "/klipper/emergency_stop", std::chrono::seconds(5));
      if (!response->success)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Klipper emergency stop was rejected: %s",
          response->message.c_str());
      }
      else
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Klipper emergency stop was issued. Firmware restart is required before reuse.");
      }
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not issue Klipper emergency stop: %s", ex.what());
    }
  }

private:
  struct FeedbackState
  {
    std::atomic<uint8_t> phase{0};
    std::mutex mutex;
    std::string detail;
  };

  struct ActiveMove
  {
    std::string operation_id;
    std::shared_ptr<FeedbackState> state;
    GoalHandle::SharedPtr handle;
    std::shared_future<GoalHandle::WrappedResult> result_future;
    bool dispatched{false};
  };

  void checkWatchdog(const Watchdog & watchdog) const
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("ROS shutdown requested during synchronized motion");
    }
    if (watchdog)
    {
      watchdog();
    }
  }

  std::string feedbackDetail(const std::shared_ptr<FeedbackState> & state) const
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    return state->detail;
  }

  Trigger::Response::SharedPtr callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & service_name,
    const std::chrono::seconds & timeout)
  {
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      throw std::runtime_error(service_name + " service is unavailable");
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (future.wait_for(timeout) != std::future_status::ready)
    {
      throw std::runtime_error("timed out waiting for " + service_name);
    }
    return future.get();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ManualStepperMove>::SharedPtr action_client_;
  rclcpp::Client<Trigger>::SharedPtr refresh_client_;
  rclcpp::Client<Trigger>::SharedPtr disable_client_;
  rclcpp::Client<Trigger>::SharedPtr emergency_client_;
  std::shared_ptr<ActiveMove> active_;
  bool any_motion_dispatched_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur3e_cartesian_xyz_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  std::atomic<bool> cancel_requested{false};
  auto cancel_service = node->create_service<std_srvs::srv::Trigger>(
    "/synchronized_motion/cancel",
    [&cancel_requested](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      cancel_requested.store(true);
      response->success = true;
      response->message =
        "Synchronized cancellation accepted; canceling UR and Klipper through ROS.";
    });
  (void)cancel_service;
  std::shared_ptr<KlipperActionClient> klipper;

  try
  {
    node->declare_parameter<std::string>("axes", "x,y,z");
    node->declare_parameter<double>("distance_m", 0.05);
    node->declare_parameter<double>("eef_step_m", 0.005);
    node->declare_parameter<double>("jump_threshold", 0.0);
    node->declare_parameter<double>("velocity_scaling", 0.05);
    node->declare_parameter<double>("acceleration_scaling", 0.05);
    node->declare_parameter<double>("max_segment_delta_rad", 0.010);
    node->declare_parameter<double>("max_joint_speed_rad_s", 0.080);
    node->declare_parameter<double>("min_segment_duration_s", 0.250);
    node->declare_parameter<double>("goal_tolerance_rad", 0.025);
    node->declare_parameter<double>("goal_velocity_tolerance_rad_s", 0.10);
    node->declare_parameter<bool>("smooth_execution", true);
    node->declare_parameter<bool>("include_velocity_setpoints", true);
    node->declare_parameter<std::string>("execution_controller", "passthrough_trajectory_controller");
    node->declare_parameter<bool>("switch_controller", true);
    node->declare_parameter<bool>("execute", false);
    node->declare_parameter<bool>("return_to_start", false);
    node->declare_parameter<double>("klipper_outbound_duration_s", 0.0);
    node->declare_parameter<bool>("synchronize_klipper", false);
    node->declare_parameter<bool>("klipper_check_only", false);
    node->declare_parameter<std::string>("klipper_stepper", "extruder");
    node->declare_parameter<double>("klipper_target_position", 60.0);
    node->declare_parameter<double>("klipper_speed", 5.0);
    node->declare_parameter<double>("klipper_acceleration", 20.0);
    node->declare_parameter<bool>("klipper_reset_position", true);
    node->declare_parameter<bool>("klipper_disable_after", true);
    node->declare_parameter<bool>("height_trial_mode", false);
    node->declare_parameter<double>("height_trial_surface_z_m", 0.0);
    node->declare_parameter<double>("height_trial_min_x_m", 0.0);
    node->declare_parameter<double>("height_trial_max_x_m", 0.0);
    node->declare_parameter<double>("height_trial_min_y_m", 0.0);
    node->declare_parameter<double>("height_trial_max_y_m", 0.0);
    // 10 m is an intentionally invalid sentinel: old callers must fail closed.
    node->declare_parameter<double>("height_trial_tcp_x_m", 10.0);
    node->declare_parameter<double>("height_trial_tcp_y_m", 10.0);
    node->declare_parameter<double>("height_trial_tcp_z_m", 10.0);
    node->declare_parameter<std::string>("printhead_geometry_sha256", "");
    node->declare_parameter<std::string>("planning_group", "ur_manipulator");
    node->declare_parameter<std::string>("end_effector_link", "tool0");
    node->declare_parameter<std::string>("robot_description", "");
    node->declare_parameter<std::string>("robot_description_semantic", "");

    const auto axes = parseAxes(node->get_parameter("axes").as_string());
    const double distance_m = node->get_parameter("distance_m").as_double();
    const double eef_step_m = node->get_parameter("eef_step_m").as_double();
    const double jump_threshold = node->get_parameter("jump_threshold").as_double();
    const double velocity_scaling = node->get_parameter("velocity_scaling").as_double();
    const double acceleration_scaling = node->get_parameter("acceleration_scaling").as_double();
    const double max_segment_delta_rad = node->get_parameter("max_segment_delta_rad").as_double();
    const double max_joint_speed_rad_s = node->get_parameter("max_joint_speed_rad_s").as_double();
    const double min_segment_duration_s = node->get_parameter("min_segment_duration_s").as_double();
    const double goal_tolerance_rad = node->get_parameter("goal_tolerance_rad").as_double();
    const double goal_velocity_tolerance_rad_s =
      node->get_parameter("goal_velocity_tolerance_rad_s").as_double();
    const bool smooth_execution = node->get_parameter("smooth_execution").as_bool();
    const bool include_velocity_setpoints =
      node->get_parameter("include_velocity_setpoints").as_bool();
    const std::string execution_controller =
      node->get_parameter("execution_controller").as_string();
    const bool switch_controller = node->get_parameter("switch_controller").as_bool();
    const bool execute = node->get_parameter("execute").as_bool();
    const bool return_to_start = node->get_parameter("return_to_start").as_bool();
    const double klipper_outbound_duration_s =
      node->get_parameter("klipper_outbound_duration_s").as_double();
    const bool synchronize_klipper = node->get_parameter("synchronize_klipper").as_bool();
    const bool klipper_check_only = node->get_parameter("klipper_check_only").as_bool();
    const std::string klipper_stepper = node->get_parameter("klipper_stepper").as_string();
    const double klipper_target_position =
      node->get_parameter("klipper_target_position").as_double();
    const double klipper_speed = node->get_parameter("klipper_speed").as_double();
    const double klipper_acceleration = node->get_parameter("klipper_acceleration").as_double();
    const bool klipper_reset_position =
      node->get_parameter("klipper_reset_position").as_bool();
    const bool klipper_disable_after =
      node->get_parameter("klipper_disable_after").as_bool();
    const std::string planning_group = node->get_parameter("planning_group").as_string();
    const std::string end_effector_link = node->get_parameter("end_effector_link").as_string();

    const double max_distance_m = envIsYes("ALLOW_15CM_CARTESIAN_MOVE") ? 0.15 : 0.10;
    if (
      !std::isfinite(distance_m) || std::abs(distance_m) <= 0.0 ||
      std::abs(distance_m) > max_distance_m)
    {
      std::ostringstream error;
      error << "distance_m must be finite, nonzero, and <= " << max_distance_m << " m";
      if (max_distance_m < 0.15)
      {
        error << " unless ALLOW_15CM_CARTESIAN_MOVE=YES";
      }
      throw std::runtime_error(error.str());
    }
    if (!std::isfinite(eef_step_m) || eef_step_m <= 0.0 || eef_step_m > 0.02)
    {
      throw std::runtime_error("eef_step_m must be > 0 and <= 0.02 m");
    }
    if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 || velocity_scaling > 0.20)
    {
      throw std::runtime_error("velocity_scaling must be > 0 and <= 0.20");
    }
    if (
      !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 ||
      acceleration_scaling > 0.20)
    {
      throw std::runtime_error("acceleration_scaling must be > 0 and <= 0.20");
    }
    if (
      !std::isfinite(max_segment_delta_rad) || max_segment_delta_rad <= 0.0 ||
      max_segment_delta_rad > 0.025)
    {
      throw std::runtime_error("max_segment_delta_rad must be > 0 and <= 0.025 rad");
    }
    if (
      !std::isfinite(max_joint_speed_rad_s) || max_joint_speed_rad_s <= 0.0 ||
      max_joint_speed_rad_s > 0.20)
    {
      throw std::runtime_error("max_joint_speed_rad_s must be > 0 and <= 0.20 rad/s");
    }
    if (
      !std::isfinite(min_segment_duration_s) || min_segment_duration_s < 0.10 ||
      min_segment_duration_s > 2.00)
    {
      throw std::runtime_error("min_segment_duration_s must be between 0.10 and 2.00 s");
    }
    if (!std::isfinite(goal_tolerance_rad) || goal_tolerance_rad <= 0.0 || goal_tolerance_rad > 0.05)
    {
      throw std::runtime_error("goal_tolerance_rad must be > 0 and <= 0.05 rad");
    }
    if (
      !std::isfinite(goal_velocity_tolerance_rad_s) || goal_velocity_tolerance_rad_s <= 0.0 ||
      goal_velocity_tolerance_rad_s > 0.50)
    {
      throw std::runtime_error("goal_velocity_tolerance_rad_s must be > 0 and <= 0.50 rad/s");
    }
    if (!isSupportedTrajectoryController(execution_controller))
    {
      throw std::runtime_error(
        "execution_controller must be passthrough_trajectory_controller or "
        "scaled_joint_trajectory_controller");
    }
    if (smooth_execution && execution_controller == "scaled_joint_trajectory_controller")
    {
      throw std::runtime_error(
        "refusing smooth_execution on scaled_joint_trajectory_controller; use "
        "passthrough_trajectory_controller");
    }
    if (
      !std::isfinite(klipper_outbound_duration_s) || klipper_outbound_duration_s < 0.0 ||
      klipper_outbound_duration_s > 300.0)
    {
      throw std::runtime_error("klipper_outbound_duration_s must be between 0 and 300 seconds");
    }
    if (synchronize_klipper)
    {
      const auto suffix = klipper_stepper.size() >= 8 ? klipper_stepper.substr(8) : "";
      if (klipper_stepper != "extruder" &&
          (klipper_stepper.rfind("extruder", 0) != 0 || suffix.empty() ||
           suffix.front() < '1' || suffix.front() > '9' ||
           !std::all_of(suffix.begin(), suffix.end(),
             [](const unsigned char value) {return value >= '0' && value <= '9';})))
      {
        throw std::runtime_error("klipper_stepper must select extruder or extruder1, extruder2, ...");
      }
      if (
        !std::isfinite(klipper_target_position) ||
        klipper_target_position < -200.0 || klipper_target_position > 200.0)
      {
        throw std::runtime_error("klipper_target_position must be between -200 and 200");
      }
      if (!std::isfinite(klipper_speed) || klipper_speed < 0.1 || klipper_speed > 5.0)
      {
        throw std::runtime_error("klipper_speed must be between 0.1 and 5 mm/s");
      }
      if (
        !std::isfinite(klipper_acceleration) ||
        std::abs(klipper_acceleration - 20.0) > 1e-8)
      {
        throw std::runtime_error("klipper_acceleration must match native extruder acceleration: 20 mm/s^2");
      }
      if (!klipper_reset_position)
      {
        throw std::runtime_error(
                "native extrusion requires klipper_reset_position for an explicit E displacement");
      }
      if (execute && !envIsYes("CONFIRM_KLIPPER_STEPPER"))
      {
        throw std::runtime_error(
                "refusing synchronized execution: set CONFIRM_KLIPPER_STEPPER=YES");
      }
    }
    if (execute && (!envIsYes("CONFIRM_REAL_ROBOT_MOVE") || !envIsYes("CONFIRM_CARTESIAN_MOVE")))
    {
      throw std::runtime_error(
        "refusing to execute: set CONFIRM_REAL_ROBOT_MOVE=YES and CONFIRM_CARTESIAN_MOVE=YES");
    }

    if (synchronize_klipper)
    {
      klipper = std::make_shared<KlipperActionClient>(node);
      klipper->refresh();
      if (klipper_check_only)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "ROS Klipper bridge check complete; no UR planning and no hardware motion occurred.");
        executor.cancel();
        if (spinner.joinable())
        {
          spinner.join();
        }
        rclcpp::shutdown();
        return 0;
      }
    }

    loadMatchingRobotDescriptions(node);

    ProgramState program_state(node);
    ActualState actual_state(node);
    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink(end_effector_link);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group.setPlanningTime(10.0);
    if (node->get_parameter("height_trial_mode").as_bool())
    {
      ur3e_print_planner::requirePrintheadGeometryIdentity(
        node->get_parameter("printhead_geometry_sha256").as_string());
      if (axes.size() != 1 || axes.front() != "z" ||
          node->get_parameter("synchronize_klipper").as_bool())
      {
        throw std::runtime_error("height trial requires UR-only Z motion");
      }
      installHeightTrialCollisionGeometry(
        node, move_group, end_effector_link,
        node->get_parameter("height_trial_surface_z_m").as_double(),
        node->get_parameter("height_trial_min_x_m").as_double(),
        node->get_parameter("height_trial_max_x_m").as_double(),
        node->get_parameter("height_trial_min_y_m").as_double(),
        node->get_parameter("height_trial_max_y_m").as_double(),
        node->get_parameter("height_trial_tcp_x_m").as_double(),
        node->get_parameter("height_trial_tcp_y_m").as_double(),
        node->get_parameter("height_trial_tcp_z_m").as_double());
    }

    auto current_state = move_group.getCurrentState(10.0);
    if (!current_state)
    {
      throw std::runtime_error("failed to receive current robot state");
    }
    move_group.setStartState(*current_state);

    const auto start_pose_stamped = move_group.getCurrentPose(end_effector_link);
    geometry_msgs::msg::Pose cursor = start_pose_stamped.pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(axes.size());

    RCLCPP_INFO(
      node->get_logger(), "Start TCP pose in %s: x=%.4f y=%.4f z=%.4f",
      start_pose_stamped.header.frame_id.c_str(), cursor.position.x, cursor.position.y,
      cursor.position.z);

    for (const auto & axis : axes)
    {
      applyAxis(cursor, axis, distance_m);
      waypoints.push_back(cursor);
      RCLCPP_INFO(
        node->get_logger(), "After %+.3f m %s: x=%.4f y=%.4f z=%.4f", distance_m,
        axis.c_str(), cursor.position.x, cursor.position.y, cursor.position.z);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction =
      move_group.computeCartesianPath(waypoints, eef_step_m, jump_threshold, trajectory, true);
    RCLCPP_INFO(node->get_logger(), "MoveIt Cartesian path fraction: %.3f", fraction);
    if (fraction < 0.99)
    {
      throw std::runtime_error("refusing: MoveIt could not compute at least 99% of the Cartesian path");
    }

    robot_trajectory::RobotTrajectory robot_trajectory(move_group.getRobotModel(), planning_group);
    robot_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
          robot_trajectory, velocity_scaling, acceleration_scaling))
    {
      throw std::runtime_error("failed to time-parameterize Cartesian trajectory");
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory);

    const auto & points = trajectory.joint_trajectory.points;
    if (points.empty())
    {
      throw std::runtime_error("planned trajectory contains no points");
    }
    const auto & joint_names = trajectory.joint_trajectory.joint_names;
    const auto & first_positions = points.front().positions;
    const auto & last_positions = points.back().positions;
    if (joint_names.size() != first_positions.size() || joint_names.size() != last_positions.size())
    {
      throw std::runtime_error("planned trajectory joint names and point positions do not match");
    }
    double max_abs_delta = 0.0;
    RCLCPP_INFO(node->get_logger(), "Planned joint deltas:");
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      const double delta = last_positions[i] - first_positions[i];
      max_abs_delta = std::max(max_abs_delta, std::abs(delta));
      RCLCPP_INFO(
        node->get_logger(), "  %s: start %.6f -> end %.6f, delta %.6f rad",
        joint_names[i].c_str(), first_positions[i], last_positions[i], delta);
    }
    RCLCPP_INFO(node->get_logger(), "Max abs planned joint delta: %.6f rad", max_abs_delta);
    if (max_abs_delta < 0.005)
    {
      throw std::runtime_error(
        "refusing: planned joint deltas are too small to produce visible Cartesian motion");
    }

    const auto & total = points.back().time_from_start;
    const double total_s = static_cast<double>(total.sec) + static_cast<double>(total.nanosec) * 1e-9;
    RCLCPP_INFO(
      node->get_logger(), "Timed trajectory points: %zu, duration %.3f s", points.size(), total_s);

    if (!execute)
    {
      if (return_to_start)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Return-to-start plan validated by reversing the guarded outbound trajectory.");
      }
      RCLCPP_INFO(node->get_logger(), "Plan-only complete; no trajectory was executed.");
    }
    else
    {
      if (!program_state.waitUntilRunning(node, 5.0))
      {
        throw std::runtime_error("External Control program is not running; do not execute");
      }
      if (switch_controller)
      {
        switchTrajectoryController(node, execution_controller);
      }

      GuardedTrajectoryExecutor guarded_executor(node, execution_controller);
      guarded_executor.prepare(joint_names, points, actual_state);
      const auto synchronized_watchdog = [&]() {
          if (cancel_requested.load())
          {
            throw std::runtime_error("synchronized motion canceled by the operator");
          }
          if (!program_state.isRunning())
          {
            throw std::runtime_error("External Control stopped during synchronized motion");
          }
          if (klipper)
          {
            klipper->ensureActiveHealthy();
          }
        };

      if (cancel_requested.load())
      {
        throw std::runtime_error("synchronized motion canceled before hardware dispatch");
      }
      if (klipper)
      {
        const auto operation_base = std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count());
        klipper->sendMove(
          operation_base + "-outbound", klipper_stepper, klipper_target_position,
          klipper_speed, klipper_acceleration, klipper_reset_position, 0.0,
          !return_to_start && klipper_disable_after, synchronized_watchdog);
      }

      const auto tcp_before = actual_state.tcpPose().pose.position;
      RCLCPP_WARN(
        node->get_logger(),
        "ROS coordinator is executing the guarded UR trajectory through %s: "
        "max segment %.4f rad, max speed %.4f rad/s",
        execution_controller.c_str(), max_segment_delta_rad, max_joint_speed_rad_s);
      const auto synchronized_start = std::chrono::steady_clock::now();
      guarded_executor.execute(
        joint_names, points, actual_state, max_segment_delta_rad, max_joint_speed_rad_s,
        min_segment_duration_s, goal_tolerance_rad, goal_velocity_tolerance_rad_s,
        smooth_execution, include_velocity_setpoints, synchronized_watchdog);
      const auto tcp_after = actual_state.tcpPose().pose.position;
      RCLCPP_INFO(
        node->get_logger(), "Observed TCP delta: dx=%.4f dy=%.4f dz=%.4f m",
        tcp_after.x - tcp_before.x, tcp_after.y - tcp_before.y, tcp_after.z - tcp_before.z);
      RCLCPP_INFO(node->get_logger(), "Guarded Cartesian execution completed.");

      if (klipper)
      {
        klipper->waitForActiveCompletion(synchronized_watchdog);
      }

      if (return_to_start)
      {
        if (!klipper)
        {
          const double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - synchronized_start).count();
          if (elapsed_s < klipper_outbound_duration_s)
          {
            const double remaining_s = klipper_outbound_duration_s - elapsed_s;
            RCLCPP_INFO(
              node->get_logger(),
              "Waiting %.3f s for the legacy external outbound move before return.",
              remaining_s);
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining_s));
          }
        }
        if (!program_state.waitUntilRunning(node, 5.0))
        {
          throw std::runtime_error("External Control stopped before return-to-start could begin");
        }

        auto return_points = points;
        std::reverse(return_points.begin(), return_points.end());
        guarded_executor.prepare(joint_names, return_points, actual_state);
        if (klipper)
        {
          const auto operation_id = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + "-return";
          klipper->sendMove(
            operation_id, klipper_stepper, 0.0, klipper_speed, klipper_acceleration,
            true, klipper_target_position, klipper_disable_after, synchronized_watchdog);
        }
        RCLCPP_WARN(
          node->get_logger(),
          "ROS coordinator is returning the guarded UR trajectory to synchronized start through %s.",
          execution_controller.c_str());
        guarded_executor.execute(
          joint_names, return_points, actual_state, max_segment_delta_rad,
          max_joint_speed_rad_s, min_segment_duration_s, goal_tolerance_rad,
          goal_velocity_tolerance_rad_s, smooth_execution, include_velocity_setpoints,
          synchronized_watchdog);
        if (klipper)
        {
          klipper->waitForActiveCompletion(synchronized_watchdog);
        }
        const auto tcp_returned = actual_state.tcpPose().pose.position;
        RCLCPP_INFO(
          node->get_logger(),
          "Return-to-start TCP error: dx=%.4f dy=%.4f dz=%.4f m",
          tcp_returned.x - tcp_before.x, tcp_returned.y - tcp_before.y,
          tcp_returned.z - tcp_before.z);
        RCLCPP_INFO(node->get_logger(), "Guarded return-to-start completed.");
      }
    }
  }
  catch (const std::exception & ex)
  {
    if (klipper)
    {
      if (klipper->hasActiveDispatchedMotion())
      {
        klipper->cancelActiveAndEmergencyNoThrow();
      }
      else if (klipper->anyMotionWasDispatched())
      {
        klipper->disableNoThrow();
      }
    }
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
