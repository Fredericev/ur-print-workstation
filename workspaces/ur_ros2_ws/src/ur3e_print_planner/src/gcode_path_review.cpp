#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "printhead_collision_geometry.hpp"
#include <Eigen/Geometry>

#include "../include/approach_policy.hpp"
#include "../include/trajectory_safety.hpp"

namespace
{

struct PathPoint
{
  long sequence{};
  long line{};
  double x{};
  double y{};
  double z{};
  std::string kind;
  double cumulative_s{};
};

constexpr size_t kMaximumPathPoints = 300000;
constexpr size_t kCartesianChunkPoints = 750;
// A 2 m downward guard extends beyond the complete reachable workspace of a
// UR3e. Within the confirmed tabletop footprint, every point below the
// measured surface is therefore collision space rather than free space.
constexpr double kBelowTableGuardDepthM = 2.0;
// Every approach first reaches this height above the print start and only then
// descends vertically. This prevents a valid-but-unintuitive low side entry.
constexpr double kApproachClearanceM = 0.12;
constexpr char kPlanningPolicyVersion[] = "continuous-cartesian-approach-v4";
constexpr double kCompletePathFraction = 1.0 - 1.0e-6;

std::string jsonEscape(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value)
  {
    switch (character)
    {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20)
        {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(character) << std::dec;
        }
        else
        {
          escaped << character;
        }
    }
  }
  return escaped.str();
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ','))
  {
    values.push_back(value);
  }
  return values;
}

std::vector<PathPoint> readPath(const std::string & path)
{
  std::ifstream input(path);
  if (!input)
  {
    throw std::runtime_error("could not open the calibrated path file");
  }
  std::string header;
  std::getline(input, header);
  if (header != "sequence,line,x,y,z,kind,cumulative_s")
  {
    throw std::runtime_error("the calibrated path file has an invalid header");
  }

  std::vector<PathPoint> points;
  std::string line;
  while (std::getline(input, line))
  {
    if (line.empty())
    {
      continue;
    }
    const auto values = splitCsv(line);
    if (values.size() != 7)
    {
      throw std::runtime_error("the calibrated path file contains an invalid row");
    }
    PathPoint point;
    point.sequence = std::stol(values[0]);
    point.line = std::stol(values[1]);
    point.x = std::stod(values[2]);
    point.y = std::stod(values[3]);
    point.z = std::stod(values[4]);
    point.kind = values[5];
    point.cumulative_s = std::stod(values[6]);
    if (
      !std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.cumulative_s))
    {
      throw std::runtime_error("the calibrated path contains a non-finite value");
    }
    points.push_back(point);
    if (points.size() > kMaximumPathPoints)
    {
      throw std::runtime_error("the calibrated path exceeds the 300000-waypoint review limit");
    }
  }
  if (points.size() < 2)
  {
    throw std::runtime_error("the calibrated path needs at least two waypoints");
  }
  return points;
}

void loadMatchingRobotModel(
  const rclcpp::Node::SharedPtr & node,
  const std::string & planning_group)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(std::chrono::seconds(5)))
  {
    throw std::runtime_error(
      "MoveIt parameter service is unavailable; start MoveIt before validation");
  }
  const std::string kinematics_prefix =
    "robot_description_kinematics." + planning_group + ".";
  const std::vector<std::string> parameter_names = {
    "robot_description",
    "robot_description_semantic",
    kinematics_prefix + "kinematics_solver",
    kinematics_prefix + "kinematics_solver_search_resolution",
    kinematics_prefix + "kinematics_solver_timeout",
    kinematics_prefix + "kinematics_solver_attempts",
  };
  auto future = client->get_parameters(
    parameter_names);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
  {
    throw std::runtime_error("timed out while loading the robot model from /move_group");
  }
  const auto parameters = future.get();
  if (
    parameters.size() != parameter_names.size() ||
    parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    parameters[2].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    parameters[3].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
    parameters[4].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
    parameters[5].get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
    parameters[0].as_string().empty() || parameters[1].as_string().empty() ||
    parameters[2].as_string().empty())
  {
    throw std::runtime_error(
      "/move_group did not provide a complete URDF, SRDF, and kinematics model");
  }
  for (size_t index = 0; index < parameter_names.size(); ++index)
  {
    if (!node->has_parameter(parameter_names[index]))
    {
      node->declare_parameter(
        parameter_names[index],
        parameters[index].get_parameter_value());
    }
  }
  const auto results = node->set_parameters(parameters);
  if (
    results.size() != parameters.size() ||
    std::any_of(
      results.begin(), results.end(),
      [](const auto & result) {return !result.successful;}))
  {
    throw std::runtime_error(
      "failed to apply the matching MoveIt robot and kinematics model");
  }
}

std::vector<size_t> sampleIndexes(const size_t length, const size_t maximum)
{
  if (length <= maximum)
  {
    std::vector<size_t> indexes(length);
    for (size_t index = 0; index < length; ++index)
    {
      indexes[index] = index;
    }
    return indexes;
  }
  std::vector<size_t> indexes;
  indexes.reserve(maximum);
  for (size_t index = 0; index < maximum; ++index)
  {
    const auto value = static_cast<size_t>(
      std::llround(
        static_cast<double>(index) * static_cast<double>(length - 1) /
        static_cast<double>(maximum - 1)));
    if (indexes.empty() || indexes.back() != value)
    {
      indexes.push_back(value);
    }
  }
  return indexes;
}

double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

double maxJointStep(
  const trajectory_msgs::msg::JointTrajectory::_points_type & points)
{
  double maximum = 0.0;
  for (size_t point_index = 1; point_index < points.size(); ++point_index)
  {
    const auto & previous = points[point_index - 1].positions;
    const auto & current = points[point_index].positions;
    if (previous.size() != current.size())
    {
      throw std::runtime_error("MoveIt returned inconsistent joint dimensions");
    }
    for (size_t joint_index = 0; joint_index < current.size(); ++joint_index)
    {
      maximum = std::max(
        maximum,
        std::abs(current[joint_index] - previous[joint_index]));
    }
  }
  return maximum;
}

struct ExpectedWorkcellGeometry
{
  double bed_min_x;
  double bed_max_x;
  double bed_min_y;
  double bed_max_y;
  double bed_z;
  Eigen::Vector3d tcp;
  double tool_radius;
  std::string tool_link;
  bool require_measured_tool;
};

bool geometryNear(double actual, double expected)
{
  return std::isfinite(actual) && std::isfinite(expected) &&
    std::abs(actual - expected) <= 1.0e-6;
}

Eigen::Isometry3d checkedPoseTransform(
  const geometry_msgs::msg::Pose & pose, const char * label)
{
  const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite() ||
    !geometryNear(orientation.norm(), 1.0))
  {
    throw std::runtime_error(std::string(label) + " has invalid position or orientation");
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(position);
  transform.rotate(orientation);
  return transform;
}

void requireMatchingSceneGeometry(
  const moveit_msgs::msg::PlanningScene & scene_msg,
  const planning_scene::PlanningScenePtr & scene,
  const moveit::core::RobotModelConstPtr & model,
  const ExpectedWorkcellGeometry & expected)
{
  const auto & world_objects = scene_msg.world.collision_objects;
  const auto table = std::find_if(world_objects.begin(), world_objects.end(),
      [](const auto & object) {return object.id == "confirmed_table_surface_guard";});
  if (table == world_objects.end() ||
    std::count_if(world_objects.begin(), world_objects.end(),
      [](const auto & object) {return object.id == "confirmed_table_surface_guard";}) != 1 ||
    table->primitives.size() != 1 || table->primitive_poses.size() != 1 ||
    !table->meshes.empty() || !table->mesh_poses.empty() ||
    !table->planes.empty() || !table->plane_poses.empty())
  {
    throw std::runtime_error("planning scene table guard shape is missing or ambiguous");
  }
  const auto & table_box = table->primitives.front();
  if (table_box.type != shape_msgs::msg::SolidPrimitive::BOX ||
    table_box.dimensions.size() != 3 ||
    !geometryNear(table_box.dimensions[0], expected.bed_max_x - expected.bed_min_x) ||
    !geometryNear(table_box.dimensions[1], expected.bed_max_y - expected.bed_min_y) ||
    !geometryNear(table_box.dimensions[2], kBelowTableGuardDepthM))
  {
    throw std::runtime_error("planning scene table guard dimensions differ from this review");
  }
  const auto & model_T_base = scene->getCurrentState().getGlobalLinkTransform("base_link");
  if (table->header.frame_id != "base_link" &&
    !(table->header.frame_id == model->getModelFrame() &&
      model_T_base.matrix().isApprox(Eigen::Matrix4d::Identity(), 1.0e-9)))
  {
    throw std::runtime_error("planning scene table guard frame differs from base_link");
  }
  const auto table_pose = checkedPoseTransform(table->pose, "table parent pose") *
    checkedPoseTransform(table->primitive_poses.front(), "table shape pose");
  const Eigen::Vector3d table_center(
    (expected.bed_min_x + expected.bed_max_x) / 2.0,
    (expected.bed_min_y + expected.bed_max_y) / 2.0,
    expected.bed_z - kBelowTableGuardDepthM / 2.0);
  if ((table_pose.translation() - table_center).cwiseAbs().maxCoeff() > 1.0e-6 ||
    !table_pose.rotation().isApprox(Eigen::Matrix3d::Identity(), 1.0e-6))
  {
    throw std::runtime_error("planning scene table guard position differs from this review");
  }

  if (!expected.require_measured_tool)
  {
    return;
  }
  if (!ur3e_print_planner::tcpInsidePrintheadEnvelope(
      expected.tcp.x(), expected.tcp.y(), expected.tcp.z()))
  {
    throw std::runtime_error("measured TCP falls outside the installed printhead CAD envelope");
  }
  const auto & attachments = scene_msg.robot_state.attached_collision_objects;
  const auto attached = std::find_if(attachments.begin(), attachments.end(),
      [](const auto & item) {return item.object.id == "measured_print_tool";});
  if (attached == attachments.end() ||
    std::count_if(attachments.begin(), attachments.end(),
      [](const auto & item) {return item.object.id == "measured_print_tool";}) != 1 ||
    !scene->getCurrentState().hasAttachedBody("measured_print_tool"))
  {
    throw std::runtime_error("planning scene is missing the measured print-tool attachment");
  }
  const auto & tool = attached->object;
  const std::set<std::string> touch_links(
    attached->touch_links.begin(), attached->touch_links.end());
  if (attached->link_name != expected.tool_link ||
    tool.header.frame_id != expected.tool_link ||
    attached->touch_links.size() != 3 ||
    touch_links != std::set<std::string>{expected.tool_link, "flange", "wrist_3_link"} ||
    tool.primitives.size() != 1 || tool.primitive_poses.size() != 1 ||
    !tool.meshes.empty() || !tool.mesh_poses.empty() ||
    !tool.planes.empty() || !tool.plane_poses.empty())
  {
    throw std::runtime_error("planning scene print-tool attachment frame or shapes differ");
  }
  const auto & head_box = tool.primitives.front();
  if (head_box.type != shape_msgs::msg::SolidPrimitive::BOX ||
    head_box.dimensions.size() != 3 ||
    !geometryNear(head_box.dimensions[0], ur3e_print_planner::kPrintheadBoxSizeX) ||
    !geometryNear(head_box.dimensions[1], ur3e_print_planner::kPrintheadBoxSizeY) ||
    !geometryNear(head_box.dimensions[2], ur3e_print_planner::kPrintheadBoxSizeZ))
  {
    throw std::runtime_error("planning scene print-tool dimensions differ from this review");
  }
  const auto tool_parent = checkedPoseTransform(tool.pose, "print-tool parent pose");
  const auto head_pose = tool_parent *
    checkedPoseTransform(tool.primitive_poses.front(), "printhead box pose");
  const Eigen::Vector3d expected_head_center(
    ur3e_print_planner::kPrintheadBoxCenterX,
    ur3e_print_planner::kPrintheadBoxCenterY,
    ur3e_print_planner::kPrintheadBoxCenterZ);
  if ((head_pose.translation() - expected_head_center).cwiseAbs().maxCoeff() > 1.0e-6 ||
    !head_pose.rotation().isApprox(Eigen::Matrix3d::Identity(), 1.0e-6))
  {
    throw std::runtime_error("planning scene print-tool position differs from this review");
  }
}

planning_scene::PlanningScenePtr loadValidationScene(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotModelConstPtr & model,
  const ExpectedWorkcellGeometry & expected)
{
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(10)))
  {
    throw std::runtime_error("planning scene service unavailable for final collision validation");
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  using Components = moveit_msgs::msg::PlanningSceneComponents;
  request->components.components = Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
    Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
  {
    throw std::runtime_error("timed out obtaining the complete planning scene for validation");
  }
  const auto response = future.get();
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  if (!scene->setPlanningSceneMsg(response->scene) ||
    !scene->getWorld()->hasObject("confirmed_table_surface_guard"))
  {
    throw std::runtime_error("incomplete collision scene or missing confirmed table guard");
  }
  // A successful scene-update request does not prove the returned snapshot
  // already contains this invocation's table height and tool calibration.
  requireMatchingSceneGeometry(response->scene, scene, model, expected);
  return scene;
}

double computeContinuousCartesianPath(
  const planning_scene::PlanningScenePtr & scene,
  const moveit::core::RobotState & start,
  const std::string & planning_group,
  const std::string & end_effector_link,
  const std::vector<geometry_msgs::msg::Pose> & targets,
  double eef_step,
  moveit_msgs::msg::RobotTrajectory & output)
{
  // GetCartesianPath's service applies TOTG resampling before returning. Its
  // geometric shortcuts can dip into the table between otherwise valid print
  // states. Keep every original IK state here; time parameterization below is
  // forbidden from changing that geometry.
  auto seed = scene->getCurrentState();
  seed.setVariablePositions(start.getVariablePositions());
  seed.update();
  const auto * group = seed.getJointModelGroup(planning_group);
  const auto * tip = seed.getLinkModel(end_effector_link);
  if (!group || !tip || targets.empty())
  {
    throw std::runtime_error("invalid Cartesian planning group, tool link or targets");
  }
  moveit::core::GroupStateValidityCallbackFn valid = [&scene](
    moveit::core::RobotState * state, const moveit::core::JointModelGroup * g, const double * q) {
      state->setJointGroupPositions(g, q);
      state->update();
      return state->satisfiesBounds(g) && scene->isStateValid(*state, "");
    };
  robot_trajectory::RobotTrajectory raw_trajectory(scene->getRobotModel(), planning_group);
  raw_trajectory.addSuffixWayPoint(seed, 0.0);
  const Eigen::Isometry3d model_T_base = seed.getGlobalLinkTransform("base_link");
  size_t completed = 0;
  for (const auto & pose : targets)
  {
    Eigen::Isometry3d base_T_target = Eigen::Isometry3d::Identity();
    base_T_target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    base_T_target.linear() = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).normalized()
      .toRotationMatrix();
    const Eigen::Isometry3d model_T_target = model_T_base * base_T_target;
    moveit::core::RobotState working(seed);
    std::vector<moveit::core::RobotStatePtr> segment;
    // The single-target overload also passes these absolute limits to IK.
    // Do not use the multi-target overload that disables the per-target check.
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &working, group, segment, tip, model_T_target, true,
      moveit::core::MaxEEFStep(eef_step),
      moveit::core::JumpThreshold(ur3e_print_planner::kMaximumJointStepRad, 0.0), valid);
    if (!std::isfinite(fraction) || fraction < kCompletePathFraction || segment.empty())
    {
      raw_trajectory.getRobotTrajectoryMsg(output);
      return static_cast<double>(completed) / targets.size();
    }
    for (size_t i = 1; i < segment.size(); ++i)
    {
      std::vector<double> previous;
      std::vector<double> next;
      raw_trajectory.getLastWayPoint().copyJointGroupPositions(group, previous);
      segment[i]->copyJointGroupPositions(group, next);
      if (previous != next)
      {
        raw_trajectory.addSuffixWayPoint(*segment[i], 0.0);
      }
    }
    // A failed IK search can mutate working beyond the accepted endpoint.
    // Only accepted states are allowed to seed the following target.
    seed = *segment.back();
    ++completed;
  }
  raw_trajectory.getRobotTrajectoryMsg(output);
  return 1.0;
}

void timeParameterizeWithoutChangingGeometry(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & planning_group,
  const moveit::core::RobotState & start,
  double velocity_scaling, double acceleration_scaling,
  moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto original = trajectory.joint_trajectory;
  robot_trajectory::RobotTrajectory timed(model, planning_group);
  timed.setRobotTrajectoryMsg(start, trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(timed, velocity_scaling, acceleration_scaling))
  {
    throw std::runtime_error("failed to time-parameterize the Cartesian trajectory");
  }
  timed.getRobotTrajectoryMsg(trajectory);
  const auto & result = trajectory.joint_trajectory;
  if (result.joint_names != original.joint_names || result.points.size() != original.points.size())
  {
    throw std::runtime_error("time parameterization changed the Cartesian joint path");
  }
  for (size_t i = 0; i < original.points.size(); ++i)
  {
    if (result.points[i].positions != original.points[i].positions)
    {
      throw std::runtime_error("time parameterization changed raw joint angles; re-review required");
    }
  }
}

void preparePositionOnlyTrajectory(trajectory_msgs::msg::JointTrajectory & trajectory)
{
  // The reviewed executor sends positions/time only (linear interpolation).
  // Check exactly the precision serialized to its CSV and display that same
  // geometry, not a spline induced by unused velocity/acceleration fields.
  for (auto & point : trajectory.points)
  {
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
    for (auto & position : point.positions)
    {
      position = std::round(position * 1.0e9) / 1.0e9;
    }
  }
}

ur3e_print_planner::JointPath jointPath(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  ur3e_print_planner::JointPath path;
  path.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points)
  {
    path.push_back(point.positions);
  }
  return path;
}

bool hasJointMotion(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.size() < 2)
  {
    return false;
  }
  const auto & first = trajectory.points.front().positions;
  for (size_t index = 1; index < trajectory.points.size(); ++index)
  {
    const auto & positions = trajectory.points[index].positions;
    if (positions.size() != first.size())
    {
      throw std::runtime_error("trajectory joint dimensions are inconsistent");
    }
    for (size_t joint = 0; joint < positions.size(); ++joint)
    {
      if (std::abs(positions[joint] - first[joint]) > 1.0e-12)
      {
        return true;
      }
    }
  }
  return false;
}

void makeStationaryApproachSerializable(trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty())
  {
    throw std::runtime_error("stationary approach has no reviewed joint state");
  }
  if (trajectory.points.size() == 1)
  {
    trajectory.points.push_back(trajectory.points.front());
  }
  constexpr int32_t kStationaryDurationNanoseconds = 50000000;
  for (size_t index = 0; index < trajectory.points.size(); ++index)
  {
    auto & stamp = trajectory.points[index].time_from_start;
    stamp.sec = 0;
    stamp.nanosec = static_cast<uint32_t>(
      kStationaryDurationNanoseconds * index / (trajectory.points.size() - 1));
  }
}

ur3e_print_planner::JointPathMetrics validateTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const planning_scene::PlanningScenePtr & scene,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotState & expected_start,
  const std::string & planning_group,
  bool approach)
{
  using namespace ur3e_print_planner;
  if (trajectory.joint_names.size() != 6)
  {
    throw std::runtime_error("review requires all six UR joint variables");
  }
  JointPath path;
  for (const auto & point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      throw std::runtime_error("trajectory joint dimensions are inconsistent");
    }
    path.push_back(point.positions);
  }
  const auto metrics = requireContinuousPath(path, approach);
  for (size_t j = 0; j < trajectory.joint_names.size(); ++j)
  {
    if (std::abs(path.front()[j] -
      expected_start.getVariablePosition(trajectory.joint_names[j])) > 1.0e-6)
    {
      throw std::runtime_error("trajectory does not start at its exact reviewed joint state");
    }
  }
  // Keep attached bodies from the full scene, while substituting each planned
  // joint state. Checking points alone missed the old forearm/wrist collision.
  auto state = scene->getCurrentState();
  const auto * group = state.getJointModelGroup(planning_group);
  if (!group)
  {
    throw std::runtime_error("unknown planning group during final validation");
  }
  const auto count = validateLinearInterpolation(path, [&](const auto & positions) {
      state.setVariablePositions(trajectory.joint_names, positions);
      state.update();
      const bool valid = state.satisfiesBounds(group) && scene->isStateValid(state, "");
      if (!valid)
      {
        collision_detection::CollisionRequest request;
        request.contacts = true;
        request.max_contacts = 5;
        collision_detection::CollisionResult result;
        scene->checkCollision(request, result, state);
        for (const auto & contact : result.contacts)
        {
          RCLCPP_ERROR(node->get_logger(), "Final interpolation collision: %s / %s",
            contact.first.first.c_str(), contact.first.second.c_str());
        }
        const auto tip = state.getGlobalLinkTransform("tool0").translation();
        RCLCPP_ERROR(node->get_logger(), "Rejected sample tool0 XYZ %.9f %.9f %.9f; bounds %s",
          tip.x(), tip.y(), tip.z(), state.satisfiesBounds(group) ? "passed" : "failed");
      }
      return valid;
    });
  RCLCPP_INFO(
    node->get_logger(), "%s final linear-interpolation validation: %zu samples, "
    "maximum joint spacing %.6f rad, maximum original step %.6f rad.",
    approach ? "Approach" : "Print", count, kCollisionSampleSpacingRad, metrics.max_step);
  return metrics;
}

std::vector<double> cartesianPathProgress(
  const moveit::core::RobotState & start_state,
  const std::string & end_effector_link,
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  moveit::core::RobotState state(start_state);
  std::vector<double> progress;
  progress.reserve(trajectory.points.size());
  Eigen::Vector3d previous;
  double cumulative_distance = 0.0;
  for (size_t index = 0; index < trajectory.points.size(); ++index)
  {
    const auto & point = trajectory.points[index];
    if (point.positions.size() != trajectory.joint_names.size())
    {
      throw std::runtime_error("trajectory position dimensions are inconsistent");
    }
    state.setVariablePositions(trajectory.joint_names, point.positions);
    state.update();
    const Eigen::Vector3d current =
      state.getGlobalLinkTransform(end_effector_link).translation();
    if (index > 0)
    {
      cumulative_distance += (current - previous).norm();
    }
    progress.push_back(cumulative_distance);
    previous = current;
  }
  if (progress.size() < 2)
  {
    throw std::runtime_error("trajectory needs at least two Cartesian progress points");
  }
  if (cumulative_distance <= 1.0e-12)
  {
    for (size_t index = 0; index < progress.size(); ++index)
    {
      progress[index] = static_cast<double>(index) /
        static_cast<double>(progress.size() - 1);
    }
  }
  else
  {
    for (auto & value : progress)
    {
      value /= cumulative_distance;
    }
  }
  progress.front() = 0.0;
  progress.back() = 1.0;
  return progress;
}

void writeTrajectoryCsv(
  const std::string & path,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & path_progress)
{
  if (path.empty())
  {
    throw std::runtime_error("trajectory output path is required");
  }
  if (
    trajectory.joint_names.size() != 6 || trajectory.points.empty() ||
    path_progress.size() != trajectory.points.size())
  {
    throw std::runtime_error("refusing to write an incomplete UR trajectory");
  }
  const std::filesystem::path destination(path);
  if (!destination.parent_path().empty())
  {
    std::filesystem::create_directories(destination.parent_path());
  }
  std::ofstream output(destination);
  if (!output)
  {
    throw std::runtime_error("could not open the full trajectory output file");
  }
  output << "time_s,path_progress";
  for (const auto & name : trajectory.joint_names)
  {
    output << "," << name;
  }
  output << "\n" << std::fixed << std::setprecision(9);
  double previous_time = -1.0;
  double previous_progress = -1.0;
  for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index)
  {
    const auto & point = trajectory.points[point_index];
    const double time_s = durationSeconds(point.time_from_start);
    if (!std::isfinite(time_s) || time_s < 0.0 || time_s <= previous_time)
    {
      throw std::runtime_error("trajectory timestamps must be finite and strictly increasing");
    }
    if (point.positions.size() != trajectory.joint_names.size())
    {
      throw std::runtime_error("trajectory position dimensions are inconsistent");
    }
    const double progress = path_progress[point_index];
    if (
      !std::isfinite(progress) || progress < 0.0 || progress > 1.0 ||
      progress < previous_progress)
    {
      throw std::runtime_error("trajectory Cartesian progress is invalid");
    }
    output << time_s << "," << progress;
    for (const double position : point.positions)
    {
      if (!std::isfinite(position))
      {
        throw std::runtime_error("trajectory contains a non-finite joint position");
      }
      output << "," << position;
    }
    output << "\n";
    previous_time = time_s;
    previous_progress = progress;
  }
}

void installFixedWorkcellCollisionGeometry(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & end_effector_link,
  const double bed_min_x,
  const double bed_max_x,
  const double bed_min_y,
  const double bed_max_y,
  const double bed_plane_z,
  const double tcp_x,
  const double tcp_y,
  const double tcp_z,
  const double tool_radius,
  const bool attach_measured_tool)
{
  // This is intentionally a forbidden half-space approximation, not a claim
  // about the table's physical thickness. Its top face is the measured surface
  // and its bottom lies beyond every pose the UR3e can reach.
  moveit::planning_interface::PlanningSceneInterface scene;

  moveit_msgs::msg::CollisionObject bed;
  bed.header.frame_id = "base_link";
  bed.id = "confirmed_table_surface_guard";
  shape_msgs::msg::SolidPrimitive bed_shape;
  bed_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  bed_shape.dimensions = {
    bed_max_x - bed_min_x,
    bed_max_y - bed_min_y,
    kBelowTableGuardDepthM,
  };
  geometry_msgs::msg::Pose bed_pose;
  bed_pose.orientation.w = 1.0;
  bed_pose.position.x = (bed_min_x + bed_max_x) / 2.0;
  bed_pose.position.y = (bed_min_y + bed_max_y) / 2.0;
  bed_pose.position.z = bed_plane_z - kBelowTableGuardDepthM / 2.0;
  bed.primitives.push_back(bed_shape);
  bed.primitive_poses.push_back(bed_pose);
  bed.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene.applyCollisionObject(bed))
  {
    throw std::runtime_error("MoveIt rejected the fixed tabletop collision guard");
  }

  if (!attach_measured_tool)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Installed fixed tabletop forbidden region %.3f x %.3f x %.3f m with "
      "its top at Z %.4f m. All reachable space below the surface inside the "
      "table footprint is collision space. "
      "Using the UR model's tool0 as the process point; no "
      "print-tool collision object was added.",
      bed_shape.dimensions[0], bed_shape.dimensions[1],
      kBelowTableGuardDepthM, bed_plane_z);
    return;
  }

  if (!ur3e_print_planner::tcpInsidePrintheadEnvelope(tcp_x, tcp_y, tcp_z))
  {
    throw std::runtime_error("measured TCP falls outside the installed printhead CAD envelope");
  }
  moveit_msgs::msg::CollisionObject tool;
  tool.header.frame_id = end_effector_link;
  tool.id = "measured_print_tool";
  // The STEP-derived whole-head envelope covers the configured nozzle TCP.
  // The old full-length tilted TCP cylinder extended beyond the CAD nozzle
  // tip and falsely collided with a printable near-table first layer.
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
  tool.primitives.push_back(head_shape);
  tool.primitive_poses.push_back(head_pose);
  tool.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!scene.applyCollisionObject(tool))
  {
    throw std::runtime_error("MoveIt rejected the measured print-tool collision object");
  }
  if (!move_group.attachObject(
      tool.id, end_effector_link,
      {end_effector_link, "flange", "wrist_3_link"}))
  {
    throw std::runtime_error("MoveIt could not attach the measured print-tool geometry");
  }
  RCLCPP_INFO(
    node->get_logger(),
    "Installed measured collision geometry: table forbidden region "
    "%.3f x %.3f x %.3f m, top at Z %.4f m; "
    "CAD envelope contains TCP (%.4f, %.4f, %.4f) m; calibrated radius %.4f m.",
    bed_shape.dimensions[0], bed_shape.dimensions[1],
    kBelowTableGuardDepthM, bed_plane_z,
    tcp_x, tcp_y, tcp_z, tool_radius);
}

void writeResult(
  const std::string & output_path,
  const std::string & review_sha256,
  const std::string & status,
  const std::string & message,
  const std::string & failure_stage,
  const std::string & approach_status,
  const std::string & approach_strategy,
  const double approach_distance_m,
  const double fraction,
  const bool source_sampled,
  const size_t source_waypoint_count,
  const std::string & planning_group,
  const std::string & end_effector_link,
  const geometry_msgs::msg::Pose * current_pose,
  const geometry_msgs::msg::Pose * path_start_pose,
  const moveit_msgs::msg::RobotTrajectory * trajectory)
{
  const std::filesystem::path destination(output_path);
  if (!destination.parent_path().empty())
  {
    std::filesystem::create_directories(destination.parent_path());
  }
  std::ofstream output(destination);
  if (!output)
  {
    throw std::runtime_error("could not open the MoveIt review result file");
  }
  output << std::fixed << std::setprecision(9);
  const bool passed = status == "passed";
  const auto & joint_trajectory = trajectory ?
    trajectory->joint_trajectory : trajectory_msgs::msg::JointTrajectory{};
  const double max_step = trajectory ? maxJointStep(joint_trajectory.points) : 0.0;
  const double duration = (
    trajectory && !joint_trajectory.points.empty()) ?
    durationSeconds(joint_trajectory.points.back().time_from_start) : 0.0;
  const auto sampled_indexes = trajectory ?
    sampleIndexes(joint_trajectory.points.size(), 900) : std::vector<size_t>{};

  output << "{\n";
  output << "  \"status\": \"" << jsonEscape(status) << "\",\n";
  output << "  \"simulation\": false,\n";
  output << "  \"execution_locked\": true,\n";
  output << "  \"review_sha256\": \"" << jsonEscape(review_sha256) << "\",\n";
  output << "  \"printhead_geometry_sha256\": \"" <<
    ur3e_print_planner::kPrintheadGeometrySha256 << "\",\n";
  output << "  \"planning_policy_version\": \"" << kPlanningPolicyVersion << "\",\n";
  output << "  \"approach_strategy\": \"" << jsonEscape(approach_strategy) << "\",\n";
  output << "  \"interpolation_validation\": \"" <<
    (passed ? "passed" : "not_passed") << "\",\n";
  output << "  \"interpolation_max_joint_spacing_rad\": " <<
    ur3e_print_planner::kCollisionSampleSpacingRad << ",\n";
  output << "  \"planning_group\": \"" << jsonEscape(planning_group) << "\",\n";
  output << "  \"end_effector_link\": \"" << jsonEscape(end_effector_link) << "\",\n";
  output << "  \"reference_frame\": \"base_link\",\n";
  output << "  \"failure_stage\": \"" << jsonEscape(failure_stage) << "\",\n";
  output << "  \"approach_status\": \"" << jsonEscape(approach_status) << "\",\n";
  output << "  \"approach_distance_m\": " << approach_distance_m << ",\n";
  output << "  \"cartesian_fraction\": " << fraction << ",\n";
  output << "  \"collision_free\": " << (passed ? "true" : "false") << ",\n";
  output << "  \"joint_limits\": \"" << (passed ? "passed" : "unconfirmed") << "\",\n";
  output << "  \"max_joint_step_rad\": " << max_step << ",\n";
  output << "  \"duration_s\": " << duration << ",\n";
  output << "  \"source_waypoint_count\": " << source_waypoint_count << ",\n";
  output << "  \"planned_point_count\": " << joint_trajectory.points.size() << ",\n";
  output << "  \"sampled\": " << (source_sampled ? "true" : "false") << ",\n";
  output << "  \"message\": \"" << jsonEscape(message) << "\",\n";
  if (current_pose)
  {
    output << "  \"current_pose\": {"
           << "\"x\":" << current_pose->position.x << ","
           << "\"y\":" << current_pose->position.y << ","
           << "\"z\":" << current_pose->position.z << ","
           << "\"qx\":" << current_pose->orientation.x << ","
           << "\"qy\":" << current_pose->orientation.y << ","
           << "\"qz\":" << current_pose->orientation.z << ","
           << "\"qw\":" << current_pose->orientation.w << "},\n";
  }
  else
  {
    output << "  \"current_pose\": null,\n";
  }
  if (path_start_pose)
  {
    output << "  \"path_start_pose\": {"
           << "\"x\":" << path_start_pose->position.x << ","
           << "\"y\":" << path_start_pose->position.y << ","
           << "\"z\":" << path_start_pose->position.z << ","
           << "\"qx\":" << path_start_pose->orientation.x << ","
           << "\"qy\":" << path_start_pose->orientation.y << ","
           << "\"qz\":" << path_start_pose->orientation.z << ","
           << "\"qw\":" << path_start_pose->orientation.w << "},\n";
  }
  else
  {
    output << "  \"path_start_pose\": null,\n";
  }
  output << "  \"joint_names\": [";
  for (size_t index = 0; index < joint_trajectory.joint_names.size(); ++index)
  {
    if (index > 0)
    {
      output << ",";
    }
    output << "\"" << jsonEscape(joint_trajectory.joint_names[index]) << "\"";
  }
  output << "],\n";
  output << "  \"trajectory\": [\n";
  for (size_t sample_index = 0; sample_index < sampled_indexes.size(); ++sample_index)
  {
    const size_t point_index = sampled_indexes[sample_index];
    const auto & point = joint_trajectory.points[point_index];
    const double progress = joint_trajectory.points.size() <= 1 ?
      1.0 : static_cast<double>(point_index) /
      static_cast<double>(joint_trajectory.points.size() - 1);
    output << "    {\"progress\":" << progress
           << ",\"time_s\":" << durationSeconds(point.time_from_start)
           << ",\"positions\":[";
    for (size_t joint_index = 0; joint_index < point.positions.size(); ++joint_index)
    {
      if (joint_index > 0)
      {
        output << ",";
      }
      output << point.positions[joint_index];
    }
    output << "]}";
    if (sample_index + 1 < sampled_indexes.size())
    {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n";
  output << "}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gcode_path_review");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  std::string output_path;
  std::string approach_trajectory_file;
  std::string print_trajectory_file;
  std::string review_sha256;
  std::string planning_group = "ur_manipulator";
  std::string end_effector_link = "tool0";
  bool source_sampled = false;
  size_t source_waypoint_count = 0;
  geometry_msgs::msg::Pose current_pose;
  bool have_current_pose = false;
  geometry_msgs::msg::Pose path_start_pose;
  bool have_path_start_pose = false;
  std::string failure_stage;
  std::string approach_status = "not_run";
  std::string approach_strategy = "continuous_cartesian_overhead_then_down";
  double approach_distance_m = 0.0;

  try
  {
    node->declare_parameter<std::string>("path_file", "");
    node->declare_parameter<std::string>("output_file", "");
    node->declare_parameter<std::string>("approach_trajectory_file", "");
    node->declare_parameter<std::string>("print_trajectory_file", "");
    node->declare_parameter<std::string>("review_sha256", "");
    const auto printhead_geometry_sha256 =
      node->declare_parameter<std::string>("printhead_geometry_sha256", "");
    node->declare_parameter<std::string>("planning_group", planning_group);
    node->declare_parameter<std::string>("end_effector_link", end_effector_link);
    node->declare_parameter<std::string>("robot_description", "");
    node->declare_parameter<std::string>("robot_description_semantic", "");
    node->declare_parameter<double>("eef_step_m", 0.005);
    node->declare_parameter<double>("velocity_scaling", 0.05);
    node->declare_parameter<double>("acceleration_scaling", 0.05);
    node->declare_parameter<double>("tool_roll_deg", 180.0);
    node->declare_parameter<double>("tool_pitch_deg", 0.0);
    node->declare_parameter<double>("tool_yaw_deg", 0.0);
    node->declare_parameter<double>("tcp_offset_x_m", 0.0);
    node->declare_parameter<double>("tcp_offset_y_m", 0.0);
    node->declare_parameter<double>("tcp_offset_z_m", 0.0);
    node->declare_parameter<double>("tool_collision_radius_m", 0.0);
    node->declare_parameter<double>("bed_min_x_m", 0.0);
    node->declare_parameter<double>("bed_max_x_m", 0.0);
    node->declare_parameter<double>("bed_min_y_m", 0.0);
    node->declare_parameter<double>("bed_max_y_m", 0.0);
    node->declare_parameter<double>("bed_plane_z_m", 0.0);
    node->declare_parameter<std::string>("operation_mode", "production_print");
    node->declare_parameter<bool>("source_sampled", false);
    node->declare_parameter<int64_t>("source_waypoint_count", 0);

    const auto path_file = node->get_parameter("path_file").as_string();
    output_path = node->get_parameter("output_file").as_string();
    approach_trajectory_file =
      node->get_parameter("approach_trajectory_file").as_string();
    print_trajectory_file =
      node->get_parameter("print_trajectory_file").as_string();
    review_sha256 = node->get_parameter("review_sha256").as_string();
    ur3e_print_planner::requirePrintheadGeometryIdentity(printhead_geometry_sha256);
    planning_group = node->get_parameter("planning_group").as_string();
    end_effector_link = node->get_parameter("end_effector_link").as_string();
    const double eef_step_m = node->get_parameter("eef_step_m").as_double();
    const double velocity_scaling = node->get_parameter("velocity_scaling").as_double();
    const double acceleration_scaling =
      node->get_parameter("acceleration_scaling").as_double();
    const double tool_roll_deg = node->get_parameter("tool_roll_deg").as_double();
    const double tool_pitch_deg = node->get_parameter("tool_pitch_deg").as_double();
    const double tool_yaw_deg = node->get_parameter("tool_yaw_deg").as_double();
    const double tcp_offset_x_m = node->get_parameter("tcp_offset_x_m").as_double();
    const double tcp_offset_y_m = node->get_parameter("tcp_offset_y_m").as_double();
    const double tcp_offset_z_m = node->get_parameter("tcp_offset_z_m").as_double();
    const double tool_collision_radius_m =
      node->get_parameter("tool_collision_radius_m").as_double();
    const double bed_min_x_m = node->get_parameter("bed_min_x_m").as_double();
    const double bed_max_x_m = node->get_parameter("bed_max_x_m").as_double();
    const double bed_min_y_m = node->get_parameter("bed_min_y_m").as_double();
    const double bed_max_y_m = node->get_parameter("bed_max_y_m").as_double();
    const double bed_plane_z_m = node->get_parameter("bed_plane_z_m").as_double();
    const auto operation_mode = node->get_parameter("operation_mode").as_string();
    source_sampled = node->get_parameter("source_sampled").as_bool();
    const auto source_count_parameter =
      node->get_parameter("source_waypoint_count").as_int();
    source_waypoint_count = source_count_parameter > 0 ?
      static_cast<size_t>(source_count_parameter) : 0;

    if (
      path_file.empty() || output_path.empty() || approach_trajectory_file.empty() ||
      print_trajectory_file.empty() || review_sha256.size() != 64)
    {
      throw std::runtime_error(
        "path_file, output_file, both trajectory files, and a 64-character "
        "review_sha256 are required");
    }
    // A failed re-review must never leave an older trajectory available under
    // the same review id.
    std::filesystem::remove(approach_trajectory_file);
    std::filesystem::remove(print_trajectory_file);
    if (!std::isfinite(eef_step_m) || eef_step_m < 0.001 || eef_step_m > 0.02)
    {
      throw std::runtime_error("eef_step_m must be between 0.001 and 0.02 m");
    }
    if (
      !std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 ||
      velocity_scaling > 0.2 ||
      !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 ||
      acceleration_scaling > 0.2)
    {
      throw std::runtime_error(
        "velocity and acceleration scaling must be greater than zero and no more than 0.2");
    }
    for (const double angle : {tool_roll_deg, tool_pitch_deg, tool_yaw_deg})
    {
      if (!std::isfinite(angle) || angle < -180.0 || angle > 180.0)
      {
        throw std::runtime_error(
          "calibrated tool roll, pitch, and yaw must be between -180 and 180 degrees");
      }
    }
    const double tcp_length_m = std::sqrt(
      tcp_offset_x_m * tcp_offset_x_m + tcp_offset_y_m * tcp_offset_y_m +
      tcp_offset_z_m * tcp_offset_z_m);
    const bool tool0_motion_test = operation_mode == "tool0_motion_test";
    const bool tool0_reference_mode = tool0_motion_test;
    if (operation_mode != "production_print" && !tool0_reference_mode)
    {
      throw std::runtime_error(
        "operation_mode must be production_print or tool0_motion_test");
    }
    if (!tool0_reference_mode && (
      !std::isfinite(tcp_length_m) || tcp_length_m < 0.005 || tcp_length_m > 1.0 ||
      !std::isfinite(tool_collision_radius_m) || tool_collision_radius_m < 0.001 ||
      tool_collision_radius_m > 0.25))
    {
      throw std::runtime_error(
        "the measured TCP length and collision radius are missing or outside safe limits");
    }
    if (!tool0_reference_mode && !ur3e_print_planner::tcpInsidePrintheadEnvelope(
        tcp_offset_x_m, tcp_offset_y_m, tcp_offset_z_m))
    {
      throw std::runtime_error(
        "measured nozzle TCP lies outside the installed printhead CAD envelope; "
        "recalibrate the tool collision model before planning");
    }
    if (
      !std::isfinite(bed_min_x_m) || !std::isfinite(bed_max_x_m) ||
      !std::isfinite(bed_min_y_m) || !std::isfinite(bed_max_y_m) ||
      !std::isfinite(bed_plane_z_m) || bed_min_x_m >= bed_max_x_m ||
      bed_min_y_m >= bed_max_y_m)
    {
      throw std::runtime_error("the fixed tabletop collision geometry is invalid");
    }

    const auto path_points = readPath(path_file);
    if (source_waypoint_count == 0)
    {
      source_waypoint_count = path_points.size();
    }
    loadMatchingRobotModel(node, planning_group);

    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink(end_effector_link);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group.setPlanningTime(15.0);
    installFixedWorkcellCollisionGeometry(
      node, move_group, end_effector_link,
      bed_min_x_m, bed_max_x_m, bed_min_y_m, bed_max_y_m, bed_plane_z_m,
      tcp_offset_x_m, tcp_offset_y_m, tcp_offset_z_m, tool_collision_radius_m,
      !tool0_reference_mode);

    auto current_state = move_group.getCurrentState(10.0);
    if (!current_state)
    {
      throw std::runtime_error("failed to receive the current UR joint state");
    }
    move_group.setStartState(*current_state);
    current_pose = move_group.getCurrentPose(end_effector_link).pose;
    have_current_pose = true;
    tf2::Quaternion tool_orientation;
    constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
    tool_orientation.setRPY(
      tool_roll_deg * degrees_to_radians,
      tool_pitch_deg * degrees_to_radians,
      tool_yaw_deg * degrees_to_radians);
    tool_orientation.normalize();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(path_points.size());
    for (const auto & point : path_points)
    {
      geometry_msgs::msg::Pose pose = current_pose;
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = point.z;
      pose.orientation.x = tool_orientation.x();
      pose.orientation.y = tool_orientation.y();
      pose.orientation.z = tool_orientation.z();
      pose.orientation.w = tool_orientation.w();
      waypoints.push_back(pose);
    }
    path_start_pose = waypoints.front();
    have_path_start_pose = true;
    approach_distance_m = std::sqrt(
      std::pow(path_start_pose.position.x - current_pose.position.x, 2.0) +
      std::pow(path_start_pose.position.y - current_pose.position.y, 2.0) +
      std::pow(path_start_pose.position.z - current_pose.position.z, 2.0));

    RCLCPP_INFO(
      node->get_logger(),
      "Plan-only review: %zu calibrated waypoints, calibrated tool orientation "
      "(roll %.3f, pitch %.3f, yaw %.3f deg). "
      "No controller action and no Klipper command will be sent.",
      waypoints.size(), tool_roll_deg, tool_pitch_deg, tool_yaw_deg);

    RCLCPP_INFO(
      node->get_logger(),
      "Planning a collision-aware overhead approach to the calibrated G-code "
      "origin at x=%.4f y=%.4f z=%.4f (straight-line distance %.4f m).",
      path_start_pose.position.x,
      path_start_pose.position.y,
      path_start_pose.position.z,
      approach_distance_m);

    failure_stage = "continuous_approach";
    approach_status = "failed";
    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    const ExpectedWorkcellGeometry expected_workcell{
      bed_min_x_m, bed_max_x_m, bed_min_y_m, bed_max_y_m, bed_plane_z_m,
      Eigen::Vector3d(tcp_offset_x_m, tcp_offset_y_m, tcp_offset_z_m),
      tool_collision_radius_m, end_effector_link, !tool0_reference_mode};
    auto validation_scene = loadValidationScene(
      node, move_group.getRobotModel(), expected_workcell);
    move_group.setStartState(*current_state);

    // First solve the exact print-start pose directly.  When the live TCP and
    // the resulting IK endpoint are already within tight print-start
    // tolerances, an overhead lift would only create a large joint-space loop
    // that returns to the same state.  The direct candidate is still generated
    // with the complete collision scene and is fully revalidated below.
    moveit_msgs::msg::RobotTrajectory direct_start_trajectory;
    const double direct_start_fraction = computeContinuousCartesianPath(
      validation_scene, *current_state, planning_group, end_effector_link,
      {path_start_pose}, std::min(eef_step_m, 0.0025), direct_start_trajectory);
    if (direct_start_trajectory.joint_trajectory.points.size() == 1)
    {
      direct_start_trajectory.joint_trajectory.points.push_back(
        direct_start_trajectory.joint_trajectory.points.front());
    }
    const ur3e_print_planner::Quaternion current_orientation{
      current_pose.orientation.x, current_pose.orientation.y,
      current_pose.orientation.z, current_pose.orientation.w};
    const ur3e_print_planner::Quaternion print_orientation{
      path_start_pose.orientation.x, path_start_pose.orientation.y,
      path_start_pose.orientation.z, path_start_pose.orientation.w};
    const bool near_zero_approach =
      std::isfinite(direct_start_fraction) &&
      direct_start_fraction >= kCompletePathFraction &&
      direct_start_trajectory.joint_trajectory.points.size() >= 2 &&
      ur3e_print_planner::canUseNearZeroApproach(
      approach_distance_m, current_orientation, print_orientation,
      jointPath(direct_start_trajectory.joint_trajectory));

    double approach_fraction = direct_start_fraction;
    std::string approach_failure_detail;
    if (near_zero_approach)
    {
      approach_strategy = "near_zero_direct_to_print_start";
      approach_trajectory = std::move(direct_start_trajectory);
      RCLCPP_INFO(
        node->get_logger(),
        "The live TCP is already at the reviewed print start; using the "
        "collision-checked direct correction instead of an overhead round trip.");
    }
    else
    {
      geometry_msgs::msg::Pose approach_staging_pose = path_start_pose;
      approach_staging_pose.position.z = std::max(
        path_start_pose.position.z + kApproachClearanceM,
        bed_plane_z_m + kApproachClearanceM);
      // One seeded Cartesian solve preserves the nearby IK posture across BOTH
      // approach and descent. Do not independently choose an arbitrary staging
      // IK solution, accept the first OMPL route, or silently fall back to it.
      approach_fraction = computeContinuousCartesianPath(
        validation_scene, *current_state, planning_group, end_effector_link,
        {approach_staging_pose, path_start_pose}, std::min(eef_step_m, 0.0025),
        approach_trajectory);
      if (
        !std::isfinite(approach_fraction) || approach_fraction < kCompletePathFraction ||
        approach_trajectory.joint_trajectory.points.size() < 2)
      {
        RCLCPP_WARN(
          node->get_logger(),
          "The direct and overhead Cartesian approaches did not complete; "
          "trying a bounded lift/rotate/traverse/descent from the same live joint state.");
        try
        {
          const double safe_height = ur3e_print_planner::decomposedApproachHeight(
            current_pose.position.z, approach_staging_pose.position.z, bed_plane_z_m);
          geometry_msgs::msg::Pose lift_pose = current_pose;
          lift_pose.position.z = safe_height;
          geometry_msgs::msg::Pose rotate_pose = lift_pose;
          rotate_pose.orientation = path_start_pose.orientation;
          geometry_msgs::msg::Pose traverse_pose = path_start_pose;
          traverse_pose.position.z = safe_height;
          moveit_msgs::msg::RobotTrajectory decomposed_trajectory;
          const double decomposed_fraction = computeContinuousCartesianPath(
            validation_scene, *current_state, planning_group, end_effector_link,
            {lift_pose, rotate_pose, traverse_pose, path_start_pose},
            std::min(eef_step_m, 0.0025), decomposed_trajectory);
          RCLCPP_INFO(
            node->get_logger(),
            "Bounded decomposed approach: safe tool0 Z %.6f m, Cartesian fraction %.9f.",
            safe_height, decomposed_fraction);
          approach_strategy = "continuous_cartesian_lift_rotate_traverse_descend";
          approach_fraction = std::isfinite(decomposed_fraction) ?
            decomposed_fraction : 0.0;
          if (
            std::isfinite(decomposed_fraction) &&
            decomposed_fraction >= kCompletePathFraction &&
            decomposed_trajectory.joint_trajectory.points.size() >= 2)
          {
            approach_trajectory = std::move(decomposed_trajectory);
          }
          else
          {
            approach_failure_detail =
              " The bounded lift/rotate/traverse/descent also failed from the "
              "recorded posture.";
          }
        }
        catch (const std::runtime_error & error)
        {
          approach_failure_detail = std::string(" The decomposed alternative was rejected: ") +
            error.what() + ".";
          RCLCPP_WARN(node->get_logger(), "%s", approach_failure_detail.c_str());
        }
      }
    }
    RCLCPP_INFO(node->get_logger(), "Continuous approach fraction: %.9f", approach_fraction);
    if (
      !std::isfinite(approach_fraction) || approach_fraction < kCompletePathFraction ||
      approach_trajectory.joint_trajectory.points.size() < 2)
    {
      throw std::runtime_error(
        "The continuous overhead approach cannot be completed from the recorded "
        "robot posture. No automatic joint-space detour or move-to-start was saved. "
        "Review a different starting posture before planning again." +
        approach_failure_detail);
    }
    if (hasJointMotion(approach_trajectory.joint_trajectory))
    {
      timeParameterizeWithoutChangingGeometry(move_group.getRobotModel(), planning_group,
        *current_state, velocity_scaling, acceleration_scaling, approach_trajectory);
    }
    else
    {
      makeStationaryApproachSerializable(approach_trajectory.joint_trajectory);
    }
    preparePositionOnlyTrajectory(approach_trajectory.joint_trajectory);
    const auto approach_metrics = validateTrajectory(
      node, validation_scene, approach_trajectory.joint_trajectory,
      *current_state, planning_group, true);
    for (size_t j = 0; j < approach_metrics.travel.size(); ++j)
    {
      RCLCPP_INFO(node->get_logger(), "Approach %s total travel %.6f rad, backtrack %.6f rad.",
        approach_trajectory.joint_trajectory.joint_names[j].c_str(),
        approach_metrics.travel[j], approach_metrics.backtrack[j]);
    }
    approach_status = "passed";

    moveit::core::RobotState path_start_state(*current_state);
    const auto & approach_joint_trajectory =
      approach_trajectory.joint_trajectory;
    path_start_state.setVariablePositions(
      approach_joint_trajectory.joint_names,
      approach_joint_trajectory.points.back().positions);
    path_start_state.update();
    move_group.setStartState(path_start_state);

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit::core::RobotState chunk_start_state(path_start_state);
    failure_stage = "print_path";
    size_t completed_waypoints = 0;
    size_t chunk_count = 0;
    bool chunk_failed = false;
    for (size_t start_index = 0; start_index < waypoints.size();
      start_index += kCartesianChunkPoints)
    {
      const size_t end_index = std::min(
        waypoints.size(), start_index + kCartesianChunkPoints);
      std::vector<geometry_msgs::msg::Pose> chunk_waypoints(
        waypoints.begin() + static_cast<std::ptrdiff_t>(start_index),
        waypoints.begin() + static_cast<std::ptrdiff_t>(end_index));
      move_group.setStartState(chunk_start_state);
      moveit_msgs::msg::RobotTrajectory chunk_trajectory;
      const double chunk_fraction = computeContinuousCartesianPath(
        validation_scene, chunk_start_state, planning_group, end_effector_link,
        chunk_waypoints, std::min(eef_step_m, 0.0025), chunk_trajectory);
      ++chunk_count;
      RCLCPP_INFO(
        node->get_logger(),
        "MoveIt collision-aware Cartesian chunk %zu (%zu-%zu of %zu): %.6f",
        chunk_count,
        start_index + 1,
        end_index,
        waypoints.size(),
        chunk_fraction);

      const auto & chunk_joint_trajectory = chunk_trajectory.joint_trajectory;
      if (trajectory.joint_trajectory.joint_names.empty())
      {
        trajectory.joint_trajectory.joint_names =
          chunk_joint_trajectory.joint_names;
      }
      else if (
        !chunk_joint_trajectory.joint_names.empty() &&
        trajectory.joint_trajectory.joint_names !=
        chunk_joint_trajectory.joint_names)
      {
        throw std::runtime_error(
          "MoveIt returned inconsistent joint names between path chunks");
      }
      for (const auto & point : chunk_joint_trajectory.points)
      {
        if (
          !trajectory.joint_trajectory.points.empty() &&
          trajectory.joint_trajectory.points.back().positions == point.positions)
        {
          continue;
        }
        auto appended = point;
        appended.time_from_start = builtin_interfaces::msg::Duration{};
        trajectory.joint_trajectory.points.push_back(std::move(appended));
      }

      const size_t chunk_size = end_index - start_index;
      if (
        !std::isfinite(chunk_fraction) || chunk_fraction < kCompletePathFraction ||
        chunk_joint_trajectory.points.empty())
      {
        chunk_failed = true;
        break;
      }
      completed_waypoints += chunk_size;
      chunk_start_state.setVariablePositions(
        chunk_joint_trajectory.joint_names,
        chunk_joint_trajectory.points.back().positions);
      chunk_start_state.update();
    }
    const double fraction = waypoints.empty() ? 0.0 :
      static_cast<double>(completed_waypoints) /
      static_cast<double>(waypoints.size());
    RCLCPP_INFO(
      node->get_logger(),
      "MoveIt full-path collision-aware Cartesian fraction across %zu chunks: %.6f",
      chunk_count,
      fraction);

    std::string status = "failed";
    failure_stage = "print_path";
    std::string message =
      "MoveIt reached the automatically placed path start, but could not validate the "
      "complete collision-aware print path.";
    if (!trajectory.joint_trajectory.points.empty())
    {
      timeParameterizeWithoutChangingGeometry(move_group.getRobotModel(), planning_group,
        path_start_state, velocity_scaling, acceleration_scaling, trajectory);
      preparePositionOnlyTrajectory(trajectory.joint_trajectory);
    }
    if (
      !chunk_failed && fraction >= kCompletePathFraction &&
      !trajectory.joint_trajectory.points.empty())
    {
      // Re-check both joined paths against a fresh complete scene after all
      // planning and timing. The print starts at the NEW approach's exact raw
      // joint state, never an old cached IK branch.
      failure_stage = "final_interpolation_validation";
      validation_scene = loadValidationScene(
        node, move_group.getRobotModel(), expected_workcell);
      validateTrajectory(node, validation_scene, approach_trajectory.joint_trajectory,
        *current_state, planning_group, true);
      validateTrajectory(node, validation_scene, trajectory.joint_trajectory,
        path_start_state, planning_group, false);
      status = "passed";
      failure_stage.clear();
      message =
        "MoveIt completed the reviewed approach and the complete print path "
        "from the same joint branch. Both serialized position-only paths passed raw "
        "joint continuity and collision/bounds sampling at <= 1 degree per joint. "
        "This is finite-sample model validation, not physical execution approval.";
    }
    if (status == "passed")
    {
      writeTrajectoryCsv(
        approach_trajectory_file,
        approach_trajectory.joint_trajectory,
        cartesianPathProgress(
          *current_state, end_effector_link, approach_trajectory.joint_trajectory));
      writeTrajectoryCsv(
        print_trajectory_file,
        trajectory.joint_trajectory,
        cartesianPathProgress(
          path_start_state,
          end_effector_link,
          trajectory.joint_trajectory));
    }

    // RViz receives the validated position-only geometry: the
    // collision-aware approach (a near-zero direct correction when already at
    // the start, otherwise overhead or bounded decomposed Cartesian), followed
    // by the full print path. This is display-only and never contacts a
    // trajectory controller.
    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local());
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    // An empty display identifier allows differing robot names without
    // pretending that a name override fixes a kinematic calibration mismatch.
    display_trajectory.model_id = "";
    moveit::core::robotStateToRobotStateMsg(
      *current_state, display_trajectory.trajectory_start);
    display_trajectory.trajectory.push_back(approach_trajectory);
    if (status == "passed" && !trajectory.joint_trajectory.points.empty())
    {
      display_trajectory.trajectory.push_back(trajectory);
    }
    display_pub->publish(display_trajectory);
    rclcpp::sleep_for(std::chrono::seconds(2));

    writeResult(
      output_path,
      review_sha256,
      status,
      message,
      failure_stage,
      approach_status,
      approach_strategy,
      approach_distance_m,
      fraction,
      source_sampled,
      source_waypoint_count,
      planning_group,
      end_effector_link,
      &current_pose,
      &path_start_pose,
      &trajectory);

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(node->get_logger(), "Plan-only path review failed: %s", error.what());
    // No partial pair of execution artifacts survives a failed serialization
    // or final validation. The backend also invalidates packages at replan start.
    for (const auto & path : {approach_trajectory_file, print_trajectory_file})
    {
      if (!path.empty())
      {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
      }
    }
    if (!output_path.empty() && review_sha256.size() == 64)
    {
      try
      {
        writeResult(
          output_path,
          review_sha256,
          "failed",
          error.what(),
          failure_stage.empty() ? "setup" : failure_stage,
          approach_status,
          approach_strategy,
          approach_distance_m,
          0.0,
          source_sampled,
          source_waypoint_count,
          planning_group,
          end_effector_link,
          have_current_pose ? &current_pose : nullptr,
          have_path_start_pose ? &path_start_pose : nullptr,
          nullptr);
      }
      catch (const std::exception & write_error)
      {
        RCLCPP_ERROR(
          node->get_logger(), "Could not write failure result: %s", write_error.what());
      }
    }
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 2;
  }
}
