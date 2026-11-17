#include "joint_sample_validation.hpp"
#include "printhead_collision_geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

using namespace std::chrono_literals;
constexpr char kTableGuard[] = "confirmed_table_surface_guard";
constexpr double kMaximumTool0TranslationErrorM = 0.0005;
constexpr double kMaximumTool0RotationErrorRad = 0.005;
constexpr std::int64_t kMaximumSampleCsvFiles = 128;

void installMoveGroupRobotDescription(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::executors::SingleThreadedExecutor & executor)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(5s))
  {
    throw std::runtime_error("MoveIt parameter service is unavailable");
  }
  const std::vector<std::string> names = {
    "robot_description",
    "robot_description_semantic",
  };
  auto future = client->get_parameters(names);
  if (executor.spin_until_future_complete(future, 10s) != rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("timed out loading robot_description from move_group");
  }
  const auto parameters = future.get();
  if (parameters.size() != names.size())
  {
    throw std::runtime_error("move_group returned an incomplete robot description");
  }
  for (std::size_t index = 0; index < parameters.size(); ++index)
  {
    if (parameters[index].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[index].as_string().empty())
    {
      throw std::runtime_error("move_group returned an invalid robot description");
    }
    node->declare_parameter(
      names[index], parameters[index].get_parameter_value(),
      rcl_interfaces::msg::ParameterDescriptor{}, true);
  }
}

void requireSafeTableCollisionPolicy(const moveit_msgs::msg::AllowedCollisionMatrix & matrix)
{
  if (matrix.entry_names.size() != matrix.entry_values.size() ||
    matrix.default_entry_names.size() != matrix.default_entry_values.size())
  {
    throw std::runtime_error("MoveIt returned a malformed allowed-collision matrix");
  }
  for (const auto & row : matrix.entry_values)
  {
    if (row.enabled.size() != matrix.entry_names.size())
    {
      throw std::runtime_error("MoveIt returned a malformed allowed-collision matrix row");
    }
  }
  if (std::any_of(
      matrix.default_entry_values.begin(), matrix.default_entry_values.end(),
      [](const bool enabled) {return enabled;}))
  {
    throw std::runtime_error("MoveIt has default collision exemptions");
  }

  const auto guard = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), kTableGuard);
  if (guard == matrix.entry_names.end())
  {
    return;
  }
  const auto guard_index = static_cast<std::size_t>(guard - matrix.entry_names.begin());
  if (std::any_of(
      matrix.entry_values[guard_index].enabled.begin(),
      matrix.entry_values[guard_index].enabled.end(),
      [](const bool enabled) {return enabled;}))
  {
    throw std::runtime_error("MoveIt allows collisions with the confirmed table guard");
  }
  for (const auto & row : matrix.entry_values)
  {
    if (row.enabled[guard_index])
    {
      throw std::runtime_error("MoveIt allows collisions with the confirmed table guard");
    }
  }
}

void requireCompleteCurrentState(
  const moveit_msgs::msg::PlanningScene & message,
  const moveit::core::RobotState & state)
{
  if (message.is_diff || message.robot_state.is_diff)
  {
    throw std::runtime_error("MoveIt returned a differential rather than complete planning scene");
  }
  const auto & joints = message.robot_state.joint_state;
  if (joints.name.size() != joints.position.size())
  {
    throw std::runtime_error("MoveIt returned a malformed current joint state");
  }
  std::set<std::string> supplied;
  for (std::size_t index = 0; index < joints.name.size(); ++index)
  {
    if (joints.name[index].empty() || !std::isfinite(joints.position[index]) ||
      !supplied.insert(joints.name[index]).second)
    {
      throw std::runtime_error("MoveIt returned an invalid current joint state");
    }
  }
  for (const auto expected : ur3e_print_planner::kUrJointNames)
  {
    if (supplied.count(std::string(expected)) != 1)
    {
      throw std::runtime_error("MoveIt current state is missing a UR joint");
    }
  }

  std::set<std::string> expected_attachments;
  for (const auto & attached : message.robot_state.attached_collision_objects)
  {
    if (attached.object.id.empty() || !expected_attachments.insert(attached.object.id).second)
    {
      throw std::runtime_error("MoveIt returned an invalid attached collision body");
    }
  }
  std::vector<const moveit::core::AttachedBody *> loaded_attachments;
  state.getAttachedBodies(loaded_attachments);
  std::set<std::string> loaded_names;
  for (const auto * attached : loaded_attachments)
  {
    if (attached != nullptr)
    {
      loaded_names.insert(attached->getName());
    }
  }
  if (loaded_names != expected_attachments)
  {
    throw std::runtime_error("MoveIt attached bodies were not preserved in the local scene");
  }
}

planning_scene::PlanningScenePtr loadCompletePlanningScene(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::executors::SingleThreadedExecutor & executor,
  const moveit::core::RobotModelConstPtr & model,
  const ur3e_print_planner::WorkcellGeometryExpectation & expected)
{
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(10s))
  {
    throw std::runtime_error("MoveIt planning-scene service is unavailable");
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023U;  // Every PlanningSceneComponents field in ROS 2 Humble.
  auto future = client->async_send_request(request);
  if (executor.spin_until_future_complete(future, 30s) != rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("timed out obtaining the complete MoveIt planning scene");
  }
  const auto response = future.get();
  requireSafeTableCollisionPolicy(response->scene.allowed_collision_matrix);

  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  if (!scene->setPlanningSceneMsg(response->scene))
  {
    throw std::runtime_error("MoveIt planning scene could not be reconstructed locally");
  }
  if (!scene->getWorld()->hasObject(kTableGuard))
  {
    throw std::runtime_error("MoveIt collision scene is missing confirmed_table_surface_guard");
  }
  requireCompleteCurrentState(response->scene, scene->getCurrentState());
  const auto & model_to_base =
    scene->getCurrentState().getGlobalLinkTransform("base_link");
  const bool identity_model_to_base = model_to_base.matrix().isApprox(
    Eigen::Matrix4d::Identity(), 1.0e-9);
  ur3e_print_planner::requireMatchingWorkcellGeometry(
    response->scene, expected, model->getModelFrame(), identity_model_to_base);
  return scene;
}

std::vector<std::string> expectedJointNames()
{
  std::vector<std::string> names;
  names.reserve(ur3e_print_planner::kUrJointNames.size());
  for (const auto name : ur3e_print_planner::kUrJointNames)
  {
    names.emplace_back(name);
  }
  return names;
}

std::vector<std::string> sampleCsvPaths(const rclcpp::Node::SharedPtr & node)
{
  const auto count = node->declare_parameter<std::int64_t>("sample_csv_count", 0);
  if (count < 1 || count > kMaximumSampleCsvFiles)
  {
    throw std::runtime_error("sample_csv_count is outside the guarded batch limit");
  }
  std::vector<std::string> paths;
  paths.reserve(static_cast<std::size_t>(count));
  std::set<std::string> unique;
  for (std::int64_t index = 0; index < count; ++index)
  {
    const auto name = "sample_csv_" + std::to_string(index);
    const auto path = node->declare_parameter<std::string>(name, "");
    if (path.empty() || !unique.insert(path).second)
    {
      throw std::runtime_error("batched sample_csv paths must be non-empty and unique");
    }
    paths.push_back(path);
  }
  return paths;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try
  {
    auto node = std::make_shared<rclcpp::Node>("validate_joint_samples");
    ur3e_print_planner::requirePrintheadGeometryIdentity(
      node->declare_parameter<std::string>("printhead_geometry_sha256", ""));
    const auto sample_csv_paths = sampleCsvPaths(node);
    const double missing = std::numeric_limits<double>::quiet_NaN();
    const ur3e_print_planner::WorkcellGeometryExpectation expected{
      node->declare_parameter<double>("bed_min_x_m", missing),
      node->declare_parameter<double>("bed_max_x_m", missing),
      node->declare_parameter<double>("bed_min_y_m", missing),
      node->declare_parameter<double>("bed_max_y_m", missing),
      node->declare_parameter<double>("bed_plane_z_m", missing),
      node->declare_parameter<double>("tool_collision_radius_m", missing),
      {
        node->declare_parameter<double>("tcp_offset_x_m", missing),
        node->declare_parameter<double>("tcp_offset_y_m", missing),
        node->declare_parameter<double>("tcp_offset_z_m", missing),
      },
    };
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    installMoveGroupRobotDescription(node, executor);
    robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
    const auto model = loader.getModel();
    if (!model)
    {
      throw std::runtime_error("MoveIt robot model could not be loaded");
    }
    const auto * group = model->getJointModelGroup("ur_manipulator");
    if (group == nullptr || group->getVariableNames() != expectedJointNames())
    {
      throw std::runtime_error("MoveIt ur_manipulator does not have the exact six expected UR joints");
    }

    const auto scene = loadCompletePlanningScene(node, executor, model, expected);
    moveit::core::RobotState candidate_state(scene->getCurrentState());
    moveit::core::RobotState reference_state(scene->getCurrentState());
    double maximum_translation_m = 0.0;
    double maximum_rotation_rad = 0.0;
    double sum_translation_squared_m = 0.0;
    double sum_rotation_squared_rad = 0.0;
    std::size_t checked = 0;
    for (const auto & sample_csv : sample_csv_paths)
    {
      // Each input retains the 200,000-row / 64 MiB parser guards.  All
      // batches are checked here, in this one process, against the one robot
      // model and complete PlanningScene snapshot loaded above.
      const auto samples = ur3e_print_planner::parseJointSampleCsv(sample_csv);
      for (std::size_t index = 0; index < samples.size(); ++index)
      {
        const auto & pair = samples[index];
        const auto global_index = checked + 1;
        candidate_state.setJointGroupPositions(group, pair.candidate.data());
        candidate_state.update(true);
        if (!candidate_state.satisfiesBounds(group) ||
          !scene->isStateValid(candidate_state, "", false))
        {
          throw std::runtime_error(
                  "candidate joint state " + std::to_string(global_index) +
                  " is out of bounds or in collision");
        }
        reference_state.setJointGroupPositions(group, pair.reference.data());
        reference_state.update(true);
        if (!reference_state.satisfiesBounds(group))
        {
          throw std::runtime_error(
                  "reference joint state " + std::to_string(global_index) +
                  " is outside the robot model bounds");
        }

        const auto & candidate_tool = candidate_state.getGlobalLinkTransform("tool0");
        const auto & reference_tool = reference_state.getGlobalLinkTransform("tool0");
        const double translation_m =
          (candidate_tool.translation() - reference_tool.translation()).norm();
        const Eigen::Quaterniond candidate_orientation(candidate_tool.rotation());
        const Eigen::Quaterniond reference_orientation(reference_tool.rotation());
        const double rotation_rad =
          candidate_orientation.angularDistance(reference_orientation);
        if (!std::isfinite(translation_m) || !std::isfinite(rotation_rad))
        {
          throw std::runtime_error("tool0 fidelity calculation produced a non-finite error");
        }
        maximum_translation_m = std::max(maximum_translation_m, translation_m);
        maximum_rotation_rad = std::max(maximum_rotation_rad, rotation_rad);
        sum_translation_squared_m += translation_m * translation_m;
        sum_rotation_squared_rad += rotation_rad * rotation_rad;
        ++checked;
      }
    }
    const double rms_translation_m = std::sqrt(
      sum_translation_squared_m / static_cast<double>(checked));
    const double rms_rotation_rad = std::sqrt(
      sum_rotation_squared_rad / static_cast<double>(checked));
    std::cout << std::setprecision(17)
              << "VALIDATION_PROTOCOL=3\n"
              << "PRINTHEAD_GEOMETRY_SHA256="
              << ur3e_print_planner::kPrintheadGeometrySha256 << '\n'
              << "VALIDATED_BATCHES=" << sample_csv_paths.size() << '\n'
              << "VALIDATED_SAMPLES=" << checked << '\n'
              << "FIDELITY_SAMPLES=" << checked << '\n'
              << "MAX_TOOL0_TRANSLATION_M=" << maximum_translation_m << '\n'
              << "RMS_TOOL0_TRANSLATION_M=" << rms_translation_m << '\n'
              << "MAX_TOOL0_ROTATION_RAD=" << maximum_rotation_rad << '\n'
              << "RMS_TOOL0_ROTATION_RAD=" << rms_rotation_rad << '\n';
    if (maximum_translation_m > kMaximumTool0TranslationErrorM ||
      maximum_rotation_rad > kMaximumTool0RotationErrorRad)
    {
      throw std::runtime_error(
              "tool0 TCP fidelity exceeded the fixed 0.0005 m / 0.005 rad hard limits");
    }
    result = 0;
  }
  catch (const std::exception & error)
  {
    std::cerr << "VALIDATION_ERROR=" << error.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
