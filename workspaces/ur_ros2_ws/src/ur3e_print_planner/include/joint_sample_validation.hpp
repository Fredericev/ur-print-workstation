#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur3e_print_planner
{

inline constexpr std::array<std::string_view, 6> kUrJointNames = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
};
inline constexpr std::size_t kMaximumJointSampleRows = 200000;

using JointSample = std::array<double, 6>;
struct JointSamplePair
{
  JointSample candidate{};
  JointSample reference{};
};
using JointSampleSequence = std::vector<JointSamplePair>;
using JointSampleValidator = std::function<bool(const JointSample &)>;

struct WorkcellGeometryExpectation
{
  double bed_min_x_m{};
  double bed_max_x_m{};
  double bed_min_y_m{};
  double bed_max_y_m{};
  double bed_plane_z_m{};
  double tool_collision_radius_m{};
  std::array<double, 3> tcp_offset_m{};
};

JointSampleSequence parseJointSampleCsv(const std::filesystem::path & path);

std::size_t validateJointSamples(
  const JointSampleSequence & samples,
  const JointSampleValidator & validator);

void requireMatchingWorkcellGeometry(
  const moveit_msgs::msg::PlanningScene & scene,
  const WorkcellGeometryExpectation & expected,
  const std::string & model_frame,
  bool model_frame_to_base_is_identity);

}  // namespace ur3e_print_planner
