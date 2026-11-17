#include "joint_sample_validation.hpp"
#include "../src/printhead_collision_geometry.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <unistd.h>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{

int failures = 0;

std::filesystem::path writeCsv(const std::string & name, const std::string & contents)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("ur_joint_sample_validation_" + std::to_string(::getpid()));
  std::filesystem::create_directories(directory);
  const auto path = directory / (name + ".csv");
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream << contents;
  stream.close();
  if (!stream)
  {
    throw std::runtime_error("failed to create CSV fixture");
  }
  return path;
}

template<typename Callable>
void expectRejected(const std::string & name, Callable callable)
{
  try
  {
    callable();
    std::cerr << "FAIL " << name << ": invalid input was accepted\n";
    ++failures;
  }
  catch (const std::runtime_error &)
  {
    std::cout << "PASS " << name << '\n';
  }
}

}  // namespace

int main()
{
  using ur3e_print_planner::parseJointSampleCsv;
  using ur3e_print_planner::requireMatchingWorkcellGeometry;
  using ur3e_print_planner::validateJointSamples;
  using ur3e_print_planner::WorkcellGeometryExpectation;

  const std::string header =
    "candidate_shoulder_pan_joint,candidate_shoulder_lift_joint,candidate_elbow_joint,"
    "candidate_wrist_1_joint,candidate_wrist_2_joint,candidate_wrist_3_joint,"
    "reference_shoulder_pan_joint,reference_shoulder_lift_joint,reference_elbow_joint,"
    "reference_wrist_1_joint,reference_wrist_2_joint,reference_wrist_3_joint\n";
  const auto valid_path = writeCsv(
    "valid",
    header +
    "0,-1.25,2e-3,3,4,5,0.1,-1.2,3e-3,3.1,4.1,5.1\n"
    "1,2,3,4,5,6,1.1,2.1,3.1,4.1,5.1,6.1\n");
  try
  {
    const auto samples = parseJointSampleCsv(valid_path);
    if (samples.size() != 2 || std::abs(samples[0].candidate[1] + 1.25) > 1e-12 ||
      std::abs(samples[0].candidate[2] - 0.002) > 1e-12 ||
      std::abs(samples[0].reference[0] - 0.1) > 1e-12 ||
      std::abs(samples[1].reference[5] - 6.1) > 1e-12)
    {
      throw std::runtime_error("valid samples changed during parsing");
    }
    size_t callbacks = 0;
    const auto checked = validateJointSamples(samples, [&callbacks](const auto &) {
      ++callbacks;
      return true;
    });
    if (checked != 2 || callbacks != 2)
    {
      throw std::runtime_error("not every sample reached the validation callback");
    }
    std::cout << "PASS exact CSV and complete callback coverage\n";
  }
  catch (const std::runtime_error & error)
  {
    std::cerr << "FAIL valid CSV: " << error.what() << '\n';
    ++failures;
  }

  const WorkcellGeometryExpectation workcell{
    -0.5175, -0.0675, -0.27, 0.27, -0.014, 0.0, {0.0, 0.0, 0.0}};
  moveit_msgs::msg::PlanningScene scene;
  moveit_msgs::msg::CollisionObject table;
  table.id = "confirmed_table_surface_guard";
  table.header.frame_id = "base_link";
  table.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_shape.dimensions = {0.45, 0.54, 2.0};
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = -0.2925;
  table_pose.position.z = -1.014;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  scene.world.collision_objects.push_back(table);
  try
  {
    requireMatchingWorkcellGeometry(scene, workcell, "world", true);
    auto canonical_world = scene;
    canonical_world.world.collision_objects[0].header.frame_id = "world";
    requireMatchingWorkcellGeometry(canonical_world, workcell, "world", true);
    std::cout << "PASS exact table geometry in base_link and identity world frame\n";
  }
  catch (const std::runtime_error & error)
  {
    std::cerr << "FAIL valid workcell geometry: " << error.what() << '\n';
    ++failures;
  }
  expectRejected("wrong table dimensions", [&]() {
    auto wrong = scene;
    wrong.world.collision_objects[0].primitives[0].dimensions[0] += 0.01;
    requireMatchingWorkcellGeometry(wrong, workcell, "world", true);
  });
  expectRejected("wrong table position", [&]() {
    auto wrong = scene;
    wrong.world.collision_objects[0].primitive_poses[0].position.x += 0.01;
    requireMatchingWorkcellGeometry(wrong, workcell, "world", true);
  });
  expectRejected("rotated table", [&]() {
    auto wrong = scene;
    wrong.world.collision_objects[0].primitive_poses[0].orientation.z = 0.1;
    requireMatchingWorkcellGeometry(wrong, workcell, "world", true);
  });
  expectRejected("nonidentity world to base frame", [&]() {
    auto wrong = scene;
    wrong.world.collision_objects[0].header.frame_id = "world";
    requireMatchingWorkcellGeometry(wrong, workcell, "world", false);
  });

  const WorkcellGeometryExpectation measured_tool{
    -0.5175, -0.0675, -0.27, 0.27, -0.014, 0.02, {0.0, 0.0, 0.1}};
  auto scene_with_tool = scene;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool0";
  attached.touch_links = {"tool0", "flange", "wrist_3_link"};
  attached.object.id = "measured_print_tool";
  attached.object.header.frame_id = "tool0";
  attached.object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive printhead_box;
  printhead_box.type = shape_msgs::msg::SolidPrimitive::BOX;
  printhead_box.dimensions = {
    ur3e_print_planner::kPrintheadBoxSizeX,
    ur3e_print_planner::kPrintheadBoxSizeY,
    ur3e_print_planner::kPrintheadBoxSizeZ};
  geometry_msgs::msg::Pose printhead_box_pose;
  printhead_box_pose.position.x = ur3e_print_planner::kPrintheadBoxCenterX;
  printhead_box_pose.position.y = ur3e_print_planner::kPrintheadBoxCenterY;
  printhead_box_pose.position.z = ur3e_print_planner::kPrintheadBoxCenterZ;
  printhead_box_pose.orientation.w = 1.0;
  attached.object.primitives.push_back(printhead_box);
  attached.object.primitive_poses.push_back(printhead_box_pose);
  scene_with_tool.robot_state.attached_collision_objects.push_back(attached);
  try
  {
    requireMatchingWorkcellGeometry(scene_with_tool, measured_tool, "world", true);
    std::cout << "PASS exact measured tool collision geometry\n";
  }
  catch (const std::runtime_error & error)
  {
    std::cerr << "FAIL valid measured tool: " << error.what() << '\n';
    ++failures;
  }
  const double nozzle_boundary_z = ur3e_print_planner::kPrintheadBoxCenterZ +
    ur3e_print_planner::kPrintheadBoxSizeZ / 2.0;
  if (!ur3e_print_planner::tcpInsidePrintheadEnvelope(0.03255, 0.0344, nozzle_boundary_z) ||
    ur3e_print_planner::tcpInsidePrintheadEnvelope(0.03255, 0.0344, nozzle_boundary_z + 0.00001))
  {
    std::cerr << "FAIL TCP enclosure at the measured nozzle boundary\n";
    ++failures;
  }
  else
  {
    std::cout << "PASS TCP enclosure at the measured nozzle boundary\n";
  }
  expectRejected("TCP outside measured printhead envelope", [&]() {
    auto wrong = measured_tool;
    wrong.tcp_offset_m[2] = nozzle_boundary_z + 0.00001;
    requireMatchingWorkcellGeometry(scene_with_tool, wrong, "world", true);
  });
  ur3e_print_planner::requirePrintheadGeometryIdentity(
    ur3e_print_planner::kPrintheadGeometrySha256);
  expectRejected("missing printhead build identity", [&]() {
    ur3e_print_planner::requirePrintheadGeometryIdentity("");
  });
  expectRejected("stale printhead build identity", [&]() {
    ur3e_print_planner::requirePrintheadGeometryIdentity(std::string(64, '0'));
  });
  expectRejected("wrong measured tool touch links", [&]() {
    auto wrong = scene_with_tool;
    wrong.robot_state.attached_collision_objects[0].touch_links.pop_back();
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("missing measured printhead envelope", [&]() {
    auto wrong = scene_with_tool;
    wrong.robot_state.attached_collision_objects[0].object.primitives.pop_back();
    wrong.robot_state.attached_collision_objects[0].object.primitive_poses.pop_back();
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("extra measured print-tool primitive", [&]() {
    auto wrong = scene_with_tool;
    auto & object = wrong.robot_state.attached_collision_objects[0].object;
    object.primitives.push_back(printhead_box);
    object.primitive_poses.push_back(printhead_box_pose);
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("wrong printhead envelope dimensions", [&]() {
    auto wrong = scene_with_tool;
    wrong.robot_state.attached_collision_objects[0].object.primitives[0].dimensions[0] -= 0.01;
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("wrong printhead envelope center", [&]() {
    auto wrong = scene_with_tool;
    wrong.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.y += 0.01;
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("rotated printhead envelope", [&]() {
    auto wrong = scene_with_tool;
    auto & orientation =
      wrong.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation;
    orientation.z = std::sin(0.1);
    orientation.w = std::cos(0.1);
    requireMatchingWorkcellGeometry(wrong, measured_tool, "world", true);
  });
  expectRejected("unexpected measured tool", [&]() {
    requireMatchingWorkcellGeometry(scene_with_tool, workcell, "world", true);
  });

  expectRejected("wrong joint header", [&]() {
    parseJointSampleCsv(writeCsv(
      "wrong_header",
      "shoulder_lift_joint,shoulder_pan_joint,elbow_joint,"
      "wrist_1_joint,wrist_2_joint,wrist_3_joint\n0,0,0,0,0,0\n"));
  });
  expectRejected("legacy candidate-only protocol", [&]() {
    parseJointSampleCsv(writeCsv(
      "candidate_only",
      "shoulder_pan_joint,shoulder_lift_joint,elbow_joint,"
      "wrist_1_joint,wrist_2_joint,wrist_3_joint\n0,0,0,0,0,0\n"));
  });
  expectRejected("missing reference joint", [&]() {
    parseJointSampleCsv(writeCsv(
      "missing_reference", header + "0,0,0,0,0,0,0,0,0,0,0\n"));
  });
  expectRejected("non-finite joint", [&]() {
    parseJointSampleCsv(writeCsv(
      "nonfinite", header + "0,0,nan,0,0,0,0,0,0,0,0,0\n"));
  });
  expectRejected("wrong column count", [&]() {
    parseJointSampleCsv(writeCsv("short", header + "0,0,0,0,0,0,0,0,0,0\n"));
  });
  expectRejected("blank data row", [&]() {
    parseJointSampleCsv(writeCsv(
      "blank", header + "0,0,0,0,0,0,0,0,0,0,0,0\n\n"));
  });
  expectRejected("missing data rows", [&]() {
    parseJointSampleCsv(writeCsv("header_only", header));
  });
  expectRejected("relative input path", [&]() {
    parseJointSampleCsv(std::filesystem::path("relative.csv"));
  });
  expectRejected("more than 200000 data rows", [&]() {
    std::string excessive = header;
    excessive.reserve(header.size() +
      (ur3e_print_planner::kMaximumJointSampleRows + 1) * 12);
    for (std::size_t row = 0;
      row <= ur3e_print_planner::kMaximumJointSampleRows; ++row)
    {
      excessive += "0,0,0,0,0,0,0,0,0,0,0,0\n";
    }
    parseJointSampleCsv(writeCsv("too_many", excessive));
  });

  try
  {
    const auto samples = parseJointSampleCsv(valid_path);
    size_t callbacks = 0;
    expectRejected("collision callback failure", [&]() {
      validateJointSamples(samples, [&callbacks](const auto &) {
        ++callbacks;
        return callbacks != 2;
      });
    });
    if (callbacks != 2)
    {
      std::cerr << "FAIL collision callback did not stop on the invalid sample\n";
      ++failures;
    }
  }
  catch (const std::runtime_error & error)
  {
    std::cerr << "FAIL callback fixture: " << error.what() << '\n';
    ++failures;
  }

  std::error_code ignored;
  std::filesystem::remove_all(valid_path.parent_path(), ignored);
  return failures == 0 ? 0 : 1;
}
