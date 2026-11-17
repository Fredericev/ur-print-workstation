#include "joint_sample_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <shape_msgs/msg/solid_primitive.hpp>
#include "printhead_collision_geometry.hpp"

namespace ur3e_print_planner
{
namespace
{

constexpr std::uintmax_t kMaximumCsvBytes = 64U * 1024U * 1024U;
constexpr std::size_t kMaximumCsvLineBytes = 1024;
constexpr double kGeometryTolerance = 1.0e-6;
constexpr double kExpectedTableGuardDepthM = 2.0;

using Vector3 = std::array<double, 3>;

bool near(const double first, const double second)
{
  return std::isfinite(first) && std::isfinite(second) &&
         std::abs(first - second) <= kGeometryTolerance;
}

Vector3 position(const geometry_msgs::msg::Point & value, const std::string & label)
{
  const Vector3 result{value.x, value.y, value.z};
  if (!std::all_of(result.begin(), result.end(), [](const double item) {
      return std::isfinite(item);
    }))
  {
    throw std::runtime_error(label + " contains a non-finite position");
  }
  return result;
}

std::array<double, 4> unitQuaternion(
  const geometry_msgs::msg::Quaternion & value,
  const std::string & label)
{
  std::array<double, 4> quaternion{value.x, value.y, value.z, value.w};
  if (!std::all_of(quaternion.begin(), quaternion.end(), [](const double item) {
      return std::isfinite(item);
    }))
  {
    throw std::runtime_error(label + " contains a non-finite orientation");
  }
  const double norm = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > kGeometryTolerance)
  {
    throw std::runtime_error(label + " orientation is not a unit quaternion");
  }
  return quaternion;
}

void requireAxisAligned(
  const geometry_msgs::msg::Quaternion & value,
  const std::string & label)
{
  const auto quaternion = unitQuaternion(value, label);
  if (std::abs(quaternion[0]) > kGeometryTolerance ||
    std::abs(quaternion[1]) > kGeometryTolerance ||
    std::abs(quaternion[2]) > kGeometryTolerance ||
    std::abs(std::abs(quaternion[3]) - 1.0) > kGeometryTolerance)
  {
    throw std::runtime_error(label + " is rotated relative to base_link");
  }
}

Vector3 rotate(
  const geometry_msgs::msg::Quaternion & value,
  const Vector3 & vector,
  const std::string & label)
{
  const auto quaternion = unitQuaternion(value, label);
  const double x = quaternion[0];
  const double y = quaternion[1];
  const double z = quaternion[2];
  const double w = quaternion[3];
  const Vector3 cross{
    y * vector[2] - z * vector[1],
    z * vector[0] - x * vector[2],
    x * vector[1] - y * vector[0],
  };
  const Vector3 second_cross{
    y * cross[2] - z * cross[1],
    z * cross[0] - x * cross[2],
    x * cross[1] - y * cross[0],
  };
  return {
    vector[0] + 2.0 * (w * cross[0] + second_cross[0]),
    vector[1] + 2.0 * (w * cross[1] + second_cross[1]),
    vector[2] + 2.0 * (w * cross[2] + second_cross[2]),
  };
}

Vector3 add(const Vector3 & first, const Vector3 & second)
{
  return {
    first[0] + second[0], first[1] + second[1], first[2] + second[2]};
}

void requireExpectedValues(const WorkcellGeometryExpectation & expected)
{
  const std::array<double, 9> values{
    expected.bed_min_x_m, expected.bed_max_x_m,
    expected.bed_min_y_m, expected.bed_max_y_m,
    expected.bed_plane_z_m, expected.tool_collision_radius_m,
    expected.tcp_offset_m[0], expected.tcp_offset_m[1], expected.tcp_offset_m[2],
  };
  if (!std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    }) || expected.bed_min_x_m >= expected.bed_max_x_m ||
    expected.bed_min_y_m >= expected.bed_max_y_m ||
    expected.tool_collision_radius_m < 0.0)
  {
    throw std::runtime_error("expected workcell geometry is invalid");
  }
}

std::string expectedHeader()
{
  std::ostringstream header;
  for (const auto prefix : {"candidate_", "reference_"})
  {
    for (std::size_t index = 0; index < kUrJointNames.size(); ++index)
    {
      if (header.tellp() > 0)
      {
        header << ',';
      }
      header << prefix << kUrJointNames[index];
    }
  }
  return header.str();
}

std::array<std::string, 12> splitStrictRow(
  const std::string & line,
  const std::size_t line_number)
{
  std::array<std::string, 12> fields;
  std::size_t begin = 0;
  for (std::size_t index = 0; index < fields.size(); ++index)
  {
    const auto comma = line.find(',', begin);
    if (index + 1 == fields.size())
    {
      if (comma != std::string::npos)
      {
        throw std::runtime_error(
                "CSV line " + std::to_string(line_number) + " does not contain exactly six columns");
      }
      fields[index] = line.substr(begin);
    }
    else
    {
      if (comma == std::string::npos)
      {
        throw std::runtime_error(
                "CSV line " + std::to_string(line_number) + " does not contain exactly six columns");
      }
      fields[index] = line.substr(begin, comma - begin);
      begin = comma + 1;
    }
  }
  return fields;
}

double parseFiniteNumber(const std::string & text, const std::size_t line_number)
{
  // std::stod accepts leading whitespace, NaN, infinity and hexadecimal
  // literals. None of those belong in the private CSV protocol.
  static const std::regex decimal_pattern(
    R"([+-]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][+-]?[0-9]+)?)",
    std::regex::ECMAScript);
  if (!std::regex_match(text, decimal_pattern))
  {
    throw std::runtime_error(
            "CSV line " + std::to_string(line_number) + " contains an invalid joint value");
  }
  try
  {
    std::size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value))
    {
      throw std::runtime_error("non-finite joint value");
    }
    return value;
  }
  catch (const std::exception &)
  {
    throw std::runtime_error(
            "CSV line " + std::to_string(line_number) + " contains an invalid joint value");
  }
}

}  // namespace

JointSampleSequence parseJointSampleCsv(const std::filesystem::path & path)
{
  if (path.empty() || !path.is_absolute() || path != path.lexically_normal())
  {
    throw std::runtime_error("sample_csv must be a normalized absolute path");
  }
  if (path.extension() != ".csv")
  {
    throw std::runtime_error("sample_csv must name a .csv file");
  }

  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (error || !std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status))
  {
    throw std::runtime_error("sample_csv must be an existing non-symlink regular file");
  }
  const auto bytes = std::filesystem::file_size(path, error);
  if (error || bytes == 0 || bytes > kMaximumCsvBytes)
  {
    throw std::runtime_error("sample_csv has an invalid or excessive file size");
  }

  std::ifstream stream(path, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("sample_csv could not be opened");
  }

  std::string line;
  if (!std::getline(stream, line))
  {
    throw std::runtime_error("sample_csv is empty");
  }
  if (!line.empty() && line.back() == '\r')
  {
    line.pop_back();
  }
  if (line != expectedHeader())
  {
    throw std::runtime_error("sample_csv does not have the exact six-joint UR header");
  }

  JointSampleSequence samples;
  samples.reserve(4096);
  std::size_t line_number = 1;
  while (std::getline(stream, line))
  {
    ++line_number;
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    if (line.empty())
    {
      throw std::runtime_error("sample_csv contains a blank data row");
    }
    if (line.size() > kMaximumCsvLineBytes)
    {
      throw std::runtime_error("sample_csv contains an excessive data row");
    }
    if (samples.size() == kMaximumJointSampleRows)
    {
      throw std::runtime_error("sample_csv exceeds the 200000-state validation limit");
    }
    const auto fields = splitStrictRow(line, line_number);
    JointSamplePair pair{};
    for (std::size_t index = 0; index < pair.candidate.size(); ++index)
    {
      pair.candidate[index] = parseFiniteNumber(fields[index], line_number);
      pair.reference[index] = parseFiniteNumber(
        fields[index + pair.candidate.size()], line_number);
    }
    samples.push_back(pair);
  }
  if (!stream.eof())
  {
    throw std::runtime_error("sample_csv could not be read completely");
  }
  if (samples.empty())
  {
    throw std::runtime_error("sample_csv must contain at least one joint state");
  }
  return samples;
}

std::size_t validateJointSamples(
  const JointSampleSequence & samples,
  const JointSampleValidator & validator)
{
  if (samples.empty() || samples.size() > kMaximumJointSampleRows)
  {
    throw std::runtime_error("joint sample sequence has an invalid length");
  }
  if (!validator)
  {
    throw std::runtime_error("joint sample validator is unavailable");
  }
  for (std::size_t index = 0; index < samples.size(); ++index)
  {
    if (!validator(samples[index].candidate))
    {
      throw std::runtime_error(
              "joint state " + std::to_string(index + 1) + " is out of bounds or in collision");
    }
  }
  return samples.size();
}

void requireMatchingWorkcellGeometry(
  const moveit_msgs::msg::PlanningScene & scene,
  const WorkcellGeometryExpectation & expected,
  const std::string & model_frame,
  const bool model_frame_to_base_is_identity)
{
  requireExpectedValues(expected);
  constexpr char table_id[] = "confirmed_table_surface_guard";
  const auto & objects = scene.world.collision_objects;
  const auto table_count = std::count_if(
    objects.begin(), objects.end(), [&table_id](const auto & object) {
      return object.id == table_id;
    });
  if (table_count != 1)
  {
    throw std::runtime_error(
            "planning scene does not contain exactly one confirmed table guard");
  }
  const auto table = std::find_if(
    objects.begin(), objects.end(), [&table_id](const auto & object) {
      return object.id == table_id;
    });
  if (table->header.frame_id != "base_link" &&
    !(table->header.frame_id == model_frame && model_frame_to_base_is_identity))
  {
    throw std::runtime_error("confirmed table guard frame does not match base_link");
  }
  if (table->primitives.size() != 1 || table->primitive_poses.size() != 1)
  {
    throw std::runtime_error("confirmed table guard does not have one primitive");
  }
  const auto & shape = table->primitives.front();
  const auto & shape_pose = table->primitive_poses.front();
  if (shape.type != shape_msgs::msg::SolidPrimitive::BOX ||
    shape.dimensions.size() != 3)
  {
    throw std::runtime_error("confirmed table guard is not one box");
  }
  const std::array<double, 3> dimensions{
    shape.dimensions[0], shape.dimensions[1], shape.dimensions[2]};
  const std::array<double, 3> expected_dimensions{
    expected.bed_max_x_m - expected.bed_min_x_m,
    expected.bed_max_y_m - expected.bed_min_y_m,
    kExpectedTableGuardDepthM,
  };
  for (std::size_t index = 0; index < dimensions.size(); ++index)
  {
    if (!near(dimensions[index], expected_dimensions[index]))
    {
      throw std::runtime_error("confirmed table guard dimensions do not match the manifest");
    }
  }
  requireAxisAligned(table->pose.orientation, "table parent pose");
  requireAxisAligned(shape_pose.orientation, "table primitive pose");
  const auto center = add(
    position(table->pose.position, "table parent pose"),
    position(shape_pose.position, "table primitive pose"));
  const Vector3 expected_center{
    (expected.bed_min_x_m + expected.bed_max_x_m) / 2.0,
    (expected.bed_min_y_m + expected.bed_max_y_m) / 2.0,
    expected.bed_plane_z_m - kExpectedTableGuardDepthM / 2.0,
  };
  for (std::size_t index = 0; index < center.size(); ++index)
  {
    if (!near(center[index], expected_center[index]))
    {
      throw std::runtime_error("confirmed table guard position does not match the manifest");
    }
  }

  constexpr char tool_id[] = "measured_print_tool";
  const auto & attachments = scene.robot_state.attached_collision_objects;
  const auto tool_count = std::count_if(
    attachments.begin(), attachments.end(), [&tool_id](const auto & attached) {
      return attached.object.id == tool_id;
    });
  const bool tool_required = expected.tool_collision_radius_m > 0.0;
  if ((!tool_required && tool_count != 0) || (tool_required && tool_count != 1))
  {
    throw std::runtime_error(
            "measured print-tool attachment does not match the manifest");
  }
  if (!tool_required)
  {
    return;
  }
  if (!tcpInsidePrintheadEnvelope(
      expected.tcp_offset_m[0], expected.tcp_offset_m[1], expected.tcp_offset_m[2]))
  {
    throw std::runtime_error("measured TCP falls outside the installed printhead CAD envelope");
  }

  const auto attached = std::find_if(
    attachments.begin(), attachments.end(), [&tool_id](const auto & item) {
      return item.object.id == tool_id;
    });
  const std::set<std::string> touch_links(
    attached->touch_links.begin(), attached->touch_links.end());
  const std::set<std::string> expected_touch_links{
    "tool0", "flange", "wrist_3_link"};
  const auto & tool = attached->object;
  if (attached->link_name != "tool0" || tool.header.frame_id != "tool0" ||
    attached->touch_links.size() != expected_touch_links.size() ||
    touch_links != expected_touch_links || tool.primitives.size() != 1 ||
    tool.primitive_poses.size() != 1 || !tool.meshes.empty() ||
    !tool.mesh_poses.empty() || !tool.planes.empty() ||
    !tool.plane_poses.empty())
  {
    throw std::runtime_error("measured print-tool frame or shape count does not match the manifest");
  }
  // The reviewed whole-head CAD envelope is the only production tool
  // primitive. The old full-length TCP cylinder projected below the nozzle
  // tip and falsely intersected the tabletop at a printable first layer.
  const auto & head_shape = tool.primitives.front();
  const auto & head_shape_pose = tool.primitive_poses.front();
  const std::array<double, 3> expected_head_dimensions{
    kPrintheadBoxSizeX, kPrintheadBoxSizeY, kPrintheadBoxSizeZ};
  if (head_shape.type != shape_msgs::msg::SolidPrimitive::BOX ||
    head_shape.dimensions.size() != expected_head_dimensions.size())
  {
    throw std::runtime_error("measured printhead envelope is not the expected box");
  }
  for (std::size_t index = 0; index < expected_head_dimensions.size(); ++index)
  {
    if (!near(head_shape.dimensions[index], expected_head_dimensions[index]))
    {
      throw std::runtime_error("measured printhead envelope dimensions do not match the CAD bounds");
    }
  }
  const auto head_center_in_parent = rotate(
    tool.pose.orientation,
    position(head_shape_pose.position, "printhead envelope primitive pose"),
    "tool parent pose");
  const auto head_center = add(
    position(tool.pose.position, "tool parent pose"), head_center_in_parent);
  const Vector3 expected_head_center{
    kPrintheadBoxCenterX, kPrintheadBoxCenterY, kPrintheadBoxCenterZ};
  for (std::size_t index = 0; index < expected_head_center.size(); ++index)
  {
    if (!near(head_center[index], expected_head_center[index]))
    {
      throw std::runtime_error("measured printhead envelope center does not match the CAD bounds");
    }
  }
  for (const auto & axis : {Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
    Vector3{0.0, 0.0, 1.0}})
  {
    const auto composed_axis = rotate(
      tool.pose.orientation,
      rotate(head_shape_pose.orientation, axis, "printhead envelope primitive pose"),
      "tool parent pose");
    for (std::size_t index = 0; index < axis.size(); ++index)
    {
      if (!near(composed_axis[index], axis[index]))
      {
        throw std::runtime_error("measured printhead envelope orientation does not match tool0");
      }
    }
  }
}

}  // namespace ur3e_print_planner
