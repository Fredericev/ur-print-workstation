#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

struct PrintWaypoint
{
  int layer;
  int index;
  std::string segment;
  geometry_msgs::msg::Pose pose;
};

struct PrintConfig
{
  double box_x_m;
  double box_y_m;
  double box_z_m;
  double layer_height_m;
  double line_spacing_m;
  double center_x_m;
  double center_y_m;
  double base_z_m;
  double approach_height_m;
  double eef_step_m;
  double jump_threshold;
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
  double bed_top_z_m;
  double bed_thickness_m;
  double bed_padding_m;
  bool add_bed_collision;
  bool use_current_state_as_start;
  bool execute_trajectory;
  bool playback_approach_trajectory;
  std::string planning_group;
  std::string end_effector_link;
  std::string start_state_name;
  std::string output_csv_path;
  std::string output_summary_path;
};

constexpr double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

geometry_msgs::msg::Quaternion makeQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

PrintConfig loadConfig(const rclcpp::Node::SharedPtr & node)
{
  auto declare_if_missing = [&node](const std::string & name, const auto & value) {
    if (!node->has_parameter(name))
    {
      node->declare_parameter(name, value);
    }
  };

  declare_if_missing("box_x_mm", 50.0);
  declare_if_missing("box_y_mm", 50.0);
  declare_if_missing("box_z_mm", 10.0);
  declare_if_missing("layer_height_mm", 2.0);
  declare_if_missing("line_spacing_mm", 5.0);
  declare_if_missing("center_x_m", 0.20);
  declare_if_missing("center_y_m", -0.25);
  declare_if_missing("base_z_m", 0.20);
  declare_if_missing("approach_height_mm", 30.0);
  declare_if_missing("eef_step_mm", 4.0);
  declare_if_missing("jump_threshold", 0.0);
  declare_if_missing("roll_deg", 180.0);
  declare_if_missing("pitch_deg", 0.0);
  declare_if_missing("yaw_deg", -90.0);
  declare_if_missing("bed_top_z_m", 0.0);
  declare_if_missing("bed_thickness_m", 0.02);
  declare_if_missing("bed_padding_m", 0.05);
  declare_if_missing("add_bed_collision", true);
  declare_if_missing("use_current_state_as_start", false);
  declare_if_missing("execute_trajectory", false);
  declare_if_missing("playback_approach_trajectory", false);
  declare_if_missing("planning_group", "ur_manipulator");
  declare_if_missing("end_effector_link", "tool0");
  declare_if_missing("start_state_name", "up");
  declare_if_missing(
    "output_csv_path", "/home/ubuntu/ur_ros2_ws/output/print_box_50x50x10_toolpath.csv");
  declare_if_missing(
    "output_summary_path", "/home/ubuntu/ur_ros2_ws/output/print_box_50x50x10_summary.yaml");

  PrintConfig cfg;
  cfg.box_x_m = node->get_parameter("box_x_mm").as_double() / 1000.0;
  cfg.box_y_m = node->get_parameter("box_y_mm").as_double() / 1000.0;
  cfg.box_z_m = node->get_parameter("box_z_mm").as_double() / 1000.0;
  cfg.layer_height_m = node->get_parameter("layer_height_mm").as_double() / 1000.0;
  cfg.line_spacing_m = node->get_parameter("line_spacing_mm").as_double() / 1000.0;
  cfg.center_x_m = node->get_parameter("center_x_m").as_double();
  cfg.center_y_m = node->get_parameter("center_y_m").as_double();
  cfg.base_z_m = node->get_parameter("base_z_m").as_double();
  cfg.approach_height_m = node->get_parameter("approach_height_mm").as_double() / 1000.0;
  cfg.eef_step_m = node->get_parameter("eef_step_mm").as_double() / 1000.0;
  cfg.jump_threshold = node->get_parameter("jump_threshold").as_double();
  cfg.roll_rad = deg_to_rad(node->get_parameter("roll_deg").as_double());
  cfg.pitch_rad = deg_to_rad(node->get_parameter("pitch_deg").as_double());
  cfg.yaw_rad = deg_to_rad(node->get_parameter("yaw_deg").as_double());
  cfg.bed_top_z_m = node->get_parameter("bed_top_z_m").as_double();
  cfg.bed_thickness_m = node->get_parameter("bed_thickness_m").as_double();
  cfg.bed_padding_m = node->get_parameter("bed_padding_m").as_double();
  cfg.add_bed_collision = node->get_parameter("add_bed_collision").as_bool();
  cfg.use_current_state_as_start = node->get_parameter("use_current_state_as_start").as_bool();
  cfg.execute_trajectory = node->get_parameter("execute_trajectory").as_bool();
  cfg.playback_approach_trajectory = node->get_parameter("playback_approach_trajectory").as_bool();
  cfg.planning_group = node->get_parameter("planning_group").as_string();
  cfg.end_effector_link = node->get_parameter("end_effector_link").as_string();
  cfg.start_state_name = node->get_parameter("start_state_name").as_string();
  cfg.output_csv_path = node->get_parameter("output_csv_path").as_string();
  cfg.output_summary_path = node->get_parameter("output_summary_path").as_string();
  return cfg;
}

std::vector<PrintWaypoint> buildToolpath(const PrintConfig & cfg)
{
  std::vector<PrintWaypoint> waypoints;
  const auto orientation = makeQuaternion(cfg.roll_rad, cfg.pitch_rad, cfg.yaw_rad);

  const double half_x = cfg.box_x_m / 2.0;
  const double half_y = cfg.box_y_m / 2.0;
  const double min_x = cfg.center_x_m - half_x;
  const double max_x = cfg.center_x_m + half_x;
  const double min_y = cfg.center_y_m - half_y;
  const double max_y = cfg.center_y_m + half_y;
  const int layer_count = std::max(1, static_cast<int>(std::ceil(cfg.box_z_m / cfg.layer_height_m)));

  auto push_pose = [&](int layer, int & index, double x, double y, double z, const std::string & segment) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;
    waypoints.push_back(PrintWaypoint{layer, index++, segment, pose});
  };

  for (int layer = 0; layer < layer_count; ++layer)
  {
    const double z = cfg.base_z_m + static_cast<double>(layer) * cfg.layer_height_m;
    int index = 0;

    push_pose(layer, index, min_x, min_y, z, "perimeter");
    push_pose(layer, index, max_x, min_y, z, "perimeter");
    push_pose(layer, index, max_x, max_y, z, "perimeter");
    push_pose(layer, index, min_x, max_y, z, "perimeter");
    push_pose(layer, index, min_x, min_y, z, "perimeter_close");

    bool left_to_right = true;
    for (double y = min_y + cfg.line_spacing_m; y <= max_y - cfg.line_spacing_m + 1e-9;
         y += cfg.line_spacing_m)
    {
      if (left_to_right)
      {
        push_pose(layer, index, min_x, y, z, "infill");
        push_pose(layer, index, max_x, y, z, "infill");
      }
      else
      {
        push_pose(layer, index, max_x, y, z, "infill");
        push_pose(layer, index, min_x, y, z, "infill");
      }
      left_to_right = !left_to_right;
    }
  }

  return waypoints;
}

void writeToolpathCsv(const std::vector<PrintWaypoint> & waypoints, const std::string & path)
{
  std::ofstream out(path);
  out << "layer,index,segment,x_m,y_m,z_m,qx,qy,qz,qw\n";
  out << std::fixed << std::setprecision(6);
  for (const auto & waypoint : waypoints)
  {
    out << waypoint.layer << ',' << waypoint.index << ',' << waypoint.segment << ','
        << waypoint.pose.position.x << ',' << waypoint.pose.position.y << ',' << waypoint.pose.position.z
        << ',' << waypoint.pose.orientation.x << ',' << waypoint.pose.orientation.y << ','
        << waypoint.pose.orientation.z << ',' << waypoint.pose.orientation.w << '\n';
  }
}

void writeSummary(
  const PrintConfig & cfg, size_t waypoint_count, double cartesian_fraction,
  const std::string & approach_status, const std::string & path)
{
  std::ofstream out(path);
  out << std::fixed << std::setprecision(4);
  out << "box_mm: [" << cfg.box_x_m * 1000.0 << ", " << cfg.box_y_m * 1000.0 << ", "
      << cfg.box_z_m * 1000.0 << "]\n";
  out << "layer_height_mm: " << cfg.layer_height_m * 1000.0 << "\n";
  out << "line_spacing_mm: " << cfg.line_spacing_m * 1000.0 << "\n";
  out << "print_center_m: [" << cfg.center_x_m << ", " << cfg.center_y_m << ", " << cfg.base_z_m
      << "]\n";
  out << "orientation_deg: [" << cfg.roll_rad * 180.0 / M_PI << ", " << cfg.pitch_rad * 180.0 / M_PI
      << ", " << cfg.yaw_rad * 180.0 / M_PI << "]\n";
  out << "waypoint_count: " << waypoint_count << "\n";
  out << "approach_plan: " << approach_status << "\n";
  out << "cartesian_fraction: " << cartesian_fraction << "\n";
}

void addPrintBedCollision(
  moveit::planning_interface::PlanningSceneInterface & planning_scene_interface,
  const PrintConfig & cfg)
{
  if (!cfg.add_bed_collision)
  {
    return;
  }

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "print_bed";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {
    cfg.box_x_m + 2.0 * cfg.bed_padding_m,
    cfg.box_y_m + 2.0 * cfg.bed_padding_m,
    cfg.bed_thickness_m};

  geometry_msgs::msg::Pose bed_pose;
  bed_pose.orientation.w = 1.0;
  bed_pose.position.x = cfg.center_x_m;
  bed_pose.position.y = cfg.center_y_m;
  bed_pose.position.z = cfg.bed_top_z_m - cfg.bed_thickness_m / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(bed_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void playbackJointTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & joint_state_pub,
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    throw std::runtime_error("joint trajectory is empty");
  }

  rclcpp::Duration last_from_start(0, 0);
  for (const auto & point : trajectory.points)
  {
    const rclcpp::Duration current_from_start(point.time_from_start);
    const rclcpp::Duration delta = current_from_start - last_from_start;
    if (delta.nanoseconds() > 0)
    {
      rclcpp::sleep_for(std::chrono::nanoseconds(delta.nanoseconds()));
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name = trajectory.joint_names;
    msg.position = point.positions;
    msg.velocity = point.velocities;
    msg.effort = point.effort;
    joint_state_pub->publish(msg);

    last_from_start = current_from_start;
  }
}

visualization_msgs::msg::Marker makePrintedPathMarker(
  const rclcpp::Node::SharedPtr & node, const std::vector<geometry_msgs::msg::Point> & points)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = node->now();
  marker.ns = "ur3e_print";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.004;
  marker.color.r = 0.35;
  marker.color.g = 0.85;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.points = points;
  return marker;
}

void publishJointTrajectoryPoint(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & joint_state_pub,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  size_t point_index)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    throw std::runtime_error("joint trajectory is empty");
  }

  const auto & point = trajectory.points.at(point_index);
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node->now();
  msg.name = trajectory.joint_names;
  msg.position = point.positions;
  msg.velocity = point.velocities;
  msg.effort = point.effort;
  joint_state_pub->publish(msg);
}

visualization_msgs::msg::Marker makeDeletePrintedPathMarker(const rclcpp::Node::SharedPtr & node)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = node->now();
  marker.ns = "ur3e_print";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

void playbackPrintTrajectoryWithMarkers(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & joint_state_pub,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_pub,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotState & trajectory_start_state,
  const moveit::core::JointModelGroup * joint_model_group,
  const std::string & end_effector_link)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    throw std::runtime_error("joint trajectory is empty");
  }

  std::vector<geometry_msgs::msg::Point> printed_points;
  printed_points.reserve(trajectory.points.size());
  moveit::core::RobotState playback_state = trajectory_start_state;

  rclcpp::Duration last_from_start(0, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto & point = trajectory.points[i];
    const rclcpp::Duration current_from_start(point.time_from_start);
    const rclcpp::Duration delta = current_from_start - last_from_start;
    if (delta.nanoseconds() > 0)
    {
      rclcpp::sleep_for(std::chrono::nanoseconds(delta.nanoseconds()));
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name = trajectory.joint_names;
    msg.position = point.positions;
    msg.velocity = point.velocities;
    msg.effort = point.effort;
    joint_state_pub->publish(msg);

    playback_state.setJointGroupPositions(joint_model_group, point.positions);
    playback_state.updateLinkTransforms();
    const auto & tool_transform = playback_state.getGlobalLinkTransform(end_effector_link);

    geometry_msgs::msg::Point printed_point;
    printed_point.x = tool_transform.translation().x();
    printed_point.y = tool_transform.translation().y();
    printed_point.z = tool_transform.translation().z();

    if (printed_points.empty())
    {
      printed_points.push_back(printed_point);
    }
    else
    {
      const auto & last_point = printed_points.back();
      const double dx = printed_point.x - last_point.x;
      const double dy = printed_point.y - last_point.y;
      const double dz = printed_point.z - last_point.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) >= 0.001)
      {
        printed_points.push_back(printed_point);
      }
    }

    if (printed_points.size() >= 2)
    {
      marker_pub->publish(makePrintedPathMarker(node, printed_points));
    }
    last_from_start = current_from_start;
  }

  if (printed_points.size() >= 2)
  {
    marker_pub->publish(makePrintedPathMarker(node, printed_points));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur3e_print_box_planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  try
  {
    const PrintConfig cfg = loadConfig(node);
    const auto toolpath = buildToolpath(cfg);
    writeToolpathCsv(toolpath, cfg.output_csv_path);

    moveit::planning_interface::MoveGroupInterface move_group(node, cfg.planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addPrintBedCollision(planning_scene_interface, cfg);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink(cfg.end_effector_link);

    moveit::core::RobotState start_state(move_group.getRobotModel());
    const moveit::core::JointModelGroup * joint_model_group =
      start_state.getJointModelGroup(cfg.planning_group);
    if (joint_model_group == nullptr)
    {
      throw std::runtime_error("planning group not found: " + cfg.planning_group);
    }

    if (cfg.use_current_state_as_start)
    {
      move_group.setStartStateToCurrentState();
      start_state = *move_group.getCurrentState(10.0);
    }
    else
    {
      start_state.setToDefaultValues(joint_model_group, cfg.start_state_name);
      move_group.setStartState(start_state);
    }

    geometry_msgs::msg::Pose approach_pose = toolpath.front().pose;
    approach_pose.position.z += cfg.approach_height_m;

    move_group.setPoseTarget(approach_pose);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    const auto approach_result = move_group.plan(approach_plan);
    const bool approach_ok = static_cast<bool>(approach_result);
    const std::string approach_status = approach_ok ? "success" : "failed";

    if (!approach_ok)
    {
      writeSummary(cfg, toolpath.size(), 0.0, approach_status, cfg.output_summary_path);
      throw std::runtime_error("failed to plan approach pose");
    }

    robot_trajectory::RobotTrajectory approach_robot_trajectory(move_group.getRobotModel(), cfg.planning_group);
    approach_robot_trajectory.setRobotTrajectoryMsg(start_state, approach_plan.trajectory_);
    moveit::core::RobotState travel_start_state = approach_robot_trajectory.getLastWayPoint();
    move_group.setStartState(travel_start_state);

    std::vector<geometry_msgs::msg::Pose> travel_poses;
    travel_poses.push_back(toolpath.front().pose);

    moveit_msgs::msg::RobotTrajectory travel_trajectory;
    const double travel_fraction = move_group.computeCartesianPath(
      travel_poses, cfg.eef_step_m, cfg.jump_threshold, travel_trajectory, true);

    if (travel_fraction < 0.99)
    {
      writeSummary(cfg, toolpath.size(), travel_fraction, approach_status, cfg.output_summary_path);
      throw std::runtime_error("failed to compute collision-free travel descent to print start");
    }

    robot_trajectory::RobotTrajectory travel_robot_trajectory(move_group.getRobotModel(), cfg.planning_group);
    travel_robot_trajectory.setRobotTrajectoryMsg(travel_start_state, travel_trajectory);
    moveit::core::RobotState print_start_state = travel_robot_trajectory.getLastWayPoint();
    move_group.setStartState(print_start_state);

    std::vector<geometry_msgs::msg::Pose> print_poses;
    print_poses.reserve(toolpath.size());
    for (size_t i = 1; i < toolpath.size(); ++i)
    {
      print_poses.push_back(toolpath[i].pose);
    }
    if (print_poses.empty())
    {
      print_poses.push_back(toolpath.front().pose);
    }

    moveit_msgs::msg::RobotTrajectory print_trajectory;
    const double print_fraction = move_group.computeCartesianPath(
      print_poses, cfg.eef_step_m, cfg.jump_threshold, print_trajectory, true);

    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local());

    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = "ur";
    moveit::core::robotStateToRobotStateMsg(start_state, display_trajectory.trajectory_start);
    display_trajectory.trajectory.push_back(approach_plan.trajectory_);
    display_trajectory.trajectory.push_back(travel_trajectory);
    display_trajectory.trajectory.push_back(print_trajectory);
    display_pub->publish(display_trajectory);

    writeSummary(cfg, toolpath.size(), print_fraction, approach_status, cfg.output_summary_path);

    RCLCPP_INFO(node->get_logger(), "Saved toolpath CSV: %s", cfg.output_csv_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Saved plan summary: %s", cfg.output_summary_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Waypoints: %zu", toolpath.size());
    RCLCPP_INFO(node->get_logger(), "Travel descent fraction: %.3f", travel_fraction);
    RCLCPP_INFO(node->get_logger(), "Print path fraction: %.3f", print_fraction);

    if (print_fraction < 0.99)
    {
      RCLCPP_WARN(node->get_logger(), "Print path fraction is below 0.99; tune pose or spacing.");
    }

    if (cfg.execute_trajectory)
    {
      if (print_fraction < 0.99)
      {
        throw std::runtime_error("refusing to execute because print path fraction is below 0.99");
      }

      auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      auto printed_path_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/printed_path", rclcpp::QoS(10).transient_local());
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      printed_path_pub->publish(makeDeletePrintedPathMarker(node));
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      if (cfg.playback_approach_trajectory)
      {
        RCLCPP_INFO(node->get_logger(), "Playing back approach trajectory in simulation...");
        playbackJointTrajectory(node, joint_state_pub, approach_plan.trajectory_.joint_trajectory);
      }
      else
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Skipping approach animation and placing robot at print start to avoid table collision preview.");
        const auto & approach_trajectory = approach_plan.trajectory_.joint_trajectory;
        publishJointTrajectoryPoint(node, joint_state_pub, approach_trajectory, approach_trajectory.points.size() - 1);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
      }

      RCLCPP_INFO(node->get_logger(), "Playing back non-printing travel descent in simulation...");
      playbackJointTrajectory(node, joint_state_pub, travel_trajectory.joint_trajectory);

      RCLCPP_INFO(node->get_logger(), "Playing back print trajectory in simulation...");
      playbackPrintTrajectoryWithMarkers(
        node, joint_state_pub, printed_path_pub, print_trajectory.joint_trajectory,
        print_start_state, joint_model_group, cfg.end_effector_link);
    }

    rclcpp::sleep_for(std::chrono::seconds(3));
  }
  catch (const std::exception & ex)
  {
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
