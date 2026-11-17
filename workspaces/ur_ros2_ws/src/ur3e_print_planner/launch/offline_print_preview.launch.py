"""Isolated RViz-only playback of a checksum-locked cached print review.

There is deliberately no move_group, ros2_control, controller manager, action
client, UR driver, URScript interface, or Klipper process in this launch file.
"""

import os
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *_args, **_kwargs):
    ur_type = LaunchConfiguration("ur_type")
    workspace_root = LaunchConfiguration("workspace_root")
    project_root = LaunchConfiguration("project_root")
    manifest = LaunchConfiguration("manifest")
    speed_factor = LaunchConfiguration("speed_factor")
    kinematics_path = LaunchConfiguration("kinematics_params_file").perform(context)
    if not os.path.isfile(kinematics_path) or not os.access(kinematics_path, os.R_OK):
        raise RuntimeError("Measured UR3e calibration is missing or unreadable: " + kinematics_path)

    description_package = "ur_description"
    moveit_package = "ur_moveit_config"
    joint_limits = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    physical = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "ur.urdf.xacro"]
            ),
            " robot_ip:=offline.invalid",
            " joint_limit_params:=",
            joint_limits,
            " kinematics_params:=",
            shlex.quote(kinematics_path),
            " physical_params:=",
            physical,
            " visual_params:=",
            visual,
            " safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20",
            " name:=ur ur_type:=",
            ur_type,
            " script_filename:=ros_control.urscript",
            " input_recipe_filename:=rtde_input_recipe.txt",
            " output_recipe_filename:=rtde_output_recipe.txt prefix:=''",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "srdf", "ur.srdf.xacro"]
            ),
            " name:=ur prefix:=''",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(semantic_content, value_type=str)
    }
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": load_yaml(moveit_package, "config/joint_limits.yaml")
    }
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur3e_print_planner"), "config", "offline_print_preview.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="offline_preview_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="offline_print_preview_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": False},
        ],
    )
    workcell = ExecuteProcess(
        cmd=[
            "python3",
            "-u",
            PathJoinSubstitution([workspace_root, "scripts", "publish_workcell_preview.py"]),
        ],
        output="screen",
    )
    playback = ExecuteProcess(
        cmd=[
            "python3",
            "-u",
            PathJoinSubstitution(
                [workspace_root, "scripts", "rviz_offline_print_playback.py"]
            ),
            "--manifest",
            manifest,
            "--workspace-root",
            workspace_root,
            "--project-root",
            project_root,
            "--speed-factor",
            speed_factor,
        ],
        output="screen",
    )
    # The playback waits for RViz as an additional guard.  The short timer
    # avoids wasting most of that wait while RViz is still loading plugins.
    return [robot_state_publisher, workcell, rviz, TimerAction(period=2.0, actions=[playback])]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("workspace_root", default_value="/home/ubuntu/ur_ros2_ws"),
            DeclareLaunchArgument(
                "project_root", default_value="/home/ubuntu/offline_preview_project"
            ),
            DeclareLaunchArgument("manifest", default_value="latest"),
            DeclareLaunchArgument("speed_factor", default_value="10"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value="/home/ubuntu/ur_ros2_ws/config/ur3e_actual_calibration.yaml",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
