"""Launch the isolated UR3e + printhead RViz placement preview.

This launch intentionally contains only joint_state_publisher_gui,
robot_state_publisher, and RViz.  It has no driver, ros2_control, controller,
MoveIt execution node, URScript client, or Klipper node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    clocking = LaunchConfiguration("clocking")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    printhead_geometry_file = LaunchConfiguration("printhead_geometry_file")

    description_package = FindPackageShare("ur3e_printhead_description")
    ur_description_package = FindPackageShare("ur_description")
    xacro_file = PathJoinSubstitution(
        [description_package, "urdf", "ur3e_with_printhead.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ur_type:=",
            ur_type,
            " clocking:=",
            clocking,
            " kinematics_params:=",
            kinematics_params_file,
            " printhead_geometry:=",
            printhead_geometry_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3e"]),
            DeclareLaunchArgument(
                "clocking",
                default_value="0",
                choices=["0", "90", "180", "270"],
                description="Printhead rotation around the tool0 Z axis in degrees.",
            ),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [ur_description_package, "config", ur_type, "default_kinematics.yaml"]
                ),
                description="UR kinematics YAML; the start script supplies measured UR3e calibration.",
            ),
            DeclareLaunchArgument(
                "printhead_geometry_file",
                default_value=PathJoinSubstitution(
                    [description_package, "config", "printhead_geometry.json"]
                ),
                description="Shared CAD candidate profile; does not confirm an installed TCP or selected head.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="printhead_preview_robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": False}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="printhead_preview_joint_state_publisher_gui",
                output="screen",
                # Only the offline RViz initial joint state. The calibrated
                # tool0 +Z axis points down in this pose, so the mounted
                # nozzle is vertical instead of the all-zero horizontal pose.
                parameters=[robot_description, {
                    "use_sim_time": False,
                    "zeros.shoulder_pan_joint": 0.0,
                    "zeros.shoulder_lift_joint": -1.35,
                    "zeros.elbow_joint": 1.55,
                    "zeros.wrist_1_joint": -1.75,
                    "zeros.wrist_2_joint": -1.57,
                    "zeros.wrist_3_joint": 0.0,
                }],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="printhead_preview_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([description_package, "rviz", "view_printhead.rviz"]),
                ],
                parameters=[robot_description, {"use_sim_time": False}],
            ),
        ]
    )
