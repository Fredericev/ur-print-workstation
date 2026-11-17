"""Calibration launch regression checks; no Docker/ROS process or robot is started."""

from __future__ import annotations

import ast
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from unittest import mock


PROJECT = Path(__file__).resolve().parents[2]
LAUNCH = PROJECT / "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/launch/ur_moveit.launch.py"
SPEC = importlib.util.spec_from_file_location(
    "windows_calibration_runtime", PROJECT / "Software/windows_runtime.py"
)
assert SPEC and SPEC.loader
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


class MoveItCalibrationTests(unittest.TestCase):
    def test_launch_selects_explicit_file_or_preserves_upstream_default(self) -> None:
        # Evaluate only the calibration selection block, without importing ROS launch
        # packages or constructing any Nodes/launch actions.
        tree = ast.parse(LAUNCH.read_text())
        setup = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name == "launch_setup")
        assignment_index = next(
            index for index, node in enumerate(setup.body)
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "kinematics_params_file"
                    for target in node.targets)
        )
        selection = ast.Module(body=setup.body[assignment_index:assignment_index + 2], type_ignores=[])
        program = compile(selection, str(LAUNCH), "exec")

        def select(path: str) -> object:
            configuration = mock.Mock()
            configuration.perform.return_value = path
            namespace = {
                "os": os, "context": object(), "description_package": "ur_description",
                "ur_type": "ur3e", "LaunchConfiguration": lambda name: configuration,
                "FindPackageShare": lambda value: value,
                "PathJoinSubstitution": lambda values: "/".join(values),
            }
            exec(program, namespace)
            return namespace["kinematics_params"]

        self.assertEqual(select(""), "ur_description/config/ur3e/default_kinematics.yaml")
        with tempfile.TemporaryDirectory() as directory:
            calibration = Path(directory) / "measured calibration.yaml"
            calibration.write_text("kinematics: {}\n")
            self.assertEqual(select(str(calibration)), str(calibration))
            with mock.patch.object(os, "access", return_value=False):
                with self.assertRaisesRegex(RuntimeError, "missing or unreadable"):
                    select(str(calibration))
            with self.assertRaisesRegex(RuntimeError, "missing or unreadable"):
                select(str(Path(directory) / "missing.yaml"))
            with self.assertRaisesRegex(RuntimeError, "missing or unreadable"):
                select(directory)

        declared = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                        and node.name == "generate_launch_description")
        arguments = [node for node in ast.walk(declared) if isinstance(node, ast.Call)
                     and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument"]
        calibration_argument = next(node for node in arguments if node.args
                                    and isinstance(node.args[0], ast.Constant)
                                    and node.args[0].value == "kinematics_params_file")
        default = next(keyword.value for keyword in calibration_argument.keywords
                       if keyword.arg == "default_value")
        self.assertEqual(ast.literal_eval(default), "")

    def test_windows_driver_and_moveit_use_the_same_override(self) -> None:
        for environment, expected in (
            ({}, "/home/ubuntu/ur_ros2_ws/config/ur3e_actual_calibration.yaml"),
            ({"KINEMATICS_PARAMS_FILE": "/legacy.yaml"}, "/legacy.yaml"),
            ({"KINEMATICS_PARAMS_FILE": "/legacy.yaml",
              "UR_KINEMATICS_PARAMS_FILE": "/measured calibration.yaml"}, "/measured calibration.yaml"),
        ):
            with self.subTest(environment=environment), mock.patch.dict(os.environ, environment, clear=True):
                self.assertEqual(runtime.robot_kinematics_params_file(), expected)
                station = runtime.WindowsRuntime(PROJECT, mock.Mock())
                with mock.patch.object(station, "_container_bash") as container:
                    station._launch_driver(robot_ip="192.168.0.10", ros_pc_ip="192.168.0.11",
                                           update_rate=125, non_blocking_read=True)
                    driver_path = container.call_args.kwargs["env"]["KINEMATICS_PARAMS_FILE"]
                with mock.patch.object(station, "docker_preflight"), \
                     mock.patch.object(station, "_require_container"), \
                     mock.patch.object(station, "wait_external_control"), \
                     mock.patch.object(station, "_container_bash") as container:
                    station.start_moveit()
                    moveit_path = container.call_args.kwargs["env"]["KINEMATICS_PARAMS_FILE"]
                self.assertEqual(driver_path, expected)
                self.assertEqual(moveit_path, driver_path)

    def test_windows_missing_calibration_fails_before_any_launch(self) -> None:
        # source is stubbed, and a nonexistent file makes the real startup body
        # return before pgrep, nohup, kill, or any ROS command can be reached.
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                ["bash", "-c", "source() { return 0; }\n" + runtime.MOVEIT_START_SCRIPT],
                env={**os.environ, "KINEMATICS_PARAMS_FILE": str(Path(directory) / "missing.yaml")},
                text=True, capture_output=True, timeout=3,
            )
        self.assertEqual(result.returncode, 66, result.stdout + result.stderr)
        self.assertIn("MoveIt calibration is missing or unreadable", result.stdout)

    def test_mac_entry_points_share_override_and_guard_precedes_process_launch(self) -> None:
        for filename in ("launch_moveit_detached.sh", "launch_real_robot_detached.sh",
                         "launch_real_robot.sh", "launch_real_robot_stable.sh"):
            path = PROJECT / "scripts" / filename
            text = path.read_text()
            with self.subTest(filename=filename):
                self.assertIn("${UR_KINEMATICS_PARAMS_FILE:-${KINEMATICS_PARAMS_FILE:-", text)
                result = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
        launch = (PROJECT / "scripts/launch_moveit_detached.sh").read_text()
        self.assertIn('-e KINEMATICS_PARAMS_FILE="$KINEMATICS_PARAMS_FILE"', launch)
        self.assertIn('kinematics_params_file:=', launch)
        self.assertLess(launch.index("MoveIt calibration is missing or unreadable"),
                        launch.index("pgrep -x move_group"))
        self.assertLess(launch.index("exit 66"), launch.index("nohup "))
        self.assertIn('kinematics_params_file:="$KINEMATICS_PARAMS_FILE"', runtime.MOVEIT_START_SCRIPT)

    def test_ros_setup_temporarily_disables_nounset_and_restores_it(self) -> None:
        # Humble setup.bash expands optional AMENT_TRACE_SETUP_FILES directly.
        # Simulate that expansion without sourcing ROS or launching a process.
        source_stub = 'source() { : "$AMENT_TRACE_SETUP_FILES"; }\nunset AMENT_TRACE_SETUP_FILES\n'
        scripts = {
            name: getattr(runtime, name)
            for name in ("DRIVER_LAUNCH_SCRIPT", "BRIDGE_LAUNCH_SCRIPT",
                         "EXTERNAL_CONTROL_SCRIPT", "MOVEIT_START_SCRIPT",
                         "PATH_VALIDATION_SCRIPT", "COORDINATOR_SCRIPT")
        }
        mac_launch = (PROJECT / "scripts/launch_moveit_detached.sh").read_text()
        scripts["mac_moveit"] = mac_launch[mac_launch.index("  set -euo pipefail"):]
        for name, script in scripts.items():
            with self.subTest(script=name):
                last_source = script.index("source /home/ubuntu/ur_ros2_ws/install/setup.bash")
                restore = script.index("set -u", last_source)
                setup_only = script[:restore + len("set -u")]
                result = subprocess.run(
                    ["bash", "-c", source_stub + setup_only
                     + '\ncase "$-" in *u*) exit 0;; *) exit 84;; esac\n'],
                    capture_output=True, text=True, timeout=3,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_stop_moveit_patterns_match_launch_but_not_the_signalling_shell(self) -> None:
        stop = (PROJECT / "scripts/stop_moveit_detached.sh").read_text()
        verified_call = re.search(
            r"verified_pid_signal\.sh\s+\\\n\s*'\$PID_FILE' '([^']+)' TERM", stop
        )
        self.assertIsNotNone(verified_call)
        pattern = verified_call.group(1)
        self.assertEqual(pattern, "[r]os2 launch ur_moveit_config")
        self.assertRegex("/usr/bin/python3 /opt/ros/humble/bin/ros2 launch ur_moveit_config ur_moveit.launch.py", pattern)
        # pkill -f sees the complete enclosing bash -lc program. Any literal
        # unbracketed expected-pattern argument would kill that shell itself.
        self.assertIsNone(re.search(pattern, stop))
        self.assertNotIn("pkill -TERM -x ur_ros2_control_node", stop)
        self.assertNotIn("pkill -KILL -x ur_ros2_control_node", stop)

    def test_build_paths_deploy_modified_moveit_launch_package(self) -> None:
        self.assertIn("ur_moveit_config", (PROJECT / "scripts/build_print_planner.sh").read_text())
        self.assertIn("src/Universal_Robots_ROS2_Driver/ur_moveit_config",
                      (PROJECT / "workspaces/ur_ros2_ws/scripts/verify_controller_overlay.py").read_text())
        source = (PROJECT / "Software/windows_runtime.py").read_text()
        tree = ast.parse(source)
        build = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                     and node.name == "_build_ros_sources")
        self.assertIn("ur_moveit_config", ast.get_source_segment(source, build))


if __name__ == "__main__":
    unittest.main()
