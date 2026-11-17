from __future__ import annotations

import importlib.util
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "windows_runtime.py"
SPEC = importlib.util.spec_from_file_location("windows_runtime", MODULE_PATH)
assert SPEC and SPEC.loader
runtime = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runtime
SPEC.loader.exec_module(runtime)


def completed(
    argv: list[str] | None = None,
    *,
    code: int = 0,
    output: str = "",
) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(argv or ["docker"], code, output, "")


class RuntimeFixture(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "docker-compose.real.yml").write_text(
            "services:\n  ros2:\n    image: test\n",
            encoding="utf-8",
        )
        workspace = self.root / "workspaces" / "ur_ros2_ws"
        for relative in (
            "src/ur_klipper_interfaces",
            "src/ur_klipper_bridge",
            "src/ur3e_print_planner",
            "src/Universal_Robots_Client_Library",
            "src/Universal_Robots_ROS2_Driver/ur_robot_driver",
            "config",
            "scripts",
            "runtime/path_review",
        ):
            (workspace / relative).mkdir(parents=True, exist_ok=True)
        (workspace / "config" / "ur3e_actual_calibration.yaml").write_text(
            "kinematics:\n",
            encoding="utf-8",
        )
        (workspace / "scripts" / "external_control_readiness_probe.py").write_text(
            "# probe\n",
            encoding="utf-8",
        )
        (workspace / "scripts" / "klipper_bridge_readiness_probe.py").write_text(
            "# probe\n",
            encoding="utf-8",
        )
        (workspace / "src/Universal_Robots_ROS2_Driver/ur_controllers").mkdir(parents=True)
        (workspace / "scripts/verify_controller_overlay.py").write_text("# offline build verifier fixture\n")
        (workspace / "scripts/smooth_trajectory.py").write_text("# shared trajectory interpolation fixture\n")
        (workspace / "scripts/supervise_with_rotating_log.py").write_text("# supervisor fixture\n")
        (workspace / "scripts/verified_pid_signal.sh").write_text("# PID guard fixture\n")
        (workspace / "config/printhead_geometry.json").write_text('{}\n')
        (self.root / "scripts").mkdir()
        (self.root / "scripts/check_ros_container_network.py").write_text("# network fixture\n")
        self.runner = mock.Mock(spec=runtime.CommandRunner)
        self.station = runtime.WindowsRuntime(self.root, self.runner)


class ValidationTests(RuntimeFixture):
    def test_validates_addresses_url_identifiers_and_angles(self) -> None:
        self.assertEqual(self.station._ipv4("Robot IP", "192.168.0.104"), "192.168.0.104")
        self.assertEqual(
            self.station._moonraker_url("http://192.168.0.110/"),
            "http://192.168.0.110",
        )
        self.assertEqual(self.station._identifier("stepper", "extruder"), "extruder")
        self.assertEqual(self.station._finite_angle("roll", "-180.5"), -180.5)

        invalid_calls = (
            lambda: self.station._ipv4("Robot IP", "127.0.0.1"),
            lambda: self.station._ipv4("Robot IP", "bad"),
            lambda: self.station._moonraker_url("http://user:pw@192.168.0.110"),
            lambda: self.station._moonraker_url("http://host/path?q=1"),
            lambda: self.station._identifier("stepper", "extruder; reboot"),
            lambda: self.station._finite_angle("roll", "nan"),
            lambda: self.station._finite_angle("roll", "361"),
        )
        for call in invalid_calls:
            with self.subTest(call=call), self.assertRaises(runtime.RuntimeFailure):
                call()

    def test_validate_project_fails_closed_for_incomplete_package(self) -> None:
        (self.root / "workspaces" / "ur_ros2_ws" / "config" / "ur3e_actual_calibration.yaml").unlink()
        with self.assertRaises(runtime.RuntimeFailure) as raised:
            self.station._validate_project()
        self.assertEqual(raised.exception.exit_code, 66)
        self.runner.run.assert_not_called()

    def test_validate_project_requires_the_shared_smooth_trajectory_module(self) -> None:
        (self.root / "workspaces/ur_ros2_ws/scripts/smooth_trajectory.py").unlink()
        with self.assertRaises(runtime.RuntimeFailure) as raised:
            self.station._validate_project()
        self.assertIn("smooth_trajectory.py", raised.exception.message)
        self.assertEqual(raised.exception.exit_code, 66)
        self.runner.run.assert_not_called()

    def test_environment_rejects_newlines_and_nul(self) -> None:
        for value in ("bad\nvalue", "bad\x00value", ""):
            with self.subTest(value=value), self.assertRaises(runtime.RuntimeFailure):
                self.station._safe_env_value("TEST", value)


class DockerCommandTests(RuntimeFixture):
    def test_docker_preflight_requires_linux_and_compose_service(self) -> None:
        self.runner.run.side_effect = [
            completed(output="linux|amd64|28.0\n"),
            completed(output="2.39.0\n"),
            completed(output="ros2\n"),
        ]
        self.station.docker_preflight()
        commands = [call.args[0] for call in self.runner.run.call_args_list]
        self.assertEqual(commands[0][:2], ["docker", "version"])
        self.assertEqual(commands[1][:3], ["docker", "compose", "version"])
        self.assertEqual(commands[2][:4], ["docker", "compose", "-f", str(self.station.compose_file)])
        for call in self.runner.run.call_args_list:
            self.assertNotIn("shell", call.kwargs)

    def test_docker_preflight_rejects_windows_container_mode(self) -> None:
        self.runner.run.return_value = completed(output="windows|amd64|28.0\n")
        with self.assertRaises(runtime.RuntimeFailure) as raised:
            self.station.docker_preflight()
        self.assertIn("Linux-container mode", raised.exception.message)

    def test_container_bash_is_only_after_docker_compose_exec(self) -> None:
        self.runner.run.return_value = completed()
        self.station._container_bash(
            "echo ready",
            env={"SAFE_VALUE": "value with spaces"},
            label="test",
        )
        command = self.runner.run.call_args.args[0]
        self.assertEqual(command[:4], ["docker", "compose", "-f", str(self.station.compose_file)])
        self.assertIn("exec", command)
        self.assertEqual(command[-3:-1], ["bash", "-lc"])
        self.assertEqual(command[-1], "echo ready")
        self.assertNotEqual(command[0], "bash")

    def test_container_bash_validates_environment_names(self) -> None:
        with self.assertRaises(runtime.RuntimeFailure):
            self.station._container_bash(
                "true",
                env={"bad-key": "value"},
                label="test",
            )
        self.runner.run.assert_not_called()

    def test_write_update_rate_preserves_bind_mount_inode_and_syncs_overlay(self) -> None:
        overlay = (self.station.workspace / "src/Universal_Robots_ROS2_Driver"
                   / "ur_robot_driver/config/ur3e_update_rate.yaml")
        overlay.parent.mkdir(parents=True)
        overlay.write_text("old\n")
        destination = self.station.workspace / "config/ur3e_update_rate.yaml"
        destination.write_text("old\n")
        inode = destination.stat().st_ino
        result = self.station._write_update_rate(125)
        self.assertEqual(result.stat().st_ino, inode)
        self.assertEqual(overlay.read_bytes(), result.read_bytes())
        self.assertEqual(
            result.read_bytes(),
            (
                b"controller_manager:\n"
                b"  ros__parameters:\n"
                b"    update_rate: 125 # Hz\n"
            ),
        )
        self.assertEqual(list(result.parent.glob("*.tmp")), [])

    def test_ros_build_avoids_symlinks_on_windows_bind_mounts(self) -> None:
        self.runner.run.return_value = completed()
        self.station._build_ros_sources()
        command = self.runner.run.call_args.args[0]
        container_script = command[-1]
        self.assertIn("colcon build --packages-select", container_script)
        self.assertNotIn("--symlink-install", container_script)
        self.assertIn("ur_controllers", container_script)
        self.assertIn("ur_client_library", container_script)
        self.assertIn("ur_robot_driver", container_script)
        self.assertIn("verify_controller_overlay.py", container_script)

    def test_ros_build_selects_overlay_urcl_before_building_the_driver(self) -> None:
        self.runner.run.return_value = completed()
        self.station._build_ros_sources()
        container_script = self.runner.run.call_args.args[0][-1]
        self.assertEqual(container_script.count("colcon build --packages-select"), 2)
        first_build, remaining_build = container_script.split(
            "colcon build --packages-select", 2
        )[1:]
        self.assertIn("ur_client_library", first_build)
        self.assertNotIn("ur_controllers", first_build)
        self.assertIn("source install/setup.bash", first_build)
        self.assertIn("ur_controllers", remaining_build)
        self.assertIn("ur_robot_driver", remaining_build)
        self.assertIn("ur3e_printhead_description", remaining_build)
        self.assertIn(
            "-Dur_client_library_DIR:PATH=/home/ubuntu/ur_ros2_ws/install/"
            "ur_client_library/lib/cmake/ur_client_library",
            remaining_build,
        )
        self.assertEqual(container_script.count("--executor sequential --parallel-workers 1"), 2)
        self.assertIn("export CMAKE_BUILD_PARALLEL_LEVEL=1", container_script)
        self.assertIn("export MAKEFLAGS=-j1", container_script)


class WindowsPackageBuildCompatibilityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.builder = (MODULE_PATH.parent / "build-windows-package.ps1").read_text(
            encoding="utf-8"
        )

    def test_powershell_builder_uses_windows_powershell_compatible_path_helper(self) -> None:
        self.assertIn("#requires -Version 5.1", self.builder)
        self.assertIn("function Get-PayloadRelativePath", self.builder)
        self.assertNotIn("[IO.Path]::GetRelativePath", self.builder)
        self.assertIn("$FullPath.StartsWith($RootPrefix", self.builder)
        self.assertIn("Get-PayloadRelativePath $Root", self.builder)

    def test_powershell_builder_always_verifies_the_pinned_local_sdk(self) -> None:
        self.assertIn('function Test-PinnedDotnetSdk', self.builder)
        self.assertIn('$ReportedVersion -eq $DotnetVersion', self.builder)
        self.assertNotIn('Get-Command dotnet', self.builder)
        self.assertIn('$env:DOTNET_ROOT = $DotnetRoot', self.builder)
        self.assertIn('$env:DOTNET_MULTILEVEL_LOOKUP = "0"', self.builder)
        self.assertIn('throw "Pinned .NET SDK $DotnetVersion is unavailable', self.builder)
        self.assertIn('throw "dotnet publish failed for $RuntimeIdentifier."', self.builder)


class NetworkGateTests(RuntimeFixture):
    def test_robot_route_must_match_explicit_external_control_ip(self) -> None:
        with mock.patch.object(
            self.station, "_route_source", return_value="192.168.0.107"
        ), mock.patch.object(self.station, "_require_robot_rtde") as rtde:
            with self.assertRaises(runtime.RuntimeFailure) as raised:
                self.station._verify_robot_route(
                    "192.168.0.104",
                    "192.168.0.108",
                )
        self.assertIn("does not match", raised.exception.message)
        rtde.assert_not_called()

    def test_robot_route_checks_rtde_after_address_match(self) -> None:
        with mock.patch.object(
            self.station, "_route_source", return_value="192.168.0.107"
        ), mock.patch.object(self.station, "_require_robot_rtde") as rtde:
            self.station._verify_robot_route(
                "192.168.0.104",
                "192.168.0.107",
            )
        rtde.assert_called_once_with("192.168.0.104")

    def test_reverse_ports_require_robot_interface_docker_mappings(self) -> None:
        self.runner.run.side_effect = [
            completed(output=f"192.168.0.107:{port}\n")
            for port in runtime.UR_REVERSE_PORTS
        ]
        self.station._verify_reverse_ports("192.168.0.107")
        self.assertEqual(self.runner.run.call_count, 4)

    def test_reverse_ports_reject_wildcard_mapping(self) -> None:
        self.runner.run.side_effect = [
            completed(output=f"0.0.0.0:{port}\n")
            for port in runtime.UR_REVERSE_PORTS
        ]
        with self.assertRaises(runtime.RuntimeFailure):
            self.station._verify_reverse_ports("192.168.0.107")

    def test_reverse_ports_reject_loopback_only_mapping(self) -> None:
        self.runner.run.return_value = completed(output="127.0.0.1:50001\n")
        with self.assertRaises(runtime.RuntimeFailure) as raised:
            self.station._verify_reverse_ports("192.168.0.107")
        self.assertEqual(raised.exception.exit_code, 80)
        self.assertIn("Do not press Play", raised.exception.message)


class WorkflowTests(RuntimeFixture):
    def test_start_ros_orders_every_fail_closed_stage(self) -> None:
        order: list[str] = []
        methods = (
            "_validate_project",
            "_verify_robot_route",
            "docker_preflight",
            "_write_update_rate",
            "_start_container",
            "_network_preflight",
            "_apply_update_rate",
            "_apply_runtime_patches",
            "_build_ros_sources",
            "_cleanup_driver",
            "_launch_driver",
            "_launch_bridge",
            "_wait_driver_ready",
            "_verify_reverse_ports",
        )
        patches = []
        for method in methods:
            patcher = mock.patch.object(
                self.station,
                method,
                side_effect=lambda *args, _method=method, **kwargs: order.append(_method),
            )
            patches.append(patcher)
            patcher.start()
            self.addCleanup(patcher.stop)

        self.station.start_ros(
            robot_ip="192.168.0.104",
            ros_pc_ip="192.168.0.107",
            moonraker_url="http://192.168.0.110",
            stepper="extruder",
            update_rate=125,
            startup_timeout=60,
            stability_seconds=5,
        )
        self.assertEqual(order, list(methods))

    def test_start_ros_rejects_bad_input_before_any_operation(self) -> None:
        with self.assertRaises(runtime.RuntimeFailure):
            self.station.start_ros(
                robot_ip="192.168.0.104; shutdown",
                ros_pc_ip="192.168.0.107",
                moonraker_url="http://192.168.0.110",
                stepper="extruder",
            )
        self.runner.run.assert_not_called()

    def test_start_ros_derives_blank_ros_pc_ip_from_windows_route(self) -> None:
        patch_names = (
            "_validate_project",
            "docker_preflight",
            "_write_update_rate",
            "_start_container",
            "_network_preflight",
            "_apply_update_rate",
            "_apply_runtime_patches",
            "_build_ros_sources",
            "_cleanup_driver",
            "_launch_bridge",
            "_wait_driver_ready",
            "_verify_reverse_ports",
        )
        patchers = [
            mock.patch.object(self.station, name)
            for name in patch_names
        ]
        for patcher in patchers:
            patcher.start()
            self.addCleanup(patcher.stop)
        with mock.patch.object(
            self.station,
            "_route_source",
            return_value="192.168.0.107",
        ) as route, mock.patch.object(
            self.station,
            "_verify_robot_route",
        ) as verify, mock.patch.object(
            self.station,
            "_launch_driver",
        ) as launch:
            self.station.start_ros(
                robot_ip="192.168.0.104",
                ros_pc_ip="",
                moonraker_url="http://192.168.0.110",
                stepper="extruder",
                stability_seconds=5,
            )
        route.assert_called_once_with("192.168.0.104")
        verify.assert_called_once_with("192.168.0.104", "192.168.0.107")
        self.assertEqual(launch.call_args.kwargs["ros_pc_ip"], "192.168.0.107")

    def test_wait_external_control_uses_container_probe(self) -> None:
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_require_container"
        ), mock.patch.object(self.station, "_container_bash") as execute:
            self.station.wait_external_control(timeout=12)
        execute.assert_called_once()
        self.assertEqual(execute.call_args.kwargs["env"], {"TIMEOUT_SECONDS": 12})
        self.assertIn("external_control_readiness_probe.py", execute.call_args.args[0])

    def test_stop_ros_is_idempotent_only_after_verified_stop(self) -> None:
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_compose"
        ) as compose, mock.patch.object(
            self.station, "_container_running", return_value=False
        ):
            self.station.stop_ros()
        compose.assert_called_once_with(
            "stop",
            runtime.ROS_SERVICE,
            timeout=90,
            label="ROS container stop",
        )

    def test_stop_ros_fails_if_container_remains_running(self) -> None:
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_compose"
        ), mock.patch.object(
            self.station, "_container_running", return_value=True
        ), self.assertRaises(runtime.RuntimeFailure):
            self.station.stop_ros()


class PathValidationTests(RuntimeFixture):
    def setUp(self) -> None:
        super().setUp()
        self.review_id = "a1b2c3d4e5f6"
        self.review_sha = "c" * 64
        self.input_path = self.station.path_review_dir / f"{self.review_id}.csv"
        self.input_path.write_text(
            "sequence,x_m,y_m,z_m,source_line,extruding\n"
            "0,0.1,0.2,0.3,1,1\n",
            encoding="utf-8",
        )

    def test_validate_path_passes_only_validated_values_into_container(self) -> None:
        output_path = self.station.path_review_dir / f"{self.review_id}.json"

        def write_result(*args, **kwargs):
            output_path.write_text('{"status":"passed"}\n', encoding="utf-8")
            return completed()

        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_require_container"
        ), mock.patch.object(
            self.station, "_container_bash", side_effect=write_result
        ) as execute:
            result = self.station.validate_path(
                review_id=self.review_id,
                review_sha256=self.review_sha,
                printhead_geometry_sha256="e" * 64,
                source_waypoint_count=2,
                source_sampled="false",
                tool_roll_deg=180,
                tool_pitch_deg=0,
                tool_yaw_deg=-10,
            )
        self.assertEqual(result, output_path)
        environment = execute.call_args.kwargs["env"]
        self.assertEqual(environment["REVIEW_SHA256"], self.review_sha)
        self.assertEqual(environment["PRINTHEAD_GEOMETRY_SHA256"], "e" * 64)
        self.assertEqual(environment["SOURCE_WAYPOINT_COUNT"], 2)
        self.assertEqual(environment["SOURCE_SAMPLED"], "false")
        self.assertEqual(
            environment["REVIEW_PATH_FILE"],
            f"/home/ubuntu/ur_ros2_ws/runtime/path_review/{self.review_id}.csv",
        )

    def test_validate_path_rejects_retired_tabletop_mode(self) -> None:
        with self.assertRaises(runtime.RuntimeFailure):
            self.station.validate_path(
                review_id=self.review_id,
                review_sha256=self.review_sha,
                printhead_geometry_sha256="e" * 64,
                source_waypoint_count=2,
                source_sampled="false",
                tool_roll_deg=180,
                tool_pitch_deg=0,
                tool_yaw_deg=0,
                tcp_offset_x_m=0,
                tcp_offset_y_m=0,
                tcp_offset_z_m=0,
                tool_collision_radius_m=0,
                operation_mode="tool0_tabletop_test",
            )

    def test_validate_path_removes_stale_result_and_requires_new_output(self) -> None:
        output_path = self.station.path_review_dir / f"{self.review_id}.json"
        output_path.write_text('{"status":"stale"}\n', encoding="utf-8")
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_require_container"
        ), mock.patch.object(
            self.station, "_container_bash", return_value=completed()
        ), self.assertRaises(runtime.RuntimeFailure) as raised:
            self.station.validate_path(
                review_id=self.review_id,
                review_sha256=self.review_sha,
                printhead_geometry_sha256="e" * 64,
                source_waypoint_count=2,
                source_sampled="false",
                tool_roll_deg=180,
                tool_pitch_deg=0,
                tool_yaw_deg=0,
            )
        self.assertFalse(output_path.exists())
        self.assertEqual(raised.exception.exit_code, 74)

    def test_validate_path_rejects_every_untrusted_field_before_docker(self) -> None:
        cases = [
            {"review_id": "../bad"},
            {"review_sha256": "bad"},
            {"source_waypoint_count": runtime.MAX_MOVEIT_WAYPOINTS + 1},
            {"source_sampled": "yes"},
            {"tool_roll_deg": "nan"},
        ]
        base = {
            "review_id": self.review_id,
            "review_sha256": self.review_sha,
            "printhead_geometry_sha256": "e" * 64,
            "source_waypoint_count": 2,
            "source_sampled": "false",
            "tool_roll_deg": 180,
            "tool_pitch_deg": 0,
            "tool_yaw_deg": 0,
        }
        for update in cases:
            with self.subTest(update=update), self.assertRaises(runtime.RuntimeFailure):
                self.station.validate_path(**{**base, **update})
        self.runner.run.assert_not_called()


class CoordinatorTests(RuntimeFixture):
    def base_arguments(self, **updates):
        values = {
            "execute": False,
            "axes": "x,y,z",
            "distance_m": 0.02,
            "max_joint_speed_rad_s": 0.03,
            "klipper_stepper": "extruder",
            "klipper_target_position": 60.0,
            "klipper_speed": 5.0,
            "klipper_acceleration": 20.0,
        }
        values.update(updates)
        return values

    def test_plan_only_coordinator_maps_typed_ros_parameters(self) -> None:
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station,
            "_require_container",
        ), mock.patch.object(
            self.station,
            "_container_bash",
            return_value=completed(),
        ) as execute:
            self.station.run_coordinator(**self.base_arguments())

        environment = execute.call_args.kwargs["env"]
        self.assertEqual(environment["EXECUTE"], "false")
        self.assertEqual(environment["DISTANCE_M"], "0.02")
        self.assertEqual(environment["KLIPPER_TARGET_POSITION"], "60.0")
        self.assertEqual(environment["KLIPPER_SPEED"], "5.0")
        self.assertEqual(environment["KLIPPER_ACCELERATION"], "20.0")
        self.assertEqual(environment["SYNCHRONIZE_KLIPPER"], "true")
        self.assertNotIn("CONFIRM_REAL_ROBOT_MOVE", environment)
        self.assertNotIn("CONFIRM_CARTESIAN_MOVE", environment)
        self.assertNotIn("CONFIRM_KLIPPER_STEPPER", environment)
        self.assertEqual(execute.call_args.kwargs["timeout"], 600)
        self.assertIn(
            "ros2 run ur3e_print_planner cartesian_xyz_moveit",
            execute.call_args.args[0],
        )

    def test_execute_coordinator_requires_and_forwards_exact_confirmations(self) -> None:
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station,
            "_require_container",
        ), mock.patch.object(
            self.station,
            "_container_bash",
            return_value=completed(),
        ) as execute:
            self.station.run_coordinator(
                **self.base_arguments(
                    execute=True,
                    klipper_reset_position=True,
                    return_to_start=True,
                    confirm_real_robot_move="YES",
                    confirm_cartesian_move="YES",
                    confirm_klipper_stepper="YES",
                )
            )
        environment = execute.call_args.kwargs["env"]
        self.assertEqual(environment["EXECUTE"], "true")
        self.assertEqual(environment["RETURN_TO_START"], "true")
        self.assertEqual(environment["KLIPPER_RESET_POSITION"], "true")
        self.assertEqual(environment["CONFIRM_REAL_ROBOT_MOVE"], "YES")
        self.assertEqual(environment["CONFIRM_CARTESIAN_MOVE"], "YES")
        self.assertEqual(environment["CONFIRM_KLIPPER_STEPPER"], "YES")
        self.assertEqual(execute.call_args.kwargs["timeout"], 1800)

    def test_coordinator_rejects_unsafe_values_before_docker(self) -> None:
        cases = [
            {"execute": "maybe"},
            {"execute": True},
            {"distance_m": float("nan")},
            {"distance_m": 0.11},
            {"klipper_stepper": "extruder; reboot"},
            {"return_to_start": True},
            {"synchronize_klipper": False},
            {
                "execute": True,
                "confirm_real_robot_move": "YES",
                "confirm_cartesian_move": "YES",
                "confirm_klipper_stepper": "yes",
            },
            {
                "execute": True,
                "confirm_real_robot_move": "YES",
                "confirm_cartesian_move": "YES",
                "confirm_klipper_stepper": "YES",
                "execution_controller": "scaled_joint_trajectory_controller",
            },
            {
                "execute": False,
                "confirm_real_robot_move": "YES",
            },
        ]
        # The bare execute=True case is intentionally invalid because no three
        # exact YES confirmations accompany it.
        for update in cases:
            with self.subTest(update=update), self.assertRaises(runtime.RuntimeFailure):
                self.station.run_coordinator(
                    **self.base_arguments(**update)
                )
        self.runner.run.assert_not_called()


class CartesianTrialTests(RuntimeFixture):
    def trial(self, **updates):
        env = {
            "AXES": "z", "DISTANCE_M": "-0.02", "EXECUTE": "false",
            "SYNCHRONIZE_KLIPPER": "false", "HEIGHT_TRIAL_MODE": "true",
            "HEIGHT_TRIAL_SURFACE_Z_M": "0.3", "HEIGHT_TRIAL_MIN_X_M": "-0.3",
            "HEIGHT_TRIAL_MAX_X_M": "0.3", "HEIGHT_TRIAL_MIN_Y_M": "-0.2",
            "HEIGHT_TRIAL_MAX_Y_M": "0.2", "HEIGHT_TRIAL_TCP_X_M": "0.0",
            "HEIGHT_TRIAL_TCP_Y_M": "0.0", "HEIGHT_TRIAL_TCP_Z_M": "0.12",
        }
        env.update(updates)
        return env

    def test_height_trial_preserves_ur_only_geometry_and_measured_tcp(self):
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_require_container"
        ), mock.patch.object(self.station, "_container_bash") as execute:
            self.station.run_cartesian(self.trial())
        env = execute.call_args.kwargs["env"]
        self.assertEqual(env["SYNCHRONIZE_KLIPPER"], "false")
        self.assertEqual(env["HEIGHT_TRIAL_MODE"], "true")
        self.assertEqual(env["HEIGHT_TRIAL_TCP_Z_M"], "0.12")
        self.assertEqual(env["HEIGHT_TRIAL_SURFACE_Z_M"], "0.3")
        self.assertEqual(env["PRINTHEAD_GEOMETRY_SHA256"], hashlib.sha256(b"{}").hexdigest())
        self.assertIn("height_trial_collision_model_v4_geometry_bound", execute.call_args.args[0])

    def test_execute_ur_only_trial_needs_two_confirmations_without_extrusion_confirmation(self):
        with mock.patch.object(self.station, "docker_preflight"), mock.patch.object(
            self.station, "_require_container"
        ), mock.patch.object(self.station, "_container_bash") as execute:
            self.station.run_cartesian(self.trial(
                EXECUTE="true", CONFIRM_REAL_ROBOT_MOVE="YES", CONFIRM_CARTESIAN_MOVE="YES",
            ))
        env = execute.call_args.kwargs["env"]
        self.assertEqual(env["EXECUTE"], "true")
        self.assertEqual(env["CONFIRM_CARTESIAN_MOVE"], "YES")
        self.assertNotIn("CONFIRM_KLIPPER_STEPPER", env)

    def test_height_trial_rejects_changed_geometry_bad_tcp_or_non_z_motion_before_docker(self):
        for update in (
            {"PRINTHEAD_GEOMETRY_SHA256": "a" * 64}, {"AXES": "x"},
            {"HEIGHT_TRIAL_TCP_Z_M": "nan"}, {"HEIGHT_TRIAL_TCP_Z_M": "0.001"},
            {"HEIGHT_TRIAL_MAX_X_M": "-0.4"}, {"SYNCHRONIZE_KLIPPER": "true"},
            {"EXECUTE": "true"},
        ):
            with self.subTest(update=update), self.assertRaises(runtime.RuntimeFailure):
                self.station.run_cartesian(self.trial(**update))
        self.runner.run.assert_not_called()


class StartupCleanupTests(RuntimeFixture):
    def test_driver_launch_failure_after_ownership_always_cleans_and_verifies(self):
        names = ("_validate_project", "_verify_robot_route", "docker_preflight", "_write_update_rate",
                 "_start_container", "_network_preflight", "_apply_update_rate", "_apply_runtime_patches",
                 "_build_ros_sources", "_wait_driver_ready", "_verify_reverse_ports")
        for name in names:
            patcher = mock.patch.object(self.station, name)
            patcher.start()
            self.addCleanup(patcher.stop)
        with mock.patch.object(self.station, "_launch_driver", side_effect=runtime.RuntimeFailure("partial launch")), \
             mock.patch.object(self.station, "_cleanup_driver") as cleanup, \
             mock.patch.object(self.station, "_container_bash") as verify:
            with self.assertRaises(runtime.RuntimeFailure):
                self.station.start_ros(robot_ip="192.168.0.104", ros_pc_ip="192.168.0.107",
                                       moonraker_url="http://192.168.0.168", stepper="extruder")
        self.assertEqual(cleanup.call_count, 2)
        self.assertEqual(verify.call_args.args[0], runtime.DRIVER_STOP_VERIFY_SCRIPT)
        self.assertTrue(self.station._cancel_checks_enabled)

    def test_existing_cancel_file_does_not_prevent_owned_stack_cleanup(self):
        marker = self.root / "cancel"
        marker.touch()
        with mock.patch.dict(runtime.os.environ, {"WORKSTATION_CANCEL_FILE": str(marker)}):
            with self.assertRaises(runtime.RuntimeFailure):
                self.station._run(["docker", "info"], timeout=2, label="check")
            self.station._cancel_checks_enabled = False
            self.station._run(["docker", "info"], timeout=2, label="cleanup")
        self.assertNotIn("WORKSTATION_CANCEL_FILE", self.runner.run.call_args.kwargs["env"])

    def test_command_runner_honors_cooperative_cancel_without_host_shell(self):
        marker = self.root / "cancel"
        marker.touch()
        with mock.patch.dict(runtime.os.environ, {"WORKSTATION_CANCEL_FILE": str(marker)}):
            with self.assertRaises(runtime.RuntimeFailure) as raised:
                runtime.CommandRunner().run(
                    [sys.executable, "-c", "import time; time.sleep(20)"],
                    cwd=self.root, timeout=3, label="offline canceled child",
                )
        self.assertEqual(raised.exception.exit_code, 125)


class CliTests(unittest.TestCase):
    def test_start_ros_uses_the_wired_raspberry_pi_by_default(self) -> None:
        parser = runtime.build_parser()

        args = parser.parse_args(["start-ros"])

        self.assertEqual(args.moonraker_url, "http://192.168.0.168")

    def test_parser_exposes_all_required_actions(self) -> None:
        parser = runtime.build_parser()
        actions = (
            ["start-ros"],
            ["wait-external-control"],
            ["start-moveit"],
            ["stop-moveit"],
            ["stop-driver"],
            ["stop-ros"],
            ["cartesian-move"],
            ["run-coordinator"],
            [
                "validate-path",
                "a1b2c3d4e5f6",
                "c" * 64,
                "2",
                "false",
                "180",
                "0",
                "0",
            ],
        )
        for argv in actions:
            with self.subTest(argv=argv):
                self.assertEqual(parser.parse_args(argv).action, argv[0])


if __name__ == "__main__":
    unittest.main()
