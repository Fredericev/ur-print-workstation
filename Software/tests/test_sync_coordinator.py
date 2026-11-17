from __future__ import annotations

import argparse
import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[2] / "scripts" / "ur_klipper_sync_test.py"
SPEC = importlib.util.spec_from_file_location("ur_klipper_sync_test", MODULE_PATH)
assert SPEC and SPEC.loader
coordinator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = coordinator
SPEC.loader.exec_module(coordinator)


def args_for(**updates) -> argparse.Namespace:
    values = {
        "stepper": "extruder",
        "axes": "x,y,z",
        "distance_m": 0.02,
        "allow_15cm": False,
        "stepper_move": 60.0,
        "stepper_speed": 5.0,
        "stepper_accel": 20.0,
        "max_joint_speed_rad_s": 0.03,
        "execute": False,
        "confirm_real_robot_move": "",
        "confirm_cartesian_move": "",
        "confirm_klipper_stepper": "",
        "return_to_start": False,
        "reset_stepper_position": False,
        "disable_stepper_after": False,
        "skip_ur_ready_check": False,
        "check_only": False,
    }
    values.update(updates)
    return argparse.Namespace(**values)


class ValidationTests(unittest.TestCase):
    def test_real_return_requires_zero_reference(self) -> None:
        args = args_for(
            execute=True,
            confirm_real_robot_move="YES",
            confirm_cartesian_move="YES",
            confirm_klipper_stepper="YES",
            return_to_start=True,
        )
        with self.assertRaises(SystemExit) as raised:
            coordinator.validate_args(args)
        self.assertEqual(raised.exception.code, 64)

    def test_real_execution_requires_all_three_exact_confirmations(self) -> None:
        args = args_for(
            execute=True,
            confirm_real_robot_move="YES",
            confirm_cartesian_move="YES",
            confirm_klipper_stepper="yes",
        )
        with self.assertRaises(SystemExit) as raised:
            coordinator.validate_args(args)
        self.assertEqual(raised.exception.code, 64)


class RosEnvironmentTests(unittest.TestCase):
    def test_plan_environment_routes_every_klipper_setting_through_ros(self) -> None:
        args = args_for(
            check_only=True,
            stepper="stepper_aux",
            stepper_move=-12.5,
            stepper_speed=2.5,
            stepper_accel=10.0,
            reset_stepper_position=True,
            disable_stepper_after=True,
            return_to_start=True,
            axes="z,x",
            distance_m=-0.04,
            max_joint_speed_rad_s=0.025,
        )

        environment = coordinator.ros_environment(args, execute=False)

        expected = {
            "EXECUTE": "false",
            "AXES": "z,x",
            "DISTANCE_M": "-0.04",
            "MAX_JOINT_SPEED_RAD_S": "0.025",
            "SYNCHRONIZE_KLIPPER": "true",
            "KLIPPER_CHECK_ONLY": "true",
            "KLIPPER_STEPPER": "stepper_aux",
            "KLIPPER_TARGET_POSITION": "-12.5",
            "KLIPPER_SPEED": "2.5",
            "KLIPPER_ACCELERATION": "10.0",
            "KLIPPER_RESET_POSITION": "true",
            "KLIPPER_DISABLE_AFTER": "true",
            "RETURN_TO_START": "true",
            "CONFIRM_REAL_ROBOT_MOVE": "",
            "CONFIRM_CARTESIAN_MOVE": "",
            "CONFIRM_KLIPPER_STEPPER": "",
        }
        for key, value in expected.items():
            self.assertEqual(environment[key], value, key)

    def test_integer_looking_values_remain_ros_doubles(self) -> None:
        args = args_for(
            stepper_move=60.0,
            stepper_speed=5.0,
            stepper_accel=20.0,
        )
        environment = coordinator.ros_environment(args, execute=False)
        self.assertEqual(environment["KLIPPER_TARGET_POSITION"], "60.0")
        self.assertEqual(environment["KLIPPER_SPEED"], "5.0")
        self.assertEqual(environment["KLIPPER_ACCELERATION"], "20.0")

    def test_real_environment_passes_only_fixed_yes_confirmations(self) -> None:
        args = args_for(
            execute=True,
            confirm_real_robot_move="YES",
            confirm_cartesian_move="YES",
            confirm_klipper_stepper="YES",
        )

        environment = coordinator.ros_environment(args, execute=True)

        self.assertEqual(environment["EXECUTE"], "true")
        self.assertEqual(environment["SYNCHRONIZE_KLIPPER"], "true")
        self.assertEqual(environment["CONFIRM_REAL_ROBOT_MOVE"], "YES")
        self.assertEqual(environment["CONFIRM_CARTESIAN_MOVE"], "YES")
        self.assertEqual(environment["CONFIRM_KLIPPER_STEPPER"], "YES")


class FixedRosDispatchTests(unittest.TestCase):
    def test_bridge_check_uses_only_the_fixed_ros_shell(self) -> None:
        args = args_for(check_only=True)

        with mock.patch.object(
            coordinator.platform, "system", return_value="Darwin"
        ), mock.patch.object(coordinator, "run_command", return_value=0) as run:
            coordinator.run_ros_coordinator(args, execute=False)

        command = run.call_args.args[0]
        environment = run.call_args.kwargs["env"]
        self.assertEqual(command, [str(coordinator.UR_MOVE_SCRIPT)])
        self.assertEqual(environment["SYNCHRONIZE_KLIPPER"], "true")
        self.assertEqual(environment["KLIPPER_CHECK_ONLY"], "true")
        self.assertEqual(environment["EXECUTE"], "false")

    def test_windows_plan_only_dispatches_python_runtime_not_shell(self) -> None:
        args = args_for(
            check_only=True,
            axes="z,x",
            distance_m=-0.04,
            max_joint_speed_rad_s=0.025,
            stepper="stepper_aux",
            stepper_move=-12.5,
            stepper_speed=2.5,
            stepper_accel=10.0,
        )
        with mock.patch.object(
            coordinator.platform, "system", return_value="Windows"
        ), mock.patch.object(coordinator, "run_command", return_value=0) as run:
            coordinator.run_ros_coordinator(args, execute=False)

        command = run.call_args.args[0]
        environment = run.call_args.kwargs["env"]
        self.assertEqual(command[:3], [
            coordinator.sys.executable,
            str(coordinator.WINDOWS_RUNTIME),
            "run-coordinator",
        ])
        self.assertNotIn(str(coordinator.UR_MOVE_SCRIPT), command)
        self.assertEqual(command[command.index("--execute") + 1], "false")
        self.assertEqual(command[command.index("--axes") + 1], "z,x")
        self.assertEqual(
            command[command.index("--klipper-check-only") + 1],
            "true",
        )
        self.assertNotIn("--confirm-real-robot-move", command)
        self.assertEqual(environment["EXECUTE"], "false")

    def test_windows_execute_dispatches_all_exact_confirmations(self) -> None:
        args = args_for(
            execute=True,
            confirm_real_robot_move="YES",
            confirm_cartesian_move="YES",
            confirm_klipper_stepper="YES",
            reset_stepper_position=True,
            return_to_start=True,
        )
        coordinator.validate_args(args)
        with mock.patch.object(
            coordinator.platform, "system", return_value="Windows"
        ), mock.patch.object(coordinator, "run_command", return_value=0) as run:
            coordinator.run_ros_coordinator(args, execute=True)

        command = run.call_args.args[0]
        self.assertEqual(command[command.index("--execute") + 1], "true")
        self.assertEqual(
            command[command.index("--confirm-real-robot-move") + 1],
            "YES",
        )
        self.assertEqual(
            command[command.index("--confirm-cartesian-move") + 1],
            "YES",
        )
        self.assertEqual(
            command[command.index("--confirm-klipper-stepper") + 1],
            "YES",
        )
        self.assertEqual(
            command[command.index("--klipper-reset-position") + 1],
            "true",
        )
        self.assertEqual(
            command[command.index("--return-to-start") + 1],
            "true",
        )

    def test_windows_external_control_uses_runtime(self) -> None:
        args = args_for()
        with mock.patch.object(
            coordinator.platform, "system", return_value="Windows"
        ), mock.patch.dict(
            coordinator.os.environ,
            {"TIMEOUT_SECONDS": "12"},
        ), mock.patch.object(coordinator, "run_command", return_value=0) as run:
            coordinator.wait_for_ur_ready(args)

        self.assertEqual(
            run.call_args.args[0],
            [
                coordinator.sys.executable,
                str(coordinator.WINDOWS_RUNTIME),
                "wait-external-control",
                "--timeout",
                "12",
            ],
        )

    def test_plan_only_main_dispatches_ros_shell_without_ur_ready_wait(self) -> None:
        args = args_for()

        with mock.patch.object(coordinator, "parse_args", return_value=args), mock.patch.object(
            coordinator, "wait_for_ur_ready"
        ) as wait_ready, mock.patch.object(
            coordinator, "run_ros_coordinator"
        ) as run_ros:
            result = coordinator.main()

        self.assertEqual(result, 0)
        wait_ready.assert_not_called()
        run_ros.assert_called_once_with(args, execute=False)

    def test_check_only_main_dispatches_ros_shell_without_ur_planning(self) -> None:
        args = args_for(check_only=True)

        with mock.patch.object(coordinator, "parse_args", return_value=args), mock.patch.object(
            coordinator, "run_ros_coordinator"
        ) as run_ros:
            result = coordinator.main()

        self.assertEqual(result, 0)
        run_ros.assert_called_once_with(args, execute=False)


class HostIsolationTests(unittest.TestCase):
    def test_host_coordinator_has_no_moonraker_write_or_gcode_api(self) -> None:
        forbidden_attributes = {
            "moonraker_request",
            "send_klipper_gcode",
            "klipper_outbound_script",
            "klipper_return_script",
            "disable_klipper_stepper",
        }
        for attribute in forbidden_attributes:
            self.assertFalse(hasattr(coordinator, attribute), attribute)

        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("urllib.request", source)
        self.assertNotIn("/printer/gcode/script", source)
        self.assertNotIn("MANUAL_STEPPER", source)


if __name__ == "__main__":
    unittest.main()
