"""Regress stale Colima network recovery without Docker or hardware access.

The HTTP, process, Docker, and VM boundaries are all replaced with mocks.  These
tests exercise the real preflight orchestration and its refusal conditions.
"""

from __future__ import annotations

from contextlib import ExitStack
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import unittest
from unittest import mock
from urllib.error import URLError


PROJECT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "colima_network_recovery_under_test", PROJECT / "scripts/check_ros_container_network.py"
)
assert SPEC is not None and SPEC.loader is not None
NETWORK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NETWORK)


class ColimaNetworkRecoveryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.home = Path("/tmp/ur-colima-recovery-test-home")
        self.endpoint = f"unix://{self.home}/.colima/default/docker.sock"
        self.containers = NETWORK.CONTAINER + "\n"
        self.container_processes = "/usr/bin/supervisord\n/usr/bin/Xvfb :1\n"
        self.container_labels = "ur3e-real ros2\n"
        self.host_processes = "/usr/bin/python3 Software/control_station.py\n"
        self.failed_inventory = False
        self.failed_process_inventory = False
        self.failed_colima_start = False
        self.colima_start_exit: int | None = None
        self.calls: list[tuple[list[str], int]] = []
        self.inspected_contexts: list[str | None] = []
        self.stack.enter_context(mock.patch.dict(os.environ, {}, clear=True))
        self.stack.enter_context(mock.patch("platform.system", return_value="Darwin"))
        self.stack.enter_context(mock.patch.object(Path, "home", return_value=self.home))
        self.command = self.stack.enter_context(
            mock.patch.object(NETWORK, "command", side_effect=self.fake_command)
        )
        self.vm_curl = self.stack.enter_context(
            mock.patch.object(NETWORK.subprocess, "run", side_effect=self.fake_vm_curl)
        )
        self.response = mock.MagicMock()
        self.response.__enter__.return_value = self.response
        self.response.read.return_value = json.dumps({"result": {"state": "ready"}}).encode()
        self.opener = mock.Mock()
        self.opener.open.return_value = self.response
        self.build_opener = self.stack.enter_context(
            mock.patch("urllib.request.build_opener", return_value=self.opener)
        )

    def fake_command(self, arguments: list[str], _description: str, *, timeout: int = 10) -> str:
        args = list(arguments)
        self.calls.append((args, timeout))
        if args[:3] == ["docker", "context", "inspect"]:
            self.inspected_contexts.append(os.environ.get("DOCKER_CONTEXT"))
            return self.endpoint + "\n"
        if args == ["docker", "context", "show"]:
            return "colima\n"
        if args == ["docker", "context", "use", "colima"]:
            return "colima\n"
        if args[:2] == ["docker", "ps"]:
            if self.failed_inventory:
                raise RuntimeError("Running container inspection failed.")
            return self.containers
        if args[:2] == ["docker", "inspect"]:
            return self.container_labels
        if args[:2] == ["docker", "top"]:
            if "pid,args" not in args:
                raise RuntimeError("Docker could not find PID field in ps output")
            if self.failed_process_inventory:
                raise RuntimeError("ROS process inspection failed.")
            return self.container_processes
        if args[:1] == ["ps"]:
            return self.host_processes
        if args[:2] == ["colima", "start"]:
            if self.colima_start_exit is not None:
                raise SystemExit(self.colima_start_exit)
            if self.failed_colima_start:
                raise RuntimeError("Colima network recovery start failed.")
        if args[:2] in (["colima", "stop"], ["colima", "start"]):
            return ""
        if args == ["docker", "start", NETWORK.CONTAINER]:
            return NETWORK.CONTAINER + "\n"
        raise AssertionError(f"Unexpected external command: {args!r}")

    def fake_vm_curl(self, arguments, **kwargs):
        args = list(arguments)
        self.assertEqual(args[:3], ["colima", "ssh", "--"])
        self.assertIn("curl", args)
        self.assertIn("http://192.168.0.168/printer/info", args)
        self.assertIn("--noproxy", args)
        self.assertIn("--max-time", args)
        self.assertTrue(kwargs.get("capture_output"))
        self.assertIsNotNone(kwargs.get("timeout"))
        return subprocess.CompletedProcess(args, 7, stdout="7", stderr="Connection refused")

    def assert_no_restart(self) -> None:
        self.assertFalse(any(args[:2] == ["colima", "stop"] for args, _ in self.calls))
        self.assertFalse(any(args[:2] == ["colima", "start"] for args, _ in self.calls))
        self.assertFalse(any(args[:2] == ["docker", "start"] for args, _ in self.calls))

    def test_healthy_preflight_never_attempts_recovery(self) -> None:
        with mock.patch.object(NETWORK, "preflight") as preflight, mock.patch.object(
            NETWORK, "recover_colima_network"
        ) as recover:
            NETWORK.ensure_preflight("192.168.0.104", "http://192.168.0.168", repair_stale_colima=True)
        preflight.assert_called_once_with("192.168.0.104", "http://192.168.0.168")
        recover.assert_not_called()
        self.assert_no_restart()

    def test_transport_recovery_is_opt_in(self) -> None:
        with mock.patch.object(
            NETWORK, "preflight", side_effect=NETWORK.ContainerTransportError("VM connection refused")
        ) as preflight, mock.patch.object(NETWORK, "recover_colima_network") as recover:
            with self.assertRaises(NETWORK.ContainerTransportError):
                NETWORK.ensure_preflight("192.168.0.104", "http://192.168.0.168")
        self.assertEqual(preflight.call_count, 1)
        recover.assert_not_called()

    def test_stale_vm_recovers_once_then_runs_full_preflight_again(self) -> None:
        with mock.patch.object(
            NETWORK, "preflight", side_effect=[NETWORK.ContainerTransportError("VM connection refused"), None]
        ) as preflight:
            NETWORK.ensure_preflight("192.168.0.104", "http://192.168.0.168", repair_stale_colima=True)
        self.assertEqual(preflight.call_count, 2)
        mutations = [(args, timeout) for args, timeout in self.calls if args[:2] in (
            ["colima", "stop"], ["colima", "start"], ["docker", "start"]
        )]
        self.assertEqual(mutations, [
            (["colima", "stop"], 60),
            (["colima", "start"], 120),
            (["docker", "start", NETWORK.CONTAINER], 30),
        ])
        commands = [args for args, _ in self.calls]
        self.assertLess(
            commands.index(["colima", "start"]),
            commands.index(["docker", "context", "use", "colima"]),
        )
        self.assertLess(
            commands.index(["docker", "context", "use", "colima"]),
            commands.index(["docker", "start", NETWORK.CONTAINER]),
        )
        request = self.opener.open.call_args.args[0]
        self.assertEqual(getattr(request, "full_url", request), "http://192.168.0.168/printer/info")
        proxy_handler = self.build_opener.call_args.args[0]
        self.assertEqual(proxy_handler.proxies, {})

    def test_failed_recheck_never_repeats_vm_restart(self) -> None:
        with mock.patch.object(
            NETWORK, "preflight", side_effect=NETWORK.ContainerTransportError("still disconnected")
        ) as preflight:
            with self.assertRaises(NETWORK.ContainerTransportError):
                NETWORK.ensure_preflight("192.168.0.104", "http://192.168.0.168", repair_stale_colima=True)
        self.assertEqual(preflight.call_count, 2)
        self.assertEqual(sum(args[:2] == ["colima", "stop"] for args, _ in self.calls), 1)
        self.assertEqual(sum(args[:2] == ["colima", "start"] for args, _ in self.calls), 1)

    def test_failed_colima_start_restores_context_without_starting_container(self) -> None:
        self.failed_colima_start = True
        with self.assertRaisesRegex(RuntimeError, "Colima network recovery start failed"):
            NETWORK.recover_colima_network("http://192.168.0.168")
        commands = [args for args, _ in self.calls]
        self.assertEqual(commands.count(["colima", "stop"]), 1)
        self.assertEqual(commands.count(["colima", "start"]), 1)
        self.assertEqual(commands.count(["docker", "context", "use", "colima"]), 1)
        self.assertLess(
            commands.index(["colima", "start"]),
            commands.index(["docker", "context", "use", "colima"]),
        )
        self.assertFalse(any(args[:2] == ["docker", "start"] for args in commands))

    def test_canceled_colima_start_restores_context_without_starting_container(self) -> None:
        self.colima_start_exit = 143
        with self.assertRaises(SystemExit) as canceled:
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assertEqual(canceled.exception.code, 143)
        commands = [args for args, _ in self.calls]
        self.assertEqual(commands.count(["colima", "start"]), 1)
        self.assertEqual(commands.count(["docker", "context", "use", "colima"]), 1)
        self.assertLess(
            commands.index(["colima", "start"]),
            commands.index(["docker", "context", "use", "colima"]),
        )
        self.assertFalse(any(args[:2] == ["docker", "start"] for args in commands))

    def test_non_transport_preflight_failure_never_recovers(self) -> None:
        for reason in ("Klipper heater shutdown", "No attached network", "Missing UR publication"):
            with self.subTest(reason=reason), mock.patch.object(
                NETWORK, "preflight", side_effect=RuntimeError(reason)
            ), mock.patch.object(NETWORK, "recover_colima_network") as recover:
                with self.assertRaisesRegex(RuntimeError, reason):
                    NETWORK.ensure_preflight("192.168.0.104", "http://192.168.0.168", repair_stale_colima=True)
                recover.assert_not_called()
        self.assert_no_restart()

    def test_host_offline_never_restarts_vm(self) -> None:
        self.opener.open.side_effect = URLError("host connection refused")
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_host_klipper_shutdown_never_restarts_vm(self) -> None:
        self.response.read.return_value = b'{"result":{"state":"shutdown"}}'
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_non_macos_runtime_never_restarts_vm(self) -> None:
        with mock.patch("platform.system", return_value="Linux"):
            with self.assertRaises(RuntimeError):
                NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_unrelated_docker_context_never_restarts_colima(self) -> None:
        self.endpoint = "unix:///var/run/docker.sock"
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_docker_host_override_must_match_colima(self) -> None:
        with mock.patch.dict(os.environ, {"DOCKER_HOST": "tcp://other-host:2375"}):
            with self.assertRaises(RuntimeError):
                NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_effective_docker_context_override_is_inspected(self) -> None:
        with mock.patch.dict(os.environ, {"DOCKER_CONTEXT": "colima"}):
            NETWORK.recover_colima_network("http://192.168.0.168")
            self.assertEqual(os.environ["DOCKER_CONTEXT"], "colima")
        inspections = [args for args, _ in self.calls if args[:3] == ["docker", "context", "inspect"]]
        self.assertEqual(len(inspections), 1)
        # Docker itself resolves DOCKER_CONTEXT when inspect has no name.
        self.assertEqual(self.inspected_contexts, ["colima"])

    def test_unrelated_colima_profile_blocks_vm_restart(self) -> None:
        with mock.patch.dict(os.environ, {"COLIMA_PROFILE": "other-project"}):
            with self.assertRaises(RuntimeError):
                NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_unrelated_running_container_blocks_vm_restart(self) -> None:
        self.containers += "unrelated-database\n"
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_runtime_idle_check_allows_an_empty_vm(self) -> None:
        self.containers = ""
        NETWORK.require_idle_colima(allow_no_container=True)
        self.assertFalse(any(args[:2] == ["docker", "top"] for args, _ in self.calls))
        self.assert_no_restart()

    def test_same_container_name_with_wrong_owner_blocks_vm_restart(self) -> None:
        self.container_labels = "another-project ros2\n"
        with self.assertRaisesRegex(RuntimeError, "unrecognized ROS container"):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_running_hardware_clients_block_vm_restart(self) -> None:
        for process in (
            "ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.0.104",
            "/opt/ros/humble/lib/ur_robot_driver/ur_ros2_control_node",
            "/opt/ros/humble/lib/moveit_ros_move_group/move_group",
            "/home/ubuntu/ur_ros2_ws/install/ur_klipper_bridge/lib/ur_klipper_bridge/klipper_bridge",
            "python3 /home/ubuntu/ur_ros2_ws/scripts/gcode_print_executor.py",
        ):
            with self.subTest(process=process):
                self.calls.clear()
                self.container_processes = process + "\n"
                with self.assertRaises(RuntimeError):
                    NETWORK.recover_colima_network("http://192.168.0.168")
                self.assert_no_restart()

    def test_host_print_executor_blocks_vm_restart(self) -> None:
        for process in (
            "python3 /Users/operator/project/scripts/run_gcode_print_execution.py --execute",
            "python3 /Users/operator/project/scripts/gcode_print_executor.py --execute",
        ):
            with self.subTest(process=process):
                self.calls.clear()
                self.host_processes = process + "\n"
                with self.assertRaises(RuntimeError):
                    NETWORK.recover_colima_network("http://192.168.0.168")
                self.assert_no_restart()

    def test_failed_container_inventory_never_restarts_vm(self) -> None:
        self.failed_inventory = True
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_failed_process_inventory_never_restarts_vm(self) -> None:
        self.failed_process_inventory = True
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_healthy_vm_transport_never_restarts_vm(self) -> None:
        self.vm_curl.side_effect = None
        self.vm_curl.return_value = subprocess.CompletedProcess([], 0, stdout="0", stderr="")
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()

    def test_unexpected_vm_probe_failure_never_restarts_vm(self) -> None:
        self.vm_curl.side_effect = None
        self.vm_curl.return_value = subprocess.CompletedProcess([], 255, stdout="", stderr="SSH failed")
        with self.assertRaises(RuntimeError):
            NETWORK.recover_colima_network("http://192.168.0.168")
        self.assert_no_restart()


if __name__ == "__main__":
    unittest.main()
