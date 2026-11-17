from __future__ import annotations

import json
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from Software import windows_runtime

spec = importlib.util.spec_from_file_location("windows_tools_tested", Path(__file__).resolve().parents[1] / "windows_tools.py")
assert spec and spec.loader
tools = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"windows_runtime": windows_runtime}):
    spec.loader.exec_module(tools)
RuntimeFailure = windows_runtime.RuntimeFailure
WindowsRuntime = windows_runtime.WindowsRuntime


class WindowsDisplayToolsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "workspaces/ur_ros2_ws"
        self.workspace.mkdir(parents=True)
        self.runtime = Mock(spec=WindowsRuntime)
        self.runtime.project_root = self.root
        self.runtime.workspace = self.workspace
        self.runtime._container_running.return_value = True
        self.playback = SimpleNamespace(
            manifest_path=self.workspace / "runtime/print_execution/current.json",
            source_path=self.root / "Test gcode/print.gcode", speed_factor=4.0,
        )
        self.loader = Mock(return_value=self.playback)
        self.writer = Mock()
        self.display = tools.WindowsDisplayTools(
            self.runtime, playback_loader=self.loader, preview_writer=self.writer,
        )

        def container(script, *, timeout, label, env=None):
            if label == "Offline replay hardware exclusion":
                return subprocess.CompletedProcess([], 0, "WORKSTATION_NO_HARDWARE=1\n")
            return subprocess.CompletedProcess([], 0, "")

        self.runtime._container_bash.side_effect = container

    def test_unverified_package_is_rejected_before_docker(self):
        self.loader.side_effect = ValueError("source checksum changed")
        with self.assertRaisesRegex(ValueError, "checksum changed"):
            self.display.offline_rviz(calibration=self.root / "calibration.json")
        self.runtime.docker_preflight.assert_not_called()
        self.writer.assert_not_called()

    def test_real_shared_verifier_rejects_missing_manifest_before_docker(self):
        project = Path(__file__).resolve().parents[2]
        with self.assertRaises((OSError, ValueError)):
            tools.load_playback(project, str(self.workspace / "missing.json"), 10.0)
        self.runtime.docker_preflight.assert_not_called()

    def test_manifest_outside_workspace_is_rejected_before_docker(self):
        self.playback.manifest_path = self.root / "other.json"
        with self.assertRaisesRegex(RuntimeFailure, "inside this project's ROS workspace"):
            self.display.offline_rviz(calibration=self.root / "calibration.json")
        self.runtime.docker_preflight.assert_not_called()

    def test_live_control_stack_is_never_stopped_or_reconfigured(self):
        self.runtime._container_bash.side_effect = lambda *args, **kwargs: subprocess.CompletedProcess([], 0, "ur_ros2_control_node")
        with self.assertRaisesRegex(RuntimeFailure, "hardware-control processes"):
            self.display.offline_rviz(calibration=self.root / "calibration.json")
        self.runtime._write_update_rate.assert_not_called()
        self.runtime._start_container.assert_not_called()
        self.runtime._cleanup_driver.assert_not_called()

    def test_failed_process_probe_cannot_be_treated_as_no_hardware(self):
        self.runtime._container_bash.side_effect = RuntimeFailure("Docker failed", 69)
        with self.assertRaisesRegex(RuntimeFailure, "absence could not be verified"):
            self.display.offline_rviz(calibration=self.root / "calibration.json")
        self.runtime._start_container.assert_not_called()

    def test_success_keeps_exact_verified_manifest_and_isolated_display_launch(self):
        url = self.display.offline_rviz(
            speed_factor=4, calibration=self.root / "calibration.json", desktop_port=6083,
        )
        self.assertEqual(url, "http://localhost:6083/vnc.html")
        self.runtime._start_container.assert_called_once_with("auto", 6083, "127.0.0.1")
        calls = self.runtime._container_bash.call_args_list
        launch = next(call for call in calls if call.kwargs["label"] == "Isolated offline RViz replay launch")
        self.assertEqual(launch.kwargs["env"]["OFFLINE_MANIFEST"],
                         "/home/ubuntu/ur_ros2_ws/runtime/print_execution/current.json")
        self.assertEqual(launch.kwargs["env"]["OFFLINE_SPEED_FACTOR"], "4.0")
        self.assertIn("ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1", launch.args[0])
        self.assertIn("ros2 launch ur3e_print_planner offline_print_preview.launch.py", launch.args[0])
        self.assertNotIn("ur_control.launch.py", launch.args[0])
        self.assertNotIn("ros2 run ur_klipper_bridge", launch.args[0])
        self.assertEqual(sum(call.kwargs["label"] == "Offline replay hardware exclusion" for call in calls), 4)

    def test_hardware_appearing_during_container_start_blocks_launch(self):
        self.runtime._container_running.return_value = False
        self.runtime._container_bash.side_effect = lambda *args, **kwargs: subprocess.CompletedProcess([], 0, "klipper_bridge")
        with self.assertRaises(RuntimeFailure):
            self.display.offline_rviz(calibration=self.root / "calibration.json")
        self.runtime._build_ros_sources.assert_not_called()


class WindowsToolPayloadTests(unittest.TestCase):
    def test_helper_initializes_same_versioned_payload_without_copying_backend_or_overwriting_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "package/project"
            (source / "Software").mkdir(parents=True)
            (source / "Software/control_station.py").write_text("backend", encoding="utf-8")
            (source / "docker-compose.real.yml").write_text("services: {}", encoding="utf-8")
            (source / "PAYLOAD_VERSION.txt").write_text("0.14.0-checksum\n", encoding="utf-8")
            env = {"UR_PRINT_PACKAGED_PROJECT_ROOT": str(source), "UR_PRINT_DATA_DIR": str(root / "data")}
            destination = tools.resolve_project_root(env)
            self.assertEqual(destination.name, "0-14-0-checksum")
            self.assertTrue((destination / "docker-compose.real.yml").is_file())
            self.assertFalse((destination / "Software").exists())
            user_state = destination / "operator-state.json"
            user_state.write_text(json.dumps({"keep": True}), encoding="utf-8")
            self.assertEqual(tools.resolve_project_root(env), destination)
            self.assertEqual(json.loads(user_state.read_text(encoding="utf-8")), {"keep": True})


if __name__ == "__main__":
    unittest.main()
