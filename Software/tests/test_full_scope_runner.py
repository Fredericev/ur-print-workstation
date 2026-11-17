"""Load and completion tests without constructing ROS nodes or sending motion."""

from __future__ import annotations

import copy
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from Software.tests.test_full_execution_scope import full_review
from Software.tests.test_print_execution import (
    execution, moveit, runner, write_request, write_trajectory,
)


def prepare(root: Path, extent: str) -> dict:
    approach, printing = root / "approach.csv", root / "print.csv"
    write_trajectory(approach, distance=0.0)
    write_trajectory(printing)
    return execution.prepare_execution_manifest(
        review=full_review(), moveit_result=moveit(),
        approach_trajectory_path=approach, print_trajectory_path=printing,
        planning_request_path=write_request(root), destination=root / "manifest.json",
        manual_stepper="extruder", execution_extent=extent,
    )


class FullScopeRunnerTests(unittest.TestCase):
    def loader(self, root: Path, manifest: dict):
        payload = copy.deepcopy(manifest)
        hashes = {}
        for key in ("planning_request", "approach_trajectory", "print_trajectory"):
            container_path = f"/home/ubuntu/ur_ros2_ws/runtime/{key}.artifact"
            payload[key]["container_path"] = container_path
            hashes[str(Path(container_path).resolve())] = payload[key]["sha256"]
        payload["execution_id"] = execution._manifest_execution_id(payload)
        path = root / "runner.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        with mock.patch.object(Path, "is_file", return_value=True), mock.patch.object(
            runner, "sha256_file", side_effect=lambda p: hashes[str(p)]
        ):
            return runner.load_manifest(path)

    def test_runner_accepts_valid_full_and_default_sample_scopes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for extent in ("full", "sample"):
                with self.subTest(extent=extent):
                    loaded = self.loader(root, prepare(root, extent))
                    self.assertEqual(loaded["execution_extent"], extent)

    def test_runner_refuses_rehashed_full_label_on_a_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = prepare(root, "sample")
            manifest["execution_extent"] = "full"
            manifest["execution_scope"]["type"] = "full_cold_motion_test"
            manifest["execution_scope"]["requested_duration_s"] = None
            with self.assertRaisesRegex(runner.ExecutionFailure, "scope|coverage|source"):
                self.loader(root, manifest)

    def test_runner_refuses_full_package_with_missing_process_only_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = prepare(root, "full")
            manifest["blocks"].pop()
            manifest["block_count"] -= 1
            manifest["execution_scope"]["motion_duration_s"] = round(
                sum(b["duration_s"] for b in manifest["blocks"]), 3
            )
            with self.assertRaisesRegex(
                runner.ExecutionFailure, "scope|source|timing|profile"
            ):
                self.loader(root, manifest)

    def test_successful_completion_distinguishes_run_progress_from_source_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for extent in ("sample", "full"):
                with self.subTest(extent=extent):
                    manifest = prepare(root, extent)
                    # Exercise the actual executor with inert one-shot actions.
                    # No ROS constructor, client, device or heating path is used.
                    for block in manifest["blocks"]:
                        block["klipper_units"] = 0.0
                    executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
                    executor._initialize_goal_tracking()
                    executor.manifest, executor.status = manifest, {}
                    executor.cancel_requested = threading.Event()
                    executor.ur_client = object()
                    executor.klipper_client = object()
                    executor.ManualStepperProgram = type("Program", (), {
                        "Goal": lambda: type("Goal", (), {})(),
                        "Feedback": type("Feedback", (), {"PHASE_DISPATCHED": 2}),
                    })
                    executor.node = type("Node", (), {
                        "get_clock": lambda _self: type("Clock", (), {
                            "now": lambda _self: type("Now", (), {
                                "nanoseconds": __import__("time").time_ns()
                            })()
                        })()
                    })()
                    executor._write_status = lambda: None
                    executor._wait_for_coordination_scaling = lambda: None
                    executor._require_coordination_scaling = lambda: None
                    executor.require_start_match = lambda *_args: None
                    executor._validate_smooth_motion = lambda *_args: None
                    executor._check_smooth_dispatch = lambda *_args: None
                    executor._trajectory_goal = lambda p, d: (p, d)
                    def send(_client, _goal, label, **kwargs):
                        if label == "Klipper":
                            kwargs["feedback_callback"](type("Message", (), {
                                "feedback": type("Value", (), {
                                    "phase": 2,
                                    "program_epoch_ros_ns": __import__("time").time_ns(),
                                })()
                            })())
                        return object(), type("Future", (), {"done": lambda _self: False})()
                    executor._send = send
                    completed = []
                    executor._await_results = lambda *_args, **_kwargs: completed.append(True)
                    executor._trajectory_socket_state = (True, __import__("time").monotonic())
                    executor.execute_print([{"positions": [0.0] * 6}])
                    self.assertEqual(len(completed), 1)
                    self.assertEqual(executor.status["state"], "completed")
                    self.assertEqual(executor.status["progress"], 1.0)
                    self.assertEqual(executor.status["source_progress"],
                                     manifest["execution_scope"]["end_progress"])
                    self.assertEqual(executor.status["execution_extent"], extent)
                    if extent == "full":
                        self.assertIn("Full G-code cold Tool0", executor.status["message"])
                        self.assertEqual(executor.status["progress"], 1.0)
                    else:
                        self.assertIn("full G-code was not executed", executor.status["message"])
                        self.assertLess(executor.status["source_progress"], 1.0)


if __name__ == "__main__":
    unittest.main()
