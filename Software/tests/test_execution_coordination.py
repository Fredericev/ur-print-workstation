"""No hardware: exercise the executor's real passthrough dispatch preparation."""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import json
import tempfile
import threading
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[2] / "workspaces/ur_ros2_ws/scripts/gcode_print_executor.py"
spec = importlib.util.spec_from_file_location("coordination_runner", PATH)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class CoordinationTests(unittest.TestCase):
    def test_bridge_protocol_rejects_running_old_code_and_accepts_fixed_code(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor.cancel_requested = threading.Event()
        executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        for version, passes in (("", False), (runner.BRIDGE_PROTOCOL_VERSION, True)):
            response = SimpleNamespace(values=[SimpleNamespace(string_value=version)])
            executor.bridge_protocol_client = SimpleNamespace(
                wait_for_service=lambda **kwargs: True,
                call_async=lambda request: SimpleNamespace(done=lambda: True, result=lambda: response),
            )
            if passes:
                executor._require_bridge_protocol()
            else:
                with self.assertRaises(runner.ExecutionFailure):
                    executor._require_bridge_protocol()

    def test_speed_scaling_is_percent_and_stale_or_scaled_timing_fails_closed(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor._initialize_goal_tracking()
        executor.latest_scaling = 100.0
        executor.latest_scaling_stamp = 10.0
        executor.scaling_stable_since = 9.0
        with patch.object(runner.time, "monotonic", return_value=10.1):
            executor._require_coordination_scaling()
            executor._scaling_callback(SimpleNamespace(data=20.0))
            with self.assertRaises(runner.ExecutionFailure):
                executor._require_coordination_scaling()
        executor.latest_scaling = 100.0
        with patch.object(runner.time, "monotonic", return_value=13.0):
            with self.assertRaises(runner.ExecutionFailure):
                executor._require_coordination_scaling()

    def test_unconfirmed_stop_blocks_another_package_even_after_app_restart(self):
        from Software.control_station import StationBackend, ApiError
        from Software.runtime_lifecycle import RuntimeLifecycle
        backend = StationBackend.__new__(StationBackend)
        with tempfile.TemporaryDirectory() as folder:
            backend._print_execution_runtime = Path(folder)
            backend.lifecycle = RuntimeLifecycle(
                runtime_dir=backend._print_execution_runtime,
                stop_owned_jobs=lambda _reason: {},
                stop_owned_runtime=lambda _reason: {},
                owned_runtime_stopped=lambda: True,
                stop_and_verify_ur=lambda: {"stopped": True},
                verify_klipper_stopped=lambda: {"stopped": True},
            )
            execution_id = "0123456789abcdef"
            status = Path(folder) / f"{execution_id}.status.json"
            status.write_text(json.dumps({
                "execution_id": execution_id,
                "state": "stop-unconfirmed",
                "stop": {},
            }))
            with self.assertRaises(ApiError):
                backend._require_no_unconfirmed_print_stop()
            status.with_name("old.recovery.json").write_text(
                json.dumps({"state": "stop-recovered", "final_probe": {"safe": True}})
            )
            status.with_name("old.recovery.claim").write_text("legacy marker")
            with self.assertRaises(ApiError):
                backend._require_no_unconfirmed_print_stop()
            status.write_text(json.dumps({"state": "failed"}))
            backend._require_no_unconfirmed_print_stop()


class CoordinationDispatchTests(unittest.TestCase):
    def test_speed_cap_and_prelude_follow_the_installed_v5_trajectory_limit(self):
        self.assertEqual(
            runner.MAX_JOINT_SPEED_RAD_S,
            runner.TRAJECTORY_LIMITS["velocity"],
        )
        self.assertAlmostEqual(
            runner.TRAJECTORY_PRELUDE_S,
            1.5 * runner.START_TOLERANCE_RAD / runner.TRAJECTORY_LIMITS["velocity"],
        )
        message = runner.RosPrintExecutor._scaling_message({
            "reason": "scaled", "value_percent": 50.0,
            "age_s": 0.1, "stable_age_s": 0.1,
        })
        self.assertIn(
            f"{runner.TRAJECTORY_LIMITS['velocity']:.3g} rad/s",
            message,
        )
        self.assertNotIn("0.03 rad/s", message)

    def test_low_speed_confirmation_is_distinct_from_legacy_slider_checkbox(self):
        from Software.control_station import StationBackend, ApiError
        payload = dict(emergency_stop=True, workspace_clear=True, wired_network=True,
            external_control_playing=True, operator_action="execute", speed_under_20=True)
        with self.assertRaises(ApiError):
            StationBackend._require_action_and_safety(payload, "execute")
        payload.update(reviewed_low_speed=True, speed_under_20=False)
        StationBackend._require_action_and_safety(payload, "execute")

    def test_uniform_segment_timing_caps_zero_velocity_cubic_peak_speed(self):
        positions = [[x] * 6 for x in (0.0, .009, .0091)]
        duration = runner.minimum_ur_duration(positions)
        segment_time = duration / (len(positions) - 1)
        peak_speed = 1.5 * .009 / segment_time
        self.assertLessEqual(peak_speed, runner.MAX_JOINT_SPEED_RAD_S + 1e-12)

    def goal(self):
        return SimpleNamespace(trajectory=SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=0)),
            points=[SimpleNamespace(positions=[x] * 6,
                time_from_start=SimpleNamespace(sec=s, nanosec=50_000_000))
                for x, s in ((0, 0), (.01, 2), (.02, 4))],
        ))

    def test_dispatch_uses_remaining_relative_hold_without_a_fake_absolute_header(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        goal = self.goal()
        executor.status = {}
        executor._write_status = lambda: None
        executor._scheduled_ur_goals = {id(goal): {"start_monotonic": 104.0}}
        with patch.object(runner.time, "monotonic", return_value=101.0):
            executor._prepare_goal_for_dispatch(goal, "UR", 103.5)
        self.assertEqual(goal.trajectory.header.stamp.sec, 0)
        self.assertEqual(goal.trajectory.header.stamp.nanosec, 0)
        self.assertEqual([p.time_from_start.sec for p in goal.trajectory.points], [3, 5, 7])
        self.assertEqual([p.positions[0] for p in goal.trajectory.points], [0, .01, .02])
        self.assertFalse(executor.status["coordination"]["physical_sync_verified"])

    def test_dispatch_refuses_expired_start_without_changing_trajectory(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        goal = self.goal()
        executor._scheduled_ur_goals = {id(goal): {"start_monotonic": 104.0}}
        with patch.object(runner.time, "monotonic", return_value=104.1):
            with self.assertRaises(runner.ExecutionFailure):
                executor._prepare_goal_for_dispatch(goal, "UR", 103.5)
        self.assertEqual(goal.trajectory.points[0].time_from_start.nanosec, 50_000_000)

    def test_uncoordinated_approach_is_unchanged(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        goal = self.goal()
        executor._prepare_goal_for_dispatch(goal, "UR", None)
        self.assertEqual(goal.trajectory.points[0].time_from_start.nanosec, 50_000_000)

    def test_production_enters_the_same_live_scaling_preflight_before_any_goal(self):
        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor.manifest = {"operation_mode": "production_print"}
        executor.status = {}
        calls = []
        executor._wait_for_coordination_scaling = lambda: calls.append("scaling_wait")
        executor._require_coordination_scaling = lambda: calls.append("scaling_guard")
        executor.require_start_match = lambda *args: self.fail("live joint check must follow scaling")
        with self.assertRaises(AssertionError):
            executor.execute_print([{"positions": [0.0] * 6}])
        self.assertEqual(calls, ["scaling_wait", "scaling_guard"])


if __name__ == "__main__":
    unittest.main()
