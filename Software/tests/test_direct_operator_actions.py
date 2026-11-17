"""Button intent replaces typing; every hardware guard remains fail-closed.

All operations below are intercepted before a subprocess or hardware API runs.
"""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

from Software import control_station as station


class DirectOperatorActionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = tempfile.TemporaryDirectory(prefix="direct-action-tests-")
        self.addCleanup(self.runtime.cleanup)
        self.backend = station.StationBackend.__new__(station.StationBackend)
        self.backend.demo = False
        self.backend.config = dict(station.DEFAULT_CONFIG)
        self.backend._print_execution_runtime = Path(self.runtime.name)
        self.backend._calibration_path = Path(self.runtime.name) / "gcode_calibration.json"
        self.backend._print_z_placement_path = Path(self.runtime.name) / "print_z_placement.json"
        self.backend._print_z_required_path = Path(self.runtime.name) / "print_z_placement.required"
        self.backend._demo_calibration = None
        self.backend._print_execution_lock = threading.RLock()
        self.backend._calibration_lock = threading.RLock()
        self.backend._height_lock = threading.RLock()
        self.backend._height_draft_m = None
        self.backend._print_z_draft_m = None
        self.backend._print_jobs = {}
        self.backend.jobs = station.JobManager(
            job_log_dir=Path(self.runtime.name) / "logs",
            cancel_dir=Path(self.runtime.name) / "cancel",
        )
        self.payload = dict(
            emergency_stop=True,
            workspace_clear=True,
            reviewed_low_speed=True,
            speed_under_20=True,
            wired_network=True,
            external_control_playing=True,
        )
        self.execution_id = "a" * 16
        self.manifest = {
            "execution_id": self.execution_id,
            "operation_mode": "tool0_motion_test",
            "simulation": False,
            "block_count": 1,
            "print_bed_m": {"plane_z": station.load_workcell_geometry()["tabletop"]["surface_z_m"]},
            "profile_sha256": "c" * 64,
        }

    def test_action_is_required_and_scoped_without_any_typed_phrase(self) -> None:
        for action in (None, "execute", "MOVE TO START"):
            with self.subTest(action=action), self.assertRaises(station.ApiError):
                station.StationBackend._require_action_and_safety(
                    {**self.payload, "operator_action": action}, "move-to-start"
                )
        station.StationBackend._require_action_and_safety(
            {**self.payload, "operator_action": "move-to-start"}, "move-to-start"
        )
        with self.assertRaises(station.ApiError):
            station.StationBackend._require_action_and_safety(
                {**self.payload, "confirmation": "MOVE TO START"}, "move-to-start"
            )

    def test_button_action_does_not_bypass_any_safety_checkbox(self) -> None:
        for flag in (
            "emergency_stop", "workspace_clear", "reviewed_low_speed",
            "wired_network", "external_control_playing",
        ):
            with self.subTest(flag=flag), self.assertRaises(station.ApiError):
                station.StationBackend._require_action_and_safety(
                    {**self.payload, "operator_action": "execute", flag: False}, "execute"
                )

    def start(self, mode: str, **changes):
        with mock.patch.object(
            self.backend, "_execution_package", return_value=(Path("unused"), self.manifest)
        ), mock.patch.object(
            self.backend, "_require_execution_planning_authorization"
        ), mock.patch.object(
            self.backend, "_require_print_stack"
        ), mock.patch.object(
            # This fixture isolates operator intent and reviewed-start state;
            # native config/temperature authorization has its own tests.
            self.backend, "_require_native_extrusion_ready"
        ), mock.patch.object(threading.Thread, "start"), mock.patch.object(
            station, "read_saved_calibration", return_value={"profile_sha256": "c" * 64}
        ):
            return self.backend.start_print_operation(
                {**self.payload, "execution_id": self.execution_id,
                 "operator_action": mode, **changes}, mode=mode
            )

    def test_move_to_start_accepts_one_scoped_action_and_rejects_double_start(self) -> None:
        # The installed printhead blocks legacy Tool0 motion on real hardware.
        # Exercise button-action dispatch only in the hardware-free demo path.
        with mock.patch.object(self.backend, "demo", True):
            job = self.start("move-to-start")
            self.assertTrue(job.motion)
            self.assertEqual(job.command[-1], "move-to-start")
            with self.assertRaises(station.ApiError) as raised:
                self.start("move-to-start")
            self.assertEqual(raised.exception.status, 409)
            self.assertEqual(len(self.backend.jobs._jobs), 1)

    def test_execute_still_requires_completed_reviewed_approach(self) -> None:
        with mock.patch.object(self.backend, "demo", True):
            with self.assertRaises(station.ApiError):
                self.start("execute")
            status_path = self.backend._print_execution_runtime / f"{self.execution_id}.status.json"
            status_path.write_text(json.dumps({"state": "at-start"}), encoding="utf-8")
            job = self.start("execute")
            self.assertEqual(job.command[-1], "execute")

    def test_failed_or_canceled_approach_requires_fresh_review_without_overwriting_evidence(self) -> None:
        self.manifest["operation_mode"] = "production_print"
        status_path = self.backend._print_execution_runtime / f"{self.execution_id}.status.json"
        for state in ("failed", "canceled"):
            with self.subTest(state=state):
                evidence = json.dumps({
                    "execution_id": self.execution_id,
                    "mode": "move-to-start", "state": state,
                    "root_error": {"message": "controller endpoint tolerance failed"},
                    "stop": {"confirmed": True},
                }).encode()
                status_path.write_bytes(evidence)
                with self.assertRaises(station.ApiError) as raised:
                    self.start("move-to-start")
                self.assertEqual(raised.exception.code, "requires_revalidation")
                self.assertEqual(status_path.read_bytes(), evidence)
                self.assertFalse(self.backend.jobs._jobs)

    def test_unsupported_execution_mode_cannot_authorize_a_subprocess(self) -> None:
        with self.assertRaises(station.ApiError):
            self.start("anything-else")
        self.assertFalse(self.backend.jobs._jobs)

    def test_reviewed_production_start_dispatches_only_after_at_start(self) -> None:
        self.manifest["operation_mode"] = "production_print"
        status_path = self.backend._print_execution_runtime / f"{self.execution_id}.status.json"
        status_path.write_text(json.dumps({"state": "at-start"}), encoding="utf-8")
        job = self.start("execute")
        self.assertTrue(job.motion)
        self.assertEqual(job.command[-1], "execute")
        self.assertEqual(len(self.backend.jobs._jobs), 1)

    def test_production_execute_still_requires_reviewed_start(self) -> None:
        self.manifest["operation_mode"] = "production_print"
        with self.assertRaises(station.ApiError) as raised:
            self.start("execute")
        self.assertIn("reviewed print start", raised.exception.message)
        self.assertFalse(self.backend.jobs._jobs)

    def test_simulated_production_package_remains_locked(self) -> None:
        self.manifest.update(operation_mode="production_print", simulation=True)
        with self.assertRaises(station.ApiError) as raised:
            self.start("move-to-start")
        self.assertIn("simulated execution package", raised.exception.message)
        self.assertFalse(self.backend.jobs._jobs)

    def test_unconfirmed_stop_still_blocks_direct_actions(self) -> None:
        previous_id = "b" * 16
        (self.backend._print_execution_runtime / f"{previous_id}.status.json").write_text(
            json.dumps({
                "execution_id": previous_id,
                "state": "stop-unconfirmed",
                "stop": {},
            }),
            encoding="utf-8",
        )
        with self.assertRaises(station.ApiError) as raised:
            self.start("move-to-start")
        self.assertEqual(raised.exception.code, "print_stop_unconfirmed")

    def test_restart_and_stops_require_their_button_action_not_typed_text(self) -> None:
        for method, action in (
            ("restart_ros", "restart-ros"), ("stop_system", "stop-ros"),
            ("stop_moveit", "stop-moveit"), ("stop_ros_driver", "stop-ros-driver"),
        ):
            with self.subTest(method=method):
                self.backend.demo = True
                operation = getattr(self.backend, method)
                with self.assertRaises(station.ApiError):
                    operation({**self.payload, "operator_action": "wrong-action"})
                sentinel = object()
                with mock.patch.object(self.backend, "_queue_ros_launch", return_value=sentinel), \
                     mock.patch.object(self.backend.jobs, "start", return_value=sentinel):
                    self.assertIs(operation({**self.payload, "operator_action": action}), sentinel)

    def test_sync_action_preserves_legacy_low_speed_and_live_stack_guards(self) -> None:
        self.backend.demo = True
        with mock.patch.object(self.backend.jobs, "start", return_value="intercepted"):
            self.assertEqual(self.backend.sync_test(
                {**self.payload, "operator_action": "sync-test"}
            ), "intercepted")
        with self.assertRaises(station.ApiError):
            self.backend.sync_test({**self.payload, "operator_action": "execute"})
        with self.assertRaises(station.ApiError):
            self.backend.sync_test(
                {**self.payload, "operator_action": "sync-test", "speed_under_20": False}
            )
        self.backend.demo = False
        with mock.patch.object(self.backend, "check_klipper", return_value={"ok": True}), \
             mock.patch.object(self.backend, "ros_status", return_value={
                 "external_control": False, "moveit_ready": True,
                 "klipper_bridge_ready": True, "unstable": False,
             }), mock.patch.object(self.backend.jobs, "start") as start, \
             self.assertRaises(station.ApiError):
            self.backend.sync_test({**self.payload, "operator_action": "sync-test"})
        start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
