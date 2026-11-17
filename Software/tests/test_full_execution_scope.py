"""Offline guarantees for explicit full-file cold motion-test packages."""

from __future__ import annotations

import copy
import json
import tempfile
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from Software.tests.test_print_execution import (
    execution, moveit, review, write_request, write_trajectory,
)
from Software.tests.test_control_station import station


def full_review() -> dict:
    payload = review()
    payload["operation_mode"] = "tool0_motion_test"
    payload["air_run_check"] = {"passed": True}
    payload["path"]["segments"] = [
        {
            "sequence": index + 1, "line": index + 1, "layer": 0,
            "x0": index * 0.01, "x1": (index + 1) * 0.01,
            "y0": 0.0, "y1": 0.0, "z0": 0.0, "z1": 0.0,
        }
        for index in range(20)
    ]
    payload["timeline"]["events"] = [
        {
            "sequence": index + 1, "line": index + 1, "duration_s": 5.0,
            "ur_distance_mm": 10.0, "klipper_units": 1.0,
            "klipper_action": "extrude",
        }
        for index in range(20)
    ] + [{
        "sequence": 21, "line": 21, "duration_s": 0.4,
        "ur_distance_mm": 0.0, "klipper_units": -0.5,
        "klipper_action": "retract",
    }]
    return payload


class FullExecutionScopeTests(unittest.TestCase):
    def prepare(self, root: Path, extent: str = "sample", *, payload=None):
        approach, printing = root / "approach.csv", root / "print.csv"
        write_trajectory(approach, distance=0.0)
        write_trajectory(printing)
        return execution.prepare_execution_manifest(
            review=payload or full_review(), moveit_result=moveit(),
            approach_trajectory_path=approach, print_trajectory_path=printing,
            planning_request_path=write_request(root),
            destination=root / f"{extent}.json", manual_stepper="extruder",
            execution_extent=extent,
        )

    def test_full_preserves_all_blocks_and_terminal_retraction_after_retiming(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sample = self.prepare(root)
            full = self.prepare(root, "full")
            expected = execution._execution_blocks(full_review(), 1.0)
            self.assertLess(sample["block_count"], len(expected))
            self.assertEqual(full["block_count"], len(expected))
            self.assertIn("print", full["trajectory_profiles"])
            self.assertNotIn("blocks", full["trajectory_profiles"])
            self.assertEqual(
                full["coordination"]["synchronized_start_count"], 1
            )
            self.assertEqual(full["execution_extent"], "full")
            self.assertEqual(full["execution_scope"]["type"], "full_cold_motion_test")
            self.assertEqual(full["execution_scope"]["end_progress"], 1.0)
            self.assertEqual(full["execution_scope"]["source_line_end"], 21)
            self.assertEqual(full["blocks"][-1]["kind"], "retract")
            self.assertFalse(full["heating_enabled"])
            self.assertEqual(full["operation_mode"], "tool0_motion_test")
            self.assertEqual([b["klipper_units"] for b in full["blocks"]],
                             [b["klipper_units"] for b in expected])
            self.assertAlmostEqual(full["execution_scope"]["motion_duration_s"],
                                   sum(b["duration_s"] for b in full["blocks"]), places=3)
            for field in ("pause_semantics", "stop_semantics", "stepper_limits"):
                self.assertEqual(full[field], sample[field])
            self.assertNotEqual(full["execution_id"], sample["execution_id"])
            self.assertEqual(execution.read_execution_manifest(root / "full.json"), full)

    def test_unknown_extent_is_rejected_before_creating_package(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for extent in ("everything", "FULL", "", None, True):
                with self.subTest(extent=extent), self.assertRaisesRegex(
                    execution.PrintExecutionError, "extent"
                ):
                    self.prepare(root, extent)
            self.assertFalse((root / "everything.json").exists())

    def test_full_scope_does_not_relax_collision_or_table_plane_gates(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            invalid = full_review()
            invalid["air_run_check"] = {"passed": False}
            with self.assertRaisesRegex(execution.PrintExecutionError, "table-plane"):
                self.prepare(root, "full", payload=invalid)

    def test_extent_change_invalidates_immutable_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root, "full")
            manifest["execution_extent"] = "sample"
            (root / "full.json").write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(execution.PrintExecutionError, "changed"):
                execution.read_execution_manifest(root / "full.json")

    def test_full_scope_rejects_missing_final_cartesian_coverage(self):
        blocks = execution._execution_blocks(full_review(), 1.0)
        incomplete = copy.deepcopy(blocks)
        incomplete[-1]["end_progress"] = 0.9
        with self.assertRaisesRegex(execution.PrintExecutionError, "full.*path|coverage"):
            execution._execution_scope(incomplete, "tool0_motion_test", "full")

    def test_rehashed_full_packages_still_reject_range_and_terminal_mismatches(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            full = self.prepare(root, "full")
            cases = {
                "progress gap": lambda m: m["blocks"][5].update(start_progress=0.251),
                "lost process-only tail": lambda m: m.update(source_block_count=m["block_count"] + 1),
                "wrong terminal line": lambda m: m.update(source_motion_line_end=99),
                "wrong full label": lambda m: m["execution_scope"].update(type="bounded_cold_motion_sample"),
                "wrong UR endpoint": lambda m: m["print_trajectory"].update(end_positions=[0.1] * 6),
                "heating enabled": lambda m: m.update(heating_enabled=True),
            }
            for name, mutate in cases.items():
                with self.subTest(name=name):
                    invalid = copy.deepcopy(full)
                    mutate(invalid)
                    invalid["execution_id"] = execution._manifest_execution_id(invalid)
                    (root / "full.json").write_text(json.dumps(invalid), encoding="utf-8")
                    with self.assertRaises(execution.PrintExecutionError):
                        execution.read_execution_manifest(root / "full.json")

    def test_public_package_exposes_exact_full_cold_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            full = self.prepare(Path(directory), "full")
            package = station.StationBackend._public_print_package(full)
            self.assertEqual(package["execution_extent"], "full")
            self.assertEqual(package["execution_scope"]["type"], "full_cold_motion_test")
            self.assertEqual(package["execution_scope"]["end_progress"], 1.0)
            self.assertEqual(package["operation_mode"], "tool0_motion_test")
            self.assertFalse(package["heating_enabled"])
            self.assertEqual(package["operator_action_required"], "execute")
            self.assertNotIn("authorization_required", package)
            self.assertEqual(package["estimated_minimum_duration_s"],
                             round(full["planned_duration_s"] + 4.0, 3))

    def test_prepare_route_transports_extent_and_defaults_to_sample(self):
        prepare = mock.Mock(return_value={"execution_id": "a" * 16})
        backend = SimpleNamespace(prepare_print_execution=prepare)
        server = ThreadingHTTPServer(("127.0.0.1", 0), station.make_handler(backend))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            for extent in (None, "full"):
                with self.subTest(extent=extent):
                    headers = {
                        "Content-Type": "application/octet-stream",
                        "X-Gcode-Filename": "cold.gcode",
                        "X-Moveit-Job-Id": "b" * 12,
                    }
                    if extent is not None:
                        headers["X-Execution-Extent"] = extent
                    request = urllib.request.Request(
                        f"http://127.0.0.1:{server.server_port}/api/print/prepare",
                        data=b"G1 X1 E1 F600\n", headers=headers, method="POST",
                    )
                    with urllib.request.urlopen(request, timeout=3) as response:
                        self.assertEqual(response.status, 200)
                    prepare.assert_called_with(
                        "cold.gcode", b"G1 X1 E1 F600\n", "b" * 12,
                        execution_extent=extent or "sample",
                    )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_host_rejects_unknown_extent_before_touching_jobs(self):
        backend = station.StationBackend.__new__(station.StationBackend)
        backend._require_no_unconfirmed_print_stop = lambda: None
        backend.jobs = mock.Mock()
        for extent in ("unknown", None, [], True):
            with self.subTest(extent=extent), self.assertRaises(station.ApiError) as raised:
                backend.prepare_print_execution("test.gcode", b"G1 X1\n", "b" * 12,
                                                execution_extent=extent)
            self.assertEqual(raised.exception.status, 400)
        backend.jobs.get.assert_not_called()

    def test_superseded_review_removes_unpublished_pending_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            backend = station.StationBackend.__new__(station.StationBackend)
            backend.demo = False
            backend._print_execution_runtime = root
            backend._print_execution_lock = threading.RLock()
            backend._calibration_lock = threading.RLock()
            backend._calibration_path = root / "gcode_calibration.json"
            backend._print_packages = {}
            backend._planning_workcell_snapshot = mock.Mock(return_value=(
                station.load_workcell_geometry(), False,
                station.load_workcell_geometry()["tabletop"]["surface_z_m"], False,
            ))
            reviewed = {"review_sha256": "c" * 64}
            metadata = {
                "review": reviewed,
                "review_id": "review-1",
                "result_path": root / "moveit.json",
                "approach_trajectory_path": root / "approach.csv",
                "print_trajectory_path": root / "print.csv",
                "planning_request_path": root / "planning.json",
            }
            backend._validate_print_preparation_request = mock.Mock(
                return_value=("d" * 64, metadata)
            )
            backend._automatic_calibration_for_analysis = mock.Mock(
                return_value={}
            )
            backend._container_workspace_path = lambda path: str(path)
            backend._require_current_planning_request = mock.Mock(
                side_effect=station.PrintExecutionError("Review was superseded.")
            )

            def write_pending(**kwargs):
                kwargs["destination"].write_bytes(b"x" * 1024)
                return {"execution_id": "e" * 16}

            with mock.patch.object(station, "parse_gcode", return_value={}), \
                 mock.patch.object(station, "build_offline_review", return_value=reviewed), \
                 mock.patch.object(station, "read_saved_calibration", return_value={"profile_sha256": "f" * 64}), \
                 mock.patch.object(station, "read_moveit_result", return_value={}), \
                 mock.patch.object(station, "prepare_execution_manifest", side_effect=write_pending):
                with self.assertRaises(station.ApiError) as raised:
                    backend.prepare_print_execution(
                        "model.gcode", b"G1 X1\n", "b" * 12,
                        execution_extent="full",
                        execution_config={
                            "manual_stepper": "extruder",
                            "stepper_speed": 5.0,
                            "stepper_accel": 20.0,
                        },
                    )

            self.assertEqual(raised.exception.status, 422)
            self.assertEqual(list(root.glob("pending-*.json")), [])
            self.assertEqual(backend._print_packages, {})


if __name__ == "__main__":
    unittest.main()
