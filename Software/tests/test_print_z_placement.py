"""Print Z placement must not masquerade as a physical tabletop measurement."""

from __future__ import annotations

import tempfile
import unittest
import json
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

from Software import control_station, gcode_calibration, workcell_geometry


GCODE = b"G21\nG90\nM83\nG1 X0 Y0 Z0.2 E1 F600\nG1 X10 Y0 E1\n"


class PrintZPlacementTests(unittest.TestCase):
    def _backend(self, directory: str, *, demo: bool = True) -> control_station.StationBackend:
        backend = control_station.StationBackend(demo=demo, data_dir=Path(directory))
        profile = gcode_calibration.default_calibration()
        profile["confirmations"].update({
            "orientation_verified": True,
            "tool_orientation_verified": True,
            "extrusion_ratio_measured": True,
            "tcp_measured": True,
        })
        analysis = control_station.parse_gcode("print-z.gcode", GCODE)
        backend.save_gcode_calibration({
            "calibration": profile,
            "bounds": analysis["bounds"],
            "source_sha256": analysis["file"]["sha256"],
        })
        return backend

    def test_operator_sets_tabletop_then_independent_print_z_for_execution(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workcell_path = Path(directory) / "workcell.json"
            workcell_path.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
            with mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", workcell_path):
                backend = self._backend(directory)
                table = backend.set_workcell_height_draft({"surface_z_mm": -20.0})
                self.assertAlmostEqual(table["surface_z_m"], -0.020)
                self.assertIsNone(table["draft_surface_z_m"])
                self.assertAlmostEqual(
                    workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"], -0.020,
                )
                self.assertEqual(
                    workcell_geometry.load_workcell_geometry()["tabletop"]["measurement"]["surface_z"],
                    "operator_selected",
                )
                with self.assertRaises(control_station.ApiError):
                    backend.start_workcell_height_trial(
                        {"operator_action": "plan-height-trial"}, execute=False,
                    )

                selected = backend.set_print_z_draft({"print_zero_z_mm": -15.0})
                self.assertAlmostEqual(selected["surface_z_m"], -0.020)
                self.assertAlmostEqual(selected["print_zero_z_m"], -0.015)
                restarted = control_station.StationBackend(demo=True, data_dir=Path(directory))
                self.assertAlmostEqual(restarted.get_workcell_height()["surface_z_m"], -0.020)
                self.assertAlmostEqual(restarted.get_workcell_height()["print_zero_z_m"], -0.015)
                review = backend.build_gcode_path_review("print-z.gcode", GCODE)
                self.assertAlmostEqual(review["collision_bed_m"]["plane_z"], -0.020)
                self.assertAlmostEqual(review["placement_model"]["print_zero_z_m"], -0.015)
                self.assertFalse(review["tabletop_height_candidate"])
                self.assertFalse(review["print_z_candidate"])
                self.assertTrue(review["placement_model"]["execution_authoritative"])

                started = backend.start_gcode_moveit_review("print-z.gcode", GCODE)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and backend.jobs.get(started["job"]["id"]).state in {"queued", "running"}:
                    time.sleep(0.02)
                self.assertEqual(backend.jobs.get(started["job"]["id"]).state, "succeeded")
                package = backend.prepare_print_execution(
                    "print-z.gcode", GCODE, started["job"]["id"], execution_extent="full",
                )
                self.assertAlmostEqual(package["print_bed_m"]["plane_z"], -0.020)
                self.assertAlmostEqual(package["print_zero_z_m"], -0.015)
                safety = {
                    "execution_id": package["execution_id"],
                    "emergency_stop": True,
                    "workspace_clear": True,
                    "reviewed_low_speed": True,
                    "wired_network": True,
                    "external_control_playing": True,
                }
                approach = backend.start_print_operation(
                    {**safety, "operator_action": "move-to-start"}, mode="move-to-start",
                )
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and backend.jobs.get(approach.id).state in {"queued", "running"}:
                    time.sleep(0.02)
                self.assertEqual(backend.jobs.get(approach.id).state, "succeeded")
                printing = backend.start_print_operation(
                    {**safety, "operator_action": "execute"}, mode="execute",
                )
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and backend.jobs.get(printing.id).state in {"queued", "running"}:
                    time.sleep(0.02)
                self.assertEqual(backend.jobs.get(printing.id).state, "succeeded")

                backend.set_workcell_height_draft({"surface_z_mm": -19.0})
                updated_review = backend.build_gcode_path_review("print-z.gcode", GCODE)
                self.assertNotEqual(updated_review["review_sha256"], review["review_sha256"])
                with self.assertRaises(control_station.ApiError) as stale:
                    backend.start_print_operation(
                        {**safety, "operator_action": "move-to-start"}, mode="move-to-start",
                    )
                self.assertEqual(stale.exception.code, "workcell_height_changed")

    def test_tabletop_above_old_print_z_requires_reselection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workcell_path = Path(directory) / "workcell.json"
            workcell_path.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
            with mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", workcell_path):
                backend = self._backend(directory)
                backend.set_print_z_draft({"print_zero_z_mm": -11.0})
                changed = backend.set_workcell_height_draft({"surface_z_mm": -10.0})
                self.assertEqual(changed["surface_z_m"], -0.010)
                self.assertIsNone(changed["print_zero_z_m"])
                self.assertTrue(changed["print_z_reverification_required"])
                self.assertAlmostEqual(
                    json.loads(backend._print_z_placement_path.read_text())["print_zero_z_m"], -0.011,
                )
                self.assertTrue(backend.build_gcode_path_review("print-z.gcode", GCODE)["print_z_candidate"])
                selected = backend.set_print_z_draft({"print_zero_z_mm": -9.0})
                self.assertFalse(selected["print_z_reverification_required"])
                self.assertAlmostEqual(selected["print_zero_z_m"], -0.009)

    def test_lowering_tabletop_preserves_valid_selected_print_z(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workcell_path = Path(directory) / "workcell.json"
            workcell_path.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
            with mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", workcell_path):
                backend = self._backend(directory)
                backend.set_print_z_draft({"print_zero_z_mm": -11.0})
                changed = backend.set_workcell_height_draft({"surface_z_mm": -20.0})
                self.assertAlmostEqual(changed["surface_z_m"], -0.020)
                self.assertAlmostEqual(changed["print_zero_z_m"], -0.011)
                self.assertFalse(changed["print_z_reverification_required"])
                review = backend.build_gcode_path_review("print-z.gcode", GCODE)
                self.assertFalse(review["tabletop_height_candidate"])
                self.assertFalse(review["print_z_candidate"])
                self.assertAlmostEqual(review["placement_model"]["print_zero_z_m"], -0.011)

    def test_selected_z_moves_only_print_path_and_keeps_fixed_collision_table(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            initial = backend.build_gcode_path_review("print-z.gcode", GCODE)
            installed_table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]

            setup = backend.set_print_z_draft({"print_zero_z_mm": -11.0})
            candidate = backend.build_gcode_path_review("print-z.gcode", GCODE)
            self.assertAlmostEqual(setup["surface_z_m"], installed_table)
            self.assertAlmostEqual(setup["print_zero_z_m"], -0.011)
            self.assertAlmostEqual(candidate["workcell_model"]["tabletop"]["surface_z_m"], installed_table)
            self.assertAlmostEqual(candidate["collision_bed_m"]["plane_z"], installed_table)
            for before, after in zip(initial["path"]["segments"], candidate["path"]["segments"]):
                for axis in ("x0", "x1", "y0", "y1"):
                    self.assertAlmostEqual(after[axis], before[axis])
                for axis in ("z0", "z1"):
                    self.assertAlmostEqual(
                        after[axis] - before[axis],
                        -0.011 - initial["placement_model"]["print_zero_z_m"],
                    )
            self.assertNotEqual(candidate["review_sha256"], initial["review_sha256"])
            self.assertFalse(candidate["print_z_candidate"])
            self.assertTrue(candidate["placement_model"]["execution_authoritative"])

            with mock.patch.object(backend.jobs, "start") as start_job:
                start_job.side_effect = lambda kind, command, **_kwargs: control_station.Job(
                    id="b" * 12, kind=kind, motion=False, command=command, state="succeeded",
                )
                started = backend.start_gcode_moveit_review("print-z.gcode", GCODE)
            self.assertFalse(started["print_z_candidate"])
            self.assertEqual(start_job.call_args.args[1][-3], str(installed_table))

            backend.set_print_z_draft({"discard": True})
            restored = backend.build_gcode_path_review("print-z.gcode", GCODE)
            self.assertEqual(restored["review_sha256"], initial["review_sha256"])

    def test_elevated_air_run_is_not_blocked_by_table_proximity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            backend.set_print_z_draft({"print_zero_z_mm": 20.0})
            review = backend.build_gcode_path_review("print-z.gcode", GCODE)
            self.assertGreater(review["print_bed_check"]["surface_gap_m"], 0.005)
            self.assertTrue(review["calibration_ready"])
            self.assertFalse(review["print_z_candidate"])
            self.assertTrue(review["placement_model"]["execution_authoritative"])
            self.assertAlmostEqual(review["collision_bed_m"]["plane_z"], table)

    def test_selected_z_rejects_below_table_and_outside_sanity_bound(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            for absolute_z in (table - 0.001, 2.001):
                with self.subTest(absolute_z=absolute_z), self.assertRaises(control_station.ApiError):
                    backend.set_print_z_draft({"print_zero_z_mm": absolute_z * 1000})
            accepted = backend.set_print_z_draft({"print_zero_z_mm": (table + 0.051) * 1000})
            self.assertAlmostEqual(accepted["print_zero_z_m"], table + 0.051)
            self.assertFalse(backend.get_workcell_height()["print_z_candidate"])

    def test_async_review_identity_changes_with_print_z(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            first = backend._gcode_review_profile_snapshot(
                control_station.hashlib.sha256(GCODE).hexdigest(), simulation_only=False,
            )
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            backend.set_print_z_draft({"print_zero_z_mm": (table + 0.003) * 1000})
            second = backend._gcode_review_profile_snapshot(
                control_station.hashlib.sha256(GCODE).hexdigest(), simulation_only=False,
            )
            self.assertNotEqual(first[0], second[0])
            self.assertEqual(first[5], table)
            self.assertAlmostEqual(second[5], table + 0.003)
            self.assertFalse(second[6])

    def test_changed_selected_z_blocks_an_existing_execution_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            backend.set_print_z_draft({"print_zero_z_mm": (table + 0.003) * 1000})
            with mock.patch.object(backend, "_execution_package", return_value=(
                Path(directory) / "old-package.json", {"print_bed_m": {"plane_z": table}},
            )):
                with self.assertRaises(control_station.ApiError) as blocked:
                    backend.start_print_operation(
                        {"execution_id": "a" * 16}, mode="move-to-start",
                    )
            self.assertEqual(blocked.exception.code, "print_z_changed")
            self.assertIsNone(backend.jobs.active())

    def test_print_z_cannot_change_while_old_height_job_is_being_registered(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            new_z = round(table + 0.010, 6)
            execution_id = "a" * 16
            manifest = {
                "execution_id": execution_id,
                "print_bed_m": {"plane_z": table},
                "print_zero_z_m": table,
                "operation_mode": "production_print",
                "simulation": True,
                "block_count": 1,
            }
            start_entered = threading.Event()
            release_start = threading.Event()
            change_attempted = threading.Event()
            change_finished = threading.Event()
            start_errors: list[Exception] = []
            change_errors: list[Exception] = []

            def paused_start(kind: str, command: list[str], **_kwargs: object) -> control_station.Job:
                start_entered.set()
                if not release_start.wait(2):
                    raise AssertionError("test did not release job registration")
                job = control_station.Job(
                    id="b" * 12, kind=kind, motion=True, command=command, state="queued",
                )
                with backend.jobs._lock:
                    backend.jobs._jobs[job.id] = job
                return job

            def start_old_package() -> None:
                try:
                    backend.start_print_operation(
                        {"execution_id": execution_id}, mode="move-to-start",
                    )
                except Exception as exc:
                    start_errors.append(exc)

            def change_print_z() -> None:
                change_attempted.set()
                try:
                    backend.set_print_z_draft({"print_zero_z_mm": new_z * 1000})
                except Exception as exc:
                    change_errors.append(exc)
                finally:
                    change_finished.set()

            with mock.patch.object(backend, "_execution_package", return_value=(
                Path(directory) / "old-package.json", manifest,
            )), mock.patch.object(backend, "_require_action_and_safety"), mock.patch.object(
                backend.jobs, "start", side_effect=paused_start,
            ):
                starter = threading.Thread(target=start_old_package)
                changer = threading.Thread(target=change_print_z)
                starter.start()
                try:
                    self.assertTrue(start_entered.wait(2))
                    changer.start()
                    self.assertTrue(change_attempted.wait(2))
                    self.assertFalse(
                        change_finished.wait(0.1),
                        "Print Z changed while an old-height motion job was being registered",
                    )
                finally:
                    release_start.set()
                    starter.join(timeout=2)
                    if changer.ident is not None:
                        changer.join(timeout=2)

            self.assertFalse(starter.is_alive())
            self.assertFalse(changer.is_alive())
            self.assertEqual(start_errors, [])
            self.assertEqual(len(change_errors), 1)
            self.assertIsInstance(change_errors[0], control_station.ApiError)
            self.assertAlmostEqual(backend.get_workcell_height()["print_zero_z_m"], table)

    def test_print_z_change_during_preflight_rejects_old_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            manifest = {
                "execution_id": "a" * 16,
                "print_bed_m": {"plane_z": table},
                "print_zero_z_m": table,
                "operation_mode": "production_print",
                "simulation": True,
                "block_count": 1,
            }

            def change_z_during_preflight() -> None:
                backend.set_print_z_draft({"print_zero_z_mm": (table + 0.010) * 1000})

            with mock.patch.object(backend, "_execution_package", return_value=(
                Path(directory) / "old-package.json", manifest,
            )), mock.patch.object(backend, "_require_action_and_safety"), mock.patch.object(
                backend, "_require_print_stack", side_effect=change_z_during_preflight,
            ), mock.patch.object(backend.jobs, "start") as start_job:
                with self.assertRaises(control_station.ApiError) as blocked:
                    backend.start_print_operation(
                        {"execution_id": "a" * 16}, mode="move-to-start",
                    )

            self.assertEqual(blocked.exception.code, "print_z_changed")
            start_job.assert_not_called()

    def test_changed_print_z_revokes_old_moveit_job_and_ready_preparation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            started = backend.start_gcode_moveit_review("print-z.gcode", GCODE)
            moveit_job_id = started["job"]["id"]
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and backend.jobs.get(moveit_job_id).state in {"queued", "running"}:
                time.sleep(0.02)
            self.assertEqual(backend.jobs.get(moveit_job_id).state, "succeeded")

            package = {
                "execution_id": "a" * 16,
                "source_sha256": control_station.hashlib.sha256(GCODE).hexdigest(),
                "execution_extent": "full",
                "moveit_job_id": moveit_job_id,
            }
            with mock.patch.object(backend, "prepare_print_execution", return_value=package):
                preparation = backend.start_print_preparation(
                    "print-z.gcode", GCODE, moveit_job_id, execution_extent="full",
                )
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    ready = backend.get_print_preparation(preparation["id"])
                    if ready["state"] == "succeeded":
                        break
                    time.sleep(0.01)
                self.assertEqual(ready["state"], "succeeded")

            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            backend.set_print_z_draft({"print_zero_z_mm": (table + 0.010) * 1000})

            with self.assertRaises(control_station.ApiError) as stale_job:
                backend.start_print_preparation(
                    "print-z.gcode", GCODE, moveit_job_id, execution_extent="full",
                )
            self.assertEqual(stale_job.exception.code, "print_z_changed")
            with self.assertRaises(control_station.ApiError) as stale_preparation:
                backend.get_print_preparation(preparation["id"])
            self.assertEqual(stale_preparation.exception.code, "print_z_changed")
            self.assertEqual(stale_preparation.exception.status, 412)

    def test_print_z_route_updates_without_changing_table(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            server = ThreadingHTTPServer(("127.0.0.1", 0), control_station.make_handler(backend))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
                request = urllib.request.Request(
                    f"http://127.0.0.1:{server.server_port}/api/workcell/print-z",
                    data=json.dumps({"print_zero_z_mm": (table + 0.003) * 1000}).encode(),
                    headers={"Content-Type": "application/json"}, method="PUT",
                )
                with urllib.request.urlopen(request, timeout=2) as response:
                    result = json.loads(response.read())
                self.assertTrue(result["ok"])
                self.assertFalse(result["print_z_candidate"])
                self.assertEqual(result["print_z_placement_mode"], "operator_selected")
                self.assertAlmostEqual(result["surface_z_m"], table)
                self.assertAlmostEqual(result["print_zero_z_m"], table + 0.003)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)

    def test_http_height_then_print_z_saves_both_independent_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workcell_path = Path(directory) / "workcell.json"
            workcell_path.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
            with mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", workcell_path):
                backend = self._backend(directory)
                server = ThreadingHTTPServer(("127.0.0.1", 0), control_station.make_handler(backend))
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                try:
                    base = f"http://127.0.0.1:{server.server_port}"
                    for route, payload in (
                        ("height", {"surface_z_mm": -20.0}),
                        ("print-z", {"print_zero_z_mm": -15.0}),
                    ):
                        request = urllib.request.Request(
                            f"{base}/api/workcell/{route}",
                            data=json.dumps(payload).encode(),
                            headers={"Content-Type": "application/json"},
                            method="PUT",
                        )
                        with urllib.request.urlopen(request, timeout=2) as response:
                            result = json.loads(response.read())
                        self.assertTrue(result["ok"])
                        self.assertIsNone(result["draft_surface_z_m"])
                    self.assertAlmostEqual(result["surface_z_m"], -0.020)
                    self.assertAlmostEqual(result["print_zero_z_m"], -0.015)
                    self.assertFalse(result["print_z_candidate"])
                    review = backend.build_gcode_path_review("print-z.gcode", GCODE)
                    self.assertAlmostEqual(review["collision_bed_m"]["plane_z"], -0.020)
                    self.assertAlmostEqual(review["placement_model"]["print_zero_z_m"], -0.015)
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join(timeout=2)

    def test_legacy_measured_z_binds_to_table_and_tcp_until_reselected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory, demo=False)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            candidate = round(table + 0.003, 6)
            installed = control_station.read_saved_calibration(backend._calibration_path)
            backend.set_print_z_draft({"print_zero_z_mm": candidate * 1000})
            backend._print_z_draft_m = candidate  # Exercise the still-supported legacy trial record.
            center = workcell_geometry.load_workcell_geometry()["tabletop"]["center_m"]
            backend._print_z_trial = {
                "print_zero_z_m": candidate,
                "table_surface_z_m": table,
                "profile_sha256": installed["profile_sha256"],
                "start_nozzle_m": {"x": center["x"], "y": center["y"], "z": candidate + 0.020},
                "target_nozzle_z_m": candidate + 0.010,
                "plan_job_id": "a" * 12,
                "execute_job_id": "b" * 12,
            }
            with mock.patch.object(backend, "require_online_mode"), mock.patch.object(
                backend, "_require_no_unconfirmed_print_stop",
            ), mock.patch.object(backend.jobs, "get", return_value=control_station.Job(
                id="b" * 12, kind="execute-print-z-trial", motion=True,
                command=[], state="succeeded",
            )), mock.patch.object(backend, "_stable_tool0_pose", return_value={}), mock.patch.object(
                backend, "_nozzle_position", return_value={
                    "x": center["x"], "y": center["y"], "z": candidate + 0.010,
                },
            ):
                result = backend.confirm_print_z_trial({
                    "operator_action": "confirm-print-z-trial",
                    "visual_clearance_confirmed": True,
                    "measured_clearance_mm": 10.0,
                })
            self.assertTrue(result["print_z_verified"])
            self.assertFalse(result["print_z_candidate"])
            self.assertAlmostEqual(result["surface_z_m"], table)
            self.assertAlmostEqual(result["print_zero_z_m"], candidate)
            self.assertTrue(backend._print_z_required_path.exists())
            review = backend.build_gcode_path_review("print-z.gcode", GCODE)
            self.assertAlmostEqual(review["collision_bed_m"]["plane_z"], table)
            self.assertFalse(review["print_z_candidate"])
            self.assertTrue(review["placement_model"]["execution_authoritative"])

            restarted = control_station.StationBackend(demo=False, data_dir=Path(directory))
            self.assertAlmostEqual(restarted.get_workcell_height()["print_zero_z_m"], candidate)
            with mock.patch.object(control_station, "read_saved_calibration", return_value={
                **installed, "profile_sha256": "f" * 64,
            }):
                stale_legacy = restarted.get_workcell_height()
            self.assertFalse(stale_legacy["print_z_verified"])
            self.assertTrue(stale_legacy["print_z_reverification_required"])
            # The new explicit path placement replaces the legacy physical-gap record.
            reverted = restarted.set_print_z_draft({"print_zero_z_mm": table * 1000})
            self.assertFalse(reverted["print_z_candidate"])
            self.assertEqual(reverted["print_z_placement_mode"], "operator_selected")
            with mock.patch.object(control_station, "read_saved_calibration", return_value={
                **installed, "profile_sha256": "f" * 64,
            }):
                stale = restarted.get_workcell_height()
            self.assertFalse(stale["print_z_verified"])
            self.assertFalse(stale["print_z_reverification_required"])
            self.assertFalse(stale["print_z_candidate"])

    def test_trial_plan_uses_physical_table_collision_plane(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory, demo=False)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            candidate = round(table + 0.003, 6)
            backend.set_print_z_draft({"print_zero_z_mm": candidate * 1000})
            backend._print_z_draft_m = candidate  # Legacy trial compatibility.
            center = workcell_geometry.load_workcell_geometry()["tabletop"]["center_m"]
            nozzle = {"x": center["x"], "y": center["y"], "z": candidate + 0.020}
            with mock.patch.object(backend, "require_online_mode"), mock.patch.object(
                backend, "_require_no_unconfirmed_print_stop",
            ), mock.patch.object(backend, "ros_status", return_value={
                "moveit_ready": True, "external_control": True, "unstable": False,
            }), mock.patch.object(backend, "_stable_tool0_pose", return_value={}), mock.patch.object(
                backend, "_nozzle_position", return_value=nozzle,
            ), mock.patch.object(backend.jobs, "start", return_value=control_station.Job(
                id="a" * 12, kind="plan-print-z-trial", motion=False,
                command=[], state="succeeded",
            )) as start_job:
                backend.start_print_z_trial({"operator_action": "plan-print-z-trial"}, execute=False)
            kwargs = start_job.call_args.kwargs
            self.assertEqual(kwargs["env"]["HEIGHT_TRIAL_SURFACE_Z_M"], str(table))
            self.assertEqual(kwargs["env"]["HEIGHT_TRIAL_TCP_Z_M"], str(
                control_station.read_saved_calibration(backend._calibration_path)
                ["calibration"]["tool_center_point_m"]["offset"]["z"],
            ))
            self.assertEqual(kwargs["env"]["SYNCHRONIZE_KLIPPER"], "false")
            self.assertFalse(kwargs["motion"])
            self.assertAlmostEqual(backend._print_z_trial["target_nozzle_z_m"], candidate + 0.010)

    def test_large_offset_cannot_plan_a_real_robot_trial(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory, demo=False)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            backend.set_print_z_draft({"print_zero_z_mm": (table + 0.007) * 1000})
            backend._print_z_draft_m = round(table + 0.007, 6)  # Legacy trial compatibility.
            with mock.patch.object(backend, "require_online_mode"), mock.patch.object(
                backend, "_require_no_unconfirmed_print_stop",
            ):
                with self.assertRaises(control_station.ApiError) as blocked:
                    backend.start_print_z_trial({"operator_action": "plan-print-z-trial"}, execute=False)
            self.assertEqual(blocked.exception.code, "print_z_tcp_remeasurement_required")
            self.assertIsNone(backend.jobs.active())

    def test_operator_selected_elevated_z_binds_moveit_review_and_execution_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            offset = 0.020
            selected = backend.set_print_z_draft({"print_zero_z_mm": offset * 1000})
            self.assertFalse(selected["print_z_verified"])
            self.assertEqual(selected["print_z_placement_mode"], "operator_selected")
            restarted = control_station.StationBackend(demo=True, data_dir=Path(directory))
            self.assertAlmostEqual(restarted.get_workcell_height()["print_zero_z_m"], offset)
            review = backend.build_gcode_path_review("print-z.gcode", GCODE)
            self.assertFalse(review["print_z_candidate"])
            self.assertTrue(review["placement_model"]["execution_authoritative"])
            started = backend.start_gcode_moveit_review("print-z.gcode", GCODE)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and backend.jobs.get(started["job"]["id"]).state in {"queued", "running"}:
                time.sleep(0.02)
            self.assertEqual(backend.jobs.get(started["job"]["id"]).state, "succeeded")
            package = backend.prepare_print_execution(
                "print-z.gcode", GCODE, started["job"]["id"], execution_extent="full",
            )
            self.assertAlmostEqual(package["print_bed_m"]["plane_z"], table)
            self.assertAlmostEqual(package["print_zero_z_m"], offset)
            self.assertEqual(package["review_sha256"], review["review_sha256"])

    def test_confirmation_requires_independent_gap_and_corrects_small_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory, demo=False)
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            candidate = round(table + 0.003, 6)
            installed = control_station.read_saved_calibration(backend._calibration_path)
            backend.set_print_z_draft({"print_zero_z_mm": candidate * 1000})
            backend._print_z_draft_m = candidate  # Legacy trial compatibility.
            center = workcell_geometry.load_workcell_geometry()["tabletop"]["center_m"]
            backend._print_z_trial = {
                "print_zero_z_m": candidate, "table_surface_z_m": table,
                "profile_sha256": installed["profile_sha256"],
                "start_nozzle_m": {"x": center["x"], "y": center["y"], "z": candidate + 0.020},
                "target_nozzle_z_m": candidate + 0.010,
                "execute_job_id": "b" * 12,
            }
            with mock.patch.object(backend, "require_online_mode"), mock.patch.object(
                backend, "_require_no_unconfirmed_print_stop",
            ), mock.patch.object(backend.jobs, "get", return_value=control_station.Job(
                id="b" * 12, kind="execute-print-z-trial", motion=True,
                command=[], state="succeeded",
            )), mock.patch.object(backend, "_stable_tool0_pose", return_value={}), mock.patch.object(
                backend, "_nozzle_position", return_value={
                    "x": center["x"], "y": center["y"], "z": candidate + 0.010,
                },
            ):
                with self.assertRaises(control_station.ApiError) as missing:
                    backend.confirm_print_z_trial({
                        "operator_action": "confirm-print-z-trial",
                        "visual_clearance_confirmed": True,
                    })
                self.assertEqual(missing.exception.code, "print_z_gap_measurement_required")
                result = backend.confirm_print_z_trial({
                    "operator_action": "confirm-print-z-trial",
                    "visual_clearance_confirmed": True,
                    "measured_clearance_mm": 9.8,
                })
            self.assertAlmostEqual(result["print_zero_z_m"], candidate + 0.0002)
            record = json.loads(backend._print_z_placement_path.read_text())
            self.assertEqual(record["measured_clearance_mm"], 9.8)

    def test_lost_verified_z_record_cannot_silently_restore_table_datum(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory)
            backend._print_z_required_path.write_text("print-z-verification-required\n")
            status = backend.get_workcell_height()
            self.assertTrue(status["print_z_reverification_required"])
            self.assertTrue(status["print_z_candidate"])
            self.assertTrue(backend.build_gcode_path_review("print-z.gcode", GCODE)["print_z_candidate"])
            with mock.patch.object(backend, "_execution_package", return_value=(
                Path(directory) / "old-package.json", {"print_bed_m": {"plane_z": status["surface_z_m"]}},
            )):
                with self.assertRaises(control_station.ApiError) as blocked:
                    backend.start_print_operation({"execution_id": "a" * 16}, mode="move-to-start")
            self.assertEqual(blocked.exception.code, "print_z_reverification_required")
            draft = backend.set_print_z_draft({"print_zero_z_mm": status["surface_z_m"] * 1000})
            self.assertIsNone(draft["print_z_draft_m"])
            self.assertFalse(draft["print_z_reverification_required"])
            self.assertEqual(draft["print_z_placement_mode"], "operator_selected")

    def test_trial_rejects_merely_downward_tilt_and_upward_move(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = self._backend(directory, demo=False)
            pose = {
                "position_m": {"x": -0.33, "y": 0.035, "z": 0.15},
                "orientation_rpy_deg": {"roll": 178.0, "pitch": 0.0, "yaw": 0.0},
            }
            with self.assertRaises(control_station.ApiError) as tilt:
                backend._nozzle_position(pose, require_print_orientation=True)
            self.assertEqual(tilt.exception.code, "print_orientation_required")
            table = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"]
            candidate = round(table + 0.003, 6)
            backend.set_print_z_draft({"print_zero_z_mm": candidate * 1000})
            backend._print_z_draft_m = candidate  # Legacy trial compatibility.
            center = workcell_geometry.load_workcell_geometry()["tabletop"]["center_m"]
            with mock.patch.object(backend, "require_online_mode"), mock.patch.object(
                backend, "_require_no_unconfirmed_print_stop",
            ), mock.patch.object(backend, "ros_status", return_value={
                "moveit_ready": True, "external_control": True, "unstable": False,
            }), mock.patch.object(backend, "_stable_tool0_pose", return_value={}), mock.patch.object(
                backend, "_nozzle_position", return_value={
                    "x": center["x"], "y": center["y"], "z": candidate + 0.004,
                },
            ):
                with self.assertRaises(control_station.ApiError):
                    backend.start_print_z_trial({"operator_action": "plan-print-z-trial"}, execute=False)
            self.assertIsNone(backend.jobs.active())
