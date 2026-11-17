"""Offline regression checks for the guarded tabletop-height workflow."""

from __future__ import annotations

import copy
import json
import re
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from Software import control_station, gcode_calibration, workcell_geometry


class WorkcellHeightTrialTests(unittest.TestCase):
    def setUp(self) -> None:
        # Height settings now commit immediately; never let a test edit the
        # workstation's installed collision geometry.
        self._geometry_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self._geometry_directory.cleanup)
        geometry_path = Path(self._geometry_directory.name) / "workcell.json"
        geometry_path.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
        geometry_patch = mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", geometry_path)
        geometry_patch.start()
        self.addCleanup(geometry_patch.stop)

    def test_unchanged_tabletop_proposal_does_not_lock_a_new_print_z(self) -> None:
        """Selecting +40 mm print Z must not inherit a no-op physical-table draft."""
        content = b"G21\nG90\nM83\nG1 X0 Y0 Z0.2 E1 F600\nG1 X10 Y0 E1\n"
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(demo=True, data_dir=Path(directory))
            profile = gcode_calibration.default_calibration()
            profile["confirmations"].update({
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "tcp_measured": True,
            })
            analysis = control_station.parse_gcode("height.gcode", content)
            backend.save_gcode_calibration({
                "calibration": profile,
                "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            })

            backend.set_print_z_draft({"print_zero_z_mm": 50.0})
            review_at_50 = backend.build_gcode_path_review("height.gcode", content)
            backend.set_print_z_draft({"print_zero_z_mm": 40.0})
            unchanged_table_mm = workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"] * 1000
            state = backend.set_workcell_height_draft({"surface_z_mm": unchanged_table_mm})
            review = backend.build_gcode_path_review("height.gcode", content)

            self.assertAlmostEqual(state["print_zero_z_m"], 0.04)
            self.assertNotEqual(review["review_sha256"], review_at_50["review_sha256"])
            self.assertIsNone(state["draft_surface_z_m"])
            self.assertFalse(review["tabletop_height_candidate"])
            self.assertTrue(review["placement_model"]["execution_authoritative"])

    def test_async_review_cache_is_scoped_to_saved_height(self) -> None:
        content = b"G21\nG90\nM83\nG1 X0 Y0 Z0.2 E1 F600\nG1 X10 Y0 E1\n"
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(demo=True, data_dir=Path(directory))
            profile = gcode_calibration.default_calibration()
            profile["confirmations"].update({
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "tcp_measured": True,
            })
            analysis = control_station.parse_gcode("height.gcode", content)
            backend.save_gcode_calibration({
                "calibration": profile, "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            })

            def finish(job_id: str) -> dict:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    result = backend.get_gcode_review_job(job_id)
                    if result["state"] not in {"queued", "running"}:
                        return result
                    time.sleep(0.02)
                self.fail("Asynchronous 3D review did not finish")

            installed = backend.start_gcode_review_job("height.gcode", content, simulation_only=False)
            installed_result = finish(installed["id"])
            self.assertEqual(installed_result["state"], "succeeded")
            backend.set_workcell_height_draft({"surface_z_mm": 20})
            candidate = backend.start_gcode_review_job("height.gcode", content, simulation_only=False)
            candidate_result = finish(candidate["id"])
            self.assertEqual(candidate_result["state"], "succeeded")
            self.assertNotEqual(candidate["id"], installed["id"])
            self.assertAlmostEqual(candidate_result["review"]["collision_bed_m"]["plane_z"], 0.020)
            self.assertFalse(candidate_result["review"]["tabletop_height_candidate"])
            self.assertTrue(candidate_result["review"]["placement_model"]["execution_authoritative"])

    def test_saved_height_rebuilds_review_and_survives_discard(self) -> None:
        content = b"G21\nG90\nM83\nG1 X0 Y0 Z0.2 E1 F600\nG1 X10 Y0 E1\n"
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(demo=True, data_dir=Path(directory))
            profile = gcode_calibration.default_calibration()
            profile["confirmations"].update({
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "tcp_measured": True,
            })
            analysis = control_station.parse_gcode("height.gcode", content)
            backend.save_gcode_calibration({
                "calibration": profile,
                "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            })
            installed = backend.build_gcode_path_review("height.gcode", content)
            backend.set_workcell_height_draft({"surface_z_mm": 20})
            candidate = backend.build_gcode_path_review("height.gcode", content)

            self.assertAlmostEqual(candidate["collision_bed_m"]["plane_z"], 0.020)
            self.assertAlmostEqual(candidate["placement_model"]["table_surface_z_m"], 0.020)
            self.assertAlmostEqual(
                candidate["path"]["segments"][0]["z0"]
                - installed["path"]["segments"][0]["z0"],
                0.020 - installed["collision_bed_m"]["plane_z"],
            )
            self.assertFalse(candidate["tabletop_height_candidate"])
            self.assertTrue(candidate["placement_model"]["execution_authoritative"])
            self.assertNotEqual(candidate["review_sha256"], installed["review_sha256"])
            self.assertAlmostEqual(
                workcell_geometry.load_workcell_geometry()["tabletop"]["surface_z_m"], 0.020,
            )
            with mock.patch.object(backend.jobs, "start") as start_job:
                start_job.side_effect = lambda kind, command, **_kwargs: control_station.Job(
                    id="b" * 12, kind=kind, motion=False, command=command, state="succeeded",
                )
                started = backend.start_gcode_moveit_review("height.gcode", content)
            self.assertFalse(started["tabletop_height_candidate"])
            self.assertEqual(start_job.call_args.args[1][-3], "0.02")

            backend.set_workcell_height_draft({"discard": True})
            restored = backend.build_gcode_path_review("height.gcode", content)
            self.assertEqual(restored["review_sha256"], candidate["review_sha256"])

    def test_ros_collision_box_matches_cad_bounds_and_mount(self) -> None:
        root = Path(__file__).resolve().parents[2]
        ros = root / "workspaces" / "ur_ros2_ws" / "src"
        package = ros / "ur3e_printhead_description"
        geometry = control_station.load_printhead_geometry()
        validation = json.loads((package / "meshes" / "visual" / (Path(geometry["source"]).stem + ".validation.json")).read_text())
        minimum = validation["geometry"]["bounds_minimum_cad_m"]
        maximum = validation["geometry"]["bounds_maximum_cad_m"]
        rotation = gcode_calibration._tool_rotation_matrix({"coordinate_transform": {
            "tool_orientation_rpy_deg": geometry["tool0_to_cad_rpy_deg"]}})
        corners = [(x, y, z) for x in (minimum[0], maximum[0])
                   for y in (minimum[1], maximum[1]) for z in (minimum[2], maximum[2])]
        transformed = [[geometry["tool0_to_cad_origin_m"][axis] + sum(rotation[row][column] * point[column]
                       for column in range(3)) for row, axis in enumerate(("x", "y", "z"))] for point in corners]
        box = geometry["collision_envelope_tool0_m"]
        for row, axis in enumerate(("x", "y", "z")):
            low, high = min(point[row] for point in transformed), max(point[row] for point in transformed)
            self.assertAlmostEqual(box["size"][axis], high - low, places=6)
            self.assertAlmostEqual(box["center"][axis], (high + low) / 2, places=6)

    def test_legacy_tool0_profile_cannot_plan_with_installed_head(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            old = gcode_calibration.default_calibration()
            old["operation_mode"] = "tool0_motion_test"
            old["tool_center_point_m"] = {
                "offset": {"x": 0, "y": 0, "z": 0}, "collision_radius_m": 0,
            }
            gcode_calibration.save_installed_calibration_draft(
                backend._calibration_path, old, "a" * 64,
            )
            analysis = {"file": {"sha256": "b" * 64}, "bounds": {}}
            with self.assertRaises(control_station.ApiError) as rejected:
                backend._automatic_calibration_for_analysis(analysis)
            self.assertEqual(rejected.exception.code, "printhead_tcp_required")

    def test_nozzle_profile_replaces_legacy_tool0_only_after_explicit_setup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            candidate = backend.get_workcell_height()["nozzle"]["tool0_to_nozzle_m"]
            offset_mm = {axis: value * 1000 for axis, value in candidate.items()}
            with self.assertRaises(control_station.ApiError):
                backend.save_nozzle_tcp({
                    "operator_action": "save-nozzle-tcp", "offset_mm": offset_mm,
                    "physically_measured": True,
                })
            staged = backend.save_nozzle_tcp({
                "operator_action": "save-nozzle-tcp", "offset_mm": offset_mm,
                "physically_measured": False, "orientation_verified": False,
            })
            self.assertFalse(staged["installed_tcp_measured"])
            saved = gcode_calibration.read_saved_calibration(backend._calibration_path)
            self.assertEqual(saved["calibration"]["operation_mode"], "production_print")
            self.assertEqual(saved["calibration"]["tool_center_point_m"]["offset"], candidate)
            self.assertFalse(saved["calibration"]["confirmations"]["tcp_measured"])
            measured = backend.save_nozzle_tcp({
                "operator_action": "save-nozzle-tcp", "offset_mm": offset_mm,
                "physically_measured": True, "orientation_verified": True,
            })
            self.assertTrue(measured["installed_tcp_measured"])

    def test_verified_height_is_atomic_and_calibration_reads_new_z(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "workcell.json"
            source.write_bytes(workcell_geometry.WORKCELL_GEOMETRY_PATH.read_bytes())
            updated = workcell_geometry.save_surface_height_m(-0.010, source)
            self.assertEqual(updated["tabletop"]["surface_z_m"], -0.010)
            with mock.patch.object(workcell_geometry, "WORKCELL_GEOMETRY_PATH", source):
                calibration = gcode_calibration.default_calibration()
                self.assertEqual(calibration["print_bed_m"]["plane_z"], -0.010)
                self.assertEqual(calibration["coordinate_transform"]["origin_m"]["z"], -0.010)

    def test_trial_is_plan_first_ur_only_and_requires_live_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            backend.set_operating_mode({"mode": "online"})
            tcp = backend.get_workcell_height()["nozzle"]["tool0_to_nozzle_m"]
            start = {
                "position_m": {"x": -0.2925 - tcp["x"], "y": tcp["y"], "z": tcp["z"] + 0.011},
                "orientation_rpy_deg": {"roll": 180.0, "pitch": 0.0, "yaw": 0.0},
            }
            final = copy.deepcopy(start)
            final["position_m"]["z"] = tcp["z"]
            current_pose = {"value": start}
            jobs: dict[str, control_station.Job] = {}
            launches: list[dict[str, str]] = []

            def fake_start(kind: str, command: list[str], **kwargs: object) -> control_station.Job:
                job = control_station.Job(
                    id=f"job{len(jobs)}", kind=kind,
                    motion=bool(kwargs["motion"]), command=command,
                    state="succeeded",
                )
                launches.append(kwargs["env"])
                jobs[job.id] = job
                if kind == "execute-height-trial":
                    current_pose["value"] = final
                return job

            with mock.patch.object(backend, "print_interlock_status", return_value={"locked": False}), \
                 mock.patch.object(backend, "ros_status", return_value={"moveit_ready": True, "external_control": True, "unstable": False}), \
                 mock.patch.object(backend, "get_current_tool0_pose", side_effect=lambda: current_pose["value"]), \
                 mock.patch.object(backend.jobs, "active", return_value=None), \
                 mock.patch.object(backend.jobs, "start", side_effect=fake_start), \
                 mock.patch.object(backend.jobs, "get", side_effect=lambda job_id: jobs[job_id]), \
                 mock.patch.object(control_station, "save_surface_height_m") as save_height:
                # Exercise the retained legacy trial endpoint without turning
                # an operator-entered tabletop coordinate into a motion permit.
                backend._height_draft_m = -0.010
                with self.assertRaises(control_station.ApiError):
                    backend.start_workcell_height_trial({"operator_action": "execute-height-trial"}, execute=True)
                plan = backend.start_workcell_height_trial({"operator_action": "plan-height-trial"}, execute=False)
                self.assertFalse(plan.motion)
                self.assertEqual(launches[0]["EXECUTE"], "false")
                self.assertEqual(launches[0]["SYNCHRONIZE_KLIPPER"], "false")
                with self.assertRaises(control_station.ApiError):
                    backend.start_workcell_height_trial({"operator_action": "execute-height-trial"}, execute=True)
                safety = {name: True for name in (
                    "emergency_stop", "workspace_clear", "reviewed_low_speed",
                    "wired_network", "external_control_playing", "installed_nozzle_verified",
                )}
                with self.assertRaises(control_station.ApiError) as unmeasured:
                    backend.start_workcell_height_trial(
                        {"operator_action": "execute-height-trial", **safety}, execute=True,
                    )
                self.assertIn("physically measured nozzle TCP", str(unmeasured.exception))
                backend.save_nozzle_tcp({
                    "operator_action": "save-nozzle-tcp",
                    "offset_mm": {axis: value * 1000 for axis, value in
                                  backend.get_workcell_height()["nozzle"]["tool0_to_nozzle_m"].items()},
                    "physically_measured": True, "orientation_verified": True,
                })
                backend.start_workcell_height_trial({"operator_action": "plan-height-trial"}, execute=False)
                execute = backend.start_workcell_height_trial(
                    {"operator_action": "execute-height-trial", **safety}, execute=True,
                )
                self.assertTrue(execute.motion)
                self.assertEqual(launches[2]["EXECUTE"], "true")
                self.assertEqual(launches[2]["SYNCHRONIZE_KLIPPER"], "false")
                with self.assertRaises(control_station.ApiError):
                    backend.confirm_workcell_height({"operator_action": "confirm-height-trial"})
                with self.assertRaises(control_station.ApiError) as missing_gap:
                    backend.confirm_workcell_height({
                        "operator_action": "confirm-height-trial",
                        "visual_clearance_confirmed": True,
                    })
                self.assertEqual(missing_gap.exception.code, "table_gap_measurement_required")
                backend.confirm_workcell_height({
                    "operator_action": "confirm-height-trial",
                    "visual_clearance_confirmed": True,
                    "measured_clearance_mm": 10.0,
                })
                save_height.assert_called_once_with(-0.010)


if __name__ == "__main__":
    unittest.main()
