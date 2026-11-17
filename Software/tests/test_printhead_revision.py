"""Tool replacement must revoke old motion authority while retaining offline review."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from Software import control_station, gcode_calibration, gcode_path_review, print_execution


def measured_profile() -> dict:
    profile = gcode_calibration.default_calibration()
    for name in ("orientation_verified", "tool_orientation_verified", "tcp_measured", "extrusion_ratio_measured"):
        profile["confirmations"][name] = True
    return profile


class PrintheadRevisionTests(unittest.TestCase):
    def test_new_cad_candidate_never_claims_physical_measurement(self):
        profile = gcode_calibration.default_calibration()
        self.assertEqual(profile["printhead_geometry_sha256"], gcode_calibration.printhead_geometry_sha256())
        self.assertFalse(profile["confirmations"]["tcp_measured"])
        self.assertFalse(profile["confirmations"]["extrusion_ratio_measured"])

    def test_legacy_measurements_are_retained_but_cannot_authorize_new_tool(self):
        old = measured_profile()
        old.pop("printhead_geometry_sha256")
        old["tool_center_point_m"]["offset"] = {"x": .0407, "y": .035, "z": .149525}
        normalized = gcode_calibration.normalize_calibration(old)
        self.assertEqual(normalized["tool_center_point_m"], old["tool_center_point_m"])
        self.assertEqual(normalized["klipper_extrusion"], old["klipper_extrusion"])
        for name in ("tcp_measured", "tool_orientation_verified", "extrusion_ratio_measured"):
            self.assertFalse(normalized["confirmations"][name])

    def test_explicit_migration_makes_exact_backup_and_is_idempotent(self):
        old = measured_profile()
        old.pop("printhead_geometry_sha256")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gcode_calibration.json"
            original = json.dumps({"source_sha256": "a" * 64, "calibration": old}, indent=4).encode()
            path.write_bytes(original)
            record = gcode_calibration.migrate_printhead_calibration(path)
            self.assertEqual(Path(record["backup_path"]).read_bytes(), original)
            self.assertEqual(gcode_calibration.migrate_printhead_calibration(path), record)
            migrated = json.loads(path.read_text())["calibration"]
            self.assertEqual(migrated["tool_center_point_m"], old["tool_center_point_m"])
            self.assertEqual(migrated["klipper_extrusion"], old["klipper_extrusion"])
            self.assertTrue(gcode_calibration.read_saved_calibration(path)["printhead_recalibration_required"])

    def test_old_setup_has_candidate_preview_but_moveit_remains_locked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old = measured_profile()
            old.pop("printhead_geometry_sha256")
            path = root / "gcode_calibration.json"
            path.write_text(json.dumps({"source_sha256": "a" * 64, "calibration": old}))
            backend = control_station.StationBackend(data_dir=root)
            content = b"G21\nG90\nM82\nG1 X0 Y0 Z1\nG1 X5 E1 F600\n"
            with mock.patch.object(control_station, "build_offline_robot_simulation", return_value={}):
                review = backend.build_gcode_path_review("candidate.gcode", content)
            self.assertTrue(review["simulation_only"])
            self.assertTrue(review["printhead_recalibration_required"])
            self.assertFalse(review["physical_calibration_complete"])
            self.assertEqual(review["calibration"]["tool_center_point_m"], gcode_calibration.default_calibration()["tool_center_point_m"])
            with mock.patch.object(backend.jobs, "start") as start:
                with self.assertRaises(control_station.ApiError) as raised:
                    backend.start_gcode_moveit_review("candidate.gcode", content)
                self.assertEqual(raised.exception.code, "printhead_recalibration_required")
                start.assert_not_called()

    def test_envelope_revision_invalidates_review_even_when_tcp_is_unchanged(self):
        profile = measured_profile()
        content = b"G21\nG90\nM82\nG1 X0 Y0 Z1\nG1 X5 E1 F600\n"
        with mock.patch.object(gcode_calibration, "printhead_geometry_sha256", return_value="a" * 64):
            first = gcode_path_review.build_offline_review("same.gcode", content, profile, allow_incomplete_simulation=True)
        with mock.patch.object(gcode_calibration, "printhead_geometry_sha256", return_value="b" * 64):
            second = gcode_path_review.build_offline_review("same.gcode", content, profile, allow_incomplete_simulation=True)
        self.assertEqual(first["tcp_offset_base_m"], second["tcp_offset_base_m"])
        self.assertNotEqual(first["review_sha256"], second["review_sha256"])

    def test_prepared_old_geometry_cannot_be_reopened(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.json"
            path.write_text(json.dumps({"version": 3, "planning_policy_version": print_execution.PLANNING_POLICY_VERSION,
                "extrusion_protocol": print_execution.EXTRUSION_PROTOCOL, "extruder": "extruder",
                "printhead_geometry_sha256": "0" * 64}))
            with self.assertRaisesRegex(print_execution.PrintExecutionError, "printhead geometry changed"):
                print_execution.read_execution_manifest(path)

    def test_only_explicit_new_extrusion_measurement_restores_its_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            profile = measured_profile()
            profile.pop("printhead_geometry_sha256")
            backend._calibration_path.write_text(json.dumps({"source_sha256": "a" * 64, "calibration": profile}))
            payload = {"operator_action": "save-nozzle-tcp", "offset_mm": {
                axis: value * 1000 for axis, value in gcode_calibration.default_calibration()["tool_center_point_m"]["offset"].items()
            }, "physically_measured": True, "orientation_verified": True}
            saved = backend.save_nozzle_tcp(payload)
            self.assertTrue(saved["installed_tcp_measured"])
            self.assertFalse(saved["extrusion_ratio_measured"])
            incomplete = gcode_calibration.read_saved_calibration(backend._calibration_path)["calibration"]
            analysis = control_station.parse_gcode("print.gcode", b"G21\nG90\nM82\nG1 X0 Y0 Z1\nG1 X5 E1 F600\n")
            self.assertFalse(gcode_calibration.validate_calibration(incomplete, analysis["bounds"], analysis["file"]["sha256"])["production_ready"])
            saved = backend.save_nozzle_tcp({**payload, "klipper_extrusion": {"units_per_e_mm": 1.25, "direction": "negative"},
                "extrusion_ratio_measured": True})
            self.assertTrue(saved["extrusion_ratio_measured"])
            calibrated = gcode_calibration.read_saved_calibration(backend._calibration_path)["calibration"]
            self.assertTrue(gcode_calibration.validate_calibration(calibrated, analysis["bounds"], analysis["file"]["sha256"])["production_ready"])
            before = backend._calibration_path.read_bytes()
            with self.assertRaises(control_station.ApiError):
                backend.save_nozzle_tcp({**payload, "klipper_extrusion": {"units_per_e_mm": 0, "direction": "negative"},
                    "extrusion_ratio_measured": True})
            self.assertEqual(backend._calibration_path.read_bytes(), before)

    def test_async_review_key_changes_for_geometry_only_update(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            with mock.patch.object(control_station, "printhead_geometry_sha256", return_value="a" * 64):
                first = backend._gcode_review_profile_snapshot("c" * 64, simulation_only=False)[0]
            with mock.patch.object(control_station, "printhead_geometry_sha256", return_value="b" * 64):
                second = backend._gcode_review_profile_snapshot("c" * 64, simulation_only=False)[0]
            self.assertNotEqual(first, second)

    def test_windows_height_trial_uses_native_runtime_and_preserves_geometry_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = control_station.StationBackend(data_dir=Path(directory))
            backend._height_draft_m = -.010
            pose = {"position_m": {"x": -.3, "y": 0, "z": .2},
                    "orientation_rpy_deg": {"roll": 180, "pitch": 0, "yaw": 0}}
            nozzle = {"x": -.3, "y": 0, "z": .03}
            with mock.patch.object(control_station.platform, "system", return_value="Windows"), \
                 mock.patch.object(control_station.shutil, "which", return_value=None), \
                 mock.patch.object(backend, "_height_trial_target", return_value=(pose, nozzle, -.02)), \
                 mock.patch.object(backend.jobs, "start", return_value=SimpleNamespace(id="plan")) as start:
                backend.start_workcell_height_trial({"operator_action": "plan-height-trial"}, execute=False)
                command = start.call_args.args[1]
                self.assertEqual(command, [control_station.sys.executable,
                                         str(control_station.WINDOWS_RUNTIME), "cartesian-move"])
                environment = start.call_args.kwargs["env"]
            for name in ("AXES", "DISTANCE_M", "EXECUTE", "SYNCHRONIZE_KLIPPER", "HEIGHT_TRIAL_MODE",
                         "HEIGHT_TRIAL_TCP_X_M", "HEIGHT_TRIAL_TCP_Y_M", "HEIGHT_TRIAL_TCP_Z_M",
                         "HEIGHT_TRIAL_SURFACE_Z_M", "PRINTHEAD_GEOMETRY_SHA256"):
                self.assertIn(name, environment)
            self.assertEqual(environment["PRINTHEAD_GEOMETRY_SHA256"], control_station.printhead_geometry_sha256())
            self.assertEqual(environment["SYNCHRONIZE_KLIPPER"], "false")
            self.assertEqual(environment["AXES"], "z")
            self.assertEqual(environment["DISTANCE_M"], "-0.02")
            self.assertEqual(environment["EXECUTE"], "false")
            self.assertEqual(environment["HEIGHT_TRIAL_MODE"], "true")
            tcp = control_station.load_printhead_geometry()["tool0_to_nozzle_m"]
            for axis in ("x", "y", "z"):
                self.assertEqual(float(environment[f"HEIGHT_TRIAL_TCP_{axis.upper()}_M"]), tcp[axis])

    def test_first_installation_migrates_an_unmeasured_new_tool_preview(self):
        # Clean source/packages intentionally exclude the developer's saved
        # calibration. Construct the unmeasured legacy fixture locally.
        template = {"source_sha256": "a" * 64, "calibration": gcode_calibration.default_calibration()}
        template["calibration"]["confirmations"] = {
            name: False for name in template["calibration"]["confirmations"]
        }
        self.assertEqual(template["calibration"]["operation_mode"], "production_print")
        self.assertTrue(all(value is False for value in template["calibration"]["confirmations"].values()))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            legacy, user_data = root / "bundled", root / "user"
            legacy.mkdir()
            (legacy / "gcode_calibration.json").write_text(json.dumps(template), encoding="utf-8")
            self.assertEqual(control_station.migrate_legacy_state(legacy, user_data), ["gcode_calibration.json"])
            backend = control_station.StationBackend(data_dir=user_data)
            content = b"G21\nG90\nM82\nG1 X0 Y0 Z1\nG1 X5 E1 F600\n"
            with mock.patch.object(control_station, "build_offline_robot_simulation", return_value={}):
                review = backend.build_gcode_path_review("first.gcode", content)
            self.assertTrue(review["simulation_only"])
            self.assertFalse(review["physical_calibration_complete"])
            self.assertEqual(review["calibration"]["tool_center_point_m"], gcode_calibration.default_calibration()["tool_center_point_m"])
            self.assertEqual(review["printhead_geometry_sha256"], gcode_calibration.printhead_geometry_sha256())
            with mock.patch.object(backend.jobs, "start") as start:
                with self.assertRaises(control_station.ApiError):
                    backend.start_gcode_moveit_review("first.gcode", content)
                start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
