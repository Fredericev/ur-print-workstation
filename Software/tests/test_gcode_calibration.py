from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "gcode_calibration.py"
SPEC = importlib.util.spec_from_file_location("gcode_calibration", MODULE_PATH)
assert SPEC and SPEC.loader
calibration = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = calibration
SPEC.loader.exec_module(calibration)


SOURCE_SHA256 = "a" * 64
BOUNDS = {
    "minimum": {"x": 0, "y": 0, "z": 0},
    "maximum": {"x": 100, "y": 200, "z": 30},
    "units": "mm",
}


def ready_profile() -> dict:
    profile = calibration.default_calibration()
    profile["coordinate_transform"]["origin_m"] = {
        "x": 0.4,
        "y": -0.2,
        "z": 0.1,
    }
    profile["print_bed_m"] = {
        "plane_z": 0.1,
        "minimum": {"x": 0.1, "y": -0.3},
        "maximum": {"x": 0.6, "y": 0.1},
        "edge_margin_m": 0.01,
    }
    profile["tool_center_point_m"] = {
        "offset": {"x": 0.0, "y": 0.0, "z": 0.05},
        "collision_radius_m": 0.01,
    }
    profile["confirmations"] = {
        "origin_measured": True,
        "orientation_verified": True,
        "tool_orientation_verified": True,
        "extrusion_ratio_measured": True,
        "tcp_measured": True,
        "print_bed_measured": True,
    }
    return profile


class GCodeCalibrationTests(unittest.TestCase):
    def test_table_gate_checks_the_robot_execution_window_not_printer_parking(self) -> None:
        profile = ready_profile()
        bounds = {
            "minimum": {"x": 0.0, "y": 0.0, "z": 0.0},
            "maximum": {"x": 500.0, "y": 500.0, "z": 30.0},
            "placement": {
                "minimum": {"x": 10.0, "y": 20.0, "z": 0.2},
                "maximum": {"x": 30.0, "y": 40.0, "z": 0.4},
                "source": "print_path",
            },
            "execution": {
                "minimum": {"x": 10.0, "y": 20.0, "z": 0.2},
                "maximum": {"x": 35.0, "y": 45.0, "z": 2.0},
                "source": "first_to_last_print",
                "first_sequence": 3,
                "last_sequence": 12,
                "excluded_lead_in_move_count": 2,
                "excluded_shutdown_move_count": 1,
            },
        }

        result = calibration.validate_calibration(profile, bounds, SOURCE_SHA256)

        self.assertTrue(result["ready_to_save"])
        self.assertTrue(result["within_print_bed"])
        self.assertEqual(
            result["source_bounds_mm"]["maximum"],
            {"x": 500.0, "y": 500.0, "z": 30.0},
        )
        self.assertEqual(
            result["execution_bounds_mm"]["maximum"],
            {"x": 35.0, "y": 45.0, "z": 2.0},
        )

    def test_fixed_table_and_moveit_replace_configurable_work_envelope(self) -> None:
        profile = ready_profile()

        result = calibration.validate_calibration(profile, BOUNDS, SOURCE_SHA256)

        self.assertNotIn("work_envelope_m", calibration.default_calibration())
        self.assertNotIn("work_envelope_m", result["calibration"])
        self.assertNotIn("within_work_envelope", result)
        self.assertNotIn(
            "work_envelope",
            {gate["code"] for gate in result["gates"]},
        )

    def test_each_print_is_centered_on_the_confirmed_fixed_table(self) -> None:
        profile = ready_profile()
        profile["coordinate_transform"]["rotation_z_deg"] = 0.0
        profile["coordinate_transform"]["origin_m"] = {
            "x": 4.0,
            "y": 4.0,
            "z": 4.0,
        }
        profile["print_bed_m"] = {
            "plane_z": 1.0,
            "minimum": {"x": 1.0, "y": 1.0},
            "maximum": {"x": 2.0, "y": 2.0},
            "edge_margin_m": 0.2,
        }
        profile["confirmations"]["origin_measured"] = False
        profile["confirmations"]["print_bed_measured"] = False
        bounds = {
            "minimum": {"x": 0.0, "y": 0.0, "z": 0.0},
            "maximum": {"x": 200.0, "y": 200.0, "z": 0.4},
            "placement": {
                "minimum": {"x": 10.0, "y": 20.0, "z": 0.2},
                "maximum": {"x": 30.0, "y": 40.0, "z": 0.4},
                "dimensions": {"x": 20.0, "y": 20.0, "z": 0.2},
                "source": "print_path",
            },
        }

        result = calibration.validate_calibration(profile, bounds, SOURCE_SHA256)

        placed = result["calibration"]
        table_z = calibration.load_workcell_geometry()["tabletop"]["surface_z_m"]
        self.assertEqual(placed["version"], 5)
        self.assertEqual(placed["placement_mode"], "center_on_fixed_table")
        self.assertEqual(
            placed["coordinate_transform"]["origin_m"],
            {"x": -0.3125, "y": -0.03, "z": table_z},
        )
        self.assertEqual(
            placed["print_bed_m"],
            {
                "plane_z": table_z,
                "minimum": {"x": -0.5175, "y": -0.27},
                "maximum": {"x": -0.0675, "y": 0.27},
                "edge_margin_m": 0.01,
            },
        )
        self.assertTrue(placed["confirmations"]["origin_measured"])
        self.assertTrue(placed["confirmations"]["print_bed_measured"])
        self.assertEqual(
            result["placement"]["print_center_m"],
            {"x": -0.2925, "y": 0.0},
        )
        self.assertEqual(result["placement"]["bounds_source"], "print_path")

    def test_legacy_tabletop_profile_migrates_to_fixed_placement(self) -> None:
        profile = ready_profile()
        profile["version"] = 3
        profile["operation_mode"] = "tool0_tabletop_test"
        profile["coordinate_transform"]["origin_m"]["z"] = profile[
            "print_bed_m"
        ]["plane_z"]
        profile["confirmations"]["tool0_reference_verified"] = True

        result = calibration.validate_calibration(profile, BOUNDS, SOURCE_SHA256)

        self.assertEqual(result["calibration"]["version"], 5)
        self.assertEqual(result["operation_mode"], "tool0_motion_test")
        self.assertNotIn("tool0_pose_recapture_required", result["calibration"])
        self.assertTrue(result["calibration"]["confirmations"]["origin_measured"])
        self.assertFalse(
            result["calibration"]["confirmations"]["tool_orientation_verified"]
        )
        self.assertFalse(
            result["calibration"]["confirmations"]["tool0_reference_verified"]
        )
        self.assertFalse(result["ready_to_save"])
        self.assertEqual(result["gates"][0]["code"], "coordinate_transform")
        self.assertEqual(result["gates"][0]["status"], "required")
        self.assertEqual(result["tcp_offset_base_m"], {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_migrated_tabletop_profile_can_save_only_after_safe_tool0_recapture(self) -> None:
        profile = ready_profile()
        profile["version"] = 3
        profile["operation_mode"] = "tool0_tabletop_test"
        profile["confirmations"]["tool0_reference_verified"] = True
        migrated = calibration.normalize_calibration(profile)
        migrated["coordinate_transform"]["origin_m"] = {
            "x": -0.35,
            "y": 0.03,
            "z": 0.15,
        }
        migrated["print_bed_m"]["plane_z"] = -0.014
        migrated["confirmations"].update(
            {
                "origin_measured": True,
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "print_bed_measured": True,
                "tool0_reference_verified": True,
            }
        )

        result = calibration.validate_calibration(migrated, BOUNDS, SOURCE_SHA256)

        self.assertEqual(result["operation_mode"], "tool0_motion_test")
        self.assertTrue(result["ready_to_save"])
        self.assertTrue(result["motion_test_ready"])

    def test_tool0_motion_test_uses_zero_tcp_without_added_clearance(self) -> None:
        profile = calibration.default_calibration()
        profile["operation_mode"] = "tool0_motion_test"
        profile["coordinate_transform"]["origin_m"] = {
            "x": -0.35,
            "y": 0.03,
            "z": 0.15,
        }
        profile["tool_center_point_m"] = {
            "offset": {"x": 0.1, "y": 0.0, "z": 0.2},
            "collision_radius_m": 0.03,
        }
        profile["print_bed_m"]["plane_z"] = -0.014
        profile["confirmations"].update(
            {
                "origin_measured": True,
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "print_bed_measured": True,
                "tool0_reference_verified": True,
            }
        )

        result = calibration.validate_calibration(profile, BOUNDS, SOURCE_SHA256)

        self.assertTrue(result["ready_to_save"])
        self.assertTrue(result["motion_test_ready"])
        self.assertFalse(result["production_ready"])
        self.assertEqual(result["tcp_offset_base_m"], {"x": 0.0, "y": 0.0, "z": 0.0})
        self.assertEqual(result["air_run_check"]["clearance_m"], 0.0)
        self.assertEqual(result["air_run_check"]["minimum_clearance_m"], 0.0)
        self.assertFalse(result["air_run_check"]["heating_allowed"])
        self.assertEqual(
            [gate["code"] for gate in result["gates"]],
            [
                "coordinate_transform",
                "extrusion_calibration",
                "tool0_reference",
                "collision_check",
            ],
        )

    def test_tool0_motion_test_allows_path_on_measured_table_plane(self) -> None:
        profile = calibration.default_calibration()
        profile["operation_mode"] = "tool0_motion_test"
        profile["coordinate_transform"]["origin_m"] = {
            "x": 0.4,
            "y": 0.0,
            "z": 0.0,
        }
        profile["print_bed_m"]["plane_z"] = 0.0
        profile["confirmations"].update(
            {
                "origin_measured": True,
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "print_bed_measured": True,
                "tool0_reference_verified": True,
            }
        )

        result = calibration.validate_calibration(profile, BOUNDS, SOURCE_SHA256)

        self.assertTrue(result["ready_to_save"])
        self.assertTrue(result["air_run_check"]["passed"])
        self.assertEqual(result["air_run_check"]["clearance_m"], 0.0)

    def test_tool0_motion_test_rejects_path_below_measured_table_plane(self) -> None:
        profile = calibration.default_calibration()
        profile["operation_mode"] = "tool0_motion_test"
        profile["confirmations"].update(
            {
                "origin_measured": True,
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "print_bed_measured": True,
                "tool0_reference_verified": True,
            }
        )

        below_table_bounds = json.loads(json.dumps(BOUNDS))
        below_table_bounds["minimum"]["z"] = -1.0
        result = calibration.validate_calibration(
            profile, below_table_bounds, SOURCE_SHA256
        )

        self.assertFalse(result["ready_to_save"])
        self.assertFalse(result["air_run_check"]["passed"])

    def test_transforms_all_bounds_into_ur_base_metres(self) -> None:
        profile = ready_profile()
        profile["coordinate_transform"]["origin_m"] = {
            "x": 0.4,
            "y": -0.2,
            "z": 0.1,
        }

        result = calibration.validate_calibration(
            profile,
            BOUNDS,
            SOURCE_SHA256,
        )

        table_z = calibration.load_workcell_geometry()["tabletop"]["surface_z_m"]
        self.assertEqual(result["status"], "ready")
        self.assertEqual(
            result["transformed_bounds_m"]["minimum"],
            {"x": -0.3425, "y": -0.1, "z": round(table_z + 0.05, 6)},
        )
        self.assertEqual(
            result["transformed_bounds_m"]["maximum"],
            {"x": -0.2425, "y": 0.1, "z": round(table_z + 0.08, 6)},
        )
        self.assertTrue(result["within_print_bed"])
        self.assertTrue(result["print_bed_surface_aligned"])
        self.assertTrue(result["execution_locked"])

    def test_rotation_uses_all_eight_corners(self) -> None:
        profile = ready_profile()
        profile["coordinate_transform"]["rotation_z_deg"] = 90

        result = calibration.validate_calibration(
            profile,
            BOUNDS,
            SOURCE_SHA256,
        )

        transformed = result["transformed_bounds_m"]
        self.assertEqual(transformed["minimum"]["x"], -0.3925)
        self.assertEqual(transformed["maximum"]["x"], -0.1925)
        self.assertEqual(transformed["minimum"]["y"], -0.05)
        self.assertEqual(transformed["maximum"]["y"], 0.05)

    def test_requires_operator_confirmations_before_save(self) -> None:
        result = calibration.validate_calibration(
            calibration.default_calibration(),
            BOUNDS,
            SOURCE_SHA256,
        )

        self.assertEqual(result["status"], "incomplete")
        self.assertFalse(result["ready_to_save"])
    def test_replaces_submitted_origin_with_automatic_centered_placement(self) -> None:
        profile = ready_profile()
        profile["coordinate_transform"]["origin_m"] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }

        result = calibration.validate_calibration(
            profile,
            BOUNDS,
            SOURCE_SHA256,
        )

        table_z = calibration.load_workcell_geometry()["tabletop"]["surface_z_m"]
        self.assertTrue(result["ready_to_save"])
        self.assertTrue(result["origin_clear_of_robot_base"])
        self.assertEqual(
            result["calibration"]["coordinate_transform"]["origin_m"],
            {"x": -0.3425, "y": -0.1, "z": table_z},
        )

    def test_rejects_invalid_hash(self) -> None:
        with self.assertRaises(calibration.CalibrationValidationError):
            calibration.validate_calibration(
                ready_profile(),
                BOUNDS,
                "not-a-checksum",
            )

    def test_print_bed_gate_fails_when_centered_path_is_too_large(self) -> None:
        profile = ready_profile()
        oversized_bounds = {
            "minimum": {"x": 0.0, "y": 0.0, "z": 0.0},
            "maximum": {"x": 500.0, "y": 200.0, "z": 30.0},
        }

        result = calibration.validate_calibration(
            profile, oversized_bounds, SOURCE_SHA256
        )

        self.assertFalse(result["within_print_bed"])
        self.assertFalse(result["ready_to_save"])
        self.assertEqual(result["gates"][3]["status"], "failed")

    def test_print_bed_gate_fails_when_path_goes_below_fixed_surface(self) -> None:
        profile = ready_profile()
        below_table_bounds = json.loads(json.dumps(BOUNDS))
        below_table_bounds["minimum"]["z"] = -1.0

        result = calibration.validate_calibration(
            profile, below_table_bounds, SOURCE_SHA256
        )

        self.assertFalse(result["print_bed_surface_aligned"])
        self.assertLess(result["print_bed_check"]["surface_gap_m"], -0.0005)
        self.assertEqual(result["gates"][3]["status"], "failed")

    def test_relative_test_is_blocked_outside_bed_and_passes_inside(self) -> None:
        profile = ready_profile()
        passed = calibration.validate_relative_motion_over_print_bed(
            profile,
            {"x": -0.4, "y": -0.2, "z": 0.2},
            "x,y",
            0.05,
        )
        self.assertEqual(passed["status"], "passed")
        self.assertFalse(passed["collision_checked"])

        with self.assertRaises(calibration.CalibrationNotReadyError):
            calibration.validate_relative_motion_over_print_bed(
                profile,
                {"x": -0.06, "y": -0.2, "z": 0.2},
                "x",
                0.05,
            )

    def test_legacy_profile_migrates_to_confirmed_fixed_bed(self) -> None:
        profile = ready_profile()
        profile["version"] = 1
        profile.pop("tool_center_point_m")
        profile.pop("print_bed_m")
        profile["confirmations"].pop("tcp_measured")
        profile["confirmations"].pop("print_bed_measured")

        normalized = calibration.normalize_calibration(profile)

        self.assertEqual(normalized["version"], 5)
        self.assertTrue(normalized["confirmations"]["print_bed_measured"])
        self.assertEqual(
            normalized["print_bed_m"]["minimum"], {"x": -0.5175, "y": -0.27}
        )

    def test_persists_only_a_ready_profile_atomically(self) -> None:
        result = calibration.validate_calibration(
            ready_profile(),
            BOUNDS,
            SOURCE_SHA256,
        )
        incomplete = calibration.validate_calibration(
            calibration.default_calibration(),
            BOUNDS,
            SOURCE_SHA256,
        )
        with tempfile.TemporaryDirectory() as tempdir:
            path = Path(tempdir) / "calibration.json"
            with self.assertRaises(calibration.CalibrationNotReadyError):
                calibration.save_calibration(path, incomplete)
            saved = calibration.save_calibration(path, result)
            loaded = calibration.read_saved_calibration(path)

        self.assertEqual(saved["source_sha256"], SOURCE_SHA256)
        self.assertEqual(saved["version"], 5)
        assert loaded
        self.assertEqual(loaded["calibration"], result["calibration"])
        self.assertEqual(loaded["profile_sha256"], result["profile_sha256"])

    def test_corrupt_saved_profile_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            path = Path(tempdir) / "calibration.json"
            path.write_text(json.dumps({"calibration": {}}), encoding="utf-8")
            with self.assertRaises(calibration.CalibrationValidationError):
                calibration.read_saved_calibration(path)


if __name__ == "__main__":
    unittest.main()
