"""Real UMS5 regression for certified continuous full-job timing.

This is deliberately an integration seam: the checked-in G-code and the
latest full-resolution MoveIt artifacts must produce the same 3,800-block
execution package the operator reviews in the workstation.
"""

from __future__ import annotations

import copy
import json
import tempfile
import time
import unittest
from pathlib import Path

from Software.gcode_path_review import build_offline_review
from Software.gcode_path_review import PLANNING_POLICY_VERSION
from Software.print_execution import prepare_execution_manifest
from Software.printhead_geometry import printhead_geometry_sha256


ROOT = Path(__file__).resolve().parents[2]
GCODE = ROOT / "Test gcode" / "UMS5_mold2.gcode"
REVIEW_ROOT = ROOT / "workspaces" / "ur_ros2_ws" / "runtime" / "path_review"
REVIEW_ID = "36432a037235"

INSTALLED_UMS5_CALIBRATION = {
    "version": 5,
    "placement_mode": "center_on_fixed_table",
    "operation_mode": "tool0_motion_test",
    "printhead_geometry_sha256": printhead_geometry_sha256(),
    "coordinate_transform": {
        "origin_m": {"x": -0.348664, "y": 0.04757, "z": -0.014},
        "rotation_z_deg": -90.0,
        "scale_m_per_mm": 0.001,
        "tool_orientation_rpy_deg": {"roll": 180.0, "pitch": 0.0, "yaw": 0.0},
    },
    "klipper_extrusion": {"units_per_e_mm": 1.0, "direction": "positive"},
    "tool_center_point_m": {
        "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "collision_radius_m": 0.0,
    },
    "work_envelope_m": {
        "minimum": {"x": -1.0, "y": -1.0, "z": -0.25},
        "maximum": {"x": 1.0, "y": 1.0, "z": 1.5},
    },
    "print_bed_m": {
        "plane_z": -0.014,
        "minimum": {"x": -0.5175, "y": -0.27},
        "maximum": {"x": -0.0675, "y": 0.27},
        "edge_margin_m": 0.01,
    },
    "confirmations": {
        "origin_measured": True,
        "orientation_verified": True,
        "tool_orientation_verified": True,
        "extrusion_ratio_measured": True,
        "tcp_measured": False,
        "print_bed_measured": True,
        "tool0_reference_verified": True,
    },
}


class UMS5DurationRegressionTests(unittest.TestCase):
    def test_full_cold_job_keeps_exact_process_knots_without_five_hour_retiming(self):
        required = (
            GCODE,
            REVIEW_ROOT / f"{REVIEW_ID}.json",
            REVIEW_ROOT / f"{REVIEW_ID}.approach.csv",
            REVIEW_ROOT / f"{REVIEW_ID}.print.csv",
        )
        missing = [path for path in required if not path.is_file()]
        if missing:
            self.skipTest("Local UMS5 job artifacts are unavailable: " + ", ".join(str(path) for path in missing))

        review = build_offline_review(
            GCODE.name,
            GCODE.read_bytes(),
            copy.deepcopy(INSTALLED_UMS5_CALIBRATION),
            maximum_preview_segments=299_999,
        )
        # The stored MoveIt result is the full-resolution trajectory used for
        # this timing regression.  UI-only review schema changes can alter the
        # current review hash without changing that trajectory, so bind the
        # freshly parsed, identical source geometry to the reviewed artifact.
        moveit_result = json.loads(
            (REVIEW_ROOT / f"{REVIEW_ID}.json").read_text(encoding="utf-8")
        )
        review["review_sha256"] = moveit_result["review_sha256"]
        moveit_result["planning_request_id"] = REVIEW_ID
        moveit_result["planning_policy_version"] = PLANNING_POLICY_VERSION
        # This offline fixture checks timing on a bare-tool0 path; it does not
        # promote the historical result into physical printhead validation.
        moveit_result["printhead_geometry_sha256"] = review["printhead_geometry_sha256"]
        fixture_request_path = REVIEW_ROOT / (moveit_result["review_sha256"] + ".request.json")
        if not fixture_request_path.is_file():
            self.skipTest(f"Local UMS5 planning request is unavailable: {fixture_request_path}")

        with tempfile.TemporaryDirectory() as directory:
            fixture_request = json.loads(fixture_request_path.read_text(encoding="utf-8"))
            fixture_request["planning_policy_version"] = PLANNING_POLICY_VERSION
            request_path = Path(directory) / "ums5.request.json"
            request_path.write_text(json.dumps(fixture_request), encoding="utf-8")
            started = time.monotonic()
            manifest = prepare_execution_manifest(
                review=review,
                moveit_result=moveit_result,
                approach_trajectory_path=REVIEW_ROOT / f"{REVIEW_ID}.approach.csv",
                print_trajectory_path=REVIEW_ROOT / f"{REVIEW_ID}.print.csv",
                planning_request_path=request_path,
                destination=Path(directory) / "ums5-full.json",
                manual_stepper="extruder",
                execution_extent="full",
            )
            preparation_s = time.monotonic()-started

        profile = manifest["trajectory_profiles"]["print"]
        audit = manifest["timing_audit"]
        self.assertEqual(manifest["block_count"], 3800)
        self.assertEqual(profile["source_point_count"], 14719)
        self.assertEqual(profile["limits"], {
            "velocity": 0.2,
            "acceleration": 2.5,
            "jerk": 100.0,
        })
        self.assertEqual(profile["waypoint_corridor_rad"], 0.0012)
        self.assertEqual(profile["minimum_segment_duration_s"], 0.002)
        # Native E acceleration, real holds, and bounded C2 corners require
        # more than the source timeline. Lock down both the multi-hour
        # failure and the later 35-minute over-retiming regression, with an
        # explicitly bounded, wire-certified C2 curve still required.
        self.assertLessEqual(profile["duration_s"], 1500.0)
        self.assertLess(preparation_s, 60.0, "Preparing this fixture must not take minutes")

        self.assertAlmostEqual(
            audit["source_nominal_gcode_duration_s"], 635.53183, places=6
        )
        self.assertAlmostEqual(audit["source_moveit_duration_s"], 727.989169, places=6)
        self.assertEqual(audit["process_only_hold_count"], 162)
        # The source has 162 stationary 6.5 mm retract/restore moves. Each
        # reaches 5 mm/s, with 0.25 s spent accelerating and 0.25 s braking
        # at 20 mm/s²: d/v + v/a = 1.55 s. Allow only sub-millisecond
        # aggregate numerical padding, independently of the compiler helper.
        holds = [block for block in manifest["blocks"] if not block["has_ur_motion"]]
        self.assertEqual(len(holds), 162)
        self.assertTrue(all(abs(abs(block["klipper_units"]) - 6.5) < 1.0e-9 for block in holds))
        physical_holds_s = 162 * (6.5 / 5.0 + 5.0 / 20.0)
        self.assertGreaterEqual(audit["process_only_hold_duration_s"], physical_holds_s)
        self.assertAlmostEqual(audit["process_only_hold_duration_s"], physical_holds_s, delta=0.001)
        self.assertAlmostEqual(audit["source_plus_holds_duration_s"], 727.989169 + physical_holds_s, delta=0.001)
        self.assertAlmostEqual(
            audit["source_moveit_duration_s"]
            + audit["process_only_hold_duration_s"]
            + audit["schedule_floor_extension_s"]
            + audit["certified_retiming_overhead_s"],
            audit["certified_print_duration_s"],
            places=5,
        )
        self.assertAlmostEqual(
            audit["certified_print_duration_s"], profile["duration_s"], places=6
        )

        marker_points = {
            marker: point
            for point in profile["points"]
            for marker in point.get("timeline_markers", [])
        }
        self.assertEqual(len(marker_points), 7600)
        for block in manifest["blocks"]:
            start = marker_points[f"block:{block['index']}:start"]
            end = marker_points[f"block:{block['index']}:end"]
            self.assertAlmostEqual(start["time_s"], block["start_time_s"], places=6)
            self.assertAlmostEqual(end["time_s"], block["end_time_s"], places=6)
            if not block["has_ur_motion"]:
                self.assertEqual(start["positions"], end["positions"])
                self.assertEqual(start["velocities"], [0.0] * 6)
                self.assertEqual(end["velocities"], [0.0] * 6)

        ordinary_boundaries = 0
        for previous, current in zip(manifest["blocks"], manifest["blocks"][1:]):
            if previous["has_ur_motion"] and current["has_ur_motion"]:
                ordinary_boundaries += 1
                point = marker_points[f"block:{previous['index']}:end"]
                self.assertTrue(
                    any(abs(value) > 1.0e-9 for value in point["velocities"]),
                    f"Ordinary boundary {previous['index']} must not manufacture a stop",
                )
        self.assertEqual(ordinary_boundaries, 3475)


if __name__ == "__main__":
    unittest.main()
