"""Public-package guarantees for uninterrupted full cold execution."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from Software.tests.test_full_execution_scope import full_review
from Software.tests.test_print_execution import (
    execution,
    moveit,
    write_request,
    write_trajectory,
)


class ContinuousFullExecutionTests(unittest.TestCase):
    def prepare(self, root: Path) -> dict:
        approach = root / "approach.csv"
        printing = root / "print.csv"
        write_trajectory(approach, distance=0.0)
        write_trajectory(printing)
        return execution.prepare_execution_manifest(
            review=full_review(),
            moveit_result=moveit(),
            approach_trajectory_path=approach,
            print_trajectory_path=printing,
            planning_request_path=write_request(root),
            destination=root / "full.json",
            manual_stepper="extruder",
            execution_extent="full",
        )

    def test_full_job_is_one_continuous_profile_with_one_shared_start(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory))

        self.assertEqual(manifest["version"], 3)
        self.assertIn("print", manifest["trajectory_profiles"])
        self.assertNotIn("blocks", manifest["trajectory_profiles"])
        self.assertEqual(manifest["coordination"]["synchronized_start_count"], 1)
        self.assertEqual(
            manifest["execution_scope"]["motion_duration_s"],
            round(manifest["trajectory_profiles"]["print"]["duration_s"], 3),
        )

    def test_ordinary_process_boundaries_are_timeline_markers_not_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory))

        profile = manifest["trajectory_profiles"]["print"]
        marker_points = {
            marker: point
            for point in profile["points"]
            for marker in point.get("timeline_markers", [])
        }
        moving_blocks = [block for block in manifest["blocks"] if block["has_ur_motion"]]
        self.assertGreater(len(moving_blocks), 2)
        for previous, current in zip(manifest["blocks"], manifest["blocks"][1:]):
            self.assertAlmostEqual(previous["end_time_s"], current["start_time_s"], places=6)
        for block in moving_blocks[:-1]:
            point = marker_points[f"block:{block['index']}:end"]
            self.assertTrue(
                any(abs(value) > 1.0e-9 for value in point["velocities"]),
                f"ordinary block {block['index']} was incorrectly compiled as a stop",
            )

    def test_explicit_process_only_retraction_remains_an_intentional_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory))

        terminal = manifest["blocks"][-1]
        self.assertEqual(terminal["kind"], "retract")
        self.assertFalse(terminal["has_ur_motion"])
        self.assertGreater(terminal["end_time_s"], terminal["start_time_s"])
        marker_points = {
            marker: point
            for point in manifest["trajectory_profiles"]["print"]["points"]
            for marker in point.get("timeline_markers", [])
        }
        start = marker_points[f"block:{terminal['index']}:start"]
        end = marker_points[f"block:{terminal['index']}:end"]
        self.assertEqual(start["positions"], end["positions"])
        self.assertTrue(all(abs(value) <= 1.0e-12 for value in start["velocities"]))
        self.assertTrue(all(abs(value) <= 1.0e-12 for value in end["velocities"]))

    def test_fast_process_only_retraction_stretches_the_shared_timeline(self):
        payload = full_review()
        payload["timeline"]["events"][-1]["klipper_units"] = -15.0
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach, printing = root / "approach.csv", root / "print.csv"
            write_trajectory(approach, distance=0.0)
            write_trajectory(printing)
            manifest = execution.prepare_execution_manifest(
                review=payload, moveit_result=moveit(),
                approach_trajectory_path=approach, print_trajectory_path=printing,
                planning_request_path=write_request(root), destination=root / "full.json",
                manual_stepper="extruder", stepper_speed=5.0, execution_extent="full",
            )
        terminal = manifest["blocks"][-1]
        self.assertGreaterEqual(terminal["duration_s"], 3.0)
        self.assertLessEqual(abs(terminal["klipper_units"])/terminal["duration_s"], 5.0)

    def test_mixed_moveit_feed_and_stepper_floors_survive_certification(self):
        payload = full_review()
        payload["path"]["segments"] = [
            {
                "sequence": 1, "line": 1, "layer": 0,
                "x0": 0.0, "x1": 0.01, "y0": 0.0, "y1": 0.0,
                "z0": 0.0, "z1": 0.0,
            },
            {
                "sequence": 2, "line": 2, "layer": 0,
                "x0": 0.01, "x1": 0.02, "y0": 0.0, "y1": 0.0,
                "z0": 0.0, "z1": 0.0,
            },
        ]
        payload["timeline"]["events"] = [
            {
                "sequence": 1, "line": 1, "duration_s": 0.6,
                "ur_distance_mm": 10.0, "klipper_units": 3.0,
                "klipper_action": "extrude",
            },
            {
                "sequence": 2, "line": 2, "duration_s": 1.0,
                "ur_distance_mm": 10.0, "klipper_units": 4.0,
                "klipper_action": "extrude",
            },
            {
                "sequence": 3, "line": 3, "duration_s": 0.1,
                "ur_distance_mm": 0.0, "klipper_units": -20.0,
                "klipper_action": "retract",
            },
        ]

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            approach = root / "approach.csv"
            printing = root / "print.csv"
            write_trajectory(approach, distance=0.0)
            printing.write_text(
                "time_s,path_progress," + ",".join(execution.UR_JOINTS) + "\n"
                + "0.000000000,0.000000000," + ",".join("0" for _ in execution.UR_JOINTS) + "\n"
                + "0.250000000,0.500000000," + ",".join(["0.01", "0", "0", "0", "0", "0"]) + "\n"
                + "2.000000000,1.000000000," + ",".join(["0.02", "0", "0", "0", "0", "0"]) + "\n",
                encoding="utf-8",
            )
            manifest = execution.prepare_execution_manifest(
                review=payload,
                moveit_result=moveit(),
                approach_trajectory_path=approach,
                print_trajectory_path=printing,
                planning_request_path=write_request(root),
                destination=root / "mixed-timing.json",
                manual_stepper="extruder",
                stepper_speed=5.0,
                execution_extent="full",
            )

        blocks = manifest["blocks"]
        profile = manifest["trajectory_profiles"]["print"]
        marker_points = {
            marker: point
            for point in profile["points"]
            for marker in point.get("timeline_markers", [])
        }
        self.assertEqual(len(blocks), 3)
        self.assertEqual(len(marker_points), 2 * len(blocks))
        self.assertAlmostEqual(manifest["timing_audit"]["source_nominal_gcode_duration_s"], 1.7)

        for index, nominal_s in enumerate((0.6, 1.0, 0.1)):
            block = blocks[index]
            start = marker_points[f"block:{index}:start"]
            end = marker_points[f"block:{index}:end"]
            certified_s = end["time_s"] - start["time_s"]
            self.assertAlmostEqual(block["start_time_s"], start["time_s"], places=6)
            self.assertAlmostEqual(block["end_time_s"], end["time_s"], places=6)
            self.assertGreaterEqual(certified_s + 1.0e-6, nominal_s)
            self.assertLessEqual(
                abs(block["klipper_units"]) / certified_s,
                manifest["stepper_limits"]["maximum_speed"] + 1.0e-6,
            )

        first, second, hold = blocks
        self.assertGreaterEqual(second["duration_s"] + 1.0e-6, 1.75)
        self.assertGreaterEqual(first["duration_s"], first["minimum_stepper_duration_s"])
        self.assertGreaterEqual(second["duration_s"], second["minimum_stepper_duration_s"])
        self.assertFalse(hold["has_ur_motion"])
        self.assertGreaterEqual(hold["duration_s"], 4.0)
        hold_start = marker_points["block:2:start"]
        hold_end = marker_points["block:2:end"]
        self.assertEqual(hold_start["positions"], hold_end["positions"])
        self.assertEqual(hold_start["velocities"], [0.0] * 6)
        self.assertEqual(hold_end["velocities"], [0.0] * 6)

    def test_timeline_mapping_does_not_rescan_the_whole_trajectory_per_block(self):
        points = [
            {
                "time_s": float(index),
                "path_progress": index / 100.0,
                "positions": [index / 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
            for index in range(101)
        ]
        blocks = [
            {
                "start_progress": index / 10.0,
                "end_progress": (index + 1) / 10.0,
                "duration_s": 10.0,
                "minimum_stepper_duration_s": 0.0,
            }
            for index in range(10)
        ]

        with mock.patch.object(
            execution,
            "slice_path",
            side_effect=AssertionError("linear slice_path rescan"),
        ):
            timeline = execution._continuous_process_path(points, blocks)

        self.assertEqual(timeline[0]["path_progress"], 0.0)
        self.assertEqual(timeline[-1]["path_progress"], 1.0)
        self.assertEqual(timeline[-1]["time_s"], 100.0)


if __name__ == "__main__":
    unittest.main()
