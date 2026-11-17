"""Offline contract for native E commands on the frozen UR process timeline."""

from __future__ import annotations

import copy
import json
import math
import re
import sys
import tempfile
import unittest
from pathlib import Path

from Software.tests.test_print_execution import (
    execution,
    moveit,
    review,
    runner,
    write_request,
    write_trajectory,
)


BRIDGE_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "workspaces/ur_ros2_ws/src/ur_klipper_bridge"
)
sys.path.insert(0, str(BRIDGE_SOURCE))
from ur_klipper_bridge.gcode import build_program_file


NATIVE_PROTOCOL = "klipper-native-extruder-v1"
SPEED_MM_S = 5.0
ACCEL_MM_S2 = 20.0
MAX_MOVE_MM = 50.0


def stopped_move_duration(distance: float, speed: float = SPEED_MM_S) -> float:
    """Physical rest-to-rest trapezoid, independent of the production solver."""
    distance = abs(distance)
    if distance <= speed * speed / ACCEL_MM_S2:
        return 2.0 * math.sqrt(distance / ACCEL_MM_S2)
    return distance / speed + speed / ACCEL_MM_S2


class NativeExtruderExecutionTests(unittest.TestCase):
    def prepare(self, root: Path, payload: dict | None = None) -> dict:
        approach, printing = root / "approach.csv", root / "print.csv"
        write_trajectory(approach, distance=0.0)
        write_trajectory(printing)
        return execution.prepare_execution_manifest(
            review=payload if payload is not None else review(),
            moveit_result=moveit(),
            approach_trajectory_path=approach,
            print_trajectory_path=printing,
            planning_request_path=write_request(root),
            destination=root / "native.json",
            extruder="extruder",
            stepper_speed=SPEED_MM_S,
            stepper_acceleration=ACCEL_MM_S2,
            execution_extent="full",
        )

    @staticmethod
    def program(manifest: dict) -> str:
        return build_program_file(
            stepper=manifest["extruder"],
            segments=[
                {"units": block["klipper_units"], "duration_s": block["duration_s"]}
                for block in manifest["blocks"]
            ],
            start_delay_s=0.0,
            maximum_speed=SPEED_MM_S,
            maximum_acceleration=ACCEL_MM_S2,
            maximum_extrude_only_distance=MAX_MOVE_MM,
            disable_after=False,
        )

    def assert_program_matches_timeline(self, manifest: dict) -> list[tuple[float, float]]:
        program = self.program(manifest)
        moves = [
            (float(match[1]), float(match[2]) / 60.0)
            for line in program.splitlines()
            if (match := re.fullmatch(r"G1 E([^ ]+) F([^ ]+)", line))
        ]
        self.assertTrue(moves)
        self.assertIn("M83", program.splitlines())
        self.assertNotIn("MANUAL_STEPPER", program)
        self.assertEqual(program.splitlines().count("M400"), 1)
        self.assertTrue(all(0.0 < abs(units) <= MAX_MOVE_MM for units, _ in moves))
        self.assertTrue(all(0.0 < speed <= SPEED_MM_S for _, speed in moves))
        expected = sum(
            float(block["duration_s"])
            for block in manifest["blocks"]
            if abs(float(block["klipper_units"])) > 1.0e-12
        )
        actual = sum(stopped_move_duration(units, speed) for units, speed in moves)
        self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(
            sum(units for units, _ in moves),
            sum(float(block["klipper_units"]) for block in manifest["blocks"]),
            places=6,
        )
        return moves

    def test_manifest_binds_the_native_extruder_protocol_and_installed_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root)
            self.assertEqual(manifest["extrusion_protocol"], NATIVE_PROTOCOL)
            self.assertEqual(manifest["extruder"], "extruder")
            self.assertEqual(manifest["stepper_limits"]["maximum_speed"], SPEED_MM_S)
            self.assertEqual(manifest["stepper_limits"]["acceleration"], ACCEL_MM_S2)
            self.assertEqual(
                manifest["stepper_limits"]["max_extrude_only_distance"], MAX_MOVE_MM,
            )
            self.assertEqual(execution.read_execution_manifest(root / "native.json"), manifest)
            self.assert_program_matches_timeline(manifest)

    def test_short_retraction_reserves_acceleration_time_in_the_shared_profile(self):
        payload = review()
        payload["timeline"]["events"][-1].update(
            klipper_units=-0.5, duration_s=0.001,
        )
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory), payload)

        retracts = [block for block in manifest["blocks"] if block["kind"] == "retract"]
        self.assertEqual(len(retracts), 1)
        retract = retracts[0]
        minimum = 2.0 * math.sqrt(0.5 / ACCEL_MM_S2)
        self.assertGreaterEqual(float(retract["duration_s"]) + 1.0e-6, minimum)
        profile = manifest["trajectory_profiles"]["print"]
        self.assertAlmostEqual(
            float(profile["duration_s"]),
            sum(float(block["duration_s"]) for block in manifest["blocks"]),
            places=6,
        )
        self.assert_program_matches_timeline(manifest)

    def test_long_positive_and_negative_moves_preserve_totals_and_native_distance_guard(self):
        payload = review()
        payload["timeline"]["events"][1].update(
            klipper_units=120.0, duration_s=0.001,
        )
        payload["timeline"]["events"][2].update(
            klipper_units=-120.0, duration_s=0.001,
        )
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory), payload)

        moves = self.assert_program_matches_timeline(manifest)
        self.assertAlmostEqual(sum(units for units, _ in moves if units > 0.0), 120.0)
        self.assertAlmostEqual(sum(units for units, _ in moves if units < 0.0), -120.0)
        for sign in (1.0, -1.0):
            selected = [
                block for block in manifest["blocks"]
                if sign * float(block["klipper_units"]) > 0.0
            ]
            minimum = sum(
                (count := max(1, math.ceil(abs(float(block["klipper_units"])) / MAX_MOVE_MM)))
                * stopped_move_duration(float(block["klipper_units"]) / count)
                for block in selected
            )
            self.assertGreaterEqual(
                sum(float(block["duration_s"]) for block in selected) + 1.0e-6,
                minimum,
            )
        blocks = manifest["blocks"]
        self.assertEqual(blocks[0]["start_progress"], 0.0)
        self.assertEqual(blocks[-1]["end_progress"], 1.0)
        for previous, following in zip(blocks, blocks[1:]):
            self.assertAlmostEqual(previous["end_progress"], following["start_progress"], places=8)
            self.assertAlmostEqual(previous["end_time_s"], following["start_time_s"], places=6)

    def test_ordinary_extrusion_boundaries_keep_one_continuous_ur_profile(self):
        payload = review()
        payload["timeline"]["events"][0].update(
            klipper_units=0.5, klipper_action="extrude", duration_s=0.2,
        )
        payload["timeline"]["events"][1].update(
            klipper_units=0.7, duration_s=0.3,
        )
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory), payload)

        profile = manifest["trajectory_profiles"]["print"]
        markers = {
            marker: point
            for point in profile["points"]
            for marker in point.get("timeline_markers", [])
        }
        boundary = markers["block:0:end"]
        self.assertTrue(any(abs(value) > 1.0e-9 for value in boundary["velocities"]))
        self.assertEqual(manifest["coordination"]["synchronized_start_count"], 1)
        self.assertNotIn("blocks", manifest["trajectory_profiles"])
        self.assert_program_matches_timeline(manifest)

    def test_authentic_legacy_manual_stepper_package_requires_preparation_again(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root)
            legacy = copy.deepcopy(manifest)
            legacy.pop("extrusion_protocol")
            legacy.pop("extruder")
            legacy["manual_stepper"] = "stepper_x"
            legacy["execution_id"] = execution._manifest_execution_id(legacy)
            path = root / "legacy.json"
            path.write_text(json.dumps(legacy), encoding="utf-8")
            with self.assertRaisesRegex(execution.PrintExecutionError, "(?i)native|extrud|protocol"):
                execution.read_execution_manifest(path)
            with self.assertRaisesRegex(runner.ExecutionFailure, "(?i)native|extrud|protocol"):
                runner.load_manifest(path)

    def test_rehashed_packages_cannot_change_native_timing_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root)
            for key, value in (("maximum_speed", 6.), ("acceleration", 19.),
                               ("max_extrude_only_distance", 60.)):
                changed = copy.deepcopy(manifest)
                changed["stepper_limits"][key] = value
                changed["execution_id"] = execution._manifest_execution_id(changed)
                path = root / "changed.json"
                path.write_text(json.dumps(changed), encoding="utf-8")
                with self.subTest(key=key):
                    with self.assertRaisesRegex(execution.PrintExecutionError, "(?i)native|extrud"):
                        execution.read_execution_manifest(path)
                    with self.assertRaisesRegex(runner.ExecutionFailure, "(?i)native|extrud"):
                        runner.load_manifest(path)


if __name__ == "__main__":
    unittest.main()
