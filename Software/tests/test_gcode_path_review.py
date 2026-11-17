from __future__ import annotations

import importlib.util
import json
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "gcode_path_review.py"
SPEC = importlib.util.spec_from_file_location("gcode_path_review", MODULE_PATH)
assert SPEC and SPEC.loader
reviewer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = reviewer
SPEC.loader.exec_module(reviewer)


READY_CALIBRATION = {
    "version": 3,
    "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
    "coordinate_transform": {
        "origin_m": {"x": 0.2, "y": -0.1, "z": 0.3},
        "rotation_z_deg": 90.0,
        "scale_m_per_mm": 0.001,
    },
    "klipper_extrusion": {
        "units_per_e_mm": 2.0,
        "direction": "negative",
    },
    "tool_center_point_m": {
        "offset": {"x": 0.0, "y": 0.0, "z": 0.05},
        "collision_radius_m": 0.01,
    },
    "work_envelope_m": {
        "minimum": {"x": -1.0, "y": -1.0, "z": -0.25},
        "maximum": {"x": 1.0, "y": 1.0, "z": 1.5},
    },
    "print_bed_m": {
        "plane_z": 0.3,
        "minimum": {"x": -0.5, "y": -0.5},
        "maximum": {"x": 0.5, "y": 0.5},
        "edge_margin_m": 0.01,
    },
    "confirmations": {
        "origin_measured": True,
        "orientation_verified": True,
        "tool_orientation_verified": True,
        "extrusion_ratio_measured": True,
        "tcp_measured": True,
        "print_bed_measured": True,
    },
}


class GCodePathReviewTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = Path(__file__).parent / "fixtures" / "sample-print.gcode"

    def test_robot_trajectory_excludes_printer_preamble_and_shutdown_motion(self) -> None:
        content = (
            "G21\n"
            "G90\n"
            "M83\n"
            "G0 X330 Y237 Z20 F9000\n"
            "G0 X10 Y20 Z0.2\n"
            "G1 X20 Y20 E1 F600\n"
            "G0 X25 Y25\n"
            "G1 X20 Y30 E1\n"
            "G0 X500 Y500 Z30\n"
        ).encode()

        review = reviewer.build_offline_review(
            "printer-preamble.gcode",
            content,
            READY_CALIBRATION,
        )

        segments = review["path"]["segments"]
        self.assertEqual([segment["line"] for segment in segments], [6, 7, 8])
        self.assertEqual(segments[0]["kind"], "print")
        self.assertAlmostEqual(segments[0]["cumulative_s"], 1.0)
        self.assertEqual(review["path"]["excluded_lead_in_move_count"], 2)
        self.assertEqual(review["path"]["excluded_shutdown_move_count"], 1)
        self.assertTrue(review["within_print_bed"])

    def test_builds_calibrated_3d_segments_and_klipper_units(self) -> None:
        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )

        self.assertEqual(review["status"], "offline_ready")
        self.assertTrue(review["execution_locked"])
        self.assertEqual(review["coordinate_frame"], "base_link")
        self.assertGreater(len(review["path"]["segments"]), 0)
        first = review["path"]["segments"][0]
        self.assertIn("z0", first)
        self.assertIn("z1", first)
        printed = next(
            segment
            for segment in review["path"]["segments"]
            if segment["kind"] == "print"
        )
        self.assertLess(printed["klipper_units"], 0)
        self.assertEqual(review["gates"][-1]["status"], "required")

    def test_legacy_tabletop_review_migrates_to_fixed_placement(self) -> None:
        profile = json.loads(json.dumps(READY_CALIBRATION))
        profile["operation_mode"] = "tool0_tabletop_test"
        profile["coordinate_transform"]["origin_m"]["z"] = 0.5
        profile["confirmations"]["tool0_reference_verified"] = True

        with self.assertRaises(reviewer.PathReviewError):
            reviewer.build_offline_review(
                self.fixture.name,
                self.fixture.read_bytes(),
                profile,
            )

        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            profile,
            allow_incomplete_simulation=True,
        )

        self.assertEqual(review["operation_mode"], "tool0_motion_test")
        self.assertEqual(
            review["placement_model"]["mode"], "center_on_fixed_table"
        )
        self.assertNotIn("tool0_pose_recapture_required", review["calibration"])
        self.assertFalse(review["motion_test_calibration_complete"])
        self.assertFalse(review["placement_model"]["execution_authoritative"])
        self.assertEqual(review["tcp_offset_base_m"], {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_requires_complete_calibration(self) -> None:
        incomplete = json.loads(json.dumps(READY_CALIBRATION))
        incomplete["confirmations"]["orientation_verified"] = False
        with self.assertRaises(reviewer.PathReviewError):
            reviewer.build_offline_review(
                self.fixture.name,
                self.fixture.read_bytes(),
                incomplete,
            )

    def test_incomplete_calibration_can_only_create_simulation_review(self) -> None:
        incomplete = json.loads(json.dumps(READY_CALIBRATION))
        incomplete["tool_center_point_m"] = {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
            "collision_radius_m": 0.0,
        }
        incomplete["confirmations"]["tcp_measured"] = False
        incomplete["confirmations"]["print_bed_measured"] = False

        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            incomplete,
            allow_incomplete_simulation=True,
        )

        self.assertEqual(review["status"], "offline_simulation")
        self.assertTrue(review["simulation_only"])
        self.assertFalse(review["physical_calibration_complete"])
        self.assertTrue(review["execution_locked"])
        self.assertFalse(review["placement_model"]["execution_authoritative"])
        gate_status = {gate["code"]: gate["status"] for gate in review["gates"]}
        self.assertEqual(gate_status["tcp_geometry"], "required")
        self.assertEqual(gate_status["print_bed"], "passed")

    def test_fixed_workcell_centers_every_print_on_confirmed_tabletop(self) -> None:
        incomplete = json.loads(json.dumps(READY_CALIBRATION))
        incomplete["coordinate_transform"]["origin_m"] = {
            "x": -0.35,
            "y": 0.05,
            "z": 0.15,
        }
        incomplete["tool_center_point_m"] = {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
            "collision_radius_m": 0.0,
        }
        incomplete["print_bed_m"] = {
            "plane_z": -0.014,
            "minimum": {"x": 0.0, "y": 0.0},
            "maximum": {"x": 0.0, "y": 0.0},
            "edge_margin_m": 0.01,
        }
        incomplete["confirmations"]["tcp_measured"] = False
        incomplete["confirmations"]["print_bed_measured"] = False

        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            incomplete,
            allow_incomplete_simulation=True,
            use_initial_workcell_model=True,
        )

        self.assertEqual(
            review["placement_model"]["mode"], "center_on_fixed_table"
        )
        self.assertFalse(review["placement_model"]["assumed"])
        self.assertFalse(review["placement_model"]["execution_authoritative"])
        self.assertEqual(review["placement_model"]["bounds_source"], "print_path")
        self.assertEqual(
            review["placement_model"]["print_center_m"],
            {"x": -0.2925, "y": 0.0},
        )
        table_z = review["collision_bed_m"]["plane_z"]
        self.assertAlmostEqual(
            review["calibration"]["coordinate_transform"]["origin_m"]["z"],
            table_z,
        )
        self.assertAlmostEqual(
            review["physical_deposition_bounds_m"]["minimum"]["z"],
            table_z + 0.0002,
        )
        self.assertAlmostEqual(
            review["deposition_bounds_m"]["minimum"]["z"],
            table_z + 0.0002,
        )
        tabletop = review["workcell_model"]["tabletop"]
        self.assertAlmostEqual(tabletop["surface_z_m"], table_z)
        self.assertEqual(tabletop["minimum"], {"x": -0.5175, "y": -0.27})
        self.assertEqual(tabletop["maximum"], {"x": -0.0675, "y": 0.27})
        self.assertEqual(tabletop["dimensions_m"], {"x": 0.45, "y": 0.54})
        self.assertIsNone(tabletop["thickness_m"])
        self.assertEqual(tabletop["footprint_source"], "fixed_workcell_config")
        self.assertFalse(tabletop["visual_only"])
        self.assertTrue(tabletop["collision_authoritative"])

        printed = [
            segment
            for segment in review["path"]["segments"]
            if segment["kind"] == "print"
        ]
        print_x = [value for segment in printed for value in (segment["x0"], segment["x1"])]
        print_y = [value for segment in printed for value in (segment["y0"], segment["y1"])]
        self.assertAlmostEqual((min(print_x) + max(print_x)) / 2.0, -0.2925)
        self.assertAlmostEqual((min(print_y) + max(print_y)) / 2.0, 0.0)

        gate_status = {gate["code"]: gate["status"] for gate in review["gates"]}
        self.assertEqual(gate_status["coordinate_transform"], "required")
        self.assertEqual(gate_status["tcp_geometry"], "required")
        self.assertEqual(gate_status["print_bed"], "passed")

    def test_writes_moveit_csv_without_losing_geometry(self) -> None:
        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "path.csv"
            result = reviewer.write_moveit_path(path, review)
            rows = path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(rows[0], "sequence,line,x,y,z,kind,cumulative_s")
        self.assertEqual(result["written_waypoint_count"], len(rows) - 1)
        self.assertEqual(result["maximum_simplification_deviation_m"], 0.0)

    def test_rejects_moveit_csv_when_waypoint_cap_loses_geometry(self) -> None:
        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "path.csv"
            with self.assertRaises(reviewer.PathReviewError):
                reviewer.write_moveit_path(path, review, maximum_waypoints=3)

    def test_review_identity_does_not_depend_on_display_sampling(self) -> None:
        full = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )
        bounded = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
            maximum_preview_segments=3,
        )

        self.assertFalse(full["path"]["sampled"])
        self.assertTrue(bounded["path"]["sampled"])
        self.assertEqual(full["review_sha256"], bounded["review_sha256"])

    def test_demo_result_is_explicitly_simulated_and_locked(self) -> None:
        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )
        result = reviewer.demo_moveit_result(review)

        self.assertTrue(result["simulation"])
        self.assertTrue(result["execution_locked"])
        self.assertEqual(len(result["joint_names"]), 6)
        self.assertEqual(len(result["trajectory"]), 120)

    def test_offline_robot_simulation_tracks_path_without_claiming_validation(self) -> None:
        review = reviewer.build_offline_review(
            self.fixture.name,
            self.fixture.read_bytes(),
            READY_CALIBRATION,
        )
        progress_updates = []
        result = reviewer.build_offline_robot_simulation(
            review,
            progress_callback=lambda completed, total: progress_updates.append(
                (completed, total)
            ),
        )

        self.assertEqual(result["status"], "passed")
        self.assertTrue(result["simulation"])
        self.assertTrue(result["offline_estimate"])
        self.assertTrue(result["execution_locked"])
        self.assertIsNone(result["collision_free"])
        self.assertEqual(result["joint_limits"], "not_validated")
        self.assertEqual(len(result["joint_names"]), 6)
        self.assertGreater(len(result["trajectory"]), 1)
        self.assertEqual(result["trajectory"][0]["progress"], 0.0)
        self.assertEqual(result["trajectory"][-1]["progress"], 1.0)

        _rotation, position = reviewer._offline_forward_kinematics(
            result["trajectory"][0]["positions"]
        )
        first_segment = review["path"]["segments"][0]
        target = [
            first_segment["tool0_x0"],
            first_segment["tool0_y0"],
            first_segment["tool0_z0"],
        ]
        tcp_error = sum(
            (position[index] - target[index]) ** 2 for index in range(3)
        ) ** 0.5
        self.assertLess(tcp_error, 0.002)
        self.assertLessEqual(
            result["maximum_tcp_error_m"],
            reviewer.OFFLINE_POSITION_TOLERANCE_M,
        )
        self.assertLessEqual(
            result["maximum_orientation_error_rad"],
            reviewer.OFFLINE_ORIENTATION_TOLERANCE_RAD,
        )
        self.assertEqual(result["joint_continuity"], "passed")
        self.assertTrue(result["full_source_geometry_validated"])
        self.assertTrue(result["requires_robot_retiming"])
        self.assertEqual(result["time_parameterization"], "not_performed")
        self.assertGreater(len(progress_updates), 1)
        self.assertEqual(progress_updates[0][0], 0)
        self.assertGreater(progress_updates[0][1], 1)
        self.assertEqual(progress_updates[-1][0], progress_updates[-1][1])
        self.assertTrue(
            all(
                previous[0] <= current[0]
                and previous[1] == current[1]
                for previous, current in zip(
                    progress_updates,
                    progress_updates[1:],
                )
            )
        )

    def test_numeric_ik_round_trips_reachable_ur3e_poses(self) -> None:
        randomizer = random.Random(8220182939500692340)
        for _ in range(50):
            expected_joints = [
                randomizer.uniform(-math.pi, math.pi) for _ in range(6)
            ]
            rotation, position = reviewer._offline_forward_kinematics(
                expected_joints
            )
            seed = [
                value + randomizer.uniform(-0.35, 0.35)
                for value in expected_joints
            ]
            _solved, position_error, orientation_error = (
                reviewer._solve_offline_ik(position, rotation, seed)
            )

            self.assertLessEqual(
                position_error, reviewer.OFFLINE_POSITION_TOLERANCE_M
            )
            self.assertLessEqual(
                orientation_error,
                reviewer.OFFLINE_ORIENTATION_TOLERANCE_RAD,
            )

    def test_numeric_ik_rejects_a_pose_outside_ur3e_reach(self) -> None:
        target_rotation = reviewer._rpy_rotation(math.pi, 0.0, 0.0)
        _joints, position_error, _orientation_error = reviewer._solve_offline_ik(
            [2.0, 0.0, 0.5],
            target_rotation,
            [0.0, -1.3, 1.6, -1.8, -math.pi / 2.0, 0.0],
        )

        self.assertGreater(
            position_error, reviewer.OFFLINE_POSITION_TOLERANCE_M
        )

    def test_forward_kinematics_matches_prior_moveit_plan_endpoints(self) -> None:
        # These joint states and Cartesian targets were produced by the real
        # ROS/MoveIt plan-only validator with this robot's calibration.
        cases = [
            (
                [0.475769173, 3.872299501, -1.554348417, -0.747404522, -1.569702279, 1.268786289],
                [-0.426, -0.072, 0.25],
            ),
            (
                [0.209472594, 4.369561354, -2.489531675, -0.30923335, -1.570796331, 1.535856658],
                [-0.25428, 0.079921, 0.27024],
            ),
            (
                [2.99610726, 0.397105315, -0.720494049, -2.119784876, -4.733795338, 3.122049678],
                [-0.426, -0.072, 0.25],
            ),
            (
                [0.205268498, 4.971170598, -1.936888621, 4.794506061, 1.581403879, 0.296080913],
                [-0.25428, 0.079921, 0.27024],
            ),
        ]
        for joints, expected_position in cases:
            _rotation, actual_position = reviewer._offline_forward_kinematics(
                joints
            )
            self.assertLess(math.dist(actual_position, expected_position), 0.0021)

    def test_moveit_result_rejects_wrong_review_hash(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            path.write_text(
                json.dumps(
                    {
                        "status": "passed",
                        "review_sha256": "wrong",
                        "joint_names": ["a", "b", "c", "d", "e", "f"],
                        "trajectory": [],
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(reviewer.PathReviewError):
                reviewer.read_moveit_result(path, "expected")

    def test_old_planning_policy_cannot_reuse_a_matching_checksum(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            payload = {
                "status": "passed", "review_sha256": "expected",
                "joint_names": ["a", "b", "c", "d", "e", "f"],
                "trajectory": [{"positions": [0] * 6}],
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(reviewer.PathReviewError, "planning policy"):
                reviewer.read_moveit_result(path, "expected")

    def test_new_failed_review_does_not_restore_an_older_pass(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = {
                "status": "passed", "review_sha256": "expected",
                "planning_policy_version": "continuous-cartesian-approach-v4",
                "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
                "interpolation_validation": "passed",
                "joint_names": ["a", "b", "c", "d", "e", "f"],
                "trajectory": [{"positions": [0] * 6}],
            }
            (root / "old.json").write_text(json.dumps(payload), encoding="utf-8")
            (root / "new.json").write_text(
                json.dumps({**payload, "status": "failed"}), encoding="utf-8"
            )
            self.assertIsNone(reviewer.find_cached_moveit_result(root, "expected"))

    def test_active_request_blocks_cache_until_its_exact_result_passes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = {
                "status": "passed", "review_sha256": "a" * 64,
                "planning_policy_version": "continuous-cartesian-approach-v4",
                "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
                "interpolation_validation": "passed",
                "joint_names": ["a", "b", "c", "d", "e", "f"],
                "trajectory": [{"positions": [0] * 6}],
            }
            (root / "old.json").write_text(json.dumps(payload), encoding="utf-8")
            reviewer.begin_planning_request(root, "a" * 64, "b" * 12)
            self.assertIsNone(reviewer.find_cached_moveit_result(root, "a" * 64))
            (root / ("b" * 12 + ".json")).write_text(json.dumps(payload), encoding="utf-8")
            self.assertIsNotNone(reviewer.find_cached_moveit_result(root, "a" * 64))

    def test_passed_result_requires_completed_interpolation_validation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            payload = {
                "status": "passed", "review_sha256": "expected",
                "planning_policy_version": "continuous-cartesian-approach-v4",
                "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
                "joint_names": ["a", "b", "c", "d", "e", "f"],
                "trajectory": [{"positions": [0] * 6}],
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(reviewer.PathReviewError, "interpolation"):
                reviewer.read_moveit_result(path, "expected")

    def test_failed_moveit_result_can_report_without_a_trajectory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            path.write_text(
                json.dumps(
                    {
                        "status": "failed",
                        "review_sha256": "expected",
                        "planning_policy_version": "continuous-cartesian-approach-v4",
                        "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
                        "joint_names": [],
                        "trajectory": [],
                        "execution_locked": True,
                        "message": "No collision-free Cartesian path was found.",
                    }
                ),
                encoding="utf-8",
            )
            result = reviewer.read_moveit_result(path, "expected")

        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["execution_locked"])

    def test_finds_newest_passed_cached_result_for_exact_review(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wrong = root / "newer-wrong.json"
            wrong.write_text(
                json.dumps(
                    {
                        "status": "passed",
                        "review_sha256": "different",
                        "joint_names": ["a", "b", "c", "d", "e", "f"],
                        "trajectory": [
                            {
                                "progress": 0.0,
                                "positions": [0, 0, 0, 0, 0, 0],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            matching = root / "matching.json"
            matching.write_text(
                json.dumps(
                    {
                        "status": "passed",
                        "review_sha256": "expected",
                        "planning_policy_version": "continuous-cartesian-approach-v4",
                        "printhead_geometry_sha256": reviewer.printhead_geometry_sha256(),
                        "interpolation_validation": "passed",
                        "joint_names": ["a", "b", "c", "d", "e", "f"],
                        "trajectory": [
                            {
                                "progress": 0.0,
                                "positions": [0, 0, 0, 0, 0, 0],
                            },
                            {
                                "progress": 1.0,
                                "positions": [1, 1, 1, 1, 1, 1],
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            result = reviewer.find_cached_moveit_result(root, "expected")

        self.assertIsNotNone(result)
        self.assertTrue(result["cached"])
        self.assertEqual(result["review_sha256"], "expected")

    def test_cached_result_ignores_failed_plan(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "failed.json"
            path.write_text(
                json.dumps(
                    {
                        "status": "failed",
                        "review_sha256": "expected",
                        "joint_names": [],
                        "trajectory": [],
                    }
                ),
                encoding="utf-8",
            )

            result = reviewer.find_cached_moveit_result(
                Path(directory),
                "expected",
            )

        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
