from __future__ import annotations

import importlib.util
import math
import random
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "gcode_parser.py"
SPEC = importlib.util.spec_from_file_location("gcode_parser", MODULE_PATH)
assert SPEC and SPEC.loader
parser = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = parser
SPEC.loader.exec_module(parser)


class GCodeParserTests(unittest.TestCase):
    def test_execution_window_excludes_printer_lead_in_and_shutdown_moves(self) -> None:
        result = parser.parse_gcode(
            "printer-preamble.gcode",
            (
                "G21\n"
                "G90\n"
                "M83\n"
                "G0 X330 Y237 Z20 F9000\n"
                "G0 X10 Y20 Z0.2\n"
                "G1 X20 Y20 E1 F600\n"
                "G0 X25 Y25\n"
                "G1 X20 Y30 E1\n"
                "G0 X500 Y500 Z30\n"
            ).encode(),
        )

        self.assertEqual(result["bounds"]["maximum"]["x"], 500.0)
        self.assertEqual(
            result["bounds"]["execution"],
            {
                "minimum": {"x": 10.0, "y": 20.0, "z": 0.2},
                "maximum": {"x": 25.0, "y": 30.0, "z": 0.2},
                "dimensions": {"x": 15.0, "y": 10.0, "z": 0.0},
                "source": "first_to_last_print",
                "first_sequence": 3,
                "last_sequence": 5,
                "excluded_lead_in_move_count": 2,
                "excluded_shutdown_move_count": 1,
            },
        )

    def test_reports_print_path_bounds_for_centered_workcell_placement(self) -> None:
        result = parser.parse_gcode(
            "centered.gcode",
            (
                "G21\n"
                "G90\n"
                "M83\n"
                "G0 X10 Y20 Z0.2 F1200\n"
                "G1 X30 Y20 E1 F600\n"
                "G1 X30 Y40 E1\n"
                "G0 X200 Y200 F1200\n"
            ).encode(),
        )

        self.assertEqual(
            result["bounds"]["placement"],
            {
                "minimum": {"x": 10.0, "y": 20.0, "z": 0.2},
                "maximum": {"x": 30.0, "y": 40.0, "z": 0.2},
                "dimensions": {"x": 20.0, "y": 20.0, "z": 0.0},
                "source": "print_path",
            },
        )

    def test_parses_common_absolute_gcode_and_builds_preview(self) -> None:
        fixture = Path(__file__).parent / "fixtures" / "sample-print.gcode"
        result = parser.parse_gcode(fixture.name, fixture.read_bytes())

        self.assertEqual(result["status"], "parsed")
        self.assertTrue(result["safe_to_plan"])
        self.assertEqual(result["summary"]["move_count"], 6)
        self.assertEqual(result["summary"]["print_move_count"], 3)
        self.assertEqual(result["summary"]["travel_move_count"], 3)
        self.assertEqual(result["summary"]["layer_count"], 2)
        self.assertEqual(result["summary"]["tool_count"], 1)
        self.assertAlmostEqual(result["bounds"]["maximum"]["x"], 20.0)
        self.assertAlmostEqual(result["bounds"]["maximum"]["y"], 20.0)
        self.assertAlmostEqual(result["bounds"]["maximum"]["z"], 0.4)
        self.assertEqual(result["file"]["generator"], "Workstation Test Slicer 1.0")
        self.assertGreater(len(result["preview"]["segments"]), 0)
        first_segment = result["preview"]["segments"][0]
        self.assertIn("z0", first_segment)
        self.assertIn("z1", first_segment)
        self.assertIn("line", first_segment)

    def test_supports_compact_relative_commands(self) -> None:
        result = parser.parse_gcode(
            "relative.gcode",
            b"G21\nG91\nM83\nG1X1Y2E.5F60\n",
        )

        self.assertEqual(result["summary"]["move_count"], 1)
        self.assertEqual(result["summary"]["print_move_count"], 1)
        self.assertAlmostEqual(result["bounds"]["maximum"]["x"], 1.0)
        self.assertAlmostEqual(result["bounds"]["maximum"]["y"], 2.0)

    def test_g90_and_g91_clear_an_explicit_extruder_mode_override(self) -> None:
        result = parser.parse_gcode(
            "cura-end.gcode",
            (
                "G21\n"
                "G90\n"
                "M82\n"
                "G1 X10 E637.62164 F600\n"
                "G91\n"
                "G1 E-2 F2700\n"
                "G1 E-2 Z0.2 F2400\n"
                "G90\n"
                "M82\n"
            ).encode(),
            maximum_preview_segments=20,
            maximum_timeline_events=20,
        )

        events = result["motion_timeline"]["events"]
        self.assertEqual(
            [event["extrusion_delta_mm"] for event in events[-2:]],
            [-2.0, -2.0],
        )
        self.assertEqual(result["summary"]["retraction_count"], 2)
        self.assertAlmostEqual(result["summary"]["extrusion_mm"], 637.62)

    def test_flags_dangerous_and_configuration_commands(self) -> None:
        result = parser.parse_gcode(
            "unsafe.gcode",
            b"G21\nG1 X1 F60\nM112\nRUN_SHELL_COMMAND CMD=unsafe\n",
        )
        codes = {issue["code"] for issue in result["issues"]}

        self.assertFalse(result["safe_to_plan"])
        self.assertEqual(result["status"], "needs_attention")
        self.assertIn("dangerous_command", codes)
        self.assertIn("configuration_command", codes)

    def test_reports_files_without_motion(self) -> None:
        result = parser.parse_gcode("empty-path.gcode", b"G21\nG90\nM82\n")

        self.assertFalse(result["safe_to_plan"])
        self.assertEqual(result["issues"][0]["code"], "no_motion_commands")

    def test_homing_is_a_non_blocking_check(self) -> None:
        result = parser.parse_gcode(
            "homing.gcode",
            b"G21\nG90\nG28\nG1 X10 Y10 F600\n",
        )

        self.assertTrue(result["safe_to_plan"])
        self.assertNotIn(
            "homing_command",
            {issue["code"] for issue in result["issues"]},
        )
        homing = result["preparation_commands"][0]
        self.assertEqual(homing["command"], "G28")
        self.assertEqual(homing["kind"], "home_axes")
        self.assertEqual(homing["line"], 3)

    def test_rejects_invalid_files(self) -> None:
        with self.assertRaises(parser.GCodeParseError):
            parser.parse_gcode("notes.txt", b"G1 X1")
        with self.assertRaises(parser.GCodeParseError):
            parser.parse_gcode("binary.gcode", b"G1\x00X1")
        with self.assertRaises(parser.GCodeParseError):
            parser.parse_gcode("bad.gcode", b"\xff\xfe")

    def test_preview_payload_is_bounded(self) -> None:
        lines = ["G21", "G90", "M83", "G1 F600"]
        lines.extend(
            f"G1 X{index % 100} Y{index // 100} E0.1"
            for index in range(parser.MAX_PREVIEW_SEGMENTS * 3)
        )
        result = parser.parse_gcode(
            "large.gcode",
            ("\n".join(lines) + "\n").encode(),
        )

        self.assertLessEqual(
            result["preview"]["returned_segment_count"],
            parser.MAX_PREVIEW_SEGMENTS,
        )
        self.assertTrue(result["preview"]["sampled"])
        self.assertNotIn(
            "preview_sampled",
            {issue["code"] for issue in result["issues"]},
        )

    def test_execution_can_request_the_complete_unsampled_timeline(self) -> None:
        event_count = parser.MAX_TIMELINE_EVENTS + 73
        lines = ["G21", "G90", "M83", "G1 F600"]
        lines.extend(
            f"G1 X{index * 0.01:.2f} E0.01"
            for index in range(1, event_count + 1)
        )
        content = ("\n".join(lines) + "\n").encode()

        display = parser.parse_gcode("timeline.gcode", content)
        execution = parser.parse_gcode(
            "timeline.gcode",
            content,
            maximum_preview_segments=event_count + 1,
            maximum_timeline_events=event_count + 1,
        )

        self.assertTrue(display["motion_timeline"]["sampled"])
        self.assertEqual(execution["motion_timeline"]["source_event_count"], event_count)
        self.assertEqual(execution["motion_timeline"]["returned_event_count"], event_count)
        self.assertFalse(execution["motion_timeline"]["sampled"])
        self.assertEqual(
            [event["sequence"] for event in execution["motion_timeline"]["events"]],
            list(range(1, event_count + 1)),
        )

    def test_interpolates_clockwise_and_counterclockwise_arcs(self) -> None:
        result = parser.parse_gcode(
            "arcs.gcode",
            (
                "G21\nG90\nM83\nG1 X10 Y0 F600\n"
                "G3 X0 Y10 I-10 J0 E1\n"
                "G2 X10 Y0 I0 J-10 E1\n"
            ).encode(),
        )

        self.assertTrue(result["safe_to_plan"])
        self.assertGreater(result["preview"]["source_segment_count"], 3)
        self.assertAlmostEqual(
            result["summary"]["total_path_mm"],
            10.0 + 10.0 * 3.141592653589793,
            places=2,
        )
        self.assertAlmostEqual(result["bounds"]["maximum"]["x"], 10.0, places=3)
        self.assertAlmostEqual(result["bounds"]["maximum"]["y"], 10.0, places=3)
        self.assertNotIn(
            "arc_preview_approximation",
            {issue["code"] for issue in result["issues"]},
        )

    def test_supports_radius_arcs_planes_and_absolute_arc_centres(self) -> None:
        result = parser.parse_gcode(
            "arc-modes.gcode",
            (
                "G21\nG90\nM83\nG1 X10 Y0 Z0 F600\n"
                "G90.1\nG3 X0 Y10 I0 J0 E1\n"
                "G18\nG2 X10 Z10 R10 E1\n"
            ).encode(),
        )

        self.assertTrue(result["safe_to_plan"])
        self.assertGreater(result["preview"]["source_segment_count"], 4)
        self.assertAlmostEqual(result["bounds"]["maximum"]["z"], 10.0, places=3)

    def test_g18_direction_matches_klipper_arc_plane_order(self) -> None:
        result = parser.parse_gcode(
            "xz-arc.gcode",
            b"G21\nG90\nG18\nG1 X10 Z0 F600\nG3 X0 Z10 I-10 K0\n",
        )

        arc_segments = result["preview"]["segments"][1:]
        self.assertTrue(result["safe_to_plan"])
        self.assertGreater(len(arc_segments), 2)
        self.assertGreater(arc_segments[0]["z1"], 0.0)
        self.assertLess(arc_segments[0]["x1"], 10.0)

    def test_randomized_arc_lengths_match_klipper_reference_direction(self) -> None:
        randomizer = random.Random(20260721)
        planes = {
            "G17": ("X", "Y", "I", "J"),
            "G18": ("X", "Z", "I", "K"),
            "G19": ("Y", "Z", "J", "K"),
        }
        for _ in range(120):
            plane, (first_axis, second_axis, first_offset, second_offset) = (
                randomizer.choice(list(planes.items()))
            )
            radius = randomizer.uniform(2.0, 40.0)
            start_angle = randomizer.uniform(-math.pi, math.pi)
            end_angle = randomizer.uniform(-math.pi, math.pi)
            if abs(end_angle - start_angle) < 0.05:
                end_angle += 0.3
            clockwise = randomizer.choice([False, True])
            start_first = radius * math.cos(start_angle)
            start_second = radius * math.sin(start_angle)
            end_first = radius * math.cos(end_angle)
            end_second = radius * math.sin(end_angle)
            command = "G2" if clockwise else "G3"
            content = (
                f"G21\nG90\n{plane}\nG1 {first_axis}{start_first:.8f} "
                f"{second_axis}{start_second:.8f} F600\n"
                f"{command} {first_axis}{end_first:.8f} "
                f"{second_axis}{end_second:.8f} "
                f"{first_offset}{-start_first:.8f} "
                f"{second_offset}{-start_second:.8f}\n"
            ).encode()
            result = parser.parse_gcode("random-arc.gcode", content)

            cross = start_first * end_second - start_second * end_first
            dot = start_first * end_first + start_second * end_second
            angular_travel = math.atan2(cross, dot)
            if angular_travel < 0.0:
                angular_travel += math.tau
            if clockwise:
                angular_travel -= math.tau
            expected_arc_length = abs(angular_travel) * radius
            expected_total = radius + expected_arc_length
            self.assertTrue(result["safe_to_plan"])
            self.assertAlmostEqual(
                result["summary"]["total_path_mm"], expected_total, delta=0.02
            )

    def test_g92_changes_coordinates_without_teleporting_the_path(self) -> None:
        result = parser.parse_gcode(
            "g92.gcode",
            b"G21\nG90\nG1 X10 F600\nG92 X0\nX1\n",
        )

        self.assertTrue(result["safe_to_plan"])
        self.assertEqual(result["summary"]["move_count"], 2)
        self.assertAlmostEqual(result["summary"]["total_path_mm"], 11.0)
        self.assertAlmostEqual(result["bounds"]["maximum"]["x"], 11.0)
        segments = result["preview"]["segments"]
        self.assertAlmostEqual(segments[0]["x1"], segments[1]["x0"])

    def test_rejects_malformed_arcs_and_unknown_coordinate_motion(self) -> None:
        malformed = parser.parse_gcode(
            "bad-arc.gcode",
            b"G21\nG90\nG1 X10 F600\nG2 X20 Y0\n",
        )
        machine_coordinates = parser.parse_gcode(
            "machine-frame.gcode",
            b"G21\nG90\nG53 G0 X10 F600\n",
        )

        self.assertFalse(malformed["safe_to_plan"])
        self.assertIn("invalid_arc_geometry", {item["code"] for item in malformed["issues"]})
        self.assertFalse(machine_coordinates["safe_to_plan"])
        self.assertIn(
            "unsupported_gcode_command",
            {item["code"] for item in machine_coordinates["issues"]},
        )

    def test_preview_simplification_remains_continuous(self) -> None:
        lines = ["G21", "G90", "M83", "G1 F600"]
        lines.extend(
            f"G1 X{index % 200} Y{(index // 200) % 40} E0.01"
            for index in range(parser.MAX_PREVIEW_SEGMENTS * 3)
        )
        result = parser.parse_gcode(
            "continuous-large.gcode",
            ("\n".join(lines) + "\n").encode(),
        )

        segments = result["preview"]["segments"]
        self.assertTrue(result["preview"]["continuous"])
        for first, second in zip(segments, segments[1:]):
            self.assertEqual(
                (first["x1"], first["y1"], first["z1"]),
                (second["x0"], second["y0"], second["z0"]),
            )


if __name__ == "__main__":
    unittest.main()
