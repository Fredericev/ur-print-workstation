from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "gcode_planner.py"
SPEC = importlib.util.spec_from_file_location("gcode_planner", MODULE_PATH)
assert SPEC and SPEC.loader
planner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = planner
SPEC.loader.exec_module(planner)


class GCodePlannerTests(unittest.TestCase):
    def test_builds_distinct_ur_and_klipper_channels(self) -> None:
        fixture = Path(__file__).parent / "fixtures" / "sample-print.gcode"
        plan = planner.build_path_plan(fixture.name, fixture.read_bytes())

        self.assertEqual(plan["status"], "draft")
        self.assertTrue(plan["execution_locked"])
        self.assertEqual(plan["ur_robot"]["responsibility"], ["X", "Y", "Z"])
        self.assertEqual(plan["ur_robot"]["waypoint_count"], 6)
        self.assertEqual(plan["klipper"]["extrusion_move_count"], 3)
        self.assertEqual(plan["synchronization"]["combined_motion_count"], 3)
        self.assertGreater(
            len(plan["synchronization"]["timeline"]["events"]),
            0,
        )

    def test_keeps_g28_in_klipper_preparation_channel(self) -> None:
        plan = planner.build_path_plan(
            "homing.gcode",
            b"G21\nG90\nM83\nG28\nG1 X10 Y10 E1 F600\n",
        )

        self.assertEqual(
            plan["klipper"]["preparation_commands"][0]["command"],
            "G28",
        )
        self.assertEqual(plan["gates"][-1]["status"], "required")

    def test_blocks_unsafe_source_files(self) -> None:
        with self.assertRaises(planner.GCodePlanError):
            planner.build_path_plan(
                "unsafe.gcode",
                b"G1 X1 F600\nM112\n",
            )


if __name__ == "__main__":
    unittest.main()
