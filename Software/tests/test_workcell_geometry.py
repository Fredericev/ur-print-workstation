from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "workcell_geometry.py"
SPEC = importlib.util.spec_from_file_location("workcell_geometry", MODULE_PATH)
assert SPEC and SPEC.loader
workcell_geometry = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = workcell_geometry
SPEC.loader.exec_module(workcell_geometry)


class WorkcellGeometryTests(unittest.TestCase):
    def test_installed_table_geometry_keeps_the_xy_workcell(self) -> None:
        workcell = workcell_geometry.load_workcell_geometry()
        tabletop = workcell["tabletop"]

        self.assertEqual(workcell["frame_id"], "base_link")
        self.assertEqual(tabletop["center_m"], {"x": -0.2925, "y": 0.0})
        self.assertEqual(tabletop["dimensions_m"], {"x": 0.45, "y": 0.54})
        self.assertEqual(tabletop["yaw_deg"], 0.0)
        self.assertGreaterEqual(tabletop["surface_z_m"], -0.5)
        self.assertLessEqual(tabletop["surface_z_m"], 0.5)
        self.assertIn(
            tabletop["measurement"]["surface_z"],
            {"operator_selected", "operator_verified_height_trial"},
        )
        self.assertIsNone(tabletop["thickness_m"])
        self.assertEqual(
            tabletop["bounds_m"],
            {
                "minimum": {"x": -0.5175, "y": -0.27},
                "maximum": {"x": -0.0675, "y": 0.27},
            },
        )
        self.assertEqual(workcell["placement"]["mode"], "center_print_bounds")


if __name__ == "__main__":
    unittest.main()
