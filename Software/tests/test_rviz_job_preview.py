from __future__ import annotations

import importlib.util
import hashlib
import tempfile
import unittest
from pathlib import Path

from Software.gcode_calibration import default_calibration, save_installed_calibration_draft


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = PROJECT_ROOT / "scripts" / "export_rviz_job_preview.py"
SPEC = importlib.util.spec_from_file_location("export_rviz_job_preview", MODULE_PATH)
assert SPEC and SPEC.loader
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


class RvizJobPreviewTests(unittest.TestCase):
    def test_fixture_gcode_is_centered_on_fixed_table(self) -> None:
        gcode = PROJECT_ROOT / "Software/tests/fixtures/sample-print.gcode"
        calibration = default_calibration()
        calibration["operation_mode"] = "tool0_motion_test"
        calibration["tool_center_point_m"] = {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.0},
            "collision_radius_m": 0.0,
        }
        calibration["confirmations"].update({
            "orientation_verified": True,
            "tool_orientation_verified": True,
            "extrusion_ratio_measured": True,
            "tool0_reference_verified": True,
        })
        with tempfile.TemporaryDirectory() as directory:
            calibration_path = Path(directory) / "calibration.json"
            save_installed_calibration_draft(
                calibration_path, calibration,
                hashlib.sha256(gcode.read_bytes()).hexdigest(),
            )
            payload = preview.build_preview(gcode, calibration_path)

        self.assertEqual(payload["frame_id"], "base_link")
        self.assertEqual(payload["source"]["name"], "sample-print.gcode")
        self.assertEqual(payload["placement"]["print_center_m"], {"x": -0.2925, "y": 0.0})
        self.assertNotIn("work_envelope_m", payload)
        print_segments = [
            segment for segment in payload["segments"] if segment["kind"] == "print"
        ]
        xs = [point for segment in print_segments for point in (segment["x0"], segment["x1"])]
        ys = [point for segment in print_segments for point in (segment["y0"], segment["y1"])]
        self.assertAlmostEqual((min(xs) + max(xs)) / 2, -0.2925, places=6)
        self.assertAlmostEqual((min(ys) + max(ys)) / 2, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
