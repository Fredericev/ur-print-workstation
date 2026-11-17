from __future__ import annotations

import unittest
import json
import threading
import urllib.error
import urllib.request

from Software import control_station
from Software.paired_timeline_import import PairedTimelineError, parse_paired_timeline


JOINT_HEADER = (
    "time_s,path_progress,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,"
    "wrist_1_joint,wrist_2_joint,wrist_3_joint\n"
)
TRAJECTORY = (
    JOINT_HEADER
    + "0,0,0,0,0,0,0,0\n"
    + "0.5,0.5,0.01,0,0,0,0,0\n"
    + "1,1,0.02,0,0,0,0,0\n"
).encode()
EXTRUSION = (
    b"G21\nM83\n"
    b"G1 E0.30 ;@sync start=0 end=0.5\n"
    b"G4 P250 ;@sync start=0.5 end=0.75\n"
    b"G1 E-0.08 ;@sync start=0.75 end=1\n"
)


class PairedTimelineImportTests(unittest.TestCase):
    def test_valid_pair_is_preview_only_and_preserves_shared_time(self) -> None:
        result = parse_paired_timeline(
            "route.csv", TRAJECTORY, "extrusion.gcode", EXTRUSION
        )
        self.assertTrue(result["ok"])
        self.assertFalse(result["execution_ready"])
        self.assertEqual(result["trajectory"]["duration_s"], 1.0)
        self.assertEqual(result["extrusion"]["duration_s"], 1.0)
        self.assertEqual(result["extrusion"]["segment_count"], 3)
        self.assertAlmostEqual(result["extrusion"]["positive_e_mm"], 0.30)
        self.assertAlmostEqual(result["extrusion"]["retraction_e_mm"], 0.08)
        self.assertEqual(result["extrusion"]["segments"][1]["delta_e_mm"], 0.0)

    def test_gap_or_overlap_is_rejected(self) -> None:
        for changed in (b"start=0.6 end=0.75", b"start=0.4 end=0.75"):
            with self.subTest(changed=changed):
                with self.assertRaisesRegex(PairedTimelineError, "continuous"):
                    parse_paired_timeline(
                        "route.csv",
                        TRAJECTORY,
                        "extrusion.gcode",
                        EXTRUSION.replace(b"start=0.5 end=0.75", changed),
                    )

    def test_different_endpoint_is_rejected(self) -> None:
        with self.assertRaisesRegex(PairedTimelineError, "same end time"):
            parse_paired_timeline(
                "route.csv", TRAJECTORY, "extrusion.gcode",
                EXTRUSION.replace(b"end=1\n", b"end=1.1\n"),
            )

    def test_unsafe_or_ambiguous_extrusion_commands_are_rejected(self) -> None:
        for data in (
            EXTRUSION.replace(b"M83", b"M82"),
            EXTRUSION.replace(b"G1 E0.30", b"G1 X10 E0.30"),
            EXTRUSION.replace(b"G1 E0.30", b"M104 S200\nG1 E0.30"),
            EXTRUSION.replace(b";@sync start=0 end=0.5", b""),
            EXTRUSION.replace(b"G4 P250", b"G4 P200"),
        ):
            with self.subTest(data=data):
                with self.assertRaises(PairedTimelineError):
                    parse_paired_timeline("route.csv", TRAJECTORY, "extrusion.gcode", data)

    def test_bad_joint_header_or_jump_is_rejected(self) -> None:
        with self.assertRaisesRegex(PairedTimelineError, "header"):
            parse_paired_timeline(
                "route.csv", TRAJECTORY.replace(b"shoulder_pan_joint", b"joint_1"),
                "extrusion.gcode", EXTRUSION,
            )
        with self.assertRaisesRegex(PairedTimelineError, "joint step"):
            parse_paired_timeline(
                "route.csv", TRAJECTORY.replace(b"0.01,0,0,0,0,0", b"1.01,0,0,0,0,0"),
                "extrusion.gcode", EXTRUSION,
            )


class PairedTimelineApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server, _ = control_station.create_server("127.0.0.1", 0, demo=True)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def _post(self, trajectory: bytes = TRAJECTORY, extrusion: bytes = EXTRUSION) -> tuple[int, dict]:
        request = urllib.request.Request(
            self.base + "/api/paired-timeline/parse",
            method="POST",
            data=json.dumps({
                "trajectory": {"name": "route.csv", "text": trajectory.decode()},
                "extrusion": {"name": "extrusion.gcode", "text": extrusion.decode()},
            }).encode(),
            headers={"Content-Type": "application/json", "Origin": self.base},
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def test_offline_pair_import_has_no_execution_approval(self) -> None:
        status, payload = self._post()
        self.assertEqual(status, 200)
        self.assertFalse(payload["analysis"]["execution_ready"])
        self.assertFalse(payload["analysis"]["moveit_validated"])
        with urllib.request.urlopen(self.base + "/api/mode", timeout=3) as response:
            self.assertEqual(json.loads(response.read())["mode"], "offline")

    def test_invalid_pair_is_rejected_before_review(self) -> None:
        status, payload = self._post(extrusion=EXTRUSION.replace(b"M83", b"M82"))
        self.assertEqual(status, 422)
        self.assertIn("error", payload)


if __name__ == "__main__":
    unittest.main()
