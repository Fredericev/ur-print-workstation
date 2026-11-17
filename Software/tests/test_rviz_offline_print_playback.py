import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "rviz_offline_print_playback.py"
)
LAUNCHER_PATH = MODULE_PATH.parents[3] / "scripts" / "start_offline_rviz_print_preview.sh"
OFFLINE_LAUNCH_PATH = (
    MODULE_PATH.parents[1]
    / "src"
    / "ur3e_print_planner"
    / "launch"
    / "offline_print_preview.launch.py"
)
OFFLINE_RVIZ_PATH = (
    MODULE_PATH.parents[1]
    / "src"
    / "ur3e_print_planner"
    / "config"
    / "offline_print_preview.rviz"
)


def load_module():
    spec = importlib.util.spec_from_file_location("rviz_offline_print_playback", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OfflineRvizPlaybackTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temporary = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temporary.name)
        self.execution_dir = self.workspace / "runtime" / "print_execution"
        self.review_dir = self.workspace / "runtime" / "path_review"
        self.execution_dir.mkdir(parents=True)
        self.review_dir.mkdir(parents=True)
        self.source_dir = self.workspace.parent / "Test gcode"
        self.source_dir.mkdir(exist_ok=True)
        self.source = self.source_dir / "fixture.gcode"
        self.source.write_text("G1 X1\n")
        self.names = list(self.module.UR_JOINTS)
        self.approach = self._write_csv(
            "review.approach.csv",
            [(0.0, 0.0, [0, 0, 0, 0, 0, 0]),
             (2.0, 1.0, [0.1, 0, 0, 0, 0, 0])],
        )
        self.print_path = self._write_csv(
            "review.print.csv",
            [(0.0, 0.0, [0.1, 0, 0, 0, 0, 0]),
             (6.0, 1.0, [0.1, 0.2, 0, 0, 0, 0])],
        )
        review = {
            "status": "passed",
            "simulation": False,
            "planning_policy_version": "continuous-cartesian-approach-v4",
            "approach_status": "passed",
            "interpolation_validation": "passed",
            "interpolation_max_joint_spacing_rad": 0.25,
            "collision_free": True,
            "joint_limits": "passed",
            "cartesian_fraction": 1.0,
            "reference_frame": "base_link",
            "joint_names": self.names,
            "planned_point_count": 2,
            "duration_s": 6.0,
            "review_sha256": "review-sha",
        }
        (self.review_dir / "review.json").write_text(json.dumps(review))
        manifest = {
            "planning_policy_version": "continuous-cartesian-approach-v4",
            "planning_request": {"review_id": "review"},
            "review_sha256": "review-sha",
            "source": {"name": self.source.name, "sha256": self._sha(self.source)},
            "source_sha256": self._sha(self.source),
            "approach_trajectory": {
                "sha256": self._sha(self.approach), "point_count": 2,
                "duration_s": 2.0, "joint_names": self.names,
            },
            "print_trajectory": {
                "sha256": self._sha(self.print_path), "point_count": 2,
                "duration_s": 6.0, "joint_names": self.names,
            },
            # A bounded cold test can contain one block, but RViz must replay
            # the complete MoveIt print CSV bound above.
            "operation_mode": "tool0_motion_test",
            "blocks": [{"index": 0}],
        }
        self.manifest = self.execution_dir / "manifest.json"
        self.manifest.write_text(json.dumps(manifest))

    def tearDown(self):
        self.temporary.cleanup()
        try:
            self.source.unlink()
            self.source_dir.rmdir()
        except OSError:
            pass

    def _write_csv(self, name, rows):
        path = self.review_dir / name
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "path_progress", *self.names])
            for timestamp, progress, positions in rows:
                writer.writerow([timestamp, progress, *positions])
        return path

    @staticmethod
    def _sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_manifest_loads_approach_then_complete_print_and_scales_display_time(self):
        playback = self.module.load_playback(
            self.manifest,
            workspace_root=self.workspace,
            project_root=self.workspace.parent,
            speed_factor=4.0,
        )

        self.assertEqual([track.name for track in playback.tracks], ["approach", "print"])
        self.assertEqual([len(track.points) for track in playback.tracks], [2, 2])
        self.assertEqual(playback.tracks[0].points[-1].time_s, 0.5)
        self.assertEqual(playback.tracks[1].points[-1].time_s, 1.5)
        self.assertEqual(playback.tracks[1].points[-1].positions[1], 0.2)
        self.assertEqual(playback.source_name, "fixture.gcode")
        self.assertEqual(playback.review_id, "review")
        summary = self.module.playback_summary(playback)
        self.assertEqual(summary["scope"], "cached_moveit_approach_plus_complete_print")
        self.assertEqual(summary["display_duration_s"], 2.0)
        self.assertFalse(summary["hardware_contacted"])
        self.assertFalse(summary["executable"])

    def test_v3_replay_uses_the_final_continuous_curve_and_timing(self):
        from Software.tests.test_full_scope_runner import prepare
        package = prepare(self.workspace, "full")
        tracks = self.module._continuous_tracks(package, speed_factor=4.0)
        self.assertEqual([track.name for track in tracks], ["approach", "print"])
        frozen = package["trajectory_profiles"]["print"]
        self.assertEqual(tracks[1].source_duration_s, frozen["duration_s"])
        self.assertEqual(tracks[1].points[-1].time_s, frozen["duration_s"]/4.0)
        self.assertEqual(tracks[1].points[-1].positions, tuple(frozen["points"][-1]["positions"]))
        self.assertGreater(len(tracks[1].points), len(frozen["points"]))

    def test_changed_v3_curve_cannot_be_replayed_under_the_old_identity(self):
        from Software.tests.test_full_scope_runner import prepare
        package = prepare(self.workspace, "full")
        package["trajectory_profiles"]["print"]["points"][0]["positions"][0] += .001
        with self.assertRaisesRegex(self.module.PlaybackError, "identity|changed"):
            self.module._continuous_tracks(package, speed_factor=4.0)

    def test_changed_review_csv_is_rejected_instead_of_displayed(self):
        with self.approach.open("a") as stream:
            stream.write("3,1,0.2,0,0,0,0,0\n")

        with self.assertRaisesRegex(self.module.PlaybackError, "checksum"):
            self.module.load_playback(
                self.manifest,
                workspace_root=self.workspace,
                project_root=self.workspace.parent,
            )

    def test_failed_or_discontinuous_review_is_rejected(self):
        review_path = self.review_dir / "review.json"
        review = json.loads(review_path.read_text())
        review["collision_free"] = False
        review_path.write_text(json.dumps(review))
        with self.assertRaisesRegex(self.module.PlaybackError, "validated"):
            self.module.load_playback(
                self.manifest,
                workspace_root=self.workspace,
                project_root=self.workspace.parent,
            )

        review["collision_free"] = True
        review_path.write_text(json.dumps(review))
        with self.print_path.open() as stream:
            rows = list(csv.reader(stream))
        rows[1][2] = "0.5"
        with self.print_path.open("w", newline="") as stream:
            csv.writer(stream).writerows(rows)
        manifest = json.loads(self.manifest.read_text())
        manifest["print_trajectory"]["sha256"] = self._sha(self.print_path)
        self.manifest.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(self.module.PlaybackError, "continuous"):
            self.module.load_playback(
                self.manifest,
                workspace_root=self.workspace,
                project_root=self.workspace.parent,
            )

    def test_speed_factor_must_be_finite_and_bounded(self):
        for invalid in (0, -1, float("nan"), float("inf"), 101):
            with self.subTest(speed_factor=invalid):
                with self.assertRaises(self.module.PlaybackError):
                    self.module.load_playback(
                        self.manifest,
                        workspace_root=self.workspace,
                        project_root=self.workspace.parent,
                        speed_factor=invalid,
                    )

    def test_cli_accepts_explicit_project_root_for_container_mounts(self):
        result = subprocess.run(
            [
                sys.executable,
                str(MODULE_PATH),
                "--manifest",
                str(self.manifest),
                "--workspace-root",
                str(self.workspace),
                "--project-root",
                str(self.workspace.parent),
                "--check-only",
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"source": "fixture.gcode"', result.stdout)

    def test_launcher_waits_for_graphical_desktop_and_isolates_ros_transport(self):
        launcher = LAUNCHER_PATH.read_text(encoding="utf-8")

        self.assertIn("xdpyinfo", launcher)
        self.assertLess(launcher.index("xdpyinfo"), launcher.index("nohup setsid ros2 launch"))
        self.assertIn("export ROS_LOCALHOST_ONLY=1", launcher)
        self.assertIn("[u]r_ros2_control_node", launcher)
        self.assertIn("[r]os2_control_node", launcher)
        self.assertIn("[u]rscript_interface", launcher)
        self.assertIn("[k]lipper_bridge", launcher)
        self.assertGreaterEqual(launcher.count("assert_no_hardware_processes"), 3)
        self.assertIn("setsid ros2 launch", launcher)
        self.assertIn("__node:=[o]ffline_preview_robot_state_publisher", launcher)
        self.assertIn("__node:=[o]ffline_print_preview_rviz", launcher)
        self.assertNotIn("pkill -x rviz2", launcher)

    def test_offline_rviz_uses_display_only_trajectory_plugin(self):
        config = OFFLINE_RVIZ_PATH.read_text(encoding="utf-8")
        launch = OFFLINE_LAUNCH_PATH.read_text(encoding="utf-8")

        self.assertIn("Class: moveit_rviz_plugin/Trajectory", config)
        self.assertIn("Trajectory Topic: /display_planned_path", config)
        self.assertNotIn("Class: moveit_rviz_plugin/MotionPlanning", config)
        self.assertTrue(config.startswith("Panels: []"))
        self.assertIn("Hide Left Dock: true", config)
        self.assertIn("offline_print_preview.rviz", launch)


if __name__ == "__main__":
    unittest.main()
