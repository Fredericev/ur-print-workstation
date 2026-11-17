from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("export_github_source", PROJECT_ROOT / "scripts/export_github_source.py")
assert SPEC is not None and SPEC.loader is not None
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


def fixture(root: Path) -> None:
    for relative in exporter.REQUIRED_FILES:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("test fixture\n", encoding="utf-8")
    (root / "Software/control_station.py").write_text('VERSION = "0.13.0"\n', encoding="utf-8")
    (root / "Software/config.json").write_text('{"private": true}\n', encoding="utf-8")
    (root / "Software/gcode_calibration.json").write_text('{"private": true}\n', encoding="utf-8")
    (root / "CONTEXT.md").write_text("local operator notes\n", encoding="utf-8")
    (root / ".github/workflows/ci.yml").parent.mkdir(parents=True)
    (root / ".github/workflows/ci.yml").write_text("name: source-check\n", encoding="utf-8")
    (root / ".github/local-note.txt").write_text("do not ship\n", encoding="utf-8")
    (root / "Software/runtime/job.log").parent.mkdir(parents=True)
    (root / "Software/runtime/job.log").write_text("do not ship\n", encoding="utf-8")
    (root / "workspaces/ur_ros2_ws/build/controller.o").parent.mkdir(parents=True)
    (root / "workspaces/ur_ros2_ws/build/controller.o").write_text("do not ship\n", encoding="utf-8")
    nested_git = root / "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/.git"
    nested_git.mkdir(parents=True)
    (nested_git / "config").write_text("vendor git state\n", encoding="utf-8")
    vendor_test = root / "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/tests"
    vendor_test.mkdir(parents=True)
    (vendor_test / "external.UR5").symlink_to("/ursim/programs.UR5")
    (root / "Software/Mac").mkdir(parents=True)
    (root / "Software/Mac/UR Print Workstation.app").symlink_to("/Users/someone/Applications/Developer.app")


class GitHubSourceExportTests(unittest.TestCase):
    def test_actual_worktree_selection_is_full_and_excludes_private_state(self) -> None:
        files, omitted = exporter.selected_files(PROJECT_ROOT)
        relative = {path.relative_to(PROJECT_ROOT).as_posix() for path in files}
        self.assertTrue(set(exporter.REQUIRED_FILES).issubset(relative))
        self.assertIn("workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_controllers/src/passthrough_trajectory_controller.cpp", relative)
        self.assertNotIn("Software/config.json", relative)
        self.assertNotIn("Software/gcode_calibration.json", relative)
        self.assertNotIn("CONTEXT.md", relative)
        self.assertFalse(any("/.git/" in name or name.endswith(".pyc") or "/runtime/" in name for name in relative))
        self.assertFalse(any("UR Print Workstation.app" in name for name in relative))
        vendor_link = (
            "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/"
            "tests/resources/dockerursim/programs/cb3/programs.UR5"
        )
        if (PROJECT_ROOT / vendor_link).is_symlink():
            self.assertIn(vendor_link, omitted)
        self.assertFalse(relative.intersection(omitted))

    def test_fixture_zip_is_self_contained_and_export_is_immutable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "project"
            root.mkdir()
            fixture(root)
            output = Path(temporary) / "out"
            package, archive = exporter.export_source(root, output)
            self.assertTrue((package / "Software/control_station.py").is_file())
            self.assertFalse((package / "Software/config.json").exists())
            self.assertFalse((package / "CONTEXT.md").exists())
            self.assertTrue((package / ".github/workflows/ci.yml").is_file())
            self.assertFalse((package / ".github/local-note.txt").exists())
            self.assertFalse((package / "Software/Mac/UR Print Workstation.app").exists())
            self.assertFalse((package / "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/.git").exists())
            manifest = json.loads((package / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["version"], "0.13.0")
            self.assertEqual(len(manifest["omitted_vendor_test_symlinks"]), 1)
            with zipfile.ZipFile(archive) as source_zip:
                self.assertIsNone(source_zip.testzip())
                self.assertIn(f"{package.name}/Software/control_station.py", source_zip.namelist())
                self.assertNotIn(f"{package.name}/Software/config.json", source_zip.namelist())
            original = archive.read_bytes()
            with self.assertRaisesRegex(exporter.ExportError, "already exists"):
                exporter.export_source(root, output)
            self.assertEqual(archive.read_bytes(), original)

    def test_runtime_symlink_is_rejected_instead_of_followed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture(root)
            runtime_source = root / "Software/print_execution.py"
            runtime_source.unlink()
            runtime_source.symlink_to("/private/outside.py")
            with self.assertRaisesRegex(exporter.ExportError, "symlink"):
                exporter.selected_files(root)


if __name__ == "__main__":
    unittest.main()
