import importlib.util
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


PROJECT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "prepare_macos_ros_artifacts", PROJECT / "scripts/prepare_macos_ros_artifacts.py"
)
recovery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recovery)


class MacOSGeneratedArtifactTests(unittest.TestCase):
    def make_workspace(self, root):
        workspace = Path(root) / "workspace"
        for name in ("build", "install", "src", "config", "runtime"):
            directory = workspace / name
            directory.mkdir(parents=True)
            (directory / "sentinel").write_text(name)
        return workspace

    def flags_for(self, flagged):
        original = recovery.os.lstat

        def metadata(path, *args, **kwargs):
            result = original(path, *args, **kwargs)
            return SimpleNamespace(
                st_mode=result.st_mode,
                st_flags=recovery.DATALESS_FLAG if Path(path) in flagged else 0,
            )

        return metadata

    @unittest.skipIf(os.name == "nt", "Exercises Darwin fcntl and O_NOFOLLOW locking")
    def test_real_directory_backup_preserves_outputs_and_operator_data(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = self.make_workspace(directory)
            placeholder = workspace / "build/sentinel"
            with mock.patch.object(recovery.platform, "system", return_value="Darwin"), \
                 mock.patch.object(recovery.os, "lstat", side_effect=self.flags_for({placeholder})):
                backup = recovery.prepare_generated_artifacts(workspace)
                self.assertIsNone(recovery.prepare_generated_artifacts(workspace))
            self.assertTrue((backup / "COLCON_IGNORE").is_file())
            for name in ("build", "install"):
                self.assertEqual((backup / name / "sentinel").read_text(), name)
                self.assertEqual(list((workspace / name).iterdir()), [])
            for name in ("src", "config", "runtime"):
                self.assertEqual((workspace / name / "sentinel").read_text(), name)

    def test_normal_local_outputs_do_not_trigger_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = self.make_workspace(directory)
            with mock.patch.object(recovery.platform, "system", return_value="Darwin"), \
                 mock.patch.object(recovery.os, "lstat", side_effect=self.flags_for(set())):
                self.assertIsNone(recovery.prepare_generated_artifacts(workspace))
            self.assertEqual((workspace / "install/sentinel").read_text(), "install")

    def test_non_macos_does_not_scan(self):
        with mock.patch.object(recovery.platform, "system", return_value="Linux"), \
             mock.patch.object(recovery, "_triggers") as scan:
            self.assertIsNone(recovery.prepare_generated_artifacts(Path("/unused")))
            scan.assert_not_called()

    @unittest.skipIf(os.name == "nt", "Exercises a macOS source symlink fixture")
    def test_symlink_to_source_is_never_scanned(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = self.make_workspace(directory)
            (workspace / "build/source-link").symlink_to(workspace / "src", target_is_directory=True)
            placeholder = workspace / "src/sentinel"
            with mock.patch.object(recovery.platform, "system", return_value="Darwin"), \
                 mock.patch.object(recovery.os, "lstat", side_effect=self.flags_for({placeholder})):
                self.assertIsNone(recovery.prepare_generated_artifacts(workspace))

    @unittest.skipIf(os.name == "nt", "Exercises Darwin fcntl and O_NOFOLLOW locking")
    def test_failure_rolls_back_renamed_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = self.make_workspace(directory)
            placeholder = workspace / "build/sentinel"
            original = recovery.os.rename

            def fail_install(source, target):
                if Path(source) == workspace / "install":
                    raise OSError("injected rename failure")
                return original(source, target)

            with mock.patch.object(recovery.platform, "system", return_value="Darwin"), \
                 mock.patch.object(recovery.os, "lstat", side_effect=self.flags_for({placeholder})), \
                 mock.patch.object(recovery.os, "rename", side_effect=fail_install):
                with self.assertRaises(recovery.ArtifactRecoveryError):
                    recovery.prepare_generated_artifacts(workspace)
            for name in ("build", "install"):
                self.assertEqual((workspace / name / "sentinel").read_text(), name)


if __name__ == "__main__":
    unittest.main()
