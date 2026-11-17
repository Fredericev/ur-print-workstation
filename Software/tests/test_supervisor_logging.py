from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "supervise_with_rotating_log.py"
)
SPEC = importlib.util.spec_from_file_location("supervise_with_rotating_log", SCRIPT)
assert SPEC and SPEC.loader
supervisor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = supervisor
SPEC.loader.exec_module(supervisor)


class RotatingLogTests(unittest.TestCase):
    def test_new_session_archives_previous_readiness_and_errors(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            path = Path(tempdir) / "driver.log"
            path.write_text("System successfully started!\nold failure\n")
            sink = supervisor.RotatingTextSink(
                path, max_bytes=4096, backups=3, new_session=True,
            )
            sink.write("new driver is initializing")
            sink.close()
            self.assertEqual(path.read_text(), "new driver is initializing\n")
            self.assertIn("old failure", path.with_suffix(".log.1").read_text())

    def test_log_storage_stays_bounded_and_keeps_one_backup(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            path = Path(tempdir) / "live.log"
            sink = supervisor.RotatingTextSink(path, max_bytes=80, backups=1)
            for index in range(20):
                sink.write(f"bounded line {index:02d}")
            sink.close()

            self.assertTrue(path.is_file())
            self.assertTrue(path.with_suffix(".log.1").is_file())
            self.assertLessEqual(path.stat().st_size, 100)
            self.assertLessEqual(path.with_suffix(".log.1").stat().st_size, 100)


if __name__ == "__main__":
    unittest.main()
