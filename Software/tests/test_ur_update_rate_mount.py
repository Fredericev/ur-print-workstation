"""Run the update-rate shell scripts with Docker and file mounts isolated."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[2]


@unittest.skipIf(os.name == "nt", "Executes POSIX shell, symlink, and bind-mount inode fixtures")
class UpdateRateMountTests(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.scripts = self.root / "scripts"
        self.scripts.mkdir()
        for name in ("prepare_ur_update_rate.sh", "apply_ur_update_rate.sh"):
            shutil.copy2(PROJECT / "scripts" / name, self.scripts / name)
        self.source = self.root / "workspaces/ur_ros2_ws/config/ur3e_update_rate.yaml"
        self.target = self.root / "container/ur3e_update_rate.yaml"
        self.target.parent.mkdir()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.trace = self.root / "commands.jsonl"
        fake = self.bin / "command"
        fake.write_text(f"#!{sys.executable}\n" + r'''
import json, os, pathlib, subprocess, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["UPDATE_RATE_TEST_TRACE"], "a") as log:
    log.write(json.dumps({"command": name, "args": args}) + "\n")
if name == "docker":
    if args[:1] != ["compose"] or args[-3:-1] != ["bash", "-lc"]:
        sys.exit("Only the isolated configuration exec is permitted")
    command = args[-1].replace(
        "/home/ubuntu/ur_ros2_ws/config/ur3e_update_rate.yaml",
        os.environ["UPDATE_RATE_TEST_SOURCE"],
    ).replace(
        "/opt/ros/humble/share/ur_robot_driver/config/ur3e_update_rate.yaml",
        os.environ["UPDATE_RATE_TEST_TARGET"],
    )
    sys.exit(subprocess.run(["/bin/bash", "-c", command]).returncode)
elif name == "findmnt":
    if os.environ.get("UPDATE_RATE_TEST_MOUNT_PROBE_FAILURE") == "true":
        sys.exit(1)
    print(os.environ["UPDATE_RATE_TEST_MOUNT_OPTIONS"])
elif name == "cp":
    if "ro" in os.environ["UPDATE_RATE_TEST_MOUNT_OPTIONS"].split(","):
        sys.exit("Attempted to write to the isolated read-only bind mount")
    sys.exit(subprocess.run(["/bin/cp", *args]).returncode)
''')
        fake.chmod(0o755)
        for name in ("docker", "findmnt", "cp"):
            (self.bin / name).symlink_to(fake)

    def run_script(self, name: str, **extra: str) -> subprocess.CompletedProcess[str]:
        env = dict(os.environ)
        env.update(
            PATH=f"{self.bin}:{os.environ['PATH']}",
            UR_TYPE="ur3e", UR_UPDATE_RATE="125",
            UPDATE_RATE_TEST_TRACE=str(self.trace),
            UPDATE_RATE_TEST_SOURCE=str(self.source),
            UPDATE_RATE_TEST_TARGET=str(self.target),
            UPDATE_RATE_TEST_MOUNT_OPTIONS="ro,relatime",
        )
        env.update(extra)
        return subprocess.run(
            ["/bin/bash", str(self.scripts / name)], env=env,
            capture_output=True, text=True, timeout=5,
        )

    def prepare(self, rate: str = "125") -> None:
        result = self.run_script("prepare_ur_update_rate.sh", UR_UPDATE_RATE=rate)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def mount_current_inode(self) -> None:
        # A file bind mount keeps the originally mounted inode alive after a
        # host rename. A hard link reproduces that behavior without Docker.
        os.link(self.source, self.target)

    def commands(self) -> list[str]:
        return [json.loads(line)["command"] for line in self.trace.read_text().splitlines()]

    def test_repeated_prepare_keeps_read_only_mounted_inode_and_mtime(self) -> None:
        self.prepare()
        self.mount_current_inode()
        before = self.source.stat()
        content = self.source.read_bytes()
        self.prepare()
        self.prepare()
        after = self.source.stat()
        self.assertEqual((after.st_ino, after.st_mtime_ns), (before.st_ino, before.st_mtime_ns))
        self.assertEqual(self.target.read_bytes(), content)
        self.assertEqual(list(self.source.parent.glob("ur3e_update_rate.yaml.*")), [])
        result = self.run_script("apply_ur_update_rate.sh")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("already active", result.stdout)
        self.assertNotIn("cp", self.commands())

    def test_changed_rate_updates_existing_mounted_inode(self) -> None:
        self.prepare()
        self.mount_current_inode()
        before_inode = self.source.stat().st_ino
        self.prepare("500")
        self.assertEqual(self.source.stat().st_ino, before_inode)
        self.assertEqual(self.source.read_bytes(), self.target.read_bytes())
        self.assertIn(b"update_rate: 500 # Hz", self.target.read_bytes())
        result = self.run_script("apply_ur_update_rate.sh")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("cp", self.commands())

    def test_workspace_driver_uses_same_rate_as_system_mount(self) -> None:
        overlay_config = self.root / "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/config"
        overlay_config.mkdir(parents=True)
        selected = overlay_config / "ur3e_update_rate.yaml"
        selected.write_text("controller_manager:\n  ros__parameters:\n    update_rate: 500 # Hz\n")
        other_robot = overlay_config / "ur5e_update_rate.yaml"
        other_robot.write_text("keep this robot configuration\n")
        self.prepare()
        self.assertEqual(selected.read_bytes(), self.source.read_bytes())
        before = selected.stat()
        self.prepare()
        after = selected.stat()
        self.assertEqual((before.st_ino, before.st_mtime_ns), (after.st_ino, after.st_mtime_ns))
        self.prepare("250")
        self.assertEqual(selected.stat().st_ino, before.st_ino)
        self.assertEqual(selected.read_bytes(), self.source.read_bytes())
        self.assertEqual(other_robot.read_text(), "keep this robot configuration\n")

    def test_stale_read_only_bind_fails_with_recreate_diagnostic_without_copy(self) -> None:
        self.prepare()
        self.mount_current_inode()
        mounted_content = self.target.read_bytes()
        replacement = self.source.with_suffix(".replacement")
        replacement.write_bytes(mounted_content.replace(b"125", b"500"))
        os.replace(replacement, self.source)  # Previous prepare script's rename.
        self.prepare("500")
        result = self.run_script("apply_ur_update_rate.sh")
        self.assertEqual(result.returncode, 78, result.stdout + result.stderr)
        self.assertIn("read-only container mount", result.stderr)
        self.assertIn("Recreate the ROS container", result.stderr)
        self.assertEqual(self.target.read_bytes(), mounted_content)
        self.assertNotIn("cp", self.commands())

    def test_writable_legacy_target_can_still_receive_override(self) -> None:
        self.prepare()
        self.target.write_text("previous writable configuration\n")
        result = self.run_script(
            "apply_ur_update_rate.sh", UPDATE_RATE_TEST_MOUNT_OPTIONS="rw,relatime",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.target.read_bytes(), self.source.read_bytes())
        self.assertIn("cp", self.commands())

    def test_uninspectable_mount_does_not_attempt_copy(self) -> None:
        self.prepare()
        self.target.write_text("previous configuration\n")
        result = self.run_script(
            "apply_ur_update_rate.sh", UPDATE_RATE_TEST_MOUNT_PROBE_FAILURE="true",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("Cannot inspect", result.stderr)
        self.assertNotIn("cp", self.commands())


if __name__ == "__main__":
    unittest.main()
