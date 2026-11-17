"""Local artifact selection and mount guards; no Docker daemon or hardware."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


PROJECT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("ros_artifact_cache", PROJECT / "scripts/prepare_ros_artifact_cache.py")
cache = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cache)


def inspected(mounts: list[dict]) -> list[dict]:
    return [{
        "Config": {"Labels": {"com.docker.compose.project": "ur3e-real",
                                "com.docker.compose.service": "ros2"}},
        "State": {"Running": True}, "Mounts": mounts,
    }]


def bind(source: Path, destination: str, writable: bool = True) -> dict:
    return {"Type": "bind", "Source": str(source), "Destination": destination, "RW": writable}


class ArtifactCacheTests(unittest.TestCase):
    @unittest.skipIf(os.name == "nt", "Exercises POSIX symlinks, permissions, or host shell fixtures")
    def test_macos_paths_are_stable_per_canonical_workspace_and_preserve_old_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workspace = root / "Desktop/project/workspaces/ur_ros2_ws"
            for name in ("build", "install", "src", "config", "runtime"):
                (workspace / name).mkdir(parents=True)
                (workspace / name / "sentinel").write_text(name)
            alias = root / "alias"
            alias.symlink_to(workspace, target_is_directory=True)
            with mock.patch.object(cache.platform, "system", return_value="Darwin"), \
                 mock.patch.object(cache.Path, "home", return_value=root / "home"):
                paths = cache.prepare_directories(workspace)
                self.assertEqual(paths, cache.prepare_directories(alias))
                other = cache.prepare_directories(root / "other/workspaces/ur_ros2_ws")
            self.assertNotEqual(paths["UR_ROS_BUILD_DIR"], other["UR_ROS_BUILD_DIR"])
            for path in paths.values():
                self.assertTrue(path.is_relative_to(root / "home/Library/Caches"))
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o755)
                (path / "cached").write_text("keep cache")
            with mock.patch.object(cache.platform, "system", return_value="Darwin"), \
                 mock.patch.object(cache.Path, "home", return_value=root / "home"):
                cache.prepare_directories(workspace)
            for path in paths.values():
                self.assertEqual((path / "cached").read_text(), "keep cache")
            for name in ("build", "install", "src", "config", "runtime"):
                self.assertEqual((workspace / name / "sentinel").read_text(), name)

    def test_other_platforms_keep_workspace_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            for system in ("Linux", "Windows"):
                with self.subTest(system=system), mock.patch.object(cache.platform, "system", return_value=system):
                    paths = cache.prepare_directories(workspace)
                    self.assertEqual(paths["UR_ROS_BUILD_DIR"], workspace.resolve() / "build")
                    self.assertEqual(paths["UR_ROS_INSTALL_DIR"], workspace.resolve() / "install")

    @unittest.skipIf(os.name == "nt", "Exercises POSIX symlinks, permissions, or host shell fixtures")
    def test_macos_existing_cache_link_to_desktop_is_refused_and_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            home = root / "home"
            workspace = root / "Desktop/workspace"
            workspace.mkdir(parents=True)
            with mock.patch.object(cache.platform, "system", return_value="Darwin"), \
                 mock.patch.object(cache.Path, "home", return_value=home):
                paths = cache.artifact_directories(workspace)
                install = paths["UR_ROS_INSTALL_DIR"]
                install.parent.mkdir(parents=True)
                install.symlink_to(workspace, target_is_directory=True)
                with self.assertRaisesRegex(cache.ArtifactCacheError, "must not use a symlink"):
                    cache.prepare_directories(workspace)
                with self.assertRaises(cache.ArtifactCacheError):
                    cache.verify_container_mounts(workspace, inspected([bind(workspace, "/home/ubuntu/ur_ros2_ws")]))
            self.assertTrue(install.is_symlink())
            self.assertEqual(list(workspace.iterdir()), [])

    def test_selected_child_mounts_override_parent_workspace_mount(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "Desktop/workspace"
            with mock.patch.object(cache.platform, "system", return_value="Darwin"), \
                 mock.patch.object(cache.Path, "home", return_value=Path(temporary) / "home"):
                paths = cache.artifact_directories(workspace)
                mounts = [bind(workspace, "/home/ubuntu/ur_ros2_ws")]
                for name in ("build", "install"):
                    mounts.append(bind(paths[f"UR_ROS_{name.upper()}_DIR"], f"/home/ubuntu/ur_ros2_ws/{name}"))
                cache.verify_container_mounts(workspace, inspected(mounts))

    def test_old_macos_parent_mount_refused_without_preparing_or_changing_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workspace = root / "workspace"
            workspace.mkdir()
            with mock.patch.object(cache.platform, "system", return_value="Darwin"), \
                 mock.patch.object(cache.Path, "home", return_value=root / "home"):
                with self.assertRaisesRegex(cache.ArtifactCacheError, "Restart this project's ROS container"):
                    cache.verify_container_mounts(workspace, inspected([bind(workspace, "/home/ubuntu/ur_ros2_ws")]))
            self.assertEqual(list(workspace.iterdir()), [])
            self.assertFalse((root / "home").exists())

    def test_legacy_non_macos_parent_mount_is_compatible(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            with mock.patch.object(cache.platform, "system", return_value="Linux"):
                cache.verify_container_mounts(workspace, inspected([bind(workspace, "/home/ubuntu/ur_ros2_ws")]))

    def test_wrong_or_read_only_child_mount_cannot_hide_behind_matching_parent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            parent = bind(workspace, "/home/ubuntu/ur_ros2_ws")
            with mock.patch.object(cache.platform, "system", return_value="Linux"):
                for child in (bind(workspace / "wrong", "/home/ubuntu/ur_ros2_ws/install"),
                              bind(workspace / "install", "/home/ubuntu/ur_ros2_ws/install", False)):
                    with self.subTest(child=child), self.assertRaises(cache.ArtifactCacheError):
                        cache.verify_container_mounts(workspace, inspected([parent, child]))

    def test_unowned_container_is_refused(self) -> None:
        data = inspected([])
        data[0]["Config"]["Labels"]["com.docker.compose.project"] = "other-project"
        with self.assertRaisesRegex(cache.ArtifactCacheError, "does not belong"):
            cache.verify_container_mounts(Path("/unused"), data)

    @unittest.skipIf(os.name == "nt", "Exercises POSIX symlinks, permissions, or host shell fixtures")
    def test_sourced_exports_quote_spaces_and_shell_characters_and_reach_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "project with ' apostrophe"
            scripts = root / "scripts"
            scripts.mkdir(parents=True)
            for name in ("prepare_ros_artifact_cache.py", "ros_artifact_environment.sh"):
                shutil.copy2(PROJECT / "scripts" / name, scripts / name)
            helper = scripts / "prepare_ros_artifact_cache.py"
            helper.write_text(helper.read_text().replace("Path.home()", "Path(os.environ['UR_ARTIFACT_TEST_HOME'])"))
            home = Path(temporary) / "home with ' quote $literal `literal`"
            env = {**os.environ, "UR_ARTIFACT_TEST_HOME": str(home)}
            result = subprocess.run(
                ["bash", "-c", "set -euo pipefail; source " + shlex.quote(str(scripts / "ros_artifact_environment.sh"))
                 + "; " + shlex.quote(sys.executable) + " -c "
                 + shlex.quote("import json,os; print(json.dumps({key:os.environ[key] for key in ('UR_ROS_BUILD_DIR','UR_ROS_INSTALL_DIR')}))")],
                env=env, capture_output=True, text=True, timeout=5,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            paths = json.loads(result.stdout)
            for variable, name in (("UR_ROS_BUILD_DIR", "build"), ("UR_ROS_INSTALL_DIR", "install")):
                path = Path(paths[variable]).resolve()
                self.assertEqual(path.name, name)
                self.assertTrue(path.is_dir())
                self.assertTrue(path.is_relative_to(home.resolve()) or path.is_relative_to(root.resolve()))
            self.assertFalse((home / "literal").exists())

    def test_all_container_creation_entries_source_environment_before_compose_up(self) -> None:
        for name in ("start_ros.sh", "launch_real_robot.sh", "launch_real_robot_detached.sh",
                     "start_printhead_rviz.sh", "extract_real_robot_calibration.sh"):
            with self.subTest(script=name):
                content = (PROJECT / "scripts" / name).read_text()
                self.assertLess(content.index('source "$ROOT_DIR/scripts/ros_artifact_environment.sh"'), content.index('up -d'))
                self.assertNotIn("prepare_macos_ros_artifacts.py", content)
        for name in ("ensure_workstation_ros_build.sh", "build_print_planner.sh"):
            content = (PROJECT / "scripts" / name).read_text()
            self.assertIn("verify_ros_artifact_mounts", content)
            self.assertNotIn("prepare_macos_ros_artifacts.py", content)
        compose = (PROJECT / "docker-compose.real.yml").read_text()
        for name in ("build", "install"):
            self.assertIn(f"source: ${{UR_ROS_{name.upper()}_DIR:-./workspaces/ur_ros2_ws/{name}}}", compose)
            self.assertIn(f"target: /home/ubuntu/ur_ros2_ws/{name}", compose)


@unittest.skipIf(os.name == "nt", "Executes POSIX shebang fixtures through the host shell")
class ArtifactBuildGuardTests(unittest.TestCase):
    def test_ensure_writes_marker_in_selected_install_and_refuses_stale_mounts_before_build(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "project with spaces"
            scripts = root / "scripts"
            scripts.mkdir(parents=True)
            for name in ("prepare_ros_artifact_cache.py", "ros_artifact_environment.sh", "ensure_workstation_ros_build.sh"):
                shutil.copy2(PROJECT / "scripts" / name, scripts / name)
            helper = scripts / "prepare_ros_artifact_cache.py"
            helper.write_text(helper.read_text().replace("Path.home()", "Path(os.environ['UR_ARTIFACT_TEST_HOME'])"))
            workspace = root / "workspaces/ur_ros2_ws"
            (workspace / "scripts").mkdir(parents=True)
            (workspace / "scripts/verify_controller_overlay.py").write_text("print('new-source-fingerprint')\n")
            trace = root / "build-trace"
            build = scripts / "build_print_planner.sh"
            build.write_text("#!/bin/bash\nprintf 'build\\n' >> " + shlex.quote(str(trace)) + "\n")
            build.chmod(0o755)
            binary = root / "bin"
            binary.mkdir()
            docker = binary / "docker"
            docker.write_text(f"#!{sys.executable}\n" + r'''
import json, os, sys
if sys.argv[1] == "inspect":
    source = os.environ["ARTIFACT_TEST_BAD_SOURCE"] if os.environ.get("ARTIFACT_TEST_BAD_SOURCE") else os.environ["UR_ROS_BUILD_DIR"]
    print(json.dumps([{"Config":{"Labels":{"com.docker.compose.project":"ur3e-real","com.docker.compose.service":"ros2"}},"State":{"Running":True},"Mounts":[
        {"Type":"bind","Source":source,"Destination":"/home/ubuntu/ur_ros2_ws/build","RW":True},
        {"Type":"bind","Source":os.environ["UR_ROS_INSTALL_DIR"],"Destination":"/home/ubuntu/ur_ros2_ws/install","RW":True}]}]))
elif sys.argv[1] != "compose":
    sys.exit("Unexpected Docker operation")
''')
            docker.chmod(0o755)
            env = {**os.environ, "UR_ARTIFACT_TEST_HOME": str(Path(temporary) / "home"),
                   "PATH": str(binary) + os.pathsep + os.environ["PATH"]}
            result = subprocess.run(["bash", str(scripts / "ensure_workstation_ros_build.sh")], env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            selection = subprocess.run([sys.executable, str(scripts / "prepare_ros_artifact_cache.py"), "--workspace", str(workspace), "--json"],
                                       env=env, capture_output=True, text=True, check=True)
            paths = json.loads(selection.stdout)
            marker = Path(paths["UR_ROS_INSTALL_DIR"]) / ".workstation_ros_bridge_build"
            self.assertEqual(marker.read_text(), "new-source-fingerprint\n")
            self.assertEqual(trace.read_text(), "build\n")
            stamp = marker.stat().st_mtime_ns
            result = subprocess.run(["bash", str(scripts / "ensure_workstation_ros_build.sh")], env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("up to date", result.stdout)
            self.assertEqual(marker.stat().st_mtime_ns, stamp)
            self.assertEqual(trace.read_text(), "build\n")
            result = subprocess.run(["bash", str(scripts / "ensure_workstation_ros_build.sh")],
                                    env={**env, "ARTIFACT_TEST_BAD_SOURCE": str(workspace / "wrong")},
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 78, result.stdout + result.stderr)
            self.assertIn("Restart this project's ROS container", result.stderr)
            self.assertEqual(trace.read_text(), "build\n")


if __name__ == "__main__":
    unittest.main()
