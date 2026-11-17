"""Offline build/deployment guard tests; no Docker, ROS nodes, or hardware."""
import importlib.util
import json
import os
import shlex
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

PROJECT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT / "workspaces/ur_ros2_ws/scripts/verify_controller_overlay.py"


class ControllerOverlayTests(unittest.TestCase):
    def module(self):
        spec = importlib.util.spec_from_file_location("overlay_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def abi_workspace(self, root):
        workspace = root / "workspace"
        config = (
            workspace / "install/ur_client_library/share/ur_client_library/cmake"
            / "ur_client_libraryConfig.cmake"
        )
        config.parent.mkdir(parents=True)
        config.write_text("# workspace package\n")
        urcl_library = workspace / "install/ur_client_library/lib/liburcl.so"
        urcl_library.parent.mkdir(parents=True)
        urcl_library.write_bytes(b"workspace urcl")
        driver_library = workspace / "install/ur_robot_driver/lib/libur_robot_driver_plugin.so"
        driver_library.parent.mkdir(parents=True)
        driver_library.write_bytes(b"workspace driver")
        cache = workspace / "build/ur_robot_driver/CMakeCache.txt"
        cache.parent.mkdir(parents=True)
        cache.write_text(f"ur_client_library_DIR:PATH={config.parent}\n")
        return workspace, driver_library, urcl_library, cache

    def abi_command_results(self, driver_library, urcl_library, driver_offset, urcl_offset):
        return [
            subprocess.CompletedProcess(
                ["ldd", str(driver_library)], 0,
                f"liburcl.so => {urcl_library} (0x0000)\n", "",
            ),
            subprocess.CompletedProcess(
                ["gdb"], 0,
                f"/* {driver_offset} | 8 */ double target_frequency_;\n", "",
            ),
            subprocess.CompletedProcess(
                ["gdb"], 0,
                f"/* {urcl_offset} | 8 */ double target_frequency_;\n", "",
            ),
        ]

    def test_rejects_system_plugin_even_if_it_has_same_package_version(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "workspace"):
                module.verify(Path(directory), Path("/opt/ros/humble"), load=False)

    def test_missing_or_unpatched_binary_fails_closed(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/ur_controllers"
            with self.assertRaises(RuntimeError):
                module.verify(workspace, prefix, load=False)
            library = prefix / "lib/libur_controllers.so"
            library.parent.mkdir(parents=True)
            library.write_bytes(b"old-system-build")
            with self.assertRaises(RuntimeError):
                module.verify(workspace, prefix, load=False)
            library.write_bytes(b"patched-cancel-deactivate-v2-build")
            self.assertEqual(module.verify(workspace, prefix, load=False), library)

    def test_missing_or_unpatched_driver_binary_fails_closed(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/ur_robot_driver"
            with self.assertRaises(RuntimeError):
                module.verify_driver(workspace, prefix, load=False)
            library = prefix / "lib/libur_robot_driver_plugin.so"
            library.parent.mkdir(parents=True)
            library.write_bytes(b"old-system-build")
            with self.assertRaises(RuntimeError):
                module.verify_driver(workspace, prefix, load=False)
            library.write_bytes(b"large-trajectory-rt-copy-v1-build")
            self.assertEqual(module.verify_driver(workspace, prefix, load=False), library)

    def test_ur_client_library_rejects_system_and_unpatched_binaries(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/ur_client_library"
            with self.assertRaisesRegex(RuntimeError, "workspace"):
                module.verify_urcl(workspace, Path("/opt/ros/humble"), load=False)
            with self.assertRaises(RuntimeError):
                module.verify_urcl(workspace, prefix, load=False)
            library = prefix / "lib/liburcl.so"
            library.parent.mkdir(parents=True)
            library.write_bytes(b"old-system-build")
            with self.assertRaises(RuntimeError):
                module.verify_urcl(workspace, prefix, load=False)
            library.write_bytes(b"[urcl-framing-v1] patched-build")
            with mock.patch.object(module.ctypes, "CDLL") as load_library:
                self.assertEqual(module.verify_urcl(workspace, prefix), library)
            load_library.assert_called_once_with(str(library))

    def test_ur_client_library_accepts_single_multiarch_binary(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            prefix = workspace / "install/ur_client_library"
            library = prefix / "lib/x86_64-linux-gnu/liburcl.so"
            library.parent.mkdir(parents=True)
            library.write_bytes(b"[urcl-framing-v1] patched-build")
            self.assertEqual(module.verify_urcl(workspace, prefix, load=False), library)
            second = prefix / "lib/aarch64-linux-gnu/liburcl.so"
            second.parent.mkdir(parents=True)
            second.write_bytes(b"[urcl-framing-v1] patched-build")
            with self.assertRaisesRegex(RuntimeError, "ambiguous"):
                module.verify_urcl(workspace, prefix, load=False)

    def test_ur_client_library_rejects_symlink_to_binary_outside_workspace(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workspace = root / "workspace"
            library = workspace / "install/ur_client_library/lib/liburcl.so"
            library.parent.mkdir(parents=True)
            system_library = root / "system/liburcl.so"
            system_library.parent.mkdir()
            system_library.write_bytes(b"[urcl-framing-v1] copied-marker")
            library.symlink_to(system_library)
            with self.assertRaisesRegex(RuntimeError, "outside the workspace"):
                module.verify_urcl(workspace, workspace / "install/ur_client_library", load=False)

    def test_driver_urcl_abi_rejects_system_cmake_cache(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace, driver_library, _, cache = self.abi_workspace(Path(directory))
            cache.write_text(
                "ur_client_library_DIR:PATH=/opt/ros/humble/share/ur_client_library/cmake\n"
            )
            with mock.patch.object(module.subprocess, "run") as run:
                with self.assertRaisesRegex(RuntimeError, "configured against a different"):
                    module.verify_driver_urcl_abi(workspace, driver_library)
            run.assert_not_called()

    def test_driver_urcl_abi_rejects_system_runtime_library(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace, driver_library, _, _ = self.abi_workspace(Path(directory))
            ldd = subprocess.CompletedProcess(
                ["ldd", str(driver_library)], 0,
                "liburcl.so => /opt/ros/humble/lib/liburcl.so (0x0000)\n", "",
            )
            with mock.patch.object(module.subprocess, "run", return_value=ldd) as run:
                with self.assertRaisesRegex(RuntimeError, "does not resolve the workspace"):
                    module.verify_driver_urcl_abi(workspace, driver_library)
            run.assert_called_once_with(
                ["ldd", str(driver_library)], capture_output=True, text=True, check=False
            )

    def test_driver_urcl_abi_rejects_compiled_rtde_layout_mismatch(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace, driver_library, urcl_library, _ = self.abi_workspace(Path(directory))
            outputs = self.abi_command_results(driver_library, urcl_library, 112, 120)
            with mock.patch.object(module.subprocess, "run", side_effect=outputs) as run:
                with self.assertRaisesRegex(RuntimeError, "ABI mismatch.*byte 112.*byte 120"):
                    module.verify_driver_urcl_abi(workspace, driver_library)
            self.assertEqual([call.args[0][0] for call in run.call_args_list], ["ldd", "gdb", "gdb"])

    def test_driver_urcl_abi_accepts_matching_cache_dependency_and_layout(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace, driver_library, urcl_library, _ = self.abi_workspace(Path(directory))
            outputs = self.abi_command_results(driver_library, urcl_library, 120, 120)
            with mock.patch.object(module.subprocess, "run", side_effect=outputs) as run:
                self.assertIsNone(module.verify_driver_urcl_abi(workspace, driver_library))
            self.assertEqual([call.args[0][0] for call in run.call_args_list], ["ldd", "gdb", "gdb"])

    def test_overlay_cli_checks_selected_ur_client_library(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            package_prefixes = {
                name: str(workspace / "install" / name)
                for name in ("ur_controllers", "ur_robot_driver", "ur_client_library")
            }
            packages = types.ModuleType("ament_index_python.packages")
            packages.get_package_prefix = package_prefixes.__getitem__
            ament = types.ModuleType("ament_index_python")
            with mock.patch.dict(sys.modules, {
                "ament_index_python": ament,
                "ament_index_python.packages": packages,
            }):
                with mock.patch.object(module, "verify", return_value=Path("controller")):
                    with mock.patch.object(module, "verify_driver", return_value=Path("driver")):
                        with mock.patch.object(module, "verify_urcl", return_value=Path("urcl")) as check_urcl:
                            with mock.patch.object(sys, "argv", ["verify_controller_overlay.py", "--workspace", str(workspace)]):
                                with mock.patch("builtins.print"):
                                    module.main()
            check_urcl.assert_called_once_with(workspace, workspace / "install/ur_client_library")

    def test_deactivation_flushes_terminal_action_without_timer(self):
        """Controller-manager deactivation removes timer progress; flush the result inline."""
        source = (
            PROJECT
            / "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver"
            / "ur_controllers/src/passthrough_trajectory_controller.cpp"
        ).read_text()
        start = source.index("PassthroughTrajectoryController::on_deactivate")
        end = source.index("PassthroughTrajectoryController::update", start)
        deactivation = source[start:end]
        finish = deactivation.index("finish_goal(active_goal")
        flush = deactivation.index("active_goal->runNonRealtime()")
        self.assertLess(finish, flush)

    def test_large_passthrough_transfer_avoids_vector_copies_and_bounds_writes(self):
        """Large print trajectories must not be copied or indexed unchecked in the RT loop."""
        driver = (
            PROJECT
            / "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver"
            / "ur_robot_driver/src/hardware_interface.cpp"
        ).read_text()
        header = (
            PROJECT
            / "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver"
            / "ur_robot_driver/include/ur_robot_driver/hardware_interface.hpp"
        ).read_text()

        const_ref = "const std::vector<std::array<double, 6>>& data"
        self.assertIn(const_ref, header)
        self.assertIn(const_ref, driver)

        transfer = driver.index("passthrough_trajectory_transfer_state_ == 2.0")
        first_write = driver.index("trajectory_joint_positions_[point_index_received]", transfer)
        bounds_guard = driver.index(
            "point_index_received >= trajectory_joint_positions_.size()", transfer
        )
        self.assertLess(bounds_guard, first_write)

    def test_mac_build_and_cache_include_actual_controller(self):
        build = (PROJECT / "scripts/build_print_planner.sh").read_text()
        ensure = (PROJECT / "scripts/ensure_workstation_ros_build.sh").read_text()
        interactive = (PROJECT / "scripts/launch_real_robot.sh").read_text()
        self.assertIn("ur_controllers", build)
        self.assertIn("ur_client_library", build)
        self.assertIn("ur_robot_driver", build)
        self.assertIn("verify_controller_overlay.py", build)
        self.assertIn("--fingerprint", ensure)
        self.assertIn("libur_controllers.so", ensure)
        self.assertIn("liburcl.so", ensure)
        self.assertIn("libur_robot_driver_plugin.so", ensure)
        self.assertIn("ensure_workstation_ros_build.sh", interactive)

    def test_fingerprint_ignores_generated_metadata_but_detects_preserved_mtime_edits(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            for relative in module.SOURCE_DIRS:
                (workspace / relative).mkdir(parents=True)
            for relative in module.SOURCE_FILES:
                configuration = workspace / relative
                configuration.parent.mkdir(parents=True, exist_ok=True)
                configuration.write_text("{}")
            source = workspace / module.SOURCE_DIRS[0] / "definition.action"
            source.write_text("original")
            stamp = source.stat().st_mtime_ns
            before = module.fingerprint(workspace)
            generated = workspace / module.SOURCE_DIRS[1] / "package.egg-info/PKG-INFO"
            generated.parent.mkdir()
            generated.write_text("generated during build")
            self.assertEqual(before, module.fingerprint(workspace))
            source.write_text("edited with preserved timestamp")
            os.utime(source, ns=(stamp, stamp))
            self.assertNotEqual(before, module.fingerprint(workspace))

    def test_fingerprint_tracks_ur_client_library_source(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            for relative in module.SOURCE_DIRS:
                (workspace / relative).mkdir(parents=True)
            for relative in module.SOURCE_FILES:
                configuration = workspace / relative
                configuration.parent.mkdir(parents=True, exist_ok=True)
                configuration.write_text("{}")
            source = workspace / "src/Universal_Robots_Client_Library/include/ur_client_library/comm/stream.h"
            source.parent.mkdir(parents=True)
            source.write_text("old framing")
            before = module.fingerprint(workspace)
            source.write_text("[urcl-framing-v1] patched framing")
            self.assertNotEqual(before, module.fingerprint(workspace))

    def test_geometry_configuration_edit_revokes_the_ros_build(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            for relative in module.SOURCE_DIRS:
                (workspace / relative).mkdir(parents=True)
            configuration = workspace / "config/printhead_geometry.json"
            configuration.parent.mkdir()
            configuration.write_text('{"geometry_revision":"old"}')
            before = module.fingerprint(workspace)
            stamp = configuration.stat().st_mtime_ns
            configuration.write_text('{"geometry_revision":"new"}')
            os.utime(configuration, ns=(stamp, stamp))
            self.assertNotEqual(before, module.fingerprint(workspace))

    def test_linux_install_symlink_does_not_force_rebuild_on_mac_host(self):
        # Colcon uses absolute Linux symlinks: -f on macOS cannot resolve them.
        # Docker is an inert stub here; an unnecessary build fails the test.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / "scripts"
            scripts.mkdir()
            ensure = scripts / "ensure_workstation_ros_build.sh"
            ensure.write_text((PROJECT / "scripts/ensure_workstation_ros_build.sh").read_text())
            for name in ("prepare_ros_artifact_cache.py", "ros_artifact_environment.sh"):
                (scripts / name).write_text((PROJECT / "scripts" / name).read_text())
            build = scripts / "build_print_planner.sh"
            build.write_text("#!/bin/sh\nexit 91\n")
            build.chmod(0o755)
            workspace = root / "workspaces/ur_ros2_ws"
            (workspace / "scripts").mkdir(parents=True)
            (workspace / "scripts/verify_controller_overlay.py").write_text("print('same-fingerprint')\n")
            library = workspace / "install/ur_controllers/lib/libur_controllers.so"
            library.parent.mkdir(parents=True)
            library.symlink_to("/home/ubuntu/ur_ros2_ws/build/ur_controllers/libur_controllers.so")
            (workspace / "install/.workstation_ros_bridge_build").write_text("same-fingerprint\n")
            binary = root / "bin"
            binary.mkdir()
            docker = binary / "docker"
            docker.write_text(
                "#!/bin/sh\nif [ \"$1\" = inspect ]; then\n" +
                "  printf '%s\\n' " + shlex.quote(json.dumps([{
                    "Config": {"Labels": {"com.docker.compose.project": "ur3e-real", "com.docker.compose.service": "ros2"}},
                    "State": {"Running": True},
                    "Mounts": [{"Type": "bind", "Source": str(workspace), "Destination": "/home/ubuntu/ur_ros2_ws", "RW": True}],
                }])) + "\nfi\nexit 0\n"
            )
            helper = scripts / "prepare_ros_artifact_cache.py"
            helper.write_text(helper.read_text().replace('if platform.system() == "Darwin":', 'if False:'))
            docker.chmod(0o755)
            result = subprocess.run(["bash", str(ensure)], capture_output=True, text=True,
                                    env={**os.environ, "PATH": str(binary) + os.pathsep + os.environ["PATH"]})
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("up to date", result.stdout)


if __name__ == "__main__":
    unittest.main()
