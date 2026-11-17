from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
INSTALL_RUNTIME = PROJECT_ROOT / "scripts" / "install_runtime.sh"


class InstallRuntimeTests(unittest.TestCase):
    def run_isolated_runtime(
        self, *, arch: str = "arm64", colima_status: int = 0,
        rosetta_fail: bool = False, context_lost: bool = False,
        daemon_unreachable: bool = False, ssh_port: int | None = None,
        running_container: str = "", control_process: str = "",
    ) -> tuple[subprocess.CompletedProcess[str], str, dict, list[str]]:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            fake_bin = root / "fake-bin"
            brew_prefix = root / "homebrew"
            home_dir = root / "home"
            docker_source = root / "docker-source"
            brew_log = root / "brew.log"
            colima_log = root / "colima.log"
            fake_bin.mkdir()
            (brew_prefix / "bin").mkdir(parents=True)
            home_dir.mkdir()
            if ssh_port is not None:
                ssh_config = home_dir / ".colima/_lima/colima/ssh.config"
                ssh_config.parent.mkdir(parents=True)
                ssh_config.write_text(f"Port {ssh_port}\n")

            docker_source.write_text(
                "#!/bin/sh\n"
                "case \"$1\" in\n"
                "  version)\n"
                "    [ \"$FAKE_DAEMON_UNREACHABLE\" = 1 ] && exit 1\n"
                "    if [ \"$FAKE_CONTEXT_LOST\" = 1 ] && [ ! -f \"$FAKE_CONTEXT_SELECTED\" ]; then exit 1; fi\n"
                "    exit 0 ;;\n"
                "  --context) [ \"$FAKE_DAEMON_UNREACHABLE\" = 1 ] && exit 1; exit 0 ;;\n"
                "  context)\n"
                "    if [ \"$2\" = inspect ]; then printf 'unix://%s/.colima/default/docker.sock\\n' \"$HOME\"; fi\n"
                "    if [ \"$2\" = show ]; then printf 'colima\\n'; fi\n"
                "    if [ \"$2\" = use ] && [ \"$3\" = colima ]; then touch \"$FAKE_CONTEXT_SELECTED\"; fi\n"
                "    exit 0 ;;\n"
                "  ps) [ -n \"$FAKE_RUNNING_CONTAINER\" ] && printf '%s\\n' \"$FAKE_RUNNING_CONTAINER\"; exit 0 ;;\n"
                "  inspect) printf 'ur3e-real ros2\\n'; exit 0 ;;\n"
                "  top) printf '%s\\n' \"$FAKE_CONTROL_PROCESS\"; exit 0 ;;\n"
                "  compose) exit 0 ;;\n"
                "esac\n"
                "exit 0\n",
                encoding="utf-8",
            )
            docker_source.chmod(0o755)

            (fake_bin / "brew").write_text(
                "#!/bin/sh\n"
                "printf '%s\\n' \"$*\" >> \"$FAKE_BREW_LOG\"\n"
                "if [ \"$1\" = \"--prefix\" ]; then\n"
                "  printf '%s\\n' \"$FAKE_BREW_PREFIX\"\n"
                "  exit 0\n"
                "fi\n"
                "if [ \"$1\" = \"list\" ]; then exit 0; fi\n"
                "if [ \"$1\" = \"unlink\" ] && [ \"$2\" = \"docker-completion\" ]; then exit 0; fi\n"
                "if [ \"$1\" = \"link\" ] && [ \"$2\" = \"docker\" ]; then\n"
                "  ln -sf \"$FAKE_DOCKER_SOURCE\" \"$FAKE_BREW_PREFIX/bin/docker\"\n"
                "  exit 0\n"
                "fi\n"
                "exit 1\n",
                encoding="utf-8",
            )
            (fake_bin / "brew").chmod(0o755)
            (fake_bin / "colima").write_text(
                "#!/bin/sh\n"
                "printf '%s\\n' \"$*\" >> \"$FAKE_COLIMA_LOG\"\n"
                "case \"$1\" in\n"
                "  status) exit \"$FAKE_COLIMA_STATUS\" ;;\n"
                "  start) if [ \"$FAKE_ROSETTA_FAIL\" = 1 ]; then\n"
                "    case \" $* \" in *' --vz-rosetta '*) exit 1 ;; esac\n"
                "  fi ;;\n"
                "  *) exit 0 ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            (fake_bin / "colima").chmod(0o755)
            (fake_bin / "uname").write_text(
                f"#!/bin/sh\nprintf '%s\\n' '{arch}'\n", encoding="utf-8"
            )
            (fake_bin / "uname").chmod(0o755)
            (fake_bin / "ps").write_text("#!/bin/sh\nprintf 'desktop-only\\n'\n")
            (fake_bin / "ps").chmod(0o755)

            env = os.environ.copy()
            for name in ("DOCKER_HOST", "DOCKER_CONTEXT", "COLIMA_PROFILE"):
                env.pop(name, None)
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "HOME": str(home_dir),
                    "FAKE_BREW_PREFIX": str(brew_prefix),
                    "FAKE_BREW_LOG": str(brew_log),
                    "FAKE_DOCKER_SOURCE": str(docker_source),
                    "FAKE_COLIMA_LOG": str(colima_log),
                    "FAKE_COLIMA_STATUS": str(colima_status),
                    "FAKE_ROSETTA_FAIL": "1" if rosetta_fail else "0",
                    "FAKE_CONTEXT_LOST": "1" if context_lost else "0",
                    "FAKE_DAEMON_UNREACHABLE": "1" if daemon_unreachable else "0",
                    "FAKE_CONTEXT_SELECTED": str(root / "selected-context"),
                    "FAKE_RUNNING_CONTAINER": running_container,
                    "FAKE_CONTROL_PROCESS": control_process,
                }
            )
            completed = subprocess.run(
                ["/bin/bash", str(INSTALL_RUNTIME)],
                cwd=PROJECT_ROOT,
                env=env,
                capture_output=True,
                text=True,
                timeout=10,
            )

            brew_commands = brew_log.read_text(encoding="utf-8")
            docker_config = json.loads(
                (home_dir / ".docker" / "config.json").read_text(encoding="utf-8")
            )
            colima_commands = colima_log.read_text(encoding="utf-8").splitlines()
            return completed, brew_commands, docker_config, colima_commands

    def test_repairs_installed_but_unlinked_docker_cli(self) -> None:
        completed, commands, docker_config, _ = self.run_isolated_runtime()
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("unlink docker-completion", commands)
        self.assertIn("link docker", commands)
        self.assertEqual(len(docker_config["cliPluginsExtraDirs"]), 1)
        self.assertTrue(docker_config["cliPluginsExtraDirs"][0].endswith(
            "/homebrew/lib/docker/cli-plugins"
        ))
        self.assertNotIn("/opt/homebrew/lib/docker/cli-plugins", docker_config["cliPluginsExtraDirs"])

    def test_intel_first_start_uses_native_arch_without_rosetta(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(
            arch="x86_64", colima_status=1,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        starts = [command for command in commands if command.startswith("start ")]
        self.assertEqual(len(starts), 1)
        self.assertIn("--arch x86_64", starts[0])
        self.assertNotIn("--vz-rosetta", starts[0])

    def test_apple_silicon_falls_back_if_rosetta_start_fails(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(
            arch="arm64", colima_status=1, rosetta_fail=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        starts = [command for command in commands if command.startswith("start ")]
        self.assertEqual(len(starts), 2)
        self.assertIn("--arch aarch64 --vm-type vz --vz-rosetta", starts[0])
        self.assertIn("--arch aarch64", starts[1])
        self.assertNotIn("--vz-rosetta", starts[1])

    def test_start_after_canceled_colima_restart_restores_context_without_restarting_vm(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(context_lost=True)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertNotIn("stop", commands)
        self.assertFalse(any(command.startswith("start ") for command in commands))

    def test_unreachable_daemon_does_not_blindly_restart_unknown_workloads(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(daemon_unreachable=True)
        self.assertNotEqual(completed.returncode, 0)
        self.assertNotIn("stop", commands)
        self.assertFalse(any(command.startswith("start ") for command in commands))

    def test_ssh_port_repair_refuses_to_restart_other_containers(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(
            ssh_port=50004, running_container="unrelated-database",
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertNotIn("stop", commands)

    def test_ssh_port_repair_refuses_active_robot_control(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(
            ssh_port=50004, running_container="ur3e-ros2-real",
            control_process="/opt/ros/humble/lib/ur_robot_driver/ur_ros2_control_node",
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertNotIn("stop", commands)

    @unittest.skipUnless(sys.platform == "darwin", "Colima restart guard is macOS-specific")
    def test_ssh_port_can_be_repaired_when_vm_has_no_workloads(self) -> None:
        completed, _, _, commands = self.run_isolated_runtime(ssh_port=50004)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(commands.count("stop"), 1)


if __name__ == "__main__":
    unittest.main()
