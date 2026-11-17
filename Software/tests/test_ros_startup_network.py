"""Exercise the real Mac launch entry points with external commands isolated.

These tests never start Docker, contact a robot, or execute a motion command.
The separate live acceptance check must still verify Colima's actual forwarding.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest


PROJECT = Path(__file__).resolve().parents[2]


class MacROSStartupTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.scripts = self.root / "scripts"
        self.scripts.mkdir()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.trace = self.root / "commands.jsonl"
        self.control_state = self.root / "partial-control-stack"
        for name in ("launch_real_robot.sh", "launch_real_robot_detached.sh"):
            shutil.copy2(PROJECT / "scripts" / name, self.scripts / name)
        for name in ("prepare_ros_artifact_cache.py", "ros_artifact_environment.sh"):
            shutil.copy2(PROJECT / "scripts" / name, self.scripts / name)
        helper = self.scripts / "prepare_ros_artifact_cache.py"
        helper.write_text(helper.read_text().replace("Path.home()", "Path(os.environ['UR_ARTIFACT_TEST_HOME'])"))
        probe = PROJECT / "scripts" / "check_ros_container_network.py"
        if probe.is_file():
            shutil.copy2(probe, self.scripts / probe.name)
        for name in (
            "install_runtime.sh", "prepare_ur_update_rate.sh",
            "apply_ur_update_rate.sh", "apply_ur_driver_runtime_patches.sh",
            "ensure_workstation_ros_build.sh", "cleanup_ur_driver.sh",
        ):
            path = self.scripts / name
            path.write_text(f"#!/bin/bash\nexec startup_stage {name}\n")
            path.chmod(0o755)
        bridge = self.scripts / "launch_klipper_bridge_detached.sh"
        bridge.write_text("#!/bin/bash\nexec startup_bridge\n")
        bridge.chmod(0o755)
        stop = self.scripts / "stop_ros_driver_detached.sh"
        stop.write_text("#!/bin/bash\nexec startup_cleanup\n")
        stop.chmod(0o755)
        fake = self.bin / "command"
        fake.write_text(
            f"#!{sys.executable}\n" + r'''
import json, os, pathlib, sys, time
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["STARTUP_TEST_TRACE"], "a") as log:
    log.write(json.dumps({"command": name, "args": args,
                         "bind": os.environ.get("UR_REVERSE_BIND_IP")}) + "\n")
if name == "startup_stage":
    stage = args[0]
    if os.environ.get("STARTUP_TEST_PAUSE_STAGE") == stage:
        # Model a remote cleanup/build operation whose effects outlive its CLI.
        if stage == "cleanup_ur_driver.sh":
            pathlib.Path(os.environ["STARTUP_TEST_CONTROL_STATE"]).touch()
        with open(os.environ["STARTUP_TEST_TRACE"], "a") as log:
            log.write(json.dumps({"command": "stage_waiting", "args": [stage]}) + "\n")
        while True:
            time.sleep(1)
    if os.environ.get("STARTUP_TEST_FAIL_STAGE") == stage:
        sys.exit(86)
elif name == "startup_cleanup":
    pathlib.Path(os.environ["STARTUP_TEST_CONTROL_STATE"]).unlink(missing_ok=True)
elif name == "route":
    print("interface: en7")
elif name == "ipconfig":
    print("192.168.0.105")
elif name == "networksetup":
    print("(1) USB Ethernet\n(Hardware Port: USB Ethernet, Device: en7)")
elif name == "docker" and "up" in args:
    # Stop at the actual compose boundary. Nothing may launch a real driver.
    sys.exit(int(os.environ.get("STARTUP_TEST_UP_EXIT", "81")))
elif name == "docker" and args[:2] == ["image", "inspect"]:
    sys.exit(int(os.environ.get("STARTUP_TEST_IMAGE_EXIT", "0")))
elif name == "docker" and args[:2] == ["context", "inspect"]:
    print("unix://" + str(pathlib.Path.home() / ".colima/default/docker.sock"))
elif name == "docker" and args[:2] == ["context", "show"]:
    print("colima")
elif name == "docker" and args[:2] == ["context", "use"]:
    if args[2:] != ["colima"]:
        sys.exit(83)
    pathlib.Path(os.environ["STARTUP_TEST_CONTEXT_LOST"]).unlink(missing_ok=True)
elif name == "docker" and args[0] == "start":
    if pathlib.Path(os.environ["STARTUP_TEST_CONTEXT_LOST"]).exists():
        print("Cannot connect to the default Docker endpoint", file=sys.stderr)
        sys.exit(83)
elif name == "docker" and args[0] == "ps":
    print("ur3e-ros2-real")
elif name == "docker" and args[0] == "top":
    if args[2:] != ["-eo", "pid,args"]:
        print("Couldn't find PID field in ps output", file=sys.stderr)
        sys.exit(1)
    print("PID COMMAND\n1 /sbin/init")
elif name == "docker" and args[:2] == ["inspect", "--format"]:
    print("ur3e-real ros2")
elif name == "colima" and args[0] == "ssh":
    print("28")
    sys.exit(1)
elif name == "colima" and args[0] == "stop":
    pathlib.Path(os.environ["STARTUP_TEST_CONTEXT_LOST"]).touch()
elif name == "colima" and args[0] == "start":
    pathlib.Path(os.environ["STARTUP_TEST_NETWORK_RECOVERED"]).touch()
elif name == "docker" and args[0] == "inspect":
    networks = {} if os.environ.get("STARTUP_TEST_NETWORK") == "missing" else {
        "ur3e-real_default": {"IPAddress": "172.19.0.2"}}
    print(json.dumps([{"State": {"Running": True},
        "Config": {"Labels": {"com.docker.compose.project": "ur3e-real",
                               "com.docker.compose.service": "ros2"}},
        "NetworkSettings": {"Networks": networks, "Ports": {
            str(p) + "/tcp": [{"HostIp": "0.0.0.0", "HostPort": str(p)}]
            for p in (50001, 50002, 50003, 50004)}}}]))
elif name == "docker" and "route" in args:
    if os.environ.get("STARTUP_TEST_ROUTE") == "missing":
        print("RTNETLINK answers: Network is unreachable", file=sys.stderr)
        sys.exit(2)
    print("192.168.0.106 via 172.19.0.1 dev eth0 src 172.19.0.2")
elif name == "docker" and "curl" in args:
    if os.environ.get("STARTUP_TEST_MOONRAKER") == "offline":
        sys.exit(7)
    if (os.environ.get("STARTUP_TEST_MOONRAKER") == "stale_vm" and
            not pathlib.Path(os.environ["STARTUP_TEST_NETWORK_RECOVERED"]).exists()):
        sys.exit(28)
    if os.environ.get("STARTUP_TEST_MOONRAKER") == "heater_shutdown":
        print(json.dumps({"result": {"state": "shutdown", "state_message":
            "Heater extruder not heating at expected rate.\nFIRMWARE_RESTART required"}}))
    elif os.environ.get("STARTUP_TEST_MOONRAKER") == "untrusted_shutdown":
        print(json.dumps({"result": {"state": "shutdown\u001b[31m", "state_message":
            "Heater extruder failed\u001b[31m\n" + "more detail " * 100}}))
    else:
        print('{"result": {"state": "ready"}}')
elif name == "docker" and "ros2 launch ur_robot_driver" in " ".join(args):
    print("FIXTURE: driver boundary reached; no hardware contacted")
    sys.exit(int(os.environ.get("STARTUP_TEST_DRIVER_EXIT", "82")))
'''
        )
        fake.chmod(0o755)
        for name in ("docker", "colima", "ps", "ping", "route", "ipconfig", "networksetup", "startup_cleanup", "startup_bridge", "startup_stage"):
            (self.bin / name).symlink_to(fake)

    def launch_environment(self, **extra: str) -> dict[str, str]:
        env = dict(os.environ)
        env.update(
            PATH=f"{self.bin}:{os.environ['PATH']}",
            UR_ARTIFACT_TEST_HOME=str(self.root / "home"),
            STARTUP_TEST_TRACE=str(self.trace),
            STARTUP_TEST_CONTROL_STATE=str(self.control_state),
            STARTUP_TEST_NETWORK_RECOVERED=str(self.root / "network-recovered"),
            STARTUP_TEST_CONTEXT_LOST=str(self.root / "context-lost"),
            ROBOT_IP="192.168.0.106", ROS_PC_IP="192.168.0.105",
            UR_REVERSE_BIND_IP="192.168.0.105", BUILD_IMAGE="false",
            MOONRAKER_URL="http://192.168.0.112", WAIT_FOR_READY="false",
            HEADLESS_MODE="false",
            DOCKER_HOST="", DOCKER_CONTEXT="", COLIMA_PROFILE="default",
        )
        env.update(extra)
        return env

    def launch(self, name: str, **extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", str(self.scripts / name)], env=self.launch_environment(**extra),
            text=True, capture_output=True, timeout=10,
        )

    def cancel_during_stage(self, stage: str) -> subprocess.CompletedProcess[str]:
        process = subprocess.Popen(
            ["bash", str(self.scripts / "launch_real_robot_detached.sh")],
            env=self.launch_environment(
                STARTUP_TEST_UP_EXIT="0", STARTUP_TEST_PAUSE_STAGE=stage,
            ),
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if self.trace.exists() and any(
                    c["command"] == "stage_waiting" and c["args"] == [stage]
                    for c in self.commands()
                ):
                    break
                if process.poll() is not None:
                    self.fail(f"Launcher exited before {stage}: {process.communicate()}")
                time.sleep(0.01)
            else:
                self.fail(f"Launcher did not reach {stage}")
            os.killpg(process.pid, signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=3)
            return subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=3)

    def commands(self) -> list[dict]:
        return [json.loads(line) for line in self.trace.read_text().splitlines()]

    def test_mac_launch_does_not_bind_mac_only_address_inside_linux_vm(self) -> None:
        for name in ("launch_real_robot_detached.sh", "launch_real_robot.sh"):
            with self.subTest(entry_point=name):
                result = self.launch(name)
                self.assertEqual(result.returncode, 81, result.stdout + result.stderr)
                up = [c for c in self.commands() if c["command"] == "docker" and "up" in c["args"]][-1]
                self.assertEqual(up["bind"], "0.0.0.0")

    def test_clean_first_online_start_builds_missing_ros_image(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", BUILD_IMAGE="auto",
            STARTUP_TEST_IMAGE_EXIT="1",
        )
        self.assertEqual(result.returncode, 81, result.stdout + result.stderr)
        up = [c for c in self.commands() if c["command"] == "docker" and "up" in c["args"]][-1]
        self.assertIn("--build", up["args"])
        self.assertNotIn("--no-build", up["args"])

    def test_existing_ros_image_does_not_rebuild(self) -> None:
        result = self.launch("launch_real_robot_detached.sh", BUILD_IMAGE="auto")
        self.assertEqual(result.returncode, 81, result.stdout + result.stderr)
        up = [c for c in self.commands() if c["command"] == "docker" and "up" in c["args"]][-1]
        self.assertIn("--no-build", up["args"])

    def test_explicit_no_build_remains_available(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", BUILD_IMAGE="false",
            STARTUP_TEST_IMAGE_EXIT="1",
        )
        self.assertEqual(result.returncode, 81, result.stdout + result.stderr)
        up = [c for c in self.commands() if c["command"] == "docker" and "up" in c["args"]][-1]
        self.assertIn("--no-build", up["args"])

    def test_mac_launch_defaults_to_non_blocking_rtde_reads(self) -> None:
        result = self.launch("launch_real_robot_detached.sh")
        self.assertEqual(result.returncode, 81)
        self.assertIn("RTDE non-blocking read: true", result.stdout)

    def test_driver_is_not_launched_when_container_has_no_network(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_NETWORK="missing",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("no attached network", result.stdout + result.stderr)
        self.assertNotIn("FIXTURE: driver boundary", result.stdout)

    def test_driver_is_not_launched_when_container_cannot_route_to_robot(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_ROUTE="missing",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("route to UR", result.stdout + result.stderr)

    def test_driver_is_not_launched_when_moonraker_is_unreachable(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_MOONRAKER="offline",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("Moonraker", result.stdout + result.stderr)

    def test_ur_driver_only_reaches_driver_when_moonraker_is_unreachable(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_MOONRAKER="offline", UR_DRIVER_ONLY="true",
        )
        self.assertEqual(result.returncode, 82, result.stdout + result.stderr)
        self.assertIn("FIXTURE: driver boundary reached", result.stdout)
        self.assertFalse(any(
            c["command"] == "docker" and "curl" in c["args"]
            for c in self.commands()
        ))

    def test_ur_driver_only_does_not_start_klipper_bridge(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_DRIVER_EXIT="0", STARTUP_TEST_MOONRAKER="offline",
            UR_DRIVER_ONLY="true",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(any(
            c["command"] == "startup_bridge" for c in self.commands()
        ))

    def test_default_startup_starts_klipper_bridge_after_driver(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_DRIVER_EXIT="0",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(any(
            c["command"] == "startup_bridge" for c in self.commands()
        ))

    def test_heater_shutdown_reason_is_visible_and_blocks_driver(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_MOONRAKER="heater_shutdown",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("shutdown", result.stderr)
        self.assertIn("Heater extruder not heating at expected rate", result.stderr)
        self.assertNotIn("FIXTURE: driver boundary", result.stdout)

    def test_untrusted_moonraker_message_is_bounded_and_sanitized(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_MOONRAKER="untrusted_shutdown",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertIn("Heater extruder failed", result.stderr)
        self.assertNotIn("\x1b", result.stderr)
        self.assertLess(len(result.stderr), 400)

    def test_healthy_network_allows_launch_to_reach_driver_boundary(self) -> None:
        result = self.launch("launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0")
        self.assertEqual(result.returncode, 82, result.stdout + result.stderr)
        self.assertIn("Container network preflight passed", result.stdout)

    @unittest.skipUnless(sys.platform == "darwin", "Automatic Colima recovery is macOS-specific")
    def test_stale_vm_network_is_repaired_once_then_real_launcher_starts_driver(self) -> None:
        class MoonrakerFixture(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"result": {"state": "ready"}}')

            def log_message(self, *_args: object) -> None:
                pass

        with ThreadingHTTPServer(("127.0.0.1", 0), MoonrakerFixture) as server:
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            try:
                result = self.launch(
                    "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
                    STARTUP_TEST_DRIVER_EXIT="0", STARTUP_TEST_MOONRAKER="stale_vm",
                    MOONRAKER_URL=f"http://127.0.0.1:{server.server_port}",
                )
            finally:
                server.shutdown()
                worker.join(timeout=2)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("FIXTURE: driver boundary reached", result.stdout)
        commands = self.commands()
        starts = [c for c in commands if c["command"] == "colima" and c["args"] == ["start"]]
        self.assertEqual(len(starts), 1)
        curls = [i for i, c in enumerate(commands) if c["command"] == "docker" and "curl" in c["args"]]
        self.assertEqual(len(curls), 2, "Recovery must rerun the original container checks")
        driver = next(i for i, c in enumerate(commands)
                      if c["command"] == "docker" and "ros2 launch ur_robot_driver" in " ".join(c["args"]))
        self.assertLess(curls[-1], driver)
        self.assertFalse(any(c["command"] == "startup_cleanup" for c in commands))

    def test_failed_launch_cleans_up_its_driver_instead_of_retrying_in_background(self) -> None:
        result = self.launch("launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0")
        self.assertEqual(result.returncode, 82)
        self.assertTrue(any(c["command"] == "startup_cleanup" for c in self.commands()))

    def test_stop_during_stale_driver_cleanup_finishes_cleanup_before_next_start(self) -> None:
        result = self.cancel_during_stage("cleanup_ur_driver.sh")
        self.assertFalse(self.control_state.exists(), "Stop left a partial remote control stack")
        self.assertEqual(result.returncode, 143, result.stdout + result.stderr)
        self.assertTrue(any(c["command"] == "startup_cleanup" for c in self.commands()))
        restarted = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_DRIVER_EXIT="0",
        )
        self.assertEqual(restarted.returncode, 0, restarted.stdout + restarted.stderr)
        self.assertEqual(
            sum(c["command"] == "startup_cleanup" for c in self.commands()), 1,
            "A successful restart must not tear its new driver down",
        )

    def test_failed_stale_driver_cleanup_stops_partial_control_stack(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_FAIL_STAGE="cleanup_ur_driver.sh",
        )
        self.assertEqual(result.returncode, 86, result.stdout + result.stderr)
        self.assertTrue(any(c["command"] == "startup_cleanup" for c in self.commands()))

    def test_canceled_preflight_does_not_stop_existing_control_stack(self) -> None:
        result = self.cancel_during_stage("ensure_workstation_ros_build.sh")
        self.assertEqual(result.returncode, 143, result.stdout + result.stderr)
        self.assertFalse(any(c["command"] == "startup_cleanup" for c in self.commands()))

    def test_failed_preflight_does_not_stop_existing_control_stack(self) -> None:
        result = self.launch(
            "launch_real_robot_detached.sh", STARTUP_TEST_UP_EXIT="0",
            STARTUP_TEST_ROUTE="missing",
        )
        self.assertEqual(result.returncode, 69, result.stdout + result.stderr)
        self.assertFalse(any(c["command"] == "startup_cleanup" for c in self.commands()))


if __name__ == "__main__":
    unittest.main()
