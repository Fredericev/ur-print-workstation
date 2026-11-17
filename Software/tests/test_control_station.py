from __future__ import annotations

import importlib.util
import hashlib
import json
import re
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "control_station.py"
SPEC = importlib.util.spec_from_file_location("control_station", MODULE_PATH)
assert SPEC and SPEC.loader
station = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = station
SPEC.loader.exec_module(station)


READY_BRIDGE_STATUS = {
    "available": True,
    "ready": True,
    "moonraker_reachable": True,
    "printer_ready": True,
    "stepper_found": True,
    "action_active": False,
    "reason": "ROS Klipper bridge is ready.",
}


class ValidationTests(unittest.TestCase):
    def test_legacy_and_missing_language_settings_normalize_to_english(self) -> None:
        for language in ("zh", "en", "fr", ""):
            with self.subTest(language=language):
                normalized = station.validate_config({**station.DEFAULT_CONFIG, "language": language})
                self.assertEqual(normalized["language"], "en")
        config = {key: value for key, value in station.DEFAULT_CONFIG.items() if key != "language"}
        self.assertEqual(station.validate_config(config)["language"], "en")

    def test_health_response_survives_closed_native_log_pipe(self) -> None:
        class ClosedLogPipe:
            def write(self, _message: str) -> int:
                raise BrokenPipeError("native app log pipe is closed")

            def flush(self) -> None:
                return None

        backend = mock.Mock()
        backend.demo = False
        server = station.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            station.make_handler(backend),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            health_url = f"http://127.0.0.1:{server.server_port}/api/health"
            with mock.patch.object(station.sys, "stderr", ClosedLogPipe()):
                with urllib.request.urlopen(health_url, timeout=1.0) as response:
                    payload = json.loads(response.read())
            self.assertEqual(response.status, 200)
            self.assertTrue(payload["ok"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1.0)

    def test_workcell_height_runbook_is_available_without_legacy_calibration_form(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("const workcellSteps = [", javascript)
        self.assertIn("function renderWorkcellModule()", javascript)
        self.assertNotIn("function renderGcodeCalibration()", javascript)
        self.assertNotIn('id="calibrationForm"', javascript)
        self.assertNotIn("workcellGeometry", javascript)
        self.assertNotIn('${escapeHtml(t("workEnvelope"))}', javascript)
        self.assertIn('workcell: renderWorkcellModule', javascript)
        self.assertNotIn("Workcell & Tool Setup", javascript)
        self.assertNotIn("UR coordinate calibration", javascript)
        self.assertNotIn("work envelope", javascript.lower())
        self.assertNotIn('${escapeHtml(t("notCalibrated"))}', javascript)
        self.assertIn('${escapeHtml(t("automaticPlacement"))}', javascript)

    def test_gcode_flow_visits_print_placement_before_motion(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )

        self.assertIn('const migratedRunbook = storedRunbook;', javascript)
        self.assertIn(
            'if (state.activeRunbook === "gcode") {\n'
            '      selectRunbook("workcell");',
            javascript,
        )
        self.assertIn(
            'if (state.activeRunbook === "workcell") {\n'
            '      state.moduleSteps.motion = 0;\n'
            '      selectRunbook("motion");',
            javascript,
        )

    def test_print_z_controls_fail_closed_with_old_backend(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )

        self.assertIn('printZBackendRestart: "This workstation is still running', javascript)
        self.assertIn('const printZApiReady = typeof height.print_z_placement_mode === "string"', javascript)
        self.assertIn('busy || !printZApiReady ? "disabled" : ""', javascript)
        self.assertNotIn('busy || !printZApiReady || height.draft_surface_z_m !== null', javascript)
        self.assertIn('if (!printZApiReady) throw new Error(t("printZBackendRestart"));', javascript)
        self.assertIn('typeof state.workcell?.print_zero_z_m !== "number"', javascript)

    def test_motion_planning_requests_timeout_and_clear_loading_state(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )

        self.assertIn("async function fetchWithTimeout(", javascript)
        self.assertIn(
            'fetchWithTimeout(\n      simulationOnly ? "/api/gcode/review/simulation"',
            javascript,
        )
        self.assertIn(
            "state.gcode.pathReviewLoading = false;\n    render();",
            javascript,
        )
        self.assertIn('retryPathReview: "Retry 3D path review"', javascript)
        self.assertIn('${escapeHtml(t("retryPathReview"))}', javascript)
        self.assertIn("const duplicate = [...elements.toastRegion.children].find", javascript)

    def test_3d_viewer_does_not_render_removed_work_envelope(self) -> None:
        app_dir = MODULE_PATH.parent / "app"
        javascript = (app_dir / "app.js").read_text(encoding="utf-8")
        viewer = (app_dir / "path-review-3d.js").read_text(encoding="utf-8")
        index = (app_dir / "index.html").read_text(encoding="utf-8")

        self.assertNotIn("function createEnvelope(", viewer)
        self.assertNotIn("review.work_envelope_m", viewer)
        self.assertNotIn("this.envelope", viewer)
        self.assertIn(
            '<script type="module" src="/path-review-3d.js"></script>',
            index,
        )
        self.assertNotIn("path-review-3d-067.bundle.js", index)
        self.assertFalse((app_dir / "path-review-3d-067.bundle.js").exists())
        self.assertIn(
            "const tableSurfaceZ = Number(\n"
            "    review.workcell_model?.tabletop?.surface_z_m",
            javascript,
        )
        self.assertIn("const tableHeightMm = Number.isFinite(tableSurfaceZ)", javascript)

    def test_main_runbook_exposes_only_the_required_end_to_end_flow(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )

        self.assertIn('executionRunbook: "Motion Test / Print Execution"', javascript)
        self.assertIn('const executionSteps = [', javascript)
        self.assertNotIn('safetyValidation: {', javascript)
        self.assertNotIn('synchronization: {', javascript)
        self.assertNotIn('monitoring: {', javascript)
        self.assertNotIn('Safety Validation', javascript)
        self.assertNotIn('Synchronization & Dry Run', javascript)
        self.assertNotIn('Monitoring & Recovery', javascript)

    def test_execution_result_separates_motion_test_scope_and_safety_copy(self) -> None:
        app_dir = MODULE_PATH.parent / "app"
        javascript = (app_dir / "app.js").read_text(encoding="utf-8")
        stylesheet = (app_dir / "styles.css").read_text(encoding="utf-8")

        self.assertIn('motionTestRunTime: "This test run"', javascript)
        self.assertIn('motionTestCoverage: "Full-path coverage"', javascript)
        self.assertIn('motionTestFinishedTitle: "Tool0 motion-test sample complete"', javascript)
        self.assertIn('class="execution-safety-copy"', javascript)
        self.assertIn('t("stopSemantics")', javascript)
        self.assertIn('.execution-safety-copy {', stylesheet)
        self.assertIn('.execution-runtime-heading > div {', stylesheet)
        self.assertIn('overflow-wrap: anywhere;', stylesheet)

    def test_live_execution_uses_a_compact_package_summary(self) -> None:
        app_dir = MODULE_PATH.parent / "app"
        javascript = (app_dir / "app.js").read_text(encoding="utf-8")
        stylesheet = (app_dir / "styles.css").read_text(encoding="utf-8")

        self.assertIn('function executionPackageMarkup({ compact = false } = {})', javascript)
        self.assertIn('class="execution-package-card compact"', javascript)
        self.assertIn('executionPackageMarkup({ compact: true })', javascript)
        self.assertIn('motionTestPackageShort: "Tool0 motion test"', javascript)
        self.assertIn('.execution-package-card.compact {', stylesheet)
        self.assertIn('.execution-package-compact-summary {', stylesheet)

    def test_fixed_workcell_is_read_only_and_jobs_are_centered(self) -> None:
        javascript = (MODULE_PATH.parent / "app" / "app.js").read_text(
            encoding="utf-8"
        )
        viewer = (MODULE_PATH.parent / "app" / "path-review-3d.js").read_text(
            encoding="utf-8"
        )
        planner = (
            MODULE_PATH.parents[1]
            / "workspaces"
            / "ur_ros2_ws"
            / "src"
            / "ur3e_print_planner"
            / "src"
            / "gcode_path_review.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn('automaticPlacement: "Automatic print placement"', javascript)
        self.assertNotIn('name: "origin_x"', javascript)
        self.assertNotIn('name: "bed_plane_z"', javascript)
        self.assertNotIn('name: "bed_min_x"', javascript)
        self.assertNotIn('id="captureCurrentToolPose"', javascript)
        self.assertIn("new THREE.PlaneGeometry(width, depth)", viewer)
        self.assertNotIn("Number(tabletop?.thickness_m) || 0.03", viewer)
        self.assertIn('bed.id = "confirmed_table_surface_guard"', planner)
        self.assertIn(
            "constexpr double kBelowTableGuardDepthM = 2.0;", planner
        )
        self.assertIn(
            "bed_plane_z - kBelowTableGuardDepthM / 2.0", planner
        )
        self.assertIn(
            "constexpr double kApproachClearanceM = 0.12;", planner
        )
        self.assertIn('"continuous-cartesian-approach-v4"', planner)
        self.assertNotIn("kApproachPlanningAttempts", planner)
        self.assertIn("approach_staging_pose", planner)
        self.assertIn("display_trajectory.trajectory.push_back", planner)
        self.assertIn(
            'if (status == "passed" && !trajectory.joint_trajectory.points.empty())',
            planner,
        )
        self.assertIn("std::filesystem::remove(approach_trajectory_file);", planner)
        self.assertIn("std::filesystem::remove(print_trajectory_file);", planner)
        self.assertNotIn("constexpr double table_thickness", planner)

    def test_windows_data_dir_uses_local_app_data(self) -> None:
        path = station.resolve_data_dir(
            system_name="Windows",
            environment={"LOCALAPPDATA": r"C:\Users\Operator\AppData\Local"},
        )
        self.assertEqual(path.name, "UR Print Workstation")
        self.assertIn("AppData", str(path))

    def test_macos_data_dir_uses_application_support(self) -> None:
        path = station.resolve_data_dir(system_name="Darwin", environment={})

        self.assertEqual(path.name, "UR Print Workstation")
        self.assertEqual(path.parent.name, "Application Support")
        self.assertNotEqual(path, MODULE_PATH.parent)

    def test_data_dir_override_takes_precedence(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            path = station.resolve_data_dir(
                system_name="Windows",
                environment={
                    "LOCALAPPDATA": r"C:\Users\Operator\AppData\Local",
                    "UR_PRINT_DATA_DIR": tempdir,
                },
            )
            self.assertEqual(path, Path(tempdir).resolve())

    def test_write_config_creates_writable_data_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            config_path = Path(tempdir) / "nested" / "config.json"
            with mock.patch.object(station, "CONFIG_PATH", config_path):
                station.write_config(station.DEFAULT_CONFIG)
            self.assertTrue(config_path.is_file())
            self.assertEqual(
                json.loads(config_path.read_text(encoding="utf-8"))["language"],
                "en",
            )

    def test_default_config_is_valid(self) -> None:
        validated = station.validate_config(station.DEFAULT_CONFIG)
        self.assertEqual(validated["ur_robot_ip"], "192.168.0.104")
        self.assertEqual(validated["klipper_url"], "http://192.168.0.168")
        self.assertEqual(validated["language"], "en")

    def test_demo_backend_uses_ephemeral_runtime_state(self) -> None:
        backend = station.StationBackend(demo=True)

        self.assertNotEqual(backend.data_dir, station.DATA_DIR)
        self.assertEqual(backend.jobs.job_log_dir.parent, backend.data_dir / "runtime")
        self.assertFalse(str(backend.jobs.job_log_dir).startswith(str(MODULE_PATH.parent)))

    def test_legacy_operator_state_is_copied_without_overwriting_new_state(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            legacy = root / "legacy"
            destination = root / "application-support"
            legacy.mkdir()
            destination.mkdir()
            (legacy / "config.json").write_text("legacy-config", encoding="utf-8")
            (legacy / "gcode_calibration.json").write_text(
                "legacy-calibration", encoding="utf-8"
            )
            (destination / "config.json").write_text("new-config", encoding="utf-8")

            migrated = station.migrate_legacy_state(legacy, destination)

            self.assertEqual(migrated, ["gcode_calibration.json"])
            self.assertEqual(
                (destination / "config.json").read_text(encoding="utf-8"),
                "new-config",
            )
            self.assertEqual(
                (destination / "gcode_calibration.json").read_text(encoding="utf-8"),
                "legacy-calibration",
            )

    def test_compose_limits_host_exposure_and_core_dumps(self) -> None:
        compose = (station.PROJECT_ROOT / "docker-compose.real.yml").read_text(
            encoding="utf-8"
        )

        self.assertIn('"127.0.0.1:${ROS_DESKTOP_PORT:-6082}:80"', compose)
        self.assertIn(
            '"${UR_REVERSE_BIND_IP:-127.0.0.1}:50001:50001"', compose
        )
        self.assertIn("core:", compose)
        self.assertIn("hard: 0", compose)

    def test_ros_runtime_uses_udp_when_fastdds_shared_memory_is_unhealthy(self) -> None:
        compose = (station.PROJECT_ROOT / "docker-compose.real.yml").read_text(
            encoding="utf-8"
        )
        profile = (
            station.PROJECT_ROOT
            / "workspaces"
            / "ur_ros2_ws"
            / "config"
            / "fastdds_udp_only.xml"
        )

        self.assertIn(
            "FASTDDS_DEFAULT_PROFILES_FILE: "
            "/home/ubuntu/ur_ros2_ws/config/fastdds_udp_only.xml",
            compose,
        )
        self.assertIn(
            "FASTRTPS_DEFAULT_PROFILES_FILE: "
            "/home/ubuntu/ur_ros2_ws/config/fastdds_udp_only.xml",
            compose,
        )
        root = ET.parse(profile).getroot()
        namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        self.assertEqual(root.findtext(".//f:useBuiltinTransports", namespaces=namespace), "false")
        self.assertEqual(root.findtext(".//f:type", namespaces=namespace), "UDPv4")

    def test_pid_stop_scripts_verify_process_identity(self) -> None:
        helper = (
            station.PROJECT_ROOT
            / "workspaces"
            / "ur_ros2_ws"
            / "scripts"
            / "verified_pid_signal.sh"
        )
        self.assertTrue(helper.is_file())
        source = helper.read_text(encoding="utf-8")
        self.assertIn('/proc/$pid/cmdline', source)
        self.assertIn("expected_pattern", source)

    def test_native_app_keeps_runtime_out_of_the_source_tree_and_clears_xattrs(self) -> None:
        swift = (MODULE_PATH.parent / "native_app" / "URPrintWorkstation.swift").read_text(
            encoding="utf-8"
        )
        builder = (MODULE_PATH.parent / "build-macos-app.command").read_text(
            encoding="utf-8"
        )

        self.assertIn("applicationSupportDirectory", swift)
        self.assertIn('environment["UR_PRINT_DATA_DIR"]', swift)
        self.assertIn('forInfoDictionaryKey: "URPrintSoftwareDirectory"', swift)
        self.assertIn('INSTALL_APP="$USER_APPLICATIONS/UR Print Workstation.app"', builder)
        self.assertIn('/usr/bin/xattr -cr "$APP"', builder)
        self.assertLess(
            builder.index('/usr/bin/xattr -cr "$APP"'),
            builder.index("/usr/bin/codesign"),
        )

    def test_rejects_invalid_motion_limits(self) -> None:
        with self.assertRaises(station.ApiError):
            station.validate_config({**station.DEFAULT_CONFIG, "distance_m": 0.5})
        with self.assertRaises(station.ApiError):
            station.validate_config({**station.DEFAULT_CONFIG, "max_joint_speed_rad_s": 0.5})

    def test_rejects_credentials_and_unsafe_stepper_names(self) -> None:
        with self.assertRaises(station.ApiError):
            station.validate_config({**station.DEFAULT_CONFIG, "klipper_url": "http://user:pass@192.168.0.110"})
        with self.assertRaises(station.ApiError):
            station.validate_config({**station.DEFAULT_CONFIG, "manual_stepper": "x; reboot"})

    def test_driver_log_uses_latest_connection_event(self) -> None:
        connected = station.parse_driver_log(
            "System successfully started!\nRobot connected to reverse interface\n"
        )
        self.assertTrue(connected["external_control"])
        dropped = station.parse_driver_log(
            "Robot connected to reverse interface\nConnection to reverse interface dropped\n"
        )
        self.assertFalse(dropped["external_control"])

    def test_live_external_control_probe_parses_current_false_state(self) -> None:
        payload = {
            "ready": False,
            "program_state_received": True,
            "program_running": False,
            "joint_state_fresh": True,
            "joint_count": 6,
            "reason": "The External Control program is not running on the teach pendant.",
        }
        with mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": False, "return_code": 2, "output": json.dumps(payload)},
        ):
            result = station.live_external_control_status()
        self.assertTrue(result["available"])
        self.assertFalse(result["ready"])
        self.assertFalse(result["program_running"])

    def test_live_klipper_bridge_probe_parses_ready_status(self) -> None:
        with mock.patch.object(
            station,
            "run_capture",
            return_value={
                "ok": True,
                "return_code": 0,
                "output": json.dumps(READY_BRIDGE_STATUS),
            },
        ):
            result = station.live_klipper_bridge_status("stepper_x")
        self.assertTrue(result["available"])
        self.assertTrue(result["ready"])
        self.assertTrue(result["moonraker_reachable"])
        self.assertTrue(result["stepper_found"])

    def test_live_tool0_pose_parses_tf2_echo_even_after_timeout(self) -> None:
        output = """
At time 123.0
- Translation: [-0.426, -0.072, 0.250]
- Rotation: in Quaternion (xyzw) [0.738, 0.674, 0.008, 0.011]
- Rotation: in RPY (radian) [3.114, 0.003, 1.480]
- Rotation: in RPY (degree) [178.442, 0.160, 84.787]
"""
        with mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": False, "return_code": 124, "output": output},
        ):
            pose = station.live_tool0_pose()
        self.assertEqual(pose["frame_id"], "base_link")
        self.assertEqual(pose["position_m"]["x"], -0.426)
        self.assertEqual(pose["orientation_rpy_deg"]["yaw"], 84.787)
        self.assertTrue(pose["execution_locked"])

    def test_driver_log_identifies_rtde_ownership_conflict(self) -> None:
        result = station.parse_driver_log(
            "Variable 'speed_slider_mask' is currently controlled by another RTDE client.\n"
            "Failed to initialize RTDE client after 3 attempts\n"
        )
        self.assertTrue(result["rtde_conflict"])
        self.assertTrue(result["unstable"])

    def test_intentional_driver_stop_clears_old_connection_errors(self) -> None:
        result = station.parse_driver_log(
            "System successfully started!\n"
            "Robot connected to reverse interface\n"
            "Unable to read from hardware\n"
            "[workstation] UR driver intentionally stopped.\n"
        )
        self.assertFalse(result["system_started"])
        self.assertFalse(result["external_control"])
        self.assertFalse(result["unstable"])

    def test_successful_start_clears_recovered_rtde_initialization_retry(self) -> None:
        result = station.parse_driver_log(
            "Variable 'speed_slider_mask' is currently controlled by another RTDE client.\n"
            "Failed to initialize RTDE client, retrying in 5 seconds\n"
            "System successfully started!\n"
        )
        self.assertTrue(result["system_started"])
        self.assertFalse(result["rtde_conflict"])
        self.assertFalse(result["unstable"])
        self.assertFalse(result["external_control"])

    def test_rtde_failure_after_successful_start_remains_blocking(self) -> None:
        result = station.parse_driver_log(
            "System successfully started!\n"
            "Variable 'speed_slider_mask' is currently controlled by another RTDE client.\n"
            "Failed to initialize RTDE client after 3 attempts\n"
        )
        self.assertTrue(result["rtde_conflict"])
        self.assertTrue(result["unstable"])

    def test_offline_container_ignores_stale_ready_logs(self) -> None:
        backend = station.StationBackend(demo=False)
        stale_log = (
            "System successfully started!\n"
            "Robot connected to reverse interface\n"
            "You can start planning now!\n"
        )
        with mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": False, "return_code": 127, "output": "Docker unavailable"},
        ), mock.patch.object(station, "tail_text", return_value=stale_log):
            status = backend.ros_status()
        self.assertFalse(status["driver_started"])
        self.assertFalse(status["external_control"])
        self.assertFalse(status["moveit_ready"])
        self.assertFalse(status["unstable"])

    def test_host_port_check_detects_colima_localhost_collision(self) -> None:
        localhost_listener = {
            "ok": True,
            "return_code": 0,
            "output": "limactl 11 user 13u IPv4 TCP 127.0.0.1:50004 (LISTEN)",
        }
        with mock.patch.object(station.platform, "system", return_value="Darwin"), mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {"ok": True, "return_code": 0, "output": f"ssh 10 user 12u IPv4 TCP *:{port} (LISTEN)"}
                for port in (50001, 50002, 50003)
            ]
            + [localhost_listener],
        ):
            status = station.host_ur_ports_status()
        self.assertFalse(status["ok"])
        self.assertEqual(status["missing"], [50004])

    def test_host_port_check_accepts_robot_interface_specific_listeners(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Darwin"
        ), mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {
                    "ok": True,
                    "return_code": 0,
                    "output": (
                        f"com.docke 10 user 12u IPv4 TCP "
                        f"192.168.12.1:{port} (LISTEN)"
                    ),
                }
                for port in station.UR_REVERSE_PORTS
            ],
        ):
            status = station.host_ur_ports_status("192.168.12.1")

        self.assertTrue(status["ok"])

    def test_host_port_check_accepts_ipv4_wildcard_covering_robot_interface(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Darwin"
        ), mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {
                    "ok": True,
                    "return_code": 0,
                    "output": f"com.docke 10 user 12u IPv4 TCP *:{port} (LISTEN)",
                }
                for port in station.UR_REVERSE_PORTS
            ],
        ):
            status = station.host_ur_ports_status("192.168.12.1")

        self.assertTrue(status["ok"])

    def test_windows_host_port_check_requires_public_listeners(self) -> None:
        output = "\n".join(
            [
                "  TCP    0.0.0.0:50001       0.0.0.0:0       LISTENING       101",
                "  TCP    [::]:50002          [::]:0          LISTENING       102",
                "  TCP    0.0.0.0:50003       0.0.0.0:0       LISTENING       103",
                "  TCP    127.0.0.1:50004     0.0.0.0:0       LISTENING       104",
            ]
        )
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": True, "return_code": 0, "output": output},
        ) as run:
            status = station.host_ur_ports_status()
        self.assertFalse(status["ok"])
        self.assertEqual(status["missing"], [50004])
        self.assertEqual(run.call_args.args[0], ["netstat", "-ano", "-p", "tcp"])

    def test_windows_host_port_check_fails_closed_when_netstat_fails(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": False, "return_code": 1, "output": "Access denied"},
        ):
            status = station.host_ur_ports_status()
        self.assertFalse(status["ok"])
        self.assertEqual(status["missing"], list(station.UR_REVERSE_PORTS))

    def test_windows_ping_uses_windows_arguments(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={
                "ok": True,
                "return_code": 0,
                "output": "Reply from 192.168.0.104: time<1ms TTL=64",
            },
        ) as run:
            ok, detail = station.ping_probe("192.168.0.104")
        self.assertTrue(ok)
        self.assertEqual(detail, "Reachable in 1 ms.")
        self.assertEqual(
            run.call_args.args[0],
            ["ping", "-n", "1", "-w", "1000", "192.168.0.104"],
        )

    def test_windows_route_uses_powershell_json(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={
                "ok": True,
                "return_code": 0,
                "output": (
                    '{"interface":"Ethernet 2","address":"192.168.0.107",'
                    '"service":"Ethernet"}'
                ),
            },
        ) as run:
            route = station.detect_route("192.168.0.104")
        self.assertEqual(
            route,
            {
                "interface": "Ethernet 2",
                "address": "192.168.0.107",
                "service": "Ethernet",
            },
        )
        self.assertEqual(run.call_args.args[0][0], "powershell.exe")
        powershell = run.call_args.args[0][-1]
        self.assertIn("$items=@(Find-NetRoute", powershell)
        self.assertIn("'IPAddress'", powershell)
        self.assertIn("'DestinationPrefix'", powershell)
        self.assertNotIn(".NetRoute.InterfaceIndex", powershell)

    def test_windows_route_fails_closed_on_unusable_output(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": False, "return_code": 1, "output": "no route"},
        ):
            route = station.detect_route("192.168.0.104")
        self.assertEqual(route, {"interface": "", "address": "", "service": ""})

    def test_virtual_windows_route_is_not_accepted_as_wired_robot_link(self) -> None:
        backend = station.StationBackend(demo=False)
        with mock.patch.object(
            station,
            "detect_route",
            return_value={
                "interface": "vEthernet (WSL)",
                "address": "172.30.32.1",
                "service": "Virtual",
            },
        ), mock.patch.object(
            station,
            "ping_probe",
            return_value=(True, "Reachable in 1 ms."),
        ), mock.patch.object(
            station,
            "dashboard_query",
            return_value=(True, "Robotmode: RUNNING"),
        ), mock.patch.object(
            station,
            "tcp_probe",
            return_value=(True, "Port 30004 responded."),
        ):
            result = backend.check_ur()
        self.assertFalse(result["ok"])
        route_check = next(
            check for check in result["checks"] if check["id"] == "route"
        )
        self.assertFalse(route_check["ok"])

    def test_stale_moveit_log_does_not_report_ready_without_live_process(self) -> None:
        backend = station.StationBackend(demo=False)
        stale_log = (
            "System successfully started!\n"
            "Robot connected to reverse interface\n"
            "You can start planning now!\n"
        )
        with mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {"ok": True, "return_code": 0, "output": "29.2.1"},
                {"ok": True, "return_code": 0, "output": "ros2"},
                {"ok": True, "return_code": 0, "output": "1007\n"},
                {"ok": False, "return_code": 1, "output": ""},
            ],
        ), mock.patch.object(
            station,
            "host_ur_ports_status",
            return_value={"ok": True, "missing": [], "listeners": {}},
        ), mock.patch.object(
            station,
            "live_external_control_status",
            return_value={
                "available": True,
                "ready": True,
                "program_running": True,
                "joint_state_fresh": True,
                "reason": "External Control is live and ready.",
            },
        ), mock.patch.object(
            station,
            "live_klipper_bridge_status",
            return_value=READY_BRIDGE_STATUS,
        ), mock.patch.object(station, "tail_text", return_value=stale_log):
            status = backend.ros_status()
        self.assertTrue(status["external_control"])
        self.assertFalse(status["moveit_running"])
        self.assertFalse(status["moveit_ready"])

    def test_plan_only_moveit_stays_ready_after_external_control_stops(self) -> None:
        backend = station.StationBackend(demo=False)
        log_text = (
            "System successfully started!\n"
            "Robot connected to reverse interface\n"
            "Connection to reverse interface dropped.\n"
            "You can start planning now!\n"
        )
        with mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {"ok": True, "return_code": 0, "output": "29.2.1"},
                {"ok": True, "return_code": 0, "output": "ros2"},
                {"ok": True, "return_code": 0, "output": "1007\n"},
                {"ok": True, "return_code": 0, "output": "1738\n"},
            ],
        ), mock.patch.object(
            station,
            "host_ur_ports_status",
            return_value={"ok": True, "missing": [], "listeners": {}},
        ), mock.patch.object(
            station,
            "live_external_control_status",
            return_value={
                "available": True,
                "ready": False,
                "program_running": False,
                "joint_state_fresh": True,
                "reason": "The External Control program is not running on the teach pendant.",
            },
        ), mock.patch.object(
            station,
            "live_klipper_bridge_status",
            return_value=READY_BRIDGE_STATUS,
        ), mock.patch.object(station, "tail_text", return_value=log_text):
            status = backend.ros_status()
        self.assertFalse(status["external_control"])
        self.assertTrue(status["moveit_running"])
        self.assertTrue(status["moveit_ready"])

    def test_old_connected_log_cannot_override_live_program_false(self) -> None:
        backend = station.StationBackend(demo=False)
        stale_log = "System successfully started!\nRobot connected to reverse interface\n"
        with mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {"ok": True, "return_code": 0, "output": "29.2.1"},
                {"ok": True, "return_code": 0, "output": "ros2"},
                {"ok": True, "return_code": 0, "output": "1007\n"},
                {"ok": False, "return_code": 1, "output": ""},
            ],
        ), mock.patch.object(
            station,
            "host_ur_ports_status",
            return_value={"ok": True, "missing": [], "listeners": {}},
        ), mock.patch.object(
            station,
            "live_external_control_status",
            return_value={
                "available": True,
                "ready": False,
                "program_running": False,
                "joint_state_fresh": True,
                "reason": "The External Control program is not running on the teach pendant.",
            },
        ), mock.patch.object(
            station,
            "live_klipper_bridge_status",
            return_value=READY_BRIDGE_STATUS,
        ), mock.patch.object(station, "tail_text", return_value=stale_log):
            status = backend.ros_status()
        self.assertFalse(status["external_control"])
        self.assertIn("not running", status["summary"])

    def test_stopped_driver_is_not_revived_by_its_old_success_log(self) -> None:
        backend = station.StationBackend(demo=False)
        stale_log = "System successfully started!\nRobot connected to reverse interface\n"
        with mock.patch.object(
            station,
            "run_capture",
            side_effect=[
                {"ok": True, "return_code": 0, "output": "29.2.1"},
                {"ok": True, "return_code": 0, "output": "ros2"},
                {"ok": False, "return_code": 1, "output": ""},
                {"ok": False, "return_code": 1, "output": ""},
            ],
        ), mock.patch.object(
            station,
            "live_klipper_bridge_status",
            return_value=READY_BRIDGE_STATUS,
        ), mock.patch.object(station, "tail_text", return_value=stale_log):
            status = backend.ros_status()
        self.assertFalse(status["driver_started"])
        self.assertFalse(status["external_control"])


class DemoBackendTests(unittest.TestCase):
    def test_busy_review_does_not_revoke_the_active_operation(self) -> None:
        with mock.patch.object(self.backend.jobs, "active", return_value=object()), mock.patch.object(
            station, "begin_planning_request"
        ) as revoke:
            with self.assertRaises(station.ApiError) as rejected:
                self.backend.start_gcode_moveit_review("test.gcode", b"G1 X1 E1\n")
            self.assertEqual(rejected.exception.status, 409)
            revoke.assert_not_called()

    def test_continuous_package_rejects_pause_and_resume_but_can_stop(self) -> None:
        execution_id, job_id = "a" * 16, "b" * 12
        self.backend._print_jobs[job_id] = {"execution_id": execution_id, "mode": "execute"}
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            self.backend, "_execution_package",
            return_value=(Path(directory) / "package.json", {
                "version": 3,
                "pause_semantics": "unsupported_during_continuous_job_use_stop",
            }),
        ), mock.patch.object(
            self.backend.jobs, "get", return_value=mock.Mock(state="running"),
        ), mock.patch.object(self.backend.jobs, "cancel", return_value="cancelled"):
            self.backend._print_execution_runtime = Path(directory)
            for paused in (True, False):
                with self.assertRaises(station.ApiError) as rejected:
                    self.backend.set_print_pause(execution_id, paused=paused)
                self.assertEqual(rejected.exception.status, 409)
            self.assertFalse((Path(directory) / f"{execution_id}.pause").exists())
            self.assertEqual(self.backend.stop_print_operation(execution_id), "cancelled")

    def setUp(self) -> None:
        self.backend = station.StationBackend(demo=True)
        self.safety = {
            "emergency_stop": True,
            "workspace_clear": True,
            "speed_under_20": True,
            "wired_network": True,
        }

    def test_demo_checks_are_complete(self) -> None:
        self.assertTrue(self.backend.check_ur()["ok"])
        self.assertTrue(self.backend.check_klipper()["ok"])
        self.assertTrue(self.backend.ros_status()["moveit_ready"])

    def test_saved_print_setup_is_revalidated_automatically_for_each_job(self) -> None:
        first = b"G21\nG90\nM83\nG1 X0 Y0 Z0 E1 F600\nG1 X20 Y10 E1\n"
        second = b"G21\nG90\nM83\nG1 X100 Y50 Z0 E1 F600\nG1 X140 Y70 E1\n"
        analysis = station.parse_gcode("first.gcode", first)
        profile = station.default_calibration()
        profile["operation_mode"] = "tool0_motion_test"
        profile["coordinate_transform"]["rotation_z_deg"] = -90.0
        profile["confirmations"].update(
            {
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "tool0_reference_verified": True,
            }
        )
        self.backend.save_gcode_calibration(
            {
                "calibration": profile,
                "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            }
        )

        review = self.backend.build_gcode_path_review("second.gcode", second)

        self.assertEqual(
            review["placement_model"]["print_center_m"],
            {"x": -0.2925, "y": 0.0},
        )
        self.assertEqual(review["source_sha256"], hashlib.sha256(second).hexdigest())

    def test_out_of_table_review_reports_the_actual_failed_gate(self) -> None:
        setup = b"G21\nG90\nM83\nG1 X0 Y0 Z0.2 E1 F600\nG1 X20 Y10 E1\n"
        analysis = station.parse_gcode("setup.gcode", setup)
        profile = station.default_calibration()
        profile["operation_mode"] = "tool0_motion_test"
        profile["confirmations"].update(
            {
                "orientation_verified": True,
                "tool_orientation_verified": True,
                "extrusion_ratio_measured": True,
                "tool0_reference_verified": True,
            }
        )
        self.backend.save_gcode_calibration(
            {
                "calibration": profile,
                "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            }
        )
        too_large = (
            b"G21\nG90\nM83\n"
            b"G1 X0 Y0 Z0.2 E1 F600\n"
            b"G1 X600 Y0 E1\n"
        )

        with self.assertRaises(station.ApiError) as raised:
            self.backend.build_gcode_path_review("too-large.gcode", too_large)

        self.assertEqual(raised.exception.status, 412)
        self.assertIn("outside the fixed table", str(raised.exception))
        self.assertIn("X", str(raised.exception))

    def test_operating_mode_defaults_offline_and_requires_explicit_online(self) -> None:
        self.assertEqual(self.backend.get_operating_mode()["mode"], "offline")
        with self.assertRaises(station.ApiError) as raised:
            self.backend.require_online_mode("capturing tool0")
        self.assertEqual(raised.exception.status, 412)

        changed = self.backend.set_operating_mode({"mode": "online"})
        self.assertEqual(changed["previous_mode"], "offline")
        self.assertTrue(changed["online"])
        self.backend.require_online_mode("capturing tool0")

    def test_operating_mode_rejects_invalid_value_and_busy_change(self) -> None:
        with self.assertRaises(station.ApiError) as invalid:
            self.backend.set_operating_mode({"mode": "automatic"})
        self.assertEqual(invalid.exception.status, 422)

        with mock.patch.object(self.backend.jobs, "active", return_value=object()):
            with self.assertRaises(station.ApiError) as busy:
                self.backend.set_operating_mode({"mode": "online"})
        self.assertEqual(busy.exception.status, 409)

    def test_incomplete_calibration_draft_is_simulation_only(self) -> None:
        content = (
            b"G21\nG90\nM83\n"
            b"G1 X10 Y5 Z0.2 E1 F600\n"
            b"G1 X20 Y5 E1\n"
        )
        analysis = station.parse_gcode("draft.gcode", content)
        profile = station.default_calibration()
        profile["coordinate_transform"]["origin_m"] = {
            "x": -0.35,
            "y": 0.05,
            "z": 0.2,
        }
        validation = self.backend.validate_gcode_calibration(
            {
                "calibration": profile,
                "bounds": analysis["bounds"],
                "source_sha256": analysis["file"]["sha256"],
            }
        )

        self.assertFalse(validation["ready_to_save"])
        self.assertEqual(validation["status"], "incomplete")

        with self.assertRaises(station.ApiError) as strict_review:
            self.backend.build_gcode_path_review("draft.gcode", content)
        self.assertEqual(strict_review.exception.status, 412)

        review = self.backend.build_gcode_simulation_review(
            "draft.gcode",
            content,
        )
        self.assertEqual(review["status"], "offline_simulation")
        self.assertTrue(review["simulation_only"])
        self.assertFalse(review["physical_calibration_complete"])
        self.assertTrue(review["execution_locked"])
        self.assertEqual(
            review["placement_model"]["mode"],
            "center_on_fixed_table",
        )
        self.assertFalse(review["placement_model"]["assumed"])
        self.assertEqual(
            review["placement_model"]["print_center_m"],
            {"x": -0.2925, "y": 0.0},
        )
        current_table_z = station.load_workcell_geometry()["tabletop"]["surface_z_m"]
        self.assertAlmostEqual(
            review["placement_model"]["table_surface_z_m"],
            current_table_z,
        )
        self.assertAlmostEqual(
            review["physical_calibration"]["coordinate_transform"]["origin_m"]["z"],
            current_table_z,
        )
        self.assertAlmostEqual(
            review["calibration"]["coordinate_transform"]["origin_m"]["z"],
            current_table_z,
        )
        self.assertAlmostEqual(
            review["deposition_bounds_m"]["minimum"]["z"],
            current_table_z,
        )
        self.assertAlmostEqual(
            review["workcell_model"]["tabletop"]["surface_z_m"],
            current_table_z,
        )
        self.assertFalse(review["workcell_model"]["tabletop"]["visual_only"])
        self.assertTrue(
            review["workcell_model"]["tabletop"]["collision_authoritative"]
        )
        self.assertIsNone(review["moveit_validation"])
        self.assertEqual(review["offline_simulation"]["status"], "passed")
        self.assertTrue(review["offline_simulation"]["execution_locked"])
        gate_status = {gate["code"]: gate["status"] for gate in review["gates"]}
        self.assertEqual(gate_status["tcp_geometry"], "required")
        self.assertEqual(gate_status["print_bed"], "passed")

        with self.assertRaises(station.ApiError) as moveit_review:
            self.backend.start_gcode_moveit_review("draft.gcode", content)
        self.assertEqual(moveit_review.exception.status, 412)

    def test_motion_requires_safety_and_scoped_button_action(self) -> None:
        with self.assertRaises(station.ApiError):
            self.backend.sync_test(self.safety)
        with self.assertRaises(station.ApiError):
            self.backend.sync_test(
                {
                    **self.safety,
                    "external_control_playing": True,
                    "operator_action": "YES",
                }
            )

    def test_demo_dry_run_finishes_without_hardware(self) -> None:
        job = self.backend.dry_run(self.safety)
        deadline = time.time() + 4
        while job.state in {"queued", "running"} and time.time() < deadline:
            time.sleep(0.05)
        self.assertEqual(job.state, "succeeded")
        self.assertIn("No hardware moved", job.public()["log"])

    def test_demo_sync_can_return_to_start_without_hardware(self) -> None:
        job = self.backend.sync_test(
            {
                **self.safety,
                "external_control_playing": True,
                "operator_action": "sync-test",
                "reset_stepper_position": True,
                "return_to_start": True,
            }
        )
        deadline = time.time() + 5
        while job.state in {"queued", "running"} and time.time() < deadline:
            time.sleep(0.05)
        self.assertEqual(job.state, "succeeded")
        self.assertIn("Return-to-start completed for UR and Klipper", job.public()["log"])


class RealWorkflowGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = station.StationBackend(demo=False)
        self.execution_runtime = tempfile.TemporaryDirectory(prefix="workflow-guard-runtime-")
        self.addCleanup(self.execution_runtime.cleanup)
        self.backend._print_execution_runtime = Path(self.execution_runtime.name)
        self.safety = {
            "emergency_stop": True,
            "workspace_clear": True,
            "speed_under_20": True,
            "wired_network": True,
        }

    def test_start_ros_waits_for_proven_driver_readiness(self) -> None:
        sentinel = object()
        with mock.patch.object(self.backend, "check_ur", return_value={"ok": True}), mock.patch.object(
            self.backend, "check_klipper", return_value={"ok": True}
        ), mock.patch.object(
            self.backend.jobs, "start", return_value=sentinel
        ) as start:
            result = self.backend.start_ros(self.safety)
        self.assertIs(result, sentinel)
        self.assertEqual(start.call_args.kwargs["env"]["WAIT_FOR_READY"], "true")

    def test_external_control_is_blocked_when_rtde_is_occupied(self) -> None:
        with mock.patch.object(
            self.backend,
            "ros_status",
            return_value={"rtde_conflict": True, "driver_started": False, "unstable": True},
        ), self.assertRaises(station.ApiError) as raised:
            self.backend.wait_external_control(self.safety)
        self.assertIn("RTDE is occupied", raised.exception.message)

    def test_restart_ros_requires_explicit_button_action(self) -> None:
        with self.assertRaises(station.ApiError) as raised:
            self.backend.restart_ros(self.safety)
        self.assertIn("restart-ros", raised.exception.message)

    def test_restart_ros_reuses_readiness_gated_launch(self) -> None:
        sentinel = object()
        with mock.patch.object(self.backend, "check_ur", return_value={"ok": True}), mock.patch.object(
            self.backend, "check_klipper", return_value={"ok": True}
        ), mock.patch.object(
            self.backend.jobs, "start", return_value=sentinel
        ) as start:
            result = self.backend.restart_ros({**self.safety, "operator_action": "restart-ros"})
        self.assertIs(result, sentinel)
        self.assertEqual(start.call_args.args[0], "restart-ros")
        self.assertEqual(start.call_args.kwargs["env"]["WAIT_FOR_READY"], "true")

    def test_return_to_start_requires_stepper_zero_reference(self) -> None:
        with self.assertRaises(station.ApiError) as raised:
            self.backend.sync_test(
                {
                    **self.safety,
                    "external_control_playing": True,
                    "operator_action": "sync-test",
                    "return_to_start": True,
                    "reset_stepper_position": False,
                }
            )
        self.assertIn("requires resetting", raised.exception.message)

    def test_return_to_start_adds_coordinator_flags(self) -> None:
        sentinel = object()
        ready_ros = {
            "external_control": True,
            "moveit_ready": True,
            "klipper_bridge_ready": True,
            "unstable": False,
        }
        with mock.patch.object(self.backend, "check_klipper", return_value={"ok": True, "extruder": {"can_extrude": True}}), mock.patch.object(
            self.backend, "ros_status", return_value=ready_ros
        ), mock.patch.object(
            self.backend,
            "validate_cartesian_test_over_print_bed",
            return_value={"status": "passed"},
        ), mock.patch.object(self.backend.jobs, "start", return_value=sentinel) as start:
            result = self.backend.sync_test(
                {
                    **self.safety,
                    "external_control_playing": True,
                    "operator_action": "sync-test",
                    "return_to_start": True,
                    "reset_stepper_position": True,
                }
            )
        self.assertIs(result, sentinel)
        command = start.call_args.args[1]
        self.assertIn("--return-to-start", command)
        self.assertIn("--reset-stepper-position", command)
        self.assertNotIn("--moonraker-url", command)

    def test_dry_run_uses_ros_coordinator_without_moonraker_write_argument(self) -> None:
        sentinel = object()
        ready_ros = {
            "moveit_ready": True,
            "klipper_bridge_ready": True,
        }
        with mock.patch.object(
            self.backend, "check_klipper", return_value={"ok": True}
        ), mock.patch.object(
            self.backend, "ros_status", return_value=ready_ros
        ), mock.patch.object(
            self.backend,
            "validate_cartesian_test_over_print_bed",
            return_value={"status": "passed"},
        ), mock.patch.object(
            self.backend.jobs, "start", return_value=sentinel
        ) as start:
            result = self.backend.dry_run(self.safety)

        self.assertIs(result, sentinel)
        command = start.call_args.args[1]
        self.assertTrue(str(command[1]).endswith("ur_klipper_sync_test.py"))
        self.assertNotIn("--moonraker-url", command)
        self.assertNotIn("--execute", command)

    def test_dry_run_fails_closed_when_ros_klipper_bridge_is_unavailable(self) -> None:
        with mock.patch.object(
            self.backend, "check_klipper", return_value={"ok": True}
        ), mock.patch.object(
            self.backend,
            "ros_status",
            return_value={"moveit_ready": True, "klipper_bridge_ready": False},
        ), mock.patch.object(self.backend.jobs, "start") as start, self.assertRaises(
            station.ApiError
        ) as raised:
            self.backend.dry_run(self.safety)

        self.assertIn("ROS Klipper bridge", raised.exception.message)
        start.assert_not_called()

    def test_sync_fails_closed_when_ros_klipper_bridge_is_unavailable(self) -> None:
        payload = {
            **self.safety,
            "external_control_playing": True,
            "operator_action": "sync-test",
        }
        with mock.patch.object(
            self.backend, "check_klipper", return_value={"ok": True}
        ), mock.patch.object(
            self.backend,
            "ros_status",
            return_value={
                "external_control": True,
                "moveit_ready": True,
                "klipper_bridge_ready": False,
                "unstable": False,
            },
        ), mock.patch.object(self.backend.jobs, "start") as start, self.assertRaises(
            station.ApiError
        ) as raised:
            self.backend.sync_test(payload)

        self.assertIn("ROS Klipper bridge", raised.exception.message)
        start.assert_not_called()

    def test_early_stop_actions_require_exact_button_intent(self) -> None:
        with self.assertRaises(station.ApiError):
            self.backend.stop_moveit({"operator_action": "stop"})
        with self.assertRaises(station.ApiError):
            self.backend.stop_ros_driver({"operator_action": "stop-ros"})

    def test_early_stop_actions_use_detached_process_scripts(self) -> None:
        sentinel = object()
        with mock.patch.object(self.backend.jobs, "start", return_value=sentinel) as start:
            result = self.backend.stop_moveit({"operator_action": "stop-moveit"})
        self.assertIs(result, sentinel)
        self.assertTrue(str(start.call_args.args[1][0]).endswith("stop_moveit_detached.sh"))

        with mock.patch.object(self.backend.jobs, "start", return_value=sentinel) as start:
            result = self.backend.stop_ros_driver({"operator_action": "stop-ros-driver"})
        self.assertIs(result, sentinel)
        self.assertTrue(str(start.call_args.args[1][0]).endswith("stop_ros_driver_detached.sh"))


class CoordinatedCancellationTests(unittest.TestCase):
    def test_lifecycle_maintenance_blocks_new_jobs_until_released(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manager = station.JobManager(
                job_log_dir=Path(directory) / "jobs",
                cancel_dir=Path(directory) / "cancel",
            )
            token = manager.begin_maintenance("stop-recovery")
            with self.assertRaises(station.ApiError) as raised:
                manager.start("demo", ["unused"], demo_lines=["unused"])
            self.assertEqual(raised.exception.code, "workstation_maintenance")

            manager.end_maintenance(token)
            job = manager.start("demo", ["unused"], demo_lines=[])
            deadline = time.monotonic() + 1.0
            while job.state in {"queued", "running"} and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(job.state, "succeeded")

    def test_shutdown_closes_job_manager_permanently(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manager = station.JobManager(
                job_log_dir=Path(directory) / "jobs",
                cancel_dir=Path(directory) / "cancel",
            )
            result = manager.stop_all(reason="application-exit", close=True)
            self.assertTrue(result["closed"])
            with self.assertRaises(station.ApiError) as raised:
                manager.start("demo", ["unused"], demo_lines=[])
            self.assertEqual(raised.exception.code, "workstation_shutting_down")

    @staticmethod
    def running_motion_job() -> tuple[station.JobManager, station.Job, mock.Mock]:
        manager = station.JobManager()
        process = mock.Mock()
        process.pid = 4321
        process.poll.return_value = None
        job = station.Job(
            id="cancel-test",
            kind="sync-test",
            motion=True,
            command=["fixed-coordinator"],
            state="running",
            process=process,
        )
        manager._jobs[job.id] = job
        return manager, job, process

    def test_motion_cancel_asks_ros_before_terminating_host_process(self) -> None:
        manager, job, _process = self.running_motion_job()
        events: list[str] = []

        def ros_cancel(_command, **_kwargs):
            events.append("ros-cancel")
            return {
                "ok": True,
                "return_code": 0,
                "output": '{"success": true, "safe": true}',
            }

        def terminate_group(_pid, _signal):
            events.append("kill-host")

        with tempfile.TemporaryDirectory() as tempdir, mock.patch.object(
            station, "CANCEL_DIR", Path(tempdir)
        ), mock.patch.object(
            station, "run_capture", side_effect=ros_cancel
        ) as run, mock.patch.object(
            station.os, "killpg", side_effect=terminate_group
        ) as kill:
            result = manager.cancel(job.id)

        self.assertIs(result, job)
        self.assertEqual(events, ["ros-cancel", "kill-host"])
        self.assertEqual(job.state, "canceled")
        kill.assert_called_once_with(4321, station.signal.SIGTERM)
        ros_command = " ".join(run.call_args.args[0])
        self.assertIn("synchronized_motion_cancel.py", ros_command)
        command = run.call_args.args[0]
        self.assertEqual(command[command.index("ubuntu") + 1], "ros2")

    def test_motion_cancel_fails_closed_if_ros_cannot_confirm_safe_stop(self) -> None:
        manager, job, _process = self.running_motion_job()
        with tempfile.TemporaryDirectory() as tempdir, mock.patch.object(
            station, "CANCEL_DIR", Path(tempdir)
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={
                "ok": False,
                "return_code": 2,
                "output": '{"success": false, "safe": false}',
            },
        ), mock.patch.object(station.os, "killpg") as kill, self.assertRaises(
            station.ApiError
        ) as raised:
            manager.cancel(job.id)

        self.assertIn("safe coordinated cancellation", raised.exception.message)
        self.assertEqual(job.state, "running")
        kill.assert_not_called()

    def test_cancel_does_not_return_while_process_creation_is_in_flight(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            manager = station.JobManager(
                job_log_dir=root / "jobs",
                cancel_dir=root / "cancel",
            )
            popen_entered = threading.Event()
            allow_popen = threading.Event()
            cancel_finished = threading.Event()
            process = mock.Mock()
            process.pid = 4321
            process.poll.return_value = None
            process.stdout = iter(())
            process.wait.return_value = 0

            def delayed_popen(*_args, **_kwargs):
                popen_entered.set()
                self.assertTrue(allow_popen.wait(2.0))
                return process

            with mock.patch.object(
                station.subprocess, "Popen", side_effect=delayed_popen
            ), mock.patch.object(station, "_terminate_host_process_tree") as stop:
                job = manager.start("race", ["fixed-command"])
                self.assertTrue(popen_entered.wait(2.0))

                cancel_thread = threading.Thread(
                    target=lambda: (manager.cancel(job.id), cancel_finished.set()),
                    daemon=True,
                )
                cancel_thread.start()
                self.assertFalse(cancel_finished.wait(0.1))
                allow_popen.set()
                self.assertTrue(cancel_finished.wait(2.0))
                cancel_thread.join(timeout=2.0)

            self.assertEqual(job.state, "canceled")
            stop.assert_called_once_with(process)

    def test_cancel_waits_for_launcher_cleanup_before_allowing_restart(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            manager = station.JobManager(
                job_log_dir=root / "jobs",
                cancel_dir=root / "cancel",
            )
            cleanup_started = threading.Event()
            allow_cleanup = threading.Event()
            cancel_finished = threading.Event()
            errors: list[Exception] = []
            process = mock.Mock()
            process.poll.return_value = None

            def finish_cleanup(*_args, **_kwargs):
                cleanup_started.set()
                allow_cleanup.wait(2.0)
                return 143

            process.wait.side_effect = finish_cleanup
            job = station.Job(
                id="start-ros-cancel",
                kind="start-ros",
                motion=False,
                command=["fixed-launcher"],
                state="running",
                process=process,
            )
            manager._jobs[job.id] = job

            def cancel():
                try:
                    manager.cancel(job.id)
                except Exception as exc:
                    errors.append(exc)
                finally:
                    cancel_finished.set()

            with mock.patch.object(station, "_terminate_host_process_tree"):
                cancel_thread = threading.Thread(target=cancel, daemon=True)
                cancel_thread.start()
                try:
                    self.assertTrue(cleanup_started.wait(1.0))
                    self.assertFalse(cancel_finished.is_set())
                    with self.assertRaises(station.ApiError):
                        manager.start("start-ros", ["unused"], demo_lines=[])
                finally:
                    allow_cleanup.set()
                    cancel_thread.join(timeout=2.0)

            self.assertTrue(cancel_finished.is_set())
            self.assertEqual(errors, [])
            self.assertEqual(job.state, "canceled")
            restarted = manager.start("start-ros", ["unused"], demo_lines=[])
            deadline = time.monotonic() + 1.0
            while restarted.state in {"queued", "running"} and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(restarted.state, "succeeded")

    def test_windows_startup_cancel_uses_cooperative_file_and_preserves_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manager = station.JobManager(job_log_dir=root / "jobs", cancel_dir=root / "cancel")
            process = mock.Mock()
            process.poll.return_value = None
            job = station.Job(id="windows-startup", kind="start-ros", motion=False,
                              command=[station.sys.executable, str(station.WINDOWS_RUNTIME), "start-ros"],
                              state="running", process=process)
            manager._jobs[job.id] = job

            def cleanup_finished(**_kwargs):
                self.assertTrue((root / "cancel" / f"{job.id}.cancel").exists())
                self.assertIs(manager.active(), job)
                return 125

            process.wait.side_effect = cleanup_finished
            with mock.patch.object(station.platform, "system", return_value="Windows"), \
                 mock.patch.object(station, "_terminate_host_process_tree") as terminate:
                manager.cancel(job.id)
            terminate.assert_not_called()
            self.assertEqual(job.state, "canceled")
            process.wait.assert_called_once_with(timeout=30.0)

    def test_cancel_keeps_new_jobs_blocked_if_launcher_cleanup_times_out(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            manager = station.JobManager(
                job_log_dir=root / "jobs",
                cancel_dir=root / "cancel",
            )
            process = mock.Mock()
            process.poll.return_value = None
            process.wait.side_effect = station.subprocess.TimeoutExpired("fixed-launcher", 30.0)
            job = station.Job(
                id="cleanup-timeout",
                kind="start-ros",
                motion=False,
                command=["fixed-launcher"],
                state="running",
                process=process,
            )
            manager._jobs[job.id] = job

            with mock.patch.object(station, "_terminate_host_process_tree"):
                with self.assertRaises(station.ApiError) as raised:
                    manager.cancel(job.id)
            self.assertEqual(raised.exception.code, "operation_cleanup_incomplete")
            self.assertEqual(job.state, "running")
            with self.assertRaises(station.ApiError):
                manager.start("start-ros", ["unused"], demo_lines=[])

    def test_job_logs_rotate_and_old_jobs_are_pruned(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            manager = station.JobManager(
                job_log_dir=root / "jobs",
                cancel_dir=root / "cancel",
            )
            manager.job_log_dir.mkdir(parents=True)
            job = station.Job(
                id="bounded-log",
                kind="test",
                motion=False,
                command=["fixed"],
                log_path=manager.job_log_dir / "current.log",
            )
            with mock.patch.object(station, "JOB_LOG_MAX_BYTES", 80):
                for index in range(20):
                    manager._append(job, f"bounded line {index:02d}")

            self.assertTrue(job.log_path.with_suffix(".log.1").is_file())
            for index in range(5):
                (manager.job_log_dir / f"old-{index}.log").write_text(
                    "old", encoding="utf-8"
                )
            manager.prune_logs(keep=2)
            self.assertEqual(
                len(list(manager.job_log_dir.glob("*.log*"))),
                2,
            )


class WindowsRuntimeTests(unittest.TestCase):
    def test_windows_builder_downloads_the_checksum_matched_embeddable_python(self) -> None:
        builder = (
            Path(__file__).resolve().parents[1] / "build-windows-package.command"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'python-$PYTHON_VERSION-embeddable-$architecture.zip',
            builder,
        )
        self.assertNotIn(
            'python-$PYTHON_VERSION-embed-$architecture.zip',
            builder,
        )

    def test_windows_builds_include_print_execution_backend(self) -> None:
        software = Path(__file__).resolve().parents[1]
        shell_builder = (software / "build-windows-package.command").read_text(
            encoding="utf-8"
        )
        powershell_builder = (software / "build-windows-package.ps1").read_text(
            encoding="utf-8"
        )
        # Both builders select all shared modules, including print execution,
        # instead of maintaining a second list that can omit a new backend.
        self.assertTrue((software / "print_execution.py").is_file())
        self.assertIn('"$SOFTWARE_DIR"/*.py', shell_builder)
        self.assertIn('Get-ChildItem -LiteralPath $SoftwareDir -Filter "*.py" -File', powershell_builder)

    def test_windows_builds_include_every_runtime_python_module(self) -> None:
        software = Path(__file__).resolve().parents[1]
        import shutil
        if shutil.which("zsh") is None:
            self.skipTest("the macOS cross-builder needs zsh")
        spec = importlib.util.spec_from_file_location(
            "windows_package_builder_fixture", software / "tests/test_windows_package_builder.py"
        )
        assert spec is not None and spec.loader is not None
        fixture_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fixture_module)
        fixture = fixture_module.WindowsPayloadSnapshotTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.snapshot()
        expected = {path.name for path in (fixture.root / "Software").glob("*.py")}
        packaged = {path.name for path in (fixture.destination / "Software").glob("*.py")}
        self.assertEqual(packaged, expected)
        self.assertIn("future_shared_module.py", packaged)

    def test_windows_powershell_python_url_matches_pinned_checksums(self) -> None:
        builder = (
            Path(__file__).resolve().parents[1] / "build-windows-package.ps1"
        ).read_text(encoding="utf-8")
        self.assertIn("python-$PythonVersion-embeddable-$Architecture.zip", builder)
        self.assertNotIn("python-$PythonVersion-embed-$Architecture.zip", builder)

    def test_windows_fixed_action_uses_bash_free_python_runtime(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"
        with tempfile.TemporaryDirectory() as tempdir:
            runtime = Path(tempdir) / "windows_runtime.py"
            runtime.touch()
            with mock.patch.object(
                station.platform, "system", return_value="Windows"
            ), mock.patch.object(
                station, "WINDOWS_RUNTIME", runtime
            ), mock.patch.object(
                station.shutil, "which", return_value=None
            ):
                command = station.platform_operation_command(
                    "stop-ros",
                    script,
                )
        self.assertEqual(
            command,
            [station.sys.executable, str(runtime), "stop-ros"],
        )

    def test_windows_height_and_print_z_trials_use_native_runtime_with_all_guards(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            backend = station.StationBackend(demo=True, data_dir=Path(directory))
            backend.demo = False
            nozzle = {"x": 0.2, "y": 0.0, "z": 0.05}
            saved = {"profile_sha256": "a" * 64, "calibration": {
                "tool_center_point_m": {"offset": {"x": 0.0, "y": 0.0, "z": 0.12}},
            }}
            with mock.patch.object(station.platform, "system", return_value="Windows"), \
                 mock.patch.object(backend, "_environment", return_value={}), \
                 mock.patch.object(backend, "_require_operator_action"), \
                 mock.patch.object(backend, "_height_trial_target", return_value=({}, nozzle, -0.02)), \
                 mock.patch.object(backend, "_print_z_trial_target", return_value=({}, nozzle, -0.02, 0.0, saved)), \
                 mock.patch.object(station, "read_saved_calibration", return_value=None), \
                 mock.patch.object(backend.jobs, "start", return_value=station.Job(
                     id="b" * 12, kind="trial", motion=False, command=[], state="succeeded",
                 )) as start:
                backend._height_draft_m = 0.02
                backend.start_workcell_height_trial({}, execute=False)
                backend._height_draft_m = None
                backend._print_z_draft_m = 0.01
                backend.start_print_z_trial({}, execute=False)
            self.assertEqual(start.call_count, 2)
            for call in start.call_args_list:
                self.assertEqual(call.args[1], [station.sys.executable, str(station.WINDOWS_RUNTIME), "cartesian-move"])
                self.assertEqual(call.kwargs["env"]["SYNCHRONIZE_KLIPPER"], "false")
                self.assertEqual(call.kwargs["env"]["HEIGHT_TRIAL_MODE"], "true")
                self.assertEqual(call.kwargs["env"]["AXES"], "z")
                self.assertIn("PRINTHEAD_GEOMETRY_SHA256", call.kwargs["env"])

    def test_windows_fixed_demo_action_never_requires_runtime_or_docker(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station.shutil, "which", return_value=None
        ):
            command = station.platform_operation_command(
                "stop-ros",
                script,
                demo=True,
            )
        self.assertEqual(command, ["demo-host-script", "stop.sh"])

    def test_windows_demo_script_has_no_runtime_dependency(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station.shutil, "which", return_value=None
        ):
            command = station.host_script_command(script, demo=True)
        self.assertEqual(command, ["demo-host-script", "stop.sh"])

    def test_windows_host_script_uses_wsl_not_direct_shell_file(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"

        def which(name: str):
            return {
                "docker.exe": r"C:\Program Files\Docker\docker.exe",
                "wsl.exe": r"C:\Windows\System32\wsl.exe",
            }.get(name)

        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.dict(station.os.environ, {}, clear=True), mock.patch.object(
            station.shutil, "which", side_effect=which
        ):
            command = station.host_script_command(script, "one")

        self.assertTrue(command[0].lower().endswith("wsl.exe"))
        self.assertEqual(command[1:4], ["--exec", "bash", "-lc"])
        self.assertNotEqual(command[0], str(script))
        self.assertEqual(command[-2:], [str(script.resolve()), "one"])

    def test_windows_host_script_fails_closed_without_docker(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.dict(station.os.environ, {}, clear=True), mock.patch.object(
            station.shutil, "which", return_value=None
        ), self.assertRaises(station.ApiError) as raised:
            station.host_script_command(script)
        self.assertEqual(raised.exception.status, 503)
        self.assertIn("Docker Desktop", raised.exception.message)

    def test_windows_host_script_fails_closed_without_bash_or_wsl(self) -> None:
        script = station.PROJECT_ROOT / "scripts" / "stop.sh"

        def which(name: str):
            return r"C:\Program Files\Docker\docker.exe" if name == "docker.exe" else None

        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.dict(station.os.environ, {}, clear=True), mock.patch.object(
            station.shutil, "which", side_effect=which
        ), self.assertRaises(station.ApiError) as raised:
            station.host_script_command(script)
        self.assertEqual(raised.exception.status, 503)
        self.assertIn("No host script was started", raised.exception.message)

    def test_windows_process_group_uses_create_new_process_group(self) -> None:
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ):
            options = station._process_group_options()
        self.assertNotIn("start_new_session", options)
        self.assertEqual(
            options["creationflags"],
            getattr(station.subprocess, "CREATE_NEW_PROCESS_GROUP", 0x00000200),
        )

    def test_windows_process_tree_uses_taskkill(self) -> None:
        process = mock.Mock()
        process.pid = 4321
        process.poll.return_value = None
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ), mock.patch.object(
            station,
            "run_capture",
            return_value={"ok": True, "return_code": 0, "output": "SUCCESS"},
        ) as run:
            station._terminate_host_process_tree(process)
        self.assertEqual(
            run.call_args.args[0],
            ["taskkill", "/PID", "4321", "/T", "/F"],
        )
        process.terminate.assert_not_called()

    def test_project_executor_discovery_requires_exact_managed_script_path(self) -> None:
        script = station.PRINT_EXECUTION_HOST
        output = (
            f" 123 1 python3 {script} 0123456789abcdef execute\n"
            " 124 1 python3 /tmp/run_gcode_print_execution.py 0123456789abcdef execute\n"
            f" 125 1 editor --file {script}.notes\n"
        )
        processes = station._parse_project_executor_processes(output, script)
        self.assertEqual(processes, [(123, f"python3 {script} 0123456789abcdef execute")])

    def test_wsl_environment_forwards_cancel_path_with_translation(self) -> None:
        environment = {
            "ROBOT_IP": "192.168.0.104",
            "WORKSTATION_CANCEL_FILE": r"C:\Temp\cancel",
        }
        with mock.patch.object(
            station.platform, "system", return_value="Windows"
        ):
            result = station._wsl_environment(
                environment,
                [r"C:\Windows\System32\wsl.exe", "--exec", "bash"],
            )
        self.assertIn("ROBOT_IP", result["WSLENV"])
        self.assertIn("WORKSTATION_CANCEL_FILE/p", result["WSLENV"])


class OfflineModeApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server, cls.backend = station.create_server("127.0.0.1", 0, demo=True)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def request_json(
        self,
        path: str,
        *,
        method: str = "GET",
        payload: dict | None = None,
    ) -> tuple[int, dict]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.base + path,
            method=method,
            data=data,
            headers={"Content-Type": "application/json", "Origin": self.base},
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_offline_mode_blocks_live_endpoints_but_keeps_local_gcode(self) -> None:
        status, mode = self.request_json("/api/mode")
        self.assertEqual(status, 200)
        self.assertEqual(mode["mode"], "offline")

        status, blocked = self.request_json("/api/ros/tool0-pose")
        self.assertEqual(status, 412)
        self.assertIn("Online mode", blocked["error"])

        status, blocked = self.request_json(
            "/api/ur/check",
            method="POST",
            payload={},
        )
        self.assertEqual(status, 412)
        self.assertIn("Online mode", blocked["error"])

        request = urllib.request.Request(
            self.base + "/api/gcode/parse",
            method="POST",
            data=b"G21\nG90\nG1 X5 Y2 F600\n",
            headers={
                "Content-Type": "application/octet-stream",
                "Origin": self.base,
                "X-Gcode-Filename": "offline.gcode",
            },
        )
        with urllib.request.urlopen(request, timeout=3) as response:
            parsed = json.loads(response.read().decode("utf-8"))
        self.assertTrue(parsed["ok"])

        status, changed = self.request_json(
            "/api/mode",
            method="PUT",
            payload={"mode": "online"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(changed["mode"], "online")
        status, live = self.request_json("/api/ros/tool0-pose")
        self.assertEqual(status, 200)
        self.assertEqual(live["pose"]["source"], "simulated_ros")


class ApiSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server, cls.backend = station.create_server("127.0.0.1", 0, demo=True)
        cls.backend.set_operating_mode({"mode": "online"})
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def get_json(self, path: str) -> dict:
        with urllib.request.urlopen(self.base + path, timeout=3) as response:
            return json.loads(response.read().decode("utf-8"))

    def post_json(self, path: str, payload: dict) -> tuple[int, dict]:
        request = urllib.request.Request(
            self.base + path,
            method="POST",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json", "Origin": self.base},
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def put_json(self, path: str, payload: dict) -> tuple[int, dict]:
        request = urllib.request.Request(
            self.base + path,
            method="PUT",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json", "Origin": self.base},
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def post_gcode(
        self,
        filename: str,
        content: bytes,
        endpoint: str = "/api/gcode/parse",
        extra_headers: dict[str, str] | None = None,
    ) -> tuple[int, dict]:
        headers = {
            "Content-Type": "application/octet-stream",
            "Origin": self.base,
            "X-Gcode-Filename": urllib.parse.quote(filename),
        }
        headers.update(extra_headers or {})
        request = urllib.request.Request(
            self.base + endpoint,
            method="POST",
            data=content,
            headers=headers,
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_health_and_static_app(self) -> None:
        health = self.get_json("/api/health")
        self.assertTrue(health["demo"])
        with urllib.request.urlopen(self.base + "/", timeout=3) as response:
            html = response.read().decode("utf-8")
        self.assertIn("UR Print Workstation", html)
        self.assertIn("/app.js", html)

    def test_operator_recovery_api_is_part_of_the_product(self) -> None:
        recovery = self.get_json("/api/print/recovery")
        self.assertFalse(recovery["print_interlock"]["locked"])

        status, payload = self.post_json(
            "/api/print/recovery",
            {"operator_action": "verify-stop-and-recover"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["recovery"]["outcome"], "no-lock")

    def test_global_status_exposes_stop_interlock(self) -> None:
        payload = self.get_json("/api/status")
        self.assertIn("print_interlock", payload)
        self.assertFalse(payload["print_interlock"]["locked"])

    def test_read_only_device_checks(self) -> None:
        status, ur = self.post_json("/api/ur/check", {})
        self.assertEqual(status, 200)
        self.assertTrue(ur["ok"])
        status, klipper = self.post_json("/api/klipper/check", {})
        self.assertEqual(status, 200)
        self.assertTrue(klipper["ok"])

    def test_cross_origin_request_is_rejected(self) -> None:
        request = urllib.request.Request(
            self.base + "/api/ur/check",
            method="POST",
            data=b"{}",
            headers={"Content-Type": "application/json", "Origin": "https://example.com"},
        )
        with self.assertRaises(urllib.error.HTTPError) as raised:
            urllib.request.urlopen(request, timeout=3)
        self.assertEqual(raised.exception.code, 403)

    def test_gcode_parse_endpoint_is_offline_and_returns_analysis(self) -> None:
        status, payload = self.post_gcode(
            "benchy.gcode",
            b"G21\nG90\nM83\nG1 X10 Y5 E1 F600\n",
        )

        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["analysis"]["file"]["name"], "benchy.gcode")
        self.assertEqual(payload["analysis"]["summary"]["print_move_count"], 1)

    def test_gcode_parse_endpoint_rejects_non_gcode_files(self) -> None:
        status, payload = self.post_gcode("notes.txt", b"G1 X10 F600\n")

        self.assertEqual(status, 422)
        self.assertFalse(payload["ok"])

    def test_gcode_plan_endpoint_returns_separate_control_channels(self) -> None:
        status, payload = self.post_gcode(
            "benchy.gcode",
            b"G21\nG90\nM83\nG28\nG1 X10 Y5 E1 F600\n",
            endpoint="/api/gcode/plan",
        )

        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["plan"]["status"], "draft")
        self.assertTrue(payload["plan"]["execution_locked"])
        self.assertEqual(payload["plan"]["ur_robot"]["waypoint_count"], 1)
        self.assertEqual(
            payload["plan"]["klipper"]["preparation_commands"][0]["command"],
            "G28",
        )

    def test_calibration_endpoints_validate_and_save_without_hardware(self) -> None:
        profile = station.default_calibration()
        profile["coordinate_transform"]["origin_m"] = {
            "x": 0.4,
            "y": -0.2,
            "z": 0.1,
        }
        profile["print_bed_m"] = {
            "plane_z": 0.1,
            "minimum": {"x": 0.3, "y": -0.3},
            "maximum": {"x": 0.6, "y": 0.1},
            "edge_margin_m": 0.01,
        }
        profile["tool_center_point_m"] = {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.05},
            "collision_radius_m": 0.01,
        }
        profile["confirmations"] = {
            "origin_measured": True,
            "orientation_verified": True,
            "tool_orientation_verified": True,
            "extrusion_ratio_measured": True,
            "tcp_measured": True,
            "print_bed_measured": True,
            "tool0_reference_verified": False,
        }
        payload = {
            "calibration": profile,
            "bounds": {
                "minimum": {"x": 0, "y": 0, "z": 0},
                "maximum": {"x": 100, "y": 150, "z": 25},
            },
            "source_sha256": "b" * 64,
        }

        status, validated = self.post_json(
            "/api/gcode/calibration/validate",
            payload,
        )
        self.assertEqual(status, 200)
        self.assertTrue(validated["validation"]["ready_to_save"])
        self.assertTrue(validated["validation"]["execution_locked"])

        status, saved = self.put_json("/api/gcode/calibration", payload)
        self.assertEqual(status, 200)
        self.assertTrue(saved["saved"])
        self.assertEqual(saved["source_sha256"], "b" * 64)

        loaded = self.get_json("/api/gcode/calibration")
        self.assertTrue(loaded["saved"])
        self.assertEqual(loaded["calibration"], saved["calibration"])
        self.assertEqual(
            loaded["workcell_geometry"]["tabletop"]["center_m"],
            {"x": -0.2925, "y": 0.0},
        )
        self.assertEqual(
            loaded["workcell_geometry"]["tabletop"]["dimensions_m"],
            {"x": 0.45, "y": 0.54},
        )
        self.assertEqual(
            loaded["workcell_geometry"]["tabletop"]["surface_z_m"],
            station.load_workcell_geometry()["tabletop"]["surface_z_m"],
        )

    def test_demo_tool0_pose_endpoint_is_read_only(self) -> None:
        payload = self.get_json("/api/ros/tool0-pose")
        self.assertTrue(payload["ok"])
        self.assertTrue(payload["pose"]["read_only"])
        self.assertTrue(payload["pose"]["execution_locked"])
        self.assertEqual(payload["pose"]["frame_id"], "base_link")

    def test_calibration_save_requires_confirmations(self) -> None:
        payload = {
            "calibration": station.default_calibration(),
            "bounds": {
                "minimum": {"x": 0, "y": 0, "z": 0},
                "maximum": {"x": 10, "y": 10, "z": 10},
            },
            "source_sha256": "c" * 64,
        }

        status, response = self.put_json("/api/gcode/calibration", payload)
        self.assertEqual(status, 412)
        self.assertFalse(response["ok"])

    def test_3d_review_and_demo_moveit_validation_remain_locked(self) -> None:
        content = b"G21\nG90\nM83\nG1 X10 Y5 Z0.2 E1 F600\nG1 X20 Y5 E1\n"
        source_sha256 = hashlib.sha256(content).hexdigest()
        profile = station.default_calibration()
        profile["coordinate_transform"]["origin_m"] = {
            "x": 0.4,
            "y": -0.2,
            "z": 0.1,
        }
        profile["print_bed_m"] = {
            "plane_z": 0.1,
            "minimum": {"x": 0.3, "y": -0.3},
            "maximum": {"x": 0.6, "y": 0.1},
            "edge_margin_m": 0.01,
        }
        profile["tool_center_point_m"] = {
            "offset": {"x": 0.0, "y": 0.0, "z": 0.05},
            "collision_radius_m": 0.01,
        }
        profile["confirmations"] = {
            "origin_measured": True,
            "orientation_verified": True,
            "tool_orientation_verified": True,
            "extrusion_ratio_measured": True,
            "tcp_measured": True,
            "print_bed_measured": True,
            "tool0_reference_verified": False,
        }
        calibration_payload = {
            "calibration": profile,
            "bounds": {
                "minimum": {"x": 0, "y": 0, "z": 0},
                "maximum": {"x": 20, "y": 5, "z": 0.2},
            },
            "source_sha256": source_sha256,
        }
        status, _saved = self.put_json(
            "/api/gcode/calibration",
            calibration_payload,
        )
        self.assertEqual(status, 200)

        status, reviewed = self.post_gcode(
            "review.gcode",
            content,
            endpoint="/api/gcode/review",
        )
        self.assertEqual(status, 200)
        self.assertTrue(reviewed["review"]["execution_locked"])
        self.assertGreater(
            len(reviewed["review"]["path"]["segments"]),
            0,
        )
        offline_simulation = reviewed["review"]["offline_simulation"]
        self.assertEqual(offline_simulation["status"], "passed")
        self.assertTrue(offline_simulation["offline_estimate"])
        self.assertTrue(offline_simulation["execution_locked"])
        self.assertIsNone(offline_simulation["collision_free"])
        self.assertGreater(len(offline_simulation["trajectory"]), 1)

        status, started = self.post_gcode(
            "review.gcode",
            content,
            endpoint="/api/gcode/review/moveit",
        )
        self.assertEqual(status, 202)
        job_id = started["job"]["id"]
        deadline = time.time() + 4
        result_payload = {}
        while time.time() < deadline:
            result_payload = self.get_json(
                f"/api/gcode/review/moveit/{job_id}"
            )
            if result_payload["job"]["state"] not in {"queued", "running"}:
                break
            time.sleep(0.05)

        self.assertEqual(result_payload["job"]["state"], "succeeded")
        self.assertTrue(result_payload["result"]["simulation"])
        self.assertTrue(result_payload["result"]["execution_locked"])
        self.assertEqual(result_payload["result"]["status"], "passed")

        status, prepared = self.post_gcode(
            "review.gcode",
            content,
            endpoint="/api/print/prepare",
            extra_headers={"X-Moveit-Job-Id": job_id},
        )
        self.assertEqual(status, 200)
        package = prepared["package"]
        self.assertTrue(package["simulation"])
        self.assertFalse(package["ready_for_authorization"])
        self.assertGreater(package["block_count"], 0)
        execution_id = package["execution_id"]

        safety = {
            "execution_id": execution_id,
            "emergency_stop": True,
            "workspace_clear": True,
            "reviewed_low_speed": True,
            "wired_network": True,
            "external_control_playing": True,
        }
        status, rejected = self.post_json(
            "/api/print/move-to-start",
            {**safety, "operator_action": "execute"},
        )
        self.assertEqual(status, 412)
        self.assertIn("move-to-start", rejected["error"])

        status, started = self.post_json(
            "/api/print/move-to-start",
            {**safety, "operator_action": "move-to-start"},
        )
        self.assertEqual(status, 202)
        move_job_id = started["job"]["id"]
        deadline = time.time() + 4
        while time.time() < deadline:
            move_job = self.get_json(f"/api/jobs/{move_job_id}")["job"]
            if move_job["state"] not in {"queued", "running"}:
                break
            time.sleep(0.05)
        self.assertEqual(move_job["state"], "succeeded")
        at_start = self.get_json(f"/api/print/{execution_id}")
        self.assertEqual(at_start["runtime"]["state"], "at-start")

        status, printing = self.post_json(
            "/api/print/start",
            {**safety, "operator_action": "execute"},
        )
        self.assertEqual(status, 202)
        print_job_id = printing["job"]["id"]
        deadline = time.time() + 4
        while time.time() < deadline:
            print_job = self.get_json(f"/api/jobs/{print_job_id}")["job"]
            if print_job["state"] not in {"queued", "running"}:
                break
            time.sleep(0.05)
        self.assertEqual(print_job["state"], "succeeded")
        completed = self.get_json(f"/api/print/{execution_id}")
        self.assertEqual(completed["runtime"]["state"], "completed")
        self.assertEqual(completed["runtime"]["current_block"], package["block_count"])

        # Merely starting a replacement review revokes old authorization. A
        # failed or unfinished replacement must never resurrect this package.
        status, _replacement = self.post_gcode(
            "review.gcode", content, endpoint="/api/gcode/review/moveit",
        )
        self.assertEqual(status, 202)
        status, rejected = self.post_gcode(
            "review.gcode", content, endpoint="/api/print/prepare",
            extra_headers={"X-Moveit-Job-Id": job_id},
        )
        self.assertEqual(status, 412)
        self.assertIn("superseded", rejected["error"])
        for endpoint, action in (
            ("/api/print/move-to-start", "move-to-start"),
            ("/api/print/start", "execute"),
        ):
            status, rejected = self.post_json(
                endpoint, {**safety, "operator_action": action},
            )
            self.assertEqual(status, 412)
            self.assertIn("planning_request", rejected["error"])


if __name__ == "__main__":
    unittest.main()
