#!/usr/bin/env python3
"""Local UR3e + Klipper control station.

The server intentionally binds to localhost and exposes only a fixed set of
safe, project-owned operations. It does not accept arbitrary shell commands.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import ipaddress
import json
import math
import mimetypes
import os
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import weakref
import webbrowser
from dataclasses import dataclass, field
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

try:
    from gcode_calibration import (
        CalibrationNotReadyError,
        CalibrationValidationError,
        default_calibration,
        print_z_candidate_ready_for_planning,
        read_saved_calibration,
        save_calibration,
        save_installed_calibration_draft,
        validate_calibration,
        validate_relative_motion_over_print_bed,
    )
    from gcode_parser import (
        GCodeParseError,
        MAX_GCODE_BYTES,
        parse_gcode,
    )
    from gcode_planner import GCodePlanError, build_path_plan
    from gcode_path_review import (
        MAX_MOVEIT_WAYPOINTS,
        OFFLINE_SIMULATION_ALGORITHM_VERSION,
        PathReviewError,
        begin_planning_request,
        build_offline_review,
        build_offline_robot_simulation,
        demo_moveit_result,
        current_planning_request,
        find_cached_moveit_result,
        read_moveit_result,
        write_moveit_path,
    )
    from print_execution import (
        PrintExecutionError,
        estimate_minimum_duration_s,
        prepare_execution_manifest,
        read_execution_manifest,
    )
    from paired_timeline_import import PairedTimelineError, parse_paired_timeline
    from runtime_lifecycle import LifecycleBlocked, RuntimeLifecycle
    from ur_stop_proof import verify_stationary_rtde
    from printhead_geometry import load_printhead_geometry, printhead_geometry_sha256
    from workcell_geometry import WorkcellGeometryError, load_workcell_geometry, save_surface_height_m
except ModuleNotFoundError:
    from Software.gcode_calibration import (
        CalibrationNotReadyError,
        CalibrationValidationError,
        default_calibration,
        print_z_candidate_ready_for_planning,
        read_saved_calibration,
        save_calibration,
        save_installed_calibration_draft,
        validate_calibration,
        validate_relative_motion_over_print_bed,
    )
    from Software.gcode_parser import (
        GCodeParseError,
        MAX_GCODE_BYTES,
        parse_gcode,
    )
    from Software.gcode_planner import GCodePlanError, build_path_plan
    from Software.gcode_path_review import (
        MAX_MOVEIT_WAYPOINTS,
        OFFLINE_SIMULATION_ALGORITHM_VERSION,
        PathReviewError,
        begin_planning_request,
        build_offline_review,
        build_offline_robot_simulation,
        demo_moveit_result,
        current_planning_request,
        find_cached_moveit_result,
        read_moveit_result,
        write_moveit_path,
    )
    from Software.print_execution import (
        PrintExecutionError,
        estimate_minimum_duration_s,
        prepare_execution_manifest,
        read_execution_manifest,
    )
    from Software.paired_timeline_import import PairedTimelineError, parse_paired_timeline
    from Software.runtime_lifecycle import LifecycleBlocked, RuntimeLifecycle
    from Software.ur_stop_proof import verify_stationary_rtde
    from Software.printhead_geometry import load_printhead_geometry, printhead_geometry_sha256
    from Software.workcell_geometry import WorkcellGeometryError, load_workcell_geometry, save_surface_height_m


SOFTWARE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(
    os.environ.get("UR_PRINT_PROJECT_ROOT", str(SOFTWARE_DIR.parent))
).expanduser().resolve()
STATIC_DIR = SOFTWARE_DIR / "app"
HEIGHT_TRIAL_CLEARANCE_M = 0.010


def resolve_data_dir(
    *,
    system_name: str | None = None,
    environment: dict[str, str] | None = None,
) -> Path:
    """Return a writable per-user state directory without moving app resources."""

    environment = os.environ if environment is None else environment
    override = environment.get("UR_PRINT_DATA_DIR", "").strip()
    if override:
        return Path(os.path.expandvars(os.path.expanduser(override))).resolve()

    system_name = platform.system() if system_name is None else system_name
    if system_name == "Windows":
        local_app_data = environment.get("LOCALAPPDATA", "").strip()
        root = (
            Path(local_app_data)
            if local_app_data
            else Path.home() / "AppData" / "Local"
        )
        return root / "UR Print Workstation"

    if system_name == "Darwin":
        return Path.home() / "Library" / "Application Support" / "UR Print Workstation"

    if system_name == "Linux":
        xdg_data_home = environment.get("XDG_DATA_HOME", "").strip()
        root = Path(xdg_data_home) if xdg_data_home else Path.home() / ".local" / "share"
        return root / "ur-print-workstation"

    # Unknown platforms retain a portable fallback.
    return SOFTWARE_DIR


DATA_DIR = resolve_data_dir()
CONFIG_PATH = DATA_DIR / "config.json"
CALIBRATION_PATH = DATA_DIR / "gcode_calibration.json"
RUNTIME_DIR = DATA_DIR / "runtime"
JOB_LOG_DIR = RUNTIME_DIR / "jobs"
CANCEL_DIR = RUNTIME_DIR / "cancel"
JOB_LOG_MAX_BYTES = 5 * 1024 * 1024
JOB_LOG_KEEP_COUNT = 100
# 3D review generation is CPU-heavy pure Python work. Keep one builder active
# and at most two waiting requests so repeated browser retries cannot create an
# unbounded queue of 20 MB request bodies. Only the four newest terminal review
# payloads are retained because each may contain thousands of path segments;
# that also guarantees every accepted request in the three-job queue can still
# be polled after a burst completes.
ASYNC_REVIEW_MAX_ACTIVE = 3
ASYNC_REVIEW_MAX_RETAINED = 4
ASYNC_REVIEW_ALGORITHM_VERSION = OFFLINE_SIMULATION_ALGORITHM_VERSION
_CURRENT_CALIBRATION = object()
PATH_REVIEW_RUNTIME = (
    PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "runtime" / "path_review"
)
PRINT_EXECUTION_RUNTIME = (
    PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "runtime" / "print_execution"
)
PRINT_EXECUTION_HOST = PROJECT_ROOT / "scripts" / "run_gcode_print_execution.py"
COMPOSE_FILE = PROJECT_ROOT / "docker-compose.real.yml"
DRIVER_LOG = PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "real_robot_driver_live.log"
MOVEIT_LOG = PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "move_group_live.log"
RVIZ_SNAPSHOT = PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "rviz_screenshot.png"
EXTERNAL_CONTROL_PROBE = (
    PROJECT_ROOT
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "external_control_readiness_probe.py"
)
KLIPPER_BRIDGE_PROBE = (
    PROJECT_ROOT
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "klipper_bridge_readiness_probe.py"
)
SYNCHRONIZED_CANCEL_CLIENT = (
    PROJECT_ROOT
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "synchronized_motion_cancel.py"
)
WINDOWS_RUNTIME = SOFTWARE_DIR / "windows_runtime.py"

VERSION = "0.14.1"


def migrate_legacy_state(legacy_dir: Path, destination: Path) -> list[str]:
    """Copy irreplaceable operator state once, never overwriting newer data."""

    migrated: list[str] = []
    if legacy_dir.resolve() == destination.resolve():
        return migrated
    for name in ("config.json", "gcode_calibration.json"):
        source = legacy_dir / name
        target = destination / name
        if not source.is_file() or target.exists():
            continue
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        migrated.append(name)
    return migrated

OPERATING_MODES = {"offline", "online"}

UR_REVERSE_PORTS = (50001, 50002, 50003, 50004)

DEFAULT_CONFIG: dict[str, Any] = {
    "language": "en",
    "ur_robot_ip": "192.168.0.104",
    "ros_pc_ip": "",
    "klipper_url": "http://192.168.0.168",
    # Persisted key retained for existing operator profiles; its value is now
    # exclusively a native Klipper extruder object, never a manual_stepper.
    "manual_stepper": "extruder",
    "ur_update_rate": 125,
    "axes": "x,y,z",
    "distance_m": 0.02,
    "max_joint_speed_rad_s": 0.03,
    "stepper_move": 60.0,
    "stepper_speed": 5.0,
    "stepper_accel": 20.0,
}


class ApiError(Exception):
    def __init__(
        self,
        status: int,
        message: str,
        *,
        code: str | None = None,
        details: dict[str, Any] | None = None,
    ):
        super().__init__(message)
        self.status = status
        self.message = message
        self.code = code
        self.details = details or {}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def tail_text(path: Path, max_bytes: int = 240_000) -> str:
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes))
            return handle.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def run_capture(
    command: list[str],
    *,
    timeout: float = 8.0,
    env: dict[str, str] | None = None,
    cwd: Path = PROJECT_ROOT,
) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "ok": completed.returncode == 0,
            "return_code": completed.returncode,
            "output": completed.stdout.strip(),
        }
    except FileNotFoundError as exc:
        return {"ok": False, "return_code": 127, "output": str(exc)}
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        return {"ok": False, "return_code": 124, "output": f"{output}\nTimed out".strip()}


def host_script_command(
    script: Path,
    *arguments: str,
    demo: bool = False,
) -> list[str]:
    """Build a safe host command for a project Bash script.

    Windows must never ask CreateProcess to execute a ``.sh`` file directly.
    Docker Desktop normally provides ``docker.exe`` while either Git Bash or a
    WSL distribution provides Bash.  WSL receives the Windows script path as a
    positional argument and converts it without evaluating user-controlled
    text as shell source.
    """

    script = script.resolve()
    if demo:
        # JobManager consumes demo_lines without spawning this command. Keep a
        # descriptive, non-executable command so Demo mode never acquires a
        # Docker/Bash prerequisite on Windows.
        return ["demo-host-script", script.name, *map(str, arguments)]
    if platform.system() != "Windows":
        return [str(script), *map(str, arguments)]

    docker = shutil.which("docker.exe") or shutil.which("docker")
    if docker is None:
        raise ApiError(
            HTTPStatus.SERVICE_UNAVAILABLE,
            "Docker Desktop is required for Windows online operations. "
            "Install Docker Desktop, start its Linux engine, then try again.",
        )

    configured_bash = os.environ.get("UR_PRINT_BASH", "").strip()
    if configured_bash:
        bash = shutil.which(configured_bash) or (
            configured_bash if Path(configured_bash).is_file() else None
        )
        if bash is None:
            raise ApiError(
                HTTPStatus.SERVICE_UNAVAILABLE,
                "UR_PRINT_BASH does not point to an available Bash executable.",
            )
        return [str(bash), str(script), *map(str, arguments)]

    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if wsl is not None:
        launcher = (
            'script="$(wslpath -a -- "$1")" || exit 127; '
            'shift; exec "$script" "$@"'
        )
        return [
            str(wsl),
            "--exec",
            "bash",
            "-lc",
            launcher,
            "ur-print-workstation",
            str(script),
            *map(str, arguments),
        ]

    git_bash = shutil.which("bash.exe") or shutil.which("bash")
    if git_bash is not None:
        return [str(git_bash), str(script), *map(str, arguments)]

    raise ApiError(
        HTTPStatus.SERVICE_UNAVAILABLE,
        "Windows online operations require Docker Desktop plus Bash. "
        "Enable Docker Desktop WSL integration, install WSL, or set "
        "UR_PRINT_BASH to Git Bash. No host script was started.",
    )


def platform_operation_command(
    windows_action: str,
    host_script: Path,
    *arguments: str,
    demo: bool = False,
) -> list[str]:
    """Select the native host orchestrator for a fixed workstation action."""

    if not re.fullmatch(r"[a-z][a-z0-9-]*", windows_action):
        raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, "Invalid workstation action.")
    if demo or platform.system() != "Windows":
        return host_script_command(host_script, *arguments, demo=demo)
    if not WINDOWS_RUNTIME.is_file():
        raise ApiError(
            HTTPStatus.SERVICE_UNAVAILABLE,
            "The Windows Docker runtime is missing from this application package. "
            "Re-extract the complete Windows ZIP, then try again.",
        )
    return [
        sys.executable,
        str(WINDOWS_RUNTIME),
        windows_action,
        *map(str, arguments),
    ]


def _wsl_environment(environment: dict[str, str], command: list[str]) -> dict[str, str]:
    """Forward the small approved runtime environment when a command uses WSL."""

    if platform.system() != "Windows" or not command:
        return environment
    executable = re.split(r"[\\/]", command[0])[-1].casefold()
    if executable not in {"wsl", "wsl.exe"}:
        return environment

    forwarded = (
        "ROBOT_IP",
        "ROS_PC_IP",
        "UR_REVERSE_BIND_IP",
        "UR_UPDATE_RATE",
        "MOONRAKER_URL",
        "KLIPPER_STEPPER",
        "WAIT_FOR_READY",
        "TIMEOUT_SECONDS",
        "WORKSTATION_JOB_ID",
        "AXES", "DISTANCE_M", "EXECUTE", "SYNCHRONIZE_KLIPPER",
        "KLIPPER_CHECK_ONLY", "RETURN_TO_START", "MAX_JOINT_SPEED_RAD_S",
        "VELOCITY_SCALING", "ACCELERATION_SCALING", "HEIGHT_TRIAL_MODE",
        "HEIGHT_TRIAL_SURFACE_Z_M", "HEIGHT_TRIAL_MIN_X_M", "HEIGHT_TRIAL_MAX_X_M",
        "HEIGHT_TRIAL_MIN_Y_M", "HEIGHT_TRIAL_MAX_Y_M", "HEIGHT_TRIAL_TCP_X_M",
        "HEIGHT_TRIAL_TCP_Y_M", "HEIGHT_TRIAL_TCP_Z_M", "PRINTHEAD_GEOMETRY_SHA256",
        "CONFIRM_REAL_ROBOT_MOVE", "CONFIRM_CARTESIAN_MOVE",
    )
    requested = [name for name in forwarded if name in environment]
    if "WORKSTATION_CANCEL_FILE" in environment:
        requested.append("WORKSTATION_CANCEL_FILE/p")
    existing = [
        item
        for item in environment.get("WSLENV", "").split(":")
        if item
    ]
    environment["WSLENV"] = ":".join(dict.fromkeys([*existing, *requested]))
    return environment


def _public_listener_lines(output: str, port: int) -> list[str]:
    """Return listeners that are reachable beyond localhost for one TCP port."""

    endpoint = re.compile(
        rf"(?<![\d.])(?:0\.0\.0\.0|\*|\[::\]|::|(?!(?:127\.))"
        rf"\d{{1,3}}(?:\.\d{{1,3}}){{3}}):{port}(?:\s|\b)",
        re.IGNORECASE,
    )
    return [
        line.strip()
        for line in output.splitlines()
        if endpoint.search(line) and re.search(r"\bLISTEN(?:ING)?\b", line, re.IGNORECASE)
    ]


def host_ur_ports_status(robot_interface_ip: str = "") -> dict[str, Any]:
    """Inspect host listeners without connecting to the robot-facing sockets."""

    def accepted_lines(output: str, port: int) -> list[str]:
        lines = _public_listener_lines(output, port)
        if not robot_interface_ip:
            return lines
        covering = re.compile(
            rf"(?<![\d.])(?:{re.escape(robot_interface_ip)}|0\.0\.0\.0|\*):{port}(?:\s|\b)"
        )
        # Colima forwards VM wildcard publications as IPv4 wildcard listeners
        # on the Mac. Those cover the robot NIC too; localhost / another NIC
        # and IPv6-only wildcard listeners do not prove IPv4 reachability.
        return [line for line in lines if covering.search(line) and "IPv6" not in line]

    missing: list[int] = []
    listeners: dict[str, str] = {}
    system_name = platform.system()
    if system_name == "Darwin":
        for port in UR_REVERSE_PORTS:
            result = run_capture(
                ["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN"],
                timeout=2.0,
            )
            output = result["output"]
            listeners[str(port)] = output
            if not result["ok"] or not accepted_lines(output, port):
                missing.append(port)
    elif system_name == "Windows":
        result = run_capture(["netstat", "-ano", "-p", "tcp"], timeout=4.0)
        for port in UR_REVERSE_PORTS:
            lines = accepted_lines(result["output"], port) if result["ok"] else []
            listeners[str(port)] = "\n".join(lines) or result["output"]
            if not lines:
                missing.append(port)
    elif system_name == "Linux":
        result = run_capture(["ss", "-ltn"], timeout=4.0)
        for port in UR_REVERSE_PORTS:
            lines = accepted_lines(result["output"], port) if result["ok"] else []
            listeners[str(port)] = "\n".join(lines) or result["output"]
            if not lines:
                missing.append(port)
    else:
        missing.extend(UR_REVERSE_PORTS)
        for port in UR_REVERSE_PORTS:
            listeners[str(port)] = f"Unsupported host operating system: {system_name}"
    return {"ok": not missing, "missing": missing, "listeners": listeners}


def moveit_process_running() -> bool:
    result = run_capture(
        [
            "docker",
            "compose",
            "-f",
            str(COMPOSE_FILE),
            "exec",
            "-T",
            "-u",
            "ubuntu",
            "ros2",
            "bash",
            "-lc",
            "pgrep -x move_group",
        ],
        timeout=4.0,
    )
    return result["ok"] and bool(result["output"].strip())


def live_tool0_pose() -> dict[str, Any]:
    """Read the current base_link -> tool0 transform without commanding motion."""

    result = run_capture(
        [
            "docker",
            "compose",
            "-f",
            str(COMPOSE_FILE),
            "exec",
            "-T",
            "-u",
            "ubuntu",
            "ros2",
            "bash",
            "-lc",
            (
                "source /opt/ros/humble/setup.bash; "
                "source /home/ubuntu/ur_ros2_ws/install/setup.bash; "
                "timeout 6 ros2 run tf2_ros tf2_echo base_link tool0"
            ),
        ],
        timeout=8.0,
    )
    output = result["output"]
    translation = re.search(
        r"- Translation:\s*\[\s*([^,\]]+),\s*([^,\]]+),\s*([^\]]+)\]",
        output,
    )
    rpy = re.search(
        r"- Rotation: in RPY \(degree\)\s*"
        r"\[\s*([^,\]]+),\s*([^,\]]+),\s*([^\]]+)\]",
        output,
    )
    if translation is None or rpy is None:
        raise ApiError(
            HTTPStatus.SERVICE_UNAVAILABLE,
            "The current tool0 pose is unavailable. Start the UR driver and "
            "wait for joint states before capturing calibration.",
        )
    try:
        position = {
            axis: round(float(translation.group(index)), 6)
            for index, axis in enumerate(("x", "y", "z"), start=1)
        }
        orientation = {
            axis: round(float(rpy.group(index)), 4)
            for index, axis in enumerate(("roll", "pitch", "yaw"), start=1)
        }
    except ValueError as exc:
        raise ApiError(
            HTTPStatus.SERVICE_UNAVAILABLE,
            "The current tool0 transform could not be interpreted.",
        ) from exc
    return {
        "frame_id": "base_link",
        "child_frame_id": "tool0",
        "position_m": position,
        "orientation_rpy_deg": orientation,
        "read_only": True,
        "execution_locked": True,
    }


def driver_process_running() -> bool:
    result = run_capture(
        [
            "docker",
            "compose",
            "-f",
            str(COMPOSE_FILE),
            "exec",
            "-T",
            "-u",
            "ubuntu",
            "ros2",
            "bash",
            "-lc",
            "pgrep -f '[u]r_ros2_control_node'",
        ],
        timeout=4.0,
    )
    return result["ok"] and bool(result["output"].strip())


def live_external_control_status(timeout: float = 2.5) -> dict[str, Any]:
    """Read current UR program and joint-state readiness without ros2cli daemon state."""
    container_probe = f"/home/ubuntu/ur_ros2_ws/scripts/{EXTERNAL_CONTROL_PROBE.name}"
    command = [
        "docker",
        "compose",
        "-f",
        str(COMPOSE_FILE),
        "exec",
        "-T",
        "-u",
        "ubuntu",
        "ros2",
        "bash",
        "-lc",
        (
            "source /opt/ros/humble/setup.bash; "
            "if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then "
            "source /home/ubuntu/ur_ros2_ws/install/setup.bash; fi; "
            f"python3 {container_probe} --timeout {timeout:g} "
            "--stable-seconds 0.25 --check-only --json"
        ),
    ]
    # Docker exec and ROS discovery add startup time outside the probe's own
    # deadline, especially on the first status refresh after opening the app.
    result = run_capture(command, timeout=timeout + 6.0)
    for line in reversed(result["output"].splitlines()):
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict) and "ready" in payload:
            return {
                "available": payload.get("program_state_received") is True,
                "ready": payload.get("ready") is True,
                "program_running": payload.get("program_running"),
                "joint_state_fresh": payload.get("joint_state_fresh") is True,
                "reason": str(payload.get("reason", "Live External Control state is unavailable.")),
            }
    return {
        "available": False,
        "ready": False,
        "program_running": None,
        "joint_state_fresh": False,
        "reason": "Live External Control state is unavailable.",
    }


def live_klipper_bridge_status(stepper: str, timeout: float = 2.5) -> dict[str, Any]:
    """Read the bridge-owned ROS status without relying on ros2cli daemon state."""
    container_probe = f"/home/ubuntu/ur_ros2_ws/scripts/{KLIPPER_BRIDGE_PROBE.name}"
    command = [
        "docker",
        "compose",
        "-f",
        str(COMPOSE_FILE),
        "exec",
        "-T",
        "-u",
        "ubuntu",
        "ros2",
        "bash",
        "-lc",
        (
            "source /opt/ros/humble/setup.bash; "
            "if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then "
            "source /home/ubuntu/ur_ros2_ws/install/setup.bash; fi; "
            f"python3 {container_probe} --timeout {timeout:g} "
            f"--stepper {stepper} --json"
        ),
    ]
    result = run_capture(command, timeout=timeout + 6.0)
    for line in reversed(result["output"].splitlines()):
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict) and "ready" in payload:
            return {
                "available": payload.get("available") is True,
                "ready": payload.get("ready") is True,
                "moonraker_reachable": payload.get("moonraker_reachable") is True,
                "printer_ready": payload.get("printer_ready") is True,
                "stepper_found": payload.get("stepper_found") is True,
                "extruder_found": payload.get("extruder_found") is True,
                "extrusion_protocol": str(payload.get("extrusion_protocol", "")),
                "extruders": list(payload.get("extruders", [])),
                "extruder_can_extrude": payload.get("extruder_can_extrude") is True,
                "extruder_temperature": payload.get("extruder_temperature"),
                "extruder_target": payload.get("extruder_target"),
                "action_active": payload.get("action_active") is True,
                "reason": str(payload.get("reason", "ROS Klipper bridge status is unavailable.")),
            }
    return {
        "available": False,
        "ready": False,
        "moonraker_reachable": False,
        "printer_ready": False,
        "stepper_found": False,
        "action_active": False,
        "reason": "ROS Klipper bridge status is unavailable.",
    }


def check_item(
    item_id: str,
    label: str,
    ok: bool,
    detail: str,
    *,
    severity: str | None = None,
) -> dict[str, Any]:
    return {
        "id": item_id,
        "label": label,
        "ok": ok,
        "severity": severity or ("pass" if ok else "fail"),
        "detail": detail,
    }


def validate_ip(value: str, *, allow_blank: bool = False) -> str:
    value = value.strip()
    if allow_blank and not value:
        return ""
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ApiError(HTTPStatus.BAD_REQUEST, f"Invalid IP address: {value}") from exc
    if parsed.version != 4:
        raise ApiError(HTTPStatus.BAD_REQUEST, "Only IPv4 addresses are supported.")
    return value


def validate_config(raw: dict[str, Any]) -> dict[str, Any]:
    merged = {**DEFAULT_CONFIG, **raw}
    result: dict[str, Any] = {}
    # Normalize saved profiles from multilingual releases to the English UI.
    result["language"] = "en"
    result["ur_robot_ip"] = validate_ip(str(merged["ur_robot_ip"]))
    result["ros_pc_ip"] = validate_ip(str(merged.get("ros_pc_ip", "")), allow_blank=True)

    url = urllib.parse.urlparse(str(merged["klipper_url"]).strip())
    if url.scheme not in {"http", "https"} or not url.hostname or url.username or url.password:
        raise ApiError(
            HTTPStatus.BAD_REQUEST,
            "Klipper URL must be an http(s) URL without embedded credentials.",
        )
    result["klipper_url"] = urllib.parse.urlunparse(url).rstrip("/")

    stepper = str(merged["manual_stepper"]).strip()
    legacy_extruder = stepper == "stepper_x"
    if legacy_extruder:
        stepper = "extruder"
    if not re.fullmatch(r"extruder(?:[1-9][0-9]*)?", stepper):
        raise ApiError(
            HTTPStatus.BAD_REQUEST,
            "Select a native Klipper extruder (extruder, extruder1, ...). "
            "Manual steppers such as lock_motor cannot be used for extrusion.",
        )
    result["manual_stepper"] = stepper

    axes = ",".join(part.strip().lower() for part in str(merged["axes"]).split(",") if part.strip())
    if not axes or any(part not in {"x", "y", "z"} for part in axes.split(",")):
        raise ApiError(HTTPStatus.BAD_REQUEST, "Axes must contain only x, y, and z.")
    result["axes"] = axes

    limits = {
        "ur_update_rate": (int, 50, 500),
        "distance_m": (float, 0.001, 0.10),
        "max_joint_speed_rad_s": (float, 0.005, 0.05),
        "stepper_move": (float, -200.0, 200.0),
        "stepper_speed": (float, 0.1, 5.0),
        "stepper_accel": (float, 20.0, 20.0),
    }
    for key, (caster, minimum, maximum) in limits.items():
        try:
            value = caster(merged[key])
        except (TypeError, ValueError) as exc:
            raise ApiError(HTTPStatus.BAD_REQUEST, f"Invalid value for {key}.") from exc
        if legacy_extruder and key in {"stepper_speed", "stepper_accel"}:
            if key == "stepper_accel" and math.isfinite(value) and value > 0.0:
                # Native E acceleration comes from printer.cfg; the old manual
                # stepper's per-command acceleration cannot be carried over.
                value = maximum
            else:
                value = min(value, maximum)
        if not math.isfinite(value) or value < minimum or value > maximum:
            raise ApiError(
                HTTPStatus.BAD_REQUEST,
                f"{key} must be between {minimum} and {maximum}.",
            )
        result[key] = value
    return result


def read_config(path: Path | None = None) -> dict[str, Any]:
    path = CONFIG_PATH if path is None else path
    if not path.exists():
        return validate_config(DEFAULT_CONFIG)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Could not read {path}: {exc}") from exc
    return validate_config(payload)


def write_config(config: dict[str, Any], path: Path | None = None) -> None:
    path = CONFIG_PATH if path is None else path
    validated = validate_config(config)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(validated, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def tcp_probe(host: str, port: int, timeout: float = 0.9) -> tuple[bool, str]:
    started = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            elapsed = (time.monotonic() - started) * 1000
            return True, f"Port {port} responded in {elapsed:.0f} ms."
    except OSError as exc:
        return False, f"Port {port} did not respond: {exc}"


def dashboard_query(host: str, timeout: float = 1.5) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, 29999), timeout=timeout) as connection:
            connection.settimeout(timeout)
            greeting = connection.recv(512).decode("utf-8", errors="replace").strip()
            connection.sendall(b"robotmode\n")
            response = connection.recv(512).decode("utf-8", errors="replace").strip()
            detail = " · ".join(part for part in (greeting, response) if part)
            return True, detail or "Dashboard server responded."
    except OSError as exc:
        return False, f"Dashboard server did not respond: {exc}"


def dashboard_stop_and_verify(host: str, timeout: float = 5.0) -> dict[str, Any]:
    """Issue the UR Dashboard stop command, then require live STOPPED state.

    This command can only remove motion.  It is intentionally independent of
    ROS so a broken action connection cannot prevent recovery containment.
    """

    deadline = time.monotonic() + timeout
    stop_response = ""
    last_state = "unavailable"
    try:
        with socket.create_connection((host, 29999), timeout=min(1.5, timeout)) as connection:
            connection.settimeout(min(1.5, timeout))
            connection.recv(512)
            connection.sendall(b"stop\n")
            stop_response = connection.recv(512).decode("utf-8", errors="replace").strip()
    except OSError as exc:
        raise RuntimeError(f"Dashboard stop command failed: {exc}") from exc

    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, 29999), timeout=1.0) as connection:
                connection.settimeout(1.0)
                connection.recv(512)
                connection.sendall(b"programState\n")
                response = connection.recv(512).decode("utf-8", errors="replace").strip()
                last_state = response or "unknown"
                if re.search(r"\bSTOPPED\b", response, re.IGNORECASE):
                    return {
                        "stopped": True,
                        "program_state": "STOPPED",
                        "stop_response": stop_response,
                        "response": response,
                        "observed_at": utc_now(),
                    }
        except OSError as exc:
            last_state = str(exc)
        time.sleep(0.2)
    return {
        "stopped": False,
        "program_state": last_state,
        "stop_response": stop_response,
        "observed_at": utc_now(),
    }


def ping_probe(host: str) -> tuple[bool, str]:
    system_name = platform.system()
    if system_name == "Darwin":
        command = ["ping", "-c", "1", "-W", "1000", host]
    elif system_name == "Windows":
        command = ["ping", "-n", "1", "-w", "1000", host]
    else:
        command = ["ping", "-c", "1", "-W", "1", host]
    result = run_capture(command, timeout=3.0)
    if result["ok"]:
        match = re.search(
            r"time\s*[=<]\s*([0-9.]+)\s*ms",
            result["output"],
            re.IGNORECASE,
        )
        detail = f"Reachable in {match.group(1)} ms." if match else "Robot is reachable."
        return True, detail
    return False, "No ping response. Check power, Ethernet, and the IP address."


def detect_route(host: str) -> dict[str, str]:
    system_name = platform.system()
    if system_name == "Darwin":
        route = run_capture(["route", "-n", "get", host], timeout=3.0)
        interface_match = re.search(r"^\s*interface:\s*(\S+)", route["output"], re.MULTILINE)
        interface = interface_match.group(1) if interface_match else ""
        address = ""
        if interface:
            address = run_capture(["ipconfig", "getifaddr", interface], timeout=2.0)["output"].strip()
        service = ""
        services = run_capture(["networksetup", "-listnetworkserviceorder"], timeout=3.0)["output"]
        current_service = ""
        for line in services.splitlines():
            service_match = re.match(r"^\(\d+\)\s+(.+)$", line.strip())
            if service_match:
                current_service = service_match.group(1)
            if interface and f"Device: {interface})" in line:
                service = current_service
                break
        return {"interface": interface, "address": address, "service": service}

    if system_name == "Windows":
        # Find-NetRoute selects the same route Windows will use for the robot.
        # Convert the result to one compact JSON object so localized table
        # headings never affect parsing.
        powershell = (
            "$ErrorActionPreference='Stop';"
            f"$items=@(Find-NetRoute -RemoteIPAddress '{host}');"
            "$source=$items|Where-Object {"
            "$_.PSObject.Properties.Name -contains 'IPAddress'"
            "}|Select-Object -First 1;"
            "$route=$items|Where-Object {"
            "$_.PSObject.Properties.Name -contains 'DestinationPrefix'"
            "}|Select-Object -First 1;"
            "if($null -eq $source -or $null -eq $route){"
            "throw 'Windows did not return both a source address and route.'"
            "};"
            "$index=[int]$route.InterfaceIndex;"
            "$adapter=Get-NetAdapter -InterfaceIndex $index -ErrorAction Stop;"
            "$wireless=([string]$adapter.MediaType -match '802\\.11|wireless|wi-fi') "
            "-or ([string]$adapter.PhysicalMediaType -match '802\\.11|wireless|wi-fi');"
            "$virtual=([bool]$adapter.Virtual) "
            "-or ([string]$adapter.InterfaceDescription -match 'virtual|hyper-v|vpn');"
            "$kind=if($wireless){'Wi-Fi'}elseif($virtual){'Virtual'}else{'Ethernet'};"
            "[pscustomobject]@{interface=[string]$adapter.Name;"
            "address=[string]$source.IPAddress;service=$kind}"
            "|ConvertTo-Json -Compress"
        )
        result = run_capture(
            ["powershell.exe", "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", powershell],
            timeout=5.0,
        )
        if result["ok"]:
            for line in reversed(result["output"].splitlines()):
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict):
                    return {
                        "interface": str(value.get("interface", "")),
                        "address": str(value.get("address", "")),
                        "service": str(value.get("service", "")),
                    }
        return {"interface": "", "address": "", "service": ""}

    route = run_capture(["ip", "route", "get", host], timeout=3.0)["output"]
    interface_match = re.search(r"\bdev\s+(\S+)", route)
    source_match = re.search(r"\bsrc\s+(\S+)", route)
    return {
        "interface": interface_match.group(1) if interface_match else "",
        "address": source_match.group(1) if source_match else "",
        "service": "Ethernet" if interface_match else "",
    }


def urlopen_json(url: str, timeout: float = 5.0) -> dict[str, Any]:
    request = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def klipper_stopped_status(base_url: str) -> dict[str, Any]:
    """Return fresh Moonraker print state without issuing a motion command."""

    payload = urlopen_json(
        base_url.rstrip("/") + "/printer/objects/query?print_stats",
        timeout=4.0,
    )
    print_stats = (
        payload.get("result", {})
        .get("status", {})
        .get("print_stats", {})
    )
    state = str(print_stats.get("state", "unknown")).casefold()
    stopped = state in {"standby", "complete", "cancelled", "canceled", "error"}
    return {
        "stopped": stopped,
        "print_state": state,
        "filename": str(print_stats.get("filename", "")),
        "observed_at": utc_now(),
    }


def parse_driver_log(text: str) -> dict[str, Any]:
    connected = text.rfind("Robot connected to reverse interface")
    dropped = text.rfind("Connection to reverse interface dropped")
    intentional_stop = text.rfind("UR driver intentionally stopped")
    system_started_at = text.rfind("System successfully started!")
    system_started = system_started_at >= 0 and system_started_at > intentional_stop
    external_control = connected >= 0 and connected > max(dropped, intentional_stop)
    rtde_mask_at = text.rfind("speed_slider_mask")
    rtde_conflict_at = text.rfind("controlled by another RTDE client")
    rtde_conflict = min(rtde_mask_at, rtde_conflict_at) > max(intentional_stop, system_started_at)
    unstable_terms = (
        "Unable to read from hardware",
        "Reconnecting to the RTDE interface",
        "process has died",
        "Pipeline producer overflowed",
        "Failed to initialize RTDE client",
    )
    last_unstable = max((text.rfind(term) for term in unstable_terms), default=-1)
    unstable = last_unstable >= 0 and last_unstable > max(connected, intentional_stop, system_started_at)
    return {
        "system_started": system_started,
        "external_control": external_control,
        "rtde_conflict": rtde_conflict,
        "unstable": unstable,
    }


@dataclass
class Job:
    id: str
    kind: str
    motion: bool
    command: list[str]
    created_at: str = field(default_factory=utc_now)
    started_at: str | None = None
    finished_at: str | None = None
    state: str = "queued"
    return_code: int | None = None
    log_lines: list[str] = field(default_factory=list)
    process: subprocess.Popen[str] | None = None
    log_path: Path | None = None
    cancel_requested: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )
    cancel_complete: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )
    process_start_lock: threading.Lock = field(
        default_factory=threading.Lock,
        repr=False,
    )

    def public(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "motion": self.motion,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "state": self.state,
            "return_code": self.return_code,
            "log": "\n".join(self.log_lines[-500:]),
        }


def _process_group_options() -> dict[str, Any]:
    if platform.system() == "Windows":
        return {
            "creationflags": getattr(
                subprocess,
                "CREATE_NEW_PROCESS_GROUP",
                0x00000200,
            )
        }
    return {"start_new_session": True}


def _terminate_host_process_tree(process: subprocess.Popen[str]) -> None:
    if platform.system() == "Windows":
        result = run_capture(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            timeout=8.0,
        )
        if result["ok"] or process.poll() is not None:
            return
        # The direct child is still terminated as a final containment measure.
        # The ROS-side coordinated cancel has already succeeded before this
        # helper is called for a motion job.
        process.terminate()
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except OSError:
        process.terminate()


def _parse_project_executor_processes(
    output: str,
    script_path: Path = PRINT_EXECUTION_HOST,
) -> list[tuple[int, str]]:
    """Parse ps output and accept only this project's exact executor path."""

    path_pattern = re.compile(
        rf"(?<!\S)[\"']?{re.escape(str(script_path.resolve()))}[\"']?(?=\s|$)"
    )
    processes: list[tuple[int, str]] = []
    for line in output.splitlines():
        match = re.match(r"^\s*(\d+)\s+\d+\s+(.+?)\s*$", line)
        if match is None or path_pattern.search(match.group(2)) is None:
            continue
        pid = int(match.group(1))
        if pid != os.getpid():
            processes.append((pid, match.group(2)))
    return processes


def project_executor_processes() -> list[tuple[int, str]]:
    if platform.system() == "Windows":
        # Current Windows jobs are owned and terminated by JobManager.  The
        # project Docker service remains the authoritative runtime boundary.
        return []
    result = run_capture(
        ["ps", "-axo", "pid=,ppid=,command="],
        timeout=5.0,
    )
    return (
        _parse_project_executor_processes(result["output"])
        if result["ok"]
        else []
    )


def stop_project_executor_processes() -> dict[str, Any]:
    """Contain legacy executor wrappers from an older workstation session."""

    if platform.system() == "Windows":
        return {"stopped_pids": [], "remaining_pids": []}
    stopped: list[int] = []
    for pid, _command in project_executor_processes():
        # Re-read the exact PID immediately before signalling to avoid acting
        # on a reused process id.
        identity = run_capture(
            ["ps", "-p", str(pid), "-o", "pid=,ppid=,command="],
            timeout=3.0,
        )
        if not identity["ok"] or not _parse_project_executor_processes(identity["output"]):
            continue
        try:
            os.killpg(pid, signal.SIGTERM)
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                continue
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not any(item[0] == pid for item in project_executor_processes()):
                break
            time.sleep(0.1)
        if any(item[0] == pid for item in project_executor_processes()):
            try:
                os.killpg(pid, signal.SIGKILL)
            except OSError:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        stopped.append(pid)
    remaining = [pid for pid, _command in project_executor_processes()]
    return {"stopped_pids": stopped, "remaining_pids": remaining}


class JobManager:
    def __init__(
        self,
        *,
        job_log_dir: Path | None = None,
        cancel_dir: Path | None = None,
    ) -> None:
        self._jobs: dict[str, Job] = {}
        self._lock = threading.RLock()
        self._configured_job_log_dir = job_log_dir
        self._configured_cancel_dir = cancel_dir
        self._closed = False
        self._maintenance: dict[str, str] = {}

    @property
    def job_log_dir(self) -> Path:
        return self._configured_job_log_dir or JOB_LOG_DIR

    @property
    def cancel_dir(self) -> Path:
        return self._configured_cancel_dir or CANCEL_DIR

    def _append(self, job: Job, line: str) -> None:
        clean = line.rstrip("\n")
        with self._lock:
            job.log_lines.append(clean)
            if len(job.log_lines) > 1200:
                del job.log_lines[:200]
            if job.log_path:
                encoded_size = len((clean + "\n").encode("utf-8"))
                if (
                    job.log_path.is_file()
                    and job.log_path.stat().st_size + encoded_size
                    > JOB_LOG_MAX_BYTES
                ):
                    backup = job.log_path.with_suffix(job.log_path.suffix + ".1")
                    backup.unlink(missing_ok=True)
                    job.log_path.replace(backup)
                with job.log_path.open("a", encoding="utf-8") as handle:
                    handle.write(clean + "\n")

    def prune_logs(self, *, keep: int = JOB_LOG_KEEP_COUNT) -> None:
        if keep < 0:
            raise ValueError("keep must not be negative")
        if not self.job_log_dir.is_dir():
            return
        logs = sorted(
            self.job_log_dir.glob("*.log*"),
            key=lambda path: (path.stat().st_mtime_ns, path.name),
            reverse=True,
        )
        for path in logs[keep:]:
            path.unlink(missing_ok=True)

    def start(
        self,
        kind: str,
        command: list[str],
        *,
        env: dict[str, str] | None = None,
        motion: bool = False,
        demo_lines: list[str] | None = None,
    ) -> Job:
        with self._lock:
            if self._closed:
                raise ApiError(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    "The workstation is shutting down and is not accepting new operations.",
                    code="workstation_shutting_down",
                )
            if self._maintenance:
                raise ApiError(
                    HTTPStatus.CONFLICT,
                    "The workstation is completing a stop recovery or shutdown.",
                    code="workstation_maintenance",
                )
            if any(job.state in {"queued", "running"} for job in self._jobs.values()):
                raise ApiError(HTTPStatus.CONFLICT, "Another workstation operation is already running.")
            if motion and any(job.motion and job.state == "running" for job in self._jobs.values()):
                raise ApiError(HTTPStatus.CONFLICT, "A real-motion operation is already running.")
            job = Job(id=uuid.uuid4().hex[:12], kind=kind, motion=motion, command=command)
            self.job_log_dir.mkdir(parents=True, exist_ok=True)
            self.prune_logs()
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            job.log_path = self.job_log_dir / f"{stamp}-{kind}-{job.id}.log"
            self._jobs[job.id] = job
            self.cancel_dir.mkdir(parents=True, exist_ok=True)
            cancel_file = self.cancel_dir / f"{job.id}.cancel"
            cancel_file.unlink(missing_ok=True)

        effective_env = os.environ.copy() if env is None else dict(env)
        effective_env["WORKSTATION_JOB_ID"] = job.id
        effective_env["WORKSTATION_CANCEL_FILE"] = str(cancel_file)
        effective_env = _wsl_environment(effective_env, command)

        thread = threading.Thread(
            target=self._run,
            args=(job, effective_env, demo_lines),
            name=f"station-job-{job.id}",
            daemon=True,
        )
        thread.start()
        return job

    def _run(self, job: Job, env: dict[str, str] | None, demo_lines: list[str] | None) -> None:
        with self._lock:
            if job.state == "canceled":
                job.finished_at = utc_now()
                canceled_before_start = True
            else:
                canceled_before_start = False
                job.state = "running"
                job.started_at = utc_now()
        if canceled_before_start:
            (self.cancel_dir / f"{job.id}.cancel").unlink(missing_ok=True)
            return
        try:
            with self._lock:
                if job.state == "canceled":
                    return
            if demo_lines is not None:
                for line in demo_lines:
                    if job.state == "canceled":
                        return
                    self._append(job, line)
                    time.sleep(0.14)
                return_code = 0
            else:
                with job.process_start_lock:
                    if job.cancel_requested.is_set():
                        return
                    process = subprocess.Popen(
                        job.command,
                        cwd=PROJECT_ROOT,
                        env=env,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1,
                        **_process_group_options(),
                    )
                    with self._lock:
                        job.process = process
                assert process.stdout is not None
                try:
                    for line in process.stdout:
                        self._append(job, line)
                finally:
                    close_output = getattr(process.stdout, "close", None)
                    if close_output is not None:
                        close_output()
                return_code = process.wait()
                if job.cancel_requested.is_set():
                    # Drain the child first so its cleanup cannot block on a
                    # full pipe, then let the cancel caller publish the result.
                    job.cancel_complete.wait(timeout=30.0)
            with self._lock:
                if job.state != "canceled":
                    job.return_code = return_code
                    job.state = "succeeded" if return_code == 0 else "failed"
        except Exception as exc:  # keep worker failures visible to the operator
            self._append(job, f"ERROR: {exc}")
            with self._lock:
                if job.state != "canceled":
                    job.return_code = 1
                    job.state = "failed"
        finally:
            with self._lock:
                job.finished_at = utc_now()
                job.process = None
            (self.cancel_dir / f"{job.id}.cancel").unlink(missing_ok=True)

    def get(self, job_id: str) -> Job:
        with self._lock:
            job = self._jobs.get(job_id)
            if not job:
                raise ApiError(HTTPStatus.NOT_FOUND, "Job not found.")
            return job

    def begin_maintenance(self, reason: str) -> str:
        token = uuid.uuid4().hex
        with self._lock:
            self._maintenance[token] = reason
        return token

    def end_maintenance(self, token: str) -> None:
        with self._lock:
            self._maintenance.pop(token, None)

    def latest(self) -> Job | None:
        with self._lock:
            return next(reversed(self._jobs.values()), None) if self._jobs else None

    def active(self) -> Job | None:
        with self._lock:
            return next((job for job in self._jobs.values() if job.state in {"queued", "running"}), None)

    def cancel(self, job_id: str) -> Job:
        job = self.get(job_id)
        job.cancel_complete.clear()
        job.cancel_requested.set()
        cancel_file = self.cancel_dir / f"{job.id}.cancel"
        self.cancel_dir.mkdir(parents=True, exist_ok=True)
        cancel_file.write_text("cancel\n", encoding="utf-8")
        maintenance_token = self.begin_maintenance("operation-cancellation")
        try:
            with job.process_start_lock:
                with self._lock:
                    if job.state not in {"queued", "running"}:
                        return job
                    process = job.process
                    current_state = job.state

                if (
                    job.motion
                    and current_state == "running"
                    and process
                    and process.poll() is None
                ):
                    container_client = (
                        "/home/ubuntu/ur_ros2_ws/scripts/"
                        f"{SYNCHRONIZED_CANCEL_CLIENT.name}"
                    )
                    result = run_capture(
                        [
                            "docker",
                            "compose",
                            "-f",
                            str(COMPOSE_FILE),
                            "exec",
                            "-T",
                            "-u",
                            "ubuntu",
                            "ros2",
                            "bash",
                            "-lc",
                            (
                                "source /opt/ros/humble/setup.bash; "
                                "if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then "
                                "source /home/ubuntu/ur_ros2_ws/install/setup.bash; fi; "
                                f"python3 {container_client} --timeout 20 --json"
                            ),
                        ],
                        timeout=28.0,
                    )
                    if result["output"]:
                        self._append(job, result["output"])
                    if not result["ok"]:
                        raise ApiError(
                            HTTPStatus.CONFLICT,
                            "ROS could not confirm a safe coordinated cancellation. "
                            "Use the hardware emergency stop; the job was not marked canceled.",
                        )

                with self._lock:
                    if job.state not in {"queued", "running"}:
                        return job
                if process and process.poll() is None:
                    cooperative_windows_startup = (
                        platform.system() == "Windows"
                        and len(job.command) >= 3
                        and Path(job.command[1]).name == "windows_runtime.py"
                        and job.command[2] == "start-ros"
                    )
                    # The Windows launcher observes WORKSTATION_CANCEL_FILE
                    # and verifies its detached control stack was stopped.
                    # taskkill /F would bypass that cleanup entirely.
                    if not cooperative_windows_startup:
                        _terminate_host_process_tree(process)
                    # The ROS launcher stops its detached driver in an EXIT
                    # trap. Let that cleanup finish before a replacement
                    # launch can start, or the old trap can kill the new one.
                    # The worker keeps draining stdout during this wait.
                    try:
                        process.wait(timeout=30.0)
                    except subprocess.TimeoutExpired as exc:
                        raise ApiError(
                            HTTPStatus.CONFLICT,
                            "The operation is still completing its stop cleanup. "
                            "Wait for it to finish before starting ROS again.",
                            code="operation_cleanup_incomplete",
                        ) from exc
                with self._lock:
                    job.state = "canceled"
                self._append(
                    job,
                    "Operation canceled by the operator. Hardware emergency stop remains authoritative.",
                )
                return job
        except Exception:
            job.cancel_requested.clear()
            cancel_file.unlink(missing_ok=True)
            raise
        finally:
            job.cancel_complete.set()
            self.end_maintenance(maintenance_token)

    def stop_all(self, *, reason: str, close: bool = False) -> dict[str, Any]:
        """Contain every process group created by this manager.

        Recovery uses this only after a durable stop-unconfirmed record exists;
        normal app shutdown also stops the complete project ROS service next.
        It therefore must not depend on the ordinary active-job gate.
        """

        with self._lock:
            if close:
                self._closed = True
            active = [
                job for job in self._jobs.values()
                if job.state in {"queued", "running"}
            ]

        stopped: list[str] = []
        for job in active:
            job.cancel_requested.set()
            self.cancel_dir.mkdir(parents=True, exist_ok=True)
            (self.cancel_dir / f"{job.id}.cancel").write_text(
                "lifecycle-stop\n",
                encoding="utf-8",
            )
            with job.process_start_lock:
                with self._lock:
                    process = job.process
                if process is not None and process.poll() is None:
                    _terminate_host_process_tree(process)
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3.0)
                with self._lock:
                    if job.state in {"queued", "running"}:
                        job.state = "canceled"
                        job.return_code = None
                        job.finished_at = utc_now()
                job.cancel_complete.set()
            self._append(
                job,
                f"Operation contained by workstation lifecycle ({reason}). "
                "Any unconfirmed hardware stop record remains locked.",
            )
            stopped.append(job.id)
        return {"stopped_job_ids": stopped, "closed": self._closed}


class StationBackend:
    def __init__(
        self,
        *,
        demo: bool = False,
        calibration_path: Path | None = None,
        data_dir: Path | None = None,
        path_review_runtime: Path | None = None,
        print_execution_runtime: Path | None = None,
    ) -> None:
        self.demo = demo
        self._data_dir_finalizer: weakref.finalize | None = None
        if demo and data_dir is None:
            data_dir = Path(
                tempfile.mkdtemp(prefix="ur-print-workstation-demo-")
            )
            self._data_dir_finalizer = weakref.finalize(
                self,
                shutil.rmtree,
                data_dir,
                True,
            )
        self.data_dir = (data_dir or DATA_DIR).resolve()
        runtime_dir = self.data_dir / "runtime"
        self.jobs = JobManager(
            job_log_dir=runtime_dir / "jobs",
            cancel_dir=runtime_dir / "cancel",
        )
        self._config_lock = threading.RLock()
        self._config_path = self.data_dir / "config.json"
        self._mode_lock = threading.RLock()
        self._calibration_lock = threading.RLock()
        self._height_lock = threading.RLock()
        self._height_draft_m: float | None = None
        self._height_trial: dict[str, Any] | None = None
        self._print_z_draft_m: float | None = None
        self._print_z_trial: dict[str, Any] | None = None
        self._print_z_placement_path = self.data_dir / "print_z_placement.json"
        # Written before the first verified placement. A missing/corrupt record
        # must never silently restore the old (possibly lower) print datum.
        self._print_z_required_path = self.data_dir / "print_z_placement.required"
        self._path_review_lock = threading.RLock()
        self._gcode_review_lock = threading.RLock()
        self._gcode_review_cpu = threading.BoundedSemaphore(value=1)
        self._print_execution_lock = threading.RLock()
        self._print_preparation_lock = threading.RLock()
        self._calibration_path = calibration_path or (
            self.data_dir / "gcode_calibration.json"
        )
        self._path_review_runtime = path_review_runtime or (
            runtime_dir / "path_review" if demo else PATH_REVIEW_RUNTIME
        )
        self._print_execution_runtime = print_execution_runtime or (
            runtime_dir / "print_execution" if demo else PRINT_EXECUTION_RUNTIME
        )
        self._demo_calibration: dict[str, Any] | None = None
        self._simulation_calibration_drafts: dict[str, dict[str, Any]] = {}
        self._path_review_jobs: dict[str, dict[str, Any]] = {}
        self._gcode_review_jobs: dict[str, dict[str, Any]] = {}
        self._gcode_review_job_ids_by_key: dict[tuple[str, ...], str] = {}
        self._print_packages: dict[str, dict[str, Any]] = {}
        self._print_jobs: dict[str, dict[str, str]] = {}
        self._print_preparations: dict[str, dict[str, Any]] = {}
        self._print_preparation_ids_by_key: dict[tuple[str, str, str, str], str] = {}
        self._closing = False
        # Every newly opened workstation starts in the safer local-preparation
        # mode. Online is an explicit operator permission, not an inferred
        # consequence of Docker or ROS already running in the background.
        self._operating_mode = "offline"
        self.config = read_config(self._config_path)
        self.lifecycle = RuntimeLifecycle(
            runtime_dir=self._print_execution_runtime,
            stop_owned_jobs=self._lifecycle_stop_jobs,
            stop_owned_runtime=self._lifecycle_stop_runtime,
            owned_runtime_stopped=self._lifecycle_runtime_stopped,
            stop_and_verify_ur=self._lifecycle_stop_and_verify_ur,
            verify_klipper_stopped=self._lifecycle_verify_klipper,
            invalidate_executions=self._invalidate_recovered_executions,
        )

    def get_operating_mode(self) -> dict[str, Any]:
        with self._mode_lock:
            mode = self._operating_mode
        return {
            "mode": mode,
            "online": mode == "online",
            "default": "offline",
            "services_unchanged": True,
            "stop_actions_available": True,
        }

    def set_operating_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = payload.get("mode")
        if mode not in OPERATING_MODES:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                "Operating mode must be either offline or online.",
            )
        with self._mode_lock:
            previous = self._operating_mode
            if mode != previous and self.jobs.active() is not None:
                raise ApiError(
                    HTTPStatus.CONFLICT,
                    "Finish or cancel the active operation before changing mode.",
                )
            self._operating_mode = mode
        return {
            **self.get_operating_mode(),
            "previous_mode": previous,
        }

    def require_online_mode(self, action: str) -> None:
        with self._mode_lock:
            online = self._operating_mode == "online"
        if not online:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                f"Switch to Online mode before {action}.",
            )

    def save_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._config_lock:
            updated = validate_config({**self.config, **payload})
            write_config(updated, self._config_path)
            self.config = updated
            return dict(self.config)

    def get_current_tool0_pose(self) -> dict[str, Any]:
        if self.demo:
            return {
                "frame_id": "base_link",
                "child_frame_id": "tool0",
                "position_m": {"x": -0.36, "y": -0.12, "z": 0.24},
                "orientation_rpy_deg": {
                    "roll": 180.0,
                    "pitch": 0.0,
                    "yaw": 90.0,
                },
                "read_only": True,
                "execution_locked": True,
                "simulation": True,
                "source": "simulated_ros",
                "captured_at": utc_now(),
            }
        return {
            **live_tool0_pose(),
            "source": "live_ros",
            "captured_at": utc_now(),
        }

    def _nozzle_position(
        self, pose: dict[str, Any], *, require_print_orientation: bool = False,
    ) -> dict[str, float]:
        """Transform the CAD nozzle candidate using the live tool0 orientation."""

        try:
            saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
        except CalibrationValidationError as exc:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                f"The installed nozzle calibration is invalid: {exc}",
            ) from exc
        profile = saved.get("calibration") if saved else None
        offset = (
            profile["tool_center_point_m"]["offset"]
            if profile and profile.get("operation_mode") == "production_print"
            else load_printhead_geometry()["tool0_to_nozzle_m"]
        )
        angles = pose["orientation_rpy_deg"]
        roll, pitch, yaw = (math.radians(float(angles[name])) for name in ("roll", "pitch", "yaw"))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
        if require_print_orientation:
            expected = (
                profile["coordinate_transform"]["tool_orientation_rpy_deg"]
                if profile and profile.get("operation_mode") == "production_print"
                else {"roll": 180.0, "pitch": 0.0, "yaw": 0.0}
            )
            er, ep, ey = (math.radians(float(expected[name])) for name in ("roll", "pitch", "yaw"))
            ecr, esr = math.cos(er), math.sin(er)
            ecp, esp = math.cos(ep), math.sin(ep)
            ecy, esy = math.cos(ey), math.sin(ey)
            target_rotation = (
                (ecy * ecp, ecy * esp * esr - esy * ecr, ecy * esp * ecr + esy * esr),
                (esy * ecp, esy * esp * esr + ecy * ecr, esy * esp * ecr - ecy * esr),
                (-esp, ecp * esr, ecp * ecr),
            )
            cosine = (sum(rotation[row][column] * target_rotation[row][column]
                          for row in range(3) for column in range(3)) - 1.0) / 2.0
            error_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
            if not math.isfinite(error_deg) or error_deg > 0.5:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Set tool0 to the exact reviewed print orientation before the height trial; "
                    "a merely downward-pointing nozzle is insufficient.",
                    code="print_orientation_required",
                )
        x, y, z = (float(offset[name]) for name in ("x", "y", "z"))
        # Rz(yaw) * Ry(pitch) * Rx(roll), matching the G-code TCP transform.
        rx = cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z
        ry = sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z
        rz = -sp * x + cp * sr * y + cp * cr * z
        # Tool0 +Z is the physical nozzle axis in the installed CAD model.
        nozzle_axis_z = cp * cr
        if nozzle_axis_z > -0.985:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Point the installed nozzle vertically downward before the table-height trial.",
            )
        position = pose["position_m"]
        return {axis: round(float(position[axis]) + component, 6)
                for axis, component in zip(("x", "y", "z"), (rx, ry, rz))}

    def get_workcell_height(self) -> dict[str, Any]:
        with self._height_lock:
            geometry = load_workcell_geometry()
            try:
                saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
            except CalibrationValidationError:
                saved = None
            installed_profile = saved.get("calibration") if saved else None
            installed_print_z_m = self._saved_print_z_m(
                float(geometry["tabletop"]["surface_z_m"]), saved,
            )
            print_z_reverification_required = self._print_z_verification_fault(installed_print_z_m)
            print_z_placement_mode = self._print_z_placement_mode()
            old_tool0_profile = bool(
                installed_profile and installed_profile.get("operation_mode") == "tool0_motion_test"
            )
            trial = copy.deepcopy(self._height_trial)
            if trial:
                for phase in ("plan_job_id", "execute_job_id"):
                    if trial.get(phase):
                        trial[phase.replace("_id", "_state")] = self.jobs.get(trial[phase]).state
            print_z_trial = copy.deepcopy(self._print_z_trial)
            if print_z_trial:
                for phase in ("plan_job_id", "execute_job_id"):
                    if print_z_trial.get(phase):
                        print_z_trial[phase.replace("_id", "_state")] = self.jobs.get(print_z_trial[phase]).state
            return {
                "surface_z_m": geometry["tabletop"]["surface_z_m"],
                "draft_surface_z_m": self._height_draft_m,
                "planning_surface_z_m": (
                    self._height_draft_m if self._height_draft_m is not None
                    else geometry["tabletop"]["surface_z_m"]
                ),
                "print_zero_z_m": (
                    self._print_z_draft_m if self._print_z_draft_m is not None
                    else installed_print_z_m if installed_print_z_m is not None
                    else None if print_z_reverification_required
                    else self._height_draft_m if self._height_draft_m is not None
                    else geometry["tabletop"]["surface_z_m"]
                ),
                "print_z_placement_mode": print_z_placement_mode,
                "print_z_draft_m": self._print_z_draft_m,
                "print_z_candidate": self._print_z_draft_m is not None or print_z_reverification_required,
                "print_z_reverification_required": print_z_reverification_required,
                "print_z_installed_m": installed_print_z_m,
                "print_z_verified": (
                    installed_print_z_m is not None and print_z_placement_mode == "legacy_measured"
                ),
                "print_z_trial": print_z_trial,
                "trial_clearance_m": HEIGHT_TRIAL_CLEARANCE_M,
                "trial": trial,
                "nozzle": load_printhead_geometry(),
                "printhead_geometry_sha256": printhead_geometry_sha256(),
                "printhead_recalibration_required": bool(saved and saved.get("printhead_recalibration_required")),
                "printhead_migration": saved.get("printhead_migration") if saved else None,
                "klipper_extrusion": copy.deepcopy(
                    installed_profile["klipper_extrusion"] if installed_profile
                    else default_calibration()["klipper_extrusion"]
                ),
                "extrusion_ratio_measured": bool(installed_profile
                    and installed_profile["confirmations"]["extrusion_ratio_measured"]),
                "tool_orientation_verified": bool(installed_profile
                    and installed_profile["confirmations"]["tool_orientation_verified"]),
                "legacy_tool0_profile_locked": old_tool0_profile,
                "installed_tcp_m": (
                    installed_profile["tool_center_point_m"]["offset"]
                    if installed_profile and not old_tool0_profile
                    else load_printhead_geometry()["tool0_to_nozzle_m"]
                ),
                "installed_tcp_measured": bool(
                    installed_profile and installed_profile["confirmations"]["tcp_measured"]
                    and installed_profile["confirmations"]["tool_orientation_verified"]
                    and not old_tool0_profile
                ),
            }

    def _planning_workcell_snapshot(self) -> tuple[dict[str, Any], bool, float, bool]:
        """Freeze the real table and the independent G-code Z origin for one review."""

        with self._height_lock:
            geometry = load_workcell_geometry()
            candidate = self._height_draft_m is not None
            if candidate:
                geometry["tabletop"]["surface_z_m"] = self._height_draft_m
                geometry["tabletop"]["measurement"]["surface_z"] = "operator_candidate"
            print_z_candidate = self._print_z_draft_m is not None
            try:
                saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
            except CalibrationValidationError:
                saved = None
            installed_print_z_m = self._saved_print_z_m(
                float(geometry["tabletop"]["surface_z_m"]), saved,
            )
            print_z_candidate = print_z_candidate or self._print_z_verification_fault(
                installed_print_z_m,
            )
            print_zero_z_m = (
                self._print_z_draft_m if self._print_z_draft_m is not None
                else installed_print_z_m if installed_print_z_m is not None
                else geometry["tabletop"]["surface_z_m"]
            )
            return geometry, candidate, print_zero_z_m, print_z_candidate

    def _print_z_placement_mode(self) -> str:
        """Expose how the current absolute base_link path Z was selected."""

        try:
            record = json.loads(self._print_z_placement_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "tabletop_default"
        if isinstance(record, dict) and record.get("version") == 2 and record.get("mode") == "operator_selected":
            return "operator_selected"
        return "legacy_measured" if isinstance(record, dict) and record.get("version") == 1 else "unknown"

    def _print_z_verification_fault(self, installed_print_z_m: float | None) -> bool:
        """Fail closed after a saved Print Z has become missing or invalid."""

        return installed_print_z_m is None and (
            self._print_z_required_path.exists() or self._print_z_placement_path.exists()
        )

    def _saved_print_z_m(
        self, surface_z_m: float, saved_profile: dict[str, Any] | None,
    ) -> float | None:
        """Return the saved path origin; legacy measured offsets retain their binding."""

        try:
            record = json.loads(self._print_z_placement_path.read_text(encoding="utf-8"))
            value = float(record["print_zero_z_m"])
        except (OSError, ValueError, TypeError, KeyError):
            return None
        if not isinstance(record, dict) or not math.isfinite(value):
            return None
        if record.get("version") == 2 and record.get("mode") == "operator_selected":
            # This is a user-selected absolute path origin, not a claim that
            # the physical table is at the same Z. The real table collision
            # plane and MoveIt path checks remain independent.
            return round(value, 6) if surface_z_m <= value <= 2.0 else None
        if not saved_profile:
            return None
        try:
            recorded_surface = float(record["table_surface_z_m"])
            measured_clearance = float(record["measured_clearance_mm"])
        except (ValueError, TypeError, KeyError):
            return None
        if (
            not isinstance(record, dict)
            or record.get("version") != 1
            or record.get("profile_sha256") != saved_profile.get("profile_sha256")
            or record.get("visual_clearance_confirmed") is not True
            or not math.isfinite(value)
            or not math.isfinite(recorded_surface)
            or not math.isfinite(measured_clearance)
            or not 9.5 <= measured_clearance <= 10.5
            or abs(recorded_surface - surface_z_m) > 0.000001
            or not surface_z_m <= value <= surface_z_m + 0.004
        ):
            return None
        return round(value, 6)

    def set_print_z_draft(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Save the operator-selected absolute G-code Z origin, not the table."""

        self._require_no_active_print_preparation()
        with self._height_lock:
            if self.jobs.active() is not None:
                raise ApiError(HTTPStatus.CONFLICT, "Wait for the active operation before changing Print Z.")
            if self._height_draft_m is not None:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Finish or discard the separate physical tabletop-height trial first.",
                )
            surface = float(load_workcell_geometry()["tabletop"]["surface_z_m"])
            if payload.get("discard") is True:
                value = surface
            else:
                try:
                    value = float(payload.get("print_zero_z_mm")) / 1000.0
                except (TypeError, ValueError) as exc:
                    raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Enter Print Z=0 in millimetres.") from exc
            if not math.isfinite(value) or not surface <= value <= 2.0:
                raise ApiError(
                    HTTPStatus.UNPROCESSABLE_ENTITY,
                    "Print Z=0 must be above the physical tabletop and at most 2000 mm in base_link. "
                    "MoveIt still checks reachability and the entire path for collisions.",
                )
            value = round(value, 6)
            record = {
                "version": 2,
                "mode": "operator_selected",
                "print_zero_z_m": value,
                "selected_at": utc_now(),
            }
            marker = self._print_z_required_path
            destination = self._print_z_placement_path
            marker_temporary = marker.with_name(f".{marker.name}.{uuid.uuid4().hex}.tmp")
            temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
            try:
                destination.parent.mkdir(parents=True, exist_ok=True)
                marker_temporary.write_text("print-z-selection-required\n", encoding="utf-8")
                marker_temporary.replace(marker)
                temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
                temporary.replace(destination)
            except OSError as exc:
                raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"Could not save selected Print Z: {exc}") from exc
            finally:
                marker_temporary.unlink(missing_ok=True)
                temporary.unlink(missing_ok=True)
            self._print_z_draft_m = None
            self._print_z_trial = None
        with self._print_execution_lock:
            self._print_packages.clear()
        return self.get_workcell_height()

    def _print_z_trial_target(
        self,
    ) -> tuple[dict[str, Any], dict[str, float], float, float, dict[str, Any]]:
        """Guard a vertical nozzle-clearance check above the real collision table."""

        self.require_online_mode("planning a Print Z clearance trial")
        self._require_no_unconfirmed_print_stop()
        candidate = self._print_z_draft_m
        if candidate is None:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Set a different Print Z candidate first.")
        table = load_workcell_geometry()["tabletop"]
        surface = float(table["surface_z_m"])
        # A larger mismatch indicates an incorrect or unmeasured physical TCP,
        # not a routine print-layer adjustment. Keep it visible for offline
        # diagnosis but do not promote it to a real-motion authorization.
        if candidate > surface + 0.004:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Print Z differs from the physical tabletop by more than 4 mm. "
                "Measure and correct the installed nozzle TCP before a real-robot trial.",
                code="print_z_tcp_remeasurement_required",
            )
        try:
            saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
        except CalibrationValidationError as exc:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, f"Verify the installed nozzle TCP first: {exc}") from exc
        profile = saved.get("calibration") if saved else None
        confirmations = profile.get("confirmations", {}) if profile else {}
        if (not saved or not profile or profile.get("operation_mode") != "production_print"
                or confirmations.get("tcp_measured") is not True
                or confirmations.get("tool_orientation_verified") is not True):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Save a physically measured downward nozzle TCP before verifying Print Z.",
                code="printhead_tcp_required",
            )
        status = self.ros_status() if not self.demo else {
            "moveit_ready": True, "external_control": True, "unstable": False,
        }
        if not status["moveit_ready"] or not status["external_control"] or status["unstable"]:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "UR External Control and MoveIt must be stable for the Print Z trial.",
            )
        pose = self._stable_tool0_pose()
        nozzle = self._nozzle_position(pose, require_print_orientation=True)
        bounds = table["bounds_m"]
        margin = float(table["safety_margin_m"]) + 0.06
        if any(not bounds["minimum"][axis] + margin <= nozzle[axis] <= bounds["maximum"][axis] - margin
               for axis in ("x", "y")):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Position the downward nozzle well inside the tabletop XY area before the Print Z trial.",
            )
        target = round(candidate + HEIGHT_TRIAL_CLEARANCE_M, 6)
        delta = round(target - nozzle["z"], 6)
        if nozzle["z"] < target + 0.001:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The nozzle is too close to the proposed Print Z height. Move it away manually first.",
            )
        if not -0.05 <= delta <= -0.001:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Start with the nozzle in the exact print orientation, 11–60 mm above "
                "the proposed Print Z position, then plan the downward trial.",
            )
        return pose, nozzle, delta, surface, saved

    def start_print_z_trial(self, payload: dict[str, Any], *, execute: bool) -> Job:
        """Plan or explicitly execute a guarded UR-only clearance trial."""

        with self._height_lock:
            self._require_operator_action(
                payload, "execute-print-z-trial" if execute else "plan-print-z-trial",
            )
            if self._height_draft_m is not None:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Finish or discard the separate physical tabletop-height trial first.",
                )
            pose, nozzle, delta, surface, saved = self._print_z_trial_target()
            if execute:
                self._require_action_and_safety(payload, "execute-print-z-trial")
                if payload.get("installed_nozzle_verified") is not True:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "Measure and verify the installed nozzle tip and mount before approaching the table.",
                    )
                trial = self._print_z_trial
                if (not trial or trial.get("print_zero_z_m") != self._print_z_draft_m
                        or trial.get("table_surface_z_m") != surface
                        or trial.get("profile_sha256") != saved["profile_sha256"]):
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Plan this exact Print Z and nozzle setup first.")
                if self.jobs.get(trial["plan_job_id"]).state != "succeeded":
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The Print Z plan-only trial has not succeeded.")
                if any(abs(nozzle[axis] - trial["start_nozzle_m"][axis]) > 0.001
                       for axis in ("x", "y", "z")):
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle moved since planning. Plan again.")
            env = self._environment()
            bounds = load_workcell_geometry()["tabletop"]["bounds_m"]
            env.update({
                "AXES": "z", "DISTANCE_M": str(delta),
                "EXECUTE": "true" if execute else "false",
                "SYNCHRONIZE_KLIPPER": "false", "KLIPPER_CHECK_ONLY": "false",
                "RETURN_TO_START": "false", "MAX_JOINT_SPEED_RAD_S": "0.02",
                "VELOCITY_SCALING": "0.02", "ACCELERATION_SCALING": "0.02",
                "HEIGHT_TRIAL_MODE": "true",
                # Collision geometry is ALWAYS the installed physical table,
                # never the candidate G-code origin.
                "HEIGHT_TRIAL_SURFACE_Z_M": str(surface),
                "HEIGHT_TRIAL_MIN_X_M": str(bounds["minimum"]["x"]),
                "HEIGHT_TRIAL_MAX_X_M": str(bounds["maximum"]["x"]),
                "HEIGHT_TRIAL_MIN_Y_M": str(bounds["minimum"]["y"]),
                "HEIGHT_TRIAL_MAX_Y_M": str(bounds["maximum"]["y"]),
                "HEIGHT_TRIAL_TCP_X_M": str(saved["calibration"]["tool_center_point_m"]["offset"]["x"]),
                "HEIGHT_TRIAL_TCP_Y_M": str(saved["calibration"]["tool_center_point_m"]["offset"]["y"]),
                "HEIGHT_TRIAL_TCP_Z_M": str(saved["calibration"]["tool_center_point_m"]["offset"]["z"]),
                "PRINTHEAD_GEOMETRY_SHA256": printhead_geometry_sha256(),
            })
            if execute:
                env.update({"CONFIRM_REAL_ROBOT_MOVE": "YES", "CONFIRM_CARTESIAN_MOVE": "YES"})
            job = self.jobs.start(
                "execute-print-z-trial" if execute else "plan-print-z-trial",
                platform_operation_command("cartesian-move", PROJECT_ROOT / "scripts" / "ur3e_cartesian_xyz_moveit.sh", demo=self.demo),
                env=env,
                motion=execute,
                demo_lines=["Demo Print Z trial; no UR or Klipper command was sent."] if self.demo else None,
            )
            if execute:
                trial["execute_job_id"] = job.id
            else:
                self._print_z_trial = {
                    "print_zero_z_m": self._print_z_draft_m,
                    "table_surface_z_m": surface,
                    "profile_sha256": saved["profile_sha256"],
                    "clearance_m": HEIGHT_TRIAL_CLEARANCE_M,
                    "start_nozzle_m": nozzle,
                    "target_nozzle_z_m": round(self._print_z_draft_m + HEIGHT_TRIAL_CLEARANCE_M, 6),
                    "distance_m": delta,
                    "plan_job_id": job.id,
                }
            return job

    def confirm_print_z_trial(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Persist the independently measured real print surface after a UR trial."""

        self._require_operator_action(payload, "confirm-print-z-trial")
        self._require_no_unconfirmed_print_stop()
        if payload.get("visual_clearance_confirmed") is not True:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Visually confirm the stationary nozzle clearance before saving Print Z.",
            )
        try:
            measured_clearance_mm = float(payload["measured_clearance_mm"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                "Independently measure the stationary nozzle-to-real-surface gap and enter millimetres.",
                code="print_z_gap_measurement_required",
            ) from exc
        if not math.isfinite(measured_clearance_mm) or not 9.5 <= measured_clearance_mm <= 10.5:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The measured gap must be 9.5–10.5 mm at the trial pose. Recheck the nozzle TCP "
                "and real print surface before saving Print Z.",
                code="print_z_gap_out_of_range",
            )
        with self._height_lock:
            if self.demo:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Demo trials cannot verify real Print Z.")
            self.require_online_mode("confirming Print Z")
            trial = self._print_z_trial
            candidate = self._print_z_draft_m
            if (not trial or candidate is None or trial.get("print_zero_z_m") != candidate
                    or not trial.get("execute_job_id")):
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Execute this exact Print Z trial before confirming it.")
            if self.jobs.get(trial["execute_job_id"]).state != "succeeded":
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The Print Z trial did not finish successfully.")
            surface = float(load_workcell_geometry()["tabletop"]["surface_z_m"])
            try:
                saved = read_saved_calibration(self._calibration_path)
            except CalibrationValidationError as exc:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, f"Nozzle setup changed: {exc}") from exc
            if (not saved or trial["profile_sha256"] != saved["profile_sha256"]
                    or abs(trial["table_surface_z_m"] - surface) > 0.000001):
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle setup or physical table changed. Plan again.")
            nozzle = self._nozzle_position(
                self._stable_tool0_pose(), require_print_orientation=True,
            )
            if abs(nozzle["z"] - trial["target_nozzle_z_m"]) > 0.0015:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Live nozzle position does not confirm the Print Z trial height.")
            if any(abs(nozzle[axis] - trial["start_nozzle_m"][axis]) > 0.0015
                   for axis in ("x", "y")):
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle moved outside the planned vertical trial path.")
            # A 10 mm target computed from the entered TCP is not itself a
            # measurement. The independent real gap supplies the actual Z=0.
            verified_z_m = round(nozzle["z"] - measured_clearance_mm / 1000.0, 6)
            if (not surface <= verified_z_m <= surface + 0.004
                    or abs(verified_z_m - candidate) > 0.0005):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The measured print surface is outside the guarded 4 mm tabletop range "
                    "or differs from the proposed Print Z by more than 0.5 mm. "
                    "Re-measure the nozzle TCP and plan a new trial.",
                    code="print_z_measurement_mismatch",
                )
            record = {
                "version": 1,
                "print_zero_z_m": verified_z_m,
                "table_surface_z_m": surface,
                "profile_sha256": saved["profile_sha256"],
                "visual_clearance_confirmed": True,
                "measured_clearance_mm": measured_clearance_mm,
                "proposed_print_zero_z_m": candidate,
                "trial_job_id": trial["execute_job_id"],
                "verified_at": utc_now(),
            }
            # The marker is written first: an interrupted save cannot make a
            # previously configured print datum silently look like a new setup.
            marker = self._print_z_required_path
            marker_temporary = marker.with_name(f".{marker.name}.{uuid.uuid4().hex}.tmp")
            destination = self._print_z_placement_path
            temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
            try:
                destination.parent.mkdir(parents=True, exist_ok=True)
                marker_temporary.write_text("print-z-verification-required\n", encoding="utf-8")
                marker_temporary.replace(marker)
                temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
                temporary.replace(destination)
            except OSError as exc:
                raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"Could not save verified Print Z: {exc}") from exc
            finally:
                marker_temporary.unlink(missing_ok=True)
                temporary.unlink(missing_ok=True)
            self._print_z_draft_m = None
            self._print_z_trial = None
            with self._print_execution_lock:
                self._print_packages.clear()
            return self.get_workcell_height()

    def save_nozzle_tcp(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._require_operator_action(payload, "save-nozzle-tcp")
        values = payload.get("offset_mm")
        if not isinstance(values, dict):
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Enter the tool0-to-nozzle X/Y/Z offsets in millimetres.")
        try:
            offset = {axis: round(float(values[axis]) / 1000.0, 6) for axis in ("x", "y", "z")}
        except (KeyError, TypeError, ValueError) as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Nozzle offsets must be finite millimetre values.") from exc
        if any(not math.isfinite(value) or abs(value) > 0.5 for value in offset.values()):
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Nozzle offsets must be within 500 mm of tool0.")
        if math.sqrt(sum(value * value for value in offset.values())) < 0.005:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "The physical nozzle cannot be at bare tool0.")
        measured = payload.get("physically_measured") is True
        orientation = payload.get("orientation_verified") is True
        if measured and not orientation:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Confirm the installed nozzle direction before marking its TCP measured.")
        with self._calibration_lock:
            if self.jobs.active() is not None:
                raise ApiError(HTTPStatus.CONFLICT, "Wait for the active operation before changing the nozzle TCP.")
            saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
            profile = copy.deepcopy(saved["calibration"] if saved else default_calibration())
            profile["operation_mode"] = "production_print"
            changed_geometry = profile.get("printhead_geometry_sha256") != printhead_geometry_sha256()
            profile["printhead_geometry_sha256"] = printhead_geometry_sha256()
            profile["tool_center_point_m"] = {
                "offset": offset,
                "collision_radius_m": load_printhead_geometry()["nozzle_collision_radius_m"],
            }
            profile["coordinate_transform"]["tool_orientation_rpy_deg"] = {
                "roll": 180.0, "pitch": 0.0, "yaw": 0.0,
            }
            profile["confirmations"]["tcp_measured"] = measured
            profile["confirmations"]["tool_orientation_verified"] = orientation
            if changed_geometry:
                profile["confirmations"]["extrusion_ratio_measured"] = False
            if "klipper_extrusion" in payload:
                profile["klipper_extrusion"] = copy.deepcopy(payload["klipper_extrusion"])
                profile["confirmations"]["extrusion_ratio_measured"] = payload.get("extrusion_ratio_measured") is True
            profile["confirmations"]["tool0_reference_verified"] = False
            source = saved["source_sha256"] if saved else "0" * 64
            try:
                if self.demo:
                    # Reuse the same schema and hash path without changing the
                    # real workstation's installed calibration file.
                    with tempfile.TemporaryDirectory() as directory:
                        self._demo_calibration = save_installed_calibration_draft(
                            Path(directory) / "calibration.json", profile, source
                        )
                else:
                    save_installed_calibration_draft(self._calibration_path, profile, source)
            except CalibrationValidationError as exc:
                raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
            with self._print_execution_lock:
                self._print_packages.clear()
            with self._height_lock:
                self._height_trial = None
                self._print_z_trial = None
        return self.get_workcell_height()

    def set_workcell_height_draft(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Save the operator-entered tabletop collision plane directly."""

        self._require_no_active_print_preparation()
        if payload.get("discard") is True:
            with self._height_lock:
                if self.jobs.active() is not None:
                    raise ApiError(HTTPStatus.CONFLICT, "Wait for the active operation before clearing a height trial.")
                self._height_draft_m = None
                self._height_trial = None
                return self.get_workcell_height()
        try:
            value = float(payload.get("surface_z_mm")) / 1000.0
        except (TypeError, ValueError) as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Enter a tabletop Z in millimetres.") from exc
        if not math.isfinite(value) or not -0.5 <= value <= 0.5:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Tabletop Z must be between -500 and 500 mm.")
        value = round(value, 6)
        with self._height_lock:
            if self.jobs.active() is not None:
                raise ApiError(HTTPStatus.CONFLICT, "Wait for the active operation before changing tabletop height.")
            current = load_workcell_geometry()["tabletop"]["surface_z_m"]
            if value == round(current, 6):
                self._height_draft_m = None
                self._height_trial = None
                return self.get_workcell_height()
            try:
                save_surface_height_m(value, measurement_source="operator_selected")
            except (OSError, WorkcellGeometryError) as exc:
                raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"Could not save tabletop height: {exc}") from exc
            self._height_draft_m = None
            self._height_trial = None
            self._print_z_draft_m = None
            self._print_z_trial = None
            with self._print_execution_lock:
                self._print_packages.clear()
            return self.get_workcell_height()

    def _height_trial_target(self) -> tuple[dict[str, Any], dict[str, float], float]:
        self.require_online_mode("planning a table-height trial")
        self._require_no_unconfirmed_print_stop()
        if self._height_draft_m is None:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Enter a tabletop height before planning a trial.")
        status = self.ros_status() if not self.demo else {"moveit_ready": True, "external_control": True, "unstable": False}
        if not status["moveit_ready"] or not status["external_control"] or status["unstable"]:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "UR External Control and MoveIt must be stable for a height trial.")
        pose = self._stable_tool0_pose()
        nozzle = self._nozzle_position(pose, require_print_orientation=True)
        tabletop = load_workcell_geometry()["tabletop"]
        bounds = tabletop["bounds_m"]
        margin = tabletop["safety_margin_m"] + 0.06  # whole CAD head footprint
        if any(not bounds["minimum"][axis] + margin <= nozzle[axis] <= bounds["maximum"][axis] - margin
               for axis in ("x", "y")):
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Position the downward nozzle well inside the tabletop XY area before the trial.")
        target = self._height_draft_m + HEIGHT_TRIAL_CLEARANCE_M
        delta = round(target - nozzle["z"], 6)
        if nozzle["z"] < target + 0.001:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle is too close to or below the proposed surface. Move it away manually first.")
        if not -0.05 <= delta <= -0.001:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Start in the exact print orientation, 11–60 mm above the proposed surface, then plan the downward trial.")
        return pose, nozzle, delta

    def _stable_tool0_pose(self) -> dict[str, Any]:
        """Reject a moving arm or a joint-state stream that cannot be sampled."""

        first = self.get_current_tool0_pose()
        time.sleep(0.25)
        second = self.get_current_tool0_pose()
        if any(abs(float(first["position_m"][axis]) - float(second["position_m"][axis])) > 0.0005
               for axis in ("x", "y", "z")):
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The UR is moving; wait for a stationary arm before planning the height trial.")
        if any(abs(float(first["orientation_rpy_deg"][axis]) - float(second["orientation_rpy_deg"][axis])) > 0.5
               for axis in ("roll", "pitch", "yaw")):
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The UR orientation changed; wait for a stationary arm.")
        return second

    def start_workcell_height_trial(self, payload: dict[str, Any], *, execute: bool) -> Job:
        with self._height_lock:
            self._require_operator_action(payload, "execute-height-trial" if execute else "plan-height-trial")
            pose, nozzle, delta = self._height_trial_target()
            if execute:
                try:
                    saved = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
                except CalibrationValidationError as exc:
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, f"Verify the installed nozzle TCP first: {exc}") from exc
                profile = saved.get("calibration") if saved else None
                confirmations = profile.get("confirmations", {}) if profile else {}
                if (not profile or profile.get("operation_mode") != "production_print"
                        or confirmations.get("tcp_measured") is not True
                        or confirmations.get("tool_orientation_verified") is not True):
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "Save a physically measured nozzle TCP and verify its installed direction before moving UR near the tabletop.",
                    )
                self._require_action_and_safety(payload, "execute-height-trial")
                if payload.get("installed_nozzle_verified") is not True:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "Measure and verify the actual installed nozzle tip and mount before a close tabletop trial.",
                    )
                trial = self._height_trial
                if not trial or trial.get("surface_z_m") != self._height_draft_m:
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Plan this exact height first.")
                if trial.get("profile_sha256") != saved["profile_sha256"]:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "The measured nozzle TCP changed since the height plan. Plan again.",
                    )
                if self.jobs.get(trial["plan_job_id"]).state != "succeeded":
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The plan-only trial has not succeeded.")
                if any(abs(nozzle[axis] - trial["start_nozzle_m"][axis]) > 0.001 for axis in ("x", "y", "z")):
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle moved since planning. Plan again.")
            env = self._environment()
            tabletop = load_workcell_geometry()["tabletop"]
            bounds = tabletop["bounds_m"]
            saved_for_collision = (
                self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
            )
            measured_tcp = (
                saved_for_collision["calibration"]["tool_center_point_m"]["offset"]
                if saved_for_collision and saved_for_collision["calibration"].get("operation_mode") == "production_print"
                else load_printhead_geometry()["tool0_to_nozzle_m"]
            )
            env.update({
                "AXES": "z", "DISTANCE_M": str(delta),
                "EXECUTE": "true" if execute else "false",
                "SYNCHRONIZE_KLIPPER": "false", "KLIPPER_CHECK_ONLY": "false",
                "RETURN_TO_START": "false", "MAX_JOINT_SPEED_RAD_S": "0.02",
                "VELOCITY_SCALING": "0.02", "ACCELERATION_SCALING": "0.02",
                "HEIGHT_TRIAL_MODE": "true",
                # Until a lower proposed table is independently verified, the
                # installed physical surface remains a collision obstacle.
                "HEIGHT_TRIAL_SURFACE_Z_M": str(max(
                    float(tabletop["surface_z_m"]), self._height_draft_m,
                )),
                "HEIGHT_TRIAL_MIN_X_M": str(bounds["minimum"]["x"]),
                "HEIGHT_TRIAL_MAX_X_M": str(bounds["maximum"]["x"]),
                "HEIGHT_TRIAL_MIN_Y_M": str(bounds["minimum"]["y"]),
                "HEIGHT_TRIAL_MAX_Y_M": str(bounds["maximum"]["y"]),
                "HEIGHT_TRIAL_TCP_X_M": str(measured_tcp["x"]),
                "HEIGHT_TRIAL_TCP_Y_M": str(measured_tcp["y"]),
                "HEIGHT_TRIAL_TCP_Z_M": str(measured_tcp["z"]),
                "PRINTHEAD_GEOMETRY_SHA256": printhead_geometry_sha256(),
            })
            if execute:
                env.update({"CONFIRM_REAL_ROBOT_MOVE": "YES", "CONFIRM_CARTESIAN_MOVE": "YES"})
            job = self.jobs.start(
                "execute-height-trial" if execute else "plan-height-trial",
                platform_operation_command("cartesian-move", PROJECT_ROOT / "scripts" / "ur3e_cartesian_xyz_moveit.sh", demo=self.demo),
                env=env,
                motion=execute,
                demo_lines=["Demo height trial; no UR or Klipper command was sent."] if self.demo else None,
            )
            if execute:
                trial["execute_job_id"] = job.id
            else:
                self._height_trial = {
                    "surface_z_m": self._height_draft_m,
                    "profile_sha256": (
                        saved_for_collision["profile_sha256"] if saved_for_collision else None
                    ),
                    "clearance_m": HEIGHT_TRIAL_CLEARANCE_M,
                    "start_nozzle_m": nozzle,
                    "target_nozzle_z_m": round(self._height_draft_m + HEIGHT_TRIAL_CLEARANCE_M, 6),
                    "distance_m": delta,
                    "plan_job_id": job.id,
                }
            return job

    def confirm_workcell_height(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._require_operator_action(payload, "confirm-height-trial")
        if payload.get("visual_clearance_confirmed") is not True:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Visually confirm the stationary nozzle clearance before saving the height.")
        try:
            measured_clearance_mm = float(payload["measured_clearance_mm"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                "Independently measure the stationary nozzle-to-real-table gap and enter millimetres.",
                code="table_gap_measurement_required",
            ) from exc
        if not math.isfinite(measured_clearance_mm) or not 9.5 <= measured_clearance_mm <= 10.5:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The measured real-table gap must be 9.5–10.5 mm at the trial pose.",
                code="table_gap_out_of_range",
            )
        with self._height_lock:
            trial = self._height_trial
            if not trial or trial.get("surface_z_m") != self._height_draft_m or not trial.get("execute_job_id"):
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Execute this height trial before confirming it.")
            if self.jobs.get(trial["execute_job_id"]).state != "succeeded":
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The height trial did not finish successfully.")
            if not self.demo:
                try:
                    saved = read_saved_calibration(self._calibration_path)
                except CalibrationValidationError as exc:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        f"The installed nozzle calibration changed: {exc}",
                    ) from exc
                if not saved or trial.get("profile_sha256") != saved["profile_sha256"]:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "The measured nozzle TCP changed since the height trial. Plan again.",
                    )
                nozzle = self._nozzle_position(
                    self._stable_tool0_pose(), require_print_orientation=True,
                )
                if abs(nozzle["z"] - trial["target_nozzle_z_m"]) > 0.0015:
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Live nozzle position does not confirm the trial height.")
                if any(abs(nozzle[axis] - trial["start_nozzle_m"][axis]) > 0.0015 for axis in ("x", "y")):
                    raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The nozzle moved outside the planned vertical trial path.")
            else:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Demo trials cannot change the installed tabletop height.")
            measured_surface_z_m = round(nozzle["z"] - measured_clearance_mm / 1000.0, 6)
            if (not -0.5 <= measured_surface_z_m <= 0.5
                    or abs(measured_surface_z_m - self._height_draft_m) > 0.0005):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The measured real tabletop differs from the proposed height by more than 0.5 mm. "
                    "Measure the nozzle TCP and plan a new trial.",
                    code="table_gap_measurement_mismatch",
                )
            try:
                save_surface_height_m(measured_surface_z_m)
            except (OSError, WorkcellGeometryError) as exc:
                raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"Could not save verified tabletop height: {exc}") from exc
            self._height_draft_m = None
            self._height_trial = None
            with self._print_execution_lock:
                self._print_packages.clear()
            return self.get_workcell_height()

    def validate_cartesian_test_over_print_bed(
        self,
        config: dict[str, Any],
    ) -> dict[str, Any]:
        """Require the live relative test path to stay over the measured bed."""

        try:
            saved = read_saved_calibration(self._calibration_path)
            if saved is None:
                raise CalibrationNotReadyError(
                    "The installed print setup is unavailable. Configure the "
                    "tool reference before Cartesian motion."
                )
            if saved["calibration"].get("operation_mode") == "tool0_motion_test":
                raise CalibrationNotReadyError(
                    "The installed printhead extends below tool0. Legacy Tool0-on-table motion is locked."
                )
            pose = self.get_current_tool0_pose()
            return validate_relative_motion_over_print_bed(
                saved["calibration"],
                pose["position_m"],
                config["axes"],
                config["distance_m"],
            )
        except (CalibrationValidationError, CalibrationNotReadyError) as exc:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, str(exc)) from exc

    def get_gcode_calibration(self) -> dict[str, Any]:
        """Return the saved workcell profile or a safe unconfirmed template."""

        with self._calibration_lock:
            try:
                saved = (
                    self._demo_calibration
                    if self.demo
                    else read_saved_calibration(self._calibration_path)
                )
            except CalibrationValidationError as exc:
                raise ApiError(
                    HTTPStatus.UNPROCESSABLE_ENTITY,
                    str(exc),
                ) from exc
            if saved is None:
                return {
                    "saved": False,
                    "calibration": default_calibration(),
                    "workcell_geometry": load_workcell_geometry(),
                    "saved_at": "",
                    "source_sha256": "",
                    "profile_sha256": "",
                }
            return {
                "saved": True,
                "calibration": saved["calibration"],
                "workcell_geometry": load_workcell_geometry(),
                "saved_at": saved["saved_at"],
                "source_sha256": saved["source_sha256"],
                "profile_sha256": saved["profile_sha256"],
                "printhead_recalibration_required": saved.get("printhead_recalibration_required", False),
                "printhead_migration": saved.get("printhead_migration"),
            }

    def validate_gcode_calibration(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            validation = validate_calibration(
                payload.get("calibration"),
                payload.get("bounds"),
                payload.get("source_sha256"),
            )
        except CalibrationValidationError as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                str(exc),
            ) from exc
        with self._calibration_lock:
            source_sha256 = validation["source_sha256"]
            self._simulation_calibration_drafts.pop(source_sha256, None)
            self._simulation_calibration_drafts[source_sha256] = validation[
                "calibration"
            ]
            while len(self._simulation_calibration_drafts) > 8:
                self._simulation_calibration_drafts.pop(
                    next(iter(self._simulation_calibration_drafts))
                )
        return validation

    def save_gcode_calibration(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        validation = self.validate_gcode_calibration(payload)
        if validation["ready_to_save"] is not True:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Complete every print-setup confirmation and keep the "
                "automatically centered path inside the fixed table before saving.",
            )
        with self._calibration_lock:
            try:
                if self.demo:
                    saved = {
                        "saved_at": utc_now(),
                        "source_sha256": validation["source_sha256"],
                        "profile_sha256": validation["profile_sha256"],
                        "calibration": validation["calibration"],
                    }
                    self._demo_calibration = saved
                else:
                    saved = save_calibration(
                        self._calibration_path,
                        validation,
                    )
            except (CalibrationValidationError, CalibrationNotReadyError) as exc:
                raise ApiError(
                    HTTPStatus.UNPROCESSABLE_ENTITY,
                    str(exc),
                ) from exc
        return {
            "saved": True,
            "saved_at": saved["saved_at"],
            "source_sha256": saved["source_sha256"],
            "profile_sha256": saved["profile_sha256"],
            "calibration": saved["calibration"],
            "validation": validation,
        }

    def _automatic_calibration_for_analysis(
        self,
        analysis: dict[str, Any],
        *,
        calibration_snapshot: dict[str, Any] | None | object = _CURRENT_CALIBRATION,
        workcell_snapshot: dict[str, Any] | None = None,
        print_zero_z_m: float | None = None,
        print_z_candidate: bool = False,
        allow_printhead_candidate: bool = False,
    ) -> dict[str, Any]:
        """Apply the installed print setup to one newly parsed G-code job."""

        if calibration_snapshot is _CURRENT_CALIBRATION:
            with self._calibration_lock:
                try:
                    saved = (
                        self._demo_calibration
                        if self.demo
                        else read_saved_calibration(self._calibration_path)
                    )
                except CalibrationValidationError as exc:
                    raise ApiError(
                        HTTPStatus.UNPROCESSABLE_ENTITY,
                        str(exc),
                    ) from exc
        else:
            saved = (
                {"calibration": copy.deepcopy(calibration_snapshot)}
                if isinstance(calibration_snapshot, dict)
                else None
            )
        if saved is None:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The installed print setup is unavailable. Motion Planning "
                "cannot safely infer tool orientation, TCP, or extrusion settings.",
            )
        if saved["calibration"].get("operation_mode") == "tool0_motion_test" and not self.demo:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The installed printhead replaced bare tool0. Open Tabletop height & nozzle, "
                "enter the measured nozzle TCP, and rebuild the review before motion planning.",
                code="printhead_tcp_required",
            )
        stale_printhead = (saved["calibration"].get("operation_mode") == "production_print"
                           and saved["calibration"].get("printhead_geometry_sha256") != printhead_geometry_sha256())
        if stale_printhead:
            if not allow_printhead_candidate:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED,
                    "The installed extruder changed. Re-measure the nozzle TCP, verify its direction, and calibrate extrusion before MoveIt or Print Execution.",
                    code="printhead_recalibration_required")
            candidate_profile = copy.deepcopy(saved["calibration"])
            candidate = default_calibration()
            candidate_profile["printhead_geometry_sha256"] = candidate["printhead_geometry_sha256"]
            candidate_profile["tool_center_point_m"] = candidate["tool_center_point_m"]
            for confirmation in ("tcp_measured", "tool_orientation_verified", "extrusion_ratio_measured"):
                candidate_profile["confirmations"][confirmation] = False
            saved = {"calibration": candidate_profile}
        try:
            validation = validate_calibration(
                saved["calibration"],
                analysis["bounds"],
                analysis["file"]["sha256"],
                workcell_snapshot=workcell_snapshot,
                print_zero_z_m=print_zero_z_m,
            )
        except CalibrationValidationError as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                str(exc),
            ) from exc
        if allow_printhead_candidate and validation["operation_mode"] == "production_print" and (
                stale_printhead or not validation["confirmations_complete"]):
            return {**validation["calibration"], "_printhead_candidate_preview": True}
        if (validation["ready_to_save"] is not True
                and not (print_z_candidate and print_z_candidate_ready_for_planning(validation))):
            if validation["within_print_bed"] is not True:
                outside_axes = []
                for axis, check in validation["print_bed_check"][
                    "axis_checks"
                ].items():
                    if check["inside"]:
                        continue
                    overflow_m = max(
                        float(check["allowed_minimum"])
                        - float(check["actual_minimum"]),
                        float(check["actual_maximum"])
                        - float(check["allowed_maximum"]),
                        0.0,
                    )
                    outside_axes.append(
                        f"{axis.upper()} by {overflow_m * 1000.0:.2f} mm"
                    )
                detail = ", ".join(outside_axes) or "an unknown axis"
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The robot trajectory is outside the fixed table safety "
                    f"margin on {detail}.",
                )
            if validation["print_bed_surface_aligned"] is not True:
                gap_mm = (
                    float(validation["print_bed_check"]["surface_gap_m"])
                    * 1000.0
                )
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The robot trajectory goes below the physical tabletop "
                    f"(minimum path gap {gap_mm:.2f} mm).",
                )
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The installed print setup is incomplete. Motion Planning "
                "requires confirmed tool orientation, TCP/tool0 reference, "
                "and extrusion mapping.",
            )
        return validation["calibration"]

    def build_gcode_path_review(
        self,
        filename: str,
        content: bytes,
        *,
        progress_callback: Callable[[int, int], None] | None = None,
        calibration_snapshot: dict[str, Any] | None | object = _CURRENT_CALIBRATION,
        workcell_snapshot: dict[str, Any] | None = None,
        tabletop_height_candidate: bool | None = None,
        print_zero_z_m: float | None = None,
        print_z_candidate: bool | None = None,
    ) -> dict[str, Any]:
        """Build an offline review using only the saved, source-matched profile."""

        if workcell_snapshot is None:
            (workcell_snapshot, tabletop_height_candidate,
             print_zero_z_m, print_z_candidate) = self._planning_workcell_snapshot()
        candidate = bool(tabletop_height_candidate)
        try:
            analysis = parse_gcode(filename, content)
        except GCodeParseError as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
        automatic_calibration = self._automatic_calibration_for_analysis(
            analysis,
            calibration_snapshot=calibration_snapshot,
            workcell_snapshot=workcell_snapshot,
            print_zero_z_m=print_zero_z_m,
            print_z_candidate=bool(print_z_candidate),
            allow_printhead_candidate=True,
        )
        try:
            printhead_candidate = automatic_calibration.pop("_printhead_candidate_preview", False)
            review = build_offline_review(
                filename,
                content,
                automatic_calibration,
                allow_incomplete_simulation=printhead_candidate,
                workcell_snapshot=workcell_snapshot,
                tabletop_height_candidate=candidate,
                print_zero_z_m=print_zero_z_m,
                print_z_candidate=bool(print_z_candidate),
            )
            if printhead_candidate:
                review["printhead_recalibration_required"] = True
                review["printhead_setup_message"] = "Preview uses the new CAD nozzle candidate. Re-measure TCP and direction, then calibrate extrusion before planning or printing."
            review["offline_simulation"] = build_offline_robot_simulation(
                review,
                progress_callback=progress_callback,
            )
            if not self.demo and not printhead_candidate:
                review["moveit_validation"] = find_cached_moveit_result(
                    self._path_review_runtime,
                    review["review_sha256"],
                )
            return review
        except (
            GCodeParseError,
            CalibrationValidationError,
            PathReviewError,
        ) as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                str(exc),
            ) from exc

    def build_gcode_simulation_review(
        self,
        filename: str,
        content: bytes,
        *,
        progress_callback: Callable[[int, int], None] | None = None,
        calibration_snapshot: dict[str, Any] | None | object = _CURRENT_CALIBRATION,
        workcell_snapshot: dict[str, Any] | None = None,
        tabletop_height_candidate: bool | None = None,
        print_zero_z_m: float | None = None,
        print_z_candidate: bool | None = None,
    ) -> dict[str, Any]:
        """Build an execution-locked 3D estimate from the latest draft."""

        if workcell_snapshot is None:
            (workcell_snapshot, tabletop_height_candidate,
             print_zero_z_m, print_z_candidate) = self._planning_workcell_snapshot()
        candidate = bool(tabletop_height_candidate)
        try:
            analysis = parse_gcode(filename, content)
        except GCodeParseError as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
        source_sha256 = analysis["file"]["sha256"]
        if calibration_snapshot is _CURRENT_CALIBRATION:
            with self._calibration_lock:
                draft = copy.deepcopy(
                    self._simulation_calibration_drafts.get(source_sha256)
                )
        else:
            draft = (
                copy.deepcopy(calibration_snapshot)
                if isinstance(calibration_snapshot, dict)
                else None
            )
        if draft is None:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Validate the current calibration draft before opening the "
                "simulation-only 3D review.",
            )
        try:
            review = build_offline_review(
                filename,
                content,
                draft,
                allow_incomplete_simulation=True,
                use_initial_workcell_model=True,
                workcell_snapshot=workcell_snapshot,
                tabletop_height_candidate=candidate,
                print_zero_z_m=print_zero_z_m,
                print_z_candidate=bool(print_z_candidate),
            )
            review["offline_simulation"] = build_offline_robot_simulation(
                review,
                progress_callback=progress_callback,
            )
            review["moveit_validation"] = None
            return review
        except (
            GCodeParseError,
            CalibrationValidationError,
            PathReviewError,
        ) as exc:
            raise ApiError(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                str(exc),
            ) from exc

    def _gcode_review_profile_snapshot(
        self,
        source_sha256: str,
        *,
        simulation_only: bool,
    ) -> tuple[str, dict[str, Any] | None, str | None, dict[str, Any], bool, float, bool]:
        """Freeze and hash the exact profile without doing path or IK work."""

        (workcell_snapshot, tabletop_height_candidate,
         print_zero_z_m, print_z_candidate) = self._planning_workcell_snapshot()
        profile_error = None
        if simulation_only:
            with self._calibration_lock:
                calibration = copy.deepcopy(
                    self._simulation_calibration_drafts.get(source_sha256)
                )
            identity: dict[str, Any] = {
                "state": "available" if calibration is not None else "missing",
                "calibration": calibration,
            }
        else:
            try:
                with self._calibration_lock:
                    saved = (
                        copy.deepcopy(self._demo_calibration)
                        if self.demo
                        else read_saved_calibration(self._calibration_path)
                    )
                identity = {
                    "state": "available" if saved is not None else "missing",
                    "calibration": (
                        saved.get("calibration") if saved is not None else None
                    ),
                }
                calibration = copy.deepcopy(identity["calibration"])
            except CalibrationValidationError as exc:
                # Keep malformed-profile requests distinct when the operator
                # repairs or replaces the file. The worker will surface the
                # normal structured validation error without blocking POST.
                try:
                    profile_bytes = self._calibration_path.read_bytes()
                except OSError:
                    profile_bytes = b""
                identity = {
                    "state": "invalid",
                    "raw_sha256": hashlib.sha256(profile_bytes).hexdigest(),
                }
                calibration = None
                profile_error = str(exc)
        canonical = json.dumps(
            {"profile": identity, "workcell": workcell_snapshot,
             "printhead_geometry_sha256": printhead_geometry_sha256(),
             "tabletop_height_candidate": tabletop_height_candidate,
             "print_zero_z_m": print_zero_z_m,
             "print_z_candidate": print_z_candidate},
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return (
            hashlib.sha256(canonical).hexdigest(),
            calibration,
            profile_error,
            workcell_snapshot,
            tabletop_height_candidate,
            print_zero_z_m,
            print_z_candidate,
        )

    @staticmethod
    def _public_gcode_review_job(
        job: dict[str, Any],
        *,
        include_result: bool,
    ) -> dict[str, Any]:
        public = {
            key: value
            for key, value in job.items()
            if not key.startswith("_") and key != "review"
        }
        if include_result and job.get("state") == "succeeded":
            public["review"] = job["review"]
        return public

    def _prune_gcode_review_jobs_locked(self) -> None:
        terminal = sorted(
            (
                job
                for job in self._gcode_review_jobs.values()
                if job["state"] in {"succeeded", "failed"}
            ),
            key=lambda job: job["_sequence"],
        )
        for job in terminal[:-ASYNC_REVIEW_MAX_RETAINED]:
            self._gcode_review_jobs.pop(job["id"], None)
            key = job["_key"]
            if self._gcode_review_job_ids_by_key.get(key) == job["id"]:
                self._gcode_review_job_ids_by_key.pop(key, None)

    def start_gcode_review_job(
        self,
        filename: str,
        content: bytes,
        *,
        simulation_only: bool,
    ) -> dict[str, Any]:
        """Start one bounded, single-flight, non-motion review build."""

        source_sha256 = hashlib.sha256(content).hexdigest()
        (
            profile_sha256,
            calibration_snapshot,
            profile_error,
            workcell_snapshot,
            tabletop_height_candidate,
            print_zero_z_m,
            print_z_candidate,
        ) = self._gcode_review_profile_snapshot(
            source_sha256,
            simulation_only=simulation_only,
        )
        mode = "simulation" if simulation_only else "installed"
        key = (
            source_sha256,
            profile_sha256,
            mode,
            filename,
            ASYNC_REVIEW_ALGORITHM_VERSION,
        )
        with self._gcode_review_lock:
            self._prune_gcode_review_jobs_locked()
            existing_id = self._gcode_review_job_ids_by_key.get(key)
            existing = (
                self._gcode_review_jobs.get(existing_id)
                if existing_id is not None
                else None
            )
            if existing is not None and existing["state"] != "failed":
                return self._public_gcode_review_job(
                    existing,
                    include_result=False,
                )

            active_count = sum(
                job["state"] in {"queued", "running"}
                for job in self._gcode_review_jobs.values()
            )
            if active_count >= ASYNC_REVIEW_MAX_ACTIVE:
                raise ApiError(
                    HTTPStatus.TOO_MANY_REQUESTS,
                    "Too many 3D reviews are already being generated. Wait for the current review to finish.",
                    code="gcode_review_queue_full",
                )

            job_id = uuid.uuid4().hex[:12]
            job = {
                "id": job_id,
                "state": "queued",
                "mode": mode,
                "phase": "queued",
                "progress": 0,
                "status_url": f"/api/gcode/review/jobs/{job_id}",
                "source_sha256": source_sha256,
                "profile_sha256": profile_sha256,
                "created_at": utc_now(),
                "started_at": None,
                "finished_at": None,
                "_key": key,
                "_sequence": time.monotonic_ns(),
                "_calibration_snapshot": calibration_snapshot,
                "_profile_error": profile_error,
                "_workcell_snapshot": workcell_snapshot,
                "_tabletop_height_candidate": tabletop_height_candidate,
                "_print_zero_z_m": print_zero_z_m,
                "_print_z_candidate": print_z_candidate,
            }
            self._gcode_review_jobs[job_id] = job
            self._gcode_review_job_ids_by_key[key] = job_id

        worker = threading.Thread(
            target=self._run_gcode_review_job,
            args=(job_id, filename, bytes(content), simulation_only),
            name=f"gcode-review-{job_id}",
            daemon=True,
        )
        worker.start()
        with self._gcode_review_lock:
            return self._public_gcode_review_job(
                self._gcode_review_jobs[job_id],
                include_result=False,
            )

    def _run_gcode_review_job(
        self,
        job_id: str,
        filename: str,
        content: bytes,
        simulation_only: bool,
    ) -> None:
        def report_progress(completed: int, total: int) -> None:
            ratio = (completed / total) if total > 0 else 0.0
            progress = max(10, min(95, 10 + round(ratio * 85)))
            with self._gcode_review_lock:
                active_job = self._gcode_review_jobs.get(job_id)
                if active_job is not None and active_job["state"] == "running":
                    active_job.update(
                        phase="estimating_robot_motion",
                        progress=progress,
                    )

        with self._gcode_review_cpu:
            with self._gcode_review_lock:
                job = self._gcode_review_jobs.get(job_id)
                if job is None:
                    return
                job.update(
                    state="running",
                    phase="building_review",
                    progress=10,
                    started_at=utc_now(),
                )
                calibration_snapshot = copy.deepcopy(
                    job["_calibration_snapshot"]
                )
                profile_error = job["_profile_error"]
                workcell_snapshot = copy.deepcopy(job["_workcell_snapshot"])
                tabletop_height_candidate = job["_tabletop_height_candidate"]
                print_zero_z_m = job["_print_zero_z_m"]
                print_z_candidate = job["_print_z_candidate"]
            try:
                if profile_error:
                    raise ApiError(
                        HTTPStatus.UNPROCESSABLE_ENTITY,
                        profile_error,
                    )
                review = (
                    self.build_gcode_simulation_review(
                        filename,
                        content,
                        progress_callback=report_progress,
                        calibration_snapshot=calibration_snapshot,
                        workcell_snapshot=workcell_snapshot,
                        tabletop_height_candidate=tabletop_height_candidate,
                        print_zero_z_m=print_zero_z_m,
                        print_z_candidate=print_z_candidate,
                    )
                    if simulation_only
                    else self.build_gcode_path_review(
                        filename,
                        content,
                        progress_callback=report_progress,
                        calibration_snapshot=calibration_snapshot,
                        workcell_snapshot=workcell_snapshot,
                        tabletop_height_candidate=tabletop_height_candidate,
                        print_zero_z_m=print_zero_z_m,
                        print_z_candidate=print_z_candidate,
                    )
                )
            except ApiError as exc:
                failure = {
                    "state": "failed",
                    "phase": "failed",
                    "progress": 100,
                    "finished_at": utc_now(),
                    "error": exc.message,
                    "http_status": int(exc.status),
                }
                if exc.code:
                    failure["code"] = exc.code
                if exc.details:
                    failure["details"] = exc.details
                with self._gcode_review_lock:
                    job = self._gcode_review_jobs.get(job_id)
                    if job is not None:
                        job.update(failure)
                        if self._gcode_review_job_ids_by_key.get(job["_key"]) == job_id:
                            self._gcode_review_job_ids_by_key.pop(job["_key"], None)
                        self._prune_gcode_review_jobs_locked()
                return
            except Exception as exc:  # keep local build failures visible to the UI
                with self._gcode_review_lock:
                    job = self._gcode_review_jobs.get(job_id)
                    if job is not None:
                        job.update(
                            state="failed",
                            phase="failed",
                            progress=100,
                            finished_at=utc_now(),
                            error=f"Internal review generation error: {exc}",
                            http_status=int(HTTPStatus.INTERNAL_SERVER_ERROR),
                        )
                        if self._gcode_review_job_ids_by_key.get(job["_key"]) == job_id:
                            self._gcode_review_job_ids_by_key.pop(job["_key"], None)
                        self._prune_gcode_review_jobs_locked()
                return

            with self._gcode_review_lock:
                job = self._gcode_review_jobs.get(job_id)
                if job is None:
                    return
                job.update(
                    state="succeeded",
                    phase="complete",
                    progress=100,
                    finished_at=utc_now(),
                    review=review,
                )
                self._prune_gcode_review_jobs_locked()

    def get_gcode_review_job(self, job_id: str) -> dict[str, Any]:
        with self._gcode_review_lock:
            job = self._gcode_review_jobs.get(job_id)
            if job is None:
                raise ApiError(HTTPStatus.NOT_FOUND, "3D review job not found.")
            return self._public_gcode_review_job(job, include_result=True)

    def start_gcode_moveit_review(
        self,
        filename: str,
        content: bytes,
    ) -> dict[str, Any]:
        """Start a non-motion MoveIt review for a calibrated, source-matched path."""

        self._require_no_active_print_preparation()
        (workcell_snapshot, tabletop_height_candidate,
         print_zero_z_m, print_z_candidate) = self._planning_workcell_snapshot()
        if self.jobs.active() is not None:
            raise ApiError(HTTPStatus.CONFLICT, "Another workstation operation is already running.")
        try:
            analysis = parse_gcode(filename, content)
        except GCodeParseError as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
        automatic_calibration = self._automatic_calibration_for_analysis(
            analysis,
            workcell_snapshot=workcell_snapshot,
            print_zero_z_m=print_zero_z_m,
            print_z_candidate=print_z_candidate,
        )
        try:
            review = build_offline_review(
                filename,
                content,
                automatic_calibration,
                maximum_preview_segments=(
                    None if self.demo else MAX_MOVEIT_WAYPOINTS - 1
                ),
                workcell_snapshot=workcell_snapshot,
                tabletop_height_candidate=tabletop_height_candidate,
                print_zero_z_m=print_zero_z_m,
                print_z_candidate=print_z_candidate,
            )
        except (
            GCodeParseError,
            CalibrationValidationError,
            PathReviewError,
        ) as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
        review_id = uuid.uuid4().hex[:12]
        result_path = self._path_review_runtime / f"{review_id}.json"
        path_metadata = {
            "source_waypoint_count": (
                review["path"]["returned_segment_count"] + 1
            ),
            "written_waypoint_count": 0,
            "sampled": review["path"]["sampled"],
        }
        if not self.demo:
            if not moveit_process_running():
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Start ROS and MoveIt before requesting MoveIt path validation.",
                )
            path_file = self._path_review_runtime / f"{review_id}.csv"
            try:
                path_metadata = write_moveit_path(path_file, review)
            except PathReviewError as exc:
                raise ApiError(
                    HTTPStatus.UNPROCESSABLE_ENTITY,
                    str(exc),
                ) from exc

        orientation = review["calibration"]["coordinate_transform"][
            "tool_orientation_rpy_deg"
        ]
        tcp = review["calibration"]["tool_center_point_m"]
        bed = review["collision_bed_m"]
        operation_mode = review["calibration"]["operation_mode"]
        command = platform_operation_command(
            "validate-path",
            PROJECT_ROOT / "scripts" / "validate_gcode_path_moveit.sh",
            review_id,
            review["review_sha256"],
            str(path_metadata["source_waypoint_count"]),
            "true"
            if path_metadata["sampled"] or review["path"]["sampled"]
            else "false",
            str(orientation["roll"]),
            str(orientation["pitch"]),
            str(orientation["yaw"]),
            str(tcp["offset"]["x"]),
            str(tcp["offset"]["y"]),
            str(tcp["offset"]["z"]),
            str(tcp["collision_radius_m"]),
            str(bed["minimum"]["x"]),
            str(bed["maximum"]["x"]),
            str(bed["minimum"]["y"]),
            str(bed["maximum"]["y"]),
            str(bed["plane_z"]),
            operation_mode,
            review["printhead_geometry_sha256"],
            demo=self.demo,
        )
        demo_lines = [
            "Starting demo-only MoveIt path review.",
            "Loading the calibrated Cartesian path and UR3e model…",
            "Checking sampled IK, joint limits, and collision-aware planning…",
            "Demo validation complete. No ROS service or hardware was contacted.",
            "Execution remains locked.",
        ]
        # Revoke the previous result and prepared packages before starting a
        # new attempt. A failed launch or plan must not restore old artifacts.
        with self.jobs._lock:
            if self.jobs.active() is not None:
                raise ApiError(HTTPStatus.CONFLICT, "Another workstation operation is already running.")
            request_path = begin_planning_request(
                self._path_review_runtime, review["review_sha256"], review_id
            )
            job = self.jobs.start(
                "gcode-path-review",
                command,
                motion=False,
                demo_lines=demo_lines if self.demo else None,
            )
        stored_review = (
            review
            if self.demo
            else {"review_sha256": review["review_sha256"]}
        )
        with self._path_review_lock:
            self._path_review_jobs[job.id] = {
                "review_id": review_id,
                "review": stored_review,
                "result_path": result_path,
                "planning_request_path": request_path,
                "approach_trajectory_path": (
                    self._path_review_runtime / f"{review_id}.approach.csv"
                ),
                "print_trajectory_path": (
                    self._path_review_runtime / f"{review_id}.print.csv"
                ),
                "path_metadata": path_metadata,
                "tabletop_height_candidate": tabletop_height_candidate,
                "print_z_candidate": print_z_candidate,
                "tabletop_surface_z_m": float(workcell_snapshot["tabletop"]["surface_z_m"]),
                "print_zero_z_m": float(print_zero_z_m),
            }
        return {
            "job": job.public(),
            "review_sha256": review["review_sha256"],
            "tabletop_height_candidate": tabletop_height_candidate,
            "print_z_candidate": print_z_candidate,
            "path_metadata": path_metadata,
        }

    def get_gcode_moveit_review_result(
        self,
        job_id: str,
    ) -> dict[str, Any]:
        job = self.jobs.get(job_id)
        if job.kind != "gcode-path-review":
            raise ApiError(
                HTTPStatus.NOT_FOUND,
                "This job is not a MoveIt path-review request.",
            )
        with self._path_review_lock:
            metadata = self._path_review_jobs.get(job_id)
        if metadata is None:
            raise ApiError(
                HTTPStatus.NOT_FOUND,
                "MoveIt path-review metadata is unavailable.",
            )
        self._require_current_planning_request(metadata)
        result = None
        if job.state not in {"queued", "running"}:
            try:
                result = (
                    demo_moveit_result(metadata["review"])
                    if self.demo
                    else read_moveit_result(
                        metadata["result_path"],
                        metadata["review"]["review_sha256"],
                    )
                )
            except PathReviewError as exc:
                if job.state == "failed":
                    raise ApiError(
                        HTTPStatus.CONFLICT,
                        f"{exc} Review the job log for the MoveIt failure.",
                    ) from exc
                raise ApiError(
                    HTTPStatus.UNPROCESSABLE_ENTITY,
                    str(exc),
                ) from exc
        return {
            "job": job.public(),
            "result": result,
            "path_metadata": metadata["path_metadata"],
        }

    def _require_current_planning_request(self, metadata: dict[str, Any]) -> None:
        try:
            request = current_planning_request(
                self._path_review_runtime, metadata["review"]["review_sha256"]
            )
        except PathReviewError as exc:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, str(exc)) from exc
        if request is None or request["review_id"] != metadata["review_id"]:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "This MoveIt review was superseded by a newer planning attempt. Validate again.",
            )

    @staticmethod
    def _write_demo_trajectory(
        path: Path,
        trajectory: list[dict[str, Any]],
    ) -> None:
        joint_names = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        rows = ["time_s,path_progress," + ",".join(joint_names)]
        final_index = max(1, len(trajectory) - 1)
        for index, point in enumerate(trajectory):
            positions = point.get("positions", [])
            if len(positions) != 6:
                raise PrintExecutionError("Demo trajectory has invalid UR joints.")
            timestamp = float(point.get("time_s", index * 0.1))
            progress = float(point.get("progress", index / final_index))
            rows.append(
                f"{timestamp:.9f},{progress:.9f},"
                + ",".join(f"{float(value):.9f}" for value in positions)
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    @staticmethod
    def _container_workspace_path(path: Path) -> str:
        workspace = (PROJECT_ROOT / "workspaces" / "ur_ros2_ws").resolve()
        try:
            relative = path.resolve().relative_to(workspace)
        except ValueError as exc:
            raise PrintExecutionError(
                "An execution artifact escaped the managed ROS workspace."
            ) from exc
        return str(Path("/home/ubuntu/ur_ros2_ws") / relative)

    @staticmethod
    def _public_print_package(
        manifest: dict[str, Any],
        *,
        moveit_job_id: str = "",
    ) -> dict[str, Any]:
        return {
            "version": manifest["version"],
            "execution_id": manifest["execution_id"],
            "source": manifest["source"],
            "source_sha256": manifest["source_sha256"],
            "review_sha256": manifest["review_sha256"],
            "simulation": manifest.get("simulation") is True,
            "operation_mode": manifest.get("operation_mode", "production_print"),
            "execution_extent": manifest.get("execution_extent", "sample" if
                manifest.get("operation_mode") == "tool0_motion_test" else "full"),
            "heating_enabled": manifest.get("heating_enabled", False),
            "block_count": manifest["block_count"],
            "extrusion_block_count": manifest["extrusion_block_count"],
            "planned_duration_s": manifest["planned_duration_s"],
            "estimated_minimum_duration_s": estimate_minimum_duration_s(manifest),
            "timing_audit": copy.deepcopy(manifest.get("timing_audit", {})),
            "execution_scope": manifest.get("execution_scope", {}),
            "manual_stepper": manifest["manual_stepper"],
            "extruder": manifest["extruder"],
            "extrusion_protocol": manifest["extrusion_protocol"],
            "extruder_config": copy.deepcopy(manifest["extruder_config"]),
            "print_bed_m": manifest["print_bed_m"],
            "print_zero_z_m": manifest.get(
                "print_zero_z_m", manifest["print_bed_m"]["plane_z"]
            ),
            "print_bed_check": manifest["print_bed_check"],
            "tool_center_point_m": manifest["tool_center_point_m"],
            "moveit_job_id": moveit_job_id,
            "ready_for_authorization": manifest.get("simulation") is not True,
            "operator_action_required": manifest.get("operator_action_required", "execute"),
            "pause_semantics": manifest["pause_semantics"],
            "pause_supported": False,
            "stop_semantics": manifest["stop_semantics"],
            "gates": [
                {"code": "immutable_source", "status": "passed"},
                {"code": "full_resolution_path", "status": "passed"},
                {
                    "code": (
                        "tool0_reference"
                        if manifest.get("operation_mode") == "tool0_motion_test"
                        else "measured_tcp"
                    ),
                    "status": "passed",
                },
                *(
                    []
                    if manifest.get("operation_mode") == "tool0_motion_test"
                    else [
                        {
                            "code": "fixed_workcell_tabletop",
                            "status": "passed",
                        }
                    ]
                ),
                {"code": "moveit_collision_review", "status": "passed"},
                {"code": "joint_trajectory_integrity", "status": "passed"},
            ],
        }

    def prepare_print_execution(
        self,
        filename: str,
        content: bytes,
        moveit_job_id: str,
        *,
        execution_extent: str = "sample",
        execution_config: dict[str, Any] | None = None,
        progress_callback: Callable[
            [str, float, int | None, int | None], None
        ] | None = None,
    ) -> dict[str, Any]:
        """Create an immutable package from the exact successful MoveIt review."""

        def report(
            phase: str,
            progress: float,
            completed_units: int | None = None,
            total_units: int | None = None,
        ) -> None:
            if progress_callback is not None:
                progress_callback(
                    phase,
                    max(0.0, min(1.0, float(progress))),
                    completed_units,
                    total_units,
                )

        report("validating_inputs", 0.01)
        _source_sha256, metadata = self._validate_print_preparation_request(
            filename,
            content,
            moveit_job_id,
            execution_extent=execution_extent,
        )
        execution_config = dict(execution_config or self._print_execution_config_snapshot())
        temporary_manifest: Path | None = None
        try:
            report("parsing_gcode", 0.04)
            analysis = parse_gcode(filename, content)
            report("rebuilding_review", 0.10)
            (workcell_snapshot, tabletop_height_candidate,
             print_zero_z_m, print_z_candidate) = self._planning_workcell_snapshot()
            if tabletop_height_candidate or print_z_candidate:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "A tabletop or Print Z candidate cannot be compiled for real motion.",
                )
            automatic_calibration = self._automatic_calibration_for_analysis(
                analysis,
                workcell_snapshot=workcell_snapshot,
                print_zero_z_m=print_zero_z_m,
            )
            review = build_offline_review(
                filename,
                content,
                automatic_calibration,
                maximum_preview_segments=MAX_MOVEIT_WAYPOINTS - 1,
                workcell_snapshot=workcell_snapshot,
                print_zero_z_m=print_zero_z_m,
            )
            with self._calibration_lock:
                installed = self._demo_calibration if self.demo else read_saved_calibration(self._calibration_path)
            if installed is None:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The installed nozzle setup is unavailable.")
            review["installed_profile_sha256"] = installed["profile_sha256"]
            report("loading_moveit_review", 0.18)
            if review["review_sha256"] != metadata["review"]["review_sha256"]:
                raise PrintExecutionError(
                    "This G-code or installed print setup differs from the MoveIt review."
                )
            if self.demo:
                result = demo_moveit_result(review)
                demo_trajectory = result["trajectory"]
                approach_path = self._print_execution_runtime / f"demo-{moveit_job_id}.approach.csv"
                print_path = self._print_execution_runtime / f"demo-{moveit_job_id}.print.csv"
                first_positions = demo_trajectory[0]["positions"]
                self._write_demo_trajectory(
                    approach_path,
                    [
                        {"time_s": 0.05, "positions": first_positions},
                        {"time_s": 0.50, "positions": first_positions},
                    ],
                )
                self._write_demo_trajectory(print_path, demo_trajectory)
            else:
                result = read_moveit_result(
                    metadata["result_path"],
                    review["review_sha256"],
                )
                approach_path = metadata["approach_trajectory_path"]
                print_path = metadata["print_trajectory_path"]

            result["planning_request_id"] = metadata["review_id"]
            request_path = metadata["planning_request_path"]

            self._print_execution_runtime.mkdir(parents=True, exist_ok=True)
            temporary_manifest = self._print_execution_runtime / f"pending-{uuid.uuid4().hex}.json"
            manifest = prepare_execution_manifest(
                review=review,
                moveit_result=result,
                approach_trajectory_path=approach_path,
                print_trajectory_path=print_path,
                planning_request_path=request_path,
                planning_request_container_path=(
                    str(request_path) if self.demo else self._container_workspace_path(request_path)
                ),
                approach_container_path=(
                    str(approach_path)
                    if self.demo
                    else self._container_workspace_path(approach_path)
                ),
                print_container_path=(
                    str(print_path)
                    if self.demo
                    else self._container_workspace_path(print_path)
                ),
                destination=temporary_manifest,
                manual_stepper=execution_config["manual_stepper"],
                stepper_speed=execution_config["stepper_speed"],
                stepper_acceleration=execution_config["stepper_accel"],
                allow_simulation=self.demo,
                execution_extent=execution_extent,
                progress_callback=progress_callback,
            )
            # A synchronous compatibility caller can overlap another request.
            # Never publish a package if its MoveIt authorization changed while
            # the relatively expensive trajectory compiler was running.
            self._require_current_planning_request(metadata)
            (_current_workcell, current_table_candidate,
             current_print_zero, current_print_candidate) = self._planning_workcell_snapshot()
            if (current_table_candidate or current_print_candidate
                    or abs(current_print_zero - print_zero_z_m) > 0.000001):
                raise PrintExecutionError("The tabletop or Print Z changed during preparation. Validate again.")
            destination = self._print_execution_runtime / f"{manifest['execution_id']}.json"
            temporary_manifest.replace(destination)
            status_path = self._print_execution_runtime / f"{manifest['execution_id']}.status.json"
            status_path.unlink(missing_ok=True)
        except (
            GCodeParseError,
            CalibrationValidationError,
            PathReviewError,
            PrintExecutionError,
        ) as exc:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
        finally:
            # The compiler can finish after its reviewed planning request was
            # superseded.  Never retain that large, unpublished pending file.
            if temporary_manifest is not None:
                temporary_manifest.unlink(missing_ok=True)

        public = self._public_print_package(manifest, moveit_job_id=moveit_job_id)
        with self._print_execution_lock:
            self._print_packages[manifest["execution_id"]] = {
                "path": destination,
                "moveit_job_id": moveit_job_id,
                "execution_config_sha256": self._print_execution_config_sha256(
                    execution_config
                ),
                "public": copy.deepcopy(public),
                "status_file_identity": self._print_manifest_file_identity(destination),
            }
        report("complete", 1.0)
        return public

    def _print_execution_config_snapshot(self) -> dict[str, Any]:
        """Freeze the Klipper mapping/limits used by an immutable package."""

        with self._config_lock:
            return {
                "manual_stepper": self.config["manual_stepper"],
                "stepper_speed": self.config["stepper_speed"],
                "stepper_accel": self.config["stepper_accel"],
            }

    @staticmethod
    def _print_execution_config_sha256(config: dict[str, Any]) -> str:
        encoded = json.dumps(
            {
                "manual_stepper": config["manual_stepper"],
                "stepper_speed": config["stepper_speed"],
                "stepper_accel": config["stepper_accel"],
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _validate_print_preparation_request(
        self,
        filename: str,
        content: bytes,
        moveit_job_id: str,
        *,
        execution_extent: str,
    ) -> tuple[str, dict[str, Any]]:
        """Validate the cheap immutable inputs before starting compilation."""

        if not isinstance(execution_extent, str) or execution_extent not in {"sample", "full"}:
            raise ApiError(
                HTTPStatus.BAD_REQUEST,
                "The execution extent must be sample or full.",
            )
        self._require_no_unconfirmed_print_stop()
        with self._height_lock:
            if self._height_draft_m is not None:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The tabletop height is still a candidate. Confirm it physically and rerun MoveIt validation before preparing execution.",
                    code="tabletop_height_unconfirmed",
                )
            if self._print_z_draft_m is not None:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "A legacy Print Z trial is still in progress. Save the desired absolute Print Z again, then rebuild the reviews before preparing execution.",
                    code="print_z_unconfirmed",
                )
            if self._planning_workcell_snapshot()[3]:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The saved Print Z setting is missing or invalid. Re-enter the desired absolute Print Z and rebuild the reviews before preparing execution.",
                    code="print_z_reverification_required",
                )
        if not re.fullmatch(r"[a-f0-9]{12}", moveit_job_id or ""):
            raise ApiError(
                HTTPStatus.BAD_REQUEST,
                "The matching MoveIt validation job id is required.",
            )
        job = self.jobs.get(moveit_job_id)
        if job.kind != "gcode-path-review" or job.state != "succeeded":
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Complete a successful MoveIt path validation before preparing execution.",
            )
        with self._path_review_lock:
            metadata = self._path_review_jobs.get(moveit_job_id)
        if metadata is None:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The MoveIt trajectory metadata is no longer available. Validate again.",
            )
        self._require_fresh_approach_for_moveit_job(moveit_job_id)
        self._require_current_planning_request(metadata)
        self._require_review_placement_current(metadata)
        return hashlib.sha256(content).hexdigest(), metadata

    def _require_review_placement_current(self, metadata: dict[str, Any]) -> None:
        """A completed MoveIt review only authorizes its recorded table and Print Z."""

        table_z = metadata.get("tabletop_surface_z_m")
        print_z = metadata.get("print_zero_z_m")
        if not isinstance(table_z, (int, float)) or not isinstance(print_z, (int, float)):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The MoveIt placement metadata is unavailable. Validate again.",
                code="execution_plan_revoked",
            )
        with self._height_lock:
            self._require_current_print_placement({
                "print_bed_m": {"plane_z": table_z},
                "print_zero_z_m": print_z,
            })

    @staticmethod
    def _public_print_preparation(record: dict[str, Any]) -> dict[str, Any]:
        public = {
            key: record[key]
            for key in (
                "id",
                "state",
                "created_at",
                "started_at",
                "finished_at",
                "execution_extent",
                "moveit_job_id",
                "source_sha256",
                "phase",
                "progress",
                "completed_units",
                "total_units",
                "estimated_remaining_s",
            )
            if record.get(key) is not None
        }
        for key in ("package", "error", "code", "details", "http_status"):
            if record.get(key) is not None:
                public[key] = record[key]
        return public

    def _matching_cached_print_package(
        self,
        *,
        source_sha256: str,
        moveit_job_id: str,
        execution_extent: str,
        execution_config_sha256: str,
    ) -> dict[str, Any] | None:
        with self._print_execution_lock:
            packages = list(self._print_packages.values())
        for metadata in reversed(packages):
            public = metadata.get("public")
            if (
                isinstance(public, dict)
                and metadata.get("moveit_job_id") == moveit_job_id
                and metadata.get("execution_config_sha256")
                == execution_config_sha256
                and public.get("source_sha256") == source_sha256
                and public.get("execution_extent") == execution_extent
            ):
                return public
        return None

    def start_print_preparation(
        self,
        filename: str,
        content: bytes,
        moveit_job_id: str,
        *,
        execution_extent: str = "sample",
    ) -> dict[str, Any]:
        """Start one idempotent background compilation for an exact source."""

        source_sha256, _metadata = self._validate_print_preparation_request(
            filename,
            content,
            moveit_job_id,
            execution_extent=execution_extent,
        )
        # Check again before returning an existing compilation or cached
        # package. A terminal approach failure must not be disguised as a
        # fresh Prepare request, including after execution settings change.
        self._require_fresh_approach_for_moveit_job(moveit_job_id)
        execution_config = self._print_execution_config_snapshot()
        execution_config_sha256 = self._print_execution_config_sha256(
            execution_config
        )
        key = (
            moveit_job_id,
            source_sha256,
            execution_extent,
            execution_config_sha256,
        )
        cached = self._matching_cached_print_package(
            source_sha256=source_sha256,
            moveit_job_id=moveit_job_id,
            execution_extent=execution_extent,
            execution_config_sha256=execution_config_sha256,
        )
        with self._print_preparation_lock:
            existing_id = self._print_preparation_ids_by_key.get(key)
            if existing_id is not None:
                existing = self._print_preparations[existing_id]
                if existing.get("state") not in {"failed", "canceled"}:
                    return self._public_print_preparation(existing)
            if cached is None and any(
                record.get("state") in {"queued", "running"}
                for record in self._print_preparations.values()
            ):
                raise ApiError(
                    HTTPStatus.CONFLICT,
                    "Wait for the current execution package preparation to finish.",
                    code="print_preparation_active",
                )
            preparation_id = uuid.uuid4().hex[:12]
            record: dict[str, Any] = {
                "id": preparation_id,
                "state": "succeeded" if cached is not None else "queued",
                "created_at": utc_now(),
                "started_at": None,
                "finished_at": utc_now() if cached is not None else None,
                "execution_extent": execution_extent,
                "moveit_job_id": moveit_job_id,
                "source_sha256": source_sha256,
                "package": cached,
                "phase": "complete" if cached is not None else "queued",
                "progress": 1.0 if cached is not None else 0.0,
                "completed_units": None,
                "total_units": None,
                "estimated_remaining_s": 0.0 if cached is not None else None,
                "error": None,
                "code": None,
                "details": None,
                "http_status": None,
            }
            self._print_preparations[preparation_id] = record
            self._print_preparation_ids_by_key[key] = preparation_id
            public = self._public_print_preparation(record)

        if cached is None:
            thread = threading.Thread(
                target=self._run_print_preparation,
                args=(preparation_id, filename, bytes(content), moveit_job_id),
                kwargs={
                    "execution_extent": execution_extent,
                    "execution_config": execution_config,
                },
                daemon=True,
                name=f"print-prepare-{preparation_id}",
            )
            thread.start()
        return public

    def _run_print_preparation(
        self,
        preparation_id: str,
        filename: str,
        content: bytes,
        moveit_job_id: str,
        *,
        execution_extent: str,
        execution_config: dict[str, Any],
    ) -> None:
        started_monotonic = time.monotonic()
        with self._print_preparation_lock:
            record = self._print_preparations[preparation_id]
            record["state"] = "running"
            record["started_at"] = utc_now()
            record["phase"] = "validating_inputs"
            record["progress"] = max(0.01, float(record.get("progress", 0.0)))

        def update_progress(
            phase: str,
            progress: float,
            completed_units: int | None = None,
            total_units: int | None = None,
        ) -> None:
            now = time.monotonic()
            with self._print_preparation_lock:
                current = self._print_preparations.get(preparation_id)
                if current is None or current.get("state") != "running":
                    return
                previous_progress = float(current.get("progress", 0.0))
                bounded_progress = max(
                    previous_progress,
                    min(0.999, max(0.0, float(progress))),
                )
                current["phase"] = str(phase)
                current["progress"] = bounded_progress
                if (
                    isinstance(completed_units, int)
                    and isinstance(total_units, int)
                    and 0 <= completed_units <= total_units
                    and total_units > 0
                ):
                    current["completed_units"] = completed_units
                    current["total_units"] = total_units
                else:
                    current["completed_units"] = None
                    current["total_units"] = None
                elapsed = max(0.0, now - started_monotonic)
                current["estimated_remaining_s"] = (
                    round(elapsed * (1.0 - bounded_progress) / bounded_progress, 1)
                    # An instantaneous early callback otherwise produces a
                    # misleading 0-second ETA that remains visible throughout
                    # the following synchronous parse/review phase.
                    if bounded_progress >= 0.02 and elapsed >= 1.0
                    else None
                )
        try:
            package = self.prepare_print_execution(
                filename,
                content,
                moveit_job_id,
                execution_extent=execution_extent,
                execution_config=execution_config,
                progress_callback=update_progress,
            )
        except ApiError as exc:
            with self._print_preparation_lock:
                record.update(
                    state="failed",
                    finished_at=utc_now(),
                    error=exc.message,
                    code=exc.code,
                    details=exc.details,
                    http_status=int(exc.status),
                    phase="failed",
                    estimated_remaining_s=None,
                )
        except Exception as exc:  # pragma: no cover - defensive containment
            with self._print_preparation_lock:
                record.update(
                    state="failed",
                    finished_at=utc_now(),
                    error=f"Internal server error: {exc}",
                    http_status=int(HTTPStatus.INTERNAL_SERVER_ERROR),
                    phase="failed",
                    estimated_remaining_s=None,
                )
        else:
            with self._print_preparation_lock:
                record.update(
                    state="succeeded",
                    finished_at=utc_now(),
                    package=package,
                    phase="complete",
                    progress=1.0,
                    estimated_remaining_s=0.0,
                )

    def get_print_preparation(self, preparation_id: str) -> dict[str, Any]:
        if not re.fullmatch(r"[a-f0-9]{12}", preparation_id or ""):
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid print preparation id.")
        with self._print_preparation_lock:
            record = self._print_preparations.get(preparation_id)
            if record is None:
                raise ApiError(HTTPStatus.NOT_FOUND, "Print preparation was not found.")
            public = self._public_print_preparation(record)
        if public["state"] == "succeeded":
            # A saved placement can change after compilation. Never keep
            # reporting the old package as ready at the preparation endpoint.
            path_reviews = getattr(self, "_path_review_jobs", None)
            if path_reviews is not None:
                with self._path_review_lock:
                    metadata = path_reviews.get(public["moveit_job_id"])
                if metadata is None:
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "The MoveIt trajectory metadata is no longer available. Validate again.",
                        code="execution_plan_revoked",
                    )
                self._require_review_placement_current(metadata)
        return public

    def _require_no_active_print_preparation(self) -> None:
        # A few narrow unit fixtures construct a backend without running the
        # production initializer. Such fixtures have no background registry
        # and therefore cannot have an active preparation.
        lock = getattr(self, "_print_preparation_lock", None)
        preparations = getattr(self, "_print_preparations", None)
        if lock is None or preparations is None:
            return
        with lock:
            active = any(
                record.get("state") in {"queued", "running"}
                for record in preparations.values()
            )
        if active:
            raise ApiError(
                HTTPStatus.CONFLICT,
                "Wait for the current execution package preparation to finish.",
                code="print_preparation_active",
            )

    def _execution_package(self, execution_id: str) -> tuple[Path, dict[str, Any]]:
        if not re.fullmatch(r"[a-f0-9]{16}", execution_id or ""):
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Print Execution package id.")
        with self._print_execution_lock:
            metadata = self._print_packages.get(execution_id)
        path = (
            Path(metadata["path"])
            if metadata is not None
            else self._print_execution_runtime / f"{execution_id}.json"
        )
        try:
            manifest = read_execution_manifest(path)
        except PrintExecutionError as exc:
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, str(exc)) from exc
        if manifest.get("execution_id") != execution_id:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The requested Print Execution package id does not match its contents.",
            )
        return path, manifest

    @staticmethod
    def _print_manifest_file_identity(path: Path) -> tuple[int, ...]:
        value = path.stat()
        return (
            value.st_dev, value.st_ino, value.st_size,
            value.st_mtime_ns, value.st_ctime_ns,
        )

    def _print_status_package(self, execution_id: str) -> dict[str, Any]:
        """Reuse an immutable display summary, never a motion authorization.

        Motion buttons still use _execution_package and its complete artifact
        validation. Status polls only invalidate this observational cache when
        the manifest file changes; a restarted app validates it once on miss.
        """
        if not re.fullmatch(r"[a-f0-9]{16}", execution_id or ""):
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Print Execution package id.")
        # Serialize cold polls so they cannot all revalidate the same package.
        with self._print_execution_lock:
            metadata = self._print_packages.get(execution_id)
            path = (
                Path(metadata["path"])
                if metadata is not None
                else self._print_execution_runtime / f"{execution_id}.json"
            )
            try:
                file_identity = self._print_manifest_file_identity(path)
            except OSError:
                file_identity = None
            if (
                file_identity is not None
                and metadata is not None
                and metadata.get("status_file_identity") == file_identity
                and isinstance(metadata.get("public"), dict)
            ):
                return copy.deepcopy(metadata["public"])

            _path, manifest = self._execution_package(execution_id)
            public = self._public_print_package(
                manifest,
                moveit_job_id=str((metadata or {}).get("moveit_job_id", "")),
            )
            try:
                unchanged = file_identity == self._print_manifest_file_identity(path)
            except OSError:
                unchanged = False
            # Never bind an old summary to a file replaced during validation.
            if file_identity is not None and unchanged:
                updated = dict(metadata or {})
                updated.update(
                    path=path,
                    public=copy.deepcopy(public),
                    status_file_identity=file_identity,
                )
                self._print_packages[execution_id] = updated
            return copy.deepcopy(public)

    @staticmethod
    def _require_operator_action(payload: dict[str, Any], action: str) -> None:
        # A scoped button click supplies intent, not a typed password. Keep the
        # intent explicit so a missing field or another route's action cannot
        # silently authorize a motion or service restart.
        if payload.get("operator_action") != action:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                f"Use the {action} button to request this operation.",
                code="operator_action_required",
            )

    @staticmethod
    def _require_action_and_safety(payload: dict[str, Any], action: str) -> None:
        # The executor enforces immutable role-specific smooth-profile limits:
        # 0.05 rad/s for the non-extruding approach and 0.03 rad/s for print.
        # Do not reuse the legacy slider<20% confirmation: coordinated timing
        # separately requires fresh, unchanged UR scaling and never sets it.
        required = ["emergency_stop", "workspace_clear", "reviewed_low_speed", "wired_network"]
        if not all(payload.get(key) is True for key in required):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Complete every live safety confirmation before real motion.",
            )
        if payload.get("external_control_playing") is not True:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Confirm that the External Control program is playing.",
            )
        StationBackend._require_operator_action(payload, action)

    def _require_print_stack(self) -> None:
        if self.demo:
            return
        status = self.ros_status()
        if (
            not status.get("driver_started")
            or not status.get("external_control")
            or not status.get("moveit_ready")
            or not status.get("klipper_bridge_ready")
            or status.get("unstable")
        ):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "UR driver, External Control, MoveIt, and the ROS Klipper bridge "
                "must all be stable before Print Execution.",
            )

    def print_interlock_status(self) -> dict[str, Any]:
        lifecycle = getattr(self, "lifecycle", None)
        if (
            lifecycle is not None
            and lifecycle.runtime_dir == self._print_execution_runtime
        ):
            return lifecycle.inspect()

        # Some diagnostic/test harnesses relocate only the runtime directory.
        # Keep the safety parser centralized in RuntimeLifecycle even there.
        return RuntimeLifecycle(
            runtime_dir=self._print_execution_runtime,
            stop_owned_jobs=lambda _reason: {},
            stop_owned_runtime=lambda _reason: {},
            owned_runtime_stopped=lambda: False,
            stop_and_verify_ur=lambda: {"stopped": False},
            verify_klipper_stopped=lambda: {"stopped": False},
        ).inspect()

    def _lifecycle_stop_jobs(self, reason: str) -> dict[str, Any]:
        return self.jobs.stop_all(
            reason=reason,
            close=reason != "stop-recovery",
        )

    def _lifecycle_stop_runtime(self, reason: str) -> dict[str, Any]:
        if self.demo:
            return {"stopped": True, "simulation": True, "reason": reason}
        command = platform_operation_command(
            "stop-ros",
            PROJECT_ROOT / "scripts" / "stop.sh",
            demo=False,
        )
        result = run_capture(
            command,
            timeout=120.0,
            env=self._environment(),
        )
        if not result["ok"]:
            raise RuntimeError(result["output"] or "ROS container stop failed")
        executor_cleanup = stop_project_executor_processes()
        if executor_cleanup["remaining_pids"]:
            raise RuntimeError(
                "Project executor processes remain: "
                + ", ".join(str(pid) for pid in executor_cleanup["remaining_pids"])
            )
        return {
            "stopped": True,
            "reason": reason,
            "output": result["output"][-4000:],
            "executor_cleanup": executor_cleanup,
        }

    def _lifecycle_runtime_stopped(self) -> bool:
        if self.demo:
            return True
        result = run_capture(
            [
                "docker",
                "compose",
                "-f",
                str(COMPOSE_FILE),
                "ps",
                "--status",
                "running",
                "--services",
            ],
            timeout=8.0,
        )
        return (
            result["ok"]
            and "ros2" not in result["output"].splitlines()
            and not project_executor_processes()
        )

    def _lifecycle_stop_and_verify_ur(self) -> dict[str, Any]:
        if self.demo:
            return {
                "stopped": True,
                "program_state": "STOPPED",
                "stationary": True,
                "stationary_sample_count": 6,
                "simulation": True,
                "observed_at": utc_now(),
            }
        host = self.config["ur_robot_ip"]
        dashboard = dashboard_stop_and_verify(host)
        if dashboard.get("stopped") is not True:
            return dashboard
        stationary = verify_stationary_rtde(host)
        # A new program could have been started while the joints were sampled.
        # Contain once more before accepting the recovery receipt.
        final_dashboard = dashboard_stop_and_verify(host)
        return {
            **final_dashboard,
            "stationary": stationary.get("stationary") is True,
            "stationary_sample_count": stationary.get("sample_count", 0),
            "stationary_evidence": stationary,
            "initial_dashboard": dashboard,
        }

    def _lifecycle_verify_klipper(self) -> dict[str, Any]:
        if self.demo:
            return {
                "stopped": True,
                "print_state": "standby",
                "simulation": True,
                "observed_at": utc_now(),
            }
        return klipper_stopped_status(self.config["klipper_url"])

    def _invalidate_recovered_executions(self, execution_ids: set[str]) -> None:
        with self._print_execution_lock:
            for execution_id in execution_ids:
                self._print_packages.pop(execution_id, None)
        with self._print_preparation_lock:
            stale_keys = [
                key
                for key, preparation_id in self._print_preparation_ids_by_key.items()
                if (
                    self._print_preparations.get(preparation_id, {})
                    .get("package", {})
                    .get("execution_id")
                    in execution_ids
                )
            ]
            for key in stale_keys:
                preparation_id = self._print_preparation_ids_by_key.pop(key)
                self._print_preparations.pop(preparation_id, None)

    @staticmethod
    def _lifecycle_api_error(exc: LifecycleBlocked) -> ApiError:
        status = (
            HTTPStatus.PRECONDITION_FAILED
            if exc.code == "operator_action_required"
            else HTTPStatus.CONFLICT
        )
        return ApiError(status, exc.message, code=exc.code, details=exc.details)

    def recover_print_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        maintenance_token = self.jobs.begin_maintenance("stop-recovery")
        try:
            return self.lifecycle.recover(
                operator_action=str(payload.get("operator_action", "")),
            )
        except LifecycleBlocked as exc:
            raise self._lifecycle_api_error(exc) from exc
        finally:
            self.jobs.end_maintenance(maintenance_token)

    def shutdown_runtime(self, *, reason: str = "application-exit") -> dict[str, Any]:
        if reason not in {"application-exit", "backend-exit", "system-shutdown"}:
            raise ApiError(HTTPStatus.BAD_REQUEST, "Unsupported shutdown reason.")
        self._closing = True
        maintenance_token = self.jobs.begin_maintenance(reason)
        try:
            return self.lifecycle.shutdown(reason=reason)
        except LifecycleBlocked as exc:
            raise self._lifecycle_api_error(exc) from exc
        finally:
            self.jobs.end_maintenance(maintenance_token)

    def _require_execution_planning_authorization(
        self,
        manifest: dict[str, Any],
    ) -> None:
        """Require the immutable package's exact MoveIt request to remain current."""

        if self.demo:
            return
        review_sha256 = str(manifest.get("review_sha256", ""))
        request = manifest.get("planning_request")
        if (
            not re.fullmatch(r"[a-f0-9]{64}", review_sha256)
            or not isinstance(request, dict)
            or not re.fullmatch(r"[a-f0-9]{64}", str(request.get("sha256", "")))
        ):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The execution package has no verifiable MoveIt authorization. Validate again.",
                code="execution_plan_revoked",
            )
        expected_path = (
            PATH_REVIEW_RUNTIME / f"{review_sha256}.request.json"
        ).resolve()
        try:
            request_path = Path(str(request.get("path", ""))).resolve()
            request_bytes = request_path.read_bytes()
        except OSError as exc:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The MoveIt authorization is no longer available. Validate again.",
                code="execution_plan_revoked",
            ) from exc
        if (
            request_path != expected_path
            or hashlib.sha256(request_bytes).hexdigest() != request["sha256"]
        ):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "This execution package was revoked. Run a fresh MoveIt review and prepare again.",
                code="execution_plan_revoked",
            )

    def _require_no_unconfirmed_print_stop(self) -> None:
        # Durable across app restarts and across different execution packages.
        # Preparing another package must not erase an unresolved stop record.
        if self.print_interlock_status()["locked"]:
            raise ApiError(
                HTTPStatus.CONFLICT,
                "A previous UR/Klipper stop is unconfirmed. New motion is locked. "
                "Keep both devices stopped and use Verify stop and recover before creating a fresh plan.",
                code="print_stop_unconfirmed",
            )

    def _require_current_print_placement(self, manifest: dict[str, Any]) -> None:
        """Check the placement while the caller holds the height lock."""

        if self._height_draft_m is not None:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Finish or discard the tabletop-height trial before moving the robot.",
            )
        if self._print_z_draft_m is not None:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "A legacy Print Z trial is still in progress. Save the desired absolute Print Z again and rebuild the path review and execution package.",
                code="print_z_unconfirmed",
            )
        if self._planning_workcell_snapshot()[3]:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The saved Print Z selection is missing or invalid. Re-enter Print Z and rebuild the path review and execution package.",
                code="print_z_reverification_required",
            )
        current_height = load_workcell_geometry()["tabletop"]["surface_z_m"]
        current_print_zero = self._planning_workcell_snapshot()[2]
        if abs(float(manifest["print_bed_m"]["plane_z"]) - current_height) > 0.000001:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The tabletop height changed. Rebuild the path review and execution package.",
                code="workcell_height_changed",
            )
        if abs(float(manifest.get("print_zero_z_m", manifest["print_bed_m"]["plane_z"]))
               - current_print_zero) > 0.000001:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The selected Print Z position changed. Rebuild the path review and execution package.",
                code="print_z_changed",
            )

    def _require_native_extrusion_ready(self, manifest: dict[str, Any]) -> None:
        """Read-only gate before either coordinated channel can be queued."""
        if self.demo:
            return
        if manifest.get("extrusion_protocol") != "klipper-native-extruder-v1":
            raise ApiError(HTTPStatus.CONFLICT, "Prepare a new native-extruder execution package.")
        result = self.check_klipper()
        live = result.get("extruder", {})
        if not result.get("ok") or live.get("name") != manifest.get("extruder"):
            raise ApiError(HTTPStatus.PRECONDITION_FAILED, "The selected native extruder is not ready.")
        if live.get("can_extrude") is not True:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                f"Extrusion is blocked at {live.get('temperature', 'unknown')} °C. "
                "Prepare the hotend at the material's required temperature before starting.",
                code="extruder_too_cold",
            )
        expected = manifest.get("extruder_config", {})
        for key in ("nozzle_diameter", "filament_diameter", "rotation_distance", "min_extrude_temp"):
            try:
                matches = math.isclose(float(live[key]), float(expected[key]), rel_tol=0.0, abs_tol=1.0e-8)
            except (KeyError, TypeError, ValueError):
                matches = False
            if not matches:
                raise ApiError(HTTPStatus.CONFLICT, f"Native extruder {key} changed. Review and prepare again.", code="extruder_config_changed")
        limits = manifest.get("stepper_limits", {})
        for live_key, frozen_key in (("max_extrude_only_velocity", "maximum_speed"),
                                     ("max_extrude_only_accel", "acceleration"),
                                     ("max_extrude_only_distance", "max_extrude_only_distance")):
            try:
                actual, frozen = float(live[live_key]), float(limits[frozen_key])
                matches = (actual >= frozen if frozen_key == "maximum_speed" else abs(actual-frozen) <= 1.0e-8)
            except (KeyError, TypeError, ValueError):
                matches = False
            if not matches:
                raise ApiError(HTTPStatus.CONFLICT, "Native extruder motion limits changed. Prepare again.", code="extruder_limits_changed")
        try:
            overrides = [float(live[key]) for key in ("speed_factor", "extrude_factor")]
            overrides_ok = all(math.isfinite(value) and abs(value - 1.0) <= 1.0e-9 for value in overrides)
        except (KeyError, TypeError, ValueError):
            overrides_ok = False
        if not overrides_ok:
            raise ApiError(HTTPStatus.CONFLICT, "Restore Klipper speed and flow overrides to 100% before starting.", code="extruder_override_changed")

    def start_print_operation(
        self,
        payload: dict[str, Any],
        *,
        mode: str,
    ) -> Job:
        if mode not in {"move-to-start", "execute"}:
            raise ApiError(HTTPStatus.BAD_REQUEST, "Unsupported Print Execution operation.")
        self._require_no_active_print_preparation()
        self._require_no_unconfirmed_print_stop()
        execution_id = str(payload.get("execution_id", ""))
        _path, manifest = self._execution_package(execution_id)
        if mode == "move-to-start":
            self._require_fresh_approach_review(execution_id)
        with self._height_lock:
            self._require_current_print_placement(manifest)
        if not self.demo:
            saved_profile = read_saved_calibration(self._calibration_path)
            if (saved_profile is None or
                    manifest.get("installed_profile_sha256", manifest.get("profile_sha256"))
                    != saved_profile["profile_sha256"]):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The installed nozzle or calibration changed. Rebuild the path review and execution package.",
                    code="print_setup_changed",
                )
        if manifest.get("operation_mode") == "tool0_motion_test" and not self.demo:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The installed printhead extends below tool0. Legacy Tool0-on-table motion is locked; review the nozzle TCP first.",
                code="legacy_tool0_motion_locked",
            )
        self._require_execution_planning_authorization(manifest)
        # The operator's explicit production start uses the same immutable
        # package, reviewed-start, live-readiness and stop guards as cold mode.
        # Physical synchronization is disclosed, not claimed as certified.
        self._require_action_and_safety(payload, mode)
        self._require_print_stack()
        if mode == "execute":
            self._require_native_extrusion_ready(manifest)
        if manifest.get("simulation") is True and not self.demo:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "A simulated execution package cannot control real hardware.",
            )
        status_path = self._print_execution_runtime / f"{execution_id}.status.json"
        if mode == "execute":
            try:
                prior = json.loads(status_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Move the robot to the reviewed print start before starting the print.",
                ) from exc
            if prior.get("state") != "at-start":
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The reviewed move-to-start operation has not completed.",
                )
        command = [sys.executable, str(PRINT_EXECUTION_HOST), execution_id, mode]
        title = (
            "Moving UR to the reviewed motion start"
            if mode == "move-to-start"
            and manifest.get("operation_mode") == "tool0_motion_test"
            else "Moving UR to the reviewed print start"
            if mode == "move-to-start"
            else "Executing cold software-coordinated Tool0 motion test"
            if manifest.get("operation_mode") == "tool0_motion_test"
            else "Executing the reviewed UR/Klipper print path (heater controlled separately)"
        )
        demo_lines = [
            f"Demo: {title}.",
            "The immutable trajectory checksum was verified.",
            "Demo completed without contacting ROS, UR, Moonraker, or Klipper.",
        ]
        with self._calibration_lock:
            if not self.demo:
                latest_profile = read_saved_calibration(self._calibration_path)
                if (latest_profile is None or latest_profile["profile_sha256"] !=
                        manifest.get("installed_profile_sha256", manifest.get("profile_sha256"))):
                    raise ApiError(
                        HTTPStatus.PRECONDITION_FAILED,
                        "The nozzle setup changed before motion started. Rebuild the review.",
                        code="print_setup_changed",
                    )
            # Hold the same lock used by Print Z and tabletop edits until the
            # motion job is registered. A height change during live preflight
            # must not let a previously checked package start afterward.
            with self._height_lock:
                self._require_current_print_placement(manifest)
                if self.demo:
                    state_name = "at-start" if mode == "move-to-start" else "completed"
                    status_path.parent.mkdir(parents=True, exist_ok=True)
                    status_path.write_text(
                        json.dumps(
                            {
                                "version": 1,
                                "execution_id": execution_id,
                                "state": state_name,
                                "mode": mode,
                                "progress": 1.0,
                                "current_block": manifest["block_count"] if mode == "execute" else 0,
                                "block_count": manifest["block_count"],
                                "message": f"Demo {mode} completed.",
                                "updated_at": time.time(),
                            },
                            indent=2,
                        )
                        + "\n",
                        encoding="utf-8",
                    )
                job = self.jobs.start(
                    f"print-{mode}",
                    command,
                    motion=True,
                    demo_lines=demo_lines if self.demo else None,
                )
        with self._print_execution_lock:
            self._print_jobs[job.id] = {"execution_id": execution_id, "mode": mode}
        return job

    def set_print_pause(self, execution_id: str, *, paused: bool) -> dict[str, Any]:
        if not re.fullmatch(r"[a-f0-9]{16}", execution_id or ""):
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Print Execution package id.")
        self._execution_package(execution_id)
        raise ApiError(
            HTTPStatus.CONFLICT,
            "Continuous Print Execution cannot pause or resume between timeline "
            "segments. Use Stop UR + Klipper to end motion.",
        )

    def stop_print_operation(self, execution_id: str) -> Job:
        if not re.fullmatch(r"[a-f0-9]{16}", execution_id or ""):
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Print Execution package id.")
        with self._print_execution_lock:
            active_job_id = next(
                (
                    job_id
                    for job_id, metadata in self._print_jobs.items()
                    if metadata["execution_id"] == execution_id
                    and self.jobs.get(job_id).state in {"queued", "running"}
                ),
                None,
            )
        if active_job_id is None:
            raise ApiError(HTTPStatus.CONFLICT, "No active Print Execution operation was found.")
        return self.jobs.cancel(active_job_id)

    def _print_runtime_status(self, execution_id: str) -> dict[str, Any] | None:
        status_path = self._print_execution_runtime / f"{execution_id}.status.json"
        if status_path.is_file() and status_path.stat().st_size <= 256_000:
            try:
                value = json.loads(status_path.read_text(encoding="utf-8"))
                if isinstance(value, dict) and value.get("execution_id") == execution_id:
                    return value
            except (OSError, json.JSONDecodeError):
                pass
        return None

    @staticmethod
    def _approach_requires_revalidation(runtime: dict[str, Any] | None) -> bool:
        return bool(runtime and runtime.get("mode") == "move-to-start"
                    and runtime.get("state") in {"failed", "canceled"})

    def _require_fresh_approach_review(self, execution_id: str) -> None:
        if self._approach_requires_revalidation(self._print_runtime_status(execution_id)):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "The move to print start failed or was canceled. Run Validate with MoveIt "
                "again from the current robot position, then Prepare a new execution package.",
                code="requires_revalidation",
            )

    def _require_fresh_approach_for_moveit_job(self, moveit_job_id: str) -> None:
        with self._print_execution_lock:
            execution_ids = [execution_id for execution_id, package in self._print_packages.items()
                             if package.get("moveit_job_id") == moveit_job_id]
        for execution_id in execution_ids:
            self._require_fresh_approach_review(execution_id)

    def get_print_execution_status(self, execution_id: str) -> dict[str, Any]:
        package = self._print_status_package(execution_id)
        runtime_status = self._print_runtime_status(execution_id)
        with self._print_execution_lock:
            matching_jobs = [
                (job_id, metadata)
                for job_id, metadata in self._print_jobs.items()
                if metadata["execution_id"] == execution_id
            ]
        job = self.jobs.get(matching_jobs[-1][0]).public() if matching_jobs else None
        return {
            "package": package,
            "runtime": runtime_status,
            "job": job,
            "requires_revalidation": self._approach_requires_revalidation(runtime_status),
            **({"code": "requires_revalidation"}
               if self._approach_requires_revalidation(runtime_status) else {}),
        }

    def require_safety(self, payload: dict[str, Any], *, motion: bool = False) -> None:
        if motion:
            self._require_no_unconfirmed_print_stop()
        required = ["emergency_stop", "workspace_clear", "speed_under_20", "wired_network"]
        if not all(payload.get(key) is True for key in required):
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Complete every safety confirmation before continuing.",
            )
        if motion:
            if payload.get("external_control_playing") is not True:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Confirm that the External Control program is playing.",
                )
            self._require_operator_action(payload, "sync-test")

    def check_ur(self) -> dict[str, Any]:
        robot_ip = self.config["ur_robot_ip"]
        if self.demo:
            route = {"interface": "en7", "address": "192.168.0.100", "service": "USB Ethernet"}
            checks = [
                check_item("route", "Wired route", True, "USB Ethernet (en7) · 192.168.0.100"),
                check_item("ping", "Ping", True, "Reachable in 1.2 ms."),
                check_item("dashboard", "Dashboard", True, "Robotmode: RUNNING"),
                check_item("rtde", "RTDE", True, "Port 30004 responded in 2 ms."),
            ]
            return {"ok": True, "summary": "UR3e network connection is ready.", "checks": checks, "route": route}

        route = detect_route(robot_ip)
        service = route.get("service", "")
        interface = route.get("interface", "")
        address = route.get("address", "")
        route_known = bool(interface and address)
        wired = route_known and service.casefold() not in {"wi-fi", "virtual"}
        route_detail = " · ".join(part for part in (service or "Unknown service", interface, address) if part)
        ping_ok, ping_detail = ping_probe(robot_ip)
        dashboard_ok, dashboard_detail = dashboard_query(robot_ip)
        rtde_ok, rtde_detail = tcp_probe(robot_ip, 30004)
        checks = [
            check_item(
                "route",
                "Wired route",
                wired,
                route_detail or "No route to the robot was detected.",
                severity="warn" if route_known and not wired else None,
            ),
            check_item("ping", "Ping", ping_ok, ping_detail),
            check_item(
                "dashboard",
                "Dashboard",
                dashboard_ok,
                dashboard_detail,
                severity="warn" if not dashboard_ok else None,
            ),
            check_item("rtde", "RTDE", rtde_ok, rtde_detail),
        ]
        ok = wired and ping_ok and rtde_ok
        summary = "UR3e network connection is ready." if ok else "UR3e connection needs attention."
        return {"ok": ok, "summary": summary, "checks": checks, "route": route}

    def check_klipper(self) -> dict[str, Any]:
        base = self.config["klipper_url"]
        stepper = self.config["manual_stepper"]
        if self.demo:
            extruder = {
                "name": stepper, "temperature": 200.0, "target": 200.0,
                "can_extrude": True, "min_extrude_temp": 170.0,
                "nozzle_diameter": 0.2, "filament_diameter": 1.75,
                "max_extrude_only_velocity": 5.0, "max_extrude_only_accel": 20.0,
                "max_extrude_only_distance": 50.0,
                "speed_factor": 1.0, "extrude_factor": 1.0,
            }
            checks = [
                check_item("moonraker", "Moonraker", True, "API responded in 4 ms."),
                check_item("klipper", "Klipper", True, "Printer state: ready"),
                check_item("stepper", "Extruder", True, f"[{stepper}] found."),
                check_item("extruder_temperature", "Extruder temperature", True, "200.0 °C · ready for extrusion (simulated)."),
            ]
            return {"ok": True, "summary": "Klipper is ready.", "checks": checks, "state": "ready", "extruder": extruder}

        checks: list[dict[str, Any]] = []
        try:
            started = time.monotonic()
            info = urlopen_json(base + "/printer/info")
            elapsed = (time.monotonic() - started) * 1000
            checks.append(check_item("moonraker", "Moonraker", True, f"API responded in {elapsed:.0f} ms."))
            result = info.get("result", {})
            state = str(result.get("state", "unknown"))
            message = str(result.get("state_message", "")).strip()
            checks.append(
                check_item(
                    "klipper",
                    "Klipper",
                    state == "ready",
                    f"Printer state: {state}" + (f" · {message}" if message else ""),
                )
            )
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            checks.append(check_item("moonraker", "Moonraker", False, f"API request failed: {exc}"))
            checks.append(check_item("klipper", "Klipper", False, "Printer state is unavailable."))
            state = "unavailable"

        stepper_ok = False
        extruder: dict[str, Any] = {"name": stepper, "can_extrude": False}
        temperature_ok = False
        temperature_detail = "Extruder status is unavailable."
        if checks[0]["ok"]:
            try:
                config = urlopen_json(base + "/printer/objects/query?configfile&gcode_move&" + stepper)
                status = config.get("result", {}).get("status", {})
                configfile = status.get("configfile", {})
                config_data = configfile.get("settings") or configfile.get("config", {})
                stepper_ok = stepper in config_data
                stepper_detail = (
                    f"[{stepper}] found; E uses filament millimetres."
                    if stepper_ok
                    else f"[{stepper}] is missing from printer.cfg. A manual_stepper cannot replace it."
                )
                if stepper_ok:
                    settings = config_data[stepper]
                    runtime = status.get(stepper, {})
                    for name in ("nozzle_diameter", "filament_diameter", "rotation_distance", "max_extrude_only_velocity", "max_extrude_only_accel"):
                        number = float(settings[name])
                        if not math.isfinite(number) or number <= 0.0:
                            raise ValueError(f"Invalid {stepper}.{name}")
                        extruder[name] = number
                    max_distance = float(settings.get("max_extrude_only_distance", 50.0))
                    if not math.isfinite(max_distance) or max_distance <= 0.0:
                        raise ValueError("Invalid maximum extrusion-only distance")
                    extruder["max_extrude_only_distance"] = max_distance
                    gcode_move = status.get("gcode_move", {})
                    for name in ("speed_factor", "extrude_factor"):
                        factor = float(gcode_move[name])
                        if not math.isfinite(factor) or factor <= 0.0:
                            raise ValueError(f"Invalid gcode_move.{name}")
                        extruder[name] = factor
                    min_temp = float(settings.get("min_extrude_temp", 170.0))
                    temperature = float(runtime["temperature"])
                    target = float(runtime["target"])
                    if not all(math.isfinite(number) for number in (min_temp, temperature, target)):
                        raise ValueError("Extruder temperature metadata is not finite")
                    extruder.update({
                        "temperature": temperature, "target": target,
                        "min_extrude_temp": min_temp,
                        "can_extrude": runtime.get("can_extrude") is True and temperature >= min_temp,
                    })
                    temperature_ok = isinstance(runtime.get("can_extrude"), bool)
                    temperature_detail = (
                        f"{temperature:.1f} °C · target {target:.1f} °C · "
                        + ("ready for extrusion." if extruder["can_extrude"] else f"extrusion is blocked; minimum {min_temp:g} °C. Prepare the hotend separately.")
                    )
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                stepper_detail = f"Could not read Klipper configuration: {exc}"
                temperature_detail = f"Could not verify native extruder status: {exc}"
                stepper_ok = False
        else:
            stepper_detail = "Skipped because Moonraker is unavailable."
        checks.append(check_item("stepper", "Extruder", stepper_ok, stepper_detail))
        checks.append(check_item("extruder_temperature", "Extruder temperature", temperature_ok, temperature_detail,
                                 severity="warn" if temperature_ok and not extruder["can_extrude"] else None))
        ok = all(item["ok"] for item in checks)
        return {
            "ok": ok,
            "summary": "Klipper is ready." if ok else "Klipper connection needs attention.",
            "checks": checks,
            "state": state,
            "extruder": extruder,
        }

    def ros_status(self) -> dict[str, Any]:
        if self.demo:
            return {
                "docker_available": True,
                "container_running": True,
                "driver_started": True,
                "external_control": True,
                "external_control_live": {
                    "available": True,
                    "ready": True,
                    "program_running": True,
                    "joint_state_fresh": True,
                    "reason": "External Control is live and ready.",
                },
                "moveit_ready": True,
                "moveit_running": True,
                "klipper_bridge_ready": True,
                "klipper_bridge_status": {
                    "available": True,
                    "ready": True,
                    "moonraker_reachable": True,
                    "printer_ready": True,
                    "stepper_found": True,
                    "action_active": False,
                    "reason": "ROS Klipper bridge is ready.",
                },
                "host_ports_ready": True,
                "missing_host_ports": [],
                "unstable": False,
                "summary": "Demo ROS stack is ready.",
            }

        docker = run_capture(["docker", "version", "--format", "{{.Server.Version}}"], timeout=4.0)
        container_running = False
        if docker["ok"]:
            services = run_capture(
                [
                    "docker",
                    "compose",
                    "-f",
                    str(COMPOSE_FILE),
                    "ps",
                    "--status",
                    "running",
                    "--services",
                ],
                timeout=5.0,
            )
            container_running = "ros2" in services["output"].splitlines()

        driver_state = parse_driver_log(tail_text(DRIVER_LOG))
        moveit_text = tail_text(MOVEIT_LOG)
        # Detached launch logs remain on disk after the container stops. Gate
        # every readiness flag with the live container so stale success lines
        # can never make an offline stack look ready.
        driver_process_started = (
            container_running and driver_process_running() and driver_state["system_started"]
        )
        host_ports = (
            host_ur_ports_status(self.config.get("ros_pc_ip", ""))
            if driver_process_started
            else {"ok": False, "missing": []}
        )
        driver_started = driver_process_started and host_ports["ok"]
        live_external = (
            live_external_control_status()
            if driver_started
            else {
                "available": False,
                "ready": False,
                "program_running": None,
                "joint_state_fresh": False,
                "reason": "The UR driver is not ready.",
            }
        )
        # A historical reverse-interface log line is useful for diagnosis but
        # is never sufficient to permit real motion. Current program state and
        # a fresh joint-state sample must both be observed during this check.
        external_control = driver_started and live_external["ready"]
        # Planning is a separate, non-motion service. It can remain healthy
        # after External Control is stopped, and plan-only dry runs should not
        # be misreported as unavailable for that reason. Real synchronized
        # motion still requires external_control in sync_test().
        moveit_running = container_running and moveit_process_running()
        moveit_ready = moveit_running and "You can start planning now!" in moveit_text
        klipper_bridge_status = (
            live_klipper_bridge_status(self.config["manual_stepper"])
            if container_running
            else {
                "available": False,
                "ready": False,
                "moonraker_reachable": False,
                "printer_ready": False,
                "stepper_found": False,
                "action_active": False,
                "reason": "The ROS container is stopped.",
            }
        )
        klipper_bridge_ready = klipper_bridge_status["ready"]
        unstable = container_running and driver_state["unstable"]
        rtde_conflict = container_running and driver_state["rtde_conflict"]
        summary = "ROS/MoveIt is ready."
        if not docker["ok"]:
            summary = "Docker is unavailable."
        elif not container_running:
            summary = "The ROS container is stopped."
        elif rtde_conflict:
            summary = "RTDE is occupied. Disable robot Fieldbus or close the other RTDE client."
        elif driver_process_started and not host_ports["ok"]:
            ports = ", ".join(str(port) for port in host_ports["missing"])
            summary = (
                f"UR host port forwarding is unavailable for {ports}. "
                "Restart ROS to repair the container network."
            )
        elif not driver_started:
            summary = "The UR driver is not ready."
        elif unstable:
            summary = "The UR driver reported an unstable connection."
        elif not external_control:
            summary = live_external["reason"]
        elif not klipper_bridge_ready:
            summary = klipper_bridge_status["reason"]
        elif not moveit_ready:
            summary = "MoveIt has not started yet."
        return {
            "docker_available": docker["ok"],
            "container_running": container_running,
            "driver_started": driver_started,
            "host_ports_ready": host_ports["ok"],
            "missing_host_ports": host_ports["missing"],
            "external_control": external_control,
            "external_control_live": live_external,
            "moveit_ready": moveit_ready,
            "moveit_running": moveit_running,
            "klipper_bridge_ready": klipper_bridge_ready,
            "klipper_bridge_status": klipper_bridge_status,
            "rtde_conflict": rtde_conflict,
            "unstable": unstable,
            "summary": summary,
        }

    def _environment(self) -> dict[str, str]:
        env = os.environ.copy()
        env.update(
            {
                "ROBOT_IP": self.config["ur_robot_ip"],
                "UR_UPDATE_RATE": str(self.config["ur_update_rate"]),
                "MOONRAKER_URL": self.config["klipper_url"],
                "KLIPPER_STEPPER": self.config["manual_stepper"],
            }
        )
        if self.config["ros_pc_ip"]:
            env["ROS_PC_IP"] = self.config["ros_pc_ip"]
            env["UR_REVERSE_BIND_IP"] = self.config["ros_pc_ip"]
        return env

    def start_ros(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload)
        if not self.demo:
            ur = self.check_ur()
            if not ur["ok"]:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "UR3e network checks must pass first.")
            klipper = self.check_klipper()
            if not klipper["ok"]:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Klipper read-only checks must pass before starting the ROS bridge.",
                )
        return self._queue_ros_launch(
            "start-ros",
            [
                "Checking local runtime…",
                "Starting ROS 2 container…",
                f"Launching UR3e driver at {self.config['ur_update_rate']} Hz…",
                "Starting the ROS Klipper bridge…",
                "UR driver started. Press Play on the External Control program.",
            ],
        )

    def _queue_ros_launch(self, kind: str, demo_lines: list[str]) -> Job:
        command = platform_operation_command(
            "start-ros",
            PROJECT_ROOT / "scripts" / "launch_real_robot_detached.sh",
            demo=self.demo,
        )
        env = self._environment()
        # Preserve the proven command-line workflow: do not invite the operator
        # to press Play until the driver has acquired RTDE, started all
        # controllers, opened the reverse ports, and remained stable.
        env["WAIT_FOR_READY"] = "true"
        return self.jobs.start(kind, command, env=env, demo_lines=demo_lines if self.demo else None)

    def restart_ros(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload)
        self._require_operator_action(payload, "restart-ros")
        if not self.demo:
            ur = self.check_ur()
            if not ur["ok"]:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "UR3e network checks must pass before restarting ROS.")
            klipper = self.check_klipper()
            if not klipper["ok"]:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "Klipper read-only checks must pass before restarting the ROS bridge.",
                )
        return self._queue_ros_launch(
            "restart-ros",
            [
                "Stopping the current ROS driver and MoveIt…",
                "Clearing stale ROS processes and connection state…",
                f"Restarting the UR3e driver at {self.config['ur_update_rate']} Hz…",
                "ROS driver is ready. Reconnect External Control, then start MoveIt.",
            ],
        )

    def wait_external_control(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload)
        if not self.demo:
            status = self.ros_status()
            if status["rtde_conflict"]:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "RTDE is occupied. Disable robot Fieldbus or close the other RTDE client, then start ROS again.",
                )
            if not status.get("host_ports_ready", False):
                ports = ", ".join(str(port) for port in status.get("missing_host_ports", [])) or "50001-50004"
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    f"UR host ports {ports} are not available. Restart ROS to repair "
                    "the container network before pressing Play.",
                )
            if not status["driver_started"] or status["unstable"]:
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The UR driver must be ready and stable before External Control can start.",
                )
        command = platform_operation_command(
            "wait-external-control",
            PROJECT_ROOT / "scripts" / "wait_for_external_control.sh",
            demo=self.demo,
        )
        env = self._environment()
        env["TIMEOUT_SECONDS"] = "90"
        lines = [
            "Checking current External Control state…",
            "Robot connected to reverse interface.",
            "robot_program_running: true",
            "Joint state stream received.",
        ]
        return self.jobs.start(
            "external-control",
            command,
            env=env,
            demo_lines=lines if self.demo else None,
        )

    def start_moveit(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload)
        if not self.demo:
            status = self.ros_status()
            if (
                not status["external_control"]
                or not status.get("klipper_bridge_ready", False)
                or status["unstable"]
            ):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "External Control and the ROS Klipper bridge must be ready before MoveIt starts.",
                )
        command = platform_operation_command(
            "start-moveit",
            PROJECT_ROOT / "scripts" / "launch_moveit_detached.sh",
            demo=self.demo,
        )
        lines = [
            "Loading the calibrated UR3e model…",
            "Starting MoveIt planning scene…",
            "move_group is ready.",
        ]
        return self.jobs.start("start-moveit", command, env=self._environment(), demo_lines=lines if self.demo else None)

    def dry_run(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload)
        config = validate_config({**self.config, **payload.get("settings", {})})
        if not self.demo:
            klipper = self.check_klipper()
            ros = self.ros_status()
            if not klipper["ok"]:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Klipper must be ready before the combined dry run.")
            if not ros.get("klipper_bridge_ready", False):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The ROS Klipper bridge must be ready before the combined dry run.",
                )
            if not ros["moveit_ready"]:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "MoveIt must be ready before planning.")
            self.validate_cartesian_test_over_print_bed(config)
        command = [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "ur_klipper_sync_test.py"),
            "--stepper",
            config["manual_stepper"],
            "--axes",
            config["axes"],
            "--distance-m",
            str(config["distance_m"]),
            "--max-joint-speed-rad-s",
            str(config["max_joint_speed_rad_s"]),
        ]
        lines = [
            "Checking Klipper without moving the stepper…",
            "Klipper state: ready",
            "Running MoveIt plan-only check…",
            "MoveIt Cartesian path fraction: 1.000",
            "Plan-only complete. No hardware moved.",
        ]
        return self.jobs.start("dry-run", command, env=self._environment(), demo_lines=lines if self.demo else None)

    def sync_test(self, payload: dict[str, Any]) -> Job:
        self.require_safety(payload, motion=True)
        config = validate_config({**self.config, **payload.get("settings", {})})
        return_to_start = payload.get("return_to_start") is True
        if return_to_start and payload.get("reset_stepper_position") is not True:
            raise ApiError(
                HTTPStatus.PRECONDITION_FAILED,
                "Return to start requires resetting the Klipper stepper position to zero first.",
            )
        if not self.demo:
            klipper = self.check_klipper()
            ros = self.ros_status()
            if not klipper["ok"]:
                raise ApiError(HTTPStatus.PRECONDITION_FAILED, "Klipper must be ready before synchronized motion.")
            if (
                not ros["external_control"]
                or not ros["moveit_ready"]
                or not ros.get("klipper_bridge_ready", False)
                or ros["unstable"]
            ):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "ROS, External Control, MoveIt, and the ROS Klipper bridge must be ready "
                    "and stable before synchronized motion.",
                )
            if not klipper.get("extruder", {}).get("can_extrude", False):
                raise ApiError(
                    HTTPStatus.PRECONDITION_FAILED,
                    "The native extruder is too cold or its temperature is unavailable. "
                    "Prepare the hotend separately before requesting E motion.",
                )
            self.validate_cartesian_test_over_print_bed(config)
        command = [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "ur_klipper_sync_test.py"),
            "--execute",
            "--confirm-real-robot-move",
            "YES",
            "--confirm-cartesian-move",
            "YES",
            "--confirm-klipper-stepper",
            "YES",
            "--stepper",
            config["manual_stepper"],
            "--stepper-move",
            str(config["stepper_move"]),
            "--stepper-speed",
            str(config["stepper_speed"]),
            "--stepper-accel",
            str(config["stepper_accel"]),
            "--axes",
            config["axes"],
            "--distance-m",
            str(config["distance_m"]),
            "--max-joint-speed-rad-s",
            str(config["max_joint_speed_rad_s"]),
            "--disable-stepper-after",
        ]
        if payload.get("reset_stepper_position") is True:
            command.append("--reset-stepper-position")
        if return_to_start:
            command.append("--return-to-start")
        lines = [
            "DEMO MODE: no hardware commands will run.",
            "Checking External Control and Klipper…",
            "Executing guarded real trajectory (simulated)…",
            "Klipper native extruder command accepted (simulated).",
            "UR command completed (simulated).",
        ]
        if return_to_start:
            lines.extend(
                [
                    "Returning guarded UR trajectory to synchronized start (simulated)…",
                    "Returning Klipper extruder E to zero (simulated)…",
                    "Return-to-start completed for UR and Klipper.",
                ]
            )
        lines.append("Synchronized test complete.")
        return self.jobs.start(
            "sync-test",
            command,
            env=self._environment(),
            motion=True,
            demo_lines=lines if self.demo else None,
        )

    def stop_system(self, payload: dict[str, Any]) -> Job:
        self._require_operator_action(payload, "stop-ros")
        active = self.jobs.active()
        if active:
            raise ApiError(HTTPStatus.CONFLICT, "Cancel the active operation before stopping ROS.")
        command = platform_operation_command(
            "stop-ros",
            PROJECT_ROOT / "scripts" / "stop.sh",
            demo=self.demo,
        )
        lines = ["Stopping the ROS container…", "ROS container stopped."]
        return self.jobs.start("stop-ros", command, demo_lines=lines if self.demo else None)

    def stop_moveit(self, payload: dict[str, Any]) -> Job:
        self._require_operator_action(payload, "stop-moveit")
        if self.jobs.active():
            raise ApiError(HTTPStatus.CONFLICT, "Cancel the active operation before stopping MoveIt.")
        command = platform_operation_command(
            "stop-moveit",
            PROJECT_ROOT / "scripts" / "stop_moveit_detached.sh",
            demo=self.demo,
        )
        lines = [
            "Stopping MoveIt planning services…",
            "MoveIt stopped. UR driver and ROS container remain available.",
        ]
        return self.jobs.start("stop-moveit", command, demo_lines=lines if self.demo else None)

    def stop_ros_driver(self, payload: dict[str, Any]) -> Job:
        self._require_operator_action(payload, "stop-ros-driver")
        if self.jobs.active():
            raise ApiError(
                HTTPStatus.CONFLICT,
                "Cancel the active operation before stopping the ROS and UR driver.",
            )
        command = platform_operation_command(
            "stop-driver",
            PROJECT_ROOT / "scripts" / "stop_ros_driver_detached.sh",
            demo=self.demo,
        )
        lines = [
            "Stopping MoveIt and the UR driver…",
            "Closing the External Control connection…",
            "UR driver and MoveIt stopped. ROS container remains available.",
        ]
        return self.jobs.start("stop-ros-driver", command, demo_lines=lines if self.demo else None)


def make_handler(backend: StationBackend) -> type[BaseHTTPRequestHandler]:
    class StationHandler(BaseHTTPRequestHandler):
        server_version = f"URPrintStation/{VERSION}"

        def log_message(self, fmt: str, *args: Any) -> None:
            try:
                sys.stderr.write(f"[{self.log_date_time_string()}] {fmt % args}\n")
                sys.stderr.flush()
            except (OSError, ValueError):
                # The macOS wrapper can disappear without taking an older
                # child process with it (for example after a forced quit).
                # A closed log stream must never prevent the local health
                # endpoint—or any other response—from being delivered.
                pass

        def _security_headers(self, *, api: bool = False) -> None:
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; style-src 'self'; script-src 'self'; "
                "img-src 'self' data:; connect-src 'self'; base-uri 'none'; frame-ancestors 'none'",
            )
            if api:
                self.send_header("Cache-Control", "no-store")

        def _send_json(self, status: int, payload: dict[str, Any]) -> None:
            data = json.dumps(payload).encode("utf-8")
            try:
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self._security_headers(api=True)
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                # A browser may abandon a legacy synchronous response while a
                # long local review is still finishing. The review result is
                # unaffected; do not attempt a second 500 response on a dead
                # socket.
                return

        def _read_json(self) -> dict[str, Any]:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Content-Length.") from exc
            if length > 65_536:
                raise ApiError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Request body is too large.")
            if length == 0:
                return {}
            try:
                value = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Request body must be valid JSON.") from exc
            if not isinstance(value, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "Request body must be a JSON object.")
            return value

        def _read_gcode_file(self) -> tuple[str, bytes]:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Content-Length.") from exc
            if length <= 0:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Select a G-code file to import.")
            if length > MAX_GCODE_BYTES:
                raise ApiError(
                    HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                    "The selected G-code file exceeds the 20 MB limit.",
                )
            encoded_name = self.headers.get("X-Gcode-Filename", "")
            filename = urllib.parse.unquote(encoded_name)
            if not filename:
                raise ApiError(
                    HTTPStatus.BAD_REQUEST,
                    "The G-code filename header is missing.",
                )
            return filename, self.rfile.read(length)

        def _read_paired_timeline_files(self) -> tuple[str, bytes, str, bytes]:
            """Read two UTF-8 source files without storing or dispatching either."""
            if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                raise ApiError(HTTPStatus.BAD_REQUEST, "Paired import requires application/json.")
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid Content-Length.") from exc
            if length <= 0 or length > 48 * 1024 * 1024:
                raise ApiError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Paired import request is empty or too large.")
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ApiError(HTTPStatus.BAD_REQUEST, "Paired import must be valid UTF-8 JSON.") from exc
            if not isinstance(payload, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "Paired import must be a JSON object.")
            trajectory = payload.get("trajectory")
            extrusion = payload.get("extrusion")
            if not isinstance(trajectory, dict) or not isinstance(extrusion, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "Select both the UR trajectory and timed extrusion files.")
            if not all(isinstance(item.get(key), str)
                       for item in (trajectory, extrusion) for key in ("name", "text")):
                raise ApiError(HTTPStatus.BAD_REQUEST, "Each paired file needs a name and UTF-8 text.")
            return (
                trajectory["name"], trajectory["text"].encode("utf-8"),
                extrusion["name"], extrusion["text"].encode("utf-8"),
            )

        def _check_origin(self) -> None:
            origin = self.headers.get("Origin")
            if not origin:
                return
            parsed = urllib.parse.urlparse(origin)
            if parsed.scheme not in {"http", "https"} or parsed.hostname not in {"127.0.0.1", "localhost"}:
                raise ApiError(HTTPStatus.FORBIDDEN, "Cross-origin requests are not allowed.")

        def _handle_error(self, exc: Exception) -> None:
            if isinstance(exc, ApiError):
                payload: dict[str, Any] = {"ok": False, "error": exc.message}
                if exc.code:
                    payload["code"] = exc.code
                if exc.details:
                    payload["details"] = exc.details
                self._send_json(exc.status, payload)
            else:
                self._send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"ok": False, "error": f"Internal server error: {exc}"},
                )

        def do_GET(self) -> None:  # noqa: N802
            try:
                path = urllib.parse.urlparse(self.path).path
                if path == "/api/health":
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "version": VERSION, "demo": backend.demo, "project_root": str(PROJECT_ROOT)},
                    )
                    return
                if path == "/api/config":
                    self._send_json(HTTPStatus.OK, {"ok": True, "config": backend.config})
                    return
                if path == "/api/mode":
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, **backend.get_operating_mode()},
                    )
                    return
                if path == "/api/gcode/calibration":
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            **backend.get_gcode_calibration(),
                        },
                    )
                    return
                if path == "/api/workcell/height":
                    self._send_json(HTTPStatus.OK, {"ok": True, **backend.get_workcell_height()})
                    return
                if path == "/api/ros/tool0-pose":
                    backend.require_online_mode("capturing the live tool0 pose")
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "pose": backend.get_current_tool0_pose(),
                        },
                    )
                    return
                if path == "/api/status":
                    latest = backend.jobs.latest()
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "demo": backend.demo,
                            "ros": backend.ros_status(),
                            "active_job": backend.jobs.active().public() if backend.jobs.active() else None,
                            "latest_job": latest.public() if latest else None,
                            "print_interlock": backend.print_interlock_status(),
                        },
                    )
                    return
                if path == "/api/print/recovery":
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "print_interlock": backend.print_interlock_status(),
                        },
                    )
                    return
                if path.startswith("/api/jobs/"):
                    job_id = path.removeprefix("/api/jobs/").split("/")[0]
                    self._send_json(HTTPStatus.OK, {"ok": True, "job": backend.jobs.get(job_id).public()})
                    return
                review_job_match = re.fullmatch(
                    r"/api/gcode/review/jobs/([a-f0-9]{12})",
                    path,
                )
                if review_job_match:
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "review_job": backend.get_gcode_review_job(
                                review_job_match.group(1)
                            ),
                        },
                    )
                    return
                moveit_review_match = re.fullmatch(
                    r"/api/gcode/review/moveit/([a-f0-9]+)",
                    path,
                )
                if moveit_review_match:
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            **backend.get_gcode_moveit_review_result(
                                moveit_review_match.group(1)
                            ),
                        },
                    )
                    return
                print_preparation_match = re.fullmatch(
                    r"/api/print/prepare/([a-f0-9]{12})",
                    path,
                )
                if print_preparation_match:
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "preparation": backend.get_print_preparation(
                                print_preparation_match.group(1)
                            ),
                        },
                    )
                    return
                print_status_match = re.fullmatch(
                    r"/api/print/([a-f0-9]{16})",
                    path,
                )
                if print_status_match:
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            **backend.get_print_execution_status(
                                print_status_match.group(1)
                            ),
                        },
                    )
                    return
                if path == "/api/rviz-snapshot":
                    if not RVIZ_SNAPSHOT.exists():
                        raise ApiError(HTTPStatus.NOT_FOUND, "RViz snapshot is unavailable.")
                    data = RVIZ_SNAPSHOT.read_bytes()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Content-Length", str(len(data)))
                    self._security_headers()
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if path.startswith("/api/"):
                    raise ApiError(HTTPStatus.NOT_FOUND, "API endpoint not found.")
                self._serve_static(path)
            except Exception as exc:
                self._handle_error(exc)

        def do_PUT(self) -> None:  # noqa: N802
            try:
                self._check_origin()
                path = urllib.parse.urlparse(self.path).path
                if path == "/api/config":
                    config = backend.save_config(self._read_json())
                    self._send_json(HTTPStatus.OK, {"ok": True, "config": config})
                    return
                if path == "/api/mode":
                    mode = backend.set_operating_mode(self._read_json())
                    self._send_json(HTTPStatus.OK, {"ok": True, **mode})
                    return
                if path == "/api/gcode/calibration":
                    saved = backend.save_gcode_calibration(self._read_json())
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, **saved},
                    )
                    return
                if path == "/api/workcell/height":
                    result = backend.set_workcell_height_draft(self._read_json())
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                if path == "/api/workcell/print-z":
                    result = backend.set_print_z_draft(self._read_json())
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                if path == "/api/workcell/nozzle-tcp":
                    result = backend.save_nozzle_tcp(self._read_json())
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                raise ApiError(HTTPStatus.NOT_FOUND, "API endpoint not found.")
            except Exception as exc:
                self._handle_error(exc)

        def do_POST(self) -> None:  # noqa: N802
            try:
                self._check_origin()
                path = urllib.parse.urlparse(self.path).path
                if path == "/api/paired-timeline/parse":
                    trajectory_name, trajectory_content, extrusion_name, extrusion_content = (
                        self._read_paired_timeline_files()
                    )
                    try:
                        analysis = parse_paired_timeline(
                            trajectory_name, trajectory_content,
                            extrusion_name, extrusion_content,
                        )
                    except PairedTimelineError as exc:
                        raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(exc)) from exc
                    self._send_json(HTTPStatus.OK, {"ok": True, "analysis": analysis})
                    return
                if path == "/api/gcode/parse":
                    filename, content = self._read_gcode_file()
                    try:
                        analysis = parse_gcode(filename, content)
                    except GCodeParseError as exc:
                        raise ApiError(
                            HTTPStatus.UNPROCESSABLE_ENTITY,
                            str(exc),
                        ) from exc
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "analysis": analysis},
                    )
                    return
                if path == "/api/gcode/plan":
                    filename, content = self._read_gcode_file()
                    try:
                        plan = build_path_plan(filename, content)
                    except (GCodeParseError, GCodePlanError) as exc:
                        raise ApiError(
                            HTTPStatus.UNPROCESSABLE_ENTITY,
                            str(exc),
                        ) from exc
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "plan": plan},
                    )
                    return
                if path == "/api/gcode/review":
                    filename, content = self._read_gcode_file()
                    preferences = {
                        item.strip().lower()
                        for item in self.headers.get("Prefer", "").split(",")
                    }
                    if "respond-async" in preferences:
                        review_job = backend.start_gcode_review_job(
                            filename,
                            content,
                            simulation_only=False,
                        )
                        self._send_json(
                            HTTPStatus.ACCEPTED,
                            {"ok": True, "review_job": review_job},
                        )
                    else:
                        review = backend.build_gcode_path_review(filename, content)
                        self._send_json(
                            HTTPStatus.OK,
                            {"ok": True, "review": review},
                        )
                    return
                if path == "/api/gcode/review/simulation":
                    filename, content = self._read_gcode_file()
                    preferences = {
                        item.strip().lower()
                        for item in self.headers.get("Prefer", "").split(",")
                    }
                    if "respond-async" in preferences:
                        review_job = backend.start_gcode_review_job(
                            filename,
                            content,
                            simulation_only=True,
                        )
                        self._send_json(
                            HTTPStatus.ACCEPTED,
                            {"ok": True, "review_job": review_job},
                        )
                    else:
                        review = backend.build_gcode_simulation_review(
                            filename,
                            content,
                        )
                        self._send_json(
                            HTTPStatus.OK,
                            {"ok": True, "review": review},
                        )
                    return
                if path == "/api/gcode/review/moveit":
                    backend.require_online_mode("requesting MoveIt path validation")
                    filename, content = self._read_gcode_file()
                    started = backend.start_gcode_moveit_review(
                        filename,
                        content,
                    )
                    self._send_json(
                        HTTPStatus.ACCEPTED,
                        {"ok": True, **started},
                    )
                    return
                if path == "/api/print/prepare":
                    filename, content = self._read_gcode_file()
                    moveit_job_id = self.headers.get("X-Moveit-Job-Id", "")
                    execution_extent = self.headers.get("X-Execution-Extent", "sample")
                    preferences = {
                        item.strip().lower()
                        for item in self.headers.get("Prefer", "").split(",")
                    }
                    if "respond-async" in preferences:
                        preparation = backend.start_print_preparation(
                            filename,
                            content,
                            moveit_job_id,
                            execution_extent=execution_extent,
                        )
                        self._send_json(
                            HTTPStatus.ACCEPTED,
                            {"ok": True, "preparation": preparation},
                        )
                    else:
                        package = backend.prepare_print_execution(
                            filename,
                            content,
                            moveit_job_id,
                            execution_extent=execution_extent,
                        )
                        self._send_json(
                            HTTPStatus.OK,
                            {"ok": True, "package": package},
                        )
                    return
                if path == "/api/gcode/calibration/validate":
                    validation = backend.validate_gcode_calibration(
                        self._read_json()
                    )
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "validation": validation},
                    )
                    return
                payload = self._read_json()
                if path == "/api/workcell/height/plan":
                    job = backend.start_workcell_height_trial(payload, execute=False)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path == "/api/workcell/height/execute":
                    job = backend.start_workcell_height_trial(payload, execute=True)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path == "/api/workcell/height/confirm":
                    result = backend.confirm_workcell_height(payload)
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                if path == "/api/workcell/print-z/plan":
                    job = backend.start_print_z_trial(payload, execute=False)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path == "/api/workcell/print-z/execute":
                    job = backend.start_print_z_trial(payload, execute=True)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path == "/api/workcell/print-z/confirm":
                    result = backend.confirm_print_z_trial(payload)
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                if path == "/api/print/recovery":
                    recovery = backend.recover_print_stop(payload)
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "recovery": recovery},
                    )
                    return
                if path == "/api/runtime/shutdown":
                    result = backend.shutdown_runtime(
                        reason=str(payload.get("reason", "application-exit")),
                    )
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": True, "shutdown": result},
                    )
                    # The native launcher waits for the response before it
                    # exits. Stop HTTP afterward even when the launcher
                    # attached to a backend it did not spawn itself.
                    threading.Thread(
                        target=self.server.shutdown,
                        daemon=True,
                        name="workstation-http-shutdown",
                    ).start()
                    return
                if path == "/api/print/move-to-start":
                    backend.require_online_mode("moving the UR to the reviewed print start")
                    job = backend.start_print_operation(payload, mode="move-to-start")
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path == "/api/print/start":
                    backend.require_online_mode("starting Print Execution")
                    job = backend.start_print_operation(payload, mode="execute")
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                pause_match = re.fullmatch(
                    r"/api/print/([a-f0-9]{16})/(pause|resume)",
                    path,
                )
                if pause_match:
                    result = backend.set_print_pause(
                        pause_match.group(1),
                        paused=pause_match.group(2) == "pause",
                    )
                    self._send_json(HTTPStatus.OK, {"ok": True, **result})
                    return
                stop_print_match = re.fullmatch(
                    r"/api/print/([a-f0-9]{16})/stop",
                    path,
                )
                if stop_print_match:
                    job = backend.stop_print_operation(stop_print_match.group(1))
                    self._send_json(HTTPStatus.OK, {"ok": True, "job": job.public()})
                    return
                if path == "/api/ur/check":
                    backend.require_online_mode("checking the live UR robot")
                    self._send_json(HTTPStatus.OK, backend.check_ur())
                    return
                if path == "/api/klipper/check":
                    backend.require_online_mode("checking live Klipper state")
                    self._send_json(HTTPStatus.OK, backend.check_klipper())
                    return
                online_operations: dict[str, Callable[[dict[str, Any]], Job]] = {
                    "/api/ros/start": backend.start_ros,
                    "/api/ros/restart": backend.restart_ros,
                    "/api/ros/wait-external": backend.wait_external_control,
                    "/api/moveit/start": backend.start_moveit,
                    "/api/dry-run": backend.dry_run,
                    "/api/sync-test": backend.sync_test,
                }
                stop_operations: dict[str, Callable[[dict[str, Any]], Job]] = {
                    "/api/moveit/stop": backend.stop_moveit,
                    "/api/ros/driver/stop": backend.stop_ros_driver,
                    "/api/system/stop": backend.stop_system,
                }
                if path in online_operations:
                    backend.require_online_mode("starting a live workstation operation")
                    job = online_operations[path](payload)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                if path in stop_operations:
                    # Stop and cancel controls remain available after the
                    # operator switches to Offline, because already-running
                    # services are intentionally not stopped automatically.
                    job = stop_operations[path](payload)
                    self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "job": job.public()})
                    return
                cancel_match = re.fullmatch(r"/api/jobs/([a-f0-9]+)/cancel", path)
                if cancel_match:
                    job = backend.jobs.cancel(cancel_match.group(1))
                    self._send_json(HTTPStatus.OK, {"ok": True, "job": job.public()})
                    return
                raise ApiError(HTTPStatus.NOT_FOUND, "API endpoint not found.")
            except Exception as exc:
                self._handle_error(exc)

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(HTTPStatus.NO_CONTENT)
            self._security_headers(api=True)
            self.end_headers()

        def _serve_static(self, request_path: str) -> None:
            relative = request_path.lstrip("/") or "index.html"
            candidate = (STATIC_DIR / relative).resolve()
            try:
                candidate.relative_to(STATIC_DIR.resolve())
            except ValueError as exc:
                raise ApiError(HTTPStatus.FORBIDDEN, "Invalid static path.") from exc
            if not candidate.is_file():
                candidate = STATIC_DIR / "index.html"
            data = candidate.read_bytes()
            mime = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"{mime}; charset=utf-8" if mime.startswith("text/") else mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self._security_headers()
            self.end_headers()
            self.wfile.write(data)

    return StationHandler


def create_server(host: str, port: int, *, demo: bool = False) -> tuple[ThreadingHTTPServer, StationBackend]:
    backend = StationBackend(demo=demo)
    server = ThreadingHTTPServer((host, port), make_handler(backend))
    return server, backend


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the local UR Print Workstation control station.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host. Keep localhost for safety.")
    parser.add_argument("--port", type=int, default=8876, help="Local HTTP port.")
    parser.add_argument("--demo", action="store_true", help="Use simulated checks and never run hardware commands.")
    parser.add_argument("--no-browser", action="store_true", help="Do not open the browser automatically.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.host not in {"127.0.0.1", "localhost"}:
        print("Refusing to bind beyond localhost. This control station is local-only.", file=sys.stderr)
        return 64
    migrate_legacy_state(SOFTWARE_DIR, DATA_DIR)
    server, backend = create_server(args.host, args.port, demo=args.demo)
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"UR Print Workstation {VERSION}")
    print(f"Mode: {'DEMO — no hardware commands' if args.demo else 'REAL HARDWARE'}")
    print(f"Open: {url}")
    if not args.no_browser:
        threading.Timer(0.7, lambda: webbrowser.open(url)).start()
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def terminate_cleanly(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate_cleanly)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            backend.shutdown_runtime(reason="backend-exit")
        except Exception as exc:
            print(f"Runtime cleanup incomplete: {exc}", file=sys.stderr)
        server.server_close()
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
