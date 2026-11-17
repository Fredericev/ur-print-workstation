#!/usr/bin/env python3
"""Windows entrypoints for display-only workstation tools; no host Bash needed."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import shutil
import sys
import tempfile
import time
import webbrowser
from pathlib import Path

from windows_runtime import RuntimeFailure, WindowsRuntime


FORBIDDEN_PROCESSES = (
    "[u]r_ros2_control_node|[d]ashboard_client|[c]ontroller_manager|[r]os2_control_node|"
    "[u]rscript_interface|[k]lipper_bridge|[g]code_print_executor|[t]ool_communication|"
    "[r]obot_state_helper|[c]ontroller_stopper_node|[t]rajectory_until_node"
)

OFFLINE_CLEANUP = r"""
set -e
pid_file=/home/ubuntu/ur_ros2_ws/offline_rviz_preview.pid
if [ -f "$pid_file" ]; then
  old_pid="$(tr -cd '0-9' < "$pid_file")"
  if [ -n "$old_pid" ] && [ -r "/proc/$old_pid/cmdline" ] &&
     tr '\0' ' ' < "/proc/$old_pid/cmdline" | grep -Fq offline_print_preview.launch.py; then
    old_pgid="$(ps -o pgid= -p "$old_pid" | tr -d ' ' || true)"
    if [ "$old_pgid" = "$old_pid" ]; then
      kill -TERM -- "-$old_pid" 2>/dev/null || true
    else
      kill -TERM "$old_pid" 2>/dev/null || true
    fi
  fi
fi
rm -f "$pid_file"
for signal in TERM KILL; do
  pkill -"$signal" -f '[o]ffline_print_preview.launch.py' 2>/dev/null || true
  pkill -"$signal" -f '__node:=[o]ffline_preview_robot_state_publisher' 2>/dev/null || true
  pkill -"$signal" -f '__node:=[o]ffline_print_preview_rviz' 2>/dev/null || true
  pkill -"$signal" -f '[p]ublish_workcell_preview.py' 2>/dev/null || true
  pkill -"$signal" -f '[r]viz_offline_print_playback.py' 2>/dev/null || true
  if [ "$signal" = TERM ]; then sleep 1; fi
done
"""

OFFLINE_LAUNCH = r"""
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ur_ros2_ws/install/setup.bash
export DISPLAY=:1 XAUTHORITY=/home/ubuntu/.Xauthority
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
log=/home/ubuntu/ur_ros2_ws/offline_rviz_preview.log
: > "$log"
nohup setsid ros2 launch ur3e_print_planner offline_print_preview.launch.py \
  workspace_root:=/home/ubuntu/ur_ros2_ws \
  project_root:=/home/ubuntu/offline_preview_project \
  manifest:="$OFFLINE_MANIFEST" speed_factor:="$OFFLINE_SPEED_FACTOR" \
  > "$log" 2>&1 &
echo "$!" > /home/ubuntu/ur_ros2_ws/offline_rviz_preview.pid
"""


def resolve_project_root(environment: dict[str, str] | None = None) -> Path:
    """Use the launcher's per-user payload without requiring a prior app launch."""
    env = os.environ if environment is None else environment
    configured = env.get("UR_PRINT_PROJECT_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    packaged = env.get("UR_PRINT_PACKAGED_PROJECT_ROOT", "").strip()
    if not packaged:
        return Path(__file__).resolve().parent.parent
    source = Path(packaged).resolve()
    marker = (source / "PAYLOAD_VERSION.txt").read_text(encoding="utf-8").strip()
    if not marker or len(marker) > 160:
        raise RuntimeFailure("The packaged project version marker is invalid.", 66)
    data = env.get("UR_PRINT_DATA_DIR", "").strip()
    if not data:
        local_app_data = env.get("LOCALAPPDATA", "").strip()
        if not local_app_data:
            raise RuntimeFailure("Windows LOCALAPPDATA is unavailable.", 66)
        data = str(Path(local_app_data) / "UR Print Workstation")
    parent = Path(data) / "projects"
    destination = parent / re.sub(r"[^A-Za-z0-9_-]", "-", marker)
    destination_marker = destination / "PAYLOAD_VERSION.txt"
    if destination_marker.is_file() and destination_marker.read_text(encoding="utf-8").strip() == marker:
        return destination.resolve()
    if destination.exists():
        raise RuntimeFailure(f"The writable project payload is incomplete: {destination}", 66)
    parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".rviz-payload-", dir=parent))
    try:
        for child in source.iterdir():
            if child.name.casefold() in {"software", "payload_version.txt"}:
                continue
            target = staging / child.name
            if child.is_dir():
                shutil.copytree(child, target)
            else:
                shutil.copy2(child, target)
        (staging / "PAYLOAD_VERSION.txt").write_text(marker + "\n", encoding="utf-8")
        try:
            staging.rename(destination)
        except OSError:
            # The native app may have initialized this exact payload while
            # the helper was copying. Never replace its mutable local state.
            if not (destination_marker.is_file() and destination_marker.read_text(encoding="utf-8").strip() == marker):
                raise
        return destination.resolve()
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def load_playback(project_root: Path, manifest: str, speed_factor: float):
    script = project_root / "workspaces/ur_ros2_ws/scripts/rviz_offline_print_playback.py"
    spec = importlib.util.spec_from_file_location("windows_offline_playback", script)
    if spec is None or spec.loader is None:
        raise RuntimeFailure("The offline playback verifier is missing.", 66)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    playback = module.load_playback(
        manifest,
        workspace_root=project_root / "workspaces/ur_ros2_ws",
        project_root=project_root,
        speed_factor=speed_factor,
    )
    print(json.dumps(module.playback_summary(playback), ensure_ascii=False), flush=True)
    return playback


def write_workcell_preview(project_root: Path, source: Path, calibration: Path) -> None:
    from gcode_calibration import read_saved_calibration
    from gcode_path_review import build_offline_review

    saved = read_saved_calibration(calibration)
    if saved is None:
        raise RuntimeFailure(f"Installed print setup was not found: {calibration}", 66)
    review = build_offline_review(source.name, source.read_bytes(), saved["calibration"])
    payload = {
        "schema_version": 1, "frame_id": review["coordinate_frame"],
        "source": review["source"], "operation_mode": review["operation_mode"],
        "placement": review["placement_model"], "table": review["workcell_model"]["tabletop"],
        "print_bed_m": review["print_bed_m"], "segments": review["path"]["segments"],
        "layers": review["layers"],
    }
    destination = project_root / "workspaces/ur_ros2_ws/runtime/rviz_job_preview.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":")) + "\n", encoding="utf-8")
    temporary.replace(destination)


class WindowsDisplayTools:
    def __init__(self, runtime: WindowsRuntime, *, playback_loader=load_playback, preview_writer=write_workcell_preview):
        self.runtime = runtime
        self.playback_loader = playback_loader
        self.preview_writer = preview_writer

    def _probe(self, command: str, *, timeout: float, label: str) -> int:
        try:
            return self.runtime._container_bash(command, timeout=timeout, label=label).returncode
        except RuntimeFailure as exc:
            return exc.exit_code

    def _assert_no_hardware(self) -> None:
        # Distinguish pgrep's expected 'no match' from Docker's own exit 1.
        # A failed docker exec must never count as evidence of no hardware.
        try:
            result = self.runtime._container_bash(
                f"status=0; pgrep -af '{FORBIDDEN_PROCESSES}' || status=$?; "
                'if [ "$status" -eq 1 ]; then echo WORKSTATION_NO_HARDWARE=1; else exit 78; fi',
                timeout=12, label="Offline replay hardware exclusion",
            )
            absent = result.returncode == 0 and result.stdout.strip() == "WORKSTATION_NO_HARDWARE=1"
        except RuntimeFailure:
            absent = False
        if not absent:
            raise RuntimeFailure(
                "Offline RViz replay refused: hardware-control processes are running, "
                "or their absence could not be verified. Stop the ROS driver/executor first. "
                "This helper will not stop them for you.", 78,
            )

    def offline_rviz(self, *, manifest: str = "latest", speed_factor: float = 10.0,
                     calibration: Path, desktop_port: int = 6082, build_policy: str = "auto") -> str:
        root = self.runtime.project_root
        # The shared verifier checks every cached source/review/trajectory and
        # complete path before the helper makes any Docker state changes.
        playback = self.playback_loader(root, manifest, speed_factor)
        try:
            relative = playback.manifest_path.resolve().relative_to(self.runtime.workspace.resolve())
        except ValueError as exc:
            raise RuntimeFailure("Execution manifest must be inside this project's ROS workspace.", 64) from exc
        self.preview_writer(root, playback.source_path, calibration)
        self.runtime.docker_preflight()
        if self.runtime._container_running():
            self._assert_no_hardware()
        self.runtime._write_update_rate(125)
        self.runtime._start_container(build_policy, desktop_port, "127.0.0.1")
        self._assert_no_hardware()
        desktop = r"test -S /tmp/.X11-unix/X1 && test -r /home/ubuntu/.Xauthority && DISPLAY=:1 XAUTHORITY=/home/ubuntu/.Xauthority xdpyinfo >/dev/null 2>&1"
        self._wait(desktop, "ROS graphical desktop", 60)
        self.runtime._build_ros_sources()
        self._assert_no_hardware()
        self.runtime._container_bash(OFFLINE_CLEANUP, timeout=20, label="Previous offline replay cleanup")
        self.runtime._container_bash(
            OFFLINE_LAUNCH,
            env={"OFFLINE_MANIFEST": "/home/ubuntu/ur_ros2_ws/" + relative.as_posix(),
                 "OFFLINE_SPEED_FACTOR": str(playback.speed_factor)},
            timeout=20, label="Isolated offline RViz replay launch",
        )
        self._wait("grep -q 'published_topic.*display_planned_path' /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log",
                   "Offline RViz publication", 45)
        self._assert_no_hardware()
        url = f"http://localhost:{desktop_port}/vnc.html"
        print(f"Offline RViz replay ready: {url}\nROS_DOMAIN_ID=84; display only; no UR or Klipper command was created.", flush=True)
        return url

    def _wait(self, command: str, label: str, attempts: int) -> None:
        for attempt in range(attempts):
            status = self._probe(command, timeout=8, label=label)
            if status == 0:
                return
            if status != 1:
                raise RuntimeFailure(f"{label} could not be checked (exit {status}).", status)
            if attempt + 1 < attempts:
                time.sleep(1)
        raise RuntimeFailure(f"{label} did not become ready. Review the ROS desktop or offline_rviz_preview.log.", 75)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="action", required=True)
    replay = subcommands.add_parser("offline-rviz", help="Replay a fully verified local print in isolated RViz")
    replay.add_argument("--manifest", default="latest")
    replay.add_argument("--speed-factor", type=float, default=10.0)
    replay.add_argument("--calibration", type=Path)
    replay.add_argument("--desktop-port", type=int, default=6082)
    replay.add_argument("--build-policy", choices=("auto", "always", "never"), default="auto")
    replay.add_argument("--open-browser", action="store_true")
    args = parser.parse_args(argv)
    try:
        root = resolve_project_root()
        data = os.environ.get("UR_PRINT_DATA_DIR") or str(Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "UR Print Workstation")
        os.environ["UR_PRINT_PROJECT_ROOT"] = str(root)
        tools = WindowsDisplayTools(WindowsRuntime(root))
        url = tools.offline_rviz(
            manifest=args.manifest, speed_factor=args.speed_factor,
            calibration=args.calibration or Path(data) / "gcode_calibration.json",
            desktop_port=args.desktop_port, build_policy=args.build_policy,
        )
        if args.open_browser:
            webbrowser.open(url)
        return 0
    except (RuntimeFailure, OSError, ValueError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return exc.exit_code if isinstance(exc, RuntimeFailure) else 66


if __name__ == "__main__":
    raise SystemExit(main())
