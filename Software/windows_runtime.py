#!/usr/bin/env python3
"""Cross-platform host orchestrator for the Windows online workstation.

This module replaces the macOS-only host shell scripts for Windows.  It never
uses a host Bash, Git Bash, or WSL process.  The only shell used here is
``bash -lc`` *inside* the project's Linux ROS container, where ROS setup files
and the proven launch commands live.

The module is intentionally synchronous so its stdout can be captured by the
workstation's existing job logger.  It is safe to import on macOS/Linux for
unit tests, but its primary runtime target is Windows with Docker Desktop in
Linux-container mode.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import math
import os
import re
import socket
import subprocess
import sys
import textwrap
import time
from pathlib import Path
from typing import Mapping, Sequence
from urllib.parse import urlparse, urlunparse


SOFTWARE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(
    os.environ.get("UR_PRINT_PROJECT_ROOT", str(SOFTWARE_DIR.parent))
).expanduser().resolve()
COMPOSE_FILE = PROJECT_ROOT / "docker-compose.real.yml"
WORKSPACE_DIR = PROJECT_ROOT / "workspaces" / "ur_ros2_ws"
PATH_REVIEW_DIR = WORKSPACE_DIR / "runtime" / "path_review"

ROS_SERVICE = "ros2"
ROS_CONTAINER = "ur3e-ros2-real"
ROS_IMAGE = "ur3e-real-ros2"
UR_REVERSE_PORTS = (50001, 50002, 50003, 50004)
MAX_MOVEIT_WAYPOINTS = 300_000
MAX_REVIEW_INPUT_BYTES = 96 * 1024 * 1024
MAX_REVIEW_OUTPUT_BYTES = 32 * 1024 * 1024

IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9_]+$")
REVIEW_ID_RE = re.compile(r"^[a-f0-9]{12}$")
SHA256_RE = re.compile(r"^[a-f0-9]{64}$")


def robot_kinematics_params_file() -> str:
    """One container-side calibration path for both the UR driver and MoveIt."""
    return (
        os.environ.get("UR_KINEMATICS_PARAMS_FILE")
        or os.environ.get("KINEMATICS_PARAMS_FILE")
        or "/home/ubuntu/ur_ros2_ws/config/ur3e_actual_calibration.yaml"
    )


class RuntimeFailure(Exception):
    """A fail-closed, operator-facing runtime error."""

    def __init__(self, message: str, exit_code: int = 1):
        super().__init__(message)
        self.message = message
        self.exit_code = exit_code if 1 <= int(exit_code) <= 125 else 1


class CommandRunner:
    """Run an argv command without a host shell and echo its captured output."""

    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str] | None = None,
        timeout: float = 60.0,
        check: bool = True,
        label: str = "command",
    ) -> subprocess.CompletedProcess[str]:
        command = [str(value) for value in argv]
        if not command or any("\x00" in value for value in command):
            raise RuntimeFailure(f"Invalid argv for {label}.", 64)
        try:
            process = subprocess.Popen(
                command,
                cwd=cwd,
                env=None if env is None else dict(env),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                shell=False,
            )
            started = time.monotonic()
            cancellation = (os.environ if env is None else env).get("WORKSTATION_CANCEL_FILE", "")
            # Detached launch commands must finish before startup cleanup: a
            # killed Docker CLI does not cancel its in-container exec process.
            finish_before_cancel = label in {
                "Detached UR ROS driver launch", "ROS Klipper bridge startup",
                "ROS coordinator and planner build",
            }
            canceled = False
            while True:
                canceled = canceled or bool(cancellation and Path(cancellation).exists())
                if canceled and not finish_before_cancel:
                    process.terminate()
                    try:
                        output, _ = process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        output, _ = process.communicate()
                    if output:
                        print(output.rstrip(), flush=True)
                    raise RuntimeFailure("Operation canceled; completing startup cleanup.", 125)
                remaining = timeout - (time.monotonic() - started)
                if remaining <= 0:
                    process.kill()
                    output, _ = process.communicate()
                    raise subprocess.TimeoutExpired(command, timeout, output=output)
                try:
                    output, _ = process.communicate(timeout=min(0.25, remaining))
                    canceled = canceled or bool(cancellation and Path(cancellation).exists())
                    break
                except subprocess.TimeoutExpired:
                    continue
            result = subprocess.CompletedProcess(command, process.returncode, output)
            if canceled:
                if output:
                    print(output.rstrip(), flush=True)
                raise RuntimeFailure("Operation canceled; completing startup cleanup.", 125)
        except FileNotFoundError as exc:
            raise RuntimeFailure(
                f"{label} could not start: {exc}. Install Docker Desktop and "
                "make sure docker.exe is on PATH.",
                69,
            ) from exc
        except subprocess.TimeoutExpired as exc:
            partial = exc.stdout or ""
            if isinstance(partial, bytes):
                partial = partial.decode("utf-8", errors="replace")
            if partial.strip():
                print(partial.rstrip(), flush=True)
            raise RuntimeFailure(f"{label} timed out after {timeout:g} seconds.", 124) from exc
        if result.stdout:
            print(result.stdout.rstrip(), flush=True)
        if check and result.returncode != 0:
            raise RuntimeFailure(
                f"{label} failed with exit code {result.returncode}.",
                result.returncode,
            )
        return result


RUNTIME_LAUNCH_PATCH = r"""
import os
from pathlib import Path

paths = [
    Path("/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py"),
    Path("/home/ubuntu/ur_ros2_ws/install/ur_robot_driver/share/ur_robot_driver/launch/ur_control.launch.py"),
]
skip_helper = os.environ["UR_SKIP_ROBOT_STATE_HELPER"].lower() in {"1", "true", "yes"}
for path in paths:
    if not path.exists():
        continue
    text = path.read_text()
    start = text.index('"consistent_controllers": [')
    end = text.index("]\n            },", start)
    block = text[start:end]
    if '"friction_model_controller",' not in block:
        marker = '                    "ur_configuration_controller",\n'
        if marker not in block:
            raise SystemExit(f"Could not patch consistent_controllers: {path}")
        block = block.replace(
            marker,
            marker + '                    "friction_model_controller",\n',
            1,
        )
        text = text[:start] + block + text[end:]

    enabled = "        robot_state_helper_node,\n"
    disabled = "        # robot_state_helper_node,  # disabled by Codex runtime patch\n"
    if skip_helper and enabled in text:
        text = text.replace(enabled, disabled, 1)
    elif not skip_helper and disabled in text:
        text = text.replace(disabled, enabled, 1)
    elif skip_helper and disabled not in text:
        raise SystemExit(f"Could not find robot_state_helper_node: {path}")
    path.write_text(text)
    print(f"Verified UR launch runtime patch: {path}")
"""


RUNTIME_XACRO_PATCH = r"""
import os
import re
from pathlib import Path

keep_alive_count = int(os.environ["UR_KEEP_ALIVE_COUNT"])
non_blocking_read = os.environ["UR_NON_BLOCKING_READ"].lower()
if not 1 <= keep_alive_count <= 100:
    raise SystemExit("UR_KEEP_ALIVE_COUNT is outside its safe range")
if non_blocking_read not in {"true", "false"}:
    raise SystemExit("UR_NON_BLOCKING_READ must be true or false")
paths = [
    Path("/opt/ros/humble/share/ur_description/urdf/ur_macro.xacro"),
    Path("/opt/ros/humble/share/ur_description/urdf/ur.ros2_control.xacro"),
]
for path in paths:
    if not path.exists():
        raise SystemExit(f"Required UR xacro is missing: {path}")
    text = path.read_text()
    patched = re.sub(r"keep_alive_count:=\d+", f"keep_alive_count:={keep_alive_count}", text)
    patched = re.sub(
        r"non_blocking_read:=(true|false)",
        f"non_blocking_read:={non_blocking_read}",
        patched,
    )
    path.write_text(patched)
    print(
        f"Verified UR xacro runtime patch: {path} "
        f"(keep_alive_count={keep_alive_count}, non_blocking_read={non_blocking_read})"
    )
"""


CLEANUP_SCRIPT = r"""
set +e
patterns=(
  "[r]os2 launch ur_robot_driver"
  "[u]r_ros2_control_node"
  "[d]ashboard_client"
  "[r]obot_state_helper"
  "[c]ontroller_stopper_node"
  "[u]rscript_interface"
  "[t]rajectory_until_node"
  "[r]obot_state_publisher"
  "[c]ontroller_manager/spawner"
  "[r]os2 launch ur_moveit_config"
  "[m]ove_group"
  "[r]viz2"
  "[r]os2 run ur_klipper_bridge klipper_bridge"
  "[/]ur_klipper_bridge/lib/ur_klipper_bridge/klipper_bridge"
  "[u]r3e_cartesian_xyz_moveit"
)
for pattern in "${patterns[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "${patterns[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done
pkill -f "[r]os2cli.daemon.daemonize" 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* 2>/dev/null || true
echo "Stale UR, MoveIt, and Klipper bridge processes were cleared."
"""


DRIVER_LAUNCH_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
set -u
log=/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log
python3 /home/ubuntu/ur_ros2_ws/scripts/verify_controller_overlay.py
pid_file=/home/ubuntu/ur_ros2_ws/real_robot_driver_live.pid
rm -f "$pid_file"
export UR_RTDE_FREQUENCY
nohup python3 /home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py \
  --log "$log" --max-bytes 10485760 --backups 3 --new-session -- \
  ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:="$UR_TYPE" \
  robot_ip:="$ROBOT_IP" \
  reverse_ip:="$ROS_PC_IP" \
  headless_mode:="$HEADLESS_MODE" \
  launch_rviz:=false \
  launch_dashboard_client:="$LAUNCH_DASHBOARD_CLIENT" \
  controller_spawner_timeout:="$CONTROLLER_SPAWNER_TIMEOUT" \
  kinematics_params_file:="$KINEMATICS_PARAMS_FILE" \
  >/dev/null 2>&1 &
echo $! > "$pid_file"
echo "UR driver pid: $(cat "$pid_file")"
"""


BRIDGE_LAUNCH_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /home/ubuntu/ur_ros2_ws/install/setup.bash
set -u
log=/home/ubuntu/ur_ros2_ws/klipper_bridge_live.log
pid_file=/home/ubuntu/ur_ros2_ws/klipper_bridge_live.pid
probe=/home/ubuntu/ur_ros2_ws/scripts/klipper_bridge_readiness_probe.py
if [ -f "$pid_file" ]; then
  old_pid=$(cat "$pid_file" 2>/dev/null || true)
  /home/ubuntu/ur_ros2_ws/scripts/verified_pid_signal.sh \
    "$pid_file" "ur_klipper_bridge.*klipper_bridge" TERM
  if [ -f "$pid_file" ] && [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
    deadline=$((SECONDS + 8))
    while kill -0 "$old_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
      sleep 0.25
    done
  fi
fi
rm -f "$pid_file"
nohup python3 /home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py \
  --log "$log" --max-bytes 10485760 --backups 3 -- \
  ros2 run ur_klipper_bridge klipper_bridge \
  --ros-args \
  -p "moonraker_url:=$MOONRAKER_URL" \
  -p "default_stepper:=$KLIPPER_STEPPER" \
  >/dev/null 2>&1 &
bridge_pid=$!
echo "$bridge_pid" > "$pid_file"
deadline=$((SECONDS + 25))
last_result=""
while [ "$SECONDS" -lt "$deadline" ]; do
  if ! kill -0 "$bridge_pid" 2>/dev/null; then
    echo "ERROR: ROS Klipper bridge stopped before becoming ready."
    tail -n 120 "$log" 2>/dev/null || true
    exit 76
  fi
  if last_result=$(python3 "$probe" \
    --timeout 1.5 --stepper "$KLIPPER_STEPPER" --json 2>&1); then
    echo "$last_result"
    echo "ROS Klipper bridge is ready."
    exit 0
  fi
  sleep 0.5
done
echo "ERROR: ROS Klipper bridge did not become ready."
[ -z "$last_result" ] || echo "$last_result"
tail -n 120 "$log" 2>/dev/null || true
exit 75
"""


DRIVER_READY_SCRIPT = r"""
set -euo pipefail
log=/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log
deadline=$((SECONDS + UR_DRIVER_START_TIMEOUT))
while [ "$SECONDS" -lt "$deadline" ]; do
  if [ -f "$log" ] && grep -q "System successfully started!" "$log"; then
    break
  fi
  if [ -f "$log" ] && grep -Eq "Failed to initialize RTDE client after|ur_ros2_control_node.*process has died" "$log"; then
    echo "ERROR: The UR driver stopped before becoming ready."
    tail -n 80 "$log" 2>/dev/null || true
    exit 79
  fi
  sleep 1
done
if ! [ -f "$log" ] || ! grep -q "System successfully started!" "$log"; then
  if [ -f "$log" ] && grep -q "speed_slider_mask.*controlled by another RTDE client" "$log"; then
    echo "ERROR: RTDE input ownership did not recover before the startup timeout. Check other clients / Fieldbus."
    exit 78
  fi
  echo "ERROR: ROS driver did not finish startup in time."
  tail -n 80 "$log" 2>/dev/null || true
  exit 75
fi
for port in 50001 50002 50003 50004; do
  if ! ss -ltn | grep -qE ":${port}[[:space:]]"; then
    echo "ERROR: Reverse-interface port $port is not listening in the ROS container."
    ss -ltn | grep -E ":5000[1-4]" || true
    exit 76
  fi
done
start_line=$(wc -l < "$log")
echo "ROS driver started; observing RTDE stability before Play..."
sleep "$UR_READY_OBSERVE_SECONDS"
recent=$(tail -n +$((start_line + 1)) "$log")
if printf "%s\n" "$recent" | grep -Eq "Failed to read from stream|Reconnecting to the RTDE interface|Connection to reverse interface dropped|Unable to read from hardware|Packet size .* larger than buffer|process has died"; then
  echo "ERROR: RTDE/reverse connection became unstable before Play."
  printf "%s\n" "$recent" | grep -E "Failed to read from stream|Reconnecting to the RTDE interface|Connection to reverse interface dropped|Unable to read from hardware|Packet size .* larger than buffer|process has died" | tail -n 20
  exit 77
fi
echo "UR driver passed startup and stability checks."
"""


EXTERNAL_CONTROL_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
set -u
probe=/home/ubuntu/ur_ros2_ws/scripts/external_control_readiness_probe.py
log=/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log
status=0
python3 "$probe" --timeout "$TIMEOUT_SECONDS" || status=$?
if [ "$status" -ne 0 ]; then
  echo "Press Play on the teach-pendant program that contains External Control, then retry." >&2
  grep -E "Robot requested program|Sent program to robot|Robot connected to reverse interface|Connection to reverse interface dropped|Unable to read from hardware|Reconnecting to the RTDE interface|process has died|speed_slider_mask|External Control speed limit" "$log" 2>/dev/null | tail -n 20 >&2 || true
fi
exit "$status"
"""


MOVEIT_START_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
set -u
if [ ! -f "$KINEMATICS_PARAMS_FILE" ] || [ ! -r "$KINEMATICS_PARAMS_FILE" ]; then
  echo "ERROR: MoveIt calibration is missing or unreadable: $KINEMATICS_PARAMS_FILE"
  exit 66
fi
log=/home/ubuntu/ur_ros2_ws/move_group_live.log
pid_file=/home/ubuntu/ur_ros2_ws/move_group_live.pid
if pgrep -x move_group >/dev/null 2>&1; then
  if [ -f "$log" ] && grep -q "You can start planning now!" "$log"; then
    echo "move_group is already running and ready; calibration changes require restarting MoveIt only."
    exit 0
  fi
  echo "ERROR: move_group is running without a current ready log."
  exit 76
fi
rm -f "$pid_file"
nohup python3 /home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py \
  --log "$log" --max-bytes 10485760 --backups 3 -- \
  ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:="$UR_TYPE" \
  kinematics_params_file:="$KINEMATICS_PARAMS_FILE" \
  launch_rviz:=false \
  launch_servo:=false \
  use_sim_time:=false \
  publish_robot_description_semantic:=true \
  >/dev/null 2>&1 &
echo $! > "$pid_file"
deadline=$((SECONDS + 45))
while [ "$SECONDS" -lt "$deadline" ]; do
  if grep -q "You can start planning now!" "$log" 2>/dev/null &&
     pgrep -x move_group >/dev/null 2>&1; then
    echo "move_group is ready."
    exit 0
  fi
  if [ -f "$pid_file" ]; then
    launch_pid=$(cat "$pid_file")
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "ERROR: MoveIt launch process stopped before becoming ready."
      tail -n 120 "$log" 2>/dev/null || true
      exit 76
    fi
  fi
  sleep 1
done
echo "ERROR: move_group did not become ready."
tail -n 120 "$log" 2>/dev/null || true
exit 75
"""


MOVEIT_STOP_SCRIPT = r"""
set +e
pid_file=/home/ubuntu/ur_ros2_ws/move_group_live.pid
if [ -f "$pid_file" ]; then
  /home/ubuntu/ur_ros2_ws/scripts/verified_pid_signal.sh \
    "$pid_file" '[r]os2 launch ur_moveit_config' TERM || exit $?
fi
pkill -TERM -f "[r]os2 launch ur_moveit_config" 2>/dev/null || true
pkill -TERM -x move_group 2>/dev/null || true
deadline=$((SECONDS + 8))
while pgrep -x move_group >/dev/null 2>&1 && [ "$SECONDS" -lt "$deadline" ]; do
  sleep 0.25
done
pkill -KILL -f "[r]os2 launch ur_moveit_config" 2>/dev/null || true
pkill -KILL -x move_group 2>/dev/null || true
rm -f "$pid_file"
if pgrep -x move_group >/dev/null 2>&1; then
  echo "ERROR: MoveIt is still running."
  exit 1
fi
echo "MoveIt stopped. UR driver and ROS container remain available."
"""


DRIVER_STOP_VERIFY_SCRIPT = r"""
rm -f \
  /home/ubuntu/ur_ros2_ws/real_robot_driver_live.pid \
  /home/ubuntu/ur_ros2_ws/move_group_live.pid \
  /home/ubuntu/ur_ros2_ws/klipper_bridge_live.pid
printf "\n[workstation] UR driver intentionally stopped.\n" \
  >> /home/ubuntu/ur_ros2_ws/real_robot_driver_live.log
if pgrep -f "[u]r_ros2_control_node" >/dev/null 2>&1; then
  echo "ERROR: UR driver is still running."
  exit 1
fi
if pgrep -x move_group >/dev/null 2>&1; then
  echo "ERROR: MoveIt is still running."
  exit 1
fi
if pgrep -f "[r]os2 run ur_klipper_bridge klipper_bridge|[/]ur_klipper_bridge/lib/ur_klipper_bridge/klipper_bridge" >/dev/null 2>&1; then
  echo "ERROR: ROS Klipper bridge is still running."
  exit 1
fi
echo "UR driver, MoveIt, and ROS Klipper bridge stopped."
"""


PATH_VALIDATION_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
set -u
if ! pgrep -x move_group >/dev/null 2>&1; then
  echo "ERROR: MoveIt move_group is not running."
  exit 75
fi
ros2 run ur3e_print_planner gcode_path_review \
  --ros-args \
  -p path_file:="$REVIEW_PATH_FILE" \
  -p output_file:="$REVIEW_OUTPUT_FILE" \
  -p approach_trajectory_file:="$APPROACH_TRAJECTORY_FILE" \
  -p print_trajectory_file:="$PRINT_TRAJECTORY_FILE" \
  -p review_sha256:="$REVIEW_SHA256" \
  -p source_waypoint_count:="$SOURCE_WAYPOINT_COUNT" \
  -p source_sampled:="$SOURCE_SAMPLED" \
  -p tool_roll_deg:="$TOOL_ROLL_DEG" \
  -p tool_pitch_deg:="$TOOL_PITCH_DEG" \
  -p tool_yaw_deg:="$TOOL_YAW_DEG" \
  -p tcp_offset_x_m:="$TCP_OFFSET_X_M" \
  -p tcp_offset_y_m:="$TCP_OFFSET_Y_M" \
  -p tcp_offset_z_m:="$TCP_OFFSET_Z_M" \
  -p tool_collision_radius_m:="$TOOL_COLLISION_RADIUS_M" \
  -p bed_min_x_m:="$BED_MIN_X_M" \
  -p bed_max_x_m:="$BED_MAX_X_M" \
  -p bed_min_y_m:="$BED_MIN_Y_M" \
  -p bed_max_y_m:="$BED_MAX_Y_M" \
  -p bed_plane_z_m:="$BED_PLANE_Z_M" \
  -p operation_mode:="$OPERATION_MODE" \
  -p printhead_geometry_sha256:="$PRINTHEAD_GEOMETRY_SHA256" \
  -p eef_step_m:=0.005 \
  -p velocity_scaling:=0.05 \
  -p acceleration_scaling:=0.05
"""


COORDINATOR_SCRIPT = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
set -u
if [ "$HEIGHT_TRIAL_MODE" = true ]; then
  binary="$(ros2 pkg prefix ur3e_print_planner)/lib/ur3e_print_planner/cartesian_xyz_moveit"
  if ! test -r "$binary" || ! grep -aFq height_trial_collision_model_v4_geometry_bound "$binary"; then
    echo "ERROR: The installed ROS planner lacks the measured nozzle/table collision guard. Rebuild ur3e_print_planner."
    exit 78
  fi
fi
ros2 run ur3e_print_planner cartesian_xyz_moveit \
  --ros-args \
  -p axes:="$AXES" \
  -p distance_m:="$DISTANCE_M" \
  -p eef_step_m:="$EEF_STEP_M" \
  -p jump_threshold:="$JUMP_THRESHOLD" \
  -p velocity_scaling:="$VELOCITY_SCALING" \
  -p acceleration_scaling:="$ACCELERATION_SCALING" \
  -p max_segment_delta_rad:="$MAX_SEGMENT_DELTA_RAD" \
  -p max_joint_speed_rad_s:="$MAX_JOINT_SPEED_RAD_S" \
  -p min_segment_duration_s:="$MIN_SEGMENT_DURATION_S" \
  -p goal_tolerance_rad:="$GOAL_TOLERANCE_RAD" \
  -p goal_velocity_tolerance_rad_s:="$GOAL_VELOCITY_TOLERANCE_RAD_S" \
  -p smooth_execution:="$SMOOTH_EXECUTION" \
  -p include_velocity_setpoints:="$INCLUDE_VELOCITY_SETPOINTS" \
  -p execution_controller:="$EXECUTION_CONTROLLER" \
  -p switch_controller:="$SWITCH_CONTROLLER" \
  -p execute:="$EXECUTE" \
  -p return_to_start:="$RETURN_TO_START" \
  -p klipper_outbound_duration_s:="$KLIPPER_OUTBOUND_DURATION_S" \
  -p synchronize_klipper:="$SYNCHRONIZE_KLIPPER" \
  -p klipper_check_only:="$KLIPPER_CHECK_ONLY" \
  -p klipper_stepper:="$KLIPPER_STEPPER" \
  -p klipper_target_position:="$KLIPPER_TARGET_POSITION" \
  -p klipper_speed:="$KLIPPER_SPEED" \
  -p klipper_acceleration:="$KLIPPER_ACCELERATION" \
  -p klipper_reset_position:="$KLIPPER_RESET_POSITION" \
  -p klipper_disable_after:="$KLIPPER_DISABLE_AFTER" \
  -p printhead_geometry_sha256:="$PRINTHEAD_GEOMETRY_SHA256" \
  -p height_trial_mode:="$HEIGHT_TRIAL_MODE" \
  -p height_trial_surface_z_m:="$HEIGHT_TRIAL_SURFACE_Z_M" \
  -p height_trial_min_x_m:="$HEIGHT_TRIAL_MIN_X_M" \
  -p height_trial_max_x_m:="$HEIGHT_TRIAL_MAX_X_M" \
  -p height_trial_min_y_m:="$HEIGHT_TRIAL_MIN_Y_M" \
  -p height_trial_max_y_m:="$HEIGHT_TRIAL_MAX_Y_M" \
  -p height_trial_tcp_x_m:="$HEIGHT_TRIAL_TCP_X_M" \
  -p height_trial_tcp_y_m:="$HEIGHT_TRIAL_TCP_Y_M" \
  -p height_trial_tcp_z_m:="$HEIGHT_TRIAL_TCP_Z_M"
"""


class WindowsRuntime:
    """Validated Docker/ROS orchestration with no host shell dependency."""

    def __init__(
        self,
        project_root: Path = PROJECT_ROOT,
        runner: CommandRunner | None = None,
    ) -> None:
        self.project_root = Path(project_root).resolve()
        self.compose_file = self.project_root / "docker-compose.real.yml"
        self.workspace = self.project_root / "workspaces" / "ur_ros2_ws"
        self.path_review_dir = self.workspace / "runtime" / "path_review"
        self.runner = runner or CommandRunner()
        self._cancel_checks_enabled = True

    @staticmethod
    def _safe_env_value(name: str, value: object, *, maximum: int = 512) -> str:
        text = str(value)
        if not text or len(text) > maximum or any(char in text for char in "\x00\r\n"):
            raise RuntimeFailure(f"Invalid value for {name}.", 64)
        return text

    @staticmethod
    def _ipv4(name: str, value: object) -> str:
        try:
            address = ipaddress.ip_address(str(value).strip())
        except ValueError as exc:
            raise RuntimeFailure(f"{name} must be a valid IPv4 address.", 64) from exc
        if (
            address.version != 4
            or address.is_unspecified
            or address.is_loopback
            or address.is_multicast
        ):
            raise RuntimeFailure(f"{name} must be a usable non-loopback IPv4 address.", 64)
        return str(address)

    @staticmethod
    def _identifier(name: str, value: object) -> str:
        text = str(value).strip()
        if not IDENTIFIER_RE.fullmatch(text):
            raise RuntimeFailure(
                f"{name} may contain only letters, digits, and underscores.",
                64,
            )
        return text

    @staticmethod
    def _native_extruder(value: object) -> str:
        text = str(value).strip()
        if not re.fullmatch(r"extruder(?:[1-9][0-9]*)?", text):
            raise RuntimeFailure(
                "Select a native Klipper extruder (extruder, extruder1, ...); "
                "manual steppers cannot be used for extrusion.", 64,
            )
        return text

    @staticmethod
    def _moonraker_url(value: object) -> str:
        text = str(value).strip()
        parsed = urlparse(text)
        if (
            parsed.scheme not in {"http", "https"}
            or not parsed.hostname
            or parsed.username
            or parsed.password
            or parsed.params
            or parsed.query
            or parsed.fragment
            or any(char.isspace() for char in text)
        ):
            raise RuntimeFailure(
                "Moonraker URL must be a plain http(s) base URL without "
                "credentials, query, fragment, or whitespace.",
                64,
            )
        return urlunparse(
            (parsed.scheme, parsed.netloc, parsed.path.rstrip("/"), "", "", "")
        )

    @staticmethod
    def _bounded_int(name: str, value: object, minimum: int, maximum: int) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeFailure(f"{name} must be an integer.", 64) from exc
        if not minimum <= parsed <= maximum:
            raise RuntimeFailure(
                f"{name} must be between {minimum} and {maximum}.",
                64,
            )
        return parsed

    @staticmethod
    def _finite_angle(name: str, value: object) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeFailure(f"{name} must be numeric.", 64) from exc
        if not math.isfinite(parsed) or not -360.0 <= parsed <= 360.0:
            raise RuntimeFailure(f"{name} must be finite and within ±360 degrees.", 64)
        return parsed

    @staticmethod
    def _bounded_float(
        name: str,
        value: object,
        minimum: float,
        maximum: float,
        *,
        absolute: bool = False,
        nonzero: bool = False,
    ) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeFailure(f"{name} must be numeric.", 64) from exc
        compared = abs(parsed) if absolute else parsed
        if (
            not math.isfinite(parsed)
            or not minimum <= compared <= maximum
            or (nonzero and abs(parsed) <= 1e-15)
        ):
            qualifier = "absolute value " if absolute else ""
            raise RuntimeFailure(
                f"{name} {qualifier}must be between {minimum:g} and {maximum:g}"
                + (" and nonzero." if nonzero else "."),
                64,
            )
        return parsed

    @staticmethod
    def _boolean(name: str, value: object) -> bool:
        if isinstance(value, bool):
            return value
        normalized = str(value).strip().casefold()
        if normalized in {"true", "1"}:
            return True
        if normalized in {"false", "0"}:
            return False
        raise RuntimeFailure(f"{name} must be true or false.", 64)

    @staticmethod
    def _ros_double(value: float) -> str:
        text = format(float(value), ".12g")
        if "." not in text and "e" not in text.casefold():
            text += ".0"
        return text

    @staticmethod
    def _axes(value: object) -> str:
        axes = [
            item.strip().casefold()
            for item in str(value).split(",")
            if item.strip()
        ]
        if not axes or any(axis not in {"x", "y", "z"} for axis in axes):
            raise RuntimeFailure(
                "Axes must be a comma-separated sequence containing only x, y, and z.",
                64,
            )
        return ",".join(axes)

    def _validate_project(self) -> None:
        required = [
            self.compose_file,
            self.workspace / "src" / "ur_klipper_interfaces",
            self.workspace / "src" / "ur_klipper_bridge",
            self.workspace / "src" / "ur3e_print_planner",
            self.workspace / "src" / "Universal_Robots_Client_Library",
            self.workspace / "src" / "Universal_Robots_ROS2_Driver" / "ur_controllers",
            self.workspace / "src" / "Universal_Robots_ROS2_Driver" / "ur_robot_driver",
            self.workspace / "scripts" / "verify_controller_overlay.py",
            self.workspace / "scripts" / "smooth_trajectory.py",
            self.workspace / "scripts" / "supervise_with_rotating_log.py",
            self.workspace / "scripts" / "verified_pid_signal.sh",
            self.workspace / "config" / "printhead_geometry.json",
            self.project_root / "scripts" / "check_ros_container_network.py",
            self.workspace / "config" / "ur3e_actual_calibration.yaml",
            self.workspace / "scripts" / "external_control_readiness_probe.py",
            self.workspace / "scripts" / "klipper_bridge_readiness_probe.py",
        ]
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise RuntimeFailure(
                "The Windows package is incomplete; required project files are "
                "missing: " + ", ".join(missing),
                66,
            )

    def _run(
        self,
        argv: Sequence[str],
        *,
        timeout: float,
        check: bool = True,
        env: Mapping[str, str] | None = None,
        label: str,
    ) -> subprocess.CompletedProcess[str]:
        effective_env = env
        cancellation = (os.environ if env is None else env).get("WORKSTATION_CANCEL_FILE", "")
        if self._cancel_checks_enabled and cancellation and Path(cancellation).exists():
            raise RuntimeFailure("Operation canceled; completing startup cleanup.", 125)
        if not self._cancel_checks_enabled:
            effective_env = dict(os.environ if env is None else env)
            effective_env.pop("WORKSTATION_CANCEL_FILE", None)
        return self.runner.run(
            argv,
            cwd=self.project_root,
            env=effective_env,
            timeout=timeout,
            check=check,
            label=label,
        )

    def _compose(
        self,
        *args: str,
        timeout: float = 60.0,
        check: bool = True,
        env: Mapping[str, str] | None = None,
        label: str = "Docker Compose",
    ) -> subprocess.CompletedProcess[str]:
        return self._run(
            ["docker", "compose", "-f", str(self.compose_file), *args],
            timeout=timeout,
            check=check,
            env=env,
            label=label,
        )

    def _container_bash(
        self,
        script: str,
        *,
        user: str = "ubuntu",
        env: Mapping[str, object] | None = None,
        timeout: float = 60.0,
        label: str,
    ) -> subprocess.CompletedProcess[str]:
        command = [
            "docker",
            "compose",
            "-f",
            str(self.compose_file),
            "exec",
            "-T",
            "-u",
            self._identifier("container user", user),
        ]
        for key, value in sorted((env or {}).items()):
            if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
                raise RuntimeFailure(f"Invalid container environment key: {key}", 64)
            command.extend(["-e", f"{key}={self._safe_env_value(key, value, maximum=2048)}"])
        # A host-side Docker CLI timeout alone leaves docker exec running.
        # Bound the Linux command first so cleanup runs after that command has
        # stopped spawning processes, including on a stalled bridge startup.
        remote_timeout = max(1.0, timeout - 8.0)
        command.extend([ROS_SERVICE, "timeout", "--signal=TERM", "--kill-after=5s",
                        f"{remote_timeout:g}s", "bash", "-lc", textwrap.dedent(script).strip()])
        return self._run(command, timeout=timeout, label=label)

    def docker_preflight(self, *, require_project: bool = True) -> None:
        if require_project:
            self._validate_project()
        elif not self.compose_file.is_file():
            raise RuntimeFailure(
                f"Docker Compose file is missing: {self.compose_file}",
                66,
            )
        result = self._run(
            [
                "docker",
                "version",
                "--format",
                "{{.Server.Os}}|{{.Server.Arch}}|{{.Server.Version}}",
            ],
            timeout=12,
            label="Docker daemon check",
        )
        fields = result.stdout.strip().split("|")
        if len(fields) != 3 or fields[0].casefold() != "linux":
            raise RuntimeFailure(
                "Docker Desktop must be running in Linux-container mode. "
                f"Reported server: {result.stdout.strip() or 'unavailable'}.",
                69,
            )
        self._run(
            ["docker", "compose", "version", "--short"],
            timeout=10,
            label="Docker Compose check",
        )
        services = self._compose(
            "config",
            "--services",
            timeout=15,
            label="Docker Compose project check",
        )
        if ROS_SERVICE not in services.stdout.split():
            raise RuntimeFailure(
                f"Docker Compose project does not define the {ROS_SERVICE!r} service.",
                66,
            )

    def _compose_environment(
        self,
        *,
        desktop_port: int = 6082,
        reverse_bind_ip: str = "127.0.0.1",
    ) -> dict[str, str]:
        env = os.environ.copy()
        env.update(
            {
                "ROS_DESKTOP_PORT": str(
                    self._bounded_int("ROS desktop port", desktop_port, 1024, 65535)
                ),
                "UR_TYPE": "ur3e",
                "UR_REVERSE_BIND_IP": self._ipv4(
                    "UR reverse bind IP",
                    reverse_bind_ip,
                ),
            }
        )
        return env

    def _container_running(self) -> bool:
        result = self._compose(
            "ps",
            "--status",
            "running",
            "--services",
            timeout=12,
            check=False,
            label="ROS container status",
        )
        return result.returncode == 0 and ROS_SERVICE in result.stdout.split()

    def _require_container(self) -> None:
        if not self._container_running():
            raise RuntimeFailure(
                "The ROS container is not running. Run start-ros first.",
                75,
            )

    @staticmethod
    def _route_source(robot_ip: str) -> str:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect((robot_ip, 30004))
                return str(probe.getsockname()[0])
        except OSError as exc:
            raise RuntimeFailure(
                f"Windows could not find a network route to UR robot {robot_ip}: {exc}",
                67,
            ) from exc

    @staticmethod
    def _require_robot_rtde(robot_ip: str, timeout: float = 1.5) -> None:
        try:
            with socket.create_connection((robot_ip, 30004), timeout=timeout):
                return
        except OSError as exc:
            raise RuntimeFailure(
                f"UR RTDE port 30004 at {robot_ip} is unavailable: {exc}",
                67,
            ) from exc

    def _verify_robot_route(self, robot_ip: str, ros_pc_ip: str) -> None:
        selected = self._route_source(robot_ip)
        if selected != ros_pc_ip:
            raise RuntimeFailure(
                "ROS PC IP does not match the Windows interface used to reach "
                f"the robot (configured {ros_pc_ip}, routed {selected}). "
                "Select the wired Ethernet IPv4 address and update External Control.",
                66,
            )
        self._require_robot_rtde(robot_ip)
        print(
            f"Verified Windows route to UR: {ros_pc_ip} -> {robot_ip} (RTDE reachable).",
            flush=True,
        )

    def _write_update_rate(self, update_rate: int) -> Path:
        rate = self._bounded_int("UR update rate", update_rate, 50, 500)
        config_dir = self.workspace / "config"
        config_dir.mkdir(parents=True, exist_ok=True)
        destination = config_dir / "ur3e_update_rate.yaml"
        content = (
            "controller_manager:\n"
            "  ros__parameters:\n"
            f"    update_rate: {rate} # Hz\n"
        )
        overlay = (self.workspace / "src" / "Universal_Robots_ROS2_Driver"
                   / "ur_robot_driver" / "config" / "ur3e_update_rate.yaml")
        for target in (destination, overlay):
            if target == overlay and not target.is_file():
                continue
            if target.is_file() and target.read_bytes() == content.encode("utf-8"):
                continue
            # Compose has a read-only single-file bind mount. Preserve the
            # inode so an existing container sees an update; replacing the
            # file atomically would strand that mount on the old contents.
            with target.open("w", encoding="utf-8", newline="\n") as stream:
                stream.write(content)
        print(f"UR driver update-rate override: {rate} Hz ({destination})", flush=True)
        return destination

    def _start_container(
        self,
        build_policy: str,
        desktop_port: int,
        reverse_bind_ip: str,
    ) -> None:
        if build_policy not in {"auto", "always", "never"}:
            raise RuntimeFailure("Build policy must be auto, always, or never.", 64)
        compose_env = self._compose_environment(
            desktop_port=desktop_port,
            reverse_bind_ip=reverse_bind_ip,
        )
        should_build = build_policy == "always"
        if build_policy == "auto":
            image = self._compose(
                "images",
                "-q",
                ROS_SERVICE,
                timeout=12,
                check=False,
                env=compose_env,
                label="ROS image check",
            )
            should_build = image.returncode != 0 or not image.stdout.strip()
        option = "--build" if should_build else "--no-build"
        self._compose(
            "up",
            "-d",
            option,
            timeout=1800 if should_build else 180,
            env=compose_env,
            label="ROS Linux container startup",
        )
        if not self._container_running():
            raise RuntimeFailure("Docker Compose returned but the ROS container is not running.", 75)

    def _apply_update_rate(self) -> None:
        self._container_bash(
            """
            set -euo pipefail
            source_file=/home/ubuntu/ur_ros2_ws/config/ur3e_update_rate.yaml
            target_file=/opt/ros/humble/share/ur_robot_driver/config/ur3e_update_rate.yaml
            test -s "$source_file"
            if ! cmp -s "$source_file" "$target_file"; then
              target_options=$(findmnt -n -o VFS-OPTIONS --target "$target_file")
              case ,$target_options, in
                *,ro,*)
                  echo "ERROR: UR update-rate differs from its read-only container mount. Recreate the ROS container to refresh the mount."
                  exit 78 ;;
              esac
              cp "$source_file" "$target_file"
            fi
            cmp -s "$source_file" "$target_file"
            sed -n "1,8p" "$target_file"
            """,
            user="root",
            timeout=30,
            label="UR update-rate activation",
        )

    def _apply_runtime_patches(
        self,
        *,
        non_blocking_read: bool,
        skip_robot_state_helper: bool,
    ) -> None:
        common = [
            "docker",
            "compose",
            "-f",
            str(self.compose_file),
            "exec",
            "-T",
            "-u",
            "root",
        ]
        self._run(
            [
                *common,
                "-e",
                f"UR_SKIP_ROBOT_STATE_HELPER={'true' if skip_robot_state_helper else 'false'}",
                ROS_SERVICE,
                "python3",
                "-c",
                textwrap.dedent(RUNTIME_LAUNCH_PATCH).strip(),
            ],
            timeout=30,
            label="UR launch runtime patch",
        )
        self._run(
            [
                *common,
                "-e",
                "UR_KEEP_ALIVE_COUNT=10",
                "-e",
                f"UR_NON_BLOCKING_READ={'true' if non_blocking_read else 'false'}",
                ROS_SERVICE,
                "python3",
                "-c",
                textwrap.dedent(RUNTIME_XACRO_PATCH).strip(),
            ],
            timeout=30,
            label="UR xacro runtime patch",
        )

    def _build_ros_sources(self) -> None:
        self._container_bash(
            """
            set -euo pipefail
            set +u
            source /opt/ros/humble/setup.bash
            set -u
            cd /home/ubuntu/ur_ros2_ws
            export CMAKE_BUILD_PARALLEL_LEVEL=1
            export MAKEFLAGS=-j1
            # Resolve the workspace URCL before configuring the driver.  The
            # base ROS image also exports ur_client_library, whose headers
            # must not be mixed with this project's patched overlay library.
            colcon build --packages-select ur_client_library \
              --allow-overriding ur_client_library \
              --executor sequential --parallel-workers 1 \
              --event-handlers console_direct+ \
              --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
            set +u
            source install/setup.bash
            set -u
            colcon build --packages-select \
              ur_controllers \
              ur_robot_driver \
              ur_klipper_interfaces \
              ur_klipper_bridge \
              ur3e_print_planner \
              ur3e_printhead_description \
              ur_moveit_config \
              --allow-overriding ur_controllers ur_robot_driver ur_moveit_config \
              --executor sequential --parallel-workers 1 \
              --event-handlers console_direct+ \
              --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
                -Dur_client_library_DIR:PATH=/home/ubuntu/ur_ros2_ws/install/ur_client_library/lib/cmake/ur_client_library
            set +u
            source install/setup.bash
            set -u
            python3 scripts/verify_controller_overlay.py
            """,
            timeout=3600,
            label="ROS coordinator and planner build",
        )

    def _cleanup_driver(self) -> None:
        self._container_bash(
            CLEANUP_SCRIPT,
            timeout=35,
            label="Stale ROS process cleanup",
        )

    def _network_preflight(self, robot_ip: str, moonraker_url: str) -> None:
        self._run(
            [sys.executable, str(self.project_root / "scripts" / "check_ros_container_network.py"),
             "--robot-ip", robot_ip, "--moonraker-url", moonraker_url],
            timeout=45,
            label="ROS container network preflight",
        )

    def _launch_driver(
        self,
        *,
        robot_ip: str,
        ros_pc_ip: str,
        update_rate: int,
        non_blocking_read: bool,
    ) -> None:
        self._container_bash(
            DRIVER_LAUNCH_SCRIPT,
            env={
                "ROBOT_IP": robot_ip,
                "ROS_PC_IP": ros_pc_ip,
                "UR_TYPE": "ur3e",
                "UR_RTDE_FREQUENCY": update_rate,
                "HEADLESS_MODE": "false",
                "LAUNCH_DASHBOARD_CLIENT": "false",
                "CONTROLLER_SPAWNER_TIMEOUT": 45,
                "KINEMATICS_PARAMS_FILE": robot_kinematics_params_file(),
                "UR_NON_BLOCKING_READ": (
                    "true" if non_blocking_read else "false"
                ),
            },
            timeout=30,
            label="Detached UR ROS driver launch",
        )

    def _launch_bridge(self, *, moonraker_url: str, stepper: str) -> None:
        self._container_bash(
            BRIDGE_LAUNCH_SCRIPT,
            env={
                "MOONRAKER_URL": moonraker_url,
                "KLIPPER_STEPPER": stepper,
            },
            timeout=35,
            label="ROS Klipper bridge startup",
        )

    def _wait_driver_ready(self, startup_timeout: int, stability_seconds: int) -> None:
        self._container_bash(
            DRIVER_READY_SCRIPT,
            env={
                "UR_DRIVER_START_TIMEOUT": self._bounded_int(
                    "driver startup timeout", startup_timeout, 15, 180
                ),
                "UR_READY_OBSERVE_SECONDS": self._bounded_int(
                    "driver stability observation", stability_seconds, 5, 120
                ),
            },
            timeout=float(startup_timeout + stability_seconds + 20),
            label="UR driver readiness and stability check",
        )

    @staticmethod
    def _published_endpoint(line: str) -> tuple[str, int] | None:
        value = line.strip()
        if not value:
            return None
        if value.startswith("[") and "]:" in value:
            host, port_text = value[1:].split("]:", 1)
        else:
            host, separator, port_text = value.rpartition(":")
            if not separator:
                return None
        try:
            return host, int(port_text)
        except ValueError:
            return None

    def _verify_reverse_ports(self, ros_pc_ip: str) -> None:
        for port in UR_REVERSE_PORTS:
            result = self._compose(
                "port",
                ROS_SERVICE,
                str(port),
                timeout=10,
                label=f"Docker publication check for UR port {port}",
            )
            endpoints = [
                endpoint
                for endpoint in (
                    self._published_endpoint(line)
                    for line in result.stdout.splitlines()
                )
                if endpoint is not None
            ]
            accepted_hosts = {ros_pc_ip}
            if not any(
                mapped_port == port and host in accepted_hosts
                for host, mapped_port in endpoints
            ):
                raise RuntimeFailure(
                    f"UR reverse port {port} is not published on the Windows "
                    f"robot-facing interface. Docker reported: "
                    f"{result.stdout.strip() or 'no mapping'}. Do not press Play.",
                    80,
                )
        print(
            "Docker publishes UR reverse ports 50001-50004 on the host. "
            "Windows Firewall must also allow them on the private wired network.",
            flush=True,
        )

    def start_ros(
        self,
        *,
        robot_ip: object,
        ros_pc_ip: object,
        moonraker_url: object,
        stepper: object,
        update_rate: object = 125,
        desktop_port: object = 6082,
        build_policy: str = "auto",
        startup_timeout: object = 60,
        stability_seconds: object = 45,
        non_blocking_read: bool = True,
        skip_robot_state_helper: bool = True,
    ) -> None:
        robot = self._ipv4("Robot IP", robot_ip)
        configured_host = str(ros_pc_ip).strip()
        if configured_host:
            host = self._ipv4("ROS PC IP", configured_host)
        else:
            # Match the established workstation behavior while avoiding any
            # adapter-name or PowerShell dependency: a UDP connect performs no
            # robot I/O, but lets Windows reveal the source IPv4 selected by
            # its routing table.
            host = self._ipv4("ROS PC IP", self._route_source(robot))
            print(
                f"Detected ROS PC IP from the Windows route to the robot: {host}",
                flush=True,
            )
        moonraker = self._moonraker_url(moonraker_url)
        stepper_name = self._native_extruder(stepper)
        rate = self._bounded_int("UR update rate", update_rate, 50, 500)
        desktop = self._bounded_int("ROS desktop port", desktop_port, 1024, 65535)
        startup = self._bounded_int("driver startup timeout", startup_timeout, 15, 180)
        stability = self._bounded_int(
            "driver stability observation", stability_seconds, 5, 120
        )

        print(
            "Windows online mode requires Docker Desktop in Linux-container "
            "mode. Windows Firewall must allow inbound TCP 50001-50004 on the "
            "private wired robot network; do not expose those ports on a "
            "public network.",
            flush=True,
        )
        # Fail before changing Docker or ROS state when Windows is routed over
        # a different interface than the teach-pendant External Control host IP.
        self._validate_project()
        self._verify_robot_route(robot, host)
        self.docker_preflight()
        self._write_update_rate(rate)
        self._start_container(build_policy, desktop, host)
        self._network_preflight(robot, moonraker)
        self._apply_update_rate()
        self._apply_runtime_patches(
            non_blocking_read=bool(non_blocking_read),
            skip_robot_state_helper=bool(skip_robot_state_helper),
        )
        # Always build the three project-owned packages.  This avoids trusting
        # timestamps or stale Linux artifacts copied from another computer.
        # Do not use --symlink-install on a Windows bind mount: NTFS/SMB
        # symlink support varies with Developer Mode and Docker Desktop.
        self._build_ros_sources()
        owns_control_stack = False
        try:
            # Claim ownership before cleanup, which can itself be interrupted
            # after stopping only part of the previous control stack.
            owns_control_stack = True
            self._cleanup_driver()
            self._launch_driver(
                robot_ip=robot,
                ros_pc_ip=host,
                update_rate=rate,
                non_blocking_read=bool(non_blocking_read),
            )
            self._launch_bridge(moonraker_url=moonraker, stepper=stepper_name)
            self._wait_driver_ready(startup, stability)
            self._verify_reverse_ports(host)
        except BaseException:
            if owns_control_stack:
                print(
                    "Startup did not pass every readiness gate; stopping the "
                    "partially started driver and bridge.",
                    file=sys.stderr,
                    flush=True,
                )
                previous_cancel_checks = self._cancel_checks_enabled
                self._cancel_checks_enabled = False
                try:
                    self._cleanup_driver()
                    self._container_bash(
                        DRIVER_STOP_VERIFY_SCRIPT,
                        timeout=15,
                        label="Failed startup stop verification",
                    )
                except RuntimeFailure as cleanup_error:
                    print(
                        f"ERROR: automatic startup cleanup also failed: "
                        f"{cleanup_error.message}",
                        file=sys.stderr,
                        flush=True,
                    )
                finally:
                    self._cancel_checks_enabled = previous_cancel_checks
            raise
        print(
            "ROS driver and Klipper bridge are ready. Press Play on the "
            "teach-pendant External Control program.",
            flush=True,
        )

    def wait_external_control(self, *, timeout: object = 90) -> None:
        seconds = self._bounded_int("External Control timeout", timeout, 1, 300)
        self.docker_preflight()
        self._require_container()
        self._container_bash(
            EXTERNAL_CONTROL_SCRIPT,
            env={"TIMEOUT_SECONDS": seconds},
            timeout=float(seconds + 15),
            label="External Control readiness check",
        )

    def start_moveit(self, *, ur_type: object = "ur3e") -> None:
        if str(ur_type) != "ur3e":
            raise RuntimeFailure("Only the calibrated ur3e model is supported.", 64)
        stepper = self._native_extruder(os.environ.get("KLIPPER_STEPPER", "extruder"))
        self.docker_preflight()
        self._require_container()
        # A fresh live program and joint stream are required, matching the
        # workstation's existing online start gate.
        self.wait_external_control(timeout=8)
        self._container_bash(
            """
            set -euo pipefail
            set +u
            source /opt/ros/humble/setup.bash
            source /home/ubuntu/ur_ros2_ws/install/setup.bash
            set -u
            python3 /home/ubuntu/ur_ros2_ws/scripts/klipper_bridge_readiness_probe.py \
              --timeout 2.5 --stepper "$KLIPPER_STEPPER" --json
            """,
            env={"KLIPPER_STEPPER": stepper},
            timeout=12,
            label="ROS Klipper bridge readiness check",
        )
        self._container_bash(
            MOVEIT_START_SCRIPT,
            env={
                "UR_TYPE": "ur3e",
                "KINEMATICS_PARAMS_FILE": robot_kinematics_params_file(),
            },
            timeout=65,
            label="MoveIt startup",
        )

    def stop_moveit(self) -> None:
        self.docker_preflight(require_project=False)
        if not self._container_running():
            print("ROS container is stopped; MoveIt is already stopped.", flush=True)
            return
        self._container_bash(
            MOVEIT_STOP_SCRIPT,
            timeout=20,
            label="MoveIt stop",
        )

    def stop_driver(self) -> None:
        self.docker_preflight(require_project=False)
        if not self._container_running():
            print("ROS container is stopped; the UR driver is already stopped.", flush=True)
            return
        self._cleanup_driver()
        self._container_bash(
            DRIVER_STOP_VERIFY_SCRIPT,
            timeout=20,
            label="UR driver stop verification",
        )

    def stop_ros(self) -> None:
        self.docker_preflight(require_project=False)
        self._compose(
            "stop",
            ROS_SERVICE,
            timeout=90,
            label="ROS container stop",
        )
        if self._container_running():
            raise RuntimeFailure("Docker reported success but the ROS container is still running.")
        print("ROS container stopped.", flush=True)

    def validate_path(
        self,
        *,
        review_id: object,
        review_sha256: object,
        source_waypoint_count: object,
        source_sampled: object,
        tool_roll_deg: object,
        tool_pitch_deg: object,
        tool_yaw_deg: object,
        tcp_offset_x_m: object = 0.0,
        tcp_offset_y_m: object = 0.0,
        tcp_offset_z_m: object = 0.05,
        tool_collision_radius_m: object = 0.01,
        bed_min_x_m: object = -1.0,
        bed_max_x_m: object = 1.0,
        bed_min_y_m: object = -1.0,
        bed_max_y_m: object = 1.0,
        bed_plane_z_m: object = 0.0,
        operation_mode: object = "production_print",
        printhead_geometry_sha256: object = "",
    ) -> Path:
        identifier = str(review_id).strip()
        checksum = str(review_sha256).strip()
        geometry_checksum = str(printhead_geometry_sha256).strip()
        if not re.fullmatch(r"[a-f0-9]{64}", geometry_checksum):
            raise RuntimeFailure("Current printhead geometry checksum is required.", 64)
        if not REVIEW_ID_RE.fullmatch(identifier):
            raise RuntimeFailure("Invalid path-review id.", 64)
        if not SHA256_RE.fullmatch(checksum):
            raise RuntimeFailure("Invalid path-review checksum.", 64)
        count = self._bounded_int(
            "source waypoint count",
            source_waypoint_count,
            1,
            MAX_MOVEIT_WAYPOINTS,
        )
        sampled_text = str(source_sampled).casefold()
        if sampled_text not in {"true", "false"}:
            raise RuntimeFailure("Source sampling state must be true or false.", 64)
        angles = {
            "TOOL_ROLL_DEG": self._finite_angle("tool roll", tool_roll_deg),
            "TOOL_PITCH_DEG": self._finite_angle("tool pitch", tool_pitch_deg),
            "TOOL_YAW_DEG": self._finite_angle("tool yaw", tool_yaw_deg),
        }
        mode = str(operation_mode).strip()
        if mode not in {
            "production_print",
            "tool0_motion_test",
        }:
            raise RuntimeFailure("Invalid operation mode.", 64)
        tcp = {
            "TCP_OFFSET_X_M": self._bounded_float(
                "TCP offset X", tcp_offset_x_m, 0.0, 1.0, absolute=True
            ),
            "TCP_OFFSET_Y_M": self._bounded_float(
                "TCP offset Y", tcp_offset_y_m, 0.0, 1.0, absolute=True
            ),
            "TCP_OFFSET_Z_M": self._bounded_float(
                "TCP offset Z", tcp_offset_z_m, 0.0, 1.0, absolute=True
            ),
            "TOOL_COLLISION_RADIUS_M": self._bounded_float(
                "tool collision radius",
                tool_collision_radius_m,
                0.0
                if mode == "tool0_motion_test"
                else 0.001,
                0.25,
            ),
        }
        if (
            mode == "production_print"
            and math.sqrt(sum(value * value for key, value in tcp.items() if key.startswith("TCP_OFFSET"))) < 0.005
        ):
            raise RuntimeFailure("Measured TCP length must be at least 0.005 m.", 64)
        bed = {
            "BED_MIN_X_M": self._bounded_float("bed minimum X", bed_min_x_m, -5.0, 5.0),
            "BED_MAX_X_M": self._bounded_float("bed maximum X", bed_max_x_m, -5.0, 5.0),
            "BED_MIN_Y_M": self._bounded_float("bed minimum Y", bed_min_y_m, -5.0, 5.0),
            "BED_MAX_Y_M": self._bounded_float("bed maximum Y", bed_max_y_m, -5.0, 5.0),
            "BED_PLANE_Z_M": self._bounded_float("bed plane Z", bed_plane_z_m, -5.0, 5.0),
        }
        if bed["BED_MIN_X_M"] >= bed["BED_MAX_X_M"] or bed["BED_MIN_Y_M"] >= bed["BED_MAX_Y_M"]:
            raise RuntimeFailure("Measured print-bed bounds are inverted.", 64)

        runtime = self.path_review_dir.resolve()
        input_path = (runtime / f"{identifier}.csv").resolve()
        output_path = (runtime / f"{identifier}.json").resolve()
        approach_path = (runtime / f"{identifier}.approach.csv").resolve()
        print_path = (runtime / f"{identifier}.print.csv").resolve()
        if input_path.parent != runtime or output_path.parent != runtime:
            raise RuntimeFailure("Path-review files escaped the managed runtime directory.", 64)
        if not input_path.is_file():
            raise RuntimeFailure("Calibrated path-review input is missing.", 66)
        input_size = input_path.stat().st_size
        if not 1 <= input_size <= MAX_REVIEW_INPUT_BYTES:
            raise RuntimeFailure("Calibrated path-review input has an invalid size.", 66)
        output_path.unlink(missing_ok=True)
        approach_path.unlink(missing_ok=True)
        print_path.unlink(missing_ok=True)

        self.docker_preflight()
        self._require_container()
        container_base = "/home/ubuntu/ur_ros2_ws/runtime/path_review"
        environment: dict[str, object] = {
            "REVIEW_PATH_FILE": f"{container_base}/{identifier}.csv",
            "REVIEW_OUTPUT_FILE": f"{container_base}/{identifier}.json",
            "APPROACH_TRAJECTORY_FILE": f"{container_base}/{identifier}.approach.csv",
            "PRINT_TRAJECTORY_FILE": f"{container_base}/{identifier}.print.csv",
            "REVIEW_SHA256": checksum,
            "SOURCE_WAYPOINT_COUNT": count,
            "SOURCE_SAMPLED": sampled_text,
            "OPERATION_MODE": mode,
            "PRINTHEAD_GEOMETRY_SHA256": geometry_checksum,
            **angles,
            **{key: self._ros_double(value) for key, value in tcp.items()},
            **{key: self._ros_double(value) for key, value in bed.items()},
        }
        print(
            "Starting full-resolution chunked MoveIt plan-only path review. "
            "No trajectory controller or Klipper command will be sent.",
            flush=True,
        )
        self._container_bash(
            PATH_VALIDATION_SCRIPT,
            env=environment,
            timeout=1800,
            label="MoveIt plan-only path review",
        )
        if not output_path.is_file():
            raise RuntimeFailure("MoveIt did not produce a path-review result.", 74)
        output_size = output_path.stat().st_size
        if not 1 <= output_size <= MAX_REVIEW_OUTPUT_BYTES:
            raise RuntimeFailure("MoveIt path-review result has an invalid size.", 74)
        print("MoveIt plan-only path review complete. Execution remains locked.", flush=True)
        return output_path

    def _cartesian_geometry_environment(
        self, environment: Mapping[str, object], *, axes: str, synchronize: bool
    ) -> dict[str, object]:
        path = self.workspace / "config" / "printhead_geometry.json"
        try:
            if path.stat().st_size > 1024 * 1024:
                raise ValueError("geometry file is too large")
            geometry = json.loads(path.read_text(encoding="utf-8"))
            encoded = json.dumps(geometry, sort_keys=True, separators=(",", ":"),
                                 ensure_ascii=True, allow_nan=False).encode("utf-8")
        except (OSError, ValueError, TypeError) as exc:
            raise RuntimeFailure(f"Cannot read installed printhead geometry: {exc}", 66) from exc
        checksum = hashlib.sha256(encoded).hexdigest()
        requested = str(environment.get("PRINTHEAD_GEOMETRY_SHA256", ""))
        if requested and requested != checksum:
            raise RuntimeFailure("Printhead geometry changed. Recalibrate and plan the trial again.", 65)
        trial = self._boolean("height trial mode", environment.get("HEIGHT_TRIAL_MODE", False))
        result: dict[str, object] = {
            "PRINTHEAD_GEOMETRY_SHA256": checksum,
            "HEIGHT_TRIAL_MODE": "true" if trial else "false",
        }
        for key in ("HEIGHT_TRIAL_SURFACE_Z_M", "HEIGHT_TRIAL_MIN_X_M", "HEIGHT_TRIAL_MAX_X_M",
                    "HEIGHT_TRIAL_MIN_Y_M", "HEIGHT_TRIAL_MAX_Y_M"):
            result[key] = self._ros_double(self._bounded_float(key, environment.get(key, 0.0), -5.0, 5.0))
        if trial:
            if axes != "z" or synchronize:
                raise RuntimeFailure("A height trial must be UR-only Z motion.", 65)
            if (float(result["HEIGHT_TRIAL_MIN_X_M"]) >= float(result["HEIGHT_TRIAL_MAX_X_M"])
                    or float(result["HEIGHT_TRIAL_MIN_Y_M"]) >= float(result["HEIGHT_TRIAL_MAX_Y_M"])):
                raise RuntimeFailure("Height trial tabletop bounds are inverted.", 65)
            tcp = [self._bounded_float(key, environment.get(key, ""), -0.5, 0.5)
                   for key in ("HEIGHT_TRIAL_TCP_X_M", "HEIGHT_TRIAL_TCP_Y_M", "HEIGHT_TRIAL_TCP_Z_M")]
            if not 0.01 <= math.sqrt(sum(value * value for value in tcp)) <= 0.5:
                raise RuntimeFailure("Measured tool0-to-nozzle TCP must be 10–500 mm from tool0.", 65)
        else:
            tcp = [10.0, 10.0, 10.0]
        result.update({f"HEIGHT_TRIAL_TCP_{axis}_M": self._ros_double(value)
                       for axis, value in zip(("X", "Y", "Z"), tcp)})
        return result

    def run_cartesian(self, environment: Mapping[str, object] | None = None) -> None:
        """Run the same guarded UR-only Cartesian/height-trial path as macOS."""
        env = os.environ if environment is None else environment
        defaults = {
            "execute": ("EXECUTE", "false"), "axes": ("AXES", "x,y,z"),
            "distance_m": ("DISTANCE_M", "0.05"),
            "max_joint_speed_rad_s": ("MAX_JOINT_SPEED_RAD_S", "0.080"),
            "klipper_stepper": ("KLIPPER_STEPPER", "extruder"),
            "klipper_target_position": ("KLIPPER_TARGET_POSITION", "60.0"),
            "klipper_speed": ("KLIPPER_SPEED", "5.0"),
            "klipper_acceleration": ("KLIPPER_ACCELERATION", "20.0"),
            "klipper_check_only": ("KLIPPER_CHECK_ONLY", "false"),
            "klipper_reset_position": ("KLIPPER_RESET_POSITION", "true"),
            "klipper_disable_after": ("KLIPPER_DISABLE_AFTER", "true"),
            "return_to_start": ("RETURN_TO_START", "false"),
            "confirm_real_robot_move": ("CONFIRM_REAL_ROBOT_MOVE", ""),
            "confirm_cartesian_move": ("CONFIRM_CARTESIAN_MOVE", ""),
            "confirm_klipper_stepper": ("CONFIRM_KLIPPER_STEPPER", ""),
            "eef_step_m": ("EEF_STEP_M", "0.005"), "jump_threshold": ("JUMP_THRESHOLD", "0.0"),
            "velocity_scaling": ("VELOCITY_SCALING", "0.05"),
            "acceleration_scaling": ("ACCELERATION_SCALING", "0.05"),
            "max_segment_delta_rad": ("MAX_SEGMENT_DELTA_RAD", "0.010"),
            "min_segment_duration_s": ("MIN_SEGMENT_DURATION_S", "0.250"),
            "goal_tolerance_rad": ("GOAL_TOLERANCE_RAD", "0.005"),
            "goal_velocity_tolerance_rad_s": ("GOAL_VELOCITY_TOLERANCE_RAD_S", "0.10"),
            "smooth_execution": ("SMOOTH_EXECUTION", "true"),
            "include_velocity_setpoints": ("INCLUDE_VELOCITY_SETPOINTS", "true"),
            "execution_controller": ("EXECUTION_CONTROLLER", "passthrough_trajectory_controller"),
            "switch_controller": ("SWITCH_CONTROLLER", "true"),
            "klipper_outbound_duration_s": ("KLIPPER_OUTBOUND_DURATION_S", "0.0"),
            "synchronize_klipper": ("SYNCHRONIZE_KLIPPER", "false"),
        }
        arguments = {name: env.get(key, default) for name, (key, default) in defaults.items()}
        arguments["allow_15cm"] = env.get("ALLOW_15CM_CARTESIAN_MOVE", "") == "YES"
        self.run_coordinator(**arguments, _ur_only=True, _geometry_environment=env)

    def run_coordinator(
        self,
        *,
        execute: object,
        axes: object,
        distance_m: object,
        max_joint_speed_rad_s: object,
        klipper_stepper: object,
        klipper_target_position: object,
        klipper_speed: object,
        klipper_acceleration: object,
        klipper_check_only: object = False,
        klipper_reset_position: object = False,
        klipper_disable_after: object = True,
        return_to_start: object = False,
        confirm_real_robot_move: object = "",
        confirm_cartesian_move: object = "",
        confirm_klipper_stepper: object = "",
        allow_15cm: object = False,
        eef_step_m: object = 0.005,
        jump_threshold: object = 0.0,
        velocity_scaling: object = 0.05,
        acceleration_scaling: object = 0.05,
        max_segment_delta_rad: object = 0.010,
        min_segment_duration_s: object = 0.250,
        goal_tolerance_rad: object = 0.005,
        goal_velocity_tolerance_rad_s: object = 0.10,
        smooth_execution: object = True,
        include_velocity_setpoints: object = True,
        execution_controller: object = "passthrough_trajectory_controller",
        switch_controller: object = True,
        klipper_outbound_duration_s: object = 0.0,
        synchronize_klipper: object = True,
        _ur_only: bool = False,
        _geometry_environment: Mapping[str, object] | None = None,
    ) -> None:
        execute_value = self._boolean("execute", execute)
        check_only = self._boolean("Klipper check-only", klipper_check_only)
        reset_position = self._boolean(
            "Klipper reset-position",
            klipper_reset_position,
        )
        disable_after = self._boolean(
            "Klipper disable-after",
            klipper_disable_after,
        )
        return_value = self._boolean("return-to-start", return_to_start)
        allow_large = self._boolean("allow-15cm", allow_15cm)
        smooth = self._boolean("smooth execution", smooth_execution)
        velocities = self._boolean(
            "include velocity setpoints",
            include_velocity_setpoints,
        )
        switch = self._boolean("switch controller", switch_controller)
        synchronize = self._boolean(
            "synchronize Klipper",
            synchronize_klipper,
        )
        if not synchronize and not _ur_only:
            raise RuntimeFailure(
                "run-coordinator requires ROS-owned Klipper synchronization.",
                64,
            )
        if execute_value and check_only:
            raise RuntimeFailure(
                "Real execution cannot use Klipper check-only mode.",
                64,
            )
        if synchronize and return_value and not reset_position:
            raise RuntimeFailure(
                "Return-to-start requires resetting the Klipper stepper "
                "position to zero first.",
                64,
            )

        distance_limit = 0.15 if allow_large else 0.10
        distance = self._bounded_float(
            "Cartesian distance",
            distance_m,
            0.001,
            distance_limit,
            absolute=True,
            nonzero=True,
        )
        if abs(distance) > 0.10 and not allow_large:
            raise RuntimeFailure(
                "Cartesian distance above 0.10 m requires the explicit 15 cm opt-in.",
                64,
            )
        joint_speed = self._bounded_float(
            "maximum joint speed",
            max_joint_speed_rad_s,
            0.005,
            0.20,
        )
        eef_step = self._bounded_float("EEF step", eef_step_m, 0.0001, 0.05)
        jump = self._bounded_float("jump threshold", jump_threshold, 0.0, 100.0)
        velocity = self._bounded_float(
            "velocity scaling",
            velocity_scaling,
            0.001,
            1.0,
        )
        acceleration = self._bounded_float(
            "acceleration scaling",
            acceleration_scaling,
            0.001,
            1.0,
        )
        max_segment = self._bounded_float(
            "maximum segment delta",
            max_segment_delta_rad,
            0.0001,
            0.5,
        )
        min_duration = self._bounded_float(
            "minimum segment duration",
            min_segment_duration_s,
            0.001,
            60.0,
        )
        goal_tolerance = self._bounded_float(
            "goal tolerance",
            goal_tolerance_rad,
            0.000001,
            1.0,
        )
        goal_velocity = self._bounded_float(
            "goal velocity tolerance",
            goal_velocity_tolerance_rad_s,
            0.0,
            5.0,
        )
        outbound_duration = self._bounded_float(
            "Klipper outbound duration",
            klipper_outbound_duration_s,
            0.0,
            3600.0,
        )
        target_position = self._bounded_float(
            "Klipper target position",
            klipper_target_position,
            -200.0,
            200.0,
        )
        stepper_speed = self._bounded_float(
            "Klipper speed",
            klipper_speed,
            0.1,
            5.0,
        )
        stepper_acceleration = self._bounded_float(
            "Klipper acceleration",
            klipper_acceleration,
            20.0,
            20.0,
        )
        axes_value = self._axes(axes)
        stepper = self._native_extruder(klipper_stepper)
        controller = self._identifier(
            "execution controller",
            execution_controller,
        )
        if controller not in {
            "passthrough_trajectory_controller",
            "scaled_joint_trajectory_controller",
        }:
            raise RuntimeFailure("Unsupported execution controller.", 64)
        if (
            execute_value
            and smooth
            and controller == "scaled_joint_trajectory_controller"
        ):
            raise RuntimeFailure(
                "Smooth real execution on scaled_joint_trajectory_controller "
                "is disabled; use passthrough_trajectory_controller.",
                65,
            )

        confirmations = {
            "CONFIRM_REAL_ROBOT_MOVE": str(confirm_real_robot_move),
            "CONFIRM_CARTESIAN_MOVE": str(confirm_cartesian_move),
            "CONFIRM_KLIPPER_STEPPER": str(confirm_klipper_stepper),
        }
        if execute_value:
            missing = [
                name
                for name, value in confirmations.items()
                if value != "YES" and (synchronize or name != "CONFIRM_KLIPPER_STEPPER")
            ]
            if missing:
                raise RuntimeFailure(
                    "Real execution requires exact YES confirmations for the UR and Cartesian move, "
                    "and for the Klipper stepper when synchronized.",
                    64,
                )
        elif any(value for value in confirmations.values()):
            raise RuntimeFailure(
                "Plan-only coordination must not carry real-motion confirmations.",
                64,
            )

        environment: dict[str, object] = {
            "EXECUTE": "true" if execute_value else "false",
            "AXES": axes_value,
            "DISTANCE_M": self._ros_double(distance),
            "EEF_STEP_M": self._ros_double(eef_step),
            "JUMP_THRESHOLD": self._ros_double(jump),
            "VELOCITY_SCALING": self._ros_double(velocity),
            "ACCELERATION_SCALING": self._ros_double(acceleration),
            "MAX_SEGMENT_DELTA_RAD": self._ros_double(max_segment),
            "MAX_JOINT_SPEED_RAD_S": self._ros_double(joint_speed),
            "MIN_SEGMENT_DURATION_S": self._ros_double(min_duration),
            "GOAL_TOLERANCE_RAD": self._ros_double(goal_tolerance),
            "GOAL_VELOCITY_TOLERANCE_RAD_S": self._ros_double(goal_velocity),
            "SMOOTH_EXECUTION": "true" if smooth else "false",
            "INCLUDE_VELOCITY_SETPOINTS": "true" if velocities else "false",
            "EXECUTION_CONTROLLER": controller,
            "SWITCH_CONTROLLER": "true" if switch else "false",
            "RETURN_TO_START": "true" if return_value else "false",
            "KLIPPER_OUTBOUND_DURATION_S": self._ros_double(outbound_duration),
            "SYNCHRONIZE_KLIPPER": "true" if synchronize else "false",
            "KLIPPER_CHECK_ONLY": "true" if check_only else "false",
            "KLIPPER_STEPPER": stepper,
            "KLIPPER_TARGET_POSITION": self._ros_double(target_position),
            "KLIPPER_SPEED": self._ros_double(stepper_speed),
            "KLIPPER_ACCELERATION": self._ros_double(stepper_acceleration),
            "KLIPPER_RESET_POSITION": "true" if reset_position else "false",
            "KLIPPER_DISABLE_AFTER": "true" if disable_after else "false",
            "ALLOW_15CM_CARTESIAN_MOVE": "YES" if allow_large else "NO",
        }
        if execute_value:
            environment.update({key: value for key, value in confirmations.items() if value})
        environment.update(self._cartesian_geometry_environment(
            _geometry_environment or {}, axes=axes_value, synchronize=synchronize,
        ))
        self.docker_preflight()
        self._require_container()
        operation = (
            ("synchronized UR and Klipper execution" if synchronize else "guarded UR Cartesian execution")
            if execute_value else
            ("ROS bridge and MoveIt plan-only coordination" if synchronize else "guarded UR Cartesian plan-only review")
        )
        print(f"Starting {operation}.", flush=True)
        self._container_bash(
            COORDINATOR_SCRIPT,
            env=environment,
            timeout=1800 if execute_value else 600,
            label=operation,
        )
        print(f"Completed {operation}.", flush=True)


def _environment_default(name: str, fallback: str) -> str:
    value = os.environ.get(name)
    return fallback if value is None else value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Windows-compatible UR Print Workstation ROS orchestrator. "
            "Docker Desktop must use Linux containers."
        )
    )
    subcommands = parser.add_subparsers(dest="action", required=True)

    start = subcommands.add_parser("start-ros")
    start.add_argument("--robot-ip", default=_environment_default("ROBOT_IP", ""))
    start.add_argument("--ros-pc-ip", default=_environment_default("ROS_PC_IP", ""))
    start.add_argument(
        "--moonraker-url",
        default=_environment_default("MOONRAKER_URL", "http://192.168.0.168"),
    )
    start.add_argument(
        "--stepper",
        default=_environment_default("KLIPPER_STEPPER", "extruder"),
    )
    start.add_argument(
        "--update-rate",
        default=_environment_default("UR_UPDATE_RATE", "125"),
    )
    start.add_argument(
        "--desktop-port",
        default=_environment_default("ROS_DESKTOP_PORT", "6082"),
    )
    start.add_argument(
        "--build-policy",
        choices=("auto", "always", "never"),
        default="auto",
    )
    start.add_argument("--startup-timeout", default="60")
    start.add_argument("--stability-seconds", default="45")
    start.add_argument(
        "--non-blocking-read", action="store_true",
        default=_environment_default("UR_NON_BLOCKING_READ", "true").casefold() in {"true", "1"},
    )
    start.add_argument("--blocking-read", dest="non_blocking_read", action="store_false")
    start.add_argument(
        "--keep-robot-state-helper",
        action="store_true",
        help="Do not apply the proven robot_state_helper-disable runtime setting.",
    )

    external = subcommands.add_parser("wait-external-control")
    external.add_argument(
        "--timeout",
        default=_environment_default("TIMEOUT_SECONDS", "90"),
    )

    moveit = subcommands.add_parser("start-moveit")
    moveit.add_argument("--ur-type", default="ur3e")

    subcommands.add_parser("stop-moveit")
    subcommands.add_parser("stop-driver")
    subcommands.add_parser("stop-ros")
    subcommands.add_parser("cartesian-move")

    coordinator = subcommands.add_parser("run-coordinator")
    coordinator.add_argument(
        "--execute",
        default=_environment_default("EXECUTE", "false"),
    )
    coordinator.add_argument("--axes", default=_environment_default("AXES", "x,y,z"))
    coordinator.add_argument(
        "--distance-m",
        default=_environment_default("DISTANCE_M", "0.02"),
    )
    coordinator.add_argument(
        "--max-joint-speed-rad-s",
        default=_environment_default("MAX_JOINT_SPEED_RAD_S", "0.03"),
    )
    coordinator.add_argument(
        "--klipper-stepper",
        default=_environment_default("KLIPPER_STEPPER", "extruder"),
    )
    coordinator.add_argument(
        "--klipper-target-position",
        default=_environment_default("KLIPPER_TARGET_POSITION", "60.0"),
    )
    coordinator.add_argument(
        "--klipper-speed",
        default=_environment_default("KLIPPER_SPEED", "5.0"),
    )
    coordinator.add_argument(
        "--klipper-acceleration",
        default=_environment_default("KLIPPER_ACCELERATION", "20.0"),
    )
    coordinator.add_argument(
        "--klipper-check-only",
        default=_environment_default("KLIPPER_CHECK_ONLY", "false"),
    )
    coordinator.add_argument(
        "--klipper-reset-position",
        default=_environment_default("KLIPPER_RESET_POSITION", "true"),
    )
    coordinator.add_argument(
        "--klipper-disable-after",
        default=_environment_default("KLIPPER_DISABLE_AFTER", "true"),
    )
    coordinator.add_argument(
        "--return-to-start",
        default=_environment_default("RETURN_TO_START", "false"),
    )
    coordinator.add_argument(
        "--confirm-real-robot-move",
        default=_environment_default("CONFIRM_REAL_ROBOT_MOVE", ""),
    )
    coordinator.add_argument(
        "--confirm-cartesian-move",
        default=_environment_default("CONFIRM_CARTESIAN_MOVE", ""),
    )
    coordinator.add_argument(
        "--confirm-klipper-stepper",
        default=_environment_default("CONFIRM_KLIPPER_STEPPER", ""),
    )
    coordinator.add_argument(
        "--allow-15cm",
        default=(
            "true"
            if _environment_default("ALLOW_15CM_CARTESIAN_MOVE", "") == "YES"
            else "false"
        ),
    )
    coordinator.add_argument(
        "--eef-step-m",
        default=_environment_default("EEF_STEP_M", "0.005"),
    )
    coordinator.add_argument(
        "--jump-threshold",
        default=_environment_default("JUMP_THRESHOLD", "0.0"),
    )
    coordinator.add_argument(
        "--velocity-scaling",
        default=_environment_default("VELOCITY_SCALING", "0.05"),
    )
    coordinator.add_argument(
        "--acceleration-scaling",
        default=_environment_default("ACCELERATION_SCALING", "0.05"),
    )
    coordinator.add_argument(
        "--max-segment-delta-rad",
        default=_environment_default("MAX_SEGMENT_DELTA_RAD", "0.010"),
    )
    coordinator.add_argument(
        "--min-segment-duration-s",
        default=_environment_default("MIN_SEGMENT_DURATION_S", "0.250"),
    )
    coordinator.add_argument(
        "--goal-tolerance-rad",
        default=_environment_default("GOAL_TOLERANCE_RAD", "0.005"),
    )
    coordinator.add_argument(
        "--goal-velocity-tolerance-rad-s",
        default=_environment_default("GOAL_VELOCITY_TOLERANCE_RAD_S", "0.10"),
    )
    coordinator.add_argument(
        "--smooth-execution",
        default=_environment_default("SMOOTH_EXECUTION", "true"),
    )
    coordinator.add_argument(
        "--include-velocity-setpoints",
        default=_environment_default("INCLUDE_VELOCITY_SETPOINTS", "true"),
    )
    coordinator.add_argument(
        "--execution-controller",
        default=_environment_default(
            "EXECUTION_CONTROLLER",
            "passthrough_trajectory_controller",
        ),
    )
    coordinator.add_argument(
        "--switch-controller",
        default=_environment_default("SWITCH_CONTROLLER", "true"),
    )
    coordinator.add_argument(
        "--klipper-outbound-duration-s",
        default=_environment_default("KLIPPER_OUTBOUND_DURATION_S", "0.0"),
    )
    coordinator.add_argument(
        "--synchronize-klipper",
        default=_environment_default("SYNCHRONIZE_KLIPPER", "true"),
    )

    validate = subcommands.add_parser("validate-path")
    validate.add_argument("review_id")
    validate.add_argument("review_sha256")
    validate.add_argument("source_waypoint_count")
    validate.add_argument("source_sampled")
    validate.add_argument("tool_roll_deg")
    validate.add_argument("tool_pitch_deg")
    validate.add_argument("tool_yaw_deg")
    validate.add_argument("tcp_offset_x_m", nargs="?", default="0.0")
    validate.add_argument("tcp_offset_y_m", nargs="?", default="0.0")
    validate.add_argument("tcp_offset_z_m", nargs="?", default="0.05")
    validate.add_argument("tool_collision_radius_m", nargs="?", default="0.01")
    validate.add_argument("bed_min_x_m", nargs="?", default="-1.0")
    validate.add_argument("bed_max_x_m", nargs="?", default="1.0")
    validate.add_argument("bed_min_y_m", nargs="?", default="-1.0")
    validate.add_argument("bed_max_y_m", nargs="?", default="1.0")
    validate.add_argument("bed_plane_z_m", nargs="?", default="0.0")
    validate.add_argument("operation_mode", nargs="?", default="production_print")
    validate.add_argument("printhead_geometry_sha256", nargs="?", default="")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    runtime = WindowsRuntime()
    try:
        if args.action == "start-ros":
            runtime.start_ros(
                robot_ip=args.robot_ip,
                ros_pc_ip=args.ros_pc_ip,
                moonraker_url=args.moonraker_url,
                stepper=args.stepper,
                update_rate=args.update_rate,
                desktop_port=args.desktop_port,
                build_policy=args.build_policy,
                startup_timeout=args.startup_timeout,
                stability_seconds=args.stability_seconds,
                non_blocking_read=args.non_blocking_read,
                skip_robot_state_helper=not args.keep_robot_state_helper,
            )
        elif args.action == "wait-external-control":
            runtime.wait_external_control(timeout=args.timeout)
        elif args.action == "start-moveit":
            runtime.start_moveit(ur_type=args.ur_type)
        elif args.action == "stop-moveit":
            runtime.stop_moveit()
        elif args.action == "stop-driver":
            runtime.stop_driver()
        elif args.action == "stop-ros":
            runtime.stop_ros()
        elif args.action == "cartesian-move":
            runtime.run_cartesian()
        elif args.action == "run-coordinator":
            runtime.run_coordinator(
                execute=args.execute,
                axes=args.axes,
                distance_m=args.distance_m,
                max_joint_speed_rad_s=args.max_joint_speed_rad_s,
                klipper_stepper=args.klipper_stepper,
                klipper_target_position=args.klipper_target_position,
                klipper_speed=args.klipper_speed,
                klipper_acceleration=args.klipper_acceleration,
                klipper_check_only=args.klipper_check_only,
                klipper_reset_position=args.klipper_reset_position,
                klipper_disable_after=args.klipper_disable_after,
                return_to_start=args.return_to_start,
                confirm_real_robot_move=args.confirm_real_robot_move,
                confirm_cartesian_move=args.confirm_cartesian_move,
                confirm_klipper_stepper=args.confirm_klipper_stepper,
                allow_15cm=args.allow_15cm,
                eef_step_m=args.eef_step_m,
                jump_threshold=args.jump_threshold,
                velocity_scaling=args.velocity_scaling,
                acceleration_scaling=args.acceleration_scaling,
                max_segment_delta_rad=args.max_segment_delta_rad,
                min_segment_duration_s=args.min_segment_duration_s,
                goal_tolerance_rad=args.goal_tolerance_rad,
                goal_velocity_tolerance_rad_s=(
                    args.goal_velocity_tolerance_rad_s
                ),
                smooth_execution=args.smooth_execution,
                include_velocity_setpoints=args.include_velocity_setpoints,
                execution_controller=args.execution_controller,
                switch_controller=args.switch_controller,
                klipper_outbound_duration_s=args.klipper_outbound_duration_s,
                synchronize_klipper=args.synchronize_klipper,
            )
        elif args.action == "validate-path":
            runtime.validate_path(
                review_id=args.review_id,
                review_sha256=args.review_sha256,
                source_waypoint_count=args.source_waypoint_count,
                source_sampled=args.source_sampled,
                tool_roll_deg=args.tool_roll_deg,
                tool_pitch_deg=args.tool_pitch_deg,
                tool_yaw_deg=args.tool_yaw_deg,
                tcp_offset_x_m=args.tcp_offset_x_m,
                tcp_offset_y_m=args.tcp_offset_y_m,
                tcp_offset_z_m=args.tcp_offset_z_m,
                tool_collision_radius_m=args.tool_collision_radius_m,
                bed_min_x_m=args.bed_min_x_m,
                bed_max_x_m=args.bed_max_x_m,
                bed_min_y_m=args.bed_min_y_m,
                bed_max_y_m=args.bed_max_y_m,
                bed_plane_z_m=args.bed_plane_z_m,
                operation_mode=args.operation_mode,
                printhead_geometry_sha256=args.printhead_geometry_sha256,
            )
        else:  # argparse requires a known action
            raise RuntimeFailure(f"Unsupported action: {args.action}", 64)
        return 0
    except RuntimeFailure as exc:
        print(f"ERROR: {exc.message}", file=sys.stderr, flush=True)
        return exc.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
