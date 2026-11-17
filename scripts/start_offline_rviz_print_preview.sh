#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$ROOT_DIR/workspaces/ur_ros2_ws"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
MANIFEST="latest"
SPEED_FACTOR="10"
FORBIDDEN_PROCESS_PATTERN='[u]r_ros2_control_node|[d]ashboard_client|[c]ontroller_manager|[r]os2_control_node|[u]rscript_interface|[k]lipper_bridge|[g]code_print_executor|[t]ool_communication|[r]obot_state_helper|[c]ontroller_stopper_node|[t]rajectory_until_node'

assert_no_hardware_processes() {
  local found
  if found="$(docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 \
      bash -lc "pgrep -af '$FORBIDDEN_PROCESS_PATTERN'")"; then
    echo "ERROR: Hardware-control processes are running; offline RViz replay was refused." >&2
    echo "$found" >&2
    echo "Stop the workstation ROS driver/executor first. This helper will not kill them." >&2
    exit 78
  fi
}

usage() {
  echo "Usage: $0 [--manifest latest|PATH] [--speed-factor NUMBER]"
  echo "Starts an isolated RViz-only replay. It never starts the UR driver or Klipper bridge."
}

while (($#)); do
  case "$1" in
    --manifest)
      MANIFEST="${2:?--manifest requires a value}"
      shift 2
      ;;
    --speed-factor)
      SPEED_FACTOR="${2:?--speed-factor requires a value}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      echo "Unknown argument: $1" >&2
      exit 64
      ;;
  esac
done

cd "$ROOT_DIR"

HOST_MANIFEST="$MANIFEST"
CONTAINER_MANIFEST="$MANIFEST"
if [[ "$MANIFEST" != "latest" ]]; then
  HOST_MANIFEST="$(python3 - "$MANIFEST" <<'PY'
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
)"
  case "$HOST_MANIFEST" in
    "$WORKSPACE_DIR"/*)
      CONTAINER_MANIFEST="/home/ubuntu/ur_ros2_ws/${HOST_MANIFEST#"$WORKSPACE_DIR"/}"
      ;;
    *)
      echo "Manifest must be inside $WORKSPACE_DIR" >&2
      exit 64
      ;;
  esac
fi

# This validation is ROS-free and verifies the source/review/CSV checksums,
# complete print point count, path continuity, and display speed before any
# container process is started.
python3 "$WORKSPACE_DIR/scripts/rviz_offline_print_playback.py" \
  --manifest "$HOST_MANIFEST" --speed-factor "$SPEED_FACTOR" --check-only
SOURCE_PATH="$(python3 "$WORKSPACE_DIR/scripts/rviz_offline_print_playback.py" \
  --manifest "$HOST_MANIFEST" --speed-factor "$SPEED_FACTOR" --source-path-only)"
python3 "$ROOT_DIR/scripts/export_rviz_job_preview.py" "$SOURCE_PATH"

"$ROOT_DIR/scripts/start_ros.sh"

# `docker compose up` can return before the VNC X server has created display
# :1. Starting RViz during that interval makes Qt exit while the ROS launch
# remains alive, so wait for the graphical desktop itself rather than merely
# the container/HTTP port.
desktop_ready=false
for _attempt in $(seq 1 60); do
  if docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc \
      'test -S /tmp/.X11-unix/X1 && test -r /home/ubuntu/.Xauthority && DISPLAY=:1 XAUTHORITY=/home/ubuntu/.Xauthority xdpyinfo >/dev/null 2>&1'; then
    desktop_ready=true
    break
  fi
  sleep 1
done
if [[ "$desktop_ready" != "true" ]]; then
  echo "ERROR: The ROS graphical desktop did not become ready on display :1." >&2
  docker compose -f "$COMPOSE_FILE" logs --tail=120 ros2 >&2 || true
  exit 75
fi

# Refuse every known hardware-control process. This helper never terminates
# such a process on the user's behalf.
assert_no_hardware_processes

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/ubuntu/ur_ros2_ws
  colcon build --packages-select ur3e_print_planner --symlink-install
'

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  set -e
  pid_file=/home/ubuntu/ur_ros2_ws/offline_rviz_preview.pid
  if [ -f "$pid_file" ]; then
    old_pid="$(tr -cd "0-9" < "$pid_file")"
    if [ -n "$old_pid" ] && [ -r "/proc/$old_pid/cmdline" ] &&
       tr "\0" " " < "/proc/$old_pid/cmdline" | grep -Fq offline_print_preview.launch.py; then
      old_pgid="$(ps -o pgid= -p "$old_pid" | tr -d " " || true)"
      if [ "$old_pgid" = "$old_pid" ]; then
        kill -TERM -- "-$old_pid" 2>/dev/null || true
      else
        kill -TERM "$old_pid" 2>/dev/null || true
      fi
    fi
  fi
  rm -f "$pid_file"
'

# Remove only orphaned processes created by an earlier offline preview. The
# bracketed patterns cannot match this cleanup shell itself.
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  for signal in TERM KILL; do
    pkill -"$signal" -f "[o]ffline_print_preview.launch.py" 2>/dev/null || true
    pkill -"$signal" -f "__node:=[o]ffline_preview_robot_state_publisher" 2>/dev/null || true
    pkill -"$signal" -f "__node:=[o]ffline_print_preview_rviz" 2>/dev/null || true
    pkill -"$signal" -f "[p]ublish_workcell_preview.py" 2>/dev/null || true
    pkill -"$signal" -f "[r]viz_offline_print_playback.py" 2>/dev/null || true
    if [ "$signal" = TERM ]; then sleep 1; fi
  done
'

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
  pid_file=/home/ubuntu/ur_ros2_ws/offline_rviz_preview.pid
  export DISPLAY=:1
  export XAUTHORITY=/home/ubuntu/.Xauthority
  export ROS_DOMAIN_ID=84
  export ROS_LOCALHOST_ONLY=1
  : > /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log
  nohup setsid ros2 launch ur3e_print_planner offline_print_preview.launch.py \
    workspace_root:=/home/ubuntu/ur_ros2_ws \
    project_root:=/home/ubuntu/offline_preview_project \
    manifest:='$CONTAINER_MANIFEST' \
    speed_factor:='$SPEED_FACTOR' \
    > /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log 2>&1 &
  echo \"\$!\" > \"\$pid_file\"
"

published=false
for _attempt in $(seq 1 45); do
  if docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 \
      grep -q 'published_topic.*display_planned_path' \
      /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log; then
    published=true
    break
  fi
  sleep 1
done

if [[ "$published" != "true" ]]; then
  echo "ERROR: RViz offline playback did not become ready." >&2
  docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 \
    tail -n 120 /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log >&2 || true
  exit 75
fi

# Recheck after launch so a concurrent workstation action cannot silently mix
# hardware execution into the isolated display session.
assert_no_hardware_processes

echo
echo "Offline RViz print replay is running in isolated ROS_DOMAIN_ID=84."
echo "Open: http://localhost:6082/vnc.html"
echo "VNC password: ubuntu"
echo "Playback log: /home/ubuntu/ur_ros2_ws/offline_rviz_preview.log"
echo "The cached animation is display-only; no UR or Klipper command was created."
