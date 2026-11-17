#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
CLOCKING="0"
ROS_DESKTOP_PORT="${ROS_DESKTOP_PORT:-6082}"
ROS_DOMAIN_ID="${PRINTHEAD_PREVIEW_ROS_DOMAIN_ID:-86}"
FORBIDDEN_PROCESS_PATTERN='[u]r_ros2_control_node|[d]ashboard_client|[c]ontroller_manager|[r]os2_control_node|[u]rscript_interface|[k]lipper_bridge|[g]code_print_executor|[t]ool_communication|[r]obot_state_helper|[c]ontroller_stopper_node|[t]rajectory_until_node'

usage() {
  cat <<EOF
Usage: $0 [--clocking 0|90|180|270]

Starts an offline ROS 2 UR3e + printhead placement check in RViz.
It never starts the UR driver, a controller, URScript, MoveIt execution, or Klipper.
EOF
}

while (($#)); do
  case "$1" in
    --clocking)
      CLOCKING="${2:?--clocking requires 0, 90, 180, or 270}"
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

case "$CLOCKING" in
  0|90|180|270) ;;
  *)
    echo "ERROR: --clocking must be 0, 90, 180, or 270." >&2
    exit 64
    ;;
esac

assert_no_hardware_processes() {
  local found
  if found="$(docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 \
      bash -lc "pgrep -af '$FORBIDDEN_PROCESS_PATTERN'")"; then
    echo "ERROR: Hardware-control processes are running; the offline preview was refused." >&2
    echo "$found" >&2
    echo "Stop the real-robot driver/executor first. This script will not terminate it." >&2
    exit 78
  fi
}

source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
cd "$ROOT_DIR"
export ROS_DESKTOP_PORT

if docker image inspect ur3e-real-ros2:latest >/dev/null 2>&1; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build ros2
else
  docker compose -f "$COMPOSE_FILE" up -d --build ros2
fi

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

assert_no_hardware_processes

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/ubuntu/ur_ros2_ws
  colcon build --packages-select ur3e_printhead_description --symlink-install
'

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  set -e
  pid_file=/home/ubuntu/ur_ros2_ws/printhead_rviz_preview.pid
  if [ -f "$pid_file" ]; then
    old_pid="$(tr -cd "0-9" < "$pid_file")"
    if [ -n "$old_pid" ] && [ -r "/proc/$old_pid/cmdline" ] &&
       tr "\0" " " < "/proc/$old_pid/cmdline" | grep -Fq view_printhead.launch.py; then
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

# Remove only processes carrying this preview's unique launch/node names.
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  for signal in TERM KILL; do
    pkill -"$signal" -f "[v]iew_printhead.launch.py" 2>/dev/null || true
    pkill -"$signal" -f "__node:=[p]rinthead_preview_robot_state_publisher" 2>/dev/null || true
    pkill -"$signal" -f "__node:=[p]rinthead_preview_joint_state_publisher_gui" 2>/dev/null || true
    pkill -"$signal" -f "__node:=[p]rinthead_preview_rviz" 2>/dev/null || true
    if [ "$signal" = TERM ]; then sleep 1; fi
  done
'

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ubuntu/.Xauthority
  export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
  export ROS_LOCALHOST_ONLY=1
  : > /home/ubuntu/ur_ros2_ws/printhead_rviz_preview.log
  nohup setsid ros2 launch ur3e_printhead_description view_printhead.launch.py \
    clocking:='$CLOCKING' \
    kinematics_params_file:=/home/ubuntu/ur_ros2_ws/config/ur3e_actual_calibration.yaml \
    > /home/ubuntu/ur_ros2_ws/printhead_rviz_preview.log 2>&1 &
  echo \"\$!\" > /home/ubuntu/ur_ros2_ws/printhead_rviz_preview.pid
"

preview_ready=false
for _attempt in $(seq 1 45); do
  if docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc \
      "export ROS_DOMAIN_ID='$ROS_DOMAIN_ID' ROS_LOCALHOST_ONLY=1; source /opt/ros/humble/setup.bash; ros2 node list 2>/dev/null | grep -Fxq /printhead_preview_rviz && ros2 node list 2>/dev/null | grep -Fxq /printhead_preview_joint_state_publisher_gui"; then
    preview_ready=true
    break
  fi
  sleep 1
done

if [[ "$preview_ready" != "true" ]]; then
  echo "ERROR: The UR3e printhead RViz preview did not become ready." >&2
  docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 \
    tail -n 160 /home/ubuntu/ur_ros2_ws/printhead_rviz_preview.log >&2 || true
  exit 75
fi

assert_no_hardware_processes

echo
echo "UR3e + printhead ROS 2 RViz preview is running (clocking ${CLOCKING} degrees)."
echo "Open: http://localhost:${ROS_DESKTOP_PORT}/vnc.html"
echo "VNC password: ${ROS_DESKTOP_PASSWORD:-ubuntu}"
echo "Use the Joint State Publisher window to move the six UR3e joints."
echo "ROS isolation: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=1"
echo "No UR driver, controller, URScript, MoveIt execution, or Klipper node was started."
