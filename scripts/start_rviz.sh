#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

cd "$ROOT_DIR"

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  pkill -x mate-screensaver || true
  pkill -x rviz2 || true

  export DISPLAY=:1
  export XAUTHORITY=/home/ubuntu/.Xauthority
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi

  publisher_pid_file=/home/ubuntu/ur_ros2_ws/workcell_preview.pid
  if [ -f "$publisher_pid_file" ]; then
    publisher_pid="$(tr -cd "0-9" < "$publisher_pid_file")"
    if [ -n "$publisher_pid" ] && [ -r "/proc/$publisher_pid/cmdline" ] && \
      tr "\0" " " < "/proc/$publisher_pid/cmdline" | grep -Fq "/home/ubuntu/ur_ros2_ws/scripts/publish_workcell_preview.py"; then
      kill "$publisher_pid" || true
    fi
  fi
  nohup python3 /home/ubuntu/ur_ros2_ws/scripts/publish_workcell_preview.py \
    > /home/ubuntu/ur_ros2_ws/workcell_preview.log 2>&1 &
  echo "$!" > "$publisher_pid_file"

  nohup rviz2 -d /home/ubuntu/ur_ros2_ws/src/ur3e_print_planner/config/view_robot_single_arm.rviz \
    > /home/ubuntu/ur_ros2_ws/rviz2.log 2>&1 &
  echo "rviz2 pid: $!"
'

echo
echo "Open the ROS desktop at: http://localhost:6082/vnc.html"
echo "VNC password: ubuntu"
