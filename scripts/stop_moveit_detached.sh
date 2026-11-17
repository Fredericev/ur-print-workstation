#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
PID_FILE="/home/ubuntu/ur_ros2_ws/move_group_live.pid"

cd "$ROOT_DIR"

if ! docker compose -f "$COMPOSE_FILE" ps --status running --services | grep -qx ros2; then
  echo "ROS container is not running; MoveIt is already stopped."
  exit 0
fi

echo "Stopping MoveIt while keeping the ROS container and UR driver available..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  set +e
  if [ -f '$PID_FILE' ]; then
    /home/ubuntu/ur_ros2_ws/scripts/verified_pid_signal.sh \
      '$PID_FILE' '[r]os2 launch ur_moveit_config' TERM || exit \$?
  fi
  pkill -TERM -f '[r]os2 launch ur_moveit_config' 2>/dev/null || true
  pkill -TERM -x move_group 2>/dev/null || true

  deadline=\$((SECONDS + 8))
  while pgrep -x move_group >/dev/null 2>&1 && [ \$SECONDS -lt \$deadline ]; do
    sleep 0.25
  done
  pkill -KILL -f '[r]os2 launch ur_moveit_config' 2>/dev/null || true
  pkill -KILL -x move_group 2>/dev/null || true
  rm -f '$PID_FILE'

  if pgrep -x move_group >/dev/null 2>&1; then
    echo 'ERROR: MoveIt is still running.'
    exit 1
  fi
  echo 'MoveIt stopped. UR driver and ROS container were left running.'
"
