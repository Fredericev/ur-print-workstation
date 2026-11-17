#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

cd "$ROOT_DIR"

if ! docker compose -f "$COMPOSE_FILE" ps --status running --services | grep -qx ros2; then
  echo "ROS container is not running; the UR driver is already stopped."
  exit 0
fi

echo "Stopping MoveIt and the UR ROS driver while keeping the ROS container available..."
"$ROOT_DIR/scripts/cleanup_ur_driver.sh"

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
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
  echo "UR driver, MoveIt, and the ROS Klipper bridge stopped. ROS container was left running."
'
