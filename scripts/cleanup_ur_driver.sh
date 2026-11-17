#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

cd "$ROOT_DIR"

echo "Stopping stale UR driver processes in the ROS container..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
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

  show_matches() {
    local pattern
    for pattern in "${patterns[@]}"; do
      pgrep -af "$pattern" || true
    done
  }

  echo "Before cleanup:"
  show_matches

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done

  pkill -f "[r]os2cli.daemon.daemonize" 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* 2>/dev/null || true

  echo "After cleanup:"
  show_matches
'
