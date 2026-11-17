#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

cd "$ROOT_DIR"

echo "Sending a no-motion hold goal to the active UR3e trajectory controller..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  python3 /home/ubuntu/ur_ros2_ws/scripts/ur3e_joint_goal.py \
    --mode hold \
    --duration "${UR_HOLD_DURATION:-30.0}" \
    --state-timeout "${UR_STATE_TIMEOUT:-20.0}" \
    --allow-latched-program-state
'
