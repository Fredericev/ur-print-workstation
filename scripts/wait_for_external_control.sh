#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
LOG_FILE="/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log"
PROBE_FILE="/home/ubuntu/ur_ros2_ws/scripts/external_control_readiness_probe.py"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-90}"

if ! [[ "$TIMEOUT_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: TIMEOUT_SECONDS must be a positive number." >&2
  exit 64
fi

cd "$ROOT_DIR"

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi

  status=0
  python3 '$PROBE_FILE' --timeout '$TIMEOUT_SECONDS' || status=\$?
  if [ \"\$status\" -ne 0 ]; then
    echo 'Press Play on the teach-pendant program that contains External Control, then run this step again.' >&2
    echo 'Recent relevant UR driver events:' >&2
    grep -E 'Robot requested program|Sent program to robot|Robot connected to reverse interface|Connection to reverse interface dropped|Unable to read from hardware|Reconnecting to the RTDE interface|process has died|speed_slider_mask|External Control speed limit' '$LOG_FILE' 2>/dev/null | tail -n 20 >&2 || true
  fi
  exit \"\$status\"
"
