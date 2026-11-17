#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.0.104}"

if [[ $# -eq 0 ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=192.168.0.104 ./scripts/ur_dashboard.sh "is in remote control"
  ROBOT_IP=192.168.0.104 ./scripts/ur_dashboard.sh robotmode

This sends one Dashboard Server command to port 29999.
Be careful with commands that start programs or change robot state.
EOF
  exit 64
fi

"$ROOT_DIR/scripts/ur_dashboard_command.py" "$ROBOT_IP" "$@"
