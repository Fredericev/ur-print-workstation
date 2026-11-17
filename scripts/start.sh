#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${ROBOT_IP:-}" && $# -eq 0 ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=192.168.0.104 ./scripts/start.sh

This project is now real-robot only. It starts the stable UR3e ROS driver.
EOF
  exit 64
fi

"$ROOT_DIR/scripts/launch_real_robot_stable.sh" "$@"
