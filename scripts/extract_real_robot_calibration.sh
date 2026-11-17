#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

ROBOT_IP="${ROBOT_IP:-${1:-}}"
TARGET_FILE="${TARGET_FILE:-/home/ubuntu/ur_ros2_ws/src/ur3e_print_planner/config/ur3e_real_calibration.yaml}"
BUILD_IMAGE="${BUILD_IMAGE:-auto}"

if [[ -z "$ROBOT_IP" ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=<robot_ip> ./scripts/extract_real_robot_calibration.sh

Optional:
  TARGET_FILE=/home/ubuntu/ur_ros2_ws/src/ur3e_print_planner/config/my_robot_calibration.yaml
EOF
  exit 64
fi

"$ROOT_DIR/scripts/install_runtime.sh"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"

cd "$ROOT_DIR"
if [[ "$BUILD_IMAGE" == "1" || "$BUILD_IMAGE" == "true" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --build
elif [[ "$BUILD_IMAGE" == "0" || "$BUILD_IMAGE" == "false" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
elif docker image inspect ur3e-real-ros2 >/dev/null 2>&1; then
  echo "Using existing local Docker image ur3e-real-ros2."
  echo "Set BUILD_IMAGE=1 if you want to rebuild it while connected to the internet."
  docker compose -f "$COMPOSE_FILE" up -d --no-build
else
  docker compose -f "$COMPOSE_FILE" up -d --build
fi

docker compose -f "$COMPOSE_FILE" exec -u ubuntu ros2 bash -lc "
source /opt/ros/humble/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=$ROBOT_IP \
  target_filename:=$TARGET_FILE
"

echo "Saved calibration to: $TARGET_FILE"
