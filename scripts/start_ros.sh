#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
BUILD_IMAGE="${BUILD_IMAGE:-auto}"
ROS_DESKTOP_PORT="${ROS_DESKTOP_PORT:-6082}"
UR_TYPE="${UR_TYPE:-ur3e}"
UR_UPDATE_RATE="${UR_UPDATE_RATE:-125}"

"$ROOT_DIR/scripts/install_runtime.sh"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
UR_TYPE="$UR_TYPE" UR_UPDATE_RATE="$UR_UPDATE_RATE" "$ROOT_DIR/scripts/prepare_ur_update_rate.sh"

cd "$ROOT_DIR"
UR_REVERSE_BIND_IP="${UR_REVERSE_BIND_IP:-127.0.0.1}"
export ROS_DESKTOP_PORT UR_TYPE UR_REVERSE_BIND_IP
if [[ "$BUILD_IMAGE" == "1" || "$BUILD_IMAGE" == "true" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --build
elif [[ "$BUILD_IMAGE" == "0" || "$BUILD_IMAGE" == "false" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
elif docker image inspect ur3e-real-ros2 >/dev/null 2>&1; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
else
  docker compose -f "$COMPOSE_FILE" up -d --build
fi

python3 "$ROOT_DIR/scripts/check_ros_container_network.py"
UR_TYPE="$UR_TYPE" "$ROOT_DIR/scripts/apply_ur_update_rate.sh"
"$ROOT_DIR/scripts/apply_ur_driver_runtime_patches.sh"

echo
echo "Real-robot ROS desktop ready: http://localhost:$ROS_DESKTOP_PORT/"
echo "Login/VNC password: ubuntu"
echo
echo "This starts the real-robot ROS desktop container only."
echo "Use scripts/launch_real_robot_stable.sh to connect to the UR3e driver."
