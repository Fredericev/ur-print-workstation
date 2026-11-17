#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Real UR3e mode. The longer spawner timeout avoids false startup failures
# while the controller is still coming back after Power On / Brake Release.
UR_TYPE="${UR_TYPE:-ur3e}" \
UR_UPDATE_RATE="${UR_UPDATE_RATE:-125}" \
KINEMATICS_PARAMS_FILE="${UR_KINEMATICS_PARAMS_FILE:-${KINEMATICS_PARAMS_FILE:-/home/ubuntu/ur_ros2_ws/config/ur3e_actual_calibration.yaml}}" \
CONTROLLER_SPAWNER_TIMEOUT="${CONTROLLER_SPAWNER_TIMEOUT:-45}" \
LAUNCH_DASHBOARD_CLIENT="${LAUNCH_DASHBOARD_CLIENT:-false}" \
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}" \
"$ROOT_DIR/scripts/launch_real_robot.sh" "$@"
