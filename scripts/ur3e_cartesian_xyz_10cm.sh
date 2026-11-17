#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

AXES="${AXES:-x,y,z}"
DISTANCE_M="${DISTANCE_M:-0.10}"
SPEED="${SPEED:-0.02}"
ACCEL="${ACCEL:-0.05}"
PAUSE="${PAUSE:-0.5}"
DRY_RUN="${DRY_RUN:-true}"
ALLOW_UNSAFE_URSCRIPT_CARTESIAN="${ALLOW_UNSAFE_URSCRIPT_CARTESIAN:-false}"

if [[ "$DRY_RUN" != "1" && "$DRY_RUN" != "true" ]]; then
  if [[ "$ALLOW_UNSAFE_URSCRIPT_CARTESIAN" != "1" && "$ALLOW_UNSAFE_URSCRIPT_CARTESIAN" != "true" ]]; then
    cat <<EOF >&2
Refusing to run the live URScript Cartesian test.

This script sends raw movel() URScript through /urscript_interface while the
ROS External Control program is active. On the real robot that can conflict
with the ROS driver command stream and trigger joint velocity limit popups.

Use dry-run only:
  DRY_RUN=true ./scripts/ur3e_cartesian_xyz_10cm.sh

For real XYZ motion, use a ROS/MoveIt trajectory path instead of this script.
EOF
    exit 65
  fi

  if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_CARTESIAN_MOVE:-}" != "YES" ]]; then
    cat <<EOF >&2
Refusing to move the real robot until you explicitly confirm.

This moves the active TCP/tool head in the robot base frame:
  axes:      $AXES
  distance:  $DISTANCE_M m per axis
  speed:     $SPEED m/s
  accel:     $ACCEL m/s^2

Before running it:
  - Clear the workspace.
  - Set the teach pendant speed slider low for the first run.
  - Keep the emergency stop within reach.
  - Make sure the External Control program is PLAYING.

Preview:
  DRY_RUN=true ./scripts/ur3e_cartesian_xyz_10cm.sh

Run:
  DRY_RUN=false CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_CARTESIAN_MOVE=YES ./scripts/ur3e_cartesian_xyz_10cm.sh

EOF
    exit 64
  fi
fi

cd "$ROOT_DIR"

echo "UR3e Cartesian TCP/tool-head move"
echo "Axes: $AXES"
echo "Distance per axis: $DISTANCE_M m"
echo "Speed: $SPEED m/s"
echo "Accel: $ACCEL m/s^2"
echo "Dry run: $DRY_RUN"

DRY_RUN_ARG=""
if [[ "$DRY_RUN" == "1" || "$DRY_RUN" == "true" ]]; then
  DRY_RUN_ARG="--dry-run"
fi

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  CONFIRM_REAL_ROBOT_MOVE='${CONFIRM_REAL_ROBOT_MOVE:-}' \
  CONFIRM_CARTESIAN_MOVE='${CONFIRM_CARTESIAN_MOVE:-}' \
  python3 /home/ubuntu/ur_ros2_ws/scripts/ur3e_cartesian_urscript.py \
    --axes '$AXES' \
    --distance-m '$DISTANCE_M' \
    --speed '$SPEED' \
    --accel '$ACCEL' \
    --pause '$PAUSE' \
    $DRY_RUN_ARG
"
