#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

JOINT="${JOINT:-wrist_3_joint}"
OFFSET_RAD="${OFFSET_RAD:-0.03}"
DURATION="${DURATION:-4.0}"
SETTLE_BEFORE="${SETTLE_BEFORE:-5.0}"
HOLD_AFTER="${HOLD_AFTER:-30.0}"
DRY_RUN="${DRY_RUN:-false}"
VERIFY_MOTION_MIN_RAD="${VERIFY_MOTION_MIN_RAD:-0.0}"

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to move the real robot until you explicitly confirm.

After the workspace is clear, speed slider is low, and emergency stop is within reach, run for example:

  CONFIRM_REAL_ROBOT_MOVE=YES JOINT=$JOINT OFFSET_RAD=$OFFSET_RAD DURATION=$DURATION HOLD_AFTER=$HOLD_AFTER ./scripts/ur3e_nudge_joint.sh

EOF
  exit 64
fi

cd "$ROOT_DIR"

echo "Sending a small joint nudge to the active UR3e trajectory controller..."
echo "Joint: $JOINT"
echo "Offset: $OFFSET_RAD rad"
echo "Duration: $DURATION s"
echo "Settle before motion: $SETTLE_BEFORE s"
echo "Hold after: $HOLD_AFTER s"
echo "Dry run: $DRY_RUN"
echo "Verify motion min: $VERIFY_MOTION_MIN_RAD rad"

DRY_RUN_ARG=""
if [[ "$DRY_RUN" == "1" || "$DRY_RUN" == "true" ]]; then
  DRY_RUN_ARG="--dry-run"
fi

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  CONFIRM_REAL_ROBOT_MOVE=YES python3 /home/ubuntu/ur_ros2_ws/scripts/ur3e_joint_goal.py \
    --mode nudge \
    --joint '$JOINT' \
    --offset-rad '$OFFSET_RAD' \
    --duration '$DURATION' \
    --settle-before '$SETTLE_BEFORE' \
    --hold-after '$HOLD_AFTER' \
    --verify-motion-min-rad '$VERIFY_MOTION_MIN_RAD' \
    $DRY_RUN_ARG
"
