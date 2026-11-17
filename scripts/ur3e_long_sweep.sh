#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

JOINT="${JOINT:-wrist_2_joint}"
AMPLITUDE_RAD="${AMPLITUDE_RAD:-0.35}"
DURATION="${DURATION:-60.0}"
SETTLE_BEFORE="${SETTLE_BEFORE:-5.0}"
START_DELAY="${START_DELAY:-1.0}"
HOLD_AFTER="${HOLD_AFTER:-10.0}"
DRY_RUN="${DRY_RUN:-false}"
VERIFY_MOTION_MIN_RAD="${VERIFY_MOTION_MIN_RAD:-0.0}"
SWEEP_END="${SWEEP_END:-current}"
WAYPOINT_PERIOD="${WAYPOINT_PERIOD:-0.05}"
MAX_STEP_RAD="${MAX_STEP_RAD:-0.002}"
SEGMENT_START_DELAY="${SEGMENT_START_DELAY:-0.05}"

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_LARGE_SWEEP:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to run the large real-robot sweep until you explicitly confirm.

This test moves one joint through a segmented triangle sweep for about one minute.
Before running it:
  - Clear the robot workspace.
  - Set the teach pendant speed slider low for the first run.
  - Keep the emergency stop within reach.
  - Start with wrist joints only unless you have checked the path.

Example:

  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES ./scripts/ur3e_long_sweep.sh

Customize:

  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES \\
  JOINT=wrist_2_joint AMPLITUDE_RAD=0.35 DURATION=60 \\
  ./scripts/ur3e_long_sweep.sh

Preview without sending:

  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES DRY_RUN=true ./scripts/ur3e_long_sweep.sh

EOF
  exit 64
fi

cd "$ROOT_DIR"

echo "Sending large UR3e sweep trajectory..."
echo "Joint: $JOINT"
echo "Amplitude: +/-$AMPLITUDE_RAD rad"
echo "Duration: $DURATION s"
echo "Path: current -> +amplitude -> -amplitude -> current"
echo "Sweep end: $SWEEP_END"
echo "Settle before motion: $SETTLE_BEFORE s"
echo "Start delay: $START_DELAY s"
echo "Hold after: $HOLD_AFTER s"
echo "Waypoint period floor: $WAYPOINT_PERIOD s"
echo "Max guarded step: $MAX_STEP_RAD rad"
echo "Segment start delay: $SEGMENT_START_DELAY s"
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
  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES python3 /home/ubuntu/ur_ros2_ws/scripts/ur3e_joint_goal.py \
    --mode sweep \
    --joint '$JOINT' \
    --amplitude-rad '$AMPLITUDE_RAD' \
    --sweep-end '$SWEEP_END' \
    --duration '$DURATION' \
    --settle-before '$SETTLE_BEFORE' \
    --start-delay '$START_DELAY' \
    --segment-start-delay '$SEGMENT_START_DELAY' \
    --hold-after '$HOLD_AFTER' \
    --waypoint-period '$WAYPOINT_PERIOD' \
    --max-step-rad '$MAX_STEP_RAD' \
    --verify-motion-min-rad '$VERIFY_MOTION_MIN_RAD' \
    $DRY_RUN_ARG
"
