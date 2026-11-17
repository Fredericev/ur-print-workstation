#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

JOINT="${JOINT:-wrist_3_joint}"
OFFSET_RAD="${OFFSET_RAD:-0.03}"
DURATION="${DURATION:-3.0}"
SETTLE_BEFORE="${SETTLE_BEFORE:-2.0}"
HOLD_AFTER="${HOLD_AFTER:-1.0}"
VERIFY_MOTION_MIN_RAD="${VERIFY_MOTION_MIN_RAD:-0.012}"

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_COMPUTER_CONTROL_VERIFY:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to verify real computer control until you explicitly confirm.

This will move $JOINT by $OFFSET_RAD rad, verify the motion from /joint_states,
then move it back by -$OFFSET_RAD rad and verify again.

Before running:
  - Clear the workspace.
  - Set the teach pendant speed slider low.
  - Keep the emergency stop within reach.

Run:
  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_COMPUTER_CONTROL_VERIFY=YES ./scripts/ur3e_verify_computer_control.sh

EOF
  exit 64
fi

cd "$ROOT_DIR"

echo "Verifying computer control of the UR3e with observed joint motion..."
echo "Joint: $JOINT"
echo "Offset: $OFFSET_RAD rad"
echo "Required observed delta: $VERIFY_MOTION_MIN_RAD rad"
echo

"$ROOT_DIR/scripts/check_ros_driver.sh"
echo

echo "Step 1/2: move $JOINT by +$OFFSET_RAD rad and verify observed motion."
CONFIRM_REAL_ROBOT_MOVE=YES \
JOINT="$JOINT" \
OFFSET_RAD="$OFFSET_RAD" \
DURATION="$DURATION" \
SETTLE_BEFORE="$SETTLE_BEFORE" \
HOLD_AFTER="$HOLD_AFTER" \
VERIFY_MOTION_MIN_RAD="$VERIFY_MOTION_MIN_RAD" \
"$ROOT_DIR/scripts/ur3e_nudge_joint.sh"

echo
echo "Step 2/2: move $JOINT back by -$OFFSET_RAD rad and verify observed motion."
CONFIRM_REAL_ROBOT_MOVE=YES \
JOINT="$JOINT" \
OFFSET_RAD="-$OFFSET_RAD" \
DURATION="$DURATION" \
SETTLE_BEFORE="$SETTLE_BEFORE" \
HOLD_AFTER="$HOLD_AFTER" \
VERIFY_MOTION_MIN_RAD="$VERIFY_MOTION_MIN_RAD" \
"$ROOT_DIR/scripts/ur3e_nudge_joint.sh"

echo
"$ROOT_DIR/scripts/check_ros_driver.sh"
echo
echo "Computer control verified: both commanded motions were observed on /joint_states."
