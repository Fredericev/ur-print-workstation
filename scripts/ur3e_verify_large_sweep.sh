#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

JOINT="${JOINT:-wrist_2_joint}"
AMPLITUDE_RAD="${AMPLITUDE_RAD:-0.35}"
DURATION="${DURATION:-60.0}"
SETTLE_BEFORE="${SETTLE_BEFORE:-5.0}"
START_DELAY="${START_DELAY:-1.0}"
HOLD_AFTER="${HOLD_AFTER:-5.0}"
VERIFY_MOTION_MIN_RAD="${VERIFY_MOTION_MIN_RAD:-0.25}"
RETURN_DURATION="${RETURN_DURATION:-8.0}"

python3 - "$DURATION" <<'PY'
import sys
duration = float(sys.argv[1])
if duration < 60.0:
    raise SystemExit("DURATION must be at least 60 seconds for large-sweep verification.")
PY

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_LARGE_SWEEP:-}" != "YES" || "${CONFIRM_COMPUTER_CONTROL_VERIFY:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to run the large real-robot verification until you explicitly confirm.

This will move $JOINT in a large 1-minute sweep:
  current -> +$AMPLITUDE_RAD rad -> -$AMPLITUDE_RAD rad -> +$AMPLITUDE_RAD rad
Then it will send a separate return move by -$AMPLITUDE_RAD rad.

It only passes if /joint_states observes at least $VERIFY_MOTION_MIN_RAD rad of real motion.

Before running:
  - Clear the full robot workspace.
  - Set the teach pendant speed slider low for the first run.
  - Keep the emergency stop within reach.
  - Make sure the wrist/tool path is clear.

Run:
  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES CONFIRM_COMPUTER_CONTROL_VERIFY=YES ./scripts/ur3e_verify_large_sweep.sh

EOF
  exit 64
fi

cd "$ROOT_DIR"

echo "Verifying computer control with a large 1-minute UR3e sweep..."
echo "Joint: $JOINT"
echo "Amplitude: +/-$AMPLITUDE_RAD rad"
echo "Duration: $DURATION s"
echo "Required observed delta: $VERIFY_MOTION_MIN_RAD rad"
echo

"$ROOT_DIR/scripts/check_ros_driver.sh"
echo

CONFIRM_REAL_ROBOT_MOVE=YES \
CONFIRM_LARGE_SWEEP=YES \
JOINT="$JOINT" \
AMPLITUDE_RAD="$AMPLITUDE_RAD" \
DURATION="$DURATION" \
SETTLE_BEFORE="$SETTLE_BEFORE" \
START_DELAY="$START_DELAY" \
HOLD_AFTER="$HOLD_AFTER" \
VERIFY_MOTION_MIN_RAD="$VERIFY_MOTION_MIN_RAD" \
SWEEP_END=plus \
"$ROOT_DIR/scripts/ur3e_long_sweep.sh"

echo
echo "Returning $JOINT by -$AMPLITUDE_RAD rad after verified sweep."
CONFIRM_REAL_ROBOT_MOVE=YES \
JOINT="$JOINT" \
OFFSET_RAD="-$AMPLITUDE_RAD" \
DURATION="$RETURN_DURATION" \
SETTLE_BEFORE=1.0 \
HOLD_AFTER=1.0 \
VERIFY_MOTION_MIN_RAD="$VERIFY_MOTION_MIN_RAD" \
"$ROOT_DIR/scripts/ur3e_nudge_joint.sh"

echo
"$ROOT_DIR/scripts/check_ros_driver.sh"
echo
echo "Large computer-control verification passed: commanded sweep was observed on /joint_states."
