#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.0.104}"
PORT="${UR_DIRECT_PORT:-30002}"
JOINT_INDEX="${JOINT_INDEX:-5}"
OFFSET_RAD="${OFFSET_RAD:-0.03}"
ACCEL="${ACCEL:-0.10}"
SPEED="${SPEED:-0.05}"
SCRIPT_FILE="$(mktemp "${TMPDIR:-/tmp}/codex_ur_nudge.XXXXXX.script")"
trap 'rm -f "$SCRIPT_FILE"' EXIT

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to move the real robot until you explicitly confirm.

After the workspace is clear, speed slider is low, and emergency stop is within reach, run:

  CONFIRM_REAL_ROBOT_MOVE=YES ROBOT_IP=$ROBOT_IP JOINT_INDEX=$JOINT_INDEX OFFSET_RAD=$OFFSET_RAD ./scripts/ur3e_direct_nudge_joint.sh

Joint index mapping:
  0 shoulder_pan
  1 shoulder_lift
  2 elbow
  3 wrist_1
  4 wrist_2
  5 wrist_3

EOF
  exit 64
fi

python3 - "$JOINT_INDEX" "$OFFSET_RAD" "$ACCEL" "$SPEED" >"$SCRIPT_FILE" <<'PY'
import math
import sys

joint_index = int(sys.argv[1])
offset = float(sys.argv[2])
accel = float(sys.argv[3])
speed = float(sys.argv[4])

if joint_index < 0 or joint_index > 5:
    raise SystemExit("JOINT_INDEX must be 0..5")
if not math.isfinite(offset) or abs(offset) > 0.10:
    raise SystemExit("OFFSET_RAD must be finite and <= 0.10")
if not math.isfinite(accel) or accel <= 0.0 or accel > 0.50:
    raise SystemExit("ACCEL must be > 0 and <= 0.50")
if not math.isfinite(speed) or speed <= 0.0 or speed > 0.20:
    raise SystemExit("SPEED must be > 0 and <= 0.20")

print("def codex_direct_joint_nudge():")
print('  textmsg("Codex direct URScript joint nudge start")')
print("  q_start = get_actual_joint_positions()")
print("  q_target = q_start")
print(f"  q_target[{joint_index}] = q_target[{joint_index}] + {offset:.6f}")
print(f"  movej(q_target, a={accel:.6f}, v={speed:.6f}, t=0, r=0)")
print("  sleep(1.000)")
print(f"  movej(q_start, a={accel:.6f}, v={speed:.6f}, t=0, r=0)")
print("  sleep(1.000)")
print('  textmsg("Codex direct URScript joint nudge done")')
print("end")
print("codex_direct_joint_nudge()")
PY

cd "$ROOT_DIR"

echo "Sending direct URScript joint nudge..."
echo "Robot IP: $ROBOT_IP"
echo "Port: $PORT"
echo "Joint index: $JOINT_INDEX"
echo "Offset: $OFFSET_RAD rad"
python3 "$ROOT_DIR/scripts/send_urscript_direct.py" \
  --robot-ip "$ROBOT_IP" \
  --port "$PORT" \
  --script-file "$SCRIPT_FILE"
