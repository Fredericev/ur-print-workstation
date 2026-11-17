#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.0.104}"
PORT="${UR_DIRECT_PORT:-30002}"
SCRIPT_FILE="$(mktemp "${TMPDIR:-/tmp}/codex_ur_no_motion.XXXXXX.script")"
trap 'rm -f "$SCRIPT_FILE"' EXIT

cat >"$SCRIPT_FILE" <<'URSCRIPT'
def codex_direct_no_motion_test():
  textmsg("Codex direct URScript no-motion test start")
  q = get_actual_joint_positions()
  movej(q, a=0.100000, v=0.050000, t=0, r=0)
  sleep(3.000)
  textmsg("Codex direct URScript no-motion test done")
end
codex_direct_no_motion_test()
URSCRIPT

cd "$ROOT_DIR"

echo "Sending direct URScript no-motion test..."
echo "Robot IP: $ROBOT_IP"
echo "Port: $PORT"
python3 "$ROOT_DIR/scripts/send_urscript_direct.py" \
  --robot-ip "$ROBOT_IP" \
  --port "$PORT" \
  --script-file "$SCRIPT_FILE"

echo
echo "Dashboard state after sending:"
ROBOT_IP="$ROBOT_IP" "$ROOT_DIR/scripts/ur_dashboard.sh" "programState" || true
ROBOT_IP="$ROBOT_IP" "$ROOT_DIR/scripts/ur_dashboard.sh" "running" || true
