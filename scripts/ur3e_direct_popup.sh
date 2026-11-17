#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.0.104}"
PORT="${UR_DIRECT_PORT:-30002}"
MESSAGE="${MESSAGE:-Codex direct URScript test}"

cd "$ROOT_DIR"

echo "Sending direct URScript popup test..."
echo "Robot IP: $ROBOT_IP"
echo "Port: $PORT"
python3 "$ROOT_DIR/scripts/send_urscript_direct.py" \
  --robot-ip "$ROBOT_IP" \
  --port "$PORT" \
  --script "popup(\"$MESSAGE\", title=\"Codex\", warning=False, error=False, blocking=False)"

echo
echo "If direct URScript is accepted, the teach pendant should show a Codex popup."
echo "Dashboard state after sending:"
ROBOT_IP="$ROBOT_IP" "$ROOT_DIR/scripts/ur_dashboard.sh" "programState" || true
ROBOT_IP="$ROBOT_IP" "$ROOT_DIR/scripts/ur_dashboard.sh" "running" || true
