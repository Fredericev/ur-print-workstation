#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-192.168.0.104}"
ROBOT_PORT="${UR_DIRECT_PORT:-30002}"
FEEDBACK_PORT="${UR_FEEDBACK_PORT:-50010}"
JOINT_INDEX="${JOINT_INDEX:-4}"
AMPLITUDE_RAD="${AMPLITUDE_RAD:-0.35}"
VERIFY_MIN_RAD="${VERIFY_MIN_RAD:-0.25}"

if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_LARGE_SWEEP:-}" != "YES" || "${CONFIRM_DIRECT_URSCRIPT_VERIFY:-}" != "YES" ]]; then
  cat <<EOF >&2
Refusing to run direct large real-robot verification until you explicitly confirm.

This bypasses ROS External Control and sends a URScript program directly to the robot.
The robot will move joint index $JOINT_INDEX through a 60 second +/-$AMPLITUDE_RAD rad sweep,
then report actual joint positions back to this Mac for verification.

Before running:
  - Stop the External Control program on the teach pendant.
  - Switch the UR3e to Remote Control mode.
  - Clear the full workspace.
  - Set the speed slider low.
  - Keep the emergency stop within reach.

Run:
  CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES CONFIRM_DIRECT_URSCRIPT_VERIFY=YES ./scripts/ur3e_direct_large_sweep_verify.sh

EOF
  exit 64
fi

ROUTE_IFACE="$(route -n get "$ROBOT_IP" 2>/dev/null | awk '/interface:/{print $2; exit}' || true)"
HOST_IP="${ROS_PC_IP:-}"
if [[ -z "$HOST_IP" && -n "$ROUTE_IFACE" ]]; then
  HOST_IP="$(ipconfig getifaddr "$ROUTE_IFACE" 2>/dev/null || true)"
fi
if [[ -z "$HOST_IP" ]]; then
  echo "Could not infer host IP for robot callback. Set ROS_PC_IP explicitly." >&2
  exit 65
fi

cd "$ROOT_DIR"

python3 "$ROOT_DIR/scripts/ur3e_direct_large_sweep_verify.py" \
  --robot-ip "$ROBOT_IP" \
  --host-ip "$HOST_IP" \
  --robot-port "$ROBOT_PORT" \
  --feedback-port "$FEEDBACK_PORT" \
  --joint-index "$JOINT_INDEX" \
  --amplitude-rad "$AMPLITUDE_RAD" \
  --verify-min-rad "$VERIFY_MIN_RAD"
