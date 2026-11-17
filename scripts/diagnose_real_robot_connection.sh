#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
CHECK_CONTROL_PORTS="${CHECK_CONTROL_PORTS:-false}"

driver_args="$(
  docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc \
    "ps -eo pid=,args= | awk '/ros2 launch ur_robot_driver|ur_control.launch.py/ && \$0 !~ /awk|diagnose_real_robot_connection/ { print }' || true" 2>/dev/null
)"

detected_robot_ip="$(printf '%s\n' "$driver_args" | sed -n 's/.*robot_ip:=\([^ ]*\).*/\1/p' | tail -1)"
detected_reverse_ip="$(printf '%s\n' "$driver_args" | sed -n 's/.*reverse_ip:=\([^ ]*\).*/\1/p' | tail -1)"

ROBOT_IP="${ROBOT_IP:-${detected_robot_ip:-192.168.12.101}}"
ROS_PC_IP="${ROS_PC_IP:-${detected_reverse_ip:-}}"

section() {
  printf '\n== %s ==\n' "$1"
}

ok_or_fail() {
  if "$@" >/tmp/ur3e_diag_cmd.out 2>&1; then
    echo "OK"
  else
    echo "FAIL"
    sed -n '1,8p' /tmp/ur3e_diag_cmd.out
  fi
}

section "Driver arguments"
if [[ -n "$driver_args" ]]; then
  printf '%s\n' "$driver_args"
else
  echo "No running ur_robot_driver launch process found in the real-robot container."
fi
echo "ROBOT_IP=$ROBOT_IP"
echo "ROS_PC_IP=${ROS_PC_IP:-unknown}"

section "Mac IPv4 addresses"
mac_ips="$(ifconfig | awk '/inet / && $2 != "127.0.0.1" { print $2 }')"
printf '%s\n' "${mac_ips:-none}"
if [[ -n "${ROS_PC_IP:-}" ]]; then
  if printf '%s\n' "$mac_ips" | grep -qx "$ROS_PC_IP"; then
    echo "ROS_PC_IP is assigned on the Mac."
  else
    echo "WARNING: ROS_PC_IP is not assigned to any current Mac interface."
  fi
fi

section "macOS route to robot"
route -n get "$ROBOT_IP" 2>/dev/null | sed -n 's/^ *gateway: /gateway: /p; s/^ *interface: /interface: /p' || true
printf 'macOS ping %s: ' "$ROBOT_IP"
ok_or_fail ping -c 2 "$ROBOT_IP"

section "Real-robot Docker container"
docker compose -f "$COMPOSE_FILE" ps --all 2>/dev/null || true
printf 'container ping %s: ' "$ROBOT_IP"
ok_or_fail docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "ping -c 2 '$ROBOT_IP'"

section "UR controller TCP ports from container"
ports=(29999)
if [[ "$CHECK_CONTROL_PORTS" == "1" || "$CHECK_CONTROL_PORTS" == "true" ]]; then
  ports=(29999 30001 30002 30003 30004)
else
  echo "Skipping 30001-30004 by default so diagnostics do not touch UR control interfaces."
  echo "Set CHECK_CONTROL_PORTS=true to probe Primary/Secondary/RTDE ports."
fi

for port in "${ports[@]}"; do
  printf 'port %s: ' "$port"
  ok_or_fail docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "timeout 2 bash -lc '</dev/tcp/$ROBOT_IP/$port'"
done

section "Reverse-control ports on Mac"
lsof -nP -iTCP:50001 -iTCP:50002 -iTCP:50003 -iTCP:50004 -sTCP:LISTEN || true

section "ROS nodes"
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; ros2 node list 2>/dev/null || true" 2>/dev/null
