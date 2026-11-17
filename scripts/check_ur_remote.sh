#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-${1:-192.168.0.104}}"
CHECK_CONTROL_PORTS="${CHECK_CONTROL_PORTS:-false}"

section() {
  printf '\n== %s ==\n' "$1"
}

section "Mac network"
ifconfig | awk '/^[a-z0-9].*:/{iface=$1; sub(":","",iface)} /inet / && $2 != "127.0.0.1" {print iface, $2}'
echo
route -n get "$ROBOT_IP" 2>/dev/null | sed -n 's/^ *gateway: /gateway: /p; s/^ *interface: /interface: /p' || true

section "Ping robot"
if ping -c 2 "$ROBOT_IP"; then
  echo "ping: OK"
else
  echo "ping: FAIL"
fi

section "UR service ports"
port_29999="closed"
port_30001="skipped"
port_30004="skipped"
ports=(29999)
if [[ "$CHECK_CONTROL_PORTS" == "1" || "$CHECK_CONTROL_PORTS" == "true" ]]; then
  ports=(29999 30001 30002 30003 30004)
else
  echo "Skipping 30001-30004 by default so this check does not touch UR control interfaces."
  echo "Set CHECK_CONTROL_PORTS=true to probe Primary/Secondary/RTDE ports."
fi

for port in "${ports[@]}"; do
  printf 'port %s: ' "$port"
  if nc -vz -G 2 "$ROBOT_IP" "$port" >/tmp/ur_port_check.out 2>&1; then
    case "$port" in
      29999) port_29999="open" ;;
      30001) port_30001="open" ;;
      30004) port_30004="open" ;;
    esac
    echo "open"
  else
    echo "closed/unreachable"
    sed -n '1p' /tmp/ur_port_check.out
  fi
done

section "Dashboard read-only commands"
commands=(
  "PolyscopeVersion"
  "robotmode"
  "safetystatus"
  "programState"
  "running"
  "is in remote control"
  "get operational mode"
)

for command in "${commands[@]}"; do
  printf '\n$ %s\n' "$command"
  if ! "$ROOT_DIR/scripts/ur_dashboard_command.py" "$ROBOT_IP" "$command" --timeout 2; then
    echo "Dashboard command failed."
    echo "If port 29999 is closed, enable Dashboard Server / Remote Control services on the teach pendant first."
    exit 1
  fi
done

section "Diagnosis"
if [[ "$CHECK_CONTROL_PORTS" != "1" && "$CHECK_CONTROL_PORTS" != "true" ]]; then
  echo "Dashboard is reachable if port 29999 is open. Control-port probing was skipped."
elif [[ "$port_29999" == "open" ]] && [[ "$port_30001" != "open" || "$port_30004" != "open" ]]; then
  cat <<EOF
Dashboard is reachable, but the ROS control interfaces are not.

On PolyScope 5.10+ check:
  Settings -> Security -> Services

Enable at least:
  - Dashboard Server
  - Primary Client Interface
  - Real-Time Data Exchange (RTDE)

Secondary Client Interface is also useful for diagnostics. Port 30003 is legacy/deprecated.
EOF
else
  echo "Required Dashboard/Primary/RTDE ports look reachable."
fi
