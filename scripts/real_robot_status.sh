#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${ROBOT_IP:-192.168.12.101}"

echo "Wired services:"
networksetup -listnetworkserviceorder | awk '
  /^\([0-9]+\)/ {
    service=$0
    sub(/^\([0-9]+\) /, "", service)
  }
  /Device: en[0-9]+/ && service !~ /Wi-Fi|iPhone|Thunderbolt/ {
    device=$0
    sub(/^.*Device: /, "", device)
    sub(/\).*$/, "", device)
    print service "|" device
  }
' | while IFS='|' read -r service device; do
  ifconfig_output="$(ifconfig "$device" 2>/dev/null || true)"
  status="$(printf '%s\n' "$ifconfig_output" | sed -n 's/.*status: //p' | head -n 1)"
  ip="$(printf '%s\n' "$ifconfig_output" | awk '/inet / && $2 != "127.0.0.1" { print $2; exit }')"
  printf "  %-36s %-8s ip=%s\n" "$service ($device)" "${status:-unknown}" "${ip:--}"
done

echo
echo "Default route:"
route -n get default 2>/dev/null | sed -n 's/^ *interface: /  interface: /p; s/^ *gateway: /  gateway: /p' || true

echo
echo "Robot ping check: $ROBOT_IP"
if ping -c 1 -W 1000 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "  reachable"
else
  echo "  not reachable"
fi

echo
echo "ROS reverse-control ports currently listening on Mac:"
lsof -nP -iTCP:50001 -iTCP:50002 -iTCP:50003 -iTCP:50004 -sTCP:LISTEN || true
