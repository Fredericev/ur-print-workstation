#!/usr/bin/env bash
set -euo pipefail

MAC_IP="${MAC_IP:-192.168.12.1}"
ROBOT_IP="${ROBOT_IP:-192.168.12.101}"
NETMASK="${NETMASK:-255.255.255.0}"
SERVICE="${SERVICE:-}"

usage() {
  cat <<EOF
Usage:
  MAC_IP=192.168.12.1 ROBOT_IP=192.168.12.101 ./scripts/setup_real_robot_network.sh

Optional:
  SERVICE="USB 10/100/1000 LAN" ./scripts/setup_real_robot_network.sh

This configures one wired macOS network service with a static IPv4 address,
then checks whether the UR3e responds to ping.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "$SERVICE" ]]; then
  SERVICE="$(
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
      if ifconfig "$device" 2>/dev/null | grep -q "status: active"; then
        printf '%s\n' "$service"
        break
      fi
    done
  )"
fi

if [[ -z "$SERVICE" ]]; then
  cat <<EOF
No active wired Ethernet service was detected.

Plug the Mac into the UR control box with an Ethernet cable, then run this again.
If macOS shows the adapter but this script cannot detect it, pass SERVICE manually:

  SERVICE="USB 10/100/1000 LAN" MAC_IP=$MAC_IP ROBOT_IP=$ROBOT_IP ./scripts/setup_real_robot_network.sh
EOF
  exit 65
fi

echo "Configuring macOS network service:"
echo "  Service: $SERVICE"
echo "  Mac IP:  $MAC_IP"
echo "  Netmask: $NETMASK"
echo

networksetup -setmanual "$SERVICE" "$MAC_IP" "$NETMASK"
networksetup -setv6off "$SERVICE" >/dev/null 2>&1 || true

echo
echo "Current service info:"
networksetup -getinfo "$SERVICE"

echo
echo "Checking robot reachability: $ROBOT_IP"
ping -c 3 "$ROBOT_IP"
