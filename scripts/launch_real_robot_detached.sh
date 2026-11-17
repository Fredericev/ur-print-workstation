#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

ROBOT_IP="${ROBOT_IP:-${1:-}}"
ROS_PC_IP="${ROS_PC_IP:-${2:-}}"
UR_TYPE="${UR_TYPE:-ur3e}"
UR_UPDATE_RATE="${UR_UPDATE_RATE:-125}"
UR_RTDE_FREQUENCY="${UR_RTDE_FREQUENCY:-$UR_UPDATE_RATE}"
LAUNCH_DASHBOARD_CLIENT="${LAUNCH_DASHBOARD_CLIENT:-false}"
HEADLESS_MODE="${HEADLESS_MODE:-false}"
CONTROLLER_SPAWNER_TIMEOUT="${CONTROLLER_SPAWNER_TIMEOUT:-45}"
KINEMATICS_PARAMS_FILE="${UR_KINEMATICS_PARAMS_FILE:-${KINEMATICS_PARAMS_FILE:-/home/ubuntu/ur_ros2_ws/config/${UR_TYPE}_actual_calibration.yaml}}"
ROS_DESKTOP_PORT="${ROS_DESKTOP_PORT:-6082}"
BUILD_IMAGE="${BUILD_IMAGE:-auto}"
WAIT_FOR_READY="${WAIT_FOR_READY:-true}"
UR_DRIVER_ONLY="${UR_DRIVER_ONLY:-false}"
UR_DRIVER_START_TIMEOUT="${UR_DRIVER_START_TIMEOUT:-60}"
UR_READY_OBSERVE_SECONDS="${UR_READY_OBSERVE_SECONDS:-45}"
UR_NON_BLOCKING_READ="${UR_NON_BLOCKING_READ:-true}"
UR_SKIP_ROBOT_STATE_HELPER="${UR_SKIP_ROBOT_STATE_HELPER:-true}"
ALLOW_WIFI_ROBOT_LINK="${ALLOW_WIFI_ROBOT_LINK:-false}"
LOG_FILE="/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log"
PID_FILE="/home/ubuntu/ur_ros2_ws/real_robot_driver_live.pid"

if [[ -z "$ROBOT_IP" ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=192.168.0.104 ./scripts/launch_real_robot_detached.sh

This starts the real UR ROS driver in the container background.
Use Local Control on the teach pendant, then press Play on the External Control program.
EOF
  exit 64
fi

# Install cancellation handling before any startup work. Preflight may inspect
# an existing healthy stack, so only clean control processes after this launch
# begins replacing them. Claim ownership before cleanup, which can itself fail
# or be canceled after stopping only part of the previous stack.
owns_control_stack=false
cleanup_failed_start() {
  startup_exit=$?
  trap - EXIT
  trap '' HUP INT TERM
  if ((startup_exit != 0)) && [[ "$owns_control_stack" == "true" ]]; then
    echo "Startup failed; stopping this project's partial ROS control stack."
    "$ROOT_DIR/scripts/stop_ros_driver_detached.sh" || \
      echo "WARNING: startup cleanup could not be confirmed. Do not press Play."
  fi
  exit "$startup_exit"
}
trap cleanup_failed_start EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Checking host -> robot network before starting Docker..."
if ! ping -c 2 "$ROBOT_IP"; then
  cat <<EOF

The Mac cannot reach the robot at $ROBOT_IP yet, so I stopped before launching the driver.

Check the robot power/network cable/Wi-Fi, then retry:
  ROBOT_IP=$ROBOT_IP ./scripts/launch_real_robot_detached.sh
EOF
  exit 67
fi
echo

ROUTE_IFACE="$(route -n get "$ROBOT_IP" 2>/dev/null | awk '/interface:/{print $2; exit}' || true)"
DETECTED_ROS_PC_IP=""
if [[ -n "${ROUTE_IFACE:-}" ]]; then
  DETECTED_ROS_PC_IP="$(ipconfig getifaddr "$ROUTE_IFACE" 2>/dev/null || true)"
fi
ROUTE_SERVICE=""
if [[ -n "${ROUTE_IFACE:-}" ]]; then
  ROUTE_SERVICE="$(
    networksetup -listnetworkserviceorder | awk -v iface="$ROUTE_IFACE" '
      /^\([0-9]+\)/ {
        service=$0
        sub(/^\([0-9]+\) /, "", service)
      }
      $0 ~ "Device: " iface "\\)" { print service; exit }
    ' || true
  )"
fi
if [[ "$ROUTE_SERVICE" == "Wi-Fi" && "$ALLOW_WIFI_ROBOT_LINK" != "1" && "$ALLOW_WIFI_ROBOT_LINK" != "true" ]]; then
  cat <<EOF
The route to the robot is using Wi-Fi ($ROUTE_IFACE, $DETECTED_ROS_PC_IP).

That link is not stable enough for UR real-time RTDE control and can cause
joint-command jumps after reconnects. Use a wired Ethernet adapter for robot
control.

Recommended direct-cable setup:
  1. Set the UR teach pendant wired IP to 192.168.12.101/255.255.255.0.
  2. Configure the Mac wired adapter:
     MAC_IP=192.168.12.1 ROBOT_IP=192.168.12.101 ./scripts/setup_real_robot_network.sh
  3. Launch:
     ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot_detached.sh

If the robot stays on $ROBOT_IP, connect the Mac to that same 192.168.0.x network
through wired Ethernet and make sure this route no longer uses Wi-Fi.

If you are only doing read-only diagnosis and knowingly accept the risk, rerun with:

  ALLOW_WIFI_ROBOT_LINK=true ROBOT_IP=$ROBOT_IP ./scripts/launch_real_robot_detached.sh
EOF
  exit 68
fi

if [[ -z "$ROS_PC_IP" ]]; then
  ROS_PC_IP="$DETECTED_ROS_PC_IP"
elif [[ -n "$DETECTED_ROS_PC_IP" && "$ROS_PC_IP" != "$DETECTED_ROS_PC_IP" ]]; then
  cat <<EOF
ROS_PC_IP does not match the Mac IP currently used to reach the robot.

  Robot IP:              $ROBOT_IP
  Route interface:       ${ROUTE_IFACE:-unknown}
  Current Mac IP:        $DETECTED_ROS_PC_IP
  Provided ROS_PC_IP:    $ROS_PC_IP

Use this value in Installation -> External Control on the teach pendant:
  Host IP: $DETECTED_ROS_PC_IP

Then launch without overriding ROS_PC_IP, or use:
  ROBOT_IP=$ROBOT_IP ROS_PC_IP=$DETECTED_ROS_PC_IP ./scripts/launch_real_robot_detached.sh
EOF
  exit 66
fi

if [[ -z "$ROS_PC_IP" ]]; then
  echo "Could not infer ROS_PC_IP. Pass it explicitly, e.g. ROS_PC_IP=192.168.0.100."
  exit 65
fi

echo "Robot IP: $ROBOT_IP"
echo "ROS PC IP for External Control: $ROS_PC_IP"
echo "Update rate: ${UR_UPDATE_RATE} Hz"
echo "RTDE frequency: ${UR_RTDE_FREQUENCY} Hz"
echo "RTDE non-blocking read: $UR_NON_BLOCKING_READ"
echo "Skip robot_state_helper: $UR_SKIP_ROBOT_STATE_HELPER"
echo "ROS desktop: http://localhost:$ROS_DESKTOP_PORT/"
echo

"$ROOT_DIR/scripts/install_runtime.sh"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
UR_TYPE="$UR_TYPE" UR_UPDATE_RATE="$UR_UPDATE_RATE" "$ROOT_DIR/scripts/prepare_ur_update_rate.sh"

cd "$ROOT_DIR"
# Docker runs inside a Linux VM on macOS; ROS_PC_IP belongs to the Mac,
# not the VM. Publish the UR ports on the VM wildcard and let Colima forward
# them. Keep ROS_PC_IP unchanged for the robot's reverse connection.
UR_REVERSE_BIND_IP="0.0.0.0"
export ROS_DESKTOP_PORT UR_TYPE UR_REVERSE_BIND_IP
# A clean GitHub download has no local ROS image. Build it on first use, while
# retaining the existing no-build path for installed workstations and explicit
# BUILD_IMAGE=false operator requests.
if [[ "$BUILD_IMAGE" == "1" || "$BUILD_IMAGE" == "true" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --build
elif [[ "$BUILD_IMAGE" == "0" || "$BUILD_IMAGE" == "false" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
elif docker image inspect ur3e-real-ros2 >/dev/null 2>&1; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
else
  echo "First setup: building the ROS desktop image before launching the driver..."
  docker compose -f "$COMPOSE_FILE" up -d --build
fi
network_preflight_args=(--robot-ip "$ROBOT_IP" --repair-stale-colima-network)
if [[ "$UR_DRIVER_ONLY" != "true" ]]; then
  network_preflight_args+=(--moonraker-url "${MOONRAKER_URL:-http://192.168.0.168}")
fi
python3 "$ROOT_DIR/scripts/check_ros_container_network.py" "${network_preflight_args[@]}"
UR_TYPE="$UR_TYPE" "$ROOT_DIR/scripts/apply_ur_update_rate.sh"
UR_NON_BLOCKING_READ="$UR_NON_BLOCKING_READ" \
UR_SKIP_ROBOT_STATE_HELPER="$UR_SKIP_ROBOT_STATE_HELPER" \
"$ROOT_DIR/scripts/apply_ur_driver_runtime_patches.sh"
"$ROOT_DIR/scripts/ensure_workstation_ros_build.sh"

owns_control_stack=true
"$ROOT_DIR/scripts/cleanup_ur_driver.sh"

echo "Starting detached UR ROS driver..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  export UR_RTDE_FREQUENCY='$UR_RTDE_FREQUENCY'
  python3 /home/ubuntu/ur_ros2_ws/scripts/verify_controller_overlay.py || exit 78
  rm -f '$PID_FILE'
  nohup python3 /home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py \
    --log '$LOG_FILE' --max-bytes 10485760 --backups 3 --new-session -- \
    ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:='$UR_TYPE' \
    robot_ip:='$ROBOT_IP' \
    reverse_ip:='$ROS_PC_IP' \
    headless_mode:='$HEADLESS_MODE' \
    launch_rviz:=false \
    launch_dashboard_client:='$LAUNCH_DASHBOARD_CLIENT' \
    controller_spawner_timeout:='$CONTROLLER_SPAWNER_TIMEOUT' \
    kinematics_params_file:='$KINEMATICS_PARAMS_FILE' \
    >/dev/null 2>&1 &
  echo \$! > '$PID_FILE'
  echo \"driver pid: \$(cat '$PID_FILE')\"
"

if [[ "$UR_DRIVER_ONLY" != "true" ]]; then
  MOONRAKER_URL="${MOONRAKER_URL:-http://192.168.0.168}" \
  KLIPPER_STEPPER="${KLIPPER_STEPPER:-extruder}" \
  "$ROOT_DIR/scripts/launch_klipper_bridge_detached.sh"
fi

echo
echo "Driver log inside the container: $LOG_FILE"

if [[ "$WAIT_FOR_READY" == "1" || "$WAIT_FOR_READY" == "true" ]]; then
  echo "Waiting for the ROS driver to become ready before pressing Play..."
  docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
    set -euo pipefail
    log='$LOG_FILE'
    deadline=\$((SECONDS + $UR_DRIVER_START_TIMEOUT))
    while [ \$SECONDS -lt \$deadline ]; do
      if [ -f \"\$log\" ] && grep -q 'System successfully started!' \"\$log\"; then
        break
      fi
      # The client's frequency reset can briefly report an input-owner conflict
      # and recover on its built-in retry. Only fail on exhausted retries,
      # process death, or the bounded startup timeout, not the first warning.
      if [ -f \"\$log\" ] && grep -Eq 'Failed to initialize RTDE client after|ur_ros2_control_node.*process has died' \"\$log\"; then
        echo 'ERROR: The UR driver stopped before becoming ready. Do not press Play.'
        tail -n 80 \"\$log\" 2>/dev/null || true
        exit 79
      fi
      sleep 1
    done
    if ! [ -f \"\$log\" ] || ! grep -q 'System successfully started!' \"\$log\"; then
      if [ -f \"\$log\" ] && grep -q 'speed_slider_mask.*controlled by another RTDE client' \"\$log\"; then
        echo 'ERROR: RTDE input ownership did not recover before the startup timeout. Check other clients / Fieldbus. Do not press Play.'
        exit 78
      fi
      echo 'ERROR: ROS driver did not finish startup in time. Do not press Play.'
      tail -n 80 \"\$log\" 2>/dev/null || true
      exit 75
    fi

    if ! ss -ltn | grep -qE ':5000[1-4] '; then
      echo 'ERROR: Reverse-interface ports 50001-50004 are not listening. Do not press Play.'
      ss -ltn | grep -E ':5000[1-4]' || true
      exit 76
    fi

    start_line=\$(wc -l < \"\$log\")
    echo 'ROS driver started; observing RTDE stability before Play...'
    sleep $UR_READY_OBSERVE_SECONDS
    recent=\$(tail -n +\$((start_line + 1)) \"\$log\")
    if printf '%s\n' \"\$recent\" | grep -Eq 'Failed to read from stream|Reconnecting to the RTDE interface|Connection to reverse interface dropped|Unable to read from hardware|Packet size .* larger than buffer|process has died'; then
      echo 'ERROR: RTDE/reverse connection became unstable before Play. Do not press Play.'
      printf '%s\n' \"\$recent\" | grep -E 'Failed to read from stream|Reconnecting to the RTDE interface|Connection to reverse interface dropped|Unable to read from hardware|Packet size .* larger than buffer|process has died' | tail -n 20
      exit 77
    fi
  "

  echo "Verifying that UR reverse ports are reachable from the robot network..."
  host_port_deadline=$((SECONDS + 12))
  while true; do
    missing_host_ports=()
    for port in 50001 50002 50003 50004; do
      listeners="$(lsof -nP -iTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)"
      if ! printf '%s\n' "$listeners" | grep -v 'IPv6' | grep -Eq "TCP (\\*|0\\.0\\.0\\.0|${ROS_PC_IP//./\\.}):${port} \\(LISTEN\\)"; then
        missing_host_ports+=("$port")
      fi
    done
    if ((${#missing_host_ports[@]} == 0)); then
      break
    fi
    if ((SECONDS >= host_port_deadline)); then
      echo "ERROR: Host port forwarding is missing for: ${missing_host_ports[*]}. Do not press Play."
      for port in 50001 50002 50003 50004; do
        lsof -nP -iTCP:"$port" -sTCP:LISTEN 2>/dev/null || true
      done
      if lsof -nP -iTCP:50004 -sTCP:LISTEN 2>/dev/null | grep -q 'limactl.*127.0.0.1:50004'; then
        echo "Colima is using port 50004 for SSH. Restart ROS from the workstation; it will move Colima SSH to port 2222 automatically."
      fi
      exit 80
    fi
    sleep 1
  done
  echo "Host forwarding is ready on ports 50001-50004."
  echo "Ready. Now press Play on the teach pendant program that contains External Control."
else
  echo "Now press Play on the teach pendant program that contains External Control."
fi
