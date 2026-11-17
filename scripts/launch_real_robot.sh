#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
HOST_DRIVER_LOG="$ROOT_DIR/workspaces/ur_ros2_ws/real_robot_driver_live.log"

ROBOT_IP="${ROBOT_IP:-${1:-}}"
ROS_PC_IP="${ROS_PC_IP:-${2:-}}"
UR_TYPE="${UR_TYPE:-ur3e}"
UR_UPDATE_RATE="${UR_UPDATE_RATE:-125}"
UR_RTDE_FREQUENCY="${UR_RTDE_FREQUENCY:-$UR_UPDATE_RATE}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
LAUNCH_DASHBOARD_CLIENT="${LAUNCH_DASHBOARD_CLIENT:-false}"
HEADLESS_MODE="${HEADLESS_MODE:-false}"
KINEMATICS_PARAMS_FILE="${UR_KINEMATICS_PARAMS_FILE:-${KINEMATICS_PARAMS_FILE:-}}"
ROS_DESKTOP_PORT="${ROS_DESKTOP_PORT:-6082}"
BUILD_IMAGE="${BUILD_IMAGE:-auto}"
CLEAN_OLD_DRIVER="${CLEAN_OLD_DRIVER:-true}"
AUTO_RESEND_ROBOT_PROGRAM="${AUTO_RESEND_ROBOT_PROGRAM:-true}"
AUTO_RESEND_DELAY_SECONDS="${AUTO_RESEND_DELAY_SECONDS:-24}"
CONTROLLER_SPAWNER_TIMEOUT="${CONTROLLER_SPAWNER_TIMEOUT:-45}"
UR_NON_BLOCKING_READ="${UR_NON_BLOCKING_READ:-true}"
UR_SKIP_ROBOT_STATE_HELPER="${UR_SKIP_ROBOT_STATE_HELPER:-true}"
ALLOW_WIFI_ROBOT_LINK="${ALLOW_WIFI_ROBOT_LINK:-false}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=<robot_ip> ./scripts/launch_real_robot.sh

Or explicitly:
  ROBOT_IP=<robot_ip> ROS_PC_IP=<your_mac_ethernet_ip> ./scripts/launch_real_robot.sh

Example:
  ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh

Headless / Remote Control mode:
  HEADLESS_MODE=true ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh

Set AUTO_RESEND_ROBOT_PROGRAM=false to skip the one-time headless reconnect helper.
EOF
  exit 0
fi

if [[ -z "$ROBOT_IP" ]]; then
  cat <<EOF
Usage:
  ROBOT_IP=<robot_ip> ./scripts/launch_real_robot.sh

Or explicitly:
  ROBOT_IP=<robot_ip> ROS_PC_IP=<your_mac_ethernet_ip> ./scripts/launch_real_robot.sh

Example:
  ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh

ROBOT_IP is the IP shown/configured on the UR3e teach pendant.
ROS_PC_IP is the IP you enter in Installation -> External Control on the teach pendant.
EOF
  exit 64
fi

echo "Safety check before continuing:"
echo "  - Keep the emergency stop within reach."
echo "  - Lower the robot speed slider for the first test."
echo "  - Start with an empty workspace around the robot."
echo
echo "Robot IP:     $ROBOT_IP"
echo "UR type:      $UR_TYPE"
echo "Update rate:  ${UR_UPDATE_RATE} Hz"
echo "RTDE freq:    ${UR_RTDE_FREQUENCY} Hz"
echo "ROS desktop:  http://localhost:$ROS_DESKTOP_PORT/"
echo "ROS login/VNC password: ubuntu"
echo "Headless:     $HEADLESS_MODE"
echo "Dashboard:    $LAUNCH_DASHBOARD_CLIENT"
echo "RTDE read:    $([[ "$UR_NON_BLOCKING_READ" == "1" || "$UR_NON_BLOCKING_READ" == "true" ]] && echo non-blocking || echo blocking)"
echo "State helper: $([[ "$UR_SKIP_ROBOT_STATE_HELPER" == "1" || "$UR_SKIP_ROBOT_STATE_HELPER" == "true" ]] && echo disabled || echo enabled)"
echo

echo "Checking host -> robot network before starting Docker..."
if ! ping -c 2 "$ROBOT_IP"; then
  cat <<EOF

The Mac cannot reach the robot at $ROBOT_IP yet, so I stopped before launching the driver.

Plug in Ethernet and configure/check the wired robot network:
  MAC_IP=192.168.12.1 ROBOT_IP=192.168.12.101 ./scripts/setup_real_robot_network.sh

Then launch again:
  ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh

If the robot stays on the current 192.168.0.x LAN, connect the Mac to that LAN
through wired Ethernet and make sure the route to $ROBOT_IP no longer uses Wi-Fi.
EOF
  exit 66
fi

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
     ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh

If the robot stays on $ROBOT_IP, connect the Mac to that same 192.168.0.x network
through wired Ethernet and make sure this route no longer uses Wi-Fi.

If you are only doing read-only diagnosis and knowingly accept the risk, rerun with:

  ALLOW_WIFI_ROBOT_LINK=true ROBOT_IP=$ROBOT_IP ./scripts/launch_real_robot.sh
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
  ROBOT_IP=$ROBOT_IP ROS_PC_IP=$DETECTED_ROS_PC_IP ./scripts/launch_real_robot.sh
EOF
  exit 67
fi

if [[ -z "$ROS_PC_IP" ]]; then
  cat <<EOF
Could not infer the Mac IP used to reach $ROBOT_IP.

Run this with ROS_PC_IP explicitly, for example:
  ROBOT_IP=$ROBOT_IP ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh
EOF
  exit 65
fi

echo
echo "ROS PC IP for External Control: $ROS_PC_IP"

if [[ -z "$KINEMATICS_PARAMS_FILE" ]]; then
  HOST_CALIBRATION_FILE="$ROOT_DIR/workspaces/ur_ros2_ws/config/${UR_TYPE}_actual_calibration.yaml"
  CONTAINER_CALIBRATION_FILE="/home/ubuntu/ur_ros2_ws/config/${UR_TYPE}_actual_calibration.yaml"
  if [[ -f "$HOST_CALIBRATION_FILE" ]]; then
    KINEMATICS_PARAMS_FILE="$CONTAINER_CALIBRATION_FILE"
    echo "Using robot calibration: $KINEMATICS_PARAMS_FILE"
  fi
fi

"$ROOT_DIR/scripts/install_runtime.sh"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
UR_TYPE="$UR_TYPE" UR_UPDATE_RATE="$UR_UPDATE_RATE" "$ROOT_DIR/scripts/prepare_ur_update_rate.sh"

cd "$ROOT_DIR"
# ROS_PC_IP is on the Mac, not in Colima's Linux VM. The VM must publish
# wildcard ports for Colima to forward; the robot still connects to ROS_PC_IP.
UR_REVERSE_BIND_IP="0.0.0.0"
export ROS_DESKTOP_PORT UR_TYPE UR_REVERSE_BIND_IP
if [[ "$BUILD_IMAGE" == "1" || "$BUILD_IMAGE" == "true" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --build
elif [[ "$BUILD_IMAGE" == "0" || "$BUILD_IMAGE" == "false" ]]; then
  docker compose -f "$COMPOSE_FILE" up -d --no-build
elif docker image inspect ur3e-real-ros2 >/dev/null 2>&1; then
  echo "Using existing local Docker image ur3e-real-ros2."
  echo "Set BUILD_IMAGE=1 if you want to rebuild it while connected to the internet."
  docker compose -f "$COMPOSE_FILE" up -d --no-build
else
  docker compose -f "$COMPOSE_FILE" up -d --build
fi

python3 "$ROOT_DIR/scripts/check_ros_container_network.py" --robot-ip "$ROBOT_IP"
UR_TYPE="$UR_TYPE" "$ROOT_DIR/scripts/apply_ur_update_rate.sh"
UR_NON_BLOCKING_READ="$UR_NON_BLOCKING_READ" \
UR_SKIP_ROBOT_STATE_HELPER="$UR_SKIP_ROBOT_STATE_HELPER" \
"$ROOT_DIR/scripts/apply_ur_driver_runtime_patches.sh"
"$ROOT_DIR/scripts/ensure_workstation_ros_build.sh"

echo
echo "Checking ROS container -> robot network..."
docker compose -f "$COMPOSE_FILE" exec -u ubuntu ros2 bash -lc "ping -c 2 '$ROBOT_IP'"

if [[ "$CLEAN_OLD_DRIVER" == "1" || "$CLEAN_OLD_DRIVER" == "true" ]]; then
  echo
  "$ROOT_DIR/scripts/cleanup_ur_driver.sh"
fi

EXTRA_ARGS=()
if [[ -n "$KINEMATICS_PARAMS_FILE" ]]; then
  EXTRA_ARGS+=("kinematics_params_file:=$KINEMATICS_PARAMS_FILE")
fi
EXTRA_ARGS_TEXT="${EXTRA_ARGS[*]-}"

echo
if [[ "$HEADLESS_MODE" == "1" || "$HEADLESS_MODE" == "true" ]]; then
  echo "Headless mode requested."
  echo "The robot must be in Remote Control mode and network services must be enabled."
  echo "Expected driver message: Robot connected to reverse interface."
  if [[ "$AUTO_RESEND_ROBOT_PROGRAM" == "1" || "$AUTO_RESEND_ROBOT_PROGRAM" == "true" ]]; then
    echo "A one-time resend_robot_program helper will check after ${AUTO_RESEND_DELAY_SECONDS}s."
    (
      sleep "$AUTO_RESEND_DELAY_SECONDS"
      docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
        source /opt/ros/humble/setup.bash
        if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
          source /home/ubuntu/ur_ros2_ws/install/setup.bash
        fi
        program_running=\"\$(timeout 8 ros2 topic echo --no-daemon --once /io_and_status_controller/robot_program_running std_msgs/msg/Bool 2>/dev/null | awk '/data:/ { print \$2; exit }')\"
        if [ \"\$program_running\" != \"true\" ]; then
          timeout 12 ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger '{}'
        else
          echo 'robot_program_running is already true; no resend needed.'
        fi
      "
    ) >/tmp/ur_resend_robot_program.log 2>&1 &
  fi
else
  echo "Start the External Control program on the teach pendant after this launch is running."
  echo "Expected driver message after pressing Play: Robot connected to reverse interface."
fi
echo

mkdir -p "$(dirname "$HOST_DRIVER_LOG")"
: >"$HOST_DRIVER_LOG"
echo "Driver log: $HOST_DRIVER_LOG"

docker compose -f "$COMPOSE_FILE" exec -u ubuntu ros2 bash -lc "
export DISPLAY=:1
export XAUTHORITY=/home/ubuntu/.Xauthority
export UR_RTDE_FREQUENCY='$UR_RTDE_FREQUENCY'
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ur_ros2_ws/install/setup.bash
fi
python3 /home/ubuntu/ur_ros2_ws/scripts/verify_controller_overlay.py || exit 78
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=$UR_TYPE \
  robot_ip:=$ROBOT_IP \
  reverse_ip:=$ROS_PC_IP \
  headless_mode:=$HEADLESS_MODE \
  launch_rviz:=$LAUNCH_RVIZ \
  launch_dashboard_client:=$LAUNCH_DASHBOARD_CLIENT \
  controller_spawner_timeout:=$CONTROLLER_SPAWNER_TIMEOUT \
  $EXTRA_ARGS_TEXT
" 2>&1 | tee "$HOST_DRIVER_LOG"
