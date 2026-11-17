#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
LOG_FILE="/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log"

cd "$ROOT_DIR"

echo "== Driver processes =="
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  ps -eo pid,stat,args | grep -E "ros2 launch ur_robot_driver|ur_ros2_control_node|robot_state_helper|urscript_interface|trajectory_until_node" | grep -v grep || true
'

echo
echo "== Driver log diagnosis =="
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  if [ -f '$LOG_FILE' ] && grep -q 'speed_slider_mask.*currently controlled by another RTDE client' '$LOG_FILE'; then
    echo 'ERROR: speed_slider_mask is controlled by another RTDE client.'
    echo 'Disable robot Fieldbus such as EtherNet/IP or PROFINET, then restart the ROS driver.'
  elif [ -f '$LOG_FILE' ] && grep -q 'Robot connected to reverse interface' '$LOG_FILE'; then
    echo 'reverse interface: connected'
  else
    echo 'reverse interface: not confirmed yet'
  fi
"

echo
echo "== ROS topics =="
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  topic_echo_once() {
    timeout 4 ros2 topic echo --no-daemon --once \
      --qos-reliability reliable \
      --qos-durability transient_local \
      "$@"
  }

  if topic_echo_once /joint_states >/tmp/ur_joint_states_check.txt 2>/dev/null; then
    echo "joint_states: OK"
  else
    echo "joint_states: FAIL"
  fi

  if topic_echo_once /io_and_status_controller/robot_program_running >/tmp/ur_program_running_check.txt 2>/dev/null; then
    printf "robot_program_running: "
    awk "/data:/ { print \$2; exit }" /tmp/ur_program_running_check.txt
  else
    echo "robot_program_running: no message"
  fi
'
