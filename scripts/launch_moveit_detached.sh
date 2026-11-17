#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

UR_TYPE="${UR_TYPE:-ur3e}"
KINEMATICS_PARAMS_FILE="${UR_KINEMATICS_PARAMS_FILE:-${KINEMATICS_PARAMS_FILE:-/home/ubuntu/ur_ros2_ws/config/${UR_TYPE}_actual_calibration.yaml}}"
LOG_FILE="/home/ubuntu/ur_ros2_ws/move_group_live.log"
PID_FILE="/home/ubuntu/ur_ros2_ws/move_group_live.pid"

cd "$ROOT_DIR"

echo "Starting MoveIt move_group for $UR_TYPE..."
docker compose -f "$COMPOSE_FILE" exec -T \
  -e KINEMATICS_PARAMS_FILE="$KINEMATICS_PARAMS_FILE" -u ubuntu ros2 bash -lc "
  set -euo pipefail
  # Humble's generated setup files read optional variables without defaults.
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  set -u
  if [ ! -f \"\$KINEMATICS_PARAMS_FILE\" ] || [ ! -r \"\$KINEMATICS_PARAMS_FILE\" ]; then
    echo \"ERROR: MoveIt calibration is missing or unreadable: \$KINEMATICS_PARAMS_FILE\"
    exit 66
  fi
  if pgrep -x move_group >/dev/null 2>&1; then
    echo 'move_group is already running; calibration changes require restarting MoveIt only.'
    exit 0
  fi
  rm -f '$PID_FILE'
  nohup python3 /home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py \
    --log '$LOG_FILE' --max-bytes 10485760 --backups 3 -- \
    ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:='$UR_TYPE' \
    kinematics_params_file:=\"\$KINEMATICS_PARAMS_FILE\" \
    launch_rviz:=false \
    launch_servo:=false \
    use_sim_time:=false \
    publish_robot_description_semantic:=true \
    >/dev/null 2>&1 &
  echo \$! > '$PID_FILE'
  echo \"move_group pid: \$(cat '$PID_FILE')\"
"

echo "Waiting for move_group action server..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  deadline=\$((SECONDS + 45))
  while [ \$SECONDS -lt \$deadline ]; do
    if [ -f '$LOG_FILE' ] && grep -q 'You can start planning now!' '$LOG_FILE' &&
       pgrep -x move_group >/dev/null 2>&1; then
      echo 'move_group is ready.'
      exit 0
    fi
    if [ -f '$PID_FILE' ]; then
      launch_pid=\$(cat '$PID_FILE')
      if ! kill -0 \"\$launch_pid\" 2>/dev/null; then
        echo 'ERROR: MoveIt launch process stopped before becoming ready.'
        tail -n 120 '$LOG_FILE' 2>/dev/null || true
        exit 76
      fi
    fi
    sleep 1
  done
  echo 'ERROR: move_group did not become ready.'
  tail -n 120 '$LOG_FILE' 2>/dev/null || true
  exit 75
"

echo "MoveIt log inside container: $LOG_FILE"
