#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

MOONRAKER_URL="${MOONRAKER_URL:-http://192.168.0.168}"
KLIPPER_STEPPER="${KLIPPER_STEPPER:-extruder}"

if [[ ! "$MOONRAKER_URL" =~ ^https?://[^[:space:]]+$ ]]; then
  echo "MOONRAKER_URL must be a valid http(s) URL without whitespace." >&2
  exit 64
fi
if [[ ! "$KLIPPER_STEPPER" =~ ^extruder([1-9][0-9]*)?$ ]]; then
  echo "KLIPPER_STEPPER must name a native extruder (extruder, extruder1, ...)." >&2
  exit 64
fi

cd "$ROOT_DIR"
"$ROOT_DIR/scripts/ensure_workstation_ros_build.sh"
echo "Starting the persistent ROS Klipper bridge..."
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu \
  -e MOONRAKER_URL="$MOONRAKER_URL" \
  -e KLIPPER_STEPPER="$KLIPPER_STEPPER" \
  ros2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ur_ros2_ws/install/setup.bash

    log=/home/ubuntu/ur_ros2_ws/klipper_bridge_live.log
    pid_file=/home/ubuntu/ur_ros2_ws/klipper_bridge_live.pid
    probe=/home/ubuntu/ur_ros2_ws/scripts/klipper_bridge_readiness_probe.py
    pid_guard=/home/ubuntu/ur_ros2_ws/scripts/verified_pid_signal.sh
    supervisor=/home/ubuntu/ur_ros2_ws/scripts/supervise_with_rotating_log.py

    if [ -f "$pid_file" ]; then
      old_pid=$(cat "$pid_file" 2>/dev/null || true)
      "$pid_guard" "$pid_file" "ur_klipper_bridge.*klipper_bridge" TERM
      if [ -f "$pid_file" ] && [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
        deadline=$((SECONDS + 8))
        while kill -0 "$old_pid" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
          sleep 0.25
        done
      fi
    fi
    sleep 0.25

    rm -f "$pid_file"
    nohup python3 "$supervisor" \
      --log "$log" --max-bytes 10485760 --backups 3 -- \
      ros2 run ur_klipper_bridge klipper_bridge \
      --ros-args \
      -p "moonraker_url:=$MOONRAKER_URL" \
      -p "default_stepper:=$KLIPPER_STEPPER" \
      >/dev/null 2>&1 &
    bridge_pid=$!
    echo "$bridge_pid" > "$pid_file"
    echo "Klipper bridge pid: $bridge_pid"

    deadline=$((SECONDS + 25))
    last_result=""
    while [ "$SECONDS" -lt "$deadline" ]; do
      if ! kill -0 "$bridge_pid" 2>/dev/null; then
        echo "ERROR: ROS Klipper bridge stopped before becoming ready."
        tail -n 120 "$log" 2>/dev/null || true
        exit 76
      fi
      if last_result=$(python3 "$probe" \
        --timeout 1.5 --stepper "$KLIPPER_STEPPER" --json 2>&1); then
        echo "$last_result"
        echo "ROS Klipper bridge is ready."
        exit 0
      fi
      sleep 0.5
    done

    echo "ERROR: ROS Klipper bridge did not become ready."
    if [ -n "$last_result" ]; then
      echo "$last_result"
    fi
    tail -n 120 "$log" 2>/dev/null || true
    exit 75
  '

echo "Klipper bridge log inside container: /home/ubuntu/ur_ros2_ws/klipper_bridge_live.log"
