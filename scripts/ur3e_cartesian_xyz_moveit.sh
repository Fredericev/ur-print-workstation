#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

AXES="${AXES:-x,y,z}"
DISTANCE_M="${DISTANCE_M:-0.05}"
EEF_STEP_M="${EEF_STEP_M:-0.005}"
JUMP_THRESHOLD="${JUMP_THRESHOLD:-0.0}"
VELOCITY_SCALING="${VELOCITY_SCALING:-0.05}"
ACCELERATION_SCALING="${ACCELERATION_SCALING:-0.05}"
MAX_SEGMENT_DELTA_RAD="${MAX_SEGMENT_DELTA_RAD:-0.010}"
MAX_JOINT_SPEED_RAD_S="${MAX_JOINT_SPEED_RAD_S:-0.080}"
MIN_SEGMENT_DURATION_S="${MIN_SEGMENT_DURATION_S:-0.250}"
GOAL_TOLERANCE_RAD="${GOAL_TOLERANCE_RAD:-0.005}"
GOAL_VELOCITY_TOLERANCE_RAD_S="${GOAL_VELOCITY_TOLERANCE_RAD_S:-0.10}"
SMOOTH_EXECUTION="${SMOOTH_EXECUTION:-true}"
INCLUDE_VELOCITY_SETPOINTS="${INCLUDE_VELOCITY_SETPOINTS:-true}"
EXECUTION_CONTROLLER="${EXECUTION_CONTROLLER:-passthrough_trajectory_controller}"
SWITCH_CONTROLLER="${SWITCH_CONTROLLER:-true}"
EXECUTE="${EXECUTE:-false}"
RETURN_TO_START="${RETURN_TO_START:-false}"
KLIPPER_OUTBOUND_DURATION_S="${KLIPPER_OUTBOUND_DURATION_S:-0.0}"
SYNCHRONIZE_KLIPPER="${SYNCHRONIZE_KLIPPER:-false}"
KLIPPER_CHECK_ONLY="${KLIPPER_CHECK_ONLY:-false}"
KLIPPER_STEPPER="${KLIPPER_STEPPER:-extruder}"
KLIPPER_TARGET_POSITION="${KLIPPER_TARGET_POSITION:-60.0}"
KLIPPER_SPEED="${KLIPPER_SPEED:-5.0}"
KLIPPER_ACCELERATION="${KLIPPER_ACCELERATION:-20.0}"
KLIPPER_RESET_POSITION="${KLIPPER_RESET_POSITION:-true}"
KLIPPER_DISABLE_AFTER="${KLIPPER_DISABLE_AFTER:-true}"
HEIGHT_TRIAL_MODE="${HEIGHT_TRIAL_MODE:-false}"
HEIGHT_TRIAL_SURFACE_Z_M="${HEIGHT_TRIAL_SURFACE_Z_M:-0.0}"
HEIGHT_TRIAL_MIN_X_M="${HEIGHT_TRIAL_MIN_X_M:-0.0}"
HEIGHT_TRIAL_MAX_X_M="${HEIGHT_TRIAL_MAX_X_M:-0.0}"
HEIGHT_TRIAL_MIN_Y_M="${HEIGHT_TRIAL_MIN_Y_M:-0.0}"
HEIGHT_TRIAL_MAX_Y_M="${HEIGHT_TRIAL_MAX_Y_M:-0.0}"
HEIGHT_TRIAL_TCP_X_M="${HEIGHT_TRIAL_TCP_X_M:-}"
HEIGHT_TRIAL_TCP_Y_M="${HEIGHT_TRIAL_TCP_Y_M:-}"
HEIGHT_TRIAL_TCP_Z_M="${HEIGHT_TRIAL_TCP_Z_M:-}"
# Bind the guarded planner to the same tool geometry as the workstation.
REQUESTED_PRINTHEAD_GEOMETRY_SHA256="${PRINTHEAD_GEOMETRY_SHA256:-}"
CURRENT_PRINTHEAD_GEOMETRY_SHA256="$(python3 - "$ROOT_DIR/workspaces/ur_ros2_ws/config/printhead_geometry.json" <<'PYHASH'
import hashlib, json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    geometry = json.load(stream)
encoded = json.dumps(geometry, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("utf-8")
print(hashlib.sha256(encoded).hexdigest())
PYHASH
)"
if [[ -n "$REQUESTED_PRINTHEAD_GEOMETRY_SHA256" && "$REQUESTED_PRINTHEAD_GEOMETRY_SHA256" != "$CURRENT_PRINTHEAD_GEOMETRY_SHA256" ]]; then
  echo "ERROR: The printhead geometry changed after this height trial was prepared. Recalibrate and plan the trial again." >&2
  exit 65
fi
PRINTHEAD_GEOMETRY_SHA256="$CURRENT_PRINTHEAD_GEOMETRY_SHA256"

if [[ "$HEIGHT_TRIAL_MODE" == "true" || "$HEIGHT_TRIAL_MODE" == "1" ]]; then
  HEIGHT_TRIAL_MODE="true"
elif [[ "$HEIGHT_TRIAL_MODE" == "false" || "$HEIGHT_TRIAL_MODE" == "0" ]]; then
  HEIGHT_TRIAL_MODE="false"
else
  echo "ERROR: HEIGHT_TRIAL_MODE must be true or false." >&2
  exit 65
fi

if [[ "$HEIGHT_TRIAL_MODE" == "true" ]]; then
  # These values are interpolated into the ROS command below. Validate their
  # syntax before invoking a shell or contacting the robot workstation.
  trial_number_pattern='^[-+]?(([0-9]+(\.[0-9]*)?)|(\.[0-9]+))([eE][-+]?[0-9]+)?$'
  for trial_tcp_name in HEIGHT_TRIAL_TCP_X_M HEIGHT_TRIAL_TCP_Y_M HEIGHT_TRIAL_TCP_Z_M; do
    trial_tcp_value="${!trial_tcp_name}"
    if [[ ! "$trial_tcp_value" =~ $trial_number_pattern ]]; then
      echo "ERROR: $trial_tcp_name requires a finite measured tool0-to-nozzle offset in metres." >&2
      exit 65
    fi
  done
  if ! awk -v x="$HEIGHT_TRIAL_TCP_X_M" -v y="$HEIGHT_TRIAL_TCP_Y_M" -v z="$HEIGHT_TRIAL_TCP_Z_M" \
      'BEGIN { tcp_len = sqrt(x*x + y*y + z*z); exit !(tcp_len >= 0.01 && tcp_len <= 0.5) }'; then
    echo "ERROR: The measured tool0-to-nozzle TCP must be 10–500 mm from tool0." >&2
    exit 65
  fi
else
  # Non-trial Cartesian calls never use this geometry. Keep ROS parameter
  # types stable while retaining the node's invalid sentinel for trial mode.
  HEIGHT_TRIAL_TCP_X_M="10.0"
  HEIGHT_TRIAL_TCP_Y_M="10.0"
  HEIGHT_TRIAL_TCP_Z_M="10.0"
fi

as_ros_double() {
  if [[ "$1" =~ ^[-+]?[0-9]+$ ]]; then
    printf '%s.0' "$1"
  else
    printf '%s' "$1"
  fi
}

KLIPPER_OUTBOUND_DURATION_S="$(as_ros_double "$KLIPPER_OUTBOUND_DURATION_S")"
KLIPPER_TARGET_POSITION="$(as_ros_double "$KLIPPER_TARGET_POSITION")"
KLIPPER_SPEED="$(as_ros_double "$KLIPPER_SPEED")"
KLIPPER_ACCELERATION="$(as_ros_double "$KLIPPER_ACCELERATION")"
if [[ "$HEIGHT_TRIAL_MODE" == "true" ]]; then
  HEIGHT_TRIAL_TCP_X_M="$(as_ros_double "$HEIGHT_TRIAL_TCP_X_M")"
  HEIGHT_TRIAL_TCP_Y_M="$(as_ros_double "$HEIGHT_TRIAL_TCP_Y_M")"
  HEIGHT_TRIAL_TCP_Z_M="$(as_ros_double "$HEIGHT_TRIAL_TCP_Z_M")"
fi

if awk "BEGIN { exit !($DISTANCE_M > 0.10 || $DISTANCE_M < -0.10) }"; then
  if awk "BEGIN { exit !($DISTANCE_M <= 0.15 && $DISTANCE_M >= -0.15) }" &&
     [[ "${ALLOW_15CM_CARTESIAN_MOVE:-}" == "YES" ]]; then
    :
  else
    cat <<EOF >&2
Refusing Cartesian distance larger than 0.10 m per axis.

For one continuous 15 cm XYZ test, first run plan-only and then explicitly opt in:

  ALLOW_15CM_CARTESIAN_MOVE=YES EXECUTE=false AXES=x,y,z DISTANCE_M=0.15 ./scripts/ur3e_cartesian_xyz_moveit.sh

  ALLOW_15CM_CARTESIAN_MOVE=YES EXECUTE=true CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_CARTESIAN_MOVE=YES \\
    AXES=x,y,z DISTANCE_M=0.15 ./scripts/ur3e_cartesian_xyz_moveit.sh
EOF
    exit 66
  fi
fi

if [[ "$EXECUTE" == "1" || "$EXECUTE" == "true" ]]; then
  if [[ "$SMOOTH_EXECUTION" == "1" || "$SMOOTH_EXECUTION" == "true" ]] &&
     [[ "$EXECUTION_CONTROLLER" == "scaled_joint_trajectory_controller" ]]; then
    cat <<EOF >&2
Refusing to execute smooth Cartesian motion on scaled_joint_trajectory_controller.

That execution path can trigger the UR External Control speed-limit popup after
larger continuous trajectories. Use the passthrough controller instead:

  EXECUTION_CONTROLLER=passthrough_trajectory_controller EXECUTE=true ...
EOF
    exit 65
  fi
  if [[ "${CONFIRM_REAL_ROBOT_MOVE:-}" != "YES" || "${CONFIRM_CARTESIAN_MOVE:-}" != "YES" ]]; then
    cat <<EOF >&2
Refusing to execute a real Cartesian MoveIt trajectory until you explicitly confirm.

Before running:
  - Press Stop Program on any active velocity-limit popup.
  - Restart/play the External Control program so it is cleanly PLAYING.
  - Clear the robot workspace.
  - Set the teach pendant speed slider low.
  - Keep the emergency stop within reach.

Plan only:
  EXECUTE=false ./scripts/ur3e_cartesian_xyz_moveit.sh

Execute:
  EXECUTE=true CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_CARTESIAN_MOVE=YES ./scripts/ur3e_cartesian_xyz_moveit.sh
EOF
    exit 64
  fi
  if [[ "$SYNCHRONIZE_KLIPPER" == "1" || "$SYNCHRONIZE_KLIPPER" == "true" ]] &&
     [[ "${CONFIRM_KLIPPER_STEPPER:-}" != "YES" ]]; then
    echo "Refusing synchronized execution without CONFIRM_KLIPPER_STEPPER=YES." >&2
    exit 64
  fi
fi

cd "$ROOT_DIR"

echo "UR3e MoveIt Cartesian XYZ test"
echo "Axes: $AXES"
echo "Distance per axis: $DISTANCE_M m"
echo "EEF step: $EEF_STEP_M m"
echo "Velocity scaling: $VELOCITY_SCALING"
echo "Acceleration scaling: $ACCELERATION_SCALING"
echo "Max guarded segment: $MAX_SEGMENT_DELTA_RAD rad"
echo "Max guarded joint speed: $MAX_JOINT_SPEED_RAD_S rad/s"
echo "Min segment duration: $MIN_SEGMENT_DURATION_S s"
echo "Goal tolerance: $GOAL_TOLERANCE_RAD rad"
echo "Goal velocity tolerance: $GOAL_VELOCITY_TOLERANCE_RAD_S rad/s"
echo "Smooth execution: $SMOOTH_EXECUTION"
echo "Include velocity setpoints: $INCLUDE_VELOCITY_SETPOINTS"
echo "Execution controller: $EXECUTION_CONTROLLER"
echo "Switch controller: $SWITCH_CONTROLLER"
echo "Execute: $EXECUTE"
echo "Return to start: $RETURN_TO_START"
echo "Synchronize Klipper through ROS: $SYNCHRONIZE_KLIPPER"
echo "Guarded tabletop-height trial: $HEIGHT_TRIAL_MODE"
if [[ "$SYNCHRONIZE_KLIPPER" == "1" || "$SYNCHRONIZE_KLIPPER" == "true" ]]; then
  echo "ROS Klipper bridge action: /klipper/manual_stepper_move"
  echo "Klipper stepper: $KLIPPER_STEPPER"
  echo "Klipper target/speed/acceleration: $KLIPPER_TARGET_POSITION / $KLIPPER_SPEED / $KLIPPER_ACCELERATION"
  echo "Klipper bridge check only: $KLIPPER_CHECK_ONLY"
fi

if [[ "$HEIGHT_TRIAL_MODE" == "true" ]]; then
  if [[ "$AXES" != "z" || "$SYNCHRONIZE_KLIPPER" != "false" ]]; then
    echo "ERROR: A height trial must be UR-only Z motion." >&2
    exit 65
  fi
  # An older installed ROS executable would ignore the new collision guard.
  # Refuse before planning or executing if the guarded binary is not installed.
  if ! docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
    binary="$(ros2 pkg prefix ur3e_print_planner)/lib/ur3e_print_planner/cartesian_xyz_moveit"
    test -r "$binary" && grep -aFq height_trial_collision_model_v4_geometry_bound "$binary"
  '; then
    echo "ERROR: The installed ROS planner lacks the measured nozzle/table collision guard. Rebuild ur3e_print_planner before a height trial." >&2
    exit 78
  fi
fi

docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ur_ros2_ws/install/setup.bash
  fi
  CONFIRM_REAL_ROBOT_MOVE='${CONFIRM_REAL_ROBOT_MOVE:-}' \
  CONFIRM_CARTESIAN_MOVE='${CONFIRM_CARTESIAN_MOVE:-}' \
  CONFIRM_KLIPPER_STEPPER='${CONFIRM_KLIPPER_STEPPER:-}' \
  ALLOW_15CM_CARTESIAN_MOVE='${ALLOW_15CM_CARTESIAN_MOVE:-}' \
  ros2 run ur3e_print_planner cartesian_xyz_moveit \
    --ros-args \
    -p axes:='$AXES' \
    -p distance_m:='$DISTANCE_M' \
    -p eef_step_m:='$EEF_STEP_M' \
    -p jump_threshold:='$JUMP_THRESHOLD' \
    -p velocity_scaling:='$VELOCITY_SCALING' \
    -p acceleration_scaling:='$ACCELERATION_SCALING' \
    -p max_segment_delta_rad:='$MAX_SEGMENT_DELTA_RAD' \
    -p max_joint_speed_rad_s:='$MAX_JOINT_SPEED_RAD_S' \
    -p min_segment_duration_s:='$MIN_SEGMENT_DURATION_S' \
    -p goal_tolerance_rad:='$GOAL_TOLERANCE_RAD' \
    -p goal_velocity_tolerance_rad_s:='$GOAL_VELOCITY_TOLERANCE_RAD_S' \
    -p smooth_execution:='$SMOOTH_EXECUTION' \
    -p include_velocity_setpoints:='$INCLUDE_VELOCITY_SETPOINTS' \
    -p execution_controller:='$EXECUTION_CONTROLLER' \
    -p switch_controller:='$SWITCH_CONTROLLER' \
    -p execute:='$EXECUTE' \
    -p return_to_start:='$RETURN_TO_START' \
    -p klipper_outbound_duration_s:='$KLIPPER_OUTBOUND_DURATION_S' \
    -p synchronize_klipper:='$SYNCHRONIZE_KLIPPER' \
    -p klipper_check_only:='$KLIPPER_CHECK_ONLY' \
    -p klipper_stepper:='$KLIPPER_STEPPER' \
    -p klipper_target_position:='$KLIPPER_TARGET_POSITION' \
    -p klipper_speed:='$KLIPPER_SPEED' \
    -p klipper_acceleration:='$KLIPPER_ACCELERATION' \
    -p klipper_reset_position:='$KLIPPER_RESET_POSITION' \
    -p klipper_disable_after:='$KLIPPER_DISABLE_AFTER' \
    -p printhead_geometry_sha256:='$PRINTHEAD_GEOMETRY_SHA256' \
    -p height_trial_mode:='$HEIGHT_TRIAL_MODE' \
    -p height_trial_surface_z_m:='$HEIGHT_TRIAL_SURFACE_Z_M' \
    -p height_trial_min_x_m:='$HEIGHT_TRIAL_MIN_X_M' \
    -p height_trial_max_x_m:='$HEIGHT_TRIAL_MAX_X_M' \
    -p height_trial_min_y_m:='$HEIGHT_TRIAL_MIN_Y_M' \
    -p height_trial_max_y_m:='$HEIGHT_TRIAL_MAX_Y_M' \
    -p height_trial_tcp_x_m:='$HEIGHT_TRIAL_TCP_X_M' \
    -p height_trial_tcp_y_m:='$HEIGHT_TRIAL_TCP_Y_M' \
    -p height_trial_tcp_z_m:='$HEIGHT_TRIAL_TCP_Z_M'
"
