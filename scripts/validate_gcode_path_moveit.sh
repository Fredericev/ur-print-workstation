#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
REVIEW_ID="${1:-}"
REVIEW_SHA256="${2:-}"
SOURCE_WAYPOINT_COUNT="${3:-0}"
SOURCE_SAMPLED="${4:-false}"
TOOL_ROLL_DEG="${5:-180.0}"
TOOL_PITCH_DEG="${6:-0.0}"
TOOL_YAW_DEG="${7:-0.0}"
TCP_OFFSET_X_M="${8:-}"
TCP_OFFSET_Y_M="${9:-}"
TCP_OFFSET_Z_M="${10:-}"
TOOL_COLLISION_RADIUS_M="${11:-}"
BED_MIN_X_M="${12:-}"
BED_MAX_X_M="${13:-}"
BED_MIN_Y_M="${14:-}"
BED_MAX_Y_M="${15:-}"
BED_PLANE_Z_M="${16:-}"
OPERATION_MODE="${17:-production_print}"
PRINTHEAD_GEOMETRY_SHA256="${18:-}"

if [[ ! "$REVIEW_ID" =~ ^[a-f0-9]{12}$ ]]; then
  echo "Invalid path-review id." >&2
  exit 64
fi
if [[ ! "$REVIEW_SHA256" =~ ^[a-f0-9]{64}$ ]]; then
  echo "Invalid path-review checksum." >&2
  exit 64
fi
if [[ ! "$PRINTHEAD_GEOMETRY_SHA256" =~ ^[a-f0-9]{64}$ ]]; then
  echo "Missing or invalid installed printhead geometry checksum. Regenerate the path review." >&2
  exit 64
fi
if [[ ! "$SOURCE_WAYPOINT_COUNT" =~ ^[0-9]+$ ]]; then
  echo "Invalid source waypoint count." >&2
  exit 64
fi
if [[ "$SOURCE_SAMPLED" != "true" && "$SOURCE_SAMPLED" != "false" ]]; then
  echo "Invalid source sampling state." >&2
  exit 64
fi
if [[ "$OPERATION_MODE" != "production_print" && \
      "$OPERATION_MODE" != "tool0_motion_test" ]]; then
  echo "Invalid operation mode." >&2
  exit 64
fi
for angle in \
  "$TOOL_ROLL_DEG" "$TOOL_PITCH_DEG" "$TOOL_YAW_DEG" \
  "$TCP_OFFSET_X_M" "$TCP_OFFSET_Y_M" "$TCP_OFFSET_Z_M" \
  "$TOOL_COLLISION_RADIUS_M" "$BED_MIN_X_M" "$BED_MAX_X_M" \
  "$BED_MIN_Y_M" "$BED_MAX_Y_M" "$BED_PLANE_Z_M"; do
  if [[ ! "$angle" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]; then
    echo "Invalid calibrated tool orientation." >&2
    exit 64
  fi
done

HOST_RUNTIME="$ROOT_DIR/workspaces/ur_ros2_ws/runtime/path_review"
HOST_PATH_FILE="$HOST_RUNTIME/$REVIEW_ID.csv"
HOST_OUTPUT_FILE="$HOST_RUNTIME/$REVIEW_ID.json"
CONTAINER_PATH_FILE="/home/ubuntu/ur_ros2_ws/runtime/path_review/$REVIEW_ID.csv"
CONTAINER_OUTPUT_FILE="/home/ubuntu/ur_ros2_ws/runtime/path_review/$REVIEW_ID.json"
CONTAINER_APPROACH_TRAJECTORY_FILE="/home/ubuntu/ur_ros2_ws/runtime/path_review/$REVIEW_ID.approach.csv"
CONTAINER_PRINT_TRAJECTORY_FILE="/home/ubuntu/ur_ros2_ws/runtime/path_review/$REVIEW_ID.print.csv"

if [[ ! -f "$HOST_PATH_FILE" ]]; then
  echo "Calibrated path-review input is missing." >&2
  exit 66
fi

echo "Starting full-resolution chunked MoveIt plan-only path review."
echo "No trajectory controller action and no Klipper command will be sent."

docker compose -f "$COMPOSE_FILE" exec -T \
  -e REVIEW_PATH_FILE="$CONTAINER_PATH_FILE" \
  -e REVIEW_OUTPUT_FILE="$CONTAINER_OUTPUT_FILE" \
  -e REVIEW_SHA256="$REVIEW_SHA256" \
  -e SOURCE_WAYPOINT_COUNT="$SOURCE_WAYPOINT_COUNT" \
  -e SOURCE_SAMPLED="$SOURCE_SAMPLED" \
  -e TOOL_ROLL_DEG="$TOOL_ROLL_DEG" \
  -e TOOL_PITCH_DEG="$TOOL_PITCH_DEG" \
  -e TOOL_YAW_DEG="$TOOL_YAW_DEG" \
  -e TCP_OFFSET_X_M="$TCP_OFFSET_X_M" \
  -e TCP_OFFSET_Y_M="$TCP_OFFSET_Y_M" \
  -e TCP_OFFSET_Z_M="$TCP_OFFSET_Z_M" \
  -e TOOL_COLLISION_RADIUS_M="$TOOL_COLLISION_RADIUS_M" \
  -e BED_MIN_X_M="$BED_MIN_X_M" \
  -e BED_MAX_X_M="$BED_MAX_X_M" \
  -e BED_MIN_Y_M="$BED_MIN_Y_M" \
  -e BED_MAX_Y_M="$BED_MAX_Y_M" \
  -e BED_PLANE_Z_M="$BED_PLANE_Z_M" \
  -e OPERATION_MODE="$OPERATION_MODE" \
  -e PRINTHEAD_GEOMETRY_SHA256="$PRINTHEAD_GEOMETRY_SHA256" \
  -e APPROACH_TRAJECTORY_FILE="$CONTAINER_APPROACH_TRAJECTORY_FILE" \
  -e PRINT_TRAJECTORY_FILE="$CONTAINER_PRINT_TRAJECTORY_FILE" \
  -u ubuntu ros2 bash -lc '
    source /opt/ros/humble/setup.bash
    if [ -f /home/ubuntu/ur_ros2_ws/install/setup.bash ]; then
      source /home/ubuntu/ur_ros2_ws/install/setup.bash
    fi
    ros2 run ur3e_print_planner gcode_path_review \
      --ros-args \
      -p path_file:="$REVIEW_PATH_FILE" \
      -p output_file:="$REVIEW_OUTPUT_FILE" \
      -p approach_trajectory_file:="$APPROACH_TRAJECTORY_FILE" \
      -p print_trajectory_file:="$PRINT_TRAJECTORY_FILE" \
      -p review_sha256:="$REVIEW_SHA256" \
      -p source_waypoint_count:="$SOURCE_WAYPOINT_COUNT" \
      -p source_sampled:="$SOURCE_SAMPLED" \
      -p tool_roll_deg:="$TOOL_ROLL_DEG" \
      -p tool_pitch_deg:="$TOOL_PITCH_DEG" \
      -p tool_yaw_deg:="$TOOL_YAW_DEG" \
      -p tcp_offset_x_m:="$TCP_OFFSET_X_M" \
      -p tcp_offset_y_m:="$TCP_OFFSET_Y_M" \
      -p tcp_offset_z_m:="$TCP_OFFSET_Z_M" \
      -p tool_collision_radius_m:="$TOOL_COLLISION_RADIUS_M" \
      -p bed_min_x_m:="$BED_MIN_X_M" \
      -p bed_max_x_m:="$BED_MAX_X_M" \
      -p bed_min_y_m:="$BED_MIN_Y_M" \
      -p bed_max_y_m:="$BED_MAX_Y_M" \
      -p bed_plane_z_m:="$BED_PLANE_Z_M" \
      -p operation_mode:="$OPERATION_MODE" \
      -p printhead_geometry_sha256:="$PRINTHEAD_GEOMETRY_SHA256" \
      -p eef_step_m:=0.005 \
      -p velocity_scaling:=0.05 \
      -p acceleration_scaling:=0.05
  '

if [[ ! -s "$HOST_OUTPUT_FILE" ]]; then
  echo "MoveIt did not produce a path-review result." >&2
  exit 74
fi

echo "MoveIt plan-only path review complete."
echo "Execution remains locked."
