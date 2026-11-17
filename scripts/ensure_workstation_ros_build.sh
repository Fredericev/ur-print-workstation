#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$ROOT_DIR/workspaces/ur_ros2_ws"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
verify_ros_artifact_mounts
MARKER="$UR_ROS_INSTALL_DIR/.workstation_ros_bridge_build"
FINGERPRINT="$(python3 "$WORKSPACE/scripts/verify_controller_overlay.py" --fingerprint)"

verify_overlay() {
  # This only loads libraries and inspects build artifacts. It never starts a
  # ROS node or touches a robot, and catches a stale CMake cache even when the
  # source fingerprint and binary existence checks both pass.
  docker compose -f "$ROOT_DIR/docker-compose.real.yml" exec -T -u ubuntu ros2 bash -lc '
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ur_ros2_ws/install/setup.bash
python3 /home/ubuntu/ur_ros2_ws/scripts/verify_controller_overlay.py
'
}

needs_build=false
if [[ ! -f "$MARKER" ]] || ! docker compose -f "$ROOT_DIR/docker-compose.real.yml" exec -T -u ubuntu ros2 \
    bash -lc 'test -f /home/ubuntu/ur_ros2_ws/install/ur_client_library/lib/liburcl.so &&
              test -f /home/ubuntu/ur_ros2_ws/install/ur_controllers/lib/libur_controllers.so &&
              test -f /home/ubuntu/ur_ros2_ws/install/ur_robot_driver/lib/libur_robot_driver_plugin.so &&
              test -d /home/ubuntu/ur_ros2_ws/install/ur3e_printhead_description/share/ur3e_printhead_description'; then
  needs_build=true
else
  # Colcon's absolute Linux symlinks only resolve inside the container. A host
  # -f test on macOS would incorrectly rebuild the same controller every launch.
  # Content, not timestamps: migrated/iCloud files can retain old mtimes.
  [[ "$(<"$MARKER")" == "$FINGERPRINT" ]] || needs_build=true
fi
if [[ "$needs_build" == "false" ]] && ! verify_overlay; then
  echo "ROS overlay failed its build/ABI check; rebuilding before launch."
  needs_build=true
fi

if [[ "$needs_build" == "true" ]]; then
  echo "Building the patched UR client library, driver/controllers, ROS coordinator and Klipper bridge..."
  "$ROOT_DIR/scripts/build_print_planner.sh"
  verify_overlay
  printf '%s\n' "$FINGERPRINT" > "$MARKER"
else
  echo "UR client library, driver, ROS coordinator and Klipper bridge build is up to date."
fi
