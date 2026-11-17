#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
source "$ROOT_DIR/scripts/ros_artifact_environment.sh"
verify_ros_artifact_mounts

cd "$ROOT_DIR"
docker compose -f "$COMPOSE_FILE" exec -T -u ubuntu ros2 bash -lc '
set -e
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ur_ros2_ws
test -f src/Universal_Robots_ROS2_Driver/ur_controllers/CMakeLists.txt
test -f src/Universal_Robots_ROS2_Driver/ur_robot_driver/CMakeLists.txt
test -f src/Universal_Robots_Client_Library/CMakeLists.txt
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
# Build the workspace URCL first. The system ROS installation also exports a
# package with this name, and a cached CMake find_package result can otherwise
# compile the driver against /opt headers while loading the workspace library.
colcon build --symlink-install --packages-select ur_client_library \
  --allow-overriding ur_client_library \
  --executor sequential --parallel-workers 1 --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon build --symlink-install --packages-select \
  ur_controllers \
  ur_robot_driver \
  ur_klipper_interfaces \
  ur_klipper_bridge \
  ur3e_print_planner \
  ur3e_printhead_description \
  ur_moveit_config \
  --allow-overriding ur_controllers ur_moveit_config ur_robot_driver \
  --executor sequential --parallel-workers 1 --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -Dur_client_library_DIR:PATH=/home/ubuntu/ur_ros2_ws/install/ur_client_library/lib/cmake/ur_client_library
source install/setup.bash
python3 scripts/verify_controller_overlay.py
'
