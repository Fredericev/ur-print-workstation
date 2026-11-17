#!/usr/bin/env bash
# Source this file before Compose creates the container or checks its outputs.
# Directory selection never moves existing outputs and never invokes Docker.
_ur_artifact_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! _ur_artifact_exports="$(python3 "$_ur_artifact_project_root/scripts/prepare_ros_artifact_cache.py" \
    --workspace "$_ur_artifact_project_root/workspaces/ur_ros2_ws" --shell)"; then
  unset _ur_artifact_project_root _ur_artifact_exports
  return 74
fi
eval "$_ur_artifact_exports"
unset _ur_artifact_project_root _ur_artifact_exports

verify_ros_artifact_mounts() {
  local artifact_project_root
  artifact_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  # Inspect only. If an existing container still uses Desktop outputs, refuse
  # to build there; switching mounts is the launch/deployment operation's job.
  docker inspect ur3e-ros2-real | python3 "$artifact_project_root/scripts/prepare_ros_artifact_cache.py" \
    --workspace "$artifact_project_root/workspaces/ur_ros2_ws" --check-container-mounts
}
