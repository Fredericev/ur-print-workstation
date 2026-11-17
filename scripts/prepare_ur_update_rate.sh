#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UR_TYPE="${UR_TYPE:-ur3e}"
UR_UPDATE_RATE="${UR_UPDATE_RATE:-125}"

if ! [[ "$UR_UPDATE_RATE" =~ ^[0-9]+$ ]] || ((UR_UPDATE_RATE <= 0)); then
  echo "UR_UPDATE_RATE must be a positive integer in Hz, got: $UR_UPDATE_RATE" >&2
  exit 64
fi

CONFIG_DIR="$ROOT_DIR/workspaces/ur_ros2_ws/config"
CONFIG_FILE="$CONFIG_DIR/${UR_TYPE}_update_rate.yaml"

mkdir -p "$CONFIG_DIR"
tmp_file="$(mktemp "$CONFIG_FILE.XXXXXX")"
trap 'rm -f "$tmp_file"' EXIT
{
  printf 'controller_manager:\n'
  printf '  ros__parameters:\n'
  printf '    update_rate: %s # Hz\n' "$UR_UPDATE_RATE"
} >"$tmp_file"

# Both the system driver and the workspace overlay must load the selected
# rate. The overlay's installed config is a symlink to this source file.
config_files=("$CONFIG_FILE")
overlay_file="$ROOT_DIR/workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/config/${UR_TYPE}_update_rate.yaml"
if [[ -f "$overlay_file" ]]; then
  config_files+=("$overlay_file")
fi
updated=false
for config_file in "${config_files[@]}"; do
  # Compose mounts the first file read-only. Renaming an existing file would
  # leave a running container bound to its previous inode.
  if [[ -f "$config_file" ]] && cmp -s "$tmp_file" "$config_file"; then
    continue
  fi
  cat "$tmp_file" > "$config_file"
  chmod 0644 "$config_file"
  updated=true
done
if [[ "$updated" == "true" ]]; then
  echo "UR driver update-rate override: ${UR_UPDATE_RATE} Hz ($CONFIG_FILE; workspace overlay synchronized)"
else
  echo "UR driver update-rate override is already prepared: ${UR_UPDATE_RATE} Hz ($CONFIG_FILE)"
fi
