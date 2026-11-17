#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
UR_TYPE="${UR_TYPE:-ur3e}"

SOURCE_FILE="/home/ubuntu/ur_ros2_ws/config/${UR_TYPE}_update_rate.yaml"
TARGET_FILE="/opt/ros/humble/share/ur_robot_driver/config/${UR_TYPE}_update_rate.yaml"

docker compose -f "$COMPOSE_FILE" exec -T -u root ros2 bash -lc "
  set -euo pipefail
  if [ ! -f '$SOURCE_FILE' ]; then
    echo 'Missing update-rate file: $SOURCE_FILE' >&2
    exit 66
  fi
  if cmp -s '$SOURCE_FILE' '$TARGET_FILE'; then
    echo 'UR driver update-rate override is already active: $TARGET_FILE'
  else
    # An older prepare script could replace the host file after Compose
    # mounted it. The workspace then sees the new inode while this read-only
    # single-file mount still sees the old one. Never copy over that mount.
    if ! target_options=\$(findmnt -n -o VFS-OPTIONS --target '$TARGET_FILE'); then
      echo 'Cannot inspect the UR update-rate target mount: $TARGET_FILE' >&2
      exit 69
    fi
    case ,\$target_options, in
      *,ro,*)
        echo 'UR update-rate configuration differs from its read-only container mount.' >&2
        echo 'The container may still hold an older file inode. Recreate the ROS container to refresh its configuration mount, then retry startup.' >&2
        exit 78
        ;;
    esac
    cp '$SOURCE_FILE' '$TARGET_FILE'
    echo 'Applied UR driver update-rate override: $TARGET_FILE'
  fi
  sed -n '1,8p' '$TARGET_FILE'
"
