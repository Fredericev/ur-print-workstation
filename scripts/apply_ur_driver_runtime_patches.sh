#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"
UR_KEEP_ALIVE_COUNT="${UR_KEEP_ALIVE_COUNT:-10}"
UR_NON_BLOCKING_READ="${UR_NON_BLOCKING_READ:-true}"
UR_SKIP_ROBOT_STATE_HELPER="${UR_SKIP_ROBOT_STATE_HELPER:-false}"

docker compose -f "$COMPOSE_FILE" exec -T -u root \
  -e UR_SKIP_ROBOT_STATE_HELPER="$UR_SKIP_ROBOT_STATE_HELPER" \
  ros2 python3 - <<'PY'
import os
from pathlib import Path

paths = [
    Path("/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py"),
    Path("/home/ubuntu/ur_ros2_ws/install/ur_robot_driver/share/ur_robot_driver/launch/ur_control.launch.py"),
]

for path in paths:
    if not path.exists():
        continue

    text = path.read_text()
    start = text.index('"consistent_controllers": [')
    end = text.index("]\n            },", start)
    block = text[start:end]

    if '"friction_model_controller",' in block:
        print(f"UR launch patch already active: friction_model_controller is consistent ({path})")
    else:
        marker = '                    "ur_configuration_controller",\n'
        if marker not in block:
            raise SystemExit(f"Could not find ur_configuration_controller in consistent_controllers: {path}")
        patched_block = block.replace(
            marker,
            marker + '                    "friction_model_controller",\n',
            1,
        )
        path.write_text(text[:start] + patched_block + text[end:])
        print(f"Applied UR launch patch: friction_model_controller is consistent ({path})")

    text = path.read_text()
    skip_helper = os.environ["UR_SKIP_ROBOT_STATE_HELPER"].lower() in {"1", "true", "yes"}
    enabled = "        robot_state_helper_node,\n"
    disabled = "        # robot_state_helper_node,  # disabled by Codex runtime patch\n"
    if skip_helper:
        if enabled in text:
            text = text.replace(enabled, disabled, 1)
            path.write_text(text)
            print(f"Applied UR launch patch: robot_state_helper disabled ({path})")
        elif disabled in text:
            print(f"UR launch patch already active: robot_state_helper disabled ({path})")
        else:
            raise SystemExit(f"Could not find robot_state_helper_node in nodes_to_start: {path}")
    else:
        if disabled in text:
            text = text.replace(disabled, enabled, 1)
            path.write_text(text)
            print(f"Applied UR launch patch: robot_state_helper enabled ({path})")
        else:
            print(f"UR launch patch already active: robot_state_helper enabled ({path})")
PY

docker compose -f "$COMPOSE_FILE" exec -T -u root \
  -e UR_KEEP_ALIVE_COUNT="$UR_KEEP_ALIVE_COUNT" \
  -e UR_NON_BLOCKING_READ="$UR_NON_BLOCKING_READ" \
  ros2 python3 - <<'PY'
import os
import re
from pathlib import Path

keep_alive_count = int(os.environ["UR_KEEP_ALIVE_COUNT"])
non_blocking_read = os.environ["UR_NON_BLOCKING_READ"].lower()
if non_blocking_read not in {"true", "false"}:
    raise SystemExit("UR_NON_BLOCKING_READ must be true or false")
paths = [
    Path("/opt/ros/humble/share/ur_description/urdf/ur_macro.xacro"),
    Path("/opt/ros/humble/share/ur_description/urdf/ur.ros2_control.xacro"),
]

for path in paths:
    text = path.read_text()
    patched = re.sub(r"keep_alive_count:=\d+", f"keep_alive_count:={keep_alive_count}", text)
    patched = re.sub(r"non_blocking_read:=(true|false)", f"non_blocking_read:={non_blocking_read}", patched)
    if patched == text:
        print(f"UR xacro runtime patch already active or unavailable: {path}")
    else:
        path.write_text(patched)
        print(
            f"Applied UR xacro runtime patch: {path} -> "
            f"keep_alive_count={keep_alive_count}, non_blocking_read={non_blocking_read}"
        )
PY

docker compose -f "$COMPOSE_FILE" exec -T -u root ros2 \
  grep -n -A10 -B2 "consistent_controllers" \
  /opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py

docker compose -f "$COMPOSE_FILE" exec -T -u root ros2 \
  grep -n -A14 "nodes_to_start" \
  /opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py

docker compose -f "$COMPOSE_FILE" exec -T -u root ros2 \
  grep -R -n -E "keep_alive_count:=|non_blocking_read:=" \
  /opt/ros/humble/share/ur_description/urdf/ur_macro.xacro \
  /opt/ros/humble/share/ur_description/urdf/ur.ros2_control.xacro
