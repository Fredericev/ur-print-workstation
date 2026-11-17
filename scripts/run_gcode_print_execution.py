#!/usr/bin/env python3
"""Run one fixed Print Execution package inside the mounted ROS container."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COMPOSE_FILE = PROJECT_ROOT / "docker-compose.real.yml"
RUNTIME = PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "runtime" / "print_execution"
IDENTIFIER = re.compile(r"^[a-f0-9]{16}$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start guarded ROS Print Execution")
    parser.add_argument("execution_id")
    parser.add_argument("mode", choices=("move-to-start", "execute"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not IDENTIFIER.fullmatch(args.execution_id):
        print("ERROR: Invalid Print Execution package id.", file=sys.stderr)
        return 64
    manifest = RUNTIME / f"{args.execution_id}.json"
    if not manifest.is_file():
        print("ERROR: Print Execution package is missing.", file=sys.stderr)
        return 66
    status = RUNTIME / f"{args.execution_id}.status.json"
    if args.mode == "execute":
        status.unlink(missing_ok=True)
    container_root = "/home/ubuntu/ur_ros2_ws/runtime/print_execution"
    command = [
        "docker",
        "compose",
        "-f",
        str(COMPOSE_FILE),
        "exec",
        "-T",
        "-u",
        "ubuntu",
        "ros2",
        "bash",
        "-lc",
        (
            "source /opt/ros/humble/setup.bash; "
            "source /home/ubuntu/ur_ros2_ws/install/setup.bash; "
            "exec python3 /home/ubuntu/ur_ros2_ws/scripts/gcode_print_executor.py "
            f"--manifest {container_root}/{args.execution_id}.json "
            f"--status {container_root}/{args.execution_id}.status.json "
            f"--mode {args.mode}"
        ),
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            check=False,
            text=True,
        )
    except FileNotFoundError as exc:
        print(f"ERROR: Docker Desktop is unavailable: {exc}", file=sys.stderr)
        return 69
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
