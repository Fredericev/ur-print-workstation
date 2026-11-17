#!/usr/bin/env python3
"""Verify the live UR External Control state without using the ROS 2 CLI daemon.

The robot-program state is transient-local, so a new subscriber receives the
current value even when External Control started before this probe.  A fresh
joint-state message is also required so an old connection log can never grant
real-motion readiness by itself.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Sequence


@dataclass
class ReadinessState:
    program_running: bool | None = None
    program_seen_at: float | None = None
    program_true_since: float | None = None
    joint_seen_at: float | None = None
    joint_count: int = 0

    def update_program(self, running: bool, now: float) -> None:
        self.program_running = running
        self.program_seen_at = now
        if running:
            if self.program_true_since is None:
                self.program_true_since = now
        else:
            self.program_true_since = None

    def update_joint_state(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        now: float,
    ) -> None:
        usable = min(len(names), len(positions))
        if usable >= 6:
            self.joint_count = usable
            self.joint_seen_at = now

    def ready(self, now: float, stable_seconds: float, joint_max_age: float) -> bool:
        return bool(
            self.program_running is True
            and self.program_true_since is not None
            and now - self.program_true_since >= stable_seconds
            and self.joint_seen_at is not None
            and now - self.joint_seen_at <= joint_max_age
            and self.joint_count >= 6
        )

    def reason(self, now: float, joint_max_age: float) -> str:
        if self.program_seen_at is None:
            return "No live robot-program state was received."
        if self.program_running is not True:
            return "The External Control program is not running on the teach pendant."
        if self.joint_seen_at is None or now - self.joint_seen_at > joint_max_age:
            return "No fresh six-joint state was received from the UR driver."
        return "The live External Control state has not remained stable long enough."

    def public(self, now: float, stable_seconds: float, joint_max_age: float) -> dict[str, object]:
        is_ready = self.ready(now, stable_seconds, joint_max_age)
        return {
            "ready": is_ready,
            "program_state_received": self.program_seen_at is not None,
            "program_running": self.program_running,
            "joint_state_fresh": (
                self.joint_seen_at is not None and now - self.joint_seen_at <= joint_max_age
            ),
            "joint_count": self.joint_count,
            "reason": "External Control is live and ready." if is_ready else self.reason(now, joint_max_age),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check the live UR External Control state.")
    parser.add_argument("--timeout", type=float, default=90.0, help="Maximum seconds to wait.")
    parser.add_argument(
        "--stable-seconds",
        type=float,
        default=0.5,
        help="How long robot_program_running must remain true.",
    )
    parser.add_argument(
        "--joint-max-age",
        type=float,
        default=1.0,
        help="Maximum accepted age of the joint-state sample.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Return immediately after a live false state is confirmed.",
    )
    parser.add_argument("--json", action="store_true", help="Print one machine-readable result.")
    args = parser.parse_args()
    if args.timeout <= 0.0 or args.stable_seconds < 0.0 or args.joint_max_age <= 0.0:
        parser.error("timeout and joint-max-age must be positive; stable-seconds cannot be negative")
    return args


def emit_result(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, separators=(",", ":")), flush=True)
        return
    if payload["ready"]:
        print("External Control is live and ready.", flush=True)
        print("robot_program_running: true", flush=True)
        print(f"Fresh joint states received: {payload['joint_count']}", flush=True)
    else:
        print(f"ERROR: {payload['reason']}", file=sys.stderr, flush=True)
        if payload["program_state_received"]:
            value = "true" if payload["program_running"] else "false"
            print(f"robot_program_running: {value}", file=sys.stderr, flush=True)


def main() -> int:
    args = parse_args()
    if not args.json:
        print("Checking live External Control and joint-state streams...", flush=True)
        if not args.check_only:
            print("Waiting for the teach-pendant External Control program to be running...", flush=True)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
    except Exception as exc:
        payload = {
            "ready": False,
            "program_state_received": False,
            "program_running": None,
            "joint_state_fresh": False,
            "joint_count": 0,
            "reason": f"The ROS readiness probe could not start: {exc}",
        }
        emit_result(payload, as_json=args.json)
        return 70

    state = ReadinessState()
    rclpy.init(args=[])
    node = Node(f"external_control_readiness_probe_{os.getpid()}")
    program_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_program(message: Bool) -> None:
        state.update_program(bool(message.data), time.monotonic())

    def on_joint_state(message: JointState) -> None:
        state.update_joint_state(message.name, message.position, time.monotonic())

    subscriptions = [
        node.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            on_program,
            program_qos,
        ),
        node.create_subscription(JointState, "/joint_states", on_joint_state, qos_profile_sensor_data),
    ]
    del subscriptions  # The node retains its subscriptions.

    deadline = time.monotonic() + args.timeout
    exit_code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if state.ready(now, args.stable_seconds, args.joint_max_age):
                exit_code = 0
                break
            if (
                args.check_only
                and state.program_running is False
                and state.joint_seen_at is not None
            ):
                exit_code = 2
                break
    finally:
        now = time.monotonic()
        payload = state.public(now, args.stable_seconds, args.joint_max_age)
        node.destroy_node()
        rclpy.shutdown()

    emit_result(payload, as_json=args.json)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
