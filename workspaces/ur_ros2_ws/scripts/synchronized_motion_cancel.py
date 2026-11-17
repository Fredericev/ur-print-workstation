#!/usr/bin/env python3
"""Request coordinated ROS cancellation and wait for cleanup to finish."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time


COORDINATOR_NODE_NAMES = {"ur3e_cartesian_xyz_moveit"}
COORDINATOR_NODE_PREFIXES = ("ur_print_executor_",)


def coordinator_is_visible(node_names: list[str]) -> bool:
    return any(
        name in COORDINATOR_NODE_NAMES
        or any(name.startswith(prefix) for prefix in COORDINATOR_NODE_PREFIXES)
        for name in node_names
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cancel the active synchronized ROS motion.")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    return args


def emit(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, separators=(",", ":")), flush=True)
    else:
        stream = sys.stdout if payload["ok"] else sys.stderr
        print(payload["message"], file=stream, flush=True)


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import Trigger
    except Exception as exc:
        emit(
            {"ok": False, "message": f"ROS cancellation client could not start: {exc}"},
            as_json=args.json,
        )
        return 70

    rclpy.init(args=[])
    node = Node(f"synchronized_motion_cancel_{os.getpid()}")
    cancel_client = node.create_client(Trigger, "/synchronized_motion/cancel")
    emergency_client = node.create_client(Trigger, "/klipper/emergency_stop")
    deadline = time.monotonic() + args.timeout

    def coordinator_visible() -> bool:
        return coordinator_is_visible(node.get_node_names())

    try:
        discovery_deadline = min(deadline, time.monotonic() + 3.0)
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if coordinator_visible() or cancel_client.service_is_ready():
                break

        if not coordinator_visible() and not cancel_client.service_is_ready():
            payload = {
                "ok": True,
                "message": (
                    "No synchronized ROS coordinator was active; the launch was "
                    "canceled before hardware dispatch."
                ),
            }
            emit(payload, as_json=args.json)
            return 0

        if not cancel_client.wait_for_service(timeout_sec=3.0):
            payload = {
                "ok": False,
                "message": (
                    "The synchronized coordinator is visible but its cancellation "
                    "service is unavailable. Use the hardware emergency stop."
                ),
            }
            emit(payload, as_json=args.json)
            return 2

        future = cancel_client.call_async(Trigger.Request())
        remaining = max(0.1, min(5.0, deadline - time.monotonic()))
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        if not future.done() or future.result() is None or not future.result().success:
            message = "ROS coordinator did not accept cancellation."
            if future.done() and future.result() is not None:
                message = future.result().message or message
            emit({"ok": False, "message": message}, as_json=args.json)
            return 3

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                not coordinator_visible()
                and not cancel_client.service_is_ready()
            ):
                emit(
                    {
                        "ok": True,
                        "message": (
                            "ROS canceled the UR goal and stopped the Klipper side. "
                            "If Klipper had already moved, restart its firmware before reuse."
                        ),
                    },
                    as_json=args.json,
                )
                return 0

        emergency_message = ""
        if emergency_client.wait_for_service(timeout_sec=1.0):
            emergency_future = emergency_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, emergency_future, timeout_sec=3.0)
            if emergency_future.done() and emergency_future.result() is not None:
                emergency_message = emergency_future.result().message
        message = (
            "Timed out waiting for the ROS coordinator to confirm cancellation. "
            "Use the hardware emergency stop."
        )
        if emergency_message:
            message += f" Klipper response: {emergency_message}"
        emit({"ok": False, "message": message}, as_json=args.json)
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
