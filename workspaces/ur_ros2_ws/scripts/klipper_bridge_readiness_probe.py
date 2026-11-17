#!/usr/bin/env python3
"""Read the live ROS Klipper bridge status without the ros2cli daemon."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check the ROS Klipper bridge status.")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--stepper", default="extruder", help="Native Klipper extruder object name.")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if not re.fullmatch(r"extruder(?:[1-9][0-9]*)?", args.stepper):
        parser.error("--stepper must select a native extruder (extruder, extruder1, ...)")
    return args


def emit(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, separators=(",", ":")), flush=True)
        return
    stream = sys.stdout if payload["ready"] else sys.stderr
    print(payload["reason"], file=stream, flush=True)


def status_payload(latest: object, stepper: str) -> dict[str, object]:
    """Old manual-stepper bridge messages can never authorize native E motion."""
    extruders = list(getattr(latest, "extruders", []))
    protocol = str(getattr(latest, "extrusion_protocol", ""))
    protocol_ok = protocol == "klipper-native-extruder-v1"
    extruder_found = stepper in extruders
    ready = bool(latest.moonraker_reachable and latest.printer_ready and extruder_found and protocol_ok)
    if not latest.moonraker_reachable:
        reason = latest.detail or "Moonraker is unreachable from the ROS container."
    elif not latest.printer_ready:
        reason = f"Klipper state is {latest.printer_state or 'unknown'}: {latest.detail}".rstrip(": ")
    elif not protocol_ok:
        reason = "The ROS bridge does not support the native extruder protocol. Rebuild and restart ROS."
    elif not extruder_found:
        reason = f"[{stepper}] is missing from the live Klipper config."
    else:
        reason = "ROS Klipper bridge is ready" + (" and currently coordinating an action." if latest.action_active else ".")
    return {
        "available": True,
        "ready": ready,
        "moonraker_reachable": bool(latest.moonraker_reachable),
        "printer_ready": bool(latest.printer_ready),
        "printer_state": str(latest.printer_state),
        "stepper_found": extruder_found,  # compatibility: selected print actuator
        "extruder_found": extruder_found,
        "extrusion_protocol": protocol,
        "extruders": extruders,
        "manual_steppers": list(latest.manual_steppers),
        "extruder_can_extrude": bool(getattr(latest, "extruder_can_extrude", False)),
        "extruder_temperature": getattr(latest, "extruder_temperature", None),
        "extruder_target": getattr(latest, "extruder_target", None),
        "action_active": bool(latest.action_active),
        "active_operation_id": str(latest.active_operation_id),
        "reason": reason,
    }


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from ur_klipper_interfaces.msg import KlipperStatus
    except Exception as exc:
        payload = {
            "available": False,
            "ready": False,
            "moonraker_reachable": False,
            "printer_ready": False,
            "stepper_found": False,
            "action_active": False,
            "reason": f"The ROS Klipper bridge probe could not start: {exc}",
        }
        emit(payload, as_json=args.json)
        return 70

    latest: KlipperStatus | None = None
    rclpy.init(args=[])
    node = Node(f"klipper_bridge_readiness_probe_{os.getpid()}")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_status(message: KlipperStatus) -> None:
        nonlocal latest
        latest = message

    subscription = node.create_subscription(
        KlipperStatus,
        "/klipper/status",
        on_status,
        qos,
    )
    del subscription

    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if latest is None:
        payload = {
            "available": False,
            "ready": False,
            "moonraker_reachable": False,
            "printer_ready": False,
            "stepper_found": False,
            "action_active": False,
            "reason": "No live /klipper/status message was received.",
        }
        emit(payload, as_json=args.json)
        return 1

    payload = status_payload(latest, args.stepper)
    emit(payload, as_json=args.json)
    return 0 if payload["ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
