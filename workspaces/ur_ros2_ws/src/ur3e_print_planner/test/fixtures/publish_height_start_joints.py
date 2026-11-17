"""Publish a recorded UR joint posture for an isolated plan-only MoveIt test."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


FIXTURE = Path(__file__).with_name("table_m20_print_m15_start_joints.json")


def main() -> None:
    interfaces = {path.name for path in Path("/sys/class/net").iterdir()}
    if interfaces != {"lo"}:
        raise SystemExit("This joint fixture requires a container started with --network none")
    posture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    rclpy.init()
    node = Node("isolated_height_plan_joint_fixture")
    publisher = node.create_publisher(JointState, "/joint_states", 10)

    def publish() -> None:
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = posture["joint_names"]
        message.position = posture["positions_rad"]
        publisher.publish(message)

    node.create_timer(0.05, publish)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
