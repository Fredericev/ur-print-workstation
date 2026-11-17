#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String


AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


class CartesianScriptClient(Node):
    def __init__(self):
        super().__init__("ur3e_cartesian_script_client")
        self._tcp_pose = None
        self._program_running = None
        self.create_subscription(
            PoseStamped,
            "/tcp_pose_broadcaster/pose",
            self._on_tcp_pose,
            qos_profile_sensor_data,
        )
        program_qos = QoSProfile(depth=10)
        program_qos.reliability = ReliabilityPolicy.RELIABLE
        program_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            self._on_program_running,
            program_qos,
        )
        self._script_pub = self.create_publisher(String, "/urscript_interface/script_command", 10)

    def _on_tcp_pose(self, msg):
        self._tcp_pose = msg

    def _on_program_running(self, msg):
        self._program_running = bool(msg.data)

    def wait_for_inputs(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp_pose is not None and self._program_running is not None:
                return True
        return False

    def current_xyz(self):
        if self._tcp_pose is None:
            raise RuntimeError("No /tcp_pose_broadcaster/pose message received.")
        p = self._tcp_pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]

    def wait_for_script_subscriber(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._script_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def publish_script(self, script):
        msg = String()
        msg.data = script
        self._script_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.2)

    def monitor_pose(self, seconds, every):
        deadline = time.monotonic() + seconds
        next_print = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_print:
                x, y, z = self.current_xyz()
                print(f"TCP base pose: x={x:.4f} y={y:.4f} z={z:.4f}", flush=True)
                next_print = now + every


def parse_axis_sequence(value):
    axes = [item.strip().lower() for item in value.split(",") if item.strip()]
    if not axes:
        raise argparse.ArgumentTypeError("axis sequence is empty")
    for axis in axes:
        if axis not in AXIS_TO_INDEX:
            raise argparse.ArgumentTypeError("axis sequence can only contain x,y,z")
    return axes


def build_script(axes, distance, speed, accel, pause):
    lines = [
        "def codex_cartesian_xyz_test():",
        '  textmsg("Codex Cartesian XYZ TCP test start")',
    ]
    for axis in axes:
        index = AXIS_TO_INDEX[axis]
        lines.extend([
            "  target = get_actual_tcp_pose()",
            f"  target[{index}] = target[{index}] + {distance:.6f}",
            f"  movel(target, a={accel:.6f}, v={speed:.6f}, t=0, r=0)",
            f"  sleep({pause:.3f})",
        ])
    lines.extend([
        '  textmsg("Codex Cartesian XYZ TCP test done")',
        "end",
        "codex_cartesian_xyz_test()",
    ])
    return "\n".join(lines)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move the UR3e active TCP/tool head along base-frame Cartesian axes."
    )
    parser.add_argument("--axes", type=parse_axis_sequence, default=parse_axis_sequence("x,y,z"))
    parser.add_argument("--distance-m", type=float, default=0.10)
    parser.add_argument("--speed", type=float, default=0.02)
    parser.add_argument("--accel", type=float, default=0.05)
    parser.add_argument("--pause", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--monitor-extra", type=float, default=3.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if not math.isfinite(args.distance_m) or abs(args.distance_m) <= 0.0 or abs(args.distance_m) > 0.10:
        print("Refusing: --distance-m must be finite, nonzero, and <= 0.10 m.", file=sys.stderr)
        return 64
    if not math.isfinite(args.speed) or args.speed <= 0.0 or args.speed > 0.05:
        print("Refusing: --speed must be > 0 and <= 0.05 m/s.", file=sys.stderr)
        return 65
    if not math.isfinite(args.accel) or args.accel <= 0.0 or args.accel > 0.20:
        print("Refusing: --accel must be > 0 and <= 0.20 m/s^2.", file=sys.stderr)
        return 66
    if not args.dry_run:
        if os.environ.get("CONFIRM_REAL_ROBOT_MOVE") != "YES":
            print("Refusing to move: set CONFIRM_REAL_ROBOT_MOVE=YES.", file=sys.stderr)
            return 67
        if os.environ.get("CONFIRM_CARTESIAN_MOVE") != "YES":
            print("Refusing Cartesian move: set CONFIRM_CARTESIAN_MOVE=YES.", file=sys.stderr)
            return 68

    rclpy.init()
    node = CartesianScriptClient()
    try:
        if not node.wait_for_inputs(args.state_timeout):
            raise RuntimeError("Did not receive TCP pose and robot_program_running in time.")

        start = node.current_xyz()
        planned = list(start)
        print("Active TCP/tool head, base-frame Cartesian test")
        print(f"Start TCP base pose: x={start[0]:.4f} y={start[1]:.4f} z={start[2]:.4f}")
        for axis in args.axes:
            planned[AXIS_TO_INDEX[axis]] += args.distance_m
            print(
                f"After +{axis.upper()}: x={planned[0]:.4f} y={planned[1]:.4f} z={planned[2]:.4f}"
            )

        script = build_script(args.axes, args.distance_m, args.speed, args.accel, args.pause)
        print("URScript:")
        print(script)

        if args.dry_run:
            print("Dry run complete; no script was sent.")
            return 0
        if node._program_running is not True:
            raise RuntimeError("External Control program is not running on the robot.")
        if not node.wait_for_script_subscriber(timeout=3.0):
            raise RuntimeError("No subscriber on /urscript_interface/script_command.")

        expected_motion_time = len(args.axes) * (abs(args.distance_m) / args.speed + args.pause)
        print("Sending URScript Cartesian move...")
        node.publish_script(script)
        node.monitor_pose(expected_motion_time + args.monitor_extra, every=1.0)
        print("Cartesian URScript command sent; monitor window completed.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
