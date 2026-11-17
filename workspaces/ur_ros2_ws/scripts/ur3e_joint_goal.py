#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

DRIVER_ERROR_MARKERS = (
    "Unable to read from hardware",
    "Failed to read from stream",
    "Reconnecting to the RTDE interface",
    "State tolerances failed",
    "exceeding the joint velocity limits",
)


def driver_log_offset(path):
    if not path or not os.path.exists(path):
        return None
    return os.path.getsize(path)


def check_driver_log(path, offset):
    if offset is None or not path or not os.path.exists(path):
        return offset
    size = os.path.getsize(path)
    if size < offset:
        offset = 0
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        handle.seek(offset)
        text = handle.read()
        next_offset = handle.tell()
    matches = [
        line
        for line in text.splitlines()
        if any(marker in line for marker in DRIVER_ERROR_MARKERS)
    ]
    if matches:
        raise RuntimeError(
            "Driver reported RTDE/control instability during motion: "
            + matches[-1]
        )
    return next_offset


class JointGoalClient(Node):
    def __init__(self, allow_latched_program_state):
        super().__init__("ur3e_joint_goal_client")
        self._joint_state = None
        self._joint_state_sample_time = None
        self._program_running = None
        self._program_running_sample_time = None
        self._allow_latched_program_state = allow_latched_program_state
        self._observed_bounds = None
        joint_qos = QoSProfile(depth=10)
        joint_qos.reliability = ReliabilityPolicy.RELIABLE
        joint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            joint_qos,
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
        self._action = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

    def _on_joint_state(self, msg):
        self._joint_state = msg
        self._joint_state_sample_time = time.monotonic()
        if self._observed_bounds is not None:
            positions = dict(zip(msg.name, msg.position))
            for name in JOINTS:
                if name not in positions:
                    continue
                value = float(positions[name])
                bounds = self._observed_bounds[name]
                bounds[0] = min(bounds[0], value)
                bounds[1] = max(bounds[1], value)

    def _on_program_running(self, msg):
        self._program_running = bool(msg.data)
        self._program_running_sample_time = time.monotonic()

    def wait_for_inputs(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state is not None and self._program_running is not None:
                return True
        return False

    def wait_for_joint_state(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state is not None:
                return True
        return False

    def wait_for_fresh_joint_state(self, timeout, max_age):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._joint_state_sample_time is None:
                continue
            if time.monotonic() - self._joint_state_sample_time <= max_age:
                return True
        return False

    def wait_for_program_running(self, timeout, stable_duration):
        deadline = time.monotonic() + timeout
        true_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            fresh = (
                self._program_running_sample_time is not None
                and now - self._program_running_sample_time < 1.0
            )
            if self._program_running is True and fresh:
                if true_since is None:
                    true_since = now
                if now - true_since >= stable_duration:
                    return True
            elif self._program_running is True:
                return True
            elif (
                self._allow_latched_program_state
                and self._program_running is True
            ):
                return True
            else:
                true_since = None
        return False

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def current_positions(self, max_age=None):
        if self._joint_state is None:
            raise RuntimeError("No /joint_states message received.")
        if max_age is not None:
            if self._joint_state_sample_time is None:
                raise RuntimeError("No fresh /joint_states sample time is available.")
            age = time.monotonic() - self._joint_state_sample_time
            if age > max_age:
                raise RuntimeError(f"/joint_states is stale: age {age:.3f}s > {max_age:.3f}s.")
        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        missing = [name for name in JOINTS if name not in by_name]
        if missing:
            raise RuntimeError(f"/joint_states is missing joints: {', '.join(missing)}")
        return [float(by_name[name]) for name in JOINTS]

    def require_ready_for_motion(self, timeout, stable_duration, joint_state_max_age):
        if not self.wait_for_fresh_joint_state(timeout, joint_state_max_age):
            raise RuntimeError("Did not receive a fresh /joint_states sample in time.")
        if not self.wait_for_program_running(timeout, stable_duration):
            raise RuntimeError("External Control program is not freshly running on the robot.")

    def start_motion_observation(self, start_positions):
        self._observed_bounds = {
            name: [float(value), float(value)]
            for name, value in zip(JOINTS, start_positions)
        }

    def observed_joint_delta(self, joint_name, start_position):
        if self._observed_bounds is None:
            return 0.0
        low, high = self._observed_bounds[joint_name]
        return max(abs(low - start_position), abs(high - start_position))

    def send_goal(self, target, duration, hold_after, start_delay, settle_before):
        current = self.current_positions(max_age=0.5)
        trajectory = [(0.0, current)]
        if settle_before > 0.0:
            trajectory.append((settle_before, current))
        trajectory.append((settle_before + duration, target))
        if hold_after > 0.0:
            trajectory.append((settle_before + duration + hold_after, target))
        self.send_trajectory(trajectory, settle_before + duration + hold_after, start_delay)

    def send_trajectory(self, trajectory, total_duration, start_delay):
        if not self._action.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(
                "Action server /scaled_joint_trajectory_controller/follow_joint_trajectory is not available."
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        goal.trajectory.header.stamp = (
            self.get_clock().now() + RclpyDuration(seconds=start_delay)
        ).to_msg()

        for time_from_start, positions in trajectory:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.time_from_start = Duration(
                sec=int(time_from_start),
                nanosec=int((time_from_start - int(time_from_start)) * 1_000_000_000),
            )
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        future = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Trajectory goal was rejected.")

        result_future = handle.get_result_async()
        wait_time = max(12.0, start_delay + total_duration * 5.0 + 20.0)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=wait_time)
        result = result_future.result()
        if result is None:
            raise RuntimeError("Timed out waiting for trajectory result.")
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory failed: code={result.result.error_code} {result.result.error_string}"
            )

    def send_step_from_current(self, target, duration, start_delay):
        current = self.current_positions(max_age=0.5)
        self.send_trajectory([(0.0, current), (duration, target)], duration, start_delay)


def parse_args():
    parser = argparse.ArgumentParser(description="Send a safe UR3e joint trajectory goal.")
    parser.add_argument("--mode", choices=["hold", "nudge", "sweep"], required=True)
    parser.add_argument("--joint", choices=JOINTS, default="wrist_2_joint")
    parser.add_argument("--offset-rad", type=float, default=0.03)
    parser.add_argument("--amplitude-rad", type=float, default=0.35)
    parser.add_argument("--cycles", type=float, default=2.0)
    parser.add_argument("--sweep-end", choices=["current", "plus", "minus"], default="current")
    parser.add_argument("--waypoint-period", type=float, default=0.05)
    parser.add_argument("--max-step-rad", type=float, default=0.002)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--hold-after", type=float, default=0.0)
    parser.add_argument("--settle-before", type=float, default=5.0)
    parser.add_argument("--start-delay", type=float, default=1.0)
    parser.add_argument("--segment-start-delay", type=float, default=0.05)
    parser.add_argument("--joint-state-max-age", type=float, default=0.5)
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--allow-latched-program-state", action="store_true")
    parser.add_argument(
        "--driver-log",
        default="/home/ubuntu/ur_ros2_ws/real_robot_driver_live.log",
    )
    parser.add_argument("--disable-driver-log-guard", action="store_true")
    parser.add_argument("--verify-motion-min-rad", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.mode in {"nudge", "sweep"}:
        if os.environ.get("CONFIRM_REAL_ROBOT_MOVE") != "YES":
            print(
                "Refusing to move: set CONFIRM_REAL_ROBOT_MOVE=YES after clearing the workspace.",
                file=sys.stderr,
            )
            return 64
        if args.allow_latched_program_state:
            print(
                "Refusing to move: latched robot_program_running state is not accepted for moving commands.",
                file=sys.stderr,
            )
            return 76
    if args.mode == "nudge":
        if not math.isfinite(args.offset_rad) or abs(args.offset_rad) > 0.10:
            print("Refusing to move: --offset-rad must be finite and <= 0.10 rad.", file=sys.stderr)
            return 65
    if args.mode == "sweep":
        if os.environ.get("CONFIRM_LARGE_SWEEP") != "YES":
            print("Refusing large sweep: set CONFIRM_LARGE_SWEEP=YES.", file=sys.stderr)
            return 67
        if not math.isfinite(args.amplitude_rad) or args.amplitude_rad <= 0.0 or args.amplitude_rad > 0.50:
            print("Refusing to sweep: --amplitude-rad must be > 0 and <= 0.50 rad.", file=sys.stderr)
            return 68
        if not math.isfinite(args.duration) or args.duration < 20.0 or args.duration > 120.0:
            print("Refusing to sweep: --duration must be between 20 and 120 seconds.", file=sys.stderr)
            return 69
        if not math.isfinite(args.cycles) or args.cycles <= 0.0 or args.cycles > 4.0:
            print("Refusing to sweep: --cycles must be > 0 and <= 4.", file=sys.stderr)
            return 70
        if not math.isfinite(args.waypoint_period) or args.waypoint_period < 0.02 or args.waypoint_period > 5.0:
            print("Refusing to sweep: --waypoint-period must be between 0.02 and 5 seconds.", file=sys.stderr)
            return 71
        if not math.isfinite(args.max_step_rad) or args.max_step_rad <= 0.0 or args.max_step_rad > 0.01:
            print("Refusing to sweep: --max-step-rad must be > 0 and <= 0.01 rad.", file=sys.stderr)
            return 72
    if not math.isfinite(args.hold_after) or args.hold_after < 0.0 or args.hold_after > 600.0:
        print("Refusing to run: --hold-after must be between 0 and 600 seconds.", file=sys.stderr)
        return 66
    if not math.isfinite(args.settle_before) or args.settle_before < 0.0 or args.settle_before > 15.0:
        print("Refusing to run: --settle-before must be between 0 and 15 seconds.", file=sys.stderr)
        return 74
    if not math.isfinite(args.start_delay) or args.start_delay < 0.25 or args.start_delay > 5.0:
        print("Refusing to run: --start-delay must be between 0.25 and 5 seconds.", file=sys.stderr)
        return 73
    if (
        not math.isfinite(args.segment_start_delay)
        or args.segment_start_delay < 0.02
        or args.segment_start_delay > 1.0
    ):
        print(
            "Refusing to run: --segment-start-delay must be between 0.02 and 1.0 seconds.",
            file=sys.stderr,
        )
        return 77
    if (
        not math.isfinite(args.joint_state_max_age)
        or args.joint_state_max_age <= 0.0
        or args.joint_state_max_age > 2.0
    ):
        print(
            "Refusing to run: --joint-state-max-age must be > 0 and <= 2 seconds.",
            file=sys.stderr,
        )
        return 78
    if not math.isfinite(args.verify_motion_min_rad) or args.verify_motion_min_rad < 0.0 or args.verify_motion_min_rad > 0.50:
        print("Refusing to run: --verify-motion-min-rad must be between 0 and 0.50 rad.", file=sys.stderr)
        return 75

    rclpy.init()
    node = JointGoalClient(args.allow_latched_program_state)
    try:
        if args.dry_run:
            if not node.wait_for_joint_state(args.state_timeout):
                raise RuntimeError("Did not receive /joint_states in time.")
        else:
            if not node.wait_for_inputs(args.state_timeout):
                raise RuntimeError("Did not receive /joint_states and robot_program_running in time.")
            if args.mode in {"nudge", "sweep"}:
                node.require_ready_for_motion(
                    args.state_timeout,
                    stable_duration=0.5,
                    joint_state_max_age=args.joint_state_max_age,
                )
            elif not node.wait_for_program_running(args.state_timeout, stable_duration=0.5):
                raise RuntimeError("External Control program is not running on the robot.")

        current = node.current_positions(
            max_age=None if args.dry_run else args.joint_state_max_age
        )
        driver_guard_offset = None
        if (
            not args.dry_run
            and args.mode in {"nudge", "sweep"}
            and not args.disable_driver_log_guard
        ):
            driver_guard_offset = driver_log_offset(args.driver_log)
            if driver_guard_offset is None:
                print(
                    f"Warning: driver log guard disabled; log not found: {args.driver_log}"
                )
            else:
                print(f"Driver log guard active: {args.driver_log}")
        if node._program_running is not True and args.dry_run:
            print("Warning: External Control program is not running; dry-run preview only.")
        target = list(current)
        if args.mode == "nudge":
            target[JOINTS.index(args.joint)] += args.offset_rad
        elif args.mode == "sweep":
            joint_index = JOINTS.index(args.joint)
            start_joint = current[joint_index]
            target_plus_joint = start_joint + args.amplitude_rad
            target_minus_joint = start_joint - args.amplitude_rad
            if args.sweep_end == "plus":
                sweep_end_joint = target_plus_joint
            elif args.sweep_end == "minus":
                sweep_end_joint = target_minus_joint
            else:
                sweep_end_joint = start_joint
            sweep_legs = [
                (target_plus_joint, args.duration * 0.25),
                (target_minus_joint, args.duration * 0.50),
                (sweep_end_joint, args.duration * 0.25),
            ]
            previous_joint = start_joint
            estimated_segments = 0
            for leg_target, _leg_duration in sweep_legs:
                estimated_segments += max(
                    1, math.ceil(abs(leg_target - previous_joint) / args.max_step_rad)
                )
                previous_joint = leg_target
            max_delta = args.max_step_rad

        print("Current joint positions:")
        for name, value in zip(JOINTS, current):
            print(f"  {name}: {value:.6f} rad")
        if args.mode == "sweep":
            print(
                f"Sweep plan: {args.joint}, amplitude +/-{args.amplitude_rad:.3f} rad, "
                f"guarded small-step triangle, requested {args.duration:.1f} s"
            )
            print(f"Sweep end: {args.sweep_end}")
            print(f"Estimated guarded action segments: {estimated_segments}")
            print(f"Largest commanded {args.joint} step: {max_delta:.6f} rad")
            print(f"Waypoint period floor: {args.waypoint_period:.3f} s")
            print(f"Settle before motion: {args.settle_before:.3f} s")
            print(f"First trajectory start delay: {args.start_delay:.3f} s")
            print(f"Per-segment start delay: {args.segment_start_delay:.3f} s")
            if args.dry_run:
                print("Dry run complete; no trajectory was sent.")
                return 0
            if args.settle_before > 0.0:
                print(f"Settling at current position for {args.settle_before:.3f} s...")
                driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
                settle_position = node.current_positions(max_age=args.joint_state_max_age)
                node.send_step_from_current(settle_position, args.settle_before, args.start_delay)
                driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
            if args.verify_motion_min_rad > 0.0:
                node.start_motion_observation(current)
            previous_joint = start_joint
            for leg_index, (leg_target, leg_duration) in enumerate(sweep_legs, start=1):
                planned_distance = abs(leg_target - previous_joint)
                leg_speed = planned_distance / max(leg_duration, args.waypoint_period)
                max_segments = max(
                    10, math.ceil(max(planned_distance, args.max_step_rad) / args.max_step_rad) * 3 + 20
                )
                print(
                    f"Leg {leg_index}: target {leg_target:.6f} rad, "
                    f"planned {leg_duration:.3f} s"
                )
                sent_segments = 0
                while True:
                    node.require_ready_for_motion(
                        timeout=2.0,
                        stable_duration=0.05,
                        joint_state_max_age=args.joint_state_max_age,
                    )
                    actual = node.current_positions(max_age=args.joint_state_max_age)
                    remaining = leg_target - actual[joint_index]
                    if abs(remaining) <= args.max_step_rad * 0.5:
                        break
                    if sent_segments >= max_segments:
                        raise RuntimeError(
                            f"Guarded sweep failed to converge on leg {leg_index}; "
                            f"remaining {remaining:.6f} rad after {sent_segments} segments."
                        )
                    step = max(-args.max_step_rad, min(args.max_step_rad, remaining))
                    target_step = list(actual)
                    target_step[joint_index] += step
                    step_duration = max(
                        args.waypoint_period,
                        abs(step) / max(leg_speed, 1.0e-6),
                    )
                    driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
                    node.send_step_from_current(
                        target_step,
                        step_duration,
                        args.segment_start_delay,
                    )
                    driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
                    sent_segments += 1
                previous_joint = leg_target
                print(f"Leg {leg_index} completed with {sent_segments} guarded segments.")
            if args.hold_after > 0.0:
                print(f"Holding current position for {args.hold_after:.3f} s...")
                node.require_ready_for_motion(
                    timeout=2.0,
                    stable_duration=0.05,
                    joint_state_max_age=args.joint_state_max_age,
                )
                hold_position = node.current_positions(max_age=args.joint_state_max_age)
                driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
                node.send_step_from_current(
                    hold_position,
                    args.hold_after,
                    args.segment_start_delay,
                )
                driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
            if args.verify_motion_min_rad > 0.0:
                observed = node.observed_joint_delta(args.joint, current[joint_index])
                if observed < args.verify_motion_min_rad:
                    raise RuntimeError(
                        f"Motion verification failed: observed {args.joint} delta "
                        f"{observed:.6f} rad < required {args.verify_motion_min_rad:.6f} rad."
                    )
                print(f"Verified observed {args.joint} delta: {observed:.6f} rad")
        else:
            print("Target joint positions:")
            for name, value in zip(JOINTS, target):
                print(f"  {name}: {value:.6f} rad")
            if args.dry_run:
                print("Dry run complete; no trajectory was sent.")
                return 0
            if args.verify_motion_min_rad > 0.0 and args.mode == "nudge":
                node.start_motion_observation(current)
            driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
            node.send_goal(target, args.duration, args.hold_after, args.start_delay, args.settle_before)
            driver_guard_offset = check_driver_log(args.driver_log, driver_guard_offset)
            if args.verify_motion_min_rad > 0.0 and args.mode == "nudge":
                joint_index = JOINTS.index(args.joint)
                observed = node.observed_joint_delta(args.joint, current[joint_index])
                if observed < args.verify_motion_min_rad:
                    raise RuntimeError(
                        f"Motion verification failed: observed {args.joint} delta "
                        f"{observed:.6f} rad < required {args.verify_motion_min_rad:.6f} rad."
                    )
                print(f"Verified observed {args.joint} delta: {observed:.6f} rad")

        print("Trajectory goal completed successfully.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
