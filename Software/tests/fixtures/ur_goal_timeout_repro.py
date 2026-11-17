#!/usr/bin/env python3
"""DIAGNOSTIC / REGRESSION ONLY: virtual-clock executor lifecycle replay.

No ROS imports, sockets, actions, controllers or hardware are used. This tests
what the existing code does if acknowledgement is delayed; it does not identify
the reason for the acknowledgement delay in a real robot incident. The original
0.70 s / 0.40 s synthetic delays now complete on the repaired executor; longer
acknowledgement delays still fail closed and the late goal must be cancelled.
"""

from __future__ import annotations

import argparse
from concurrent.futures import Future
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch


EXECUTOR_PATH = Path(__file__).resolve().parents[3] / "workspaces" / "ur_ros2_ws" / "scripts" / "gcode_print_executor.py"
spec = importlib.util.spec_from_file_location("diagnostic_real_executor", EXECUTOR_PATH)
assert spec and spec.loader
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
Executor = runner.RosPrintExecutor


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


class Clock:
    def __init__(self):
        self.seconds = 1000.0
        self.events = []

    def monotonic(self):
        return self.seconds

    time = monotonic

    def later(self, delay, callback):
        self.events.append((self.seconds + delay, callback))

    def sleep(self, seconds):
        self.seconds += seconds
        ready = [event for event in self.events if event[0] <= self.seconds + 1e-9]
        self.events = [event for event in self.events if event not in ready]
        for _, callback in sorted(ready, key=lambda event: event[0]):
            callback()


class Event:
    def __init__(self, clock):
        self.clock = clock
        self.flag = False

    def set(self):
        self.flag = True

    def is_set(self):
        return self.flag

    def wait(self, timeout):
        if not self.flag:
            self.clock.sleep(timeout)
        return self.flag


class Handle:
    accepted = True

    def __init__(self, label):
        self.label = label
        self.result_future = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        if not self.result_future.done():
            self.result_future.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0, success=False)))
        return resolved(SimpleNamespace(goals_canceling=[self.label]))


class Client:
    def __init__(self, clock, label, delays):
        self.clock, self.label, self.delays = clock, label, delays
        self.requests = []

    def send_goal_async(self, goal, **kwargs):
        number = len(self.requests)
        delay = self.delays[min(number, len(self.delays) - 1)]
        future, handle = Future(), Handle(self.label)
        record = {
            "sent_at": self.clock.seconds, "ack_delay": delay,
            "handle": handle, "future": future,
        }
        self.requests.append(record)

        def accept():
            future.set_result(handle)
            feedback = kwargs.get("feedback_callback")
            if feedback:
                feedback(SimpleNamespace(feedback=SimpleNamespace(
                    phase=2, program_epoch_ros_ns=int(self.clock.seconds * 1e9)
                )))

        self.clock.later(delay, accept)
        def finish():
            if not handle.result_future.done():
                handle.result_future.set_result(SimpleNamespace(
                    status=4, result=SimpleNamespace(error_code=0, success=True)
                ))
        self.clock.later(delay + 2.0, finish)
        return future


def run_case(klipper_ack_s, ur_ack_s, *, ordinary_ur_only=False, ur_goal_build_s=0.0):
    clock = Clock()
    executor = Executor.__new__(Executor)
    executor.cancel_requested = Event(clock)
    executor.active_goals = []
    executor._initialize_goal_tracking()
    executor.latest_scaling = None
    executor.latest_scaling_stamp = 0.0
    executor.scaling_stable_since = 0.0
    # The synthetic replay isolates action acknowledgement timing. Model the
    # independent trajectory-point connection as live so it reaches that seam.
    executor._trajectory_socket_state = (True, clock.seconds)
    executor.status = {}
    executor.closed = False
    executor.ur_client = Client(clock, "UR", [ur_ack_s])
    executor.klipper_client = Client(clock, "Klipper", [klipper_ack_s])
    emergency = []
    executor.emergency_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: emergency.append(clock.seconds) or resolved(
            SimpleNamespace(success=True)
        ),
    )
    executor.Trigger = SimpleNamespace(Request=lambda: object())
    executor.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(clock.seconds * 1e9)
    )))
    executor.ManualStepperProgram = SimpleNamespace(
        Feedback=SimpleNamespace(PHASE_DISPATCHED=2), Goal=lambda: SimpleNamespace()
    )
    executor.FollowJointTrajectory = SimpleNamespace(Goal=lambda: SimpleNamespace(
        trajectory=SimpleNamespace(joint_names=[], points=[], header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0)
        ))
    ))
    executor.JointTolerance = lambda: SimpleNamespace(
        name="", position=0.0, velocity=0.0, acceleration=0.0
    )
    executor.require_start_match = lambda *args: None
    executor.require_live_program = lambda: None
    executor.wait_for_services = lambda: None
    executor._require_coordination_scaling = lambda: None
    real_trajectory_goal = executor._trajectory_goal
    def inert_ur_goal(*args, **kwargs):
        clock.sleep(ur_goal_build_s)
        return real_trajectory_goal(*args, **kwargs)

    executor._trajectory_goal = inert_ur_goal
    executor._write_status = lambda: None
    executor.close = lambda: setattr(executor, "closed", True)
    joint_feedback_count = 0

    def stationary_joint_feedback():
        # Drive the production observation callback, not a mocked stop verdict.
        # This synthetic stream proves no facts about real arm motion. Keep the
        # independent socket heartbeat live while isolating ACK timing.
        nonlocal joint_feedback_count
        joint_feedback_count += 1
        executor._trajectory_socket_callback(SimpleNamespace(data=True))
        executor._scaling_callback(SimpleNamespace(data=100.0))
        stamp_ns = int(clock.seconds * 1e9)
        executor._joint_callback(SimpleNamespace(
            name=list(runner.UR_JOINTS), position=[0.0] * 6, velocity=[0.0] * 6,
            header=SimpleNamespace(stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000
            )),
        ))
        clock.later(0.02, stationary_joint_feedback)

    with tempfile.TemporaryDirectory(prefix="ur-goal-timeout-mock-") as directory:
        root = Path(directory)
        marker = root / "planning.request.json"
        marker.write_text("mock-only review marker", encoding="utf-8")
        executor.manifest = {
            "version": 3,
            "planning_policy_version": runner.PLANNING_POLICY_VERSION,
            "printhead_geometry_sha256": runner.PRINTHEAD_GEOMETRY_SHA256,
            "planning_request": {"container_path": str(marker), "sha256": runner.sha256_file(marker)},
            "execution_id": "hardware-isolated-mock",
            "operation_mode": "tool0_motion_test",
            "manual_stepper": "extruder", "extruder": "extruder",
            "extrusion_protocol": "klipper-native-extruder-v1",
            "stepper_limits": {"maximum_speed": 5, "acceleration": 20, "max_extrude_only_distance": 50},
            "print_trajectory": {"container_path": str(root / "unused.csv")},
            "blocks": [
                {
                    "index": index, "kind": "travel" if index == 0 else "print",
                    "start_progress": index / 100, "end_progress": (index + 1) / 100,
                    "duration_s": 2.0,
                    "klipper_units": 0 if (index == 0 or ordinary_ur_only) else 1,
                    "source_line_start": 63 + index,
                    "source_line_end": 63 + index,
                    "klipper_target_position": 1.0,
                }
                for index in range(13)
            ],
        }
        trajectory = [
            {"path_progress": 0, "time_s": 0, "positions": [0] * 6},
            {"path_progress": 1, "time_s": 30, "positions": [0] * 6},
        ]
        points = []
        for index in range(14):
            markers = []
            if index > 0:
                markers.append(f"block:{index-1}:end")
            if index < 13:
                markers.append(f"block:{index}:start")
            points.append({
                "time_s": index * 2.0, "positions": [0.0] * 6,
                "timeline_markers": markers, "path_progress": index / 100,
            })
        profile = runner.compile_profile(points)
        for block in executor.manifest["blocks"]:
            block["start_time_s"] = block["index"] * 2.0
            block["end_time_s"] = (block["index"] + 1) * 2.0
        executor.manifest["trajectory_policy_version"] = runner.TRAJECTORY_POLICY
        executor.manifest["trajectory_profiles"] = {
            "approach": runner.compile_profile(
                [{"time_s":0,"positions":[0]*6}, {"time_s":1,"positions":[0]*6}],
                limit_class="approach",
            ),
            "print": profile,
        }
        def fixture_curve_check(profile):
            # This fixture tests transport timing, NOT collision checking.
            # Command-boundary smooth tests cover the real read-only scene gate.
            executor._validated_profile_identity = executor._profile_identity(profile)
            executor._validated_start_positions = [0.0]*6
            executor._minimum_validated_hold_s = .05
        executor._validate_smooth_motion = fixture_curve_check
        args = SimpleNamespace(manifest=root / "unused.json", status=root / "unused-status.json",
                               mode="execute")
        messages = {
            "builtin_interfaces.msg": SimpleNamespace(Duration=lambda **values: SimpleNamespace(**values)),
            "trajectory_msgs.msg": SimpleNamespace(JointTrajectoryPoint=lambda: SimpleNamespace()),
        }
        with patch.dict(sys.modules, messages), patch.object(runner, "time", clock), patch.object(
            runner, "threading", SimpleNamespace(Event=lambda: Event(clock))
        ), patch.object(runner, "parse_args", return_value=args), patch.object(
            runner, "load_manifest", return_value=executor.manifest
        ), patch.object(runner, "load_trajectory", return_value=trajectory), patch.object(
            runner, "RosPrintExecutor", return_value=executor
        ), contextlib.redirect_stdout(io.StringIO()) as output:
            clock.later(0.02, stationary_joint_feedback)
            code = runner.main()
            time_at_return = clock.seconds
            before_late = [record["future"].done() for record in executor.ur_client.requests]
            clock.sleep(max(ur_ack_s, 1) + 0.1)

    return {
        "klipper_ack_s": klipper_ack_s, "ur_ack_s": ur_ack_s,
        "ur_goal_build_s": ur_goal_build_s,
        "ordinary_ur_only": ordinary_ur_only, "return_code": code,
        "terminal_state": executor.status.get("state"),
        "terminal_message": executor.status.get("message"),
        "block": executor.status.get("current_block"),
        "source_line": executor.status.get("source_line"),
        "progress": executor.status.get("progress"),
        "ur_requests_sent": len(executor.ur_client.requests),
        "last_ur_ack_done_when_main_returned": before_late[-1] if before_late else None,
        "last_ur_ack_done_after_advance": executor.ur_client.requests[-1]["future"].done() if before_late else None,
        "last_ur_cancel_count_after_late_ack": executor.ur_client.requests[-1]["handle"].cancel_count if before_late else 0,
        "elapsed_from_last_ur_send_to_main_return_s": round(time_at_return - executor.ur_client.requests[-1]["sent_at"], 6) if before_late else None,
        "klipper_goal_cancel_count": sum(record["handle"].cancel_count for record in executor.klipper_client.requests),
        "klipper_emergency_stop_count": len(emergency),
        "active_goals_after_failure_cleanup": len(executor.active_goals),
        "stdout": output.getvalue().strip(),
        "hardware_contacted": False,
        "close_and_controller_restore_mocked": True,
        "live_readiness_and_scaling_mocked": True,
        "joint_feedback": "synthetic stationary six-axis frames through the real joint callback",
        "joint_feedback_frame_count": joint_feedback_count,
    }


def expired_dispatch_case():
    clock = Clock()
    executor = Executor.__new__(Executor)
    executor.active_goals = []
    executor.cancel_requested = Event(clock)
    executor._initialize_goal_tracking()
    executor.require_live_program = lambda: None
    executor._prepare_goal_for_dispatch = lambda *_args: clock.sleep(0.20)
    client = Client(clock, "UR", [0.01])
    with tempfile.TemporaryDirectory(prefix="ur-expired-send-mock-") as directory:
        marker = Path(directory) / "planning.request.json"
        marker.write_text("test-only marker", encoding="utf-8")
        executor.manifest = {
            "planning_policy_version": runner.PLANNING_POLICY_VERSION,
            "printhead_geometry_sha256": runner.PRINTHEAD_GEOMETRY_SHA256,
            "planning_request": {"container_path": str(marker), "sha256": runner.sha256_file(marker)},
        }
        with patch.object(runner, "time", clock):
            try:
                executor._send(client, object(), "UR", deadline=clock.seconds + 0.10)
            except runner.ExecutionFailure as error:
                return {"ur_requests_sent": len(client.requests), "message": str(error), "hardware_contacted": False}
    raise AssertionError("An expired dispatch unexpectedly returned success")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assert-no-orphan", action="store_true")
    parser.add_argument("--assert-no-send-after-deadline", action="store_true")
    args = parser.parse_args()
    delayed = run_case(0.70, 0.40)
    fast = run_case(0.20, 0.40)
    ordinary = run_case(0.0, 8.20, ordinary_ur_only=True)
    exhausted = run_case(0.70, 0.01, ur_goal_build_s=0.40)
    overdue = run_case(0.70, 4.0)
    expired = expired_dispatch_case()
    assert delayed["terminal_state"] == "completed" and delayed["block"] == 13, delayed
    assert fast["return_code"] == 0 and fast["terminal_state"] == "completed", fast
    assert ordinary["terminal_message"] == "Timed out sending UR goal", ordinary
    assert exhausted["terminal_state"] == "completed", exhausted
    assert overdue["last_ur_cancel_count_after_late_ack"] == 1, overdue
    assert expired["ur_requests_sent"] == 0, expired
    report = {"original_synthetic_delays_now_complete": delayed, "faster_klipper_ack_only": fast,
              "ordinary_ur_ack_beyond_8s": ordinary,
              "former_build_budget_exhaustion_now_prebuilt": exhausted,
              "late_ack_is_now_cancelled": overdue,
              "expired_final_dispatch_sends_nothing": expired}
    print(json.dumps(report, indent=2))
    if args.assert_no_orphan:
        assert overdue["last_ur_cancel_count_after_late_ack"] > 0, (
            "Late accepted UR goal was sent but never registered/cancelled after _send timed out"
        )
    if args.assert_no_send_after_deadline:
        assert expired["ur_requests_sent"] == 0, expired


if __name__ == "__main__":
    main()
