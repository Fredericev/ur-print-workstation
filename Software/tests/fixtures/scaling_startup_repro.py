#!/usr/bin/env python3
"""DIAGNOSTIC ONLY: reproduce scaling readiness timing without ROS or hardware.

The production preflight, scaling callback and execute_print entry are real.
ROS clients, clocks and initial node resources are inert test doubles. Execution
stops at the reviewed-start-check boundary, before trajectory building or goal
submission. This is a synthetic timing experiment, not a replay of a real job.
"""
from __future__ import annotations

import argparse
from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


EXECUTOR_PATH = Path(__file__).resolve().parents[3] / "workspaces" / "ur_ros2_ws" / "scripts" / "gcode_print_executor.py"
SPEC = importlib.util.spec_from_file_location("scaling_diagnostic_executor", EXECUTOR_PATH)
assert SPEC and SPEC.loader
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class Clock:
    def __init__(self):
        self.now = 1000.0
        self.events = []

    def monotonic(self):
        return self.now

    time = monotonic

    def later(self, delay, callback):
        self.events.append((self.now + delay, callback))

    def sleep(self, duration):
        target = self.now + duration
        while self.events and min(item[0] for item in self.events) <= target:
            event = min(self.events, key=lambda item: item[0])
            self.events.remove(event)
            self.now = event[0]
            event[1]()
        self.now = target


class Event:
    def __init__(self, clock):
        self.clock = clock

    def is_set(self):
        return False

    def wait(self, timeout):
        self.clock.sleep(timeout)
        return False


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


class BoundaryReached(Exception):
    """Expected probe termination, before any motion preparation or dispatch."""


def run_case(*, preflight_switch_s=0.06, scaling=100.0, post_preflight_delay_s=0.0):
    clock = Clock()
    executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
    executor._initialize_goal_tracking()
    executor.latest_scaling = None
    executor.latest_scaling_stamp = 0.0
    executor.scaling_stable_since = 0.0
    executor.latest_joints = dict.fromkeys(runner.UR_JOINTS, 0.0)
    executor.latest_joint_stamp = clock.now
    executor.program_running = True
    executor.program_state_stamp = clock.now
    # This probe isolates speed-scaling readiness, so model the separate
    # trajectory-point socket as connected throughout its inert preflight.
    executor._trajectory_socket_state = (True, clock.now)
    executor.cancel_requested = Event(clock)
    executor.controller_switched = False
    executor.status = {"progress": 0.0}
    executor.manifest = {"operation_mode": "tool0_motion_test"}
    executor._write_status = lambda: None
    executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
    executor.SwitchController = SimpleNamespace(Request=lambda: SimpleNamespace(STRICT=2))
    executor.bridge_protocol_client = SimpleNamespace(
        wait_for_service=lambda **kwargs: True,
        call_async=lambda request: resolved(SimpleNamespace(values=[SimpleNamespace(
            string_value=runner.BRIDGE_PROTOCOL_VERSION
        )])),
    )
    executor.controller_protocol_client = SimpleNamespace(
        wait_for_service=lambda **kwargs: True,
        call_async=lambda request: resolved(SimpleNamespace(values=[SimpleNamespace(
            string_value=runner.CONTROLLER_STOP_PROTOCOL_VERSION
        )])),
    )
    mock_switch_requests = []

    def switch(request):
        mock_switch_requests.append(request)
        future = Future()
        clock.later(preflight_switch_s, lambda: future.set_result(SimpleNamespace(ok=True)))
        return future

    executor.switch_controller_client = SimpleNamespace(
        wait_for_service=lambda **kwargs: True, call_async=switch,
    )

    def forbidden_goal(*args, **kwargs):
        raise AssertionError("The diagnostic must not reach any goal submission")

    client = SimpleNamespace(
        server_is_ready=lambda: True, wait_for_server=lambda **kwargs: True,
        send_goal_async=forbidden_goal,
    )
    executor.ur_client = executor.klipper_client = client

    def stop_before_motion(*args):
        raise BoundaryReached()

    executor.require_start_match = stop_before_motion
    feedback_times = []

    def feedback():
        feedback_times.append(clock.now)
        executor._scaling_callback(SimpleNamespace(data=scaling))

    if scaling is not None:
        # Twenty 50 Hz messages: first at +10 ms; stream ends at +390 ms.
        for index in range(20):
            clock.later(0.01 + index * 0.02, feedback)

    with patch.object(runner, "time", clock):
        executor.wait_for_services()
        preflight_returned_at = clock.now
        clock.sleep(post_preflight_delay_s)
        entered_execute_at = clock.now
        error, traceback_frames = "", []
        try:
            executor.execute_print([{"positions": [0.0] * 6}])
        except BoundaryReached:
            result = "passed_scaling_guard"
        except runner.ExecutionFailure as exc:
            result, error = "rejected", str(exc)
            frame = exc.__traceback__
            while frame:
                traceback_frames.append({"function": frame.tb_frame.f_code.co_name, "line": frame.tb_lineno})
                frame = frame.tb_next
        else:
            raise AssertionError("Execution unexpectedly passed the diagnostic boundary")
        checked_at = clock.now

    return {
        "result": result,
        "error": error,
        "preflight_return_s": round(preflight_returned_at - 1000, 6),
        "execute_entered_s": round(entered_execute_at - 1000, 6),
        "execute_check_s": round(checked_at - 1000, 6),
        "first_scaling_feedback_s": round(feedback_times[0] - 1000, 6) if feedback_times else None,
        "scaling_percent": executor.latest_scaling,
        "feedback_age_s": round(checked_at - executor.latest_scaling_stamp, 6) if feedback_times else None,
        "stable_age_s": round(checked_at - executor.scaling_stable_since, 6) if feedback_times else None,
        "feedback_count": len(feedback_times),
        "mock_controller_switch_count": len(mock_switch_requests),
        "goal_requests_sent": 0,
        "progress": executor.status["progress"],
        "traceback": traceback_frames,
        "hardware_contacted": False,
        "synthetic_not_incident_replay": True,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assert-startup-waits", action="store_true",
                        help="Red-capable assertion: a healthy 100%% stream must not fail only because startup was quick")
    args = parser.parse_args()
    fast = run_case()
    print(json.dumps({"fast_preflight_healthy_scaling": fast}, indent=2))
    if args.assert_startup_waits:
        assert fast["result"] == "passed_scaling_guard", (
            "Healthy 100% scaling was rejected at startup instead of waiting for its 0.25 s observation window"
        )
    else:
        print(json.dumps({
            "slower_preflight_same_feedback": run_case(preflight_switch_s=0.30),
            "fast_preflight_then_wait_0_30s": run_case(post_preflight_delay_s=0.30),
            "missing_scaling": run_case(scaling=None),
            "scaled_20_percent": run_case(preflight_switch_s=0.30, scaling=20.0),
            "stale_feedback": run_case(preflight_switch_s=0.30, post_preflight_delay_s=2.2),
        }, indent=2))


if __name__ == "__main__":
    main()
