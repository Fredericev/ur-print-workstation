#!/usr/bin/env python3
"""DIAGNOSTIC ONLY: accepted actions, scaling failure, missing result/feedback.

Runs the actual executor send/abort/close lifecycle using inert action futures.
A separate local thread delivers result futures and joint callbacks while close
is waiting. No ROS imports, network, controller calls or hardware are used.
Missing UR results and joint data model consequences consistent with a driver
crash; they do not prove the real crash cause or recreate DDS scheduling.
"""
from __future__ import annotations

import argparse
from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace


EXECUTOR_PATH = Path(__file__).resolve().parents[3] / "workspaces" / "ur_ros2_ws" / "scripts" / "gcode_print_executor.py"
SPEC = importlib.util.spec_from_file_location("stop_unknown_diagnostic_executor", EXECUTOR_PATH)
assert SPEC and SPEC.loader
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self):
        self.result_future = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return resolved(SimpleNamespace(goals_canceling=["mock-goal"]))


class DiagnosticWatchdog(Exception):
    pass


def run_case(mode):
    assert mode in {"healthy_callbacks", "ur_result_and_joint_stream_lost", "joint_stream_lost_only"}
    executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
    executor._initialize_goal_tracking()
    executor.active_goals = []
    executor.cancel_requested = threading.Event()
    executor.latest_scaling = None
    executor.scaling_stable_since = 0.0
    executor._scaling_callback(SimpleNamespace(data=100.0))
    executor.scaling_stable_since -= 1.0
    executor._coordination_active = True
    executor.program_running, executor.program_state_stamp = True, time.monotonic()
    executor._trajectory_socket_state = (True, time.monotonic())
    executor.latest_joints = dict.fromkeys(runner.UR_JOINTS, 0.0)
    executor.latest_joint_stamp = time.monotonic()
    executor.Trigger = SimpleNamespace(Request=lambda: object())
    executor.emergency_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: resolved(SimpleNamespace(success=True)),
    )
    events = []
    executor.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns())),
        destroy_node=lambda: events.append("destroy_node"),
    )
    executor.executor = SimpleNamespace(shutdown=lambda **kwargs: events.append("shutdown_executor"))
    executor.rclpy = SimpleNamespace(shutdown=lambda: events.append("shutdown_rclpy"))
    executor.spin_thread = SimpleNamespace(join=lambda **kwargs: events.append("join_spin_thread"))
    executor._restore_scaled_controller = lambda: events.append("mock_restore_controller")
    executor.status = {}
    handles = {label: Handle() for label in ("Klipper", "UR")}

    with tempfile.TemporaryDirectory(prefix="stop-unknown-diagnostic-") as directory:
        executor.status_path = Path(directory) / "diagnostic.status.json"
        marker = Path(directory) / "planning.request.json"
        marker.write_text("diagnostic-only marker", encoding="utf-8")
        executor.manifest = {
            "planning_policy_version": runner.PLANNING_POLICY_VERSION,
            "printhead_geometry_sha256": runner.PRINTHEAD_GEOMETRY_SHA256,
            "planning_request": {"container_path": str(marker), "sha256": runner.sha256_file(marker)},
        }
        entries = []
        for label, handle in handles.items():
            client = SimpleNamespace(send_goal_async=lambda goal, handle=handle: resolved(handle))
            result = executor._send(client, object(), label)
            entries.append((label, *result))
        executor._scaling_callback(SimpleNamespace(data=0.0))
        try:
            executor._await_results(entries, 1.0)
        except runner.ExecutionFailure as exc:
            scaling_failure = str(exc)
            executor._record_root_error(exc)
        else:
            raise AssertionError("The real scaling guard did not reject the dropped scaling")

        initial_confirmed, initial_message = executor._abort_active_goals(timeout_s=0.03)
        executor._publish_failure_state("stop-unconfirmed")
        callback_errors, callback_count = [], []
        worker_stop = threading.Event()

        def complete(label):
            handles[label].result_future.set_result(SimpleNamespace(
                status=5, result=SimpleNamespace(error_code=0, success=False)
            ))
            events.append("result_" + label)

        def deliver_callbacks():
            try:
                time.sleep(0.05)
                complete("Klipper")
                if mode != "ur_result_and_joint_stream_lost":
                    complete("UR")
                if mode == "healthy_callbacks":
                    for _ in range(60):
                        if worker_stop.is_set():
                            return
                        stamp = time.time_ns()
                        executor._joint_callback(SimpleNamespace(
                            name=list(runner.UR_JOINTS), position=[0.0] * 6, velocity=[0.0] * 6,
                            header=SimpleNamespace(stamp=SimpleNamespace(
                                sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000
                            )),
                        ))
                        callback_count.append(1)
                        time.sleep(0.02)
            except Exception as exc:
                callback_errors.append(str(exc))

        worker = threading.Thread(target=deliver_callbacks, daemon=True)
        worker.start()

        def watchdog(_seconds):
            raise DiagnosticWatchdog("Bounded observation: real close is still retaining the node")

        started, close_returned = time.monotonic(), False
        try:
            executor.close(cleanup_wait=watchdog)
            close_returned = True
        except DiagnosticWatchdog:
            pass
        finally:
            worker_stop.set()
            worker.join(timeout=1.0)
        records = [{
            "label": record["label"], "acknowledged": record["acknowledged"],
            "result_done": bool(record.get("result_future") and record["result_future"].done()),
            "terminal": executor._request_is_terminal(record),
        } for record in executor._goal_requests]

    assert not callback_errors and not worker.is_alive(), (callback_errors, "callback worker stuck")
    return {
        "mode": mode, "initial_stop_confirmed": initial_confirmed,
        "close_returned": close_returned, "close_observed_s": round(time.monotonic() - started, 3),
        "status": executor.status, "remaining_ledger": records,
        "cancel_counts": {label: handle.cancel_count for label, handle in handles.items()},
        "joint_callbacks_delivered_on_other_thread": len(callback_count), "events": events,
        "hardware_contacted": False, "synthetic_not_incident_replay": True,
        "ros_dds_and_executor_scheduling_not_emulated": True,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assert-crash-close-completes", action="store_true",
                        help="Deliberately red symptom assertion; missing stop evidence must retain the node")
    args = parser.parse_args()
    if args.assert_crash_close_completes:
        case = run_case("ur_result_and_joint_stream_lost")
        print(json.dumps(case, indent=2))
        assert case["close_returned"], "Reproduced persistent stop-unconfirmed while UR terminal result and joint feedback are absent"
        return
    cases = [run_case(mode) for mode in (
        "healthy_callbacks", "ur_result_and_joint_stream_lost", "joint_stream_lost_only"
    )]
    print(json.dumps(cases, indent=2))
    assert cases[0]["close_returned"] and not cases[0]["remaining_ledger"], "Terminal callbacks were not reconciled"
    for case in cases[1:]:
        assert not case["close_returned"] and "destroy_node" not in case["events"], "Stop evidence was bypassed"
    assert all(all(count == 1 for count in case["cancel_counts"].values()) for case in cases)


if __name__ == "__main__":
    main()
