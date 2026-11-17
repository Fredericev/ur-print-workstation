import importlib.util
import json
from pathlib import Path

import pytest

from ur_klipper_bridge.gcode import (
    build_completion_barrier_script,
    build_disable_script,
    build_dispatch_script,
    build_program_file,
    start_delay_until,
)
from ur_klipper_bridge.validation import ValidationError


def test_builds_non_blocking_dispatch_with_optional_coordinate_reset():
    script = build_dispatch_script(
        stepper="extruder",
        target_position=60.0,
        speed=5.0,
        acceleration=20.0,
        set_position_before_move=True,
        initial_position=0.0,
    )
    assert script.splitlines() == [
        "SAVE_GCODE_STATE NAME=ur_native_extrusion",
        "ACTIVATE_EXTRUDER EXTRUDER=extruder", "G21", "M83",
        "G1 E30 F300", "G1 E30 F300",
    ]


def test_completion_barrier_waits_for_mcu_queue_not_only_gcode_queue():
    assert build_completion_barrier_script(
        stepper="extruder",
        target_position=-12.5,
        speed=2.5,
        acceleration=10.0,
    ) == "M400\nRESTORE_GCODE_STATE NAME=ur_native_extrusion"


def test_builds_disable_command():
    assert build_disable_script("extruder") == (
        "SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0"
    )


def test_absolute_start_converts_to_one_guarded_remaining_delay():
    assert start_delay_until(11_250_000_000, 10_000_000_000) == 1.25


def test_absolute_start_fails_closed_after_arming_guard_is_missed():
    with pytest.raises(ValidationError):
        start_delay_until(10_100_000_000, 10_000_000_000)


@pytest.mark.parametrize("elapsed_s", [0.0, 0.05, 0.5, 2.0])
def test_executor_start_plan_is_accepted_by_real_bridge_builders(elapsed_s):
    executor_path = Path(__file__).resolve().parents[3] / "scripts" / "gcode_print_executor.py"
    spec = importlib.util.spec_from_file_location("bridge_timing_real_executor", executor_path)
    executor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(executor)
    now_ns = 10_000_000_000
    plan = executor.synchronized_start_plan(now_ns)
    remaining = start_delay_until(
        plan["klipper_start_ns"], now_ns + round(elapsed_s * 1_000_000_000)
    )
    assert remaining == pytest.approx(executor.SYNC_START_LEAD_S - elapsed_s)
    script = build_dispatch_script(
        stepper="extruder", target_position=0.3, speed=0.1,
        acceleration=20.0, set_position_before_move=True,
        initial_position=0.0, start_delay_s=remaining,
    )
    assert f"G4 P{round(remaining * 1000)}" in script.splitlines()


def test_start_horizon_remains_bounded_at_five_seconds():
    assert start_delay_until(15_000_000_000, 10_000_000_000) == 5.0
    with pytest.raises(ValidationError):
        start_delay_until(15_001_000_000, 10_000_000_000)
    with pytest.raises(ValueError):
        build_dispatch_script(
            stepper="extruder", target_position=0.3, speed=0.1,
            acceleration=20.0, set_position_before_move=False,
            initial_position=0.0, start_delay_s=5.001,
        )


def test_stepper_name_cannot_inject_gcode():
    with pytest.raises(ValidationError):
        build_disable_script("stepper_x\nM112")


def test_continuous_program_has_one_start_dwell_and_one_terminal_barrier():
    script = build_program_file(
        stepper="extruder",
        segments=[
            {"units": 1.0, "duration_s": 2.0},
            {"units": 0.5, "duration_s": 1.0},
            {"units": 0.0, "duration_s": 1.25},
            {"units": -0.25, "duration_s": 0.5},
        ],
        start_delay_s=4.0,
        maximum_speed=5.0,
        disable_after=True,
    )
    lines = script.splitlines()
    assert lines[:5] == [
        "SAVE_GCODE_STATE NAME=ur_native_extrusion",
        "ACTIVATE_EXTRUDER EXTRUDER=extruder", "G21", "M83", "G4 P4000",
    ]
    assert lines[5].startswith("G1 E1 F")
    assert lines[6].startswith("G1 E0.5 F")
    travel = lines.index("G4 P1250")
    assert lines[travel + 1].startswith("G1 E-0.25 F")
    assert lines[-3:] == [
        "M400", "RESTORE_GCODE_STATE NAME=ur_native_extrusion",
        "SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0",
    ]
    assert lines.count("M400") == 1
    assert "MANUAL_STEPPER" not in script
    assert "M220" not in script and "M221" not in script and "M204" not in script


def test_full_program_dwell_rounding_does_not_accumulate_clock_drift():
    script = build_program_file(
        stepper="extruder",
        segments=[{"units": 0.0, "duration_s": .0015} for _ in range(903)],
        start_delay_s=0.0, maximum_speed=5.0, disable_after=True,
    )
    milliseconds = sum(int(line[4:]) for line in script.splitlines() if line.startswith("G4 P"))
    assert abs(milliseconds/1000 - 903*.0015) <= .0005 + 1e-9


def test_continuous_program_rejects_a_segment_faster_than_installed_limit():
    with pytest.raises(ValidationError, match="minimum"):
        build_program_file(
            stepper="extruder",
            segments=[{"units": 2.0, "duration_s": 0.1}],
            start_delay_s=4.0,
            maximum_speed=5.0,
            disable_after=False,
        )


def test_executor_close_drains_ros_workers_before_destroying_node():
    executor_path = Path(__file__).resolve().parents[3] / "scripts" / "gcode_print_executor.py"
    spec = importlib.util.spec_from_file_location("executor_cleanup_order", executor_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    events = []
    node_alive = True

    class HandlerFuture:
        def __init__(self):
            self._done = False
            self._exception = None
            self.exception_retrieved = False

        def done(self):
            return self._done

        def run_queued_handler(self):
            assert node_alive
            events.append("queued_handler_ran")
            self._exception = RuntimeError("queued callback failed")
            self._done = True

        def result(self):
            assert node_alive
            events.append("handler_result")
            self.exception_retrieved = True
            raise self._exception

    future = HandlerFuture()

    class WorkerPool:
        def shutdown(self, *, wait):
            assert wait is True
            assert node_alive
            future.run_queued_handler()
            events.append("worker_pool_drained")

    class Executor:
        def __init__(self):
            self._executor = WorkerPool()
            self._futures = [future]

        def shutdown(self, *args, **kwargs):
            events.append("executor_shutdown")
            return True

    class SpinThread:
        def join(self, *args, **kwargs):
            events.append("spin_joined")

        @staticmethod
        def is_alive():
            return False

    class Logger:
        @staticmethod
        def error(_message):
            events.append("cleanup_error_logged")

    class Node:
        @staticmethod
        def get_logger():
            return Logger()

        def destroy_node(self):
            nonlocal node_alive
            events.append("node_destroyed")
            node_alive = False

    class Rclpy:
        @staticmethod
        def shutdown():
            assert not node_alive
            events.append("rclpy_shutdown")

    instance = object.__new__(module.RosPrintExecutor)
    instance._has_unresolved_goals = lambda: False
    instance.active_goals = []
    instance._stopping = False
    instance._stop_confirmed = True
    instance._restore_scaled_controller = lambda: None
    instance.executor = Executor()
    instance.spin_thread = SpinThread()
    instance.node = Node()
    instance.rclpy = Rclpy()

    instance.close()

    assert events == [
        "executor_shutdown",
        "spin_joined",
        "queued_handler_ran",
        "worker_pool_drained",
        "handler_result",
        "cleanup_error_logged",
        "node_destroyed",
        "rclpy_shutdown",
    ]
    assert future.exception_retrieved is True


def test_executor_close_does_not_destroy_ros_if_worker_drain_is_unavailable(
    monkeypatch, tmp_path
):
    executor_path = Path(__file__).resolve().parents[3] / "scripts" / "gcode_print_executor.py"
    spec = importlib.util.spec_from_file_location("executor_cleanup_fail_closed", executor_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    events = []

    class HardExit(Exception):
        def __init__(self, code):
            self.code = code

    class ExecutorWithoutWorkerPool:
        _futures = []

        @staticmethod
        def shutdown():
            events.append("executor_shutdown")
            return True

    class SpinThread:
        @staticmethod
        def join(*_args, **_kwargs):
            events.append("spin_joined")

        @staticmethod
        def is_alive():
            return False

    class Logger:
        @staticmethod
        def error(_message):
            events.append("cleanup_error_logged")

    class Node:
        @staticmethod
        def get_logger():
            return Logger()

        @staticmethod
        def destroy_node():
            events.append("node_destroyed")

    class Rclpy:
        @staticmethod
        def shutdown():
            events.append("rclpy_shutdown")

    instance = object.__new__(module.RosPrintExecutor)
    instance._has_unresolved_goals = lambda: False
    instance.active_goals = []
    instance._stopping = False
    instance._stop_confirmed = True
    instance._restore_scaled_controller = lambda: None
    instance.executor = ExecutorWithoutWorkerPool()
    instance.spin_thread = SpinThread()
    instance.node = Node()
    instance.rclpy = Rclpy()
    instance.status_path = tmp_path / "status.json"
    instance.manifest = {"execution_id": "cleanup-test"}
    instance._set_status = lambda *_args, **_updates: events.append(
        "unsafe_status_write"
    )

    def hard_exit(code):
        events.append("hard_exit")
        raise HardExit(code)

    monkeypatch.setattr(module.os, "_exit", hard_exit)

    with pytest.raises(HardExit) as exited:
        instance.close()

    assert exited.value.code == 70
    assert events == [
        "executor_shutdown",
        "spin_joined",
        "hard_exit",
    ]
    status = json.loads(instance.status_path.read_text(encoding="utf-8"))
    assert status["state"] == "failed"
    assert status["execution_id"] == "cleanup-test"
    assert status["cleanup"]["confirmed"] is False
