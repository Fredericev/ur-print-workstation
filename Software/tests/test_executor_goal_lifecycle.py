"""Hardware-isolated action lifecycle tests at the executor/client boundary."""
from __future__ import annotations

from concurrent.futures import Future
import importlib.util
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock


EXECUTOR_PATH = Path(__file__).resolve().parents[2] / "workspaces" / "ur_ros2_ws" / "scripts" / "gcode_print_executor.py"
SPEC = importlib.util.spec_from_file_location("goal_lifecycle_executor", EXECUTOR_PATH)
assert SPEC and SPEC.loader
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


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
        self.now += duration
        while True:
            ready = [event for event in self.events if event[0] <= self.now + 1e-9]
            if not ready:
                return
            self.events = [event for event in self.events if event not in ready]
            for _, callback in sorted(ready, key=lambda event: event[0]):
                callback()


class Event:
    def __init__(self, clock):
        self.clock, self.flag = clock, False

    def is_set(self):
        return self.flag

    def set(self):
        self.flag = True

    def wait(self, timeout):
        if not self.flag:
            self.clock.sleep(timeout)
        return self.flag


class GoalHandle:
    def __init__(self, clock, *, accepted=True, terminal_after_cancel=0.01):
        self.clock = clock
        self.accepted = accepted
        self.terminal_after_cancel = terminal_after_cancel
        self.cancel_calls = 0
        self.result_future = Future()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.terminal_after_cancel is not None:
            def cancelled():
                if not self.result_future.done():
                    self.result_future.set_result(SimpleNamespace(
                        status=5, result=SimpleNamespace(error_code=0, success=False)
                    ))
            self.clock.later(self.terminal_after_cancel, cancelled)
        return resolved(SimpleNamespace(goals_canceling=["mock-goal"]))


class Client:
    def __init__(self, clock, *, ack_after=None, terminal_after_cancel=0.01):
        self.clock = clock
        self.ack_after = ack_after
        self.handle = GoalHandle(clock, terminal_after_cancel=terminal_after_cancel)
        self.future = Future()
        self.sent_count = 0

    def send_goal_async(self, _goal, **_kwargs):
        self.sent_count += 1
        if self.ack_after is not None:
            self.clock.later(self.ack_after, lambda: self.future.set_result(self.handle))
        return self.future


class GoalLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.clock = Clock()
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(runner, "time", self.clock).start()
        self.executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        self.executor.cancel_requested = Event(self.clock)
        self.executor.active_goals = []
        self.executor._initialize_goal_tracking()
        self.executor.status = {}
        self.executor._write_status = lambda: None
        self.executor.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.clock.now * 1e9)
        )))
        self.executor._trajectory_socket_state = (True, self.clock.now)
        marker = Path(self.directory.name) / "review.request.json"
        marker.write_text("test-only marker", encoding="utf-8")
        self.executor.manifest = {
            "planning_policy_version": runner.PLANNING_POLICY_VERSION,
            "printhead_geometry_sha256": runner.PRINTHEAD_GEOMETRY_SHA256,
            "planning_request": {"container_path": str(marker), "sha256": runner.sha256_file(marker)},
        }
        self.executor.require_live_program = lambda: None
        self.emergency_calls = []
        self.executor.emergency_client = SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda _request: self.emergency_calls.append(self.clock.now) or resolved(SimpleNamespace(success=True)),
        )
        self.executor.Trigger = SimpleNamespace(Request=lambda: object())

    def joint_frame(self, *, velocity=0.0, position=0.0, source_time=None, missing_velocity=False):
        nanoseconds = int((self.clock.now if source_time is None else source_time) * 1e9)
        return SimpleNamespace(
            name=list(runner.UR_JOINTS), position=[position] * 6,
            velocity=[] if missing_velocity else [velocity] * 6,
            header=SimpleNamespace(stamp=SimpleNamespace(
                sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000
            )),
        )

    def stream_joints(self, *, duration=1.0, interval=0.02, **values):
        for index in range(1, round(duration / interval) + 1):
            self.clock.later(index * interval, lambda: self.executor._joint_callback(self.joint_frame(**values)))

    def cancel_response(self):
        self.executor._set_status = lambda *_args, **_kwargs: None
        return self.executor._cancel_callback(None, SimpleNamespace())

    def submit_accepted_ur(self):
        client = Client(self.clock, ack_after=0.01)
        self.executor._send(
            client, object(), "UR", deadline=self.clock.now + 0.1
        )
        return client

    def test_expired_deadline_never_submits_a_goal(self):
        client = Client(self.clock)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(client, object(), "UR", deadline=self.clock.now - 0.01)
        self.assertEqual(client.sent_count, 0)

    def test_disconnected_or_stale_socket_during_dispatch_preparation_blocks_ur_goal(self):
        for connected, age_s in ((False, 0.0), (True, 3.0)):
            with self.subTest(connected=connected, age_s=age_s):
                client = Client(self.clock)
                self.executor._prepare_goal_for_dispatch = lambda *_args: setattr(
                    self.executor, "_trajectory_socket_state", (connected, self.clock.now - age_s)
                )

                with self.assertRaisesRegex(runner.ExecutionFailure, "trajectory socket"):
                    self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)

                self.assertEqual(client.sent_count, 0)
                self.assertEqual(self.executor._goal_requests, [])

    def test_cancel_service_does_not_confirm_without_post_stop_joint_feedback(self):
        client = Client(self.clock, ack_after=0.01)
        self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)
        self.executor._set_status = lambda *_args, **_kwargs: None
        response = self.executor._cancel_callback(None, SimpleNamespace())
        self.assertTrue(client.handle.result_future.done())
        self.assertFalse(response.success, "ROS action cancellation alone does not prove a stationary arm")
        self.assertIn("stationary", response.message)

    def test_cancel_service_confirms_only_after_fresh_continuous_stationary_feedback(self):
        self.submit_accepted_ur()
        self.stream_joints(duration=1.0)
        started = self.clock.now
        response = self.cancel_response()
        self.assertTrue(response.success, response.message)
        self.assertGreaterEqual(self.clock.now - started, 0.27)

    def test_cancel_service_rejects_missing_velocity_and_nonfinite_values(self):
        self.submit_accepted_ur()
        for values in ({"missing_velocity": True}, {"velocity": float("nan")}, {"position": float("inf")}):
            with self.subTest(values=values):
                self.stream_joints(**values)
                self.assertFalse(self.cancel_response().success)

    def test_cancel_service_rejects_single_stale_or_duplicate_feedback(self):
        self.submit_accepted_ur()
        # A full pre-stop stationary history cannot be reused as evidence.
        self.stream_joints(duration=0.5)
        self.clock.sleep(0.5)
        old_source = self.clock.now - 1.0
        self.stream_joints(source_time=old_source)
        self.assertFalse(self.cancel_response().success)
        self.clock.later(0.01, lambda: self.executor._joint_callback(self.joint_frame()))
        self.assertFalse(self.cancel_response().success, "A single new zero-velocity frame is not a dwell")
        duplicate = self.clock.now + 0.02
        self.stream_joints(source_time=duplicate)
        self.assertFalse(self.cancel_response().success, "Repeated source timestamps must not build a dwell")

    def test_cancel_service_rejects_moving_arm_even_after_action_is_terminal(self):
        client = Client(self.clock, ack_after=0.01)
        self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)
        self.stream_joints(velocity=0.004)
        self.assertFalse(self.cancel_response().success)
        self.assertEqual(client.handle.result_future.result().status, 5)

    def test_cancel_service_rejects_position_drift_despite_zero_reported_velocity(self):
        self.submit_accepted_ur()
        for index in range(1, 61):
            position = 0.002 if index % 2 else 0.0
            self.clock.later(index * 0.02, lambda position=position: self.executor._joint_callback(
                self.joint_frame(position=position)
            ))
        self.assertFalse(self.cancel_response().success)

    def test_feedback_gap_resets_stationary_dwell(self):
        self.submit_accepted_ur()
        self.stream_joints(duration=0.2)
        self.clock.later(0.4, lambda: self.executor._joint_callback(self.joint_frame()))
        self.assertFalse(self.cancel_response().success)

    def test_stale_live_state_never_sends_even_with_immediate_ack(self):
        client = Client(self.clock)
        client.future.set_result(client.handle)
        self.executor.require_live_program = mock.Mock(side_effect=runner.ExecutionFailure("live joints are stale"))
        with self.assertRaisesRegex(runner.ExecutionFailure, "stale"):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 1)
        self.assertEqual(client.sent_count, 0)

    def test_timeout_cancels_late_acceptance_and_waits_for_terminal_result(self):
        client = Client(self.clock, ack_after=0.08, terminal_after_cancel=0.05)
        with self.assertRaisesRegex(runner.ExecutionFailure, "Timed out sending UR goal"):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.02)
        self.stream_joints()
        confirmed, _message = self.executor._abort_active_goals(timeout_s=0.5)
        self.assertEqual(client.sent_count, 1)
        self.assertEqual(client.handle.cancel_calls, 1)
        self.assertTrue(confirmed)
        self.assertTrue(client.handle.result_future.done())
        self.assertEqual(client.handle.result_future.result().status, 5)

    def test_main_treats_an_unanswered_send_as_stop_unconfirmed(self):
        client = Client(self.clock)
        self.executor.wait_for_services = lambda: None
        self.executor.move_to_start = lambda _trajectory: self.executor._send(
            client, object(), "UR", deadline=self.clock.now + 0.02
        )
        self.executor.close = lambda: None
        self.executor._set_status = lambda state, message: setattr(self.executor, "terminal", (state, message))
        root = Path(self.directory.name)
        args = SimpleNamespace(
            manifest=root / "manifest",
            status=root / "status",
            mode="move-to-start",
        )
        manifest = {"execution_id": "mock", "approach_trajectory": {"container_path": str(root / "trajectory")}}
        with mock.patch.object(runner, "parse_args", return_value=args), mock.patch.object(
            runner, "load_manifest", return_value=manifest
        ), mock.patch.object(runner, "load_trajectory", return_value=[]), mock.patch.object(
            runner, "RosPrintExecutor", return_value=self.executor
        ):
            self.assertEqual(runner.main(), 2)
        self.assertEqual(self.executor.terminal[0], "stop-unconfirmed")
        self.assertIn("unknown", self.executor.terminal[1])
        self.assertEqual(client.sent_count, 1)

    def test_close_keeps_callbacks_alive_until_late_goal_is_terminal(self):
        client = Client(self.clock, ack_after=0.10, terminal_after_cancel=0.05)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.02)
        events = []
        self.executor.status = {"message": "Timed out sending UR goal", "state": "stop-unconfirmed"}
        self.executor._set_status = lambda state, message: self.executor.status.update(state=state, message=message)
        def restore():
            self.assertTrue(client.handle.result_future.done(), "Do not restore controllers before terminal stop")
            events.append("restore")
        self.executor._restore_scaled_controller = restore
        self.executor.node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
        def shutdown():
            events.append("shutdown")
            return True
        worker_pool = SimpleNamespace(
            shutdown=lambda *, wait: events.append("drain") if wait else None
        )
        self.executor.executor = SimpleNamespace(
            shutdown=shutdown,
            _executor=worker_pool,
            _futures=[],
        )
        self.executor.rclpy = SimpleNamespace(shutdown=lambda: None)
        self.executor.spin_thread = SimpleNamespace(
            join=lambda **kwargs: events.append("join"),
            is_alive=lambda: False,
        )
        self.executor.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.clock.now * 1e9)))
        self.stream_joints()
        self.executor.close()
        self.assertEqual(client.handle.cancel_calls, 1)
        self.assertEqual(events, ["restore", "shutdown", "join", "drain", "destroy"])

    def test_cancel_ack_without_terminal_result_does_not_confirm_stop(self):
        client = Client(self.clock, ack_after=0.01, terminal_after_cancel=None)
        self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)
        confirmed, message = self.executor._abort_active_goals(timeout_s=0.1)
        self.assertFalse(confirmed)
        self.assertIn("terminal stop is unconfirmed", message)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(Client(self.clock), object(), "UR", deadline=self.clock.now + 1)
        self.assertEqual(client.sent_count, 1)

    def test_unknown_stop_close_keeps_node_alive_with_watchdog(self):
        client = Client(self.clock)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.02)
        self.executor.status = {"message": "Timed out sending UR goal"}
        self.executor._set_status = lambda state, message: self.executor.status.update(state=state, message=message)
        self.executor._restore_scaled_controller = lambda: self.fail("unknown stop must not restore controller")
        self.executor.node = SimpleNamespace(destroy_node=lambda: self.fail("late callbacks must remain possible"))
        def watchdog(_seconds):
            raise TimeoutError("test watchdog: cleanup still waiting")
        with self.assertRaisesRegex(TimeoutError, "test watchdog"):
            self.executor.close(cleanup_wait=watchdog)
        self.assertEqual(self.executor.status["state"], "stop-unconfirmed")

    def test_cancellation_during_result_wait_is_not_submitted_twice(self):
        client = Client(self.clock, ack_after=0.01)
        handle, future = self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)
        self.executor.cancel_requested.set()
        self.stream_joints()
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._await_results([("UR", handle, future)], 1.0)
        confirmed, _message = self.executor._abort_active_goals(timeout_s=0.1)
        self.assertTrue(confirmed)
        self.assertEqual(handle.cancel_calls, 1)

    def test_dispatch_hook_cannot_send_after_spending_the_deadline(self):
        client = Client(self.clock)
        self.executor._prepare_goal_for_dispatch = lambda *_args: self.clock.sleep(0.2)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 0.1)
        self.assertEqual(client.sent_count, 0)

    def test_concurrent_stop_requests_share_one_cancel_and_emergency_request(self):
        client = Client(self.clock, ack_after=0.01)
        self.executor._send(client, object(), "Klipper", deadline=self.clock.now + 0.1)
        self.stream_joints()
        barrier, results = threading.Barrier(3), []
        def stop():
            barrier.wait(timeout=1)
            results.append(self.executor._abort_active_goals(timeout_s=0.5))
        threads = [threading.Thread(target=stop) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait(timeout=1)
        for thread in threads:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())
        self.assertEqual(len(results), 2)
        self.assertTrue(all(result[0] for result in results))
        self.assertEqual(client.handle.cancel_calls, 1)
        self.assertEqual(len(self.emergency_calls), 1)

    def test_failed_emergency_confirmation_is_retried_with_backoff(self):
        """Only the idempotent stop request may be retried after a lost reply."""
        client = Client(self.clock, ack_after=0.01)
        _handle, result_future = self.executor._send(
            client, object(), "Klipper", deadline=self.clock.now + 0.1
        )
        result_future.set_result(SimpleNamespace(
            status=6,
            result=SimpleNamespace(success=False, may_have_moved=True),
        ))
        responses = iter((
            resolved(SimpleNamespace(success=False)),
            resolved(SimpleNamespace(success=True)),
        ))
        calls = []
        self.executor.emergency_client = SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda _request: calls.append(self.clock.now) or next(responses),
        )

        confirmed, _message = self.executor._abort_active_goals(timeout_s=0.0)
        self.assertFalse(confirmed)
        self.assertEqual(len(calls), 1)

        self.clock.sleep(runner.EMERGENCY_STOP_RETRY_INTERVAL_S)
        confirmed, message = self.executor._abort_active_goals(timeout_s=0.1)

        self.assertTrue(confirmed, message)
        self.assertEqual(len(calls), 2)
        self.assertTrue(self.executor.status["stop"]["klipper_emergency_confirmed"])

    def test_terminal_klipper_failure_before_dispatch_does_not_emergency_stop(self):
        """An upload/validation rejection is not a moving printer.

        The bridge result carries the authoritative may_have_moved bit.  A
        terminal false value must let the coordinator clean up without turning
        a harmless pre-dispatch failure into a Klipper firmware shutdown.
        """
        client = Client(self.clock, ack_after=0.01)
        _handle, result_future = self.executor._send(
            client, object(), "Klipper", deadline=self.clock.now + 0.1
        )
        result_future.set_result(SimpleNamespace(
            status=6,
            result=SimpleNamespace(success=False, may_have_moved=False),
        ))

        confirmed, message = self.executor._abort_active_goals(timeout_s=0.5)

        self.assertTrue(confirmed, message)
        self.assertEqual(self.emergency_calls, [])
        self.assertFalse(self.executor.status["stop"]["klipper_emergency_required"])
        self.assertFalse(self.executor.status["stop"]["ur_stationary"]["required"])
        self.assertTrue(self.executor.status["stop"]["ur_stationary"]["confirmed"])

    def test_unknown_ur_submission_still_requires_post_stop_stationary_evidence(self):
        client = Client(self.clock)
        with self.assertRaisesRegex(runner.ExecutionFailure, "Timed out sending UR goal"):
            self.executor._send(
                client, object(), "UR", deadline=self.clock.now + 0.02
            )

        confirmed, message = self.executor._abort_active_goals(timeout_s=0.0)

        self.assertFalse(confirmed)
        self.assertIn("unknown", message)
        self.assertIn("stationary", message)
        self.assertTrue(self.executor.status["stop"]["ur_stationary"]["required"])
        self.assertFalse(self.executor.status["stop"]["ur_stationary"]["confirmed"])

    def test_transport_error_is_unknown_not_a_rejection_or_permission_to_retry(self):
        client = Client(self.clock)
        client.future.set_exception(RuntimeError("transport response lost"))
        with self.assertRaisesRegex(runner.ExecutionFailure, "unknown"):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 1)
        confirmed, message = self.executor._abort_active_goals(timeout_s=0)
        self.assertFalse(confirmed)
        self.assertIn("unknown", message)
        with self.assertRaises(runner.ExecutionFailure):
            self.executor._send(client, object(), "UR", deadline=self.clock.now + 1)
        self.assertEqual(client.sent_count, 1)


class FullExecutorSyntheticReplayTests(unittest.TestCase):
    @staticmethod
    def replay(**kwargs):
        path = Path(__file__).resolve().parent / "fixtures" / "ur_goal_timeout_repro.py"
        spec = importlib.util.spec_from_file_location("executor_synthetic_delay_replay", path)
        assert spec and spec.loader
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.run_case(**kwargs)

    def test_original_synthetic_ack_delays_complete_one_thirteen_segment_job(self):
        # Identical injected delays to the original red diagnostic, not a
        # replay of the real incident's raw manifest or measured latencies.
        result = self.replay(klipper_ack_s=0.70, ur_ack_s=0.40)
        self.assertEqual(result["return_code"], 0)
        self.assertEqual(result["terminal_state"], "completed")
        self.assertEqual(result["block"], 13)
        self.assertEqual(result["ur_requests_sent"], 1)
        self.assertFalse(result["hardware_contacted"])

    def test_full_failure_chain_waits_for_late_ack_and_cancels_without_retry(self):
        result = self.replay(klipper_ack_s=0.70, ur_ack_s=4.0)
        self.assertEqual(result["return_code"], 2)
        self.assertEqual(result["block"], 1)
        self.assertEqual(result["ur_requests_sent"], 1)
        self.assertTrue(result["last_ur_ack_done_when_main_returned"])
        self.assertEqual(result["last_ur_cancel_count_after_late_ack"], 1)


if __name__ == "__main__":
    unittest.main()
