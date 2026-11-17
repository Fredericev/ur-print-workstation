"""Hardware-isolated readiness and structured-stop tests through real executor calls."""
from __future__ import annotations

import importlib.util
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

from Software.tests.test_executor_goal_lifecycle import Clock, Event, resolved, runner


class StopAtReviewedStart(Exception):
    pass


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patch = mock.patch.object(runner, "time", self.clock)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        self.executor._initialize_goal_tracking()
        self.executor.latest_scaling = None
        self.executor.latest_scaling_stamp = 0.0
        self.executor.scaling_stable_since = 0.0
        self.executor.program_running = True
        self.executor.program_state_stamp = self.clock.now
        self.executor.latest_joints = {name: 0.0 for name in runner.UR_JOINTS}
        self.executor.latest_joint_stamp = self.clock.now
        self.executor.cancel_requested = Event(self.clock)
        self.executor.manifest = {"operation_mode": "tool0_motion_test"}
        self.executor.status = {}
        self.executor._write_status = lambda: None
        self.executor.require_start_match = lambda *args: (_ for _ in ()).throw(StopAtReviewedStart())

    def execute(self):
        return self.executor.execute_print([{"positions": [0.0] * 6}])

    def samples(self, value, *, start=0.01, duration=6.0):
        for index in range(round(duration / 0.02)):
            self.clock.later(start + index * 0.02, lambda: self.executor._scaling_callback(SimpleNamespace(data=value)))

    def test_cold_start_real_preflight_chain_waits_then_passes(self):
        path = Path(__file__).resolve().parent / "fixtures" / "scaling_startup_repro.py"
        spec = importlib.util.spec_from_file_location("readiness_actual_preflight_replay", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.run_case()
        self.assertEqual(result["result"], "passed_scaling_guard")
        self.assertGreaterEqual(result["stable_age_s"], 0.25)
        self.assertEqual(result["goal_requests_sent"], 0)

    def test_missing_scaled_and_invalid_streams_timeout_before_reviewed_start(self):
        for value, reason in ((None, "missing"), (20.0, "scaled"), (float("nan"), "invalid")):
            with self.subTest(reason=reason):
                self.clock.now, self.clock.events = 1000.0, []
                self.executor.latest_scaling = None
                self.executor.latest_scaling_stamp = 0.0
                self.executor.scaling_stable_since = 0.0
                self.executor._scaling_invalid = False
                if value is not None:
                    self.samples(value)
                started = self.clock.now
                with self.assertRaises(runner.ExecutionFailure) as caught:
                    self.execute()
                self.assertIn("Timed out waiting", str(caught.exception))
                self.assertEqual(caught.exception.diagnostics["scaling"]["reason"], reason)
                self.assertGreaterEqual(self.clock.now - started, 5.0)
                self.assertLess(self.clock.now - started, 5.03)

    def test_stale_stream_times_out_and_fresh_recovery_must_warm_up_again(self):
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.clock.sleep(3.0)
        with self.assertRaises(runner.ExecutionFailure) as caught:
            self.execute()
        self.assertEqual(caught.exception.diagnostics["scaling"]["reason"], "stale")
        self.samples(100.0)
        resumed = self.clock.now
        with self.assertRaises(StopAtReviewedStart):
            self.execute()
        self.assertGreaterEqual(self.clock.now - resumed, 0.25)

    def test_waiting_can_be_canceled_before_any_goal_or_start_check(self):
        self.clock.later(0.10, self.executor.cancel_requested.set)
        with self.assertRaisesRegex(runner.ExecutionFailure, "canceled while waiting"):
            self.execute()
        self.assertLess(self.clock.now, 1000.13)

    def test_one_good_frame_cannot_become_stable_just_by_waiting(self):
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.clock.sleep(0.5)
        with self.assertRaises(runner.ExecutionFailure) as caught:
            self.execute()
        self.assertEqual(caught.exception.diagnostics["scaling"]["reason"], "stale")

    def test_stability_duration_ends_at_last_observed_frame(self):
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.clock.sleep(0.1)
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.clock.sleep(0.2)
        snapshot = self.executor._scaling_snapshot()
        self.assertEqual(snapshot["reason"], "warming")
        self.assertAlmostEqual(snapshot["stable_age_s"], 0.1)

    def test_fresh_scaling_callback_overtaking_snapshot_is_not_stale(self):
        """At print start, a 7 ms newer UR frame must not look stale."""
        self.executor._coordination_active = True
        self.executor.latest_scaling = 100.0
        self.executor.latest_scaling_stamp = self.clock.now - 0.01
        self.executor.scaling_stable_since = self.clock.now - 45.0
        self.executor._scaling_invalid = False
        original_lock = self.executor._goal_lock

        class OvertakenLock:
            triggered = False

            def __enter__(inner):
                if not inner.triggered:
                    inner.triggered = True
                    self.clock.now += 0.007
                    self.executor._scaling_callback(SimpleNamespace(data=100.0))
                return original_lock.__enter__()

            def __exit__(inner, *args):
                return original_lock.__exit__(*args)

        lock = OvertakenLock()
        self.executor._goal_lock = lock
        self.executor.require_live_program()
        self.assertTrue(lock.triggered)
        snapshot = self.executor._scaling_snapshot()
        self.assertEqual(snapshot["reason"], "ready")
        self.assertGreaterEqual(snapshot["age_s"], 0.0)

    def test_diagnostics_do_not_report_future_joint_feedback_after_callback(self):
        """A fresh concurrent joint frame must not get a negative diagnostic age."""
        original_lock = self.executor._goal_lock

        class OvertakenLock:
            triggered = False

            def __enter__(inner):
                if not inner.triggered:
                    inner.triggered = True
                    self.clock.now += 0.007
                    self.executor.latest_joint_stamp = self.clock.now
                return original_lock.__enter__()

            def __exit__(inner, *args):
                return original_lock.__exit__(*args)

        self.executor._goal_lock = OvertakenLock()
        diagnostics = self.executor._diagnostic_snapshot()
        self.assertGreaterEqual(diagnostics["joint_feedback_age_s"], 0.0)

    def test_running_scaling_change_fails_immediately_with_frozen_values(self):
        self.samples(100.0)
        with self.assertRaises(StopAtReviewedStart):
            self.execute()
        self.executor._scaling_callback(SimpleNamespace(data=20.0))
        started = self.clock.now
        with self.assertRaises(runner.ExecutionFailure) as caught:
            self.executor.require_live_program()
        self.assertEqual(self.clock.now, started)
        snapshot = caught.exception.diagnostics["scaling"]
        self.assertEqual(snapshot["reason"], "scaled")
        self.assertEqual(snapshot["value_percent"], 20.0)
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.assertEqual(snapshot["value_percent"], 20.0)

    def test_external_control_drop_is_not_masked_by_derived_zero_scaling(self):
        """A dead External Control program is the cause; zero scaling is a symptom."""
        self.executor._coordination_active = True
        self.executor.program_running = False
        self.executor.program_state_stamp = self.clock.now
        self.executor._scaling_callback(SimpleNamespace(data=0.0))
        self.executor._abort_active_goals = mock.Mock(return_value=(False, "UR terminal pending"))

        with self.assertRaisesRegex(runner.ExecutionFailure, "External Control stopped"):
            self.executor.require_live_program()

        self.executor._abort_active_goals.assert_called_once_with()

    def test_old_controller_is_rejected_before_other_preflight_or_controller_switch(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=lambda **kwargs: True,
            call_async=lambda request: resolved(SimpleNamespace(values=[])),
        )
        self.executor._require_bridge_protocol = lambda: (_ for _ in ()).throw(StopAtReviewedStart())
        with self.assertRaisesRegex(runner.ExecutionFailure, "stop lifecycle"):
            self.executor.wait_for_services()
        self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "version_missing")

    def test_wrong_controller_version_is_rejected_but_exact_version_reaches_preflight(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: (_ for _ in ()).throw(StopAtReviewedStart())
        requested_names = []
        for value in ("old-controller", runner.CONTROLLER_STOP_PROTOCOL_VERSION):
            def query(request, value=value):
                requested_names.append(request.names)
                return resolved(SimpleNamespace(values=[SimpleNamespace(string_value=value)]))
            self.executor.controller_protocol_client = SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=query)
            if value == runner.CONTROLLER_STOP_PROTOCOL_VERSION:
                with self.assertRaises(StopAtReviewedStart):
                    self.executor.wait_for_services()
                self.assertTrue(self.executor.status["controller_stop_protocol"]["confirmed"])
            else:
                with self.assertRaisesRegex(runner.ExecutionFailure, "Rebuild.*restart"):
                    self.executor.wait_for_services()
                self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "version_mismatch")
                self.assertEqual(self.executor.status["controller_stop_protocol"]["reported"], value)
        self.assertEqual(requested_names, [["stop_lifecycle_guard_version"]] * 2)

    def test_late_controller_service_can_still_report_a_valid_version(self):
        """Service discovery must not consume the entire response deadline."""
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: (_ for _ in ()).throw(StopAtReviewedStart())
        requests = []

        def available_near_deadline(*, timeout_sec):
            self.clock.sleep(min(timeout_sec, max(0.0, 1002.9 - self.clock.now)))
            return self.clock.now >= 1002.9 - 1e-9

        def query(request):
            requests.append(request.names)
            future = Future()
            self.clock.later(0.2, lambda: future.set_result(SimpleNamespace(values=[
                SimpleNamespace(string_value=runner.CONTROLLER_STOP_PROTOCOL_VERSION)
            ])))
            return future

        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=available_near_deadline, call_async=query,
        )
        with self.assertRaises(StopAtReviewedStart):
            self.executor.wait_for_services()
        self.assertEqual(requests, [["stop_lifecycle_guard_version"]])
        self.assertTrue(self.executor.status["controller_stop_protocol"]["confirmed"])
        self.assertLess(self.clock.now, 1003.2)

    def test_unanswered_controller_version_query_retries_then_reports_timeout(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: self.fail("Unconfirmed version must stop preflight")
        requests = []
        pending = []
        removed = []

        def query(request):
            requests.append(request.names)
            future = Future()
            pending.append(future)
            return future

        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=lambda **kwargs: True, call_async=query,
            remove_pending_request=removed.append,
        )
        with self.assertRaisesRegex(runner.ExecutionFailure, "Parameter query timed out") as caught:
            self.executor.wait_for_services()
        self.assertNotIn("Reported version: None", str(caught.exception))
        self.assertEqual(len(requests), 2)
        self.assertEqual(removed, pending)
        self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "query_timeout")
        self.assertLess(self.clock.now, 1006.1)

    def test_controller_version_query_retries_one_lost_response(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: (_ for _ in ()).throw(StopAtReviewedStart())
        requests = []
        pending = []
        removed = []

        def query(request):
            requests.append(request.names)
            future = Future()
            pending.append(future)
            if len(requests) == 2:
                self.clock.later(0.1, lambda: future.set_result(SimpleNamespace(values=[
                    SimpleNamespace(string_value=runner.CONTROLLER_STOP_PROTOCOL_VERSION)
                ])))
            return future

        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=lambda **kwargs: True, call_async=query,
            remove_pending_request=removed.append,
        )
        with self.assertRaises(StopAtReviewedStart):
            self.executor.wait_for_services()
        self.assertEqual(requests, [["stop_lifecycle_guard_version"]] * 2)
        self.assertEqual(removed, pending[:1])
        self.assertTrue(self.executor.status["controller_stop_protocol"]["confirmed"])
        self.assertLess(self.clock.now, 1003.2)

    def test_malformed_controller_parameter_response_cannot_confirm_version(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: self.fail("Malformed version must stop preflight")
        for response in (
            None,
            SimpleNamespace(values=None),
            SimpleNamespace(values=[SimpleNamespace(string_value=123)]),
        ):
            with self.subTest(response=response):
                self.executor.controller_protocol_client = SimpleNamespace(
                    wait_for_service=lambda **kwargs: True,
                    call_async=lambda request, response=response: resolved(response),
                )
                with self.assertRaisesRegex(runner.ExecutionFailure, "contained no stop lifecycle version"):
                    self.executor.wait_for_services()
                self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "version_missing")
                self.assertFalse(self.executor.status["controller_stop_protocol"]["confirmed"])

    def test_controller_parameter_query_errors_do_not_impersonate_an_old_version(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor._require_bridge_protocol = lambda: self.fail("Unconfirmed version must stop preflight")
        queries = []

        def query(request):
            queries.append(request.names)
            raise RuntimeError("temporary parameter client failure")

        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=lambda **kwargs: True, call_async=query,
        )
        with self.assertRaisesRegex(runner.ExecutionFailure, "Parameter query failed") as caught:
            self.executor.wait_for_services()
        self.assertNotIn("Reported version: None", str(caught.exception))
        self.assertEqual(len(queries), 2)
        self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "query_failed")

    def test_controller_version_wait_can_be_canceled_without_switching(self):
        self.executor.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace())
        self.executor.controller_protocol_client = SimpleNamespace(
            wait_for_service=lambda **kwargs: True, call_async=lambda request: Future(),
        )
        self.executor._require_bridge_protocol = lambda: self.fail("Cancellation must prevent further preflight")
        self.clock.later(0.10, self.executor.cancel_requested.set)
        with self.assertRaisesRegex(runner.ExecutionFailure, "canceled.*stop lifecycle"):
            self.executor.wait_for_services()
        self.assertLess(self.clock.now, 1000.13)

    def test_missing_controller_service_has_a_bounded_readonly_timeout(self):
        def unavailable(**kwargs):
            self.clock.sleep(kwargs["timeout_sec"])
            return False
        self.executor.controller_protocol_client = SimpleNamespace(wait_for_service=unavailable)
        with self.assertRaisesRegex(runner.ExecutionFailure, "parameter service is unavailable"):
            self.executor.wait_for_services()
        self.assertEqual(self.executor.status["controller_stop_protocol"]["reason"], "service_unavailable")
        self.assertLess(self.clock.now, 1003.11)

    def test_root_failure_freezes_trigger_values_while_current_diagnostics_can_recover(self):
        self.samples(100.0)
        with self.assertRaises(StopAtReviewedStart):
            self.execute()
        self.executor._scaling_callback(SimpleNamespace(data=20.0))
        with self.assertRaises(runner.ExecutionFailure) as caught:
            self.executor.require_live_program()
        self.executor._scaling_callback(SimpleNamespace(data=100.0))
        self.executor._record_root_error(caught.exception)
        first = self.executor.status["root_error"]
        self.executor._record_root_error(RuntimeError("later cleanup error"))
        self.assertIs(self.executor.status["root_error"], first)
        self.assertEqual(first["diagnostics"]["scaling"]["value_percent"], 20.0)
        self.assertEqual(self.executor._diagnostic_snapshot()["scaling"]["value_percent"], 100.0)

    def test_stop_status_replaces_current_reasons_and_never_accumulates_them(self):
        self.executor._record_root_error(RuntimeError("original execution failure"))
        self.executor.status["stop"] = {"confirmed": False, "reasons": ["Klipper pending", "UR pending"]}
        self.executor._publish_failure_state("stop-unconfirmed")
        self.executor.status["stop"] = {"confirmed": False, "reasons": ["UR pending"]}
        for _ in range(4):
            self.executor._publish_failure_state("stop-unconfirmed")
            self.assertEqual(self.executor.status["message"], "original execution failure; UR pending")
        self.executor.status["stop"] = {"confirmed": True, "reasons": []}
        self.executor._publish_failure_state("failed")
        self.assertEqual(self.executor.status["message"], "original execution failure")

    def test_status_write_failure_does_not_replace_the_original_error(self):
        self.executor._record_root_error(RuntimeError("original execution failure"))
        self.executor.status["stop"] = {"confirmed": False, "reasons": ["UR pending"]}
        self.executor._write_status = lambda: (_ for _ in ()).throw(OSError("disk unavailable"))
        with mock.patch("builtins.print") as printed:
            self.executor._publish_failure_state("stop-unconfirmed")
        self.assertEqual(self.executor.status["state"], "stop-unconfirmed")
        self.assertEqual(self.executor.status["root_error"]["message"], "original execution failure")
        self.assertIn("original execution failure", printed.call_args.args[0])


class StopStatusTests(unittest.TestCase):
    def test_recovered_klipper_is_removed_from_current_stop_reasons_without_unlocking_ur(self):
        path = Path(__file__).resolve().parent / "fixtures" / "stop_unknown_close_repro.py"
        spec = importlib.util.spec_from_file_location("structured_stop_actual_close_replay", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.run_case("ur_result_and_joint_stream_lost")
        status = result["status"]
        self.assertEqual(status["state"], "stop-unconfirmed")
        self.assertFalse(result["close_returned"])
        self.assertNotIn("terminal stop", status["root_error"]["message"])
        self.assertTrue(any("UR goal" in reason for reason in status["stop"]["reasons"]))
        self.assertFalse(any("Klipper goal" in reason for reason in status["stop"]["reasons"]))
        self.assertEqual(status["message"].count("UR goal terminal"), 1)
        self.assertNotIn("Klipper goal terminal", status["message"])
        self.assertEqual(status["root_error"]["diagnostics"]["scaling"]["value_percent"], 0.0)
        self.assertEqual(status["diagnostics"]["scaling"]["reason"], "scaled")
        current_goals = {goal["label"]: goal for goal in status["stop"]["goals"]}
        self.assertTrue(current_goals["Klipper"]["terminal"])
        self.assertFalse(current_goals["UR"]["terminal"])


if __name__ == "__main__":
    unittest.main()
