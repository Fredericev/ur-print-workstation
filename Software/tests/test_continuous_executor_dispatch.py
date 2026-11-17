"""Hardware-free proof that a cold job crosses each action boundary once."""

from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from Software.tests.test_full_scope_runner import prepare, runner


class ContinuousExecutorDispatchTests(unittest.TestCase):
    def _executor(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = prepare(Path(directory), "full")

        executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        executor._initialize_goal_tracking()
        executor.manifest = manifest
        executor.status = {}
        executor.cancel_requested = threading.Event()
        executor.ManualStepperProgram = NS(
            Goal=lambda: NS(),
            Feedback=NS(PHASE_DISPATCHED=2),
        )
        executor.node = NS(
            get_clock=lambda: NS(now=lambda: NS(nanoseconds=time.time_ns()))
        )
        executor._wait_for_coordination_scaling = lambda: None
        executor._require_coordination_scaling = lambda: None
        executor.require_start_match = lambda *_args: None
        validated = []
        executor._validate_smooth_motion = lambda profile: validated.append(profile)
        executor._check_smooth_dispatch = lambda _goal: None
        executor._write_status = lambda: None
        executor._set_status = lambda state, message, **updates: executor.status.update(
            state=state, message=message, **updates
        )
        executor._trajectory_goal = lambda profile, duration: NS(
            kind="ur", profile=profile, duration=duration
        )

        sent = []

        def send(_client, goal, label, **kwargs):
            sent.append((label, goal))
            if label == "Klipper":
                kwargs["feedback_callback"](
                    NS(feedback=NS(
                        phase=executor.ManualStepperProgram.Feedback.PHASE_DISPATCHED,
                        program_epoch_ros_ns=time.time_ns(),
                    ))
                )
            return object(), NS(done=lambda: False)

        executor.klipper_client = object()
        executor.ur_client = object()
        executor._send = send
        awaited = []
        executor._await_results = lambda entries, timeout_s, **kwargs: awaited.append(
            (list(entries), timeout_s, kwargs)
        )

        return executor, manifest, sent, awaited, validated

    def test_full_job_sends_one_program_goal_and_one_ur_goal(self):
        executor, manifest, sent, awaited, validated = self._executor()
        executor._trajectory_socket_callback(NS(data=True))

        first = manifest["trajectory_profiles"]["print"]["points"][0]["positions"]
        executor.execute_print([{"positions": first}])

        self.assertEqual([label for label, _goal in sent], ["Klipper", "UR"])
        self.assertEqual(len(awaited), 1)
        self.assertEqual(len(awaited[0][0]), 2)
        self.assertEqual(validated, [manifest["trajectory_profiles"]["print"]])
        program = sent[0][1]
        self.assertEqual(list(program.segment_units), [
            float(block["klipper_units"]) for block in manifest["blocks"]
        ])
        self.assertEqual(list(program.segment_durations_s), [
            float(block["duration_s"]) for block in manifest["blocks"]
        ])
        self.assertEqual(program.start_delay_s, runner.SYNC_START_LEAD_S)
        self.assertTrue(program.disable_after)

    def test_missing_disconnected_or_stale_trajectory_socket_never_queues_klipper(self):
        for condition in ("missing", "disconnected", "stale"):
            with self.subTest(condition=condition):
                executor, manifest, sent, _awaited, _validated = self._executor()
                if condition != "missing":
                    executor._trajectory_socket_callback(NS(data=condition == "stale"))
                if condition == "stale":
                    executor._trajectory_socket_state = (
                        True, executor._trajectory_socket_state[1] - 3.0
                    )
                first = manifest["trajectory_profiles"]["print"]["points"][0]["positions"]
                with self.assertRaisesRegex(runner.ExecutionFailure, "trajectory socket"):
                    executor.execute_print([{"positions": first}])
                self.assertEqual(sent, [], "No Klipper or UR goal may be queued")


if __name__ == "__main__":
    unittest.main()
