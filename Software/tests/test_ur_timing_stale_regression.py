"""Hardware-free regression for a scaling callback racing print admission."""
from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest import mock

from Software.tests.test_executor_goal_lifecycle import Clock, runner


class InterleavingClock(Clock):
    """Pin the exact callback/snapshot ordering without threads or ROS."""

    def __init__(self):
        super().__init__()
        self.after_next_read = None

    def monotonic(self):
        observed = self.now
        callback = self.after_next_read
        if callback is not None:
            self.after_next_read = None
            callback()
        return observed


class UrTimingStaleRegressionTests(unittest.TestCase):
    def setUp(self):
        self.clock = InterleavingClock()
        patcher = mock.patch.object(runner, "time", self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.executor = runner.RosPrintExecutor.__new__(runner.RosPrintExecutor)
        self.executor._initialize_goal_tracking()
        self.executor._diagnostic_snapshot = lambda: {}
        # State after 45 seconds of continuous 100% scaling feedback.
        self.executor.latest_scaling = 100.0
        self.executor.latest_scaling_stamp = 999.99
        self.executor.scaling_stable_since = 954.374
        self.executor._scaling_invalid = False

    def test_fresh_callback_between_clock_read_and_snapshot_lock_is_not_stale(self):
        self.clock.now = 1000.0

        def receive_live_frame():
            self.clock.now = 1000.007
            self.executor._scaling_callback(SimpleNamespace(data=100.0))

        self.clock.after_next_read = receive_live_frame
        # Real print-admission guard: the sample is fresh, 100%, and was
        # continuously stable for 45.633 s. A negative age is only a race.
        self.executor._require_coordination_scaling()

    def test_genuinely_old_sample_still_fails_closed(self):
        self.clock.now = 1002.1
        with self.assertRaises(runner.ExecutionFailure) as caught:
            self.executor._require_coordination_scaling()
        self.assertEqual(caught.exception.diagnostics["scaling"]["reason"], "stale")


if __name__ == "__main__":
    unittest.main()
