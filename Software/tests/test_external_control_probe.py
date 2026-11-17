from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "workspaces"
    / "ur_ros2_ws"
    / "scripts"
    / "external_control_readiness_probe.py"
)
SPEC = importlib.util.spec_from_file_location("external_control_readiness_probe", MODULE_PATH)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


class ReadinessStateTests(unittest.TestCase):
    def test_false_program_state_is_never_ready(self) -> None:
        state = probe.ReadinessState()
        state.update_program(False, 10.0)
        state.update_joint_state([f"joint_{index}" for index in range(6)], [0.0] * 6, 10.0)
        self.assertFalse(state.ready(11.0, 0.5, 1.5))
        self.assertIn("not running", state.reason(11.0, 1.5))

    def test_true_program_without_fresh_joints_is_not_ready(self) -> None:
        state = probe.ReadinessState()
        state.update_program(True, 10.0)
        self.assertFalse(state.ready(11.0, 0.5, 1.5))
        self.assertIn("No fresh", state.reason(11.0, 1.5))

    def test_five_joint_message_is_rejected(self) -> None:
        state = probe.ReadinessState()
        state.update_program(True, 10.0)
        state.update_joint_state([f"joint_{index}" for index in range(5)], [0.0] * 5, 10.5)
        self.assertFalse(state.ready(10.6, 0.5, 1.0))

    def test_stale_joint_message_is_rejected(self) -> None:
        state = probe.ReadinessState()
        state.update_program(True, 10.0)
        state.update_joint_state([f"joint_{index}" for index in range(6)], [0.0] * 6, 10.1)
        self.assertFalse(state.ready(12.0, 0.5, 1.0))

    def test_stable_true_program_and_fresh_six_joint_message_are_ready(self) -> None:
        state = probe.ReadinessState()
        state.update_program(True, 10.0)
        state.update_joint_state([f"joint_{index}" for index in range(6)], [0.0] * 6, 10.6)
        self.assertTrue(state.ready(10.6, 0.5, 1.0))

    def test_false_update_clears_previous_true_state(self) -> None:
        state = probe.ReadinessState()
        state.update_program(True, 10.0)
        state.update_joint_state([f"joint_{index}" for index in range(6)], [0.0] * 6, 10.6)
        self.assertTrue(state.ready(10.6, 0.5, 1.0))
        state.update_program(False, 10.7)
        self.assertFalse(state.ready(10.8, 0.5, 1.0))
        self.assertIsNone(state.program_true_since)


if __name__ == "__main__":
    unittest.main()
