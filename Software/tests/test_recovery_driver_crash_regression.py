"""Offline reproduction of recovery after a UR driver crash during printing.

The executor can durably record an unconfirmed stop because a crashed driver
cannot provide a terminal action result or fresh post-stop joint samples.  That
incident must still offer an operator-initiated *verification attempt*: the
button must not be disabled before live stop checks can run.  This test never
contacts a robot or a Klipper host.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from Software.runtime_lifecycle import RuntimeLifecycle


class DriverCrashRecoveryRegression(unittest.TestCase):
    def test_unconfirmed_joint_stop_does_not_disable_live_recovery_attempt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runtime = Path(temporary)
            execution_id = "0123456789abcdef"
            (runtime / f"{execution_id}.status.json").write_text(
                json.dumps(
                    {
                        "execution_id": execution_id,
                        "state": "stop-unconfirmed",
                        "message": (
                            "The live UR joint-state stream stopped during Print Execution; "
                            "UR goal terminal stop is unconfirmed; Fresh post-stop UR joint "
                            "feedback has not confirmed a continuously stationary arm"
                        ),
                        "stop": {
                            "confirmed": False,
                            "klipper_emergency_required": True,
                            "klipper_emergency_confirmed": True,
                            "ur_stationary": {
                                "required": True,
                                "confirmed": False,
                                "sample_count": 0,
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            lifecycle = RuntimeLifecycle(
                runtime_dir=runtime,
                stop_owned_jobs=lambda _reason: {"stopped": True},
                stop_owned_runtime=lambda _reason: {"stopped": True},
                owned_runtime_stopped=lambda: True,
                stop_and_verify_ur=lambda: {"stopped": False},
                verify_klipper_stopped=lambda: {"stopped": True},
            )

            status = lifecycle.inspect()

            self.assertTrue(status["locked"])
            self.assertTrue(
                status["recovery_available"],
                "The Safe Recovery button must remain enabled for a fresh live stop "
                "verification attempt; this must not itself clear the lock.",
            )
            self.assertTrue((runtime / f"{execution_id}.status.json").exists())


if __name__ == "__main__":
    unittest.main()
