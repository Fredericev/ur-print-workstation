from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from Software.runtime_lifecycle import LifecycleBlocked, RuntimeLifecycle


def stop_record(execution_id: str = "0123456789abcdef") -> dict:
    return {
        "execution_id": execution_id,
        "state": "stop-unconfirmed",
        "stop": {
            "confirmed": False,
            "klipper_emergency_required": True,
            "klipper_emergency_confirmed": True,
            "ur_stationary": {"required": True, "confirmed": True},
        },
    }


class RuntimeLifecycleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.runtime = Path(self.temporary.name)
        self.events: list[str] = []
        self.runtime_stopped = True
        self.ur = {
            "stopped": True,
            "program_state": "STOPPED",
            "stationary": True,
            "stationary_sample_count": 6,
        }
        self.klipper = {"stopped": True, "print_state": "standby"}

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_record(self, name: str, payload: dict | None = None) -> Path:
        path = self.runtime / name
        path.write_text(json.dumps(payload or stop_record()), encoding="utf-8")
        return path

    def lifecycle(self, *, during_ur_probe=None) -> RuntimeLifecycle:
        def stop_runtime(reason: str) -> dict:
            self.events.append(f"stop-runtime:{reason}")
            return {"stopped": True}

        def stop_jobs(reason: str) -> dict:
            self.events.append(f"stop-jobs:{reason}")
            return {"stopped": True}

        def verify_runtime() -> bool:
            self.events.append("verify-runtime")
            return self.runtime_stopped

        def probe_ur() -> dict:
            self.events.append("probe-ur")
            if during_ur_probe:
                during_ur_probe()
            return dict(self.ur)

        def probe_klipper() -> dict:
            self.events.append("probe-klipper")
            return dict(self.klipper)

        return RuntimeLifecycle(
            runtime_dir=self.runtime,
            stop_owned_jobs=stop_jobs,
            stop_owned_runtime=stop_runtime,
            owned_runtime_stopped=verify_runtime,
            stop_and_verify_ur=probe_ur,
            verify_klipper_stopped=probe_klipper,
        )

    def test_inspect_groups_every_blocking_copy(self) -> None:
        self.write_record("0123456789abcdef.status.json")
        self.write_record("0123456789abcdef 2.status.json")
        self.write_record("0123456789abcdef.status 3.json")

        status = self.lifecycle().inspect()

        self.assertTrue(status["locked"])
        self.assertEqual(status["record_count"], 3)
        self.assertEqual(status["execution_ids"], ["0123456789abcdef"])

    def test_corrupt_stop_record_still_disables_recovery(self) -> None:
        (self.runtime / "0123456789abcdef.status.json").write_text("{broken", encoding="utf-8")

        status = self.lifecycle().inspect()

        self.assertTrue(status["locked"])
        self.assertFalse(status["recovery_available"])
        self.assertEqual(status["code"], "stop_record_unverifiable")

    def test_recovery_stops_owned_runtime_before_live_proof_and_archives_all_copies(self) -> None:
        first = self.write_record("0123456789abcdef.status.json")
        second = self.write_record("0123456789abcdef 2.status.json")
        third = self.write_record("0123456789abcdef.status 3.json")
        manifest = self.runtime / "0123456789abcdef.json"
        manifest.write_text('{"execution_id":"0123456789abcdef"}', encoding="utf-8")

        result = self.lifecycle().recover(
            operator_action="verify-stop-and-recover"
        )

        self.assertEqual(result["outcome"], "recovered")
        self.assertEqual(result["archived_records"], 3)
        self.assertTrue(result["requires_fresh_moveit_review"])
        self.assertFalse(first.exists())
        self.assertFalse(second.exists())
        self.assertFalse(third.exists())
        self.assertFalse(manifest.exists())
        self.assertFalse(self.lifecycle().inspect()["locked"])
        self.assertLess(self.events.index("stop-runtime:stop-recovery"), self.events.index("probe-ur"))
        archive = Path(result["archive_path"])
        receipt = json.loads((archive / "recovery-receipt.json").read_text(encoding="utf-8"))
        self.assertEqual(len(receipt["records"]), 3)
        self.assertTrue(all(len(item["sha256"]) == 64 for item in receipt["records"]))

    def test_recovery_refuses_to_clear_if_ur_is_not_stopped(self) -> None:
        record = self.write_record("0123456789abcdef.status.json")
        self.ur = {"stopped": False, "program_state": "PLAYING"}

        with self.assertRaises(LifecycleBlocked) as raised:
            self.lifecycle().recover(operator_action="verify-stop-and-recover")

        self.assertEqual(raised.exception.code, "ur_not_stopped")
        self.assertTrue(record.exists())

    def test_recovery_refuses_to_clear_if_klipper_is_active(self) -> None:
        record = self.write_record("0123456789abcdef.status.json")
        self.klipper = {"stopped": False, "print_state": "printing"}

        with self.assertRaises(LifecycleBlocked) as raised:
            self.lifecycle().recover(operator_action="verify-stop-and-recover")

        self.assertEqual(raised.exception.code, "klipper_not_stopped")
        self.assertTrue(record.exists())

    def test_recovery_rechecks_incomplete_old_evidence_with_fresh_ur_proof(self) -> None:
        payload = stop_record()
        payload["stop"]["ur_stationary"]["confirmed"] = False
        record = self.write_record("0123456789abcdef.status.json", payload)

        result = self.lifecycle().recover(operator_action="verify-stop-and-recover")

        self.assertEqual(result["outcome"], "recovered")
        self.assertFalse(record.exists())
        self.assertIn("probe-ur", self.events)

    def test_recovery_keeps_lock_if_fresh_ur_stationary_proof_is_missing(self) -> None:
        payload = stop_record()
        payload["stop"]["ur_stationary"]["confirmed"] = False
        record = self.write_record("0123456789abcdef.status.json", payload)
        self.ur = {"stopped": True, "program_state": "STOPPED"}

        with self.assertRaises(LifecycleBlocked) as raised:
            self.lifecycle().recover(operator_action="verify-stop-and-recover")

        self.assertEqual(raised.exception.code, "ur_stationary_unconfirmed")
        self.assertTrue(record.exists())
        self.assertIn("probe-ur", self.events)

    def test_recovery_fails_closed_if_record_changes_during_live_probe(self) -> None:
        record = self.write_record("0123456789abcdef.status.json")

        def mutate() -> None:
            payload = stop_record()
            payload["observed_after_probe"] = True
            record.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaises(LifecycleBlocked) as raised:
            self.lifecycle(during_ur_probe=mutate).recover(
                operator_action="verify-stop-and-recover"
            )

        self.assertEqual(raised.exception.code, "incident_changed")
        self.assertTrue(record.exists())

    def test_shutdown_stops_owned_resources_but_never_clears_interlock(self) -> None:
        record = self.write_record("0123456789abcdef.status.json")

        result = self.lifecycle().shutdown(reason="application-exit")

        self.assertEqual(result["outcome"], "stopped")
        self.assertTrue(record.exists())
        self.assertTrue(self.lifecycle().inspect()["locked"])
        self.assertEqual(
            self.events,
            [
                "stop-jobs:application-exit",
                "stop-runtime:application-exit",
                "verify-runtime",
            ],
        )

    def test_recovery_is_idempotent_when_no_lock_exists(self) -> None:
        result = self.lifecycle().recover(
            operator_action="verify-stop-and-recover"
        )

        self.assertEqual(result["outcome"], "no-lock")
        self.assertEqual(self.events, [])


if __name__ == "__main__":
    unittest.main()
