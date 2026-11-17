"""Fail-closed recovery and process-lifecycle ownership for the workstation.

The browser and native launchers deliberately see only three operations:
inspect, recover, and shutdown.  Device proof, project-runtime containment,
and evidence archiving stay together here so no caller can accidentally
"clear" a safety interlock by editing one status field.
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


EXECUTION_ID_RE = re.compile(r"^[a-f0-9]{16}$")
STATUS_FILE_RE = re.compile(
    r"^(?P<execution_id>[a-f0-9]{16})(?: \d+)?\.status(?: \d+)?\.json$"
)
MAX_STATUS_BYTES = 256_000


class LifecycleBlocked(RuntimeError):
    """A recovery/shutdown operation that must fail without clearing state."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = details or {}


class RuntimeLifecycle:
    """Own the stop-unconfirmed reconciliation and local runtime shutdown."""

    def __init__(
        self,
        *,
        runtime_dir: Path,
        stop_owned_jobs: Callable[[str], dict[str, Any]],
        stop_owned_runtime: Callable[[str], dict[str, Any]],
        owned_runtime_stopped: Callable[[], bool],
        stop_and_verify_ur: Callable[[], dict[str, Any]],
        verify_klipper_stopped: Callable[[], dict[str, Any]],
        invalidate_executions: Callable[[set[str]], None] | None = None,
    ) -> None:
        self.runtime_dir = Path(runtime_dir)
        self._stop_owned_jobs = stop_owned_jobs
        self._stop_owned_runtime = stop_owned_runtime
        self._owned_runtime_stopped = owned_runtime_stopped
        self._stop_and_verify_ur = stop_and_verify_ur
        self._verify_klipper_stopped = verify_klipper_stopped
        self._invalidate_executions = invalidate_executions or (lambda _ids: None)
        self._gate = threading.Lock()

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _read_unresolved(self) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        if not self.runtime_dir.is_dir():
            return records
        for path in sorted(self.runtime_dir.glob("*.json")):
            name_match = STATUS_FILE_RE.fullmatch(path.name)
            if name_match is None:
                continue
            try:
                raw = path.read_bytes()
                if len(raw) > MAX_STATUS_BYTES:
                    raise ValueError("status file is too large")
                payload = json.loads(raw.decode("utf-8"))
            except (OSError, UnicodeDecodeError, ValueError) as exc:
                raise LifecycleBlocked(
                    "stop_record_unverifiable",
                    "A previous execution status cannot be verified. The motion lock was kept.",
                    details={"path": str(path)},
                ) from exc
            if not isinstance(payload, dict) or payload.get("state") != "stop-unconfirmed":
                continue
            execution_id = str(payload.get("execution_id", ""))
            if (
                not EXECUTION_ID_RE.fullmatch(execution_id)
                or execution_id != name_match.group("execution_id")
            ):
                raise LifecycleBlocked(
                    "stop_record_unverifiable",
                    "A previous stop record has an invalid or mismatched execution id. "
                    "The motion lock was kept.",
                    details={"path": str(path)},
                )
            records.append(
                {
                    "path": path,
                    "raw": raw,
                    "sha256": hashlib.sha256(raw).hexdigest(),
                    "payload": payload,
                    "execution_id": execution_id,
                }
            )
        return records

    @staticmethod
    def _fingerprint(records: list[dict[str, Any]]) -> list[tuple[str, str]]:
        return sorted((str(item["path"]), item["sha256"]) for item in records)

    def inspect(self) -> dict[str, Any]:
        try:
            records = self._read_unresolved()
            execution_ids = sorted({item["execution_id"] for item in records})
            return {
                "locked": bool(records),
                "verifiable": True,
                "record_count": len(records),
                "execution_ids": execution_ids,
                "recovery_available": bool(records),
                "code": None,
                "message": (
                    "A previous UR/Klipper stop is unconfirmed. A fresh stop check can be requested."
                    if records
                    else "No unconfirmed stop is blocking motion."
                ),
            }
        except LifecycleBlocked as exc:
            return {
                "locked": True,
                "verifiable": False,
                "record_count": None,
                "execution_ids": [],
                "recovery_available": False,
                "code": exc.code,
                "message": exc.message,
                "details": exc.details,
            }

    def recover(self, *, operator_action: str) -> dict[str, Any]:
        if operator_action != "verify-stop-and-recover":
            raise LifecycleBlocked(
                "operator_action_required",
                "Use the Verify stop and recover button to request this operation.",
            )
        if not self._gate.acquire(blocking=False):
            raise LifecycleBlocked(
                "lifecycle_busy",
                "A stop recovery or shutdown is already in progress.",
            )
        try:
            initial = self._read_unresolved()
            if not initial:
                return {
                    "outcome": "no-lock",
                    "archived_records": 0,
                    "requires_fresh_moveit_review": False,
                    "print_interlock": self.inspect(),
                }
            try:
                jobs = self._stop_owned_jobs("stop-recovery")
                runtime = self._stop_owned_runtime("stop-recovery")
            except Exception as exc:
                raise LifecycleBlocked(
                    "runtime_stop_failed",
                    f"The project runtime could not be stopped: {exc}. The motion lock was kept.",
                ) from exc
            if not self._owned_runtime_stopped():
                raise LifecycleBlocked(
                    "runtime_stop_incomplete",
                    "A project-owned ROS or execution process is still running. The motion lock was kept.",
                )

            # Snapshot only after the old executor is contained.  It may have
            # legitimately refreshed its stop observation while being stopped.
            snapshot = self._read_unresolved()
            if not snapshot:
                raise LifecycleBlocked(
                    "incident_changed",
                    "The stop record changed while recovery was starting. Retry from the refreshed screen.",
                )
            try:
                ur = self._stop_and_verify_ur()
            except Exception as exc:
                raise LifecycleBlocked(
                    "ur_proof_unavailable",
                    f"The UR stopped state could not be verified: {exc}. The motion lock was kept.",
                ) from exc
            if ur.get("stopped") is not True:
                raise LifecycleBlocked(
                    "ur_not_stopped",
                    "The UR program is not confirmed STOPPED. Stop it on the pendant, then retry.",
                    details={"ur": ur},
                )
            if ur.get("stationary") is not True:
                raise LifecycleBlocked(
                    "ur_stationary_unconfirmed",
                    "Fresh UR joint feedback did not prove a continuously stationary arm. "
                    "The motion lock was kept; check the UR connection and retry.",
                    details={"ur": ur},
                )

            try:
                klipper = self._verify_klipper_stopped()
            except Exception as exc:
                raise LifecycleBlocked(
                    "klipper_proof_unavailable",
                    f"The Klipper stopped state could not be verified: {exc}. The motion lock was kept.",
                ) from exc
            if klipper.get("stopped") is not True:
                raise LifecycleBlocked(
                    "klipper_not_stopped",
                    "Klipper still reports an active or paused print. Stop it, then retry.",
                    details={"klipper": klipper},
                )

            current = self._read_unresolved()
            if self._fingerprint(current) != self._fingerprint(snapshot):
                raise LifecycleBlocked(
                    "incident_changed",
                    "The stop record changed during verification. The motion lock was kept; retry once it is stable.",
                )

            archive_path = self._archive_recovery(
                current,
                jobs=jobs,
                runtime=runtime,
                ur=ur,
                klipper=klipper,
            )
            return {
                "outcome": "recovered",
                "archived_records": len(current),
                "archive_path": str(archive_path),
                "requires_fresh_moveit_review": True,
                "ros_remains_stopped": True,
                "print_interlock": self.inspect(),
            }
        finally:
            self._gate.release()

    def _archive_recovery(
        self,
        records: list[dict[str, Any]],
        *,
        jobs: dict[str, Any],
        runtime: dict[str, Any],
        ur: dict[str, Any],
        klipper: dict[str, Any],
    ) -> Path:
        recovery_id = f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{uuid.uuid4().hex[:8]}"
        archive_root = self.runtime_dir / "archive"
        archive_root.mkdir(parents=True, exist_ok=True)
        temporary = archive_root / f".{recovery_id}.tmp"
        destination = archive_root / recovery_id
        records_dir = temporary / "records"
        artifacts_dir = temporary / "artifacts"
        records_dir.mkdir(parents=True)
        artifacts_dir.mkdir(parents=True)

        execution_ids = {item["execution_id"] for item in records}
        receipt_records: list[dict[str, Any]] = []
        manifests: list[tuple[Path, bytes, str]] = []
        try:
            for item in records:
                target = records_dir / item["path"].name
                target.write_bytes(item["raw"])
                receipt_records.append(
                    {
                        "original_path": str(item["path"]),
                        "archive_name": target.name,
                        "execution_id": item["execution_id"],
                        "sha256": item["sha256"],
                    }
                )
            for execution_id in sorted(execution_ids):
                manifest = self.runtime_dir / f"{execution_id}.json"
                if not manifest.is_file():
                    continue
                raw = manifest.read_bytes()
                digest = hashlib.sha256(raw).hexdigest()
                (artifacts_dir / manifest.name).write_bytes(raw)
                manifests.append((manifest, raw, digest))

            receipt = {
                "schema": 1,
                "recovery_id": recovery_id,
                "committed_at": self._now(),
                "meaning": "hardware-stop-reconciled",
                "records": receipt_records,
                "revoked_execution_ids": sorted(execution_ids),
                "owned_job_cleanup": jobs,
                "owned_runtime_cleanup": runtime,
                "live_proof": {"ur": ur, "klipper": klipper},
            }
            (temporary / "recovery-receipt.json").write_text(
                json.dumps(receipt, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary.replace(destination)

            # Revoke the old package before removing the last blocking status.
            self._invalidate_executions(execution_ids)
            for path, raw, digest in manifests:
                if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                    raise LifecycleBlocked(
                        "incident_changed",
                        "An execution manifest changed during recovery. The motion lock was kept.",
                    )
                path.unlink()
            for item in records:
                path = item["path"]
                if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                    raise LifecycleBlocked(
                        "incident_changed",
                        "A stop record changed during archive commit. The motion lock was kept.",
                    )
                path.unlink()
            return destination
        except LifecycleBlocked:
            raise
        except Exception as exc:
            if temporary.exists():
                shutil.rmtree(temporary, ignore_errors=True)
            raise LifecycleBlocked(
                "evidence_archive_failed",
                f"Recovery evidence could not be archived: {exc}. The motion lock was kept.",
            ) from exc

    def shutdown(self, *, reason: str) -> dict[str, Any]:
        if not self._gate.acquire(blocking=False):
            raise LifecycleBlocked(
                "lifecycle_busy",
                "A stop recovery or shutdown is already in progress.",
            )
        try:
            try:
                jobs = self._stop_owned_jobs(reason)
                runtime = self._stop_owned_runtime(reason)
            except Exception as exc:
                raise LifecycleBlocked(
                    "shutdown_incomplete",
                    f"Project-owned runtime cleanup failed: {exc}",
                ) from exc
            if not self._owned_runtime_stopped():
                raise LifecycleBlocked(
                    "shutdown_incomplete",
                    "Project-owned ROS or execution processes remain after shutdown.",
                )
            return {
                "outcome": "stopped",
                "reason": reason,
                "jobs": jobs,
                "runtime": runtime,
                "print_interlock_preserved": self.inspect()["locked"],
            }
        finally:
            self._gate.release()
