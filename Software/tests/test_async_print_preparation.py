"""Regression coverage for non-blocking, idempotent execution preparation."""

from __future__ import annotations

import threading
import time
import json
from pathlib import Path
import tempfile
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest import mock

from Software.tests.test_control_station import station


class AsyncPreparationTests(unittest.TestCase):
    def backend_with_registry(self) -> station.StationBackend:
        backend = station.StationBackend.__new__(station.StationBackend)
        backend._print_execution_lock = threading.RLock()
        backend._print_packages = {}
        backend._print_preparation_lock = threading.RLock()
        backend._print_preparations = {}
        backend._print_preparation_ids_by_key = {}
        runtime = tempfile.TemporaryDirectory(prefix="async-preparation-")
        self.addCleanup(runtime.cleanup)
        backend._print_execution_runtime = Path(runtime.name)
        backend._config_lock = threading.RLock()
        backend.config = {
            "manual_stepper": "stepper_x",
            "stepper_speed": 5.0,
            "stepper_accel": 20.0,
        }
        backend._validate_print_preparation_request = mock.Mock(
            return_value=("1" * 64, {})
        )
        return backend

    def test_single_flight_returns_immediately_and_compiles_once(self) -> None:
        backend = self.backend_with_registry()
        release = threading.Event()
        entered = threading.Event()
        package = {
            "execution_id": "a" * 16,
            "source_sha256": "1" * 64,
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
        }

        def compile_package(*_args, **_kwargs):
            entered.set()
            self.assertTrue(release.wait(2))
            return package

        backend.prepare_print_execution = mock.Mock(side_effect=compile_package)
        started_at = time.monotonic()
        first = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        second = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        self.assertLess(time.monotonic() - started_at, 0.5)
        self.assertTrue(entered.wait(1))
        self.assertEqual(first["id"], second["id"])
        self.assertIn(first["state"], {"queued", "running"})
        self.assertEqual(backend.prepare_print_execution.call_count, 1)

        release.set()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            result = backend.get_print_preparation(first["id"])
            if result["state"] == "succeeded":
                break
            time.sleep(0.01)
        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(result["package"], package)
        self.assertNotIn("content", result)

    def test_public_preparation_exposes_bounded_progress_and_eta(self) -> None:
        record = {
            "id": "c" * 12,
            "state": "running",
            "created_at": "2026-09-21T12:00:00+00:00",
            "started_at": "2026-09-21T12:00:01+00:00",
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
            "source_sha256": "1" * 64,
            "phase": "certifying_smooth_curve",
            "progress": 0.625,
            "completed_units": 625,
            "total_units": 1000,
            "estimated_remaining_s": 42.5,
        }

        public = station.StationBackend._public_print_preparation(record)

        self.assertEqual(public["phase"], "certifying_smooth_curve")
        self.assertEqual(public["progress"], 0.625)
        self.assertEqual(public["completed_units"], 625)
        self.assertEqual(public["total_units"], 1000)
        self.assertEqual(public["estimated_remaining_s"], 42.5)

    def test_worker_publishes_monotonic_callback_progress_phase_and_eta(self) -> None:
        backend = self.backend_with_registry()
        first_reported = threading.Event()
        continue_after_first = threading.Event()
        regressive_reported = threading.Event()
        continue_after_regressive = threading.Event()
        final_reported = threading.Event()
        finish_compilation = threading.Event()
        package = {
            "execution_id": "a" * 16,
            "source_sha256": "1" * 64,
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
        }

        def compile_package(*_args, progress_callback=None, **_kwargs):
            if progress_callback is None:
                raise AssertionError("background preparation omitted progress_callback")
            progress_callback("mapping_timeline", 0.45, 45, 100)
            first_reported.set()
            if not continue_after_first.wait(2):
                raise RuntimeError("test did not release the first progress report")
            progress_callback("certifying_print_curve", 0.20, 20, 100)
            regressive_reported.set()
            if not continue_after_regressive.wait(2):
                raise RuntimeError("test did not release the regressive progress report")
            # ETA remains intentionally unknown for instantaneous startup
            # callbacks; after measurable work it becomes meaningful.
            time.sleep(1.01)
            progress_callback("certifying_approach_curve", 0.80, 80, 100)
            final_reported.set()
            if not finish_compilation.wait(2):
                raise RuntimeError("test did not release compilation")
            return package

        backend.prepare_print_execution = mock.Mock(side_effect=compile_package)
        started = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        self.assertTrue(first_reported.wait(1))
        first = backend.get_print_preparation(started["id"])
        self.assertEqual(first["state"], "running")
        self.assertEqual(first["phase"], "mapping_timeline")
        self.assertEqual(first["progress"], 0.45)
        self.assertEqual(first["completed_units"], 45)
        self.assertEqual(first["total_units"], 100)
        self.assertNotIn("estimated_remaining_s", first)

        continue_after_first.set()
        self.assertTrue(regressive_reported.wait(1))
        regressive = backend.get_print_preparation(started["id"])
        self.assertEqual(regressive["phase"], "certifying_print_curve")
        self.assertEqual(regressive["progress"], first["progress"])
        self.assertNotIn("estimated_remaining_s", regressive)

        continue_after_regressive.set()
        self.assertTrue(final_reported.wait(2))
        final_running = backend.get_print_preparation(started["id"])
        self.assertEqual(final_running["phase"], "certifying_approach_curve")
        self.assertEqual(final_running["progress"], 0.80)
        self.assertIn("estimated_remaining_s", final_running)
        self.assertGreater(final_running["estimated_remaining_s"], 0.0)
        observed_progress = [
            started["progress"],
            first["progress"],
            regressive["progress"],
            final_running["progress"],
        ]
        self.assertEqual(
            observed_progress,
            sorted(observed_progress),
        )

        finish_compilation.set()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            succeeded = backend.get_print_preparation(started["id"])
            if succeeded["state"] == "succeeded":
                break
            time.sleep(0.01)
        self.assertEqual(succeeded["state"], "succeeded")
        self.assertEqual(succeeded["phase"], "complete")
        self.assertEqual(succeeded["progress"], 1.0)
        self.assertEqual(succeeded["estimated_remaining_s"], 0.0)
        self.assertEqual(succeeded["package"], package)

    def test_failed_worker_never_publishes_success_progress_or_eta(self) -> None:
        backend = self.backend_with_registry()

        def fail_after_last_progress(*_args, progress_callback=None, **_kwargs):
            if progress_callback is None:
                raise AssertionError("background preparation omitted progress_callback")
            progress_callback("writing_package", 1.0, 100, 100)
            raise station.ApiError(422, "Package write failed.")

        backend.prepare_print_execution = mock.Mock(side_effect=fail_after_last_progress)
        started = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            failed = backend.get_print_preparation(started["id"])
            if failed["state"] == "failed":
                break
            time.sleep(0.01)

        self.assertEqual(failed["state"], "failed")
        self.assertEqual(failed["phase"], "failed")
        self.assertLess(failed["progress"], 1.0)
        self.assertNotIn("estimated_remaining_s", failed)
        self.assertNotIn("package", failed)
        self.assertEqual(failed["error"], "Package write failed.")

    def test_cached_exact_package_completes_without_compiling(self) -> None:
        backend = self.backend_with_registry()
        package = {
            "execution_id": "a" * 16,
            "source_sha256": "1" * 64,
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
        }
        backend._print_packages["a" * 16] = {
            "path": "/unused",
            "moveit_job_id": "b" * 12,
            "execution_config_sha256": backend._print_execution_config_sha256(
                backend.config
            ),
            "public": package,
        }
        backend.prepare_print_execution = mock.Mock()

        result = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )

        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(result["package"], package)
        backend.prepare_print_execution.assert_not_called()

    def test_failed_approach_cannot_reuse_cached_preparation_but_a_new_moveit_review_can(self) -> None:
        backend = self.backend_with_registry()
        package = {
            "execution_id": "a" * 16, "source_sha256": "1" * 64,
            "execution_extent": "full", "moveit_job_id": "b" * 12,
        }
        backend._print_packages[package["execution_id"]] = {
            "path": "/unused", "moveit_job_id": "b" * 12,
            "execution_config_sha256": backend._print_execution_config_sha256(backend.config),
            "public": package,
        }
        backend.prepare_print_execution = mock.Mock()
        first = backend.start_print_preparation("model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full")
        self.assertEqual(first["package"], package)
        status_path = backend._print_execution_runtime / f"{package['execution_id']}.status.json"
        evidence = json.dumps({"execution_id": package["execution_id"], "mode": "move-to-start",
                               "state": "failed", "root_error": {"message": "endpoint failure"}}).encode()
        status_path.write_bytes(evidence)
        for changed_config in (False, True):
            with self.subTest(changed_config=changed_config):
                if changed_config:
                    backend.config["stepper_speed"] = 6.0
                with self.assertRaises(station.ApiError) as raised:
                    backend.start_print_preparation("model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full")
                self.assertEqual(raised.exception.code, "requires_revalidation")
                self.assertEqual(status_path.read_bytes(), evidence)
        backend.prepare_print_execution.assert_not_called()
        fresh_package = {**package, "execution_id": "c" * 16, "moveit_job_id": "d" * 12}
        backend.prepare_print_execution = mock.Mock(return_value=fresh_package)
        fresh = backend.start_print_preparation("model.gcode", b"G1 X1\n", "d" * 12, execution_extent="full")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            fresh = backend.get_print_preparation(fresh["id"])
            if fresh["state"] == "succeeded":
                break
            time.sleep(0.01)
        self.assertEqual(fresh["package"], fresh_package)

    def test_failed_preparation_can_be_retried_with_a_new_id(self) -> None:
        backend = self.backend_with_registry()
        package = {
            "execution_id": "a" * 16,
            "source_sha256": "1" * 64,
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
        }
        backend.prepare_print_execution = mock.Mock(
            side_effect=[
                station.ApiError(503, "Temporary compiler failure."),
                package,
            ]
        )

        first = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            failed = backend.get_print_preparation(first["id"])
            if failed["state"] == "failed":
                break
            time.sleep(0.01)
        self.assertEqual(failed["state"], "failed")

        second = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        self.assertNotEqual(first["id"], second["id"])
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            succeeded = backend.get_print_preparation(second["id"])
            if succeeded["state"] == "succeeded":
                break
            time.sleep(0.01)
        self.assertEqual(succeeded["state"], "succeeded")
        self.assertEqual(backend.prepare_print_execution.call_count, 2)

    def test_changed_stepper_config_does_not_reuse_cached_package(self) -> None:
        backend = self.backend_with_registry()
        package = {
            "execution_id": "a" * 16,
            "source_sha256": "1" * 64,
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
        }
        backend._print_packages["a" * 16] = {
            "path": "/unused",
            "moveit_job_id": "b" * 12,
            "execution_config_sha256": backend._print_execution_config_sha256(
                backend.config
            ),
            "public": package,
        }
        backend.config["stepper_speed"] = 6.0
        backend.prepare_print_execution = mock.Mock(return_value=package)

        result = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            result = backend.get_print_preparation(result["id"])
            if result["state"] == "succeeded":
                break
            time.sleep(0.01)
        self.assertEqual(result["state"], "succeeded")
        backend.prepare_print_execution.assert_called_once()
        frozen_config = backend.prepare_print_execution.call_args.kwargs[
            "execution_config"
        ]
        self.assertEqual(frozen_config["stepper_speed"], 6.0)

    def test_worker_preserves_structured_api_failure(self) -> None:
        backend = self.backend_with_registry()
        backend.prepare_print_execution = mock.Mock(
            side_effect=station.ApiError(
                412, "Review was superseded.", code="execution_plan_revoked"
            )
        )
        started = backend.start_print_preparation(
            "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
        )
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            result = backend.get_print_preparation(started["id"])
            if result["state"] == "failed":
                break
            time.sleep(0.01)
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error"], "Review was superseded.")
        self.assertEqual(result["code"], "execution_plan_revoked")
        self.assertEqual(result["http_status"], 412)

    def test_active_preparation_blocks_new_planning_and_motion(self) -> None:
        backend = self.backend_with_registry()
        backend._print_preparations["c" * 12] = {
            "id": "c" * 12,
            "state": "running",
        }
        with self.assertRaises(station.ApiError) as blocked:
            backend._require_no_active_print_preparation()
        self.assertEqual(blocked.exception.status, 409)
        self.assertEqual(blocked.exception.code, "print_preparation_active")

        backend.prepare_print_execution = mock.Mock()
        with self.assertRaises(station.ApiError) as second_compile:
            backend.start_print_preparation(
                "other.gcode",
                b"G1 X2\n",
                "b" * 12,
                execution_extent="sample",
            )
        self.assertEqual(second_compile.exception.status, 409)
        self.assertEqual(second_compile.exception.code, "print_preparation_active")
        backend.prepare_print_execution.assert_not_called()

        backend._require_no_active_print_preparation = mock.Mock(
            side_effect=station.ApiError(
                409,
                "Preparation is active.",
                code="print_preparation_active",
            )
        )
        backend.jobs = mock.Mock()
        with self.assertRaises(station.ApiError):
            backend.start_gcode_moveit_review("model.gcode", b"G1 X1\n")
        backend.jobs.active.assert_not_called()
        with self.assertRaises(station.ApiError):
            backend.start_print_operation({}, mode="move-to-start")

    def test_async_transport_returns_202_and_exposes_poll_route(self) -> None:
        preparation = {
            "id": "c" * 12,
            "state": "running",
            "execution_extent": "full",
            "moveit_job_id": "b" * 12,
            "source_sha256": "1" * 64,
        }
        backend = SimpleNamespace(
            start_print_preparation=mock.Mock(return_value=preparation),
            get_print_preparation=mock.Mock(
                return_value={**preparation, "state": "succeeded", "package": {"execution_id": "a" * 16}}
            ),
        )
        server = ThreadingHTTPServer(("127.0.0.1", 0), station.make_handler(backend))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            request = urllib.request.Request(
                f"http://127.0.0.1:{server.server_port}/api/print/prepare",
                data=b"G1 X1\n",
                headers={
                    "Content-Type": "application/octet-stream",
                    "X-Gcode-Filename": "model.gcode",
                    "X-Moveit-Job-Id": "b" * 12,
                    "X-Execution-Extent": "full",
                    "Prefer": "respond-async",
                },
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=2) as response:
                self.assertEqual(response.status, 202)
                body = station.json.loads(response.read())
            self.assertEqual(body["preparation"], preparation)
            backend.start_print_preparation.assert_called_once_with(
                "model.gcode", b"G1 X1\n", "b" * 12, execution_extent="full"
            )

            with urllib.request.urlopen(
                f"http://127.0.0.1:{server.server_port}/api/print/prepare/{'c' * 12}",
                timeout=2,
            ) as response:
                self.assertEqual(response.status, 200)
                polled = station.json.loads(response.read())
            self.assertEqual(polled["preparation"]["state"], "succeeded")
            backend.get_print_preparation.assert_called_once_with("c" * 12)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
