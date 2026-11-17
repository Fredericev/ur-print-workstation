"""Regression coverage for non-blocking, source-bound 3D review builds."""

from __future__ import annotations

import json
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest import mock

from Software.tests.test_control_station import station


class AsyncGcodeReviewTests(unittest.TestCase):
    @staticmethod
    def wait_for_terminal(
        backend: station.StationBackend,
        job_id: str,
        *,
        timeout: float = 2.0,
    ) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        result: dict[str, object] = {}
        while time.monotonic() < deadline:
            result = backend.get_gcode_review_job(job_id)
            if result["state"] in {"succeeded", "failed"}:
                return result
            time.sleep(0.01)
        raise AssertionError(f"review job {job_id} did not finish: {result}")

    def test_single_flight_returns_immediately_and_builds_once(self) -> None:
        backend = station.StationBackend(demo=True)
        entered = threading.Event()
        release = threading.Event()
        review = {"source_sha256": "a" * 64, "path": {"segments": []}}

        def slow_build(*_args: object, **_kwargs: object) -> dict[str, object]:
            entered.set()
            self.assertTrue(release.wait(2.0))
            return review

        with mock.patch.object(
            backend,
            "build_gcode_path_review",
            side_effect=slow_build,
        ) as build:
            started_at = time.monotonic()
            first = backend.start_gcode_review_job(
                "model.gcode", b"G1 X1\n", simulation_only=False
            )
            second = backend.start_gcode_review_job(
                "model.gcode", b"G1 X1\n", simulation_only=False
            )
            self.assertLess(time.monotonic() - started_at, 0.25)
            self.assertEqual(first["id"], second["id"])
            self.assertIn(first["state"], {"queued", "running"})
            self.assertTrue(entered.wait(1.0))
            self.assertEqual(build.call_count, 1)

            release.set()
            completed = self.wait_for_terminal(backend, first["id"])
            cached = backend.start_gcode_review_job(
                "model.gcode", b"G1 X1\n", simulation_only=False
            )

        self.assertEqual(completed["state"], "succeeded")
        self.assertEqual(completed["progress"], 100)
        self.assertEqual(completed["review"], review)
        self.assertEqual(cached["id"], first["id"])
        self.assertEqual(build.call_count, 1)

    def test_source_profile_and_mode_are_part_of_deduplication_identity(self) -> None:
        backend = station.StationBackend(demo=True)
        backend.build_gcode_path_review = mock.Mock(return_value={"kind": "installed"})
        backend.build_gcode_simulation_review = mock.Mock(return_value={"kind": "simulation"})

        first = backend.start_gcode_review_job(
            "model.gcode", b"G1 X1\n", simulation_only=False
        )
        changed_source = backend.start_gcode_review_job(
            "model.gcode", b"G1 X2\n", simulation_only=False
        )
        changed_mode = backend.start_gcode_review_job(
            "model.gcode", b"G1 X1\n", simulation_only=True
        )

        self.assertEqual(len({first["id"], changed_source["id"], changed_mode["id"]}), 3)
        for job in (first, changed_source, changed_mode):
            self.wait_for_terminal(backend, job["id"])

        profile = station.default_calibration()
        backend._demo_calibration = {
            "saved_at": station.utc_now(),
            "source_sha256": "f" * 64,
            "profile_sha256": "e" * 64,
            "calibration": profile,
        }
        changed_profile = backend.start_gcode_review_job(
            "model.gcode", b"G1 X1\n", simulation_only=False
        )
        self.assertNotEqual(first["id"], changed_profile["id"])
        self.wait_for_terminal(backend, changed_profile["id"])

    def test_failure_is_terminal_and_preserves_api_status(self) -> None:
        backend = station.StationBackend(demo=True)
        backend.build_gcode_path_review = mock.Mock(
            side_effect=station.ApiError(
                412,
                "The path is outside the table.",
                code="path_outside_table",
            )
        )

        started = backend.start_gcode_review_job(
            "bad.gcode", b"G1 X999\n", simulation_only=False
        )
        failed = self.wait_for_terminal(backend, started["id"])

        self.assertEqual(failed["state"], "failed")
        self.assertEqual(failed["phase"], "failed")
        self.assertEqual(failed["progress"], 100)
        self.assertEqual(failed["http_status"], 412)
        self.assertEqual(failed["code"], "path_outside_table")
        self.assertEqual(failed["error"], "The path is outside the table.")
        self.assertNotIn("review", failed)

        retried = backend.start_gcode_review_job(
            "bad.gcode", b"G1 X999\n", simulation_only=False
        )
        self.assertNotEqual(started["id"], retried["id"])
        self.wait_for_terminal(backend, retried["id"])

    def test_profile_is_frozen_while_queued_and_new_profile_gets_new_job(self) -> None:
        backend = station.StationBackend(demo=True)
        original = station.default_calibration()
        changed = station.default_calibration()
        changed["coordinate_transform"]["rotation_z_deg"] = 12.0
        backend._demo_calibration = {
            "saved_at": station.utc_now(),
            "source_sha256": "a" * 64,
            "profile_sha256": "b" * 64,
            "calibration": original,
        }
        backend.build_gcode_path_review = mock.Mock(return_value={"ok": True})

        backend._gcode_review_cpu.acquire()
        try:
            started = backend.start_gcode_review_job(
                "queued.gcode", b"G1 X1\n", simulation_only=False
            )
            with backend._calibration_lock:
                backend._demo_calibration = {
                    "saved_at": station.utc_now(),
                    "source_sha256": "c" * 64,
                    "profile_sha256": "d" * 64,
                    "calibration": changed,
                }
        finally:
            backend._gcode_review_cpu.release()

        completed = self.wait_for_terminal(backend, started["id"])
        self.assertEqual(completed["state"], "succeeded")
        frozen = backend.build_gcode_path_review.call_args.kwargs[
            "calibration_snapshot"
        ]
        self.assertEqual(
            frozen["coordinate_transform"]["rotation_z_deg"],
            original["coordinate_transform"]["rotation_z_deg"],
        )

        replacement = backend.start_gcode_review_job(
            "queued.gcode", b"G1 X1\n", simulation_only=False
        )
        self.assertNotEqual(started["id"], replacement["id"])
        self.wait_for_terminal(backend, replacement["id"])

    def test_background_work_and_retained_results_are_bounded(self) -> None:
        backend = station.StationBackend(demo=True)
        release = threading.Event()

        def blocked_build(*_args: object, **_kwargs: object) -> dict[str, object]:
            self.assertTrue(release.wait(2.0))
            return {"ok": True}

        backend.build_gcode_path_review = mock.Mock(side_effect=blocked_build)
        jobs = [
            backend.start_gcode_review_job(
                f"model-{index}.gcode",
                f"G1 X{index}\n".encode(),
                simulation_only=False,
            )
            for index in range(station.ASYNC_REVIEW_MAX_ACTIVE)
        ]
        with self.assertRaises(station.ApiError) as saturated:
            backend.start_gcode_review_job(
                "overflow.gcode", b"G1 X999\n", simulation_only=False
            )
        self.assertEqual(saturated.exception.status, 429)

        release.set()
        for job in jobs:
            self.wait_for_terminal(backend, job["id"])

        backend.build_gcode_path_review = mock.Mock(return_value={"ok": True})
        for index in range(station.ASYNC_REVIEW_MAX_RETAINED + 2):
            job = backend.start_gcode_review_job(
                f"retained-{index}.gcode",
                f"G1 Y{index}\n".encode(),
                simulation_only=False,
            )
            self.wait_for_terminal(backend, job["id"])
        with backend._gcode_review_lock:
            self.assertLessEqual(
                len(backend._gcode_review_jobs),
                station.ASYNC_REVIEW_MAX_RETAINED,
            )

    def test_async_transport_and_legacy_sync_transport(self) -> None:
        queued = {
            "id": "c" * 12,
            "state": "queued",
            "mode": "installed",
            "phase": "queued",
            "progress": 0,
            "status_url": f"/api/gcode/review/jobs/{'c' * 12}",
        }
        completed = {
            **queued,
            "state": "succeeded",
            "phase": "complete",
            "progress": 100,
            "review": {"source_sha256": "1" * 64},
        }
        backend = SimpleNamespace(
            start_gcode_review_job=mock.Mock(return_value=queued),
            get_gcode_review_job=mock.Mock(return_value=completed),
            build_gcode_path_review=mock.Mock(return_value=completed["review"]),
            build_gcode_simulation_review=mock.Mock(return_value={"simulation": True}),
        )
        server = ThreadingHTTPServer(("127.0.0.1", 0), station.make_handler(backend))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_port}"
            async_request = urllib.request.Request(
                f"{base_url}/api/gcode/review",
                data=b"G1 X1\n",
                headers={
                    "Content-Type": "application/octet-stream",
                    "X-Gcode-Filename": "model.gcode",
                    "Prefer": "respond-async",
                },
                method="POST",
            )
            with urllib.request.urlopen(async_request, timeout=1.0) as response:
                self.assertEqual(response.status, 202)
                payload = json.loads(response.read())
            self.assertEqual(payload["review_job"], queued)
            backend.start_gcode_review_job.assert_called_once_with(
                "model.gcode", b"G1 X1\n", simulation_only=False
            )

            simulation_request = urllib.request.Request(
                f"{base_url}/api/gcode/review/simulation",
                data=b"G1 X1\n",
                headers={
                    "Content-Type": "application/octet-stream",
                    "X-Gcode-Filename": "model.gcode",
                    "Prefer": "respond-async",
                },
                method="POST",
            )
            with urllib.request.urlopen(simulation_request, timeout=1.0) as response:
                self.assertEqual(response.status, 202)
            backend.start_gcode_review_job.assert_called_with(
                "model.gcode", b"G1 X1\n", simulation_only=True
            )

            with urllib.request.urlopen(
                f"{base_url}{queued['status_url']}", timeout=1.0
            ) as response:
                self.assertEqual(response.status, 200)
                status = json.loads(response.read())
            self.assertEqual(status["review_job"], completed)

            sync_request = urllib.request.Request(
                f"{base_url}/api/gcode/review",
                data=b"G1 X1\n",
                headers={
                    "Content-Type": "application/octet-stream",
                    "X-Gcode-Filename": "model.gcode",
                },
                method="POST",
            )
            with urllib.request.urlopen(sync_request, timeout=1.0) as response:
                self.assertEqual(response.status, 200)
                legacy = json.loads(response.read())
            self.assertEqual(legacy["review"], completed["review"])
            backend.build_gcode_path_review.assert_called_once_with(
                "model.gcode", b"G1 X1\n"
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
