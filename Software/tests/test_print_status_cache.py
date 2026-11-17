"""Status polling is observational; motion buttons still revalidate packages."""

from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

from Software.tests.test_control_station import station


class PrintStatusCacheTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = tempfile.TemporaryDirectory(prefix="print-status-cache-")
        self.addCleanup(self.runtime.cleanup)
        self.root = Path(self.runtime.name)
        self.execution_id = "a" * 16
        self.manifest_path = self.root / f"{self.execution_id}.json"
        self.manifest_path.write_text("prepared manifest", encoding="utf-8")
        self.status_path = self.root / f"{self.execution_id}.status.json"
        self.write_status("at-start")
        self.backend = station.StationBackend.__new__(station.StationBackend)
        self.backend.demo = False
        self.backend._print_execution_lock = threading.RLock()
        self.backend._print_execution_runtime = self.root
        self.backend._print_packages = {}
        self.backend._print_jobs = {}
        self.backend.jobs = mock.Mock()
        self.manifest = {"execution_id": self.execution_id}
        self.package = {
            "execution_id": self.execution_id,
            "source": {"name": "prepared.gcode"},
        }

    def write_status(self, state: str) -> None:
        self.status_path.write_text(json.dumps({
            "execution_id": self.execution_id,
            "state": state,
        }), encoding="utf-8")

    def cache_prepared_package(self) -> None:
        value = self.manifest_path.stat()
        self.backend._print_packages[self.execution_id] = {
            "path": self.manifest_path,
            "moveit_job_id": "review-job",
            "public": self.package,
            "status_file_identity": (
                value.st_dev, value.st_ino, value.st_size,
                value.st_mtime_ns, value.st_ctime_ns,
            ),
        }

    def test_prepared_package_poll_never_revalidates_and_reads_live_status(self) -> None:
        self.cache_prepared_package()
        with mock.patch.object(
            station, "read_execution_manifest", return_value=self.manifest
        ) as validate, mock.patch.object(
            self.backend, "_public_print_package", return_value=self.package
        ) as summarize:
            first = self.backend.get_print_execution_status(self.execution_id)
            self.write_status("completed")
            second = self.backend.get_print_execution_status(self.execution_id)
        validate.assert_not_called()
        summarize.assert_not_called()
        self.assertEqual(first["runtime"]["state"], "at-start")
        self.assertEqual(second["runtime"]["state"], "completed")
        self.assertEqual(second["package"], self.package)

    def test_restarted_backend_validates_only_first_poll(self) -> None:
        with mock.patch.object(
            station, "read_execution_manifest", return_value=self.manifest
        ) as validate, mock.patch.object(
            self.backend, "_public_print_package", return_value=self.package
        ) as summarize:
            for _ in range(3):
                self.backend.get_print_execution_status(self.execution_id)
        validate.assert_called_once_with(self.manifest_path)
        summarize.assert_called_once()

    def test_concurrent_cold_polls_share_one_validation(self) -> None:
        with mock.patch.object(
            station, "read_execution_manifest", return_value=self.manifest
        ) as validate, mock.patch.object(
            self.backend, "_public_print_package", return_value=self.package
        ), ThreadPoolExecutor(max_workers=4) as workers:
            responses = list(workers.map(
                self.backend.get_print_execution_status, [self.execution_id] * 4
            ))
        validate.assert_called_once_with(self.manifest_path)
        self.assertTrue(all(value["package"] == self.package for value in responses))

    def test_manifest_replaced_during_validation_is_not_cached(self) -> None:
        def load_and_replace(_path: Path) -> dict:
            replacement = self.root / "replacement.json"
            replacement.write_text("changed manifest", encoding="utf-8")
            replacement.replace(self.manifest_path)
            return self.manifest

        with mock.patch.object(
            station, "read_execution_manifest", side_effect=load_and_replace
        ) as validate, mock.patch.object(
            self.backend, "_public_print_package", return_value=self.package
        ):
            self.backend.get_print_execution_status(self.execution_id)
            self.backend.get_print_execution_status(self.execution_id)
        self.assertEqual(validate.call_count, 2)

    def test_manifest_replacement_invalidates_status_cache(self) -> None:
        self.cache_prepared_package()
        replacement = self.root / "replacement.json"
        replacement.write_text("changed manifest", encoding="utf-8")
        replacement.replace(self.manifest_path)
        with mock.patch.object(
            station, "read_execution_manifest",
            side_effect=station.PrintExecutionError("changed after preparation"),
        ) as validate:
            with self.assertRaisesRegex(station.ApiError, "changed after preparation"):
                self.backend.get_print_execution_status(self.execution_id)
        validate.assert_called_once_with(self.manifest_path)

    def test_cached_response_cannot_mutate_package_summary(self) -> None:
        self.cache_prepared_package()
        with mock.patch.object(station, "read_execution_manifest", return_value=self.manifest), \
                mock.patch.object(self.backend, "_public_print_package", return_value=self.package):
            first = self.backend.get_print_execution_status(self.execution_id)
            first["package"]["source"]["name"] = "response-only.gcode"
            second = self.backend.get_print_execution_status(self.execution_id)
        self.assertEqual(second["package"]["source"]["name"], "prepared.gcode")

    def test_motion_buttons_revalidate_even_with_warm_status_cache(self) -> None:
        self.cache_prepared_package()
        for mode in ("move-to-start", "execute"):
            with self.subTest(mode=mode), mock.patch.object(
                station, "read_execution_manifest",
                side_effect=station.PrintExecutionError("artifact changed after review"),
            ) as validate:
                with self.assertRaisesRegex(station.ApiError, "artifact changed after review"):
                    self.backend.start_print_operation(
                        {"execution_id": self.execution_id}, mode=mode
                    )
                validate.assert_called_once_with(self.manifest_path)
        self.backend.jobs.start.assert_not_called()

    def test_failed_approach_poll_exposes_revalidation_without_changing_failure_or_stop_evidence(self) -> None:
        self.cache_prepared_package()
        evidence = json.dumps({
            "execution_id": self.execution_id, "mode": "move-to-start", "state": "failed",
            "root_error": {"message": "endpoint tolerance"}, "stop": {"confirmed": True},
        }).encode()
        self.status_path.write_bytes(evidence)
        response = self.backend.get_print_execution_status(self.execution_id)
        self.assertTrue(response["requires_revalidation"])
        self.assertEqual(response["code"], "requires_revalidation")
        self.assertEqual(response["runtime"]["root_error"]["message"], "endpoint tolerance")
        self.assertEqual(response["runtime"]["stop"], {"confirmed": True})
        self.assertEqual(self.status_path.read_bytes(), evidence)


if __name__ == "__main__":
    unittest.main()
