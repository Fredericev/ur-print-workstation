"""Exercise Stop -> Start with real local processes, without ROS or hardware."""

from __future__ import annotations

import os
import signal
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

from Software import control_station as station


HELPER_SOURCE = r"""
import signal
import sys
import time
from pathlib import Path

cycle = Path(sys.argv[1])
def stop(_signal, _frame):
    (cycle / "helper-stopped").touch()
    raise SystemExit(0)
signal.signal(signal.SIGTERM, stop)
(cycle / "helper-ready").touch()
while True:
    time.sleep(0.01)
"""


SPAWN_SOURCE = r"""
import subprocess
import sys
import time
from pathlib import Path

helper, cycle, runtime = map(Path, sys.argv[1:])
process = subprocess.Popen(
    [sys.executable, str(helper), str(cycle)],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    start_new_session=True,
)
(cycle / "helper-pid").write_text(str(process.pid))
(runtime / "helper-pid").write_text(str(process.pid))
(runtime / "helper-cycle").write_text(str(cycle))
deadline = time.monotonic() + 5.0
while not (cycle / "helper-ready").exists():
    if process.poll() is not None or time.monotonic() >= deadline:
        raise SystemExit("Detached helper did not become ready")
    time.sleep(0.005)
"""


LAUNCHER_SOURCE = r"""#!/bin/bash
set -euo pipefail
cycle="$1"
runtime="$2"
python="$3"
spawner="$4"
helper="$5"
mode="$6"
completed=0
cleanup() {
    if [[ "$completed" == 1 ]]; then return; fi
    touch "$cycle/cleanup-started"
    while [[ ! -f "$cycle/release-cleanup" ]]; do sleep 0.005; done
    # Match the launcher risk: old cleanup acts on the currently named runtime.
    if [[ -f "$runtime/helper-pid" ]]; then
        kill -TERM "$(cat "$runtime/helper-pid")" 2>/dev/null || true
    fi
    touch "$cycle/cleanup-complete"
}
trap cleanup EXIT
trap 'exit 143' TERM INT
"$python" "$spawner" "$helper" "$cycle" "$runtime"
touch "$cycle/launcher-started"
if [[ "$mode" == starting ]]; then
    while [[ ! -f "$cycle/release-startup" ]]; do sleep 0.005; done
fi
touch "$cycle/launcher-ready"
echo "Local runtime ready"
if [[ "$mode" == success ]]; then completed=1; exit 0; fi
while true; do sleep 0.02; done
"""


STOP_SOURCE = r"""
import os
import signal
import sys
import time
from pathlib import Path

runtime = Path(sys.argv[1])
cycle = Path((runtime / "helper-cycle").read_text())
os.kill(int((runtime / "helper-pid").read_text()), signal.SIGTERM)
deadline = time.monotonic() + 5.0
while not (cycle / "helper-stopped").exists():
    if time.monotonic() >= deadline:
        raise SystemExit("Detached helper did not stop")
    time.sleep(0.005)
"""


@unittest.skipIf(os.name == "nt", "Exercises the POSIX launcher EXIT trap")
class RestartCycleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.runtime = self.root / "runtime"
        self.runtime.mkdir()
        self.manager = station.JobManager(
            job_log_dir=self.root / "jobs", cancel_dir=self.root / "cancel"
        )
        self.helper = self.write_script("helper.py", HELPER_SOURCE)
        self.spawner = self.write_script("spawn.py", SPAWN_SOURCE)
        self.launcher = self.write_script("launcher.sh", LAUNCHER_SOURCE)
        self.stopper = self.write_script("stop.py", STOP_SOURCE)
        self.cycles: list[Path] = []
        self.cancel_threads: list[threading.Thread] = []

    def tearDown(self) -> None:
        try:
            for cycle in self.cycles:
                (cycle / "release-cleanup").touch()
            self.manager.stop_all(reason="restart-regression-cleanup", close=True)
            for thread in self.cancel_threads:
                thread.join(timeout=5.0)
            for cycle in self.cycles:
                pid_file = cycle / "helper-pid"
                if pid_file.exists() and not (cycle / "helper-stopped").exists():
                    try:
                        os.kill(int(pid_file.read_text()), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    self.wait_for(
                        lambda: (cycle / "helper-stopped").exists(),
                        "Detached test helper did not finish cleanup",
                    )
            self.wait_for(
                lambda: all(job.process is None for job in self.manager._jobs.values()),
                "Job workers did not finish after test cleanup",
            )
        finally:
            self.temporary.cleanup()

    def write_script(self, name: str, source: str) -> Path:
        path = self.root / name
        path.write_text(source, encoding="utf-8")
        return path

    def wait_for(self, predicate, message: str) -> None:
        deadline = time.monotonic() + 5.0
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail(message)
            time.sleep(0.005)

    def start_cycle(self, *, mode: str = "running") -> tuple[station.Job, Path]:
        cycle = self.root / f"cycle-{len(self.cycles)}"
        cycle.mkdir()
        self.cycles.append(cycle)
        job = self.manager.start(
            "start-ros",
            [
                "/bin/bash", str(self.launcher), str(cycle), str(self.runtime),
                sys.executable, str(self.spawner), str(self.helper), mode,
            ],
        )
        started_marker = "launcher-started" if mode == "starting" else "launcher-ready"
        self.wait_for(
            lambda: (cycle / started_marker).exists(),
            f"Launcher did not start: {job.log_lines}",
        )
        self.assertTrue((cycle / "helper-ready").exists())
        self.assertFalse((cycle / "helper-stopped").exists())
        return job, cycle

    def test_twelve_start_stop_restart_cycles_wait_for_exit_cleanup(self) -> None:
        for index in range(12):
            with self.subTest(cycle=index):
                mode = "starting" if index % 2 == 0 else "running"
                job, cycle = self.start_cycle(mode=mode)
                if mode == "starting":
                    self.assertFalse((cycle / "launcher-ready").exists())
                errors: list[BaseException] = []
                cancel_finished = threading.Event()

                def cancel() -> None:
                    try:
                        self.manager.cancel(job.id)
                    except BaseException as exc:
                        errors.append(exc)
                    finally:
                        cancel_finished.set()

                thread = threading.Thread(target=cancel, daemon=True)
                self.cancel_threads.append(thread)
                thread.start()
                try:
                    self.wait_for(
                        lambda: (cycle / "cleanup-started").exists(),
                        f"Launcher EXIT cleanup did not begin: {job.log_lines}",
                    )
                    self.assertFalse(cancel_finished.is_set())
                    with self.assertRaises(station.ApiError) as raised:
                        self.manager.start("start-ros", [sys.executable, "-c", "pass"])
                    self.assertEqual(raised.exception.code, "workstation_maintenance")
                finally:
                    (cycle / "release-cleanup").touch()
                    thread.join(timeout=5.0)

                self.assertFalse(thread.is_alive(), "Stop did not finish")
                self.assertEqual(errors, [])
                self.assertEqual(job.state, "canceled")
                self.assertTrue((cycle / "cleanup-complete").exists())
                self.wait_for(
                    lambda: (cycle / "helper-stopped").exists(),
                    "The old detached helper remained alive after Stop",
                )
                self.wait_for(lambda: job.process is None, "Canceled worker did not finish")

        restarted, cycle = self.start_cycle(mode="success")
        self.wait_for(lambda: restarted.state == "succeeded", "Final restart failed")
        self.assertFalse((cycle / "helper-stopped").exists())

    def test_stop_and_restart_after_launcher_has_naturally_succeeded(self) -> None:
        for index in range(4):
            with self.subTest(cycle=index):
                job, cycle = self.start_cycle(mode="success")
                self.wait_for(lambda: job.state == "succeeded", "Launcher did not finish")
                self.assertFalse((cycle / "helper-stopped").exists())
                stop = self.manager.start(
                    "stop-ros-driver", [sys.executable, str(self.stopper), str(self.runtime)]
                )
                self.wait_for(
                    lambda: stop.state not in {"queued", "running"}, "Stop did not finish"
                )
                self.assertEqual(stop.state, "succeeded", stop.log_lines)
                self.assertTrue((cycle / "helper-stopped").exists())
                self.assertFalse((cycle / "cleanup-started").exists())


if __name__ == "__main__":
    unittest.main()
