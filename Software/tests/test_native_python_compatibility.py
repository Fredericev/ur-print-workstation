from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path


SOFTWARE_DIR = Path(__file__).resolve().parents[1]
NATIVE_PYTHON = Path(
    os.environ.get("UR_PRINT_PYTHON", "/usr/bin/python3")
)


class NativePythonCompatibilityTests(unittest.TestCase):
    def test_control_station_imports_with_native_app_python(self) -> None:
        """Exercise the same interpreter and import path as the macOS launcher."""

        if not NATIVE_PYTHON.is_file():
            self.skipTest(f"native app Python is unavailable: {NATIVE_PYTHON}")

        completed = subprocess.run(
            [str(NATIVE_PYTHON), "-c", "import control_station"],
            cwd=SOFTWARE_DIR,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )

        self.assertEqual(
            completed.returncode,
            0,
            msg=(
                f"{NATIVE_PYTHON} could not import the local control service:\n"
                f"{completed.stdout}{completed.stderr}"
            ),
        )


if __name__ == "__main__":
    unittest.main()
