"""Exercise Mac installer failure paths without touching the real app."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


BUILDER = Path(__file__).resolve().parents[1] / "build-macos-app.command"


class MacAppBuilderTests(unittest.TestCase):
    def run_isolated_builder(
        self, *, running: bool = False, compile_failure: bool = True,
    ) -> tuple[subprocess.CompletedProcess[str], bool, bool, bool, bool]:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            software = root / "Software"
            native = software / "native_app"
            native.mkdir(parents=True)
            apps = root / "Applications"
            old_app = apps / "UR Print Workstation.app"
            old_app.mkdir(parents=True)
            marker = old_app / "previous-version.txt"
            marker.write_text("keep me", encoding="utf-8")
            mac_dir = software / "Mac"
            mac_dir.mkdir()
            app_link = mac_dir / "UR Print Workstation.app"
            app_link.symlink_to(old_app)
            for name in ("AppIcon.icns", "Info.plist", "URPrintWorkstation.swift", "main.swift"):
                (native / name).write_text("fixture", encoding="utf-8")

            fake_xcode = root / "fake-xcode-select"
            fake_xcode.write_text("#!/bin/sh\nprintf '/fixture/Developer\\n'\n", encoding="utf-8")
            fake_xcode.chmod(0o755)
            fake_swift = root / "fake-swiftc"
            fake_swift.write_text(
                "#!/bin/sh\n"
                "if [ \"$1\" = --version ]; then printf 'Swift fixture\\n'; exit 0; fi\n"
                + (
                    "printf 'fixture compiler failed\\n' >&2\nexit 42\n"
                    if compile_failure else
                    "while [ \"$#\" -gt 0 ]; do\n"
                    "  if [ \"$1\" = -o ]; then shift; printf '#!/bin/sh\\n' > \"$1\"; exit 0; fi\n"
                    "  shift\n"
                    "done\nexit 2\n"
                ),
                encoding="utf-8",
            )
            fake_swift.chmod(0o755)
            fake_ps = root / "fake-ps"
            fake_ps.write_text(
                "#!/bin/sh\n"
                + ("printf 'URPrintWorkstation\\n'\n" if running else "exit 0\n"),
                encoding="utf-8",
            )
            fake_ps.chmod(0o755)
            source = BUILDER.read_text(encoding="utf-8")
            source = source.replace("/usr/bin/xcode-select", str(fake_xcode))
            source = source.replace("/usr/bin/swiftc", str(fake_swift))
            source = source.replace("/bin/ps -axo comm=", f"{fake_ps} -axo comm=")
            if not compile_failure:
                fake_tool = root / "fake-tool"
                fake_tool.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
                fake_tool.chmod(0o755)
                for original in ("/usr/bin/plutil", "/usr/bin/xattr", "/usr/bin/codesign"):
                    source = source.replace(original, str(fake_tool))
            copied = software / "build-macos-app.command"
            copied.write_text(source, encoding="utf-8")
            copied.chmod(0o755)
            result = subprocess.run(
                ["/bin/zsh", str(copied)],
                env={**os.environ, "UR_PRINT_APP_DIRECTORY": str(apps)},
                capture_output=True, text=True, timeout=10,
            )
            backup_apps = list(apps.glob("UR Print Workstation.backup-*.app"))
            old_preserved = marker.is_file() and marker.read_text(encoding="utf-8") == "keep me"
            backup_preserved = len(backup_apps) == 1 and (
                backup_apps[0] / "previous-version.txt"
            ).read_text(encoding="utf-8") == "keep me"
            new_binary = (old_app / "Contents/MacOS/URPrintWorkstation").is_file()
            link_points_to_app = app_link.is_symlink() and app_link.resolve() == old_app.resolve()
            return result, old_preserved, backup_preserved, new_binary, link_points_to_app

    def test_compile_failure_preserves_existing_installation(self) -> None:
        result, old_app_preserved, _, _, _ = self.run_isolated_builder()
        self.assertEqual(result.returncode, 42, result.stdout + result.stderr)
        self.assertIn("fixture compiler failed", result.stderr)
        self.assertTrue(old_app_preserved)

    def test_running_workstation_blocks_replacement_before_compiling(self) -> None:
        result, old_app_preserved, _, _, _ = self.run_isolated_builder(running=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Close UR Print Workstation", result.stderr)
        self.assertNotIn("fixture compiler failed", result.stderr)
        self.assertTrue(old_app_preserved)

    def test_successful_update_retains_previous_app_for_rollback(self) -> None:
        result, old_in_place, backup, new_binary, link = self.run_isolated_builder(
            compile_failure=False
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(old_in_place)
        self.assertTrue(backup)
        self.assertTrue(new_binary)
        self.assertTrue(link)


if __name__ == "__main__":
    unittest.main()
