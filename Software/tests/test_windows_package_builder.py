from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import zipfile


SOFTWARE = Path(__file__).resolve().parents[1]
BUILDER = SOFTWARE / "build-windows-package.command"


@unittest.skipUnless(shutil.which("zsh"), "the macOS cross-builder needs zsh")
class WindowsPayloadSnapshotTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "Project with spaces"
        self.destination = Path(self.temporary.name) / "snapshot"
        for relative in (
            "docker-compose.real.yml",
            "docker/ros2/Dockerfile",
            "scripts/start_ros.sh",
            "scripts/verify_team_release.py",
            "scripts/__pycache__/cached.pyc",
            "real_robot_files/externalcontrol.urcap",
            "workspaces/ur_ros2_ws/src/planner/CMakeLists.txt",
            "workspaces/ur_ros2_ws/src/vendor/.git/config",
            "workspaces/ur_ros2_ws/src/vendor/.github/workflows/check.yml",
            "workspaces/ur_ros2_ws/config/printhead_geometry.json",
            "workspaces/ur_ros2_ws/scripts/executor.py",
            "Software/control_station.py",
            "Software/future_shared_module.py",
            "Software/windows_runtime.py",
            "Software/README.md",
            "Software/config.json",
            "Software/gcode_calibration.json",
            "Software/runtime/private.log",
            "Software/app/index.html",
            "Software/app/nested/new_asset.js",
        ):
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(relative + "\n", encoding="utf-8")

    def snapshot(self) -> None:
        script = BUILDER.read_text(encoding="utf-8")
        functions = script[script.index("copy_snapshot_tree() {"):script.index("build_architecture() {")]
        preamble = "\n".join((
            "set -euo pipefail",
            "export LC_ALL=C LANG=C",
            'PROJECT_ROOT="$1"',
            'SOFTWARE_DIR="$PROJECT_ROOT/Software"',
            'APP_VERSION="0.14.0"',
        ))
        subprocess.run(
            ["zsh", "-c", preamble + "\n" + functions + '\ncopy_project_payload "$2"\n',
             "snapshot-test", str(self.root), str(self.destination)],
            check=True, capture_output=True, text=True,
        )

    def test_new_modules_assets_and_ros_sources_ship_without_local_state(self) -> None:
        self.snapshot()
        for relative in (
            "Software/future_shared_module.py",
            "Software/app/nested/new_asset.js",
            "workspaces/ur_ros2_ws/src/planner/CMakeLists.txt",
            "workspaces/ur_ros2_ws/scripts/executor.py",
            "workspaces/ur_ros2_ws/config/printhead_geometry.json",
        ):
            self.assertEqual((self.destination / relative).read_bytes(), (self.root / relative).read_bytes())
        for relative in (
            "Software/config.json", "Software/gcode_calibration.json", "Software/runtime",
            "scripts/verify_team_release.py", "scripts/__pycache__",
            "workspaces/ur_ros2_ws/src/vendor/.git",
            "workspaces/ur_ros2_ws/src/vendor/.github",
        ):
            self.assertFalse((self.destination / relative).exists(), relative)

    def test_payload_identity_uses_all_files_and_canonical_sha256_records(self) -> None:
        self.snapshot()
        records = "".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(self.destination).as_posix()}\n"
            for path in sorted(self.destination.rglob("*"), key=lambda item: item.relative_to(self.destination).as_posix())
            if path.is_file() and path.name != "PAYLOAD_VERSION.txt"
        )
        expected = hashlib.sha256(records.encode("utf-8")).hexdigest()[:20]
        identity = (self.destination / "PAYLOAD_VERSION.txt").read_text(encoding="ascii")
        self.assertEqual(identity, f"0.14.0-{expected}\n")
        (self.root / "Software/future_shared_module.py").write_text("updated\n", encoding="utf-8")
        shutil.rmtree(self.destination)
        self.snapshot()
        self.assertNotEqual((self.destination / "PAYLOAD_VERSION.txt").read_text(encoding="ascii"), identity)


POWERSHELL = (
    os.environ.get("UR_TEST_POWERSHELL")
    or shutil.which("powershell.exe")
    or shutil.which("powershell")
    or shutil.which("pwsh")
)


@unittest.skipUnless(POWERSHELL, "PowerShell is required to exercise the Windows ZIP writer")
class PowerShellPackageArchiveTests(unittest.TestCase):
    def test_archive_uses_portable_paths_and_preserves_every_manifest_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage = root / "UR Print Workstation test package"
            contents = {
                "UR Print Workstation.exe": b"MZ\x00native launcher fixture",
                "Replay Offline Print.cmd": b"@echo off\r\nexit /b 0\r\n",
                "python/python313._pth": b"python313.zip\n.\n../project/Software\n",
                "project/Software/control_station.py": b'VERSION = "0.14.2"\n',
                "project/Software/app/.hidden-support.txt": b"required hidden support asset\n",
                "project/Software/app/assets/model with spaces.bin": bytes(range(256)),
            }
            for relative, content in contents.items():
                path = stage / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            archive_path = root / "package.zip"
            script = root / "archive-test.ps1"
            script.write_text(r'''param([string]$Builder, [string]$SourceDirectory, [string]$ArchivePath)
$ErrorActionPreference = "Stop"
$Tokens = $null
$Errors = $null
$Ast = [System.Management.Automation.Language.Parser]::ParseFile($Builder, [ref]$Tokens, [ref]$Errors)
if ($Errors.Count) { throw ($Errors | Out-String) }
foreach ($Name in @("Get-PayloadRelativePath", "Get-CanonicalHashLines", "New-PortablePackageArchive")) {
    $Function = $Ast.FindAll({ param($Node) $Node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -eq $Name }, $false)
    if ($Function.Count -ne 1) { throw "Missing or ambiguous package function: $Name" }
    Invoke-Expression $Function[0].Extent.Text
}
if ($env:OS -eq "Windows_NT") {
    $Hidden = Join-Path $SourceDirectory "project/Software/app/.hidden-support.txt"
    [IO.File]::SetAttributes($Hidden, ([IO.File]::GetAttributes($Hidden) -bor [IO.FileAttributes]::Hidden))
}
$Manifest = ((Get-CanonicalHashLines $SourceDirectory "SHA256SUMS.txt") -join "`n") + "`n"
[IO.File]::WriteAllText((Join-Path $SourceDirectory "SHA256SUMS.txt"), $Manifest, [Text.Encoding]::ASCII)
New-PortablePackageArchive $SourceDirectory $ArchivePath
''', encoding="utf-8")
            subprocess.run(
                [str(POWERSHELL), "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
                 "-File", str(script), "-Builder", str(SOFTWARE / "build-windows-package.ps1"),
                 "-SourceDirectory", str(stage), "-ArchivePath", str(archive_path)],
                check=True, capture_output=True, text=True,
            )
            prefix = stage.name + "/"
            with zipfile.ZipFile(archive_path) as archive:
                self.assertEqual(len(archive.infolist()), len(set(archive.namelist())))
                self.assertTrue(all("\\" not in name for name in archive.namelist()))
                self.assertEqual(
                    set(archive.namelist()),
                    {prefix + name for name in contents} | {prefix + "SHA256SUMS.txt"},
                )
                for relative, content in contents.items():
                    self.assertEqual(archive.read(prefix + relative), content)
                records = {}
                for line in archive.read(prefix + "SHA256SUMS.txt").decode("ascii").splitlines():
                    digest, relative = line.split("  ", 1)
                    records[relative] = digest
                self.assertEqual(set(records), set(contents))
                for relative, content in contents.items():
                    self.assertEqual(records[relative], hashlib.sha256(content).hexdigest())


if __name__ == "__main__":
    unittest.main()
