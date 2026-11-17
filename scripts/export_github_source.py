#!/usr/bin/env python3
"""Export a clean, self-contained GitHub source tree without touching the workcell.

The active checkout contains patched, nested vendor repositories, a developer
app symlink, and private/generated state. This allowlist exporter deliberately
copies source files, not Git gitlinks or host build artifacts. It uses only the
Python standard library and never starts Docker, ROS, a driver, or a device.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = (
    "README.md", "REAL_ROBOT.md", "ROS_ON_MACOS.md",
    "docker-compose.real.yml", ".gitignore", ".gitattributes",
)
TREE_PATHS = (
    ".github/workflows", "docs", "scripts", "docker", "real_robot_files",
    "Software/tests", "Software/app", "Software/native_app", "Software/windows_app",
    "workspaces/ur_ros2_ws/src", "workspaces/ur_ros2_ws/config",
    "workspaces/ur_ros2_ws/scripts",
)
SOFTWARE_SUFFIXES = {".py", ".command", ".ps1", ".md"}
EXCLUDED_NAMES = {
    ".git", "__pycache__", ".pytest_cache", ".DS_Store", "node_modules",
    "build", "install", "runtime", "log", "bin", "obj", "dist",
}
REQUIRED_FILES = (
    "README.md", "docker-compose.real.yml", "docker/ros2/Dockerfile",
    "Software/control_station.py", "Software/print_execution.py",
    "Software/paired_timeline_import.py", "Software/runtime_lifecycle.py",
    "Software/ur_stop_proof.py", "Software/printhead_geometry.py",
    "Software/app/index.html", "Software/app/vendor/three/LICENSE.txt",
    "Software/native_app/Info.plist", "Software/native_app/main.swift",
    "Software/native_app/URPrintWorkstation.swift",
    "Software/windows_app/Program.cs",
    "Software/windows_app/URPrintWorkstation.Windows.csproj",
    "scripts/build_print_planner.sh", "scripts/ensure_workstation_ros_build.sh",
    "workspaces/ur_ros2_ws/scripts/gcode_print_executor.py",
    "workspaces/ur_ros2_ws/scripts/verify_controller_overlay.py",
    "workspaces/ur_ros2_ws/scripts/smooth_trajectory.py",
    "workspaces/ur_ros2_ws/config/workcell_geometry.json",
    "workspaces/ur_ros2_ws/config/printhead_geometry.json",
    "workspaces/ur_ros2_ws/config/ur3e_actual_calibration.yaml",
    "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/CMakeLists.txt",
    "workspaces/ur_ros2_ws/src/Universal_Robots_Client_Library/LICENSE_apache_2_0",
    "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/LICENSE",
    "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_controllers/CMakeLists.txt",
    "workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/CMakeLists.txt",
    "workspaces/ur_ros2_ws/src/ur3e_print_planner/CMakeLists.txt",
    "workspaces/ur_ros2_ws/src/ur3e_printhead_description/CMakeLists.txt",
    "workspaces/ur_ros2_ws/src/ur_klipper_bridge/setup.py",
    "workspaces/ur_ros2_ws/src/ur_klipper_interfaces/CMakeLists.txt",
)
MAX_SOURCE_FILE_BYTES = 95 * 1024 * 1024
VENDOR_TEST_PREFIX = (
    "workspaces", "ur_ros2_ws", "src", "Universal_Robots_Client_Library", "tests",
)


class ExportError(ValueError):
    """The source tree cannot be safely exported."""


def source_version(root: Path) -> str:
    backend = root / "Software/control_station.py"
    if not backend.is_file() or backend.is_symlink():
        raise ExportError("Required backend is missing or is a symlink")
    match = re.search(r'^VERSION = "([0-9]+\.[0-9]+\.[0-9]+)"$', backend.read_text(encoding="utf-8"), re.MULTILINE)
    if match is None:
        raise ExportError("Backend VERSION must be a three-part release version")
    return match.group(1)


def selected_files(root: Path) -> tuple[list[Path], list[str]]:
    """Return allowlisted regular files; never follow a symlinked directory."""
    files: dict[str, Path] = {}
    omitted_links: list[str] = []

    def add(path: Path) -> None:
        relative = path.relative_to(root)
        parts = relative.parts
        if path.is_symlink():
            # URCL carries an external simulator fixture, not executable code.
            if parts[:len(VENDOR_TEST_PREFIX)] == VENDOR_TEST_PREFIX:
                omitted_links.append(relative.as_posix())
                return
            raise ExportError(f"Selected source symlink is not portable: {relative.as_posix()}")
        if not path.is_file():
            raise ExportError(f"Selected source is not a regular file: {relative.as_posix()}")
        if path.stat().st_size > MAX_SOURCE_FILE_BYTES:
            raise ExportError(f"Source file exceeds the GitHub-safe size budget: {relative.as_posix()}")
        files[relative.as_posix()] = path

    for relative in ROOT_FILES:
        path = root / relative
        if path.exists() or path.is_symlink():
            add(path)
    software = root / "Software"
    if software.is_symlink():
        raise ExportError("Software must not be a symlink")
    if software.is_dir():
        for path in software.iterdir():
            if path.suffix in SOFTWARE_SUFFIXES or path.name == ".gitignore":
                add(path)
    for relative in ("Software/Mac/README-MAC.txt", "Software/Windows/README-WINDOWS.txt"):
        path = root / relative
        if path.exists() or path.is_symlink():
            add(path)
    for relative in TREE_PATHS:
        tree = root / relative
        if tree.is_symlink():
            raise ExportError(f"Selected source directory is a symlink: {relative}")
        if not tree.exists():
            continue
        if not tree.is_dir():
            raise ExportError(f"Selected source directory is not a directory: {relative}")
        # os.walk does not follow child symlinks, but exposes them for rejection.
        for current, directories, names in os.walk(tree, followlinks=False):
            current_path = Path(current)
            keep = []
            for name in sorted(directories):
                if name in EXCLUDED_NAMES or (name == ".github" and current_path != root):
                    continue
                directory = current_path / name
                if directory.is_symlink():
                    add(directory)
                else:
                    keep.append(name)
            directories[:] = keep
            for name in sorted(names):
                if name in EXCLUDED_NAMES or name.endswith((".pyc", ".pyo")):
                    continue
                add(current_path / name)
    missing = [relative for relative in REQUIRED_FILES if relative not in files]
    if missing:
        raise ExportError("Required source files are missing: " + ", ".join(missing))
    return [files[name] for name in sorted(files)], sorted(omitted_links)


def export_source(root: Path, output_dir: Path) -> tuple[Path, Path]:
    root = root.resolve(strict=True)
    version = source_version(root)
    files, omitted_links = selected_files(root)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_dir = output_dir.resolve()
    name = f"UR-Print-Workstation-{version}-source"
    package = output_dir / name
    archive_path = output_dir / (name + ".zip")
    checksum_path = archive_path.with_name(archive_path.name + ".sha256")
    lock_path = output_dir / ("." + name + ".export-lock")
    for output in (package, archive_path, checksum_path):
        if output.exists() or output.is_symlink():
            raise ExportError(f"Export already exists; do not replace a published version: {output.name}")
    try:
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise ExportError(f"Another export or interrupted build owns {lock_path.name}") from error
    os.close(lock_fd)
    staging = Path(tempfile.mkdtemp(prefix=".source-stage-", dir=output_dir))
    staged_package = staging / name
    staged_package.mkdir()
    manifest_files = []
    try:
        for source in files:
            relative = source.relative_to(root)
            destination = staged_package / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            # Use captured file bytes, never copy a replacement symlink target.
            if source.is_symlink():
                raise ExportError(f"Source changed to a symlink during export: {relative.as_posix()}")
            content = source.read_bytes()
            destination.write_bytes(content)
            mode = 0o755 if source.stat().st_mode & stat.S_IXUSR else 0o644
            destination.chmod(mode)
            manifest_files.append({"path": relative.as_posix(), "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content), "mode": oct(mode)})
        manifest = {"schema": "ur-print-source-export-v1", "version": version, "files": manifest_files, "omitted_vendor_test_symlinks": omitted_links}
        (staged_package / "SOURCE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        staged_archive = staging / archive_path.name
        archive_modes = {record["path"]: int(record["mode"], 8) for record in manifest_files}
        archive_modes["SOURCE_MANIFEST.json"] = 0o644
        with zipfile.ZipFile(staged_archive, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in sorted(staged_package.rglob("*")):
                if path.is_file():
                    # Windows chmod cannot represent Unix executable bits.
                    # Bind ZIP permissions to the recorded manifest instead
                    # of re-reading platform-dependent staging permissions.
                    relative = path.relative_to(staged_package).as_posix()
                    info = zipfile.ZipInfo.from_file(path, path.relative_to(staging).as_posix())
                    info.create_system = 3
                    info.external_attr = (stat.S_IFREG | archive_modes[relative]) << 16
                    info.compress_type = zipfile.ZIP_DEFLATED
                    archive.writestr(info, path.read_bytes())
        with zipfile.ZipFile(staged_archive) as archive:
            corrupt = archive.testzip()
            if corrupt is not None:
                raise ExportError(f"Source ZIP failed integrity verification: {corrupt}")
        digest = hashlib.sha256(staged_archive.read_bytes()).hexdigest()
        staged_checksum = staging / checksum_path.name
        staged_checksum.write_text(f"{digest}  {archive_path.name}\n", encoding="ascii")
        # Re-check under the exclusive export lock before publishing anything.
        if any(path.exists() or path.is_symlink() for path in (package, archive_path, checksum_path)):
            raise ExportError("Export outputs appeared during staging; nothing was replaced")
        staged_package.rename(package)
        staged_archive.rename(archive_path)
        staged_checksum.rename(checksum_path)
        staging.rmdir()
    except Exception:
        # Keep exact staging evidence, never delete user data or overwrite outputs.
        raise
    finally:
        lock_path.unlink()
    return package, archive_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--output-dir", type=Path)
    arguments = parser.parse_args()
    output = arguments.output_dir or arguments.project_root / "Software/dist/source"
    try:
        package, archive = export_source(arguments.project_root, output)
    except (OSError, ExportError) as error:
        parser.exit(1, f"Source export failed: {error}\n")
    print(f"GitHub-ready source folder: {package}")
    print(f"Source archive: {archive}")
    print(f"Checksum: {archive}.sha256")
    print("Nested vendor Git metadata, local settings, jobs and build state were excluded. No devices were contacted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
