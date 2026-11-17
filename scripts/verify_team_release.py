#!/usr/bin/env python3
"""Offline integrity and source-snapshot check for team ZIP releases."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import stat
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]
SOFTWARE = ROOT / "Software"
VERSION_MATCH = re.search(
    r'^VERSION = "([0-9]+\.[0-9]+\.[0-9]+)"$',
    (SOFTWARE / "control_station.py").read_text(encoding="utf-8"),
    re.MULTILINE,
)
if VERSION_MATCH is None:
    raise SystemExit("Backend release version is missing or invalid")
VERSION = VERSION_MATCH.group(1)

SHARED_PATHS = (
    "Software/control_station.py",
    "Software/paired_timeline_import.py",
    "Software/print_execution.py",
    "Software/app/index.html",
    "workspaces/ur_ros2_ws/scripts/gcode_print_executor.py",
    "workspaces/ur_ros2_ws/config/workcell_geometry.json",
    "workspaces/ur_ros2_ws/config/printhead_geometry.json",
    "workspaces/ur_ros2_ws/config/ur3e_actual_calibration.yaml",
)
SHARED_PREFIXES = (
    "Software/app/",
    "workspaces/ur_ros2_ws/src/",
    "workspaces/ur_ros2_ws/scripts/",
    "workspaces/ur_ros2_ws/config/",
    "scripts/",
    "docker/",
    "real_robot_files/",
)
FORBIDDEN_SEGMENTS = {"__pycache__", ".pytest_cache", ".DS_Store"}
FORBIDDEN_WORKSPACE_SEGMENTS = {"build", "install", "runtime", "log"}
FORBIDDEN_SOFTWARE_FILES = {"config.json", "gcode_calibration.json"}
SOURCE_ONLY_SHARED_FILES = {"scripts/verify_team_release.py"}
SOURCE_MANIFEST = "SOURCE_MANIFEST.json"


def shared_runtime_file(relative: str) -> bool:
    parts = Path(relative).parts
    return (
        relative.startswith(SHARED_PREFIXES)
        or relative == "docker-compose.real.yml"
        or (len(parts) == 2 and parts[0] == "Software" and (parts[1].endswith(".py") or parts[1] == "README.md"))
    ) and relative not in SOURCE_ONLY_SHARED_FILES


def file_hash(stream) -> str:
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def source_selection(root: Path) -> tuple[dict[str, Path], list[str]]:
    """Use the exporter's allowlist so the ZIP cannot invent or omit source files."""
    exporter_path = Path(__file__).resolve().with_name("export_github_source.py")
    spec = importlib.util.spec_from_file_location("export_github_source", exporter_path)
    if spec is None or spec.loader is None:
        raise ValueError(f"Cannot load source selection rules: {exporter_path}")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    files, omitted = exporter.selected_files(root)
    return {path.relative_to(root).as_posix(): path for path in files}, omitted


def inspect_source(path: Path, folder: str, root: Path | None = None) -> tuple[dict[str, str], set[str]]:
    """Validate the source ZIP against its manifest and the export allowlist."""
    root = root or ROOT
    selected, omitted = source_selection(root)
    prefix = f"{folder}/"
    manifest_member = prefix + SOURCE_MANIFEST
    with zipfile.ZipFile(path) as archive:
        corrupt = archive.testzip()
        if corrupt is not None:
            raise ValueError(f"Corrupt ZIP member {corrupt} in {path}")
        infos = archive.infolist()
        if len({info.filename for info in infos}) != len(infos):
            raise ValueError(f"Duplicate source ZIP members in {path}")
        members = {}
        for info in infos:
            name = info.filename
            if not name.startswith(prefix):
                raise ValueError(f"Source ZIP member escapes release folder: {name}")
            relative = name[len(prefix):]
            parts = relative.rstrip("/").split("/")
            if not relative or any(part in ("", ".", "..") for part in parts) or "\\" in relative:
                raise ValueError(f"Unsafe source ZIP member: {name}")
            if info.is_dir():
                continue
            if not stat.S_ISREG(info.external_attr >> 16):
                raise ValueError(f"Source ZIP member is not a regular file: {name}")
            members[relative] = info
        expected = set(selected) | {SOURCE_MANIFEST}
        actual = set(members)
        if actual != expected:
            raise ValueError(
                f"Source ZIP file list differs from clean export: "
                f"missing={sorted(expected - actual)[:5]}, extra={sorted(actual - expected)[:5]}"
            )
        try:
            manifest = json.loads(archive.read(manifest_member))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError(f"Invalid source manifest in {path}: {error}") from error
        if not isinstance(manifest, dict) or set(manifest) != {
            "schema", "version", "files", "omitted_vendor_test_symlinks"
        }:
            raise ValueError(f"Malformed source manifest in {path}")
        if manifest["schema"] != "ur-print-source-export-v1" or manifest["version"] != VERSION:
            raise ValueError(f"Source manifest schema or release version differs in {path}")
        if manifest["omitted_vendor_test_symlinks"] != omitted:
            raise ValueError(f"Source manifest omitted symlinks differ in {path}")
        records = manifest["files"]
        if not isinstance(records, list) or len(records) != len(selected):
            raise ValueError(f"Source manifest file list is incomplete in {path}")
        hashes: dict[str, str] = {}
        for record in records:
            if not isinstance(record, dict) or set(record) != {"path", "sha256", "bytes", "mode"}:
                raise ValueError(f"Malformed source manifest file record in {path}")
            relative = record["path"]
            if not isinstance(relative, str) or relative not in selected or relative in hashes:
                raise ValueError(f"Unexpected or duplicate source manifest file: {relative}")
            digest = record["sha256"]
            if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
                raise ValueError(f"Invalid source manifest hash: {relative}")
            info = members[relative]
            if type(record["bytes"]) is not int or record["bytes"] != info.file_size:
                raise ValueError(f"Source manifest size differs from ZIP: {relative}")
            mode = oct((info.external_attr >> 16) & 0o777)
            if record["mode"] != mode:
                raise ValueError(f"Source manifest mode differs from ZIP: {relative}")
            with archive.open(info) as stream:
                actual_hash = file_hash(stream)
            if digest != actual_hash:
                raise ValueError(f"Source manifest hash differs from ZIP: {relative}")
            source = selected[relative]
            if source.is_symlink() or not source.is_file() or actual_hash != hashlib.sha256(source.read_bytes()).hexdigest():
                raise ValueError(f"Source ZIP differs from current source: {relative}")
            hashes[relative] = actual_hash
        if set(hashes) != set(selected):
            raise ValueError(f"Source manifest file list differs from ZIP in {path}")
        for relative in SHARED_PATHS:
            if relative not in hashes:
                raise ValueError(f"Required runtime file missing from source ZIP: {relative}")
        return {relative: digest for relative, digest in hashes.items() if shared_runtime_file(relative)}, set(omitted)


def release_zip(name: str, category: str) -> Path:
    path = SOFTWARE / category / f"{name}.zip"
    checksum_path = path.with_name(path.name + ".sha256")
    if not path.is_file() or not checksum_path.is_file():
        raise ValueError(f"Release archive or checksum is missing: {path}")
    checksum_line = checksum_path.read_text(encoding="ascii").strip().split()
    if (
        len(checksum_line) != 2
        or checksum_line[1] != path.name
        or re.fullmatch(r"[0-9a-fA-F]{64}", checksum_line[0]) is None
    ):
        raise ValueError(f"Malformed checksum file: {checksum_path}")
    with path.open("rb") as stream:
        actual = file_hash(stream)
    if actual != checksum_line[0].lower():
        raise ValueError(f"Checksum mismatch: {path}")
    return path


def inspect(path: Path, folder: str, project_prefix: str, executable: str, omitted_links: set[str]) -> dict[str, str]:
    with zipfile.ZipFile(path) as archive:
        corrupt = archive.testzip()
        if corrupt is not None:
            raise ValueError(f"Corrupt ZIP member {corrupt} in {path}")
        infos = archive.infolist()
        names = {info.filename for info in infos}
        if len(names) != len(infos):
            raise ValueError(f"Duplicate package ZIP members in {path}")
        if f"{folder}/{executable}" not in names:
            raise ValueError(f"Native launcher missing from {path}")
        hashes: dict[str, str] = {}
        for relative in SHARED_PATHS:
            member = f"{folder}/{project_prefix}{relative}"
            if member not in names:
                raise ValueError(f"Required shared file missing: {member}")
            source = ROOT / relative
            expected = hashlib.sha256(source.read_bytes()).hexdigest()
            actual = hashlib.sha256(archive.read(member)).hexdigest()
            if actual != expected:
                raise ValueError(f"Package differs from current source: {member}")

        for info in infos:
            name = info.filename
            if not name.startswith(f"{folder}/{project_prefix}"):
                continue
            relative = name[len(f"{folder}/{project_prefix}") :]
            parts = Path(relative).parts
            if any(part in FORBIDDEN_SEGMENTS for part in parts) or name.endswith(".pyc"):
                raise ValueError(f"Generated cache leaked into {path}: {name}")
            if parts[:1] == ("Software",) and len(parts) == 2 and parts[1] in FORBIDDEN_SOFTWARE_FILES:
                raise ValueError(f"Private software settings leaked into {path}: {name}")
            if parts[:2] == ("workspaces", "ur_ros2_ws") and len(parts) >= 3:
                if parts[2] in FORBIDDEN_WORKSPACE_SEGMENTS:
                    raise ValueError(f"Runtime/ROS build state leaked into {path}: {name}")
            if shared_runtime_file(relative) and not info.is_dir():
                if stat.S_ISLNK(info.external_attr >> 16):
                    if relative in omitted_links:
                        continue
                    raise ValueError(f"Unexpected shared source symlink in {path}: {name}")
                source = ROOT / relative
                # ROS vendors symlinks to simulator files, some intentionally
                # broken outside the simulator. ZIP tools differ in whether
                # those links are stored or skipped; they are not runtime code.
                if source.is_symlink():
                    if relative in omitted_links:
                        continue
                    raise ValueError(f"Unexpected current source symlink: {relative}")
                if not source.is_file():
                    raise ValueError(f"Packaged shared file has no current source: {name}")
                with archive.open(info) as stream:
                    actual = file_hash(stream)
                if actual != hashlib.sha256(source.read_bytes()).hexdigest():
                    raise ValueError(f"Package differs from current source: {name}")
                hashes[relative] = actual
        return hashes


def main(report_path: Path | None = None) -> int:
    source_name = f"UR-Print-Workstation-{VERSION}-source"
    source_zip = release_zip(source_name, "dist/source")
    source_fingerprint, omitted_links = inspect_source(source_zip, source_name)
    mac_name = f"UR-Print-Workstation-{VERSION}-mac-arm64"
    win_names = [f"UR-Print-Workstation-{VERSION}-win-{arch}" for arch in ("x64", "arm64")]
    packages = [
        (release_zip(mac_name, "dist/mac"), mac_name, "", "Software/Mac/UR Print Workstation.app/Contents/MacOS/URPrintWorkstation"),
        *(
            (release_zip(name, "Windows"), name, "project/", "UR Print Workstation.exe")
            for name in win_names
        ),
    ]
    fingerprints = [inspect(*entry, omitted_links) for entry in packages]
    for index, item in enumerate(fingerprints[1:], start=1):
        if item != fingerprints[0]:
            baseline = fingerprints[0]
            missing = sorted(set(baseline) - set(item))[:5]
            extra = sorted(set(item) - set(baseline))[:5]
            changed = sorted(key for key in baseline.keys() & item.keys() if baseline[key] != item[key])[:5]
            raise ValueError(
                f"Shared source differs in {packages[index][0].name}: "
                f"missing={missing}, extra={extra}, changed={changed}"
            )
    for index, item in enumerate(fingerprints):
        if item != source_fingerprint:
            missing = sorted(set(source_fingerprint) - set(item))[:5]
            extra = sorted(set(item) - set(source_fingerprint))[:5]
            changed = sorted(key for key in source_fingerprint.keys() & item.keys() if source_fingerprint[key] != item[key])[:5]
            raise ValueError(
                f"Package differs from source ZIP snapshot in {packages[index][0].name}: "
                f"missing={missing}, extra={extra}, changed={changed}"
            )
    print(f"OK  {source_zip}")
    for path, _, _, _ in packages:
        print(f"OK  {path}")
    print(f"Shared source snapshot: {VERSION}; {len(source_fingerprint)} files match source ZIP and all platforms")
    if report_path is not None:
        package_records = []
        for path in [source_zip, *(entry[0] for entry in packages)]:
            with path.open("rb") as stream:
                package_records.append({"file": path.name, "sha256": file_hash(stream)})
        report = {
            "schema": "ur-print-platform-parity-v1",
            "version": VERSION,
            "verified_at_utc": datetime.now(timezone.utc).isoformat(),
            "result": "passed",
            "shared_file_count": len(source_fingerprint),
            "packages": package_records,
            "shared_files": source_fingerprint,
            "scope": "Exact shared source parity, archive integrity, and exclusion of local runtime/settings",
            "hardware_acceptance": "Not performed; required on each target computer and workcell",
        }
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Parity report: {report_path}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, help="Save the verified package hashes and complete shared-source fingerprint as JSON")
    arguments = parser.parse_args()
    try:
        sys.exit(main(arguments.report))
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        raise SystemExit(f"Release verification failed: {error}") from error
