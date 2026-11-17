#!/usr/bin/env python3
"""Replace iCloud-placeholder ROS outputs with empty rebuild directories.

Only Darwin build/install metadata is inspected. Source, configuration, runtime
and operator state are never opened or moved. Old generated directories are
preserved together under a timestamped, COLCON_IGNORE-marked backup directory.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import platform
import stat
import sys
import tempfile


# Apple sys/stat.h calls this read-only synthetic flag SF_DATALESS. Python's
# stat module does not expose it on all supported macOS Python installations.
DATALESS_FLAG = getattr(stat, "SF_DATALESS", 0x40000000)
GENERATED_DIRECTORIES = ("build", "install")


class ArtifactRecoveryError(RuntimeError):
    """Recovery cannot safely replace the generated directory pair."""


def _first_dataless_regular_file(directory: Path) -> Path | None:
    try:
        root_metadata = os.lstat(directory)
    except FileNotFoundError:
        return None
    if not stat.S_ISDIR(root_metadata.st_mode):
        return None  # Includes symlink roots; never inspect their targets.

    def walk_error(error: OSError) -> None:
        raise error

    for root, directories, files in os.walk(directory, followlinks=False, onerror=walk_error):
        # Explicitly prune symlink directories rather than depending on the
        # link-target classification performed by os.walk.
        real_directories = []
        for name in directories:
            try:
                metadata = os.lstat(Path(root) / name)
            except FileNotFoundError:
                continue
            if stat.S_ISDIR(metadata.st_mode):
                real_directories.append(name)
        directories[:] = real_directories
        for name in files:
            path = Path(root) / name
            try:
                metadata = os.lstat(path)
            except FileNotFoundError:
                continue
            if stat.S_ISREG(metadata.st_mode) and getattr(metadata, "st_flags", 0) & DATALESS_FLAG:
                return path
    return None


def _triggers(workspace: Path) -> list[Path]:
    return [path for name in GENERATED_DIRECTORIES
            if (path := _first_dataless_regular_file(workspace / name)) is not None]


def _lock_path(workspace: Path) -> Path:
    # Keep the lock off Desktop/iCloud as well: a placeholder lock file could
    # otherwise fail before recovery. The canonical workspace isolates locks.
    identity = hashlib.sha256(os.fsencode(workspace.resolve())).hexdigest()[:24]
    return Path(tempfile.gettempdir()) / f"ur-print-generated-artifacts-{identity}.lock"


def _replace_generated_pair(workspace: Path) -> Path:
    existing = []
    for name in GENERATED_DIRECTORIES:
        path = workspace / name
        try:
            metadata = os.lstat(path)
        except FileNotFoundError:
            continue
        if not stat.S_ISDIR(metadata.st_mode):
            raise ArtifactRecoveryError(f"Refusing to replace non-directory or symlink: {path}")
        existing.append(path)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    backup = Path(tempfile.mkdtemp(prefix=f"generated-artifacts-dataless-{timestamp}-", dir=workspace))
    # Colcon recursively searches the workspace. Preserve old outputs without
    # letting their generated package metadata enter the fresh build discovery.
    (backup / "COLCON_IGNORE").write_text("Preserved iCloud-placeholder build/install outputs.\n", encoding="utf-8")
    moved = []
    created = []
    try:
        for path in existing:
            destination = backup / path.name
            os.rename(path, destination)
            moved.append((path, destination))
        for name in GENERATED_DIRECTORIES:
            path = workspace / name
            path.mkdir(mode=0o755)
            created.append(path)
    except OSError as error:
        # Remove only our new empty directories. Never recursively delete an
        # output, including one that another process started writing meanwhile.
        for path in reversed(created):
            try:
                path.rmdir()
            except OSError:
                pass
        for original, preserved in reversed(moved):
            if not os.path.lexists(original):
                try:
                    os.rename(preserved, original)
                except OSError:
                    pass
        raise ArtifactRecoveryError(
            f"Could not replace generated ROS outputs: {error}. "
            f"Original outputs remain at their old paths or in {backup}."
        ) from error
    return backup


def prepare_generated_artifacts(workspace: Path) -> Path | None:
    """Recover only when a real generated file has macOS's dataless flag."""
    if platform.system() != "Darwin":
        return None
    workspace = Path(workspace)
    if not _triggers(workspace):
        return None

    import fcntl  # Darwin-only; the module remains importable on Windows.

    lock_fd = os.open(_lock_path(workspace), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(lock_fd, "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        # Two launch paths may discover the same placeholders. Rescan while
        # locked so the second invocation sees the new local directories.
        if not _triggers(workspace):
            return None
        return _replace_generated_pair(workspace)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    args = parser.parse_args()
    try:
        backup = prepare_generated_artifacts(args.workspace)
    except (OSError, ArtifactRecoveryError) as error:
        print(f"ROS generated-artifact preflight failed: {error}", file=sys.stderr)
        return 74
    if backup is not None:
        print(f"iCloud placeholders found in ROS outputs; old build/install preserved at {backup}.")
        print("Created empty local build/install directories for a complete ROS rebuild.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
