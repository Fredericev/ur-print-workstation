#!/usr/bin/env python3
"""Select local ROS output directories without moving files or invoking Docker."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shlex
import sys


class ArtifactCacheError(RuntimeError):
    """The selected artifact directories are not usable by this container."""


def artifact_directories(workspace: Path) -> dict[str, Path]:
    workspace = workspace.resolve()
    if platform.system() == "Darwin":
        identity = hashlib.sha256(os.fsencode(workspace)).hexdigest()[:24]
        root = (Path.home() / "Library" / "Caches" / "UR Print Workstation"
                / "ros-artifacts" / identity)
    else:
        root = workspace
    directories = {"UR_ROS_BUILD_DIR": root / "build", "UR_ROS_INSTALL_DIR": root / "install"}
    if platform.system() == "Darwin":
        # A pre-existing cache link back to Desktop would put outputs under
        # iCloud again. Refuse it rather than replacing a user-owned path.
        for path in directories.values():
            for ancestor in (path, *path.parents):
                if ancestor == Path.home():
                    break
                if ancestor.is_symlink():
                    raise ArtifactCacheError(f"ROS cache directory must not use a symlink: {ancestor}")
    return directories


def prepare_directories(workspace: Path) -> dict[str, Path]:
    directories = artifact_directories(workspace)
    for path in directories.values():
        # Existing outputs, including legacy directories on Desktop, are never
        # moved, opened, chmodded, scanned for placeholders or deleted here.
        path.mkdir(mode=0o755, parents=True, exist_ok=True)
        if not path.is_dir():
            raise ArtifactCacheError(f"ROS artifact path is not a directory: {path}")
    return directories


def verify_container_mounts(workspace: Path, inspected: object) -> None:
    if not isinstance(inspected, list) or len(inspected) != 1 or not isinstance(inspected[0], dict):
        raise ArtifactCacheError("Expected one inspected project ROS container")
    container = inspected[0]
    labels = container.get("Config", {}).get("Labels", {}) or {}
    if (labels.get("com.docker.compose.project") != "ur3e-real"
            or labels.get("com.docker.compose.service") != "ros2"):
        raise ArtifactCacheError("Inspected container does not belong to this ROS project")
    if not container.get("State", {}).get("Running"):
        raise ArtifactCacheError("The project ROS container is not running")

    mounts = container.get("Mounts", [])
    if not isinstance(mounts, list):
        raise ArtifactCacheError("Docker did not return the project container mounts")
    selected = artifact_directories(workspace)
    for variable, name in (("UR_ROS_BUILD_DIR", "build"), ("UR_ROS_INSTALL_DIR", "install")):
        target = PurePosixPath("/home/ubuntu/ur_ros2_ws") / name
        candidates = []
        for mount in mounts:
            if not isinstance(mount, dict) or not isinstance(mount.get("Destination"), str):
                continue
            destination = PurePosixPath(mount["Destination"])
            if destination.is_absolute() and target.is_relative_to(destination):
                candidates.append((len(destination.parts), destination, mount))
        if not candidates:
            raise ArtifactCacheError(f"No host bind mount supplies {target}")
        _, destination, mount = max(candidates, key=lambda item: item[0])
        source = mount.get("Source")
        if mount.get("Type") != "bind" or not isinstance(source, str) or not mount.get("RW"):
            raise ArtifactCacheError(f"{target} requires a writable host bind mount")
        actual = (Path(source) / Path(str(target.relative_to(destination)))).resolve()
        expected = selected[variable].resolve()
        if actual != expected:
            raise ArtifactCacheError(
                f"ROS {name} mount uses {actual}, but this workspace selected {expected}. "
                "Restart this project's ROS container to apply its build/install mounts, "
                "then retry. This check does not restart the container."
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--shell", action="store_true", help="Print safely quoted exports for a sourced shell")
    output.add_argument("--json", action="store_true", help="Print the selected host directories")
    output.add_argument("--check-container-mounts", action="store_true", help="Check Docker inspect JSON from stdin, without creating directories")
    args = parser.parse_args()
    try:
        if args.check_container_mounts:
            verify_container_mounts(args.workspace, json.load(sys.stdin))
            print("ROS build/install mounts match the selected host artifact directories.")
            return 0
        directories = prepare_directories(args.workspace)
        if args.shell:
            for variable, path in directories.items():
                print(f"export {variable}={shlex.quote(str(path))}")
        else:
            print(json.dumps({name: str(path) for name, path in directories.items()}, sort_keys=True))
    except (OSError, ValueError, ArtifactCacheError) as error:
        print(f"ROS artifact directory preflight failed: {error}", file=sys.stderr)
        return 78 if args.check_container_mounts else 74
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
