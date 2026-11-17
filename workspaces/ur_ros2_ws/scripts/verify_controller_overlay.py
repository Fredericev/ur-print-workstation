#!/usr/bin/env python3
"""Offline-only build identity check. Never creates a ROS node or controller."""
import argparse
import ctypes
import hashlib
from pathlib import Path
import re
import shlex
import subprocess

SOURCE_DIRS = (
    "src/ur_klipper_interfaces", "src/ur_klipper_bridge", "src/ur3e_print_planner",
    "src/Universal_Robots_ROS2_Driver/ur_moveit_config",
    "src/Universal_Robots_ROS2_Driver/ur_controllers",
    "src/Universal_Robots_ROS2_Driver/ur_robot_driver",
    "src/Universal_Robots_Client_Library",
)
SOURCE_FILES = ("config/printhead_geometry.json",)


def fingerprint(workspace):
    digest = hashlib.sha256()
    for relative in SOURCE_DIRS:
        directory = workspace / relative
        if not directory.is_dir():
            raise RuntimeError(f"Required ROS source directory is missing: {directory}")
        for path in sorted(directory.rglob("*")):
            if not path.is_file() or any(part in {"__pycache__", ".git"} or part.endswith(".egg-info") for part in path.parts):
                continue
            digest.update(path.relative_to(workspace).as_posix().encode() + b"\0")
            digest.update(path.read_bytes())
    # The planner's generated C++ geometry is a build input even when no source
    # file changes. An updated CAD envelope must revoke the installed build.
    for relative in SOURCE_FILES:
        path = workspace / relative
        if not path.is_file():
            raise RuntimeError(f"Required ROS build configuration is missing: {path}")
        digest.update(relative.encode() + b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()


def verify(workspace, prefix, *, load=True):
    expected = (workspace / "install/ur_controllers").resolve()
    if prefix.resolve() != expected:
        raise RuntimeError(f"UR stop guard requires the workspace controller, but ROS selected {prefix}; rebuild and source the workspace overlay")
    library = prefix / "lib/libur_controllers.so"
    if not library.is_file() or b"cancel-deactivate-v2" not in library.read_bytes():
        raise RuntimeError(f"Patched UR controller binary is missing or stale: {library}; rebuild before starting ROS")
    if load:
        # Loading the plugin library verifies link dependencies only. No plugin
        # instance, ROS node, controller, service or action is constructed.
        ctypes.CDLL(str(library))
    return library


def verify_driver(workspace, prefix, *, load=True):
    expected = (workspace / "install/ur_robot_driver").resolve()
    if prefix.resolve() != expected:
        raise RuntimeError(
            f"Large-trajectory guard requires the workspace UR driver, but ROS selected {prefix}; "
            "rebuild and source the workspace overlay"
        )
    library = prefix / "lib/libur_robot_driver_plugin.so"
    if not library.is_file() or b"large-trajectory-rt-copy-v1" not in library.read_bytes():
        raise RuntimeError(f"Patched UR driver binary is missing or stale: {library}; rebuild before starting ROS")
    if load:
        verify_driver_urcl_abi(workspace, library)
        ctypes.CDLL(str(library))
    return library


def _rtde_target_frequency_offset(library):
    """Read the compiled C++ layout, not the current source header."""
    result = subprocess.run(
        ["gdb", "-q", "-batch", "-ex", f"file {shlex.quote(str(library))}",
         "-ex", "ptype /o 'urcl::rtde_interface::RTDEClient'"],
        capture_output=True, text=True, check=False,
    )
    offsets = re.findall(
        r"/\*\s*(\d+)\s*\|\s*8\s*\*/\s+double target_frequency_;",
        result.stdout,
    )
    if result.returncode != 0 or len(offsets) != 1:
        raise RuntimeError(
            f"Cannot verify RTDEClient ABI in {library}; a debug-symbol build and gdb are required"
        )
    return int(offsets[0])


def verify_driver_urcl_abi(workspace, driver_library):
    """Reject a driver compiled against /opt headers but loaded with workspace liburcl."""
    urcl_prefix = workspace / "install/ur_client_library"
    configs = list(urcl_prefix.rglob("ur_client_libraryConfig.cmake"))
    if len(configs) != 1:
        raise RuntimeError("Expected one workspace ur_client_library CMake config; rebuild before starting ROS")
    expected_config_dir = configs[0].parent.resolve()
    cache = workspace / "build/ur_robot_driver/CMakeCache.txt"
    if not cache.is_file():
        raise RuntimeError(f"UR driver CMake cache is missing: {cache}; rebuild before starting ROS")
    matches = re.findall(r"^ur_client_library_DIR:(?:PATH|UNINITIALIZED)=(.+)$", cache.read_text(), re.MULTILINE)
    if len(matches) != 1 or Path(matches[0]).resolve() != expected_config_dir:
        raise RuntimeError(
            "UR driver was configured against a different ur_client_library; "
            "reconfigure it against the workspace package before starting ROS"
        )

    urcl_library = urcl_prefix / "lib/liburcl.so"
    if not urcl_library.is_file():
        alternatives = list((urcl_prefix / "lib").glob("*/liburcl.so"))
        if len(alternatives) != 1:
            raise RuntimeError("Cannot identify one workspace liburcl.so for UR driver ABI verification")
        urcl_library = alternatives[0]
    dependency = subprocess.run(["ldd", str(driver_library)], capture_output=True, text=True, check=False)
    resolved = re.search(r"^\s*liburcl\.so\s*=>\s*(/\S+)", dependency.stdout, re.MULTILINE)
    if dependency.returncode != 0 or resolved is None or Path(resolved[1]).resolve() != urcl_library.resolve():
        raise RuntimeError(
            "UR driver does not resolve the workspace liburcl.so at runtime; "
            "source the workspace overlay before starting ROS"
        )

    driver_offset = _rtde_target_frequency_offset(driver_library)
    urcl_offset = _rtde_target_frequency_offset(urcl_library)
    if driver_offset != urcl_offset:
        raise RuntimeError(
            "UR driver/liburcl ABI mismatch: RTDEClient target_frequency_ is at "
            f"byte {driver_offset} in the driver and byte {urcl_offset} in liburcl; rebuild "
            "the driver against workspace ur_client_library headers"
        )


def verify_urcl(workspace, prefix, *, load=True):
    expected = (workspace / "install/ur_client_library").resolve()
    if prefix.resolve() != expected:
        raise RuntimeError(
            f"UR packet-framing guard requires the workspace ur_client_library, but ROS selected {prefix}; "
            "rebuild and source the workspace overlay"
        )

    # Prefer colcon's usual lib/ location. Some installations use a single
    # architecture-qualified lib/ subdirectory instead.
    library = prefix / "lib/liburcl.so"
    if not library.is_file():
        multiarch_libraries = sorted(path for path in (prefix / "lib").glob("*/liburcl.so") if path.is_file())
        if len(multiarch_libraries) > 1:
            raise RuntimeError(f"UR client library install is ambiguous: {multiarch_libraries}")
        if not multiarch_libraries:
            raise RuntimeError(f"Patched UR client library binary is missing: {library}; rebuild before starting ROS")
        library = multiarch_libraries[0]

    if not library.resolve().is_relative_to(workspace.resolve()):
        raise RuntimeError(f"UR client library resolves outside the workspace: {library}; rebuild before starting ROS")
    if b"[urcl-framing-v1]" not in library.read_bytes():
        raise RuntimeError(f"Patched UR client library binary is stale: {library}; rebuild before starting ROS")
    if load:
        # Verify link dependencies without constructing a client, ROS node,
        # controller, socket or robot connection.
        ctypes.CDLL(str(library))
    return library


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fingerprint", action="store_true")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    if args.fingerprint:
        print(fingerprint(args.workspace))
    else:
        from ament_index_python.packages import get_package_prefix
        urcl_library = verify_urcl(args.workspace, Path(get_package_prefix("ur_client_library")))
        controller_library = verify(args.workspace, Path(get_package_prefix("ur_controllers")))
        driver_library = verify_driver(args.workspace, Path(get_package_prefix("ur_robot_driver")))
        print(
            "Verified patched UR client library, controller and driver overlays (no ROS node started): "
            f"{urcl_library}, {controller_library}, {driver_library}"
        )


if __name__ == "__main__":
    main()
