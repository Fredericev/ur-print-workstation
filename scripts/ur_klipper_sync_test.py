#!/usr/bin/env python3
"""Launch the ROS-owned UR + Klipper synchronized coordinator.

This host-side program validates operator inputs and starts a fixed ROS
executable inside the existing container.  It deliberately contains no
Moonraker request code and no G-code: all Klipper reads and writes used by a
dry run or synchronized test are owned by the ROS Klipper bridge.
"""

from __future__ import annotations

import argparse
import os
import platform
import re
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
UR_MOVE_SCRIPT = ROOT_DIR / "scripts" / "ur3e_cartesian_xyz_moveit.sh"
UR_READY_SCRIPT = ROOT_DIR / "scripts" / "wait_for_external_control.sh"
WINDOWS_RUNTIME = ROOT_DIR / "Software" / "windows_runtime.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the ROS-owned UR + Klipper synchronized test. By default this "
            "checks the ROS bridge and plans the UR path without moving hardware."
        )
    )
    parser.add_argument("--execute", action="store_true", help="Execute real UR and Klipper motion.")
    parser.add_argument(
        "--confirm-real-robot-move",
        default=os.environ.get("CONFIRM_REAL_ROBOT_MOVE", ""),
        help="Must be YES when --execute is used.",
    )
    parser.add_argument(
        "--confirm-cartesian-move",
        default=os.environ.get("CONFIRM_CARTESIAN_MOVE", ""),
        help="Must be YES when --execute is used.",
    )
    parser.add_argument(
        "--confirm-klipper-stepper",
        default=os.environ.get("CONFIRM_KLIPPER_STEPPER", ""),
        help="Must be YES when --execute is used.",
    )
    parser.add_argument("--stepper", default="extruder", help="Native Klipper extruder object name.")
    parser.add_argument(
        "--stepper-move",
        type=float,
        default=60.0,
        help="Extruder E target in filament mm for the test.",
    )
    parser.add_argument("--stepper-speed", type=float, default=5.0, help="Extruder speed in filament mm/s.")
    parser.add_argument("--stepper-accel", type=float, default=20.0, help="Extruder acceleration in filament mm/s².")
    parser.add_argument(
        "--reset-stepper-position",
        action="store_true", default=True,
        help="Use an explicit relative E displacement (enabled for native extrusion).",
    )
    parser.add_argument(
        "--return-to-start",
        action="store_true",
        help="Return both the UR and Klipper stepper to their synchronized start positions.",
    )
    parser.add_argument(
        "--disable-stepper-after",
        action="store_true",
        help="Disable the selected extruder motor after physical motion completes.",
    )
    parser.add_argument("--axes", default="x,y,z", help="UR Cartesian axes sequence.")
    parser.add_argument("--distance-m", type=float, default=0.02, help="UR distance per axis in meters.")
    parser.add_argument(
        "--max-joint-speed-rad-s",
        type=float,
        default=0.03,
        help="UR guarded joint speed limit.",
    )
    parser.add_argument(
        "--allow-15cm",
        action="store_true",
        help="Allow UR distance up to 0.15 m per axis.",
    )
    parser.add_argument(
        "--skip-ur-ready-check",
        action="store_true",
        help="Skip the live External Control check before real execution.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Check the ROS Klipper bridge only; do not plan or move either device.",
    )
    return parser.parse_args()


def fail(message: str, code: int = 1) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(code)


def stop_if_canceled() -> None:
    cancel_file = os.environ.get("WORKSTATION_CANCEL_FILE", "")
    if cancel_file and Path(cancel_file).exists():
        fail("operation canceled before the ROS coordinator dispatched hardware", 130)


def validate_args(args: argparse.Namespace) -> None:
    if not re.fullmatch(r"extruder(?:[1-9][0-9]*)?", args.stepper):
        fail("--stepper must select a native extruder (extruder, extruder1, ...)")
    axes = [axis.strip().lower() for axis in args.axes.split(",") if axis.strip()]
    if not axes or any(axis not in {"x", "y", "z"} for axis in axes):
        fail("--axes must be a comma-separated list containing only x, y, z")
    if args.distance_m == 0.0:
        fail("--distance-m must be nonzero")
    max_distance = 0.15 if args.allow_15cm else 0.10
    if abs(args.distance_m) > max_distance:
        fail(
            f"--distance-m must be <= {max_distance:.2f} m. "
            "Use --allow-15cm for a 0.15 m test."
        )
    if not -200.0 <= args.stepper_move <= 200.0:
        fail("--stepper-move must be between -200 and 200")
    if not 0.1 <= args.stepper_speed <= 5.0:
        fail("--stepper-speed must be between 0.1 and 5 mm/s")
    if abs(args.stepper_accel - 20.0) > 1e-8:
        fail("--stepper-accel must match native extruder acceleration: 20 mm/s²")
    if not 0.005 <= args.max_joint_speed_rad_s <= 0.20:
        fail("--max-joint-speed-rad-s must be between 0.005 and 0.20")
    if args.return_to_start and not args.reset_stepper_position:
        fail(
            "--return-to-start requires --reset-stepper-position because "
            "the test needs an explicit logical E origin",
            64,
        )
    if args.execute:
        missing = []
        if args.confirm_real_robot_move != "YES":
            missing.append("--confirm-real-robot-move YES")
        if args.confirm_cartesian_move != "YES":
            missing.append("--confirm-cartesian-move YES")
        if args.confirm_klipper_stepper != "YES":
            missing.append("--confirm-klipper-stepper YES")
        if missing:
            fail("real execution requires " + ", ".join(missing), 64)


def run_command(command: list[str], env: dict[str, str] | None = None) -> int:
    process = subprocess.run(command, cwd=ROOT_DIR, env=env)
    return process.returncode


def ros_double(value: float) -> str:
    """Format a ROS parameter so YAML always preserves its double type."""
    text = format(float(value), ".12g")
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def ros_environment(args: argparse.Namespace, *, execute: bool) -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "EXECUTE": "true" if execute else "false",
            "AXES": args.axes,
            "DISTANCE_M": ros_double(args.distance_m),
            "MAX_JOINT_SPEED_RAD_S": ros_double(args.max_joint_speed_rad_s),
            "SYNCHRONIZE_KLIPPER": "true",
            "KLIPPER_CHECK_ONLY": "true" if args.check_only else "false",
            "KLIPPER_STEPPER": args.stepper,
            "KLIPPER_TARGET_POSITION": ros_double(args.stepper_move),
            "KLIPPER_SPEED": ros_double(args.stepper_speed),
            "KLIPPER_ACCELERATION": ros_double(args.stepper_accel),
            "KLIPPER_RESET_POSITION": "true" if args.reset_stepper_position else "false",
            "KLIPPER_DISABLE_AFTER": "true" if args.disable_stepper_after else "false",
            "RETURN_TO_START": "true" if args.return_to_start else "false",
            "CONFIRM_REAL_ROBOT_MOVE": "YES" if execute else "",
            "CONFIRM_CARTESIAN_MOVE": "YES" if execute else "",
            "CONFIRM_KLIPPER_STEPPER": "YES" if execute else "",
        }
    )
    if args.allow_15cm:
        env["ALLOW_15CM_CARTESIAN_MOVE"] = "YES"
    return env


def wait_for_ur_ready(args: argparse.Namespace) -> None:
    if args.skip_ur_ready_check:
        print("Skipping the live UR External Control ready check.", flush=True)
        return
    print("Checking live UR External Control through ROS…", flush=True)
    command = (
        [
            sys.executable,
            str(WINDOWS_RUNTIME),
            "wait-external-control",
            "--timeout",
            os.environ.get("TIMEOUT_SECONDS", "90"),
        ]
        if platform.system() == "Windows"
        else [str(UR_READY_SCRIPT)]
    )
    code = run_command(command)
    if code != 0:
        fail(f"UR External Control ready check failed with exit code {code}", code)


def windows_coordinator_command(
    args: argparse.Namespace,
    *,
    execute: bool,
) -> list[str]:
    """Map validated coordinator inputs to the Windows Docker orchestrator."""

    command = [
        sys.executable,
        str(WINDOWS_RUNTIME),
        "run-coordinator",
        "--execute",
        "true" if execute else "false",
        "--axes",
        args.axes,
        "--distance-m",
        ros_double(args.distance_m),
        "--max-joint-speed-rad-s",
        ros_double(args.max_joint_speed_rad_s),
        "--klipper-stepper",
        args.stepper,
        "--klipper-target-position",
        ros_double(args.stepper_move),
        "--klipper-speed",
        ros_double(args.stepper_speed),
        "--klipper-acceleration",
        ros_double(args.stepper_accel),
        "--klipper-check-only",
        "true" if args.check_only else "false",
        "--klipper-reset-position",
        "true" if args.reset_stepper_position else "false",
        "--klipper-disable-after",
        "true" if args.disable_stepper_after else "false",
        "--return-to-start",
        "true" if args.return_to_start else "false",
        "--allow-15cm",
        "true" if args.allow_15cm else "false",
        "--synchronize-klipper",
        "true",
    ]
    if execute:
        command.extend(
            [
                "--confirm-real-robot-move",
                args.confirm_real_robot_move,
                "--confirm-cartesian-move",
                args.confirm_cartesian_move,
                "--confirm-klipper-stepper",
                args.confirm_klipper_stepper,
            ]
        )
    return command


def run_ros_coordinator(args: argparse.Namespace, *, execute: bool) -> None:
    stop_if_canceled()
    environment = ros_environment(args, execute=execute)
    command = (
        windows_coordinator_command(args, execute=execute)
        if platform.system() == "Windows"
        else [str(UR_MOVE_SCRIPT)]
    )
    code = run_command(
        command,
        env=environment,
    )
    if code != 0:
        operation = "synchronized execution" if execute else "plan-only check"
        fail(f"ROS {operation} failed with exit code {code}", code)


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(line_buffering=True)

    args = parse_args()
    validate_args(args)

    print("UR + Klipper ROS synchronized coordinator")
    print("Klipper control owner: ROS 2 /klipper/manual_stepper_move")
    print("Host Moonraker/G-code writes: disabled")
    print(
        f"Klipper extruder={args.stepper}, E target={args.stepper_move:g} mm, "
        f"speed={args.stepper_speed:g} mm/s, accel={args.stepper_accel:g} mm/s²"
    )
    print(
        f"UR axes={args.axes}, distance_m={args.distance_m:g}, "
        f"max_joint_speed={args.max_joint_speed_rad_s:g}"
    )
    print(f"Execute: {args.execute}")
    print(f"Return to start: {args.return_to_start}")

    if args.check_only:
        stop_if_canceled()
        run_ros_coordinator(args, execute=False)
        print("ROS bridge check complete. No hardware moved.")
        return 0

    if not args.execute:
        stop_if_canceled()
        print("Running ROS bridge readiness check and UR plan-only check; no hardware will move.")
        run_ros_coordinator(args, execute=False)
        print("Plan-only complete. No hardware moved.")
        return 0

    stop_if_canceled()
    wait_for_ur_ready(args)
    stop_if_canceled()
    run_ros_coordinator(args, execute=True)
    print("Synchronized ROS test complete.")
    if args.return_to_start:
        print("Return-to-start completed for UR and Klipper.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
