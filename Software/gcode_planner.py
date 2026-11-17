"""Build a bounded, non-executable UR + Klipper plan from parsed G-code."""

from __future__ import annotations

from typing import Any

try:
    from gcode_parser import parse_gcode
except ModuleNotFoundError:
    from Software.gcode_parser import parse_gcode


_HEATER_COMMANDS = {"M104", "M109", "M140", "M190"}
_FAN_COMMANDS = {"M106", "M107"}
_MACHINE_SETTING_COMMANDS = {
    "M201",
    "M203",
    "M204",
    "M205",
    "M220",
    "M221",
    "M84",
}


class GCodePlanError(ValueError):
    """The source file cannot be promoted into a draft path plan."""


def _command_total(counts: dict[str, int], commands: set[str]) -> int:
    return sum(int(counts.get(command, 0)) for command in commands)


def build_path_plan(filename: object, content: object) -> dict[str, Any]:
    """Create a read-only two-channel plan; never send commands to hardware."""

    analysis = parse_gcode(filename, content)
    if not analysis["safe_to_plan"]:
        raise GCodePlanError(
            "Resolve the blocking parser findings before generating a path plan."
        )

    summary = analysis["summary"]
    command_counts = analysis["command_counts"]
    preparation_commands = analysis["preparation_commands"]
    heater_count = _command_total(command_counts, _HEATER_COMMANDS)
    fan_count = _command_total(command_counts, _FAN_COMMANDS)
    machine_setting_count = _command_total(
        command_counts,
        _MACHINE_SETTING_COMMANDS,
    )

    return {
        "status": "draft",
        "execution_locked": True,
        "file": analysis["file"],
        "source_sha256": analysis["file"]["sha256"],
        "coordinate_frame": {
            "source": "G-code XYZ",
            "target": "UR base",
            "calibrated": False,
            "scale": "1 mm = 1 mm",
        },
        "ur_robot": {
            "controller": "ROS 2 · MoveIt · UR driver",
            "responsibility": ["X", "Y", "Z"],
            "waypoint_count": summary["xyz_move_count"],
            "path_mm": summary["total_path_mm"],
            "print_path_mm": summary["print_path_mm"],
            "travel_path_mm": summary["travel_path_mm"],
            "estimated_duration_s": summary["estimated_duration_s"],
            "preview": analysis["preview"],
        },
        "klipper": {
            "controller": "ROS 2 Klipper Bridge · Moonraker",
            "responsibility": [
                "Extrusion E",
                "Retraction",
                "Heaters",
                "Fans",
                "Preparation",
            ],
            "extrusion_move_count": summary["extrusion_move_count"],
            "extrusion_mm": summary["extrusion_mm"],
            "retraction_count": summary["retraction_count"],
            "heater_command_count": heater_count,
            "fan_command_count": fan_count,
            "machine_setting_command_count": machine_setting_count,
            "preparation_commands": preparation_commands,
        },
        "synchronization": {
            "combined_motion_count": summary["combined_motion_count"],
            "ur_only_motion_count": summary["ur_only_motion_count"],
            "klipper_only_motion_count": summary["klipper_only_motion_count"],
            "timeline": analysis["motion_timeline"],
        },
        "gates": [
            {
                "code": "source_parsed",
                "status": "passed",
            },
            {
                "code": "coordinate_transform",
                "status": "required",
            },
            {
                "code": "collision_check",
                "status": "required",
            },
            {
                "code": "extrusion_calibration",
                "status": "required",
            },
            {
                "code": "preparation_review",
                "status": "required" if preparation_commands else "passed",
            },
        ],
    }
