"""Offline-only parsing of a paired UR trajectory and timed extrusion file.

This module deliberately produces no execution manifest and has no ROS or
Moonraker dependency. A valid pair is not a MoveIt or physical-sync approval.
"""

from __future__ import annotations

import csv
import hashlib
import io
import math
import re
from typing import Any

try:
    from print_execution import MAX_INPUT_JOINT_STEP_RAD, UR_JOINTS
except ModuleNotFoundError:
    from Software.print_execution import MAX_INPUT_JOINT_STEP_RAD, UR_JOINTS


MAX_IMPORT_FILE_BYTES = 20 * 1024 * 1024
MAX_IMPORT_POINTS = 1_000_000
MAX_IMPORT_SEGMENTS = 500_000
MAX_PREVIEW_ITEMS = 1_000
TIME_TOLERANCE_S = 1.0e-6
_NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?"
_SYNC = re.compile(rf"^@sync\s+start=({_NUMBER})\s+end=({_NUMBER})$", re.I)
_MOVE = re.compile(rf"^G1\s+E({_NUMBER})$", re.I)
_DWELL = re.compile(rf"^G4\s+P({_NUMBER})$", re.I)


class PairedTimelineError(ValueError):
    """The two files do not define one unambiguous, reviewable timeline."""


def _text(name: object, content: object, extension: str) -> str:
    if not isinstance(name, str) or not name.lower().endswith(extension):
        raise PairedTimelineError(f"Select a {extension} file for this channel.")
    if not isinstance(content, bytes) or not 0 < len(content) <= MAX_IMPORT_FILE_BYTES:
        raise PairedTimelineError(f"{name} must be a nonempty file of at most 20 MB.")
    try:
        text = content.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise PairedTimelineError(f"{name} must be UTF-8 text.") from exc
    if "\x00" in text:
        raise PairedTimelineError(f"{name} contains a NUL byte.")
    return text


def _number(value: str, label: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise PairedTimelineError(f"{label} must be a finite number.") from exc
    if not math.isfinite(number):
        raise PairedTimelineError(f"{label} must be a finite number.")
    return number


def _trajectory(text: str) -> tuple[list[dict[str, Any]], float, float]:
    try:
        rows = csv.reader(io.StringIO(text, newline=""))
        header = next(rows, None)
        if tuple(header or ()) != ("time_s", "path_progress", *UR_JOINTS):
            raise PairedTimelineError("UR trajectory CSV has an invalid joint header.")
        points: list[dict[str, Any]] = []
        maximum_step = 0.0
        previous_time = -1.0
        previous_progress = -1.0
        previous_positions: list[float] | None = None
        for line, row in enumerate(rows, start=2):
            if len(row) != 8:
                raise PairedTimelineError(f"UR trajectory row {line} must have 8 columns.")
            current_time = _number(row[0], f"UR trajectory row {line} time_s")
            progress = _number(row[1], f"UR trajectory row {line} path_progress")
            positions = [_number(value, f"UR trajectory row {line} joint") for value in row[2:]]
            if current_time < 0 or current_time <= previous_time:
                raise PairedTimelineError("UR trajectory times must strictly increase from zero.")
            if not 0.0 <= progress <= 1.0 or progress < previous_progress:
                raise PairedTimelineError("UR trajectory path_progress must increase from 0 to 1.")
            if previous_positions is not None:
                maximum_step = max(maximum_step, max(
                    abs(value - previous_positions[index])
                    for index, value in enumerate(positions)
                ))
            points.append({"time_s": current_time, "progress": progress, "positions": positions})
            if len(points) > MAX_IMPORT_POINTS:
                raise PairedTimelineError("UR trajectory has too many points.")
            previous_time, previous_progress, previous_positions = current_time, progress, positions
    except csv.Error as exc:
        raise PairedTimelineError(f"Could not parse UR trajectory CSV: {exc}") from exc
    if len(points) < 2:
        raise PairedTimelineError("UR trajectory requires at least two joint points.")
    if abs(points[0]["time_s"]) > TIME_TOLERANCE_S:
        raise PairedTimelineError("UR trajectory must start at time_s=0.")
    if abs(points[0]["progress"]) > TIME_TOLERANCE_S or abs(points[-1]["progress"] - 1) > TIME_TOLERANCE_S:
        raise PairedTimelineError("UR trajectory path_progress must cover 0 through 1.")
    if maximum_step > MAX_INPUT_JOINT_STEP_RAD:
        raise PairedTimelineError(
            f"UR trajectory joint step {maximum_step:.4f} rad exceeds {MAX_INPUT_JOINT_STEP_RAD:g} rad."
        )
    return points, points[-1]["time_s"], maximum_step


def _extrusion(text: str) -> tuple[list[dict[str, float]], float]:
    relative_e = False
    millimetres = False
    segments: list[dict[str, float]] = []
    cursor = 0.0
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        command, separator, comment = raw_line.partition(";")
        command = command.strip()
        comment = comment.strip()
        if not command:
            continue
        if command.upper() == "G21" and not segments:
            millimetres = True
            continue
        if command.upper() == "M83" and not segments:
            relative_e = True
            continue
        if not (relative_e and millimetres):
            raise PairedTimelineError("Extrusion file must begin with G21 and M83 before timed moves.")
        sync = _SYNC.fullmatch(comment) if separator else None
        if sync is None:
            raise PairedTimelineError(f"Extrusion line {line_number} needs ';@sync start=<s> end=<s>'.")
        start = _number(sync.group(1), f"extrusion line {line_number} start")
        end = _number(sync.group(2), f"extrusion line {line_number} end")
        if start < 0 or end <= start:
            raise PairedTimelineError(f"Extrusion line {line_number} has invalid times.")
        if abs(start - cursor) > TIME_TOLERANCE_S:
            raise PairedTimelineError("Extrusion segments must form a continuous timeline without gaps or overlap.")
        move = _MOVE.fullmatch(command)
        dwell = _DWELL.fullmatch(command)
        if move is not None:
            delta = _number(move.group(1), f"extrusion line {line_number} E")
        elif dwell is not None:
            milliseconds = _number(dwell.group(1), f"extrusion line {line_number} P")
            if milliseconds < 0 or abs(milliseconds / 1000.0 - (end - start)) > TIME_TOLERANCE_S:
                raise PairedTimelineError(f"Extrusion line {line_number} dwell does not match its @sync duration.")
            delta = 0.0
        else:
            raise PairedTimelineError(
                f"Extrusion line {line_number} is unsupported; only relative G1 E and G4 P are allowed."
            )
        segments.append({"start_time_s": start, "end_time_s": end, "delta_e_mm": delta})
        if len(segments) > MAX_IMPORT_SEGMENTS:
            raise PairedTimelineError("Extrusion file has too many timed segments.")
        cursor = end
    if not segments:
        raise PairedTimelineError("Extrusion file has no timed segments.")
    return segments, cursor


def _sample(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if len(items) <= MAX_PREVIEW_ITEMS:
        return items
    return [items[round(index * (len(items) - 1) / (MAX_PREVIEW_ITEMS - 1))]
            for index in range(MAX_PREVIEW_ITEMS)]


def parse_paired_timeline(
    trajectory_name: object,
    trajectory_content: object,
    extrusion_name: object,
    extrusion_content: object,
) -> dict[str, Any]:
    """Validate a paired import for offline review only; never authorize motion."""

    trajectory_text = _text(trajectory_name, trajectory_content, ".csv")
    extrusion_text = _text(extrusion_name, extrusion_content, ".gcode")
    points, ur_duration, maximum_step = _trajectory(trajectory_text)
    segments, extrusion_duration = _extrusion(extrusion_text)
    if abs(ur_duration - extrusion_duration) > TIME_TOLERANCE_S:
        raise PairedTimelineError("UR and extrusion files must have the same end time.")
    return {
        "ok": True,
        "mode": "paired_timeline_v1",
        "execution_ready": False,
        "moveit_validated": False,
        "physical_sync_verified": False,
        "trajectory": {
            "name": trajectory_name,
            "sha256": hashlib.sha256(trajectory_content).hexdigest(),
            "point_count": len(points),
            "duration_s": ur_duration,
            "maximum_joint_step_rad": maximum_step,
            "joint_names": list(UR_JOINTS),
            "preview_sampled": len(points) > MAX_PREVIEW_ITEMS,
            "preview_points": _sample(points),
        },
        "extrusion": {
            "name": extrusion_name,
            "sha256": hashlib.sha256(extrusion_content).hexdigest(),
            "segment_count": len(segments),
            "duration_s": extrusion_duration,
            "positive_e_mm": sum(max(0.0, item["delta_e_mm"]) for item in segments),
            "retraction_e_mm": -sum(min(0.0, item["delta_e_mm"]) for item in segments),
            "preview_sampled": len(segments) > MAX_PREVIEW_ITEMS,
            "segments": _sample(segments),
        },
    }
