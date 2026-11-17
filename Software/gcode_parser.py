"""Offline, bounded and fail-closed G-code inspection.

The parser intentionally models only commands whose geometric meaning is
known.  Unsupported path-changing commands become blocking findings instead
of being silently discarded.  Preview data may be simplified for the UI, but
all statistics and bounds are accumulated from the unsimplified motion.
"""

from __future__ import annotations

import hashlib
import math
import re
from collections import Counter
from pathlib import PurePosixPath
from typing import Any


MAX_GCODE_BYTES = 20 * 1024 * 1024
MAX_PREVIEW_SEGMENTS = 15_000
MAX_TIMELINE_EVENTS = 1_200
MAX_ARC_SEGMENTS = 10_000
ARC_CHORD_TOLERANCE_MM = 0.02
ARC_MAX_SEGMENT_MM = 1.0
ALLOWED_EXTENSIONS = {".gcode", ".gc", ".ngc", ".nc"}

_COMMAND_RE = re.compile(
    r"^\s*(?:N\d+\s+)?(?P<command>[GMT]\s*-?\d+(?:\.\d+)?|[A-Za-z_][A-Za-z0-9_]*)",
    re.IGNORECASE,
)
_WORD_RE = re.compile(
    r"(?P<name>[A-Za-z])\s*"
    r"(?P<value>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)"
)
_PAREN_COMMENT_RE = re.compile(r"\([^)]*\)")
_LAYER_RE = re.compile(r"(?:^|;)\s*LAYER\s*:?\s*(-?\d+)", re.IGNORECASE)
_GENERATED_BY_RE = re.compile(
    r"(?:generated|sliced)\s+(?:by|with)\s+(.+)$",
    re.IGNORECASE,
)

_DANGEROUS_COMMANDS = {
    "M112",
    "FORCE_MOVE",
    "SET_KINEMATIC_POSITION",
    "SYSTEM_SHUTDOWN",
}
_CONFIG_COMMANDS = {
    "FIRMWARE_RESTART",
    "RESTART",
    "SAVE_CONFIG",
    "RUN_SHELL_COMMAND",
}
_METADATA_COMMANDS = {
    "EXCLUDE_OBJECT_DEFINE",
    "EXCLUDE_OBJECT_START",
    "EXCLUDE_OBJECT_END",
    "M73",
    "M117",
    "M118",
}
_EXTERNAL_PREPARATION_COMMANDS = {
    "PRINT_START",
    "PRINT_END",
    "START_PRINT",
    "END_PRINT",
}
_KNOWN_NON_MOTION_G_CODES = {
    "G4",
    "G10",
    "G11",
    "G17",
    "G18",
    "G19",
    "G20",
    "G21",
    "G28",
    "G90",
    "G90.1",
    "G91",
    "G91.1",
    "G92",
}
_KNOWN_NON_MOTION_M_CODES = {
    "M2",
    "M18",
    "M82",
    "M83",
    "M84",
    "M104",
    "M105",
    "M106",
    "M107",
    "M109",
    "M110",
    "M114",
    "M115",
    "M140",
    "M190",
    "M201",
    "M203",
    "M204",
    "M205",
    "M220",
    "M221",
    "M400",
    "M572",
    "M900",
}


class GCodeParseError(ValueError):
    """A user-correctable import or encoding error."""


def _clean_filename(filename: object) -> str:
    if not isinstance(filename, str):
        raise GCodeParseError("A G-code filename is required.")
    normalized = filename.replace("\\", "/").strip()
    name = PurePosixPath(normalized).name
    if not name or name in {".", ".."}:
        raise GCodeParseError("A G-code filename is required.")
    if len(name) > 180 or any(ord(character) < 32 for character in name):
        raise GCodeParseError("The G-code filename is not valid.")
    suffix = PurePosixPath(name).suffix.lower()
    if suffix not in ALLOWED_EXTENSIONS:
        allowed = ", ".join(sorted(ALLOWED_EXTENSIONS))
        raise GCodeParseError(f"Use a supported G-code file: {allowed}.")
    return name


def _decode_content(content: object) -> tuple[bytes, str]:
    if not isinstance(content, bytes):
        raise GCodeParseError("The uploaded G-code must be a file.")
    if not content:
        raise GCodeParseError("The selected G-code file is empty.")
    if len(content) > MAX_GCODE_BYTES:
        raise GCodeParseError("The selected G-code file exceeds the 20 MB limit.")
    if b"\x00" in content:
        raise GCodeParseError("The selected file is not plain-text G-code.")
    try:
        text = content.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise GCodeParseError("The G-code file must use UTF-8 text encoding.") from exc
    return content, text


def _rounded(value: float, digits: int = 4) -> float:
    rounded = round(value, digits)
    return 0.0 if rounded == -0.0 else rounded


def _normalize_numeric_command(letter: str, value: float) -> str:
    if abs(value - round(value)) <= 1e-10:
        value_text = str(int(round(value)))
    else:
        value_text = f"{value:.6f}".rstrip("0").rstrip(".")
    return f"{letter.upper()}{value_text}"


def _point_segment_distance(point: tuple[float, float, float], start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    delta = tuple(end[index] - start[index] for index in range(3))
    length_squared = sum(value * value for value in delta)
    if length_squared <= 1e-20:
        return math.sqrt(sum((point[index] - start[index]) ** 2 for index in range(3)))
    ratio = max(
        0.0,
        min(
            1.0,
            sum((point[index] - start[index]) * delta[index] for index in range(3))
            / length_squared,
        ),
    )
    projection = tuple(start[index] + ratio * delta[index] for index in range(3))
    return math.sqrt(sum((point[index] - projection[index]) ** 2 for index in range(3)))


def _merge_preview_range(segments: list[dict[str, Any]]) -> dict[str, Any]:
    first = segments[0]
    second = segments[-1]
    start = (float(first["x0"]), float(first["y0"]), float(first["z0"]))
    end = (float(second["x1"]), float(second["y1"]), float(second["z1"]))
    deviation = max(float(segment.get("_max_deviation_mm", 0.0)) for segment in segments)
    for segment in segments[:-1]:
        point = (float(segment["x1"]), float(segment["y1"]), float(segment["z1"]))
        deviation = max(deviation, _point_segment_distance(point, start, end))
    extrusion = sum(float(segment.get("extrusion_delta_mm", 0.0)) for segment in segments)
    return {
        "sequence": int(second.get("sequence", first.get("sequence", 0))),
        "line": int(second.get("line", first.get("line", 0))),
        "x0": first["x0"],
        "y0": first["y0"],
        "z0": first["z0"],
        "x1": second["x1"],
        "y1": second["y1"],
        "z1": second["z1"],
        "kind": "print" if extrusion > 1e-9 else "travel",
        "duration_s": _rounded(
            sum(float(segment.get("duration_s", 0.0)) for segment in segments),
            5,
        ),
        "cumulative_s": second.get("cumulative_s", first.get("cumulative_s", 0.0)),
        "extrusion_delta_mm": _rounded(extrusion, 5),
        "_max_deviation_mm": deviation,
        "_source_count": sum(int(segment.get("_source_count", 1)) for segment in segments),
    }


def _lttb_keep_indexes(
    points: list[tuple[float, float, float]], maximum_points: int
) -> list[int]:
    """Largest-triangle-three-buckets simplification in linear time."""

    length = len(points)
    if maximum_points >= length or maximum_points < 3:
        return list(range(length)) if maximum_points >= length else [0, length - 1]
    every = (length - 2) / (maximum_points - 2)
    selected = [0]
    anchor = 0
    for bucket in range(maximum_points - 2):
        average_start = int(math.floor((bucket + 1) * every)) + 1
        average_end = int(math.floor((bucket + 2) * every)) + 1
        average_end = min(average_end, length)
        if average_start >= average_end:
            average = points[-1]
        else:
            count = average_end - average_start
            average = tuple(
                sum(points[index][axis] for index in range(average_start, average_end)) / count
                for axis in range(3)
            )

        range_start = int(math.floor(bucket * every)) + 1
        range_end = int(math.floor((bucket + 1) * every)) + 1
        range_end = min(range_end, length - 1)
        anchor_point = points[anchor]
        direction = tuple(average[axis] - anchor_point[axis] for axis in range(3))
        best_score = -1.0
        best_index = range_start
        for index in range(range_start, max(range_start + 1, range_end)):
            vector = tuple(points[index][axis] - anchor_point[axis] for axis in range(3))
            cross = (
                vector[1] * direction[2] - vector[2] * direction[1],
                vector[2] * direction[0] - vector[0] * direction[2],
                vector[0] * direction[1] - vector[1] * direction[0],
            )
            score = sum(value * value for value in cross)
            if score > best_score:
                best_score = score
                best_index = index
        selected.append(best_index)
        anchor = best_index
    selected.append(length - 1)
    return sorted(set(selected))


def _simplify_preview_segments(
    segments: list[dict[str, Any]], maximum: int
) -> tuple[list[dict[str, Any]], float]:
    if len(segments) <= maximum:
        return segments, 0.0
    points = [
        (float(segments[0]["x0"]), float(segments[0]["y0"]), float(segments[0]["z0"]))
    ]
    points.extend(
        (float(segment["x1"]), float(segment["y1"]), float(segment["z1"]))
        for segment in segments
    )
    best = _lttb_keep_indexes(points, maximum + 1)

    simplified: list[dict[str, Any]] = []
    maximum_deviation = 0.0
    for left, right in zip(best, best[1:]):
        merged = _merge_preview_range(segments[left:right])
        maximum_deviation = max(
            maximum_deviation, float(merged.get("_max_deviation_mm", 0.0))
        )
        simplified.append(merged)
    return simplified, maximum_deviation


class _BoundedPreview:
    def __init__(self, maximum: int) -> None:
        self.maximum = maximum
        self.segments: list[dict[str, Any]] = []
        self.source_count = 0
        self.maximum_deviation_mm = 0.0

    def append(self, segment: dict[str, Any]) -> None:
        self.source_count += 1
        segment["_max_deviation_mm"] = 0.0
        segment["_source_count"] = 1
        self.segments.append(segment)
        if len(self.segments) <= self.maximum * 2:
            return
        self.segments, deviation = _simplify_preview_segments(
            self.segments, self.maximum
        )
        self.maximum_deviation_mm = max(self.maximum_deviation_mm, deviation)

    def public_segments(self) -> list[dict[str, Any]]:
        if len(self.segments) > self.maximum:
            self.segments, deviation = _simplify_preview_segments(
                self.segments, self.maximum
            )
            self.maximum_deviation_mm = max(self.maximum_deviation_mm, deviation)
        return [
            {key: value for key, value in segment.items() if not key.startswith("_")}
            for segment in self.segments
        ]


def _directed_sweep(start_angle: float, end_angle: float, clockwise: bool, full_circle: bool = False) -> float:
    if full_circle:
        return -math.tau if clockwise else math.tau
    sweep = end_angle - start_angle
    if clockwise:
        while sweep >= -1e-12:
            sweep -= math.tau
    else:
        while sweep <= 1e-12:
            sweep += math.tau
    return sweep


def _arc_center_from_radius(
    start_u: float,
    start_v: float,
    end_u: float,
    end_v: float,
    radius: float,
    clockwise: bool,
) -> tuple[float, float, float, float]:
    dx = end_u - start_u
    dy = end_v - start_v
    chord = math.hypot(dx, dy)
    radius_abs = abs(radius)
    if chord <= 1e-12:
        raise ValueError("A radius-format full circle has no unique centre.")
    if radius_abs <= 1e-12 or chord > 2.0 * radius_abs + 1e-7:
        raise ValueError("The arc radius is too small for its endpoints.")
    midpoint_u = (start_u + end_u) / 2.0
    midpoint_v = (start_v + end_v) / 2.0
    height = math.sqrt(max(0.0, radius_abs * radius_abs - (chord / 2.0) ** 2))
    perpendicular_u = -dy / chord
    perpendicular_v = dx / chord
    candidates = []
    for sign in (1.0, -1.0):
        center_u = midpoint_u + sign * height * perpendicular_u
        center_v = midpoint_v + sign * height * perpendicular_v
        start_angle = math.atan2(start_v - center_v, start_u - center_u)
        end_angle = math.atan2(end_v - center_v, end_u - center_u)
        sweep = _directed_sweep(start_angle, end_angle, clockwise)
        candidates.append((center_u, center_v, start_angle, sweep))
    wants_major = radius < 0.0
    preferred = [
        candidate
        for candidate in candidates
        if (abs(candidate[3]) > math.pi + 1e-9) == wants_major
    ]
    return (preferred or candidates)[0]


def _arc_points(
    start: dict[str, float],
    target: dict[str, float],
    parameters: dict[str, float],
    *,
    units_factor: float,
    plane: str,
    clockwise: bool,
    center_absolute: bool,
) -> tuple[list[dict[str, float]], float]:
    # Plane order matches Klipper's gcode_arcs implementation exactly.
    plane_axes = {
        "G17": ("x", "y", "z", "i", "j"),
        "G18": ("x", "z", "y", "i", "k"),
        "G19": ("y", "z", "x", "j", "k"),
    }
    u_axis, v_axis, w_axis, u_offset, v_offset = plane_axes[plane]
    start_u, start_v = start[u_axis], start[v_axis]
    end_u, end_v = target[u_axis], target[v_axis]
    has_radius = "r" in parameters
    has_center = u_offset in parameters or v_offset in parameters
    if has_radius == has_center:
        raise ValueError("Use either R or I/J/K centre values for an arc, not both or neither.")

    if has_radius:
        center_u, center_v, start_angle, sweep = _arc_center_from_radius(
            start_u,
            start_v,
            end_u,
            end_v,
            parameters["r"] * units_factor,
            clockwise,
        )
        radius = math.hypot(start_u - center_u, start_v - center_v)
    else:
        if center_absolute:
            center_u = (
                parameters[u_offset] * units_factor
                if u_offset in parameters
                else start_u
            )
            center_v = (
                parameters[v_offset] * units_factor
                if v_offset in parameters
                else start_v
            )
        else:
            center_u = start_u + parameters.get(u_offset, 0.0) * units_factor
            center_v = start_v + parameters.get(v_offset, 0.0) * units_factor
        radius = math.hypot(start_u - center_u, start_v - center_v)
        end_radius = math.hypot(end_u - center_u, end_v - center_v)
        tolerance = max(0.01, radius * 0.001)
        if radius <= 1e-12 or abs(radius - end_radius) > tolerance:
            raise ValueError("The arc endpoint is not on the circle defined by I/J/K.")
        start_angle = math.atan2(start_v - center_v, start_u - center_u)
        end_angle = math.atan2(end_v - center_v, end_u - center_u)
        full_circle = math.hypot(end_u - start_u, end_v - start_v) <= 1e-9
        sweep = _directed_sweep(start_angle, end_angle, clockwise, full_circle)

    turns = parameters.get("p", 1.0)
    if turns < 1.0 or abs(turns - round(turns)) > 1e-9:
        raise ValueError("Arc turn count P must be a positive integer.")
    sweep += (-1.0 if clockwise else 1.0) * math.tau * (int(round(turns)) - 1)

    planar_length = abs(sweep) * radius
    linear_delta = target[w_axis] - start[w_axis]
    arc_length = math.hypot(planar_length, linear_delta)
    if radius <= ARC_CHORD_TOLERANCE_MM:
        maximum_angle = math.pi / 8.0
    else:
        maximum_angle = 2.0 * math.acos(
            max(-1.0, min(1.0, 1.0 - ARC_CHORD_TOLERANCE_MM / radius))
        )
    segment_count = max(
        1,
        math.ceil(abs(sweep) / max(maximum_angle, 1e-6)),
        math.ceil(arc_length / ARC_MAX_SEGMENT_MM),
    )
    if segment_count > MAX_ARC_SEGMENTS:
        raise ValueError("The arc requires too many interpolation segments.")

    fractions = {index / segment_count for index in range(1, segment_count + 1)}
    lower_angle = min(start_angle, start_angle + sweep)
    upper_angle = max(start_angle, start_angle + sweep)
    first_cardinal = math.floor(lower_angle / (math.pi / 2.0)) - 1
    last_cardinal = math.ceil(upper_angle / (math.pi / 2.0)) + 1
    for cardinal_index in range(first_cardinal, last_cardinal + 1):
        cardinal_angle = cardinal_index * math.pi / 2.0
        fraction = (cardinal_angle - start_angle) / sweep
        if 1e-12 < fraction < 1.0 - 1e-12:
            fractions.add(fraction)
    if len(fractions) > MAX_ARC_SEGMENTS + 8:
        raise ValueError("The arc requires too many interpolation segments.")

    points: list[dict[str, float]] = []
    ordered_fractions = sorted(fractions)
    for index, fraction in enumerate(ordered_fractions, start=1):
        angle = start_angle + sweep * fraction
        point = target.copy()
        point[u_axis] = center_u + radius * math.cos(angle)
        point[v_axis] = center_v + radius * math.sin(angle)
        point[w_axis] = start[w_axis] + linear_delta * fraction
        point["e"] = start["e"] + (target["e"] - start["e"]) * fraction
        if index == len(ordered_fractions):
            point = target.copy()
        point["_fraction"] = fraction
        points.append(point)
    return points, arc_length


def parse_gcode(
    filename: object,
    content: object,
    *,
    maximum_preview_segments: int = MAX_PREVIEW_SEGMENTS,
    maximum_timeline_events: int = MAX_TIMELINE_EVENTS,
) -> dict[str, Any]:
    """Parse one G-code file without saving it or executing any command."""

    if (
        isinstance(maximum_preview_segments, bool)
        or not isinstance(maximum_preview_segments, int)
        or maximum_preview_segments < 2
        or maximum_preview_segments > 300_000
    ):
        raise ValueError("maximum_preview_segments must be between 2 and 300000")
    if (
        isinstance(maximum_timeline_events, bool)
        or not isinstance(maximum_timeline_events, int)
        or maximum_timeline_events < 2
        or maximum_timeline_events > 300_000
    ):
        raise ValueError("maximum_timeline_events must be between 2 and 300000")
    safe_name = _clean_filename(filename)
    raw_bytes, text = _decode_content(content)
    lines = text.splitlines()

    # logical_position follows G92 and absolute/relative programming.  The
    # physical position is a stable source frame used by the UR conversion.
    logical_position = {"x": 0.0, "y": 0.0, "z": 0.0, "e": 0.0}
    physical_position = {"x": 0.0, "y": 0.0, "z": 0.0, "e": 0.0}
    minimum = {"x": 0.0, "y": 0.0, "z": 0.0}
    maximum = {"x": 0.0, "y": 0.0, "z": 0.0}
    print_minimum: dict[str, float] | None = None
    print_maximum: dict[str, float] | None = None
    execution_minimum: dict[str, float] | None = None
    execution_maximum: dict[str, float] | None = None
    execution_final_minimum: dict[str, float] | None = None
    execution_final_maximum: dict[str, float] | None = None
    execution_first_sequence: int | None = None
    execution_last_sequence: int | None = None
    xyz_absolute = True
    extruder_absolute = True
    arc_center_absolute = False
    arc_plane = "G17"
    modal_motion: str | None = None
    units_factor = 1.0
    feed_mm_min: float | None = None
    speed_factor = 1.0
    extrusion_factor = 1.0
    current_tool = 0
    tool_e_positions: dict[int, float] = {0: 0.0}

    command_counts: Counter[str] = Counter()
    tool_ids: set[int] = set()
    explicit_layers: set[int] = set()
    printed_z_values: set[float] = set()
    issues_by_key: dict[tuple[str, str], dict[str, Any]] = {}
    preparation_commands: list[dict[str, Any]] = []
    preview = _BoundedPreview(maximum_preview_segments)
    timeline_events: list[dict[str, Any]] = []
    timeline_stride = 1
    timeline_source_count = 0
    last_timeline_event: dict[str, Any] | None = None

    command_count = 0
    comment_count = 0
    blank_line_count = 0
    move_count = 0
    print_move_count = 0
    travel_move_count = 0
    retraction_count = 0
    xyz_move_count = 0
    extrusion_move_count = 0
    combined_motion_count = 0
    ur_only_motion_count = 0
    klipper_only_motion_count = 0
    total_path_mm = 0.0
    print_path_mm = 0.0
    travel_path_mm = 0.0
    positive_extrusion_mm = 0.0
    estimated_duration_s = 0.0
    movements_without_feed = 0
    max_nozzle_temp_c: float | None = None
    max_bed_temp_c: float | None = None
    generator = ""

    def add_issue(code: str, severity: str, *, line: int | None = None, command: str = "") -> None:
        key = (code, command)
        existing = issues_by_key.get(key)
        if existing:
            existing["occurrences"] += 1
            return
        issue: dict[str, Any] = {"code": code, "severity": severity, "occurrences": 1}
        if line is not None:
            issue["line"] = line
        if command:
            issue["command"] = command
        issues_by_key[key] = issue

    def append_timeline(event: dict[str, Any]) -> None:
        nonlocal timeline_stride, timeline_source_count
        timeline_source_count += 1
        if timeline_source_count % timeline_stride == 0:
            timeline_events.append(event)
        if len(timeline_events) > maximum_timeline_events:
            timeline_events[:] = timeline_events[::2]
            timeline_stride *= 2

    for line_number, raw_line in enumerate(lines, start=1):
        layer_match = _LAYER_RE.search(raw_line)
        if layer_match:
            explicit_layers.add(int(layer_match.group(1)))

        semicolon_index = raw_line.find(";")
        if semicolon_index >= 0:
            semicolon_comment = raw_line[semicolon_index + 1 :].strip()
            code_text = raw_line[:semicolon_index]
            comment_count += 1
            if not generator:
                generated_match = _GENERATED_BY_RE.search(semicolon_comment)
                if generated_match:
                    generator = generated_match.group(1).strip()[:120]
        else:
            code_text = raw_line
        if "(" in code_text and ")" in code_text:
            comment_count += len(_PAREN_COMMENT_RE.findall(code_text))
            code_text = _PAREN_COMMENT_RE.sub(" ", code_text)
        code_text = code_text.split("*", 1)[0].strip()
        if not code_text:
            blank_line_count += 1
            continue

        command_match = _COMMAND_RE.match(code_text)
        words: list[tuple[str, float]] = []
        invalid_number = False
        for match in _WORD_RE.finditer(code_text):
            name = match.group("name").lower()
            try:
                value = float(match.group("value"))
            except ValueError:
                invalid_number = True
                continue
            if not math.isfinite(value):
                invalid_number = True
                continue
            # A leading N word is a sequence number, never a parameter.
            if name == "n" and not words:
                continue
            words.append((name, value))
        if invalid_number:
            add_issue("non_finite_parameter", "error", line=line_number)

        numeric_commands = [
            _normalize_numeric_command(name, value)
            for name, value in words
            if name in {"g", "m", "t"}
        ]
        extended_command = ""
        if command_match:
            first = command_match.group("command").replace(" ", "").upper()
            axis_only_prefix = (
                len(first) == 1
                and first.lower() in {name for name, _value in words}
                and first.lower() not in {"g", "m", "n", "t"}
            )
            if not axis_only_prefix and not re.fullmatch(
                r"[A-Z][-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:E[-+]?\d+)?",
                first,
            ):
                extended_command = first

        parameters: dict[str, float] = {
            name: value for name, value in words if name not in {"g", "m", "n", "t"}
        }
        commands = numeric_commands[:]
        if extended_command:
            commands.insert(0, extended_command)
        explicit_motion = next(
            (command for command in commands if command in {"G0", "G1", "G2", "G3"}),
            None,
        )
        if explicit_motion:
            modal_motion = explicit_motion
        elif not commands and any(axis in parameters for axis in ("x", "y", "z", "e")):
            explicit_motion = modal_motion
            if explicit_motion:
                commands = [explicit_motion]

        if not commands:
            continue
        command_count += len(commands)
        command_counts.update(commands)

        for command in commands:
            if command in _DANGEROUS_COMMANDS:
                add_issue("dangerous_command", "error", line=line_number, command=command)
            if command in _CONFIG_COMMANDS:
                add_issue("configuration_command", "error", line=line_number, command=command)
            if command in _EXTERNAL_PREPARATION_COMMANDS:
                preparation_commands.append(
                    {"command": command, "kind": "external_macro", "line": line_number}
                )
                add_issue("external_macro_not_expanded", "warning", line=line_number, command=command)
            elif command and command[0].isalpha() and command[0] not in {"G", "M", "T"} and command not in _METADATA_COMMANDS:
                add_issue("extended_command_not_modelled", "warning", line=line_number, command=command)

        # Mode and state commands are applied in source order.
        for command in commands:
            if command == "G20":
                units_factor = 25.4
                add_issue("inch_units", "warning", line=line_number, command=command)
            elif command == "G21":
                units_factor = 1.0
            elif command == "G90":
                xyz_absolute = True
                # Marlin/Klipper coordinate semantics apply G90 to every
                # motion axis, including E.  An explicit M83 may override E
                # again, but a later G90/G91 clears that override.
                extruder_absolute = True
            elif command == "G91":
                xyz_absolute = False
                extruder_absolute = False
            elif command == "G90.1":
                arc_center_absolute = True
            elif command == "G91.1":
                arc_center_absolute = False
            elif command in {"G17", "G18", "G19"}:
                arc_plane = command
            elif command == "M82":
                extruder_absolute = True
            elif command == "M83":
                extruder_absolute = False
            elif command == "M220" and "s" in parameters:
                speed_factor = max(0.0, parameters["s"] / 100.0)
            elif command == "M221" and "s" in parameters:
                extrusion_factor = max(0.0, parameters["s"] / 100.0)
            elif command.startswith("T") and command[1:].isdigit():
                tool_e_positions[current_tool] = logical_position["e"]
                current_tool = int(command[1:])
                tool_ids.add(current_tool)
                logical_position["e"] = tool_e_positions.get(current_tool, 0.0)

        if "G28" in commands:
            preparation_commands.append({"command": "G28", "kind": "home_axes", "line": line_number})

        if "G92" in commands:
            for axis in ("x", "y", "z", "e"):
                if axis in parameters:
                    logical_position[axis] = parameters[axis] * units_factor
            tool_e_positions[current_tool] = logical_position["e"]

        if "G4" in commands:
            if "p" in parameters:
                estimated_duration_s += max(0.0, parameters["p"] / 1000.0)
            elif "s" in parameters:
                estimated_duration_s += max(0.0, parameters["s"])

        if any(command in {"G10", "G11"} for command in commands):
            add_issue("firmware_retraction_not_modelled", "error", line=line_number, command=next(command for command in commands if command in {"G10", "G11"}))

        for command in commands:
            if re.fullmatch(r"G-?\d+(?:\.\d+)?", command) and command not in {"G0", "G1", "G2", "G3"} | _KNOWN_NON_MOTION_G_CODES:
                severity = "error" if any(axis in parameters for axis in ("x", "y", "z", "e")) else "warning"
                add_issue("unsupported_gcode_command", severity, line=line_number, command=command)
            if re.fullmatch(r"M-?\d+(?:\.\d+)?", command) and command not in _KNOWN_NON_MOTION_M_CODES and command not in _DANGEROUS_COMMANDS and command not in _METADATA_COMMANDS:
                add_issue("unmodelled_process_command", "warning", line=line_number, command=command)

        if any(axis in parameters for axis in ("a", "b", "c")) and explicit_motion:
            add_issue("unsupported_extruder_axis", "error", line=line_number, command=explicit_motion)

        if any(command in {"M104", "M109"} for command in commands) and "s" in parameters:
            value = parameters["s"]
            max_nozzle_temp_c = value if max_nozzle_temp_c is None else max(max_nozzle_temp_c, value)
        if any(command in {"M140", "M190"} for command in commands) and "s" in parameters:
            value = parameters["s"]
            max_bed_temp_c = value if max_bed_temp_c is None else max(max_bed_temp_c, value)

        command = explicit_motion
        if command is None:
            continue
        if "f" in parameters:
            feed_mm_min = parameters["f"] * units_factor

        start = physical_position.copy()
        target = physical_position.copy()
        target_logical = logical_position.copy()
        for axis in ("x", "y", "z"):
            if axis not in parameters:
                continue
            value = parameters[axis] * units_factor
            if xyz_absolute:
                delta = value - logical_position[axis]
                target_logical[axis] = value
            else:
                delta = value
                target_logical[axis] = logical_position[axis] + value
            target[axis] = physical_position[axis] + delta
        if "e" in parameters:
            value = parameters["e"] * units_factor
            if extruder_absolute:
                target_logical["e"] = value
                raw_de = value - logical_position["e"]
            else:
                target_logical["e"] = logical_position["e"] + value
                raw_de = value
            target["e"] = physical_position["e"] + raw_de
        else:
            raw_de = 0.0
        de = raw_de * extrusion_factor

        arc_points: list[dict[str, float]] | None = None
        if command in {"G2", "G3"}:
            try:
                arc_points, xyz_distance = _arc_points(
                    start,
                    target,
                    parameters,
                    units_factor=units_factor,
                    plane=arc_plane,
                    clockwise=command == "G2",
                    center_absolute=arc_center_absolute,
                )
            except ValueError:
                add_issue("invalid_arc_geometry", "error", line=line_number, command=command)
                physical_position = target
                logical_position = target_logical
                tool_e_positions[current_tool] = logical_position["e"]
                continue
        else:
            dx = target["x"] - start["x"]
            dy = target["y"] - start["y"]
            dz = target["z"] - start["z"]
            xyz_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            linear_point = target.copy()
            linear_point["_fraction"] = 1.0
            arc_points = [linear_point]

        effective_distance = max(xyz_distance, abs(de))
        if effective_distance <= 1e-12:
            physical_position = target
            logical_position = target_logical
            tool_e_positions[current_tool] = logical_position["e"]
            continue

        move_count += 1
        total_path_mm += xyz_distance
        extruding = de > 1e-9 and xyz_distance > 1e-9
        if extruding:
            print_move_count += 1
            print_path_mm += xyz_distance
            positive_extrusion_mm += de
            printed_z_values.add(round(target["z"], 4))
            if execution_first_sequence is None:
                execution_first_sequence = move_count
                execution_minimum = {
                    axis: start[axis] for axis in ("x", "y", "z")
                }
                execution_maximum = {
                    axis: start[axis] for axis in ("x", "y", "z")
                }
        else:
            travel_move_count += 1
            travel_path_mm += xyz_distance
        if de < -1e-9:
            retraction_count += 1

        move_duration_s = 0.0
        effective_feed = (feed_mm_min or 0.0) * speed_factor
        if effective_feed > 0.0:
            move_duration_s = effective_distance / effective_feed * 60.0
            estimated_duration_s += move_duration_s
        else:
            movements_without_feed += 1

        ur_active = xyz_distance > 1e-9
        klipper_active = abs(de) > 1e-9
        if ur_active:
            xyz_move_count += 1
        if de > 1e-9:
            extrusion_move_count += 1
        if ur_active and klipper_active:
            combined_motion_count += 1
        elif ur_active:
            ur_only_motion_count += 1
        elif klipper_active:
            klipper_only_motion_count += 1

        last_timeline_event = {
            "sequence": move_count,
            "line": line_number,
            "command": command,
            "duration_s": _rounded(move_duration_s, 5),
            "cumulative_s": _rounded(estimated_duration_s, 4),
            "ur_action": "print_path" if ur_active and de > 1e-9 else "travel" if ur_active else "hold",
            "ur_distance_mm": _rounded(xyz_distance, 4),
            "klipper_action": "extrude" if de > 1e-9 else "retract" if de < -1e-9 else "idle",
            "extrusion_delta_mm": _rounded(de, 5),
            "feed_mm_min": _rounded(effective_feed, 3) if effective_feed else None,
        }
        append_timeline(last_timeline_event)

        previous = start
        previous_fraction = 0.0
        for point in arc_points:
            fraction = float(point.pop("_fraction", 1.0))
            fraction_delta = fraction - previous_fraction
            point_de = de * fraction_delta
            point_duration = move_duration_s * fraction_delta
            point_cumulative = estimated_duration_s - move_duration_s * (1.0 - fraction)
            for axis in ("x", "y", "z"):
                minimum[axis] = min(minimum[axis], point[axis])
                maximum[axis] = max(maximum[axis], point[axis])
                if execution_minimum is not None and execution_maximum is not None:
                    execution_minimum[axis] = min(
                        execution_minimum[axis], previous[axis], point[axis]
                    )
                    execution_maximum[axis] = max(
                        execution_maximum[axis], previous[axis], point[axis]
                    )
            if extruding:
                if print_minimum is None or print_maximum is None:
                    print_minimum = {
                        axis: min(previous[axis], point[axis])
                        for axis in ("x", "y", "z")
                    }
                    print_maximum = {
                        axis: max(previous[axis], point[axis])
                        for axis in ("x", "y", "z")
                    }
                else:
                    for axis in ("x", "y", "z"):
                        print_minimum[axis] = min(
                            print_minimum[axis], previous[axis], point[axis]
                        )
                        print_maximum[axis] = max(
                            print_maximum[axis], previous[axis], point[axis]
                        )
            point_distance = math.sqrt(
                sum((point[axis] - previous[axis]) ** 2 for axis in ("x", "y", "z"))
            )
            if point_distance > 1e-12:
                preview.append(
                    {
                        "sequence": move_count,
                        "line": line_number,
                        "x0": _rounded(previous["x"]),
                        "y0": _rounded(previous["y"]),
                        "z0": _rounded(previous["z"]),
                        "x1": _rounded(point["x"]),
                        "y1": _rounded(point["y"]),
                        "z1": _rounded(point["z"]),
                        "kind": "print" if extruding else "travel",
                        "duration_s": _rounded(point_duration, 5),
                        "cumulative_s": _rounded(point_cumulative, 4),
                        "extrusion_delta_mm": _rounded(point_de, 5),
                    }
                )
            previous = point
            previous_fraction = fraction

        if extruding:
            assert execution_minimum is not None and execution_maximum is not None
            execution_final_minimum = dict(execution_minimum)
            execution_final_maximum = dict(execution_maximum)
            execution_last_sequence = move_count

        physical_position = target
        logical_position = target_logical
        tool_e_positions[current_tool] = logical_position["e"]

    if move_count == 0:
        add_issue("no_motion_commands", "error")
    elif print_move_count == 0:
        add_issue("no_extrusion_moves", "warning")
    if movements_without_feed:
        add_issue("missing_feed_rate", "warning")
        issues_by_key[("missing_feed_rate", "")]["occurrences"] = movements_without_feed
    if last_timeline_event and (not timeline_events or timeline_events[-1]["sequence"] != last_timeline_event["sequence"]):
        timeline_events.append(last_timeline_event)
        if len(timeline_events) > maximum_timeline_events:
            timeline_events.pop(-2)

    issues = list(issues_by_key.values())
    severity_order = {"error": 0, "warning": 1, "info": 2}
    issues.sort(key=lambda issue: (severity_order.get(issue["severity"], 3), issue.get("line", 10**12), issue["code"]))
    error_count = sum(issue["severity"] == "error" for issue in issues)
    warning_count = sum(issue["severity"] == "warning" for issue in issues)
    dimensions = {axis: _rounded(maximum[axis] - minimum[axis]) for axis in ("x", "y", "z")}
    placement_minimum = print_minimum if print_minimum is not None else minimum
    placement_maximum = print_maximum if print_maximum is not None else maximum
    placement_dimensions = {
        axis: _rounded(placement_maximum[axis] - placement_minimum[axis])
        for axis in ("x", "y", "z")
    }
    if execution_final_minimum is None or execution_final_maximum is None:
        execution_final_minimum = dict(minimum)
        execution_final_maximum = dict(maximum)
        execution_first_sequence = 1 if move_count else 0
        execution_last_sequence = move_count
        execution_source = "all_motion"
    else:
        execution_source = "first_to_last_print"
    assert execution_first_sequence is not None and execution_last_sequence is not None
    execution_dimensions = {
        axis: _rounded(
            execution_final_maximum[axis] - execution_final_minimum[axis]
        )
        for axis in ("x", "y", "z")
    }
    layer_count = len(explicit_layers) if explicit_layers else len(printed_z_values)
    top_commands = [
        {"command": command, "count": count}
        for command, count in command_counts.most_common(12)
    ]
    preview_segments = preview.public_segments()

    return {
        "file": {
            "name": safe_name,
            "size_bytes": len(raw_bytes),
            "sha256": hashlib.sha256(raw_bytes).hexdigest(),
            "line_count": len(lines),
            "generator": generator,
        },
        "status": "needs_attention" if error_count else "parsed",
        "safe_to_plan": error_count == 0,
        "summary": {
            "command_count": command_count,
            "move_count": move_count,
            "print_move_count": print_move_count,
            "travel_move_count": travel_move_count,
            "retraction_count": retraction_count,
            "xyz_move_count": xyz_move_count,
            "extrusion_move_count": extrusion_move_count,
            "combined_motion_count": combined_motion_count,
            "ur_only_motion_count": ur_only_motion_count,
            "klipper_only_motion_count": klipper_only_motion_count,
            "layer_count": layer_count,
            "tool_count": max(1, len(tool_ids)) if print_move_count else len(tool_ids),
            "estimated_duration_s": _rounded(estimated_duration_s, 2),
            "timing_model": "nominal_feed_without_acceleration",
            "timing_exact": False,
            "total_path_mm": _rounded(total_path_mm, 2),
            "print_path_mm": _rounded(print_path_mm, 2),
            "travel_path_mm": _rounded(travel_path_mm, 2),
            "extrusion_mm": _rounded(positive_extrusion_mm, 2),
            "max_nozzle_temp_c": _rounded(max_nozzle_temp_c, 1) if max_nozzle_temp_c is not None else None,
            "max_bed_temp_c": _rounded(max_bed_temp_c, 1) if max_bed_temp_c is not None else None,
            "blank_line_count": blank_line_count,
            "comment_count": comment_count,
            "error_count": error_count,
            "warning_count": warning_count,
        },
        "command_counts": dict(sorted(command_counts.items())),
        "bounds": {
            "minimum": {axis: _rounded(minimum[axis]) for axis in ("x", "y", "z")},
            "maximum": {axis: _rounded(maximum[axis]) for axis in ("x", "y", "z")},
            "dimensions": dimensions,
            "units": "mm",
            "placement": {
                "minimum": {
                    axis: _rounded(placement_minimum[axis])
                    for axis in ("x", "y", "z")
                },
                "maximum": {
                    axis: _rounded(placement_maximum[axis])
                    for axis in ("x", "y", "z")
                },
                "dimensions": placement_dimensions,
                "source": "print_path" if print_minimum is not None else "all_motion",
            },
            "execution": {
                "minimum": {
                    axis: _rounded(execution_final_minimum[axis])
                    for axis in ("x", "y", "z")
                },
                "maximum": {
                    axis: _rounded(execution_final_maximum[axis])
                    for axis in ("x", "y", "z")
                },
                "dimensions": execution_dimensions,
                "source": execution_source,
                "first_sequence": execution_first_sequence,
                "last_sequence": execution_last_sequence,
                "excluded_lead_in_move_count": max(
                    0, execution_first_sequence - 1
                ),
                "excluded_shutdown_move_count": max(
                    0, move_count - execution_last_sequence
                ),
            },
        },
        "preview": {
            "projection": "XY",
            "units": "mm",
            "segments": preview_segments,
            "source_segment_count": preview.source_count,
            "returned_segment_count": len(preview_segments),
            "sampled": preview.source_count > len(preview_segments),
            "continuous": True,
            "arc_chord_tolerance_mm": ARC_CHORD_TOLERANCE_MM,
            "maximum_simplification_deviation_mm": _rounded(preview.maximum_deviation_mm, 4),
        },
        "top_commands": top_commands,
        "motion_timeline": {
            "events": timeline_events,
            "source_event_count": timeline_source_count,
            "returned_event_count": len(timeline_events),
            "sampled": timeline_stride > 1,
            "timing_model": "nominal_feed_without_acceleration",
        },
        "preparation_commands": preparation_commands,
        "issues": issues,
        "parser_capabilities": {
            "linear_motion": ["G0", "G1", "modal-axis-lines"],
            "arc_motion": ["G2", "G3", "G17", "G18", "G19", "IJK", "R", "G90.1", "G91.1"],
            "coordinate_modes": ["G20", "G21", "G90", "G91", "G92"],
            "extrusion_modes": ["M82", "M83", "M221", "per-tool-E"],
            "unsupported_path_commands_blocked": True,
        },
    }
