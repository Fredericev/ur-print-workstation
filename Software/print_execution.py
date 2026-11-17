"""Fail-closed preparation for synchronized UR + Klipper print execution."""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspaces/ur_ros2_ws/scripts"))
from smooth_trajectory import compile_profile, slice_path, validate_bundle
from smooth_trajectory import POLICY as TRAJECTORY_POLICY
from execution_scope import validate_execution_scope as _validate_execution_scope
from execution_scope import estimate_minimum_duration_s
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspaces/ur_ros2_ws/src/ur_klipper_bridge"))
from ur_klipper_bridge.validation import (
    EXTRUSION_PROTOCOL, DEFAULT_EXTRUDER_MAX_SPEED,
    DEFAULT_EXTRUDER_MAX_ACCELERATION, DEFAULT_EXTRUDER_MAX_DISTANCE,
    minimum_extrusion_duration, validate_extruder, validate_native_execution,
)

try:
    from gcode_path_review import PLANNING_POLICY_VERSION
    from printhead_geometry import printhead_geometry_sha256
except ModuleNotFoundError:
    from Software.gcode_path_review import PLANNING_POLICY_VERSION
    from Software.printhead_geometry import printhead_geometry_sha256


UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
MAX_TRAJECTORY_POINTS = 1_000_000
MAX_INPUT_JOINT_STEP_RAD = 0.35
APPROACH_JOIN_TOLERANCE_RAD = 1.0e-6
MAX_BLOCK_DURATION_S = 5.0
MAX_BLOCK_KLIPPER_UNITS = 180.0
# UR3e passthrough consumes at 500 Hz.  This is a command-supply floor, not a
# per-G-code-block dwell; ordinary block markers share one continuous curve.
MIN_PROCESS_INTERVAL_S = 0.002
TOOL0_MOTION_TEST_DURATION_S = 30.0

PreparationProgressCallback = Callable[
    [str, float, Optional[int], Optional[int]], None
]


class PrintExecutionError(ValueError):
    """The reviewed job cannot be turned into a real execution package."""


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise PrintExecutionError(f"{name} must be a finite number.")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise PrintExecutionError(f"{name} must be a finite number.") from exc
    if not math.isfinite(number):
        raise PrintExecutionError(f"{name} must be a finite number.")
    return number


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _manifest_execution_id(payload: dict[str, Any]) -> str:
    """Bind every executable manifest field into one stable local identity."""

    identity_payload = {
        key: value for key, value in payload.items() if key != "execution_id"
    }
    canonical = json.dumps(
        identity_payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()[:16]


def read_trajectory_csv(path: Path, *, include_points: bool = False) -> dict[str, Any]:
    """Validate one full-resolution trajectory artifact without ROS."""

    if not path.is_file():
        raise PrintExecutionError(f"Trajectory artifact is missing: {path.name}.")
    points: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader)
            if tuple(header) != ("time_s", "path_progress", *UR_JOINTS):
                raise PrintExecutionError(
                    f"Trajectory {path.name} has an invalid joint header."
                )
            previous_time = -1.0
            previous_progress = -1.0
            previous_positions: list[float] | None = None
            maximum_step = 0.0
            for row_index, row in enumerate(reader, start=2):
                if len(row) != 8:
                    raise PrintExecutionError(
                        f"Trajectory {path.name} row {row_index} is invalid."
                    )
                time_s = _finite(row[0], f"trajectory row {row_index} time")
                path_progress = _finite(
                    row[1], f"trajectory row {row_index} Cartesian progress"
                )
                positions = [
                    _finite(value, f"trajectory row {row_index} joint")
                    for value in row[2:]
                ]
                if time_s < 0.0 or time_s <= previous_time:
                    raise PrintExecutionError(
                        f"Trajectory {path.name} timestamps are not strictly increasing."
                    )
                if (
                    path_progress < 0.0
                    or path_progress > 1.0
                    or path_progress < previous_progress
                ):
                    raise PrintExecutionError(
                        f"Trajectory {path.name} Cartesian progress is invalid."
                    )
                if previous_positions is not None:
                    maximum_step = max(
                        maximum_step,
                        max(
                            abs(value - previous_positions[index])
                            for index, value in enumerate(positions)
                        ),
                    )
                points.append(
                    {
                        "time_s": time_s,
                        "path_progress": path_progress,
                        "positions": positions,
                    }
                )
                previous_time = time_s
                previous_progress = path_progress
                previous_positions = positions
                if len(points) > MAX_TRAJECTORY_POINTS:
                    raise PrintExecutionError(
                        f"Trajectory {path.name} exceeds the point limit."
                    )
    except (OSError, UnicodeDecodeError, csv.Error) as exc:
        raise PrintExecutionError(
            f"Could not read trajectory {path.name}: {exc}"
        ) from exc
    if len(points) < 2:
        raise PrintExecutionError(
            f"Trajectory {path.name} needs at least two points."
        )
    if points[0]["path_progress"] > 1.0e-9 or points[-1]["path_progress"] < 1.0 - 1.0e-9:
        raise PrintExecutionError(
            f"Trajectory {path.name} does not cover Cartesian progress 0 through 1."
        )
    if maximum_step > MAX_INPUT_JOINT_STEP_RAD:
        raise PrintExecutionError(
            f"Trajectory {path.name} contains a {maximum_step:.4f} rad joint "
            "jump, above the reviewed execution limit."
        )
    return {
        **({"points": points} if include_points else {}),
        "path": str(path.resolve()),
        "sha256": _sha256(path),
        "joint_names": list(UR_JOINTS),
        "point_count": len(points),
        "duration_s": round(points[-1]["time_s"], 6),
        "maximum_joint_step_rad": round(maximum_step, 6),
        "start_positions": points[0]["positions"],
        "end_positions": points[-1]["positions"],
    }


def _process_kind(segment: dict[str, Any]) -> str:
    action = str(segment.get("klipper_action", "")).casefold()
    if action == "extrude":
        return "print"
    if action == "retract":
        return "retract"
    if action == "idle":
        return "travel"
    if segment.get("kind") == "print":
        return "print"
    units = _finite(segment.get("klipper_units", 0.0), "klipper_units")
    if units > 0.0:
        return "print"
    if units < 0.0:
        return "retract"
    return "travel"


def _execution_blocks(
    review: dict[str, Any],
    trajectory_duration_s: float,
) -> list[dict[str, Any]]:
    segments = review.get("path", {}).get("segments", [])
    if not segments:
        raise PrintExecutionError("The reviewed path contains no print segments.")
    timeline = review.get("timeline", {})
    events = timeline.get("events", [])
    if timeline.get("sampled") is True or not isinstance(events, list) or not events:
        raise PrintExecutionError(
            "Print Execution requires the complete unsampled motion timeline."
        )

    sequence_lengths: dict[int, float] = {}
    sequence_layers: dict[int, int] = {}
    for segment in segments:
        sequence = int(segment.get("sequence", 0))
        distance = math.sqrt(
            sum(
                (
                    _finite(segment[f"{axis}1"], f"segment {axis}1")
                    - _finite(segment[f"{axis}0"], f"segment {axis}0")
                )
                ** 2
                for axis in ("x", "y", "z")
            )
        )
        sequence_lengths[sequence] = sequence_lengths.get(sequence, 0.0) + distance
        sequence_layers[sequence] = int(segment.get("layer", 0))
    total_distance = sum(sequence_lengths.values())
    if total_distance <= 1.0e-12:
        raise PrintExecutionError("The reviewed UR path has zero Cartesian length.")

    blocks: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    completed_distance = 0.0

    for event in events:
        sequence = int(event.get("sequence", 0))
        event_distance = sequence_lengths.get(sequence, 0.0)
        reports_ur_motion = _finite(
            event.get("ur_distance_mm", 0.0), "timeline UR distance"
        ) > 1.0e-9
        if reports_ur_motion and event_distance <= 1.0e-12:
            raise PrintExecutionError(
                f"Timeline event at line {event.get('line', 0)} lost its full-resolution UR geometry."
            )
        start_progress = completed_distance / total_distance
        if reports_ur_motion:
            completed_distance += event_distance
        end_progress = completed_distance / total_distance
        kind = _process_kind(event)
        units = _finite(event.get("klipper_units", 0.0), "klipper units")
        has_ur_motion = end_progress > start_progress + 1.0e-12
        layer = sequence_layers.get(sequence, 0)
        nominal_duration = _finite(event.get("duration_s", 0.0), "event duration")
        seconds_per_m = (
            nominal_duration / event_distance
            if has_ur_motion and event_distance > 1.0e-12
            else None
        )
        flow_units_per_m = (
            units / event_distance
            if has_ur_motion and abs(units) > 1.0e-12 and event_distance > 1.0e-12
            else None
        )
        if nominal_duration < 0.0:
            raise PrintExecutionError("A G-code timeline event has negative duration.")
        proposed_duration = nominal_duration

        split = False
        if current is not None:
            split = current["kind"] != kind
            split = split or current["has_ur_motion"] != has_ur_motion
            split = split or current["duration_s"] + proposed_duration > MAX_BLOCK_DURATION_S
            split = split or abs(current["klipper_units"] + units) > MAX_BLOCK_KLIPPER_UNITS
            split = split or layer != current["layer"]
            for rate_name, rate_value in (
                ("seconds_per_m", seconds_per_m),
                ("flow_units_per_m", flow_units_per_m),
            ):
                current_rate = current[rate_name]
                if (current_rate is None) != (rate_value is None):
                    split = True
                elif (
                    current_rate is not None
                    and rate_value is not None
                    and not math.isclose(
                        float(current_rate),
                        float(rate_value),
                        rel_tol=0.002,
                        abs_tol=1.0e-9,
                    )
                ):
                    split = True
        if current is None or split:
            if current is not None:
                blocks.append(current)
            current = {
                "index": len(blocks),
                "kind": kind,
                "has_ur_motion": has_ur_motion,
                "layer": layer,
                "source_line_start": int(event.get("line", 0)),
                "source_line_end": int(event.get("line", 0)),
                "start_progress": max(0.0, min(1.0, start_progress)),
                "end_progress": max(0.0, min(1.0, end_progress)),
                "duration_s": proposed_duration,
                "klipper_units": units,
                "seconds_per_m": seconds_per_m,
                "flow_units_per_m": flow_units_per_m,
                "event_count": 1,
            }
        else:
            current["source_line_end"] = int(event.get("line", 0))
            current["end_progress"] = max(0.0, min(1.0, end_progress))
            current["duration_s"] += proposed_duration
            current["klipper_units"] += units
            current["event_count"] += 1
    if current is not None:
        blocks.append(current)
    if abs(completed_distance - total_distance) > max(1.0e-9, total_distance * 1.0e-6):
        raise PrintExecutionError(
            "The complete G-code timeline does not match the reviewed UR geometry."
        )

    # Blocks are synchronization markers inside one continuous trajectory, not
    # separately dispatched goals.  Never add the former 250 ms dwell to every
    # marker; only a genuinely stationary process interval needs a positive
    # duration, and the trajectory compiler owns the 2 ms controller floor.
    for index, block in enumerate(blocks):
        block["index"] = index
        block["duration_s"] = round(
            max(
                MIN_PROCESS_INTERVAL_S if not block["has_ur_motion"] else 0.0,
                float(block["duration_s"]),
            ),
            6,
        )
        block["start_progress"] = round(float(block["start_progress"]), 9)
        block["end_progress"] = round(float(block["end_progress"]), 9)
        block["klipper_units"] = round(float(block["klipper_units"]), 6)
        if block["seconds_per_m"] is not None:
            block["seconds_per_m"] = round(float(block["seconds_per_m"]), 9)
        if block["flow_units_per_m"] is not None:
            block["flow_units_per_m"] = round(float(block["flow_units_per_m"]), 9)
    return blocks


def _execution_scope(
    blocks: list[dict[str, Any]],
    operation_mode: str,
    execution_extent: str = "sample",
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Bound commissioning motion while preserving full-path validation.

    A Tool0 test is deliberately a short sample of the already validated full
    trajectory.  It ends only after a complete synchronization block and only
    after at least one positive extrusion block has been included. Production
    Print always retains every source block. An explicitly selected full cold
    test likewise retains every block; it never changes the operation mode.
    """

    if not isinstance(execution_extent, str) or execution_extent not in {"sample", "full"}:
        raise PrintExecutionError("The execution extent must be sample or full.")
    if not blocks:
        raise PrintExecutionError("The execution scope contains no motion blocks.")
    if operation_mode != "tool0_motion_test" or execution_extent == "full":
        if (
            abs(float(blocks[0]["start_progress"])) > 1.0e-9
            or abs(float(blocks[-1]["end_progress"]) - 1.0) > 1.0e-9
        ):
            raise PrintExecutionError("The full execution path is missing Cartesian coverage.")
        return blocks, {
            "type": ("full_cold_motion_test" if operation_mode == "tool0_motion_test"
                     else "full_source_path"),
            "full_path_validated": True,
            "requested_duration_s": None,
            "motion_duration_s": round(sum(block["duration_s"] for block in blocks), 3),
            "end_progress": 1.0,
            "source_line_end": int(blocks[-1]["source_line_end"]),
        }

    selected: list[dict[str, Any]] = []
    duration_s = 0.0
    includes_extrusion = False
    for block in blocks:
        selected.append(dict(block))
        duration_s += float(block["duration_s"])
        includes_extrusion = includes_extrusion or (
            block["kind"] == "print" and float(block["klipper_units"]) > 0.0
        )
        if duration_s >= TOOL0_MOTION_TEST_DURATION_S and includes_extrusion:
            break
    for index, block in enumerate(selected):
        block["index"] = index
    return selected, {
        "type": "bounded_cold_motion_sample",
        "full_path_validated": True,
        "requested_duration_s": TOOL0_MOTION_TEST_DURATION_S,
        "motion_duration_s": round(duration_s, 3),
        "end_progress": round(float(selected[-1]["end_progress"]), 9),
        "source_line_end": int(selected[-1]["source_line_end"]),
    }


def _append_timeline_point(
    path: list[dict[str, Any]],
    *,
    time_s: float,
    path_progress: float,
    positions: list[float],
    markers: list[str],
) -> None:
    point = {
        "time_s": float(time_s),
        "path_progress": float(path_progress),
        "positions": list(positions),
        "timeline_markers": list(markers),
    }
    if path and abs(path[-1]["time_s"] - point["time_s"]) <= 1.0e-12:
        if max(abs(a-b) for a,b in zip(path[-1]["positions"], point["positions"])) > 1.0e-9:
            raise PrintExecutionError("A process boundary maps to two different UR positions.")
        for marker in markers:
            if marker not in path[-1]["timeline_markers"]:
                path[-1]["timeline_markers"].append(marker)
        path[-1]["path_progress"] = point["path_progress"]
        return
    if path and point["time_s"] <= path[-1]["time_s"]:
        raise PrintExecutionError("The unified UR/Klipper timeline is not strictly increasing.")
    path.append(point)


def _indexed_slice_path(
    path: list[dict[str, Any]],
    progresses: list[float],
    start: float,
    end: float,
) -> list[dict[str, Any]]:
    """Clip one progress interval without rescanning the complete trajectory."""

    if not 0.0 <= start <= end <= 1.0:
        raise PrintExecutionError("Invalid trajectory progress interval.")

    def at(progress: float) -> dict[str, Any]:
        if progress <= progresses[0]:
            return dict(path[0])
        high = bisect.bisect_left(progresses, progress)
        if high >= len(path):
            return dict(path[-1])
        low = high - 1
        span = progresses[high] - progresses[low]
        fraction = 0.0 if span <= 0.0 else (progress - progresses[low]) / span
        return {
            "time_s": float(path[low]["time_s"])
            + fraction * (float(path[high]["time_s"]) - float(path[low]["time_s"])),
            "path_progress": progress,
            "positions": [
                first + fraction * (last - first)
                for first, last in zip(path[low]["positions"], path[high]["positions"])
            ],
        }

    first = at(start)
    if end - start <= 1.0e-12:
        return [
            {
                "time_s": 0.0,
                "path_progress": start,
                "positions": list(first["positions"]),
            },
            {
                "time_s": MIN_PROCESS_INTERVAL_S,
                "path_progress": end,
                "positions": list(first["positions"]),
            },
        ]
    interior_start = bisect.bisect_right(progresses, start)
    interior_end = bisect.bisect_left(progresses, end)
    return [
        first,
        *(dict(point) for point in path[interior_start:interior_end]),
        at(end),
    ]


def _continuous_process_path(
    print_points: list[dict[str, Any]],
    blocks: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Create one UR path whose only holds are explicit process-only events."""
    if len(print_points) < 2:
        raise PrintExecutionError("The reviewed print trajectory is incomplete.")
    progresses = [float(point["path_progress"]) for point in print_points]
    if any(right < left for left, right in zip(progresses, progresses[1:])):
        raise PrintExecutionError("The reviewed print trajectory progress is not monotonic.")
    timeline: list[dict[str, Any]] = []
    cursor = 0.0
    for index, block in enumerate(blocks):
        start = float(block["start_progress"])
        end = float(block["end_progress"])
        duration = float(block["duration_s"])
        start_marker = f"block:{index}:start"
        end_marker = f"block:{index}:end"
        section = _indexed_slice_path(print_points, progresses, start, end)
        if end > start + 1.0e-12:
            source_start = float(section[0]["time_s"])
            source_end = float(section[-1]["time_s"])
            source_span = source_end - source_start
            if source_span <= 0.0:
                raise PrintExecutionError("A moving process block has no source trajectory time.")
            duration = max(
                source_span,
                duration,
                float(block.get("minimum_stepper_duration_s", 0.0)),
            )
            for point_index, point in enumerate(section):
                ratio = (float(point["time_s"])-source_start)/source_span
                markers = []
                if point_index == 0:
                    markers.append(start_marker)
                if point_index == len(section)-1:
                    markers.append(end_marker)
                _append_timeline_point(
                    timeline,
                    time_s=cursor + duration*ratio,
                    path_progress=float(point.get(
                        "path_progress", start + (end-start)*ratio
                    )),
                    positions=point["positions"],
                    markers=markers,
                )
        else:
            duration = max(duration, MIN_PROCESS_INTERVAL_S)
            position = section[0]["positions"]
            _append_timeline_point(
                timeline,
                time_s=cursor,
                path_progress=start,
                positions=position,
                markers=[start_marker],
            )
            _append_timeline_point(
                timeline,
                time_s=cursor + duration,
                path_progress=end,
                positions=position,
                markers=[end_marker],
            )
        cursor += duration
    if len(timeline) < 2:
        raise PrintExecutionError("The unified execution timeline has no motion duration.")
    return timeline


def _bind_blocks_to_profile(
    blocks: list[dict[str, Any]],
    profile: dict[str, Any],
) -> None:
    marker_times = {
        marker: float(point["time_s"])
        for point in profile["points"]
        for marker in point.get("timeline_markers", [])
    }
    cumulative_position = 0.0
    for index, block in enumerate(blocks):
        try:
            start = marker_times[f"block:{index}:start"]
            end = marker_times[f"block:{index}:end"]
        except KeyError as exc:
            raise PrintExecutionError("The continuous profile lost a process boundary.") from exc
        if end <= start:
            raise PrintExecutionError("A continuous process segment has no positive duration.")
        block["start_time_s"] = round(start, 6)
        block["end_time_s"] = round(end, 6)
        block["duration_s"] = round(end-start, 6)
        block["klipper_start_position"] = 0.0
        block["klipper_target_position"] = round(float(block["klipper_units"]), 6)
        cumulative_position += float(block["klipper_units"])
        block["klipper_cumulative_position"] = round(cumulative_position, 6)


def validate_execution_scope(manifest: dict[str, Any]) -> None:
    try:
        _validate_execution_scope(manifest)
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise PrintExecutionError(str(exc)) from exc


def prepare_execution_manifest(
    *,
    review: dict[str, Any],
    moveit_result: dict[str, Any],
    approach_trajectory_path: Path,
    print_trajectory_path: Path,
    planning_request_path: Path,
    planning_request_container_path: str | None = None,
    approach_container_path: str | None = None,
    print_container_path: str | None = None,
    destination: Path,
    manual_stepper: str | None = None,
    extruder: str | None = None,
    stepper_speed: float = 5.0,
    stepper_acceleration: float = 20.0,
    allow_simulation: bool = False,
    execution_extent: str = "sample",
    progress_callback: PreparationProgressCallback | None = None,
) -> dict[str, Any]:
    """Bind exact reviewed artifacts into an execution package."""

    def report(
        phase: str,
        progress: float,
        completed_units: int | None = None,
        total_units: int | None = None,
    ) -> None:
        if progress_callback is not None:
            progress_callback(
                phase,
                max(0.0, min(1.0, float(progress))),
                completed_units,
                total_units,
            )

    report("validating_review", 0.20)
    if not isinstance(execution_extent, str) or execution_extent not in {"sample", "full"}:
        raise PrintExecutionError("The execution extent must be sample or full.")
    if (review.get("tabletop_height_candidate") is True
            or review.get("print_z_candidate") is True):
        raise PrintExecutionError(
            "An unverified tabletop or Print Z candidate cannot authorize real motion."
        )
    if any(
        payload.get("planning_policy_version") != PLANNING_POLICY_VERSION
        for payload in (review, moveit_result)
    ):
        raise PrintExecutionError(
            "This review uses an obsolete planning policy. Validate the complete path again."
        )
    try:
        request_bytes = planning_request_path.read_bytes()
        request = json.loads(request_bytes)
    except (OSError, ValueError) as exc:
        raise PrintExecutionError("The current planning request cannot be verified.") from exc
    if (
        not isinstance(request, dict)
        or request.get("planning_policy_version") != PLANNING_POLICY_VERSION
        or request.get("review_sha256") != review.get("review_sha256")
        or not request.get("review_id")
        or request.get("review_id") != moveit_result.get("planning_request_id")
    ):
        raise PrintExecutionError("This MoveIt review was superseded. Validate the current path again.")
    simulated = moveit_result.get("simulation") is True
    if moveit_result.get("interpolation_validation") != "passed":
        raise PrintExecutionError("MoveIt must pass final joint-interpolation validation before execution.")
    operation_mode = str(review.get("operation_mode", "production_print"))
    if operation_mode not in {
        "production_print",
        "tool0_motion_test",
    }:
        raise PrintExecutionError(
            "The reviewed operation mode is retired or invalid. Reopen Calibration "
            "and migrate it to Tool0 Motion Test before preparing execution."
        )
    # Historical production callers already execute the whole path. Preserve
    # that contract; the new extent selector only chooses cold-test coverage.
    if operation_mode == "production_print":
        execution_extent = "full"
    if operation_mode == "tool0_motion_test":
        table_plane_check = review.get("air_run_check", {})
        # Older reviews do not contain the measured table-plane check and must
        # never be promoted into a real motion-test package.
        if table_plane_check.get("passed") is not True:
            raise PrintExecutionError(
                "The Tool0 motion test has not passed the measured table-plane check."
            )
    if simulated and not allow_simulation:
        raise PrintExecutionError(
            "A simulated MoveIt result cannot authorize real Print Execution."
        )
    if (
        moveit_result.get("status") != "passed"
        or moveit_result.get("approach_status") != "passed"
        or moveit_result.get("collision_free") is not True
        or _finite(moveit_result.get("cartesian_fraction", 0.0), "Cartesian fraction") < 1.0 - 1.0e-6
        or moveit_result.get("joint_limits") != "passed"
    ):
        raise PrintExecutionError(
            "MoveIt must pass the full collision-aware path and joint-limit review."
        )
    if moveit_result.get("review_sha256") != review.get("review_sha256"):
        raise PrintExecutionError(
            "The MoveIt result belongs to a different calibrated path."
        )
    if (review.get("printhead_geometry_sha256") != printhead_geometry_sha256()
            or moveit_result.get("printhead_geometry_sha256") != review.get("printhead_geometry_sha256")):
        raise PrintExecutionError("The installed printhead geometry changed. Validate and prepare the path again.")
    path = review.get("path", {})
    if path.get("sampled") is True or path.get("continuous") is not True:
        raise PrintExecutionError(
            "Print Execution requires the complete continuous source geometry."
        )
    try:
        native_extruder = validate_extruder(extruder if extruder is not None else manual_stepper)
    except ValueError as exc:
        raise PrintExecutionError("Select a native Klipper extruder and prepare again.") from exc
    maximum_stepper_speed = _finite(stepper_speed, "Klipper speed")
    stepper_acceleration_value = _finite(stepper_acceleration, "Klipper acceleration")
    if not 0.0 < maximum_stepper_speed <= DEFAULT_EXTRUDER_MAX_SPEED:
        raise PrintExecutionError("Native extruder speed must be above zero and at most 5 mm/s.")
    if stepper_acceleration_value != DEFAULT_EXTRUDER_MAX_ACCELERATION:
        raise PrintExecutionError("Native extruder acceleration must match the installed 20 mm/s².")

    report("loading_trajectories", 0.23)
    approach = read_trajectory_csv(approach_trajectory_path, include_points=True)
    print_trajectory = read_trajectory_csv(print_trajectory_path, include_points=True)
    approach_points = approach.pop("points")
    print_points = print_trajectory.pop("points")
    if max(
        abs(end - start)
        for end, start in zip(approach["end_positions"], print_trajectory["start_positions"])
    ) > APPROACH_JOIN_TOLERANCE_RAD:
        raise PrintExecutionError(
            "The approach end and print start joint positions differ. Replan the full path "
            "from the reviewed approach branch; old print trajectories cannot be spliced in."
        )
    approach["container_path"] = approach_container_path or approach["path"]
    print_trajectory["container_path"] = print_container_path or print_trajectory["path"]
    report("mapping_timeline", 0.28)
    blocks = _execution_blocks(review, print_trajectory["duration_s"])
    source_block_count = len(blocks)
    source_motion_line_end = int(blocks[-1]["source_line_end"])
    blocks, execution_scope = _execution_scope(blocks, operation_mode, execution_extent)
    source_nominal_gcode_duration_s = sum(
        float(block["duration_s"]) for block in blocks
    )
    extrusion_blocks = [block for block in blocks if block["kind"] != "travel"]
    if not extrusion_blocks or not any(
        block["kind"] == "print" and abs(block["klipper_units"]) > 0
        for block in extrusion_blocks
    ):
        raise PrintExecutionError("The execution package contains no printing extrusion event.")
    for block in blocks:
        units = float(block["klipper_units"])
        # Every native E-only G1 ramps from/to rest. Include all split moves'
        # ramps before compiling the shared UR timeline, never slow E alone.
        block["minimum_stepper_duration_s"] = (
            minimum_extrusion_duration(
                units, maximum_stepper_speed, stepper_acceleration_value,
                DEFAULT_EXTRUDER_MAX_DISTANCE,
            ) + 2.0e-6
            if abs(units) > 1.0e-12 else 0.0
        )
        if not block["has_ur_motion"]:
            block["duration_s"] = max(
                float(block["duration_s"]),
                block["minimum_stepper_duration_s"],
            )

    source_section = slice_path(
        print_points,
        float(blocks[0]["start_progress"]),
        float(blocks[-1]["end_progress"]),
    )
    source_moveit_duration_s = (
        float(source_section[-1]["time_s"])-float(source_section[0]["time_s"])
    )
    process_only_blocks = [block for block in blocks if not block["has_ur_motion"]]
    process_only_hold_duration_s = sum(
        float(block["duration_s"]) for block in process_only_blocks
    )

    # Compile the complete selected path once. Process blocks remain immutable
    # progress/extrusion markers, but are no longer independently dispatched
    # trajectories. Explicit process-only events insert real stationary holds;
    # ordinary print/travel boundaries retain the profile's shared q/v/a.
    timeline = _continuous_process_path(print_points, blocks)
    scheduled_timeline_duration_s = float(timeline[-1]["time_s"])
    schedule_floor_extension_s = max(
        0.0,
        scheduled_timeline_duration_s
        - source_moveit_duration_s
        - process_only_hold_duration_s,
    )
    report("certifying_print_curve", 0.36, 0, len(timeline))
    try:
        print_profile = compile_profile(
            timeline,
            progress_callback=lambda fraction, completed, total: report(
                "certifying_print_curve",
                0.36 + 0.54 * fraction,
                completed,
                total,
            ),
        )
    except (ValueError, KeyError, TypeError) as exc:
        raise PrintExecutionError(
            f"Cannot prepare one continuous bounded print trajectory: {exc}"
        ) from exc
    _bind_blocks_to_profile(blocks, print_profile)
    for block in blocks:
        units = abs(float(block["klipper_units"]))
        duration = float(block["duration_s"])
        if units > 1.0e-12 and duration + 1.0e-9 < minimum_extrusion_duration(
            units, maximum_stepper_speed, stepper_acceleration_value,
            DEFAULT_EXTRUDER_MAX_DISTANCE,
        ):
            raise PrintExecutionError(
                "The continuous UR timeline is too short for native extruder acceleration."
            )
    execution_scope["motion_duration_s"] = round(print_profile["duration_s"], 3)
    extrusion_blocks = [block for block in blocks if block["kind"] != "travel"]

    report("certifying_approach_curve", 0.91, 0, len(approach_points))
    try:
        approach_profile = compile_profile(
            approach_points,
            limit_class="approach",
            progress_callback=lambda fraction, completed, total: report(
                "certifying_approach_curve",
                0.91 + 0.05 * fraction,
                completed,
                total,
            ),
        )
    except (ValueError, KeyError, TypeError) as exc:
        raise PrintExecutionError(f"Cannot prepare a bounded smooth approach: {exc}") from exc

    timing_audit = {
        "source_nominal_gcode_duration_s": round(
            source_nominal_gcode_duration_s, 6
        ),
        "source_moveit_duration_s": round(source_moveit_duration_s, 6),
        "process_only_hold_count": len(process_only_blocks),
        "process_only_hold_duration_s": round(process_only_hold_duration_s, 6),
        "source_plus_holds_duration_s": round(
            source_moveit_duration_s+process_only_hold_duration_s, 6
        ),
        "scheduled_timeline_duration_s": round(
            scheduled_timeline_duration_s, 6
        ),
        "schedule_floor_extension_s": round(schedule_floor_extension_s, 6),
        "certified_print_duration_s": round(print_profile["duration_s"], 6),
        "certified_retiming_overhead_s": round(
            max(0.0, print_profile["duration_s"] - scheduled_timeline_duration_s),
            6,
        ),
        "source_approach_duration_s": round(approach["duration_s"], 6),
        "certified_approach_duration_s": round(approach_profile["duration_s"], 6),
        "ordinary_block_stop_count": 0,
        "controller_update_rate_hz": 500,
        "minimum_segment_duration_s": MIN_PROCESS_INTERVAL_S,
        "stepper_speed_cap_units_s": maximum_stepper_speed,
        "extruder_acceleration_mm_s2": stepper_acceleration_value,
        "extruder_junction_behavior": "rest_to_rest_per_native_E_move",
    }

    report("writing_package", 0.97)
    manifest = {
        "version": 3,
        "trajectory_policy_version": TRAJECTORY_POLICY,
        "trajectory_profiles": {"approach": approach_profile, "print": print_profile},
        "timing_audit": timing_audit,
        "planning_policy_version": PLANNING_POLICY_VERSION,
        "planning_request": {
            "path": str(planning_request_path.resolve()),
            "container_path": planning_request_container_path or str(planning_request_path.resolve()),
            "sha256": hashlib.sha256(request_bytes).hexdigest(),
            "review_id": request["review_id"],
        },
        "operation_mode": operation_mode,
        "execution_extent": execution_extent,
        "source_block_count": source_block_count,
        "source_motion_line_end": source_motion_line_end,
        # This executor intentionally owns motion channels only.  It never
        # forwards slicer heater, fan, homing, or preparation commands.
        "heating_enabled": False,
        "source": review["source"],
        "source_sha256": review["source_sha256"],
        "profile_sha256": review["profile_sha256"],
        "printhead_geometry_sha256": review["printhead_geometry_sha256"],
        "installed_profile_sha256": review.get(
            "installed_profile_sha256", review["profile_sha256"]
        ),
        "review_sha256": review["review_sha256"],
        "coordinate_frame": "base_link",
        "end_effector_link": "tool0",
        "extrusion_protocol": EXTRUSION_PROTOCOL,
        "extruder": native_extruder,
        "manual_stepper": native_extruder,
        "extruder_config": {
            "name": native_extruder,
            "nozzle_diameter": 0.2,
            "filament_diameter": 1.75,
            "rotation_distance": 10.0,
            "min_extrude_temp": 170.0,
        },
        "stepper_limits": {
            "maximum_speed": maximum_stepper_speed,
            "acceleration": stepper_acceleration_value,
            "max_extrude_only_distance": DEFAULT_EXTRUDER_MAX_DISTANCE,
        },
        "simulation": simulated,
        "print_bed_m": review.get("collision_bed_m", review["print_bed_m"]),
        "print_zero_z_m": review.get("placement_model", {}).get(
            "print_zero_z_m", review["print_bed_m"]["plane_z"]
        ),
        "print_bed_check": review["print_bed_check"],
        "tool_center_point_m": review["calibration"]["tool_center_point_m"],
        "approach_trajectory": approach,
        "print_trajectory": print_trajectory,
        "execution_scope": execution_scope,
        "coordination": {
            "method": "single_continuous_ur_goal_and_native_extruder_program",
            "synchronized_start_count": 1,
            "physical_sync_verified": False,
        },
        "blocks": blocks,
        "block_count": len(blocks),
        "extrusion_block_count": len(extrusion_blocks),
        "planned_duration_s": round(
            approach_profile["duration_s"] + print_profile["duration_s"],
            3,
        ),
        "execution_locked": True,
        "operator_action_required": "execute",
        "pause_semantics": "unsupported_during_continuous_job_use_stop",
        "stop_semantics": "coordinated_cancel_with_post_dispatch_klipper_emergency_stop",
    }
    try:
        validate_bundle(manifest)
        validate_execution_scope(manifest)
    except (ValueError, KeyError, TypeError) as exc:
        raise PrintExecutionError(f"Cannot prepare a consistent smooth execution package: {exc}") from exc
    manifest["execution_id"] = _manifest_execution_id(manifest)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    temporary.replace(destination)
    report("complete", 1.0)
    return manifest


def read_execution_manifest(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PrintExecutionError(f"Could not read the execution package: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("version") != 3:
        raise PrintExecutionError("The execution package has an invalid version.")
    if payload.get("planning_policy_version") != PLANNING_POLICY_VERSION:
        raise PrintExecutionError("This package uses an obsolete planning policy. Validate and prepare again.")
    if payload.get("extrusion_protocol") != EXTRUSION_PROTOCOL:
        raise PrintExecutionError("This package predates native extrusion; validate and prepare again.")
    try:
        validate_extruder(payload.get("extruder"))
    except ValueError as exc:
        raise PrintExecutionError("Execution package has no native extruder.") from exc
    if payload.get("printhead_geometry_sha256") != printhead_geometry_sha256():
        raise PrintExecutionError("The installed printhead geometry changed. Validate and prepare again.")
    if payload.get("execution_id") != _manifest_execution_id(payload):
        raise PrintExecutionError("The execution package changed after preparation.")
    try:
        validate_bundle(payload)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise PrintExecutionError(f"Smooth trajectory package is invalid: {exc}") from exc
    if payload.get("operation_mode") not in {
        "production_print",
        "tool0_motion_test",
    }:
        raise PrintExecutionError(
            "This package uses a retired operation mode. Prepare a new package "
            "from Production Print or Tool0 Motion Test."
        )
    if not isinstance(payload.get("blocks"), list) or not payload["blocks"]:
        raise PrintExecutionError("The execution package contains no synchronized blocks.")
    validate_execution_scope(payload)
    try:
        validate_native_execution(payload)
    except (ValueError, KeyError, TypeError) as exc:
        raise PrintExecutionError(f"Native extrusion package is invalid: {exc}") from exc
    for artifact_key in ("planning_request", "approach_trajectory", "print_trajectory"):
        artifact = payload.get(artifact_key)
        if not isinstance(artifact, dict):
            raise PrintExecutionError(f"The {artifact_key} artifact is missing.")
        artifact_path = Path(str(artifact.get("path", "")))
        if not artifact_path.is_file() or _sha256(artifact_path) != artifact.get("sha256"):
            raise PrintExecutionError(f"The {artifact_key} artifact changed after review.")
    return payload
