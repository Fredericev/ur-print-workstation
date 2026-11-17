"""Offline G-code-to-workcell calibration validation and persistence."""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from printhead_geometry import load_printhead_geometry, printhead_geometry_sha256
    from workcell_geometry import (
        center_gcode_on_workcell,
        fixed_print_bed_m,
        load_workcell_geometry,
    )
except ModuleNotFoundError:
    from Software.printhead_geometry import load_printhead_geometry, printhead_geometry_sha256
    from Software.workcell_geometry import (
        center_gcode_on_workcell,
        fixed_print_bed_m,
        load_workcell_geometry,
    )


_FIXED_WORKCELL = load_workcell_geometry()
_FIXED_PRINT_BED = fixed_print_bed_m(_FIXED_WORKCELL)
_PRINTHEAD = load_printhead_geometry()


DEFAULT_CALIBRATION: dict[str, Any] = {
    "version": 5,
    "placement_mode": "center_on_fixed_table",
    # Production printing uses a measured tool0-to-nozzle TCP.  The explicit
    # test mode instead treats tool0 itself as the process point so an operator
    # can evaluate UR + Klipper timing before the final print head is modeled.
    "operation_mode": "production_print",
    "printhead_geometry_sha256": printhead_geometry_sha256(_PRINTHEAD),
    "coordinate_transform": {
        "origin_m": {
            "x": _FIXED_WORKCELL["tabletop"]["center_m"]["x"],
            "y": _FIXED_WORKCELL["tabletop"]["center_m"]["y"],
            "z": _FIXED_WORKCELL["tabletop"]["surface_z_m"],
        },
        "rotation_z_deg": 0.0,
        "scale_m_per_mm": 0.001,
        "tool_orientation_rpy_deg": {
            "roll": 180.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    },
    "klipper_extrusion": {
        "units_per_e_mm": 1.0,
        "direction": "positive",
    },
    "tool_center_point_m": {
        # Vector from tool0 to the material/nozzle tip, expressed in tool0.
        # CAD-derived starting estimate; physical measurement remains required.
        "offset": _PRINTHEAD["tool0_to_nozzle_m"],
        "collision_radius_m": _PRINTHEAD["nozzle_collision_radius_m"],
    },
    "print_bed_m": _FIXED_PRINT_BED,
    "confirmations": {
        "origin_measured": True,
        "orientation_verified": False,
        "tool_orientation_verified": False,
        "extrusion_ratio_measured": False,
        "tcp_measured": False,
        "print_bed_measured": True,
        "tool0_reference_verified": False,
    },
}

_AXES = ("x", "y", "z")
_SOURCE_HASH_LENGTH = 64
_MINIMUM_ORIGIN_DISTANCE_FROM_BASE_M = 0.1
_BED_BELOW_PLANE_TOLERANCE_M = 0.0005
_MINIMUM_TCP_LENGTH_M = 0.005
_MINIMUM_TOOL_COLLISION_RADIUS_M = 0.001
_OPERATION_MODES = {
    "production_print",
    "tool0_motion_test",
}
_LEGACY_TABLETOP_MODE = "tool0_tabletop_test"


class CalibrationValidationError(ValueError):
    """The submitted calibration profile is malformed or outside hard limits."""


class CalibrationNotReadyError(ValueError):
    """The calibration is valid to inspect but is not ready to save."""


def _round(value: float, digits: int = 6) -> float:
    result = round(value, digits)
    return 0.0 if result == -0.0 else result


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CalibrationValidationError(f"{name} must be an object.")
    return value


def _number(
    value: object,
    name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool):
        raise CalibrationValidationError(f"{name} must be a number.")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationValidationError(f"{name} must be a number.") from exc
    if not math.isfinite(result):
        raise CalibrationValidationError(f"{name} must be finite.")
    if result < minimum or result > maximum:
        raise CalibrationValidationError(
            f"{name} must be between {minimum} and {maximum}."
        )
    return result


def _axis_values(
    value: object,
    name: str,
    *,
    minimum: float,
    maximum: float,
) -> dict[str, float]:
    mapping = _mapping(value, name)
    return {
        axis: _number(
            mapping.get(axis),
            f"{name}.{axis}",
            minimum=minimum,
            maximum=maximum,
        )
        for axis in _AXES
    }


def _xy_values(
    value: object,
    name: str,
    *,
    minimum: float,
    maximum: float,
) -> dict[str, float]:
    mapping = _mapping(value, name)
    return {
        axis: _number(
            mapping.get(axis),
            f"{name}.{axis}",
            minimum=minimum,
            maximum=maximum,
        )
        for axis in ("x", "y")
    }


def _confirmation(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise CalibrationValidationError(f"{name} must be true or false.")
    return value


def default_calibration() -> dict[str, Any]:
    """Return a fresh calibration template."""

    value = json.loads(json.dumps(DEFAULT_CALIBRATION))
    printhead = load_printhead_geometry()
    value["printhead_geometry_sha256"] = printhead_geometry_sha256(printhead)
    value["tool_center_point_m"] = {
        "offset": dict(printhead["tool0_to_nozzle_m"]),
        "collision_radius_m": printhead["nozzle_collision_radius_m"],
    }
    workcell = load_workcell_geometry()
    value["print_bed_m"] = fixed_print_bed_m(workcell)
    value["coordinate_transform"]["origin_m"]["z"] = workcell["tabletop"]["surface_z_m"]
    return value


def normalize_calibration(
    value: object,
    *,
    workcell: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate and normalize one calibration profile."""

    calibration = _mapping(value, "calibration")
    submitted_operation_mode = calibration.get(
        "operation_mode", DEFAULT_CALIBRATION["operation_mode"]
    )
    legacy_tabletop_profile = submitted_operation_mode == _LEGACY_TABLETOP_MODE
    if submitted_operation_mode not in _OPERATION_MODES and not legacy_tabletop_profile:
        raise CalibrationValidationError(
            "operation_mode must be production_print or tool0_motion_test."
        )
    operation_mode = (
        "tool0_motion_test" if legacy_tabletop_profile else submitted_operation_mode
    )
    geometry_digest = calibration.get("printhead_geometry_sha256", "")
    if geometry_digest:
        geometry_digest = normalize_source_sha256(geometry_digest)
    elif geometry_digest != "":
        raise CalibrationValidationError("printhead_geometry_sha256 must be a checksum or empty.")
    # Task placement now comes from the fixed workcell and parsed print bounds;
    # a captured tool0 pose can no longer override the table location.
    transform = _mapping(
        calibration.get("coordinate_transform"),
        "coordinate_transform",
    )
    extrusion = _mapping(
        calibration.get("klipper_extrusion"),
        "klipper_extrusion",
    )
    tcp = _mapping(
        calibration.get(
            "tool_center_point_m",
            DEFAULT_CALIBRATION["tool_center_point_m"],
        ),
        "tool_center_point_m",
    )
    # Physical workcell geometry is installed configuration, not job input.
    # Submitted or legacy bed values are deliberately ignored.
    bed = fixed_print_bed_m(load_workcell_geometry() if workcell is None else workcell)
    confirmations = _mapping(
        calibration.get("confirmations"),
        "confirmations",
    )

    origin = _axis_values(
        transform.get("origin_m"),
        "coordinate_transform.origin_m",
        minimum=-5.0,
        maximum=5.0,
    )
    rotation = _number(
        transform.get("rotation_z_deg"),
        "coordinate_transform.rotation_z_deg",
        minimum=-180.0,
        maximum=180.0,
    )
    scale = _number(
        transform.get("scale_m_per_mm"),
        "coordinate_transform.scale_m_per_mm",
        minimum=0.0001,
        maximum=0.01,
    )
    tool_orientation_value = transform.get("tool_orientation_rpy_deg")
    if tool_orientation_value is None:
        tool_orientation_value = DEFAULT_CALIBRATION["coordinate_transform"][
            "tool_orientation_rpy_deg"
        ]
    tool_orientation_mapping = _mapping(
        tool_orientation_value,
        "coordinate_transform.tool_orientation_rpy_deg",
    )
    tool_orientation = {
        axis: _number(
            tool_orientation_mapping.get(axis),
            f"coordinate_transform.tool_orientation_rpy_deg.{axis}",
            minimum=-180.0,
            maximum=180.0,
        )
        for axis in ("roll", "pitch", "yaw")
    }
    units_per_e_mm = _number(
        extrusion.get("units_per_e_mm"),
        "klipper_extrusion.units_per_e_mm",
        minimum=0.0001,
        maximum=100_000.0,
    )
    direction = extrusion.get("direction")
    if direction not in {"positive", "negative"}:
        raise CalibrationValidationError(
            "klipper_extrusion.direction must be positive or negative."
        )

    tcp_offset = _axis_values(
        tcp.get("offset"),
        "tool_center_point_m.offset",
        minimum=-1.0,
        maximum=1.0,
    )
    tcp_collision_radius = _number(
        tcp.get("collision_radius_m"),
        "tool_center_point_m.collision_radius_m",
        minimum=0.0,
        maximum=0.25,
    )

    bed_plane_z = _number(
        bed.get("plane_z"),
        "print_bed_m.plane_z",
        minimum=-5.0,
        maximum=5.0,
    )
    bed_minimum = _xy_values(
        bed.get("minimum"),
        "print_bed_m.minimum",
        minimum=-5.0,
        maximum=5.0,
    )
    bed_maximum = _xy_values(
        bed.get("maximum"),
        "print_bed_m.maximum",
        minimum=-5.0,
        maximum=5.0,
    )
    bed_edge_margin = _number(
        bed.get("edge_margin_m"),
        "print_bed_m.edge_margin_m",
        minimum=0.0,
        maximum=0.25,
    )

    origin_measured = True
    orientation_verified = _confirmation(
        confirmations.get("orientation_verified"),
        "confirmations.orientation_verified",
    )
    tool_orientation_verified = _confirmation(
        confirmations.get("tool_orientation_verified", False),
        "confirmations.tool_orientation_verified",
    )
    extrusion_ratio_measured = _confirmation(
        confirmations.get("extrusion_ratio_measured"),
        "confirmations.extrusion_ratio_measured",
    )
    tcp_measured = _confirmation(
        confirmations.get("tcp_measured", False),
        "confirmations.tcp_measured",
    )
    print_bed_measured = True
    tool0_reference_verified = _confirmation(
        confirmations.get("tool0_reference_verified", False),
        "confirmations.tool0_reference_verified",
    )
    if legacy_tabletop_profile:
        # The old mode's manually captured origin is ignored by fixed placement.
        # Clear confirmations whose meaning changed across the migration.
        tool_orientation_verified = False
        tool0_reference_verified = False
    if operation_mode == "production_print" and geometry_digest != printhead_geometry_sha256():
        # Preserve the recorded offsets for review, but an older tool's
        # measurements cannot authorize motion with the replacement printhead.
        tcp_measured = False
        tool_orientation_verified = False
        extrusion_ratio_measured = False

    return {
        "version": 5,
        "placement_mode": "center_on_fixed_table",
        "operation_mode": operation_mode,
        "printhead_geometry_sha256": geometry_digest,
        "coordinate_transform": {
            "origin_m": {axis: _round(origin[axis]) for axis in _AXES},
            "rotation_z_deg": _round(rotation),
            "scale_m_per_mm": _round(scale),
            "tool_orientation_rpy_deg": {
                axis: _round(tool_orientation[axis])
                for axis in ("roll", "pitch", "yaw")
            },
        },
        "klipper_extrusion": {
            "units_per_e_mm": _round(units_per_e_mm),
            "direction": direction,
        },
        "tool_center_point_m": {
            "offset": {axis: _round(tcp_offset[axis]) for axis in _AXES},
            "collision_radius_m": _round(tcp_collision_radius),
        },
        "print_bed_m": {
            "plane_z": _round(bed_plane_z),
            "minimum": {
                axis: _round(bed_minimum[axis]) for axis in ("x", "y")
            },
            "maximum": {
                axis: _round(bed_maximum[axis]) for axis in ("x", "y")
            },
            "edge_margin_m": _round(bed_edge_margin),
        },
        "confirmations": {
            "origin_measured": origin_measured,
            "orientation_verified": orientation_verified,
            "tool_orientation_verified": tool_orientation_verified,
            "extrusion_ratio_measured": extrusion_ratio_measured,
            "tcp_measured": tcp_measured,
            "print_bed_measured": print_bed_measured,
            "tool0_reference_verified": tool0_reference_verified,
        },
    }


def normalize_bounds(value: object) -> dict[str, Any]:
    """Validate the parser's millimetre XYZ bounds."""

    bounds = _mapping(value, "bounds")
    minimum = _axis_values(
        bounds.get("minimum"),
        "bounds.minimum",
        minimum=-1_000_000.0,
        maximum=1_000_000.0,
    )
    maximum = _axis_values(
        bounds.get("maximum"),
        "bounds.maximum",
        minimum=-1_000_000.0,
        maximum=1_000_000.0,
    )
    for axis in _AXES:
        if minimum[axis] > maximum[axis]:
            raise CalibrationValidationError(
                f"bounds.minimum.{axis} cannot exceed bounds.maximum.{axis}."
            )
    placement_value = bounds.get("placement")
    if placement_value is None:
        placement_minimum = dict(minimum)
        placement_maximum = dict(maximum)
        placement_source = "all_motion"
    else:
        placement = _mapping(placement_value, "bounds.placement")
        placement_minimum = _axis_values(
            placement.get("minimum"),
            "bounds.placement.minimum",
            minimum=-1_000_000.0,
            maximum=1_000_000.0,
        )
        placement_maximum = _axis_values(
            placement.get("maximum"),
            "bounds.placement.maximum",
            minimum=-1_000_000.0,
            maximum=1_000_000.0,
        )
        for axis in _AXES:
            if placement_minimum[axis] > placement_maximum[axis]:
                raise CalibrationValidationError(
                    f"bounds.placement.minimum.{axis} cannot exceed "
                    f"bounds.placement.maximum.{axis}."
                )
            if (
                placement_minimum[axis] < minimum[axis]
                or placement_maximum[axis] > maximum[axis]
            ):
                raise CalibrationValidationError(
                    "bounds.placement must stay inside the complete motion bounds."
                )
        placement_source = placement.get("source")
        if placement_source not in {"print_path", "all_motion"}:
            raise CalibrationValidationError(
                "bounds.placement.source must be print_path or all_motion."
            )
    execution_value = bounds.get("execution")
    if execution_value is None:
        execution_minimum = dict(minimum)
        execution_maximum = dict(maximum)
        execution_source = "all_motion"
        execution_first_sequence = 0
        execution_last_sequence = 0
        excluded_lead_in_move_count = 0
        excluded_shutdown_move_count = 0
    else:
        execution = _mapping(execution_value, "bounds.execution")
        execution_minimum = _axis_values(
            execution.get("minimum"),
            "bounds.execution.minimum",
            minimum=-1_000_000.0,
            maximum=1_000_000.0,
        )
        execution_maximum = _axis_values(
            execution.get("maximum"),
            "bounds.execution.maximum",
            minimum=-1_000_000.0,
            maximum=1_000_000.0,
        )
        for axis in _AXES:
            if execution_minimum[axis] > execution_maximum[axis]:
                raise CalibrationValidationError(
                    f"bounds.execution.minimum.{axis} cannot exceed "
                    f"bounds.execution.maximum.{axis}."
                )
            if (
                execution_minimum[axis] < minimum[axis]
                or execution_maximum[axis] > maximum[axis]
            ):
                raise CalibrationValidationError(
                    "bounds.execution must stay inside the complete motion bounds."
                )
        execution_source = execution.get("source")
        if execution_source not in {"first_to_last_print", "all_motion"}:
            raise CalibrationValidationError(
                "bounds.execution.source must be first_to_last_print or all_motion."
            )
        try:
            execution_first_sequence = int(execution.get("first_sequence", 0))
            execution_last_sequence = int(execution.get("last_sequence", 0))
            excluded_lead_in_move_count = int(
                execution.get("excluded_lead_in_move_count", 0)
            )
            excluded_shutdown_move_count = int(
                execution.get("excluded_shutdown_move_count", 0)
            )
        except (TypeError, ValueError) as exc:
            raise CalibrationValidationError(
                "bounds.execution sequence metadata must use integers."
            ) from exc
        if min(
            execution_first_sequence,
            execution_last_sequence,
            excluded_lead_in_move_count,
            excluded_shutdown_move_count,
        ) < 0 or (
            execution_last_sequence
            and execution_first_sequence > execution_last_sequence
        ):
            raise CalibrationValidationError(
                "bounds.execution sequence metadata is invalid."
            )
    return {
        "minimum": {axis: _round(minimum[axis], 4) for axis in _AXES},
        "maximum": {axis: _round(maximum[axis], 4) for axis in _AXES},
        "units": "mm",
        "placement": {
            "minimum": {
                axis: _round(placement_minimum[axis], 4) for axis in _AXES
            },
            "maximum": {
                axis: _round(placement_maximum[axis], 4) for axis in _AXES
            },
            "source": placement_source,
        },
        "execution": {
            "minimum": {
                axis: _round(execution_minimum[axis], 4) for axis in _AXES
            },
            "maximum": {
                axis: _round(execution_maximum[axis], 4) for axis in _AXES
            },
            "dimensions": {
                axis: _round(
                    execution_maximum[axis] - execution_minimum[axis], 4
                )
                for axis in _AXES
            },
            "source": execution_source,
            "first_sequence": execution_first_sequence,
            "last_sequence": execution_last_sequence,
            "excluded_lead_in_move_count": excluded_lead_in_move_count,
            "excluded_shutdown_move_count": excluded_shutdown_move_count,
        },
    }


def normalize_source_sha256(value: object) -> str:
    if not isinstance(value, str):
        raise CalibrationValidationError("source_sha256 is required.")
    digest = value.strip().lower()
    if (
        len(digest) != _SOURCE_HASH_LENGTH
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise CalibrationValidationError(
            "source_sha256 must be a 64-character hexadecimal checksum."
        )
    return digest


def _transform_point(
    point_mm: dict[str, float],
    calibration: dict[str, Any],
) -> dict[str, float]:
    transform = calibration["coordinate_transform"]
    origin = transform["origin_m"]
    scale = transform["scale_m_per_mm"]
    angle = math.radians(transform["rotation_z_deg"])
    cosine = math.cos(angle)
    sine = math.sin(angle)
    scaled_x = point_mm["x"] * scale
    scaled_y = point_mm["y"] * scale
    return {
        "x": origin["x"] + scaled_x * cosine - scaled_y * sine,
        "y": origin["y"] + scaled_x * sine + scaled_y * cosine,
        "z": origin["z"] + point_mm["z"] * scale,
    }


def _transformed_bounds(
    bounds: dict[str, Any],
    calibration: dict[str, Any],
) -> dict[str, Any]:
    corners = [
        {"x": x, "y": y, "z": z}
        for x in (bounds["minimum"]["x"], bounds["maximum"]["x"])
        for y in (bounds["minimum"]["y"], bounds["maximum"]["y"])
        for z in (bounds["minimum"]["z"], bounds["maximum"]["z"])
    ]
    transformed = [_transform_point(corner, calibration) for corner in corners]
    minimum = {
        axis: min(point[axis] for point in transformed) for axis in _AXES
    }
    maximum = {
        axis: max(point[axis] for point in transformed) for axis in _AXES
    }
    return {
        "minimum": {axis: _round(minimum[axis]) for axis in _AXES},
        "maximum": {axis: _round(maximum[axis]) for axis in _AXES},
        "dimensions": {
            axis: _round(maximum[axis] - minimum[axis]) for axis in _AXES
        },
        "units": "m",
    }


def _tool_rotation_matrix(calibration: dict[str, Any]) -> tuple[tuple[float, ...], ...]:
    orientation = calibration["coordinate_transform"]["tool_orientation_rpy_deg"]
    roll = math.radians(orientation["roll"])
    pitch = math.radians(orientation["pitch"])
    yaw = math.radians(orientation["yaw"])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def tool_tip_offset_in_base(calibration: dict[str, Any]) -> dict[str, float]:
    """Rotate the measured tool0-to-tip TCP vector into base_link."""

    if calibration.get("operation_mode") == "tool0_motion_test":
        return {axis: 0.0 for axis in _AXES}

    rotation = _tool_rotation_matrix(calibration)
    offset = calibration["tool_center_point_m"]["offset"]
    values = (offset["x"], offset["y"], offset["z"])
    return {
        axis: _round(sum(rotation[row][column] * values[column] for column in range(3)))
        for row, axis in enumerate(_AXES)
    }


def _shift_bounds(
    bounds: dict[str, Any],
    offset: dict[str, float],
    *,
    sign: float,
) -> dict[str, Any]:
    minimum = {
        axis: _round(bounds["minimum"][axis] + sign * offset[axis])
        for axis in _AXES
    }
    maximum = {
        axis: _round(bounds["maximum"][axis] + sign * offset[axis])
        for axis in _AXES
    }
    return {
        "minimum": minimum,
        "maximum": maximum,
        "dimensions": dict(bounds["dimensions"]),
        "units": "m",
    }


def _profile_hash(calibration: dict[str, Any]) -> str:
    canonical = json.dumps(
        calibration,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def validate_calibration(
    calibration: object,
    bounds: object,
    source_sha256: object,
    *,
    workcell_snapshot: dict[str, Any] | None = None,
    print_zero_z_m: float | None = None,
) -> dict[str, Any]:
    """Validate one offline calibration against a parsed G-code work volume."""

    workcell = load_workcell_geometry() if workcell_snapshot is None else workcell_snapshot
    normalized = normalize_calibration(calibration, workcell=workcell)
    normalized_bounds = normalize_bounds(bounds)
    placement = center_gcode_on_workcell(
        normalized_bounds,
        normalized["coordinate_transform"]["rotation_z_deg"],
        normalized["coordinate_transform"]["scale_m_per_mm"],
        workcell,
    )
    if print_zero_z_m is not None:
        try:
            print_zero = float(print_zero_z_m)
        except (TypeError, ValueError) as exc:
            raise CalibrationValidationError("Print Z origin must be a finite number.") from exc
        surface_z = float(workcell["tabletop"]["surface_z_m"])
        if not math.isfinite(print_zero) or not surface_z <= print_zero <= 2.0:
            raise CalibrationValidationError(
                "Print Z origin must be above the physical tabletop and within the bounded base_link workspace."
            )
        placement["origin_m"]["z"] = _round(print_zero)
    placement["print_zero_z_m"] = placement["origin_m"]["z"]
    placement["print_z_offset_m"] = _round(
        placement["origin_m"]["z"] - float(workcell["tabletop"]["surface_z_m"])
    )
    normalized["coordinate_transform"]["origin_m"] = dict(
        placement["origin_m"]
    )
    source_digest = normalize_source_sha256(source_sha256)
    execution_bounds = normalized_bounds["execution"]
    deposition_bounds = _transformed_bounds(execution_bounds, normalized)
    tcp_offset_base = tool_tip_offset_in_base(normalized)
    # MoveIt targets tool0 while the physical G-code coordinates describe the
    # material/nozzle tip.  tool0 = tip - R(tool0) * tcp_offset.
    transformed_bounds = _shift_bounds(
        deposition_bounds,
        tcp_offset_base,
        sign=-1.0,
    )
    print_bed = normalized["print_bed_m"]
    bed_dimensions = {
        axis: _round(print_bed["maximum"][axis] - print_bed["minimum"][axis])
        for axis in ("x", "y")
    }
    bed_dimensions_valid = all(
        bed_dimensions[axis] > 2.0 * print_bed["edge_margin_m"]
        for axis in ("x", "y")
    )
    bed_usable_minimum = {
        axis: _round(print_bed["minimum"][axis] + print_bed["edge_margin_m"])
        for axis in ("x", "y")
    }
    bed_usable_maximum = {
        axis: _round(print_bed["maximum"][axis] - print_bed["edge_margin_m"])
        for axis in ("x", "y")
    }
    bed_axis_checks: dict[str, dict[str, Any]] = {}
    for axis in ("x", "y"):
        actual_minimum = deposition_bounds["minimum"][axis]
        actual_maximum = deposition_bounds["maximum"][axis]
        allowed_minimum = bed_usable_minimum[axis]
        allowed_maximum = bed_usable_maximum[axis]
        inside = (
            bed_dimensions_valid
            and actual_minimum >= allowed_minimum
            and actual_maximum <= allowed_maximum
        )
        bed_axis_checks[axis] = {
            "inside": inside,
            "actual_minimum": actual_minimum,
            "actual_maximum": actual_maximum,
            "allowed_minimum": allowed_minimum,
            "allowed_maximum": allowed_maximum,
            "clearance_minimum_m": _round(actual_minimum - allowed_minimum),
            "clearance_maximum_m": _round(allowed_maximum - actual_maximum),
        }
    within_print_bed = bed_dimensions_valid and all(
        check["inside"] for check in bed_axis_checks.values()
    )
    path_minimum_z = deposition_bounds["minimum"]["z"]
    bed_surface_gap_m = _round(path_minimum_z - print_bed["plane_z"])
    # The operator may intentionally place the whole path above the tabletop
    # for an air run. The table remains an obstacle; proximity to it is not a
    # prerequisite for a valid print-path Z origin.
    print_bed_surface_aligned = (
        bed_surface_gap_m >= -_BED_BELOW_PLANE_TOLERANCE_M
    )
    confirmations = normalized["confirmations"]
    tool0_motion_test = normalized["operation_mode"] == "tool0_motion_test"
    tool0_reference_mode = tool0_motion_test
    confirmations_complete = (
        confirmations["origin_measured"]
        and confirmations["orientation_verified"]
        and confirmations["tool_orientation_verified"]
        and confirmations["extrusion_ratio_measured"]
        and (
            confirmations["tool0_reference_verified"]
            and confirmations["print_bed_measured"]
            if tool0_reference_mode
            else confirmations["tcp_measured"]
            and confirmations["print_bed_measured"]
        )
    )
    tcp_offset = normalized["tool_center_point_m"]["offset"]
    tcp_length_m = math.sqrt(sum(tcp_offset[axis] ** 2 for axis in _AXES))
    tcp_geometry_valid = (
        tcp_length_m >= _MINIMUM_TCP_LENGTH_M
        and normalized["tool_center_point_m"]["collision_radius_m"]
        >= _MINIMUM_TOOL_COLLISION_RADIUS_M
    )
    origin = normalized["coordinate_transform"]["origin_m"]
    origin_distance_from_base_m = math.sqrt(
        origin["x"] ** 2 + origin["y"] ** 2 + origin["z"] ** 2
    )
    origin_clear_of_robot_base = (
        origin_distance_from_base_m >= _MINIMUM_ORIGIN_DISTANCE_FROM_BASE_M
    )
    production_coordinate_ready = (
        confirmations["origin_measured"]
        and confirmations["orientation_verified"]
        and confirmations["tool_orientation_verified"]
        and confirmations["tcp_measured"]
        and tcp_geometry_valid
        and confirmations["print_bed_measured"]
        and origin_clear_of_robot_base
        and within_print_bed
        and print_bed_surface_aligned
    )
    # Tool0 Motion Test may intentionally place tool0 on the measured table
    # plane.  It has no additional positive-Z clearance requirement; the small
    # tolerance only absorbs measurement/rounding noise and still rejects a
    # path that is materially below the table.
    air_run_clearance_m = bed_surface_gap_m
    tool0_table_plane_valid = (
        air_run_clearance_m >= -_BED_BELOW_PLANE_TOLERANCE_M
    )
    tool0_table_plane_ready = (
        confirmations["print_bed_measured"] and tool0_table_plane_valid
    )
    tool0_test_coordinate_ready = (
        confirmations["origin_measured"]
        and confirmations["orientation_verified"]
        and confirmations["tool_orientation_verified"]
        and confirmations["print_bed_measured"]
        and confirmations["tool0_reference_verified"]
        and origin_clear_of_robot_base
        and within_print_bed
        and tool0_table_plane_ready
    )
    coordinate_ready = {
        "tool0_motion_test": tool0_test_coordinate_ready,
        "production_print": production_coordinate_ready,
    }[normalized["operation_mode"]]
    extrusion_ready = confirmations["extrusion_ratio_measured"]
    ready_to_save = coordinate_ready and extrusion_ready
    direction_sign = (
        1 if normalized["klipper_extrusion"]["direction"] == "positive" else -1
    )

    return {
        "status": "ready" if ready_to_save else "incomplete",
        "ready_to_save": ready_to_save,
        "operation_mode": normalized["operation_mode"],
        "production_ready": production_coordinate_ready and extrusion_ready,
        "motion_test_ready": tool0_test_coordinate_ready and extrusion_ready,
        "execution_locked": True,
        "source_sha256": source_digest,
        "profile_sha256": _profile_hash(normalized),
        "printhead_geometry_sha256": printhead_geometry_sha256(),
        "printhead_geometry_matches": normalized["printhead_geometry_sha256"] == printhead_geometry_sha256(),
        "placement": placement,
        "workcell_geometry": workcell,
        "calibration": normalized,
        "source_bounds_mm": normalized_bounds,
        "execution_bounds_mm": execution_bounds,
        "transformed_bounds_m": transformed_bounds,
        "deposition_bounds_m": deposition_bounds,
        "tcp_offset_base_m": tcp_offset_base,
        "tcp_check": {
            "measured": confirmations["tcp_measured"],
            "geometry_valid": True if tool0_reference_mode else tcp_geometry_valid,
            "reference": "tool0" if tool0_reference_mode else "measured_tcp",
            "length_m": 0.0 if tool0_reference_mode else _round(tcp_length_m),
            "minimum_length_m": _MINIMUM_TCP_LENGTH_M,
            "collision_radius_m": normalized["tool_center_point_m"][
                "collision_radius_m"
            ],
            "minimum_collision_radius_m": _MINIMUM_TOOL_COLLISION_RADIUS_M,
        },
        "within_print_bed": within_print_bed,
        "print_bed_surface_aligned": print_bed_surface_aligned,
        "air_run_check": {
            "required": False,
            "measured": confirmations["print_bed_measured"],
            "passed": tool0_table_plane_ready,
            "path_minimum_z": path_minimum_z,
            "guard_plane_z": print_bed["plane_z"],
            "clearance_m": air_run_clearance_m,
            "minimum_clearance_m": 0.0,
            "heating_allowed": False,
            "production_print_allowed": False,
        },
        "print_bed_check": {
            "measured": confirmations["print_bed_measured"],
            "dimensions_valid": bed_dimensions_valid,
            "dimensions_m": bed_dimensions,
            "plane_z": print_bed["plane_z"],
            "edge_margin_m": print_bed["edge_margin_m"],
            "usable_minimum": bed_usable_minimum,
            "usable_maximum": bed_usable_maximum,
            "axis_checks": bed_axis_checks,
            "path_minimum_z": path_minimum_z,
            "surface_gap_m": bed_surface_gap_m,
            "surface_aligned": print_bed_surface_aligned,
            "minimum_surface_gap_m": -_BED_BELOW_PLANE_TOLERANCE_M,
            "maximum_surface_gap_m": None,
        },
        "origin_clear_of_robot_base": origin_clear_of_robot_base,
        "origin_distance_from_base_m": _round(origin_distance_from_base_m),
        "minimum_origin_distance_from_base_m": (
            _MINIMUM_ORIGIN_DISTANCE_FROM_BASE_M
        ),
        "confirmations_complete": confirmations_complete,
        "extrusion_mapping": {
            "formula": "klipper_units = E_mm × units_per_e_mm × direction",
            "units_per_e_mm": normalized["klipper_extrusion"][
                "units_per_e_mm"
            ],
            "direction_sign": direction_sign,
        },
        "gates": [
            {
                "code": "coordinate_transform",
                "status": "passed" if coordinate_ready else "required",
            },
            {
                "code": "extrusion_calibration",
                "status": "passed" if extrusion_ready else "required",
            },
            {
            "code": "tool0_reference" if tool0_reference_mode else "tcp_geometry",
            "status": (
                "passed"
                    if tool0_reference_mode and confirmations["tool0_reference_verified"]
                    else "required"
                    if tool0_reference_mode
                    else "passed"
                    if confirmations["tcp_measured"] and tcp_geometry_valid
                    else "failed"
                    if confirmations["tcp_measured"]
                    else "required"
                ),
            },
            *(
                []
                if tool0_motion_test
                else [
                    {
                        "code": "print_bed",
                        "status": (
                            "passed"
                            if confirmations["print_bed_measured"]
                            and within_print_bed
                            and print_bed_surface_aligned
                            else "failed"
                            if confirmations["print_bed_measured"]
                            else "required"
                        ),
                    }
                ]
            ),
            {
                "code": "collision_check",
                "status": "required",
            },
        ],
    }


def print_z_candidate_ready_for_planning(validation: dict[str, Any]) -> bool:
    """Allow a height candidate to be reviewed, never executed, above the real table.

    A candidate may deliberately put the first layer more than 5 mm above the
    table. All installed-tool, XY, and below-table checks still apply. The
    ordinary ready_to_save gate remains authoritative for physical execution.
    """

    return bool(
        validation["operation_mode"] == "production_print"
        and validation["confirmations_complete"]
        and validation["tcp_check"]["geometry_valid"]
        and validation["within_print_bed"]
        and validation["origin_clear_of_robot_base"]
        and validation["print_bed_check"]["surface_gap_m"] >= 0.0
    )


def read_saved_calibration(path: Path) -> dict[str, Any] | None:
    """Read a persisted profile. Corrupt or incompatible data fails closed."""

    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationValidationError(
            f"Could not read the saved calibration profile: {exc}"
        ) from exc
    if not isinstance(payload, dict):
        raise CalibrationValidationError(
            "The saved calibration profile must be a JSON object."
        )
    calibration = normalize_calibration(payload.get("calibration"))
    source_sha256 = normalize_source_sha256(payload.get("source_sha256"))
    return {
        "saved_at": str(payload.get("saved_at", "")),
        "source_sha256": source_sha256,
        "profile_sha256": _profile_hash(calibration),
        "calibration": calibration,
        "printhead_geometry_matches": calibration["printhead_geometry_sha256"] == printhead_geometry_sha256(),
        "printhead_recalibration_required": (
            calibration["operation_mode"] == "production_print"
            and calibration["printhead_geometry_sha256"] != printhead_geometry_sha256()
        ),
        "printhead_migration": payload.get("printhead_migration"),
    }


def migrate_printhead_calibration(path: Path) -> dict[str, Any] | None:
    """Back up an older installed profile and revoke only its tool confirmations.

    This explicit migration retains the exact original bytes in a sibling
    backup and leaves the measured offsets available for comparison.
    """
    saved = read_saved_calibration(path)
    if saved is None or not saved["printhead_recalibration_required"]:
        return None
    original = path.read_bytes()
    payload = json.loads(original)
    current_digest = printhead_geometry_sha256()
    previous = payload.get("printhead_migration", {})
    if (isinstance(previous, dict) and previous.get("required_geometry_sha256") == current_digest
            and payload["calibration"]["confirmations"].get("tcp_measured") is False
            and payload["calibration"]["confirmations"].get("tool_orientation_verified") is False
            and payload["calibration"]["confirmations"].get("extrusion_ratio_measured") is False):
        return previous
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup = path.with_name(f"{path.stem}.before-printhead-{stamp}{path.suffix}")
    with backup.open("xb") as handle:
        handle.write(original)
    confirmations = payload["calibration"]["confirmations"]
    confirmations["tcp_measured"] = False
    confirmations["tool_orientation_verified"] = False
    confirmations["extrusion_ratio_measured"] = False
    record = {
        "reason": "installed_printhead_geometry_changed",
        "required_geometry_sha256": current_digest,
        "previous_geometry_sha256": payload["calibration"].get("printhead_geometry_sha256", ""),
        "backup_path": str(backup.resolve()),
        "migrated_at": datetime.now(timezone.utc).isoformat(),
    }
    payload["printhead_migration"] = record
    payload["profile_sha256"] = _profile_hash(normalize_calibration(payload["calibration"]))
    temporary = path.with_suffix(path.suffix + ".migration.tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return record


def save_calibration(
    path: Path,
    validation: dict[str, Any],
) -> dict[str, Any]:
    """Atomically persist a ready calibration profile."""

    if validation.get("ready_to_save") is not True:
        if validation.get("operation_mode") == "tool0_motion_test":
            message = (
                "Confirm the Tool0 reference and orientation, and keep the "
                "automatically centered path inside and on or above the fixed "
                "table before saving."
            )
        else:
            message = (
                "Complete the tool and extrusion confirmations, measure the nozzle "
                "TCP, and keep the automatically centered path inside the fixed "
                "table before saving."
            )
        raise CalibrationNotReadyError(
            message
        )
    return save_installed_calibration_draft(
        path, validation["calibration"], validation["source_sha256"]
    )


def save_installed_calibration_draft(
    path: Path,
    calibration: object,
    source_sha256: object,
) -> dict[str, Any]:
    """Persist a normalized setup that may remain locked until checked per G-code."""

    normalized = normalize_calibration(calibration)
    payload = {
        "version": 5,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "source_sha256": normalize_source_sha256(source_sha256),
        "profile_sha256": _profile_hash(normalized),
        "calibration": normalized,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return payload


def validate_relative_motion_over_print_bed(
    calibration: object,
    start_position_m: object,
    axes: object,
    distance_m: object,
) -> dict[str, Any]:
    """Fail closed unless a relative Cartesian test stays over the real bed.

    The synchronized test moves one Cartesian axis at a time from the live
    tool0 pose.  A rectangular print bed is convex, so checking the start and
    each cumulative endpoint also covers every point on
    the intervening straight segments.  This is a footprint/bounds guard, not
    a substitute for MoveIt collision checking or a measured TCP/tool model.
    """

    normalized = normalize_calibration(calibration)
    tool0_motion_test = normalized["operation_mode"] == "tool0_motion_test"
    if tool0_motion_test:
        if not normalized["confirmations"]["tool0_reference_verified"]:
            raise CalibrationNotReadyError(
                "Confirm that tool0 is the temporary process point before "
                "planning a Tool0 motion test."
            )
        if not normalized["confirmations"]["print_bed_measured"]:
            raise CalibrationNotReadyError(
                "Measure and confirm the table surface height before "
                "planning a Tool0 motion test."
            )
    elif not (
        normalized["confirmations"]["print_bed_measured"]
        and normalized["confirmations"]["tcp_measured"]
    ):
        raise CalibrationNotReadyError(
            "Measure and confirm the TCP and tool orientation before planning "
            "or executing Cartesian motion over the fixed tabletop."
        )

    start = _axis_values(
        start_position_m,
        "start_position_m",
        minimum=-5.0,
        maximum=5.0,
    )
    if not isinstance(axes, str):
        raise CalibrationValidationError("axes must be a comma-separated string.")
    selected_axes = [axis.strip().lower() for axis in axes.split(",") if axis.strip()]
    if not selected_axes or any(axis not in _AXES for axis in selected_axes):
        raise CalibrationValidationError("axes may contain only x, y, and z.")
    if len(set(selected_axes)) != len(selected_axes):
        raise CalibrationValidationError("axes cannot contain duplicates.")
    distance = _number(
        distance_m,
        "distance_m",
        minimum=-5.0,
        maximum=5.0,
    )

    bed = normalized["print_bed_m"]
    usable_minimum = {
        axis: bed["minimum"][axis] + bed["edge_margin_m"]
        for axis in ("x", "y")
    }
    usable_maximum = {
        axis: bed["maximum"][axis] - bed["edge_margin_m"]
        for axis in ("x", "y")
    }
    if any(
        usable_minimum[axis] >= usable_maximum[axis] for axis in ("x", "y")
    ):
        raise CalibrationNotReadyError(
            "The fixed tabletop XY size must be larger than twice its safety margin."
        )

    tool0_waypoints = [{axis: start[axis] for axis in _AXES}]
    cursor = dict(tool0_waypoints[0])
    for axis in selected_axes:
        cursor = dict(cursor)
        cursor[axis] += distance
        tool0_waypoints.append(cursor)
    tcp_offset_base = tool_tip_offset_in_base(normalized)
    tip_waypoints = [
        {
            axis: point[axis] + tcp_offset_base[axis]
            for axis in _AXES
        }
        for point in tool0_waypoints
    ]

    violations: list[str] = []
    for index, point in enumerate(tip_waypoints):
        for axis in ("x", "y"):
            if not usable_minimum[axis] <= point[axis] <= usable_maximum[axis]:
                violations.append(
                    f"waypoint {index} {axis.upper()}={point[axis]:.4f} m is "
                    f"outside the usable bed range {usable_minimum[axis]:.4f}…"
                    f"{usable_maximum[axis]:.4f} m"
                )
        minimum_z = bed["plane_z"] - _BED_BELOW_PLANE_TOLERANCE_M
        if point["z"] < minimum_z:
            violations.append(
                f"waypoint {index} Z={point['z']:.4f} m is below the required "
                f"guard height Z={minimum_z:.4f} m"
            )
    if violations:
        raise CalibrationNotReadyError(
            "Cartesian test blocked by the print-bed safety guard: "
            + "; ".join(violations[:4])
            + ("; additional violations omitted" if len(violations) > 4 else "")
            + "."
        )

    return {
        "status": "passed",
        "coordinate_frame": "base_link",
        "start_position_m": {axis: _round(start[axis]) for axis in _AXES},
        "tool0_waypoints_m": [
            {axis: _round(point[axis]) for axis in _AXES}
            for point in tool0_waypoints
        ],
        "tool_tip_waypoints_m": [
            {axis: _round(point[axis]) for axis in _AXES}
            for point in tip_waypoints
        ],
        "usable_bed_minimum_m": {
            axis: _round(usable_minimum[axis]) for axis in ("x", "y")
        },
        "usable_bed_maximum_m": {
            axis: _round(usable_maximum[axis]) for axis in ("x", "y")
        },
        "bed_plane_z_m": bed["plane_z"],
        "edge_margin_m": bed["edge_margin_m"],
        "guard_mode": (
            "fixed_table_plane"
            if tool0_motion_test
            else "fixed_table_surface"
        ),
        "minimum_air_run_clearance_m": 0.0,
        "collision_checked": False,
    }
