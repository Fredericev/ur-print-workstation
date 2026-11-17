"""Fixed, measured workcell geometry shared by preview and execution gates."""

from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any


WORKCELL_GEOMETRY_PATH = (
    Path(__file__).resolve().parents[1]
    / "workspaces"
    / "ur_ros2_ws"
    / "config"
    / "workcell_geometry.json"
)


class WorkcellGeometryError(ValueError):
    """The installed fixed-workcell definition is missing or invalid."""


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise WorkcellGeometryError(f"{name} must be an object.")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise WorkcellGeometryError(f"{name} must be a finite number.")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise WorkcellGeometryError(f"{name} must be a finite number.") from exc
    if not math.isfinite(result):
        raise WorkcellGeometryError(f"{name} must be a finite number.")
    return result


def _rounded(value: float) -> float:
    result = round(value, 6)
    return 0.0 if result == -0.0 else result


def load_workcell_geometry(path: Path | None = None) -> dict[str, Any]:
    """Load and validate the one installed, axis-aligned workcell definition."""

    source = WORKCELL_GEOMETRY_PATH if path is None else Path(path)
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WorkcellGeometryError(
            f"Could not read fixed workcell geometry from {source}: {exc}"
        ) from exc
    root = _mapping(value, "workcell_geometry")
    if root.get("schema_version") != 1:
        raise WorkcellGeometryError("workcell_geometry.schema_version must be 1.")
    if root.get("frame_id") != "base_link":
        raise WorkcellGeometryError("The fixed workcell frame must be base_link.")

    tabletop = _mapping(root.get("tabletop"), "workcell_geometry.tabletop")
    center_value = _mapping(tabletop.get("center_m"), "tabletop.center_m")
    dimensions_value = _mapping(
        tabletop.get("dimensions_m"), "tabletop.dimensions_m"
    )
    center = {
        axis: _rounded(_number(center_value.get(axis), f"tabletop.center_m.{axis}"))
        for axis in ("x", "y")
    }
    dimensions = {
        axis: _rounded(
            _number(dimensions_value.get(axis), f"tabletop.dimensions_m.{axis}")
        )
        for axis in ("x", "y")
    }
    if any(value <= 0.0 or value > 5.0 for value in dimensions.values()):
        raise WorkcellGeometryError(
            "The fixed tabletop X/Y dimensions must be greater than 0 and at most 5 m."
        )

    yaw = _rounded(_number(tabletop.get("yaw_deg"), "tabletop.yaw_deg"))
    if abs(yaw) > 1e-9:
        raise WorkcellGeometryError(
            "Only a tabletop aligned with base_link X/Y (yaw 0 degrees) is supported."
        )
    surface_z = _rounded(
        _number(tabletop.get("surface_z_m"), "tabletop.surface_z_m")
    )
    margin = _rounded(
        _number(tabletop.get("safety_margin_m"), "tabletop.safety_margin_m")
    )
    if margin < 0.0 or any(dimension <= 2.0 * margin for dimension in dimensions.values()):
        raise WorkcellGeometryError(
            "The tabletop safety margin must be non-negative and leave a usable XY area."
        )
    thickness_value = tabletop.get("thickness_m")
    if thickness_value is not None:
        thickness_value = _rounded(_number(thickness_value, "tabletop.thickness_m"))
        if thickness_value <= 0.0:
            raise WorkcellGeometryError("tabletop.thickness_m must be positive or null.")

    placement = _mapping(root.get("placement"), "workcell_geometry.placement")
    if placement.get("mode") != "center_print_bounds":
        raise WorkcellGeometryError(
            "The fixed workcell placement mode must be center_print_bounds."
        )
    if placement.get("vertical_datum") != "gcode_z0_on_tabletop":
        raise WorkcellGeometryError(
            "The fixed workcell vertical datum must be gcode_z0_on_tabletop."
        )

    minimum = {
        axis: _rounded(center[axis] - dimensions[axis] / 2.0)
        for axis in ("x", "y")
    }
    maximum = {
        axis: _rounded(center[axis] + dimensions[axis] / 2.0)
        for axis in ("x", "y")
    }
    return {
        "schema_version": 1,
        "frame_id": "base_link",
        "tabletop": {
            "center_m": center,
            "dimensions_m": dimensions,
            "bounds_m": {"minimum": minimum, "maximum": maximum},
            "yaw_deg": yaw,
            "surface_z_m": surface_z,
            "thickness_m": thickness_value,
            "safety_margin_m": margin,
            "measurement": _mapping(
                tabletop.get("measurement"), "tabletop.measurement"
            ),
        },
        "placement": {
            "mode": "center_print_bounds",
            "vertical_datum": "gcode_z0_on_tabletop",
        },
    }


def fixed_print_bed_m(workcell: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return the legacy-compatible bed shape derived from fixed geometry."""

    geometry = load_workcell_geometry() if workcell is None else workcell
    tabletop = geometry["tabletop"]
    return {
        "plane_z": tabletop["surface_z_m"],
        "minimum": dict(tabletop["bounds_m"]["minimum"]),
        "maximum": dict(tabletop["bounds_m"]["maximum"]),
        "edge_margin_m": tabletop["safety_margin_m"],
    }


def save_surface_height_m(
    surface_z_m: object,
    path: Path | None = None,
    *,
    measurement_source: str = "operator_verified_height_trial",
) -> dict[str, Any]:
    """Commit the tabletop collision plane without changing its XY geometry."""

    height = _rounded(_number(surface_z_m, "tabletop.surface_z_m"))
    if not -0.5 <= height <= 0.5:
        raise WorkcellGeometryError("Tabletop height must be between -500 and 500 mm.")
    if measurement_source not in {"operator_selected", "operator_verified_height_trial"}:
        raise WorkcellGeometryError("Unsupported tabletop height source.")
    target = WORKCELL_GEOMETRY_PATH if path is None else Path(path)
    # Validate the existing file before preserving its other installed fields.
    load_workcell_geometry(target)
    data = json.loads(target.read_text(encoding="utf-8"))
    data["tabletop"]["surface_z_m"] = height
    data["tabletop"]["measurement"]["surface_z"] = measurement_source
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, target)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
    return load_workcell_geometry(target)


def center_gcode_on_workcell(
    bounds: dict[str, Any],
    rotation_z_deg: float,
    scale_m_per_mm: float,
    workcell: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return the deterministic transform origin that centers printed material."""

    geometry = load_workcell_geometry() if workcell is None else workcell
    placement_bounds = _mapping(bounds.get("placement"), "bounds.placement")
    minimum = _mapping(placement_bounds.get("minimum"), "bounds.placement.minimum")
    maximum = _mapping(placement_bounds.get("maximum"), "bounds.placement.maximum")
    source_center_mm = {
        axis: (
            _number(minimum.get(axis), f"bounds.placement.minimum.{axis}")
            + _number(maximum.get(axis), f"bounds.placement.maximum.{axis}")
        )
        / 2.0
        for axis in ("x", "y")
    }
    rotation = math.radians(_number(rotation_z_deg, "rotation_z_deg"))
    scale = _number(scale_m_per_mm, "scale_m_per_mm")
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    transformed_center = {
        "x": scale
        * (source_center_mm["x"] * cosine - source_center_mm["y"] * sine),
        "y": scale
        * (source_center_mm["x"] * sine + source_center_mm["y"] * cosine),
    }
    tabletop = geometry["tabletop"]
    print_center = tabletop["center_m"]
    origin = {
        "x": _rounded(print_center["x"] - transformed_center["x"]),
        "y": _rounded(print_center["y"] - transformed_center["y"]),
        "z": tabletop["surface_z_m"],
    }
    return {
        "mode": "center_on_fixed_table",
        "bounds_source": str(placement_bounds.get("source", "all_motion")),
        "source_center_mm": {
            axis: _rounded(source_center_mm[axis]) for axis in ("x", "y")
        },
        "print_center_m": dict(print_center),
        "origin_m": origin,
    }
