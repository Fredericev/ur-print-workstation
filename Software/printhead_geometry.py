"""Provisional installed-printhead transform shared by path review and UI."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any


PRINTHEAD_GEOMETRY_PATH = (
    Path(__file__).resolve().parents[1]
    / "workspaces" / "ur_ros2_ws" / "config" / "printhead_geometry.json"
)


def load_printhead_geometry() -> dict[str, Any]:
    value = json.loads(PRINTHEAD_GEOMETRY_PATH.read_text(encoding="utf-8"))
    if value.get("schema_version") != 1 or value.get("clocking_deg") != 0:
        raise ValueError("Unsupported installed printhead geometry.")
    for name in ("tool0_to_cad_origin_m", "active_nozzle_in_cad_m", "tool0_to_nozzle_m"):
        if set(value[name]) != {"x", "y", "z"}:
            raise ValueError(f"Invalid printhead {name}.")
    origin = value["tool0_to_cad_origin_m"]
    cad = value["active_nozzle_in_cad_m"]
    rotation = value["tool0_to_cad_rpy_deg"]
    roll, pitch, yaw = (math.radians(float(rotation[axis])) for axis in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    matrix = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
              (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
              (-sp, cp * sr, cp * cr))
    expected = {axis: origin[axis] + sum(matrix[row][column] * cad[other]
                for column, other in enumerate(("x", "y", "z")))
                for row, axis in enumerate(("x", "y", "z"))}
    if any(abs(expected[axis] - value["tool0_to_nozzle_m"][axis]) > 1e-6
           for axis in ("x", "y", "z")):
        raise ValueError("The printhead nozzle transform does not match its CAD mounting transform.")
    return value


def printhead_geometry_sha256(geometry: dict[str, Any] | None = None) -> str:
    """Identify the complete installed tool model, including mounting and envelope."""
    value = load_printhead_geometry() if geometry is None else geometry
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()
