#!/usr/bin/env python3
"""Generate the planner's collision constants from the shared geometry JSON."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def generate_header(config_path: Path) -> str:
    geometry = json.loads(config_path.read_text(encoding="utf-8"))
    if geometry.get("schema_version") != 1:
        raise ValueError("Unsupported printhead geometry schema")
    source_sha256 = geometry.get("source_sha256", "")
    if not isinstance(source_sha256, str) or re.fullmatch(r"[a-f0-9]{64}", source_sha256) is None:
        raise ValueError("Printhead STEP SHA-256 is missing or invalid")
    if not isinstance(geometry.get("geometry_revision"), str) or not geometry["geometry_revision"]:
        raise ValueError("Printhead geometry revision is missing")
    envelope = geometry["collision_envelope_tool0_m"]
    constants = {}
    for field, prefix in (("size", "kPrintheadBoxSize"), ("center", "kPrintheadBoxCenter")):
        values = envelope[field]
        if set(values) != {"x", "y", "z"}:
            raise ValueError(f"Printhead envelope {field} must define X/Y/Z")
        for axis in ("x", "y", "z"):
            value = values[axis]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Printhead envelope {field}/{axis} must be a finite number")
            if field == "size" and not 0.0 < value <= 1.0:
                raise ValueError("Printhead envelope dimensions must be positive and within 1 m")
            if field == "center" and abs(value) > 1.0:
                raise ValueError("Printhead envelope center must be within 1 m of tool0")
            constants[prefix + axis.upper()] = float(value)
    canonical = json.dumps(
        geometry, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False,
    ).encode("utf-8")
    fingerprint = hashlib.sha256(canonical).hexdigest()
    lines = [
        "// Generated from config/printhead_geometry.json; do not edit.",
        "#pragma once", "", "namespace ur3e_print_planner", "{",
        f'constexpr char kPrintheadGeometrySha256[] = "{fingerprint}";',
        f'constexpr char kPrintheadSourceSha256[] = "{source_sha256}";',
    ]
    lines.extend(f"constexpr double {name} = {value:.17g};" for name, value in constants.items())
    lines.extend(["}  // namespace ur3e_print_planner", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    content = generate_header(args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Avoid forcing recompilation when CMake reconfigures unrelated inputs.
    if not args.output.is_file() or args.output.read_text(encoding="utf-8") != content:
        args.output.write_text(content, encoding="utf-8")


if __name__ == "__main__":
    main()
