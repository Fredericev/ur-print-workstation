#!/usr/bin/env python3
"""Export one G-code job as fixed-table base_link geometry for RViz."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Software.gcode_calibration import read_saved_calibration  # noqa: E402
from Software.gcode_path_review import build_offline_review  # noqa: E402


DEFAULT_OUTPUT = (
    PROJECT_ROOT / "workspaces" / "ur_ros2_ws" / "runtime" / "rviz_job_preview.json"
)
DEFAULT_CALIBRATION = (
    Path.home()
    / "Library"
    / "Application Support"
    / "UR Print Workstation"
    / "gcode_calibration.json"
)


def build_preview(gcode_path: Path, calibration_path: Path) -> dict[str, Any]:
    saved = read_saved_calibration(calibration_path)
    if saved is None:
        raise ValueError(f"Installed print setup was not found: {calibration_path}")
    review = build_offline_review(
        gcode_path.name,
        gcode_path.read_bytes(),
        saved["calibration"],
    )
    return {
        "schema_version": 1,
        "frame_id": review["coordinate_frame"],
        "source": review["source"],
        "operation_mode": review["operation_mode"],
        "placement": review["placement_model"],
        "table": review["workcell_model"]["tabletop"],
        "print_bed_m": review["print_bed_m"],
        "segments": review["path"]["segments"],
        "layers": review["layers"],
    }


def write_preview(payload: dict[str, Any], destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":")) + "\n")
    temporary.replace(destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("gcode", type=Path)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    payload = build_preview(args.gcode.resolve(), args.calibration.expanduser().resolve())
    write_preview(payload, args.output.resolve())
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "source": payload["source"]["name"],
                "segments": len(payload["segments"]),
                "print_center_m": payload["placement"]["print_center_m"],
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
