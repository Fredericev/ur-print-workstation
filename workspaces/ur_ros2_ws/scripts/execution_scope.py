"""Hardware-independent validation of immutable print execution coverage."""

from __future__ import annotations

import math
from typing import Any


MAX_EXECUTION_BLOCKS = 1_000_000
PROGRESS_TOLERANCE = 1.0e-9
SYNC_START_LEAD_S = 4.0


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite.")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite.") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite.")
    return number


def estimate_minimum_duration_s(manifest: dict[str, Any]) -> float:
    """Motion time plus mandatory synchronized starts, excluding validation.

    Continuous v3 jobs have one shared relative start lead. Legacy packages
    retain their old per-block estimate. Live collision checks, network latency
    and endpoint settling can make the actual run longer; this is not an ETA.
    """

    if manifest.get("version") == 3:
        starts = manifest.get("coordination", {}).get("synchronized_start_count")
        if isinstance(starts, bool) or not isinstance(starts, int) or starts != 1:
            raise ValueError("A continuous job must have exactly one synchronized start.")
    else:
        starts = sum(abs(_finite(block.get("klipper_units"), "block Klipper units")) > 1.0e-12
                     for block in manifest.get("blocks", []))
    return round(_finite(manifest.get("planned_duration_s"), "planned motion duration")
                 + SYNC_START_LEAD_S * starts, 3)


def validate_execution_scope(manifest: dict[str, Any]) -> None:
    """Reject full-file labels on incomplete frozen execution packages.

    Both the desktop prepare/read path and the ROS runner use this exact
    validator. Legacy packages retain their original sample/production scope.
    New full cold packages must retain every process block, contiguous progress
    coverage, and the reviewed final UR joint geometry.
    """

    mode = manifest.get("operation_mode", "production_print")
    if not isinstance(mode, str) or mode not in {"production_print", "tool0_motion_test"}:
        raise ValueError("The execution operation mode is invalid.")
    scope = manifest.get("execution_scope", {})
    extent = manifest.get("execution_extent", "full" if mode == "production_print" else "sample")
    if not isinstance(extent, str) or extent not in {"sample", "full"}:
        raise ValueError("The execution extent must be sample or full.")
    if mode == "production_print" and extent != "full":
        raise ValueError("Production execution must cover the full source path.")
    expected_type = ("full_source_path" if mode == "production_print" else
                     "full_cold_motion_test" if extent == "full" else
                     "bounded_cold_motion_sample")
    if not isinstance(scope, dict) or scope.get("type") != expected_type:
        raise ValueError("The execution extent and frozen scope do not match.")
    if scope.get("full_path_validated") is not True:
        raise ValueError("The complete source path has not been validated.")
    blocks = manifest.get("blocks", [])
    if not isinstance(blocks, list) or not 0 < len(blocks) <= MAX_EXECUTION_BLOCKS:
        raise ValueError("The execution scope contains an invalid number of motion blocks.")
    block_count = manifest.get("block_count")
    if isinstance(block_count, bool) or not isinstance(block_count, int) or block_count != len(blocks):
        raise ValueError("The execution scope block count does not match.")
    source_count = manifest.get("source_block_count")
    if source_count is not None and (
        isinstance(source_count, bool) or not isinstance(source_count, int)
        or not len(blocks) <= source_count <= MAX_EXECUTION_BLOCKS
    ):
        raise ValueError("The source motion block count is invalid.")
    previous_progress = 0.0
    for index, block in enumerate(blocks):
        if (not isinstance(block, dict) or isinstance(block.get("index"), bool)
                or not isinstance(block.get("index"), int) or block.get("index") != index):
            raise ValueError("The execution scope has invalid block ordering.")
        start = _finite(block.get("start_progress"), "block start progress")
        end = _finite(block.get("end_progress"), "block end progress")
        if not 0.0 <= start <= end <= 1.0:
            raise ValueError("The execution block progress is invalid.")
        if abs(start - previous_progress) > PROGRESS_TOLERANCE:
            raise ValueError("The execution path has a gap or overlap in Cartesian coverage.")
        previous_progress = end
    if abs(_finite(scope.get("end_progress"), "scope end progress") - previous_progress) > PROGRESS_TOLERANCE:
        raise ValueError("The execution scope and final motion progress differ.")
    duration = sum(_finite(block.get("duration_s"), "block duration") for block in blocks)
    if abs(_finite(scope.get("motion_duration_s"), "scope motion duration") - duration) > 0.001:
        raise ValueError("The execution scope duration differs from frozen motion timing.")
    if extent != "full":
        return
    if abs(previous_progress - 1.0) > PROGRESS_TOLERANCE:
        raise ValueError("The full execution path is missing Cartesian coverage.")
    if scope.get("requested_duration_s") is not None:
        raise ValueError("A full execution path cannot carry a sample duration limit.")
    if mode == "tool0_motion_test":
        if manifest.get("heating_enabled") is not False:
            raise ValueError("A full cold motion test cannot enable heating.")
        if source_count != len(blocks):
            raise ValueError("The full execution path lost source motion blocks.")
        if (manifest.get("source_motion_line_end") != blocks[-1].get("source_line_end")
                or scope.get("source_line_end") != blocks[-1].get("source_line_end")):
            raise ValueError("The full execution path lost terminal source events.")
    try:
        profiles = manifest["trajectory_profiles"]
        end_positions = (
            profiles["print"]["points"][-1]["positions"]
            if "print" in profiles
            else profiles["blocks"][-1]["points"][-1]["positions"]
        )
        source_end = manifest["print_trajectory"]["end_positions"]
        if len(end_positions) != 6 or len(source_end) != 6:
            raise ValueError("invalid final UR joint count")
        if max(abs(_finite(a, "final UR position") - _finite(b, "source UR endpoint"))
               for a, b in zip(end_positions, source_end)) > 1.0e-6:
            raise ValueError("final UR geometry differs from the full source")
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise ValueError(f"The full execution path is incomplete: {exc}") from exc
