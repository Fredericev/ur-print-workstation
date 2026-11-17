"""Pure, injection-safe Klipper command builders."""

from __future__ import annotations

from .validation import (
    ValidationError,
    validate_finite_number,
    validate_positive_number,
    validate_extruder,
    extrusion_submove_count,
    extrusion_peak_speed,
)


MINIMUM_ABSOLUTE_START_GUARD_S = 0.20
# Covers the executor's bounded four-second preparation window. Both the
# absolute ROS-time adapter and legacy relative G4 path share this limit.
MAXIMUM_START_DELAY_S = 5.0
MAXIMUM_PROGRAM_SEGMENTS = 1_000_000


def start_delay_until(
    start_time_nanoseconds: int,
    now_nanoseconds: int,
) -> float:
    """Convert a requested ROS start time to a best-effort relative delay.

    This does not map ROS time to Klipper print_time or the MCU clock. Network
    and Klippy queueing latency occur after this calculation.
    """

    if isinstance(start_time_nanoseconds, bool) or not isinstance(
        start_time_nanoseconds, int
    ):
        raise ValidationError("start_time_nanoseconds must be an integer")
    if start_time_nanoseconds <= 0:
        raise ValidationError("start_time_nanoseconds must be positive")
    remaining = (start_time_nanoseconds - int(now_nanoseconds)) / 1_000_000_000
    if remaining < MINIMUM_ABSOLUTE_START_GUARD_S:
        raise ValidationError(
            "The synchronized start deadline is too close or has already passed"
        )
    if remaining > MAXIMUM_START_DELAY_S:
        raise ValidationError(
            "The requested start deadline is more than "
            f"{MAXIMUM_START_DELAY_S:g} seconds away"
        )
    return remaining


def format_number(value: object) -> str:
    """Format a validated finite number without exponent-sized noise."""
    number = validate_finite_number("value", value)
    if number == 0.0:
        number = 0.0
    return format(number, ".12g")


_GCODE_STATE_NAME = "ur_native_extrusion"


def _native_header(extruder: str, acceleration: float) -> list[str]:
    # Preserve the operator's coordinate/extrusion mode. Overrides are checked
    # before dispatch, never silently changed to match the compiled program.
    return [
        f"SAVE_GCODE_STATE NAME={_GCODE_STATE_NAME}",
        f"ACTIVATE_EXTRUDER EXTRUDER={extruder}",
        "G21",
        "M83",
    ]


def build_dispatch_script(
    *,
    stepper: object,
    target_position: object,
    speed: object,
    acceleration: object,
    set_position_before_move: bool,
    initial_position: object,
    start_delay_s: object = 0.0,
    maximum_extrude_only_distance: object = 50.0,
) -> str:
    """Build native relative E moves; the legacy ROS field names are retained."""
    extruder = validate_extruder(stepper)
    target = validate_finite_number("target_position", target_position)
    safe_speed = validate_positive_number("speed", speed)
    safe_acceleration = validate_positive_number("acceleration", acceleration)
    initial = validate_finite_number("initial_position", initial_position)
    start_delay = validate_finite_number("start_delay_s", start_delay_s)
    if start_delay < 0.0 or start_delay > MAXIMUM_START_DELAY_S:
        raise ValidationError(
            f"start_delay_s must be between 0 and {MAXIMUM_START_DELAY_S:g} seconds"
        )
    if set_position_before_move is not True:
        raise ValidationError("native extrusion requires set_position_before_move=true")
    distance = target - initial
    count = extrusion_submove_count(distance, maximum_extrude_only_distance)
    if count > MAXIMUM_PROGRAM_SEGMENTS:
        raise ValidationError("native extrusion dispatch expands to too many moves")
    lines = _native_header(extruder, safe_acceleration)
    if start_delay > 0.0:
        lines.append(f"G4 P{round(start_delay * 1000)}")
    if distance != 0.0:
        lines.extend(
            f"G1 E{format_number(distance / count)} F{format_number(safe_speed * 60.0)}"
            for _ in range(count)
        )
    return "\n".join(lines)


def build_completion_barrier_script(
    *, stepper: object, target_position: object, speed: object, acceleration: object,
) -> str:
    """Wait for native E motion without dispatching another extrusion move."""
    validate_extruder(stepper)
    validate_finite_number("target_position", target_position)
    validate_positive_number("speed", speed)
    validate_positive_number("acceleration", acceleration)
    return f"M400\nRESTORE_GCODE_STATE NAME={_GCODE_STATE_NAME}"


def build_disable_script(stepper: object) -> str:
    """Remove holding current from only the selected native extruder."""
    extruder = validate_extruder(stepper)
    return f"SET_STEPPER_ENABLE STEPPER={extruder} ENABLE=0"


def build_program_file(
    *,
    stepper: object,
    segments: object,
    start_delay_s: object,
    maximum_speed: object,
    disable_after: object,
    maximum_acceleration: object = 20.0,
    maximum_extrude_only_distance: object = 50.0,
) -> str:
    """Build a native E-only virtual-SD program on the allotted timeline.

    Every pure E G1 starts and stops in stock Klipper.  The requested segment
    time therefore includes acceleration/deceleration, also when a long move
    needs splitting to retain max_extrude_only_distance protection.
    """
    extruder = validate_extruder(stepper)
    delay = validate_finite_number("start_delay_s", start_delay_s)
    if delay < 0.0 or delay > MAXIMUM_START_DELAY_S:
        raise ValidationError(
            f"start_delay_s must be between 0 and {MAXIMUM_START_DELAY_S:g} seconds"
        )
    speed_limit = validate_positive_number("maximum_speed", maximum_speed)
    acceleration = validate_positive_number("maximum_acceleration", maximum_acceleration)
    distance_limit = validate_positive_number(
        "maximum_extrude_only_distance", maximum_extrude_only_distance,
    )
    if not isinstance(disable_after, bool):
        raise ValidationError("disable_after must be a bool")
    if not isinstance(segments, (list, tuple)):
        raise ValidationError("segments must be a list or tuple")
    if not 0 < len(segments) <= MAXIMUM_PROGRAM_SEGMENTS:
        raise ValidationError(f"segments must contain 1-{MAXIMUM_PROGRAM_SEGMENTS} entries")

    lines = _native_header(extruder, acceleration)
    if delay > 0.0:
        lines.append(f"G4 P{round(delay * 1000)}")
    dwell_duration_s = 0.0
    emitted_dwell_ms = 0
    emitted_moves = 0
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ValidationError(f"segment {index} must be an object")
        units = validate_finite_number(f"segment {index} units", segment.get("units"))
        duration = validate_positive_number(f"segment {index} duration_s", segment.get("duration_s"))
        if abs(units) <= 1.0e-12:
            dwell_duration_s += duration
            next_dwell_ms = round(dwell_duration_s * 1000)
            dwell_ms = next_dwell_ms - emitted_dwell_ms
            emitted_dwell_ms = next_dwell_ms
            lines.append(f"G4 P{dwell_ms}")
            continue
        count = extrusion_submove_count(units, distance_limit)
        emitted_moves += count
        if emitted_moves > MAXIMUM_PROGRAM_SEGMENTS:
            raise ValidationError("native extrusion program expands to too many moves")
        peak = extrusion_peak_speed(units / count, duration / count, speed_limit, acceleration)
        lines.extend(
            f"G1 E{format_number(units / count)} F{format_number(peak * 60.0)}"
            for _ in range(count)
        )
    lines.extend(("M400", f"RESTORE_GCODE_STATE NAME={_GCODE_STATE_NAME}"))
    if disable_after:
        lines.append(build_disable_script(extruder))
    return "\n".join(lines) + "\n"
