"""Pure validation helpers shared by the ROS node and unit tests."""

from __future__ import annotations

import math
import re
from urllib.parse import urlsplit


_STEPPER_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")
_EXTRUDER_PATTERN = re.compile(r"^extruder(?:[1-9][0-9]*)?$")
_OPERATION_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")

EXTRUSION_PROTOCOL = "klipper-native-extruder-v1"
DEFAULT_EXTRUDER_MAX_SPEED = 5.0
DEFAULT_EXTRUDER_MAX_ACCELERATION = 20.0
DEFAULT_EXTRUDER_MAX_DISTANCE = 50.0


class ValidationError(ValueError):
    """Raised when an externally supplied value is unsafe or ambiguous."""


def validate_base_url(value: object) -> str:
    """Return a normalized, strict HTTP(S) Moonraker base URL."""
    if not isinstance(value, str) or not value:
        raise ValidationError("moonraker_url must be a non-empty string")
    contains_whitespace = any(character.isspace() for character in value)
    if value != value.strip() or contains_whitespace:
        raise ValidationError("moonraker_url must not contain whitespace")

    parsed = urlsplit(value)
    if parsed.scheme not in {"http", "https"}:
        raise ValidationError("moonraker_url must use http or https")
    if not parsed.hostname:
        raise ValidationError("moonraker_url must include a host")
    if parsed.username is not None or parsed.password is not None:
        raise ValidationError("moonraker_url must not contain credentials")
    if parsed.query or parsed.fragment:
        raise ValidationError(
            "moonraker_url must not contain a query or fragment"
        )
    if parsed.path not in {"", "/"}:
        raise ValidationError("moonraker_url must not contain a path")
    try:
        parsed.port
    except ValueError as exc:
        raise ValidationError(
            f"moonraker_url has an invalid port: {exc}"
        ) from exc

    return f"{parsed.scheme}://{parsed.netloc}".rstrip("/")


def validate_stepper(value: object) -> str:
    """Validate the suffix from a Klipper [manual_stepper <suffix>] section."""
    if not isinstance(value, str) or not _STEPPER_PATTERN.fullmatch(value):
        raise ValidationError(
            "stepper may only contain ASCII letters, digits, and underscores"
        )
    return value


def validate_extruder(value: object) -> str:
    """Allow only native Klipper extruders, never a manual-stepper alias."""
    if not isinstance(value, str) or not _EXTRUDER_PATTERN.fullmatch(value):
        raise ValidationError("extruder must be 'extruder' or 'extruder1', 'extruder2', etc.")
    return value


def validate_operation_id(value: object) -> str:
    """Validate an operator-visible action correlation identifier."""
    is_safe_identifier = (
        isinstance(value, str) and _OPERATION_ID_PATTERN.fullmatch(value)
    )
    if not is_safe_identifier:
        raise ValidationError(
            "operation_id must be 1-128 safe identifier characters and start "
            "with a letter or digit"
        )
    return value


def validate_finite_number(name: str, value: object) -> float:
    """Return a finite float while rejecting booleans and non-number inputs."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValidationError(f"{name} must be finite")
    return number


def validate_positive_number(name: str, value: object) -> float:
    """Return a finite number that is strictly greater than zero."""
    number = validate_finite_number(name, value)
    if number <= 0.0:
        raise ValidationError(f"{name} must be greater than zero")
    return number


def estimate_move_duration(
    start_position: object,
    target_position: object,
    speed: object,
    acceleration: object,
) -> float:
    """Estimate a stopped triangular/trapezoidal motion duration."""
    start = validate_finite_number("start_position", start_position)
    target = validate_finite_number("target_position", target_position)
    velocity = validate_positive_number("speed", speed)
    accel = validate_positive_number("acceleration", acceleration)
    distance = abs(target - start)
    if distance == 0.0:
        return 0.0

    acceleration_and_deceleration_distance = velocity * velocity / accel
    if distance >= acceleration_and_deceleration_distance:
        return (
            (2.0 * velocity / accel)
            + ((distance - acceleration_and_deceleration_distance) / velocity)
        )
    return 2.0 * math.sqrt(distance / accel)


def extrusion_submove_count(units: object, maximum_distance: object = 50.0) -> int:
    """Split long relative E moves without changing Klipper's distance guard."""
    distance = abs(validate_finite_number("units", units))
    limit = validate_positive_number("maximum_extrude_only_distance", maximum_distance)
    # A nominal E50 can become 50.00000000000001 when Klipper subtracts
    # accumulated positions. Leave a small numerical margin below the guard.
    safe_limit = limit * (1.0 - 1.0e-9)
    return max(1, math.ceil(distance / safe_limit))


def minimum_extrusion_duration(
    units: object,
    speed: object = 5.0,
    acceleration: object = 20.0,
    max_distance: object = 50.0,
) -> float:
    """Minimum native E duration, including every split move's full stop.

    Stock Klipper's pure-extrusion moves do not carry velocity across G1
    junctions.  Equal distance splits minimize total time under a common cap.
    """
    distance = abs(validate_finite_number("units", units))
    count = extrusion_submove_count(distance, max_distance)
    return count * estimate_move_duration(0.0, distance / count, speed, acceleration)


def extrusion_peak_speed(
    units: object, duration_s: object, maximum_speed: object, acceleration: object,
) -> float:
    """Solve d = v*T - v*v/a for one stopped native E move."""
    distance = abs(validate_finite_number("units", units))
    duration = validate_positive_number("duration_s", duration_s)
    limit = validate_positive_number("maximum_speed", maximum_speed)
    accel = validate_positive_number("acceleration", acceleration)
    if distance == 0.0:
        return 0.0
    minimum = estimate_move_duration(0.0, distance, limit, accel)
    if duration + 1.0e-9 < minimum:
        raise ValidationError(
            f"native extrusion duration {duration:g}s is shorter than its "
            f"minimum {minimum:g}s at speed {limit:g}mm/s and acceleration {accel:g}mm/s^2"
        )
    discriminant = max(0.0, duration * duration - 4.0 * distance / accel)
    return min(limit, 2.0 * distance / (duration + math.sqrt(discriminant)))


def validate_native_execution(payload: dict) -> None:
    """Reject packages compiled for a different native E timing contract."""
    if payload.get("extrusion_protocol") != EXTRUSION_PROTOCOL:
        raise ValidationError("Package predates native extrusion; validate and prepare again")
    extruder = validate_extruder(payload.get("extruder"))
    if payload.get("manual_stepper", extruder) != extruder:
        raise ValidationError("Package extrusion targets disagree")
    limits = payload["stepper_limits"]
    speed = validate_positive_number("maximum_speed", limits["maximum_speed"])
    accel = validate_positive_number("acceleration", limits["acceleration"])
    distance = validate_positive_number("max_extrude_only_distance", limits["max_extrude_only_distance"])
    if speed > DEFAULT_EXTRUDER_MAX_SPEED or abs(accel - DEFAULT_EXTRUDER_MAX_ACCELERATION) > 1e-9 or abs(distance - DEFAULT_EXTRUDER_MAX_DISTANCE) > 1e-9:
        raise ValidationError("Package native extruder limits differ from the installed timing contract")
    for index, block in enumerate(payload["blocks"]):
        duration = validate_positive_number(f"block {index} duration_s", block["duration_s"])
        minimum = minimum_extrusion_duration(block["klipper_units"], speed, accel, distance)
        if duration + 1e-9 < minimum:
            raise ValidationError(f"Block {index} is shorter than its native extrusion minimum")
