import math

import pytest

from ur_klipper_bridge.validation import (
    ValidationError,
    estimate_move_duration,
    validate_base_url,
    validate_finite_number,
    validate_operation_id,
    validate_positive_number,
    validate_stepper,
)


def test_validates_and_normalizes_base_url():
    assert validate_base_url("http://192.168.0.110/") == "http://192.168.0.110"
    assert validate_base_url("https://moonraker.local:7125") == (
        "https://moonraker.local:7125"
    )


@pytest.mark.parametrize(
    "value",
    [
        "",
        "192.168.0.110",
        "ftp://192.168.0.110",
        "http://user:secret@192.168.0.110",
        "http://192.168.0.110/printer",
        "http://192.168.0.110?x=1",
        " http://192.168.0.110",
    ],
)
def test_rejects_unsafe_base_urls(value):
    with pytest.raises(ValidationError):
        validate_base_url(value)


@pytest.mark.parametrize("value", ["stepper_x", "X2", "axis_01"])
def test_accepts_safe_stepper_names(value):
    assert validate_stepper(value) == value


@pytest.mark.parametrize("value", ["", "stepper x", "x;M112", "../x", "x-y"])
def test_rejects_unsafe_stepper_names(value):
    with pytest.raises(ValidationError):
        validate_stepper(value)


def test_validates_operation_identifier():
    assert validate_operation_id("sync-20260719:01") == "sync-20260719:01"
    with pytest.raises(ValidationError):
        validate_operation_id("contains a space")


@pytest.mark.parametrize("value", [math.inf, -math.inf, math.nan, True, "1"])
def test_rejects_non_finite_or_non_numeric_values(value):
    with pytest.raises(ValidationError):
        validate_finite_number("value", value)


@pytest.mark.parametrize("value", [0.0, -1.0])
def test_rejects_non_positive_motion_limits(value):
    with pytest.raises(ValidationError):
        validate_positive_number("speed", value)


def test_estimates_trapezoidal_and_triangular_moves():
    assert estimate_move_duration(0.0, 60.0, 5.0, 20.0) == pytest.approx(12.25)
    assert estimate_move_duration(0.0, 1.0, 5.0, 20.0) == pytest.approx(
        2.0 * math.sqrt(1.0 / 20.0)
    )
    assert estimate_move_duration(5.0, 5.0, 5.0, 20.0) == 0.0
