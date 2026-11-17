"""Fresh, read-only UR joint-stationarity proof for incident recovery.

This RTDE client subscribes only to controller *outputs*.  It never registers
an input recipe, sends a URScript program, or starts robot motion.  The RTDE
START packet starts this client's output stream, not a robot program.
"""

from __future__ import annotations

import math
import socket
import struct
import time
from datetime import datetime, timezone
from typing import Any


_MAX_PACKET_BYTES = 4096
_OUTPUT_FIELDS = b"timestamp,actual_q,actual_qd"
_OUTPUT_TYPES = b"DOUBLE,VECTOR6D,VECTOR6D"
_DATA_FORMAT = "!13d"
_DATA_BYTES = struct.calcsize(_DATA_FORMAT)


def _packet(kind: bytes, payload: bytes = b"") -> bytes:
    return struct.pack("!H", len(payload) + 3) + kind + payload


def _read_exact(connection: socket.socket, count: int, deadline: float) -> bytes:
    result = bytearray()
    while len(result) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Timed out waiting for fresh UR RTDE feedback")
        connection.settimeout(min(remaining, 1.0))
        try:
            part = connection.recv(count - len(result))
        except socket.timeout:
            continue
        if not part:
            raise RuntimeError("UR RTDE connection closed before stop proof completed")
        result.extend(part)
    return bytes(result)


def _read_packet(connection: socket.socket, deadline: float) -> tuple[bytes, bytes]:
    size = struct.unpack("!H", _read_exact(connection, 2, deadline))[0]
    if size < 3 or size > _MAX_PACKET_BYTES:
        raise RuntimeError(f"Invalid UR RTDE packet size: {size}")
    content = _read_exact(connection, size - 2, deadline)
    return content[:1], content[1:]


def _read_kind(connection: socket.socket, expected: bytes, deadline: float) -> bytes:
    # Controller text messages can be interleaved with setup and data packets.
    for _ in range(64):
        kind, payload = _read_packet(connection, deadline)
        if kind == expected:
            return payload
        if kind != b"M":
            raise RuntimeError(f"Unexpected UR RTDE packet type while awaiting {expected!r}")
    raise RuntimeError("Too many UR RTDE text messages without joint feedback")


def verify_stationary_rtde(
    host: str,
    *,
    port: int = 30004,
    timeout_s: float = 5.0,
    dwell_s: float = 0.5,
    minimum_samples: int = 5,
    maximum_velocity_rad_s: float = 0.003,
    maximum_position_drift_rad: float = 0.001,
) -> dict[str, Any]:
    """Prove six joints stationary from new RTDE samples after a new connection.

    Return ``stationary=False`` if the stream ends or times out before enough
    evidence arrives.  Invalid protocol/recipe responses raise so recovery
    remains locked with a diagnostic error.
    """

    if timeout_s <= dwell_s or dwell_s <= 0 or minimum_samples < 2:
        raise ValueError("RTDE stop-proof timing is invalid")
    deadline = time.monotonic() + timeout_s
    try:
        connection = socket.create_connection((host, port), timeout=min(timeout_s, 2.0))
    except OSError as exc:
        raise RuntimeError(f"Cannot connect to UR read-only RTDE feedback: {exc}") from exc

    with connection:
        connection.settimeout(min(timeout_s, 2.0))
        connection.sendall(_packet(b"V", struct.pack("!H", 2)))
        if _read_kind(connection, b"V", deadline) != b"\x01":
            raise RuntimeError("UR RTDE protocol version 2 was not accepted")

        # Protocol v2 setup: frequency (double) followed by an output-only
        # comma-separated recipe.  No input recipe or data write is sent.
        connection.sendall(_packet(b"O", struct.pack("!d", 25.0) + _OUTPUT_FIELDS))
        recipe = _read_kind(connection, b"O", deadline)
        if len(recipe) < 2 or recipe[0] == 0 or recipe[1:] != _OUTPUT_TYPES:
            raise RuntimeError("UR RTDE did not accept the expected read-only joint recipe")
        recipe_id = recipe[0]

        connection.sendall(_packet(b"S"))
        if _read_kind(connection, b"S", deadline) != b"\x01":
            raise RuntimeError("UR RTDE did not start its read-only output stream")

        streak_position: tuple[float, ...] | None = None
        streak_host_time = 0.0
        streak_robot_time = 0.0
        last_robot_time: float | None = None
        sample_count = 0
        maximum_velocity_seen = 0.0
        failure_reason = "No continuous stationary UR joint interval was observed"
        while time.monotonic() < deadline:
            try:
                payload = _read_kind(connection, b"U", deadline)
            except (OSError, RuntimeError, TimeoutError) as exc:
                failure_reason = str(exc)
                break
            if len(payload) != 1 + _DATA_BYTES or payload[0] != recipe_id:
                raise RuntimeError("UR RTDE joint sample does not match the verified output recipe")
            values = struct.unpack(_DATA_FORMAT, payload[1:])
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("UR RTDE joint sample contains a non-finite value")
            robot_time = values[0]
            if last_robot_time is not None and robot_time <= last_robot_time:
                # Duplicates or rewinds cannot count as fresh observations.
                streak_position = None
                sample_count = 0
                maximum_velocity_seen = 0.0
                failure_reason = "UR RTDE joint timestamps did not advance"
                continue
            last_robot_time = robot_time
            position = values[1:7]
            speed = max(abs(value) for value in values[7:13])
            if speed > maximum_velocity_rad_s:
                streak_position = None
                sample_count = 0
                maximum_velocity_seen = 0.0
                failure_reason = "UR joint velocity exceeds the stationary limit"
                continue
            now = time.monotonic()
            if streak_position is None or max(abs(a - b) for a, b in zip(position, streak_position)) > maximum_position_drift_rad:
                streak_position = position
                streak_host_time = now
                streak_robot_time = robot_time
                sample_count = 1
                maximum_velocity_seen = speed
                continue
            sample_count += 1
            maximum_velocity_seen = max(maximum_velocity_seen, speed)
            if (
                sample_count >= minimum_samples
                and now - streak_host_time >= dwell_s
                and robot_time - streak_robot_time >= dwell_s
            ):
                return {
                    "stationary": True,
                    "sample_count": sample_count,
                    "observed_dwell_s": round(now - streak_host_time, 3),
                    "maximum_velocity_rad_s": maximum_velocity_seen,
                    "maximum_allowed_velocity_rad_s": maximum_velocity_rad_s,
                    "maximum_allowed_position_drift_rad": maximum_position_drift_rad,
                    "observed_at": datetime.now(timezone.utc).isoformat(),
                    "source": "new-read-only-rtde-connection",
                }

        return {
            "stationary": False,
            "sample_count": sample_count,
            "reason": failure_reason,
            "observed_at": datetime.now(timezone.utc).isoformat(),
            "source": "new-read-only-rtde-connection",
        }
