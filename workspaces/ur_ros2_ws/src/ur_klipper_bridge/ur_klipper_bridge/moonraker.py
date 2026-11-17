"""Small standard-library Moonraker client with explicit read/write methods."""

from __future__ import annotations

import hashlib
import http.client
import json
import math
import re
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field, replace
from typing import Any, Callable

from .gcode import build_disable_script
from .validation import (
    ValidationError,
    validate_base_url,
    validate_positive_number,
    validate_extruder,
)


_MANUAL_STEPPER_KEY = re.compile(r"^manual_stepper\s+([A-Za-z0-9_]+)$")
_EXTRUDER_KEY = re.compile(r"^extruder(?:[1-9][0-9]*)?$")
_VIRTUAL_SD_FILENAME = re.compile(
    r"^[A-Za-z0-9][A-Za-z0-9_.-]*\.gcode$"
)
_MAX_GCODE_FILE_BYTES = 32 * 1024 * 1024
_MAX_MOTION_STATUS_BYTES = 64 * 1024
_MAX_FULL_STATUS_BYTES = 1024 * 1024


class MoonrakerError(RuntimeError):
    """A categorized Moonraker transport, HTTP, or response failure."""

    def __init__(self, code: str, message: str):
        """Create a failure with a stable machine-readable code."""
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ExtruderSnapshot:
    """Native extruder configuration and live heater permission."""

    can_extrude: bool
    temperature: float
    target: float
    maximum_speed: float
    maximum_acceleration: float
    maximum_distance: float
    minimum_temperature: float


@dataclass(frozen=True)
class KlipperSnapshot:
    """Read-only Moonraker/Klipper readiness snapshot."""

    reachable: bool
    ready: bool
    printer_state: str
    detail: str
    manual_steppers: tuple[str, ...]
    extruders: tuple[str, ...] = ()
    extruder_states: dict[str, ExtruderSnapshot] = field(default_factory=dict)
    speed_factor: float = 0.0
    extrude_factor: float = 0.0
    toolhead_max_accel: float = 0.0
    toolhead_max_velocity: float = 0.0
    minimum_cruise_ratio: float = 0.5


@dataclass(frozen=True)
class PrintStatsSnapshot:
    """Read-only state for the one Moonraker virtual-SD print job."""

    state: str
    filename: str
    message: str
    print_duration_s: float
    total_duration_s: float


@dataclass(frozen=True)
class PrinterStateSnapshot:
    """The minimal read-only printer state needed for a stop postcondition."""

    state: str
    detail: str


def _validate_virtual_sd_filename(value: object) -> str:
    """Return a conservative basename safe for Moonraker query and upload."""
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 128
        or ".." in value
        or _VIRTUAL_SD_FILENAME.fullmatch(value) is None
    ):
        raise ValidationError(
            "filename must be a safe 1-128 character .gcode basename"
        )
    return value


def _validate_gcode_contents(value: object) -> bytes:
    """Encode one bounded UTF-8 virtual-SD program without NUL bytes."""
    if not isinstance(value, str) or not value or "\x00" in value:
        raise ValidationError(
            "G-code file must be a non-empty string without NUL bytes"
        )
    encoded = value.encode("utf-8")
    if len(encoded) > _MAX_GCODE_FILE_BYTES:
        raise ValidationError(
            f"G-code file must not exceed {_MAX_GCODE_FILE_BYTES} bytes"
        )
    return encoded


def _multipart_gcode_body(
    filename: str,
    contents: bytes,
) -> tuple[bytes, str]:
    """Build Moonraker's multipart upload using a content-bound boundary."""
    checksum = hashlib.sha256(contents).hexdigest()
    seed = hashlib.sha256(filename.encode("ascii") + contents).hexdigest()
    boundary = f"----ur-klipper-{seed}"
    suffix = 0
    while boundary.encode("ascii") in contents:
        suffix += 1
        boundary = f"----ur-klipper-{seed}-{suffix}"

    boundary_bytes = boundary.encode("ascii")
    chunks: list[bytes] = []

    def append_field(name: str, value: str) -> None:
        chunks.extend(
            [
                b"--" + boundary_bytes + b"\r\n",
                (
                    f'Content-Disposition: form-data; name="{name}"\r\n\r\n'
                ).encode("ascii"),
                value.encode("ascii"),
                b"\r\n",
            ]
        )

    append_field("root", "gcodes")
    append_field("path", "")
    append_field("checksum", checksum)
    chunks.extend(
        [
            b"--" + boundary_bytes + b"\r\n",
            (
                'Content-Disposition: form-data; name="file"; '
                f'filename="{filename}"\r\n'
            ).encode("ascii"),
            b"Content-Type: text/x-gcode\r\n\r\n",
            contents,
            b"\r\n",
            b"--" + boundary_bytes + b"--\r\n",
        ]
    )
    return b"".join(chunks), boundary


def _response_duration(name: str, value: object) -> float:
    """Normalize a non-negative print_stats duration from Moonraker."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MoonrakerError(
            "INVALID_RESPONSE",
            f"Moonraker print_stats {name} is not a number",
        )
    try:
        duration = float(value)
    except OverflowError as exc:
        raise MoonrakerError(
            "INVALID_RESPONSE",
            f"Moonraker print_stats {name} is invalid",
        ) from exc
    if not math.isfinite(duration) or duration < 0.0:
        raise MoonrakerError(
            "INVALID_RESPONSE",
            f"Moonraker print_stats {name} is invalid",
        )
    return duration


def _live_number(name: str, value: object) -> float:
    """Require a finite JSON number, rather than coerce a missing live value."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MoonrakerError("INVALID_RESPONSE", f"Invalid {name} in Klipper status")
    try:
        number = float(value)
    except (ValueError, OverflowError) as exc:
        raise MoonrakerError("INVALID_RESPONSE", f"Invalid {name} in Klipper status") from exc
    if not math.isfinite(number):
        raise MoonrakerError("INVALID_RESPONSE", f"Invalid {name} in Klipper status")
    return number


def _decode_response(body: bytes, path: str) -> dict[str, Any]:
    try:
        decoded = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MoonrakerError("INVALID_RESPONSE", f"Moonraker returned invalid JSON for {path}") from exc
    if not isinstance(decoded, dict):
        raise MoonrakerError("INVALID_RESPONSE", f"Moonraker returned a non-object response for {path}")
    if "error" in decoded:
        raise MoonrakerError("MOONRAKER_ERROR", f"Moonraker rejected {path}: {decoded['error']}")
    return decoded


def _printer_state_snapshot(response: dict[str, Any]) -> PrinterStateSnapshot:
    """Parse printer-info identically for readiness and independent stop probes."""
    result = response.get("result")
    if not isinstance(result, dict):
        raise MoonrakerError("INVALID_RESPONSE", "Missing result in /printer/info")
    raw_state = result.get("state", "")
    state = raw_state if isinstance(raw_state, str) else str(raw_state)
    raw_detail = result.get("state_message", "")
    detail = raw_detail if isinstance(raw_detail, str) else str(raw_detail)
    return PrinterStateSnapshot(state=state, detail=detail)


class _ObservationDeadline:
    """Interrupt one request's socket I/O when its total read time expires.

    A socket timeout alone restarts with each recv, allowing fragmented headers
    or body bytes to extend a read indefinitely. This watchdog owns exactly the
    socket captured for this request. Completion and expiry share a guard, and
    the timer is joined before the pooled connection can serve another read.
    """

    def __init__(self, request_socket: socket.socket, deadline: float):
        self._socket = request_socket
        self._deadline = deadline
        self._guard = threading.Lock()
        self._completed = False
        self._expired = False
        self._timer = threading.Timer(max(0., deadline - time.monotonic()), self._expire)
        self._timer.daemon = True
        self._timer.start()

    def _expire(self) -> None:
        with self._guard:
            if self._completed:
                return
            self._expired = True
            try:
                # Call the socket base implementation directly: SSLSocket's
                # override also clears its SSL wrapper, racing its reader.
                socket.socket.shutdown(self._socket, socket.SHUT_RDWR)
            except OSError:
                pass

    @property
    def expired(self) -> bool:
        with self._guard:
            return self._expired or time.monotonic() >= self._deadline

    def check(self) -> None:
        if self.expired:
            raise TimeoutError("absolute observation request deadline exceeded")

    def finish(self) -> None:
        with self._guard:
            self._completed = True
        self._timer.cancel()
        self._timer.join()


class _MotionStatusConnection:
    """One reusable observation connection, isolated from control/stop requests.

    No hidden retries: an uncertain response discards the socket and raises.
    Only the caller's existing bounded read policy may attempt a new GET.
    """

    def __init__(self, base_url: str):
        self._address = urllib.parse.urlsplit(base_url)
        self._connection: http.client.HTTPConnection | None = None
        self._lock = threading.Lock()

    def _discard(self) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            connection.close()

    def close(self) -> None:
        with self._lock:
            self._discard()

    def read(
        self, path: str, timeout_s: float,
        maximum_response_bytes: int = _MAX_MOTION_STATUS_BYTES,
    ) -> dict[str, Any]:
        started = time.monotonic()
        if not self._lock.acquire(timeout=timeout_s):
            raise MoonrakerError("NETWORK_ERROR", "Moonraker status connection lock timed out")
        phase = "connect"
        reused = self._connection is not None
        response: http.client.HTTPResponse | None = None
        watchdog: _ObservationDeadline | None = None
        try:
            remaining = timeout_s - (time.monotonic() - started)
            if remaining <= 0.0:
                raise TimeoutError("status read budget exhausted while waiting for connection lock")
            if self._connection is None:
                connection_type = http.client.HTTPSConnection if self._address.scheme == "https" else http.client.HTTPConnection
                self._connection = connection_type(
                    self._address.hostname, self._address.port, timeout=remaining,
                )
                self._connection.connect()
            connection = self._connection
            request_socket = connection.sock
            # Keep this exact socket even if getresponse() detaches it for a
            # Connection: close response. Never consult the later pool state
            # from the timer callback.
            watchdog = _ObservationDeadline(request_socket, started + timeout_s)

            def set_remaining_timeout() -> None:
                remaining = timeout_s - (time.monotonic() - started)
                if remaining <= 0.0:
                    raise TimeoutError("status read budget exhausted")
                connection.timeout = remaining
                active_socket = connection.sock or request_socket
                if active_socket is not None:
                    active_socket.settimeout(remaining)

            phase = "send"
            set_remaining_timeout()
            connection.request("GET", path, headers={"Accept": "application/json"})
            phase = "headers"
            set_remaining_timeout()
            response = connection.getresponse()
            if not 200 <= response.status < 300:
                raise MoonrakerError("HTTP_ERROR", f"Moonraker returned HTTP {response.status} for {path}")
            phase = "body"
            set_remaining_timeout()
            body = response.read(maximum_response_bytes + 1)
            watchdog.check()
            if len(body) > maximum_response_bytes:
                raise MoonrakerError("INVALID_RESPONSE", "Moonraker observation status exceeded its size limit")
            if response.length is not None and response.length > 0:
                raise http.client.IncompleteRead(body, response.length)
            if response.will_close:
                self._discard()
            decoded = _decode_response(body, path)
            watchdog.finish()
            watchdog.check()
            return decoded
        except MoonrakerError as exc:
            self._discard()
            if watchdog is not None and watchdog.expired:
                elapsed = time.monotonic() - started
                raise MoonrakerError(
                    "NETWORK_ERROR",
                    f"Moonraker status GET failed during {phase} after {elapsed:.3f}s "
                    f"(reused_connection={reused}) for {path}: absolute observation request deadline exceeded",
                ) from exc
            raise
        except (OSError, http.client.HTTPException) as exc:
            self._discard()
            elapsed = time.monotonic() - started
            raise MoonrakerError(
                "NETWORK_ERROR",
                f"Moonraker status GET failed during {phase} after {elapsed:.3f}s "
                f"(reused_connection={reused}) for {path}: {type(exc).__name__}: {exc}",
            ) from exc
        finally:
            if watchdog is not None:
                watchdog.finish()
            if response is not None:
                response.close()
            self._lock.release()


class MoonrakerClient:
    """Moonraker client; this bridge is the sole caller of write methods."""

    def __init__(
        self,
        base_url: object,
        *,
        timeout_s: object = 8.0,
        opener: Callable[..., Any] | None = None,
    ):
        """Create a strictly configured client with an injectable opener."""
        self.base_url = validate_base_url(base_url)
        self.timeout_s = validate_positive_number("timeout_s", timeout_s)
        self._opener = opener or urllib.request.urlopen
        parsed = urllib.parse.urlsplit(self.base_url)
        proxies = urllib.request.getproxies()
        uses_proxy = bool(proxies.get(parsed.scheme) or proxies.get("all")) and not urllib.request.proxy_bypass(parsed.netloc)
        # Preserve explicit test/custom openers and configured proxy behavior.
        # Readiness and runtime observations share a socket; every write and
        # the minimal emergency confirmation probe remain independent.
        self._motion_connection = _MotionStatusConnection(self.base_url) if opener is None and not uses_proxy else None

    def close(self) -> None:
        """Release the status socket after the bridge has stopped its workers."""
        if self._motion_connection is not None:
            self._motion_connection.close()

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        *,
        timeout_s: object | None = None,
    ) -> dict[str, Any]:
        if method not in {"GET", "POST"}:
            raise ValueError("Moonraker method must be GET or POST")
        contains_whitespace = any(
            character.isspace() for character in path
        )
        if not path.startswith("/") or contains_whitespace:
            raise ValueError(
                "Moonraker path must be absolute and contain no whitespace"
            )

        data: bytes | None = None
        headers: dict[str, str] = {}
        if method == "POST":
            if payload is None:
                data = b""
            else:
                data = json.dumps(
                    payload,
                    separators=(",", ":"),
                ).encode("utf-8")
                headers["Content-Type"] = "application/json"

        return self._request_bytes(
            method,
            path,
            data=data,
            headers=headers,
            timeout_s=timeout_s,
        )

    def _request_bytes(
        self,
        method: str,
        path: str,
        *,
        data: bytes | None,
        headers: dict[str, str] | None = None,
        timeout_s: object | None = None,
    ) -> dict[str, Any]:
        """Issue one request with an already encoded body and decode JSON."""
        if method not in {"GET", "POST"}:
            raise ValueError("Moonraker method must be GET or POST")
        contains_whitespace = any(character.isspace() for character in path)
        if not path.startswith("/") or contains_whitespace:
            raise ValueError(
                "Moonraker path must be absolute and contain no whitespace"
            )
        if data is not None and not isinstance(data, bytes):
            raise TypeError("Moonraker request data must be bytes or None")

        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers=headers or {},
            method=method,
        )
        timeout = (
            self.timeout_s
            if timeout_s is None
            else validate_positive_number("timeout_s", timeout_s)
        )
        try:
            with self._opener(request, timeout=timeout) as response:
                body = response.read()
        except urllib.error.HTTPError as exc:
            raise MoonrakerError(
                "HTTP_ERROR",
                f"Moonraker returned HTTP {exc.code} for {path}",
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise MoonrakerError(
                "NETWORK_ERROR",
                f"Moonraker request failed for {path}: {exc}",
            ) from exc

        return _decode_response(body, path)

    def _read_observation(
        self,
        path: str,
        *,
        timeout_s: object | None = None,
        maximum_response_bytes: int = _MAX_MOTION_STATUS_BYTES,
    ) -> dict[str, Any]:
        """Read a fresh readiness/runtime sample without a new TCP handshake.

        Explicit openers and proxy configuration retain their existing path.
        This transport performs no retry; its caller owns the read policy.
        """
        if self._motion_connection is None:
            return self._request("GET", path, timeout_s=timeout_s)
        timeout = self.timeout_s if timeout_s is None else validate_positive_number("timeout_s", timeout_s)
        return self._motion_connection.read(path, timeout, maximum_response_bytes)

    def read_printer_state(
        self,
        *,
        timeout_s: object | None = None,
    ) -> PrinterStateSnapshot:
        """Read only Klipper's current state without querying configuration.

        This deliberately uses just ``/printer/info`` so an emergency-stop
        postcondition can be checked even while the printer is in shutdown.
        ``read_status`` additionally needs configfile data and is therefore
        not a valid emergency confirmation probe.
        """
        info = self._request(
            "GET",
            "/printer/info",
            timeout_s=timeout_s,
        )
        return _printer_state_snapshot(info)

    def read_status(self) -> KlipperSnapshot:
        """Read native extruders, motion limits, overrides and heater permission."""
        printer = _printer_state_snapshot(self._read_observation("/printer/info"))

        config_response = self._read_observation(
            "/printer/objects/query?configfile&gcode_move&toolhead",
            maximum_response_bytes=_MAX_FULL_STATUS_BYTES,
        )
        try:
            configfile = config_response["result"]["status"]["configfile"]
        except (KeyError, TypeError) as exc:
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Missing configfile status in Moonraker response",
            ) from exc
        if not isinstance(configfile, dict):
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker configfile status is not an object",
            )

        sections: dict[str, dict[str, Any]] = {}
        for field in ("config", "settings"):
            raw_sections = configfile.get(field, {})
            if isinstance(raw_sections, dict):
                for name, values in raw_sections.items():
                    if isinstance(values, dict):
                        sections.setdefault(str(name), {}).update(values)

        manual_steppers = []
        for section_name in sections:
            match = _MANUAL_STEPPER_KEY.fullmatch(section_name)
            if match:
                manual_steppers.append(match.group(1))

        extruders = tuple(sorted(name for name in sections if _EXTRUDER_KEY.fullmatch(name)))
        status = config_response["result"]["status"]
        gcode_move = status.get("gcode_move", {})
        toolhead = status.get("toolhead", {})
        if not isinstance(gcode_move, dict) or not isinstance(toolhead, dict):
            raise MoonrakerError("INVALID_RESPONSE", "Invalid gcode_move/toolhead status")

        def number(name: str, value: object) -> float:
            try:
                if isinstance(value, bool):
                    raise ValueError("boolean is not a motion value")
                converted = float(value)
                if not math.isfinite(converted):
                    raise ValueError("non-finite motion value")
                return converted
            except (TypeError, ValueError) as exc:
                raise MoonrakerError("INVALID_RESPONSE", f"Invalid {name} in Klipper status") from exc

        states: dict[str, ExtruderSnapshot] = {}
        if extruders:
            response = self._read_observation("/printer/objects/query?" + "&".join(extruders))
            try:
                live_extruders = response["result"]["status"]
            except (KeyError, TypeError) as exc:
                raise MoonrakerError("INVALID_RESPONSE", "Missing native extruder status") from exc
            for name in extruders:
                live = live_extruders.get(name)
                config = sections[name]
                if not isinstance(live, dict) or not isinstance(live.get("can_extrude"), bool):
                    raise MoonrakerError("INVALID_RESPONSE", f"Missing {name} heater permission")
                states[name] = ExtruderSnapshot(
                    can_extrude=live["can_extrude"],
                    temperature=number(f"{name}.temperature", live.get("temperature")),
                    target=number(f"{name}.target", live.get("target")),
                    maximum_speed=number(f"{name}.max_extrude_only_velocity", config.get("max_extrude_only_velocity")),
                    maximum_acceleration=number(f"{name}.max_extrude_only_accel", config.get("max_extrude_only_accel")),
                    maximum_distance=number(f"{name}.max_extrude_only_distance", config.get("max_extrude_only_distance", 50.0)),
                    minimum_temperature=number(f"{name}.min_extrude_temp", config.get("min_extrude_temp", 170.0)),
                )

        return KlipperSnapshot(
            reachable=True,
            ready=printer.state.lower() == "ready",
            printer_state=printer.state,
            detail=printer.detail,
            manual_steppers=tuple(sorted(set(manual_steppers))),
            extruders=extruders,
            extruder_states=states,
            speed_factor=number("gcode_move.speed_factor", gcode_move.get("speed_factor", 0.0)),
            extrude_factor=number("gcode_move.extrude_factor", gcode_move.get("extrude_factor", 0.0)),
            toolhead_max_accel=number("toolhead.max_accel", toolhead.get("max_accel", 0.0)),
            toolhead_max_velocity=number("toolhead.max_velocity", toolhead.get("max_velocity", 0.0)),
            minimum_cruise_ratio=number("toolhead.minimum_cruise_ratio", toolhead.get("minimum_cruise_ratio", sections.get("printer", {}).get("minimum_cruise_ratio", 0.5))),
        )

    def read_motion_status(
        self,
        reference: KlipperSnapshot,
        *,
        include_print_stats: bool = False,
        timeout_s: object | None = None,
    ) -> tuple[KlipperSnapshot, PrintStatsSnapshot | None]:
        """Read one small live sample using configuration from fresh preflight.

        Runtime guards need current heater permission, overrides, toolhead limits
        and Klipper state.  Re-fetching the entire static configfile for each
        sample wastes the serialized Klippy connection.  Only immutable extruder
        caps and object inventory come from ``reference``; no live field is cached
        or carried forward when a response is missing or malformed.
        """
        if not isinstance(reference, KlipperSnapshot):
            raise ValidationError("reference must be a preflight KlipperSnapshot")
        if not isinstance(include_print_stats, bool):
            raise ValidationError("include_print_stats must be a boolean")
        fields = [
            ("webhooks", "state,state_message"),
            ("gcode_move", "speed_factor,extrude_factor"),
            ("toolhead", "max_accel,max_velocity,minimum_cruise_ratio"),
        ]
        for name in reference.extruders:
            validate_extruder(name)
            if not isinstance(reference.extruder_states.get(name), ExtruderSnapshot):
                raise ValidationError(f"Preflight reference is missing {name} configuration")
            fields.append((name, "temperature,target,can_extrude"))
        if include_print_stats:
            fields.append(("print_stats", "state,filename,message,print_duration,total_duration"))
        query = urllib.parse.urlencode(fields, safe=",")
        path = f"/printer/objects/query?{query}"
        response = self._read_observation(path, timeout_s=timeout_s)
        try:
            status = response["result"]["status"]
        except (KeyError, TypeError) as exc:
            raise MoonrakerError("INVALID_RESPONSE", "Missing live motion status") from exc
        if not isinstance(status, dict):
            raise MoonrakerError("INVALID_RESPONSE", "Invalid live motion status")

        def live_object(name: str) -> dict[str, Any]:
            value = status.get(name)
            if not isinstance(value, dict):
                raise MoonrakerError("INVALID_RESPONSE", f"Missing or invalid {name} status")
            return value

        def live_text(name: str, value: object, *, nonempty: bool = False) -> str:
            if not isinstance(value, str) or (nonempty and not value.strip()):
                raise MoonrakerError("INVALID_RESPONSE", f"Missing or invalid {name} in Klipper status")
            return value

        webhooks = live_object("webhooks")
        state = live_text("webhooks.state", webhooks.get("state"), nonempty=True).strip().lower()
        detail = live_text("webhooks.state_message", webhooks.get("state_message"))
        gcode_move = live_object("gcode_move")
        toolhead = live_object("toolhead")
        extruder_states = {}
        for name in reference.extruders:
            live = live_object(name)
            if not isinstance(live.get("can_extrude"), bool):
                raise MoonrakerError("INVALID_RESPONSE", f"Missing or invalid {name}.can_extrude")
            extruder_states[name] = replace(
                reference.extruder_states[name],
                can_extrude=live["can_extrude"],
                temperature=_live_number(f"{name}.temperature", live.get("temperature")),
                target=_live_number(f"{name}.target", live.get("target")),
            )
        snapshot = replace(
            reference,
            reachable=True,
            ready=state == "ready",
            printer_state=state,
            detail=detail,
            extruder_states=extruder_states,
            speed_factor=_live_number("gcode_move.speed_factor", gcode_move.get("speed_factor")),
            extrude_factor=_live_number("gcode_move.extrude_factor", gcode_move.get("extrude_factor")),
            toolhead_max_accel=_live_number("toolhead.max_accel", toolhead.get("max_accel")),
            toolhead_max_velocity=_live_number("toolhead.max_velocity", toolhead.get("max_velocity")),
            minimum_cruise_ratio=_live_number("toolhead.minimum_cruise_ratio", toolhead.get("minimum_cruise_ratio")),
        )
        print_stats = None
        if include_print_stats:
            live = live_object("print_stats")
            print_stats = PrintStatsSnapshot(
                state=live_text("print_stats.state", live.get("state"), nonempty=True).strip().lower(),
                filename=live_text("print_stats.filename", live.get("filename")),
                message=live_text("print_stats.message", live.get("message")),
                print_duration_s=_response_duration("print_duration", live.get("print_duration")),
                total_duration_s=_response_duration("total_duration", live.get("total_duration")),
            )
        return snapshot, print_stats

    def post_gcode(
        self,
        script: str,
        *,
        timeout_s: object | None = None,
    ) -> None:
        """POST one internally generated G-code script to Moonraker."""
        if not isinstance(script, str) or not script or len(script) > 16384:
            raise ValidationError(
                "G-code script must contain 1-16384 characters"
            )
        self._request(
            "POST",
            "/printer/gcode/script",
            {"script": script},
            timeout_s=timeout_s,
        )

    def upload_gcode(
        self,
        filename: object,
        contents: object,
        *,
        timeout_s: object | None = None,
    ) -> str:
        """Upload one generated G-code program to Moonraker's gcodes root."""
        safe_filename = _validate_virtual_sd_filename(filename)
        encoded_contents = _validate_gcode_contents(contents)
        body, boundary = _multipart_gcode_body(
            safe_filename,
            encoded_contents,
        )
        response = self._request_bytes(
            "POST",
            "/server/files/upload",
            data=body,
            headers={
                "Content-Type": f"multipart/form-data; boundary={boundary}",
            },
            timeout_s=timeout_s,
        )
        try:
            response_container = response
            if "item" not in response_container:
                response_container = response_container["result"]
            item = response_container["item"]
            root = item["root"]
            uploaded_path = item["path"]
        except (KeyError, TypeError) as exc:
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker upload result is missing its item path",
            ) from exc
        if root != "gcodes" or uploaded_path != safe_filename:
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker upload result did not match the requested "
                "gcode path",
            )
        return safe_filename

    def start_print(self, filename: object) -> None:
        """Start exactly one validated virtual-SD G-code basename."""
        safe_filename = _validate_virtual_sd_filename(filename)
        encoded_filename = urllib.parse.quote(safe_filename, safe="")
        self._request(
            "POST",
            f"/printer/print/start?filename={encoded_filename}",
        )

    def read_print_state(self) -> PrintStatsSnapshot:
        """Read the current Klipper print_stats state without mutation."""
        response = self._request(
            "GET",
            "/printer/objects/query?print_stats",
        )
        try:
            stats = response["result"]["status"]["print_stats"]
        except (KeyError, TypeError) as exc:
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Missing print_stats status in Moonraker response",
            ) from exc
        if not isinstance(stats, dict):
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker print_stats status is not an object",
            )

        raw_state = stats.get("state")
        raw_filename = stats.get("filename", "")
        raw_message = stats.get("message", "")
        if not isinstance(raw_state, str) or not raw_state.strip():
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker print_stats state is missing or invalid",
            )
        invalid_text = not isinstance(raw_filename, str) or not isinstance(
            raw_message,
            str,
        )
        if invalid_text:
            raise MoonrakerError(
                "INVALID_RESPONSE",
                "Moonraker print_stats text fields are invalid",
            )
        return PrintStatsSnapshot(
            state=raw_state.strip().lower(),
            filename=raw_filename,
            message=raw_message,
            print_duration_s=_response_duration(
                "print_duration",
                stats.get("print_duration", 0.0),
            ),
            total_duration_s=_response_duration(
                "total_duration",
                stats.get("total_duration", 0.0),
            ),
        )

    def cancel_print(self) -> None:
        """Cancel Moonraker's active virtual-SD print, if any."""
        self._request("POST", "/printer/print/cancel")

    def disable_stepper(self, stepper: object) -> None:
        """Disable one strictly validated native extruder, preserving the lock motor."""
        self.post_gcode(build_disable_script(validate_extruder(stepper)))

    def emergency_stop(self) -> None:
        """Request Klipper's emergency stop endpoint."""
        self._request("POST", "/printer/emergency_stop")
