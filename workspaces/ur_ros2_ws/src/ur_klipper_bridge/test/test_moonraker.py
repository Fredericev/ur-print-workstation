import hashlib
import http.client
import json
import threading
import time
from types import SimpleNamespace
import urllib.error
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from ur_klipper_bridge.moonraker import (
    ExtruderSnapshot,
    KlipperSnapshot,
    MoonrakerClient,
    MoonrakerError,
)
from ur_klipper_bridge.validation import ValidationError


class FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        return False

    def read(self):
        if isinstance(self._payload, bytes):
            return self._payload
        return json.dumps(self._payload).encode("utf-8")


class QueueOpener:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def __call__(self, request, *, timeout):
        self.requests.append((request, timeout))
        return FakeResponse(self.responses.pop(0))


def test_readiness_poll_uses_get_only_and_finds_manual_stepper():
    opener = QueueOpener(
        [
            {
                "result": {
                    "state": "ready",
                    "state_message": "Printer is ready",
                }
            },
            {
                "result": {
                    "status": {
                        "configfile": {
                            "config": {
                                "manual_stepper stepper_x": {},
                                "printer": {},
                            }
                        }
                    }
                }
            },
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    snapshot = client.read_status()

    assert snapshot.ready is True
    assert snapshot.manual_steppers == ("stepper_x",)
    assert [request.get_method() for request, _timeout in opener.requests] == [
        "GET",
        "GET",
    ]
    assert opener.requests[0][0].full_url.endswith("/printer/info")


def test_gcode_write_is_one_json_post():
    opener = QueueOpener([{"result": "ok"}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    client.post_gcode("MANUAL_STEPPER STEPPER=stepper_x ENABLE=0")

    request, timeout = opener.requests[0]
    assert request.get_method() == "POST"
    assert request.full_url.endswith("/printer/gcode/script")
    assert json.loads(request.data.decode("utf-8")) == {
        "script": "MANUAL_STEPPER STEPPER=stepper_x ENABLE=0"
    }
    assert timeout == 8.0


def test_completion_write_can_extend_http_timeout():
    opener = QueueOpener([{"result": "ok"}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    client.post_gcode(
        "MANUAL_STEPPER STEPPER=stepper_x MOVE=1 SPEED=1 ACCEL=1 SYNC=1",
        timeout_s=42.0,
    )

    assert opener.requests[0][1] == 42.0


def test_emergency_stop_is_explicit_empty_post():
    opener = QueueOpener([{"result": "ok"}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    client.emergency_stop()

    request, _timeout = opener.requests[0]
    assert request.get_method() == "POST"
    assert request.full_url.endswith("/printer/emergency_stop")
    assert request.data == b""


def test_printer_state_probe_reads_only_printer_info_with_a_short_timeout():
    opener = QueueOpener(
        [
            {
                "result": {
                    "state": "shutdown",
                    "state_message": "Shutdown due to webhooks request",
                }
            }
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    snapshot = client.read_printer_state(timeout_s=0.75)

    assert snapshot.state == "shutdown"
    assert snapshot.detail == "Shutdown due to webhooks request"
    assert len(opener.requests) == 1
    request, timeout = opener.requests[0]
    assert request.get_method() == "GET"
    assert request.full_url.endswith("/printer/info")
    assert timeout == 0.75


def test_invalid_json_is_categorized():
    opener = QueueOpener([b"not-json"])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    with pytest.raises(MoonrakerError, match="invalid JSON") as error:
        client.read_status()
    assert error.value.code == "INVALID_RESPONSE"


def test_virtual_sd_upload_is_multipart_with_sha256_checksum():
    opener = QueueOpener(
        [
            {
                "result": {
                    "item": {
                        "root": "gcodes",
                        "path": "ur_cold_job-42.gcode",
                    },
                    "action": "create_file",
                }
            }
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)
    contents = "G4 P4000\nM400\n"

    uploaded_path = client.upload_gcode(
        "ur_cold_job-42.gcode",
        contents,
        timeout_s=30.0,
    )

    assert uploaded_path == "ur_cold_job-42.gcode"
    request, timeout = opener.requests[0]
    assert request.get_method() == "POST"
    assert request.full_url.endswith("/server/files/upload")
    assert timeout == 30.0
    content_type = request.get_header("Content-type")
    assert content_type.startswith("multipart/form-data; boundary=")
    body = request.data
    assert b'name="root"' in body
    assert b"gcodes" in body
    assert b'name="path"' in body
    assert b'name="checksum"' in body
    checksum = hashlib.sha256(contents.encode("utf-8")).hexdigest().encode()
    assert checksum in body
    assert b'name="file"; filename="ur_cold_job-42.gcode"' in body
    assert contents.encode("utf-8") in body


def test_virtual_sd_upload_accepts_documented_top_level_item_response():
    opener = QueueOpener(
        [
            {
                "item": {
                    "root": "gcodes",
                    "path": "ur_cold_job-42.gcode",
                },
                "action": "create_file",
            }
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    uploaded_path = client.upload_gcode(
        "ur_cold_job-42.gcode",
        "M400\n",
    )

    assert uploaded_path == "ur_cold_job-42.gcode"


@pytest.mark.parametrize(
    "filename",
    [
        "../escape.gcode",
        "nested/file.gcode",
        "/absolute.gcode",
        "query.gcode?root=config",
        "not-gcode.txt",
        ".hidden.gcode",
    ],
)
def test_virtual_sd_operations_reject_unsafe_filename(filename):
    opener = QueueOpener([])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    with pytest.raises(ValidationError, match="filename"):
        client.upload_gcode(filename, "M400\n")
    with pytest.raises(ValidationError, match="filename"):
        client.start_print(filename)

    assert opener.requests == []


def test_virtual_sd_upload_requires_exact_gcodes_response_path():
    opener = QueueOpener(
        [
            {
                "result": {
                    "item": {
                        "root": "config",
                        "path": "ur_cold_job.gcode",
                    }
                }
            }
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    with pytest.raises(MoonrakerError, match="upload result") as error:
        client.upload_gcode("ur_cold_job.gcode", "M400\n")

    assert error.value.code == "INVALID_RESPONSE"


def test_virtual_sd_start_is_explicit_empty_post():
    opener = QueueOpener([{"result": "ok"}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    client.start_print("ur_cold_job-42.gcode")

    request, timeout = opener.requests[0]
    assert request.get_method() == "POST"
    assert request.full_url.endswith(
        "/printer/print/start?filename=ur_cold_job-42.gcode"
    )
    assert request.data == b""
    assert timeout == 8.0


def test_print_stats_read_is_get_only_and_normalized():
    opener = QueueOpener(
        [
            {
                "result": {
                    "status": {
                        "print_stats": {
                            "state": "printing",
                            "filename": "ur_cold_job-42.gcode",
                            "message": "",
                            "print_duration": 12.5,
                            "total_duration": 13.0,
                        }
                    }
                }
            }
        ]
    )
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    snapshot = client.read_print_state()

    assert snapshot.state == "printing"
    assert snapshot.filename == "ur_cold_job-42.gcode"
    assert snapshot.message == ""
    assert snapshot.print_duration_s == 12.5
    assert snapshot.total_duration_s == 13.0
    request, _timeout = opener.requests[0]
    assert request.get_method() == "GET"
    assert request.full_url.endswith("/printer/objects/query?print_stats")
    assert request.data is None


def test_print_stats_read_rejects_missing_status():
    opener = QueueOpener([{"result": {"status": {}}}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    with pytest.raises(MoonrakerError, match="print_stats") as error:
        client.read_print_state()

    assert error.value.code == "INVALID_RESPONSE"


def test_cancel_print_is_explicit_empty_post():
    opener = QueueOpener([{"result": "ok"}])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    client.cancel_print()

    request, _timeout = opener.requests[0]
    assert request.get_method() == "POST"
    assert request.full_url.endswith("/printer/print/cancel")
    assert request.data == b""


def motion_reference():
    return KlipperSnapshot(
        reachable=True,
        ready=True,
        printer_state="ready",
        detail="Original preflight",
        manual_steppers=("lock_motor",),
        extruders=("extruder", "extruder1"),
        extruder_states={
            name: ExtruderSnapshot(
                can_extrude=True,
                temperature=205.0,
                target=210.0,
                maximum_speed=5.0,
                maximum_acceleration=20.0,
                maximum_distance=50.0,
                minimum_temperature=170.0,
            )
            for name in ("extruder", "extruder1")
        },
        speed_factor=1.0,
        extrude_factor=1.0,
        toolhead_max_accel=1000.0,
        toolhead_max_velocity=1000.0,
        minimum_cruise_ratio=0.5,
    )


def motion_response():
    return {
        "result": {
            "status": {
                "webhooks": {"state": "ready", "state_message": "Live state"},
                "gcode_move": {"speed_factor": 0.9, "extrude_factor": 1.1},
                "toolhead": {
                    "max_accel": 900.0,
                    "max_velocity": 800.0,
                    "minimum_cruise_ratio": 0.4,
                },
                "extruder": {"temperature": 212.0, "target": 215.0, "can_extrude": True},
                "extruder1": {"temperature": 32.0, "target": 0.0, "can_extrude": False},
                "print_stats": {
                    "state": "printing",
                    "filename": "ur_native.gcode",
                    "message": "",
                    "print_duration": 12.5,
                    "total_duration": 13.0,
                },
            }
        }
    }


def test_motion_read_selects_only_live_fields_in_one_get_and_preserves_reference():
    reference = motion_reference()
    original_states = dict(reference.extruder_states)
    opener = QueueOpener([motion_response()])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    snapshot, stats = client.read_motion_status(reference, include_print_stats=True, timeout_s=0.75)

    assert len(opener.requests) == 1
    request, timeout = opener.requests[0]
    assert request.get_method() == "GET"
    assert request.data is None
    assert timeout == 0.75
    url = urllib.parse.urlsplit(request.full_url)
    assert url.path == "/printer/objects/query"
    assert urllib.parse.parse_qs(url.query) == {
        "webhooks": ["state,state_message"],
        "gcode_move": ["speed_factor,extrude_factor"],
        "toolhead": ["max_accel,max_velocity,minimum_cruise_ratio"],
        "extruder": ["temperature,target,can_extrude"],
        "extruder1": ["temperature,target,can_extrude"],
        "print_stats": ["state,filename,message,print_duration,total_duration"],
    }
    assert snapshot.ready and snapshot.reachable
    assert snapshot.detail == "Live state"
    assert (snapshot.speed_factor, snapshot.extrude_factor) == (0.9, 1.1)
    assert (snapshot.toolhead_max_accel, snapshot.toolhead_max_velocity, snapshot.minimum_cruise_ratio) == (900.0, 800.0, 0.4)
    assert snapshot.manual_steppers == ("lock_motor",)
    assert snapshot.extruders == reference.extruders
    assert snapshot.extruder_states["extruder"].temperature == 212.0
    assert not snapshot.extruder_states["extruder1"].can_extrude
    for name in reference.extruders:
        before, current = original_states[name], snapshot.extruder_states[name]
        assert (current.maximum_speed, current.maximum_acceleration, current.maximum_distance, current.minimum_temperature) == (
            before.maximum_speed, before.maximum_acceleration, before.maximum_distance, before.minimum_temperature,
        )
        assert current is not before
    assert reference.extruder_states == original_states
    assert reference.detail == "Original preflight"
    assert snapshot.extruder_states is not reference.extruder_states
    assert stats.state == "printing"
    assert stats.filename == "ur_native.gcode"
    assert stats.print_duration_s == 12.5
    assert stats.total_duration_s == 13.0


def test_motion_read_without_print_stats_requests_a_fresh_sample_each_time():
    first = motion_response()
    second = motion_response()
    second["result"]["status"]["webhooks"] = {"state": "shutdown", "state_message": "Fresh shutdown"}
    second["result"]["status"]["extruder"]["temperature"] = 170.0
    opener = QueueOpener([first, second])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    first_snapshot, first_stats = client.read_motion_status(motion_reference())
    second_snapshot, second_stats = client.read_motion_status(motion_reference())

    assert first_snapshot.ready
    assert not second_snapshot.ready
    assert second_snapshot.printer_state == "shutdown"
    assert second_snapshot.detail == "Fresh shutdown"
    assert second_snapshot.extruder_states["extruder"].temperature == 170.0
    assert first_stats is second_stats is None
    assert len(opener.requests) == 2
    assert all("print_stats" not in request.full_url for request, _ in opener.requests)


@pytest.mark.parametrize(
    "object_name,field,value",
    [
        ("webhooks", "state", None),
        ("webhooks", "state", ""),
        ("webhooks", "state", 1),
        ("webhooks", "state_message", None),
        ("gcode_move", "speed_factor", True),
        ("gcode_move", "speed_factor", "1.0"),
        ("gcode_move", "extrude_factor", float("nan")),
        ("toolhead", "max_accel", float("inf")),
        ("toolhead", "max_accel", 10 ** 400),
        ("toolhead", "max_velocity", None),
        ("toolhead", "minimum_cruise_ratio", False),
        ("extruder", "temperature", "210"),
        ("extruder", "target", float("-inf")),
        ("extruder", "can_extrude", 1),
        ("print_stats", "state", None),
        ("print_stats", "filename", None),
        ("print_stats", "message", None),
        ("print_stats", "print_duration", -1.0),
        ("print_stats", "total_duration", float("nan")),
        ("print_stats", "total_duration", 10 ** 400),
    ],
)
def test_motion_read_rejects_malformed_live_fields(object_name, field, value):
    response = motion_response()
    response["result"]["status"][object_name][field] = value
    opener = QueueOpener([response])
    client = MoonrakerClient("http://192.168.0.110", opener=opener)

    with pytest.raises(MoonrakerError) as error:
        client.read_motion_status(motion_reference(), include_print_stats=True)

    assert error.value.code == "INVALID_RESPONSE"
    assert len(opener.requests) == 1


@pytest.mark.parametrize("object_name", ["webhooks", "gcode_move", "toolhead", "extruder", "extruder1", "print_stats"])
def test_motion_read_rejects_missing_live_objects(object_name):
    response = motion_response()
    del response["result"]["status"][object_name]
    client = MoonrakerClient("http://192.168.0.110", opener=QueueOpener([response]))

    with pytest.raises(MoonrakerError) as error:
        client.read_motion_status(motion_reference(), include_print_stats=True)

    assert error.value.code == "INVALID_RESPONSE"


@pytest.mark.parametrize(
    "object_name,field",
    [
        ("webhooks", "state_message"),
        ("gcode_move", "speed_factor"),
        ("toolhead", "minimum_cruise_ratio"),
        ("extruder", "temperature"),
        ("extruder1", "can_extrude"),
        ("print_stats", "print_duration"),
        ("print_stats", "filename"),
    ],
)
def test_motion_read_rejects_missing_live_fields(object_name, field):
    response = motion_response()
    del response["result"]["status"][object_name][field]
    client = MoonrakerClient("http://192.168.0.110", opener=QueueOpener([response]))

    with pytest.raises(MoonrakerError) as error:
        client.read_motion_status(motion_reference(), include_print_stats=True)

    assert error.value.code == "INVALID_RESPONSE"


def test_motion_read_preserves_transport_error_and_does_not_retry():
    calls = []

    def timeout_opener(request, *, timeout):
        calls.append((request, timeout))
        raise urllib.error.URLError("timed out")

    client = MoonrakerClient("http://192.168.0.110", opener=timeout_opener)

    with pytest.raises(MoonrakerError, match="timed out") as error:
        client.read_motion_status(motion_reference(), timeout_s=0.5)

    assert error.value.code == "NETWORK_ERROR"
    assert len(calls) == 1


@pytest.mark.parametrize("response", [{}, {"result": None}, {"result": {"status": None}}, {"result": {"status": []}}])
def test_motion_read_rejects_malformed_status_container(response):
    client = MoonrakerClient("http://192.168.0.110", opener=QueueOpener([response]))

    with pytest.raises(MoonrakerError) as error:
        client.read_motion_status(motion_reference())

    assert error.value.code == "INVALID_RESPONSE"


@pytest.fixture
def motion_http_server():
    """An inert HTTP/1.1 server with observable TCP accept/request counts."""
    requests = []

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def do_GET(self):
            requests.append((self.path, self.client_address[1]))
            body = json.dumps(motion_response()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", requests
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


@pytest.fixture
def observation_http_server():
    """Serve full readiness and compact status over observable loopback TCP."""
    state = SimpleNamespace(requests=[], maximum_speed=5., config_padding_bytes=0, block_config=False,
                            entered=threading.Event(), release=threading.Event())

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def send_json(self, payload):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            state.requests.append(("GET", self.path, self.client_address[1]))
            if self.path == "/printer/info":
                self.send_json({"result": {"state": "ready", "state_message": "Loopback ready"}})
            elif "?configfile&" in self.path:
                if state.block_config:
                    state.entered.set()
                    state.release.wait(2.)
                self.send_json({"result": {"status": {
                    "configfile": {"padding": "x" * state.config_padding_bytes, "settings": {
                        "extruder": {"max_extrude_only_velocity": state.maximum_speed,
                                     "max_extrude_only_accel": 20.},
                        "manual_stepper lock_motor": {},
                    }},
                    "gcode_move": {"speed_factor": 1., "extrude_factor": 1.},
                    "toolhead": {"max_accel": 1000., "max_velocity": 1000., "minimum_cruise_ratio": .5},
                }}})
            elif self.path == "/printer/objects/query?extruder":
                self.send_json({"result": {"status": {"extruder": {
                    "temperature": 200., "target": 205., "can_extrude": True,
                }}}})
            else:
                self.send_json(motion_response())

        def do_POST(self):
            state.requests.append(("POST", self.path, self.client_address[1]))
            self.send_json({"result": "ok"})

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", state
    finally:
        state.release.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


def test_full_readiness_reuses_one_tcp_connection_for_every_fresh_get(observation_http_server):
    address, state = observation_http_server
    client = MoonrakerClient(address)
    try:
        first = client.read_status()
        second = client.read_status()
        assert first.ready and second.ready
        assert len(state.requests) == 6
        assert all(method == "GET" for method, _path, _port in state.requests)
        assert len({port for _method, _path, port in state.requests}) == 1
    finally:
        client.close()


def test_full_readiness_and_runtime_share_connection_without_caching_config(observation_http_server):
    address, state = observation_http_server
    client = MoonrakerClient(address)
    try:
        first = client.read_status()
        state.maximum_speed = 4.
        second = client.read_status()
        live, _stats = client.read_motion_status(second)
        assert first.extruder_states["extruder"].maximum_speed == 5.
        assert second.extruder_states["extruder"].maximum_speed == 4.
        assert live.extruder_states["extruder"].maximum_speed == 4.
        assert live.extruder_states["extruder"].temperature == 212.
        assert live.speed_factor == .9
        assert len(state.requests) == 7
        assert len({port for _method, _path, port in state.requests}) == 1
    finally:
        client.close()


@pytest.mark.parametrize("padding_bytes,accepted", [(70 * 1024, True), (1024 * 1024, False)])
def test_full_configuration_has_its_own_bounded_response_limit(observation_http_server, padding_bytes, accepted):
    address, state = observation_http_server
    client = MoonrakerClient(address)
    state.config_padding_bytes = padding_bytes
    try:
        if accepted:
            assert client.read_status().ready
            assert len(state.requests) == 3
        else:
            with pytest.raises(MoonrakerError) as caught:
                client.read_status()
            assert caught.value.code == "INVALID_RESPONSE"
            assert len(state.requests) == 2, "full-config transport must not retry implicitly"
            previous_port = state.requests[0][2]
            state.config_padding_bytes = 0
            assert client.read_status().ready
            assert state.requests[2][2] != previous_port
    finally:
        client.close()


@pytest.fixture
def deadline_http_server():
    """Drip valid local responses faster than a per-recv socket timeout."""
    state = SimpleNamespace(mode="healthy", requests=[], posts=[], entered=threading.Event(), drip_interval_s=.04)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def do_GET(self):
            state.requests.append((self.path, self.client_address[1]))
            if self.path == "/printer/info":
                body = b'{"result":{"state":"shutdown","state_message":"loopback stop confirmed"}}'
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            mode = state.mode
            body = json.dumps(motion_response(), separators=(",", ":")).encode()
            try:
                if mode != "healthy":
                    state.entered.set()
                if mode == "slow_headers":
                    self.wfile.write(b"HTTP/1.1 200 OK\r\n")
                    for _index in range(8):
                        self.wfile.write(b"X-Slow: fragment\r\n")
                        self.wfile.flush()
                        time.sleep(state.drip_interval_s)
                    self.wfile.write(f"Content-Length: {len(body)}\r\n\r\n".encode() + body)
                    self.wfile.flush()
                    return
                self.send_response(200)
                if mode == "slow_chunked":
                    self.send_header("Transfer-Encoding", "chunked")
                elif mode == "slow_close_body":
                    self.send_header("Connection", "close")
                    self.close_connection = True
                else:
                    self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                if mode == "healthy":
                    self.wfile.write(body)
                    return
                size = (len(body) + 7) // 8
                for offset in range(0, len(body), size):
                    chunk = body[offset:offset + size]
                    self.wfile.write((f"{len(chunk):x}\r\n".encode() + chunk + b"\r\n") if mode == "slow_chunked" else chunk)
                    self.wfile.flush()
                    time.sleep(state.drip_interval_s)
                if mode == "slow_chunked":
                    self.wfile.write(b"0\r\n\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def do_POST(self):
            state.posts.append((self.path, self.client_address[1]))
            body = b'{"result":"ok"}'
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", state
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


@pytest.mark.parametrize("mode", ["slow_headers", "slow_body", "slow_chunked", "slow_close_body"])
def test_observation_deadline_interrupts_fragmented_headers_and_body(deadline_http_server, mode):
    address, state = deadline_http_server
    client = MoonrakerClient(address)
    state.mode = mode
    started = time.monotonic()
    try:
        with pytest.raises(MoonrakerError) as caught:
            client.read_motion_status(motion_reference(), timeout_s=.12)
        elapsed = time.monotonic() - started
        assert caught.value.code == "NETWORK_ERROR"
        assert .08 <= elapsed < .25, "progress must not reset the absolute observation deadline"
        assert len(state.requests) == 1, "the transport must not retry"
        state.mode = "healthy"
        snapshot, _stats = client.read_motion_status(motion_reference(), timeout_s=.5)
        assert snapshot.ready
        assert state.requests[1][1] != state.requests[0][1]
    finally:
        client.close()


def test_observation_deadline_does_not_block_independent_stop_requests(deadline_http_server):
    address, state = deadline_http_server
    client = MoonrakerClient(address, timeout_s=1.)
    state.mode = "slow_body"
    state.drip_interval_s = .06
    errors = []

    def read_slow_body():
        try:
            client.read_motion_status(motion_reference(), timeout_s=.3)
        except Exception as exc:
            errors.append(exc)

    reader = threading.Thread(target=read_slow_body, daemon=True)
    reader.start()
    try:
        assert state.entered.wait(1.)
        client.emergency_stop()
        assert client.read_printer_state(timeout_s=.1).state == "shutdown"
        assert reader.is_alive(), "stop channel must finish while the observation is still waiting"
        assert state.posts[0][0] == "/printer/emergency_stop"
        assert state.posts[0][1] != state.requests[0][1]
        reader.join(timeout=.6)
        assert not reader.is_alive()
        assert len(errors) == 1
        assert isinstance(errors[0], MoonrakerError)
        assert errors[0].code == "NETWORK_ERROR"
    finally:
        reader.join(timeout=1.)
        client.close()


def test_late_canceled_watchdog_cannot_close_a_later_pooled_request(fault_http_server, monkeypatch):
    address, state = fault_http_server
    timers = []

    class DeferredTimer:
        """Model a timer callback already queued when cancellation occurs."""

        def __init__(self, _interval, callback):
            self.callback = callback
            self.canceled = False
            self.joined = False
            timers.append(self)

        def start(self):
            pass

        def cancel(self):
            self.canceled = True

        def join(self):
            self.joined = True

    monkeypatch.setattr(threading, "Timer", DeferredTimer)
    client = MoonrakerClient(address)
    errors = []
    results = []
    client.read_motion_status(motion_reference(), timeout_s=.5)
    first_timer = timers[0]
    assert first_timer.canceled and first_timer.joined
    state.mode = "blocked"

    def next_read():
        try:
            results.append(client.read_motion_status(motion_reference(), timeout_s=.5))
        except Exception as exc:
            errors.append(exc)

    reader = threading.Thread(target=next_read, daemon=True)
    reader.start()
    try:
        assert state.entered.wait(1.)
        first_timer.callback()
        state.release.set()
        reader.join(timeout=1.)
        assert not reader.is_alive()
        assert errors == []
        assert results[0][0].ready
        assert state.requests[0][2] == state.requests[1][2], "the second read must reuse the first socket"
        assert all(timer.canceled and timer.joined for timer in timers)
    finally:
        state.release.set()
        reader.join(timeout=1.)
        client.close()


def test_emergency_and_confirmation_remain_independent_of_blocked_full_readiness(observation_http_server):
    address, state = observation_http_server
    client = MoonrakerClient(address, timeout_s=1.)
    state.block_config = True
    errors = []

    def full_read():
        try:
            client.read_status()
        except Exception as exc:
            errors.append(exc)

    reader = threading.Thread(target=full_read, daemon=True)
    reader.start()
    try:
        assert state.entered.wait(1.)
        client.emergency_stop()
        assert client.read_printer_state(timeout_s=.5).state == "ready"
        assert reader.is_alive()
        assert [method for method, _path, _port in state.requests] == ["GET", "GET", "POST", "GET"]
        observation_port = state.requests[0][2]
        assert state.requests[1][2] == observation_port
        assert all(port != observation_port for _method, _path, port in state.requests[2:])
    finally:
        state.release.set()
        reader.join(timeout=1.)
        client.close()
    assert not errors


def test_motion_poll_reuses_connection_when_later_tcp_connects_are_lost(
    motion_http_server, monkeypatch,
):
    base_url, requests = motion_http_server
    original_connect = http.client.HTTPConnection.connect
    connects = []

    def lose_later_connections(connection):
        connects.append(connection)
        if len(connects) > 1:
            raise TimeoutError("simulated later TCP SYN loss")
        return original_connect(connection)

    monkeypatch.setattr(http.client.HTTPConnection, "connect", lose_later_connections)
    client = MoonrakerClient(base_url)
    try:
        for _ in range(4):
            snapshot, _stats = client.read_motion_status(motion_reference(), timeout_s=0.5)
            assert snapshot.ready
        assert len(connects) == 1
        assert len(requests) == 4
        assert len({port for _path, port in requests}) == 1
    finally:
        if hasattr(client, "close"):
            client.close()


@pytest.fixture
def fault_http_server():
    """Loopback-only Moonraker substitute; never contacts a printer or robot."""
    state = SimpleNamespace(mode="healthy", requests=[], entered=threading.Event(), release=threading.Event())

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def send_json(self, body, *, status=200, close=False, extra_length=0):
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body) + extra_length))
            if close:
                self.send_header("Connection", "close")
                self.close_connection = True
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            state.requests.append(("GET", self.path, self.client_address[1]))
            if self.path == "/printer/info":
                self.send_json(b'{"result":{"state":"shutdown","state_message":"stub stopped"}}')
                return
            mode = state.mode
            if mode == "blocked":
                state.entered.set()
                state.release.wait(2.)
            if mode == "disconnect":
                self.close_connection = True
                return
            body = json.dumps(motion_response()).encode("utf-8")
            if mode == "invalid":
                body = b"invalid JSON"
            elif mode == "oversize":
                body = b" " * (64 * 1024 + 1)
            self.send_json(body, status=503 if mode == "http503" else 200,
                           close=mode in {"close", "truncated"},
                           extra_length=20 if mode == "truncated" else 0)

        def do_POST(self):
            state.requests.append(("POST", self.path, self.client_address[1]))
            self.send_json(b'{"result":"ok"}')

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", state
    finally:
        state.release.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


@pytest.mark.parametrize("mode,code,phase", [
    ("disconnect", "NETWORK_ERROR", "headers"),
    ("truncated", "NETWORK_ERROR", "body"),
    ("invalid", "INVALID_RESPONSE", None),
    ("oversize", "INVALID_RESPONSE", None),
    ("http503", "HTTP_ERROR", None),
])
def test_runtime_transport_discards_failed_socket_without_hidden_retry(fault_http_server, mode, code, phase):
    address, state = fault_http_server
    client = MoonrakerClient(address)
    try:
        client.read_motion_status(motion_reference(), timeout_s=.5)
        state.mode = mode
        with pytest.raises(MoonrakerError) as caught:
            client.read_motion_status(motion_reference(), timeout_s=.5)
        assert caught.value.code == code
        if phase:
            assert f"during {phase}" in str(caught.value)
            assert "reused_connection=True" in str(caught.value)
        assert len(state.requests) == 2, "the transport must never retry behind the budget owner's back"
        assert state.requests[0][2] == state.requests[1][2]
        state.mode = "healthy"
        snapshot, _ = client.read_motion_status(motion_reference(), timeout_s=.5)
        assert snapshot.ready
        assert state.requests[-1][2] != state.requests[0][2]
    finally:
        client.close()


def test_runtime_transport_respects_server_close_and_explicit_client_close(fault_http_server):
    address, state = fault_http_server
    client = MoonrakerClient(address)
    try:
        state.mode = "close"
        client.read_motion_status(motion_reference(), timeout_s=.5)
        state.mode = "healthy"
        client.read_motion_status(motion_reference(), timeout_s=.5)
        client.read_motion_status(motion_reference(), timeout_s=.5)
        assert state.requests[0][2] != state.requests[1][2]
        assert state.requests[1][2] == state.requests[2][2]
        client.close()
        client.read_motion_status(motion_reference(), timeout_s=.5)
        assert state.requests[-1][2] != state.requests[2][2]
    finally:
        client.close()


def test_emergency_and_confirmation_do_not_wait_for_blocked_runtime_socket(fault_http_server):
    address, state = fault_http_server
    client = MoonrakerClient(address, timeout_s=1.)
    errors = []
    state.mode = "blocked"

    def read_blocked_status():
        try:
            client.read_motion_status(motion_reference(), timeout_s=1.)
        except Exception as exc:
            errors.append(exc)

    reader = threading.Thread(target=read_blocked_status, daemon=True)
    reader.start()
    try:
        assert state.entered.wait(1.), "runtime GET must enter the local simulated stall"
        client.emergency_stop()
        stopped = client.read_printer_state(timeout_s=.5)
        assert stopped.state == "shutdown"
        assert reader.is_alive(), "the control channel must complete while the GET is still blocked"
        assert [r[0] for r in state.requests] == ["GET", "POST", "GET"]
        assert state.requests[1][1] == "/printer/emergency_stop"
        assert state.requests[0][2] != state.requests[1][2]
    finally:
        state.release.set()
        reader.join(timeout=1.)
        client.close()
    assert not errors


@pytest.mark.parametrize("phase", ["connect", "send", "headers", "body"])
def test_runtime_transport_reports_failure_phase_and_never_replays(monkeypatch, phase):
    calls = []

    class Connection:
        def __init__(self, *_args, **_kwargs):
            self.sock = SimpleNamespace(settimeout=lambda value: calls.append(("timeout", value)))

        def connect(self):
            calls.append(("connect", None))
            if phase == "connect":
                raise TimeoutError("simulated SYN loss")

        def request(self, method, *_args, **_kwargs):
            calls.append((method, None))
            if phase == "send":
                raise TimeoutError("simulated send stall")

        def getresponse(self):
            if phase == "headers":
                raise TimeoutError("simulated response stall")
            return SimpleNamespace(status=200, read=self.read, close=lambda: None)

        def read(self, _size):
            raise TimeoutError("simulated body stall")

        def close(self):
            calls.append(("close", None))

    monkeypatch.setattr(urllib.request, "getproxies", lambda: {})
    monkeypatch.setattr(http.client, "HTTPConnection", Connection)
    client = MoonrakerClient("http://127.0.0.1:43210")
    with pytest.raises(MoonrakerError) as caught:
        client.read_motion_status(motion_reference(), timeout_s=.3)
    assert caught.value.code == "NETWORK_ERROR"
    assert f"during {phase}" in str(caught.value)
    assert sum(name == "connect" for name, _ in calls) == 1
    assert sum(name == "GET" for name, _ in calls) <= 1
    assert calls[-1][0] == "close"


def test_configured_proxy_keeps_existing_opener_path(monkeypatch):
    opener = QueueOpener([motion_response()])
    monkeypatch.setattr(urllib.request, "getproxies", lambda: {"http": "http://proxy.invalid:8080"})
    monkeypatch.setattr(urllib.request, "proxy_bypass", lambda _host: False)
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    client = MoonrakerClient("http://printer.invalid")
    snapshot, _ = client.read_motion_status(motion_reference(), timeout_s=.25)
    assert snapshot.ready
    assert len(opener.requests) == 1
    assert opener.requests[0][1] == .25
    assert opener.requests[0][0].get_method() == "GET"
    client.close()


@pytest.mark.parametrize("scheme", ["http", "https"])
def test_reused_socket_gets_each_reads_current_timeout(monkeypatch, scheme):
    sockets = []
    constructors = []

    class Connection:
        def __init__(self, host, port, *, timeout):
            constructors.append((host, port, timeout))
            self.sock = SimpleNamespace(settimeout=lambda value: sockets.append(value))

        def connect(self):
            pass

        def request(self, method, *_args, **_kwargs):
            assert method == "GET"

        def getresponse(self):
            return SimpleNamespace(status=200, read=lambda _size: json.dumps(motion_response()).encode(),
                                   length=0, will_close=False, close=lambda: None)

        def close(self):
            pass

    monkeypatch.setattr(urllib.request, "getproxies", lambda: {})
    monkeypatch.setattr(http.client, "HTTPSConnection" if scheme == "https" else "HTTPConnection", Connection)
    client = MoonrakerClient(f"{scheme}://printer.invalid:43210")
    client.read_motion_status(motion_reference(), timeout_s=1.)
    previous_count = len(sockets)
    client.read_motion_status(motion_reference(), timeout_s=.1)
    assert len(constructors) == 1
    assert constructors[0][:2] == ("printer.invalid", 43210)
    assert sockets[previous_count:]
    assert all(0. < timeout <= .1 for timeout in sockets[previous_count:])
    client.close()
