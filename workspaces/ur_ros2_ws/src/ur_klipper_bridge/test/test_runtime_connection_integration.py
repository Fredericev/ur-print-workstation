"""Real HTTP transport + real bridge callback, with loopback and inert motion."""

import http.client
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from test_bridge_timing import bridge_type, make_case, make_program_case
from ur_klipper_bridge.moonraker import MoonrakerClient


@pytest.mark.parametrize("disconnect_once,stale_before_start", [
    (False, False), (True, False), (True, True),
])
def test_queued_program_retains_one_dispatch_across_connection_reuse_and_reconnect(
    bridge_type, monkeypatch, disconnect_once, stale_before_start,
):
    bridge, goal, inert, _clock = make_program_case(bridge_type)
    state = SimpleNamespace(ports=[], reads=0, active_reads=0, dispatched=[], disconnected=False)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def do_GET(self):
            assert self.path.startswith("/printer/objects/query?webhooks=")
            state.ports.append(self.client_address[1])
            state.reads += 1
            state.dispatched.append(len(inert.starts))
            if disconnect_once and state.reads == 2:
                state.disconnected = True
                self.close_connection = True
                return
            if inert.starts:
                state.active_reads += 1
            status = {
                "webhooks": {"state": "ready", "state_message": "Loopback simulation"},
                "gcode_move": {"speed_factor": 1., "extrude_factor": 1.},
                "toolhead": {"max_accel": 1000., "max_velocity": 1000., "minimum_cruise_ratio": .5},
                "extruder": {"temperature": 200., "target": 200., "can_extrude": True},
                "print_stats": {
                    "state": ("complete" if state.active_reads >= 4 else "printing") if inert.starts else "standby",
                    "filename": inert.starts[0] if inert.starts else "", "message": "",
                    "print_duration": float(state.reads), "total_duration": float(state.reads),
                },
            }
            body = json.dumps({"result": {"status": status}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    original_connect = http.client.HTTPConnection.connect
    connects = []

    def restrict_new_connections(connection):
        connects.append(connection)
        permitted = 2 if disconnect_once else 1
        if len(connects) > permitted:
            raise TimeoutError("simulated subsequent TCP connect failure")
        original_connect(connection)

    monkeypatch.setattr(http.client.HTTPConnection, "connect", restrict_new_connections)
    monkeypatch.setattr("urllib.request.getproxies", lambda: {})
    client = MoonrakerClient(f"http://127.0.0.1:{server.server_port}")
    inert.read_motion_status = client.read_motion_status
    bridge._request_timeout_s = 8.
    try:
        if stale_before_start:
            # Leave the previous action's pooled socket in place, then have
            # the server close it on the new action's preflight request.
            client.read_motion_status(inert.read_status(), include_print_stats=True, timeout_s=.5)
        result = bridge._program_execute_callback(goal)
        assert result.success
        assert goal.terminal == "succeeded"
        assert len(inert.uploads) == len(inert.starts) == 1
        assert inert.emergency_count == 0
        assert state.active_reads == 4
        assert state.dispatched[0] == 0
        assert state.reads == 5 + int(disconnect_once) + int(stale_before_start)
        assert len(set(state.ports)) == (2 if disconnect_once else 1)
        assert len(connects) == (2 if disconnect_once else 1)
        if disconnect_once:
            assert state.disconnected
        if stale_before_start:
            assert state.dispatched[:3] == [0, 0, 0], "stale socket must recover before upload/start"
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.)


@pytest.mark.parametrize("program", [False, True])
def test_real_monitor_connection_failure_is_rejected_before_any_dispatch(bridge_type, monkeypatch, program):
    bridge, goal, inert = (make_program_case(bridge_type) if program else make_case(bridge_type))[:3]
    attempts = []

    def unavailable_connection(_connection):
        attempts.append(bridge._motion_dispatched)
        raise TimeoutError("simulated loopback connection unavailable")

    monkeypatch.setattr(http.client.HTTPConnection, "connect", unavailable_connection)
    monkeypatch.setattr("urllib.request.getproxies", lambda: {})
    client = MoonrakerClient("http://127.0.0.1:43210")
    inert.read_motion_status = client.read_motion_status
    try:
        result = bridge._program_execute_callback(goal) if program else bridge._execute_callback(goal)
        assert result.success is False
        assert result.error_code == "NETWORK_ERROR"
        assert result.may_have_moved is False
        assert attempts == [False, False]
        assert inert.emergency_count == 0
        if program:
            assert inert.uploads == []
            assert inert.starts == []
        else:
            assert inert.scripts == []
    finally:
        client.close()
