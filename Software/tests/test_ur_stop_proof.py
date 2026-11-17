"""Read-only, hardware-isolated RTDE stop-proof tests."""

from __future__ import annotations

import socket
import struct
import threading
import time
import unittest

from Software.ur_stop_proof import verify_stationary_rtde


def _packet(kind: bytes, payload: bytes = b"") -> bytes:
    return struct.pack("!H", len(payload) + 3) + kind + payload


def _exact(connection: socket.socket, length: int) -> bytes:
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            raise RuntimeError("RTDE test peer closed early")
        data.extend(chunk)
    return bytes(data)


def _request(connection: socket.socket) -> tuple[bytes, bytes]:
    size = struct.unpack("!H", _exact(connection, 2))[0]
    packet = _exact(connection, size - 2)
    return packet[:1], packet[1:]


class FakeReadOnlyRtde:
    def __init__(self, *, velocity: float = 0.0, duplicate_timestamps: bool = False,
                 recipe_types: bytes = b"DOUBLE,VECTOR6D,VECTOR6D") -> None:
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.velocity = velocity
        self.duplicate_timestamps = duplicate_timestamps
        self.recipe_types = recipe_types
        self.requests: list[bytes] = []
        self.error: BaseException | None = None
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self) -> None:
        try:
            with self.listener:
                connection, _ = self.listener.accept()
                with connection:
                    connection.settimeout(2)
                    kind, payload = _request(connection)
                    self.requests.append(kind)
                    assert kind == b"V" and payload == b"\x00\x02"
                    connection.sendall(_packet(b"V", b"\x01"))
                    kind, payload = _request(connection)
                    self.requests.append(kind)
                    assert kind == b"O" and payload[8:] == b"timestamp,actual_q,actual_qd"
                    connection.sendall(_packet(b"O", b"\x01" + self.recipe_types))
                    if self.recipe_types != b"DOUBLE,VECTOR6D,VECTOR6D":
                        return
                    kind, payload = _request(connection)
                    self.requests.append(kind)
                    assert kind == b"S" and not payload
                    connection.sendall(_packet(b"S", b"\x01"))
                    for index in range(16):
                        timestamp = 1.0 if self.duplicate_timestamps else 1.0 + index * 0.01
                        values = (timestamp, *([0.2] * 6), *([self.velocity] * 6))
                        try:
                            connection.sendall(_packet(b"U", b"\x01" + struct.pack("!13d", *values)))
                        except (BrokenPipeError, ConnectionResetError):
                            break
                        time.sleep(0.01)
        except BaseException as exc:
            self.error = exc

    def finish(self) -> None:
        self.thread.join(timeout=3)
        if self.error:
            raise self.error
        if self.thread.is_alive():
            raise AssertionError("RTDE fake server did not stop")


class UrStopProofTests(unittest.TestCase):
    def _check(self, server: FakeReadOnlyRtde) -> dict:
        try:
            return verify_stationary_rtde(
                "127.0.0.1", port=server.port, timeout_s=0.6,
                dwell_s=0.05, minimum_samples=4,
            )
        finally:
            server.finish()

    def test_fresh_stationary_samples_are_accepted_without_input_recipe(self) -> None:
        server = FakeReadOnlyRtde()
        result = self._check(server)
        self.assertTrue(result["stationary"])
        self.assertGreaterEqual(result["sample_count"], 4)
        self.assertEqual(server.requests, [b"V", b"O", b"S"])

    def test_moving_joints_never_prove_stationarity(self) -> None:
        result = self._check(FakeReadOnlyRtde(velocity=0.1))
        self.assertFalse(result["stationary"])

    def test_duplicate_robot_timestamps_never_count_as_fresh_samples(self) -> None:
        result = self._check(FakeReadOnlyRtde(duplicate_timestamps=True))
        self.assertFalse(result["stationary"])

    def test_unexpected_recipe_type_is_rejected(self) -> None:
        with self.assertRaises(RuntimeError):
            self._check(FakeReadOnlyRtde(recipe_types=b"DOUBLE,NOT_FOUND,VECTOR6D"))


if __name__ == "__main__":
    unittest.main()
