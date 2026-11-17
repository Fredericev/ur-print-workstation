#!/usr/bin/env python3
import argparse
import socket
import sys


def read_available(sock: socket.socket) -> str:
    chunks = []
    sock.settimeout(0.5)
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            break
        if not data:
            break
        chunks.append(data)
        if len(data) < 4096:
            break
    return b"".join(chunks).decode("utf-8", errors="replace").strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="Send one command to a UR Dashboard Server.")
    parser.add_argument("robot_ip")
    parser.add_argument("command", nargs="+")
    parser.add_argument("--port", type=int, default=29999)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    command = " ".join(args.command).strip()
    if not command:
        print("empty command", file=sys.stderr)
        return 64

    with socket.create_connection((args.robot_ip, args.port), timeout=args.timeout) as sock:
        banner = read_available(sock)
        if banner:
            print(banner)
        sock.sendall((command + "\n").encode("utf-8"))
        response = read_available(sock)
        if response:
            print(response)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
