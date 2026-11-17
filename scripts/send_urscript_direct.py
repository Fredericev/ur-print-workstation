#!/usr/bin/env python3
import argparse
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="Send URScript directly to a UR controller port.")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--port", type=int, default=30002)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--script-file")
    parser.add_argument("--script")
    parser.add_argument("--linger", type=float, default=0.2)
    args = parser.parse_args()

    if bool(args.script_file) == bool(args.script):
        print("Provide exactly one of --script-file or --script.", file=sys.stderr)
        return 64

    if args.script_file:
        with open(args.script_file, "r", encoding="utf-8") as handle:
            script = handle.read()
    else:
        script = args.script

    if not script.endswith("\n"):
        script += "\n"

    payload = script.encode("utf-8")
    with socket.create_connection((args.robot_ip, args.port), timeout=args.timeout) as sock:
        sock.sendall(payload)
        time.sleep(args.linger)
    print(f"Sent {len(payload)} bytes of URScript to {args.robot_ip}:{args.port}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
