#!/usr/bin/env python3
import argparse
import socket
import sys
import textwrap
import time


def build_script(host_ip, feedback_port, joint_index, amplitude, t1, t2, t3, accel, speed):
    return textwrap.dedent(
        f"""\
        def codex_direct_large_sweep_verify():
          socket_open("{host_ip}", {feedback_port}, "codex")
          sleep(0.200)
          q0 = get_actual_joint_positions()
          socket_send_line("start", "codex")
          socket_send_line(to_str(q0[{joint_index}]), "codex")

          q = q0
          q[{joint_index}] = q0[{joint_index}] + {amplitude:.6f}
          movej(q, a={accel:.6f}, v={speed:.6f}, t={t1:.6f}, r=0)
          q_now = get_actual_joint_positions()
          socket_send_line("plus", "codex")
          socket_send_line(to_str(q_now[{joint_index}]), "codex")

          q = q0
          q[{joint_index}] = q0[{joint_index}] - {amplitude:.6f}
          movej(q, a={accel:.6f}, v={speed:.6f}, t={t2:.6f}, r=0)
          q_now = get_actual_joint_positions()
          socket_send_line("minus", "codex")
          socket_send_line(to_str(q_now[{joint_index}]), "codex")

          movej(q0, a={accel:.6f}, v={speed:.6f}, t={t3:.6f}, r=0)
          q_now = get_actual_joint_positions()
          socket_send_line("return", "codex")
          socket_send_line(to_str(q_now[{joint_index}]), "codex")
          socket_close("codex")
        end
        codex_direct_large_sweep_verify()
        """
    )


def send_urscript(robot_ip, robot_port, script, timeout):
    payload = script.encode("utf-8")
    with socket.create_connection((robot_ip, robot_port), timeout=timeout) as sock:
        sock.sendall(payload)
        time.sleep(0.2)
    print(f"Sent {len(payload)} bytes of URScript to {robot_ip}:{robot_port}.", flush=True)


def parse_feedback(lines):
    pairs = {}
    if len(lines) % 2 != 0:
        raise RuntimeError(f"Expected label/value feedback pairs, got: {lines!r}")
    for index in range(0, len(lines), 2):
        label = lines[index].strip()
        value = float(lines[index + 1].strip())
        pairs[label] = value
    return pairs


def main():
    parser = argparse.ArgumentParser(description="Verify direct URScript control with a large UR3e sweep.")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--host-ip", required=True)
    parser.add_argument("--robot-port", type=int, default=30002)
    parser.add_argument("--feedback-port", type=int, default=50010)
    parser.add_argument("--joint-index", type=int, default=4)
    parser.add_argument("--amplitude-rad", type=float, default=0.35)
    parser.add_argument("--verify-min-rad", type=float, default=0.25)
    parser.add_argument("--segment1-sec", type=float, default=15.0)
    parser.add_argument("--segment2-sec", type=float, default=30.0)
    parser.add_argument("--segment3-sec", type=float, default=15.0)
    parser.add_argument("--accel", type=float, default=0.20)
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.joint_index < 0 or args.joint_index > 5:
        print("joint-index must be 0..5", file=sys.stderr)
        return 64
    if args.amplitude_rad <= 0.0 or args.amplitude_rad > 0.50:
        print("amplitude-rad must be > 0 and <= 0.50", file=sys.stderr)
        return 65
    total_motion = args.segment1_sec + args.segment2_sec + args.segment3_sec
    if total_motion < 60.0:
        print("total segment time must be at least 60 seconds", file=sys.stderr)
        return 66

    script = build_script(
        args.host_ip,
        args.feedback_port,
        args.joint_index,
        args.amplitude_rad,
        args.segment1_sec,
        args.segment2_sec,
        args.segment3_sec,
        args.accel,
        args.speed,
    )

    print("Direct URScript large sweep verification")
    print(f"Robot: {args.robot_ip}:{args.robot_port}")
    print(f"Feedback server: {args.host_ip}:{args.feedback_port}")
    print(f"Joint index: {args.joint_index}")
    print(f"Amplitude: +/-{args.amplitude_rad:.3f} rad")
    print(f"Motion time: {total_motion:.1f} s")
    if args.dry_run:
        print(script)
        return 0

    deadline = total_motion + 45.0
    lines = []
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", args.feedback_port))
        server.listen(1)
        server.settimeout(args.connect_timeout)
        send_urscript(args.robot_ip, args.robot_port, script, args.connect_timeout)
        print("Waiting for robot feedback connection...", flush=True)
        try:
            conn, addr = server.accept()
        except socket.timeout as exc:
            raise RuntimeError(
                "Robot did not connect back. Put the UR3e in Remote Control mode and stop the External Control program."
            ) from exc
        print(f"Robot feedback connected from {addr[0]}:{addr[1]}", flush=True)
        with conn:
            conn.settimeout(deadline)
            buffer = b""
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        lines.append(line)
                        print(f"feedback: {line}", flush=True)

    feedback = parse_feedback(lines)
    required = {"start", "plus", "minus", "return"}
    missing = sorted(required.difference(feedback))
    if missing:
        raise RuntimeError(f"Missing feedback labels: {', '.join(missing)}")

    start = feedback["start"]
    plus_delta = abs(feedback["plus"] - start)
    minus_delta = abs(feedback["minus"] - start)
    return_delta = abs(feedback["return"] - start)
    print(f"Observed + sweep delta: {plus_delta:.6f} rad")
    print(f"Observed - sweep delta: {minus_delta:.6f} rad")
    print(f"Observed return error: {return_delta:.6f} rad")

    if plus_delta < args.verify_min_rad or minus_delta < args.verify_min_rad:
        raise RuntimeError(
            f"Motion verification failed: observed deltas {plus_delta:.6f}, {minus_delta:.6f} "
            f"< required {args.verify_min_rad:.6f} rad"
        )
    print("Direct computer control verified by robot-reported joint positions.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
