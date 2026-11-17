#!/usr/bin/env python3
"""Run one child process and bound its combined stdout/stderr on disk."""

from __future__ import annotations

import argparse
import logging
import logging.handlers
import os
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any


class RotatingTextSink:
    """Small raw-line facade over the standard rotating-file handler."""

    def __init__(self, path: Path, *, max_bytes: int, backups: int, new_session: bool = False) -> None:
        if max_bytes < 1 or backups < 1:
            raise ValueError("max_bytes and backups must be positive")
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.is_file() and path.stat().st_size > max_bytes:
            with path.open("rb") as source:
                source.seek(-max_bytes, os.SEEK_END)
                retained = source.read()
            with path.open("wb") as target:
                target.write(retained)
        self._logger = logging.getLogger(f"bounded-log-{id(self)}")
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self._handler = logging.handlers.RotatingFileHandler(
            path,
            maxBytes=max_bytes,
            backupCount=backups,
            encoding="utf-8",
        )
        if new_session and path.stat().st_size:
            # Readiness must never match a success/error from an older launch.
            # Preserve that evidence in the existing bounded backup chain.
            self._handler.doRollover()
        self._handler.setFormatter(logging.Formatter("%(message)s"))
        self._logger.addHandler(self._handler)

    def write(self, line: str) -> None:
        self._logger.info(line.rstrip("\r\n"))

    def close(self) -> None:
        self._logger.removeHandler(self._handler)
        self._handler.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--max-bytes", type=int, default=10 * 1024 * 1024)
    parser.add_argument("--backups", type=int, default=3)
    parser.add_argument("--new-session", action="store_true")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a child command is required after --")
    if args.max_bytes < 1 or args.backups < 1:
        parser.error("--max-bytes and --backups must be positive")
    return args


def run(args: argparse.Namespace) -> int:
    sink = RotatingTextSink(
        args.log,
        max_bytes=args.max_bytes,
        backups=args.backups,
        new_session=args.new_session,
    )
    child: subprocess.Popen[str] | None = None

    def forward(received: int, _frame: Any) -> None:
        if child is None or child.poll() is not None:
            return
        try:
            os.killpg(child.pid, received)
        except OSError:
            child.terminate()

    signal.signal(signal.SIGTERM, forward)
    signal.signal(signal.SIGINT, forward)
    try:
        child = subprocess.Popen(
            args.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        assert child.stdout is not None
        for line in child.stdout:
            sink.write(line)
        return child.wait()
    except Exception as exc:
        sink.write(f"[workstation supervisor] {type(exc).__name__}: {exc}")
        return 127
    finally:
        sink.close()


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
