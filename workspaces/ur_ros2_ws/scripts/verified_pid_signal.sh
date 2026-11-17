#!/usr/bin/env bash
set -euo pipefail

pid_file="${1:-}"
expected_pattern="${2:-}"
requested_signal="${3:-TERM}"

if [[ -z "$pid_file" || -z "$expected_pattern" ]]; then
  echo "Usage: verified_pid_signal.sh PID_FILE EXPECTED_PATTERN [SIGNAL]" >&2
  exit 64
fi
if [[ ! -f "$pid_file" ]]; then
  exit 0
fi

pid="$(cat "$pid_file" 2>/dev/null || true)"
if [[ ! "$pid" =~ ^[0-9]+$ ]] || ! kill -0 "$pid" 2>/dev/null; then
  rm -f "$pid_file"
  exit 0
fi

if [[ ! -r "/proc/$pid/cmdline" ]]; then
  echo "Refusing to signal PID $pid: its command line is unavailable." >&2
  exit 65
fi
command_line="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
if ! printf '%s\n' "$command_line" | grep -Eq -- "$expected_pattern"; then
  echo "Ignoring stale PID file $pid_file: PID $pid belongs to another process." >&2
  rm -f "$pid_file"
  exit 0
fi

kill "-$requested_signal" "$pid"
