#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker-compose.real.yml"

cd "$ROOT_DIR"

compose_status=0
docker compose -f "$COMPOSE_FILE" stop ros2 || compose_status=$?

# The executor is intentionally launched as a detached process so it can
# outlive an HTTP request.  If the backend crashed, it may also outlive the
# app; reuse the backend's exact-path process matcher instead of a broad pkill.
executor_status=0
UR_PRINT_PROJECT_ROOT="$ROOT_DIR" \
PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
python3 -c '
import json
from Software.control_station import stop_project_executor_processes

result = stop_project_executor_processes()
print(json.dumps(result, sort_keys=True))
raise SystemExit(1 if result.get("remaining_pids") else 0)
' || executor_status=$?

if (( compose_status != 0 || executor_status != 0 )); then
  echo "Local runtime cleanup was incomplete (compose=$compose_status, executor=$executor_status)." >&2
  exit 1
fi
