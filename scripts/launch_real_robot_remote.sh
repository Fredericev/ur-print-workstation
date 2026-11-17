#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

HEADLESS_MODE=true "$ROOT_DIR/scripts/launch_real_robot.sh" "$@"
