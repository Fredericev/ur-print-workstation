#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COLIMA_SSH_PORT="${COLIMA_SSH_PORT:-2222}"
COLIMA_CONFIG="$HOME/.colima/default/colima.yaml"
COLIMA_SSH_CONFIG="$HOME/.colima/_lima/colima/ssh.config"
UR_REVERSE_PORTS=(50001 50002 50003 50004)

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is not installed. Install Homebrew before continuing."
  exit 1
fi

missing=()
for pkg in colima docker docker-buildx docker-compose; do
  if ! brew list "$pkg" >/dev/null 2>&1; then
    missing+=("$pkg")
  fi
done

if ((${#missing[@]} > 0)); then
  echo "Installing: ${missing[*]}"
  brew install "${missing[@]}"
fi

# Homebrew's docker formula now ships its own shell completions.  Machines
# that still have the older, separate docker-completion formula linked can
# finish an install/upgrade with the Docker keg present but no docker command
# in Homebrew's bin directory.  Repair that known conflict before attempting
# to start Colima.
runtime_brew_prefix="$(brew --prefix)"
export PATH="$runtime_brew_prefix/bin:$runtime_brew_prefix/sbin:$PATH"
if ! command -v docker >/dev/null 2>&1 && brew list docker >/dev/null 2>&1; then
  echo "Docker is installed but not linked; repairing Homebrew links..."
  if brew list docker-completion >/dev/null 2>&1; then
    brew unlink docker-completion
  fi
  brew link docker
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker CLI is installed but unavailable on PATH."
  exit 1
fi

mkdir -p "$HOME/.docker"
UR_PRINT_DOCKER_PLUGIN_DIR="$runtime_brew_prefix/lib/docker/cli-plugins" python3 - <<'PY'
import json
import os
from pathlib import Path
cfg_path = Path.home() / '.docker' / 'config.json'
try:
    cfg = json.loads(cfg_path.read_text()) if cfg_path.exists() else {}
except Exception:
    cfg = {}
extra = cfg.get('cliPluginsExtraDirs') or []
plugin_dir = os.environ['UR_PRINT_DOCKER_PLUGIN_DIR']
if plugin_dir not in extra:
    extra.append(plugin_dir)
cfg['cliPluginsExtraDirs'] = extra
cfg_path.write_text(json.dumps(cfg, indent=2) + '\n')
PY

configure_colima_ssh_port() {
  if [[ ! -f "$COLIMA_CONFIG" ]]; then
    return 0
  fi

  COLIMA_CONFIG="$COLIMA_CONFIG" COLIMA_SSH_PORT="$COLIMA_SSH_PORT" python3 - <<'PY'
import os
import re
from pathlib import Path

path = Path(os.environ["COLIMA_CONFIG"])
safe_port = os.environ["COLIMA_SSH_PORT"]
text = path.read_text()
updated, count = re.subn(r"(?m)^sshPort:[ \t]*\d+[ \t]*$", f"sshPort: {safe_port}", text, count=1)
if count == 0:
    raise SystemExit(f"Could not find sshPort in {path}")
if updated != text:
    path.write_text(updated)
    print(f"Reserved Colima SSH port {safe_port}; UR ports 50001-50004 remain available.")
PY
}

current_colima_ssh_port() {
  if [[ -f "$COLIMA_SSH_CONFIG" ]]; then
    awk '/^[[:space:]]*Port[[:space:]]+[0-9]+/{print $2; exit}' "$COLIMA_SSH_CONFIG"
  fi
}

colima_uses_ur_port() {
  local current_port
  local reserved_port
  current_port="$(current_colima_ssh_port)"
  for reserved_port in "${UR_REVERSE_PORTS[@]}"; do
    if [[ "$current_port" == "$reserved_port" ]]; then
      echo "Colima SSH currently occupies UR port $current_port. Checking whether it can be moved to $COLIMA_SSH_PORT..."
      return 0
    fi
  done
  return 1
}

start_colima() {
  case "$(uname -m)" in
    arm64)
      echo "Starting Apple Silicon Colima with Rosetta support for amd64 robot images..."
      if ! colima start --arch aarch64 --vm-type vz --vz-rosetta --cpu 4 --memory 8 --disk 60 --ssh-port "$COLIMA_SSH_PORT"; then
        echo "Rosetta-backed startup failed, retrying with native Colima defaults..."
        colima start --arch aarch64 --cpu 4 --memory 8 --disk 60 --ssh-port "$COLIMA_SSH_PORT"
      fi
      ;;
    x86_64)
      echo "Starting Intel Colima without Apple Silicon Rosetta flags..."
      colima start --arch x86_64 --cpu 4 --memory 8 --disk 60 --ssh-port "$COLIMA_SSH_PORT"
      ;;
    *)
      echo "Unsupported macOS CPU architecture: $(uname -m)" >&2
      return 1
      ;;
  esac
}

configure_colima_ssh_port

# Canceling a previous VM recovery can leave Docker's global selection on its
# default socket while Colima is healthy. Re-select the known local Colima
# endpoint instead of blindly restarting the VM again. Explicit user overrides
# remain authoritative.
restore_colima_context() {
  local endpoint
  local selected_context
  if [[ -n "${DOCKER_HOST:-}" || -n "${DOCKER_CONTEXT:-}" ]]; then
    return 1
  fi
  selected_context="$(docker context show 2>/dev/null)" || return 1
  [[ "$selected_context" == "default" || "$selected_context" == "colima" ]] || return 1
  endpoint="$(docker context inspect colima --format '{{.Endpoints.docker.Host}}' 2>/dev/null)" || return 1
  [[ "$endpoint" == "unix://$HOME/.colima/default/docker.sock" ]] || return 1
  docker --context colima version >/dev/null 2>&1 || return 1
  echo "Restoring the local Colima Docker connection after an interrupted stop/start..."
  docker context use colima
}

if ! docker version >/dev/null 2>&1; then
  restore_colima_context || true
fi

if ! colima status >/dev/null 2>&1; then
  start_colima
elif colima_uses_ur_port; then
  python3 "$ROOT_DIR/scripts/check_ros_container_network.py" --check-idle-colima
  colima stop
  start_colima
else
  echo "Colima is already running."
fi

if ! docker version >/dev/null 2>&1; then
  restore_colima_context || true
fi
if ! docker version >/dev/null 2>&1; then
  echo "Docker is still unavailable. Running workloads cannot be verified; automatic VM restart was not attempted." >&2
  exit 1
fi

docker version
docker compose version
