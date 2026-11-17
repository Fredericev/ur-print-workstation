#!/bin/zsh
set -euo pipefail
export LC_ALL=C LANG=C

SOFTWARE_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SOFTWARE_DIR/.." && pwd)"
WINDOWS_APP="$SOFTWARE_DIR/windows_app"
BUILD_ROOT="$SOFTWARE_DIR/runtime/windows-build"
DOTNET_ROOT_LOCAL="$BUILD_ROOT/dotnet"
CACHE_DIR="$BUILD_ROOT/cache"
PUBLISH_ROOT="$BUILD_ROOT/publish"
STAGE_ROOT="$BUILD_ROOT/stage"
PAYLOAD_SNAPSHOT="$BUILD_ROOT/project-payload"
DIST_ROOT="$SOFTWARE_DIR/Windows"
PROJECT_FILE="$WINDOWS_APP/URPrintWorkstation.Windows.csproj"
APP_VERSION="$(/usr/bin/sed -nE 's/^VERSION = "([0-9]+\.[0-9]+\.[0-9]+)"$/\1/p' "$SOFTWARE_DIR/control_station.py")"
if [[ -z "$APP_VERSION" ]] \
   || ! /usr/bin/grep -Fq "<Version>$APP_VERSION</Version>" "$PROJECT_FILE" \
   || ! /usr/bin/grep -Fq "<FileVersion>$APP_VERSION.0</FileVersion>" "$PROJECT_FILE" \
   || ! /usr/bin/grep -Fq "<AssemblyVersion>$APP_VERSION.0</AssemblyVersion>" "$PROJECT_FILE" \
   || ! /usr/bin/grep -Fq "UR-Print-Workstation-$APP_VERSION-win-x64.zip" "$WINDOWS_APP/README-WINDOWS.txt" \
   || ! /usr/bin/grep -Fq "UR-Print-Workstation-$APP_VERSION-win-arm64.zip" "$WINDOWS_APP/README-WINDOWS.txt"; then
  echo "Backend, Windows launcher, and package guide versions do not match; update them before packaging." >&2
  exit 65
fi

DOTNET_VERSION="${DOTNET_VERSION:-10.0.302}"
PYTHON_VERSION="${PYTHON_VERSION:-3.13.14}"
PYTHON_X64_SHA256="2dcbfaca0c0742b6201209e5dbfc129ef5602e44544d6313b09eabc3931cdf65"
PYTHON_ARM64_SHA256="ba81a3db4de19a305bfdf67caaefa088db4241a51de7d120b1aa95f8d8cf2a7d"

mkdir -p "$BUILD_ROOT" "$CACHE_DIR" "$PUBLISH_ROOT" "$STAGE_ROOT" "$DIST_ROOT"

if command -v dotnet >/dev/null 2>&1 \
   && [[ "$(dotnet --version 2>/dev/null)" == "$DOTNET_VERSION" ]]; then
  DOTNET="$(command -v dotnet)"
else
  DOTNET="$DOTNET_ROOT_LOCAL/dotnet"
  if [[ ! -x "$DOTNET" ]] || [[ "$("$DOTNET" --version 2>/dev/null)" != "$DOTNET_VERSION" ]]; then
    INSTALL_SCRIPT="$CACHE_DIR/dotnet-install.sh"
    echo "Downloading the official .NET SDK installer..."
    /usr/bin/curl \
      --proto '=https' \
      --tlsv1.2 \
      --fail \
      --location \
      --retry 3 \
      "https://dot.net/v1/dotnet-install.sh" \
      --output "$INSTALL_SCRIPT"
    /bin/chmod 755 "$INSTALL_SCRIPT"
    case "$(/usr/bin/uname -m)" in
      arm64) SDK_ARCHITECTURE="arm64" ;;
      x86_64) SDK_ARCHITECTURE="x64" ;;
      *) echo "A 64-bit Mac build computer is required." >&2; exit 65 ;;
    esac
    "$INSTALL_SCRIPT" \
      --version "$DOTNET_VERSION" \
      --architecture "$SDK_ARCHITECTURE" \
      --install-dir "$DOTNET_ROOT_LOCAL" \
      --no-path
  fi
fi

download_python() {
  local architecture="$1"
  local expected_sha="$2"
  local archive="$CACHE_DIR/python-$PYTHON_VERSION-embeddable-$architecture.zip"
  local url="https://www.python.org/ftp/python/$PYTHON_VERSION/python-$PYTHON_VERSION-embeddable-$architecture.zip"

  if [[ ! -f "$archive" ]] || \
     [[ "$(/usr/bin/shasum -a 256 "$archive" | /usr/bin/awk '{print $1}')" != "$expected_sha" ]]; then
    /bin/rm -f "$archive"
    echo "Downloading official Python $PYTHON_VERSION for Windows $architecture..."
    /usr/bin/curl \
      --proto '=https' \
      --tlsv1.2 \
      --fail \
      --location \
      --retry 3 \
      "$url" \
      --output "$archive"
  fi

  local actual_sha
  actual_sha="$(/usr/bin/shasum -a 256 "$archive" | /usr/bin/awk '{print $1}')"
  if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "Python archive checksum mismatch for $architecture." >&2
    echo "Expected: $expected_sha" >&2
    echo "Actual:   $actual_sha" >&2
    exit 74
  fi
  echo "$archive"
}

copy_snapshot_tree() {
  local source="$1" destination="$2"
  shift 2
  mkdir -p "$destination"
  # Keep publication independent of macOS /usr/bin/rsync, which can stall
  # under File Provider storage. Both directories are newly staged.
  COPYFILE_DISABLE=1 /usr/bin/tar -C "$source" \
    --exclude='.git' --exclude='.github' \
    --exclude='.DS_Store' --exclude='.pytest_cache' \
    --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='verify_team_release.py' "$@" -cf - . \
    | COPYFILE_DISABLE=1 /usr/bin/tar -C "$destination" -xf -
}

copy_project_payload() {
  local destination="$1"
  local software_payload="$destination/Software"
  local workspace_payload="$destination/workspaces/ur_ros2_ws"

  mkdir -p \
    "$software_payload" \
    "$workspace_payload" \
    "$destination/docker" \
    "$destination/scripts" \
    "$destination/real_robot_files"

  /bin/cp "$PROJECT_ROOT/docker-compose.real.yml" "$destination/"
  copy_snapshot_tree "$PROJECT_ROOT/docker" "$destination/docker" --exclude='.DS_Store'
  copy_snapshot_tree "$PROJECT_ROOT/scripts" "$destination/scripts" --exclude='.DS_Store' --exclude='__pycache__'
  copy_snapshot_tree "$PROJECT_ROOT/real_robot_files" "$destination/real_robot_files" --exclude='.DS_Store'
  copy_snapshot_tree "$PROJECT_ROOT/workspaces/ur_ros2_ws/src" "$workspace_payload/src" --exclude='.DS_Store' --exclude='.git' --exclude='__pycache__'
  copy_snapshot_tree "$PROJECT_ROOT/workspaces/ur_ros2_ws/config" "$workspace_payload/config" --exclude='.DS_Store'
  copy_snapshot_tree "$PROJECT_ROOT/workspaces/ur_ros2_ws/scripts" "$workspace_payload/scripts" --exclude='.DS_Store' --exclude='__pycache__'

  # Match the Mac package: every shared Python module belongs to the payload.
  # Config/calibration JSON and generated runtime directories stay per-user.
  /bin/cp "$SOFTWARE_DIR"/*.py "$SOFTWARE_DIR/README.md" "$software_payload/"
  copy_snapshot_tree "$SOFTWARE_DIR/app" "$software_payload/app" --exclude='.DS_Store'

  # A changed support module, UI asset, or ROS file must get a new writable
  # project directory. Hash the complete snapshot, not a few entry points.
  local payload_hash
  payload_hash="$(
    cd "$destination"
    /usr/bin/find . -type f ! -path './PAYLOAD_VERSION.txt' -print0 \
      | LC_ALL=C /usr/bin/sort -z \
      | /usr/bin/xargs -0 /usr/bin/shasum -a 256 \
      | /usr/bin/sed 's/  \.\//  /' \
      | /usr/bin/shasum -a 256 \
      | /usr/bin/awk '{print substr($1,1,20)}'
  )"
  /usr/bin/printf '%s-%s\n' "$APP_VERSION" "$payload_hash" \
    > "$destination/PAYLOAD_VERSION.txt"
}

build_architecture() {
  local runtime_identifier="$1"
  local python_architecture="$2"
  local python_sha="$3"
  local publish_dir="$PUBLISH_ROOT/$runtime_identifier"
  local package_name="UR-Print-Workstation-$APP_VERSION-$runtime_identifier"
  local stage_dir="$STAGE_ROOT/$package_name"
  local python_archive

  python_archive="$(download_python "$python_architecture" "$python_sha" | /usr/bin/tail -n 1)"

  echo "Publishing native Windows launcher for $runtime_identifier..."
  /bin/rm -rf "$publish_dir" "$stage_dir"
  mkdir -p "$publish_dir" "$stage_dir/python" "$stage_dir/project"

  "$DOTNET" publish "$PROJECT_FILE" \
    --configuration Release \
    --runtime "$runtime_identifier" \
    --self-contained true \
    --output "$publish_dir" \
    -p:EnableWindowsTargeting=true \
    -p:PublishSingleFile=false \
    -p:DebugType=None \
    -p:DebugSymbols=false

  copy_snapshot_tree "$publish_dir" "$stage_dir" --exclude='*.pdb' --exclude='*.xml'
  /usr/bin/unzip -q "$python_archive" -d "$stage_dir/python"

  # The embeddable distribution deliberately isolates sys.path. Add only the
  # packaged Software directory so control_station.py can import its modules.
  /usr/bin/printf \
    "python313.zip\n.\n../project/Software\n" \
    > "$stage_dir/python/python313._pth"

  copy_snapshot_tree "$PAYLOAD_SNAPSHOT" "$stage_dir/project"
  /bin/cp \
    "$WINDOWS_APP/README-WINDOWS.txt" \
    "$WINDOWS_APP/THIRD-PARTY-NOTICES.txt" \
    "$WINDOWS_APP/Replay Offline Print.cmd" \
    "$stage_dir/"

  /usr/bin/find "$stage_dir" \
    -name '.DS_Store' -o \
    -name '__pycache__' -o \
    -name '*.pyc' \
    | while IFS= read -r unwanted; do
        /bin/rm -rf "$unwanted"
      done

  local manifest="$stage_dir/SHA256SUMS.txt"
  (
    cd "$stage_dir"
    /usr/bin/find . -type f ! -path './SHA256SUMS.txt' -print0 \
      | /usr/bin/sort -z \
      | /usr/bin/xargs -0 /usr/bin/shasum -a 256 \
      | /usr/bin/sed 's/  \.\//  /'
  ) > "$manifest"

  local archive="$DIST_ROOT/$package_name.zip"
  /bin/rm -f "$archive" "$archive.sha256"
  (
    cd "$STAGE_ROOT"
    /usr/bin/zip -q -r -X "$archive" "$package_name"
  )
  local archive_hash
  archive_hash="$(/usr/bin/shasum -a 256 "$archive" | /usr/bin/awk '{print $1}')"
  /usr/bin/printf "%s  %s\n" "$archive_hash" "$package_name.zip" \
    > "$archive.sha256"

  echo "Created:"
  echo "  $archive"
  echo "  $archive.sha256"
}

# Published version names are immutable, like the Mac portable package.
for runtime_identifier in win-x64 win-arm64; do
  archive="$DIST_ROOT/UR-Print-Workstation-$APP_VERSION-$runtime_identifier.zip"
  if [[ -e "$archive" || -e "$archive.sha256" ]]; then
    echo "Release $archive already exists; use a new version for a new release." >&2
    exit 65
  fi
done

# Capture the project exactly once. Both architecture packages must contain the
# same application sources even if a developer edits the working tree while a
# long cross-platform build is running.
/bin/rm -rf "$PAYLOAD_SNAPSHOT"
mkdir -p "$PAYLOAD_SNAPSHOT"
copy_project_payload "$PAYLOAD_SNAPSHOT"

build_architecture "win-x64" "amd64" "$PYTHON_X64_SHA256"
build_architecture "win-arm64" "arm64" "$PYTHON_ARM64_SHA256"
/bin/cp "$WINDOWS_APP/README-WINDOWS.txt" "$DIST_ROOT/README-WINDOWS.txt"

echo
echo "Windows packages are ready in:"
echo "  $DIST_ROOT"
