#!/bin/zsh
set -euo pipefail
export LC_ALL=C LANG=C

# Build a separate, relocatable pilot package. This script never installs or
# replaces the workstation currently used on this Mac.
SOFTWARE_DIR="${0:A:h}"
PROJECT_DIR="${SOFTWARE_DIR:h}"
DIST_DIR="$SOFTWARE_DIR/dist/mac"
PLIST="$SOFTWARE_DIR/native_app/Info.plist"
VERSION="$(/usr/libexec/PlistBuddy -c 'Print :CFBundleShortVersionString' "$PLIST")"
BACKEND_VERSION="$(/usr/bin/sed -n 's/^VERSION = "\([^"]*\)".*/\1/p' "$SOFTWARE_DIR/control_station.py")"
ARCH="$(/usr/bin/uname -m)"

if [[ "$ARCH" != "arm64" && "$ARCH" != "x86_64" ]]; then
  echo "Unsupported macOS architecture: $ARCH" >&2
  exit 1
fi
if [[ ! "$VERSION" =~ '^[0-9][0-9A-Za-z.-]*$' ]]; then
  echo "Invalid bundle version: $VERSION" >&2
  exit 1
fi
if [[ "$VERSION" != "$BACKEND_VERSION" ]]; then
  echo "Bundle version $VERSION does not match backend version $BACKEND_VERSION." >&2
  exit 1
fi
if [[ ! -s "$SOFTWARE_DIR/native_app/AppIcon.icns" ]]; then
  echo "Missing application icon." >&2
  exit 1
fi

PACKAGE_NAME="UR-Print-Workstation-$VERSION-mac-$ARCH"
PACKAGE_DIR="$DIST_DIR/$PACKAGE_NAME"
ARCHIVE="$DIST_DIR/$PACKAGE_NAME.zip"
CHECKSUM="$ARCHIVE.sha256"
if [[ -e "$PACKAGE_DIR" || -e "$ARCHIVE" || -e "$CHECKSUM" ]]; then
  echo "Release $PACKAGE_NAME already exists; use a new version for a new release." >&2
  exit 1
fi

/bin/mkdir -p "$DIST_DIR"
# Sign outside Desktop/File Provider storage: its FinderInfo metadata can
# invalidate codesign before the bundle is even archived.
STAGING="$(/usr/bin/mktemp -d "${TMPDIR:-/tmp}/ur-print-mac-stage.XXXXXXXX")"
STAGED_PACKAGE="$STAGING/$PACKAGE_NAME"
/bin/mkdir -p \
  "$STAGED_PACKAGE/Software/Mac" \
  "$STAGED_PACKAGE/Software/app" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws" \
  "$STAGED_PACKAGE/Test gcode"

copy_file() {
  local source="$1" destination="$2"
  if [[ ! -f "$source" ]]; then
    echo "Required release input is missing: $source" >&2
    exit 1
  fi
  /bin/cp -p "$source" "$destination"
}

copy_tree() {
  local source="$1" destination="$2"
  if [[ ! -d "$source" ]]; then
    echo "Required release directory is missing: $source" >&2
    exit 1
  fi
  /bin/mkdir -p "$destination"
  # The system rsync can block indefinitely under File Provider storage;
  # archive a clean tree instead. Do not carry source extended attributes.
  COPYFILE_DISABLE=1 /usr/bin/tar -C "$source" \
    --exclude='.git' \
    --exclude='.github' \
    --exclude='.pytest_cache' \
    --exclude='__pycache__' \
    --exclude='.DS_Store' \
    --exclude='*.pyc' \
    --exclude='verify_team_release.py' \
    -cf - . | COPYFILE_DISABLE=1 /usr/bin/tar -C "$destination" -xf -
}

# This is the complete source/runtime tree required by the existing Online
# and Offline paths. User settings and generated ROS/build/print history are
# deliberately not read from the developer machine.
for filename in README.md REAL_ROBOT.md ROS_ON_MACOS.md docker-compose.real.yml; do
  copy_file "$PROJECT_DIR/$filename" "$STAGED_PACKAGE/$filename"
done
copy_tree "$PROJECT_DIR/scripts" "$STAGED_PACKAGE/scripts"
copy_tree "$PROJECT_DIR/docker" "$STAGED_PACKAGE/docker"
copy_tree "$PROJECT_DIR/real_robot_files" "$STAGED_PACKAGE/real_robot_files"
copy_tree "$PROJECT_DIR/workspaces/ur_ros2_ws/src" "$STAGED_PACKAGE/workspaces/ur_ros2_ws/src"
copy_tree "$PROJECT_DIR/workspaces/ur_ros2_ws/scripts" "$STAGED_PACKAGE/workspaces/ur_ros2_ws/scripts"
copy_tree "$PROJECT_DIR/workspaces/ur_ros2_ws/config" "$STAGED_PACKAGE/workspaces/ur_ros2_ws/config"
copy_tree "$SOFTWARE_DIR/app" "$STAGED_PACKAGE/Software/app"
for source in "$SOFTWARE_DIR"/*.py; do
  copy_file "$source" "$STAGED_PACKAGE/Software/${source:t}"
done
copy_file "$SOFTWARE_DIR/README.md" "$STAGED_PACKAGE/Software/README.md"
copy_file "$SOFTWARE_DIR/PORTABLE-MAC-README.md" "$STAGED_PACKAGE/README-MAC-PORTABLE.md"

APP="$STAGED_PACKAGE/Software/Mac/UR Print Workstation.app"
CONTENTS="$APP/Contents"
/bin/mkdir -p "$CONTENTS/MacOS" "$CONTENTS/Resources"
/usr/bin/swiftc -O -framework Cocoa -framework WebKit \
  "$SOFTWARE_DIR/native_app/URPrintWorkstation.swift" \
  "$SOFTWARE_DIR/native_app/main.swift" \
  -o "$CONTENTS/MacOS/URPrintWorkstation"
copy_file "$PLIST" "$CONTENTS/Info.plist"
copy_file "$SOFTWARE_DIR/native_app/AppIcon.icns" "$CONTENTS/Resources/AppIcon.icns"
/bin/chmod 755 "$CONTENTS/MacOS/URPrintWorkstation"

# The original developer installer writes an absolute project path into the
# plist. This package intentionally leaves that key absent: the existing Swift
# launcher resolves Software as the parent of Software/Mac.
if /usr/libexec/PlistBuddy -c 'Print :URPrintSoftwareDirectory' "$CONTENTS/Info.plist" >/dev/null 2>&1; then
  echo "Portable app contains an absolute Software directory override." >&2
  exit 1
fi

# Ad-hoc signing is sufficient for local verification, not public Gatekeeper
# distribution. A Developer ID signed/notarized build is a later release gate.
/usr/bin/xattr -cr "$APP"
/usr/bin/codesign --force --deep --sign - "$APP"
/usr/bin/codesign --verify --deep --strict "$APP"

# Check the important runtime imports from the isolated tree without creating
# Python caches or connecting to any devices.
(
  cd "$STAGED_PACKAGE/Software"
  PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -c 'import control_station; import print_execution'
)

for required in \
  "$STAGED_PACKAGE/Software/control_station.py" \
  "$STAGED_PACKAGE/Software/app/index.html" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/scripts/smooth_trajectory.py" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/scripts/execution_scope.py" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/config/ur3e_actual_calibration.yaml" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/config/printhead_geometry.json" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/config/workcell_geometry.json"; do
  if [[ ! -s "$required" ]]; then
    echo "Portable package is missing $required" >&2
    exit 1
  fi
done
for forbidden in \
  "$STAGED_PACKAGE/Software/config.json" \
  "$STAGED_PACKAGE/Software/gcode_calibration.json" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/build" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/install" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/runtime" \
  "$STAGED_PACKAGE/workspaces/ur_ros2_ws/log"; do
  if [[ -e "$forbidden" ]]; then
    echo "Portable package contains a private/generated path: $forbidden" >&2
    exit 1
  fi
done

# The staged directory is new, and the final name is immutable. A failed
# build leaves staging for inspection instead of touching an existing release.
/bin/mv "$STAGED_PACKAGE" "$PACKAGE_DIR"
/bin/rmdir "$STAGING"
/usr/bin/ditto -c -k --sequesterRsrc --keepParent "$PACKAGE_DIR" "$ARCHIVE"
(
  cd "$DIST_DIR"
  /usr/bin/shasum -a 256 "${ARCHIVE:t}" > "${CHECKSUM:t}"
)

echo "Portable Mac package: $PACKAGE_DIR"
echo "ZIP for colleagues: $ARCHIVE"
echo "SHA-256: $CHECKSUM"
