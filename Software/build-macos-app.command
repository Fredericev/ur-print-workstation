#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
USER_APPLICATIONS="${UR_PRINT_APP_DIRECTORY:-${HOME:?}/Applications}"
if [[ "$USER_APPLICATIONS" != /* ]]; then
  echo "UR_PRINT_APP_DIRECTORY must be an absolute Applications directory." >&2
  exit 64
fi
USER_APPLICATIONS="${USER_APPLICATIONS:A}"
if [[ "$USER_APPLICATIONS" == / || "$USER_APPLICATIONS" == "${HOME:A}" || "$USER_APPLICATIONS" == "$SCRIPT_DIR" || "$USER_APPLICATIONS" == "${SCRIPT_DIR:h}" ]]; then
  echo "Refusing a home, filesystem, or project root as the app installation directory." >&2
  exit 64
fi
INSTALL_APP="$USER_APPLICATIONS/UR Print Workstation.app"
PROJECT_APP_LINK="$SCRIPT_DIR/Mac/UR Print Workstation.app"
SOURCE="$SCRIPT_DIR/native_app/URPrintWorkstation.swift"
MAIN="$SCRIPT_DIR/native_app/main.swift"
ICON="$SCRIPT_DIR/native_app/AppIcon.icns"

if [[ ! -s "$ICON" ]]; then
  echo "Missing app icon. Run Software/native_app/build-icon.command first." >&2
  exit 1
fi
for source_file in "$SOURCE" "$MAIN" "$SCRIPT_DIR/native_app/Info.plist"; do
  if [[ ! -s "$source_file" ]]; then
    echo "Required app source is missing: $source_file" >&2
    exit 1
  fi
done
if ! /usr/bin/xcode-select --print-path >/dev/null 2>&1 || ! /usr/bin/swiftc --version >/dev/null 2>&1; then
  echo "Install Xcode Command Line Tools before building the Mac app (xcode-select --install)." >&2
  exit 1
fi
if [[ -L "$INSTALL_APP" || ( -e "$INSTALL_APP" && ! -d "$INSTALL_APP" ) ]]; then
  echo "Refusing to replace an app installation that is a symlink or non-directory: $INSTALL_APP" >&2
  exit 64
fi
if [[ -e "$PROJECT_APP_LINK" && ! -L "$PROJECT_APP_LINK" ]]; then
  echo "The project app path is not a symlink. Keep it unchanged and install from a clean source checkout." >&2
  exit 64
fi

require_app_stopped() {
  # Refuse replacement if any native workstation is open, including one
  # launched through the project symlink. Do not terminate it from the builder.
  if /bin/ps -axo comm= | /usr/bin/grep -E '/URPrintWorkstation$|^URPrintWorkstation$' >/dev/null; then
    echo "Close UR Print Workstation before installing. The running app was left unchanged." >&2
    return 1
  fi
}
require_app_stopped

mkdir -p "$SCRIPT_DIR/Mac" "$USER_APPLICATIONS"
# Compile/sign/verify a new bundle first. Failed builds cannot remove the
# currently installed version. Staging is intentionally retained on failure.
STAGING="$(/usr/bin/mktemp -d "$USER_APPLICATIONS/.ur-print-install.XXXXXXXX")"
APP="$STAGING/UR Print Workstation.app"
CONTENTS="$APP/Contents"
mkdir -p "$CONTENTS/MacOS" "$CONTENTS/Resources"

/usr/bin/swiftc \
  -O \
  -framework Cocoa \
  -framework WebKit \
  "$SOURCE" \
  "$MAIN" \
  -o "$CONTENTS/MacOS/URPrintWorkstation"

/bin/cp "$SCRIPT_DIR/native_app/Info.plist" "$CONTENTS/Info.plist"
/bin/cp "$ICON" "$CONTENTS/Resources/AppIcon.icns"
/usr/bin/plutil -replace URPrintSoftwareDirectory -string "$SCRIPT_DIR" "$CONTENTS/Info.plist"
/bin/chmod 755 "$CONTENTS/MacOS/URPrintWorkstation"
/usr/bin/xattr -cr "$APP"
/usr/bin/codesign --force --deep --sign - "$APP"
/usr/bin/touch "$APP"
/usr/bin/xattr -cr "$APP"
/usr/bin/codesign --verify --deep --strict "$APP"

# Recheck after compiling in case the operator opened an app during the build.
require_app_stopped
/bin/ln -s "$INSTALL_APP" "$STAGING/new-project-link"
BACKUP_APP=""
if [[ -e "$INSTALL_APP" ]]; then
  BACKUP_APP="$USER_APPLICATIONS/UR Print Workstation.backup-${STAGING:t}.app"
  /bin/mv "$INSTALL_APP" "$BACKUP_APP"
fi
if ! /bin/mv "$APP" "$INSTALL_APP"; then
  if [[ -n "$BACKUP_APP" && ! -e "$INSTALL_APP" ]]; then
    /bin/mv "$BACKUP_APP" "$INSTALL_APP"
  fi
  echo "Installation failed; the previous app was restored or kept at $BACKUP_APP." >&2
  exit 1
fi
# macOS mv -h replaces a known symlink without following its directory target.
/bin/mv -h "$STAGING/new-project-link" "$PROJECT_APP_LINK"
/bin/rmdir "$STAGING"

echo "Built and installed: $INSTALL_APP"
echo "Project link: $PROJECT_APP_LINK"
if [[ -n "$BACKUP_APP" ]]; then
  echo "Previous app retained for rollback: $BACKUP_APP"
fi
