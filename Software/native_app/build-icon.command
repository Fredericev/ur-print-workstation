#!/bin/zsh
# Repackage the existing Windows artwork; no new artwork, app launch or ROS call.
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
SOURCE="$SCRIPT_DIR/../windows_app/AppIcon.ico"
DESTINATION="$SCRIPT_DIR/AppIcon.icns"
ICON_WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ur-print-icon.XXXXXX")"
ICONSET="$ICON_WORK_DIR/AppIcon.iconset"
mkdir -p "$ICONSET"

# The incumbent source is 256 px. Keep that native resolution instead of
# labelling an enlarged bitmap as new high-resolution artwork.
sips -s format png "$SOURCE" --out "$ICON_WORK_DIR/source.png" >/dev/null
for specification in '16 icon_16x16' '32 icon_16x16@2x' '32 icon_32x32' \
                     '64 icon_32x32@2x' '128 icon_128x128' \
                     '256 icon_128x128@2x' '256 icon_256x256'; do
    SIZE="${specification%% *}"
    NAME="${specification#* }"
    sips -z "$SIZE" "$SIZE" "$ICON_WORK_DIR/source.png" --out "$ICONSET/$NAME.png" >/dev/null
done
iconutil --convert icns "$ICONSET" --output "$DESTINATION"
echo "Created: $DESTINATION"
echo "Conversion intermediates: $ICON_WORK_DIR"
