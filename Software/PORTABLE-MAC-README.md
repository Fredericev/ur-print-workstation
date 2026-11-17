# UR Print Workstation for Mac — portable internal package

This ZIP contains the full Offline and Online workstation, ROS source, scripts,
the shared UR3e calibration, and the current printhead/workcell model. It does
not contain the developer's G-code files, application settings, browser data,
operation logs, ROS build products, or print history.

## Install

1. Use the ZIP matching your Mac architecture. The current internal build is
   for Apple Silicon (`mac-arm64`). Verify the ZIP against its `.sha256` file.
2. Extract the complete ZIP to a local, writable folder. Keep the entire
   `UR-Print-Workstation-<version>-mac-<architecture>` folder together. The
   `.app` is inside `Software/Mac`; moving it alone breaks its project path.
3. Install Python 3.9 or newer. The launcher checks the usual system,
   Homebrew, and Anaconda locations, or `UR_PRINT_PYTHON` if set. Python is not
   bundled in this internal package.
4. Open `Software/Mac/UR Print Workstation.app`. Initial application settings
   are created in `~/Library/Application Support/UR Print Workstation`.

Offline import and 3D review do not require a robot connection. Online mode
requires the workcell's wired network, Docker and Colima, ROS setup, and the
usual UR/Klipper prerequisites. The ROS workspace builds on each new Mac; the
downloaded ZIP does not contain a developer-machine build or installed ROS
overlay. The package has the same controls as the development workstation,
including Online mode; sharing a robot still requires one operator to control
the physical workcell at a time.

## Update and rollback

Keep the previous extracted folder until the replacement has passed offline
checks and, when appropriate, the workcell's controlled connection checks.
Close the app and stop the old ROS/container stack before switching folders.
Back up `~/Library/Application Support/UR Print Workstation` and the old
folder's `workspaces/ur_ros2_ws/config`, `workspaces/ur_ros2_ws/runtime`, and
any personal files in `Test gcode`. Extract the new version into a separate
folder. Compare the old and new machine profiles: carry forward the actually
measured tabletop height and any verified UR calibration/printhead changes.
Preserve prior runtime incident records. Rebuild ROS from the new source before
Online use and rerun path review for the selected job. If the new version fails,
return to the previous complete folder and its saved state.

There is no automatic updater or cross-computer ownership lock in this pilot
package. It is ad-hoc signed so it can be checked locally; it is not yet a
Developer ID signed and notarized public macOS release. Treat a downloaded
copy blocked by macOS Gatekeeper as a release-signing task for the maintainer.
