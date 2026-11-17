UR PRINT WORKSTATION FOR MAC
============================

Quick start
-----------

For a prebuilt Release package, double-click "UR Print Workstation.app".
For a GitHub source download, build the local app first with
"Software/build-macos-app.command", or run "Software/start.command" with
Python 3.9 or newer. The source tree does not include a prebuilt .app.

Keep this Mac folder inside the main Software folder. The application locates
the shared backend and project files one folder above it.

Offline and Online
------------------

Offline G-code parsing, placement using the operator-entered actual tabletop Z
and independent Print Z, path conversion, and robot-path
simulation do not contact the UR robot, ROS, MoveIt, Docker, or Klipper.

Online operations use the existing guarded workcell workflow. A real motion
test still requires the wired UR network, the configured container runtime,
External Control, the physical emergency stop, and operator confirmation.

Print Execution is included in version 0.14.2. Before the first real print,
restart the ROS stack, confirm the installed table/TCP/tool values, and rerun
the exact file's online MoveIt review. A short, bounded motion acceptance test
is required before printing. Timed synchronized execution requires a stable
100% pendant speed slider; commanded joint speed is capped in software.

Diagnostics
-----------

The native app log and user settings are in
"~/Library/Application Support/UR Print Workstation". ROS execution evidence
and its diagnostic logs remain under "workspaces/ur_ros2_ws/runtime" and "log".
