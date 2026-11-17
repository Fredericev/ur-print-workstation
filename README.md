# UR3e Real Robot ROS Project

## Download and start the workstation

For a colleague who wants to **use** the software, download the ZIP matching
their computer from the repository's **Releases → Assets**, together with its
`.sha256` file. Keep the complete extracted folder together. The GitHub
**Code → Download ZIP** button downloads source code, not an installed app.
The [GitHub distribution guide](docs/GITHUB_DISTRIBUTION.md) covers both paths.

From a source download, Offline mode on a Mac needs Python 3.9 or newer:

```bash
cd Software
./start.command
```

For a native Mac window, build the local app after extracting the full source
tree with `Software/build-macos-app.command`. On Windows, build the native
package with `Software/build-windows-package.ps1`, or use the prebuilt Release
asset. Online mode additionally needs Docker/ROS, the wired robot network,
and a new local workcell acceptance test; downloading code alone cannot
verify real UR motion.

For installing the same full workstation on teammates' computers and updating
without losing calibration or stop evidence, see the
[team deployment and update guide](docs/TEAM_DEPLOYMENT.md).
The current internal package checksums and acceptance status are in the
[0.14.2 English release note](docs/RELEASE_0.14.2_ENGLISH.md).

For a future external path-planning algorithm, see the documented
[UR trajectory and timed extrusion import contract](docs/ur_extrusion_shared_timeline.md).
The paired import currently checks files offline only and cannot command the robot.

This workspace is now real-robot only. The old URSim / virtual PolyScope
environment has been removed.

## Start The Real UR3e Driver

```bash
cd /path/to/extracted-project
ROBOT_IP=<robot-ip> ./scripts/launch_real_robot_stable.sh
```

The stable path runs the UR driver at `125 Hz` by default, which is more
forgiving on macOS + Docker than the ROS package default of `500 Hz`. To test
the stock rate again, launch with `UR_UPDATE_RATE=500`.

Use Local Control on the teach pendant for this path. Start the ROS driver
first, then press Play on the program that contains the External Control node.

To keep the driver running in the container background:

```bash
ROBOT_IP=<robot-ip> ./scripts/launch_real_robot_detached.sh
```

To open RViz in the browser-accessible ROS desktop:

```bash
./scripts/start_rviz.sh
```

## Replay A Complete Print In RViz Without A Robot

The offline replay displays the configured table, centered G-code model, cached
MoveIt approach, and the complete print motion. It does not start the UR
driver, a controller, URScript, or Klipper:

```bash
cd /path/to/extracted-project
./scripts/start_offline_rviz_print_preview.sh --manifest latest --speed-factor 10
```

Then open `http://localhost:6082/vnc.html`, click **Connect**, and use the VNC
password `ubuntu` if prompted. RViz loops the animation automatically. The
display speed factor only changes playback. Full v3 packages replay the final
frozen quintic curve and commissioning timing, rather than the older raw
MoveIt CSV timing. Use `--speed-factor 1` for nominal time or `100` for a faster
review. Legacy/sample packages still display the complete cached MoveIt path.

`latest` means the newest checksum-bound execution package with a complete,
passed MoveIt review. The launcher prints the selected G-code name and review
ID before opening RViz, verifies every source/trajectory checksum, waits for
the graphical desktop, and refuses to run alongside hardware-control
processes. It also uses isolated localhost-only ROS domain `84`, so it does not
mix with the real-robot ROS graph.

This is a kinematic review, not a physics simulation or execution proof. The
cached approach begins at the joint posture recorded when the review was
created. Before real movement, reconnect the UR3e and plan the approach again
from fresh joint feedback.

The real-robot ROS desktop is:

```text
http://localhost:6082/
```

VNC password:

```text
ubuntu
```

The desktop port is bound to localhost only. Override the local VNC password by
setting `ROS_DESKTOP_PASSWORD` before starting the container. UR reverse ports
bind only to the configured `ROS_PC_IP` robot-facing interface.

## Useful Commands

```bash
# Start only the real-robot ROS desktop container
./scripts/start_ros.sh

# Enter the real-robot ROS container
./scripts/ros_shell.sh

# Stop the real-robot ROS container
./scripts/stop.sh

# Check robot network and Dashboard/RTDE ports
ROBOT_IP=192.168.0.104 ./scripts/check_ur_remote.sh

# Send one Dashboard command
ROBOT_IP=192.168.0.104 ./scripts/ur_dashboard.sh "is in remote control"
```

## Kept Files

- `docker-compose.real.yml`: Docker compose file for the real UR3e ROS desktop.
- `docker/ros2/Dockerfile`: ROS 2, MoveIt, and UR driver image.
- `scripts/launch_real_robot.sh`: Launch the real UR ROS driver.
- `scripts/launch_real_robot_stable.sh`: Conservative 125 Hz real-robot launch path for this Mac.
- `scripts/launch_real_robot_detached.sh`: Launch the real UR ROS driver in the container background.
- `scripts/start_rviz.sh`: Open RViz in the noVNC ROS desktop.
- `scripts/start_offline_rviz_print_preview.sh`: Replay a checksum-locked full print in isolated, display-only RViz.
- `scripts/setup_real_robot_network.sh`: Configure/check Mac-to-UR network.
- `scripts/check_ur_remote.sh`: Check ping, Dashboard, RTDE, and UR service ports.
- `REAL_ROBOT.md`: Full real UR3e connection notes.

## Removed

The URSim container, virtual PolyScope profile, simulation compose file, and
simulation launch scripts were removed so this project does not accidentally
start or store virtual robot resources. The offline RViz replay above does not
reintroduce URSim: it only renders a verified joint trajectory on the URDF
model and has no virtual or physical controller.
