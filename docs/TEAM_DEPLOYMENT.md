# Team deployment and updates

This project uses one physical UR3e. Every teammate receives the **same full-featured UR Print Workstation release**: both Offline and Online modes are present. There is no reduced colleague edition. The team coordinates which computer is connected to the robot; the software does not implement a network-wide ownership lease.

## What is and is not portable

- Share a **versioned release package**, not the development checkout's `Software/Mac/UR Print Workstation.app` symlink or an old Windows ZIP. The development Mac app contains a path from the developer's computer.
- A release must carry one matching backend, UI, native launcher, ROS workspace, workcell geometry, printhead model, and UR calibration snapshot. The source snapshot is common; each computer keeps its own network settings, operator data, job records, and logs. The tabletop and Print Z settings are stored locally, but each operator must enter the **current actual height of the shared physical table** and the intended Print Z before planning; they are not independent machine calibrations.
- Offline review does not prove a route is safe to execute. On every computer that will control the UR, first validate its local network route, Docker/ROS environment, External Control host address, calibration, MoveIt scene, and stop behavior. A package checksum cannot substitute for this acceptance test.
- macOS and Windows binaries differ, but their application/backend version and functional scope must match. `VERSION` in `Software/control_station.py` is the release source of truth; native version fields and package names must agree.
- Export the clean GitHub source ZIP, build the three internal platform ZIPs, then run `python3 scripts/verify_team_release.py`. It checks archive integrity, private-file exclusions, and exact common backend/ROS/geometry content against the source ZIP and across platforms. Release only if it passes.

From the project root, run `python3 scripts/export_github_source.py`, build an Apple-silicon Mac package with `Software/build-macos-portable-package.command` and both Windows architectures with `Software/build-windows-package.command`, then run the verifier above. See [GITHUB_DISTRIBUTION.md](GITHUB_DISTRIBUTION.md) for the source-repository and Release-asset distinction. Change the backend and native app versions together before a future release; do not silently replace a ZIP that teammates already received under the same version.

## Installing on another computer

1. Obtain the versioned package and matching SHA-256 file from the release owner. Verify the archive hash before extracting. Keep the **entire extracted folder** together; do not copy out only the `.app` or `.exe`.
2. Use the matching CPU architecture. On macOS, provide Python 3.9+; Online also needs the project's supported container runtime. On Windows, the package includes Python and .NET, but Online additionally needs the supported Docker Desktop/WSL setup. See each package's own README for exact requirements.
3. Start in Offline mode and check G-code import and 3D review using a known sample. Enter that computer's UR IP, local robot-facing IP and Klipper URL in Settings. Do not reuse another computer's local IP.
4. Before this computer's first Online use, verify the pendant's External Control program targets **this computer's** robot-facing IP, its reverse ports 50001–50004 are reachable, the UR/ROS and Klipper endpoints answer read-only checks, and the correct workcell/printhead/UR calibration is installed. Follow the existing connection and low-speed commissioning runbooks; never use an old MoveIt review or execution package after changing computer or geometry.

## Updating without losing workcell state

Updates are **manual and stopped-only**. Do not swap Python/ROS files during a print, a move-to-start operation, an active recovery, or while a driver is attached. Confirm UR and Klipper are stopped, close the workstation, and stop the old computer's ROS session before replacing a release.

1. Record the current package version and keep its entire extracted directory as a rollback copy.
2. Back up the operator's writable data directory and the installed workcell configuration before updating. On macOS, the data directory is `~/Library/Application Support/UR Print Workstation`; on Windows, `%LOCALAPPDATA%\UR Print Workstation`. Keep `workspaces/ur_ros2_ws/config` (including table height, printhead and UR calibration), `workspaces/ur_ros2_ws/runtime` (including stop evidence), and job logs. Do **not** treat a package's default table Z as the current physical table height.
3. Extract the new release to a **new** folder. Transfer only the site's reviewed configuration and necessary runtime/stop evidence with a documented migration; never overwrite the new program files with old ones. If migration or schema validation fails, do not go Online.
4. Run the release tests, then the local Offline smoke test. On the control computer, run read-only UR/Klipper checks and a new MoveIt review against the exact file and current height. Only then perform a deliberate low-speed hardware acceptance test. Do not auto-start a print as part of updating.
5. If the new release fails, stop both devices, close the new app, restore the previous **complete** release and its matching configuration/runtime backup, and investigate. Do not mix a new backend with an old ROS executor or vice versa.

The team may later automate this stopped-only migration and rollback. Until a clean-machine install, upgrade, rollback, and real-workcell acceptance test have been recorded, package output is an **internal pilot**, not a certified production installer. Apple Developer ID signing/notarization and Windows code signing are separate distribution tasks; the current local build credentials do not provide them.

## Folder cleanup policy

Keep ROS `build`, `install`, `runtime`, and `log` directories; print-execution evidence and diagnostics can be required for safe recovery. Keep CAD, G-code, calibration, workcell and source files. Only regenerateable Python/pytest caches and Finder `.DS_Store` files are routine cleanup candidates. Retain old release archives until the replacement package has been verified on its target platform and the rollback window is closed.

The Windows packaging cache (`Software/runtime/windows-build` and `Software/windows_app/bin`/`obj`) is regenerable. Removing it does not affect the installed workstation or the completed ZIPs; the next Windows package build will need to recreate/download its toolchain.
