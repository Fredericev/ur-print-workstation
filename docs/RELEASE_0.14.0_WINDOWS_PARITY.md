# 0.14.0 — complete Windows source parity with the current Mac workstation

Release date: 2026-11-13.

The installed Mac application directly loads this working directory's
`Software` through `URPrintSoftwareDirectory`. This release therefore uses the
latest working-directory code as its baseline, rather than the old 0.13.0 Mac
ZIP. The old Windows ZIPs were released in September and omitted subsequent
October fixes. Both platforms previously still displayed 0.13.0, so version
numbers alone could not establish parity. This release aligns the shared
backend and native publication versions at 0.14.0.

## Files to share with colleagues

- Standard Intel/AMD Windows computers: `Software/Windows/UR-Print-Workstation-0.14.0-win-x64.zip`.
- Windows ARM computers: `Software/Windows/UR-Print-Workstation-0.14.0-win-arm64.zip`.
- Each ZIP has a matching `.zip.sha256` file for transfer-integrity verification.

Extract the entire ZIP into a local directory, then double-click
`UR Print Workstation.exe`. Retain every file in the directory. Python and
.NET are included, so no compilation, Git Bash, or separate installation of
those runtimes is required. If WebView2 is unavailable, the launcher opens the
same interface in the default browser.

## Functionality and platform adaptation

The Mac package and both Windows packages contain the same complete interface,
Python backend, 3D assets, workcell and printhead geometry, UR calibration,
ROS sources, and executor. Features include:

- Original G-code and paired shared-timeline file import, parsing, and planning.
- Tabletop Z, independent Print Z, extrusion-ratio calibration, height trials, and 3D path review.
- MoveIt validation, move-to-start, complete print execution, and UR/extrusion coordination.
- Native `extruder` feeding/retraction and live temperature/configuration/override checks.
- Stop confirmation, recovery protection, repeated Start/Stop, ROS/MoveIt/bridge lifecycle controls, and diagnostics.

Windows also provides `Replay Offline Print.cmd`. Double-click it to replay a
complete print in RViz through Docker. It requires a complete execution package
that has passed MoveIt review on the local machine, with the matching original
G-code retained in the local project's `Test gcode` directory. The replay
verifies trajectory hashes first, uses isolated domain 84, and refuses to run
alongside hardware-control processes.

Windows host operations use the bundled Python runtime to call Docker
directly. Height and Print Z trials also use this path. The release synchronizes
container log rotation, process-identity checks, cleanup after startup
cancellation, read-only update-rate mount handling, and ROS build-cache/controller
verification. The native launcher checks backend version, working directory,
and demo mode to avoid connecting to another version or stopping a service it
does not own.

Each computer's window appearance follows its operating system. Personal
settings, job history, ROS binary builds, and old stop evidence are not copied
from the developer's machine. A colleague's settings are stored in
`%LOCALAPPDATA%\UR Print Workstation`. Project configuration such as workcell
geometry is delivered with the full source. The real network and current
tabletop height still need configuration for the target workcell.

## Verification and records

`python3 scripts/verify_team_release.py --report Software/Windows/PARITY-REPORT-0.14.0.json`
checks SHA-256 and archive integrity for the source ZIP, portable Mac package,
and both Windows packages. It checks every shared runtime file for omissions,
changes, or leaked private/runtime state. The machine-readable report contains
the ZIP hashes and complete shared-file fingerprint.

Actual validation results for this release:

- Complete workstation Python regression suite: 646 tests passed.
- ROS/Klipper bridge offline regression suite: 221 tests passed.
- JavaScript UI regression suite: 27 tests passed; the complete cold-start browser workflow passed with no console errors.
- Windows launcher service-identity/shutdown-ownership behavior tests passed; both x64 and ARM64 builds passed.
- Each Windows package had 1,079 file hashes checked, complete internal manifests, correct PE architectures, and identical project payloads.
- The local Mac launcher was aligned to 0.14.0 (build 17), signature verification passed, and the previous launcher was retained as a backup.

GUI validation was attempted in the existing Windows 11 virtual machine, but
the current Parallels edition does not support guest command-line execution,
and automated input could not operate the guest interface. Actual Windows
GUI startup and real-device acceptance are therefore not claimed as passed.
The virtual machine was restored to its previous suspended state. No real
robot motion, heating, or extrusion was executed.

All Online functionality is retained. The target Windows computer also needs
Docker Desktop Linux containers, the robot's wired network, and reverse ports
50001–50004. The first Online ROS startup prepares the runtime from the bundled
source. Source parity does not establish real-hardware validation on the target
Windows machine. Complete that workcell's acceptance procedure in the
[team deployment guide](TEAM_DEPLOYMENT.md) before the first motion.
