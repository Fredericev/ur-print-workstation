# UR Print Workstation 0.14.2 — English release

Release date: 2026-11-16.

This release publishes the complete English workstation on both Mac and Windows.
It also corrects clean-source ZIP permission metadata on Windows and makes the
regression fixtures independent of developer-local calibration and Unix-only
process behavior. These developer/distribution fixes retain application behavior.
It retains all 913 interface translation keys and the full application feature
set. Existing profiles or browser storage that selected Chinese now use English.
Interface labels, operation messages, number/date formatting, generated print
animations, installation prompts, and the published project documentation are
English. An English-copy check also runs in source CI.

## Downloads

Choose a prebuilt ZIP under the GitHub Release's **Assets** section:

| Computer | Download |
| --- | --- |
| Windows Intel/AMD 64-bit | `UR-Print-Workstation-0.14.2-win-x64.zip` |
| Windows ARM64 | `UR-Print-Workstation-0.14.2-win-arm64.zip` |
| Apple-silicon Mac | `UR-Print-Workstation-0.14.2-mac-arm64.zip` |

Each ZIP has a matching `.zip.sha256` file. Download both when checking transfer
integrity. The explicitly named `UR-Print-Workstation-0.14.2-source.zip` and
GitHub's automatic **Source code** downloads are for development; they are not
the Windows application.

On Windows, extract the entire ZIP to a local folder, then double-click
`UR Print Workstation.exe`. Keep all files together. Python and .NET are included;
no source checkout or compilation is required. If WebView2 is unavailable, the
launcher opens the identical local application in the default browser.

## Feature parity

The Mac and both Windows builds share the same complete interface, backend,
3D assets, geometry/configuration definitions, patched ROS sources and executors.
They include G-code and paired timeline import, tabletop/Print Z/TCP setup,
3D review, MoveIt validation, move-to-start, full print execution, native Klipper
extrusion/retraction, stop confirmation and recovery, runtime controls and logs.
`Replay Offline Print.cmd` provides Windows access to the same verified,
display-only RViz replay using Docker and isolated ROS domain 84.

Offline G-code workflows run without Docker. Online robot operations and offline
RViz replay need Docker Desktop with Linux containers on Windows. Online use also
needs the target computer's wired robot route, reverse ports 50001–50004, local
workcell configuration, and acceptance checks before physical motion.

The release verifier compares all shared runtime files in the clean source ZIP,
Mac ZIP and both Windows ZIPs. `PARITY-REPORT-0.14.2.json` records exact shared-file
fingerprints and package checksums. Native Windows compilation, offline tests,
and archive checks do not constitute a Windows real-workcell acceptance test.
The available Parallels setup could not complete automated guest GUI interaction;
Windows native GUI launch and physical robot behavior remain unverified here.
No robot motion, heating, or extrusion was commanded while preparing this release.

## GitHub publishing

Publish the clean exported source tree, never the active development directory.
The exporter excludes developer settings, personal G-code, execution history,
logs, generated ROS builds, CAD, nested Git metadata and the local Mac app symlink.
Attach runnable ZIPs and their checksum files to the `v0.14.2` GitHub Release so
colleagues can download ready-made packages. See the repository's
[distribution guide](GITHUB_DISTRIBUTION.md) and
[team deployment guide](TEAM_DEPLOYMENT.md) for installation and updates.
