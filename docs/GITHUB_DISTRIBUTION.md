# GitHub distribution: source versus runnable packages

This project controls one physical UR3e. All computers get the same complete
Offline and Online software. Each computer still needs its own network setup
and a deliberate workcell acceptance test; a download does not certify motion.

## Prepare the repository source

Do **not** initialize Git or upload the current development folder directly.
It contains two patched vendor repositories with their own `.git` directories,
an absolute Mac app symlink, workcell logs, build output, and local settings.
A direct `git add .` can record the vendor trees as unusable gitlinks instead
of their patched source. The root `.gitignore` is only a second line of defense.

From this project root, with the software stopped:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s Software/tests -p 'test_*.py'
python3 scripts/export_github_source.py
```

The exporter creates a new, immutable folder and ZIP in
`Software/dist/source/`, plus a `.zip.sha256` file and a source manifest. It
copies the full UI, backend, native launcher sources, ROS source (including the
patched UR libraries), scripts, geometry/calibration defaults, and tests. It
excludes CAD, personal G-code, settings, logs, build/install/runtime state,
nested Git metadata, and the developer-machine app symlink. Inspect the
manifest and the new folder before publishing. Use **that exported folder** as
the root of a new GitHub repository; do not upload the parent development tree.
The ZIP is a source archive, not a prebuilt app.

This repository currently has no project-level license. The vendored code has
its own licenses, and the bundled External Control URCap needs a redistribution
decision. Use a **private** repository for teammates until the owner has
reviewed the project's license and what workcell calibration may be public.

## Publish apps people can run

Build new versioned packages from the same reviewed source snapshot using
`Software/build-macos-portable-package.command` on Apple Silicon and
`Software/build-windows-package.command` on Mac, or
`Software/build-windows-package.ps1` on Windows. Run
`python3 scripts/verify_team_release.py` after the three package ZIPs exist.
The ZIPs and their matching `.sha256` files belong under one GitHub **Release
→ Assets** entry. Do not substitute GitHub's automatically generated **Source
code (zip)**: that archive contains source, not a compiled `.app` or `.exe`.
Do not replace assets silently under an already-shared version; increase the
backend and native app versions together for each future release.

Use the package matching the recipient's architecture. The current Mac build
targets Apple Silicon; an Intel Mac needs its own build on an Intel Mac. Mac
users need Python 3.9+; the Windows release includes its Python and .NET
runtime. The Mac app is not Developer ID notarized and the Windows app is not
code-signed, so these are internal pilot packages, not frictionless public
installers. Do not work around platform security policy on a teammate's
computer without the team's approval; arrange signing for broader rollout.

## If someone downloads GitHub source instead

Keep the entire source folder together. On Mac, install Python 3.9+ and run
`Software/start.command` from Finder/Terminal for Offline mode. To create a
native Mac window from source, install Xcode Command Line Tools and run
`Software/build-macos-app.command`. On Windows, build the native package with
the included PowerShell script or obtain the Release asset. A source download
does not carry a developer-machine ROS build, Python runtime, or live job data.

For Online use, follow [TEAM_DEPLOYMENT.md](TEAM_DEPLOYMENT.md). On a new Mac,
the first **Start ROS** builds the local container image and ROS overlay; on
Windows, Docker Desktop/WSL builds its own ROS overlay. Before any move or
print, configure the wired route, current real tabletop and Print Z, the
robot-specific UR calibration and tool model, and the pendant External Control
host address for **that computer**. Validate a fresh route and stop behavior
locally. Never reuse another computer's review or execution package.

## Updates

Publish a new tag and new immutable release assets. Verify each checksum on
the receiving computer. Stop UR and Klipper, close the old workstation, keep
the entire old folder and data backup, install the new folder side by side,
then repeat local Offline and hardware acceptance. No auto-update or
cross-computer control lock is implemented; the team coordinates the shared
robot. See the stopped-only rollback procedure in the deployment guide.
