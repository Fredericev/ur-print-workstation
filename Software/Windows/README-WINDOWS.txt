UR PRINT WORKSTATION FOR WINDOWS
================================

Choose the package
------------------

- Most Intel/AMD Windows computers:
  UR-Print-Workstation-0.14.2-win-x64.zip

- Windows on ARM, including Apple-silicon Parallels:
  UR-Print-Workstation-0.14.2-win-arm64.zip

Open Settings > System > About > System type if you are unsure.

This 0.14.2 package uses the same interface, backend, geometry, and ROS
source snapshot as the current Mac workstation. It includes native extruder
control, paired timeline import, tabletop/Print Z calibration and height trials,
3D path review, MoveIt planning, move-to-start, full print execution,
stop/recovery, ROS lifecycle controls, and diagnostics.

Quick start
-----------

1. Copy the correct ZIP file to the Windows computer.
2. Extract the entire ZIP to a normal local folder.
3. Open the extracted folder.
4. Double-click "UR Print Workstation.exe".
5. Keep every extracted file and folder beside the EXE in place.

Do not run the EXE inside the ZIP preview. Extract all files first.
For a normal Intel/AMD computer, send the win-x64 ZIP and its .sha256 file.
No source checkout, build tools, Git Bash, or separate Python/.NET is needed.

No separate Python or .NET installation is required. Windows 11 normally
includes Microsoft Edge WebView2. If it is unavailable, the app automatically
opens the same local interface in the default browser.

User files
----------

The extracted package is treated as read-only. Configuration, calibration,
logs, browser data, and a versioned writable project copy are stored per user
in:

  %LOCALAPPDATA%\UR Print Workstation

Diagnostics
-----------

Choose File > Open Diagnostics Folder in the application. The Windows launcher
log is "windows-launcher.log". Operation logs are in the runtime subfolder.

Architecture
------------

- win-x64: normal Intel/AMD 64-bit Windows computers.
- win-arm64: Windows on ARM computers, including Apple-silicon Parallels VMs.

Use the package that matches Windows System type in Settings > System > About.

Online hardware requirements
----------------------------

Offline G-code parsing, placement on the operator-entered tabletop Z with an
independent Print Z, path conversion, and robot-path simulation
run entirely inside this package.

Print Execution is included in version 0.14.2. The native heated extruder must be configured and ready. Before the first real print,
restart the packaged ROS stack, confirm the installed table/TCP/tool values, rerun
the exact file's online MoveIt review, and perform the site's short, bounded
motion acceptance test. Timed UR/Klipper execution requires a stable 100%
pendant speed slider; the software separately caps commanded joint speed.

Online UR3e, ROS 2, MoveIt, and Klipper operations additionally require:

- a wired Ethernet route to the UR robot;
- Docker Desktop using Linux containers;
- the UR reverse-interface ports 50001 through 50004 allowed by Windows
  Defender Firewall; and
- a Windows hardware workflow that has passed a real workcell acceptance test.

The physical emergency stop is always authoritative. Complete a real-workcell
acceptance test before allowing production motion.

Checksums
---------

Each ZIP has a matching .sha256 file. It records the SHA-256 checksum of the
archive so the recipient can verify that the transfer did not corrupt it.

Version and feature verification
--------------------------------

The release source is shared with the Mac application; only the native window
and host operating-system commands differ. The release verifier checks every
shared source file against the current Mac workspace and both Windows ZIPs.
Windows real-hardware acceptance is a separate machine-local step; an offline
package check cannot verify robot networking or physical motion.

Offline RViz full-print replay
------------------------------

Double-click "Replay Offline Print.cmd" beside the EXE. Docker Desktop is
required for RViz. The helper uses the same per-user project as the application
and selects the latest complete, passed MoveIt execution package; it verifies
the frozen source/trajectory checksums before starting the display-only viewer.
Keep the matching original G-code in that project's "Test gcode" folder.
No developer execution history is bundled. The replay uses isolated ROS domain
84 and refuses to run alongside active hardware-control processes.
