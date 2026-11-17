# Windows native launcher

This folder contains the Windows shell for UR Print Workstation. It is a
self-contained .NET 10 WinForms application that:

- starts the packaged Python backend without requiring Python to be installed;
- embeds the existing UI with Microsoft Edge WebView2;
- falls back to the default browser if WebView2 is unavailable;
- stores configuration, calibration, logs, and WebView data in
  `%LOCALAPPDATA%\UR Print Workstation`;
- refuses to reuse port 8876 when it belongs to a different project copy; and
- stops only the backend process that it started.

The release pipeline is `../build-windows-package.command`. It cross-publishes
both `win-x64` and `win-arm64`, adds the matching official Python embeddable
runtime, creates clean project payloads, and writes portable ZIP archives plus
SHA-256 checksums under `../Windows`.

The portable packages do not install Python or .NET. Windows 11 normally
includes the WebView2 Evergreen Runtime; when it does not, the application
opens the same localhost UI in the default browser.

The native launcher and Offline workflows are Windows-portable. Real Online
UR/ROS/Klipper operations also require a supported Docker Desktop setup and
the included native Windows host-control runtime (Git Bash is not required). A Windows package must not be described
as hardware-ready until those operations have passed a wired, real-hardware
acceptance test on the target Windows architecture.
