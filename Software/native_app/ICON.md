# Application icon

The Mac icon reuses `../windows_app/AppIcon.ico`, the project's existing
256 × 256 artwork. No new design or AI-generated artwork was introduced.

Run `./Software/native_app/build-icon.command` from the project root to rebuild
`AppIcon.icns` with native-resolution and smaller frames. The Mac build script
requires this file and copies it into `Contents/Resources` before signing.
`AppIcon-preview.png` is a PNG preview of the same asset.

Verification: `python3 -m unittest Software.tests.test_native_icon`, shell syntax
checks, plist validation, installed/source byte equality and macOS code-signature
verification. The 2026-09-06 repair installed the resource without restarting the
running workstation or changing ROS/control code. A running Dock entry may keep
its cached icon until a subsequent normal launch.
