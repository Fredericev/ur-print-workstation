# UR Print Workstation: colleague distribution and update research

Date: 2026-09-25. Scope: the existing macOS Swift/WebKit launcher, Python
service, Windows WinForms/WebView2 launcher, and Docker/ROS 2 workspace. This
is research into release options; **no package, updater, or device was changed**.

## Findings

Establish a reproducible, recoverable **internal pilot release** before making
it available to colleagues. Apply application updates only after UR, Klipper,
and local jobs have stopped. Each real workstation still requires its own
configuration and motion acceptance test when first connecting with a new
version. This is an engineering recommendation for this project's physical
motion risks, rather than an Apple/Microsoft installation restriction. Begin
with version numbers, checksums, release notes, and manual installation. Add
update notifications after installation and rollback have been validated. Do
not hot-replace running Python files or ROS containers during printing.

## Repository state and remaining gaps

| Component | State at the time of this review | Required before colleague distribution |
|---|---|---|
| macOS | `Software/build-macos-app.command` compiles Swift, writes the **absolute path** of the development machine's `Software` directory into `URPrintSoftwareDirectory`, finds local Python ≥3.9, applies a temporary ad-hoc signature with `codesign --sign -`, and places the app in the user's `~/Applications`. | Copying only the `.app` does not carry Python or the ROS project. Define a fixed read-only release location or include the required payload in the package; use Developer ID signing, Hardened Runtime, and Apple notarization, then test on another Mac. The current ad-hoc signature is not a formal external-distribution signature. [Apple notarization requirements](https://developer.apple.com/documentation/security/notarizing-macos-software-before-distribution) |
| Windows | `Software/build-windows-package.ps1` creates x64/arm64 ZIPs containing self-contained .NET, embedded Python, and project files, adds SHA-256 checksums, and writes `PAYLOAD_VERSION.txt`. | ZIPs can support internal trials, but there is no installer or automatic updater. Provide explicit stop, user-configuration backup, replacement, and startup-validation steps. `Software/Windows/README-WINDOWS.txt` still says `0.12.1`, while the build script and Windows project say `0.13.0`; existing ZIPs are also `0.12.1`. They cannot be treated as one approved release. [Microsoft self-contained distribution and update options](https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/publish-first-app) |
| Docker/ROS | The base image in `docker/ros2/Dockerfile` uses a mutable tag. `docker-compose.real.yml` bind-mounts the ROS workspace from the project directory and includes host network ports and `seccomp=unconfined`. | Record the base-image/build-artifact digest, ROS package versions, workspace source version, and safety-relevant configuration in the release manifest. Stop and validate before updating the container. Docker documents that tags are mutable, whereas digests can pin build inputs; deliberate periodic updates are still required to obtain security fixes. [Docker build best practices](https://docs.docker.com/build/building/best-practices/) |

## Formal macOS distribution requirements

1. Choose an App + DMG or a signed `.pkg`, placing the required application payload somewhere **independent of development-machine paths**. If Python or other executables are bundled, sign internal executables first and the outer app last. Apple does not recommend `codesign --deep` as the routine signing workflow. [Apple signing-depth guidance](https://developer.apple.com/library/archive/technotes/tn2206/)
2. Use a `Developer ID Application` certificate for external distribution. Installer packages also need `Developer ID Installer` where appropriate for their format. Notarization requires a valid signature, Hardened Runtime, and a secure timestamp; the existing `--sign -` is insufficient. Submit with `notarytool`, inspect notarization logs, staple the ticket with `stapler`, and test the actual distribution file on a clean, separate Mac. Apple no longer accepts the old `altool` notarization upload route. [Apple notarization requirements](https://developer.apple.com/documentation/security/notarizing-macos-software-before-distribution); [custom notarization workflow](https://developer.apple.com/documentation/security/customizing-the-notarization-workflow); [packaging and cross-machine testing](https://developer.apple.com/documentation/xcode/packaging-mac-software-for-distribution)
3. Maintain `CFBundleShortVersionString`, the user-facing `Major.Minor.Patch`, and increment `CFBundleVersion` for every release. Do not change only the displayed version without rebuilding the Python/ROS payload. [Apple version field](https://developer.apple.com/documentation/bundleresources/information-property-list/cfbundleshortversionstring); [Apple build number](https://developer.apple.com/documentation/bundleresources/information-property-list/cfbundleversion)
4. If Sparkle is adopted for Mac updates later, separately design signed update archives and an appcast, and protect the private key. Sparkle is a **third-party open-source framework, not an Apple-native automatic update service**. Its official documentation requires cryptographic signing of update archives and uses increasing `CFBundleVersion` values to identify newer releases. [Sparkle documentation](https://sparkle-project.org/documentation/); [publishing signed updates](https://sparkle-project.org/documentation/publishing/)

## Formal Windows distribution and update options

The current ZIP is an unpackaged, self-contained WinForms publication. A
colleague does not need to install .NET separately, but must retain the full
directory. **The ZIP itself has no automatic update mechanism**. Microsoft
lists these development paths:

- **Small internal trial:** continue using versioned ZIPs, SHA-256, and manual update instructions. Code signing can reduce Smart App Control/SmartScreen blocking of unknown unsigned binaries, but does not replace testing. [Microsoft Windows distribution choices](https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/choose-distribution-path); [Smart App Control](https://learn.microsoft.com/en-us/windows/apps/develop/smart-app-control/overview)
- **Managed long-term release:** package WinForms as MSIX, then distribute through Microsoft Store, company MDM/Intune, or signed MSIX + `.appinstaller`. The last option supports startup or periodic update checks. Non-Store MSIX must be signed with a certificate trusted by the target Windows installation; self-signing is appropriate only for tests whose target machines have been configured to trust that certificate. [Microsoft MSIX signing](https://learn.microsoft.com/en-us/windows/msix/package/sign-msix-package-guide); [App Installer automatic updates](https://learn.microsoft.com/en-us/windows/msix/app-installer/auto-update-and-repair--overview)
- **Traditional installer for complex external dependencies:** if Docker, network/firewall setup, and robot workcell configuration do not fit naturally into MSIX, consider a signed MSI/EXE and enterprise deployment tools. Updates need their own design on that route; `.appinstaller` capabilities cannot be assumed for the current ZIP. [Microsoft distribution-path comparison](https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/choose-distribution-path)

WinForms currently uses WebView2, so check for the WebView2 Runtime on target
machines. Microsoft states that Windows 11 usually includes the Evergreen
Runtime, but its presence cannot be assumed on every computer. An online
bootstrapper or complete offline installer can be provided.
[Microsoft WebView2 distribution](https://learn.microsoft.com/en-us/microsoft-edge/webview2/concepts/distribution)

## Shared dependencies and release manifest

Online ROS/MoveIt/UR use needs a supported Docker/Compose Linux-container
environment. The current Mac installation script uses Homebrew + Colima,
rather than Docker Desktop. The Windows route uses Docker Desktop; a WSL 2
backend also needs to meet the Windows, WSL, virtualization, and memory
requirements. The official Docker Desktop page labels the Windows Arm version
as Early Access at the time of this review. Validate it separately; the
existence of a `win-arm64` application ZIP does not establish supported Online
functionality. If the Mac route later changes to Docker Desktop, separately
check its officially supported macOS range and organizational commercial
licensing. The current app's `LSMinimumSystemVersion=12.0` is not an **Online**
compatibility guarantee. [Docker Mac installation/licensing](https://docs.docker.com/desktop/setup/install/mac-install/);
[Docker Windows installation/licensing/WSL requirements](https://docs.docker.com/desktop/setup/install/windows-install/)

Each release should have a machine-readable `release-manifest` and
colleague-facing release notes covering at least:

- One product version, Mac build number, Windows package version, Python backend/API version, ROS workspace commit or archive hash, Docker image digest, and SHA-256 for every ZIP/DMG/PKG/MSIX.
- Validated platform architectures, Docker/Desktop and ROS compatibility range, configuration schema version, and UR/Klipper device/firmware requirements.
- Migration and rollback steps that distinguish read-only application files from the user's tabletop/TCP/network/calibration settings, G-code, logs, and history, without deleting user data during replacement.
- Prerelease test records: offline unit/integration tests, clean-machine installation, upgrade/rollback, offline operation, failure recovery, and on-site acceptance for each physical workstation.

**Proposed update channels (design recommendation):** use `pilot` for developers
and one test workstation, and `stable` for colleagues. Do not let a printing
process apply updates automatically. Downloads can be offered automatically,
but installation, ROS image/controller changes, and first real-hardware use
should explicitly pass stopped-state checks. If MSIX App Installer is used
later, configure its actual update policy. If Sparkle is used, implement an
application-level policy to defer updates while jobs are active. On update
failure, retain the complete previously accepted release and its matching
configuration backup. Rolling back only the frontend while retaining a new
executor/ROS stack is insufficient.

## Recommended sequence

1. Freeze the current version and create an accurate installation matrix. Fix the mismatch between Windows documentation and package versions.
2. Validate clean-machine installation, Offline mode, and preflight before real-device connection, so colleagues can run without development-machine directories. Retain the first-device acceptance gate.
3. Establish a release pipeline: passing tests → pinned dependencies/images → builds for both platforms → signing/notarization → hashes/release manifest → package validation on an independent machine → admission to `pilot`.
4. Begin with manual, stopped-only update and rollback. After this is stable, evaluate Mac Sparkle and Windows MSIX/App Installer or enterprise MDM. Test the installation experience together with the robot's safety state.
