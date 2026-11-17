# UR Print Workstation 0.13.0 — team pilot

Rebuilt 2026-09-27 from the shared project snapshot. All packages contain the
same Online and Offline functionality; there is no colleague-only edition or
cross-computer control lock.

These ZIPs supersede the earlier local 0.13.0 pilot builds made before the
GitHub first-install fixes. The older local ZIPs were retained in a recoverable
backup and must not be attached to the GitHub Release. No GitHub release of
0.13.0 has been published by this procedure.

| Platform | Package | SHA-256 |
| --- | --- | --- |
| Apple-silicon Mac | `Software/dist/mac/UR-Print-Workstation-0.13.0-mac-arm64.zip` | `b86146714439686e5dbb191e9f95f12c918fb2b9b756bb0215616e41171531a1` |
| Windows x64 | `Software/Windows/UR-Print-Workstation-0.13.0-win-x64.zip` | `1ad37cdf3ffcb1bfac14d4c0be94e28417086ed1ec0ccd1d4fe078ee3d2e4568` |
| Windows ARM64 | `Software/Windows/UR-Print-Workstation-0.13.0-win-arm64.zip` | `14c756b1599590cd4f8077a6a12c3ddd0d125c16b8ffe6f4ed27d6d16b179039` |

The matching `.zip.sha256` files travel with each ZIP. On the release machine,
`python3 scripts/verify_team_release.py` checks archive integrity, excludes
local settings/runtime state, and compares common source/configuration files
across the three packages and the GitHub source export. The clean exported
source passed 538 offline regression tests (one local-artifact integration
test skipped). The installed developer Mac app was not replaced.

This is **not yet a production-certified installer**. No live UR/Klipper print
was run as part of this release. This build cannot stand in for the required
wired-network and workcell acceptance test on each colleague's computer. The Mac app is ad-hoc
signed, not Apple Developer ID notarized, and the Windows app is unsigned.
Follow [TEAM_DEPLOYMENT.md](TEAM_DEPLOYMENT.md) for stopped-only rollout,
machine-local network setup, acceptance, and rollback.
