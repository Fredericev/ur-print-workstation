# ROS startup repair record — 2026-10-09

The actual failed job triggered by **Start ROS and UR driver** was `180a9f94be8d`.
Host-to-UR networking, Docker, and container-network checks had all passed before this failure:

```text
cp: cannot create regular file '/opt/ros/humble/share/ur_robot_driver/config/ur3e_update_rate.yaml': Read-only file system
```

The configuration-preparation script replaced the host file with a rename on every run,
while Compose mounted that individual file read-only. The existing container remained bound to
the old inode; after detecting different contents, the configuration-application script tried to
copy onto the read-only destination. This failed before the UR driver was actually launched.
After the repair, preparing the same value preserves both inode and modification time;
changing the rate also preserves the existing inode. The application script only verifies the
read-only mount and provides an explicit refresh instruction for a stale mount. The affected local
container was refreshed, and repeated configuration-preparation and application checks succeeded.

Separately, the workspace overlay's driver configuration was still 500 Hz; the 125 Hz setting in
the system's read-only mount did not override it. The preparation script now synchronizes the
overlay configuration for the selected model without changing other models, ensuring that the
selected rate is still used when the driver selects the workspace package.

An earlier compilation attempt had a separate failure: colcon encountered
`Errno 35: Resource deadlock avoided` while reading and writing old generated files, ultimately
wrapping it as a shell-extension error. Container shell extensions, file permissions, and the
writable workspace mount were normal. Mac file metadata confirmed 724 iCloud placeholder files
in `build` and 123 in `install`. Reading `colcon_command_prefix_build.sh` from the container
reproduced the same Errno 35.

First, a macOS recovery tool that checks only generated-directory file flags retained the old
`build/install` and created empty directories for rebuilding. The backup root contains
`COLCON_IGNORE` so old artifacts are not rediscovered as build inputs. Source, configuration,
runtime jobs, and user calibration are excluded from this recovery. A complete startup retest
found iCloud placeholders in the new artifacts again, demonstrating that rebuilding alone was
not a durable solution.

The final startup workflow places Mac compilation output in a local cache outside iCloud.
Compose binds `build/install` separately while preserving the existing container paths,
`/home/ubuntu/ur_ros2_ws/build` and `install`. Every shell entrypoint that creates a container first
selects the same cache. Build entrypoints compare the actual mounts with the selected directories
and explicitly reject old bindings, rather than replacing directories underneath a running
container. Build markers are also read and written in the local cache. The old placeholder-recovery
tool remains a separate utility and is no longer part of normal startup.

Old artifacts from this repair are retained at:

```text
workspaces/ur_ros2_ws/generated-artifacts-dataless-20261009T225806.309407Z-5v6tw6ld/
workspaces/ur_ros2_ws/generated-artifacts-dataless-20261009T232210.519888Z-pzbrys3z/
```

The current actual local compilation output is:

```text
/Users/lilbo/Library/Caches/UR Print Workstation/ros-artifacts/c765dfc296d09cd58cd7f87d/{build,install}
```

After a complete rebuild succeeded, the original directories were retained and the artifacts were
copied into that cache. Migration checks compared directory and regular-file permissions and
content SHA256 values, original symlink targets, and build markers: all 1,950 `build` records and
736 `install` records matched. The copy preserved modification times and executable permissions;
regular files did not inherit iCloud document-tracking flags. The new cache contained no
`SF_DATALESS` files, and the container mounts were then refreshed.

## Validation

- The actual configuration script returned 1 with a read-only-file error before the repair;
  it succeeded after the mount refresh.
- Two consecutive preparations and checks of the 125 Hz configuration succeeded without recreating
  stale-inode mounts.
- Six configuration-mount/workspace-rate consistency regressions and five generated-directory
  recovery regressions passed.
- Eighteen overlay regressions passed, along with shell syntax checks.
- Eleven regressions covering cache selection, stale-mount rejection, environment inheritance for
  paths containing spaces, and repeated startup passed.
- The final complete Python regression suite passed 577 tests.
- The actual local ROS container completed a full overlay build. URCL, controller, and driver loading
  and ABI checks passed; the new printhead geometry hash was present in the compiled artifacts.
- The actual button-invoked `launch_real_robot_detached.sh` was run with the user's current
  configuration, `UR_DRIVER_ONLY=false`, `WAIT_FOR_READY=true`, and `HEADLESS_MODE=false`,
  and returned 0. The driver reported `System successfully started!`; the trajectory START control
  rate was 125 Hz. The Klipper bridge reported ready, and the 45-second RTDE stability and host
  port-forwarding checks for 50001–50004 passed.
- Driver and bridge build checks during the full startup, and a subsequent repeated build check,
  reused the cache directly without recompilation. Cache-mount, library-loading, and ABI checks passed.
- The final live API reported `container_running`, `driver_started`, `host_ports_ready`, and
  `klipper_bridge_ready` as true, and `rtde_conflict` and `unstable` as false.

The desktop app's initial retry was blocked by Klipper's read-only check before a new job was
created; networking recovered afterward. The final `/api/klipper/check` passed its Moonraker,
Klipper-ready, and `stepper_x` checks, preserving the existing connection gate. The same complete
script invoked by the desktop button passed real local hardware-startup validation. Teach-pendant
External Control was not yet running, which is the normal next step after driver readiness.
This repair did not select the UI's physical safety confirmations on the operator's behalf.

The user confirmed that Klipper was powered on and still at `192.168.0.168`. The current host route
used wired interface `en12`, with local address `192.168.0.111/24`. The target ARP address was
`0e:56:21:e1:d0:1a`; TCP 22, 80, 443, 7125, and ICMP did not respond. The user subsequently
confirmed that a phone or another computer on the same network could open the Klipper web page
and see printer status. The service was therefore reachable from another device, concentrating
the current problem on this Mac's access path. Wi-Fi was disabled, and IPv4 used only `en12`.
The same newly created local process could connect to UR Dashboard (`.112:29999`) and the gateway
(`.1:80`), while `.168:80` still timed out or returned `Host is down`. No HTTP proxy was configured
on the Mac, and no installed system network extension was found. mDNS found neither an available
Moonraker service nor a common Klipper hostname.

The investigation needed to distinguish stale neighbor-cache entries, an address conflict, and
network isolation between the wired interface and printer. An attempt to clear only `.168`'s
ARP cache was rejected by `sudo -n` because macOS required an administrator password; the diagnostic
script did not clear the cache. After the user was asked to run the single-address cleanup command
in their own terminal, they confirmed that the Mac could connect. Actual retests returned HTTP 200,
ready status, and hostname `BTT-CB1` from `/printer/info` on both the Mac and container; the REJECT
flag on the original target route had disappeared. The exact underlying network fault was not
further established; recovery alone does not prove an address conflict or network isolation.
No network-adapter, printer-address, or router setting was changed.

The full startup log is `/tmp/ur-ros-startup-repair-full-launch.log`, the final regression log is
`/tmp/ur-ros-startup-repair-final-python-tests.log`, and the migration-comparison log is
`/tmp/ur-ros-artifact-cache-migration.log`.

No arm trajectory, teach-pendant program start, extrusion, or heating command was sent. User
calibration files retained their contents from the start of this investigation, and old compilation
caches remain in the backup directories listed above.

The preceding validation of the new extruder covered offline features and compilation in an
isolated ROS environment, but did not complete acceptance of local online startup. This repair
retested the actual current container and application startup entrypoint.
