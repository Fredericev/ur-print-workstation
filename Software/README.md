# UR Print Workstation

UR Print Workstation is a local, English-first control console for the tested
UR3e + ROS 2 + MoveIt + Klipper workflow in the parent project. The interface
also supports an immediate English / Chinese language switch.

The application exposes only operations that are already backed by project
scripts:

1. Eight selectable workstation runbooks, from `Connection` and G-code
   preparation through planning, validation, synchronization, and monitoring.
2. Resettable operator safety confirmations.
3. Read-only UR3e wired-network, Dashboard, and RTDE checks.
4. A direct workstation-to-Moonraker **read-only preflight** for Klipper state and the
   configured native `extruder`, including its heater/sensor and extrusion
   readiness. It sends no G-code or motion command.
5. ROS container, detached UR driver, and ROS Klipper bridge readiness.
6. External Control connection verification.
7. MoveIt startup with the real calibration file used by the project.
8. A combined plan-only dry run: the ROS bridge verifies Klipper while MoveIt
   plans the UR path, and neither machine moves.
9. One guarded synchronized test owned by ROS: MoveIt controls the UR3e and the
   ROS Klipper bridge controls Klipper through Moonraker.
10. An interactive 3D Robot Path Review with the actual UR3e collision meshes,
    calibrated G-code path, UR/Klipper event layers, playback, and a plan-only
    MoveIt validation.
11. A hardware-isolated Offline robot simulation that estimates the six UR3e
    joints from the saved calibration and plays the arm together with the
    toolpath without starting ROS, MoveIt, Docker, Klipper, or the robot.
12. Guarded Print Execution that checksum-locks the complete G-code timeline,
    current table/measured TCP, calibration, collision-aware MoveIt result, approach
    trajectory, print trajectory, and synchronized UR/Klipper blocks before
    allowing separately requested move-to-start and reviewed-path execution.
13. A separate Tool0 motion-test mode for commissioning before the final
    print-head CAD/TCP is available. It binds the G-code process point directly
    to `tool0`, disables all heating commands, permits the path to touch but
    not pass below the configured table plane, and still requires full MoveIt
    collision and joint-limit validation. It does not weaken Production Print
    calibration. Tests can select either a bounded sample or the complete
    reviewed model trajectory. Native E motion requires a separately prepared
    hotend at extrusion-ready temperature, even when the software sends no
    heating commands; neither option certifies material printing.

## Offline and Online modes

The mode selector in the header separates local job preparation from live
robot access:

- **Offline** is the default on every new application launch. It permits local
  G-code import and parsing, dual-channel draft planning, tabletop-aware
  placement, offline Cartesian 3D route playback, estimated robot + toolpath playback when local IK succeeds, and
  playback of an exact cached MoveIt result.
  New live UR, Klipper, ROS, MoveIt, dry-run, and synchronized operations are
  rejected by both the interface and backend.
- **Online** permits live device checks and the existing guarded control
  workflow. Selecting Online does not start the container runtime, ROS, MoveIt,
  External Control, Klipper, or robot motion; the operator must still complete
  the Connection runbook.

Switching back to Offline does not silently stop ROS services that may already
be running. Stop and cancel controls remain available for recovery, while new
start and motion actions are locked.

The UR wired-network check and Klipper/Moonraker preflight do not require ROS:
they are direct read-only checks from the workstation computer. In contrast, `Capture current
tool0 pose` reads the live ROS transform from `base_link` to `tool0`, so it
requires the UR driver and fresh joint states. It does not require MoveIt.

## G-code import and parsing

Choose `G-code Import & Parse` from the runbook selector to open the offline
file-preparation workspace. First choose **Original G-code** or **UR trajectory +
extrusion G-code**. The original workflow accepts `.gcode`, `.gc`, `.ngc`, and
`.nc` files up to 20 MB and is otherwise unchanged. The paired workflow accepts
one six-joint `.csv` and one timed `.gcode` (20 MB each), checks their shared
time axis offline, and remains execution-locked. It cannot yet feed MoveIt or
Print Execution. The exact v1 file syntax and the existing coordination method
are recorded in [UR/extrusion shared timeline](../docs/ur_extrusion_shared_timeline.md).

The parser reads the selected file in memory and returns:

- command, move, layer, extrusion, retraction, and estimated-duration summaries;
- XYZ bounds and a bounded XY print/travel preview;
- top-command and source-file details;
- structured preparation metadata for homing, plus warnings for inch units,
  approximate arc previews, and missing feed rates;
- blocking findings for emergency, forced-motion, shell, firmware-restart, and
  configuration-save commands.

This runbook does not save the source file, contact ROS, connect to Moonraker,
or send any hardware command. A successful parse means the file is ready for
operator review; it does not mean the file is ready to execute.

After review, `Path planning` creates a read-only dual-channel draft:

- **UR Robot** owns G-code X, Y, and Z tool motion through
  `ROS 2 -> MoveIt -> UR driver`;
- **Klipper** owns extrusion E, retraction, heaters, fans, machine settings,
  and preparation commands through
  `ROS 2 -> Klipper Bridge -> Moonraker`;
- the synchronized timeline shows combined, UR-only, and Klipper-only motion
  counts and representative source lines.

The draft remains execution-locked. It requires the installed print setup,
current-table checks, collision-aware MoveIt validation, and preparation-command
review before any execution workflow can use it.

### Adjustable tabletop and print placement

The Workcell runbook records the operator-entered **actual tabletop Z** in
`base_link`, then separately accepts the desired **Print Z** where G-code
`Z=0` places the nozzle TCP. Adjusting the table collision plane does not
automatically shift the print path. The optional height trial helps check
clearance; it is not a prerequisite for setting either value.

For each imported file, the backend automatically:

- rotates and scales the G-code with the installed print setup;
- translates the centre of the extruded print bounds to the fixed tabletop
  centre; travel-only moves do not affect placement;
- maps G-code `Z=0` to the selected Print Z;
- checks the deposited path against the current table footprint and edge margin;
- applies the installed tool/TCP and Klipper extrusion mapping; and
- requires a new MoveIt collision and joint-limit review before execution.

The installed table geometry comes from
`workspaces/ur_ros2_ws/config/workcell_geometry.json`: a `0.45 × 0.54 m`
surface centred at `(-0.2925, 0) m`, yaw `0°`, with a `0.01 m` edge margin.
Its surface Z is operator-adjustable and **must not be assumed to be -14 mm**;
the thickness remains explicitly unknown.
Robot reach and arbitrary 3D obstacles are handled by the MoveIt planning
scene rather than by a manually entered rectangular Work Envelope.

The previously confirmed tool orientation, TCP/tool0 reference, G-code
rotation, scale, and extrusion mapping are treated as installation settings
and automatically revalidated for every source checksum. A missing or
incomplete installed print setup fails closed instead of opening an editable
per-job Workcell page.

### 3D Robot Path Review

Select `Robot trajectory & 3D review` from Motion Planning. The screen uses the
UR3e visual meshes from the installed
`ur_description` package and the robot-specific joint origins from the
project's extracted UR calibration. It shows:

- the calibrated print and travel path in `base_link`;
- extrusion and retraction events owned by the ROS Klipper bridge;
- the fixed table footprint and source layers;
- orbit, pan, zoom, camera presets, layer filtering, segment inspection, and
  timeline playback;
- a locally estimated robot joint animation in Offline mode;
- the validated robot joint animation returned by the MoveIt plan-only check
  when an online result is available.

The initial 3D view now runs a local numeric inverse-kinematics estimate against
the robot-specific UR3e chain. This unlocks `Play robot + toolpath` in Offline
mode and lets the operator inspect the expected arm posture along the path. It
is labelled `SIMULATION ONLY`, remains execution-locked, and does not claim
collision, joint-limit, singularity, or MoveIt validation. `Validate with
MoveIt` remains the separate online safety check: it sends the full converted
path to a dedicated ROS planning node, validates it in consecutive chunks that
inherit the preceding joint state, loads the active planning scene, computes
collision-aware Cartesian paths, checks every returned fraction and the joined
trajectory's joint steps, then returns a display-sampled copy of the validated
full-resolution joint trajectory.

The planned nozzle route, layers, segment inspection, progress slider, and
`Play toolpath` remain available offline even if local IK cannot produce a
reliable six-axis trajectory. In that case the UR3e model stays still; the
route preview is not a robot-motion or collision validation.

Local inverse kinematics is generated as a bounded background task. The UI
submits the exact file once, polls short status requests, and keeps showing the
generation phase until the result is ready. Identical file/profile requests
share one calculation, while a changed G-code file, print setup, or simulation
mode receives a separate source-bound result. Leaving the review or selecting a
different file prevents an older completion from replacing the current view or
starting an online MoveIt check.

If the exact same G-code and calibration were validated previously, the
matching six-axis result is labelled as a cached online trajectory and can be
replayed read-only while the workstation is Offline. A changed file or
calibration cannot reuse that cache.

Planning-policy changes also invalidate cached reviews and prepared execution
packages. The current continuous-Cartesian approach policy requires final
joint-interpolation validation and an exact joint-state join from the approach
endpoint into the newly planned print path. Beginning a replacement review
revokes the earlier result and execution package immediately, even if the new
attempt fails. The workstation and ROS executor both enforce that revocation;
old files are retained for diagnosis, not silently reused for motion.

The approach uses the current joint posture, reaches a staging pose 0.12 m above
the print start, and descends with the same seeded Cartesian IK chain. There is
no automatic OMPL detour if this route fails. Conservative approach limits are
0.35 rad per raw joint step, 180 degrees cumulative travel per joint, and
90 degrees backtracking per joint. These cumulative limits do not apply to the
repeated motion in the printed shape.

The planner retains every accepted Cartesian IK state locally. It does not use
the Cartesian service's TOTG-resampled output, which can geometrically shortcut
the path near the table. IPTP may change timing but must preserve every position
and the point count. Both CSV-precision, position-only paths are then checked
against a complete planning-scene snapshot at a maximum spacing of 1 degree per
joint. This is the geometric planning review, not the final smooth execution
curve or a claim that execution timing is the same as RViz playback.

Driver and MoveIt startup use the same measured kinematics file. After changing
that file, restart MoveIt and revalidate; changing a display model name does not
update geometry. Tool0 cold tests still omit the unmeasured print head. Finite
sampling is not continuous-space collision certification, and the executor's
small live-state-to-recorded-start tolerance segment is outside this saved-path
check. The executor separately checks its initial alignment and nominal smooth
curve before dispatch. Physical tool/TCP validation and execution safety gates
remain required.

The planning node has no trajectory-controller action client and sends no
Klipper command. Every result is marked `execution_locked`. A passed review
still does not replace operator approval or the later execution safety gates.

### Print Execution

Select `Print Execution` only after the exact path has passed the online MoveIt
review. Preparing an execution package does not move hardware. It creates an
immutable manifest whose ID covers every executable field and whose artifact
hashes cover the exact approach and print joint trajectories. Changing a block,
timing value, extrusion amount, safety parameter, or trajectory after review
invalidates the package.

The host maps each complete G-code Cartesian interval to the corresponding
MoveIt joint trajectory using Cartesian path progress computed from tool0
forward kinematics. During preparation, `bounded-quintic-v6-source-timed`
freezes one complete print profile's positions, velocities, accelerations and
final times. Monotone interior waypoints can have nonzero velocity; only the
approach/whole-job endpoints and explicit process-only holds stop. Timeline
markers do not create independent motion goals. Continuous minimum-jerk curves use shared interior
velocity and acceleration, rather than resetting acceleration at every dense
IK waypoint. Every final curve is wire-certified; failure to certify blocks
preparation instead of selecting a multi-hour fallback. Exact process-marker
knots and explicit stationary holds remain. Redundant unlabelled neighbours
within one controller update may be merged; support knots lie on the source
chord. The unified reviewed print path adds support at no more than 0.02 rad per joint (the
serialized general certificate remains 0.02 rad). The same-progress Bernstein
corridor is 0.0012 rad, not a Cartesian
tolerance: a separate RobotModel FK gate enforces 0.5 mm / 0.005 rad tool0
fidelity against the original reviewed trajectory. Local time allocation
avoids expanding an entire trajectory because of a tiny clipped interval.
Both approach and print use limits of 0.2 rad/s, 2.5 rad/s² and 100 rad/s³.
Source chord times remain lower bounds. Limits are checked across each full
polynomial, not just at its waypoints. This is C2 interpolation, not Ruckig and
not a physical jerk guarantee. Adjacent timing ratios are searched at fine
logarithmic resolution before bisection so narrow feasible corners are not
skipped and replaced by a large, unnecessary time stretch.
G-code `F` is interpreted in mm/min (`F1800` is 30 mm/s); nominal move duration
is distance in mm divided by `F`, multiplied by 60. This nominal duration and
the source MoveIt intervals are lower bounds, not promises of physical print
speed: joint limits, bounded corner transitions and the native extruder's
velocity/acceleration/distance limits can
lengthen a move. A process boundary by itself does not impose a 0.25 s floor.
The driver executes the frozen joint q/v/a/time trajectory through its fixed
URScript, not a newly generated URScript per G-code line.
Preparation accounts for URClient 2.11.0's actual integer q/v/a encoding and
float32-to-microsecond segment timing. Profiles must survive this wire conversion
unchanged; an unverified URClient version or old unquantized package is rejected.

Before either actuator is armed, a read-only MoveIt gate checks the final
nominal curve and initial alignment against the whole robot and attached tools,
at a maximum per-joint sample spacing of 0.002 rad. It verifies joint bounds,
the fixed table's parent/shape poses, dimensions, frame and orientation, and
rejects table collision exemptions. Missing, timed-out or failed validation
blocks motion. The old geometric-review flag cannot bypass this check.
Production uses one local C++ batch validator, not one DDS service round trip
per joint sample. Package math is certified once per executor process and
reused only for the exact hashed profile. Status polling uses a file-bound
observation cache; movement buttons retain full authorization and artifact
checks. No live collision certificate is reused across a changed scene.

Choose the Tool0 test extent before preparing the immutable package:

- **Sample** ends at a complete block after its requested duration, so a long
  first block can exceed the nominal 30-second sample.
- **Full** retains every model-motion and extrusion/retraction block in the
  reviewed print window, through its final Cartesian point. Changing the extent
  requires a new package; it cannot expand an already prepared sample at runtime.

The model window extends from the first through the last positive-extrusion XYZ
move. Slicer initialization/lead-in and shutdown moves outside that window are
excluded. Heater, fan, homing and slicer preparation commands are never forwarded.
"Full" therefore means the complete reviewed model trajectory, not execution of
every raw command in a printer-specific G-code file.

UR and Klipper durations use the same final profile. V3 sends one complete UR
action and uploads one immutable Klipper virtual-SD program, then starts it
once. There is one 4 s coordination lead, not a lead and action-result wait at
every timeline segment. The native extruder is selected with
`ACTIVATE_EXTRUDER EXTRUDER=extruder`, then `G21` and `M83` establish filament
millimetres and relative E moves. Each `G1 E... F...` includes Klipper's
rest-to-rest acceleration and deceleration. Long moves are split to preserve
the 50 mm distance guard; every submove's ramp time is included before the
common UR timeline is frozen. Feed is solved from that final allotted duration,
rather than using distance divided by duration. The current installed limits
are 5 mm/s and 20 mm/s². Zero-extrusion travel intervals dwell intentionally;
the UR can keep moving during travel. Process-only retractions hold the UR as
specified by G-code. Dwell rounding is cumulative to avoid millisecond drift.
One terminal `M400` precedes G-code state restoration and virtual-SD completion;
ordinary moves do not insert additional `M400` barriers. Long execution
is polled through read-only `print_stats`, not a long `/printer/gcode/script`
HTTP request. Cancellation or uncertain post-start failure uses emergency stop.

The historical manual-stepper `UMS5_mold2.gcode` audit retained all 3,800 timeline segments. Its 3,475
adjacent moving boundaries have no forced zero-velocity stop. The previous
2,897 repeated start leads added 11,588 s; the new overhead is 4 s.
The obsolete commissioning retimer produced 18,919.124 s (about 5 h 15 min).
The source MoveIt path is 727.989169 s. This fixture contains 162 real
process-only holds totaling 210.600162 s at the installed unloaded-stepper
cap (5 units/s); moving source/feed/stepper lower bounds add up to 854.700354 s.
Thus even before bounded corner transitions the shared timeline is at least
1,065.300516 s (17 min 45 s), not 10 min. Do not subtract old leads from an old
profile's duration: a new complete curve must be re-certified. These are
offline nominal cold-test estimates, not a physical production-speed guarantee.
The historical v6 manual-stepper fixture took 1,328.669851 s (22 min 09 s), prepared in
about 16.5 s on the development computer, and retains zero artificial stops at
the 3,475 ordinary moving boundaries. Independent local MoveIt checks passed
66,599 approach/print candidate states; maximum tool0 translation deviation is
0.394258 mm, below the unchanged 0.5 mm gate. Cached approach time is 9.983196 s;
the real approach must be replanned from fresh joints. See
`CONTINUOUS_EXECUTION_VALIDATION.md` for evidence and model/hardware limitations.
Those saved durations predate native E acceleration accounting. Rebuild the
diagnostic audit and prepare a fresh native package to obtain current timing;
historical manual-stepper packages cannot run against the migrated bridge.

Coordination uses a best-effort UR relative hold and Klipper scheduled dwell;
there is no shared physical clock guarantee. The Klipper move must be confirmed
as queued before the UR trajectory is sent; either side missing the arming
deadline fails closed. Fresh stable unscaled UR timing is required without
relaxing safety settings. A reviewed production-mode package can now be started
explicitly after the UR reaches its reviewed print start; the blanket
physical-synchronization-certification lock has been removed. This is
best-effort software coordination, not certification of simultaneous physical
motion or material deposition. The executor sends no heater, fan, homing, or
slicer preparation commands. The operator must prepare and verify the hotend
separately before material printing. Native temperature and `can_extrude`
checks gate every coordinated E start, including Tool0 tests. A cold hotend may
connect, but cold extrusion is not enabled. Speed and flow overrides must be
100%. Segment endpoint timing does not certify instantaneous material flow
during E acceleration ramps. Session safety checkboxes, live readiness
checks, the reviewed-start requirement and stop-unconfirmed locks still apply:

1. Move to start executes only the reviewed approach; Klipper does not move.
2. Start the selected reviewed sample/full path starts the coordinated UR/Klipper
   continuous job after verifying that all six live joints are still at the locked
   first print point. Full-path completion is distinct from sample completion;
   action success does not prove a physically synchronized or deposited print.

Old v1/v2 packages, earlier smooth-policy packages, and manual-stepper packages
without `extrusion_protocol=klipper-native-extruder-v1` must be prepared again;
they are not valid current continuous jobs.
Restart the workstation backend to load changed Python code, then prepare a new
package; do not reuse old execution IDs. Existing stop-unconfirmed locks are not
cleared by this update. The installed UR script starts each spline segment from
measured joint positions, so tracking error, payload/TCP calibration and real
controller timing still require supervised hardware validation.

Pause/Resume are unsupported during the continuous job and are absent from the
UI; compatibility API requests fail closed without creating a pause file.
Stop requests coordinated cancellation of both channels and invokes the ROS
Klipper emergency-stop path. The physical emergency stop remains the primary
safety control.

The current executor controls native `[extruder]` for extrusion/retraction.
Its motor, heater and sensor are configured together; `[tmc2209 extruder]`
contains the driver configuration. `[manual_stepper lock_motor]` remains
independent. Printer-specific macro bodies and heater/fan preparation are not
silently expanded or executed by this path; prepare the hotend separately.
The legacy `manual_stepper` configuration/API key is retained as a compatibility
name but now selects a native object, defaulting to `extruder`. Legacy ROS
action names likewise remain; they no longer generate `MANUAL_STEPPER`
commands for the extrusion motor. Rebuild the ROS interface/bridge packages,
restart the workstation and bridge, and prepare new execution packages.

## Control ownership

The workstation application is the operator interface, not the Klipper motion
controller. Its direct Moonraker access is limited to the read-only preflight
in step 3.

For the dry run and synchronized test, the control path is:

```text
UR Print Workstation
        |
        v
ROS synchronized coordinator
        |--------------------------|
        v                          v
MoveIt / UR driver          ROS Klipper bridge
        |                          |
        v                          v
      UR3e                      Moonraker
                                   |
                                   v
                                Klipper
```

The ROS Klipper bridge is the only component in this workflow that may send
Klipper motion G-code. The workstation does not contain a direct-write
fallback. This keeps UR and Klipper timing, readiness checks, return-to-start,
and cleanup inside the same ROS-owned one-shot coordinator.

## Quick launch on macOS

Open the `Mac` folder and double-click `UR Print Workstation.app`. It is a native macOS
application with its own resizable window, Dock icon, menu bar, and embedded
workstation interface. It starts or reuses the local control service in the
background; it does not open Terminal or the default browser.

Keep the application inside this `Software` folder so it can find the backend,
configuration, project scripts, and operation logs. No command entry is
required. The app stops a backend process that it started when its last window
is closed. Existing ROS containers and detached hardware services retain their
normal project-managed lifecycle.

## Quick launch on Windows

Open the `Windows` delivery folder, choose the ZIP that matches the target
computer, extract the entire archive to a normal local folder, then
double-click `UR Print Workstation.exe`.

- `win-x64` is for normal Intel/AMD 64-bit Windows computers.
- `win-arm64` is for Windows on ARM, including Apple-silicon Parallels VMs.

The Windows package includes the application, a self-contained .NET runtime,
and the official Python embeddable runtime. It does not require a separate
Python or .NET installation. Windows 11 normally includes WebView2; if it is
unavailable, the application opens its local interface in the default browser.

Configuration, calibration, logs, the writable project workspace, and WebView
data are stored below `%LOCALAPPDATA%\UR Print Workstation`. Online operations
also require Docker Desktop using Linux containers, a wired route to the UR,
and Windows Defender Firewall access for ports 50001 through 50004.

The Windows launcher and Offline workflow can be tested without hardware.
Online UR/ROS/MoveIt/Klipper behavior must still pass a guarded acceptance test
on the target Windows computer and real workcell before physical motion is
authorized.

## Safety boundary

- The physical emergency stop is always authoritative.
- This application is not an emergency stop.
- Real motion stays locked until all safety checks are confirmed, External
  Control is confirmed playing, and the operator clicks the specific action
  button. No typed confirmation phrase is required.
- The backend repeats ROS, External Control, MoveIt, and ROS Klipper bridge
  readiness checks before it launches the synchronized test.
- The step 3 workstation-to-Moonraker check is read-only. All later Klipper motion is
  issued by the ROS Klipper bridge.
- The server binds only to `127.0.0.1`; it is not a remote-control service.

## Start the real workstation

For troubleshooting, the legacy browser-based fallback can still be started by
double-clicking `start.command`, or by running:

```bash
cd /path/to/extracted-project/Software
python3 control_station.py
```

This fallback opens the interface at <http://127.0.0.1:8876>.

## Start demo mode

Demo mode never calls Docker, ROS, Moonraker, Klipper, or the UR robot. Use it
to teach the workflow and verify the interface:

```bash
./start-demo.command
```

The header displays `DEMO MODE` whenever hardware commands are disabled.

## Configuration

Use the settings button in the application. Values are stored in
`config.json`. The backend validates all addresses, axes, stepper names, and
motion limits before saving or executing anything.

Default addresses:

- UR3e: `192.168.0.104`
- Moonraker (wired Raspberry Pi): `http://192.168.0.168`
- Native extruder: `extruder` (legacy saved key: `manual_stepper`)
- UR control / RTDE frequency: `125 Hz`

Leave `ROS PC IP` empty to let the platform runtime infer the wired workstation
address used to reach the robot.

The installed print setup is stored separately in `gcode_calibration.json` after
commissioning. It is reused and revalidated automatically for each new G-code
file. The tabletop X/Y footprint stays in the installed workcell configuration;
its base-link Z surface is a separate operator-entered coordinate for the
actual physical surface. The initial installed value is `-0.014 m` (−14 mm).

The normal runbook goes from G-code Preparation to **Tabletop Z**, then
**Print Z & nozzle setup**, and then Motion Planning. Set the actual tabletop
surface in base-link millimetres first, then independently set the desired
nozzle TCP height for G-code Z=0. This tabletop is height-adjustable: after
moving it, enter its new actual height. For example, a tabletop actually at
−20 mm and Print Z at −15 mm creates a nominal 5 mm modelled gap. Software
collision checks use the entered table height, so confirm it matches the
physical setup before commanding motion.

**Print G-code Z=0**
is an operator-selected absolute base-link Z datum for the route. Saving it
shifts the entire G-code path in Z only; X/Y, the entered physical tabletop,
and its MoveIt collision plane stay fixed. This is a path-placement setting,
not a claim that a print surface exists at that height. An elevated air run can
therefore be reviewed, cold-tested, and prepared without a nozzle-to-table gap
trial. The selector rejects values below the physical tabletop and has a
2 m sanity bound; the full path must still pass the usual 3D/MoveIt collision,
reachability, current-posture, and execution checks. A saved Z does not itself
command motion or certify material printing. Changing it invalidates prior
reviews and execution packages; review and prepare the new route before motion.
If its saved record is missing or corrupt, re-enter the desired Z to restore a
valid selection. **Reset print Z to tabletop** selects the current entered
tabletop datum for the path without changing the tabletop geometry.

Production path review now uses the full installed-printhead CAD bounding box
as its conservative collision envelope. The former full-length TCP cylinder
extended below the CAD nozzle tip when the TCP had lateral offset, creating a
false tabletop collision at printable first-layer clearance. The nozzle TCP
must lie inside that CAD envelope; otherwise planning and execution reject the
profile until its collision geometry is recalibrated. The fixed tabletop guard,
full-path MoveIt checks, and execution-time scene checks remain active. This
model does not verify the physical mounting transform or actual nozzle gap:
inspect those on the real machine before material printing. Changing Print Z
cannot reuse an older MoveIt review, preparation result, or execution package.

Saving a new tabletop Z directly updates the planning collision plane; it
does not move UR or certify the physical measurement. The normal configuration
does not require the old 10 mm height-trial workflow. Any changed tabletop or
Print Z invalidates old MoveIt reviews, preparation results, and execution
packages. Rebuild the G-code review, MoveIt plan, and execution package from
the two saved heights before moving to print start or printing. A MoveIt pass
only establishes finite-sample collision/reachability checks against the
*entered* workcell model, not that the actual table is where it was entered.
The CAD nozzle offset is provisional until the operator measures and saves the
installed tool0-to-nozzle TCP. Legacy bare-tool0 tabletop motion is locked while
the new head is installed. Starting the reviewed production-mode path is an
explicit operator action; the software does not certify physical UR/Klipper
synchronization or control hotend temperature.

Legacy commissioning API (not shown in the normal interface):

- `GET /api/mode` returns the current operator-selected mode.
- `PUT /api/mode` switches explicitly between `offline` and `online` and is
  rejected while an operation is active.

- `GET /api/gcode/calibration` loads the saved profile or a safe draft.
- `POST /api/gcode/calibration/validate` performs the offline transform and
  fixed-table checks.
- `PUT /api/gcode/calibration` saves only a profile that passes every
  calibration gate.

3D path-review API:

- `POST /api/gcode/review` automatically applies the installed print setup and
  creates the offline 3D review. Send `Prefer: respond-async` to receive a
  background review job immediately; callers that omit the preference retain
  the legacy synchronous response.
- `POST /api/gcode/review/simulation` creates an execution-locked local 3D
  estimate from the latest validated draft; it cannot be promoted to MoveIt or
  Print Execution authorization. It supports the same asynchronous preference.
- `GET /api/gcode/review/jobs/<job_id>` returns the queued/running phase and the
  completed review or its structured failure. Results are bound to the exact
  source checksum, installed/draft profile, review mode, and offline-IK
  algorithm version.
- `POST /api/gcode/review/moveit` starts a ROS/MoveIt plan-only validation.
- `GET /api/gcode/review/moveit/<job_id>` returns progress and the validated
  display-sampled joint trajectory from the full-resolution chunked check.

Complete and incomplete read-only simulations use the same confirmed fixed
tabletop and automatic-centering rule. An incomplete tool/TCP profile remains
non-authoritative for motion, but the software no longer invents a path-fitted
table or a tabletop thickness for display.

## Tests

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
node --check app/app.js
node --check app/path-review-3d.js
./build-macos-app.command
./build-windows-package.command
```

The test suite runs in simulated mode and does not contact or move hardware.

For a repeatable, read-only conversion audit across synthetic edge cases,
downloaded public G-code samples, local files, and calibrated UR3e FK/IK
round trips, run:

```bash
python3 tools/audit_gcode_conversion.py
```

The JSON result is written under `runtime/algorithm_audit/`. A passed offline
audit verifies parsing and numeric conversion only. Real execution still
requires the online MoveIt planning scene, collision checks, confirmed fixed
table/tool setup, and an operator-supervised low-speed hardware acceptance test.

## Operation logs

Every launched operation writes a timestamped text file under `runtime/jobs/`.
That directory is ignored by version control and can be copied into a support
ticket when troubleshooting a failed connection or plan.
