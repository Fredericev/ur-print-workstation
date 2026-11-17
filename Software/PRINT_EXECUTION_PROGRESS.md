# Print Execution Development Progress

Updated: 2026-09-17

## 2026-09-23 operator-start update

- The blanket production-mode certification lock was removed. After a reviewed
  move-to-start, an explicit operator action can start the production-mode
  UR/stepper path, subject to the existing live checks, collision validation,
  immutable package checks, and stop-unconfirmed lock.
- This is software coordination, not proof of physical simultaneous motion or
  deposited material. The execution package still sends no heater, fan, homing,
  or slicer-preparation commands; the real hotend must be prepared and verified
  separately before material printing.
- No connected UR or Klipper run was performed for this update.

## Current v6 speed correction

- Fixed narrow adjacent timing-ratio feasibility windows and wrong-direction
  tangent projection without relaxing collision/FK gates or source-time bounds.
- Installed 0.2 rad/s, 2.5 rad/s², finite 100 rad/s³ interpolation limits and
  a tighter 0.0012 rad corridor; whole-curve wire certification remains mandatory.
- Fresh complete UMS5 fixture: preparation 16.48 s, cached approach 9.98 s,
  full cold timeline 22 min 09 s instead of the old five-hour profile.
- All 3,475 ordinary moving boundaries retain nonzero shared velocity;
  independent MoveIt checks passed for 66,599 final states (max tool0 deviation
  0.394258 mm / 0.001171 rad). Tests also retain every explicit source hold.
- Current test/build evidence and limitations are in
  `CONTINUOUS_EXECUTION_VALIDATION.md`. No hardware commands were issued.
- Restart backend/bridge while devices are safely stopped, replan from fresh
  joints and prepare again. Old smooth-policy packages cannot be reused.

## Continuous cold-job execution (historical commissioning design)

- V3 freezes one continuous UR print profile plus a process/extrusion timeline.
  It sends one UR goal and starts one uploaded Klipper virtual-SD file.
- Ordinary block boundaries no longer force stops, result waits or repeated
  4-second start leads. Explicit G-code travels/retractions remain intentional.
- Pause/Resume and their pause-file runner path were removed; Stop and all
  stop-unconfirmed protections remain. Old v1/v2 packages must be prepared again.
- Moving stepper segments use zero-acceleration queued motion for unloaded
  cold testing only; physical synchronization and production printing remain
  unverified/locked. No shared physical clock is claimed.
- Offline RViz can replay the exact full v3 curve and timing with checksum and
  manifest-identity verification; it never contacts either device.
- The complete UMS5 fixture audit preserves 3,800 segments and checks 3,475
  ordinary moving boundaries with zero artificial boundary stops. It retains
  903 intentional zero-extrusion intervals and one final M400 barrier.

The sections below record earlier implementation stages, not current execution
or authorization semantics.

## Fixed tabletop and automatic placement

- The confirmed table is now a shared, read-only installation configuration:
  centre `(-0.2925, 0) m`, size `0.45 × 0.54 m`, yaw `0°`, and surface
  `Z=-0.014 m` in `base_link`.
- Every file is rotated/scaled and then translated so the centre of its
  extruded print bounds lands at the table centre. Travel-only moves do not
  affect placement, and G-code `Z=0` is the tabletop.
- The same normalized Z datum is used by the 3D review, offline IK animation,
  MoveIt path input, collision bed, and immutable execution package; the UI
  can no longer display one height while execution uses another.
- Retained the existing `Tool0 motion test`; it now permits tool0 to touch the
  measured table plane while still rejecting paths below that plane.
- Tabletop execution remains fail-closed until the centered path fits inside
  the fixed table and MoveIt passes collision review.
- Verified `CCR10_mold.gcode` offline with tabletop Z `-0.014 m`: path minimum
  and tabletop both resolved to `-0.014 m`, and the 360-point UR3e IK animation
  passed with a maximum TCP error of `0.000799 m`. No hardware was contacted.

## Completed

- Added fixed-table geometry and measured nozzle TCP safety gates.
- Added immutable execution packages that bind the exact G-code, calibration,
  MoveIt review, approach trajectory, print trajectory, and checksums.
- Added a ROS 2 print executor for synchronized UR trajectory and Klipper
  extrusion blocks.
- Added guarded Move to Start and Start Print operations with exact typed
  authorization phrases.
- Added pause-after-current-block, resume, and coordinated UR + Klipper stop.
- Added live execution status, block progress, source line, process state, and
  completion views in the Print Execution runbook.
- Added full unsampled G-code timeline handling, including Klipper-only
  extrusion and retraction while the UR holds position.
- Added MoveIt collision geometry for the fixed tabletop and mounted tool.
- Added the Klipper action start delay used to align extrusion with UR motion.
- Corrected Print Execution sidebar locking, footer navigation, and collision
  validation wording.
- Added English and Chinese text for all new interface labels.
- Added Cartesian path-progress values computed from MoveIt tool0 forward
  kinematics, so G-code intervals are mapped to the real six-axis trajectory
  instead of approximated by trajectory-point index.
- Split synchronized blocks whenever feed rate or extrusion rate changes.
- Bound every executable manifest field into the execution ID; editing any
  block, timing, extrusion, limit, bed/TCP value, or artifact reference now
  invalidates the package.
- Split Offline draft review from physical authorization: an incomplete
  validated calibration draft can now open a clearly labelled, read-only 3D
  robot + toolpath simulation, while MoveIt and Print Execution remain locked
  until the exact measured profile is complete and saved.
- Added a separate `Tool0 cold motion test` operation mode. It treats `tool0`
  as the process point, forces the effective TCP and tool collision radius to
  zero, ignores measured nozzle/bed-edge requirements, keeps heating disabled,
  and uses the measured table plane as a lower bound without an added clearance.
  Production Print remains fail-closed on measured TCP and bed geometry.
- Replaced the former path-fitted tabletop preview with the same confirmed
  fixed workcell geometry used by validation and MoveIt.
- Corrected Marlin-compatible E-axis state handling: `G90` and `G91` now clear
  prior `M82`/`M83` overrides. This prevents Cura-style end G-code such as
  `G91` followed by `E-2` from becoming an erroneous hundreds-of-millimetres
  retract.
- Corrected Print Execution's live ROS subscriptions to match the UR driver's
  QoS contracts: transient-local/reliable for
  `robot_program_running` and sensor-data QoS for `/joint_states`. This fixes
  the contradiction where Connection reported healthy live signals but the
  executor could not receive them.
- Added per-signal Print Execution preflight details for External Control,
  joint state, the Klipper action server, and the trajectory action server;
  future failures name the missing signal instead of combining four causes in
  one generic error.
- Redesigned the Workcell & Tool Setup mode selector as bounded, responsive,
  fully clickable cards. Radio inputs are isolated from the global text-input
  layout, mode changes use the correct change event, and the execution progress
  heading now reflows without overlapping its state label.

## Verification completed

- All 172 workstation tests passed after the Tool0 test-mode, bounded
  commissioning sample, E-axis state,
  final UI, Cartesian-progress,
  rate-boundary, manifest-integrity, and cross-platform packaging changes.
- The complete-timeline parser and Cartesian-progress slicing tests passed.
- The guarded Print Execution API flow passed end to end in Demo mode,
  including deliberate rejection of a wrong authorization phrase.
- 34 ROS Klipper bridge/interface tests passed inside the running container.
- ROS interface compatibility checks passed for `start_delay_s` and controller
  switching.
- The isolated UI flow was exercised from G-code import through calibration,
  3D review, immutable package preparation, Move to Start, Start Print, and
  completion. No real hardware was contacted.
- `ur3e_print_planner` rebuilt successfully inside the ROS container after the
  full-resolution trajectory artifact format was upgraded to include Cartesian
  path progress.
- The native Mac application and both Windows 0.12.0 packages were rebuilt.
  The Windows x64/arm64 archives include `print_execution.py`, the ROS print
  executor, the upgraded Klipper action, and the upgraded MoveIt planner; ZIP,
  external checksum, and complete internal checksum-manifest checks passed.
- A packaging regression test prevents future Windows builds from omitting the
  Print Execution backend.
- The Offline draft flow was exercised in an isolated native-browser session:
  the six-axis animation played, incomplete TCP/bed gates stayed Required, and
  the MoveIt control remained disabled. Both Windows 0.12.0 archives were then
  rebuilt and their external checksums verified.
- The current 448 KB `CCR10_mold.gcode` was converted without sampling to
  13,951 tool0 waypoints. Live MoveIt validated the approach and all 19 path
  chunks with Cartesian fraction 1.0, collision checking enabled, and joint
  limits passed. The resulting 12,093-point six-axis trajectory was bound into
  a cold synchronized commissioning package. The executable sample contains
  the first 13 complete synchronization blocks (33.844 seconds of coordinated
  motion, ending at source line 176); the full path remains validated. Its
  largest Klipper block is 15 units (guard limit: 200).
- Live read-only checks on 2026-08-05 confirmed the UR reverse interface,
  fresh joint states, External Control running, Moonraker reachable, and the
  ROS Klipper bridge ready with `stepper_x` present.
- A read-only instance of the repaired Print Execution subscriber observed all
  four required live signals simultaneously: External Control, fresh six-joint
  state, Klipper action server, and passthrough trajectory action server. No
  controller switch or motion goal was sent during this probe.
- Windows 0.12.1 x64 and arm64 packages were rebuilt from the repaired source;
  both archive checksums passed and package inspection confirmed the QoS,
  diagnostic, and interface-layout fixes are included.

## Before the first production print

1. Deliberately restart the ROS container/stack from the application before
   Print Execution so the running processes load the rebuilt planner, executor,
   and Klipper action definition. This was not done automatically while the
   robot was connected.
2. Confirm the installed fixed-table configuration, then enter the measured
   nozzle TCP, tool collision radius, G-code rotation, and tool orientation.
   Never use the Demo fixture values for the tool.
3. Re-run the exact file's full online MoveIt review after calibration.
4. Commission at the teach-pendant speed below 20%, with the physical emergency
   stop in hand, first without material and then with a short low-flow test.

## Cold-test commissioning state (historical)

- No real Print Execution was run.
- The workstation profile migrates to `tool0_motion_test`: G-code `Z=0` and
  the fixed table plane both use `-0.014 m` in `base_link`. Motion Test uses
  the same fixed X/Y table bounds and still requires the full online MoveIt
  collision/joint review. The redundant configurable Work Envelope was removed.
- The current Klipper `printer.cfg` was backed up as
  `printer.cfg.before-tool0-cold-test-20260805`. Its unconnected
  `[heater_generic hotend]` section is commented with the marker
  `UR_TOOL0_COLD_TEST_DISABLED`, and Klipper is ready after
  `FIRMWARE_RESTART`. Restore and validate the real temperature sensor before
  any heated production print.
- The live Docker/ROS/UR processes were not stopped or restarted; no UR or
  Klipper motion command was sent while preparing this test.
- The native Mac workstation was reopened after the UI update. The live
  Docker/ROS stack remained connected; only the local application process was
  restarted.
