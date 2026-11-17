# Shared UR trajectory and extrusion timeline: current method and future algorithm interface

This document records the synchronization method actually used by the current
software, as a reference for future path-planning algorithms. The existing
original G-code → MoveIt → execution package workflow is unchanged. A separate
pair-file entry point accepts a UR trajectory and timed extrusion G-code for
**offline inspection** only; it cannot yet enter MoveIt review or generate an
execution package. These states must remain distinct.

## Core principle

First establish a reviewed, executable **shared motion timeline**, then make
both the UR joint samples and each extrusion amount refer to it. If a segment
must be lengthened to satisfy joint, collision, or extrusion-motor limits,
**UR and extrusion must be retimed together**. Slowing only one file is not
sufficient.

## What the current software actually does

1. Parse the original G-code's XYZ path, E increments, and nominal motion duration. A positive `E` increment means extrusion, a negative increment means retraction, and zero means no extrusion during the segment. After calibration, `klipper_units = extrusion_delta_mm × units_per_e_mm × direction_sign`. `direction_sign` is the motor-direction calibration established during installation and must not be applied a second time.
2. MoveIt reviews the path and outputs a six-joint trajectory CSV: `time_s,path_progress,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint`. `time_s` is in seconds and joint angles are in radians. `path_progress` is the 0–1 progress of **this reviewed path itself**, not the original G-code line number.
3. The software combines consecutive motion/process events into synchronization blocks. Each block has at least `start_progress`, `end_progress`, `klipper_units`, and `duration_s`. Events with zero UR displacement that require waiting or extrusion become explicit stationary segments. Ordinary block boundaries are timing markers; they do not issue a separate UR motion command for each block.
4. For each segment, choose a minimum duration that satisfies the MoveIt source-trajectory duration, nominal process duration, and native extrusion velocity, acceleration, and single-command distance limits, then generate a continuous UR timeline. Every extrusion-only E command is calculated from rest to rest, including each acceleration and deceleration when long moves are split. Smooth-curve certification may lengthen the duration further. **The certified execution package defines the final timing**, which can exceed the original G-code estimate.
5. Once the smooth curve is fixed, bind each synchronization block to its final `start_time_s`, `end_time_s`, and `duration_s`. The software sends the entire UR curve as one ROS `FollowJointTrajectory` goal; it does not generate and run a new URScript for each model.
6. On the extrusion side, send each block's `klipper_units` and **final** `duration_s` to the bridge as parallel arrays. The bridge generates a Klipper virtual-SD `.gcode` file from them rather than directly executing the slicer's original file. The current executor controls only the UR/native-extruder motion channels; it does not forward heating, fan, homing, or other preparation commands from the original file.

The current parser includes original `G4` waits in the import page's total
estimate, but **not every original `G4` becomes a separate event on the final
execution timeline**. Do not assume that every millisecond of waiting shown
on import appears in the execution package. A future paired-file interface
must explicitly represent every interval that waits without extrusion.

### How the final Klipper file expresses timing

It does not write an absolute timestamp on every line. Motion speed and wait
duration express time implicitly:

- Save G-code state at the start, select the first extruder with `ACTIVATE_EXTRUDER EXTRUDER=extruder`, and use `G21` and `M83`. The default object is `[extruder]`, whose heater, sensor, and motor belong to the same extruder; the lock motor remains separate.
- For nonzero extrusion `ΔK` with allocated duration `Δt`, generate relative `G1 E<signed_increment> F<speed×60>`. `E` is in filament mm and `F` is in mm/min. Motions exceeding the native single-command distance protection are split into equal submoves. Their allocated timing includes each submove's acceleration and deceleration.
- For zero extrusion, generate `G4 P<wait_milliseconds>`. UR may simultaneously complete the segment's travel motion without extrusion. Milliseconds are rounded using cumulative wait time to reduce accumulated error in long files.
- Add one software-coordinated startup wait. At the end, execute one `M400`, restore G-code state, and disable only the selected extrusion motor if requested. Do not insert `M400` between ordinary moves.

The currently installed extrusion-only limits are `V=5 mm/s`, `a=20 mm/s²`,
and a maximum single-command distance of 50 mm. Every native extrusion-only
`G1` starts from rest and stops at its end inside Klipper. For `d=|ΔE|`, the
minimum duration is `2√(d/a)` when `d≤V²/a`, otherwise `d/V+V/a`. To match an
allocated duration `T`, calculate the submove speed as
`v=2d/(T+√(T²−4d/a))`, then output `F=60v`. Using `d/T` omits acceleration and
deceleration, causing actual extrusion to finish later than the UR timeline.

Native cold-extrusion protection remains enabled. Connection checks can be
completed with a cold hotend, but actual E motion must pass live temperature
and `can_extrude` checks. The software does not heat automatically; prepare the
hotend separately for the material. Execution packages bind the
`klipper-native-extruder-v1` protocol and current extrusion parameters. After
changing the configuration, restart the bridge and prepare the execution
package again. Do not reuse old manual-stepper execution packages.

The extrusion G-code is therefore an **execution artifact compiled from the
final timeline**, rather than an authoritative timing source that can be
uploaded unchanged after arbitrary speed changes.

## Output contract for future path-planning algorithms

A future algorithm should produce UR data and matching extrusion data for the
**newly planned geometric path**, without depending on the old G-code's path
or E values:

| Data | Minimum required semantics |
| --- | --- |
| UR trajectory | Strictly increasing relative `time_s`, six joint angles in radians with explicit names and order, and monotonic `path_progress` for the new path itself. The first point should use `time_s=0`. |
| Extrusion events | Each interval's `start_time_s`, `end_time_s`, and signed `extrusion_delta_mm`. Positive means extrusion, negative means retraction, and zero means no extrusion/travel. All times are relative to the same origin. |
| Job metadata | A common job identifier, coordinate frame, nozzle/TCP and extrusion units, and calibration provenance. Do not pair E values from an old file with a new UR path. |

Define the common origin as the start of the formal printing trajectory after
the robot has reached and confirmed the **reviewed print start**. The approach
and move-to-start motion are outside that printing timeline. Extrusion events
should continuously cover the complete printing trajectory. Intervals without
extrusion must be explicit zero-increment segments rather than inferred by the
reader. Validate endpoints, durations, joint/motor speeds, retraction direction,
and complete coverage before execution.

For example, the semantics could be `[0.0, 0.5] s → +0.30 mm E`,
`[0.5, 1.0] s → 0`, and `[1.0, 1.5] s → −0.08 mm E`. Ordinary `G1 E...` lines
alone cannot express absolute segment start/end times shared with a UR
trajectory.

### Implemented paired-file offline import v1

The first file is UTF-8 `.csv` with exactly these columns:

```csv
time_s,path_progress,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint
0,0,0,0,0,0,0,0
0.5,0.5,0.01,0,0,0,0,0
1,1,0.02,0,0,0,0,0
```

Time starts at 0 seconds and increases strictly. Path progress is monotonic
nondecreasing from 0 to 1, and joint angles are in radians. The current importer
also rejects adjacent joint jumps exceeding 0.35 rad, but this is **not** a
complete proof of velocity, acceleration, collision safety, or reachability.

The second file is UTF-8 `.gcode` and accepts only this restricted format:

```gcode
G21
M83
G1 E0.30 ;@sync start=0 end=0.5
G4 P250 ;@sync start=0.5 end=0.75
G1 E-0.08 ;@sync start=0.75 end=1
```

`G21` specifies E millimeters and `M83` specifies relative E increments. The E
value in `G1 E...` is the signed increment for that interval. The P value in
`G4 P...` is milliseconds and must match the interval duration in the comment.
Every interval requires `;@sync start=<seconds> end=<seconds>`. The first starts
at 0, consecutive intervals are contiguous, and the final endpoint matches the
UR CSV endpoint. Comment lines are permitted, but heating, fan, homing, XYZ
motion, macros, and every other executable command are rejected. Each file is
limited to 20 MB.

The importer returns only an offline summary and bounded preview samples. It
does not save files or call ROS, UR, or Klipper. **Passing format validation
does not mean the input is executable**. Missing steps include obtaining
trustworthy nozzle geometry from the external six-joint path, complete
MoveIt/collision review, shared retiming of both channels, installed-parameter
binding, a traceable execution package, and physical-synchronization
certification. The UI does not enable Start Print for this mode.

A future execution workflow should parse both inputs into one internal
timeline, validate and retime them together as needed, then generate the UR
goal and Klipper virtual-SD file. Do not import an already timed UR trajectory
while directly running external extrusion G-code without checking it: if UR
is retimed, the two channels can lose synchronization.

## Limits of synchronization

The current program provides only **software-coordinated startup**. The
Klipper program starts with a wait of approximately 4 seconds, and the executor
schedules the UR goal from the startup time reported by the bridge.
`physical_sync_verified=false` explicitly states that hardware clock
synchronization and actual nozzle material-flow timing have not been certified.
A shared timeline aligns nominal segment durations. UR can remain continuous
across ordinary block boundaries, while native extrusion-only E decelerates
to rest between `G1` commands. Instantaneous material flow within a segment
has not been physically certified against UR path progress. Measurement and
certification are still required before actual material printing.

## Source locations

- `Software/gcode_parser.py`: original G-code motion, E increments, and nominal duration.
- `Software/paired_timeline_import.py`: format and shared-timeline checks for paired-file offline import v1; it does not generate execution packages.
- `Software/gcode_path_review.py`: mapping from E millimeters to calibrated Klipper units.
- `Software/print_execution.py`: UR trajectory CSV validation, synchronization blocks, shared retiming, final timing binding, and execution packages.
- `workspaces/ur_ros2_ws/scripts/gcode_print_executor.py`: one UR joint-trajectory goal, Klipper segment arrays, and coordinated startup.
- `workspaces/ur_ros2_ws/src/ur_klipper_interfaces/action/ManualStepperProgram.action`: the `segment_units[]` / `segment_durations_s[]` action interface.
- `workspaces/ur_ros2_ws/src/ur_klipper_bridge/ur_klipper_bridge/gcode.py`: generation of the final Klipper virtual-SD G-code.
- `workspaces/ur_ros2_ws/scripts/execution_scope.py`: the software-coordinated startup wait constant and execution-scope checks.
