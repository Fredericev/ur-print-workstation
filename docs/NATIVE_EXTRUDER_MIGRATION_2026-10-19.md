# Native extruder software migration

This change connects the operator-tested `[extruder]` configuration to the workstation,
ROS bridge, and print executor. The first channel is named `extruder`; a future second native
channel will be named `extruder1`, but this change does not create its configuration.
`lock_motor` remains an independent manual stepper motor.

## Commands and timing conversion

- Positive and negative E increments produce native feeding and retraction commands:
  `ACTIVATE_EXTRUDER`, `G21`, `M83`, and `G1 E… F…`.
- The installed feed ratio and direction are preserved. Native E uses filament millimeters;
  F uses mm/min.
- G-code state is saved and restored, and a final `M400` confirms completion. Only the selected
  extrusion motor is disabled; no global disable command that would release the locking motor is used.
- Acceleration and deceleration time for each extrusion-only move is calculated using limits of
  5 mm/s, 20 mm/s², and 50 mm per move. Splitting a long move also accounts for each rest-to-rest
  interval, and the UR trajectory is retimed accordingly.
- Feed velocity returns to zero between extrusion-only segments. Matching endpoint durations do not
  establish physical synchronization between instantaneous extrusion and UR motion within a segment.
  `physical_sync_verified=false` remains explicit.

## Compatibility and runtime checks

The known legacy setting `stepper_x` automatically migrates to `extruder`, preserving network
addresses and other operator settings. Old execution packages and old bridge versions cannot be used
for new execution: validation and preparation must be repeated under protocol
`klipper-native-extruder-v1`. Subsequent status-timeout hardening updated the bridge execution
protocol to `native-extruder-program-v5`; subsequent connection-reuse hardening updated it to
`native-extruder-program-v6`. The executor rejects an older bridge that is still running.
Historical ROS action names and some `manual_stepper` data keys remain for interface compatibility;
their controlled value is now the native extruder. Status still truthfully lists the locking motor
under `manual_steppers`, while `extruders` lists extrusion heads separately.

Connection, planning, and preview are allowed while cold. Before E motion is sent, the software
checks live `can_extrude`, temperature, nozzle and filament diameters, `rotation_distance`, motion
limits, and 100% speed/extrusion overrides. Execution is rejected if the configuration differs from
the execution package. The bridge continues to check native-extrusion conditions during execution;
if they are lost, it stops and retains recovery protection for uncertain motion.
The software does not automatically send heating commands; the hotend must be prepared separately
for the material.

## Validation

- Workstation Python regressions: 627 tests passed; 10 related tests passed again after a subsequent
  change to configuration-migration boundaries.
- Isolated ROS bridge tests: 96 passed, covering low temperature, missing targets, changed limits
  or overrides, cancellation, and stop confirmation.
- Node UI tests: 27 passed.
- ROS interfaces, bridge, and C++ coordinator compilation and installation passed, as did the
  existing UR driver ABI check.
- New native execution-package diagnostics: the independently integrated E acceleration/deceleration
  trajectory matched the nominal total UR duration; wait-rounding error was approximately 1 µs.
- Physical-device checks were read-only: `extruder`, a 0.2 mm nozzle, 1.75 mm filament,
  `rotation_distance=10`, limits of 5/20/50, and all overrides at 100%; cold-state
  `can_extrude=false` was displayed correctly.

Migration validation sent no robot-motion, heating, or extrusion commands. Material flow,
feed quantity, and physical UR/extrusion synchronization still require measurements on the real system.

Related implementations: `Software/print_execution.py`, `Software/control_station.py`,
`workspaces/ur_ros2_ws/scripts/gcode_print_executor.py`, and
`workspaces/ur_ros2_ws/src/ur_klipper_bridge/ur_klipper_bridge/`.
See the [shared timeline](ur_extrusion_shared_timeline.md) for more detailed timing conventions.
