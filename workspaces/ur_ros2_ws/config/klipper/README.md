# First native extruder configuration

`print.cfg` is a native extruder configuration assembled from the complete
configuration supplied by the user on 2026-10-19. The nozzle diameter is
**0.2 mm**, the filament diameter is **1.75 mm**, and filament is fed by a
conventional motor. The user confirmed that native E extrusion worked after
copying the configuration. Regression checks for the software migration are
still performed offline; they do not establish real-hardware synchronization
of the complete UR and extrusion system.

## Configuration structure

- The first extruder uses Klipper's standard `[extruder]` name and contains the filament motor, heater, and temperature sensor.
- Driver settings such as UART and current are in `[tmc2209 extruder]`.
- This revision does not define a second extruder. A future second extruder should use `[extruder1]` and `[tmc2209 extruder1]`.
- `lock_motor` and both fans retain their original configuration.
- The original pins, current, microsteps, `rotation_distance: 10`, temperature range, power, and PID values are preserved. Extrusion-only velocity and acceleration are explicitly set to 5 mm/s and 20 mm/s².
- Klipper's default minimum extrusion temperature of 170°C and extrusion-only distance limit of 50 mm are retained. `min_temp: 0` is the sensor's lower temperature limit; it does not permit cold extrusion.

## Replacement scope

This is a complete configuration file and must not be included directly into
the original complete configuration. When replacing the corresponding
`printer.cfg` contents, remove these three old sections to prevent duplicate
pin assignments:

```ini
[manual_stepper stepper_x]
[tmc2209 manual_stepper stepper_x]
[heater_generic hotend]
```

Other includes, macros, and the `SAVE_CONFIG` section at the end of the file
were not present in the body supplied by the user. Before switching, check
whether they still reference `stepper_x` or `hotend`, or define the old PID
values again. For example, change `SET_HEATER_TEMPERATURE HEATER=hotend` to
`HEATER=extruder`. References in `verify_heater`, `heater_fan`, and related
status objects must also point to the new object.

## Native extrusion interface

The first extrusion motor belongs to `extruder` and uses native `G1 E...`
control. Its temperature-control target is also `extruder`. `lock_motor`
remains a separate manual stepper. The native E axis uses filament millimeters;
`G1` feedrate `F` is in mm/min, so 5 mm/s corresponds to `F300`. The original
installed value of `rotation_distance: 10` is retained.

After selecting `ACTIVATE_EXTRUDER EXTRUDER=extruder`, the software uses `G21`,
`M83`, and signed `G1 E... F...` commands for feeding and retraction. The old
`MANUAL_STEPPER STEPPER=stepper_x ...` command can no longer control this motor.
Temperature control can use `M104`, `M109`, or an explicit `HEATER=extruder`
target. The current software executor does not send heating commands; prepare
the hotend separately to the temperature required by the material. Connection
checks permit a cold hotend, while actual E motion still requires passing the
temperature and `can_extrude` checks.

Each native extrusion-only `G1` accelerates from rest and decelerates to rest.
The software calculates its duration from the configured 5 mm/s and 20 mm/s²
limits, splits motions that exceed the single-command distance limit, and
includes every acceleration/deceleration interval in the timeline shared by
UR and extrusion. It does not bypass the configuration by disabling cold
extrusion protection or increasing the 50 mm distance limit. Old manual-stepper
execution packages must be prepared again. These timing calculations do not
certify physical material-flow synchronization or deposited material flowrate.

## Validation status

Offline checks covered the complete INI structure, required fields, absence
of duplicate pin assignments, and consistency of retained sections with the
user's original configuration. The user reported successful configuration
loading and native filament-feed tests. Repository offline validation does not
include the target include files, temperature-control accuracy, feed
calibration, or real-hardware certification of UR/extrusion synchronization.

References:

- [Klipper extruder configuration](https://www.klipper3d.org/Config_Reference.html#extruder)
- [Klipper additional extruder naming](https://www.klipper3d.org/Config_Reference.html#extruder1)
- [Klipper G-code](https://www.klipper3d.org/G-Codes.html#g-code-commands)
