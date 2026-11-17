# New extruder geometry update — 2026-10-07

The workstation now uses `Extruder/NEW_ExtruderAssembly.STEP`. Display meshes,
TCP candidates and the tool0 collision envelope share
`workspaces/ur_ros2_ws/config/printhead_geometry.json`. The ROS planner generates
its geometry constants from that file; its build identity includes the file.

## Installation candidate

The supplied CAD has its outer flange face at CAD Y=+5 mm and centre
(0,+5,+5) mm. For the existing zero-degree flange clocking, CAD is rotated
-90 degrees about X and translated (0,-5,+5) mm relative to tool0.
The right nozzle candidate is (32.55,34.4,145.275) mm relative to tool0.
These are CAD candidates, not physical installation measurements.
Both nozzles are at the same height in the exported pose. Verify the actual
printing endpoint of the selected nozzle and the parked state of the other
nozzle. The motor bodies were intentionally omitted from the supplied CAD;
the exported visual mesh contains the geometry present in that file.

## Updating an existing workstation

The current Mac app reads this source directory, so reopening it loads the
updated backend and assets. Its default mode remains Offline. Portable copies
must be rebuilt from the updated source; previous release ZIPs are unchanged.
The bundled first-installation calibration is also updated to the new CAD
candidate, with all measurement confirmations false. It permits offline
preview while retaining the production calibration requirements.

A changed printhead identity invalidates production calibration and cached
path reviews/execution packages. Existing measurements are preserved in a
backup rather than being replaced with CAD values. On the Workcell page,
measure and save the tool0-to-nozzle TCP, verify the tool orientation, and
confirm the installed extrusion mapping. If the feed mechanism changed,
remeasure its ratio. Keep actual table
height and Print Z separate from the TCP calibration. A geometry hash does not
constitute physical measurement.

After installation calibration, rebuild the ROS overlay if prompted and
create a new MoveIt review and execution package for each print. Old packages
remain available for diagnosis but are rejected for execution. The desktop,
ROS planner and executor all require the same printhead geometry identity.

The update retains the existing one-selected-nozzle, one-E-channel workflow.
Automatic nozzle switching or independent simultaneous extrusion require
separately defined selection positions, per-nozzle calibration and firmware
routing; placing two nozzles in the CAD does not enable those operations.

The existing motor and Klipper electrical, heater and sensor configuration
were retained. The execution workflow
continues to leave heating and fan preparation to the existing setup.

## Verification

Completed on 2026-10-07:

- Python regression suite: 555 tests passed, including first-installation
  migration and stale-geometry rejection.
- JavaScript UI regression suite: 12 test files passed.
- Chrome/WebGL smoke test: imported the sample G-code, built its path,
  loaded the new extruder mesh, and exercised playback, scrub and restart;
  no browser errors were reported.
- ROS Humble build in an isolated ARM64 Linux environment:
  `ur_klipper_interfaces`, `ur3e_print_planner` (all executables), and
  `ur3e_printhead_description` built successfully. Both CTest targets passed.
- Full URDF expansion with the installed UR robot description passed.
- Mesh manifest, binary STL triangle count, copied asset hashes and all four
  visual clockings were verified.
- The installed Mac desktop app launched successfully in Offline mode. Its
  Workcell form displays the new CAD TCP candidate and the previous saved TCP
  separately, with physical measurement confirmations cleared.

The installed calibration backup is
`~/Library/Application Support/UR Print Workstation/gcode_calibration.before-printhead-20261007T235128414550Z.json`.
Existing table placement and measurement values were preserved. Old execution
packages must be regenerated after completing installation calibration.

The original default Colima virtual machine was initially stopped and failed
to reach guest SSH readiness during the startup check. Its existing disk was
preserved and the profile was returned to its original stopped state. ROS was
built and tested in a separate temporary profile; online startup through the
original virtual machine remains unverified.

No robot motion or Klipper actuation was commanded. Physical TCP, nozzle
printing/parked positions, extrusion and thermal behaviour still require
installation checks. Successful offline tests and a ROS build do not replace
those measurements.
