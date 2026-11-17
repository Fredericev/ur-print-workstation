# UR3e printhead RViz preview

This package is an offline ROS 2 / RViz placement check for the supplied
`NEW_ExtruderAssembly.STEP`. It rigidly attaches the tessellated printhead to
the UR3e `tool0` frame and lets the operator move all six robot joints with
`joint_state_publisher_gui`.

The launch contains only:

- `joint_state_publisher_gui`
- `robot_state_publisher`
- RViz

It does not start the UR driver, `ros2_control`, a controller manager, MoveIt
execution, URScript, or Klipper.  Use the repository-level launcher:

```bash
./scripts/start_printhead_rviz.sh --clocking 0
```

The initial offline joint pose aims the nozzle vertically downward. Other
flange clocking values (`90`, `180`, `270` degrees) are placement previews only;
the workstation's installed transform currently assumes clocking `0`.

The mesh URI, CAD mounting transform, nozzle candidate and full-head collision
envelope come from the shared `workspaces/ur_ros2_ws/config/printhead_geometry.json`.
CMake installs that profile into this package's `share/config` directory;
rebuild the package after changing the profile. The launch also accepts
`printhead_geometry_file` for a separate offline candidate.

At clocking 0, the candidate mounting face is CAD `Y=+5 mm`, with flange centre
`[0, 5, 5] mm`. The translation `[0, -5, 5] mm` and rotation `[-90, 0, 0] deg`
map that centre to `tool0`; each other clocking rotates the full mounting
transform around `tool0` and keeps the contact centre fixed.

The orange `nozzle_tcp` sphere is the right exported tip at CAD
`[32.55, -140.275, 39.4] mm`, producing candidate tool0 offset
`[32.55, 34.4, 145.275] mm`. A grey `left_nozzle_candidate` marks the other
exported tip at CAD `[-32.55, -140.275, 39.4] mm`. Both tips are at the same
height in the STEP export; this does not establish the mechanism's real
selected or retracted end positions. Neither marker is a physical TCP
measurement. The software retains the single active-nozzle workflow.

The simplified full-head box uses the same geometry profile as the
workstation's MoveIt print-path checks and guarded table-height trial. Confirm
the real mount, selected nozzle state and measured TCP before live motion.
The box describes the exported-pose envelope; it does not validate other
mechanism positions or authorize production extrusion.

`scripts/export_step_visual_mesh.py` preserves CAD axes and metre units. Its
output filenames follow the supplied STEP filename. It verifies the
tessellation's STEP hash and mesh bounds before exporting, and derives the
mounting-face candidate from the `base` mesh's maximum Y face. Use
`--mounting-plane-y-m` when a future design requires an explicit different face.
