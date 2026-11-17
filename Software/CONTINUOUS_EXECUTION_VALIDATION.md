# Continuous full cold-test validation — 2026-09-17

Result: offline implementation/build/regression passed. No new UR or Klipper
motion command was sent during this work. This is not hardware certification.

## Current v6 speed and curve regression

The final policy is `bounded-quintic-v6-source-timed`: 0.2 rad/s velocity,
2.5 rad/s² acceleration, finite 100 rad/s³ interpolation jerk, a 0.0012 rad
same-progress corridor, and unchanged 0.002 rad collision-sample spacing.
Source/feed/stepper timings remain lower bounds. A fine logarithmic timing-ratio
search fixes the narrow feasible-window failure. A wrong-direction minimum-jerk
tangent is projected toward a feasible source secant rather than clamped to a
manufactured zero-velocity process boundary. Endpoints and real holds still stop.

Fresh offline preparation through the public UMS5 preparation seam produced:

| Check | Current result |
| --- | --- |
| Preparation | 16.48 s on this computer |
| Cached approach | 9.983196 s; actual approach requires fresh joints |
| Full cold model timeline | 1,328.669851 s (22 min 09 s) |
| Source/feed/cold-stepper lower bound | 1,065.300516 s (17 min 45 s) |
| Frozen UR curve | 14,204 points; one print goal |
| Timeline / ordinary moving boundaries | 3,800 / 3,475; zero artificial stops |
| Certified full-curve velocity / acceleration / jerk | 0.197017 / 2.480034 / 99.994148 |
| Final independent MoveIt collision/FK gate | 66,599 candidate/reference states passed |
| Maximum tool0 translation / rotation deviation | 0.394258 mm / 0.001171 rad |
| Klipper program | 323,690 bytes; one start; one terminal M400 |
| Nominal encoded motor/UR duration difference | −0.000226 s |

The collision/FK check used the isolated ROS domain 89 and the unchanged fixed
table/whole-robot geometry, with limits of 0.5 mm and 0.005 rad. These are finite
sample model results, not continuous-space or hardware guarantees. The cached
historical v2 MoveIt source is used only as an offline fixture, not promoted to
a current planning approval. `ums5_final_v6_display_only.json` retains that
historical planning policy and is marked simulation/diagnostic-only; playback
checksum validation passed for the exact new frozen curve. Replan from fresh
robot joints and prepare a new package before a real test.

Reproduce the final offline command audit and display-only checksum check:

```bash
python3 Software/tools/audit_continuous_execution.py --prepared workspaces/ur_ros2_ws/runtime/diagnostics/ums5_final_v6.json
python3 workspaces/ur_ros2_ws/scripts/rviz_offline_print_playback.py --manifest workspaces/ur_ros2_ws/runtime/diagnostics/ums5_final_v6_display_only.json --speed-factor 100 --check-only
```

The check-only command does not start RViz or publish ROS messages; the actual
graphical viewer was not launched during this final offline verification.

Final regressions: 403 workstation Python tests, 79 ROS tests (77 bridge + two
planner suites), three selected ROS packages built, four static UI tests,
hardware-isolated Motion Planning/3D browser smoke, and the full cold DEMO
Prepare → Move to start → Start flow passed. No actuator was controlled.

Status polling reuses an immutable, file-identity-bound mathematical package
certificate; dispatch still performs fresh checks. Repeated math checks and
per-sample ROS service round trips are eliminated, not the final live scene
gate. Old smooth-policy execution packages are rejected and must be prepared
again. Already running backend/bridge processes were not restarted here.

The evidence below records the earlier continuous-v3 commissioning baseline,
not the current v6 motion duration or limit set.

## Cause and correction

The previous executor dispatched independent rest-to-rest UR/Klipper actions
for process blocks, waited for their terminal results, and repeatedly inserted
a 4-second start lead. V3 compiles one whole-job UR spline and one virtual-SD
stepper program. Process blocks are now telemetry/timing markers, not separate
starts. Explicit source travels and process-only retractions are preserved.

Tests were added before their fixes for one-goal dispatch, ordinary-boundary
continuity, fast retraction timing, full-profile progress lookup, stale printer
state, cumulative dwell quantization, and aborted-action result handling.

## Historical continuous-v3 commissioning evidence

- Software Python regression: 378 tests passed.
- ROS bridge regression against the installed overlay: 75 tests passed.
- ROS build: all 5 selected workstation packages passed; patched controller
  library/overlay verified without instantiating a controller or ROS node.
- Bridge Flake8 (99-character project style), Python compile and JS syntax passed.
- Four static UI tests passed, including direct button intent and Stop guards.
- Fresh hardware-isolated DEMO browser: Prepare → Move to start → Full cold test
  completed; no console errors, typed confirmation or external browser requests.
- English/Chinese range selector at 360/640 px: no text/radio or option overlap.
- Native ROS Goal CDR encode/decode round-trip: 14,718-point UR goal
  (2,708,592 bytes) and 3,800-segment Klipper goal (60,877 bytes) matched exactly.
  No ROS node/client was created; the codec fixture is not collision approval.

The existing validated `UMS5_mold2.gcode` full-file fixture was rebuilt through
the public preparation path (source/calibration/review hashes unchanged):

| Check | Result |
| --- | --- |
| Complete timeline | 3,800 segments |
| Frozen UR trajectory | 14,718 points; one action |
| Ordinary adjacent moving boundaries | 3,475; zero artificial boundary stops |
| Klipper | One upload/start; 327,656-byte program; one terminal M400 |
| Intentional zero-extrusion intervals | 903, retained |
| Maximum logical stepper target | 8.18826 units (limit 200) |
| Maximum commanded segment speed | 4.999996 units/s (limit 5) |
| Repeated-start overhead | 11,588 s previously; 4 s now |
| Nominal encoded motor/UR duration difference | −0.000056 s |
| Final print duration | 18,919.124 s at the commissioning limits |
| Minimum including cached approach + one lead | 18,960.186 s, excluding preflight/settling |

The audit decodes the generated MOVE/SPEED/SET_POSITION commands and sums their
nominal durations and motor units. This tests the command encoding, not the
physical MCU/robot clocks. Segment-average extrusion is not validated
instantaneous nozzle-flow control.

Full-v3 offline RViz verification passed using the final frozen quintic curve
and time allocation, not the older raw CSV timing. It samples 103,967 print
display points at bounded joint spacing and is display-only.

The obsolete v3 generated packages were removed during project cleanup on
2026-09-18. These historical results remain documented above; use the current
v6 reproduction commands at the top of this document for offline verification.

## Required before using the changed runtime

Restart the workstation backend and ROS stack while both devices are safely
stopped, then replan from fresh robot joints and Prepare a current v6 package. Old
execution IDs cannot be reused. Existing live processes were not restarted
by this offline validation. Do not press Play while restarting services.

Pause/Resume are unsupported; Stop and stop-unconfirmed locking remain. The
final continuous curve must still pass the live read-only whole-robot MoveIt
gate before either channel is armed. Sample cap/spacing, robot v/a/jerk limits,
readiness, stationary feedback, scaling and endpoint checks remain enforced.

Moving manual-stepper segments use ACCEL=0 to avoid rest-to-rest traps; this can
command instantaneous motor-speed changes. It is for supervised unloaded cold
testing only. Software start coordination is best effort, not a common physical
clock. Material printing remains locked until separately validated physical
synchronization, installed TCP/tool and process control are available.
