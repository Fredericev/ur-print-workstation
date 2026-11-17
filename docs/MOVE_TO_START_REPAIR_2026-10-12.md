# Move to reviewed print start — 2026-10-12

The two observed failures occurred at different stages of the same approach trajectory.

1. At 14:29:36, job `3b5960adc35b` had sent the UR approach trajectory when the controller reported
   `Robot not within tolerances at end of trajectory.`
2. At 14:30:02, job `943830054ae3` reused execution package `aa995e057b290504` and reported,
   before sending any new motion, that the current joints differed from the original reviewed start by `1.0013 rad > 0.03 rad`.

The original approach moved wrist_3 from `0.597066402` to `1.598360925 rad`,
a displacement of `1.001294523 rad`. Read-only joint feedback confirmed that the robot was near
the reviewed endpoint: the maximum position error across all six joints was approximately
`0.00006644 rad`, with zero velocity. The user also confirmed that the first motion visibly reached
the print start. The second error resulted from retrying an old approach after the current position
had changed; no second trajectory was sent.

A defect was confirmed in the controller: after receiving hardware DONE for the first motion,
it immediately ABORTed if the current feedback exceeded a position, velocity, or acceleration
tolerance, without allowing the convergence wait specified by `goal_time_tolerance=2s`
in the request. The original time check ran only after the joint tolerances were already satisfied.
The original log did not identify the first joint or physical quantity outside tolerance,
so the evidence does not establish that residual velocity was the cause.

## Repair

- A positive goal time tolerance allows endpoint feedback to converge after DONE. The existing
  six-joint limits remain `0.005rad` position, `0.003rad/s` velocity, and `0.5rad/s²` acceleration.
- Time advances using the unscaled cycle during DONE. A separate limit bounds the total wait
  from DONE, preventing waits longer than 2 seconds when upload time was not counted as
  IN_MOTION or speed scaling is zero.
- Success is allowed exactly at the deadline; convergence after the deadline still fails.
  Zero and negative values retain the original single-check behavior.
- The old goal remains active while waiting. Cancellation, hardware abort, communication failure,
  and controller deactivation retain priority.
- Endpoint failures report the specific joint, position/velocity/acceleration quantity,
  measured value, target value, and tolerance.
- The independent endpoint and stationary checks after Python action success remain in place.
  A failed action is not retrospectively treated as successful.
- A failed or canceled move-to-start package requires a new MoveIt review. The same package
  cannot be retried, and the same review cannot be used to Prepare again. Both frontend and
  backend instruct the operator to review again from the current posture; old stop records remain.

## Validation and deployment

- New regressions using the real mock-controller/action reproduced the failure with the old library,
  then passed with the repaired library.
- Sixteen endpoint, deactivation, and communication-failure tests and one large-trajectory test passed.
- Isolated tests used a separate build directory, `--network none`, and a separate ROS domain;
  they did not access the physical arm.
- The complete Python regression suite passed 580 tests; JavaScript regressions passed 12 suites.
- The new controller was compiled and installed in the actual local ROS cache. Library loading
  and URCL/driver compatibility checks passed.
- The same 17 mock-controller tests passed again when explicitly linked to the installed library.
- Before deployment, there were no active workstation jobs, External Control was not running,
  and read-only feedback was continuously stationary for 0.5 seconds. The workstation was closed
  and reopened normally; the controller library was not replaced in a running process.
- The new backend's actual execution-package status API returned `requires_revalidation=true`.
- Restarting services exposed a transient network problem: the Mac could connect, but the Colima VM
  received connection refusals from both UR and Klipper. Restarting Colima and restoring the original
  `colima` Docker context restored both connections. The full startup script successfully launched
  the new driver and Klipper bridge; 125 Hz, the 45-second stability check, and ports 50001–50004
  passed. MoveIt startup and readiness checks also passed.
- The user subsequently confirmed the connection. Final read-only API checks showed External Control,
  the UR driver, MoveIt, and the Klipper bridge ready, with no RTDE conflict, active job, or
  unconfirmed-stop lock.
- No physical move, print, extrusion, heating, or teach-pendant Play command was sent. Calibration,
  TCP, table, Print Z, and connection-configuration file SHA256 values were unchanged across deployment.

Evidence is stored in `workspaces/ur_ros2_ws/runtime/diagnostics/move-start-20261012/`.
Build and test logs are `/tmp/ur-move-start-*`; the system may clean up these temporary logs.

The user has started External Control. The next step is to import the original G-code again and,
from the current posture, repeat **Validate with MoveIt → Prepare → Move to reviewed print start**.
Previously failed execution packages are not unlocked or marked as having reached the endpoint.
No physical motion was replayed on the user's behalf during this repair.
