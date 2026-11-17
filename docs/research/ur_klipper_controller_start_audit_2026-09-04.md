# Installed controller start-time audit — 2026-09-04

Scope: read-only investigation of the installed UR/Klipper execution chain. This
note does not authorize motion, certify synchronization, or change production
code. It supplements the older September 1 research note: the earlier statement
that the selected UR controller honors a future trajectory header does **not**
apply to the passthrough controller actually selected by this workstation.

## Installed implementation and evidence

Read-only `/proc/202438/maps` showed the driver loading
`/opt/ros/humble/lib/libur_controllers.so`, while its UR hardware plugin and driver
executable came from `/home/ubuntu/ur_ros2_ws/install/ur_robot_driver`.
`dpkg-query` reported `ur_controllers 2.13.0` and
`joint_trajectory_controller 2.53.1`; `dpkg -V ros-humble-ur-controllers` reported no
changed package files. Therefore the working-tree controller source alone is not
proof of what is running: the corresponding installed release must be checked.

The installed passthrough release accepts and executes the goal without consuming
`trajectory.header.stamp`. Its `update()` ignores the supplied absolute time and
immediately begins transfer of point positions and relative `time_from_start`
values. The hardware interface converts these into segment durations and starts
execution after enough points are buffered, without a shared absolute start
field. [UR controller 2.13.0 source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.0/ur_controllers/src/passthrough_trajectory_controller.cpp),
[local hardware interface](../../workspaces/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/src/hardware_interface.cpp).

## Scaled JTC: better timestamp semantics, different operational tradeoffs

The installed JTC `Trajectory::update()` records `header.stamp`. Sampling uses
that timestamp plus each point's relative time. However, before the first point
is due, it interpolates from the current sampled state toward that first point;
a future header does not by itself prohibit early motion. An exactly stationary
prefix requires matching stationary start positions, or a separately reviewed
prelude. With positions only, JTC interpolates linearly in joint space; velocities
and accelerations change the interpolation order.
[JTC 2.53.1 trajectory source](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/src/trajectory.cpp).

Scaled JTC initializes its trajectory clock from ROS time and then advances it
by `period * scaling_factor`. Thus speed scaling below one affects not only path
duration but also progress toward a future timestamp after initialization.
Klipper's independent queue does not automatically acquire this same scaled
clock. Scaled JTC also returns interpolation and streaming to the ROS host,
unlike passthrough's robot-side execution; adopting it is a substantive backend
change, not a drop-in synchronization guarantee.
[Scaled JTC 2.13.0 source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.0/ur_controllers/src/scaled_joint_trajectory_controller.cpp),
[UR controller documentation](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html).

JTC cancellation queues a hold-position trajectory and marks the action canceled.
The software terminal action state does not prove the robot has physically
stopped; joint velocity freshness and an appropriate independent stop mechanism
remain necessary. It is not a safety-rated common stop circuit.
[JTC cancellation implementation](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp).

## Minimal cold-state test recommendation

Retain passthrough for the already exercised robot-side execution backend, but
remove the ineffective future header. For an explicitly experimental cold-state
Tool0 test, a visible stationary prefix can be assigned a relative duration
calculated from the shared intended host-monotonic start time immediately before
dispatch. This is **software delay coordination**, not a controller-level start
barrier: message acceptance, buffer transfer, execution scheduling, speed scaling
and the other endpoint's queue introduce residual timing error.

The test should reject an expired preparation budget before dispatch, track
pending as well as accepted goals, clean up late acceptances after timeout, and
never resend a goal merely because its acceptance response was late. Keep
production printing locked until endpoint synchronization and stopping behavior
have a separately demonstrated acceptance criterion. A wider lead window can
reduce avoidable setup timeouts; it does not remove end-to-end latency.

Prefer a fail-closed freshness/stability check of speed scaling over adding
untested generic scale compensation in this repair. The actual published topic
is `/speed_scaling_state_broadcaster/speed_scaling`, expressed in **percent**:
100.0 corresponds to factor 1.0. The hardware interface name ending in
`speed_scaling_factor` is not this topic. A stationary preflight reading cannot
prove the same scaling will persist during motion. Never bypass protective
scaling or silently increase the operator's slider to obtain synchronization.
[Broadcaster 2.13.0 source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.13.0/ur_controllers/src/speed_scaling_state_broadcaster.cpp).

For the minimal cold test, require fresh, stable 100% scaling without changing
the slider, and monitor changes or stale feedback to trigger the existing
two-sided operational stop. This still has finite detection/cancellation latency
and cannot make already queued extrusion instantaneously follow a protective UR
slowdown. General compensation at a stable factor below one should be a separate
measured experiment, not silently included in this repair.

## Read-only observed state

At 18:18:03 CDT, the ROS topics reported:

- UR program running, robot mode 7 (RUNNING), safety mode 1 (NORMAL).
- Scaled JTC active; ordinary JTC and passthrough inactive.
- All six sampled joint velocities zero.
- Joint positions in pan/lift/elbow/wrist1/wrist2/wrist3 order:
  `[0.52320385, -2.53316702, -2.23037767, 0.05279486, 1.56953919, 2.09847569]` rad.
- Subsequent speed-scaling observation was 100.0 percent.
- Klipper reachable but not ready: `printer_state=shutdown`,
  `action_active=false`, detail `Shutdown due to webhooks request`.

These are timestamped observations, not continuing safety assurances. No goal,
controller switch, restart, topic publication, or write service was sent during
this audit.
