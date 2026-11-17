# Research on synchronized UR3e and extrusion start/stop

Date: 2026-09-01

## Findings

In robotic additive manufacturing, reliable synchronization usually requires
one of the following architectures, rather than merely making two independent
controllers receive a start command over a network at the same time:

1. **One motion controller interpolates the robot and extrusion axis together.** This is the strongest approach. The robot trajectory and extrusion axis share a timebase, velocity profile, and stop state.
2. **The robot controller acts as the process master.** Local digital signals control extrusion start/stop, while analog signals, pulses, or a real-time fieldbus continuously adjust extrusion speed according to actual TCP velocity.
3. **Distributed clocks and time-triggered execution.** All drives receive time-stamped trajectories in advance, then execute against a shared hardware timebase such as EtherCAT Distributed Clocks.
4. **A common hardware trigger.** Two controllers with preloaded trajectories wait in an armed state for the same electrical edge. Their accuracy must be measured.
5. **Host-side network coordination.** Simultaneous commands over ROS, TCP/IP, HTTP, or ordinary DDS provide best-effort coincidence. Without synchronized endpoint clocks and timed execution, they cannot provide a hard guarantee.

In this project, `ROS 2 -> Moonraker HTTP -> Klipper G4 -> MANUAL_STEPPER`
and `ROS 2 -> UR trajectory controller` fall into category 5. This supports
functional validation but does not prove physical synchronization. For
production synchronization, the most direct candidate is to move the extrusion
motor to **UR MotionPlus Advanced + an EtherCAT CiA 402/CSP/DC drive**, allowing
the UR controller to generate robot and extrusion-axis setpoints together.
Klipper can retain temperature control, fans, and non-real-time auxiliary
functions. If the drive cannot be changed yet, use hardwired start/stop gating
and actual TCP-velocity feedforward from the robot controller, bypassing
Moonraker HTTP control during printing.

## Distinguish four meanings of synchronization

| Concept | Meaning | Can shared timestamps alone guarantee it? |
|---|---|---|
| Command synchronization | Both controllers receive the same target time or start command. | Yes. |
| Physical-motion synchronization | Actual UR joint and extrusion-motor motion starts/stops within a specified deviation. | No; endpoint timing and measurement are required. |
| Process synchronization | Actual nozzle volumetric flow matches actual TCP speed, layer height, and line width. | No; melt pressure and transport delay also matter. |
| Safety synchronization | One safety event puts both robot and extrusion drive into states established by risk assessment. | No; safety circuits, a safety PLC, STO, or equivalent are required. |

In FFF, process synchronization determines print quality, which does not
necessarily mean that the first step pulse on both motor axes occurs at the
same instant. Melt backpressure delays nozzle output relative to extrusion-motor
motion. Klipper's Pressure Advance is intended to reduce corner buildup and
oozing during travel. Production systems therefore often require prepressure,
early flow reduction, or retraction; physically simultaneous start/stop is not
necessarily the best process command. [Klipper Pressure Advance](https://www.klipper3d.org/Pressure_Advance.html).
For flow delay caused by backpressure in robotic printing, see
[Algorithms for Large Scale Additive Manufacturing in a Free-Flying Environment](https://doi.org/10.3389/frspt.2022.900549).

## Industry and research implementations

### 1. One interpolator: treating extrusion as a robot external axis

UR's MotionPlus is the closest available off-the-shelf option for this project.
It supports planning robot and external-axis motion together in URScript with
`movel_with_axis_group`, `movep_with_axis_group`, `movej_with_axis_group`, and
`servoj_with_axis_group`. The official documentation states that robot and
external-axis trajectory profiles are synchronized and reach their targets at
the same time. [MotionPlus Axis Group API](https://docs.universal-robots.com/motionplus/mp1.5/URScript/motionplus-urscript-intro.html)

MotionPlus's EtherCAT process sends external-axis positions to the drive every
2 ms, matching UR's 500 Hz control cycle. Compatible drives must support
EtherCAT, Distributed Clock, CiA 402, and CSP (Cyclic Synchronous Position)
mode. [UR MotionPlus Servo Setup](https://docs.universal-robots.com/motionplus/mp1.5/servos/servo.html)

MotionPlus also couples stop states: a robot protective stop or E-stop requests
an external-axis stop, and an external-axis drive fault stops the robot. The
speed slider scales external-axis speed as well to preserve synchronization.
[MotionPlus FAQ](https://docs.universal-robots.com/motionplus/mp1.5/faq.html)

MotionPlus 1.5 requires PolyScope 5.24 and the corresponding URCap. Only the
Advanced license supports coordinated/synchronized motion.
[MotionPlus 1.5 Release Notes](https://docs.universal-robots.com/motionplus/mp1.5/release-notes.html);
[MotionPlus License Packages](https://docs.universal-robots.com/motionplus/mp1.5/install.html)

EtherCAT Distributed Clocks aligns slave clocks and compensates propagation
delay. The EtherCAT Technology Group specifies network clock jitter
significantly below 1 μs. This describes bus-clock synchronization, rather than
motor mechanical-position error. Actual following error still depends on the
2 ms control cycle, drive tuning, encoder, and mechanics.
[EtherCAT Distributed Clocks](https://www.ethercat.org/en/technology.html)

**Inference for this project:** model the extrusion motor as a rotary external
axis that does not participate in frame tracking. Axis targets can represent
cumulative filament length or motor angle, while `movel_with_axis_group` makes
the extrusion profile follow each TCP segment. This is kinematically reasonable,
but official UR examples mainly cover positioners and linear axes. Before a
formal purchase, ask UR or the drive integrator to confirm continuous-rotation
range, position-reset strategy, numerical range during long prints, and
whether extrusion-axis applications are supported.

### 2. Robot as process master: digital start/stop and TCP-velocity feedforward

Many retrofitted robotic printing systems use this approach:

- The robot controller outputs digital start/stop signals at print/travel boundaries.
- It simultaneously outputs an analog signal or pulses proportional to actual TCP velocity.
- The local extrusion controller converts TCP speed, line width, layer height, and a flow correction factor into motor speed.
- The process controller compensates nozzle pressure and transport delay.

Badarinath and Prabhu's ABB robotic FFF testbed sends spatially varying process
parameters over TCP/IP and uses analog/digital I/O to match extrusion speed
with deposition speed in real time. The paper reports a requested/actual
volumetric-flow fit of `R²=0.9983` and specimen extrusion-width/height errors
below 5%. This shows that the architecture can produce good process results;
it is not a safety-rated or microsecond synchronization claim.
[Integration and evaluation of robotic fused filament fabrication system](https://doi.org/10.1016/j.addma.2021.101951)

Gregorio's study of FANUC with an independent Duet controller describes the
same pattern: digital/serial signals control start/stop, and an analog signal
corresponding to real-time TCP velocity controls extrusion speed. Its tests
also show the limitations of independent controllers: a 50 Hz analog-input
update on the Duet side can cause up to approximately 20 ms of delay. The
author explicitly identifies unavoidable imperfect synchronization in a
separate printhead controller and recommends an oscilloscope or acquisition
device to quantify latency.
[Developing a Custom Printhead Controller to Print Large-Scale Parts Using Industrial Motion Systems](https://publications.polymtl.ca/10364/1/2022_PaulGregorio.pdf)

More recent research into robotic screw extrusion likewise does not focus on
one simultaneous start. It obtains a real-time analog TCP-velocity output from
the robot controller and continuously regulates screw flow with velocity
feedforward, reducing overfill and under-extrusion during acceleration and
deceleration. [Velocity-dependent real-time screw extrusion control](https://doi.org/10.1080/17452759.2025.2551084)

This approach is stronger than HTTP/G-code block-start commands because the
fast loop runs between the robot controller and the local extrusion controller.
It still requires measurement and compensation of delay from I/O, controllers,
drives, and material rheology.

### 3. A central PLC and real-time fieldbus

Large additive-manufacturing systems often use a central PLC or industrial
motion controller to manage robots, pumps, extruders, and peripherals. EtherCAT
distributed clocks let all DC-capable servo drives execute setpoints on a
shared timeline. A recent robotic additive-manufacturing research project
explicitly targets synchronous robot/extruder control through a central PLC
and EtherCAT.
[IKV Aachen EtherCAT-based controller project](https://ikv-aachen.de/en/student-research-theses/ethercat-based-controller-for-synchronous-robot-and-extruder-control-in-additive-manufacturing/)

This requires a real-time coordination interface in the robot controller, or
both robot and extruder axes controllable by the same real-time master.
Running ROS 2 `controller_manager` on the host can put software update calls
in the same cycle, but does not automatically align the UR controller clock
with the Klipper MCU clock or remove TCP/HTTP queuing. Official ROS 2 Control
documentation emphasizes minimizing real-time-loop jitter and detecting
missed cycles. It is a scheduling framework, rather than a cross-device
hardware-clock guarantee.
[ros2_control Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)

### 4. A common hardware trigger

When both controllers must be retained, a common approach is:

1. Fully preload both trajectories and put both controllers into an armed state.
2. Obtain ready confirmation from both.
3. Send one hardware pulse to both controllers.
4. Have the endpoints respond to that edge in local hardware/real-time tasks rather than returning the edge to the host for processing.
5. Measure deviation between the trigger, STEP/DIR, and actual robot-motion feedback with a logic analyzer.

This is more reliable than simultaneously sending two network requests, but
accuracy depends on each endpoint's fixed delay from input sampling to
execution. URScript scheduling uses 2 ms frames, so ordinary URScript/digital
I/O methods should initially be evaluated at millisecond scale. Microsecond
claims require measurement.
[URScript Thread Scheduling](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-scriptmanual/G5/ThreadScheduling.htm)

Standard Klipper `gcode_button`, Moonraker, and G-code input still pass through
the Klippy host and are not MCU-local hardware triggers. Retaining Klipper
stepper control in this architecture would require a custom Klipper MCU module:
preload the step queue, then let an MCU input edge or synchronized MCU clock
release it directly.

### 5. Software timestamps and ordinary networks

UR RTDE reaches up to 500 Hz on e-Series, but the official documentation states
that the controller can skip output packets when resources are insufficient,
retaining only the latest data. RTDE is suitable for real-time data exchange
and status/process feedback loops. It does not give an external PC a hard
guarantee of physical start timing.
[UR RTDE Guide](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html)

Klipper queues step pulses ahead of time against its own `print_time` and MCU
clock. Its official code overview states that step commands normally enter the
MCU queue at least approximately 100 ms in advance. Klipper synchronizes clocks
among its own one or more MCUs, but ROS time is not Klipper print time.
[Klipper Code Overview](https://www.klipper3d.org/Code_Overview.html)

A software-timestamp approach must therefore satisfy all of these conditions:

- Host clocks are synchronized, for example through PTP.
- The UR endpoint supports execution at a future time.
- The Klipper endpoint maps the same absolute time to print time/MCU clock.
- Both trajectories are preloaded before the deadline.
- Endpoint feedback reports actual/planned motion start, rather than only acceptance of an HTTP request.

The current project meets only part of the first condition: UR receives a
future trajectory header, while the Klipper bridge converts remaining ROS
time to `G4` and then sends it over HTTP. HTTP and Klipper queue delays occur
before `G4`, causing actual Klipper startup to lag behind the shared target
time.

## Design startup, normal stop, and safety stop separately

### Normal startup

A recommended sequence is: preheat and confirm process readiness → preload/arm
both sides → build extrusion pressure or run calibrated priming → bring the
robot to deposition speed → follow actual TCP velocity with material flow.
Whether extrusion should start earlier depends on the material and hotend
pressure model, rather than a fixed requirement for simultaneous startup.

### Normal stop and travel

Gradually reduce flow along the TCP deceleration profile and apply pressure
advance, coast, or retract so nozzle flow is near zero as the robot enters a
travel move. Simply setting UR and the stepper motor to zero at the same time
often leaves residual nozzle pressure that continues material flow.

### Protective stop and emergency stop

Software cancellation and Klipper `M112` are operational protections only.
Klipper API `emergency_stop` puts Klipper into a shutdown state but still
depends on the host and communication path; it is not a safety-rated hardware
circuit. [Klipper API Server](https://github.com/Klipper3d/klipper/blob/master/docs/API_Server.md)

Risk assessment must determine the safety architecture of production equipment.
Typical measures include:

- A dual-channel safety relay or safety PLC connected to UR External Emergency Stop/Safeguard Stop.
- The same safety chain controlling extrusion-drive STO or a safety contactor.
- Avoiding reliance on an ordinary MCU enable pin or Moonraker request as the sole safety-stop mechanism.
- Independent safe states for heater, pump, and residual-pressure risks.

UR3e safety I/O uses paired redundant Category 3 / PLd interfaces. Emergency
Stop is Stop Category 1, while Safeguard Stop is Stop Category 2. UR also
requires emergency stops to be reserved for emergencies; ordinary production
stops should use separate operational-stop logic.
[UR3e Safety I/O](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR3e/H_g5_sections/firstuse/Safety_IO_g5_en.htm);
[Emergency Stop vs Safeguard Stop](https://www.universal-robots.com/articles/ur/application-installation/connecting-an-emergency-stop-device-vs-a-safeguard-protective-device/)

## Options for the current project

### Option A: MotionPlus Advanced and an EtherCAT extrusion axis (recommended; hardware changes)

1. Confirm that the UR3e's PolyScope can be upgraded to 5.24 or later and obtain a MotionPlus Advanced license.
2. Replace the current open-loop stepper drive with a compatible EtherCAT CiA 402/CSP/DC closed-loop drive and encoder motor, or choose a closed-loop stepper/servo solution tested by UR.
3. Define the extrusion axis as a MotionPlus rotary axis and calibrate feed constant, gear ratio, and cumulative extrusion units.
4. Convert G-code segments to shared TCP/extrusion-axis targets for `movel_with_axis_group`/`servoj_with_axis_group`.
5. Retain Klipper for temperature, fans, and non-real-time workflows, removing its responsibility for the printing E-axis step queue.
6. Connect drive E-stop/STO to the shared UR safety circuit established by risk assessment.
7. Include pressure advance/material-flow dynamic compensation in material tests.

This is not a plug-in replacement for the current ROS 2 driver. The current
project controls the UR's six joints through `ur_robot_driver` and
`FollowJointTrajectory`, whereas MotionPlus exposes synchronized robot/external-axis
motion through the URScript Axis Group API. The official MotionPlus
architecture specifies that the UR controller generates external-axis targets
and sends them to EtherCAT through an internal real-time channel of the
MotionPlus URCap. The EtherCAT URScript API also cannot be called directly from
the secondary interface. The execution backend would therefore need to run
MotionPlus URScript/URCap on the robot, or use a UR-validated driver extension.
Simply adding a seventh axis to the existing ROS trajectory is insufficient.
[MotionPlus architecture and limitations](https://docs.universal-robots.com/motionplus/mp1.5/intro.html);
[Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

MotionPlus EtherCAT supports only a direct connection from the control box's
main Ethernet port to the first drive, without an ordinary switch. ROS/TCP
communication needs the officially supported additional network-interface
arrangement. [UR MotionPlus Servo Setup](https://docs.universal-robots.com/motionplus/mp1.5/servos/servo.html)

### Option B: Retain the motor and add a hardwired, robot-master process interface (moderate changes)

1. Keep Klipper temperature control, but stop using Moonraker to initiate extrusion block by block during printing.
2. Add a local real-time extrusion controller that receives a `RUN_GATE` digital signal and `FLOW_REF` analog/pulse signal from the UR control box.
3. Generate a flow reference inside the UR controller from actual TCP velocity, combined with each segment's `E / path_length` correction factor.
4. Compensate fixed I/O delay, stepper-drive response, and hotend pressure dynamics in the extrusion controller.
5. Use a hardwired stop gate to stop operational flow. Keep safety stopping in an independent safety circuit/STO path.

This route can reuse most existing hotend and Klipper configuration. Research
also demonstrates good printing quality with it. Synchronization error must
still be quantified experimentally rather than labeled a hard guarantee.

### Option C: Continue Moonraker/ROS network coordination (experimental use only)

If hardware changes are not feasible in the short term, improvements include
measuring Moonraker round-trip time, increasing preload windows, implementing
absolute `print_time` scheduling inside Klipper, recording actual UR/Klipper
startup feedback, and compensating latency. As long as motion reaches the
Klipper queue through HTTP/G-code, however, no strict bound can be provided
under network blocking or abnormal host scheduling.

## Required hardware acceptance

Express any guarantee as a numerical requirement first. For example, if the
permitted deposition-start spatial error is `Δx` and print speed is `v_tcp`,
the limit on a pure timing deviation is:

```text
Δt_max = Δx / v_tcp
```

Recommended acceptance method:

1. Use one logic analyzer/DAQ to acquire UR trigger output, extrusion STEP/DIR, drive ready/STO, and an independent TCP-motion sensor.
2. Use RTDE `actual_qd`/TCP speed as supporting data, without treating network arrival time as the sole physical time reference.
3. Repeat at least 1000 times. Report the median, P99, maximum startup deviation, and maximum stop deviation rather than a single successful trial.
4. Inject Moonraker delays, network disconnections, Klipper host load, UR speed scaling, protective stops, and drive faults.
5. Measure cold motor synchronization separately from actual hot-nozzle material output. A high-speed camera, weighing/flow sensor, or extrusion pressure/force sensor can validate process lag.
6. For normal stops, measure trailing material, buildup, and retraction. For safety stops, measure whether a safe state is still reached under a single fault.

Until these acceptance tests are complete, use descriptions such as
"software-coordinated startup" or "measured to a specified P99 deviation,"
rather than claiming guaranteed physically simultaneous start/stop.

## Hardware information still needed

Before selecting an option, confirm:

- The exact current UR3e PolyScope version and MotionPlus license availability.
- Extrusion motor, drive, encoder, gear ratio, and motor power.
- The current Klipper MCU model, STEP/DIR voltage, and whether additional real-time/safety interfaces can be installed.
- Whether the hotend uses filament extrusion or screw/pellet extrusion, and the acceptable process lag.
- Target print speed, nozzle diameter, and acceptable deposition-start spatial error.
- The existing emergency stop, safety relays, drive STO, and heater safety circuit.
