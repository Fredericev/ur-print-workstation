# Connect to a real UR3e

This project now retains only the real-robot connection workflow. Use `docker-compose.real.yml` and `scripts/launch_real_robot.sh` / `scripts/launch_real_robot_stable.sh` to connect to the real robot.

## 0. Safety

- Keep the emergency-stop button within reach.
- Lower the speed slider on the teach pendant for the first test.
- Make sure the robot's surroundings are clear of people, fixtures, cables, and unsecured objects.
- Connect and inspect joint states first. Execute MoveIt trajectories only after those checks pass.

## 1. Wiring and IP addresses

Connect the computer directly to the UR control cabinet with an Ethernet cable.

A common configuration is:

- Mac wired adapter: `192.168.12.1`
- UR3e control cabinet: `192.168.12.101`
- Subnet mask: `255.255.255.0`

You can also use the laboratory's existing subnet. The computer must be able to ping the robot, and the robot must be able to reach the computer's External Control ports.

The macOS settings path is usually:

`System Settings -> Network -> USB/LAN Ethernet -> Details -> TCP/IP -> Configure IPv4: Manually`

After configuring the address, test it in a Mac terminal:

```bash
ping 192.168.12.101
```

You can also use the project script to configure the wired adapter:

```bash
cd /path/to/extracted-project
MAC_IP=192.168.12.1 ROBOT_IP=192.168.12.101 ./scripts/setup_real_robot_network.sh
```

Check the current connection status:

```bash
ROBOT_IP=192.168.12.101 ./scripts/real_robot_status.sh
```

For more complete diagnostics:

```bash
./scripts/diagnose_real_robot_connection.sh
```

Check only Remote Control / Dashboard Server. By default, this does not probe
`30001-30004`, avoiding the Primary/RTDE interfaces when the robot's control
services are unstable:

```bash
ROBOT_IP=192.168.0.104 ./scripts/check_ur_remote.sh
```

To explicitly check the Primary/Secondary/RTDE ports:

```bash
CHECK_CONTROL_PORTS=true ROBOT_IP=192.168.0.104 ./scripts/check_ur_remote.sh
```

Send a single Dashboard command:

```bash
ROBOT_IP=192.168.0.104 ./scripts/ur_dashboard.sh "is in remote control"
```

## 2. Robot-side configuration

On the UR teach pendant:

1. Confirm that the robot is powered on and its brakes are released.
2. Confirm that the External Control URCap is installed.
3. Open `Installation -> External Control`.
4. Set Host IP to the Mac's wired adapter address, for example `192.168.12.1`.
5. Keep the default port, `50002`.
6. Create a Program and insert an `External Control` node.
7. Save the installation and program.

## 3. Start the ROS driver from the computer

Run this from the project directory:

```bash
cd /path/to/extracted-project
ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh
```

This project defaults to a `125 Hz` UR driver control loop. Running the ROS
package's default `500 Hz` directly over the macOS + Docker connection is more
likely to produce RTDE jitter and brief disconnections. To test the default
rate again, specify it explicitly:

```bash
UR_UPDATE_RATE=500 ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot.sh
```

If the Mac can already ping the robot, you can omit `ROS_PC_IP`; the script
will try to infer it:

```bash
ROBOT_IP=192.168.12.101 ./scripts/launch_real_robot.sh
```

After startup, open the real-robot desktop, which defaults to port `6082`:

```text
http://localhost:6082/
```

Then open the program containing the `External Control` node on the teach
pendant and press Play.

The control channel is connected only when the ROS terminal shows a message
such as:

```text
Robot connected to reverse interface
```

## 4. Send the first control goal from the computer

After confirming that the pendant's `External Control` program is running,
first send a trajectory goal that holds the current joint angles without
moving. The default hold time is `30 s`:

```bash
cd /path/to/extracted-project
./scripts/ur3e_hold_current.sh
```

This message confirms that the computer can control the UR3e through the ROS
action:

```text
Trajectory goal completed successfully.
```

Next, perform a very small joint test. By default, only `wrist_3_joint` moves,
with an offset of `0.03 rad` over `4 s`, followed by another `30 s` hold. Before
running it, clear the workspace, lower the speed slider, and keep the emergency
stop within reach:

```bash
CONFIRM_REAL_ROBOT_MOVE=YES JOINT=wrist_3_joint OFFSET_RAD=0.03 DURATION=4.0 HOLD_AFTER=30.0 ./scripts/ur3e_nudge_joint.sh
```

`ur3e_nudge_joint.sh` deliberately requires `CONFIRM_REAL_ROBOT_MOVE=YES` to
prevent unexpected robot movement if the script is run accidentally.

For a larger one-minute test, use `ur3e_long_sweep.sh`. By default, it moves the
more visible `wrist_2_joint` through 2 back-and-forth cycles of `+/-0.35 rad`
over `60 s`, then holds for `10 s`:

```bash
CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES ./scripts/ur3e_long_sweep.sh
```

To preview without sending the motion to the robot:

```bash
CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES DRY_RUN=true ./scripts/ur3e_long_sweep.sh
```

To customize it:

```bash
CONFIRM_REAL_ROBOT_MOVE=YES CONFIRM_LARGE_SWEEP=YES \
JOINT=wrist_2_joint AMPLITUDE_RAD=0.35 DURATION=60 CYCLES=2 \
./scripts/ur3e_long_sweep.sh
```

Open RViz to observe the real robot's state:

```bash
./scripts/start_rviz.sh
```

Then open this address in a browser:

```text
http://localhost:6082/vnc.html
```

The default password is `ubuntu`; override it with `ROS_DESKTOP_PASSWORD`
before startup. Port 6082 listens only on localhost. When macOS uses Colima,
UR ports 50001–50004 are forwarded from the Linux VM's IPv4 wildcard address
to the Mac. The robot should still connect to the wired adapter selected by
`ROS_PC_IP`, so connect the computer only to trusted laboratory networks.
RTDE uses nonblocking reads by default to prevent occasional virtualization
scheduling delays from causing dropped-frame crashes in the control node.
RViz receives the robot model state from `/joint_states` and `/tf`, so motions
sent by commands also appear in RViz.

To use robot Remote Control / driver headless mode:

```bash
ROBOT_IP=192.168.12.101 ROS_PC_IP=192.168.12.1 ./scripts/launch_real_robot_remote.sh
```

This mode requires the robot to be in Remote Control, with services including
Dashboard Server, Primary Client Interface, and RTDE enabled. It does not
require pressing Play on the External Control program on the pendant, but its
safety and network configuration requirements are stricter.

## 5. Extract the real robot's calibration

This step is not strictly required to make the robot move, but it affects
end-effector pose accuracy in ROS / MoveIt.

After powering on the robot, run:

```bash
cd /path/to/extracted-project
ROBOT_IP=192.168.12.101 ./scripts/extract_real_robot_calibration.sh
```

Then supply the calibration file when starting the real robot:

```bash
KINEMATICS_PARAMS_FILE=/home/ubuntu/ur_ros2_ws/src/ur3e_print_planner/config/ur3e_real_calibration.yaml \
ROBOT_IP=192.168.12.101 \
ROS_PC_IP=192.168.12.1 \
./scripts/launch_real_robot.sh
```

## 6. Troubleshooting

- `ping` fails: fix the network before starting the driver.
- The teach pendant cannot connect to the remote PC: the computer IP in External Control is incorrect, the computer-side driver has not started, or port `50002` is not forwarded to the container.
- ROS reports `Controller is not running`: Play has not been pressed on the pendant's External Control program.
- ROS reports `Connection to reverse interface dropped`: the robot program was stopped, a protective stop or emergency stop occurred, or motion was commanded from the pendant. Press Play again after recovery.
- External Control disconnects shortly after connecting: first use `launch_real_robot_stable.sh` or confirm `Update rate: 125 Hz`. Then check for a second RTDE client occupying the robot and confirm that the robot and Mac share the same wired subnet.
