# Klipper communication repair and online acceptance (2026-10-20)

## Current failure and actual repair

After the user reconnected Klipper, `/printer/info` at `192.168.0.168` returned ready from the Mac,
while connections to the same address from the Colima VM and ROS container were immediately
refused during TCP connect. Five consecutive container connections failed; captured TCP-header
records showed RST/ACK, with no HTTP query reaching the service. NAT can generate refusal packets
bearing the remote address, so that RST cannot be attributed directly to the Klipper host.

This was a reproduced virtual-network failure, rather than extruder-configuration parsing or
G-code syntax. The backend was still running `native-extruder-program-v5`; v6, previously completed
offline, had not yet been loaded. There were no active jobs, UR External Control was not running,
and Klipper was standby/cold with a target temperature of 0. After stopping this project's old
control processes, the existing Colima recovery workflow, guarded by an idle-workload check,
restarted the virtual network. The original container's `/printer/info` check passed immediately
after recovery. Existing configuration, Docker context, and failure evidence were preserved.
No robot motion, heating, or extrusion was started.

The differences between the Mac, VM, and container, together with the results before and after
recovery, locate this fault's layer. They do not prove that all intermittent timeouts on October 19
had the same cause; those earlier failures still lack complete packet captures from both endpoints.

## Additional software changes

v6 already used merged read-only queries, persistent HTTP connections, and one budgeted network
retry. This round reproduced another pre-start gap: a successful independent configuration query
did not establish that the persistent monitoring connection was still usable. The old logic first
checked that connection only after uploading and starting.

Each action now obtains fresh status through the same monitoring connection before sending any
G-code or upload/start request, rechecking temperature, overrides, limits, and print state.
An old disconnected connection is reconnected first. On failure, nothing is uploaded, started,
or fed, and no emergency stop is sent when no action has yet been dispatched. Runtime cancellation,
stop confirmation, and read-timeout budgets remain effective. New tests reproduced the old
behavior's failure before verifying the repair.

After network recovery, another new connection was captured retransmitting SYN repeatedly,
with no completed handshake recorded, corresponding to an idle-readiness query timeout.
Another new connection in the same period worked normally, and runtime monitoring over the reused
connection continued to succeed. At that time, each complete idle-status read opened three TCP
connections. Idle-readiness checks and runtime monitoring now share one persistent GET connection,
while still reading the latest complete configuration each time. Control writes and minimal
post-stop confirmation queries remain independent. Complete configuration responses are limited
to 1 MiB, and runtime-monitoring responses to 64 KiB. Failed connections are discarded, and the
existing bounded read-only policy decides whether to retry. The exact location of that connection
establishment anomaly remains undetermined; connection reuse must not be described as eliminating
underlying packet loss.

The passive raw-packet observer's kernel receive queue dropped packets, so “no SYN/ACK recorded”
alone does not prove that an actual connection failed. A later missing handshake record for port
44474 was ruled out as a fault by continuously successful application queries on the same port.
The report primarily uses application-query results for acceptance, retaining raw header records
with their observation limits explicitly identified.

An independent review reproduced another timeout gap with a local service: if response fragments
kept arriving, per-socket-operation timeouts restarted the clock. A 0.15-second read budget could
previously return only after 0.377 seconds. Connected observation requests now interrupt socket I/O
at an absolute deadline; the review observed `NETWORK_ERROR` returning after approximately
0.154 seconds. The deadline timer holds only this request's socket and is canceled and joined before
the reuse lock is released, preventing interference with the next request. The stop channel remains
independent. Underlying DNS/connect implementations and system scheduling still provide no hard
real-time guarantee.

All 221 bridge tests passed, including local HTTP fault injection, pre-dispatch checks on the same
channel, live complete-configuration changes, connection reuse, and an independent stop channel
while an observation query was blocked. The workstation's 627 Python tests and 27 Node UI tests
passed. The local ROS build succeeded, and the bridge service was reloaded.

## Online acceptance method and boundaries

Read-only acceptance directly called production `KlipperBridgeNode._read_motion_status` and the
real Moonraker client, using a lazy state object instead of a ROS node/action. It created and
submitted no motion goal. Dynamic safety state and `print_stats` were read once per second for
20 minutes and 1,200 queries, exceeding the previous failure-triggering duration of approximately
10 minutes. An independent container query opening a fresh TCP connection every 5 seconds served
as a control, alongside passive observation retaining only TCP-header metadata. Reconnection after
normal server closure of a persistent connection was counted as normal behavior, separately from
timeout retries.

Deployment of this repair reloaded only the Klipper bridge. During this period, the UI's Start ROS
separately started the UR driver. External Control was not running when acceptance began; the user
subsequently connected External Control. Before the final patch reload, another check confirmed no
execution job, no Klipper action, `print_stats=standby`, and target temperature 0.
Actual read-only parameters reported `native-extruder-program-v6`; the bridge was ready and Klipper's
cold-extrusion protection was effective. Passing software tests and read-only stability checks does
not establish verification of material flow, hot-load behavior, or physical UR/extrusion synchronization.

Acceptance data is stored in `workspaces/ur_ros2_ws/runtime/diagnostics/klipper_online_20261020/`.
Raw statistics for the long-duration and final-transport acceptance runs are stored in
`persistent_monitor_summary.json` and `final_transport_monitor_summary.json`, respectively.

The first round was interrupted after 288 successful queries by container shutdown caused by
workstation exit cleanup: exit 137, without OOM. It is counted neither as a Moonraker timeout nor
as completed continuous acceptance. Its records were retained, and independent background acceptance
was restarted from the beginning.

## Completed application-layer acceptance

| Check | Duration / samples | Failures / timeout retries | Median / P99 / longest request |
| --- | --- | --- | --- |
| Continuous real GET through the production runtime-monitoring core | 1,200 seconds / 1,200 queries | 0 / 0 | 0.148 / 0.273 / 0.411 seconds |
| Final transport version with the absolute deadline | 300 seconds / 300 queries | 0 / 0 | 0.148 / 0.273 / 0.570 seconds |
| Independent control opening fresh TCP every 5 seconds | Approximately 10 minutes / 120 queries | 0 / retries disabled | 0.152 / — / 0.281 seconds |

The first run completed at 16:41:36 UTC, and the second at 16:41:58 UTC.
The first started before the final deadline patch, so it is not described as “20 minutes of
continuous acceptance of the final transport version.” Online confirmation of the final transport
is recorded independently as the second run. Both actually called the production monitoring method
and real Moonraker, without submitting a motion action, heating, extrusion, emergency stop, or print
start. A total of 875 automated tests passed.

Passive observation completed 21 minutes. A final kernel-filtered observer then monitored only
the target TCP port 80 for 3 minutes. Its receive-drop count was 0, and all three captured handshakes
completed. This short record cannot be extrapolated to establish zero physical-link packet loss
over the entire earlier period. All auxiliary diagnostic processes have ended; existing production
services remain running. Packet captures, control queries, cleanup evidence, and diagnostic source
code have been collected in the same evidence directory.

The current repair status is “deployed; software regressions passed; read-only field communication
acceptance passed.” This round did not perform hot extrusion, short coordinated motion, or a full
print, so long-term reliability under those actual loads remains unconfirmed. External network,
host, and device faults can still occur. Genuine sustained communication loss must retain stop
protection, rather than presenting stale state as healthy.
