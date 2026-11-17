# Moonraker runtime-status timeout hardening

## Confirmed observations

At 2026-10-19 12:41:34 (Chicago), execution package `4624be5dfeff0043` encountered a status-read
timeout at approximately 5.3%, segment 69, source line 356:

```
NETWORK_ERROR: /printer/objects/query?configfile&gcode_move&toolhead:
<urlopen error timed out>
```

Execution records confirm completion of UR cancellation, continuous stationary observation,
and the Klipper emergency stop. The error occurred during runtime monitoring, rather than
G-code syntax parsing. Moonraker logs from the same period showed approximately 2–8.5% process CPU
usage, stable memory, and a CPU temperature of approximately 44°C; there was no evidence supporting
Moonraker overload or overheating. The `urlopen error` wrapper more strongly supports an inference
of a timeout while connecting or sending the request. Without packet captures from the incident,
however, the evidence cannot distinguish a physical-link problem, virtual-network problem, or other
connection delay. A small number of later read-only queries from the host and ROS container
succeeded, which does not rule out an intermittent fault.

## Software weaknesses and repairs

The old monitor read print status every second and reread full status every two seconds: host
information, the complete configuration, and extruder status, requiring three HTTP round trips.
A single network failure during the complete-configuration query terminated the job.
In a single read-only measurement on the current device, the old configuration query returned
approximately 26.8 KB, compared with approximately 0.5 KB for the compact dynamic query.
This demonstrates a load difference, but does not establish that the old query caused the incident
and is not a sustained-load performance test.

The new implementation still reads the complete configuration before every action. During execution,
it sends one merged GET restricted to the necessary fields: `webhooks`, `gcode_move`, `toolhead`,
the native extruder, and the program's required `print_stats`. Temperature, extrusion permission,
overrides, Klipper state, and print identity all come from the same fresh response.
Static limits reuse only this action's preflight values and are read again before the next action;
stale health or completion state is never substituted for a query. Idle polling and manual refresh
cannot insert a complete-configuration read during execution.

Only `NETWORK_ERROR` from a read-only GET is retried, once. The first attempt normally receives
at most 6 seconds. After a 250 ms wait, the retry receives only the remaining budget, approximately
1.75 seconds. Lock waiting also counts against the budget. A configured timeout below 8 seconds
uses the smaller value; a value above 8 seconds does not expand the runtime-monitoring budget.
Responses arriving after the budget, continuous loss of communication, invalid responses,
low temperature, abnormal overrides, or incorrect file identity all prevent continuation.
Cancellation is checked both before retrying and during the wait. Control writes such as upload,
start, and feeding are never automatically replayed.

A failed read immediately publishes unavailable status, preserving the existing coordinated-stop
and post-stop confirmation mechanisms. Logs record failure category, query error, retry duration,
and budget to aid the next investigation. The read-only bridge execution version was updated to
`native-extruder-program-v5`; the new executor rejects older bridges. Native E commands and the
execution package's nominal timeline are unchanged. The native-extrusion protocol remains
`klipper-native-extruder-v1`.

## Validation scope

Regressions use isolated real action callbacks, clients, and controllable clocks. Coverage includes
recovery from a single timeout, continuous loss of communication, budget limits, late responses,
cancellation during a retry, temperature/override/status faults, stale completion, and wrong file
identity. Real Moonraker validation used GET only, without starting motion or heating, and did not
disconnect a running device to manufacture a fault.

Results for this round: all 181 bridge tests, 627 workstation Python tests, and 27 UI tests passed.
New tests first reproduced the old runtime monitor's failure after one network timeout and a
concurrency problem where a queued idle refresh crossed into execution; they then verified the
repairs. The ROS container's merged read-only query through the new client returned valid status
from real Moonraker, with a single measured request of approximately 0.255 seconds. This does not
replace real-hardware validation during a long material print. ROS compilation and existing driver
ABI checks passed. After confirming there were no active jobs, only the Klipper bridge was restarted;
its read-only parameter reported `native-extruder-program-v5`, and the readiness probe succeeded.

## Another timeout at 13:15 and the offline connection repair

Subsequent execution package `5bd891e3875b0880` failed at approximately 33.9%, segment 708.
v5 logs show that an earlier query recovered through a retry within 6.518 seconds; a later query
and its retry both timed out. Protective abort occurred after 8.006 seconds, and the emergency-stop
request succeeded approximately 70 ms later. Stops on both the UR and extrusion sides were confirmed.
Thus, v5's retry did work, but did not solve the underlying intermittent connection failure.

The last query before the failure reached Klippy at monotonic time 8148.709044; the emergency stop
arrived approximately 9.256 seconds later. During that interval, the MCU responded normally,
with `bytes_invalid=0`, `print_stall=0`, and approximately 1 ms MCU round-trip time.
Moonraker CPU usage was approximately 2.55–6.11%, with temperature approximately 44.77°C.
This evidence does not support a stalled extrusion command or MCU. Local fault injection and the
Python standard-library source confirmed that `<urlopen error timed out>` corresponds to connection
establishment or request sending; response-header or response-body timeouts have different exception
wrappers. The old client used `urlopen`, forcing TCP closure after each query. The container recorded
98 cumulative SYN retransmissions, but that count was not specific to the target IP, so it cannot
all be attributed to this incident.

The current route had no overlapping subnet or enabled proxy/VPN. After the failure, 120 read-only
queries each from the host and container succeeded. Those 240 samples did not reproduce the field's
intermittent fault and therefore do not establish that the external network was repaired.
The device later became unreachable. The user requested offline validation, and no further probe
or control requests were sent to the device.

v6 changes direct runtime monitoring to an independent persistent GET connection, reducing the
need to establish a new TCP connection every second. When the server closes the connection or a
connection error occurs, the old connection is discarded. Reconnection occurs only on the next
budgeted GET retry; the transport layer does not hide retries. Emergency-stop POST and stop-status
confirmation use neither the monitoring connection nor its lock. The existing configured-proxy path
and test opener are retained; HTTPS uses standard certificate verification. Exceptions additionally
record connect/send/headers/body stage, elapsed time, and whether the connection was reused.
Responses are limited to 64 KiB.

Offline tests use a local HTTP/1.1 simulation service to reproduce “the first connection succeeds,
but subsequent TCP connection establishment fails.” The old transport fails; v6 obtains successive
fresh states using the same connection. Integration tests combining the real bridge action callback
and real HTTP client also verify exactly one upload and one start after reconnection, and completion
of an independent emergency-stop request while GET is blocked. Invalid responses, HTTP errors,
oversized or incomplete responses, timeouts at each stage, server-initiated closure, cancellation,
and the existing budgets are all within isolated test coverage. Offline compilation starts no ROS
node and does not automatically resume a failed print.

The new source and build artifacts require `native-extruder-program-v6`. No new bridge is started
while the device is offline. After device connectivity returns, ROS must be restarted from the
workstation to load that version. Whether the physical link still has intermittent faults, and
reliability during long real prints, still require online acceptance; passing offline tests cannot
replace it.

v6 offline results: 198 bridge tests, including local HTTP connection-failure tests and real
action-callback integration tests; 627 workstation Python tests; and 27 UI tests, for 852 passed
tests in total. Compilation and the existing UR driver ABI check passed. No new ROS node was
started, and neither the robot nor Klipper was connected for validation. Existing processes must
be reloaded through the normal startup workflow after the device returns; a source update does
not establish online acceptance.

After the device returns, first confirm the new version, connection state, and sustained query
stability through read-only checks, then perform an operator-authorized short coordinated-motion
acceptance test. Until then, record the status as “offline software repair passed; field link and
real operation pending acceptance.”

Networks, the Klipper host, and devices can still fail. This repair reduces false stops caused by
transient read failures while preserving stop protection for genuine communication loss. It does
not promise error-free operation or a hard real-time upper bound on physical stopping.
HTTP timeouts apply to socket operations; emergency-stop requests and stop confirmations have
their own deadlines. The 8-second monitoring budget must not be interpreted as an absolute
guarantee from network disconnection to physical stop.

References: [Moonraker status-query field selection](https://moonraker.readthedocs.io/en/latest/external_api/printer/#query-printer-object-status),
[Klipper status objects](https://www.klipper3d.org/Status_Reference.html).

## Online follow-up on 2026-10-20

For the virtual-network failure after the device returned, actual v6 deployment, same-channel
pre-dispatch checks, and additional repairs and acceptance for sharing idle/runtime query
connections, see [Klipper communication repair and online acceptance](KLIPPER_ONLINE_ACCEPTANCE_2026-10-20.md).
The offline test counts and not-yet-deployed statements above describe the state on October 19.
