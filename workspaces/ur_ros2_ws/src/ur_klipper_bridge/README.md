# Native extruder bridge

The bridge is the ROS owner of Klipper motion writes through Moonraker. The
first feed motor, heater, and temperature sensor now belong to `[extruder]`;
`[manual_stepper lock_motor]` remains a separate actuator. See the installed
reference configuration in `../../config/klipper/print.cfg`.

The configured extruder name defaults to `extruder`. Command generation selects
it with `ACTIVATE_EXTRUDER EXTRUDER=extruder`, saves the G-code state, and uses
`G21`, `M83`, and signed relative `G1 E... F...` moves. E measures filament
millimetres; F measures millimetres per minute. State is restored after the
terminal `M400`. Motor disable targets only the selected extruder. There is no
`MANUAL_STEPPER ... stepper_x` fallback or global motor-disable command.

Legacy ROS action/type names and the `default_stepper` parameter remain for API
compatibility. Their selected actuator must be a native extruder object. The
execution protocol is `klipper-native-extruder-v1`; the bridge action protocol
is `native-extruder-program-v6`. Rebuild the ROS interface and bridge packages,
restart the bridge, and prepare fresh execution packages after migration.
Previously prepared manual-stepper packages cannot authorize native E motion.

Readiness queries the native extruder configuration and temperature status.
A cold hotend can connect, but actual E motion requires live `can_extrude`
readiness and valid temperature. The current installation uses a 0.2 mm nozzle,
1.75 mm filament, 5 mm/s extrusion-only velocity, 20 mm/s² extrusion-only
acceleration, and Klipper's 50 mm single-move distance guard. Speed and flow
overrides must remain at 100%. The bridge sends no heater or fan commands and
does not relax cold-extrusion protection; prepare the hotend separately.

Stock Klipper stops at every pure E `G1` junction. Program timing includes each
move's acceleration and deceleration, including additional moves created to
respect the distance guard. The common UR/extrusion compiler reserves these
durations before freezing the UR profile; it never slows only Klipper after UR
timing is frozen. Zero-E intervals use `G4`, with cumulative millisecond
rounding. One final `M400` confirms queued movement completion without adding
per-segment barriers.

This coordinates nominal segment timing. It does not certify physical clock
alignment, instantaneous extrusion flow, or deposited material. Cancellation
and uncertain post-dispatch failure retain the coordinated emergency-stop
path; a stopped job requires recovery and a fresh explicit start.

Every action reads the full configuration during preflight. While it runs,
one compact selected-field GET supplies current heater permission, speed/flow
overrides, toolhead limits, Klippy state and, for programs, print state. The
active action owns monitoring; idle polling and manual full refresh cannot
insert configuration reads during motion. Configuration limits are reused only
within that action, and all live safety fields require a new valid response.
Before any upload/start or E command, preflight also reads through the same
persistent monitoring connection and rechecks its fresh safety fields. This
recovers a stale idle socket before dispatch; an unavailable monitor or busy
virtual-SD job rejects the action without sending motion or emergency commands.

One transient `NETWORK_ERROR` on this read may retry once, after a cancelable
250 ms delay. Lock waiting, the first read, delay and retry share the smaller of
the configured request timeout and an eight-second observation budget. The
normal first request gets six seconds, leaving at most 1.75 seconds for retry
after the delay. A late or invalid response, persistent connection failure,
explicit unsafe state or wrong running file aborts through the normal stop path.
Upload/start/motion writes are never replayed by this policy.

This observation budget is not a real-time bound on physical stopping: socket
timeouts apply to socket operations, and emergency-stop delivery/confirmation
has separate limits. Recovery/final-failure logs include elapsed time, read
budget and the error. See `docs/MOONRAKER_STATUS_TIMEOUT_2026-10-19.md` at the
project root for the observed incident and verification scope.

Direct readiness and runtime status GETs share one HTTP/1.1 connection instead
of forcing new TCP handshakes for every sample. Idle readiness still reads the
latest full configuration on every refresh; only the connection is reused.
Full configuration replies are bounded to 1 MiB and compact runtime replies to
64 KiB. A broken socket is discarded; the transport
never retries on its own, and only the existing read policy may reconnect.
Configured proxy routes and explicitly injected openers retain the urllib path.
HTTPS uses the standard verified TLS connection. Each request updates its
socket timeout; malformed, oversized or truncated responses cannot poison the
next sample. A deadline watchdog also interrupts connected-socket I/O when
fragmented headers or body bytes would otherwise keep resetting socket timeouts.
It owns the exact request socket and completes before the next pooled read.
DNS/connect implementation and OS scheduling still preclude hard real-time
guarantees. Status errors report connect/send/headers/body phase and elapsed
time. Control writes and emergency confirmation keep independent connections,
so a blocked status socket cannot hold their lock or cause a POST replay.
This mitigates recurring connection establishment failures; it cannot repair
external network equipment or certify an offline fix against real packet loss.
