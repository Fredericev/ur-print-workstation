# Starting ROS again after Stop — 2026-10-19

## Observed failure

Start/Restart jobs between 11:00 and 11:07 that day all failed before launching the UR driver:
`Container connection to Moonraker /printer/info failed (exit 7)`.
Requests from the Mac to the same `/printer/info` returned Klipper ready, while requests from the
container and Colima VM immediately returned Connection refused. The failure remained reproducible
after restarting the container; UR Dashboard also refused connections from the same VM. After
confirming there were no control processes or other running containers, restarting the original
Colima profile and restoring the original Docker context recovered the existing network preflight
and the container-to-UR-Dashboard connection. This locates the fault in the VM network path;
it does not establish the underlying mechanism or prove that Stop itself caused that network state.

## Repair

- The detached startup entrypoint used by the GUI supports one conditional recovery after a
  connection error in network preflight. It first requires confirmation that the Mac receives ready
  from the same Klipper endpoint, VM curl returns a connection error or timeout, and the current
  Docker endpoint belongs to the default Colima profile.
- Recovery rejects an incorrect profile, other running containers, active UR/MoveIt/bridge/print
  processes, or an unreadable workload inventory. Device shutdown, a disconnected host, missing
  container networking or ports, and similar conditions continue to block startup as before.
  Execution records and stop locks are preserved.
- Recovery preserves the original profile configuration and restores the original Docker context
  on both success and failure. It starts only the original ROS container and repeats the complete
  preflight; failure does not cause a restart loop.
- A separate isolated regression reproduced a startup-cancellation timing error: the old startup job
  was marked canceled while its EXIT cleanup was still running, allowing a new Start to launch and
  then be stopped by the old cleanup. The backend now maintains its operation gate during
  cancellation and waits for startup-script cleanup to finish. After 30 seconds it reports incomplete
  cleanup, with the active job continuing to block new operations. The worker continuously drains
  output so a blocked pipe cannot stall cleanup.

## Validation

- Two cancellation regressions failed before the repair. Afterward, all 102 `test_control_station`
  tests passed.
- The full Python regression suite passed 601 tests. After additional cases for restoring the context
  when Colima startup failed or was canceled, all 21 network-recovery tests passed. External commands
  and device requests were isolated in these tests.
- Shell syntax checks passed.
- Actual startup from a fully stopped state succeeded. The UR driver, MoveIt, and bridge were then
  stopped, and another startup succeeded. Both runs passed driver, bridge, 45-second RTDE stability,
  and port 50001–50004 checks.
- The final live API showed the container, driver, host ports, and bridge ready, with no active job,
  RTDE conflict, connection instability, or unconfirmed-stop lock. External Control and MoveIt awaited
  operator startup through the original workflow.
- The local workstation was exited and reopened normally, and the backend loaded the repair from
  the current source files.

Actual start/stop retest logs: `/tmp/ur-stop-restart-launch-1.log` and
`/tmp/ur-stop-restart-launch-2.log`. Full test log:
`/tmp/ur-stop-restart-python-tests.log`. The system may clean up temporary logs.
No arm trajectory, extruder move, heating, teach-pendant Play, or old-print recovery command was sent.

## Hardening against recurrence

The user further requested reliable repeated Start/Stop over long periods. This round extended
protection to the actual buttons, HTTP status synchronization, startup scripts, and runtime
entrypoints, preserving the existing robot-motion and stop-evidence gates.

- UI job queries continue retrying after transient failures. Polling is guarded by job ID and
  generation, so late replies from old jobs cannot overwrite a new job. Old global-status replies
  also cannot restore a previous active job.
- Normal cancellation and Print Stop share a waiting state, blocking repeated cancellation and new
  startup until the cancellation response returns. Request timeouts cover the total backend wait
  for coordinated cancellation and exit cleanup.
- Startup entrypoints install EXIT/HUP/INT/TERM handlers early and acquire ownership of the control
  stack before beginning old-driver cleanup. Cancellation or failure during cleanup completes the
  remaining cleanup; preflight failure preserves an existing healthy control stack.
- If interrupted Colima recovery leaves Docker pointing at the invalid default socket, the next
  install/startup verifies and reselects the local Colima connection. The VM is not blindly restarted
  when workloads cannot be verified. SSH-port repair uses the same shared idle-workload check.
- Docker's top interface must retain the PID field: actual hardware investigation showed that the
  daemon rejects a request for args alone. Using `-eo pid,args` correctly identifies running UR
  control processes and prevents VM restart.
- Each completed job closes its read pipe, avoiding accumulated file descriptors during repeated
  startup and shutdown over long periods.

The new `test_restart_cycles.py` uses real local startup processes, detached simulated control
processes, and delayed EXIT cleanup, alternating cancellation before readiness with stopping after
readiness. The same test failed with the old release. The new code passed five consecutive rounds,
covering 60 cancellation-cleanup cycles, 20 stops after natural completion, and five final restarts.

A new full-entrypoint integration test invokes the real detached script and network checker,
using local HTTP and Docker/Colima command doubles to verify: initial network failure → one recovery
→ restoration of a lost context → complete recheck → driver launch. The command doubles enforce
Docker top's PID requirement.

The current full Python regression suite passed 616 tests. After the Docker-interface correction,
23 network tests and 20 full-entrypoint startup tests passed. The UI has independent regressions
for transient failures, reordered responses, cancellation, and the next Start.
The project's existing `offline-source-ci.yml` automatically discovers these Python/JavaScript tests,
so future changes enter that workflow. This round ran locally only; no remote CI was triggered.

Logs for this round: `/tmp/ur-restart-prevention-full-tests.log` and
`/tmp/ur-restart-prevention-targeted-tests.log`.

The final 15 targeted UI tests and the complete 27-test JavaScript set used by CI passed.
Additional coverage handles job-record loss after an external backend restart: old non-motion
ROS/MoveIt UI locks are released only after new status confirms there is no active job. Lost motion
records still explicitly indicate uncertainty; the UI does not invent stop evidence or clear safety locks.

The local native app reloaded the new code. The final actual startup log is
`/tmp/ur-restart-prevention-live-launch.log`; final targeted runtime logs are
`/tmp/ur-restart-prevention-final-runtime-tests.log`.

The final hardened startup returned 0 and passed the 45-second stability observation and host
reverse-port checks. After reloading, the API confirmed the driver, bridge, container, and host ports
ready, with no active job, RTDE conflict, connection instability, or stop lock. This round sent no
motion, extrusion, heating, or teach-pendant Play command. The final 51 targeted runtime tests passed.
