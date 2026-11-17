// Hardware-isolated rendering check for the persistent stop interlock.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";
const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const fn = source.match(/function executionRuntimeMarkup\([^]*?\n\}/)[0];
const context = vm.createContext({
  state: { execution: { runtime: { state: "stop-unconfirmed", message: "Unknown outcome", progress: .1 } } },
  t: (key) => key, escapeHtml: (value) => String(value), materialIcon: () => "icon",
});
vm.runInContext(fn, context);
const html = context.executionRuntimeMarkup();
assert.ok(html.includes('execution-runtime-card fail'));
assert.ok(html.includes('executionStateStopUnconfirmed'));
assert.ok(html.includes('Unknown outcome'));
context.state.execution.runtime = {
  state: "stop-unconfirmed", progress: 0,
  message: "Obsolete Klipper pending; repeated old error",
  root_error: { message: "UR feedback stopped" },
  stop: { reasons: ["UR terminal result is missing"] },
  preflight: { program_state_received: true, external_control_running: true,
    joint_state_fresh: true, klipper_action_server: true, trajectory_action_server: true },
};
const structured = context.executionRuntimeMarkup();
assert.ok(structured.includes('<details class="execution-error-details">'));
assert.ok(structured.includes('executionStopLockedDesc'));
assert.ok(structured.includes('UR feedback stopped'));
assert.ok(structured.includes('UR terminal result is missing'));
assert.ok(!structured.includes('Obsolete Klipper pending'));
assert.ok(!structured.includes('class="ready"'), 'Historical green preflight must not look live during a stop fault');
assert.ok(structured.includes('executionBlockProgressHint'));
context.state.execution.runtime = {
  state: "running", progress: .25,
  mode: "move-to-start",
  message: "Moving through the reviewed collision-free approach · measured 25% · nominally 16 s remaining",
  motion: { source: "live_joint_state", nominal_remaining_s: 16 },
  preflight: { program_state_received: true, external_control_running: true,
    joint_state_fresh: true, klipper_action_server: true, trajectory_action_server: true },
};
const measured = context.executionRuntimeMarkup();
assert.ok(measured.includes('value="0.25"'));
assert.ok(!measured.includes('style='), 'Runtime progress must comply with the application CSP');
assert.ok(measured.includes('executionLiveJointProgressHint'));
assert.ok(!measured.includes('executionBlockProgressHint'));
assert.ok(measured.includes('executionMovingToStart'));
assert.ok(measured.includes('executionTimeRemaining'));
assert.ok(!measured.includes('nominally'), 'Do not duplicate the backend diagnostic paragraph in the summary');
assert.ok(!measured.includes('execution-preflight-grid'), 'Start-check snapshots must disappear once running');
context.state.execution.runtime.mode = 'execute';
context.state.execution.runtime.motion = null;
assert.ok(!context.executionRuntimeMarkup().includes('executionAwaitingMotion'), 'An extrusion-only block is not waiting for UR motion');
context.state.execution.runtime.mode = 'move-to-start';
context.state.execution.runtime.motion = { source: 'live_joint_state', nominal_remaining_s: null };
assert.ok(!context.executionRuntimeMarkup().includes('executionTimeRemaining'), 'An unavailable ETA must not be displayed as zero seconds');
context.state.execution.runtime.state = 'preflight';
assert.ok(context.executionRuntimeMarkup().includes('class="execution-preflight-caption"'));
context.state.execution.runtime.state = 'completed';
context.state.execution.package = { operation_mode: 'tool0_motion_test' };
assert.ok(!context.executionRuntimeMarkup().includes('execution-preflight-grid'));
assert.ok(context.executionRuntimeMarkup().includes('motionTestFinishedDesc'));
context.state.execution.package.execution_extent = 'full';
assert.ok(context.executionRuntimeMarkup().includes('fullColdFinishedDesc'));
assert.ok(source.includes('reviewed_low_speed: state.execution.reviewedLowSpeed'));
assert.ok(source.includes('"executionSpeed", state.execution, "reviewedLowSpeed"'));
console.log("Stop-unconfirmed warning and dedicated low-speed authorization passed (no hardware).");
