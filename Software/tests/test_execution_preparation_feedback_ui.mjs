// Hardware-isolated UI regression coverage for long cold-test preparation.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(
  process.env.APP_JS_UNDER_TEST || new URL("../app/app.js", import.meta.url),
  "utf8",
);
const extract = name => {
  const match = source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`));
  assert.ok(match, `Missing ${name}`);
  return match[0];
};
const text = (key, values = {}) => values.time ? `${key}:${values.time}` : key;
const escapeHtml = value => String(value);

// A running backend summary is the source of truth for phase, progress, and ETA.
// The polling loop must retain it and re-render before waiting for the next poll.
const timers = [];
const renderSnapshots = [];
const sourceFile = { name: "large-model.gcode" };
let reply = 0;
let progressContext;
progressContext = vm.createContext({
  state: {
    activeJob: null,
    jobSubmitting: false,
    gcode: { sourceFile, pathReviewMoveitJob: { id: "abc123def456" } },
    execution: {
      extent: "full", preparing: false, preparation: null,
      package: null, runtime: null, error: "",
    },
  },
  moveitExecutionReady: () => true,
  jobIsRunning: () => false,
  render: () => {
    if (progressContext.state.execution.preparation) {
      renderSnapshots.push(progressContext.preparationProgressMarkup());
    }
  },
  notify: () => {},
  t: text,
  escapeHtml,
  encodeURIComponent,
  window: { setTimeout: callback => { timers.push(callback); return timers.length; } },
  fetchWithTimeout: async () => {
    reply += 1;
    if (reply === 1) {
      return { ok: true, status: 202, json: async () => ({ preparation: {
        id: "feed1234feed", state: "running", phase: "certifying_print_curve",
        progress: 0.625, completed_units: 625, total_units: 1000,
        estimated_remaining_s: 42.5,
      } }) };
    }
    return { ok: true, status: 200, json: async () => ({ preparation: {
      id: "feed1234feed", state: "succeeded", phase: "complete", progress: 1,
      package: { execution_id: "prepared-package" },
    } }) };
  },
});
for (const name of [
  "executionDuration", "preparationPhaseLabel", "preparationProgressMarkup",
  "preparePrintExecution",
]) vm.runInContext(extract(name), progressContext);

progressContext.state.execution.preparation = {
  phase: "queued", progress: 0, estimated_remaining_s: null,
};
const initialProgress = progressContext.preparationProgressMarkup();
assert.match(initialProgress, /preparationEtaPending/,
  "Unknown ETA must stay in its estimating state");
assert.doesNotMatch(initialProgress, /0m 00s/,
  "A null ETA must not be presented as zero remaining time");
progressContext.state.execution.preparation = null;

const pendingPreparation = progressContext.preparePrintExecution();
await new Promise(resolve => setImmediate(resolve));
assert.equal(timers.length, 1, "Running preparation should wait before its next bounded poll");
assert.equal(progressContext.state.execution.preparation.phase, "certifying_print_curve");
assert.equal(progressContext.state.execution.preparation.progress, 0.625);
assert.match(renderSnapshots.at(-1), /preparationPhaseCertifyingPrintCurve/);
assert.match(renderSnapshots.at(-1), /value="0\.625" max="1"/);
assert.match(renderSnapshots.at(-1), />63%<\/strong>/);
assert.match(renderSnapshots.at(-1), /preparationEta:0m 43s/);

timers[0]();
await pendingPreparation;
assert.equal(progressContext.state.execution.preparation, null,
  "Terminal preparation state must not remain as stale progress after the package is ready");
assert.equal(progressContext.state.execution.package.execution_id, "prepared-package");

// The package summary must distinguish the reviewed MoveIt clock from the
// final executable clock instead of presenting both as a generic motion time.
const timingContext = vm.createContext({
  state: { execution: { package: {
    operation_mode: "tool0_motion_test",
    execution_extent: "full",
    simulation: false,
    block_count: 3800,
    extrusion_block_count: 3476,
    planned_duration_s: 1500,
    estimated_minimum_duration_s: 1500,
    timing_audit: {
      source_nominal_gcode_duration_s: 600,
      source_moveit_duration_s: 728,
      process_only_hold_duration_s: 211,
      schedule_floor_extension_s: 50,
      certified_retiming_overhead_s: 501,
      certified_approach_duration_s: 10,
    },
    execution_scope: { end_progress: 1 },
    manual_stepper: "stepper_x",
    print_bed_m: {
      minimum: { x: -0.5, y: -0.25 }, maximum: { x: -0.1, y: 0.25 }, plane_z: -0.014,
    },
    tool_center_point_m: { offset: { x: 0, y: 0, z: 0 }, collision_radius_m: 0 },
    gates: [],
    execution_id: "0123456789abcdef",
    source_sha256: "a".repeat(64),
  } } },
  t: text,
  escapeHtml,
  materialIcon: name => name,
});
for (const name of ["executionDuration", "executionGateLabel", "executionPackageMarkup"]) {
  vm.runInContext(extract(name), timingContext);
}
const packageMarkup = timingContext.executionPackageMarkup();
assert.match(packageMarkup, /nominalGcodeTime<\/small><strong>10m 00s/);
assert.match(packageMarkup, /reviewedMoveitTime<\/small><strong>12m 08s/);
assert.match(packageMarkup, /scheduleFloorExtension<\/small><strong>0m 50s/);
assert.match(packageMarkup, /finalExecutableTime<\/small><strong>25m 00s/);
assert.match(packageMarkup, /executionTimingBreakdown/);
assert.match(packageMarkup, /executionTimingHint/);
timingContext.state.execution.package.operation_mode = "production_print";
const productionMarkup = timingContext.executionPackageMarkup();
assert.match(productionMarkup, /executionTimingBreakdown/);
assert.match(productionMarkup, /executionTimingHint/,
  "Production prints must also explain why feed-only G-code time differs from executable robot time");

console.log("Preparation progress/ETA and explicit execution timing labels passed (no hardware).");
