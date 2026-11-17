// The paired source is a read-only import, never a shortcut to print execution.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const extract = name => {
  const match = source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`));
  assert.ok(match, `Missing ${name}`);
  return match[0];
};

function contextFor(state, fetchWithTimeout) {
  const requests = [];
  const context = vm.createContext({
    state,
    t: key => key,
    render() {},
    notify() {},
    resetPrintExecution() {},
    stopPathReviewPlayback() {},
    destroyPathReviewViewer() {},
    fetchWithTimeout: async (...args) => {
      requests.push(args);
      return fetchWithTimeout(...args);
    },
  });
  for (const name of ["clearImportedJob", "checkPairedImport", "parseGcodeFile"]) {
    vm.runInContext(extract(name), context);
  }
  return { context, requests };
}

const trajectory = { name: "route.csv", size: 12, text: async () => "time_s,path_progress,..." };
const extrusion = { name: "extrusion.gcode", size: 20, text: async () => "G21\nM83\n" };
const state = {
  gcode: {
    importMode: "paired", importRequestId: 0, pathReviewRequestId: 0,
    result: null, plan: null, sourceFile: null,
  },
  pairedImport: {
    trajectoryFile: trajectory, extrusionFile: extrusion,
    result: null, error: "", checking: false, requestId: 0,
  },
  moduleSteps: { motion: 1 },
};
const analysis = { execution_ready: false, moveit_validated: false };
const { context, requests } = contextFor(state, async () => ({
  ok: true, json: async () => ({ analysis }),
}));
await context.checkPairedImport();
assert.equal(requests.length, 1);
assert.equal(requests[0][0], "/api/paired-timeline/parse");
assert.deepEqual(Object.keys(JSON.parse(requests[0][1].body)), ["trajectory", "extrusion"]);
assert.equal(state.pairedImport.result, analysis);
assert.equal(state.gcode.result, null, "the paired result cannot populate the original G-code route");

context.clearImportedJob();
assert.equal(state.pairedImport.result, null);
assert.equal(state.gcode.sourceFile, null);
assert.equal(state.gcode.plan, null);
assert.equal(state.moduleSteps.motion, 0);
assert.equal(state.gcode.importRequestId, 1);

let finishRequest;
const pendingState = {
  gcode: { importMode: "paired", importRequestId: 0, pathReviewRequestId: 0 },
  pairedImport: {
    trajectoryFile: trajectory, extrusionFile: extrusion,
    result: null, error: "", checking: false, requestId: 0,
  },
  moduleSteps: { motion: 0 },
};
const pendingContext = contextFor(pendingState, () => new Promise(resolve => { finishRequest = resolve; }));
const pending = pendingContext.context.checkPairedImport();
await new Promise(resolve => setImmediate(resolve));
pendingState.gcode.importMode = "gcode";
pendingState.pairedImport.requestId += 1;
finishRequest({ ok: true, json: async () => ({ analysis }) });
await pending;
assert.equal(pendingState.pairedImport.result, null, "late paired response must not restore stale state");

let finishOriginal;
const originalState = {
  gcode: { importMode: "gcode", importRequestId: 0, pathReviewRequestId: 0 },
  pairedImport: { requestId: 0 }, moduleSteps: { motion: 0 },
};
const originalContext = contextFor(originalState, () => new Promise(resolve => { finishOriginal = resolve; }));
const originalPending = originalContext.context.parseGcodeFile({ name: "old.gcode", size: 5 });
await new Promise(resolve => setImmediate(resolve));
originalState.gcode.importMode = "paired";
originalState.gcode.importRequestId += 1;
finishOriginal({ ok: true, json: async () => ({ analysis: { safe_to_plan: true } }) });
await originalPending;
assert.equal(originalState.gcode.result, null, "late original response must not restore stale state");

assert.match(source, /state\.gcode\.importMode === "gcode"\s*&& state\.gcode\.result\?\.safe_to_plan/);
console.log("Paired import UI state and execution-gate isolation: OK");
