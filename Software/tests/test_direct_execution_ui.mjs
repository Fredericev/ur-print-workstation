// Explicit button actions, no typed modal; all transports are inert doubles.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";
const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const index = readFileSync(new URL("../app/index.html", import.meta.url), "utf8");
const extract = name => source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`))[0];
const calls = [];
const context = vm.createContext({
  state: { safety: { emergency_stop: true, workspace_clear: true, wired_network: true },
    execution: { package: { execution_id: "fixture", operation_mode: "tool0_motion_test" },
      runtime: { state: "at-start" }, externalControlPlaying: true, reviewedLowSpeed: true },
    moduleSteps: { execution: 1 }, jobSubmitting: false },
  showConfirmation: () => { throw new Error("Typed confirmation must not be invoked"); },
  startJob: async (...args) => { calls.push(args); },
  safetyPayload: extra => ({ emergency_stop: true, workspace_clear: true, wired_network: true, ...extra }),
  jobIsRunning: () => false,
  refreshPrintExecutionStatus: async () => {}, render: () => {}, notify: () => {}, t: key => key,
  window: { setTimeout: () => {} },
  api: async (...args) => { calls.push(args); return { job: { id: "stop", state: "running" } }; },
});
vm.runInContext(extract("startExecutionMotion"), context);
await context.startExecutionMotion("execute");
assert.equal(calls.length, 1);
assert.equal(calls[0][0], "/api/print/start");
assert.equal(calls[0][1].operator_action, "execute");
assert.equal(calls[0][1].execution_id, "fixture");
assert.equal(calls[0][1].reviewed_low_speed, true);
assert.ok(!("confirmation" in calls[0][1]));
context.state.execution.runtime = { state: "stop-unconfirmed" };
await context.startExecutionMotion("move-to-start");
assert.equal(calls.length, 1, "Direct click must not bypass an unconfirmed stop");
context.state.execution.runtime = null;
await context.startExecutionMotion("execute");
assert.equal(calls.length, 1, "Printing still requires the reviewed start");
for (const state of ["failed", "canceled"]) {
  context.state.execution.runtime = { state, mode: "move-to-start" };
  await context.startExecutionMotion("move-to-start");
  assert.equal(calls.length, 1, "A failed or canceled approach must be replanned from the current posture");
}
context.state.execution.runtime = null;
context.state.execution.requiresRevalidation = true;
await context.startExecutionMotion("move-to-start");
assert.equal(calls.length, 1, "The backend revalidation field must also block stale motion");
context.state.execution.requiresRevalidation = false;
await context.startExecutionMotion("move-to-start");
assert.equal(calls[1][1].operator_action, "move-to-start");
vm.runInContext(extract("stopPrintExecution"), context);
await context.stopPrintExecution();
assert.equal(calls[2][0], "/api/print/fixture/stop");
assert.equal(calls[2][1].method, "POST");
assert.doesNotMatch(source, /function showConfirmation|function hideConfirmation|confirmInput|confirmDialog|START MOTION TEST|MOVE TO START|STOP PRINT/);
assert.doesNotMatch(index, /confirmDialog|confirmInput/);

// The generic POST dispatcher locks immediately, not after the response.
let resolveResponse;
let posts = 0;
const dispatch = vm.createContext({ state: { jobSubmitting: false }, render: () => {},
  api: () => { posts++; return new Promise(done => { resolveResponse = done; }); },
  notify: () => {}, pollJob: () => {} });
vm.runInContext(extract("startJob"), dispatch);
const pending = dispatch.startJob("/fixture", {});
assert.equal(dispatch.state.jobSubmitting, true);
await dispatch.startJob("/fixture", {});
assert.equal(posts, 1, "Double click must not submit a second operation");
resolveResponse({ job: { id: "fixture", state: "running" } });
await pending;
assert.equal(dispatch.state.jobSubmitting, false);

// The default full cold range is transported immutably and requests background preparation.
const sourceFile = { name: "fixture.gcode" };
let preparedRequest;
const prepareContext = vm.createContext({
  state: {
    activeJob: null, jobSubmitting: false,
    gcode: { sourceFile, pathReviewMoveitJob: { id: "review" } },
    execution: { extent: "full", preparing: false, package: null, runtime: null },
  },
  moveitExecutionReady: () => true,
  jobIsRunning: () => false,
  render: () => {},
  notify: () => {},
  t: key => key,
  encodeURIComponent,
  fetchWithTimeout: async (...args) => {
    preparedRequest = args;
    return { ok: true, json: async () => ({ package: { execution_id: "full" } }) };
  },
});
vm.runInContext(extract("preparePrintExecution"), prepareContext);
await prepareContext.preparePrintExecution();
assert.equal(preparedRequest[1].headers["X-Execution-Extent"], "full");
assert.equal(preparedRequest[1].headers.Prefer, "respond-async");
assert.equal(preparedRequest[2], 20000);
assert.equal(prepareContext.state.execution.package.execution_id, "full");
prepareContext.state.execution.package = null;
prepareContext.state.execution.extent = "sample";
await prepareContext.preparePrintExecution();
assert.equal(preparedRequest[1].headers["X-Execution-Extent"], "sample");
assert.equal(preparedRequest[2], 20000);

// A long full-model compile is polled over short requests, so the browser never holds one
// multi-minute WKWebView connection open.
const asyncSourceFile = { name: "full.gcode" };
const asyncRequests = [];
let asyncReply = 0;
const asyncPrepareContext = vm.createContext({
  state: {
    activeJob: null, jobSubmitting: false,
    gcode: { sourceFile: asyncSourceFile, pathReviewMoveitJob: { id: "review-full" } },
    execution: { extent: "full", preparing: false, package: null, runtime: null },
  },
  moveitExecutionReady: () => true,
  jobIsRunning: () => false,
  render: () => {},
  notify: () => {},
  t: key => key,
  encodeURIComponent,
  window: { setTimeout: callback => { callback(); return 1; }, clearTimeout: () => {} },
  fetchWithTimeout: async (...args) => {
    asyncRequests.push(args);
    asyncReply += 1;
    if (asyncReply === 1) {
      return { ok: true, status: 202, json: async () => ({ preparation: { id: "abc123def456", state: "running" } }) };
    }
    return { ok: true, status: 200, json: async () => ({ preparation: {
      id: "abc123def456", state: "succeeded", package: { execution_id: "full-async" },
    } }) };
  },
});
vm.runInContext(extract("preparePrintExecution"), asyncPrepareContext);
await asyncPrepareContext.preparePrintExecution();
assert.equal(asyncRequests.length, 2);
assert.equal(asyncRequests[0][0], "/api/print/prepare");
assert.equal(asyncRequests[1][0], "/api/print/prepare/abc123def456");
assert.equal(asyncRequests[1][2], 20000);
assert.equal(asyncPrepareContext.state.execution.package.execution_id, "full-async");

// A stale preparation must not unlock or inject an error into a newer source's preparation.
const staleSourceFile = { name: "stale.gcode" };
const currentSourceFile = { name: "current.gcode" };
const preparationTimers = [];
const raceRequests = [];
const raceNotifications = [];
let preparePosts = 0;
const raceContext = vm.createContext({
  state: {
    activeJob: null, jobSubmitting: false,
    gcode: { sourceFile: staleSourceFile, pathReviewMoveitJob: { id: "aaaaaaaaaaaa" } },
    execution: { extent: "full", preparing: false, package: null, runtime: null, error: "" },
  },
  moveitExecutionReady: () => true,
  jobIsRunning: () => false,
  render: () => {},
  notify: (...args) => raceNotifications.push(args),
  t: key => key,
  encodeURIComponent,
  window: { setTimeout: callback => { preparationTimers.push(callback); return preparationTimers.length; } },
  fetchWithTimeout: async (path) => {
    raceRequests.push(path);
    if (path === "/api/print/prepare") {
      preparePosts += 1;
      const id = preparePosts === 1 ? "aaaaaaaaaaaa" : "bbbbbbbbbbbb";
      return { ok: true, status: 202, json: async () => ({ preparation: { id, state: "running" } }) };
    }
    assert.equal(path, "/api/print/prepare/bbbbbbbbbbbb");
    return { ok: true, status: 200, json: async () => ({ preparation: {
      id: "bbbbbbbbbbbb", state: "succeeded", package: { execution_id: "current-async" },
    } }) };
  },
});
vm.runInContext(extract("preparePrintExecution"), raceContext);
const stalePreparation = raceContext.preparePrintExecution();
await new Promise(resolve => setImmediate(resolve));
assert.equal(preparationTimers.length, 1);
raceContext.state.gcode = { sourceFile: currentSourceFile, pathReviewMoveitJob: { id: "bbbbbbbbbbbb" } };
raceContext.state.execution = { extent: "full", preparing: false, package: null, runtime: null, error: "" };
const currentExecutionState = raceContext.state.execution;
const currentPreparation = raceContext.preparePrintExecution();
await new Promise(resolve => setImmediate(resolve));
assert.equal(preparationTimers.length, 2);
preparationTimers[0]();
await stalePreparation;
assert.equal(raceContext.state.execution, currentExecutionState);
assert.equal(currentExecutionState.preparing, true, "Stale completion must not unlock the current preparation");
assert.equal(currentExecutionState.error, "", "Stale completion must not inject an error into the current source");
assert.deepEqual(raceNotifications, []);
preparationTimers[1]();
await currentPreparation;
assert.equal(currentExecutionState.preparing, false);
assert.equal(currentExecutionState.package.execution_id, "current-async");
assert.deepEqual(raceRequests, [
  "/api/print/prepare",
  "/api/print/prepare",
  "/api/print/prepare/bbbbbbbbbbbb",
]);
console.log("Direct button intent, stop/start guards, and immediate double-click lock passed (no hardware).");
