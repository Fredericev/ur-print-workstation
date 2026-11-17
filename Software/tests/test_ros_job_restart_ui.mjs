// Exercise delayed and failed HTTP replies; no ROS or hardware transports run.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const extract = name => source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`))[0];
const deferred = () => {
  let resolve;
  const promise = new Promise(done => { resolve = done; });
  return { promise, resolve };
};

function fixture(api) {
  const timers = new Map();
  let timerId = 0;
  const notifications = [];
  const context = vm.createContext({
    state: {
      activeJob: { id: "old", state: "running" }, jobSubmitting: false,
      jobCanceling: null, jobPollTimer: null, jobSuccessHandler: null,
      jobPollGeneration: 0, demo: false, rosStages: {}, completed: new Set(),
    },
    api, render: () => {}, refreshStatus: async () => {}, refreshPrintExecutionStatus: async () => {},
    notify: (...args) => notifications.push(args), t: value => value,
    window: {
      setTimeout: callback => { timers.set(++timerId, callback); return timerId; },
      clearTimeout: id => timers.delete(id),
    },
  });
  for (const name of ["jobIsRunning", "pollJob", "startJob", "cancelActiveJob", "stopPrintExecution"]) {
    vm.runInContext(extract(name), context);
  }
  return { context, timers, notifications };
}

test("a temporary job-status failure retries and eventually enables Start", async () => {
  let attempts = 0;
  const { context, timers } = fixture(async () => {
    if (++attempts === 1) throw new Error("temporary network failure");
    return { job: { id: "old", state: "canceled" } };
  });
  await context.pollJob("old");
  assert.equal(timers.size, 1, "A failed read must retain a recovery poll");
  await [...timers.values()][0]();
  assert.equal(context.state.activeJob.state, "canceled");
  assert.equal(context.jobIsRunning(), false);
});

test("a late poll for the canceled job cannot replace or finish a new Start", async () => {
  const response = deferred();
  const { context } = fixture(() => response.promise);
  const pending = context.pollJob("old");
  let completedNew = 0;
  context.state.activeJob = { id: "new", state: "running" };
  context.state.jobSuccessHandler = () => { completedNew++; };
  response.resolve({ job: { id: "old", state: "succeeded" } });
  await pending;
  assert.equal(context.state.activeJob.id, "new");
  assert.equal(completedNew, 0);
  assert.equal(typeof context.state.jobSuccessHandler, "function");
});

test("a stale failed poll cannot schedule over the current job", async () => {
  let rejectRead;
  const { context, timers, notifications } = fixture(() => new Promise((_, reject) => { rejectRead = reject; }));
  const pending = context.pollJob("old");
  context.state.activeJob = { id: "new", state: "running" };
  rejectRead(new Error("old connection failed"));
  await pending;
  assert.equal(timers.size, 0);
  assert.equal(notifications.length, 0);
});

test("a late cancel reply cannot replace the new job", async () => {
  const response = deferred();
  const { context } = fixture(() => response.promise);
  const pending = context.cancelActiveJob();
  context.state.activeJob = { id: "new", state: "running" };
  response.resolve({ job: { id: "old", state: "canceled" } });
  await pending;
  assert.equal(context.state.activeJob.id, "new");
});

test("cancel in progress keeps Start locked and repeated Cancel does not overlap", async () => {
  const response = deferred();
  let cancels = 0;
  const { context } = fixture(() => { cancels++; return response.promise; });
  const pending = context.cancelActiveJob();
  context.state.activeJob = { id: "old", state: "canceled" };
  assert.equal(context.jobIsRunning(), true, "A terminal poll cannot bypass the pending cancel request");
  await context.cancelActiveJob();
  assert.equal(cancels, 1);
  // Move to another job before releasing the old reply, so this test never polls it.
  context.state.activeJob = { id: "new", state: "succeeded" };
  response.resolve({ job: { id: "old", state: "canceled" } });
  await pending;
  assert.equal(context.jobIsRunning(), false);
});

test("an old system-status request cannot restore the old job after a new Start", async () => {
  const response = deferred();
  const { context } = fixture(() => response.promise);
  vm.runInContext(extract("refreshStatus"), context);
  const pending = context.refreshStatus(false);
  context.state.activeJob = { id: "new", state: "running" };
  response.resolve({ ros: {}, active_job: { id: "old", state: "running" } });
  await pending;
  assert.equal(context.state.activeJob.id, "new");
});

test("overlapping reads of the same job cannot regress its completed state", async () => {
  const oldRead = deferred();
  let reads = 0;
  let successes = 0;
  const { context, timers } = fixture(() => ++reads === 1
    ? oldRead.promise : Promise.resolve({ job: { id: "old", state: "succeeded" } }));
  context.state.jobSuccessHandler = () => { successes++; };
  const pending = context.pollJob("old");
  await context.pollJob("old");
  oldRead.resolve({ job: { id: "old", state: "running" } });
  await pending;
  assert.equal(context.state.activeJob.state, "succeeded");
  assert.equal(successes, 1);
  assert.equal(timers.size, 0);
});

test("normal cancel refreshes the stopped state and the next Start completes", async () => {
  const calls = [];
  const { context } = fixture(async (path, options) => {
    calls.push([path, options]);
    if (path.endsWith("/cancel")) return { job: { id: "old", state: "canceled" } };
    if (path === "/api/ros/start") return { job: { id: "new", state: "running" } };
    return { job: { id: path.endsWith("new") ? "new" : "old", state: path.endsWith("new") ? "succeeded" : "canceled" } };
  });
  let refreshed = 0;
  context.refreshStatus = async () => { refreshed++; };
  await context.cancelActiveJob();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(context.jobIsRunning(), false);
  assert.ok(refreshed > 0, "A canceled launcher must refresh live ROS readiness");
  let started = 0;
  await context.startJob("/api/ros/start", {}, () => { started++; });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(context.state.activeJob.id, "new");
  assert.equal(context.state.activeJob.state, "succeeded");
  assert.equal(started, 1);
  assert.equal(calls[0][1].timeoutMs, 65000);
});

test("a late Print Stop reply cannot overwrite a new job or execution package", async () => {
  const response = deferred();
  const { context } = fixture(() => response.promise);
  const execution = { package: { execution_id: "print-old" }, stopping: false };
  context.state.execution = execution;
  const pending = context.stopPrintExecution();
  context.state.activeJob = { id: "new", state: "running" };
  context.state.execution = { package: { execution_id: "print-new" }, stopping: false };
  response.resolve({ job: { id: "old", state: "canceled" } });
  await pending;
  assert.equal(context.state.activeJob.id, "new");
  assert.equal(context.state.execution.package.execution_id, "print-new");
  assert.equal(context.state.execution.stopping, false);
  assert.equal(execution.stopping, false);
  assert.equal(context.state.jobCanceling, null);
});

test("Print Stop shares the pending cancel gate and releases it after an error", async () => {
  const response = deferred();
  let posts = 0;
  const { context } = fixture(() => { posts++; return response.promise; });
  context.state.execution = { package: { execution_id: "print" }, stopping: false };
  const pending = context.stopPrintExecution();
  await context.cancelActiveJob();
  await context.startJob("/api/ros/start", {});
  assert.equal(posts, 1, "No operation can overlap the pending Print Stop");
  response.resolve(Promise.reject(new Error("stop transport failed")));
  await pending;
  assert.equal(context.state.execution.stopping, false);
  assert.equal(context.state.jobCanceling, null);
  assert.equal(context.state.activeJob.state, "running", "Transport failure is not proof that hardware stopped");
});

test("Print Stop invalidates a job read that was in flight before cancellation", async () => {
  const oldRead = deferred();
  let reads = 0;
  const { context, timers } = fixture(path => {
    if (path === "/api/jobs/old" && ++reads === 1) return oldRead.promise;
    return Promise.resolve({ job: { id: "old", state: "canceled" } });
  });
  context.state.execution = { package: { execution_id: "print" }, stopping: false };
  const pending = context.pollJob("old");
  await context.stopPrintExecution();
  oldRead.resolve({ job: { id: "old", state: "running" } });
  await pending;
  assert.equal(context.state.activeJob.state, "canceled");
  assert.equal(context.jobIsRunning(), false);
  assert.equal(timers.size, 0);
});

test("a restarted external backend releases only the missing service-job UI lock", async () => {
  const calls = [];
  const { context, timers } = fixture(async path => {
    calls.push(path);
    if (path === "/api/jobs/old") throw Object.assign(new Error("Job not found"), { status: 404 });
    assert.equal(path, "/api/status");
    return { ros: { driver_started: false }, active_job: null, latest_job: null,
      print_interlock: { locked: true, recovery_available: true } };
  });
  vm.runInContext(extract("refreshStatus"), context);
  const runtime = { state: "stop-unconfirmed" };
  context.state.activeJob = { id: "old", kind: "start-ros", motion: false, state: "running" };
  context.state.execution = { runtime };
  context.state.jobSuccessHandler = () => { throw new Error("A lost job has no success proof"); };
  await context.pollJob("old");
  assert.equal(context.jobIsRunning(), false);
  assert.equal(context.state.activeJob, null, "Only the stale service UI job is released");
  assert.equal(context.state.printInterlock.locked, true);
  assert.equal(context.state.execution.runtime, runtime, "Missing service status cannot alter motion evidence");
  assert.equal(context.state.jobSuccessHandler, null);
  assert.equal(timers.size, 0);
  assert.deepEqual(calls, ["/api/jobs/old", "/api/status"]);
});

test("missing service status adopts the server's actual different active job", async () => {
  const { context } = fixture(async path => {
    if (path === "/api/jobs/old") throw Object.assign(new Error("Job not found"), { status: 404 });
    if (path === "/api/status") return { ros: {}, active_job: { id: "new", kind: "stop-ros", motion: false, state: "running" }, print_interlock: { locked: true } };
    return { job: { id: "new", kind: "stop-ros", motion: false, state: "running" } };
  });
  vm.runInContext(extract("refreshStatus"), context);
  context.state.activeJob = { id: "old", kind: "start-ros", motion: false, state: "running" };
  context.state.jobSuccessHandler = () => { throw new Error("Lost-job callback must not belong to the server's different job"); };
  await context.pollJob("old");
  assert.equal(context.state.activeJob.id, "new");
  assert.equal(context.state.jobSuccessHandler, null);
  assert.equal(context.jobIsRunning(), true);
  assert.equal(context.state.printInterlock.locked, true);
});

test("404 without a fresh system-status response keeps the service operation locked", async () => {
  const { context, timers } = fixture(async path => {
    if (path === "/api/jobs/old") throw Object.assign(new Error("Job not found"), { status: 404 });
    throw new Error("backend still unavailable");
  });
  vm.runInContext(extract("refreshStatus"), context);
  context.state.activeJob = { id: "old", kind: "stop-ros", motion: false, state: "running" };
  await context.pollJob("old");
  assert.equal(context.state.activeJob.id, "old");
  assert.equal(context.jobIsRunning(), true);
  assert.equal(timers.size, 1, "Recovery retries until fresh server status is available");
});

test("a missing motion job remains explicitly unconfirmed and retains recovery locks", async () => {
  const { context, timers, notifications } = fixture(async path => {
    if (path === "/api/jobs/old") throw Object.assign(new Error("Job not found"), { status: 404 });
    return { ros: {}, active_job: null, print_interlock: { locked: true, recovery_available: true } };
  });
  vm.runInContext(extract("refreshStatus"), context);
  context.state.activeJob = { id: "old", kind: "print-execute", motion: true, state: "running" };
  const runtime = { state: "running", physical_stop_confirmed: false };
  context.state.execution = { runtime };
  await context.pollJob("old");
  assert.equal(context.state.activeJob.id, "old");
  assert.equal(context.jobIsRunning(), true);
  assert.equal(context.state.printInterlock.locked, true);
  assert.equal(context.state.execution.runtime, runtime);
  assert.ok(notifications.some(([message]) => message === "operationMotionRecordLost"));
  assert.equal(timers.size, 1);
});
