// Regression coverage for long-running 3D review generation. All transports are inert.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const extract = name => source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`))[0];
const openPathReviewSource = extract("openGcodePathReview");
const validateMoveitSource = extract("validatePathWithMoveIt");

function makeState(sourceFile) {
  return {
    gcode: {
      sourceFile,
      view: "planning",
      pathReviewLoading: false,
      pathReview: null,
      pathReviewError: "",
      pathReviewMoveitError: "",
      pathReviewMoveitResult: null,
      pathReviewSimulationResult: null,
      pathReviewAutoValidationAttempted: false,
      pathReviewRequestedSimulation: false,
      pathReviewProgress: 0,
      pathReviewPlaying: false,
      pathReviewRequestId: 0,
      pathReviewJob: null,
    },
  };
}

function makeContext(state, fetchWithTimeout, timers = []) {
  const notifications = [];
  const context = vm.createContext({
    state,
    fetchWithTimeout,
    encodeURIComponent,
    stopPathReviewPlayback: () => {},
    render: () => {},
    notify: (...args) => notifications.push(args),
    t: key => key,
    hasRobotMotionTrajectory: value => Boolean(value?.trajectory?.length),
    hasValidatedMoveItTrajectory: () => false,
    moveitValidationAvailable: () => false,
    validatePathWithMoveIt: async () => {},
    window: {
      setTimeout: callback => {
        timers.push(callback);
        return timers.length;
      },
    },
  });
  vm.runInContext(openPathReviewSource, context);
  return { context, notifications };
}

// A slow review is submitted once, then polled with bounded individual requests.
// The browser does not hold a single 30-second connection open.
{
  const sourceFile = { name: "complete.gcode" };
  const state = makeState(sourceFile);
  const requests = [];
  const timers = [];
  let reply = 0;
  const { context, notifications } = makeContext(state, async (...args) => {
    requests.push(args);
    reply += 1;
    if (reply === 1) {
      return { ok: true, status: 202, json: async () => ({ review_job: {
        id: "abc123def456", state: "queued", phase: "queued", progress: 0,
        status_url: "/api/gcode/review/jobs/abc123def456",
      } }) };
    }
    return { ok: true, status: 200, json: async () => ({ review_job: {
      id: "abc123def456", state: "succeeded", phase: "complete", progress: 100,
      review: { id: "complete-review", offline_simulation: { trajectory: [[0, 1, 2]] } },
    } }) };
  }, timers);
  const pending = context.openGcodePathReview();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(requests.length, 1);
  assert.equal(requests[0][0], "/api/gcode/review");
  assert.equal(requests[0][1].headers.Prefer, "respond-async");
  assert.equal(requests[0][2], 20000, "Each request must have its own bounded timeout");
  assert.equal(state.gcode.pathReviewJob.progress, 0);
  assert.equal(state.gcode.pathReviewLoading, true);
  assert.equal(timers.length, 1);
  timers[0]();
  await pending;
  assert.deepEqual(requests.map(request => request[0]), [
    "/api/gcode/review",
    "/api/gcode/review/jobs/abc123def456",
  ]);
  assert.equal(requests[1][2], 20000);
  assert.equal(state.gcode.pathReview.id, "complete-review");
  assert.equal(state.gcode.pathReviewSimulationResult.trajectory.length, 1);
  assert.equal(state.gcode.pathReviewLoading, false);
  assert.equal(state.gcode.pathReviewJob, null);
  assert.deepEqual(notifications, []);
}

// A legacy backend may still return the review immediately with HTTP 200.
{
  const sourceFile = { name: "legacy.gcode" };
  const state = makeState(sourceFile);
  const requests = [];
  const { context } = makeContext(state, async (...args) => {
    requests.push(args);
    return { ok: true, status: 200, json: async () => ({ review: { id: "legacy-review" } }) };
  });
  await context.openGcodePathReview();
  assert.equal(requests.length, 1);
  assert.equal(state.gcode.pathReview.id, "legacy-review");
}

// A cached background result may already be terminal in the POST summary; the
// full review is still retrieved once from its status endpoint.
{
  const sourceFile = { name: "cached.gcode" };
  const state = makeState(sourceFile);
  const requests = [];
  const { context } = makeContext(state, async (...args) => {
    requests.push(args);
    if (requests.length === 1) {
      return { ok: true, status: 202, json: async () => ({ review_job: {
        id: "caced00caced", state: "succeeded", phase: "complete", progress: 100,
        status_url: "/api/gcode/review/jobs/caced00caced",
      } }) };
    }
    return { ok: true, status: 200, json: async () => ({ review_job: {
      id: "caced00caced", state: "succeeded", phase: "complete", progress: 100,
      review: { id: "cached-review" },
    } }) };
  });
  await context.openGcodePathReview();
  assert.deepEqual(requests.map(request => request[0]), [
    "/api/gcode/review",
    "/api/gcode/review/jobs/caced00caced",
  ]);
  assert.equal(state.gcode.pathReview.id, "cached-review");
}

// Simulation-only review uses the same background protocol without contacting
// MoveIt or changing the physical execution gates.
{
  const sourceFile = { name: "simulation.gcode" };
  const state = makeState(sourceFile);
  const requests = [];
  const { context } = makeContext(state, async (...args) => {
    requests.push(args);
    return { ok: true, status: 200, json: async () => ({ review: {
      id: "simulation-review", simulation_only: true,
    } }) };
  });
  await context.openGcodePathReview(true, true);
  assert.equal(requests[0][0], "/api/gcode/review/simulation");
  assert.equal(requests[0][1].headers.Prefer, "respond-async");
  assert.equal(state.gcode.pathReview.simulation_only, true);
  assert.equal(state.gcode.pathReviewRequestedSimulation, true);
}

// Reopening an exact completed review returns the same lightweight job first;
// the UI must fetch its retained result instead of treating it as malformed.
{
  const sourceFile = { name: "cached-complete.gcode" };
  const state = makeState(sourceFile);
  const requests = [];
  const { context } = makeContext(state, async (path) => {
    requests.push(path);
    if (requests.length === 1) {
      return { ok: true, status: 202, json: async () => ({ review_job: {
        id: "123456abcdef", state: "succeeded", phase: "complete", progress: 100,
        status_url: "/api/gcode/review/jobs/123456abcdef",
      } }) };
    }
    return { ok: true, status: 200, json: async () => ({ review_job: {
      id: "123456abcdef", state: "succeeded", phase: "complete", progress: 100,
      review: { id: "cached-review" },
    } }) };
  });
  await context.openGcodePathReview();
  assert.deepEqual(requests, [
    "/api/gcode/review",
    "/api/gcode/review/jobs/123456abcdef",
  ]);
  assert.equal(state.gcode.pathReview.id, "cached-review");
}

// Loading locks out a second click before the first POST resolves.
{
  const sourceFile = { name: "duplicate.gcode" };
  const state = makeState(sourceFile);
  let resolvePost;
  let posts = 0;
  const { context } = makeContext(state, () => {
    posts += 1;
    return new Promise(resolve => { resolvePost = resolve; });
  });
  const first = context.openGcodePathReview();
  await context.openGcodePathReview();
  assert.equal(posts, 1);
  resolvePost({ ok: true, status: 200, json: async () => ({ review: { id: "one" } }) });
  await first;
  assert.equal(state.gcode.pathReview.id, "one");
}

// Completion from an old file must not overwrite the newly selected file.
{
  const staleFile = { name: "stale.gcode" };
  const currentFile = { name: "current.gcode" };
  const state = makeState(staleFile);
  const timers = [];
  const requests = [];
  const { context, notifications } = makeContext(state, async (path) => {
    requests.push(path);
    if (path === "/api/gcode/review" && requests.length === 1) {
      return { ok: true, status: 202, json: async () => ({ review_job: {
        id: "aaaaaaaaaaaa", state: "running", progress: 10,
        status_url: "/api/gcode/review/jobs/aaaaaaaaaaaa",
      } }) };
    }
    assert.equal(path, "/api/gcode/review");
    return { ok: true, status: 200, json: async () => ({ review: { id: "current-review" } }) };
  }, timers);
  const stale = context.openGcodePathReview();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(timers.length, 1);
  state.gcode.sourceFile = currentFile;
  state.gcode.pathReviewLoading = false;
  const current = context.openGcodePathReview(true);
  await current;
  assert.equal(state.gcode.pathReview.id, "current-review");
  timers[0]();
  await stale;
  assert.equal(state.gcode.pathReview.id, "current-review");
  assert.deepEqual(requests, ["/api/gcode/review", "/api/gcode/review"]);
  assert.deepEqual(notifications, []);
}

// Leaving the 3D review may keep the harmless local calculation, but completion
// must not automatically start the separate online MoveIt validation.
{
  const sourceFile = { name: "leave-review.gcode" };
  const state = makeState(sourceFile);
  let resolvePost;
  let moveitStarts = 0;
  const timers = [];
  const { context } = makeContext(state, () => new Promise(resolve => { resolvePost = resolve; }), timers);
  context.moveitValidationAvailable = () => true;
  context.validatePathWithMoveIt = async () => { moveitStarts += 1; };
  const pending = context.openGcodePathReview();
  state.gcode.view = "planning";
  resolvePost({ ok: true, status: 200, json: async () => ({ review: { id: "completed-locally" } }) });
  await pending;
  assert.equal(state.gcode.pathReview.id, "completed-locally");
  assert.equal(moveitStarts, 0);
  assert.equal(timers.length, 0, "Leaving the page must not schedule online validation");
}

// The cached-review fast branch rechecks the page before its deferred online gate.
{
  const sourceFile = { name: "cached-leave.gcode" };
  const state = makeState(sourceFile);
  state.gcode.pathReview = { id: "already-loaded", simulation_only: false };
  const timers = [];
  let moveitStarts = 0;
  const { context } = makeContext(state, () => {
    throw new Error("A cached review must not issue another review POST");
  }, timers);
  context.moveitValidationAvailable = () => true;
  context.validatePathWithMoveIt = async () => { moveitStarts += 1; };
  await context.openGcodePathReview();
  assert.equal(timers.length, 1);
  state.gcode.view = "planning";
  timers[0]();
  assert.equal(moveitStarts, 0);
  assert.equal(state.gcode.pathReviewAutoValidationAttempted, false);
}

// Backend terminal failures remain visible instead of becoming generic timeouts.
{
  const sourceFile = { name: "failed.gcode" };
  const state = makeState(sourceFile);
  let reply = 0;
  const { context, notifications } = makeContext(state, async () => {
    reply += 1;
    return reply === 1
      ? { ok: true, status: 202, json: async () => ({ review_job: {
        id: "ffffffffffff", state: "running", progress: 10,
        status_url: "/api/gcode/review/jobs/ffffffffffff",
      } }) }
      : { ok: true, status: 200, json: async () => ({ review_job: {
        id: "ffffffffffff", state: "failed", progress: 100,
        error: "The path contains an unreachable pose.", http_status: 422,
      } }) };
  });
  context.window.setTimeout = callback => { callback(); return 1; };
  await context.openGcodePathReview();
  assert.equal(state.gcode.pathReview, null);
  assert.equal(state.gcode.pathReviewError, "The path contains an unreachable pose.");
  assert.deepEqual(notifications, [["The path contains an unreachable pose.", "error"]]);
}

assert.match(
  source,
  /async function parseGcodeFile\(file\)[^]*pathReviewRequestId = Number\(state\.gcode\.pathReviewRequestId \|\| 0\) \+ 1;/,
  "Selecting another file must invalidate every older review completion",
);

// A MoveIt response for an older source/review cannot overwrite a new import.
{
  const staleFile = { name: "moveit-stale.gcode" };
  const currentFile = { name: "moveit-current.gcode" };
  const staleReview = { id: "stale-review", simulation_only: false };
  const currentReview = { id: "current-review", simulation_only: false };
  const timers = [];
  const notifications = [];
  const state = makeState(staleFile);
  state.gcode.pathReview = staleReview;
  const context = vm.createContext({
    state,
    moveitValidationAvailable: () => true,
    isOnlineMode: () => true,
    resetPrintExecution: () => {},
    stopPathReviewPlayback: () => {},
    render: () => {},
    notify: (...args) => notifications.push(args),
    t: key => key,
    encodeURIComponent,
    hasRobotMotionTrajectory: () => false,
    document: { getElementById: () => null },
    api: async () => { throw new Error("A stale MoveIt request must stop before polling"); },
    fetchWithTimeout: async () => ({
      ok: true,
      status: 202,
      json: async () => ({ job: { id: "moveit-job", state: "running", log: "" } }),
    }),
    window: { setTimeout: callback => { timers.push(callback); return timers.length; } },
  });
  vm.runInContext(validateMoveitSource, context);
  const stale = context.validatePathWithMoveIt();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(timers.length, 1);
  state.gcode.sourceFile = currentFile;
  state.gcode.pathReview = currentReview;
  state.gcode.pathReviewRequestId += 1;
  state.gcode.pathReviewMoveitRunning = false;
  state.gcode.pathReviewMoveitResult = { id: "current-result" };
  timers[0]();
  await stale;
  assert.equal(state.gcode.pathReview, currentReview);
  assert.equal(state.gcode.pathReviewMoveitResult.id, "current-result");
  assert.equal(state.gcode.pathReviewMoveitRunning, false);
  assert.deepEqual(notifications, []);
}

console.log("Async 3D path review UI polling, deduplication, stale-file guard, and errors passed (no hardware).");
