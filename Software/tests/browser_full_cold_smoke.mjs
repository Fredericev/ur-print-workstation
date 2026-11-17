#!/usr/bin/env node
/** Actual-app end-to-end test against a fresh, hardware-isolated DEMO service.
 * Never uses the operator's running service, ROS, UR, or Moonraker.
 */
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { createHash } from "node:crypto";
import { createServer } from "node:net";
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "../..");
const fixture = join(root, "Software/tests/fixtures/sample-print.gcode");
const output = join(root, "workspaces/ur_ros2_ws/runtime/diagnostics/full_cold_20260916");
const profile = mkdtempSync(join(tmpdir(), "ur-full-cold-browser-"));
const delay = ms => new Promise(done => setTimeout(done, ms));
const stationLog = [];
const browserEvents = [];
const requests = [];
const blockedRequests = [];
let station;
let chrome;
let cdp;

async function unusedPort() {
  const server = createServer();
  await new Promise((done, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", done);
  });
  const port = server.address().port;
  await new Promise(done => server.close(done));
  assert.notEqual(port, 8876, "Never use the real workstation port");
  return port;
}

async function waitForJson(url, label) {
  for (let i = 0; i < 120; i++) {
    try {
      const response = await fetch(url);
      if (response.ok) return await response.json();
    } catch { /* Fresh subprocess is still starting. */ }
    await delay(100);
  }
  throw new Error(`Timed out waiting for ${label}: ${url}`);
}

function connectCdp(url, permittedOrigin) {
  const socket = new WebSocket(url);
  const pending = new Map();
  let nextId = 0;
  const send = (method, params = {}) => new Promise((done, reject) => {
    const id = ++nextId;
    pending.set(id, { done, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
  socket.onmessage = ({ data }) => {
    const message = JSON.parse(data);
    if (message.id) {
      const waiter = pending.get(message.id);
      if (!waiter) return;
      pending.delete(message.id);
      if (message.error) waiter.reject(new Error(message.error.message));
      else waiter.done(message.result);
    } else if (message.method === "Fetch.requestPaused") {
      const { requestId, request } = message.params;
      const permitted = request.url.startsWith(`${permittedOrigin}/`)
        || request.url.startsWith("data:") || request.url === "about:blank";
      if (!permitted) blockedRequests.push(request.url);
      void send(permitted ? "Fetch.continueRequest" : "Fetch.failRequest",
        permitted ? { requestId } : { requestId, errorReason: "BlockedByClient" });
    } else if (message.method === "Network.requestWillBeSent") {
      requests.push(message.params.request);
    } else if (["Runtime.exceptionThrown", "Log.entryAdded", "Network.loadingFailed"].includes(message.method)) {
      browserEvents.push(message);
    }
  };
  const ready = new Promise((done, reject) => { socket.onopen = done; socket.onerror = reject; });
  const evaluate = async expression => {
    const result = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) {
      throw new Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
    }
    return result.result.value;
  };
  const waitFor = async (expression, label, attempts = 250) => {
    for (let i = 0; i < attempts; i++) {
      const value = await evaluate(expression);
      if (value) return value;
      await delay(100);
    }
    throw new Error(`Timed out waiting for ${label}`);
  };
  return { socket, ready, send, evaluate, waitFor };
}

async function stopOwnedProcess(process) {
  if (!process || process.exitCode !== null) return;
  const exited = new Promise(done => process.once("exit", done));
  process.kill("SIGTERM");
  const terminated = await Promise.race([exited.then(() => true), delay(3000).then(() => false)]);
  if (!terminated && process.exitCode === null) {
    process.kill("SIGKILL");
    await exited;
  }
}

function errors() {
  return browserEvents.filter(event => {
    if (event.method === "Log.entryAdded") {
      return event.params.entry.level === "error"
        && !event.params.entry.text.includes("GL Driver Message");
    }
    return event.method === "Runtime.exceptionThrown"
      || event.method === "Network.loadingFailed";
  });
}

async function isolatedApi(origin, endpoint, method = "GET", body = undefined) {
  assert.ok(origin.startsWith("http://127.0.0.1:"));
  assert.ok(!origin.endsWith(":8876"), "Never send test requests to real workstation");
  const response = await fetch(`${origin}${endpoint}`, {
    method,
    headers: { Origin: origin, "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await response.json();
  assert.ok(response.ok, `${endpoint}: ${JSON.stringify(data)}`);
  return data;
}

async function click(selector) {
  await cdp.waitFor(`Boolean(document.querySelector(${JSON.stringify(selector)}) && !document.querySelector(${JSON.stringify(selector)}).disabled)`, selector);
  await cdp.evaluate(`document.querySelector(${JSON.stringify(selector)}).click()`);
}

async function screenshot(name, width) {
  await cdp.send("Emulation.setDeviceMetricsOverride", { width, height: 1150, deviceScaleFactor: 1, mobile: false });
  await cdp.evaluate("document.fonts.ready.then(() => true)");
  await cdp.evaluate("new Promise(done => requestAnimationFrame(() => requestAnimationFrame(done)))");
  const captured = await cdp.send("Page.captureScreenshot", { format: "png", captureBeyondViewport: true });
  writeFileSync(join(output, `${name}-${width}.png`), Buffer.from(captured.data, "base64"));
}

mkdirSync(output, { recursive: true });
try {
  const stationPort = await unusedPort();
  const origin = `http://127.0.0.1:${stationPort}`;
  station = spawn(process.env.PYTHON_BIN || "python3", [
    join(root, "Software/control_station.py"), "--demo", "--no-browser", "--port", String(stationPort),
  ], { cwd: root, stdio: ["ignore", "pipe", "pipe"] });
  for (const stream of [station.stdout, station.stderr]) stream.on("data", chunk => stationLog.push(chunk.toString()));
  station.on("error", error => stationLog.push(error.message));
  const health = await waitForJson(`${origin}/api/health`, "fresh DEMO service");
  assert.equal(health.demo, true, "Hardware isolation is mandatory");

  // Seed only this fresh demo's installed tool0 setup, not any operator file.
  const content = readFileSync(fixture);
  const sourceSha = createHash("sha256").update(content).digest("hex");
  const parsedResponse = await fetch(`${origin}/api/gcode/parse`, {
    method: "POST", headers: { Origin: origin, "Content-Type": "application/octet-stream", "X-Gcode-Filename": "sample-print.gcode" }, body: content,
  });
  assert.ok(parsedResponse.ok);
  const parsedSeed = await parsedResponse.json();
  const calibration = (await isolatedApi(origin, "/api/gcode/calibration")).calibration;
  calibration.operation_mode = "tool0_motion_test";
  for (const flag of Object.keys(calibration.confirmations)) calibration.confirmations[flag] = true;
  calibration.confirmations.tcp_measured = false;
  const seeded = await isolatedApi(origin, "/api/gcode/calibration", "PUT", {
    calibration, bounds: parsedSeed.analysis?.bounds || parsedSeed.result?.bounds || parsedSeed.bounds,
    source_sha256: sourceSha,
  });
  assert.equal(seeded.saved, true);

  const debugPort = await unusedPort();
  chrome = spawn(process.env.CHROME_BIN || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", [
    "--headless=new", "--no-first-run", "--no-default-browser-check", "--enable-unsafe-swiftshader",
    "--use-gl=angle", "--use-angle=swiftshader", `--remote-debugging-port=${debugPort}`,
    `--user-data-dir=${profile}`, "about:blank",
  ], { stdio: "ignore" });
  const pages = await waitForJson(`http://127.0.0.1:${debugPort}/json/list`, "Chrome debugger");
  cdp = connectCdp(pages.find(page => page.type === "page").webSocketDebuggerUrl, origin);
  await cdp.ready;
  await Promise.all(["Page", "Runtime", "DOM", "Network", "Log"].map(domain => cdp.send(`${domain}.enable`)));
  await cdp.send("Fetch.enable", { patterns: [{ urlPattern: "*", requestStage: "Request" }] });
  await cdp.send("Emulation.setDeviceMetricsOverride", { width: 1440, height: 1150, deviceScaleFactor: 1, mobile: false });
  await cdp.send("Page.navigate", { url: `${origin}/` });
  await cdp.waitFor("document.readyState === 'complete' && typeof state !== 'undefined' && state.demo === true", "DEMO UI");
  await cdp.evaluate("changeOperatingMode('online')");
  await cdp.waitFor("state.operatingMode === 'online' && !state.modeChanging", "DEMO online mode");

  await cdp.evaluate("selectRunbook('gcode')");
  await cdp.waitFor("Boolean(document.querySelector('[data-import-mode=\"gcode\"]'))", "import mode chooser");
  await cdp.evaluate("document.querySelector('[data-import-mode=\"gcode\"]').click()");
  await cdp.waitFor("Boolean(document.querySelector('#gcodeFileInput'))", "file input");
  const documentNode = await cdp.send("DOM.getDocument", { depth: 1, pierce: true });
  const inputNode = await cdp.send("DOM.querySelector", { nodeId: documentNode.root.nodeId, selector: "#gcodeFileInput" });
  await cdp.send("DOM.setFileInputFiles", { nodeId: inputNode.nodeId, files: [fixture] });
  const parsed = await cdp.waitFor("!state.gcode.parsing && state.gcode.result && ({name:state.gcode.result.file.name,safeToPlan:state.gcode.result.safe_to_plan})", "import parse");
  assert.equal(parsed.safeToPlan, true);
  await click("#nextButton");
  const placement = await cdp.waitFor(`state.activeRunbook === 'workcell'
    && document.querySelector('#printZeroZMm')
    && state.workcell
    && ({tableZ:state.workcell.surface_z_m,printZeroZ:state.workcell.print_zero_z_m,
      candidate:state.workcell.print_z_candidate})`, "Print Z placement before motion");
  assert.ok(Number.isFinite(placement.tableZ), "Physical tabletop height must be loaded");
  assert.equal(placement.printZeroZ, placement.tableZ, "Unmodified print Z starts at the installed tabletop");
  assert.equal(placement.candidate, false, "The default placement must not create an unverified candidate");
  await cdp.evaluate("document.querySelector('#printZeroZMm').value = '20'");
  await click("#setPrintZDraftButton");
  const selectedZ = await cdp.waitFor("state.workcell && state.workcell.print_z_placement_mode === 'operator_selected' && Math.abs(state.workcell.print_zero_z_m - 0.020) < 0.000001 && ({table:state.workcell.surface_z_m,candidate:state.workcell.print_z_candidate})", "operator-selected elevated Print Z");
  assert.equal(selectedZ.table, placement.tableZ, "Selecting Print Z must not move the physical tabletop");
  assert.equal(selectedZ.candidate, false, "Elevated Print Z must not require the tabletop gap trial");
  await click("#discardPrintZDraftButton");
  await cdp.waitFor(`state.workcell && Math.abs(state.workcell.print_zero_z_m - ${placement.tableZ}) < 0.000001`, "reset Print Z to tabletop");
  await click("#nextButton");
  await cdp.waitFor("state.activeRunbook === 'motion'", "motion planning after Print Z placement");
  await click("#openModuleDependency");
  await cdp.waitFor("!state.gcode.planning && Boolean(state.gcode.plan)", "channel plan");
  await click("#openPathReviewFromPlan");
  await cdp.waitFor("!state.gcode.pathReviewLoading && Boolean(state.gcode.pathReview)", "3D review");
  // 3D Review automatically requests the demo MoveIt trajectory.
  await cdp.waitFor("!state.gcode.pathReviewMoveitRunning && moveitExecutionReady()", "complete demo MoveIt validation", 500);
  await cdp.evaluate("selectRunbook('execution')");
  await click("#executionExtentFull");
  await click("#prepareExecutionButton");
  const execution = await cdp.waitFor("!state.execution.preparing && state.execution.package && JSON.parse(JSON.stringify(state.execution.package))", "full immutable execution package", 500);
  assert.equal(execution.operation_mode, "tool0_motion_test");
  assert.equal(execution.execution_extent, "full");
  assert.equal(execution.execution_scope.type, "full_cold_motion_test");
  assert.equal(execution.execution_scope.end_progress, 1);
  for (const width of [1440, 1024]) await screenshot("full-package", width);

  await click("#nextButton");
  for (const selector of ["#executionEstop", "#executionClear", "#executionSpeed", "#executionWired", "#executionExternal"]) await click(selector);
  assert.equal(await cdp.evaluate("document.querySelectorAll('#confirmInput').length"), 0);
  await click("#moveToStartButton");
  // An immediate second call must not issue a second motion request.
  await cdp.evaluate("startExecutionMotion('move-to-start')");
  await cdp.waitFor("state.execution.runtime?.state === 'at-start' && !jobIsRunning()", "demo reviewed start");
  await click("#startPrintButton");
  await cdp.waitFor("state.execution.runtime?.state === 'completed' && !jobIsRunning()", "full cold execution completed", 500);
  const completed = await cdp.evaluate(`({
    currentBlock:state.execution.runtime.current_block,
    blockCount:state.execution.package.block_count,
    scope:state.execution.package.execution_scope,
    completeText:document.querySelector('.execution-complete-card')?.innerText || '',
    body:document.querySelector('.execution-page')?.innerText || '',
    hasTypedConfirmation:Boolean(document.querySelector('#confirmInput')),
    pageWidth:document.documentElement.scrollWidth,
  })`);
  assert.equal(completed.currentBlock, completed.blockCount);
  assert.equal(completed.hasTypedConfirmation, false);
  assert.ok(completed.completeText.length, "Full job completion card must be visible");
  assert.ok(!completed.body.includes("Full G-code was not run"), "Full completion must not use short-sample copy");
  for (const width of [1440, 1024]) await screenshot("full-completed", width);

  const postRequests = requests.filter(request => request.method === "POST" && request.url.startsWith(`${origin}/api/print/`));
  const prepare = postRequests.find(request => request.url.endsWith("/prepare"));
  assert.ok(prepare);
  assert.equal(Object.entries(prepare.headers).find(([name]) => name.toLowerCase() === "x-execution-extent")?.[1], "full");
  for (const [endpoint, action] of [["move-to-start", "move-to-start"], ["start", "execute"]]) {
    const submitted = postRequests.filter(request => request.url.endsWith(`/${endpoint}`));
    assert.equal(submitted.length, 1, `${action}: no double-click duplicates`);
    const body = JSON.parse(submitted[0].postData);
    assert.equal(body.operator_action, action);
    assert.ok(!Object.hasOwn(body, "confirmation"), "No typed phrase payload");
    for (const flag of ["emergency_stop", "workspace_clear", "reviewed_low_speed", "wired_network", "external_control_playing"]) assert.equal(body[flag], true);
  }
  assert.deepEqual(blockedRequests, [], "No attempt to contact hardware or other services");
  assert.deepEqual(errors(), [], "No browser exceptions or failed network requests");
  const summary = { isolation: "fresh DEMO subprocess; browser network restricted to its ephemeral origin", origin,
    parsed, executionId: execution.execution_id, scope: execution.execution_scope,
    completed, printActions: postRequests.map(request => ({ method: request.method, endpoint: new URL(request.url).pathname })),
    consoleErrors: errors(), blockedRequests };
  rmSync(join(output, "browser-full-cold-failure.json"), { force: true });
  writeFileSync(join(output, "browser-full-cold-summary.json"), JSON.stringify(summary, null, 2));
  console.log(JSON.stringify(summary, null, 2));
} catch (error) {
  const diagnostics = cdp ? await cdp.evaluate(`({
    body:document.body?.innerText?.slice(0,5000),
    executionError:typeof state==='undefined'?null:state.execution.error,
    reviewError:typeof state==='undefined'?null:state.gcode.pathReviewError,
    moveitError:typeof state==='undefined'?null:state.gcode.pathReviewMoveitError,
    extent:typeof state==='undefined'?null:state.execution.extent,
  })`).catch(() => null) : null;
  writeFileSync(join(output, "browser-full-cold-failure.json"), JSON.stringify({ error: error.stack, diagnostics, browserEvents, blockedRequests, stationLog }, null, 2));
  throw error;
} finally {
  cdp?.socket.close();
  await stopOwnedProcess(chrome);
  await stopOwnedProcess(station);
  // Exact, test-owned mkdtemp directory; never an operator workspace.
  rmSync(profile, { recursive: true, force: true });
}
