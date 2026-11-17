#!/usr/bin/env node
/** End-to-end, read-only browser smoke test for Motion Planning and 3D Review. */

import { spawn } from "node:child_process";
import { createHash } from "node:crypto";
import { mkdtempSync, readFileSync, rmSync } from "node:fs";
import { createServer } from "node:net";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const PROJECT_ROOT = resolve(HERE, "../..");
const GCODE_PATH = resolve(
  process.argv[2] || join(PROJECT_ROOT, "Test gcode", "CCR10_mold2.gcode"),
);
const GCODE_SHA256 = createHash("sha256").update(readFileSync(GCODE_PATH)).digest("hex");
const STATION_URL = process.env.UR_PRINT_STATION_URL || "http://127.0.0.1:8876/";
const CHROME_PATH = process.env.CHROME_BIN
  || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";
const PROFILE_PATH = mkdtempSync(join(tmpdir(), "ur-print-chrome-"));

const sleep = (milliseconds) => new Promise((resolvePromise) => {
  setTimeout(resolvePromise, milliseconds);
});

async function unusedPort() {
  const server = createServer();
  await new Promise((resolvePromise, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", resolvePromise);
  });
  const { port } = server.address();
  await new Promise((resolvePromise) => server.close(resolvePromise));
  return port;
}

async function waitForJson(url, attempts = 100) {
  for (let attempt = 0; attempt < attempts; attempt += 1) {
    try {
      const response = await fetch(url);
      if (response.ok) return await response.json();
    } catch {
      // Chrome is still starting.
    }
    await sleep(100);
  }
  throw new Error(`Chrome DevTools did not start: ${url}`);
}

function connectCdp(webSocketUrl) {
  const socket = new WebSocket(webSocketUrl);
  const pending = new Map();
  const events = [];
  let nextId = 1;

  socket.onmessage = ({ data }) => {
    const message = JSON.parse(data);
    if (message.id) {
      const waiter = pending.get(message.id);
      if (!waiter) return;
      pending.delete(message.id);
      if (message.error) waiter.reject(new Error(message.error.message));
      else waiter.resolve(message.result);
      return;
    }
    if (["Runtime.exceptionThrown", "Log.entryAdded", "Network.loadingFailed"].includes(message.method)) {
      events.push(message);
    }
  };

  const ready = new Promise((resolvePromise, reject) => {
    socket.onopen = resolvePromise;
    socket.onerror = reject;
  });

  function send(method, params = {}) {
    const id = nextId;
    nextId += 1;
    socket.send(JSON.stringify({ id, method, params }));
    return new Promise((resolvePromise, reject) => {
      pending.set(id, { resolve: resolvePromise, reject });
    });
  }

  async function evaluate(expression) {
    const result = await send("Runtime.evaluate", {
      expression,
      awaitPromise: true,
      returnByValue: true,
    });
    if (result.exceptionDetails) {
      throw new Error(
        result.exceptionDetails.exception?.description
          || result.exceptionDetails.text,
      );
    }
    return result.result.value;
  }

  async function waitFor(expression, label, attempts = 160) {
    for (let attempt = 0; attempt < attempts; attempt += 1) {
      const value = await evaluate(expression);
      if (value) return value;
      await sleep(100);
    }
    throw new Error(`Timed out waiting for ${label}`);
  }

  return { socket, events, ready, send, evaluate, waitFor };
}

function browserErrors(events) {
  return events
    .map((event) => ({
      method: event.method,
      text: event.params?.exceptionDetails?.exception?.description
        || event.params?.entry?.text
        || event.params?.errorText
        || "",
      url: event.params?.entry?.url || event.params?.requestId || "",
    }))
    .filter((event) => !event.text.includes("GL Driver Message"));
}

const port = await unusedPort();
const chrome = spawn(
  CHROME_PATH,
  [
    "--headless=new",
    "--no-first-run",
    "--no-default-browser-check",
    "--enable-unsafe-swiftshader",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    `--remote-debugging-port=${port}`,
    `--user-data-dir=${PROFILE_PATH}`,
    "about:blank",
  ],
  { stdio: "ignore" },
);

let cdp;
try {
  await waitForJson(`http://127.0.0.1:${port}/json/version`);
  const pages = await waitForJson(`http://127.0.0.1:${port}/json/list`);
  const page = pages.find((item) => item.type === "page");
  if (!page) throw new Error("Chrome DevTools page target was not found");
  cdp = connectCdp(page.webSocketDebuggerUrl);
  await cdp.ready;
  await Promise.all([
    cdp.send("Page.enable"),
    cdp.send("Runtime.enable"),
    cdp.send("DOM.enable"),
    cdp.send("Log.enable"),
    cdp.send("Network.enable"),
  ]);
  await cdp.send("Page.navigate", { url: STATION_URL });
  await cdp.waitFor("document.readyState === 'complete'", "application load");
  await cdp.waitFor(
    "typeof window.URPathReview3D?.mount === 'function'",
    "3D module",
  );

  await cdp.evaluate("selectRunbook('gcode')");
  await cdp.waitFor("Boolean(document.querySelector('[data-import-mode=\"gcode\"]'))", "import mode chooser");
  await cdp.evaluate("document.querySelector('[data-import-mode=\"gcode\"]').click()");
  await cdp.waitFor(
    "Boolean(document.querySelector('#gcodeFileInput'))",
    "G-code input",
  );
  const documentNode = await cdp.send("DOM.getDocument", {
    depth: 1,
    pierce: true,
  });
  const inputNode = await cdp.send("DOM.querySelector", {
    nodeId: documentNode.root.nodeId,
    selector: "#gcodeFileInput",
  });
  if (!inputNode.nodeId) throw new Error("G-code input node was not found");
  await cdp.send("DOM.setFileInputFiles", {
    nodeId: inputNode.nodeId,
    files: [GCODE_PATH],
  });

  const parsed = await cdp.waitFor(
    `!state.gcode.parsing && state.gcode.result && ({
      name: state.gcode.result.file.name,
      safeToPlan: state.gcode.result.safe_to_plan
    })`,
    "G-code parse",
  );
  await cdp.evaluate("document.querySelector('#nextButton').click()");
  // The current workflow visits installation setup before Motion Planning.
  await cdp.waitFor("state.activeRunbook === 'workcell' && state.workcell", "Workcell setup");
  await cdp.waitFor("!document.querySelector('#nextButton').disabled", "saved Print Z");
  await cdp.evaluate("document.querySelector('#nextButton').click()");
  await cdp.waitFor(
    "Boolean(document.querySelector('#openModuleDependency'))",
    "Motion Planning entry",
  );
  await cdp.evaluate("document.querySelector('#openModuleDependency').click()");
  const planned = await cdp.waitFor(
    `!state.gcode.planning && state.gcode.plan && ({
      waypoints: state.gcode.plan.ur_robot.waypoint_count,
      sourceEvents: state.gcode.plan.synchronization.source_event_count
    })`,
    "channel plan",
  );
  await cdp.waitFor(
    "Boolean(document.querySelector('#openPathReviewFromPlan'))",
    "3D Review button",
  );
  await cdp.evaluate("document.querySelector('#openPathReviewFromPlan').click()");
  const reviewed = await cdp.waitFor(
    `!state.gcode.pathReviewLoading && state.gcode.pathReview && ({
      frame: state.gcode.pathReview.coordinate_frame,
      sourceSha256: state.gcode.pathReview.source_sha256,
      segmentCount: state.gcode.pathReview.path.segments.length,
      printCenter: state.gcode.pathReview.placement_model.print_center_m,
      hasWorkEnvelope: Object.prototype.hasOwnProperty.call(
        state.gcode.pathReview,
        'work_envelope_m'
      )
    })`,
    "3D path review",
    1200,
  );
  const rendered = await cdp.waitFor(
    `state.gcode.pathReviewViewer
      && document.querySelector('#pathReviewViewport canvas')
      && ({
        canvas: true,
        worktable: state.gcode.pathReviewViewer.worktable?.name,
        hasEnvelopeObject: Boolean(state.gcode.pathReviewViewer.envelope),
        tableGeometry:
          state.gcode.pathReviewViewer.worktable?.children?.[0]?.geometry?.type,
        pathObjectCount: state.gcode.pathReviewViewer.pathObjects?.length
      })`,
    "WebGL viewer mount",
  );

  // A disconnected workstation must still let the operator inspect and play
  // the planned nozzle route when neither IK nor MoveIt has joint motion.
  await cdp.evaluate(`(() => {
    state.operatingMode = 'offline';
    state.gcode.pathReviewSimulationResult = null;
    state.gcode.pathReviewMoveitResult = null;
    render();
    return true;
  })()`);
  await cdp.waitFor(
    "state.gcode.pathReviewViewer && state.gcode.pathReviewViewer.jointNodes.length === 6",
    "offline path viewer",
  );
  // Wait for the actual configured CAD asset, including its asynchronous load.
  const printhead = await cdp.waitFor(`(() => {
    const meshes = [];
    state.gcode.pathReviewViewer?.robotRoot?.traverse(object => {
      if (object.isMesh && object.geometry?.attributes?.position?.count > 100000) {
        meshes.push({ name: object.name, vertices: object.geometry.attributes.position.count });
      }
    });
    return meshes.length ? { meshes, source: state.workcell?.nozzle?.source } : null;
  })()`, "new printhead CAD mesh", 600);
  if (printhead.source !== 'NEW_ExtruderAssembly.STEP') {
    throw new Error(`Wrong printhead loaded: ${JSON.stringify(printhead)}`);
  }
  const routeControls = await cdp.evaluate(`({
    playEnabled: !document.querySelector('#pathReviewPlayButton')?.disabled,
    restartEnabled: !document.querySelector('#pathReviewRestartButton')?.disabled,
    scrubEnabled: !document.querySelector('#pathReviewProgress')?.disabled,
    segmentCount: state.gcode.pathReview.path.segments.length,
    jointPositions: state.gcode.pathReviewViewer.jointNodes.map(node => node.rotation.z)
  })`);
  if (!routeControls.playEnabled || !routeControls.restartEnabled || !routeControls.scrubEnabled) {
    throw new Error(`Offline route controls are disabled: ${JSON.stringify(routeControls)}`);
  }
  await cdp.evaluate("document.querySelector('#pathReviewPlayButton').click()");
  await cdp.waitFor("state.gcode.pathReviewProgress > 0", "offline route playback");
  const played = await cdp.evaluate(`({
    progress: state.gcode.pathReviewProgress,
    jointPositions: state.gcode.pathReviewViewer.jointNodes.map(node => node.rotation.z)
  })`);
  if (JSON.stringify(played.jointPositions) !== JSON.stringify(routeControls.jointPositions)) {
    throw new Error("Arm joints moved without an offline or MoveIt trajectory");
  }
  await cdp.evaluate("document.querySelector('#pathReviewPlayButton').click()");
  const scrubbed = await cdp.evaluate(`(() => {
    const slider = document.querySelector('#pathReviewProgress');
    slider.value = '500';
    slider.dispatchEvent(new Event('input', { bubbles: true }));
    return state.gcode.pathReviewProgress;
  })()`);
  if (Math.abs(scrubbed - 0.5) > 0.001) {
    throw new Error(`Offline route scrub did not move to 50%: ${scrubbed}`);
  }
  await cdp.evaluate("document.querySelector('#pathReviewRestartButton').click()");
  const restarted = await cdp.evaluate("state.gcode.pathReviewProgress");
  if (restarted !== 0) throw new Error(`Offline route restart did not reset progress: ${restarted}`);

  const errors = browserErrors(cdp.events);
  if (!parsed.safeToPlan) throw new Error("G-code was not safe to plan");
  if (reviewed.sourceSha256 !== GCODE_SHA256) {
    throw new Error(`3D review source mismatch: ${reviewed.sourceSha256}`);
  }
  if (reviewed.hasWorkEnvelope || rendered.hasEnvelopeObject) {
    throw new Error("Removed Work Envelope is still present");
  }
  if (rendered.tableGeometry !== "PlaneGeometry") {
    throw new Error(`Unknown table thickness must render as a plane, got ${rendered.tableGeometry}`);
  }
  if (errors.length) {
    throw new Error(`Browser errors: ${JSON.stringify(errors)}`);
  }
  console.log(JSON.stringify({ parsed, planned, reviewed, rendered, printhead, routeControls, played, scrubbed, restarted }, null, 2));
} catch (error) {
  if (cdp) {
    const diagnostics = await cdp.evaluate(`({
      title: document.title,
      body: document.body?.innerText?.slice(0, 1200),
      reviewLoading: typeof state === 'undefined' ? null : state.gcode.pathReviewLoading,
      reviewError: typeof state === 'undefined' ? null : state.gcode.pathReviewError,
      viewerReady: typeof window.URPathReview3D?.mount,
      canvasCount: document.querySelectorAll('canvas').length
    })`).catch(() => null);
    console.error(JSON.stringify({ diagnostics, browserErrors: browserErrors(cdp.events) }, null, 2));
  }
  throw error;
} finally {
  cdp?.socket.close();
  const chromeExited = chrome.exitCode === null
    ? new Promise(done => chrome.once("exit", done)) : Promise.resolve();
  chrome.kill("SIGTERM");
  await Promise.race([chromeExited, sleep(1500)]);
  if (chrome.exitCode === null) chrome.kill("SIGKILL");
  await chromeExited;
  rmSync(PROFILE_PATH, { recursive: true, force: true, maxRetries: 10, retryDelay: 100 });
}
