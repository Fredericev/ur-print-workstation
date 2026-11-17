#!/usr/bin/env node
/** Fresh DEMO server only: exercise paired import and original-flow switching. */
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { createServer } from "node:net";
import { mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "../..");
const owned = mkdtempSync(join(tmpdir(), "ur-paired-import-"));
const pause = ms => new Promise(done => setTimeout(done, ms));
let serverProcess;
let chrome;
let socket;

async function unusedPort() {
  const server = createServer();
  await new Promise((done, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", done);
  });
  const port = server.address().port;
  await new Promise(done => server.close(done));
  assert.notEqual(port, 8876);
  return port;
}

async function waitForJson(url) {
  for (let index = 0; index < 100; index += 1) {
    try {
      const response = await fetch(url);
      if (response.ok) return response.json();
    } catch { /* Test-owned process is starting. */ }
    await pause(100);
  }
  throw new Error(`Timed out waiting for ${url}`);
}

async function stopOwned(process) {
  if (!process || process.exitCode !== null) return;
  const exited = new Promise(done => process.once("exit", done));
  process.kill("SIGTERM");
  if (!await Promise.race([exited.then(() => true), pause(3000).then(() => false)])) {
    process.kill("SIGKILL");
    await exited;
  }
}

function connectCdp(url) {
  const pending = new Map();
  let nextId = 0;
  const browser = new WebSocket(url);
  const ready = new Promise((done, reject) => {
    browser.onopen = done;
    browser.onerror = reject;
  });
  browser.onmessage = ({ data }) => {
    const message = JSON.parse(data);
    if (!message.id) return;
    const waiter = pending.get(message.id);
    if (!waiter) return;
    pending.delete(message.id);
    if (message.error) waiter.reject(new Error(message.error.message));
    else waiter.resolve(message.result);
  };
  const send = (method, params = {}) => new Promise((done, reject) => {
    const id = ++nextId;
    pending.set(id, { resolve: done, reject });
    browser.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async expression => {
    const result = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.text);
    return result.result?.value;
  };
  const waitFor = async (expression, label) => {
    for (let index = 0; index < 100; index += 1) {
      const value = await evaluate(expression);
      if (value) return value;
      await pause(100);
    }
    throw new Error(`Timed out waiting for ${label}`);
  };
  return { browser, ready, send, evaluate, waitFor };
}

try {
  const originPort = await unusedPort();
  const origin = `http://127.0.0.1:${originPort}`;
  serverProcess = spawn(process.env.PYTHON_BIN || "python3", [
    join(root, "Software/control_station.py"), "--demo", "--no-browser", "--port", String(originPort),
  ], {
    cwd: root,
    env: { ...process.env, UR_PRINT_DATA_DIR: join(owned, "state") },
    stdio: "ignore",
  });
  assert.equal((await waitForJson(`${origin}/api/health`)).demo, true);

  const trajectoryPath = join(owned, "route.csv");
  const extrusionPath = join(owned, "extrusion.gcode");
  const originalPath = join(owned, "original.gcode");
  writeFileSync(trajectoryPath,
    "time_s,path_progress,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint\n"
    + "0,0,0,0,0,0,0,0\n1,1,0.02,0,0,0,0,0\n");
  writeFileSync(extrusionPath, "G21\nM83\nG1 E0.3 ;@sync start=0 end=1\n");
  writeFileSync(originalPath, "G21\nG90\nM83\nG1 X0 Y0 Z0 E0.1 F600\nG1 X10 Y0 E0.2 F600\n");

  const debugPort = await unusedPort();
  chrome = spawn(process.env.CHROME_BIN || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", [
    "--headless=new", "--no-first-run", "--no-default-browser-check", "--enable-unsafe-swiftshader",
    "--use-gl=angle", "--use-angle=swiftshader", `--remote-debugging-port=${debugPort}`,
    `--user-data-dir=${join(owned, "chrome")}`, "about:blank",
  ], { stdio: "ignore" });
  const pages = await waitForJson(`http://127.0.0.1:${debugPort}/json/list`);
  const page = pages.find(item => item.type === "page");
  assert.ok(page);
  const cdp = connectCdp(page.webSocketDebuggerUrl);
  socket = cdp.browser;
  await cdp.ready;
  await Promise.all(["Page", "Runtime", "DOM"].map(domain => cdp.send(`${domain}.enable`)));
  await cdp.send("Page.navigate", { url: `${origin}/` });
  await cdp.waitFor("document.readyState === 'complete' && typeof state !== 'undefined'", "application");
  await cdp.evaluate("selectRunbook('gcode')");
  await cdp.waitFor("Boolean(document.querySelector('[data-import-mode=\"paired\"]'))", "mode choice");
  await cdp.evaluate("document.querySelector('[data-import-mode=\"paired\"]').click()");
  await cdp.waitFor("Boolean(document.querySelector('#pairedTrajectoryFile'))", "paired inputs");
  const fileControls = await cdp.evaluate(`({
    hidden: ['#pairedTrajectoryFile', '#pairedExtrusionFile'].every(selector => document.querySelector(selector).hidden),
    buttons: [...document.querySelectorAll('#choosePairedTrajectory, #choosePairedExtrusion')].map(button => button.textContent.trim()),
    names: [...document.querySelectorAll('.paired-file-name')].map(name => name.textContent.trim()),
  })`);
  assert.equal(fileControls.hidden, true, "native localized file inputs must not be visible");
  assert.deepEqual(fileControls.buttons, ["Choose file", "Choose file"]);
  assert.deepEqual(fileControls.names, ["No file selected", "No file selected"]);
  for (const [selector, path] of [["#pairedTrajectoryFile", trajectoryPath], ["#pairedExtrusionFile", extrusionPath]]) {
    const documentNode = await cdp.send("DOM.getDocument", { depth: 1, pierce: true });
    const input = await cdp.send("DOM.querySelector", { nodeId: documentNode.root.nodeId, selector });
    assert.ok(input.nodeId);
    await cdp.send("DOM.setFileInputFiles", { nodeId: input.nodeId, files: [path] });
  }
  await cdp.evaluate("document.querySelector('#checkPairedImport').click()");
  const loaded = await cdp.waitFor("state.pairedImport.result && !state.pairedImport.checking && ({pointCount:state.pairedImport.result.trajectory.point_count,locked:document.querySelector('#nextButton').disabled,oldResult:state.gcode.result})", "paired import");
  assert.equal(loaded.pointCount, 2);
  assert.equal(loaded.locked, true);
  assert.equal(loaded.oldResult, null);
  const chosenNames = await cdp.evaluate("[...document.querySelectorAll('.paired-file-name')].map(name => name.textContent.trim())");
  assert.deepEqual(chosenNames, ["route.csv", "extrusion.gcode"]);
  await cdp.evaluate("document.querySelector('[data-import-mode=\"gcode\"]').click()");
  const reset = await cdp.waitFor("state.gcode.importMode === 'gcode' && Boolean(document.querySelector('#gcodeFileInput')) && ({oldPair:state.pairedImport.result,oldSource:state.gcode.sourceFile})", "original mode");
  assert.equal(reset.oldPair, null);
  assert.equal(reset.oldSource, null);
  const documentNode = await cdp.send("DOM.getDocument", { depth: 1, pierce: true });
  const originalInput = await cdp.send("DOM.querySelector", {
    nodeId: documentNode.root.nodeId, selector: "#gcodeFileInput",
  });
  await cdp.send("DOM.setFileInputFiles", { nodeId: originalInput.nodeId, files: [originalPath] });
  const original = await cdp.waitFor("state.gcode.result && !state.gcode.parsing && ({name:state.gcode.result.file.name,ready:state.gcode.result.safe_to_plan,pair:state.pairedImport.result})", "original G-code import");
  assert.equal(original.name, "original.gcode");
  assert.equal(original.ready, true);
  assert.equal(original.pair, null);
  console.log("Fresh DEMO browser paired import and mode isolation: OK");
} finally {
  socket?.close();
  await stopOwned(chrome);
  await stopOwned(serverProcess);
  rmSync(owned, { recursive: true, force: true });
}
