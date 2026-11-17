#!/usr/bin/env node
/** Verify the cold-test range selector with real translations and production CSS only. */
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { readFileSync, mkdtempSync, mkdirSync, writeFileSync, rmSync } from "node:fs";
import { createServer } from "node:http";
import { tmpdir } from "node:os";
import { resolve, dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "../..");
const source = readFileSync(join(root, "Software/app/app.js"), "utf8");
assert.match(source, /class="execution-extent-options"/);
assert.match(source, /class="execution-extent-option"/);

const context = vm.createContext({ state: { language: "en" } });
vm.runInContext(source.slice(source.indexOf("const translations ="), source.indexOf("const connectionSteps =")), context);
for (const name of ["t", "escapeHtml", "materialIcon"]) {
  const match = source.match(new RegExp(`function ${name}\\([^]*?\\n\\}`));
  assert.ok(match, name);
  vm.runInContext(match[0], context);
}

function selector(language, width) {
  context.state.language = language;
  const choice = (extent) => `<label class="execution-extent-option">
    <input id="executionExtent${extent === "full" ? "Full" : "Sample"}" type="radio" name="executionExtent" value="${extent}" ${extent === "full" ? "checked" : ""}>
    <span><strong>${context.escapeHtml(context.t(extent === "full" ? "executionExtentFull" : "executionExtentSample"))}</strong>
    <small>${context.escapeHtml(context.t(extent === "full" ? "executionExtentFullDesc" : "executionExtentSampleDesc"))}</small></span>
  </label>`;
  return `<section class="section execution-prepare-card" data-language="${language}" style="width:${width}px">
    <span class="module-hero-icon">${context.materialIcon("inventory_2")}</span>
    <h2>${context.escapeHtml(context.t("prepareExecution"))}</h2>
    <fieldset class="execution-extent"><legend>${context.escapeHtml(context.t("executionExtent"))}</legend>
      <div class="execution-extent-options">${choice("full")}${choice("sample")}</div>
    </fieldset>
    <p>${context.escapeHtml(context.t("fullColdBoundary"))}</p>
    <button class="button primary" type="button">${context.materialIcon("lock")} ${context.escapeHtml(context.t("prepareExecution"))}</button>
  </section>`;
}

const server = createServer((request, response) => {
  const url = new URL(request.url, "http://127.0.0.1");
  if (url.pathname === "/styles.css") {
    response.setHeader("Content-Type", "text/css");
    response.end(readFileSync(join(root, "Software/app/styles.css")));
  } else if (url.pathname === "/assets/MaterialSymbolsOutlined.woff2") {
    response.setHeader("Content-Type", "font/woff2");
    response.end(readFileSync(join(root, "Software/app/assets/MaterialSymbolsOutlined.woff2")));
  } else if (url.pathname === "/") {
    const language = url.searchParams.get("language") === "zh" ? "zh" : "en";
    const width = Number(url.searchParams.get("width") || 640);
    response.setHeader("Content-Type", "text/html; charset=utf-8");
    response.end(`<!doctype html><html lang="${language}"><head><link rel="stylesheet" href="/styles.css">
      <style>body{min-width:0;margin:0;padding:24px;background:#eef2f7}.execution-page{width:${width}px;max-width:calc(100vw - 48px)}</style>
      </head><body><main class="execution-page">${selector(language, width)}</main></body></html>`);
  } else {
    response.statusCode = 404;
    response.end();
  }
});
await new Promise((done, reject) => { server.once("error", reject); server.listen(0, "127.0.0.1", done); });

const debugPortServer = createServer();
await new Promise(done => debugPortServer.listen(0, "127.0.0.1", done));
const debugPort = debugPortServer.address().port;
await new Promise(done => debugPortServer.close(done));
const profile = mkdtempSync(join(tmpdir(), "ur-execution-extent-"));
const chrome = spawn(process.env.CHROME_BIN || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  ["--headless=new", "--no-first-run", "--no-default-browser-check", `--remote-debugging-port=${debugPort}`,
    `--user-data-dir=${profile}`, "about:blank"], { stdio: "ignore" });
const delay = ms => new Promise(done => setTimeout(done, ms));
const output = resolve(process.argv[2] || join(root, "workspaces/ur_ros2_ws/runtime/diagnostics/execution_extent_layout_20260916"));
mkdirSync(output, { recursive: true });
let socket;
try {
  let pages;
  for (let i = 0; i < 100; i += 1) {
    try { pages = await (await fetch(`http://127.0.0.1:${debugPort}/json/list`)).json(); break; }
    catch { await delay(100); }
  }
  assert.ok(pages, "Chrome started");
  socket = new WebSocket(pages.find(page => page.type === "page").webSocketDebuggerUrl);
  const pending = new Map();
  let next = 0;
  socket.onmessage = ({ data }) => {
    const message = JSON.parse(data);
    if (!message.id) return;
    const task = pending.get(message.id);
    pending.delete(message.id);
    if (message.error) task.reject(new Error(message.error.message));
    else task.resolve(message.result);
  };
  await new Promise((done, reject) => { socket.onopen = done; socket.onerror = reject; });
  const send = (method, params = {}) => new Promise((done, reject) => {
    const id = ++next;
    pending.set(id, { resolve: done, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async expression => {
    const result = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
    return result.result.value;
  };
  await send("Page.enable");
  for (const language of ["en", "zh"]) for (const width of [360, 640]) {
    await send("Emulation.setDeviceMetricsOverride", { width: 1024, height: 720, deviceScaleFactor: 1, mobile: false });
    await send("Page.navigate", { url: `http://127.0.0.1:${server.address().port}/?width=${width}&language=${language}` });
    for (let i = 0; i < 100; i += 1) {
      if (await evaluate("document.readyState === 'complete' && !!document.querySelector('.execution-extent')")) break;
      await delay(50);
    }
    await evaluate("document.fonts.ready.then(() => true)");
    await evaluate("new Promise(done => requestAnimationFrame(() => requestAnimationFrame(done)))");
    const report = await evaluate(`(() => {
      const root = document.querySelector('.execution-prepare-card');
      const options = [...document.querySelectorAll('.execution-extent-option')];
      const radios = [...document.querySelectorAll('.execution-extent input[type="radio"]')];
      const rect = element => element.getBoundingClientRect();
      const intersects = (a,b) => a.left < b.right-.5 && b.left < a.right-.5 && a.top < b.bottom-.5 && b.top < a.bottom-.5;
      return {
        overflow: root.scrollWidth > root.clientWidth + 1 || [...root.querySelectorAll('*')].some(el => el.scrollWidth > el.clientWidth + 1),
        radioSizes: radios.map(item => [rect(item).width, rect(item).height]),
        textOverlapsRadio: options.some(item => intersects(rect(item.querySelector('input')), rect(item.querySelector('span')))),
        optionsOverlap: intersects(rect(options[0]), rect(options[1])),
        sameRow: Math.abs(rect(options[0]).top - rect(options[1]).top) < 2,
        labels: options.map(item => item.innerText.trim()),
      };
    })()`);
    assert.equal(report.overflow, false, `${language}/${width}: overflow`);
    assert.deepEqual(report.radioSizes, [[18, 18], [18, 18]], `${language}/${width}: radio size`);
    assert.equal(report.textOverlapsRadio, false, `${language}/${width}: text/radio overlap`);
    assert.equal(report.optionsOverlap, false, `${language}/${width}: option overlap`);
    assert.equal(report.sameRow, width === 640, `${language}/${width}: responsive columns`);
    assert.ok(report.labels.every(label => label.length < 40), `${language}/${width}: copy should stay concise`);
    const screenshot = await send("Page.captureScreenshot", { format: "png", captureBeyondViewport: false });
    writeFileSync(join(output, `extent-${language}-${width}.png`), Buffer.from(screenshot.data, "base64"));
    console.log(`${language}/${width}px: compact selector has no overlap`);
  }
} finally {
  socket?.close();
  await new Promise(done => { if (chrome.exitCode !== null) done(); else { chrome.once("exit", done); chrome.kill("SIGTERM"); } });
  rmSync(profile, { recursive: true, force: true });
  await new Promise(done => server.close(done));
}
