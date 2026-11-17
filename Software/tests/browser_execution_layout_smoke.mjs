#!/usr/bin/env node
/** Render the real execution components/CSS without any workstation API. */
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
const context = vm.createContext({ state: { language: "en", execution: {} } });
vm.runInContext(source.slice(source.indexOf("const translations ="), source.indexOf("const connectionSteps =")), context);
for (const name of ["t", "escapeHtml", "materialIcon", "liveHealthCard", "executionRuntimeMarkup"]) {
  const match = source.match(new RegExp(`function ${name}\\([^]*?\\n\\}`));
  assert.ok(match, name);
  vm.runInContext(match[0], context);
}
const preflight = { program_state_received: true, external_control_running: true,
  joint_state_fresh: true, klipper_action_server: true, trajectory_action_server: true };
const modes = ["preflight", "running", "completed", "failed"];
function component(language, mode, width) {
  context.state.language = language;
  context.state.execution = {
    package: { operation_mode: "tool0_motion_test", block_count: 1 },
    runtime: { state: mode, mode: "move-to-start", preflight,
      progress: mode === "running" ? .89 : mode === "completed" ? 1 : 0,
      current_block: mode === "completed" ? 1 : 0, block_count: 1,
      message: mode === "preflight" ? "Checking devices" : "A deliberately long backend diagnostic that must not crowd the running summary",
      motion: { source: "live_joint_state", nominal_remaining_s: 16 },
      root_error: { message: "Klipper motion failed: Moonraker returned HTTP 504 for /printer/gcode/script" } },
  };
  const health = context.liveHealthCard("precision_manufacturing", context.t("urDriver"), true, "UR")
    + context.liveHealthCard("cable", context.t("externalControl"), true, context.t("healthExternalShort"))
    + context.liveHealthCard("route", context.t("moveit"), true)
    + context.liveHealthCard("lan", context.t("klipperBridge"), true, "Klipper");
  return `<article class="fixture" data-language="${language}" data-mode="${mode}" style="width:${width}px">
    <h2>${language} · ${mode} · ${width}px</h2>
    <section class="info-panel"><div class="info-panel-header"><strong class="panel-title">${context.materialIcon("hub")} ${context.escapeHtml(context.t("systemHealthTitle"))}</strong></div>
    <div class="info-panel-body module-health-grid compact">${health}</div></section>
    ${context.executionRuntimeMarkup()}</article>`;
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
    const width = Number(url.searchParams.get("width") || 320);
    const language = url.searchParams.get("language") || "en";
    response.setHeader("Content-Type", "text/html; charset=utf-8");
    response.end(`<!doctype html><html lang="${language}"><head><link rel="stylesheet" href="/styles.css">
    <style>body{min-width:0;padding:20px;background:#f6f7f8}.fixtures{display:flex;gap:20px;align-items:flex-start}.fixture{flex:none}.fixture h2{font-size:13px;color:#727c86}.fixture>.execution-runtime-card{margin-top:16px}</style>
    </head><body><main class="fixtures">${modes.map(mode => component(language, mode, width)).join("")}</main></body></html>`);
  } else { response.statusCode = 404; response.end(); }
});
await new Promise((done, reject) => { server.once("error", reject); server.listen(0, "127.0.0.1", done); });
const portServer = createServer();
await new Promise(done => portServer.listen(0, "127.0.0.1", done));
const debugPort = portServer.address().port;
await new Promise(done => portServer.close(done));
const profile = mkdtempSync(join(tmpdir(), "ur-layout-smoke-"));
const chrome = spawn(process.env.CHROME_BIN || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  ["--headless=new", "--no-first-run", "--no-default-browser-check",
    `--remote-debugging-port=${debugPort}`, `--user-data-dir=${profile}`, "about:blank"], { stdio: "ignore" });
const delay = ms => new Promise(done => setTimeout(done, ms));
let socket;
const output = resolve(process.argv[2] || join(root, "workspaces/ur_ros2_ws/runtime/diagnostics/ui_speed_20260916"));
mkdirSync(output, { recursive: true });
try {
  let pages;
  for (let i = 0; i < 100; i++) {
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
    const id = ++next; pending.set(id, { resolve: done, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
  const evaluate = async expression => {
    const result = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
    return result.result.value;
  };
  await send("Page.enable");
  for (const language of ["en", "zh"]) for (const width of [320, 640]) {
    await send("Emulation.setDeviceMetricsOverride", { width: 4*(width+20)+20, height: 1250, deviceScaleFactor: 1, mobile: false });
    await send("Page.navigate", { url: `http://127.0.0.1:${server.address().port}/?width=${width}&language=${language}` });
    for (let i = 0; i < 100; i++) {
      if (await evaluate("document.querySelectorAll('.fixture').length === 4 && document.readyState === 'complete'")) break;
      await delay(50);
    }
    await evaluate("document.fonts.ready.then(() => true)");
    await evaluate("new Promise(done => requestAnimationFrame(() => requestAnimationFrame(done)))");
    const issues = await evaluate(`(() => {
      const problems = [];
      const rect = element => element.getBoundingClientRect();
      const intersects = (a,b) => a.left < b.right-.5 && b.left < a.right-.5 && a.top < b.bottom-.5 && b.top < a.bottom-.5;
      for (const fixture of document.querySelectorAll('.fixture')) {
        const name = fixture.dataset.mode;
        for (const element of fixture.querySelectorAll('.module-health-card,.execution-runtime-card,.execution-runtime-heading')) {
          if (element.scrollWidth > element.clientWidth+1) problems.push(name+': horizontal overflow '+element.className);
        }
        for (const card of fixture.querySelectorAll('.module-health-card')) {
          if (intersects(rect(card.querySelector('strong')),rect(card.querySelector('small')))) problems.push(name+': health label/status overlap');
          if (!card.getAttribute('aria-label')) problems.push(name+': missing full health label');
        }
        const heading = fixture.querySelector('.execution-runtime-heading');
        if (intersects(rect(heading.children[1]),rect(heading.lastElementChild))) problems.push(name+': percentage overlaps summary');
        const caption = fixture.querySelector('.execution-preflight-caption');
        if (caption && rect(caption).bottom > rect(fixture.querySelector('.execution-preflight-grid')).top+.5) problems.push(name+': preflight caption overlaps checks');
        if (name !== 'preflight' && caption) problems.push(name+': historical start-check snapshot remains');
      }
      return problems;
    })()`);
    assert.deepEqual(issues, [], `${language}/${width}`);
    await evaluate("document.querySelectorAll('details').forEach(item => item.open = true)");
    const screenshot = await send("Page.captureScreenshot", { format: "png", captureBeyondViewport: true });
    writeFileSync(join(output, `execution-${language}-${width}.png`), Buffer.from(screenshot.data, "base64"));
    console.log(`${language}/${width}px: four states, no overlaps; screenshot saved`);
  }
} finally {
  socket?.close();
  await new Promise(done => { if (chrome.exitCode !== null) done(); else { chrome.once("exit", done); chrome.kill("SIGTERM"); } });
  rmSync(profile, { recursive: true, force: true });
  await new Promise(done => server.close(done));
}
