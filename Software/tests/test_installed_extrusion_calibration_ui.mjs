// Exercise the real installation form and save handler with a local API stub.
// No DOM browser, robot process, network connection or motor command is used.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const renderer = source.match(/function renderWorkcellModule\(\) \{[^]*?\n\}/)?.[0];
assert.ok(renderer);
const mainContent = { innerHTML: "" };
const listeners = new Map();
const inputs = Object.fromEntries([
  "nozzleTcpX", "nozzleTcpY", "nozzleTcpZ",
  "installedExtrusionRatio", "installedExtrusionDirection",
].map(id => [id, { value: "" }]));
const calls = [];
const notifications = [];
let invalidations = 0;
const state = {
  workcell: {
    surface_z_m: -0.014, draft_surface_z_m: null,
    print_zero_z_m: -0.014, print_z_placement_mode: "operator_selected",
    nozzle: { tool0_to_nozzle_m: { x: 0.03255, y: 0.0344, z: 0.145275 } },
    installed_tcp_m: { x: 0.0407, y: 0.035, z: 0.149525 },
    installed_tcp_measured: false, tool_orientation_verified: false,
    klipper_extrusion: { units_per_e_mm: 1, direction: "positive" },
    extrusion_ratio_measured: false, printhead_recalibration_required: true,
  },
  nozzleVerification: { measured: false, orientation: false, extrusion: false },
  workcellSafety: {}, jobSubmitting: false,
};
const context = vm.createContext({
  state, elements: { mainContent },
  document: {
    getElementById(id) {
      return inputs[id] ||= { value: "", addEventListener(event, callback) { listeners.set(`${id}:${event}`, callback); } };
    },
  },
  modulePageHeader: () => "", materialIcon: () => "",
  escapeHtml: value => String(value), t: key => key,
  jobMarkup: () => "", bindJobCancel: () => {}, jobIsRunning: () => false,
  invalidatePlanningForWorkcellChange: () => { invalidations += 1; },
  render: () => context.renderWorkcellModule(),
  notify: (message, kind) => notifications.push({ message, kind }),
  api: async (path, request) => {
    calls.push({ path, body: request.body });
    assert.equal(path, "/api/workcell/nozzle-tcp", "Calibration must only call the local save API");
    return { ...state.workcell, klipper_extrusion: request.body.klipper_extrusion,
      installed_tcp_measured: request.body.physically_measured,
      tool_orientation_verified: request.body.orientation_verified,
      extrusion_ratio_measured: request.body.extrusion_ratio_measured,
      printhead_recalibration_required: false };
  },
});
vm.runInContext(renderer, context);
context.renderWorkcellModule();
assert.match(mainContent.innerHTML, /geometryRecalibrationRequired/);
assert.match(mainContent.innerHTML, /nozzleTcpCadLead/);
assert.match(mainContent.innerHTML, /id="nozzleTcpX"[^>]*value="32\.550"/,
  "A changed geometry must prefill the revised CAD candidate, not the saved old tool TCP");
assert.match(mainContent.innerHTML, /id="nozzleTcpY"[^>]*value="34\.400"/);
assert.match(mainContent.innerHTML, /id="nozzleTcpZ"[^>]*value="145\.275"/);
assert.match(mainContent.innerHTML, /previousToolTcp[^]*\[40\.700, 35\.000, 149\.525\] mm/,
  "The old tool TCP must remain visible with an explicit comparison-only label");
assert.equal(JSON.stringify(state.workcell.installed_tcp_m), JSON.stringify({ x: 0.0407, y: 0.035, z: 0.149525 }),
  "Prefilling the display must not modify the installed calibration");
assert.match(mainContent.innerHTML, /id="installedExtrusionRatio"[^>]*value="1"/);
assert.doesNotMatch(mainContent.innerHTML.match(/<input id="installedExtrusionMeasured"[^>]*>/)?.[0] || "", /checked/);

state.workcell.printhead_recalibration_required = false;
state.workcell.installed_tcp_m = { x: 0.0328, y: 0.0341, z: 0.1451 };
state.workcell.installed_tcp_measured = true;
context.renderWorkcellModule();
assert.match(mainContent.innerHTML, /id="nozzleTcpX"[^>]*value="32\.800"/,
  "A matching installed geometry must retain its actual measured TCP");
assert.match(mainContent.innerHTML, /id="nozzleTcpY"[^>]*value="34\.100"/);
assert.match(mainContent.innerHTML, /id="nozzleTcpZ"[^>]*value="145\.100"/);
assert.doesNotMatch(mainContent.innerHTML, /previousToolTcp/);

inputs.nozzleTcpX.value = "32.55";
inputs.nozzleTcpY.value = "34.4";
inputs.nozzleTcpZ.value = "145.275";
inputs.installedExtrusionRatio.value = "2.5";
inputs.installedExtrusionDirection.value = "negative";
for (const id of ["nozzlePhysicallyMeasured", "nozzleOrientationVerified", "installedExtrusionMeasured"]) {
  listeners.get(`${id}:change`)({ target: { checked: true } });
}
await listeners.get("saveNozzleTcpButton:click")();
assert.equal(calls.length, 1);
assert.equal(JSON.stringify(calls[0]), JSON.stringify({ path: "/api/workcell/nozzle-tcp", body: {
  operator_action: "save-nozzle-tcp", offset_mm: { x: 32.55, y: 34.4, z: 145.275 },
  physically_measured: true, orientation_verified: true,
  klipper_extrusion: { units_per_e_mm: 2.5, direction: "negative" },
  extrusion_ratio_measured: true,
} }));
assert.equal(state.workcell.extrusion_ratio_measured, true);
assert.equal(invalidations, 1);
assert.equal(state.nozzleVerification.extrusion, false, "Saving must not pre-check a future physical confirmation");
assert.doesNotMatch(mainContent.innerHTML.match(/<input id="installedExtrusionMeasured"[^>]*>/)?.[0] || "", /checked/);

inputs.installedExtrusionRatio.value = "0";
await listeners.get("saveNozzleTcpButton:click")();
assert.equal(calls.length, 1, "Invalid ratio must not reach the save API");
assert.deepEqual(notifications.at(-1), { message: "extrusionRatioInvalid", kind: "error" });
console.log("Installed TCP, orientation and extrusion calibration can be explicitly saved without hardware motion.");
