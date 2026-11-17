// Hardware-isolated regression for setting the physical collision plane before Print Z.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const renderSource = source.match(/function renderWorkcellModule\(\) \{[^]*?\n\}/)?.[0];
assert.ok(renderSource, "The actual workcell renderer must be testable");

const mainContent = { innerHTML: "" };
const listeners = new Map();
const inputs = {
  proposedHeightMm: { value: "-20" },
  printZeroZMm: { value: "-15" },
};
const calls = [];
const state = {
  workcell: {
    surface_z_m: -0.014, planning_surface_z_m: -0.020,
    draft_surface_z_m: -0.020, print_zero_z_m: -0.014,
    print_z_placement_mode: "operator_selected", print_z_candidate: false,
    nozzle: { tool0_to_nozzle_m: { x: 0, y: 0, z: 0 } },
    installed_tcp_m: { x: 0, y: 0, z: 0 }, trial: null,
  },
  heightMeasuredClearanceMm: "", printZMeasuredClearanceMm: "",
  nozzleVerification: { measured: false, orientation: false },
  workcellSafety: {}, jobSubmitting: false,
};
let invalidations = 0;
const context = vm.createContext({
  state, elements: { mainContent },
  document: {
    getElementById(id) {
      if (!inputs[id]) inputs[id] = { addEventListener(event, callback) { listeners.set(`${id}:${event}`, callback); } };
      return inputs[id];
    },
    querySelectorAll: () => [],
  },
  modulePageHeader: () => "", materialIcon: () => "",
  escapeHtml: value => String(value), t: key => key,
  jobMarkup: () => "", bindJobCancel: () => {},
  jobIsRunning: () => false, isOnlineMode: () => false,
  invalidatePlanningForWorkcellChange: () => { invalidations += 1; },
  render: () => context.renderWorkcellModule(), notify: message => { throw new Error(message); },
  api: async (path, request) => {
    calls.push({ path, body: request.body });
    if (path === "/api/workcell/height") return {
      ...state.workcell, surface_z_m: -0.020, planning_surface_z_m: -0.020,
      draft_surface_z_m: null, trial: null,
    };
    if (path === "/api/workcell/print-z") return {
      ...state.workcell, print_zero_z_m: -0.015, print_z_placement_mode: "operator_selected",
    };
    throw new Error(`Unexpected ${path}`);
  },
});
vm.runInContext(renderSource, context);

const tag = id => mainContent.innerHTML.match(new RegExp(`<input id="${id}"[^>]*>`))?.[0] || "";
context.renderWorkcellModule();
assert.doesNotMatch(tag("printZeroZMm"), /disabled/,
  "A tabletop edit must not lock Print Z selection");
assert.match(tag("printZeroZMm"), /min="-14\.000"/,
  "Before saving a tabletop edit, Print Z's minimum should reflect the active collision plane");
assert.ok(mainContent.innerHTML.indexOf("heightCurrent") < mainContent.innerHTML.indexOf("printZeroTitle"),
  "Physical tabletop must be the first setup step");
assert.doesNotMatch(mainContent.innerHTML, /planHeightTrialButton|executeHeightTrialButton|confirmHeightTrialButton/,
  "Height setup must not expose the trial as a required print step");

await listeners.get("setHeightDraftButton:click")();
assert.equal(JSON.stringify(calls[0]), JSON.stringify({ path: "/api/workcell/height", body: { surface_z_mm: -20 } }));
assert.equal(state.workcell.surface_z_m, -0.020);
assert.equal(state.workcell.draft_surface_z_m, null);
assert.match(tag("printZeroZMm"), /min="-20\.000"/);
assert.doesNotMatch(tag("printZeroZMm"), /disabled/);
await listeners.get("setPrintZDraftButton:click")();
assert.equal(JSON.stringify(calls[1]), JSON.stringify({ path: "/api/workcell/print-z", body: { print_zero_z_mm: -15 } }));
assert.equal(state.workcell.print_zero_z_m, -0.015);
assert.equal(invalidations, 2, "Each real geometry change must invalidate prior reviews");
state.workcell.print_zero_z_m = null;
state.workcell.print_z_reverification_required = true;
state.workcell.print_z_candidate = true;
context.renderWorkcellModule();
assert.match(tag("printZeroZMm"), /value=""/,
  "An invalid old Print Z should render as an empty input, not a fabricated tabletop value");
assert.doesNotMatch(tag("printZeroZMm"), /disabled/);
assert.doesNotMatch(tag("proposedHeightMm"), /disabled/,
  "An invalid Print Z must not lock tabletop editing");
console.log("Tabletop -20 then Print Z -15 is editable and saved without a height trial (no hardware).");
