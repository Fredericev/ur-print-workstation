// Hardware-isolated check of the actual tabletop proposal click handler.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const handlerSource = source.match(
  /document\.getElementById\("setHeightDraftButton"\)\?\.addEventListener\("click", async \(\) => \{[^]*?\n  \}\);/,
)?.[0];
assert.ok(handlerSource, "The physical-table proposal handler must remain testable");

const input = { value: "-14.000" };
const calls = [];
let handler;
let invalidations = 0;
const height = { draft_surface_z_m: null };
const state = { workcell: height, workcellSafety: {}, printZMeasuredClearanceMm: "x", heightMeasuredClearanceMm: "x" };
const context = vm.createContext({
  currentMm: -14,
  height,
  state,
  document: {
    getElementById(id) {
      if (id === "setHeightDraftButton") return { addEventListener: (_event, callback) => { handler = callback; } };
      if (id === "proposedHeightMm") return input;
      return null;
    },
  },
  api: async (_path, request) => { calls.push(request.body); return { ...height }; },
  invalidatePlanningForWorkcellChange: () => { invalidations += 1; },
  render: () => {},
  notify: (message) => { throw new Error(message); },
  t: key => key,
});
vm.runInContext(handlerSource, context);

await handler();
assert.equal(JSON.stringify(calls[0]), JSON.stringify({ discard: true }));
assert.equal(invalidations, 0, "An unchanged table with no draft must not revoke a valid review");

height.draft_surface_z_m = -0.014;
await handler();
assert.equal(JSON.stringify(calls[1]), JSON.stringify({ discard: true }));
assert.equal(invalidations, 1, "Clearing an existing candidate must revoke its comparison review");

input.value = "40.000";
await handler();
assert.equal(JSON.stringify(calls[2]), JSON.stringify({ surface_z_mm: 40 }));
assert.equal(invalidations, 2, "A genuinely changed physical tabletop still needs re-review");
console.log("Same-value tabletop proposal is a no-op; real table changes still invalidate review (no hardware).");
