// Render the real execution step with inert DOM helpers; no device transport.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const renderSource = source.match(/function renderExecutionModule\(\) \{[^]*?\n\}/)?.[0];
assert.ok(renderSource, "The execution renderer must be available to the UI regression");
const mainContent = { innerHTML: "" };
const listeners = new Map();
const navigation = [];
const state = {
  moduleSteps: { execution: 1 },
  printInterlock: { locked: false },
  execution: {
    package: { operation_mode: "production_print", execution_extent: "full" },
    runtime: { state: "at-start" }, job: { state: "succeeded" },
    reviewedLowSpeed: true, externalControlPlaying: true,
  },
  safety: { emergency_stop: true, workspace_clear: true, wired_network: true },
  workcell: { print_z_candidate: false },
  jobSubmitting: false, activeJob: null, rosStatus: {}, demo: false,
};
const context = vm.createContext({
  state, elements: { mainContent },
  document: { getElementById: id => ({ addEventListener: (_event, callback) => listeners.set(id, callback) }) },
  selectRunbook: id => navigation.push(id),
  moveitExecutionReady: () => true,
  modulePageHeader: () => "", executionPackageMarkup: () => "",
  executionAuthorizationMarkup: () => "", executionRuntimeMarkup: () => "",
  jobMarkup: () => "", bindExecutionConfirmations: () => {}, bindJobCancel: () => {},
  materialIcon: icon => `<i>${icon}</i>`, escapeHtml: value => String(value), t: key => key,
});
vm.runInContext(renderSource, context);
context.renderExecutionModule();
const production = mainContent.innerHTML;
assert.doesNotMatch(production.match(/<button id="startPrintButton"[^>]*>/)?.[0] || "", /disabled/,
  "Reviewed production print must be startable after the robot reaches print start");
assert.match(production, /productionPrintNoticeAtStart/,
  "At-start production print should disclose the software coordination limitation");
assert.doesNotMatch(production, /<i>lock<\/i>/,
  "The production start notice is not a lock");

const renderedStartButton = () => mainContent.innerHTML.match(/<button id="startPrintButton"[^>]*>/)?.[0] || "";
state.execution.runtime.state = "idle";
context.renderExecutionModule();
assert.match(renderedStartButton(), /disabled/, "Starting before the reviewed position remains blocked");
state.execution.runtime.state = "at-start";
state.safety.workspace_clear = false;
context.renderExecutionModule();
assert.match(renderedStartButton(), /disabled/, "Missing live authorization remains blocked");
state.safety.workspace_clear = true;
state.execution.runtime.state = "stop-unconfirmed";
context.renderExecutionModule();
assert.match(renderedStartButton(), /disabled/, "An unconfirmed stop remains blocked");
state.execution.runtime.state = "at-start";

state.execution.package.operation_mode = "tool0_motion_test";
context.renderExecutionModule();
const coldTest = mainContent.innerHTML;
assert.doesNotMatch(coldTest, /productionPrintNoticeAtStart/);
assert.doesNotMatch(coldTest.match(/<button id="startPrintButton"[^>]*>/)?.[0] || "", /disabled/);
state.execution.package.operation_mode = "production_print";
const failure = { state: "failed", mode: "move-to-start", root_error: { message: "endpoint tolerance" } };
state.execution.runtime = failure;
context.renderExecutionModule();
assert.match(mainContent.innerHTML.match(/<button id="moveToStartButton"[^>]*>/)?.[0] || "", /disabled/,
  "The failed approach button must not offer a stale retry");
assert.match(mainContent.innerHTML, /approachRevalidationDesc/);
assert.ok(listeners.has("reviewApproachAgainButton"), "A manual return to MoveIt review must be offered");
listeners.get("reviewApproachAgainButton")();
assert.deepEqual(navigation, ["motion"]);
assert.equal(state.moduleSteps.motion, 1);
assert.equal(state.execution.runtime, failure, "Navigation must preserve the failed runtime evidence");
assert.equal(state.execution.package.operation_mode, "production_print");
state.execution.runtime = { state: "canceled", mode: "move-to-start" };
context.renderExecutionModule();
assert.match(mainContent.innerHTML.match(/<button id="moveToStartButton"[^>]*>/)?.[0] || "", /disabled/);
console.log("At-start production action is available with a coordination notice; cold-test action is unaffected (no hardware).");
