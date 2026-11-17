// Hardware-isolated regression tests for the workstation's in-memory gates.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const names = ["isRobotMotionTrajectory", "hasValidatedMoveItTrajectory", "moveitExecutionReady", "validatePathWithMoveIt"];
const functions = names.map((name) => {
  const match = source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n\\}`));
  assert.ok(match, `Missing ${name}`);
  return match[0];
}).join("\n");
const policy = "continuous-cartesian-approach-v4";
const result = {
  status: "passed", collision_free: true, joint_limits: "passed",
  cartesian_fraction: 1, approach_status: "passed",
  planning_policy_version: policy, interpolation_validation: "passed",
  joint_names: Array(6).fill("joint"),
  trajectory: [{ positions: Array(6).fill(0) }, { positions: Array(6).fill(0.01) }],
};
const state = { gcode: {
  sourceFile: { name: "test.gcode" }, pathReview: {},
  pathReviewMoveitResult: result, pathReviewMoveitJob: { id: "old" },
} };
const context = vm.createContext({
  PLANNING_POLICY_VERSION: policy, state,
  t: (key) => key, render() {}, notify() {}, resetPrintExecution() {},
  stopPathReviewPlayback() {}, moveitValidationAvailable: () => true,
  fetchWithTimeout: async () => { throw new Error("Plan failed"); },
});
vm.runInContext(functions, context);
assert.equal(context.moveitExecutionReady(), true);
assert.equal(context.hasValidatedMoveItTrajectory(), true);
for (const missing of ["planning_policy_version", "interpolation_validation"]) {
  state.gcode.pathReviewMoveitResult = { ...result };
  delete state.gcode.pathReviewMoveitResult[missing];
  assert.equal(context.moveitExecutionReady(), false, `Missing ${missing} must lock execution`);
  assert.equal(context.hasValidatedMoveItTrajectory(), false);
}
state.gcode.pathReviewMoveitResult = result;
await context.validatePathWithMoveIt();
assert.equal(state.gcode.pathReviewMoveitResult, null, "A failed replacement must not retain the old pass");
assert.equal(context.moveitExecutionReady(), false);
console.log("UI planning policy and failed-replan gates passed (no browser or hardware).");
