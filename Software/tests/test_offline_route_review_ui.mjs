// Offline 3D route playback must not depend on a six-axis robot trajectory.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const extract = name => source.match(new RegExp(`function ${name}\\([^]*?\\n\\}`))[0];
const playButton = { disabled: true, innerHTML: "" };
const progressValues = [];
const state = {
  gcode: {
    view: "path-review",
    pathReview: { path: { segments: [{ x0: 0, x1: 1 }] } },
    pathReviewPlaying: false,
    pathReviewProgress: 0,
    pathReviewPlaybackSpeed: 1,
    pathReviewAnimationFrame: null,
    pathReviewLastFrame: 0,
    pathReviewViewer: {
      setProgress(value) {
        progressValues.push(value);
        state.gcode.pathReviewProgress = value;
      },
    },
  },
};
const context = vm.createContext({
  state,
  document: { getElementById: id => id === "pathReviewPlayButton" ? playButton : null },
  hasRobotMotionTrajectory: () => false,
  materialIcon: icon => icon,
  escapeHtml: text => text,
  t: key => key,
  stopPathReviewPlayback: () => { state.gcode.pathReviewPlaying = false; },
  window: { requestAnimationFrame: () => 1 },
});
for (const name of [
  "hasReviewRoute",
  "updatePathReviewPlaybackButton",
  "pathReviewPlaybackFrame",
  "togglePathReviewPlayback",
]) vm.runInContext(extract(name), context);

assert.equal(context.hasReviewRoute(), true);
context.updatePathReviewPlaybackButton();
assert.equal(playButton.disabled, false);
assert.match(playButton.innerHTML, /playPreview/);
context.togglePathReviewPlayback();
assert.equal(state.gcode.pathReviewPlaying, true);
context.pathReviewPlaybackFrame(1000);
context.pathReviewPlaybackFrame(1600);
assert.ok(progressValues.some(value => value > 0), "the offline route cursor should advance");

state.gcode.pathReview = null;
context.updatePathReviewPlaybackButton();
assert.equal(playButton.disabled, true);
context.togglePathReviewPlayback();
assert.equal(state.gcode.pathReviewPlaying, false);

assert.doesNotMatch(source, /moduleAvailable|module-availability|module-page-badge/);
console.log("Offline route playback and Available-label removal: OK");
