// English is fixed across old browser preferences, backend settings, and renderers.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const html = readFileSync(new URL("../app/index.html", import.meta.url), "utf8");
const stored = new Map([["urStationLanguage", "zh"]]);
const locales = [];
const nodes = new Map();
const nodeFor = id => {
  if (!nodes.has(id)) nodes.set(id, {
    hidden: false, textContent: "", dataset: {},
    addEventListener() {}, setAttribute() {},
  });
  return nodes.get(id);
};
const document = {
  documentElement: { lang: "" }, title: "",
  getElementById: nodeFor,
  querySelectorAll: () => [],
  querySelector: () => null,
};
const config = { language: "zh" };
const context = vm.createContext({
  localStorage: { getItem: key => stored.get(key) ?? null, setItem: (key, value) => stored.set(key, value) },
  document,
  window: { addEventListener() {}, setInterval() {} },
  Intl: {
    DateTimeFormat: function(locale, options) {
      locales.push(locale);
      return new Intl.DateTimeFormat(locale, options);
    },
    NumberFormat: function(locale, options) {
      locales.push(locale);
      return new Intl.NumberFormat(locale, options);
    },
  },
});
vm.runInContext(source, context);
const app = vm.runInContext("({state, translations, t, formatNumber, initialize, applyStaticTranslations, updateClock, elements})", context);
assert.equal(app.state.language, "en", "A saved legacy locale cannot change the application language");
assert.deepEqual(Object.keys(app.translations), ["en"]);
assert.ok(Object.keys(app.translations.en).length >= 900, "Retain the complete product string catalog");
assert.doesNotMatch(source + html, /[\u3400-\u4dbf\u4e00-\u9fff]/u);
assert.doesNotMatch(html, /languageButton/);
assert.doesNotMatch(source, /elements\.languageButton/);

// Rendering must stay English even if an old integration assigns a legacy locale.
app.state.language = "zh";
app.elements.currentDateTime = nodeFor("currentDateTime");
assert.equal(app.t("productName"), "UR Print Workstation");
assert.equal(app.t("printZeroDelta", { offset: 3 }), "Path offset above physical tabletop: 3 mm");
assert.equal(app.t("stopTitle"), "Stop the ROS system?");
app.formatNumber(1234.5, 1);
app.updateClock();
assert.ok(locales.length >= 2);
assert.ok(locales.every(locale => locale === "en-US"));

// The real initializer must ignore an old backend language and migrate local storage.
app.state.language = "en";
context.configFromOldBackend = config;
vm.runInContext(`
  bindGlobalEvents = () => {};
  render = () => {};
  api = async path => ({
    "/api/health": {demo: false},
    "/api/config": {config: configFromOldBackend},
    "/api/mode": {mode: "offline"},
    "/api/status": {ros: {}, active_job: null},
    "/api/workcell/height": {},
  })[path];
  renderOperatingModeControl = () => {};
  renderRunbookMenu = () => {};
`, context);
await app.initialize();
assert.equal(app.state.config.language, "zh", "Exercise the actual legacy backend setting");
assert.equal(app.state.language, "en");
assert.equal(stored.get("urStationLanguage"), "en");
app.state.language = "zh";
app.applyStaticTranslations();
assert.equal(document.documentElement.lang, "en");
assert.equal(document.title, "UR Print Workstation");
console.log("English-only rendering preserves the complete catalog and ignores legacy language preferences (no hardware).");
