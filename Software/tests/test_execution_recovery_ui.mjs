// Hardware-isolated UI/native contract: recovery and lifecycle cleanup are product workflows.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const source = readFileSync(new URL("../app/app.js", import.meta.url), "utf8");
const styles = readFileSync(new URL("../app/styles.css", import.meta.url), "utf8");
const backend = readFileSync(new URL("../control_station.py", import.meta.url), "utf8");
const macApp = readFileSync(new URL("../native_app/URPrintWorkstation.swift", import.meta.url), "utf8");
const windowsApp = readFileSync(new URL("../windows_app/Program.cs", import.meta.url), "utf8");
const stopScript = readFileSync(new URL("../../scripts/stop.sh", import.meta.url), "utf8");

assert.match(source, /function printRecoveryMarkup\(/);
assert.match(source, /async function recoverPrintStop\(/);
assert.match(source, /\/api\/print\/recovery/);
assert.match(source, /verify-stop-and-recover/);
assert.match(source, /timeoutMs:\s*150000/);
assert.match(source, /id="recoverPrintStopButton"/);
assert.match(styles, /\.print-recovery/);
assert.match(backend, /\/api\/print\/recovery/);
assert.match(backend, /def recover_print_stop/);
assert.match(backend, /def shutdown_runtime/);
assert.match(macApp, /api\/runtime\/shutdown/);
assert.match(windowsApp, /api\/runtime\/shutdown/);
assert.match(stopScript, /stop_project_executor_processes/);
assert.match(source, /const stopUnconfirmed = runtime\?\.state === "stop-unconfirmed";/);
assert.match(source, /stopUnconfirmed \|\| active \|\| atStart/);
assert.match(source, /stopUnconfirmed \|\| active \|\| !atStart/);

console.log("Operator recovery and native lifecycle cleanup are exposed (no hardware). ");
