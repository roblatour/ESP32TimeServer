const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const { test } = require("node:test");
const { parseArgs, countResults, suitesFor, runSuite } = require("./run_all.js");

function temporaryDirectory(t) {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), "firmware-master-test-"));
  t.after(() => fs.rmSync(directory, { recursive: true, force: true }));
  return directory;
}

test("arguments require external output and validate the optional build directory", t => {
  const out = temporaryDirectory(t);
  assert.throws(() => parseArgs([]), /Usage:/);
  assert.throws(() => parseArgs([__dirname]), /outside the source tree/);
  assert.throws(() => parseArgs([out, "--unknown"]), /Usage:/);
  assert.throws(() => parseArgs([out, "--build-dir"]), /Usage:/);
  assert.throws(() => parseArgs([out, "--build-dir", out]), /compile_commands/);
  fs.writeFileSync(path.join(out, "compile_commands.json"), "[]");
  const options = parseArgs([out, "--overwrite", "--build-dir", out]);
  assert.equal(options.overwrite, true);
  assert.equal(suitesFor(options).length, 3);
  const hostSuites = suitesFor({ out });
  assert.equal(hostSuites.length, 2);
  assert.equal(hostSuites[1].args.at(-1), "--network-only");
  assert.equal(hostSuites[0].args.length, 2);
});

test("counts grouped result lines without counting diagnostics or the summary twice", () => {
  assert.deepEqual(countResults("PASS: first\r\nPTP host tests passed\r\nPASS: failed-input rejection\nFAIL: suite\nerror: FAIL example\nTotal passes: 3\nTotal fails: 1\n"), { passes: 3, fails: 1 });
});

test("suite captures both output streams without a shell", t => {
  const directory = temporaryDirectory(t);
  const results = path.join(directory, "results.txt");
  const log = fs.openSync(results, "w");
  try {
    const passed = runSuite({ name: "Fixture", args: ["-e", "process.stdout.write('PASS: fixture\\n'); process.stderr.write('diagnostic\\n')"] }, log, results);
    assert.equal(passed, true);
    const text = fs.readFileSync(results, "utf8");
    assert.match(text, /diagnostic/);
    assert.deepEqual(countResults(text), { passes: 1, fails: 0 });
  } finally {
    fs.closeSync(log);
  }
});

test("failed and silent suites are recorded and later suites still execute", t => {
  const directory = temporaryDirectory(t);
  const results = path.join(directory, "results.txt");
  const log = fs.openSync(results, "w");
  try {
    assert.equal(runSuite({ name: "Exit failure", args: [] }, log, results, () => ({ status: 2 })), false);
    assert.equal(runSuite({ name: "Spawn failure", args: [] }, log, results, () => ({ error: new Error("ENOENT") })), false);
    assert.equal(runSuite({ name: "Signal failure", args: [] }, log, results, () => ({ status: null, signal: "SIGTERM" })), false);
    assert.equal(runSuite({ name: "Silent", args: [] }, log, results, () => ({ status: 0 })), false);
    assert.equal(runSuite({ name: "Explicit failure", args: [] }, log, results, () => {
      fs.writeSync(log, "FAIL: fixture\n");
      return { status: 0 };
    }), false);
    assert.equal(runSuite({ name: "Recovery", args: [] }, log, results, () => {
      fs.writeSync(log, "PASS: subsequent suite\n");
      return { status: 0 };
    }), true);
    assert.deepEqual(countResults(fs.readFileSync(results, "utf8")), { passes: 1, fails: 5 });
  } finally {
    fs.closeSync(log);
  }
});

for (const failed of [false, true]) {
  test(`CLI combines logs, overwrites with permission, and summarizes ${failed ? "failure" : "success"}`, t => {
    const directory = temporaryDirectory(t);
    const results = path.join(directory, "results.txt");
    const fixture = path.join(directory, "preload.cjs");
    fs.writeFileSync(results, "old results");
    fs.writeFileSync(path.join(directory, "compile_commands.json"), "[]");
    fs.writeFileSync(fixture, `
const fs = require("node:fs");
let calls = 0;
require("node:child_process").spawnSync = (command, args, options) => {
  calls++;
  fs.writeSync(options.stdio[1], "PASS: fixture " + calls + "\\n");
  fs.writeSync(options.stdio[2], "stderr fixture " + calls + "\\n");
  return { status: ${failed} && calls === 1 ? 2 : 0 };
};
`);
    const args = ["-r", fixture, path.join(__dirname, "run_all.js"), directory, "--overwrite"];
    if (failed) args.push("--build-dir", directory);
    const result = spawnSync(process.execPath, args, { encoding: "utf8" });
    assert.equal(result.status, failed ? 1 : 0, result.stderr);
    const text = fs.readFileSync(results, "utf8");
    assert.doesNotMatch(text, /old results/);
    assert.match(text, /stderr fixture 2/);
    assert.match(result.stdout, new RegExp(`Total passes: ${failed ? 3 : 2}`));
    assert.match(result.stdout, new RegExp(`Total fails: ${failed ? 1 : 0}`));
    assert.deepEqual(countResults(text), { passes: failed ? 3 : 2, fails: failed ? 1 : 0 });
    assert.equal(text.includes("SKIP:"), !failed);
  });
}

test("noninteractive overwrite refusal preserves existing results", t => {
  const directory = temporaryDirectory(t);
  const results = path.join(directory, "results.txt");
  fs.writeFileSync(results, "existing results");
  const result = spawnSync(process.execPath, [path.join(__dirname, "run_all.js"), directory], { encoding: "utf8", stdio: ["ignore", "pipe", "pipe"] });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /--overwrite/);
  assert.equal(fs.readFileSync(results, "utf8"), "existing results");
});
