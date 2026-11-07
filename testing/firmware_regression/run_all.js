const fs = require("node:fs");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const readline = require("node:readline/promises");

const root = path.resolve(__dirname, "..", "..");
const usage = "Usage: node .\\testing\\firmware_regression\\run_all.js <output-directory> [--build-dir <idf-build-directory>] [--overwrite]";

function parseArgs(args) {
  const options = { overwrite: false };
  for (let index = 0; index < args.length; index++) {
    const value = args[index];
    if (value === "--overwrite") options.overwrite = true;
    else if (value === "--build-dir" && args[index + 1] && !args[index + 1].startsWith("--")) {
      if (options.build) throw new Error(usage);
      options.build = path.resolve(args[++index]);
    } else if (!value.startsWith("--") && !options.out) options.out = path.resolve(value);
    else throw new Error(usage);
  }
  if (!options.out) throw new Error(usage);
  const relative = path.relative(root, options.out);
  if (!relative || (!path.isAbsolute(relative) && relative !== ".." && !relative.startsWith(`..${path.sep}`)))
    throw new Error("Choose an output directory outside the source tree.");
  if (options.build && !fs.existsSync(path.join(options.build, "compile_commands.json")))
    throw new Error("The ESP-IDF build directory must contain compile_commands.json from a successful build.");
  return options;
}

function countResults(text) {
  const lines = text.split(/\r?\n/);
  return {
    passes: lines.filter(line => /^PASS:/.test(line) || line === "PTP host tests passed").length,
    fails: lines.filter(line => /^FAIL:/.test(line)).length,
  };
}

function suitesFor(options) {
  const host = path.join(options.out, "host");
  const suites = [
    { name: "Full host regression", args: [path.join(__dirname, "run.js"), host] },
    { name: "Network regression in both debug modes", args: [path.join(__dirname, "run.js"), host, "--network-only"] },
  ];
  if (options.build) suites.push({
    name: "Firmware build variants",
    args: [path.join(__dirname, "check_build_variants.js"), options.build, path.join(options.out, "variants")],
  });
  return suites;
}

function runSuite(suite, log, results, execute = spawnSync) {
  const started = Date.now();
  const start = fs.fstatSync(log).size;
  fs.writeSync(log, `\n=== ${suite.name} ===\n${JSON.stringify([process.execPath, ...suite.args])}\n`);
  console.log(`Running: ${suite.name} (output saved to ${results})`);
  let failure;
  try {
    const result = execute(process.execPath, suite.args, { cwd: root, stdio: ["ignore", log, log] });
    if (result.error) failure = result.error.message;
    else if (result.status !== 0) failure = result.signal ? `signal ${result.signal}` : `exit code ${result.status}`;
  } catch (error) {
    failure = error.message;
  }
  const section = fs.readFileSync(results).subarray(start).toString("utf8");
  const counts = countResults(section);
  if (!failure && counts.fails) failure = "failure markers reported despite a zero exit code";
  if (!failure && !counts.passes) failure = "no recognized pass results were reported";
  if (failure && !counts.fails) fs.writeSync(log, `\nFAIL: ${suite.name}: ${failure}\n`);
  fs.writeSync(log, `\nSUITE ${failure ? "FAILED" : "COMPLETED"}: ${suite.name} (${((Date.now() - started) / 1000).toFixed(1)} seconds)${failure ? `: ${failure}` : ""}\n`);
  console.log(`${failure ? "Failed" : "Completed"}: ${suite.name}`);
  return !failure;
}

async function main(args = process.argv.slice(2)) {
  if (args.length === 1 && args[0] === "--help") {
    console.log(usage);
    return;
  }
  const options = parseArgs(args);
  const results = path.join(options.out, "results.txt");
  if (fs.existsSync(results) && !options.overwrite) {
    if (!process.stdin.isTTY) throw new Error(`Results already exist: ${results}. Use --overwrite for a noninteractive run.`);
    const prompt = readline.createInterface({ input: process.stdin, output: process.stdout });
    let answer;
    try {
      answer = await prompt.question(`Overwrite ${results}? [y/N] `);
    } finally {
      prompt.close();
    }
    if (!/^y(es)?$/i.test(answer.trim())) {
      console.log("Cancelled; existing results were not changed.");
      process.exitCode = 1;
      return;
    }
  }
  fs.mkdirSync(options.out, { recursive: true });
  const log = fs.openSync(results, "w");
  const interrupted = () => {
    fs.writeSync(log, "\nFAIL: Regression run interrupted; results are incomplete.\n");
    fs.closeSync(log);
    console.error(`Interrupted; partial results saved to ${results}`);
    process.exit(130);
  };
  process.once("SIGINT", interrupted);
  let failedSuites = 0;
  const suites = suitesFor(options);
  try {
    fs.writeSync(log, `Firmware regression results\nStarted: ${new Date().toISOString()}\nRepository: ${root}\nNode: ${process.version}\n`);
    if (!options.build) {
      const skipped = "SKIP: Firmware build variants (supply --build-dir after a successful ESP-IDF build).";
      fs.writeSync(log, `${skipped}\n`);
      console.log(skipped);
    }
    for (const suite of suites) {
      if (!runSuite(suite, log, results)) failedSuites++;
      await new Promise(resolve => setImmediate(resolve));
    }
    const counts = countResults(fs.readFileSync(results, "utf8"));
    const summary = `\nTotal passes: ${counts.passes}\nTotal fails: ${counts.fails}\nSuites completed successfully: ${suites.length - failedSuites}/${suites.length}\nFinished: ${new Date().toISOString()}\n`;
    fs.writeSync(log, summary);
    console.log(summary.trim());
    console.log(`Results: ${results}`);
    if (counts.fails || failedSuites) process.exitCode = 1;
  } finally {
    process.removeListener("SIGINT", interrupted);
    fs.closeSync(log);
  }
}

if (require.main === module) main().catch(error => {
  console.error(error.message);
  process.exitCode = 1;
});

module.exports = { parseArgs, countResults, suitesFor, runSuite };
