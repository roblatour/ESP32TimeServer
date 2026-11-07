const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");

const temporary = fs.mkdtempSync(path.join(os.tmpdir(), "Arduino-graph-test-"));
const project = path.join(temporary, "project");
const build = path.join(project, "build_s31");
const script = path.join(__dirname, "check_build_graph.js");

function write(name, data) {
  fs.writeFileSync(path.join(build, name), data);
}

function audit() {
  return spawnSync(process.execPath, [script, build, "s31"], { encoding: "utf8" });
}

try {
  fs.mkdirSync(build, { recursive: true });
  const description = {
    target: "esp32s31",
    project_path: project,
    idf_path: path.join(temporary, "sdk"),
    c_compiler: path.join(temporary, "toolchain", "bin", "riscv-gcc"),
    build_components: ["jef-sure__hd44780"],
    build_component_paths: [path.join(project, "managed_components", "jef-sure__hd44780")],
  };
  const commands = ["gnss_receiver", "gnss_uart"].map(name => ({
    file: path.join(project, "main", "custom", `${name}.cpp`),
    command: `${description.c_compiler} -I${project}`,
  }));
  const map = commands.map(entry => `${entry.file}.obj`).join("\n");
  const image = Buffer.alloc(32);
  image[0] = 0xe9;
  image.writeUInt16LE(0x20, 12);
  image.writeUInt16LE(0, 15);
  const partitions = Buffer.alloc(64);
  for (const [index, subtype] of [0x10, 0x11].entries()) {
    const offset = index * 32;
    partitions.writeUInt16LE(0x50aa, offset);
    partitions[offset + 3] = subtype;
    partitions.writeUInt32LE(0x20000 + index * 0x400000, offset + 4);
    partitions.writeUInt32LE(0x400000, offset + 8);
  }
  write("project_description.json", JSON.stringify(description));
  write("compile_commands.json", JSON.stringify(commands));
  write("ESP32TimeServer.map", map);
  write("sdkconfig", [
    'CONFIG_IDF_TARGET="esp32s31"', "CONFIG_ESP32S31_REV_MIN_0=y",
    'CONFIG_ESPTOOLPY_FLASHSIZE="16MB"', 'CONFIG_ESPTOOLPY_FLASHFREQ="80m"',
    "CONFIG_COMPILER_OPTIMIZATION_PERF=y", "CONFIG_FREERTOS_HZ=1000",
    "CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=320",
  ].join("\n"));
  write("flasher_args.json", JSON.stringify({
    app: { file: "app.bin" },
    bootloader: { file: "boot.bin", offset: "0x2000" },
    "partition-table": { file: "partitions.bin", offset: "0x8000" },
  }));
  write("app.bin", image);
  write("boot.bin", image);
  write("partitions.bin", partitions);

  let result = audit();
  assert.equal(result.status, 0, result.stderr);

  description.build_components.push("espressif__arduino-esp32");
  write("project_description.json", JSON.stringify(description));
  result = audit();
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /Retired GNSS dependency found in component graph/);
  description.build_components.pop();
  write("project_description.json", JSON.stringify(description));

  commands.push({ file: path.join(project, "components", "SparkFun_u-blox_GNSS_v3", "vendor.cpp") });
  write("compile_commands.json", JSON.stringify(commands));
  result = audit();
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /Retired GNSS dependency found in compile graph/);
  commands.pop();
  write("compile_commands.json", JSON.stringify(commands));

  write("ESP32TimeServer.map", `${map}\nlibarduino-esp32.a`);
  result = audit();
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /Retired GNSS dependency found in link map/);
  write("ESP32TimeServer.map", map);

  image.writeUInt16LE(0x12, 12);
  write("app.bin", image);
  result = audit();
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /wrong chip ID/);
  image.writeUInt16LE(0x20, 12);
  write("app.bin", image);
  image.writeUInt16LE(0x12, 12);
  write("boot.bin", image);
  result = audit();
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /Bootloader image has the wrong chip ID/);
} finally {
  fs.rmSync(temporary, { recursive: true, force: true });
}
