// Verify the selected target's generated component graph, link map, and image layout.
const fs = require("node:fs");
const path = require("node:path");

const profiles = {
  pre_v3: { target: "esp32p4", revision: "CONFIG_ESP32P4_REV_MIN_1=y", minimum: 1, flash: 32 },
  v3_0: { target: "esp32p4", revision: "CONFIG_ESP32P4_REV_MIN_300=y", minimum: 300, flash: 32 },
  v3_1: { target: "esp32p4", revision: "CONFIG_ESP32P4_REV_MIN_301=y", minimum: 301, flash: 32 },
  s31: { target: "esp32s31", revision: "CONFIG_ESP32S31_REV_MIN_0=y", minimum: 0, flash: 16 },
};

const [directory, profile] = process.argv.slice(2);
if (!directory || !profiles[profile] || process.argv.length !== 4)
  throw new Error("Usage: node check_build_graph.js <build-directory> <pre_v3|v3_0|v3_1|s31>");

const build = path.resolve(directory);
const expected = profiles[profile];
const read = name => fs.readFileSync(path.join(build, ...name.split(/[\\/]/)));
const descriptionText = read("project_description.json").toString("utf8");
const description = JSON.parse(descriptionText);
const commandsText = read("compile_commands.json").toString("utf8");
const commands = JSON.parse(commandsText);
const map = read("ESP32TimeServer.map").toString("utf8");
const config = read("sdkconfig").toString("utf8");
const flasher = JSON.parse(read("flasher_args.json").toString("utf8"));
const image = read(flasher.app.file);
const bootloader = read(flasher.bootloader.file);
const partitionTable = read(flasher["partition-table"].file);
const requireCheck = (condition, message) => {
  if (!condition) throw new Error(message);
};

function withoutHostRoots(content) {
  const roots = [build, description.project_path, description.idf_path,
    path.dirname(path.dirname(description.c_compiler))];
  const forms = new Set();
  for (const root of roots) {
    if (!root || root === "." || root === path.parse(root).root) continue;
    const forward = root.replace(/\\/g, "/");
    const backward = forward.replace(/\//g, "\\");
    for (const form of [forward, backward]) {
      forms.add(form);
      forms.add(JSON.stringify(form).slice(1, -1));
    }
  }
  for (const form of [...forms].sort((a, b) => b.length - a.length))
    content = content.split(form).join("<host-root>");
  return content;
}

requireCheck(description.target === expected.target, "Build target does not match the requested profile");
for (const setting of [
  `CONFIG_IDF_TARGET="${expected.target}"`, expected.revision,
  `CONFIG_ESPTOOLPY_FLASHSIZE="${expected.flash}MB"`, 'CONFIG_ESPTOOLPY_FLASHFREQ="80m"',
  "CONFIG_COMPILER_OPTIMIZATION_PERF=y", "CONFIG_FREERTOS_HZ=1000",
])
  requireCheck(config.split(/\r?\n/).includes(setting), `Missing configuration: ${setting}`);

const retired = /sparkfun|arduino|time_server_gnss/i;
for (const [name, content] of [["component graph", descriptionText], ["compile graph", commandsText], ["link map", map]])
  requireCheck(!retired.test(withoutHostRoots(content)), `Retired GNSS dependency found in ${name}`);
for (const name of ["gnss_receiver", "gnss_uart"]) {
  requireCheck(commands.some(entry => new RegExp(`[\\\\/]${name}\\.cpp$`).test(entry.file)),
    `${name} is absent from the compilation graph`);
  requireCheck(new RegExp(`${name}\\.cpp\\.(?:obj|o)\\b`).test(map), `${name} is absent from the link map`);
}

const components = new Set(description.build_components);
const requiredLcd = expected.target === "esp32s31" ? ["jef-sure__hd44780"] :
  ["esp-idf-lib__hd44780", "esp-idf-lib__pcf8574"];
const otherLcd = expected.target === "esp32s31" ? ["esp-idf-lib__hd44780", "esp-idf-lib__pcf8574"] :
  ["jef-sure__hd44780"];
requireCheck(requiredLcd.every(name => components.has(name)), "Target LCD components are missing");
requireCheck(otherLcd.every(name => !components.has(name)), "Components for the other target are present");
for (const [name, data] of [["Application", image], ["Bootloader", bootloader]]) {
  requireCheck(data.length >= 19 && data[0] === 0xe9 &&
    data.readUInt16LE(12) === (expected.target === "esp32p4" ? 0x12 : 0x20),
  `${name} image has the wrong chip ID`);
  requireCheck(data.readUInt16LE(15) === expected.minimum, `${name} image has the wrong minimum revision`);
}

const partitions = [];
for (let offset = 0; offset + 32 <= partitionTable.length; offset += 32) {
  if (partitionTable.readUInt16LE(offset) !== 0x50aa) break;
  partitions.push({
    type: partitionTable[offset + 2], subtype: partitionTable[offset + 3],
    offset: partitionTable.readUInt32LE(offset + 4), size: partitionTable.readUInt32LE(offset + 8),
  });
}
const apps = partitions.filter(partition => partition.type === 0);
const ota = apps.filter(partition => partition.subtype === 0x10 || partition.subtype === 0x11);
requireCheck(ota.length === 2 && apps.every(partition => image.length <= partition.size),
  "Application does not fit every application/OTA partition");
requireCheck(partitions.every(partition => partition.offset + partition.size <= expected.flash * 1024 * 1024),
  "A partition exceeds the configured flash size");
const bootSpace = Number(flasher["partition-table"].offset) - Number(flasher.bootloader.offset);
requireCheck(bootSpace > 0 && bootloader.length <= bootSpace, "Bootloader overlaps the partition table");

const cpu = config.match(/^CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=(\d+)$/m);
requireCheck(cpu && Number(cpu[1]) > 0, "Generated CPU frequency is missing or invalid");
console.log(`PASS: ${profile} native GNSS graph and target/revision image`);
console.log(`PASS: ${expected.flash} MB flash, 80 MHz flash, ${cpu[1]} MHz CPU, -O2, 1 kHz tick`);
console.log(`PASS: application ${image.length} bytes / ${Math.min(...apps.map(partition => partition.size))} bytes; ` +
  `bootloader ${bootloader.length} bytes / ${bootSpace} bytes`);
