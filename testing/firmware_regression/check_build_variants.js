// Syntax-check feature combinations using an existing ESP-IDF build's real compiler and flags.
const fs = require("node:fs");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..", "..");
if (!process.argv[2] || !process.argv[3])
  throw new Error("Usage: node check_build_variants.js <idf-build-directory> <output-directory>");
const build = path.resolve(process.argv[2]);
const out = path.resolve(process.argv[3]);
fs.mkdirSync(out, { recursive: true });
const entries = JSON.parse(fs.readFileSync(path.join(build, "compile_commands.json"), "utf8"));
const entry = entries.find(e => path.resolve(e.file) === path.join(root, "main", "main.cpp"));
if (!entry) throw new Error("main.cpp compilation command not found");
const oteEntry = entries.find(e => path.resolve(e.file) === path.join(root, "main", "custom", "native_ote.cpp"));
if (!oteEntry) throw new Error("native_ote.cpp compilation command not found");
const space = entry.command.indexOf(" ");
const compiler = entry.command.slice(0, space);
const options = entry.command.slice(space + 1).replace(/ -o .+$/, "") + ` -I${quoted(__dirname)}`;
const source = fs.readFileSync(path.join(root, "main", "main.cpp"), "utf8");
const settingsTemplate = fs.readFileSync(path.join(root, "main", "DoNotPublish",
  "ESP32TimeServerSettings - Github.h"), "utf8");
const expectedSecretsIncludes = [
  '#include "DoNotPublish/ESP32TimeServerSettingsSecrets.h"',
  '#include "ESP32TimeServerSettingsSecrets.h"',
];
const secretIncludes = settingsTemplate.match(/^\s*#include\s+["<](?:DoNotPublish[\\/][^">]+|ESP32TimeServerSettingsSecrets\.h)[">]/gm) || [];
if (secretIncludes.length !== 1 || !expectedSecretsIncludes.includes(secretIncludes[0].trim()))
  throw new Error("The shareable settings template must include only the expected testable secrets header");
const settings = settingsTemplate.replace(secretIncludes[0].trim(), '#include "build_variants_secrets.h"');
const oteSource = fs.readFileSync(oteEntry.file, "utf8");
const diagnosticNames = ["determine_stack_sizes", "health_check"];
const diagnosticSources = Object.fromEntries(diagnosticNames.map(name => [
  name, fs.readFileSync(path.join(root, "main", "custom", `${name}.cpp`), "utf8")
]));
const diagnosticHeaders = Object.fromEntries(diagnosticNames.map(name => [
  name, fs.readFileSync(path.join(root, "main", "custom", `${name}.h`), "utf8")
]));
const variants = [
  ...[0, 1, 2, 4, 6, 8, 9].flatMap(preferIPvX => [
    {
      name: `udp-only-ip-${preferIPvX}`, preferIPvX, flags: {
        NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 0, PTP_L3_SERVER_ENABLED: 1,
        MQTT_ENABLED: 0, OTE_UPDATES_ENABLED: 0, STARTUP_HEALTH_TEST_ENABLED: 1,
        CALCULATE_STACK_SIZES_ENABLED: 1
      }
    },
    {
      name: `all-transports-ip-${preferIPvX}`, preferIPvX, flags: {
        NTP_SERVER_ENABLED: 1, PTP_L2_SERVER_ENABLED: 1, PTP_L3_SERVER_ENABLED: 1
      }
    }
  ]),
  { name: "ntp-udp", flags: { PTP_L2_SERVER_ENABLED: 0, PTP_L3_SERVER_ENABLED: 1 } },
  {
    name: "udp-independent-of-l2-auth", flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 0, PTP_L3_SERVER_ENABLED: 1,
      PTP_L2_AUTHENTICATION_ENABLED: 0, PTP_L2_AUTHENTICATION_REQUIRED: 1
    }
  },
  { name: "l2-udp", flags: { NTP_SERVER_ENABLED: 0, PTP_L3_SERVER_ENABLED: 1 } },
  {
    name: "udp-l2-only-mode-rejected", preferIPvX: 7,
    flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1, PTP_L3_SERVER_ENABLED: 1,
      MQTT_ENABLED: 0, OTE_UPDATES_ENABLED: 0
    }, rejected: true,
    error: /PreferIPvX = 7 requires/
  },
  {
    name: "invalid-udp-domain", flags: { PTP_L3_SERVER_ENABLED: 1, PTP_L3_UDP_DOMAIN: 128 },
    rejected: true, error: /Invalid PTP L3 UDP settings/
  },
  {
    name: "invalid-udp-enable", flags: { PTP_L3_SERVER_ENABLED: 2 },
    rejected: true, error: /Time service enable settings must each be/
  },
  { name: "default", flags: {} },
  { name: "mqtt-no-tf", flags: { MQTT_ENABLED: 1, MQTT_USE_TF_CARD: 0 } },
  {
    name: "mqtt-invalid-tf-setting", flags: { MQTT_ENABLED: 1, MQTT_USE_TF_CARD: 2 },
    rejected: true, error: /MQTT_USE_TF_CARD must be 0 or 1/
  },
  { name: "ptp-legacy-selection", flags: { PTP_L2_MASTER_SELECTION_ENABLED: 0 } },
  {
    name: "ptp-legacy-selection-auth", flags: {
      PTP_L2_MASTER_SELECTION_ENABLED: 0,
      PTP_L2_AUTHENTICATION_ENABLED: 1, PTP_L2_AUTHENTICATION_REQUIRED: 1
    }
  },
  { name: "ptp-selection-capacity-limit", flags: { PTP_L2_FOREIGN_MASTER_CAPACITY: 32 } },
  {
    name: "invalid-ptp-selection", flags: { PTP_L2_MASTER_SELECTION_ENABLED: 2 },
    rejected: true, error: /Invalid PTP master-selection settings/
  },
  {
    name: "invalid-ptp-selection-capacity", flags: { PTP_L2_FOREIGN_MASTER_CAPACITY: 0 },
    rejected: true, error: /Invalid PTP master-selection settings/
  },
  { name: "ptp-auth-mixed", flags: { PTP_L2_AUTHENTICATION_ENABLED: 1 } },
  {
    name: "ptp-auth-required", flags: {
      PTP_L2_AUTHENTICATION_ENABLED: 1,
      PTP_L2_AUTHENTICATION_REQUIRED: 1
    }
  },
  {
    name: "ntp-auth-required", flags: {
      NTP_AUTHENTICATION_ENABLED: 1,
      NTP_AUTHENTICATION_REQUIRED: 1, PTP_L2_AUTHENTICATION_ENABLED: 1
    }
  },
  {
    name: "both-auth-required-health", flags: {
      NTP_AUTHENTICATION_ENABLED: 1,
      NTP_AUTHENTICATION_REQUIRED: 1, PTP_L2_AUTHENTICATION_ENABLED: 1,
      PTP_L2_AUTHENTICATION_REQUIRED: 1, STARTUP_HEALTH_TEST_ENABLED: 1
    }
  },
  {
    name: "ptp-auth-only", flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1,
      NTP_AUTHENTICATION_ENABLED: 0, PTP_L2_AUTHENTICATION_ENABLED: 1
    }
  },
  {
    name: "ntp-required-without-auth", flags: {
      NTP_SERVER_ENABLED: 1,
      NTP_AUTHENTICATION_ENABLED: 0, NTP_AUTHENTICATION_REQUIRED: 1
    },
    rejected: true, error: /NTP authenticated-only policy requires/
  },
  {
    name: "ptp-required-without-auth", flags: {
      PTP_L2_SERVER_ENABLED: 1,
      PTP_L2_AUTHENTICATION_ENABLED: 0, PTP_L2_AUTHENTICATION_REQUIRED: 1
    },
    rejected: true, error: /PTP authenticated-only policy requires/
  },
  {
    name: "ntp-only", flags: {
      NTP_SERVER_ENABLED: 1, PTP_L2_SERVER_ENABLED: 0,
      STARTUP_HEALTH_TEST_ENABLED: 1
    }
  },
  {
    name: "ptp-only", flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1,
      STARTUP_HEALTH_TEST_ENABLED: 1
    }
  },
  {
    name: "ptp-only-minimal", flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1,
      MQTT_ENABLED: 0, OTE_UPDATES_ENABLED: 0, NTP_AUTHENTICATION_ENABLED: 0,
      LIQUID_CRYSTAL_DISPLAY_ENABLED: 1,
      STARTUP_HEALTH_TEST_ENABLED: 1, CALCULATE_STACK_SIZES_ENABLED: 1
    }, preferIPvX: 7
  },
  ...[0, 1, 2, 4, 6, 8, 9].map(mode => ({
    name: `ptp-only-invalid-ip-mode-${mode}`, flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1,
      MQTT_ENABLED: 0, OTE_UPDATES_ENABLED: 0
    }, preferIPvX: mode, rejected: true,
    error: /PTP-only operation with OTE and MQTT disabled requires PreferIPvX = 7/,
  })),
  ...["NTP_SERVER_ENABLED", "OTE_UPDATES_ENABLED", "MQTT_ENABLED"].map(feature => ({
    name: `l2-conflicts-with-${feature.toLowerCase()}`,
    flags: {
      NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 1, MQTT_ENABLED: 0,
      OTE_UPDATES_ENABLED: 0, [feature]: 1
    }, preferIPvX: 7, rejected: true,
    error: /PreferIPvX = 7 requires PTP enabled and NTP, OTE, and MQTT disabled/,
  })),
  {
    name: "all-time-services-disabled",
    flags: { NTP_SERVER_ENABLED: 0, PTP_L2_SERVER_ENABLED: 0, PTP_L3_SERVER_ENABLED: 0 },
    rejected: true,
    error: /At least one time service \(NTP or PTP\) must be enabled/,
  },
  { name: "ipv4-only", flags: {}, preferIPvX: 1 },
  { name: "ipv6-only", flags: {}, preferIPvX: 2 },
  ...[0, 1, 2].map(mqttQos => ({
    name: `mqtt-qos-${mqttQos}`, flags: { MQTT_ENABLED: 1 }, mqttQos
  })),
  ...[0, 1, 2].map(mqttQos => ({
    name: `external-tf-qos-${mqttQos}`, flags: {
      S31_EXTERNAL_TF_CARD_ENABLED: 1, MQTT_ENABLED: 1, DEBUG_ENABLED: 1,
      LIQUID_CRYSTAL_DISPLAY_ENABLED: 1, UPTIME_RESTART_BUTTON_ENABLED: 1,
      RBG_LED_KY_016_MODULE_ENABLED: 1, RBG_LED_WS2812_MODULE_ENABLED: 0
    }, mqttQos
  })),
  {
    name: "external-tf-no-mqtt", flags: {
      S31_EXTERNAL_TF_CARD_ENABLED: 1, MQTT_ENABLED: 0
    }
  },
  {
    name: "debug-lcd-ubx-loopback", flags: {
      DEBUG_ENABLED: 1, LIQUID_CRYSTAL_DISPLAY_ENABLED: 1,
      UPTIME_RESTART_BUTTON_ENABLED: 1, UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED: 1
    }, loopback: true
  },
  {
    name: "ky016", flags: {
      DEBUG_ENABLED: 1, RBG_LED_KY_016_MODULE_ENABLED: 1,
      RBG_LED_WS2812_MODULE_ENABLED: 0, LIQUID_CRYSTAL_DISPLAY_ENABLED: 1
    }
  },
  {
    name: "ws2812", flags: {
      RBG_LED_KY_016_MODULE_ENABLED: 0, RBG_LED_WS2812_MODULE_ENABLED: 1
    }
  },
  {
    name: "minimal", flags: {
      MQTT_ENABLED: 0, OTE_UPDATES_ENABLED: 0,
      NTP_AUTHENTICATION_ENABLED: 0, RBG_LED_WS2812_MODULE_ENABLED: 0,
      PPS_ISR_LATENCY_AUTO: 0, NTP_HW_TX_TIMESTAMPS: 0
    }
  },
  {
    name: "legacy-calibration", flags: {
      DEBUG_ENABLED: 1, PTP_FREE_RUNNING: 0,
      PTP_LATENCY_AUTO: 1, FINE_LOCAL_TIMEBASE: 0, LIQUID_CRYSTAL_DISPLAY_ENABLED: 1
    }
  },
  { name: "legacy-no-calibration", flags: { PTP_FREE_RUNNING: 0, PPS_ISR_LATENCY_AUTO: 0 } },
  {
    name: "health-and-stack-diagnostics", flags: {
      DEBUG_ENABLED: 1,
      STARTUP_HEALTH_TEST_ENABLED: 1, CALCULATE_STACK_SIZES_ENABLED: 1
    }
  },
  {
    name: "health-no-auth-or-mqtt", flags: {
      STARTUP_HEALTH_TEST_ENABLED: 1,
      CALCULATE_STACK_SIZES_ENABLED: 1, NTP_AUTHENTICATION_ENABLED: 0,
      MQTT_ENABLED: 0
    }
  },
  {
    name: "health-no-auth", flags: {
      STARTUP_HEALTH_TEST_ENABLED: 1,
      NTP_AUTHENTICATION_ENABLED: 0
    }
  },
  {
    name: "health-no-client-reporting", flags: {
      STARTUP_HEALTH_TEST_ENABLED: 1,
      MQTT_CLIENT_REPORTING_ENABLED: 0
    }
  },
  {
    name: "stack-forces-debug", flags: {
      DEBUG_ENABLED: 0,
      CALCULATE_STACK_SIZES_ENABLED: 1
    }
  },
  {
    name: "legacy-offset-measurement-off", flags: {
      DEBUG_ENABLED: 1,
      PTP_FREE_RUNNING: 0, PTP_OFFSET_MEASUREMENT: 0
    }
  },
  {
    name: "non-ublox-lcd", flags: {
      DEBUG_ENABLED: 1,
      UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED: 0, LIQUID_CRYSTAL_DISPLAY_ENABLED: 1
    }
  },
  {
    name: "non-ublox-quiet", flags: {
      DEBUG_ENABLED: 0,
      UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED: 0, LIQUID_CRYSTAL_DISPLAY_ENABLED: 0
    }
  },
  {
    name: "delayed-psram", flags: {},
    sdkDefines: ["CONFIG_SPIRAM=1", "CONFIG_SPIRAM_BOOT_INIT=0", "CONFIG_SPIRAM_BOOT_HW_INIT=1",
      "CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=1", "CONFIG_SPIRAM_USE_MALLOC=1",
      "CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384"]
  },
  {
    name: "delayed-psram-isr-iram", flags: {},
    sdkDefines: ["CONFIG_SPIRAM=1", "CONFIG_SPIRAM_BOOT_INIT=0", "CONFIG_SPIRAM_BOOT_HW_INIT=1",
      "CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=1", "CONFIG_SPIRAM_USE_MALLOC=1",
      "CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384", "CONFIG_ARDUINO_ISR_IRAM=1"]
  },
  {
    name: "delayed-psram-without-protection", flags: {},
    sdkDefines: ["CONFIG_SPIRAM=1", "CONFIG_SPIRAM_BOOT_INIT=0", "CONFIG_SPIRAM_BOOT_HW_INIT=1",
      "CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=0"],
    rejected: true, error: /Delayed PSRAM initialization requires CONFIG_SPIRAM_BOOT_HW_INIT/
  },
  {
    name: "delayed-psram-without-boot-hardware", flags: {},
    sdkDefines: ["CONFIG_SPIRAM=1", "CONFIG_SPIRAM_BOOT_INIT=0", "CONFIG_SPIRAM_BOOT_HW_INIT=0",
      "CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=1"],
    rejected: true, error: /Delayed PSRAM initialization requires CONFIG_SPIRAM_BOOT_HW_INIT/
  },
];
function configure(text, flags) {
  for (const [name, value] of Object.entries(flags))
    text = text.replace(new RegExp(`(#define ${name} )\\d+`, "g"), `$1${value}`);
  return text;
}
function quoted(value) { return `"${value.replace(/\\/g, "\\\\")}"`; }
for (const variant of variants) {
  const header = path.join(out, `${variant.name}_settings.h`);
  let headerText = configure(settings, { PTP_L3_SERVER_ENABLED: 0, ...variant.flags });
  if (variant.preferIPvX !== undefined)
    headerText = headerText.replace(/(PreferIPvX = )\d+/, `$1${variant.preferIPvX}`);
  if (variant.mqttQos !== undefined) {
    if (!/(MQTT_QOS = )\d+/.test(headerText))
      throw new Error("MQTT_QOS setting not found");
    headerText = headerText.replace(/(MQTT_QOS = )\d+/, `$1${variant.mqttQos}`);
  }
  if (variant.loopback) headerText = headerText.replace("PTPPPSLoopbackPin = -1", "PTPPPSLoopbackPin = 23");
  fs.writeFileSync(header, headerText);
  const diagnosticHeaderPaths = Object.fromEntries(diagnosticNames.map(name => [
    name, path.join(out, `${variant.name}_${name}.h`)
  ]));
  function diagnosticIncludes(text) {
    for (const name of diagnosticNames)
      text = text.replace(new RegExp(`"(?:custom/)?${name}\\.h"`, "g"),
        `"${diagnosticHeaderPaths[name]}"`);
    return text.replace('"ESP32TimeServerSettings.h"', `"${header}"`);
  }
  for (const name of diagnosticNames)
    fs.writeFileSync(diagnosticHeaderPaths[name],
      diagnosticIncludes(configure(diagnosticHeaders[name], variant.flags)));
  const file = path.join(out, `${variant.name}.cpp`);
  fs.writeFileSync(file, diagnosticIncludes(configure(source, variant.flags)));
  const response = path.join(out, `${variant.name}.rsp`);
  const sdkDefines = (variant.sdkDefines || []).map(definition => `-D${definition}`).join(" ");
  fs.writeFileSync(response, `${options} ${sdkDefines} -fsyntax-only ${quoted(file)}`);
  const result = spawnSync(compiler, [`@${response}`], { cwd: entry.directory, encoding: "utf8" });
  if (result.error) throw result.error;
  if (variant.rejected) {
    if (result.status === 0 || !(variant.error || /At least one time service.*must be enabled/).test(result.stderr))
      throw new Error(`Invalid configuration was not explicitly rejected: ${variant.name}`);
    console.log(`PASS: ${variant.name} rejected`);
    continue;
  }
  if (result.status !== 0) {
    process.stderr.write(result.stderr);
    process.exit(result.status || 1);
  }
  for (const [protocol, enabled] of [
    ["ntp", (variant.flags.NTP_SERVER_ENABLED ?? 1) && (variant.flags.NTP_AUTHENTICATION_ENABLED ?? 1)],
    ["ptp_l2", (variant.flags.PTP_L2_SERVER_ENABLED ?? 1) && (variant.flags.PTP_L2_AUTHENTICATION_ENABLED ?? 0)],
  ]) {
    if (!enabled) continue;
    const authFile = path.join(out, `${variant.name}_${protocol}_auth.cpp`);
    fs.writeFileSync(authFile, fs.readFileSync(path.join(root, "main", "custom", `${protocol}_auth.cpp`), "utf8")
      .replace('"ESP32TimeServerSettings.h"', `"${header}"`)
      .replace(`"${protocol}_auth.h"`, `"${path.join(root, "main", "custom", `${protocol}_auth.h`)}"`));
    const authResponse = path.join(out, `${variant.name}_${protocol}_auth.rsp`);
    fs.writeFileSync(authResponse, `${options} -fsyntax-only ${quoted(authFile)}`);
    const authResult = spawnSync(compiler, [`@${authResponse}`], { cwd: entry.directory, encoding: "utf8" });
    if (authResult.error) throw authResult.error;
    if (authResult.status !== 0) {
      process.stderr.write(authResult.stderr);
      process.exit(authResult.status || 1);
    }
  }
  for (const name of diagnosticNames) {
    const diagnosticFile = path.join(out, `${variant.name}_${name}.cpp`);
    fs.writeFileSync(diagnosticFile, diagnosticIncludes(diagnosticSources[name])
      .replace('"ntp_auth.h"', `"${path.join(root, "main", "custom", "ntp_auth.h")}"`));
    const diagnosticResponse = path.join(out, `${variant.name}_${name}.rsp`);
    fs.writeFileSync(diagnosticResponse, `${options} -fsyntax-only ${quoted(diagnosticFile)}`);
    const diagnosticResult = spawnSync(compiler, [`@${diagnosticResponse}`],
      { cwd: entry.directory, encoding: "utf8" });
    if (diagnosticResult.error) throw diagnosticResult.error;
    if (diagnosticResult.status !== 0) {
      process.stderr.write(diagnosticResult.stderr);
      process.exit(diagnosticResult.status || 1);
    }
  }
  const oteFile = path.join(out, `${variant.name}_ote.cpp`);
  fs.writeFileSync(oteFile, oteSource
    .replace('"ESP32TimeServerSettings.h"', `"${header}"`)
    .replace('"native_ote.h"', `"${path.join(root, "main", "custom", "native_ote.h")}"`));
  const oteSpace = oteEntry.command.indexOf(" ");
  const oteCompiler = oteEntry.command.slice(0, oteSpace);
  const oteOptions = oteEntry.command.slice(oteSpace + 1).replace(/ -o .+$/, "") + ` -I${quoted(__dirname)}`;
  const oteResponse = path.join(out, `${variant.name}_ote.rsp`);
  fs.writeFileSync(oteResponse, `${oteOptions} -fsyntax-only ${quoted(oteFile)}`);
  const oteResult = spawnSync(oteCompiler, [`@${oteResponse}`], { cwd: oteEntry.directory, encoding: "utf8" });
  if (oteResult.error) throw oteResult.error;
  if (oteResult.status !== 0) {
    process.stderr.write(oteResult.stderr);
    process.exit(oteResult.status || 1);
  }
  console.log(`PASS: ${variant.name}`);
}
