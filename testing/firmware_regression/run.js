// Run from a Visual Studio x64 Native Tools prompt: node run.js <output-directory>
const fs = require("node:fs");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const crypto = require("node:crypto");

const root = path.resolve(__dirname, "..", "..");
const output = process.argv[2];
if (!output) throw new Error("Supply an output directory outside the source tree");
const out = path.resolve(output);
fs.mkdirSync(out, { recursive: true });
const source = fs.readFileSync(path.join(root, "main", "main.cpp"), "utf8").replace(/\r\n/g, "\n");
function section(start, end) {
  const a = source.indexOf(start);
  const b = source.indexOf(end, a);
  if (a < 0 || b <= a) throw new Error(`Source section not found: ${start}`);
  return source.slice(a, b);
}
function fn(signature) {
  return section(signature, "\n}\n") + "\n}\n";
}
function write(name, value) {
  const target = path.join(out, name);
  fs.mkdirSync(path.dirname(target), { recursive: true });
  fs.writeFileSync(target, value);
}

write("lwip\\inet.h", "#pragma once\n#include <stdint.h>\nstruct in6_addr { uint8_t s6_addr[16]; };\n");
write("esp_timer.h", "#pragma once\n#include <stdint.h>\n#ifdef __cplusplus\nextern \"C\" {\n#endif\nint64_t esp_timer_get_time(void);\n#ifdef __cplusplus\n}\n#endif\n");
write("freertos\\FreeRTOS.h", "#pragma once\n#define pdTRUE 1\n#define portMAX_DELAY 0xffffffffu\ntypedef int portMUX_TYPE;\n#define portMUX_INITIALIZER_UNLOCKED 0\n#define portENTER_CRITICAL(m) ((void)(m))\n#define portEXIT_CRITICAL(m) ((void)(m))\n");
write("freertos\\semphr.h", "#pragma once\ntypedef void *SemaphoreHandle_t;\nstatic inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }\nstatic inline int xSemaphoreTake(SemaphoreHandle_t m, unsigned int t) { (void)m; (void)t; return 1; }\nstatic inline void xSemaphoreGive(SemaphoreHandle_t m) { (void)m; }\nstatic inline void vSemaphoreDelete(SemaphoreHandle_t m) { (void)m; }\n");
write("freertos\\task.h", "#pragma once\n");

const functions = [
  "static time_t utc_now()",
  "static void ptp_map_publish(", "static bool ptp_map_offset_at(",
  "static inline uint64_t ntp64_from_unix_ns(", "static uint64_t ntp64_from_tai_ns(",
  "static int64_t get_disciplined_time_in_tai_ns(",
  "static uint64_t get_disciplined_time_in_ntp64_format(",
  "static void write_ntp_timestamp(",
  "static uint32_t ones_complement_add(", "static uint16_t ones_complement_fold(",
  "static uint16_t internet_checksum(", "static uint32_t udp_pseudo_header_sum(",
  "static bool udp_checksum_is_valid(", "static bool is_ipv4_ntp_request(",
  "static bool is_ipv6_extension_header(", "static bool is_ipv6_ntp_request(",
  "static bool hardware_ntp_frame_is_acceptable(", "static uint16_t ipv6_udp_checksum(",
  "static bool load_latency_calibration(", "static bool save_latency_calibration(",
  "static size_t json_append(",
];
write("firmware_under_test.h", [
  `#include "${path.join(root, "main", "custom", "internet_checksum.h").replace(/\\/g, "\\\\")}"`,
  section("#define TIMING_LOGI", "// the following is used"),
  section("struct sync_faults_t", "struct sync_state_t"),
  section("static std::atomic<uint32_t> s_ntp_sync_state_sequence", "static constexpr int64_t Sync_Reboot_After_Us"),
  section("static std::atomic<uint32_t> s_model_seq", "static_assert(PreferIPvX"),
  ...functions.map(fn),
  section("struct ntp_reply_status_t", "static constexpr size_t ETH_HEADER_SIZE"),
  section("struct tx_delay_estimate_t", "#else\nstatic inline uint64_t compensate_basic_t3"),
].join("\n"));
write("startup_under_test.h", fn("static bool wait_for_gnss_startup_qualification("));
write("gnss_app_under_test.h", [
  fn("static void build_candidate_baud_rates("),
  fn("static uint32_t uptime_ms()"),
  fn("static bool reopen_gnss_uart("),
  fn("static bool read_gnss_uart_byte("),
  section("struct nmea_rmc_time_t", "static void refresh_sync_state_locked("),
  ...["static bool is_digit_char(", "static bool parse_two_digits(",
    "static bool get_nmea_field(", "static bool parse_nmea_rmc_sentence("].map(fn),
  section("static constexpr size_t NMEA_GSV_GROUP_LIMIT", "#endif\n\nstruct nmea_sentence_buffer_t"),
  section("struct nmea_sentence_buffer_t", "static bool wait_for_nmea_rmc_time("),
  fn("static bool wait_for_nmea_rmc_time("),
  section("struct gnss_probe_result_t", "static bool wait_for_gnss_pvt("),
  fn("static gnss_probe_result_t probe_gnss_uart(uint32_t baud)\n"),
  fn("static bool try_gnss_begin("),
  fn("static bool confirm_saved_gnss_baud_rate("),
  fn("static bool find_initial_gnss_baud_rate("),
  fn("static bool change_gnss_baud_rate("),
  fn("static bool set_gnss_baud_rate("),
  fn("static bool check_uptime_request()"),
].join("\n"));
write("native_startup_under_test.h", [
  section("#if CONFIG_SPIRAM && !CONFIG_SPIRAM_BOOT_INIT && (!CONFIG_SPIRAM_BOOT_HW_INIT",
    "#endif\n") + "#endif\n",
  fn("static bool initialize_nvs_storage()"),
  fn("static esp_err_t configure_console()"),
  fn("static void initialize_delayed_psram()"),
  fn("static void confirm_running_ota_image()"),
  fn('extern "C" void app_main()'),
].join("\n"));
write("pps_capture_under_test.h", [
  fn("static bool ptp_pps_output_available("),
  fn("static void setup_pps_input()"),
  fn("static void setup_ptp_pps_capture()"),
  fn("static void setup_ptp_pps_loopback(esp_eth_mac_t *mac)\n"),
].join("\n"));
write("sync_under_test.h", [
  fn("static int64_t days_from_civil("),
  fn("static time_t epoch_from_utc("),
  fn("static bool publish_leap_information("),
  fn("static bool refresh_leap_information("),
  fn("static bool wait_for_gnss_pvt("),
  fn("static bool current_gnss_timing_is_valid()"),
  fn("static bool acquire_sync_candidate("),
  fn("static bool first_sync_candidates_are_plausible("),
].join("\n"));
write("stationary_mode_under_test.h", section(
  "#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED\n        if (first_sync)",
  "\n#if DEBUG_ENABLED\n        char date_string[16]"));
write("timestamp_under_test.h", [
  section("#define TIMING_LOGI", "// the following is used"),
  ...[
    "static inline uint64_t ntp64_from_unix_ns(",
    "static uint64_t ntp64_from_tai_ns(",
    "static void ptp_comp_publish(", "static bool ptp_comp_read(",
    "static void ptp_map_publish(", "static bool ptp_map_offset_at(",
    "static int64_t tai_ns_from_hardware_timestamp(",
    "static uint64_t ntp64_from_hardware_timestamp(", "static bool timestamp_is_valid(",
    "static void record_hw_tx_timestamp_result(", "static esp_err_t transmit_with_hw_timestamp(",
    "static bool ptp_write_timestamp(",
  ].map(fn),
].join("\n"));
write("transport_under_test.h", fn("static void ethernet_transport_recovery_task("));
write("load_under_test.h", [
  section("static constexpr uint32_t NTP_ADMISSION_BURST_REQUESTS", "#ifndef UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED"),
  fn("static bool ntp_load_shedding_active("),
  fn("static void record_ntp_input("),
  fn("static void record_ntp_load_shedding("),
  fn("static bool admit_ntp_request("),
].join("\n"));
write("ingress_under_test.h", fn("static esp_err_t ntp_ethernet_input("));
write("recovery_constants_under_test.h",
  section("static constexpr uint32_t ETHERNET_RECOVERY_DELAY_MS", "#if NTP_SERVER_ENABLED\nstruct hardware_ntp_request_t"));
write("recovery_under_test.h", [
  section("enum class ethernet_recovery_result_t", "static void ethernet_recovery_worker_task("),
  fn("static void ethernet_recovery_worker_task("),
  fn("static ethernet_recovery_result_t recover_ethernet_with_timeout("),
].join("\n"));
write("network_under_test.h", [
  fn("static void ethernet_event_handler("),
  fn("static void ethernet_ip_event_handler("),
  fn("static bool configure_static_ip()\n"),
  fn("static void attach_ethernet_network()"),
].join("\n"));
const macAddressFunctions = [
  fn("static esp_err_t read_active_ethernet_mac(uint8_t mac[6])\n{"),
  fn("static void format_mac_address(const uint8_t mac[6], char *text, size_t text_size)\n{"),
  fn("static esp_err_t log_active_ethernet_mac()\n{"),
  fn("static esp_err_t configure_mac_address()\n{"),
].join("\n");
if (/\besp_efuse_mac_get_default\s*\(/.test(macAddressFunctions))
  throw new Error("MAC configuration must not use the expanded eFuse MAC API");
write("mac_address_under_test.h", macAddressFunctions);
write("ethernet_startup_under_test.h",
  section("    ESP_ERROR_CHECK(esp_eth_driver_install(", "\n}\n"));
const settingsTemplate = fs.readFileSync(path.join(root, "main", "DoNotPublish",
  "ESP32TimeServerSettings - Github.h"), "utf8").replace(/\r\n/g, "\n");
const mqttSettingInteger = name => {
  const match = settingsTemplate.match(new RegExp(`(?:^#define ${name} |^static constexpr int ${name} = )(\\d+)`, "m"));
  if (!match) throw new Error(`${name} not found in the settings template`);
  return Number(match[1]);
};
const mqttL2FollowerCapacity = mqttSettingInteger("PTP_L2_FOLLOWER_CAPACITY");
const mqttL3FollowerCapacity = mqttSettingInteger("PTP_L3_UDP_FOLLOWER_CAPACITY");
const mqttPreferIPvX = mqttSettingInteger("PreferIPvX");
const boardSettingsStart = settingsTemplate.indexOf("#if CONFIG_IDF_TARGET_ESP32P4\n#define TIME_SERVER_BOARD_HAS_TF_CARD 1\n");
if (boardSettingsStart < 0) throw new Error("Target-specific board settings not found");
write("board_pins_fixture.h", settingsTemplate.slice(boardSettingsStart)
  .replace(/(#define S31_EXTERNAL_TF_CARD_ENABLED )\d+/, "$1TEST_S31_EXTERNAL_TF_CARD_ENABLED"));
write("board_under_test.h", [
  section("static constexpr uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold_ESP32P4_ChipVersion1_3",
    "static uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold ="),
  section("#if FINE_LOCAL_TIMEBASE &&", "// Precision announced in NTP replies"),
  fn("static bool ptp_pps_output_available("),
  fn("static bool ntp_threshold_for_chip("),
  section("struct board_pin_t", "static esp_err_t validate_chip_configuration("),
  fn("static esp_err_t validate_chip_configuration("),
  "#if CONFIG_IDF_TARGET_ESP32S31\n" + fn("static esp_err_t yt8531_phy_init(") + "\n#endif\n",
].join("\n"));
write("tf_card_under_test.h", [
  section("struct mqtt_tf_queue_snapshot_t", "static bool mqtt_tf_queue_files("),
  fn("static void setup_mqtt_tf_storage()"),
  fn("static void setup_mqtt_tf_queue()"),
].join("\n"));
write("mqtt_storage_under_test.h", [
  section("using mqtt_ram_queue_t", "static SemaphoreHandle_t s_mqtt_report_mutex"),
  fn("static void *mqtt_allocate_queue_storage("),
  fn("static bool mqtt_allocate_ram_queue()"),
  fn("static void mqtt_fallback_from_tf()"),
  section("struct mqtt_tf_queue_snapshot_t", "static bool mqtt_tf_queue_files("),
  fn("static bool mqtt_tf_queue_files("),
  fn("static bool mqtt_tf_refresh_queue_count()"),
  fn("static bool mqtt_tf_write_report("),
  fn("static bool mqtt_enqueue_report("),
  fn("static void mqtt_send_queued_messages()"),
].join("\n"));
write("mqtt_storage_status_under_test.h", [
  fn("static int get_required_top_line_message()"),
  fn("static void control_RGB_LED(RGB_LED_Colour Colour, bool enabled)\n{"),
  fn("static void update_RGB_LED()"),
  "static std::string lcd_top_line(int required_top_line_message)\n{\n" +
  section("                char top_line_message[21];", "                display_line(0, top_line_message);") +
  "return top_line_message;\n}\n",
].join("\n"));
write("emac_config_under_test.h", section(
  "    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();",
  "    esp_eth_mac_t *mac = esp_eth_mac_new_esp32"));
write("s31_lcd_under_test.h", [
  fn("static esp_err_t lcd_try_device_address("),
  fn("static esp_err_t setup_lcd()"),
  fn("static void display_line(uint8_t row, const char *text)"),
].join("\n"));
write("network_modes_under_test.h", [
  "#if PTP_L2_SERVER_ENABLED\n" + [
    fn("static esp_err_t read_active_ethernet_mac(uint8_t mac[6])\n{"),
    fn("static void format_mac_address(const uint8_t mac[6], char *text, size_t text_size)\n{"),
  ].join("\n") + "\n#endif\n",
  section("static_assert(PreferIPvX", "static char s_ip_address"),
  section("static constexpr bool L2_ONLY", "static void update_selected_ip_address_locked()"),
  fn("static void display_selected_ip_address("),
  fn("void perform_health_check()"),
].join("\n"));
write("restart_under_test.h", [
  section("#define TIMING_LOGI", "// the following is used"),
  fn("static void restart_deadline_callback("),
  fn("static void controlled_restart(const char *reason)\n"),
].join("\n"));
const mqttFunction = fn("static bool mqtt_build_report(");
write("mqtt_report_limits_under_test.h", "#define MQTT_USE_TF_CARD 1\n" + section("// The report buffer is sized from the client limits",
  "static constexpr size_t MQTT_NTP_EVENT_QUEUE_DEPTH"));
{
  const start = settingsTemplate.indexOf("static constexpr size_t MQTT_NTP_CLIENT_REPORT_LIMIT");
  const last = settingsTemplate.indexOf("static constexpr size_t MQTT_RAM_QUEUE_BYTES");
  if (start < 0 || last < start) throw new Error("MQTT report limit settings not found");
  write("mqtt_report_settings_under_test.h", [
    `#define PTP_L2_FOLLOWER_CAPACITY ${mqttL2FollowerCapacity}`,
    `#define PTP_L3_UDP_FOLLOWER_CAPACITY ${mqttL3FollowerCapacity}`,
    `static constexpr int PreferIPvX = ${mqttPreferIPvX};`,
    settingsTemplate.slice(start, settingsTemplate.indexOf("\n", last) + 1),
  ].join("\n"));
}
const mqttNames = new Set(mqttFunction.match(/\bs_\w+/g));
mqttNames.add("s_ptp_service_active");
const mqttMetrics = [];
const mqttCounters = [];
for (const name of mqttNames) {
  const match = source.match(new RegExp(`static std::atomic<([^>]+)>\\s+${name}\\{[^}]*\\};`));
  if (match) mqttMetrics.push(`static std::atomic<${match[1]}> ${name}{};`);
  if (match && match[1] === "uint32_t") mqttCounters.push(`    ${name}.store(UINT32_MAX);`);
}
mqttMetrics.push(`static void mqtt_metrics_set_maximum()\n{\n${mqttCounters.join("\n")}\n}`);
write("mqtt_metrics_under_test.h", mqttMetrics.join("\n"));
write("mqtt_report_under_test.h", fn("static size_t json_append(") + mqttFunction);
write("ptp_service_under_test.h", [
  section("static uint8_t s_ptp_source_mac", "#if PTP_L2_SERVER_ENABLED\nstatic constexpr"),
  section("static constexpr uint8_t PTP_L2_MULTICAST_MAC", "static void ptp_write_u16(").replace(/\n#endif\s*$/, ""),
  section("class ptp_clock_lock_t", "static bool ptp_time_qualified("),
  ...["static void ptp_write_u16(", "static void ptp_write_sync_origin(", "static bool ptp_time_qualified(",
    "static void ptp_build_message(", "static bool ptp_transmit_raw(",
    "static void ptp_l2_build_frame(", "static bool ptp_l2_transmit(",
    "static bool ptp_l2_receive_frame(", "static ptp::MessageKind ptp_l2_parse_received_frame(",
    "static bool ptp_l2_service_qualified(", "static void ptp_l2_service_task("].map(fn),
].join("\n"));
write("ptp_startup_under_test.h", fn("static bool start_ptp_l2_service()"));
write("ptp_shared_under_test.h", [
  section("static uint8_t s_ptp_source_mac", "#if PTP_L2_SERVER_ENABLED\nstatic constexpr"),
  section("class ptp_clock_lock_t", "static bool ptp_time_qualified("),
  ...["static void ptp_write_u16(", "static void ptp_write_sync_origin(", "static bool ptp_time_qualified(",
    "static void ptp_build_message(", "static bool ptp_transmit_raw("].map(fn),
].join("\n"));
write("leap_fixture.h", `
#include <ctime>
#include "${path.join(root, "main", "custom", "leap_seconds.h").replace(/\\/g, "\\\\")}"
static timescale::LeapState s_leap_state;
[[maybe_unused]] static std::atomic<bool> s_timescale_requalification_required{false};
[[maybe_unused]] static time_t s_test_tai_seconds = 1700000000;
[[maybe_unused]] static time_t test_time(time_t *) { return s_test_tai_seconds; }
#define time test_time
static void set_test_leap(int64_t now_us) {
  s_leap_state.publish({true, 37, 0, 0, now_us});
}
`);

function run(command, args) {
  if (command === "cl.exe") {
    if (!args.some(value => value.startsWith("/DPTP_ENABLED=")))
      args.push("/DPTP_ENABLED=PTP_L2_SERVER_ENABLED");
    if (!args.some(value => value.startsWith("/DPTP_L3_SERVER_ENABLED=")))
      args.push("/DPTP_L3_SERVER_ENABLED=0");
  }
  if (command === "cl.exe" && !args.some(value => value.startsWith("/DNTP_SERVER_ENABLED")))
    args.push("/DNTP_SERVER_ENABLED=1", "/DPTP_L2_SERVER_ENABLED=0");
  const result = spawnSync(command, args, { cwd: out, encoding: "utf8", stdio: "inherit" });
  if (result.error) throw result.error;
  if (result.status !== 0) process.exit(result.status || 1);
}
function runNetworkTests(debug) {
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`,
    path.join(__dirname, "network_test.cpp"), `/Fenetwork-debug-${debug}.exe`]);
  run(path.join(out, `network-debug-${debug}.exe`), []);
}
function runMacAddressTests() {
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    "/DDEBUG_ENABLED=1", `/I${out}`,
    path.join(__dirname, "mac_address_test.cpp"), "/Femac-address-tests.exe"]);
  run(path.join(out, "mac-address-tests.exe"), []);
}
if (process.argv.includes("--mac-only")) {
  runMacAddressTests();
  process.exit(0);
}
function runNetworkModeTests(debug = 0) {
  for (const mode of [0, 1, 2, 4, 6, 7, 8, 9]) {
    for (const columns of mode === 7 ? [20, 22] : [20]) {
      for (const health of mode === 7 ? [0, 1] : [1]) {
        const name = `network-mode-${mode}-lcd-${columns}-health-${health}-debug-${debug}`;
        run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4100", "/wd4127", "/std:c++17",
          "/D_CRT_SECURE_NO_WARNINGS",
          `/DDEBUG_ENABLED=${debug}`,
          `/DTEST_NETWORK_MODE=${mode}`, `/DTEST_LCD_COLUMNS=${columns}`,
          `/DSTARTUP_HEALTH_TEST_ENABLED=${health}`,
          `/DNTP_SERVER_ENABLED=${mode === 7 ? 0 : 1}`,
          `/DPTP_L2_SERVER_ENABLED=${mode === 7 || mode === 0 ? 1 : 0}`,
          `/I${out}`, path.join(__dirname, "network_modes_test.cpp"), `/Fe${name}.exe`]);
        run(path.join(out, `${name}.exe`), []);
      }
    }
  }
}
function runBoardTests() {
  for (const [target, qos, tf] of [
    ["p4", 1, 0], ["s31", 0, 0], ["s31", 1, 0], ["s31", 2, 0],
    ["s31", 0, 1], ["s31", 1, 1], ["s31", 2, 1],
  ]) {
    const name = `board-${target}-qos-${qos}-tf-${tf}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/std:c++17", `/I${out}`,
      `/DTEST_MQTT_QOS=${qos}`, `/DTEST_S31_EXTERNAL_TF_CARD_ENABLED=${tf}`,
      `/DCONFIG_IDF_TARGET_ESP32P4=${target === "p4" ? 1 : 0}`,
      `/DCONFIG_IDF_TARGET_ESP32S31=${target === "s31" ? 1 : 0}`,
      "/DCONFIG_ESP32P4_REV_MIN_FULL=300", "/DCONFIG_ESP32P4_REV_MAX_FULL=399",
      "/DCONFIG_ESP32S31_REV_MIN_FULL=0", "/DCONFIG_ESP32S31_REV_MAX_FULL=99",
      path.join(__dirname, "board_support_test.cpp"), `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
  for (const [target, tf] of [["p4", 0], ["s31", 0], ["s31", 1]]) {
    for (const qos of [0, 1, 2]) {
      for (const [debug, useTf] of [[0, 0], [1, 0], [0, 1], [1, 1]]) {
        const name = `tf-${target}-enabled-${tf}-qos-${qos}-debug-${debug}-use-${useTf}`;
        run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/std:c++17",
          "/D_CRT_SECURE_NO_WARNINGS", `/I${out}`, `/DTEST_MQTT_QOS=${qos}`,
          `/DDEBUG_ENABLED=${debug}`, `/DMQTT_USE_TF_CARD=${useTf}`, `/DTEST_S31_EXTERNAL_TF_CARD_ENABLED=${tf}`,
          `/DCONFIG_IDF_TARGET_ESP32P4=${target === "p4" ? 1 : 0}`,
          `/DCONFIG_IDF_TARGET_ESP32S31=${target === "s31" ? 1 : 0}`,
          path.join(__dirname, "tf_card_test.cpp"), `/Fe${name}.exe`]);
        run(path.join(out, `${name}.exe`), []);
      }
    }
  }
  for (const qos of [0, 1, 2]) {
    for (const [debug, abortHeap] of [[0, 0], [1, 0], [0, 1], [1, 1]]) {
      const name = `mqtt-storage-qos-${qos}-debug-${debug}-abort-${abortHeap}`;
      run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/wd4244", "/wd4505", "/std:c++17",
        "/D_CRT_SECURE_NO_WARNINGS", `/I${out}`, `/I${path.join(root, "main", "custom")}`,
        `/DTEST_MQTT_QOS=${qos}`, `/DDEBUG_ENABLED=${debug}`, `/DCONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS=${abortHeap}`,
        path.join(__dirname, "mqtt_storage_test.cpp"), `/Fe${name}.exe`]);
      run(path.join(out, `${name}.exe`), []);
    }
  }
  for (const [columns, rows] of [[20, 4], [10, 2]]) {
    const name = `s31-lcd-${columns}x${rows}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/std:c++17",
      `/DLCD_TEST_COLUMNS=${columns}`, `/DLCD_TEST_ROWS=${rows}`, `/I${out}`,
      path.join(__dirname, "s31_lcd_test.cpp"), `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
}
if (process.argv.includes("--board-only")) {
  runBoardTests();
  process.exit(0);
}
if (process.argv.includes("--network-only")) {
  for (const debug of [0, 1]) {
    runNetworkTests(debug);
    runNetworkModeTests(debug);
  }
  process.exit(0);
}
if (!["--gnss-only", "--auth-only", "--ptp-only"].some(option => process.argv.includes(option))) {
  run("python", ["-B", path.join(__dirname, "build_helper_test.py")]);
  console.log("PASS: isolated build profile helper");
  run(process.execPath, [path.join(__dirname, "check_build_graph_test.js")]);
  console.log("PASS: portable native build-graph audit");
  runMacAddressTests();
  runBoardTests();
}
function runGnssTests() {
  const component = path.join(root, "main", "custom");
  const stubs = path.join(__dirname, "gnss_uart_stubs");
  const receiverStubs = path.join(__dirname, "gnss_receiver_stubs");
  for (const directory of [path.join(root, "main"), path.join(root, "main", "custom")]) {
    for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
      if (!entry.isFile() || !/\.(?:c|cpp|h|hpp)$/.test(entry.name)) continue;
      const contents = fs.readFileSync(path.join(directory, entry.name), "utf8");
      if (/#\s*include\s*[<"](?:Arduino|HardwareSerial|Stream|SparkFun_u-blox_GNSS_v3)\.h[>"]|\b(?:millis|micros|initArduino|delay)\s*\(|\b(?:HardwareSerial|SFE_UBLOX_GNSS_SERIAL)\b|\bSerial\s*\.|\bCONFIG_ARDUINO_[A-Z0-9_]+\b/.test(contents))
        throw new Error(`Obsolete Arduino or SparkFun usage remains in ${entry.name}`);
    }
  }
  const mainCmake = fs.readFileSync(path.join(root, "main", "CMakeLists.txt"), "utf8");
  const mainManifest = fs.readFileSync(path.join(root, "main", "idf_component.yml"), "utf8");
  if (/arduino-esp32|SparkFun_u-blox_GNSS_v3|time_server_gnss/.test(mainCmake + mainManifest))
    throw new Error("Application still requires a retired GNSS dependency");
  const nativeHeader = fs.readFileSync(path.join(component, "gnss_uart.h"), "utf8");
  if (/\b(?:Arduino|Stream|SFE_UBLOX|HardwareSerial)\b/.test(nativeHeader))
    throw new Error("Native UART public API exposes a compatibility type");
  console.log("PASS: application Arduino/SparkFun API and dependency ownership scan");
  for (const ublox of [0, 1]) {
    const name = `gnss-receiver-ublox-${ublox}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
      "/D_CRT_SECURE_NO_WARNINGS", `/DUBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED=${ublox}`,
      `/I${stubs}`, `/I${receiverStubs}`, `/I${out}`, `/I${component}`,
      path.join(__dirname, "gnss_receiver_test.cpp"), path.join(component, "gnss_receiver.cpp"),
      `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
  for (const xtal of [0, 1]) {
    const name = `gnss-uart-xtal-${xtal}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4244", "/wd4267", "/std:c++17",
      "/D_CRT_SECURE_NO_WARNINGS", `/DSOC_UART_SUPPORT_XTAL_CLK=${xtal}`,
      `/I${stubs}`, `/I${receiverStubs}`, `/I${out}`, `/I${component}`,
      path.join(__dirname, "gnss_uart_test.cpp"),
      path.join(component, "gnss_uart.cpp"), path.join(component, "gnss_receiver.cpp"),
      `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
  const startupCases = [0, 1].flatMap(consoleUart =>
    [0, 1].map(rollback => ({ consoleUart, rollback, psram: 0, bootInit: 0 })));
  startupCases.push({ consoleUart: 0, rollback: 1, psram: 1, bootInit: 0 },
    { consoleUart: 0, rollback: 1, psram: 1, bootInit: 0, isrIram: 1 },
    { consoleUart: 1, rollback: 1, psram: 1, bootInit: 1 });
  for (const { consoleUart, rollback, psram, bootInit, isrIram = 0 } of startupCases) {
    const name = `native-startup-console-${consoleUart}-rollback-${rollback}-psram-${psram}-boot-${bootInit}-iram-${isrIram}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
      `/DCONFIG_ESP_CONSOLE_UART=${consoleUart}`, `/DCONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=${rollback}`,
      `/DCONFIG_SPIRAM=${psram}`, `/DCONFIG_SPIRAM_BOOT_INIT=${bootInit}`,
      `/DCONFIG_ARDUINO_ISR_IRAM=${isrIram}`,
      "/DCONFIG_SPIRAM_BOOT_HW_INIT=1", "/DCONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=1",
      "/DCONFIG_SPIRAM_USE_MALLOC=1", "/DCONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384",
      `/I${stubs}`, `/I${out}`, path.join(__dirname, "native_startup_test.cpp"), `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
  for (const { name, bootHw, protection } of [
    { name: "protection", bootHw: 1, protection: 0 },
    { name: "boot-hardware", bootHw: 0, protection: 1 }
  ]) {
    const invalidPsram = spawnSync("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
      "/DCONFIG_ESP_CONSOLE_UART=0", "/DCONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=0",
      "/DCONFIG_SPIRAM=1", "/DCONFIG_SPIRAM_BOOT_INIT=0",
      `/DCONFIG_SPIRAM_BOOT_HW_INIT=${bootHw}`,
      `/DCONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=${protection}`,
      "/DCONFIG_SPIRAM_USE_MALLOC=1", "/DCONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384",
      `/I${stubs}`, `/I${out}`, path.join(__dirname, "native_startup_test.cpp"), `/Feinvalid-psram-${name}.exe`],
      { cwd: out, encoding: "utf8" });
    if (invalidPsram.error) throw invalidPsram.error;
    if (invalidPsram.status === 0 ||
      !`${invalidPsram.stdout}\n${invalidPsram.stderr}`.includes("Delayed PSRAM initialization requires"))
      throw new Error(`Delayed PSRAM without ${name} was not explicitly rejected`);
    console.log(`PASS: delayed PSRAM without ${name} is rejected`);
  }
}
if (!process.argv.includes("--auth-only") && !process.argv.includes("--ptp-only"))
  runGnssTests();
if (process.argv.includes("--gnss-only")) process.exit(0);
write("psa\\crypto.h", `#include "${path.join(__dirname, "auth_crypto_fixture.h").replace(/\\/g, "\\\\")}"\n`);
write("esp_log.h", `#pragma once
#include <cstdarg>
#include <cstdio>
inline unsigned s_fixture_warning_logs = 0, s_fixture_error_logs = 0;
inline char s_fixture_last_log[256]{};
inline void fixture_log(bool warning, const char *format, ...) {
  ++(warning ? s_fixture_warning_logs : s_fixture_error_logs);
  va_list args;
  va_start(args, format);
  vsnprintf(s_fixture_last_log, sizeof(s_fixture_last_log), format, args);
  va_end(args);
}
inline void fixture_info(const char *, ...) {}
#define ESP_LOGE(tag, ...) ((void)(tag), fixture_log(false, __VA_ARGS__))
#define ESP_LOGW(tag, ...) ((void)(tag), fixture_log(true, __VA_ARGS__))
#define ESP_LOGI(tag, ...) ((void)(tag), fixture_info(__VA_ARGS__))
`);
write("auth_settings.h", `#pragma once
#include <array>
#include <cstdint>
#include "${path.join(root, "main", "custom", "setting_types.c").replace(/\\/g, "\\\\")}"
#define NTP_AUTHENTICATION_ENABLED 1
#ifndef PTP_L2_AUTHENTICATION_ENABLED
#define PTP_L2_AUTHENTICATION_ENABLED 1
#endif
#ifndef NTP_AUTHENTICATION_REQUIRED
#define NTP_AUTHENTICATION_REQUIRED 0
#endif
#ifndef PTP_L2_AUTHENTICATION_REQUIRED
#define PTP_L2_AUTHENTICATION_REQUIRED 0
#endif
#ifndef TEST_INVALID_PTP_KEYS
#define TEST_INVALID_PTP_KEYS 0
#endif
#ifndef TEST_EMPTY_PTP_KEYS
#define TEST_EMPTY_PTP_KEYS 0
#endif
#ifndef TEST_PTP_INIT_FAILURE
#define TEST_PTP_INIT_FAILURE 0
#endif
#ifndef TEST_PTP_IMPORT_FAILURE
#define TEST_PTP_IMPORT_FAILURE 0
#endif
#ifndef TEST_PTP_ARRAY_CONTAINER
#define TEST_PTP_ARRAY_CONTAINER 0
#endif
#define PTP_L2_AUTHENTICATION_SPP 1
#define PTP_L2_AUTHENTICATION_ACTIVE_KEY_ID 1
static constexpr symmetric_key_t symmetric_keys[] = {{1, "abcdefghijklmnopqrstuvwxyz123456"}};
#if TEST_EMPTY_PTP_KEYS
static constexpr std::array<symmetric_key_t, 0> ptp_l2_symmetric_keys{};
#elif TEST_PTP_ARRAY_CONTAINER
static constexpr std::array<symmetric_key_t, 2> ptp_l2_symmetric_keys{{
{1, "abcdefghijklmnopqrstuvwxyz123456"}, {2, "123456abcdefghijklmnopqrstuvwxyz"},
}};
#else
static constexpr symmetric_key_t ptp_l2_symmetric_keys[] = {
#if TEST_INVALID_PTP_KEYS == 1
{1, ""},
#elif TEST_INVALID_PTP_KEYS == 2
{1, "abcdefghijklmnopqrstuvwxyz123456"}, {1, "123456abcdefghijklmnopqrstuvwxyz"},
#elif TEST_INVALID_PTP_KEYS == 3
{2, "abcdefghijklmnopqrstuvwxyz123456"},
#elif TEST_INVALID_PTP_KEYS == 4
{0, "abcdefghijklmnopqrstuvwxyz123456"},
#elif TEST_INVALID_PTP_KEYS == 5
{1, "abcdefghijklmnopqrstuvwxyz12345!"},
#elif TEST_INVALID_PTP_KEYS == 6
{1, "abcdefghijklmnopqrstuvwxyz123456"}, {2, "123456abcdefghijklmnopqrstuvwxyz"},
{3, "abcdefghijklmnopqrstuvwxyz123456"}, {4, "abcdefghijklmnopqrstuvwxyz123456"},
{5, "abcdefghijklmnopqrstuvwxyz123456"},
#elif TEST_INVALID_PTP_KEYS == 7
{1, "abcdefghijklmnopqrstuvwxyz12345"},
#elif TEST_INVALID_PTP_KEYS == 8
{1, {'a','a','a','a','a','a','a','a','a','a','a','a','a','a','a','a',
     'a','a','a','a','a','a','a','a','a','a','a','a','a','a','a','a','a'}},
#elif TEST_INVALID_PTP_KEYS == 9
{1, "abcdefghijklmnopqrstuvwxyz12345\\x80"},
#else
{1, "abcdefghijklmnopqrstuvwxyz123456"}, {2, "123456abcdefghijklmnopqrstuvwxyz"},
#endif
};
#endif
`);
for (const protocol of ["ntp", "ptp_l2"])
  write(`${protocol}_auth_under_test.cpp`, fs.readFileSync(path.join(root, "main", "custom", `${protocol}_auth.cpp`), "utf8")
    .replace('"ESP32TimeServerSettings.h"', '"auth_settings.h"'));
const vector = Buffer.alloc(70);
vector[0] = 1; vector[1] = 0x12; vector[3] = 70;
vector[20] = 1; vector[29] = 1; vector[31] = 7; vector[32] = 1;
vector.writeUInt16BE(0x8009, 44); vector[47] = 22; vector[48] = 1; vector[53] = 1;
const icv = crypto.createHmac("sha256", "abcdefghijklmnopqrstuvwxyz123456").update(vector.subarray(0, 54)).digest().subarray(0, 16);
const rotated = Buffer.from(vector);
rotated[53] = 2;
crypto.createHmac("sha256", "123456abcdefghijklmnopqrstuvwxyz").update(rotated.subarray(0, 54)).digest().copy(rotated, 54, 0, 16);
const ntpVector = Buffer.alloc(48); ntpVector[0] = 0x23;
const ntpDigest = crypto.createHash("sha256").update("abcdefghijklmnopqrstuvwxyz123456").update(ntpVector).digest().subarray(0, 20);
write("auth_vectors.h", [
  ["EXPECTED_PTP_ICV", icv], ["ROTATED_PTP_PACKET", rotated], ["EXPECTED_NTP_DIGEST", ntpDigest],
].map(([name, bytes]) => `static constexpr uint8_t ${name}[] = {${[...bytes].join(",")}};`).join("\n"));
const authCases = [
  ...[0, 1].flatMap(ntpRequired => [0, 1].map(ptpRequired => ({ ntpRequired, ptpRequired }))),
  ...[0, 1].flatMap(ptpRequired => [
    ...[1, 2, 3, 4, 5, 6, 7, 8, 9].map(invalid => ({ ptpRequired, invalid })),
    { ptpRequired, empty: 1 }, { ptpRequired, initFailure: 1 },
    { ptpRequired, importFailure: 1 }, { ptpRequired, importFailure: 2 },
    { ptpRequired, array: 1 },
  ]),
];
for (const [index, { ntpRequired = 0, ptpRequired = 0, invalid = 0, empty = 0,
  initFailure = 0, importFailure = 0, array = 0 }] of authCases.entries()) {
  const name = `auth-${index}`;
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/std:c++17",
    `/DNTP_AUTHENTICATION_REQUIRED=${ntpRequired}`, `/DPTP_L2_AUTHENTICATION_REQUIRED=${ptpRequired}`,
    `/DTEST_INVALID_PTP_KEYS=${invalid}`, `/DTEST_EMPTY_PTP_KEYS=${empty}`,
    `/DTEST_PTP_INIT_FAILURE=${initFailure}`, `/DTEST_PTP_IMPORT_FAILURE=${importFailure}`,
    `/DTEST_PTP_ARRAY_CONTAINER=${array}`, `/I${out}`, `/I${__dirname}`, `/I${path.join(root, "main", "custom")}`,
    path.join(__dirname, "auth_test.cpp"), path.join(out, "ntp_auth_under_test.cpp"),
    path.join(out, "ptp_l2_auth_under_test.cpp"), `/Fe${name}.exe`]);
  run(path.join(out, `${name}.exe`), []);
}
if (process.argv.includes("--auth-only")) process.exit(0);
run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
  path.join(__dirname, "ptp_host_tests.cpp"), "/Feptp-host-tests.exe"]);
run(path.join(out, "ptp-host-tests.exe"), []);
run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
  path.join(__dirname, "ptp_master_selection_test.cpp"), "/Feptp-master-selection-tests.exe"]);
run(path.join(out, "ptp-master-selection-tests.exe"), []);
for (const [enabled, required, selection, empty = 0, invalid = 0] of [
  [0, 0, 0], [0, 0, 1], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 1],
  [1, 0, 0, 1], [1, 0, 1, 1], [1, 1, 0, 1], [1, 1, 1, 1],
  [1, 0, 1, 0, 1], [1, 1, 1, 0, 1],
]) {
  const name = `ptp-service-auth-${enabled}-required-${required}-selection-${selection}-empty-${empty}-invalid-${invalid}`;
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/wd4456", "/std:c++17", `/I${out}`,
    `/I${path.join(root, "main", "custom")}`, `/DPTP_L2_AUTHENTICATION_ENABLED=${enabled}`,
    `/DPTP_L2_AUTHENTICATION_REQUIRED=${required}`,
    `/DTEST_EMPTY_PTP_KEYS=${empty}`, `/DTEST_INVALID_PTP_KEYS=${invalid}`,
    `/DPTP_L2_MASTER_SELECTION_ENABLED=${selection}`,
    path.join(__dirname, "ptp_service_test.cpp"),
    ...(enabled ? [path.join(out, "ptp_l2_auth_under_test.cpp")] : []), `/Fe${name}.exe`]);
  run(path.join(out, `${name}.exe`), []);
}
run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4127", "/std:c++17", `/I${out}`,
  path.join(__dirname, "ptp_l3_udp_test.cpp"), "/Feptp-l3-udp-tests.exe"]);
run(path.join(out, "ptp-l3-udp-tests.exe"), []);
if (process.argv.includes("--ptp-only")) process.exit(0);
run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
  path.join(__dirname, "leap_seconds_test.cpp"), "/Feleap-seconds-tests.exe"]);
run(path.join(out, "leap-seconds-tests.exe"), []);
for (const [target, minimum] of [["p4", 300], ["p4", 0], ["s31", 0]]) {
  const name = `pps-capture-${target}-${minimum}`;
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17", `/I${out}`,
    `/DCONFIG_IDF_TARGET_ESP32P4=${target === "p4" ? 1 : 0}`,
    `/DCONFIG_IDF_TARGET_ESP32S31=${target === "s31" ? 1 : 0}`,
    `/DCONFIG_ESP32P4_REV_MIN_FULL=${minimum}`,
    path.join(__dirname, "pps_capture_test.cpp"), `/Fe${name}.exe`]);
  run(path.join(out, `${name}.exe`), []);
}
const mqttLimitSetting = name => {
  const match = settingsTemplate.match(new RegExp(`static constexpr size_t ${name} = (\\d+);`));
  if (!match) throw new Error(`${name} not found in the settings template`);
  return Number(match[1]);
};
const mqttNtpClientLimit = mqttLimitSetting("MQTT_NTP_CLIENT_REPORT_LIMIT");
run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17", `/I${path.join(root, "main", "custom")}`,
  path.join(__dirname, "mqtt_queue_test.cpp"), "/Femqtt-queue-tests.exe"]);
run(path.join(out, "mqtt-queue-tests.exe"), []);
for (const [ntp, l2, l3] of [[1, 0, 0], [0, 1, 0], [0, 0, 1],
[1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1]]) {
  const ptp = l2 || l3;
  const mqttPtpClientLimit = (l2 ? mqttL2FollowerCapacity : 0) +
    (l3 ? mqttL3FollowerCapacity * ((mqttPreferIPvX !== 2) + (mqttPreferIPvX !== 1)) : 0);
  for (const details of [0, 1]) {
    const name = `mqtt-${ntp}-${l2}-${l3}-${details}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4505", "/wd4127", "/std:c++17",
      `/DNTP_SERVER_ENABLED=${ntp}`, `/DPTP_L2_SERVER_ENABLED=${l2}`,
      `/DPTP_L3_SERVER_ENABLED=${l3}`, `/DPTP_ENABLED=${ptp}`, "/DDEBUG_ENABLED=0",
      "/DNTP_AUTHENTICATION_ENABLED=1", "/DMQTT_DISTINGUISH_AUTHENTICATED_REPORTING_ENABLED=1",
      `/DMQTT_CLIENT_REPORTING_ENABLED=${details}`, `/DMQTT_MEMORY_REPORTING_ENABLED=${details}`,
      `/DMQTT_HISTORICAL_REPORTING_ENABLED=${details}`, `/I${out}`, `/I${path.join(root, "main", "custom")}`,
      path.join(__dirname, "mqtt_report_test.cpp"), `/Fe${name}.exe`]);
    const result = spawnSync(path.join(out, `${name}.exe`), [], { encoding: "utf8" });
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(result.stderr || `Report test failed: ${name}`);
    const reports = result.stdout.trim().split(/\r?\n/).map(line => JSON.parse(line));
    if (reports.length !== 6) throw new Error("Missing report snapshots");
    for (const report of reports) {
      if (("ntp" in report.this_period) !== Boolean(ntp) ||
        ("ptp" in report.this_period) !== Boolean(ptp) ||
        ("clients" in report.this_period) || ("clients_overflown" in report.this_period) ||
        (ntp && (("clients" in report.this_period.ntp) !== Boolean(details) ||
          ("clients_overflown" in report.this_period.ntp) !== Boolean(details))))
        throw new Error(`Wrong protocol report shape: ${name}`);
    }
    if (ntp && details &&
      (reports[4].this_period.ntp.clients.length !== 10 ||
        reports[5].this_period.ntp.clients.length !== mqttNtpClientLimit ||
        reports[5].this_period.ntp.clients.some(client => client.address.length !== 45)))
      throw new Error(`Missing maximum-size IPv6 client reports: ${name}`);
    if (ptp) {
      const followers = reports.map(report => report.this_period.ptp);
      if (followers[0].followers.current !== mqttPtpClientLimit ||
        followers[0].followers.max !== mqttPtpClientLimit ||
        followers[0].follower_overflow_drops !== 1 ||
        followers[1].followers.min !== mqttPtpClientLimit || followers[1].follower_overflow_drops !== 0 ||
        followers[2].followers.current !== 0 || followers[3].followers.current !== 0 ||
        followers[0].clients.length !== mqttPtpClientLimit ||
        followers[0].clients.some(client => client.active_entire_period !== false) ||
        followers[1].clients.some(client => client.active_entire_period !== true) ||
        followers[2].clients.some(client => client.active_entire_period !== false) ||
        followers[3].clients.some(client => client.active_entire_period !== false) ||
        followers[4].clients.length !== mqttPtpClientLimit ||
        followers[5].clients.length !== mqttPtpClientLimit ||
        followers[0].clients[0].mac !== "00:00:00:00:00:00")
        throw new Error(`Wrong follower lifecycle snapshots: ${name}`);
    }
    console.log(`PASS: ${name} JSON shape, frozen snapshots, expiry, withdrawal and buffer bounds`);
  }
}
run("cl.exe", ["/nologo", "/W4", "/WX", "/std:c11", "/TC", "/c",
  `/I${out}`, path.join(root, "main", "custom", "ntp_cache.c"), "/Focache.obj"]);
runNetworkModeTests();
for (const debug of [0, 1]) {
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`, `/I${path.join(root, "main", "custom")}`,
    path.join(__dirname, "test.cpp"), "cache.obj", `/Fetests-debug-${debug}.exe`]);
  run(path.join(out, `tests-debug-${debug}.exe`), []);
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`,
    path.join(__dirname, "startup_test.cpp"), `/Festartup-debug-${debug}.exe`]);
  run(path.join(out, `startup-debug-${debug}.exe`), []);
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`, `/I${path.join(root, "main", "custom")}`,
    path.join(__dirname, "sync_test.cpp"), `/Fesync-debug-${debug}.exe`]);
  run(path.join(out, `sync-debug-${debug}.exe`), []);
  for (const freeRunning of [0, 1]) {
    const name = `timestamps-debug-${debug}-free-${freeRunning}`;
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
      `/DDEBUG_ENABLED=${debug}`, `/DPTP_FREE_RUNNING=${freeRunning}`, `/I${out}`,
      path.join(__dirname, "timestamp_test.cpp"), `/Fe${name}.exe`]);
    run(path.join(out, `${name}.exe`), []);
  }
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`,
    path.join(__dirname, "transport_test.cpp"), `/Fetransport-debug-${debug}.exe`]);
  run(path.join(out, `transport-debug-${debug}.exe`), []);
  for (const test of ["ingress", "load", "recovery"]) {
    run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/std:c++17",
      `/DDEBUG_ENABLED=${debug}`, `/I${out}`,
      path.join(__dirname, `${test}_test.cpp`), `/Fe${test}-debug-${debug}.exe`]);
    run(path.join(out, `${test}-debug-${debug}.exe`), []);
  }
  runNetworkTests(debug);
  run("cl.exe", ["/nologo", "/EHsc", "/W4", "/WX", "/wd4100", "/std:c++17",
    `/DDEBUG_ENABLED=${debug}`, `/I${out}`,
    path.join(__dirname, "restart_test.cpp"), `/Ferestart-debug-${debug}.exe`]);
  run(path.join(out, `restart-debug-${debug}.exe`), []);
}
