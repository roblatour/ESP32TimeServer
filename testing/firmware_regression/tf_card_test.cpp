#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <io.h>

#define MQTT_ENABLED 1
#include "board_pins_fixture.h"

static constexpr int MQTT_QOS = TEST_MQTT_QOS;
static constexpr const char *TAG = "tf-test";
static unsigned logs = 0;
[[maybe_unused]] static void record_log(const char *, const char *, ...) { ++logs; }
#define ESP_LOGW(...) record_log(__VA_ARGS__)
#define ESP_LOGI(...) record_log(__VA_ARGS__)
#define TIMING_LOGW(...) do { if (DEBUG_ENABLED) record_log(__VA_ARGS__); } while (0)
#define TIMING_LOGI(...) do { if (DEBUG_ENABLED) record_log(__VA_ARGS__); } while (0)

static std::atomic<bool> s_tf_queue_available{false};
static std::atomic<int> s_mqtt_queue_fallback_status{0};
static unsigned allocations = 0;
static bool allocation_ok = true;
static bool mqtt_allocate_ram_queue() { ++allocations; return allocation_ok; }

#if TIME_SERVER_BOARD_HAS_TF_CARD && MQTT_USE_TF_CARD
using esp_err_t = int;
using gpio_num_t = int;
using DWORD = unsigned long;
static constexpr int ESP_OK = 0, ESP_FAIL = -1, FR_OK = 0, FS_FAT32 = 3;
static constexpr char TF_MOUNT_POINT[] = "/tfcard";
static constexpr char TF_QUEUE_DIRECTORY[] = "/tfcard/Queue";
enum class failure_t { none, initialize, mount, free_clusters, fat_type, directory, probe_open, probe_write, probe_read };
static failure_t failure = failure_t::none;
static unsigned initializations = 0, releases = 0, mounts = 0, unmounts = 0;
static bool host_active = false, card_mounted = false;
static int host_token = 0;
struct sdmmc_card_t {};
static sdmmc_card_t card;
static sdmmc_card_t *s_tf_card = nullptr;
static std::atomic<size_t> s_mqtt_queued_messages_count{0};
static uint64_t s_tf_queue_next_sequence = 0;
[[maybe_unused]] static const char *esp_err_to_name(esp_err_t) { return "injected"; }
struct esp_vfs_fat_mount_config_t
{
    bool format_if_mount_failed = true;
    int max_files = 0;
    size_t allocation_unit_size = 0;
};
struct sdmmc_host_t
{
    int slot = 2, max_freq_khz = 20000;
    void *pwr_ctrl_handle = nullptr;
};

static esp_err_t mount_card(const char *path, const sdmmc_host_t *host,
                            const esp_vfs_fat_mount_config_t *config, sdmmc_card_t **output)
{
    assert(host_active && host != nullptr && !card_mounted);
    assert(std::strcmp(path, TF_MOUNT_POINT) == 0);
    assert(!config->format_if_mount_failed && config->max_files == 4 &&
           config->allocation_unit_size == 16 * 1024);
    ++mounts;
    if (failure == failure_t::mount)
        return ESP_FAIL;
    card_mounted = true;
    *output = &card;
    return ESP_OK;
}

#if CONFIG_IDF_TARGET_ESP32P4
using sd_pwr_ctrl_handle_t = void *;
struct sd_pwr_ctrl_ldo_config_t { int ldo_chan_id = -1; };
#define SDMMC_HOST_DEFAULT() sdmmc_host_t{}
static esp_err_t sd_pwr_ctrl_new_on_chip_ldo(const sd_pwr_ctrl_ldo_config_t *config, sd_pwr_ctrl_handle_t *output)
{
    assert(config->ldo_chan_id == 4);
    ++initializations;
    if (failure == failure_t::initialize)
        return ESP_FAIL;
    host_active = true;
    *output = &host_token;
    return ESP_OK;
}
static void sd_pwr_ctrl_del_on_chip_ldo(sd_pwr_ctrl_handle_t handle)
{
    assert(handle == &host_token && host_active && !card_mounted);
    host_active = false;
    ++releases;
}
struct sdmmc_slot_config_t
{
    int width = 0, clk = -1, cmd = -1, d0 = -1, d1 = -1, d2 = -1, d3 = -1;
    unsigned flags = 0;
};
#define SDMMC_SLOT_CONFIG_DEFAULT() sdmmc_slot_config_t{}
static constexpr unsigned SDMMC_SLOT_FLAG_INTERNAL_PULLUP = 1;
static esp_err_t esp_vfs_fat_sdmmc_mount(const char *path, const sdmmc_host_t *host,
                                       const sdmmc_slot_config_t *slot,
                                       const esp_vfs_fat_mount_config_t *config, sdmmc_card_t **output)
{
    assert(host->pwr_ctrl_handle == &host_token);
    assert(slot->width == 4 && slot->clk == 43 && slot->cmd == 44 &&
           slot->d0 == 39 && slot->d1 == 40 && slot->d2 == 41 && slot->d3 == 42);
    assert(slot->flags == SDMMC_SLOT_FLAG_INTERNAL_PULLUP);
    return mount_card(path, host, config, output);
}
#else
using spi_host_device_t = int;
static constexpr int SDSPI_DEFAULT_DMA = -1;
#define SDSPI_HOST_DEFAULT() sdmmc_host_t{}
struct spi_bus_config_t
{
    int mosi_io_num = -1, miso_io_num = -1, sclk_io_num = -1;
    int quadwp_io_num = 0, quadhd_io_num = 0;
    int data4_io_num = 0, data5_io_num = 0, data6_io_num = 0, data7_io_num = 0;
    int max_transfer_sz = 0;
};
static esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *config, int dma)
{
    assert(host == 2 && dma == SDSPI_DEFAULT_DMA);
    assert(config->mosi_io_num == 1 && config->miso_io_num == 3 && config->sclk_io_num == 2);
    assert(config->quadwp_io_num == -1 && config->quadhd_io_num == -1 &&
           config->data4_io_num == -1 && config->data5_io_num == -1 &&
           config->data6_io_num == -1 && config->data7_io_num == -1);
    assert(config->max_transfer_sz == 4000);
    ++initializations;
    if (failure == failure_t::initialize)
        return ESP_FAIL;
    host_active = true;
    return ESP_OK;
}
static esp_err_t spi_bus_free(spi_host_device_t host)
{
    assert(host == 2 && host_active && !card_mounted);
    host_active = false;
    ++releases;
    return ESP_OK;
}
struct sdspi_device_config_t { spi_host_device_t host_id = -1; gpio_num_t gpio_cs = -1; };
#define SDSPI_DEVICE_CONFIG_DEFAULT() sdspi_device_config_t{}
static esp_err_t esp_vfs_fat_sdspi_mount(const char *path, const sdmmc_host_t *host,
                                       const sdspi_device_config_t *slot,
                                       const esp_vfs_fat_mount_config_t *config, sdmmc_card_t **output)
{
    assert(host->slot == 2 && host->max_freq_khz == 4000 && host->pwr_ctrl_handle == nullptr);
    assert(slot->host_id == 2 && slot->gpio_cs == 0);
    return mount_card(path, host, config, output);
}
#endif

static void esp_vfs_fat_sdcard_unmount(const char *path, sdmmc_card_t *mounted)
{
    assert(std::strcmp(path, TF_MOUNT_POINT) == 0 && mounted == &card);
    assert(host_active && card_mounted);
    card_mounted = false;
    ++unmounts;
}
struct FATFS { int fs_type = FS_FAT32; };
static FATFS test_filesystem;
static int f_getfree(const char *drive, DWORD *free_clusters, FATFS **output)
{
    assert(std::strcmp(drive, "0:") == 0);
    *free_clusters = 10;
    *output = &test_filesystem;
    test_filesystem.fs_type = failure == failure_t::fat_type ? 2 : FS_FAT32;
    return failure == failure_t::free_clusters ? ESP_FAIL : FR_OK;
}
static void mqtt_log_tf_directory(const char *path) { assert(std::strcmp(path, TF_MOUNT_POINT) == 0); }
static int test_mkdir(const char *path, int)
{
    assert(std::strcmp(path, TF_QUEUE_DIRECTORY) == 0);
    errno = failure == failure_t::directory ? EACCES : EEXIST;
    return -1;
}
static FILE *test_fopen(const char *path, const char *mode)
{
    assert(std::strcmp(path, "/tfcard/Queue/PROBE.TMP") == 0 && std::strcmp(mode, "wb+") == 0);
    if (failure == failure_t::probe_open)
        return nullptr;
    FILE *file = std::tmpfile();
    assert(file != nullptr);
    return file;
}
static size_t test_fwrite(const void *data, size_t size, size_t count, FILE *file)
{
    return failure == failure_t::probe_write ? 0 : std::fwrite(data, size, count, file);
}
static size_t test_fread(void *data, size_t size, size_t count, FILE *file)
{
    return failure == failure_t::probe_read ? 0 : std::fread(data, size, count, file);
}
static int test_unlink(const char *path) { assert(std::strcmp(path, "/tfcard/Queue/PROBE.TMP") == 0); return 0; }
struct mqtt_tf_queue_snapshot_t;
static bool mqtt_tf_queue_files(mqtt_tf_queue_snapshot_t &);
#define fileno _fileno
#define fsync _commit
#define mkdir test_mkdir
#define fopen test_fopen
#define fwrite test_fwrite
#define fread test_fread
#define unlink test_unlink
#endif

#include "tf_card_under_test.h"

#if TIME_SERVER_BOARD_HAS_TF_CARD && MQTT_USE_TF_CARD
static bool mqtt_tf_queue_files(mqtt_tf_queue_snapshot_t &files)
{
    files.count = 2;
    files.next_sequence = 0x1003;
    return true;
}
#endif

int main()
{
#if TIME_SERVER_BOARD_HAS_TF_CARD && MQTT_USE_TF_CARD
    if (MQTT_QOS == 0)
    {
        setup_mqtt_tf_queue();
        assert(initializations == 0 && mounts == 0 && !s_tf_queue_available.load());
        assert(allocations == 0);
    }
    else
    {
        for (const failure_t injected : {failure_t::initialize, failure_t::mount, failure_t::free_clusters,
                                        failure_t::fat_type, failure_t::directory, failure_t::probe_open,
                                        failure_t::probe_write, failure_t::probe_read, failure_t::none})
        {
            failure = injected;
            initializations = releases = mounts = unmounts = logs = 0;
            s_tf_card = nullptr;
            s_tf_queue_available.store(false);
            s_mqtt_queued_messages_count.store(0);
            s_tf_queue_next_sequence = 0;
            allocations = 0;
            setup_mqtt_tf_queue();
            assert(initializations == 1);
            if (injected == failure_t::none)
            {
                assert(host_active && card_mounted && mounts == 1 && releases == 0 && unmounts == 0);
                assert(s_tf_card == &card && s_tf_queue_available.load());
                assert(s_mqtt_queued_messages_count.load() == 2 && s_tf_queue_next_sequence == 0x1003);
                assert(allocations == 0);
            }
            else
            {
                assert(!host_active && !card_mounted && s_tf_card == nullptr);
                assert(!s_tf_queue_available.load() && s_mqtt_queued_messages_count.load() == 0);
                assert(releases == (injected == failure_t::initialize ? 0U : 1U));
                assert(unmounts == (injected == failure_t::initialize || injected == failure_t::mount ? 0U : 1U));
                assert(logs == (DEBUG_ENABLED ? 1U : 0U));
                assert(allocations == 1 && s_mqtt_queue_fallback_status.load() == 0);
                allocation_ok = false;
                s_mqtt_queue_fallback_status.store(0);
                setup_mqtt_tf_queue();
                assert(s_mqtt_queue_fallback_status.load() == 0);
                allocation_ok = true;
            }
        }
    }
#else
    setup_mqtt_tf_queue();
    assert(logs == (DEBUG_ENABLED && MQTT_QOS > 0 ? 1U : 0U));
    assert(allocations == (MQTT_QOS > 0 ? 1U : 0U));
    assert(s_mqtt_queue_fallback_status.load() == 0);
#endif
    std::puts("PASS: TF transport, QoS gating, FAT32/probe checks, cleanup, RAM fallback and backlog restoration");
}
