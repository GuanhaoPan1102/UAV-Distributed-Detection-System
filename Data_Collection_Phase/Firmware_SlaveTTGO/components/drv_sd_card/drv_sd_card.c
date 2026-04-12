#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "drv_sd_card.h"

static const char *TAG = "DRV_SD_CARD";

// --- TTGO T8 V1.7.1 固定腳位定義 ---
#define SD_CS_PIN    13
#define SD_MOSI_PIN  15
#define SD_MISO_PIN  2
#define SD_SCLK_PIN  14

static sdmmc_card_t *g_card = NULL;
static bool g_is_mounted = false;
static char g_mount_point[32];

esp_err_t sd_card_init(const char* mount_point) {
    esp_err_t ret;
    strncpy(g_mount_point, mount_point, sizeof(g_mount_point));

    // 1. 掛載配置
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };

    // 2. 初始化 SPI 總線 (使用 VSPI 控制器)
    ESP_LOGI(TAG, "Initializing SPI bus with TTGO T8 default pins...");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    // [修正] ESP-IDF v5 使用 SPI_DMA_CH_AUTO 自動分配 DMA 通道
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus.");
        return ret;
    }

    // 3. 配置 SD 卡 Host 與 Slot 參數
    // [修正] ESP-IDF v5 要求獨立宣告 host 並傳入
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;

    // 4. 執行掛載 ([修正] 傳入 5 個參數)
    ESP_LOGI(TAG, "Mounting filesystem to %s", mount_point);
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &g_card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed (%s). Check if SD card is inserted.", esp_err_to_name(ret));
        return ret;
    }

    // 5. 輸出卡片詳細資訊
    sdmmc_card_print_info(stdout, g_card);
    g_is_mounted = true;
    
    ESP_LOGI(TAG, "SD Card successfully mounted on TTGO hardware.");
    return ESP_OK;
}

bool sd_card_is_mounted(void) {
    return g_is_mounted;
}

size_t sd_card_get_free_size_mb(void) {
    if (!g_is_mounted) return 0;

    FATFS *fs;
    DWORD free_clusters;
    if (f_getfree("0:", &free_clusters, &fs) != FR_OK) {
        return 0;
    }
    return (size_t)(free_clusters * fs->csize * 0.5 / 1024);
}