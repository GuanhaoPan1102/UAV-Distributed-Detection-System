#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// 引入 Logger 專屬組件
#include "dc_protocol.h"
#include "drv_espnow_rx.h"
#include "app_dashboard.h"

static const char *TAG = "LOGGER_MAIN";

void app_main(void) {
    // 1. 初始化 NVS (ESP-NOW 與 Wi-Fi 儲存配置必備)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "---------------------------------------------");
    ESP_LOGI(TAG, "  Master Logger System Starting...");
    ESP_LOGI(TAG, "---------------------------------------------");

    // 2. 建立資料管線 (Queue)
    // 深度設為 20，足以緩衝多個 Slave 同時噴過來的狀態封包
    QueueHandle_t rx_queue = xQueueCreate(20, sizeof(espnow_payload_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX Queue. System Halted.");
        return;
    }

    // 3. 初始化「耳朵」：ESP-NOW 接收驅動
    // 它會自動印出本機 MAC 地址，請記得更新到 Slave 的 drv_espnow_tx.c 中
    espnow_rx_init(rx_queue);

    // 4. 啟動「大腦」：儀表板 UI 引擎
    // 它會負責從 Queue 拿資料並渲染漂亮的終端機表格
    app_dashboard_start(rx_queue);

    ESP_LOGI(TAG, "Logger is ready. Waiting for Slave messages...");

    // 5. 主任務進入閒置狀態
    // 所有的工作都交給 drv_espnow_rx (中斷) 與 dashboard_task (任務) 處理
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}