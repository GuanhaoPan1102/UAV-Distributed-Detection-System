#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/uart.h"

// 引入我們開發的所有組件 (換成 RC 版專用協定)
#include "dc_protocol_rc.h" 
#include "drv_gps_atgm336h.h"
#include "drv_uart_rx.h"
#include "drv_sd_card.h"
#include "drv_espnow_tx.h"
#include "app_file_manager.h"

static const char *TAG = "SLAVE_MAIN_RC";

// 根據您的實驗需求，定義此 Slave 的編號
#define SLAVE_NODE_ID 1

// --- [看門狗機制]：UART 健康度追蹤 ---
static int64_t last_uart_rx_time_us = 0;
static portMUX_TYPE watchdog_mux = portMUX_INITIALIZER_UNLOCKED;

// 提供給 app_file_manager 呼叫：只要收到合法封包(含 0xEE) 就來餵狗
void uart_watchdog_feed(void) {
    portENTER_CRITICAL(&watchdog_mux);
    last_uart_rx_time_us = esp_timer_get_time();
    portEXIT_CRITICAL(&watchdog_mux);
}

// 檢查 UART 是否存活 (距離上次收到訊號是否小於 5 秒)
static uint8_t check_uart_health(void) {
    int64_t now = esp_timer_get_time();
    int64_t last;
    
    portENTER_CRITICAL(&watchdog_mux);
    last = last_uart_rx_time_us;
    portEXIT_CRITICAL(&watchdog_mux);

    // 如果系統剛開機還沒收到資料，或是超過 5 秒沒聲音，判定為斷線
    if (last > 0 && (now - last) < 5000000ULL) {
        return 1; // 正常
    }
    return 0; // 異常/斷線
}

// --- [任務]：系統監控與 Logger 回報 ---
static void sys_monitor_task(void *pvParameters) {
    bool is_registered = false;

    while (1) {
        // 每 60 秒執行一次心跳檢查
        vTaskDelay(pdMS_TO_TICKS(60000)); 

        gps_atgm_status_t gps = gps_atgm_get_status();
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint8_t uart_ok = check_uart_health();

        // 1. 註冊邏輯：如果 GPS 已經 Fixed 且尚未註冊，發送註冊包
        if (gps.is_fixed && !is_registered) {
            espnow_payload_t reg_msg = {0};
            reg_msg.msg_type = MSG_TYPE_REGISTER;
            reg_msg.node_id = SLAVE_NODE_ID;
            
            // 完整保留原版欄位，確保 Logger 端解析不會報錯
            reg_msg.data.reg.lat = gps.latitude;
            reg_msg.data.reg.lon = gps.longitude;
            reg_msg.data.reg.alt = (int32_t)gps.altitude; 
            
            reg_msg.data.reg.timestamp = (int64_t)tv.tv_sec; // 秒級 UTC

            espnow_report(&reg_msg);
            is_registered = true;
            ESP_LOGI(TAG, "Node Registration Sent to Logger.");
        }

        // 2. 心跳邏輯：無論是否註冊，定期回報硬體健康度
        espnow_payload_t hb_msg = {0};
        hb_msg.msg_type = MSG_TYPE_HEARTBEAT;
        hb_msg.node_id = SLAVE_NODE_ID;
        hb_msg.data.heartbeat.gps_fix_status = gps.is_fixed ? 1 : 0;
        hb_msg.data.heartbeat.uart_status = uart_ok;

        espnow_report(&hb_msg);
        
        // 本地螢幕除錯輸出
        ESP_LOGI(TAG, "--- System Heartbeat ---");
        ESP_LOGI(TAG, "GPS: %s | UART: %s | SD Free: %zu MB", 
                 gps.is_fixed ? "FIXED" : "SEARCHING",
                 uart_ok ? "ONLINE" : "OFFLINE",
                 sd_card_get_free_size_mb());
    }
}

void app_main(void) {
    // 1. 初始化 NVS (Wi-Fi/ESP-NOW 底層需要)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing SlaveTTGO RC Version (Node %d)...", SLAVE_NODE_ID);

    // 2. 初始化 SD 卡 (使用 TTGO T8 固定腳位)
    if (sd_card_init("/sdcard") != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: SD Card Mount Failed. System Halted.");
        return; 
    }

    // 3. 初始化 ESP-NOW 發送模組 (Unicast 給 Logger，並開啟廣播監聽能力)
    espnow_tx_init();

    // 4. 初始化 GPS (建立微秒級絕對時間基準)
    // 腳位請依照您的擴充板連接調整，例如：UART_NUM_1, TX:23, RX:18, PPS:19
    gps_atgm_init(UART_NUM_1, 23, 18, 19);

    // 5. 建立通訊隊列 (UART RX 與 File Manager 的橋樑，深度 50)
    QueueHandle_t data_sync_queue = xQueueCreate(50, sizeof(uart_scanner_to_ttgo_t));
    if (data_sync_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create data queue.");
        return;
    }

    // 6. 啟動 UART 接收驅動 (對接 Scanner，具備硬體流控)
    // 假設 Scanner 接在 UART2: TX:27, RX:26, RTS:33, CTS:25
    uart_rx_init(data_sync_queue, UART_NUM_2, 27, 26, 33, 25);

    // 7. 啟動檔案管理大腦 (現在它具備了「5秒冷卻」的強力抵抗機制)
    app_file_manager_start(data_sync_queue);

    // 8. 啟動系統監控與心跳任務 (優先權最低)
    xTaskCreate(sys_monitor_task, "sys_monitor", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "SlaveTTGO RC fully started. Waiting for Logger commands and Scanner data...");
}