#include <stdio.h>
#include <string.h>
#include <stdlib.h>         // strtoul
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dc_protocol_rc.h" 
#include "drv_espnow_rx.h"
#include "app_dashboard.h"
#include "app_route_mapper.h"   // Route Mapping 模組
#include "drv_sd_card.h"        // SD 卡驅動

static const char *TAG = "LOGGER_MAIN_RC";

/**
 * @brief 發送遠端控制指令 (具備 5 次連續噴發冗餘機制)
 */
void send_remote_control_command(control_cmd_t command) {
    espnow_payload_t cmd_msg = {0};
    cmd_msg.msg_type = MSG_TYPE_CONTROL;
    cmd_msg.node_id = 0;
    cmd_msg.data.ctrl.command = command;
    cmd_msg.data.ctrl.timestamp = (uint32_t)(esp_timer_get_time() / 1000);

    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if (!esp_now_is_peer_exist(broadcast_mac)) {
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, broadcast_mac, 6);
        peer_info.channel = ESPNOW_WIFI_CHANNEL;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);
    }

    const int burst_count = 5;
    const int interval_ms = 10;
    for (int i = 0; i < burst_count; i++) {
        esp_now_send(broadcast_mac, (uint8_t *)&cmd_msg, sizeof(cmd_msg));
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

/**
 * @brief 序列埠終端機監聽任務
 *
 *  指令說明：
 *    [s]      → 強制存檔 (FORCE SAVE)
 *    [r]      → 重置所有節點 (RESET NODE)
 *    [l<N>]   → 開始錄製 Route N 的 Mapping（例如 l2 = 開始 Route 2）
 *    [l]      → 停止錄製並儲存 Mapping 至 /sdcard/route_mapping.csv
 *
 *  [l] 的解析規則：
 *    - 輸入 "l" 後緊跟數字（例如 "l2"、"l12"），視為「開始錄製 Route N」
 *    - 單獨輸入 "l"（後面沒有數字，或下一個字元是換行/空白），視為「停止錄製」
 */
void console_input_task(void *pvParameters) {
    printf("\n\033[1;36m--------------------------------------------------\033[0m\n");
    printf("\033[1;36m  Remote Control Dashboard Ready\033[0m\n");
    printf("\033[1;36m  [s]    - Force Save & Close All Sessions\033[0m\n");
    printf("\033[1;36m  [r]    - Reset All Distributed Nodes\033[0m\n");
    printf("\033[1;36m  [l<N>] - Start recording Route N mapping\033[0m\n");
    printf("\033[1;36m  [l]    - Stop recording & save mapping\033[0m\n");
    printf("\033[1;36m--------------------------------------------------\033[0m\n\n");

    char c;
    while (1) {
        if (scanf("%c", &c) != EOF) {

            // ── [s] 強制存檔 ──────────────────────────────────
            if (c == 's') {
                send_remote_control_command(CTRL_CMD_FORCE_SAVE);
                espnow_payload_t self_msg = {0};
                self_msg.msg_type = MSG_TYPE_CONTROL;
                self_msg.data.ctrl.command = CTRL_CMD_FORCE_SAVE;
                app_dashboard_print_event(&self_msg);

            // ── [r] 節點重置 ──────────────────────────────────
            } else if (c == 'r') {
                send_remote_control_command(CTRL_CMD_RESET_NODE);
                espnow_payload_t self_msg = {0};
                self_msg.msg_type = MSG_TYPE_CONTROL;
                self_msg.data.ctrl.command = CTRL_CMD_RESET_NODE;
                app_dashboard_print_event(&self_msg);

            // ── [l] / [l<N>] Route Mapping ───────────────────
            } else if (c == 'l') {
                /*
                 * PuTTY 使用 raw 模式：每個按鍵立即送出，沒有 Enter 緩衝。
                 *
                 * 策略：按下 'l' 後，開啟一個 500ms 的時間窗，
                 * 在窗口內若收到數字字元，累積為 route_id → 開始錄製；
                 * 500ms 內沒有任何數字 → 判定為單獨的 "l" → 停止錄製。
                 *
                 * 時間窗用 esp_timer_get_time()（微秒）實作，
                 * 內部以 10ms 為間隔輪詢，不阻塞其他任務。
                 */
                char     num_buf[8]  = {0};
                int      num_len     = 0;
                int64_t  deadline_us = esp_timer_get_time() + 500 * 1000; // 500ms

                while (esp_timer_get_time() < deadline_us &&
                       num_len < (int)(sizeof(num_buf) - 1)) {
                    int next_ch = getchar();
                    if (next_ch != EOF && next_ch >= '0' && next_ch <= '9') {
                        num_buf[num_len++] = (char)next_ch;
                        // 收到第一個數字後，再給 200ms 等待後續位數（如 "12"）
                        deadline_us = esp_timer_get_time() + 200 * 1000;
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(10)); // 讓出 CPU，避免忙等
                    }
                }
                num_buf[num_len] = '\0';

                if (num_len > 0) {
                    // ── 開始錄製 Route N ──
                    uint8_t route_id = (uint8_t)strtoul(num_buf, NULL, 10);
                    if (route_id == 0) {
                        printf("\033[1;31m[MAPPER]\033[0m Route ID must be >= 1. Ignored.\n");
                    } else {
                        app_route_mapper_start(route_id);
                    }
                } else {
                    // ── 停止錄製 ──
                    if (app_route_mapper_is_recording()) {
                        app_route_mapper_stop();
                    } else {
                        printf("\033[1;33m[MAPPER]\033[0m No active recording. "
                               "Use [l<N>] to start (e.g. l2 for Route 2).\n");
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief ESP-NOW 發送狀態回調 (v5.5 新版簽名)
 */
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    // 廣播模式下不印 Log，避免干擾畫面
}

void app_main(void)
{
    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化 SD 卡（必須在任何寫檔操作之前完成）
    esp_err_t sd_ret = sd_card_init("/sdcard");
    if (sd_ret != ESP_OK) {
        // SD 卡失敗不中止開機，但 Route Mapping 寫檔功能將無法使用
        ESP_LOGE(TAG, "SD card mount failed! Route mapping will not be saved.");
    }

    // 3. 建立 RX Queue
    QueueHandle_t rx_queue = xQueueCreate(64, sizeof(espnow_payload_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return;
    }

    // 4. 初始化各模組
    app_dashboard_init();
    app_route_mapper_init();

    app_dashboard_start(rx_queue);

    // 5. 初始化 ESP-NOW 接收
    espnow_rx_init(rx_queue);

    // 6. 註冊發送回調
    esp_now_register_send_cb(espnow_send_cb);

    // 7. 啟動 Console 監聽任務
    xTaskCreate(console_input_task, "console_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Logger Master RC System Initialized. Waiting for data...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}