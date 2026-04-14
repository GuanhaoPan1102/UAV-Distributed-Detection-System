#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // [新增] 引入 Queue 標頭檔

// 引用新的 RC 專用協定
#include "dc_protocol_rc.h" 
#include "drv_espnow_rx.h"
#include "app_dashboard.h"

static const char *TAG = "LOGGER_MAIN_RC";

/**
 * @brief 發送遠端控制指令 (具備 5 次連續噴發冗餘機制)
 * @param command 控制指令代碼 (來自 control_cmd_t)
 */
void send_remote_control_command(control_cmd_t command) {
    espnow_payload_t cmd_msg = {0};
    cmd_msg.msg_type = MSG_TYPE_CONTROL;
    cmd_msg.node_id = 0; // Logger Master 固定為 0
    cmd_msg.data.ctrl.command = command;
    cmd_msg.data.ctrl.timestamp = (uint32_t)(esp_timer_get_time() / 1000);

    // 廣播 MAC 位址: FF:FF:FF:FF:FF:FF
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // 檢查 Peer 是否已存在，廣播需要手動加入 FF 節點到 Peer List
    if (!esp_now_is_peer_exist(broadcast_mac)) {
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, broadcast_mac, 6);
        peer_info.channel = ESPNOW_WIFI_CHANNEL; // 引用自 dc_protocol_rc.h
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);
    }

    // --- 連續噴發 5 次機制：對抗無線干擾，確保 Slave 至少能收到一次 ---
    const int burst_count = 5;
    const int interval_ms = 10;

    for (int i = 0; i < burst_count; i++) {
        esp_now_send(broadcast_mac, (uint8_t *)&cmd_msg, sizeof(cmd_msg));
        // 稍微延遲讓硬體緩衝區清空
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

/**
 * @brief 序列埠終端機監聽任務 (與 VS Code Serial Monitor 互動)
 */
void console_input_task(void *pvParameters) {
    printf("\n\033[1;36m--------------------------------------------------\033[0m\n");
    printf("\033[1;36m  Remote Control Dashboard Ready\033[0m\n");
    printf("\033[1;36m  [s] - Force Save & Close All Sessions\033[0m\n");
    printf("\033[1;36m  [r] - Reset All Distributed Nodes\033[0m\n");
    printf("\033[1;36m--------------------------------------------------\033[0m\n\n");
    
    char c;
    while (1) {
        // 使用 scanf 讀取終端輸入的單一字元
        if (scanf("%c", &c) != EOF) {
            if (c == 's') {
                send_remote_control_command(CTRL_CMD_FORCE_SAVE);
                // 為了讓你在 Logger 畫面看到自己發送的回饋，我們將訊息推給 dashboard 顯示
                espnow_payload_t self_msg = {0};
                self_msg.msg_type = MSG_TYPE_CONTROL;
                self_msg.data.ctrl.command = CTRL_CMD_FORCE_SAVE;
                app_dashboard_print_event(&self_msg); 
                
            } else if (c == 'r') {
                send_remote_control_command(CTRL_CMD_RESET_NODE);
                espnow_payload_t self_msg = {0};
                self_msg.msg_type = MSG_TYPE_CONTROL;
                self_msg.data.ctrl.command = CTRL_CMD_RESET_NODE;
                app_dashboard_print_event(&self_msg);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 保持流暢度，避免卡住 CPU
    }
}

/**
 * @brief ESP-NOW 發送狀態回調
 * 注意：這裡適配了 ESP-IDF v5.5 的新版簽名結構 (esp_now_send_info_t)
 */
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    // 廣播模式下 status 通常不代表對端是否真的收到，因此不印出 Log 以免干擾畫面
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

    // [修正核心] 建立 Queue：深度為 64 個封包，每個大小為 espnow_payload_t
    QueueHandle_t rx_queue = xQueueCreate(64, sizeof(espnow_payload_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return;
    }

    // 2. 初始化 Dashboard 介面，並把 Queue 交給它讓它能持續監聽
    app_dashboard_init();
    app_dashboard_start(rx_queue);

    // 3. 初始化 ESP-NOW 接收端，並把同一個 Queue 傳入供底層寫入
    espnow_rx_init(rx_queue); 

    // 4. [RC 版新增] 註冊發送狀態回調，開啟 Master 發射功能
    esp_now_register_send_cb(espnow_send_cb);

    // 5. [RC 版新增] 啟動 Console 監聽任務 (核心互動邏輯)
    xTaskCreate(console_input_task, "console_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Logger Master RC System Initialized. Waiting for data...");

    // 主迴圈
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}