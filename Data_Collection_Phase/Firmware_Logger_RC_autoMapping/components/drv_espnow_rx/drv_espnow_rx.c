#include <string.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "dc_protocol_rc.h"
#include "drv_espnow_rx.h"

static const char *TAG = "DRV_ESPNOW_RX";

static QueueHandle_t s_rx_queue = NULL;

/**
 * @brief ESP-NOW 底層接收回呼函數 (執行於高優先權 Wi-Fi 任務)
 */
static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (s_rx_queue == NULL || data == NULL) return;

    if (data_len == sizeof(espnow_payload_t)) {
        espnow_payload_t payload;
        memcpy(&payload, data, sizeof(espnow_payload_t));
        
        // 將資料送入 Queue。此處不阻塞，確保 Wi-Fi 任務流暢
        if (xQueueSend(s_rx_queue, &payload, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX Queue full, dropped a packet from Node %d", payload.node_id);
        }
    } else {
        ESP_LOGW(TAG, "Received unknown packet size: %d bytes", data_len);
    }
}

void espnow_rx_init(QueueHandle_t rx_queue) {
    s_rx_queue = rx_queue;

    // 1. 基礎網路與事件迴圈初始化 (採用 WITHOUT_ABORT)
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

    // 2. 初始化 Wi-Fi 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 3. 設定 LR 模式、固定頻道與關閉省電模式
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    // 關鍵：關閉省電模式，避免 Logger 在監聽時進入淺睡眠導致掉包
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 4. 印出 Logger 自己的 MAC 位址 (供 Slave 參考)
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Logger MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Channel: %d | Mode: Long Range | PS: NONE", ESPNOW_WIFI_CHANNEL);
    ESP_LOGI(TAG, "========================================");

    // 5. 初始化 ESP-NOW 並註冊接收回呼函數
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW RX Initialized and listening...");
}