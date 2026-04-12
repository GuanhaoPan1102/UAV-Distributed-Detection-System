#include <string.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "dc_protocol.h" 
#include "drv_espnow_tx.h"

static const char *TAG = "DRV_ESPNOW_TX";

// 請填寫 Logger 的 MAC 位址
static const uint8_t logger_mac_unicast[6] = {0x2C, 0xBC, 0xBB, 0xA7, 0xBF, 0xAC}; 

void espnow_tx_init(void) {
    // 1. 基礎網路初始化 (採用 WITHOUT_ABORT 避免多次呼叫導致崩潰)
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 2. [重要] 設定 LR 模式、固定頻道與關閉省電模式
    // 啟用長距離協議
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    
    // 設定物理頻道 (來自 dc_protocol.h)
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    // 關閉 Wi-Fi 省電模式 (關鍵：確保低延遲發送)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "TX Initialized: Channel %d, LR Mode, PS_NONE", ESPNOW_WIFI_CHANNEL);

    // 3. 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 4. 註冊 Peer (Logger)
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, logger_mac_unicast, 6);
    peer_info.channel = ESPNOW_WIFI_CHANNEL; 
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Logger peer");
    }
}

void espnow_report(espnow_payload_t *payload) {
    esp_err_t err = esp_now_send(logger_mac_unicast, (uint8_t *)payload, sizeof(espnow_payload_t));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Send failed: %s", esp_err_to_name(err));
    }
}