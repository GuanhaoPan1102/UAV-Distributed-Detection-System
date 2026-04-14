#include <string.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "dc_protocol_rc.h" 
#include "drv_espnow_tx.h"
#include "app_file_manager.h" 

static const char *TAG = "DRV_ESPNOW_TX_RC";

// 請填寫 Logger 的 MAC 位址
static const uint8_t logger_mac_unicast[6] = {0x2C, 0xBC, 0xBB, 0xA7, 0xBF, 0xAC}; 

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data_len == sizeof(espnow_payload_t)) {
        espnow_payload_t *payload = (espnow_payload_t *)data;

        if (payload->msg_type == MSG_TYPE_CONTROL) {
            if (payload->data.ctrl.command == CTRL_CMD_FORCE_SAVE) {
                ESP_LOGW(TAG, "!!! [RC] MASTER COMMAND: FORCE SAVE RECEIVED !!!");
                
                // [修正] 改為安全觸發，不直接去碰 SD 卡
                app_file_manager_trigger_force_stop();
                
            } else if (payload->data.ctrl.command == CTRL_CMD_RESET_NODE) {
                ESP_LOGE(TAG, "!!! [RC] MASTER COMMAND: SYSTEM RESET !!!");
                // Reset 可以保留，因為它只是重開機，不會有搶奪指標的問題
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }
    }
}

void espnow_tx_init(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

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
        ESP_LOGD(TAG, "Send failed: %s", esp_err_to_name(err)); 
    }
}