#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

#include "dc_protocol.h"  // 確保此處已統一名稱為 dc_protocol.h
#include "drv_ble_scan.h"

static const char *TAG = "DRV_BLE_SCAN";
static QueueHandle_t g_scan_queue = NULL;

// 掃描參數設定 (維持被動掃描與停用重複過濾)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x0050, // 50ms
    .scan_window            = 0x0050, // 50ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE 
};

/**
 * @brief 預過濾 Checksum 檢查
 * @return true 代表校驗通過，為真正的 UAV 封包
 */
static bool verify_uav_packet(uint8_t *data, uint8_t len) {
    // 1. 長度檢查：若剩餘長度不足以容納一個完整的 payload，直接剔除
    if (len < sizeof(uav_ble_payload_t)) {
        return false;
    }

    // 2. Checksum 檢查 (XOR 校驗)
    uint8_t calculated_xor = 0;
    // 計算除了最後一個 byte (checksum 欄位) 以外的所有資料
    for (int i = 0; i < sizeof(uav_ble_payload_t) - 1; i++) {
        calculated_xor ^= data[i];
    }

    // 取得封包內宣告的 checksum
    uint8_t expected_xor = data[sizeof(uav_ble_payload_t) - 1];

    return (calculated_xor == expected_xor);
}



static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t *adv_data = scan_result->scan_rst.ble_adv;
            uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;

            // 遍歷封包尋找特徵碼
            for (int i = 0; i < adv_data_len; i++) {
                if (adv_data[i] == BLE_PACKET_IDENTIFIER) {
                    
                    // 檢查從 i 開始的資料是否符合 UAV 結構與校驗
                    if (verify_uav_packet(&adv_data[i], adv_data_len - i)) {
                        
                        // 通過校驗，判定為目標無人機，捕捉時間戳
                        int64_t now_us = esp_timer_get_time();

                        scanner_queue_item_t item;
                        item.timestamp_us = now_us;
                        item.rssi = scan_result->scan_rst.rssi;
                        memcpy(item.mac, scan_result->scan_rst.bda, 6);
                        memcpy(&item.payload, &adv_data[i], sizeof(uav_ble_payload_t));

                        if (g_scan_queue != NULL) {
                            if (xQueueSend(g_scan_queue, &item, 0) != pdPASS) {
                                // 隊列滿了通常代表 UART 傳輸或狀態機卡住
                                ESP_LOGW(TAG, "Scan Queue Overflow!");
                            }
                        }
                        // 找到一個有效的 UAV Payload 後即可跳出此封包的尋找
                        break; 
                    }
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

void ble_scan_init(QueueHandle_t output_queue)
{
    g_scan_queue = output_queue;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
    
    ESP_LOGI(TAG, "BLE Scanner active with Checksum Filtering.");
}

void ble_scan_start(void) { esp_ble_gap_start_scanning(0); }
void ble_scan_stop(void)  { esp_ble_gap_stop_scanning(); }