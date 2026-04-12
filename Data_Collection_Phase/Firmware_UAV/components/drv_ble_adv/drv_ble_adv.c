#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

#include "dc_protocol.h"
#include "drv_ble_adv.h"

static const char *TAG = "DRV_BLE_ADV";

// 藍牙廣播參數設定
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x00A0, // 100ms (10Hz 頻率)
    .adv_int_max        = 0x00A0, 
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/**
 * @brief 計算 XOR 校驗碼
 */
static uint8_t calculate_uav_checksum(uav_ble_payload_t *payload) {
    uint8_t xor_val = 0;
    uint8_t *ptr = (uint8_t *)payload;
    // 計算除了最後一個 byte (checksum 欄位) 以外的所有資料
    for (size_t i = 0; i < sizeof(uav_ble_payload_t) - 1; i++) {
        xor_val ^= ptr[i];
    }
    return xor_val;
}

/**
 * @brief 更新廣播內容並即時發射
 */
void ble_adv_update_data(uav_ble_payload_t *payload) {
    // 1. 強制填入識別碼並計算校驗
    payload->identifier = BLE_PACKET_IDENTIFIER; // 0xAA
    payload->checksum = calculate_uav_checksum(payload);

    // 2. 封裝成標準藍牙廣播格式 (AD Structure)
    // 格式：[長度] [類型:0xFF 廠商自定義] [自定義資料...]
    uint8_t raw_adv_data[31];
    uint8_t adv_idx = 0;

    // Flags: 一般可發現模式
    raw_adv_data[adv_idx++] = 0x02; // Length
    raw_adv_data[adv_idx++] = 0x01; // AD Type: Flags
    raw_adv_data[adv_idx++] = 0x06; // Data

    // Manufacturer Specific Data
    raw_adv_data[adv_idx++] = sizeof(uav_ble_payload_t) + 1; // Length (Payload + Type)
    raw_adv_data[adv_idx++] = 0xFF;                          // AD Type: Manufacturer Specific Data
    
    // 填入 Payload 內容
    memcpy(&raw_adv_data[adv_idx], payload, sizeof(uav_ble_payload_t));
    adv_idx += sizeof(uav_ble_payload_t);

    // 3. 配置到硬體
    esp_err_t err = esp_ble_gap_config_adv_data_raw(raw_adv_data, adv_idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config RAW adv data failed: %s", esp_err_to_name(err));
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
        default:
            break;
    }
}

void ble_adv_init(void) {
    esp_err_t ret;

    // 1. 釋放傳統藍牙記憶體
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 2. 初始化 BT 控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(TAG, "Init controller failed: %s", esp_err_to_name(ret)); return; }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(TAG, "Enable controller failed: %s", esp_err_to_name(ret)); return; }

    // 3. 初始化 Bluedroid 堆疊
    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(TAG, "Init bluedroid failed: %s", esp_err_to_name(ret)); return; }

    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(TAG, "Enable bluedroid failed: %s", esp_err_to_name(ret)); return; }

    // 4. 註冊 GAP 回調並設定參數
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) { ESP_LOGE(TAG, "GAP register failed: %s", esp_err_to_name(ret)); return; }

    ESP_LOGI(TAG, "BLE Advertising Initialized with 10Hz Interval.");
}