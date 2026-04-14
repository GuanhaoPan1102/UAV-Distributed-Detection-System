#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dc_protocol.h"
#include "drv_ble_adv.h"
#include "drv_gps_neo6m.h" // [引用自另一份 driver] 用於抓取 GPS 資料

static const char *TAG = "DRV_BLE_ADV";
static bool g_is_advertising = false;

// 1. 藍牙廣播參數：採用不可連接模式 (ADV_TYPE_NONCONN_IND)
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x00A0, // 100ms
    .adv_int_max        = 0x00A0, 
    .adv_type           = ADV_TYPE_NONCONN_IND, 
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
    for (size_t i = 0; i < sizeof(uav_ble_payload_t) - 1; i++) {
        xor_val ^= ptr[i];
    }
    return xor_val;
}

/**
 * @brief [核心改動]：將 GPS 數據封裝並配置為 RAW 廣播格式
 */
static void _update_and_config_raw_data(gps_data_t *gps) {
    static uint8_t seq_counter = 0;
    uav_ble_payload_t payload = {
        .identifier = BLE_PACKET_IDENTIFIER, // 0xAA
        .seq_num = seq_counter++,
        .lat = gps->latitude_scaled,
        .lon = gps->longitude_scaled,
        .alt = gps->altitude_m,
        .spd = gps->speed_kph_scaled
    };
    payload.checksum = calculate_uav_checksum(&payload);

    // 封裝成標準廣告結構 (Flags + Manufacturer Data)
    uint8_t raw_adv_data[31];
    uint8_t adv_idx = 0;

    raw_adv_data[adv_idx++] = 0x02; // Flags Length
    raw_adv_data[adv_idx++] = 0x01; // Type: Flags
    raw_adv_data[adv_idx++] = 0x06; 

    raw_adv_data[adv_idx++] = sizeof(uav_ble_payload_t) + 1; // Mfg Data Length
    raw_adv_data[adv_idx++] = 0xFF; // Type: Mfg Data
    memcpy(&raw_adv_data[adv_idx], &payload, sizeof(uav_ble_payload_t));
    adv_idx += sizeof(uav_ble_payload_t);

    esp_ble_gap_config_adv_data_raw(raw_adv_data, adv_idx);
}

/**
 * @brief 藍牙事件處理
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            if (!g_is_advertising) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                g_is_advertising = true;
                ESP_LOGI(TAG, "Advertising successfully started");
            }
            break;
        default: break;
    }
}

/**
 * @brief [引用自另一份 driver]：10Hz 自動廣播任務
 */
static void ble_broadcast_task(void *pvParameters) {
    ESP_LOGI(TAG, "BLE 10Hz Broadcast Task Started.");
    while(1) {
        // 1. 取得最新的 GPS 資料
        gps_data_t current_gps = gnss_get_data();

        // 2. 更新並觸發廣播
        _update_and_config_raw_data(&current_gps);

        // 3. 延遲 100ms (10Hz)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ble_adv_init(void) {
    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // 2. [新增改動]：設定最大發射功率 (+9dBm)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);

    esp_ble_gap_register_callback(gap_event_handler);

    // 3. [引用自另一份 driver]：在 Driver 內部直接啟動任務，不煩擾 main.c
    xTaskCreate(ble_broadcast_task, "ble_tx_task", 4096, NULL, 10, NULL);
}