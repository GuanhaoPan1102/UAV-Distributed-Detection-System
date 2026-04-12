#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dc_protocol.h"
#include "drv_gps_neo6m.h"
#include "drv_ble_adv.h"

static const char *TAG = "MAIN";

// ... (保持 _test_gps() 邏輯不變) ...

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("DRV_GPS_NEO6M", ESP_LOG_INFO);
    esp_log_level_set("BLE_BROADCAST", ESP_LOG_INFO);
    
    // 現在初始化變得極度乾淨
    gnss_init();
    
    ble_adv_init();

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, " UAV System Started");
    ESP_LOGI(TAG, " GNSS Rate: 5Hz, BLE Rate: 10Hz");
    ESP_LOGI(TAG, "==========================================");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}