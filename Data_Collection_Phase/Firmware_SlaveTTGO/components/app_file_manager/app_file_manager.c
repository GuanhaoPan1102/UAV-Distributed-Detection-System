#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "dc_protocol.h"
#include "drv_espnow_tx.h" 
#include "app_file_manager.h"

static const char *TAG = "APP_FILE_MGR";

// 宣告定義在 main.c 的函數
extern void uart_watchdog_feed(void);

#define SLAVE_NODE_ID 1
#define MAX_OPEN_FILES 5
#define UAV_TRACKING_LED_PIN 21
#define SD_BASE_PATH "/sdcard"

static uint32_t g_data_file_index = 1; 

typedef struct {
    uint8_t mac[6];
    FILE *f;
    char filename[32];
    uint32_t session_id;
} active_file_t;

static active_file_t g_files[MAX_OPEN_FILES];

/**
 * 啟動時掃描 SD 卡，續接 dataN.csv 的編號
 */
static void _find_next_file_index(const char* base_path) {
    DIR *dir = opendir(base_path);
    if (!dir) return;
    uint32_t max_idx = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "data", 4) == 0) {
            uint32_t idx = 0;
            if (sscanf(entry->d_name, "data%lu.csv", (unsigned long *)&idx) == 1) {
                if (idx > max_idx) max_idx = idx;
            }
        }
    }
    closedir(dir);
    g_data_file_index = max_idx + 1;
    ESP_LOGI(TAG, "Next file index initialized: %lu", (unsigned long)g_data_file_index);
}

/**
 * 更新 GPIO 21 指示燈
 */
static void _update_tracking_led(void) {
    int active_sessions = 0;
    for (int i = 0; i < MAX_OPEN_FILES; i++) {
        if (g_files[i].f != NULL) active_sessions++;
    }
    gpio_set_level(UAV_TRACKING_LED_PIN, active_sessions > 0 ? 1 : 0);
}

/**
 * 透過 ESP-NOW 回報事件給 Logger
 */
static void _report_to_logger(uint8_t type, uint8_t *uav_mac, const char *fname) {
    espnow_payload_t msg = {0};
    msg.msg_type = type;
    msg.node_id = SLAVE_NODE_ID;
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t now_us = (int64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;

    if (type == MSG_TYPE_UAV_EVENT) {
        msg.data.uav_event.timestamp_us = now_us;
        msg.data.uav_event.event_type = (uav_mac != NULL) ? 1 : 0; 
        if (uav_mac) memcpy(msg.data.uav_event.uav_id, uav_mac, 6);
    } else if (type == MSG_TYPE_FILE_SAVED) {
        msg.data.file.timestamp = now_us;
        if (fname) strncpy(msg.data.file.filename, fname, sizeof(msg.data.file.filename)-1);
    }
    if (type == MSG_TYPE_FILE_SAVED) {
        for (int i = 0; i < 3; i++) {
            espnow_report(&msg);
            vTaskDelay(pdMS_TO_TICKS(100)); // 間隔 100ms，避開瞬間干擾
        }
    } else {
        // 一般事件（Discover/Lost）發一次即可，或是你覺得保險也可以發 2 次
        espnow_report(&msg);
    }
}

/**
 * 檔案與追蹤 Session 管理
 */
static FILE* _manage_session_file(uint8_t *mac, uint8_t event_flag) {
    for (int i = 0; i < MAX_OPEN_FILES; i++) {
        if (g_files[i].f != NULL && memcmp(g_files[i].mac, mac, 6) == 0) {
            if (event_flag == 0xFF) { 
                ESP_LOGW(TAG, "LOST UAV [%02X:%02X:%02X:%02X:%02X:%02X], closing %s",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], g_files[i].filename);
                
                _report_to_logger(MSG_TYPE_FILE_SAVED, NULL, g_files[i].filename);
                fclose(g_files[i].f);
                g_files[i].f = NULL;
                _update_tracking_led();
                return NULL;
            }
            return g_files[i].f;
        }
    }

    if (event_flag == 0xFF) return NULL;

    for (int i = 0; i < MAX_OPEN_FILES; i++) {
        if (g_files[i].f == NULL) {
            memcpy(g_files[i].mac, mac, 6);
            g_files[i].session_id = g_data_file_index++;
            snprintf(g_files[i].filename, sizeof(g_files[i].filename), "%s/data%lu.csv", 
                     SD_BASE_PATH, (unsigned long)g_files[i].session_id);
            
            g_files[i].f = fopen(g_files[i].filename, "w");
            if (g_files[i].f) {
                ESP_LOGI(TAG, "DISCOVER UAV [%02X:%02X:%02X:%02X:%02X:%02X], recording to %s",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], g_files[i].filename);
                
                fprintf(g_files[i].f, "UTC_Time_us,RSSI,Lat,Lon,Alt,Spd,Seq\n");
                fflush(g_files[i].f);
                _report_to_logger(MSG_TYPE_UAV_EVENT, mac, NULL);
                _update_tracking_led();
                return g_files[i].f;
            }
        }
    }
    return NULL;
}

/**
 * 主要檔案處理任務
 */
static void file_manager_task(void *pvParameters) {
    QueueHandle_t input_queue = (QueueHandle_t)pvParameters;
    uart_scanner_to_ttgo_t rx_data;

    while (1) {
        if (xQueueReceive(input_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            // 收到任何串口封包都餵食看門狗
            uart_watchdog_feed();

            // 過濾心跳包
            if (rx_data.event_flag == 0xEE) continue;

            // 取得精準微秒時間戳
            struct timeval tv;
            gettimeofday(&tv, NULL);
            uint64_t now_us = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;

            FILE *f = _manage_session_file(rx_data.mac, rx_data.event_flag);

            if (f && rx_data.event_flag != 0xFF) {
                // 嵌套結構數據解包並寫入
                fprintf(f, "%llu,%d,%ld,%ld,%d,%d,%u\n",
                        now_us,
                        (int)rx_data.ble_data.rssi,
                        (long)rx_data.ble_data.payload.lat,
                        (long)rx_data.ble_data.payload.lon,
                        (int)rx_data.ble_data.payload.alt,
                        (int)rx_data.ble_data.payload.spd,
                        (unsigned int)rx_data.ble_data.payload.seq_num);
                
                fflush(f);
            }
        }
    }
}

void app_file_manager_start(QueueHandle_t input_queue) {
    gpio_reset_pin(UAV_TRACKING_LED_PIN);
    gpio_set_direction(UAV_TRACKING_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UAV_TRACKING_LED_PIN, 0);

    _find_next_file_index(SD_BASE_PATH);

    xTaskCreate(file_manager_task, "file_mgr_task", 4096, (void*)input_queue, 10, NULL);
    
    ESP_LOGI(TAG, "File Manager Started.");
}