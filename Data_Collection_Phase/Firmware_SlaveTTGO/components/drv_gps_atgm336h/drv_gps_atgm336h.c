#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "drv_gps_atgm336h.h"

static const char *TAG = "DRV_GPS";

#define GPS_BUF_SIZE 1024
#define UTC_OFFSET_SEC (8 * 3600) // 台北時區加 8 小時

// --- CASIC 指令 ---
const char* GPS_CMD_BAUD_115200 = "$PCAS01,5*19\r\n";
const char* GPS_CMD_10HZ         = "$PCAS02,100*1E\r\n";
const char* GPS_CMD_REDUCE_MSG   = "$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n";
const char* GPS_CMD_SAVE         = "$PCAS00*01\r\n";

// --- 全域變數 ---
static gps_atgm_status_t g_status = {0};
static SemaphoreHandle_t g_gps_mutex = NULL; 
static int g_uart_num = UART_NUM_1;

// --- [核心同步變數] ---
static volatile time_t _next_pps_timestamp = 0; // 預測下一秒的時間戳 (UTC+8)

// ==========================================
// PPS 中斷處理 (ISR) - 對齊以前的 gnss_sync.c
// ==========================================
static void IRAM_ATTR pps_isr_handler(void* arg)
{
    if (!g_status.is_time_synced) {
        // 階段一: 錨點初始化 (第一次對齊)
        if (_next_pps_timestamp > 0) {
            struct timeval tv;
            tv.tv_sec = _next_pps_timestamp;
            tv.tv_usec = 0; // PPS 到達的瞬間，微秒絕對為 0
            settimeofday(&tv, NULL);
            g_status.is_time_synced = true;
        }
    } else {
        // 階段二: 硬體同步 (鎖定後，每一秒直接遞增並歸零微秒)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        tv.tv_sec += 1;
        tv.tv_usec = 0; 
        settimeofday(&tv, NULL);
    }
}

// 輔助：NMEA 座標轉換
static int32_t parse_nmea_coord(const char *coord_str, char dir) {
    if (!coord_str || strlen(coord_str) < 4) return 0;
    double val = atof(coord_str);
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);
    if (dir == 'S' || dir == 'W') decimal_degrees = -decimal_degrees;
    return (int32_t)(decimal_degrees * 10000000.0);
}

// --- [核心] 解析 RMC 並預測下一秒 ---
static void _process_rmc_for_sync(const char *time_str, const char *date_str) {
    if (strlen(time_str) < 6 || strlen(date_str) < 6) return;

    struct tm tm_struct = {0};
    char tmp[3] = {0};

    // 解析 HHMMSS
    strncpy(tmp, time_str, 2);     tm_struct.tm_hour = atoi(tmp);
    strncpy(tmp, time_str + 2, 2); tm_struct.tm_min  = atoi(tmp);
    strncpy(tmp, time_str + 4, 2); tm_struct.tm_sec  = atoi(tmp);
    // 解析 DDMMYY
    strncpy(tmp, date_str, 2);     tm_struct.tm_mday = atoi(tmp);
    strncpy(tmp, date_str + 2, 2); tm_struct.tm_mon  = atoi(tmp) - 1;
    strncpy(tmp, date_str + 4, 2); tm_struct.tm_year = atoi(tmp) + 100;

    // 強制時區為 UTC 避免 mktime 亂跑
    setenv("TZ", "UTC", 1);
    tzset();

    time_t current_gps_utc = mktime(&tm_struct);
    
    if (!g_status.is_time_synced) {
        // 階段一準備：預測下一秒 = 這次時間 + 1秒 + 8小時偏移
        _next_pps_timestamp = current_gps_utc + 1 + UTC_OFFSET_SEC;
    } else {
        // 階段三：背景稽核 (Watchdog)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        // 將系統目前的「本地時間」轉回比較基準
        int64_t diff = llabs((int64_t)tv.tv_sec - (int64_t)(current_gps_utc + UTC_OFFSET_SEC));
        if (diff >= 2) {
            ESP_LOGE(TAG, "Watchdog: Drift detected (%llds). Re-syncing...", diff);
            g_status.is_time_synced = false;
        }
    }
}

static void gps_uart_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(GPS_BUF_SIZE);
    char line[128];
    int line_len = 0;

    while (1) {
        int rx_bytes = uart_read_bytes(g_uart_num, data, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(50));
        if (rx_bytes > 0) {
            for (int i = 0; i < rx_bytes; i++) {
                char c = (char)data[i];
                if (c == '\n' || line_len >= sizeof(line) - 1) {
                    line[line_len] = '\0';
                    
                    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
                        char *token = strtok(line, ",");
                        int field = 0, fix_quality = 0;
                        float alt = 0.0;
                        while (token != NULL) {
                            if (field == 6) fix_quality = atoi(token);
                            if (field == 9) alt = atof(token);
                            token = strtok(NULL, ",");
                            field++;
                        }
                        xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
                        g_status.is_fixed = (fix_quality > 0);
                        g_status.altitude = alt;
                        xSemaphoreGive(g_gps_mutex);
                    } 
                    else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
                        char *token = strtok(line, ",");
                        int field = 0;
                        char lat_str[20]={0}, lon_str[20]={0}, time_str[20]={0}, date_str[20]={0};
                        char lat_dir=0, lon_dir=0, status_char='V';

                        while (token != NULL) {
                            if (field == 1) strncpy(time_str, token, 19);
                            if (field == 2) status_char = token[0];
                            if (field == 3) strncpy(lat_str, token, 19);
                            if (field == 4) lat_dir = token[0];
                            if (field == 5) strncpy(lon_str, token, 19);
                            if (field == 6) lon_dir = token[0];
                            if (field == 9) strncpy(date_str, token, 19);
                            token = strtok(NULL, ",");
                            field++;
                        }

                        if (status_char == 'A') {
                            int32_t lat = parse_nmea_coord(lat_str, lat_dir);
                            int32_t lon = parse_nmea_coord(lon_str, lon_dir);
                            
                            // 同步核心：預測下一秒
                            _process_rmc_for_sync(time_str, date_str);

                            xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
                            g_status.latitude = lat;
                            g_status.longitude = lon;
                            xSemaphoreGive(g_gps_mutex);
                        }
                    }
                    line_len = 0;
                } else if (c != '\r') {
                    line[line_len++] = c;
                }
            }
        }
    }
    free(data);
}

void gps_atgm_init(int uart_num, int tx_pin, int rx_pin, int pps_pin) {
    g_uart_num = uart_num;
    g_gps_mutex = xSemaphoreCreateMutex();

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(g_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_uart_num, tx_pin, rx_pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(g_uart_num, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));

    // 配置 115200 / 10Hz
    uart_write_bytes(g_uart_num, GPS_CMD_BAUD_115200, strlen(GPS_CMD_BAUD_115200));
    vTaskDelay(pdMS_TO_TICKS(200)); 
    uart_set_baudrate(g_uart_num, 115200);
    uart_write_bytes(g_uart_num, GPS_CMD_10HZ, strlen(GPS_CMD_10HZ));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(g_uart_num, GPS_CMD_REDUCE_MSG, strlen(GPS_CMD_REDUCE_MSG));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(g_uart_num, GPS_CMD_SAVE, strlen(GPS_CMD_SAVE));

    // PPS 中斷
    if (pps_pin >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pps_pin),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(pps_pin, pps_isr_handler, NULL);
    }
    xTaskCreate(gps_uart_task, "gps_task", 4096, NULL, 12, NULL);
}

gps_atgm_status_t gps_atgm_get_status(void) {
    gps_atgm_status_t current_status;
    xSemaphoreTake(g_gps_mutex, portMAX_DELAY);
    current_status = g_status;
    xSemaphoreGive(g_gps_mutex);
    return current_status;
}