#include <stdlib.h>
#include <string.h>
#include "drv_gps_neo6m.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "DRV_GPS_NEO6M";

// ==========================================
// [硬體腳位定義] UAV PCB 固定配置
// ==========================================
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN   5
#define GPS_RX_PIN   4
#define GPS_BAUD     9600

#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

static QueueHandle_t uart_queue;
static SemaphoreHandle_t gps_data_mutex = NULL; 

static gps_data_t _latest_data = {0};

// -----------------------------------------------------------------------------
// 私有函式：資料處理與解析
// -----------------------------------------------------------------------------
static int32_t _convert_nmea_to_scaled(double nmea_val) 
{
    int degrees = (int)(nmea_val / 100.0);           
    double minutes = nmea_val - (degrees * 100.0);   
    double decimal_deg = degrees + (minutes / 60.0); 
    return (int32_t)(decimal_deg * 10000000.0);      
}

static void _parse_rmc(char *line)
{
    char *rest = line;
    char *token;
    int field_index = 0;
    
    gps_data_t temp_data = {0};
    bool is_valid_flag = false;

    if (gps_data_mutex != NULL && xSemaphoreTake(gps_data_mutex, portMAX_DELAY)) {
        temp_data.altitude_m = _latest_data.altitude_m;
        xSemaphoreGive(gps_data_mutex);
    }

    while((token = strsep(&rest, ",")) != NULL ) {
        size_t len = strlen(token);

        switch (field_index) {
        case 2: // Status (A=Active, V=Void)
            if (len > 0) {
                if (token[0] == 'A') {
                    is_valid_flag = true;
                    temp_data.is_valid = 1;
                } else {
                    return; 
                }
            }
            break;
        case 3: // Latitude
            if (len > 0 && is_valid_flag) {
                double lat_raw = strtod(token, NULL); 
                temp_data.latitude_scaled = _convert_nmea_to_scaled(lat_raw);
            }
            break;
        case 4: // N/S Indicator
            if (len > 0 && is_valid_flag && token[0] == 'S') {
                temp_data.latitude_scaled *= -1;
            }
            break;
        case 5: // Longitude
            if (len > 0 && is_valid_flag) {
                double lon_raw = strtod(token, NULL); 
                temp_data.longitude_scaled = _convert_nmea_to_scaled(lon_raw);
            }
            break;
        case 6: // E/W Indicator
            if (len > 0 && is_valid_flag && token[0] == 'W') {
                temp_data.longitude_scaled *= -1;
            }
            break;
        case 7: // Speed over ground (knots)
            if (len > 0 && is_valid_flag) {
                double speed_knots = strtod(token, NULL);
                temp_data.speed_kph_scaled = (int16_t)(speed_knots * 185.2); 
            }
            break;
        default:
            break;
        }
        field_index++;
    }

    if (is_valid_flag && gps_data_mutex != NULL) {
        xSemaphoreTake(gps_data_mutex, portMAX_DELAY);
        _latest_data.is_valid = temp_data.is_valid;
        _latest_data.latitude_scaled = temp_data.latitude_scaled;
        _latest_data.longitude_scaled = temp_data.longitude_scaled;
        _latest_data.speed_kph_scaled = temp_data.speed_kph_scaled;
        _latest_data.altitude_m = temp_data.altitude_m;
        xSemaphoreGive(gps_data_mutex);
    }
}

static void _parse_gga(char *line)
{
    char *rest = line;
    char *token;
    int field_index = 0;
    
    int16_t parsed_alt = 0;
    bool alt_updated = false;

    while((token = strsep(&rest, ",")) != NULL ) {
        size_t len = strlen(token);

        switch (field_index) {
        case 6: // Fix Quality
            if (len > 0 && atoi(token) == 0) {
                return; 
            }
            break;
        case 9: // Altitude (MSL)
            if (len > 0) {
                parsed_alt = (int16_t)strtol(token, NULL, 10);
                alt_updated = true;
            }
            break;
        }
        field_index++;
    }

    if (alt_updated && gps_data_mutex != NULL) {
        xSemaphoreTake(gps_data_mutex, portMAX_DELAY);
        _latest_data.altitude_m = parsed_alt;
        xSemaphoreGive(gps_data_mutex);
    }
}

static void _process_nmea_line(char *line)
{
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        _parse_rmc(line);
    } else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        _parse_gga(line);
    }
}

// -----------------------------------------------------------------------------
// 硬體配置與 FreeRTOS Task
// -----------------------------------------------------------------------------

static void configure_neo6m(void)
{
    ESP_LOGI(TAG, "Configuring u-blox NEO-6M GPS...");

    // 1. UBX-CFG-RATE (5Hz)
    const uint8_t ubx_cfg_rate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 
        0xC8, 0x00, 0x01, 0x00, 0x00, 0x00, 
        0xDD, 0x46
    };
    uart_write_bytes(GPS_UART_NUM, (const char*)ubx_cfg_rate, sizeof(ubx_cfg_rate));
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // 2. UBX-CFG-PRT (115200 baud)
    const uint8_t ubx_cfg_prt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 
        0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
        0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 
        0x00, 0x00, 0x00, 0x00, 
        0xC0, 0x7E 
    };
    uart_write_bytes(GPS_UART_NUM, (const char*)ubx_cfg_prt, sizeof(ubx_cfg_prt));
    
    vTaskDelay(pdMS_TO_TICKS(200)); 
    uart_set_baudrate(GPS_UART_NUM, 115200);

    // 3. UBX-CFG-CFG (Save to BBR/Flash)
    const uint8_t ubx_cfg_save[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x07, 
        0x21, 0xAF 
    };
    uart_write_bytes(GPS_UART_NUM, (const char*)ubx_cfg_save, sizeof(ubx_cfg_save));
    
    ESP_LOGI(TAG, "NEO-6M Configured: 5Hz @ 115200 baud.");
}

static void gps_event_task(void *pvParameters)
{
    uart_event_t event;
    
    uint8_t* dtmp = (uint8_t*) calloc(RD_BUF_SIZE, sizeof(uint8_t));
    if (dtmp == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for GPS buffer.");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            
            size_t buffered_size;
            uart_get_buffered_data_len(GPS_UART_NUM, &buffered_size);
            
            if (buffered_size > 1000) {
                ESP_LOGW(TAG, "Buffer full (%d bytes), flushing old data!", buffered_size);
                uart_flush_input(GPS_UART_NUM);
                xQueueReset(uart_queue);
                continue; 
            }

            switch (event.type) {
            case UART_PATTERN_DET: {
                int pos = uart_pattern_pop_pos(GPS_UART_NUM);
                if (pos != -1) {
                    int len = uart_read_bytes(GPS_UART_NUM, dtmp, pos+1, pdMS_TO_TICKS(100));
                    if (len > 0 && len < RD_BUF_SIZE) {
                        dtmp[len] = '\0';
                        char *start = strchr((char *)dtmp, '$');
                        if (start != NULL) {
                            _process_nmea_line(start);
                        }
                    }
                } else {
                    uart_flush_input(GPS_UART_NUM);
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(GPS_UART_NUM);
                xQueueReset(uart_queue);
                break;
            default:
                break;
            }
        }
    }
    
    free(dtmp);
    vTaskDelete(NULL);
}

// -----------------------------------------------------------------------------
// 公開 API
// -----------------------------------------------------------------------------

void gnss_init(void)
{
    ESP_LOGI(TAG, "Initializing GNSS Parser...");

    if (gps_data_mutex == NULL) {
        gps_data_mutex = xSemaphoreCreateMutex();
    }

    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_enable_pattern_det_baud_intr(GPS_UART_NUM, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(GPS_UART_NUM, 20);

    configure_neo6m(); // 內部也改為直接使用硬體巨集

    xTaskCreate(gps_event_task, "gps_task", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "GPS Init Complete. (UART%d, TX:%d, RX:%d)", GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN);
}

gps_data_t gnss_get_data(void)
{
    gps_data_t copy_data = {0};
    
    if (gps_data_mutex != NULL) {
        xSemaphoreTake(gps_data_mutex, portMAX_DELAY);
        copy_data = _latest_data;
        xSemaphoreGive(gps_data_mutex);
    }
    
    return copy_data;
}