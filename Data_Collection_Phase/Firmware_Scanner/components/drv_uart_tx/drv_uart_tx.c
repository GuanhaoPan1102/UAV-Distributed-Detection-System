#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "drv_uart_tx.h"

static const char *TAG = "DRV_UART_TX";
static int g_uart_num = UART_NUM_2;
static SemaphoreHandle_t tx_mutex = NULL; // 互斥鎖

void uart_tx_init(int uart_num, int tx_pin, int rx_pin, int rts_pin, int cts_pin)
{
    g_uart_num = uart_num;

    // 建立互斥鎖
    tx_mutex = xSemaphoreCreateMutex();
    if (tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX Mutex!");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS, 
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(g_uart_num, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(g_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_uart_num, tx_pin, rx_pin, rts_pin, cts_pin));

    ESP_LOGI(TAG, "UART TX Initialized (Thread-Safe & HW Flow Control ON)");
}

void uart_tx_send_packet(uart_scanner_to_ttgo_t *packet)
{
    packet->header[0] = UART_HEADER_0;
    packet->header[1] = UART_HEADER_1;

    uint8_t *ptr = (uint8_t *)packet;
    uint8_t xor_sum = 0;
    size_t data_len = sizeof(uart_scanner_to_ttgo_t);
    
    for (size_t i = 0; i < data_len - 1; i++) {
        xor_sum ^= ptr[i];
    }
    packet->uart_checksum = xor_sum;

    // --- 取得互斥鎖才允許寫入 UART ---
    if (xSemaphoreTake(tx_mutex, portMAX_DELAY) == pdTRUE) {
        int len = uart_write_bytes(g_uart_num, (const char *)packet, data_len);
        if (len < 0) {
            ESP_LOGE(TAG, "UART Write Failed!");
        }
        // 寫入完畢，釋放鎖
        xSemaphoreGive(tx_mutex);
    }
}