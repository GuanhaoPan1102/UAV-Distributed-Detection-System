#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "dc_protocol_rc.h"
#include "drv_uart_rx.h"

static const char *TAG = "DRV_UART_RX";
static int g_uart_num;

/**
 * @brief UART 接收狀態機任務
 */
static void uart_rx_task(void *pvParameters) {
    QueueHandle_t out_queue = (QueueHandle_t)pvParameters;
    uint8_t rx_byte;
    int state = 0;
    
    // 封包總長度與資料段長度 (排除 Header 2 bytes 與 Checksum 1 byte)
    const size_t total_size = sizeof(uart_scanner_to_ttgo_t);
    const size_t body_size = total_size - 3; 
    
    uint8_t body_buf[sizeof(uart_scanner_to_ttgo_t)]; 
    int body_idx = 0;

    ESP_LOGI(TAG, "UART Receiver Task started (Packet Size: %d bytes)", total_size);

    while (1) {
        // 逐字讀取以精確對齊 Header
        if (uart_read_bytes(g_uart_num, &rx_byte, 1, portMAX_DELAY) > 0) {
            switch (state) {
                case 0: // 等待 Header 0 (0xEB)
                    if (rx_byte == UART_HEADER_0) state = 1;
                    break;

                case 1: // 等待 Header 1 (0x90)
                    if (rx_byte == UART_HEADER_1) {
                        state = 2;
                        body_idx = 0;
                    } else if (rx_byte == UART_HEADER_0) {
                        state = 1;
                    } else {
                        state = 0;
                    }
                    break;

                case 2: // 接收資料段 (MAC, Event, Age, Payload, RSSI)
                    body_buf[body_idx++] = rx_byte;
                    if (body_idx >= body_size) state = 3;
                    break;

                case 3: { // 驗證 Checksum
                    uint8_t xor_calc = UART_HEADER_0 ^ UART_HEADER_1;
                    for (int k = 0; k < body_size; k++) {
                        xor_calc ^= body_buf[k];
                    }

                    if (xor_calc == rx_byte) {
                        // --- [核心] 封包驗證成功，組裝結構體 ---
                        uart_scanner_to_ttgo_t item;
                        item.header[0] = UART_HEADER_0;
                        item.header[1] = UART_HEADER_1;
                        
                        // 依照 dc_protocol.h 的順序複製資料
                        // body_buf 依序含: mac(6), event(1), age(4), ble_data(16)
                        memcpy(&item.mac, body_buf, body_size);
                        item.uart_checksum = rx_byte;

                        // 將完整的封包送入隊列，交由 app_file_manager 處理
                        if (xQueueSend(out_queue, &item, 0) != pdPASS) {
                            ESP_LOGW(TAG, "Queue Full! Data dropped.");
                        }
                    } else {
                        ESP_LOGW(TAG, "Checksum Mismatch! Calc:0x%02X, Recv:0x%02X", xor_calc, rx_byte);
                    }
                    state = 0;
                    break;
                }
            }
        }
    }
}

void uart_rx_init(QueueHandle_t output_queue, int uart_num, int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    g_uart_num = uart_num;

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        // 啟用硬體流控：當 Slave 忙碌時拉高 RTS 告知 Scanner 暫停發送
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS, 
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024 * 4, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, rts_pin, cts_pin));

    // 接收任務設為高優先權，確保及時處理 UART 緩衝
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, (void *)output_queue, 15, NULL);
    
    ESP_LOGI(TAG, "UART RX initialized on UART %d (RTS:%d, CTS:%d)", uart_num, rts_pin, cts_pin);
}