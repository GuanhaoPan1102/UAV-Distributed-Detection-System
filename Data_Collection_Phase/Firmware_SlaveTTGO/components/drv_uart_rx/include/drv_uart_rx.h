#ifndef DRV_UART_RX_H
#define DRV_UART_RX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "dc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UART 接收驅動 (具備 RTS/CTS 流控)
 * @param output_queue 接收成功後的資料隊列 (存放類型為 uart_scanner_to_ttgo_t)
 * @param uart_num 使用的 UART 編號
 * @param tx_pin, rx_pin, rts_pin, cts_pin 腳位配置
 */
void uart_rx_init(QueueHandle_t output_queue, int uart_num, int tx_pin, int rx_pin, int rts_pin, int cts_pin);

#ifdef __cplusplus
}
#endif

#endif