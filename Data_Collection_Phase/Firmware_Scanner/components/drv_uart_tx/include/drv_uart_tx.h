#ifndef DRV_UART_TX_H
#define DRV_UART_TX_H

#include "dc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化具備硬體流控 (CTS/RTS) 的 UART 模組
 * @param uart_num 使用的 UART 編號 (建議使用 UART_NUM_2)
 * @param tx_pin TX 腳位 (Scanner 發送)
 * @param rx_pin RX 腳位
 * @param rts_pin RTS 腳位 (Scanner 準備好接收)
 * @param cts_pin CTS 腳位 (Scanner 聽從 Slave 停止發送)
 */
void uart_tx_init(int uart_num, int tx_pin, int rx_pin, int rts_pin, int cts_pin);

/**
 * @brief 發送一筆符合 dc_protocol 規範的 UART 封包
 * @param packet 準備發送的結構體指針
 */
void uart_tx_send_packet(uart_scanner_to_ttgo_t *packet);

#ifdef __cplusplus
}
#endif

#endif // DRV_UART_TX_H