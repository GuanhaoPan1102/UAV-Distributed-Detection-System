#ifndef DRV_ESPNOW_RX_H
#define DRV_ESPNOW_RX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "dc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Wi-Fi 站點模式與 ESP-NOW 接收功能
 * @param rx_queue 將收到的 espnow_payload_t 送入此佇列
 * @note 必須在 nvs_flash_init() 之後呼叫
 */
void espnow_rx_init(QueueHandle_t rx_queue);

#ifdef __cplusplus
}
#endif

#endif // DRV_ESPNOW_RX_H