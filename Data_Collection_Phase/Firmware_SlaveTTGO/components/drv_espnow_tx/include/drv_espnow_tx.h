#ifndef DRV_ESPNOW_TX_H
#define DRV_ESPNOW_TX_H

#include "dc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Wi-Fi 站點模式與 ESP-NOW 發送功能
 * @note 必須在 nvs_flash_init() 之後呼叫
 */
void espnow_tx_init(void);

/**
 * @brief 將依照 dc_protocol 封裝好的資料發送給 Logger
 * @param msg 指向 espnow_payload_t 結構體的指標
 */
void espnow_report(espnow_payload_t *msg);

#ifdef __cplusplus
}
#endif

#endif // DRV_ESPNOW_TX_H