#ifndef DRV_BLE_SCAN_H
#define DRV_BLE_SCAN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 BLE 掃描器
 * @param output_queue 掃描結果存放的隊列 (存放類型為 scanner_queue_item_t)
 */
void ble_scan_init(QueueHandle_t output_queue);

/**
 * @brief 手動控制掃描開關 (選用)
 */
void ble_scan_start(void);
void ble_scan_stop(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_BLE_SCAN_H