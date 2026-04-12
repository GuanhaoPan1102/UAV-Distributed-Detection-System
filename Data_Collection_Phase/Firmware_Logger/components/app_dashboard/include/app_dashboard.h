#ifndef APP_DASHBOARD_H
#define APP_DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 啟動儀表板任務，開始接管 UI 顯示與狀態維護
 * @param rx_queue 來自 drv_espnow_rx 的封包佇列
 */
void app_dashboard_start(QueueHandle_t rx_queue);

#ifdef __cplusplus
}
#endif

#endif // APP_DASHBOARD_H