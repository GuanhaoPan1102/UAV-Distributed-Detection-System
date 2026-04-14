#ifndef APP_DASHBOARD_H
#define APP_DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// 務必引用 RC 版本的通訊協定
#include "dc_protocol_rc.h"

/**
 * @brief 初始化 Dashboard (印出歡迎畫面)
 */
void app_dashboard_init(void);

/**
 * @brief 啟動 Dashboard 監聽任務
 * @param rx_queue 來自 ESP-NOW RX 的資料佇列
 */
void app_dashboard_start(QueueHandle_t rx_queue);

/**
 * @brief 處理並印出單一事件 (公開此函數，讓 main.c 可以手動觸發 UI 回饋)
 * @param msg 傳入的 ESP-NOW 封包指標
 */
void app_dashboard_print_event(espnow_payload_t *msg);

#endif // APP_DASHBOARD_H