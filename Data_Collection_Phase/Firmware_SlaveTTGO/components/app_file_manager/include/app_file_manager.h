#ifndef APP_FILE_MANAGER_H
#define APP_FILE_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief 啟動檔案管理任務
 * @param input_queue 來自 drv_uart_rx 的資料隊列，結構為 uart_scanner_to_ttgo_t
 */
void app_file_manager_start(QueueHandle_t input_queue);

#endif