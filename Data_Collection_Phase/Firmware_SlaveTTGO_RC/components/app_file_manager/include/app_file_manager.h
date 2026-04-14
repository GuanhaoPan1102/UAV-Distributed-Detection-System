#ifndef APP_FILE_MANAGER_H
#define APP_FILE_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void app_file_manager_start(QueueHandle_t input_queue);

/**
 * @brief [修正] 透過 Queue 觸發強制斷檔 (安全的中斷呼叫方式)
 */
void app_file_manager_trigger_force_stop(void);

#endif