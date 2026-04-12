#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dc_protocol.h"
#include "app_dashboard.h"

static const char *TAG = "LOGGER_DASH";

/**
 * @brief 將 Unix Timestamp 轉換為格式化的字串 (HH:MM:SS)
 */
static void _format_unix_time(int64_t timestamp, char *buf, size_t len) {
    time_t raw_time = (time_t)timestamp;
    struct tm *tm_info = gmtime(&raw_time); // 因為 Slave 已經加過 +8，這裡用 gmtime 避免二次偏移
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm_info);
}

static void _process_and_print_event(espnow_payload_t *msg) {
    char node_time_str[32];

    switch (msg->msg_type) {
        case MSG_TYPE_REGISTER:
            // 解析 Slave 傳過來的時間戳
            _format_unix_time(msg->data.reg.timestamp, node_time_str, sizeof(node_time_str));
            
            printf("\033[32m[REG]\033[0m \033[1mNode %02d\033[0m Registered!\n", msg->node_id);
            printf("      └─ Node Time: \033[35m%s\033[0m (UTC+8)\n", node_time_str);
            printf("      └─ Location : (%.7f, %.7f) | Alt: %.2fm\n",
                   (double)msg->data.reg.lat / 10000000.0,
                   (double)msg->data.reg.lon / 10000000.0,
                   msg->data.reg.alt);
            break;

        case MSG_TYPE_HEARTBEAT:
            // 心跳包通常印在一行就好，維持簡潔
            printf("\033[34m[HB]\033[0m Node %02d Status: GPS:%s | UART:%s\n",
                   msg->node_id,
                   msg->data.heartbeat.gps_fix_status ? "FIXED" : "SEARCH",
                   msg->data.heartbeat.uart_status ? "ON" : "OFF");
            break;

        case MSG_TYPE_UAV_EVENT:
            // 若為 Discover (1) 或 Lost (0)
            if (msg->data.uav_event.event_type == 1) {
                printf("\033[33m[WARN]\033[0m Node %02d: \033[1mDISCOVER\033[0m [%02X:%02X:%02X:%02X:%02X:%02X]\n",
                       msg->node_id, 
                       msg->data.uav_event.uav_id[0], msg->data.uav_event.uav_id[1],
                       msg->data.uav_event.uav_id[2], msg->data.uav_event.uav_id[3],
                       msg->data.uav_event.uav_id[4], msg->data.uav_event.uav_id[5]);
            } else {
                printf("\033[31m[INFO]\033[0m Node %02d: \033[1mLOST\033[0m Target\n", msg->node_id);
            }
            break;

        case MSG_TYPE_FILE_SAVED:
            printf("\033[36m[FILE]\033[0m Node %02d saved session to: %s\n", 
                   msg->node_id, msg->data.file.filename);
            break;

        default:
            break;
    }
}

static void dashboard_task(void *pvParameters) {
    QueueHandle_t rx_queue = (QueueHandle_t)pvParameters;
    espnow_payload_t msg;

    printf("\n--- Logger Dashboard Sequential Mode (UTC+8 Monitor) ---\n\n");

    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            _process_and_print_event(&msg);
        }
    }
}

void app_dashboard_start(QueueHandle_t rx_queue) {
    xTaskCreate(dashboard_task, "dash_task", 4096, (void *)rx_queue, 5, NULL);
}