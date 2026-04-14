#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dc_protocol_rc.h"
#include "app_dashboard.h"

static const char *TAG = "LOGGER_DASH";

/**
 * @brief 將 Unix Timestamp 轉換為格式化的字串 (HH:MM:SS)
 */
static void _format_unix_time(int64_t timestamp, char *buf, size_t len) {
    time_t raw_time = (time_t)timestamp;
    // 因為 Slave 在傳送前已經加過 +8 小時偏移，這裡用 gmtime 避免二次偏移
    struct tm *tm_info = gmtime(&raw_time); 
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm_info);
}

/**
 * @brief 處理並列印所有來自節點的事件 (已改為公開函數)
 */
void app_dashboard_print_event(espnow_payload_t *msg) {
    char node_time_str[32];

    switch (msg->msg_type) {
        case MSG_TYPE_REGISTER:
            _format_unix_time(msg->data.reg.timestamp, node_time_str, sizeof(node_time_str));
            
            printf("\033[32m[REG]\033[0m \033[1mNode %02d\033[0m Registered!\n", msg->node_id);
            printf("      └─ Node Time: \033[35m%s\033[0m (UTC+8)\n", node_time_str);
            printf("      └─ Location : (%.7f, %.7f) | Alt: %.2fm\n",
                   (double)msg->data.reg.lat / 10000000.0,
                   (double)msg->data.reg.lon / 10000000.0,
                   msg->data.reg.alt);
            break;

        case MSG_TYPE_HEARTBEAT:
            // 心跳包維持簡潔
            printf("\033[34m[HB]\033[0m Node %02d Status: GPS:%s | UART:%s\n",
                   msg->node_id,
                   msg->data.heartbeat.gps_fix_status ? "FIXED" : "SEARCH",
                   msg->data.heartbeat.uart_status ? "ON" : "OFF");
            break;

        case MSG_TYPE_UAV_EVENT:
            // 處理無人機進入或離開掃描範圍的事件
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

        // [RC 版新增] 攔截控制封包，顯示 Logger 自己發出的指令回饋
        case MSG_TYPE_CONTROL:
            if (msg->data.ctrl.command == CTRL_CMD_FORCE_SAVE) {
                printf("\n\033[1;33m[CTRL]\033[0m >>> MASTER COMMAND SENT: FORCE SAVE <<<\n\n");
            } else if (msg->data.ctrl.command == CTRL_CMD_RESET_NODE) {
                printf("\n\033[1;31m[CTRL]\033[0m >>> MASTER COMMAND SENT: SYSTEM RESET <<<\n\n");
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Dashboard 任務：持續從佇列中取出資料並列印
 */
static void dashboard_task(void *pvParameters) {
    QueueHandle_t rx_queue = (QueueHandle_t)pvParameters;
    espnow_payload_t msg;

    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            app_dashboard_print_event(&msg);
        }
    }
}

void app_dashboard_init(void) {
    printf("\n\033[1;35m===========================================\033[0m\n");
    printf("\033[1;35m   UAV Distributed Detection System RC    \033[0m\n");
    printf("\033[1;35m         Master Dashboard v2026           \033[0m\n");
    printf("\033[1;35m===========================================\033[0m\n\n");
}

void app_dashboard_start(QueueHandle_t rx_queue) {
    xTaskCreate(dashboard_task, "dash_task", 4096, (void *)rx_queue, 5, NULL);
}