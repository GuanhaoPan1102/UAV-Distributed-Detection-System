#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "dc_protocol.h"
#include "drv_ble_scan.h"
#include "drv_uart_tx.h"

static const char *TAG = "SCANNER_MAIN";

// --- 系統參數設定 ---
#define MAX_TRACKING_DRONES     5         // 同時追蹤的最大數量
#define CANDIDATE_THRESHOLD_US  (3000000) // 3 秒進入邏輯
#define TIMEOUT_THRESHOLD_US    (5000000) // 5 秒離開邏輯
#define PACKET_BUF_SIZE         100       // Candidate 期間緩衝深度

// UART 補償常數與心跳設定
#define UART_SERIAL_OFFSET_US   3000      // 115200bps 下傳輸 34bytes 約需 3ms
#define UART_IDLE_HEARTBEAT_US  (3000000) // 3 秒沒有發送任何資料就補發心跳

// --- 資料結構定義 ---
typedef enum {
    STATE_CANDIDATE,
    STATE_ACTIVE
} tracker_state_t;

typedef struct drone_session {
    uint8_t mac[6];
    tracker_state_t state;
    int64_t first_seen_us;
    int64_t last_seen_us;
    
    // Candidate 階段暫存陣列
    scanner_queue_item_t *buffer;
    uint16_t buffer_count;

    struct drone_session *next;
} drone_session_t;

static drone_session_t *session_list = NULL;
static QueueHandle_t scan_queue = NULL;

// 看門狗防護：記錄最後一次向 Slave 發送成功發送的時間
static int64_t last_uart_tx_time_us = 0; 

// --- 輔助函數 ---
drone_session_t* find_session(uint8_t *mac) {
    drone_session_t *curr = session_list;
    while (curr) {
        if (memcmp(curr->mac, mac, 6) == 0) return curr;
        curr = curr->next;
    }
    return NULL;
}

/**
 * @brief 封裝並發送 UART 封包 (支援 Burst 時間補償與 Heartbeat)
 */
void send_to_uart(scanner_queue_item_t *item, uint8_t event, int burst_idx) {
    uart_scanner_to_ttgo_t tx_packet = {0}; // 初始化清零
    
    memcpy(tx_packet.mac, item->mac, 6);
    tx_packet.event_flag = event;
    
    // 如果是 0xEE 心跳包，不含實質資料，不需要計算 age_us
    if (event != 0xEE) {
        uint32_t raw_age = (uint32_t)(esp_timer_get_time() - item->timestamp_us);
        tx_packet.age_us = raw_age + (burst_idx * UART_SERIAL_OFFSET_US);
        tx_packet.ble_data.rssi = item->rssi;
        memcpy(&tx_packet.ble_data.payload, &item->payload, sizeof(uav_ble_payload_t));
    }
    
    // 透過驅動發送 (底層 drv_uart_tx 已實作 Mutex，保證不會撞包)
    uart_tx_send_packet(&tx_packet);

    // 只要有發送，就更新最後發送時間，重置心跳計時器
    last_uart_tx_time_us = esp_timer_get_time();
}

// --- 任務：處理掃描隊列 (3秒進入邏輯) ---
void tracker_task(void *pvParameters) {
    scanner_queue_item_t rx_item;
    while (1) {
        if (xQueueReceive(scan_queue, &rx_item, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();
            drone_session_t *s = find_session(rx_item.mac);

            if (s == NULL) {
                // 新目標出現，建立 Session 進入觀察期
                ESP_LOGI(TAG, "New Candidate: %02X:%02X:%02X...", rx_item.mac[0], rx_item.mac[1], rx_item.mac[2]);
                s = malloc(sizeof(drone_session_t));
                memcpy(s->mac, rx_item.mac, 6);
                s->state = STATE_CANDIDATE;
                s->first_seen_us = now;
                s->last_seen_us = now;
                s->buffer = malloc(sizeof(scanner_queue_item_t) * PACKET_BUF_SIZE);
                s->buffer_count = 0;
                s->next = session_list;
                session_list = s;
            }

            s->last_seen_us = now;

            if (s->state == STATE_CANDIDATE) {
                // 暫存資料
                if (s->buffer_count < PACKET_BUF_SIZE) {
                    memcpy(&s->buffer[s->buffer_count++], &rx_item, sizeof(scanner_queue_item_t));
                }

                // 檢查是否達標 3 秒
                if (now - s->first_seen_us >= CANDIDATE_THRESHOLD_US) {
                    s->state = STATE_ACTIVE;
                    ESP_LOGI(TAG, "Target ACTIVE: %02X... Bursting %d pkts.", s->mac[0], s->buffer_count);
                    
                    // A. 發送「發現」旗標 (0x01)
                    send_to_uart(&s->buffer[0], 0x01, 0);

                    // B. 爆發傳輸暫存資料 (0x00)，附帶排隊延遲補償
                    for (int i = 0; i < s->buffer_count; i++) {
                        send_to_uart(&s->buffer[i], 0x00, i + 1); 
                    }
                    
                    free(s->buffer);
                    s->buffer = NULL;
                }
            } else {
                // 已穩定目標：即時轉發 (0x00)
                send_to_uart(&rx_item, 0x00, 0);
            }
        }
    }
}

// --- 任務：超時檢查與空閒心跳 (5秒離開邏輯) ---
void timeout_cleanup_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒執行一次
        int64_t now = esp_timer_get_time();
        drone_session_t **curr = &session_list;

        // 1. 檢查 5 秒遺失邏輯
        while (*curr) {
            drone_session_t *s = *curr;
            if (now - s->last_seen_us >= TIMEOUT_THRESHOLD_US) {
                ESP_LOGW(TAG, "Target LOST: %02X:%02X...", s->mac[0], s->mac[1]);
                
                // 發送「遺失」旗標封包給 Slave (0xFF)
                scanner_queue_item_t lost_item = {0};
                memcpy(lost_item.mac, s->mac, 6);
                lost_item.timestamp_us = now;
                send_to_uart(&lost_item, 0xFF, 0);

                if (s->buffer) free(s->buffer);
                *curr = s->next;
                free(s);
            } else {
                curr = &((*curr)->next);
            }
        }

        // 2. 空閒心跳邏輯：如果超過 3 秒沒發送任何資料，戳一下 Slave
        if (now - last_uart_tx_time_us >= UART_IDLE_HEARTBEAT_US) {
            scanner_queue_item_t hb_item = {0};
            memset(hb_item.mac, 0xFF, 6); // 填入 Dummy MAC 以防解析報錯
            send_to_uart(&hb_item, 0xEE, 0); // 0xEE 為心跳標籤
            ESP_LOGD(TAG, "Sent UART Idle Heartbeat (0xEE)");
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // 初始化 UART TX (注意腳位請對應您的 Scanner PCB)
    // 假設 UART2, TX:17, RX:16, RTS:18, CTS:19
    uart_tx_init(UART_NUM_2, 17, 16, 18, 19); 
    
    scan_queue = xQueueCreate(50, sizeof(scanner_queue_item_t));
    ble_scan_init(scan_queue);

    // 啟動核心任務
    xTaskCreate(tracker_task, "tracker_task", 4096, NULL, 10, NULL);
    xTaskCreate(timeout_cleanup_task, "cleanup_task", 2048, NULL, 5, NULL);
    
    // 初始化最後發送時間
    last_uart_tx_time_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Scanner system successfully started.");
}