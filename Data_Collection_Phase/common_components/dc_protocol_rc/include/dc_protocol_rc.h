#ifndef DC_PROTOCOL_RC_H
#define DC_PROTOCOL_RC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * 1. 系統通訊標頭與識別碼 (RC 遙控版)
 ******************************************************************************/
#define BLE_PACKET_IDENTIFIER   0xAA  
#define UART_HEADER_0           0xEB  
#define UART_HEADER_1           0x90
#define ESPNOW_WIFI_CHANNEL     1     // 系統統一使用的 Wi-Fi 頻道

/*******************************************************************************
 * 2. [第一段鏈路] UAV -> Scanner (BLE 廣播)
 * 這是無人機發出的原始數據包
 ******************************************************************************/
typedef struct __attribute__((packed)) {
    uint8_t identifier;         // 應為 0xAA
    uint8_t seq_num;            // 無人機發出的序號
    int32_t lat;                // 無人機 GPS 緯度 (Scaled by 10^7)
    int32_t lon;                // 無人機 GPS 經度 (Scaled by 10^7)
    int16_t alt;                // 無人機 GPS 高度 (m)
    int16_t spd;                // 無人機速度 (km/h)
    uint8_t checksum;           // 廣播校驗碼
} uav_ble_payload_t;

/*******************************************************************************
 * 3. [Scanner 內部緩衝] BLE Scan -> Tracker Task
 * 用於 Scanner 內部處理，計算資料年齡與 RSSI
 ******************************************************************************/
typedef struct {
    uint8_t   mac[6];           // 無人機 MAC 地址
    int8_t    rssi;             // 掃描到的訊號強度
    int64_t   timestamp_us;     // Scanner 收到資料的當下微秒
    uav_ble_payload_t payload;  // 原始廣播數據
} scanner_queue_item_t;

/*******************************************************************************
 * 4. [第二段鏈路] Scanner -> TTGO (UART 傳輸) 
 * 這是嵌套結構的核心：將 BLE 數據包封裝在 UART 協議中
 ******************************************************************************/
typedef struct __attribute__((packed)) {
    uav_ble_payload_t payload;  // 嵌套：無人機原始數據
    int8_t   rssi;              // 訊號強度
} ble_packet_queue_item_t;

typedef struct __attribute__((packed)) {
    uint8_t  header[2];         // 0xEB, 0x90
    uint8_t  mac[6];            // 無人機 MAC
    uint8_t  event_flag;        // 0x01: Discover, 0xFF: Lost, 0xEE: Heartbeat
    uint32_t age_us;            // 資料在 Scanner 停留的時間
    ble_packet_queue_item_t ble_data; // 嵌套內容
    uint8_t  uart_checksum;     // UART 鏈路校驗碼
} uart_scanner_to_ttgo_t;

/*******************************************************************************
 * 5. [第三段鏈路] TTGO Slave -> Master (ESP-NOW)
 * 用於回報節點狀態與追蹤事件給 Logger
 ******************************************************************************/
typedef enum {
    MSG_TYPE_REGISTER   = 0,    // 節點註冊 (上電後定位完成時)
    MSG_TYPE_FILE_SAVED = 1,    // SD 卡檔案存檔完成回報
    MSG_TYPE_BLE_DATA   = 2,    // 即時數據轉發 (若需要監控實時軌跡)
    MSG_TYPE_HEARTBEAT  = 3,    // 節點健康度回報
    MSG_TYPE_UAV_EVENT  = 4,    // 發現/遺失無人機事件
    MSG_TYPE_CONTROL    = 5     // [RC 版新增] 遠端控制指令
} espnow_msg_type_t;

// [RC 版新增] 控制指令定義
typedef enum {
    CTRL_CMD_FORCE_SAVE = 0x01, // 強制存檔
    CTRL_CMD_RESET_NODE = 0x02  // 重置節點
} control_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;          // 訊息類型 (espnow_msg_type_t)
    uint8_t  node_id;           // Slave 節點 ID
    union {
        // 註冊包：回報 Slave 自己的位置
        struct __attribute__((packed)) {
            int32_t lat; int32_t lon; float alt; int64_t timestamp;
        } reg;
        
        // 存檔回報：回報剛關閉的 CSV 檔名
        struct __attribute__((packed)) {
            int64_t timestamp; char filename[16];  
        } file;
        
        // 即時資料：轉發 Scanner 嵌套數據
        struct __attribute__((packed)) {
            int64_t timestamp_us; ble_packet_queue_item_t ble_data; 
        } live_data;
        
        // 心跳包：回報硬體狀態
        struct __attribute__((packed)) {
            uint8_t uart_status; uint8_t gps_fix_status; 
        } heartbeat;
        
        // 事件包：發現或遺失無人機
        struct __attribute__((packed)) {
            int64_t timestamp_us; uint8_t uav_id[6]; uint8_t event_type;       
        } uav_event;

        // [RC 版新增] 遠端控制資料段
        struct __attribute__((packed)) {
            uint8_t  command;       // control_cmd_t
            uint32_t timestamp;     // Logger 發出的時間戳 
            uint32_t extra_info;    // 預留資訊 
        } ctrl;

    } data;
} espnow_payload_t;

#ifdef __cplusplus
}
#endif

#endif // DC_PROTOCOL_RC_H