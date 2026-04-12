#ifndef DRV_GPS_ATGM336H_H
#define DRV_GPS_ATGM336H_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief GPS 完整狀態結構體
 */
typedef struct {
    int32_t  latitude;      // 緯度 (Scaled by 10^7)
    int32_t  longitude;     // 經度 (Scaled by 10^7)
    float    altitude;      // 高度 (m)
    bool     is_fixed;      // 是否取得衛星定位 (Fix)
    bool     is_time_synced;// 系統時間是否「曾與」衛星同步
    bool     pps_is_valid;  // PPS 脈衝目前是否穩定 (Watchdog)
} gps_atgm_status_t;

/**
 * @brief 初始化 ATGM336H 驅動
 * @param uart_num 使用的 UART 編號 (例: UART_NUM_1)
 * @param tx_pin TX 腳位
 * @param rx_pin RX 腳位
 * @param pps_pin PPS 信號腳位
 */
void gps_atgm_init(int uart_num, int tx_pin, int rx_pin, int pps_pin);

/**
 * @brief 獲取目前的 GPS 完整狀態
 */
gps_atgm_status_t gps_atgm_get_status(void);

#endif // DRV_GPS_ATGM336H_H