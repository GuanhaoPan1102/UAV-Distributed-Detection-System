#ifndef DRV_GPS_NEO6M_H
#define DRV_GPS_NEO6M_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- UAV 專用 GPS 資料結構 ---
typedef struct __attribute__((packed)) {
    int32_t latitude_scaled;  // 緯度: (度 + 分/60) * 10^7
    int32_t longitude_scaled; // 經度: (度 + 分/60) * 10^7
    int16_t altitude_m;       // 高度: 公尺
    int16_t speed_kph_scaled; // 速度: km/h * 100
    uint8_t is_valid;         // 定位是否有效 (1: Valid, 0: Invalid)
} gps_data_t;

/**
 * @brief 初始化 NEO-6M GPS 模組與 UART 監聽任務
 */
void gnss_init(void);

/**
 * @brief 取得最新的 GPS 解析數據 (具備執行緒安全)
 * @return gps_data_t 最新的一筆 GPS 結構體
 */
gps_data_t gnss_get_data(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_GPS_NEO6M_H