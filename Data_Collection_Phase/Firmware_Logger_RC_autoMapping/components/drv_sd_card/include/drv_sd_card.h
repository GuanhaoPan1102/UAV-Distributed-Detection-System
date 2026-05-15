#ifndef DRV_SD_CARD_H
#define DRV_SD_CARD_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化並掛載 TTGO T8 板載 SD 卡
 * @param mount_point 掛載路徑 (建議使用 "/sdcard")
 * @return esp_err_t ESP_OK 代表掛載成功
 */
esp_err_t sd_card_init(const char* mount_point);

/**
 * @brief 檢查 SD 卡是否已正常掛載
 */
bool sd_card_is_mounted(void);

/**
 * @brief 獲取 SD 卡剩餘空間 (單位: MB)
 */
size_t sd_card_get_free_size_mb(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_SD_CARD_H