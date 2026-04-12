#ifndef DRV_BLE_ADV_H
#define DRV_BLE_ADV_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 BLE 廣播模組，並啟動 10Hz 的廣播更新任務
 */
void ble_adv_init(void);

#ifdef __cplusplus
}
#endif

#endif // DRV_BLE_ADV_H