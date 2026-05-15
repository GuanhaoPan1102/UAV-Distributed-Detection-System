#ifndef APP_ROUTE_MAPPER_H
#define APP_ROUTE_MAPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "dc_protocol_rc.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─── 容量設定 ─────────────────────────────────────────────
#define ROUTE_MAPPER_MAX_NODES      8    // 最多支援幾個節點 (S1~S8)
#define ROUTE_MAPPER_MAX_FILES_PER_NODE 32  // 每個節點最多記錄幾個不重複檔案編號

// ─── 公開 API ─────────────────────────────────────────────

/**
 * @brief 初始化 Route Mapper 模組 (呼叫一次)
 */
void app_route_mapper_init(void);

/**
 * @brief 開始錄製一條新的 Route
 * @param route_id  路線編號 (例如輸入 "l2" 時傳入 2)
 * @return true  成功開始錄製
 * @return false 已在錄製中，拒絕重複開始
 */
bool app_route_mapper_start(uint8_t route_id);

/**
 * @brief 停止錄製並將 Mapping 結果寫入 SD 卡
 *        輸出格式（CSV，Tab 分隔）：
 *        Route\tS1_files\tS2_files\t...\tSN_files
 *        其中同一節點多個檔案以空格分隔，例如 "53 54"
 * @return true  成功寫入
 * @return false 目前未在錄製中
 */
bool app_route_mapper_stop(void);

/**
 * @brief 將一筆 FILE_SAVED 封包餵入 Mapper
 *        若目前正在錄製，自動去重後記錄
 * @param node_id   節點編號 (1-based，對應 S1, S2, ...)
 * @param filename  Slave 回報的完整路徑字串，例如 "/sdcard/data52.csv"
 */
void app_route_mapper_feed(uint8_t node_id, const char *filename);

/**
 * @brief 查詢目前是否正在錄製
 */
bool app_route_mapper_is_recording(void);

/**
 * @brief 取得目前正在錄製的 Route ID (未錄製時回傳 0)
 */
uint8_t app_route_mapper_current_route(void);

#ifdef __cplusplus
}
#endif

#endif // APP_ROUTE_MAPPER_H