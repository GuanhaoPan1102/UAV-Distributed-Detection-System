#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "app_route_mapper.h"

static const char *TAG = "ROUTE_MAPPER";

// ─── 內部資料結構 ──────────────────────────────────────────

/**
 * @brief 單一節點的錄製狀態
 */
typedef struct {
    uint32_t file_ids[ROUTE_MAPPER_MAX_FILES_PER_NODE]; // 去重後的檔案編號（數字部分）
    uint8_t  file_count;                                // 已記錄的不重複檔案數量
} node_record_t;

/**
 * @brief 全域錄製狀態
 */
static struct {
    bool          is_recording;
    uint8_t       route_id;
    node_record_t nodes[ROUTE_MAPPER_MAX_NODES]; // index 0 = Node 1 (S1)
} s_mapper = {0};

// ─── 內部輔助函數 ──────────────────────────────────────────

/**
 * @brief 從路徑字串中萃取數字編號
 *        例如 "/sdcard/data52.csv" → 52
 *             "/sdcard/data52"     → 52
 *        若無法解析則回傳 UINT32_MAX
 */
static uint32_t _extract_file_id(const char *filename) {
    if (filename == NULL) return UINT32_MAX;

    // 找到最後一個 '/' 或從頭開始
    const char *base = strrchr(filename, '/');
    base = (base != NULL) ? base + 1 : filename;

    // 跳過非數字前綴（例如 "data"）
    while (*base && (*base < '0' || *base > '9')) {
        base++;
    }

    if (*base == '\0') return UINT32_MAX;

    return (uint32_t)strtoul(base, NULL, 10);
}

/**
 * @brief 檢查某節點是否已記錄過該 file_id（去重用）
 */
static bool _is_duplicate(node_record_t *node, uint32_t file_id) {
    for (uint8_t i = 0; i < node->file_count; i++) {
        if (node->file_ids[i] == file_id) return true;
    }
    return false;
}

/**
 * @brief 將本次錄製結果寫入 SD 卡
 *        檔案路徑：/sdcard/route_mapping.csv
 *        格式（Tab 分隔）：
 *          Route\tS1\tS2\tS3\tS4\n
 *          2\t52\t50\t53 54\t52 53\n
 *
 *        多個檔案編號以空格連接；節點無資料時填 "-"
 */
static void _write_mapping_to_sdcard(void) {
    const char *map_path = "/sdcard/route_mapping.csv";

    // 以追加模式開啟，保留歷史資料
    FILE *fp = fopen(map_path, "a");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for writing", map_path);
        return;
    }

    // ── 找出本次有資料的最大節點編號，決定欄位寬度 ──
    uint8_t max_node_used = 0;
    for (uint8_t n = 0; n < ROUTE_MAPPER_MAX_NODES; n++) {
        if (s_mapper.nodes[n].file_count > 0) {
            max_node_used = n + 1; // 1-based
        }
    }
    if (max_node_used == 0) max_node_used = 1; // 至少輸出一欄

    // ── 寫入一行資料 ──
    fprintf(fp, "%u", s_mapper.route_id);

    for (uint8_t n = 0; n < max_node_used; n++) {
        fprintf(fp, "\t");
        node_record_t *nd = &s_mapper.nodes[n];
        if (nd->file_count == 0) {
            fprintf(fp, "-");
        } else {
            for (uint8_t f = 0; f < nd->file_count; f++) {
                if (f > 0) fprintf(fp, " ");
                fprintf(fp, "%lu", (unsigned long)nd->file_ids[f]);
            }
        }
    }
    fprintf(fp, "\n");
    fclose(fp);

    ESP_LOGI(TAG, "Mapping saved to %s", map_path);
}

// ─── 公開 API 實作 ─────────────────────────────────────────

void app_route_mapper_init(void) {
    memset(&s_mapper, 0, sizeof(s_mapper));
    ESP_LOGI(TAG, "Route Mapper initialized.");
}

bool app_route_mapper_start(uint8_t route_id) {
    if (s_mapper.is_recording) {
        printf("\n\033[1;33m[MAPPER]\033[0m Already recording Route \033[1;33m%d\033[0m. "
               "Press [l] to stop it first.\n\n", s_mapper.route_id);
        return false;
    }
    memset(&s_mapper, 0, sizeof(s_mapper));
    s_mapper.is_recording = true;
    s_mapper.route_id     = route_id;

    printf("\n\033[1;32m[MAPPER]\033[0m ╔══════════════════════════════════════╗\n");
    printf("\033[1;32m[MAPPER]\033[0m   Input received  : \033[1;36ml%d\033[0m\n", route_id);
    printf("\033[1;32m[MAPPER]\033[0m   Route ID parsed : \033[1;33m%d\033[0m\n", route_id);
    printf("\033[1;32m[MAPPER]\033[0m   Status          : \033[1;32mRECORDING\033[0m\n");
    printf("\033[1;32m[MAPPER]\033[0m   Press [l] to stop and save mapping.\n");
    printf("\033[1;32m[MAPPER]\033[0m ╚══════════════════════════════════════╝\n\n");
    return true;
}

bool app_route_mapper_stop(void) {
    if (!s_mapper.is_recording) {
        ESP_LOGW(TAG, "Not currently recording.");
        return false;
    }

    s_mapper.is_recording = false;

    // ── 終端機摘要輸出 ──
    printf("\n\033[1;32m[MAPPER]\033[0m >>> Recording stopped for Route %d. Summary:\n",
           s_mapper.route_id);
    printf("  %-8s", "Route");
    for (uint8_t n = 0; n < ROUTE_MAPPER_MAX_NODES; n++) {
        if (s_mapper.nodes[n].file_count > 0) {
            printf("  S%-7d", n + 1);
        }
    }
    printf("\n  %-8u", s_mapper.route_id);

    for (uint8_t n = 0; n < ROUTE_MAPPER_MAX_NODES; n++) {
        node_record_t *nd = &s_mapper.nodes[n];
        if (nd->file_count == 0) continue;

        printf("  ");
        for (uint8_t f = 0; f < nd->file_count; f++) {
            if (f > 0) printf(",");
            printf("%lu", (unsigned long)nd->file_ids[f]);
        }
        // 對齊補空格（簡易排版）
        int pad = 8 - (int)(nd->file_count * 3);
        for (int p = 0; p < pad && p >= 0; p++) printf(" ");
    }
    printf("\n\n");

    // ── 寫入 SD 卡 ──
    _write_mapping_to_sdcard();

    printf("\033[1;32m[MAPPER]\033[0m Mapping written to /sdcard/route_mapping.csv\n\n");
    return true;
}

void app_route_mapper_feed(uint8_t node_id, const char *filename) {
    if (!s_mapper.is_recording) return;
    if (node_id == 0 || node_id > ROUTE_MAPPER_MAX_NODES) {
        ESP_LOGW(TAG, "Node ID %d out of range (1~%d), ignored.", node_id, ROUTE_MAPPER_MAX_NODES);
        return;
    }

    uint32_t file_id = _extract_file_id(filename);
    if (file_id == UINT32_MAX) {
        ESP_LOGW(TAG, "Cannot parse file ID from: %s", filename);
        return;
    }

    node_record_t *nd = &s_mapper.nodes[node_id - 1]; // 轉 0-based

    if (_is_duplicate(nd, file_id)) {
        // 已記錄過，靜默忽略（廣播冗餘封包造成）
        return;
    }

    if (nd->file_count >= ROUTE_MAPPER_MAX_FILES_PER_NODE) {
        ESP_LOGW(TAG, "Node %d file record full (%d/%d), dropped file_id %lu",
                 node_id, nd->file_count, ROUTE_MAPPER_MAX_FILES_PER_NODE,
                 (unsigned long)file_id);
        return;
    }

    nd->file_ids[nd->file_count++] = file_id;
    ESP_LOGD(TAG, "Route %d | S%d | file_id=%lu (total=%d)",
             s_mapper.route_id, node_id, (unsigned long)file_id, nd->file_count);
}

bool app_route_mapper_is_recording(void) {
    return s_mapper.is_recording;
}

uint8_t app_route_mapper_current_route(void) {
    return s_mapper.is_recording ? s_mapper.route_id : 0;
}