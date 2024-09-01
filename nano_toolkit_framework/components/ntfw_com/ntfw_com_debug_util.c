/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :debug utility functions source file
 *
 * CREATED:2019/09/15 00:11:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:デバッグユーティリティ関係の関数群
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:
 *
 *******************************************************************************
 *
 * Copyright (c) 2024 Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_com_debug_util.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ntfw_com_value_util.h"
#include "ntfw_com_mem_alloc.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ログ出力タグ */
static const char* LOG_TAG = "Debug";

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** 動的メモリ確保のエラー情報表示用のフック関数 */
static void v_disp_alloc_faild_hook(size_t size, uint32_t caps, const char *function_name);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
/*******************************************************************************
 *
 * NAME: b_dbg_valid_date
 *
 * DESCRIPTION:日付判定チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_date        s_date          R   判定対象
 *   int            i_year          R   判定年
 *   int            i_month         R   判定月
 *   int            i_day           R   判定日
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_dbg_valid_date(ts_date_t s_date, const int i_year, const int i_month, const int i_day) {
    return (s_date.i_year == i_year && s_date.i_month == i_month && s_date.i_day == i_day);
}

/*******************************************************************************
 *
 * NAME: b_dbg_disp_open_file
 *
 * DESCRIPTION:ファイルオープンチェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルもしくはディレクトリのパス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_dbg_disp_open_file(const char* pc_path) {
    // パラメータチェック
    if (pc_path == NULL) {
        ESP_LOGE(LOG_TAG, "File Path NULL");
        return false;
    }
    FILE* f = fopen(pc_path, "r");
    if (f == NULL) {
        ESP_LOGE(LOG_TAG, "File None Path:%s", pc_path);
        return false;
    }
    // ファイルオープンに失敗した場合
    ESP_LOGI(LOG_TAG, "File Open Path:%s", pc_path);
    fclose(f);
    return true;
}

/*******************************************************************************
 *
 * NAME: v_dbg_register_failed_alloc
 *
 * DESCRIPTION:メモリ確保エラーの情報表示有効化
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_register_failed_alloc() {
    heap_caps_register_failed_alloc_callback(v_disp_alloc_faild_hook);
}

/*******************************************************************************
 *
 * NAME: v_dbg_disp_heap_info
 *
 * DESCRIPTION:ヒープメモリ情報の表示
 *
 * PARAMETERS:      Name        RW  Usage
 * const char*      pc_pref     R   プレフィックス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_disp_heap_info(const char* pc_pref) {
    // 入力チェック
    if (pc_pref == NULL) {
        // 文字列表示
        ESP_LOGI(LOG_TAG, "Prefix Not Found");
        return;
    }
    ESP_LOGI(LOG_TAG, "%s //==========================================================================", pc_pref);
    ESP_LOGI(LOG_TAG, "%s // Heap Memory Test", pc_pref);
    ESP_LOGI(LOG_TAG, "%s //==========================================================================", pc_pref);
    ESP_LOGI(LOG_TAG, "%s esp_get_free_heap_size()                              : %6lu", pc_pref, esp_get_free_heap_size() );
    ESP_LOGI(LOG_TAG, "%s esp_get_minimum_free_heap_size()                      : %6lu", pc_pref, esp_get_minimum_free_heap_size() );
    ESP_LOGI(LOG_TAG, "%s xPortGetFreeHeapSize()                                : %6d", pc_pref, xPortGetFreeHeapSize() );
    ESP_LOGI(LOG_TAG, "%s xPortGetMinimumEverFreeHeapSize()                     : %6d", pc_pref, xPortGetMinimumEverFreeHeapSize() );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_EXEC)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_EXEC) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_32BIT)             : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_32BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_8BIT)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_8BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_DMA)               : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_DMA) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID2)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID2) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID3)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID3)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID4) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID4)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID5) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID5)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID6) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID6)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID7) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_PID7)              : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_SPIRAM)            : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_SPIRAM) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_INTERNAL)          : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_INTERNAL) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_DEFAULT)           : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_DEFAULT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_free_size(MALLOC_CAP_INVALID)           : %6d", pc_pref, heap_caps_get_free_size(MALLOC_CAP_INVALID) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_EXEC)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_EXEC) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_32BIT)    : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_DMA)      : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_DMA) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID2)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID2) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID3)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID3)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID4) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID4)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID5) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID5)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID6) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID6)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID7) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_PID7)     : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)   : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)  : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_largest_free_block(MALLOC_CAP_INVALID)  : %6d", pc_pref, heap_caps_get_largest_free_block(MALLOC_CAP_INVALID) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_EXEC)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_EXEC) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT)     : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_DMA)       : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_DMA) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID2)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID2) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID3)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID3)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID4) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID4)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID5) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID5)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID6) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID6)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID7) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_PID7)      : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_PID3) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)    : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)  : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)   : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT) );
    ESP_LOGI(LOG_TAG, "%s heap_caps_get_minimum_free_size(MALLOC_CAP_INVALID)   : %6d", pc_pref, heap_caps_get_minimum_free_size(MALLOC_CAP_INVALID) );
}

/*******************************************************************************
 *
 * NAME: v_dbg_disp_stack_info
 *
 * DESCRIPTION:スタック情報表示
 *
 * PARAMETERS:      Name        RW  Usage
 * char*            pc_pref     R   プレフィックス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_disp_stack_info(const char* pc_pref) {
	UBaseType_t t_type = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(LOG_TAG, "%s stack_high_water_mark=%d", pc_pref, t_type);
}

/*******************************************************************************
 *
 * NAME: v_dbg_disp_hex_data
 *
 * DESCRIPTION:データの１６進数表示
 *
 * PARAMETERS:      Name        RW  Usage
 * const char*      pc_pref     R   プレフィックス
 * const uint8_t*   pu8_data    R   対象データ配列
 * uint32_t         u32_len     R   対象データ長
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_disp_hex_data(const char* pc_pref, const uint8_t* pu8_data, uint32_t u32_len) {
    // 入力チェック
    if (pc_pref == NULL || pu8_data == NULL) {
        // 文字列表示
        ESP_LOGI(LOG_TAG, "HEX Data Not Found");
        return;
    }
    // データ文字列
    static const char* pc_hex = "0123456789ABCDEF";
    char c_data[(u32_len * 2) + 1];
    // 文字列編集
    uint8_t u8_data;
    uint32_t u32_pos = 0;
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        u8_data = pu8_data[u32_idx];
        c_data[u32_pos]     = pc_hex[u8_data / 16];
        c_data[u32_pos + 1] = pc_hex[u8_data % 16];
        u32_pos += 2;
    }
    c_data[u32_pos] = '\0';
    // 文字列表示
    ESP_LOGI(LOG_TAG, "%s%s", pc_pref, c_data);
}

/*******************************************************************************
 *
 * NAME: v_dbg_file_list
 *
 * DESCRIPTION:ファイルリスト表示
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルもしくはディレクトリのパス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_file_list(char* pc_path) {
    // パラメータチェック
    if (pc_path == NULL) {
        ESP_LOGE(LOG_TAG, "File Path NULL");
        return;
    }
    // ディレクトリリスト
    struct dirent *ent;
    DIR *p_dir = opendir(pc_path);
    if (p_dir == NULL) {
        ESP_LOGE(LOG_TAG, "Path Error %s", pc_path);
        return;
    }
    while ((ent = readdir(p_dir)) != NULL) {
        if (ent->d_type == DT_REG) {
            ESP_LOGI(LOG_TAG, "File:%s/%s", pc_path, ent->d_name);
        } else if (ent->d_type == DT_DIR) {
            ESP_LOGI(LOG_TAG, "Dir :%s/%s", pc_path, ent->d_name);
        }
    }
    closedir(p_dir);
}

/*******************************************************************************
 *
 * NAME: v_dbg_file_info
 *
 * DESCRIPTION:ファイル情報
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルもしくはディレクトリのパス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_dbg_file_info(const char* pc_path) {
    // パラメータチェック
    if (pc_path == NULL) {
        ESP_LOGE(LOG_TAG, "File Path NULL");
        return;
    }
    struct stat statBuf;
    if (stat(pc_path, &statBuf) != 0) {
        return;
    }
    ESP_LOGI(LOG_TAG, "//**********************************************************");
    ESP_LOGI(LOG_TAG, "File Path :%s", pc_path);
    ESP_LOGI(LOG_TAG, "File Size :%ld", statBuf.st_size);
    ESP_LOGI(LOG_TAG, "File User :%04x", statBuf.st_uid);
    ESP_LOGI(LOG_TAG, "File Group:%o", statBuf.st_gid);
    ESP_LOGI(LOG_TAG, "File Mode :%lu", statBuf.st_mode);
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/
/** 動的メモリ確保のエラー情報表示用のフック関数 */
static void v_disp_alloc_faild_hook(size_t size, uint32_t caps, const char *function_name) {
    ESP_LOGE(LOG_TAG, "// Allocation Error func=%s size=%d caps=%lu", function_name, size, caps);
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
