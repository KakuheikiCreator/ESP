/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common memory allocation functions source file
 *
 * CREATED:2022/03/26 00:00:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:動的メモリ割り当て関係の関数群
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:
 *
 *******************************************************************************
 *
 * Copyright (c) 2024 Kakuheiki.Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_com_mem_alloc.h"

#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** ログ接頭辞 */
#define LOG_TAG "COM_MEM_ALLOC"

/** Block Time */
#ifndef MEM_ALLOC_BLOCK_TIME
    // デフォルト値は無制限にウェイト
    #define MEM_ALLOC_BLOCK_TIME    (portMAX_DELAY)
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * メモリ領域情報
 */
typedef struct s_mem_area_info_t {
    uint8_t* pu8_address;                       // アドレス
    uint32_t u32_size;                          // サイズ
    struct s_mem_area_info_t* ps_addr_prev;     // 前の空き領域情報（アドレス昇順）
    struct s_mem_area_info_t* ps_addr_next;     // 次の空き領域情報（アドレス昇順）
    struct s_mem_area_info_t* ps_size_prev;     // 前の空き領域情報（サイズ昇順）
    struct s_mem_area_info_t* ps_size_next;     // 次の空き領域情報（サイズ昇順）
} ts_mem_area_info_t;

//==============================================================================
// クリティカルセクション関係
//==============================================================================
/** mutex init function */
typedef void (*tf_initialize)();

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

//==============================================================================
// メモリストレージ
//==============================================================================
/** メモリストレージ */
static uint8_t* pu8_mem_storage = NULL;

//==============================================================================
// 領域情報
//==============================================================================
/** 領域情報の先頭 */
static ts_mem_area_info_t* ps_info_list = NULL;
/** 領域情報の最終インデックス */
static uint32_t u32_info_last_idx = 0;

//==============================================================================
// 空き領域の情報
//==============================================================================
/** 空き領域情報（アドレス昇順）の先頭 */
static ts_mem_area_info_t* ps_addr_top  = NULL;
/** 空き領域情報（アドレス昇順）の末尾 */
static ts_mem_area_info_t* ps_addr_tail = NULL;
/** 空き領域情報（サイズ昇順）の先頭 */
static ts_mem_area_info_t* ps_size_top  = NULL;
/** 空き領域情報（サイズ昇順）の末尾 */
static ts_mem_area_info_t* ps_size_tail = NULL;
/** 空き領域の個数 */
static uint32_t u32_mem_free_cnt = 1;

//==============================================================================
// 未使用の領域情報
//==============================================================================
/** 再利用対象となる、使われていない領域情報（アドレス昇順）の先頭 */
static ts_mem_area_info_t* ps_unused_top = NULL;

//==============================================================================
// 関連情報
//==============================================================================
/** メモリ割り当て領域アドレス（先端） */
static uint8_t* pu8_mem_usage_top = NULL;
/** メモリ割り当て領域アドレス（末尾） */
static uint8_t* pu8_mem_usage_tail = NULL;
/** 使用領域のサイズ（空き領域情報除く） */
static uint32_t u32_mem_val_usage_size = 0;
/** ユーザーへの割り当て済みサイズ */
static uint32_t u32_mem_val_alloc_size = 0;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
//==============================================================================
// 初期処理
//==============================================================================
/** Initial processing */
static void v_initialize();
/** Dummy initial processing */
static void v_initialize_dummy();
/** Initialize function */
static tf_initialize pf_initialize = v_initialize;
/** Mutex Initial processing */
static void v_mutex_init();
/** Dummy Mutex initial processing */
static void v_mutex_init_dummy();
/** Initialize function */
static tf_initialize pf_mutex_init = v_mutex_init;

//==============================================================================
// 領域の確保と解放
//==============================================================================
/** 空き領域からメモリ確保処理 */
static uint8_t* pu8_mem_alloc(uint32_t u32_size);
/** メモリ領域の再定義処理 */
static uint8_t* pu8_mem_realloc(void* pv_memory, uint32_t u32_size);
/** メモリ解放処理 */
static long l_mem_free_memory(void* pv_memory);

//==============================================================================
// 領域情報の検索関数
//==============================================================================
/** 直前のアドレスの領域情報を検索 */
static ts_mem_area_info_t* ps_search_prev_addr_area(uint8_t* pu8_pointer);
/** 直前のサイズの領域情報を検索 */
static ts_mem_area_info_t* ps_search_prev_size_area(uint32_t u32_size);
/** サイズが確保出来る最小の領域情報を検索 */
static ts_mem_area_info_t* ps_search_alloc_area(uint32_t u32_size);

//==============================================================================
// 領域情報の操作関数
//==============================================================================
/** 領域情報のサイズ更新処理 */
static void v_upd_area_size(ts_mem_area_info_t* ps_target, uint32_t u32_new_size);
/** 空き領域情報を追加 */
static uint32_t u32_add_list(uint8_t* pu8_area, uint32_t u32_upd_size);
/** 領域情報を未使用リストへ追加 */
static void v_add_unused_list(ts_mem_area_info_t* ps_info);
/** リストから領域情報を取り外す（アドレスのリンクリスト） */
static void v_remove_area_info_addr(ts_mem_area_info_t* ps_info);
/** リストから領域情報を取り外す（サイズのリンクリスト） */
static void v_remove_area_info_size(ts_mem_area_info_t* ps_info);
/** 未使用の領域情報を取得 */
static ts_mem_area_info_t* ps_get_unused_info();

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: u32_mem_alloc_size
 *
 * DESCRIPTION:allocate area size
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 * uint32_t:割り当て済みのメモリサイズ
 *
 ******************************************************************************/
uint32_t u32_mem_alloc_size() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return 0;
    }

    //==========================================================================
    // 割り当て済みのメモリサイズを取得
    //==========================================================================
    uint32_t u32_alloc_size = u32_mem_val_alloc_size;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return u32_alloc_size;
}

/*******************************************************************************
 *
 * NAME: u32_mem_usage_size
 *
 * DESCRIPTION:Usage area size
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 * uint32_t:内部の利用領域（空き領域情報除く）
 *
 ******************************************************************************/
uint32_t u32_mem_usage_size() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return 0;
    }

    //==========================================================================
    // 内部の利用領域のメモリサイズを取得
    //==========================================================================
    uint32_t u32_usage_size = u32_mem_val_usage_size;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return u32_usage_size;
}

/*******************************************************************************
 *
 * NAME: u32_mem_unused_size
 *
 * DESCRIPTION:Usage area size
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 * uint32_t:内部の空き領域（管理情報領域除く）
 *
 ******************************************************************************/
uint32_t u32_mem_unused_size() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return 0;
    }

    //==========================================================================
    // 内部の空き領域（管理情報領域除く）のサイズ算出
    //==========================================================================
    // 領域情報のサイズ
    uint32_t u32_area_info_size = sizeof(ts_mem_area_info_t) * u32_mem_free_cnt;
    // 未使用領域のサイズ
    uint32_t u32_unused_size = MEM_STORAGE_SIZE - u32_area_info_size - u32_mem_val_usage_size;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return u32_unused_size;
}

/*******************************************************************************
 *
 * NAME: u32_mem_unused_cnt
 *
 * DESCRIPTION:Number of memory segments
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 * uint32_t:空き領域の個数
 *
 ******************************************************************************/
uint32_t u32_mem_unused_cnt() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return 0;
    }

    //==========================================================================
    // 空き領域の個数を取得
    //==========================================================================
    uint32_t u32_free_cnt = u32_mem_free_cnt;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return u32_free_cnt;
}

/*******************************************************************************
 *
 * NAME: s_mem_unused_info_addr
 *
 * DESCRIPTION:Memory segment information
 *
 * PARAMETERS:      Name        RW  Usage
 * uint32_t         u32_idx     R   空き領域情報のインデックス
 *
 * RETURNS:
 * ts_mem_segment_info_t:空き領域情報
 *
 ******************************************************************************/
ts_mem_segment_info_t s_mem_unused_info_addr(uint32_t u32_idx) {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // 空き領域情報の生成
    //==========================================================================
    ts_mem_segment_info_t s_info;
    s_info.pu8_address = NULL;
    s_info.u32_size    = 0;

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return s_info;
    }

    //==========================================================================
    // 空き領域情報の編集
    //==========================================================================
    if (u32_idx < u32_mem_free_cnt) {
        ts_mem_area_info_t* ps_info = ps_addr_top;
        uint32_t u32_chk_idx;
        for (u32_chk_idx = 0; ps_info != NULL; u32_chk_idx++) {
            if (u32_chk_idx == u32_idx) {
                // アドレス
                s_info.pu8_address = ps_info->pu8_address;
                // サイズ
                s_info.u32_size    = ps_info->u32_size;
                break;
            }
            ps_info = ps_info->ps_addr_next;
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return s_info;
}

/*******************************************************************************
 *
 * NAME: s_mem_unused_info_size
 *
 * DESCRIPTION:Memory segment information
 *
 * PARAMETERS:      Name        RW  Usage
 * uint32_t         u32_idx     R   空き領域情報のインデックス
 *
 * RETURNS:
 * ts_mem_segment_info_t:空き領域情報
 *
 ******************************************************************************/
ts_mem_segment_info_t s_mem_unused_info_size(uint32_t u32_idx) {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==========================================================================
    // 空き領域情報の生成
    //==========================================================================
    ts_mem_segment_info_t s_info;
    s_info.pu8_address = NULL;
    s_info.u32_size    = 0;

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return s_info;
    }

    //==========================================================================
    // 空き領域情報の編集
    //==========================================================================
    if (u32_idx < u32_mem_free_cnt) {
        ts_mem_area_info_t* ps_info = ps_size_top;
        uint32_t u32_chk_idx;
        for (u32_chk_idx = 0; ps_info != NULL; u32_chk_idx++) {
            if (u32_chk_idx == u32_idx) {
                // アドレス
                s_info.pu8_address = ps_info->pu8_address;
                // サイズ
                s_info.u32_size    = ps_info->u32_size;
                break;
            }
            ps_info = ps_info->ps_size_next;
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return s_info;
}

/*******************************************************************************
 *
 * NAME: pv_mem_malloc
 *
 * DESCRIPTION:Dynamic Memory Assignment with Exclusive Control
 *
 * PARAMETERS:      Name        RW  Usage
 * uint32_t         u32_size    R   確保するサイズ
 *
 * RETURNS:
 * void*:確保したメモリ領域へのポインタ
 *
 ******************************************************************************/
void* pv_mem_malloc(uint32_t u32_size) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    pf_initialize();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // メモリ領域の確保処理
    //==========================================================================
    uint8_t* pu8_memory = pu8_mem_alloc(u32_size);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return (void*)pu8_memory;
}

/*******************************************************************************
 *
 * NAME: pv_mem_calloc
 *
 * DESCRIPTION:Dynamic Clear Memory Assignment with Exclusive Control
 *
 * PARAMETERS:      Name        RW  Usage
 * uint32_t         u32_size    R   確保するサイズ
 *
 * RETURNS:
 * void*:確保したメモリ領域へのポインタ
 *
 ******************************************************************************/
void* pv_mem_calloc(uint32_t u32_size) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    pf_initialize();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // メモリ領域の確保処理
    //==========================================================================
    uint8_t* pu8_memory = pu8_mem_alloc(u32_size);
    if (pu8_memory != NULL) {
        // メモリをクリア
        memset(pu8_memory, 0x00, u32_size);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return (void*)pu8_memory;
}

/*******************************************************************************
 *
 * NAME: pv_mem_realloc
 *
 * DESCRIPTION:Dynamic Memory reassignment with Exclusive Control
 *
 * PARAMETERS:      Name        RW  Usage
 * void*            pv_memory   R   再定義の対象メモリ領域へのポインタ
 * uint32_t         u32_size    R   確保するサイズ
 *
 * RETURNS:
 * void*:再定義したメモリ領域へのポインタ
 *
 ******************************************************************************/
void* pv_mem_realloc(void* pv_memory, uint32_t u32_size) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    pf_initialize();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // パラメータによる処理
    //==========================================================================
    uint8_t* pu8_memory = pu8_mem_realloc(pv_memory, u32_size);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return (void*)pu8_memory;
}

/*******************************************************************************
 *
 * NAME: pv_mem_clone
 *
 * DESCRIPTION:動的に確保されたバイト配列のクローン生成
 *
 * PARAMETERS:      Name        RW  Usage
 *   void*          pv_data     R   クローン対象のデータ
 *   uint32_t       u32_size    R   クローン対象のデータサイズ
 *
 * RETURNS:
 *   void*:動的にメモリ確保されたコピーデータへのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
void* pv_mem_clone(void* pv_data, uint32_t u32_size) {
    uint8_t* pu8_copy = (uint8_t*)pv_mem_malloc(u32_size);
    memcpy(pu8_copy, pv_data, u32_size);
    return pu8_copy;
}

/*******************************************************************************
 *
 * NAME: l_mem_free
 *
 * DESCRIPTION:Release dynamic memory with exclusive control
 *
 * PARAMETERS:      Name        RW  Usage
 * void*            pv_memory   R   解放する対象のメモリへのポインタ
 *
 * RETURNS:
 * uint32_t:解放されたメモリサイズ、解放不能の場合は-1
 *
 ******************************************************************************/
long l_mem_free(void* pv_memory) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    pf_initialize();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return false;
    }

    //==========================================================================
    // メモリ領域の解放処理
    //==========================================================================
    long l_free_size = l_mem_free_memory(pv_memory);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return l_free_size;
}

/*******************************************************************************
 *
 * NAME: l_mem_sizeof
 *
 * DESCRIPTION:Get dynamically allocated memory size
 *
 * PARAMETERS:      Name        RW  Usage
 * void*            pv_memory   R   対象のメモリへのポインタ
 *
 * RETURNS:
 * long:確保されたメモリサイズ、確保された領域で無い場合は-1
 *
 ******************************************************************************/
long l_mem_sizeof(void* pv_memory) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    pf_initialize();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, MEM_ALLOC_BLOCK_TIME) == pdFALSE) {
        return false;
    }

    //==========================================================================
    // 動的に確保したメモリサイズの取得
    //==========================================================================
    long l_alloc_size = -1;

    do {
        // 入力チェック
        if (pv_memory == NULL) {
            break;
        }
        // 先頭アドレスを算出
        uint8_t* pu8_target = (uint8_t*)(pv_memory - sizeof(uint32_t));
        // 入力チェック
        if (pu8_target < pu8_mem_usage_top || pu8_target > pu8_mem_usage_tail) {
            // 確保されたメモリ領域では無い場合
            break;
        }
        // アドレスの算出
        l_alloc_size = *((uint32_t*)pu8_target) - sizeof(uint32_t);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return l_alloc_size;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_initialize
 *
 * DESCRIPTION:Initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_initialize() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    pf_mutex_init();

    //==============================================================================
    // ストレージ情報
    //==============================================================================
    // メモリストレージ生成
    static uint8_t u8_mem_storsge[MEM_STORAGE_SIZE];
    // メモリストレージ設定
    pu8_mem_storage = u8_mem_storsge;
    // メモリストレージクリア
    memset(pu8_mem_storage, 0x00, MEM_STORAGE_SIZE);

    //==========================================================================
    // 空き領域情報を初期化
    //==========================================================================
    // 空き領域情報の先頭アドレス
    ps_info_list = (ts_mem_area_info_t*)u8_mem_storsge;
    // 領域情報の最終インデックス
    u32_info_last_idx = 0;
    // 空き領域情報の初期化
    ts_mem_area_info_t* ps_info = &ps_info_list[u32_info_last_idx];
    // アドレス
    ps_info->pu8_address = (uint8_t*)&ps_info_list[u32_info_last_idx + 1];
    // サイズ
    ps_info->u32_size = MEM_STORAGE_SIZE - sizeof(ts_mem_area_info_t);
    ps_info->ps_addr_prev = NULL;   // 前の空き領域情報（アドレス昇順）
    ps_info->ps_addr_next = NULL;   // 次の空き領域情報（アドレス昇順）
    ps_info->ps_size_prev = NULL;   // 前の空き領域情報（サイズ昇順）
    ps_info->ps_size_next = NULL;   // 次の空き領域情報（サイズ昇順）

    //==========================================================================
    // リンクリスト情報の初期化
    //==========================================================================
    // 空き領域情報（アドレス昇順）の先頭
    ps_addr_top  = ps_info;
    // 空き領域情報（アドレス昇順）の末尾
    ps_addr_tail = ps_info;
    // 空き領域情報（サイズ昇順）の先頭
    ps_size_top  = ps_info;
    // 空き領域情報（サイズ昇順）の末尾
    ps_size_tail = ps_info;
    // 空き領域情報（領域情報のアドレス昇順）の先頭
    ps_unused_top = NULL;

    //==========================================================================
    // 関連情報の初期化
    //==========================================================================
    // メモリ割り当て領域アドレス（先端）
    pu8_mem_usage_top = (uint8_t*)0xFFFFFFFF;
    // メモリ割り当て領域アドレス（末尾）
    pu8_mem_usage_tail = &pu8_mem_storage[MEM_STORAGE_SIZE - 1];
    // 使用領域のサイズ（空き領域情報除く）
    u32_mem_val_usage_size = 0;
    // ユーザーへの割り当て済みサイズ
    u32_mem_val_alloc_size = 0;
    // 空き領域の個数
    u32_mem_free_cnt = 1;

    //==========================================================================
    // 初期処理の切り替え
    //==========================================================================
    pf_initialize = v_initialize_dummy;
}

/*******************************************************************************
 *
 * NAME: v_initialize_dummy
 *
 * DESCRIPTION:Dummy initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_initialize_dummy() {
    return;
}

/*******************************************************************************
 *
 * NAME: v_mutex_init
 *
 * DESCRIPTION:Mutex Initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mutex_init() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    //==========================================================================
    // ミューテックス初期処理の切り替え
    //==========================================================================
    pf_mutex_init = v_mutex_init_dummy;
}

/*******************************************************************************
 *
 * NAME: v_mutex_init_dummy
 *
 * DESCRIPTION:Dummy Mutex initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mutex_init_dummy() {
    return;
}

/*******************************************************************************
 *
 * NAME: pu8_mem_alloc
 *
 * DESCRIPTION:空き領域情報からのメモリ確保処理
 *
 * PARAMETERS:          Name        RW  Usage
 * uint32_t             u32_size    R   確保するメモリサイズ
 *
 * RETURNS:
 *   uint8_t*:allocated memory
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint8_t* pu8_mem_alloc(uint32_t u32_size) {
    // 入力チェック
    if (u32_size == 0) {
        return NULL;
    }
    // 管理用の項目も含めた割り当てサイズを算出
    uint32_t u32_alloc_size = u32_size + sizeof(uint32_t);
    // 確保対象の空き領域情報を探索 ※サイズ昇順なので末尾から検索
    ts_mem_area_info_t* ps_target = ps_search_alloc_area(u32_alloc_size);
    if (ps_target == NULL) {
        // 空き領域が無い場合
        return NULL;
    }

    //==========================================================================
    // メモリ領域の確保
    //==========================================================================
    // 確保後の空き領域のサイズを算出
    uint32_t u32_new_size  = ps_target->u32_size - u32_alloc_size;
    // 確保したメモリ（更新後の空き領域の直後）へのポインタを取得
    uint8_t* pu8_alloc_mem = ps_target->pu8_address + u32_new_size;
    uint32_t* pu32_alloc_mem = (uint32_t*)pu8_alloc_mem;
    // 利用領域の先頭アドレスを必要に応じて更新
    if (pu8_alloc_mem < pu8_mem_usage_top) {
        pu8_mem_usage_top = pu8_alloc_mem;
    }

    //==========================================================================
    // 空き領域のサイズを更新
    //==========================================================================
    v_upd_area_size(ps_target, u32_new_size);

    //==========================================================================
    // 先頭にサイズを書き込んで返却値を編集
    //==========================================================================
    // 割り当て領域の先頭にサイズを書き込む
    *pu32_alloc_mem = u32_alloc_size;
    // サイズを書き込んだ直後のポインタ
    uint8_t* pu8_result = pu8_alloc_mem + sizeof(uint32_t);

    //==========================================================================
    // 領域全体のサイズ情報を更新
    //==========================================================================
    // 割り当て済み（ユーザーから見た）のサイズを更新
    u32_mem_val_alloc_size += u32_size;
    // 実際の使用領域のサイズを更新
    u32_mem_val_usage_size += u32_alloc_size;

    // サイズを書き込んだ直後のポインタを返却
    return pu8_result;
}

/*******************************************************************************
 *
 * NAME: pu8_mem_realloc
 *
 * DESCRIPTION:メモリ領域の再定義処理
 *
 * PARAMETERS:          Name            RW  Usage
 * void*                pv_memory       R   解放対象のメモリ
 * uint32_t             u32_size        R   再定義後のサイズ
 *
 * RETURNS:
 *   uint8_t*:再定義されたメモリへのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint8_t* pu8_mem_realloc(void* pv_memory, uint32_t u32_size) {
    //==========================================================================
    // メモリ解放判定
    //==========================================================================
    if (u32_size == 0) {
        // メモリを解放
        l_mem_free_memory(pv_memory);
        return NULL;
    }

    //==========================================================================
    // パラメータによる処理
    //==========================================================================
    uint8_t* pu8_memory = NULL;
    do {
        // 再定義するメモリを確保
        pu8_memory = pu8_mem_alloc(u32_size);
        // メモリ領域のコピーの有無を判定
        if (pu8_memory == NULL || pv_memory == NULL) {
            // 確保に失敗したか、コピー元が無いのでNULLを返却
            break;
        }
        // コピーサイズを判定
        uint32_t u32_copy_len = *((uint32_t*)(pv_memory - sizeof(uint32_t)));
        if (u32_copy_len > u32_size) {
            u32_copy_len = u32_size;
        }
        // メモリイメージをコピー
        memcpy(pu8_memory, pv_memory, u32_copy_len);
        // コピー元の解放処理
        l_mem_free_memory(pv_memory);
    } while(false);
    // 再定義したメモリ領域へのポインタを返却
    return pu8_memory;
}

/*******************************************************************************
 *
 * NAME: l_mem_free_memory
 *
 * DESCRIPTION:メモリ解放処理
 *
 * PARAMETERS:          Name            RW  Usage
 * void*                pv_memory       R   解放対象のメモリ
 *
 * RETURNS:
 *   uint32_t:解放されたメモリ（ユーザーに割り当てた）のサイズ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static long l_mem_free_memory(void* pv_memory) {
    long l_free_size = -1;
    do {
        // 入力チェック
        if (pv_memory == NULL) {
            break;
        }
        // 先頭アドレスを算出
        uint8_t* pu8_target = (uint8_t*)(pv_memory - sizeof(uint32_t));
        // 入力チェック
        if (pu8_target < pu8_mem_usage_top || pu8_target > pu8_mem_usage_tail) {
            // 確保されたメモリ領域では無い場合
            break;
        }
        // 確保メモリサイズを取得
        uint32_t u32_alloc_size = *((uint32_t*)pu8_target);
        // 既存の空き領域情報の有無を判定
        if (ps_addr_tail == NULL) {
            // 既存の空き領域が無いので、単独の空き領域として追加
            l_free_size = u32_add_list(pu8_target, u32_alloc_size);
            break;
        }
        // 直前の空き領域情報を検索
        ts_mem_area_info_t* ps_prev = ps_search_prev_addr_area(pu8_target);
        if (ps_prev == NULL) {
            // 全て割り当て済みなので、単独の空き領域として追加
            l_free_size = u32_add_list(pu8_target, u32_alloc_size);
            break;
        }
        // 直後の空き領域との結合を判定
        uint32_t u32_new_size = u32_alloc_size;
        ts_mem_area_info_t* ps_next = ps_prev->ps_addr_next;
        if (ps_next != NULL) {
            uint8_t* pu8_next_addr = pu8_target + u32_alloc_size;
            if (ps_next->pu8_address == pu8_next_addr) {
                // 直後の空き領域と統合する場合
                u32_new_size = u32_new_size + ps_next->u32_size;
                // 直後の空き領域情報をリサイクル候補に変更
                v_add_unused_list(ps_next);
            } else if (ps_next->pu8_address < pu8_next_addr) {
                // アドレス不正の場合
                break;
            }
        }
        // 直前の空き領域の拡大を判定
        uint8_t* pu8_after = (uint8_t*)(ps_prev->pu8_address + ps_prev->u32_size);
        if (pu8_after == pu8_target) {
            // 直前の空き領域を拡大
            u32_new_size = u32_new_size + ps_prev->u32_size;
            v_upd_area_size(ps_prev, u32_new_size);
            // 結果返信
            l_free_size = u32_alloc_size;
            break;
        }
        // 新規に空き領域を追加
        if (u32_add_list(pu8_target, u32_new_size) != u32_new_size) {
            // 新規に空き領域を追加出来なかった場合
            break;
        }
        // 結果返信
        l_free_size = u32_alloc_size;
    } while(false);
    // 結果判定
    if (l_free_size <= 0) {
        // 結果返信
        return l_free_size;
    }
    // ユーザーから見たサイズ
    long l_user_size = l_free_size - sizeof(uint32_t);
    // 割り当て済み（ユーザーから見た）のサイズを更新
    u32_mem_val_alloc_size -= l_user_size;
    // 実際の使用領域のサイズを更新
    u32_mem_val_usage_size -= l_free_size;

    // 結果返信
    return l_user_size;
}

/*******************************************************************************
 *
 * NAME: ps_search_prev_addr_area
 *
 * DESCRIPTION:直前のアドレスの領域情報を検索
 *
 * PARAMETERS:          Name            RW  Usage
 * uint8_t*             pu8_pointer*    R   検索アドレス
 *
 * RETURNS:
 *   ts_mem_area_info_t*:Last area information
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_mem_area_info_t* ps_search_prev_addr_area(uint8_t* pu8_pointer) {
    // 末尾から検索
    ts_mem_area_info_t* ps_prev = ps_addr_tail;
    while (ps_prev != NULL) {
        if (ps_prev->pu8_address < pu8_pointer) {
            // 結果として設定
            break;
        }
        // 前の要素へ
        ps_prev = ps_prev->ps_addr_prev;
    }
    // 検索結果を返却
    return ps_prev;
}

/*******************************************************************************
 *
 * NAME: ps_search_prev_size_area
 *
 * DESCRIPTION:直前のサイズの領域情報を検索
 *
 * PARAMETERS:          Name        RW  Usage
 * uint32_t             u32_size    R   検索サイズ
 *
 * RETURNS:
 *   ts_mem_area_info_t*=search result
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_mem_area_info_t* ps_search_prev_size_area(uint32_t u32_size) {
    // 検索結果
    ts_mem_area_info_t* ps_prev = ps_size_tail;
    // 末尾から検索
    while (ps_prev != NULL) {
        if (ps_prev->u32_size <= u32_size) {
            break;
        }
        // 前の要素へ
        ps_prev = ps_prev->ps_size_prev;
    }
    return ps_prev;
}

/*******************************************************************************
 *
 * NAME: ps_search_alloc_area
 *
 * DESCRIPTION:サイズが確保出来る最小の領域情報を検索
 *
 * PARAMETERS:          Name        RW  Usage
 * uint32_t             u32_size    R   検索サイズ
 *
 * RETURNS:
 *   ts_mem_area_info_t*=search result
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_mem_area_info_t* ps_search_alloc_area(uint32_t u32_size) {
    // 検索結果
    ts_mem_area_info_t* ps_result = NULL;
    // 末尾(最大サイズ)の空き領域情報から検索
    ts_mem_area_info_t* ps_chk = ps_size_tail;
    while (ps_chk != NULL) {
        if (ps_chk->u32_size < u32_size) {
            break;
        }
        // 結果の候補として設定
        ps_result = ps_chk;
        // 前の要素へ
        ps_chk = ps_chk->ps_size_prev;
    }
    // 検索結果を返却
    return ps_result;
}

/*******************************************************************************
 *
 * NAME: v_upd_area_size
 *
 * DESCRIPTION:領域情報のサイズ更新処理
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_mem_area_info_t*  ps_target       R   更新対象
 * uint32_t             u32_new_size    R   更新後サイズ
 *
 * RETURNS:
 *   void*:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_upd_area_size(ts_mem_area_info_t* ps_target, uint32_t u32_new_size) {
    //==========================================================================
    // 対象の空き領域が無くなるか判定（全域を割り当てるか判定）
    //==========================================================================
    if (u32_new_size == 0) {
        // 空き領域では無くなるので、対象を再利用リストに付け替える
        v_add_unused_list(ps_target);
        return;
    } else if (ps_target->u32_size == u32_new_size) {
        // サイズに変化が無いので何もしない
        return;
    }
    //==========================================================================
    // サイズリストの再編成判定
    //==========================================================================
    //
    // サイズ更新後の直前の空き領域情報を検索
    ts_mem_area_info_t* ps_prev = ps_search_prev_size_area(u32_new_size);
    // 対象の領域情報を更新
    ps_target->u32_size = u32_new_size;
    // リンクリストの再編成を判定
    if (ps_prev == ps_target || ps_prev == ps_target->ps_size_prev) {
        return;
    }

    //==========================================================================
    // サイズのリストから対象の空き領域情報を除去
    //==========================================================================
    v_remove_area_info_size(ps_target);

    //==========================================================================
    // 空き領域情報の移動
    //==========================================================================
    // 移動先の直後の空き領域
    ts_mem_area_info_t* ps_next = ps_size_top;
    // 直前の空き領域情報を編集
    if (ps_prev != NULL) {
        // 移動先に移動
        ps_next = ps_prev->ps_size_next;
        ps_prev->ps_size_next = ps_target;
    } else {
        // 先頭に移動
        ps_size_top = ps_target;
    }
    // 直後の空き領域情報を編集
    if (ps_next != NULL) {
        ps_next->ps_size_prev = ps_target;
    } else {
        ps_size_tail = ps_target;
    }
    // 対象の空き領域を編集
    ps_target->ps_size_prev = ps_prev;
    ps_target->ps_size_next = ps_next;
}

/*******************************************************************************
 *
 * NAME: u32_add_list
 *
 * DESCRIPTION:空き領域情報を追加
 *
 * PARAMETERS:      Name            RW  Usage
 * uint8_t*         pu8_area        R   空き領域の先頭アドレス
 * uint32_t         u32_size        R   空き領域サイズ
 *
 * RETURNS:
 *   uint32_t:解放メモリサイズ（管理領域を含まない）
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint32_t u32_add_list(uint8_t* pu8_area, uint32_t u32_size) {
    //==========================================================================
    // 空き領域情報を取得
    //==========================================================================
    // 未使用領域情報の取得を判定
    ts_mem_area_info_t* ps_info = ps_get_unused_info();
    if (ps_info == NULL) {
        return 0;
    }
    // アドレスを編集
    ps_info->pu8_address = pu8_area;
    // サイズを編集
    ps_info->u32_size = u32_size;

    //==========================================================================
    // 空き領域のアドレスリストに追加
    //==========================================================================
    // 空き領域情報リスト（アドレス）を検索
    ts_mem_area_info_t* ps_addr_prev = ps_search_prev_addr_area(pu8_area);
    ts_mem_area_info_t* ps_addr_next = ps_addr_top;
    // 直前の空き領域を編集
    if (ps_addr_prev != NULL) {
        // 直後の空き領域を更新
        ps_addr_next = ps_addr_prev->ps_addr_next;
        // 直前の空き領域と結合
        ps_addr_prev->ps_addr_next = ps_info;
    } else {
        // 空き領域をトップの設定
        ps_addr_top = ps_info;
    }
    // 直後の空き領域と結合
    if (ps_addr_next != NULL) {
        ps_addr_next->ps_addr_prev = ps_info;
    } else {
        ps_addr_tail = ps_info;
    }
    // 前後の空き領域と結合
    ps_info->ps_addr_prev = ps_addr_prev;
    ps_info->ps_addr_next = ps_addr_next;

    //==========================================================================
    // 空き領域のサイズリストに追加
    //==========================================================================
    // 空き領域情報リスト（サイズ）を検索
    ts_mem_area_info_t* ps_size_prev = ps_search_prev_size_area(u32_size);
    ts_mem_area_info_t* ps_size_next = ps_size_top;
    // 直前の空き領域との結合
    if (ps_size_prev != NULL) {
        // 直後の空き領域を更新
        ps_size_next = ps_size_prev->ps_size_next;
        // 直前の空き領域との結合
        ps_size_prev->ps_size_next = ps_info;
    } else {
        // 追加する空き領域を先頭にする
        ps_size_top = ps_info;
    }
    // 直後の空き領域と結合
    if (ps_size_next != NULL) {
        // 直後の空き領域と結合
        ps_size_next->ps_size_prev = ps_info;
    } else {
        // 追加する空き領域を末尾にする
        ps_size_tail = ps_info;
    }
    // 前後の空き領域と結合
    ps_info->ps_size_prev = ps_size_prev;
    ps_info->ps_size_next = ps_size_next;

    // 正常終了
    return u32_size;
}

/*******************************************************************************
 *
 * NAME: v_add_unused_list
 *
 * DESCRIPTION:領域情報の再利用リストへの追加
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_mem_area_info_t*  ps_info     R   追加対象
 *
 * RETURNS:
 *   void*:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_add_unused_list(ts_mem_area_info_t* ps_info) {
    //==========================================================================
    // 領域情報をクリア
    //==========================================================================
    // リストから対象を除去
    v_remove_area_info_addr(ps_info);
    v_remove_area_info_size(ps_info);
    // 領域情報自体のアドレスを設定
    ps_info->pu8_address = (uint8_t*)&ps_info;
    // サイズをクリア
    ps_info->u32_size = 0;
    // 前の空き領域情報（アドレス昇順）
    ps_info->ps_addr_prev = NULL;
    // 次の空き領域情報（アドレス昇順）
    ps_info->ps_addr_next = NULL;
    // 前の空き領域情報（サイズ昇順）
    ps_info->ps_size_prev = NULL;
    // 次の空き領域情報（サイズ昇順）
    ps_info->ps_size_next = NULL;
    // 空き領域の個数を更新
    u32_mem_free_cnt--;

    //==========================================================================
    // 再利用リストの有無を判定
    //==========================================================================
    if (ps_unused_top == NULL) {
        ps_unused_top = ps_info;
        return;
    }

    //==========================================================================
    // リンクリスト（領域情報アドレス昇順）への挿入点を探索
    //==========================================================================
    // 領域情報のアドレス降順のリンクリストを検索し、挿入位置を特定する
    ts_mem_area_info_t* ps_prev = NULL;
    ts_mem_area_info_t* ps_next = ps_unused_top;
    while(ps_next != NULL) {
        if (ps_next >= ps_info) {
            break;
        }
        // 直前の領域情報
        ps_prev = ps_next;
        // 次の領域情報へ
        ps_next = ps_next->ps_addr_next;
    }

    //==========================================================================
    // リンクリスト（領域情報アドレス昇順）への挿入
    //==========================================================================
    // 前の領域情報を編集
    if (ps_prev != NULL) {
        // 前の領域情報からのリンク（アドレス）を設定
        ps_prev->ps_addr_next = ps_info;
    } else {
        // 再利用トップへ挿入
        ps_unused_top = ps_info;
    }
    // 次の領域情報を編集
    if (ps_next != NULL) {
        // 次の領域情報からのリンク（アドレス）を設定
        ps_next->ps_addr_prev = ps_info;
    }
    // 前後の領域情報と結合
    ps_info->ps_addr_prev = ps_prev;
    ps_info->ps_addr_next = ps_next;
}

/*******************************************************************************
 *
 * NAME: v_remove_area_info_addr
 *
 * DESCRIPTION:アドレスのリンクリストから領域情報を抜き取る
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_mem_area_info_t*  ps_info     R   リストから取り除く対象の領域情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_remove_area_info_addr(ts_mem_area_info_t* ps_info) {
    //==========================================================================
    // リストからの取り出し
    //==========================================================================
    // 前の空き領域情報（アドレス昇順）
    ts_mem_area_info_t* ps_addr_prev = ps_info->ps_addr_prev;
    // 次の空き領域情報（アドレス昇順）
    ts_mem_area_info_t* ps_addr_next = ps_info->ps_addr_next;
    // リンクリスト（アドレス）の前の領域情報を付け替え
    if (ps_addr_prev != NULL) {
        ps_addr_prev->ps_addr_next = ps_addr_next;
    }else {
        ps_addr_top = ps_addr_next;
    }
    // リンクリスト（アドレス）の次の領域情報を付け替え
    if (ps_addr_next != NULL) {
        ps_addr_next->ps_addr_prev = ps_addr_prev;
    }else {
        ps_addr_tail = ps_addr_prev;
    }
}

/*******************************************************************************
 *
 * NAME: v_remove_area_info_size
 *
 * DESCRIPTION:サイズのリンクリストから領域情報を抜き取る
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_mem_area_info_t*  ps_info     R   リストから取り除く対象の領域情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_remove_area_info_size(ts_mem_area_info_t* ps_info) {
    //==========================================================================
    // リストからの取り出し
    //==========================================================================
    // 前の空き領域情報（サイズ昇順）
    ts_mem_area_info_t* ps_size_prev = ps_info->ps_size_prev;
    // 次の空き領域情報（サイズ昇順）
    ts_mem_area_info_t* ps_size_next = ps_info->ps_size_next;
    // リンクリスト（サイズ）の前の領域情報を付け替え
    if (ps_size_prev != NULL) {
        ps_size_prev->ps_size_next = ps_size_next;
    }else {
        ps_size_top = ps_size_next;
    }
    // リンクリスト（サイズ）の次の領域情報を付け替え
    if (ps_size_next != NULL) {
        ps_size_next->ps_size_prev = ps_size_prev;
    }else {
        ps_size_tail = ps_size_prev;
    }
}

/*******************************************************************************
 *
 * NAME: ps_get_unused_info
 *
 * DESCRIPTION:未使用の領域情報を取得
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *  ts_mem_area_info_t*:領域情報を取り出す
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_mem_area_info_t* ps_get_unused_info() {
    //==========================================================================
    // 再利用リストからの領域情報の取り出し
    //==========================================================================
    ts_mem_area_info_t* ps_info = ps_unused_top;
    if (ps_info != NULL) {
        // 後続の空き領域の有無を判定
        ps_unused_top = ps_info->ps_addr_next;
        if (ps_unused_top != NULL) {
            // 次の領域情報から切り離す
            ps_info->ps_addr_next = NULL;
            ps_unused_top->ps_addr_prev = NULL;
        }
        // 空き領域の個数を更新
        u32_mem_free_cnt++;
        // 空き領域を返信
        return ps_info;
    }

    //==========================================================================
    // 新規に領域情報を取得
    //==========================================================================
    // 領域情報追加後の終端を取得
    uint8_t* pu8_info_tail = (uint8_t*)&ps_info_list[u32_info_last_idx + 2];
    if (pu8_info_tail > pu8_mem_usage_top) {
        // 領域情報の配置場所が無い場合
        return NULL;
    }
    // 領域情報の最終インデックスを更新
    u32_info_last_idx++;
    // 新しい領域情報を初期化
    ps_info = &ps_info_list[u32_info_last_idx];
    // アドレス
    ps_info->pu8_address = NULL;
    // サイズ
    ps_info->u32_size    = 0;
    // 前の空き領域情報（アドレス昇順）
    ps_info->ps_addr_prev = NULL;
    // 次の空き領域情報（アドレス昇順）
    ps_info->ps_addr_next = NULL;
    // 前の空き領域情報（サイズ昇順）
    ps_info->ps_size_prev = NULL;
    // 次の空き領域情報（サイズ昇順）
    ps_info->ps_size_next = NULL;
    // 空き領域の個数を更新
    u32_mem_free_cnt++;
    // 新しい領域を返却
    return ps_info;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
