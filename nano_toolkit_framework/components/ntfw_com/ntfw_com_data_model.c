/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :source file for types and functions related to the data model
 *
 * CREATED:2020/08/03 03:58:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:データモデルに関する型と関数群
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
#include "ntfw_com_data_model.h"

#include <string.h>
#include "ntfw_com_value_util.h"
#include "ntfw_com_mem_alloc.h"


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** ログ接頭辞 */
#define LOG_TAG_DATA_MODEL "COM_DATA_MODEL"

/** キューブロックサイズ */
#define QUEUE_BLOCK_SIZE    (4)
/** キューブロック数 */
#define QUEUE_BLOCK_COUNT   (4)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** リンクリストキューの初期値 */
static const ts_linked_queue_t s_linked_queue_default = {
    .s_mutex = NULL,    // ミューテックス
    .pt_top  = NULL,    // 先頭要素
    .pt_tail = NULL,    // 末尾要素
    .u16_count  = 0,    // 要素数
    .t_size  = 0        // データ長
};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** リンクリストのクリア */
static void v_clear_linked_queue(ts_linked_queue_t* ps_queue);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: ps_mdl_create_array
 *
 * DESCRIPTION:バイト配列情報構造体の生成処理（引数のバイト配列を直接参照）
 *
 * PARAMETERS:      Name        RW  Usage
 *   uint8_t*       pu8_data    R   対象データ
 *   size_t         t_size      R   対象データのサイズ
 *
 * RETURNS:
 *   動的にメモリ確保して生成したバイト配列情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_mdl_create_u8_array(uint8_t* pu8_data, size_t t_size) {
    // 配列情報の生成
    ts_u8_array_t* ps_array = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (ps_array == NULL) {
        return NULL;
    }
    ps_array->b_clone = false;
    ps_array->t_size  = t_size;
    ps_array->pu8_values = pu8_data;
    // 結果返却
    return ps_array;
}

/*******************************************************************************
 *
 * NAME: ps_mdl_clone_u8_array
 *
 * DESCRIPTION:動的に確保されたバイト配列情報構造体の生成処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   uint8_t*       pu8_data    R   対象データ
 *   size_t         t_size      R   対象データのサイズ
 *
 * RETURNS:
 *   動的にメモリ確保して生成したバイト配列情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_mdl_clone_u8_array(const uint8_t* pu8_data, size_t t_size) {
    // 配列情報の生成
    ts_u8_array_t* ps_array = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (ps_array == NULL) {
        return NULL;
    }
    ps_array->b_clone = true;
    ps_array->t_size  = t_size;
    ps_array->pu8_values = (uint8_t*)pv_mem_clone((void*)pu8_data, t_size);
    if (ps_array->pu8_values == NULL) {
        l_mem_free(ps_array);
        return NULL;
    }
    // 結果返却
    return ps_array;
}

/*******************************************************************************
 *
 * NAME: ps_mdl_empty_u8_array
 *
 * DESCRIPTION:空のバイト配列情報構造体の生成処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   size_t         t_size      R   対象データのサイズ
 *
 * RETURNS:
 *   動的にメモリ確保して生成したバイト配列情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_mdl_empty_u8_array(size_t t_size) {
    // 配列情報の生成
    ts_u8_array_t* ps_array = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (ps_array == NULL) {
        return NULL;
    }
    ps_array->b_clone = true;
    ps_array->t_size  = t_size;
    ps_array->pu8_values = pv_mem_calloc(t_size);
    if (ps_array->pu8_values == NULL) {
        l_mem_free(ps_array);
        return NULL;
    }
    // 値を初期化
    memset(ps_array->pu8_values, 0x00, t_size);
    // 結果返却
    return ps_array;
}

/*******************************************************************************
 *
 * NAME: ps_mdl_random_u8_array
 *
 * DESCRIPTION:乱数のバイト配列情報構造体の生成処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   size_t         t_size      R   対象データのサイズ
 *
 * RETURNS:
 *   動的にメモリ確保して生成したバイト配列情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_mdl_random_u8_array(size_t t_size) {
    // 配列情報の生成
    ts_u8_array_t* ps_array = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (ps_array == NULL) {
        return NULL;
    }
    ps_array->b_clone = true;
    ps_array->t_size  = t_size;
    ps_array->pu8_values = pv_mem_malloc(t_size);
    if (ps_array->pu8_values == NULL) {
        l_mem_free(ps_array);
        return NULL;
    }
    // 値を初期化
    b_vutil_set_u8_rand_array(ps_array->pu8_values, t_size);
    // 結果返却
    return ps_array;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_delete_u8_array
 *
 * DESCRIPTION:動的に確保されたバイト配列情報構造体の解放処理
 *
 * PARAMETERS:      Name        RW  Usage
 * ts_array_t*      ps_array    W   バイト配列情報へのポインタ
 *
 * RETURNS:
 *   ESP_OK:正常終了
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_mdl_delete_u8_array(ts_u8_array_t* ps_array) {
    // 入力チェック
    if (ps_array == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // メモリ解放処理
    if (ps_array->b_clone && ps_array->pu8_values != NULL) {
        l_mem_free(ps_array->pu8_values);
        ps_array->pu8_values = NULL;
    }
    l_mem_free(ps_array);
    // 結果を返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_queue_send_array
 *
 * DESCRIPTION:動的に確保されたバイト配列データのキューイング
 *
 * PARAMETERS:          Name        RW  Usage
 * QueueHandle_t        x_queue     R   キュー
 * const ts_u8_array_t* ps_array    R   配列データ
 * TickType_t           x_wait      R   ウェイトタイム
 *
 * RETURNS:
 *   pdTRUE:正常終了
 *
 * NOTES:
 * None.
 ******************************************************************************/
BaseType_t sts_mdl_queue_send(QueueHandle_t x_queue, uint8_t* pu8_list, uint32_t u32_size, TickType_t x_wait) {
    // 入力チェック
    if (x_queue == NULL || pu8_list == NULL || u32_size == 0) {
        return pdFALSE;
    }
    // 配列のエントリ
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_size; u32_idx++) {
        if (xQueueSend(x_queue, &pu8_list[u32_idx], x_wait) != pdTRUE) {
            return pdFALSE;
        }
    }
    return pdTRUE;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_queue_send_array
 *
 * DESCRIPTION:動的に確保されたバイト配列データのキューイング
 *
 * PARAMETERS:          Name        RW  Usage
 * QueueHandle_t        x_queue     R   キュー
 * const ts_u8_array_t* ps_array    R   配列データ
 * TickType_t           x_wait      R   ウェイトタイム
 *
 * RETURNS:
 *   pdTRUE:正常終了
 *
 * NOTES:
 * None.
 ******************************************************************************/
BaseType_t sts_mdl_queue_send_array(QueueHandle_t x_queue, const ts_u8_array_t* ps_array, TickType_t x_wait) {
    // 入力チェック
    if (x_queue == NULL || ps_array == NULL) {
        return pdFALSE;
    }
    // 配列のエントリ
    size_t t_idx;
    for (t_idx = 0; ps_array->t_size > t_idx; t_idx++) {
        if (xQueueSend(x_queue, &ps_array->pu8_values[t_idx], x_wait) != pdTRUE) {
            return pdFALSE;
        }
    }
    return pdTRUE;
}

//==============================================================================
// リンクリストキューの処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: ps_mdl_create_linked_queue
 *
 * DESCRIPTION:リンクリストキューの生成処理
 *
 * PARAMETERS:          Name            RW  Usage
 *
 * RETURNS:
 *   ts_linked_queue_t*:生成したキューのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_linked_queue_t* ps_mdl_create_linked_queue() {
    // キュー生成
    ts_linked_queue_t* ps_queue =
            (ts_linked_queue_t*)pv_mem_malloc(sizeof(ts_linked_queue_t));
    if (ps_queue == NULL) {
        return NULL;
    }
    // キュー初期化
    *ps_queue = s_linked_queue_default;
    ps_queue->s_mutex = xSemaphoreCreateMutex();
    // 生成したキューを返信
    return ps_queue;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_delete_linked_queue
 *
 * DESCRIPTION:リンクリストキューの削除処理
 *
 * PARAMETERS:          Name            RW  Usage
 *
 * RETURNS:
 *   ts_linked_queue_t*:生成したキューのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_mdl_delete_linked_queue(ts_linked_queue_t* ps_queue) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTake(ps_queue->s_mutex, portMAX_DELAY) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 要素をクリア
    //==========================================================================
    v_clear_linked_queue(ps_queue);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGive(ps_queue->s_mutex);

    //==========================================================================
    // キュー削除
    //==========================================================================
    // ミューテックスの解放
    vSemaphoreDelete(ps_queue->s_mutex);
    // リンクリストキュー自体の解放
    l_mem_free(ps_queue);

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_clear_linked_queue
 *
 * DESCRIPTION:リンクリストキューのクリア処理
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_linked_queue_t*   ps_queue    RW  リンクリストキュー
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_mdl_clear_linked_queue(ts_linked_queue_t* ps_queue) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTake(ps_queue->s_mutex, portMAX_DELAY) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // リンクリストのクリア処理
    //==========================================================================
    v_clear_linked_queue(ps_queue);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGive(ps_queue->s_mutex);

    // 読み出したデータを返却
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_mdl_linked_enqueue
 *
 * DESCRIPTION:リンクリストキューへのエンキュー
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_linked_queue_t*   ps_queue    RW  リンクリストキュー
 * uint8_t*             pu8_list    R   データ配列
 * size_t               t_size      R   データ配列サイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_mdl_linked_enqueue(ts_linked_queue_t* ps_queue,
                                 const uint8_t* pu8_list,
                                 size_t t_size) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pu8_list == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (t_size == 0) {
        return ESP_OK;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTake(ps_queue->s_mutex, portMAX_DELAY) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // エンキュー処理
    //==========================================================================
    // エレメント生成
    ts_linked_element_t* ps_elm = (ts_linked_element_t*)pv_mem_malloc(sizeof(ts_linked_element_t));
    if (ps_elm == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ps_elm->ps_prev  = ps_queue->pt_tail;
    ps_elm->ps_next  = NULL;
    ps_elm->pv_value = ps_mdl_clone_u8_array(pu8_list, t_size);
    if (ps_elm->pv_value == NULL) {
        l_mem_free(ps_elm);
        return ESP_ERR_NO_MEM;
    }
    // エレメントの追加
    if (ps_queue->pt_top == NULL) {
        ps_queue->pt_top  = ps_elm;
        ps_queue->pt_tail = ps_elm;
    } else {
        ps_queue->pt_tail->ps_next = ps_elm;
        ps_queue->pt_tail = ps_elm;
    }
    // データ長更新
    ps_queue->u16_count++;
    ps_queue->t_size += t_size;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGive(ps_queue->s_mutex);
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_mdl_linked_dequeue
 *
 * DESCRIPTION:リンクリストキューからデキュー
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_linked_queue_t*   ps_queue    RW  リンクリストキュー
 * size_t               t_size      R   データ配列サイズ
 *
 * RETURNS:
 *   ts_u8_array_t*:デキューしたデータ配列構造体のポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_mdl_linked_dequeue(ts_linked_queue_t* ps_queue, size_t t_size) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_queue == NULL) {
        return NULL;
    }
    if (ps_queue->u16_count == 0) {
        return NULL;
    }
    if (t_size == 0) {
        return NULL;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTake(ps_queue->s_mutex, portMAX_DELAY) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // デキュー処理
    //==========================================================================
    // デキューサイズ算出
    size_t t_rem_size = t_size;
    if (t_rem_size > ps_queue->t_size) {
        t_rem_size = ps_queue->t_size;
    }
    // 返信値の生成
    ts_u8_array_t* pt_ret = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (pt_ret == NULL) {
        return NULL;
    }
    pt_ret->pu8_values = (uint8_t*)pv_mem_malloc(t_rem_size);
    if (pt_ret->pu8_values == NULL) {
        l_mem_free(pt_ret);
        return NULL;
    }
    pt_ret->b_clone = true;
    pt_ret->t_size  = t_rem_size;
    // ワークデータ
    ts_linked_element_t* pt_wk_elm;
    ts_u8_array_t* pt_wk_array;
    uint8_t* pu8_wk_data;
    // コピー先インデックス
    uint32_t u32_pos = 0;
    while (t_rem_size > 0) {
        pt_wk_elm   = ps_queue->pt_top;
        pt_wk_array = pt_wk_elm->pv_value;
        if (pt_wk_array->t_size > t_rem_size) {
            // 先頭データ配列の置き換え
            memcpy(&pt_ret->pu8_values[u32_pos], pt_wk_array->pu8_values, t_rem_size);
            if (pt_wk_array->b_clone) {
                pu8_wk_data = pt_wk_array->pu8_values;
                pt_wk_array->t_size -= t_rem_size;
                pt_wk_array->pu8_values = (uint8_t*)pv_mem_malloc(pt_wk_array->t_size);
                memcpy(pt_wk_array->pu8_values, &pu8_wk_data[t_rem_size], pt_wk_array->t_size);
                if (pu8_wk_data != NULL) {
                    l_mem_free(pu8_wk_data);
                }
            } else {
                pt_wk_array->pu8_values += t_rem_size;
                pt_wk_array->t_size -= t_rem_size;
            }
            ps_queue->t_size -= t_rem_size;
            break;
        }
        // 先頭データ配列の読み出し
        // データコピー
        memcpy(&pt_ret->pu8_values[u32_pos], pt_wk_array->pu8_values, pt_wk_array->t_size);
        // 先頭データを更新
        ps_queue->pt_top = pt_wk_elm->ps_next;
        // コピー先情報更新
        u32_pos += pt_wk_array->t_size;
        t_rem_size -= pt_wk_array->t_size;
        // キュー情報更新
        ps_queue->u16_count--;
        ps_queue->t_size -= pt_wk_array->t_size;
        // デキューした要素を廃棄
        sts_mdl_delete_u8_array(pt_wk_array);
        l_mem_free(pt_wk_elm);
    }
    // 末尾の要素に対応
    if (ps_queue->pt_top == NULL) {
        ps_queue->pt_tail = NULL;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGive(ps_queue->s_mutex);

    // 読み出したデータを返却
    return pt_ret;
}


/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_clear_linked_queue
 *
 * DESCRIPTION:リンクリストキューのクリア処理
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_linked_queue_t*   ps_queue    RW  リンクリストキュー
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_clear_linked_queue(ts_linked_queue_t* ps_queue) {
    // 要素数の判定
    if (ps_queue->u16_count == 0) {
        return;
    }

    //==========================================================================
    // リンクリストのクリア処理
    //==========================================================================
    // ワークデータ
    ts_linked_element_t* pt_wk_elm;
    while (ps_queue->u16_count > 0) {
        // 先頭データを更新
        pt_wk_elm        = ps_queue->pt_top;
        ps_queue->pt_top = pt_wk_elm->ps_next;
        // デキューした要素を廃棄
        sts_mdl_delete_u8_array(pt_wk_elm->pv_value);
        l_mem_free(pt_wk_elm);
        ps_queue->u16_count--;
    }
    // 末尾をクリア
    ps_queue->pt_tail = NULL;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
