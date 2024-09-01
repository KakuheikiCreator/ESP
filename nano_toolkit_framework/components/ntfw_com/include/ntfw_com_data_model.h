/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :header file for types and functions related to the data model
 *
 * CREATED:2020/08/03 03:58:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:データモデルに関する型と関数群
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
#ifndef  __NTFW_COM_DATA_MODEL_H__
#define  __NTFW_COM_DATA_MODEL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <esp_system.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * uint8_t型の配列情報
 */
typedef struct {
    bool b_clone;           // クローンフラグ（動的にメモリ確保：true）
    size_t t_size;          // 値サイズ
    uint8_t* pu8_values;    // 値配列
} ts_u8_array_t;


//==============================================================================
// リンクリストの定義
//==============================================================================
/**
 * リンクリスト要素
 */
typedef struct s_linked_element_t {
    struct s_linked_element_t* ps_prev;
    struct s_linked_element_t* ps_next;
    void* pv_value;
} ts_linked_element_t;

/**
 * リンクリストキュー
 */
typedef struct {
    SemaphoreHandle_t s_mutex;      // ミューテックス
    ts_linked_element_t* pt_top;    // 先頭要素
    ts_linked_element_t* pt_tail;   // 末尾要素
    uint16_t u16_count;             // 要素数
    size_t t_size;                  // データ長
} ts_linked_queue_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/

//==============================================================================
// バイト配列型の処理
//==============================================================================
/** バイト配列情報構造体の生成処理（引数の配列を直接参照） */
extern ts_u8_array_t* ps_mdl_create_u8_array(uint8_t* pu8_data, size_t t_size);
/** バイト配列情報構造体の生成処理（引数の配列のクローンを参照） */
extern ts_u8_array_t* ps_mdl_clone_u8_array(const uint8_t* pu8_data, size_t t_size);
/** 空のバイト配列情報構造体の生成処理 */
extern ts_u8_array_t* ps_mdl_empty_u8_array(size_t t_size);
/** 乱数のバイト配列情報構造体の生成処理 */
extern ts_u8_array_t* ps_mdl_random_u8_array(size_t t_size);
/** 動的に確保されたバイト配列情報構造体の解放処理 */
extern esp_err_t sts_mdl_delete_u8_array(ts_u8_array_t* ps_array);

//==============================================================================
// キュー型の処理
//==============================================================================
/** 動的に確保されたバイト配列データのキューイング */
extern BaseType_t sts_mdl_queue_send(QueueHandle_t x_queue, uint8_t* pu8_list, uint32_t u32_size, TickType_t x_wait);
/** 動的に確保されたバイト配列データのキューイング */
extern BaseType_t sts_mdl_queue_send_array(QueueHandle_t x_queue, const ts_u8_array_t* ps_array, TickType_t x_wait);

//==============================================================================
// リンクリストキューの処理
//==============================================================================
/** リンクリストキューの生成処理 */
extern ts_linked_queue_t* ps_mdl_create_linked_queue();
/** リンクリストキューの削除処理 */
extern esp_err_t sts_mdl_delete_linked_queue(ts_linked_queue_t* ps_queue);
/** リンクリストキューのクリア処理 */
extern esp_err_t sts_mdl_clear_linked_queue(ts_linked_queue_t* ps_queue);
/** リンクリストキューへのエンキュー */
extern esp_err_t sts_mdl_linked_enqueue(ts_linked_queue_t* ps_queue, const uint8_t* pu8_list, size_t t_size);
/** リンクリストキューからデキュー */
extern ts_u8_array_t* ps_mdl_linked_dequeue(ts_linked_queue_t* ps_queue, size_t t_size);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_COM_DATA_MODEL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
