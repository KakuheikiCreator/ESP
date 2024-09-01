/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :debug utility functions header file
 *
 * CREATED:2019/09/29 23:02:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:デバッグユーティリティ関係の関数群
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

#ifndef __NTFW_COM_DEBUG_UTIL_H__
#define __NTFW_COM_DEBUG_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "ntfw_com_value_util.h"
#include "ntfw_com_date_time.h"

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
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/
//==============================================================================
// チェックメソッド
//==============================================================================
/** 日付判定チェック */
extern bool b_dbg_valid_date(ts_date_t s_date, const int i_year, const int i_month, const int i_day);
/** File Open Check */
extern bool b_dbg_disp_open_file(const char* pc_path);

//==============================================================================
// 情報表示関数
//==============================================================================
/** メモリ確保エラーの情報表示有効化 */
extern void v_dbg_register_failed_alloc();
/** ヒープメモリ情報の表示 */
extern void v_dbg_disp_heap_info(const char* pc_pref);
/** スタック情報表示 */
extern void v_dbg_disp_stack_info(const char* pc_pref);
/** バイト配列表示 */
extern void v_dbg_disp_hex_data(const char* pc_pref, const uint8_t* pu8_data, uint32_t u32_len);
/** Display File List */
extern void v_dbg_file_list(char* pc_path);
/** ファイル情報 */
extern void v_dbg_file_info(const char* pc_path);

#ifdef __cplusplus
}
#endif

#endif /* __NTFW_COM_DEBUG_UTIL_H__ */
