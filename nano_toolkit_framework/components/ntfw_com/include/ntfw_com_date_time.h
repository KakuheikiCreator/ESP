/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common date time library header file
 *
 * CREATED:2019/10/14 18:15:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:日付・時刻に関する共通ライブラリ
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
#ifndef  __NTFW_COM_DATE_TIME_H__
#define  __NTFW_COM_DATE_TIME_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Days in 400 years(Gregorian calendar) */
#define DT_UTIL_DAYS_400YEARS   (146097)
/** Days in 100 years(Gregorian calendar) */
#define DT_UTIL_DAYS_100YEARS   (36524)
/** Days in 4 years(Gregorian calendar) */
#define DT_UTIL_DAYS_4YEARS     (1461)

/** 遅延の閾値 */
#define DT_UTIL_DELAY_THRESHOLD_MS  (1000 / CONFIG_FREERTOS_HZ)

/** ティックカウント（ミリ秒単位） */
#define xTaskGetTickCountMSec() (xTaskGetTickCount() * portTICK_PERIOD_MS)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 構造体：日付保持 */
typedef struct {
    int i_year;
    int i_month;
    int i_day;
} ts_date_t;

/** 構造体：時刻保持 */
typedef struct {
    int i_hour;
    int i_minutes;
    int i_seconds;
} ts_time_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/

//==============================================================================
// ウェイト関数
//==============================================================================
/** ディレイ（ミリ秒単位） */
extern int64_t i64_dtm_delay_msec(int64_t i64_msec);
/** ディレイ（マイクロ秒単位） */
extern int64_t i64_dtm_delay_usec(int64_t i64_usec);
/** 指定時刻までディレイ（ミリ秒単位） */
extern int64_t i64_dtm_delay_until_msec(int64_t i64_msec);
/** 指定時刻までディレイ（マイクロ秒単位） */
extern int64_t i64_dtm_delay_until_usec(int64_t i64_usec);
/** ビジーウェイト処理（ミリ秒単位） */
extern int64_t i64_dtm_wait_msec(int64_t i64_msec);
/** ビジーウェイト処理（マイクロ秒単位） */
extern int64_t i64_dtm_wait_usec(int64_t i64_usec);

//==============================================================================
// チェック関数
//==============================================================================
/** 日付チェック */
extern bool b_dtm_valid_date(int i_year, int i_month, int i_day);
/** 時刻チェック */
extern bool b_dtm_valid_time(int i_hour, int i_min, int i_sec);
/** 時チェック */
extern bool b_dtm_valid_hour(int i_hour);
/** 分チェック */
extern bool b_dtm_valid_min(int i_min);
/** 秒チェック */
extern bool b_dtm_valid_sec(int i_sec);
/** うるう年チェック */
extern bool b_dtm_is_leap_year(int i_year);

//==============================================================================
// 変換関数
//==============================================================================
/** 変換関数：紀元1月1日からの経過日数への変換 */
extern int i_dtm_date_to_days(int i_year, uint8_t u8_month, uint8_t u8_day);
/** 変換関数：紀元1月1日からの経過日数から日付への変換 */
extern ts_date_t s_dtm_day_to_date(int i_days);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_COM_DATE_TIME_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
