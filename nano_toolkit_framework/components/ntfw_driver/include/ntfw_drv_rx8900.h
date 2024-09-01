/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :RX8900 RTC Driver functions header file
 *
 * CREATED:2019/11/24 21:40:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:RX8900のドライバ関数群
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
#ifndef  __NTFW_DRV_RX8900_H__
#define  __NTFW_DRV_RX8900_H__

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
#include <string.h>
#include <string.h>
#include "ntfw_io_i2c_master.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** レジスタアドレス */
typedef enum {
    DRV_RX8900_SUNDAY             = 0x01,
    DRV_RX8900_MONDAY             = 0x02,
    DRV_RX8900_TUESDAY            = 0x04,
    DRV_RX8900_WEDNESDAY          = 0x08,
    DRV_RX8900_THURSDAY           = 0x10,
    DRV_RX8900_FRIDAY             = 0x20,
    DRV_RX8900_SATURDAY           = 0x40
} te_rx8900_day_of_week_t;

/** 構造体：RX8900日付時刻 */
typedef struct {
    uint8_t u8_sec;         // 秒
    uint8_t u8_min;         // 分
    uint8_t u8_hour;        // 時
    uint8_t u8_week;        // 週
    uint8_t u8_day;         // 日
    uint8_t u8_month;       // 月
    uint8_t u8_year;        // 年
} ts_rx8900_datetime_t;

/** 構造体：RX8900レジスタ */
typedef struct {
    ts_rx8900_datetime_t ts_datetime;             // 日付時刻
    uint8_t u8_ram;                               // RAM
    uint8_t u8_alarm_min;                         // アラーム（分）
    uint8_t u8_alarm_hour;                        // アラーム（時）
    uint8_t u8_alarm_day_or_week;                 // アラーム（日）
    uint16_t u16_counter;                         // タイマーカウンター
    uint8_t u8_ex_register;                       // 拡張レジスタ
    uint8_t u8_flg_register;                      // フラグレジスタ
    uint8_t u8_ctr_register;                      // コントロールレジスタ
    uint8_t u8_temperature;                       // 温度
    uint8_t u8_backup_func;                       // バックアップ電源制御
    uint8_t u8_filler[2];                         // 空欄
} ts_rx8900_register_t;


/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/
//==============================================================================
// マスター側機能
//==============================================================================
/** RX8900初期化処理 */
extern esp_err_t sts_rx8900_init(i2c_port_t e_port_num);
/** RX8900リセット処理 */
extern esp_err_t sts_rx8900_reset(i2c_port_t e_port_num);
/** RX8900レジスタ読み込み処理 */
extern ts_rx8900_register_t s_rx8900_read(i2c_port_t e_port_num);
/** RX8900日時レジスタ書き込み処理 */
extern esp_err_t sts_rx8900_write_datetime(i2c_port_t e_port_num, ts_rx8900_datetime_t s_datetime);
/** RX8900アラーム（分）書き込み処理 */
extern esp_err_t sts_rx8900_write_alarm_min(i2c_port_t e_port_num, uint8_t u8_min, bool b_enable);
/** RX8900アラーム（時）書き込み処理 */
extern esp_err_t sts_rx8900_write_alarm_hour(i2c_port_t e_port_num, uint8_t u8_hour, bool b_enable);
/** RX8900アラーム（日）書き込み処理 */
extern esp_err_t sts_rx8900_write_alarm_day(i2c_port_t e_port_num, uint8_t u8_day, bool b_enable);
/** RX8900アラーム（週）書き込み処理 */
extern esp_err_t sts_rx8900_write_alarm_week(i2c_port_t e_port_num, uint8_t u8_week, bool b_enable);
/** RX8900カウンタ書き込み処理 */
extern esp_err_t sts_rx8900_write_counter(i2c_port_t e_port_num, uint16_t u16_cnt);
/** RX8900拡張レジスタ書き込み処理 */
extern esp_err_t sts_rx8900_write_ex(i2c_port_t e_port_num, uint8_t u8_ex);
/** RX8900フラグレジスタ書き込み処理 */
extern esp_err_t sts_rx8900_write_flg(i2c_port_t e_port_num, uint8_t u8_flg);
/** RX8900コントロールレジスタ書き込み処理 */
extern esp_err_t sts_rx8900_write_ctl(i2c_port_t e_port_num, uint8_t u8_ctl);
/** 温度データを100倍した摂氏の値に変換 */
extern float f_rx8900_celsius(uint8_t u8_temperature);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_DRV_RX8900_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
