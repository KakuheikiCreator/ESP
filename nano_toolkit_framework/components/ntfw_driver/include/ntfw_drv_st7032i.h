/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :ST7032I LCD Driver functions header file
 *
 * CREATED:2020/03/28 01:07:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:ST7032I LCD I2C draiver
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
#ifndef  __NTFW_DRV_ST7032I_H__
#define  __NTFW_DRV_ST7032I_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***        Include files                                                   ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_system.h>
#include <esp_err.h>
#include "ntfw_io_i2c_master.h"

/******************************************************************************/
/***        Macro Definitions                                               ***/
/******************************************************************************/
// Icon data Size
#define ST7032I_ICON_DATA_SIZE  (16)

/******************************************************************************/
/***        Type Definitions                                                ***/
/******************************************************************************/
/** 画面表示制御 */
typedef enum {
    DRV_ST7032I_DISP_NONE     = 0x00,
    DRV_ST7032I_DISP_ON       = 0x01,
    DRV_ST7032I_DISP_CURSOR   = 0x02,
    DRV_ST7032I_DISP_BLINK    = 0x04,
    DRV_ST7032I_DISP_ICON     = 0x08,
    DRV_ST7032I_DISP_MAX      = 0x10
} te_st7032i_disp_sts_t;

/******************************************************************************/
/***        Exported Variables                                              ***/
/******************************************************************************/

/******************************************************************************/
/***        Local Variables                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***        Local Function Prototypes                                       ***/
/******************************************************************************/

/******************************************************************************/
/***        Exported Functions                                              ***/
/******************************************************************************/
// 初期化
extern esp_err_t sts_st7032i_init(i2c_port_t e_port_num);
// 画面制御設定（ディスプレイON/OFF、カーソル表示、カーソル点滅、アイコン表示）
extern esp_err_t sts_st7032i_disp_control(i2c_port_t e_port_num, te_st7032i_disp_sts_t e_disp_sts);
// コントラスト設定
extern esp_err_t sts_st7032i_set_contrast(i2c_port_t e_port_num, uint8_t u8_contrast);
// スクリーンクリア
extern esp_err_t sts_st7032i_clear_screen(i2c_port_t e_port_num);
// アイコンクリア
extern esp_err_t sts_st7032i_clear_icon(i2c_port_t e_port_num);
// カーソル移動（行、列）
extern esp_err_t sts_st7032i_set_cursor(i2c_port_t e_port_num, uint8_t u8_row_no, uint8_t u8_col_no);
// カーソル移動（ホームへ）
extern esp_err_t sts_st7032i_return_home(i2c_port_t e_port_num);
// カーソル移動（左）
extern esp_err_t sts_st7032i_cursor_shift_l(i2c_port_t e_port_num);
// カーソル移動（右）
extern esp_err_t sts_st7032i_cursor_shift_r(i2c_port_t e_port_num);
// 外字登録
extern esp_err_t sts_st7032i_char_regist(i2c_port_t e_port_num, uint8_t u8_ch, uint8_t *pu8_cg_data_list);
// 文字表示
extern esp_err_t sts_st7032i_write_char(i2c_port_t e_port_num, char c_ch);
// 文字列表示
extern esp_err_t sts_st7032i_write_string(i2c_port_t e_port_num, char* pc_str);
// アイコン表示制御
extern esp_err_t sts_st7032i_write_icon(i2c_port_t e_port_num, uint8_t u8_reg_addr, uint8_t u8_data);

/******************************************************************************/
/***        Local Functions                                                 ***/
/******************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_DRV_ST7032I_H__ */

/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/
