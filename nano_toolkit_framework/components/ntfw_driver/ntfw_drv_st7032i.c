/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :ST7032I LCD Driver functions source file
 *
 * CREATED:2020/03/28 12:07:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:ST7032I LCD I2C draiver
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
/***        Include files                                                   ***/
/******************************************************************************/
#include "ntfw_drv_st7032i.h"

#include <string.h>
#include <esp_timer.h>
#include "ntfw_com_date_time.h"

/******************************************************************************/
/***        Macro Definitions                                               ***/
/******************************************************************************/
/** Block Time */
#ifndef ST7032I_BLOCK_TIME
    // デフォルト値は無制限にウェイト
    #define ST7032I_BLOCK_TIME  (portMAX_DELAY)
#endif

// Address Read/Write
#define I2C_ADDR_ST7032I        (0x3E)

// Control byte
#define ST7032I_RS_CMD               (0x00)
#define ST7032I_RS_DATA              (0x40)

// Instruction commands
#define ST7032I_CMD_CLEAR_DISP       (0x01)
#define ST7032I_CMD_RETURN_HOME      (0x02)
#define ST7032I_CMD_ENTRY_MODE_DEF   (0x06)
#define ST7032I_CMD_DISP_CNTR_DEF    (0x08)
#define ST7032I_CMD_CURSOR_SHIFT_L   (0x10)
#define ST7032I_CMD_CURSOR_SHIFT_R   (0x14)
#define ST7032I_CMD_OSC_FREQ         (0x14)
#define ST7032I_CMD_DISP_SHIFT_L     (0x18)
#define ST7032I_CMD_DISP_SHIFT_R     (0x1C)
#define ST7032I_CMD_FUNC_SET_DEF     (0x38)
#define ST7032I_CMD_FUNC_SET_EX      (0x39)
#define ST7032I_CMD_SET_CG_ADDR      (0x40)
#define ST7032I_CMD_SET_DD_ADDR      (0x80)
#define ST7032I_CMD_SET_ICON_ADDR    (0x40)
#define ST7032I_CMD_DISP_CNTR_EX     (0x50)
#define ST7032I_CMD_FOLLOWER_CNTR    (0x6C)
#define ST7032I_CMD_CONTRAST_LOW     (0x70)
#define ST7032I_CMD_DATA_WRITE       (0x80)

/******************************************************************************/
/***        Type Definitions                                                ***/
/******************************************************************************/
/**
 * 構造体：LCDの状態情報
 */
typedef struct {
    // I2Cアドレス
    ts_i2c_address_t s_address;
    // コントラスト
    uint8_t u8_contrast;
    // アイコン表示
    bool b_icon_disp_flg;
    // 次回コマンド実行可能時刻
    int64_t i64_next_exec;
} ts_st7032i_state_t;

/******************************************************************************/
/***        Exported Variables                                              ***/
/******************************************************************************/

/******************************************************************************/
/***        Local Variables                                                 ***/
/******************************************************************************/
/** ST7032i状態情報 */
static ts_st7032i_state_t s_state[I2C_NUM_MAX] = {};

/******************************************************************************/
/***        Local Function Prototypes                                       ***/
/******************************************************************************/
// コマンドの送信
static esp_err_t sts_write_cmd(ts_st7032i_state_t* ps_state, uint8_t u8_cmd);
// データの送信
static esp_err_t sts_write_data(ts_st7032i_state_t* ps_state, uint8_t u8_data);
// データの送信
static esp_err_t sts_write_data_list(ts_st7032i_state_t* ps_state, uint8_t* pu8_data, size_t t_len);
// 画面制御設定（アイコン表示、コントラスト）
static esp_err_t sts_write_control_ex(ts_st7032i_state_t* ps_state);
// 次回コマンド実行可能時刻（usec）取得
static int64_t i64_next_exec_time(uint8_t u8_cmd);

/******************************************************************************/
/***        Exported Functions                                              ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_st7032i_init
 *
 * DESCRIPTION:LCDの初期化処理
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_init(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cポート
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // 初期化処理シーケンス
    //==========================================================================
    do {
        // 初期処理
        ts_st7032i_state_t* ps_state = &s_state[e_port_num];
        // アドレス生成
        ts_i2c_address_t s_address;
        s_address.e_port_no   = e_port_num;
        s_address.u16_address = I2C_ADDR_ST7032I;
        // LCD状態の初期化
        ps_state->s_address       = s_address;
        ps_state->u8_contrast     = 40;
        ps_state->b_icon_disp_flg = true;
        ps_state->i64_next_exec   = 0;
        // Function Set Default(IS=1)
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_DEF | 0x01);
        if (sts_result != ESP_OK) {
            break;
        }
        // Internal OSC frequency
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_OSC_FREQ);
        if (sts_result != ESP_OK) {
            break;
        }
        // Display Contrast Lower set
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_CONTRAST_LOW | 0x08);
        if (sts_result != ESP_OK) {
            break;
        }
        // Power/ICON Control/Contrast Higher set
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_DISP_CNTR_EX | 0x0E);
        if (sts_result != ESP_OK) {
            break;
        }
        // Follower Control
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FOLLOWER_CNTR);
        if (sts_result != ESP_OK) {
            break;
        }
        // Function Set Default(IS=0)
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_DEF);
        if (sts_result != ESP_OK) {
            break;
        }
        // Display Switch On
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_DISP_CNTR_DEF | 0x04);
        if (sts_result != ESP_OK) {
            break;
        }
        // Clear Screen
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_CLEAR_DISP);
    } while (false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_disp_control
 *
 * DESCRIPTION:画面制御設定（ディスプレイ表示、カーソル表示、アイコン表示）
 *
 * PARAMETERS:              Name            RW  Usage
 *   i2c_port_t             e_port_num      R   I2Cポート番号
 *   te_st7032i_disp_sts_t  e_disp_sts      R   画面表示制御ステータス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_disp_control(i2c_port_t e_port_num, te_st7032i_disp_sts_t e_disp_sts) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // pre processing
    //==========================================================================
    // コマンドの編集
    uint8_t u8_cntr_cmd = ST7032I_CMD_DISP_CNTR_DEF;
    // 画面表示設定
    if ((e_disp_sts & DRV_ST7032I_DISP_ON) != 0x00) {
        u8_cntr_cmd |= 0x04;
    }
    // カーソル表示設定
    if ((e_disp_sts & DRV_ST7032I_DISP_CURSOR) != 0x00) {
        u8_cntr_cmd |= 0x02;
    }
    // カーソル点滅表示設定
    if ((e_disp_sts & DRV_ST7032I_DISP_BLINK) != 0x00) {
        u8_cntr_cmd |= 0x01;
    }
    // アイコン表示
    ts_st7032i_state_t* ps_state = &s_state[e_port_num];
    ps_state->b_icon_disp_flg = ((e_disp_sts & DRV_ST7032I_DISP_ICON) != 0x00);

    //==========================================================================
    // コマンドの送信
    //==========================================================================
    do {
        // 画面制御設定（ディスプレイON/OFF、カーソル表示、カーソル点滅）
        esp_err_t sts_result = sts_write_cmd(ps_state, u8_cntr_cmd);
        if (sts_result != ESP_OK) {
            return sts_result;
        }
        // アイコン表示設定
        sts_result = sts_write_control_ex(ps_state);
    } while (false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_set_contrast
 *
 * DESCRIPTION:コントラスト設定
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *   uint8_t            u8_contrast     R   画面コントラスト値（0-63）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_set_contrast(i2c_port_t e_port_num, uint8_t u8_contrast) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // コントラスト
    if (u8_contrast >= 64) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // コマンドの送信
    //==========================================================================
    // コントラスト設定
    ts_st7032i_state_t* ps_state = &s_state[e_port_num];
    ps_state->u8_contrast = u8_contrast;
    // コントラスト表示設定
    sts_result = sts_write_control_ex(ps_state);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータスを返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_clear_screen
 *
 * DESCRIPTION:LCDをクリア
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   LCDの状態情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_clear_screen(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // Display Clear
    //==========================================================================
    sts_result = sts_write_cmd(&s_state[e_port_num], ST7032I_CMD_CLEAR_DISP);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_clear_icon
 *
 * DESCRIPTION:アイコン表示をクリア
 *
 * PARAMETERS:      Name            RW  Usage
 *   i2c_port_t     e_port_num      R   ポート番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_clear_icon(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // Clear ICON
    //==========================================================================
    esp_err_t sts_result;
    uint8_t u8_reg_addr = 0;
    do {
        sts_result = sts_st7032i_write_icon(e_port_num, u8_reg_addr, 0x00);
        u8_reg_addr++;
    } while(u8_reg_addr < ST7032I_ICON_DATA_SIZE && sts_result == ESP_OK);

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_set_cursor
 *
 * DESCRIPTION:カーソル移動（行、列）
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *   uint8_t            u8_row_no       R   移動先行
 *   uint8_t            u8_col_no       R   移動先列
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_set_cursor(i2c_port_t e_port_num, uint8_t u8_row_no, uint8_t u8_col_no) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 行番号、列番号
    if (u8_row_no >= 2 || u8_col_no >= 0x40) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    //==========================================================================
    // カーソル設定
    //==========================================================================
    // アドレス計算
    uint8_t u8_cmd = ST7032I_CMD_SET_DD_ADDR | (0x40 * u8_row_no + u8_col_no);
    // カーソル移動（行、列）
    sts_result = sts_write_cmd(&s_state[e_port_num], u8_cmd);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 実行結果の返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_return_home
 *
 * DESCRIPTION:カーソル移動（先頭）
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_return_home(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // 画面制御設定（ディスプレイON/OFF、カーソル表示、カーソル点滅）
    //==========================================================================
    sts_result = sts_write_cmd(&s_state[e_port_num], ST7032I_CMD_RETURN_HOME);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_cursor_shift_l
 *
 * DESCRIPTION:カーソル移動（左）
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_cursor_shift_l(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // カーソルシフト（左）
    //==========================================================================
    sts_result = sts_write_cmd(&s_state[e_port_num], ST7032I_CMD_CURSOR_SHIFT_L);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_cursor_shift_r
 *
 * DESCRIPTION:カーソル移動（右）
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_cursor_shift_r(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // カーソルシフト（左）
    //==========================================================================
    sts_result = sts_write_cmd(&s_state[e_port_num], ST7032I_CMD_CURSOR_SHIFT_R);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/******************************************************++***********************
 *
 * NAME: sts_st7032i_char_regist
 *
 * DESCRIPTION:外字登録処理
 *
 * PARAMETERS:      Name                RW  Usage
 *   i2c_port_t     e_port_num          R   I2Cポート番号
 *   uint8_t        u8_ch               R   Character Code
 *   uint8*         pu8_cg_data_list    R   CGData 8 Byte
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_char_regist(i2c_port_t e_port_num, uint8_t u8_ch, uint8_t* pu8_cg_data_list) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    do {
        // CGアドレスの書き込み
        sts_result = sts_write_cmd(&s_state[e_port_num], ST7032I_CMD_SET_CG_ADDR | (u8_ch << 3));
        if (sts_result != ESP_OK) {
            break;
        }
        // データの書き込み
        sts_result = sts_write_data_list(&s_state[e_port_num], pu8_cg_data_list, 8);
        if (sts_result != ESP_OK) {
            break;
        }
    } while(false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_write_char
 *
 * DESCRIPTION:文字表示
 *
 * PARAMETERS:              Name            RW  Usage
 *   i2c_port_t             e_port_num      R   I2Cポート番号
 *   char                   c_ch            R   表示文字
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_write_char(i2c_port_t e_port_num, char c_ch) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    // 文字データの書き込み処理
    sts_result = sts_write_data(&s_state[e_port_num], c_ch);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 実行結果の返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_write_string
 *
 * DESCRIPTION:文字列表示
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *   char*              pc_str          R   表示文字列
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_write_string(i2c_port_t e_port_num, char* pc_str) {
    //==========================================================================
    // 書き込み処理
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // 文字列データの書き込み
    //==========================================================================
    sts_result = sts_write_data_list(&s_state[e_port_num], (uint8_t*)pc_str, strlen(pc_str));

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_st7032i_write_icon
 *
 * DESCRIPTION:アイコン表示制御
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_port_t         e_port_num      R   I2Cポート番号
 *   uint8_t            u8_reg_addr     R   レジスタアドレス
 *   uint8_t            u8_data         R   表示データ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_st7032i_write_icon(i2c_port_t e_port_num, uint8_t u8_reg_addr, uint8_t u8_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポートチェック
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return sts_result;
    }

    //==========================================================================
    // アイコンデータの書き込み処理
    //==========================================================================
    do {
        // ステータス取得
        ts_st7032i_state_t* ps_state = &s_state[e_port_num];
        // コマンドの送信（IS=1）
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_EX);
        if (sts_result != ESP_OK) {
            break;
        }
        // コマンドの送信（アイコンアドレス）
        uint8_t valid_add = u8_reg_addr & 0x0F;
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_SET_ICON_ADDR | valid_add);
        if (sts_result != ESP_OK) {
            break;
        }
        // コマンドの送信（IS=0）
        sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_DEF);
        if (sts_result != ESP_OK) {
            break;
        }
        // データの送信
        sts_result = sts_write_data(ps_state, u8_data);
    } while(false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_result;
}

/******************************************************************************/
/***        Local Functions                                                 ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_write_cmd
 *
 * DESCRIPTION:コマンドの書き込み（継続コマンド有り）
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_st7032i_state_t*    ps_state        R   ST7032Iステータス
 *   uint8_t                u8_cmd          R   コマンド
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_write_cmd(ts_st7032i_state_t* ps_state, uint8_t u8_cmd) {
    // コマンド実行が可能になるまで待つ
    i64_dtm_delay_until_usec(ps_state->i64_next_exec);
    // スタートコンディション
    esp_err_t sts_result = sts_io_i2c_mst_start_write(ps_state->s_address);
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    // コマンドの送信
    uint8_t u8_tx_data[] = {ST7032I_RS_CMD, u8_cmd};
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    // 次回コマンド実行可能時刻を更新
    ps_state->i64_next_exec = i64_next_exec_time(u8_cmd);
    // 完了ステータス返信
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: sts_write_data
 *
 * DESCRIPTION:データの書き込み
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_st7032i_state_t*    ps_state        R   ST7032Iステータス
 *   uint8_t                u8_data         R   データ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_write_data(ts_st7032i_state_t* ps_state, uint8_t u8_data) {
    // 完了ステータス返信
    uint8_t u8_data_list[] = {u8_data};
    return sts_write_data_list(ps_state, u8_data_list, 1);
}

/*******************************************************************************
 *
 * NAME: sts_write_data_list
 *
 * DESCRIPTION:データリストの書き込み
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_st7032i_state_t*    ps_state        R   ST7032Iステータス
 *   uint8_t*               pu8_data        R   データ
 *   size_t                 t_len           R   データサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_write_data_list(ts_st7032i_state_t* ps_state, uint8_t* pu8_data, size_t t_len) {
    // データ編集
    size_t t_list_size = t_len + 1;
    uint8_t u8_tx_data[t_list_size];
    u8_tx_data[0] = ST7032I_RS_DATA;
    memcpy(&u8_tx_data[1], pu8_data, t_len);
    // コマンド実行が可能になるまで待つ
    i64_dtm_delay_until_usec(ps_state->i64_next_exec);
    // スタートコンディション
    esp_err_t sts_result = sts_io_i2c_mst_start_write(ps_state->s_address);
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    // 制御バイトとデータの送信
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, t_list_size, true);
    // 次回コマンド実行可能時刻を更新
    ps_state->i64_next_exec = i64_next_exec_time(ST7032I_CMD_DATA_WRITE);
    // 完了ステータス返信
    return sts_result;
}

/*******************************************************************************
 *
 * NAME: b_write_control_ex
 *
 * DESCRIPTION:画面制御設定（アイコン表示、コントラスト）
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_st7032i_state_t*    ps_state        R   ST7032Iステータス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_write_control_ex(ts_st7032i_state_t* ps_state) {
    //==========================================================================
    // コマンド編集
    //==========================================================================
    // コマンド：アイコン表示・コントラスト上位桁
    uint8_t cmd_disp = ST7032I_CMD_DISP_CNTR_EX;
    // アイコン表示
    cmd_disp |= (0x08 * ps_state->b_icon_disp_flg);
    // booster circuit
    cmd_disp |= 0x04;
    // コントラスト上位桁
    cmd_disp |= ((ps_state->u8_contrast >> 4) & 0x03);
    // コマンド：コントラスト下位桁
    uint8_t cmd_contrast = ST7032I_CMD_CONTRAST_LOW | (ps_state->u8_contrast & 0x0f);

    //==========================================================================
    // コマンド書き込み
    //==========================================================================
    // コマンドの送信（IS=1）
    esp_err_t sts_result = sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_EX);
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    // コマンドの送信（アイコン表示・コントラスト上位桁）
    sts_result = sts_write_cmd(ps_state, cmd_disp);
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    // コマンドの送信（コントラスト下位桁）
    sts_result = sts_write_cmd(ps_state, cmd_contrast);
    if (sts_result != ESP_OK) {
        return sts_result;
    }
    // コマンドの送信（IS=0）
    return sts_write_cmd(ps_state, ST7032I_CMD_FUNC_SET_DEF);
}

/*******************************************************************************
 *
 * NAME: i64_next_exec_time
 *
 * DESCRIPTION:次回のコマンド実行可能時刻を取得する
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint8_t        u8_cmd          R   コマンド
 *
 * RETURNS:
 *   int64_t:次回処理実行時間（マイクロ秒）
 *
 * NOTES:
 * None.
 ******************************************************************************/
static int64_t i64_next_exec_time(uint8_t u8_cmd) {
    // デバイスの次回実行可能時刻を設定
    int64_t i64_add_usec;
    switch(u8_cmd) {
        case ST7032I_CMD_CLEAR_DISP:
        case ST7032I_CMD_RETURN_HOME:
            // 1.08ミリ秒待つ
            i64_add_usec = 1080;
            break;
        default:
            // 27マイクロ秒待つ
            i64_add_usec = 27;
    }
    // 現在時刻からコマンド実行後の時刻を設定
    return esp_timer_get_time() + i64_add_usec;
}

/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/
