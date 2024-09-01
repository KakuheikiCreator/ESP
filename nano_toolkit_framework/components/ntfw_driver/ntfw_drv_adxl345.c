/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :ADXL345 Driver functions source file
 *
 * CREATED:2019/12/06 04:16:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:ADXL345 3-AXIS SENSOR driver
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
#include "ntfw_drv_adxl345.h"

#include <string.h>
#include "ntfw_com_value_util.h"
#include "ntfw_com_date_time.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// レジスタアドレス（Read用）
#define ADXL345_CALIBRATION_CNT        (10)
#define ADXL345_READ_START             (0x1D)
#define ADXL345_READ_LENGTH            (29)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
// 有効アドレスチェック
static bool b_valid_address(ts_i2c_address_t s_address);
// レジスタの読み込み
static esp_err_t sts_read_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data);
// レジスタへの書き込み（セグメント０）
static esp_err_t sts_write_seg_0(ts_i2c_address_t s_address, ts_adxl345_register_t* ps_adxl345_register);
// レジスタへの書き込み（セグメント１）
static esp_err_t sts_write_seg_1(ts_i2c_address_t s_address, ts_adxl345_register_t* ps_adxl345_register);
// レジスタへの書き込み
static esp_err_t sts_write_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t u8_data);
// レジスタへの書き込み
static esp_err_t sts_write(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data, uint8_t u8_len);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/*****************************************************************************
 *
 * NAME: sts_adxl345_init
 *
 * DESCRIPTION:センサー初期化処理
 *
 *   Output(Hz) Bandwidth(Hz) 消費電力(uA) サンプリングレート
 *   0.10       0.05           23          0000
 *   0.20       0.10           23          0001
 *   0.39       0.20           23          0010
 *   0.78       0.39           23          0011
 *   1.56       0.78           34          0100
 *   3.13       1.56           40          0101
 *   6.25       3.13           45          0110
 *   12.5       6.25           50          0111
 *     25       12.5           60          1000
 *     50         25           90          1001
 *    100         50          140          1010
 *    200        100          140          1011
 *    400        200          140          1100
 *    800        400          140          1101
 *   1600        800           90          1110
 *   3200       1600          140          1111
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t        u8_rate         R   サンプリングレート
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 *****************************************************************************/
esp_err_t sts_adxl345_init(ts_i2c_address_t s_address, uint8_t u8_rate) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 初期処理
    //==========================================================================
    // 初期状態の読み込み
    ts_adxl345_register_t s_register;
    sts_val = sts_adxl345_read(s_address, &s_register);
    if (sts_val == ESP_OK) {
        // デフォルト値の編集
        v_adxl345_edit_default(&s_register);
        // 省電力・データレートコントロール
        s_register.u8_bw_rate = (u8_rate & 0x0F);
        // レジスタ書き込み
        sts_val = sts_adxl345_write(s_address, &s_register);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:v_adxl345_edit_default
 *
 * DESCRIPTION:デフォルト値編集
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_adxl345_register* ps_register     W   編集対象
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 *****************************************************************************/
void v_adxl345_edit_default(ts_adxl345_register_t* ps_register) {
    // 初期クリア
    memset(ps_register, 0, sizeof(ts_adxl345_register_t));
    // 書き込み用レジスタ情報１編集
    ps_register->u8_tap_thresh   = 0xFF;	// タップ閾値（16G）
    ps_register->i8_offset_x     = 0;		// オフセット値（X軸）
    ps_register->i8_offset_y     = 0;		// オフセット値（Y軸）
    ps_register->i8_offset_z     = 0;		// オフセット値（Z軸）
    ps_register->u8_act_thresh   = 0xFF;	// アクティブ閾値（16G）
    ps_register->u8_inact_thresh = 0xFF;	// インアクティブ閾値（16G）
    ps_register->u8_ff_thresh    = 0xFF;	// 自由落下加速度閾値（16G）
    ps_register->u8_ff_time      = 0xFF;	// 自由落下時間閾値（1275ms）
    // 書き込み用レジスタ情報２編集
    ps_register->u8_bw_rate      = 0x0E;	// 省電力・データレートコントロール（1600Hz）
    ps_register->u8_power_crl    = 0x08;	// リンク・スリープ無効、測定モード
    ps_register->u8_data_format  = 0x0B;	// デフォルトフォーマット、±16G、最大分解能モード
    ps_register->u8_fifo_ctl     = 0x1F;	// _fifo_無効、キューサイズ閾値=31
}

/*****************************************************************************
 *
 * NAME: sts_adxl345_read
 *
 * DESCRIPTION:レジスタ情報読み込み
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_adxl345_register* ps_register     W   編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_read(ts_i2c_address_t s_address,
                           ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // レジスタ情報読み込み
    //==========================================================================
    do {
        // 書き込み開始
        sts_val = sts_io_i2c_mst_start_write(s_address);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタアドレス書き込み
        uint8_t u8_reg_address = ADXL345_READ_START;
        sts_val = sts_io_i2c_mst_write(&u8_reg_address, 1, true);
        if (sts_val != ESP_OK) {
            break;
        }
        // 読み込み開始
        sts_val = sts_io_i2c_mst_start_read(s_address);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ読み込み
        sts_val = sts_io_i2c_mst_read_stop((uint8_t*)ps_register, ADXL345_READ_LENGTH);
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME: sts_adxl345_read_g
 *
 * DESCRIPTION:加速度（XYZ軸）読み込み
 *
 * PARAMETERS:             Name            RW  Usage
 *   ts_i2c_address        s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_adxl345_axes_data* ps_axes_data    W   編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_read_g(ts_i2c_address_t s_address, ts_adxl345_axes_data_t* ps_axes_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 加速度（XYZ軸）読み込み
    //==========================================================================
    do {
        // 書き込み開始
        sts_val = sts_io_i2c_mst_start_write(s_address);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタアドレス書き込み
        uint8_t u8_val = 0x32;
        sts_val = sts_io_i2c_mst_write(&u8_val, 1, true);
        if (sts_val != ESP_OK) {
            break;
        }
        // 読み込み開始
        sts_val = sts_io_i2c_mst_start_read(s_address);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ読み込み
        uint8_t u8_data[6];
        sts_val = sts_io_i2c_mst_read_stop(u8_data, 6);
        if (sts_val != ESP_OK) {
            break;
        }
        // 結果編集
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        ps_axes_data->i16_data_x = u_conv.i16_values[0];
        u_conv.u8_values[0] = u8_data[3];
        u_conv.u8_values[1] = u8_data[2];
        ps_axes_data->i16_data_y = u_conv.i16_values[0];
        u_conv.u8_values[0] = u8_data[5];
        u_conv.u8_values[1] = u8_data[4];
        ps_axes_data->i16_data_z = u_conv.i16_values[0];
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_write
 *
 * DESCRIPTION:編集値書き込み
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_adxl345_register* ps_register   R   レジスタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_write(ts_i2c_address_t s_address, ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 編集値書き込み
    //==========================================================================
    do {
        // レジスタセグメント０の書き込み
        sts_val = sts_write_seg_0(s_address, ps_register);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタセグメント１の書き込み
        sts_val = sts_write_seg_1(s_address, ps_register);
        if (sts_val != ESP_OK) {
            break;
        }
        // データフォーマットの書き込み判定
        sts_val = sts_write_byte(s_address, 0x31, ps_register->u8_data_format);
        if (sts_val != ESP_OK) {
            break;
        }
        // FIFOコントロールの書き込み判定
        sts_val = sts_write_byte(s_address, 0x38, ps_register->u8_fifo_ctl);
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME: sts_adxl345_zeroing
 *
 * DESCRIPTION:ゼロイング処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   ts_i2c_address   s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_zeroing(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ゼロイング処理
    //==========================================================================
    do {
        //----------------------------------------------------------------------
        // レジスタ読み込み
        //----------------------------------------------------------------------
        ts_adxl345_register_t s_register;
        esp_err_t sts_val = sts_adxl345_read(s_address, &s_register);
        if (sts_val != ESP_OK) {
            break;
        }
        //----------------------------------------------------------------------
        // 較正モードに設定更新
        //----------------------------------------------------------------------
        // オフセットクリア
        s_register.i8_offset_x = 0;
        s_register.i8_offset_y = 0;
        s_register.i8_offset_z = 0;
        sts_val = sts_write_seg_0(s_address, &s_register);
        if (sts_val != ESP_OK) {
            break;
        }
        // BW_RATE：電力モード（通常）・サンプリングレート更新（200Hz）
        sts_val = sts_write_byte(s_address, 0x2C, 0x0B);
        if (sts_val != ESP_OK) {
            break;
        }
        // DATA_FORMAT：セルフテストモード、最大分解能モード、レンジ16G
        sts_val = sts_write_byte(s_address, 0x31, 0x8B);
        if (sts_val != ESP_OK) {
            break;
        }
        //----------------------------------------------------------------------
        // ５サンプルの読み飛ばし
        //----------------------------------------------------------------------
        ts_adxl345_axes_data_t s_axes_data;
        uint8_t idx;
        for (idx = 0; idx < 5; idx++) {
            // 一定間隔で取得
            i64_dtm_delay_msec(10);
            // サンプル値の読み込み
            sts_val = sts_adxl345_read_g(s_address, &s_axes_data);
            if (sts_val != ESP_OK) {
                break;
            }
        }
        //----------------------------------------------------------------------
        // １０サンプルの平均値を取得
        //----------------------------------------------------------------------
        int32_t i32_sum_x = 0;
        int32_t i32_sum_y = 0;
        int32_t i32_sum_z = 0;
        for (idx = 0; idx < ADXL345_CALIBRATION_CNT; idx++) {
            // 一定間隔で取得
            i64_dtm_delay_msec(10);
            // サンプル値の読み込み
            sts_val = sts_adxl345_read_g(s_address, &s_axes_data);
            if (sts_val != ESP_OK) {
                break;
            }
            i32_sum_x += s_axes_data.i16_data_x;
            i32_sum_y += s_axes_data.i16_data_y;
            i32_sum_z += s_axes_data.i16_data_z;
        }
        //----------------------------------------------------------------------
        // オフセット値の算出
        // １０回計測した加速度（3.9mg/LSB）の平均値→オフセット値（15.6mg/LSB）
        //----------------------------------------------------------------------
        // オフセットX軸（較正値）
        s_register.i8_offset_x = (i32_sum_x / ADXL345_CALIBRATION_CNT) / -4;
        // オフセットY軸（較正値）
        s_register.i8_offset_y = (i32_sum_y / ADXL345_CALIBRATION_CNT) / -4;
        // オフセットZ軸（較正値）
        s_register.i8_offset_z = (i32_sum_z / ADXL345_CALIBRATION_CNT) / -4;
        //----------------------------------------------------------------------
        // レジスタ書き込み
        //----------------------------------------------------------------------
        // セグメント０更新
        sts_val = sts_write_seg_0(s_address, &s_register);
        if (sts_val != ESP_OK) {
            break;
        }
        // BW_RATE
        sts_val = sts_write_byte(s_address, 0x2C, s_register.u8_bw_rate);
        if (sts_val != ESP_OK) {
            break;
        }
        // DATA_FORMAT
        sts_val = sts_write_byte(s_address, 0x31, s_register.u8_data_format);
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:s_adxl345_g_data
 *
 * DESCRIPTION:加速度取得（XYZ軸）
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_adxl345_register* ps_register     R   レジスタ
 *
 * RETURNS:
 *   ts_adxl345_axes_data 読み込みした加速度データ構造体（15.6mg/LSB）
 *
 * NOTES:
 * None.
 *****************************************************************************/
ts_adxl345_axes_data_t s_adxl345_g_data(ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 加速度データ
    ts_adxl345_axes_data_t s_axes_data = {
        .i16_data_x = 0,
        .i16_data_y = 0,
        .i16_data_z = 0,
    };
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return s_axes_data;
    }

    //==========================================================================
    // 加速度取得（XYZ軸）
    //==========================================================================
    // 加速度編集
    tu_type_converter_t u_conv;
    // 加速度（X軸）
    u_conv.u8_values[0] = ps_register->u8_data_x1;
    u_conv.u8_values[1] = ps_register->u8_data_x0;
    s_axes_data.i16_data_x = u_conv.i16_values[0];
    // 加速度（Y軸）
    u_conv.u8_values[0] = ps_register->u8_data_y1;
    u_conv.u8_values[1] = ps_register->u8_data_y0;
    s_axes_data.i16_data_y = u_conv.i16_values[0];
    // 加速度（Z軸）
    u_conv.u8_values[0] = ps_register->u8_data_z1;
    u_conv.u8_values[1] = ps_register->u8_data_z0;
    s_axes_data.i16_data_z = u_conv.i16_values[0];
    // 変換データ返却
    return s_axes_data;
}

/*****************************************************************************
 *
 * NAME:s_adxl345_act_status
 *
 * DESCRIPTION:アクティブイベントステータス取得
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_adxl345_register* ps_register     R   レジスタ
 *
 * RETURNS:
 *   ts_adxl345_axests_val アクティブイベントステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
ts_adxl345_axes_sts_t s_adxl345_act_status(ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // アクティブイベント
    ts_adxl345_axes_sts_t s_status = {
        .b_status_x = false,
        .b_status_y = false,
        .b_status_z = false
    };
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return s_status;
    }

    //==========================================================================
    // アクティブイベントステータス取得
    //==========================================================================
    // ステータス編集
    s_status.b_status_x = (ps_register->u8_act_tap_sts >> 6) & 0x01;
    s_status.b_status_y = (ps_register->u8_act_tap_sts >> 5) & 0x01;
    s_status.b_status_z = (ps_register->u8_act_tap_sts >> 4) & 0x01;
    // 編集値を返却
    return s_status;
}

/*****************************************************************************
 *
 * NAME:u8ADXL345_getTapStatus
 *
 * DESCRIPTION:タップイベントステータス取得
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_adxl345_register* ps_register     R   レジスタ
 *
 * RETURNS:
 *   ts_adxl345_axests_val タップイベントステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
ts_adxl345_axes_sts_t s_adxl345_tap_status(ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // アクティブイベント
    ts_adxl345_axes_sts_t s_status = {
        .b_status_x = false,
        .b_status_y = false,
        .b_status_z = false
    };
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return s_status;
    }

    //==========================================================================
    // タップイベントステータス取得
    //==========================================================================
    // ステータス編集
    s_status.b_status_x = (ps_register->u8_act_tap_sts >> 2) & 0x01;
    s_status.b_status_y = (ps_register->u8_act_tap_sts >> 1) & 0x01;
    s_status.b_status_z = ps_register->u8_act_tap_sts & 0x01;
    // 編集値を返却
    return s_status;
}

/*****************************************************************************
 *
 * NAME:sADXL345_getIntStatus
 *
 * DESCRIPTION:割り込みステータス取得
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_adxl345_register* ps_register     R   レジスタ
 *
 * RETURNS:
 * ts_adxl345_interrupt_sts  割り込みイベントステータス構造体
 *
 * NOTES:
 * None.
 *****************************************************************************/
ts_adxl345_interrupt_sts_t s_adxl345_int_status(ts_adxl345_register_t* ps_register) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 割り込みステータス
    ts_adxl345_interrupt_sts_t s_status = {
        .b_sts_data_ready  = false,
        .b_sts_single_tap  = false,
        .b_sts_double_tap  = false,
        .b_sts_activity    = false,
        .b_sts_in_activity = false,
        .b_sts_free_fall   = false,
        .b_sts_watermark   = false,
        .b_sts_overrun     = false,
    };
    // 編集対象レジスタ情報
    if (ps_register == NULL) {
        return s_status;
    }

    //==========================================================================
    // 割り込みステータス取得
    //==========================================================================
    // ステータス編集
    s_status.b_sts_data_ready  = (ps_register->u8_int_source >> 7);
    s_status.b_sts_single_tap  = (ps_register->u8_int_source >> 6) & 0x01;
    s_status.b_sts_double_tap  = (ps_register->u8_int_source >> 5) & 0x01;
    s_status.b_sts_activity    = (ps_register->u8_int_source >> 4) & 0x01;
    s_status.b_sts_in_activity = (ps_register->u8_int_source >> 3) & 0x01;
    s_status.b_sts_free_fall   = (ps_register->u8_int_source >> 2) & 0x01;
    s_status.b_sts_watermark   = (ps_register->u8_int_source >> 1) & 0x01;
    s_status.b_sts_overrun     = (ps_register->u8_int_source) & 0x01;
    // 編集値を返却
    return s_status;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_offset
 *
 * DESCRIPTION:オフセット編集
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   int8_t         i8_ofs_x        R   オフセット（X軸：15.6 mg/LSB）
 *   int8_t         i8_ofs_y        R   オフセット（Y軸：15.6 mg/LSB）
 *   int8_t         i8_ofs_z        R   オフセット（Z軸：15.6 mg/LSB）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_offset(ts_i2c_address_t s_address, int8_t i8_ofs_x, int8_t i8_ofs_y, int8_t i8_ofs_z) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // オフセット編集
    //==========================================================================
    // 書き込み開始
    sts_val = sts_io_i2c_mst_start_write(s_address);
    if (sts_val == ESP_OK) {
        // レジスタアドレスとデータを書き込み
        int8_t i8_offset[4] = {0x1E, i8_ofs_x, i8_ofs_y, i8_ofs_z};
        sts_val = sts_io_i2c_mst_write_stop((uint8_t*)i8_offset, 4, true);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_bw_rate
 *
 * DESCRIPTION:電力モード・データレート
 *
 *   Output(Hz) Bandwidth(Hz) 消費電力(uA) サンプリングレート
 *   0.10       0.05           23          0000
 *   0.20       0.10           23          0001
 *   0.39       0.20           23          0010
 *   0.78       0.39           23          0011
 *   1.56       0.78           34          0100
 *   3.13       1.56           40          0101
 *   6.25       3.13           45          0110
 *   12.5       6.25           50          0111
 *     25       12.5           60          1000
 *     50         25           90          1001
 *    100         50          140          1010
 *    200        100          140          1011
 *    400        200          140          1100
 *    800        400          140          1101
 *   1600        800           90          1110
 *   3200       1600          140          1111
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_low_pwr       R   省電力モードビット
 *   uint8_t              u8_rate         R   サンプリングレート
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_bw_rate(ts_i2c_address_t s_address, bool b_low_pwr, uint8_t u8_rate) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 電力モード・データレート
    //==========================================================================
    uint8_t u8_bw_rate = (uint8_t)((b_low_pwr << 4) | (u8_rate & 0x0F));
    sts_val = sts_write_byte(s_address, 0x31, u8_bw_rate);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}


/*****************************************************************************
 *
 * NAME:sts_adxl345_set_data_format
 *
 * DESCRIPTION:出力フォーマット（Gレンジ、精度、左右寄せ、割り込み）
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_adxl345_range     e_range       R   Gレンジ（0～3:2,4,8,16）
 *   bool                 b_full_res    R   最大分解能モード（true:最大分解能）
 *   bool                 b_justify     R   左右寄せ（true:左寄せ）
 *   bool                 b_int_inv     R   割り込み出力（true:アクティブ・ハイ）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_data_format(ts_i2c_address_t s_address,
                                      te_adxl345_range_t e_range,
                                      bool b_full_res, bool b_justify, bool b_int_inv) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 出力フォーマット（Gレンジ、精度、左右寄せ、割り込み）
    //==========================================================================
    uint8_t u8_fmt;
    sts_val = sts_read_byte(s_address, 0x2C, &u8_fmt);
    if (sts_val == ESP_OK) {
        u8_fmt = u8_fmt | (b_int_inv << 5);
        u8_fmt = u8_fmt | (b_full_res << 3);
        u8_fmt = u8_fmt | (b_justify << 2);
        u8_fmt = u8_fmt & (e_range | 0xFC);
        sts_val = sts_write_byte(s_address, 0x2C, u8_fmt);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_data_format
 *
 * DESCRIPTION:出力制御（セルフテスト、SPI出力モード）
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_self_test   R   セルフテスト（true:有効）
 *   bool                 b_spi_mode    R   SPIモード（true:3線式）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_output_ctl(ts_i2c_address_t s_address, bool b_self_test, bool b_spi_mode) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 出力制御（セルフテスト、SPI出力モード）
    //==========================================================================
    uint8_t u8_fmt;
    sts_val = sts_read_byte(s_address, 0x2C, &u8_fmt);
    if (sts_val == ESP_OK) {
        u8_fmt = u8_fmt | (b_self_test << 7);
        u8_fmt = u8_fmt | (b_spi_mode << 6);
        sts_val = sts_write_byte(s_address, 0x2C, u8_fmt);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_sleep
 *
 * DESCRIPTION:スリープ（オートスリープ、スリープ、スリープ時周波数）
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_auto_sleep  R   オートスリープ（true:オートスリープ有効）
 *   bool                 b_sleep       R   スリープ（true:スリープモード）
 *   uint8_t              u8_sleep_rate R   スリープ時サンプリングレート（0～3:8,4,2,1）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_sleep(ts_i2c_address_t s_address, bool b_auto_sleep, bool b_sleep, uint8_t u8_sleep_rate) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // スリープ（オートスリープ、スリープ、スリープ時周波数）
    //==========================================================================
    uint8_t u8_power_crl;
    sts_val = sts_read_byte(s_address, 0x2D, &u8_power_crl);
    if (sts_val == ESP_OK) {
        u8_power_crl = u8_power_crl | (b_auto_sleep << 4);
        u8_power_crl = u8_power_crl | (b_sleep << 2);
        u8_power_crl = u8_power_crl & (u8_sleep_rate | 0xFC);
        sts_val = sts_write_byte(s_address, 0x2D, u8_power_crl);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_measure
 *
 * DESCRIPTION:計測モード（スタンバイ、アクティブ・インアクティブリンク）
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_address         s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                   b_measure       R   Measureビット
 *   bool                   b_link          R   Linkビット
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_measure(ts_i2c_address_t s_address,
                                  bool b_measure, bool b_link) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 計測モード（スタンバイ、アクティブ・インアクティブリンク）
    //==========================================================================
    uint8_t u8_power_crl;
    sts_val = sts_read_byte(s_address, 0x2D, &u8_power_crl);
    if (sts_val == ESP_OK) {
        u8_power_crl = u8_power_crl | (b_measure << 3);
        u8_power_crl = u8_power_crl | (b_link << 5);
        sts_val = sts_write_byte(s_address, 0x2D, u8_power_crl);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:u8_adxl345_edt_fifo_ctl
 *
 * DESCRIPTION:FIFO設定（モード、トリガ出力先、プールサイズ閾値）
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_adxl345_mode      e_mode        R   FIFOモード
 *   bool                 b_trigger     R   イベントトリガーのリンク先（true:INT2）
 *   uint8_t              u8_samples    R   Samplesビット
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_fifo_ctl(ts_i2c_address_t s_address,
                                   te_adxl345_mode_t e_mode,
                                   bool b_trigger, uint8_t u8_samples) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // FIFO設定（モード、トリガ出力先、プールサイズ閾値）
    //==========================================================================
    uint8_t u8_fifo_crl = (uint8_t)(e_mode << 6);
    u8_fifo_crl = u8_fifo_crl | (b_trigger << 5);
    u8_fifo_crl = u8_fifo_crl | u8_samples;
    sts_val = sts_write_byte(s_address, 0x38, u8_fifo_crl);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_int_enable
 *
 * DESCRIPTION:有効割り込み編集（タップ・アクティブ・自由落下等）
 *
 * PARAMETERS:                 Name         RW  Usage
 *   ts_i2c_address            s_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_adxl345_interrupt_sts  s_status     R   割り込み有効ステータス構造体
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_int_enable(ts_i2c_address_t s_address, ts_adxl345_interrupt_sts_t s_status) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 有効割り込み編集（タップ・アクティブ・自由落下等）
    //==========================================================================
    uint8_t u8_int_enable;
    u8_int_enable = (s_status.b_sts_data_ready << 7);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_single_tap & 0x01) << 6);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_double_tap & 0x01) << 5);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_activity & 0x01) << 4);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_in_activity & 0x01) << 3);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_free_fall & 0x01) << 2);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_watermark & 0x01) << 1);
    u8_int_enable = u8_int_enable | (s_status.b_sts_overrun & 0x01);
    sts_val = sts_write_byte(s_address, 0x2E, u8_int_enable);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_int_map
 *
 * DESCRIPTION:割り込み出力先編集（タップ・アクティブ・自由落下等）
 *
 * PARAMETERS:                 Name         RW  Usage
 *   ts_i2c_address            s_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_adxl345_interrupt_sts  s_status     R   割り込み先ステータス構造体
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_int_map(ts_i2c_address_t s_address, ts_adxl345_interrupt_sts_t s_status) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 割り込み出力先編集（タップ・アクティブ・自由落下等）
    //==========================================================================
    uint8_t u8_int_enable;
    u8_int_enable = (s_status.b_sts_data_ready << 7);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_single_tap & 0x01) << 6);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_double_tap & 0x01) << 5);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_activity & 0x01) << 4);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_in_activity & 0x01) << 3);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_free_fall & 0x01) << 2);
    u8_int_enable = u8_int_enable | ((s_status.b_sts_watermark & 0x01) << 1);
    u8_int_enable = u8_int_enable | (s_status.b_sts_overrun & 0x01);
    sts_val = sts_write_byte(s_address, 0x2E, u8_int_enable);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_tap_threshold
 *
 * DESCRIPTION:タップ閾値編集（加速度、継続時間）
 *
 * PARAMETERS:           Name         RW  Usage
 *   ts_i2c_address      s_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t             u8_threshold R   タップ加速度閾値（62.5 mg/LSB）
 *   uint8_t             u8_duration  R   タップ継続時間閾値（625 μs/LSB）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_tap_threshold(ts_i2c_address_t s_address, uint8_t u8_threshold, uint8_t u8_duration) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // タップ閾値編集（加速度、継続時間）
    //==========================================================================
    sts_val = sts_write_byte(s_address, 0x1D, u8_threshold);
    if (sts_val == ESP_OK) {
        sts_val = sts_write_byte(s_address, 0x21, u8_duration);
    }

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_dbl_tap_threshold
 *
 * DESCRIPTION:ダブルタップ閾値編集（間隔、測定期間）
 *
 * PARAMETERS:       Name         RW  Usage
 *   ts_i2c_address  s_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t         u8_latent    R   ダブルタップ間隔閾値（1.25ms/LSB）
 *   uint8_t         u8_window    R   ダブルタップ測定期間閾値（1.25ms/LSB）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_dbl_tap_threshold(ts_i2c_address_t s_address, uint8_t u8_latent, uint8_t u8_window) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ダブルタップ閾値編集（間隔、測定期間）
    //==========================================================================
    uint8_t u8_data[2] = {u8_latent, u8_window};
    sts_val = sts_write(s_address, 0x22, u8_data, 2);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_tap_axes
 *
 * DESCRIPTION:タップ間のタップ有効無効、タップ有効軸
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address           s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 * bool                     b_suppress      R   タップ間のタップ有効無効
 * ts_adxl345_axests_val    s_axests_val    R   タップ有効軸
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_tap_axes(ts_i2c_address_t s_address, bool b_suppress, ts_adxl345_axes_sts_t s_axests_val) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // タップ間のタップ有効無効、タップ有効軸
    //==========================================================================
    uint8_t u8_tap_axes = 0x00;
    u8_tap_axes = u8_tap_axes | (b_suppress << 3);
    u8_tap_axes = u8_tap_axes | (s_axests_val.b_status_x << 2);
    u8_tap_axes = u8_tap_axes | (s_axests_val.b_status_y << 1);
    u8_tap_axes = u8_tap_axes | s_axests_val.b_status_z;
    sts_val = sts_write_byte(s_address, 0x21, u8_tap_axes);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_edt_active_ctl
 *
 * DESCRIPTION:アクティブ制御編集（加速度、絶対／相対、有効軸）
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address           s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 * uint8_t                  u8_act_th       R   アクティブ加速度閾値（62.5 mg/LSB）
 * bool                     b_acdc          R   アクティブACDC（true:AC）
 * ts_adxl345_axests_val    s_axests_val    R   アクティブ判定有効軸
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_active_ctl(ts_i2c_address_t s_address,
                                     uint8_t u8_act_th,
                                     bool b_acdc,
                                     ts_adxl345_axes_sts_t s_axests_val) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // アクティブ制御編集（加速度、絶対／相対、有効軸）
    //==========================================================================
    do {
        // アクティブ加速度閾値
        sts_val = sts_write_byte(s_address, 0x24, u8_act_th);
        if (sts_val != ESP_OK) {
            break;
        }
        // 更新判定（有効軸等）
        uint8_t u8_act_inact_ctl;
        sts_val = sts_read_byte(s_address, 0x27, &u8_act_inact_ctl);
        if (sts_val != ESP_OK) {
            break;
        }
        u8_act_inact_ctl = u8_act_inact_ctl | ((b_acdc & 0x01) << 3);
        u8_act_inact_ctl = u8_act_inact_ctl | ((s_axests_val.b_status_x & 0x01) << 2);
        u8_act_inact_ctl = u8_act_inact_ctl | ((s_axests_val.b_status_y & 0x01) << 1);
        u8_act_inact_ctl = u8_act_inact_ctl | (s_axests_val.b_status_z & 0x01);
        sts_val = sts_write_byte(s_address, 0x27, u8_act_inact_ctl);
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_edt_in_active_ctl
 *
 * DESCRIPTION:インアクティブ制御編集（加速度、継続時間、絶対／相対、有効軸）
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_address         s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t                u8_inact_th     R   インアクティブ加速度閾値（62.5 mg/LSB）
 *   uint8_t                u8_inact_time   R   インアクティブ時間閾値（1 sec/LSB）
 *   bool                   b_acdc          R   インアクティブACDC
 *   ts_adxl345_axests_val  s_axests_val    R   インアクティブ判定有効軸
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_edt_in_active_ctl(ts_i2c_address_t s_address,
                                        uint8_t u8_inact_th,
                                        uint8_t u8_inact_time,
                                        bool b_acdc,
                                        ts_adxl345_axes_sts_t s_axests_val) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // インアクティブ制御編集（加速度、継続時間、絶対／相対、有効軸）
    //==========================================================================
    do {
        // 書き込み：インアクティブ加速度閾値、インアクティブ継続時間閾値
        uint8_t u8_data[2] = {u8_inact_th, u8_inact_time};
        sts_val = sts_write(s_address, 0x24, u8_data, 2);
        if (sts_val != ESP_OK) {
            break;
        }
        // 書き込み：有効軸等
        uint8_t u8_inact_ctl;
        sts_val = sts_read_byte(s_address, 0x27, &u8_inact_ctl);
        if (sts_val != ESP_OK) {
            break;
        }
        u8_inact_ctl = u8_inact_ctl | (b_acdc << 3);
        u8_inact_ctl = u8_inact_ctl | (s_axests_val.b_status_x << 2);
        u8_inact_ctl = u8_inact_ctl | (s_axests_val.b_status_y << 1);
        u8_inact_ctl = u8_inact_ctl | s_axests_val.b_status_z;
        sts_val = sts_write_byte(s_address, 0x27, u8_inact_ctl);
    } while(false);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:sts_adxl345_set_free_fall
 *
 * DESCRIPTION:自由落下閾値編集（加速度、継続時間）
 *
 * PARAMETERS:      Name          RW  Usage
 *   ts_i2c_address s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t        u8_thresh_ff  R   自由落下加速度閾値（62.5 mg/LSB）
 *   uint8_t        u8_time_ff    R   自由落下時間閾値（5 ms/LSB）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
esp_err_t sts_adxl345_set_free_fall(ts_i2c_address_t s_address,
                                    uint8_t u8_thresh_ff,
                                    uint8_t u8_time_ff) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // トランザクション開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 自由落下閾値編集（加速度、継続時間）
    //==========================================================================
    // 書き込み：自由落下加速度閾値、自由落下継続時間閾値
    uint8_t u8_data[2] = {u8_thresh_ff, u8_time_ff};
    sts_val = sts_write(s_address, 0x28, u8_data, 2);

    //==========================================================================
    // トランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*****************************************************************************
 *
 * NAME:i16_adxl345_conv_g_val
 *
 * DESCRIPTION:
 *   加速度の算出処理、これだけの為に浮動小数点演算するのは煩雑なので
 *   整数値計算で概算
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_adxl345_axes_data s_axes_data     R   加速度データ
 * bool                 b_round_up      R   小数点以下切り上げフラグ
 *
 * RETURNS:
 *      int16_t:概算した３軸の加速度の合成値
 *
 * NOTES:
 * None.
 *****************************************************************************/
int16_t i16_adxl345_conv_g_val(ts_adxl345_axes_data_t *s_axes_data, bool b_round_up) {
    // 加速度の二乗値（三軸の加速度を合成）を取得
    int32_t i32_gx = s_axes_data->i16_data_x;
    int32_t i32_gy = s_axes_data->i16_data_y;
    int32_t i32_gz = s_axes_data->i16_data_z;
    uint64_t u64_gpow = i32_gx * i32_gx + i32_gy * i32_gy + i32_gz * i32_gz;
    // 概算値（加速度 = √u64Gpow）を返す
    return (int16_t)u64_vutil_sqrt(u64_gpow, b_round_up);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/*****************************************************************************
 *
 * NAME: b_valid_address
 *
 * DESCRIPTION:有効アドレスチェック
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス
 *
 * RETURNS:
 *   true:有効なI2Cアドレス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static bool b_valid_address(ts_i2c_address_t s_address) {
    // ポート番号
    if (!b_io_i2c_mst_valid_port(s_address.e_port_no)) {
        return false;
    }
    // アドレス
    return (s_address.u16_address == I2C_ADDR_ADXL345_L ||
             s_address.u16_address == I2C_ADDR_ADXL345_H);
}

/*****************************************************************************
 *
 * NAME: sts_read_byte
 *
 * DESCRIPTION:デバイスから１バイト読み込み
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス
 *   uint8_t        u8_reg_address  R   レジスタアドレス
 *   uint8_t*       pu8_data        R   読み込みデータポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_read_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data) {
    // レジスタアドレスの書き込み開始
    esp_err_t sts_val = sts_io_i2c_mst_start_write(s_address);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // レジスタアドレス書き込み
    sts_val = sts_io_i2c_mst_write(&u8_reg_address, 1, true);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // 読み込み開始
    sts_val = sts_io_i2c_mst_start_read(s_address);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // データ読み込み
    return sts_io_i2c_mst_read_stop(pu8_data, 1);
}

/*****************************************************************************
 *
 * NAME: sts_write_seg_0
 *
 * DESCRIPTION:レジスタへの書き込み（セグメント０）
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス
 *   ts_adxl345_register* ps_register     R   レジスタアドレス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_write_seg_0(ts_i2c_address_t s_address,
                                  ts_adxl345_register_t* ps_register) {
    return sts_write(s_address, 0x1D, (uint8_t*)&ps_register->u8_tap_thresh, 14);
}

/*****************************************************************************
 *
 * NAME: sts_write_seg_1
 *
 * DESCRIPTION:レジスタへの書き込み（セグメント１）
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス
 *   ts_adxl345_register* ps_register     R   レジスタアドレス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_write_seg_1(ts_i2c_address_t s_address,
                                  ts_adxl345_register_t* ps_register) {
    return sts_write(s_address, 0x2C, (uint8_t*)&ps_register->u8_bw_rate, 4);
}

/*****************************************************************************
 *
 * NAME: sts_write_byte
 *
 * DESCRIPTION:デバイスへのデータの書き込み
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス
 *   uint8_t        u8_reg_address  R   レジスタアドレス
 *   uint8_t        u8_data         R   書き込みデータ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_write_byte(ts_i2c_address_t s_address,
                                 uint8_t u8_reg_address,
                                 uint8_t u8_data) {
    // 書き込みデータ編集
    uint8_t u8_tx_data[] = {u8_reg_address, u8_data};
    // 書き込み開始
    esp_err_t sts_val = sts_io_i2c_mst_start_write(s_address);
    if (sts_val != ESP_OK) {
       return sts_val;
    }
    // データ書き込み
    return sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
}

/*****************************************************************************
 *
 * NAME: sts_write
 *
 * DESCRIPTION:レジスタへの書き込み
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス
 *   uint8_t        u8_reg_address  R   レジスタアドレス
 *   uint8_t*       pu8_data        R   書き込みデータ
 *   uint8_t        u8_len          R   書き込みサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_write(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data, uint8_t u8_len) {
    // 書き込みデータの編集
    uint8_t u8_tx_data[u8_len + 1];
    u8_tx_data[0] = u8_reg_address;
    memcpy(&u8_tx_data[1], &pu8_data[0], u8_len);
    // 書き込み開始
    esp_err_t sts_val = sts_io_i2c_mst_start_write(s_address);
    if (sts_val != ESP_OK) {
       return sts_val;
    }
    // データ書き込み
    return sts_io_i2c_mst_write_stop(u8_tx_data, u8_len + 1, I2C_MASTER_ACK);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
