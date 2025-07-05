/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :LIS3DH Driver functions source file
 *
 * CREATED:2020/01/01 18:02:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:LIS3DH Accelerometer Processing Unit driver
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
#include "ntfw_drv_lis3dh.h"

#include <string.h>
#include <esp_log.h>
#include "ntfw_com_value_util.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * I2C初期処理
 */
typedef esp_err_t (*tf_i2c_init_t)(ts_i2c_mst_address_t* ps_address);

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
// 有効アドレスチェック
static bool b_valid_address(ts_i2c_mst_address_t* ps_address);
/** I2C初期処理 */
static esp_err_t sts_i2c_init(ts_i2c_mst_address_t* ps_address);
/** I2C初期処理（ダミー） */
static esp_err_t sts_i2c_init_dmy(ts_i2c_mst_address_t* ps_address);
/** I2C初期処理 */
static volatile tf_i2c_init_t pf_i2c_init = sts_i2c_init;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/

/******************************************************************************/
/***        Exported Functions                                              ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_lis3dh_who_am_i
 *
 * DESCRIPTION:レジスタ情報読み込み
 *
 * PARAMETERS:          Name        RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_who_am_i(ts_i2c_mst_address_t* ps_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // who am i
    //==========================================================================
    uint8_t u8_reg_address = 0x0F;
    uint8_t u8_data;
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        if (u8_data != 0x33) {
            sts_val = ESP_ERR_NOT_FOUND;
        }
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_rate
 *
 * DESCRIPTION:レート設定
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address*      ps_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_low_pwr     R   省電力モードフラグ
 *   te_lis3dh_data_rate  e_rate        R   レート
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_rate(ts_i2c_mst_address_t* ps_address, bool b_low_pwr, te_lis3dh_data_rate_t e_rate) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // レート設定
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x20;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = (e_rate | (b_low_pwr << 3) | (u8_data & 0x0F));
        // I2Cスレーブへのデータ送信処理
        uint8_t u8_tx_data[] = {0x20, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_enable_axis
 *
 * DESCRIPTION:３軸の有効無効設定
 *
 * PARAMETERS:          Name        RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool               b_x         R   有効無効フラグ（X軸）
 *   bool               b_y         R   有効無効フラグ（Y軸）
 *   bool               b_z         R   有効無効フラグ（Z軸）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_enable_axis(ts_i2c_mst_address_t* ps_address, bool b_x, bool b_y, bool b_z) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ３軸の有効無効設定
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x20;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    // I2Cスレーブへのレジスタアドレスを送信し、データを受信
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = (u8_data & 0xF8) | b_x | (b_y << 1) | (b_z <<2);
        // 制御レジスタの書き込み
        uint8_t u8_tx_data[] = {0x20, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_hpcf
 *
 * DESCRIPTION:ハイパスフィルタ設定
 *
 * PARAMETERS:            Name          RW  Usage
 *   ts_i2c_address*      ps_address    R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_hpf_mode   e_hpf_mode    R   ハイパスフィルターモード
 *   uint8_t              u8_hpcf       R   カットオフ周波数
 *   bool                 b_fds         R   フィルタリング対象セレクタ（true:内部フィルタ出力）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_hpcf(ts_i2c_mst_address_t* ps_address, te_lis3dh_hpf_mode_t e_hpf_mode, uint8_t u8_hpcf, bool b_fds) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ハイパスフィルタ設定
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x21;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    // I2Cスレーブへのレジスタアドレスを送信し、データを受信
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((e_hpf_mode << 6) | ((u8_hpcf << 4) & 0x30)) | ((b_fds << 3) & 0x08) | (u8_data & 0x07);
        // 制御レジスタの書き込み
        uint8_t u8_tx_data[] = {0x21, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_upd_settings
 *
 * DESCRIPTION:データ更新方式設定
 *
 * PARAMETERS:          Name        RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool               b_upd_type  R   更新タイプ(false:continuos update)
 *   bool               b_format    R   フォーマット(false:little endian)
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_upd_settings(ts_i2c_mst_address_t* ps_address, bool b_upd_type, bool b_format) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // データ更新方式設定
    //==========================================================================
    // アドレスレジスタ
    uint8_t u8_reg_address = 0x23;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    // I2Cスレーブへのレジスタアドレスを送信し、データを受信
    return sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((b_upd_type << 6) | ((b_format << 5) & 0x40)) | (u8_data & 0x3F);
        // 制御レジスタの書き込み
        uint8_t u8_tx_data[] = {0x23, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_range
 *
 * DESCRIPTION:レンジ設定
 *
 * PARAMETERS:          Name        RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_range    e_range     R   レンジ
 *   bool               b_hr        R   高解像度モード有効フラグ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_range(ts_i2c_mst_address_t* ps_address, te_lis3dh_range_t e_range, bool b_hr) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // レンジ設定
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x23;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((e_range << 4) & 0x30) | ((b_hr << 3) & 0x08) | (u8_data & 0xC7);
        // 制御レジスタの書き込み
        uint8_t u8_tx_data[] = {0x23, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_fifo_mode
 *
 * DESCRIPTION:FIFOモード
 *
 * PARAMETERS:              Name        RW  Usage
 *   ts_i2c_address*        ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_fifo_mode    e_fifo_mode R   FIFOモード
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_fifo_mode(ts_i2c_mst_address_t* ps_address, te_lis3dh_fifo_mode_t e_fifo_mode) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // FIFOモード
    //==========================================================================
    do {
        // レジスタアドレス
        uint8_t u8_reg_address = 0x24;
        // 制御レジスタ５の読み込み
        uint8_t u8_data;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
        if (sts_val != ESP_OK) {
            break;
        }
        // FIFO有効無効判定
        if (e_fifo_mode != DRV_LIS3DH_MODE_BYPASS) {
            // FIFO有効化
            u8_data = u8_data | 0x40;
        } else {
            // FIFO無効化
            u8_data = u8_data & 0xBF;
        }
        // 制御レジスタの書き込み
        uint8_t u8_tx_data[] = {0x24, u8_data};
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタアドレス
        u8_reg_address = 0x2E;
        // FIFO制御レジスタ読み込み
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタ編集
        u8_data = ((e_fifo_mode << 6) & 0xC0) | (u8_data & 0x3F);
        // 制御レジスタの書き込み
        u8_tx_data[0] = 0x2E;
        u8_tx_data[1] = u8_data;
        sts_val = sts_io_i2c_mst_tx(ps_address, u8_tx_data, 2);
    } while(false);

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_fifo_cnt
 *
 * DESCRIPTION:FIFOカウント
 *
 * PARAMETERS:          Name        RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t            u8_fifo_cnt W   FIFOカウント
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_fifo_cnt(ts_i2c_mst_address_t* ps_address, uint8_t* pu8_fifo_cnt) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // FIFOカウント
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x2F;
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data, 1);
    if (sts_val == ESP_OK) {
        // カウント
        *pu8_fifo_cnt = u8_data & 0x1F;
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_status
 *
 * DESCRIPTION:ステータス
 *
 * PARAMETERS:          Name        \RW  Usage
 *   ts_i2c_address*    ps_address  R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t*           pu8_status  W   ステータス編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_status(ts_i2c_mst_address_t* ps_address, uint8_t* pu8_status) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ステータスレジスタ読み込み
    //==========================================================================
    // レジスタアドレス
    uint8_t u8_reg_address = 0x27;
    // ステータスレジスタの読み込み
    sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, pu8_status, 1);

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_acceleration
 *
 * DESCRIPTION:加速度（XYZ軸）読み込み
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_address*        ps_address      R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_lis3dh_axes_data*   ps_axes_data    W   編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_acceleration(ts_i2c_mst_address_t* ps_address, ts_lis3dh_axes_data_t* ps_axes_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(ps_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2C初期処理
    pf_i2c_init(ps_address);

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_tran_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // 加速度読み込み
    //==========================================================================
    do {
        //----------------------------------------------------------------------
        // 加速度（X軸）読み込み
        //----------------------------------------------------------------------
        // レジスタアドレス
        uint8_t u8_reg_address = 0x28;
        // 加速度（X軸）読み込み
        uint8_t u8_data[2];
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[0], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        u8_reg_address = 0x29;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[1], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[0];
        u_conv.u8_values[1] = u8_data[1];
        ps_axes_data->i16_data_x = u_conv.i16_values[0];

        //----------------------------------------------------------------------
        // 加速度（Y軸）読み込み
        //----------------------------------------------------------------------
        u8_reg_address = 0x2A;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[0], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        u8_reg_address = 0x2B;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[1], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        u_conv.u8_values[0] = u8_data[0];
        u_conv.u8_values[1] = u8_data[1];
        ps_axes_data->i16_data_y = u_conv.i16_values[0];

        //----------------------------------------------------------------------
        // 加速度（Z軸）読み込み
        //----------------------------------------------------------------------
        u8_reg_address = 0x2C;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[0], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        u8_reg_address = 0x2D;
        sts_val = sts_io_i2c_mst_txrx(ps_address, &u8_reg_address, 1, &u8_data[1], 1);
        if (sts_val != ESP_OK) {
            break;
        }
        u_conv.u8_values[0] = u8_data[0];
        u_conv.u8_values[1] = u8_data[1];
        ps_axes_data->i16_data_z = u_conv.i16_values[0];
    } while(false);

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_tran_end();

    // 結果返信
    return sts_val;
}

/******************************************************************************/
/***        Local Functions                                                 ***/
/******************************************************************************/

/*****************************************************************************
 *
 * NAME: b_valid_address
 *
 * DESCRIPTION:有効アドレスチェック
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_mst_address_t*  ps_address      R   I2Cアドレス
 *
 * RETURNS:
 *   true:有効なI2Cアドレス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static bool b_valid_address(ts_i2c_mst_address_t* ps_address) {
    // NULLチェック
    if (ps_address == NULL) {
        return false;
    }
    // アドレス
    return (ps_address->u16_address == I2C_ADDR_LIS3DH_L ||
            ps_address->u16_address == I2C_ADDR_LIS3DH_H);
}

/*****************************************************************************
 *
 * NAME: sts_i2c_init
 *
 * DESCRIPTION:I2C初期処理
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_mst_address_t*  ps_address      R   I2Cアドレス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
  *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_i2c_init(ts_i2c_mst_address_t* ps_address) {
    //==========================================================================
    // I2C初期化処理
    //==========================================================================
    // デバイス追加
    esp_err_t sts_val = sts_io_i2c_mst_add_device(ps_address);
    if (sts_val == ESP_OK) {
        pf_i2c_init = sts_i2c_init_dmy;
    }
    // 結果ステータス返却
    return sts_val;
}

/*****************************************************************************
 *
 * NAME: sts_i2c_init
 *
 * DESCRIPTION:I2C初期処理
 *
 * PARAMETERS:              Name            RW  Usage
 *   ts_i2c_mst_address_t*  ps_address      R   I2Cアドレス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
  *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_i2c_init_dmy(ts_i2c_mst_address_t* ps_address) {
    return ESP_OK;
}

/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/
