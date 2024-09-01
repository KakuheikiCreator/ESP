/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :LIS3DH Driver functions source file
 *
 * CREATED:2020/01/01 18:02:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:LIS3DH Accelerometer Processing Unit driver
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

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
// レジスタの1バイト読み込み
static esp_err_t sts_read_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data);
// レジスタへの書き込み
static esp_err_t sts_write_byte(ts_i2c_address_t s_address, uint8_t u8_address, uint8_t u8_data);

/******************************************************************************/
/***        Exported Functions                                              ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_lis3dh_who_am_i
 *
 * DESCRIPTION:レジスタ情報読み込み
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_who_am_i(ts_i2c_address_t s_address) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // who am i
    //==========================================================================
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x0F, &u8_data);
    if (sts_val == ESP_OK) {
        if (u8_data != 0x33) {
            sts_val = ESP_ERR_NOT_FOUND;
        }
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

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
 *   ts_i2c_address       s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_low_pwr     R   省電力モードフラグ
 *   te_lis3dh_data_rate  e_rate        R   レート
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_rate(ts_i2c_address_t s_address, bool b_low_pwr, te_lis3dh_data_rate_t e_rate) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // レート設定
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x20, &u8_data);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = (e_rate | (b_low_pwr << 3) | (u8_data & 0x0F));
        // 制御レジスタの書き込み
        sts_val = sts_write_byte(s_address, 0x20, u8_data);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_enable_axis
 *
 * DESCRIPTION:３軸の有効無効設定
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_x             R   有効無効フラグ（X軸）
 *   bool                 b_y             R   有効無効フラグ（Y軸）
 *   bool                 b_z             R   有効無効フラグ（Z軸）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_enable_axis(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ３軸の有効無効設定
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x20, &u8_data);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = (u8_data & 0xF8) | b_x | (b_y << 1) | (b_z <<2);
        // 制御レジスタの書き込み
        sts_val =sts_write_byte(s_address, 0x20, u8_data);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_hpcf
 *
 * DESCRIPTION:ハイパスフィルタ設定
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_hpf_mode   e_hpf_mode      R   ハイパスフィルターモード
 *   uint8_t              u8_hpcf         R   カットオフ周波数
 *   bool                 b_fds           R   フィルタリング対象セレクタ（true:内部フィルタ出力）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_hpcf(ts_i2c_address_t s_address, te_lis3dh_hpf_mode_t e_hpf_mode, uint8_t u8_hpcf, bool b_fds) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ハイパスフィルタ設定
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x21, &u8_data);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((e_hpf_mode << 6) | ((u8_hpcf << 4) & 0x30)) | ((b_fds << 3) & 0x08) | (u8_data & 0x07);
        // 制御レジスタの書き込み
        sts_val = sts_write_byte(s_address, 0x21, u8_data);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_upd_settings
 *
 * DESCRIPTION:データ更新方式設定
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool                 b_upd_type      R   更新タイプ(false:continuos update)
 *   bool                 b_format        R   フォーマット(false:little endian)
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_upd_settings(ts_i2c_address_t s_address, bool b_upd_type, bool b_format) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // データ更新方式設定
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x23, &u8_data);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((b_upd_type << 6) | ((b_format << 5) & 0x40)) | (u8_data & 0x3F);
        // 制御レジスタの書き込み
        sts_val = sts_write_byte(s_address, 0x23, u8_data);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_range
 *
 * DESCRIPTION:レンジ設定
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_range      e_range         R   レンジ
 *   bool                 b_hr            R   高解像度モード有効フラグ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_range(ts_i2c_address_t s_address, te_lis3dh_range_t e_range, bool b_hr) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // レンジ設定
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x23, &u8_data);
    if (sts_val == ESP_OK) {
        // レジスタ編集
        u8_data = ((e_range << 4) & 0x30) | ((b_hr << 3) & 0x08) | (u8_data & 0xC7);
        // 制御レジスタの書き込み
        sts_val = sts_write_byte(s_address, 0x23, u8_data);
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_set_fifo_mode
 *
 * DESCRIPTION:FIFOモード
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_lis3dh_fifo_mode  e_fifo_mode     R   FIFOモード
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_set_fifo_mode(ts_i2c_address_t s_address, te_lis3dh_fifo_mode_t e_fifo_mode) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // FIFOモード
    //==========================================================================
    do {
        // 制御レジスタ５の読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x24, &u8_data);
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
        sts_val = sts_write_byte(s_address, 0x24, u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // FIFO制御レジスタ読み込み
        sts_val = sts_read_byte(s_address, 0x2E, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタ編集
        u8_data = ((e_fifo_mode << 6) & 0xC0) | (u8_data & 0x3F);
        // 制御レジスタの書き込み
        sts_val = sts_write_byte(s_address, 0x2E, u8_data);
    } while(false);

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_fifo_cnt
 *
 * DESCRIPTION:FIFOカウント
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t              u8_fifo_cnt     W   FIFOカウント
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_fifo_cnt(ts_i2c_address_t s_address, uint8_t* pu8_fifo_cnt) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // FIFOカウント
    //==========================================================================
    // 制御レジスタ読み込み
    uint8_t u8_data;
    sts_val = sts_read_byte(s_address, 0x2F, &u8_data);
    if (sts_val == ESP_OK) {
        // カウント
        *pu8_fifo_cnt = u8_data & 0x1F;
    }

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_status
 *
 * DESCRIPTION:ステータス
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t*             pu8_status      W   ステータス編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_status(ts_i2c_address_t s_address, uint8_t* pu8_status) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // ステータスレジスタ読み込み
    //==========================================================================
    sts_val = sts_read_byte(s_address, 0x27, pu8_status);

    //==========================================================================
    // I2Cトランザクション終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_lis3dh_acceleration
 *
 * DESCRIPTION:加速度（XYZ軸）読み込み
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_lis3dh_axes_data* ps_axes_data    W   編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_lis3dh_acceleration(ts_i2c_address_t s_address, ts_lis3dh_axes_data_t* ps_axes_data) {
    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_val = sts_io_i2c_mst_begin();
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
        uint8_t u8_data[2];
        sts_val = sts_read_byte(s_address, 0x28, &u8_data[0]);
        if (sts_val != ESP_OK) {
            break;
        }
        sts_val = sts_read_byte(s_address, 0x29, &u8_data[1]);
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
        sts_val = sts_read_byte(s_address, 0x2A, &u8_data[0]);
        if (sts_val != ESP_OK) {
            break;
        }
        sts_val = sts_read_byte(s_address, 0x2B, &u8_data[1]);
        if (sts_val != ESP_OK) {
            break;
        }
        u_conv.u8_values[0] = u8_data[0];
        u_conv.u8_values[1] = u8_data[1];
        ps_axes_data->i16_data_y = u_conv.i16_values[0];

        //----------------------------------------------------------------------
        // 加速度（Z軸）読み込み
        //----------------------------------------------------------------------
        sts_val = sts_read_byte(s_address, 0x2C, &u8_data[0]);
        if (sts_val != ESP_OK) {
            break;
        }
        sts_val = sts_read_byte(s_address, 0x2D, &u8_data[1]);
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
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}
/******************************************************************************/
/***        Local Functions                                                 ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_read_byte
 *
 * DESCRIPTION:デバイスから１バイト読み込み
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス
 *   uint8_t        u8_address      R   レジスタアドレス
 *   uint8_t*       pu8_data        R   読み込みデータポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
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

/*******************************************************************************
 *
 * NAME: sts_write_byte
 *
 * DESCRIPTION:デバイスへのデータの書き込み
 *
 * PARAMETERS:      Name            RW  Usage
 *   ts_i2c_address s_address       R   I2Cアドレス
 *   uint8_t        u8_address      R   レジスタアドレス
 *   uint8_t        u8_data         R   書き込みデータ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_write_byte(ts_i2c_address_t s_address,
                                   uint8_t u8_address,
                                   uint8_t u8_data) {
    // 書き込み開始
    esp_err_t sts_val = sts_io_i2c_mst_start_write(s_address);
    if (sts_val != ESP_OK) {
       return sts_val;
    }
    // レジスタアドレスとデータの書き込み
    uint8_t u8_data_list[2] = {u8_address, u8_data};
    return sts_io_i2c_mst_write_stop(u8_data_list, 2, true);
}

/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/
