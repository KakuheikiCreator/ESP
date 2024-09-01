/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :MPU_6050 Driver functions source file
 *
 * CREATED:2019/12/06 04:16:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:MPU_6050 3-AXIS SENSOR driver
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
#include "ntfw_drv_mpu-6050.h"

#include <string.h>
#include "ntfw_com_value_util.h"
#include "ntfw_com_date_time.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** 待ち時間：クリティカルセクション */
#define EVT_TAKE_WAIT_TICK  (1000 / portTICK_PERIOD_MS)
// レジスタアドレス（Read用）
#define MPU_6050_CALIBRATION_CNT        (5)
#define MPU_6050_READ_START             (0x1D)
#define MPU_6050_READ_LENGTH            (29)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;
// ゼロイング補正値（加速度）
static ts_mpu_6050_axes_data_t s_accel_zeroing_data = {0, 0, 0};
// ゼロイング補正値（ジャイロ）
static ts_mpu_6050_axes_data_t s_gyro_zeroing_data = {0, 0, 0};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
// 有効アドレスチェック
static bool b_valid_address(ts_i2c_address_t s_address);
/** 有効加速度レンジチェック */
static bool b_valid_accel_range(te_mpu_6050_accel_range_t e_accel_range);
/** 有効ジャイロレンジチェック */
static bool b_valid_gyro_range(te_mpu_6050_gyro_range_t e_gyro_range);
// レジスタの1バイト読み込み
static esp_err_t sts_read_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data);
// レジスタの読み込み
static esp_err_t sts_read(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data, uint8_t u8_size);
// レジスタへの書き込み
static esp_err_t sts_write_byte(ts_i2c_address_t s_address, uint8_t u8_address, uint8_t u8_data);

/******************************************************************************/
/***        Exported Functions                                              ***/
/******************************************************************************/
/*******************************************************************************
 *
 * NAME: sts_mpu_6050_init
 *
 * DESCRIPTION:初期化処理
 *
 * PARAMETERS:                  Name            RW  Usage
 * ts_i2c_address_t             s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 * te_mpu_6050_accel_range_t    e_accel_range   R   加速度のレンジ
 * te_mpu_6050_gyro_range_t     e_gyro_range    R   ジャイロのレンジ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_init(ts_i2c_address_t s_address,
                            te_mpu_6050_accel_range_t e_accel_range,
                            te_mpu_6050_gyro_range_t e_gyro_range) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 加速度レンジ
    if (!b_valid_accel_range(e_accel_range)) {
        return ESP_ERR_INVALID_ARG;
    }
    // ジャイロレンジ
    if (!b_valid_gyro_range(e_gyro_range)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 初期化処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        // ミューテックスの初期化
        if (s_mutex == NULL) {
            s_mutex = xSemaphoreCreateRecursiveMutex();
        }
        // デバイスリセット
        sts_val = sts_mpu_6050_device_reset(s_address);
        if (sts_val != ESP_OK) {
            break;
        }
        // 内部オシレータ8MHz
        sts_val = sts_mpu_6050_set_clock(s_address, DRV_MPU_6050_CLK_INTERNAL);
        if (sts_val != ESP_OK) {
            break;
        }
        // 分割数
        sts_val = sts_mpu_6050_set_smplrt_div(s_address, 0x00);
        if (sts_val != ESP_OK) {
            break;
        }
        // ローパスフィルタ
        sts_val = sts_mpu_6050_set_dlpf_cfg(s_address, DRV_MPU_6050_LPF_260_256);
        if (sts_val != ESP_OK) {
            break;
        }
        // ハイパスフィルタ
        sts_val = sts_mpu_6050_set_accel_hpf(s_address, DRV_MPU_6050_ACCEL_HPF_RESET);
        if (sts_val != ESP_OK) {
            break;
        }
        // 設定：加速度レンジ
        sts_val = sts_mpu_6050_set_accel_range(s_address, e_accel_range);
        if (sts_val != ESP_OK) {
            break;
        }
        // 設定：ジャイロレンジ
        sts_val = sts_mpu_6050_set_gyro_range(s_address, e_gyro_range);
        if (sts_val != ESP_OK) {
            break;
        }
        // FIFO無効化
        sts_val = sts_mpu_6050_set_fifo_enable(s_address, false, false, false, false, false);
        if (sts_val != ESP_OK) {
            break;
        }
        // スリープサイクル無効化
        sts_val = sts_mpu_6050_set_sleep_cycle(s_address, DRV_MPU_6050_SLEEP_CYCLE_NONE);
        // ゼロイング補正値をクリア
        v_mpu_6050_zeroing_clear();
    } while(false);

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_smplrt_div
 *
 * DESCRIPTION:ジャイロのサンプリングレート分割数設定処理
 *   Gyroscope Output Rate ＝8KHz(DLPFが有効の場合は1KHz)
 *   サンプルレート = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   uint8_t            u8_div          R   分割数
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_smplrt_div(ts_i2c_address_t s_address, uint8_t u8_div) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 分周数の設定処理
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    // 分周数の設定処理
    sts_val = sts_write_byte(s_address, 0x19, u8_div);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_dlpf_cfg
 *
 * DESCRIPTION:加速度とジャイロのローパスフィルター設定
 *
 * PARAMETERS:                  Name        RW  Usage
 *   ts_i2c_address_t           s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_accel_lpf_t    u8_dlpf_cfg R   サンプリングレート
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_dlpf_cfg(ts_i2c_address_t s_address, te_mpu_6050_accel_lpf_t u8_dlpf_cfg) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ローパスフィルター設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1A, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0xF8) | (u8_dlpf_cfg & 0x07);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1A, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_accel_self_test
 *
 * DESCRIPTION:加速度セルフテスト
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool               b_x             R   セルフテスト有効無効（X軸）
 *   bool               b_y             R   セルフテスト有効無効（Y軸）
 *   bool               b_z             R   セルフテスト有効無効（Z軸）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_accel_self_test(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 加速度セルフテスト
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1C, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0x1F) | (b_x << 7) | ((b_y & 0x01) << 6) | ((b_z & 0x01) << 7);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1C, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_accel_range
 *
 * DESCRIPTION:加速度レンジ設定
 *
 * PARAMETERS:                  Name        RW  Usage
 *   ts_i2c_address_t           s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_accel_range_t  e_range     R   加速度レンジ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_accel_range(ts_i2c_address_t s_address, te_mpu_6050_accel_range_t e_range) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 加速度レンジ
    if (!b_valid_accel_range(e_range)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 加速度レンジ設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1C, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0xE7) | ((e_range & 0x03) << 3);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1C, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_accel_hpf
 *
 * DESCRIPTION:ハイパスフィルタ設定
 *
 * PARAMETERS:                  Name        RW  Usage
 *   ts_i2c_address_t           s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_accel_hpf_t    e_hpf       R   加速度ハイパスフィルタ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_accel_hpf(ts_i2c_address_t s_address, te_mpu_6050_accel_hpf_t e_hpf) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ハイパスフィルタ設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1C, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0xF8) | (e_hpf & 0x07);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1C, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_gyro_self_test
 *
 * DESCRIPTION:ジャイロセルフテスト設定
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool               b_x             R   セルフテスト有効無効（X軸）
 *   bool               b_y             R   セルフテスト有効無効（Y軸）
 *   bool               b_z             R   セルフテスト有効無効（Z軸）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_gyro_self_test(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ジャイロセルフテスト設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1B, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0x1F) | (b_x << 7) | ((b_y & 0x01) << 6) | ((b_z & 0x01) << 7);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1B, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_gyro_range
 *
 * DESCRIPTION:ジャイロレンジ設定
 *
 * PARAMETERS:                  Name          RW  Usage
 *   ts_i2c_address_t           s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_gyro_range_t   e_range       R   角速度レンジ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_gyro_range(ts_i2c_address_t s_address, te_mpu_6050_gyro_range_t e_range) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }
    // ジャイロレンジ
    if (!b_valid_gyro_range(e_range)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ジャイロレンジ設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x1B, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0xE7) | ((e_range & 0x03) << 3);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x1B, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_fifo_enable
 *
 * DESCRIPTION:FIFO有効無効設定
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   bool               b_temp          R   FIFO有効フラグ（温度）
 *   bool               b_x             R   FIFO有効フラグ（X軸）
 *   bool               b_y             R   FIFO有効フラグ（Y軸）
 *   bool               b_z             R   FIFO有効フラグ（Z軸）
 *   bool               b_accel         R   FIFO有効フラグ（加速度）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_fifo_enable(ts_i2c_address_t s_address, bool b_temp, bool b_x, bool b_y, bool b_z, bool b_accel) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // FIFO有効無効設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x23, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        uint8_t u8_fifo = (b_temp << 7) | ((b_x & 0x01) << 6) | ((b_y & 0x01) << 5) | ((b_z & 0x01) << 4) | ((b_accel & 0x01) << 3);
        u8_data = (u8_data & 0x07) | u8_fifo;
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x23, u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタ読み込み
        sts_val = sts_read_byte(s_address, 0x6A, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        if (u8_fifo == 0x00) {
            u8_data = (u8_data & 0xBF) | 0x04;
        } else {
            u8_data = u8_data | 0x44;
        }
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6A, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_clock
 *
 * DESCRIPTION:クロック設定
 *   0：内部オシレータ8MHz
 *   1：ジャイロスコープX軸を参照したPLL出力
 *   2：ジャイロスコープY軸を参照したPLL出力
 *   3：ジャイロスコープZ軸を参照したPLL出力
 *   4：外部参照（32.768kHz）したPLL出力
 *   5：外部参照（19.2MHz）したPLL出力
 *   6：予約済み
 *   7：クロックを停止し、タイミングジェネレーターをリセット状態に保つ
 *
 * PARAMETERS:              Name        RW  Usage
 *   ts_i2c_address_t       s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_clock_t    e_clock     R   クロック設定
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   none.
 *
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_clock(ts_i2c_address_t s_address, te_mpu_6050_clock_t e_clock) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クロック設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x6B, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = (u8_data & 0xF8) | (e_clock & 0x07);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6B, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_set_sleep_cycle
 *
 * DESCRIPTION:スリープサイクル設定
 *
 * PARAMETERS:              Name        RW  Usage
 *   ts_i2c_address_t       s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   te_mpu_6050_cycle_t    e_cycle     R   スリープサイクル設定
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_set_sleep_cycle(ts_i2c_address_t s_address, te_mpu_6050_cycle_t e_cycle) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // スリープサイクル設定
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x6B, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // スリープサイクル判定
        if (e_cycle == DRV_MPU_6050_SLEEP_CYCLE_NONE) {
            u8_data = u8_data & 0x0F;
        } else if (e_cycle == DRV_MPU_6050_SLEEP_CYCLE_SLEEP) {
            u8_data = (u8_data & 0x0F) | 0x40;
        } else {
            u8_data = (u8_data & 0x0F) | 0x20;
        }
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6B, u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // レジスタ読み込み
        sts_val = sts_read_byte(s_address, 0x6C, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // スリープサイクル編集
        u8_data = (e_cycle << 6) | (u8_data & 0x3F);
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6C, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_read_accel
 *
 * DESCRIPTION: 加速度（XYZ軸）読み込み
 *
 * PARAMETERS:                  Name          RW  Usage
 *   ts_i2c_address_t           s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_mpu_6050_axes_data_t    ps_axes_data  W   編集対象の加速度データポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_read_accel(ts_i2c_address_t s_address, ts_mpu_6050_axes_data_t* ps_axes_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ゼロイング補正値
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    // ゼロイング補正値
    int16_t i16_data_x = s_accel_zeroing_data.i16_data_x;
    int16_t i16_data_y = s_accel_zeroing_data.i16_data_y;
    int16_t i16_data_z = s_accel_zeroing_data.i16_data_z;

    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 加速度（XYZ軸）読み込み
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data[6];
        sts_val = sts_read(s_address, 0x3B, u8_data, 6);
        if (sts_val != ESP_OK) {
            break;
        }
        // 加速度編集
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        u_conv.u8_values[2] = u8_data[3];
        u_conv.u8_values[3] = u8_data[2];
        u_conv.u8_values[4] = u8_data[5];
        u_conv.u8_values[5] = u8_data[4];
        ps_axes_data->i16_data_x = u_conv.i16_values[0] - i16_data_x;
        ps_axes_data->i16_data_y = u_conv.i16_values[1] - i16_data_y;
        ps_axes_data->i16_data_z = u_conv.i16_values[2] - i16_data_z;
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_read_celsius
 *
 * DESCRIPTION: 温度（摂氏）読み込み
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   float*                pf_temp       W   編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_read_celsius(ts_i2c_address_t s_address, float* pf_temp) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 温度（摂氏）読み込み
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data[2];
        esp_err_t sts_val = sts_read(s_address, 0x41, u8_data, 2);
        if (sts_val != ESP_OK) {
            break;
        }
        // 温度編集
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        // 340 LSB/degrees and Offset 35 degrees and Difference -521
        // ((temperature + (35 * 340) - 521) / 340.0)
        *pf_temp = f_mpu_6050_celsius(u_conv.i16_values[0]);
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_read_gyro
 *
 * DESCRIPTION: ジャイロ（XYZ軸）読み込み
 *
 * PARAMETERS:                  Name          RW  Usage
 *   ts_i2c_address_t           s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   ts_mpu_6050_axes_data_t    ps_axes_data  W   編集対象の加速度データポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_read_gyro(ts_i2c_address_t s_address, ts_mpu_6050_axes_data_t* ps_axes_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ゼロイング補正値
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    // ゼロイング補正値
    int16_t i16_data_x = s_gyro_zeroing_data.i16_data_x;
    int16_t i16_data_y = s_gyro_zeroing_data.i16_data_y;
    int16_t i16_data_z = s_gyro_zeroing_data.i16_data_z;

    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // ジャイロ（XYZ軸）読み込み
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data[6];
        esp_err_t sts_val = sts_read(s_address, 0x43, u8_data, 6);
        if (sts_val != ESP_OK) {
            break;
        }
        // ジャイロ編集
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        u_conv.u8_values[2] = u8_data[3];
        u_conv.u8_values[3] = u8_data[2];
        u_conv.u8_values[4] = u8_data[5];
        u_conv.u8_values[5] = u8_data[4];
        ps_axes_data->i16_data_x = u_conv.i16_values[0] - i16_data_x;
        ps_axes_data->i16_data_y = u_conv.i16_values[1] - i16_data_y;
        ps_axes_data->i16_data_z = u_conv.i16_values[2] - i16_data_z;
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_fifo_reset
 *
 * DESCRIPTION: FIFOリセット
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_fifo_reset(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // FIFOリセット
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x6A, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = u8_data | 0x04;
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6A, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_device_reset
 *
 * DESCRIPTION: デバイスリセット
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_device_reset(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // デバイスリセット
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x6B, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        u8_data = u8_data | 0x80;
        // レジスタ書き込み
        sts_val = sts_write_byte(s_address, 0x6B, u8_data);
    } while(false);
    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_who_am_i
 *
 * DESCRIPTION: who am i
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_who_am_i(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // who am i
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data;
        sts_val = sts_read_byte(s_address, 0x75, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ編集
        if (u8_data != 0x68) {
            sts_val = ESP_ERR_NOT_FOUND;
        }
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_fifo_cnt
 *
 * DESCRIPTION: Read FIFO data count
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   int16_t*              pi16_cnt      W   FIFOバッファサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_fifo_cnt(ts_i2c_address_t s_address, int16_t* pi16_cnt) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // Read FIFO data count
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data[2];
        sts_val = sts_read(s_address, 0x72, u8_data, 2);
        if (sts_val != ESP_OK) {
            break;
        }
        // カウント
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        *pi16_cnt = u_conv.i16_values[0];
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_fifo_data
 *
 * DESCRIPTION: Read FIFO data
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *   int16_t*              pi16_cnt      W   FIFOバッファデータ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_fifo_data(ts_i2c_address_t s_address, int16_t* pi16_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // Read FIFO data
    //==========================================================================
    // トランザクション開始
    esp_err_t sts_val = sts_io_i2c_mst_begin();
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    do {
        // レジスタ読み込み
        uint8_t u8_data[2];
        sts_val = sts_read(s_address, 0x74, u8_data, 2);
        if (sts_val != ESP_OK) {
            break;
        }
        // データ
        tu_type_converter_t u_conv;
        u_conv.u8_values[0] = u8_data[1];
        u_conv.u8_values[1] = u8_data[0];
        *pi16_data = u_conv.i16_values[0];
    } while(false);

    // トランザクション終了
    sts_io_i2c_mst_end();

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_zeroing_accel
 *
 * DESCRIPTION: 加速度（XYZ軸）ゼロイング
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_zeroing_accel(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // サンプリング
    //==========================================================================
    // 三軸合計値
    int32_t i32_data_x = 0;
    int32_t i32_data_y = 0;
    int32_t i32_data_z = 0;
    // 10msec間隔でサンプリング
    esp_err_t sts_val;
    ts_mpu_6050_axes_data_t s_axes_data;
    // 次回実行時刻
    int64_t i64_next_msec = xTaskGetTickCountMSec();
    int i_idx;
    for (i_idx = 0; i_idx < MPU_6050_CALIBRATION_CNT; i_idx++) {
        // 指定時刻までディレイ（ミリ秒単位）
        i64_dtm_delay_until_msec(i64_next_msec);
        // 加速度を読み込み
        sts_val = sts_mpu_6050_read_accel(s_address, &s_axes_data);
        if (sts_val != ESP_OK) {
            return sts_val;
        }
        // 積算
        i32_data_x = i32_data_x + s_axes_data.i16_data_x;
        i32_data_y = i32_data_y + s_axes_data.i16_data_y;
        i32_data_z = i32_data_z + s_axes_data.i16_data_z;
        // 10ミリ秒後に更新
        i64_next_msec += 10;
    }

    //==========================================================================
    // ゼロイング補正値を設定
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    s_accel_zeroing_data.i16_data_x = (int16_t)(i32_data_x / MPU_6050_CALIBRATION_CNT);
    s_accel_zeroing_data.i16_data_y = (int16_t)(i32_data_y / MPU_6050_CALIBRATION_CNT);
    s_accel_zeroing_data.i16_data_z = (int16_t)(i32_data_z / MPU_6050_CALIBRATION_CNT);

    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_mpu_6050_zeroing_gyro
 *
 * DESCRIPTION: ジャイロ（XYZ軸）ゼロイング
 *
 * PARAMETERS:             Name          RW  Usage
 *   ts_i2c_address_t      s_address     R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 *   None.
 ******************************************************************************/
esp_err_t sts_mpu_6050_zeroing_gyro(ts_i2c_address_t s_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 有効アドレス
    if (!b_valid_address(s_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // サンプリング
    //==========================================================================
    // 三軸合計値
    int32_t i32_data_x = 0;
    int32_t i32_data_y = 0;
    int32_t i32_data_z = 0;
    // 10msec間隔でサンプリング
    esp_err_t sts_val;
    ts_mpu_6050_axes_data_t s_axes_data;
    // 次回実行時刻
    int64_t i64_next_msec = xTaskGetTickCountMSec();
    int i_idx;
    for (i_idx = 0; i_idx < MPU_6050_CALIBRATION_CNT; i_idx++) {
        // 指定時刻までディレイ（ミリ秒単位）
        i64_dtm_delay_until_msec(i64_next_msec);
        // ジャイロを読み込み
        sts_val = sts_mpu_6050_read_gyro(s_address, &s_axes_data);
        if (sts_val != ESP_OK) {
            return sts_val;
        }
        // 積算
        i32_data_x = i32_data_x + s_axes_data.i16_data_x;
        i32_data_y = i32_data_y + s_axes_data.i16_data_y;
        i32_data_z = i32_data_z + s_axes_data.i16_data_z;
        // 10ミリ秒後に更新
        i64_next_msec += 10;
    }

    //==========================================================================
    // ゼロイング補正値を設定
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    // ゼロイング補正値を設定
    s_gyro_zeroing_data.i16_data_x = (int16_t)(i32_data_x / MPU_6050_CALIBRATION_CNT);
    s_gyro_zeroing_data.i16_data_y = (int16_t)(i32_data_y / MPU_6050_CALIBRATION_CNT);
    s_gyro_zeroing_data.i16_data_z = (int16_t)(i32_data_z / MPU_6050_CALIBRATION_CNT);

    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_mpu_6050_zeroing_clear
 *
 * DESCRIPTION: ゼロイングクリア
 *
 * PARAMETERS:             Name          RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 *   None.
 ******************************************************************************/
void v_mpu_6050_zeroing_clear() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // ゼロイングクリア
    //==========================================================================
    s_accel_zeroing_data.i16_data_x = 0;
    s_accel_zeroing_data.i16_data_y = 0;
    s_accel_zeroing_data.i16_data_z = 0;
    s_gyro_zeroing_data.i16_data_x  = 0;
    s_gyro_zeroing_data.i16_data_y  = 0;
    s_gyro_zeroing_data.i16_data_z  = 0;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*****************************************************************************
 *
 * NAME:i16_mpu_6050_composite_value
 *
 * DESCRIPTION:
 *   加速度の算出処理、これだけの為に浮動小数点演算するのは煩雑なので
 *   整数値計算で概算
 *
 * PARAMETERS:                  Name            RW  Usage
 *   ts_mpu_6050_axes_data_t*   ps_axes_data    R   加速度
 *   bool                       b_round_up      R   小数点以下切り上げフラグ
 *
 * RETURNS:
 *   int16_t:概算した３軸の加速度の合成値
 *
 * NOTES:
 * None.
 *****************************************************************************/
int16_t i16_mpu_6050_composite_value(ts_mpu_6050_axes_data_t* ps_axes_data, bool b_round_up) {
    // 加速度の二乗値（三軸の加速度を合成）を取得
    int64_t i64_gx = ps_axes_data->i16_data_x;
    int64_t i64_gy = ps_axes_data->i16_data_y;
    int64_t i64_gz = ps_axes_data->i16_data_z;
    uint64_t u64_gpow = (i64_gx * i64_gx) + (i64_gy * i64_gy) + (i64_gz * i64_gz);
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
    return (s_address.u16_address == I2C_ADDR_MPU_6050_L ||
             s_address.u16_address == I2C_ADDR_MPU_6050_H);
}

/*****************************************************************************
 *
 * NAME: b_valid_accel_range
 *
 * DESCRIPTION:有効加速度レンジチェック
 *
 * PARAMETERS:                  Name            RW  Usage
 * te_mpu_6050_accel_range_t    e_accel_range   R   加速度レンジ
 *
 * RETURNS:
 *   true:有効な加速度レンジ
 *
 * NOTES:
 * None.
 *****************************************************************************/
static bool b_valid_accel_range(te_mpu_6050_accel_range_t e_accel_range) {
    return (e_accel_range >= DRV_MPU_6050_ACCEL_RANGE_2G && e_accel_range <= DRV_MPU_6050_ACCEL_RANGE_16G);
}

/*****************************************************************************
 *
 * NAME: b_valid_gyro_range
 *
 * DESCRIPTION:有効ジャイロレンジチェック
 *
 * PARAMETERS:                  Name            RW  Usage
 * te_mpu_6050_accel_range_t    e_accel_range   R   加速度レンジ
 *
 * RETURNS:
 *   true:有効な加速度レンジ
 *
 * NOTES:
 * None.
 *****************************************************************************/
static bool b_valid_gyro_range(te_mpu_6050_gyro_range_t e_gyro_range) {
    return (e_gyro_range >= DRV_MPU_6050_GYRO_RANGE_250 && e_gyro_range <= DRV_MPU_6050_GYRO_RANGE_2000);
}

/*****************************************************************************
 *
 * NAME: sts_read_byte
 *
 * DESCRIPTION:デバイスから１バイト読み込み
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス
 *   uint8_t            u8_reg_address  R   レジスタアドレス
 *   uint8_t*           pu8_data        R   読み込みデータポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_read_byte(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data) {
    return sts_read(s_address, u8_reg_address, pu8_data, 1);
}

/*****************************************************************************
 *
 * NAME: sts_read
 *
 * DESCRIPTION:指定アドレスのレジスタから指定バイト数分のデータ読み込み
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス
 *   uint8_t            u8_reg_address  R   レジスタアドレス
 *   uint8_t*           pu8_data        R   読み込みデータポインタ
 *   uint8_t            u8_size         R   読み込みデータサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_read(ts_i2c_address_t s_address, uint8_t u8_reg_address, uint8_t* pu8_data, uint8_t u8_size) {
    // レジスタアドレスの書き込み開始
    esp_err_t ts_sts = sts_io_i2c_mst_start_write(s_address);
    if (ts_sts != ESP_OK) {
        return ts_sts;
    }
    // レジスタアドレス書き込み
    ts_sts = sts_io_i2c_mst_write(&u8_reg_address, 1, true);
    if (ts_sts != ESP_OK) {
        return ts_sts;
    }
    // 読み込み開始
    ts_sts = sts_io_i2c_mst_start_read(s_address);
    if (ts_sts != ESP_OK) {
        return ts_sts;
    }
    // データ読み込み
    return sts_io_i2c_mst_read_stop(pu8_data, u8_size);
}

/*****************************************************************************
 *
 * NAME: sts_write_byte
 *
 * DESCRIPTION:デバイスへのデータの書き込み
 *
 * PARAMETERS:          Name            RW  Usage
 *   ts_i2c_address_t   s_address       R   I2Cアドレス
 *   uint8_t            u8_address      R   レジスタアドレス
 *   uint8_t            u8_data         R   書き込みデータ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 *****************************************************************************/
static esp_err_t sts_write_byte(ts_i2c_address_t s_address,
                                   uint8_t u8_address,
                                   uint8_t u8_data) {
    // 書き込み開始
    esp_err_t ts_sts = sts_io_i2c_mst_start_write(s_address);
    if (ts_sts != ESP_OK) {
       return ts_sts;
    }
    // データ書き込み
    uint8_t u8_tx_data[] = {u8_address, u8_data};
    return sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
