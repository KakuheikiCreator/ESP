/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :RX8900 RTC Driver functions source file
 *
 * CREATED:2019/11/24 23:57:00
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
/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_drv_rx8900.h"

#include "ntfw_com_date_time.h"
#include "ntfw_com_value_util.h"


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** RX8900 I2C Address */
#define I2C_ADDR_RX8900A            (0x32)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** レジスタアドレス */
typedef enum {
    DRV_RX8900_RGST_ADR_SEC             = 0x00,
    DRV_RX8900_RGST_ADR_MIN             = 0x01,
    DRV_RX8900_RGST_ADR_HOUR            = 0x02,
    DRV_RX8900_RGST_ADR_WEEK            = 0x03,
    DRV_RX8900_RGST_ADR_DAY             = 0x04,
    DRV_RX8900_RGST_ADR_MONTH           = 0x05,
    DRV_RX8900_RGST_ADR_YEAR            = 0x06,
    DRV_RX8900_RGST_ADR_ALM_MIN         = 0x08,
    DRV_RX8900_RGST_ADR_ALM_HOUR        = 0x09,
    DRV_RX8900_RGST_ADR_ALM_DAY_OR_WEEK = 0x0A,
    DRV_RX8900_RGST_ADR_COUNTER         = 0x0B,
    DRV_RX8900_RGST_ADR_EX_RGST         = 0x0D,
    DRV_RX8900_RGST_ADR_FLG_RGST        = 0x0E,
    DRV_RX8900_RGST_ADR_CTR_RGST        = 0x0F
} te_rx8900_addr_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** 有効週チェック */
static bool b_valid_week(te_rx8900_day_of_week_t e_week);
/** ローカル環境変数への変換処理 */
static ts_rx8900_register_t ts_conv_local(ts_rx8900_register_t s_register);
/** １バイトデータの読み込み処理 */
static esp_err_t sts_read_byte(i2c_port_t e_port_num, uint8_t u8_address, uint8_t* pu8_data);
/** １バイトデータの読み込み処理 */
static esp_err_t sts_read(i2c_port_t e_port_num, uint8_t u8_address, uint8_t* pu8_data, size_t t_size);
/** １バイトデータの書き込み処理 */
static esp_err_t sts_write_byte(i2c_port_t e_port_num, uint8_t u8_address, uint8_t u8_data);
/** １バイトデータの書き込み処理 */
static esp_err_t sts_write(i2c_port_t e_port_num, uint8_t u8_address, uint8_t* pu8_data, size_t t_size);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_rx8900_init
 *
 * DESCRIPTION:RX8900初期化処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_init(i2c_port_t e_port_num) {
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
    // 初期化シーケンス
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val;
    do {
        // 拡張レジスタ設定
        // WADA：曜アラーム（0）
        // USEL：秒更新（0）
        // TE：プリセッタブルカウンタ停止(0)
        // FSEL：FOUT 周波数1Hz(0b10)
        // TSEL：カウントダウン・ソースクロック周期4096Hz(0b00)
        sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_EX_RGST, 0x08);
        if (sts_val != ESP_OK) {
            break;
        }
        // フラグレジスタ設定
        // UF：時刻更新割り込みイベントフラグ
        // TF：定周期タイマー割り込みイベントフラグ
        // AF：アラームフラグ
        // VLF：Voltage Low Flag
        // VDET：Voltage Detect Flag
        sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_FLG_RGST, 0x00);
    } while(false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_reset
 *
 * DESCRIPTION:RX8900リセット処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_reset(i2c_port_t e_port_num) {
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
    // リセット処理
    //==========================================================================
    esp_err_t sts_val;
    do {
        uint8_t u8_data;
        sts_val = sts_read_byte(e_port_num, DRV_RX8900_RGST_ADR_CTR_RGST, &u8_data);
        if (sts_val != ESP_OK) {
            break;
        }
        sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_CTR_RGST, (u8_data | 0x01));
    } while(false);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: s_rx8900_read
 *
 * DESCRIPTION:RX8900レジスタ読み込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *
 * RETURNS:
 *   ts_rx8900_register レジスタ
 *
 ******************************************************************************/
ts_rx8900_register_t s_rx8900_read(i2c_port_t e_port_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // レジスタ値初期化
    ts_rx8900_register_t s_register;
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        memset(&s_register, 0x00, sizeof(ts_rx8900_register_t));
        return ts_conv_local(s_register);
    }

    //==========================================================================
    // I2Cトランザクションの開始
    //==========================================================================
    esp_err_t sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        return ts_conv_local(s_register);
    }

    //==========================================================================
    // レジスタ読み込み
    //==========================================================================
    // レジスタ１読み込み
    if (sts_read(e_port_num, 0x00, (uint8_t*)&s_register, 15) != ESP_OK) {
        memset(&s_register, 0x00, sizeof(ts_rx8900_register_t));
    }
    // レジスタ２読み込み
    if (sts_read(e_port_num, 0x17, (uint8_t*)&s_register.u8_temperature, 4) != ESP_OK) {
        memset(&s_register, 0x00, sizeof(ts_rx8900_register_t));
    }

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    return ts_conv_local(s_register);
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_datetime
 *
 * DESCRIPTION:RX8900日時レジスタ書き込み処理
 *
 * PARAMETERS:            Name            RW  Usage
 *   i2c_port_t           e_port_num      R   I2Cポート番号
 *   ts_rx8900_datetime_t s_datetime      R   日時レジスタ
 *
 * RETURNS:
 *   ts_rx8900_register レジスタ
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_datetime(i2c_port_t e_port_num, ts_rx8900_datetime_t s_datetime) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 日付チェック
    if (!b_dtm_valid_date((int)s_datetime.u8_year,
                              (int)s_datetime.u8_month,
                              (int)s_datetime.u8_day)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 週チェック
    if (!b_valid_week((int)s_datetime.u8_week)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 時刻チェック
    if (!b_dtm_valid_time((int)s_datetime.u8_hour,
                             (int)s_datetime.u8_min,
                             (int)s_datetime.u8_sec)) {
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
    // 日時レジスタ書き込み処理
    //==========================================================================
    // 送信データの編集
    ts_rx8900_datetime_t s_tx_datetime;
    // 年
    s_tx_datetime.u8_year = u8_vutil_binary_to_bcd(s_datetime.u8_year);
    // 月
    s_tx_datetime.u8_month = u8_vutil_binary_to_bcd(s_datetime.u8_month);
    // 日
    s_tx_datetime.u8_day = u8_vutil_binary_to_bcd(s_datetime.u8_day);
    // 時
    s_tx_datetime.u8_hour = u8_vutil_binary_to_bcd(s_datetime.u8_hour);
    // 分
    s_tx_datetime.u8_min = u8_vutil_binary_to_bcd(s_datetime.u8_min);
    // 秒
    s_tx_datetime.u8_sec = u8_vutil_binary_to_bcd(s_datetime.u8_sec);
    // 週
    s_tx_datetime.u8_week = (0x01 << s_datetime.u8_week);
    // レジスタへの書き込み
    esp_err_t sts_val = sts_write(e_port_num, DRV_RX8900_RGST_ADR_SEC, (uint8_t*)&s_tx_datetime, sizeof(ts_rx8900_datetime_t));

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_alarm_min
 *
 * DESCRIPTION:RX8900アラーム（分）書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_min        R   設定値（分）
 *   bool             b_enable      R   有効フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_alarm_min(i2c_port_t e_port_num, uint8_t u8_min, bool b_enable) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 分チェック
    if (!b_dtm_valid_min((int)u8_min)) {
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
    // アラーム（分）書き込み処理
    //==========================================================================
    uint8_t u8_data = ((!b_enable) << 7) | (u8_min & 0x7F);
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_ALM_MIN, u8_data);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_alarm_hour
 *
 * DESCRIPTION:RX8900アラーム（時）書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_hour       R   設定値（時）
 *   bool             b_enable      R   有効フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_alarm_hour(i2c_port_t e_port_num, uint8_t u8_hour, bool b_enable) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 時チェック
    if (!b_dtm_valid_hour((int)u8_hour)) {
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
    // アラーム（時）書き込み処理
    //==========================================================================
    uint8_t u8_data = ((!b_enable) << 7) | (u8_hour & 0x3F);
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_ALM_HOUR, u8_data);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_alarm_day
 *
 * DESCRIPTION:RX8900アラーム（日）書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_hour       R   設定値（時）
 *   bool             b_enable      R   有効フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_alarm_day(i2c_port_t e_port_num, uint8_t u8_day, bool b_enable) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // 日付チェック
    if (!b_dtm_valid_date(2000, 1, (int)u8_day)) {
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
    // アラーム（日）書き込み処理
    //==========================================================================
    uint8_t u8_data = ((!b_enable) << 7) | (u8_day & 0x3F);
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_ALM_DAY_OR_WEEK, u8_data);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_alarm_week
 *
 * DESCRIPTION:RX8900アラーム（週）書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_hour       R   設定値（時）
 *   bool             b_enable      R   有効フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_alarm_week(i2c_port_t e_port_num, uint8_t u8_week, bool b_enable) {
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
    // アラーム（週）書き込み処理
    //==========================================================================
    uint8_t u8_data = ((!b_enable) << 7) | (u8_week & 0x7F);
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_ALM_DAY_OR_WEEK, u8_data);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_counter
 *
 * DESCRIPTION:RX8900カウンタ書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint16_t         u16_cnt       R   カウンタ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_counter(i2c_port_t e_port_num, uint16_t u16_cnt) {
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
    // カウンタ書き込み処理
    //==========================================================================
    uint8_t pu8_data[] = {(uint8_t)u16_cnt, (uint8_t)((u16_cnt >> 8) & 0x0F)};
    esp_err_t sts_val = sts_write(e_port_num, DRV_RX8900_RGST_ADR_COUNTER, pu8_data, 2);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_ex
 *
 * DESCRIPTION:RX8900拡張レジスタ書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_ex         R   拡張レジスタ値
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_ex(i2c_port_t e_port_num, uint8_t u8_ex) {
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
    // 拡張レジスタ書き込み処理
    //==========================================================================
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_EX_RGST, u8_ex);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_flg
 *
 * DESCRIPTION:RX8900フラグレジスタ書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_flg        R   フラグレジスタ値
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_rx8900_write_flg(i2c_port_t e_port_num, uint8_t u8_flg) {
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
    // フラグレジスタ書き込み処理
    //==========================================================================
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_FLG_RGST, u8_flg & 0x3B);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx8900_write_ctl
 *
 * DESCRIPTION:RX8900コントロールレジスタ書き込み処理
 *
 * PARAMETERS:        Name          RW  Usage
 *   i2c_port_t       e_port_num    R   I2Cポート番号
 *   uint8_t          u8_ctl        R   コントロースレジスタ値
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
extern esp_err_t sts_rx8900_write_ctl(i2c_port_t e_port_num, uint8_t u8_ctl) {
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
    // コントロールレジスタ書き込み処理
    //==========================================================================
    esp_err_t sts_val = sts_write_byte(e_port_num, DRV_RX8900_RGST_ADR_CTR_RGST, u8_ctl & 0xF9);

    //==========================================================================
    // I2Cトランザクションの終了
    //==========================================================================
    sts_io_i2c_mst_end();

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: i_rx8900_celsius
 *
 * DESCRIPTION:温度データを100倍した摂氏の値に変換
 *
 * PARAMETERS:      Name              RW  Usage
 *   uint8_t        u8_temperatureR   R   温度データ
 *
 * RETURNS:
 *   ts_rx8900_register レジスタ
 *
 ******************************************************************************/
float f_rx8900_celsius(uint8_t u8_temperature) {
    // 分子を100000倍して計算し、分母を1000する事で100倍の値を算出
    return ((float)u8_temperature * 2 - 187.19) / 3.218;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: b_valid_week
 *
 * DESCRIPTION:有効週チェック
 *
 * PARAMETERS:                  Name        RW  Usage
 *   te_rx8900_day_of_week_t    e_week      R   週の値
 *
 * RETURNS:
 *   true:有効な週
 *
 ******************************************************************************/
static bool b_valid_week(te_rx8900_day_of_week_t e_week) {
    return (e_week == DRV_RX8900_SUNDAY ||
             e_week == DRV_RX8900_MONDAY ||
             e_week == DRV_RX8900_TUESDAY ||
             e_week == DRV_RX8900_WEDNESDAY ||
             e_week == DRV_RX8900_THURSDAY ||
             e_week == DRV_RX8900_FRIDAY ||
             e_week == DRV_RX8900_SATURDAY);
}

/*******************************************************************************
 *
 * NAME: ts_conv_local
 *
 * DESCRIPTION:ローカル環境変数への変換処理
 *
 * PARAMETERS:              Name        RW  Usage
 *   ts_rx8900_register_t   s_register  R   変換対象レジスタ
 *
 * RETURNS:
 *   ts_rx8900_register 変換結果
 *
 ******************************************************************************/
static ts_rx8900_register_t ts_conv_local(ts_rx8900_register_t s_register) {
    // 日時の変換
    ts_rx8900_datetime_t* ps_dt = &s_register.ts_datetime;
    ps_dt->u8_year = u8_vutil_bcd_to_binary(ps_dt->u8_year);
    ps_dt->u8_month = u8_vutil_bcd_to_binary(ps_dt->u8_month);
    ps_dt->u8_day = u8_vutil_bcd_to_binary(ps_dt->u8_day);
    ps_dt->u8_hour = u8_vutil_bcd_to_binary(ps_dt->u8_hour);
    ps_dt->u8_min = u8_vutil_bcd_to_binary(ps_dt->u8_min);
    ps_dt->u8_sec = u8_vutil_bcd_to_binary(ps_dt->u8_sec);
    ps_dt->u8_week = ((ps_dt->u8_week >> 1) & 0x01)
                   + ((ps_dt->u8_week >> 2) & 0x01) * 2
                   + ((ps_dt->u8_week >> 3) & 0x01) * 3
                   + ((ps_dt->u8_week >> 4) & 0x01) * 4
                   + ((ps_dt->u8_week >> 5) & 0x01) * 5
                   + ((ps_dt->u8_week >> 6) & 0x01) * 6;
   // 毎時アラームの変換
    uint8_t u8_val;
    if ((s_register.u8_alarm_min & 0x80) == 0x00) {
        // アラームが有効な場合のみ変換
        s_register.u8_alarm_min = u8_vutil_bcd_to_binary(s_register.u8_alarm_min & 0x7F);
    }
    // 毎日アラームの変換
    if ((s_register.u8_alarm_hour & 0x80) == 0x00) {
        // アラームが有効な場合のみ変換
        s_register.u8_alarm_hour = u8_vutil_bcd_to_binary(s_register.u8_alarm_hour & 0x7F);
    }
    // 週／日アラームの変換
    u8_val = s_register.u8_ex_register;
    if ((u8_val & 0x40) != 0x00 && (s_register.u8_alarm_day_or_week & 0x80) == 0x00) {
        // 日アラームが有効な場合のみ変換
        s_register.u8_alarm_day_or_week = u8_vutil_bcd_to_binary(s_register.u8_alarm_day_or_week & 0x7F);
    }
    // カウンタの値変換
    s_register.u16_counter = (s_register.u16_counter << 8) + (s_register.u16_counter >> 8);
    // 結果返却
    return s_register;
}

/*******************************************************************************
 *
 * NAME: sts_read_byte
 *
 * DESCRIPTION:１バイトデータの読み込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *   uint8_t          u8_address      R   アドレス
 *   uint8_t*         pu8_data        R   データ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_read_byte(i2c_port_t e_port_num, uint8_t u8_address, uint8_t* pu8_data) {
    return sts_read(e_port_num, u8_address, pu8_data, 1);
}

/*******************************************************************************
 *
 * NAME: sts_read
 *
 * DESCRIPTION:１バイトデータの読み込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *   uint8_t          u8_reg_address  R   レジスタアドレス
 *   uint8_t*         pu8_data        W   データ
 *   size_t           t_size          R   読み込みサイズ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_read(i2c_port_t e_port_num,
                           uint8_t u8_reg_address,
                           uint8_t* pu8_data,
                           size_t t_size) {
    // 書き込み開始
    ts_i2c_address_t s_address = {e_port_num, I2C_ADDR_RX8900A};
    esp_err_t tsts_val = sts_io_i2c_mst_start_write(s_address);
    if (tsts_val != ESP_OK) {
        return tsts_val;
    }
    // アドレス書き込み
    uint8_t u8_reg_data[] = {u8_reg_address};
    tsts_val = sts_io_i2c_mst_write(u8_reg_data, 1, true);
    if (tsts_val != ESP_OK) {
        return tsts_val;
    }
    // 読み込み開始
    tsts_val = sts_io_i2c_mst_start_read(s_address);
    if (tsts_val != ESP_OK) {
        return tsts_val;
    }
    // データ読み込み
    return sts_io_i2c_mst_read_stop(pu8_data, t_size);
}

/*******************************************************************************
 *
 * NAME: sts_write_byte
 *
 * DESCRIPTION:１バイトデータの書き込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *   uint8_t          u8_reg_address  R   アドレス
 *   uint8_t          u8_data         R   データ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_write_byte(i2c_port_t e_port_num, uint8_t u8_address, uint8_t u8_data) {
    return sts_write(e_port_num, u8_address, &u8_data, 1);
}


/*******************************************************************************
 *
 * NAME: sts_write_byte
 *
 * DESCRIPTION:１バイトデータの書き込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   i2c_port_t       e_port_num      R   I2Cポート番号
 *   uint8_t          u8_address      R   レジスタアドレス
 *   uint8_t*         pu8_data        R   データ
 *   size_t           t_size          R   書き込みサイズ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_write(i2c_port_t e_port_num, uint8_t u8_reg_address, uint8_t* pu8_data, size_t t_size) {
    // 書き込み開始
    ts_i2c_address_t s_address = {e_port_num, I2C_ADDR_RX8900A};
    esp_err_t tsts_val = sts_io_i2c_mst_start_write(s_address);
    if (tsts_val != ESP_OK) {
       return tsts_val;
    }
    // 送信データ編集
    size_t t_tx_size = t_size + 1;
    uint8_t u8_reg_data[t_tx_size];
    u8_reg_data[0] = u8_reg_address;
    memcpy(&u8_reg_data[1], pu8_data, t_size);
    // データ送信
    return sts_io_i2c_mst_write_stop(u8_reg_data, t_tx_size, true);
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
