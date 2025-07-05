/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common I2C master library source file
 *
 * CREATED:2025/01/15 05:30:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:I2Cマスタの共通系ライブラリ
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:
 *
 *******************************************************************************
 *
 * Copyright (c) 2025 Kakuheiki.Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_io_i2c_mst.h"

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include "ntfw_com_mem_alloc.h"
#include "ntfw_com_value_util.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 構造体：I2Cデバイス情報 */
typedef struct s_i2c_dev_info_t {
    // デバイスポート
    i2c_port_t e_port_no;
    // デバイスアドレス
    uint16_t u16_address;
    // デバイスハンドル
    i2c_master_dev_handle_t s_dev_handle;
    // 次要素ポインタ
    struct s_i2c_dev_info_t* ps_next;
} ts_i2c_dev_info_t;

/** 構造体：I2Cバス情報 */
typedef struct {
    // バススピード
    ts_i2c_mst_freq_mode_t e_freq_mode;
    // SCLピン
    gpio_num_t e_scl_pin;
    // SDAピン
    gpio_num_t e_sda_pin;
    // バスハンドル
    i2c_master_bus_handle_t s_bus_handle;
    // タイムアウト時間
    int i_max_wait_ms;
    // デバイスリストトップ
    ts_i2c_dev_info_t* ps_dev_top;
} ts_i2c_bus_info_t;

/**
 * ミューテックス取得処理
 */
typedef SemaphoreHandle_t (*tf_get_mutex_t)();

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static volatile SemaphoreHandle_t s_mutex = NULL;

/** I2Cバス情報リスト */
static ts_i2c_bus_info_t s_bus_list[I2C_NUM_MAX] = {
    {
        .e_freq_mode = I2C_MST_FREQ_HZ_MAX,
        .e_scl_pin = GPIO_NUM_NC,
        .e_sda_pin = GPIO_NUM_NC,
        .s_bus_handle = NULL,
        .i_max_wait_ms = -1,
        .ps_dev_top = NULL,
    },
#if SOC_HP_I2C_NUM >= 2
    {
        .e_freq_mode = I2C_MST_FREQ_HZ_MAX,
        .e_scl_pin = GPIO_NUM_NC,
        .e_sda_pin = GPIO_NUM_NC,
        .s_bus_handle = NULL,
        .i_max_wait_ms = -1,
        .ps_dev_top = NULL,
    },
#endif
#if SOC_LP_I2C_SUPPORTED
    {
        .e_freq_mode = I2C_MST_FREQ_HZ_MAX,
        .e_scl_pin = GPIO_NUM_NC,
        .e_sda_pin = GPIO_NUM_NC,
        .s_bus_handle = NULL,
        .i_max_wait_ms = -1,
        .ps_dev_top = NULL,
    },
#endif
};

/** キャッシュデータ：デバイス情報 */
static ts_i2c_dev_info_t* ps_cache_dev_info = NULL;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** ミューテックス取得処理（初期処理） */
static SemaphoreHandle_t get_mutex_init();
/** ミューテックス取得処理 */
static SemaphoreHandle_t get_mutex();
/** ミューテックス取得関数 */
static volatile tf_get_mutex_t pf_get_mutex = get_mutex_init;
/** I2Cデバイス情報取得処理 */
static ts_i2c_dev_info_t* ps_get_dev_info(i2c_port_t e_port_no, uint16_t u16_address);
/** I2Cバス解放処理 */
static esp_err_t sts_i2c_bus_deinit(ts_i2c_bus_info_t* ps_bus);
/** I2Cデバイス設定の追加処理 */
static esp_err_t sts_i2c_add_device(i2c_port_t e_port_no,
                                    uint16_t u16_address,
                                    i2c_addr_bit_len_t e_addr_type);

/******************************************************************************/
/***      Local Function Pointer                                            ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

//==============================================================================
// マスター側機能（I2Cバスアクセス）
//==============================================================================

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_bus_init
 *
 * DESCRIPTION:I2Cバスの初期処理
 *
 * PARAMETERS:          Name          RW  Usage
 *   i2c_port_t         e_port_no     R   I2Cポート番号
 *   ts_i2c_freq_mode_t e_freq        R   バススピードモード
 *   gpio_num_t         e_scl_pin     R   SCLピン番号
 *   gpio_num_t         e_sda_pin     R   SDAピン番号
 *   bool               b_pullup      R   プルアップフラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_bus_init(i2c_port_num_t e_port_no,
                                  ts_i2c_mst_freq_mode_t e_freq,
                                  gpio_num_t e_scl_pin,
                                  gpio_num_t e_sda_pin,
                                  bool b_pullup) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2Cバススピード
    if (e_freq >= I2C_MST_FREQ_HZ_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    // 割り当てピン番号チェック
    if (!b_vutil_valid_gpio(e_scl_pin) || !b_vutil_valid_gpio(e_sda_pin) || e_scl_pin == e_sda_pin) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 設定済みのバスとデバイスを解放
    //==========================================================================
    // I2Cバス情報
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    // I2Cバスの解放処理
    sts_i2c_bus_deinit(ps_bus);

    //==========================================================================
    // I2Cポートの初期化処理
    //==========================================================================
    // バス設定
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = e_port_no,
        .scl_io_num = e_scl_pin,
        .sda_io_num = e_sda_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = b_pullup,
    };
#if SOC_LP_I2C_SUPPORTED
    if (e_port_no == LP_I2C_NUM_0) {
        i2c_mst_config.lp_source_clk = LP_I2C_SCLK_DEFAULT;
    }
#endif
    // I2Cバス初期化
    esp_err_t sts_val = i2c_new_master_bus(&i2c_mst_config, &ps_bus->s_bus_handle);
    if (sts_val == ESP_OK) {
        // I2Cバス情報を初期化
        ps_bus->e_freq_mode = e_freq;
        ps_bus->e_scl_pin = e_scl_pin;
        ps_bus->e_sda_pin = e_sda_pin;
        ps_bus->i_max_wait_ms = -1,
        ps_bus->ps_dev_top = NULL;
    } else {
        ps_bus->s_bus_handle = NULL;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;                                      
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_bus_deinit
 *
 * DESCRIPTION:I2Cバスのリソース解放処理
 *
 * PARAMETERS:          Name          RW  Usage
 *   i2c_port_t         e_port_no     R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_bus_deinit(i2c_port_num_t e_port_no) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cバスの解放処理
    //==========================================================================
    esp_err_t sts_val = sts_i2c_bus_deinit(&s_bus_list[e_port_no]);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_add_device
 *
 * DESCRIPTION:I2Cバスのデバイス設定追加処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_add_device(ts_i2c_mst_address_t* ps_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイス設定の追加処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = sts_i2c_add_device(e_port_no, u16_address, e_addr_type);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_del_device
 *
 * DESCRIPTION:I2Cバスのデバイス設定リソース解放処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_del_device(ts_i2c_mst_address_t* ps_address) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイス設定の削除処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // I2Cバス情報
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    if (ps_bus->s_bus_handle != NULL) {
        // 同一ポートのI2Cデバイスを探索
        ts_i2c_dev_info_t* ps_pre_dev = NULL;
        ts_i2c_dev_info_t* ps_tgt_dev = ps_bus->ps_dev_top;
        while (ps_tgt_dev != NULL) {
            // ハンドル判定
            if (ps_tgt_dev->u16_address != u16_address) {
                // 次デバイス情報をチェック
                ps_pre_dev = ps_tgt_dev;
                ps_tgt_dev = ps_tgt_dev->ps_next;
                continue;
            }
            // デバイス設定を削除
            sts_val = i2c_master_bus_rm_device(ps_tgt_dev->s_dev_handle);

            // デバイス情報のキャッシュクリア
            if (ps_cache_dev_info == ps_tgt_dev) {
                // キャッシュクリア
                ps_cache_dev_info = NULL;
            }
            // デバイス情報を削除
            if (ps_pre_dev == NULL) {
                ps_bus->ps_dev_top = ps_tgt_dev->ps_next;
            } else {
                ps_pre_dev->ps_next = ps_tgt_dev->ps_next;
            }
            // デバイスのメモリ解放
            l_mem_free(ps_tgt_dev);
            // デバイス設定の解放完了
            break;
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_set_timeout_ms
 *
 * DESCRIPTION:I2Cバスのタイムアウト時間を設定（バス初期化時デフォルト：-1）
 *
 * PARAMETERS:      Name            RW  Usage
 * i2c_port_num_t   e_port_no       R   I2Cポート番号
 * int              i_max_wait_ms   R   最大タイムアウト時間（ミリ秒）
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_set_timeout_ms(i2c_port_num_t e_port_no, int i_max_wait_ms) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
        return ESP_ERR_INVALID_ARG;
    }
    // タイムアウト時間
    if (i_max_wait_ms < -1) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイス設定の追加処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // I2Cバス情報の有効判定
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    if (ps_bus->s_bus_handle != NULL) {
        // 最大ウェイト時間を設定
        ps_bus->i_max_wait_ms = i_max_wait_ms;
        // 結果ステータス
        sts_val = ESP_OK;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_tran_begin
 *
 * DESCRIPTION:トランザクション開始
 *
 * PARAMETERS:        Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_tran_begin() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }
    // 結果ステータス返却
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_tran_end
 *
 * DESCRIPTION:トランザクション終了
 *
 * PARAMETERS:        Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_tran_end() {
    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(pf_get_mutex()) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_set_timeout_ms
 *
 * DESCRIPTION:I2Cスレーブへのデータ送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 * uint8_t*                 pu8_tx_data     R   送信データ
 * size_t                   t_tx_len        R   送信データ長
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_tx(ts_i2c_mst_address_t* ps_address,
                            const uint8_t* pu8_tx_data,
                            const size_t t_tx_len) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }
    // 送信データ
    if (pu8_tx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイスへデータ送信
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // デバイス情報を取得
    ts_i2c_dev_info_t* ps_dev = ps_get_dev_info(e_port_no, u16_address);
    if (ps_dev != NULL) {
        // I2Cバス情報取得
        ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
        // データ送信
        sts_val = i2c_master_transmit(ps_dev->s_dev_handle,
                                      pu8_tx_data, t_tx_len,
                                      ps_bus->i_max_wait_ms);
        // タイムアウト判定
        if (sts_val != ESP_OK) {
            // バスクラッシュと判断してリセット
            i2c_master_bus_reset(ps_bus->s_bus_handle);
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_txrx
 *
 * DESCRIPTION:I2Cスレーブへのデータ送受信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 * uint8_t*                 pu8_tx_data     R   送信データ
 * size_t                   t_tx_len        R   送信データ長
 * uint8_t*                 pu8_rx_data     R   受信データ
 * size_t                   t_rx_len        R   受信データ長
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_txrx(ts_i2c_mst_address_t* ps_address,
                              const uint8_t* pu8_tx_data,
                              const size_t t_tx_len, 
                              uint8_t* pu8_rx_data,
                              size_t t_rx_len) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }
    // 送信データ
    if (pu8_tx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // 受信データ
    if (pu8_rx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイスへデータ送信
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // デバイス情報を取得
    ts_i2c_dev_info_t* ps_dev = ps_get_dev_info(e_port_no, u16_address);
    if (ps_dev != NULL) {
        // I2Cバス情報取得
        ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
        // データ送信とデータ受信
        sts_val = i2c_master_transmit_receive(ps_dev->s_dev_handle,
                                              pu8_tx_data, t_tx_len,
                                              pu8_rx_data, t_rx_len,
                                              ps_bus->i_max_wait_ms);
        // タイムアウト判定
        if (sts_val != ESP_OK) {
            // バスクラッシュと判断してリセット
            i2c_master_bus_reset(ps_bus->s_bus_handle);
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;                                         
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_rx
 *
 * DESCRIPTION:I2Cスレーブからのデータ受信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 * uint8_t*                 pu8_rx_data     R   受信データ
 * size_t                   t_rx_len        R   受信データ長
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_rx(ts_i2c_mst_address_t* ps_address,
                            uint8_t* pu8_rx_data,
                            size_t t_rx_len) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }
    // 受信データ
    if (pu8_rx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイスへデータ送信
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // デバイス情報を取得
    ts_i2c_dev_info_t* ps_dev = ps_get_dev_info(e_port_no, u16_address);
    if (ps_dev != NULL) {
        // I2Cバス情報取得
        ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
        // データ送信
        sts_val = i2c_master_receive(ps_dev->s_dev_handle,
                                     pu8_rx_data, t_rx_len,
                                     ps_bus->i_max_wait_ms);
        // タイムアウト判定
        if (sts_val != ESP_OK) {
            // バスクラッシュと判断してリセット
            i2c_master_bus_reset(ps_bus->s_bus_handle);
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_get_handle
 *
 * DESCRIPTION:I2Cバスとデバイスのハンドル取得
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_i2c_address_t*        ps_address      R   I2Cアドレス
 * i2c_master_bus_handle_t* ps_bus_hndl     R   I2Cバスハンドル
 * i2c_master_dev_handle_t* ps_dev_hndl     R   I2Cデバイスハンドル
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_get_handle(ts_i2c_mst_address_t* ps_address,
                                    i2c_master_bus_handle_t* ps_bus_hndl,
                                    i2c_master_dev_handle_t* ps_dev_hndl) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // I2Cアドレスチェック
    if (ps_address == NULL) {
         return ESP_ERR_INVALID_ARG;
    }
    // ポート番号
    i2c_port_t e_port_no = ps_address->e_port_no;
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
         return ESP_ERR_INVALID_ARG;
    }
    // アドレス
    uint16_t u16_address = ps_address->u16_address;
    i2c_addr_bit_len_t e_addr_type = e_io_i2c_mst_adress_type(u16_address);
    if (e_addr_type == 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2Cバスハンドル
    if (ps_bus_hndl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // I2Cデバイスハンドル
    if (ps_dev_hndl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cデバイスへデータ送信
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // デバイス情報を取得
    ts_i2c_dev_info_t* ps_dev = ps_get_dev_info(e_port_no, u16_address);
    if (ps_dev != NULL) {
        // I2Cバス情報取得
        ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
        // バスハンドル
        *ps_bus_hndl = ps_bus->s_bus_handle;
        // デバイスハンドル
        *ps_dev_hndl = ps_dev->s_dev_handle;
        // 結果ステータス更新
        sts_val = ESP_OK;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_bus_reset
 *
 * DESCRIPTION:I2Cバスのリセット
 *
 * PARAMETERS:      Name        RW  Usage
 * i2c_port_num_t   e_port_no   R   I2Cポート
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_bus_reset(i2c_port_num_t e_port_no) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ポート番号
    if (!b_io_i2c_mst_valid_port(e_port_no)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cバスのリセット
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_INVALID_STATE;
    // I2Cバス情報
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    if (ps_bus->s_bus_handle != NULL) {
        // I2Cバスのリセット
        sts_val = i2c_master_bus_reset(ps_bus->s_bus_handle);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 終了ステータスを返却
    return sts_val;                                      
}

/*******************************************************************************
 *
 * NAME: e_io_i2c_mst_adress_type
 *
 * DESCRIPTION:I2Cアドレスタイプ取得
 *
 * PARAMETERS:      Name        RW  Usage
 * uint16_t         u16_data    R   アドレス
 *
 * RETURNS:
 *   i2c_addr_bit_len_t アドレスタイプ ※エラー時は0xFF
 *
 ******************************************************************************/
i2c_addr_bit_len_t e_io_i2c_mst_adress_type(uint16_t u16_address) {
    // 7bitアドレス判定
    if (b_io_i2c_mst_valid_7bit_adress(u16_address)) {
        return I2C_ADDR_BIT_LEN_7;
    }
#if SOC_I2C_SUPPORT_10BIT_ADDR
    // 10bitアドレス判定
    if (b_io_i2c_mst_valid_10bit_adress(u16_address)) {
        return I2C_ADDR_BIT_LEN_10;
    }
#endif
    return 0xff;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: get_mutex_init
 *
 * DESCRIPTION:ミューテックス取得処理（初期処理）
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   SemaphoreHandle_t ミューテックス
 *
 ******************************************************************************/
static SemaphoreHandle_t get_mutex_init() {
    // Mutexの初期化
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
        pf_get_mutex = get_mutex;
    }
    return s_mutex;
}

/*******************************************************************************
 *
 * NAME: get_mutex
 *
 * DESCRIPTION:ミューテックス取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   SemaphoreHandle_t ミューテックス
 *
 ******************************************************************************/
static SemaphoreHandle_t get_mutex() {
    return s_mutex;
}

/*******************************************************************************
 *
 * NAME: ps_get_dev_info
 *
 * DESCRIPTION:I2Cデバイス情報取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * i2c_port_t               e_port_no       R   I2Cポート
 * uint16_t                 u16_address     R   I2Cアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static ts_i2c_dev_info_t* ps_get_dev_info(i2c_port_t e_port_no, uint16_t u16_address) {
    //==========================================================================
    // キャッシュ情報判定
    //==========================================================================
    if (ps_cache_dev_info != NULL) {
        if (ps_cache_dev_info->e_port_no == e_port_no && ps_cache_dev_info->u16_address == u16_address) {
            return ps_cache_dev_info;
        }
    }

    //==========================================================================
    // デバイス情報探索
    //==========================================================================
    // I2Cバス情報
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    // I2Cバスの初期化済みチェック
    if (ps_bus->s_bus_handle == NULL) {
        return NULL;
    }
    // I2Cデバイスの追加済みチェック
    ts_i2c_dev_info_t* ps_dev = ps_bus->ps_dev_top;
    while (ps_dev != NULL) {
        if (ps_dev->u16_address == u16_address) {
            break;
        }
        ps_dev = ps_dev->ps_next;
    }
    // 探索結果をキャッシュ
    ps_cache_dev_info = ps_dev;
    // 対象デバイス無し
    return ps_dev;
}

/*******************************************************************************
 *
 * NAME: sts_i2c_bus_deinit
 *
 * DESCRIPTION:I2Cバス解放処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_i2c_bus_info_t*   ps_bus      RW  I2Cバス情報
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_i2c_bus_deinit(ts_i2c_bus_info_t* ps_bus) {
    // 入力チェック
    if (ps_bus->e_freq_mode == I2C_MST_FREQ_HZ_MAX) {
        return ESP_OK;
    }
    // I2Cデバイスを解放
    ts_i2c_dev_info_t* ps_pre_dev;
    ts_i2c_dev_info_t* ps_chk_dev = ps_bus->ps_dev_top;
    while (ps_chk_dev != NULL) {
        // デバイス設定を削除
        i2c_master_bus_rm_device(ps_chk_dev->s_dev_handle);
        // 次デバイス情報
        ps_pre_dev = ps_chk_dev;
        ps_chk_dev = ps_chk_dev->ps_next;
        // デバイスのメモリ解放
        l_mem_free(ps_pre_dev);
    }
    // I2Cバスを解放
    esp_err_t sts_val = i2c_del_master_bus(ps_bus->s_bus_handle);
    // I2Cバス情報をクリア
    ps_bus->e_freq_mode = I2C_MST_FREQ_HZ_MAX;
    ps_bus->e_scl_pin = GPIO_NUM_NC;
    ps_bus->e_sda_pin = GPIO_NUM_NC;
    ps_bus->s_bus_handle = NULL;
    ps_bus->i_max_wait_ms = -1;
    ps_bus->ps_dev_top = NULL;
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_i2c_add_device
 *
 * DESCRIPTION:I2Cデバイス設定の追加処理
 *
 * PARAMETERS:              Name            RW  Usage
 * i2c_port_t               e_port_no       R   I2Cポート
 * uint16_t                 u16_address     R   I2Cアドレス
 * i2c_addr_bit_len_t       e_addr_type     R   I2Cアドレスタイプ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_i2c_add_device(i2c_port_t e_port_no,
                                    uint16_t u16_address,
                                    i2c_addr_bit_len_t e_addr_type) {
    //==========================================================================
    // 実行可否チェック
    //==========================================================================
    // I2Cバス情報
    ts_i2c_bus_info_t* ps_bus = &s_bus_list[e_port_no];
    // I2Cバスの初期化済みチェック
    if (ps_bus->s_bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // I2Cデバイスの追加済みチェック
    ts_i2c_dev_info_t* ps_chk_dev = ps_bus->ps_dev_top;
    while (ps_chk_dev != NULL) {
        if (u16_address == ps_chk_dev->u16_address) {
            return ESP_OK;
        }
        ps_chk_dev = ps_chk_dev->ps_next;
    }

    //==========================================================================
    // I2Cデバイス情報を生成
    //==========================================================================
    // I2Cデバイス情報を生成
    ts_i2c_dev_info_t* ps_dev = pv_mem_malloc(sizeof(ts_i2c_dev_info_t));
    if (ps_dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // I2Cポート
    ps_dev->e_port_no = e_port_no;
    // デバイスアドレス
    ps_dev->u16_address = u16_address;
    // デバイスハンドル
    ps_dev->s_dev_handle = NULL;
    // 次要素ポインタ
    ps_dev->ps_next = NULL;

    //==========================================================================
    // デバイス設定の追加
    //==========================================================================
    // I2Cデバイス設定
    i2c_device_config_t s_dev_config = {
        .dev_addr_length = e_addr_type,
        .device_address = u16_address,
        .scl_speed_hz = ps_bus->e_freq_mode,
    };
    // I2Cデバイス設定を追加
    esp_err_t sts_val = i2c_master_bus_add_device(ps_bus->s_bus_handle, &s_dev_config, &ps_dev->s_dev_handle);
    if (sts_val == ESP_OK) {
        // I2CバスにI2Cデバイスを追加
        ps_dev->ps_next = ps_bus->ps_dev_top;
        ps_bus->ps_dev_top = ps_dev;
    } else {
        // I2Cデバイス情報を解放
        l_mem_free(ps_dev);
    }
    // 結果返信
    return sts_val;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
