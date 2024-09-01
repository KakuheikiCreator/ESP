/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common I2C library source file
 *
 * CREATED:2019/11/17 12:24:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:I2Cの共通系ライブラリ
 *
 * Note:
 * 実装するにあたり、テストしたところ次の注意点に留意する必要が有った
 * １．キューイングされたコマンドの実行タイミングについての注意点
 *   ESP-IDFではI2Cのスタートコンディション、ストップコンディション、読み込み、
 *   書き込みの各処理は、キューイングされる仕組みになっている。
 *   実際に送受信をするのは、i2c_master_cmd_begin関数が実行されたタイミング
 *   となるが、このメソッドの実行タイミングについては注意が必要となる。
 *   この際、スタートコンディション、ストップコンディションのみ単独で
 *   キューイングされた状態で実行したところ、動作不良が発生した。
 *
 *   この為、書き込みと読み込みのそれぞれについて、次のような制御が必要となる。
 *   a.書き込み時
 *     スタートは必ずデバイスのアドレスや、データの書き込み等の送信処理まで
 *     キューイングしてから実行する。
 *     ストップについてもデータの送信とセットで実行する、
 *   b.読み込み時
 *     書き込み時と同様に、読み込み時もデバイスのアドレスとセットで読み込み
 *     フラグが送信される事から、アドレスの書き込みもキューイングされた
 *     状態で実行する事
 *     同様にストップについても終端データの読み込みとセットで実行する事
 *
 * ２．排他制御について
 *   ESP-IDFでは、2ポートのI2Cバスを制御できる様になっているが
 *   コマンドをキューイングするキューは共有されているので
 *   キューイングされたコマンドを、i2c_master_cmd_begin関数で実行する際にのみ
 *   ポートを指定する様になっている。
 *   つまりコマンドのキューイング時にはポート指定をしていない。
 *   この為、実行単位の処理のキューイングについては、アクセスを同一ポートの
 *   同一デバイスへのトランザクション処理に割り当てする必要がある。
 *   同じポートでも他のデバイスへのアクセスや、同じデバイスでも他の処理を
 *   介入させてしまう事は許されない。
 *
 * ３．書き込み処理について
 *   キューに送信データの書き込みをする際に、i2c_master_write関数を使用しない事
 *   この関数の引数はポインタになっているが、この関数はその参照データを
 *   内部的にポインタ参照している様で、i2c_master_cmd_begin関数実行時に
 *   引数で渡したデータの変数が書き換わっていると、問題が生じるケースが発生した。
 *
 * Bug fix:
 * ESP32のI2Cの機能にはバグが存在するとの事で、次の様に対応した。
 * １．I2C通信を実行していると、タイムアウト(コード263)のエラーが頻発
 *     これは次の様に既に多くのGitでも問題が認識されている
 *
 * https://www.esp32.com/viewtopic.php?t=2178
 * >i2c randombly fails
 *
 * ネット上の情報や様々な試行で原因を調査したが、詳細な原因は不明であったが
 * 試行の結果、想定されないストップコンディションが送信されると、この現象が
 * 発生している様子であった。
 * つまりスレーブからの読み取りの際にACK返信をしているのに、その直後に
 * ストップコンディションを送信してしまうとエラーが発生していた。
 * この為、終端データの読み込み時にACK返信しない様に、ストップコンディション
 * の送信が伴う読み込み処理で制御する様にし、問題を回避した。
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
#include "ntfw_io_i2c_master.h"

#include <freertos/FreeRTOS.h>
#include "ntfw_com_date_time.h"
#include "ntfw_com_value_util.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Readフラグ */
#define COM_I2C_MST_FLG_WRITE     (0x00)
#define COM_I2C_MST_FLG_READ      (0x01)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 列挙型：コマンド種類 */
typedef enum {
    COM_I2C_MST_INIT = 0,           /** I2C initialize process */
    COM_I2C_MST_BEGIN,              /** I2C begin process */
    COM_I2C_MST_END,                /** I2C end process */
    COM_I2C_MST_START,              /** I2C start process */
    COM_I2C_MST_READ,               /** I2C read process */
    COM_I2C_MST_WRITE               /** I2C write process */
} ts_i2c_mst_command_t;

/** 構造体：I2Cキュー情報 */
typedef struct {
    /** ポート番号 */
    i2c_port_t e_port_no;
    /** コマンドハンドラ */
    i2c_cmd_handle_t v_cmd_hndl;
    /** 順序制御フラグ */
    bool b_order_flg[6];
} ts_i2c_queue_info_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** キュー実行のウェイト時間 */
static const TickType_t TICKS_TO_WAIT = 2000 / portTICK_PERIOD_MS;

/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

/** 構造体：I2Cポート設定 */
static i2c_config_t s_i2c_config_list[I2C_NUM_MAX];

/** I2Cキュー情報 */
static volatile ts_i2c_queue_info_t s_queue_info = {
    .e_port_no   = I2C_NUM_MAX,
    .v_cmd_hndl  = NULL,
    .b_order_flg = {true, false, false, false, false, false}
};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** 初期処理 */
static esp_err_t sts_i2c_init(i2c_port_t e_port_no);
/** スタートコンディションの送信 */
static esp_err_t sts_i2c_start(ts_i2c_address_t s_address, uint8_t u8_read_flg);
/** ポインタ参照問題に対応した書き込み処理 */
static esp_err_t sts_i2c_write(i2c_cmd_handle_t s_cmd_handle,
                                uint8_t* pu8_data,
                                size_t t_data_len,
                                bool b_ack_en);
/** リセットコンディション */
static void v_reset_condition();

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
 * NAME: sts_io_i2c_mst_init
 *
 * DESCRIPTION:I2Cマスタ初期化処理
 *
 * PARAMETERS:          Name          RW  Usage
 *   i2c_port_t         e_port_no     R   I2Cポート番号（内部回路番号：0/1）
 *   ts_i2c_freq_mode_t e_freq        R   バススピードモード
 *   gpio_num_t         e_scl_pin     R   SCLピン番号
 *   gpio_num_t         e_sda_pin     R   SDAピン番号
 *   gpio_pullup_t      e_pullup      R   プルアップ設定
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_init(i2c_port_t e_port_no,
                               ts_i2c_freq_mode_t e_freq,
                               gpio_num_t e_scl_pin,
                               gpio_num_t e_sda_pin,
                               gpio_pullup_t e_pullup) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // Mutexの初期化
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // I2Cポートの初期化処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val;
    do {
        //======================================================================
        // チェック
        //======================================================================
        // ポート番号チェック
        if (!b_io_i2c_mst_valid_port(e_port_no)) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // クロック周波数チェック
        if (e_freq < I2C_FREQ_HZ_LOW || e_freq > I2C_FREQ_HZ_1M) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // 割り当てピン番号チェック
        if (!b_vutil_valid_gpio(e_scl_pin) || !b_vutil_valid_gpio(e_sda_pin) || e_scl_pin == e_sda_pin) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // プルアップチェック
        if (!b_vutil_valid_pullup(e_pullup)) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // 実行順序
        if (!s_queue_info.b_order_flg[COM_I2C_MST_INIT]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // ポート設定
        //======================================================================
        i2c_config_t* ps_i2c_config = &s_i2c_config_list[e_port_no];
        // I2Cマスターモード
        ps_i2c_config->mode = I2C_MODE_MASTER;
        // クロックスピード
        ps_i2c_config->master.clk_speed = (uint32_t)e_freq;
        // SCLピン
        ps_i2c_config->scl_io_num = e_scl_pin;
        // SDLピン
        ps_i2c_config->sda_io_num = e_sda_pin;
        // プルアップ設定（SCLピン）
        ps_i2c_config->scl_pullup_en = e_pullup;
        // プルアップ設定（SDAピン）
        ps_i2c_config->sda_pullup_en = e_pullup;
        // 初期化処理
        sts_val = sts_i2c_init(e_port_no);
        if (sts_val != ESP_OK ) {
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;

    } while (false);

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
 * NAME: sts_io_i2c_mst_begin
 *
 * DESCRIPTION:トランザクション開始
 *
 * PARAMETERS:        Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_begin() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // チェック
    //==========================================================================
    // 順序制御チェック
    if (!s_queue_info.b_order_flg[COM_I2C_MST_BEGIN]) {
        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 順序制御フラグ更新
    //==========================================================================
    s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = false;
    s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
    s_queue_info.b_order_flg[COM_I2C_MST_END]   = true;
    s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
    s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
    s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;

    // 結果ステータス返却
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_end
 *
 * DESCRIPTION:トランザクション終了
 *
 * PARAMETERS:        Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_end() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //======================================================================
        // チェック
        //======================================================================
        // 順序制御チェック
        if (!s_queue_info.b_order_flg[COM_I2C_MST_END]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // クリティカルセクション終了
        //======================================================================
        if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = true;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;

    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_start_read
 *
 * DESCRIPTION:読み込みスタートコンディションの送信
 *
 * PARAMETERS:        Name        RW  Usage
 * ts_i2c_address_t   s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_start_read(ts_i2c_address_t s_address) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val;
    do {
        //======================================================================
        // スタートコンディション送信
        //======================================================================
        sts_val = sts_i2c_start(s_address, COM_I2C_MST_FLG_READ);
        if (sts_val != ESP_OK ) {
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_start_write
 *
 * DESCRIPTION:書き込みスタートコンディションの送信
 *
 * PARAMETERS:        Name        RW  Usage
 * ts_i2c_address_t   s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_start_write(ts_i2c_address_t s_address) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val;
    do {
        //======================================================================
        // スタートコンディション送信
        //======================================================================
        sts_val = sts_i2c_start(s_address, COM_I2C_MST_FLG_WRITE);
        if (sts_val != ESP_OK ) {
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = true;
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_read
 *
 * DESCRIPTION:スレーブからの読み込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   uint8_t*         pu8_data        W   読み込み先バッファ
 *   size_t           t_data_len      R   読み込みサイズ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_read(uint8_t* pu8_data, size_t t_data_len) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //======================================================================
        // チェック
        //======================================================================
        // 読み込みサイズチェック
        if (t_data_len == 0) {
            break;
        }
        // 順序制御チェック
        if (!s_queue_info.b_order_flg[COM_I2C_MST_READ]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // スレーブからの読み込み処理
        //======================================================================
        // キューイング：スレーブからのデータ読み込み（後続データあり）
        sts_val = i2c_master_read(s_queue_info.v_cmd_hndl, pu8_data, t_data_len, I2C_MASTER_ACK);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を待つ
        sts_val = i2c_master_cmd_begin(s_queue_info.e_port_no, s_queue_info.v_cmd_hndl, TICKS_TO_WAIT);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;

    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_read_stop
 *
 * DESCRIPTION:スレーブからの読み込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   uint8_t*         pu8_data        W   読み込み先バッファ
 *   size_t           t_data_len      R   読み込みサイズ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_read_stop(uint8_t* pu8_data, size_t t_data_len) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //======================================================================
        // チェック
        //======================================================================
        // 読み込みサイズ
        if (t_data_len == 0) {
            break;
        }
        // 順序制御チェック
        if (!s_queue_info.b_order_flg[COM_I2C_MST_READ]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // スレーブからの読み込み処理
        //======================================================================
        // キューイング：スレーブからのデータ読み込み
        sts_val = i2c_master_read(s_queue_info.v_cmd_hndl, pu8_data, t_data_len, I2C_MASTER_LAST_NACK);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイング：ストップコンディションの送信
        sts_val = i2c_master_stop(s_queue_info.v_cmd_hndl);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
        sts_val = i2c_master_cmd_begin(s_queue_info.e_port_no, s_queue_info.v_cmd_hndl, TICKS_TO_WAIT);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // ポート番号クリア
        s_queue_info.e_port_no = I2C_NUM_MAX;
        // I2Cリンクの削除
        i2c_cmd_link_delete(s_queue_info.v_cmd_hndl);

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = true;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了 ※開始はスタートに記述
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_write
 *
 * DESCRIPTION:スレーブへの書き込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   uint8_t*         pu8_data        R   書き込みデータ配列
 *   size_t           t_data_len      R   書き込みサイズ
 *   bool             b_ack_flg       R   ACK返信フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_write(uint8_t* pu8_data, size_t t_data_len, bool b_ack_flg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //======================================================================
        // 入力チェック
        //======================================================================
        // 書き込みサイズ
        if (t_data_len <= 0) {
            break;
        }
        // 順序制御チェック
        if (!s_queue_info.b_order_flg[COM_I2C_MST_WRITE]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // スレーブへの書き込み処理
        //======================================================================
       // キューイング：データの書き込み
        sts_val = sts_i2c_write(s_queue_info.v_cmd_hndl, pu8_data, t_data_len, b_ack_flg);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
        sts_val = i2c_master_cmd_begin(s_queue_info.e_port_no, s_queue_info.v_cmd_hndl, TICKS_TO_WAIT);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = false;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = true;

    } while (false);

    //==========================================================================
    // クリティカルセクション終了 ※開始はスタートに記述
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 完了ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_write_stop
 *
 * DESCRIPTION:スレーブへの書き込み処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   uint8_t*         pu8_data        R   書き込みデータ配列
 *   size_t           t_data_len      R   書き込みサイズ
 *   bool             b_ack_flg       R   ACK返信フラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_write_stop(uint8_t* pu8_data, size_t t_data_len, bool b_ack_flg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //======================================================================
        // 入力チェック
        //======================================================================
        // 書き込みサイズ
        if (t_data_len <= 0) {
            break;
        }
        // 順序制御チェック
        if (!s_queue_info.b_order_flg[COM_I2C_MST_WRITE]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // スレーブへの書き込み処理
        //======================================================================
        // キューイング：データの書き込み
        sts_val = sts_i2c_write(s_queue_info.v_cmd_hndl, pu8_data, t_data_len, b_ack_flg);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイング：ストップコンディションの送信
        sts_val = i2c_master_stop(s_queue_info.v_cmd_hndl);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
        sts_val = i2c_master_cmd_begin(s_queue_info.e_port_no, s_queue_info.v_cmd_hndl, TICKS_TO_WAIT);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // ポート番号クリア
        s_queue_info.e_port_no = I2C_NUM_MAX;
        // I2Cリンクの削除
        i2c_cmd_link_delete(s_queue_info.v_cmd_hndl);

        //======================================================================
        // 順序制御フラグ更新
        //======================================================================
        s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = true;
        s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_END]   = true;
        s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
        s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
        s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;

    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_i2c_mst_ping
 *
 * DESCRIPTION:スレーブへのPing処理
 *
 * PARAMETERS:        Name        RW  Usage
 *   ts_i2c_address   s_address   R   I2Cアドレス（ポート番号とスレーブアドレス）
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_i2c_mst_ping(ts_i2c_address_t s_address) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, IO_I2C_MST_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }

    // 結果ステータス
    esp_err_t sts_val;
    do {
        //======================================================================
        // 実行順序チェック
        //======================================================================
        if (!s_queue_info.b_order_flg[COM_I2C_MST_START]) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // １．クリティカルセクションの開始
        // ２．書き込みモードでスタート
        // ３．ストップ
        //======================================================================
        // キューイング：スタートコンディションとデバイスアドレスの送信
        sts_val = sts_i2c_start(s_address, COM_I2C_MST_WRITE);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイング：ストップコンディションの送信
        sts_val = i2c_master_stop(s_queue_info.v_cmd_hndl);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
        sts_val = i2c_master_cmd_begin(s_queue_info.e_port_no, s_queue_info.v_cmd_hndl, TICKS_TO_WAIT);
        if (sts_val != ESP_OK) {
            // リセットコンディション
            v_reset_condition();
            break;
        }
        // ポート番号
        s_queue_info.e_port_no = I2C_NUM_MAX;
        // I2Cリンクの削除
        i2c_cmd_link_delete(s_queue_info.v_cmd_hndl);

    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    if (xSemaphoreGiveRecursive(s_mutex) == pdFALSE) {
        return ESP_ERR_INVALID_STATE;
    }
    // 結果ステータス返却
    return sts_val;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_i2c_init
 *
 * DESCRIPTION:初期処理
 *
 * PARAMETERS:            Name            RW  Usage
 *   i2c_port_t           e_port_no       R   I2Cポート番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_i2c_init(i2c_port_t e_port_no) {
    //==========================================================================
    // ポート設定
    //==========================================================================
    i2c_config_t* ps_i2c_config = &s_i2c_config_list[e_port_no];
    // パラメータ初期設定
    esp_err_t sts_val = i2c_param_config(e_port_no, ps_i2c_config);
    if (sts_val != ESP_OK ) {
        return sts_val;
    }
    // ピン設定
    sts_val = i2c_set_pin(e_port_no,
                           ps_i2c_config->sda_io_num,
                           ps_i2c_config->scl_io_num,
                           ps_i2c_config->sda_pullup_en,
                           ps_i2c_config->scl_pullup_en,
                           I2C_MODE_MASTER);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // タイムアウト設定（クロック80MHz換算で設定、最大13msec）
    sts_val = i2c_set_timeout(e_port_no, 0xFFFFF);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // ドライバのインストール
    return i2c_driver_install(e_port_no, ps_i2c_config->mode, 0, 0, 0);
}

/*******************************************************************************
 *
 * NAME: sts_i2c_start
 *
 * DESCRIPTION:スタートコンディションのキューイング
 *
 * PARAMETERS:            Name            RW  Usage
 *   ts_i2c_address       s_address       R   I2Cアドレス
 *   uint8_t              u8_read_flg     R   読み込みフラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_i2c_start(ts_i2c_address_t s_address, uint8_t u8_read_flg) {
    //==========================================================================
    // チェック
    //==========================================================================
    // ポート番号
    if (s_address.e_port_no < I2C_NUM_0 || s_address.e_port_no >= I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    // アドレス判定
    bool b_7bit_addr = b_io_i2c_mst_valid_7bit_adress(s_address.u16_address);
    bool b_10bit_addr = b_io_i2c_mst_valid_10bit_adress(s_address.u16_address);
    if (!b_7bit_addr && !b_10bit_addr) {
        return ESP_ERR_INVALID_ARG;
    }
    // 実行順序
    if (!s_queue_info.b_order_flg[COM_I2C_MST_START]) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // キュー情報の編集
    //==========================================================================
    if (s_queue_info.e_port_no != I2C_NUM_MAX) {
        // I2Cリンクの削除
        i2c_cmd_link_delete(s_queue_info.v_cmd_hndl);
    }
    // ポート番号
    s_queue_info.e_port_no = s_address.e_port_no;
    // ハンドラ
    s_queue_info.v_cmd_hndl = i2c_cmd_link_create();

    //==========================================================================
    // キューイング：スタートコンディション送信
    //==========================================================================
    // キューイング：スタートコンディション
    esp_err_t sts_val = i2c_master_start(s_queue_info.v_cmd_hndl);
    if (sts_val != ESP_OK) {
        // 状態リセット
        v_reset_condition();
        // ステータス返信
        return sts_val;
    }

    //==========================================================================
    // キューイング：マスターからのアドレス書き込み
    //==========================================================================
    uint8_t u8_address;
    if (b_7bit_addr) {
        // 下位7ビットをアドレスとして送信
        u8_address = (uint8_t)(s_address.u16_address << 1) | u8_read_flg;
        sts_val = i2c_master_write_byte(s_queue_info.v_cmd_hndl, u8_address, true);
        if (sts_val != ESP_OK) {
            // 状態リセット
            v_reset_condition();
            // ステータス返信
            return sts_val;
        }
    } else {
        // 10bitアドレスの場合には最初に上位2ビットを送信
        u8_address = ((uint8_t)(s_address.u16_address >> 7) & 0xFE) | u8_read_flg;
        sts_val = i2c_master_write_byte(s_queue_info.v_cmd_hndl, u8_address, true);
        if (sts_val != ESP_OK) {
            // 状態リセット
            v_reset_condition();
            // ステータス返信
            return sts_val;
        }
        // 下位8ビットを送信
        u8_address = (uint8_t)s_address.u16_address;
        sts_val = i2c_master_write_byte(s_queue_info.v_cmd_hndl, u8_address, true);
        if (sts_val != ESP_OK) {
            // 状態リセット
            v_reset_condition();
            // ステータス返信
            return sts_val;
        }
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_i2c_write
 *
 * DESCRIPTION:ポインタ参照の問題に対応した書き込み処理
 *
 * PARAMETERS:          Name            RW  Usage
 *   i2c_cmd_handle_t   s_cmd_handle    R   I2Cコマンドハンドラ
 *   uint8_t*           pu8_data        R   書き込みデータ配列
 *   size_t             t_data_len      R   書き込みデータ長
 *   bool               b_ack_en        R   ACKフラグ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_i2c_write(i2c_cmd_handle_t s_cmd_handle,
                                uint8_t* pu8_data,
                                size_t t_data_len,
                                bool b_ack_en) {
    // キューイング：データの書き込み
    esp_err_t sts_val = ESP_OK;
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < t_data_len; u8_idx++) {
        sts_val = i2c_master_write_byte(s_cmd_handle, pu8_data[u8_idx], b_ack_en);
        if (sts_val != ESP_OK) {
            return sts_val;
        }
    }
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_reset_condition
 *
 * DESCRIPTION:I2Cポートとキュー情報のリセット処理
 *
 * PARAMETERS:            Name          RW  Usage
 *
 * RETURNS:
 *
 ******************************************************************************/
static void v_reset_condition() {
    //==========================================================================
    // ドライバ再インストール
    //==========================================================================
    // コマンドハンドラの削除
    if (s_queue_info.e_port_no != I2C_NUM_MAX) {
        i2c_cmd_link_delete(s_queue_info.v_cmd_hndl);
    }
    // ポート番号
    i2c_port_t e_port_no = s_queue_info.e_port_no;
    s_queue_info.e_port_no = I2C_NUM_MAX;
    // ドライバのアンインストール
    i2c_driver_delete(e_port_no);
    // ポート初期化（ドライバインストール含む）
    sts_i2c_init(e_port_no);

    //==========================================================================
    // ステータスクリア
    //==========================================================================
    // 順序制御フラグ更新
    s_queue_info.b_order_flg[COM_I2C_MST_INIT]  = true;
    s_queue_info.b_order_flg[COM_I2C_MST_BEGIN] = true;
    s_queue_info.b_order_flg[COM_I2C_MST_END]   = false;
    s_queue_info.b_order_flg[COM_I2C_MST_START] = true;
    s_queue_info.b_order_flg[COM_I2C_MST_READ]  = false;
    s_queue_info.b_order_flg[COM_I2C_MST_WRITE] = false;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
