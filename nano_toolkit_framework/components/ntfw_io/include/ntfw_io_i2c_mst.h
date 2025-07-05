/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common I2C master library header file
 *
 * CREATED:2025/01/15 05:01:00
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
 * Copyright (c) 2024 Kakuheiki.Nakanohito
 * Released under the MIT license
 * https://opensource.org/licenses/mit-license.php
 *
 ******************************************************************************/
#ifndef  __NTFW_IO_I2C_MSTN_H__
#define  __NTFW_IO_I2C_MSTN_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Block Time */
#ifndef IO_I2C_MST_BLOCK_TIME
    #define IO_I2C_MST_BLOCK_TIME   (portMAX_DELAY)
#endif

/**
 * ポート番号判定
 */
#define b_io_i2c_mst_valid_port(port_num) (port_num >= I2C_NUM_0 && port_num < I2C_NUM_MAX)
/**
 * 7bitアドレス判定
 * 0～0b00000111はシステムで予約、0b01111000以降もシステムで予約
 */
#define b_io_i2c_mst_valid_7bit_adress(address) (address < 0x78 && address > 0x07)
/**
 * 10bitアドレス判定
 * 0b011110XXXXXXXXXXのアドレスは10bitモードなので、アドレス先頭6bitに0b011110の前提
 */
#define b_io_i2c_mst_valid_10bit_adress(address) ((address & 0xFC00) == 0x7800)
/**
 * 10bitアドレスへの変換
 */
#define u16_io_i2c_mst_10bit_adress(address) ((uint16_t)(0x7800 | (address & 0x03FF))

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** I2Cバススピードモード */
typedef enum {
    I2C_MST_FREQ_HZ_LOW  = 10000,   // 低速モード
    I2C_MST_FREQ_HZ_STD  = 100000,  // 標準モード
    I2C_MST_FREQ_HZ_FAST = 400000,  // ファーストモード
    I2C_MST_FREQ_HZ_1M   = 1000000, // 1Mbpsモード
    I2C_MST_FREQ_HZ_MAX,            // MAX
} ts_i2c_mst_freq_mode_t;

/** 構造体：デバイスアドレス */
typedef struct {
    i2c_port_t e_port_no;   // I2Cポート番号
    uint16_t u16_address;   // I2Cスレーブアドレス（10bit時：0b011110～）
} ts_i2c_mst_address_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/
//==============================================================================
// マスター側機能（チェック）
//==============================================================================

//==============================================================================
// マスター側機能（I2Cバスアクセス）
//==============================================================================
/** I2Cバスの初期処理 */
extern esp_err_t sts_io_i2c_mst_bus_init(i2c_port_num_t e_port_no,
                                         ts_i2c_mst_freq_mode_t e_freq,
                                         gpio_num_t e_scl_pin,
                                         gpio_num_t e_sda_pin,
                                         bool b_pullup);
/** I2Cバスのリソース解放処理 */
extern esp_err_t sts_io_i2c_mst_bus_deinit(i2c_port_num_t e_port_no);
/** I2Cバスのデバイス設定追加処理 */
extern esp_err_t sts_io_i2c_mst_add_device(ts_i2c_mst_address_t* ps_address);
/** I2Cバスのデバイス設定リソース解放処理 */
extern esp_err_t sts_io_i2c_mst_del_device(ts_i2c_mst_address_t* ps_address);
/** I2Cバスのタイムアウト時間を設定（バス初期化時デフォルト：-1） */
extern esp_err_t sts_io_i2c_mst_set_timeout_ms(i2c_port_num_t e_port_no, int i_max_wait_ms);
/** トランザクション開始 */
extern esp_err_t sts_io_i2c_mst_tran_begin();
/** トランザクション終了 */
extern esp_err_t sts_io_i2c_mst_tran_end();
/** I2Cスレーブへのデータ送信処理 */
extern esp_err_t sts_io_i2c_mst_tx(ts_i2c_mst_address_t* ps_address,
                                   const uint8_t* pu8_tx_data,
                                   const size_t t_tx_len);
/** I2Cスレーブへのデータ送受信処理 */
extern esp_err_t sts_io_i2c_mst_txrx(ts_i2c_mst_address_t* ps_address,
                                     const uint8_t* pu8_tx_data,
                                     const size_t t_tx_len, 
                                     uint8_t* pu8_rx_data,
                                     size_t t_rx_len);
/** I2Cスレーブからのデータ受信処理 */
extern esp_err_t sts_io_i2c_mst_rx(ts_i2c_mst_address_t* ps_address,
                                   uint8_t* pu8_rx_data,
                                   size_t t_rx_len);
/** I2Cバスとデバイスのハンドル取得 */
extern esp_err_t sts_io_i2c_mst_get_handle(ts_i2c_mst_address_t* ps_address,
                                           i2c_master_bus_handle_t* ps_bus_hndl,
                                           i2c_master_dev_handle_t* ps_dev_hndl);
/** I2Cバスのリセット */
extern esp_err_t sts_io_i2c_mst_bus_reset(i2c_port_num_t e_port_no);
/** I2Cアドレスタイプ取得 */
extern i2c_addr_bit_len_t e_io_i2c_mst_adress_type(uint16_t u16_address);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_IO_I2C_MSTN_
H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
