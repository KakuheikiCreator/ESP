/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common I2C library header file
 *
 * CREATED:2019/11/17 12:09:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:I2Cの共通系ライブラリ
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
#ifndef  __NTFW_IO_I2C_MST_H__
#define  __NTFW_IO_I2C_MST_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <driver/i2c.h>


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
#define b_io_i2c_mst_valid_port(port_num) (port_num == I2C_NUM_0 || port_num == I2C_NUM_1)
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
    I2C_FREQ_HZ_LOW  = 10000,	// 低速モード
    I2C_FREQ_HZ_STD  = 100000,	// 標準モード
    I2C_FREQ_HZ_FAST = 400000,	// ファーストモード
    I2C_FREQ_HZ_1M   = 1000000,	// 1Mbpsモード
} ts_i2c_freq_mode_t;

/** 構造体：デバイスアドレス */
typedef struct {
    i2c_port_t e_port_no;		// I2Cポート番号
    uint16_t u16_address;       // I2Cスレーブアドレス（10bit時：0b011110～）
} ts_i2c_address_t;

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
/** I2Cマスタ初期化処理 */
extern esp_err_t sts_io_i2c_mst_init(i2c_port_t e_port_no,
                                      ts_i2c_freq_mode_t e_freq,
                                      gpio_num_t e_scl_pin,
                                      gpio_num_t e_sda_pin,
                                      gpio_pullup_t e_pullup);
/** トランザクション開始 */
extern esp_err_t sts_io_i2c_mst_begin();
/** トランザクション終了 */
extern esp_err_t sts_io_i2c_mst_end();
/** 読み込みスタートコンディションの送信 */
extern esp_err_t sts_io_i2c_mst_start_read(ts_i2c_address_t s_address);
/** 書き込みスタートコンディションの送信 */
extern esp_err_t sts_io_i2c_mst_start_write(ts_i2c_address_t s_address);
/** スレーブからの読み込み処理 */
extern esp_err_t sts_io_i2c_mst_read(uint8_t *pu8_data, size_t t_data_len);
/** スレーブからの読み込み処理 */
extern esp_err_t sts_io_i2c_mst_read_stop(uint8_t *pu8_data, size_t t_data_len);
/** スレーブへの書き込み処理（配列） */
extern esp_err_t sts_io_i2c_mst_write(uint8_t* pu8_data, size_t t_data_len, bool b_ack_flg);
/** スレーブへの書き込み処理（配列） */
extern esp_err_t sts_io_i2c_mst_write_stop(uint8_t* pu8_data, size_t t_data_len, bool b_ack_flg);
/** スレーブへのPing処理 */
extern esp_err_t sts_io_i2c_mst_ping(ts_i2c_address_t s_address);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_IO_I2C_MST_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
