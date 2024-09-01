/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :LIS3DH Driver functions header file
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
#ifndef  __NTFW_DRV_LIS3DH_H__
#define  __NTFW_DRV_LIS3DH_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ntfw_io_i2c_master.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// I2C Address
#define I2C_ADDR_LIS3DH_L       (0x18)
#define I2C_ADDR_LIS3DH_H       (0x19)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** レジスタアドレス */
typedef enum {
    DRV_LIS3DH_ADR_STATUS_REG_AUX   = 0x07,	// 各軸のOver Runとデータの準備状況
    DRV_LIS3DH_ADR_OUT_ADC1_L       = 0x08,	// ADC
    DRV_LIS3DH_ADR_OUT_ADC1_H       = 0x09,	// ADC
    DRV_LIS3DH_ADR_OUT_ADC2_L       = 0x0A,	// ADC
    DRV_LIS3DH_ADR_OUT_ADC2_H       = 0x0B,	// ADC
    DRV_LIS3DH_ADR_OUT_ADC3_L       = 0x0C,	// ADC
    DRV_LIS3DH_ADR_OUT_ADC3_H       = 0x0D,	// ADC
    DRV_LIS3DH_ADR_INT_COUNTER_REG  = 0x0E,	//
    DRV_LIS3DH_ADR_WHO_AM_I         = 0x0F,	// デバイスID
    DRV_LIS3DH_ADR_TEMP_CFG_REG     = 0x1F,	// ADCと温度計設定
    DRV_LIS3DH_ADR_CTRL_REG1        = 0x20,	// 制御レジスタ１：データレート、省電力モード、各軸の有効無効
    DRV_LIS3DH_ADR_CTRL_REG2        = 0x21,	// 制御レジスタ２：ハイパスフィルタモード、ハイパスフィルタカットオフ周波数選択、フィルタ有効化
    DRV_LIS3DH_ADR_CTRL_REG3        = 0x22,	// 制御レジスタ３：割り込みの有効無効設定
    DRV_LIS3DH_ADR_CTRL_REG4        = 0x23,	// 制御レジスタ４：ブロックデータの更新設定、ビッグエンディアン/リトルエンディアン、レンジ、高解像度、セルフテスト、SPI
    DRV_LIS3DH_ADR_CTRL_REG5        = 0x24,	// 制御レジスタ５：メモリコンテンツの再起動、FIFO有効無効、割り込みラッチ
    DRV_LIS3DH_ADR_CTRL_REG6        = 0x25,	// 制御レジスタ６：割り込み、
    DRV_LIS3DH_ADR_REFERENCE        = 0x26,	// 参照割り込み制御
    DRV_LIS3DH_ADR_STATUS_REG2      = 0x27,	// ステータス：オーバーラン、データ準備
    DRV_LIS3DH_ADR_OUT_X_L          = 0x28,	// 加速度（X軸）
    DRV_LIS3DH_ADR_OUT_X_H          = 0x29,	// 加速度（X軸）
    DRV_LIS3DH_ADR_OUT_Y_L          = 0x2A,	// 加速度（Y軸）
    DRV_LIS3DH_ADR_OUT_Y_H          = 0x2B,	// 加速度（Y軸）
    DRV_LIS3DH_ADR_OUT_Z_L          = 0x2C,	// 加速度（Z軸）
    DRV_LIS3DH_ADR_OUT_Z_H          = 0x2D,	// 加速度（Z軸）
    DRV_LIS3DH_ADR_FIFO_CTRL_REG    = 0x2E,	// FIFO設定、トリガー発生源（INT1/INT2）
    DRV_LIS3DH_ADR_FIFO_SRC_REG     = 0x2F,	//
    DRV_LIS3DH_ADR_INT1_CFG         = 0x30,	//
    DRV_LIS3DH_ADR_INT1_SOURCE      = 0x31,	//
    DRV_LIS3DH_ADR_INT1_THS         = 0x32,	//
    DRV_LIS3DH_ADR_INT1_DURATION    = 0x33,	//
    DRV_LIS3DH_ADR_CLICK_CFG        = 0x38,	//
    DRV_LIS3DH_ADR_CLICK_SRC        = 0x39,	//
    DRV_LIS3DH_ADR_CLICK_THS        = 0x3A,	//
    DRV_LIS3DH_ADR_TIME_LIMIT       = 0x3B,	//
    DRV_LIS3DH_ADR_TIME_LATENCY     = 0x3C,	//
    DRV_LIS3DH_ADR_TIME_WINDOW      = 0x3D	//
} te_lis3dh_reg_addr_t;

/** データレート */
typedef enum {
    DRV_LIS3DH_RATE_PWR_DOWN    = 0x00,	// パワーダウン
    DRV_LIS3DH_RATE_LPW_1HZ     = 0x10,	// Normal / low power mode (1 Hz)
    DRV_LIS3DH_RATE_LPW_10HZ    = 0x20,	// Normal / low power mode (10 Hz)
    DRV_LIS3DH_RATE_LPW_25HZ    = 0x30,	// Normal / low power mode (25 Hz)
    DRV_LIS3DH_RATE_LPW_50HZ    = 0x40,	// Normal / low power mode (50 Hz)
    DRV_LIS3DH_RATE_LPW_100H    = 0x50,	// Normal / low power mode (100 Hz)
    DRV_LIS3DH_RATE_LPW_200HZ   = 0x60,	// Normal / low power mode (200 Hz)
    DRV_LIS3DH_RATE_LPW_400HZ   = 0x70,	// Normal / low power mode (400 Hz)
    DRV_LIS3DH_RATE_LPW_1600HZ  = 0x80,	// Normal / low power mode (1600 Hz)
    DRV_LIS3DH_RATE_LPW_5000HZ  = 0x90	// Normal / low power mode (5000 Hz)
} te_lis3dh_data_rate_t;

/** データレンジ */
typedef enum {
    DRV_LIS3DH_RANGE_2G  = 0x00,	// +/-2G
    DRV_LIS3DH_RANGE_4G  = 0x01,	// +/-4G
    DRV_LIS3DH_RANGE_8G  = 0x02,	// +/-8G
    DRV_LIS3DH_RANGE_16G = 0x03,	// +/-16G
} te_lis3dh_range_t;

/** ハイパスフィルタモード */
typedef enum {
    DRV_LIS3DH_HPF_RESET_READ = 0x00,	// Normal mode (reset reading HP_RESET_FILTER)
    DRV_LIS3DH_HPF_REFERENCE  = 0x01,	// Reference signal for filtering
    DRV_LIS3DH_HPF_NORMAL     = 0x02,	// Normal mode
    DRV_LIS3DH_HPF_AUTO_RESET = 0x03,	// Autoreset on interrupt event
} te_lis3dh_hpf_mode_t;

/** FIFOモード */
typedef enum {
    DRV_LIS3DH_MODE_BYPASS    = 0x00,	// FIFOモード：バイパス
    DRV_LIS3DH_MODE_FIFO      = 0x01,	// FIFOモード：FIFO
    DRV_LIS3DH_MODE_STREAM    = 0x02,	// FIFOモード：ストリーム
    DRV_LIS3DH_MODE_TRIGGER   = 0x03,	// FIFOモード：トリガー
} te_lis3dh_fifo_mode_t;

/** ３軸データ */
typedef struct {
    int16_t i16_data_x;		// データ（X軸）
    int16_t i16_data_y;		// データ（Y軸）
    int16_t i16_data_z;		// データ（Z軸）
} ts_lis3dh_axes_data_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
/** 判定：who am i */
extern esp_err_t sts_lis3dh_who_am_i(ts_i2c_address_t s_address);
/** 書き込み：レート */
extern esp_err_t sts_lis3dh_set_rate(ts_i2c_address_t s_address, bool b_low_pwr, te_lis3dh_data_rate_t e_rate);
/** 書き込み：３軸の有効無効 */
extern esp_err_t sts_lis3dh_set_enable_axis(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z);
/** 書き込み：HPCF */
extern esp_err_t sts_lis3dh_set_hpcf(ts_i2c_address_t s_address, te_lis3dh_hpf_mode_t e_hpf_mode, uint8_t u8_hpcf, bool b_fds);
/** 書き込み：データ更新方式 */
extern esp_err_t sts_lis3dh_set_upd_settings(ts_i2c_address_t s_address, bool b_upd_type, bool b_format);
/** 書き込み：データレンジ */
extern esp_err_t sts_lis3dh_set_range(ts_i2c_address_t s_address, te_lis3dh_range_t e_range, bool b_hr);
/** 書き込み：FIFOモード */
extern esp_err_t sts_lis3dh_set_fifo_mode(ts_i2c_address_t s_address, te_lis3dh_fifo_mode_t e_fifo_mode);
/** 読み込み：FIFOカウント */
extern esp_err_t sts_lis3dh_fifo_cnt(ts_i2c_address_t s_address, uint8_t* pu8_fifo_cnt);
/** 読み込み：ステータス */
extern esp_err_t sts_lis3dh_status(ts_i2c_address_t s_address, uint8_t* pu8_status);
/** 読み込み：加速度（XYZ軸） */
extern esp_err_t sts_lis3dh_acceleration(ts_i2c_address_t s_address, ts_lis3dh_axes_data_t* ps_axes_data);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_DRV_LIS3DH_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
