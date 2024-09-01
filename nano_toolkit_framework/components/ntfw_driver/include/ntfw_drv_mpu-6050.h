/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :MPU-6050 Driver functions header file
 *
 * CREATED:2019/12/26 02:08:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:MPU-6050 Motion Processing Unit driver
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
#ifndef  __NTFW_DRV_MPU_6050_H__
#define  __NTFW_DRV_MPU_6050_H__

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
#define I2C_ADDR_MPU_6050_L     (0x68)
#define I2C_ADDR_MPU_6050_H     (0x69)

// レンジ事の角速度：LSB/(º/s)
#define DRV_MPU_6050_GYRO_LSB_250     (131)
#define DRV_MPU_6050_GYRO_LSB_500     (65.5)
#define DRV_MPU_6050_GYRO_LSB_1000    (32.8)
#define DRV_MPU_6050_GYRO_LSB_2000    (16.4)

// レンジ事の加速度：LSB/g
#define DRV_MPU_6050_ACCEL_LSB_2G     (16384)
#define DRV_MPU_6050_ACCEL_LSB_4G     (8192)
#define DRV_MPU_6050_ACCEL_LSB_8G     (4096)
#define DRV_MPU_6050_ACCEL_LSB_16G    (2048)

/**
 *  変換関数：摂氏
 *  340 LSB/degrees and Offset 35 degrees and Difference -521
 *  ((temperature + (35 * 340) - 521) / 340.0)
 */
#define f_mpu_6050_celsius(i16_temp) (((float)i16_temp + 11379.0) / 340.0)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 加速度レンジ */
typedef enum {
    DRV_MPU_6050_ACCEL_RANGE_2G     = 0x00,
    DRV_MPU_6050_ACCEL_RANGE_4G     = 0x01,
    DRV_MPU_6050_ACCEL_RANGE_8G     = 0x02,
    DRV_MPU_6050_ACCEL_RANGE_16G    = 0x03
} te_mpu_6050_accel_range_t;

/** ジャイロレンジ */
typedef enum {
    DRV_MPU_6050_GYRO_RANGE_250     = 0x00,
    DRV_MPU_6050_GYRO_RANGE_500     = 0x01,
    DRV_MPU_6050_GYRO_RANGE_1000    = 0x02,
    DRV_MPU_6050_GYRO_RANGE_2000    = 0x03
} te_mpu_6050_gyro_range_t;

/** 加速度/ジャイロのローパスフィルター */
typedef enum {
    DRV_MPU_6050_LPF_260_256  = 0x00,
    DRV_MPU_6050_LPF_184_188  = 0x01,
    DRV_MPU_6050_LPF_094_098  = 0x02,
    DRV_MPU_6050_LPF_044_042  = 0x03,
    DRV_MPU_6050_LPF_021_020  = 0x04,
    DRV_MPU_6050_LPF_010_010  = 0x05,
    DRV_MPU_6050_LPF_005_005  = 0x06,
    DRV_MPU_6050_LPF_RESERVED = 0x07
} te_mpu_6050_accel_lpf_t;

/** 加速度ハイパスフィルター */
typedef enum {
    DRV_MPU_6050_ACCEL_HPF_RESET    = 0x00,
    DRV_MPU_6050_ACCEL_HPF_5HZ      = 0x01,
    DRV_MPU_6050_ACCEL_HPF_2P5HZ    = 0x02,
    DRV_MPU_6050_ACCEL_HPF_1P25HZ   = 0x03,
    DRV_MPU_6050_ACCEL_HPF_0P63HZ   = 0x04,
    DRV_MPU_6050_ACCEL_HPF_HOLD     = 0x07
} te_mpu_6050_accel_hpf_t;

/** クロック選択 */
typedef enum {
    DRV_MPU_6050_CLK_INTERNAL  = 0x00,
    DRV_MPU_6050_CLK_PLL_X     = 0x01,
    DRV_MPU_6050_CLK_PLL_Y     = 0x02,
    DRV_MPU_6050_CLK_PLL_Z     = 0x03,
    DRV_MPU_6050_CLK_EXT_0     = 0x04,
    DRV_MPU_6050_CLK_EXT_1     = 0x05,
    DRV_MPU_6050_CLK_RESERVED  = 0x06,
    DRV_MPU_6050_CLK_STOP      = 0x07
} te_mpu_6050_clock_t;

/** スリープタイプ */
typedef enum {
    DRV_MPU_6050_SLEEP_CYCLE_0125  = 0x00,	// 1.25Hz
    DRV_MPU_6050_SLEEP_CYCLE_0250  = 0x01,	// 2.5Hz
    DRV_MPU_6050_SLEEP_CYCLE_0500  = 0x02,	// 5Hz
    DRV_MPU_6050_SLEEP_CYCLE_1000  = 0x03,	// 10Hz
    DRV_MPU_6050_SLEEP_CYCLE_SLEEP = 0x04,	// スリープ
    DRV_MPU_6050_SLEEP_CYCLE_NONE  = 0x08	// スリープサイクル無効
} te_mpu_6050_cycle_t;

/** ３軸データ */
typedef struct {
    int16_t i16_data_x;			// データ（X軸）
    int16_t i16_data_y;			// データ（Y軸）
    int16_t i16_data_z;			// データ（Z軸）
} ts_mpu_6050_axes_data_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
/** 初期化処理 */
extern esp_err_t sts_mpu_6050_init(ts_i2c_address_t s_address, te_mpu_6050_accel_range_t e_accel_range, te_mpu_6050_gyro_range_t e_gyro_range);
/** ジャイロのサンプリングレート分割数設定処理 */
extern esp_err_t sts_mpu_6050_set_smplrt_div(ts_i2c_address_t s_address, uint8_t u8_div);
/** ローパスフィルター設定 */
extern esp_err_t sts_mpu_6050_set_dlpf_cfg(ts_i2c_address_t s_address, te_mpu_6050_accel_lpf_t u8_dlpf_cfg);
/** 加速度セルフテスト */
extern esp_err_t sts_mpu_6050_set_accel_self_test(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z);
/** 加速度レンジ設定 */
extern esp_err_t sts_mpu_6050_set_accel_range(ts_i2c_address_t s_address, te_mpu_6050_accel_range_t e_range);
/** ハイパスフィルタ設定 */
extern esp_err_t sts_mpu_6050_set_accel_hpf(ts_i2c_address_t s_address, te_mpu_6050_accel_hpf_t e_hpf);
/** ジャイロセルフテスト */
extern esp_err_t sts_mpu_6050_set_gyro_self_test(ts_i2c_address_t s_address, bool b_x, bool b_y, bool b_z);
/** ジャイロレンジ設定 */
extern esp_err_t sts_mpu_6050_set_gyro_range(ts_i2c_address_t s_address, te_mpu_6050_gyro_range_t e_range);
/** FIFO有効無効設定 */
extern esp_err_t sts_mpu_6050_set_fifo_enable(ts_i2c_address_t s_address, bool b_temp, bool b_x, bool b_y, bool b_z, bool b_accel);
/** クロック設定 */
extern esp_err_t sts_mpu_6050_set_clock(ts_i2c_address_t s_address, te_mpu_6050_clock_t e_clock);
/** スリープサイクル設定 */
extern esp_err_t sts_mpu_6050_set_sleep_cycle(ts_i2c_address_t s_address, te_mpu_6050_cycle_t e_cycle);
/** 加速度（XYZ軸）読み込み */
extern esp_err_t sts_mpu_6050_read_accel(ts_i2c_address_t s_address, ts_mpu_6050_axes_data_t* ps_axes_data);
/** 温度読み込み */
extern esp_err_t sts_mpu_6050_read_celsius(ts_i2c_address_t s_address, float* pf_temp);
/** ジャイロ（XYZ軸）読み込み */
extern esp_err_t sts_mpu_6050_read_gyro(ts_i2c_address_t s_address, ts_mpu_6050_axes_data_t* ps_axes_data);
/** FIFOリセット */
extern esp_err_t sts_mpu_6050_fifo_reset(ts_i2c_address_t s_address);
/** デバイスリセット */
extern esp_err_t sts_mpu_6050_device_reset(ts_i2c_address_t s_address);
/** who am i */
extern esp_err_t sts_mpu_6050_who_am_i(ts_i2c_address_t s_address);
/** FIFOカウント */
extern esp_err_t sts_mpu_6050_fifo_cnt(ts_i2c_address_t s_address, int16_t* pi16_cnt);
/** FIFOデータ */
extern esp_err_t sts_mpu_6050_fifo_data(ts_i2c_address_t s_address, int16_t* pi16_data);
/** 加速度（XYZ軸）ゼロイング */
extern esp_err_t sts_mpu_6050_zeroing_accel(ts_i2c_address_t s_address);
/** ジャイロ（XYZ軸）ゼロイング */
extern esp_err_t sts_mpu_6050_zeroing_gyro(ts_i2c_address_t s_address);
/** ゼロイングクリア */
extern void v_mpu_6050_zeroing_clear();
/** ３軸の加速度の合成値 */
extern int16_t i16_mpu_6050_composite_value(ts_mpu_6050_axes_data_t* ps_axes_data, bool b_round_up);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_DRV_MPU_6050_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
