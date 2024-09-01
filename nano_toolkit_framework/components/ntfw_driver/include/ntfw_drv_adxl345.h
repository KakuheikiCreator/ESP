/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :ADXL345 Driver functions header file
 *
 * CREATED:2019/12/05 22:11:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:ADXL345 3-AXIS SENSOR driver
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
#ifndef  __NTFW_DRV_ADXL345_H__
#define  __NTFW_DRV_ADXL345_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_system.h>
#include "ntfw_io_i2c_master.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// I2C Address
#define I2C_ADDR_ADXL345_H     (0x1D)
#define I2C_ADDR_ADXL345_L     (0x53)

// 関数定義：FIFO_TRIG取得
#define b_adxl345_fifo_trigger(ps_register) ((bool)(ps_register->u8_fifo_status >> 7))
// 関数定義：FIFOプールサイズ取得
#define u8ADXL345_fifo_pool_size(ps_register) ((uint8_t)(ps_register->u8_fifo_status & 0x1F))
// 関数定義：未参照データ有無判定
#define b_adxl345_data_ready(ps_register) ((bool)(ps_register->u8_int_source >> 7))
// 関数定義：スリープ判定
#define b_adxl345_sleep(ps_register) ((bool)((ps_register->u8_act_tap_sts >> 3) & 0x01))

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** FIFOモード */
typedef enum {
    DRV_ADXL345_MODE_BYPASS   = 0x00,	// FIFOモード：バイパス
    DRV_ADXL345_MODE_FIFO     = 0x01,	// FIFOモード：FIFO
    DRV_ADXL345_MODE_STREAM   = 0x02,	// FIFOモード：ストリーム
    DRV_ADXL345_MODE_TRIGGER  = 0x03,	// FIFOモード：トリガー
} te_adxl345_mode_t;

/** 加速度レンジ */
typedef enum {
    DRV_ADXL345_2G            = 0x00,
    DRV_ADXL345_4G            = 0x01,
    DRV_ADXL345_8G            = 0x02,
    DRV_ADXL345_16G           = 0x03,
} te_adxl345_range_t;

/**
 * ADXL345のレジスタ情報
 */
typedef struct {
    uint8_t u8_tap_thresh;		// タップ閾値（加速度）
    int8_t  i8_offset_x;		// オフセット（X）
    int8_t  i8_offset_y;		// オフセット（Y）
    int8_t  i8_offset_z;		// オフセット（Z）
    uint8_t u8_tap_duration;	// タップ閾値（時間）
    uint8_t u8_tap_latency;		// ダブルタップ閾値（最小間隔）
    uint8_t u8_tap_window;		// ダブルタップ閾値（最大間隔）
    uint8_t u8_act_thresh;		// アクティブ閾値（加速度）
    uint8_t u8_inact_thresh;	// インアクティブ閾値（加速度）
    uint8_t u8_inact_time;		// インアクティブ閾値（時間）：0-255秒
    uint8_t u8_act_inact_ctl;	// アクティブ・インアクティブ軸
    uint8_t u8_ff_thresh;		// 自由落下閾値（加速度）
    uint8_t u8_ff_time;			// 自由落下閾値（時間）
    uint8_t u8_tap_axes;		// タップ軸（シングルタップ・ダブルタップ）
    uint8_t u8_act_tap_sts;		// アクティブタップステータス
    uint8_t u8_bw_rate;			// 省電力・データレートコントロール
    uint8_t u8_power_crl;		// アクティブ・インアクティブ連携・スリープモード等
    uint8_t u8_int_enable;		// イベント割り込みの有効無効
    uint8_t u8_int_map;			// 各イベント発生時のIntピンマップ
    uint8_t u8_int_source;		// イベント発生のステータス
    uint8_t u8_data_format;		// SPIやI2C等の接続方式を指定
    uint8_t u8_data_x0;			// X軸加速度下位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_data_x1;			// X軸加速度上位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_data_y0;			// Y軸加速度下位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_data_y1;			// Y軸加速度上位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_data_z0;			// Z軸加速度下位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_data_z1;			// Z軸加速度上位桁（FIFOモード時は一番古いデータ）
    uint8_t u8_fifo_ctl;		// FIFOコントロール
    uint8_t u8_fifo_status;		// FIFOステータス
} ts_adxl345_register_t;

/** ３軸データ */
typedef struct {
    int16_t i16_data_x;			// データ（X軸）
    int16_t i16_data_y;			// データ（Y軸）
    int16_t i16_data_z;			// データ（Z軸）
} ts_adxl345_axes_data_t;

/** ３軸ステータス */
typedef struct {
    bool b_status_x;			// ステータス（X軸）
    bool b_status_y;			// ステータス（Y軸）
    bool b_status_z;			// ステータス（Z軸）
} ts_adxl345_axes_sts_t;

/** 割り込みステータス */
typedef struct {
    bool b_sts_data_ready;		// ステータス（データ読み込み）
    bool b_sts_single_tap;		// ステータス（シングルタップ）
    bool b_sts_double_tap;		// ステータス（ダブルタップ）
    bool b_sts_activity;		// ステータス（アクティブ）
    bool b_sts_in_activity;		// ステータス（インアクティブ）
    bool b_sts_free_fall;		// ステータス（自由落下）
    bool b_sts_watermark;		// ステータス（FIFOプールサイズ閾値超え）
    bool b_sts_overrun;			// ステータス（オーバーラン）
} ts_adxl345_interrupt_sts_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
/** センサー初期化処理 */
extern esp_err_t sts_adxl345_init(ts_i2c_address_t s_address, uint8_t u8_rate);
/** デフォルト値編集 */
extern void v_adxl345_edit_default(ts_adxl345_register_t* ps_register);
/** レジスタ情報読み込み */
extern esp_err_t sts_adxl345_read(ts_i2c_address_t s_address, ts_adxl345_register_t* ps_register);
/** 加速度（XYZ軸）読み込み */
extern esp_err_t sts_adxl345_read_g(ts_i2c_address_t s_address, ts_adxl345_axes_data_t* ps_axes_data);
/** レジスタ情報書き込み */
extern esp_err_t sts_adxl345_write(ts_i2c_address_t s_address, ts_adxl345_register_t* ps_register);
/** 較正処理 */
extern esp_err_t sts_adxl345_calibration(ts_i2c_address_t s_address,
                                          int8_t i8_abs_x,
                                          int8_t i8_abs_y,
                                          int8_t i8_abs_z);
/** ゼロイング処理 */
extern esp_err_t sts_adxl345_zeroing(ts_i2c_address_t s_address);
/** 加速度取得（XYZ軸） */
extern ts_adxl345_axes_data_t s_adxl345_g_data(ts_adxl345_register_t* ps_register);
/** アクティブイベントステータス取得 */
extern ts_adxl345_axes_sts_t s_adxl345_act_status(ts_adxl345_register_t* ps_register);
/** タップベントステータス取得 */
extern ts_adxl345_axes_sts_t s_adxl345_tap_status(ts_adxl345_register_t* ps_register);
/** 割り込みステータス取得 */
extern ts_adxl345_interrupt_sts_t s_adxl345_int_status(ts_adxl345_register_t* ps_register);
/** 設定：オフセット */
extern esp_err_t sts_adxl345_set_offset(ts_i2c_address_t s_address, int8_t i8_ofs_x, int8_t i8_ofs_y, int8_t i8_ofs_z);
/** 設定：電力モード・データレート */
extern esp_err_t sts_adxl345_set_bw_rate(ts_i2c_address_t s_address, bool b_low_pwr, uint8_t u8Rate);
/** 設定：出力フォーマット（Gレンジ、精度、左右寄せ、割り込み） */
extern esp_err_t sts_adxl345_set_data_format(ts_i2c_address_t s_address,
                                              te_adxl345_range_t e_range,
                                              bool b_full_res, bool b_justify, bool b_int_inv);
/** 設定：出力制御（セルフテスト、SPI出力モード） */
extern esp_err_t sts_adxl345_set_output_ctl(ts_i2c_address_t s_address, bool b_self_test, bool b_spi_mode);
/** 設定：スリープ（オートスリープ、スリープ、スリープ時周波数） */
extern esp_err_t sts_adxl345_set_sleep(ts_i2c_address_t s_address, bool b_auto_sleep, bool b_sleep, uint8_t u8_sleep_rate);
/** 設定：計測モード（スタンバイ、アクティブ・インアクティブリンク） */
extern esp_err_t sts_adxl345_set_measure(ts_i2c_address_t s_address, bool b_measure, bool b_link);
/** 設定：FIFO制御（モード、トリガ出力先、プールサイズ閾値） */
extern esp_err_t sts_adxl345_set_fifo_ctl(ts_i2c_address_t s_address,
                                           te_adxl345_mode_t e_mode,
                                           bool b_trigger,
                                           uint8_t u8_samples);
/** 設定：有効割り込み編集（タップ・アクティブ・自由落下等） */
extern esp_err_t sts_adxl345_set_int_enable(ts_i2c_address_t s_address, ts_adxl345_interrupt_sts_t s_status);
/** 設定：割り込み出力先編集（タップ・アクティブ・自由落下等） */
extern esp_err_t sts_adxl345_set_int_map(ts_i2c_address_t s_address, ts_adxl345_interrupt_sts_t s_status);
/** 設定：タップ閾値編集（加速度、継続時間） */
extern esp_err_t sts_adxl345_set_tap_threshold(ts_i2c_address_t s_address, uint8_t u8_threshold, uint8_t u8_duration);
/** 設定：ダブルタップ閾値編集（間隔、測定期間） */
extern esp_err_t sts_adxl345_set_dbl_tap_threshold(ts_i2c_address_t s_address, uint8_t u8_latent, uint8_t u8_window);
/** タップ設定編集（タップ間のタップ有効無効、タップ有効軸） */
extern esp_err_t sts_adxl345_set_tap_axes(ts_i2c_address_t s_address, bool b_suppress, ts_adxl345_axes_sts_t s_axes_sts);
/** アクティブ制御編集（加速度、絶対／相対、有効軸） */
extern esp_err_t sts_adxl345_set_active_ctl(ts_i2c_address_t s_address,
                                             uint8_t u8_act_th,
                                             bool b_acdc,
                                             ts_adxl345_axes_sts_t s_axes_sts);
/** インアクティブ制御編集（加速度、継続時間、絶対／相対、有効軸） */
extern esp_err_t sts_adxl345_set_in_active_ctl(ts_i2c_address_t s_address,
                                                uint8_t u8_inact_th,
                                                uint8_t u8_inact_time,
                                                bool b_acdc,
                                                ts_adxl345_axes_sts_t s_axes_sts);
/** 自由落下閾値編集（加速度、継続時間） */
extern esp_err_t sts_adxl345_set_free_fall(ts_i2c_address_t s_address, uint8_t u8_thresh_ff, uint8_t u8_time_ff);
/** 加速度算出処理（ニュートン法で概算したXYZ軸の合成値） */
extern int16_t i16_adxl345_conv_g_val(ts_adxl345_axes_data_t *s_axes_data, bool b_round_up);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_DRV_ADXL345_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
