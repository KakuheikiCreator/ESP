/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :GPIO Utility functions header file
 *
 * CREATED:2023/03/10 08:10:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION: GPIOユーティリティ関係の関数群
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
#ifndef  __NTFW_IO_GPIO_UTIL_H__
#define  __NTFW_IO_GPIO_UTIL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <driver/spi_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali_scheme.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
#ifndef NTFW_ADC_DEFAULT_VREF
#define NTFW_ADC_DEFAULT_VREF   (1100)  // ADC参照電圧（mV）
//#define NTFW_ADC_DEFAULT_VREF   (3300)  // ADC参照電圧（mV）
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** ADC較正モード */
typedef enum {
    NTFW_ADC_CALIBRATION_NONE = 0,       // 較正無し
#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    NTFW_ADC_CALIBRATION_CURVE_FITTING = 1,  // カーブフィッティング
#endif
#ifdef ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    NTFW_ADC_CALIBRATION_LINE_FITTING = 2,   // ラインフィッティング
#endif
} te_adc_calibration_mode_t;

/** 構造体：ADCワンショットコンテキスト */
typedef struct {
    adc_unit_t e_unit;                              // ADCユニット
    adc_atten_t e_atten;                            // 減衰（DB）
    adc_bitwidth_t e_bitwidth;                      // ADC量子化ビット数
    te_adc_calibration_mode_t e_cal_mode;           // 較正モード
    adc_oneshot_unit_handle_t s_handle;             // ADCユニットのハンドル
    adc_cali_handle_t* ps_calibration_handle;       // 較正ハンドル
} ts_adc_oneshot_context;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/


//==============================================================================
// ワンショットADC読み込み関連処理
//==============================================================================
/** ADCユニットコンテキストの生成処理 */
extern ts_adc_oneshot_context* ps_adc_oneshot_ctx(adc_unit_t e_unit,
                                                   soc_periph_adc_digi_clk_src_t e_clk_src,
                                                   adc_ulp_mode_t e_ulp_mode);
/** ADCユニットコンテキスト（較正）の生成処理 */
extern ts_adc_oneshot_context* ps_adc_oneshot_calibration_ctx(adc_unit_t e_unit,
                                                               soc_periph_adc_digi_clk_src_t e_clk_src,
                                                               adc_ulp_mode_t e_ulp_mode,
                                                               adc_atten_t e_atten);
/** ADCユニットコンテキストの削除処理 */
extern esp_err_t sts_adc_oneshot_delete_ctx(ts_adc_oneshot_context* ps_ctx);
/** ADCチャンネル設定処理 */
extern esp_err_t sts_adc_oneshot_config_channel(ts_adc_oneshot_context* ps_ctx,
                                                 adc_channel_t e_adc_channel,
                                                 adc_atten_t e_atten,
                                                 adc_bitwidth_t e_bitwidth);

/** ADCのワンショット読み込み（RAWデータ） */
extern int i_adc_oneshot_raw_data(ts_adc_oneshot_context* ps_ctx,
                                   adc_channel_t e_adc_channel);
/** ADCのワンショット読み込み（較正済み電圧） */
extern int i_adc_oneshot_voltage(ts_adc_oneshot_context* ps_ctx, adc_channel_t e_adc_channel);

//==============================================================================
// SPI master 関連処理
//==============================================================================
/** SPI master initialize */
esp_err_t sts_spi_mst_bus_initialize(spi_host_device_t e_host_id,
                                     const spi_bus_config_t* ps_bus_config,
                                     spi_dma_chan_t e_dma_chan,
                                     bool b_pullup);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_IO_GPIO_UTIL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
