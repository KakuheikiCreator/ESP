/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :GPIO Utility functions header file
 *
 * CREATED:2023/03/10 08:10:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION: GPIOユーティリティ関係の関数群
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
#ifndef  __NTFW_IO_GPIO_UTIL_H__
#define  __NTFW_IO_GPIO_UTIL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali_scheme.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
#ifndef GPIO_ADC_DEFAULT_VREF
#define GPIO_ADC_DEFAULT_VREF   (1100)  // ADC参照電圧（mV）
#endif

//==============================================================================
// IO level
//==============================================================================
#define GPIO_LEVEL_LOW  (0)  // GPIO LOW level
#define GPIO_LEVEL_HIGH (1)  // GPIO HIGH level

//==============================================================================
// IO pin map
//==============================================================================
#if defined(CONFIG_IDF_TARGET_ESP32)
/** 入力ピンマップ */
#define GPIO_INPUT_PIN_MAP  (0x000000FF0EEFFFFF)
/** 出力ピンマップ */
#define GPIO_OUTPUT_PIN_MAP (0x000000030EEFFFFF)
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
/** 入力ピンマップ */
#define GPIO_INPUT_PIN_MAP  (0x00000000003C07FF)
/** 出力ピンマップ */
#define GPIO_OUTPUT_PIN_MAP (0x00000000003C07FF)
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
/** 入力ピンマップ */
#define GPIO_INPUT_PIN_MAP  (0x00000000007FFFFF)
/** 出力ピンマップ */
#define GPIO_OUTPUT_PIN_MAP (0x00000000007FFFFF)
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
/** 入力ピンマップ */
#define GPIO_INPUT_PIN_MAP  (0x0001FFFB0E3FFFFF)
/** 出力ピンマップ */
#define GPIO_OUTPUT_PIN_MAP (0x0001FFFB0E3FFFFF)
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif
/** 入出力ピンマップ */
#define GPIO_IO_PIN_MAP (GPIO_INPUT_PIN_MAP & GPIO_OUTPUT_PIN_MAP)


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
// GPIO関連処理
//==============================================================================
/** input pin map mode setting */
extern esp_err_t sts_input_pin_map(uint64_t u64_pin_map, bool b_pullup, bool b_pulldown);
/** outpiut pin map mode setting */
extern esp_err_t sts_output_pin_map(uint64_t u64_pin_map, bool b_open_drain, bool b_pullup, bool b_pulldown);
/** io pin map mode setting */
extern esp_err_t sts_io_pin_map(uint64_t u64_pin_map, bool b_open_drain, bool b_pullup, bool b_pulldown);
/** interrupt pin map mode setting */
extern esp_err_t sts_interrupt_pin_map(uint64_t u64_pin_map, gpio_int_type_t  e_int_type);
/** enable the interrupt pin map */
extern esp_err_t sts_interrupt_enable(uint64_t u64_pin_map);
/** disable the interrupt pin map */
extern esp_err_t sts_interrupt_disable(uint64_t u64_pin_map);

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
extern int i_adc_oneshot_raw_data(ts_adc_oneshot_context* ps_ctx, adc_channel_t e_adc_channel);
/** ADCのワンショット読み込み（較正済み電圧） */
extern int i_adc_oneshot_voltage(ts_adc_oneshot_context* ps_ctx, adc_channel_t e_adc_channel);

//==============================================================================
// SPI master 関連処理
//==============================================================================
/** SPI master initialize */
extern esp_err_t sts_spi_mst_bus_init(spi_host_device_t e_host_id,
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
