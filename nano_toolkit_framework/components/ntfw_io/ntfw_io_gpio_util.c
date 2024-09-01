/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :GPIO Utility functions source file
 *
 * CREATED:2023/03/11 05:04:00
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

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_io_gpio_util.h"

#include <esp_adc/adc_cali.h>
#include "ntfw_com_value_util.h"
#include "ntfw_com_mem_alloc.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ログ出力タグ */
//static const char* LOG_TAG = "Debug";

/** SPI CS GPIO NUMBER*/
static const gpio_num_t e_spi_pin_cs[SPI_HOST_MAX] = {
    GPIO_NUM_11, GPIO_NUM_5, GPIO_NUM_15
};


/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: ps_adc_oneshot_ctx
 *
 * DESCRIPTION: ADCユニットコンテキストの生成処理
 *
 * PARAMETERS:                      Name        RW  Usage
 * (adc_unit_t                      e_unit      R   ADCユニット
 * soc_periph_adc_digi_clk_src_t    e_clk_src   R   クロックソース
 * adc_ulp_mode_t                   e_ulp_mode  R   コプロセッサモード
 *
 * RETURNS:
 * ts_adc_oneshot_context*：ADCワンショットコンテキスト
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_adc_oneshot_context* ps_adc_oneshot_ctx(adc_unit_t e_unit,
                                           soc_periph_adc_digi_clk_src_t e_clk_src,
                                           adc_ulp_mode_t e_ulp_mode) {
    // ADCワンショットハンドラ
    ts_adc_oneshot_context* ps_adc_ctx = pv_mem_calloc(sizeof(ts_adc_oneshot_context));
    if (ps_adc_ctx == NULL) {
        return NULL;
    }
    ps_adc_ctx->e_unit = e_unit;                        // ADCユニットID
    ps_adc_ctx->e_cal_mode = NTFW_ADC_CALIBRATION_NONE; // 較正モード
    ps_adc_ctx->ps_calibration_handle = NULL;           // 較正コンテキスト
    // ADC初期設定
    adc_oneshot_unit_init_cfg_t s_adc_init_cfg = {
            .unit_id = e_unit,      // 1.ADCユニット
            .clk_src = e_clk_src,   // 2.クロックソース
            .ulp_mode = e_ulp_mode  // 3.コプロセッサモード
    };
    // ADCワンショット初期処理
    if (adc_oneshot_new_unit(&s_adc_init_cfg, &ps_adc_ctx->s_handle) != ESP_OK) {
        // 初期化失敗
        l_mem_free(ps_adc_ctx);
        return NULL;
    }
    // 結果返信
    return ps_adc_ctx;
}

/*******************************************************************************
 *
 * NAME: ps_adc_oneshot_calibration_ctx
 *
 * DESCRIPTION: ADCユニットコンテキスト（較正）の生成処理
 *
 * PARAMETERS:                      Name        RW  Usage
 * (adc_unit_t                      e_unit      R   ADCユニット
 * soc_periph_adc_digi_clk_src_t    e_clk_src   R   クロックソース
 * adc_ulp_mode_t                   e_ulp_mode  R   コプロセッサモード
 * adc_atten_t                      e_atten     R   電圧レンジ
 *
 * RETURNS:
 * ts_adc_oneshot_context*：ADCワンショットコンテキスト
 *
 * NOTES:
 * 電圧レンジ
 * ADC_ATTEN_DB_0  : 0dB減衰   電圧100mV～950mV
 * ADC_ATTEN_DB_2_5: 2.5dB減衰 電圧100mV～1250mV
 * ADC_ATTEN_DB_6  : 6dB減衰   電圧150mV～1750mV
 * ADC_ATTEN_DB_11 : 11dB減衰  電圧150mV～2450mV
 *
 ******************************************************************************/
ts_adc_oneshot_context* ps_adc_oneshot_calibration_ctx(adc_unit_t e_unit,
                                                       soc_periph_adc_digi_clk_src_t e_clk_src,
                                                       adc_ulp_mode_t e_ulp_mode,
                                                       adc_atten_t e_atten) {
    // ADCユニットコンテキスト生成
    ts_adc_oneshot_context* ps_ctx = ps_adc_oneshot_ctx(e_unit, e_clk_src, e_ulp_mode);
    if (ps_ctx == NULL) {
        return NULL;
    }
    // ADC較正ハンドル生成
    ps_ctx->ps_calibration_handle = pv_mem_calloc(sizeof(adc_cali_handle_t));
    if (ps_ctx->ps_calibration_handle == NULL) {
        sts_adc_oneshot_delete_ctx(ps_ctx);
        return NULL;
    }
    // 較正スキーム開始
    adc_cali_scheme_ver_t e_cheme_mask;
    if (adc_cali_check_scheme(&e_cheme_mask) == ESP_OK) {
        // 結果ステータス
        esp_err_t sts_val = ESP_OK;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (e_cheme_mask == ADC_CALI_SCHEME_VER_CURVE_FITTING) {
            adc_cali_curve_fitting_config_t s_cali_config = {
                .unit_id = e_unit,
                .atten = e_atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            sts_val = adc_cali_create_scheme_curve_fitting(&s_cali_config, ps_ctx->ps_calibration_handle);
        }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if(e_cheme_mask == ADC_CALI_SCHEME_VER_LINE_FITTING) {
            adc_cali_line_fitting_config_t s_cali_config = {
                .unit_id = e_unit,
                .atten = e_atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT
            };
            // 基準電圧の判別
            adc_cali_line_fitting_efuse_val_t e_cali_val;
            adc_cali_scheme_line_fitting_check_efuse(&e_cali_val);
            if (e_cali_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF) {
                s_cali_config.default_vref = NTFW_ADC_DEFAULT_VREF;
            }
            // ADC キャリブレーション ラインのフィッティング スキーム
            sts_val = adc_cali_create_scheme_line_fitting(&s_cali_config, ps_ctx->ps_calibration_handle);
        }
#endif
        // 較正スキーム判定
        if (sts_val != ESP_OK) {
            sts_adc_oneshot_delete_ctx(ps_ctx);
            return NULL;
        }
    }
    // 生成したコンテキストを返却
    return ps_ctx;
}

/*******************************************************************************
 *
 * NAME: sts_adc_oneshot_delete_ctx
 *
 * DESCRIPTION: ADCユニットコンテキストの削除処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_adc_oneshot_context*  ps_ctx      RW  ADCワンショットコンテキスト
 *
 * RETURNS:
 *   ESP_OK: 正常削除
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_adc_oneshot_delete_ctx(ts_adc_oneshot_context* ps_ctx) {
    // 入力チェック
    if (ps_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // 較正の有無を判定
    esp_err_t sts_val = ESP_OK;
    if (ps_ctx->ps_calibration_handle != NULL) {
        // 較正スキームを削除
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        sts_val = adc_cali_delete_scheme_curve_fitting(*ps_ctx->ps_calibration_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        sts_val = adc_cali_delete_scheme_line_fitting(*ps_ctx->ps_calibration_handle);
#endif
        if (sts_val != ESP_OK) {
            return sts_val;
        }
    }
    // ADCワンショットハンドル解放
    sts_val = adc_oneshot_del_unit(ps_ctx->s_handle);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // メモリ解放
    l_mem_free(ps_ctx->ps_calibration_handle);
    l_mem_free(ps_ctx);
    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_adc_oneshot_config_channel
 *
 * DESCRIPTION: ADCチャンネル設定処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_adc_oneshot_context*  ps_ctx          RW  ADCワンショットコンテキスト
 * adc_channel_t            e_adc_channel   R   ADCチャンネル
 * adc_atten_t              e_atten         R   電圧レンジ
 * adc_bitwidth_t           e_bitwidth      R   量子符号ビット数
 *
 * RETURNS:
 *   ESP_OK: 正常削除
 *
 * NOTES:
 * 電圧レンジ
 * ADC_ATTEN_DB_0  : 0dB減衰   電圧100mV～950mV
 * ADC_ATTEN_DB_2_5: 2.5dB減衰 電圧100mV～1250mV
 * ADC_ATTEN_DB_6  : 6dB減衰   電圧150mV～1750mV
 * ADC_ATTEN_DB_11 : 11dB減衰  電圧150mV～2450mV
 * None.
 ******************************************************************************/
esp_err_t sts_adc_oneshot_config_channel(ts_adc_oneshot_context* ps_ctx,
                                         adc_channel_t e_adc_channel,
                                         adc_atten_t e_atten,
                                         adc_bitwidth_t e_bitwidth) {
    // 入力チェック
    if (ps_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // ADCワンショット設定
    adc_oneshot_chan_cfg_t adc_channel_cfg = {
        .bitwidth = e_bitwidth, // 4.量子符号ビット数
        .atten = e_atten,       // 5.減衰率（DB換算）
    };
    // ADCユニットにチャンネル設定
    return adc_oneshot_config_channel(ps_ctx->s_handle, e_adc_channel, &adc_channel_cfg);
}

/*******************************************************************************
 *
 * NAME: i_adc_oneshot_raw_data
 *
 * DESCRIPTION: ADCのワンショット読み込み（RAWデータ）
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_adc_oneshot_context*  ps_ctx          RW  ADCワンショットコンテキスト
 * adc_channel_t            e_adc_channel   R   ADCチャンネル
 *
 * RETURNS:
 *   int: 読み取った電圧(rawデータ)
 *
 * NOTES:
 * None.
 ******************************************************************************/
int i_adc_oneshot_raw_data(ts_adc_oneshot_context* ps_ctx,
                           adc_channel_t e_adc_channel) {
    // 較正されていないADC読み込み
    int i_raw_val;
    if (adc_oneshot_read(ps_ctx->s_handle, e_adc_channel, &i_raw_val) != ESP_OK) {
        return -1;
    }
    // 結果返却
    return i_raw_val;
}

/*******************************************************************************
 *
 * NAME: i_adc_oneshot_voltage
 *
 * DESCRIPTION: ADCのワンショット読み込み
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_adc_oneshot_context*  ps_ctx          RW  ADCワンショットコンテキスト
 * adc_channel_t            e_adc_channel   R   ADCチャンネル
 *
 * RETURNS:
 *   int: 読み取った電圧(mV)
 *
 * NOTES:
 * None.
 ******************************************************************************/
int i_adc_oneshot_voltage(ts_adc_oneshot_context* ps_ctx, adc_channel_t e_adc_channel) {
    // 較正ハンドルの有無を判定
    if (ps_ctx->ps_calibration_handle == NULL) {
        return -1;
    }
    // 較正されたADC読み込み
    int i_result;
    esp_err_t sts_val;
    sts_val = adc_oneshot_get_calibrated_result(ps_ctx->s_handle,
                                                *ps_ctx->ps_calibration_handle,
                                                e_adc_channel,
                                                &i_result);
    if (sts_val != ESP_OK) {
        return -1;
    }
    return i_result;
}

/*******************************************************************************
 *
 * NAME: sts_spi_mst_bus_initialize
 *
 * DESCRIPTION: SPI master initialize
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_adc_oneshot_context*  ps_ctx          RW  ADCワンショットコンテキスト
 * adc_channel_t            e_adc_channel   R   ADCチャンネル
 * bool                     b_pullup        R   プルアップフラグ
 *
 * RETURNS:
 *   int: 読み取った電圧(mV)
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_spi_mst_bus_initialize(spi_host_device_t e_host_id,
                                     const spi_bus_config_t* ps_bus_cfg,
                                     spi_dma_chan_t e_dma_chan,
                                     bool b_pullup) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // HOST ID
    if (e_host_id >= SPI_HOST_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    // MOSI
    if (!b_vutil_valid_pin(ps_bus_cfg->mosi_io_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // MISO
    if (!b_vutil_valid_pin(ps_bus_cfg->miso_io_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    // SCLK
    if (!b_vutil_valid_pin(ps_bus_cfg->sclk_io_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 初期処理
    //==========================================================================
    // GPIO Reset
    gpio_reset_pin(ps_bus_cfg->mosi_io_num);
    gpio_reset_pin(ps_bus_cfg->miso_io_num);
    gpio_reset_pin(ps_bus_cfg->sclk_io_num);
    gpio_reset_pin(e_spi_pin_cs[e_host_id]);
    // WP(Write Protect)
    bool b_quadwp = b_vutil_valid_pin(ps_bus_cfg->quadwp_io_num);
    if (b_quadwp) {
        gpio_reset_pin(ps_bus_cfg->quadwp_io_num);
    }
    // HD(Hold)
    bool b_quadhd = b_vutil_valid_pin(ps_bus_cfg->quadhd_io_num);
    if (b_quadhd) {
        gpio_reset_pin(ps_bus_cfg->quadhd_io_num);
    }

    //==========================================================================
    // プルアップ判定
    //==========================================================================
    if (b_pullup) {
        // GPIO map output
        uint64_t u64_spi_pin_map = 0x00;
        u64_spi_pin_map |= (1ULL << ps_bus_cfg->mosi_io_num);   // MOSI
        u64_spi_pin_map |= (1ULL << ps_bus_cfg->miso_io_num);   // MISO
        u64_spi_pin_map |= (1ULL << ps_bus_cfg->sclk_io_num);   // SCLK
        u64_spi_pin_map |= (1ULL << e_spi_pin_cs[e_host_id]);   // CS
        // WP(Write Protect)
        if (b_quadwp) {
            u64_spi_pin_map |= (1ULL << ps_bus_cfg->quadwp_io_num);
        }
        // HD(Hold)
        if (b_quadhd) {
            u64_spi_pin_map |= (1ULL << ps_bus_cfg->quadhd_io_num);
        }
        // GPIO setting
        gpio_config_t s_gpio_spi_cfg;
        s_gpio_spi_cfg.pin_bit_mask = u64_spi_pin_map;
        s_gpio_spi_cfg.mode         = GPIO_MODE_INPUT_OUTPUT_OD;
        s_gpio_spi_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
        s_gpio_spi_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        s_gpio_spi_cfg.intr_type    = GPIO_INTR_DISABLE;
        // GPIO config
        gpio_config(&s_gpio_spi_cfg);
    }

    //==========================================================================
    // SPI初期化処理
    //==========================================================================
    return spi_bus_initialize(e_host_id, ps_bus_cfg, e_dma_chan);
}


/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
