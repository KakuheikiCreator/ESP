/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :Common Touchpad framework functions header file
 *
 * CREATED:2020/03/22 18:00:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:タッチパッドの簡易フレームワーク関数群
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
#ifndef  __NTFW_TOUCHPAD_UTIL_H__
#define  __NTFW_TOUCHPAD_UTIL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <esp_system.h>
#include <esp_err.h>
#include <sdkconfig.h>
#include <hal/touch_sensor_types.h>
#include <freertos/FreeRTOS.h>
#include <driver/touch_pad.h>

/******************************************************************************/
/***      HARDWARE: ESP32 ESP32S3                                           ***/
/******************************************************************************/
#if (CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S3)

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/** Touchpad number of samples */
#ifndef IO_TOUCHPAD_NUMBER_OF_SAMPLES
    #define IO_TOUCHPAD_NUMBER_OF_SAMPLES   (128)
#endif

/** Touchpad default threshold */
#ifndef IO_TOUCHPAD_DEFAULT_THRESHOLD
#if defined(CONFIG_IDF_TARGET_ESP32)
    #define IO_TOUCHPAD_DEFAULT_THRESHOLD   (700)
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    #define IO_TOUCHPAD_DEFAULT_THRESHOLD   (700)
#endif
#endif

/** Touchpad filter period */
#ifndef IO_TOUCHPAD_FILTER_PERIOD
    #define IO_TOUCHPAD_FILTER_PERIOD   (10)
#endif

/** Touchpad status queue size */
#ifndef IO_TOUCHPAD_STS_QUEUE_SIZE
    #define IO_TOUCHPAD_STS_QUEUE_SIZE  (16)
#endif

/** Demon process cycle */
#ifndef NTFW_TOUCHPAD_DEAMON_WAIT
    #define IO_TOUCHPAD_DEAMON_WAIT     (50 / portTICK_PERIOD_MS)
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/
//==============================================================================
// タッチパッド関連関数
//==============================================================================
/** タッチパッドの初期処理 */
extern esp_err_t sts_io_touchpad_init();
/** タッチピンの有効化 */
extern esp_err_t sts_io_touchpad_pin_enable(touch_pad_t e_touch_num);
/** タッチピンの無効化 */
extern esp_err_t sts_io_touchpad_pin_disable(touch_pad_t e_touch_num);

//------------------------------------------------------------------------------
// タッチピンの平均値取得処理
//------------------------------------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32)
/** ESP32 */
extern uint16_t u16_io_touchpad_pin_average(touch_pad_t e_touch_num);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
/** ESP32-S3 */
extern uint32_t u32_io_touchpad_pin_average(touch_pad_t e_touch_num);
#endif
/** タッチピンの閾値設定処理 */
#if defined(CONFIG_IDF_TARGET_ESP32)
/** ESP32 */
extern esp_err_t sts_io_touchpad_pin_threshold(touch_pad_t e_touch_num, uint16_t u16_threshold);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
/** ESP32-S3 */
extern esp_err_t sts_io_touchpad_pin_threshold(touch_pad_t e_touch_num, uint32_t u32_threshold);
#endif
/** タッチパッドのステータスチェックの有効化 */
extern esp_err_t sts_io_touchpad_check_enable();
/** タッチパッドのステータスチェックの無効化 */
extern esp_err_t sts_io_touchpad_check_disable();
/** タッチパッド割り込みのステータス取得処理 */
extern uint32_t u32_io_touchpad_pinmap(TickType_t t_tick);
/** タッチパッド割り込みのステータスクリア処理 */
extern void v_io_touchpad_clear_pinmap();

/** END:defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3) */
#endif

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_TOUCHPAD_UTIL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
