/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :Common Touchpad framework functions header file
 *
 * CREATED:2020/03/22 18:00:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:タッチパッドの簡易フレームワーク関数群
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
#include <freertos/FreeRTOS.h>
#include <driver/touch_pad.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Touchpad number of samples */
#ifndef IO_TOUCHPAD_NUMBER_OF_SAMPLES
    #define IO_TOUCHPAD_NUMBER_OF_SAMPLES   (128)
#endif

/** Touchpad default threshold */
#ifndef IO_TOUCHPAD_DEFAULT_THRESHOLD
    #define IO_TOUCHPAD_DEFAULT_THRESHOLD   (700)
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
/** タッチピンの平均値取得処理 */
extern uint16_t u16_io_touchpad_pin_average(touch_pad_t e_touch_num);
/** タッチピンの閾値設定処理 */
extern esp_err_t sts_io_touchpad_pin_threshold(touch_pad_t e_touch_num, uint16_t u16_threshold);
/** タッチパッドのステータスチェックの有効化 */
extern esp_err_t sts_io_touchpad_check_enable();
/** タッチパッドのステータスチェックの無効化 */
extern esp_err_t sts_io_touchpad_check_disable();
/** タッチパッド割り込みのステータス取得処理 */
extern uint32_t u32_io_touchpad_pinmap(TickType_t t_tick);
/** タッチパッド割り込みのステータスクリア処理 */
extern void v_io_touchpad_clear_pinmap();

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_TOUCHPAD_UTIL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
