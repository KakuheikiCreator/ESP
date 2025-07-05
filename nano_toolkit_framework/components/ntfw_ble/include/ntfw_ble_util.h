/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Utilities
 *
 * MODULE :Bluetooth Low Energy Utilities functions header file
 *
 * CREATED:2025/04/28 14:08:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:Bluetooth Low Energyのユーティリティ関数群
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
#ifndef  __NTFW_BLE_UTIL_H__
#define  __NTFW_BLE_UTIL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdbool.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>
#include <esp_gattc_api.h>
#include <ntfw_com_data_model.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** BLE Framework Debug */
//#define BLE_UTIL_DEBUG

/** TX Power DBM min and max */
#if (CONFIG_IDF_TARGET_ESP32)
    #define NTFW_BLE_PWR_LVL_MIN    (ESP_PWR_LVL_N12)
    #define NTFW_BLE_PWR_LVL_MAX    (ESP_PWR_LVL_P9)
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3)
    #define NTFW_BLE_PWR_LVL_MIN    (ESP_PWR_LVL_N24)
    #define NTFW_BLE_PWR_LVL_MAX    (ESP_PWR_LVL_P21)
#elif (CONFIG_IDF_TARGET_ESP32C6)
    #define NTFW_BLE_PWR_LVL_MIN    (ESP_PWR_LVL_N15)
    #define NTFW_BLE_PWR_LVL_MAX    (ESP_PWR_LVL_P20)
#elif (CONFIG_IDF_TARGET_ESP32H2)
    #define NTFW_BLE_PWR_LVL_MIN    (ESP_PWR_LVL_N24)
    #define NTFW_BLE_PWR_LVL_MAX    (ESP_PWR_LVL_P20)
#else
    #define NTFW_BLE_PWR_LVL_MIN    (ESP_PWR_LVL_N12)
    #define NTFW_BLE_PWR_LVL_MAX    (ESP_PWR_LVL_P9)
#endif


//==============================================================================
// BLE共通
//==============================================================================
// アドレスクリア
#define v_ble_util_addr_clear(t_bda) v_ble_util_addr_cpy(t_bda, t_ble_util_bda_none)
// アドレスクリア判定
#define b_ble_util_addr_clear(t_bda) (memcmp(t_bda, t_ble_util_bda_none, ESP_BD_ADDR_LEN) == 0)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
//==============================================================================
// BLE Common Type Define
//==============================================================================
/** Bluetooth LE アドレス文字列型 */
typedef char tc_ble_util_bda_string_t[18];

/** 構造体：ボンディング済みデバイス情報 */
typedef struct {
    int i_device_cnt;                       // デバイス数
    esp_ble_bond_dev_t* ps_dev_list;        // デバイスリスト
} ts_ble_util_bond_dev_list_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** BLE Address None */
extern const esp_bd_addr_t t_ble_util_bda_none;

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/
//==============================================================================
// 文字列化処理
//==============================================================================
/** BLEアドレス文字列の取得 */
extern void v_ble_util_address_to_str(tc_ble_util_bda_string_t tc_bda, esp_bd_addr_t t_bda);
/** BLEキータイプ文字列の取得 */
extern const char* pc_ble_util_key_type_to_str(esp_ble_key_type_t t_key_type);
/** 認証リクエストタイプ文字列取得 */
extern const char* pc_ble_util_auth_req_to_str(esp_ble_auth_req_t auth_req);
/** BLEのGAPイベント文字列の取得 */
extern const char* pc_ble_util_gap_event_to_str(esp_gap_ble_cb_event_t e_event);
/** BLEのGATTサーバーイベント文字列の取得 */
extern const char* pc_ble_util_gatts_event_to_str(esp_gatts_cb_event_t e_event);
/** BLEのGATTクライアントイベント文字列の取得 */
extern const char* pc_ble_util_gattc_event_to_str(esp_gattc_cb_event_t e_event);

//==============================================================================
// 情報関数
//==============================================================================
/** BLEのボンディングデバイスの表示 */
extern esp_err_t sts_ble_util_display_bonded_devices();
/** BLEのボンディング済みデバイスリスト取得処理 */
extern ts_ble_util_bond_dev_list_t* ps_ble_util_bond_dev_list();
/** BLEのボンディング済みデバイスリストの削除処理 */
extern void v_ble_util_delete_bond_dev_list(ts_ble_util_bond_dev_list_t* ps_dev_list);

//==============================================================================
// ユーティリティ
//==============================================================================
/** BLEのアドレスコピー処理 */
extern void v_ble_util_addr_cpy(esp_bd_addr_t t_to_bda, const esp_bd_addr_t t_from_bda);
/** BLEのアドレス比較処理 */
extern long l_ble_util_addr_cmp(const esp_bd_addr_t t_bda1, const esp_bd_addr_t t_bda2);
/** BLEのID比較処理 */
extern bool b_ble_util_id_equal(esp_gatt_id_t* ps_id1, esp_gatt_id_t* ps_id2);
/** BLEのUUID比較処理 */
extern bool b_ble_util_uuid_equal(esp_bt_uuid_t* ps_uuid1, esp_bt_uuid_t* ps_uuid2);
/** BLEのBASE_UUID編集処理 */
extern bool b_ble_util_edit_base_uuid(uint8_t* pu8_uuid);
/** 電波出力値の取得処理 */
extern int8_t i8_ble_util_power_dbm(esp_power_level_t e_power_level);
/** BLEのアドバタイジングデータ（RAW形式）の生成処理 */
extern ts_u8_array_t* ps_ble_util_create_adv_data_raw(esp_ble_adv_data_t* ps_adv_data, char* pc_dev_name, esp_power_level_t e_tx_power);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_BLE_UTIL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
