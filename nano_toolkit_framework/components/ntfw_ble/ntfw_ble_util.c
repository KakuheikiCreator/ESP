/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Utilities
 *
 * MODULE :Bluetooth Low Energy Utilities functions source file
 *
 * CREATED:2025/04/28 14:08:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:Bluetooth Low Energyのフレームワークとユーティリティ関数群
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
/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <ntfw_ble_util.h>

#include <esp_err.h>
#include <string.h>
#include <esp_log.h>
#include <ntfw_com_mem_alloc.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/** ログ接頭辞 */
#define LOG_TAG "BLE_UTIL"

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** BLE Address None */
const esp_bd_addr_t t_ble_util_bda_none = {0x40};

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

//==============================================================================
// 共通系の定数定義
//==============================================================================
/** BASE UUID */
static const uint8_t u8_base_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#if (CONFIG_IDF_TARGET_ESP32)
/** TX Power DBM List */
static int8_t i8_power_dbm_list[] = {-12, -9, -6, -3, 0, 3, 6, 9, -12, -9, -6, -3, 0, 3, 6, 9};
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3)
/** TX Power DBM List */
static int8_t i8_power_dbm_list[] = {-24, -21, -18, -15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 20, 21};
#elif (CONFIG_IDF_TARGET_ESP32C6)
/** TX Power DBM List */
static int8_t i8_power_dbm_list[] = {-15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 20};
#elif (CONFIG_IDF_TARGET_ESP32H2)
/** TX Power DBM List */
static int8_t i8_power_dbm_list[] = {-24, -21, -18, -15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 20};
#endif

//==============================================================================
// GAP関係の定数定義
//==============================================================================
/** GAPイベント名 */
static const char* pc_ble_gap_evt_str[] = {
    [ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT] = "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT] = "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT] = "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_RESULT_EVT] = "ESP_GAP_BLE_SCAN_RESULT_EVT",
    [ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT] = "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT] = "ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_ADV_START_COMPLETE_EVT] = "ESP_GAP_BLE_ADV_START_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_START_COMPLETE_EVT] = "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT",
    [ESP_GAP_BLE_AUTH_CMPL_EVT] = "ESP_GAP_BLE_AUTH_CMPL_EVT",
    [ESP_GAP_BLE_KEY_EVT] = "ESP_GAP_BLE_KEY_EVT",
    [ESP_GAP_BLE_SEC_REQ_EVT] = "ESP_GAP_BLE_SEC_REQ_EVT",
    [ESP_GAP_BLE_PASSKEY_NOTIF_EVT] = "ESP_GAP_BLE_PASSKEY_NOTIF_EVT",
    [ESP_GAP_BLE_PASSKEY_REQ_EVT] = "ESP_GAP_BLE_PASSKEY_REQ_EVT",
    [ESP_GAP_BLE_OOB_REQ_EVT] = "ESP_GAP_BLE_OOB_REQ_EVT",
    [ESP_GAP_BLE_LOCAL_IR_EVT] = "ESP_GAP_BLE_LOCAL_IR_EVT",
    [ESP_GAP_BLE_LOCAL_ER_EVT] = "ESP_GAP_BLE_LOCAL_ER_EVT",
    [ESP_GAP_BLE_NC_REQ_EVT] = "ESP_GAP_BLE_NC_REQ_EVT",
    [ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT] = "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT",
    [ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT] = "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT] = "ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT",
    [ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT] = "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT",
    [ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT] = "ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT] = "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT",
    [ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT] = "ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT",
    [ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT] = "ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT",
    [ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT] = "ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_CHANNELS_EVT] = "ESP_GAP_BLE_SET_CHANNELS_EVT",
    [ESP_GAP_BLE_READ_PHY_COMPLETE_EVT] = "ESP_GAP_BLE_READ_PHY_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_PREFERRED_DEFAULT_PHY_COMPLETE_EVT] = "ESP_GAP_BLE_SET_PREFERRED_DEFAULT_PHY_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT] = "ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_SET_REMOVE_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_SET_REMOVE_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_SET_CLEAR_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_ADV_SET_CLEAR_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_STOP_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_STOP_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_CREATE_SYNC_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_CREATE_SYNC_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_CANCEL_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_CANCEL_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_TERMINATE_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_TERMINATE_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_ADD_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_ADD_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_REMOVE_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_REMOVE_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_CLEAR_DEV_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_CLEAR_DEV_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT] = "ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT] = "ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT",
    [ESP_GAP_BLE_PREFER_EXT_CONN_PARAMS_SET_COMPLETE_EVT] = "ESP_GAP_BLE_PREFER_EXT_CONN_PARAMS_SET_COMPLETE_EVT",
    [ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT] = "ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT",
    [ESP_GAP_BLE_EXT_ADV_REPORT_EVT] = "ESP_GAP_BLE_EXT_ADV_REPORT_EVT",
    [ESP_GAP_BLE_SCAN_TIMEOUT_EVT] = "ESP_GAP_BLE_SCAN_TIMEOUT_EVT",
    [ESP_GAP_BLE_ADV_TERMINATED_EVT] = "ESP_GAP_BLE_ADV_TERMINATED_EVT",
    [ESP_GAP_BLE_SCAN_REQ_RECEIVED_EVT] = "ESP_GAP_BLE_SCAN_REQ_RECEIVED_EVT",
    [ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT] = "ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT",
    [ESP_GAP_BLE_SC_OOB_REQ_EVT] = "ESP_GAP_BLE_SC_OOB_REQ_EVT",
    [ESP_GAP_BLE_SC_CR_LOC_OOB_EVT] = "ESP_GAP_BLE_SC_CR_LOC_OOB_EVT",
    [ESP_GAP_BLE_GET_DEV_NAME_COMPLETE_EVT] = "ESP_GAP_BLE_GET_DEV_NAME_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_RECV_ENABLE_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_RECV_ENABLE_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_TRANS_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_TRANS_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SET_INFO_TRANS_COMPLETE_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SET_INFO_TRANS_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_PAST_PARAMS_COMPLETE_EVT] = "ESP_GAP_BLE_SET_PAST_PARAMS_COMPLETE_EVT",
    [ESP_GAP_BLE_PERIODIC_ADV_SYNC_TRANS_RECV_EVT] = "ESP_GAP_BLE_PERIODIC_ADV_SYNC_TRANS_RECV_EVT",
    [ESP_GAP_BLE_DTM_TEST_UPDATE_EVT] = "ESP_GAP_BLE_DTM_TEST_UPDATE_EVT",
    [ESP_GAP_BLE_ADV_CLEAR_COMPLETE_EVT] = "ESP_GAP_BLE_ADV_CLEAR_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_RPA_TIMEOUT_COMPLETE_EVT] = "ESP_GAP_BLE_SET_RPA_TIMEOUT_COMPLETE_EVT",
    [ESP_GAP_BLE_ADD_DEV_TO_RESOLVING_LIST_COMPLETE_EVT] = "ESP_GAP_BLE_ADD_DEV_TO_RESOLVING_LIST_COMPLETE_EVT",
    [ESP_GAP_BLE_VENDOR_CMD_COMPLETE_EVT] = "ESP_GAP_BLE_VENDOR_CMD_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_PRIVACY_MODE_COMPLETE_EVT] = "ESP_GAP_BLE_SET_PRIVACY_MODE_COMPLETE_EVT",
    [ESP_GAP_BLE_SET_CSA_SUPPORT_COMPLETE_EVT] = "ESP_GAP_BLE_SET_CSA_SUPPORT_COMPLETE_EVT",
    [ESP_GAP_BLE_EVT_MAX] = "ESP_GAP_BLE_EVT_MAX",
};

//==============================================================================
// GATTサーバー関係の定数定義
//==============================================================================
/** GATTサーバーイベント名 */
static const char* pc_ble_gatts_evt_str[] = {
    [ESP_GATTS_REG_EVT] = "ESP_GATTS_REG_EVT",
    [ESP_GATTS_READ_EVT] = "ESP_GATTS_READ_EVT",
    [ESP_GATTS_WRITE_EVT] = "ESP_GATTS_WRITE_EVT",
    [ESP_GATTS_EXEC_WRITE_EVT] = "ESP_GATTS_EXEC_WRITE_EVT",
    [ESP_GATTS_MTU_EVT] = "ESP_GATTS_MTU_EVT",
    [ESP_GATTS_CONF_EVT] = "ESP_GATTS_CONF_EVT",
    [ESP_GATTS_UNREG_EVT] = "ESP_GATTS_UNREG_EVT",
    [ESP_GATTS_CREATE_EVT] = "ESP_GATTS_CREATE_EVT",
    [ESP_GATTS_ADD_INCL_SRVC_EVT] = "ESP_GATTS_ADD_INCL_SRVC_EVT",
    [ESP_GATTS_ADD_CHAR_EVT] = "ESP_GATTS_ADD_CHAR_EVT",
    [ESP_GATTS_ADD_CHAR_DESCR_EVT] = "ESP_GATTS_ADD_CHAR_DESCR_EVT",
    [ESP_GATTS_DELETE_EVT] = "ESP_GATTS_DELETE_EVT",
    [ESP_GATTS_START_EVT] = "ESP_GATTS_START_EVT",
    [ESP_GATTS_STOP_EVT] = "ESP_GATTS_STOP_EVT",
    [ESP_GATTS_CONNECT_EVT] = "ESP_GATTS_CONNECT_EVT",
    [ESP_GATTS_DISCONNECT_EVT] = "ESP_GATTS_DISCONNECT_EVT",
    [ESP_GATTS_OPEN_EVT] = "ESP_GATTS_OPEN_EVT",
    [ESP_GATTS_CANCEL_OPEN_EVT] = "ESP_GATTS_CANCEL_OPEN_EVT",
    [ESP_GATTS_CLOSE_EVT] = "ESP_GATTS_CLOSE_EVT",
    [ESP_GATTS_LISTEN_EVT] = "ESP_GATTS_LISTEN_EVT",
    [ESP_GATTS_CONGEST_EVT] = "ESP_GATTS_CONGEST_EVT",
    [ESP_GATTS_RESPONSE_EVT] = "ESP_GATTS_RESPONSE_EVT",
    [ESP_GATTS_CREAT_ATTR_TAB_EVT] = "ESP_GATTS_CREAT_ATTR_TAB_EVT",
    [ESP_GATTS_SET_ATTR_VAL_EVT] = "ESP_GATTS_SET_ATTR_VAL_EVT",
    [ESP_GATTS_SEND_SERVICE_CHANGE_EVT] = "ESP_GATTS_SEND_SERVICE_CHANGE_EVT",
};

//==============================================================================
// GATTクライアント関係の定数定義
//==============================================================================
/** GATTクライアントイベント名 */
static const char* pc_ble_gattc_evt_str[] = {
    [ESP_GATTC_REG_EVT] = "ESP_GATTC_REG_EVT",
    [ESP_GATTC_UNREG_EVT] = "ESP_GATTC_UNREG_EVT",
    [ESP_GATTC_OPEN_EVT] = "ESP_GATTC_OPEN_EVT",
    [ESP_GATTC_READ_CHAR_EVT] = "ESP_GATTC_READ_CHAR_EVT",
    [ESP_GATTC_WRITE_CHAR_EVT] = "ESP_GATTC_WRITE_CHAR_EVT",
    [ESP_GATTC_CLOSE_EVT] = "ESP_GATTC_CLOSE_EVT",
    [ESP_GATTC_SEARCH_CMPL_EVT] = "ESP_GATTC_SEARCH_CMPL_EVT",
    [ESP_GATTC_SEARCH_RES_EVT] = "ESP_GATTC_SEARCH_RES_EVT",
    [ESP_GATTC_READ_DESCR_EVT] = "ESP_GATTC_READ_DESCR_EVT",
    [ESP_GATTC_WRITE_DESCR_EVT] = "ESP_GATTC_WRITE_DESCR_EVT",
    [ESP_GATTC_NOTIFY_EVT] = "ESP_GATTC_NOTIFY_EVT",
    [ESP_GATTC_PREP_WRITE_EVT] = "ESP_GATTC_PREP_WRITE_EVT",
    [ESP_GATTC_EXEC_EVT] = "ESP_GATTC_EXEC_EVT",
    [ESP_GATTC_ACL_EVT] = "ESP_GATTC_ACL_EVT",
    [ESP_GATTC_CANCEL_OPEN_EVT] = "ESP_GATTC_CANCEL_OPEN_EVT",
    [ESP_GATTC_SRVC_CHG_EVT] = "ESP_GATTC_SRVC_CHG_EVT",
    [ESP_GATTC_ENC_CMPL_CB_EVT] = "ESP_GATTC_ENC_CMPL_CB_EVT",
    [ESP_GATTC_CFG_MTU_EVT] = "ESP_GATTC_CFG_MTU_EVT",
    [ESP_GATTC_ADV_DATA_EVT] = "ESP_GATTC_ADV_DATA_EVT",
    [ESP_GATTC_MULT_ADV_ENB_EVT] = "ESP_GATTC_MULT_ADV_ENB_EVT",
    [ESP_GATTC_MULT_ADV_UPD_EVT] = "ESP_GATTC_MULT_ADV_UPD_EVT",
    [ESP_GATTC_MULT_ADV_DATA_EVT] = "ESP_GATTC_MULT_ADV_DATA_EVT",
    [ESP_GATTC_MULT_ADV_DIS_EVT] = "ESP_GATTC_MULT_ADV_DIS_EVT",
    [ESP_GATTC_CONGEST_EVT] = "ESP_GATTC_CONGEST_EVT",
    [ESP_GATTC_BTH_SCAN_ENB_EVT] = "ESP_GATTC_BTH_SCAN_ENB_EVT",
    [ESP_GATTC_BTH_SCAN_CFG_EVT] = "ESP_GATTC_BTH_SCAN_CFG_EVT",
    [ESP_GATTC_BTH_SCAN_RD_EVT] = "ESP_GATTC_BTH_SCAN_RD_EVT",
    [ESP_GATTC_BTH_SCAN_THR_EVT] = "ESP_GATTC_BTH_SCAN_THR_EVT",
    [ESP_GATTC_BTH_SCAN_PARAM_EVT] = "ESP_GATTC_BTH_SCAN_PARAM_EVT",
    [ESP_GATTC_BTH_SCAN_DIS_EVT] = "ESP_GATTC_BTH_SCAN_DIS_EVT",
    [ESP_GATTC_SCAN_FLT_CFG_EVT] = "ESP_GATTC_SCAN_FLT_CFG_EVT",
    [ESP_GATTC_SCAN_FLT_PARAM_EVT] = "ESP_GATTC_SCAN_FLT_PARAM_EVT",
    [ESP_GATTC_SCAN_FLT_STATUS_EVT] = "ESP_GATTC_SCAN_FLT_STATUS_EVT",
    [ESP_GATTC_ADV_VSC_EVT] = "ESP_GATTC_ADV_VSC_EVT",
    [ESP_GATTC_REG_FOR_NOTIFY_EVT] = "ESP_GATTC_REG_FOR_NOTIFY_EVT",
    [ESP_GATTC_UNREG_FOR_NOTIFY_EVT] = "ESP_GATTC_UNREG_FOR_NOTIFY_EVT",
    [ESP_GATTC_CONNECT_EVT] = "ESP_GATTC_CONNECT_EVT",
    [ESP_GATTC_DISCONNECT_EVT] = "ESP_GATTC_DISCONNECT_EVT",
    [ESP_GATTC_READ_MULTIPLE_EVT] = "ESP_GATTC_READ_MULTIPLE_EVT",
    [ESP_GATTC_QUEUE_FULL_EVT] = "ESP_GATTC_QUEUE_FULL_EVT",
    [ESP_GATTC_SET_ASSOC_EVT] = "ESP_GATTC_SET_ASSOC_EVT",
    [ESP_GATTC_GET_ADDR_LIST_EVT] = "ESP_GATTC_GET_ADDR_LIST_EVT",
    [ESP_GATTC_DIS_SRVC_CMPL_EVT] = "ESP_GATTC_DIS_SRVC_CMPL_EVT",
    [ESP_GATTC_READ_MULTI_VAR_EVT] = "ESP_GATTC_READ_MULTI_VAR_EVT",
};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_ble_util_address_to_str
 *
 * DESCRIPTION:BLEアドレス文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 * c_ble_fwk_bda_string_t   tc_addr     W   BLEアドレス文字列
 * esp_bd_addr_t            t_bda       R   BLEアドレス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_ble_util_address_to_str(tc_ble_util_bda_string_t tc_addr, esp_bd_addr_t t_bda) {
    if (tc_addr == NULL || t_bda == NULL) {
        return;
    }
    sprintf(tc_addr, "%02X:%02X:%02X:%02X:%02X:%02X", t_bda[0], t_bda[1], t_bda[2], t_bda[3], t_bda[4], t_bda[5]);
}

/*******************************************************************************
 *
 * NAME: pc_ble_fwk_key_type_to_str
 *
 * DESCRIPTION:BLEキータイプ文字列の取得
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_ble_key_type_t   t_key_type  R   キータイプ
 *
 * RETURNS:
 *   const char*:キータイプ文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_ble_util_key_type_to_str(esp_ble_key_type_t t_key_type) {
    switch(t_key_type) {
    case ESP_LE_KEY_NONE:
        return "ESP_LE_KEY_NONE";
    case ESP_LE_KEY_PENC:
        return "ESP_LE_KEY_PENC";
    case ESP_LE_KEY_PID:
        return "ESP_LE_KEY_PID";
    case ESP_LE_KEY_PCSRK:
        return "ESP_LE_KEY_PCSRK";
    case ESP_LE_KEY_PLK:
        return "ESP_LE_KEY_PLK";
    case ESP_LE_KEY_LLK:
        return "ESP_LE_KEY_LLK";
    case ESP_LE_KEY_LENC:
        return "ESP_LE_KEY_LENC";
    case ESP_LE_KEY_LID:
        return "ESP_LE_KEY_LID";
    case ESP_LE_KEY_LCSRK:
        return "ESP_LE_KEY_LCSRK";
    default:
        break;
    }
    // 対応するキーが無い場合
    return "INVALID BLE KEY TYPE";
}

/*******************************************************************************
 *
 * NAME: pc_ble_util_auth_req_to_str
 *
 * DESCRIPTION:認証リクエストタイプ文字列取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_ble_auth_req_t     t_auth_req  R   認証リクエストタイプ
 *
 * RETURNS:
 *   const char*:認証リクエストタイプ文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_ble_util_auth_req_to_str(esp_ble_auth_req_t t_auth_req) {
    // 認証リクエストタイプの判定処理
    switch(t_auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        return "ESP_LE_AUTH_NO_BOND";
    case ESP_LE_AUTH_BOND:
        return "ESP_LE_AUTH_BOND";
    case ESP_LE_AUTH_REQ_MITM:
        return "ESP_LE_AUTH_REQ_MITM";
    case ESP_LE_AUTH_REQ_SC_ONLY:
        return "ESP_LE_AUTH_REQ_SC_ONLY";
    case ESP_LE_AUTH_REQ_SC_BOND:
        return "ESP_LE_AUTH_REQ_SC_BOND";
    case ESP_LE_AUTH_REQ_SC_MITM:
        return "ESP_LE_AUTH_REQ_SC_MITM";
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        return "ESP_LE_AUTH_REQ_SC_MITM_BOND";
    default:
        break;
    }
    return "INVALID BLE AUTH REQ";
}

/*******************************************************************************
 *
 * NAME: pc_ble_util_gatts_event_to_str
 *
 * DESCRIPTION:BLEのGAPイベント文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_gap_ble_cb_event_t e_event     R   GAPイベント
 *
 * RETURNS:
 *   const char*:イベント文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_ble_util_gap_event_to_str(esp_gap_ble_cb_event_t e_event) {
    // 入力チェック
    if (e_event < 0 || e_event > ESP_GAP_BLE_EVT_MAX) {
        return "ESP_GAP_EVT_ERR";
    }
    return pc_ble_gap_evt_str[e_event];
}

/*******************************************************************************
 *
 * NAME: pc_ble_util_gatts_event_to_str
 *
 * DESCRIPTION:BLEのGATTサーバーイベント文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_gatts_cb_event_t   e_event     R   GATTSイベント
 *
 * RETURNS:
 *   const char*:イベント文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_ble_util_gatts_event_to_str(esp_gatts_cb_event_t e_event) {
    // 入力チェック
    if (e_event < 0 || e_event > ESP_GATTS_SEND_SERVICE_CHANGE_EVT) {
        return "ESP_GATTS_EVT_ERR";
    }
    return pc_ble_gatts_evt_str[e_event];
}

/*******************************************************************************
 *
 * NAME: pc_ble_util_gattc_event_to_str
 *
 * DESCRIPTION:BLEのGATTクライアントイベント文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_gattc_cb_event_t   e_event     R   GATTCイベント
 *
 * RETURNS:
 *   const char*:イベント文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_ble_util_gattc_event_to_str(esp_gattc_cb_event_t e_event) {
    // 入力チェック
    if (e_event < 0 || e_event > ESP_GATTC_READ_MULTI_VAR_EVT) {
        return "ESP_GATTC_EVT_ERR";
    }
    return pc_ble_gattc_evt_str[e_event];
}

/*******************************************************************************
 *
 * NAME: sts_ble_util_display_bonded_devices
 *
 * DESCRIPTION:BLEボンディングデバイス表示
 *
 * PARAMETERS:      Name                RW  Usage
 *
 * RETURNS:
 *   表示成功:ESP_OK
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_ble_util_display_bonded_devices() {
    // ボンディング成功デバイス数
    int i_dev_num = esp_ble_get_bond_device_num();
    if (i_dev_num == 0) {
        ESP_LOGI(LOG_TAG, "No bonded devices");
        return ESP_OK;
    }
    // メモリ確保
    esp_ble_bond_dev_t dev_list[i_dev_num];
    // ボンディング済みデバイスリスト取得
    esp_err_t sts_val = esp_ble_get_bond_device_list(&i_dev_num, dev_list);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // ボンディング済みデバイスの表示
    uint8_t* pu8_addr;
    int i_idx;
    for (i_idx = 0; i_idx < i_dev_num; i_idx++) {
        pu8_addr = (uint8_t*)dev_list[i_idx].bd_addr;
        ESP_LOGI(LOG_TAG, "Bond Device Address  = %02x:%02x:%02x:%02x:%02x:%02x",
                pu8_addr[0], pu8_addr[1], pu8_addr[2], pu8_addr[3], pu8_addr[4], pu8_addr[5]);
        ESP_LOGI(LOG_TAG, "Bond Device Key Mask = %02x", dev_list[i_idx].bond_key.key_mask);
    }
    // 表示成功
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_ble_util_bond_dev_list
 *
 * DESCRIPTION:BLEのボンディング済みデバイスリスト取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   ts_ble_fwk_bond_dev_list_t:ボンディングリスト
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_ble_util_bond_dev_list_t* ps_ble_util_bond_dev_list() {
    // ボンディング済みデバイス数
    int i_bond_cnt = esp_ble_get_bond_device_num();
    if (i_bond_cnt <= 0) {
        return NULL;
    }
    // ボンディングデバイスリストを動的に確保
    ts_ble_util_bond_dev_list_t* ps_dev_list = pv_mem_malloc(sizeof(ts_ble_util_bond_dev_list_t));
    if (ps_dev_list == NULL) {
        return NULL;
    }
    ps_dev_list->ps_dev_list = pv_mem_malloc(sizeof(esp_ble_bond_dev_t) * i_bond_cnt);
    if (ps_dev_list->ps_dev_list == NULL) {
        l_mem_free(ps_dev_list);
        return NULL;
    }
    // ボンディング済みデバイスを編集
    ps_dev_list->i_device_cnt = i_bond_cnt;
    // ボンディングデバイスリストを取得
    esp_err_t sts_val = esp_ble_get_bond_device_list(&i_bond_cnt, ps_dev_list->ps_dev_list);
    if (sts_val != ESP_OK) {
        // 確保したメモリを解放
        v_ble_util_delete_bond_dev_list(ps_dev_list);
    }
    // 結果返却
    return ps_dev_list;
}

/*******************************************************************************
 *
 * NAME: v_ble_util_delete_bond_dev_list
 *
 * DESCRIPTION:BLEのボンディング済みデバイスリストの削除処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_ble_fwk_bond_dev_list_t*  ps_dev_list W   メモリ解放して削除する対象
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_ble_util_delete_bond_dev_list(ts_ble_util_bond_dev_list_t* ps_dev_list) {
    if (ps_dev_list == NULL) {
        return;
    }
    // メモリを解放
    l_mem_free(ps_dev_list->ps_dev_list);
    l_mem_free(ps_dev_list);
}

/*******************************************************************************
 *
 * NAME: v_ble_util_addr_cpy
 *
 * DESCRIPTION:BLEのアドレスコピー処理
 *
 * PARAMETERS:      Name         RW  Usage
 * esp_bd_addr_t    t_to_bda     W   編集先
 * esp_bd_addr_t    t_from_bda   R   編集元
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_ble_util_addr_cpy(esp_bd_addr_t t_to_bda, const esp_bd_addr_t t_from_bda) {
    t_to_bda[0] = t_from_bda[0];
    t_to_bda[1] = t_from_bda[1];
    t_to_bda[2] = t_from_bda[2];
    t_to_bda[3] = t_from_bda[3];
    t_to_bda[4] = t_from_bda[4];
    t_to_bda[5] = t_from_bda[5];
}

/*******************************************************************************
 *
 * NAME: l_ble_util_addr_cmp
 *
 * DESCRIPTION:BLEのアドレス比較処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda1      R   比較値１
 * esp_bd_addr_t    t_bda2      R   比較値２
 *
 * RETURNS:
 * 比較結果:pu8_addr1 - pu8_addr2
 *
 * NOTES:
 * None.
 ******************************************************************************/
long l_ble_util_addr_cmp(const esp_bd_addr_t t_bda1, const esp_bd_addr_t t_bda2) {
    // 値１編集
    long l_val1 = t_bda1[0];
    l_val1 = (l_val1 << 8) + t_bda1[1];
    l_val1 = (l_val1 << 8) + t_bda1[2];
    l_val1 = (l_val1 << 8) + t_bda1[3];
    l_val1 = (l_val1 << 8) + t_bda1[4];
    l_val1 = (l_val1 << 8) + t_bda1[5];
    // 値２編集
    long l_val2 = t_bda2[0];
    l_val2 = (l_val2 << 8) + t_bda2[1];
    l_val2 = (l_val2 << 8) + t_bda2[2];
    l_val2 = (l_val2 << 8) + t_bda2[3];
    l_val2 = (l_val2 << 8) + t_bda2[4];
    l_val2 = (l_val2 << 8) + t_bda2[5];
    // 比較結果
    return l_val1 - l_val2;
}

/*******************************************************************************
 *
 * NAME: b_ble_util_id_equal
 *
 * DESCRIPTION:BLEのID比較処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_id_t*   pu8_addr1   R   比較値１
 * esp_gatt_id_t*   pu8_addr2   R   比較値２
 *
 * RETURNS:
 * 比較結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_ble_util_id_equal(esp_gatt_id_t* ps_id1, esp_gatt_id_t* ps_id2) {
    if (ps_id1->inst_id == ps_id2->inst_id) {
        return b_ble_util_uuid_equal(&ps_id1->uuid, &ps_id2->uuid);
    }
    // 結果不一致
    return false;
}

/*******************************************************************************
 *
 * NAME: b_ble_util_uuid_equal
 *
 * DESCRIPTION:BLEのUUID比較処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bt_uuid_t*   ps_uuid1    R   比較値１
 * esp_bt_uuid_t*   ps_uuid2    R   比較値２
 *
 * RETURNS:
 * 比較結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_ble_util_uuid_equal(esp_bt_uuid_t* ps_uuid1, esp_bt_uuid_t* ps_uuid2) {
    if (ps_uuid1->len == ps_uuid2->len) {
        // アドレス比較
        return (memcmp(ps_uuid1->uuid.uuid128, ps_uuid2->uuid.uuid128, ps_uuid1->len) == 0);
    }
    // 不一致
    return false;
}

/*******************************************************************************
 *
 * NAME: b_ble_util_edit_base_uuid
 *
 * DESCRIPTION:BLEのBASE_UUID編集処理
 *
 * PARAMETERS:      Name        RW  Usage
 * uint8_t*         pu8_uuid    W   編集対象
 *
 * RETURNS:
 * true:編集完了
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_ble_util_edit_base_uuid(uint8_t* pu8_uuid) {
    // 入力チェック
    if (pu8_uuid == NULL) {
        return false;
    }
    // BASE UUID
    memcpy(pu8_uuid, u8_base_uuid, sizeof(u8_base_uuid));
    // 正常終了
    return true;
}

/*******************************************************************************
 *
 * NAME: i8_ble_util_power_dbm
 *
 * DESCRIPTION:BLEの電波の出力の値（DBM）の取得処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_power_level_t    e_power_level   R   電力
 *
 * RETURNS:
 * int8:電波の出力
 *
 * NOTES:
 * None.
 ******************************************************************************/
int8_t i8_ble_util_power_dbm(esp_power_level_t e_power_level) {
    // 入力チェック
    if (e_power_level < NTFW_BLE_PWR_LVL_MIN || e_power_level > NTFW_BLE_PWR_LVL_MAX) {
        return -128;
    }
    // 電波の出力を返信
    return i8_power_dbm_list[e_power_level];
}

/*******************************************************************************
 *
 * NAME: ps_ble_util_create_adv_data_raw
 *
 * DESCRIPTION:BLEのアドバタイズデータ(RAW形式)の生成処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_ble_adv_data_t*  ps_adv_data     R   アドバタイズデータ
 * char*                pc_dev_name     R   デバイス名
 * esp_power_level_t    e_tx_power      R   送信電力
 *
 * RETURNS:
 * ts_u8_array_t:編集されたRAWデータ
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_u8_array_t* ps_ble_util_create_adv_data_raw(esp_ble_adv_data_t* ps_adv_data, char* pc_dev_name, esp_power_level_t e_tx_power) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_adv_data == NULL) {
        return NULL;
    }

    //==========================================================================
    // 編集処理
    //==========================================================================
    // 編集バッファ
    uint8_t u8_raw_buff[251];
    uint8_t u8_raw_idx = 0;

    //--------------------------------------------------------------------------
    // データタイプ：Flags
    //--------------------------------------------------------------------------
    u8_raw_buff[u8_raw_idx++] = 0x02;              // サイズ
    u8_raw_buff[u8_raw_idx++] = 0x01;              // データタイプ
    u8_raw_buff[u8_raw_idx++] = ps_adv_data->flag; // データ

    //--------------------------------------------------------------------------
    // データタイプ：Complete list of 16-bit UUIDs available
    //--------------------------------------------------------------------------
    uint16_t u16_service_uuid_len = ps_adv_data->service_uuid_len;
    uint8_t* pu8_service_uuid = ps_adv_data->p_service_uuid;
    if (u16_service_uuid_len > 0 && (u16_service_uuid_len % 16) == 0 && pu8_service_uuid != NULL) {
        u8_raw_buff[u8_raw_idx++] = u16_service_uuid_len + 1;   // サイズ
        u8_raw_buff[u8_raw_idx++] = 0x03;                       // データタイプ
        memcpy(&u8_raw_buff[u8_raw_idx], pu8_service_uuid, u16_service_uuid_len);
        u8_raw_idx += u16_service_uuid_len;
    }

    //--------------------------------------------------------------------------
    // データタイプ：Complete Local Name
    //--------------------------------------------------------------------------
    if (ps_adv_data->include_name && pc_dev_name != NULL) {
        int i_name_len = strlen(pc_dev_name);
        if ((u8_raw_idx + 2 +i_name_len) > 251) {
            return NULL;
        }
        u8_raw_buff[u8_raw_idx++] = i_name_len + 1; // サイズ
        u8_raw_buff[u8_raw_idx++] = 0x09;           // データタイプ
        memcpy(&u8_raw_buff[u8_raw_idx], pc_dev_name, i_name_len);
        u8_raw_idx += i_name_len;
    }

    //--------------------------------------------------------------------------
    // データタイプ：TX Power Level
    //--------------------------------------------------------------------------
    if (ps_adv_data->include_txpower && u8_raw_idx <= 248) {
        // 値編集        
        u8_raw_buff[u8_raw_idx++] = 0x02;              // サイズ
        u8_raw_buff[u8_raw_idx++] = 0x0A;              // データタイプ
        u8_raw_buff[u8_raw_idx++] = i8_ble_util_power_dbm(e_tx_power);
    }

    //--------------------------------------------------------------------------
    // Service Data
    //--------------------------------------------------------------------------
    uint16_t u16_service_data_len = ps_adv_data->service_data_len;
    uint8_t* pu8_service_data = ps_adv_data->p_service_data;
    if (u16_service_data_len > 0 && pu8_service_data != NULL) {
        if ((u8_raw_idx + 2 + u16_service_data_len) > 251) {
            return NULL;
        }
        u8_raw_buff[u8_raw_idx++] = u16_service_data_len + 1;   // サイズ
        u8_raw_buff[u8_raw_idx++] = 0x16;                       // データタイプ
        memcpy(&u8_raw_buff[u8_raw_idx], pu8_service_data, u16_service_data_len);
        u8_raw_idx += u16_service_data_len;
    }

    //--------------------------------------------------------------------------
    // Manufacturer Specific Data
    //--------------------------------------------------------------------------
    uint16_t u16_manufacture_len = ps_adv_data->manufacturer_len;
    uint8_t* pu8_manufacturer_data = ps_adv_data->p_manufacturer_data;
    if (u16_manufacture_len > 0 && pu8_manufacturer_data != NULL) {
        if ((u8_raw_idx + 2 + u16_manufacture_len) > 251) {
            return NULL;
        }
        u8_raw_buff[u8_raw_idx++] = u16_manufacture_len + 1;    // サイズ
        u8_raw_buff[u8_raw_idx++] = 0xFF;                       // データタイプ
        memcpy(&u8_raw_buff[u8_raw_idx], pu8_manufacturer_data, u16_manufacture_len);
        u8_raw_idx += u16_manufacture_len;
    }
    
    //--------------------------------------------------------------------------
    // RAWデータのバイト配列生成
    //--------------------------------------------------------------------------
    return ps_mdl_clone_u8_array(u8_raw_buff, u8_raw_idx);
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
