/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :Bluetooth Low Energy Framework functions header file
 *
 * CREATED:2020/01/24 00:21:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:Bluetooth Low Energyのフレームワークとユーティリティ関数群
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
#ifndef  __NTFW_BLE_FMWK_H__
#define  __NTFW_BLE_FMWK_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>
#include <esp_gattc_api.h>
#include <ntfw_com_data_model.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** BLE Framework Debug */
//#define COM_BLE_DEBUG

/** Block Time */
#ifndef BLE_UTIL_BLOCK_TIME
    // デフォルト値は無制限にウェイト
    #define BLE_UTIL_BLOCK_TIME (portMAX_DELAY)
#endif
/** Retry Count */
#ifndef BLE_UTIL_RETRY_CNT
    // デフォルト値は5回
    #define BLE_UTIL_RETRY_CNT  (5)
#endif

//==============================================================================
// BLE共通
//==============================================================================
// 暗号化
#define ESP_BLE_SEC_NO_ENCRYPT (0)
// キーマスク
#define ESP_BLE_NO_KEY_MASK    (0)
// アドレスクリア
#define v_com_ble_addr_clear(t_bda) v_com_ble_addr_cpy(t_bda, t_com_ble_bda_none)
// アドレスクリア判定
#define b_com_ble_addr_clear(t_bda) (memcmp(t_bda, t_com_ble_bda_none, ESP_BD_ADDR_LEN) == 0)


//==============================================================================
// GATT関連の定義
//==============================================================================
/** サービスUUID */
#define COM_BLE_GATT_SVC_UUID_SIZE      (16)

/** アトリビュート値の最大サイズ */
#define COM_BLE_GATT_DATA_LEN_MAX       (512)

/** MTUサイズ（デフォルト） */
#ifndef COM_BLE_GATT_MTU_DEFAULT
    #define COM_BLE_GATT_MTU_DEFAULT    (23)    // MTUデフォルト
#endif
/** MTUサイズ（最大） */
#ifndef COM_BLE_GATT_MTU_MAX
    #define COM_BLE_GATT_MTU_MAX        (517)   // MTU最大値
#endif

/** GATTサーバーの受信書き込みデータバッファサイズ */
#ifndef COM_BLE_GATT_RX_BUFF_SIZE
    #define COM_BLE_GATT_RX_BUFF_SIZE   (32)
#endif

/** GATT 基本サービスUUID： 汎用アクセス*/
#define COM_BLE_GATT_SVC_UUID_GEN_ACCSESS   (0x1800)
/** GATT 基本サービスUUID： 汎用属性*/
#define COM_BLE_GATT_SVC_UUID_GEN_ATTRIBUTE (0x1801)

//==============================================================================
// GATT Clientの定義
//==============================================================================
#define GATTC_STS_CONNECTING    (GATTC_STS_REQUEST_OPEN | GATTC_STS_OPEN)
#define GATTC_STS_CONNECTED     (GATTC_STS_OPEN | GATTC_STS_SET_MTU | GATTC_STS_SET_SVC)

//==============================================================================
// SPP関連の定義
//==============================================================================
// SPPのサービスインデックス
#define BLE_SPPS_SVC_IDX        (0)
/** SPPサーバーのデータ受信処理 */
#define ps_com_ble_spps_rx_data(t_gatt_if, t_tick) ps_com_ble_gatts_rx_data(t_gatt_if, BLE_SPPS_SVC_IDX, t_tick)
/** SPPサーバーの受信バッファのクリア処理 */
#define v_com_ble_spps_rx_clear(t_gatt_if) v_com_ble_gatts_rx_clear(t_gatt_if, BLE_SPPS_SVC_IDX)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
//==============================================================================
// BLE Common Type Define
//==============================================================================
/** Bluetooth LE アドレス文字列型 */
typedef char tc_com_ble_bda_string_t[18];

/** 構造体：ボンディング済みデバイス情報 */
typedef struct {
    int i_device_cnt;                           // デバイス数
    esp_ble_bond_dev_t* ps_dev_list;            // デバイスリスト
} ts_com_ble_bond_dev_list_t;

//==============================================================================
// GAP Type Define
//==============================================================================
/** GAP Device Status */
typedef enum {
    GAP_DEV_STS_DEVICE_NONE     = (0x0000),         // デバイス無し
    GAP_DEV_STS_EXEC_RSSI       = (0x0001 << 0),    // 実行中フラグ：RSSI受信要求
    GAP_DEV_STS_SET_ADDRESS     = (0x0001 << 1),    // 設定済フラグ：アドレス
    GAP_DEV_STS_SET_NAME        = (0x0001 << 2),    // 設定済フラグ：デバイス名
    GAP_DEV_STS_SET_RSSI        = (0x0001 << 3),    // 設定済フラグ：RSSI
    GAP_DEV_STS_REQ_PASSKEY     = (0x0001 << 4),    // 要求受信フラグ：パスキーリクエスト
    GAP_DEV_STS_REQ_NUM_CHK     = (0x0001 << 5),    // 要求受信フラグ：番号確認リクエスト
    GAP_DEV_STS_RPY_PASSKEY     = (0x0001 << 6),    // 要求返信フラグ：パスキーリクエスト
    GAP_DEV_STS_RPY_NUM_CHK     = (0x0001 << 7),    // 要求返信フラグ：番号確認リクエスト
    GAP_DEV_STS_AUTHENTICATED   = (0x0001 << 8),    // 認証済みフラグ
    GAP_DEV_STS_DISCONNECTING   = (0x0001 << 9),    // 切断中
} te_gap_dev_sts_t;

/** 構造体：GAP・SMP（セキュリティ・マネージャー・プロトコル）の設定 */
typedef struct {
    char* pc_device_name;                       // デバイス名
    esp_ble_auth_req_t t_auth_req;              // 認証リクエストタイプ
    esp_ble_io_cap_t t_iocap;                   // デバイスのIO組み合わせ
    uint8_t u8_init_key;                        // 初期キーの設定
    uint8_t u8_rsp_key;                         // 応答キーの設定
    uint8_t u8_max_key_size;                    // 最大キーサイズ
    uint8_t u8_auth_option;                     // 受け入れ権限設定
    esp_gap_ble_cb_t v_callback;                // ユーザーコールバック関数のポインタ
} ts_com_ble_gap_config_t;

/** 構造体：GAPデバイス情報 */
typedef struct {
    esp_ble_addr_type_t e_addr_type;            // アドレスタイプ
    esp_bd_addr_t t_bda;                        // BLEアドレス
    char* pc_name;                              // BLEデバイス名
    int i_rssi;                                 // RSSI強度
    te_gap_dev_sts_t e_sts;                     // GAPデバイスステータス
} ts_com_ble_gap_device_info_t;

/** 構造体：GAPデバイスリスト */
typedef struct {
    bool b_scan_processing;                     // スキャン処理中フラグ
    uint32_t u32_scan_duration;                 // スキャン実行時間
    uint16_t u16_count;                         // スキャン結果件数
    ts_com_ble_gap_device_info_t* ps_device;    // スキャン結果
} ts_com_ble_gap_device_list_t;

//==============================================================================
// GATT Common Type Define
// アプリケーションID:インターフェースID:アトリビュートテーブル:サービスID=1:1:1:1
//==============================================================================
/** GATT受信データタイプ */
typedef enum {
    GATT_RX_TYPE_READ_DATA   = (0x00),      // 読み込みデータ
    GATT_RX_TYPE_READ_DESC   = (0x01),      // 読み込みディスクリプタ
    GATT_RX_TYPE_WRITE_DATA  = (0x02),      // 書き込みディスクリプタ
    GATT_RX_TYPE_NOTIFY      = (0x03),      // 通知データ
    GATT_RX_TYPE_INDICATE    = (0x04),      // 通知データ（受信応答）
} te_com_ble_gatt_rx_type_t;

/** GATTの受信データ情報 */
typedef struct {
    uint16_t u16_app_id;                    // アプリケーションID   ※キー１
    esp_gatt_if_t t_gatt_if;                // GATTインターフェース ※キー２
    uint16_t u16_con_id;                    // コネクションID       ※キー３
    esp_bd_addr_t t_bda;                    // リモートデバイスアドレス
    te_com_ble_gatt_rx_type_t e_type;       // 受信データタイプ
    esp_gatt_status_t t_status;             // 結果ステータス
    uint16_t u16_attr_hndl;                 // アトリビュートハンドル
    uint8_t u8_svc_idx;                     // サービスインデックス
    uint16_t u16_hndl_idx;                  // アトリビュートハンドルインデックス
    bool b_auto_rsp;                        // 自動返信
    ts_u8_array_t* ps_array;                // 受信データ配列　※解放時はFreeする事！！！
} ts_com_ble_gatt_rx_data_t;

//==============================================================================
// GATT Server Type Define
// アプリケーションID:インターフェースID=1:1
// インターフェースID:サービスID=1:N
// サービスID:アトリビュートテーブル=1:1
//==============================================================================
/** GATTプロファイルのサービス設定 */
typedef struct {
    uint8_t u8_inst_id;                             // サービスインスタンスID
    uint8_t u8_max_nb_attr;                         // アトリビュート要素数
    esp_gatts_attr_db_t* ps_attr_db;                // アトリビュートテーブル
} ts_com_ble_gatts_svc_config_t;

/** GATTプロファイルのインターフェース設定 */
typedef struct {
    uint16_t u16_app_id;                            // アプリケーションID
    esp_ble_sec_act_t e_con_sec;                    // 接続時のセキュリティタイプ
    uint8_t u8_svc_cnt;                             // サービス数
    ts_com_ble_gatts_svc_config_t* ps_svc_cfg;      // サービス情報配列
    esp_gatts_cb_t fc_gatts_cb;                     // インターフェース毎のコールバック関数
    void* pv_app_param;                             // アプリケーションパラメータ
    void* pv_usr_param;                             // ユーザー利用パラメータ
} ts_com_ble_gatts_if_config_t;

/** GATTサーバーのサービス情報（参照用） */
typedef struct {
    uint16_t u16_app_id;                // アプリケーションID   ※キー１（GATTインターフェースと1:1）
    esp_gatt_if_t t_gatt_if;            // GATTインターフェース ※キー２
    uint8_t u8_svc_inst_id;             // サービスインスタンスID
    uint16_t u16_num_handle;            // ハンドル数
    uint16_t *pu16_handles;             // アトリビュートハンドルリスト
} ts_com_ble_gatts_svc_info_t;

/** GATTクライアントからの接続情報（参照用） */
typedef struct {
    uint16_t u16_app_id;                // アプリケーションID   ※キー１（GATTインターフェースと1:1）
    esp_gatt_if_t t_gatt_if;            // GATTインターフェース ※キー２
    uint16_t u16_con_id;                // コネクションID       ※キー３
    esp_bd_addr_t t_bda;                // リモートデバイスアドレス
    esp_ble_addr_type_t e_addr_type;    // アドレスタイプ
    int i_rssi;                         // RSSI強度
    uint16_t u16_mtu;                   // MTUサイズ
} ts_com_ble_gatts_con_info_t;

//==============================================================================
// GATT Client Type Define
// アプリケーションID:インターフェースID:アトリビュートテーブル:サービスID=1:1:1:1
//==============================================================================
/** GATT Client Connection Status */
typedef enum {
    GATTC_STS_NONE              = (0x00),       // ステータス無し（未接続）
    GATTC_STS_REQUEST_OPEN      = (0x01 << 0),  // OPENリクエスト済み
    GATTC_STS_OPEN              = (0x01 << 1),  // OPEN
    GATTC_STS_REQUEST_MTU       = (0x01 << 2),  // MTUリクエスト済み
    GATTC_STS_SET_MTU           = (0x01 << 3),  // MTU設定済み
    GATTC_STS_SEARCH_SVC        = (0x01 << 4),  // サービス検索中
    GATTC_STS_SET_SVC           = (0x01 << 5),  // サービス検索済み
} te_gattc_con_sts_t;

/** GATTクライアント側のインターフェース設定 */
typedef struct {
    uint16_t u16_app_id;                        // アプリケーションID
    uint8_t u8_svc_cnt;                         // サービス数
    esp_bt_uuid_t* pt_svc_uuid;                 // サービスのUUID配列
    esp_ble_sec_act_t e_con_sec;                // 接続時のセキュリティタイプ
    esp_gattc_cb_t fc_gattc_cb;                 // インターフェース毎のコールバック関数
    void* pv_app_param;                         // アプリケーションパラメータ
    void* pv_usr_param;                         // ユーザー利用パラメータ
} ts_com_ble_gattc_if_config_t;

/** GATTサーバーのサービス情報 */
typedef struct s_com_ble_gattc_svc_info {
    esp_gatt_id_t s_svc_id;                     // サービスID
    bool     b_primary;                         // プライマリサービスフラグ
    uint16_t u16_svc_start_hndl;                // サービスのハンドル（開始）
    uint16_t u16_svc_end_hndl;                  // サービスのハンドル（終了）
    uint16_t u16_db_elem_cnt;                   // アトリビュートDBの要素数
    esp_gattc_db_elem_t* ps_db_elems;           // アトリビュートDB情報
} ts_com_ble_gattc_svc_info_t;

/** GATTサーバーのコネクション情報 */
typedef struct {
    // GATTインターフェースとアプリケーションIDは１：１の関係
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース ※キー１
    uint16_t u16_app_id;                        // アプリケーションID   ※キー２
    uint16_t u16_con_id;                        // コネクションID       ※キー３
    esp_bd_addr_t t_bda;                        // リモートデバイスアドレス
    uint16_t u16_mtu;                           // MTUサイズ
    esp_gatt_auth_req_t e_sec_auth_req;         // セキュアアクセスモード
    uint16_t u16_svc_cnt;                       // サービス数
    ts_com_ble_gattc_svc_info_t* ps_service;    // サービス情報
} ts_com_ble_gattc_con_info_t;

//==============================================================================
// SPP Type Define
//==============================================================================
/**
 * SPPサーバーのアトリビュートＤＢのインデックス
 */
typedef enum {
    SPPS_ATTR_IDX_SVC = 0,

    // クライアントからの受信データ
    SPPS_ATTR_IDX_RX_DATA_CHAR,
    SPPS_ATTR_IDX_RX_DATA_VAL,

    // サーバーからの送信通知データ
    SPPS_ATTR_IDX_TX_DATA_CHAR,
    SPPS_ATTR_IDX_TX_DATA_VAL,
    SPPS_ATTR_IDX_TX_DATA_CFG,

    // クライアントからの受信コマンド
    SPPS_ATTR_IDX_RX_CMD_CHAR,
    SPPS_ATTR_IDX_RX_CMD_VAL,

    // サーバーからの送信ステータス
    SPPS_ATTR_IDX_TX_STS_CHAR,
    SPPS_ATTR_IDX_TX_STS_VAL,
    SPPS_ATTR_IDX_TX_STS_CFG,

    // アトリビュートDBのサイズ
    SPPS_ATTR_IDX_NB,
} te_com_ble_spps_attr_idx_t;

/**
 * SPP接続ステータス
 */
typedef enum {
    COM_BLE_SPP_CON_DISCONNECTED = (0x00),  // 未接続
    COM_BLE_SPP_CON_CONNECTING,             // 接続中
    COM_BLE_SPP_CON_CONNECTED,              // 接続済み
    COM_BLE_SPP_CON_ERROR,                  // 接続エラー
} te_com_ble_spp_connection_sts_t;


/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** BLE Address None */
extern const esp_bd_addr_t t_com_ble_bda_none;

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/
//==============================================================================
// 文字列化処理
//==============================================================================
/** BLEアドレス文字列の取得 */
extern void v_com_ble_address_to_str(tc_com_ble_bda_string_t tc_bda, esp_bd_addr_t t_bda);
/** BLEキータイプ文字列の取得 */
extern const char* pc_com_ble_key_type_to_str(esp_ble_key_type_t t_key_type);
/** 認証リクエストタイプ文字列取得 */
extern const char* pc_com_ble_auth_req_to_str(esp_ble_auth_req_t auth_req);
/** BLEのGAPイベント文字列の取得 */
extern const char* pc_com_ble_gap_event_to_str(esp_gap_ble_cb_event_t e_event);
/** BLEのGATTサーバーイベント文字列の取得 */
extern const char* pc_com_ble_gatts_event_to_str(esp_gatts_cb_event_t e_event);
/** BLEのGATTクライアントイベント文字列の取得 */
extern const char* pc_com_ble_gattc_event_to_str(esp_gattc_cb_event_t e_event);

//==============================================================================
// 情報表示関係
//==============================================================================
/** BLEのボンディングデバイスの表示 */
extern esp_err_t sts_com_ble_display_bonded_devices();

//==============================================================================
// ユーティリティ
//==============================================================================
/** BLEのアドレスコピー処理 */
extern void v_com_ble_addr_cpy(esp_bd_addr_t t_to_bda, const esp_bd_addr_t t_from_bda);
/** BLEのアドレス比較処理 */
extern long l_com_ble_addr_cmp(const esp_bd_addr_t t_bda1, const esp_bd_addr_t t_bda2);
/** BLEのID比較処理 */
extern bool b_com_ble_id_equal(esp_gatt_id_t* ps_id1, esp_gatt_id_t* ps_id2);
/** BLEのUUID比較処理 */
extern bool b_com_ble_uuid_equal(esp_bt_uuid_t* ps_uuid1, esp_bt_uuid_t* ps_uuid2);
/** BLEのBASE_UUID編集処理 */
extern bool b_com_ble_edit_base_uuid(uint8_t* pu8_uuid);

//==============================================================================
// BLE共通
//==============================================================================
/** BLEの初期化処理 */
extern esp_err_t sts_com_ble_init();
/** BLEの切断処理 */
extern esp_err_t sts_com_ble_disconnect(esp_bd_addr_t t_bda);
/** BLEのボンディング済みデバイス判定処理 */
extern esp_err_t sts_com_ble_bonded_dev(esp_bd_addr_t t_bda);
/** BLEのボンディング済みデバイスリスト取得処理 */
extern ts_com_ble_bond_dev_list_t* ps_com_ble_bond_dev_list();
/** BLEのボンディング済みデバイスリストの削除処理 */
extern void v_com_ble_delete_bond_dev_list(ts_com_ble_bond_dev_list_t* ps_dev_list);
/** BLEのボンディング済みデバイスの全削除処理 */
extern esp_err_t sts_com_ble_disbonding_all();
/** BLEのボンディング済みデバイス削除処理 */
extern esp_err_t sts_com_ble_disbonding(esp_bd_addr_t t_bda);

//==============================================================================
// GAPプロファイル関係
//==============================================================================
/** BLEのGAPとSMPの初期設定処理 */
extern esp_err_t sts_com_ble_gap_smp_init(ts_com_ble_gap_config_t s_cfg);
/** BLEのパスキー応答処理 */
extern esp_err_t sts_com_ble_gap_passkey_reply(esp_bd_addr_t t_bda,
                                                bool b_accept,
                                                uint32_t u32_passkey);
/** BLEの番号確認応答処理 */
extern esp_err_t sts_com_ble_gap_confirm_reply(esp_bd_addr_t t_bda, bool b_accept);
/** BLEのSMPにおけるクリアPINコードの設定処理 */
extern esp_err_t sts_com_ble_gap_set_static_pass_key(uint32_t u32_static_passkey);
/** BLEのRSSIの読み取り処理 */
extern int i_com_ble_gap_read_rssi(esp_bd_addr_t t_bda);
/** BLEデバイスのステータス読み取り処理 */
extern te_gap_dev_sts_t e_com_ble_gap_device_sts(esp_bd_addr_t t_bda);
/** BLEデバイスのステータス更新待ち処理 */
extern te_gap_dev_sts_t e_com_ble_gap_device_sts_wait(esp_bd_addr_t t_bda,
                                                       te_gap_dev_sts_t e_chk_sts,
                                                       TickType_t t_max_wait);
/** BLEのアドレス把握済みのデバイスリストの生成処理 */
extern ts_com_ble_gap_device_list_t* ps_com_ble_gap_create_device_list();
/** BLEのアドレス把握済みのデバイスリストの削除処理 */
extern void v_com_ble_gap_delete_device_list(ts_com_ble_gap_device_list_t* ps_list);
/** 名称が一致するBLEのデバイス情報の生成処理 */
extern ts_com_ble_gap_device_info_t* ps_com_ble_gap_create_device_info(char* pc_device_name);
/** BLEのデバイス情報の削除処理 */
extern void v_com_ble_gap_delete_device_info(ts_com_ble_gap_device_info_t* ps_result);

//==============================================================================
// GAPプロファイルのアドバタイザ関係
//==============================================================================
/** アドバタイジングデータの設定処理 */
extern esp_err_t sts_com_ble_gap_set_adv_data(esp_ble_adv_data_t* ps_adv_data);
/** アドバタイジングの開始処理 */
extern esp_err_t sts_com_ble_gap_start_advertising(esp_ble_adv_params_t* ps_adv_params);
/** アドバタイジングの停止処理 */
extern esp_err_t sts_com_ble_gap_stop_advertising();
/** GAPアドバタイザの接続デバイスステータス取得処理 */
extern te_gap_dev_sts_t e_com_ble_gap_adv_device_status();
/** GAPアドバタイザのリモートBLEアドレスの編集処理 */
extern esp_err_t sts_com_ble_gap_adv_edit_remote_bda(esp_bd_addr_t t_rmt_bda);
/** GAPアドバタイザの接続デバイス情報の生成処理 */
extern ts_com_ble_gap_device_info_t* ps_com_ble_gap_adv_create_device_info();

//==============================================================================
// GAPプロファイルのスキャナ関係
//==============================================================================
/** スキャンパラメータの設定処理 */
extern esp_err_t sts_com_ble_gap_set_scan_params(esp_ble_scan_params_t* ps_scan_params);
/** スキャン中判定 */
extern bool b_com_ble_gap_is_scanning();
/** スキャンの開始処理 */
extern esp_err_t sts_com_ble_gap_start_scan(uint32_t u32_duration);
/** スキャンの停止処理 */
extern esp_err_t sts_com_ble_gap_stop_scan();
/** BLEのアドレス把握済みのデバイス数の取得 */
extern uint16_t u16_com_ble_gap_scan_device_count();

//==============================================================================
// GATTプロファイル共通
//==============================================================================
/** GATTプロファイルの受信データの削除処理 */
extern void v_com_ble_gatt_delete_rx_data(ts_com_ble_gatt_rx_data_t* ps_data);

//==============================================================================
// GATTプロファイルのサーバー関係
//==============================================================================
/** GATTサーバーの初期設定処理 */
extern esp_err_t sts_com_ble_gatts_init();
/** GATTサーバーのデフォルト設定の生成処理 */
extern ts_com_ble_gatts_if_config_t s_com_ble_gatts_app_config_default();
/** GATTサーバーへのアプリケーション登録処理 */
extern esp_err_t sts_com_ble_gatts_app_register(ts_com_ble_gatts_if_config_t* ps_if_cfg);
/** GATTサーバーのGATTインターフェース取得処理 */
extern esp_gatt_if_t t_com_ble_gatts_if(uint16_t u16_app_id);
/** GATTサーバーのサービス情報取得処理 */
extern ts_com_ble_gatts_svc_info_t s_com_ble_gatts_svc_info(esp_gatt_if_t t_gatt_if, uint8_t u8_svc_idx);
/** GATTサーバーへのコネクション有無判定 */
extern bool b_com_ble_gatts_is_connected(esp_gatt_if_t t_gatt_if);
/** GATTサーバーへのコネクション情報生成 */
extern ts_com_ble_gatts_con_info_t* ps_com_ble_gatts_create_con_info(esp_gatt_if_t t_gatt_if);
/** GATTサーバーへのコネクション情報削除処理 */
extern void v_com_ble_gatts_delete_con_info(ts_com_ble_gatts_con_info_t* ps_con_info);
/** GATTサーバーのアトリビュートハンドルインデックスの取得処理 */
extern esp_err_t sts_com_ble_gatts_get_handle_idx(esp_gatt_if_t t_gatt_if,
                                                   uint16_t u16_handle,
                                                   uint8_t* pu8_svc_idx,
                                                   uint16_t* pu16_hndl_idx);
/** GATTサーバーのアトリビュートの取得処理 */
extern esp_gatts_attr_db_t* ps_com_ble_gatts_get_attribute(esp_gatt_if_t t_gatt_if,
                                                            uint8_t u8_svc_idx,
                                                            uint16_t u16_hndl_idx);
/** GATTプロファイルの受信バッファからのデータ取得処理 */
extern ts_com_ble_gatt_rx_data_t* ps_com_ble_gatts_rx_data(esp_gatt_if_t t_gatt_if,
                                                          uint8_t u8_svc_idx,
                                                          TickType_t t_tick);
/** GATTプロファイルの受信バッファのクリア処理 */
extern void v_com_ble_gatts_rx_clear(esp_gatt_if_t t_gatt_if,
                                      uint8_t u8_svc_idx);
/** GATTサーバーからのレスポンスの送信処理 */
extern esp_err_t sts_com_ble_gatts_tx_data(esp_gatt_if_t t_gatt_if,
                                            struct gatts_read_evt_param* ps_param,
                                            uint8_t u8_auth_req,
                                            uint8_t* pu8_value,
                                            uint16_t u16_length);

/** GATTプロファイルにおけるIndicateの送信処理 */
extern esp_err_t sts_com_ble_gatts_indicate(esp_gatt_if_t t_gatt_if,
                                             uint8_t u8_svc_idx,
                                             uint16_t u16_handle,
                                             uint8_t* pu8_data,
                                             size_t t_data_len);
/** GATTプロファイルにおけるNotifyの送信処理 */
extern esp_err_t sts_com_ble_gatts_notify(esp_gatt_if_t t_gatt_if,
                                           uint8_t u8_svc_idx,
                                           uint16_t u16_handle,
                                           uint8_t* pu8_data,
                                           size_t t_data_len);

//==============================================================================
// GATTプロファイルのクライアント関係
//==============================================================================
/** GATTクライアントのデフォルト設定の生成処理 */
extern ts_com_ble_gattc_if_config_t s_com_ble_gattc_app_config_default();
/** GATTクライアントのプロファイル初期処理 */
extern esp_err_t sts_com_ble_gattc_register(ts_com_ble_gattc_if_config_t* ps_app_info, uint16_t u16_size);
/** GATTクライアントのGATTインターフェース取得処理 */
extern esp_gatt_if_t t_com_ble_gattc_if(uint16_t u16_app_id);
/** GATTサーバーとの接続処理 */
extern esp_err_t sts_com_ble_gattc_open(esp_gatt_if_t t_gatt_if,
                                         esp_bd_addr_t t_bda,
                                         esp_ble_addr_type_t e_remote_addr_type,
                                         bool b_direct);
/** GATTサーバーとの切断処理 */
extern esp_err_t sts_com_ble_gattc_close(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda);
/** GATTサーバーとのコネクションステータス取得 */
extern te_gattc_con_sts_t e_com_ble_gattc_con_sts(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda);
/** GATTサーバーとのコネクションステータス更新待ち処理 */
extern te_gattc_con_sts_t e_com_ble_gattc_con_sts_wait(esp_gatt_if_t t_gatt_if,
                                                        esp_bd_addr_t t_bda,
                                                        te_gattc_con_sts_t e_chk_sts,
                                                        TickType_t t_max_wait);
/** GATTサーバーとのコネクション情報取得処理 */
extern ts_com_ble_gattc_con_info_t* ps_com_ble_gattc_create_con_info(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda);
/** GATTサーバーとのコネクション情報削除処理 */
extern void v_com_ble_gattc_delete_con_info(ts_com_ble_gattc_con_info_t* ps_con_info);
/** GATTサーバーからのアトリビュートDBの取得処理 */
extern esp_err_t sts_com_ble_gattc_get_db(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id, esp_gatt_id_t s_svc_id);
/** GATTサーバーからの取得したアトリビュートDBのローカルキャッシュのリフレッシュ処理 */
extern esp_err_t sts_com_ble_gattc_cache_clean(esp_gatt_if_t t_gatt_if);
/** GATTサーバーからGATTクライアントの通知有効化処理 */
extern esp_err_t sts_com_ble_gattc_enable_notify(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id, esp_gatt_id_t s_svc_id);
/** GATTサーバーへのDescriptor書き込み処理 */
extern esp_err_t sts_com_ble_gattc_write_cccd(esp_gatt_if_t t_gatt_if,
                                               uint16_t u16_con_id,
                                               uint16_t u16_char_handle,
                                               uint8_t u8_value,
                                               esp_gatt_write_type_t e_write_type,
                                               esp_gatt_auth_req_t e_auth_req);
/** GATTサーバーからの受信データ取得処理 */
extern ts_com_ble_gatt_rx_data_t* ps_com_ble_gattc_rx_data(esp_gatt_if_t t_gatt_if,
                                                          uint16_t u16_con_id,
                                                          esp_gatt_id_t s_svc_id,
                                                          TickType_t t_tick);
/** GATTサーバーからの受信バッファクリア処理 */
extern void v_com_ble_gattc_rx_clear(esp_gatt_if_t t_gatt_if,
                                      uint16_t u16_con_id,
                                      esp_gatt_id_t s_svc_id);

//==============================================================================
// SPPプロファイルのサーバー側処理
//==============================================================================
/** SPPアプリケーション設定の生成処理 */
extern ts_com_ble_gatts_if_config_t s_com_ble_spps_config(esp_gatt_perm_t t_perm_read, esp_gatt_perm_t t_perm_write);
/** SPPサーバーのユーザーコールバック関数の設定処理 */
extern void v_com_ble_spps_set_usr_cb(esp_gatts_cb_t fc_spps_cb);
/** SPPサーバーのSPPサービス情報の生成処理 */
extern ts_com_ble_gatts_svc_config_t* ps_com_ble_spps_create_svc(esp_gatt_perm_t t_perm_read, esp_gatt_perm_t t_perm_write);
/** SPPサーバーのSPPサービス情報の削除処理 */
extern void v_com_ble_spps_delete_svc(ts_com_ble_gatts_svc_config_t* ps_svc);
/** SPPサーバーのデータ送信処理 */
extern esp_err_t sts_com_ble_spps_tx_data(esp_gatt_if_t t_gatt_if, uint8_t* pu8_data, size_t t_len);

//==============================================================================
// SPPプロファイルのクライアント側処理
//==============================================================================
/** SPPアプリケーション設定の生成処理 */
extern ts_com_ble_gattc_if_config_t s_com_ble_sppc_config();
/** SPPクライアントのユーザーコールバック関数の設定処理 */
extern void v_com_ble_sppc_set_usr_cb(esp_gattc_cb_t fc_sppc_cb);
/** SPP接続ステータスの取得処理 */
extern te_com_ble_spp_connection_sts_t e_com_ble_sppc_con_sts(ts_com_ble_gattc_con_info_t* ps_con);
/** SPPクライアントからのデータ送信処理 */
extern esp_err_t sts_com_ble_sppc_tx_data(ts_com_ble_gattc_con_info_t* ps_con, uint8_t* pu8_data, size_t t_len);
/** SPPクライアントでのデータ受信処理 */
extern ts_com_ble_gatt_rx_data_t* ps_com_ble_sppc_rx_data(ts_com_ble_gattc_con_info_t* ps_con, TickType_t t_tick);
/** SPPクライアントからの受信バッファクリア処理 */
extern void v_com_ble_sppc_rx_clear(ts_com_ble_gattc_con_info_t* ps_con);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_BLE_FMWK_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
