/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :Bluetooth Low Energy Framework functions source file
 *
 * CREATED:2020/01/24 01:34:00
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
#include <ntfw_ble_fmwk.h>

#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <ntfw_com_value_util.h>
#include <ntfw_com_mem_alloc.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** ENUM文字列取得マクロ */
#define pc_enum_str(e_val) #e_val

/** ログ接頭辞 */
#define LOG_TAG "COM_BLE_FMWK"
/** データ型サイズ：uint8_t */
#define DEF_SIZE_CHAR       (sizeof(uint8_t))
/** 待ち時間：処理待ち時間 */
#ifndef BLE_PROCESS_WAIT_TICK
    #define BLE_PROCESS_WAIT_TICK   (20 / portTICK_PERIOD_MS)
#endif

//==============================================================================
// GAP関係の定数
//==============================================================================
/** 待ち時間：GAPデバイスステータス更新待ち時間 */
#ifndef GAP_DEVICE_STS_UPD_WAIT_TICK
    #define GAP_DEVICE_STS_UPD_WAIT_TICK (20 / portTICK_PERIOD_MS)
#endif
/** GAPステータス：アドバタイズの実行判定 */
#define GAP_STS_CHK_EXEC_ADVERTISE (GAP_STS_EXEC_CONFIG_PRIVACY | GAP_STS_EXEC_CONFIG_ADVERTISE | GAP_STS_EXEC_ADVERTISING)
/** GAPステータス：スキャンパラメータ設定 */
#define GAP_STS_SET_SCAN_CFG    (GAP_STS_WAIT_CONFIG_SCAN | GAP_STS_SET_CONFIG_SCAN)
/** GAPステータス：スキャン実行可能判定 */
#define GAP_STS_CHK_SCAN_EXEC   (GAP_STS_WAIT_SCAN | GAP_STS_SET_CONFIG_PRIVACY | GAP_STS_SET_CONFIG_SCAN)
/** GAPステータス：スキャン開始 */
#define GAP_STS_START_SCAN  (GAP_STS_WAIT_SCAN | GAP_STS_EXEC_SCAN)
/** GAPデバイスステータス：認証処理 */
#define GAP_DEV_STS_AUTH    (GAP_DEV_STS_REQ_PASSKEY | GAP_DEV_STS_REQ_NUM_CHK | GAP_DEV_STS_AUTHENTICATED)

//==============================================================================
// GATT関係の定数
//==============================================================================
#define INVALID_HANDLE      (0)
/** 待ち時間：GATTステータス更新待ち時間 */
#ifndef GATT_CON_STS_UPD_WAIT_TICK
    #define GATT_CON_STS_UPD_WAIT_TICK  (20 / portTICK_PERIOD_MS)
#endif
/** 待ち時間：GATTデータ送信待ち */
#ifndef GATT_TX_WAIT_TICK
    #define GATT_TX_WAIT_TICK   (20 / portTICK_PERIOD_MS)
#endif

//==============================================================================
// GATTClient関係の定数
//==============================================================================
/** GATT Client サービス検索判定マスク */
#define GATTC_STS_SEARCH_SVC_MASK   (GATTC_STS_OPEN | GATTC_STS_SET_MTU | GATTC_STS_SEARCH_SVC | GATTC_STS_SET_SVC)
/** GATT Client サービス検索判定パターン */
#define GATTC_STS_SEARCH_SVC_PTN    (GATTC_STS_OPEN | GATTC_STS_SET_MTU)

//==============================================================================
// SPP関係の定数
//==============================================================================
// SPPサーバーのデータキューイングウェイトタイム
#ifndef BLE_SPP_QUEUE_WAIT
    // デフォルト値は無制限にウェイト
    #define BLE_SPP_QUEUE_WAIT      (portMAX_DELAY)
#endif

/** BLE GATTプロファイル情報配列のSPPインターフェースインデックス */
#define BLE_SPPS_IF_IDX             (0)
#define BLE_SPPS_SVC_INST_IDX       (0)         // SPPのサービスインスタンスインデックス
#define BLE_SPPS_CMD_MAX_LEN        (20)        // SPPのコマンド最大長
#define BLE_SPPS_STS_MAX_LEN        (20)        // SPPのデータ最大長
// Characteristic Value
#define BLE_SPPS_UUID_SERVICE       (0xABF0)    // サービス値
#define BLE_SPPS_UUID_RX_DATA       (0xABF1)    // 受信データ値
#define BLE_SPPS_UUID_TX_DATA       (0xABF2)    // 送信データ値
#define BLE_SPPS_UUID_RX_CMD        (0xABF3)    // 受信コマンド値
#define BLE_SPPS_UUID_TX_STS        (0xABF4)    // ステータス値

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

//==============================================================================
// GAP関係の型定義
//==============================================================================
/** GAP Status */
typedef enum {
    GAP_STS_WAIT_CONFIG_ADVERTISE = (0x00000001 << 0),  // 実行待ちフラグ：アドバタイズデータ設定
    GAP_STS_WAIT_CONFIG_SCAN_RSP  = (0x00000001 << 1),  // 実行待ちフラグ：スキャン応答データ設定
    GAP_STS_WAIT_CONFIG_SCAN      = (0x00000001 << 2),  // 実行待ちフラグ：スキャン設定
    GAP_STS_WAIT_ADVERTISING      = (0x00000001 << 3),  // 実行待ちフラグ：アドバタイズ
    GAP_STS_WAIT_SCAN             = (0x00000001 << 4),  // 実行待ちフラグ：スキャン
    GAP_STS_EXEC_CONFIG_PRIVACY   = (0x00000001 << 5),  // 実行中フラグ：プライバシー機能の設定
    GAP_STS_EXEC_CONFIG_ADVERTISE = (0x00000001 << 6),  // 実行中フラグ：アドバタイズデータ設定
    GAP_STS_EXEC_CONFIG_SCAN_RSP  = (0x00000001 << 7),  // 実行中フラグ：スキャン応答データ設定
    GAP_STS_EXEC_CONFIG_SCAN      = (0x00000001 << 8),  // 実行中フラグ：スキャン設定
    GAP_STS_EXEC_ADVERTISING      = (0x00000001 << 9),  // 実行中フラグ：アドバタイズ
    GAP_STS_EXEC_SCAN             = (0x00000001 << 10), // 実行中フラグ：スキャン
    GAP_STS_EXEC_BONDING          = (0x00000001 << 11), // 実行中フラグ：ボンディング
    GAP_STS_SET_CONFIG_PRIVACY    = (0x00000001 << 12), // 設定済フラグ：プライバシー機能の設定
    GAP_STS_SET_CONFIG_ADVERTISE  = (0x00000001 << 13), // 設定済フラグ：アドバタイズデータ
    GAP_STS_SET_CONFIG_SCAN_RSP   = (0x00000001 << 14), // 設定済フラグ：スキャン応答データ
    GAP_STS_SET_CONFIG_SCAN       = (0x00000001 << 15), // 設定済フラグ：スキャン設定
} te_gap_sts_t;

/** 構造体：GAPプロファイルの制御ステータス */
typedef struct {
    uint32_t              u32_status;           // 状態ステータス
    esp_ble_adv_data_t    s_adv_config;         // GAPアドバタイズデータ設定
    esp_ble_adv_data_t    s_scan_rsp_config;    // GAPスキャン応答データ設定
    esp_ble_scan_params_t s_scan_config;        // スキャン設定
    esp_ble_adv_params_t  s_adv_params;         // アドバタイズパラメータ
    uint32_t              u32_scan_duration;    // スキャン実行時間
    int64_t               i64_scan_timeout;     // スキャンタイムアウト時刻
} ts_gap_status_t;

// 接続先のデバイス名として登録されるか、接続先情報を受信するかで領域確保する、接続状態をステータス管理する。
/** 構造体：GAPプロファイルのデバイス情報 */
typedef struct s_gap_device_t {
    uint16_t              u16_status;           // 接続ステータス
    esp_ble_addr_type_t   e_addr_type;          // アドレスタイプ
    esp_bd_addr_t         t_bda;                // 接続アドレス
    char*                 pc_name;              // 接続対象デバイス名
    int                   i_rssi;               // RSSI強度
    esp_ble_auth_req_t    t_auth_mode;          // 認証モード
    struct s_gap_device_t* ps_next;             // 次のデバイス情報
} ts_gap_device_t;

/** 構造体：GAPプロファイルの制御情報 */
typedef struct {
    // GAPの設定情報
    ts_com_ble_gap_config_t s_config;
    // GAPのステータス情報
    ts_gap_status_t s_status;
    // GAPデバイス情報数
    uint16_t u16_dev_cnt;
    // GAPのデバイス情報
    ts_gap_device_t* ps_device;
} ts_gap_ctrl_t;

//==============================================================================
// GATTサーバー関係の型定義
//==============================================================================
/** GATT Server Status */
typedef enum {
    GATTS_STS_INIT      = (0x01 << 0),          // 初期化完了
    GATTS_STS_IF_CFG    = (0x01 << 1),          // インターフェース設定済み
} te_gatts_sts_t;

/** GATTプロファイルのサービス制御ステータス */
typedef struct {
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース   ※キー１
    uint16_t u16_app_id;                        // アプリケーションID     ※キー２（GATTインターフェースと1対1）
    uint8_t u8_svc_inst_id;                     // サービスインスタンスID ※キー３
    ts_com_ble_gatts_svc_config_t s_cfg;        // サービス設定
    uint8_t u8_max_nb_attr;                     // アトリビュート要素数
    uint16_t u16_num_handle;                    // ハンドル数
    uint16_t *pu16_handles;                     // アトリビュートハンドルリスト
    QueueHandle_t t_rx_queue;                   // 受信書き込みデータキュー
} ts_gatts_svc_status_t;

/** GATTサーバーのコネクション制御ステータス定義 */
typedef struct s_gatts_con_status_t {
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース ※キー１
    uint16_t u16_app_id;                        // アプリケーションID   ※キー２（GATTインターフェースと1対1）
    uint16_t u16_con_id;                        // コネクションID       ※キー３
    esp_bd_addr_t t_bda;                        // リモートデバイスアドレス
    uint16_t u16_mtu;                           // MTUサイズ
    esp_gatts_attr_db_t* ps_rx_buff_attr;       // 受信中のアトリビュート
    ts_com_ble_gatt_rx_data_t* ps_rx_buff_data; // 受信中のデータ
    ts_linked_queue_t* ps_rx_buff;              // 分割受信中のデータバッファ
    struct s_gatts_con_status_t* ps_next;       // 次のステータス
} ts_gatts_con_status_t;

/** GATTサーバーのインターフェース制御ステータス定義 */
typedef struct s_gatts_if_status_t {
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース ※キー１
    uint16_t u16_app_id;                        // アプリケーションID   ※キー２（GATTインターフェースと1対1）
    ts_com_ble_gatts_if_config_t s_cfg;         // GATTインターフェース設定
    uint8_t u8_svc_cnt;                         // GATTサーバーのサービス数
    ts_gatts_svc_status_t* ps_svc_sts;          // GATTサーバーのサービスステータス
    ts_gatts_con_status_t* ps_con_sts;          // GATTサーバーのコネクションステータス
    struct s_gatts_if_status_t* ps_next;        // 次のステータス
} ts_gatts_if_status_t;

/** 構造体：GATTサーバーの制御情報 */
typedef struct {
    uint8_t u8_status;                          // GATTサーバーステータス
    ts_gatts_if_status_t* ps_if_status;         // GATTサーバーのアプリケーションステータス
} ts_gatts_ctrl_t;

//==============================================================================
// GATTクライアント関係の型定義
//==============================================================================
/** GATTクライアントのNotify受信情報 */
typedef struct {
    uint16_t u16_handle;                    // 受信中のハンドル
    uint8_t u8_pkt_all;                     // 受信中の全パケット数
    uint8_t u8_pkt_num;                     // 受信中の現在パケット番号
    ts_linked_queue_t* ps_queue;            // Notifyデータの受信キュー
} ts_gattc_rcv_notify_status_t;

/** GATTクライアントのサービスステータス */
typedef struct s_gattc_svc_status_t {
    esp_gatt_id_t s_svc_id;                     // サービスID
    uint8_t u8_svc_idx;                         // サービスインデックス
    bool     b_primary;                         // プライマリサービスフラグ
    uint16_t u16_start_hndl;                    // アトリビュートハンドル（開始）
    uint16_t u16_end_hndl;                      // アトリビュートハンドル（終了）
    uint16_t u16_db_elem_cnt;                   // アトリビュートDBの要素数
    esp_gattc_db_elem_t* ps_db_elems;           // アトリビュートDB情報
    ts_gattc_rcv_notify_status_t s_notify;      // Notifyステータス
    QueueHandle_t t_rx_queue;                   // RXデータキュー
    struct s_gattc_con_status_t* ps_con_sts;    // コネクションステータス
    struct s_gattc_svc_status_t* ps_next;       // 次のサービスステータス
} ts_gattc_svc_status_t;

/** GATTクライアントのコネクション制御ステータス */
typedef struct s_gattc_con_status_t {
    // GATTインターフェースとアプリケーションIDは１：１の関係
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース ※キー１
    uint16_t u16_app_id;                        // アプリケーションID   ※キー２
    uint16_t u16_con_id;                        // コネクションID       ※キー３
    uint8_t u8_status;                          // ステータス
    esp_bd_addr_t t_bda;                        // リモートデバイスアドレス
    uint16_t u16_mtu;                           // MTUサイズ
    esp_gatt_auth_req_t e_sec_auth_req;         // セキュアアクセスモード
    uint16_t u16_svc_cnt;                       // サービス数
    ts_gattc_svc_status_t* ps_svc_sts;          // サービス情報
    struct s_gattc_con_status_t* ps_next;       // 次のステータス
} ts_gattc_con_status_t;

/** GATTクライアントのインターフェース制御ステータス定義 */
typedef struct s_gattc_if_status_t {
    // GATTインターフェースとアプリケーションIDは１：１の関係
    esp_gatt_if_t t_gatt_if;                    // GATTインターフェース ※キー１
    uint16_t u16_app_id;                        // アプリケーションID   ※キー２
    ts_com_ble_gattc_if_config_t* ps_if_cfg;    // GATTインターフェース設定
    bool b_req_cache_clear;                     // GATTクライアントのキャッシュクリア要求フラグ
    ts_gattc_con_status_t* ps_con_sts;          // GATTクライアントのコネクションステータス
} ts_gattc_if_status_t;

/** GATTクライアントの制御情報 */
typedef struct {
    // GATTクライアントアプリケーション情報の数
    uint16_t u16_if_count;
    // GATTクライアントインターフェース設定
    ts_com_ble_gattc_if_config_t* ps_if_config;
    // GATTクライアントアプリケーションステータス
    ts_gattc_if_status_t* ps_if_status;
} ts_gattc_ctrl_t;

//==============================================================================
// SPPサーバー関係の型定義
//==============================================================================
/** SPPサーバーのuuid */
typedef struct {
    uint16_t u16_service;                   // UUID:SPP Service
    uint16_t u16_primary_service;           // UUID:プライマリサービス
    uint16_t u16_character_declaration;     // UUID:キャラクタリスティック定義
    uint16_t u16_character_client_config;   // UUID:クライアントキャラクタリスティック設定
    uint16_t u16_rx_data;                   // UUID:受信データのキャラクタリスティック
    uint16_t u16_tx_data;                   // UUID:送信データのキャラクタリスティック
    uint16_t u16_rx_cmd;                    // UUID:コマンドのキャラクタリスティック
    uint16_t u16_tx_sts;                    // UUID:ステータスのキャラクタリスティック
} ts_spps_uuid_t;

/** SPPサーバーのアトリビュートのデータベース設定 */
typedef struct {
    uint8_t u8_char_prop_read_notify;       // Notifyデータのプロパティ
    uint8_t u8_char_prop_read_write;        // 読み書きデータのプロパティ
    uint8_t u8_char_prop_rw_auth;           // 読み書きデータのプロパティ（認証付き）
    uint8_t u8_val_data_receive[32];        // 受信データのキャラクタリスティック値
    uint8_t u8_val_data_notify[32];         // 通知データのキャラクタリスティック値
    uint8_t u8_val_command[16];             // コマンドのキャラクタリスティック値
    uint8_t u8_val_status[16];              // ステータスのキャラクタリスティック値
    uint8_t u8_cccd_data_notify[2];         // 通知データのCCCD
    uint8_t u8_cccd_status[2];              // ステータスのCCCD
} ts_spps_values_t;

/** SPPサーバーのコネクション制御ステータス定義 */
typedef struct s_spps_status_t {
    esp_gatt_if_t t_gatt_if;                // GATTインターフェース   ※キー１
    uint8_t u8_svc_idx;                     // サービスインデックス   ※キー２
    uint8_t u8_svc_inst_id;                 // サービスインスタンスID
    bool b_notify_data;                     // Data Notifyの有効フラグ
    bool b_notify_status;                   // Status Notifyの有効フラグ
    uint16_t u16_hndl_data_ntf;             // アトリビュートハンドル：データ通知
    struct s_spps_status_t* ps_next;        // 次のステータス
} ts_spps_status_t;

//==============================================================================
// SPPクライアント関係の型定義
//==============================================================================
/**
 * SPPクライアントのアトリビュートＤＢのインデックス
 */
typedef enum{
    SPPC_ATTR_IDX_SVC,

    // サーバーへの送信データ
    SPPC_ATTR_IDX_TX_DATA_VAL,

    // サーバーからの受信データ
    SPPC_ATTR_IDX_RX_DATA_VAL,
    SPPC_ATTR_IDX_RX_DATA_CFG,

    // サーバーへの送信コマンド
    SPPC_ATTR_IDX_TX_CMD_VAL,

    // サーバーからの受信ステータス
    SPPC_ATTR_IDX_RX_STS_VAL,
    SPPC_ATTR_IDX_RX_STS_CFG,

    // アトリビュートDBのサイズ
    SPPC_ATTR_IDX_NB
} te_com_ble_spp_c_attr_idx_t;

/** SPPクライアントのコネクション制御ステータス定義 */
typedef struct s_sppc_status_t {
    // GATTインターフェースとアプリケーションIDは１：１の関係
    esp_gatt_if_t t_gatt_if;                // GATTインターフェース     ※キー１
    uint16_t u16_con_id;                    // コネクションID           ※キー２
    ts_gattc_con_status_t* ps_con_sts;      // 対象コネクションステータス
    ts_gattc_svc_status_t* ps_svc_sts;      // 対象サービスステータス
    uint16_t u16_hndl_tx_data;              // アトリビュートハンドル：送信データ項目
    uint16_t u16_hndl_tx_cmd;               // アトリビュートハンドル：送信コマンド項目
    uint16_t u16_hndl_rx_data;              // アトリビュートハンドル：受信データ項目
    uint16_t u16_hndl_notify[2];            // Notify項目ハンドル（有効化済み）
    struct s_sppc_status_t* ps_next;        // 次のステータス
} ts_sppc_status_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** BLE Address None */
const esp_bd_addr_t t_com_ble_bda_none = {0x40};

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** BASE UUID */
static const uint8_t u8_base_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

//==============================================================================
// GAP関係の定数定義
//==============================================================================
/** GAPプロファイルのステータスデフォルト値 */
static const ts_com_ble_gap_config_t s_gap_cfg_default = {
        .pc_device_name  = NULL,                // デバイス名
        .t_auth_req      = ESP_LE_AUTH_NO_BOND, // 認証リクエストタイプ
        .t_iocap         = ESP_IO_CAP_NONE,     // デバイスのIO組み合わせ
        .u8_init_key     = 0x00,                // 初期キーの設定
        .u8_rsp_key      = 0x00,                // 応答キーの設定
        .u8_max_key_size = 16,                  // 最大キーサイズ
        .u8_auth_option  = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE,    // 受け入れ権限設定
        .v_callback      = NULL                 // ユーザーコールバック関数のポインタ
};

/** GAPプロファイルのステータスデフォルト値 */
static const ts_gap_status_t s_gap_sts_default = {
        .u32_status        = 0x00,              // 状態ステータス
        .s_adv_config      = {0x00},            // GAPアドバタイズデータ設定
        .s_scan_rsp_config = {0x00},            // GAPスキャン応答データ設定
        .s_scan_config     = {0x00},            // スキャン設定
        .s_adv_params      = {0x00},            // アドバタイズパラメータ
        .u32_scan_duration = 0x00,              // スキャン実行時間
        .i64_scan_timeout  = 0                  // スキャンタイムアウト
};

/** GAPプロファイルのデバイスステータスのデフォルト値 */
static const ts_gap_device_t s_gap_dev_default = {
        .u16_status  = 0x00,                    // ステータス
        .e_addr_type = BLE_ADDR_TYPE_PUBLIC,    // アドレスタイプ
        .t_bda       = {0x40},                  // アドレス
        .pc_name     = NULL,                    // 接続対象デバイス名
        .i_rssi      = 127,                     // RSSI
        .t_auth_mode = ESP_LE_AUTH_NO_BOND,     // 認証モード
        .ps_next     = NULL                     // 次の接続情報
};

//==============================================================================
// GATTサーバー関係の定数定義
//==============================================================================
/** GATTサーバーイベント名 */
static const char* pc_ble_gatts_evt_str[] = {
    "ESP_GATTS_REG_EVT",
    "ESP_GATTS_READ_EVT",
    "ESP_GATTS_WRITE_EVT",
    "ESP_GATTS_EXEC_WRITE_EVT",
    "ESP_GATTS_MTU_EVT",
    "ESP_GATTS_CONF_EVT",
    "ESP_GATTS_UNREG_EVT",
    "ESP_GATTS_CREATE_EVT",
    "ESP_GATTS_ADD_INCL_SRVC_EVT",
    "ESP_GATTS_ADD_CHAR_EVT",
    "ESP_GATTS_ADD_CHAR_DESCR_EVT",
    "ESP_GATTS_DELETE_EVT",
    "ESP_GATTS_START_EVT",
    "ESP_GATTS_STOP_EVT",
    "ESP_GATTS_CONNECT_EVT",
    "ESP_GATTS_DISCONNECT_EVT",
    "ESP_GATTS_OPEN_EVT",
    "ESP_GATTS_CANCEL_OPEN_EVT",
    "ESP_GATTS_CLOSE_EVT",
    "ESP_GATTS_LISTEN_EVT",
    "ESP_GATTS_CONGEST_EVT",
    "ESP_GATTS_RESPONSE_EVT",
    "ESP_GATTS_CREAT_ATTR_TAB_EVT",
    "ESP_GATTS_SET_ATTR_VAL_EVT",
    "ESP_GATTS_SEND_SERVICE_CHANGE_EVT"
};

/**
 * GATTサーバーアプリケーション設定のデフォルト値
 */
static const ts_com_ble_gatts_if_config_t s_gatts_cfg_default = {
    .u16_app_id   = 0,              // アプリケーションID
    .e_con_sec    = 0,              // 接続時のセキュリティタイプ
    .u8_svc_cnt   = 0,              // サービス数
    .ps_svc_cfg   = NULL,           // サービス設定配列
    .fc_gatts_cb  = NULL,           // インターフェース毎のコールバック関数
    .pv_app_param = NULL,           // アプリケーションパラメータ
    .pv_usr_param = NULL            // ユーザー利用パラメータ
};

/**
 * GATTプロファイルのサーバーステータス
 */
static const ts_gatts_if_status_t s_gatts_if_sts_default = {
    .t_gatt_if  = ESP_GATT_IF_NONE,     // GATTインターフェース ※キー１
    .u16_app_id = 0,                    // アプリケーションID   ※キー２（GATTインターフェースと1対1）
    .s_cfg      = s_gatts_cfg_default,  // GATTインターフェース設定
    .u8_svc_cnt = 0,                    // GATTサービス数
    .ps_svc_sts = NULL,                 // GATTサーバーのサービスステータス
    .ps_con_sts = NULL,                 // GATTサーバーのコネクションステータス
    .ps_next    = NULL,                 // 次のステータス
};

/**
 * GATTプロファイルのコネクションステータス
 */
static const ts_gatts_con_status_t s_gatts_con_sts_default = {
    .t_gatt_if        = ESP_GATT_IF_NONE,           // GATTインターフェース ※キー１
    .u16_app_id       = 0,                          // アプリケーションID   ※キー２（GATTインターフェースと1対1）
    .u16_con_id       = 0,                          // コネクションID       ※キー３
    .t_bda            = {0x00},                     // リモートデバイスアドレス
    .u16_mtu          = COM_BLE_GATT_MTU_DEFAULT,   // MTUサイズ
    .ps_rx_buff_attr  = NULL,                       // 分割受信アトリビュート
    .ps_rx_buff_data  = NULL,                       // 分割受信データ
    .ps_rx_buff       = NULL,                       // 分割書き込みデータバッファ
    .ps_next          = NULL                        // 次のステータス
};

/**
 * GATTプロファイルのサービス情報
 */
static const ts_com_ble_gatts_svc_info_t s_gatts_svc_info_default = {
    .u16_app_id     = 0,                // アプリケーションID
    .t_gatt_if      = ESP_GATT_IF_NONE, // GATTインターフェース
    .u8_svc_inst_id = 0,                // サービスインスタンスID
    .u16_num_handle = 0,                // ハンドル数
    .pu16_handles   = NULL              // ハンドルリスト
};

//==============================================================================
// GATTクライアント関係の定数定義
//==============================================================================
/** GATTクライアントイベント名 */
static const char* pc_ble_gattc_evt_str[] = {
    "ESP_GATTC_REG_EVT",                /*!< GATTクライアントの登録完了通知イベント */
    "ESP_GATTC_UNREG_EVT",              /*!< GATTクライアントの登録解除完了通知イベント */
    "ESP_GATTC_OPEN_EVT",               /*!< GATT仮想接続の接続完了通知イベント */
    "ESP_GATTC_READ_CHAR_EVT",          /*!< GATTキャラクタリスティックの読み込み完了通知イベント */
    "ESP_GATTC_WRITE_CHAR_EVT",         /*!< GATTキャラクタリスティックの書き込み完了通知イベント */
    "ESP_GATTC_CLOSE_EVT",              /*!< GATT仮想接続の切断完了通知イベント */
    "ESP_GATTC_SEARCH_CMPL_EVT",        /*!< When GATT service discovery is completed, the event comes */
    "ESP_GATTC_SEARCH_RES_EVT",         /*!< When GATT service discovery result is got, the event comes */
    "ESP_GATTC_READ_DESCR_EVT",         /*!< When GATT characteristic descriptor read completes, the event comes */
    "ESP_GATTC_WRITE_DESCR_EVT",        /*!< When GATT characteristic descriptor write completes, the event comes */
    "ESP_GATTC_NOTIFY_EVT",             /*!< When GATT notification or indication arrives, the event comes */
    "ESP_GATTC_PREP_WRITE_EVT",         /*!< When GATT prepare-write operation completes, the event comes */
    "ESP_GATTC_EXEC_EVT",               /*!< When write execution completes, the event comes */
    "ESP_GATTC_ACL_EVT",                /*!< When ACL connection is up, the event comes */
    "ESP_GATTC_CANCEL_OPEN_EVT",        /*!< When GATT client ongoing connection is cancelled, the event comes */
    "ESP_GATTC_SRVC_CHG_EVT",           /*!< When "service changed" occurs, the event comes */
    "ESP_GATTC_EVT_ERR:16",             // Event None
    "ESP_GATTC_ENC_CMPL_CB_EVT",        /*!< When encryption procedure completes, the event comes */
    "ESP_GATTC_CFG_MTU_EVT",            /*!< When configuration of MTU completes, the event comes */
    "ESP_GATTC_ADV_DATA_EVT",           /*!< When advertising of data, the event comes */
    "ESP_GATTC_MULT_ADV_ENB_EVT",       /*!< When multi-advertising is enabled, the event comes */
    "ESP_GATTC_MULT_ADV_UPD_EVT",       /*!< When multi-advertising parameters are updated, the event comes */
    "ESP_GATTC_MULT_ADV_DATA_EVT",      /*!< When multi-advertising data arrives, the event comes */
    "ESP_GATTC_MULT_ADV_DIS_EVT",       /*!< When multi-advertising is disabled, the event comes */
    "ESP_GATTC_CONGEST_EVT",            /*!< When GATT connection congestion comes, the event comes */
    "ESP_GATTC_BTH_SCAN_ENB_EVT",       /*!< When batch scan is enabled, the event comes */
    "ESP_GATTC_BTH_SCAN_CFG_EVT",       /*!< When batch scan storage is configured, the event comes */
    "ESP_GATTC_BTH_SCAN_RD_EVT",        /*!< When Batch scan read event is reported, the event comes */
    "ESP_GATTC_BTH_SCAN_THR_EVT",       /*!< When Batch scan threshold is set, the event comes */
    "ESP_GATTC_BTH_SCAN_PARAM_EVT",     /*!< When Batch scan parameters are set, the event comes */
    "ESP_GATTC_BTH_SCAN_DIS_EVT",       /*!< When Batch scan is disabled, the event comes */
    "ESP_GATTC_SCAN_FLT_CFG_EVT",       /*!< When Scan filter configuration completes, the event comes */
    "ESP_GATTC_SCAN_FLT_PARAM_EVT",     /*!< When Scan filter parameters are set, the event comes */
    "ESP_GATTC_SCAN_FLT_STATUS_EVT",    /*!< When Scan filter status is reported, the event comes */
    "ESP_GATTC_ADV_VSC_EVT",            /*!< When advertising vendor spec content event is reported, the event comes */
    "ESP_GATTC_EVT_ERR:35",             // Event None
    "ESP_GATTC_EVT_ERR:36",             // Event None
    "ESP_GATTC_EVT_ERR:37",             // Event None
    "ESP_GATTC_REG_FOR_NOTIFY_EVT",     /*!< When register for notification of a service completes, the event comes */
    "ESP_GATTC_UNREG_FOR_NOTIFY_EVT",   /*!< When unregister for notification of a service completes, the event comes */
    "ESP_GATTC_CONNECT_EVT",            /*!< 物理接続の接続完了通知イベント */
    "ESP_GATTC_DISCONNECT_EVT",         /*!< 物理接続の切断完了通知イベント */
    "ESP_GATTC_READ_MULTIPLE_EVT",      /*!< When the ble characteristic or descriptor multiple complete, the event comes */
    "ESP_GATTC_QUEUE_FULL_EVT",         /*!< When the gattc command queue full, the event comes */
    "ESP_GATTC_SET_ASSOC_EVT",          /*!< When the ble gattc set the associated address complete, the event comes */
    "ESP_GATTC_GET_ADDR_LIST_EVT",      /*!< When the ble get gattc address list in cache finish, the event comes */
    "ESP_GATTC_DIS_SRVC_CMPL_EVT",      /*!< When the ble discover service complete, the event comes */
    "ESP_GATTC_READ_MULTI_VAR_EVT",     /*!< When read multiple variable characteristic complete, the event comes */
};

// GATTクライアント設定のデフォルト値
static const ts_com_ble_gattc_if_config_t s_gattc_if_cfg_default = {
    .u16_app_id   = 0,                      // アプリケーションID
    .u8_svc_cnt   = 0,                      // サービス数
    .pt_svc_uuid  = NULL,                   // サービスのUUID
    .e_con_sec    = ESP_BLE_SEC_NO_ENCRYPT, // 接続時のセキュリティタイプ
    .fc_gattc_cb  = NULL,                   // インターフェース毎のイベントハンドラ
    .pv_app_param = NULL,                   // アプリケーションパラメータ
    .pv_usr_param = NULL,                   // ユーザー利用パラメータ
};

// GATTクライアントIFステータスのデフォルト値
static const ts_gattc_if_status_t s_gattc_if_sts_default = {
    .t_gatt_if  = ESP_GATT_IF_NONE,         // GATTインターフェース ※キー１
    .u16_app_id = 0,                        // アプリケーションID   ※キー２
    .ps_if_cfg  = NULL,                     // GATTインターフェース設定
    .b_req_cache_clear = false,             // GATTクライアントのキャッシュクリア要求フラグ
    .ps_con_sts = NULL                      // コネクションステータス
};

// GATTクライアントステータスのデフォルト値
static const ts_gattc_con_status_t s_gattc_con_sts_default = {
    .t_gatt_if       = ESP_GATT_IF_NONE,    // インターフェース ※キー１
    .u16_app_id      = 0,                   // アプリケーションID        ※キー２
    .u16_con_id      = 0,                   // コネクションID            ※キー３
    .u8_status       = 0x00,                // ステータス
    .t_bda           = {0x00},              // リモートデバイスアドレス
    .u16_mtu         = COM_BLE_GATT_MTU_DEFAULT,            // MTUサイズはデフォルト値で初期化
    .e_sec_auth_req  = ESP_GATT_AUTH_REQ_SIGNED_NO_MITM,    // セキュアアクセス権限
    .u16_svc_cnt     = 0,                   // サービス数
    .ps_svc_sts      = NULL,                // サービス情報
    .ps_next         = NULL                 // 次のステータス
};

//==============================================================================
// SPPサーバー関係の定数定義
//==============================================================================
// SPP UUID 定数
static const ts_spps_uuid_t s_spps_uuid = {
    .u16_service                 = BLE_SPPS_UUID_SERVICE,               // SPP Service
    .u16_primary_service         = ESP_GATT_UUID_PRI_SERVICE,           // プライマリサービス
    .u16_character_declaration   = ESP_GATT_UUID_CHAR_DECLARE,          // キャラクタリスティック定義
    .u16_character_client_config = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,    // クライアントキャラクタリスティック設定
    .u16_rx_data                 = BLE_SPPS_UUID_RX_DATA,               // 受信データのキャラクタリスティック
    .u16_tx_data                 = BLE_SPPS_UUID_TX_DATA,               // 送信データのキャラクタリスティック
    .u16_rx_cmd                  = BLE_SPPS_UUID_RX_CMD,                // 受信コマンドのキャラクタリスティック
    .u16_tx_sts                  = BLE_SPPS_UUID_TX_STS,                // 送信ステータスのキャラクタリスティック
};

// SPP キャラクタリスティック値とCCCD 定数
static const ts_spps_values_t s_spps_vals = {
    .u8_char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY,
    .u8_char_prop_read_write  = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ,
    .u8_char_prop_rw_auth     = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_AUTH,
    .u8_val_data_receive      = {0x00},
    .u8_val_data_notify       = {0x00},
    .u8_val_command           = {0x00},
    .u8_val_status            = {0x00},
    .u8_cccd_data_notify      = {0x00, 0x00},
    .u8_cccd_status           = {0x00, 0x00},
};

// SPPサービスUUID
static const esp_bt_uuid_t s_spp_service_uuid = {
    .len  = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = BLE_SPPS_UUID_SERVICE}
};

// GATTアトリビュートのデータベース定義（SPP）
// ┣アトリビュート制御タイプ
// ┗アトリビュートタイプ
static const esp_gatts_attr_db_t s_spp_attr_db[SPPS_ATTR_IDX_NB] = {
    //SPP -  Service Declaration
    // シリアルポートプロファイルのサービス定義
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_PRI_SERVICE（0x2800）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：値
    [SPPS_ATTR_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_primary_service, ESP_GATT_PERM_READ,
    sizeof(s_spps_uuid.u16_service), sizeof(s_spps_uuid.u16_service), (uint8_t*)&s_spps_uuid.u16_service}},

    //SPP -  data receive characteristic Declaration
    // データ受信のキャラクタリスティック定義
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_CHAR_DECLARE（0x2803）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ
    [SPPS_ATTR_IDX_RX_DATA_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_declaration, ESP_GATT_PERM_READ,
    DEF_SIZE_CHAR, DEF_SIZE_CHAR, (uint8_t*)&s_spps_vals.u8_char_prop_read_write}},

    //SPP -  data receive characteristic Value
    // 受信データのキャラクタリスティック値
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：BLE_GATT_UUID_SPP_DATA_RECEIVE（0xABF1）
    // ┣パーミッション：ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE（読み書き可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ
    [SPPS_ATTR_IDX_RX_DATA_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_rx_data, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    COM_BLE_GATT_DATA_LEN_MAX, sizeof(s_spps_vals.u8_val_data_receive), (uint8_t*)s_spps_vals.u8_val_data_receive}},

    //SPP -  data notify characteristic Declaration
    // 通知データのキャラクタリスティック定義
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：BLE_GATT_UUID_SPP_DATA_RECEIVE（0x2803）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY
    [SPPS_ATTR_IDX_TX_DATA_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_declaration, ESP_GATT_PERM_READ,
    DEF_SIZE_CHAR, DEF_SIZE_CHAR, (uint8_t*)&s_spps_vals.u8_char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    // データ通知のキャラクタリスティック値
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：BLE_SPPS_UUID_TX_DATA（0xABF2）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(512Byte)
    // ┣値最大長：エレメント値のサイズ(20Byte)
    // ┗エレメント値：配列値（20Byte）
    [SPPS_ATTR_IDX_TX_DATA_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_tx_data, ESP_GATT_PERM_READ,
    COM_BLE_GATT_DATA_LEN_MAX, sizeof(s_spps_vals.u8_val_data_notify), (uint8_t*)s_spps_vals.u8_val_data_notify}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    // データ通知のキャラクタリスティックディスクリプタ
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_CHAR_CLIENT_CONFIG（0x2902）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み書き可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(512Byte)
    // ┣値最大長：エレメント値のサイズ(20Byte)
    // ┗エレメント値：データ通知配列値（20Byte）
    [SPPS_ATTR_IDX_TX_DATA_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_client_config, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(s_spps_vals.u8_cccd_data_notify), (uint8_t*)s_spps_vals.u8_cccd_data_notify}},

    //SPP -  command characteristic Declaration
    // コマンドのキャラクタリスティック定義
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_CHAR_DECLARE（0x2803）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(512Byte)
    // ┣値最大長：エレメント値のサイズ(1Byte)
    // ┗エレメント値：ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ
    [SPPS_ATTR_IDX_RX_CMD_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_declaration, ESP_GATT_PERM_READ,
    DEF_SIZE_CHAR, DEF_SIZE_CHAR, (uint8_t*)&s_spps_vals.u8_char_prop_read_write}},

    //SPP -  command characteristic Value
    // コマンドのキャラクタリスティック値
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：BLE_SPPS_UUID_RX_CMD（0xABF3）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み書き可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(20Byte)
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：コマンド配列値
    [SPPS_ATTR_IDX_RX_CMD_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_rx_cmd, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    BLE_SPPS_CMD_MAX_LEN, sizeof(s_spps_vals.u8_val_command), (uint8_t*)s_spps_vals.u8_val_command}},

    //SPP -  status characteristic Declaration
    // ステータスのキャラクタリスティック定義
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_CHAR_DECLARE（0x2803）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(1Byte)
    // ┣値最大長：エレメント値のサイズ(1Byte)
    // ┗エレメント値：ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY
    [SPPS_ATTR_IDX_TX_STS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_declaration, ESP_GATT_PERM_READ,
    DEF_SIZE_CHAR, DEF_SIZE_CHAR, (uint8_t*)&s_spps_vals.u8_char_prop_read_notify}},

    //SPP -  status characteristic Value
    // ステータスのキャラクタリスティック値
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：BLE_SPPS_UUID_TX_STS（0xABF4）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み込み可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ
    // ┣値最大長：エレメント値のサイズ
    // ┗エレメント値：ステータス配列値
    [SPPS_ATTR_IDX_TX_STS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_tx_sts, ESP_GATT_PERM_READ,
    BLE_SPPS_STS_MAX_LEN, sizeof(s_spps_vals.u8_val_status), (uint8_t*)s_spps_vals.u8_val_status}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    // ステータスのキャラクタリスティックディスクリプタ
    // アトリビュート制御タイプ：自動返信
    // アトリビュートタイプ
    // ┣UUID長：ESP_UUID_LEN_16
    // ┣UUID値：ESP_GATT_UUID_CHAR_CLIENT_CONFIG（0x2902）
    // ┣パーミッション：ESP_GATT_PERM_READ（読み書き可能、暗号化無し、認証不要）
    // ┣値最大長：エレメント値の最大サイズ(2Byte)
    // ┣値最大長：エレメント値のサイズ(2Byte)
    // ┗エレメント値：ステータス配列値（2Byte）
    [SPPS_ATTR_IDX_TX_STS_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&s_spps_uuid.u16_character_client_config, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(s_spps_vals.u8_cccd_status), (uint8_t*)s_spps_vals.u8_cccd_status}},
};

//==============================================================================
// 共通変数定義
//==============================================================================
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

//==============================================================================
// GAPプロファイル関係の変数定義
//==============================================================================
/** GAPプロファイル制御情報 */
static ts_gap_ctrl_t s_gap_ctrl = {
    .s_config    = s_gap_cfg_default,   // GAPの設定情報
    .s_status    = s_gap_sts_default,   // GAPのステータス情報
    .u16_dev_cnt = 0,                   // GAPのデバイス情報数
    .ps_device   = NULL                 // GAPのデバイス情報
};

//==============================================================================
// GATTサーバー関係の変数定義
//==============================================================================
// GATTサーバー制御情報
static ts_gatts_ctrl_t s_gatts_ctrl = {
    // GATTサーバーステータス
    .u8_status = 0,
    // GATTサーバーのインターフェースステータス
    .ps_if_status = NULL,
};

//==============================================================================
// GATTクライアント関係の変数定義
//==============================================================================
// GATTクライアント制御情報
static ts_gattc_ctrl_t s_gattc_ctrl = {
    // GATTクライアントアプリケーション情報の数
    .u16_if_count = 0,
    // GATTクライアントアプリケーション設定
    .ps_if_config = NULL,
    // GATTクライアントアプリケーションステータス
    .ps_if_status = NULL,
};

//==============================================================================
// SPPサーバー関係の変数定義
//==============================================================================
// SPPサーバーのコネクションステータス
static ts_spps_status_t* ps_spps_status = NULL;

//==============================================================================
// SPPクライアント関係の変数定義
//==============================================================================
// SPPクライアントのコネクションステータス
static ts_sppc_status_t* ps_sppc_status = NULL;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
//==============================================================================
// 共通系
//==============================================================================

//==============================================================================
// BLE共通関数
//==============================================================================
/** 物理接続の切断 */
static esp_err_t sts_com_disconnect(esp_bd_addr_t t_bda);

//==============================================================================
// GAP関連のローカル関数
//==============================================================================
/** GAPプロファイルのイベントハンドラ */
static void v_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t *pu_param);

/** GAPプロファイルのデバイス情報追加 */
static ts_gap_device_t* ps_gap_add_device(esp_bd_addr_t t_bda);
/** GAPプロファイルのデバイス情報検索 */
static ts_gap_device_t* ps_gap_get_device(esp_bd_addr_t t_bda);
/** GAPプロファイルのデバイス情報生成 */
static ts_gap_device_t* ps_gap_create_device(struct ble_scan_result_evt_param* ps_param);
/** GAPプロファイルのデバイス情報削除 */
static esp_err_t sts_gap_del_device(esp_bd_addr_t t_bda);
/** GAPプロファイルのデバイス情報リフレッシュ処理 */
static void v_gap_minimize_device_list();
/** GAPプロファイルのアドバタイズ開始処理 */
static esp_err_t sts_gap_start_advertise(esp_ble_adv_params_t* ps_adv_params);
/** GAPプロファイルのアドバタイズ開始処理（ローカルプライバシーモード設定） */
static esp_err_t sts_gap_start_advertise_step_0();
/** GAPプロファイルのアドバタイズ開始処理（パラメータ設定） */
static esp_err_t sts_gap_start_advertise_step_1();
/** GAPプロファイルのアドバタイズ開始処理（アドバタイズ開始） */
static esp_err_t sts_gap_start_advertise_step_2();
/** GAPプロファイルのスキャン開始処理 */
static esp_err_t sts_gap_start_scan(uint32_t u32_duration);
/** GAPプロファイルのスキャン開始処理（パラメータ設定） */
static esp_err_t sts_gap_start_scan_step_1();
/** GAPプロファイルのスキャン開始処理（スキャン開始） */
static esp_err_t sts_gap_start_scan_step_2();
/** GAPプロファイルのスキャンステータスの更新処理（タイムアウト判定等） */
static esp_err_t sts_gap_update_scan_status();
/** GAPプロファイルのローカルプライバシー機能の設定処理 */
static esp_err_t sts_gap_config_local_privacy(esp_ble_addr_type_t e_addr_type);

//==============================================================================
// GATTサーバー関連の関数
//==============================================================================
/** GATTサーバーのインターフェースステータス取得処理 */
static ts_gatts_if_status_t* ps_gatts_get_if_status(esp_gatt_if_t t_gatt_if);

/** GATTサーバーのサービスステータス取得処理 */
static ts_gatts_svc_status_t* ps_gatts_get_svc_status(ts_gatts_if_status_t* ps_if_sts, uint8_t u8_svc_inst_id);

/** GATTサーバーのコネクションステータス追加処理 */
static ts_gatts_con_status_t* ps_gatts_add_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id);

/** GATTサーバーのコネクションステータス取得処理 */
static ts_gatts_con_status_t* ps_gatts_get_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id);

/** GATTプロファイルのアトリビュートハンドルインデックス取得処理 */
static esp_err_t sts_gatts_get_handle_idx(ts_gatts_if_status_t* ps_if_sts,
                                           uint16_t u16_handle,
                                           uint8_t* pu8_svc_idx,
                                           uint16_t* pu16_hndl_idx);

/** GATTプロファイルのアトリビュート取得処理 */
static esp_gatts_attr_db_t* ps_gatts_get_attribute(ts_gatts_if_status_t* ps_if_sts,
                                                    uint8_t u8_svc_idx,
                                                    uint16_t pu16_hndl_idx);

/** GATTプロファイルのハンドルに対応したアトリビュート取得処理 */
static esp_gatts_attr_db_t* ps_gatts_get_handle_attribute(ts_gatts_if_status_t* ps_if_sts,
                                                           uint16_t u16_handle,
                                                           uint8_t* pu8_svc_idx,
                                                           uint16_t* pu16_hndl_idx);

/** GATTクライアントのコネクションステータス削除処理 */
static void v_gatts_del_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id);

/** GATTクライアントのアトリビュート値の書き込み処理 */
static esp_err_t sts_gatts_write_attr_value(esp_gatts_attr_db_t* ps_attr, ts_com_ble_gatt_rx_data_t* ps_param);

/** GATTプロファイルのIndicateもしくはNotify処理 */
static esp_err_t sts_gatts_indication(esp_gatt_if_t t_gatt_if,
                                       uint8_t u8_svc_idx,
                                       uint16_t u16_handle,
                                       uint8_t* pu8_data,
                                       uint16_t u16_data_len,
                                       bool b_need_confirm);

/** GATTサーバーの共通イベントハンドラ */
static void v_gatts_evt_com_cb(esp_gatts_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gatts_cb_param_t* pu_param);

/** GATTサーバーのダミーイベントハンドラ */
static void v_gatts_evt_dmy_cb(esp_gatts_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gatts_cb_param_t* pu_param);

/** GATTプロファイルのイベント処理：インターフェース登録 */
static esp_err_t sts_gatts_evt_register(esp_gatt_if_t t_gatt_if,
                                         esp_ble_gatts_cb_param_t* pu_param);

/** GATTプロファイルのイベント処理：書き込みデータ受信 */
static esp_err_t sts_gatts_evt_write(ts_gatts_if_status_t* ps_if_sts,
                                      struct gatts_write_evt_param* ps_param);

/** GATTプロファイルのイベント処理：分割書き込み完了通知イベント */
static esp_err_t sts_gatts_evt_exec_write(ts_gatts_if_status_t* ps_if_sts,
                                           struct gatts_exec_write_evt_param* ps_param);
/** GATTプロファイルのイベント処理：インターフェース登録解除 */
static esp_err_t sts_gatts_evt_unregist(ts_gatts_if_status_t* ps_tgt_sts,
                                         ts_gatts_if_status_t* ps_bef_sts);

//==============================================================================
// GATTクライアント関連の関数
//==============================================================================
/** GATTプロファイルのサーバーへの接続処理 */
static esp_err_t sts_gattc_open(esp_gatt_if_t t_gatt_if,
                                 esp_bd_addr_t t_bda,
                                 esp_ble_addr_type_t e_addr_type,
                                 bool b_direct);

/** GATTプロファイルのサーバーとの切断処理 */
static esp_err_t sts_gattc_close(esp_gatt_if_t t_gatt_if,
                                 esp_bd_addr_t t_bda);
/** GATTクライアントのIFステータス取得処理 */
static ts_gattc_if_status_t* ps_gattc_get_if_status(esp_gatt_if_t t_gatt_if);

/** GATTクライアントのコネクションステータス追加取得処理 */
static ts_gattc_con_status_t* ps_gattc_add_con_status(ts_gattc_if_status_t* ps_if_sts, esp_bd_addr_t t_bda);

/** GATTクライアントのアドレスに対応したコネクションステータス取得処理 */
static ts_gattc_con_status_t* ps_gattc_get_con_status_bda(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda);

/** GATTクライアントのオープンたコネクションステータス取得処理 */
static ts_gattc_con_status_t* ps_gattc_get_con_status_id(ts_gattc_if_status_t* ps_if_sts, uint16_t u16_con_id);

/** GATTクライアントのコネクションステータス削除処理 */
static void v_gattc_del_con_status(ts_gattc_if_status_t* ps_if_sts, esp_bd_addr_t t_bda);

/** GATTクライアントのサービス検索処理 */
static esp_err_t sts_gattc_search_service(esp_bd_addr_t t_bda);

/** GATTクライアントのサービスステータス取得追加処理 */
static ts_gattc_svc_status_t* ps_gattc_add_svc_status(ts_gattc_if_status_t* ps_if_sts, uint16_t u16_con_id, esp_gatt_id_t* ps_svc_id);

/** GATTクライアントのサービスステータス取得処理 */
static ts_gattc_svc_status_t* ps_gattc_get_svc_status(ts_gattc_if_status_t* ps_if_sts, uint16_t u16_con_id, esp_gatt_id_t s_svc_id);

/** GATTクライアントのアトリビュートハンドルに対応したサービスステータス取得処理 */
static ts_gattc_svc_status_t* ps_gattc_get_handle_svc_status(ts_gattc_if_status_t* ps_if_sts, uint16_t u16_handle);

/** GATTサーバーからのアトリビュートDBの取得処理 */
static esp_err_t sts_gattc_get_db(ts_gattc_svc_status_t* ps_service);

/** GATTサーバからのNotify対象としてGATTクライアントを登録する処理 */
static esp_err_t sts_gattc_register_for_notify(ts_gattc_svc_status_t* ps_svc_sts);

/** GATTサーバーへの有効でセキュアなアクセス権限の取得処理 */
static esp_gatt_auth_req_t e_gattc_get_auth_req(esp_ble_auth_req_t t_auth_req);

/** GATTサーバーへのDescriptor書き込み処理 */
static esp_err_t sts_gattc_write_cccd(ts_gattc_svc_status_t* ps_service,
                                       uint16_t u16_char_handle,
                                       uint8_t u8_value,
                                       esp_gatt_write_type_t e_write_type,
                                       esp_gatt_auth_req_t e_auth_req);

/** GATTクライアントの共通イベントハンドラ */
static void v_gattc_evt_com_cb(esp_gattc_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gattc_cb_param_t* pu_param);

/** GATTクライアントのダミーイベントハンドラ */
static void v_gattc_evt_dmy_cb(esp_gattc_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gattc_cb_param_t* pu_param);

/** GATTクライアントのESP_GATTC_REG_EVTイベント処理 */
static esp_err_t sts_gattc_evt_register(esp_gatt_if_t t_gatt_if,
                                         esp_ble_gattc_cb_param_t* pu_param);

/** GATTクライアントのESP_GATTC_OPEN_EVTイベント処理 */
static esp_err_t sts_gattc_evt_open(ts_gattc_if_status_t* ps_if_sts,
                                     struct gattc_open_evt_param* pu_param);

/** GATTクライアントのESP_GATTC_READ_CHAR_EVTイベント処理 */
static esp_err_t sts_gattc_evt_read(esp_gattc_cb_event_t e_event,
                                     ts_gattc_if_status_t* ps_if_sts,
                                     struct gattc_read_char_evt_param* pu_param);

/** GATTクライアントのESP_GATTC_SEARCH_RES_EVTイベント処理 */
static esp_err_t sts_gattc_evt_search_result(ts_gattc_if_status_t* ps_if_sts,
                                              struct gattc_search_res_evt_param* pu_param);

/** GATTクライアントのESP_GATTC_NOTIFY_EVTイベント処理 */
static esp_err_t sts_gattc_evt_notify(ts_gattc_if_status_t* ps_if_sts,
                                       struct gattc_notify_evt_param* pu_param);

//==============================================================================
// SPPサーバー関連の関数
//==============================================================================
/** GATTサーバーのSPPインターフェースのイベントハンドラ */
static void v_spps_evt_cb(esp_gatts_cb_event_t e_event,
                           esp_gatt_if_t t_gatt_if,
                           esp_ble_gatts_cb_param_t* pu_param);
/** GATTサーバーのSPPインターフェースのステータス取得処理 */
static ts_spps_status_t* ps_spps_get_status(esp_gatt_if_t t_gatt_if, uint8_t u8_svc_idx);

//==============================================================================
// SPPサーバー関連の変数
//==============================================================================
// SPPサーバーのユーザーコールバック関数
static esp_gatts_cb_t fc_spps_usr_evt_cb = v_gatts_evt_dmy_cb;

//==============================================================================
// SPPクライアント関連の関数
//==============================================================================
/** SPPクライアントのインターフェース毎のイベントハンドラ */
static void v_sppc_evt_cb(esp_gattc_cb_event_t e_event,
                           esp_gatt_if_t t_gatt_if,
                           esp_ble_gattc_cb_param_t* pu_param);
/** SPPクライアントのコネクションステータス追加検索処理 */
static ts_sppc_status_t* ps_sppc_add_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id);
/** SPPクライアントのコネクションステータス検索処理 */
static ts_sppc_status_t* ps_sppc_get_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id);
/** SPPクライアントのコネクションステータス削除処理 */
static void v_sppc_del_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id);
/** SPP接続ステータスの取得処理 */
static te_com_ble_spp_connection_sts_t e_sppc_con_sts(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id);
/** SPPクライアントのサービス判定処理 */
static bool b_sppc_chk_status(ts_gattc_svc_status_t* ps_svc_sts);

//==============================================================================
// SPPクライアント関連の変数
//==============================================================================
// SPPクライアントのユーザーコールバック関数
static esp_gattc_cb_t fc_sppc_usr_evt_cb = v_gattc_evt_dmy_cb;

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_com_ble_address_to_str
 *
 * DESCRIPTION:BLEアドレス文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 * c_com_ble_bda_string_t   tc_addr     W   BLEアドレス文字列
 * esp_bd_addr_t            t_bda       R   BLEアドレス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_address_to_str(tc_com_ble_bda_string_t tc_addr, esp_bd_addr_t t_bda) {
    if (tc_addr == NULL || t_bda == NULL) {
        return;
    }
    sprintf(tc_addr, "%02X:%02X:%02X:%02X:%02X:%02X", t_bda[0], t_bda[1], t_bda[2], t_bda[3], t_bda[4], t_bda[5]);
}

/*******************************************************************************
 *
 * NAME: pc_com_ble_key_type_to_str
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
const char* pc_com_ble_key_type_to_str(esp_ble_key_type_t t_key_type) {
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
 * NAME: pc_com_ble_auth_req_to_str
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
const char* pc_com_ble_auth_req_to_str(esp_ble_auth_req_t t_auth_req) {
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
 * NAME: pc_com_ble_gatts_event_to_str
 *
 * DESCRIPTION:BLEのGATTサーバーイベント文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_gap_ble_cb_event_t e_event     R   GAPイベント
 *
 * RETURNS:
 *   const char*:キータイプ文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_com_ble_gatts_event_to_str(esp_gatts_cb_event_t e_event) {
    // 入力チェック
    if (e_event < 0 || e_event > ESP_GATTS_SEND_SERVICE_CHANGE_EVT) {
        return "ESP_GATTS_EVT_ERR";
    }
    return pc_ble_gatts_evt_str[e_event];
}

/*******************************************************************************
 *
 * NAME: pc_com_ble_gattc_event_to_str
 *
 * DESCRIPTION:BLEのGATTクライアントイベント文字列の取得
 *
 * PARAMETERS:              Name        RW  Usage
 *   esp_gattc_cb_event_t   e_event     R   GAPイベント
 *
 * RETURNS:
 *   const char*:キータイプ文字列
 *
 * NOTES:
 * None.
 ******************************************************************************/
const char* pc_com_ble_gattc_event_to_str(esp_gattc_cb_event_t e_event) {
    // 入力チェック
    if (e_event < 0 || e_event > ESP_GATTC_READ_MULTI_VAR_EVT) {
        return "ESP_GATTC_EVT_ERR";
    }
    return pc_ble_gattc_evt_str[e_event];
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_display_bonded_devices
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
esp_err_t sts_com_ble_display_bonded_devices() {
    // ボンディング成功デバイス数
    int i_dev_num = esp_ble_get_bond_device_num();
    if (i_dev_num == 0) {
#ifdef COM_BLE_DEBUG
        ESP_LOGI(LOG_TAG, "No bonded devices");
#endif
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
#ifdef COM_BLE_DEBUG
    uint8_t* pu8_addr;
    int i_idx;
    for (i_idx = 0; i_idx < i_dev_num; i_idx++) {
        pu8_addr = (uint8_t*)dev_list[i_idx].bd_addr;
        ESP_LOGI(LOG_TAG, "Bond Device Address  = %02x:%02x:%02x:%02x:%02x:%02x",
                pu8_addr[0], pu8_addr[1], pu8_addr[2], pu8_addr[3], pu8_addr[4], pu8_addr[5]);
        ESP_LOGI(LOG_TAG, "Bond Device Key Mask = %02x", dev_list[i_idx].bond_key.key_mask);
    }
#endif
    // 表示成功
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_addr_cpy
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
void v_com_ble_addr_cpy(esp_bd_addr_t t_to_bda, const esp_bd_addr_t t_from_bda) {
    t_to_bda[0] = t_from_bda[0];
    t_to_bda[1] = t_from_bda[1];
    t_to_bda[2] = t_from_bda[2];
    t_to_bda[3] = t_from_bda[3];
    t_to_bda[4] = t_from_bda[4];
    t_to_bda[5] = t_from_bda[5];
}

/*******************************************************************************
 *
 * NAME: l_com_ble_addr_cmp
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
long l_com_ble_addr_cmp(const esp_bd_addr_t t_bda1, const esp_bd_addr_t t_bda2) {
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
 * NAME: b_com_ble_id_equal
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
bool b_com_ble_id_equal(esp_gatt_id_t* ps_id1, esp_gatt_id_t* ps_id2) {
    if (ps_id1->inst_id == ps_id2->inst_id) {
        return b_com_ble_uuid_equal(&ps_id1->uuid, &ps_id2->uuid);
    }
    // 結果不一致
    return false;
}

/*******************************************************************************
 *
 * NAME: b_com_ble_uuid_equal
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
bool b_com_ble_uuid_equal(esp_bt_uuid_t* ps_uuid1, esp_bt_uuid_t* ps_uuid2) {
    if (ps_uuid1->len == ps_uuid2->len) {
        // アドレス比較
        return (memcmp(ps_uuid1->uuid.uuid128, ps_uuid2->uuid.uuid128, ps_uuid1->len) == 0);
    }
    // 不一致
    return false;
}

/*******************************************************************************
 *
 * NAME: b_com_ble_edit_base_uuid
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
bool b_com_ble_edit_base_uuid(uint8_t* pu8_uuid) {
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
 * NAME: sts_ble_init
 *
 * DESCRIPTION:BLEの初期化処理
 *   Bluetoothコントローラの初期化を行い、Bluetoothを有効化する。
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_init() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    // クリティカルセクション
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Bluetooth LE 初期処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // BLEステータスを判定
        esp_bt_controller_status_t e_bt_sts = esp_bt_controller_get_status();
        if (e_bt_sts == ESP_BT_CONTROLLER_STATUS_ENABLED) {
            e_bt_sts = ESP_ERR_INVALID_STATE;
            break;
        }
        // BluetoothコントローラのBSS、データ、およびその他の領域を解放
        // BSSセクション：初期値なし（0もしくはNULL）のグローバル変数もしくはstatic変数を格納するセクション
        sts_val = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (sts_val != ESP_OK) {
            break;
        }
        // Bluetoothコントローラーのデフォルト設定の構造体を生成
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        // Bluetoothコントローラを初期化して、タスクその他のリソースを割り当てる
        sts_val = esp_bt_controller_init(&bt_cfg);
        if (sts_val != ESP_OK) {
            break;
        }
        // 引数で指定したモードでBluetoothコントローラを有効にする
        sts_val = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (sts_val != ESP_OK) {
            break;
        }
        // Bluetoothの初期化処理
        sts_val = esp_bluedroid_init();
        if (sts_val != ESP_OK) {
            break;
        }
        // Bluetoothを有効にする
        sts_val = esp_bluedroid_enable();
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_disconnect
 *
 * DESCRIPTION:BLEの切断処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_bd_addr_t        t_bda       R   対象デバイスのBLEアドレス
 *
 * RETURNS:
 *   ESP_OK:切断成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_disconnect(esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 切断処理
    //==========================================================================
    esp_err_t sts_val = sts_com_disconnect(t_bda);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_bonded_dev
 *
 * DESCRIPTION:BLEのボンディング済みデバイス判定処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_bd_addr_t        t_bda       R   対象デバイスのBLEアドレス
 *
 * RETURNS:
 *   ESP_OK:対象デバイスはボンディング済み
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_bonded_dev(esp_bd_addr_t t_bda) {
    // ボンディング済みデバイスリストを取得
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    ts_com_ble_bond_dev_list_t* ps_bond_list = ps_com_ble_bond_dev_list();
    if (ps_bond_list == NULL) {
        return sts_val;
    }
    // デバイス探索
    int i_idx;
    esp_ble_bond_dev_t* ps_dev_list  = ps_bond_list->ps_dev_list;
    for (i_idx = 0; i_idx < ps_bond_list->i_device_cnt; i_idx++) {
        if (l_com_ble_addr_cmp(t_bda, ps_dev_list[i_idx].bd_addr) == 0l) {
            // 対象デバイスはボンディング済み
            sts_val = ESP_OK;
            break;
        }
    }
    // ボンディング済みデバイスリストの削除処理
    v_com_ble_delete_bond_dev_list(ps_bond_list);
    ps_bond_list = NULL;
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_bond_dev_list
 *
 * DESCRIPTION:BLEのボンディング済みデバイスリスト取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   ts_com_ble_bond_dev_list_t:ボンディングリスト
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_bond_dev_list_t* ps_com_ble_bond_dev_list() {
    // ボンディング済みデバイス数
    int i_bond_cnt = esp_ble_get_bond_device_num();
    if (i_bond_cnt <= 0) {
        return NULL;
    }
    // ボンディングデバイスリストを動的に確保
    ts_com_ble_bond_dev_list_t* ps_dev_list = pv_mem_malloc(sizeof(ts_com_ble_bond_dev_list_t));
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
        v_com_ble_delete_bond_dev_list(ps_dev_list);
    }
    // 結果返却
    return ps_dev_list;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_delete_bond_dev_list
 *
 * DESCRIPTION:BLEのボンディング済みデバイスリストの削除処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_delete_bond_dev_list(ts_com_ble_bond_dev_list_t* ps_dev_list) {
    if (ps_dev_list == NULL) {
        return;
    }
    // メモリを解放
    l_mem_free(ps_dev_list->ps_dev_list);
    l_mem_free(ps_dev_list);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_disbonding_all
 *
 * DESCRIPTION:BLEのボンディング済みデバイスの全削除処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   ESP_OK:削除成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_disbonding_all() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // ボンディングデバイスの削除
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // ボンディング成功デバイス数
        int dev_num = esp_ble_get_bond_device_num();
        if (dev_num == 0) {
            break;
        }
        // メモリ確保
        esp_ble_bond_dev_t dev_list[dev_num];
        // デバイスリスト取得
        sts_val = esp_ble_get_bond_device_list(&dev_num, dev_list);
        if (sts_val != ESP_OK) {
            break;
        }
        // ボンディングデバイスの削除処理
        int i_idx;
        for (i_idx = 0; i_idx < dev_num; i_idx++) {
            // コネクションの切断処理
            sts_com_disconnect(dev_list[i_idx].bd_addr);
            // ボンディングデバイスの削除処理
            sts_val = esp_ble_remove_bond_device(dev_list[i_idx].bd_addr);
            if (sts_val != ESP_OK) {
                break;
            }
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_disbonding
 *
 * DESCRIPTION:BLEのボンディング済みデバイス削除処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_bd_addr_t        t_bda       R   対象デバイスのBLEアドレス
 *
 * RETURNS:
 *   ESP_OK:削除成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_disbonding(esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // ボンディング済みデバイスの削除
    //==========================================================================
    // コネクションの切断処理
    sts_com_disconnect(t_bda);
    // ボンディングデバイスの削除
    esp_err_t sts_val = esp_ble_remove_bond_device(t_bda);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_smp_adv_init
 *
 * DESCRIPTION:BLEのGAP・SMP情報のアドバタイザ側の初期設定処理
 *
 * PARAMETERS:                  Name        RW  Usage
 *   ts_com_ble_gap_adv_cfg     s_cfg       R   Advertise設定
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_smp_init(ts_com_ble_gap_config_t s_cfg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    // クリティカルセクション
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // パラメータ設定
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        // GAPプロファイル（機器の接続・認証・暗号化を行う）のイベントコールバック関数の登録
        sts_val = esp_ble_gap_register_callback(v_gap_event_cb);
        if (sts_val != ESP_OK) break;
        // BLEのGAPプロファイルにおいての認証モード設定処理
        // BTM_LE_AUTH_REQ_MITM：認証時のMITM（Man In The Middle：中間者攻撃）プロテクションを有効化
        // BTM_LE_AUTH_REQ_SC_ONLY：セキュアコネクションのみ有効
        // BTM_LE_AUTH_BOND：認証後にデバイスとボンディング（ペアリングで交換した鍵を保存すること）
        // ESP_LE_AUTH_REQ_SC_MITM_BOND = (ESP_LE_AUTH_REQ_MITM | ESP_LE_AUTH_REQ_SC_ONLY | ESP_LE_AUTH_BOND)
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &s_cfg.t_auth_req, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        // BLEのGAPプロファイルにおいて相手に通知されるデバイスのＩＯ機能の設定処理
        // ESP_IO_CAP_OUT   ：6桁の数値を表示可能なディスプレイ装置を持つ(Input装置なし)
        // ESP_IO_CAP_IO    ：6桁の数値を表示可能なディスプレイ装置と"Yes"/"No"の確認入力機能を持つ
        // ESP_IO_CAP_IN    ：’0'～'9'の数値の入力機能を持つ（Output装置なし）
        // ESP_IO_CAP_NONE  ：一切のInput装置、Output装置を持たない
        // ESP_IO_CAP_KBDISP：’0'～'9'の数値の入力機能と6桁の数値を表示可能なディスプレイ装置を持つ
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &s_cfg.t_iocap, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        // BLEのGAPプロファイルにおけるペアリング時のOut-of-band設定処理
        // デフォルトで無効化する
        uint8_t u8_oob_support = ESP_BLE_OOB_DISABLE;
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &u8_oob_support, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        // マスターの場合には、スレーブからどのような応答キーを期待しているのかと
        // どのようなキーをスレーブ側に配布可能かを設定します。
        // スレーブの場合には、マスターからどのタイプのキーの配布を期待しているのかと
        // マスターへの応答の際に、どの様なキーをマスターに返信出来るのかを設定します。
        // ESP_BLE_ENC_KEY_MASK：暗号化されたキーの交換をする
        // ESP_BLE_ID_KEY_MASK：IRK（Bluetoothアドレスを隠蔽するプライバシー機能に利用される鍵）を交換する
        // ESP_BLE_CSR_KEY_MASK：暗号化されていないデータに電子署名する為に使われるCSRKキーを交換する
        // ESP_BLE_LINK_KEY_MASK：初期化キーと応答キーでリンクキー（BLEとBR / EDRの共存モードで使用されているキー）を交換する
        //  初期キーの設定
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &s_cfg.u8_init_key, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        //  応答キーの設定
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &s_cfg.u8_rsp_key, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        //  最大キーサイズの設定
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &s_cfg.u8_max_key_size, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        // BLEのGAPプロファイルにおけるペアリング時の受け入れ権限の設定処理
        // 指定された権限だけを受け入れるか設定する
        sts_val = esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &s_cfg.u8_auth_option, sizeof(uint8_t));
        if (sts_val != ESP_OK) break;
        // GAPデバイス名の設定
        sts_val = esp_ble_gap_set_device_name(s_cfg.pc_device_name);
        if (sts_val != ESP_OK) break;
        // 既存のGAPデバイス名が有る場合には解放する
        if (s_gap_ctrl.s_config.pc_device_name != NULL) {
            l_mem_free(s_gap_ctrl.s_config.pc_device_name);
            s_gap_ctrl.s_config.pc_device_name = NULL;
        }
        // 制御情報をコピー
        s_gap_ctrl.s_config = s_cfg;
        // デバイス名のコピー
        if (s_cfg.pc_device_name != NULL) {
            s_gap_ctrl.s_config.pc_device_name = pv_mem_malloc(strlen(s_cfg.pc_device_name) + 1);
            strcpy(s_gap_ctrl.s_config.pc_device_name, s_cfg.pc_device_name);
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_passkey_reply
 *
 * DESCRIPTION:BLEのパスキー応答処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_bd_addr_t    t_bda           R   応答先アドレス
 * bool             b_accept        R   パスキーのエントリ結果
 * uint32_t         u32_passkey     R   パスキー
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_passkey_reply(esp_bd_addr_t t_bda, bool b_accept, uint32_t u32_passkey) {
    // 入力チェック
    if (u32_passkey > 999999) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // パスキー応答
    //==========================================================================
    // リプライ判定
    esp_err_t sts_val = ESP_ERR_INVALID_STATE;
    // GAPデバイスステータス
    ts_gap_device_t* ps_device = ps_gap_get_device(t_bda);
    if (ps_device != NULL) {
        // ステータス判定
        if ((ps_device->u16_status & GAP_DEV_STS_REQ_PASSKEY) != 0x00 &&
            (ps_device->u16_status & GAP_DEV_STS_RPY_PASSKEY) == 0x00) {
            // パスキーのリプライ送信
            sts_val = esp_ble_passkey_reply(t_bda, b_accept, u32_passkey);
            if (sts_val == ESP_OK) {
                // ステータス更新
                ps_device->u16_status |= GAP_DEV_STS_RPY_PASSKEY;
            }
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_confirm_reply
 *
 * DESCRIPTION:BLEの番号確認応答処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   応答先アドレス
 * bool             b_accept    R   PIN確認結果
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_confirm_reply(esp_bd_addr_t t_bda, bool b_accept) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 番号確認応答
    //==========================================================================
    // リプライ判定
    esp_err_t sts_val = ESP_ERR_INVALID_STATE;
    // GAPデバイスステータス
    ts_gap_device_t* ps_device = ps_gap_get_device(t_bda);
    // ステータス判定
    if ((ps_device->u16_status & GAP_DEV_STS_REQ_NUM_CHK) != 0x00 &&
        (ps_device->u16_status & GAP_DEV_STS_RPY_NUM_CHK) == 0x00) {
        // 番号の確認結果を応答
        sts_val = esp_ble_confirm_reply(t_bda, b_accept);
        if (sts_val == ESP_OK) {
            // 制御ステータスを更新
            ps_device->u16_status |= GAP_DEV_STS_RPY_NUM_CHK;
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_set_static_pass_key
 *
 * DESCRIPTION:BLEのSMPにおけるクリアPINコードの設定処理
 *
 * PARAMETERS:        Name                 RW  Usage
 *   uint32_t         u32_static_passkey   R   PINコード
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_set_static_pass_key(uint32_t u32_static_passkey) {
    // BLEのGAPプロファイルにおけるPINコードの設定処理
    // ペアリングの際のPINコードは数字６桁の固定値、型はuint32_t
    return esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &u32_static_passkey, sizeof(uint32_t));
}

/*******************************************************************************
 *
 * NAME: i_com_ble_gap_read_rssi
 *
 * DESCRIPTION:BLEのRSSIの読み取り処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   取得先アドレス
 *
 * RETURNS:
 *   int:RSSI値（-127～20、読み取れない場合は127）
 *
 * NOTES:
 * None.
 ******************************************************************************/
int i_com_ble_gap_read_rssi(esp_bd_addr_t t_bda) {
    // RSSI値
    int i_rssi = 127;

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return i_rssi;
    }

    //==========================================================================
    // RSSIの読み取り要求
    //==========================================================================
    // デバイス情報
    ts_gap_device_t* ps_device = NULL;
    do {
        // デバイス情報の取得
        ps_device = ps_gap_get_device(t_bda);
        if (ps_device == NULL) {
            break;
        }
        // 読み取り要求
        if (esp_ble_gap_read_rssi(t_bda) != ESP_OK) {
            ps_device = NULL;
            break;
        }
        // RSSIの要求実行中に更新
        ps_device->u16_status |= GAP_DEV_STS_EXEC_RSSI;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 要求結果判定
    //==========================================================================
    if (ps_device == NULL) {
        // 取得不能値の返却
        return i_rssi;
    }

    //==========================================================================
    // 読み取り完了待ち
    //==========================================================================
    // RSSI受信待ち
    uint16_t u16_status = 0x00;
    int i_cnt;
    for (i_cnt = 0; (i_cnt < BLE_UTIL_RETRY_CNT) && ((u16_status & GAP_DEV_STS_EXEC_RSSI) == 0x00); i_cnt++) {
        //----------------------------------------------------------------------
        // 待ち
        //----------------------------------------------------------------------
        vTaskDelay(GAP_DEVICE_STS_UPD_WAIT_TICK);

        //----------------------------------------------------------------------
        // クリティカルセクション
        //----------------------------------------------------------------------
        if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
            return i_rssi;
        }

        //----------------------------------------------------------------------
        // RSSI取得
        //----------------------------------------------------------------------
        // デバイス情報の取得
        ps_device = ps_gap_get_device(t_bda);
        if (ps_device != NULL) {
            // ステータス取得
            u16_status = ps_device->u16_status;
            // RSSI取得
            i_rssi = ps_device->i_rssi;
        }

        //----------------------------------------------------------------------
        // クリティカルセクション終了
        //----------------------------------------------------------------------
        xSemaphoreGiveRecursive(s_mutex);
    }

    // RSSI返却
    return i_rssi;
}

/*******************************************************************************
 *
 * NAME: e_com_ble_gap_device_sts
 *
 * DESCRIPTION:BLEデバイスのステータス読み取り処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   取得先アドレス
 *
 * RETURNS:
 *   te_gap_dev_sts_t:デバイスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_gap_dev_sts_t e_com_ble_gap_device_sts(esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return GAP_DEV_STS_DEVICE_NONE;
    }

    //==========================================================================
    // デバイスステータスの取得
    //==========================================================================
    // デバイスステータス
    te_gap_dev_sts_t e_device_sts = GAP_DEV_STS_DEVICE_NONE;
    // 対象デバイス情報取得
    ts_gap_device_t* ps_device = ps_gap_get_device(t_bda);
    if (ps_device != NULL) {
        // デバイスステータスを設定
        e_device_sts = ps_device->u16_status;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // GAPデバイスステータス返信
    return e_device_sts;
}

/*******************************************************************************
 *
 * NAME: e_com_ble_gap_device_sts_wait
 *
 * DESCRIPTION:BLEデバイスのステータス更新待ち処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   取得先アドレス
 * te_gap_dev_sts_t e_chk_sts   R   GAPデバイスステータス（論理和可能）
 * TickType_t       t_max_wait  R   最大待ち時間
 *
 * RETURNS:
 *   te_gap_dev_sts_t:デバイスステータス
 *
 * NOTES:
 *   指定されたGAPデバイスステータスと部分一致するまでウェイト
 * None.
 ******************************************************************************/
te_gap_dev_sts_t e_com_ble_gap_device_sts_wait(esp_bd_addr_t t_bda,
                                               te_gap_dev_sts_t e_chk_sts,
                                               TickType_t t_max_wait) {
    // タイムアウト時刻の取得
    TickType_t t_timeout = xTaskGetTickCount() + t_max_wait;
    // GAPデバイスステータス
    te_gap_dev_sts_t e_sts;
    // 判定ステータスチェック
    if (e_chk_sts == GAP_DEV_STS_DEVICE_NONE) {
        // ステータス完全一致
        do {
            // GAPデバイスステータス取得
            e_sts = e_com_ble_gap_device_sts(t_bda);
            // ステータスチェック
            if (e_sts == GAP_DEV_STS_DEVICE_NONE) {
                break;
            }
            // ウェイト
            vTaskDelay(GAP_DEVICE_STS_UPD_WAIT_TICK);
        } while (t_timeout >= xTaskGetTickCount());
    } else {
        // ステータス部分一致
        do {
            // GAPデバイスステータス取得
            e_sts = e_com_ble_gap_device_sts(t_bda);
            // ステータスチェック
            if ((e_sts & e_chk_sts) != GAP_DEV_STS_DEVICE_NONE) {
                break;
            }
            // ウェイト
            vTaskDelay(GAP_DEVICE_STS_UPD_WAIT_TICK);
        } while (t_timeout >= xTaskGetTickCount());
    }
    // 結果ステータス返却
    return e_sts;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gap_create_device_list
 *
 * DESCRIPTION:BLEのアドレス把握済みのデバイスリストの生成処理
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   ts_com_ble_gap_device_list_t*:1件も無い場合にはNULLを返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gap_device_list_t* ps_com_ble_gap_create_device_list() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // BLEのアドレス把握済みのデバイスリスト
    //==========================================================================
    ts_com_ble_gap_device_list_t* ps_list = NULL;
    do {
        // デバイス数判定
        if (s_gap_ctrl.u16_dev_cnt == 0) {
            break;
        }
        // デバイスリスト生成
        ps_list = pv_mem_malloc(sizeof(ts_com_ble_gap_device_list_t));
        if (ps_list == NULL) {
            break;
        }
        // スキャン完了
        uint32_t u32_sts = s_gap_ctrl.s_status.u32_status;
        ps_list->b_scan_processing = ((u32_sts & GAP_STS_START_SCAN) == 0x00);
        // スキャン実行時間
        ps_list->u32_scan_duration = s_gap_ctrl.s_status.u32_scan_duration;
        // リモートデバイス件数
        ps_list->u16_count = s_gap_ctrl.u16_dev_cnt;
        // リモートデバイス情報生成
        ts_com_ble_gap_device_info_t* ps_edit_list = pv_mem_malloc(sizeof(ts_com_ble_gap_device_info_t) * s_gap_ctrl.u16_dev_cnt);
        if (ps_edit_list == NULL) {
            l_mem_free(ps_list);
            ps_list = NULL;
            break;
        }
        ps_list->ps_device = ps_edit_list;
        // デバイス情報の編集
        ts_gap_device_t* ps_scan_device = s_gap_ctrl.ps_device;
        ts_com_ble_gap_device_info_t* ps_edit_device = NULL;
        uint16_t u16_idx = 0;
        while (ps_scan_device != NULL) {
            // デバイス情報の編集
            ps_edit_device = &ps_edit_list[u16_idx];
            // BLEアドレス情報
            ps_edit_device->e_addr_type = ps_scan_device->e_addr_type;
            v_com_ble_addr_cpy(ps_edit_device->t_bda, ps_scan_device->t_bda);
            // BLEデバイス名
            ps_edit_device->pc_name = NULL;
            if (ps_scan_device->pc_name != NULL) {
                ps_edit_device->pc_name = pv_mem_malloc(strlen(ps_scan_device->pc_name) + 1);
                if (ps_edit_device->pc_name != NULL) {
                    strcpy(ps_edit_device->pc_name, ps_scan_device->pc_name);
                }
            }
            // RSSI強度
            ps_edit_device->i_rssi = ps_scan_device->i_rssi;
            // GAPデバイスステータス
            ps_edit_device->e_sts = ps_scan_device->u16_status;
            // 次のデバイス情報へ
            ps_scan_device = ps_scan_device->ps_next;
            u16_idx++;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返信
    return ps_list;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gap_delete_device_list
 *
 * DESCRIPTION:BLEのアドレス把握済みのデバイスリストの削除処理
 *
 * PARAMETERS:                      Name        RW  Usage
 * ts_com_ble_gap_device_list_t*    ps_list     R   デバイスリスト
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gap_delete_device_list(ts_com_ble_gap_device_list_t* ps_list) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_list == NULL) {
        return;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // BLEのアドレス把握済みのデバイスリスト
    //==========================================================================
    ts_com_ble_gap_device_info_t* ps_device;
    uint16_t u16_idx;
    for (u16_idx = 0; u16_idx < ps_list->u16_count; u16_idx++) {
        // デバイス情報の編集
        ps_device = &ps_list->ps_device[u16_idx];
        // BLEデバイス名
        if (ps_device->pc_name != NULL) {
            l_mem_free(ps_device->pc_name);
            ps_device->pc_name = NULL;
        }
    }
    // 個々のデバイス情報の解放
    l_mem_free(ps_list->ps_device);
    ps_list->ps_device = NULL;
    // デバイスリスト自体の解放
    l_mem_free(ps_list);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gap_create_device_info
 *
 * DESCRIPTION:名称が一致するBLEのデバイス情報の生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 * char*            pc_device_name  R   デバイス名
 *
 * RETURNS:
 *   ts_com_ble_gap_device_info_t*:デバイス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gap_device_info_t* ps_com_ble_gap_create_device_info(char* pc_device_name) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pc_device_name == NULL) {
        return NULL;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // BLEのアドレス把握済みのデバイスリスト
    //==========================================================================
    ts_com_ble_gap_device_info_t* ps_device = NULL;
    // デバイス情報の編集
    ts_gap_device_t* ps_gap_device = s_gap_ctrl.ps_device;
    while(ps_gap_device != NULL) {
        if (i_vutil_strcmp(pc_device_name, ps_gap_device->pc_name) != 0) {
            // 次のデバイス情報
            ps_gap_device = ps_gap_device->ps_next;
            // 探索を継続
            continue;
        }
        // リモートデバイス情報生成
        ps_device = pv_mem_malloc(sizeof(ts_com_ble_gap_device_info_t));
        if (ps_device == NULL) {
            break;
        }
        // BLEアドレス情報
        ps_device->e_addr_type = ps_gap_device->e_addr_type;
        v_com_ble_addr_cpy(ps_device->t_bda, ps_gap_device->t_bda);
        // BLEデバイス名
        ps_device->pc_name = NULL;
        if (ps_gap_device->pc_name != NULL) {
            ps_device->pc_name = pv_mem_malloc(strlen(ps_gap_device->pc_name) + 1);
            if (ps_device->pc_name != NULL) {
                strcpy(ps_device->pc_name, ps_gap_device->pc_name);
            }
        }
        // RSSI強度
        ps_device->i_rssi = ps_gap_device->i_rssi;
        // GAPデバイスステータス
        ps_device->e_sts = ps_gap_device->u16_status;
        // 探索終了
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 検索結果を返信
    return ps_device;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gap_delete_device_info
 *
 * DESCRIPTION:BLEのデバイス情報の削除処理
 *
 * PARAMETERS:                      Name        RW  Usage
 * ts_com_ble_gap_device_info_t*    ps_result   R   デバイス情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gap_delete_device_info(ts_com_ble_gap_device_info_t*  ps_result) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // BLEスキャン結果を解放
    //==========================================================================
    if (ps_result != NULL) {
        // BLEデバイス名
        if (ps_result->pc_name != NULL) {
            l_mem_free(ps_result->pc_name);
        }
        // メモリ解放
        l_mem_free(ps_result);
        ps_result = NULL;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_set_adv_data
 *
 * DESCRIPTION:アドバタイジングデータの設定処理
 *
 * PARAMETERS:            Name              RW  Usage
 *   esp_ble_adv_data_t*  ps_adv_data       R   アドバタイズパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_set_adv_data(esp_ble_adv_data_t* ps_adv_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // アドバタイズ設定値の有無を判定
    if (ps_adv_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // パラメータ設定
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        // GAPステータス
        ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
        // 次が実行中の場合にはエラー
        // ローカルプライバシーモードの設定
        // アドバタイズパラメータの設定
        // スキャン応答パラメータの設定
        // アドバタイズ
        if ((ps_status->u32_status & GAP_STS_CHK_EXEC_ADVERTISE) != 0x00) {
            // エラーステータス返却
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 設定データの種別判定
        if (!ps_adv_data->set_scan_rsp) {
            // アドバタイズデータ設定の場合
            // パラメータを更新
            ps_status->s_adv_config = *ps_adv_data;
            // ステータスをパラメータ設定待ちに更新
            ps_status->u32_status |= GAP_STS_WAIT_CONFIG_ADVERTISE;
        } else {
            // スキャン応答データ設定の場合
            // パラメータを更新
            ps_status->s_scan_rsp_config = *ps_adv_data;
            // ステータスをパラメータ設定待ちに更新
            ps_status->u32_status |= GAP_STS_WAIT_CONFIG_SCAN_RSP;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_start_advertising
 *
 * DESCRIPTION:アドバタイジングの開始処理
 *
 * PARAMETERS:              Name            RW  Usage
 *   esp_ble_adv_params_t*  ps_adv_params   R  アドバタイズパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_start_advertising(esp_ble_adv_params_t* ps_adv_params) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // アドバタイジングの開始処理
    //==========================================================================
    // アドバタイズの実行開始を試行
    esp_err_t sts_val = sts_gap_start_advertise(ps_adv_params);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_stop_advertising
 *
 * DESCRIPTION:アドバタイジングの停止処理
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_stop_advertising() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // アドバタイジングの停止処理
    //==========================================================================
    // アドバタイズ停止中の場合はエラー
    esp_err_t sts_val = ESP_ERR_INVALID_STATE;
    // パラメータ設定
    uint32_t u32_status = s_gap_ctrl.s_status.u32_status;
    if ((u32_status & GAP_STS_EXEC_ADVERTISING) != 0x00) {
        // アドバタイズ停止
        sts_val = esp_ble_gap_stop_advertising();
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: e_com_ble_gap_adv_device_status
 *
 * DESCRIPTION:GAPアドバタイザの接続デバイスステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   te_gap_dev_sts_t GAPデバイスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_gap_dev_sts_t e_com_ble_gap_adv_device_status() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return GAP_DEV_STS_DEVICE_NONE;
    }

    //==========================================================================
    // GAPデバイスのステータス取得
    // ※アドバタイザに２デバイス以上接続する事は無い前提
    //==========================================================================
    te_gap_dev_sts_t e_device_sts = GAP_DEV_STS_DEVICE_NONE;
    // デバイス数判定
    if (s_gap_ctrl.u16_dev_cnt == 1) {
        // GAPデバイスステータス
        e_device_sts = s_gap_ctrl.ps_device->u16_status;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返信
    return e_device_sts;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_adv_edit_remote_bda
 *
 * DESCRIPTION:GAPアドバタイザのリモートBLEアドレスの編集処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   esp_bd_addr_t  t_rmt_bda   W   編集対象のBLEアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_adv_edit_remote_bda(esp_bd_addr_t t_rmt_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // GAPデバイスのステータス取得
    // ※アドバタイザに２デバイス以上接続する事は無い前提
    //==========================================================================
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    // デバイス数判定
    if (s_gap_ctrl.u16_dev_cnt == 1) {
        // リモートBLEアドレス情報
        v_com_ble_addr_cpy(t_rmt_bda, s_gap_ctrl.ps_device[0].t_bda);
        // ステータス成功
        sts_val = ESP_OK;
    } else {
        // BLEアドレスクリア
        v_com_ble_addr_cpy(t_rmt_bda, t_com_ble_bda_none);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gap_adv_create_device_info
 *
 * DESCRIPTION:GAPアドバタイザの接続デバイス情報の生成処理
 *
 * PARAMETERS:                      Name        RW  Usage
 *
 * RETURNS:
 *   ts_com_ble_gap_device_info_t* GAPデバイスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gap_device_info_t* ps_com_ble_gap_adv_create_device_info() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // GAPデバイスのステータス取得
    // ※アドバタイザに２デバイス以上接続する事は無い前提
    //==========================================================================
    // GAPデバイスの生成
    ts_com_ble_gap_device_info_t* ps_device = NULL;
    do {
        // デバイス数判定
        if (s_gap_ctrl.u16_dev_cnt != 1) {
            break;
        }
        // 編集元のGAPのデバイス情報取得
        ts_gap_device_t* ps_gap_device = s_gap_ctrl.ps_device;
        // 編集対象のGAPデバイス情報
        ps_device = pv_mem_malloc(sizeof(ts_com_ble_gap_device_info_t));
        if (ps_device == NULL) {
            break;
        }
        // BLEアドレスタイプ
        ps_device->e_addr_type = ps_gap_device->e_addr_type;
        // BLEアドレス情報
        v_com_ble_addr_cpy(ps_device->t_bda, ps_gap_device->t_bda);
        // BLEデバイス名
        ps_device->pc_name = NULL;
        char* pc_dev_name = ps_gap_device->pc_name;
        if (pc_dev_name != NULL) {
            ps_device->pc_name = pv_mem_malloc(strlen(pc_dev_name) + 1);
            if (ps_device->pc_name != NULL) {
                strcpy(ps_device->pc_name, pc_dev_name);
            }
        }
        // RSSI強度
        ps_device->i_rssi = ps_gap_device->i_rssi;
        // GAPデバイスステータス
        ps_device->e_sts = ps_gap_device->u16_status;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 生成したGAPデバイス情報を返却
    return ps_device;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_set_adv_params
 *
 * DESCRIPTION:スキャンパラメータの設定処理
 *
 * PARAMETERS:              Name            RW  Usage
 * esp_ble_scan_params_t    ps_scan_params  R   スキャンパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_set_scan_params(esp_ble_scan_params_t* ps_scan_params) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // アドバタイズ設定値の有無を判定
    if (ps_scan_params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // パラメータ設定
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        // GAPステータス
        ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
        // スキャン開始（実行待ちか実行中）の場合にはエラー
        if ((ps_status->u32_status & GAP_STS_START_SCAN) != 0x00) {
            // エラーステータス
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // スキャンパラメータ設定
        ps_status->s_scan_config = *ps_scan_params;
        // ステータスをスキャンパラメータの設定待ちに更新
        ps_status->u32_status &= ~GAP_STS_SET_CONFIG_SCAN;
        ps_status->u32_status |= GAP_STS_WAIT_CONFIG_SCAN;
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: b_com_ble_gap_is_scanning
 *
 * DESCRIPTION:スキャン中判定
 *
 * PARAMETERS:        Name            RW  Usage
 *
 * RETURNS:
 *   true:スキャン中
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_com_ble_gap_is_scanning() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // スキャン開始処理
    //==========================================================================
    // スキャンステータスの更新処理　※タイムアウトによる更新
    sts_gap_update_scan_status();
    // GAPステータス
    uint32_t u32_status = s_gap_ctrl.s_status.u32_status;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return ((u32_status & GAP_STS_START_SCAN) != 0x00);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_start_scan
 *
 * DESCRIPTION:スキャンの開始処理
 *
 * PARAMETERS:        Name            RW  Usage
 *   uint32_t         u32_duration    R   スキャン時間
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_start_scan(uint32_t u32_duration) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // スキャン時間の有無を判定
    if (u32_duration == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // スキャン開始処理
    //==========================================================================
    // スキャン実行開始処理
    esp_err_t sts_val = sts_gap_start_scan(u32_duration);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gap_stop_scan
 *
 * DESCRIPTION:スキャンの停止処理
 *
 * PARAMETERS:        Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gap_stop_scan() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // スキャン停止処理
    //==========================================================================
    // スキャン停止中の場合はエラー
    esp_err_t sts_val = ESP_ERR_INVALID_STATE;
    // パラメータ設定
    uint32_t u32_status = s_gap_ctrl.s_status.u32_status;
    if ((u32_status & GAP_STS_EXEC_SCAN) != 0x00) {
        // スキャン停止
        sts_val = esp_ble_gap_stop_scanning();
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: u16_com_ble_gap_scan_device_count
 *
 * DESCRIPTION:BLEのアドレス把握済みのデバイス数の取得
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   uint16_t:把握済みのデバイス数
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint16_t u16_com_ble_gap_scan_device_count() {
    return s_gap_ctrl.u16_dev_cnt;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gatt_delete_rx_data
 *
 * DESCRIPTION:GATTプロファイルの書き込み受信データの削除処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gatt_rx_data_t*   ps_data     RW  削除対象の受信データ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gatt_delete_rx_data(ts_com_ble_gatt_rx_data_t* ps_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_data == NULL) {
        return;
    }
    // 受信データの解放
    sts_mdl_delete_u8_array(ps_data->ps_array);
    // 受信書き込みデータの解放
    l_mem_free(ps_data);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_init
 *
 * DESCRIPTION:GATTサーバーの初期設定処理
 *
 * PARAMETERS:                Name          RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_init() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 初期化処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    if ((s_gatts_ctrl.u8_status & GATTS_STS_INIT) == 0x00) {
        // GATTSモジュールにイベントコールバック関数を登録
        sts_val = esp_ble_gatts_register_callback(v_gatts_evt_com_cb);
        if (sts_val == ESP_OK){
            s_gatts_ctrl.u8_status |= GATTS_STS_INIT;
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: s_com_ble_gatts_app_config_default
 *
 * DESCRIPTION:GATTサーバーのアプリケーション情報の生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   com_ble_gatts_app_config 生成されたGATTサーバーアプリケーション設定
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatts_if_config_t s_com_ble_gatts_app_config_default() {
    // デフォルト値を返却
    return s_gatts_cfg_default;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_app_register
 *
 * DESCRIPTION:GATTサーバーへのアプリケーション登録処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * com_ble_gatts_app_config_t*  ps_app_cfg  R   GATTインターフェース設定
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_app_register(ts_com_ble_gatts_if_config_t* ps_if_cfg) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_if_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // セキュリティタイプの判定
    esp_ble_sec_act_t e_con_sec = ps_if_cfg->e_con_sec;
    if (e_con_sec < ESP_BLE_SEC_ENCRYPT || e_con_sec > ESP_BLE_SEC_ENCRYPT_MITM) {
        return ESP_ERR_INVALID_ARG;
    }
    // サービス数
    if (ps_if_cfg->u8_svc_cnt <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // サービス設定
    ts_com_ble_gatts_svc_config_t* ps_svc_cfg = ps_if_cfg->ps_svc_cfg;
    if (ps_svc_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // アトリビュートテーブル
    if (ps_svc_cfg->ps_attr_db == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // インターフェース毎のコールバック関数
    if (ps_if_cfg->fc_gatts_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // インターフェースステータスの追加
    //==========================================================================
    // 既に登録済みの場合にはエラー
    esp_err_t sts_val = ESP_OK;
    do {
        // インターフェースステータスの探索
        ts_gatts_if_status_t s_sts_dmy;
        ts_gatts_if_status_t* ps_sts_bef = &s_sts_dmy;
        ts_gatts_if_status_t* ps_sts_add = s_gatts_ctrl.ps_if_status;
        while (ps_sts_add != NULL) {
            if (ps_sts_add->u16_app_id == ps_if_cfg->u16_app_id) {
                break;
            }
            // 次のステータスへ
            ps_sts_bef = ps_sts_add;
            ps_sts_add = ps_sts_add->ps_next;
        }
        // すでにアプリケーションIDが追加されていた場合には終了
        if (ps_sts_add != NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // GATTサーバーのインターフェースステータス生成
        //----------------------------------------------------------------------
        // メモリの割り当てと追加
        ps_sts_add = pv_mem_malloc(sizeof(ts_gatts_if_status_t));
        if (ps_sts_add == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
        // GATTサーバーのIFステータス初期化
        *ps_sts_add = s_gatts_if_sts_default;
        ps_sts_add->u16_app_id = ps_if_cfg->u16_app_id;
        ps_sts_add->s_cfg = *ps_if_cfg;

        //----------------------------------------------------------------------
        // GATTサーバーのサービスステータス生成
        //----------------------------------------------------------------------
        ps_sts_add->u8_svc_cnt = ps_if_cfg->u8_svc_cnt;
        ps_sts_add->ps_svc_sts = pv_mem_malloc(ps_if_cfg->u8_svc_cnt * sizeof(ts_gatts_svc_status_t));
        if (ps_sts_add->ps_svc_sts == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            // メモリを解放して終了
            l_mem_free(ps_sts_add);
            break;
        }
        // GATTサーバーのサービスステータス
        ts_com_ble_gatts_svc_config_t* ps_svc_cfg = ps_if_cfg->ps_svc_cfg;
        ts_gatts_svc_status_t* ps_svc_sts = ps_sts_add->ps_svc_sts;
        uint8_t u8_svc_idx;
        for (u8_svc_idx = 0; u8_svc_idx < ps_if_cfg->u8_svc_cnt; u8_svc_idx++) {
            ps_svc_sts[u8_svc_idx].t_gatt_if      = ESP_GATT_IF_NONE;
            ps_svc_sts[u8_svc_idx].u16_app_id     = ps_sts_add->u16_app_id;
            ps_svc_sts[u8_svc_idx].u8_svc_inst_id = ps_svc_cfg[u8_svc_idx].u8_inst_id;
            ps_svc_sts[u8_svc_idx].s_cfg          = ps_svc_cfg[u8_svc_idx];
            ps_svc_sts[u8_svc_idx].u8_max_nb_attr = ps_svc_cfg[u8_svc_idx].u8_max_nb_attr;
            ps_svc_sts[u8_svc_idx].u16_num_handle = 0;
            ps_svc_sts[u8_svc_idx].pu16_handles   = NULL;
            ps_svc_sts[u8_svc_idx].t_rx_queue = xQueueCreate(COM_BLE_GATT_RX_BUFF_SIZE, sizeof(ts_com_ble_gatt_rx_data_t*));
            if (ps_svc_sts[u8_svc_idx].t_rx_queue == NULL) {
                sts_val = ESP_ERR_NO_MEM;
                // メモリを解放して終了
                l_mem_free(ps_sts_add->ps_svc_sts);
                l_mem_free(ps_sts_add);
                break;
            }
        }
        // 生成したステータスを追加
        ps_sts_bef->ps_next = ps_sts_add;
        s_gatts_ctrl.u8_status    |= GATTS_STS_IF_CFG;
        s_gatts_ctrl.ps_if_status =  s_sts_dmy.ps_next;

        //======================================================================
        // GATTサーバーにアプリケーションIDを登録
        //======================================================================
        sts_val = esp_ble_gatts_app_register(ps_sts_add->u16_app_id);
    } while(false);


    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: t_com_ble_gatts_if
 *
 * DESCRIPTION:GATTサーバーのGATTインターフェース取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 * uint16_t             u16_app_id  R   アプリケーションID
 *
 * RETURNS:
 * esp_gatt_if_t:GATTインターフェース
 * ※アプリケーションIDとの紐づけがされていない場合にはESP_GATT_IF_NONE
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_gatt_if_t t_com_ble_gatts_if(uint16_t u16_app_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_GATT_IF_NONE;
    }

    //==========================================================================
    // GATTインターフェース探索
    //==========================================================================
    // GATTインターフェースを順次検索
    esp_gatt_if_t t_gatt_if = ESP_GATT_IF_NONE;
    ts_gatts_if_status_t* ps_sts = s_gatts_ctrl.ps_if_status;
    while (ps_sts != NULL) {
        if (ps_sts->u16_app_id == u16_app_id) {
            t_gatt_if = ps_sts->t_gatt_if;
            break;
        }
        // 次のステータスへ
        ps_sts = ps_sts->ps_next;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // GATTインターフェースを返す
    return t_gatt_if;
}

/*******************************************************************************
 *
 * NAME: s_com_ble_gatts_svc_info
 *
 * DESCRIPTION:GATTサーバーのサービス情報取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_gatt_if_t        t_gatt_if   R   GATTインターフェース
 * uint8_t              u8_svc_idx  R   サービスインデックス
 *
 * RETURNS:
 * ts_com_ble_gatts_svc_info:サービス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatts_svc_info_t s_com_ble_gatts_svc_info(esp_gatt_if_t t_gatt_if, uint8_t u8_svc_idx) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return s_gatts_svc_info_default;
    }

    //==========================================================================
    // ハンドルインデックスリストの探索
    //==========================================================================
    // サービス情報生成
    ts_com_ble_gatts_svc_info_t s_svc_info = s_gatts_svc_info_default;
    do {
        // GATTインターフェースの取得
        ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスの探索
        if (u8_svc_idx >= ps_if_sts->u8_svc_cnt) {
            break;
        }
        // サービスステータス取得
        ts_gatts_svc_status_t* ps_svc_sts = &ps_if_sts->ps_svc_sts[u8_svc_idx];
        s_svc_info.u16_app_id     = ps_svc_sts->u16_app_id;
        s_svc_info.t_gatt_if      = ps_svc_sts->t_gatt_if;
        s_svc_info.u8_svc_inst_id = ps_svc_sts->u8_svc_inst_id;
        s_svc_info.u16_num_handle = ps_svc_sts->u16_num_handle;
        s_svc_info.pu16_handles   = ps_svc_sts->pu16_handles;
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // サービス情報を返却
    return s_svc_info;
}

/*******************************************************************************
 *
 * NAME: b_com_ble_gatts_is_connected
 *
 * DESCRIPTION:GATTサーバーへのコネクション有無判定
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_if_t    t_gatt_if   R   GATTインターフェース
 *
 * RETURNS:
 *   true:GATTクライアントからの接続有り
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_com_ble_gatts_is_connected(esp_gatt_if_t t_gatt_if) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // 接続判定
    //==========================================================================
    // 判定結果
    bool b_result = false;
    do {
        //----------------------------------------------------------------------
        // GATTサーバーコネクション
        //----------------------------------------------------------------------
        // GATTサーバーのコネクションステータスの探索
        ts_gatts_con_status_t* ps_con_sts = NULL;
        // GATTインターフェースステータス
        ts_gatts_if_status_t* ps_if_sts = s_gatts_ctrl.ps_if_status;
        while (ps_if_sts != NULL) {
            if (ps_if_sts->t_gatt_if != t_gatt_if) {
                // 次のステータス
                ps_if_sts = ps_if_sts->ps_next;
                continue;
            }
            // GATTコネクション
            ps_con_sts = ps_if_sts->ps_con_sts;
            break;
        }
        // コネクション判定
        b_result = (ps_con_sts != NULL);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // GATTクライアントからの接続状態を返却
    return b_result;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gatts_create_con_info
 *
 * DESCRIPTION:GATTサーバーへのコネクション情報取得
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_if_t    t_gatt_if   R   GATTインターフェース
 *
 * RETURNS:
 *   ts_com_ble_gatts_con_info*:GATTクライアントからの接続情報
 *   ※無い場合にはDefault Value
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatts_con_info_t* ps_com_ble_gatts_create_con_info(esp_gatt_if_t t_gatt_if) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // アプリケーション情報設定
    //==========================================================================
    // GATTインターフェースステータスの探索
    ts_com_ble_gatts_con_info_t* ps_con_info = NULL;
    ts_gatts_if_status_t* ps_if_sts = s_gatts_ctrl.ps_if_status;
    while (ps_if_sts != NULL) {
        if (ps_if_sts->t_gatt_if != t_gatt_if) {
            // 次のステータス
            ps_if_sts = ps_if_sts->ps_next;
            continue;
        }
        // GATTコネクション判定
        if (ps_if_sts->ps_con_sts == NULL) {
            break;
        }
        // GATTコネクション情報　※編集元
        ts_gatts_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
        // GATT接続情報の初期値　※編集先
        ps_con_info = pv_mem_malloc(sizeof(ts_com_ble_gatts_con_info_t));
        if (ps_con_info == NULL) {
            break;
        }
        ps_con_info->u16_app_id = ps_con_sts->u16_app_id;
        ps_con_info->t_gatt_if  = ps_con_sts->t_gatt_if;
        ps_con_info->u16_con_id = ps_con_sts->u16_con_id;
        v_com_ble_addr_cpy(ps_con_info->t_bda, ps_con_sts->t_bda);
        ps_con_info->u16_mtu    = ps_con_sts->u16_mtu;
        // GAPデバイス情報の取得　※編集元
        ts_gap_device_t* ps_dev = ps_gap_get_device(ps_con_sts->t_bda);
        if (ps_dev != NULL) {
            ps_con_info->e_addr_type = ps_dev->e_addr_type;
            ps_con_info->i_rssi      = ps_dev->i_rssi;
        } else {
            ps_con_info->e_addr_type = BLE_ADDR_TYPE_PUBLIC;
            ps_con_info->i_rssi      = 127;
        }
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // GATTクライアントからの接続情報を返却
    return ps_con_info;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gatts_delete_con_info
 *
 * DESCRIPTION:GATTサーバーへのコネクション情報削除処理
 *
 * PARAMETERS:                  Name            RW  Usage
 * ts_com_ble_gatts_con_info_t* ps_con_info     R   コネクション情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gatts_delete_con_info(ts_com_ble_gatts_con_info_t* ps_con_info) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // コネクション情報の削除
    //==========================================================================
    if (ps_con_info != NULL) {
        // コネクション情報の解放
        l_mem_free(ps_con_info);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_get_handle_idx
 *
 * DESCRIPTION:GATTサーバーのアトリビュートハンドルインデックスの取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint16_t         u16_handle      R   アトリビュートハンドル
 * uint8_t*         pu8_svc_idx     W   サービスインデックス
 * uint16_t*        pu16_hndl_idx   W   アトリビュートインデックス
 *
 * RETURNS:
 * esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_get_handle_idx(esp_gatt_if_t t_gatt_if,
                                           uint16_t u16_handle,
                                           uint8_t* pu8_svc_idx,
                                           uint16_t* pu16_hndl_idx) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // ハンドルインデックスの取得処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;
    ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
    if (ps_if_sts != NULL) {
        // 受信ハンドルチェック
        sts_val = sts_gatts_get_handle_idx(ps_if_sts, u16_handle, pu8_svc_idx, pu16_hndl_idx);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gatts_get_attribute
 *
 * DESCRIPTION:GATTサーバーのアトリビュートの取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint8_t          u8_svc_idx      R   サービスインデックス
 * uint16_t         u16_hndl_idx    R   アトリビュートインデックス
 *
 * RETURNS:
 *  esp_gatts_attr_db_t*:アトリビュートへの参照
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_gatts_attr_db_t* ps_com_ble_gatts_get_attribute(esp_gatt_if_t t_gatt_if,
                                                    uint8_t u8_svc_idx,
                                                    uint16_t u16_hndl_idx) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // ハンドルインデックスの取得処理
    //==========================================================================
    // 結果ステータス
    esp_gatts_attr_db_t* ps_attr = NULL;
    // インターフェースステータス
    ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
    if (ps_if_sts != NULL) {
        // アトリビュートステータス
        ps_attr = ps_gatts_get_attribute(ps_if_sts, u8_svc_idx, u16_hndl_idx);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    return ps_attr;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gatts_rx_data
 *
 * DESCRIPTION:GATTサーバーの受信書き込みデータ取得処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_if_t    t_gatt_if   R   GATTインターフェース
 * uint8_t          u8_svc_idx  R   サービスインデックス
 * TickType_t       t_tick      R   デキュー時の最大待ち時間
 *
 * RETURNS:
 *   ts_com_ble_gatt_rx_data:受信書き込みデータ
 *   ※無い場合にはウェイト
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatt_rx_data_t* ps_com_ble_gatts_rx_data(esp_gatt_if_t t_gatt_if,
                                                    uint8_t u8_svc_idx,
                                                    TickType_t t_tick) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // 受信キューの取得
    //==========================================================================
    QueueHandle_t t_rx_queue = NULL;
    do {
        //----------------------------------------------------------------------
        // キューハンドルの取得
        //----------------------------------------------------------------------
        // GATTインターフェースステータス
        ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス取得
        if (u8_svc_idx >= ps_if_sts->u8_svc_cnt) {
            break;
        }
        // サービスステータス
        ts_gatts_svc_status_t* ps_svc_sts = &ps_if_sts->ps_svc_sts[u8_svc_idx];
        if (ps_svc_sts == NULL) {
            break;
        }
        // 受信キューを取得
        t_rx_queue = ps_svc_sts->t_rx_queue;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 受信キューを判定
    //==========================================================================
    if (t_rx_queue == NULL) {
        return NULL;
    }

    //==========================================================================
    // 受信データを取得
    //==========================================================================
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    xQueueReceive(t_rx_queue, &ps_rx_data, t_tick);

    // 受信データ情報を返信
    return ps_rx_data;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gatts_rx_clear
 *
 * DESCRIPTION:GATTプロファイルの受信バッファのクリア処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_if_t    t_gatt_if   R   GATTインターフェース
 * uint8_t          u8_svc_idx  R   サービスインデックス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gatts_rx_clear(esp_gatt_if_t t_gatt_if, uint8_t u8_svc_idx) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // GATTサーバーの受信キュー取得
    //==========================================================================
    QueueHandle_t t_rxw_queue = NULL;
    do {
        // GATTインターフェースステータス
        ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス取得
        if (u8_svc_idx >= ps_if_sts->u8_svc_cnt) {
            break;
        }
        ts_gatts_svc_status_t* ps_svc_sts = &ps_if_sts->ps_svc_sts[u8_svc_idx];
        // 受信キュー取得
        t_rxw_queue = ps_svc_sts->t_rx_queue;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 受信書き込みデータの取得
    //==========================================================================
    // キューの有無を判定
    if (t_rxw_queue == NULL) {
        return;
    }
    // 受信データのキュークリア
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    while (xQueueReceive(t_rxw_queue, &ps_rx_data, 0) == pdTRUE) {
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
    }
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_tx_data
 *
 * DESCRIPTION:GATTサーバーからのレスポンスの送信処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_gatt_if_t        t_gatt_if   R   GATTインターフェース
 * gatts_read_evt_param ps_param    R   READパラメータ
 * uint8_t              u8_auth_req R   認証リクエスト
 * uint8_t*             pu8_value   R   値
 * uint16_t             u16_length  R   値の長さ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_tx_data(esp_gatt_if_t t_gatt_if,
                                    struct gatts_read_evt_param* ps_param,
                                    uint8_t u8_auth_req,
                                    uint8_t* pu8_value,
                                    uint16_t u16_length) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // レスポンス送信処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // GATTサーバーイベント処理中のインターフェースステータス
        ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // 受信ハンドルチェック
        uint8_t u8_svc_idx;
        uint16_t u16_hndl_idx;
        // アトリビュートの取得
        esp_gatts_attr_db_t* ps_rx_attr = ps_gatts_get_handle_attribute(ps_if_sts, ps_param->handle, &u8_svc_idx, &u16_hndl_idx);
        if (ps_rx_attr == NULL) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // レスポンスを編集
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.handle              = ps_param->handle;
        rsp.attr_value.handle   = ps_param->handle;
        rsp.attr_value.offset   = ps_param->offset;
        rsp.attr_value.len      = u16_length;
        rsp.attr_value.auth_req = u8_auth_req;
        memcpy(rsp.attr_value.value, pu8_value, u16_length);
        // レスポンスを返信
        sts_val = esp_ble_gatts_send_response(t_gatt_if, ps_param->conn_id, ps_param->trans_id, ESP_GATT_OK, &rsp);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // レスポンスを返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_indicate
 *
 * DESCRIPTION:BLEのGATTプロファイルにおけるIndicateの送信処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint16_t         u16_handle      R   アトリビュートハンドル
 * uint8_t*         pu8_data        R   送信データポインタ
 * size_t           t_data_len      R   送信データ長
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_indicate(esp_gatt_if_t t_gatt_if,
                                     uint8_t u8_svc_idx,
                                     uint16_t u16_handle,
                                     uint8_t* pu8_data,
                                     size_t t_data_len) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Indication処理
    //==========================================================================
    // Indicate通知
    esp_err_t sts_val = sts_gatts_indication(t_gatt_if,
                                             u8_svc_idx,
                                             u16_handle,
                                             pu8_data,
                                             t_data_len,
                                             true);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gatts_notify
 *
 * DESCRIPTION:BLEのGATTプロファイルにおけるNotifyの送信処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint16_t         u16_handle      R   アトリビュートハンドル
 * uint8_t*         pu8_data        R   送信データポインタ
 * size_t           t_data_len      R   送信データ長
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gatts_notify(esp_gatt_if_t t_gatt_if,
                                      uint8_t u8_svc_idx,
                                      uint16_t u16_handle,
                                      uint8_t* pu8_data,
                                      size_t t_data_len) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Notify処理
    //==========================================================================
    // Indicate通知
    esp_err_t sts_val = sts_gatts_indication(t_gatt_if,
                                             u8_svc_idx,
                                             u16_handle,
                                             pu8_data,
                                             t_data_len,
                                             false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: s_com_ble_gattc_app_config_default
 *
 * DESCRIPTION:GATTクライアントのデフォルト設定の生成処理
 *
 * PARAMETERS:                  Name        RW  Usage
 *
 * RETURNS:
 *   com_ble_gattc_app_config:
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gattc_if_config_t s_com_ble_gattc_app_config_default() {
    return s_gattc_if_cfg_default;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_register
 *
 * DESCRIPTION:GATTクライアントプロファイル初期処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * gattc_application_config_t*  ps_app_cfg  R   GATTクライアント設定
 * uint16_t                     u16_size    R   GATTクライアントアプリケーション数
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_register(ts_com_ble_gattc_if_config_t* ps_app_cfg, uint16_t u16_size) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 引数の有無を判定
    if (ps_app_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // アプリケーション毎のチェック
    int i_idx;
    for (i_idx = 0; i_idx < u16_size; i_idx++) {
        // サービス数
        if (ps_app_cfg[i_idx].u8_svc_cnt == 0) {
            return ESP_ERR_INVALID_ARG;
        }
        // サービスのUUID配列
        if (ps_app_cfg[i_idx].pt_svc_uuid == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        // 接続時のセキュリティタイプ
        esp_ble_sec_act_t e_con_sec = ps_app_cfg[i_idx].e_con_sec;
        if (e_con_sec < ESP_BLE_SEC_ENCRYPT || e_con_sec > ESP_BLE_SEC_ENCRYPT_MITM) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // アプリケーション情報設定
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // 既に登録済みの場合にはエラー
        if (s_gattc_ctrl.ps_if_config != NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        //----------------------------------------------------------------------
        // GATTCモジュールにイベントコールバック関数を登録
        //----------------------------------------------------------------------
        sts_val = esp_ble_gattc_register_callback(v_gattc_evt_com_cb);
        if (sts_val != ESP_OK) {
            break;
        }
        //----------------------------------------------------------------------
        // メモリの確保
        //----------------------------------------------------------------------
        // GATTクライアントのアプリケーション情報初期化
        ts_com_ble_gattc_if_config_t* ps_config;
        ps_config = pv_mem_malloc(sizeof(ts_com_ble_gattc_if_config_t) * u16_size);
        if (ps_config == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
        ts_gattc_if_status_t* ps_status;
        ps_status = pv_mem_malloc(sizeof(ts_gattc_if_status_t) * u16_size);
        if (ps_status == NULL) {
            l_mem_free(ps_config);
            sts_val = ESP_ERR_NO_MEM;
            break;
        }

        //----------------------------------------------------------------------
        // 初期化
        //----------------------------------------------------------------------
        // インターフェース数
        s_gattc_ctrl.u16_if_count = u16_size;
        // GATTクライアントのアプリケーション情報初期化
        s_gattc_ctrl.ps_if_config = ps_config;
        // GATTクライアントのステータス情報を初期化
        s_gattc_ctrl.ps_if_status = ps_status;
        int i_idx;
        for (i_idx = 0; i_idx < u16_size; i_idx++) {
            ps_config[i_idx] = ps_app_cfg[i_idx];
            ps_status[i_idx] = s_gattc_if_sts_default;
            ps_status[i_idx].u16_app_id = ps_app_cfg[i_idx].u16_app_id;
            ps_status[i_idx].ps_if_cfg  = &ps_app_cfg[i_idx];
            // アプリケーションIDを登録
            sts_val = esp_ble_gattc_app_register(ps_app_cfg[i_idx].u16_app_id);
            if (sts_val != ESP_OK){
                break;
            }
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: t_com_ble_gattc_if
 *
 * DESCRIPTION:GATTクライアントのGATTインターフェース取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 * uint16_t             u16_app_id  R   アプリケーションID
 *
 * RETURNS:
 * esp_gatt_if_t:GATTインターフェース
 * ※アプリケーションIDとの紐づけがされていない場合にはESP_GATT_IF_NONE
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_gatt_if_t t_com_ble_gattc_if(uint16_t u16_app_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_GATT_IF_NONE;
    }

    //==========================================================================
    // GATTインターフェース探索
    //==========================================================================
    // IFステータスの探索
    ts_gattc_if_status_t* ps_if_status = s_gattc_ctrl.ps_if_status;
    esp_gatt_if_t t_gatt_if = ESP_GATT_IF_NONE;
    int i_idx;
    for (i_idx = 0; i_idx < s_gattc_ctrl.u16_if_count; i_idx++) {
        if (ps_if_status[i_idx].u16_app_id != u16_app_id) {
            continue;
        }
        t_gatt_if = ps_if_status[i_idx].t_gatt_if;
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // GATTインターフェースを返す
    return t_gatt_if;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_open
 *
 * DESCRIPTION:GATTサーバーとの接続処理
 *
 * PARAMETERS:          Name                RW  Usage
 * esp_gatt_if_t        t_gatt_if           R   GATTインターフェース
 * esp_bd_addr_t        t_bda               R   リモートアドレス
 * esp_ble_addr_type_t  e_remote_addr_type  R   リモートアドレスタイプ
 * bool                 b_direct            R   自動接続
 *
 * RETURNS:
 *   com_ble_gattc_app_status* GATTクライアントのステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_open(esp_gatt_if_t t_gatt_if,
                                 esp_bd_addr_t t_bda,
                                 esp_ble_addr_type_t e_remote_addr_type,
                                 bool b_direct) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 接続処理
    //==========================================================================
    // GATTプロファイルでの接続開始
    esp_err_t sts_val = sts_gattc_open(t_gatt_if, t_bda, e_remote_addr_type, b_direct);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_close
 *
 * DESCRIPTION:GATTサーバーとの切断処理
 *
 * PARAMETERS:          Name                RW  Usage
 * esp_gatt_if_t        t_gatt_if           R   GATTインターフェース
 * esp_bd_addr_t        t_bda               R   リモートアドレス
 *
 * RETURNS:
 *   com_ble_gattc_app_status* GATTクライアントのステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_close(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 切断処理
    //==========================================================================
    // GATTプロファイルでの切断開始
    esp_err_t sts_val = sts_gattc_close(t_gatt_if, t_bda);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: e_com_ble_gattc_con_sts
 *
 * DESCRIPTION:GATTサーバーとのコネクションステータス取得
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_gatt_if_t        t_gatt_if   R   GATTインターフェース
 * esp_bd_addr_t        t_bda       R   リモートアドレス
 *
 * RETURNS:
 *   te_gattc_con_sts:GATTサーバーへのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_gattc_con_sts_t e_com_ble_gattc_con_sts(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // ステータス判定
    //==========================================================================
    te_gattc_con_sts_t e_status = GATTC_STS_NONE;
    do {
        //---------------------------------------------------------------------
        // サービス情報
        //---------------------------------------------------------------------
        // コネクションステータス取得
        ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_bda(t_gatt_if, t_bda);
        if (ps_con_sts == NULL) {
            break;
        }
        // コネクションのステータス取得
        e_status = ps_con_sts->u8_status;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return e_status;
}

/*******************************************************************************
 *
 * NAME: e_com_ble_gattc_con_sts_wait
 *
 * DESCRIPTION:GATTサーバーとのコネクションステータス待ち処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_gatt_if_t        t_gatt_if   R   GATTインターフェース
 * esp_bd_addr_t        t_bda       R   リモートアドレス
 * te_gattc_con_sts_t   e_chk_sts   R   対象のGATTサーバーとのコネクションステータス（論理和可能）
 * TickType_t           t_max_wait  R   最大待ち時間
 *
 * RETURNS:
 *   te_gattc_con_sts:GATTサーバーへのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_gattc_con_sts_t e_com_ble_gattc_con_sts_wait(esp_gatt_if_t t_gatt_if,
                                                esp_bd_addr_t t_bda,
                                                te_gattc_con_sts_t e_chk_sts,
                                                TickType_t t_max_wait) {
    // タイムアウト時刻の取得
    TickType_t t_timeout = xTaskGetTickCount() + t_max_wait;
    // GATTクライアントステータス
    te_gattc_con_sts_t e_sts;
    // 判定ステータスによる判別
    if (e_chk_sts == GATTC_STS_NONE) {
        // 判定ステータスがGATTC_STS_NONEの場合は完全一致
        do {
            // GATTクライアントステータス取得
            e_sts = e_com_ble_gattc_con_sts(t_gatt_if, t_bda);
            // ステータスチェック
            if (e_sts == GATTC_STS_NONE) {
                break;
            }
            // ウェイト
            vTaskDelay(GATT_CON_STS_UPD_WAIT_TICK);
        } while (t_timeout >= xTaskGetTickCount());
    } else {
        // 判定ステータスがGATTC_STS_NONE以外の場合は、部分一致
        do {
            // GATTクライアントステータス取得
            e_sts = e_com_ble_gattc_con_sts(t_gatt_if, t_bda);
            // ステータスチェック
            if ((e_sts & e_chk_sts) != GATTC_STS_NONE) {
                break;
            }
            // ウェイト
            vTaskDelay(GATT_CON_STS_UPD_WAIT_TICK);
        } while (t_timeout >= xTaskGetTickCount());
    }
    // 結果ステータス返却
    return e_sts;
}


/*******************************************************************************
 *
 * NAME: ps_com_ble_gattc_create_con_info
 *
 * DESCRIPTION:GATTサーバーのコネクション情報取得処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_gatt_if_t        t_gatt_if   R   GATTインターフェース
 * esp_bd_addr_t        t_bda       R   リモートアドレス
 *
 * RETURNS:
 *   ts_com_ble_gattc_con_info*:GATTサーバーのコネクション情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gattc_con_info_t* ps_com_ble_gattc_create_con_info(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // サービス情報編集
    //==========================================================================
    ts_com_ble_gattc_con_info_t* ps_con = NULL;
    do {
        // コネクションステータス取得
        ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_bda(t_gatt_if, t_bda);
        if (ps_con_sts == NULL) {
            break;
        }
        // サービス設定済み判定
        if ((ps_con_sts->u8_status & GATTC_STS_CONNECTED) != GATTC_STS_CONNECTED) {
            break;
        }
        // コネクション情報取得
        ps_con = pv_mem_malloc(sizeof(ts_com_ble_gattc_con_info_t));
        if (ps_con == NULL) {
            break;
        }
        // コネクション情報の編集
        ps_con->t_gatt_if      = ps_con_sts->t_gatt_if;
        ps_con->u16_app_id     = ps_con_sts->u16_app_id;
        ps_con->u16_con_id     = ps_con_sts->u16_con_id;
        v_com_ble_addr_cpy(ps_con->t_bda, ps_con_sts->t_bda);
        ps_con->u16_mtu        = ps_con_sts->u16_mtu;
        ps_con->e_sec_auth_req = ps_con_sts->e_sec_auth_req;
        ps_con->u16_svc_cnt    = ps_con_sts->u16_svc_cnt;
        // サービス情報の生成
        ps_con->ps_service     = pv_mem_malloc(sizeof(ts_com_ble_gattc_svc_info_t) * ps_con->u16_svc_cnt);
        if (ps_con->ps_service == NULL) {
            break;
        }
        // サービス情報の編集
        ts_gattc_svc_status_t* ps_svc_sts = ps_con_sts->ps_svc_sts;
        ts_com_ble_gattc_svc_info_t* ps_svc_info;
        uint16_t u16_idx = 0;
        while (ps_svc_sts != NULL) {
            ps_svc_info = &ps_con->ps_service[u16_idx];
            ps_svc_info->s_svc_id           = ps_svc_sts->s_svc_id;
            ps_svc_info->b_primary          = ps_svc_sts->b_primary;
            ps_svc_info->u16_svc_start_hndl = ps_svc_sts->u16_start_hndl;
            ps_svc_info->u16_svc_end_hndl   = ps_svc_sts->u16_end_hndl;
            ps_svc_info->u16_db_elem_cnt    = ps_svc_sts->u16_db_elem_cnt;
            ps_svc_info->ps_db_elems        = ps_svc_sts->ps_db_elems;
            // 次のサービスへ
            ps_svc_sts = ps_svc_sts->ps_next;
            u16_idx++;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return ps_con;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gattc_delete_con_info
 *
 * DESCRIPTION:GATTサーバーのコネクション情報削除処理
 *
 * PARAMETERS:                      Name        RW  Usage
 * ts_com_ble_gattc_con_info_t*     ps_con      R   コネクション情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_gattc_delete_con_info(ts_com_ble_gattc_con_info_t* ps_con) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // コネクション情報の削除
    //==========================================================================
    if (ps_con != NULL) {
        // サービスの解放
        l_mem_free(ps_con->ps_service);
        // コネクション情報の解放
        l_mem_free(ps_con);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_get_db
 *
 * DESCRIPTION:GATTサーバーからのアトリビュートDBの取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTクライアントインターフェース
 * uint16_t         u16_con_id      R   コネクションID
 * esp_gatt_id_t    s_svc_id        R   サービスID
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_get_db(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id, esp_gatt_id_t s_svc_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // アトリビュートの個数を取得してサービス毎のアトリビュートDB情報を取得
    //==========================================================================
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;

    do {
        // IFステータス取得
        ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス取得
        ts_gattc_svc_status_t* ps_svc_sts = ps_gattc_get_svc_status(ps_if_sts, u16_con_id, s_svc_id);
        if (ps_svc_sts != NULL) {
            // サービスのアトリビュートDB取得
            sts_val = sts_gattc_get_db(ps_svc_sts);
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_cache_clean
 *
 * DESCRIPTION:GATTサーバーからの取得したアトリビュートDBのローカルキャッシュのリフレッシュ処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTクライアントインターフェース
 *
 * RETURNS:
 *   esp_err_t 結果ステータス、未接続の場合には「ESP_ERR_NOT_FOUND」を返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_cache_clean(esp_gatt_if_t t_gatt_if) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // アトリビュートの個数を取得してサービス毎のアトリビュートDB情報を取得
    //==========================================================================
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;
    do {
        // 入力チェック
        if (t_gatt_if == ESP_GATT_IF_NONE) {
            break;
        }
        // GATTクライアントのステータス取得
        ts_gattc_if_status_t* ps_gattc_is_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_gattc_is_sts == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        if (ps_gattc_is_sts->ps_con_sts == NULL) {
            sts_val = ESP_ERR_NOT_FOUND;
            break;
        }
        // キャッシュクリアのリクエスト
        ps_gattc_is_sts->b_req_cache_clear = true;
        // ローカルキャッシュのアドレスリストを取得
        sts_val = esp_ble_gattc_cache_get_addr_list(t_gatt_if);
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返信
    return sts_val;
}


/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_enable_notify
 *
 * DESCRIPTION:GATTサーバーからGATTクライアントへの通知有効化処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTクライアントインターフェース
 * uint16_t         u16_con_id      R   コネクションID
 * esp_gatt_id_t    s_svc_id        R   サービスID
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_enable_notify(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id, esp_gatt_id_t s_svc_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // GATTサーバーからGATTクライアントへの通知有効化処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;

    do {
        // インターフェースステータス取得
        ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス取得
        ts_gattc_svc_status_t* ps_svc_sts = ps_gattc_get_svc_status(ps_if_sts, u16_con_id, s_svc_id);
        if (ps_svc_sts != NULL) {
            // Notify有効化
            sts_val = sts_gattc_register_for_notify(ps_svc_sts);
        }

    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_gattc_write_cccd
 *
 * DESCRIPTION:GATTサーバーへのCCCD書き込み処理
 *
 * PARAMETERS:              Name                RW  Usage
 * esp_gatt_if_t            t_gatt_if           R   GATTクライアントインターフェース
 * uint16_t                 u16_con_id          R   コネクションID
 * uint16_t                 u16_char_handle     R   キャラクタリスティックハンドル
 * uint8_t                  u8_value            R   値
 * esp_gatt_write_type_t    e_write_type        R   書き込みタイプ
 * esp_gatt_auth_req_t      e_auth_req          R   認証リクエストタイプ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_gattc_write_cccd(esp_gatt_if_t t_gatt_if,
                                       uint16_t u16_con_id,
                                       uint16_t u16_char_handle,
                                       uint8_t u8_value,
                                       esp_gatt_write_type_t e_write_type,
                                       esp_gatt_auth_req_t e_auth_req) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // GATTサーバーへのCCCD書き込み処理
    //==========================================================================
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;

    do {
        // インターフェースステータス
        ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // コネクションステータス
        ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, u16_con_id);
        if (ps_con_sts == NULL) {
            break;
        }
        // 各サービス毎にCCCDに書き込み
        ts_gattc_svc_status_t* ps_svc_sts = ps_con_sts->ps_svc_sts;
        while (ps_svc_sts != NULL) {
            sts_val = sts_gattc_write_cccd(ps_svc_sts,
                                           u16_char_handle,
                                           u8_value,
                                           e_write_type,
                                           e_auth_req);
            if (sts_val != ESP_OK) {
                break;
            }
            // 次のサービス
            ps_svc_sts = ps_svc_sts->ps_next;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_gattc_rx_data
 *
 * DESCRIPTION:GATTサーバーからの受信データ取得処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_gatt_if_t        t_gatt_if       R   GATTクライアントインターフェース
 * uint16_t             u16_con_id      R   コネクションID
 * esp_gatt_id_t        s_svc_id        R   サービスID
 * TickType_t           t_tick          R   待ち時間
 *
 * RETURNS:
 *   ts_com_ble_gatt_rx_data*：受信データ
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatt_rx_data_t* ps_com_ble_gattc_rx_data(esp_gatt_if_t t_gatt_if,
                                                  uint16_t u16_con_id,
                                                  esp_gatt_id_t s_svc_id,
                                                  TickType_t t_tick) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // キューハンドルの取得
    //==========================================================================
    QueueHandle_t t_rx_queue = NULL;
    do {
        //----------------------------------------------------------------------
        // キューハンドルの取得
        //----------------------------------------------------------------------
        // インターフェースステータス
        ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス
        ts_gattc_svc_status_t* ps_svc_sts = ps_gattc_get_svc_status(ps_if_sts,
                                                                  u16_con_id,
                                                                  s_svc_id);
        // キューハンドラ
        if (ps_svc_sts != NULL) {
            t_rx_queue = ps_svc_sts->t_rx_queue;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 受信データ取得
    //==========================================================================
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    if (t_rx_queue != NULL){
        xQueueReceive(t_rx_queue, &ps_rx_data, t_tick);
    }

    // 結果返信
    return ps_rx_data;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_gattc_rx_clear
 *
 * DESCRIPTION:GATTサーバーからの受信バッファクリア処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_gatt_if_t        t_gatt_if       R   GATTクライアントインターフェース
 * uint16_t             u16_con_id      R   コネクションID
 * esp_gatt_id_t        s_svc_id        R   サービスID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
extern void v_com_ble_gattc_rx_clear(esp_gatt_if_t t_gatt_if,
                                      uint16_t u16_con_id,
                                      esp_gatt_id_t s_svc_id) {

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // キューの取得
    //==========================================================================
    QueueHandle_t t_rx_queue = NULL;
    do {
        // インターフェースステータス
        ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // サービスステータス
        ts_gattc_svc_status_t* ps_svc_sts = ps_gattc_get_svc_status(ps_if_sts,
                                                                  u16_con_id,
                                                                  s_svc_id);
        if (ps_svc_sts == NULL) {
            break;
        }
        // キューハンドラ
        t_rx_queue = ps_svc_sts->t_rx_queue;
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // データ取得
    //==========================================================================
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    if (t_rx_queue != NULL){
        while (xQueueReceive(t_rx_queue, &ps_rx_data, 0) == pdTRUE) {
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
        }
    }
}

/*******************************************************************************
 *
 * NAME: s_com_ble_spps_config
 *
 * DESCRIPTION:SPPサーバーのアプリケーション設定の生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_perm_t  t_perm_read     R   読み込み権限
 * esp_gatt_perm_t  t_perm_write    R   書き込み権限
 *
 * RETURNS:
 *   com_ble_gatts_app_config 生成されたSPPサーバーアプリケーション設定
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatts_if_config_t s_com_ble_spps_config(esp_gatt_perm_t t_perm_read, esp_gatt_perm_t t_perm_write) {
    // 初期値
    ts_com_ble_gatts_if_config_t t_cfg = s_gatts_cfg_default;
    // サービス情報
    t_cfg.u8_svc_cnt  = 1;
    t_cfg.ps_svc_cfg  = ps_com_ble_spps_create_svc(t_perm_read, t_perm_write);
    // イベントハンドラ
    t_cfg.fc_gatts_cb = v_spps_evt_cb;
    // 編集値を返却
    return t_cfg;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_spps_set_usr_cb
 *
 * DESCRIPTION:SPPサーバーのユーザーコールバック関数の設定処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatts_cb_t   fc_spps_cb      R   ユーザーコールバック関数
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_spps_set_usr_cb(esp_gatts_cb_t fc_spps_cb) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // ユーザーコールバック関数の設定処理
    //==========================================================================
    if (fc_spps_cb != NULL) {
        fc_spps_usr_evt_cb = fc_spps_cb;
    } else {
        fc_spps_usr_evt_cb = v_gatts_evt_dmy_cb;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_spps_create_svc
 *
 * DESCRIPTION:SPPサーバーのSPPサービス情報の生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_perm_t  t_perm_read     R   読み込み権限
 * esp_gatt_perm_t  t_perm_write    R   書き込み権限
 *
 * RETURNS:
 *   ts_com_ble_gatts_service* SPPサービス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatts_svc_config_t* ps_com_ble_spps_create_svc(esp_gatt_perm_t t_perm_read,
                                                        esp_gatt_perm_t t_perm_write) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 読み込み権限チェック
    uint16_t u16_mask = (ESP_GATT_PERM_WRITE |
                         ESP_GATT_PERM_WRITE_ENCRYPTED |
                         ESP_GATT_PERM_WRITE_ENC_MITM |
                         ESP_GATT_PERM_WRITE_SIGNED |
                         ESP_GATT_PERM_WRITE_SIGNED_MITM);
    if ((t_perm_read & u16_mask) != 0x00) {
        return NULL;
    }
    // 書き込み権限チェック
    u16_mask = (ESP_GATT_PERM_READ | ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_READ_ENC_MITM);
    if ((t_perm_write & u16_mask) != 0x00) {
        return NULL;
    }

    //==========================================================================
    // アトリビュートテーブルを生成
    //==========================================================================
    uint32_t u32_size = sizeof(esp_gatts_attr_db_t) * SPPS_ATTR_IDX_NB;
    // 動的にクローンを生成して返却
    esp_gatts_attr_db_t* ps_attr_db =
            (esp_gatts_attr_db_t*)pv_mem_clone((void*)s_spp_attr_db, u32_size);
    // 権限の書き換え
    esp_gatt_perm_t t_perm_rw = t_perm_read | t_perm_write;
    ps_attr_db[SPPS_ATTR_IDX_SVC].att_desc.perm          = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_RX_DATA_CHAR].att_desc.perm = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_RX_DATA_VAL].att_desc.perm  = t_perm_rw;
    ps_attr_db[SPPS_ATTR_IDX_TX_DATA_CHAR].att_desc.perm = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_TX_DATA_VAL].att_desc.perm  = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_TX_DATA_CFG].att_desc.perm  = t_perm_rw;
    ps_attr_db[SPPS_ATTR_IDX_RX_CMD_CHAR].att_desc.perm  = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_RX_CMD_VAL].att_desc.perm   = t_perm_rw;
    ps_attr_db[SPPS_ATTR_IDX_TX_STS_CHAR].att_desc.perm  = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_TX_STS_VAL].att_desc.perm   = t_perm_read;
    ps_attr_db[SPPS_ATTR_IDX_TX_STS_CFG].att_desc.perm   = t_perm_rw;

    // 認証権限定義の書き換え
    u16_mask = (ESP_GATT_PERM_WRITE_SIGNED | ESP_GATT_PERM_WRITE_SIGNED_MITM);
    if ((t_perm_write & u16_mask) != 0x00) {
        uint8_t* pu8_val = (uint8_t*)&s_spps_vals.u8_char_prop_rw_auth;
        ps_attr_db[SPPS_ATTR_IDX_RX_DATA_CHAR].att_desc.value = pu8_val;
        ps_attr_db[SPPS_ATTR_IDX_RX_CMD_CHAR].att_desc.value  = pu8_val;
    }

    //==========================================================================
    // アトリビュートテーブルを生成
    //==========================================================================
    // サービス生成
    ts_com_ble_gatts_svc_config_t* ps_service = pv_mem_malloc(sizeof(ts_com_ble_gatts_svc_config_t));
    ps_service->u8_inst_id     = BLE_SPPS_SVC_INST_IDX; // サービスインスタンスID
    ps_service->u8_max_nb_attr = SPPS_ATTR_IDX_NB;      // アトリビュート要素数
    ps_service->ps_attr_db     = ps_attr_db;            // アトリビュートテーブル

    // 編集結果を返却
    return ps_service;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_spps_delete_svc
 *
 * DESCRIPTION:SPPサーバーのSPPサービス情報の削除処理
 *
 * PARAMETERS:                  Name    RW  Usage
 * ts_com_ble_gatts_service_t*  ps_svc  R   SPPサービス情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
extern void v_com_ble_spps_delete_svc(ts_com_ble_gatts_svc_config_t* ps_svc) {
    // アトリビュートDBを解放
    l_mem_free(ps_svc[0].ps_attr_db);
    // SPPサービスの解放
    l_mem_free(ps_svc);
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_spps_tx_data
 *
 * DESCRIPTION:SPPサーバーのデータ送信処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint8_t*         pu8_data        R   送信データ
 * size_t           t_len           R   送信データ長
 *
 * RETURNS:
 *   ESP_OK:成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_spps_tx_data(esp_gatt_if_t t_gatt_if,
                                   uint8_t* pu8_data,
                                   size_t t_len) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Notify処理
    //==========================================================================
    // IFステータス
    esp_err_t sts_val = ESP_OK;
    do {
        ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // SPPステータス取得
        ts_spps_status_t* ps_spp_sts = ps_spps_get_status(t_gatt_if, BLE_SPPS_SVC_IDX);
        if (ps_spp_sts == NULL) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // CCCD判定
        if (!ps_spp_sts->b_notify_data) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // Indicate通知
        sts_val = sts_gatts_indication(t_gatt_if,
                                       BLE_SPPS_SVC_INST_IDX,
                                       ps_spp_sts->u16_hndl_data_ntf,
                                       pu8_data,
                                       t_len,
                                       false);
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: s_com_ble_sppc_config
 *
 * DESCRIPTION:GATTクライアントのSPPアプリケーション設定の生成処理
 *
 * PARAMETERS:                  Name        RW  Usage
 *
 * RETURNS:
 *   ts_com_ble_gattc_app_config:SPPアプリケーション設定
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gattc_if_config_t s_com_ble_sppc_config() {
    // GATTアプリケーション設定
    ts_com_ble_gattc_if_config_t s_gattc_app_config = s_gattc_if_cfg_default;
    // サービス数
    s_gattc_app_config.u8_svc_cnt = 1;
    // サービスのUUID
    s_gattc_app_config.pt_svc_uuid = pv_mem_malloc(sizeof(esp_bt_uuid_t));
    s_gattc_app_config.pt_svc_uuid[0] = s_spp_service_uuid;
    // インターフェース毎のコールバック関数
    s_gattc_app_config.fc_gattc_cb = v_sppc_evt_cb;
    // 編集結果を返信
    return s_gattc_app_config;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_sppc_set_usr_cb
 *
 * DESCRIPTION:SPPクライアントのユーザーコールバック関数の設定処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gattc_cb_t   fc_sppc_cb      R   ユーザーコールバック関数
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_sppc_set_usr_cb(esp_gattc_cb_t fc_sppc_cb) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // ユーザーコールバック関数の設定処理
    //==========================================================================
    if (fc_sppc_cb != NULL) {
        fc_sppc_usr_evt_cb = fc_sppc_cb;
    } else {
        fc_sppc_usr_evt_cb = v_gattc_evt_dmy_cb;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: e_com_ble_sppc_con_sts
 *
 * DESCRIPTION:SPP接続ステータスの取得処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gattc_svc_info_t* ps_con      R   コネクション情報
 *
 * RETURNS:
 *   te_com_ble_spp_connection_sts_t:現在の接続ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_com_ble_spp_connection_sts_t e_com_ble_sppc_con_sts(ts_com_ble_gattc_con_info_t* ps_con) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return COM_BLE_SPP_CON_ERROR;
    }

    //==========================================================================
    // 接続ステータスの取得
    //==========================================================================
    te_com_ble_spp_connection_sts_t e_con_sts = COM_BLE_SPP_CON_ERROR;
    if (ps_con != NULL) {
        e_con_sts = e_sppc_con_sts(ps_con->t_gatt_if, ps_con->u16_con_id);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return e_con_sts;
}

/*******************************************************************************
 *
 * NAME: sts_com_ble_sppc_tx_data
 *
 * DESCRIPTION:SPPクライアントからのデータ送信処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gattc_svc_info_t* ps_con      R   コネクション情報
 * uint8_t*                     pu8_data    R   送信データ配列
 * size_t                       t_len       R   送信データサイズ
 *
 * RETURNS:
 *   ESP_OK:成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_ble_sppc_tx_data(ts_com_ble_gattc_con_info_t* ps_con, uint8_t* pu8_data, size_t t_len) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (t_len == 0) {
        return ESP_OK;
    }
    if (ps_con == NULL || pu8_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // データの書き込み処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // SPPステータス取得
        ts_sppc_status_t* ps_spp_sts = ps_sppc_get_status(ps_con->t_gatt_if, ps_con->u16_con_id);
        if (ps_spp_sts == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // データの送信処理
        esp_err_t sts_val = ESP_OK;
        uint32_t u32_len = t_len;
        uint32_t u32_pos = 0;
        while (u32_len > COM_BLE_GATT_DATA_LEN_MAX) {
            sts_val = esp_ble_gattc_write_char(ps_con->t_gatt_if,
                                               ps_con->u16_con_id,
                                               ps_spp_sts->u16_hndl_tx_data,
                                               COM_BLE_GATT_DATA_LEN_MAX,
                                               &pu8_data[u32_pos],
                                               ESP_GATT_WRITE_TYPE_RSP,
                                               ps_con->e_sec_auth_req);
            if (sts_val != ESP_OK) {
                break;
            }
            // データ長を更新
            u32_len -= COM_BLE_GATT_DATA_LEN_MAX;
            // ポジションを更新
            u32_pos += COM_BLE_GATT_DATA_LEN_MAX;
        }
        // 終端データの書き込み
        if (u32_len > 0) {
            sts_val = esp_ble_gattc_write_char(ps_con->t_gatt_if,
                                               ps_con->u16_con_id,
                                               ps_spp_sts->u16_hndl_tx_data,
                                               u32_len,
                                               &pu8_data[u32_pos],
                                               ESP_GATT_WRITE_TYPE_RSP,
                                               ps_con->e_sec_auth_req);
        }
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_ble_sppc_rx_data
 *
 * DESCRIPTION:SPPクライアントからのデータ受信処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gattc_con_info_t* ps_con      R   コネクション情報
 * TickType_t                   t_tick      R   待ち時間
 *
 * RETURNS:
 * ts_com_ble_gatt_rx_data*:受信データ
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_ble_gatt_rx_data_t* ps_com_ble_sppc_rx_data(ts_com_ble_gattc_con_info_t* ps_con,
                                                   TickType_t t_tick) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // キューハンドルの取得
    //==========================================================================
    QueueHandle_t t_rx_queue = NULL;
    do {
        // SPPステータス取得
        ts_sppc_status_t* ps_spp_sts = ps_sppc_get_status(ps_con->t_gatt_if, ps_con->u16_con_id);
        if (ps_spp_sts == NULL) {
            break;
        }
        // サービスステータス取得
        ts_gattc_svc_status_t* ps_svc_sts = ps_spp_sts->ps_svc_sts;
        if (ps_svc_sts != NULL) {
            t_rx_queue = ps_svc_sts->t_rx_queue;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // SPPクライアントからのデータ受信処理
    //==========================================================================
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    if (t_rx_queue != NULL) {
        xQueueReceive(t_rx_queue, &ps_rx_data, t_tick);
    }

    // 結果返信
    return ps_rx_data;
}

/*******************************************************************************
 *
 * NAME: v_com_ble_sppc_rx_clear
 *
 * DESCRIPTION:SPPクライアントからの受信バッファクリア処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gattc_con_info_t* ps_con      R   コネクション情報
 *
 * RETURNS:
 *   ts_com_ble_gatt_rx_data*:受信データ
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_ble_sppc_rx_clear(ts_com_ble_gattc_con_info_t* ps_con) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // GATTサーバーへのCCCD書き込み処理
    //==========================================================================
    do {
        // SPPステータス取得
        ts_sppc_status_t* ps_spp_sts = ps_sppc_get_status(ps_con->t_gatt_if, ps_con->u16_con_id);
        if (ps_spp_sts == NULL) {
            // サービス情報Aの削除
            break;
        }
        // サービスステータス取得
        ts_gattc_svc_status_t* ps_svc_sts = ps_spp_sts->ps_svc_sts;
        if (ps_svc_sts == NULL) {
            break;
        }
        //----------------------------------------------------------------------
        // キュークリア
        //----------------------------------------------------------------------
        if (ps_svc_sts->t_rx_queue == NULL) {
            break;
        }
        ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
        while (xQueueReceive(ps_svc_sts->t_rx_queue, &ps_rx_data, 0) == pdTRUE) {
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_com_disconnect
 *
 * DESCRIPTION:物理接続の切断
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   対象のアドレス
 *
 * RETURNS:
 *   結果ステータス：ESP_OK
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_com_disconnect(esp_bd_addr_t t_bda) {
    //==========================================================================
    // GAPデバイス情報探索
    //==========================================================================
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        if (l_com_ble_addr_cmp(t_bda, ps_device->t_bda) == 0) {
            // 切断判定
            if ((ps_device->u16_status & GAP_DEV_STS_DISCONNECTING) != 0x00) {
                // 結果返信
                return ESP_OK;
            }
            // 切断中にステータスを更新
            ps_device->u16_status |= GAP_DEV_STS_DISCONNECTING;
            // 物理的な切断
            return esp_ble_gap_disconnect(t_bda);
        }
        // 次のデバイスへ
        ps_device = ps_device->ps_next;
    }

    // 結果返信
    return ESP_ERR_INVALID_STATE;
}

/*******************************************************************************
 *
 * NAME: v_gap_event_cb
 *
 * DESCRIPTION:GAPプロファイルのアドバタイザ側のイベントハンドラ関数
 *
 * PARAMETERS:                Name          RW  Usage
 *   esp_gap_ble_cb_event_t   e_event       R   GAPイベントタイプ
 *   esp_ble_gap_cb_param_t*  pu_param      R   GAPコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // イベント毎の処理
    //==========================================================================
    // 受信アドレス
    esp_bd_addr_t* pt_bda = NULL;
    // GAP設定
    ts_com_ble_gap_config_t* ps_config = &s_gap_ctrl.s_config;
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // GAPデバイス
    ts_gap_device_t* ps_device = NULL;
#ifdef COM_BLE_DEBUG
    // BLEアドレス文字列
    tc_com_ble_bda_string_t tc_bda;
#endif
    // イベント判定
    switch (e_event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // アドバタイズパラメータの設定完了通知イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
#endif
        // ステータスの設定中フラグをオフにする
        ps_status->u32_status &= ~GAP_STS_EXEC_CONFIG_ADVERTISE;
        // アドバタイズデータの設定完了通知イベント
        if (pu_param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        // ステータスを設定済みに更新
        ps_status->u32_status |= GAP_STS_SET_CONFIG_ADVERTISE;
        // 実行待ちの場合にはアドバタイズ実行開始処理
        sts_gap_start_advertise_step_2();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // スキャン応答パラメータの設定完了通知イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
#endif
        // ステータスの設定中フラグをオフにする
        ps_status->u32_status &= ~GAP_STS_EXEC_CONFIG_SCAN_RSP;
        // スキャン応答データの設定完了通知イベント
        if (pu_param->scan_rsp_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        // ステータスを設定済みに更新
        ps_status->u32_status |= GAP_STS_SET_CONFIG_SCAN_RSP;
        // 実行待ちの場合にはアドバタイズ実行開始処理
        sts_gap_start_advertise_step_2();
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // スキャンパラメータの設定完了通知イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
#endif
        // ステータスの設定中フラグをオフにする
        ps_status->u32_status &= ~GAP_STS_EXEC_CONFIG_SCAN;
        // ステータス判定
        if (pu_param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        // ステータスを設定済みに更新
        ps_status->u32_status |= GAP_STS_SET_CONFIG_SCAN;
        // 実行待ちの場合にはスキャン実行開始処理
        sts_gap_start_scan_step_2();
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        // ピアデバイスからのスキャン結果通知イベント
        // 1つのスキャン結果が揃う度に起きる通知イベント
#ifdef COM_BLE_DEBUG
        v_com_ble_address_to_str(tc_bda, pu_param->scan_rst.bda);
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SCAN_RESULT_EVT");
        ESP_LOGI(LOG_TAG, "GAP_EVT:searched Adv Data Len %d, Scan Response Len %d", pu_param->scan_rst.adv_data_len, pu_param->scan_rst.scan_rsp_len);
        ESP_LOGI(LOG_TAG, "GAP_EVT:search_evt = %d dev_type = %d", pu_param->scan_rst.search_evt, pu_param->scan_rst.dev_type);
        ESP_LOGI(LOG_TAG, "GAP_EVT:bda = %s addr_type = %d", tc_bda, pu_param->scan_rst.ble_addr_type);
#endif
        if (pu_param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            // スキャン結果以外の場合は対象外
            break;
        }
        // デバイスタイプの判定
        if (pu_param->scan_rst.dev_type != ESP_BT_DEVICE_TYPE_BLE) {
            // BLE以外のデバイスの場合は対象外
            break;
        }
        // デバイス情報の取得
        ps_device = ps_gap_create_device(&pu_param->scan_rst);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // アドバタイズ開始の完了通知イベント
        // このタイミングまでにパスキーを設定する必要がある
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
#endif
        // advertising start complete event to indicate advertising start successfully or failed
        if (pu_param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ps_status->u32_status |= GAP_STS_EXEC_ADVERTISING;
            ps_status->u32_status &= ~GAP_STS_WAIT_ADVERTISING;
        } else {
            // アドバタイズ開始失敗
            ps_status->u32_status &= ~(GAP_STS_WAIT_ADVERTISING | GAP_STS_EXEC_ADVERTISING);
#ifdef COM_BLE_DEBUG
            ESP_LOGE(LOG_TAG, "GAP_EVT:advertising start failed. status = 0x%04x", pu_param->adv_start_cmpl.status);
#endif
        }
        // 接続情報リセット
        v_gap_minimize_device_list();
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // スキャン開始通知
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
#endif
        //scan start complete event to indicate scan start successfully or failed
        if (pu_param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            // スキャン開始失敗
            ps_status->u32_status &= ~GAP_STS_START_SCAN;
#ifdef COM_BLE_DEBUG
            ESP_LOGE(LOG_TAG, "GAP_EVT:scan start failed. status = 0x%04x", pu_param->scan_start_cmpl.status);
#endif
        }
        // 接続情報リセット
        v_gap_minimize_device_list();
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        // 認証完了通知
        // ※アドバタイザとスキャナの両方で起きるイベント
        // すべき事：認証完了確認
#ifdef COM_BLE_DEBUG
        v_com_ble_address_to_str(tc_bda, pu_param->ble_security.auth_cmpl.bd_addr);
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_AUTH_CMPL_EVT");
        ESP_LOGI(LOG_TAG, "GAP_EVT:address = %s", tc_bda);
        ESP_LOGI(LOG_TAG, "GAP_EVT:address type = %d", pu_param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(LOG_TAG, "GAP_EVT:pair status = %s", pu_param->ble_security.auth_cmpl.success ? "success" : "fail");
        ESP_LOGI(LOG_TAG, "GAP_EVT:fail reason = %x", pu_param->ble_security.auth_cmpl.fail_reason);
        ESP_LOGI(LOG_TAG, "GAP_EVT:device type = %x", pu_param->ble_security.auth_cmpl.dev_type);
        ESP_LOGI(LOG_TAG, "GAP_EVT:auth mode = %s",pc_com_ble_auth_req_to_str(pu_param->ble_security.auth_cmpl.auth_mode));
#endif
        // ステータスを更新
        ps_status->u32_status &= ~GAP_STS_EXEC_BONDING;
        // デバイスアドレス
        pt_bda = &pu_param->ble_security.auth_cmpl.bd_addr;
        // 認証済みデバイス情報取得
        ps_device = ps_gap_add_device(*pt_bda);
        if (ps_device == NULL) {
            break;
        }
        // デバイスステータス更新
        ps_device->u16_status &= ~GAP_DEV_STS_AUTH;
        // 認証結果判定
        if (!pu_param->ble_security.auth_cmpl.success) {
#ifdef COM_BLE_DEBUG
            // エラーメッセージ
            ESP_LOGE(LOG_TAG, "GAP_EVT:authentication completion error reason = 0x%x", pu_param->ble_security.auth_cmpl.fail_reason);
#endif
            // コネクションの切断
            sts_com_disconnect(*pt_bda);
            // ボンディングデバイスの削除処理
            esp_ble_remove_bond_device(*pt_bda);
            break;
        }
        // 認証済み
        ps_device->u16_status |= GAP_DEV_STS_AUTHENTICATED;
        ps_device->t_auth_mode = pu_param->ble_security.auth_cmpl.auth_mode;
        // GATTサービス検索
        sts_gattc_search_service(*pt_bda);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        // ボンディングのキー交換イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_KEY_EVT");
#endif
        // ステータスを更新
        s_gap_ctrl.s_status.u32_status |= GAP_STS_EXEC_BONDING;
        // ピアデバイスとのBLEキー情報共有によるキータイプをログ出力
        //shows the ble key info share with peer device to the user.
#ifdef COM_BLE_DEBUG
        ESP_LOGI(LOG_TAG, "GAP_EVT:key type = %s", pc_com_ble_key_type_to_str(pu_param->ble_security.ble_key.key_type));
#endif
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        // セキュリティリクエスト通知イベント ※スキャン側にもある
        // すべき事：接続開始要求に対する返信
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SEC_REQ_EVT");
#endif
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should sent the security response with negative(false) accept value */
        // セキュリティレスポンス
        esp_ble_gap_security_rsp(pu_param->ble_security.ble_req.bd_addr, true);
        break;
#ifdef COM_BLE_DEBUG
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        // the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        // パスキーの表示イベント
        // ピアデバイスに入力するパスキーを表示する
        ///show the passkey number to the user to input it in the peer deivce.
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SEC_REQ_EVT");
        ESP_LOGI(LOG_TAG, "GAP_EVT:The passkey Notify number:%06ld", pu_param->ble_security.key_notif.passkey);
        break;
#endif
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // パスキーの返信要求イベント通知
        // 設定済みの場合にはパスキーを返信
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_PASSKEY_REQ_EVT");
#endif
        // 対象デバイス情報取得
        ps_device = ps_gap_add_device(pu_param->ble_security.ble_req.bd_addr);
        if (ps_device == NULL) {
            break;
        }
        // ステータス更新
        ps_device->u16_status |= GAP_DEV_STS_REQ_PASSKEY;
        break;
#ifdef COM_BLE_DEBUG
    case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
        // アウトオブバンドの要求イベント
        // ※ペアリング時の通信の一部をBluetooth以外でやり取りする際のデータ要求イベント
        // すべき事：アウトオブバウンドによる認証時は、通知されたトークンを応答
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
#endif
    case ESP_GAP_BLE_NC_REQ_EVT:
        // パスキーの確認要求 ※スキャン側にもある
        // すべき事：パスキー認証の場合には、相手デバイスに表示されているパスキーを確認して返信する
        // IOにDisplayYesNO機能があり、ピアデバイスIOにもDisplayYesNo機能がある場合、アプリはこのevtを受け取ります。
        // パスキーをユーザーに表示し、ピアデバイスにて表示される番号と一致するか確認した結果を返信する
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer deivce. */
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_NC_REQ_EVT");
#endif
        // 対象デバイス情報取得
        ps_device = ps_gap_add_device(pu_param->ble_security.key_notif.bd_addr);
        if (ps_device == NULL) {
            break;
        }
        // 番号確認要求に対応
        ps_device->u16_status |= GAP_DEV_STS_REQ_NUM_CHK;
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        // アドバタイズ停止通知イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
#endif
        if (pu_param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ps_status->u32_status &= ~(GAP_STS_WAIT_ADVERTISING | GAP_STS_EXEC_ADVERTISING);
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        // スキャン停止通知イベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
#endif
        if (pu_param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ps_status->u32_status &= ~GAP_STS_START_SCAN;
        }
        break;
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        // ローカルデバイス上でプライバシー機能（ランダムアドレス機能）の有効化または無効化が完了した事の通知イベント
        // ※アドバタイズ側とスキャン側の両方のイベント
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT");
#endif
        // ステータス更新
        ps_status->u32_status &= ~GAP_STS_EXEC_CONFIG_PRIVACY;
        // 結果ステータス判定
        if (pu_param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
#ifdef COM_BLE_DEBUG
            ESP_LOGE(LOG_TAG, "GAP_EVT:config local privacy failed, error status = 0x%04x", pu_param->local_privacy_cmpl.status);
#endif
            break;
        }
        // ステータスをローカルプライバシー設定済みに変更
        ps_status->u32_status |= GAP_STS_SET_CONFIG_PRIVACY;
        //======================================================================
        // スキャンパラメータ、スキャン応答パラメータ、アドバタイズパラメータの設定
        // イベント：ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVTでアドバタイズ開始判定
        // イベント：ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVTでアドバタイズ開始判定
        // イベント：ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVTでスキャン開始判定
        //======================================================================
        // アドバタイズパラメータの設定を開始
#ifdef COM_BLE_DEBUG
        ESP_LOGI(LOG_TAG, "start_advertising addr_type = 0x%02x", ps_status->s_adv_params.own_addr_type);
#endif
        sts_gap_start_advertise_step_1();
        // スキャンパラメータの設定開始
        sts_gap_start_scan_step_1();
        break;
#ifdef COM_BLE_DEBUG
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
        // ボンディングデバイスの削除完了通知イベント
        // セキュリティデータベースからボンディングしたピアデバイスの情報を削除した際の完了通知イベント
        v_com_ble_address_to_str(tc_bda, pu_param->remove_bond_dev_cmpl.bd_addr);
        ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT");
        ESP_LOGI(LOG_TAG, "GAP_EVT:status = %d", pu_param->remove_bond_dev_cmpl.status);
        ESP_LOGI(LOG_TAG, "GAP_EVT:bda = %s", tc_bda);
        break;
#endif
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        // RSSIの測定完了通知
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT");
#endif
        ps_device = ps_gap_get_device(pu_param->read_rssi_cmpl.remote_addr);
        if (ps_device != NULL) {
            // ステータス更新
            ps_device->u16_status &= ~GAP_DEV_STS_EXEC_RSSI;
            ps_device->u16_status |= GAP_DEV_STS_SET_RSSI;
            // RSSI更新
            ps_device->i_rssi = pu_param->read_rssi_cmpl.rssi;
        }
        break;
    default:
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "GAP_EVT=%d", e_event);
#endif
        break;
    }

    //==========================================================================
    // ユーザーイベント処理
    //==========================================================================
    if (ps_config->v_callback != NULL) {
        ps_config->v_callback(e_event, pu_param);
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: ps_gap_add_device
 *
 * DESCRIPTION:GAPプロファイルのデバイス情報追加
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   探索対象のアドレス
 *
 * RETURNS:
 *   ts_com_ble_gap_device*:デバイス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gap_device_t* ps_gap_add_device(esp_bd_addr_t t_bda) {
    // デバイス情報探索
    ts_gap_device_t* ps_before = NULL;
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        if (l_com_ble_addr_cmp(t_bda, ps_device->t_bda) == 0) {
            // 発見したデバイスを返却
            return ps_device;
        }
        // 次のデバイスへ
        ps_before = ps_device;
        ps_device = ps_device->ps_next;
    }
    // デバイス情報を生成してリストに追加
    ps_device = pv_mem_malloc(sizeof(ts_gap_device_t));
    if (ps_device == NULL) {
        return NULL;
    }
    *ps_device = s_gap_dev_default;
    ps_device->u16_status = GAP_DEV_STS_SET_ADDRESS;
    v_com_ble_addr_cpy(ps_device->t_bda, t_bda);
    // リンクに追加
    if (ps_before != NULL) {
        ps_before->ps_next = ps_device;
    } else {
        s_gap_ctrl.ps_device = ps_device;
    }
    s_gap_ctrl.u16_dev_cnt++;
    // 生成したデバイス情報を返却
    return ps_device;
}

/*******************************************************************************
 *
 * NAME: ps_gap_get_device
 *
 * DESCRIPTION:GAPプロファイルのデバイス情報検索
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   探索対象のアドレス
 *
 * RETURNS:
 *   ts_com_ble_gap_device*:デバイス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gap_device_t* ps_gap_get_device(esp_bd_addr_t t_bda) {
    // デバイス情報探索
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        if (l_com_ble_addr_cmp(t_bda, ps_device->t_bda) == 0) {
            // 発見したデバイスを返却
            return ps_device;
        }
        // 次のデバイスへ
        ps_device = ps_device->ps_next;
    }
    // 検索結果無し
    return NULL;
}

/*******************************************************************************
 *
 * NAME: ps_gap_create_device
 *
 * DESCRIPTION:GAPプロファイルのデバイス情報生成
 *
 * PARAMETERS:                  Name        RW  Usage
 * ble_scan_result_evt_param*   ps_param    R   検索結果
 *
 * RETURNS:
 *   ts_com_ble_gap_device*:デバイス情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gap_device_t* ps_gap_create_device(struct ble_scan_result_evt_param* ps_param) {
    //==========================================================================
    // 検索条件の取得
    //==========================================================================
    // アドレス
    esp_bd_addr_t* pt_addr = &ps_param->bda;
    // スキャン結果データからデバイス名取得
    uint8_t u8_name_len = 0;
    uint8_t* pu8_name = esp_ble_resolve_adv_data(ps_param->ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &u8_name_len);
    if (u8_name_len == 0) {
        return NULL;
    }
    char* pc_name = pv_mem_malloc(u8_name_len + 1);
    if (pc_name == NULL) {
        return NULL;
    }
    memcpy(pc_name, pu8_name, u8_name_len);
    pc_name[u8_name_len] = '\0';

    //==========================================================================
    // デバイス情報検索
    //==========================================================================
    // デバイス情報探索
    ts_gap_device_t* ps_before = NULL;
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        // アドレス判定
        if (l_com_ble_addr_cmp(*pt_addr, ps_device->t_bda) == 0) {
            break;
        }
        // 次のデバイスを対象とする
        ps_before = ps_device;
        ps_device = ps_device->ps_next;
    }

    //==========================================================================
    // デバイス情報を生成
    //==========================================================================
    // 探索結果判定
    if (ps_device == NULL) {
        // デバイス情報の生成
        ps_device = pv_mem_malloc(sizeof(ts_gap_device_t));
        if (ps_device == NULL) {
            l_mem_free(pc_name);
            return NULL;
        }
        // デバイス情報を初期化
        *ps_device = s_gap_dev_default;
        // 生成したデバイス情報を設定
        if (ps_before != NULL) {
            ps_before->ps_next = ps_device;
        } else {
            s_gap_ctrl.ps_device = ps_device;
        }
        s_gap_ctrl.u16_dev_cnt++;
    }
    // デバイス情報を更新
    ps_device->u16_status |= (GAP_DEV_STS_SET_ADDRESS | GAP_DEV_STS_SET_NAME | GAP_DEV_STS_SET_RSSI);
    ps_device->e_addr_type = ps_param->ble_addr_type;
    v_com_ble_addr_cpy(ps_device->t_bda, ps_param->bda);
    if (ps_device->pc_name != NULL) {
        l_mem_free(ps_device->pc_name);
    }
    ps_device->pc_name = pc_name;
    ps_device->i_rssi  = ps_param->rssi;
    ps_device->t_auth_mode = s_gap_ctrl.s_config.t_auth_req;

    // 変数済みのデバイス情報を返却
    return ps_device;
}

/*******************************************************************************
 *
 * NAME: sts_gap_del_device
 *
 * DESCRIPTION:GAPプロファイルのデバイス情報削除
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_bd_addr_t    t_bda       R   探索対象のアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_del_device(esp_bd_addr_t t_bda) {
    //==========================================================================
    // デバイス情報探索
    //==========================================================================
    ts_gap_device_t s_device_dmy = {
        .ps_next = s_gap_ctrl.ps_device
    };
    ts_gap_device_t* ps_device_bef = &s_device_dmy;
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        if (l_com_ble_addr_cmp(t_bda, ps_device->t_bda) == 0) {
            break;
        }
        // 次のデバイスへ
        ps_device_bef = ps_device;
        ps_device = ps_device->ps_next;
    }
    // 探索結果判定
    if (ps_device == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    //==========================================================================
    // 対象デバイスの削除
    //==========================================================================
    // 対象デバイスをリンクリストから切り離す
    ps_device_bef->ps_next = ps_device->ps_next;
    ps_device->ps_next = NULL;
    s_gap_ctrl.u16_dev_cnt--;
    s_gap_ctrl.ps_device = s_device_dmy.ps_next;
    // デバイス名の解放
    l_mem_free(ps_device->pc_name);
    // デバイス情報自体の解放
    l_mem_free(ps_device);

    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_gap_minimize_device_list
 *
 * DESCRIPTION:GAPプロファイルのデバイス情報リフレッシュ処理
 * 認証中か認証済み以外のデバイス情報を解放する。
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gap_minimize_device_list() {
    //==========================================================================
    // デバイス情報リフレッシュ
    //==========================================================================
    // デバイス情報探索
    ts_gap_device_t* ps_delete = NULL;
    ts_gap_device_t* ps_before = NULL;
    ts_gap_device_t* ps_device = s_gap_ctrl.ps_device;
    while (ps_device != NULL) {
        // ステータス
        if ((ps_device->u16_status & GAP_DEV_STS_AUTH) != 0x00) {
            ps_before = ps_device;
            ps_device = ps_device->ps_next;
            continue;
        }
        // デバイス情報のリンクを更新
        if (ps_before != NULL) {
            ps_before->ps_next = ps_device->ps_next;
        } else {
            s_gap_ctrl.ps_device = ps_device->ps_next;
        }
        // 次のデバイスを対象とする
        ps_delete = ps_device;
        ps_device = ps_device->ps_next;
        // デバイス名を解放
        if (ps_delete->pc_name != NULL) {
            l_mem_free(ps_delete->pc_name);
        }
        // デバイス情報を解放
        l_mem_free(ps_delete);
        s_gap_ctrl.u16_dev_cnt--;
    }
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_advertise
 *
 * DESCRIPTION:GAPプロファイルのアドバタイズ開始処理
 *
 * PARAMETERS:              Name            RW  Usage
 * esp_ble_adv_params_t*    ps_adv_params   R   アドバタイズパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_advertise(esp_ble_adv_params_t* ps_adv_params) {
    //==========================================================================
    // 実行可否を判定
    //==========================================================================
    // パラメータチェック
    if (ps_adv_params == NULL) {
        // エラーステータス返却
        return ESP_ERR_INVALID_ARG;
    }
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 次が実行中の場合にはエラー
    // ローカルプライバシーモードの設定
    // アドバタイズパラメータの設定
    // スキャン応答パラメータの設定
    // アドバタイズ
    if ((ps_status->u32_status & GAP_STS_CHK_EXEC_ADVERTISE) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // ADV_TYPE_IND：コネクション可能、スキャン可能
    // ADV_TYPE_DIRECT_IND_HIGH：コネクション可能、スキャン不可能、デューティサイクル高
    // ADV_TYPE_SCAN_IND：コネクション不可能、スキャン可能
    // ADV_TYPE_NONCONN_IND：コネクション不可能、スキャン不可能
    // ADV_TYPE_DIRECT_IND_LOW：コネクション可能、スキャン不可能、デューティサイクル低
    esp_ble_adv_type_t e_adv_type = ps_status->s_adv_params.adv_type;
    uint32_t u32_chk_mask = GAP_STS_WAIT_CONFIG_ADVERTISE;
    if (e_adv_type == ADV_TYPE_IND || e_adv_type == ADV_TYPE_SCAN_IND) {
        u32_chk_mask |= GAP_STS_WAIT_CONFIG_SCAN_RSP;
    }
    // 必要なパラメータが未設定な場合にはエラー
    if ((ps_status->u32_status & u32_chk_mask) != u32_chk_mask) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // プライバシー機能の設定開始
    //==========================================================================
    // パラメータ設定
    s_gap_ctrl.s_status.s_adv_params = *ps_adv_params;
    // プライバシー機能を有効化
    esp_err_t sts_val = sts_gap_config_local_privacy(ps_status->s_adv_params.own_addr_type);
    if (sts_val == ESP_OK) {
        // ステータを実行中に更新
        ps_status->u32_status |= (GAP_STS_WAIT_ADVERTISING | GAP_STS_EXEC_CONFIG_PRIVACY);
    }
    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_advertise_step_0
 *
 * DESCRIPTION:GAPプロファイルのアドバタイズ開始処理（ローカルプライバシーモード設定）
 *
 * PARAMETERS:                Name          RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_advertise_step_0() {
    //==========================================================================
    // 実行可否を判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 次が実行中の場合にはエラー
    // ローカルプライバシーモードの設定
    // アドバタイズパラメータの設定
    // スキャン応答パラメータの設定
    // アドバタイズ
    if ((ps_status->u32_status & GAP_STS_CHK_EXEC_ADVERTISE) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // 実行時のパラメータ判定
    // ADV_TYPE_IND：コネクション可能、スキャン可能
    // ADV_TYPE_DIRECT_IND_HIGH：コネクション可能、スキャン不可能、デューティサイクル高
    // ADV_TYPE_SCAN_IND：コネクション不可能、スキャン可能
    // ADV_TYPE_NONCONN_IND：コネクション不可能、スキャン不可能
    // ADV_TYPE_DIRECT_IND_LOW：コネクション可能、スキャン不可能、デューティサイクル低
    esp_ble_adv_type_t e_adv_type = ps_status->s_adv_params.adv_type;
    uint32_t u32_chk_mask = GAP_STS_WAIT_CONFIG_ADVERTISE;
    if (e_adv_type == ADV_TYPE_IND || e_adv_type == ADV_TYPE_SCAN_IND) {
        u32_chk_mask |= GAP_STS_WAIT_CONFIG_SCAN_RSP;
    }
    // 必要なパラメータが未設定な場合にはエラー
    if ((ps_status->u32_status & u32_chk_mask) != u32_chk_mask) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // プライバシー機能の設定開始
    //==========================================================================
    // プライバシー機能を有効化
    esp_err_t sts_val = sts_gap_config_local_privacy(ps_status->s_adv_params.own_addr_type);
    if (sts_val == ESP_OK) {
        // ステータを実行中に更新
        ps_status->u32_status |= (GAP_STS_WAIT_ADVERTISING | GAP_STS_EXEC_CONFIG_PRIVACY);
    }
    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_advertise_step_1
 *
 * DESCRIPTION:GAPプロファイルのアドバタイズ開始処理（パラメータ設定）
 *
 * PARAMETERS:                Name          RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_advertise_step_1() {
    //==========================================================================
    // パラメータ設定の実行判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 次が実行中の場合にはエラー
    // ローカルプライバシーモードの設定
    // アドバタイズパラメータの設定
    // スキャン応答パラメータの設定
    // アドバタイズ
    if ((ps_status->u32_status & GAP_STS_CHK_EXEC_ADVERTISE) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // アドバタイズ実行待ちか判定
    if ((ps_status->u32_status & GAP_STS_WAIT_ADVERTISING) == 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // パラメータ設定
    //==========================================================================
    // アドバタイズデータ設定
    esp_err_t sts_val = esp_ble_gap_config_adv_data(&ps_status->s_adv_config);
    if (sts_val == ESP_OK) {
        // ステータスを実行中に更新
        ps_status->u32_status |= GAP_STS_EXEC_CONFIG_ADVERTISE;
    }
    // スキャン応答データの設定を判定
    // ADV_TYPE_IND：コネクション可能、スキャン可能
    // ADV_TYPE_DIRECT_IND_HIGH：コネクション可能、スキャン不可能、デューティサイクル高
    // ADV_TYPE_SCAN_IND：コネクション不可能、スキャン可能
    // ADV_TYPE_NONCONN_IND：コネクション不可能、スキャン不可能
    // ADV_TYPE_DIRECT_IND_LOW：コネクション可能、スキャン不可能、デューティサイクル低
    esp_ble_adv_type_t e_adv_type = ps_status->s_adv_params.adv_type;
    if (e_adv_type == ADV_TYPE_IND || e_adv_type == ADV_TYPE_SCAN_IND) {
        // スキャン応答データを設定
        sts_val = esp_ble_gap_config_adv_data(&ps_status->s_scan_rsp_config);
        if (sts_val == ESP_OK) {
            // ステータスを実行中に更新
            ps_status->u32_status |= GAP_STS_EXEC_CONFIG_SCAN_RSP;
        }
    }
    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_advertise_step_2
 *
 * DESCRIPTION:GAPプロファイルのアドバタイズ開始処理（アドバタイズ開始）
 *
 * PARAMETERS:                Name          RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_advertise_step_2() {
    //==========================================================================
    // アドバタイズ実行判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 次が実行中の場合にはエラー
    // ローカルプライバシーモードの設定
    // アドバタイズパラメータの設定
    // スキャン応答パラメータの設定
    // アドバタイズ
    if ((ps_status->u32_status & GAP_STS_CHK_EXEC_ADVERTISE) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // アドバタイズ実行待ちか判定
    if ((ps_status->u32_status & GAP_STS_WAIT_ADVERTISING) == 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // アドバタイズの開始処理
    //==========================================================================
    esp_err_t sts_val = esp_ble_gap_start_advertising(&ps_status->s_adv_params);
    if (sts_val == ESP_OK) {
        // アドバタイズ実行中に更新
        ps_status->u32_status |= GAP_STS_EXEC_ADVERTISING;
    }
    // アドバタイズ待ちの状態をクリア
    ps_status->u32_status &= ~GAP_STS_WAIT_ADVERTISING;
    // 結果ステータスを返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_scan
 *
 * DESCRIPTION:GAPプロファイルのスキャン開始処理
 *
 * PARAMETERS:      Name            RW  Usage
 * uint32_t         u32_duration    R   スキャン実行時間
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_scan(uint32_t u32_duration) {
    //==========================================================================
    // スキャンステータスの更新処理　※タイムアウトによる更新
    //==========================================================================
    sts_gap_update_scan_status();

    //==========================================================================
    // 実行可否を判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // スキャン起動中の場合にはエラー
    if ((ps_status->u32_status & GAP_STS_START_SCAN) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // スキャンパラメータが未設定な場合にはエラー
    if ((ps_status->u32_status & GAP_STS_SET_SCAN_CFG) == 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // プライバシー機能の設定開始
    //==========================================================================
    // プライバシー機能を有効化
    esp_err_t sts_val = sts_gap_config_local_privacy(ps_status->s_scan_config.own_addr_type);
    if (sts_val == ESP_OK) {
        // ステータをスキャン開始に更新
        ps_status->u32_status |= (GAP_STS_WAIT_SCAN | GAP_STS_EXEC_CONFIG_PRIVACY);
        // スキャン実行時間設定
        ps_status->u32_scan_duration = u32_duration;
    }
    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_scan_step_1
 *
 * DESCRIPTION:GAPプロファイルのスキャン開始処理（スキャンパラメータ設定）
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_scan_step_1() {
    //==========================================================================
    // 実行可否を判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // スキャンパラメータの設定待ちでは無い場合はエラー
    if ((ps_status->u32_status & GAP_STS_WAIT_CONFIG_SCAN) == 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // 既にスキャンパラメータ設定中の場合にはエラー
    if ((ps_status->u32_status & GAP_STS_EXEC_CONFIG_SCAN) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // パラメータ設定
    //==========================================================================
    // スキャンパラメータ設定
    esp_err_t sts_val = esp_ble_gap_set_scan_params(&ps_status->s_scan_config);
    if (sts_val == ESP_OK){
        // ステータスを実行待ちから実行中に更新
        ps_status->u32_status |= GAP_STS_EXEC_CONFIG_SCAN;
    }
    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_start_scan_step2
 *
 * DESCRIPTION:GAPプロファイルのスキャン開始処理（スキャン開始）
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_start_scan_step_2() {
    //==========================================================================
    // 実行可否を判定
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 既にスキャン実行中であればエラー
    if ((ps_status->u32_status & GAP_STS_EXEC_SCAN) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // スキャン実行可能でなければエラー
    if ((ps_status->u32_status & GAP_STS_CHK_SCAN_EXEC) != GAP_STS_CHK_SCAN_EXEC) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // スキャンを開始
    //==========================================================================
    esp_err_t sts_val = esp_ble_gap_start_scanning(ps_status->u32_scan_duration);
    if (sts_val == ESP_OK) {
        // スキャン実行中に更新
        ps_status->u32_status &= ~GAP_STS_WAIT_SCAN;
        ps_status->u32_status |= GAP_STS_EXEC_SCAN;
        ps_status->i64_scan_timeout = esp_timer_get_time() + (ps_status->u32_scan_duration * 1000000);
    }
    // 結果ステータスを返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_update_scan_status
 *
 * DESCRIPTION:GAPプロファイルのスキャンステータスの更新処理（タイムアウト判定等）
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス更新時はESP_OK
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_update_scan_status() {
    // 結果ステータス
    esp_err_t sts_val = ESP_FAIL;
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // スキャン開始中判定
    if ((ps_status->u32_status & GAP_STS_START_SCAN) == 0x00) {
        return sts_val;
    }
    // タイムアウト判定
    if (ps_status->i64_scan_timeout < esp_timer_get_time()) {
        // スキャンタイムアウト
        ps_status->u32_status &= ~GAP_STS_START_SCAN;
        ps_status->u32_scan_duration = 0;
        ps_status->i64_scan_timeout  = 0;
        sts_val = ESP_OK;
    }
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_gap_config_local_privacy
 *
 * DESCRIPTION:GAPプロファイルのローカルプライバシー機能の設定処理
 *
 * PARAMETERS:          Name        RW  Usage
 * esp_ble_addr_type_t  e_addr_type R   アドレスタイプ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gap_config_local_privacy(esp_ble_addr_type_t e_addr_type) {
    // GAPステータス
    ts_gap_status_t* ps_status = &s_gap_ctrl.s_status;
    // 既に実行中の場合にはエラー
    if ((ps_status->u32_status & GAP_STS_EXEC_CONFIG_PRIVACY) != 0x00) {
        // エラーステータス返却
        return ESP_ERR_INVALID_STATE;
    }
    // アドレスタイプを判定
    // BLE_ADDR_TYPE_PUBLIC
    // BLE_ADDR_TYPE_RANDOM
    // BLE_ADDR_TYPE_RPA_PUBLIC
    // BLE_ADDR_TYPE_RPA_RANDOM
    bool b_local_privacy = (e_addr_type == BLE_ADDR_TYPE_RPA_PUBLIC || e_addr_type == BLE_ADDR_TYPE_RPA_RANDOM);
    // GAPプライバシー機能を有効化
    esp_err_t sts_val = esp_ble_gap_config_local_privacy(b_local_privacy);
    if (sts_val == ESP_OK) {
        // ステータスをプライバシー機能実行中に移行
        ps_status->u32_status |= GAP_STS_EXEC_CONFIG_PRIVACY;
    }
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_gatts_get_if_status
 *
 * DESCRIPTION:GATTサーバーのインターフェースステータス取得処理
 *
 * PARAMETERS:      Name        RW  Usage
 * esp_gatt_if_t    t_gatt_if   R   GATTインターフェース
 *
 * RETURNS:
 *   ts_gatts_if_status* コネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gatts_if_status_t* ps_gatts_get_if_status(esp_gatt_if_t t_gatt_if) {
    // ステータスの探索
    ts_gatts_if_status_t* ps_if_sts = s_gatts_ctrl.ps_if_status;
    while (ps_if_sts != NULL) {
        if (ps_if_sts->t_gatt_if == t_gatt_if) {
            break;
        }
        // 次のステータス
        ps_if_sts = ps_if_sts->ps_next;
    }

    // 検索結果無し
    return ps_if_sts;
}

/*******************************************************************************
 *
 * NAME: ps_gatts_get_svc_status
 *
 * DESCRIPTION:GATTサーバーのサービスステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gatts_if_status_t*    ps_if_sts       R   GATTインターフェースステータス
 * uint8_t                  u8_svc_inst_id  R   サービスインスタンスID
 *
 *
 * RETURNS:
 *   ts_gatts_svc_status* サービスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gatts_svc_status_t* ps_gatts_get_svc_status(ts_gatts_if_status_t* ps_if_sts, uint8_t u8_svc_inst_id) {
    // サービス情報の検索
    ts_gatts_svc_status_t* ps_svc_sts = ps_if_sts->ps_svc_sts;
    uint8_t u8_svc_idx;
    for (u8_svc_idx = 0; u8_svc_idx < ps_if_sts->u8_svc_cnt; u8_svc_idx++) {
        if (ps_svc_sts[u8_svc_idx].u8_svc_inst_id == u8_svc_inst_id) {
            return &ps_svc_sts[u8_svc_idx];
        }
    }
    // 見つからなかった場合
    return NULL;
}

/*******************************************************************************
 *
 * NAME: ps_gatts_add_con_status
 *
 * DESCRIPTION:GATTサーバーのコネクションステータス追加処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gatts_if_status_t*    ps_if_sts   R   GATTインターフェースステータス
 * uint16_t                 u16_con_id  R   コネクションID
 *
 * RETURNS:
 *   ts_gatts_con_status* コネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gatts_con_status_t* ps_gatts_add_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id) {
    // コネクションの探索
    ts_gatts_con_status_t* ps_before  = NULL;
    ts_gatts_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (ps_con_sts->u16_con_id == u16_con_id) {
            return ps_con_sts;
        }
        // 次のステータスへ
        ps_before = ps_con_sts;
        ps_con_sts = ps_con_sts->ps_next;
    }
    // ステータス生成
    ps_con_sts = pv_mem_malloc(sizeof(ts_gatts_con_status_t));
    if (ps_con_sts == NULL) {
        return NULL;
    }
    // ステータス初期化
    *ps_con_sts = s_gatts_con_sts_default;
    ps_con_sts->t_gatt_if  = ps_if_sts->t_gatt_if;
    ps_con_sts->u16_app_id = ps_if_sts->u16_app_id;
    ps_con_sts->u16_con_id = u16_con_id;
    ps_con_sts->ps_rx_buff = ps_mdl_create_linked_queue();
    if (ps_con_sts->ps_rx_buff == NULL) {
        l_mem_free(ps_con_sts);
        return NULL;
    }
    // ステータスを追加
    if (ps_before == NULL) {
        ps_if_sts->ps_con_sts = ps_con_sts;
    } else {
        ps_before->ps_next = ps_con_sts;
    }
    // ステータスを返信
    return ps_con_sts;
}

/*******************************************************************************
 *
 * NAME: ps_gatts_get_con_status
 *
 * DESCRIPTION:GATTサーバーのコネクションステータス取得処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gatts_if_status_t*    ps_if_sts   R   GATTインターフェースステータス
 * uint16_t                 u16_con_id  R   コネクションID
 *
 * RETURNS:
 *   ts_gatts_con_status* コネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gatts_con_status_t* ps_gatts_get_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id) {
    // コネクションの探索
    ts_gatts_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (ps_con_sts->u16_con_id == u16_con_id) {
            break;
        }
        // 次のステータスへ
        ps_con_sts = ps_con_sts->ps_next;
    }
    // ステータスを返信
    return ps_con_sts;
}

/*******************************************************************************
 *
 * NAME: sts_gatts_get_handle_idx
 *
 * DESCRIPTION:GATTプロファイルのアトリビュートハンドルインデックス取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint16_t         u16_handle      R   GATTサーバーアトリビュートハンドル
 * uint8_t*         pu8_svc_idx     W   サービスインデックス
 * uint16_t*        u16_attr_idx    W   アトリビュートインデックス
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_get_handle_idx(ts_gatts_if_status_t* ps_if_sts,
                                           uint16_t u16_handle,
                                           uint8_t* pu8_svc_idx,
                                           uint16_t* pu16_hndl_idx) {
    // サービスのループ
    ts_gatts_svc_status_t* ps_svc_sts = ps_if_sts->ps_svc_sts;
    uint8_t u8_idx;
    uint16_t u16_idx;
    uint16_t u16_num_handle;
    uint16_t* pu16_handle;
    for (u8_idx = 0; u8_idx < ps_if_sts->u8_svc_cnt; u8_idx++) {
        u16_num_handle = ps_svc_sts[u8_idx].u16_num_handle;
        pu16_handle = ps_svc_sts[u8_idx].pu16_handles;
        for (u16_idx = 0; u16_idx < u16_num_handle; u16_idx++) {
            if(pu16_handle[u16_idx] == u16_handle) {
                *pu8_svc_idx   = u8_idx;
                *pu16_hndl_idx = u16_idx;
                return ESP_OK;
            }
        }
    }
    // 対象ハンドルが無い場合
    *pu8_svc_idx   = 0;
    *pu16_hndl_idx = 0;
    return ESP_ERR_NOT_FOUND;
}

/*******************************************************************************
 *
 * NAME: ps_gatts_get_attribute
 *
 * DESCRIPTION:GATTプロファイルのアトリビュートの取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint8_t          u8_svc_idx      R   サービスインデックス
 * uint16_t         u16_hndl_idx    R   ハンドルインデックス
 *
 * RETURNS:
 *   esp_gatts_attr_db_t*:GATTサーバーアトリビュート
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatts_attr_db_t* ps_gatts_get_attribute(ts_gatts_if_status_t* ps_if_sts,
                                                    uint8_t u8_svc_idx,
                                                    uint16_t u16_hndl_idx) {
    // サービス設定
    if (u8_svc_idx >= ps_if_sts->u8_svc_cnt) {
        return NULL;
    }
    // サービス設定
    ts_com_ble_gatts_svc_config_t* ps_cfg = &ps_if_sts->ps_svc_sts[u8_svc_idx].s_cfg;
    // アトリビュートテーブル
    if (u16_hndl_idx >= ps_cfg->u8_max_nb_attr) {
        return NULL;
    }
    // アトリビュートディスクリプタ
    return &ps_cfg->ps_attr_db[u16_hndl_idx];
}

/*******************************************************************************
 *
 * NAME: ps_gatts_get_handle_attribute
 *
 * DESCRIPTION:GATTプロファイルのハンドルに対応したアトリビュート取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 * uint16_t         u16_handle      R   GATTサーバーアトリビュートハンドル
 * uint8_t*         pu8_svc_idx     W   サービスインデックス
 * uint16_t*        u16_attr_idx    W   アトリビュートインデックス
 *
 * RETURNS:
 *   esp_gatts_attr_db_t*:GATTサーバーアトリビュート
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatts_attr_db_t* ps_gatts_get_handle_attribute(ts_gatts_if_status_t* ps_if_sts,
                                                           uint16_t u16_handle,
                                                           uint8_t* pu8_svc_idx,
                                                           uint16_t* pu16_hndl_idx) {
    // サービスのループ
    ts_gatts_svc_status_t* ps_svc_sts = ps_if_sts->ps_svc_sts;
    ts_com_ble_gatts_svc_config_t* ps_cfg = NULL;
    uint8_t u8_svc_idx;
    uint16_t u16_hndl_idx;
    uint16_t u16_num_handle;
    uint16_t* pu16_handles;
    for (u8_svc_idx = 0; u8_svc_idx < ps_if_sts->u8_svc_cnt; u8_svc_idx++) {
        u16_num_handle = ps_svc_sts[u8_svc_idx].u16_num_handle;
        pu16_handles = ps_svc_sts[u8_svc_idx].pu16_handles;
        for (u16_hndl_idx = 0; u16_hndl_idx < u16_num_handle; u16_hndl_idx++) {
            if(pu16_handles[u16_hndl_idx] != u16_handle) {
                continue;
            }
            // インデックスを編集
            *pu8_svc_idx   = u8_svc_idx;
            *pu16_hndl_idx = u16_hndl_idx;
            // サービス設定
            ps_cfg = &ps_svc_sts[u8_svc_idx].s_cfg;
            // アトリビュートディスクリプタ
            return &ps_cfg->ps_attr_db[u16_hndl_idx];
        }
    }
    // インデックスを初期化
    *pu8_svc_idx   = 0;
    *pu16_hndl_idx = 0;
    // 対象ハンドルが無い場合
    return NULL;
}


/*******************************************************************************
 *
 * NAME: v_gatts_del_con_status
 *
 * DESCRIPTION:GATTサーバーのコネクションステータス削除処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gatts_if_status_t*    ps_if_sts   R   GATTクライアントインターフェースステータス
 * uint16_t                 u16_con_id  R   コネクションID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gatts_del_con_status(ts_gatts_if_status_t* ps_if_sts, uint16_t u16_con_id) {
    ts_gatts_con_status_t* ps_con_bef = NULL;
    ts_gatts_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (ps_con_sts->u16_con_id != u16_con_id) {
            // 次のステータスへ
            ps_con_bef = ps_con_sts;
            ps_con_sts = ps_con_sts->ps_next;
            continue;
        }
        // 対象をパージ
        if (ps_con_bef == NULL) {
            ps_if_sts->ps_con_sts = ps_con_sts->ps_next;
        } else {
            ps_con_bef->ps_next = ps_con_sts->ps_next;
        }
        // 受信中データのクリア
        if (ps_con_sts->ps_rx_buff_data != NULL) {
            l_mem_free(ps_con_sts->ps_rx_buff_data);
            ps_con_sts->ps_rx_buff_data = NULL;
        }
        // 受信パケットバッファの解放
        sts_mdl_delete_linked_queue(ps_con_sts->ps_rx_buff);
        ps_con_sts->ps_rx_buff = NULL;
        // コネクションステータスメモリ解放
        l_mem_free(ps_con_sts);
        ps_con_sts = NULL;
        break;
    }
}

/*******************************************************************************
 *
 * NAME: sts_gatts_write_attr_value
 *
 * DESCRIPTION:GATTクライアントのアトリビュート値の書き込み処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatts_attr_db_t*         ps_attr     RW  アトリビュート
 * ts_com_ble_gatt_rx_data_t*   ps_param    R   受信データ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_write_attr_value(esp_gatts_attr_db_t* ps_attr,
                                             ts_com_ble_gatt_rx_data_t* ps_param) {
    // 書き込み判定
    if ((ps_attr->att_desc.perm & ESP_GATT_PERM_WRITE) != 0x0000) {
        // 書き込み不能エラー
        return ESP_ERR_INVALID_STATE;
    }
    // 自動返信の場合のみ対応
    if (ps_attr->attr_control.auto_rsp != ESP_GATT_AUTO_RSP) {
        return ESP_OK;
    }
    // データ編集
    ts_u8_array_t* ps_array = ps_param->ps_array;
    return esp_ble_gatts_set_attr_value(ps_param->u16_attr_hndl, ps_array->t_size, ps_array->pu8_values);
}

/*******************************************************************************
 *
 * NAME: sts_gatts_indication
 *
 * DESCRIPTION:GATTプロファイルのNotifyもしくはIndicate処理
 *
 * PARAMETERS:      Name                RW  Usage
 * esp_gatt_if_t    t_gatt_if           R   GATTサーバーインターフェース
 * uint8_t          u8_svc_idx          R   対象サービスインデックス
 * uint16_t         u16_handle          R   対象ハンドル
 * uint8_t*         pu8_data            R   送信データポインタ
 * uint16_t         u16_data_len        R   送信データ長
 * bool             b_need_confirm      R   返信要否
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_indication(esp_gatt_if_t t_gatt_if,
                                       uint8_t u8_svc_idx,
                                       uint16_t u16_handle,
                                       uint8_t* pu8_data,
                                       uint16_t u16_data_len,
                                       bool b_need_confirm) {
    // インターフェースステータス取得
    ts_gatts_if_status_t* ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
    if (ps_if_sts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // コネクションステータス取得
    ts_gatts_con_status_t* ps_con_sts = &ps_if_sts->ps_con_sts[0];
    if (ps_con_sts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    //==========================================================================
    // 一括送信判定
    //==========================================================================
    if (u16_data_len < 2) {
        // MTUサイズ-3バイト以下の場合には、一括でNotify通知を行う
        // ※UARTで受信したデータはNotify通知と言う形式で送信する
        // 引数１：SPPのGATTインターフェース
        // 引数２：コネクションID
        // 引数３：SPPのData Notifyのアトリビュートハンドル
        // 引数４：送信データ長
        // 引数５：送信値
        // 引数６：受信応答要否 true:Indication(応答必要)
        return esp_ble_gatts_send_indicate(ps_con_sts->t_gatt_if,
                                            ps_con_sts->u16_con_id,
                                            u16_handle,
                                            u16_data_len,
                                            pu8_data,
                                            b_need_confirm);
    } else if (u16_data_len <= (ps_con_sts->u16_mtu - 3) && (pu8_data[0] != '#' || pu8_data[1] != '#')) {
        // MTUサイズ-3バイト以下の場合には、一括でNotify通知を行う
        // ※UARTで受信したデータはNotify通知と言う形式で送信する
        // 引数１：SPPのGATTインターフェース
        // 引数２：コネクションID
        // 引数３：SPPのData Notifyのアトリビュートハンドル
        // 引数４：送信データ長
        // 引数５：送信値
        // 引数６：受信応答要否 true:Indication(応答必要)
        return esp_ble_gatts_send_indicate(ps_con_sts->t_gatt_if,
                                            ps_con_sts->u16_con_id,
                                            u16_handle,
                                            u16_data_len,
                                            pu8_data,
                                            b_need_confirm);
    }

    //==========================================================================
    // 分割送信
    //==========================================================================
    // サイズ確認、Readバッファからの読み込みサイズ（読み込み回数）を算出
    uint16_t u16_unit_size = ps_con_sts->u16_mtu - 7;
    uint8_t u8_total_num = u16_data_len / u16_unit_size;
    if ((u16_data_len % u16_unit_size) != 0) {
        u8_total_num++;
    }
#ifdef COM_BLE_DEBUG
    ESP_LOGI(LOG_TAG, "%s split packet Tx len:%d mtu:%d", __func__, u16_data_len, ps_con_sts->u16_mtu);
#endif
    // 通知データ
    uint16_t u16_notify_size = ps_con_sts->u16_mtu - 3;
    uint8_t u8_notify_data[u16_notify_size];
    // MTUのサイズに合わせてReadバッファから読み込みしたデータを送信
    esp_err_t sts_val = ESP_OK;
    uint8_t* pu8_from = pu8_data;
    uint16_t u16_len = u16_data_len;
    uint8_t u8_pkt_idx = 1;
    while (u8_pkt_idx <= u8_total_num) {
#ifdef COM_BLE_DEBUG
        ESP_LOGI(LOG_TAG, "%s split packet %d/%d unit:%d", __func__, u8_pkt_idx, u8_total_num, u16_unit_size);
#endif
        if (esp_ble_get_cur_sendable_packets_num(ps_con_sts->u16_con_id) <= 0) {
            // 排他制御一時停止
            if (xSemaphoreGiveRecursive(s_mutex) != pdTRUE) {
                return ESP_ERR_TIMEOUT;
            }
            // 送信中なのでウェイト
            vTaskDelay(GATT_TX_WAIT_TICK);
            // 排他制御再開
            if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
                return ESP_ERR_TIMEOUT;
            }
        }
        // パケットヘッダー
        u8_notify_data[0] = '#';
        u8_notify_data[1] = '#';
        u8_notify_data[2] = u8_total_num;
        u8_notify_data[3] = u8_pkt_idx;
        if (u8_pkt_idx < u8_total_num) {
            // Readバッファ
            memcpy(&u8_notify_data[4], pu8_from, u16_unit_size);
            sts_val = esp_ble_gatts_send_indicate(ps_con_sts->t_gatt_if,
                                                  ps_con_sts->u16_con_id,
                                                  u16_handle,
                                                  u16_notify_size,
                                                  u8_notify_data,
                                                  b_need_confirm);
            u16_len -= u16_unit_size;
        } else {
            memcpy(&u8_notify_data[4], pu8_from, u16_len);
            sts_val = esp_ble_gatts_send_indicate(ps_con_sts->t_gatt_if,
                                                  ps_con_sts->u16_con_id,
                                                  u16_handle,
                                                  u16_len + 4,
                                                  u8_notify_data,
                                                  b_need_confirm);
        }
        // 結果判定
        if (sts_val != ESP_OK) {
            return sts_val;
        }
        // パケット番号更新
        u8_pkt_idx++;
        // 転送データ開始位置
        pu8_from += u16_unit_size;
    }
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_gatts_evt_com_cb
 *
 * DESCRIPTION:GATTサーバーの共通イベントハンドラ
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatts_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gatts_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gatts_evt_com_cb(esp_gatts_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gatts_cb_param_t* pu_param) {
    //==========================================================================
    // アプリケーションの登録通知イベント処理
    //==========================================================================
#ifdef COM_BLE_DEBUG
    // イベントメッセージ
    ESP_LOGI(LOG_TAG, "GATTS_EVT=%s gatt_if=0x%x", pc_com_ble_gatts_event_to_str(e_event), t_gatt_if);
#endif
    // イベント処理
    if (e_event == ESP_GATTS_REG_EVT) {
        //----------------------------------------------------------------------
        // クリティカルセクション開始
        //----------------------------------------------------------------------
        if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
            return;
        }

        //----------------------------------------------------------------------
        // アプリケーションの登録通知イベント
        //----------------------------------------------------------------------
        sts_gatts_evt_register(t_gatt_if, pu_param);

        //----------------------------------------------------------------------
        // クリティカルセクション終了
        //----------------------------------------------------------------------
        xSemaphoreGiveRecursive(s_mutex);
        // 処理終了
        return;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // 共通イベントコールバック
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_gap_sts = &s_gap_ctrl.s_status;
    // GATTインターフェース設定
    ts_com_ble_gatts_if_config_t* ps_if_cfg = NULL;
    // GATTインターフェースステータス
    ts_gatts_if_status_t* ps_if_sts = NULL;
    // GATTインターフェースステータス
    ts_gatts_if_status_t* ps_bef_sts = NULL;
    // サービスステータス
    ts_gatts_svc_status_t* ps_svc_sts = NULL;
    // GATTサーバーイベント処理中のコネクションステータス
    ts_gatts_con_status_t* ps_con_sts = NULL;
    // アプリケーションループ
    uint32_t u32_size = 0;
    // GATTサーバーインターフェースステータス
    ps_if_sts = s_gatts_ctrl.ps_if_status;
    while (ps_if_sts != NULL) {
        //======================================================================
        // GATTS共通処理 Start
        //======================================================================
        // イベント処理
        // アプリケーションIDの登録通知の有無を判定
        // ※ESP_GATT_IF_NONE(0xFF)：すべてのプロファイルのコールバック関数を実行
        if (t_gatt_if != ESP_GATT_IF_NONE &&
            t_gatt_if != ps_if_sts->t_gatt_if) {
            // 次のステータス
            ps_bef_sts = ps_if_sts;
            ps_if_sts  = ps_if_sts->ps_next;
            continue;
        }
        // アプリケーション情報
        ps_if_cfg  = &ps_if_sts->s_cfg;
        ps_con_sts = NULL;
        // イベント判定
        switch (e_event) {
        case ESP_GATTS_WRITE_EVT:
            // GATTクライアントからの書き込みイベント
            // 書き込みデータの受信処理
            sts_gatts_evt_write(ps_if_sts, &pu_param->write);
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // バッファデータの処理イベント
            sts_gatts_evt_exec_write(ps_if_sts, &pu_param->exec_write);
            break;
        case ESP_GATTS_MTU_EVT:
            // MTUの変更通知イベント
            ps_con_sts = ps_gatts_get_con_status(ps_if_sts, pu_param->mtu.conn_id);
            if (ps_con_sts != NULL) {
                ps_con_sts->u16_mtu = pu_param->mtu.mtu;
            }
            break;
        case ESP_GATTS_UNREG_EVT:
            // アプリケーションの登録解除通知イベント
            sts_gatts_evt_unregist(ps_if_sts, ps_bef_sts);
            break;
        case ESP_GATTS_CONNECT_EVT:
            // GAPステータスを更新（アドバタイズ実行中の場合には実行待ちに移行）
            if ((ps_gap_sts->u32_status & GAP_STS_EXEC_ADVERTISING) != 0x00) {
                ps_gap_sts->u32_status |= GAP_STS_WAIT_ADVERTISING;
            }
            ps_gap_sts->u32_status &= ~GAP_STS_EXEC_ADVERTISING;
            // 接続情報更新
            ps_con_sts = ps_gatts_add_con_status(ps_if_sts, pu_param->connect.conn_id);
            v_com_ble_addr_cpy(ps_con_sts->t_bda, pu_param->connect.remote_bda);
            // コネクションのセキュリティ設定
            esp_ble_set_encryption(pu_param->connect.remote_bda, ps_if_cfg->e_con_sec);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            // GATTクライアントとの物理的な切断通知イベント
#ifdef COM_BLE_DEBUG
            ESP_LOGI(LOG_TAG, "ESP_GATTS:Disconnect reason = 0x%x", pu_param->disconnect.reason);
#endif
            // コネクションステータスの削除
            v_gatts_del_con_status(ps_if_sts, pu_param->disconnect.conn_id);
            // 物理切断なので、GAPデバイス情報も削除する
            sts_gap_del_device(pu_param->disconnect.remote_bda);
            // 実行待ちの場合にはアドバタイジングの実行開始
            sts_gap_start_advertise_step_0();
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            // GATTアトリビュートテーブル生成完了通知イベント
            if (pu_param->add_attr_tab.status != ESP_GATT_OK) {
#ifdef COM_BLE_DEBUG
                ESP_LOGI(LOG_TAG, "ESP_GATTS:create attribute table failed, status = 0x%x", pu_param->add_attr_tab.status);
#endif
                break;
            }
            // サービス情報の更新
            ps_svc_sts = ps_gatts_get_svc_status(ps_if_sts, pu_param->add_attr_tab.svc_inst_id);
            if (ps_svc_sts == NULL) {
                break;
            }
            // ハンドル数の編集
            ps_svc_sts->u16_num_handle = pu_param->add_attr_tab.num_handle;
            // 現在のハンドルリストを解放
            if (ps_svc_sts->pu16_handles != NULL) {
                l_mem_free(ps_svc_sts->pu16_handles);
            }
            // ハンドルリストの編集
            u32_size = pu_param->add_attr_tab.num_handle * sizeof(uint16_t);
            ps_svc_sts->pu16_handles =
                    (uint16_t*)pv_mem_clone((void*)pu_param->add_attr_tab.handles, u32_size);
            // サービスIDのアトリビュートを指定してサービスを開始する
            esp_ble_gatts_start_service(ps_svc_sts->pu16_handles[0]);
            break;
        default:
            break;
        }
        // コールバック関数が有れば実行
        if (ps_if_cfg->fc_gatts_cb != NULL) {
            ps_if_cfg->fc_gatts_cb(e_event, t_gatt_if, pu_param);
        }
        // 次のステータス
        ps_bef_sts = ps_if_sts;
        ps_if_sts  = ps_if_sts->ps_next;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_gatts_evt_dmy_cb
 *
 * DESCRIPTION:GATTサーバーのダミーイベントハンドラ
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatts_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gatts_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gatts_evt_dmy_cb(esp_gatts_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gatts_cb_param_t* pu_param) {
    return;
}

/*******************************************************************************
 *
 * NAME: sts_gatts_evt_register
 *
 * DESCRIPTION:GATTプロファイルのイベント処理：インターフェース登録
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェース
 * esp_ble_gatts_cb_param_t*    ps_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_evt_register(esp_gatt_if_t t_gatt_if,
                                         esp_ble_gatts_cb_param_t* pu_param) {
    //======================================================================
    // 入力チェック
    //======================================================================
    if (pu_param->reg.status != ESP_GATT_OK) {
        // 処理終了
        return ESP_ERR_INVALID_ARG;
    }

    //======================================================================
    // GATTサーバーのインターフェースステータスの更新
    //======================================================================
    // GATTサーバーインターフェースステータスの探索
    ts_gatts_if_status_t* ps_if_sts;
    ts_com_ble_gatts_if_config_t* ps_if_cfg;
    ts_com_ble_gatts_svc_config_t* ps_svc_cfg;
    for (ps_if_sts = s_gatts_ctrl.ps_if_status; ps_if_sts != NULL; ps_if_sts = ps_if_sts->ps_next) {
        // アプリケーションIDをチェック
        if (ps_if_sts->u16_app_id != pu_param->reg.app_id) {
            continue;
        }
        //------------------------------------------------------------------
        // インターフェースの更新
        //------------------------------------------------------------------
        ps_if_sts->t_gatt_if = t_gatt_if;
        ps_if_cfg = &ps_if_sts->s_cfg;
        //------------------------------------------------------------------
        // サービス情報の更新
        //------------------------------------------------------------------
        int i_svc_idx;
        for (i_svc_idx = 0; i_svc_idx < ps_if_cfg->u8_svc_cnt; i_svc_idx++) {
            // サービスステータス更新
            ps_if_sts->ps_svc_sts[i_svc_idx].t_gatt_if = t_gatt_if;
            // GATTサーバーのアトリビュートテーブルの生成処理
            // 引数１：サーバーアトリビュートテーブル
            // 引数２：GATTサーバーアクセスインターフェース
            // 引数３：サーバーアトリビュートテーブルサイズ
            // 引数４：サービスのインスタンスインデックス
            ps_svc_cfg = &ps_if_cfg->ps_svc_cfg[i_svc_idx];
            esp_ble_gatts_create_attr_tab(ps_svc_cfg->ps_attr_db,
                                          t_gatt_if,
                                          ps_svc_cfg->u8_max_nb_attr,
                                          ps_svc_cfg->u8_inst_id);
        }
        // コールバック関数が有れば実行
        if (ps_if_cfg->fc_gatts_cb != NULL) {
            ps_if_cfg->fc_gatts_cb(ESP_GATTS_REG_EVT, t_gatt_if, pu_param);
        }
        break;
    }

    // 結果ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gatts_evt_write
 *
 * DESCRIPTION:GATTプロファイルの書き込みデータ受信処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gatts_if_status_t*    ps_if_sts   R   GATTインターフェースステータス
 * gatts_write_evt_param_t* ps_param    R   Writeパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_evt_write(ts_gatts_if_status_t* ps_if_sts,
                                      struct gatts_write_evt_param* ps_param) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // コネクションの取得
    ts_gatts_con_status_t* ps_con_sts = ps_gatts_get_con_status(ps_if_sts, ps_param->conn_id);
    if (ps_con_sts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 受信パケット情報生成
    //==========================================================================
    // 受信書き込みデータ
    esp_gatts_attr_db_t*     ps_rx_attr = NULL;
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    if (ps_param->is_prep) {
        // 分割パケットの場合
        ps_rx_attr = ps_con_sts->ps_rx_buff_attr;
        ps_rx_data = ps_con_sts->ps_rx_buff_data;
    }
    // 対象アトリビュートの有無を判定
    if (ps_rx_attr == NULL) {
        // 受信ハンドルチェック
        uint8_t u8_svc_idx;
        uint16_t u16_hndl_idx;
        // アトリビュートの取得
        ps_rx_attr = ps_gatts_get_handle_attribute(ps_if_sts, ps_param->handle, &u8_svc_idx, &u16_hndl_idx);
        if (ps_rx_attr == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        bool b_auto_rsp = (ps_rx_attr->attr_control.auto_rsp == ESP_GATT_AUTO_RSP);
        // 受信中データの編集
        ps_rx_data = pv_mem_malloc(sizeof(ts_com_ble_gatt_rx_data_t));
        if (ps_rx_data == NULL) {
            return ESP_ERR_NO_MEM;
        }
        ps_rx_data->u16_app_id      = ps_con_sts->u16_app_id;       // アプリケーションID   ※キー１
        ps_rx_data->t_gatt_if       = ps_con_sts->t_gatt_if;        // GATTインターフェース ※キー２
        ps_rx_data->u16_con_id      = ps_param->conn_id;            // コネクションID       ※キー３
        v_com_ble_addr_cpy(ps_rx_data->t_bda, ps_con_sts->t_bda);    // リモートデバイスアドレス
        ps_rx_data->e_type          = GATT_RX_TYPE_WRITE_DATA;      // 受信データタイプ
        ps_rx_data->t_status        = ESP_GATT_OK;                  // 結果ステータス
        ps_rx_data->u16_attr_hndl   = ps_param->handle;             // アトリビュートハンドル
        ps_rx_data->u8_svc_idx      = u8_svc_idx;                   // サービスインデックス
        ps_rx_data->u16_hndl_idx    = u16_hndl_idx;                 // ハンドルインデックス
        ps_rx_data->b_auto_rsp      = b_auto_rsp;                   // 自動返信
        ps_rx_data->ps_array        = NULL;                         // 受信データ配列
    }

    //==========================================================================
    // 非分割データ受信処理
    //==========================================================================
    // 分割パケット判定
    ts_gatts_svc_status_t* ps_svc_sts;
    if (!ps_param->is_prep) {
        // 受信データ編集
        ps_rx_data->ps_array = ps_mdl_clone_u8_array(ps_param->value, ps_param->len);
        if (ps_rx_data->ps_array == NULL) {
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
            return ESP_ERR_NO_MEM;
        }
        // サービスステータスの取得
        ps_svc_sts = &ps_if_sts->ps_svc_sts[ps_rx_data->u8_svc_idx];
        // 自動返信の場合には値を更新
        sts_gatts_write_attr_value(ps_rx_attr, ps_rx_data);
        // 受信データをエンキュー
        if (xQueueSend(ps_svc_sts->t_rx_queue, &ps_rx_data, BLE_SPP_QUEUE_WAIT) != pdPASS) {
            // エンキュー対象の受信データクリア
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
            // 受信失敗
            return ESP_FAIL;
        }
        // 受信完了
        return ESP_OK;
    }

    //==========================================================================
    // 分割データ受信処理
    //==========================================================================
    // 受信バッファの情報を更新
    ps_con_sts->ps_rx_buff_attr = ps_rx_attr;
    ps_con_sts->ps_rx_buff_data = ps_rx_data;
    // 書き込み対象のアトリビュートハンドルの判定
    if (ps_param->handle != ps_rx_data->u16_attr_hndl) {
        // アトリビュートハンドルが一致しない場合
        // 受信中のデータバッファをクリア
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
        ps_con_sts->ps_rx_buff_attr = NULL;
        ps_con_sts->ps_rx_buff_data = NULL;
        // バッファリング中のデータをクリア
        sts_mdl_clear_linked_queue(ps_con_sts->ps_rx_buff);
        // 受信失敗
        return ESP_ERR_INVALID_STATE;
    }
    // 分割データのバッファリング
    return sts_mdl_linked_enqueue(ps_con_sts->ps_rx_buff, ps_param->value, ps_param->len);
}

/*******************************************************************************
 *
 * NAME: sts_gatts_evt_exec_write
 *
 * DESCRIPTION:GATTプロファイルのイベント処理：分割書き込み完了通知イベント
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_gatts_if_status_t*        ps_if_sts   R   GATTインターフェースステータス
 * gatts_exec_write_evt_param*  ps_param    R   Exec Writeパラメータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_evt_exec_write(ts_gatts_if_status_t* ps_if_sts,
                                           struct gatts_exec_write_evt_param* ps_param) {
    // 受信バッファの処理イベント
    //==========================================================================
    // 受信情報取得
    //==========================================================================
    // コネクションステータスの取得
    ts_gatts_con_status_t* ps_con_sts = ps_gatts_get_con_status(ps_if_sts, ps_param->conn_id);
    if (ps_con_sts == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    // 受信中データ取得
    esp_gatts_attr_db_t*     ps_rx_attr = ps_con_sts->ps_rx_buff_attr;
    ts_com_ble_gatt_rx_data_t* ps_rx_data = ps_con_sts->ps_rx_buff_data;
    if (ps_rx_attr == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // 受信中の情報をクリア
    ps_con_sts->ps_rx_buff_attr = NULL;
    ps_con_sts->ps_rx_buff_data = NULL;
    // 受信データ取得
    ts_linked_queue_t* ps_buff = ps_con_sts->ps_rx_buff;
    ps_rx_data->ps_array = ps_mdl_linked_dequeue(ps_buff, ps_buff->t_size);
    if (ps_rx_data->ps_array == NULL) {
        // 受信中のデータバッファクリア
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
        // 受信中データが見つからない
        return ESP_ERR_NOT_FOUND;
    }

    //==========================================================================
    // 受信キャンセル処理
    //==========================================================================
    // 受信実行判定
    if (ps_param->exec_write_flag == ESP_GATT_PREP_WRITE_CANCEL) {
        // 受信キャンセルの場合には受信中のデータをクリア
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
        // 正常終了
        return ESP_OK;
    }

    //==========================================================================
    // 受信処理
    //==========================================================================
    // 自動返信の場合には値を更新
    sts_gatts_write_attr_value(ps_rx_attr, ps_rx_data);
    // サービスステータスの取得
    ts_gatts_svc_status_t* ps_svc_sts = &ps_if_sts->ps_svc_sts[ps_rx_data->u8_svc_idx];
    // 受信データをエンキュー
    if (xQueueSend(ps_svc_sts->t_rx_queue, &ps_rx_data, BLE_SPP_QUEUE_WAIT) != pdPASS) {
        // キューイングエラー
        // 受信データクリア
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
        // 異常終了
        return ESP_FAIL;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gatts_evt_unregist
 *
 * DESCRIPTION:GATTプロファイルのイベント処理：インターフェース登録解除
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gatts_if_status_t*    ps_tgt_sts  RW  パージ対象ステータス
 * ts_gatts_if_status_t*    ps_bef_sts  RW  直前のステータス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gatts_evt_unregist(ts_gatts_if_status_t* ps_tgt_sts,
                                         ts_gatts_if_status_t* ps_bef_sts) {
    //==========================================================================
    // GATTインターフェースステータスのパージ
    //==========================================================================
    if (ps_bef_sts != NULL) {
        ps_bef_sts->ps_next = ps_tgt_sts->ps_next;
    } else {
        s_gatts_ctrl.ps_if_status = ps_tgt_sts->ps_next;
    }

    //==========================================================================
    // ステータスの解放
    //==========================================================================
    //--------------------------------------------------------------------------
    // GATTサーバーのサービスステータスを解放
    //--------------------------------------------------------------------------
    // アトリビュートハンドルリストの解放
    ts_gatts_svc_status_t* ps_svc_sts = ps_tgt_sts->ps_svc_sts;
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < ps_tgt_sts->u8_svc_cnt; u8_idx++) {
        // アトリビュートDBの解放
        l_mem_free(ps_svc_sts[u8_idx].s_cfg.ps_attr_db);
        // アトリビュートハンドルの解放
        l_mem_free(ps_svc_sts[u8_idx].pu16_handles);
    }
    // サービスステータスの解放
    l_mem_free(ps_svc_sts);

    //--------------------------------------------------------------------------
    // コネクションステータスの解放
    //--------------------------------------------------------------------------
    ts_gatts_con_status_t* ps_con_sts = ps_tgt_sts->ps_con_sts;
    ts_gatts_con_status_t* ps_con_bef;
    while (ps_con_sts != NULL) {
        // 次のコネクション
        ps_con_bef = ps_con_sts;
        ps_con_sts = ps_con_sts->ps_next;
        // 受信中データの解放
        if (ps_con_bef->ps_rx_buff_data != NULL) {
            v_com_ble_gatt_delete_rx_data(ps_con_bef->ps_rx_buff_data);
        }
        // 受信データ
        if (ps_con_bef->ps_rx_buff != NULL) {
            sts_mdl_delete_linked_queue(ps_con_bef->ps_rx_buff);
        }
        // コネクションステータスを解放
        l_mem_free(ps_con_bef);
    }

    //--------------------------------------------------------------------------
    // インターフェースステータスの解放
    //--------------------------------------------------------------------------
    l_mem_free(ps_tgt_sts);
    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_open
 *
 * DESCRIPTION:GATTプロファイルのサーバーへの接続処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_gatt_if_t        t_gatt_if       R   GATTインターフェース
 * esp_bd_addr_t        t_bda           R   リモートアドレス
 * esp_ble_addr_type_t  e_bda_type      R   リモートアドレスタイプ
 * bool                 b_direct        R   自動接続
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_open(esp_gatt_if_t t_gatt_if,
                                 esp_bd_addr_t t_bda,
                                 esp_ble_addr_type_t e_addr_type,
                                 bool b_direct) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // IFステータスの探索
    ts_gattc_if_status_t* ps_if_status = ps_gattc_get_if_status(t_gatt_if);
    if (ps_if_status == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 接続開始処理
    //==========================================================================
    // コネクションステータス生成
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_add_con_status(ps_if_status, t_bda);
    if (ps_con_sts == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // 接続状態を判定
    if ((ps_con_sts->u8_status & GATTC_STS_CONNECTING) != GATTC_STS_NONE) {
        // 既にオープンしているか、オープン要求中の場合
        return ESP_OK;
    }
    // 結果ステータス
    esp_err_t sts_val;
    // スキャン実行中の場合には停止
    if ((s_gap_ctrl.s_status.u32_status & GAP_STS_EXEC_SCAN) != 0x00) {
        sts_val = esp_ble_gap_stop_scanning();
        if (sts_val != ESP_OK) {
            return sts_val;
        }
    }
    // コネクションのステータスをOPENに更新
    ps_con_sts->u8_status |= GATTC_STS_REQUEST_OPEN;
    // GATTクライアントとして接続を開始
    tc_com_ble_bda_string_t tc_bda;
    v_com_ble_address_to_str(tc_bda, t_bda);
    return esp_ble_gattc_open(t_gatt_if, t_bda, e_addr_type, b_direct);
}

/*******************************************************************************
 *
 * NAME: sts_gattc_close
 *
 * DESCRIPTION:GATTプロファイルのサーバーとの切断処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_gatt_if_t        t_gatt_if       R   GATTインターフェース
 * esp_bd_addr_t        t_bda           R   リモートアドレス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_close(esp_gatt_if_t t_gatt_if,
                                  esp_bd_addr_t t_bda) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_bda(t_gatt_if, t_bda);
    if (ps_con_sts == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // 状態判定
    if ((ps_con_sts->u8_status & GATTC_STS_CONNECTING) == GATTC_STS_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 切断処理
    //==========================================================================
    // GATTサーバーとの切断を開始
    return esp_ble_gattc_close(t_gatt_if, ps_con_sts->u16_con_id);
}

/*******************************************************************************
 *
 * NAME: ps_gattc_get_if_status
 *
 * DESCRIPTION:GATTクライアントのIFステータス取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェース
 *
 * RETURNS:
 *   ts_com_ble_gattc_con_status* GATTクライアントのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_if_status_t* ps_gattc_get_if_status(esp_gatt_if_t t_gatt_if) {
    // IFステータスの探索
    ts_gattc_if_status_t* ps_if_status = s_gattc_ctrl.ps_if_status;
    uint16_t u16_idx;
    for (u16_idx = 0; u16_idx < s_gattc_ctrl.u16_if_count; u16_idx++) {
        if (ps_if_status[u16_idx].t_gatt_if == t_gatt_if) {
            return &ps_if_status[u16_idx];
        }
    }
    return NULL;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_add_con_status
 *
 * DESCRIPTION:GATTクライアントのコネクションステータス取得追加処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts   R   GATTインターフェースステータス
 * esp_bd_addr_t            t_bda       R   リモートアドレス
 *
 * RETURNS:
 * ts_gattc_con_status* GATTクライアントのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_con_status_t* ps_gattc_add_con_status(ts_gattc_if_status_t* ps_if_sts, esp_bd_addr_t t_bda) {
    // コネクションの探索
    ts_gattc_con_status_t* ps_before  = NULL;
    ts_gattc_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (l_com_ble_addr_cmp(ps_con_sts->t_bda, t_bda) == 0) {
            return ps_con_sts;
        }
        // 次のステータスへ
        ps_before  = ps_con_sts;
        ps_con_sts = ps_con_sts->ps_next;
    }
    // ステータス生成
    ps_con_sts = pv_mem_malloc(sizeof(ts_gattc_con_status_t));
    if (ps_con_sts == NULL) {
        return NULL;
    }
    // ステータス初期化
    *ps_con_sts = s_gattc_con_sts_default;
    ps_con_sts->t_gatt_if  = ps_if_sts->t_gatt_if;
    ps_con_sts->u16_app_id = ps_if_sts->u16_app_id;
    v_com_ble_addr_cpy(ps_con_sts->t_bda, t_bda);
    // ステータスを追加
    if (ps_before == NULL) {
        ps_if_sts->ps_con_sts = ps_con_sts;
    } else {
        ps_before->ps_next = ps_con_sts;
    }
    // ステータスを返信
    return ps_con_sts;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_get_con_status_bda
 *
 * DESCRIPTION:GATTクライアントのアドレスに対応したコネクションステータス取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTクライアントインターフェース
 * esp_bd_addr_t    t_bda           R   リモートアドレス
 *
 * RETURNS:
 * ts_com_ble_gattc_con_status* GATTクライアントのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_con_status_t* ps_gattc_get_con_status_bda(esp_gatt_if_t t_gatt_if, esp_bd_addr_t t_bda) {
    // IFステータスの探索
    ts_gattc_if_status_t* ps_if_status = ps_gattc_get_if_status(t_gatt_if);
    if (ps_if_status == NULL) {
        return NULL;
    }
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts = ps_if_status->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (l_com_ble_addr_cmp(ps_con_sts->t_bda, t_bda) == 0) {
            return ps_con_sts;
        }
        ps_con_sts = ps_con_sts->ps_next;
    }
    // ステータスを返信
    return NULL;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_get_con_status_id
 *
 * DESCRIPTION:GATTクライアントのオープンされたコネクションステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts       R   GATTインターフェースステータス
 * uint16_t                 u16_con_id      R   コネクションID
 *
 * RETURNS:
 *   ts_com_ble_gattc_con_status* GATTクライアントのコネクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_con_status_t* ps_gattc_get_con_status_id(ts_gattc_if_status_t* ps_if_sts, uint16_t u16_con_id) {
    // コネクションの探索
    ts_gattc_con_status_t* ps_con_sts;
    for (ps_con_sts = ps_if_sts->ps_con_sts; ps_con_sts != NULL; ps_con_sts = ps_con_sts->ps_next) {
        // コネクションIDのチェック
        if (ps_con_sts->u16_con_id != u16_con_id) {
            continue;
        }
        // コネクション状態のチェック
        if ((ps_con_sts->u8_status & GATTC_STS_OPEN) != GATTC_STS_OPEN) {
            continue;
        }
        return ps_con_sts;
    }
    // ステータスを返信
    return NULL;
}

/*******************************************************************************
 *
 * NAME: v_gattc_del_con_status
 *
 * DESCRIPTION:GATTクライアントのコネクションステータス削除処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts   R   GATTクライアントインターフェースステータス
 * esp_bd_addr_t            t_bda       R   リモートデバイスアドレス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gattc_del_con_status(ts_gattc_if_status_t* ps_if_sts, esp_bd_addr_t t_bda) {
    ts_gattc_con_status_t* ps_con_bef = NULL;
    ts_gattc_con_status_t* ps_con_sts = ps_if_sts->ps_con_sts;
    while (ps_con_sts != NULL) {
        if (l_com_ble_addr_cmp(ps_con_sts->t_bda, t_bda)) {
            // 次のステータスへ
            ps_con_bef = ps_con_sts;
            ps_con_sts = ps_con_sts->ps_next;
            continue;
        }
        // 対象をパージ
        if (ps_con_bef == NULL) {
            ps_if_sts->ps_con_sts = ps_con_sts->ps_next;
        } else {
            ps_con_bef->ps_next = ps_con_sts->ps_next;
        }
        // サービスステータスを解放
        ts_gattc_svc_status_t* ps_svc_sts = ps_con_sts->ps_svc_sts;
        ts_gattc_svc_status_t* ps_svc_bef = NULL;
        while (ps_svc_sts != NULL) {
            ps_svc_sts->ps_db_elems = NULL;
            ps_svc_sts->ps_con_sts = NULL;
            ps_svc_bef = ps_svc_sts;
            ps_svc_sts = ps_svc_sts->ps_next;
            l_mem_free(ps_svc_bef);
        }
        // 次のコネクションへ
        ps_con_bef = ps_con_sts;
        ps_con_sts = ps_con_sts->ps_next;
        // コネクションステータスメモリ解放
        l_mem_free(ps_con_bef);
    }
}

/*******************************************************************************
 *
 * NAME: sts_gattc_search_service
 *
 * DESCRIPTION:GATTクライアントのサービス検索処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_bd_addr_t        t_bda           R   GATTインターフェースステータス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_search_service(esp_bd_addr_t t_bda) {
    // GAP設定
    ts_com_ble_gap_config_t* ps_gap_cfg = &s_gap_ctrl.s_config;
    // GAPステータス
    ts_gap_device_t* ps_gap_sts = ps_gap_get_device(t_bda);
    // 認証状態を判定
    if (ps_gap_cfg->u8_auth_option == ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE &&
        (ps_gap_sts->u16_status & GAP_DEV_STS_AUTHENTICATED) != GAP_DEV_STS_AUTHENTICATED) {
        // 認証が必要だが未認証の場合
        return ESP_ERR_INVALID_STATE;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts;
    // コネクションの探索とサービスの検索
    uint16_t u16_idx;
    for (u16_idx = 0; u16_idx < s_gattc_ctrl.u16_if_count; u16_idx++) {
        // コネクションステータスの取得
        ps_con_sts = s_gattc_ctrl.ps_if_status[u16_idx].ps_con_sts;
        while (ps_con_sts != NULL) {
            if (l_com_ble_addr_cmp(ps_con_sts->t_bda, t_bda) == 0 &&
                (ps_con_sts->u8_status & GATTC_STS_SEARCH_SVC_MASK) == GATTC_STS_SEARCH_SVC_PTN) {
                // GATTステータスがOPENかつMTUが設定済みで、サービスの検索中ではない場合
                // GATTサービスの検索処理
                ps_con_sts->u8_status |= GATTC_STS_SEARCH_SVC;
                ps_con_sts->u8_status &= ~GATTC_STS_SET_SVC;
                sts_val = esp_ble_gattc_search_service(ps_con_sts->t_gatt_if, ps_con_sts->u16_con_id, NULL);
                if (sts_val != ESP_OK) {
                    return sts_val;
                }
            }
            ps_con_sts = ps_con_sts->ps_next;
        }
    }
    // 正常終了
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_add_svc_status
 *
 * DESCRIPTION:GATTクライアントのサービスステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts       R   GATTインターフェースステータス
 * uint16_t                 u16_con_id      R   コネクションID
 * esp_gatt_id_t            s_svc_id        R   サービスID
 *
 * RETURNS:
 *   ts_gattc_svc_status_t* GATTクライアントのサービスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_svc_status_t* ps_gattc_add_svc_status(ts_gattc_if_status_t* ps_if_sts,
                                                       uint16_t u16_con_id,
                                                       esp_gatt_id_t* ps_svc_id) {
    // コネクションの探索
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, u16_con_id);
    if (ps_con_sts == NULL) {
        return NULL;
    }
    // サービスの探索
    ts_gattc_svc_status_t* ps_svc_sts = ps_con_sts->ps_svc_sts;
    ts_gattc_svc_status_t* ps_bef_sts = NULL;
    uint8_t u8_svc_idx = 0;
    while (ps_svc_sts != NULL) {
        if (b_com_ble_id_equal(&ps_svc_sts->s_svc_id, ps_svc_id)) {
            return ps_svc_sts;
        }
        // 次のサービスへ
        ps_bef_sts = ps_svc_sts;
        ps_svc_sts = ps_svc_sts->ps_next;
        u8_svc_idx++;
    }
    // ステータス生成
    ps_svc_sts = pv_mem_malloc(sizeof(ts_gattc_svc_status_t));
    if (ps_con_sts == NULL) {
        return NULL;
    }
    // ステータス初期化
    ps_svc_sts->s_svc_id        = *ps_svc_id;   // サービスID
    ps_svc_sts->u8_svc_idx      = u8_svc_idx;   // サービスインデックス
    ps_svc_sts->b_primary       = false;        // プライマリサービスフラグ
    ps_svc_sts->u16_start_hndl  = 0;            // サービスのハンドル（開始）
    ps_svc_sts->u16_end_hndl    = 0;            // サービスのハンドル（終了）
    ps_svc_sts->u16_db_elem_cnt = 0;            // アトリビュートDBの要素数
    ps_svc_sts->ps_db_elems     = NULL;         // アトリビュートDB情報
    // Notifyステータス取得
    ts_gattc_rcv_notify_status_t* ps_notify = &ps_svc_sts->s_notify;
    // 受信中のハンドル
    ps_notify->u16_handle = 0;
    // 受信中の全パケット数
    ps_notify->u8_pkt_all = 0;
    // 受信中の現在パケット番号
    ps_notify->u8_pkt_num = 0;
    // Notifyデータの受信キュー
    ps_notify->ps_queue = ps_mdl_create_linked_queue();
    // RXデータキュー
    ps_svc_sts->t_rx_queue =
            xQueueCreate(COM_BLE_GATT_RX_BUFF_SIZE, sizeof(ts_com_ble_gatt_rx_data_t*));
    ps_svc_sts->ps_con_sts         = ps_con_sts;    // コネクションステータス
    ps_svc_sts->ps_next            = NULL;          // 次のサービスステータス
    // ステータス追加
    if (ps_bef_sts != NULL) {
        ps_bef_sts->ps_next     = ps_svc_sts;
        ps_con_sts->u16_svc_cnt++;
    } else {
        ps_con_sts->ps_svc_sts  = ps_svc_sts;
        ps_con_sts->u16_svc_cnt = 1;
    }
    // ステータスを返信
    return ps_svc_sts;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_get_svc_status
 *
 * DESCRIPTION:GATTクライアントのサービスステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts       R   GATTインターフェースステータス
 * uint16_t                 u16_con_id      R   コネクションID
 * esp_gatt_id_t            s_svc_id        R   サービスID
 *
 * RETURNS:
 *   ts_gattc_svc_status_t* GATTクライアントのサービスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_svc_status_t* ps_gattc_get_svc_status(ts_gattc_if_status_t* ps_if_sts,
                                                       uint16_t u16_con_id,
                                                       esp_gatt_id_t s_svc_id) {
    // コネクションの探索
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, u16_con_id);
    if (ps_con_sts == NULL) {
        return NULL;
    }
    // サービスの探索
    ts_gattc_svc_status_t* ps_svc_sts = ps_con_sts->ps_svc_sts;
    while (ps_svc_sts != NULL) {
        if (b_com_ble_id_equal(&ps_svc_sts->s_svc_id, &s_svc_id)) {
            return ps_svc_sts;
        }
        // 次のサービスへ
        ps_svc_sts = ps_svc_sts->ps_next;
    }
    // 対象サービスなし
    return NULL;
}

/*******************************************************************************
 *
 * NAME: ps_gattc_get_handle_svc_status
 *
 * DESCRIPTION:GATTクライアントのアトリビュートハンドルに対応したサービスステータス取得処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts       R   GATTインターフェース
 * uint16_t                 u16_handle      R   GATTサーバーアトリビュートハンドル
 *
 * RETURNS:
 *   ts_gattc_svc_status_t* GATTクライアントのサービスステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_gattc_svc_status_t* ps_gattc_get_handle_svc_status(ts_gattc_if_status_t* ps_if_sts,
                                                              uint16_t u16_handle) {
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts;
    // サービスステータス
    ts_gattc_svc_status_t* ps_svc_sts;
    // コネクションのループ
    for (ps_con_sts = ps_if_sts->ps_con_sts; ps_con_sts != NULL; ps_con_sts = ps_con_sts->ps_next) {
        // サービスのループ
        for (ps_svc_sts = ps_con_sts->ps_svc_sts; ps_svc_sts != NULL; ps_svc_sts = ps_svc_sts->ps_next) {
            if (u16_handle >= ps_svc_sts->u16_start_hndl &&
                u16_handle <= ps_svc_sts->u16_end_hndl) {
                // エラー
                return ps_svc_sts;
            }
        }
    }
    // 対象サービスなし
    return NULL;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_get_db
 *
 * DESCRIPTION:GATTサーバーからのアトリビュートDBの取得処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gattc_svc_status_t*   ps_service  R   ステータス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_get_db(ts_gattc_svc_status_t* ps_service) {
    //==========================================================================
    // アトリビュートの個数を取得してサービス毎のアトリビュートDB情報を取得
    //==========================================================================
    // コネクションステータスの取得
    ts_gattc_con_status_t* ps_con_sts = ps_service->ps_con_sts;
    // アトリビュートの個数を取得
    esp_gatt_status_t ret_sts = esp_ble_gattc_get_attr_count(ps_con_sts->t_gatt_if,
                                                             ps_con_sts->u16_con_id,
                                                             ESP_GATT_DB_ALL,
                                                             ps_service->u16_start_hndl,
                                                             ps_service->u16_end_hndl,
                                                             INVALID_HANDLE,
                                                             &ps_service->u16_db_elem_cnt);
    if (ret_sts != ESP_GATT_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    // アトリビュート領域の解放
    if (ps_service->ps_db_elems != NULL) {
        l_mem_free(ps_service->ps_db_elems);
    }
    // アトリビュート領域の生成
    ps_service->ps_db_elems = (esp_gattc_db_elem_t*)pv_mem_malloc(ps_service->u16_db_elem_cnt * sizeof(esp_gattc_db_elem_t));
    if (ps_service->ps_db_elems == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // GATTアトリビュートDB情報の取得
    ret_sts = esp_ble_gattc_get_db(ps_con_sts->t_gatt_if,
                                   ps_con_sts->u16_con_id,
                                   ps_service->u16_start_hndl,
                                   ps_service->u16_end_hndl,
                                   ps_service->ps_db_elems,
                                   &ps_service->u16_db_elem_cnt);
    if(ret_sts != ESP_GATT_OK){
        return ESP_ERR_INVALID_RESPONSE;
    }
#ifdef COM_BLE_DEBUG
    //==========================================================================
    // DEBUG START
    //==========================================================================
    // 一時変数に代入
    esp_gattc_db_elem_t* pt_db_elems = ps_service->ps_db_elems;
    // 取得したアトリビュートDB情報を表示
    ESP_LOGI(LOG_TAG,"%s:get_db con_id = %d elem_count = %d",__func__, ps_con_sts->u16_con_id, ps_service->u16_db_elem_cnt);
    for (int i = 0; i < ps_service->u16_db_elem_cnt; i++) {
        switch(pt_db_elems[i].type){
        case ESP_GATT_DB_PRIMARY_SERVICE:
            ESP_LOGI(LOG_TAG,"attr_type = PRIMARY_SERVICE handle=%d start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle, pt_db_elems[i].end_handle,
                    pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        case ESP_GATT_DB_SECONDARY_SERVICE:
            ESP_LOGI(LOG_TAG,"attr_type = SECONDARY_SERVICE handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle,
                    pt_db_elems[i].end_handle, pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        case ESP_GATT_DB_CHARACTERISTIC:
            ESP_LOGI(LOG_TAG,"attr_type = CHARACTERISTIC handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle,
                    pt_db_elems[i].end_handle, pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        case ESP_GATT_DB_DESCRIPTOR:
            ESP_LOGI(LOG_TAG,"attr_type = DESCRIPTOR handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle, pt_db_elems[i].end_handle,
                    pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        case ESP_GATT_DB_INCLUDED_SERVICE:
            ESP_LOGI(LOG_TAG,"attr_type = INCLUDED_SERVICE handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle, pt_db_elems[i].end_handle,
                    pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        case ESP_GATT_DB_ALL:
            ESP_LOGI(LOG_TAG,"attr_type = ESP_GATT_DB_ALL handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle,
                    pt_db_elems[i].end_handle, pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        default:
            ESP_LOGI(LOG_TAG,"attr_type = DEFAULT(%d) handle=%d,start_handle=%d end_handle=%d properties=0x%X uuid=0x%04X",
                    pt_db_elems[i].type, pt_db_elems[i].attribute_handle, pt_db_elems[i].start_handle,
                    pt_db_elems[i].end_handle, pt_db_elems[i].properties, pt_db_elems[i].uuid.uuid.uuid16);
            break;
        }
    }
    //==========================================================================
    // DEBUG END
    //==========================================================================
#endif

    // 結果ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_register_for_notify
 *
 * DESCRIPTION:GATTサーバーからの通知対象としてGATTクライアントを登録
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gattc_svc_status_t*   ps_svc_sts  R   GATTサービスステータス
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_register_for_notify(ts_gattc_svc_status_t* ps_svc_sts) {
    //==========================================================================
    // GATTサーバーからGATTクライアントへの通知有効化処理
    //==========================================================================
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts = ps_svc_sts->ps_con_sts;
    // キャラクタリスティック数を取得
    uint16_t u16_count = 0;
    esp_gatt_status_t e_gatt_ret;
    e_gatt_ret = esp_ble_gattc_get_attr_count(ps_con_sts->t_gatt_if,
                                              ps_con_sts->u16_con_id,
                                              ESP_GATT_DB_CHARACTERISTIC,
                                              ps_svc_sts->u16_start_hndl,
                                              ps_svc_sts->u16_end_hndl,
                                              INVALID_HANDLE,
                                              &u16_count);
    // 検索結果判定
    if (e_gatt_ret != ESP_GATT_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    // 対象の有無を判定
    if (u16_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    // アトリビュートDBのローカルキャッシュからキャラクタリスティックリストを取得
    esp_gattc_char_elem_t t_char_elem_rst[u16_count];
    e_gatt_ret = esp_ble_gattc_get_all_char(ps_con_sts->t_gatt_if,
                                            ps_con_sts->u16_con_id,
                                            ps_svc_sts->u16_start_hndl,
                                            ps_svc_sts->u16_end_hndl,
                                            t_char_elem_rst,
                                            &u16_count,
                                            0);
    // 結果判定
    if (e_gatt_ret != ESP_GATT_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // Notifyの有効化
    int i_idx;
    for (i_idx = 0; i_idx < u16_count; i_idx++) {
        // Notify通知可能なアトリビュートについて、Notify有効化
        if ((t_char_elem_rst[i_idx].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0x00) {
            continue;
        }
        sts_val = esp_ble_gattc_register_for_notify(ps_con_sts->t_gatt_if,
                                                    ps_con_sts->t_bda,
                                                    t_char_elem_rst[i_idx].char_handle);
        // 結果判定
        if (sts_val != ESP_OK) {
            return sts_val;
        }
    }

    // 結果ステータス返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: e_gattc_get_auth_req
 *
 * DESCRIPTION:GATTサーバーへの有効でセキュアなアクセス権限の取得処理
 *
 * PARAMETERS:          Name            RW  Usage
 * esp_ble_auth_req_t   t_auth_req      R   認証モード
 *
 * RETURNS:
 *   esp_gatt_auth_req_t:アクセス権限
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatt_auth_req_t e_gattc_get_auth_req(esp_ble_auth_req_t t_auth_req) {
    if (t_auth_req == ESP_LE_AUTH_REQ_SC_MITM || (t_auth_req & 0x01) == 0x01) {
        return ESP_GATT_AUTH_REQ_SIGNED_MITM;
    }
    // 結果返信
    return ESP_GATT_AUTH_REQ_SIGNED_NO_MITM;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_write_cccd
 *
 * DESCRIPTION:GATTサーバーへのCCCD書き込み処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_svc_status_t*   ps_service      R   GATTクライアントステータス
 * uint16_t                 u16_char_handle R   キャラクタリスティックハンドル
 * uint8_t*                 pu8_value       R   データ
 * esp_gatt_write_type_t    e_write_type    R   書き込みタイプ
 * esp_gatt_auth_req_t      e_auth_req      R   認証リクエスト
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_write_cccd(ts_gattc_svc_status_t* ps_service,
                                       uint16_t u16_char_handle,
                                       uint8_t u8_value,
                                       esp_gatt_write_type_t e_write_type,
                                       esp_gatt_auth_req_t e_auth_req) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (u8_value > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    // コネクションステータス
    ts_gattc_con_status_t* ps_con_sts = ps_service->ps_con_sts;
    // ディスクリプタの個数を検索
    uint16_t u16_count = 0;
    esp_gatt_status_t e_gatt_ret = esp_ble_gattc_get_attr_count(ps_con_sts->t_gatt_if,
                                                                ps_con_sts->u16_con_id,
                                                                ESP_GATT_DB_DESCRIPTOR,
                                                                ps_service->u16_start_hndl,
                                                                ps_service->u16_end_hndl,
                                                                u16_char_handle,
                                                                &u16_count);
    if (e_gatt_ret != ESP_GATT_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    // ディスクリプタが無いか判定
    if (u16_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    // ディスクリプタの情報を取得
    esp_gattc_descr_elem_t t_descr_elem[u16_count];
    e_gatt_ret = esp_ble_gattc_get_all_descr(ps_con_sts->t_gatt_if,
                                             ps_con_sts->u16_con_id,
                                             u16_char_handle,
                                             t_descr_elem,
                                             &u16_count,
                                             0);
    if (e_gatt_ret != ESP_GATT_OK){
        return ESP_ERR_INVALID_RESPONSE;
    }
    // 各CCCDへの書き込み処理
    uint16_t u16_value = u8_value;
    esp_err_t sts_val = ESP_OK;
    int i_idx;
    for (i_idx = 0; i_idx < u16_count; i_idx++) {
        if (t_descr_elem[i_idx].uuid.len != ESP_UUID_LEN_16 ||
            t_descr_elem[i_idx].uuid.uuid.uuid16 != ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            continue;
        }
        // CCCDへの書き込み
        sts_val = esp_ble_gattc_write_char_descr(ps_con_sts->t_gatt_if,
                                                 ps_con_sts->u16_con_id,
                                                 t_descr_elem[i_idx].handle,
                                                 sizeof(uint16_t),
                                                 (uint8_t*)&u16_value,
                                                 e_write_type,
                                                 e_auth_req);
        // 結果判定
        if (sts_val != ESP_OK) {
            return sts_val;
        }
    }
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_gattc_evt_com_cb
 *
 * DESCRIPTION:GATTクライアントの共通イベントハンドラ
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gattc_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gattc_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gattc_evt_com_cb(esp_gattc_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gattc_cb_param_t* pu_param) {
    //==========================================================================
    // GATTクライアント共通イベント処理
    //==========================================================================
#ifdef COM_BLE_DEBUG
    // イベントメッセージ
    ESP_LOGI(LOG_TAG, "GATTC_EVT=%s gatt_if=0x%x", pc_com_ble_gattc_event_to_str(e_event), t_gatt_if);
#endif

    //==========================================================================
    // インターフェース登録判定
    // ※GATTクライアントインターフェースとアプリケーションＩＤは１対１の関係
    //==========================================================================
    /** GATTクライアントイベント処理中のアプリケーション設定 */
    ts_com_ble_gattc_if_config_t* ps_if_cfg = NULL;
    if (e_event == ESP_GATTC_REG_EVT) {
        //----------------------------------------------------------------------
        // クリティカルセクション開始
        //----------------------------------------------------------------------
        if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
            return;
        }
        //----------------------------------------------------------------------
        // アプリケーションIDの設定通知イベント
        //----------------------------------------------------------------------
        sts_gattc_evt_register(t_gatt_if, pu_param);
        //----------------------------------------------------------------------
        // クリティカルセクション終了
        //----------------------------------------------------------------------
        xSemaphoreGiveRecursive(s_mutex);
        return;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_UTIL_BLOCK_TIME) != pdTRUE) {
        return;
    }

    //==========================================================================
    // インターフェース登録判定以外のGATTクライアント共通イベント処理
    //==========================================================================
    // GAPステータス
    ts_gap_status_t* ps_gap_sts = &s_gap_ctrl.s_status;
    /** GATTクライアントイベント処理ステータス */
    ts_gattc_if_status_t*  ps_if_sts = NULL;
    ts_gattc_con_status_t* ps_con_sts = NULL;
    ts_gattc_svc_status_t* ps_svc_sts = NULL;
    // GATTインターフェース毎のループ
    int i_if_idx;
    for (i_if_idx = 0; i_if_idx < s_gattc_ctrl.u16_if_count; i_if_idx++) {
        //======================================================================
        // 対象GATTインターフェースの判定
        //======================================================================
        if (t_gatt_if != ESP_GATT_IF_NONE &&
            t_gatt_if != s_gattc_ctrl.ps_if_status[i_if_idx].t_gatt_if) {
            continue;
        }
        ps_if_cfg = &s_gattc_ctrl.ps_if_config[i_if_idx];
        ps_if_sts = &s_gattc_ctrl.ps_if_status[i_if_idx];

        //======================================================================
        // GATTC共通処理 Start
        //======================================================================
        // 結果ステータス
        esp_err_t sts_val = ESP_OK;
        // アプリケーションIDの登録通知の有無を判定
        /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        switch (e_event) {
        case ESP_GATTC_UNREG_EVT:
            break;
        case ESP_GATTC_OPEN_EVT:
            // 論理的なコネクションの確立完了イベント
            // ESP_GATTC_CONNECT_EVTの後にこのイベントが発生する
            // GATTサーバーとの接続開始通知イベント
            sts_gattc_evt_open(ps_if_sts, &pu_param->open);
            break;
        case ESP_GATTC_READ_CHAR_EVT:
            // 読み込みデータの受信イベント
            sts_gattc_evt_read(e_event, ps_if_sts, &pu_param->read);
            break;
        case ESP_GATTC_CLOSE_EVT:
            // コネクションの削除
            v_gattc_del_con_status(ps_if_sts, pu_param->close.remote_bda);
            break;
        case ESP_GATTC_SEARCH_CMPL_EVT:
            // 全てのサービスの検出完了通知イベント
            if (pu_param->search_cmpl.status != ESP_GATT_OK) {
                break;
            }
#ifdef COM_BLE_DEBUG
            ESP_LOGI(LOG_TAG, "ESP_GATTC:gatt_if = %d, conn_id=%d", t_gatt_if, pu_param->search_cmpl.conn_id);
            if(pu_param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
                // リモートデバイスからのサービス情報の場合
                ESP_LOGI(LOG_TAG, "ESP_GATTC:get service information from remote device");
            } else if (pu_param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
                // リモートデバイスのNVSフラッシュからの場合
                ESP_LOGI(LOG_TAG, "ESP_GATTC:get service information from flash");
            } else {
                ESP_LOGI(LOG_TAG, "ESP_GATTC:unknown service source");
            }
#endif
            // コネクションステータスを取得
            ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, pu_param->search_cmpl.conn_id);
            if (ps_con_sts == NULL) {
                break;
            }
            // サービス毎のアトリビュートDB情報を取得
            for (ps_svc_sts = ps_con_sts->ps_svc_sts; ps_svc_sts != NULL; ps_svc_sts = ps_svc_sts->ps_next) {
                sts_val = sts_gattc_get_db(ps_svc_sts);
                if (sts_val != ESP_OK) {
                    break;
                }
#ifdef COM_BLE_DEBUG
                if (ps_svc_sts->ps_db_elems != NULL) {
                    ESP_LOGI(LOG_TAG,"ESP_GATTC:DB IS NOT NULL");
                } else {
                    ESP_LOGI(LOG_TAG,"ESP_GATTC:DB IS NULL");
                }
#endif
            }
            // コネクションステータス更新
            ps_con_sts->u8_status |= GATTC_STS_SET_SVC;
            ps_con_sts->u8_status &= ~GATTC_STS_SEARCH_SVC;
            break;
        case ESP_GATTC_SEARCH_RES_EVT:
            // GATTサービスの検索結果通知イベント、１件見つかる毎にイベント通知を受ける
            sts_gattc_evt_search_result(ps_if_sts, &pu_param->search_res);
            break;
        case ESP_GATTC_READ_DESCR_EVT:
            // ディスクリプタの読み込みイベント
            sts_gattc_evt_read(e_event, ps_if_sts, &pu_param->read);
            break;
        case ESP_GATTC_WRITE_DESCR_EVT:
            // ディスクリプタの書き込みイベント
            break;
        case ESP_GATTC_NOTIFY_EVT:
            // サーバーからの通知イベント
            sts_gattc_evt_notify(ps_if_sts, &pu_param->notify);
            break;
        case ESP_GATTC_SRVC_CHG_EVT:
            // GATTサービスの変更通知の受信イベント
            // コネクションステータスの取得
            ps_con_sts = ps_gattc_get_con_status_bda(t_gatt_if, pu_param->srvc_chg.remote_bda);
            if (ps_con_sts == NULL) {
                break;
            }
            // ステータス更新
            ps_con_sts->u8_status &= ~GATTC_STS_SET_SVC;
            // サービスの再検索
            sts_gattc_search_service(ps_con_sts->t_bda);
            break;
        case ESP_GATTC_CFG_MTU_EVT:
            // フレームサイズの設定完了通知イベント
            // ※論理的なコネクション確立後に必ず設定が行われる
            if (pu_param->cfg_mtu.status != ESP_GATT_OK) {
                break;
            }
            // コネクションステータス
            ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, pu_param->cfg_mtu.conn_id);
            if (ps_con_sts == NULL) {
                break;
            }
            // MTUサイズの保存
            ps_con_sts->u16_mtu = pu_param->cfg_mtu.mtu;
            ps_con_sts->u8_status |= GATTC_STS_SET_MTU;
            ps_con_sts->u8_status &= ~GATTC_STS_REQUEST_MTU;
            // GATTサービス検索
            sts_gattc_search_service(ps_con_sts->t_bda);
            break;
        case ESP_GATTC_CONNECT_EVT:
            // 物理的なコネクションの確立完了イベント
            // GAPステータスを更新（スキャン実行中の場合には停止する）
            ps_gap_sts->u32_status &= ~GAP_STS_START_SCAN;
            ps_gap_sts->u32_scan_duration = 0;
            ps_gap_sts->i64_scan_timeout  = 0;
#ifdef COM_BLE_DEBUG
            do {
                // 接続イベント ※接続時に全インターフェースで発生する
                tc_com_ble_bda_string_t tc_bda;
                v_com_ble_address_to_str(tc_bda, pu_param->connect.remote_bda);
                ESP_LOGI(LOG_TAG, "ESP_GATTC: gatt_if = %d, conn_id=%d", t_gatt_if, pu_param->connect.conn_id);
                ESP_LOGI(LOG_TAG, "ESP_GATTC:     bda = %s", tc_bda);
            } while(false);
#endif
            break;
        case ESP_GATTC_DISCONNECT_EVT:
            // GATTサーバーとの物理的な切断通知イベント
            // GATTクライアントの接続ステータスを解放
            v_gattc_del_con_status(ps_if_sts, pu_param->disconnect.remote_bda);
            // 物理切断なので、GAPデバイス情報も削除する
            sts_gap_del_device(pu_param->disconnect.remote_bda);
            break;
        case ESP_GATTC_GET_ADDR_LIST_EVT:
            // キャッシュしているデータベースのアドレスリストの通知イベント
            if (ps_if_sts->b_req_cache_clear) {
                // ローカルデータベースのクリア処理
                uint8_t u8_num_addr = pu_param->get_addr_list.num_addr;
                esp_bd_addr_t* ps_addr_list = pu_param->get_addr_list.addr_list;
                uint8_t u8_idx;
                for (u8_idx = 0; u8_idx < u8_num_addr; u8_idx++) {
                    if (esp_ble_gattc_cache_refresh(ps_addr_list[u8_idx]) != ESP_OK) {
                        break;
                    }
                }
            }
            break;
        default:
            break;
        }

        //======================================================================
        // GATTC共通処理 End
        //======================================================================
        // コールバック関数が有れば実行
        if (ps_if_cfg->fc_gattc_cb != NULL) {
            ps_if_cfg->fc_gattc_cb(e_event, t_gatt_if, pu_param);
        }
        // インターフェース設定をクリア
        ps_if_cfg = NULL;
        // インターフェースステータスをクリア
        ps_if_sts = NULL;
        // コネクションステータスをクリア
        ps_con_sts = NULL;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_gattc_evt_dmy_cb
 *
 * DESCRIPTION:GATTクライアントのダミーイベントハンドラ
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gattc_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gattc_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_gattc_evt_dmy_cb(esp_gattc_cb_event_t e_event,
                                esp_gatt_if_t t_gatt_if,
                                esp_ble_gattc_cb_param_t* pu_param) {
    return;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_evt_register
 *
 * DESCRIPTION:GATTクライアントのESP_GATTC_REG_EVTイベント処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatt_if_t*               t_gatt_if   R   GATTインターフェース
 * esp_ble_gattc_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_evt_register(esp_gatt_if_t t_gatt_if,
                                         esp_ble_gattc_cb_param_t* pu_param) {
    // アプリケーションIDの設定通知イベント
    if (pu_param->reg.status != ESP_GATT_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    /** GATTクライアントイベント処理中のアプリケーション設定 */
    ts_com_ble_gattc_if_config_t* ps_if_cfg = NULL;
    // GATTクライアントアプリケーションステータス
    ts_gattc_if_status_t* ps_if_sts;
    // インターフェースの探索
    uint16_t u16_idx;
    for (u16_idx = 0; u16_idx < s_gattc_ctrl.u16_if_count; u16_idx++) {
        // 処理対象
        ps_if_cfg = &s_gattc_ctrl.ps_if_config[u16_idx];
        // アプリケーションIDの設定通知イベント
        if (pu_param->reg.app_id != ps_if_cfg->u16_app_id) {
            continue;
        }
        // GATTクライアントのインターフェース設定
        ps_if_sts = &s_gattc_ctrl.ps_if_status[u16_idx];
        ps_if_sts->t_gatt_if = t_gatt_if;
        // コールバック関数が有れば実行
        if (ps_if_cfg->fc_gattc_cb != NULL) {
            ps_if_cfg->fc_gattc_cb(ESP_GATTC_REG_EVT, t_gatt_if, pu_param);
        }
        break;
    }
    // 結果ステータスを返却
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_evt_open
 *
 * DESCRIPTION:GATTクライアントのESP_GATTC_OPEN_EVTイベント処理
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_gattc_if_status_t*    ps_if_sts   R   GATTインターフェース
 * gattc_open_evt_param_t*  pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_evt_open(ts_gattc_if_status_t* ps_if_sts,
                                     struct gattc_open_evt_param* pu_param) {
    if (pu_param->status != ESP_GATT_OK) {
        // コネクションステータス削除
        v_gattc_del_con_status(ps_if_sts, pu_param->remote_bda);
        return ESP_ERR_INVALID_STATE;
    }
    // GAPデバイスステータスの取得
    // ※事前にスキャン等で把握していないデバイスはエラーとする
    ts_gap_device_t* ps_gap_dev = ps_gap_get_device(pu_param->remote_bda);
    if (ps_gap_dev == NULL) {
        // コネクションステータス削除
        v_gattc_del_con_status(ps_if_sts, pu_param->remote_bda);
        return ESP_ERR_INVALID_STATE;
    }
    // コネクションステータスを取得
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_bda(ps_if_sts->t_gatt_if, pu_param->remote_bda);
    if (ps_con_sts == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((ps_con_sts->u8_status & GATTC_STS_OPEN) == GATTC_STS_OPEN) {
        // 既にオープンされている場合
        // コネクションステータスを削除
        v_gattc_del_con_status(ps_if_sts, pu_param->remote_bda);
        // エラーとする
        return ESP_ERR_INVALID_STATE;
    }
    // コネクションステータスを更新
    // アドレス編集
    v_com_ble_addr_cpy(ps_con_sts->t_bda, pu_param->remote_bda);
    // セキュアアクセスモード
    ps_con_sts->e_sec_auth_req = e_gattc_get_auth_req(ps_gap_dev->t_auth_mode);
    // コネクションID
    ps_con_sts->u16_con_id = pu_param->conn_id;
    // コネクションオープン
    ps_con_sts->u8_status |= GATTC_STS_OPEN;
    ps_con_sts->u8_status &= ~GATTC_STS_REQUEST_OPEN;
    // MTUサイズの設定リクエスト
    ps_con_sts->u8_status |= GATTC_STS_REQUEST_MTU;
    ps_con_sts->u8_status &= ~GATTC_STS_SET_MTU;
    return esp_ble_gattc_send_mtu_req(ps_if_sts->t_gatt_if, pu_param->conn_id);
}

/*******************************************************************************
 *
 * NAME: sts_gattc_evt_read
 *
 * DESCRIPTION:GATTクライアントのESP_GATTC_READ_CHAR_EVTイベント処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gattc_cb_event_t         e_event     R   GATTイベントタイプ
 * ts_gattc_if_status_t*        ps_if_sts   R   GATTインターフェースステータス
 * gattc_read_char_evt_param*   pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_evt_read(esp_gattc_cb_event_t e_event,
                                     ts_gattc_if_status_t* ps_if_sts,
                                     struct gattc_read_char_evt_param* pu_param) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // ステータス判定
    if (pu_param->status != ESP_GATT_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    // 受信サイズ判定
    if (pu_param->value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // サービスステータスの取得
    ts_gattc_svc_status_t* ps_svc_sts =
        ps_gattc_get_handle_svc_status(ps_if_sts, pu_param->handle);
    if (ps_svc_sts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // コネクションステータス取得
    ts_gattc_con_status_t* ps_con_sts = ps_svc_sts->ps_con_sts;
    // リモートデバイスアドレス
    if (pu_param->conn_id != ps_con_sts->u16_con_id) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // 受信データ編集
    //==========================================================================
    ts_com_ble_gatt_rx_data_t* ps_rx_data = pv_mem_malloc(sizeof(ts_com_ble_gatt_rx_data_t));
    if (ps_rx_data == NULL) {
        // 結果ステータス返信
        return ESP_ERR_NO_MEM;
    }
    // アプリケーションID   ※キー１
    ps_rx_data->u16_app_id = ps_con_sts->u16_app_id;
    // GATTインターフェース ※キー２
    ps_rx_data->t_gatt_if = ps_con_sts->t_gatt_if;
    // コネクションID       ※キー３
    ps_rx_data->u16_con_id = ps_con_sts->u16_con_id;
    // リモートデバイスアドレス
    v_com_ble_addr_cpy(ps_rx_data->t_bda, ps_con_sts->t_bda);
    // 受信データタイプ
    if (e_event == ESP_GATTC_READ_CHAR_EVT) {
        ps_rx_data->e_type = GATT_RX_TYPE_READ_DATA;
    } else {
        ps_rx_data->e_type = GATT_RX_TYPE_READ_DESC;
    }
    // 結果ステータス
    ps_rx_data->t_status = ESP_GATT_OK;
    // アトリビュートハンドル
    ps_rx_data->u16_attr_hndl = pu_param->handle;
    // サービスインデックス
    ps_rx_data->u8_svc_idx = ps_svc_sts->u8_svc_idx;
    // アトリビュートハンドルインデックス
    ps_rx_data->u16_hndl_idx = pu_param->handle - ps_svc_sts->u16_start_hndl;
    // 自動返信
    ps_rx_data->b_auto_rsp = false;
    // 受信データ配列　※解放時はFreeする事！！！
    ps_rx_data->ps_array = ps_mdl_clone_u8_array(pu_param->value, pu_param->value_len);

    //==========================================================================
    // 受信データエンキュー
    //==========================================================================
    // データをエンキュー
    if (xQueueSend(ps_svc_sts->t_rx_queue, &ps_rx_data, 0) != pdTRUE) {
        return ESP_FAIL;
    }

    // 結果ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_evt_search_result
 *
 * DESCRIPTION:GATTクライアントのサービス検索結果通知
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_gattc_if_status_t*        ps_if_sts   R   GATTインターフェースステータス
 * gattc_search_res_evt_param*  pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_evt_search_result(ts_gattc_if_status_t* ps_if_sts,
                                              struct gattc_search_res_evt_param* pu_param) {
    // サービスステータスの取得
    ts_gattc_svc_status_t* ps_svc_sts = ps_gattc_add_svc_status(ps_if_sts, pu_param->conn_id, &pu_param->srvc_id);
    if (ps_svc_sts == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // サービス情報の編集
    ps_svc_sts->b_primary       = pu_param->is_primary;     // プライマリサービスフラグ
    ps_svc_sts->u16_start_hndl  = pu_param->start_handle;   // アトリビュートハンドル（開始）
    ps_svc_sts->u16_end_hndl    = pu_param->end_handle;     // アトリビュートハンドル（終了）
    ps_svc_sts->u16_db_elem_cnt = 0;                        // アトリビュートDBの要素数
    ps_svc_sts->ps_db_elems     = NULL;                     // アトリビュートDB情報
    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_gattc_evt_notify
 *
 * DESCRIPTION:GATTクライアントのESP_GATTC_NOTIFY_EVTイベント処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_gattc_if_status_t*        ps_if_sts   R   GATTインターフェースステータス
 * gattc_notify_evt_param*      pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_gattc_evt_notify(ts_gattc_if_status_t* ps_if_sts,
                                       struct gattc_notify_evt_param* pu_param) {
    //==========================================================================
     // GATTサービスからの通知または通告の受信通知イベント
    //==========================================================================
    // サービスステータスの取得
    ts_gattc_svc_status_t* ps_svc_sts =
        ps_gattc_get_handle_svc_status(ps_if_sts, pu_param->handle);
    if (ps_svc_sts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // コネクションステータス取得
    ts_gattc_con_status_t* ps_con_sts = ps_svc_sts->ps_con_sts;
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // リモートデバイスアドレス
    if (l_com_ble_addr_cmp(pu_param->remote_bda, ps_con_sts->t_bda) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // 受信サイズ判定
    uint16_t u16_rcv_len = pu_param->value_len;
    if (u16_rcv_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // GATTサーバーからのNotify受信
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    ts_u8_array_t* ps_array = NULL;
    uint8_t* pu8_rcv_val = pu_param->value;
    do {
        //----------------------------------------------------------------------
        // 分割されていないパケットの受信
        //----------------------------------------------------------------------
        if (u16_rcv_len < 2) {
            ps_array = ps_mdl_clone_u8_array(pu8_rcv_val, u16_rcv_len);
            break;
        } else if ((pu8_rcv_val[0] != '#') || (pu8_rcv_val[1] != '#')) {
            ps_array = ps_mdl_clone_u8_array(pu8_rcv_val, u16_rcv_len);
            break;
        }

        //----------------------------------------------------------------------
        // 分割パケットを受信
        //----------------------------------------------------------------------
        // 入力チェック
        if (u16_rcv_len <= 4) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // パケット番号チェック
        uint8_t u8_pkt_all = pu8_rcv_val[2];    // 全パケット数
        uint8_t u8_pkt_num = pu8_rcv_val[3];    // 現在パケット番号
        if (u8_pkt_num > u8_pkt_all) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // Notifyステータスを取得
        ts_gattc_rcv_notify_status_t* ps_notify = &ps_svc_sts->s_notify;
        // 先頭パケット判定
        if (u8_pkt_num == 1) {
            // 先頭パケットの処理
            if (ps_notify->u8_pkt_num != 0) {
                // 受信中のデータが有れば、エラーデータとしてクリア
                sts_mdl_clear_linked_queue(ps_notify->ps_queue);
            }
            // 受信中のハンドル
            ps_notify->u16_handle = pu_param->handle;
            // 受信中の全パケット数
            ps_notify->u8_pkt_all = u8_pkt_all;
            // 受信中の現在パケット番号
            ps_notify->u8_pkt_num = u8_pkt_num;
        } else {
            // 先頭パケット以外の処理
            // ハンドル妥当性チェック
            if (pu_param->handle != ps_notify->u16_handle) {
                sts_val = ESP_ERR_INVALID_ARG;
                break;
            }
            // パケット番号判定
            if (u8_pkt_all != ps_notify->u8_pkt_all) {
                sts_val = ESP_ERR_INVALID_ARG;
                break;
            }
            // パケット番号判定
            if (u8_pkt_num != (ps_notify->u8_pkt_num + 1)) {
                sts_val = ESP_ERR_INVALID_ARG;
                break;
            }
            // パケット番号更新
            ps_notify->u8_pkt_num = u8_pkt_num;
        }
        // Notifyバッファにエンキュー
        ts_linked_queue_t* ps_queue = ps_notify->ps_queue;
        sts_mdl_linked_enqueue(ps_queue, &pu8_rcv_val[4], u16_rcv_len - 4);
        // 終端パケット判定
        if (u8_pkt_num != u8_pkt_all) {
            return sts_val;
        }
        // 受信データ情報コピー
        ps_array = ps_mdl_linked_dequeue(ps_queue, ps_queue->t_size);
    } while (false);

    //==========================================================================
    // 受信データエンキュー
    //==========================================================================
    // 受信データ編集
    ts_com_ble_gatt_rx_data_t* ps_rx_data = pv_mem_malloc(sizeof(ts_com_ble_gatt_rx_data_t));
    if (ps_rx_data == NULL) {
        // 結果ステータス返信
        return ESP_ERR_NO_MEM;
    }
    // アプリケーションID   ※キー１
    ps_rx_data->u16_app_id = ps_con_sts->u16_app_id;
    // GATTインターフェース ※キー２
    ps_rx_data->t_gatt_if = ps_con_sts->t_gatt_if;
    // コネクションID       ※キー３
    ps_rx_data->u16_con_id = ps_con_sts->u16_con_id;
    // リモートデバイスアドレス
    v_com_ble_addr_cpy(ps_rx_data->t_bda, ps_con_sts->t_bda);
    // Notify判定
    if (pu_param->is_notify) {
        // 受信データタイプ
        ps_rx_data->e_type = GATT_RX_TYPE_NOTIFY;
    } else {
        // 受信データタイプ
        ps_rx_data->e_type = GATT_RX_TYPE_INDICATE;
    }
    // 結果ステータス
    ps_rx_data->t_status = ESP_GATT_OK;
    // アトリビュートハンドル
    ps_rx_data->u16_attr_hndl = pu_param->handle;
    // サービスインデックス
    ps_rx_data->u8_svc_idx = ps_svc_sts->u8_svc_idx;
    // アトリビュートハンドルインデックス
    ps_rx_data->u16_hndl_idx = pu_param->handle - ps_svc_sts->u16_start_hndl;
    // 自動返信
    ps_rx_data->b_auto_rsp = false;
    // 受信データ配列　※解放時はFreeする事！！！
    ps_rx_data->ps_array = ps_array;
    // データをエンキュー
    if (xQueueSend(ps_svc_sts->t_rx_queue, &ps_rx_data, 0) != pdTRUE) {
        sts_val = ESP_FAIL;
    }

    // 結果ステータス返信
    return sts_val;
}

//==============================================================================
// SPPプロファイルのサーバー側処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: v_spps_evt_cb
 *
 * DESCRIPTION:GATTプロファイルのイベントハンドラ関数（SPP用）
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gatts_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gatts_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_spps_evt_cb(esp_gatts_cb_event_t e_event,
                           esp_gatt_if_t t_gatt_if,
                           esp_ble_gatts_cb_param_t* pu_param) {
    // パラメータタイプ
    struct gatts_write_evt_param* ps_write;
    struct gatts_add_attr_tab_evt_param* ps_add_attr_tab;
    // GATTインターフェースステータス
    ts_gatts_if_status_t* ps_if_sts;
    // GATTサーバーサービスステータス
    ts_gatts_svc_status_t* ps_svc_sts;
    // SPP受信データ
    ts_spps_status_t* ps_spp_sts;
    ts_spps_status_t* ps_bef_sts;
    // 結果ステータス
    esp_err_t sts_val;
    // 各種インデックス
    uint8_t u8_svc_idx;
    uint16_t u16_hndl_idx;
    // イベント判定
    switch (e_event) {
    case ESP_GATTS_REG_EVT:
        // アプリケーションIDの登録イベント
        //======================================================================
        // 入力チェック
        //======================================================================
        if (pu_param->reg.status != ESP_GATT_OK) {
            break;
        }
        // GATTサーバーイベント処理中のアプリケーションステータス
        ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        //======================================================================
        // SPPステータスの探索
        //======================================================================
        ps_bef_sts = NULL;
        for (ps_spp_sts = ps_spps_status; ps_spp_sts != NULL; ps_spp_sts = ps_spp_sts->ps_next) {
            if (ps_spp_sts->t_gatt_if == t_gatt_if) {
                break;
            }
            ps_bef_sts = ps_spp_sts;
        }
        if (ps_spp_sts != NULL) {
            break;
        }
        //======================================================================
        // SPPステータスの初期化処理
        //======================================================================
        esp_gatts_attr_db_t* ps_attr;
        esp_attr_desc_t* ps_desc;
        tu_type_converter_t u_cnv;
        uint16_t u16_uuid;
        uint16_t u16_value;
        for (u8_svc_idx = 0; u8_svc_idx < ps_if_sts->u8_svc_cnt; u8_svc_idx++) {
            // サービスの情報取得
            ps_attr = ps_gatts_get_attribute(ps_if_sts, u8_svc_idx, 0);
            ps_desc = &ps_attr->att_desc;
            // SPP判定
            if (ps_desc->uuid_length != 2) {
                continue;
            }
            u_cnv.u8_values[0] = ps_desc->uuid_p[0];
            u_cnv.u8_values[1] = ps_desc->uuid_p[1];
            u16_uuid = u_cnv.u16_values[0];
            if (u16_uuid != ESP_GATT_UUID_PRI_SERVICE) {
                continue;
            }
            u_cnv.u8_values[0] = ps_desc->value[0];
            u_cnv.u8_values[1] = ps_desc->value[1];
            u16_value = u_cnv.u16_values[0];
            if (u16_value != BLE_SPPS_UUID_SERVICE) {
                continue;
            }
            // SPPステータス判定
            ps_svc_sts = &ps_if_sts->ps_svc_sts[u8_svc_idx];
            ps_spp_sts = pv_mem_malloc(sizeof(ts_spps_status_t));
            ps_spp_sts->t_gatt_if         = t_gatt_if;  // GATTインターフェース ※キー１
            ps_spp_sts->u8_svc_idx        = u8_svc_idx; // サービスインデックス ※キー２
            ps_spp_sts->u8_svc_inst_id    = ps_svc_sts->u8_svc_inst_id; // GATTインターフェース
            ps_spp_sts->b_notify_data     = false;      // Data Notifyの有効フラグ
            ps_spp_sts->b_notify_status   = false;      // Status Notifyの有効フラグ
            ps_spp_sts->u16_hndl_data_ntf = 0;          // アトリビュートハンドル：データ通知
            ps_spp_sts->ps_next           = NULL;       // 次のステータス
            // ステータスの追加
            if (ps_bef_sts != NULL) {
                ps_bef_sts->ps_next = ps_spp_sts;
            } else {
                ps_spps_status = ps_spp_sts;
            }
            ps_bef_sts = ps_spp_sts;
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        // GATTクライアントからの書き込みリクエストの受信通知イベント
        //======================================================================
        // 入力チェック
        //======================================================================
        ps_write = &pu_param->write;
        // 分割パケットのバッファリング判定
        if (ps_write->is_prep) {
            break;
        }
        // GATTサーバーイベント処理中のアプリケーションステータス
        ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // アトリビュートのハンドルインデックス取得
        sts_val = sts_gatts_get_handle_idx(ps_if_sts, ps_write->handle, &u8_svc_idx, &u16_hndl_idx);
        if (sts_val != ESP_OK) {
            break;
        }
        // SPPステータス
        ps_spp_sts = ps_spps_get_status(t_gatt_if, u8_svc_idx);
        if (ps_spp_sts == NULL) {
            break;
        }
        //======================================================================
        // CCCDの更新処理
        //======================================================================
        // SPPサービスの通知データ設定（アトリビュート値の更新通知の可否設定）イベント判定
        if (u16_hndl_idx == SPPS_ATTR_IDX_TX_DATA_CFG) {
            // CCCD (Client Characteristic Configuration Descriptor) の書き込み
            if (ps_write->len == 2) {
                // Notifyの有効フラグ更新
                ps_spp_sts->b_notify_data = (((ps_write->value[0] << 8) | ps_write->value[1]) == 0x0100);
            }
        // SPPサービスのステータス通知設定
        } else if (u16_hndl_idx == SPPS_ATTR_IDX_TX_STS_CFG) {
            // CCCD (Client Characteristic Configuration Descriptor) の書き込み
            if (pu_param->write.len == 2) {
                // Notifyの有効フラグ更新
                ps_spp_sts->b_notify_status = (((ps_write->value[0] << 8) | ps_write->value[1])== 0x0100);
            }
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        // 接続通知
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        // 切断通知
        //======================================================================
        // SPPステータスのCCCDをクリア
        //======================================================================
        ps_bef_sts = NULL;
        for (ps_spp_sts = ps_spps_status; ps_spp_sts != NULL; ps_spp_sts = ps_spp_sts->ps_next) {
            if (ps_spp_sts->t_gatt_if == t_gatt_if) {
                // Notifyの有効フラグを初期化
                ps_spp_sts->b_notify_data = false;
                ps_spp_sts->b_notify_status = false;
                break;
            }
            ps_bef_sts = ps_spp_sts;
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        // GATTアトリビュートテーブル生成完了通知イベント
        ps_add_attr_tab = &pu_param->add_attr_tab;
        if (ps_add_attr_tab->status != ESP_GATT_OK) {
            break;
        }
        // GATTサーバーイベント処理中のアプリケーションステータス
        ps_if_sts = ps_gatts_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // SPPステータス
        ps_spp_sts = ps_spps_get_status(t_gatt_if, ps_add_attr_tab->svc_inst_id);
        if (ps_spp_sts == NULL) {
            break;
        }
        // データハンドル
        ps_spp_sts->u16_hndl_data_ntf = ps_add_attr_tab->handles[SPPS_ATTR_IDX_TX_DATA_VAL];
        break;
    default:
        break;

    }
    // ユーザーイベント関数を実行
    fc_spps_usr_evt_cb(e_event, t_gatt_if, pu_param);
}

/*******************************************************************************
 *
 * NAME: ps_spps_get_status
 *
 * DESCRIPTION:GATTサーバーのSPPインターフェースのステータス取得処理
 *
 * PARAMETERS:          Name                RW  Usage
 * esp_gatt_if_t        t_gatt_if           R   GATTインターフェース
 * uint8_t              u8_svc_idx          R   サービスインデックス
 *
 * RETURNS:
 *   ts_spps_con_status 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_spps_status_t* ps_spps_get_status(esp_gatt_if_t t_gatt_if, uint8_t u8_svc_idx) {
    // SPPステータスの初期化処理
    struct s_spps_status_t* ps_chk_sts;
    for (ps_chk_sts = ps_spps_status; ps_chk_sts != NULL; ps_chk_sts = ps_chk_sts->ps_next) {
        if (ps_chk_sts->t_gatt_if == t_gatt_if &&
            ps_chk_sts->u8_svc_idx == u8_svc_idx) {
            return ps_chk_sts;
        }
    }
    return NULL;
}

//==============================================================================
// SPPプロファイルのクライアント側処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: v_sppc_evt_cb
 *
 * DESCRIPTION:SPPクライアントの共通イベントハンドラ
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gattc_cb_event_t         e_event     R   GATTイベントタイプ
 * esp_gatt_if_t                t_gatt_if   R   GATTインターフェースタイプ
 * esp_ble_gattc_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_sppc_evt_cb(esp_gattc_cb_event_t e_event,
                           esp_gatt_if_t t_gatt_if,
                           esp_ble_gattc_cb_param_t* pu_param) {
    // GATTクライアント関係のステータス
    ts_gattc_if_status_t* ps_if_sts;
    ts_gattc_con_status_t* ps_con_sts;
    ts_gattc_svc_status_t* ps_svc_sts;
    ts_sppc_status_t* ps_spp_sts;
    // アトリビュートエレメント
    esp_gattc_db_elem_t* ps_tx_elm;
    esp_gattc_db_elem_t* ps_rx_elm;
    esp_gattc_db_elem_t* ps_cmd_elm;
    // イベント処理
    switch (e_event) {
#ifdef COM_BLE_DEBUG
    case ESP_GATTC_WRITE_CHAR_EVT:
        // GATTサービスへのCharacteristic書き込み完了通知イベント
        if (pu_param->write.status != ESP_GATT_OK) {
            ESP_LOGI(LOG_TAG, "ESP_SPPC:write char failed, status = 0x%04x", pu_param->write.status);
        }
        break;
#endif
    case ESP_GATTC_SEARCH_CMPL_EVT:
        // サービスの検索完了通知イベント
        // インターフェースステータスの取得
        ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // コネクションステータスを取得
        ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, pu_param->search_cmpl.conn_id);
        if (ps_con_sts == NULL) {
            break;
        }
        // サービスの取得済み判定
        if ((ps_con_sts->u8_status & GATTC_STS_SET_SVC) == 0x00) {
            break;
        }
        // SPPサービスを探索
        for (ps_svc_sts = ps_con_sts->ps_svc_sts; ps_svc_sts != NULL; ps_svc_sts = ps_svc_sts->ps_next) {
            // SPPサービス判定
            if (!b_sppc_chk_status(ps_svc_sts)) {
                continue;
            }
            // Notify対象として全てのサービスにGATTクライアントを登録
            if (sts_gattc_register_for_notify(ps_svc_sts) != ESP_OK) {
                continue;
            }
            // アトリビュート情報取得
            ps_tx_elm  = &ps_svc_sts->ps_db_elems[SPPC_ATTR_IDX_TX_DATA_VAL];
            ps_cmd_elm = &ps_svc_sts->ps_db_elems[SPPC_ATTR_IDX_TX_CMD_VAL];
            ps_rx_elm  = &ps_svc_sts->ps_db_elems[SPPC_ATTR_IDX_RX_DATA_VAL];
            // SPPステータスの生成
            ps_spp_sts = ps_sppc_add_status(t_gatt_if, ps_con_sts->u16_con_id);
            if (ps_spp_sts != NULL) {
                ps_spp_sts->ps_con_sts       = ps_con_sts;
                ps_spp_sts->ps_svc_sts       = ps_svc_sts;
                ps_spp_sts->u16_hndl_tx_data = ps_tx_elm->attribute_handle;
                ps_spp_sts->u16_hndl_tx_cmd  = ps_cmd_elm->attribute_handle;
                ps_spp_sts->u16_hndl_rx_data = ps_rx_elm->attribute_handle;
            }
            break;
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        // GATTサービスへのDescriptor書き込み完了通知イベント
        // 結果判定
        if (pu_param->write.status != ESP_GATT_OK){
            break;
        }
        // インターフェースの探索
        ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // 対象がSPPサービスのキャラクタリスティックなのか判定
        ps_svc_sts = ps_gattc_get_handle_svc_status(ps_if_sts, pu_param->write.handle);
        if (!b_sppc_chk_status(ps_svc_sts)) {
            break;
        }
        // SPPCステータス取得
        ps_spp_sts = ps_sppc_get_status(t_gatt_if, pu_param->write.conn_id);
        if (ps_spp_sts != NULL) {
            // Notify有効化済みハンドル
            if (ps_spp_sts->u16_hndl_notify[0] == 0) {
                ps_spp_sts->u16_hndl_notify[0] = pu_param->write.handle;
            } else {
                ps_spp_sts->u16_hndl_notify[1] = pu_param->write.handle;
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        // サービス通知の登録完了イベント
        // キューイング書き込み設定の完了通知イベント
        if (pu_param->reg_for_notify.status != ESP_GATT_OK){
            break;
        }
        // インターフェースの探索
        ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
        if (ps_if_sts == NULL) {
            break;
        }
        // 対象がSPPサービスのキャラクタリスティックか判定
        ps_svc_sts = ps_gattc_get_handle_svc_status(ps_if_sts, pu_param->reg_for_notify.handle);
        if (!b_sppc_chk_status(ps_svc_sts)) {
            break;
        }
        // Notify通知登録が完了したキャラクタリスティックのCCCDを更新して、Notify有効化
        ps_con_sts = ps_svc_sts->ps_con_sts;
        sts_gattc_write_cccd(ps_svc_sts,
                             pu_param->reg_for_notify.handle,
                             0x01,
                             ESP_GATT_WRITE_TYPE_RSP,
                             ps_con_sts->e_sec_auth_req);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        // GATTサーバーとの切断通知イベント
        v_sppc_del_status(t_gatt_if, pu_param->disconnect.conn_id);
        break;
    default:
        break;
    }
    // ユーザーイベント関数を実行
    fc_sppc_usr_evt_cb(e_event, t_gatt_if, pu_param);
}

/*******************************************************************************
 *
 * NAME: ps_sppc_add_status
 *
 * DESCRIPTION:SPPクライアントのステータス検索追加処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェースタイプ
 * uint16_t         u16_con_id      R   コネクションID
 *
 * RETURNS:
 * ts_sppc_status*:SPPステータスのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_sppc_status_t* ps_sppc_add_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id) {
    // SPPコネクションステータスの探索
    ts_sppc_status_t* ps_spp_sts = ps_sppc_status;
    ts_sppc_status_t* ps_bef_sts = NULL;
    while (ps_spp_sts != NULL) {
        if (ps_spp_sts->t_gatt_if == t_gatt_if && ps_spp_sts->u16_con_id == u16_con_id) {
            return ps_spp_sts;
        }
        ps_bef_sts = ps_spp_sts;
        ps_spp_sts = ps_spp_sts->ps_next;
    }
    // ステータスが生成されていないので、新規に生成
    ps_spp_sts = pv_mem_malloc(sizeof(ts_sppc_status_t));
    if (ps_spp_sts == NULL) {
        // 結果返信
        return NULL;
    }
    ps_spp_sts->t_gatt_if          = t_gatt_if;
    ps_spp_sts->u16_con_id         = u16_con_id;
    ps_spp_sts->ps_con_sts         = NULL;
    ps_spp_sts->ps_svc_sts         = NULL;
    ps_spp_sts->u16_hndl_tx_data   = 0;
    ps_spp_sts->u16_hndl_tx_cmd    = 0;
    ps_spp_sts->u16_hndl_rx_data   = 0;
    ps_spp_sts->u16_hndl_notify[0] = 0;
    ps_spp_sts->u16_hndl_notify[1] = 0;
    ps_spp_sts->ps_next            = NULL;
    if (ps_bef_sts == NULL) {
        ps_sppc_status = ps_spp_sts;
    } else {
        ps_bef_sts->ps_next = ps_spp_sts;
    }
    // 結果返信
    return ps_spp_sts;
}

/*******************************************************************************
 *
 * NAME: ps_sppc_get_status
 *
 * DESCRIPTION:SPPクライアントステータスの検索処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェースタイプ
 * uint16_t         u16_con_id      R   コネクションID
 * esp_gatt_id_t    s_svc_id        R   サービスID
 *
 * RETURNS:
 * ts_sppc_status*:SPPステータスのポインタ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_sppc_status_t* ps_sppc_get_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id) {
    // SPPコネクションステータスの探索
    ts_sppc_status_t* ps_chk_sts;
    for (ps_chk_sts = ps_sppc_status; ps_chk_sts != NULL; ps_chk_sts = ps_chk_sts->ps_next) {
        if (ps_chk_sts->t_gatt_if == t_gatt_if && ps_chk_sts->u16_con_id == u16_con_id) {
            return ps_chk_sts;
        }
    }
    // 結果無し
    return NULL;
}

/*******************************************************************************
 *
 * NAME: v_sppc_del_status
 *
 * DESCRIPTION:SPPクライアントのコネクションステータスの削除処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェースタイプ
 * uint16_t         u16_con_id      R   コネクションID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_sppc_del_status(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id) {
    // SPPコネクションステータスの探索
    ts_sppc_status_t* ps_chk_sts = ps_sppc_status;
    ts_sppc_status_t* ps_bef_sts = NULL;
    ts_sppc_status_t* ps_del_sts = NULL;
    while (ps_chk_sts != NULL) {
        // 対象データ判定
        if (ps_chk_sts->t_gatt_if != t_gatt_if || ps_chk_sts->u16_con_id != u16_con_id) {
            // 次のステータス
            ps_bef_sts = ps_chk_sts;
            ps_chk_sts = ps_chk_sts->ps_next;
            continue;
        }
        // リンクからの分離
        if (ps_bef_sts != NULL) {
            ps_bef_sts->ps_next = ps_chk_sts->ps_next;
        } else {
            ps_sppc_status = ps_chk_sts->ps_next;
        }
        // ステータス更新
        ps_del_sts = ps_chk_sts;
        ps_chk_sts = ps_chk_sts->ps_next;
        // ステータス削除
        l_mem_free(ps_del_sts);
    }
}

/*******************************************************************************
 *
 * NAME: e_sppc_con_sts
 *
 * DESCRIPTION:SPP接続ステータスの取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_gatt_if_t    t_gatt_if       R   GATTインターフェースタイプ
 * uint16_t         u16_con_id      R   コネクションID
 *
 * RETURNS:
 * te_com_ble_spp_connection_sts_t:接続ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_spp_connection_sts_t e_sppc_con_sts(esp_gatt_if_t t_gatt_if, uint16_t u16_con_id) {
    //==========================================================================
    // GATT接続ステータス判定
    //==========================================================================
    // インターフェースステータス
    ts_gattc_if_status_t* ps_if_sts = ps_gattc_get_if_status(t_gatt_if);
    if (ps_if_sts == NULL) {
        return COM_BLE_SPP_CON_DISCONNECTED;
    }
    // コネクションステータス取得
    ts_gattc_con_status_t* ps_con_sts = ps_gattc_get_con_status_id(ps_if_sts, u16_con_id);
    if (ps_con_sts == NULL) {
        return COM_BLE_SPP_CON_DISCONNECTED;
    }

    //==========================================================================
    // SPPステータスの判定
    //==========================================================================
    ts_sppc_status_t* ps_sppc_sts = ps_sppc_get_status(t_gatt_if, u16_con_id);
    if (ps_sppc_sts == NULL) {
        return COM_BLE_SPP_CON_CONNECTING;
    }
    if (ps_sppc_sts->u16_hndl_notify[0] == 0 || ps_sppc_sts->u16_hndl_notify[1] == 0) {
        return COM_BLE_SPP_CON_CONNECTING;
    }
    // SPP接続済み
    return COM_BLE_SPP_CON_CONNECTED;
}

/*******************************************************************************
 *
 * NAME: b_sppc_chk_status
 *
 * DESCRIPTION:SPPクライアントのサービス判定処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_gattc_svc_status_t*   ps_svc_sts      R   サービスステータス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_sppc_chk_status(ts_gattc_svc_status_t* ps_svc_sts) {
    // サービスステータスのNULLチェック
    if (ps_svc_sts == NULL) {
        return false;
    }
    // UUIDチェック
    esp_bt_uuid_t* pt_chk_uuid = &ps_svc_sts->ps_db_elems[SPPC_ATTR_IDX_SVC].uuid;
    if (pt_chk_uuid->len != 2) {
        return false;
    }
    if (pt_chk_uuid->uuid.uuid16 != BLE_SPPS_UUID_SERVICE) {
        return false;
    }
    // エレメント数のチェック
    return (ps_svc_sts->u16_db_elem_cnt == SPPC_ATTR_IDX_NB);
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
