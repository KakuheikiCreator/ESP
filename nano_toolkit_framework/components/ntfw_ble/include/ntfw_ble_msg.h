/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :BLE messaging framework function header file
 *
 * CREATED:2021/06/13 13:03:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:BLEのシリアル通信を利用したセキュアメッセージング機能
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
#ifndef  __NTFW_BLE_MESSGAE_H__
#define  __NTFW_BLE_MESSGAE_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <esp_system.h>
#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <ntfw_com_data_model.h>
#include <ntfw_ble_fmwk.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** BLE Message Debug */
//#define COM_BLE_MSG_DEBUG

/** MTUサイズ（デフォルト） */
#ifndef COM_MSG_SIZE_MTU
    /** メッセージのMTUサイズ */
    #define COM_MSG_SIZE_MTU    (256)
#endif

/** 受信デーモンタスクのスタックの深さ ※最小3072程度 */
#ifndef COM_MSG_RX_DEAMON_STACK_DEPTH
    #define COM_MSG_RX_DEAMON_STACK_DEPTH   (6144)
#endif

/** イベント通知デーモンタスクのスタックの深さ ※最小2048程度 */
#ifndef COM_MSG_EVT_DEAMON_STACK_DEPTH
    #define COM_MSG_EVT_DEAMON_STACK_DEPTH  (4096)
#endif

/** 受信デーモンタスクの優先度 */
#ifndef COM_MSG_RX_DEAMON_PRIORITIES
    #define COM_MSG_RX_DEAMON_PRIORITIES    (configMAX_PRIORITIES - 3)
#endif

/** イベント通知デーモンタスクの優先度 */
#ifndef COM_MSG_EVT_DEAMON_PRIORITIES
    #define COM_MSG_EVT_DEAMON_PRIORITIES   (configMAX_PRIORITIES - 4)
#endif

/** デーモンタスクの遅延間隔 ※Watchdog timer measures */
#ifndef COM_MSG_DEAMON_DELAY_INTERVAL_MSEC
    #define COM_MSG_DEAMON_DELAY_INTERVAL_MSEC  (500)
#endif

/** リトライ時間 */
#ifndef COM_MSG_RETRY_WAIT
    #define COM_MSG_RETRY_WAIT  (100 / portTICK_PERIOD_MS)
#endif

/** メッセージ受信チェック */
#ifndef COM_MSG_RX_CHK_TIMEOUT
    #define COM_MSG_RX_CHK_TIMEOUT  (1000 / portTICK_PERIOD_MS)
#endif

/** 受信メッセージのキューイングタイムアウト */
#ifndef COM_MSG_QUEUE_TIMEOUT
    #define COM_MSG_QUEUE_TIMEOUT   (3000 / portTICK_PERIOD_MS)
#endif

/** イベント待ちタイムアウト */
#ifndef COM_MSG_EVT_CHK_TIMEOUT
    #define COM_MSG_EVT_CHK_TIMEOUT   (portMAX_DELAY)
#endif

/** トランザクションタイムアウト（オープン） */
#ifndef COM_MSG_TRN_TIMEOUT_MS_OPEN
    #define COM_MSG_TRN_TIMEOUT_MS_OPEN (90000)
#endif

/** トランザクションタイムアウト（ペアリング） */
#ifndef COM_MSG_TRN_TIMEOUT_MS_PAIRING
    #define COM_MSG_TRN_TIMEOUT_MS_PAIRING  (90000)
#endif

/** トランザクションタイムアウト（ステータスチェック） */
#ifndef COM_MSG_TRN_TIMEOUT_MS_STS_CHK
    #define COM_MSG_TRN_TIMEOUT_MS_STS_CHK  (5000)
#endif

/** イベントエンキュー最大リトライ回数 */
#ifndef COM_MSG_EVT_MAX_RETRY_CNT
    #define COM_MSG_EVT_MAX_RETRY_CNT   (3)
#endif

/** メッセージの認証タグサイズ */
#define COM_MSG_SIZE_AUTH_TAG   (32)

/** 共通鍵サイズ */
#define COM_MSG_SIZE_CIPHER_KEY (32)

/** ステータスサイズ */
#define COM_MSG_SIZE_TICKET_STS (32)

/** ハッシュストレッチング */
#ifndef COM_MSG_AUTH_STRETCHING
    #define COM_MSG_AUTH_STRETCHING (8)
#endif

/** 認証タグのチェック値 */
#ifndef COM_MSG_AUTH_CHECK_VALUE
    #define COM_MSG_AUTH_CHECK_VALUE    (0xA5)
#endif

/** 受信キューサイズ */
#ifndef COM_MSG_RX_QUEUE_SIZE
    #define COM_MSG_RX_QUEUE_SIZE   (32)
#endif

/** イベントキューサイズ */
#ifndef COM_MSG_EVT_QUEUE_SIZE
    #define COM_MSG_EVT_QUEUE_SIZE  (32)
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/**
 * メッセージタイプ
 */
typedef enum {
    COM_BLE_MSG_TYP_RESPONSE = 0x00,    // 受信通知
    COM_BLE_MSG_TYP_RESET,              // リセット
    COM_BLE_MSG_TYP_PING,               // PING
    COM_BLE_MSG_TYP_PAIRING_REQ,        // ペアリング要求
    COM_BLE_MSG_TYP_PAIRING_RSP,        // ペアリング応答
    COM_BLE_MSG_TYP_DIGEST_MATCH,       // ダイジェスト一致
    COM_BLE_MSG_TYP_DIGEST_ERR,         // ダイジェスト不一致
    COM_BLE_MSG_TYP_STATUS_REQ,         // ステータス要求
    COM_BLE_MSG_TYP_STATUS_RSP1,        // ステータス応答１
    COM_BLE_MSG_TYP_STATUS_RSP2,        // ステータス応答２
    COM_BLE_MSG_TYP_DATA,               // データ
    COM_BLE_MSG_TYP_CIPHERTEXT,         // 暗号文
    COM_BLE_MSG_TYP_CNT                 // メッセージタイプ数
} te_com_ble_msg_type_t;

/**
 * 接続ステータス
 */
typedef enum {
    COM_BLE_MSG_CON_DISCONNECTED = 0x00,    // 未接続
    COM_BLE_MSG_CON_CONNECTING,             // 接続中
    COM_BLE_MSG_CON_WAIT_PASSKEY,           // パスキー返信待ち
    COM_BLE_MSG_CON_WAIT_NUM_CHK,           // 番号確認待ち
    COM_BLE_MSG_CON_CONNECTED,              // 接続済み
    COM_BLE_MSG_CON_ERROR,                  // ステータスチェックエラー
} te_com_ble_msg_connection_sts_t;

/**
 * トランザクションステータス
 */
typedef enum {
    COM_BLE_MSG_TRN_NONE = 0x00,        // トランザクションなし
    COM_BLE_MSG_TRN_OPEN,               // オープントランザクション実行中
    COM_BLE_MSG_TRN_PAIRING,            // ペアリングトランザクション実行中
    COM_BLE_MSG_TRN_STS_CHK,            // ステータスチェックトランザクション実行中
} te_com_ble_msg_transaction_sts_t;

/**
 * チケットアクセスイベント
 */
typedef enum {
    COM_BLE_MSG_TICKET_EVT_CREATE = 0x00,   // チケット作成
    COM_BLE_MSG_TICKET_EVT_READ,            // チケット読み込み
    COM_BLE_MSG_TICKET_EVT_UPDATE,          // チケット更新
    COM_BLE_MSG_TICKET_EVT_DELETE,          // チケット削除
} te_com_ble_msg_ticket_evt_t;

/**
 * コールバックイベント
 */
typedef enum {
    COM_BLE_MSG_EVT_RX_RESPONSE = 0,        // 応答を受信
    COM_BLE_MSG_EVT_RX_RESET,               // リセットメッセージ受信
    COM_BLE_MSG_EVT_RX_PING,                // PINGメッセージ受信
    COM_BLE_MSG_EVT_RX_DATA,                // データメッセージ受信
    COM_BLE_MSG_EVT_RX_CIPHERTEXT,          // 暗号メッセージ受信
    COM_BLE_MSG_EVT_GATT_CONNECT,           // GATT接続
    COM_BLE_MSG_EVT_GATT_DISCONNECT,        // GATT切断
    COM_BLE_MSG_EVT_LINK_SUCCESS,           // リンク成功
    COM_BLE_MSG_EVT_OPEN_SUCCESS,           // オープン成功
    COM_BLE_MSG_EVT_OPEN_TIMEOUT,           // オープンタイムアウト
    COM_BLE_MSG_EVT_PAIRING_START,          // ペアリング開始
    COM_BLE_MSG_EVT_PAIRING_SUCCESS,        // ペアリング成功
    COM_BLE_MSG_EVT_PAIRING_ERR,            // ペアリングエラー
    COM_BLE_MSG_EVT_PAIRING_TIMEOUT,        // ペアリングタイムアウト
    COM_BLE_MSG_EVT_STATUS_CHK,             // ステータスチェック開始
    COM_BLE_MSG_EVT_STATUS_OK,              // ステータス正常
    COM_BLE_MSG_EVT_STATUS_ERR,             // ステータス異常
    COM_BLE_MSG_EVT_STATUS_TIMEOUT,         // ステータスチェックタイムアウト
    COM_BLE_MSG_EVT_HANDLING_ERR,           // メッセージハンドリングエラー
    COM_BLE_MSG_EVT_COUNT                   // コールバックイベント数
} te_com_ble_msg_event;

/**
 * メッセージ
 */
typedef struct {
    esp_bd_addr_t t_rcv_bda;                            // BLEアドレス
    uint32_t u32_rcv_tick_ms;                           // 受信ティック（ミリ秒）
    uint64_t u64_device_id;                             // デバイスID
    te_com_ble_msg_type_t e_type;                       // メッセージタイプ
    uint16_t u16_length;                                // メッセージ長
    uint32_t u32_seq_no;                                // シーケンス番号
    uint8_t u8_auth_tag[COM_MSG_SIZE_AUTH_TAG];         // 認証タグ
    ts_u8_array_t* ps_data;                             // 本文のデータ
} ts_com_msg_t;

/**
 * 認証チケット
 */
typedef struct {
    uint64_t u64_own_device_id;                         // 自デバイスID
    uint64_t u64_rmt_device_id;                         // 相手デバイスID
    uint8_t u8_enc_key[COM_MSG_SIZE_CIPHER_KEY];        // 暗号鍵
    uint8_t u8_own_sts[COM_MSG_SIZE_TICKET_STS];        // 自ステータス
    uint8_t u8_rmt_sts_hash[COM_MSG_SIZE_TICKET_STS];   // 相手ステータスハッシュ
    uint32_t u32_max_seq_no;                            // 最大シーケンス番号
    uint32_t u32_tx_seq_no;                             // 送信シーケンス番号
    uint32_t u32_rx_seq_no;                             // 受信シーケンス番号
} ts_com_msg_auth_ticket_t;

/**
 * チケットアクセスコールバック関数
 */
typedef esp_err_t (*tf_com_ble_msg_ticket_cb_t)(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket);

/**
 * メッセージイベントコールバック関数
 */
typedef void (*tf_com_ble_msg_evt_cb_t)(te_com_ble_msg_event e_msg_evt);


/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/
/** メッセージサーバーの初期処理 */
extern esp_err_t sts_com_msg_init_svr(uint16_t u16_app_id,
                                       uint64_t u64_device_id,
                                       uint16_t u16_max_length,
                                       tf_com_ble_msg_evt_cb_t pf_evt_cb,
                                       tf_com_ble_msg_ticket_cb_t pf_tkt_cb);
/** メッセージクライアントの初期処理 */
extern esp_err_t sts_com_msg_init_cli(uint16_t u16_app_id,
                                       uint64_t u64_device_id,
                                       uint16_t u16_max_length,
                                       tf_com_ble_msg_evt_cb_t pf_evt_cb,
                                       tf_com_ble_msg_ticket_cb_t pf_tkt_cb);
/** 受信メッセージのエンキュー有効化処理 */
extern void v_com_msg_rx_enabled(te_com_ble_msg_type_t e_type);
/** 受信メッセージのエンキュー無効化処理 */
extern void v_com_msg_rx_disabled(te_com_ble_msg_type_t e_type);
/** ペアリング機能の設定 */
extern void v_com_msg_config_pairing(bool b_enabled);
/** ステータスチェック機能の設定 */
extern void v_com_msg_config_sts_chk(bool b_enabled);
/** ペアリングの有効判定 */
extern bool b_com_msg_is_paired(uint64_t u64_device_id);
/** 接続ステータス取得 */
extern te_com_ble_msg_connection_sts_t e_com_msg_connection_sts();
/** トランザクションステータスの取得処理 */
extern te_com_ble_msg_transaction_sts_t sts_com_msg_transaction_sts();
/** メッセージサーバーへの接続開始 */
extern esp_err_t sts_com_msg_open_server(ts_com_ble_gap_device_info_t* ps_device);
/** コネクションを切断 */
extern esp_err_t sts_com_msg_close();
/** 接続先デバイスIDの編集 */
extern esp_err_t sts_com_msg_edit_remote_dev_id(uint64_t* pu64_device_id);
/** ペアリング中の公開鍵のペアの編集処理 */
extern esp_err_t sts_com_msg_edit_public_key_pair(uint8_t* pu8_client_key, uint8_t* pu8_server_key);
/** メッセージの受信処理 */
extern ts_com_msg_t* ps_com_msg_rx_msg(TickType_t t_tick);
/** メッセージの受信ウェイト */
extern esp_err_t sts_com_msg_rx_wait(TickType_t t_tick);
/** RESETメッセージの送信処理 */
extern esp_err_t sts_com_msg_tx_reset_msg();
/** PINGメッセージの送信処理 */
extern esp_err_t sts_com_msg_tx_ping_msg();
/** ペアリング要求 */
extern esp_err_t sts_com_msg_tx_pairing_request();
/** ペアリング認証（ダイジェスト比較結果の通知） */
extern esp_err_t sts_com_msg_tx_pairing_certification(bool b_result, uint32_t u32_max_seq_no);
/** ステータスチェック要求 */
extern esp_err_t sts_com_msg_tx_sts_chk_request();
/** 平文メッセージの送信処理 */
extern esp_err_t sts_com_msg_tx_plain_msg(uint64_t u64_device_id, ts_u8_array_t* ps_data);
/** 暗号メッセージの送信処理 */
extern esp_err_t sts_com_msg_tx_cipher_msg(uint64_t u64_device_id, ts_u8_array_t* ps_data);
/** メッセージの削除処理 */
extern esp_err_t sts_com_msg_delete_msg(ts_com_msg_t* ps_msg);
/** チケットの削除処理 */
extern esp_err_t sts_com_msg_delete_ticket(uint64_t u64_device_id);
/** チケットステータスのクリア処理 */
extern esp_err_t sts_com_msg_clear_status(uint64_t u64_device_id);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_BLE_MESSGAE_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
