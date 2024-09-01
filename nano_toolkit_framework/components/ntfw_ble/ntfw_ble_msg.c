/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :BLE messaging framework function source file
 *
 * CREATED:2021/06/14 01:46:00
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
/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <ntfw_ble_msg.h>

#include <string.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp_gatt_common_api.h>
#include <ntfw_com_mem_alloc.h>
#include <ntfw_com_value_util.h>
#include <ntfw_com_date_time.h>
#include <ntfw_com_debug_util.h>
#include <ntfw_cryptography.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** ログ接頭辞 */
#define LOG_TAG "COM_BLE_MSG"

/** uin16_t max value */
#define U16_MAX             (0xffff)
/** uint32_t max value */
#define U32_MAX             (0xffffffff)

/** message minimum length */
#define MSG_SIZE_DEFAULT    (1024)
/** message header length */
#define MSG_SIZE_HEADER     (MSG_POS_BODY)
/** message footer length */
#define MSG_SIZE_FOOTER     (15)
/** message random values length */
#define MSG_SIZE_RANDOM     (13)
/** cipher tag length */
#define MSG_SIZE_CIPHER_TAG (TAG_BYTES)
/** cipher IV length */
#define MSG_SIZE_CIPHER_IV  (12)
/** cipher header length */
#define MSG_SIZE_CIPHER_HEADER  (MSG_SIZE_CIPHER_TAG + MSG_SIZE_CIPHER_IV)
/** digest match data length */
#define MSG_SIZE_DIGEST_MATCH_DATA  (36)
/** check code length */
#define MSG_SIZE_CHECK_CODE (32)
/** check random number length */
#define MSG_SIZE_CHECK_RANDOM   (32)

/** Receive message device id position */
#define MSG_POS_DEVICE_ID   (0)
/** Receive message type position */
#define MSG_POS_TYPE        (8)
/** Receive message length position */
#define MSG_POS_MSG_LEN     (9)
/** Receive message seq number position */
#define MSG_POS_SEQ_NO      (11)
/** Authentication tag position */
#define MSG_POS_AUTH_TAG    (15)
/** message body position */
#define MSG_POS_BODY        (47)
/** status response 1 random position */
#define MSG_POS_STS_RSP1_RND    (MSG_POS_BODY + MSG_SIZE_CHECK_CODE)
/** plain data position */
#define MSG_POS_PLANIN_DATA MSG_POS_BODY
/** cipher tag position */
#define MSG_POS_CIPHER_TAG  MSG_POS_BODY
/** cipher iv position */
#define MSG_POS_CIPHER_IV   (63)
/** cipher data position */
#define MSG_POS_CIPHER_DATA (75)
/** GAP status check mask(passkey reply) */
#define MSG_GAP_CHK_PASSKEY (GAP_DEV_STS_REQ_PASSKEY | GAP_DEV_STS_RPY_PASSKEY)
/** GAP status check mask(number check) */
#define MSG_GAP_CHK_NUM_CHK (GAP_DEV_STS_REQ_NUM_CHK | GAP_DEV_STS_RPY_NUM_CHK)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/**
 * 動作モード
 */
typedef enum {
    COM_BLE_MSG_MODE_BLE_INIT = 0x00,   // 初期状態
    COM_BLE_MSG_MODE_BLE_SERVER,        // BLEサーバーモード
    COM_BLE_MSG_MODE_BLE_CLIENT,        // BLEクライアントモード
} te_com_ble_msg_mode_t;

/**
 * メッセージ受信ステータス
 */
typedef enum {
    COM_BLE_MSG_RCV_NORMAL = 0x00,      // 正常受信
    COM_BLE_MSG_RCV_NOT_FOUND,          // 受信データ無し
    COM_BLE_MSG_RCV_NO_MEM_ERR,         // メモリ確保エラー
    COM_BLE_MSG_RCV_LENGTH_ERR,         // メッセージ長エラー
    COM_BLE_MSG_RCV_DEV_ID_ERR,         // メッセージデバイスIDエラー
    COM_BLE_MSG_RCV_TYPE_ERR,           // メッセージタイプエラー
    COM_BLE_MSG_RCV_SEQ_ERR,            // メッセージシーケンス番号エラー
    COM_BLE_MSG_RCV_STOP_TKN_ERR,       // ストップトークンエラー
    COM_BLE_MSG_RCV_AUTH_ERR,           // メッセージ認証エラー
    COM_BLE_MSG_RCV_DECRYPT_ERR,        // 復号エラー
    COM_BLE_MSG_RCV_TRAN_ERR,           // トランザクションエラー
    COM_BLE_MSG_RCV_PAIRING_ERR,        // ペアリングエラー
    COM_BLE_MSG_RCV_STS_CHK_ERR,        // ステータスチェックエラー
    COM_BLE_MSG_RCV_RECEIVER_ERR,       // 受信処理エラー
    COM_BLE_MSG_RCV_HANDLING_ERR,       // 受信ハンドリングエラー
    COM_BLE_MSG_RCV_TIMEOUT_ERR,        // 受信タイムアウト
    COM_BLE_MSG_RCV_ADDRESS_ERR,        // 受信アドレスエラー
} te_com_ble_msg_rcv_sts_t;

/**
 * 機能制御設定
 */
typedef enum {
    MSG_FUNC_CTL_PAIRING = 0x01,            // ペアリング有効
    MSG_FUNC_CTL_STS_CHK = 0x02,            // ステータスチェック有効
} te_msg_function_ctrl_t;

/**
 * トランザクションのチェックタイプ
 */
typedef enum {
    MSG_CHK_TRN_NONE = 0x00,                // チェック不要
    MSG_CHK_TRN_EXEC,                       // トランザクション実行中のみ有効
    MSG_CHK_TRN_STOP,                       // トランザクション停止中のみ有効
} te_msg_chk_trn_type_t;

/**
 * ペアリングステータス
 */
typedef enum {
    MSG_PAIRING_CHK_NONE   = 0x00,          // 認証無し
    MSG_PAIRING_CHK_LOCAL  = 0x01,          // ダイジェスト一致（ローカル）
    MSG_PAIRING_CHK_REMOTE = 0x02,          // ダイジェスト一致（リモート）
    MSG_PAIRING_CHK_ALL    = 0x03,          // ダイジェスト一致
} te_msg_pairing_sts_t;

/**
 * メッセージ定義
 */
typedef struct {
    bool b_fixed_length;                    // メッセージ長固定
    uint16_t u16_length;                    // メッセージ長
    uint16_t u16_body_length;               // 本文長
    bool b_fixed_seq;                       // SEQ番号固定
    uint32_t u32_seq_no;                    // メッセージSEQ番号
    bool b_pairing;                         // ペアリング必須
    bool b_encryption;                      // 本文の暗号化有無
    te_msg_chk_trn_type_t e_chk_trn_type;   // トランザクションチェックタイプ
    uint16_t u16_map_before_rx;             // 直前の受信メッセージタイプマップ ※エラータイプに1を立てる
    uint16_t u16_map_before_tx;             // 直前の送信メッセージタイプマップ ※エラータイプに1を立てる
    bool b_response;                        // レスポンス有無
} ts_msg_definition_t;

/**
 * メッセージ本文：ダイジェスト一致
 */
typedef struct {
    uint8_t u8_sts_hash[COM_MSG_SIZE_TICKET_STS];   // ステータスハッシュ
    uint32_t u32_max_seq_no;                        // 最大シーケンス番号
} ts_msg_digest_match_t;

/**
 * IV編集
 */
typedef struct {
    uint32_t u32_seq_no;                            // シーケンス番号
    uint8_t u8_iv[MSG_SIZE_CIPHER_IV];              // 初期ベクトル
} ts_msg_edit_iv_t;

/**
 * 送受信履歴
 */
typedef struct {
    esp_bd_addr_t t_bda;                            // リモートデバイスBLEアドレス
    uint32_t u32_tick_ms;                           // 受信ティック（ミリ秒）
    uint64_t u64_device_id;                         // リモートデバイスID
    uint32_t u32_seq_no;                            // シーケンス番号
    te_com_ble_msg_type_t e_type;                   // メッセージタイプ
} ts_msg_history_t;

/**
 * トランザクション情報
 */
typedef struct {
    te_com_ble_msg_transaction_sts_t e_sts;         // トランザクションステータス
    uint64_t u64_device_id;                         // リモートデバイスID
    esp_bd_addr_t t_bda;                            // リモートデバイスBLEアドレス
    uint32_t u32_timeout_ms;                        // トランザクションタイムアウト（ミリ秒）
} ts_transaction_info_t;

/**
 * ペアリング情報
 */
typedef struct {
    te_msg_pairing_sts_t e_sts;                         // ペアリングステータス
    uint8_t u8_com_key[COM_MSG_SIZE_CIPHER_KEY];        // 共通鍵
    ts_crypto_x25519_context_t* ps_x25519_ctx;          // X25519コンテキスト
    uint8_t u8_dev_status[COM_MSG_SIZE_TICKET_STS];     // 自デバイスステータス
    uint8_t u8_rmt_sts_hash[COM_MSG_SIZE_TICKET_STS];   // 相手デバイスステータスハッシュ
    uint32_t u32_max_seq_no;                            // 最大シーケンス番号
} ts_pairing_info_t;

/**
 * ステータスチェック情報
 */
typedef struct {
    uint8_t u8_tx_rand[COM_MSG_SIZE_TICKET_STS];        // 送信ステータスチェック乱数
    uint8_t u8_rx_rand[COM_MSG_SIZE_TICKET_STS];        // 受信ステータスチェック乱数
} ts_sts_check_info_t;

/**
 * GATTインターフェース取得関数
 */
typedef esp_gatt_if_t (*tf_get_gatt_if_t)();

/**
 * 接続ステータス関数
 */
typedef te_com_ble_msg_connection_sts_t (*tf_connection_sts_t)();

/**
 * データ受信関数
 */
typedef ts_com_ble_gatt_rx_data_t* (*tf_ble_rx_data_t)(TickType_t t_tick);

/**
 * メッセージ送信処理
 */
typedef esp_err_t (*tf_ble_tx_msg_t)(ts_u8_array_t* ps_msg);

/**
 * 受信キュークリア
 */
typedef void (*tf_ble_rx_clear_t)();

/**
 * 受信キューの読み飛ばし
 */
typedef void (*tf_ble_rx_through_t)(size_t t_len);


/**
 * 通信制御設定
 */
typedef struct {
    te_com_ble_msg_mode_t e_mode;           // 動作モード
    uint16_t u16_app_id;                    // アプリケーションID
    uint64_t u64_device_id;                 // 自デバイスID
    te_msg_function_ctrl_t s_func_ctl;      // 機能制御
    uint32_t u32_max_length;                // 最大メッセージサイズ
    tf_get_gatt_if_t pf_gatt_if;            // GATTインターフェース取得関数
    tf_connection_sts_t pf_connect_sts;     // 接続ステータス取得関数
    tf_ble_rx_data_t pf_rx_data;            // データ受信関数
    tf_ble_tx_msg_t pf_tx_msg;              // メッセージ送信関数
    tf_ble_rx_clear_t pf_rx_clear;          // 受信キュークリア関数
    tf_ble_rx_through_t pf_rx_through;      // 受信キュー読み飛ばし関数
    tf_com_ble_msg_ticket_cb_t pf_tkt_cb;   // チケットアクセスイベントコールバック関数
    tf_com_ble_msg_evt_cb_t pf_evt_cb;      // 受信イベントコールバック関数
} ts_msg_ctrl_cfg_t;

/**
 * 通信制御ステータス
 */
typedef struct {
    esp_gatt_if_t t_gatt_if;                // GATTインターフェース
    uint64_t u64_rmt_device_id;             // リモートデバイスID
    esp_bd_addr_t t_rmt_bda;                // リモートデバイスBLEアドレス
    ts_com_msg_auth_ticket_t s_rmt_ticket;  // リモートデバイスチケット
    uint64_t u64_tx_count;                  // 送信カウンタ
    uint64_t u64_rx_count;                  // 受信カウンタ
    ts_msg_history_t s_bef_tx_msg;          // 直前の送信メッセージ履歴
    ts_msg_history_t s_bef_rx_msg;          // 直前の受信メッセージ履歴
    ts_msg_history_t s_bef_rx_rsp;          // 直前の返信メッセージ履歴
    ts_transaction_info_t s_tran;           // トランザクション情報
    ts_pairing_info_t s_pairing;            // ペアリング情報
    ts_sts_check_info_t s_sts_chk;          // ステータスチェック情報
    ts_com_ble_gattc_con_info_t* ps_con;    // BLEコネクション
} ts_msg_ctrl_sts_t;

/**
 * デーモンタスク制御ステータス
 */
typedef struct {
    TaskHandle_t s_rx_deamon_handle;        // RXデーモンタスクハンドル
    QueueHandle_t s_rx_queue_handle;        // 受信キューハンドラ
    uint32_t u32_rx_enqueue_filter;         // 受信キューイングフィルタ
    TaskHandle_t s_evt_deamon_handle;       // イベント通知デーモンタスクハンドル
    QueueHandle_t s_evt_queue_handle;       // イベント通知キューハンドラ
} ts_msg_deamon_sts_t;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
//==============================================================================
// Local Message Functions
//==============================================================================
/** begin message daemon Task */
static esp_err_t sts_msg_begin_daemon_task();
/** message rx daemon task */
static void v_msg_rx_daemon_task(void* pv_parameters);
/** message event daemon task */
static void v_msg_evt_daemon_task(void* pv_parameters);
/** message event enqueue processing */
static void v_msg_evt_enqueue(te_com_ble_msg_event e_msg_evt);
/** message controller init */
static void v_msg_ctrl_sts_init();
/** message controller connection Reset */
static void v_msg_ctrl_sts_connection_reset();
/** message controller transaction reset */
static void v_msg_ctrl_sts_transaction_reset();
/** message controller transaction timeout */
static void v_msg_ctrl_sts_transaction_timeout();
/** message history reset processing */
static void v_msg_history_reset(ts_msg_history_t* ps_msg_history);
/** BLE Client Get Connection */
static ts_com_ble_gattc_con_info_t* ps_get_connection();
/** BLE Rx Message */
static te_com_ble_msg_rcv_sts_t e_rx_message(ts_com_msg_t* ps_rx_msg, TickType_t t_tick);
/** BLE Rx Message check */
static te_com_ble_msg_rcv_sts_t e_rx_msg_check(ts_com_msg_t* ps_rx_msg);
/** BLE Rx Message event */
static te_com_ble_msg_rcv_sts_t e_rx_msg_event(ts_com_msg_t* ps_rx_msg);
/** BLE Tx response */
static esp_err_t sts_tx_response(te_com_ble_msg_type_t e_rx_type,
                                  te_com_ble_msg_rcv_sts_t e_rx_sts,
                                  uint32_t u32_seq_no);
/** BLE Tx reset message */
static esp_err_t sts_tx_reset_msg();
/** BLE Tx ping message */
static esp_err_t sts_tx_ping_msg();
/** update tx history */
static void v_upd_tx_history(ts_u8_array_t* ps_msg);
/** Public key received check */
static bool b_is_public_key_received();
/** pairing check */
static bool b_is_paired(uint64_t u64_device_id);
/** begin open transactions  */
static esp_err_t sts_begin_open(esp_bd_addr_t t_bda, uint32_t* pu32_timeout_ms);
/** begin pairing transactions  */
static esp_err_t sts_begin_pairing();
/** begin status check transaction */
static esp_err_t sts_begin_sts_chk();
/** status code check */
static esp_err_t sts_status_check(uint64_t u64_device_id, uint8_t* pu8_chk_code, ts_com_msg_auth_ticket_t* ps_ticket);
/** encryption message */
static esp_err_t sts_msg_encryption(ts_u8_array_t* ps_msg, uint16_t u16_data_len, uint8_t* pu8_key);
/** decryption message */
static ts_u8_array_t* ps_msg_decryption(ts_com_msg_t* ps_rx_msg, uint8_t* pu8_key);
/** BLE edit Rx message header */
static te_com_ble_msg_rcv_sts_t e_edit_rx_header(ts_com_msg_t* ps_rx_msg, ts_com_ble_gatt_rx_data_t* ps_rx_data);
/** edit auth tag */
static esp_err_t sts_edit_auth_tag(uint8_t* pu8_tag, ts_u8_array_t* ps_msg);
/** edit check code */
static esp_err_t sts_edit_check_code(ts_com_msg_auth_ticket_t* ps_ticket, uint8_t* pu8_rand, uint8_t* pu8_digest);
/** create message data */
static ts_u8_array_t* ps_create_msg_data(te_com_ble_msg_type_t e_type, ts_u8_array_t* ps_data);
/** initialize ticket */
static void v_init_ticket(ts_com_msg_auth_ticket_t* ps_ticket);
/** create ticket */
static esp_err_t sts_create_ticket(ts_transaction_info_t* ps_tran, ts_pairing_info_t* ps_pairing);
/** read ticket */
static ts_com_msg_auth_ticket_t* ps_read_ticket(uint64_t u64_device_id, ts_com_msg_auth_ticket_t* ps_cache_ticket);

//==============================================================================
// BLE functions(Client side and Server side)
//==============================================================================
/** BLE GATT Interface (Server Side) */
static esp_gatt_if_t t_gatt_if_svr();
/** BLE GATT Interface (Client Side) */
static esp_gatt_if_t t_gatt_if_cli();
/** BLE GATT Interface (Default) */
static esp_gatt_if_t t_gatt_if_default();
/** BLE SPP event callback (Server Side) */
static void v_spp_evt_cb_svr(esp_gatts_cb_event_t e_event,
                              esp_gatt_if_t t_gatt_if,
                              esp_ble_gatts_cb_param_t* pu_param);
/** BLE SPP event callback (Client Side) */
static void v_spp_evt_cb_cli(esp_gattc_cb_event_t e_event,
                              esp_gatt_if_t t_gatt_if,
                              esp_ble_gattc_cb_param_t* pu_param);
/** BLE Connection status (Server Side) */
static te_com_ble_msg_connection_sts_t e_connect_sts_svr();
/** BLE Connection status (Client Side) */
static te_com_ble_msg_connection_sts_t e_connect_sts_cli();
/** BLE Rx Data(Server Side) */
static ts_com_ble_gatt_rx_data_t* ps_ble_rx_data_svr();
/** BLE Rx Data(Client Side) */
static ts_com_ble_gatt_rx_data_t* ps_ble_rx_data_cli();
/** BLE Server Rx Data queue clea(Server Side)r */
static void v_ble_rx_clear_svr();
/** BLE Client Rx Data queue clear(Client Side) */
static void v_ble_rx_clear_cli();
/** BLE Server Rx through(Server Side) */
static void v_ble_rx_through_svr(size_t t_len);
/** BLE Client Rx through(Client Side) */
static void v_ble_rx_through_cli(size_t t_len);
/** BLE Tx Message(Server Side) */
static esp_err_t sts_ble_tx_msg_svr(ts_u8_array_t* ps_msg);
/** BLE Tx Message(Client Side) */
static esp_err_t sts_ble_tx_msg_cli(ts_u8_array_t* ps_msg);

//==============================================================================
// Dummy functions
//==============================================================================
/**
 * ダミー関数：接続ステータス取得
 */
static te_com_ble_msg_connection_sts_t e_msg_dmy_connect_sts();
/**
 * ダミー関数：データ受信関数
 */
static ts_com_ble_gatt_rx_data_t* ps_msg_dmy_rx_data(TickType_t t_tick);
/**
 * ダミー関数：メッセージ送信関数
 */
static esp_err_t sts_msg_dmy_tx_msg(ts_u8_array_t* ps_msg);
/**
 * ダミー関数：受信データキュークリア
 */
static void v_msg_dmy_rx_clear();
/**
 * 受信キューの読み飛ばし
 */
static void v_msg_dmy_rx_through(size_t t_len);
/**
 * ダミー関数：チケットアクセスコールバック関数
 */
static esp_err_t sts_msg_dmy_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket);
/**
 * ダミー関数：メッセージイベントコールバック関数
 */
static void v_msg_dmy_evt_cb(te_com_ble_msg_event e_msg_evt);


/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
//==============================================================================
// constant definition
//==============================================================================
/** メッセージ定義 */
static const ts_msg_definition_t MSG_DEF[] = {
    // 受信通知：COM_BLE_MSG_TYP_RESPONSE
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 64,                    // レコード長
        .u16_body_length   = 2,                     // 本文長
        .b_fixed_seq       = false,                 // SEQ番号は可変
        .u32_seq_no        = U32_MAX,               // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_NONE,      // トランザクションの状態は何でもよい
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = false,                 // レスポンス有無
    },
    // リセット：COM_BLE_MSG_TYP_RESET
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 62,                    // レコード長
        .u16_body_length   = 0,                     // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 0,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_NONE,      // トランザクションの状態は何でもよい
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // PING：COM_BLE_MSG_TYP_PING
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 62,                    // レコード長
        .u16_body_length   = 0,                     // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 1,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_NONE,      // トランザクションの状態は何でもよい
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // ペアリング要求：COM_BLE_MSG_TYP_PAIRING_REQ
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 94,                    // レコード長
        .u16_body_length   = 32,                    // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 2,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_STOP,      // トランザクション停止中のみ有効
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // ペアリング応答：COM_BLE_MSG_TYP_PAIRING_RSP
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 94,                    // レコード長
        .u16_body_length   = 32,                    // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 3,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_EXEC,      // トランザクション実行中のみ有効
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        // 直前にペアリング要求を送信していないとエラー
        .u16_map_before_tx = (0xFFFF ^ (0x0001 << COM_BLE_MSG_TYP_PAIRING_REQ)),
        .b_response        = true,                  // レスポンス有無
    },
    // ダイジェスト一致：COM_BLE_MSG_TYP_DIGEST_MATCH
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 126,                   // レコード長
        .u16_body_length   = 64,                    // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 4,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = true,                  // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_EXEC,      // トランザクション実行中のみ有効
        // 直前にペアリング要求かペアリング応答を受信していないとエラー
        .u16_map_before_rx = (0xFFFF ^ ((0x0001 << COM_BLE_MSG_TYP_PAIRING_REQ) | (0x0001 << COM_BLE_MSG_TYP_PAIRING_RSP))),
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // ダイジェスト不一致：COM_BLE_MSG_TYP_DIGEST_ERR
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 62,                    // レコード長
        .u16_body_length   = 0,                     // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 5,                     // SEQ番号
        .b_pairing         = false,                 // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_EXEC,      // トランザクション実行中のみ有効
        // 直前にペアリング要求かペアリング応答を受信していないとエラー
        .u16_map_before_rx = (0xFFFF ^ ((0x0001 << COM_BLE_MSG_TYP_PAIRING_REQ) | (0x0001 << COM_BLE_MSG_TYP_PAIRING_RSP))),
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // ステータス要求：COM_BLE_MSG_TYP_STATUS_REQ
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 94,                    // レコード長
        .u16_body_length   = 32,                    // 本文長
        .b_fixed_seq       = false,                 // SEQ番号は可変
        .u32_seq_no        = U32_MAX,               // SEQ番号
        .b_pairing         = true,                  // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_STOP,      // トランザクション停止中のみ有効
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // ステータス応答１：COM_BLE_MSG_TYP_STATUS_RES1
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 126,                   // レコード長
        .u16_body_length   = 64,                    // 本文長
        .b_fixed_seq       = false,                 // SEQ番号は可変
        .u32_seq_no        = U32_MAX,               // SEQ番号
        .b_pairing         = true,                  // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_EXEC,      // トランザクション実行中のみ有効
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        // 直前にステータス要求を送信していないとエラー
        .u16_map_before_tx = (0xFFFF ^ (0x0001 << COM_BLE_MSG_TYP_STATUS_REQ)),
        .b_response        = true,                  // レスポンス有無
    },
    // ステータス応答２：COM_BLE_MSG_TYP_STATUS_RES2
    {
        .b_fixed_length    = true,                  // 固定長メッセージ
        .u16_length        = 94,                    // レコード長
        .u16_body_length   = 32,                    // 本文長
        .b_fixed_seq       = false,                 // SEQ番号は可変
        .u32_seq_no        = U32_MAX,               // SEQ番号
        .b_pairing         = true,                  // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_EXEC,      // トランザクション実行中のみ有効
        // 直前にステータス要求を受信していないとエラー
        .u16_map_before_rx = (0xFFFF ^ (0x0001 << COM_BLE_MSG_TYP_STATUS_REQ)),
        // 直前にステータス応答１を送信していないとエラー
        .u16_map_before_tx = (0xFFFF ^ (0x0001 << COM_BLE_MSG_TYP_STATUS_RSP1)),
        .b_response        = true,                  // レスポンス有無
    },
    // データ：COM_BLE_MSG_TYP_DATA
    {
        .b_fixed_length    = false,                 // 固定長メッセージ
        .u16_length        = 62,                    // レコード長（ヘッダー長＋フッター長）
        .u16_body_length   = 0,                     // 本文長
        .b_fixed_seq       = true,                  // SEQ番号は固定値
        .u32_seq_no        = 6,                     // SEQ番号
        .b_pairing         = true,                  // ペアリング必須
        .b_encryption      = false,                 // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_NONE,      // トランザクションの状態は何でもよい
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
    // 暗号データ：COM_BLE_MSG_TYP_CRYPTOGRAPHY
    {
        .b_fixed_length    = false,                 // 固定長メッセージ
        .u16_length        = 106,                   // レコード長（ヘッダー長＋最小本文データ長＋フッター長）
        .u16_body_length   = AES_BLOCK_BYTES,       // 本文長　※パディング必須
        .b_fixed_seq       = false,                 // SEQ番号は可変
        .u32_seq_no        = U32_MAX,               // SEQ番号
        .b_pairing         = true,                  // ペアリング必須
        .b_encryption      = true,                  // 本文の暗号化有無
        .e_chk_trn_type    = MSG_CHK_TRN_NONE,      // トランザクションの状態は何でもよい
        .u16_map_before_rx = 0x0000,                // 直前の受信メッセージは何でも良い
        .u16_map_before_tx = 0x0000,                // 直前の送信メッセージは何でも良い
        .b_response        = true,                  // レスポンス有無
    },
};

//==============================================================================
// variable definition
//==============================================================================
/** ミューテックス：ステータス値用 */
static SemaphoreHandle_t s_mutex_sts = NULL;

/** 制御設定 */
static ts_msg_ctrl_cfg_t s_msg_ctrl_cfg = {
    .e_mode         = COM_BLE_MSG_MODE_BLE_INIT,    // 動作モード
    .u16_app_id     = 0,                            // アプリケーションID
    .u64_device_id  = 0,                            // 自デバイスID
    .s_func_ctl     = 0x00,                         // 機能制御
    .u32_max_length = MSG_SIZE_DEFAULT,             // 最大メッセージサイズ
    .pf_gatt_if     = t_gatt_if_default,            // GATTインターフェースの取得関数
    .pf_connect_sts = e_msg_dmy_connect_sts,        // 接続ステータス取得関数
    .pf_rx_data     = ps_msg_dmy_rx_data,           // データ受信関数
    .pf_tx_msg      = sts_msg_dmy_tx_msg,           // メッセージ送信関数
    .pf_rx_clear    = v_msg_dmy_rx_clear,           // 受信キュークリア関数
    .pf_rx_through  = v_msg_dmy_rx_through,         // 受信キューの読み飛ばし関数
    .pf_tkt_cb      = sts_msg_dmy_ticket_cb,        // チケットアクセスイベントコールバック関数
    .pf_evt_cb      = v_msg_dmy_evt_cb,             // 受信イベントコールバック関数
};

/** 制御ステータス */
static ts_msg_ctrl_sts_t s_msg_ctrl_sts = {
    .t_gatt_if      = 0,                    // GATTインターフェース
    .u64_rmt_device_id = 0,                 // リモートデバイスID
    .t_rmt_bda      = {0x40},               // リモートデバイスID
    .s_rmt_ticket   = {                     // リモートデバイスチケット
        .u64_own_device_id = 0,             // 自デバイスID
        .u64_rmt_device_id = 0,             // 相手デバイスID
        .u8_enc_key      = {0x00},          // 暗号鍵
        .u8_own_sts      = {0x00},          // 自ステータス
        .u8_rmt_sts_hash = {0x00},          // 相手ステータスハッシュ
        .u32_max_seq_no  = 0,               // 最大シーケンス番号
        .u32_tx_seq_no   = 0,               // 送信シーケンス番号
        .u32_rx_seq_no   = 0,               // 受信シーケンス番号
    },
    .u64_tx_count   = 0,                    // 送信カウンタ
    .u64_rx_count   = 0,                    // 受信カウンタ
    .s_bef_tx_msg = {                       // 直前の送信メッセージ履歴
        .u64_device_id = 0,
        .t_bda         = {0x40},
        .u32_tick_ms   = 0,
        .u32_seq_no    = 0,
        .e_type = COM_BLE_MSG_TYP_CNT
    },
    .s_bef_rx_msg = {                       // 直前の受信メッセージ履歴
        .u64_device_id = 0,
        .t_bda         = {0x40},
        .u32_tick_ms   = 0,
        .u32_seq_no    = 0,
        .e_type = COM_BLE_MSG_TYP_CNT
    },
    .s_tran = {
        .e_sts          = COM_BLE_MSG_TRN_NONE,     // トランザクションステータス
        .u64_device_id  = 0,                // トランザクション実行中の相手デバイスID
        .t_bda          = {0x40},           // トランザクション実行中の相手デバイスBLEアドレス
        .u32_timeout_ms = U32_MAX,          // トランザクションタイムアウト（ミリ秒）
    },
    .s_pairing = {                          // ペアリング情報
        .e_sts = MSG_PAIRING_CHK_NONE,      // ペアリングステータス
        .u8_com_key        = {0},           // 共通鍵
        .ps_x25519_ctx     = NULL,          // X25519コンテキスト
        .u8_dev_status     = {0},           // 自デバイスステータス
        .u8_rmt_sts_hash   = {0},           // 相手デバイスステータスハッシュ
        .u32_max_seq_no    = 0              // 最大シーケンス番号
    },
    .s_sts_chk = {
        .u8_tx_rand = {0},                  // 送信ステータスチェック乱数
        .u8_rx_rand = {0},                  // 受信ステータスチェック乱数
    },
    .ps_con = NULL,                         // BLEコネクション
};

/** デーモンタスク制御ステータス */
static ts_msg_deamon_sts_t s_msg_deamon_sts = {
    .s_rx_deamon_handle    = NULL,          // RXデーモンタスクハンドル
    .s_rx_queue_handle     = NULL,          // 受信キューハンドラ
    .u32_rx_enqueue_filter = 0x00000000,    // 受信キューイングフィルタ
    .s_evt_deamon_handle   = NULL,          // イベント通知デーモンタスクハンドル
    .s_evt_queue_handle    = NULL,          // イベント通知キューハンドラ
};

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_com_msg_init_svr
 *
 * DESCRIPTION:メッセージサーバーの初期処理
 *
 * PARAMETERS:                  Name            RW  Usage
 * uint16_t                     u16_app_id      R   アプリケーションID
 * uint64_t                     u64_device_id   R   デバイスID
 * uint16_t                     u16_max_length  R   メッセージの最大サイズ
 * tf_com_ble_msg_evt_cb_t      pf_evt_cb       R   イベントコールバック関数
 * tf_com_ble_msg_ticket_cb_t   pf_tkt_cb       R   チケットアクセスコールバック関数
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_init_svr(uint16_t u16_app_id,
                               uint64_t u64_device_id,
                               uint16_t u16_max_length,
                               tf_com_ble_msg_evt_cb_t pf_evt_cb,
                               tf_com_ble_msg_ticket_cb_t pf_tkt_cb) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pf_evt_cb == NULL || pf_tkt_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ミューテックスを生成
    //==========================================================================
    if (s_mutex_sts == NULL) {
        s_mutex_sts = xSemaphoreCreateRecursiveMutex();
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // メッセージサーバーの初期処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // 起動判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.e_mode != COM_BLE_MSG_MODE_BLE_INIT) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 制御設定の初期化
        //----------------------------------------------------------------------
        // サーバー設定
        s_msg_ctrl_cfg.e_mode = COM_BLE_MSG_MODE_BLE_SERVER;    // モード
        s_msg_ctrl_cfg.u16_app_id      = u16_app_id;            // アプリケーションID
        s_msg_ctrl_cfg.u64_device_id   = u64_device_id;         // デバイスID
        s_msg_ctrl_cfg.u32_max_length  = u16_max_length;         // メッセージ最大サイズの設定
        // 利用関数の設定
        s_msg_ctrl_cfg.pf_gatt_if      = t_gatt_if_svr;         // GATTインターフェース取得関数
        s_msg_ctrl_cfg.pf_connect_sts  = e_connect_sts_svr;     // コネクションステータス取得関数
        s_msg_ctrl_cfg.pf_rx_data      = ps_ble_rx_data_svr;    // データ受信関数
        s_msg_ctrl_cfg.pf_tx_msg       = sts_ble_tx_msg_svr;    // メッセージ送信関数
        s_msg_ctrl_cfg.pf_rx_clear     = v_ble_rx_clear_svr;    // 受信キュークリア関数
        s_msg_ctrl_cfg.pf_rx_through   = v_ble_rx_through_svr;  // 受信キュー読み飛ばし関数
        s_msg_ctrl_cfg.pf_tkt_cb       = pf_tkt_cb;             // チケットアクセスコールバック関数
        s_msg_ctrl_cfg.pf_evt_cb       = pf_evt_cb;             // イベントコールバック関数

        //----------------------------------------------------------------------
        // 制御ステータスの初期化
        //----------------------------------------------------------------------
        v_msg_ctrl_sts_init();
        // チケット
        ts_com_msg_auth_ticket_t* ps_rmt_ticket = &s_msg_ctrl_sts.s_rmt_ticket;
        ps_rmt_ticket->u64_own_device_id = s_msg_ctrl_cfg.u64_device_id;
        ps_rmt_ticket->u64_rmt_device_id = s_msg_ctrl_cfg.u64_device_id;

        //----------------------------------------------------------------------
        // SPPサーバーのユーザーコールバック登録
        //----------------------------------------------------------------------
        v_com_ble_spps_set_usr_cb(v_spp_evt_cb_svr);

        //----------------------------------------------------------------------
        // メッセージ処理デーモンタスクの開始
        //----------------------------------------------------------------------
        // メッセージ受信キューのフィルタ設定
        s_msg_deamon_sts.u32_rx_enqueue_filter = 0;
        // デーモンタスクの開始
        sts_val = sts_msg_begin_daemon_task();
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_init_cli
 *
 * DESCRIPTION:メッセージクライアントの初期処理
 *
 * PARAMETERS:                  Name            RW  Usage
 * uint16_t                     u16_app_id      R   アプリケーションID
 * uint64_t                     u64_device_id   R   デバイスID
 * uint16_t                     u16_max_length  R   メッセージの最大サイズ
 * tf_com_ble_msg_evt_cb_t      pf_evt_cb       R   メッセージイベント関数
 * tf_com_ble_msg_ticket_cb_t   pf_tkt_cb       R   チケットアクセスコールバック関数
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_init_cli(uint16_t u16_app_id,
                               uint64_t u64_device_id,
                               uint16_t u16_max_length,
                               tf_com_ble_msg_evt_cb_t pf_evt_cb,
                               tf_com_ble_msg_ticket_cb_t pf_tkt_cb) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pf_evt_cb == NULL || pf_tkt_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // ミューテックスを生成
    //==========================================================================
    if (s_mutex_sts == NULL) {
        s_mutex_sts = xSemaphoreCreateRecursiveMutex();
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // メッセージクライアントの初期処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // 起動判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.e_mode != COM_BLE_MSG_MODE_BLE_INIT) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // MTUサイズの設定
        //----------------------------------------------------------------------
        if (esp_ble_gatt_set_local_mtu(COM_MSG_SIZE_MTU) != ESP_OK) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 制御設定の初期化
        //----------------------------------------------------------------------
        // サーバー設定
        s_msg_ctrl_cfg.e_mode = COM_BLE_MSG_MODE_BLE_CLIENT;    // モード
        s_msg_ctrl_cfg.u16_app_id     = u16_app_id;             // アプリケーションID
        s_msg_ctrl_cfg.u64_device_id  = u64_device_id;          // デバイスID
        s_msg_ctrl_cfg.u32_max_length = u16_max_length;         // メッセージ最大サイズの設定
        // 利用関数の設定
        s_msg_ctrl_cfg.pf_gatt_if     = t_gatt_if_cli;          // GATTインターフェース
        s_msg_ctrl_cfg.pf_connect_sts = e_connect_sts_cli;      // 接続確認関数
        s_msg_ctrl_cfg.pf_rx_data     = ps_ble_rx_data_cli;     // データ受信関数
        s_msg_ctrl_cfg.pf_tx_msg      = sts_ble_tx_msg_cli;     // メッセージ送信関数
        s_msg_ctrl_cfg.pf_rx_clear    = v_ble_rx_clear_cli;     // 受信キュークリア関数
        s_msg_ctrl_cfg.pf_rx_through  = v_ble_rx_through_cli;   // 受信キュー読み飛ばし関数
        s_msg_ctrl_cfg.pf_tkt_cb      = pf_tkt_cb;              // チケットアクセスイベントコールバック関数
        s_msg_ctrl_cfg.pf_evt_cb      = pf_evt_cb;              // 受信イベントコールバック関数

        //----------------------------------------------------------------------
        // 制御ステータスの初期化
        //----------------------------------------------------------------------
        v_msg_ctrl_sts_init();

        //----------------------------------------------------------------------
        // SPPクライアントのユーザーコールバック登録
        //----------------------------------------------------------------------
        v_com_ble_sppc_set_usr_cb(v_spp_evt_cb_cli);

        //----------------------------------------------------------------------
        // メッセージ処理デーモンタスクの開始
        //----------------------------------------------------------------------
        // メッセージ受信キューのフィルタ設定
        s_msg_deamon_sts.u32_rx_enqueue_filter = 0;
        // デーモンタスクの開始
        sts_val = sts_msg_begin_daemon_task();
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_com_msg_rx_enabled
 *
 * DESCRIPTION:受信メッセージのエンキュー有効化処理
 *
 * PARAMETERS:              Name    RW  Usage
 * te_com_ble_msg_type_t    e_type  R   有効化するメッセージタイプ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_msg_rx_enabled(te_com_ble_msg_type_t e_type) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return;
    }

    //==========================================================================
    // メッセージ受信キューのフィルタ設定を有効化
    //==========================================================================
    s_msg_deamon_sts.u32_rx_enqueue_filter |= (0x00000001 << e_type);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);
}

/*******************************************************************************
 *
 * NAME: v_com_msg_rx_disabled
 *
 * DESCRIPTION:受信メッセージのエンキュー無効化処理
 *
 * PARAMETERS:              Name    RW  Usage
 * te_com_ble_msg_type_t    e_type  R   有効化するメッセージタイプ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_msg_rx_disabled(te_com_ble_msg_type_t e_type) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return;
    }

    //==========================================================================
    // メッセージ受信キューのフィルタ設定を有効化
    //==========================================================================
    s_msg_deamon_sts.u32_rx_enqueue_filter ^= (0x00000001 << e_type);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);
}

/*******************************************************************************
 *
 * NAME: v_com_msg_config_pairing
 *
 * DESCRIPTION:ペアリング機能の設定
 *
 * PARAMETERS:  Name            RW  Usage
 * bool         b_enabled       R   ペアリング機能の有効化フラグ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_msg_config_pairing(bool b_enabled) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return;
    }

    //==========================================================================
    // 有効化判定
    //==========================================================================
    if (b_enabled) {
        // ペアリング有効
        s_msg_ctrl_cfg.s_func_ctl |= MSG_FUNC_CTL_PAIRING;
    } else {
        // ペアリング無効
        s_msg_ctrl_cfg.s_func_ctl ^= MSG_FUNC_CTL_PAIRING;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);
}

/*******************************************************************************
 *
 * NAME: v_com_msg_config_sts_chk
 *
 * DESCRIPTION:ステータスチェック機能の設定
 *
 * PARAMETERS:  Name            RW  Usage
 * bool         b_enabled       R   ステータスチェック機能の有効化フラグ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_com_msg_config_sts_chk(bool b_enabled) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return;
    }

    //==========================================================================
    // ステータスチェック要求の送信処理
    //==========================================================================
    // 有効化判定
    if (b_enabled) {
        // ステータスチェック有効
        s_msg_ctrl_cfg.s_func_ctl |= MSG_FUNC_CTL_STS_CHK;
    } else {
        // ステータスチェック無効
        s_msg_ctrl_cfg.s_func_ctl ^= MSG_FUNC_CTL_STS_CHK;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);
}

/*******************************************************************************
 *
 * NAME: b_com_msg_is_paired
 *
 * DESCRIPTION:ペアリング済み判定
 *
 * PARAMETERS:  Name            RW  Usage
 * uint64_t     u64_device_id   R   デバイスID
 *
 * RETURNS:
 *   true:ペアリング済み
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_com_msg_is_paired(uint64_t u64_device_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // チケット取得
    //==========================================================================
    // チケットの有効判定
    bool b_result = b_is_paired(u64_device_id);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return b_result;
}

/*******************************************************************************
 *
 * NAME: e_com_msg_connection_sts
 *
 * DESCRIPTION:接続ステータス取得
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   te_com_ble_msg_connection_sts_t:接続ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_com_ble_msg_connection_sts_t e_com_msg_connection_sts() {
    // 接続ステータス返却
    return s_msg_ctrl_cfg.pf_connect_sts();
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_transaction_sts
 *
 * DESCRIPTION:トランザクションステータスの取得処理
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   te_com_ble_msg_transaction_sts_t:トランザクションステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
te_com_ble_msg_transaction_sts_t sts_com_msg_transaction_sts() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // トランザクションステータスの取得
    //==========================================================================
    te_com_ble_msg_transaction_sts_t e_sts = s_msg_ctrl_sts.s_tran.e_sts;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return e_sts;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_open_server
 *
 * DESCRIPTION:メッセージサーバーへの接続
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_ble_gap_scan_result*  ps_device   R   スキャン結果
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_open_server(ts_com_ble_gap_device_info_t* ps_device) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // パラメータチェック
    if (ps_device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // クライアント判定
    if (s_msg_ctrl_cfg.e_mode != COM_BLE_MSG_MODE_BLE_CLIENT) {
        // クライアントのみ実行可能
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // サーバーへの接続処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // オープントランザクションの開始判定
        //----------------------------------------------------------------------
        uint32_t u32_timeout_ms = 0;
        sts_val = sts_begin_open(ps_device->t_bda, &u32_timeout_ms);
        // トランザクションの開始判定
        if (sts_val != ESP_OK) {
            break;
        }
        //----------------------------------------------------------------------
        // GATT接続シーケンス
        //----------------------------------------------------------------------
        // GATTサーバーへの接続
        esp_gatt_if_t t_gatt_if = s_msg_ctrl_cfg.pf_gatt_if();
        sts_val = sts_com_ble_gattc_open(t_gatt_if, ps_device->t_bda, ps_device->e_addr_type, true);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_close
 *
 * DESCRIPTION:コネクションを切断
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_close() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // ディスコネクト処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    // コネクションを切断
    ts_com_ble_gattc_con_info_t* ps_con = s_msg_ctrl_sts.ps_con;
    if (ps_con != NULL) {
        sts_val = sts_com_ble_disconnect(ps_con->t_bda);
    }
    // 制御ステータス初期化
    v_msg_ctrl_sts_init(s_msg_ctrl_cfg.u64_device_id);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_edit_remote_dev_id
 *
 * DESCRIPTION:接続先デバイスIDの取得
 *
 * PARAMETERS:  Name            RW  Usage
 * uint64_t*    pu64_device_id  R   デバイスIDポインタ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_edit_remote_dev_id(uint64_t* pu64_device_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // デバイスIDの取得
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // 接続確認
        if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
            // 結果ステータス
            sts_val = ESP_ERR_NOT_FOUND;
            break;
        }
        // 受信履歴のチェック
        if (s_msg_ctrl_sts.u64_rmt_device_id == s_msg_ctrl_cfg.u64_device_id) {
            // 結果ステータス
            sts_val = ESP_ERR_NOT_FOUND;
            break;
        }
        // デバイスID編集
        *pu64_device_id = s_msg_ctrl_sts.u64_rmt_device_id;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_edit_public_key_pair
 *
 * DESCRIPTION:ペアリング中の公開鍵のペアの編集処理
 *
 * PARAMETERS:  Name            RW  Usage
 * uint8_t*     pu8_client_key  W   公開鍵（クライアント）
 * uint8_t*     pu8_server_key  W   公開鍵（サーバー）
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_edit_public_key_pair(uint8_t* pu8_client_key, uint8_t* pu8_server_key) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pu8_client_key == NULL || pu8_server_key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // トランザクション開始判定
        //----------------------------------------------------------------------
        ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
        if (ps_tran->e_sts != COM_BLE_MSG_TRN_PAIRING) {
            // ペアリングトランザクションが開始していない
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // タイムアウト判定
        //----------------------------------------------------------------------
        if (ps_tran->u32_timeout_ms < xTaskGetTickCountMSec()) {
            // タイムアウト
            sts_val = ESP_ERR_TIMEOUT;
            break;
        }

        //----------------------------------------------------------------------
        // 接続判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
            // 未接続
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 公開鍵の受信済み判定
        //----------------------------------------------------------------------
        if (!b_is_public_key_received()) {
            // 公開鍵未受信
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 公開鍵のペアを編集
        //----------------------------------------------------------------------
        ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
        // X25519コンテキスト
        ts_crypto_x25519_context_t* ps_x25519_ctx = ps_pairing->ps_x25519_ctx;
        // クライアント側の公開鍵を編集
        memcpy(pu8_client_key, ps_x25519_ctx->u8_cli_public_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
        // サーバー側の公開鍵を編集
        memcpy(pu8_server_key, ps_x25519_ctx->u8_svr_public_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);

    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_com_msg_rx_msg
 *
 * DESCRIPTION:メッセージの読み込み処理
 *
 * PARAMETERS:              Name            RW  Usage
 * TickType_t               t_tick          R   ウェイト時間
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
ts_com_msg_t* ps_com_msg_rx_msg(TickType_t t_tick) {
    // メッセージの読み込み
    ts_com_msg_t* ps_msg = NULL;
    if (xQueueReceive(s_msg_deamon_sts.s_rx_queue_handle, &ps_msg, t_tick) != pdPASS) {
        ps_msg = NULL;
    }
    // 受信データを返却
    return ps_msg;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_rx_wait
 *
 * DESCRIPTION:メッセージの受信ウェイト
 *
 * PARAMETERS:      Name            RW  Usage
 * TickType_t       t_tick          R   タイムアウト時間
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_rx_wait(TickType_t t_tick) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 受信カウント宣言
    //==========================================================================
    uint64_t u64_rx_count_now = s_msg_ctrl_sts.u64_rx_count;
    uint64_t u64_rx_count_pre = u64_rx_count_now;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // タイムアウト時刻を算出
    TickType_t t_timeout = t_tick;
    if (t_timeout != portMAX_DELAY) {
        t_timeout += xTaskGetTickCount();
    }

    //==========================================================================
    // ウェイトループ
    //==========================================================================
    // タイムアウトまでのループ
    do {
        //----------------------------------------------------------------------
        // クリティカルセクション開始
        //----------------------------------------------------------------------
        if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        //----------------------------------------------------------------------
        // 受信カウント更新
        //----------------------------------------------------------------------
        u64_rx_count_pre = u64_rx_count_now;
        u64_rx_count_now = s_msg_ctrl_sts.u64_rx_count;

        //----------------------------------------------------------------------
        // クリティカルセクション終了
        //----------------------------------------------------------------------
        xSemaphoreGiveRecursive(s_mutex_sts);

        //----------------------------------------------------------------------
        // 受信カウント判定
        //----------------------------------------------------------------------
        if (u64_rx_count_now > u64_rx_count_pre) {
            return ESP_OK;
        }
        // ウェイト
        vTaskDelay(COM_MSG_RETRY_WAIT);
    } while(t_timeout >= xTaskGetTickCount());

    // 結果返信
    return ESP_ERR_TIMEOUT;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_reset_msg
 *
 * DESCRIPTION:RESETメッセージの送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_reset_msg() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Resetメッセージ送信
    //==========================================================================
    esp_err_t sts_val = sts_tx_reset_msg();

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_ping_msg
 *
 * DESCRIPTION:PINGメッセージの送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_ping_msg() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // Pingメッセージ送信
    //==========================================================================
    esp_err_t sts_val = sts_tx_ping_msg();

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_pairing_request
 *
 * DESCRIPTION:ペアリング要求
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_pairing_request() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // ペアリングトランザクション開始
        //----------------------------------------------------------------------
        sts_val = sts_begin_pairing();
        if (sts_val != ESP_OK) {
            break;
        }
        // クライアント側のX25519コンテキストの生成
        ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
        ps_pairing->ps_x25519_ctx = ps_crypto_x25519_client_context();

        //----------------------------------------------------------------------
        // ペアリング要求の送信処理
        //----------------------------------------------------------------------
        // メッセージの生成
        ts_u8_array_t* ps_msg_data = ps_create_msg_data(COM_BLE_MSG_TYP_PAIRING_REQ, NULL);
        if (ps_msg_data == NULL) {
            // メッセージ生成エラー
            sts_val = ESP_ERR_NO_MEM;
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        }
        // ペアリング要求送信
        sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg_data);
        if (sts_val != ESP_OK) {
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        }
        // 生成したメッセージを解放
        sts_mdl_delete_u8_array(ps_msg_data);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_pairing_certification
 *
 * DESCRIPTION:ペアリング認証（ダイジェスト比較結果の通知）
 *
 * PARAMETERS:      Name            RW  Usage
 * bool             b_result        R   承認結果
 * uint32_t         u32_max_seq_no  R   最大シーケンス番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_pairing_certification(bool b_result, uint32_t u32_max_seq_no) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    bool b_end_tran = false;
    te_com_ble_msg_event e_evt = COM_BLE_MSG_EVT_PAIRING_SUCCESS;
    do {
        //======================================================================
        // 状態チェック
        //======================================================================
        //----------------------------------------------------------------------
        // ペアリング機能の有効判定
        //----------------------------------------------------------------------
        if ((s_msg_ctrl_cfg.s_func_ctl & MSG_FUNC_CTL_PAIRING) == 0x00) {
            // 機能無効
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // トランザクション開始判定
        //----------------------------------------------------------------------
        ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
        if (ps_tran->e_sts != COM_BLE_MSG_TRN_PAIRING) {
            // ペアリングトランザクションが開始していない
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // ペアリング認証済み判定
        //----------------------------------------------------------------------
        ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
        if ((ps_pairing->e_sts & MSG_PAIRING_CHK_LOCAL) != 0x00) {
            // 既に認証済み
            sts_val = ESP_ERR_INVALID_STATE;
            b_end_tran = true;
            break;
        }

        //----------------------------------------------------------------------
        // タイムアウト判定
        //----------------------------------------------------------------------
        if (ps_tran->u32_timeout_ms < xTaskGetTickCountMSec()) {
            // タイムアウト
            sts_val = ESP_ERR_TIMEOUT;
            b_end_tran = true;
            break;
        }

        //----------------------------------------------------------------------
        // 接続判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
            // 未接続
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 公開鍵の受信済み判定
        //----------------------------------------------------------------------
        if (!b_is_public_key_received()) {
            // 公開鍵未受信
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //======================================================================
        // 認証通知処理
        //======================================================================
        //----------------------------------------------------------------------
        // ステータス判定
        //----------------------------------------------------------------------
        te_com_ble_msg_type_t e_result;
        if (b_result) {
            // ダイジェスト一致
            e_result = COM_BLE_MSG_TYP_DIGEST_MATCH;
            ps_pairing->e_sts |= MSG_PAIRING_CHK_LOCAL;
            // 最大シーケンス番号の更新(後に認証した方が優先される)
            ps_pairing->u32_max_seq_no = u32_max_seq_no;
        } else {
            // ダイジェスト不一致
            e_result = COM_BLE_MSG_TYP_DIGEST_ERR;
            b_end_tran = true;
            e_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
        }

        //----------------------------------------------------------------------
        // チケット生成判定
        //----------------------------------------------------------------------
        if (ps_pairing->e_sts == MSG_PAIRING_CHK_ALL) {
            // ダイジェスト一致
            b_end_tran = true;
            // チケットの生成
            sts_val = sts_create_ticket(ps_tran, ps_pairing);
            if (sts_val != ESP_OK) {
                e_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
                break;
            }
        }

        //----------------------------------------------------------------------
        // ペアリング結果の通知
        //----------------------------------------------------------------------
        // メッセージ生成
        ts_u8_array_t* ps_msg_data = ps_create_msg_data(e_result, NULL);
        if (ps_msg_data == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            b_end_tran = true;
            e_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
            break;
        }
        // ペアリング結果送信
        sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg_data);
        // 生成したメッセージを解放
        sts_mdl_delete_u8_array(ps_msg_data);
        // 結果ステータス判定
        if (sts_val != ESP_OK) {
            b_end_tran = true;
            e_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
            break;
        }
    } while(false);

    //==========================================================================
    // 終了処理
    //==========================================================================
    // トランザクション終了判定
    if (b_end_tran) {
        // イベントエンキュー
        v_msg_evt_enqueue(e_evt);
        // トランザクション終了
        v_msg_ctrl_sts_transaction_reset();
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_sts_chk_request
 *
 * DESCRIPTION:ステータスチェック要求処理
 *
 * PARAMETERS:              Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_sts_chk_request() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // ステータスチェックトランザクション開始
        //----------------------------------------------------------------------
        sts_val = sts_begin_sts_chk();
        if (sts_val != ESP_OK) {
            break;
        }

        //----------------------------------------------------------------------
        // ステータスチェック要求の送信処理
        //----------------------------------------------------------------------
        // メッセージの生成
        ts_u8_array_t* ps_msg_data = ps_create_msg_data(COM_BLE_MSG_TYP_STATUS_REQ, NULL);
        if (ps_msg_data == NULL) {
            // トランザクション終了
            sts_val = ESP_ERR_NO_MEM;
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        }
        // ステータスチェック要求の送信
        sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg_data);
        // 生成したメッセージを解放
        sts_mdl_delete_u8_array(ps_msg_data);
        if (sts_val != ESP_OK) {
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        }
    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_plain_msg
 *
 * DESCRIPTION:平文メッセージの送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * uint64_t                 u64_device_id   R   デバイスID
 * ts_u8_array_t*           ps_data         R   送信メッセージデータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_plain_msg(uint64_t u64_device_id,
                                   ts_u8_array_t* ps_data) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // メッセージ
    ts_u8_array_t* ps_msg = NULL;
    do {
        //----------------------------------------------------------------------
        // 接続判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
            // 未接続
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージ生成
        //----------------------------------------------------------------------
        ps_msg = ps_create_msg_data(COM_BLE_MSG_TYP_DATA, ps_data);
        if (ps_msg == NULL) {
            // メッセージ生成エラー
            sts_val = ESP_ERR_NO_MEM;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージの送信
        //----------------------------------------------------------------------
        sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg);
    } while(false);

    // 送信メッセージ解放
    sts_mdl_delete_u8_array(ps_msg);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_tx_cipher_msg
 *
 * DESCRIPTION:暗号文メッセージの送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * uint64_t                 u64_device_id   R   デバイスID
 * ts_u8_array_t*           ps_data         R   送信メッセージデータ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_tx_cipher_msg(uint64_t u64_device_id,
                                    ts_u8_array_t* ps_data) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 主処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // ペアリング済み判定
        //----------------------------------------------------------------------
        if (!b_is_paired(u64_device_id)) {
            // 未ペアリング
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 接続判定
        //----------------------------------------------------------------------
        if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
            // 未接続
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージ生成
        //----------------------------------------------------------------------
        ts_u8_array_t* ps_msg = ps_create_msg_data(COM_BLE_MSG_TYP_CIPHERTEXT, ps_data);
        if (ps_msg == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージの送信
        //----------------------------------------------------------------------
        sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg);

        //----------------------------------------------------------------------
        // メッセージ解放
        //----------------------------------------------------------------------
        sts_mdl_delete_u8_array(ps_msg);

    } while (false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_delete_msg
 *
 * DESCRIPTION:メッセージの削除処理
 *
 * PARAMETERS:      Name        RW  Usage
 * ts_com_msg_t*    ps_msg      RW  メッセージ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_delete_msg(ts_com_msg_t* ps_msg) {
    // 入力チェック
    if (ps_msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // 本文のデータ
    sts_mdl_delete_u8_array(ps_msg->ps_data);
    ps_msg->ps_data = NULL;
    // メッセージを解放
    l_mem_free(ps_msg);
    // 正常に削除
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_delete_ticket
 *
 * DESCRIPTION:チケットの削除処理
 *
 * PARAMETERS:      Name            RW  Usage
 * uint64_t         u64_device_id   R   デバイスID
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_delete_ticket(uint64_t u64_device_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // チケットの削除処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // キャッシュチケット
        ts_com_msg_auth_ticket_t s_cache_ticket;
        ts_com_msg_auth_ticket_t* ps_ticket = ps_read_ticket(u64_device_id, &s_cache_ticket);
        if (ps_ticket == NULL) {
            // 対象チケットなし
            sts_val = ESP_ERR_NOT_FOUND;
            break;
        }
        // チケットを削除
        sts_val = s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_DELETE, ps_ticket);
        // チケットクリア ※リモートチケットキャッシュのクリアを想定
        v_init_ticket(ps_ticket);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_com_msg_clear_status
 *
 * DESCRIPTION:チケットステータスのクリア処理
 *
 * PARAMETERS:      Name            RW  Usage
 * uint64_t         u64_device_id   R   デバイスID
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_com_msg_clear_status(uint64_t u64_device_id) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // チケットステータスのクリア処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_NOT_FOUND;
    do {
        // キャッシュチケット
        ts_com_msg_auth_ticket_t s_cache_ticket;
        ts_com_msg_auth_ticket_t* ps_ticket = ps_read_ticket(u64_device_id, &s_cache_ticket);
        if (ps_ticket == NULL) {
            // 対象チケットなし
            break;
        }
        // 自デバイスの受信ステータスを乱数で更新
        b_vutil_set_u8_rand_array(ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS);
        // チケットを更新
        sts_val = s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_UPDATE, ps_ticket);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return sts_val;
}


/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_msg_begin_daemon_task
 *
 * DESCRIPTION:begin message daemon task
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_begin_daemon_task() {
    //==========================================================================
    // メッセージ受信デーモンタスクの開始
    //==========================================================================
    // 受信メッセージのキュー生成
    if (s_msg_deamon_sts.s_rx_queue_handle == NULL) {
        s_msg_deamon_sts.s_rx_queue_handle = xQueueCreate(COM_MSG_RX_QUEUE_SIZE, sizeof(ts_com_msg_t*));
    }
    if (s_msg_deamon_sts.s_rx_queue_handle == NULL) {
        // 受信メッセージのキュー生成エラー
        return ESP_FAIL;
    }
    // メッセージ受信デーモンの開始
    portBASE_TYPE b_rx_type = xTaskCreatePinnedToCore(v_msg_rx_daemon_task,
                                                      "msg_rx_deamon_task",
                                                      COM_MSG_RX_DEAMON_STACK_DEPTH,
                                                      (void*)s_msg_deamon_sts.s_rx_queue_handle,
                                                      COM_MSG_RX_DEAMON_PRIORITIES,
                                                      &s_msg_deamon_sts.s_rx_deamon_handle,
                                                      tskNO_AFFINITY);
    if (b_rx_type != pdPASS) {
        // 受信デーモン開始に失敗
        return ESP_FAIL;
    }

    //==========================================================================
    // イベント通知デーモンタスクの開始
    //==========================================================================
    // デーモンにイベント通知するキュー生成
    if (s_msg_deamon_sts.s_evt_queue_handle == NULL) {
        s_msg_deamon_sts.s_evt_queue_handle = xQueueCreate(COM_MSG_EVT_QUEUE_SIZE, sizeof(te_com_ble_msg_event));
    }
    if (s_msg_deamon_sts.s_evt_queue_handle == NULL) {
        // イベント通知キューの生成エラー
        return ESP_FAIL;
    }
    // イベント通知デーモンタスクの開始
    portBASE_TYPE b_evt_type = xTaskCreatePinnedToCore(v_msg_evt_daemon_task,
                                                       "msg_evt_deamon_task",
                                                       COM_MSG_EVT_DEAMON_STACK_DEPTH,
                                                       (void*)s_msg_deamon_sts.s_evt_queue_handle,
                                                       COM_MSG_EVT_DEAMON_PRIORITIES,
                                                       &s_msg_deamon_sts.s_evt_deamon_handle,
                                                       tskNO_AFFINITY);
    if (b_evt_type != pdPASS) {
        // デーモン開始に失敗
        return ESP_FAIL;
    }

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_msg_rx_daemon_task
 *
 * DESCRIPTION:message receiving daemon task
 *
 * PARAMETERS:  Name            RW  Usage
 * void*        pv_parameters   R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_rx_daemon_task(void* pv_parameters) {
    // 現在時刻
    int64_t i64_now_msec = xTaskGetTickCountMSec();
    // 次の遅延時間
    int64_t i64_next_delay_msec = i64_now_msec + COM_MSG_DEAMON_DELAY_INTERVAL_MSEC;
    // パラメータから受信キューハンドルを取得
    QueueHandle_t s_rx_handle = (QueueHandle_t)pv_parameters;
    // 受信結果ステータス
    te_com_ble_msg_rcv_sts_t e_rcv_sts = COM_BLE_MSG_RCV_NOT_FOUND;
    // 受信メッセージ
    ts_com_msg_t s_rx_msg = {0};
    // 受信メッセージ（キュー投入用）
    ts_com_msg_t* ps_rx_msg = NULL;
    // 受信フィルター
    uint32_t u32_rx_flt = 0;
    // タイムアウト時刻
    TickType_t t_timeout = 0;
    // デーモンプロセスの無限ループ
    while (true) {
        //======================================================================
        // 遅延処理　※Watchdog timer measures
        //======================================================================
        i64_now_msec = xTaskGetTickCountMSec();
        if (i64_now_msec >= i64_next_delay_msec) {
            // 遅延処理
            vTaskDelay(1);
            // 遅延時間更新
            i64_next_delay_msec = i64_now_msec + COM_MSG_DEAMON_DELAY_INTERVAL_MSEC;
        }

        //======================================================================
        // トランザクションタイムアウト処理
        //======================================================================
        // クリティカルセクション開始
        if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // トランザクションタイムアウト判定
        ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
        if (ps_tran->u32_timeout_ms < xTaskGetTickCountMSec()) {
            // トランザクションタイムアウト
            v_msg_ctrl_sts_transaction_timeout();
        }

        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex_sts);

        //======================================================================
        // メッセージ受信処理
        //======================================================================
        // メッセージ受信
        e_rcv_sts = e_rx_message(&s_rx_msg, COM_MSG_RX_CHK_TIMEOUT);
        // 受信メッセージの有無を判定
        if (e_rcv_sts == COM_BLE_MSG_RCV_NOT_FOUND) {
            continue;
        }

        //======================================================================
        // 受信メッセージ処理
        //======================================================================
        if (e_rcv_sts == COM_BLE_MSG_RCV_NORMAL) {
            // クリティカルセクション開始
            if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
                continue;
            }

            //------------------------------------------------------------------
            // 受信メッセージチェック
            //------------------------------------------------------------------
            e_rcv_sts = e_rx_msg_check(&s_rx_msg);
            if (e_rcv_sts == COM_BLE_MSG_RCV_NORMAL) {
                // メッセージ受信イベント処理を実行
                e_rcv_sts = e_rx_msg_event(&s_rx_msg);
            }

            //------------------------------------------------------------------
            // メッセージ受信キューのフィルタ設定
            //------------------------------------------------------------------
            u32_rx_flt = s_msg_deamon_sts.u32_rx_enqueue_filter;

            // クリティカルセクション終了
            xSemaphoreGiveRecursive(s_mutex_sts);
        }

        //======================================================================
        // 受信処理の結果判定
        //======================================================================
        // 受信エラー判定
        if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
            // 受信エラーの場合
            // 本文のデータが有ればクリアしてからリトライ
            sts_mdl_delete_u8_array(s_rx_msg.ps_data);
            s_rx_msg.ps_data = NULL;
            continue;
        }
        // エンキュー対象か判定
        if ((u32_rx_flt & (0x00000001 << s_rx_msg.e_type)) == 0x00000000) {
            // エンキュー対象外の場合
            // 本文のデータが有ればクリアしてからリトライ
            sts_mdl_delete_u8_array(s_rx_msg.ps_data);
            s_rx_msg.ps_data = NULL;
            continue;
        }

        //======================================================================
        // 受信メッセージをクローン
        //======================================================================
        ps_rx_msg = (ts_com_msg_t*)pv_mem_clone((void*)&s_rx_msg, sizeof(ts_com_msg_t));
        // オリジナルの本文データをクリアする
        s_rx_msg.ps_data = NULL;

        //======================================================================
        // 受信メッセージエンキュー処理
        //======================================================================
        // 接続してサービス検索完了までウェイト
        t_timeout = xTaskGetTickCount() + COM_MSG_QUEUE_TIMEOUT;
        // 成功するまで実行
        while (xQueueSendToBack(s_rx_handle, &ps_rx_msg, COM_MSG_RETRY_WAIT) != pdPASS) {
            // タイムアウト判定
            if (t_timeout < xTaskGetTickCount()) {
                // メッセージを解放してからリトライ
                sts_com_msg_delete_msg(ps_rx_msg);
                ps_rx_msg = NULL;
                break;
            }
        }
    }
}

/*******************************************************************************
 *
 * NAME: v_msg_evt_daemon_task
 *
 * DESCRIPTION:event processing daemon task
 *
 * PARAMETERS:  Name            RW  Usage
 * void*        pv_parameters   R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_evt_daemon_task(void* pv_parameters) {
    // 現在時刻
    int64_t i64_now_msec = xTaskGetTickCountMSec();
    // 次の遅延時間
    int64_t i64_next_delay_msec = i64_now_msec + COM_MSG_DEAMON_DELAY_INTERVAL_MSEC;
    // パラメータからイベントキューハンドルを取得
    QueueHandle_t s_evt_handle = (QueueHandle_t)pv_parameters;
    // イベント種別
    te_com_ble_msg_event e_msg_evt;
    // デーモンプロセスの無限ループ
    while (true) {
        //======================================================================
        // 遅延処理　※Watchdog timer measures
        //======================================================================
        i64_now_msec = xTaskGetTickCountMSec();
        if (i64_now_msec >= i64_next_delay_msec) {
            // 遅延処理
            vTaskDelay(1);
            // 遅延時間更新
            i64_next_delay_msec = i64_now_msec + COM_MSG_DEAMON_DELAY_INTERVAL_MSEC;
        }

        //======================================================================
        // イベントの読み込み
        //======================================================================
        if (xQueueReceive(s_evt_handle, &e_msg_evt, COM_MSG_EVT_CHK_TIMEOUT) != pdPASS) {
            continue;
        }

        //======================================================================
        // RESET送信イベント判定
        //======================================================================
        if (e_msg_evt == COM_BLE_MSG_EVT_LINK_SUCCESS) {
            // RESET送信イベント
            // 接続ステータス判定
            te_com_ble_msg_connection_sts_t e_con_sts = s_msg_ctrl_cfg.pf_connect_sts();
            if (e_con_sts != COM_BLE_MSG_CON_CONNECTED) {
                continue;
            }

            //------------------------------------------------------------------
            // クリティカルセクション開始
            //------------------------------------------------------------------
            if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
                continue;
            }

            //------------------------------------------------------------------
            // リモートBLEアドレスを編集
            //------------------------------------------------------------------
            v_com_ble_addr_cpy(s_msg_ctrl_sts.t_rmt_bda, s_msg_ctrl_sts.s_tran.t_bda);

            //------------------------------------------------------------------
            // トランザクションリセット
            //------------------------------------------------------------------
            v_msg_ctrl_sts_transaction_reset();

            //------------------------------------------------------------------
            // クリティカルセクション終了
            //------------------------------------------------------------------
            xSemaphoreGiveRecursive(s_mutex_sts);

            //------------------------------------------------------------------
            // Resetメッセージ送信
            // ※内部にクリティカルセクション有り
            //------------------------------------------------------------------
            while (sts_com_msg_tx_reset_msg() != ESP_OK) {
                vTaskDelay(COM_MSG_RETRY_WAIT);
            }
        } else if (e_msg_evt == COM_BLE_MSG_EVT_GATT_DISCONNECT) {
            // メッセージ機能のコネクションを切断 ※スレッドセーフ
            sts_com_msg_close();
        }
        // イベントコールバック
        s_msg_ctrl_cfg.pf_evt_cb(e_msg_evt);
    }

}

/*******************************************************************************
 *
 * NAME: v_msg_evt_enqueue
 *
 * DESCRIPTION:message event enqueue processing
 *
 * PARAMETERS:          Name            RW  Usage
 * te_com_ble_msg_event e_msg_evt       R   メッセージイベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_evt_enqueue(te_com_ble_msg_event e_msg_evt) {
    //==========================================================================
    // イベントエンキュー
    //==========================================================================
    int i_cnt;
    for (i_cnt = 0; i_cnt < COM_MSG_EVT_MAX_RETRY_CNT; i_cnt++) {
        if (xQueueSendToBack(s_msg_deamon_sts.s_evt_queue_handle, &e_msg_evt, COM_MSG_RETRY_WAIT) == pdPASS) {
            break;
        }
    }
}

/*******************************************************************************
 *
 * NAME: v_msg_ctrl_sts_init
 *
 * DESCRIPTION:Message Controller init
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_ctrl_sts_init() {
    //==========================================================================
    // カウンタリセット
    //==========================================================================
    // 送信カウンタ
    s_msg_ctrl_sts.u64_tx_count = 0;
    // 受信カウンタ
    s_msg_ctrl_sts.u64_rx_count = 0;

    //==========================================================================
    // コネクションステータスリセット
    //==========================================================================
    v_msg_ctrl_sts_connection_reset();
}

/*******************************************************************************
 *
 * NAME: v_msg_ctrl_sts_connection_reset
 *
 * DESCRIPTION:Message Controller Connection Reset
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_ctrl_sts_connection_reset() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return;
    }

    //==========================================================================
    // コネクションリセット処理
    //==========================================================================
    do {
        //----------------------------------------------------------------------
        // 受信データのクリア
        //----------------------------------------------------------------------
        s_msg_ctrl_cfg.pf_rx_clear();

        //----------------------------------------------------------------------
        // リモートデバイス情報をクリア
        //----------------------------------------------------------------------
        // リモートデバイスID
        s_msg_ctrl_sts.u64_rmt_device_id = s_msg_ctrl_cfg.u64_device_id;
        // リモートデバイスBLEアドレス
        v_com_ble_addr_clear(s_msg_ctrl_sts.t_rmt_bda);
        // リモートデバイスチケット
        v_init_ticket(&s_msg_ctrl_sts.s_rmt_ticket);

        //----------------------------------------------------------------------
        // 送受信履歴のクリア
        //----------------------------------------------------------------------
        // 直前の送信履歴
        v_msg_history_reset(&s_msg_ctrl_sts.s_bef_tx_msg);
        // 直前の受信履歴
        v_msg_history_reset(&s_msg_ctrl_sts.s_bef_rx_msg);
        // 直前の返信履歴
        v_msg_history_reset(&s_msg_ctrl_sts.s_bef_rx_rsp);

        //----------------------------------------------------------------------
        // トランザクションステータスリセット
        //----------------------------------------------------------------------
        v_msg_ctrl_sts_transaction_reset();

        //----------------------------------------------------------------------
        // BLEコネクションリセット
        //----------------------------------------------------------------------
        v_com_ble_gattc_delete_con_info(s_msg_ctrl_sts.ps_con);
        s_msg_ctrl_sts.ps_con = NULL;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);
}

/*******************************************************************************
 *
 * NAME: v_msg_ctrl_sts_transaction_reset
 *
 * DESCRIPTION:Message Controller transaction clear
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_ctrl_sts_transaction_reset() {
    //--------------------------------------------------------------------------
    // トランザクション状態判定
    //--------------------------------------------------------------------------
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    if (ps_tran->e_sts == COM_BLE_MSG_TRN_NONE) {
        return;
    }

    //--------------------------------------------------------------------------
    // トランザクションステータス
    //--------------------------------------------------------------------------
    ps_tran->e_sts         = COM_BLE_MSG_TRN_NONE;          // トランザクションステータス
    ps_tran->u64_device_id = s_msg_ctrl_cfg.u64_device_id;  // トランザクション実行中の相手デバイスID
    v_com_ble_addr_clear(ps_tran->t_bda);                   // トランザクション実行中の相手デバイスBLEアドレス
    ps_tran->u32_timeout_ms = U32_MAX;                      // トランザクションタイムアウト

    //--------------------------------------------------------------------------
    // ペアリングステータス
    //--------------------------------------------------------------------------
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // ペアリングステータス
    ps_pairing->e_sts = MSG_PAIRING_CHK_NONE;
    // 共通鍵
    memset(ps_pairing->u8_com_key, 0x00, COM_MSG_SIZE_CIPHER_KEY);
    // X25519コンテキスト
    v_crypto_x25519_delete_context(ps_pairing->ps_x25519_ctx);
    ps_pairing->ps_x25519_ctx = NULL;
    // 自デバイスステータス
    memset(ps_pairing->u8_dev_status, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 相手デバイスステータスハッシュ
    memset(ps_pairing->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 最大シーケンス番号
    ps_pairing->u32_max_seq_no = 0;

    //--------------------------------------------------------------------------
    // ステータスチェック
    //--------------------------------------------------------------------------
    ts_sts_check_info_t* ps_sts_chk = &s_msg_ctrl_sts.s_sts_chk;
    memset(ps_sts_chk->u8_tx_rand, 0x00, COM_MSG_SIZE_TICKET_STS);  // 送信ステータスチェック乱数
    memset(ps_sts_chk->u8_rx_rand, 0x00, COM_MSG_SIZE_TICKET_STS);  // 受信ステータスチェック乱数
}

/*******************************************************************************
 *
 * NAME: v_msg_ctrl_sts_transaction_timeout
 *
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_ctrl_sts_transaction_timeout() {
    // トランザクション判定
    switch (s_msg_ctrl_sts.s_tran.e_sts) {
    case COM_BLE_MSG_TRN_OPEN:
        // オープントランザクション実行中
        // BLEの切断処理
        sts_com_ble_disconnect(s_msg_ctrl_sts.s_tran.t_bda);
        // イベントエンキュー
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_OPEN_TIMEOUT);
        break;
    case COM_BLE_MSG_TRN_PAIRING:
        // ペアリングトランザクション実行中
        // イベントエンキュー
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_PAIRING_TIMEOUT);
        break;
    case COM_BLE_MSG_TRN_STS_CHK:
        // ステータスチェックトランザクション実行中
        // イベントエンキュー
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_STATUS_TIMEOUT);
        break;
    default:
        break;
    }
    // トランザクションをクリア
    v_msg_ctrl_sts_transaction_reset();
}

/*******************************************************************************
 *
 * NAME: v_msg_history_reset
 *
 * DESCRIPTION:Message history reset processing
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_msg_history*  ps_msg_history  W   対象のメッセージ履歴
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_history_reset(ts_msg_history_t* ps_msg_history) {
    // 送受信履歴
    v_com_ble_addr_clear(ps_msg_history->t_bda);
    ps_msg_history->u64_device_id  = s_msg_ctrl_cfg.u64_device_id;
    ps_msg_history->u32_tick_ms    = 0;
    ps_msg_history->u32_seq_no     = 0;
    ps_msg_history->e_type         = COM_BLE_MSG_TYP_CNT;
}

/*******************************************************************************
 *
 * NAME: ps_get_connection
 *
 * DESCRIPTION:BLE Get Connection
 *
 * PARAMETERS:          Name            RW  Usage
 *
 * RETURNS:
 * ts_com_ble_gattc_con_info*:サーバーへのコネクション
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_ble_gattc_con_info_t* ps_get_connection() {
    //==========================================================================
    // リモートデバイスBLEアドレスの取得
    //==========================================================================
    esp_bd_addr_t t_rmt_bda;
    if (s_msg_ctrl_sts.s_tran.e_sts == COM_BLE_MSG_TRN_NONE) {
        // 接続済みアドレス
        v_com_ble_addr_cpy(t_rmt_bda, s_msg_ctrl_sts.t_rmt_bda);
    } else {
        // 接続中アドレス
        v_com_ble_addr_cpy(t_rmt_bda, s_msg_ctrl_sts.s_tran.t_bda);
    }
    // 有効アドレスチェック
    if (b_com_ble_addr_clear(t_rmt_bda)) {
        return NULL;
    }

    //==========================================================================
    // コネクションの有効判定
    //==========================================================================
    esp_gatt_if_t t_gatt_if = s_msg_ctrl_cfg.pf_gatt_if();
    te_gattc_con_sts_t e_con_sts = e_com_ble_gattc_con_sts(t_gatt_if, t_rmt_bda);
    if ((e_con_sts & GATTC_STS_CONNECTED) != GATTC_STS_CONNECTED) {
        // コネクションが無効の場合は既存のコネクション情報をクリア
        v_com_ble_gattc_delete_con_info(s_msg_ctrl_sts.ps_con);
        s_msg_ctrl_sts.ps_con = NULL;
        return NULL;
    }

    //==========================================================================
    // コネクション情報の取得
    //==========================================================================
    // コネクション情報の有効判定
    ts_com_ble_gattc_con_info_t* ps_con = s_msg_ctrl_sts.ps_con;
    if (ps_con != NULL) {
        if (l_com_ble_addr_cmp(t_rmt_bda, ps_con->t_bda) == 0) {
            return ps_con;
        }
        // リモートデバイスのコネクションでは無いので解放する
        v_com_ble_gattc_delete_con_info(s_msg_ctrl_sts.ps_con);
        s_msg_ctrl_sts.ps_con = NULL;
    }
    // コネクション情報生成
    ps_con = ps_com_ble_gattc_create_con_info(t_gatt_if, t_rmt_bda);
    if (ps_con == NULL) {
        return NULL;
    }
    // サービスの有無を判定
    if (ps_con->u16_svc_cnt == 0) {
        // サービスの無いコネクション情報は無効なのでクリア
        v_com_ble_gattc_delete_con_info(ps_con);
        return NULL;
    }
    // コネクション情報の更新
    s_msg_ctrl_sts.ps_con = ps_con;
    // 取得結果を返却
    return s_msg_ctrl_sts.ps_con;
}

/*******************************************************************************
 *
 * NAME: e_rx_message
 *
 * DESCRIPTION:BLE Rx Message
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_t*                ps_rx_msg   W   受信メッセージの編集対象
 * TickType_t                   t_tick      R   ウェイト時間
 *
 * RETURNS:
 *   te_com_ble_msg_rcv_sts_t: 受信結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_rcv_sts_t e_rx_message(ts_com_msg_t* ps_rx_msg,
                                              TickType_t t_tick) {
    // 型変換構造体
    tu_type_converter_t u_conv;
    // 受信ステータス
    te_com_ble_msg_rcv_sts_t e_rcv_sts = COM_BLE_MSG_RCV_NORMAL;
    // 受信データ(UART)
    ts_com_ble_gatt_rx_data_t* ps_ble_data = NULL;
    // 受信メッセージ全体のバッファ
    ts_u8_array_t* ps_msg_buff = NULL;
    // データ受信関数
    tf_ble_rx_data_t pf_rx_data = s_msg_ctrl_cfg.pf_rx_data;

    //==========================================================================
    // 先頭データ読み込み
    //==========================================================================
    ps_ble_data = pf_rx_data(t_tick);
    if (ps_ble_data == NULL) {
        // 受信データなし
        return COM_BLE_MSG_RCV_NOT_FOUND;
    }

    //==========================================================================
    // メッセージ読み込み処理
    //==========================================================================
    do {
        //----------------------------------------------------------------------
        // ヘッダー情報の編集
        //----------------------------------------------------------------------
        // メッセージヘッダー部を編集
        e_rcv_sts = e_edit_rx_header(ps_rx_msg, ps_ble_data);
        if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
            // 受信ヘッダーエラー
            // 受信データなし
            break;
        }

        //----------------------------------------------------------------------
        // 受信データをコピー
        //----------------------------------------------------------------------
        // メッセージ全体の領域を確保
        ps_msg_buff = ps_mdl_empty_u8_array(ps_rx_msg->u16_length);
        if (ps_msg_buff == NULL) {
            // メモリ確保に失敗
            // 受信応答を送信
            e_rcv_sts = COM_BLE_MSG_RCV_RECEIVER_ERR;
            break;
        }
        // 受信データ
        ts_u8_array_t* ps_rx_data = ps_ble_data->ps_array;
        // 先頭の受信データをコピー
        memcpy(ps_msg_buff->pu8_values, ps_rx_data->pu8_values, ps_rx_data->t_size);

        //----------------------------------------------------------------------
        // フッターまで受信
        //----------------------------------------------------------------------
        uint32_t u32_msg_length = ps_rx_msg->u16_length;
        uint32_t u32_pos = ps_rx_data->t_size;
        while (u32_pos < u32_msg_length) {
            //------------------------------------------------------------------
            // データ受信
            //------------------------------------------------------------------
            // 直前の受信データを解放
            v_com_ble_gatt_delete_rx_data(ps_ble_data);
            // データ受信
            ps_ble_data = pf_rx_data(t_tick);
            if (ps_ble_data == NULL) {
                // 受信タイムアウト
                // 受信応答を送信
                e_rcv_sts = COM_BLE_MSG_RCV_TIMEOUT_ERR;
                // キューが空なのでブレーク
                break;
            }

            //------------------------------------------------------------------
            // 受信データ長チェック
            //------------------------------------------------------------------
            ps_rx_data = ps_ble_data->ps_array;
            if ((u32_pos + ps_rx_data->t_size) > u32_msg_length) {
                // 受信データサイズエラー
                // 受信応答を送信
                e_rcv_sts = COM_BLE_MSG_RCV_LENGTH_ERR;
                break;
            }

            //------------------------------------------------------------------
            // 送信元デバイスチェック
            //------------------------------------------------------------------
            if (l_com_ble_addr_cmp(ps_ble_data->t_bda, ps_rx_msg->t_rcv_bda) != 0) {
                // 受信アドレスエラー
                // 受信応答を送信
                e_rcv_sts = COM_BLE_MSG_RCV_ADDRESS_ERR;
                break;
            }

            //------------------------------------------------------------------
            // 受信データをコピー
            //------------------------------------------------------------------
            memcpy(&ps_msg_buff->pu8_values[u32_pos], ps_rx_data->pu8_values, ps_rx_data->t_size);
            u32_pos = u32_pos + ps_rx_data->t_size;
        }
        // エラー判定
        if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
            break;
        }

        //======================================================================
        // 受信メッセージの単体チェック
        //======================================================================
        //----------------------------------------------------------------------
        // ストップトークンチェック
        //----------------------------------------------------------------------
        // 受信メッセージ
        uint8_t* pu8_msg_buff = ps_msg_buff->pu8_values;
        u_conv.u8_values[0] = pu8_msg_buff[ps_msg_buff->t_size - 2];
        u_conv.u8_values[1] = pu8_msg_buff[ps_msg_buff->t_size - 1];
        if (u_conv.u16_values[0] != (uint16_t)ps_rx_msg->u32_seq_no) {
            // ストップトークンエラー
            // 受信応答を送信
            e_rcv_sts = COM_BLE_MSG_RCV_STOP_TKN_ERR;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージの署名タグチェック
        //----------------------------------------------------------------------
        // メッセージの認証ハッシュ生成
        uint8_t u8_auth_tag[COM_MSG_SIZE_AUTH_TAG];
        if (sts_edit_auth_tag(u8_auth_tag, ps_msg_buff) != ESP_OK) {
            // メッセージ認証タグの生成エラー
            e_rcv_sts = COM_BLE_MSG_RCV_RECEIVER_ERR;
            break;
        }
        // ハッシュ値の検証
        if (memcmp(ps_rx_msg->u8_auth_tag, u8_auth_tag, COM_MSG_SIZE_AUTH_TAG) != 0) {
            // 認証タグエラー
            e_rcv_sts = COM_BLE_MSG_RCV_AUTH_ERR;
            break;
        }

        //======================================================================
        // メッセージ本文の編集
        //======================================================================
        // 本文のサイズ判定
        uint16_t u16_body_size = ps_rx_msg->u16_length - (MSG_SIZE_HEADER + MSG_SIZE_FOOTER);
        if (u16_body_size > 0) {
            // 暗号文も一時的にそのままデータとして編集
            ps_rx_msg->ps_data = ps_mdl_clone_u8_array(&pu8_msg_buff[MSG_POS_BODY], u16_body_size);
            if (ps_rx_msg->ps_data == NULL) {
                e_rcv_sts = COM_BLE_MSG_RCV_NO_MEM_ERR;
                break;
            }
        }

#ifdef COM_BLE_MSG_DEBUG
        if (ps_rx_msg->e_type != COM_BLE_MSG_TYP_RESPONSE) {
            ESP_LOGW(LOG_TAG, "%s L#%d rx type=%d", __func__, __LINE__, ps_rx_msg->e_type);
        } else {
            ESP_LOGW(LOG_TAG, "%s L#%d rx rsp_type=%d tick=%ldms", __func__, __LINE__, ps_rx_msg->ps_data->pu8_values[0], xTaskGetTickCount() * portTICK_PERIOD_MS);
        }
#endif
    } while(false);

    //==========================================================================
    // 終了処理
    //==========================================================================
    // 受信エラー処理
    if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
       // 受信エラーの場合
        // コネクションリセット
        v_msg_ctrl_sts_connection_reset();
    }
    // 受信メッセージバッファを削除
    sts_mdl_delete_u8_array(ps_msg_buff);
    // BLE受信データを削除
    v_com_ble_gatt_delete_rx_data(ps_ble_data);

    // 結果ステータスを返信
    return e_rcv_sts;
}

/*******************************************************************************
 *
 * NAME: e_rx_msg_check
 *
 * DESCRIPTION:Rx Message check
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_msg_t*        ps_rx_msg   W   受信メッセージの編集対象
 *
 * RETURNS:
 *   te_com_ble_msg_rcv_sts_t: 受信結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_rcv_sts_t e_rx_msg_check(ts_com_msg_t* ps_rx_msg) {
    // 受信ステータス
    te_com_ble_msg_rcv_sts_t e_rcv_sts = COM_BLE_MSG_RCV_NORMAL;
    // メッセージ定義の取得
    const ts_msg_definition_t* ps_rx_def = &MSG_DEF[ps_rx_msg->e_type];
    // 本文データ
    ts_u8_array_t* ps_data = ps_rx_msg->ps_data;
    // リモートチケット
    ts_com_msg_auth_ticket_t* ps_ticket = NULL;
    // 直前の受信履歴
    ts_msg_history_t* ps_bef_rx_msg = &s_msg_ctrl_sts.s_bef_rx_msg;
    // トランザクション情報
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    // ペアリング情報
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // 受信SEQのカウントアップフラグ
    bool b_rx_seq_update = false;

    do {
        //----------------------------------------------------------------------
        // デバイスIDのチェック
        //----------------------------------------------------------------------
        if (ps_rx_msg->u64_device_id == s_msg_ctrl_cfg.u64_device_id) {
            // デバイスIDエラー
            e_rcv_sts = COM_BLE_MSG_RCV_DEV_ID_ERR;
            break;
        }

        //----------------------------------------------------------------------
        // 直前の受信履歴のメッセージタイプをチェック
        //----------------------------------------------------------------------
        if (((ps_rx_def->u16_map_before_rx >> ps_bef_rx_msg->e_type) & 0x0001) != 0x0000) {
            // 直前の受信メッセージタイプエラー
            e_rcv_sts = COM_BLE_MSG_RCV_TRAN_ERR;
            break;
        }

        //----------------------------------------------------------------------
        // メッセージヘッダーのトランザクション関連チェック
        //----------------------------------------------------------------------
        // トランザクションチェックタイプ
        if (ps_rx_def->e_chk_trn_type == MSG_CHK_TRN_EXEC) {
            // トランザクション実行中チェック
            if (ps_tran->e_sts == COM_BLE_MSG_TRN_NONE) {
                // トランザクションが実行中では無いのでエラー
                // 受信タイプエラー
                e_rcv_sts = COM_BLE_MSG_RCV_TYPE_ERR;
                break;
            }
            // トランザクションのデバイスIDチェック
            if (ps_rx_msg->u64_device_id != ps_tran->u64_device_id) {
                // メッセージのデバイスIDが違うのでエラー
                e_rcv_sts = COM_BLE_MSG_RCV_TRAN_ERR;
                break;
            }
            // 物理アドレスチェック
            if (l_com_ble_addr_cmp(ps_rx_msg->t_rcv_bda, ps_tran->t_bda) != 0) {
                // トランザクションの通信相手エラー
                e_rcv_sts = COM_BLE_MSG_RCV_TRAN_ERR;
                break;
            }
            // トランザクションのタイムアウトをチェック
            if (ps_rx_msg->u32_rcv_tick_ms > ps_tran->u32_timeout_ms) {
                // トランザクションタイムアウト
                e_rcv_sts = COM_BLE_MSG_RCV_TRAN_ERR;
                break;
            }
        } else if (ps_rx_def->e_chk_trn_type == MSG_CHK_TRN_STOP) {
            // トランザクション停止中チェック
            if (ps_tran->e_sts != COM_BLE_MSG_TRN_NONE) {
                // トランザクションが実行中なのでエラー
                // 受信タイプエラー
                e_rcv_sts = COM_BLE_MSG_RCV_TYPE_ERR;
                break;
            }
        }

        //----------------------------------------------------------------------
        // シーケンス番号の関連チェック
        //----------------------------------------------------------------------
        // シーケンス番号のチェック用メッセージ定義（受信通知の場合には受信メッセージタイプ）
        const ts_msg_definition_t* ps_seq_chk_def = ps_rx_def;
        // シーケンス番号のチェックのメッセージ定義判定
        if (ps_rx_msg->e_type == COM_BLE_MSG_TYP_RESPONSE) {
            // レスポンスタイプを判定
            uint8_t u8_seq_chk_type = ps_data->pu8_values[0];
            if (u8_seq_chk_type == COM_BLE_MSG_TYP_RESPONSE) {
                // チェックタイプエラー
                e_rcv_sts = COM_BLE_MSG_RCV_TYPE_ERR;
                break;
            }
            // 受信通知の場合には、送信時のメッセージタイプで判定
            ps_seq_chk_def = &MSG_DEF[u8_seq_chk_type];
        }
        // シーケンス番号チェックタイプの判定
        if (ps_seq_chk_def->b_fixed_seq) {
            // シーケンス番号が固定値の場合
            if (ps_rx_msg->u32_seq_no != ps_seq_chk_def->u32_seq_no) {
                // シーケンスエラー
                e_rcv_sts = COM_BLE_MSG_RCV_SEQ_ERR;
                break;
            }
        } else {
            // シーケンス番号が可変値の場合
            // チケット読み込み
            ps_ticket = ps_read_ticket(ps_rx_msg->u64_device_id, &s_msg_ctrl_sts.s_rmt_ticket);
            if (ps_ticket == NULL) {
                // チケットが無いのでペアリングエラー
                e_rcv_sts = COM_BLE_MSG_RCV_PAIRING_ERR;
                break;
            }
            // メッセージタイプ判定
            if (ps_rx_msg->e_type == COM_BLE_MSG_TYP_RESPONSE) {
                // 受信通知の場合
                // 受信メッセージのSEQ番号チェック
                if (ps_rx_msg->u32_seq_no > ps_ticket->u32_tx_seq_no) {
                    // 受信シーケンス番号エラー
                    e_rcv_sts = COM_BLE_MSG_RCV_SEQ_ERR;
                    break;
                }
            } else {
                // 受信通知以外の場合
                // 受信メッセージのSEQ番号チェック
                if (ps_rx_msg->u32_seq_no <= ps_ticket->u32_rx_seq_no) {
                    // 過去に受信したシーケンス番号以下なのでエラー
                    e_rcv_sts = COM_BLE_MSG_RCV_SEQ_ERR;
                    break;
                }
                // 受信SEQのアップデートフラグ
                b_rx_seq_update = true;
            }
        }

        //======================================================================
        // メッセージ本文の復号
        //======================================================================
        // 本文の暗号判定
        if (ps_data != NULL && ps_rx_def->b_encryption) {
            // 本文を復号
            if (ps_rx_msg->e_type == COM_BLE_MSG_TYP_DIGEST_MATCH) {
                // ダイジェスト一致の場合
                ps_data = ps_msg_decryption(ps_rx_msg, ps_pairing->u8_com_key);
            } else {
                // 暗号データの場合
                ts_u8_array_t* ps_pack_data = ps_msg_decryption(ps_rx_msg, ps_ticket->u8_enc_key);
                // アンパディング
                ps_data = ps_crypto_pkcs7_unpadding(ps_pack_data, AES_BLOCK_BYTES);
                // 元の平文を解放
                sts_mdl_delete_u8_array(ps_pack_data);
            }
            // 本文をプレーンデータと差し替え
            sts_mdl_delete_u8_array(ps_rx_msg->ps_data);
            ps_rx_msg->ps_data = ps_data;
            // 編集結果判定
            if (ps_rx_msg->ps_data == NULL) {
                // 復号エラー
                e_rcv_sts = COM_BLE_MSG_RCV_DECRYPT_ERR;
                break;
            }
        }

        //======================================================================
        // 受信SEQの更新
        //======================================================================
        if (b_rx_seq_update) {
            // 受信シーケンス更新
            ps_ticket->u32_rx_seq_no = ps_rx_msg->u32_seq_no;
            if (s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_UPDATE, ps_ticket) != ESP_OK) {
                // シーケンス番号エラーとする
                e_rcv_sts = COM_BLE_MSG_RCV_SEQ_ERR;
                // 異常終了
                break;
            }
        }

#ifdef COM_BLE_MSG_DEBUG
        if (ps_rx_msg->e_type != COM_BLE_MSG_TYP_RESPONSE) {
            ESP_LOGW(LOG_TAG, "%s L#%d rx type=%d", __func__, __LINE__, ps_rx_msg->e_type);
        } else {
            ESP_LOGW(LOG_TAG, "%s L#%d rx rsp_type=%d tick=%ldms", __func__, __LINE__, ps_rx_msg->ps_data->pu8_values[0], xTaskGetTickCount() * portTICK_PERIOD_MS);
        }
#endif
    } while(false);

    //==========================================================================
    // 終了処理
    //==========================================================================
    // 受信エラー処理
    if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
        // 受信エラーの場合
        // コネクションリセット
        v_msg_ctrl_sts_connection_reset();
    }

    // 結果ステータスを返信
    return e_rcv_sts;
}

/*******************************************************************************
 *
 * NAME: e_rx_msg_event
 *
 * DESCRIPTION:BLE Rx Message event processing
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_t*                ps_rx_msg   R   受信メッセージ
 *
 * RETURNS:
 *   te_com_ble_msg_rcv_sts_t: 受信結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_rcv_sts_t e_rx_msg_event(ts_com_msg_t* ps_rx_msg) {
    // 結果ステータス
    te_com_ble_msg_rcv_sts_t e_rcv_sts = COM_BLE_MSG_RCV_NORMAL;
    // コールバックイベント
    te_com_ble_msg_event e_cb_evt = COM_BLE_MSG_EVT_COUNT;
    // 受信メッセージ本文
    ts_u8_array_t* ps_rx_data = ps_rx_msg->ps_data;
    // リモートデバイスチケット
    ts_com_msg_auth_ticket_t* ps_ticket = &s_msg_ctrl_sts.s_rmt_ticket;
    // トランザクション情報
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    // ペアリング情報
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // ステータスチェック
    ts_sts_check_info_t* ps_sts_chk = &s_msg_ctrl_sts.s_sts_chk;

    do {
        //======================================================================
        // リモートデバイス情報の更新処理
        //======================================================================
        // リモートデバイスのBluetoothアドレス
        v_com_ble_addr_cpy(s_msg_ctrl_sts.t_rmt_bda, ps_rx_msg->t_rcv_bda);
        // 接続判定
        if (s_msg_ctrl_sts.u64_rmt_device_id != ps_rx_msg->u64_device_id) {
            // リモートデバイスID
            s_msg_ctrl_sts.u64_rmt_device_id = ps_rx_msg->u64_device_id;
            // 接続イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_OPEN_SUCCESS);
        }

        //======================================================================
        // 受信処理
        // ※メッセージタイプ毎の受信メッセージのハンドリング処理
        //======================================================================
        // 受信データ本文（ダイジェスト一致）
        ts_msg_digest_match_t* ps_digest_match = NULL;
        // 公開鍵（受信）
        uint8_t u8_receive_key[CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE];
        // 送信メッセージ
        ts_u8_array_t* ps_tx_msg = NULL;

        //----------------------------------------------------------------------
        // イベント判定
        //----------------------------------------------------------------------
        esp_err_t sts_val = ESP_OK;
        switch (ps_rx_msg->e_type) {
        case COM_BLE_MSG_TYP_RESPONSE:
            //------------------------------------------------------------------
            // 受信通知
            //------------------------------------------------------------------
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_RX_RESPONSE);
            break;
        case COM_BLE_MSG_TYP_RESET:
            //------------------------------------------------------------------
            // リセット
            //------------------------------------------------------------------
            // コネクションリセット
            v_msg_ctrl_sts_connection_reset();
            // リモートデバイスIDを再設定
            s_msg_ctrl_sts.u64_rmt_device_id = ps_rx_msg->u64_device_id;
            // リモートデバイスのBluetoothアドレスを再設定
            v_com_ble_addr_cpy(s_msg_ctrl_sts.t_rmt_bda, ps_rx_msg->t_rcv_bda);
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_RX_RESET);
            break;
        case COM_BLE_MSG_TYP_PING:
            //------------------------------------------------------------------
            // PING
            //------------------------------------------------------------------
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_RX_PING);
            break;
        case COM_BLE_MSG_TYP_PAIRING_REQ:
            //------------------------------------------------------------------
            // ペアリング要求
            //------------------------------------------------------------------
            // ペアリングトランザクションの初期処理
            if (sts_begin_pairing() != ESP_OK) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // 相手デバイス公開鍵を保存
            // 受信した公開鍵（Curve25519を想定）を設定
            u8_receive_key[0] = 0x03;
            u8_receive_key[1] = 0x00;
            u8_receive_key[2] = 0x1D;
            u8_receive_key[3] = 0x20;
            memcpy(&u8_receive_key[4], ps_rx_data->pu8_values, ps_rx_data->t_size);
            // サーバー側のX25519コンテキストを生成
            ps_pairing->ps_x25519_ctx = ps_crypto_x25519_server_context(u8_receive_key);
            // コンテキストの生成を確認
            if (ps_pairing->ps_x25519_ctx == NULL) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_NO_MEM_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // 共通鍵のコピー
            memcpy(ps_pairing->u8_com_key, ps_pairing->ps_x25519_ctx->u8_key, CRYPTO_X25519_KEY_SIZE);
#ifdef COM_BLE_MSG_DEBUG
            do {
                uint32_t u32_key_len = (COM_MSG_SIZE_CIPHER_KEY * 2) + 1;
                char c_key_str[u32_key_len];
                v_vutil_u8_to_hex_string(ps_pairing->u8_com_key, COM_MSG_SIZE_CIPHER_KEY, c_key_str);
                ESP_LOGW(LOG_TAG, "%s L#%d com_key=%s", __func__, __LINE__, c_key_str);
            } while(false);
#endif
            // ペアリング応答メッセージ生成
            ps_tx_msg = ps_create_msg_data(COM_BLE_MSG_TYP_PAIRING_RSP, NULL);
            if (ps_tx_msg == NULL) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // ペアリング応答送信
            sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_tx_msg);
            // 送信済みデータを解放
            sts_mdl_delete_u8_array(ps_tx_msg);
            if (sts_val != ESP_OK) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_PAIRING_START);
            break;
        case COM_BLE_MSG_TYP_PAIRING_RSP:
            //------------------------------------------------------------------
            // ペアリング応答
            //------------------------------------------------------------------
            if (ps_pairing->ps_x25519_ctx == NULL) {
                // ペアリングエラー
                e_rcv_sts = COM_BLE_MSG_RCV_PAIRING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // 受信した公開鍵（Curve25519を想定）を設定
            u8_receive_key[0] = 0x20;
            memcpy(&u8_receive_key[1], ps_rx_data->pu8_values, ps_rx_data->t_size);
            // 共通鍵を生成
            sts_val = sts_crypto_x25519_client_secret(ps_pairing->ps_x25519_ctx, u8_receive_key);
            if (sts_val != ESP_OK) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_NO_MEM_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // 共通鍵をコピー
            memcpy(ps_pairing->u8_com_key, ps_pairing->ps_x25519_ctx->u8_key, CRYPTO_X25519_KEY_SIZE);
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_PAIRING_START);
            break;
        case COM_BLE_MSG_TYP_DIGEST_MATCH:
            //------------------------------------------------------------------
            // ダイジェスト一致
            //------------------------------------------------------------------
            ps_digest_match = (ts_msg_digest_match_t*)ps_rx_data->pu8_values;
            // 相手ステータスハッシュ
            memcpy(ps_pairing->u8_rmt_sts_hash, ps_digest_match->u8_sts_hash, COM_MSG_SIZE_TICKET_STS);
            // 最大シーケンス番号
            ps_pairing->u32_max_seq_no = ps_digest_match->u32_max_seq_no;
#ifdef COM_BLE_MSG_DEBUG
            do {
                char sts_txt[(COM_MSG_SIZE_TICKET_STS * 2) + 1];
                v_vutil_u8_to_hex_string(ps_pairing->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS, sts_txt);
                ESP_LOGW(LOG_TAG, "%s L#%d rx_hash=%s", __func__, __LINE__, sts_txt);
                ESP_LOGW(LOG_TAG, "%s L#%d rx_no=%ld", __func__, __LINE__, ps_pairing->u32_max_seq_no);
            } while(false);
#endif
            // 相互認証チェック
            if (ps_pairing->e_sts == MSG_PAIRING_CHK_LOCAL) {
                // 相互認証完了
                // チケットの書き込み
                if (sts_create_ticket(ps_tran, ps_pairing) != ESP_OK) {
                    // ペアリングエラー
                    // 受信ステータス
                    e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                    // ユーザーイベント
                    e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                    break;
                }
                // イベントエンキュー
                v_msg_evt_enqueue(COM_BLE_MSG_EVT_PAIRING_SUCCESS);
                // トランザクション終了
                v_msg_ctrl_sts_transaction_reset();
                break;
            }
            // 既に受信しているか判定
            if (ps_pairing->e_sts == MSG_PAIRING_CHK_REMOTE) {
                // ダイジェスト一致を複数回受信
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_PAIRING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
                break;
            }
            // リモートダイジェスト一致
            ps_pairing->e_sts = MSG_PAIRING_CHK_REMOTE;
            break;
        case COM_BLE_MSG_TYP_DIGEST_ERR:
            //------------------------------------------------------------------
            // ダイジェスト不一致
            //------------------------------------------------------------------
            // 受信ステータス
            e_rcv_sts = COM_BLE_MSG_RCV_PAIRING_ERR;
            // ユーザーイベント
            e_cb_evt = COM_BLE_MSG_EVT_PAIRING_ERR;
            break;
        case COM_BLE_MSG_TYP_STATUS_REQ:
            //------------------------------------------------------------------
            // ステータス要求
            //------------------------------------------------------------------
            // ステータスチェックトランザクション開始
            if (sts_begin_sts_chk() != ESP_OK) {
#ifdef COM_BLE_MSG_DEBUG
                ESP_LOGW(LOG_TAG, "%s L#%d Evt=COM_BLE_MSG_EVT_STATUS_ERR", __func__, __LINE__);
#endif
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_STS_CHK_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_STATUS_ERR;
                break;
            }
            // 受信したチェック乱数を保存
            memcpy(ps_sts_chk->u8_rx_rand, ps_rx_data->pu8_values, COM_MSG_SIZE_TICKET_STS);
#ifdef COM_BLE_MSG_DEBUG
            do {
                char sts_txt[(COM_MSG_SIZE_TICKET_STS * 2) + 1];
                v_vutil_u8_to_hex_string(ps_sts_chk->u8_rx_rand, COM_MSG_SIZE_TICKET_STS, sts_txt);
                ESP_LOGW(LOG_TAG, "%s L#%d rx_rand=%s", __func__, __LINE__, sts_txt);
            } while(false);
#endif
            // 応答メッセージを生成
            ps_tx_msg = ps_create_msg_data(COM_BLE_MSG_TYP_STATUS_RSP1, NULL);
            if (ps_tx_msg == NULL) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // ステータス応答１を返信する
            sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_tx_msg);
            // 送信済みデータを解放
            sts_mdl_delete_u8_array(ps_tx_msg);
            if (sts_val != ESP_OK) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_STATUS_CHK);
            break;
        case COM_BLE_MSG_TYP_STATUS_RSP1:
            //------------------------------------------------------------------
            // ステータス応答１
            //------------------------------------------------------------------
            // 受信したチェック乱数を保存
            memcpy(ps_sts_chk->u8_rx_rand, &ps_rx_data->pu8_values[MSG_SIZE_CHECK_CODE], MSG_SIZE_CHECK_RANDOM);
#ifdef COM_BLE_MSG_DEBUG
            do {
                char sts_txt[(COM_MSG_SIZE_TICKET_STS * 2) + 1];
                v_vutil_u8_to_hex_string(ps_sts_chk->u8_rx_rand, COM_MSG_SIZE_TICKET_STS, sts_txt);
                ESP_LOGW(LOG_TAG, "%s L#%d rx_rand=%s", __func__, __LINE__, sts_txt);
            } while(false);
#endif
            // 応答メッセージを生成
            ps_tx_msg = ps_create_msg_data(COM_BLE_MSG_TYP_STATUS_RSP2, NULL);
            if (ps_tx_msg == NULL) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // ステータス応答２を返信する
            sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_tx_msg);
            // 送信済みデータを解放
            sts_mdl_delete_u8_array(ps_tx_msg);
            if (sts_val != ESP_OK) {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
                break;
            }
            // チェックコードの検証
            if (sts_status_check(ps_rx_msg->u64_device_id, ps_rx_data->pu8_values, ps_ticket) == ESP_OK) {
                // イベントエンキュー
                v_msg_evt_enqueue(COM_BLE_MSG_EVT_STATUS_OK);
            } else {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_STS_CHK_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_STATUS_ERR;
            }
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        case COM_BLE_MSG_TYP_STATUS_RSP2:
            //------------------------------------------------------------------
            // ステータス応答２
            //------------------------------------------------------------------
            // チェックコードの検証
            if (sts_status_check(ps_rx_msg->u64_device_id, ps_rx_data->pu8_values, ps_ticket) == ESP_OK) {
                // イベントエンキュー
                v_msg_evt_enqueue(COM_BLE_MSG_EVT_STATUS_OK);
            } else {
                // 受信ステータス
                e_rcv_sts = COM_BLE_MSG_RCV_STS_CHK_ERR;
                // ユーザーイベント
                e_cb_evt = COM_BLE_MSG_EVT_STATUS_ERR;
            }
            // トランザクション終了
            v_msg_ctrl_sts_transaction_reset();
            break;
        case COM_BLE_MSG_TYP_DATA:
            //------------------------------------------------------------------
            // データ
            //------------------------------------------------------------------
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_RX_DATA);
            break;
        case COM_BLE_MSG_TYP_CIPHERTEXT:
            //------------------------------------------------------------------
            // 暗号データ
            //------------------------------------------------------------------
            // イベントエンキュー
            v_msg_evt_enqueue(COM_BLE_MSG_EVT_RX_CIPHERTEXT);
            break;
        default:
            // 受信ステータス
            e_rcv_sts = COM_BLE_MSG_RCV_HANDLING_ERR;
            // ユーザーイベント
            e_cb_evt = COM_BLE_MSG_EVT_HANDLING_ERR;
        }
    } while(false);

    //==========================================================================
    // 終了処理
    //==========================================================================

    //--------------------------------------------------------------------------
    // 受信応答処理
    //--------------------------------------------------------------------------
    // メッセージ定義の取得
    const ts_msg_definition_t* ps_rx_def = &MSG_DEF[ps_rx_msg->e_type];
    // 受信通知判定
    if (ps_rx_def->b_response) {
        // 受信応答が必要なメッセージの場合
        // 受信応答を送信
        sts_tx_response(ps_rx_msg->e_type, e_rcv_sts, ps_rx_msg->u32_seq_no);
    }

    //--------------------------------------------------------------------------
    // 受信ステータス処理
    //--------------------------------------------------------------------------
    if (e_rcv_sts != COM_BLE_MSG_RCV_NORMAL) {
        // 受信エラーの場合
        // ユーザーイベント判定
        if (e_cb_evt != COM_BLE_MSG_EVT_COUNT) {
            // イベントエンキュー
            v_msg_evt_enqueue(e_cb_evt);
        }
        // コネクションリセット
        if (e_rcv_sts != COM_BLE_MSG_RCV_STS_CHK_ERR) {
            v_msg_ctrl_sts_connection_reset();
        }
        // 異常終了
        return e_rcv_sts;
    }

    //--------------------------------------------------------------------------
    // 正常受信処理
    //--------------------------------------------------------------------------
    // 受信カウンタの更新
    s_msg_ctrl_sts.u64_rx_count++;
    // 受信履歴の更新
    ts_msg_history_t* ps_bef_rx_msg = NULL;
    if (ps_rx_msg->e_type != COM_BLE_MSG_TYP_RESPONSE) {
        // 受信メッセージ
        ps_bef_rx_msg = &s_msg_ctrl_sts.s_bef_rx_msg;
    } else {
        // 受信応答メッセージ
        ps_bef_rx_msg = &s_msg_ctrl_sts.s_bef_rx_rsp;
    }
    v_com_ble_addr_cpy(ps_bef_rx_msg->t_bda, ps_rx_msg->t_rcv_bda);
    ps_bef_rx_msg->u32_tick_ms    = ps_rx_msg->u32_rcv_tick_ms;
    ps_bef_rx_msg->u64_device_id  = ps_rx_msg->u64_device_id;
    ps_bef_rx_msg->u32_seq_no     = ps_rx_msg->u32_seq_no;
    ps_bef_rx_msg->e_type         = ps_rx_msg->e_type;

    // 結果ステータスを返信
    return e_rcv_sts;
}

/*******************************************************************************
 *
 * NAME: sts_tx_response
 *
 * DESCRIPTION:BLEの受信応答の送信処理
 *
 * PARAMETERS:              Name            RW  Usage
 * te_com_ble_msg_type_t    e_rx_type       R   受信メッセージタイプ
 * te_com_ble_msg_rcv_sts_t e_rx_sts        R   受信ステータス
 * uint32_t                 u32_seq_no      R   メッセージSEQ番号
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_tx_response(te_com_ble_msg_type_t e_rx_type,
                                  te_com_ble_msg_rcv_sts_t e_rx_sts,
                                  uint32_t u32_seq_no) {
    //==========================================================================
    // 受信通知メッセージ生成
    //==========================================================================
#ifdef COM_BLE_MSG_DEBUG
    ESP_LOGW(LOG_TAG, "%s L#%d rcv_type=%d rx_sts=%02x seq=%ld", __func__, __LINE__, e_rx_type, e_rx_sts, u32_seq_no);
#endif
    // メッセージ定義の取得
    const ts_msg_definition_t* ps_def = &MSG_DEF[COM_BLE_MSG_TYP_RESPONSE];
    // メッセージデータ
    ts_u8_array_t* ps_msg = ps_mdl_empty_u8_array(ps_def->u16_length);
    if (ps_msg == NULL) {
        return ESP_ERR_NO_MEM;
    }

    //--------------------------------------------------------------------------
    // ヘッダー編集
    //--------------------------------------------------------------------------
    // 編集：デバイスID
    uint8_t* pu8_value = ps_msg->pu8_values;
    tu_type_converter_t u_conv;
    u_conv.u64_value = s_msg_ctrl_cfg.u64_device_id;
    pu8_value[MSG_POS_DEVICE_ID]     = u_conv.u8_values[0];
    pu8_value[MSG_POS_DEVICE_ID + 1] = u_conv.u8_values[1];
    pu8_value[MSG_POS_DEVICE_ID + 2] = u_conv.u8_values[2];
    pu8_value[MSG_POS_DEVICE_ID + 3] = u_conv.u8_values[3];
    pu8_value[MSG_POS_DEVICE_ID + 4] = u_conv.u8_values[4];
    pu8_value[MSG_POS_DEVICE_ID + 5] = u_conv.u8_values[5];
    pu8_value[MSG_POS_DEVICE_ID + 6] = u_conv.u8_values[6];
    pu8_value[MSG_POS_DEVICE_ID + 7] = u_conv.u8_values[7];
    // 編集：メッセージタイプ
    pu8_value[MSG_POS_TYPE] = COM_BLE_MSG_TYP_RESPONSE;
    // 編集：メッセージ長
    u_conv.u16_values[0] = ps_def->u16_length;
    pu8_value[MSG_POS_MSG_LEN]     = u_conv.u8_values[0];
    pu8_value[MSG_POS_MSG_LEN + 1] = u_conv.u8_values[1];
    // 編集：シーケンス番号
    u_conv.u32_values[0] = u32_seq_no;
    pu8_value[MSG_POS_SEQ_NO]     = u_conv.u8_values[0];
    pu8_value[MSG_POS_SEQ_NO + 1] = u_conv.u8_values[1];
    pu8_value[MSG_POS_SEQ_NO + 2] = u_conv.u8_values[2];
    pu8_value[MSG_POS_SEQ_NO + 3] = u_conv.u8_values[3];

    //--------------------------------------------------------------------------
    // 本文編集
    //--------------------------------------------------------------------------
    // 受信メッセージタイプ
    pu8_value[MSG_POS_BODY] = e_rx_type;
    // 受信ステータス
    pu8_value[MSG_POS_BODY + 1] = e_rx_sts;

    //--------------------------------------------------------------------------
    // フッター編集
    //--------------------------------------------------------------------------
    // 乱数
    b_vutil_set_u8_rand_array(&pu8_value[MSG_POS_BODY + 2], MSG_SIZE_RANDOM);
    // ストップトークン
    u_conv.u16_values[0] = u32_seq_no;
    pu8_value[ps_def->u16_length - 2] = u_conv.u8_values[0];
    pu8_value[ps_def->u16_length - 1] = u_conv.u8_values[1];

    //==========================================================================
    // レスポンス送信処理
    //==========================================================================
    // レスポンスを返信
    esp_err_t sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg);
    // レスポンスデータクリア
    sts_mdl_delete_u8_array(ps_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_tx_reset_msg
 *
 * DESCRIPTION:BLE Tx reset message
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * esp_err_t:送信結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_tx_reset_msg() {
    //----------------------------------------------------------------------
    // 接続判定
    //----------------------------------------------------------------------
    if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
        // 未接続
        return ESP_ERR_INVALID_STATE;
    }

    //----------------------------------------------------------------------
    // メッセージ生成
    //----------------------------------------------------------------------
    ts_u8_array_t* ps_msg = ps_create_msg_data(COM_BLE_MSG_TYP_RESET, NULL);
    if (ps_msg == NULL) {
        return ESP_ERR_NO_MEM;
    }

    //----------------------------------------------------------------------
    // メッセージの送信
    //----------------------------------------------------------------------
    esp_err_t sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg);

    //----------------------------------------------------------------------
    // 送信メッセージ解放
    //----------------------------------------------------------------------
    sts_mdl_delete_u8_array(ps_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_tx_ping_msg
 *
 * DESCRIPTION:BLE Tx ping message
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * esp_err_t:送信結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_tx_ping_msg() {
    //----------------------------------------------------------------------
    // 接続判定
    //----------------------------------------------------------------------
    if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
        // 未接続
        return  ESP_ERR_INVALID_STATE;
    }

    //----------------------------------------------------------------------
    // メッセージ生成
    //----------------------------------------------------------------------
    ts_u8_array_t* ps_msg = ps_create_msg_data(COM_BLE_MSG_TYP_PING, NULL);
    if (ps_msg == NULL) {
        return  ESP_ERR_NO_MEM;
    }

    //----------------------------------------------------------------------
    // メッセージの送信
    //----------------------------------------------------------------------
    esp_err_t sts_val = s_msg_ctrl_cfg.pf_tx_msg(ps_msg);

    //----------------------------------------------------------------------
    // 送信メッセージ解放
    //----------------------------------------------------------------------
    sts_mdl_delete_u8_array(ps_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_upd_tx_history
 *
 * DESCRIPTION:送信履歴の更新処理
 *
 * PARAMETERS:      Name        RW  Usage
 * ts_u8_array_t*   ps_msg      R   送信メッセージ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_upd_tx_history(ts_u8_array_t* ps_msg) {
    //==========================================================================
    // 送信カウンタの更新
    //==========================================================================
    s_msg_ctrl_sts.u64_tx_count++;

    //==========================================================================
    // 送信履歴の更新
    //==========================================================================
    uint8_t* pu8_value = ps_msg->pu8_values;
    if (pu8_value[MSG_POS_TYPE] == COM_BLE_MSG_TYP_RESPONSE) {
        // 受信通知の場合は履歴を更新しない
        return;
    }

    //==========================================================================
    // 送信履歴の更新
    //==========================================================================
    // 受信通知以外の場合　※送信履歴のBLEアドレスは使用しない
    ts_msg_history_t* ps_bef_tx_msg = &s_msg_ctrl_sts.s_bef_tx_msg;
    // デバイスID
    tu_type_converter_t u_conv;
    u_conv.u8_values[0] = pu8_value[MSG_POS_DEVICE_ID];
    u_conv.u8_values[1] = pu8_value[MSG_POS_DEVICE_ID + 1];
    u_conv.u8_values[2] = pu8_value[MSG_POS_DEVICE_ID + 2];
    u_conv.u8_values[3] = pu8_value[MSG_POS_DEVICE_ID + 3];
    u_conv.u8_values[4] = pu8_value[MSG_POS_DEVICE_ID + 4];
    u_conv.u8_values[5] = pu8_value[MSG_POS_DEVICE_ID + 5];
    u_conv.u8_values[6] = pu8_value[MSG_POS_DEVICE_ID + 6];
    u_conv.u8_values[7] = pu8_value[MSG_POS_DEVICE_ID + 7];
    ps_bef_tx_msg->u64_device_id = u_conv.u64_value;
    // シーケンス番号
    u_conv.u8_values[0] = pu8_value[MSG_POS_SEQ_NO];
    u_conv.u8_values[1] = pu8_value[MSG_POS_SEQ_NO + 1];
    u_conv.u8_values[2] = pu8_value[MSG_POS_SEQ_NO + 2];
    u_conv.u8_values[3] = pu8_value[MSG_POS_SEQ_NO + 3];
    ps_bef_tx_msg->u32_seq_no = u_conv.u32_values[0];
    // 送信時間
    ps_bef_tx_msg->u32_tick_ms = xTaskGetTickCountMSec();
    // タイプ
    ps_bef_tx_msg->e_type = pu8_value[MSG_POS_TYPE];
}

/*******************************************************************************
 *
 * NAME: b_is_public_key_received
 *
 * DESCRIPTION:Public key received check
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   true:公開鍵受信済み
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_is_public_key_received() {
    //----------------------------------------------------------------------
    // 公開鍵の受信済み判定
    //----------------------------------------------------------------------
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // X25519コンテキスト
    ts_crypto_x25519_context_t* ps_x25519_ctx = ps_pairing->ps_x25519_ctx;
    if (ps_x25519_ctx == NULL) {
        return false;
    }
    // 公開鍵（サーバー）
    uint8_t* pu8_svr_key = ps_x25519_ctx->u8_svr_public_key;
    if (pu8_svr_key[0] != 0x20) {
        return false;
    }
    // 公開鍵（クライアント）
    uint8_t* pu8_cli_key = ps_x25519_ctx->u8_cli_public_key;
    tu_type_converter_t u_type_conv;
    u_type_conv.u8_values[0] = pu8_cli_key[0];
    u_type_conv.u8_values[1] = pu8_cli_key[1];
    u_type_conv.u8_values[2] = pu8_cli_key[2];
    u_type_conv.u8_values[3] = pu8_cli_key[3];
    // 公開鍵受信済み
    return (u_type_conv.u32_values[0] == 0x201D0003);
}

/*******************************************************************************
 *
 * NAME: b_is_paired
 *
 * DESCRIPTION:pairing check
 *
 * PARAMETERS:  Name            RW  Usage
 * uint64_t     u64_device_id   R   対象デバイスID
 *
 * RETURNS:
 *   true:ペアリング済み
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_is_paired(uint64_t u64_device_id) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // リモートデバイスの有無を判定
    if (u64_device_id == s_msg_ctrl_cfg.u64_device_id) {
        return false;
    }

    //==========================================================================
    // チケット取得
    //==========================================================================
    // チケット取得
    ts_com_msg_auth_ticket_t s_ticket;
    ts_com_msg_auth_ticket_t* ps_ticket = ps_read_ticket(u64_device_id, &s_ticket);
    // チケットの有効判定
    if (ps_ticket == NULL) {
        return false;
    }
    // シーケンス番号の有効判定
    if (ps_ticket->u32_tx_seq_no >= ps_ticket->u32_max_seq_no) {
        return false;
    }
    // 結果返信
    return true;
}

/*******************************************************************************
 *
 * NAME: sts_begin_open
 *
 * DESCRIPTION:オープントランザクションの開始
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_bd_addr_t    t_bda           R   接続先BLEアドレス
 * uint32_t*        pu32_timeout_ms W   トランザクションタイムアウトの編集先
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_begin_open(esp_bd_addr_t t_bda, uint32_t* pu32_timeout_ms) {
    //==========================================================================
    // トランザクション開始チェック
    //==========================================================================
    // 現在の接続先を判定
    if (l_com_ble_addr_cmp(t_bda, s_msg_ctrl_sts.t_rmt_bda) == 0) {
        // 既に接続済み
        return ESP_ERR_INVALID_STATE;
    }
    // トランザクション状態チェック
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    if (ps_tran->e_sts != COM_BLE_MSG_TRN_NONE) {
        // トランザクション実行中
        return ESP_ERR_INVALID_STATE;
    }
    // 接続状態チェック
    if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_DISCONNECTED) {
        // 接続中もしくは接続済み
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // オープントランザクションの開始処理
    //==========================================================================
    // オープントランザクション開始
    ps_tran->e_sts = COM_BLE_MSG_TRN_OPEN;
    // 相手デバイスIDクリア
    ps_tran->u64_device_id = s_msg_ctrl_cfg.u64_device_id;
    // BLEのアドレスコピー
    v_com_ble_addr_cpy(ps_tran->t_bda, t_bda);
    // トランザクションタイムアウト
    *pu32_timeout_ms = xTaskGetTickCountMSec() + COM_MSG_TRN_TIMEOUT_MS_OPEN;
    ps_tran->u32_timeout_ms = *pu32_timeout_ms;

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_begin_pairing
 *
 * DESCRIPTION:ペアリングトランザクションの開始
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_begin_pairing() {
    //==========================================================================
    // トランザクション開始チェック
    //==========================================================================
    //--------------------------------------------------------------------------
    // ペアリング機能の有効チェック
    //--------------------------------------------------------------------------
    if ((s_msg_ctrl_cfg.s_func_ctl & MSG_FUNC_CTL_PAIRING) == 0x00) {
        return ESP_ERR_INVALID_STATE;
    }

    //--------------------------------------------------------------------------
    // トランザクション状態チェック
    //--------------------------------------------------------------------------
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    if (ps_tran->e_sts != COM_BLE_MSG_TRN_NONE) {
        // トランザクション実行中
        return ESP_ERR_INVALID_STATE;
    }

    //--------------------------------------------------------------------------
    // 接続チェック
    //--------------------------------------------------------------------------
    if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
        // 未接続
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // ペアリングトランザクションの開始処理
    //==========================================================================
    //--------------------------------------------------------------------------
    // トランザクション情報を編集
    //--------------------------------------------------------------------------
    // ペアリングトランザクション開始
    ps_tran->e_sts = COM_BLE_MSG_TRN_PAIRING;
    // 相手デバイスID
    ps_tran->u64_device_id = s_msg_ctrl_sts.u64_rmt_device_id;
    // BLEのアドレスコピー
    v_com_ble_addr_cpy(ps_tran->t_bda, s_msg_ctrl_sts.t_rmt_bda);
    // トランザクションタイムアウト
    ps_tran->u32_timeout_ms = xTaskGetTickCountMSec() + COM_MSG_TRN_TIMEOUT_MS_PAIRING;

    //--------------------------------------------------------------------------
    // ペアリングステータスを編集
    //--------------------------------------------------------------------------
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // ペアリングステータス
    ps_pairing->e_sts = MSG_PAIRING_CHK_NONE;
    // 共通鍵
    memset(ps_pairing->u8_com_key, 0x00, COM_MSG_SIZE_CIPHER_KEY);
    // X25519コンテキストを削除
    v_crypto_x25519_delete_context(ps_pairing->ps_x25519_ctx);
    ps_pairing->ps_x25519_ctx = NULL;
    // 自デバイスステータス
    b_vutil_set_u8_rand_array(ps_pairing->u8_dev_status, COM_MSG_SIZE_TICKET_STS);
    // 相手デバイスステータスハッシュ
    memset(ps_pairing->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 最大シーケンス番号
    ps_pairing->u32_max_seq_no = 0;
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_begin_sts_chk
 *
 * DESCRIPTION:ステータスチェックトランザクションの開始
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_begin_sts_chk() {
    //==========================================================================
    // トランザクション開始チェック
    //==========================================================================
    //--------------------------------------------------------------------------
    // 機能の有効判定
    //--------------------------------------------------------------------------
    if ((s_msg_ctrl_cfg.s_func_ctl & MSG_FUNC_CTL_STS_CHK) == 0x00) {
        return ESP_ERR_INVALID_STATE;
    }

    //--------------------------------------------------------------------------
    // トランザクション状態チェック
    //--------------------------------------------------------------------------
    ts_transaction_info_t* ps_tran = &s_msg_ctrl_sts.s_tran;
    if (ps_tran->e_sts != COM_BLE_MSG_TRN_NONE) {
        // トランザクション実行中
        return ESP_ERR_INVALID_STATE;
    }

    //--------------------------------------------------------------------------
    // ペアリング済み判定
    //--------------------------------------------------------------------------
    if (!b_is_paired(s_msg_ctrl_sts.u64_rmt_device_id)) {
        return ESP_ERR_INVALID_STATE;
    }

    //--------------------------------------------------------------------------
    // 接続判定
    //--------------------------------------------------------------------------
    if (s_msg_ctrl_cfg.pf_connect_sts() != COM_BLE_MSG_CON_CONNECTED) {
#ifdef COM_BLE_MSG_DEBUG
        ESP_LOGE(LOG_TAG, "%s L#%d Err=ESP_ERR_INVALID_STATE", __func__, __LINE__);
#endif
        // 未接続
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // ステータスチェックトランザクションの開始処理
    //==========================================================================
    // ステータスチェックトランザクション開始
    ps_tran->e_sts = COM_BLE_MSG_TRN_STS_CHK;
    // 相手デバイスID
    ps_tran->u64_device_id = s_msg_ctrl_sts.u64_rmt_device_id;
    // BLEのアドレスコピー
    v_com_ble_addr_cpy(ps_tran->t_bda, s_msg_ctrl_sts.t_rmt_bda);
    // トランザクションタイムアウト
    ps_tran->u32_timeout_ms = xTaskGetTickCountMSec() + COM_MSG_TRN_TIMEOUT_MS_STS_CHK;

    //--------------------------------------------------------------------------
    // ステータスチェックを初期化
    //--------------------------------------------------------------------------
    ts_sts_check_info_t* ps_sts_chk = &s_msg_ctrl_sts.s_sts_chk;
    // 送信ステータスチェック乱数
    memset(ps_sts_chk->u8_tx_rand, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 受信ステータスチェック乱数
    memset(ps_sts_chk->u8_rx_rand, 0x00, COM_MSG_SIZE_TICKET_STS);

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_status_check
 *
 * DESCRIPTION:ステータスチェック
 *
 * PARAMETERS:                  Name            RW  Usage
 * uint64_t                     u64_device_id   R   デバイスID
 * uint8_t*                     pu8_chk_code    R   チェックコード
 * ts_com_msg_auth_ticket_t*    ps_ticket       R   チケット
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_status_check(uint64_t u64_device_id, uint8_t* pu8_chk_code, ts_com_msg_auth_ticket_t* ps_ticket) {
    // 自デバイスの１次ステータスハッシュ生成
    ts_u8_array_t* ps_own_sts = ps_mdl_create_u8_array(ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS);
    if (ps_own_sts == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint8_t u8_own_hash[COM_MSG_SIZE_TICKET_STS];
    esp_err_t sts_val = sts_crypto_sha256(ps_own_sts, COM_MSG_AUTH_STRETCHING, u8_own_hash);
    sts_mdl_delete_u8_array(ps_own_sts);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // １次ステータスハッシュに送信したチェック乱数を排他的論理和
    ts_sts_check_info_t* ps_sts_chk = &s_msg_ctrl_sts.s_sts_chk;
    uint8_t* pu8_rand = ps_sts_chk->u8_tx_rand;
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < COM_MSG_SIZE_TICKET_STS; u8_idx++) {
        u8_own_hash[u8_idx] ^= pu8_rand[u8_idx];
    }
    // ２次ステータスコードハッシュを生成
    ts_u8_array_t* ps_own_hash = ps_mdl_create_u8_array(u8_own_hash, COM_MSG_SIZE_TICKET_STS);
    if (ps_own_sts == NULL) {
        return ESP_ERR_NO_MEM;
    }
    uint8_t u8_chk_code[COM_MSG_SIZE_TICKET_STS];
    sts_val = sts_crypto_sha256(ps_own_hash, COM_MSG_AUTH_STRETCHING, u8_chk_code);
    sts_mdl_delete_u8_array(ps_own_hash);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
#ifdef COM_BLE_MSG_DEBUG
    char sts_txt[(COM_MSG_SIZE_TICKET_STS * 2) + 1];
    v_vutil_u8_to_hex_string(ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d tkt sts_code=%s", __func__, __LINE__, sts_txt);
    v_vutil_u8_to_hex_string(ps_sts_chk->u8_tx_rand, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d tkt sts_rand=%s", __func__, __LINE__, sts_txt);
    v_vutil_u8_to_hex_string(u8_chk_code, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d tkt sts_hash=%s", __func__, __LINE__, sts_txt);
    v_vutil_u8_to_hex_string(pu8_chk_code, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d rmt sts_hash=%s", __func__, __LINE__, sts_txt);
#endif
    // チェックコードを比較
    if (memcmp(pu8_chk_code, u8_chk_code, COM_MSG_SIZE_TICKET_STS) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // チェックOK
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_encryption
 *
 * DESCRIPTION:メッセージの暗号化
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_u8_array_t*       ps_msg          RW  メッセージ
 * uint16_t             u16_data_len    R   データ長
 * uint8_t*             pu8_key*        R   共通鍵
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_encryption(ts_u8_array_t* ps_msg,
                                     uint16_t u16_data_len,
                                     uint8_t* pu8_key) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // キーセット
    ts_crypto_keyset_t* ps_keyset = NULL;
    // 認証タグ
    ts_u8_array_t* ps_auth_tag = NULL;
    // 平文データ
    ts_u8_array_t* ps_plane = NULL;
    // 暗号データ
    ts_u8_array_t* ps_cipher = NULL;
    // 型変換
    tu_type_converter_t u_conv;

    //==========================================================================
    // 主処理
    //==========================================================================
    do {
        // メッセージ
        uint8_t* pu8_msg = ps_msg->pu8_values;
        // IVの生成
        b_vutil_set_u8_rand_array(&pu8_msg[MSG_POS_CIPHER_IV], MSG_SIZE_CIPHER_IV);
        // IVの取得
        ts_msg_edit_iv_t s_edit_iv;
        u_conv.u8_values[0] = pu8_msg[MSG_POS_SEQ_NO];
        u_conv.u8_values[1] = pu8_msg[MSG_POS_SEQ_NO + 1];
        u_conv.u8_values[2] = pu8_msg[MSG_POS_SEQ_NO + 2];
        u_conv.u8_values[3] = pu8_msg[MSG_POS_SEQ_NO + 3];
        s_edit_iv.u32_seq_no = u_conv.u32_values[0];
        memcpy(s_edit_iv.u8_iv, &pu8_msg[MSG_POS_CIPHER_IV], MSG_SIZE_CIPHER_IV);
        // キーの生成
        ps_keyset = ps_crypto_create_keyset();
        if (ps_keyset == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
        // 共通鍵
        ps_keyset->ps_key = ps_mdl_create_u8_array(pu8_key, COM_MSG_SIZE_CIPHER_KEY);
        if (ps_keyset->ps_key == NULL) {
            break;
        }
        // 初期ベクトル
        ps_keyset->ps_key_iv = ps_mdl_create_u8_array((uint8_t*)&s_edit_iv, IV_BYTES);
        if (ps_keyset->ps_key_iv == NULL) {
            break;
        }
        // ナンス
        ps_keyset->ps_nonce = NULL;
        // 認証タグ初期ベクトル
        ps_keyset->ps_auth_iv  = ps_mdl_empty_u8_array(MSG_SIZE_CIPHER_TAG);
        if (ps_keyset->ps_auth_iv == NULL) {
            break;
        }
        // 認証タグ
        ps_auth_tag = ps_mdl_empty_u8_array(MSG_SIZE_CIPHER_TAG);
        if (ps_auth_tag == NULL) {
            break;
        }
        // 平文の取得
        ps_plane = ps_mdl_create_u8_array(&pu8_msg[MSG_POS_CIPHER_DATA], u16_data_len);
        if (ps_plane == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
#ifdef COM_BLE_MSG_DEBUG
        do {
            char c_txt_str[(ps_plane->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_plane->pu8_values, ps_plane->t_size, c_txt_str);
            ESP_LOGW(LOG_TAG, "%s L#%d plane=%s", __func__, __LINE__, c_txt_str);
        } while(false);
#endif
        // 暗号(AES GCMモード)の生成
        ps_cipher = ps_crypto_aes_gcm_enc(ps_keyset, ps_plane, ps_auth_tag);
        if (ps_cipher == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
#ifdef COM_BLE_MSG_DEBUG
        do {
            char c_key_str[(COM_MSG_SIZE_CIPHER_KEY * 2) + 1];
            v_vutil_u8_to_hex_string(pu8_key, COM_MSG_SIZE_CIPHER_KEY, c_key_str);
            ESP_LOGW(LOG_TAG, "%s L#%d key=%s", __func__, __LINE__, c_key_str);
            char c_cipher_str[(ps_cipher->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_cipher->pu8_values, ps_cipher->t_size, c_cipher_str);
            ESP_LOGW(LOG_TAG, "%s L#%d cipher=%s", __func__, __LINE__, c_cipher_str);
            char c_tag_str[(ps_auth_tag->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_auth_tag->pu8_values, ps_auth_tag->t_size, c_tag_str);
            ESP_LOGW(LOG_TAG, "%s L#%d tag=%s", __func__, __LINE__, c_tag_str);
        } while(false);
#endif
        // 認証タグを編集
        memcpy(&pu8_msg[MSG_POS_CIPHER_TAG], ps_auth_tag->pu8_values, ps_auth_tag->t_size);
        // 平文を暗号に更新
        memcpy(&pu8_msg[MSG_POS_CIPHER_DATA], ps_cipher->pu8_values, ps_cipher->t_size);
    } while(false);

    //==========================================================================
    // 後処理
    //==========================================================================
    // 暗号文の解放
    sts_mdl_delete_u8_array(ps_cipher);
    // 平文の解放
    sts_mdl_delete_u8_array(ps_plane);
    // 認証タグの解放
    sts_mdl_delete_u8_array(ps_auth_tag);
    // キーの解放
    sts_crypto_delete_keyset(ps_keyset);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_msg_decryption
 *
 * DESCRIPTION:メッセージの復号
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_com_msg_t*        ps_rx_msg       RW  メッセージ
 * uint8_t*             pu8_key         R   共通鍵
 *
 * RETURNS:
 *   ts_u8_array_t*:正常終了時はプレーンテキスト、異常時はNULL
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_u8_array_t* ps_msg_decryption(ts_com_msg_t* ps_rx_msg, uint8_t* pu8_key) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    //--------------------------------------------------------------------------
    // 入力チェック
    //--------------------------------------------------------------------------
    // サイズチェック
    const size_t t_min_size = (MSG_SIZE_HEADER + MSG_SIZE_CIPHER_HEADER + MSG_SIZE_FOOTER);
    if (ps_rx_msg->u16_length <= t_min_size) {
        return NULL;
    }

    //--------------------------------------------------------------------------
    // 変数宣言
    //--------------------------------------------------------------------------
    // キーセット
    ts_crypto_keyset_t* ps_keyset = NULL;
    // 認証タグ
    ts_u8_array_t* ps_auth_tag = NULL;
    // 暗号データ
    ts_u8_array_t* ps_cipher = NULL;
    // 復号データ
    ts_u8_array_t* ps_plane  = NULL;

    //==========================================================================
    // 主処理
    //==========================================================================
    do {
        //----------------------------------------------------------------------
        // 復号処理
        //----------------------------------------------------------------------
        // キー情報の生成
        ps_keyset = ps_crypto_create_keyset();
        if (ps_keyset == NULL) {
            break;
        }
        // 共通鍵
        ps_keyset->ps_key = ps_mdl_create_u8_array(pu8_key, COM_MSG_SIZE_CIPHER_KEY);
        if (ps_keyset->ps_key == NULL) {
            break;
        }
        // 初期ベクトル
        ts_u8_array_t* ps_data = ps_rx_msg->ps_data;
        uint8_t* pu8_data = ps_data->pu8_values;
        ts_msg_edit_iv_t s_edit_iv;
        s_edit_iv.u32_seq_no = ps_rx_msg->u32_seq_no;
        memcpy(s_edit_iv.u8_iv, &pu8_data[MSG_SIZE_CIPHER_TAG], MSG_SIZE_CIPHER_IV);
        ps_keyset->ps_key_iv = ps_mdl_create_u8_array((uint8_t*)&s_edit_iv, IV_BYTES);
        if (ps_keyset->ps_key_iv == NULL) {
            break;
        }
        // ナンス
        ps_keyset->ps_nonce    = NULL;
        // 認証タグ初期ベクトル
        ps_keyset->ps_auth_iv  = ps_mdl_empty_u8_array(MSG_SIZE_CIPHER_TAG);
        if (ps_keyset->ps_auth_iv == NULL) {
            break;
        }
        // 認証タグ
        ps_auth_tag = ps_mdl_empty_u8_array(MSG_SIZE_CIPHER_TAG);
        if (ps_auth_tag == NULL) {
            break;
        }
        uint8_t* pu8_auth_tag = ps_auth_tag->pu8_values;
        memcpy(pu8_auth_tag, pu8_data, MSG_SIZE_CIPHER_TAG);
        // 暗号の取得
        uint32_t u32_data_len = ps_data->t_size - MSG_SIZE_CIPHER_HEADER;
        ps_cipher = ps_mdl_create_u8_array(&pu8_data[MSG_SIZE_CIPHER_HEADER], u32_data_len);
        if (ps_cipher == NULL) {
            break;
        }
        // 復号処理(AES GCMモード)
        ps_plane = ps_crypto_aes_gcm_dec(ps_keyset, ps_cipher, ps_auth_tag);
        if (ps_plane == NULL) {
            break;
        }
        // 認証タグの検証
        if (memcmp(ps_auth_tag->pu8_values, pu8_data, MSG_SIZE_CIPHER_TAG) != 0) {
            sts_mdl_delete_u8_array(ps_plane);
            ps_plane = NULL;
            break;
        }
#ifdef COM_BLE_MSG_DEBUG
        do {
            char c_key_str[(COM_MSG_SIZE_CIPHER_KEY * 2) + 1];
            v_vutil_u8_to_hex_string(pu8_key, COM_MSG_SIZE_CIPHER_KEY, c_key_str);
            ESP_LOGW(LOG_TAG, "%s L#%d key=%s", __func__, __LINE__, c_key_str);
            char c_cipher_str[(ps_cipher->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_cipher->pu8_values, ps_cipher->t_size, c_cipher_str);
            ESP_LOGW(LOG_TAG, "%s L#%d cipher=%s", __func__, __LINE__, c_cipher_str);
            char c_plane_str[(ps_plane->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_plane->pu8_values, ps_plane->t_size, c_plane_str);
            ESP_LOGW(LOG_TAG, "%s L#%d plane=%s", __func__, __LINE__, c_plane_str);
            char c_tag_str[(ps_auth_tag->t_size * 2) + 1];
            v_vutil_u8_to_hex_string(ps_auth_tag->pu8_values, ps_auth_tag->t_size, c_tag_str);
            ESP_LOGW(LOG_TAG, "%s L#%d tagA=%s", __func__, __LINE__, c_tag_str);
            v_vutil_u8_to_hex_string(pu8_data, ps_auth_tag->t_size, c_tag_str);
            ESP_LOGW(LOG_TAG, "%s L#%d tagB=%s", __func__, __LINE__, c_tag_str);
        } while(false);
#endif
    } while(false);

    //==========================================================================
    // 後処理
    //==========================================================================
    // 暗号の解放
    sts_mdl_delete_u8_array(ps_cipher);
    // 認証タグ
    sts_mdl_delete_u8_array(ps_auth_tag);
    // キーの解放
    sts_crypto_delete_keyset(ps_keyset);

    // 結果を返信
    return ps_plane;
}

/*******************************************************************************
 *
 * NAME: e_edit_rx_header
 *
 * DESCRIPTION:BLE edit Rx message header
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_t*                ps_rx_msg   W   編集対象のメッセージ
 * ts_com_ble_gatt_rx_data_t*   ps_rx_data  R   受信データ
 *
 * RETURNS:
 *   te_com_ble_msg_rcv_sts_t 受信ステータス
 *
 * NOTES:
 *   単項目チェック（定義ごとのチェックも含む）も実施
 * None.
 ******************************************************************************/
static te_com_ble_msg_rcv_sts_t e_edit_rx_header(ts_com_msg_t* ps_rx_msg, ts_com_ble_gatt_rx_data_t* ps_rx_data) {
    //==========================================================================
    // 受信データサイズチェック
    //==========================================================================
    // 受信データ
    ts_u8_array_t* ps_array = ps_rx_data->ps_array;
    if (ps_array == NULL) {
        // 受信データサイズエラー
        return COM_BLE_MSG_RCV_LENGTH_ERR;
    }
    if (ps_array->t_size < MSG_SIZE_HEADER) {
        // 受信データサイズエラー
        return COM_BLE_MSG_RCV_LENGTH_ERR;
    }

    //==========================================================================
    // 項目編集
    //==========================================================================
    // BLEアドレス
    v_com_ble_addr_cpy(ps_rx_msg->t_rcv_bda, ps_rx_data->t_bda);
    // 受信ティック（ミリ秒）
    ps_rx_msg->u32_rcv_tick_ms = xTaskGetTickCountMSec();
    // 送信デバイスID
    tu_type_converter_t u_conv;
    u_conv.u8_values[0] = ps_array->pu8_values[MSG_POS_DEVICE_ID];
    u_conv.u8_values[1] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 1];
    u_conv.u8_values[2] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 2];
    u_conv.u8_values[3] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 3];
    u_conv.u8_values[4] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 4];
    u_conv.u8_values[5] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 5];
    u_conv.u8_values[6] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 6];
    u_conv.u8_values[7] = ps_array->pu8_values[MSG_POS_DEVICE_ID + 7];
    ps_rx_msg->u64_device_id = u_conv.u64_value;
    // タイプ編集
    ps_rx_msg->e_type = ps_array->pu8_values[MSG_POS_TYPE];
    // メッセージ長
    u_conv.u8_values[0] = ps_array->pu8_values[MSG_POS_MSG_LEN];
    u_conv.u8_values[1] = ps_array->pu8_values[MSG_POS_MSG_LEN + 1];
    ps_rx_msg->u16_length = u_conv.u16_values[0];
    // シーケンス番号
    u_conv.u8_values[0] = ps_array->pu8_values[MSG_POS_SEQ_NO];
    u_conv.u8_values[1] = ps_array->pu8_values[MSG_POS_SEQ_NO + 1];
    u_conv.u8_values[2] = ps_array->pu8_values[MSG_POS_SEQ_NO + 2];
    u_conv.u8_values[3] = ps_array->pu8_values[MSG_POS_SEQ_NO + 3];
    ps_rx_msg->u32_seq_no = u_conv.u32_values[0];
    // 認証タグサイズ
    memcpy(ps_rx_msg->u8_auth_tag, &ps_array->pu8_values[MSG_POS_AUTH_TAG], COM_MSG_SIZE_AUTH_TAG);

    //==========================================================================
    // タイプチェック
    //==========================================================================
    if (ps_rx_msg->e_type >= COM_BLE_MSG_TYP_CNT) {
        // 受信タイプエラー
        return COM_BLE_MSG_RCV_TYPE_ERR;
    }

    //==========================================================================
    // メッセージ長チェック
    //==========================================================================
    // メッセージ定義の取得
    const ts_msg_definition_t* ps_rx_def = &MSG_DEF[ps_rx_msg->e_type];
    // 最大データ長チェック
    if (ps_rx_msg->u16_length > s_msg_ctrl_cfg.u32_max_length) {
        // 受信データサイズエラー
        return COM_BLE_MSG_RCV_LENGTH_ERR;
    }
    // 既定データ長チェック
    if (ps_rx_def->b_fixed_length) {
        // 固定長メッセージの場合
        if (ps_rx_msg->u16_length != ps_rx_def->u16_length) {
            // 受信データサイズエラー
            return COM_BLE_MSG_RCV_LENGTH_ERR;
        }
    } else {
        // 可変長メッセージの場合
        if (ps_rx_msg->u16_length < ps_rx_def->u16_length) {
            // 可変長メッセージで、最小メッセージ長より短い場合
            // 受信データサイズエラー
            return COM_BLE_MSG_RCV_LENGTH_ERR;
        }
    }
    // 受信データサイズが既にメッセージ長を超えている場合はエラー
    if (ps_rx_msg->u16_length < ps_array->t_size) {
        // 受信データサイズエラー
        return COM_BLE_MSG_RCV_LENGTH_ERR;
    }

    //==========================================================================
    // シーケンス番号チェック
    //==========================================================================
    // シーケンス番号チェック
    // 受信通知以外の場合
    if (ps_rx_def->b_fixed_seq && ps_rx_msg->u32_seq_no != ps_rx_def->u32_seq_no) {
        // SEQ番号が既定値では無い
        return COM_BLE_MSG_RCV_SEQ_ERR;
    }

    //==========================================================================
    // 結果返信
    //==========================================================================
    // 正常終了
    return COM_BLE_MSG_RCV_NORMAL;
}

/*******************************************************************************
 *
 * NAME: sts_edit_auth_tag
 *
 * DESCRIPTION:認証タグの編集処理
 *
 * PARAMETERS:      Name        RW  Usage
 * uint8_t*         pu8_tag     W   認証タグの編集先
 * ts_u8_array_t*   ps_msg      R   メッセージ
 *
 * RETURNS:
 *   esp_err_t 結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_edit_auth_tag(uint8_t* pu8_tag, ts_u8_array_t* ps_msg) {
    // メッセージヘッダー
    uint8_t* pu8_value = ps_msg->pu8_values;
    // 認証タグの退避
    uint8_t u8_origin_tag[COM_MSG_SIZE_AUTH_TAG];
    memcpy(u8_origin_tag, &pu8_value[MSG_POS_AUTH_TAG], COM_MSG_SIZE_AUTH_TAG);
    // 認証タグの初期化
    memset(&pu8_value[MSG_POS_AUTH_TAG], COM_MSG_AUTH_CHECK_VALUE, COM_MSG_SIZE_AUTH_TAG);
    // ハッシュ値の算出
    esp_err_t sts_val = sts_crypto_sha256(ps_msg, COM_MSG_AUTH_STRETCHING, pu8_tag);
    // 認証タグを元に戻す
    memcpy(&pu8_value[MSG_POS_AUTH_TAG], u8_origin_tag, COM_MSG_SIZE_AUTH_TAG);
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_edit_check_code
 *
 * DESCRIPTION:チェックコード編集
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット
 * uint8_t*                     pu8_rand    R   チェック乱数
 * uint8_t*                     pu8_digest  W   ダイジェスト
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_edit_check_code(ts_com_msg_auth_ticket_t* ps_ticket, uint8_t* pu8_rand, uint8_t* pu8_digest) {
    //==========================================================================
    // 相手デバイスのステータスハッシュ値にチェック乱数をXOR
    //==========================================================================
    uint8_t u8_token[COM_MSG_SIZE_TICKET_STS];
    memcpy(u8_token, ps_ticket->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS);
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < COM_MSG_SIZE_TICKET_STS; u8_idx++) {
        u8_token[u8_idx] ^= pu8_rand[u8_idx];
    }

    //==========================================================================
    // 再度ハッシュ関数を通す
    //==========================================================================
    ts_u8_array_t* ps_hash = ps_mdl_create_u8_array(u8_token, COM_MSG_SIZE_TICKET_STS);
    if (ps_hash == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // チェックコードを生成
    sts_crypto_sha256(ps_hash, COM_MSG_AUTH_STRETCHING, pu8_digest);
    sts_mdl_delete_u8_array(ps_hash);
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_create_msg_data
 *
 * DESCRIPTION:メッセージの生成
 *
 * PARAMETERS:              Name            RW  Usage
 * te_com_ble_msg_type_t    e_type          R   メッセージタイプ
 * ts_u8_array_t*           ps_data         R   本文データ
 *
 * RETURNS:
 * ts_u8_array_t*:メッセージデータ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_u8_array_t* ps_create_msg_data(te_com_ble_msg_type_t e_type,
                                          ts_u8_array_t* ps_data) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // メッセージ定義の取得
    const ts_msg_definition_t* ps_def = &MSG_DEF[e_type];
    // シーケンス番号の振出
    uint32_t u32_seq_no = ps_def->u32_seq_no;
    // チケット情報の取得
    ts_com_msg_auth_ticket_t* ps_ticket = NULL;
    if (ps_def->b_pairing && e_type != COM_BLE_MSG_TYP_RESPONSE) {
        // ペアリング必須
        // チケットの読み込み（送信シーケンスをカウントアップ）
        ps_ticket = ps_read_ticket(s_msg_ctrl_sts.u64_rmt_device_id, &s_msg_ctrl_sts.s_rmt_ticket);
        if (ps_ticket == NULL) {
            return NULL;
        }
        // チケットの有効期限チェック
        if (ps_ticket->u32_tx_seq_no >= ps_ticket->u32_max_seq_no) {
            return NULL;
        }
        // チケットを更新
        ps_ticket->u32_tx_seq_no++;
        if (s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_UPDATE, ps_ticket) != ESP_OK) {
            // チケット初期化
            v_init_ticket(ps_ticket);
            // 結果返信
            return NULL;
        }
        // 送信シーケンス番号
        u32_seq_no = ps_ticket->u32_tx_seq_no;
    }

    //==========================================================================
    // メッセージ生成
    //==========================================================================
    //--------------------------------------------------------------------------
    // メッセージ長
    //--------------------------------------------------------------------------
    // メッセージ長を取得
    uint32_t u32_msg_len  = ps_def->u16_length;
    // 本文長
    uint32_t u32_body_len = ps_def->u16_body_length;
    // データ長
    uint32_t u32_data_len = 0;
    // 可変長判定
    if (!ps_def->b_fixed_length) {
        // 入力チェック
        if (ps_data != NULL) {
            u32_data_len = ps_data->t_size;
        }
        // 暗号判定
        if (!ps_def->b_encryption) {
            // 平文メッセージ
            u32_body_len = u32_data_len;
        } else {
            // 暗号メッセージ
            // 認証タグ、IV、パディングされたデータ長から本文全体のサイズを算出
            u32_data_len = u32_crypto_pkcs7_padded_length(u32_data_len, AES_BLOCK_BYTES);
            u32_body_len = MSG_SIZE_CIPHER_HEADER + u32_data_len;
        }
        // メッセージ長
        u32_msg_len = MSG_SIZE_HEADER + u32_body_len + MSG_SIZE_FOOTER;
    }
#ifdef COM_BLE_MSG_DEBUG
    ESP_LOGW(LOG_TAG, "%s L#%d MSG Type     = %d", __func__, __LINE__, e_type);
    ESP_LOGW(LOG_TAG, "%s L#%d u16_msg_len  = %ld", __func__, __LINE__, u32_msg_len);
    ESP_LOGW(LOG_TAG, "%s L#%d u16_body_len = %ld", __func__, __LINE__, u32_body_len);
#endif
    //--------------------------------------------------------------------------
    // メッセージ長チェック
    //--------------------------------------------------------------------------
    if (u32_msg_len > s_msg_ctrl_cfg.u32_max_length) {
        return NULL;
    }

    //--------------------------------------------------------------------------
    // メッセージ生成
    //--------------------------------------------------------------------------
    ts_u8_array_t* ps_msg = ps_mdl_empty_u8_array(u32_msg_len);
    if (ps_msg == NULL) {
        return NULL;
    }

    //==========================================================================
    // メッセージを編集
    //==========================================================================
    //--------------------------------------------------------------------------
    // ヘッダーの編集
    //--------------------------------------------------------------------------
    uint8_t* pu8_values = ps_msg->pu8_values;
    tu_type_converter_t u_conv;
    // デバイスID
    u_conv.u64_value = s_msg_ctrl_cfg.u64_device_id;
    pu8_values[MSG_POS_DEVICE_ID]     = u_conv.u8_values[0];
    pu8_values[MSG_POS_DEVICE_ID + 1] = u_conv.u8_values[1];
    pu8_values[MSG_POS_DEVICE_ID + 2] = u_conv.u8_values[2];
    pu8_values[MSG_POS_DEVICE_ID + 3] = u_conv.u8_values[3];
    pu8_values[MSG_POS_DEVICE_ID + 4] = u_conv.u8_values[4];
    pu8_values[MSG_POS_DEVICE_ID + 5] = u_conv.u8_values[5];
    pu8_values[MSG_POS_DEVICE_ID + 6] = u_conv.u8_values[6];
    pu8_values[MSG_POS_DEVICE_ID + 7] = u_conv.u8_values[7];
    // タイプ
    pu8_values[MSG_POS_TYPE] = e_type;
    // メッセージ長
    u_conv.u16_values[0] = u32_msg_len;
    pu8_values[MSG_POS_MSG_LEN]     = u_conv.u8_values[0];
    pu8_values[MSG_POS_MSG_LEN + 1] = u_conv.u8_values[1];
    // シーケンス番号
    u_conv.u32_values[0] = u32_seq_no;
    pu8_values[MSG_POS_SEQ_NO]     = u_conv.u8_values[0];
    pu8_values[MSG_POS_SEQ_NO + 1] = u_conv.u8_values[1];
    pu8_values[MSG_POS_SEQ_NO + 2] = u_conv.u8_values[2];
    pu8_values[MSG_POS_SEQ_NO + 3] = u_conv.u8_values[3];

    //--------------------------------------------------------------------------
    // フッターの編集
    //--------------------------------------------------------------------------
    // メッセージ長を取得
    uint16_t u16_footer_pos = MSG_SIZE_HEADER + u32_body_len;
    // フッター乱数
    b_vutil_set_u8_rand_array(&pu8_values[u16_footer_pos], MSG_SIZE_RANDOM);
    // ストップトークン
    u_conv.u16_values[0] = u32_seq_no;
    pu8_values[u32_msg_len - 2] = u_conv.u8_values[0];
    pu8_values[u32_msg_len - 1] = u_conv.u8_values[1];

    //--------------------------------------------------------------------------
    // 本文の編集
    //--------------------------------------------------------------------------
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ペアリング情報
    ts_pairing_info_t* ps_pairing = &s_msg_ctrl_sts.s_pairing;
    // ステータス情報
    ts_sts_check_info_t* ps_sts_chk = &s_msg_ctrl_sts.s_sts_chk;
    // X25519コンテキスト
    ts_crypto_x25519_context_t* ps_x25519_ctx = ps_pairing->ps_x25519_ctx;
    // ステータス
    ts_u8_array_t* ps_status = NULL;
    // 本文（ダイジェスト一致）
    ts_msg_digest_match_t* ps_digest_match;
    // データタイプ判定
    switch (e_type) {
    case COM_BLE_MSG_TYP_RESPONSE:
        // 受信応答　※受信応答のメッセージは応答送信処理で生成する
        break;
    case COM_BLE_MSG_TYP_RESET:
        // リセット
        break;
    case COM_BLE_MSG_TYP_PING:
        // PING
        break;
    case COM_BLE_MSG_TYP_PAIRING_REQ:
        // ペアリング要求
        // X25519チェック
        if (ps_x25519_ctx == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 公開鍵
        memcpy(&pu8_values[MSG_POS_BODY], &ps_x25519_ctx->u8_cli_public_key[4], CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE - 4);
        break;
    case COM_BLE_MSG_TYP_PAIRING_RSP:
        // ペアリング応答
        // X25519チェック
        if (ps_x25519_ctx == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 公開鍵
        memcpy(&pu8_values[MSG_POS_BODY], &ps_x25519_ctx->u8_svr_public_key[1], CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE - 1);
        break;
    case COM_BLE_MSG_TYP_DIGEST_MATCH:
        // ダイジェスト一致
        // ステータス生成
        ps_status = ps_mdl_create_u8_array(ps_pairing->u8_dev_status, COM_MSG_SIZE_TICKET_STS);
        if (ps_status == NULL) {
            sts_val = ESP_ERR_NO_MEM;
            break;
        }
        // ステータスハッシュ生成
        ps_digest_match = (ts_msg_digest_match_t*)&pu8_values[MSG_POS_CIPHER_DATA];
        sts_val = sts_crypto_sha256(ps_status, COM_MSG_AUTH_STRETCHING, ps_digest_match->u8_sts_hash);
        // ステータスの解放
        sts_mdl_delete_u8_array(ps_status);
        ps_status = NULL;
        // ステータス判定
        if (sts_val != ESP_OK) {
            break;
        }
        // 本文：最大送信SEQ番号
        ps_digest_match->u32_max_seq_no = ps_pairing->u32_max_seq_no;
        // ステータスハッシュ以降を暗号化
        sts_val = sts_msg_encryption(ps_msg, MSG_SIZE_DIGEST_MATCH_DATA, ps_pairing->u8_com_key);
        break;
    case COM_BLE_MSG_TYP_DIGEST_ERR:
        // ダイジェスト不一致
        break;
    case COM_BLE_MSG_TYP_STATUS_REQ:
        // ステータス要求
        // チェック乱数
        b_vutil_set_u8_rand_array(ps_sts_chk->u8_tx_rand, COM_MSG_SIZE_TICKET_STS);
        memcpy(&pu8_values[MSG_POS_BODY], ps_sts_chk->u8_tx_rand, COM_MSG_SIZE_TICKET_STS);
        break;
    case COM_BLE_MSG_TYP_STATUS_RSP1:
        // ステータス応答１
        // チェックコード
        sts_val = sts_edit_check_code(ps_ticket, ps_sts_chk->u8_rx_rand, &pu8_values[MSG_POS_BODY]);
        if (sts_val != ESP_OK) {
            break;
        }
        // チェック乱数
        b_vutil_set_u8_rand_array(ps_sts_chk->u8_tx_rand, COM_MSG_SIZE_TICKET_STS);
        memcpy(&pu8_values[MSG_POS_STS_RSP1_RND], ps_sts_chk->u8_tx_rand, COM_MSG_SIZE_TICKET_STS);
       break;
    case COM_BLE_MSG_TYP_STATUS_RSP2:
        // ステータス応答２
        // チェックコード
        sts_val = sts_edit_check_code(ps_ticket, ps_sts_chk->u8_rx_rand, &pu8_values[MSG_POS_BODY]);
        if (sts_val != ESP_OK) {
            break;
        }
        break;
    case COM_BLE_MSG_TYP_DATA:
        // データ
        // データ書き込み
        memcpy(&ps_msg->pu8_values[MSG_POS_PLANIN_DATA], ps_data->pu8_values, ps_data->t_size);
        break;
    case COM_BLE_MSG_TYP_CIPHERTEXT:
        // 暗号データ
        // IVを生成
        b_vutil_set_u8_rand_array(&pu8_values[MSG_POS_CIPHER_IV], MSG_SIZE_CIPHER_IV);
        // パディング済みデータ書き込み
        sts_val = sts_crypto_pkcs7_padding(&ps_msg->pu8_values[MSG_POS_CIPHER_DATA], ps_data, AES_BLOCK_BYTES);
        if (sts_val != ESP_OK) {
            break;
        }
        // 本文の暗号化
        sts_val = sts_msg_encryption(ps_msg, u32_data_len, ps_ticket->u8_enc_key);
        break;
    default:
        break;
    }
    // エラー判定
    if (sts_val != ESP_OK) {
        // メッセージクリア
        sts_mdl_delete_u8_array(ps_msg);
        // 結果返信
        return NULL;
    }

    // 結果返信
    return ps_msg;
}

/*******************************************************************************
 *
 * NAME: initialize ticket
 *
 * DESCRIPTION:チケット初期化処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   W   対象チケット
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_init_ticket(ts_com_msg_auth_ticket_t* ps_ticket) {
    // 自デバイスID
    ps_ticket->u64_own_device_id = s_msg_ctrl_cfg.u64_device_id;
    // 相手デバイスID
    ps_ticket->u64_rmt_device_id = s_msg_ctrl_cfg.u64_device_id;
    // 暗号鍵
    memset(ps_ticket->u8_enc_key, 0x00, COM_MSG_SIZE_CIPHER_KEY);
    // 自ステータス
    memset(ps_ticket->u8_own_sts, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 相手ステータスハッシュ
    memset(ps_ticket->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
    ps_ticket->u32_max_seq_no = 0;      // 最大シーケンス番号
    ps_ticket->u32_tx_seq_no  = 0;      // 送信シーケンス番号
    ps_ticket->u32_rx_seq_no  = 0;      // 受信シーケンス番号
}

/*******************************************************************************
 *
 * NAME: sts_create_ticket
 *
 * DESCRIPTION:チケット生成処理
 *
 * PARAMETERS:              Name            RW  Usage
 * ts_transaction_info_t*   ps_tran         R   トランザクション情報
 * ts_pairing_info_t*       ps_pairing      R   ペアリング情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_create_ticket(ts_transaction_info_t* ps_tran, ts_pairing_info_t* ps_pairing) {
    // チケット作成
    ts_com_msg_auth_ticket_t* ps_ticket = &s_msg_ctrl_sts.s_rmt_ticket;
    // 自デバイスID
    ps_ticket->u64_own_device_id = s_msg_ctrl_cfg.u64_device_id;
    // 相手デバイスID
    ps_ticket->u64_rmt_device_id = ps_tran->u64_device_id;
    // 暗号鍵
    memcpy(ps_ticket->u8_enc_key, ps_pairing->u8_com_key, COM_MSG_SIZE_CIPHER_KEY);
    // 自ステータス
    memcpy(ps_ticket->u8_own_sts, ps_pairing->u8_dev_status, COM_MSG_SIZE_TICKET_STS);
    // 相手ステータスハッシュ
    memcpy(ps_ticket->u8_rmt_sts_hash, ps_pairing->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS);
    // 最大シーケンス番号
    ps_ticket->u32_max_seq_no = ps_pairing->u32_max_seq_no;
    // 送信シーケンス番号
    ps_ticket->u32_tx_seq_no = 0;
    // 受信シーケンス番号
    ps_ticket->u32_rx_seq_no = 0;
#ifdef COM_BLE_MSG_DEBUG
    char sts_txt[(COM_MSG_SIZE_TICKET_STS * 2) + 1];
    v_vutil_u8_to_hex_string(ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d own_code=%s", __func__, __LINE__, sts_txt);
    v_vutil_u8_to_hex_string(ps_ticket->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d rmt_hash=%s", __func__, __LINE__, sts_txt);
    ESP_LOGW(LOG_TAG, "%s L#%d seq_no=%ld", __func__, __LINE__, ps_ticket->u32_max_seq_no);
#endif
    // チケットの書き込み
    esp_err_t sts_val = s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_CREATE, ps_ticket);
    if (sts_val != ESP_OK) {
        // チケット初期化
        v_init_ticket(ps_ticket);
    }
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_read_ticket
 *
 * DESCRIPTION:read ticket
 *
 * PARAMETERS:                  Name            RW  Usage
 * uint64_t                     u64_device_id   R   対象のデバイスID
 * ts_com_msg_auth_ticket_t*    ps_cache_ticket W   キャッシュ用チケット
 *
 * RETURNS:
 *   ts_com_msg_auth_ticket_t* 読み込んだチケット
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_msg_auth_ticket_t* ps_read_ticket(uint64_t u64_device_id,
                                                 ts_com_msg_auth_ticket_t* ps_cache_ticket) {
    // 自デバイスIDチェック
    if (s_msg_ctrl_cfg.u64_device_id == u64_device_id) {
        return NULL;
    }
    // チケットキャッシュの判定
    ts_com_msg_auth_ticket_t* ps_ticket = &s_msg_ctrl_sts.s_rmt_ticket;
    if (ps_ticket->u64_rmt_device_id == u64_device_id) {
        return ps_ticket;
    }
    // チケットキャッシュと一致しない場合
    ps_ticket = ps_cache_ticket;
    // 自デバイスID
    ps_ticket->u64_own_device_id = s_msg_ctrl_cfg.u64_device_id;
    // 相手デバイスID
    ps_ticket->u64_rmt_device_id = u64_device_id;
    // チケット読み込み
    if (s_msg_ctrl_cfg.pf_tkt_cb(COM_BLE_MSG_TICKET_EVT_READ, ps_ticket) != ESP_OK) {
        // チケット初期化
        v_init_ticket(ps_ticket);
        // 読み込みエラー
        return NULL;
    }
    // 結果返信
    return ps_ticket;
}

/*******************************************************************************
 *
 * NAME: t_gatt_if_svr
 *
 * DESCRIPTION:BLE GATT Interface (Server Side)
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_gatt_if_t:GATTインターフェース
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatt_if_t t_gatt_if_svr() {
    // GATtインターフェース取得処理
    esp_gatt_if_t t_gatt_if = t_com_ble_gatts_if(s_msg_ctrl_cfg.u16_app_id);
    if (t_gatt_if != ESP_GATT_IF_NONE) {
        // GATTインターフェースが取得済みの場合には取得関数を切り替え
        s_msg_ctrl_sts.t_gatt_if  = t_gatt_if;
        s_msg_ctrl_cfg.pf_gatt_if = t_gatt_if_default;
    }
    return t_gatt_if;
}

/*******************************************************************************
 *
 * NAME: t_gatt_if_cli
 *
 * DESCRIPTION:BLE GATT Interface (Client Side)
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_gatt_if_t:GATTインターフェース
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatt_if_t t_gatt_if_cli() {
    // GATtインターフェース取得処理
    esp_gatt_if_t t_gatt_if = t_com_ble_gattc_if(s_msg_ctrl_cfg.u16_app_id);
    if (t_gatt_if != ESP_GATT_IF_NONE) {
        // GATTインターフェースが取得済みの場合には取得関数を切り替え
        s_msg_ctrl_sts.t_gatt_if  = t_gatt_if;
        s_msg_ctrl_cfg.pf_gatt_if = t_gatt_if_default;
    }
    return t_gatt_if;
}

/*******************************************************************************
 *
 * NAME: t_gatt_if_default
 *
 * DESCRIPTION:デフォルト関数：GATTインターフェース取得関数
 *
 * PARAMETERS:                  Name        RW  Usage
 *
 * RETURNS:
 *   esp_gatt_if_t:GATTインターフェース
 * NOTES:
 * None.
 ******************************************************************************/
static esp_gatt_if_t t_gatt_if_default() {
    return s_msg_ctrl_sts.t_gatt_if;
}

/*******************************************************************************
 *
 * NAME: e_connect_sts_svr
 *
 * DESCRIPTION:BLE Connection status (Server Side)
 *
 * PARAMETERS:              Name        RW  Usage
 *
 * RETURNS:
 *   te_com_ble_msg_connection_sts_t:接続状態を表すステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_connection_sts_t e_connect_sts_svr() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return COM_BLE_MSG_CON_ERROR;
    }

    //==========================================================================
    // 接続ステータス判定
    //==========================================================================
    te_com_ble_msg_connection_sts_t e_con_sts = COM_BLE_MSG_CON_ERROR;
    do {
        //----------------------------------------------------------------------
        // GAPステータス判定
        //----------------------------------------------------------------------
        te_gap_dev_sts_t e_gap_sts = e_com_ble_gap_adv_device_status();
        if (e_gap_sts != GAP_DEV_STS_DEVICE_NONE) {
            if ((e_gap_sts & MSG_GAP_CHK_PASSKEY) == GAP_DEV_STS_REQ_PASSKEY) {
                e_con_sts = COM_BLE_MSG_CON_WAIT_PASSKEY;
                break;
            }
            if ((e_gap_sts & MSG_GAP_CHK_NUM_CHK) == GAP_DEV_STS_REQ_NUM_CHK) {
                e_con_sts = COM_BLE_MSG_CON_WAIT_NUM_CHK;
                break;
            }
        }

        //----------------------------------------------------------------------
        // GATTステータス判定
        //----------------------------------------------------------------------
        // 接続状況を返却
        if (!b_com_ble_gatts_is_connected(s_msg_ctrl_cfg.pf_gatt_if())) {
            // 未接続
            e_con_sts = COM_BLE_MSG_CON_DISCONNECTED;
            break;
        }
        // リモートアドレスを判定
        if (b_com_ble_addr_clear(s_msg_ctrl_sts.t_rmt_bda)) {
            // 接続中
            e_con_sts = COM_BLE_MSG_CON_CONNECTING;
            break;
        }
        // 接続済み
        e_con_sts = COM_BLE_MSG_CON_CONNECTED;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return e_con_sts;
}

/*******************************************************************************
 *
 * NAME: e_connect_sts_cli
 *
 * DESCRIPTION:BLE Connection status (Client Side)
 *
 * PARAMETERS:              Name        RW  Usage
 *
 * RETURNS:
 *   te_com_ble_msg_connection_sts_t:接続状態を表すステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_connection_sts_t e_connect_sts_cli() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex_sts, portMAX_DELAY) != pdTRUE) {
        return COM_BLE_MSG_CON_ERROR;
    }

    //==========================================================================
    // 接続ステータス判定
    //==========================================================================
    te_com_ble_msg_connection_sts_t e_con_sts = COM_BLE_MSG_CON_ERROR;
    do {
        //----------------------------------------------------------------------
        // 判定用のBLEアドレス
        //----------------------------------------------------------------------
        esp_bd_addr_t t_rmt_bda;
        // トランザクション種別
        if (s_msg_ctrl_sts.s_tran.e_sts == COM_BLE_MSG_TRN_NONE) {
            // 接続状況を判定
            if (b_com_ble_addr_clear(s_msg_ctrl_sts.t_rmt_bda)) {
                // 未接続
                e_con_sts = COM_BLE_MSG_CON_DISCONNECTED;
                break;
            }
            // 接続済みアドレス
            v_com_ble_addr_cpy(t_rmt_bda, s_msg_ctrl_sts.t_rmt_bda);
        } else {
            // 接続中アドレス
            v_com_ble_addr_cpy(t_rmt_bda, s_msg_ctrl_sts.s_tran.t_bda);
        }

        //----------------------------------------------------------------------
        // GAPステータスチェック
        //----------------------------------------------------------------------
        te_gap_dev_sts_t e_dev_sts = e_com_ble_gap_device_sts(t_rmt_bda);
        if (e_dev_sts == GAP_DEV_STS_DEVICE_NONE) {
            // デバイスステータスが無いので未接続
            e_con_sts = COM_BLE_MSG_CON_DISCONNECTED;
            break;
        }
        if ((e_dev_sts & MSG_GAP_CHK_PASSKEY) == GAP_DEV_STS_REQ_PASSKEY) {
            e_con_sts = COM_BLE_MSG_CON_WAIT_PASSKEY;
            break;
        }
        if ((e_dev_sts & MSG_GAP_CHK_NUM_CHK) == GAP_DEV_STS_REQ_NUM_CHK) {
            e_con_sts = COM_BLE_MSG_CON_WAIT_NUM_CHK;
            break;
        }

        //----------------------------------------------------------------------
        // SPP接続判定
        //----------------------------------------------------------------------
        // コネクション判定
        ts_com_ble_gattc_con_info_t* ps_con = ps_get_connection();
        if (ps_con == NULL) {
            // 接続中
            e_con_sts = COM_BLE_MSG_CON_CONNECTING;
            break;
        }
        // 接続ステータスを取得
        te_com_ble_spp_connection_sts_t e_spp_con_sts = e_com_ble_sppc_con_sts(ps_con);
        // SPP接続判定：未接続
        if (e_spp_con_sts == COM_BLE_SPP_CON_DISCONNECTED) {
            // SPP未接続なので未接続
            e_con_sts = COM_BLE_MSG_CON_DISCONNECTED;
            break;
        }
        // SPP接続判定：接続中
        if (e_spp_con_sts == COM_BLE_SPP_CON_CONNECTING) {
            e_con_sts = COM_BLE_MSG_CON_CONNECTING;
            break;
        }
        // SPP接続判定：接続
        if (e_spp_con_sts == COM_BLE_SPP_CON_CONNECTED) {
            e_con_sts = COM_BLE_MSG_CON_CONNECTED;
            break;
        }
        // 接続エラー
        e_con_sts = COM_BLE_MSG_CON_ERROR;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex_sts);

    // 結果返信
    return e_con_sts;
}

/*******************************************************************************
 *
 * NAME: ps_ble_rx_data_svr
 *
 * DESCRIPTION:BLE Rx Data(Server side)
 *
 * PARAMETERS:              Name        RW  Usage
 * TickType_t               t_tick      R   ウェイト時間
 *
 * RETURNS:
 *   ts_com_ble_gatt_rx_data*:受信データ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_ble_gatt_rx_data_t* ps_ble_rx_data_svr(TickType_t t_tick) {
    // タイムアウト時刻
    TickType_t t_timeout = xTaskGetTickCount() + t_tick;
    // 受信データ取得
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    // データ受信ループ
    while(t_timeout >= xTaskGetTickCount()) {
        // データ受信
        ps_rx_data = ps_com_ble_spps_rx_data(s_msg_ctrl_cfg.pf_gatt_if(), COM_MSG_RETRY_WAIT);
        if (ps_rx_data == NULL) {
            continue;
        }
        // 受信データ判定
        if (ps_rx_data->u16_hndl_idx != SPPS_ATTR_IDX_RX_DATA_VAL) {
            // 対象データでは無いので解放
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
            ps_rx_data = NULL;
            continue;
        }
        // 正常受信
        break;
    }
    // 受信データ返却
    return ps_rx_data;
}

/*******************************************************************************
 *
 * NAME: ps_ble_rx_data_cli
 *
 * DESCRIPTION:BLE Rx Data(Client side)
 *
 * PARAMETERS:              Name        RW  Usage
 * TickType_t               t_tick      R   ウェイト時間
 *
 * RETURNS:
 *   ts_com_ble_gatt_rx_data*:受信データ
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_ble_gatt_rx_data_t* ps_ble_rx_data_cli(TickType_t t_tick) {
    // タイムアウト時刻
    TickType_t t_timeout = xTaskGetTickCount() + t_tick;
    // 接続の有無を判定
    ts_com_ble_gattc_con_info_t* ps_con_info = NULL;
    // 受信データ取得
    ts_com_ble_gatt_rx_data_t* ps_rx_data = NULL;
    // データ受信ループ
    while(t_timeout >= xTaskGetTickCount()) {
        // 接続の有無を判定
        ps_con_info = ps_get_connection();
        if (ps_con_info == NULL) {
            // ウェイトしてリトライ
            vTaskDelay(COM_MSG_RETRY_WAIT);
            continue;
        }
        // 受信データ取得処理
        ps_rx_data = ps_com_ble_sppc_rx_data(ps_con_info, COM_MSG_RETRY_WAIT);
        if (ps_rx_data == NULL) {
            continue;
        }
        // サーバーからの送信データ判定
        if (ps_rx_data->u16_hndl_idx != SPPS_ATTR_IDX_TX_DATA_VAL) {
            // サーバーからの送信データ以外は解放する
            v_com_ble_gatt_delete_rx_data(ps_rx_data);
            ps_rx_data = NULL;
            continue;
        }
        // 正常受信
        break;
    }
    // 受信データ取得処理
    return ps_rx_data;
}

/*******************************************************************************
 *
 * NAME: v_ble_rx_clear_svr
 *
 * DESCRIPTION:BLE Rx Data queue clear(Server side)
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_rx_clear_svr() {
    // 受信データバッファクリア
    tf_get_gatt_if_t pf_gatt_if = s_msg_ctrl_cfg.pf_gatt_if;
    v_com_ble_spps_rx_clear(pf_gatt_if());
}

/*******************************************************************************
 *
 * NAME: v_ble_rx_clear_cli
 *
 * DESCRIPTION:BLE Rx Data queue clear(Client side)
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_rx_clear_cli() {
    //==========================================================================
    // BLEクライアントの受信データキュークリア
    //==========================================================================
    // 接続の有無を判定
    ts_com_ble_gattc_con_info_t* ps_con = ps_get_connection();
    if (ps_con == NULL) {
        return;
    }
    // 受信データバッファクリア
    esp_gatt_id_t* ps_svc_id = &ps_con->ps_service[BLE_SPPS_SVC_IDX].s_svc_id;
    v_com_ble_gattc_rx_clear(ps_con->t_gatt_if, ps_con->u16_con_id, *ps_svc_id);
}

/*******************************************************************************
 *
 * NAME: v_ble_rx_through_svr
 *
 * DESCRIPTION:受信データキューのデータ読み飛ばし処理
 *
 * PARAMETERS:              Name        RW  Usage
 * size_t                   t_len       R   読み飛ばしデータ長
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_rx_through_svr(size_t t_len) {
    //==========================================================================
    // BLEサーバーの受信キュー読み飛ばし処理
    //==========================================================================
    // GATTインターフェース取得処理
    tf_get_gatt_if_t pf_gatt_if = s_msg_ctrl_cfg.pf_gatt_if;
    esp_gatt_if_t t_gatt_if = pf_gatt_if();
    // 受信データ
    ts_com_ble_gatt_rx_data_t* ps_rx_data;
    // 受信データ取得
    uint32_t u32_len = 0;
    while (u32_len < t_len) {
        // 受信データの読み出し
        ps_rx_data = ps_com_ble_spps_rx_data(t_gatt_if, 0);
        if (ps_rx_data == NULL) {
            break;
        }
        // データ長を更新
        u32_len += ps_rx_data->ps_array->t_size;
        // 受信データを解放
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
    }
    // 受信データ長
    if (u32_len > t_len) {
        // 受信データ長エラーの場合には受信データバッファクリア
        v_com_ble_spps_rx_clear(t_gatt_if);
    }
}

/*******************************************************************************
 *
 * NAME: v_ble_rx_through_cli
 *
 * DESCRIPTION:BLEのデータ読み飛ばし処理
 *
 * PARAMETERS:              Name        RW  Usage
 * size_t                   t_len       R   読み飛ばしデータ長
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_rx_through_cli(size_t t_len) {
    //==========================================================================
    // BLEクライアントの受信キュー読み飛ばし処理
    //==========================================================================
    // 接続の有無を判定
    ts_com_ble_gattc_con_info_t* ps_con = ps_get_connection();
    if (ps_con == NULL) {
        return;
    }
    // 受信データ取得処理
    ts_com_ble_gatt_rx_data_t* ps_rx_data;
    uint32_t u32_len = 0;
    while (u32_len < t_len) {
        ps_rx_data = ps_com_ble_sppc_rx_data(ps_con, 0);
        if (ps_rx_data == NULL) {
            break;
        }
        // データ長を更新
        u32_len += ps_rx_data->ps_array->t_size;
        // 受信データを解放
        v_com_ble_gatt_delete_rx_data(ps_rx_data);
    }
    // 受信データ長
    if (u32_len > t_len) {
        // 受信データ長エラーの場合には受信データバッファクリア
        v_com_ble_sppc_rx_clear(ps_con);
    }
}

/*******************************************************************************
 *
 * NAME: sts_ble_tx_msg_svr
 *
 * DESCRIPTION:BLE Tx Message(Server side)
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_u8_array_t*           ps_msg      R   送信メッセージ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_ble_tx_msg_svr(ts_u8_array_t* ps_msg) {
    //==========================================================================
    // 認証タグ編集
    //==========================================================================
    // 認証タグの生成
    uint8_t u8_auth_tag[COM_MSG_SIZE_AUTH_TAG];
    esp_err_t sts_val = sts_edit_auth_tag(u8_auth_tag, ps_msg);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // 認証タグを編集
    memcpy(&ps_msg->pu8_values[MSG_POS_AUTH_TAG], u8_auth_tag, COM_MSG_SIZE_AUTH_TAG);

    //==========================================================================
    // サーバーからの送信処理
    //==========================================================================
    // GATTインターフェース取得処理
    esp_gatt_if_t t_gatt_if = s_msg_ctrl_cfg.pf_gatt_if();
    // メッセージを送信
    sts_val = sts_com_ble_spps_tx_data(t_gatt_if, ps_msg->pu8_values, ps_msg->t_size);
    if (sts_val != ESP_OK) {
        // 結果返信
        return sts_val;
    }
#ifdef COM_BLE_MSG_DEBUG
    TickType_t t_now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGW(LOG_TAG, "%s tick=%ldms type=%d t_size=%d", __func__, t_now, ps_msg->pu8_values[MSG_POS_TYPE], ps_msg->t_size);
#endif

    //==========================================================================
    // 送信履歴の更新
    //==========================================================================
    v_upd_tx_history(ps_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_ble_tx_msg_cli
 *
 * DESCRIPTION:BLE Tx Message(Client side)
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_u8_array_t*           ps_msg      R   送信メッセージ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_ble_tx_msg_cli(ts_u8_array_t* ps_msg) {
    //==========================================================================
    // 認証タグ編集
    //==========================================================================
    // 認証タグの生成
    uint8_t u8_auth_tag[COM_MSG_SIZE_AUTH_TAG];
    esp_err_t sts_val = sts_edit_auth_tag(u8_auth_tag, ps_msg);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // 認証タグを編集
    memcpy(&ps_msg->pu8_values[MSG_POS_AUTH_TAG], u8_auth_tag, COM_MSG_SIZE_AUTH_TAG);

    //==========================================================================
    // クライアントからの送信処理
    //==========================================================================
    // 接続の有無を判定
    ts_com_ble_gattc_con_info_t* ps_con = ps_get_connection();
    if (ps_con == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    // メッセージを送信
    sts_val = sts_com_ble_sppc_tx_data(ps_con, ps_msg->pu8_values, ps_msg->t_size);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
#ifdef COM_BLE_MSG_DEBUG
    TickType_t t_now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGW(LOG_TAG, "%s tick=%ldms type=%d t_size=%d", __func__, t_now, ps_msg->pu8_values[MSG_POS_TYPE], ps_msg->t_size);
#endif

    //==========================================================================
    // 送信履歴の更新
    //==========================================================================
    v_upd_tx_history(ps_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_spp_evt_cb_svr
 *
 * DESCRIPTION:BLE SPP event callback (Server Side)
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
static void v_spp_evt_cb_svr(esp_gatts_cb_event_t e_event,
                              esp_gatt_if_t t_gatt_if,
                              esp_ble_gatts_cb_param_t* pu_param) {
    //==========================================================================
    // SPPクライアントユーザーイベント処理
    //==========================================================================
    // イベント処理
    switch (e_event) {
    case ESP_GATTS_CONNECT_EVT:
        // 接続通知
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_GATT_CONNECT);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        // 切断通知
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_GATT_DISCONNECT);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_spp_evt_cb_cli
 *
 * DESCRIPTION:BLE SPP event callback (Client Side)
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
static void v_spp_evt_cb_cli(esp_gattc_cb_event_t e_event,
                              esp_gatt_if_t t_gatt_if,
                              esp_ble_gattc_cb_param_t* pu_param) {
    //==========================================================================
    // SPPクライアントユーザーイベント処理
    //==========================================================================
    // イベント処理
    switch (e_event) {
    case ESP_GATTC_WRITE_DESCR_EVT:
        // GATTサービスへのDescriptor書き込み完了通知イベント
        // リンク成功イベント
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_LINK_SUCCESS);
        break;
    case ESP_GATTC_CONNECT_EVT:
        // 接続通知
        // 接続通知イベント
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_GATT_CONNECT);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        // 切断通知
        // 切断通知イベント
        v_msg_evt_enqueue(COM_BLE_MSG_EVT_GATT_DISCONNECT);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 *
 * NAME: e_msg_dmy_connect_sts
 *
 * DESCRIPTION:ダミー関数：接続ステータス取得
 *
 * PARAMETERS:                  Name        RW  Usage
 *
 * RETURNS:
 *   te_com_ble_msg_connection_sts_t:接続ステータス
 * NOTES:
 * None.
 ******************************************************************************/
static te_com_ble_msg_connection_sts_t e_msg_dmy_connect_sts() {
    return COM_BLE_MSG_CON_DISCONNECTED;
}

/*******************************************************************************
 *
 * NAME: ps_msg_dmy_rx_data
 *
 * DESCRIPTION:ダミー関数：データ受信関数
 *
 * PARAMETERS:                  Name        RW  Usage
 * TickType_t                   t_tick      R   待ち時間
 *
 * RETURNS:
 * ts_com_ble_gatt_rx_data*:受信データ
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_ble_gatt_rx_data_t* ps_msg_dmy_rx_data(TickType_t t_tick) {
    return NULL;
}

/*******************************************************************************
 *
 * NAME: sts_msg_dmy_tx_msg
 *
 * DESCRIPTION:ダミー関数：メッセージ送信関数
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_u8_array_t*           ps_msg      R   送信データ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_dmy_tx_msg(ts_u8_array_t* ps_msg) {
    return ESP_ERR_INVALID_STATE;
}

/*******************************************************************************
 *
 * NAME: v_msg_dmy_rx_clear
 *
 * DESCRIPTION:ダミー関数：受信データキュークリア
 *
 * PARAMETERS:          Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_dmy_rx_clear() {
    // 接続ステータスクリア
    return;
}

/*******************************************************************************
 *
 * NAME: v_msg_dmy_rx_through
 *
 * DESCRIPTION:ダミー関数：受信キューの読み飛ばし
 *
 * PARAMETERS:          Name        RW  Usage
 * size_t               t_len       R   読み飛ばしデータサイズ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_dmy_rx_through(size_t t_len) {
    return;
}

/*******************************************************************************
 *
 * NAME: v_msg_dmy_ticket_cb
 *
 * DESCRIPTION:ダミー関数：チケットアクセスコールバック関数
 *
 * PARAMETERS:                  Name        RW  Usage
 * te_com_ble_msg_ticket_evt_t  e_evt       R   イベント種別
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_dmy_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket) {
    return ESP_ERR_INVALID_STATE;
}

/*******************************************************************************
 *
 * NAME: v_msg_dmy_evt_cb
 *
 * DESCRIPTION:ダミー関数：メッセージイベントコールバック関数
 *
 * PARAMETERS:              Name        RW  Usage
 * te_com_ble_msg_event     e_msg_evt   R   コールバックイベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_dmy_evt_cb(te_com_ble_msg_event e_msg_evt) {
    return;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
