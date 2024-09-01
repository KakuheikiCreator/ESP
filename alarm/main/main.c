/*******************************************************************************
 *
 * MODULE :GATT Messager Library functions source file
 *
 * CREATED:2021/06/10 00:32:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:GATTメッセンジャーのテストコード
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
#include <stdio.h>
#include <string.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_bt.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>

#include "settings.h"
#include <ntfw_com_value_util.h>
#include <ntfw_com_mem_alloc.h>
#include <ntfw_com_data_model.h>
#include <ntfw_com_debug_util.h>
#include <ntfw_cryptography.h>
#include <ntfw_io_gpio_util.h>
#include <ntfw_io_file_util.h>
#include <ntfw_io_i2c_master.h>
#include <ntfw_io_gpio_util.h>
#include <ntfw_ble_fmwk.h>
#include <ntfw_ble_msg.h>
#include <ntfw_drv_st7032i.h>
#include <ntfw_drv_mpu-6050.h>


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** デバッグモード */
//#define DEBUG_ALARM

/** ログ出力タグ */
#define LOG_MSG_TAG "BLE_ALARM"

//==============================================================================
// ウォッチドックタイマー設定
//==============================================================================
/** タイムアウト：ウォッチドックタイマー */
#define TWDT_TIMEOUT_MSEC       (2000)

/*
 * Macro to check the outputs of TWDT functions and trigger an abort if an
 * incorrect code is returned.
 */
#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})

//==============================================================================
// デバイス設定
//==============================================================================
/** GPIO Value HIGH */
#define GPIO_HIGH   (0x01)
/** GPIO Value LOW */
#define GPIO_LOW    (0x00)

/** GPIO Pairing setting pin */
#define COM_GPIO_PAIRING_SETTING    (GPIO_NUM_23)
/** GPIO Port1 radar */
#define COM_GPIO_PORT1_RADAR        (GPIO_NUM_32)
/** GPIO Port1 motion */
#define COM_GPIO_PORT1_MOTION       (GPIO_NUM_33)
/** GPIO Port2 radar */
#define COM_GPIO_PORT2_RADAR        (GPIO_NUM_25)
/** GPIO Port2 motion */
#define COM_GPIO_PORT2_MOTION       (GPIO_NUM_27)

/** 5way switch ADC channel */
#define COM_5WAY_CHANNEL            (ADC_CHANNEL_6)

//==============================================================================
// 加速度センサー
//==============================================================================
/** acceleration threshold */
#define COM_AXES_THRESHOLD          (500)

/** sensor disable interval(msec) */
#define COM_SENSOR_DISABLE_INTERVAL (3000)

//==============================================================================
// LCD
//==============================================================================
/** LCDコントラスト */
#ifndef COM_LCD_CONTRAST
    #define COM_LCD_CONTRAST    (0x28)
#endif

/** LCD空行サイズ */
#define COM_LCD_LINE_SIZE       (16)

/** LCD空行 */
#define COM_LCD_EMPTY_LINE      "                "

//==============================================================================
// 各種ファイル設定値
//==============================================================================
/** microSDマウント先 */
#ifndef COM_MOUN_SDT
    #define COM_MOUNT_SD        "/sdcard"
#endif

/** 設定ファイルパス */
#ifndef COM_PATH_SETTING
    #define COM_PATH_SETTING    "/sdcard/setting.json"
#endif

/** メッセージファイルパス */
#ifndef COM_PATH_MESSAGE
    #define COM_PATH_MESSAGE    "/sdcard/message.json"
#endif

/** チケットファイルパス */
#ifndef COM_PATH_TICKET
    #define COM_PATH_TICKET     "/sdcard/ticket.json"
#endif


//==============================================================================
// BluetoothLE関連
//==============================================================================
/** パスキー */
#ifndef GAP_STATIC_PASSKEYT
    #define GAP_STATIC_PASSKEYT (123456)
#endif
/** BLE GATTアプリケーションID */
#define BLE_GATT_APP_ID         (0x2E11)


//==============================================================================
// MSG関連の定義
//==============================================================================
/** UARTタスクの優先度 */
#define BLE_MSG_UART_PRIORITIES (configMAX_PRIORITIES - 5)
/** メッセージデバイスID */
#define BLE_MSG_DEVICE_ID       (0x00000000000000F0)
/** 最大メッセージサイズ */
#define BLE_MSG_MAX_SIZE        (2048)
/** メッセージ最大シーケンスサイズ */
#ifndef BLE_MSG_MAX_SEQ_NO
    #define BLE_MSG_MAX_SEQ_NO  (0xFFFFFFFF)
#endif
/** 確認メッセージコードサイズ */
#define BLE_MSG_CODE_SIZE       (48)
/** 公開鍵サイズ（クライアント） */
#define BLE_MSG_PUBLIC_KEY_CLI_SIZE (36)
/** 公開鍵サイズ（サーバー） */
#define BLE_MSG_PUBLIC_KEY_SVR_SIZE (33)
/** メッセージチェックコードサイズ */
#define BLE_MSG_CHECK_CODE_SIZE     (32)
/** メッセージIDサイズ */
#define BLE_MSG_ID_SIZE         (6)
/** メッセージデータサイズ */
#define BLE_MSG_REC_SIZE        (2)

//==============================================================================
// チケット定義
//==============================================================================
// リモートデバイスアドレスサイズ(Base64)
#define COM_TICKET_DEV_BDA_BASE64_SIZE (8)
// リモートデバイスアドレスサイズ(uint8_t)
#define COM_TICKET_DEV_BDA_SIZE (6)
// チケットリスト
#define COM_TICKET_LIST         "ticket_list"
// 自デバイスID
#define COM_TICKET_OWN_DEV_ID   "own_device_id"
// リモートデバイスID
#define COM_TICKET_RMT_DEV_ID   "rmt_device_id"
// リモートデバイスアドレス
#define COM_TICKET_RMT_DEV_BDA  "rmt_device_bda"
// リモートデバイス名
#define COM_TICKET_RMT_DEV_NAME "rmt_device_name"
// 暗号鍵
#define COM_TICKET_ENC_KEY      "enc_key"
// 自ステータス
#define COM_TICKET_OWN_STS      "own_sts"
// 相手ステータスハッシュ
#define COM_TICKET_RMT_HASH     "rmt_sts_hash"
// 最大シーケンス番号
#define COM_TICKET_MAX_SEQ      "max_seq_no"
// 送信シーケンス番号
#define COM_TICKET_TX_SEQ       "tx_seq_no"
// 受信シーケンス番号
#define COM_TICKET_RX_SEQ       "rx_seq_no"

//==============================================================================
// タスク関係
//==============================================================================
/** タスク優先度（高） */
//#define TASK_PRIORITIES_HIGH    (configMAX_PRIORITIES - 5)
/** タスク優先度（中） */
#define TASK_PRIORITIES_MEDIUM  (configMAX_PRIORITIES - 10)
/** タスク優先度（低） */
#define TASK_PRIORITIES_LOW     (configMAX_PRIORITIES - 15)
/** 待ち時間：タスク終端 */
#define IDLE_TASK_WAIT_TICK     (500 / portTICK_PERIOD_MS)

//==============================================================================
// イベント関係
//==============================================================================
/** 待ち時間：クリティカルセクション */
#define EVT_TAKE_WAIT_TICK  (1000 / portTICK_PERIOD_MS)
/** 待ち時間：センサー起動待ち */
#define EVT_BOOT_WAIT_TICK   (20 / portTICK_PERIOD_MS)
/** 待ち時間：イベントエンキュー */
#define EVT_ENQUEUE_WAIT_TICK   (100 / portTICK_PERIOD_MS)
/** 待ち時間：受信イベント処理 */
#define EVT_RX_WAIT_TICK    (100 / portTICK_PERIOD_MS)
/** 待ち時間：ディスコネクトタイムアウト */
#define EVT_DISCONNECT_TIMEOUT  (500 / portTICK_PERIOD_MS)
/** 待ち時間：接続タイムアウト（ミリ秒） */
#define EVT_CONNECTION_TIMEOUT_MS  (5000)
/** 待ち時間：ペアリングタイムアウト（ミリ秒） */
#define EVT_PAIRING_TIMEOUT_MS  (90000)
/** イベントキューサイズ */
#define EVT_QUEUE_SIZE  (32)

/** デバイスステータスマスク：センサー検知 */
#define DEV_STS_MASK_SENSOR (DEV_STS_PAIRING_ENABLED | DEV_STS_ACCELERATION | DEV_STS_ACCELERATION_ALARM | DEV_STS_PORT1_RADAR | DEV_STS_PORT1_MOTION | DEV_STS_PORT2_RADAR | DEV_STS_PORT2_MOTION)
/** デバイスステータスマスク：警報時センサー検知 */
#define DEV_STS_MASK_ALARM (DEV_STS_PAIRING_ENABLED | DEV_STS_ACCELERATION)
/** デバイスステータスマスク：警報判定 */
#define DEV_STS_MASK_ALARM_CHECK (DEV_STS_REMOTE_STS_ERR | DEV_STS_ACCELERATION_ALARM | DEV_STS_PORT1_RADAR | DEV_STS_PORT1_MOTION | DEV_STS_PORT2_RADAR | DEV_STS_PORT2_MOTION)


//==============================================================================
// 警報機関係の定義
//==============================================================================
/** タイマータスクのウェイト時間 */
#define COM_TIMER_TASK_WAIT_MSEC    (30)
/** タイマータスクの警報間隔 */
#define COM_ALARM_INTERVAL_MSEC     (500)

//==============================================================================
// メッセージID
//==============================================================================
// メッセージID：起動エラー
#define COM_MSG_ID_ERR_BOOT         "E0000"
// メッセージID：スキャンタイムアウトエラー
#define COM_MSG_ID_ERR_SCAN_TIMEOUT "E0001"
// メッセージID：接続エラー
#define COM_MSG_ID_ERR_CONNECT      "E0002"
// メッセージID：ペアリングエラー
#define COM_MSG_ID_ERR_PAIRING      "E0003"
// メッセージID：リモートチケットエラー
#define COM_MSG_ID_ERR_RMT_TICKET   "E0004"
// メッセージID：ステータスチェックエラー
#define COM_MSG_ID_ERR_STATUS_CHK   "E0005"
// メッセージID：送受信エラー
#define COM_MSG_ID_ERR_TXRX         "E0006"
// メッセージID：接続タイムアウトエラー
#define COM_MSG_ID_ERR_TIMEOUT      "E0007"
// メッセージID：警報
#define COM_MSG_ID_ERR_ALARM        "E0008"

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * キー入力電圧閾値
 */
typedef enum {
    VOLTAGE_THRESHOID_NONE  = 2900,
    VOLTAGE_THRESHOID_PUSH  = 2400,
    VOLTAGE_THRESHOID_LEFT  = 1800,
    VOLTAGE_THRESHOID_UP    = 1100,
    VOLTAGE_THRESHOID_RIGHT = 460,
    VOLTAGE_THRESHOID_DOWN  = 71,
} te_input_voltage_threshold_t;

/**
 * デバイスステータス
 */
typedef enum {
    DEV_STS_REMOTE_STS_ERR      = 0x0001,     // 通信：リモートステータスエラー
    DEV_STS_CONTROLLER_LINK     = 0x0002,     // 通信：コントローラーリンク
    DEV_STS_PAIRING_ENABLED     = 0x0004,     // センサー：ペアリング有効
    DEV_STS_ACCELERATION        = 0x0008,     // センサー：加速度センサー検知
    DEV_STS_ACCELERATION_ALARM  = 0x0010,     // センサー：加速度センサー検知（警報値）
    DEV_STS_PORT1_RADAR         = 0x0020,     // センサー：モジュール１のレーダー検知
    DEV_STS_PORT1_MOTION        = 0x0040,     // センサー：モジュール１の動体検知
    DEV_STS_PORT2_RADAR         = 0x0080,     // センサー：モジュール２のレーダー検知
    DEV_STS_PORT2_MOTION        = 0x0100      // センサー：モジュール２の動体検知
} te_device_status_t;

/**
 * 接続ステータス
 */
typedef enum {
    CON_STS_DISCONNECTED = 0,   // 未接続
    CON_STS_CONNECTING,         // 接続中
    CON_STS_PAIRING,            // ペアリング中
    CON_STS_CHECK,              // ステータスチェック中
    CON_STS_CONNECTED,          // 接続済み
} te_connection_sts_t;

/**
 * 画面ID定義
 */
typedef enum {
    SCR_ID_MSG_DISPLAY = 0,     // メッセージ表示
    SCR_ID_STATUS_DISPLAY,      // ステータス表示
    SCR_ID_TICKET_DELETE,       // チケット削除
    SCR_ID_PAIRING_CHECK,       // ペアリングチェック
    SCR_ID_COUNT,               // 画面数
} te_usr_screen_id_t;

/**
 * カーソルタイプ
 */
typedef enum {
    CURSOR_TYPE_NONE = 0,       // 非表示
    CURSOR_TYPE_DISPLAY,        // 表示
    CURSOR_TYPE_WAIT,           // 実行中
} te_usr_cursor_type_t;

/**
 * 制御コマンド
 */
typedef enum {
    CTL_CMD_ACK = 0x00,         // 応答
    CTL_CMD_NACK,               // 否定応答
    CTL_CMD_READ,               // 読み込み
    CTL_CMD_UPDATE,             // 更新
    CTL_CMD_UNPAIR,             // ペアリング解除
    CTL_CMD_COUNT,              // 制御コマンド数
} te_msg_ctrl_cmd_t;

/**
 * 動作モード
 */
typedef enum {
    OPR_MODE_NORMAL = 0x00,     // 通常モード
    OPR_MODE_ALERT,             // 警戒モード
    OPR_MODE_ALARM,             // 警報モード
    OPR_MODE_COUNT,             // 動作モード数
} te_msg_operating_mode_t;

/**
 * ユーザーイベント定義
 */
typedef enum {
    EVT_SCR_INIT = 0,           // 画面：初期処理
    EVT_BLE_CONNECT,            // BLE：接続通知
    EVT_BLE_CONNECT_ERROR,      // BLE：接続エラー
    EVT_BLE_DISCONNECT,         // BLE：切断通知
    EVT_MSG_CONNECT,            // MSG：メッセージ機能接続
    EVT_MSG_PAIR_CD_CHK,        // MSG：ペアリング確認用コードチェック
    EVT_MSG_PAIR_OK,            // MSG：ペアリング成功
    EVT_MSG_PAIR_ERROR,         // MSG：ペアリングエラー
    EVT_MSG_STS_OK,             // MSG：ステータス正常
    EVT_MSG_STS_ERROR,          // MSG：ステータスエラー
    EVT_MSG_RX_DATA,            // MSG：制御データ受信
    EVT_MSG_RX_ERROR,           // MSG：制御データ受信エラー
    EVT_TIMEOUT,                // タイムアウト
    EVT_SENSOR_UPDATE,          // センサーステータス更新
    EVT_SENSOR_ERROR,           // センサーステータスエラー
    EVT_INPUT_UP,               // キー入力：上
    EVT_INPUT_DOWN,             // キー入力：下
    EVT_INPUT_LEFT,             // キー入力：左
    EVT_INPUT_RIGHT,            // キー入力：右
    EVT_INPUT_PUSH,             // キー入力：プッシュ
    EVT_COUNT,                  // イベント数
} te_usr_event_t;

/**
 * LCDステータス
 */
typedef struct {
    te_usr_cursor_type_t e_cursor_type;     // カーソルタイプ
    uint8_t u8_cursor_row;                  // カーソル位置（行）
    uint8_t u8_cursor_col;                  // カーソル位置（列）
    char c_buff[2][COM_LCD_LINE_SIZE + 1];  // スクリーンバッファ
} ts_lcd_sts_t;

/**
 * 加速度センサーステータス
 */
typedef struct {
    ts_i2c_address_t s_address;                 // I2Cアドレス
} ts_accelerometer_sts_t;

/**
 * SDカードステータス
 */
typedef struct {
    esp_vfs_fat_sdmmc_mount_config_t s_mnt_cfg;
    sdmmc_card_t* ps_card;
} ts_sd_sts_t;

/**
 * デバイス設定
 */
typedef struct {
    uint64_t u64_device_id;         // デバイスID
    char c_device_name[17];         // デバイス名
} ts_device_settings_t;

/**
 * 表示メッセージ情報
 */
typedef struct s_msg_info_t {
    char c_msg_id[BLE_MSG_ID_SIZE];     // メッセージID
    char c_msg[17];                     // メッセージ
    struct s_msg_info_t* ps_next;       // 次のメッセージ
} ts_msg_info_t;

/**
 * チケットノード
 */
typedef struct s_ticket_node_t {
    esp_bd_addr_t t_rmt_device_bda;                     // BLEアドレス
    ts_com_msg_auth_ticket_t s_ticket;                  // チケット
    struct s_ticket_node_t* ps_next;                    // 次のチケット
} ts_ticket_node_t;

/**
 * チケットリスト
 */
typedef struct {
    ts_ticket_node_t* ps_ticket_top;                // チケットリンクリスト（先頭）
    ts_ticket_node_t* ps_ticket_tail;               // チケットリンクリスト（末尾）
} ts_com_ticket_list_t;

/**
 * イベント情報
 */
typedef struct {
    te_usr_event_t e_event;                         // イベント種別
    te_msg_operating_mode_t e_operating_mode;       // 動作モード
    uint16_t u16_device_sts;                        // デバイスステータス
} ts_com_event_info_t;

/**
 * 共通イベントコールバック関数
 */
typedef void (*tf_com_evt_cb_t)(ts_com_event_info_t* ps_evt);

/**
 * 制御メッセージ
 */
typedef struct {
    esp_bd_addr_t t_bda;                            // BLEアドレス
    te_msg_ctrl_cmd_t e_cmd;                        // 制御コマンド
    te_msg_operating_mode_t e_mode;                 // 動作モード
} ts_ctrl_msg_t;

/**
 * 共通ステータス
 */
typedef struct {
    te_msg_operating_mode_t e_operate_mode;         // 動作モード
    uint16_t u16_device_sts;                        // デバイスステータス
    te_usr_screen_id_t e_scr_id;                    // 現在画面ID
    char c_msg_id[BLE_MSG_ID_SIZE];                 // メッセージID
    esp_bd_addr_t t_rmt_bda;                        // 接続BLEアドレス
    uint64_t u64_rmt_device_id;                     // 接続デバイスID
    bool b_secure_connect;                          // セキュアコネクトフラグ
    char c_pair_chk_code[BLE_MSG_CODE_SIZE + 1];    // ペアリング確認用コード(Base64)
    ts_ctrl_msg_t s_ctrl_msg;                       // 制御メッセージ
    int64_t i64_timeout_ms;                         // タイムアウト時刻（ミリ秒）
} ts_com_status_t;

/**
 * 各画面のステータス
 */
typedef struct {
    int i_disp_row;                     // 表示位置（行）
    te_usr_cursor_type_t e_cursor_type; // カーソルタイプ
    uint8_t u8_cursor_row;              // カーソル位置（行）
    uint8_t u8_cursor_col;              // カーソル位置（列）
    tf_com_evt_cb_t pf_evt_cb;          // 各画面のイベントコールバック
} ts_scr_status_t;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
//==============================================================================
// 初期処理
//==============================================================================
/** デバイス初期処理 */
static void v_init_device();
/** アプリケーション初期処理 */
static esp_err_t sts_init_application();

//==============================================================================
// LCD関連
//==============================================================================
/** LCD初期処理 */
static void v_lcd_init();
/** LCDスクリーン描画処理 */
static void v_lcd_screen_drawing();
/** LCDカーソル描画処理 */
static void v_lcd_cursor_drawing();

//==============================================================================
// センサー関連
//==============================================================================
/** 加速度センサー初期処理 */
static void v_accelerometer_init();
/** 加速度センサー読み込み */
static uint16_t u16_accelerometer_read();
/** センサーステータス読み込み */
static uint16_t u16_sensor_sts_read();

//==============================================================================
// ファイル関連
//==============================================================================
/** 設定ファイル読み込み処理 */
static bool b_read_setting();
/** メッセージファイル読み込み処理 */
static bool b_read_message();
/** チケットファイル読み込み処理 */
static bool b_read_ticket_file();
/** チケットファイル書き込み処理 */
static bool b_write_ticket_file();

//==============================================================================
// BluetoothLE関連処理
//==============================================================================
/** BluetoothLE初期化処理 */
static esp_err_t sts_ble_init();
/** BLE切断イベント処理 */
static void v_ble_disconnection();
/** GAPプロファイルのイベントコールバック */
static void v_ble_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param);

//==============================================================================
// BLEメッセンジャー関連処理
//==============================================================================
/** BLEメッセージイベント処理 */
static void v_msg_evt_cb(te_com_ble_msg_event e_msg_evt);
/** BLEメッセージチケットイベント処理 */
static esp_err_t sts_msg_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット生成処理　※イベントコールバック */
static esp_err_t sts_msg_ticket_create(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット参照処理　※イベントコールバック */
static esp_err_t sts_msg_ticket_read(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット更新処理　※イベントコールバック */
static esp_err_t sts_msg_ticket_update(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット削除処理　※イベントコールバック */
static esp_err_t sts_msg_ticket_delete(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケットノードの取得処理 */
static ts_ticket_node_t* ps_msg_ticket_get_node(uint64_t u64_device_id);
/** BLEメッセージチケット情報の編集処理 */
static esp_err_t sts_msg_ticket_edit_info(uint32_t u32_idx, esp_bd_addr_t t_bda, ts_com_msg_auth_ticket_t* ps_info);
/** BLEペアリング確認用コードの編集処理 */
static esp_err_t sts_msg_pairing_check_code_edit(char* pc_code);
/** BLEペアリング解消処理 */
static esp_err_t sts_msg_unpairing();

//==============================================================================
// 制御メッセージ送受信
//==============================================================================
/** 制御メッセージの送信処理 */
static esp_err_t sts_tx_ctrl_msg(te_msg_ctrl_cmd_t e_cmd);
/** 制御メッセージの受信処理 */
static esp_err_t sts_rx_ctrl_msg();

//==============================================================================
// スレッド間連携
//==============================================================================
/** イベントのエンキュー処理 */
static esp_err_t sts_evt_enqueue(te_usr_event_t e_evt);
/** イベントのデキュー処理 */
static esp_err_t sts_evt_dequeue(ts_com_event_info_t* ps_evt);
/** タイムアウト判定 */
static bool b_evt_chk_timeout();
/** タイムアウト時間の設定 */
static void v_evt_set_timeout(int64_t i64_timeout_ms);
/** タイムアウト時間のクリア */
static void v_evt_clear_timeout();
/** 接続ステータスの更新処理 */
static void v_upd_link_sts(bool b_linked);
/** センサーステータス参照 */
static uint16_t u16_sensor_sts();
/** センサーステータス更新 */
static bool b_upd_sensor_sts(uint16_t u16_sensor_sts);
/** 動作モードの取得 */
static te_msg_operating_mode_t e_get_operating_mode();
/** 動作モードの設定 */
static bool b_set_operating_mode(te_msg_operating_mode_t e_mode);
/** 動作モードを警報に設定 */
static void v_set_alarm_mode();

//==============================================================================
// イベント処理タスク
//==============================================================================
/** イベント処理タスク */
static void v_task_event(void* args);
/** タイマーイベント処理タスク */
static void v_task_timer_event(void* args);

//==============================================================================
// イベントハンドリング
//==============================================================================
/** 共通イベント処理 */
static void v_evt_common(ts_com_event_info_t* ps_evt);
/** 受信コマンド実行処理 */
static void v_evt_exec_command(ts_com_event_info_t* ps_evt);

//==============================================================================
// イベント処理（イベントソース）
//==============================================================================
/** 画面遷移イベント処理 */
static void v_evt_screen_change(te_usr_screen_id_t e_scr_id);
/** メッセージ表示イベント処理 */
static void v_evt_show_msg(char* pc_msg_id);

//==============================================================================
// 各画面のイベント処理
//==============================================================================
/** メッセージ表示 */
static void v_scr_message_display(ts_com_event_info_t* ps_evt);
/** ステータス表示 */
static void v_scr_status_display(ts_com_event_info_t* ps_evt);
/** チケット削除 */
static void v_scr_ticket_delete(ts_com_event_info_t* ps_evt);
/** ペアリングチェック */
static void v_scr_pairing_check(ts_com_event_info_t* ps_evt);

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

// イベントキュー
static QueueHandle_t s_evt_queue = NULL;

/** イベントデーモンハンドル */
static TaskHandle_t s_evt_handle;

/** タイマーイベントデーモンハンドル */
static TaskHandle_t s_timer_handle;

/** SPIバス情報 */
static spi_bus_config_t s_spi_bus_cfg = {
    .mosi_io_num = GPIO_NUM_13,     // ピン番号：MOSI
    .miso_io_num = GPIO_NUM_16,     // ピン番号：MISO
    .sclk_io_num = GPIO_NUM_14,     // ピン番号：SCLK
    .quadwp_io_num = GPIO_NUM_NC,   // ピン番号：ライトプロテクト
    .quadhd_io_num = GPIO_NUM_NC,   // ピン番号：カード挿入検知
    .max_transfer_sz = 4096,        // 最大転送サイズ
};

/** ADCワンショットコンテキスト（較正） */
static ts_adc_oneshot_context* ps_adc_ctx = NULL;

/** LCDステータス */
static ts_lcd_sts_t s_lcd_sts = {
    .e_cursor_type = CURSOR_TYPE_NONE,
    .u8_cursor_row = 0,
    .u8_cursor_col = 0,
    .c_buff = {
        [0] = "                ",
        [1] = "                "
    }
};

/** 加速度センサーステータス */
static ts_accelerometer_sts_t s_axes_sts = {
    .s_address = {
        .e_port_no   = I2C_NUM_0,           // I2Cポート番号
        .u16_address = I2C_ADDR_MPU_6050_L  // I2Cスレーブアドレス（10bit時：0b011110～）
    }
};

/** SDステータス */
static ts_sd_sts_t s_sd_sts = {
    .s_mnt_cfg = {
        .format_if_mount_failed = false,    // マウント不可能時のフォーマット設定：フォーマットしない
        .max_files = 5,                     // 最大オープンファイル数
        .allocation_unit_size = 16 * 1024   // アロケーションユニットサイズ：16KB（Windows最大）
    },
    .ps_card = NULL
};

/** デバイス設定 */
static ts_device_settings_t s_dev_settings = {
    .u64_device_id = 0,                     // デバイスID
    .c_device_name = "                "     // デバイス名
};

/** 表示メッセージ情報 */
static ts_msg_info_t* ps_msg_top = NULL;

/** リモートデバイス情報 */
static ts_com_ticket_list_t s_com_ticket_list = {
    .ps_ticket_top    = NULL,       // チケットリンクリスト（先頭）
    .ps_ticket_tail   = NULL,       // チケットリンクリスト（末尾）
};

/** 共通ステータス */
static ts_com_status_t s_com_status = {
    .e_operate_mode = OPR_MODE_NORMAL,          // 動作モード
    .u16_device_sts = 0x00,                      // デバイスステータス
    .e_scr_id = SCR_ID_STATUS_DISPLAY,          // 現在画面ID
    .c_msg_id = {0x00},                         // メッセージID
    .t_rmt_bda = {0x40},                        // リモートBLEアドレス
    .u64_rmt_device_id = 0x00,                  // 接続デバイスID
    .b_secure_connect = false,                  // セキュアコネクトフラグ
    .c_pair_chk_code = {0x00},                  // ペアリング確認用コード
    .s_ctrl_msg = {
        .t_bda  = {0x40},                       // 送信元アドレス
        .e_cmd  = CTL_CMD_COUNT,                // 制御コマンド
        .e_mode = OPR_MODE_COUNT,               // 動作モード
    },
    .i64_timeout_ms = MAX_VALUE_INT64,          // タイムアウト（ミリ秒）
};

/** 各画面のステータス */
static ts_scr_status_t s_scr_sts_list[SCR_ID_COUNT] = {
    // メッセージ表示
    [SCR_ID_MSG_DISPLAY] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                     // カーソル位置（行）
        .u8_cursor_col = 0,                     // カーソル位置（列）
        .pf_evt_cb = v_scr_message_display,     // イベントコールバック
    },
    // ステータス表示
    [SCR_ID_STATUS_DISPLAY] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                     // カーソル位置（行）
        .u8_cursor_col = 0,                     // カーソル位置（列）
        .pf_evt_cb = v_scr_status_display,      // イベントコールバック
    },
    // チケット削除
    [SCR_ID_TICKET_DELETE] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                     // カーソル位置（行）
        .u8_cursor_col = 0,                     // カーソル位置（列）
        .pf_evt_cb = v_scr_ticket_delete,       // イベントコールバック
    },
    // ペアリングチェック
    [SCR_ID_PAIRING_CHECK] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                     // カーソル位置（行）
        .u8_cursor_col = 0,                     // カーソル位置（列）
        .pf_evt_cb = v_scr_pairing_check,       // イベントコールバック
    },
};

//==============================================================================
// SPP関係の定義
//==============================================================================
// 製造元データポイント
static uint8_t ble_manufacturer[3] = {'E', 'S', 'P'};
/** サービス（オリジナル）のUUID */
static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x55, 0x8f, 0xaf, 0xa3, 0x8f, 0xc2, 0x47, 0x2b, 0x83, 0xcb, 0xad, 0xe9, 0x3b, 0xa6, 0xfb, 0x31
};

/** アドバタイズデータ設定：アドバタイズデータ設定 */
static esp_ble_adv_data_t gap_adv_config = {
    .set_scan_rsp     = false,      // スキャン応答設定：スキャン応答としては利用しない
    .include_txpower  = true,       // 送信出力情報設定：送信出力の情報を付加する
    .min_interval     = 0x0006,     // アドバタイズデータの送信間隔の最小時間：Time = min_interval * 1.25 msec
    .max_interval     = 0x0010,     // アドバタイズデータの送信間隔の最大時間：Time = max_interval * 1.25 msec
    .appearance       = 0x00,       // 外部装置としてのタイプ
    .manufacturer_len = 0,          // 製造者固有データ（データ長）
    .p_manufacturer_data = NULL,    // 製造者固有データ（製造者名ポインタ？）
    .service_data_len = 0,          // サービスデータ長
    .p_service_data   = NULL,       // サービスデータポインタ
    .service_uuid_len = sizeof(sec_service_uuid),   // サービスUUID長
    .p_service_uuid   = sec_service_uuid,           // サービスUUIDポインタ
    // アドバタイジングモード設定フラグ：常時デバイスの発見が可能、BLEのみ対応
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/** アドバタイズデータ設定：スキャン応答データ設定 */
static esp_ble_adv_data_t gap_scan_rsp_config = {
    .set_scan_rsp = true,   // スキャン応答設定：スキャン応答としては利用
    .include_name = true,   // デバイス名を含めるかどうか：含める
    .manufacturer_len = sizeof(ble_manufacturer),   // 製造者固有データ（データ長）
    .p_manufacturer_data = ble_manufacturer,        // 製造者固有データ（製造者名ポインタ？）
};

/** アドバタイジングパラメータ */
static esp_ble_adv_params_t gap_adv_params = {
    // アドバタイジング間隔：Time = N * 0.625 msec
    // Time Range: 20 ms to 40 ms
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    // アドバタイズタイプ
    // ADV_TYPE_IND：コネクション可能、スキャン可能
    // ADV_TYPE_DIRECT_IND_HIGH：コネクション可能、スキャン不可能、デューティサイクル高
    // ADV_TYPE_SCAN_IND：コネクション不可能、スキャン可能
    // ADV_TYPE_NONCONN_IND：コネクション不可能、スキャン不可能
    // ADV_TYPE_DIRECT_IND_LOW：コネクション可能、スキャン不可能、デューティサイクル低
    .adv_type           = ADV_TYPE_IND,
    // アドレスタイプ
    // BLE_ADDR_TYPE_PUBLIC    ：デバイス固有アドレス
    // BLE_ADDR_TYPE_RANDOM    ：ランダムアドレス
    // BLE_ADDR_TYPE_RPA_PUBLIC：解決可能（一定間隔で変更される）なデバイス固有アドレス
    // BLE_ADDR_TYPE_RPA_RANDOM：解決可能（一定間隔で変更される）なランダムアドレス
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
//    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
//    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
//    .own_addr_type      = BLE_ADDR_TYPE_RPA_RANDOM,
    .channel_map        = ADV_CHNL_ALL, // アドバタイジングに使用するチャンネル
    // アドバタイジングのフィルタリングポリシー
    // ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY  ：スキャン要求許可、接続要求許可
    // ADV_FILTER_ALLOW_SCAN_WLST_CON_ANY ：ホワイトリストデバイスからのみスキャン要求許可、接続要求許可
    // ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST ：スキャン要求許可、ホワイトリストデバイスからのみ接続要求許可
    // ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST：ホワイトリストデバイスからのみスキャン要求許可と接続要求許可
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//******************************************************************************
// GATTプロファイルのインターフェース毎の情報を管理する構造体
// アプリケーション側で管理する必要がある
//******************************************************************************
static ts_com_ble_gatts_if_config_t s_gatts_cfg_tbls;

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: app_main
 *
 * DESCRIPTION:メイン関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void app_main() {
    //==========================================================================
    // 初期処理
    //==========================================================================
    // ミューテックスの初期化
    s_mutex = xSemaphoreCreateRecursiveMutex();
    // デバイス初期処理
    v_init_device();
    // アプリケーション初期処理
    ESP_ERROR_CHECK(sts_init_application());
    // BluetoothLEの初期処理
    ESP_ERROR_CHECK(sts_ble_init());

    //==========================================================================
    // イベント処理プロセスの起動
    //==========================================================================
    // イベントキューの生成
    s_evt_queue = xQueueCreate(EVT_QUEUE_SIZE, sizeof(te_usr_event_t));
    // イベント処理タスクを起動する
    xTaskCreatePinnedToCore(v_task_event, "event task", 32768, NULL, TASK_PRIORITIES_LOW, &s_evt_handle, tskNO_AFFINITY);
    // タイマー処理タスクを起動する
    xTaskCreatePinnedToCore(v_task_timer_event, "timer event task", 16384, NULL, TASK_PRIORITIES_MEDIUM, &s_timer_handle, tskNO_AFFINITY);
    // 初期イベント：起動画面への画面遷移
    v_evt_screen_change(SCR_ID_STATUS_DISPLAY);

    //==========================================================================
    // mainの終端処理
    //==========================================================================
    while(true) {
        // アイドルタスク待ち
        vTaskDelay(IDLE_TASK_WAIT_TICK);
    }
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_init_device
 *
 * DESCRIPTION:デバイス初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool: true(正常処理)
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_init_device() {
    //==========================================================================
    // ESP32の共通初期処理
    //==========================================================================
    // NVS領域をクリア、エラー時には異常終了する
//    ESP_ERROR_CHECK(nvs_flash_erase());
    // Initialize NVS.
    // 不揮発性メモリ領域のデフォルトパーティションを初期化する
    esp_err_t sts_val = nvs_flash_init();
    // NVS領域を初期化した際のエラー判定として次の場合を判定
    // 1.NVSの空きページが無い
    // 2.新バージョンのデータがパーティションに含まれている
    if (sts_val == ESP_ERR_NVS_NO_FREE_PAGES || sts_val == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS領域をクリア、エラー時には異常終了する
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(sts_val);

    //==========================================================================
    // ウォッチドックタイマーの初期化処理
    //==========================================================================
    // ウォッチドックタイマーの初期化処理
    esp_task_wdt_config_t s_wdt_cfg = {
        .timeout_ms = TWDT_TIMEOUT_MSEC,                        // タイムアウト時間
        .idle_core_mask = (0x01 << portNUM_PROCESSORS) - 0x01,  // 全てのコアを対象に設定
        .trigger_panic = true                                   // パニックトリガーON
    };
    CHECK_ERROR_CODE(esp_task_wdt_reconfigure(&s_wdt_cfg), ESP_OK);

    //==========================================================================
    // ログ出力設定
    //==========================================================================
#ifndef DEBUG_ALARM
    esp_log_level_set("*", ESP_LOG_NONE);
#else
    esp_log_level_set("*", ESP_LOG_INFO);
#endif

    //==========================================================================
    // GPIO設定（ペアリング入力）
    //==========================================================================
    // GPIO setting
    gpio_config_t s_gpio_cfg_pairing;
    s_gpio_cfg_pairing.pin_bit_mask = (1ULL << COM_GPIO_PAIRING_SETTING);
    s_gpio_cfg_pairing.mode         = GPIO_MODE_INPUT;
    s_gpio_cfg_pairing.pull_up_en   = GPIO_PULLUP_ENABLE;
    s_gpio_cfg_pairing.pull_down_en = GPIO_PULLDOWN_DISABLE;
    s_gpio_cfg_pairing.intr_type    = GPIO_INTR_DISABLE;
    // GPIO config
    ESP_ERROR_CHECK(gpio_config(&s_gpio_cfg_pairing));

    //==========================================================================
    // GPIO設定（センサー入力ポート）
    //==========================================================================
    // GPIO map
    uint64_t u64_pin_map_input = 0x00;
    u64_pin_map_input |= (1ULL << COM_GPIO_PORT1_RADAR);
    u64_pin_map_input |= (1ULL << COM_GPIO_PORT1_MOTION);
    u64_pin_map_input |= (1ULL << COM_GPIO_PORT2_RADAR);
    u64_pin_map_input |= (1ULL << COM_GPIO_PORT2_MOTION);
    // GPIO setting
    gpio_config_t s_gpio_cfg_input;
    s_gpio_cfg_input.pin_bit_mask = u64_pin_map_input;
    s_gpio_cfg_input.mode         = GPIO_MODE_INPUT;
    // TODO: DEBUG Start
//    s_gpio_cfg_input.pull_up_en   = GPIO_PULLUP_DISABLE;
//    s_gpio_cfg_input.pull_down_en = GPIO_PULLDOWN_ENABLE;
    s_gpio_cfg_input.pull_up_en   = GPIO_PULLUP_ENABLE;
    s_gpio_cfg_input.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // TODO: DEBUG End
    s_gpio_cfg_input.intr_type    = GPIO_INTR_DISABLE;
    // GPIO config
    ESP_ERROR_CHECK(gpio_config(&s_gpio_cfg_input));

    //==========================================================================
    // ADC初期化
    //==========================================================================
    // ５方向スイッチ読み取り用ADCコンテキストを生成
    ps_adc_ctx = ps_adc_oneshot_calibration_ctx(ADC_UNIT_1,
                                                ADC_DIGI_CLK_SRC_DEFAULT,
                                                ADC_ULP_MODE_DISABLE,
                                                ADC_ATTEN_DB_11);
    // ADCチャンネル設定
    sts_val = sts_adc_oneshot_config_channel(ps_adc_ctx,
                                             COM_5WAY_CHANNEL,
                                             ADC_ATTEN_DB_11,
                                             ADC_BITWIDTH_12);
    // エラーチェック
    ESP_ERROR_CHECK(sts_val);

    //==========================================================================
    // SPIバス初期化処理
    //==========================================================================
    // SPIバスの初期化
    sdmmc_host_t s_host = SDSPI_HOST_DEFAULT();
    s_host.slot = SPI2_HOST;
    esp_err_t ret = sts_spi_mst_bus_initialize(s_host.slot, &s_spi_bus_cfg, SDSPI_DEFAULT_DMA, true);
    if (ret != ESP_OK) {
        return;
    }

    //==========================================================================
    // I2Cバス初期処理
    //==========================================================================
    // I2Cバスの初期化
    sts_val = sts_io_i2c_mst_init(I2C_NUM_0,            // I2Cポート番号
                                  I2C_FREQ_HZ_FAST,     // バススピードモード
                                  GPIO_NUM_22,          // SCLピン番号
                                  GPIO_NUM_21,          // SDAピン番号
                                  GPIO_PULLUP_ENABLE);  // プルアップ設定
    // エラーチェック
    ESP_ERROR_CHECK(sts_val);
    // クロックストレッチのタイムアウトを最大に設定
    i2c_set_timeout(I2C_NUM_0, 0xFFFFF);
    // I2Cプルアップ
    gpio_set_pull_mode(GPIO_NUM_22, GPIO_PULLUP_ONLY);
    gpio_pullup_en(GPIO_NUM_22);
    gpio_set_pull_mode(GPIO_NUM_21, GPIO_PULLUP_ONLY);
    gpio_pullup_en(GPIO_NUM_21);

    //==========================================================================
    // LCD初期化
    //==========================================================================
    v_lcd_init();

    //==========================================================================
    // 加速度センサー初期化
    //==========================================================================
    v_accelerometer_init();
}

/*******************************************************************************
 *
 * NAME: sts_init_application
 *
 * DESCRIPTION:アプリケーション初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_init_application() {
    //==========================================================================
    // SDMMC初期化処理
    //==========================================================================
    // SDマウント設定
    esp_vfs_fat_sdmmc_mount_config_t* ps_mnt_cfg = &s_sd_sts.s_mnt_cfg;
    // SDMMCカードのマウント
    // 第１引数：マウントパス
    // 第２引数：CS
    // 第３引数：CD
    // 第４引数：WP
    // 第５引数：SDマウント設定
    s_sd_sts.ps_card = ps_futil_sdmmc_hspi_mount(COM_MOUNT_SD, GPIO_NUM_15, GPIO_NUM_NC, GPIO_NUM_NC, ps_mnt_cfg);
    if (s_sd_sts.ps_card == NULL) {
        // ファイルシステムのマウント失敗時
        return ESP_FAIL;
    }

    //==========================================================================
    // 設定ファイル読み込み
    //==========================================================================
    if (!b_read_setting()) {
        return ESP_FAIL;
    }

    //==========================================================================
    // メッセージファイル読み込み
    //==========================================================================
    if (!b_read_message()) {
        return ESP_FAIL;
    }

    //==========================================================================
    // チケット情報を読み込み
    //==========================================================================
    if (!b_read_ticket_file()) {
        return ESP_FAIL;
    }

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_lcd_init
 *
 * DESCRIPTION:LCD初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_lcd_init() {
    //==========================================================================
    // 初期化
    //==========================================================================
    // LCD初期化
    ESP_ERROR_CHECK(sts_st7032i_init(I2C_NUM_0));
    // コントラスト設定
    ESP_ERROR_CHECK(sts_st7032i_set_contrast(I2C_NUM_0, COM_LCD_CONTRAST));
    // カーソル非表示
    ESP_ERROR_CHECK(sts_st7032i_disp_control(I2C_NUM_0, DRV_ST7032I_DISP_NONE));
    // カーソル位置初期化
    ESP_ERROR_CHECK(sts_st7032i_return_home(I2C_NUM_0));
    // クリアアイコン
    ESP_ERROR_CHECK(sts_st7032i_clear_icon(I2C_NUM_0));
    // クリアスクリーン
    ESP_ERROR_CHECK(sts_st7032i_clear_screen(I2C_NUM_0));
}

/*******************************************************************************
 *
 * NAME: v_lcd_screen_drawing
 *
 * DESCRIPTION:LCDスクリーン描画処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_lcd_screen_drawing() {
    //==========================================================================
    // LCD書き込み
    //==========================================================================
    // バッファに終端文字を書き込み
    s_lcd_sts.c_buff[0][16] = '\0';
    s_lcd_sts.c_buff[1][16] = '\0';
    // カーソルを先頭に移動
    ESP_ERROR_CHECK(sts_st7032i_set_cursor(I2C_NUM_0, 0, 0));
    // 文字列書き込み
    ESP_ERROR_CHECK(sts_st7032i_write_string(I2C_NUM_0, s_lcd_sts.c_buff[0]));
    // カーソルを先頭に移動
    ESP_ERROR_CHECK(sts_st7032i_set_cursor(I2C_NUM_0, 1, 0));
    // 文字列書き込み
    ESP_ERROR_CHECK(sts_st7032i_write_string(I2C_NUM_0, s_lcd_sts.c_buff[1]));
    // カーソル位置設定
    ESP_ERROR_CHECK(sts_st7032i_set_cursor(I2C_NUM_0, s_lcd_sts.u8_cursor_row, s_lcd_sts.u8_cursor_col));
}

/*******************************************************************************
 *
 * NAME: v_lcd_cursor_drawing
 *
 * DESCRIPTION:LCDカーソル描画処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_lcd_cursor_drawing() {
    // カーソル位置設定
    ESP_ERROR_CHECK(sts_st7032i_set_cursor(I2C_NUM_0, s_lcd_sts.u8_cursor_row, s_lcd_sts.u8_cursor_col));
    // カーソルタイプ
    te_st7032i_disp_sts_t e_disp_sts = DRV_ST7032I_DISP_ON;
    if (s_lcd_sts.e_cursor_type == CURSOR_TYPE_DISPLAY) {
        e_disp_sts = DRV_ST7032I_DISP_ON
                   | DRV_ST7032I_DISP_BLINK;
    } else if (s_lcd_sts.e_cursor_type == CURSOR_TYPE_WAIT) {
        e_disp_sts = DRV_ST7032I_DISP_ON
                   | DRV_ST7032I_DISP_CURSOR;
    }
    // カーソル描画
    ESP_ERROR_CHECK(sts_st7032i_disp_control(I2C_NUM_0, e_disp_sts));
}

/*******************************************************************************
 *
 * NAME: v_accelerometer_init
 *
 * DESCRIPTION:加速度センサー初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_accelerometer_init() {
    //==========================================================================
    // 加速度センサー初期化
    //==========================================================================
    // リセット
    ESP_ERROR_CHECK(sts_mpu_6050_device_reset(s_axes_sts.s_address));
    // 初期化
    ESP_ERROR_CHECK(sts_mpu_6050_init(s_axes_sts.s_address, DRV_MPU_6050_ACCEL_RANGE_4G, DRV_MPU_6050_GYRO_RANGE_250));
    // 設定：ジャイロサンプリングレート
    // サンプルレート = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Gyroscope Output Rate ＝8KHz(DLPFが有効の場合は1KHz)
//    ESP_ERROR_CHECK(sts_mpu_6050_set_smplrt_div(s_axes_sts.s_address, 0x00));
    // 設定：ローパスフィルター
    // 加速度:260Hz以上をカット、ジャイロ:258Hz以上をカット
//    ESP_ERROR_CHECK(sts_mpu_6050_set_dlpf_cfg(s_axes_sts.s_address, DRV_MPU_6050_LPF_260_256));
    // 設定：ハイパスフィルター
    ESP_ERROR_CHECK(sts_mpu_6050_set_accel_hpf(s_axes_sts.s_address, DRV_MPU_6050_ACCEL_HPF_0P63HZ));
//    ESP_ERROR_CHECK(sts_mpu_6050_set_accel_hpf(s_axes_sts.s_address, DRV_MPU_6050_ACCEL_HPF_1P25HZ));
    // 設定：ジャイロレンジ
//    ESP_ERROR_CHECK(sts_mpu_6050_set_gyro_range(s_axes_sts.s_address, DRV_MPU_6050_GYRO_RANGE_250));
    // 設定：加速度レンジ
//    ESP_ERROR_CHECK(sts_mpu_6050_set_accel_range(s_axes_sts.s_address, DRV_MPU_6050_ACCEL_RANGE_2G));
    // 設定：FIFO有効無効設定（FIFO無効化）
//    ESP_ERROR_CHECK(sts_mpu_6050_set_fifo_enable(s_axes_sts.s_address, false, false, false, false, false));
    // クロック設定 ※内部オシレータ8MHz
//    ESP_ERROR_CHECK(sts_mpu_6050_set_clock(s_axes_sts.s_address, DRV_MPU_6050_CLK_INTERNAL));
    // who am i
//    ESP_ERROR_CHECK(sts_mpu_6050_who_am_i(s_axes_sts.s_address));
    // 起動待ち
    vTaskDelay(EVT_BOOT_WAIT_TICK);
    // zeroing accel
    ESP_ERROR_CHECK(sts_mpu_6050_zeroing_accel(s_axes_sts.s_address));
    // zeroing gyro
//    ESP_ERROR_CHECK(sts_mpu_6050_zeroing_gyro(s_axes_sts.s_address));
}

/*******************************************************************************
 *
 * NAME: u16_accelerometer_read
 *
 * DESCRIPTION:加速度センサー読み込み
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   uint16_t:加速度の合成値
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint16_t u16_accelerometer_read() {
    // 加速度（XYZ軸）読み込み
    ts_mpu_6050_axes_data_t s_axes_data;
    esp_err_t sts_val = sts_mpu_6050_read_accel(s_axes_sts.s_address, &s_axes_data);
    if (sts_val != ESP_OK) {
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d read accel sts=%d", __func__, __LINE__, sts_val);
#endif
        return 0;
    }
    // ３軸の加速度の合成値
    return i16_mpu_6050_composite_value(&s_axes_data, false);
}

/*******************************************************************************
 *
 * NAME: u16_sensor_sts_read
 *
 * DESCRIPTION:センサーステータス読み込み
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint16_t u16_sensor_sts_read() {
    // 前回検出値
    static uint16_t u16_before_map = 0x00;
    // レーダー無効化の期限
    static int64_t i64_desable_end = 0x00;
    // 現在時刻
    int64_t i64_now = xTaskGetTickCountMSec();
    // 検出値マップ
    uint16_t u16_detected_map = 0x00;
    // GPIO Pairing setting pin
    if (gpio_get_level(COM_GPIO_PAIRING_SETTING) != GPIO_HIGH) {
        u16_detected_map |= DEV_STS_PAIRING_ENABLED;
    }
    // 加速度チェック
    uint16_t u16_composite_val = u16_accelerometer_read();
    if (u16_composite_val >= COM_AXES_THRESHOLD) {
        // 加速度を検知
#ifdef DEBUG_ALARM
        ESP_LOGW(LOG_MSG_TAG, "%s L#%d accelerometer=%d", __func__, __LINE__, u16_composite_val);
#endif
        u16_detected_map |= DEV_STS_ACCELERATION;
        // レーダー検知無効の期限を更新
        i64_desable_end = i64_now + COM_SENSOR_DISABLE_INTERVAL;
    }
    // レーダー検知の有効判定　※地震による誤作動防止対策
    if (i64_now > i64_desable_end) {
        // GPIO Port1 radar
        if (gpio_get_level(COM_GPIO_PORT1_RADAR) != GPIO_HIGH) {
            u16_detected_map |= DEV_STS_PORT1_RADAR;
        }
        // GPIO Port2 radar
        if (gpio_get_level(COM_GPIO_PORT2_RADAR) != GPIO_HIGH) {
            u16_detected_map |= DEV_STS_PORT2_RADAR;
        }
    }
    // GPIO Port1 motion
    if (gpio_get_level(COM_GPIO_PORT1_MOTION) != GPIO_HIGH) {
        u16_detected_map |= DEV_STS_PORT1_MOTION;
    }
    // GPIO Port2 motion
    if (gpio_get_level(COM_GPIO_PORT2_MOTION) != GPIO_HIGH) {
        u16_detected_map |= DEV_STS_PORT2_MOTION;
    }
    // ペアリングスイッチだけチャタリングする
    uint16_t u16_result_map = (u16_before_map | ~DEV_STS_PAIRING_ENABLED) & u16_detected_map;
    // 前回検出値を更新
    u16_before_map = u16_detected_map;
    // 検出値を返却
    return u16_result_map;
}

/*******************************************************************************
 *
 * NAME: b_read_setting
 *
 * DESCRIPTION:設定ファイル読み込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool: true(正常処理完了)
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_read_setting() {
    // 設定ファイル読み込み
    cJSON* ps_setting = ps_futil_cjson_parse_file(COM_PATH_SETTING, i_vutil_conv_to_kilo(10));
    if (ps_setting == NULL) {
        // 設定ファイルの読み込みエラー
        return false;
    }
    // 結果ステータス
    bool b_sts = false;
    do {
        // デバイスIDの読み込み
        cJSON* ps_device_id = cJSON_GetObjectItem(ps_setting, "device_id");
        if (ps_device_id == NULL) {
            break;
        }
        s_dev_settings.u64_device_id = u64_vutil_to_numeric(ps_device_id->valuestring);
        // リモートデバイスIDを合わせて初期化
        s_com_status.u64_rmt_device_id = s_dev_settings.u64_device_id;
        // デバイス名の読み込み
        cJSON* ps_device_name = cJSON_GetObjectItem(ps_setting, "device_name");
        if (ps_device_name == NULL) {
            break;
        }
        int i_len = i_vutil_strlen(ps_device_name->valuestring);
        if (i_len <= 0 || i_len > 16) {
            break;
        }
        strcpy(s_dev_settings.c_device_name, ps_device_name->valuestring);
        // 編集完了
        b_sts = true;
    } while(false);
    // cJSON解放
    cJSON_Delete(ps_setting);

    // 結果返信
    return b_sts;
}

/*******************************************************************************
 *
 * NAME: b_read_message
 *
 * DESCRIPTION:メッセージファイル読み込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool: true(正常処理完了)
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_read_message() {
    // メッセージファイルを読み込み
    cJSON* ps_message = ps_futil_cjson_parse_file(COM_PATH_MESSAGE, i_vutil_conv_to_kilo(10));
    if (ps_message == NULL) {
        return false;
    }
    // メッセージリストの取得
    cJSON* ps_messsage_list = cJSON_GetObjectItem(ps_message, "msg_list");
    if (ps_messsage_list == NULL) {
        // cJSON解放
        cJSON_Delete(ps_message);
        return false;
    }
    // メッセージリストサイズ
    int i_list_size = cJSON_GetArraySize(ps_messsage_list);
    if (i_list_size <= 0) {
        // cJSON解放
        cJSON_Delete(ps_message);
        return false;
    }
    // メッセージループ
    ts_msg_info_t* ps_msg_bef = NULL;
    ts_msg_info_t* ps_msg_tgt = NULL;
    cJSON* ps_msg_elm = NULL;
    cJSON* ps_msg_id = NULL;
    cJSON* ps_msg = NULL;
    int i_idx;
    for (i_idx = 0; i_idx < i_list_size; i_idx++) {
        // メッセージ要素
        ps_msg_elm = cJSON_GetArrayItem(ps_messsage_list, i_idx);
        // メッセージIDの読み込み
        ps_msg_id = cJSON_GetObjectItem(ps_msg_elm, "msg_id");
        if (ps_msg_id == NULL) {
            break;
        }
        if (i_vutil_strlen(ps_msg_id->valuestring) != 5) {
            break;
        }
        // メッセージの読み込み
        ps_msg = cJSON_GetObjectItem(ps_msg_elm, "msg");
        if (ps_msg == NULL) {
            break;
        }
        if (i_vutil_strlen(ps_msg->valuestring) > 16) {
            break;
        }
        // 表示メッセージ情報生成
        ps_msg_bef = ps_msg_tgt;
        ps_msg_tgt = pv_mem_malloc(sizeof(ts_msg_info_t));
        if (ps_msg_tgt == NULL) {
            break;
        }
        // メッセージ編集
        strcpy(ps_msg_tgt->c_msg_id, ps_msg_id->valuestring);
        strcpy(ps_msg_tgt->c_msg, ps_msg->valuestring);
        ps_msg_tgt->ps_next = NULL;
        // メッセージ追加
        if (ps_msg_bef != NULL) {
            ps_msg_bef->ps_next = ps_msg_tgt;
        } else {
            ps_msg_top = ps_msg_tgt;
        }
    }
    // cJSON解放
    cJSON_Delete(ps_message);
    // エラー判定
    if (i_idx < i_list_size) {
        // 動的に生成したメッセージを解放
        ps_msg_tgt = ps_msg_top;
        while (ps_msg_tgt != NULL) {
            ps_msg_bef = ps_msg_tgt;
            ps_msg_tgt = ps_msg_tgt->ps_next;
            l_mem_free(ps_msg_bef);
        }
        ps_msg_top = NULL;
        // エラー返信
        return false;
    }

    // 正常読み込み
    return true;
}

/*******************************************************************************
 *
 * NAME: b_read_ticket_file
 *
 * DESCRIPTION:チケットファイル読み込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool: true(正常処理完了)
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_read_ticket_file() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    // 読み込み結果
    bool b_result = false;
    do {

        //----------------------------------------------------------------------
        // チケットファイルを読み込み
        //----------------------------------------------------------------------
        cJSON* ps_ticket_root = ps_futil_cjson_parse_file(COM_PATH_TICKET, i_vutil_conv_to_kilo(10));
        if (ps_ticket_root == NULL) {
            b_result = true;
            break;
        }
        // チケットリストの取得
        cJSON* ps_ticket_list = cJSON_GetObjectItem(ps_ticket_root, COM_TICKET_LIST);
        if (ps_ticket_list == NULL) {
            // cJSON解放
            cJSON_Delete(ps_ticket_root);
            b_result = true;
            break;
        }
        // チケットリストサイズ
        int i_list_size = cJSON_GetArraySize(ps_ticket_list);
        if (i_list_size <= 0) {
            // cJSON解放
            cJSON_Delete(ps_ticket_root);
            b_result = true;
            break;
        }

        //----------------------------------------------------------------------
        // チケット情報生成
        //----------------------------------------------------------------------
        // 先頭のダミーチケット
        ts_ticket_node_t s_ticket_node_dmy_top = {0x00};
        s_ticket_node_dmy_top.ps_next = NULL;
        // 作業用チケットポインタ
        ts_ticket_node_t* ps_ticket_node_bef = NULL;
        ts_ticket_node_t* ps_ticket_node_tgt = &s_ticket_node_dmy_top;
        ts_com_msg_auth_ticket_t* ps_ticket_edit;
        // 各項目(CJSON)
        cJSON* ps_ticket_elm      = NULL;   // チケット
        cJSON* ps_own_device_id   = NULL;   // 自デバイスID
        cJSON* ps_rmt_device_id   = NULL;   // リモートデバイスID
        cJSON* ps_rmt_device_bda  = NULL;   // リモートデバイスアドレス
        cJSON* ps_enc_key = NULL;           // 暗号鍵
        cJSON* ps_own_sts = NULL;           // 自ステータス
        cJSON* ps_rmt_sts_hash = NULL;      // 相手ステータスハッシュ
        cJSON* ps_max_seq_no;               // 最大シーケンス番号
        cJSON* ps_tx_seq_no;                // 送信シーケンス番号
        cJSON* ps_rx_seq_no;                // 受信シーケンス番号
        int i_idx;
        for (i_idx = 0; i_idx < i_list_size; i_idx++) {
            // チケット要素
            ps_ticket_elm = cJSON_GetArrayItem(ps_ticket_list, i_idx);
            // 自デバイスIDの読み込み
            ps_own_device_id = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_OWN_DEV_ID);
            if (ps_own_device_id == NULL) {
                break;
            }
            if (!b_vutil_dec_string(ps_own_device_id->valuestring, 20)) {
                break;
            }
            // リモートデバイスIDの読み込み
            ps_rmt_device_id = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_RMT_DEV_ID);
            if (ps_rmt_device_id == NULL) {
                break;
            }
            if (!b_vutil_dec_string(ps_rmt_device_id->valuestring, 20)) {
                break;
            }
            // リモートデバイスアドレス
            ps_rmt_device_bda = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_RMT_DEV_BDA);
            if (ps_rmt_device_bda == NULL) {
                break;
            }
            int i_dev_bda_len = i_vutil_strlen(ps_rmt_device_bda->valuestring);
            if (i_dev_bda_len != COM_TICKET_DEV_BDA_BASE64_SIZE) {
                break;
            }
            if (i_vutil_byte_len_base64(ps_rmt_device_bda->valuestring, COM_TICKET_DEV_BDA_BASE64_SIZE) != COM_TICKET_DEV_BDA_SIZE) {
                break;
            }
            // 暗号鍵の読み込み
            ps_enc_key = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_ENC_KEY);
            if (ps_enc_key == NULL) {
                break;
            }
            if (i_vutil_byte_len_base64(ps_enc_key->valuestring, 44) != COM_MSG_SIZE_CIPHER_KEY) {
                break;
            }
            // 自ステータス
            ps_own_sts = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_OWN_STS);
            if (ps_own_sts == NULL) {
                break;
            }
            if (i_vutil_byte_len_base64(ps_own_sts->valuestring, 44) != COM_MSG_SIZE_TICKET_STS) {
                break;
            }
            // 相手ステータスハッシュ
            ps_rmt_sts_hash = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_RMT_HASH);
            if (ps_rmt_sts_hash == NULL) {
                break;
            }
            if (i_vutil_byte_len_base64(ps_rmt_sts_hash->valuestring, 44) != COM_MSG_SIZE_TICKET_STS) {
                break;
            }
            // 最大シーケンス番号
            ps_max_seq_no = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_MAX_SEQ);
            if (ps_max_seq_no == NULL) {
                break;
            }
            if (!b_vutil_dec_string(ps_max_seq_no->valuestring, 10)) {
                break;
            }
            // 送信シーケンス番号
            ps_tx_seq_no = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_TX_SEQ);
            if (ps_tx_seq_no == NULL) {
                break;
            }
            if (!b_vutil_dec_string(ps_tx_seq_no->valuestring, 10)) {
                break;
            }
            // 受信シーケンス番号
            ps_rx_seq_no = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_RX_SEQ);
            if (ps_rx_seq_no == NULL) {
                break;
            }
            if (!b_vutil_dec_string(ps_rx_seq_no->valuestring, 10)) {
                break;
            }
            // チケット生成
            ps_ticket_node_bef = ps_ticket_node_tgt;
            ps_ticket_node_tgt = pv_mem_malloc(sizeof(ts_ticket_node_t));
            if (ps_ticket_node_tgt == NULL) {
                break;
            }
            ps_ticket_node_bef->ps_next = ps_ticket_node_tgt;
            // リモートデバイスアドレス
            i_vutil_base64_decode(ps_ticket_node_tgt->t_rmt_device_bda, ps_rmt_device_bda->valuestring);
            // 編集チケット
            ps_ticket_edit = &ps_ticket_node_tgt->s_ticket;
            // 自デバイスID
            ps_ticket_edit->u64_own_device_id = u64_vutil_to_numeric(ps_own_device_id->valuestring);
            // リモートデバイスID
            ps_ticket_edit->u64_rmt_device_id = u64_vutil_to_numeric(ps_rmt_device_id->valuestring);
            // 暗号鍵
            i_vutil_base64_decode(ps_ticket_edit->u8_enc_key, ps_enc_key->valuestring);
            // 自ステータス
            i_vutil_base64_decode(ps_ticket_edit->u8_own_sts, ps_own_sts->valuestring);
            // リモートステータスハッシュ
            i_vutil_base64_decode(ps_ticket_edit->u8_rmt_sts_hash, ps_rmt_sts_hash->valuestring);
            // 最大シーケンス番号
            ps_ticket_edit->u32_max_seq_no = u32_vutil_to_numeric(ps_max_seq_no->valuestring);
            // 送信シーケンス番号
            ps_ticket_edit->u32_tx_seq_no = u32_vutil_to_numeric(ps_tx_seq_no->valuestring);
            // 受信シーケンス番号
            ps_ticket_edit->u32_rx_seq_no = u32_vutil_to_numeric(ps_rx_seq_no->valuestring);
            // 次のチケットを初期化
            ps_ticket_node_tgt->ps_next = NULL;
        }
        // cJSON解放
        cJSON_Delete(ps_ticket_root);
        // 編集エラー判定
        if (i_idx < i_list_size) {
            // 動的に生成したチケットを解放
            ps_ticket_node_tgt = s_ticket_node_dmy_top.ps_next;
            while (ps_ticket_node_tgt != NULL) {
                ps_ticket_node_bef = ps_ticket_node_tgt;
                ps_ticket_node_tgt = ps_ticket_node_tgt->ps_next;
                l_mem_free(ps_ticket_node_bef);
            }
            break;
        }

        //----------------------------------------------------------------------
        // チケット情報の更新
        //----------------------------------------------------------------------
        // 既存のチケット情報を削除
        ts_ticket_node_t* ps_ticket_del = NULL;
        ts_ticket_node_t* ps_ticket_chk = s_com_ticket_list.ps_ticket_top;
        while(ps_ticket_chk != NULL) {
            ps_ticket_del = ps_ticket_chk;
            ps_ticket_chk = ps_ticket_chk->ps_next;
            l_mem_free(ps_ticket_del);
        }
        s_com_ticket_list.ps_ticket_top  = NULL;
        s_com_ticket_list.ps_ticket_tail = NULL;
        // チケット情報そ更新
        s_com_ticket_list.ps_ticket_top = s_ticket_node_dmy_top.ps_next;
        if (ps_ticket_node_tgt != &s_ticket_node_dmy_top) {
            s_com_ticket_list.ps_ticket_tail = ps_ticket_node_tgt;
        }
        // 正常に終了1
        b_result = true;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return b_result;
}

/*******************************************************************************
 *
 * NAME: b_write_ticket_file
 *
 * DESCRIPTION:チケットファイル書き込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool: true(正常処理完了)
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_write_ticket_file() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    // 書き出し結果
    esp_err_t sts_val = ESP_FAIL;
    do {
        //----------------------------------------------------------------------
        // チケットイメージの編集
        //----------------------------------------------------------------------
        // チケットファイルのJSONルート生成
        cJSON* ps_ticket_root = cJSON_CreateObject();
        // チケットリスト追加
        cJSON* ps_ticket_list = cJSON_CreateArray();
        cJSON_AddItemToObject(ps_ticket_root, COM_TICKET_LIST, ps_ticket_list);
        // チケット要素
        cJSON* ps_ticket_elm = NULL;
        // チケット追加ループ
        ts_ticket_node_t* ps_ticket_node = s_com_ticket_list.ps_ticket_top;
        ts_com_msg_auth_ticket_t* ps_ticket;
        // ステータスダミー
        uint8_t u8_wk_sts_dmy[COM_MSG_SIZE_TICKET_STS];
        memset(u8_wk_sts_dmy, 0x00, COM_MSG_SIZE_TICKET_STS);
        // 編集用ワーク
        char c_wk_edit[45];
        // チケットループ
        while(ps_ticket_node != NULL) {
            //------------------------------------------------------------------
            // cJSONのチケット情報を編集して追加
            //------------------------------------------------------------------
            // チケット
            ps_ticket = &ps_ticket_node->s_ticket;
            // cJSONオブジェクト
            ps_ticket_elm = cJSON_CreateObject();
            // 自デバイスID
            b_vutil_edit_dec_string(c_wk_edit, ps_ticket->u64_own_device_id);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_OWN_DEV_ID, c_wk_edit);
            // リモートデバイスID
            b_vutil_edit_dec_string(c_wk_edit, ps_ticket->u64_rmt_device_id);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RMT_DEV_ID, c_wk_edit);
            // リモートデバイスアドレス
            i_vutil_base64_encode(c_wk_edit, ps_ticket_node->t_rmt_device_bda, COM_TICKET_DEV_BDA_SIZE);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RMT_DEV_BDA, c_wk_edit);
            // リモートデバイス名
            c_wk_edit[0] = '\0';
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RMT_DEV_NAME, c_wk_edit);
            // 暗号鍵
            i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_enc_key, COM_MSG_SIZE_CIPHER_KEY);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_ENC_KEY, c_wk_edit);
            // 自ステータス ※ダミー値を書き出す
#ifdef DEBUG_ALARM
            i_vutil_base64_encode(c_wk_edit, u8_wk_sts_dmy, COM_MSG_SIZE_TICKET_STS);
//            i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS);
#else
            i_vutil_base64_encode(c_wk_edit, u8_wk_sts_dmy, COM_MSG_SIZE_TICKET_STS);
#endif
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_OWN_STS, c_wk_edit);
            // 相手ステータスハッシュ ※ダミー値を書き出す
#ifdef DEBUG_ALARM
            i_vutil_base64_encode(c_wk_edit, u8_wk_sts_dmy, COM_MSG_SIZE_TICKET_STS);
//            i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS);
#else
            i_vutil_base64_encode(c_wk_edit, u8_wk_sts_dmy, COM_MSG_SIZE_TICKET_STS);
#endif
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RMT_HASH, c_wk_edit);
            // 最大シーケンス番号
            b_vutil_edit_dec_string(c_wk_edit, ps_ticket->u32_max_seq_no);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_MAX_SEQ, c_wk_edit);
            // 送信シーケンス番号
            b_vutil_edit_dec_string(c_wk_edit, ps_ticket->u32_tx_seq_no);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_TX_SEQ, c_wk_edit);
            // 受信シーケンス番号
            b_vutil_edit_dec_string(c_wk_edit, ps_ticket->u32_rx_seq_no);
            cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RX_SEQ, c_wk_edit);
            // チケットを追加
            cJSON_AddItemToArray(ps_ticket_list, ps_ticket_elm);

            // 次のチケット
            ps_ticket_node = ps_ticket_node->ps_next;
        }

        //----------------------------------------------------------------------
        // チケットファイルを書き込み
        //----------------------------------------------------------------------
        sts_val = sts_futil_cjson_write_file(COM_PATH_TICKET, ps_ticket_root);
        // cJSON解放
        cJSON_Delete(ps_ticket_root);
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 正常書き込み
    return (sts_val == ESP_OK);
}

/*******************************************************************************
 *
 * NAME: sts_ble_init
 *
 * DESCRIPTION:BluetoothLE初期化処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *  ESP_OK:正常に初期化
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_ble_init() {
    //==========================================================================
    // BLEの初期化処理
    //==========================================================================
    // BLE初期化処理
    esp_err_t sts_val = sts_com_ble_init();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // 電波の出力設定
    sts_val = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // ボンディングデバイス表示
#ifdef DEBUG_ALARM
    sts_val = sts_com_ble_display_bonded_devices();
    if (sts_val != ESP_OK) {
        return sts_val;
    }
#endif

    //==========================================================================
    // GAP セキュリティ マネージャー プロトコル（SMP）設定
    //==========================================================================
    // SMP設定の編集
    ts_com_ble_gap_config_t s_ble_gap_cfg;
    s_ble_gap_cfg.pc_device_name  = s_dev_settings.c_device_name;
    s_ble_gap_cfg.t_auth_req      = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    s_ble_gap_cfg.t_iocap         = ESP_IO_CAP_KBDISP;
    s_ble_gap_cfg.u8_init_key     = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    s_ble_gap_cfg.u8_rsp_key      = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    s_ble_gap_cfg.u8_max_key_size = 16;
    s_ble_gap_cfg.u8_auth_option  = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    s_ble_gap_cfg.v_callback      = v_ble_gap_event_cb;
    // SMP設定
    sts_val = sts_com_ble_gap_smp_init(s_ble_gap_cfg);
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // GAP アドバタイジング設定
    //==========================================================================
    // アドバタイジングデータの設定処理
    sts_val = sts_com_ble_gap_set_adv_data(&gap_adv_config);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：アドバタイズデータの設定エラー
        return sts_val;
    }
    // スキャン応答データの設定処理
    sts_val = sts_com_ble_gap_set_adv_data(&gap_scan_rsp_config);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：スキャン応答データの設定エラー
        return sts_val;
    }

    //==========================================================================
    // GATT SPPサーバー初期設定
    //==========================================================================
    // GATtサーバー初期処理
    sts_com_ble_gatts_init();
    // GATTサーバーのSPPアプリケーション設定の生成処理
    s_gatts_cfg_tbls = s_com_ble_spps_config(ESP_GATT_PERM_READ_ENC_MITM, ESP_GATT_PERM_WRITE_SIGNED_MITM);
    s_gatts_cfg_tbls.u16_app_id = BLE_GATT_APP_ID;          // アプリケーションID
    s_gatts_cfg_tbls.e_con_sec  = ESP_BLE_SEC_ENCRYPT_MITM; // 接続のセキュリティモード
    // アプリケーション設定を登録
    sts_val = sts_com_ble_gatts_app_register(&s_gatts_cfg_tbls);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
        return sts_val;
    }

    //==========================================================================
    // BLEメッセージング機能初期設定(SPPプロファイルを利用)
    //==========================================================================
    // メッセージサーバー初期処理
    sts_val = sts_com_msg_init_svr(BLE_GATT_APP_ID, BLE_MSG_DEVICE_ID, BLE_MSG_MAX_SIZE, v_msg_evt_cb, sts_msg_ticket_cb);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
        return sts_val;
    }
    // ペアリング機能の設定
    v_com_msg_config_pairing(true);
    // ステータスチェック機能の設定
    v_com_msg_config_sts_chk(true);
    // 受信メッセージのエンキュー有効化処理
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_CIPHERTEXT);   // 暗号データ

    //==========================================================================
    // アドバタイジングの開始処理
    //==========================================================================
    sts_val = sts_com_ble_gap_start_advertising(&gap_adv_params);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：アドバタイズ開始
        return sts_val;
    }

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_ble_disconnection
 *
 * DESCRIPTION:BLE切断イベント処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_disconnection() {
    // リモートアドレス取得
    esp_bd_addr_t t_rmt_bda;
    if (sts_com_ble_gap_adv_edit_remote_bda(t_rmt_bda) != ESP_OK) {
        return;
    }
    // 切断開始処理
    if (sts_com_ble_disconnect(t_rmt_bda) != ESP_OK) {
        return;
    }
    // 切断完了待ち
    te_gap_dev_sts_t e_sts = GAP_DEV_STS_DEVICE_NONE;
    do {
        esp_task_wdt_reset();
        e_sts = e_com_ble_gap_device_sts_wait(t_rmt_bda, GAP_DEV_STS_DISCONNECTING, EVT_DISCONNECT_TIMEOUT);
    } while((e_sts & GAP_DEV_STS_DISCONNECTING) == GAP_DEV_STS_DISCONNECTING);
}

/*******************************************************************************
 *
 * NAME: v_ble_gap_event_cb
 *
 * DESCRIPTION:GATTプロファイルのイベントハンドラ関数（SPP用）
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_gap_ble_cb_event_t       e_event     R   GAPイベントタイプ
 * esp_ble_gatts_cb_param_t*    pu_param    R   GATTコールバックパラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_ble_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param) {
    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // ユーザーイベント
    te_usr_event_t e_usr_evt;
    // 承認フラグ
    bool b_accept = false;
    // イベント判定
    switch (e_event) {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // アドバタイズ開始完了通知
        // BLEのGAPプロファイルにおけるPINコードの設定処理
        // ペアリングの際のPINコードは数字６桁の固定値、型はuint32_t
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
#endif
        // パスキー設定
        sts_com_ble_gap_set_static_pass_key(GAP_STATIC_PASSKEYT);
        // コントローラー切断
        v_upd_link_sts(false);
        // メッセージ機能切断
        while (sts_evt_enqueue(EVT_BLE_DISCONNECT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        // 認証完了通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_AUTH_CMPL_EVT");
#endif
        // 認証成功を判定
        e_usr_evt = EVT_BLE_CONNECT;
        if (!pu_param->ble_security.auth_cmpl.success) {
            e_usr_evt = EVT_BLE_CONNECT_ERROR;
        }
        // 認証成功
        while (sts_evt_enqueue(e_usr_evt) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // パスキーの返信要求 ※サーバー側とスキャン側の両方にある
#ifdef DEBUG_ALARM
    {
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_PASSKEY_REQ_EVT");
        tc_com_ble_bda_string_t tc_bda;
        v_com_ble_address_to_str(tc_bda, pu_param->ble_security.ble_req.bd_addr);
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d gap_passkey_reply", __func__, __LINE__);
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d :address = %s", __func__, __LINE__, tc_bda);
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d :passkey = %d", __func__, __LINE__, GAP_STATIC_PASSKEYT);
    }
#endif
        // ペアリング判定
        if ((u16_sensor_sts() & DEV_STS_PAIRING_ENABLED) != 0x00) {
            // ペアリング可能な場合
            b_accept = true;
        } else if (sts_com_ble_bonded_dev(pu_param->ble_security.ble_req.bd_addr) == ESP_OK) {
            // ペアリング済みの場合
            b_accept = true;
        }
        // ※本来はピアデバイスに表示されている値を返信？
        sts_com_ble_gap_passkey_reply(pu_param->ble_security.ble_req.bd_addr, b_accept, GAP_STATIC_PASSKEYT);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // PINコードの確認要求 ※スキャン側にもある
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_NC_REQ_EVT");
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
#endif
        // すべき事：PINコード認証の場合には、相手デバイスに表示されているPINコードの確認結果を返信
        // IOにDisplayYesNO機能があり、ピアデバイスIOにもDisplayYesNo機能がある場合、アプリはこのevtを受け取ります。
        // パスキーをユーザーに表示し、ピアデバイスにて表示される番号と一致するか確認した結果を返信する
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer deivce. */
        // ペアリング判定
        if ((u16_sensor_sts() & DEV_STS_PAIRING_ENABLED) != 0x00) {
            // ペアリング可能な場合
            b_accept = true;
        } else if (sts_com_ble_bonded_dev(pu_param->ble_security.ble_req.bd_addr) == ESP_OK) {
            // ペアリング済みの場合
            b_accept = true;
        }
        // ※本来はピアデバイスに表示されている値と比較して、一致する事を確認した上で返信
        sts_com_ble_gap_confirm_reply(pu_param->ble_security.ble_req.bd_addr, b_accept);
        break;
    default:
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=%d", e_event);
#endif
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_msg_evt_cb
 *
 * DESCRIPTION:メッセージイベントコールバック関数
 *
 * PARAMETERS:              Name        RW  Usage
 * te_com_ble_msg_event     e_msg_evt   R   メッセージイベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_evt_cb(te_com_ble_msg_event e_msg_evt) {
    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // イベント毎の処理
    switch (e_msg_evt) {
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_RX_RESPONSE:
        // 応答を受信
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_RX_RESPONSE");
        break;
#endif
    case COM_BLE_MSG_EVT_RX_RESET:
        // リセットメッセージ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_RX_RESET");
#endif
        // メッセージ機能接続イベント
        while (sts_evt_enqueue(EVT_MSG_CONNECT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_RX_PING:
        // PINGメッセージ受信
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_RX_PING");
        break;
    case COM_BLE_MSG_EVT_RX_DATA:
        // データメッセージ受信
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_RX_DATA");
        break;
#endif
    case COM_BLE_MSG_EVT_RX_CIPHERTEXT:
        // 暗号メッセージ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_RX_CIPHERTEXT");
#endif
        while (sts_evt_enqueue(EVT_MSG_RX_DATA) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_GATT_CONNECT:
        // GATT接続
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_GATT_CONNECT");
        break;
    case COM_BLE_MSG_EVT_GATT_DISCONNECT:
        // GATT切断
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_GATT_DISCONNECT");
        break;
    case COM_BLE_MSG_EVT_OPEN_SUCCESS:
        // オープン成功
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_OPEN_SUCCESS");
        break;
#endif
    case COM_BLE_MSG_EVT_OPEN_TIMEOUT:
        // オープンタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_OPEN_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_START:
        // ペアリング開始
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_PAIRING_START");
#endif
        // ペアリング確認用コードチェック
        while (sts_evt_enqueue(EVT_MSG_PAIR_CD_CHK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_SUCCESS:
        // ペアリング成功
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_PAIRING_SUCCESS");
#endif
        while (sts_evt_enqueue(EVT_MSG_PAIR_OK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_ERR:
        // ペアリングエラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_PAIRING_ERR");
#endif
        while (sts_evt_enqueue(EVT_MSG_PAIR_ERROR) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_TIMEOUT:
        // ペアリングタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_PAIRING_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_STATUS_CHK:
        // ステータスチェック
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_STATUS_CHK");
        break;
#endif
    case COM_BLE_MSG_EVT_STATUS_OK:
        // ステータスチェック正常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_STATUS_OK");
#endif
        // コントローラーリンク
        v_upd_link_sts(true);
        // MSG：ステータス正常
        while (sts_evt_enqueue(EVT_MSG_STS_OK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_STATUS_ERR:
        // ステータスチェック異常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_STATUS_ERR");
#endif
        // MSG：ステータス異常警報
        while (sts_evt_enqueue(EVT_MSG_STS_ERROR) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_STATUS_TIMEOUT:
        // ステータスチェックタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_STATUS_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_HANDLING_ERR:
        // メッセージハンドリングエラー
        ESP_LOGI(LOG_MSG_TAG, "Evt=COM_BLE_MSG_EVT_HANDLING_ERR");
        break;
#endif
    default:
        // その他のイベント
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "Evt=%d", e_msg_evt);
#endif
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_cb
 *
 * DESCRIPTION:チケットアクセスコールバック関数
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
static esp_err_t sts_msg_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket) {
    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;
    // イベント毎の処理
    switch (e_evt) {
    case COM_BLE_MSG_TICKET_EVT_CREATE:
        // チケット生成
        sts_val = sts_msg_ticket_create(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_READ:
        // チケット読み込み
        sts_val = sts_msg_ticket_read(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_UPDATE:
        // チケット更新
        sts_val = sts_msg_ticket_update(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_DELETE:
        // チケット削除
        sts_val = sts_msg_ticket_delete(ps_ticket);
        break;
    default:
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 正常終了
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_create
 *
 * DESCRIPTION:BLEメッセージチケット生成処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:※イベントコールバック
 *
 ******************************************************************************/
static esp_err_t sts_msg_ticket_create(ts_com_msg_auth_ticket_t* ps_ticket) {
    // GAPデバイス情報取得
    esp_bd_addr_t t_rmt_bda;
    if (sts_com_ble_gap_adv_edit_remote_bda(t_rmt_bda) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    // チケットノード検索
    ts_ticket_node_t* ps_ticket_node = ps_msg_ticket_get_node(ps_ticket->u64_rmt_device_id);
    // チケットの追加判定
    if (ps_ticket_node == NULL) {
        ps_ticket_node = pv_mem_malloc(sizeof(ts_ticket_node_t));
        ps_ticket_node->ps_next = NULL;
        // チケット追加
        if (s_com_ticket_list.ps_ticket_top == NULL) {
            s_com_ticket_list.ps_ticket_top  = ps_ticket_node;
            s_com_ticket_list.ps_ticket_tail = ps_ticket_node;
        } else {
            s_com_ticket_list.ps_ticket_tail->ps_next = ps_ticket_node;
            s_com_ticket_list.ps_ticket_tail = ps_ticket_node;
        }
    }
    // チケットの編集
    v_com_ble_addr_cpy(ps_ticket_node->t_rmt_device_bda, t_rmt_bda);
    ps_ticket_node->s_ticket = *ps_ticket;
    // チケットファイルに書き込み
    if (!b_write_ticket_file()) {
        // エラーステータスを返信
        return ESP_FAIL;
    }

    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_read
 *
 * DESCRIPTION:BLEメッセージチケット参照処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:※イベントコールバック
 *
 ******************************************************************************/
static esp_err_t sts_msg_ticket_read(ts_com_msg_auth_ticket_t* ps_ticket) {
    // チケットノード検索
    ts_ticket_node_t* ps_ticket_node = ps_msg_ticket_get_node(ps_ticket->u64_rmt_device_id);
    if (ps_ticket_node == NULL) {
        // チケットが見つからない
        return ESP_ERR_NOT_FOUND;
    }
    // 対象を編集
    *ps_ticket = ps_ticket_node->s_ticket;
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_update
 *
 * DESCRIPTION:BLEメッセージチケット更新処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:※イベントコールバック
 *
 ******************************************************************************/
static esp_err_t sts_msg_ticket_update(ts_com_msg_auth_ticket_t* ps_ticket) {
    // チケットノード検索
    ts_ticket_node_t* ps_ticket_node = ps_msg_ticket_get_node(ps_ticket->u64_rmt_device_id);
    if (ps_ticket_node == NULL) {
        // チケットが見つからない
        return ESP_ERR_NOT_FOUND;
    }
    // チケットの更新
    ps_ticket_node->s_ticket = *ps_ticket;
    // チケットファイルに書き込み
    if (!b_write_ticket_file()) {
        return ESP_FAIL;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_delete
 *
 * DESCRIPTION:BLEメッセージチケット削除処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * ts_com_msg_auth_ticket_t*    ps_ticket   R   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:※イベントコールバック
 *
 ******************************************************************************/
static esp_err_t sts_msg_ticket_delete(ts_com_msg_auth_ticket_t* ps_ticket) {
    // 対象のデバイスID
    uint64_t u64_device_id = ps_ticket->u64_rmt_device_id;
    // チケットノード探索
    ts_ticket_node_t* ps_bef_node = NULL;
    ts_ticket_node_t* ps_chk_node = s_com_ticket_list.ps_ticket_top;
    ts_com_msg_auth_ticket_t* ps_chk_ticket = NULL;
    while (ps_chk_node != NULL) {
        ps_chk_ticket = &ps_chk_node->s_ticket;
        if (ps_chk_ticket->u64_rmt_device_id == u64_device_id) {
            break;
        }
        // 次ノード
        ps_bef_node = ps_chk_node;
        ps_chk_node = ps_chk_node->ps_next;
    }
    // 探索結果判定
    if (ps_chk_node == NULL) {
        // チケットノードが見つからない
        return ESP_ERR_NOT_FOUND;
    }
    // リンクリストから切り離し
    if (s_com_ticket_list.ps_ticket_top == ps_chk_node) {
        s_com_ticket_list.ps_ticket_top = ps_chk_node->ps_next;
    } else {
        ps_bef_node->ps_next = ps_chk_node->ps_next;
    }
    // ボンディング情報を削除
#ifdef DEBUG_ALARM
    ESP_LOGE(LOG_MSG_TAG, "Disbonding!!!");
#endif
    sts_com_ble_disbonding(ps_chk_node->t_rmt_device_bda);
    // チケットノードの解放
    l_mem_free(ps_chk_node);
    // チケットファイルに書き込み
    if (!b_write_ticket_file()) {
        return ESP_FAIL;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_msg_ticket_get_node
 *
 * DESCRIPTION:BLEメッセージチケットノードの取得処理
 *
 * PARAMETERS:  Name            RW  Usage
 * uint64_t     u64_device_id   R   対象のデバイスID
 *
 * RETURNS:
 *   ts_ticket_node_t: 検索結果のチケットノード
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_ticket_node_t* ps_msg_ticket_get_node(uint64_t u64_device_id) {
    ts_ticket_node_t* ps_ticket_node = s_com_ticket_list.ps_ticket_top;
    ts_com_msg_auth_ticket_t* ps_ticket;
    while (ps_ticket_node != NULL) {
        ps_ticket = &ps_ticket_node->s_ticket;
        if (ps_ticket->u64_rmt_device_id == u64_device_id) {
            break;
        }
        ps_ticket_node = ps_ticket_node->ps_next;
    }
    // 検索結果ノード
    return ps_ticket_node;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_edit_info
 *
 * DESCRIPTION:BLEメッセージチケット情報の編集処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * uint32_t                     u32_idx     R   チケットインデックス
 * esp_bd_addr_t                t_bda       W   リモートアドレス
 * ts_com_msg_auth_ticket_t*    ps_info     W   チケット情報
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_ticket_edit_info(uint32_t u32_idx, esp_bd_addr_t t_bda, ts_com_msg_auth_ticket_t* ps_info) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // チケット選択
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;
    ts_ticket_node_t* ps_ticket_node = s_com_ticket_list.ps_ticket_top;
    uint32_t u32_pos = 0;
    while (ps_ticket_node != NULL) {
        // インデックス判定
        if (u32_pos < u32_idx) {
            // 次のノードへ
            ps_ticket_node = ps_ticket_node->ps_next;
            u32_pos++;
            continue;
        }
        // BLEアドレス
        v_com_ble_addr_cpy(t_bda, ps_ticket_node->t_rmt_device_bda);
        // チケット参照
        *ps_info = ps_ticket_node->s_ticket;
        // ステータス更新
        sts_val = ESP_OK;
        break;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果ステータス
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_msg_pairing_check_code_edit
 *
 * DESCRIPTION:BLEペアリング確認用コードの編集処理
 *
 * PARAMETERS:      Name        RW  Usage
 * char*            pc_code     W   メッセージコードの編集対象
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_pairing_check_code_edit(char* pc_code) {
    // 公開鍵のペアを取得
    uint8_t u8_cli_key[BLE_MSG_PUBLIC_KEY_CLI_SIZE];
    uint8_t u8_svr_key[BLE_MSG_PUBLIC_KEY_SVR_SIZE];
    esp_err_t sts_val = sts_com_msg_edit_public_key_pair(u8_cli_key, u8_svr_key);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // 元トークンを生成
    uint8_t u8_wk_token[BLE_MSG_CHECK_CODE_SIZE];
    int i_idx;
    for (i_idx = 0; i_idx < BLE_MSG_CHECK_CODE_SIZE; i_idx++) {
        u8_wk_token[i_idx] = u8_cli_key[i_idx + 4] ^ u8_svr_key[i_idx + 1];
    }
    // Base64に変換
    i_vutil_base64_encode(pc_code, u8_wk_token, BLE_MSG_CHECK_CODE_SIZE);
    // 結果ステータス
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_unpairing
 *
 * DESCRIPTION:BLEペアリング解消処理
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_unpairing() {
    //==========================================================================
    // 初期処理
    //==========================================================================
    // 接続先の有無を判定
    if (b_com_ble_addr_clear(s_com_status.t_rmt_bda)) {
        return ESP_ERR_INVALID_STATE;
    }
    // リモートBLEアドレス
    esp_bd_addr_t t_bda;
    v_com_ble_addr_cpy(t_bda, s_com_status.t_rmt_bda);
    // リモートデバイスID
    uint64_t u64_device_id = s_com_status.u64_rmt_device_id;
    // ネットワーク切断
    v_ble_disconnection();

    //==========================================================================
    // ペアリング解除
    //==========================================================================
    // リモートデバイスのデバイスID判定
    esp_err_t sts_val = ESP_OK;
    if (u64_device_id == s_dev_settings.u64_device_id) {
        // ボンディング解除
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "Disbonding!!!");
#endif
        sts_val = sts_com_ble_disbonding(s_com_status.t_rmt_bda);
    } else {
        // チケット削除
        sts_val = sts_com_msg_delete_ticket(u64_device_id);
    }
    // リモートBLEアドレスクリア
    v_com_ble_addr_clear(s_com_status.t_rmt_bda);
    // リモートデバイスIDクリア
    s_com_status.u64_rmt_device_id = s_dev_settings.u64_device_id;
    // セキュアコネクトフラグをオフに更新
    s_com_status.b_secure_connect = false;
    // ペアリング確認用コードクリア
    memset(s_com_status.c_pair_chk_code, 0x00, BLE_MSG_CODE_SIZE + 1);
    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_tx_ctrl_msg
 *
 * DESCRIPTION:制御メッセージの送信処理
 *
 * PARAMETERS:              Name        RW  Usage
 * te_msg_ctrl_cmd_t        e_cmd       R   制御コマンド
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_tx_ctrl_msg(te_msg_ctrl_cmd_t e_cmd) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // コンテキスト取得
    //==========================================================================
    uint64_t u64_rmt_device_id = s_com_status.u64_rmt_device_id;
    te_msg_operating_mode_t e_operating_mode = s_com_status.e_operate_mode;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // 送信データの編集
    //==========================================================================
    uint8_t u8_data[2];
    u8_data[0] = e_cmd;
    if (e_cmd != CTL_CMD_NACK) {
        u8_data[1] = e_operating_mode;
    } else {
        u8_data[1] = OPR_MODE_NORMAL;
    }
    ts_u8_array_t* ps_data = ps_mdl_create_u8_array(u8_data, 2);
    if (ps_data == NULL) {
        return ESP_ERR_NO_MEM;
    }

    //==========================================================================
    // 暗号メッセージの送信処理
    //==========================================================================
    esp_err_t sts_val = sts_com_msg_tx_cipher_msg(u64_rmt_device_id, ps_data);
    // 送信データの削除
    sts_mdl_delete_u8_array(ps_data);
    // 結果返却
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_rx_ctrl_msg
 *
 * DESCRIPTION:制御メッセージの受信処理
 *
 * PARAMETERS:      Name                    RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_rx_ctrl_msg() {
    // メッセージ受信
    ts_com_msg_t* ps_rx_msg = ps_com_msg_rx_msg(EVT_RX_WAIT_TICK);
    if (ps_rx_msg == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_FAIL;
    // 受信した制御メッセージを初期化
    ts_ctrl_msg_t* ps_ctrl_msg = &s_com_status.s_ctrl_msg;
    v_com_ble_addr_clear(ps_ctrl_msg->t_bda);
    ps_ctrl_msg->e_cmd  = CTL_CMD_COUNT;
    ps_ctrl_msg->e_mode = OPR_MODE_COUNT;
    do {
        // ステータスチェック済みか判定
        if (!s_com_status.b_secure_connect) {
            break;
        }
        // メッセージタイプをチェック
        if (ps_rx_msg->e_type != COM_BLE_MSG_TYP_CIPHERTEXT) {
            break;
        }
        // メッセージサイズ
        ts_u8_array_t* ps_data = ps_rx_msg->ps_data;
        if (ps_data->t_size != BLE_MSG_REC_SIZE) {
            break;
        }
        // 制御コマンド
        if (ps_data->pu8_values[0] >= CTL_CMD_COUNT) {
            break;
        }
        // 動作モード
        if (ps_data->pu8_values[1] >= OPR_MODE_COUNT) {
            break;
        }
        // 結果を編集
        // リモートデバイスのアドレス管理を行う
        v_com_ble_addr_cpy(ps_ctrl_msg->t_bda, ps_rx_msg->t_rcv_bda);
        ps_ctrl_msg->e_cmd  = ps_data->pu8_values[0];
        ps_ctrl_msg->e_mode = ps_data->pu8_values[1];
        // 正常終了
        sts_val = ESP_OK;
    } while(false);
    // 受信メッセージを解放
    sts_com_msg_delete_msg(ps_rx_msg);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_evt_enqueue
 *
 * DESCRIPTION:イベントのエンキュー処理
 *
 * PARAMETERS:      Name            RW  Usage
 * te_com_event_t   e_evt           R   イベント種別
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static esp_err_t sts_evt_enqueue(te_usr_event_t e_evt) {
    //==========================================================================
    // イベントのエンキュー処理
    //==========================================================================
    if (xQueueSend(s_evt_queue, &e_evt, 0) != pdPASS) {
        return ESP_FAIL;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_evt_dequeue
 *
 * DESCRIPTION:イベントのデキュー処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 * te_com_event_t*  pe_evt          R   イベント種別
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static esp_err_t sts_evt_dequeue(ts_com_event_info_t* ps_evt) {
    //==========================================================================
    // イベントのデキュー
    //==========================================================================
    if (xQueueReceive(s_evt_queue, &ps_evt->e_event, EVT_ENQUEUE_WAIT_TICK) != pdTRUE) {
        return ESP_FAIL;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // イベント初期化
    //==========================================================================
    // 動作モード
    ps_evt->e_operating_mode = s_com_status.e_operate_mode;
    // デバイスステータス
    ps_evt->u16_device_sts = s_com_status.u16_device_sts;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: b_evt_chk_timeout
 *
 * DESCRIPTION:タイムアウト判定
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   bool:タイムアウト判定結果
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static bool b_evt_chk_timeout() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // タイムアウト判定
    //==========================================================================
    bool b_timeout = (s_com_status.i64_timeout_ms < xTaskGetTickCountMSec());

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 正常終了
    return b_timeout;
}

/*******************************************************************************
 *
 * NAME: v_evt_set_timeout
 *
 * DESCRIPTION:タイムアウト時間の設定
 *
 * PARAMETERS:      Name            RW  Usage
 *   int64_t        i64_timeout_ms  R   タイムアウトまでの時間（ミリ秒）
 *
 * RETURNS:
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static void v_evt_set_timeout(int64_t i64_timeout_ms) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // タイムアウト設定
    //==========================================================================
    s_com_status.i64_timeout_ms = xTaskGetTickCountMSec() + i64_timeout_ms;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_evt_clear_timeout
 *
 * DESCRIPTION:タイムアウト時間のクリア
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static void v_evt_clear_timeout() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // タイムアウト設定
    //==========================================================================
    s_com_status.i64_timeout_ms = MAX_VALUE_INT64;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_upd_link_sts
 *
 * DESCRIPTION:接続ステータスの更新処理
 *
 * PARAMETERS:      Name       RW  Usage
 * bool             b_linked   R   リンクステータス
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_upd_link_sts(bool b_linked) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // リンクステータスの更新
    //==========================================================================
    if (b_linked) {
        // コントローラ接続
        s_com_status.u16_device_sts |= DEV_STS_CONTROLLER_LINK;
    } else {
        // コントローラ切断
        s_com_status.u16_device_sts &= ~DEV_STS_CONTROLLER_LINK;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: u16_sensor_sts
 *
 * DESCRIPTION:センサーステータス参照
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static uint16_t u16_sensor_sts() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // センサーステータスの更新
    //==========================================================================
    uint16_t u16_sensor_sts = s_com_status.u16_device_sts;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return u16_sensor_sts;
}

/*******************************************************************************
 *
 * NAME: b_upd_sensor_sts
 *
 * DESCRIPTION:センサーステータス更新
 *
 * PARAMETERS:      Name            RW  Usage
 * uint16_t         u16_sensor_sts  R   センサーステータス
 *
 * RETURNS:
 *   true:ステータス変更が有った場合
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_upd_sensor_sts(uint16_t u16_sensor_sts) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // センサーステータスの更新
    //==========================================================================
    // 動作モード
    te_msg_operating_mode_t e_operate_mode = s_com_status.e_operate_mode;
    // デバイスステータス（新）
    uint16_t u16_device_sts_new = s_com_status.u16_device_sts;
    // 動作モード判定
    if (e_operate_mode == OPR_MODE_ALARM) {
        // 警報モード：ペアリング有効と加速度のみを反映
        // ステータスクリア
        u16_device_sts_new &= ~DEV_STS_MASK_ALARM;
        // ステータス更新
        u16_device_sts_new |= (u16_sensor_sts & DEV_STS_MASK_ALARM);
    } else {
        // 警報モード以外：センサーステータス全般を反映
        // ステータスクリア
        u16_device_sts_new &= ~DEV_STS_MASK_SENSOR;
        // ステータス更新
        u16_device_sts_new |= (u16_sensor_sts & DEV_STS_MASK_SENSOR);
    }
    // ステータス更新判定
    bool b_sts_update = (u16_device_sts_new != s_com_status.u16_device_sts);
    // ステータス更新
    s_com_status.u16_device_sts = u16_device_sts_new;

    //==========================================================================
    // 警報判定
    //==========================================================================
    if (e_operate_mode == OPR_MODE_ALERT && (u16_device_sts_new & DEV_STS_MASK_ALARM_CHECK) != 0x00) {
        // 警報モードへ移行
        v_set_alarm_mode();
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return b_sts_update;
}

/*******************************************************************************
 *
 * NAME: e_get_operating_mode
 *
 * DESCRIPTION:動作モードの取得
 *
 * PARAMETERS:              Name        RW  Usage
 *
 * RETURNS:
 * te_msg_operating_mode_t:動作モード
 *
 * NOTES:
 * None.
 ******************************************************************************/
static te_msg_operating_mode_t e_get_operating_mode() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // 動作モード取得
    //==========================================================================
    te_msg_operating_mode_t e_operating_mode = s_com_status.e_operate_mode;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 動作モード返信
    return e_operating_mode;
}

/*******************************************************************************
 *
 * NAME: b_set_operating_mode
 *
 * DESCRIPTION:動作モードの更新
 *
 * PARAMETERS:              Name        RW  Usage
 * te_msg_operating_mode_t  e_mode      R   動作モード
 *
 * RETURNS:
 *   true: モード変更（警報モードに移行）
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_set_operating_mode(te_msg_operating_mode_t e_mode) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (e_mode >= OPR_MODE_COUNT) {
        return false;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return false;
    }

    //==========================================================================
    // センサーステータスの更新
    //==========================================================================
    bool b_result = false;
    // 動作モード判定
    if (s_com_status.e_operate_mode != OPR_MODE_ALARM) {
        // 動作モード更新
        s_com_status.e_operate_mode = e_mode;
        // 結果更新
        b_result = true;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return b_result;
}

/*******************************************************************************
 *
 * NAME: v_set_alarm_mode
 *
 * DESCRIPTION:警報モード設定
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_set_alarm_mode() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // リモートステータスエラー処理
    //==========================================================================
    // 警報モードに移行
    s_com_status.e_operate_mode = OPR_MODE_ALARM;
    // ステータス更新
    s_com_status.u16_device_sts |= DEV_STS_REMOTE_STS_ERR;

    //==========================================================================
    // ステータスクリア
    //==========================================================================
    ts_ticket_node_t* ps_ticket_node = s_com_ticket_list.ps_ticket_top;
    ts_com_msg_auth_ticket_t* ps_ticket;
    while (ps_ticket_node != NULL) {
        ps_ticket = &ps_ticket_node->s_ticket;
        // ステータスクリア
        memset(ps_ticket->u8_own_sts, 0x00, COM_MSG_SIZE_TICKET_STS);
        memset(ps_ticket->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
        // 次のノードへ
        ps_ticket_node = ps_ticket_node->ps_next;
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_task_event
 *
 * DESCRIPTION:イベント処理
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          args            R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_event(void* args) {
    //==========================================================================
    // WDTにタスクを登録
    //==========================================================================
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    //==========================================================================
    // イベントループ
    //==========================================================================
    esp_err_t sts_evt_val;
    ts_com_event_info_t s_evt = {
        .e_event = EVT_COUNT,
        .e_operating_mode = OPR_MODE_NORMAL,
        .u16_device_sts = 0x0000
    };
    ts_scr_status_t* ps_scr_sts;
    while (true) {
        //----------------------------------------------------------------------
        // WDTリセット
        //----------------------------------------------------------------------
        esp_task_wdt_reset();
        vTaskDelay(1);

        //----------------------------------------------------------------------
        // イベントチェック
        //----------------------------------------------------------------------
        sts_evt_val = sts_evt_dequeue(&s_evt);

        //----------------------------------------------------------------------
        // タイムアウト処理
        //----------------------------------------------------------------------
        if (b_evt_chk_timeout()) {
#ifdef DEBUG_ALARM
            ESP_LOGE(LOG_MSG_TAG, "%s L#%d", __func__, __LINE__);
#endif
            // タイムアウトクリア
            v_evt_clear_timeout();
            // タイムアウトイベント
            ts_com_event_info_t s_timeout_evt = {
                .e_event          = EVT_TIMEOUT,
                .e_operating_mode = s_evt.e_operating_mode,
                .u16_device_sts   = s_evt.u16_device_sts,
            };
            // タイムアウト処理
            v_evt_common(&s_timeout_evt);
        }

        //----------------------------------------------------------------------
        // イベント取得の有無を判定
        //----------------------------------------------------------------------
        if (sts_evt_val != ESP_OK) {
            continue;
        }

        //----------------------------------------------------------------------
        // 共通イベント処理
        //----------------------------------------------------------------------
        v_evt_common(&s_evt);

        //----------------------------------------------------------------------
        // 画面イベント
        //----------------------------------------------------------------------
        // 画面イベント処理
        ps_scr_sts = &s_scr_sts_list[s_com_status.e_scr_id];
        ps_scr_sts->pf_evt_cb(&s_evt);
    }

    //==========================================================================
    // タスク削除
    //==========================================================================
    vTaskDelete(NULL);

}

/*******************************************************************************
 *
 * NAME: v_task_timer_event
 *
 * DESCRIPTION:タイマーイベント処理タスク
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          args            R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_timer_event(void* args) {
    //==========================================================================
    // WDTにタスクを登録
    //==========================================================================
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    //==========================================================================
    // 入力チェックループ
    //==========================================================================
    // 動作モード
    te_msg_operating_mode_t e_ope_mode_now = OPR_MODE_NORMAL;
    te_msg_operating_mode_t e_ope_mode_bef = OPR_MODE_NORMAL;
    // イベント
    te_usr_event_t e_evt_input_now = EVT_COUNT;
    te_usr_event_t e_evt_input_bef = EVT_COUNT;
    // センサーステータス
    uint16_t u16_sensor_sts = 0x00;
    // 電圧
    int i_voltage = 0;
    // 次回実行時刻
    int64_t i64_next_msec = xTaskGetTickCountMSec();
    // 入力チェックループ
    while (true) {
        //----------------------------------------------------------------------
        // WDTリセット
        //----------------------------------------------------------------------
        esp_task_wdt_reset();
        vTaskDelay(1);

        //----------------------------------------------------------------------
        // ウェイト処理
        //----------------------------------------------------------------------
        // 指定時刻までディレイ（ミリ秒単位）
        i64_dtm_delay_until_msec(i64_next_msec);
        i64_next_msec = xTaskGetTickCountMSec() + COM_TIMER_TASK_WAIT_MSEC;
        i64_next_msec = i64_next_msec - (i64_next_msec % COM_TIMER_TASK_WAIT_MSEC);

        //----------------------------------------------------------------------
        // センサー情報チェック
        //----------------------------------------------------------------------
        // センサーチェック
        u16_sensor_sts = u16_sensor_sts_read();

        //----------------------------------------------------------------------
        // センサーイベント処理
        //----------------------------------------------------------------------
        // センサーステータス更新判定
        if (b_upd_sensor_sts(u16_sensor_sts)) {
            // センサーステータス更新
            te_usr_event_t e_evt_sensor = EVT_SENSOR_UPDATE;
            // 警報モードへの移行済み判定
            e_ope_mode_bef = e_ope_mode_now;
            e_ope_mode_now = e_get_operating_mode();
            if (e_ope_mode_now == OPR_MODE_ALARM && e_ope_mode_now != e_ope_mode_bef) {
                // センサー検知警報
                e_evt_sensor = EVT_SENSOR_ERROR;
            }
            // イベント通知
#ifdef DEBUG_ALARM
            uint16_t u16_disp_sts = 0x0000;
            if ((u16_sensor_sts & DEV_STS_PORT1_RADAR) != 0x00) {
                u16_disp_sts |= 0x1000;
            }
            if ((u16_sensor_sts & DEV_STS_PORT1_MOTION) != 0x00) {
                u16_disp_sts |= 0x0100;
            }
            if ((u16_sensor_sts & DEV_STS_PORT2_RADAR) != 0x00) {
                u16_disp_sts |= 0x0010;
            }
            if ((u16_sensor_sts & DEV_STS_PORT2_MOTION) != 0x00) {
                u16_disp_sts |= 0x0001;
            }
            ESP_LOGW(LOG_MSG_TAG, "%s L#%d sensor=%04x:%04x", __func__, __LINE__, u16_disp_sts, s_com_status.u16_device_sts);
#endif
            while (sts_evt_enqueue(e_evt_sensor) != ESP_OK) {
                vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
            }
        }

        //----------------------------------------------------------------------
        // ５方向スイッチ
        //----------------------------------------------------------------------
        i_voltage = i_adc_oneshot_voltage(ps_adc_ctx, COM_5WAY_CHANNEL);
        if (i_voltage < 0) {
            continue;
        }

        //----------------------------------------------------------------------
        // 入力キー判定
        //----------------------------------------------------------------------
        // 未入力チェック
        if (i_voltage > VOLTAGE_THRESHOID_NONE) {
            // 入力履歴初期化
            e_evt_input_bef = EVT_COUNT;
            continue;
        }
        // 入力チェック
        if (i_voltage > VOLTAGE_THRESHOID_PUSH) {
            // 入力：プッシュ
            e_evt_input_now = EVT_INPUT_PUSH;
        } else if (i_voltage > VOLTAGE_THRESHOID_LEFT) {
            // 入力：左
            e_evt_input_now = EVT_INPUT_LEFT;
        } else if (i_voltage > VOLTAGE_THRESHOID_UP) {
            // 入力：上
            e_evt_input_now = EVT_INPUT_UP;
        } else if (i_voltage > VOLTAGE_THRESHOID_RIGHT) {
            // 入力：右
            e_evt_input_now = EVT_INPUT_RIGHT;
        } else {
            // 入力：下
            e_evt_input_now = EVT_INPUT_DOWN;
        }

        //----------------------------------------------------------------------
        // 入力イベントエンキュー
        //----------------------------------------------------------------------
        // 入力履歴との比較
        if (e_evt_input_now != e_evt_input_bef) {
            while (sts_evt_enqueue(e_evt_input_now) != ESP_OK) {
                vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
            }
            // 入力履歴更新
            e_evt_input_bef = e_evt_input_now;
        }
    }

    //==========================================================================
    // タスク削除
    //==========================================================================
    vTaskDelete(NULL);
}

/*******************************************************************************
 *
 * NAME: v_evt_common
 *
 * DESCRIPTION:共通イベント処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_common(ts_com_event_info_t* ps_evt) {
    //==========================================================================
    // 共通イベント処理
    // ※通信系イベントについてはここで処理を行う
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_FAIL;
    // イベント処理
    switch (ps_evt->e_event) {
#ifdef DEBUG_ALARM
    case EVT_SCR_INIT:
        // 画面：初期処理
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_SCR_INIT");
        break;
#endif
    case EVT_BLE_CONNECT:
        // BLE：接続通知
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_CONNECT");
#endif
        // セキュアコネクトフラグをオフに更新
        s_com_status.b_secure_connect = false;
        // リモートデバイスアドレス取得
        sts_com_ble_gap_adv_edit_remote_bda(s_com_status.t_rmt_bda);
        // ステータスチェック完了までのタイムアウト時間を設定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "v_evt_set_timeout(EVT_CONNECTION_TIMEOUT_MS)");
#endif
        v_evt_set_timeout(EVT_CONNECTION_TIMEOUT_MS);
#ifdef DEBUG_ALARM
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
#endif
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_CONNECT_ERROR");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ネットワーク切断
        v_ble_disconnection();
        // メッセージ表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_CONNECT);
        break;
    case EVT_BLE_DISCONNECT:
        // BLE:切断通知
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_DISCONNECT");
#endif
        // タイムアウト時間をリセット
        v_evt_clear_timeout();
        // ペアリングエラー判定
        if (s_com_status.e_scr_id == SCR_ID_PAIRING_CHECK) {
            // ペアリング中に切断した場合はペアリング解除する
            sts_msg_unpairing();
            // メッセージ表示後にステータス表示画面に遷移
            v_evt_show_msg(COM_MSG_ID_ERR_PAIRING);
            break;
        }
        // リモートBLEアドレスクリア
        v_com_ble_addr_clear(s_com_status.t_rmt_bda);
        // 接続デバイスIDクリア
        s_com_status.u64_rmt_device_id = s_dev_settings.u64_device_id;
        // セキュアコネクトフラグをオフに更新
        s_com_status.b_secure_connect = false;
        // ペアリング確認用コードクリア
        memset(s_com_status.c_pair_chk_code, 0x00, BLE_MSG_CODE_SIZE + 1);
#ifdef DEBUG_ALARM
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
#endif
        break;
    case EVT_MSG_CONNECT:
        // MSG:メッセージ機能接続
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_CONNECT");
#endif
        // リモートデバイスIDを取得
        sts_com_msg_edit_remote_dev_id(&s_com_status.u64_rmt_device_id);
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_PAIR_CD_CHK");
#endif
        // ペアリング機能の無効判定
        if ((ps_evt->u16_device_sts & DEV_STS_PAIRING_ENABLED) == 0x00) {
            // タイムアウト時間をクリア
            v_evt_clear_timeout();
            // ペアリングNGとする
            sts_com_msg_tx_pairing_certification(false, 0xFFFFFFFF);
            // ペアリング中に切断した場合はペアリング解除する
            sts_msg_unpairing();
            // メッセージ表示後にステータス表示画面に遷移
            v_evt_show_msg(COM_MSG_ID_ERR_PAIRING);
            break;
        }
        // タイムアウト時間を更新
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "v_evt_set_timeout(EVT_PAIRING_TIMEOUT_MS)");
#endif
        v_evt_set_timeout(EVT_PAIRING_TIMEOUT_MS);
        // ペアリングチェック画面への画面遷移
        v_evt_screen_change(SCR_ID_PAIRING_CHECK);
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_PAIR_OK");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ペアリング機能の無効判定
        if ((ps_evt->u16_device_sts & DEV_STS_PAIRING_ENABLED) == 0x00) {
            // ペアリング中に切断した場合はペアリング解除する
            sts_msg_unpairing();
            // メッセージ表示画面に遷移
            v_evt_show_msg(COM_MSG_ID_ERR_PAIRING);
            break;
        }
        // ステータス画面への画面遷移
        v_evt_screen_change(SCR_ID_STATUS_DISPLAY);
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_PAIR_ERR");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ペアリング中に切断した場合はペアリング解除する
        sts_msg_unpairing();
        // メッセージ表示後にステータス表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_PAIRING);
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_STS_OK");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // セキュアコネクトフラグをオンに更新
        s_com_status.b_secure_connect = true;
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータスエラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_STS_ALARM");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ネットワーク切断
        v_ble_disconnection();
        // アラームモードに移行
        v_set_alarm_mode();
        // メッセージ表示後にステータス表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_STATUS_CHK);
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_RX_DATA");
#endif
        // メッセージ受信
        sts_val = sts_rx_ctrl_msg();
        if (sts_val != ESP_OK || !s_com_status.b_secure_connect) {
            // ネットワーク切断
            v_ble_disconnection();
            // エラーメッセージ表示後にステータス表示画面に遷移
            v_evt_show_msg(COM_MSG_ID_ERR_TXRX);
            break;
        }
        // 受信データ処理
        v_evt_exec_command(ps_evt);
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_RX_ERROR");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ネットワーク切断
        v_ble_disconnection();
        // エラーメッセージ表示後にステータス表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_TXRX);
        break;
    case EVT_TIMEOUT:
        // タイムアウト
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_TIMEOUT");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ネットワーク切断
        v_ble_disconnection();
        // エラーメッセージ表示後にステータス表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_TIMEOUT);
        break;
#ifdef DEBUG_ALARM
    case EVT_SENSOR_UPDATE:
        // センサーステータス更新
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_SENSOR_DETECTION");
        break;
#endif
    case EVT_SENSOR_ERROR:
        // センサーステータスエラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_SENSOR_ERROR");
#endif
        // タイムアウト時間をクリア
        v_evt_clear_timeout();
        // ネットワーク切断
        v_ble_disconnection();
        // メッセージ表示後にステータス表示画面に遷移
        v_evt_show_msg(COM_MSG_ID_ERR_ALARM);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_evt_exec_command
 *
 * DESCRIPTION:受信コマンド実行処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_exec_command(ts_com_event_info_t* ps_evt) {
    // 制御メッセージをチェック
    ts_ctrl_msg_t* ps_ctrl_msg = &s_com_status.s_ctrl_msg;
    // リモートデバイスID
    uint64_t u64_rmt_device_id = s_com_status.u64_rmt_device_id;
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "%s L#%d cmd=%d", __func__, __LINE__, ps_ctrl_msg->e_cmd);
#endif
    // コマンド判定
    switch (ps_ctrl_msg->e_cmd) {
    case CTL_CMD_READ:
        // 動作モード読み込み
        // 結果応答
        sts_tx_ctrl_msg(CTL_CMD_ACK);
        break;
    case CTL_CMD_UPDATE:
        // 動作モード更新
        if (!b_set_operating_mode(ps_ctrl_msg->e_mode)) {
            // エラー応答
            sts_tx_ctrl_msg(CTL_CMD_NACK);
            break;
        }
        // 結果応答
        sts_tx_ctrl_msg(CTL_CMD_ACK);
        break;
    case CTL_CMD_UNPAIR:
        // ペアリング解除
        // ボンディング情報を削除
#ifdef DEBUG_ALARM
    ESP_LOGE(LOG_MSG_TAG, "Disbonding!!!");
#endif
        sts_com_ble_disbonding(ps_ctrl_msg->t_bda);
        // チケット削除
        sts_com_msg_delete_ticket(u64_rmt_device_id);
        break;
    default:
        // 受信エラーを返信
        sts_tx_ctrl_msg(CTL_CMD_NACK);
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_evt_screen_change
 *
 * DESCRIPTION:画面遷移イベント処理
 *
 * PARAMETERS:      Name            RW  Usage
 * te_usr_scr_id_t  e_scr_id        R   遷移先画面ID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_screen_change(te_usr_screen_id_t e_scr_id) {
    // 画面IDの更新
    s_com_status.e_scr_id = e_scr_id;
    // 画面ステータスの初期化
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[e_scr_id];
    ps_scr_status->i_disp_row = 0;                      // 表示位置（行）
    ps_scr_status->e_cursor_type = CURSOR_TYPE_NONE;    // カーソルタイプ
    ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
    ps_scr_status->u8_cursor_col = 0;                   // カーソル位置（列）
    // 画面初期処理イベント
    while (sts_evt_enqueue(EVT_SCR_INIT) != ESP_OK) {
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
}

/*******************************************************************************
 *
 * NAME: v_evt_show_msg
 *
 * DESCRIPTION: メッセージ表示イベント
 *
 * PARAMETERS:          Name        RW  Usage
 * char*                pc_msg_id   R   表示メッセージID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_show_msg(char* pc_msg_id) {
    // メッセージ表示画面に遷移
    strcpy(s_com_status.c_msg_id, pc_msg_id);
    v_evt_screen_change(SCR_ID_MSG_DISPLAY);
}

/*******************************************************************************
 *
 * NAME: v_scr_message_display
 *
 * DESCRIPTION:メッセージ表示画面のイベント処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_message_display(ts_com_event_info_t* ps_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_MSG_DISPLAY];
    // 画面描画フラグ
    bool b_scr_draw = false;
    // イベント判定
    switch (ps_evt->e_event) {
    case EVT_SCR_INIT:
        // 初期処理判定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MSG ID:%s", s_com_status.c_msg_id);
#endif
        // 画面：初期処理
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルタイプ
        ps_scr_status->u8_cursor_row = 1;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 15;                  // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
        break;
    case EVT_BLE_DISCONNECT:
        // BLE：切断通知
        break;
    case EVT_MSG_CONNECT:
        // MSG；接続通知
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータス異常
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
        break;
    case EVT_TIMEOUT:
        // タイムアウト
        break;
    case EVT_SENSOR_UPDATE:
        // センサーステータス更新
        break;
    case EVT_SENSOR_ERROR:
        // センサーステータスエラー
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
        // ステータス表示画面へ画面遷移
        v_evt_screen_change(SCR_ID_STATUS_DISPLAY);
        break;
    default:
        break;
    }

    //==========================================================================
    // 画面描画
    //==========================================================================
    if (b_scr_draw) {
        //----------------------------------------------------------------------
        // 表示メッセージ探索
        //----------------------------------------------------------------------
        ts_msg_info_t* ps_msg = ps_msg_top;
        while (ps_msg != NULL) {
            if (i_vutil_strcmp(ps_msg->c_msg_id, s_com_status.c_msg_id) == 0) {
                break;
            }
            // 次メッセージ
            ps_msg = ps_msg->ps_next;
        }

        //----------------------------------------------------------------------
        // 画面描画バッファの編集
        //----------------------------------------------------------------------
        // バッファクリア
        strcpy(s_lcd_sts.c_buff[0], COM_LCD_EMPTY_LINE);
        strcpy(s_lcd_sts.c_buff[1], COM_LCD_EMPTY_LINE);
        // 表示列判定
        sprintf(s_lcd_sts.c_buff[0], "MSG ID:%s", s_com_status.c_msg_id);
        // エラー判定
        if (ps_msg != NULL) {
            sprintf(s_lcd_sts.c_buff[1], "%s", ps_msg->c_msg);
        } else {
            sprintf(s_lcd_sts.c_buff[1], "%s", COM_LCD_EMPTY_LINE);
        }
        // スペースパディング
        i_vutil_str_rpad(s_lcd_sts.c_buff[0], ' ', COM_LCD_LINE_SIZE);
        i_vutil_str_rpad(s_lcd_sts.c_buff[1], ' ', COM_LCD_LINE_SIZE);

        //----------------------------------------------------------------------
        // カーソル描画バッファ編集
        //----------------------------------------------------------------------
        s_lcd_sts.e_cursor_type = ps_scr_status->e_cursor_type;
        s_lcd_sts.u8_cursor_row = ps_scr_status->u8_cursor_row;
        s_lcd_sts.u8_cursor_col = ps_scr_status->u8_cursor_col;

        //----------------------------------------------------------------------
        // 画面描画
        //----------------------------------------------------------------------
        v_lcd_screen_drawing();
        v_lcd_cursor_drawing();
    }
}

/*******************************************************************************
 *
 * NAME: v_scr_status_display
 *
 * DESCRIPTION:ステータス表示画面
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_status_display(ts_com_event_info_t* ps_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_STATUS_DISPLAY];
    // 画面描画フラグ
    bool b_scr_draw = false;

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (ps_evt->e_event) {
    case EVT_SCR_INIT:
        // 初期処理
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_SCR_INIT");
#endif
        // 画面遷移
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_NONE;    // カーソルウェイト
        ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 0;                   // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_BLE_CONNECT");
#endif
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_BLE_CONNECT_ERROR");
#endif
        break;
    case EVT_BLE_DISCONNECT:
        // BLE：切断通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_DISCONNECT");
#endif
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_CONNECT:
        // MSG：メッセージ機能接続
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_CONNECT");
#endif
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_CD_CHK");
#endif
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_OK");
#endif
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_ERR");
#endif
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_STS_OK");
#endif
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータス異常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_STS_ALARM");
#endif
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_RX_DATA");
#endif
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_RX_ERROR");
#endif
        break;
    case EVT_TIMEOUT:
        // タイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_TIMEOUT");
#endif
        break;
    case EVT_SENSOR_UPDATE:
        // センサーステータス検知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_SENSOR_UPDATE");
#endif
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_SENSOR_ERROR:
        // センサーステータス警報
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_SENSOR_ERROR");
#endif
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_UP");
#endif
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_DOWN");
#endif
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_LEFT");
#endif
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_RIGHT");
#endif
        // チケット選択画面への画面遷移
        v_evt_screen_change(SCR_ID_TICKET_DELETE);
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_PUSH");
#endif
        break;
    default:
        break;
    }

    //==========================================================================
    // 画面描画
    //==========================================================================
    if (b_scr_draw) {
        //----------------------------------------------------------------------
        // 動作モード編集
        //----------------------------------------------------------------------
        switch (e_get_operating_mode()) {
        case OPR_MODE_NORMAL:
            // 通常モード
            strcpy(s_lcd_sts.c_buff[0], "Mode  :Normal   ");
            break;
        case OPR_MODE_ALERT:
            // 警戒モード
            strcpy(s_lcd_sts.c_buff[0], "Mode  :Alert    ");
            break;
        case OPR_MODE_ALARM:
            // 警報モード
            strcpy(s_lcd_sts.c_buff[0], "Mode  :Alarm    ");
            break;
        default:
            // エラー
            strcpy(s_lcd_sts.c_buff[0], "Mode  :Error    ");
            break;
        }

        //----------------------------------------------------------------------
        // ステータス編集
        //----------------------------------------------------------------------
        // デバイスステータス
        uint16_t u16_device_sts = ps_evt->u16_device_sts;
        // 初期化
        strcpy(s_lcd_sts.c_buff[1], "Status:    -    ");
        char* pc_status = &s_lcd_sts.c_buff[1][0];
        // 通信：リモートステータスエラー
        if ((u16_device_sts & DEV_STS_REMOTE_STS_ERR) != 0x00) {
            pc_status[7] = 'E';
        }
        // 通信：コントローラーリンク
        if ((u16_device_sts & DEV_STS_CONTROLLER_LINK) != 0x00) {
            pc_status[8] = 'L';
        }
        // センサー：ペアリング
        if ((u16_device_sts & DEV_STS_PAIRING_ENABLED) != 0x00) {
            pc_status[9] = 'P';
        }
        // センサー：加速度センサー検知
        if ((u16_device_sts & DEV_STS_ACCELERATION) != 0x00) {
            pc_status[10] = 'A';
        }
        // センサー：モジュール１のレーダー検知
        if ((u16_device_sts & DEV_STS_PORT1_RADAR) != 0x00) {
            pc_status[12] = 'R';
        }
        // センサー：モジュール１の動体検知
        if ((u16_device_sts & DEV_STS_PORT1_MOTION) != 0x00) {
            pc_status[13] = 'M';
        }
        // センサー：モジュール２のレーダー検知
        if ((u16_device_sts & DEV_STS_PORT2_RADAR) != 0x00) {
            pc_status[14] = 'R';
        }
        // センサー：モジュール２の動体検知
        if ((u16_device_sts & DEV_STS_PORT2_MOTION) != 0x00) {
            pc_status[15] = 'M';
        }

        //----------------------------------------------------------------------
        // カーソル設定
        //----------------------------------------------------------------------
        s_lcd_sts.e_cursor_type = ps_scr_status->e_cursor_type;
        s_lcd_sts.u8_cursor_row = ps_scr_status->u8_cursor_row;
        s_lcd_sts.u8_cursor_col = ps_scr_status->u8_cursor_col;

        //----------------------------------------------------------------------
        // 画面描画
        //----------------------------------------------------------------------
        v_lcd_screen_drawing();
        v_lcd_cursor_drawing();
    }
}

/*******************************************************************************
 *
 * NAME: v_scr_ticket_delete
 *
 * DESCRIPTION:チケット削除画面のイベント処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_ticket_delete(ts_com_event_info_t* ps_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_TICKET_DELETE];
    // チケットインデックス
    uint32_t u32_ticket_idx;
    // BLEアドレス
    esp_bd_addr_t t_bda;
    // チケット情報
    ts_com_msg_auth_ticket_t s_ticket_info;
    // 画面描画フラグ
    bool b_scr_draw = false;

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (ps_evt->e_event) {
    case EVT_SCR_INIT:
        // 初期処理
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_SCR_INIT");
#endif
        // 未接続時は初期化してスキャン開始
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソル非表示
        ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 11;                  // カーソル位置（列）
        // チケット参照
        if (sts_msg_ticket_edit_info(0, t_bda, &s_ticket_info) != ESP_OK) {
            // チケットが無いので、ステータス表示画面に遷移
            v_evt_screen_change(SCR_ID_STATUS_DISPLAY);
            return;
        }
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_CONNECT:
        // MSG：接続通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_CONNECT");
#endif
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_BLE_CONNECT_ERROR");
#endif
        break;
    case EVT_BLE_DISCONNECT:
        // BLE：切断通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_DISCONNECT");
#endif
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_CD_CHK");
#endif
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_OK");
#endif
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_PAIR_ERR");
#endif
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_STS_OK");
#endif
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータス異常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_STS_ALARM");
#endif
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_RX_DATA");
#endif
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_MSG_RX_ERROR");
#endif
        break;
    case EVT_TIMEOUT:
        // タイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_TIMEOUT");
#endif
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_UP");
#endif
        //----------------------------------------------------------------------
        // 前のチケットへ
        //----------------------------------------------------------------------
        // 表示行の移動可否判定
        if (ps_scr_status->i_disp_row < 2) {
            break;
        }
        // チケットインデックス算出
        u32_ticket_idx = (ps_scr_status->i_disp_row / 2) - 1;
        // チケット参照
        if (sts_msg_ticket_edit_info(u32_ticket_idx, t_bda, &s_ticket_info) != ESP_OK) {
            break;
        }
        // 表示行の更新
        ps_scr_status->i_disp_row = ps_scr_status->i_disp_row - 2;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_DOWN");
#endif
        //----------------------------------------------------------------------
        // 次のチケットへ
        //----------------------------------------------------------------------
        // チケットインデックス算出
        u32_ticket_idx = (ps_scr_status->i_disp_row / 2) + 1;
        // チケット参照
        if (sts_msg_ticket_edit_info(u32_ticket_idx, t_bda, &s_ticket_info) != ESP_OK) {
            break;
        }
        // 表示行の更新
        ps_scr_status->i_disp_row = ps_scr_status->i_disp_row + 2;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_LEFT");
#endif
        //----------------------------------------------------------------------
        // カーソル左移動
        //----------------------------------------------------------------------
        // チケットインデックス算出
        u32_ticket_idx = ps_scr_status->i_disp_row / 2;
        // チケット参照
        if (sts_msg_ticket_edit_info(u32_ticket_idx, t_bda, &s_ticket_info) != ESP_OK) {
            break;
        }
        // カーソル移動
        ps_scr_status->u8_cursor_col = 11;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_RIGHT");
#endif
        //----------------------------------------------------------------------
        // カーソル右移動
        //----------------------------------------------------------------------
        // チケットインデックス算出
        u32_ticket_idx = ps_scr_status->i_disp_row / 2;
        // チケット参照
        if (sts_msg_ticket_edit_info(u32_ticket_idx, t_bda, &s_ticket_info) != ESP_OK) {
            break;
        }
        // カーソル移動
        ps_scr_status->u8_cursor_col = 14;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_PUSH");
#endif
        //----------------------------------------------------------------------
        // 削除判定
        //----------------------------------------------------------------------
        if (ps_scr_status->u8_cursor_col != 11) {
            // ステータス表示画面に遷移
            v_evt_screen_change(SCR_ID_STATUS_DISPLAY);
            break;
        }
        //----------------------------------------------------------------------
        // 選択チケット削除
        //----------------------------------------------------------------------
        // チケットインデックス算出
        u32_ticket_idx = ps_scr_status->i_disp_row / 2;
        // 対象チケット情報取得
        if (sts_msg_ticket_edit_info(u32_ticket_idx, t_bda, &s_ticket_info) != ESP_OK) {
            break;
        }
        // チケット削除
        sts_com_msg_delete_ticket(s_ticket_info.u64_rmt_device_id);
        // ステータス表示画面に遷移
        v_evt_screen_change(SCR_ID_STATUS_DISPLAY);
        break;
    default:
        break;
    }

    //==========================================================================
    // 画面描画
    //==========================================================================
    if (b_scr_draw) {
        //----------------------------------------------------------------------
        // スクリーンバッファ編集
        //----------------------------------------------------------------------
        // バッファクリア
        strcpy(s_lcd_sts.c_buff[0], COM_LCD_EMPTY_LINE);
        strcpy(s_lcd_sts.c_buff[1], COM_LCD_EMPTY_LINE);
        // コメント行
        strcpy(s_lcd_sts.c_buff[0], "DELETE     OK/NG");
        // BLEアドレスを編集
        char c_bda[9];
        i_vutil_base64_encode(c_bda, t_bda, 6);
        sprintf(s_lcd_sts.c_buff[1], "%s:%07lld", c_bda, s_ticket_info.u64_rmt_device_id);
        // スペースパディング
        i_vutil_str_rpad(s_lcd_sts.c_buff[1], ' ', COM_LCD_LINE_SIZE);

        //----------------------------------------------------------------------
        // カーソル描画バッファ編集
        //----------------------------------------------------------------------
        s_lcd_sts.e_cursor_type = ps_scr_status->e_cursor_type;
        s_lcd_sts.u8_cursor_row = ps_scr_status->u8_cursor_row;
        s_lcd_sts.u8_cursor_col = ps_scr_status->u8_cursor_col;

        //----------------------------------------------------------------------
        // 画面描画
        //----------------------------------------------------------------------
        v_lcd_screen_drawing();
        v_lcd_cursor_drawing();
    }
}

/*******************************************************************************
 *
 * NAME: v_scr_pairing_check
 *
 * DESCRIPTION:ペアリングチェック画面のイベント処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_com_event_info_t* ps_evt      R   イベント情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_pairing_check(ts_com_event_info_t* ps_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_PAIRING_CHECK];
    // 画面描画フラグ
    bool b_scr_draw = false;

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (ps_evt->e_event) {
    case EVT_SCR_INIT:
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_SCR_INIT");
        uint8_t u8_cli_key[BLE_MSG_PUBLIC_KEY_CLI_SIZE];
        uint8_t u8_svr_key[BLE_MSG_PUBLIC_KEY_SVR_SIZE];
        if (sts_com_msg_edit_public_key_pair(u8_cli_key, u8_svr_key) == ESP_OK) {
            v_dbg_disp_hex_data("CLI:", u8_cli_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
            v_dbg_disp_hex_data("SVR:", u8_svr_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
        }
#endif
        // 公開鍵のペアからペアリング確認用コードを編集
        sts_msg_pairing_check_code_edit(s_com_status.c_pair_chk_code);
        // 画面：初期処理
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルタイプ
        ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 11;                  // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_CONNECT:
        // MSG：接続通知
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
        break;
    case EVT_BLE_DISCONNECT:
        // BLE：切断通知
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータス異常
        break;
    case EVT_TIMEOUT:
        // タイムアウト
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_UP");
#endif
        if (ps_scr_status->i_disp_row == 0) {
            break;
        }
        // 表示行更新
        ps_scr_status->i_disp_row = 0;
        // カーソルタイプ更新
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_NONE) {
            ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY;
        }
        // 画面再描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_DOWN");
#endif
        if (ps_scr_status->i_disp_row != 0) {
            break;
        }
        // 表示行更新
        ps_scr_status->i_disp_row = 2;
        // カーソルタイプ更新
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_NONE) {
            ps_scr_status->e_cursor_type = CURSOR_TYPE_WAIT;
        }
        // 画面再描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_LEFT");
#endif
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // カーソル移動（列）
        ps_scr_status->u8_cursor_col = 11;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_RIGHT");
#endif
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // カーソル移動（列）
        ps_scr_status->u8_cursor_col = 14;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrEvt=EVT_INPUT_PUSH");
#endif
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // 列判定
        if (ps_scr_status->u8_cursor_col == 11) {
            // ペアリング認証（ダイジェスト比較結果の通知）
            if (sts_com_msg_tx_pairing_certification(true, BLE_MSG_MAX_SEQ_NO) != ESP_OK) {
                // ネットワーク切断
                v_ble_disconnection();
                // メッセージ表示後にステータス表示画面に遷移
                v_evt_show_msg(COM_MSG_ID_ERR_TXRX);
            }
        } else {
            // ペアリングエラー
            sts_com_msg_tx_pairing_certification(false, BLE_MSG_MAX_SEQ_NO);
            // ネットワーク切断
            v_ble_disconnection();
            // メッセージ表示後にステータス表示画面に遷移
            v_evt_show_msg(COM_MSG_ID_ERR_PAIRING);
        }
        // カーソルタイプ
        ps_scr_status->e_cursor_type = CURSOR_TYPE_NONE;
        // 画面描画
        b_scr_draw = true;
        break;
    default:
        break;
    }

    //==========================================================================
    // 画面描画
    //==========================================================================
    if (b_scr_draw) {
        //----------------------------------------------------------------------
        // 画面描画バッファの編集
        //----------------------------------------------------------------------
        // 表示列判定
        if (ps_scr_status->i_disp_row == 0) {
            // デバイスＩＤ
            strcpy(s_lcd_sts.c_buff[0], "CODE CHECK OK/NG");
            // ペアリング確認用コード編集
            memcpy(s_lcd_sts.c_buff[1], &s_com_status.c_pair_chk_code[0], COM_LCD_LINE_SIZE);
        } else {
            // ペアリング確認用コード編集
            memcpy(s_lcd_sts.c_buff[0], &s_com_status.c_pair_chk_code[16], COM_LCD_LINE_SIZE);
            memcpy(s_lcd_sts.c_buff[1], &s_com_status.c_pair_chk_code[32], COM_LCD_LINE_SIZE);
        }
        // スペースパディング
        i_vutil_str_rpad(s_lcd_sts.c_buff[0], ' ', COM_LCD_LINE_SIZE);
        i_vutil_str_rpad(s_lcd_sts.c_buff[1], ' ', COM_LCD_LINE_SIZE);

        //----------------------------------------------------------------------
        // カーソル描画バッファ編集
        //----------------------------------------------------------------------
        s_lcd_sts.e_cursor_type = ps_scr_status->e_cursor_type;
        s_lcd_sts.u8_cursor_row = ps_scr_status->u8_cursor_row % 2;
        s_lcd_sts.u8_cursor_col = ps_scr_status->u8_cursor_col % COM_LCD_LINE_SIZE;

        //----------------------------------------------------------------------
        // 画面描画
        //----------------------------------------------------------------------
        v_lcd_screen_drawing();
        v_lcd_cursor_drawing();
    }
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
