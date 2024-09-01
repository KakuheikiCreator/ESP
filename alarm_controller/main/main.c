/*******************************************************************************
 *
 * MODULE :alarm controller main source file
 *
 * CREATED:2022/12/02 07:55:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:警報機のコントローラ機能を実装
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
#include <nvs_flash.h>
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
#define TWDT_TIMEOUT_MSEC       (3000)

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

/** 5way switch ADC channel */
#define COM_5WAY_CHANNEL    (ADC_CHANNEL_6)

/** LCDコントラスト */
#ifndef COM_LCD_CONTRAST
    #define COM_LCD_CONTRAST    (0x28)
#endif

/** LCD空行サイズ */
#define COM_LCD_LINE_SIZE       (16)

/** LCD空行 */
#define COM_LCD_EMPTY_LINE      "                "

/** microSDマウント先 */
#ifndef COM_SD_MOUNT
    #define COM_SD_MOUNT        "/sdcard"
#endif

/** 設定ファイルパス */
#ifndef COM_PATH_SETTING
    #define COM_PATH_SETTING   "/sdcard/setting.json"
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
// パスキー
#ifndef GAP_STATIC_PASSKEYT
    #define GAP_STATIC_PASSKEY      (123456)
#endif

/** スキャン時間（単位：ミリ秒） */
#ifndef BLE_GAP_SCAN_TIME
    #define BLE_GAP_SCAN_TIME       (90000)
#endif

/** BLE GATTアプリケーションID */
#define BLE_GATT_APP_ID             (0x2E11)
/** BLE GATTプロファイルで管理するインターフェース数 */
#define BLE_GATT_IF_CNT             (1)
/** BLE GATTプロファイル情報配列のSPPインターフェースインデックス */
#define BLE_GATT_SVC_IDX            (0)

/** UARTタスクの優先度 */
#define BLE_MSG_UART_PRIORITIES     (configMAX_PRIORITIES - 5)
/** メッセージデバイスID */
#define BLE_MSG_DEVICE_ID           (0x0000000000000001)
/** 最大メッセージサイズ */
#define BLE_MSG_MAX_SIZE            (128)
/** メッセージ最大シーケンスサイズ */
#define BLE_MSG_MAX_SEQ_NO          (0xFFFFFFFF)
/** 確認メッセージコードサイズ */
#define BLE_MSG_CODE_SIZE           (48)
/** 公開鍵サイズ（クライアント） */
#define BLE_MSG_PUBLIC_KEY_CLI_SIZE (36)
/** 公開鍵サイズ（サーバー） */
#define BLE_MSG_PUBLIC_KEY_SVR_SIZE (33)
/** メッセージチェックコードサイズ */
#define BLE_MSG_CHECK_CODE_SIZE     (32)
/** メッセージIDサイズ */
#define BLE_MSG_ID_SIZE             (6)
/** メッセージデータサイズ */
#define BLE_MSG_REC_SIZE            (2)

/** BLE GATTプロファイル情報配列のSPPインターフェースインデックス */
#define BLE_MSG_CON_CHK_STS     (COM_BLE_MSG_CON_DISCONNECTED | COM_BLE_MSG_CON_CONNECTED | COM_BLE_MSG_CON_ERROR)


//==============================================================================
// タスク関係
//==============================================================================
/** タスク優先度（高） */
//#define TASK_PRIORITIES_HIGH    (2)
/** タスク優先度（中） */
#define TASK_PRIORITIES_MEDIUM  (1)
/** タスク優先度（低） */
#define TASK_PRIORITIES_LOW     (0)
/** 待ち時間：タスク終端 */
#define IDLE_TASK_WAIT_TICK    (500 / portTICK_PERIOD_MS)

//==============================================================================
// イベント関係
//==============================================================================
/** 待ち時間：クリティカルセクション */
#define EVT_TAKE_WAIT_TICK          (1000 / portTICK_PERIOD_MS)
/** 待ち時間：イベントエンキュー */
#define EVT_ENQUEUE_WAIT_TICK       (50 / portTICK_PERIOD_MS)
/** 待ち時間：受信イベント処理 */
#define EVT_RX_WAIT_TICK            (100 / portTICK_PERIOD_MS)
/** 待ち時間：タイマー処理 */
#define EVT_TIMER_WAIT_TICK         (50 / portTICK_PERIOD_MS)
/** 待ち時間：ディスコネクトタイムアウト */
#define EVT_DISCONNECT_TIMEOUT      (2000 / portTICK_PERIOD_MS)
/** 待ち時間：接続タイムアウト（ミリ秒） */
#define EVT_CONNECTION_TIMEOUT_MS   (5000)
/** 待ち時間：ペアリングタイムアウト（ミリ秒） */
#define EVT_PAIRING_TIMEOUT_MS      (90000)
/** 待ち時間：ステータスチェックアウト（ミリ秒） */
#define EVT_STATUS_CHECK_TIMEOUT_MS (1000)
/** 待ち時間：モードチェックタイムアウト（ミリ秒） */
#define EVT_MODE_CHECK_TIMEOUT_MS   (1000)
/** イベントキューサイズ */
#define EVT_QUEUE_SIZE              (16)
/** イベントキューウェイト時間 */
#define EVT_QUEUE_WAIT              (0)

//==============================================================================
// チケット定義
//==============================================================================
// リモートデバイスアドレスサイズ(Base64)
#define COM_TICKET_DEV_BDA_BASE64_SIZE (8)
// リモートデバイスアドレスサイズ(uint8_t)
#define COM_TICKET_DEV_BDA_SIZE (6)
// リモートデバイス名サイズ
#define COM_TICKET_DEV_NAME_SIZE (16)
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
 * 接続ステータス
 */
typedef enum {
    CON_STS_DISCONNECTED = 0,   // 切断
    CON_STS_SCANNING,           // スキャン実行
    CON_STS_CONNECTING,         // 接続開始
    CON_STS_PAIRING,            // ペアリング中
    CON_STS_CHECK,              // ステータスチェック中
    CON_STS_MODE_CHECK,         // 動作モードチェック中
    CON_STS_CONNECTED,          // 接続済み
} te_connection_sts_t;

/**
 * 画面ID定義
 */
typedef enum {
    SCR_ID_MSG_DISPLAY = 0,     // メッセージ表示
    SCR_ID_BOOT,                // 起動画面
    SCR_ID_DEVICE_SELECT,       // デバイス選択
    SCR_ID_PAIRING_CHECK,       // ペアリングチェック
    SCR_ID_DEVICE_CONTROL,      // デバイス制御
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
    OPR_MODE_ALERT  = 0x01,     // 警戒モード
    OPR_MODE_ALARM  = 0x02,     // 警報モード
    OPR_MODE_COUNT,             // 動作モード数
} te_operating_mode_t;

/**
 * ユーザーイベント定義
 */
typedef enum {
    EVT_SCR_INIT = 0,           // 画面：初期処理
    EVT_BLE_SCAN_START,         // BLE：スキャン開始
    EVT_BLE_SCAN_RESULT,        // BLE：スキャン結果取得
    EVT_BLE_SCAN_COMPLETE,      // BLE：スキャン完了
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
    esp_bd_addr_t t_rmt_device_bda;                         // BLEアドレス
    char c_rmt_device_name[COM_TICKET_DEV_NAME_SIZE + 1];   // リモートデバイス名
    ts_com_msg_auth_ticket_t s_ticket;                      // チケット
    struct s_ticket_node_t* ps_next;                        // 次のチケット
} ts_ticket_node_t;

/**
 * リモートデバイス情報ステータス
 */
typedef struct {
    ts_com_ble_gap_device_list_t* ps_scan_list;     // 検索結果リスト
    ts_ticket_node_t* ps_ticket_top;                // チケットリンクリスト（先頭）
    ts_ticket_node_t* ps_ticket_tail;               // チケットリンクリスト（末尾）
} ts_com_remote_status_t;

/**
 * 共通イベントコールバック関数
 */
typedef void (*tf_com_evt_cb_t)(te_usr_event_t e_evt);

/**
 * 共通ステータス
 */
typedef struct {
    te_usr_screen_id_t e_scr_id;                    // 現在画面ID
    char c_msg_id[BLE_MSG_ID_SIZE];                 // 表示メッセージID
    te_connection_sts_t e_connect_sts;              // 接続ステータス
    uint16_t u16_select_idx;                        // 選択インデックス
    ts_com_ble_gap_device_info_t* ps_gap_device;    // 選択GAPデバイス
    ts_com_msg_auth_ticket_t s_ticket;              // 選択チケット
    te_operating_mode_t e_operating_mode;           // 選択デバイス動作モード
    char c_pair_chk_code[BLE_MSG_CODE_SIZE + 1];    // ペアリング確認用コード(Base64)
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
// ファイル関連
//==============================================================================
/** 設定ファイル読み込み処理 */
static bool b_read_setting();
/** メッセージファイル読み込み処理 */
static bool b_read_message();
/** チケットファイル読み込み処理 */
static bool b_read_ticket();
/** チケットファイル書き込み処理 */
static bool b_write_ticket();

//==============================================================================
// BluetoothLE関連処理
//==============================================================================
/** BluetoothLE初期化処理 */
static esp_err_t sts_ble_init();
/** GAPプロファイルのイベントコールバック */
static void v_ble_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param);

//==============================================================================
// BLEメッセンジャー関連処理
//==============================================================================
/** BLEメッセージイベント処理 */
static void v_msg_evt_cb(te_com_ble_msg_event e_msg_evt);
/** BLEメッセージチケットイベント処理 */
static esp_err_t sts_msg_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット生成処理 */
static esp_err_t sts_msg_ticket_create(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット参照処理 */
static esp_err_t sts_msg_ticket_read(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット更新処理 */
static esp_err_t sts_msg_ticket_update(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケット削除処理 */
static esp_err_t sts_msg_ticket_delete(ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケットのコピー処理 */
static esp_err_t sts_msg_ticket_copy(esp_bd_addr_t t_bda, ts_com_msg_auth_ticket_t* ps_ticket);
/** BLEメッセージチケットノードの取得処理 */
static ts_ticket_node_t* ps_msg_ticket_get_node(uint64_t u64_device_id);
/** BLEペアリング確認用コードの編集処理 */
static esp_err_t sts_msg_pairing_check_code_edit(char* pc_code);

//==============================================================================
// 制御メッセージ関連処理
//==============================================================================
/** 制御メッセージの送信処理 */
static esp_err_t sts_tx_ctrl_msg(te_msg_ctrl_cmd_t e_cmd, te_operating_mode_t e_mode);
/** 制御メッセージの受信処理 */
static esp_err_t sts_rx_ctrl_msg();

//==============================================================================
// デバイス情報関連
//==============================================================================
/** デバイスリストのリフレッシュ処理 */
static void v_refresh_scan_result();
/** デバイス選択処理 */
static esp_err_t sts_select_device(uint16_t u16_idx);
/** 前デバイス選択処理 */
static esp_err_t sts_prev_device();
/** 次デバイス選択処理 */
static esp_err_t sts_next_device();

//==============================================================================
// スレッド間連携
//==============================================================================
/** イベントのエンキュー処理 */
static esp_err_t sts_evt_enqueue(te_usr_event_t e_evt);
/** イベントのデキュー処理 */
static esp_err_t sts_evt_dequeue(te_usr_event_t* pe_evt);
/** タイムアウト判定 */
static bool b_evt_chk_timeout();
/** タイムアウト時間の設定 */
static void v_evt_set_timeout(int64_t i64_timeout_ms);
/** タイムアウト時間のクリア */
static void v_evt_clear_timeout();
/** 接続ステータス更新イベント処理 */
static bool b_evt_upd_connect_sts(te_connection_sts_t e_sts);

//==============================================================================
// イベント関連処理
//==============================================================================
/** イベント処理タスク */
static void v_task_event(void* args);
/** タイマーイベント処理タスク */
static void v_task_timer_event(void* args);
/** 共通イベント処理 */
static void v_evt_common(te_usr_event_t e_evt);
/** ペアリング解除処理 */
static void v_evt_unpairing(esp_bd_addr_t t_bda, uint64_t u64_device_id);
/** 画面遷移イベント処理 */
static void v_evt_screen_change(te_usr_screen_id_t e_scr_id);
/** メッセージ表示イベント処理 */
static void v_evt_show_error_msg(char* pc_msg_id);

//==============================================================================
// 各画面のイベント処理
//==============================================================================
/** メッセージ表示 */
static void v_scr_message_display(te_usr_event_t e_evt);
/** 起動画面 */
static void v_scr_boot(te_usr_event_t e_evt);
/** デバイス選択 */
static void v_scr_device_select(te_usr_event_t e_evt);
/** ペアリング確認 */
static void v_scr_pairing_check(te_usr_event_t e_evt);
/** デバイスコントロール */
static void v_scr_device_control(te_usr_event_t e_evt);

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

/** イベントキュー */
static QueueHandle_t s_evt_queue = NULL;

/** イベントデーモンハンドル */
static TaskHandle_t s_evt_handle;

/** 入力チェックデーモンハンドル */
static TaskHandle_t s_input_handle;

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

/** デバイスステータス */
static ts_com_remote_status_t s_com_rmt_dev_sts = {
    .ps_scan_list     = NULL,       // 検索結果リスト
    .ps_ticket_top    = NULL,       // チケットリンクリスト（先頭）
    .ps_ticket_tail   = NULL,       // チケットリンクリスト（末尾）
};

/** 共通ステータス */
static ts_com_status_t s_com_status = {
    .e_scr_id = SCR_ID_BOOT,                    // 現在画面ID
    .c_msg_id = {0x00},                         // メッセージID
    .e_connect_sts = CON_STS_DISCONNECTED,      // メッセンジャー機能接続ステータス
    .u16_select_idx = 0,                        // 選択インデックス
    .ps_gap_device = NULL,                      // 選択GAPデバイス
    .s_ticket = {0},                            // 選択デバイスのチケット
    .e_operating_mode = OPR_MODE_COUNT,         // 選択デバイス動作モード
    .c_pair_chk_code = {0x00},                  // ペアリング確認用コード
    .i64_timeout_ms = MAX_VALUE_INT64,          // タイムアウト（ミリ秒）
};

/** 各画面のステータス */
static ts_scr_status_t s_scr_sts_list[SCR_ID_COUNT] = {
    // 起動画面
    [SCR_ID_BOOT] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                      // カーソル位置（行）
        .u8_cursor_col = 0,                      // カーソル位置（列）
        .pf_evt_cb = v_scr_boot,                // イベントコールバック
    },
    // デバイス選択
    [SCR_ID_DEVICE_SELECT] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                      // カーソル位置（行）
        .u8_cursor_col = 0,                      // カーソル位置（列）
        .pf_evt_cb = v_scr_device_select,       // イベントコールバック
    },
    // デバイス制御
    [SCR_ID_DEVICE_CONTROL] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                      // カーソル位置（行）
        .u8_cursor_col = 0,                      // カーソル位置（列）
        .pf_evt_cb = v_scr_device_control,      // イベントコールバック
    },
    // ペアリングチェック
    [SCR_ID_PAIRING_CHECK] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                      // カーソル位置（行）
        .u8_cursor_col = 0,                      // カーソル位置（列）
        .pf_evt_cb = v_scr_pairing_check,       // イベントコールバック
    },
    // メッセージ表示
    [SCR_ID_MSG_DISPLAY] = {
        .i_disp_row = -1,                       // 表示位置（行）
        .e_cursor_type = CURSOR_TYPE_NONE,      // カーソルタイプ
        .u8_cursor_row = 0,                      // カーソル位置（行）
        .u8_cursor_col = 0,                      // カーソル位置（列）
        .pf_evt_cb = v_scr_message_display,     // イベントコールバック
    },
};

//==============================================================================
// BluetoothLE関連
//==============================================================================
// GATTアプリケーション設定
static ts_com_ble_gattc_if_config_t s_gattc_app_config[BLE_GATT_IF_CNT];

/**
 * スキャンパラメータ
 */
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
//    .own_addr_type          = BLE_ADDR_TYPE_RPA_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/


// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

// SDMMCで接続するとJTAGピンの一部（GPIO12:TDI）をアサインしてしまう為
// SPIモードで接続
//#define USE_SPI_MODE

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13
#endif //USE_SPI_MODE

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
    s_evt_queue = xQueueCreate(EVT_QUEUE_SIZE, sizeof(te_usr_event_t*));
    // イベント処理タスクを起動する
    xTaskCreatePinnedToCore(v_task_event, "event task", 65536, NULL, TASK_PRIORITIES_MEDIUM, &s_evt_handle, tskNO_AFFINITY);
    // 入力イベント処理タスクを起動する
    xTaskCreatePinnedToCore(v_task_timer_event, "timer event task", 8192, NULL, TASK_PRIORITIES_LOW, &s_input_handle, tskNO_AFFINITY);
    // 起動画面への画面遷移
    v_evt_screen_change(SCR_ID_BOOT);

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
    // 不揮発性メモリ領域のデフォルトパーティションを初期化する
    esp_err_t sts_val = nvs_flash_init();
    // NVS領域の初期化結果のエラーパターンを判定
    // 1.NVSの空きページが無い
    // 2.新バージョンのデータがパーティションに含まれている
    if (sts_val == ESP_ERR_NVS_NO_FREE_PAGES || sts_val == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS領域をクリア、エラー時には異常終了する
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    // エラーチェック
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
//    CHECK_ERROR_CODE(esp_task_wdt_init(&s_wdt_cfg), ESP_OK);
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
    // SPIバスの初期処理
    //==========================================================================
    // SPIバス初期化
    sdmmc_host_t s_host = SDSPI_HOST_DEFAULT();
    s_host.slot = SPI2_HOST;
//    s_host.max_freq_khz = SDMMC_FREQ_DEFAULT;
//    s_host.max_freq_khz = SDMMC_FREQ_PROBING;
    ESP_ERROR_CHECK(sts_spi_mst_bus_initialize(s_host.slot, &s_spi_bus_cfg, SPI_DMA_CH1, true));

    //==========================================================================
    // I2Cバスの初期処理
    //==========================================================================
    // I2Cバス初期化
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
    s_sd_sts.ps_card = ps_futil_sdmmc_hspi_mount(COM_SD_MOUNT, GPIO_NUM_15, GPIO_NUM_NC, GPIO_NUM_NC, ps_mnt_cfg);
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
    if (!b_read_ticket()) {
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
 * NAME: b_read_ticket
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
static bool b_read_ticket() {
    //==========================================================================
    // チケットファイルを読み込み
    //==========================================================================
    cJSON* ps_ticket_root = ps_futil_cjson_parse_file(COM_PATH_TICKET, i_vutil_conv_to_kilo(10));
    if (ps_ticket_root == NULL) {
        return true;
    }
    // チケットリストの取得
    cJSON* ps_ticket_list = cJSON_GetObjectItem(ps_ticket_root, COM_TICKET_LIST);
    if (ps_ticket_list == NULL) {
        // cJSON解放
        cJSON_Delete(ps_ticket_root);
        return true;
    }
    // チケットリストサイズ
    int i_list_size = cJSON_GetArraySize(ps_ticket_list);
    if (i_list_size <= 0) {
        // cJSON解放
        cJSON_Delete(ps_ticket_root);
        return true;
    }

    //==========================================================================
    // チケットループ
    //==========================================================================
    // 先頭のダミーチケット
    ts_ticket_node_t s_ticket_node_dmy_top;
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
    cJSON* ps_rmt_device_name = NULL;   // リモートデバイス名
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
        // リモートデバイス名の読み込み
        ps_rmt_device_name = cJSON_GetObjectItem(ps_ticket_elm, COM_TICKET_RMT_DEV_NAME);
        if (ps_rmt_device_name == NULL) {
            break;
        }
        if (ps_rmt_device_name->valuestring == NULL) {
            break;
        }
        if (i_vutil_strlen(ps_rmt_device_name->valuestring) < 0) {
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

        //======================================================================
        // チケット追加
        //======================================================================
        // チケット生成
        ps_ticket_node_bef = ps_ticket_node_tgt;
        ps_ticket_node_tgt = pv_mem_malloc(sizeof(ts_ticket_node_t));
        if (ps_ticket_node_tgt == NULL) {
            break;
        }
        ps_ticket_node_bef->ps_next = ps_ticket_node_tgt;
        s_com_rmt_dev_sts.ps_ticket_tail = ps_ticket_node_tgt;
        // 編集チケット
        ps_ticket_edit = &ps_ticket_node_tgt->s_ticket;
        // リモートデバイスアドレス
        i_vutil_base64_decode(ps_ticket_node_tgt->t_rmt_device_bda, ps_rmt_device_bda->valuestring);
        // リモートデバイス名
        strcpy(ps_ticket_node_tgt->c_rmt_device_name, ps_rmt_device_name->valuestring);
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
        s_com_rmt_dev_sts.ps_ticket_top  = NULL;
        s_com_rmt_dev_sts.ps_ticket_tail = NULL;
        // エラー返信
        return false;
    }
    // 正常に読み込み
    s_com_rmt_dev_sts.ps_ticket_top = s_ticket_node_dmy_top.ps_next;

    // 正常読み込み
    return true;
}

/*******************************************************************************
 *
 * NAME: b_write_ticket
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
static bool b_write_ticket() {
    //==========================================================================
    // チケットイメージの編集
    //==========================================================================
    // チケットファイルのJSONルート生成
    cJSON* ps_ticket_root = cJSON_CreateObject();
    // チケットリスト追加
    cJSON* ps_ticket_list = cJSON_CreateArray();
    cJSON_AddItemToObject(ps_ticket_root, COM_TICKET_LIST, ps_ticket_list);
    // チケット要素
    cJSON* ps_ticket_elm = NULL;
    // チケット追加ループ
    ts_ticket_node_t* ps_ticket_node = s_com_rmt_dev_sts.ps_ticket_top;
    ts_com_msg_auth_ticket_t* ps_ticket;
    char c_wk_edit[45];
    while(ps_ticket_node != NULL) {
        //----------------------------------------------------------------------
        // チケット
        //----------------------------------------------------------------------
        ps_ticket = &ps_ticket_node->s_ticket;

        //----------------------------------------------------------------------
        // CJSONチケット編集
        //----------------------------------------------------------------------
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
        cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_RMT_DEV_NAME, ps_ticket_node->c_rmt_device_name);
        // 暗号鍵
        i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_enc_key, COM_MSG_SIZE_CIPHER_KEY);
        cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_ENC_KEY, c_wk_edit);
        // 自ステータス
        i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_own_sts, COM_MSG_SIZE_TICKET_STS);
        cJSON_AddStringToObject(ps_ticket_elm, COM_TICKET_OWN_STS, c_wk_edit);
        // 相手ステータスハッシュ
        i_vutil_base64_encode(c_wk_edit, ps_ticket->u8_rmt_sts_hash, COM_MSG_SIZE_TICKET_STS);
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

        //----------------------------------------------------------------------
        // 次のチケット
        //----------------------------------------------------------------------
        ps_ticket_node = ps_ticket_node->ps_next;
    }

    //==========================================================================
    // チケットファイルを書き込み
    //==========================================================================
    esp_err_t sts_val = sts_futil_cjson_write_file(COM_PATH_TICKET, ps_ticket_root);
    // cJSON解放
    cJSON_Delete(ps_ticket_root);

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
    esp_err_t sts_ret = sts_com_ble_init();
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }
    // 電波の出力設定
    sts_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }
    // ボンディングデバイス表示
    sts_ret = sts_com_ble_display_bonded_devices();
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }

    //==========================================================================
    // セキュリティ マネージャー プロトコル（SMP）設定
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
    sts_ret = sts_com_ble_gap_smp_init(s_ble_gap_cfg);
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }
    // スキャンパラメータの設定処理
    sts_ret = sts_com_ble_gap_set_scan_params(&ble_scan_params);
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }

    //==========================================================================
    // SPPクライアント初期設定
    //==========================================================================
    // GATTクライアント設定
    s_gattc_app_config[BLE_GATT_SVC_IDX] = s_com_ble_sppc_config();
    // アプリケーションID
    s_gattc_app_config[BLE_GATT_SVC_IDX].u16_app_id = BLE_GATT_APP_ID;
    // 接続時のセキュリティタイプ
    s_gattc_app_config[BLE_GATT_SVC_IDX].e_con_sec = ESP_BLE_SEC_ENCRYPT_MITM;
    // GATTクライアント設定
    sts_ret = sts_com_ble_gattc_register(s_gattc_app_config, BLE_GATT_IF_CNT);
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }

    //==========================================================================
    // BLEメッセージング機能初期設定(SPPプロファイルを利用)
    //==========================================================================
    // メッセージクライアントの初期処理
    sts_ret = sts_com_msg_init_cli(BLE_GATT_APP_ID,
                                   s_dev_settings.u64_device_id,
                                   BLE_MSG_MAX_SIZE,
                                   v_msg_evt_cb,
                                   sts_msg_ticket_cb);
    if (sts_ret != ESP_OK) {
        return sts_ret;
    }
    // ペアリング機能の有効化
    v_com_msg_config_pairing(true);
    // ステータスチェック機能の有効化
    v_com_msg_config_sts_chk(true);
    // 受信メッセージのエンキュー有効化
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_CIPHERTEXT);   // 暗号データ

    // 結果返信
    return sts_ret;
}

/*******************************************************************************
 *
 * NAME: v_gap_adv_event_cb
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
    // 入力チェック
    //==========================================================================
    // パラメータが有る場合のみ処理
    if (pu_param == NULL) {
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // ユーザーイベント
    te_usr_event_t e_usr_evt;
    // イベント判定
    switch (e_event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        // スキャン結果通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_SCAN_RESULT_EVT");
#endif
        // エンキュー成功まで実施
        while (sts_evt_enqueue(EVT_BLE_SCAN_RESULT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // スキャン開始
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
#endif
        // エンキュー成功まで実施
        while (sts_evt_enqueue(EVT_BLE_SCAN_START) != ESP_OK) {
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
        while (sts_evt_enqueue(e_usr_evt) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // ピアデバイスに表示されている値の返信要求 ※サーバー側とスキャン側の両方にある
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_PASSKEY_REQ_EVT");
#endif
        sts_com_ble_gap_passkey_reply(pu_param->ble_security.ble_req.bd_addr, true, GAP_STATIC_PASSKEY);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // PINコードの確認要求 ※サーバー側とスキャン側の両方にある
        // すべき事：PINコード認証の場合には、相手デバイスに表示されているPINコードの確認結果を返信
        // IOにDisplayYesNO機能があり、ピアデバイスIOにもDisplayYesNo機能がある場合、アプリはこのevtを受け取ります。
        // パスキーをユーザーに表示し、ピアデバイスにて表示される番号と一致するか確認した結果を返信する
        // GAP番号確認リクエストに応答する
        // ※MSGコード確認は中間者攻撃等も考慮してあるので、ここは自動応答する
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_NC_REQ_EVT");
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
#endif
        sts_com_ble_gap_confirm_reply(pu_param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        // スキャン完了イベント
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
#endif
        while (sts_evt_enqueue(EVT_BLE_SCAN_COMPLETE) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    default:
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "GapEvt=%d", e_event);
#endif
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_msg_evt_cb
 *
 * DESCRIPTION:メッセージイベントコールバック関数
 *
 * PARAMETERS:          Name        RW  Usage
 * te_com_ble_msg_event e_msg_evt   R   メッセージイベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_evt_cb(te_com_ble_msg_event e_msg_evt) {
    //==========================================================================
    // イベント処理
    //==========================================================================
    // イベント毎の処理
    switch (e_msg_evt) {
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_RX_RESPONSE:
        // 応答を受信
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RX_RESPONSE");
        break;
    case COM_BLE_MSG_EVT_RX_RESET:
        // リセットメッセージ受信
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RX_RESET");
        break;
    case COM_BLE_MSG_EVT_RX_PING:
        // PINGメッセージ受信
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RX_PING");
        break;
    case COM_BLE_MSG_EVT_RX_DATA:
        // データメッセージ受信
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RX_DATA");
        break;
#endif
    case COM_BLE_MSG_EVT_RX_CIPHERTEXT:
        // 暗号メッセージ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RX_CIPHERTEXT");
#endif
        // 制御データ受信
        while (sts_evt_enqueue(EVT_MSG_RX_DATA) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_GATT_CONNECT:
        // 接続
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_GATT_CONNECT");
        break;
#endif
    case COM_BLE_MSG_EVT_GATT_DISCONNECT:
        // 切断
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_GATT_DISCONNECT");
#endif
        // 切断通知
        while (sts_evt_enqueue(EVT_BLE_DISCONNECT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_LINK_SUCCESS:
        // リンク成功
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_LINK_SUCCESS");
        break;
#endif
    case COM_BLE_MSG_EVT_OPEN_SUCCESS:
        // 接続通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_OPEN_SUCCESS");
#endif
        // 接続イベント
        while (sts_evt_enqueue(EVT_MSG_CONNECT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_OPEN_TIMEOUT:
        // オープンタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_OPEN_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_START:
        // ペアリング開始
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_START");
#endif
        // ペアリング認証（ダイジェスト比較結果の通知）
        // ペアリング確認用コードチェック
        while (sts_evt_enqueue(EVT_MSG_PAIR_CD_CHK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_SUCCESS:
        // ペアリング成功
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_SUCCESS");
#endif
        while (sts_evt_enqueue(EVT_MSG_PAIR_OK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_ERR:
        // ペアリングエラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_ERR");
#endif
        while (sts_evt_enqueue(EVT_MSG_PAIR_ERROR) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_PAIRING_TIMEOUT:
        // ペアリングタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_STATUS_CHK:
        // ステータスチェック開始
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_CHK");
        break;
#endif
    case COM_BLE_MSG_EVT_STATUS_OK:
        // ステータス正常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_OK");
#endif
        // MSG：ステータス正常
        while (sts_evt_enqueue(EVT_MSG_STS_OK) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_STATUS_ERR:
        // ステータス異常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_ERR");
#endif
        // MSG：ステータスエラー
        while (sts_evt_enqueue(EVT_MSG_STS_ERROR) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
    case COM_BLE_MSG_EVT_STATUS_TIMEOUT:
        // ステータスチェックタイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_TIMEOUT");
#endif
        // タイムアウトイベント
        while (sts_evt_enqueue(EVT_TIMEOUT) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        break;
#ifdef DEBUG_ALARM
    case COM_BLE_MSG_EVT_HANDLING_ERR:
        // メッセージハンドリングエラー
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_HANDLING_ERR");
        break;
#endif
    default:
        // その他のイベント
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MsgEvt=%d", e_msg_evt);
#endif
        break;
    }
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
    // クリティカルセクション開始
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
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "COM_BLE_MSG_TICKET_EVT_CREATE id=%llu", ps_ticket->u64_rmt_device_id);
#endif
        sts_val = sts_msg_ticket_create(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_READ:
        // チケット読み込み
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "COM_BLE_MSG_TICKET_EVT_READ id=%llu", ps_ticket->u64_rmt_device_id);
#endif
        sts_val = sts_msg_ticket_read(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_UPDATE:
        // チケット更新
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "COM_BLE_MSG_TICKET_EVT_UPDATE id=%llu", ps_ticket->u64_rmt_device_id);
#endif
        sts_val = sts_msg_ticket_update(ps_ticket);
        break;
    case COM_BLE_MSG_TICKET_EVT_DELETE:
        // チケット削除
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "COM_BLE_MSG_TICKET_EVT_DELETE id=%llu", ps_ticket->u64_rmt_device_id);
#endif
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
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_ticket_create(ts_com_msg_auth_ticket_t* ps_ticket) {
    // 選択デバイス判定
    ts_com_ble_gap_device_info_t* ps_device = s_com_status.ps_gap_device;
    if (ps_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // チケットノード検索
    ts_ticket_node_t* ps_ticket_node = ps_msg_ticket_get_node(ps_ticket->u64_rmt_device_id);
    if (ps_ticket_node == NULL) {
        // チケットの生成
        ps_ticket_node = pv_mem_malloc(sizeof(ts_ticket_node_t));
        ps_ticket_node->ps_next = NULL;
        // チケット追加
        if (s_com_rmt_dev_sts.ps_ticket_top == NULL) {
            s_com_rmt_dev_sts.ps_ticket_top  = ps_ticket_node;
            s_com_rmt_dev_sts.ps_ticket_tail = ps_ticket_node;
        } else {
            s_com_rmt_dev_sts.ps_ticket_tail->ps_next = ps_ticket_node;
            s_com_rmt_dev_sts.ps_ticket_tail = ps_ticket_node;
        }
    }
    // BLEアドレス
    v_com_ble_addr_cpy(ps_ticket_node->t_rmt_device_bda, ps_device->t_bda);
    // デバイス名
    int i_dev_name_len = i_vutil_strlen(ps_device->pc_name);
    if (i_dev_name_len > 0) {
        if (i_dev_name_len <= COM_TICKET_DEV_NAME_SIZE) {
            strcpy(ps_ticket_node->c_rmt_device_name, ps_device->pc_name);
            ps_ticket_node->c_rmt_device_name[i_dev_name_len] = '\0';
        } else {
            memcpy(ps_ticket_node->c_rmt_device_name, ps_device->pc_name, COM_TICKET_DEV_NAME_SIZE);
            ps_ticket_node->c_rmt_device_name[COM_TICKET_DEV_NAME_SIZE] = '\0';
        }
    } else {
        ps_ticket_node->c_rmt_device_name[i_dev_name_len] = '\0';
    }
    // チケットファイルに書き込み
    ps_ticket_node->s_ticket = *ps_ticket;
    if (!b_write_ticket()) {
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
 * NOTES:
 * None.
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
 * NOTES:
 * None.
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
    if (!b_write_ticket()) {
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
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_ticket_delete(ts_com_msg_auth_ticket_t* ps_ticket) {
    // 対象のデバイスID
    uint64_t u64_device_id = ps_ticket->u64_rmt_device_id;
    // チケットノード探索
    ts_ticket_node_t* ps_bef_node = NULL;
    ts_ticket_node_t* ps_chk_node = s_com_rmt_dev_sts.ps_ticket_top;
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
    if (s_com_rmt_dev_sts.ps_ticket_top == ps_chk_node) {
        s_com_rmt_dev_sts.ps_ticket_top = ps_chk_node->ps_next;
    }
    if (s_com_rmt_dev_sts.ps_ticket_tail == ps_chk_node) {
        s_com_rmt_dev_sts.ps_ticket_tail = ps_bef_node;
    }
    if (ps_bef_node != NULL) {
        ps_bef_node->ps_next = ps_chk_node->ps_next;
    }
    // チケットノードの解放
    l_mem_free(ps_chk_node);
    // チケットファイルに書き込み
    if (!b_write_ticket()) {
        return ESP_FAIL;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_msg_ticket_copy
 *
 * DESCRIPTION:BLEメッセージチケットのコピー処理
 *
 * PARAMETERS:                  Name        RW  Usage
 * esp_bd_addr_t                t_bda       R   対象デバイスのアドレス
 * ts_com_msg_auth_ticket_t*    ps_ticket   W   編集対象のチケット
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_msg_ticket_copy(esp_bd_addr_t t_bda, ts_com_msg_auth_ticket_t* ps_ticket) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_ticket == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_ERR_INVALID_ARG;
    // チケット検索
    ts_ticket_node_t* ps_ticket_node = s_com_rmt_dev_sts.ps_ticket_top;
    while (ps_ticket_node != NULL) {
        if (l_com_ble_addr_cmp(ps_ticket_node->t_rmt_device_bda, t_bda) == 0) {
            // 選択チケットチケット情報をコピー
            *ps_ticket = ps_ticket_node->s_ticket;
            sts_val = ESP_OK;
            break;
        }
        ps_ticket_node = ps_ticket_node->ps_next;
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
    ts_ticket_node_t* ps_ticket_node = s_com_rmt_dev_sts.ps_ticket_top;
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
 * NAME: sts_tx_ctrl_msg
 *
 * DESCRIPTION:制御メッセージの送信処理
 *
 * PARAMETERS:              Name        RW  Usage
 * te_msg_ctrl_cmd_t        e_cmd       R   制御コマンド
 * te_msg_operating_mode_t  e_mode      R   動作モード
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_tx_ctrl_msg(te_msg_ctrl_cmd_t e_cmd, te_operating_mode_t e_mode) {
    // 選択デバイスのチケット
    ts_com_msg_auth_ticket_t* ps_msg_ticket = &s_com_status.s_ticket;
    if (ps_msg_ticket->u32_max_seq_no == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t u64_device_id = ps_msg_ticket->u64_rmt_device_id;
    // 送信データの編集
    uint8_t u8_rec[2];
    u8_rec[0] = e_cmd;
    u8_rec[1] = e_mode;
    ts_u8_array_t* ps_rec = ps_mdl_create_u8_array(u8_rec, 2);
    if (ps_rec == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // 暗号メッセージの送信処理
    esp_err_t sts_val = sts_com_msg_tx_cipher_msg(u64_device_id, ps_rec);
    // 送信データの削除
    sts_mdl_delete_u8_array(ps_rec);
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
    // 動作モードを初期化
    s_com_status.e_operating_mode = OPR_MODE_COUNT;
    do {
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
        if (ps_data->pu8_values[0] != CTL_CMD_ACK) {
            break;
        }
        // 動作モード
        if (ps_data->pu8_values[1] >= OPR_MODE_COUNT) {
            break;
        }
        // 結果を編集
        s_com_status.e_operating_mode = ps_data->pu8_values[1];
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
 * NAME: v_refresh_scan_result
 *
 * DESCRIPTION:スキャン結果のリフレッシュ処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_refresh_scan_result() {
    //==========================================================================
    // 選択デバイスアドレス取得
    //==========================================================================
    ts_com_ble_gap_device_info_t* ps_gap_device = s_com_status.ps_gap_device;
    esp_bd_addr_t t_dev_bda = {0x00};
    if (ps_gap_device != NULL) {
        v_com_ble_addr_cpy(t_dev_bda, ps_gap_device->t_bda);
    }

    //==========================================================================
    // スキャン結果をリフレッシュ
    //==========================================================================
    // 現状のスキャン結果をクリア
    v_com_ble_gap_delete_device_list(s_com_rmt_dev_sts.ps_scan_list);
    // スキャン結果を取得
    s_com_rmt_dev_sts.ps_scan_list = ps_com_ble_gap_create_device_list();

    //==========================================================================
    // 関連情報のリフレッシュ
    //==========================================================================
    // スキャン結果の有無を判定
    if (s_com_rmt_dev_sts.ps_scan_list == NULL) {
        // 選択済みデバイス情報をクリア
        s_com_status.u16_select_idx = 0;                // 選択デバイスインデックス
        s_com_status.ps_gap_device  = NULL;             // 選択GAPデバイス
        ts_com_msg_auth_ticket_t s_empty_ticket = {0};
        s_com_status.s_ticket = s_empty_ticket;         // 選択チケット
        s_com_status.e_operating_mode = OPR_MODE_COUNT; // 選択デバイス動作モード
        // 結果返信
        return;
    }
    // 選択インデックスの更新判定
    if (ps_gap_device == NULL) {
        // 選択GAPデバイスを初期化
        sts_select_device(0);
        return;
    }
    // 選択GAPデバイスを検索
    ts_com_ble_gap_device_list_t* ps_scan_list = s_com_rmt_dev_sts.ps_scan_list;
    ts_com_ble_gap_device_info_t* ps_device = ps_scan_list->ps_device;
    uint16_t u16_tgt_idx = 0;
    uint16_t u16_idx = 0;
    for (u16_idx = 0; u16_idx < ps_scan_list->u16_count; u16_idx++) {
        if (l_com_ble_addr_cmp(ps_device[u16_idx].t_bda, t_dev_bda) == 0) {
            // 選択GAPデバイスで初期化
            u16_tgt_idx = u16_idx;
            break;
        }
    }
    // 選択GAPデバイスを初期化
    sts_select_device(u16_tgt_idx);
}

/*******************************************************************************
 *
 * NAME: sts_select_device
 *
 * DESCRIPTION:デバイス選択処理
 *
 * PARAMETERS:      Name        RW  Usage
 * uint16_t         u16_idx     R   選択デバイスインデックス
 *
 * RETURNS:
 *   esp_ble_bond_dev_t*:選択されたボンディングデバイス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_select_device(uint16_t u16_idx) {
    //==========================================================================
    // ステータスチェック
    //==========================================================================
    // スキャン結果との整合性チェック
    ts_com_ble_gap_device_list_t* ps_scan_list = s_com_rmt_dev_sts.ps_scan_list;
    if (ps_scan_list == NULL) {
        return ESP_FAIL;
    }
    if (u16_idx >= ps_scan_list->u16_count) {
        return ESP_FAIL;
    }

    //==========================================================================
    // 選択GAPデバイスをリフレッシュ
    //==========================================================================
    s_com_status.u16_select_idx = u16_idx;
    s_com_status.ps_gap_device  = &ps_scan_list->ps_device[u16_idx];
    ts_com_ble_gap_device_info_t* ps_gap_device = s_com_status.ps_gap_device;

    //==========================================================================
    // 選択チケットをリフレッシュ
    //==========================================================================
    // 選択チケットの更新
    ts_com_msg_auth_ticket_t* ps_ticket = &s_com_status.s_ticket;
    if (sts_msg_ticket_copy(ps_gap_device->t_bda, ps_ticket) != ESP_OK) {
        // 対象が無い場合には、チケットを初期化
        ts_com_msg_auth_ticket_t s_empty_ticket = {0};
        s_com_status.s_ticket = s_empty_ticket;
        // 選択チケットが無い場合は動作モードをクリア
        s_com_status.e_operating_mode = OPR_MODE_COUNT;
    }

    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: sts_prev_device
 *
 * DESCRIPTION:前デバイス選択処理
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_prev_device() {
    // 選択デバイスの変更
    return sts_select_device(s_com_status.u16_select_idx - 1);
}

/*******************************************************************************
 *
 * NAME: sts_next_device
 *
 * DESCRIPTION:次デバイス選択処理
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_next_device() {
    // 選択デバイスの変更
    return sts_select_device(s_com_status.u16_select_idx + 1);
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
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // イベントのエンキュー処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    if (xQueueSend(s_evt_queue, &e_evt, 0) != pdPASS) {
        sts_val = ESP_FAIL;
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
 * NAME: sts_evt_dequeue
 *
 * DESCRIPTION:イベントのデキュー処理
 *
 * PARAMETERS:      Name            RW  Usage
 * te_com_event_t*  pe_evt          R   イベント種別
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:複数のイベントソースに対応するために排他制御を行う
 * None.
 ******************************************************************************/
static esp_err_t sts_evt_dequeue(te_usr_event_t* pe_evt) {
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // イベントのデキュー
    if (xQueueReceive(s_evt_queue, pe_evt, EVT_ENQUEUE_WAIT_TICK) != pdTRUE) {
        sts_val = ESP_FAIL;
    }
    // 結果返信
    return sts_val;
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
 * NAME: b_evt_upd_connect_sts
 *
 * DESCRIPTION: 接続ステータス更新
 *
 * PARAMETERS:          Name        RW  Usage
 * te_connection_sts_t  e_sts       R   コネクションステータス
 *
 * RETURNS:
 * true:ステータス更新成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_evt_upd_connect_sts(te_connection_sts_t e_sts) {
    //==========================================================================
    // 接続ステータス更新
    //==========================================================================
    // ステータス更新フラグ
    bool b_result = false;
    do {
        //----------------------------------------------------------------------
        // 入力チェック
        //----------------------------------------------------------------------
        // ステータス変更済み判定
        te_connection_sts_t e_sts_now = s_com_status.e_connect_sts;
        if (e_sts == e_sts_now) {
            break;
        }
        // 選択デバイス
        ts_com_ble_gap_device_info_t* ps_gap_device = s_com_status.ps_gap_device;
        // 選択デバイスの必須チェック
        if (e_sts != CON_STS_DISCONNECTED && e_sts != CON_STS_SCANNING) {
            // 切断またはスキャン以外に更新する場合
            // 選択デバイスは必須
            if (ps_gap_device == NULL) {
                break;
            }
        }

        //----------------------------------------------------------------------
        // 接続ステータス更新の初期処理
        //----------------------------------------------------------------------
        // 選択デバイスのアドレス取得
        esp_bd_addr_t t_bda;
        if (ps_gap_device != NULL) {
            // 選択デバイスアドレス取得
            v_com_ble_addr_cpy(t_bda, ps_gap_device->t_bda);
        } else {
            v_com_ble_addr_clear(t_bda);
        }
        // 選択デバイスのチケット参照
        ts_com_msg_auth_ticket_t* ps_ticket = &s_com_status.s_ticket;
        // ペアリング確認用コードをクリア
        memset(s_com_status.c_pair_chk_code, 0x00, BLE_MSG_CODE_SIZE + 1);
        // スキャン停止
        if (e_sts != CON_STS_SCANNING) {
            // スキャン中判定
            if (b_com_ble_gap_is_scanning()) {
                // スキャン停止する
                sts_com_ble_gap_stop_scan();
            }
        }

        //----------------------------------------------------------------------
        // 各接続ステータス毎の処理
        //----------------------------------------------------------------------
        // コネクションステータスを判定
        switch (e_sts) {
        case CON_STS_DISCONNECTED:
            // 切断
            //------------------------------------------------------------------
            // 切断処理
            //------------------------------------------------------------------
            // 接続判定
            if (ps_gap_device != NULL) {
                if (sts_com_ble_disconnect(t_bda) == ESP_OK) {
                    // 切断完了待ち
                    e_com_ble_gap_device_sts_wait(t_bda, GAP_DEV_STS_DEVICE_NONE, EVT_DISCONNECT_TIMEOUT);
                }
            }

            //------------------------------------------------------------------
            // ペアリング解消処理
            //------------------------------------------------------------------
            // 接続中判定
            if (e_sts_now >= CON_STS_CONNECTING && e_sts_now <= CON_STS_MODE_CHECK) {
                // 接続中の切断は、警報機側で警報かペアリング解消と判断
                v_evt_unpairing(t_bda, ps_ticket->u64_rmt_device_id);
            }

            //------------------------------------------------------------------
            // 接続ステータス更新
            //------------------------------------------------------------------
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d v_evt_clear_timeout()", __func__, __LINE__);
#endif
            // タイムアウト時間をクリア
            v_evt_clear_timeout();
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_DISCONNECTED;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_SCANNING:
            // スキャン実行
            //------------------------------------------------------------------
            // 切断処理
            //------------------------------------------------------------------
            // 接続判定
            if (ps_gap_device != NULL) {
                if (sts_com_ble_disconnect(t_bda) == ESP_OK) {
                    // 切断完了待ち
                    e_com_ble_gap_device_sts_wait(t_bda, GAP_DEV_STS_DEVICE_NONE, EVT_DISCONNECT_TIMEOUT);
                }
            }

            //------------------------------------------------------------------
            // スキャン開始
            //------------------------------------------------------------------
            // スキャン結果のリフレッシュ
            v_refresh_scan_result();
            // スキャン開始
            if (!b_com_ble_gap_is_scanning()) {
                // スキャン開始
                sts_com_ble_gap_start_scan(BLE_GAP_SCAN_TIME);
            }

            //------------------------------------------------------------------
            // 接続ステータス更新
            //------------------------------------------------------------------
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d v_evt_clear_timeout()", __func__, __LINE__);
#endif
            // タイムアウト時間をクリア
            v_evt_clear_timeout();
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_SCANNING;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_CONNECTING:
            // 接続開始
            // メッセージサーバーへの接続開始
            if (sts_com_msg_open_server(ps_gap_device) != ESP_OK) {
                break;
            }
            // タイムアウト時間を設定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d v_evt_set_timeout(EVT_CONNECTION_TIMEOUT_MS)", __func__, __LINE__);
#endif
            v_evt_set_timeout(EVT_CONNECTION_TIMEOUT_MS);
            // メッセンジャー機能接続ステータスを更新
            s_com_status.e_connect_sts = CON_STS_CONNECTING;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_PAIRING:
            // ペアリング実行
            // ペアリング開始
            if (sts_com_msg_tx_pairing_request() != ESP_OK) {
                break;
            }
            // タイムアウト時間を設定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d v_evt_set_timeout(EVT_PAIRING_TIMEOUT_MS)", __func__, __LINE__);
#endif
            v_evt_set_timeout(EVT_PAIRING_TIMEOUT_MS);
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_PAIRING;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_CHECK:
            // ステータスチェック開始
            // 選択デバイス情報の更新 ※チケット情報更新
            sts_select_device(s_com_status.u16_select_idx);
            // コマンド送信：ステータスチェックリクエスト
            if (sts_com_msg_tx_sts_chk_request() != ESP_OK) {
                break;
            }
            // タイムアウト時間を設定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d v_evt_set_timeout(EVT_STATUS_CHECK_TIMEOUT_MS)", __func__, __LINE__);
#endif
            v_evt_set_timeout(EVT_STATUS_CHECK_TIMEOUT_MS);
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_CHECK;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_MODE_CHECK:
            // 動作モードチェック
            // コマンド送信：動作モード読み込みリクエスト
            if (sts_tx_ctrl_msg(CTL_CMD_READ, OPR_MODE_NORMAL) != ESP_OK) {
                break;
            }
            // タイムアウト時間を設定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "%s L#%d v_evt_set_timeout(EVT_MODE_CHECK_TIMEOUT_MS)", __func__, __LINE__);
#endif
            v_evt_set_timeout(EVT_MODE_CHECK_TIMEOUT_MS);
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_MODE_CHECK;
            // ステータス更新完了
            b_result = true;
            break;
        case CON_STS_CONNECTED:
            // 接続済み
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d v_evt_clear_timeout()", __func__, __LINE__);
#endif
            // タイムアウト時間をクリア
            v_evt_clear_timeout();
            // 接続ステータス更新
            s_com_status.e_connect_sts = CON_STS_CONNECTED;
            // ステータス更新完了
            b_result = true;
            break;
        default:
            break;
        }
    } while(false);

    // 結果返信
    return b_result;
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
    te_usr_event_t e_evt;
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
        sts_evt_val = sts_evt_dequeue(&e_evt);

        //----------------------------------------------------------------------
        // タイムアウト判定 ※イベントの有無に関係なくイベント処理前に実施
        //----------------------------------------------------------------------
        if (b_evt_chk_timeout()) {
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d v_evt_clear_timeout()", __func__, __LINE__);
#endif
            // タイムアウト時間をクリア
            v_evt_clear_timeout();
            // タイムアウトイベント処理
            v_evt_common(EVT_TIMEOUT);
        }

        //----------------------------------------------------------------------
        // イベント取得の有無を判定
        //----------------------------------------------------------------------
        if (sts_evt_val != ESP_OK) {
            continue;
        }

        //----------------------------------------------------------------------
        // 共通イベント処理
        // ※通信系イベントについてはここで処理を行う
        //----------------------------------------------------------------------
        v_evt_common(e_evt);

        //----------------------------------------------------------------------
        // 画面イベント処理
        //----------------------------------------------------------------------
        ps_scr_sts = &s_scr_sts_list[s_com_status.e_scr_id];
        ps_scr_sts->pf_evt_cb(e_evt);
    }

    //==========================================================================
    // End
    //==========================================================================
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
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
    int i_voltage = 0;
    te_usr_event_t e_evt_input   = EVT_COUNT;
    te_usr_event_t e_evt_history = EVT_COUNT;
    // 次回実行時刻
    int64_t i64_next_msec = xTaskGetTickCountMSec();
    while (true) {
        //----------------------------------------------------------------------
        // WDTリセット
        //----------------------------------------------------------------------
        esp_task_wdt_reset();
        vTaskDelay(1);

        //----------------------------------------------------------------------
        // ウェイト処理
        //----------------------------------------------------------------------
        // 次の時刻に更新
        i64_next_msec += EVT_TIMER_WAIT_TICK;
        // 指定時刻までディレイ（ミリ秒単位）
        i64_dtm_delay_until_usec(i64_next_msec);

        //----------------------------------------------------------------------
        // 電圧取得
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
            e_evt_history = EVT_COUNT;
            continue;
        }
        // 入力チェック
        if (i_voltage > VOLTAGE_THRESHOID_PUSH) {
            // 入力：プッシュ
            e_evt_input = EVT_INPUT_PUSH;
        } else if (i_voltage > VOLTAGE_THRESHOID_LEFT) {
            // 入力：左
            e_evt_input = EVT_INPUT_LEFT;
        } else if (i_voltage > VOLTAGE_THRESHOID_UP) {
            // 入力：上
            e_evt_input = EVT_INPUT_UP;
        } else if (i_voltage > VOLTAGE_THRESHOID_RIGHT) {
            // 入力：右
            e_evt_input = EVT_INPUT_RIGHT;
        } else {
            // 入力：下
            e_evt_input = EVT_INPUT_DOWN;
        }
        // 入力履歴との比較
        if (e_evt_input == e_evt_history) {
            continue;
        }

        //----------------------------------------------------------------------
        // 入力イベントエンキュー
        //----------------------------------------------------------------------
        while (sts_evt_enqueue(e_evt_input) != ESP_OK) {
            vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        }
        // 入力履歴更新
        e_evt_history = e_evt_input;
    }
}

/*******************************************************************************
 *
 * NAME: v_evt_common
 *
 * DESCRIPTION:共通イベント処理
 *
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_common(te_usr_event_t e_evt) {
    //==========================================================================
    // イベント処理
    // ※通信系イベントについてはここで処理を行う
    //==========================================================================
    // 現在の選択チケット
    ts_com_msg_auth_ticket_t* ps_ticket = &s_com_status.s_ticket;
    // 現在の接続ステータス
    te_connection_sts_t e_con_sts = s_com_status.e_connect_sts;
    // イベント処理
    switch (e_evt) {
#ifdef DEBUG_ALARM
    case EVT_SCR_INIT:
        // 画面初期処理
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_SCR_INIT SCR_ID=%d", s_com_status.e_scr_id);
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_SCAN_START");
        break;
#endif
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_SCAN_RESULT");
#endif
        // スキャン結果をリフレッシュ
        v_refresh_scan_result();
        break;
#ifdef DEBUG_ALARM
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_SCAN_COMPLETE");
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_CONNECT");
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
        break;
#endif
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続エラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_CONNECT_ERROR");
#endif
        // 接続ステータス更新：切断
        b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
        // メッセージ表示画面に遷移
        v_evt_show_error_msg(COM_MSG_ID_ERR_CONNECT);
        break;
    case EVT_BLE_DISCONNECT:
        // BLE：切断通知
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_BLE_DISCONNECT");
        // ボンディング済みデバイスの表示
        sts_com_ble_display_bonded_devices();
#endif
        // 接続ステータス更新：切断
        if (b_evt_upd_connect_sts(CON_STS_DISCONNECTED)) {
            // 接続エラー判定
            // ※接続中の突然の切断は、電波障害と想定して接続エラーとする
            if (e_con_sts == CON_STS_CONNECTED) {
                // メッセージ表示画面に遷移
                v_evt_show_error_msg(COM_MSG_ID_ERR_CONNECT);
            }
        }
        break;
    case EVT_MSG_CONNECT:
        // MSG；接続通知
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_CONNECT");
#endif
        // ペアリングの有無を判定
        if (!b_com_msg_is_paired(ps_ticket->u64_rmt_device_id)) {
            // ペアリングされていない場合にはペアリング開始
            // 接続ステータス更新：ペアリング開始
            if (!b_evt_upd_connect_sts(CON_STS_PAIRING)) {
                // ステータス更新エラー
                // 接続ステータス更新：切断
                b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
                // メッセージ表示画面に遷移
                v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
            }
            break;
        }
        // ペアリング済みの場合にはステータスチェック開始
        // 接続ステータス更新：ステータスチェック開始
        if (!b_evt_upd_connect_sts(CON_STS_CHECK)) {
            // ステータス更新エラー
            // 接続ステータス更新：切断
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
        }
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_CD_CHK");
#endif
        // ペアリングチェック画面に画面遷移
        v_evt_screen_change(SCR_ID_PAIRING_CHECK);
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_PAIR_OK");
#endif
        // 接続ステータス更新：ステータスチェック開始
        if (!b_evt_upd_connect_sts(CON_STS_CHECK)) {
            // ステータス更新エラー
            // 接続ステータス更新：切断
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
        }
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_PAIR_ERR");
#endif
        // 接続ステータス更新：切断
        b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
        // メッセージ表示画面に遷移
        v_evt_show_error_msg(COM_MSG_ID_ERR_PAIRING);
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_STS_OK");
#endif
        // 接続ステータス更新：動作モード読み込み
        if (!b_evt_upd_connect_sts(CON_STS_MODE_CHECK)) {
            // ステータス更新エラー
            // 接続ステータス更新：切断
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_TIMEOUT);
        }
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータスエラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_STS_NG");
#endif
        // 接続ステータス更新：切断
        b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
        // メッセージ表示画面に遷移
        v_evt_show_error_msg(COM_MSG_ID_ERR_STATUS_CHK);
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御メッセージ受信
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_RX_DATA");
#endif
        // メッセージ受信
        if (sts_rx_ctrl_msg() != ESP_OK) {
            // 受信エラー
            // 接続ステータス更新：切断
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
            break;
        }
        // 接続ステータス判定
        if (e_con_sts != CON_STS_MODE_CHECK) {
            break;
        }
        // 接続ステータス更新：接続済み
        if (!b_evt_upd_connect_sts(CON_STS_CONNECTED)) {
            // ステータス更新エラー
            // 接続ステータス更新：切断
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_TIMEOUT);
            break;
        }
        // デバイス制御に画面遷移
        v_evt_screen_change(SCR_ID_DEVICE_CONTROL);
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_MSG_RX_ERROR");
#endif
        // 接続ステータス更新：切断
        b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
        // メッセージ表示画面に遷移
        v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
        break;
    case EVT_TIMEOUT:
        // タイムアウト
#ifdef DEBUG_ALARM
    ESP_LOGI(LOG_MSG_TAG, "ComEvt=EVT_TIMEOUT");
#endif
        //----------------------------------------------------------------------
        // タイムアウト共通処理
        //----------------------------------------------------------------------
        // 接続ステータス更新：切断
        b_evt_upd_connect_sts(CON_STS_DISCONNECTED);

        //----------------------------------------------------------------------
        // スキャンタイムアウト判定
        //----------------------------------------------------------------------
        // 更新前の接続ステータス
        if (e_con_sts == CON_STS_SCANNING) {
            // エラー表示：スキャンタイムアウトエラー
            v_evt_show_error_msg(COM_MSG_ID_ERR_SCAN_TIMEOUT);
            break;
        }

        //----------------------------------------------------------------------
        // 接続後のタイムアウト判定
        //----------------------------------------------------------------------
        if (e_con_sts == CON_STS_CONNECTED) {
            // ※電波障害もしくは先方でチケットが削除されたケース
            // エラー表示：接続エラー
            v_evt_show_error_msg(COM_MSG_ID_ERR_CONNECT);
            break;
        }

        //----------------------------------------------------------------------
        // 接続中のタイムアウト
        //----------------------------------------------------------------------
        // ペアリングタイムアウト判定
        if (e_con_sts == CON_STS_PAIRING) {
            // メッセージ表示画面に遷移
            v_evt_show_error_msg(COM_MSG_ID_ERR_PAIRING);
            break;
        }
        // その他の接続中のタイムアウト判定
        if (e_con_sts >= CON_STS_CONNECTING && e_con_sts <= CON_STS_MODE_CHECK) {
            // ※警報機側で警報かペアリング解消と判断
            // エラー表示：リモートチケットエラー
            v_evt_show_error_msg(COM_MSG_ID_ERR_RMT_TICKET);
            break;
        }
        // メッセージ表示後に起動画面に遷移
        v_evt_show_error_msg(COM_MSG_ID_ERR_TIMEOUT);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_evt_unpairing
 *
 * DESCRIPTION: ペアリング解除処理
 *
 * PARAMETERS:      Name            RW  Usage
 * esp_bd_addr_t    t_bda           R   対象デバイスアドレス
 * uint64_t         u64_device_id   R   対象デバイスID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_unpairing(esp_bd_addr_t t_bda, uint64_t u64_device_id) {
    //==========================================================================
    // ペアリング解除
    //==========================================================================
    // ボンディング情報を削除
    if (!b_com_ble_addr_clear(t_bda)) {
#ifdef DEBUG_ALARM
        ESP_LOGE(LOG_MSG_TAG, "Disbonding!!!");
#endif
        sts_com_ble_disbonding(t_bda);
    }
    // チケット情報取得
    if (u64_device_id != s_dev_settings.u64_device_id) {
        sts_com_msg_delete_ticket(u64_device_id);
    }

    //==========================================================================
    // 選択済みデバイス情報をクリア
    //==========================================================================
    s_com_status.u16_select_idx = 0;                // 選択インデックス
    s_com_status.ps_gap_device  = NULL;             // 選択GAPデバイス
    ts_com_msg_auth_ticket_t s_empty_ticket = {0};
    s_com_status.s_ticket = s_empty_ticket;         // 選択チケット
    s_com_status.e_operating_mode = OPR_MODE_COUNT; // 選択デバイス動作モード
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
 * NAME: v_evt_show_error_msg
 *
 * DESCRIPTION: エラーメッセージ表示イベント
 *
 * PARAMETERS:          Name        RW  Usage
 * char*                pc_msg_id   R   表示メッセージID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_evt_show_error_msg(char* pc_msg_id) {
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
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_message_display(te_usr_event_t e_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_DEVICE_CONTROL];
    // 画面描画フラグ
    bool b_scr_draw = false;
    // イベント判定
    switch (e_evt) {
    case EVT_SCR_INIT:
        // 初期処理判定
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "MSG ID:%s", s_com_status.c_msg_id);
#endif
        // 画面：初期処理
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルタイプ
        ps_scr_status->u8_cursor_row = 1;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 15;                   // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始:
        break;
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
        break;
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続通知エラー
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
        // MSG：ステータスエラー
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
        // ブート画面へ画面遷移
        v_evt_screen_change(SCR_ID_BOOT);
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
        // メッセージ探索
        ts_msg_info_t* ps_msg = ps_msg_top;
        while (ps_msg != NULL) {
            if (i_vutil_strcmp(ps_msg->c_msg_id, s_com_status.c_msg_id) == 0) {
                break;
            }
            // 次メッセージ
            ps_msg = ps_msg->ps_next;
        }
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
 * NAME: v_scr_boot
 *
 * DESCRIPTION:起動画面
 *
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_boot(te_usr_event_t e_evt) {
    //==========================================================================
    // 共通処理
    //==========================================================================
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_BOOT];

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (e_evt) {
    case EVT_SCR_INIT:
        // BLE：初期処理
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_SCR_INIT", SCR_ID_BOOT);
#endif
        // 接続ステータス更新：スキャン開始
        b_evt_upd_connect_sts(CON_STS_SCANNING);
        // 画面遷移
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_WAIT;    // カーソルウェイト
        ps_scr_status->u8_cursor_row = 1;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 12;                  // カーソル位置（列）
        // カーソル設定
        s_lcd_sts.e_cursor_type = ps_scr_status->e_cursor_type;
        s_lcd_sts.u8_cursor_row = ps_scr_status->u8_cursor_row;
        s_lcd_sts.u8_cursor_col = ps_scr_status->u8_cursor_col;
        // 描画バッファに書き込み
        strcpy(s_lcd_sts.c_buff[0], " Controller is  ");
        strcpy(s_lcd_sts.c_buff[1], "  scanning...   ");
        // 画面描画
        v_lcd_screen_drawing();
        v_lcd_cursor_drawing();
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_START", SCR_ID_BOOT);
#endif
        break;
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_RESULT", SCR_ID_BOOT);
#endif
        // スキャン結果を判定
        if (s_com_rmt_dev_sts.ps_scan_list != NULL) {
            // デバイス選択画面に画面遷移
            v_evt_screen_change(SCR_ID_DEVICE_SELECT);
        }
        break;
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_COMPLETE", SCR_ID_BOOT);
#endif
        // 接続ステータス更新：スキャン開始
        b_evt_upd_connect_sts(CON_STS_SCANNING);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 *
 * NAME: v_scr_device_select
 *
 * DESCRIPTION:デバイス選択画面のイベント処理
 *
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_device_select(te_usr_event_t e_evt) {
    //==========================================================================
    // 共通処理
    //==========================================================================
    // スキャン結果の有無を判定
    if (s_com_rmt_dev_sts.ps_scan_list == NULL) {
        // ブート画面に画面遷移
        v_evt_screen_change(SCR_ID_BOOT);
        return;
    }
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_DEVICE_SELECT];
    // 選択デバイス
    ts_com_ble_gap_device_info_t* ps_gsp_device = s_com_status.ps_gap_device;
    // 選択デバイスのチケット
    ts_com_msg_auth_ticket_t* ps_msg_ticket = &s_com_status.s_ticket;
    // 画面描画フラグ
    bool b_scr_draw = false;

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (e_evt) {
    case EVT_SCR_INIT:
        // 初期処理
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_SCR_INIT", SCR_ID_DEVICE_SELECT);
#endif
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルウェイト
        ps_scr_status->u8_cursor_row = 1;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 8;                   // カーソル位置（列）
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_START", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_RESULT", SCR_ID_DEVICE_SELECT);
#endif
        // 画面再描画
        b_scr_draw = true;
        break;
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_SCAN_COMPLETE", SCR_ID_DEVICE_SELECT);
#endif
        // 接続ステータス更新：スキャン開始
        b_evt_upd_connect_sts(CON_STS_SCANNING);
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_CONNECT", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続通知エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_BLE_CONNECT_ERROR", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_CONNECT:
        // MSG：接続通知
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_CONNECT", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_PAIR_CD_CHK:
        // MSG：ペアリング確認用コードチェック
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_CD_CHK", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_PAIR_OK:
        // MSG：ペアリング成功
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_PAIR_OK", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_PAIR_ERROR:
        // MSG：ペアリングエラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_PAIR_ERR", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_STS_OK:
        // MSG：ステータス正常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_STS_OK", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_STS_ERROR:
        // MSG：ステータス異常
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_STS_NG", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_MSG_RX_DATA", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御データ受信エラー
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ComEvt=EVT_MSG_RX_ERROR", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_TIMEOUT:
        // タイムアウト
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_TIMEOUT", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_INPUT_UP", SCR_ID_DEVICE_SELECT);
#endif
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // 選択デバイス変更
        if (sts_prev_device() == ESP_OK) {
            // 表示行を更新
            ps_scr_status->i_disp_row = s_com_status.u16_select_idx * 2;
            // 画面描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_INPUT_DOWN", SCR_ID_DEVICE_SELECT);
#endif
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // 選択デバイス変更
        if (sts_next_device() == ESP_OK) {
            // 表示行を更新
            ps_scr_status->i_disp_row = s_com_status.u16_select_idx * 2;
            // 画面描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_INPUT_LEFT", SCR_ID_DEVICE_SELECT);
#endif
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_INPUT_RIGHT", SCR_ID_DEVICE_SELECT);
#endif
        // 選択デバイスの有無を判定
        if (ps_gsp_device == NULL) {
            break;
        }
        // 接続ステータス更新：接続中
        b_evt_upd_connect_sts(CON_STS_CONNECTING);
        // カーソルをウェイトにする
        ps_scr_status->e_cursor_type = CURSOR_TYPE_WAIT;
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_INPUT_PUSH", SCR_ID_DEVICE_SELECT);
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
        // スクリーンバッファ編集
        //----------------------------------------------------------------------
        // バッファクリア
        strcpy(s_lcd_sts.c_buff[0], COM_LCD_EMPTY_LINE);
        strcpy(s_lcd_sts.c_buff[1], COM_LCD_EMPTY_LINE);
        // デバイス名を編集
        int i_name_len = i_vutil_strlen(ps_gsp_device->pc_name) % 17;
        if (i_name_len > 0) {
            memcpy(s_lcd_sts.c_buff[0], ps_gsp_device->pc_name, i_name_len);
        }
        // BLEアドレスを編集
        i_vutil_base64_encode(s_lcd_sts.c_buff[1], ps_gsp_device->t_bda, 6);
        // 送信カウント残数を編集
        if (ps_msg_ticket->u32_max_seq_no > 0) {
            sprintf(&s_lcd_sts.c_buff[1][8], " TX%05ld", ps_msg_ticket->u32_tx_seq_no);
        } else {
            strcpy(&s_lcd_sts.c_buff[1][8], " TX NONE");
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
 * NAME: v_scr_pairing_check
 *
 * DESCRIPTION:ペアリングチェック画面のイベント処理
 *
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_pairing_check(te_usr_event_t e_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_PAIRING_CHECK];
    // 画面描画フラグ
    bool b_scr_draw = false;

    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (e_evt) {
    case EVT_SCR_INIT:
#ifdef DEBUG_ALARM
        ESP_LOGI(LOG_MSG_TAG, "ScrID=%d ScrEvt=EVT_SCR_INIT", SCR_ID_PAIRING_CHECK);
        // 公開鍵の取得
        uint8_t u8_cli_key[BLE_MSG_PUBLIC_KEY_CLI_SIZE];
        uint8_t u8_svr_key[BLE_MSG_PUBLIC_KEY_SVR_SIZE];
        if (sts_com_msg_edit_public_key_pair(u8_cli_key, u8_svr_key) == ESP_OK) {
            v_dbg_disp_hex_data("CLI:", u8_cli_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
            v_dbg_disp_hex_data("SVR:", u8_svr_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
        }
#endif
        // ペアリング確認用コードを編集
        sts_msg_pairing_check_code_edit(s_com_status.c_pair_chk_code);
        // 画面：初期処理
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルタイプ
        ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 11;                  // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始
        break;
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
        break;
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続通知エラー
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
        // MSG：ステータスエラー
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御メッセージ受信
        break;
    case EVT_TIMEOUT:
        // タイムアウト
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
        if (ps_scr_status->i_disp_row != 0) {
            // 画面描画
            ps_scr_status->i_disp_row = 0;
            // カーソル表示
            if (ps_scr_status->e_cursor_type != CURSOR_TYPE_NONE) {
                ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY;
            }
            // 画面再描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
        if (ps_scr_status->i_disp_row == 0) {
            // 画面描画
            ps_scr_status->i_disp_row = 1;
            // カーソル非表示
            if (ps_scr_status->e_cursor_type != CURSOR_TYPE_NONE) {
                ps_scr_status->e_cursor_type = CURSOR_TYPE_WAIT;
            }
            // 画面再描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
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
        // カーソルタイプ判定
        if (ps_scr_status->e_cursor_type != CURSOR_TYPE_DISPLAY) {
            break;
        }
        // MSGコード確認
        if (ps_scr_status->u8_cursor_col == 11) {
            // ペアリング認証（ダイジェスト比較結果の通知）
            if (sts_com_msg_tx_pairing_certification(true, BLE_MSG_MAX_SEQ_NO) != ESP_OK) {
                // 接続ステータス更新：切断
                b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
                // メッセージ表示に画面遷移
                v_evt_show_error_msg(COM_MSG_ID_ERR_TXRX);
            }
        } else {
            // ペアリングエラー
            sts_com_msg_tx_pairing_certification(false, BLE_MSG_MAX_SEQ_NO);
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
        // バッファクリア
        strcpy(s_lcd_sts.c_buff[0], COM_LCD_EMPTY_LINE);
        strcpy(s_lcd_sts.c_buff[1], COM_LCD_EMPTY_LINE);
        // 表示列判定
        if (ps_scr_status->i_disp_row == 0) {
            // ペアリング確認用コード編集
            strcpy(s_lcd_sts.c_buff[0], "CODE CHECK OK/NG");
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
 * NAME: v_scr_device_control
 *
 * DESCRIPTION:デバイスコントロール画面のイベント処理
 *
 * PARAMETERS:      Name        RW  Usage
 * te_usr_event_t   e_evt       R   イベント
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_scr_device_control(te_usr_event_t e_evt) {
    // 画面ステータス
    ts_scr_status_t* ps_scr_status = &s_scr_sts_list[SCR_ID_DEVICE_CONTROL];
    // 画面描画フラグ
    bool b_scr_draw = false;
    //==========================================================================
    // イベント判定
    //==========================================================================
    switch (e_evt) {
    case EVT_SCR_INIT:
        // 初期処理判定
        // 画面：初期処理
        ps_scr_status->i_disp_row    = 0;                   // 表示位置（行）
        ps_scr_status->e_cursor_type = CURSOR_TYPE_DISPLAY; // カーソルタイプ
        ps_scr_status->u8_cursor_row = 0;                   // カーソル位置（行）
        ps_scr_status->u8_cursor_col = 0;                   // カーソル位置（列）
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_BLE_SCAN_START:
        // BLE：スキャン開始
        break;
    case EVT_BLE_SCAN_RESULT:
        // BLE：スキャン結果取得
        break;
    case EVT_BLE_SCAN_COMPLETE:
        // BLE：スキャン完了
        break;
    case EVT_BLE_CONNECT:
        // BLE：接続通知
        break;
    case EVT_BLE_CONNECT_ERROR:
        // BLE：接続通知エラー
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
        // MSG：ステータスエラー
        break;
    case EVT_MSG_RX_DATA:
        // MSG：制御データ受信
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_MSG_RX_ERROR:
        // MSG：制御メッセージ受信
        // 画面描画
        b_scr_draw = true;
        break;
    case EVT_TIMEOUT:
        // タイムアウト
        break;
    case EVT_INPUT_UP:
        // キー入力（上）
        if (s_com_status.e_operating_mode != OPR_MODE_ALARM &&
            ps_scr_status->u8_cursor_row > 0) {
            ps_scr_status->u8_cursor_row--;
            // 画面描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_DOWN:
        // キー入力：下
        if (s_com_status.e_operating_mode != OPR_MODE_ALARM &&
            ps_scr_status->u8_cursor_row < 1) {
            ps_scr_status->u8_cursor_row++;
            // 画面描画
            b_scr_draw = true;
        }
        break;
    case EVT_INPUT_LEFT:
        // キー入力：左
        break;
    case EVT_INPUT_RIGHT:
        // キー入力：右
        break;
    case EVT_INPUT_PUSH:
        // キー入力：プッシュ
        if (ps_scr_status->u8_cursor_row == 0) {
            // 通常モード/警戒モード
            if (s_com_status.e_operating_mode == OPR_MODE_NORMAL) {
                // 警戒モードへ移行
                sts_tx_ctrl_msg(CTL_CMD_UPDATE, OPR_MODE_ALERT);
            } else if (s_com_status.e_operating_mode == OPR_MODE_ALERT) {
                // 通常モードへ移行
                sts_tx_ctrl_msg(CTL_CMD_UPDATE, OPR_MODE_NORMAL);
            }
        } else {
            // ペアリング解除メッセージ送信
            sts_tx_ctrl_msg(CTL_CMD_UNPAIR, OPR_MODE_NORMAL);
            // 選択デバイスアドレス取得
            esp_bd_addr_t t_bda;
            if (s_com_status.ps_gap_device != NULL) {
                v_com_ble_addr_cpy(t_bda, s_com_status.ps_gap_device->t_bda);
            } else {
                v_com_ble_addr_clear(t_bda);
            }
            // 選択デバイスID
            uint64_t u64_device_id = s_com_status.s_ticket.u64_rmt_device_id;
            // 切断処理
            b_evt_upd_connect_sts(CON_STS_DISCONNECTED);
            // ペアリング解除
            v_evt_unpairing(t_bda, u64_device_id);
            // 起動画面への画面遷移
            v_evt_screen_change(SCR_ID_BOOT);
        }
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
        // バッファクリア
        strcpy(s_lcd_sts.c_buff[0], COM_LCD_EMPTY_LINE);
        strcpy(s_lcd_sts.c_buff[1], COM_LCD_EMPTY_LINE);
        // 制御メニュー編集
        if (s_com_status.e_operating_mode == OPR_MODE_NORMAL) {
            // 通常モード
            strcpy(s_lcd_sts.c_buff[0], "Normal mode");
            strcpy(s_lcd_sts.c_buff[1], "Unpair");
        } else if (s_com_status.e_operating_mode == OPR_MODE_ALERT) {
            // 警戒モード
            strcpy(s_lcd_sts.c_buff[0], "Alert mode");
            strcpy(s_lcd_sts.c_buff[1], "Unpair");
        } else if (s_com_status.e_operating_mode == OPR_MODE_ALARM) {
            // 警報モード
            strcpy(s_lcd_sts.c_buff[0], "Alarm mode");
            strcpy(s_lcd_sts.c_buff[1], "Unpair");
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

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
