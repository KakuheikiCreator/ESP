/*******************************************************************************
 *
 * MODULE :GATT Messager Library functions source file
 *
 * CREATED:2021/06/10 00:32:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:GATTメッセンジャーのテストコード
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

//#define configCHECK_FOR_STACK_OVERFLOW  2
//#define configUSE_MALLOC_FAILED_HOOK    1

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_bt.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <freertos/task.h>

#include "ntfw_com_value_util.h"
#include "ntfw_com_data_model.h"
#include "ntfw_com_debug_util.h"
#include "ntfw_ble_msg.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** デバッグ */
//#define GATTS_MSG_SERVER

/** ログ出力タグ */
#define LOG_MSG_TAG "BLE_MSG_SVR"

/** データ型サイズ：uint8_t */
#define DEF_SIZE_CHAR   (sizeof(uint8_t))

/** BLE GAPデバイス名 */
#define BLE_GAP_DEVICE_NAME         "BLE_MSG_SVR:0000"
#define BLE_GAP_SVR_PASSKEY         (123456)  // サーバーパスキー
#define BLE_GAP_CLI_PASSKEY         (654321)  // クライアントパスキー

/** BLE GATTアプリケーションID */
#define BLE_GATT_APP_ID             (0x88)
/** BLE GATTプロファイルで管理するインターフェース数 */
#define BLE_GATT_IF_CNT             (1)
/** BLE GATTプロファイル情報配列のSPPインターフェースインデックス */
#define BLE_GATT_SVC_IDX            (0)


/** UARTタスクの優先度 */
#define BLE_MSG_UART_PRIORITIES     (configMAX_PRIORITIES - 5)
/** 受信ウェイトタイム */
#define BLE_MSG_RX_WAIT_TICK        (100 / portTICK_PERIOD_MS)
/** 送信ウェイトタイム */
#define BLE_MSG_TX_WAIT_TICK        (100 / portTICK_PERIOD_MS)
/** 主処理のウェイト時間 */
#define BLE_MSG_MAIN_WAIT_TICK      (1000 / portTICK_PERIOD_MS)
/** メッセージイベントウェイトタイム */
#define BLE_MSG_EVT_WAIT_TICK       (1000 / portTICK_PERIOD_MS)
/** メッセージデバイスID */
#define BLE_MSG_DEVICE_ID           (0x00000000000000F0)
/** 最大メッセージサイズ */
#define BLE_MSG_MAX_SIZE            (2048)

/** チケットリストサイズ */
#define MSG_TICKET_LIST_SIZE        (16)
/** メッセージ最大シーケンスサイズ */
#define MSG_MAX_SEQ_NO              (0xFFFFFFFF)


/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * チケットリスト
 */
typedef struct {
    uint32_t u32_size;
    ts_com_msg_auth_ticket_t s_ticket_list[MSG_TICKET_LIST_SIZE];
} ts_ticket_list_t;


/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;

/** チケットリスト */
static ts_ticket_list_t s_ticket_list = {
    .u32_size = 0,
};

// 製造元データポイント
static uint8_t ble_manufacturer[3] = {'E', 'S', 'P'};
/** サービス（オリジナル）のUUID */
// TODO: サービスの識別子（BASE UUID | サービスID）
static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
//    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
    0x76, 0xf1, 0x47, 0xed, 0x23, 0x2b, 0x4a, 0x58, 0xa5, 0xd4, 0xc7, 0x51, 0xAB, 0xF0, 0x00, 0x00
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
//    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
//    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
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

//==============================================================================
// アトリビュートＤＢで管理する値の構造体定義
//==============================================================================
// UART送信イベントキュー
static QueueHandle_t t_uart_tx_evt_queue = NULL;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** 初期処理 */
static void v_app_init();
/** GAPプロファイルのイベントコールバック */
static void v_gap_adv_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param);
/** メッセージイベント処理 */
static void v_msg_evt_cb(te_com_ble_msg_event e_msg_evt);
/** チケットイベント処理 */
static esp_err_t sts_msg_ticket_cb(te_com_ble_msg_ticket_evt_t e_evt, ts_com_msg_auth_ticket_t* ps_ticket);
/** チケット生成処理 */
static ts_com_msg_auth_ticket_t* ps_msg_ticket_create(uint64_t u64_device_id);
/** チケット検索処理 */
static ts_com_msg_auth_ticket_t* ps_msg_ticket_read(uint64_t u64_device_id);
/** チケット削除処理 */
static void v_msg_ticket_delete(uint64_t u64_device_id);
/** メッセージ受信タスク */
static void v_msg_task_rx(void* pvParameters);
/** メッセージ送信タスク */
static void v_msg_task_tx(void* pvParameters);


/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/**
 * 主処理
 */
void app_main() {
    //**************************************************************************
    // ESP32の共通初期処理
    //**************************************************************************
    v_app_init();

    //**************************************************************************
    // メッセージ処理タスクの開始
    //**************************************************************************
    portBASE_TYPE b_type;
    // メッセージ受信タスクの開始
    b_type = xTaskCreate(v_msg_task_rx, "uart_rx_task", 8192, (void*)UART_NUM_0, BLE_MSG_UART_PRIORITIES, NULL);
    if (b_type != pdPASS) {
        ESP_LOGI(LOG_MSG_TAG, "Task Create Err:spp_rx_task");
    }
    // メッセージ送信タスクの開始
    b_type = xTaskCreate(v_msg_task_tx, "uart_tx_task", 8192, (void*)UART_NUM_0, BLE_MSG_UART_PRIORITIES, NULL);
    if (b_type != pdPASS) {
        ESP_LOGI(LOG_MSG_TAG, "Task Create Err:spp_tx_task");
    }

    //**************************************************************************
    // 主処理
    //**************************************************************************
    // リモートアドレス
    esp_bd_addr_t t_rmt_bda;
#ifdef GATTS_MSG_SERVER
    // デバイスID
    uint64_t u64_device_id;
    // ステータス値
    esp_err_t sts_val = ESP_OK;
#endif
    // 主処理
    while(true) {
        //======================================================================
        // ウェイト
        //======================================================================
        vTaskDelay(BLE_MSG_MAIN_WAIT_TICK);

        //======================================================================
        // 接続状況モニタリング
        //======================================================================
        // 接続ステータス
        te_com_ble_msg_connection_sts_t e_sts = e_com_msg_connection_sts();
        // 接続状況を表示
        if (e_sts == COM_BLE_MSG_CON_CONNECTED) {
#ifdef GATTS_MSG_SERVER
            // 接続先デバイスIDの編集
            sts_val = sts_com_msg_edit_remote_dev_id(&u64_device_id);
            if (sts_val != ESP_OK) {
                ESP_LOGE(LOG_MSG_TAG, "%s Message client DeviceID not found! sts=%d", __func__, sts_val);
                // リトライ
                continue;
            }
            // メッセージ機能のペアリング判定
            if (b_com_msg_is_paired(u64_device_id)) {
                // 接続状況を表示
                ESP_LOGE(LOG_MSG_TAG, "%s L#%d Message client Paired!", __func__, __LINE__);
            }
#endif
            continue;
        }
        // パスキー応答
        if (e_sts == COM_BLE_MSG_CON_WAIT_PASSKEY) {
            sts_com_ble_gap_adv_edit_remote_bda(t_rmt_bda);
#ifdef GATTS_MSG_SERVER
            tc_com_ble_bda_string_t tc_bda;
            v_com_ble_address_to_str(tc_bda, t_rmt_bda);
            ESP_LOGE(LOG_MSG_TAG, "%s L#%d passkey_reply bda=%s", __func__, __LINE__, tc_bda);
#endif
            // ※本来はピアデバイスに表示されている値を返信？
            sts_com_ble_gap_passkey_reply(t_rmt_bda, true, BLE_GAP_CLI_PASSKEY);
        }
        // 番号チェック
        if (e_sts == COM_BLE_MSG_CON_WAIT_NUM_CHK) {
            sts_com_ble_gap_adv_edit_remote_bda(t_rmt_bda);
#ifdef GATTS_MSG_SERVER
            tc_com_ble_bda_string_t tc_bda;
            v_com_ble_address_to_str(tc_bda, t_rmt_bda);
            ESP_LOGE(LOG_MSG_TAG, "%s L#%d NC bda=%s", __func__, __LINE__, tc_bda);
#endif
            // チェック結果返信
            // ※本来はピアデバイスに表示されている値と比較して、一致する事を確認した上で返信
            sts_com_ble_gap_confirm_reply(t_rmt_bda, true);
        }
    }

    //**************************************************************************
    // mainの終端処理
    //**************************************************************************
    // 無限にウェイト
    vTaskDelay(portMAX_DELAY);
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_app_init
 *
 * DESCRIPTION:初期処理
 *
 * PARAMETERS:              Name        RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_app_init() {
    //==========================================================================
    // ESP32の共通初期処理
    //==========================================================================
    // NVS領域をクリア、エラー時には異常終了する
    ESP_ERROR_CHECK(nvs_flash_erase());
    // Initialize NVS.
    // 不揮発性メモリ領域のデフォルトパーティションを初期化する
    esp_err_t sts_ret = nvs_flash_init();
    // NVS領域を初期化した際のエラー判定として次の場合を判定
    // 1.NVSの空きページが無い
    // 2.新バージョンのデータがパーティションに含まれている
    if (sts_ret == ESP_ERR_NVS_NO_FREE_PAGES || sts_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS領域をクリア、エラー時には異常終了する
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(sts_ret);

    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    s_mutex = xSemaphoreCreateRecursiveMutex();

    //==========================================================================
    // ログ出力設定
    //==========================================================================
//    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set("*", ESP_LOG_INFO);

    //==========================================================================
    // URART初期処理
    //==========================================================================
    // UART Config
    uart_config_t uart_config = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_RTS,
         .rx_flow_ctrl_thresh = 122,
    };
    // Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // 送信イベントキューの生成
    t_uart_tx_evt_queue = xQueueCreate(32, sizeof(uint8_t));
    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 4096, 32, &t_uart_tx_evt_queue, 0);

    //==========================================================================
    // デバッグ機能の初期化
    //==========================================================================
    v_dbg_register_failed_alloc();

    //==========================================================================
    // BLEの初期化処理
    //==========================================================================
    // BLE初期化処理
    sts_ret = sts_com_ble_init();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "MAIN %s init controller failed: %s", __func__, esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // 電波の出力設定
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    // ボンディングデバイス表示
    sts_ret = sts_com_ble_display_bonded_devices();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "bonding devaice display error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // ボンディングデバイス削除
    sts_ret = sts_com_ble_disbonding_all();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "bonding devaice remove error:%s", esp_err_to_name(sts_ret));
#endif
        return;
    }

    //==========================================================================
    // セキュリティ マネージャー プロトコル（SMP）設定
    //==========================================================================
    // SMP設定の編集
    ts_com_ble_gap_config_t s_ble_gap_cfg;
    s_ble_gap_cfg.pc_device_name  = BLE_GAP_DEVICE_NAME;
    s_ble_gap_cfg.t_auth_req      = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    s_ble_gap_cfg.t_iocap         = ESP_IO_CAP_KBDISP;
//    s_ble_gap_cfg.u8_init_key     = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK | ESP_BLE_CSR_KEY_MASK;
//    s_ble_gap_cfg.u8_rsp_key      = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK | ESP_BLE_CSR_KEY_MASK;
    s_ble_gap_cfg.u8_init_key     = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    s_ble_gap_cfg.u8_rsp_key      = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    s_ble_gap_cfg.u8_max_key_size = 16;
    s_ble_gap_cfg.u8_auth_option  = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    s_ble_gap_cfg.v_callback      = v_gap_adv_event_cb;
    // SMP設定
    sts_ret = sts_com_ble_gap_smp_init(s_ble_gap_cfg);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：SMP初期処理
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_smp_init error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // アドバタイジングデータの設定処理
    sts_ret = sts_com_ble_gap_set_adv_data(&gap_adv_config);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：アドバタイズデータの設定エラー
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_set_adv_data error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // スキャン応答データの設定処理
    sts_ret = sts_com_ble_gap_set_adv_data(&gap_scan_rsp_config);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：スキャン応答データの設定エラー
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_set_adv_data error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // アドバタイジングの開始処理
    sts_ret = sts_com_ble_gap_start_advertising(&gap_adv_params);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：アドバタイズ開始
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_start_advertising error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }

    //==========================================================================
    // GATTサーバー初期設定
    //==========================================================================
    sts_com_ble_gatts_init();
    /** GATTサーバーのSPPアプリケーション設定の生成処理 */
    s_gatts_cfg_tbls = s_com_ble_spps_config(ESP_GATT_PERM_READ_ENC_MITM, ESP_GATT_PERM_WRITE_SIGNED_MITM);
    s_gatts_cfg_tbls.u16_app_id = BLE_GATT_APP_ID;          // アプリケーションID
    s_gatts_cfg_tbls.e_con_sec  = ESP_BLE_SEC_ENCRYPT_MITM; // 接続のセキュリティモード
    // アプリケーション設定を登録
    esp_err_t sts_val = sts_com_ble_gatts_app_register(&s_gatts_cfg_tbls);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "message server app register error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }

    //==========================================================================
    // BLEメッセージ初期処理
    //==========================================================================
    // メッセージサーバー初期処理
    sts_val = sts_com_msg_init_svr(BLE_GATT_APP_ID, BLE_MSG_DEVICE_ID, BLE_MSG_MAX_SIZE, v_msg_evt_cb, sts_msg_ticket_cb);
    if (sts_val != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "message server initialize error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // ペアリング機能の設定
    v_com_msg_config_pairing(true);
    // ステータスチェック機能の設定
    v_com_msg_config_sts_chk(true);
    // 受信メッセージのエンキュー有効化処理
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_DATA);         // データ
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_CIPHERTEXT);   // 暗号データ
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
static void v_gap_adv_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param) {
    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, BLE_MSG_EVT_WAIT_TICK) != pdTRUE) {
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d Take fail", __func__, __LINE__);
#endif
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    // 初期設定通り共通機能が実行してくれるので、ここではアドバタイズスタート時にPINコード設定のみ
    //esp_err_t sts_val = ESP_OK;
    switch (e_event) {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // BLEのGAPプロファイルにおけるPINコードの設定処理
        // ペアリングの際のPINコードは数字６桁の固定値、型はuint32_t
#ifdef GATTS_MSG_SERVER
        ESP_LOGI(LOG_MSG_TAG, "sts_com_ble_gap_set_static_pass_key:%d", BLE_GAP_SVR_PASSKEY);
#endif
        // パスキー設定
        sts_com_ble_gap_set_static_pass_key(BLE_GAP_SVR_PASSKEY);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // パスキーの返信要求 ※サーバー側とスキャン側の両方にある
#ifdef GATTS_MSG_SERVER
    {
        tc_com_ble_bda_string_t tc_bda;
        v_com_ble_address_to_str(tc_bda, pu_param->ble_security.ble_req.bd_addr);
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d gap_passkey_reply", __func__, __LINE__);
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d :address = %s", __func__, __LINE__, tc_bda);
    }
#endif
        // ※本来はピアデバイスに表示されている値を返信？
        //sts_com_ble_gap_passkey_reply(pu_param->ble_security.ble_req.bd_addr, true, BLE_GAP_CLI_PASSKEY);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // パスキーの確認要求 ※スキャン側にもある
        // すべき事：PINコード認証の場合には、相手デバイスに表示されているPINコードの確認結果を返信
        // IOにDisplayYesNO機能があり、ピアデバイスIOにもDisplayYesNo機能がある場合、アプリはこのevtを受け取ります。
        // パスキーをユーザーに表示し、ピアデバイスにて表示される番号と一致するか確認した結果を返信する
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer deivce. */
        break;
    default:
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
 * te_com_ble_msg_event     e_msg_evt   R   イベント種別
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
    if (xSemaphoreTakeRecursive(s_mutex, BLE_MSG_EVT_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    esp_err_t  sts_val = ESP_OK;
    do {
        // イベント毎の処理
        switch (e_msg_evt) {
        case COM_BLE_MSG_EVT_RX_RESPONSE:
            // 応答を受信
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_RESPONSE");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_RESET:
            // リセットメッセージ受信
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_RESET");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_PING:
            // PINGメッセージ受信
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_PING");
#endif
            break;
#ifdef GATTS_MSG_SERVER
        case COM_BLE_MSG_EVT_GATT_CONNECT:
            // GATT接続
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_GATT_CONNECT");
            break;
        case COM_BLE_MSG_EVT_GATT_DISCONNECT:
            // GATT切断
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_GATT_DISCONNECT");
            break;
#endif
        case COM_BLE_MSG_EVT_OPEN_SUCCESS:
            // オープン成功
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_OPEN_SUCCESS");
#endif
            break;
        case COM_BLE_MSG_EVT_OPEN_TIMEOUT:
            // オープンタイムアウト
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_OPEN_TIMEOUT");
#endif
            break;
        case COM_BLE_MSG_EVT_PAIRING_START:
            // ペアリング開始
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_START");
#endif
            // とりあえず無条件にペアリングOKとする
            sts_val = sts_com_msg_tx_pairing_certification(true, 0xFFFFFFFF);
            if (sts_val != ESP_OK) {
                ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts=%d", __func__, __LINE__, sts_val);
            }
            break;
        case COM_BLE_MSG_EVT_PAIRING_SUCCESS:
            // ペアリング成功
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_SUCCESS");
#endif
            // ペアリング認証（ダイジェスト比較結果の通知）
            break;
        case COM_BLE_MSG_EVT_PAIRING_ERR:
            // ペアリングエラー
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_ERR");
#endif
            break;
        case COM_BLE_MSG_EVT_PAIRING_TIMEOUT:
            // ペアリングタイムアウト
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_TIMEOUT");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_CHK:
            // ステータスチェック
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_CHK");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_OK:
            // ステータス正常
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_OK");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_ERR:
            // ステータス異常
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_ERR");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_TIMEOUT:
            // ステータスチェックタイムアウト
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_TIMEOUT");
#endif
            break;
        case COM_BLE_MSG_EVT_HANDLING_ERR:
            // メッセージハンドリングエラー
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_HANDLING_ERR");
#endif
            break;
        default:
            // メッセージハンドリングエラー
#ifdef GATTS_MSG_SERVER
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=%d", e_msg_evt);
#endif
            break;
        }
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);
}

/*******************************************************************************
 *
 * NAME: v_msg_ticket_cb
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
    if (xSemaphoreTakeRecursive(s_mutex, BLE_MSG_EVT_WAIT_TICK) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    // チケットの取得
    ts_com_msg_auth_ticket_t* ps_wk_ticket = NULL;
    do {
        // イベント毎の処理
#ifdef GATTS_MSG_SERVER
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d id=%llu evt=%d", __func__, __LINE__, ps_ticket->u64_rmt_device_id, e_evt);
#endif
        switch (e_evt) {
        case COM_BLE_MSG_TICKET_EVT_CREATE:
            // チケット生成
            ps_wk_ticket = ps_msg_ticket_create(ps_ticket->u64_rmt_device_id);
            // チケットの有無判定
            if (ps_wk_ticket == NULL) {
                sts_val = ESP_ERR_NOT_FOUND;
                break;
            }
            // 結果を編集
            *ps_wk_ticket = *ps_ticket;
            break;
        case COM_BLE_MSG_TICKET_EVT_READ:
            // チケット読み込み
            ps_wk_ticket = ps_msg_ticket_read(ps_ticket->u64_rmt_device_id);
            // チケットの有無判定
            if (ps_wk_ticket == NULL) {
                sts_val = ESP_ERR_NOT_FOUND;
                break;
            }
            // 結果を編集
            *ps_ticket = *ps_wk_ticket;
            break;
        case COM_BLE_MSG_TICKET_EVT_UPDATE:
            // チケット更新
            ps_wk_ticket = ps_msg_ticket_read(ps_ticket->u64_rmt_device_id);
            // チケットの有無判定
            if (ps_wk_ticket == NULL) {
                sts_val = ESP_ERR_NOT_FOUND;
                break;
            }
            // 結果を編集
            *ps_wk_ticket = *ps_ticket;
            break;
        case COM_BLE_MSG_TICKET_EVT_DELETE:
            // チケット削除
            v_msg_ticket_delete(ps_ticket->u64_rmt_device_id);
            break;
        }
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 正常終了
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: ps_msg_ticket_create
 *
 * DESCRIPTION:チケット生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 * uint64_t         u64_device_id   R   デバイスID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_msg_auth_ticket_t* ps_msg_ticket_create(uint64_t u64_device_id) {
    // チケットの追加設定
    if (s_ticket_list.u32_size >= MSG_TICKET_LIST_SIZE) {
        return NULL;
    }
    // チケット読み込み
    ts_com_msg_auth_ticket_t* ps_ticket = ps_msg_ticket_read(BLE_MSG_DEVICE_ID);
    if (ps_ticket == NULL) {
        // 新しいチケットを割り当て
        ts_com_msg_auth_ticket_t* ps_list = s_ticket_list.s_ticket_list;
        ps_ticket = &ps_list[s_ticket_list.u32_size];
    }
    // チケットの編集
    ps_ticket->u64_own_device_id = BLE_MSG_DEVICE_ID;      // 自デバイスID
    ps_ticket->u64_rmt_device_id = u64_device_id;       // 相手デバイスID
    // 暗号鍵
    memset(ps_ticket->u8_enc_key, 0x00, COM_MSG_SIZE_CIPHER_KEY);
    // 自ステータス
    memset(ps_ticket->u8_own_sts, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 相手ステータスハッシュ
    memset(ps_ticket->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
    ps_ticket->u32_max_seq_no = 0;                      // 最大シーケンス番号
    ps_ticket->u32_tx_seq_no  = 0;                      // 送信シーケンス番号
    ps_ticket->u32_rx_seq_no  = 0;                      // 受信シーケンス番号
    // チケット件数を更新
    s_ticket_list.u32_size++;
    // 編集したチケット返却
    return ps_ticket;
}

/*******************************************************************************
 *
 * NAME: ps_msg_ticket_read
 *
 * DESCRIPTION:チケット検索関数
 *
 * PARAMETERS:      Name            RW  Usage
 * uint64_t         u64_device_id   R   デバイスID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static ts_com_msg_auth_ticket_t* ps_msg_ticket_read(uint64_t u64_device_id) {
    // チケットの探索
    ts_com_msg_auth_ticket_t* ps_list = s_ticket_list.s_ticket_list;
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < s_ticket_list.u32_size; u32_idx++) {
        if (ps_list[u32_idx].u64_rmt_device_id == u64_device_id) {
            return &ps_list[u32_idx];
        }
    }
    // 対象チケットなし
    return NULL;
}

/*******************************************************************************
 *
 * NAME: v_msg_ticket_delete
 *
 * DESCRIPTION:チケット削除処理
 *
 * PARAMETERS:      Name            RW  Usage
 * uint64_t         u64_device_id   R   デバイスID
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_ticket_delete(uint64_t u64_device_id) {
    // チケットの探索
    ts_com_msg_auth_ticket_t* ps_ticket = ps_msg_ticket_read(u64_device_id);
    // チケットの編集
    ps_ticket->u64_own_device_id = BLE_MSG_DEVICE_ID;      // 自デバイスID
    ps_ticket->u64_rmt_device_id = BLE_MSG_DEVICE_ID;      // 相手デバイスID
    // 暗号鍵
    memset(ps_ticket->u8_enc_key, 0x00, COM_MSG_SIZE_CIPHER_KEY);
    // 自ステータス
    memset(ps_ticket->u8_own_sts, 0x00, COM_MSG_SIZE_TICKET_STS);
    // 相手ステータスハッシュ
    memset(ps_ticket->u8_rmt_sts_hash, 0x00, COM_MSG_SIZE_TICKET_STS);
    ps_ticket->u32_max_seq_no = 0;                      // 最大シーケンス番号
    ps_ticket->u32_tx_seq_no  = 0;                      // 送信シーケンス番号
    ps_ticket->u32_rx_seq_no  = 0;                      // 受信シーケンス番号
}

/*******************************************************************************
 *
 * NAME: v_msg_task_rx
 *
 * DESCRIPTION:メッセージの受信処理タスク
 *
 * PARAMETERS:  Name            RW  Usage
 * void*        pvParameters    R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_task_rx(void *pvParameters) {
    // メッセージ
    ts_com_msg_t* ps_msg = NULL;
    // 送信データ
    ts_u8_array_t* ps_data = NULL;
    for (;;) {
        //======================================================================
        // メッセージ受信
        //======================================================================
        ps_msg = ps_com_msg_rx_msg(portMAX_DELAY);
        if (ps_msg == NULL) {
#ifdef GATTS_MSG_SERVER
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "RX Error!!!");
#endif
            continue;
        }

        //======================================================================
        // 受信イベント処理
        //======================================================================
        // UARTデータ書き込み
        ps_data = ps_msg->ps_data;
        uart_write_bytes(UART_NUM_0, ps_data->pu8_values, ps_data->t_size);
        // 受信データ解放
        sts_com_msg_delete_msg(ps_msg);
        ps_msg = NULL;
    }
    vTaskDelete(NULL);
}

/*******************************************************************************
 *
 * NAME: v_msg_task_tx
 *
 * DESCRIPTION:メッセージの送信処理タスク
 *
 * PARAMETERS:  Name            RW  Usage
 * void*        pvParameters    R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_msg_task_tx(void* pvParameters) {
    // 接続中デバイスID
    uint64_t u64_device_id = BLE_MSG_DEVICE_ID;
    // UARTイベント
    uart_event_t s_event;
    // UARTデータサイズ
    size_t t_data_len;
    // 送信データ
    ts_u8_array_t* ps_data = NULL;
    for (;;) {
        //======================================================================
        // シリアルポートからの値入力
        //======================================================================
        // シリアルポートからの値入力検知
        if (xQueueReceive(t_uart_tx_evt_queue, (void*)&s_event, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // イベント・受信データサイズの判定
        if (s_event.type != UART_DATA || s_event.size == 0) {
            continue;
        }
        // UARTのReadバッファを生成
        uart_get_buffered_data_len(UART_NUM_0, &t_data_len);
        // 読み込みデータバッファの解放
        sts_mdl_delete_u8_array(ps_data);
        // データバッファを生成
        ps_data = ps_mdl_empty_u8_array(t_data_len);
        if (ps_data == NULL) {
            continue;
        }
        // UARTのReadバッファを読み込み
        uart_read_bytes(UART_NUM_0, ps_data->pu8_values, ps_data->t_size, portMAX_DELAY);
        // デバッグ出力
#ifdef GATTS_MSG_SERVER
        char c_dbg_str[ps_data->t_size + 1];
        memcpy(c_dbg_str, ps_data->pu8_values, ps_data->t_size);
        c_dbg_str[ps_data->t_size] = '\0';
        ESP_LOGI(LOG_MSG_TAG, "Read=%s", c_dbg_str);
#endif

        //======================================================================
        // クリティカルセクション
        //======================================================================
        if (xSemaphoreTakeRecursive(s_mutex, BLE_MSG_TX_WAIT_TICK) != pdTRUE) {
            continue;
        }

        //======================================================================
        // 送信処理
        //======================================================================
        do {
            //------------------------------------------------------------------
            // 接続
            //------------------------------------------------------------------
            // 接続確認
            if (e_com_msg_connection_sts() != COM_BLE_MSG_CON_CONNECTED) {
#ifdef GATTS_MSG_SERVER
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "disconnected!!! sts=%d", e_com_msg_connection_sts());
#endif
                break;
            }
            // 接続先デバイスIDの取得
            if (sts_com_msg_edit_remote_dev_id(&u64_device_id) != ESP_OK) {
#ifdef GATTS_MSG_SERVER
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "Unable to obtain device ID!!!");
#endif
                break;
            }
            // 平文メッセージの送信処理
//            if (sts_com_msg_tx_plain_msg(u64_device_id, ps_data) != ESP_OK) {
//#ifdef GATTS_MSG_SERVER
//        // デバッグ出力
//        ESP_LOGE(LOG_MSG_TAG, "TX Error!!!");
//#endif
//                break;
//            }
            // 暗号文メッセージの送信処理
            if (sts_com_msg_tx_cipher_msg(u64_device_id, ps_data) != ESP_OK) {
#ifdef GATTS_MSG_SERVER
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "TX Error!!!");
#endif
                break;
            }
        } while(false);

        //======================================================================
        // クリティカルセクション終了
        //======================================================================
        xSemaphoreGiveRecursive(s_mutex);
    }
    vTaskDelete(NULL);
}

/*******************************************************************************
 *
 * NAME: vApplicationStackOverflowHook
 *
 * DESCRIPTION:メモリアロケーションエラーのフック関数
 *
 * PARAMETERS:      Name            RW  Usage
 * TaskHandle_t*    pxTask          R   タスクハンドル
 * signed char*     pcTaskName      R   タスク名
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
#ifdef GATTS_MSG_SERVER
//void vApplicationStackOverflowHook(TaskHandle_t* pxTask, signed char* pcTaskName) {
//    ESP_LOGE(LOG_MSG_TAG, "StackOverflow=%s", pcTaskName);
//}
#endif

/*******************************************************************************
 *
 * NAME: vApplicationMallocFailedHook
 *
 * DESCRIPTION:メモリアロケーションエラーのフック関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
#ifdef GATTS_MSG_SERVER
void vApplicationMallocFailedHook() {
    ESP_LOGE(LOG_MSG_TAG, "Malloc ERR");
}
#endif

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
