/*******************************************************************************
 *
 * MODULE :GATT Messager Library functions source file
 *
 * CREATED:2021/06/10 00:32:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:GATTメッセンジャークライアント
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <esp_log.h>
#include <esp_bt.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include "ntfw_com_date_time.h"
#include "ntfw_ble_msg.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** デバッグ */
//#define GATTC_MSG_CLIENT

/** ログ出力タグ */
#define LOG_MSG_TAG "BLE_MSG_CLI"

/** データ型サイズ：uint8_t */
#define DEF_SIZE_CHAR       (sizeof(uint8_t))

/** 初期処理のウェイト時間 */
#define INIT_WAIT_TICK      (100 / portTICK_PERIOD_MS)

/** 主処理のウェイト時間 */
#define MAIN_WAIT_TICK      (1000 / portTICK_PERIOD_MS)

/** 主処理のウェイト時間 */
#define EVT_TAKE_WAIT_TICK  (1000 / portTICK_PERIOD_MS)

/** リトライのウェイト時間 */
#define EVT_RETRY_TICK      (100 / portTICK_PERIOD_MS)

/** BLE GAPデバイスステータスの更新ウェイト時間 */
#define BLE_GAP_STS_UPD_WAIT_TICK   (60000 / portTICK_PERIOD_MS)
/** BLE GATT待ちウェイト時間 */
#define BLE_GATT_STS_UPD_WAIT_TICK  (1000 / portTICK_PERIOD_MS)

/** BLE GAPデバイス名 */
#define BLE_GAP_DEVICE_NAME         "BLE_MSG_CLI:0000"
#define BLE_GAP_SERVER_NAME         "BLE_MSG_SVR:0000"

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
#define BLE_MSG_RX_WAIT_TICK        (10 / portTICK_PERIOD_MS)
/** メッセージデバイスID */
#define BLE_MSG_DEVICE_ID           (0x0000000000000001)
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
 * 制御ステータス
 */
typedef struct {
    esp_bd_addr_t s_rmt_bda;        // リモートBLEアドレス
    uint64_t u64_rmt_device_id;     // リモートデバイスID
    uint32_t u32_passkey;           // パスキー
} ts_control_status_t;


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

/** 制御ステータス */
static ts_control_status_t s_cntr_sts = {
    .s_rmt_bda = {0x00},
    .u64_rmt_device_id = 0,
    .u32_passkey = 0,
};

/** チケットリスト */
static ts_ticket_list_t s_ticket_list = {
    .u32_size = 0,
};

// GATTインターフェース
static esp_gatt_if_t t_app_gatt_if = ESP_GATT_IF_NONE;

// GATTアプリケーション設定
static ts_com_ble_gattc_if_config_t s_gattc_app_config[BLE_GATT_IF_CNT];

/**
 * スキャンパラメータ
 */
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
//    .own_addr_type          = BLE_ADDR_TYPE_RANDOM,
//    .own_addr_type          = BLE_ADDR_TYPE_RPA_PUBLIC,
//    .own_addr_type          = BLE_ADDR_TYPE_RPA_RANDOM,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

//==============================================================================
// アトリビュートＤＢで管理する値の構造体定義
//==============================================================================
// UART送信イベントキュー
static QueueHandle_t s_uart_tx_queue = NULL;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** アプリケーション初期処理 */
static void v_app_init();
/** GAPプロファイルのイベントコールバック */
static void v_gap_event_cb(esp_gap_ble_cb_event_t e_event, esp_ble_gap_cb_param_t* pu_param);
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
    // 初期処理
    //**************************************************************************
    v_app_init();

    //**************************************************************************
    // 主処理
    //**************************************************************************
    // ステータス値
    esp_err_t sts_val = ESP_OK;
    // スキャン開始処理
    uint32_t u32_duration = 30;
    // ステータスチェックフラグ
    bool b_sts_chk = false;
    // スキャン結果リスト
    ts_com_ble_gap_device_list_t* ps_list = NULL;
    // スキャン結果
    ts_com_ble_gap_device_info_t* ps_device = NULL;
    // 接続ステータス
    te_com_ble_msg_connection_sts_t e_con_sts;
    // トランザクションステータス
    te_com_ble_msg_transaction_sts_t e_trn_sts = COM_BLE_MSG_TRN_NONE;

    while(true) {
        //======================================================================
        // ウェイト
        //======================================================================
        vTaskDelay(MAIN_WAIT_TICK);

        //======================================================================
        // メッセージサーバーへの接続
        //======================================================================
        e_con_sts = e_com_msg_connection_sts();
        if (e_con_sts == COM_BLE_MSG_CON_DISCONNECTED) {
            //------------------------------------------------------------------
            // スキャン開始
            //------------------------------------------------------------------
            if (!b_com_ble_gap_is_scanning()) {
                // スキャン開始処理
                sts_val = sts_com_ble_gap_start_scan(u32_duration);
                if (sts_val != ESP_OK) {
                    continue;
                }
            }

            //------------------------------------------------------------------
            // スキャン結果判定
            //------------------------------------------------------------------
            uint16_t u16_cnt = u16_com_ble_gap_scan_device_count();
            if (u16_cnt == 0) {
                continue;
            }

            //------------------------------------------------------------------
            // 対象デバイス名を検索
            //------------------------------------------------------------------
            // BLEデバイスのスキャン結果を削除
            v_com_ble_gap_delete_device_list(ps_list);
            // BLEデバイスのスキャン結果を取得
            ps_list = ps_com_ble_gap_create_device_list();
            if (ps_list == NULL) {
                // リトライ
                continue;
            }
            // 検索結果の探索
            uint16_t u16_idx;
            for (u16_idx = 0; u16_idx < ps_list->u16_count; u16_idx++) {
                ps_device = &ps_list->ps_device[u16_idx];
                tc_com_ble_bda_string_t tc_bda;
                v_com_ble_address_to_str(tc_bda, ps_device->t_bda);
                if (strcmp(ps_device->pc_name, BLE_GAP_SERVER_NAME) == 0) {
#ifdef GATTC_MSG_CLIENT
                    ESP_LOGE(LOG_MSG_TAG, "%s AD=%s Device name=%s", __func__, tc_bda, ps_device->pc_name);
#endif
                    break;
                }
            }
            if (u16_idx == ps_list->u16_count) {
                // リトライ
                continue;
            }
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "%s Device Count:%d", __func__, u16_cnt);
#endif

            //------------------------------------------------------------------
            // メッセージサーバーへの接続
            //------------------------------------------------------------------
            // BLEのGAPプロファイルにおけるPINコードの設定処理
            // ペアリングの際のPINコードは数字６桁の固定値、型はuint32_t
            sts_val = sts_com_ble_gap_set_static_pass_key(BLE_GAP_CLI_PASSKEY);
            if (sts_val != ESP_OK) {
                // リトライ
                continue;
            }
            // メッセージサーバーへの接続開始
            sts_val = sts_com_msg_open_server(ps_device);
            if (sts_val != ESP_OK) {
                // リトライ
                continue;
            }
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "%s L#%d Open Server", __func__, __LINE__);
#endif

            //------------------------------------------------------------------
            // 接続までのイベント待ち
            //------------------------------------------------------------------
            do {
                // 遅延処理
                i64_dtm_delay_msec(1000);
                e_con_sts = e_com_msg_connection_sts();
                if ((e_con_sts & COM_BLE_MSG_CON_WAIT_NUM_CHK) == COM_BLE_MSG_CON_WAIT_NUM_CHK) {
                    // GAP番号確認リクエストに応答する
                    sts_val = sts_com_ble_gap_confirm_reply(ps_device->t_bda, true);
                }
           } while(e_con_sts != COM_BLE_MSG_CON_DISCONNECTED &&
                    e_con_sts != COM_BLE_MSG_CON_CONNECTED &&
                    e_con_sts != COM_BLE_MSG_CON_ERROR);
            // コネクション判定
            if (e_con_sts != COM_BLE_MSG_CON_CONNECTED) {
                // 未接続なのでリトライ
                continue;
            }

            //------------------------------------------------------------------
            // 接続先デバイス情報編集
            //------------------------------------------------------------------
            // 接続先BLEアドレス
            v_com_ble_addr_cpy(s_cntr_sts.s_rmt_bda, ps_device->t_bda);
            // 接続先デバイスID
            sts_val = sts_com_msg_edit_remote_dev_id(&s_cntr_sts.u64_rmt_device_id);
            if (sts_val != ESP_OK) {
#ifdef GATTC_MSG_CLIENT
                ESP_LOGE(LOG_MSG_TAG, "%s Message server DeviceID not found! sts=%d", __func__, sts_val);
#endif
                // リトライ
                continue;
            }
            //------------------------------------------------------------------
            // メッセージ機能のペアリング開始
            //------------------------------------------------------------------
            sts_val = sts_com_msg_tx_pairing_request();
            if (sts_val != ESP_OK) {
                // リトライ
#ifdef GATTC_MSG_CLIENT
                ESP_LOGE(LOG_MSG_TAG, "%s Pairing request error", __func__);
#endif
                continue;
            }
            //------------------------------------------------------------------
            // ステータスチェックフラグを更新
            //------------------------------------------------------------------
            b_sts_chk = true;
        }
        // メッセージ機能のペアリング判定
        if (b_com_msg_is_paired(s_cntr_sts.u64_rmt_device_id) && b_sts_chk) {
            // 接続状況を表示
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "%s L#%d Message server Paired!", __func__, __LINE__);
#endif
            // トランザクションステータス
            e_trn_sts = sts_com_msg_transaction_sts();
            if (e_trn_sts != COM_BLE_MSG_TRN_NONE) {
                // リトライ
                continue;
            }
            // ステータスチェック開始
            sts_val = sts_com_msg_tx_sts_chk_request();
            if (sts_val != ESP_OK) {
                // リトライ
                continue;
            }
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "%s Status request", __func__);
#endif
            // ステータスチェックフラグを更新
            b_sts_chk = false;
       }
    }
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
    s_uart_tx_queue = xQueueCreate(32, sizeof(uint8_t));
    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 4096, 32, &s_uart_tx_queue, 0);

    //==========================================================================
    // BLEの初期化処理
    //==========================================================================
    // BLE初期化処理
    sts_ret = sts_com_ble_init();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "MAIN %s init controller failed: %s", __func__, esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // 電波の出力設定
    sts_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "tx power setting error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // ボンディングデバイス表示
    sts_ret = sts_com_ble_display_bonded_devices();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "bonding devaice display error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // ボンディングデバイス削除
    sts_ret = sts_com_ble_disbonding_all();
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "bonding devaice remove error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
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
    s_ble_gap_cfg.v_callback      = v_gap_event_cb;
    // SMP設定
    sts_ret = sts_com_ble_gap_smp_init(s_ble_gap_cfg);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：スキャンパラメータ設定
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_smp_init error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // スキャンパラメータの設定処理
    sts_ret = sts_com_ble_gap_set_scan_params(&ble_scan_params);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：スキャンパラメータ設定
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "%s L#%d sts_com_ble_gap_set_scan_params error!!", __func__, __LINE__);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }

    //==========================================================================
    // SPPクライアント設定
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
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "%s gattc register error, error code = %x", __func__, sts_ret);
#endif
        ESP_ERROR_CHECK(sts_ret);
    }

    //==========================================================================
    // BLEメッセージ初期処理
    //==========================================================================
    // メッセージクライアントの初期処理
    sts_ret = sts_com_msg_init_cli(BLE_GATT_APP_ID, BLE_MSG_DEVICE_ID, BLE_MSG_MAX_SIZE, v_msg_evt_cb, sts_msg_ticket_cb);
    if (sts_ret != ESP_OK) {
        // エラーメッセージログ：エラーコードの文字列表現を表示
#ifdef GATTC_MSG_CLIENT
        ESP_LOGE(LOG_MSG_TAG, "message client initialize error:%s", esp_err_to_name(sts_ret));
#endif
        ESP_ERROR_CHECK(sts_ret);
    }
    // ペアリング機能の有効化
    v_com_msg_config_pairing(true);
    // ステータスチェック機能の有効化
    v_com_msg_config_sts_chk(true);
    // 受信メッセージのエンキュー有効化
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_DATA);         // データ
    v_com_msg_rx_enabled(COM_BLE_MSG_TYP_CIPHERTEXT);   // 暗号データ

    //==========================================================================
    // 設定後の初期処理
    //==========================================================================
    // GATTインターフェースを取得
    t_app_gatt_if = t_com_ble_gattc_if(BLE_GATT_APP_ID);
    while (t_app_gatt_if == ESP_GATT_IF_NONE) {
        // ウェイトしてリトライ
        vTaskDelay(INIT_WAIT_TICK);
        t_app_gatt_if = t_com_ble_gattc_if(BLE_GATT_APP_ID);
    }
    // BLEのローカルキャッシュのクリア
    sts_ret = sts_com_ble_gattc_cache_clean(t_app_gatt_if);
    if (sts_ret != ESP_OK && sts_ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGE(LOG_MSG_TAG, "%s gattc cache clean error code = %x", __func__, sts_ret);
        ESP_ERROR_CHECK(sts_ret);
    }

    //==========================================================================
    // メッセージ処理タスクの開始
    //==========================================================================
    portBASE_TYPE b_type;
    b_type = xTaskCreate(v_msg_task_rx, "uart_rx_task", 16384, (void*)UART_NUM_0, BLE_MSG_UART_PRIORITIES, NULL);
    if (b_type != pdPASS) {
        ESP_LOGE(LOG_MSG_TAG, "Task Create Err:spp_rx_task");
    }
    // メッセージ送信タスクの開始
    b_type = xTaskCreate(v_msg_task_tx, "uart_tx_task", 16384, (void*)UART_NUM_0, BLE_MSG_UART_PRIORITIES, NULL);
    if (b_type != pdPASS) {
        ESP_LOGE(LOG_MSG_TAG, "Task Create Err:spp_tx_task");
    }
}

/*******************************************************************************
 *
 * NAME: v_gap_adv_event_cb
 *
 * DESCRIPTION:GATTプロファイルのイベントハンドラ関数（SPP用）
 *
 * PARAMETERS:              Name        RW  Usage
 * esp_gap_ble_cb_event_t   e_event     R   GAPイベントタイプ
 * esp_ble_gatts_cb_param_t u_param     R   GATTコールバックパラメータ
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
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
        return;
    }

    //==========================================================================
    // イベント処理
    //==========================================================================
    switch (e_event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        // 認証完了通知
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // ピアデバイスに表示されている値の返信要求 ※サーバー側とスキャン側の両方にある
        sts_com_ble_gap_passkey_reply(pu_param->ble_security.ble_req.bd_addr, true, BLE_GAP_SVR_PASSKEY);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        // 番号確認要求 ※サーバー側とクライアント側の両方にある
        // すべき事：PINコード認証の場合には、相手デバイスに表示されているPINコードの確認結果を返信
        // IOにDisplayYesNO機能があり、ピアデバイスIOにもDisplayYesNo機能がある場合、アプリはこのevtを受け取ります。
        // パスキーをユーザーに表示し、ピアデバイスにて表示される番号と一致するか確認した結果を返信する
        // ※本来はピアデバイスに表示されている値と比較して、一致する事を確認した上で返信
        v_com_ble_addr_cpy(s_cntr_sts.s_rmt_bda, pu_param->ble_security.key_notif.bd_addr);
        s_cntr_sts.u32_passkey = pu_param->ble_security.key_notif.passkey;
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
 * te_com_ble_msg_cb_evt_t  e_msg_evt   R   イベント種別
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
    esp_err_t  sts_val = ESP_OK;
    do {
        // イベント毎の処理
        switch (e_msg_evt) {
        case COM_BLE_MSG_EVT_RX_RESPONSE:
            // 応答を受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_RESPONSE");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_RESET:
            // リセットメッセージ受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_RESET");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_PING:
            // PINGメッセージ受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_PING");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_DATA:
            // データメッセージ受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_DATA");
#endif
            break;
        case COM_BLE_MSG_EVT_RX_CIPHERTEXT:
            // 暗号メッセージ受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_RCV_CIPHERTEXT");
#endif
            break;
#ifdef GATTC_MSG_CLIENT
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
            // 接続直後のメッセージ受信
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_OPEN_SUCCESS");
#endif
            break;
        case COM_BLE_MSG_EVT_PAIRING_START:
            // ペアリング開始
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_START");
#endif
            // ペアリング認証（ダイジェスト比較結果の通知）
            // とりあえず無条件にペアリングOKとする
            sts_val = sts_com_msg_tx_pairing_certification(true, 0xFFFFFFFF);
            if (sts_val != ESP_OK) {
                break;
            }
            break;
        case COM_BLE_MSG_EVT_PAIRING_SUCCESS:
            // ペアリング成功
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_SUCCESS");
#endif
            break;
        case COM_BLE_MSG_EVT_PAIRING_ERR:
            // ペアリングエラー
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_PAIRING_ERR");
#endif
            // メッセージ機能のペアリングの開始
            sts_val = sts_com_msg_tx_pairing_request();
            if (sts_val != ESP_OK) {
                // リトライ
                continue;
            }
            break;
        case COM_BLE_MSG_EVT_STATUS_CHK:
            // ステータスチェック開始
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_CHK");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_OK:
            // ステータス正常
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_OK");
#endif
            break;
        case COM_BLE_MSG_EVT_STATUS_ERR:
            // ステータス異常
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_STATUS_ERR");
#endif
            // チケット削除
//            sts_com_msg_delete_ticket(ps_msg->u64_device_id);
            break;
        case COM_BLE_MSG_EVT_HANDLING_ERR:
            // メッセージハンドリングエラー
#ifdef GATTC_MSG_CLIENT
            ESP_LOGE(LOG_MSG_TAG, "MsgEvt=COM_BLE_MSG_EVT_HANDLING_ERR");
#endif
            break;
        default:
            // メッセージハンドリングエラー
#ifdef GATTC_MSG_CLIENT
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
    if (xSemaphoreTakeRecursive(s_mutex, EVT_TAKE_WAIT_TICK) != pdTRUE) {
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
    ts_com_msg_t* ps_msg;
    // 送信データ
    ts_u8_array_t* ps_data = NULL;
    while (true) {
        //======================================================================
        // 受信イベント処理
        //======================================================================
        // メッセージ受信
        ps_msg = ps_com_msg_rx_msg(portMAX_DELAY);
        if (ps_msg == NULL) {
#ifdef GATTC_MSG_CLIENT
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "RX Error!!!");
#endif
            continue;
        }

        // UARTデータ書き込み
        ps_data = ps_msg->ps_data;
        uart_write_bytes(UART_NUM_0, ps_data->pu8_values, ps_data->t_size);
        // 動的に確保されている本文データを解放
        sts_com_msg_delete_msg(ps_msg);
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
    // UARTイベント
    uart_event_t s_event;
    // UARTデータサイズ
    size_t t_data_len;
    // 接続中デバイスID
    uint64_t u64_device_id = 0;
    // 送信データ
    ts_u8_array_t* ps_data = NULL;
    for (;;) {
        //======================================================================
        // シリアルポートからの値入力検知
        //======================================================================
        // Waiting for UART event.
        if (xQueueReceive(s_uart_tx_queue, (void*)&s_event, portMAX_DELAY) != pdTRUE) {
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
        // 読み込みデータバッファを生成
        ps_data = ps_mdl_empty_u8_array(t_data_len);
        if (ps_data == NULL) {
            // リトライ
            continue;
        }
        // UARTのReadバッファを読み込み
        uart_read_bytes(UART_NUM_0, ps_data->pu8_values, ps_data->t_size, portMAX_DELAY);
        // デバッグ出力
#ifdef GATTC_MSG_CLIENT
        char c_dbg_str[ps_data->t_size + 1];
        memcpy(c_dbg_str, ps_data->pu8_values, ps_data->t_size);
        c_dbg_str[ps_data->t_size] = '\0';
        ESP_LOGI(LOG_MSG_TAG, "Read=%s", c_dbg_str);
#endif

        //======================================================================
        // クリティカルセクション開始
        //======================================================================
        if (xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        //======================================================================
        // イベント処理
        //======================================================================
        do {
            // 接続確認
            if (e_com_msg_connection_sts() != COM_BLE_MSG_CON_CONNECTED) {
                // 読み捨ててリトライ
#ifdef GATTC_MSG_CLIENT
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "disconnected!!! sts=%d", e_com_msg_connection_sts());
#endif
                break;
            }
            // 接続先デバイスIDの編集
            if (sts_com_msg_edit_remote_dev_id(&u64_device_id) != ESP_OK) {
                // 読み捨ててリトライ
#ifdef GATTC_MSG_CLIENT
        // デバッグ出力
        ESP_LOGE(LOG_MSG_TAG, "Unable to obtain device ID!!!");
#endif
                break;
            }
            // 平文メッセージの送信処理
//            if (sts_com_msg_tx_plain_msg(u64_device_id, ps_data) != ESP_OK) {
//                // 読み捨ててリトライ
//#ifdef GATTC_MSG_CLIENT
//        // デバッグ出力
//        ESP_LOGE(LOG_MSG_TAG, "TX Error!!!");
//#endif
//                break;
//            }
            // 暗号文メッセージの送信処理
            if (sts_com_msg_tx_cipher_msg(u64_device_id, ps_data) != ESP_OK) {
                // 読み捨ててリトライ
#ifdef GATTC_MSG_CLIENT
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

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
