/*******************************************************************************
 *
 * MODULE :Common Touchpad framework functions source file
 *
 * CREATED:2020/03/22 22:04:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:タッチパッドの簡易フレームワーク
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
#include "ntfw_io_touchpad_fmwk.h"

#include <string.h>
#include <freertos/semphr.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
#/** Demon process depth of stack */
#define COM_TOUCHPAD_DEAMON_STACK_DEPTH (1024)
/** Demon process priority */
#define COM_TOUCHPAD_DEAMON_PRIORITIES  (configMAX_PRIORITIES - 4)


/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
typedef struct {
    TaskHandle_t s_deamon_task;             // デーモンタスクハンドル
    QueueHandle_t s_sts_queue;              // タッチステータスキューハンドラ
    uint32_t u32_chk_target;                // チェック対象ステータス
    uint16_t u16_threshold[TOUCH_PAD_MAX];  // 閾値
} ts_touchpad_status_t;

/**
 * ミューテックス取得処理
 */
typedef SemaphoreHandle_t (*tf_get_mutex_t)();

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;
/** 制御ステータス */
static ts_touchpad_status_t s_ctrl_sts = {
    .s_deamon_task  = NULL,     // デーモンタスクハンドル
    .s_sts_queue    = NULL,     // タッチステータスキューハンドラ
    .u32_chk_target = 0,        // チェック対象ステータス
    .u16_threshold  = {IO_TOUCHPAD_DEFAULT_THRESHOLD}  // 閾値
};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
//==============================================================================
// Local Common Functions
//==============================================================================
/** Get Mutex function(initial) */
static SemaphoreHandle_t s_get_mutex_init();
/** Get Mutex function */
static SemaphoreHandle_t s_get_mutex();
/** Get Mutex function */
static tf_get_mutex_t pf_get_mutex = s_get_mutex_init;

//==============================================================================
// Touch Pad Functions
//==============================================================================
/** TouchPad Deamon Function */
static void v_touchpad_daemon_task(void* pv_parameters);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: sts_io_touchpad_init
 *
 * DESCRIPTION:標準初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_init() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // クリティカルセクション
    //==========================================================================
    // ステータスキューの生成
    if (s_ctrl_sts.s_sts_queue == NULL) {
        s_ctrl_sts.s_sts_queue = xQueueCreate(IO_TOUCHPAD_STS_QUEUE_SIZE, sizeof(uint32_t));
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // タッチパッド初期処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // タッチパッド初期処理
        sts_val = touch_pad_init();
        if (sts_val != ESP_OK) {
            break;
        }
        // タッチセンサーの基準電圧設定
        sts_val = touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
        if (sts_val != ESP_OK) {
            break;
        }
        // タッチセンサーの測定をハードウェアタイマー制御
        sts_val = touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
        if (sts_val != ESP_OK) {
            break;
        }
        // ノイズフィルタの適用開始：較正期間は10ミリ秒
        sts_val = touch_pad_filter_start(IO_TOUCHPAD_FILTER_PERIOD);
    } while(false);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_touchpad_pin_enable
 *
 * DESCRIPTION:タッチピンの有効化
 *
 * PARAMETERS:      Name        RW  Usage
 *   touch_pad_t    e_touch_num R   タッチパッド番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_pin_enable(touch_pad_t e_touch_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // タッチパッド番号チェック
    if (e_touch_num >= TOUCH_PAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // タッチピンの有効化
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // タッチピンの初期処理
        sts_val = touch_pad_io_init(e_touch_num);
        if (sts_val != ESP_OK) {
            break;
        }
        // タッチセンサーの充放電速度設定
        sts_val = touch_pad_set_cnt_mode(e_touch_num, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW);
        if (sts_val != ESP_OK) {
            break;
        }
        // 初回計測完了まで待つ
        sts_val = touch_pad_config(e_touch_num, 0);
        if (sts_val != ESP_OK) {
            break;
        }
        // 閾値の初期設定
        sts_val = touch_pad_config(e_touch_num, IO_TOUCHPAD_DEFAULT_THRESHOLD);
        if (sts_val != ESP_OK) {
            break;
        }
        // 初期化済みピン設定
        s_ctrl_sts.u32_chk_target |= (0x01 << e_touch_num);
        s_ctrl_sts.u16_threshold[e_touch_num] = IO_TOUCHPAD_DEFAULT_THRESHOLD;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // ステータス返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_io_touchpad_pin_disable
 *
 * DESCRIPTION:タッチピンの無効化
 *
 * PARAMETERS:      Name        RW  Usage
 *   touch_pad_t    e_touch_num R   タッチパッド番号
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_pin_disable(touch_pad_t e_touch_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // タッチパッド番号チェック
    if (e_touch_num >= TOUCH_PAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // タッチピンの無効化
    //==========================================================================
    s_ctrl_sts.u32_chk_target ^= (0x01 << e_touch_num);
    s_ctrl_sts.u16_threshold[e_touch_num] = IO_TOUCHPAD_DEFAULT_THRESHOLD;

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // ステータス返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: u16_io_touchpad_pin_average
 *
 * DESCRIPTION:タッチピンの平均値取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 * touch_pad_t      e_touch_num     R   タッチパッド番号
 *
 * RETURNS:
 *   uint16_t:平均値
 *
 ******************************************************************************/
uint16_t u16_io_touchpad_pin_average(touch_pad_t e_touch_num) {
    // タッチパッド番号チェック
    if (e_touch_num >= TOUCH_PAD_MAX) {
        return 0;
    }
    // データのサンプリング
    uint32_t u32_avg = 0;
    uint16_t u16_val;
    int i_idx;
    for (i_idx = 0; i_idx < IO_TOUCHPAD_NUMBER_OF_SAMPLES; i_idx++) {
        if (touch_pad_read(e_touch_num, &u16_val) != ESP_OK) {
            return 0;
        }
        u32_avg += u16_val;
    }
    // アベレージ算出
    return (uint16_t)(u32_avg /= IO_TOUCHPAD_NUMBER_OF_SAMPLES);
}

/*******************************************************************************
 *
 * NAME: sts_io_touchpad_pin_threshold
 *
 * DESCRIPTION:タッチピンの閾値設定処理
 *
 * PARAMETERS:      Name            RW  Usage
 * touch_pad_t      e_pad           R   タッチパッド番号
 * uint16_t         u16_threshold   R   閾値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_pin_threshold(touch_pad_t e_touch_num, uint16_t u16_threshold) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    // ステータス
    esp_err_t sts_val = ESP_OK;
    do {
        // タッチパッド番号チェック
        if (e_touch_num >= TOUCH_PAD_MAX) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        if ((s_ctrl_sts.u32_chk_target & (0x01 << e_touch_num)) == 0x0) {
            sts_val = ESP_ERR_INVALID_ARG;
            break;
        }
        // タッチパッドの閾値設定
        sts_val = touch_pad_set_thresh(e_touch_num, u16_threshold);
        s_ctrl_sts.u16_threshold[e_touch_num] = u16_threshold;
    } while(false);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // ステータス返信
    return sts_val;
}


/*******************************************************************************
 *
 * NAME: sts_io_touchpad_check_enable
 *
 * DESCRIPTION:タッチパッドのチェックデーモンの有効化
 *
 * PARAMETERS:      Name        RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_check_enable() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // デーモンタスクの起動処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        // デーモンタスクの起動状態判定
        if (s_ctrl_sts.s_deamon_task != NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ステータスキューのチェック
        if (s_ctrl_sts.s_sts_queue == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // メッセージ受信タスクの開始
        portBASE_TYPE b_type = xTaskCreatePinnedToCore(v_touchpad_daemon_task,
                                                       "touchpad_deamon_task",
                                                       COM_TOUCHPAD_DEAMON_STACK_DEPTH,
                                                       (void*)NULL,
                                                       COM_TOUCHPAD_DEAMON_PRIORITIES,
                                                       &s_ctrl_sts.s_deamon_task,
                                                       tskNO_AFFINITY);
        if (b_type != pdPASS) {
            // タスク開始に失敗
            sts_val = ESP_FAIL;
            break;
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
 * NAME: sts_io_touchpad_check_disable
 *
 * DESCRIPTION:タッチパッドのチェックデーモンの無効化
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_io_touchpad_check_disable() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // チェックデーモンの無効化
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;

    do {
        // デーモンタスクの有無を判定
        if (s_ctrl_sts.s_deamon_task == NULL) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // デーモンタスクを停止
        vTaskDelete(s_ctrl_sts.s_deamon_task);
        s_ctrl_sts.s_deamon_task = NULL;

        // タッチステータスのキュークリア
        uint32_t u32_sts;
        while (xQueueReceive(s_ctrl_sts.s_sts_queue, &u32_sts, 0) == pdTRUE);
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
 * NAME: u32_io_touchpad_pinmap
 *
 * DESCRIPTION:タッチパッド割り込みのステータス取得処理
 *
 * PARAMETERS:      Name            RW  Usage
 *   TickType_t     t_tick          R   最大待ち時間
 *
 * RETURNS:
 *   uint32_t:タッチステータス
 *
 ******************************************************************************/
uint32_t u32_io_touchpad_pinmap(TickType_t t_tick) {
    //==========================================================================
    // ステータスキューの取得
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    // キュー
    QueueHandle_t s_sts_queue = s_ctrl_sts.s_sts_queue;
    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);

    //==========================================================================
    // タッチステータスの取得
    //==========================================================================
    // キューの有無を判定
    if (s_sts_queue == NULL) {
        return 0;
    }
    // タッチステータスをデキュー
    uint32_t u32_sts_map = 0;
    if (xQueueReceive(s_sts_queue, &u32_sts_map, t_tick) != pdTRUE) {
        return 0;
    }

    // ステータスマップを返却
    return u32_sts_map;
}


/*******************************************************************************
 *
 * NAME: v_io_touchpad_clear_pinmap
 *
 * DESCRIPTION:タッチパッド割り込みのステータスクリア処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 ******************************************************************************/
void v_io_touchpad_clear_pinmap() {
    //==========================================================================
    // ステータスキューの取得
    //==========================================================================
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
        return;
    }
    //==========================================================================
    // タッチステータスクリア
    //==========================================================================
    if (s_ctrl_sts.s_sts_queue != NULL) {
        // タッチステータスをデキュー
        uint32_t u32_sts_map;
        while (xQueueReceive(s_ctrl_sts.s_sts_queue, &u32_sts_map, 0) == pdTRUE);
    }
    // クリティカルセクション終了
    xSemaphoreGiveRecursive(s_mutex);
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: s_get_mutex_init
 *
 * DESCRIPTION:Get Mutex function(initial)
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   SemaphoreHandle_t: Generated Mutex
 *
 * NOTES:
 * None.
 ******************************************************************************/
static SemaphoreHandle_t s_get_mutex_init() {
    // ミューテックスを生成
    s_mutex = xSemaphoreCreateRecursiveMutex();
    // 関数を切り替え
    pf_get_mutex = s_get_mutex;
    // ミューテックスを返却
    return s_mutex;
}

/*******************************************************************************
 *
 * NAME: s_get_mutex
 *
 * DESCRIPTION:Get Mutex function
 *
 * PARAMETERS:  Name            RW  Usage
 *
 * RETURNS:
 *   SemaphoreHandle_t: Generated Mutex
 *
 * NOTES:
 * None.
 ******************************************************************************/
static SemaphoreHandle_t s_get_mutex() {
    // ミューテックスを返却
    return s_mutex;
}

/*******************************************************************************
 *
 * NAME: v_touchpad_daemon_task
 *
 * DESCRIPTION:Touchpad Daemon task
 *
 * PARAMETERS:  Name            RW  Usage
 * void*        pv_parameters   R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_touchpad_daemon_task(void* pv_parameters) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    // ステータスキュー
    QueueHandle_t s_sts_queue;
    // チェックステータス
    uint32_t u32_chk_target;
    // チェックステータス
    uint16_t u16_threshold[TOUCH_PAD_MAX];
    // 今回ピンステータスマップ
    uint32_t u32_sts_map;
    // 前回ピンステータスマップ
    uint32_t u32_last_sts_map = 0;
    // タッチパッド値
    uint16_t u16TouchVal;
    // 現在ティック
    TickType_t t_tick_now = xTaskGetTickCount();
    // 前回ティック
    TickType_t t_tick_bef = t_tick_now;
    // 経過ティック
    TickType_t t_tick_elapsed = 0;

    //==========================================================================
    // 主処理
    //==========================================================================
    while (true) {
        //----------------------------------------------------------------------
        // コンテキスト取得
        //----------------------------------------------------------------------
        // クリティカルセクション開始
        if (xSemaphoreTakeRecursive(pf_get_mutex(), portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // ステータスキュー
        s_sts_queue    = s_ctrl_sts.s_sts_queue;
        // チェック対象
        u32_chk_target = s_ctrl_sts.u32_chk_target;
        // 閾値
        memcpy(u16_threshold, s_ctrl_sts.u16_threshold, sizeof(s_ctrl_sts.u16_threshold));

        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex);

        //----------------------------------------------------------------------
        // ステータス取得
        //----------------------------------------------------------------------
        // ステータス初期化
        u32_sts_map = 0;
        // タッチパッド毎のループ
        int i_pad_idx;
        for (i_pad_idx = 0; i_pad_idx < TOUCH_PAD_MAX; i_pad_idx++) {
            do {
                // チェック対象なのか判定
                if ((u32_chk_target & 0x01) == 0x00) {
                    break;
                }
                // タッチパッドの計測値を取得
                if (touch_pad_read(i_pad_idx, &u16TouchVal) != ESP_OK) {
                    break;
                }
                // 閾値チェック
//                ESP_LOGE("TOUCH", "Touchpad map=%d", u16TouchVal);
                if (u16TouchVal >= u16_threshold[i_pad_idx]) {
                    break;
                }
                // ステータス更新
                u32_sts_map |= (0x0001 << i_pad_idx);
            } while(false);
            // 次のチェック対象に更新
            u32_chk_target = u32_chk_target >> 1;
        }

        //----------------------------------------------------------------------
        // ステータス通知処理
        //----------------------------------------------------------------------
        // 直前のステータスと比較
        if (u32_sts_map != u32_last_sts_map) {
            // ステータスを通知
            xQueueSendToBack(s_sts_queue, &u32_sts_map, portMAX_DELAY);
            // 直前のステータスを更新
            u32_last_sts_map = u32_sts_map;
        }

        //----------------------------------------------------------------------
        // ウェイト処理
        //----------------------------------------------------------------------
        t_tick_bef = t_tick_now;
        t_tick_now = xTaskGetTickCount();
        t_tick_elapsed = t_tick_now - t_tick_bef;
        if (IO_TOUCHPAD_DEAMON_WAIT > t_tick_elapsed) {
            vTaskDelay(IO_TOUCHPAD_DEAMON_WAIT - t_tick_elapsed);
        }
    }
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
