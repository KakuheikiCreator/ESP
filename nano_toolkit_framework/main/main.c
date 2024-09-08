/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :Common Library functions source file
 *
 * CREATED:2019/10/01 06:44:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:共通ライブラリのテストコード関係メイン処理
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

// SD connection setting
//#define USE_SDMMC_HSPI_MODE

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sdkconfig.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <nvs_flash.h>
#include <hal/gpio_hal.h>
#include <soc/io_mux_reg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <driver/rtc_io.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>

#include "settings.h"

#include "ntfw_com_value_util.h"
#include "ntfw_com_mem_alloc.h"
#include "ntfw_com_data_model.h"
#include "ntfw_com_date_time.h"
#include "ntfw_com_debug_util.h"
#include "ntfw_cryptography.h"
#include "ntfw_io_file_util.h"
#include "ntfw_io_gpio_util.h"
#include "ntfw_io_i2c_master.h"
#include "ntfw_io_touchpad_fmwk.h"
#include "ntfw_ble_fmwk.h"
#include "ntfw_ble_msg.h"
#include "ntfw_drv_adxl345.h"
#include "ntfw_drv_lis3dh.h"
#include "ntfw_drv_mpu-6050.h"
#include "ntfw_drv_rx8900.h"
#include "ntfw_drv_st7032i.h"


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// タイムアウト
#define TWDT_TIMEOUT_MSEC       (3000)
#define TASK_RESET_PERIOD_S     (2)

/** 待ち時間 */
#ifndef COM_WAIT_TIME
    #define EVT_ENQUEUE_WAIT_TICK  (10 / portTICK_PERIOD_MS)
#endif

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

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** WatchDog Test Code */
static void v_task_chk_watchdog(void* args);
static void v_reset_task(void *arg);

/** Test Code Main */
static void v_task_test_main(void* args);

/** Memory allocate */
static void v_task_chk_mem_alloc(void* args);
static void v_task_chk_mem_alloc_00();
static void v_task_chk_mem_alloc_01();
static void v_task_chk_mem_alloc_02();
static void v_task_chk_mem_alloc_03();
static void v_task_chk_mem_alloc_04();
static void v_task_chk_mem_alloc_05();
static void v_task_chk_mem_alloc_06();
static uint32_t u32_task_chk_memory(uint32_t* pu32_size, void** ppv_mem, uint32_t u32_idx);
static void v_task_chk_mem_alloc_disp_area();
static void v_task_chk_mem_alloc_disp_info();

/** Value Util Test Code */
static void v_task_chk_value_util(void* args);
static void v_task_chk_value_util_00();
static void v_task_chk_value_util_01();
static void v_task_chk_value_util_02();
static void v_task_chk_value_util_03();
static void v_task_chk_value_util_04();

/** Cryptography Test Code */
static void v_task_chk_cryptography(void* args);
static void v_task_chk_cryptography_00();
static void v_task_chk_cryptography_01();
static void v_task_chk_cryptography_02();
static void v_task_chk_cryptography_03();
static void v_task_chk_cryptography_04();
static void v_task_chk_cryptography_05();
static void v_task_chk_cryptography_06();

/** ADC Test Code */
static void v_task_chk_adc(void* args);
static void v_task_chk_adc_efuse();
static void v_task_chk_adc_00(ts_adc_oneshot_context* ps_adc_ctx);

/** Touch Test Code */
static void v_task_chk_touch(void* args);

/** File Util Test Code */
static void v_task_chk_file_util(void* args);
static void v_task_chk_file_util_00();
static void v_task_chk_file_util_01();
static void v_task_chk_file_util_02(sdmmc_card_t* ps_card);
static void v_task_chk_file_util_03();
/** Common Date Time Test Code */
static void v_task_chk_com_date_time(void* args);
static void v_task_chk_com_date_time_00();
static void v_task_chk_com_date_time_01();
static void v_task_chk_com_date_time_02();
/** I2C Util Test Code */
static void v_task_chk_com_i2c_mst(void* args);
static void v_task_chk_com_i2c_mst_00();
static void v_task_chk_com_i2c_mst_01();
static void v_task_chk_com_i2c_mst_02();
/** RX8900 Test Code */
static void v_task_chk_rx8900(void* args);
/** ST7032I TestCode */
static void v_task_chk_st7032i(void* args);
static void v_task_chk_st7032i_00();
/** ADXL345 Test Code */
static void v_task_chk_adxl345(void* args);
/** LIS3DH Test Code */
static void v_task_chk_lis3dh(void* args);
/** MPU6050 Test Code */
static void v_task_chk_mpu6050(void* args);
static void v_task_chk_mpu6050_00();
/** MPU6050 Test Code */
static void v_mpu6050_read();
static void v_mpu6050_fifo_read();

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ログ出力タグ */
static const char* TAG = "Test";

/** タスクハンドラの配列 */
static TaskHandle_t task_handles[portNUM_PROCESSORS];

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
    //**************************************************************************
    // ESP32の共通初期処理
    //**************************************************************************
    esp_err_t sts_ret;

    // Initialize NVS.
    // 不揮発性メモリ領域のデフォルトパーティションを初期化する
    sts_ret = nvs_flash_init();
    // NVS領域を初期化した際のエラー判定として次の場合を判定
    // 1.NVSの空きページが無い
    // 2.新バージョンのデータがパーティションに含まれている
    if (sts_ret == ESP_ERR_NVS_NO_FREE_PAGES || sts_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS領域をクリア、エラー時には異常終了する
        ESP_ERROR_CHECK(nvs_flash_erase());
        sts_ret = nvs_flash_init();
    }
    // エラーチェック
    ESP_ERROR_CHECK(sts_ret);

    //==========================================================================
    // ウォッチドックタイマーの処理
    //==========================================================================
    v_task_chk_watchdog((void*)NULL);

    //==========================================================================
    // テストコードタスクの登録
    //==========================================================================
    // テストコード
    xTaskCreate(v_task_test_main, "Test Task", 65536, NULL, configMAX_PRIORITIES - 10, NULL);
    // タッチボタンのテストコード：特定のコアにバインドしない
    xTaskCreate(v_task_chk_touch, "Test Touch", 8192, NULL, configMAX_PRIORITIES - 8, NULL);

}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_task_chk_watchdog
 *
 * DESCRIPTION:WatchDog Timerのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_watchdog(void* args) {
    ESP_LOGI(TAG, "Initialize TWDT");
    // ウォッチドックタイマーの初期化を解除
    esp_task_wdt_deinit();
    //Initialize or reinitialize TWDT
    // ウォッチドックタイマーの初期化処理
    esp_task_wdt_config_t s_wdt_cfg = {
        .timeout_ms = TWDT_TIMEOUT_MSEC,                        // タイムアウト時間
        .idle_core_mask = (0x01 << portNUM_PROCESSORS) - 0x01,  // 全てのコアを対象に設定
        .trigger_panic = true                                   // パニックトリガーON
    };
    CHECK_ERROR_CODE(esp_task_wdt_init(&s_wdt_cfg), ESP_OK);
//    CHECK_ERROR_CODE(esp_task_wdt_reconfigure(&s_wdt_cfg), ESP_OK);

    // 各CPUコアにウォッチドックタイマーのリセットタスクを割り当てる
    // Create user tasks and add them to watchdog
    for(int i = 0; i < portNUM_PROCESSORS; i++){
        // 引数１：タスク入力関数（内部で無限ループする関数）へのポインター
        // 引数２：タスク名（デフォルト１６バイト、configMAX_TASK_NAME_LENでサイズが定義されている）
        // 引数３：タスクスタックのバイト数
        // 引数４：作成されるタスクの引数ポインタ
        // 引数５：タスクの実行優先度、portPRIVILEGE_BITを設定すると別に特権モード（システム）のタスクが作成される
        // 引数６：作成されたタスクを参照できるハンドルを返すために使用されます。
        // 引数７：タスクを割り当てるCPUコアのIDを指定する、tskNO_AFFINITYの値が指定された場合には利用可能な任意のコアで実行される
        xTaskCreatePinnedToCore(v_reset_task, "reset task", 1024, NULL, configMAX_PRIORITIES - 5, &task_handles[i], i);
    }

//    ESP_LOGI(TAG, "Delay for 10 seconds");
    // 指定された時間（10000ミリ秒）の間、タスクをBlocked状態にしてウェイト
//    vTaskDelay(pdMS_TO_TICKS(10000));   //Delay for 10 seconds

//    ESP_LOGI(TAG, "Unsubscribing and deleting tasks");
    //Delete and unsubscribe Users Tasks from Task Watchdog, then unsubscribe idle task
//    for(int i = 0; i < portNUM_PROCESSORS; i++){
//        // 登録済みのタスクを削除
//        vTaskDelete(task_handles[i]);   //Delete user task first (prevents the resetting of an unsubscribed task)
//        // タスクをウォッチドックタイマーの監視対象から除外する
//        CHECK_ERROR_CODE(esp_task_wdt_delete(task_handles[i]), ESP_OK);     //Unsubscribe task from TWDT
//        // タスクがウォッチドックタイマーの監視対象であるのか判定する
//        CHECK_ERROR_CODE(esp_task_wdt_status(task_handles[i]), ESP_ERR_NOT_FOUND);  //Confirm task is unsubscribed
//
//        // unsubscribe idle task
//        //Unsubscribe Idle Task from TWDT
//        // 指定されたCPUコアのアイドルタスクを取得して、そのタスクをウォッチドックタイマーの監視対象から除外する
//        CHECK_ERROR_CODE(esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(i)), ESP_OK);
//        //Confirm Idle task has unsubscribed
//        // 指定されたCPUコアのアイドルタスクがウォッチドックタイマーの監視対象であるのか判定する
//        CHECK_ERROR_CODE(esp_task_wdt_status(xTaskGetIdleTaskHandleForCPU(i)), ESP_ERR_NOT_FOUND);
//    }

    //Deinit TWDT after all tasks have unsubscribed
    // ウォッチドックタイマーの初期化解除処理、既に初期化解除済みの場合や監視対象タスクが有る場合にはエラーとなる
//    CHECK_ERROR_CODE(esp_task_wdt_deinit(), ESP_OK);
    // 自タスクがウォッチドックタイマーの監視対象か判定する
//    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_ERR_INVALID_STATE);     //Confirm TWDT has been deinitialized
    ESP_LOGI(TAG, "Complete");
}

/*******************************************************************************
 *
 * NAME: v_reset_task
 *
 * DESCRIPTION:ウォッチドックタイマーのリセットタスク
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_reset_task(void *arg) {
    //Subscribe this task to TWDT, then check if it is subscribed
    // ウォッチドックタイマーの監視対象に自タスクを追加
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    // 自タスクがウォッチドックタイマーの監視対象なのかを判定
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);
    // リセットタスクの無限ループ開始
    while(true) {
        // ウォッチドックタイマーをリセットする
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);  //Comment this line to trigger a TWDT timeout
        // 自タスクをタイムアウトまでウェイトする
        vTaskDelay(pdMS_TO_TICKS(TASK_RESET_PERIOD_S * 1000));
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_watchdog
 *
 * DESCRIPTION:Test Code Main
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_test_main(void* args) {
    //==========================================================================
    // Memory allocate
    //==========================================================================
    v_task_chk_mem_alloc((void*)NULL);

    //==========================================================================
    // Value Utility
    //==========================================================================
    v_task_chk_value_util((void*)NULL);

    //==========================================================================
    // Cryptography
    //==========================================================================
    v_task_chk_cryptography((void*)NULL);

    //==========================================================================
    // ADC
    //==========================================================================
    v_task_chk_adc((void*)NULL);

    //==========================================================================
    // File Utility
    //==========================================================================
    v_task_chk_file_util((void*)NULL);

    //==========================================================================
    // Date Time Utility
    //==========================================================================
    v_task_chk_com_date_time((void*)NULL);

    //==========================================================================
    // I2C init
    //==========================================================================
    // I2C初期処理
    esp_err_t sts = sts_io_i2c_mst_init(I2C_NUM_0, I2C_FREQ_HZ_FAST, GPIO_NUM_22, GPIO_NUM_21, GPIO_PULLUP_ENABLE);
//    esp_err_t sts = sts_com_i2c_mst_init(I2C_NUM_0, I2C_FREQ_HZ_FAST, GPIO_NUM_17, GPIO_NUM_16, GPIO_PULLUP_ENABLE);
//    esp_err_t sts = sts_com_i2c_mst_init(I2C_NUM_0, I2C_FREQ_HZ_STD, GPIO_NUM_17, GPIO_NUM_16, GPIO_PULLUP_ENABLE);
    if (sts != ESP_OK) {
        ESP_LOGE(TAG, "v_task_test_main:sts_com_i2c_mst_init Error %d", sts);
    }
    i2c_set_timeout(I2C_NUM_0, 0xFFFFF);
    // プルアップ
    gpio_set_level(GPIO_NUM_22, 1);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_22], PIN_FUNC_GPIO);
    gpio_set_level(GPIO_NUM_21, 1);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_21], PIN_FUNC_GPIO);
    //    gpio_set_level(GPIO_NUM_17, 1);
    //    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_17], PIN_FUNC_GPIO);
    //    gpio_set_level(GPIO_NUM_16, 1);
    //    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_16], PIN_FUNC_GPIO);

    //==========================================================================
    // I2C Time Utility
    //==========================================================================
//    v_task_chk_com_i2c_mst((void*)NULL);

    //==========================================================================
    // RX8900
    //==========================================================================
//    v_task_chk_rx8900((void*)NULL);

    //==========================================================================
    // ST7032I
    //==========================================================================
    v_task_chk_st7032i((void*)NULL);

    //==========================================================================
    // ADXL345
    //==========================================================================
//    v_task_chk_adxl345((void*)NULL);

    //==========================================================================
    // LIS3DH
    //==========================================================================
//    v_task_chk_lis3dh((void*)NULL);

    //==========================================================================
    // MPU6050
    //==========================================================================
    v_task_chk_mpu6050((void*)NULL);

    //==========================================================================
    // End
    //==========================================================================
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// End of Test");
    ESP_LOGI(TAG, "//===========================================================");
    while (true) {
        vTaskDelay(100);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc(void* args) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    v_task_chk_mem_alloc_00();

    //==========================================================================
    // 単一領域確保
    //==========================================================================
    v_task_chk_mem_alloc_01();

    //==========================================================================
    // 複数領域確保
    //==========================================================================
    v_task_chk_mem_alloc_02();

    //==========================================================================
    // 部分的に解放しながら複数領域確保、フラグメンテーション有り
    //==========================================================================
    v_task_chk_mem_alloc_03();

    //==========================================================================
    // ランダムに確保と解放
    //==========================================================================
    v_task_chk_mem_alloc_04();

    //==========================================================================
    // calloc
    //==========================================================================
    v_task_chk_mem_alloc_05();

    //==========================================================================
    // realloc
    //==========================================================================
    v_task_chk_mem_alloc_06();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_00
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_00() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: initialize");
    ESP_LOGI(TAG, "//===========================================================");
    int i_test_no = 0;
    // 初期処理を確認
    void* pv_mem_0 = pv_mem_malloc(0);
    if (pv_mem_0 == NULL) {
        ESP_LOGI(TAG, "pv_mem_malloc: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "pv_mem_malloc: No.%d Failure", i_test_no++);
    }
    // メモリ解放
    long l_free_size_0 = l_mem_free(pv_mem_0);
    if (l_free_size_0 == -1) {
        ESP_LOGI(TAG, "b_com_free: No.%d Success size=%ld", i_test_no++, l_free_size_0);
    } else {
        ESP_LOGE(TAG, "b_com_free: No.%d Failure size=%ld", i_test_no++, l_free_size_0);
    }
    void* pv_mem_1 = pv_mem_malloc(10);
    if (pv_mem_1 != NULL) {
        ESP_LOGI(TAG, "pv_mem_malloc: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "pv_mem_malloc: No.%d Failure", i_test_no++);
    }
    long l_free_size_1 = l_mem_free(pv_mem_1);
    if (l_free_size_1 == 10) {
        ESP_LOGI(TAG, "b_com_free: No.%d Success size=%ld", i_test_no++, l_free_size_1);
    } else {
        ESP_LOGE(TAG, "b_com_free: No.%d Failure size=%ld", i_test_no++, l_free_size_1);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_01
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_01() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 01");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // 初期状態の表示
    //==========================================================================
    // 領域サイズの表示1
    v_task_chk_mem_alloc_disp_area();
    // 割り当て済みのサイズ
    uint32_t u32_alloc_size = 0;
    // 未使用領域のサイズ
    uint32_t u32_unused_size = 0;
    // 未使用領域の数
    uint32_t u32_unused_cnt = 0;

    //==========================================================================
    // メモリ確保のテスト
    //==========================================================================
    // テスト番号
    int i_test_no = 0;
    void* pv_mem;
    uint32_t u32_size;
    for (u32_size = 0; u32_size < 100; u32_size++) {
        // メモリ確保
        pv_mem = pv_mem_malloc(u32_size);
        // 割り当て済みのサイズ
        u32_alloc_size = u32_mem_alloc_size();
        if (u32_alloc_size != u32_size) {
            ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_alloc_size);
        }
        if (u32_size == 0) {
            if (pv_mem == NULL) {
                ESP_LOGI(TAG, "pv_mem_malloc : No.%d Success size=%lu", i_test_no, (unsigned long)u32_size);
            } else {
                ESP_LOGE(TAG, "pv_mem_malloc : No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size);
            }
            // 未使用領域のサイズ
            u32_unused_size = u32_mem_unused_size();
            if (u32_unused_size != (MEM_STORAGE_SIZE - 24)) {
                ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_size);
            }
        } else {
            uint32_t u32_mem_size = *((uint32_t*)(pv_mem - sizeof(uint32_t)));
            if (u32_mem_size == (u32_size + sizeof(uint32_t))) {
                ESP_LOGI(TAG, "pv_mem_malloc : No.%d Success size=%lu", i_test_no, (unsigned long)u32_size);
            } else {
                ESP_LOGE(TAG, "pv_mem_malloc : No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size);
            }
            // 未使用領域のサイズ
            u32_unused_size = u32_mem_unused_size();
            if (u32_unused_size != (MEM_STORAGE_SIZE - 24 - u32_mem_size)) {
                ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_size);
            }
        }
        // 未使用領域の数
        u32_unused_cnt = u32_mem_unused_cnt();
        if (u32_unused_cnt != 1) {
            ESP_LOGE(TAG, "u32_mem_unused_cnt: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_cnt);
        }
        // メモリ解放
        long l_result = l_mem_free(pv_mem);
        if (u32_size == 0) {
            if (l_result == -1) {
                ESP_LOGI(TAG, "b_com_free: No.%d Success", i_test_no);
            } else {
                ESP_LOGE(TAG, "b_com_free: No.%d Failure", i_test_no);
            }
        } else {
            if (l_result == u32_size) {
                ESP_LOGI(TAG, "b_com_free: No.%d Success", i_test_no);
            } else {
                ESP_LOGE(TAG, "b_com_free: No.%d Failure", i_test_no);
            }
        }
        // テストケース番号更新
        i_test_no++;
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_02
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_02() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 02");
    ESP_LOGI(TAG, "//===========================================================");
    // テスト番号
    int i_test_no = 0;
    // 領域サイズの表示1
    v_task_chk_mem_alloc_disp_area();
    // 割り当て済みのサイズ
    uint32_t u32_alloc_size = 0;
    // 未使用領域のサイズ
    uint32_t u32_unused_size = 0;
    // 未使用領域の数
    uint32_t u32_unused_cnt = 0;
    // 初期処理を確認
    void* pv_mem[100];
    uint32_t u32_total_size = 0;
    uint32_t u32_total_mem_size = 0;
    uint32_t u32_size;
    for (u32_size = 0; u32_size < 100; u32_size++) {
        // 合計サイズ
        u32_total_size += u32_size;
        // メモリ確保
        pv_mem[u32_size] = pv_mem_malloc(u32_size);
        // 割り当て済みのサイズ
        u32_alloc_size = u32_mem_alloc_size();
        if (u32_alloc_size != u32_total_size) {
            ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_alloc_size);
        }
        if (u32_size == 0) {
            if (pv_mem[u32_size] != NULL) {
                ESP_LOGE(TAG, "pv_mem_malloc: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size);
            }
            // 未使用領域のサイズ
            u32_unused_size = u32_mem_unused_size();
            if (u32_unused_size != (MEM_STORAGE_SIZE - 24)) {
                ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_size);
            }
        } else {
            uint32_t u32_mem_size = *((uint32_t*)(pv_mem[u32_size] - sizeof(uint32_t)));
            u32_total_mem_size = u32_total_mem_size + u32_mem_size;
            if (u32_mem_size != (u32_size + sizeof(uint32_t))) {
                ESP_LOGE(TAG, "pv_mem_malloc: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size);
            }
            // 未使用領域のサイズ
            u32_unused_size = u32_mem_unused_size();
            if (u32_unused_size != (MEM_STORAGE_SIZE - 24 - u32_total_mem_size)) {
                ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_size);
            }
        }
        // 未使用領域の数
        u32_unused_cnt = u32_mem_unused_cnt();
        if (u32_unused_cnt != 1) {
            ESP_LOGE(TAG, "u32_mem_unused_cnt: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_unused_cnt);
        }
        // テストケース番号更新
        i_test_no++;
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
    // メモリ解放
    long l_free_size;
    u32_size = 100;
    do {
        // カウントダウン
        u32_size--;
        // メモリ解放
        l_free_size = l_mem_free(pv_mem[u32_size]);
        if (l_free_size == u32_size) {
            ESP_LOGI(TAG, "l_mem_free: No.%d Success idx=%lu size=%ld", i_test_no, (unsigned long)u32_size, l_free_size);
        } else if (l_free_size == -1 && u32_size == 0) {
            ESP_LOGI(TAG, "l_mem_free: No.%d Success idx=%lu size=%ld", i_test_no, (unsigned long)u32_size, l_free_size);
        } else {
            ESP_LOGE(TAG, "l_mem_free: No.%d Failure idx=%lu size=%ld", i_test_no, (unsigned long)u32_size, l_free_size);
        }
        // テストケース番号更新
        i_test_no++;
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    } while(u32_size != 0);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_03
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_03() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 03");
    ESP_LOGI(TAG, "//===========================================================");
    // 領域サイズの表示1
    v_task_chk_mem_alloc_disp_area();
    // 割り当て済みのサイズ
    uint32_t u32_alloc_size = 0;
    // 未使用領域のサイズ
    uint32_t u32_unused_size = MEM_STORAGE_SIZE - 24;
    // 未使用領域の数
    uint32_t u32_unused_cnt = 1;
    // テスト番号
    int i_test_no = 0;
    // メモリ配列
    void* pv_mem[100];
    // サイズ配列
    uint32_t u32_size[100];
    // フリー
    uint8_t u8_free[100];
    // フリーインデックス
    uint32_t u32_free_idx;
    // チェック値
    uint32_t u32_chk_val = 0;
    // フリークリア
    memset(u8_free, 0x00, 100);
    //==========================================================================
    // 直前の偶数サイズを解放しながらメモリ確保
    //==========================================================================
    // サイクルカウンタ
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < 50; u32_idx++) {
        u32_size[u32_idx] = u32_idx + 1;
        // メモリ確保
        pv_mem[u32_idx] = pv_mem_malloc(u32_size[u32_idx]);
        //----------------------------------------------------------------------
        // 割り当て済みのサイズ
        //----------------------------------------------------------------------
        u32_alloc_size = u32_alloc_size + u32_size[u32_idx];
        // メモリ配列
        if (u32_mem_alloc_size() != u32_alloc_size) {
            ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size[u32_idx]);
        }
        //----------------------------------------------------------------------
        // 未使用領域のサイズ
        //----------------------------------------------------------------------
        u32_unused_size = u32_unused_size - u32_size[u32_idx] - sizeof(uint32_t);
        u32_chk_val = u32_mem_unused_size();
        if (u32_chk_val != u32_unused_size) {
            ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_chk_val);
        }
        //----------------------------------------------------------------------
        // 未使用領域の数
        //----------------------------------------------------------------------
        u32_chk_val = u32_mem_unused_cnt();
        if (u32_chk_val != u32_unused_cnt) {
            ESP_LOGE(TAG, "u32_mem_unused_cnt: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_chk_val);
        }
        //----------------------------------------------------------------------
        // メモリ領域の重複チェック
        //----------------------------------------------------------------------
        if (u32_task_chk_memory(u32_size, pv_mem, u32_idx) == u32_idx) {
            ESP_LOGI(TAG, "u32_task_chk_memory: No.%d Success idx=%lu", i_test_no, (unsigned long)u32_idx);
        } else {
            ESP_LOGE(TAG, "u32_task_chk_memory: No.%d Failure idx=%lu", i_test_no, (unsigned long)u32_idx);
        }

        //----------------------------------------------------------------------
        // メモリ解放判定
        //----------------------------------------------------------------------
        if ((u32_idx % 2) == 1) {
            u32_free_idx = u32_idx - 1;
            // メモリ解放（直前の偶数サイズのメモリ領域を解放）
            u32_chk_val = l_mem_free(pv_mem[u32_free_idx]);
            if (u32_chk_val == u32_size[u32_idx - 1]) {
                ESP_LOGI(TAG, "l_mem_free: No.%d Success idx=%lu size=%lu", i_test_no, (unsigned long)u32_free_idx, (unsigned long)u32_chk_val);
                // 割り当てメモリサイズが減る
                u32_alloc_size = u32_alloc_size - u32_chk_val;
                // 未使用領域のサイズは増える
                u32_unused_size = u32_unused_size + u32_chk_val + sizeof(uint32_t) - 24;
                // 未使用領域が増える
                u32_unused_cnt++;
            } else {
                ESP_LOGE(TAG, "l_mem_free: No.%d Failure idx=%lu size=%lu", i_test_no, (unsigned long)u32_free_idx, (unsigned long)u32_chk_val);
            }
            u8_free[u32_free_idx] = 0x01;
        }
        // テストケース番号更新
        i_test_no++;
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }

    //==========================================================================
    // 空き領域情報を表示
    //==========================================================================
//    v_task_chk_mem_alloc_disp_info();

    //==========================================================================
    // フラグメンテーションしている状態で再実行
    //==========================================================================
    uint32_t u32_add = u32_idx;
    uint32_t u32_wk_idx;
    for (u32_idx = 0; u32_idx < 50; u32_idx++) {
        u32_wk_idx = u32_add + u32_idx;
        u32_size[u32_wk_idx] = u32_idx + 1;
        // メモリ確保
//        ESP_LOGI(TAG, "pv_com_mem_alloc size=%d", u32_size[u32_wk_idx]);
        pv_mem[u32_wk_idx] = pv_mem_malloc(u32_size[u32_wk_idx]);
        //----------------------------------------------------------------------
        // 割り当て済みのサイズ
        //----------------------------------------------------------------------
        u32_alloc_size = u32_alloc_size + u32_size[u32_wk_idx];
        // メモリ配列
        if (u32_mem_alloc_size() != u32_alloc_size) {
            ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_size[u32_wk_idx]);
        }
        //----------------------------------------------------------------------
        // メモリ領域の重複チェック
        //----------------------------------------------------------------------
        if (u32_task_chk_memory(u32_size, pv_mem, u32_idx) == u32_idx) {
            ESP_LOGI(TAG, "u32_task_chk_memory: No.%d Success idx=%lu", i_test_no, (unsigned long)u32_idx);
        } else {
            ESP_LOGE(TAG, "u32_task_chk_memory: No.%d Failure idx=%lu", i_test_no, (unsigned long)u32_idx);
        }

        //----------------------------------------------------------------------
        // 空き領域情報を表示
        //----------------------------------------------------------------------
//        v_task_chk_mem_alloc_disp_info();

        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
        //----------------------------------------------------------------------
        // メモリ解放判定
        //----------------------------------------------------------------------
        if ((u32_wk_idx % 2) == 0) {
            u32_free_idx = u32_wk_idx - 1;
            // メモリ解放（直前の奇数サイズのメモリ領域を解放）
            u32_chk_val = l_mem_free(pv_mem[u32_free_idx]);
            if (u32_chk_val == u32_size[u32_free_idx]) {
                ESP_LOGI(TAG, "l_mem_free: No.%d Success idx=%lu addr=%lx size=%lu result=%lu", i_test_no, (unsigned long)u32_free_idx, (unsigned long)pv_mem[u32_free_idx], (unsigned long)u32_size[u32_free_idx], (unsigned long)u32_chk_val);
                // 割り当てメモリサイズが減る
                u32_alloc_size = u32_alloc_size - u32_chk_val;
                // 未使用領域のサイズは増える
                u32_unused_size = u32_unused_size + u32_chk_val + sizeof(uint32_t) - 24;
                // 未使用領域が増える
                u32_unused_cnt++;
            } else {
                ESP_LOGE(TAG, "l_mem_free: No.%d Failure idx=%lu addr=%lx size=%lu result=%lu", i_test_no, (unsigned long)u32_free_idx, (unsigned long)pv_mem[u32_free_idx], (unsigned long)u32_size[u32_free_idx], (unsigned long)u32_chk_val);
            }
            u8_free[u32_free_idx] = 0x01;
        }

        //----------------------------------------------------------------------
        // 空き領域情報を表示
        //----------------------------------------------------------------------
//        v_task_chk_mem_alloc_disp_info();

        // テストケース番号更新
        i_test_no++;
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }

    //==========================================================================
    // 後処理（全てのメモリを解放）
    //==========================================================================
    for (u32_idx = 0; u32_idx < 100; u32_idx++) {
        // メモリ解放判定
        if (u8_free[u32_idx] == 0x00) {
            l_mem_free(pv_mem[u32_idx]);
        }
    }
    // メモリ配列
    u32_chk_val = u32_mem_alloc_size();
    if (u32_chk_val != 0) {
        ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure size=%lu", i_test_no, (unsigned long)u32_chk_val);
    }
    //----------------------------------------------------------------------
    // 空き領域情報を表示
    //----------------------------------------------------------------------
    v_task_chk_mem_alloc_disp_info();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_04
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_04() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 04");
    ESP_LOGI(TAG, "//===========================================================");
    // 最大インデックス
    const uint32_t u32_count = 200;
    // サイズ配列
    uint32_t u32_size[u32_count];
    // メモリ配列
    void* pv_mem[u32_count];

    //==========================================================================
    // ランダムなメモリサイズを生成
    //==========================================================================
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_count; u32_idx++) {
        // サイズ配列
        u32_size[u32_idx] = u32_vutil_random() % 50;
    }

    //==========================================================================
    // 乱数でメモリ確保
    //==========================================================================
    // 割り当て済みのサイズ
    uint32_t u32_alloc_size = 0;
    // 内部で割り当て済みのサイズ
    uint32_t u32_inner_size = 0;
    // 未使用領域のサイズ
    uint32_t u32_unused_size;
    // チェック値
    uint32_t u32_chk_val;
    // メモリ解放値
    long l_free_val;
    // 乱数
    uint32_t u32_rand;
    // テスト番号
    int i_test_no = 0;
    // サイクルカウンタ
    u32_idx = 0;
    for (u32_idx = 0; u32_idx < u32_count; u32_idx++) {
        // テストケース番号
        i_test_no = u32_idx + 1;
        // メモリ配列
        pv_mem[u32_idx] = pv_mem_malloc(u32_size[u32_idx]);
        //----------------------------------------------------------------------
        // 割り当て済みのサイズ
        //----------------------------------------------------------------------
        u32_alloc_size += u32_size[u32_idx];
        u32_chk_val = u32_mem_alloc_size();
        if (u32_chk_val != u32_alloc_size) {
            ESP_LOGE(TAG, "u32_mem_alloc_size: No.%d Failure expected=%lu result=%lu", i_test_no, (unsigned long)u32_alloc_size, (unsigned long)u32_chk_val);
        }
        //----------------------------------------------------------------------
        // 未使用領域のサイズ
        //----------------------------------------------------------------------
        if (u32_size[u32_idx] > 0) {
            u32_inner_size = u32_inner_size + u32_size[u32_idx] + sizeof(uint32_t);
        }
        u32_unused_size = MEM_STORAGE_SIZE - (u32_mem_unused_cnt() * 24) - u32_inner_size;
        u32_chk_val = u32_mem_unused_size();
        if (u32_unused_size != u32_chk_val) {
            ESP_LOGE(TAG, "u32_mem_unused_size: No.%d Failure size=%lu result=%lu", i_test_no, (unsigned long)u32_unused_size, (unsigned long)u32_chk_val);
        }
        //----------------------------------------------------------------------
        // メモリ領域の重複チェック
        //----------------------------------------------------------------------
        if (u32_task_chk_memory(u32_size, pv_mem, u32_idx) == u32_idx) {
            ESP_LOGI(TAG, "u32_task_chk_memory: No.%d Success idx=%lu size=%lu pv=%lx", i_test_no, (unsigned long)u32_idx, (unsigned long)u32_size[u32_idx], (unsigned long)pv_mem[u32_idx]);
        } else {
            ESP_LOGE(TAG, "u32_task_chk_memory: No.%d Failure idx=%lu size=%lu pv=%lx", i_test_no, (unsigned long)u32_idx, (unsigned long)u32_size[u32_idx], (unsigned long)pv_mem[u32_idx]);
        }

        //----------------------------------------------------------------------
        // メモリ解放判定
        //----------------------------------------------------------------------
        // 乱数
        u32_rand = u32_vutil_random();
        if ((u32_rand % 3) < 1 && u32_idx > 0) {
            // メモリ解放（直前の偶数サイズのメモリ領域を解放）
            uint32_t u32_free_idx = u32_rand % u32_idx;
            l_free_val = l_mem_free(pv_mem[u32_free_idx]);
            if (l_free_val == u32_size[u32_free_idx] || (l_free_val == -1 && u32_size[u32_free_idx] == 0)) {
                ESP_LOGI(TAG, "l_mem_free: No.%d Success idx=%lu size=%ld", i_test_no, (unsigned long)u32_free_idx, l_free_val);
                if (l_free_val > 0) {
                    // 想定されるユーザーから見たメモリサイズを更新
                    u32_alloc_size = u32_alloc_size - l_free_val;
                    // 想定される内部的な割り当てメモリサイズを更新
                    u32_inner_size = u32_inner_size - l_free_val - sizeof(uint32_t);
                }
                // メモリを解放
                u32_size[u32_free_idx] = 0;
                pv_mem[u32_free_idx]   = NULL;
            } else {
                ESP_LOGE(TAG, "l_mem_free: No.%d Failure idx=%lu size=%ld", i_test_no, (unsigned long)u32_free_idx, l_free_val);
            }
        }
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_05
 *
 * DESCRIPTION:callocのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_05() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 05");
    ESP_LOGI(TAG, "//===========================================================");
    // テスト番号
    long l_free_size = 0;
    uint32_t u32_chk_idx;
    uint32_t u32_size;
    for (u32_size = 0; u32_size < 1000; u32_size++) {
        // メモリ確保
        uint8_t* pu8_mem = (uint8_t*)pv_mem_calloc(u32_size);
        // 値の検証
        for (u32_chk_idx = 0; u32_chk_idx < u32_size; u32_chk_idx++) {
            if (pu8_mem[u32_chk_idx] != 0x00) {
                ESP_LOGE(TAG, "l_com_mem_free: Failure size=%lu", (unsigned long)u32_size);
            }
        }
        if (u32_chk_idx == u32_size) {
            ESP_LOGI(TAG, "pv_com_mem_calloc: Success size=%lu", (unsigned long)u32_size);
        }
        // 確保したメモリに乱数を書き込み
        b_vutil_set_u8_rand_array(pu8_mem, u32_size);
        // メモリ解放
        l_free_size = l_mem_free(pu8_mem);
        if (l_free_size != u32_size && (l_free_size != -1 && u32_size == 0)) {
            ESP_LOGE(TAG, "l_com_mem_free: Failure size=%lu", (unsigned long)u32_size);
        }
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_06
 *
 * DESCRIPTION:reallocのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_06() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Memory Allocation functions: memory allocate 06");
    ESP_LOGI(TAG, "//===========================================================");
    // テストデータ生成
    uint8_t u8_str[100];
    uint32_t u32_set_idx;
    for (u32_set_idx = 0; u32_set_idx < 100; u32_set_idx++) {
        // ポインタの差分判定
        u8_str[u32_set_idx] = u32_vutil_random();
    }
    uint32_t u32_size;
    for (u32_size = 0; u32_size < 100; u32_size++) {
        // メモリ確保
        uint8_t* pu8_mem_org = (uint8_t*)pv_mem_calloc(u32_size);
        if (u32_size == 0) {
            if (pu8_mem_org != NULL) {
                ESP_LOGE(TAG, "pv_com_mem_calloc: Failure size=%lu", (unsigned long)u32_size);
            }
            continue;
        }
        // データをコピー
        memcpy(pu8_mem_org, u8_str, u32_size);
        // メモリ再定義
        uint8_t* pu8_mem = (uint8_t*)pv_mem_realloc(pu8_mem_org, u32_size);
        if (pu8_mem == NULL) {
            ESP_LOGE(TAG, "pv_com_mem_realloc: Failure size=%lu", (unsigned long)u32_size);
        }
        // データの判定
        if (memcmp(pu8_mem, u8_str, u32_size) == 0) {
            ESP_LOGI(TAG, "pv_com_mem_realloc: Success size=%lu", (unsigned long)u32_size);
        } else {
            ESP_LOGE(TAG, "pv_com_mem_realloc: Failure size=%lu", (unsigned long)u32_size);
        }
        // メモリ解放
        if (l_mem_free(pu8_mem) != u32_size) {
            ESP_LOGE(TAG, "l_com_mem_free: Failure size=%lu", (unsigned long)u32_size);
        }
        // ウェイト
        vTaskDelay(EVT_ENQUEUE_WAIT_TICK);
    }
}

/*******************************************************************************
 *
 * NAME: b_task_chk_memory
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 * uint32_t*        pu32_size       R   サイズ配列
 * void**           ppv_mem         R   メモリ配列
 * uint32_t         u32_idx         R   インデックス
 *
 * RETURNS:
 *  u32_idx:check ok
 * NOTES:
 * None.
 ******************************************************************************/
static uint32_t u32_task_chk_memory(uint32_t* pu32_size, void** ppv_mem, uint32_t u32_idx) {
    // 入力チェック
    if (pu32_size[u32_idx] == 0) {
        return u32_idx;
    }
    // 対象
    uint32_t u32_tgt_size  = pu32_size[u32_idx] + sizeof(uint32_t);
    uint8_t* pu8_tgt_begin = ppv_mem[u32_idx] - sizeof(uint32_t);
    uint8_t* pu8_tgt_end   = (uint8_t*)(pu8_tgt_begin + u32_tgt_size - 1);
    // チェック値
    uint32_t u32_chk_size;
    uint8_t* pu8_chk_begin;
    uint8_t* pu8_chk_end;
    // チェックループ
    uint32_t u32_chk_idx;
    for (u32_chk_idx = 0; u32_chk_idx < u32_idx; u32_chk_idx++) {
        if (pu32_size[u32_chk_idx] == 0) {
            continue;
        }
        u32_chk_size  = pu32_size[u32_chk_idx] + sizeof(uint32_t);
        pu8_chk_begin = ppv_mem[u32_chk_idx] - sizeof(uint32_t);
        pu8_chk_end   = (uint8_t*)(pu8_chk_begin + u32_chk_size - 1);
        // 先頭アドレス検証
        if (pu8_chk_begin <= pu8_tgt_begin && pu8_tgt_begin <= pu8_chk_end) {
            break;
        }
        // 末尾アドレス検証
        if (pu8_chk_begin <= pu8_tgt_end && pu8_tgt_end <= pu8_chk_end) {
            break;
        }
    }
    // 結果返信
    return u32_chk_idx;
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_disp_area
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_disp_area() {
    // 割り当て済みのサイズ
    ESP_LOGI(TAG, "u32_mem_alloc_size: size=%lu", (unsigned long)u32_mem_alloc_size());
    // 未使用領域のサイズ
    ESP_LOGI(TAG, "u32_mem_unused_size:size=%lu", (unsigned long)u32_mem_unused_size());
    // 未使用領域の数
    ESP_LOGI(TAG, "u32_mem_unused_cnt: size=%lu", (unsigned long)u32_mem_unused_cnt());
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mem_alloc_disp_info
 *
 * DESCRIPTION:Memory allocateのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mem_alloc_disp_info() {
    //　空き領域数
    uint32_t u32_cnt = u32_mem_unused_cnt();
    // 空き領域情報
    ts_mem_segment_info_t s_seg_info;
    uint32_t u32_total_size = 0;
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_cnt; u32_idx++) {
        s_seg_info = s_mem_unused_info_addr(u32_idx);
        u32_total_size = u32_total_size + s_seg_info.u32_size;
        ESP_LOGI(TAG, "addr idx=%03lu addr=%08lx size=%lu", (unsigned long)u32_idx, (unsigned long)s_seg_info.pu8_address, (unsigned long)s_seg_info.u32_size);
    }
    // 合計メモリサイズ
    ESP_LOGI(TAG, "area total size=%lu", (unsigned long)u32_total_size);
    // サイズによる検索
    u32_total_size = 0;
    for (u32_idx = 0; u32_idx < u32_cnt; u32_idx++) {
        s_seg_info = s_mem_unused_info_size(u32_idx);
        u32_total_size = u32_total_size + s_seg_info.u32_size;
        ESP_LOGI(TAG, "size idx=%03lu addr=%08lx size=%lu", (unsigned long)u32_idx, (unsigned long)s_seg_info.pu8_address, (unsigned long)s_seg_info.u32_size);
    }
    // 合計メモリサイズ
    ESP_LOGI(TAG, "area total size=%lu", (unsigned long)u32_total_size);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util(void* args) {
    //==========================================================================
    // チェックメソッド
    //==========================================================================
    v_task_chk_value_util_00();
    //==========================================================================
    // 文字列関連関数
    //==========================================================================
    v_task_chk_value_util_01();
    //==========================================================================
    // 生成関数
    //==========================================================================
    v_task_chk_value_util_02();
    //==========================================================================
    // 変換関数
    //==========================================================================
    v_task_chk_value_util_03();
    //==========================================================================
    // 簡易計算関数
    //==========================================================================
    v_task_chk_value_util_04();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util_00
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *   チェック関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util_00() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Value Utility functions: Check functions");
    ESP_LOGI(TAG, "//===========================================================");
    int i_test_no = 0;
    //----------------------------------------------------------------
    /** GPIO番号チェック */
    //----------------------------------------------------------------
    int i_cnt;
    for (i_cnt = -1; i_cnt < (GPIO_NUM_MAX + 1); i_cnt++) {
        switch (i_cnt) {
        case -1:
        case 0:
        case 1:
        case 2:
        case 20:
        case 24:
        case 28:
        case 29:
        case 30:
        case 31:
        case 40:
            if (b_vutil_valid_gpio((gpio_num_t)i_cnt)) {
                ESP_LOGE(TAG, "b_vutil_valid_gpio: No.%d Failure", i_test_no++);
            } else {
                ESP_LOGI(TAG, "b_vutil_valid_gpio: No.%d Success", i_test_no++);
            }
            break;
        default:
            if (b_vutil_valid_gpio((gpio_num_t)i_cnt)) {
                ESP_LOGI(TAG, "b_vutil_valid_gpio: No.%d Success", i_test_no++);
            } else {
                ESP_LOGE(TAG, "b_vutil_valid_gpio: No.%d Failure", i_test_no++);
            }
            break;
        }
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util_01
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *   文字列関連関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util_01() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Value Utility functions: String functions");
    ESP_LOGI(TAG, "//===========================================================");
    /** 配列上の文字インデックス探索（先頭） */
    int i_test_no = 0;
    if (i_vutil_index_of("1234567890JIHGFEDCBA", '1') == 0) {
        ESP_LOGI(TAG, "i_vutil_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_index_of("JIHGFEDCBA1234567890", 'A') == 9) {
        ESP_LOGI(TAG, "i_vutil_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_index_of("JIHGFEDCBA1234567890", '0') == 19) {
        ESP_LOGI(TAG, "i_vutil_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_index_of("JIHGFEDCBA1234567890", '?') == -1) {
        ESP_LOGI(TAG, "i_vutil_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_index_of("", 'A') == -1) {
        ESP_LOGI(TAG, "i_vutil_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_index_of: No.%d Failure", i_test_no++);
    }

    /** 配列上の文字インデックス探索（最終） */
    if (i_vutil_last_index_of("1234567890JIHGFEDCBA1234567890JIHGFEDCBA", '1') == 20) {
        ESP_LOGI(TAG, "i_vutil_last_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_last_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_last_index_of("JIHGFEDCBA1234567890JIHGFEDCBA1234567890", 'A') == 29) {
        ESP_LOGI(TAG, "i_vutil_last_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_last_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_last_index_of("JIHGFEDCBA1234567890JIHGFEDCBA1234567890", '0') == 39) {
        ESP_LOGI(TAG, "i_vutil_last_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_last_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_last_index_of("JIHGFEDCBA1234567890JIHGFEDCBA1234567890", '?') == -1) {
        ESP_LOGI(TAG, "i_vutil_last_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_last_index_of: No.%d Failure", i_test_no++);
    }
    if (i_vutil_last_index_of("", 'A') == -1) {
        ESP_LOGI(TAG, "i_vutil_last_index_of: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_last_index_of: No.%d Failure", i_test_no++);
    }
    // 文字置き換え
    char pc_str[11];
    strcpy(pc_str, "R23R567R9R");
    if (b_vutil_replace_char(pc_str, 'R', '1')) {
        ESP_LOGI(TAG, "b_vutil_replace_char: No.%d Success:%s", i_test_no++, pc_str);
    } else {
        ESP_LOGE(TAG, "b_vutil_replace_char: No.%d Failure:%s", i_test_no++, pc_str);
    }
    if (b_vutil_replace_char(pc_str, 'R', '4')) {
        ESP_LOGI(TAG, "b_vutil_replace_char: No.%d Success:%s", i_test_no++, pc_str);
    } else {
        ESP_LOGE(TAG, "b_vutil_replace_char: No.%d Failure:%s", i_test_no++, pc_str);
    }
    if (b_vutil_replace_char(pc_str, 'R', '8')) {
        ESP_LOGI(TAG, "b_vutil_replace_char: No.%d Success:%s", i_test_no++, pc_str);
    } else {
        ESP_LOGE(TAG, "b_vutil_replace_char: No.%d Failure:%s", i_test_no++, pc_str);
    }
    if (b_vutil_replace_char(pc_str, 'R', '0')) {
        ESP_LOGI(TAG, "b_vutil_replace_char: No.%d Success:%s", i_test_no++, pc_str);
    } else {
        ESP_LOGE(TAG, "b_vutil_replace_char: No.%d Failure:%s", i_test_no++, pc_str);
    }
    if (b_vutil_replace_char(pc_str, 'R', 'X')) {
        ESP_LOGE(TAG, "b_vutil_replace_char: No.%d Failure:%s", i_test_no++, pc_str);
    } else {
        ESP_LOGI(TAG, "b_vutil_replace_char: No.%d Success:%s", i_test_no++, pc_str);
    }
    /** 文字列切り出し */
    char c_edit[21];
    int i_size = i_vutil_substr(c_edit, "JIHGFEDCBA1234567890JIHGFEDCBA1234567890", 0, 10);
    if (i_size == 10 && strcmp(c_edit, "JIHGFEDCBA") == 0) {
        ESP_LOGI(TAG, "i_vutil_substr: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_substr: No.%d Failure", i_test_no++);
    }
    i_size = i_vutil_substr(c_edit, "JIHGFEDCBA1234567890JIHGFEDCBA1234567890", 25, 20);
    if (i_size == 15 && strcmp(c_edit, "EDCBA1234567890") == 0) {
        ESP_LOGI(TAG, "i_vutil_substr: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_substr: No.%d Failure", i_test_no++);
    }
    i_size = i_vutil_substr(c_edit, "JIHGFEDCBA1234567890JIHGFEDCBA1234567890", 40, 10);
    if (i_size == -1 && strcmp(c_edit, "") == 0) {
        ESP_LOGI(TAG, "i_vutil_substr: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_substr: No.%d Failure", i_test_no++);
    }
    i_size = i_vutil_substr(c_edit, "", 0, 10);
    if (i_size == -1 && strcmp(c_edit, "") == 0) {
        ESP_LOGI(TAG, "i_vutil_substr: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_substr: No.%d Failure", i_test_no++);
    }
    /** 先頭からの文字列切り出し */
    i_size = i_vutil_str_left(c_edit, "JIHGFEDCBA1234567890JIHGFEDCBA1234567890", 10);
    if (i_size == 10 && strcmp(c_edit, "JIHGFEDCBA") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_left: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_left: No.%d Failure", i_test_no++);
    }
    i_size = i_vutil_str_left(c_edit, "JIHGFEDCBA1234567890", 30);
    if (i_size == 20 && strcmp(c_edit, "JIHGFEDCBA1234567890") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_left: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_left: No.%d Failure", i_test_no++);
    }
    i_size = i_vutil_str_left(c_edit, "", 10);
    if (i_size == 0 && strcmp(c_edit, "") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_left: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_left: No.%d Failure", i_test_no++);
    }
    /** 文字充填（後方）：充填文字数10文字 */
    char c_edit_rpad[21];
    strcpy(c_edit_rpad, "1234567890");
    int i_add_rpad = i_vutil_str_rpad(c_edit_rpad, '#', sizeof(c_edit_rpad) - 1);
    if (i_add_rpad == 10 && strcmp(c_edit_rpad, "1234567890##########") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_rpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_rpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（後方）：充填文字数0文字 */
    strcpy(c_edit_rpad, "12345678901234567890");
    i_add_rpad = i_vutil_str_rpad(c_edit_rpad, '#', sizeof(c_edit_rpad) - 1);
    if (i_add_rpad == 0 && strcmp(c_edit_rpad, "12345678901234567890") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_rpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_rpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（後方）：編集対象0文字 */
    char c_edit_zero[1];
    c_edit_zero[0] = '\0';
    i_add_rpad = i_vutil_str_rpad(c_edit_zero, '#', sizeof(c_edit_zero) - 1);
    if (i_add_rpad == 0 && c_edit_zero[0] == '\0') {
        ESP_LOGI(TAG, "i_vutil_str_rpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_rpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（後方）：パラメータエラー */
    i_add_rpad = i_vutil_str_rpad(NULL, '#', 20);
    if (i_add_rpad == -1) {
        ESP_LOGI(TAG, "i_vutil_str_rpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_rpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（前方）：充填文字数10文字 */
    char c_edit_lpad[21];
    strcpy(c_edit_lpad, "1234567890");
    int i_add_lpad = i_vutil_str_lpad(c_edit_lpad, '#', sizeof(c_edit_lpad) - 1);
    if (i_add_lpad == 10 && strcmp(c_edit_lpad, "##########1234567890") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_lpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_lpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（前方）：充填文字数0文字 */
    strcpy(c_edit_rpad, "12345678901234567890");
    i_add_lpad = i_vutil_str_lpad(c_edit_rpad, '#', sizeof(c_edit_lpad) - 1);
    if (i_add_lpad == 0 && strcmp(c_edit_rpad, "12345678901234567890") == 0) {
        ESP_LOGI(TAG, "i_vutil_str_lpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_lpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（前方）：編集対象0文字 */
    c_edit_zero[0] = '\0';
    i_add_lpad = i_vutil_str_lpad(c_edit_zero, '#', sizeof(c_edit_zero) - 1);
    if (i_add_lpad == 0 && c_edit_zero[0] == '\0') {
        ESP_LOGI(TAG, "i_vutil_str_lpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_lpad: No.%d Failure", i_test_no++);
    }
    /** 文字充填（前方）：パラメータエラー */
    i_add_lpad = i_vutil_str_lpad(NULL, '#', 21);
    if (i_add_lpad == -1) {
        ESP_LOGI(TAG, "i_vutil_str_lpad: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "i_vutil_str_lpad: No.%d Failure", i_test_no++);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util_02
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *   生成関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util_02() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Value Utility functions: Generate functions");
    ESP_LOGI(TAG, "//===========================================================");
    //----------------------------------------------------------------
    /** 生成関数：疑似乱数の合成で強化された乱数の生成処理 */
    //----------------------------------------------------------------
    int i_chk_cnt = 25000;
    uint32_t u32_cnt_list[256] = {0};
    tu_type_converter_t tu_val;
    int i_cnt;
    for (i_cnt = 0; i_cnt < i_chk_cnt; i_cnt++) {
        tu_val.u32_values[0] = u32_vutil_random();
        u32_cnt_list[tu_val.u8_values[0]]++;
        u32_cnt_list[tu_val.u8_values[1]]++;
        u32_cnt_list[tu_val.u8_values[2]]++;
        u32_cnt_list[tu_val.u8_values[3]]++;
    }
    double d_entropy = d_vutil_entropy(u32_cnt_list, 256, i_chk_cnt * 4);
    // エントロピー（ランダムな度合い）を表示
    int i_test_no = 0;
    ESP_LOGI(TAG, "u32_vutil_rand: No.%d entropy:%lf", i_test_no++, d_entropy);

    //----------------------------------------------------------------
    /** 生成関数：乱数配列（uint8） */
    //----------------------------------------------------------------
    memset(&u32_cnt_list[0], 0x00, sizeof(u32_cnt_list));
    uint8_t u8_rand_array[4];
    for (i_cnt = 0; i_cnt < i_chk_cnt; i_cnt++) {
        b_vutil_set_u8_rand_array(u8_rand_array, 4);
        u32_cnt_list[u8_rand_array[0]]++;
        u32_cnt_list[u8_rand_array[1]]++;
        u32_cnt_list[u8_rand_array[2]]++;
        u32_cnt_list[u8_rand_array[3]]++;
    }
    d_entropy = d_vutil_entropy(u32_cnt_list, 256, i_chk_cnt * 4);
    // エントロピー（ランダムな度合い）を表示
    ESP_LOGI(TAG, "v_vutil_set_u8_rand_array: No.%d entropy:%lf", i_test_no++, d_entropy);

    //----------------------------------------------------------------
    /** 生成関数：乱数配列（uint32） */
    //----------------------------------------------------------------
    uint32_t u32_rnd_list[64] = {0};
    memset(&u32_cnt_list[0], 0x00, sizeof(u32_cnt_list));
    for (i_cnt = 0; i_cnt < i_chk_cnt; i_cnt++) {
        if ((i_cnt % 64) == 0) {
            b_vutil_set_u32_rand_array(u32_rnd_list, 64);
        }
        tu_val.u32_values[0] = u32_rnd_list[i_cnt % 64];
        u32_cnt_list[tu_val.u8_values[0]]++;
        u32_cnt_list[tu_val.u8_values[1]]++;
        u32_cnt_list[tu_val.u8_values[2]]++;
        u32_cnt_list[tu_val.u8_values[3]]++;
    }
    d_entropy = d_vutil_entropy(u32_cnt_list, 256, i_chk_cnt * 4);
    ESP_LOGI(TAG, "v_vutil_set_u32_rand_array: No.%d entropy:%lf", i_test_no++, d_entropy);
    /** 生成関数：乱数文字列 */
    char c_rand_string[33];
    memset(&u32_cnt_list[0], 0x00, sizeof(u32_cnt_list));
    for (i_cnt = 0; i_cnt < i_chk_cnt; i_cnt++) {
        if ((i_cnt % 64) == 0) {
            b_vutil_set_rand_string(c_rand_string, "1234567890abcdefghijABCDEFGHIJ", 32);
//            ESP_LOGI(TAG, "v_vutil_set_rand_string: str:%s", c_rand_string);
        }
        u32_cnt_list[(uint8_t)c_rand_string[i_cnt % 64]]++;
    }
    d_entropy = d_vutil_entropy(u32_cnt_list, 256, i_chk_cnt);
    ESP_LOGI(TAG, "v_vutil_set_rand_string: No.%d entropy:%lf", i_test_no++, d_entropy);

    /** 生成関数：乱数文字列（英数小文字） */
    for (i_cnt = 0; i_cnt < 32; i_cnt++) {
        b_vutil_set_rand_lwr_alphanumeric(c_rand_string, 16);
        ESP_LOGI(TAG, "v_vutil_set_rand_lwr_alphanumeric: str:%s", c_rand_string);
    }

    /** 生成関数：乱数文字列（英数大文字） */
    for (i_cnt = 0; i_cnt < 32; i_cnt++) {
        b_vutil_set_rand_upr_alphanumeric(c_rand_string, 8);
        ESP_LOGI(TAG, "v_vutil_set_rand_upr_alphanumeric: str:%s", c_rand_string);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util_03
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *   変換関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util_03() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Value Utility functions: Convert functions");
    ESP_LOGI(TAG, "//===========================================================");
    /** 変換関数：数値→BCD形式 */
    uint32_t u32_val = u32_vutil_binary_to_bcd(12345678);
    ESP_LOGI(TAG, "u32_vutil_binary_to_bcd: val:%08lx", (unsigned long)u32_val);
    /** 変換関数：BCD形式→数値 */
    u32_val = u32_vutil_bcd_to_binary(0x87654321);
    ESP_LOGI(TAG, "u32_vutil_bcd_to_binary: val:%lu", (unsigned long)u32_val);
    /** 変換関数：uint8→二進表現 */
    u32_val = u32_vutil_u8_to_binary(0xA5);
    ESP_LOGI(TAG, "u32_vutil_u8_to_binary:  val:%lu", (unsigned long)u32_val);
    /** 変換関数：数字文字列から数値変換 */
    u32_val = u32_vutil_array_to_u32("0123456789", 0, 5);
    ESP_LOGI(TAG, "u32_vutil_string_to_u32: val:%lu", (unsigned long)u32_val);
    u32_val = u32_vutil_array_to_u32("9876543210", 5, 5);
    ESP_LOGI(TAG, "u32_vutil_string_to_u32: val:%lu", (unsigned long)u32_val);
    u32_val = u32_vutil_array_to_u32("9876543210", 3, 6);
    ESP_LOGI(TAG, "u32_vutil_string_to_u32: val:%lu", (unsigned long)u32_val);
    u32_val = u32_vutil_array_to_u32("987654321A", 5, 5);
    ESP_LOGI(TAG, "u32_vutil_string_to_u32: val:%lu", (unsigned long)u32_val);
    u32_val = u32_vutil_array_to_u32("98765A3210", 5, 5);
    ESP_LOGI(TAG, "u32_vutil_string_to_u32: val:%lu", (unsigned long)u32_val);
    char c_edit[32];
    uint32_t u32_len = u32_vutil_upper_case(c_edit, "Test String");
    ESP_LOGI(TAG, "Upper case Len:%lu Text:%s", (unsigned long)u32_len, c_edit);
    u32_len = u32_vutil_upper_case(c_edit, "abc 12345 \\^");
    ESP_LOGI(TAG, "Upper case Len:%lu Text:%s", (unsigned long)u32_len, c_edit);
    u32_len = u32_vutil_upper_case(c_edit, "123C56'@+e");
    ESP_LOGI(TAG, "Upper case Len:%lu Text:%s", (unsigned long)u32_len, c_edit);
    //          /** 変換関数：マスキング処理(uint8) */
    //          extern uint8_t u8_vutil_masking(const uint8_t u8_val,
    //                                             const uint8_t* pu8_mask,
    //                                             const uint8_t u8_size);
    //          /** 変換関数：マスキング処理(uint32) */
    //          extern uint32_t u32_vutil_masking(const uint32_t u32_val,
    //                                               const uint8_t* pu8_mask,
    //                                               const uint8_t u8_size);
    //          /** 変換関数：配列のマスキング処理 */
    //          extern void v_vutil_masking(uint8_t* pu8_token,
    //                                         const uint8_t* pu8_mask,
    //                                         const uint8_t u8_size);
    //
    //          /** 変換関数：エントロピー（０．０～１．０の値）の算出 */
    //          extern double d_vutil_entropy(int* pi_list,
    //                                            const int i_list_size,
    //                                            const int i_sample_size);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_value_util_04
 *
 * DESCRIPTION:Value Utilityのテストケース関数
 *   計算関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_value_util_04() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// Value Utility functions: calculation functions");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // 平方根
    //==========================================================================
    ESP_LOGI(TAG, "sqrt(1)=%lld round down", u64_vutil_sqrt(1, false));
    ESP_LOGI(TAG, "sqrt(4)=%lld round down", u64_vutil_sqrt(4, false));
    ESP_LOGI(TAG, "sqrt(16)=%lld round down", u64_vutil_sqrt(16, false));
    ESP_LOGI(TAG, "sqrt(256)=%lld round down", u64_vutil_sqrt(256, false));
    ESP_LOGI(TAG, "sqrt(65536)=%lld round down", u64_vutil_sqrt(65536, false));
    ESP_LOGI(TAG, "sqrt(1)=%lld round up", u64_vutil_sqrt(1, true));
    ESP_LOGI(TAG, "sqrt(4)=%lld round up", u64_vutil_sqrt(4, true));
    ESP_LOGI(TAG, "sqrt(16)=%lld round up", u64_vutil_sqrt(16, true));
    ESP_LOGI(TAG, "sqrt(256)=%lld round up", u64_vutil_sqrt(256, true));
    ESP_LOGI(TAG, "sqrt(65536)=%lld round up", u64_vutil_sqrt(65536, true));
    ESP_LOGI(TAG, "sqrt(0)=%lld round down", u64_vutil_sqrt(0, false));
    ESP_LOGI(TAG, "sqrt(3)=%lld round down", u64_vutil_sqrt(3, false));
    ESP_LOGI(TAG, "sqrt(15)=%lld round down", u64_vutil_sqrt(15, false));
    ESP_LOGI(TAG, "sqrt(255)=%lld round down", u64_vutil_sqrt(255, false));
    ESP_LOGI(TAG, "sqrt(65535)=%lld round down", u64_vutil_sqrt(65535, false));
    ESP_LOGI(TAG, "sqrt(0)=%lld round up", u64_vutil_sqrt(0, true));
    ESP_LOGI(TAG, "sqrt(3)=%lld round up", u64_vutil_sqrt(3, true));
    ESP_LOGI(TAG, "sqrt(15)=%lld round up", u64_vutil_sqrt(15, true));
    ESP_LOGI(TAG, "sqrt(255)=%lld round up", u64_vutil_sqrt(255, true));
    ESP_LOGI(TAG, "sqrt(65535)=%lld round up", u64_vutil_sqrt(65535, true));
    ESP_LOGI(TAG, "sqrt(224)=%lld round down", u64_vutil_sqrt(224, false));
    ESP_LOGI(TAG, "sqrt(225)=%lld round down", u64_vutil_sqrt(225, false));
    ESP_LOGI(TAG, "sqrt(65024)=%lld round down", u64_vutil_sqrt(65024, false));
    ESP_LOGI(TAG, "sqrt(65025)=%lld round down", u64_vutil_sqrt(65025, false));
    ESP_LOGI(TAG, "sqrt(224)=%lld round up", u64_vutil_sqrt(224, true));
    ESP_LOGI(TAG, "sqrt(225)=%lld round up", u64_vutil_sqrt(225, true));
    ESP_LOGI(TAG, "sqrt(65024)=%lld round up", u64_vutil_sqrt(65024, true));
    ESP_LOGI(TAG, "sqrt(65025)=%lld round up", u64_vutil_sqrt(65025, true));
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography
 *
 * DESCRIPTION:暗号処理のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography(void* args) {
    //==========================================================================
    // 共通処理
    //==========================================================================
    v_task_chk_cryptography_00();
    //==========================================================================
    // ハッシュ関数(SHA)
    //==========================================================================
    v_task_chk_cryptography_01();
    //==========================================================================
    // 共通鍵関数(Message Digest)
    //==========================================================================
    v_task_chk_cryptography_02();
    //==========================================================================
    // 共通鍵関数(AES-256-CBC)
    //==========================================================================
    v_task_chk_cryptography_03();
    //==========================================================================
    // 共通鍵関数(AES-256- CTR)
    //==========================================================================
    v_task_chk_cryptography_04();
    //==========================================================================
    // 共通鍵関数(AES-256-GCM)
    //==========================================================================
    v_task_chk_cryptography_05();
    //==========================================================================
    // 共通鍵共有(ECDH)
    //==========================================================================
    v_task_chk_cryptography_06();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_00
 *
 * DESCRIPTION:暗号処理のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_00() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST Crypto random token");
    ESP_LOGI(TAG, "//===========================================================");
    /** 鍵の生成処理 */
    uint32_t u32_len = 32;
    char* pc_key = "135790AbCdEfGhZ";
    ts_u8_array_t* ps_token = ps_crypto_random_token(pc_key, u32_len);
    if (ps_token == NULL) {
        ESP_LOGE(TAG, "ps_crypto_random_token=ERR! no create");
    }
    // トークンの長さ判定
    if (ps_token->t_size == 32) {
        ESP_LOGI(TAG, "ps_crypto_create_key=OK!");
    } else {
        ESP_LOGE(TAG, "ps_crypto_create_key=ERR! Keys length do not match");
    }
    // チェック処理
    uint32_t u32_str_len = strlen(pc_key);
    uint8_t* pu8_values = ps_token->pu8_values;
    uint32_t u32_col_idx;
    uint8_t u8_chk_val;
    uint32_t u32_ch_idx;
    for (u32_col_idx = 0; u32_col_idx < u32_len; u32_col_idx++) {
        u8_chk_val = pu8_values[u32_col_idx];
        for (u32_ch_idx = 0; u32_ch_idx < u32_str_len; u32_ch_idx++) {
            if (u8_chk_val == pu8_values[u32_ch_idx]) {
                // 一致
                break;
            }
            // エラーチェック
            if (u32_ch_idx == u32_str_len) {
                ESP_LOGE(TAG, "ps_crypto_random_token=ERR! invalid token");
                break;
            }
        }
    }
    // トークンの削除処理
    if (sts_mdl_delete_u8_array(ps_token) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_delete_key=OK!");
    } else {
        ESP_LOGE(TAG, "sts_crypto_delete_key=ERR!");
    }
    //
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_01
 *
 * DESCRIPTION:暗号処理のテストケース関数
 * SHAによるハッシュ関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_01() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST SHA");
    ESP_LOGI(TAG, "//===========================================================");
    ts_u8_array_t* ps_array = ps_mdl_clone_u8_array((uint8_t*)"dogcatcrabhuman", 15);
    // ハッシュ関数(SHA1)
    uint8_t u8_sha1_hash[20];
    if (sts_crypto_sha1(ps_array, 0, u8_sha1_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha1 No.1:OK");
        v_dbg_disp_hex_data("sha1 No.1=", u8_sha1_hash, 20);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha1 No.1:Error");
    }
    if (sts_crypto_sha1(ps_array, 3, u8_sha1_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha1 No.2:OK");
        v_dbg_disp_hex_data("sha1 No.2=", u8_sha1_hash, 20);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha1 No.2:Error");
    }
    // ハッシュ関数(SHA224)
    uint8_t u8_sha224_hash[28];
    if (sts_crypto_sha224(ps_array, 0, u8_sha224_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha224 No.1:OK");
        v_dbg_disp_hex_data("sha224 No.1=", u8_sha224_hash, 28);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha224 No.1:Error");
    }
    if (sts_crypto_sha224(ps_array, 3, u8_sha224_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha224 No.2:OK");
        v_dbg_disp_hex_data("sha224 No.2=", u8_sha224_hash, 28);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha224 No.2:Error");
    }
    // ハッシュ関数(SHA256)
    uint8_t u8_sha256_hash[32];
    if (sts_crypto_sha256(ps_array, 0, u8_sha256_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha256 No.1:OK");
        v_dbg_disp_hex_data("sha256 No.1=", u8_sha256_hash, 32);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha256 No.1:Error");
    }
    if (sts_crypto_sha256(ps_array, 3, u8_sha256_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha256 No.2:OK");
        v_dbg_disp_hex_data("sha256 No.2=", u8_sha256_hash, 32);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha256 No.2:Error");
    }
    // ハッシュ関数(SHA384)
    uint8_t u8_sha384_hash[48];
    if (sts_crypto_sha384(ps_array, 0, u8_sha384_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha384 No.1:OK");
        v_dbg_disp_hex_data("sha384 No.1=", u8_sha384_hash, 48);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha384 No.1:Error");
    }
    if (sts_crypto_sha384(ps_array, 3, u8_sha384_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha384 No.2:OK");
        v_dbg_disp_hex_data("sha384 No.2=", u8_sha384_hash, 48);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha384 No.2:Error");
    }
    // ハッシュ関数(SHA512)
    uint8_t u8_sha512_hash[64];
    if (sts_crypto_sha512(ps_array, 0, u8_sha512_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha512 No.1:OK");
        v_dbg_disp_hex_data("sha512 No.1=", u8_sha512_hash, 64);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha512 No.1:Error");
    }
    if (sts_crypto_sha512(ps_array, 3, u8_sha512_hash) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_sha512 No.2:OK");
        v_dbg_disp_hex_data("sha512 No.2=", u8_sha512_hash, 64);
    } else {
        ESP_LOGE(TAG, "sts_crypto_sha512 No.2:Error");
    }
    // 値の削除
    sts_mdl_delete_u8_array(ps_array);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_02
 *
 * DESCRIPTION:メッセージダイジェストのテストケース処理
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_02() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST Message Digest");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // アルゴリズム毎の検証
    //==========================================================================
    // 結果ステータス
    esp_err_t sts_val;
   // ハッシュアルゴリズム
    mbedtls_md_type_t e_target_list[] = {
        MBEDTLS_MD_MD5,         /**< The MD5 message digest. */
        MBEDTLS_MD_SHA1,        /**< The SHA-1 message digest. */
        MBEDTLS_MD_SHA224,      /**< The SHA-224 message digest. */
        MBEDTLS_MD_SHA256,      /**< The SHA-256 message digest. */
        MBEDTLS_MD_SHA384,      /**< The SHA-384 message digest. */
        MBEDTLS_MD_SHA512,      /**< The SHA-512 message digest. */
//        MBEDTLS_MD_RIPEMD160    /**< The RIPEMD-160 message digest. */
    };
    // アルゴリズム
    mbedtls_md_type_t e_type;
    // 対象メッセージ
    const int MSG_LEN = 32;
    ts_u8_array_t* ps_msg = ps_mdl_random_u8_array(MSG_LEN);
    v_dbg_disp_hex_data("// Message=", ps_msg->pu8_values, ps_msg->t_size);
    // アルゴリズム毎のループ
    int i_idx;
    for (i_idx = 0; i_idx < 6; i_idx++) {
        e_type = e_target_list[i_idx];
        // ダイジェスト情報
        const mbedtls_md_info_t* ps_md_info = mbedtls_md_info_from_type(e_type);
        if (ps_md_info == NULL) {
            ESP_LOGE(TAG, "// MAC  algorithm=%d Not supported idx=%d", e_type, i_idx);
            continue;
        }
        // ダイジェストサイズ
        uint8_t u8_digest_len = mbedtls_md_get_size(ps_md_info);
        // ダイジェスト
        uint8_t pu8_digest[u8_digest_len];
        // ダイジェスト盛医ｊ列
        char pc_string[(u8_digest_len * 2) + 1];
        //----------------------------------------------------------------------
        // MAC関数
        //----------------------------------------------------------------------
        sts_val = sts_crypto_mac(e_type, ps_msg, pu8_digest);
        if (sts_val == ESP_OK) {
            v_vutil_u8_to_hex_string(pu8_digest, u8_digest_len, pc_string);
            ESP_LOGI(TAG, "// MAC  algorithm=%d Digest=%s", e_type, pc_string);
        } else {
            ESP_LOGE(TAG, "// MAC  algorithm=%d length=%d Error", e_type, u8_digest_len);
        }
        //----------------------------------------------------------------------
        // HMAC関数
        //----------------------------------------------------------------------
        // HMACキー
        ts_u8_array_t* ps_key = ps_mdl_random_u8_array(u8_digest_len);
        v_dbg_disp_hex_data("// HMAC key=", ps_key->pu8_values, ps_key->t_size);
        // HMAC生成
        sts_val = sts_crypto_hmac(e_type, ps_key, ps_msg, pu8_digest);
        if (sts_val == ESP_OK) {
            v_vutil_u8_to_hex_string(pu8_digest, u8_digest_len, pc_string);
            ESP_LOGI(TAG, "// HMAC algorithm=%d Digest=%s", e_type, pc_string);
        } else {
            ESP_LOGE(TAG, "// HMAC algorithm=%d length=%d Error", e_type, u8_digest_len);
        }
        // キークリア
        sts_mdl_delete_u8_array(ps_key);
    }
    // メッセージクリア
    sts_mdl_delete_u8_array(ps_msg);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_03
 *
 * DESCRIPTION:暗号処理のテストケース関数
 * AES 256bit CBCモードによる暗号化・復号の処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_03() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST AES-256-CBC");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // 値を生成
    //==========================================================================
    /** 共通鍵の生成処理 */
    ts_crypto_keyset_t* ps_keyset = ps_crypto_create_keyset();
    ps_keyset->ps_key      = ps_mdl_random_u8_array(AES_256_KEY_BYTES); // 共通鍵
    ps_keyset->ps_key_iv   = ps_mdl_random_u8_array(IV_BYTES);          // 初期ベクトル
    ps_keyset->ps_nonce    = NULL;                                         // ナンス
    ps_keyset->ps_auth_iv  = NULL;                                         // 認証タグ初期ベクトル

    // 鍵情報表示
    ts_u8_array_t* ps_key = ps_keyset->ps_key;
    v_dbg_disp_hex_data("Key     =", ps_key->pu8_values, IV_BYTES);
    uint8_t* pu8_kiv = ps_keyset->ps_key_iv->pu8_values;
    v_dbg_disp_hex_data("Key IV  =", pu8_kiv, IV_BYTES);
    // 平文
    const uint32_t PLANE_LEN = 64;
    ts_u8_array_t* ps_plane = ps_mdl_random_u8_array(PLANE_LEN);
    uint8_t* pu8_plane = ps_plane->pu8_values;
    v_dbg_disp_hex_data("Plane   =", pu8_plane, PLANE_LEN);

    //==========================================================================
    // 暗号化処理
    //==========================================================================
    // 平文（パディング）（PKCS#7）
    ts_u8_array_t* ps_pad_plane = ps_crypto_pkcs7_padding(ps_plane, AES_BLOCK_BYTES);
    uint8_t* pu8_pad_plane = ps_pad_plane->pu8_values;
    v_dbg_disp_hex_data("PadPlane=", pu8_pad_plane, ps_pad_plane->t_size);
    // 認証タグ
    ts_u8_array_t* ps_enc_auth_tag = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    /** 暗号化処理(AES) */
    ts_u8_array_t* ps_cipher = ps_crypto_aes_gcm_enc(ps_keyset, ps_pad_plane, ps_enc_auth_tag);
    uint8_t* pu8_cipher = ps_cipher->pu8_values;
    v_dbg_disp_hex_data("Cipher  =", pu8_cipher, ps_cipher->t_size);
    if (ps_cipher == NULL) {
        ESP_LOGE(TAG, "Cipher error");
    }
    //==========================================================================
    // 復号処理
    //==========================================================================
    // 認証タグ
    ts_u8_array_t* ps_dec_auth_tag = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    // 復号処理
    ts_u8_array_t* ps_dec_plane = ps_crypto_aes_gcm_dec(ps_keyset, ps_cipher, ps_dec_auth_tag);
    uint8_t* pu8_dec_plane = ps_dec_plane->pu8_values;
    v_dbg_disp_hex_data("DecPlane=", pu8_dec_plane, ps_dec_plane->t_size);
    /** アンパディング処理（PKCS#7） */
    ts_u8_array_t* ps_upad_plane = ps_crypto_pkcs7_unpadding(ps_dec_plane, AES_BLOCK_BYTES);
    uint8_t* pu8_upad_plane = ps_upad_plane->pu8_values;
    v_dbg_disp_hex_data("UpadPlane=", pu8_upad_plane, ps_upad_plane->t_size);

    //==========================================================================
    // 値を解放
    //==========================================================================
    // 共通鍵の削除処理
    sts_crypto_delete_keyset(ps_keyset);
    // 値の解放
    sts_mdl_delete_u8_array(ps_plane);
    // 値の解放
    sts_mdl_delete_u8_array(ps_pad_plane);
    // 値の解放
    sts_mdl_delete_u8_array(ps_cipher);
    // 値の解放
    sts_mdl_delete_u8_array(ps_dec_plane);
    // 値の解放
    sts_mdl_delete_u8_array(ps_upad_plane);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_04
 *
 * DESCRIPTION:暗号処理のテストケース関数
 * AES256 CTRによる暗号化・復号の処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_04() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST AES-256-CTR");
    ESP_LOGI(TAG, "//===========================================================");
    // プレーンテキスト
    const uint32_t PLANE_LEN = 70;
    ts_u8_array_t* ps_plane = ps_mdl_random_u8_array(PLANE_LEN);
    // 共通鍵
    ts_u8_array_t* ps_key = ps_mdl_random_u8_array(AES_256_KEY_BYTES);
    // オフセット
    size_t t_offset_enc = 0;
    size_t t_offset_dec = 0;
    // ナンスカウンター
    uint8_t u8_ncnt_enc[AES_BLOCK_BYTES];
    uint8_t u8_ncnt_dec[AES_BLOCK_BYTES];
    // ナンスカウンターに乱数を設定
    b_vutil_set_u8_rand_array(u8_ncnt_enc, AES_BLOCK_BYTES);
    memcpy(u8_ncnt_dec, u8_ncnt_enc, AES_BLOCK_BYTES);
    // ストリームブロック
    uint8_t u8_sblk_enc[AES_BLOCK_BYTES] = {0x00};
    uint8_t u8_sblk_dec[AES_BLOCK_BYTES] = {0x00};
    // 暗号化処理(AES CTRモード)
    ts_u8_array_t* ps_enc = ps_crypto_aes_ctr(ps_key, &t_offset_enc, u8_ncnt_enc, u8_sblk_enc, ps_plane);
    // 復号処理(AES CTRモード)
    ts_u8_array_t* ps_dec = ps_crypto_aes_ctr(ps_key, &t_offset_dec, u8_ncnt_dec, u8_sblk_dec, ps_enc);
    // 結果表示
    v_dbg_disp_hex_data("ps_key=", ps_key->pu8_values, ps_key->t_size);
    ESP_LOGI(TAG, "t_offset_enc=%d", t_offset_enc);
    ESP_LOGI(TAG, "t_offset_dec=%d", t_offset_dec);
    v_dbg_disp_hex_data("u8_ncnt_enc=", u8_ncnt_enc, 16);
    v_dbg_disp_hex_data("u8_ncnt_dec=", u8_ncnt_dec, 16);
    v_dbg_disp_hex_data("Inp=", ps_plane->pu8_values, ps_plane->t_size);
    v_dbg_disp_hex_data("SBE=", u8_sblk_enc, AES_BLOCK_BYTES);
    v_dbg_disp_hex_data("Enc=", ps_enc->pu8_values, ps_enc->t_size);
    v_dbg_disp_hex_data("SBD=", u8_sblk_dec, AES_BLOCK_BYTES);
    v_dbg_disp_hex_data("Dec=", ps_dec->pu8_values, ps_dec->t_size);
    // メモリを解放
    sts_mdl_delete_u8_array(ps_plane);
    sts_mdl_delete_u8_array(ps_key);
    sts_mdl_delete_u8_array(ps_enc);
    sts_mdl_delete_u8_array(ps_dec);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_05
 *
 * DESCRIPTION:暗号処理のテストケース関数
 * AES256 GCMによる暗号化・復号の処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_05() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST AES-256-GCM");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // 値を生成
    //==========================================================================
    // 共通鍵の生成処理
    ts_crypto_keyset_t* ps_key = ps_crypto_create_keyset();
    ps_key->ps_key      = ps_mdl_random_u8_array(AES_256_KEY_BYTES); // 共通鍵
    ps_key->ps_key_iv   = ps_mdl_random_u8_array(IV_BYTES);          // 初期ベクトル
    ps_key->ps_nonce    = NULL;                                         // ナンス
    ps_key->ps_auth_iv  = ps_mdl_random_u8_array(AES_BLOCK_BYTES);   // 認証タグ初期ベクトル

    // 鍵情報表示
    uint8_t* pu8_key = ps_key->ps_key->pu8_values;
    v_dbg_disp_hex_data("Key     =", pu8_key, IV_BYTES);
    uint8_t* pu8_kiv = ps_key->ps_key_iv->pu8_values;
    v_dbg_disp_hex_data("Key IV  =", pu8_kiv, IV_BYTES);
    uint8_t* pu8_aiv = ps_key->ps_auth_iv->pu8_values;
    v_dbg_disp_hex_data("Auth IV =", pu8_aiv, AES_BLOCK_BYTES);
    // 平文
    const uint32_t PLANE_LEN = 100;
    ts_u8_array_t* ps_enc_plane = ps_mdl_random_u8_array(PLANE_LEN);
    uint8_t* pu8_enc_plane = ps_enc_plane->pu8_values;
    v_dbg_disp_hex_data("EncPlane=", pu8_enc_plane, ps_enc_plane->t_size);

    //==========================================================================
    // 暗号化処理
    //==========================================================================
    ESP_LOGI(TAG, "//-----------------------------------------------------------");
    ts_u8_array_t* ps_auth_tag_enc = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    // 暗号化処理(AES GCMモード)
    ts_u8_array_t* ps_cipher0 = ps_crypto_aes_gcm_enc(ps_key, ps_enc_plane, ps_auth_tag_enc);
    // 暗号表示
    v_dbg_disp_hex_data("Cipher0 =", ps_cipher0->pu8_values, ps_cipher0->t_size);
    // 認証タグ表示
    v_dbg_disp_hex_data("AuthTag0=", ps_auth_tag_enc->pu8_values, IV_BYTES);

    //==========================================================================
    // 復号処理
    //==========================================================================
    ESP_LOGI(TAG, "//-----------------------------------------------------------");
    ts_u8_array_t* ps_auth_tag_dec = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    // 復号処理(AES GCMモード)
    ts_u8_array_t* ps_dec_plane = ps_crypto_aes_gcm_dec(ps_key, ps_cipher0, ps_auth_tag_dec);
    // 復号した値表示
    v_dbg_disp_hex_data("DecPlane=", ps_dec_plane->pu8_values, ps_dec_plane->t_size);
    // 認証タグ表示
    v_dbg_disp_hex_data("AuthTag0=", ps_auth_tag_dec->pu8_values, IV_BYTES);

    //==========================================================================
    // 追加認証データを指定して暗号化
    //==========================================================================
    ESP_LOGI(TAG, "//-----------------------------------------------------------");
    ts_u8_array_t* ps_auth_tag_chk = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    // 認証追加データ
    ts_u8_array_t* ps_auth_iv = ps_key->ps_auth_iv;
    memset(ps_auth_iv->pu8_values, 0x01, AES_BLOCK_BYTES);
    v_dbg_disp_hex_data("Add Data=", ps_auth_iv->pu8_values, AES_BLOCK_BYTES);
    ts_u8_array_t* ps_cipher1 = ps_crypto_aes_gcm_enc(ps_key, ps_enc_plane, ps_auth_tag_chk);
    uint8_t* pu8_cipher1 = ps_cipher1->pu8_values;
    v_dbg_disp_hex_data("Cipher1 =", pu8_cipher1, ps_cipher1->t_size);
    // 認証タグ表示
    v_dbg_disp_hex_data("AuthTag1=", ps_auth_tag_chk->pu8_values, AES_BLOCK_BYTES);

    //==========================================================================
    // 復号処理
    //==========================================================================
    ESP_LOGI(TAG, "//-----------------------------------------------------------");
    ts_u8_array_t* ps_auth_tag1_dec = ps_mdl_empty_u8_array(AES_BLOCK_BYTES);
    // 復号処理(AES GCMモード)
    ts_u8_array_t* ps_dec_plane1 = ps_crypto_aes_gcm_dec(ps_key, ps_cipher1, ps_auth_tag1_dec);
    // 復号した値表示
    v_dbg_disp_hex_data("DecPlane=", ps_dec_plane1->pu8_values, ps_dec_plane1->t_size);
    // 認証タグ表示
    v_dbg_disp_hex_data("AuthTag1=", ps_auth_tag1_dec->pu8_values, IV_BYTES);

    //==========================================================================
    // 値を解放
    //==========================================================================
    // 共通鍵の削除処理
    sts_crypto_delete_keyset(ps_key);
    // 値の解放
    sts_mdl_delete_u8_array(ps_enc_plane);
    // 値の解放
    sts_mdl_delete_u8_array(ps_cipher0);
    // 値の解放
    sts_mdl_delete_u8_array(ps_cipher1);
    // 値の解放
    sts_mdl_delete_u8_array(ps_dec_plane);
    // 認証タグ：暗号化
    sts_mdl_delete_u8_array(ps_auth_tag_enc);
    // 認証タグ：復号
    sts_mdl_delete_u8_array(ps_auth_tag_dec);
    // 認証タグ：復号チェック
    sts_mdl_delete_u8_array(ps_auth_tag_chk);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_cryptography_06
 *
 * DESCRIPTION:暗号処理のテストケース関数
 * ECDHによる共通鍵の共有プロセス
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_cryptography_06() {
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// TEST ECDH");
    ESP_LOGI(TAG, "//===========================================================");
    //==========================================================================
    // ECDHコンテキスト生成
    //==========================================================================
    // クライアント側
    ts_crypto_x25519_context_t* ps_client_ctx = ps_crypto_x25519_client_context();
    if (ps_client_ctx != NULL) {
        ESP_LOGI(TAG, "ps_crypto_x25519_client_context=Client Key OK!");
        v_dbg_disp_hex_data("Client Public Key=", ps_client_ctx->u8_cli_public_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
    } else {
        ESP_LOGE(TAG, "ps_crypto_ecdh_client_context=Client Key ERR! not create");
        return;
    }
    // サーバー側
    ts_crypto_x25519_context_t* ps_server_ctx = ps_crypto_x25519_server_context(ps_client_ctx->u8_cli_public_key);
    if (ps_server_ctx != NULL) {
        ESP_LOGI(TAG, "ps_crypto_x25519_server_context=Server Key OK!");
        v_dbg_disp_hex_data("Server Public Key=", ps_server_ctx->u8_svr_public_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
    } else {
        ESP_LOGE(TAG, "ps_crypto_ecdh_server_context=Server Key ERR! not create");
        return;
    }

    //==========================================================================
    // クライアント側の共通鍵生成
    //==========================================================================
    if (sts_crypto_x25519_client_secret(ps_client_ctx, ps_server_ctx->u8_svr_public_key) == ESP_OK) {
        ESP_LOGI(TAG, "sts_crypto_ecdh_client_secret=client Common Key OK!");
    } else {
        ESP_LOGE(TAG, "sts_crypto_ecdh_client_secret=client Common Key ERR! not create");
        return;
    }

    //==========================================================================
    // 結果検証
    //==========================================================================
    v_dbg_disp_hex_data("Client Public Key=", ps_client_ctx->u8_cli_public_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
    v_dbg_disp_hex_data("Client Public Key=", ps_server_ctx->u8_cli_public_key, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
    v_dbg_disp_hex_data("Server Public Key=", ps_client_ctx->u8_svr_public_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
    v_dbg_disp_hex_data("Server Public Key=", ps_server_ctx->u8_svr_public_key, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
    v_dbg_disp_hex_data("Client Key=", ps_client_ctx->u8_key, CRYPTO_X25519_KEY_SIZE);
    v_dbg_disp_hex_data("Server Key=", ps_server_ctx->u8_key, CRYPTO_X25519_KEY_SIZE);

    //==========================================================================
    // コンテキストの削除
    //==========================================================================
    // クライアント側コンテキストを削除
    v_crypto_x25519_delete_context(ps_client_ctx);
    // サーバー側コンテキストを削除
    v_crypto_x25519_delete_context(ps_server_ctx);

}

/*******************************************************************************
 *
 * NAME: v_task_chk_adc
 *
 * DESCRIPTION: ADCのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_adc(void* args) {
    //==========================================================================
    // ADCのキャリブレーション値の書き込み状況チェック
    //==========================================================================
    v_task_chk_adc_efuse();

    //==========================================================================
    // ADC1設定
    //==========================================================================
    ts_adc_oneshot_context* ps_adc1_ctx;
//    ps_adc1_ctx = ps_adc_oneshot_ctx(ADC_UNIT_1,
//                                     ADC_DIGI_CLK_SRC_DEFAULT,
//                                     ADC_ULP_MODE_DISABLE);
    ps_adc1_ctx = ps_adc_oneshot_calibration_ctx(ADC_UNIT_1,
                                                 ADC_DIGI_CLK_SRC_DEFAULT,
                                                 ADC_ULP_MODE_DISABLE,
                                                 ADC_ATTEN_DB_12);
    // ADC1の値の幅を設定
    // ADC_WIDTH_BIT_9 : 0～511
    // ADC_WIDTH_BIT_10: 0～1023
    // ADC_WIDTH_BIT_11: 0～2047
    // ADC_WIDTH_BIT_12: 0～4095
//    adc1_config_width(ADC_WIDTH_BIT_12);
    // ADC1に割り当てるピン（チャンネル）と、電圧レンジを設定
    // チャンネル
    // ADC1_CHANNEL_0:GPIO36
    // ADC1_CHANNEL_1:GPIO37
    // ADC1_CHANNEL_2:GPIO38
    // ADC1_CHANNEL_3:GPIO39
    // ADC1_CHANNEL_4:GPIO32
    // ADC1_CHANNEL_5:GPIO33
    // ADC1_CHANNEL_6:GPIO34
    // ADC1_CHANNEL_7:GPIO35
    // 電圧レンジ
    // ADC_ATTEN_DB_0  : 0dB減衰   電圧1.1 Vまで
    // ADC_ATTEN_DB_2_5: 2.5dB減衰 電圧1.5 Vまで
    // ADC_ATTEN_DB_6  : 6dB減衰   電圧2.2 Vまで
    // ADC_ATTEN_DB_11 : 11dB減衰  電圧3.9 Vまで
//    adc1_config_channel_atten(ADC_CHANNEL_5, ADC_ATTEN_DB_11);

    // 参照電圧
//    esp_err_t sts_val = adc2_vref_to_gpio(GPIO_NUM_25);
//    if (sts_val != ESP_OK) {
//        ESP_LOGI(TAG, "sts_com_delete_u8_array=OK!  A");
//        return;
//    }
    // チャンネル設定
//    sts_adc_oneshot_config_channel(ps_adc1_ctx, ADC_CHANNEL_6, ADC_ATTEN_DB_12, ADC_BITWIDTH_DEFAULT);
    sts_adc_oneshot_config_channel(ps_adc1_ctx, ADC_CHANNEL_6, ADC_ATTEN_DB_12, ADC_BITWIDTH_DEFAULT);
    // ADCキャラクタリスティックを設定
    // ADC1の設定
//    esp_adc_cal_characteristics_t s_adc1_chars;
//    esp_adc_cal_value_t e_val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &s_adc1_chars);
//    if (e_val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
//        ESP_LOGI(TAG, "Characterized using Two Point Value");
//    } else if (e_val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
//        ESP_LOGI(TAG, "Characterized using eFuse Vref = %d", s_adc1_chars.vref);
//    } else {
//        ESP_LOGI(TAG, "Characterized using Default Vref");
//    }
//    esp_err_t sts_val = sts_io_gpio_util_adc1_config(&s_adc1_chars,
//                                                      ADC_CHANNEL_5,
//                                                      ADC_ATTEN_DB_11,
//                                                      ADC_WIDTH_BIT_12);
//    if (sts_val != ESP_OK) {
//        ESP_LOGE(TAG, "sts_com_gpio_util_adc1_config=ERR!");
//        return;
//    }

    //==========================================================================
    // ADC2設定
    //==========================================================================
    // ADC2の値の幅を設定
    //adc2_config_width(ADC_WIDTH_BIT_12);
    // ADC2に割り当てるピン（チャンネル）と、電圧レンジを設定
    // チャンネル
    // ADC2_CHANNEL_0:GPIO4
    // ADC2_CHANNEL_1:GPIO0
    // ADC2_CHANNEL_2:GPIO2
    // ADC2_CHANNEL_3:GPIO15
    // ADC2_CHANNEL_4:GPIO13
    // ADC2_CHANNEL_5:GPIO12
    // ADC2_CHANNEL_6:GPIO14
    // ADC2_CHANNEL_7:GPIO27
    // ADC2_CHANNEL_8:GPIO25
    // ADC2_CHANNEL_9:GPIO26

    // ※Wifi使用時はADC2の使用は不可
    //adc2_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11);

    //==========================================================================
    // テストケース０
    //==========================================================================
    v_task_chk_adc_00(ps_adc1_ctx);

    //==========================================================================
    // コンテキストを解放
    //==========================================================================
    // ADC1のコンテキストを解放
    sts_adc_oneshot_delete_ctx(ps_adc1_ctx);

}

/*******************************************************************************
 *
 * NAME: v_task_chk_adc_efuse
 *
 * DESCRIPTION: eFuseメモリのADC設定値チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_adc_efuse() {
    // ADC キャリブレーション ラインのフィッティング スキーム
    adc_cali_line_fitting_efuse_val_t e_cali_val;
    if (adc_cali_scheme_line_fitting_check_efuse(&e_cali_val) == ESP_OK) {
        if (e_cali_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP) {
            ESP_LOGI(TAG, "eFuse line fitting: ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP");
        } else if (e_cali_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF) {
            ESP_LOGI(TAG, "eFuse line fitting: ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF");
        } else if (e_cali_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF) {
            ESP_LOGI(TAG, "eFuse line fitting: ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF");
        } else {
            ESP_LOGI(TAG, "eFuse line fitting: other");
        }
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_adc_00
 *
 * DESCRIPTION: ADCのテストケース関数
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_adc_oneshot_context*  ps_adc_ctx  R   コンテキスト
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_adc_00(ts_adc_oneshot_context* ps_adc_ctx) {
    // GPIO番号
    int i_io_num;
    adc_oneshot_channel_to_io(ADC_UNIT_1, ADC_CHANNEL_6, &i_io_num);
    ESP_LOGI(TAG, "ADC1 ADC_CHANNEL_5 GPIO num: %d", i_io_num);
    // 指定回数分サンプリング
    int i_adc_raw_val = 0;
    int i_idx;
    for (i_idx = 0; i_idx < 64; i_idx++) {
        // ADCのワンショット読み込み
        i_adc_raw_val += i_adc_oneshot_voltage(ps_adc_ctx, ADC_CHANNEL_6);
    }
    // 平均値を取得
    i_adc_raw_val /= 64;
    // 算出した値をmVに変換
    ESP_LOGI(TAG, "ADC1 raw to voltage: %dmV", i_adc_raw_val);
    // 電圧取得
    ESP_LOGI(TAG, "ADC1 voltage: %d(raw)", i_adc_oneshot_raw_data(ps_adc_ctx, ADC_CHANNEL_6));
    ESP_LOGI(TAG, "ADC1 voltage: %dmV", i_adc_oneshot_voltage(ps_adc_ctx, ADC_CHANNEL_6));
}

/*******************************************************************************
 *
 * NAME: v_task_chk_touch
 *
 * DESCRIPTION:タッチセンサーのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_touch(void* args) {
    //==========================================================================
    // 起動要因の判定
    //==========================================================================
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                ESP_LOGI(TAG, "Wake up from GPIO %d", pin);
            } else {
                ESP_LOGI(TAG, "Wake up from GPIO");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            ESP_LOGI(TAG, "Wake up from touch on pad %d", esp_sleep_get_touchpad_wakeup_status());
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Not a deep sleep reset");
    }

    //==========================================================================
    // タッチパッド初期化
    //==========================================================================
    // タッチパッド標準初期処理
    sts_io_touchpad_init();
    // タッチパッドのピン有効化処理
    sts_io_touchpad_pin_enable(TOUCH_PAD_NUM0);
    sts_io_touchpad_pin_enable(TOUCH_PAD_NUM2);
    sts_io_touchpad_pin_enable(TOUCH_PAD_NUM8);
    sts_io_touchpad_pin_enable(TOUCH_PAD_NUM9);
    // タッチパッドのチェック有効化
    sts_io_touchpad_check_enable();

    //==========================================================================
    // タッチパッドテストコード
    //==========================================================================
    // フィルタリングされた値の取得
    uint32_t u32_sts_map;
    while (true) {
        u32_sts_map = u32_io_touchpad_pinmap(portMAX_DELAY);
        ESP_LOGI(TAG, "Touchpad map=%lu", (unsigned long)u32_sts_map);
    }

    //==========================================================================
    // End
    //==========================================================================
    ESP_LOGI(TAG, "//===========================================================");
    ESP_LOGI(TAG, "// End of Test");
    ESP_LOGI(TAG, "//===========================================================");
    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_file_util
 *
 * DESCRIPTION:File Utilityのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_file_util(void* args) {
    //==========================================================================
    // SPI初期化処理
    //==========================================================================
    // SPIバスの初期化
    sdmmc_host_t s_host = SDSPI_HOST_DEFAULT();
    s_host.slot = HSPI_HOST;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_13,
        .miso_io_num = GPIO_NUM_16,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 8192,        // 最大転送サイズ
    };
    esp_err_t ret = sts_spi_mst_bus_initialize(s_host.slot, &bus_cfg, SPI_DMA_CH1, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    //==========================================================================
    // SDMMC初期化処理
    //==========================================================================
    // ログ出力
    ESP_LOGI(TAG, "Initializing SD card");
    // SDマウント設定
    esp_vfs_fat_sdmmc_mount_config_t s_mount_cfg;
    s_mount_cfg.format_if_mount_failed = true;      // マウント不可能時のフォーマット設定：フォーマットする
    s_mount_cfg.max_files = 5;                      // 最大オープンファイル数
    s_mount_cfg.allocation_unit_size = 16 * 1024;   // アロケーションユニットサイズ：16KB（Windows最大）
    // SDMMCカードのマウント
    sdmmc_card_t* ps_card = ps_futil_sdmmc_hspi_mount("/sdcard", GPIO_NUM_15, GPIO_NUM_NC, GPIO_NUM_NC, &s_mount_cfg);
//    sdmmc_card_t* ps_card = ps_futil_sdmmc_hspi_mount("/sdcard", GPIO_NUM_25, GPIO_NUM_NC, GPIO_NUM_NC, &s_mount_cfg);
    if (ps_card == NULL) {
        // ファイルシステムのマウント失敗時
        ESP_LOGE(TAG, "%s L#%d Failed to mount filesystem.", __func__, __LINE__);
        return;
    }
    // SDカード情報の表示
    sdmmc_card_print_info(stdout, ps_card);

    //==========================================================================
    // テストコード実施
    //==========================================================================
    // テスト：変換関数
    v_task_chk_file_util_00();
    // ファイル情報の取得関数
    v_task_chk_file_util_01();
    // ファイル・ディレクトリの操作関数
    v_task_chk_file_util_02(ps_card);
    // JSON関連関数
    v_task_chk_file_util_03();

    //==========================================================================
    // SDカードをアンマウント
    //==========================================================================
    // All done, unmount partition and disable SDMMC or SPI peripheral
    sts_futil_sdmmc_unmount();
    ESP_LOGI(TAG, "Card unmounted");
}

/*******************************************************************************
 *
 * NAME: v_task_chk_file_util_00
 *
 * DESCRIPTION:File Utilityのテストケース関数
 *   変換関数のテストケース
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_file_util_00() {
    //==========================================================================
    // 開始
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_00 Begin");
    ESP_LOGI(TAG, "//==========================================================================");

    //==========================================================================
    // ファイルパスチェック
    //==========================================================================
    int i_test_no = 0;
    // パス
    if (b_futil_valid_path("/path")) {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    }
    if (b_futil_valid_path("/path/path/path/test")) {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    }
    if (b_futil_valid_path("/")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path/path/path/test/")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // NULL
    if (b_futil_valid_path(NULL)) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // サイズ
    if (b_futil_valid_path("")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // 先頭文字
    if (b_futil_valid_path("path")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // 禁則文字：\:*?"<>|
    if (b_futil_valid_path("/\\path")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path\\")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/:path")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path:")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/|path")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path|")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // スプリット
    if (b_futil_valid_path("//path/path/test")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    // スプリット
    if (b_futil_valid_path("/path/path/test/")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path/path/test//")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    if (b_futil_valid_path("/path/path//test")) {
        ESP_LOGE(TAG, "b_vutil_valid_path: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_vutil_valid_path: No.%d Success", i_test_no++);
    }
    vTaskDelay(10);

    //==========================================================================
    // 短縮ファイル名の生成処理関連のテストケース
    //==========================================================================
    // ディレクトリリスト
//    v_dbg_file_list("/sdcard");
//    v_dbg_file_list("/sdcard/12345678");
//    v_dbg_file_list("/sdcard/123456~1");
    // ファイル存在チェック
//    b_dbg_disp_open_file("/sdcard/1234~1.json");
//    b_dbg_disp_open_file("/sdcard/12345~1.json");
//    b_dbg_disp_open_file("/sdcard/123456~1.json");
//    b_dbg_disp_open_file("/sdcard/12345678.json");
//    b_dbg_disp_open_file("/sdcard/a234.txt");
//    b_dbg_disp_open_file("/sdcard/a2345.txt");
//    b_dbg_disp_open_file("/sdcard/a23456.txt");
//    b_dbg_disp_open_file("/sdcard/a234567.txt");
//    b_dbg_disp_open_file("/sdcard/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/a23456789.txt");
//    b_dbg_disp_open_file("/sdcard/a23456~1.txt");
//    b_dbg_disp_open_file("/SDCARD/a234.txt");
//    b_dbg_disp_open_file("/SDCARD/a2345.txt");
//    b_dbg_disp_open_file("/SDCARD/a23456.txt");
//    b_dbg_disp_open_file("/SDCARD/a234567.txt");
//    b_dbg_disp_open_file("/SDCARD/a2345678.txt");
//    b_dbg_disp_open_file("/SDCARD/a23456789.txt");
//    b_dbg_disp_open_file("/sdcard/12345678/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/123456~1/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/i2345678/123456~1.json");
//    b_dbg_disp_open_file("/sdcard/i2345678/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/i2345678/a23456~1.txt");
//    b_dbg_disp_open_file("/sdcard/I2345678/123456~1.json");
//    b_dbg_disp_open_file("/sdcard/I2345678/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/I2345678/a23456~1.txt");
//    b_dbg_disp_open_file("/sdcard/I_3456~1/12345678.json");
//    b_dbg_disp_open_file("/sdcard/I_3456~1/123456~1.json");
//    b_dbg_disp_open_file("/sdcard/I_3456~1/a2345678.txt");
//    b_dbg_disp_open_file("/sdcard/I_3456~1/a23456~1.txt");
    vTaskDelay(10);

    //==========================================================================
    // 短縮ファイル名
    //==========================================================================
    char c_path[256];
    b_futil_sfn(c_path, "", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "a", 2);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "a", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "abc", 3);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "abc", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "abc.txt", 4);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "abc.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "+", 5);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "+", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "+abc", 6);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "+abc", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "abc+", 7);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "abc+", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "12345678", 8);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "12345678", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "123456789", 9);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "123456789", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "12345678.txt", 10);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "12345678.t+t", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "12345678.t+t", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "123456789.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "123456789.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn(c_path, "12345678.txtx", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    b_futil_sfn_path(c_path, "12345678.txtx", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/12345678", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/123456789", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/12345678.t+t", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/123456789.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/12345678.txtx", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/123=/12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "/abc/123=/12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "//123=/12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================
    b_futil_sfn_path(c_path, "////12345678.txt", 1);
    ESP_LOGI(TAG, "SNF:%s", c_path);
    //==========================================================================

    //==========================================================================
    // 終了
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_00 End");
    ESP_LOGI(TAG, "//==========================================================================");

}

/*******************************************************************************
 *
 * NAME: v_task_chk_file_util_01
 *
 * DESCRIPTION:File Utilityのテストケース関数
 *   ファイル情報の取得関数
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_file_util_01() {
    //==========================================================================
    // 開始
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_01 Begin");
    ESP_LOGI(TAG, "//==========================================================================");

    //==========================================================================
    // テスト用ディレクトリ・ファイル作成
    //==========================================================================
    char* pc_path = "/sdcard/futil/test-001.dat";
    FILE* fp_test = ps_futil_fopen(pc_path, "wb");
    if (fp_test == NULL) {
        ESP_LOGE(TAG, "File Pointer NULL path:%s", pc_path);
        return;
    }
    // ファイルへの書き込み
    int i_cnt;
    char* pc_num = "1234567890";
    for (i_cnt = 0; i_cnt < 1024; i_cnt++) {
        if (fwrite(&pc_num[i_cnt % 10], sizeof(i_cnt), 1, fp_test) < 1) {
            ESP_LOGE(TAG, "File Create Error");
            break;
        }
    }
    // ファイルクローズ
    fclose(fp_test);

    //==========================================================================
    // ファイルサイズの取得テスト
    //==========================================================================
    long l_size = l_futil_file_size(pc_path);
    if (l_size == 1024 * 4) {
        ESP_LOGI(TAG, "check OK File size size:%ld", l_size);
    } else {
        ESP_LOGE(TAG, "check NG File size size:%ld", l_size);
    }

    //==========================================================================
    // ファイル・ディレクトリの存在判定テスト
    //==========================================================================
    // 存在するケース
    if (b_futil_exist(pc_path)) {
        ESP_LOGI(TAG, "check OK File or Dir Exist");
    } else {
        ESP_LOGE(TAG, "check NG File or Dir Exist");
    }
    // 存在しないケース
    if (!b_futil_exist("/sdcard/futil/none-001.dat")) {
        ESP_LOGI(TAG, "check OK File or Dir Not Exist");
    } else {
        ESP_LOGE(TAG, "check NG File or Dir Not Exist");
    }
    // ファイルが存在するケース
    if (b_futil_file_exist(pc_path)) {
        ESP_LOGI(TAG, "check OK File Exist");
    } else {
        ESP_LOGE(TAG, "check NG File Exist");
    }
    // 存在しないケース
    if (!b_futil_file_exist("/sdcard/futil/none-001.dat")) {
        ESP_LOGI(TAG, "check OK File Not Exist");
    } else {
        ESP_LOGE(TAG, "check NG File Not Exist");
    }
    // ディレクトリが存在するケース
    if (b_futil_directory_exist("/sdcard/futil")) {
        ESP_LOGI(TAG, "check OK File Exist");
    } else {
        ESP_LOGE(TAG, "check NG File Exist");
    }
    // 存在しないケース
    if (!b_futil_directory_exist("/sdcard/futil/none")) {
        ESP_LOGI(TAG, "check OK File Not Exist");
    } else {
        ESP_LOGE(TAG, "check NG File Not Exist");
    }

    //==========================================================================
    // テスト用ディレクトリ・ファイル削除
    //==========================================================================
    // ディレクトリの削除
    if (b_futil_remove_directory("/sdcard/futil")) {
        ESP_LOGI(TAG, "check OK Dir delete");
    } else {
        ESP_LOGE(TAG, "check NG Dir delete");
    }

    //==========================================================================
    // 終了
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_01 End");
    ESP_LOGI(TAG, "//==========================================================================");

}

/*******************************************************************************
 *
 * NAME: v_task_chk_file_util_02
 *
 * DESCRIPTION:File Utilityのテストケース関数
 *   ファイル・ディレクトリの操作関数
 *
 *
 * PARAMETERS:      Name        RW  Usage
 * sdmmc_card_t*    ps_card     R   SDカード情報
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_file_util_02(sdmmc_card_t* ps_card) {
    //==========================================================================
    // 開始
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_02 Begin");
    ESP_LOGI(TAG, "//==========================================================================");

    //==========================================================================
    // テスト対象ファイル作成
    //==========================================================================
    // マウントしたSDMMCへの書き込みファイルオープン
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    FILE* fp_hello = ps_futil_fopen("/sdcard/hello.txt", "w");
    if (fp_hello == NULL) {
        // ファイルオープンエラー
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    //==========================================================================
    // 基本的な操作テスト
    //==========================================================================
    // SDカード情報を取得
    ts_sdmmc_info_t s_info;
    sts_futil_sdmmc_edit_info(&s_info, ps_card);
    // SDカード名をファイルへの書き込み
    fprintf(fp_hello, "Hello %s!\n", ps_card->cid.name);
    // 書き込みファイルをクローズ
    fclose(fp_hello);
    ESP_LOGI(TAG, "check OK card name write");

    // 書き込みしたファイルをリネーム
    // Rename original file
    if (rename("/sdcard/hello.txt", "/sdcard/foo.txt") != 0) {
        // リネームに失敗した場合
        ESP_LOGE(TAG, "Rename failed");
        return;
    }
    ESP_LOGI(TAG, "check OK renaming file");

    // リネームしたファイルを読み込みモードでオープン
    // Open renamed file for reading
    FILE* fp_foo = ps_futil_fopen("/sdcard/foo.txt", "r");
    if (fp_foo == NULL) {
        // ファイルオープンに失敗した場合
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    ESP_LOGI(TAG, "check OK rename file open");

    // 1行読み込みする
    char line[64];
    while (fgets(line, sizeof(line), fp_foo) != NULL) {
        // 改行を終端文字に変換
        // strip newline
        b_vutil_replace_char(line, '\n', '\0');
        ESP_LOGI(TAG, "Readline:%s", line);
    }
    // 読み込みファイルをクローズ
    fclose(fp_foo);
    ESP_LOGI(TAG, "check OK rename file open");

    //==========================================================================
    // ファイルコピー
    //==========================================================================
    if (b_futil_copy_file("/sdcard/foo.txt", "/sdcard/foo2.txt")) {
        if (b_futil_file_exist("/sdcard/foo2.txt")) {
            ESP_LOGI(TAG, "check OK File Copy OK");
        } else {
            ESP_LOGE(TAG, "check OK File Copy NG");
        }
    } else {
        // コピー失敗
        ESP_LOGE(TAG, "check NG File Copy");
    }
    // リネームしたファイルを読み込みモードでオープン
    // Open renamed file for reading
    FILE* fp_foo2 = ps_futil_fopen("/sdcard/foo2.txt", "r");
    if (fp_foo2 == NULL) {
        // ファイルオープンに失敗した場合
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    ESP_LOGI(TAG, "check OK copy file open");

    // 1行読み込みする
    while (fgets(line, sizeof(line), fp_foo) != NULL) {
        // 改行を終端文字に変換
        // strip newline
        b_vutil_replace_char(line, '\n', '\0');
        ESP_LOGI(TAG, "Readline:%s", line);
    }
    // 読み込みファイルをクローズ
    fclose(fp_foo2);
    ESP_LOGI(TAG, "check OK copy file open");

    //==========================================================================
    // ファイル移動
    //==========================================================================
    if (b_futil_move_file("/sdcard/foo2.txt", "/sdcard/foo3.txt")) {
        if (!b_futil_file_exist("/sdcard/foo2.txt") && b_futil_file_exist("/sdcard/foo3.txt")) {
            ESP_LOGI(TAG, "check OK File Move");
        } else {
            ESP_LOGE(TAG, "check NG File Move");
        }
    } else {
        // コピー失敗
        ESP_LOGE(TAG, "check NG File Move");
    }
    // Delete it if it exists
    unlink("/sdcard/foo.txt");
    unlink("/sdcard/foo2.txt");
    unlink("/sdcard/foo3.txt");

    //==========================================================================
    // 複数階層ディレクトリのフォルダ作成テスト
    //==========================================================================
    const char* test_dir_0 = "/sdcard/test";
    const char* test_dir_1 = "/sdcard/test/test/test";
    if (b_futil_make_directory(test_dir_0)) {
        ESP_LOGI(TAG, "check OK Directory No.1 make");
    } else {
        ESP_LOGE(TAG, "check NG Directory No.1 make");
    }
    if (b_futil_make_directory(test_dir_1)) {
        ESP_LOGI(TAG, "check OK Directory No.2 make");
    } else {
        ESP_LOGE(TAG, "check NG Directory No.2 make");
    }
    // ディレクトリ情報参照
    v_dbg_file_info(test_dir_0);
    v_dbg_file_info(test_dir_1);

    //==========================================================================
    // 複数階層ディレクトリのファイル作成テスト
    //==========================================================================
    const char* test_file_0 = "/sdcard/test/test/test/test/try/12345678.txt";
    const char* test_file_1 = "/sdcard/test/test/12345678.txt";
    FILE* ps_test_file_0 = ps_futil_fopen(test_file_0, "w");
    fprintf(ps_test_file_0, "Hello!\n");
    fclose(ps_test_file_0);
    FILE* ps_test_file_1 = ps_futil_fopen(test_file_1, "w");
    fprintf(ps_test_file_1, "Good Bye!\n");
    fclose(ps_test_file_1);
    // ファイル情報参照
    v_dbg_file_info(test_file_0);
    v_dbg_file_info(test_file_1);

    //==========================================================================
    // フォルダ削除テスト
    //==========================================================================
    if (b_futil_remove_directory(test_dir_1)) {
        ESP_LOGI(TAG, "check OK Directory No.2 remove");
    } else {
        ESP_LOGE(TAG, "check NG Directory No.2 remove");
    }
    if (b_futil_remove_directory(test_dir_0)) {
        ESP_LOGI(TAG, "check OK Directory No.1 remove");
    } else {
        ESP_LOGE(TAG, "check NG Directory No.1 remove");
    }

    //==========================================================================
    // パスチェック
    //==========================================================================
//    const char* file_fmt_0 = "/sdcard/many/test%04d/test%04d/test%04d.txt";
//    char pc_path[64];
//    int i_cnt;
//    int64_t i64_time = esp_timer_get_time();
//    for (i_cnt = 0; i_cnt < 100000; i_cnt++) {
//        sprintf(pc_path, file_fmt_0, (i_cnt / 100) % 10, (i_cnt / 10) % 10, i_cnt % 10);
//        if (!b_futil_valid_path(pc_path)) {
//            ESP_LOGE(TAG, "File Path check error");
//        }
//        if ((i_cnt % 100) == 0) {
//            vTaskDelay(10);
//        }
//    }
//    ESP_LOGI(TAG, "File Path check OK time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);

    //==========================================================================
    // 大容量ファイルの作成
    //==========================================================================
    char* pc_wk = "12345678901234567890";
    char pc_num[101];
    sprintf(pc_num, "%s%s%s%s%s", pc_wk, pc_wk, pc_wk, pc_wk, pc_wk);
    int64_t i64_time = esp_timer_get_time();
    FILE* ps_big_file = ps_futil_fopen("/sdcard/big_file.dat", "w");
    int i_size;
    for (i_size = 0; i_size < (1024 * 10); i_size++) {
        if (fwrite(pc_num, 100, 1, ps_big_file) < 1) {
            ESP_LOGE(TAG, "Big File Write Error");
            break;
        }
        // タスク切替
        if ((i_size  % 100) == 0) {
            vTaskDelay(10);
        }
    }
    fclose(ps_big_file);
    ESP_LOGI(TAG, "Big File create end time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);
    if (b_futil_exist("/sdcard/big_file.dat")) {
        ESP_LOGI(TAG, "check OK Big File created");
    } else {
        ESP_LOGE(TAG, "check NG Big File not create");
    }

    //==========================================================================
    // 大容量ファイルのコピー
    //==========================================================================
    i64_time = esp_timer_get_time();
    if (b_futil_copy_file("/sdcard/big_file.dat", "/sdcard/cpy_file.dat")) {
        ESP_LOGI(TAG, "check OK Copy File create");
    } else {
        ESP_LOGE(TAG, "check NG Copy File not create");
    }
    ESP_LOGI(TAG, "Big File copy OK time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);
    if (b_futil_exist("/sdcard/cpy_file.dat")) {
        ESP_LOGI(TAG, "check OK Copy File exist");
    } else {
        ESP_LOGE(TAG, "check NG Copy File not exist");
    }
    // Delete file
    unlink("/sdcard/big_file.dat");
    unlink("/sdcard/cpy_file.dat");
    if (b_futil_exist("/sdcard/big_file.dat")) {
        ESP_LOGE(TAG, "check NG Big File not delete");
    } else {
        ESP_LOGI(TAG, "check OK Big File delete");
    }

    //==========================================================================
    // 複数ファイル・フォルダの作成削除処理
    //==========================================================================
    // ファイルとディレクトリを生成
    const char* file_fmt_0 = "/sdcard/many/test%04d/test%04d/test%04d.txt";
    const char* dir_fmt_0 = "/sdcard/many/test%04d/test%04d/test%04d";
    FILE* ps_file;
    i64_time = esp_timer_get_time();
    char pc_path[64];
    int i_cnt;
    for (i_cnt = 0; i_cnt < 1000; i_cnt++) {
        // ファイル作成
        sprintf(pc_path, file_fmt_0, (i_cnt / 100) % 10, (i_cnt / 10) % 10, i_cnt % 10);
        ps_file = ps_futil_fopen(pc_path, "w");
        fprintf(ps_file, "Hello! No.%d\n", i_cnt);
        fclose(ps_file);
        // ディレクトリの作成
        sprintf(pc_path, dir_fmt_0, (i_cnt / 100) % 10, (i_cnt / 10) % 10, i_cnt % 10);
        b_futil_make_directory(pc_path);
        // タスク切替
        if ((i_cnt  % 100) == 0) {
            vTaskDelay(10);
        }
    }
    ESP_LOGI(TAG, "check OK Many directories create time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);

    //==========================================================================
    // 複数のフォルダとファイルのコピー
    //==========================================================================
    i64_time = esp_timer_get_time();
    if (b_futil_copy_directory("/sdcard/many", "/sdcard/many_cpy")) {
        ESP_LOGI(TAG, "check OK Many directories copy time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);
    } else {
        ESP_LOGE(TAG, "check NG Many directories copy ");
    }

    //==========================================================================
    // フォルダ内容の移動
    //==========================================================================
    if (b_futil_remove_directory("/sdcard/member")) {
        ESP_LOGE(TAG, "check OK Directory remove:/sdcard/member");
    } else {
        ESP_LOGI(TAG, "check NG Directory remove:/sdcard/member");
    }
    if (b_futil_make_directory("/sdcard/member")) {
        ESP_LOGI(TAG, "check OK Member copy Directory make");
    } else {
        ESP_LOGE(TAG, "check NG Member copy Directory make");
    }
    i64_time = esp_timer_get_time();
    if (b_futil_copy_member("/sdcard/many", "/sdcard/member")) {
        ESP_LOGI(TAG, "check OK Member copy time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);
    } else {
        ESP_LOGE(TAG, "check NG Member copy");
    }

    //==========================================================================
    // フォルダの移動
    //==========================================================================
    i64_time = esp_timer_get_time();
    if (b_futil_move_directory("/sdcard/member", "/sdcard/move")) {
        ESP_LOGI(TAG, "check OK Directories move time: %lld ms", (esp_timer_get_time() - i64_time) / 1000);
    } else {
        ESP_LOGE(TAG, "check NG Directories move");
    }
    if (b_futil_exist("/sdcard/member")) {
        ESP_LOGE(TAG, "check OK Move Directories Exist:/sdcard/member");
    } else {
        ESP_LOGI(TAG, "check NG Move Directories not Exist:/sdcard/member");
    }
    if (b_futil_exist("/sdcard/move")) {
        ESP_LOGI(TAG, "check OK Move Directories Exist:/sdcard/move");
    } else {
        ESP_LOGE(TAG, "check NG Move Directories not Exist:/sdcard/move");
    }

    //==========================================================================
    // 一括削除
    //==========================================================================
    if (b_futil_remove_directory("/sdcard/many")) {
        ESP_LOGI(TAG, "check OK Directory remove:/sdcard/many");
    } else {
        ESP_LOGE(TAG, "check NG Directory remove:/sdcard/many");
    }
    if (b_futil_exist("/sdcard/many")) {
        ESP_LOGE(TAG, "check NG Directory exist:/sdcard/many");
    } else {
        ESP_LOGI(TAG, "check OK Directory not exist:/sdcard/many");
    }
    if (b_futil_remove_directory("/sdcard/many_cpy")) {
        ESP_LOGI(TAG, "check OK Directory remove:/sdcard/many_cpy");
    } else {
        ESP_LOGE(TAG, "check NG Directory remove:/sdcard/many_cpy");
    }
    if (b_futil_remove_directory("/sdcard/move")) {
        ESP_LOGI(TAG, "check OK Directory remove:/sdcard/move");
    } else {
        ESP_LOGE(TAG, "check NG Directory remove:/sdcard/move");
    }

    //==========================================================================
    // 終了
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_02 End");
    ESP_LOGI(TAG, "//==========================================================================");

}

/*******************************************************************************
 *
 * NAME: v_task_chk_file_util_03
 *
 * DESCRIPTION:File Utilityのチェックメソッドのテストケース関数
 *   JSON関連関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_file_util_03() {
    //==========================================================================
    // 開始
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_03 Begin");
    ESP_LOGI(TAG, "//==========================================================================");

    //==========================================================================
    // JSONファイルの作成テスト
    //==========================================================================
    // JSONファイルの作成
    cJSON* ps_root  = cJSON_CreateObject();
    cJSON* ps_array = cJSON_CreateArray();
    cJSON_AddNumberToObject(ps_root, "count", 1);
    cJSON_AddItemToObject(ps_root, "items", ps_array);
    cJSON_AddItemToArray(ps_array, cJSON_CreateString("こんにちわ世界"));
    cJSON_AddItemToArray(ps_array, cJSON_CreateFalse());
    cJSON_AddItemToArray(ps_array, cJSON_CreateNull());
    // JSONファイルの書き込み
    esp_err_t sts_val = sts_futil_cjson_write_file("/sdcard/12345~1.json", ps_root);
    if (sts_val == ESP_OK) {
        ESP_LOGI(TAG, "JSON write OK:/sdcard/12345~1.json");
    } else {
        ESP_LOGE(TAG, "JSON write NG:/sdcard/12345~1.json");
    }

    //==========================================================================
    // JSONファイルの読み込みテスト
    //==========================================================================
    cJSON* ps_cjson = ps_futil_cjson_parse_file("/sdcard/12345~1.json", i_vutil_conv_to_kilo(10));
    if (ps_cjson != NULL) {
        ESP_LOGI(TAG, "JSON read OK:/sdcard/12345~1.json");
    } else {
        ESP_LOGE(TAG, "JSON read NG:/sdcard/12345~1.json");
    }

    //==========================================================================
    // JSONファイルの書き込みテスト
    //==========================================================================
    sts_val = sts_futil_cjson_write_file("/sdcard/write.json", ps_cjson);
    if (sts_val == ESP_OK) {
        ESP_LOGI(TAG, "JSON write OK:/sdcard/write.json");
    } else {
        ESP_LOGE(TAG, "JSON write NG:/sdcard/write.json");
    }

    //==========================================================================
    // JSON情報の解放
    //==========================================================================
    // JSONハンドラを解放
    cJSON_free(ps_cjson);
    // 同じ内容のJSONファイルを削除
    if (remove("/sdcard/write.json") == 0) {
        ESP_LOGI(TAG, "File remove OK:/sdcard/write.json");
    } else {
        ESP_LOGE(TAG, "File remove NG:/sdcard/write.json");
    }
    // JSONファイルの削除
    if (remove("/sdcard/12345~1.json") == 0) {
        ESP_LOGI(TAG, "File remove OK:/sdcard/12345~1.json");
    } else {
        ESP_LOGE(TAG, "File remove NG:/sdcard/12345~1.json");
    }

    //==========================================================================
    // 終了
    //==========================================================================
    ESP_LOGI(TAG, "\r\n");
    ESP_LOGI(TAG, "//==========================================================================");
    ESP_LOGI(TAG, "// FUTIL_03 End");
    ESP_LOGI(TAG, "//==========================================================================");

}

/*******************************************************************************
 *
 * NAME: s_com_dt_day_to_date
 *
 * DESCRIPTION:Date Time Utilityのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_date_time(void* args) {
    // テスト：チェックメソッド
    v_task_chk_com_date_time_00();
    // テスト：変換メソッド
    v_task_chk_com_date_time_01();
    // テスト：ウェイト
    v_task_chk_com_date_time_02();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_date_time_00
 *
 * DESCRIPTION:Date Time Utilityのチェックメソッドのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_date_time_00() {
    //----------------------------------------------------------------
    /** 日付チェック */
    //----------------------------------------------------------------
    // チェック番号のカウントアップ
    int i_test_no = 0;
    // 日付妥当性チェック
    // ０年１月１日
    if (b_dtm_valid_date(0, 1, 1)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 0年
    if (b_dtm_valid_date(0, 2, 29)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    if (b_dtm_valid_date(1, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 4年
    if (b_dtm_valid_date(3, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    if (b_dtm_valid_date(4, 2, 29)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    if (b_dtm_valid_date(5, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 100年
    if (b_dtm_valid_date(99, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    if (b_dtm_valid_date(100, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    if (b_dtm_valid_date(101, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 396年
    if (b_dtm_valid_date(395, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    if (b_dtm_valid_date(396, 2, 29)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    if (b_dtm_valid_date(397, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 400年
    if (b_dtm_valid_date(399, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    if (b_dtm_valid_date(400, 2, 29)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    if (b_dtm_valid_date(401, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 1900年
    if (b_dtm_valid_date(1900, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 2000年
    if (b_dtm_valid_date(2000, 2, 29)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 0月1日
    if (b_dtm_valid_date(2019, 0, 1)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 1月1日
    if (b_dtm_valid_date(2019, 1, 1)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 12月1日
    if (b_dtm_valid_date(2019, 12, 1)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 13月1日
    if (b_dtm_valid_date(2019, 13, 1)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 1月31日
    if (b_dtm_valid_date(2019, 1, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 2月28日
    if (b_dtm_valid_date(2019, 2, 28)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 2月29日
    if (b_dtm_valid_date(2019, 2, 29)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 3月31日
    if (b_dtm_valid_date(2019, 3, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 4月30日
    if (b_dtm_valid_date(2019, 4, 30)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 4月31日
    if (b_dtm_valid_date(2019, 4, 31)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 5月31日
    if (b_dtm_valid_date(2019, 5, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 6月30日
    if (b_dtm_valid_date(2019, 6, 30)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 6月31日
    if (b_dtm_valid_date(2019, 6, 31)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 7月31日
    if (b_dtm_valid_date(2019, 7, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 8月31日
    if (b_dtm_valid_date(2019, 8, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 9月30日
    if (b_dtm_valid_date(2019, 9, 30)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 9月31日
    if (b_dtm_valid_date(2019, 9, 31)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 10月31日
    if (b_dtm_valid_date(2019, 10, 31)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 11月30日
    if (b_dtm_valid_date(2019, 11, 30)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    // 11月31日
    if (b_dtm_valid_date(2019, 11, 31)) {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    }
    // 12月31日
    if (b_dtm_valid_date(2019, 12, 30)) {
        ESP_LOGI(TAG, "b_dt_util_valid_date: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_date: No.%d Failure", i_test_no++);
    }
    //----------------------------------------------------------------
    /** 時刻チェック */
    //----------------------------------------------------------------
    // マイナス時
    if (b_dtm_valid_time(-1, 0, 0)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    // マイナス分
    if (b_dtm_valid_time(0, -1, 0)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    // マイナス秒
    if (b_dtm_valid_time(0, 0, -1)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    // ０時０分０秒
    if (b_dtm_valid_time(0, 0, 0)) {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    }
    // ０時０分０秒
    if (b_dtm_valid_time(23, 59, 59)) {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    }
    // オーバー時
    if (b_dtm_valid_time(24, 0, 0)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    // オーバー分
    if (b_dtm_valid_time(0, 60, 0)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    // オーバー秒
    if (b_dtm_valid_time(0, 0, 60)) {
        ESP_LOGE(TAG, "b_dt_util_valid_time: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_valid_time: No.%d Success", i_test_no++);
    }
    //----------------------------------------------------------------
    /** うるう年チェック */
    //----------------------------------------------------------------
    // 紀元前402年
    if (b_dtm_is_leap_year(-402)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前401年（紀元前１年を０年として換算）
    if (b_dtm_is_leap_year(-400)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
    // 紀元前400年
    if (b_dtm_is_leap_year(-399)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前102年
    if (b_dtm_is_leap_year(-101)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前101年
    if (b_dtm_is_leap_year(-100)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前100年
    if (b_dtm_is_leap_year(-99)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前6年
    if (b_dtm_is_leap_year(-5)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前5年
    if (b_dtm_is_leap_year(-4)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
    // 紀元前4年
    if (b_dtm_is_leap_year(-3)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 紀元前1年
    if (b_dtm_is_leap_year(0)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
    // 1年
    if (b_dtm_is_leap_year(1)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 4年
    if (b_dtm_is_leap_year(4)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
    // 96年
    if (b_dtm_is_leap_year(96)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
    // 99年
    if (b_dtm_is_leap_year(99)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 100年
    if (b_dtm_is_leap_year(100)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 101年
    if (b_dtm_is_leap_year(101)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 200年
    if (b_dtm_is_leap_year(200)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 300年
    if (b_dtm_is_leap_year(300)) {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    } else {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    }
    // 400年
    if (b_dtm_is_leap_year(400)) {
        ESP_LOGI(TAG, "b_dt_util_is_leap_year: No.%d Success", i_test_no++);
    } else {
        ESP_LOGE(TAG, "b_dt_util_is_leap_year: No.%d Failure", i_test_no++);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_date_time_util_01
 *
 * DESCRIPTION:Date Time Utilityの変換メソッドのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_date_time_01() {
    /** 変換関数：紀元1月1日からの経過日数への変換、紀元前１年は0年として換算 */
    int i_val = i_dtm_date_to_days(0, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0000/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(1, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0001/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(2, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0002/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(3, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0003/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(4, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0004/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(5, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0005/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(400, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0400/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(401, 1, 1);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 0401/1/1 val:%d", i_val);
    i_val = i_dtm_date_to_days(2019, 10, 7);
    ESP_LOGI(TAG, "u32_dt_util_date_to_days: 2019/10/7 val:%d", i_val);
    /** 変換関数：紀元1月1日からの経過日数から日付への変換 */
//    int i_idx;
//    for (i_idx = 0; i_idx < 1500; i_idx++) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
//    for (i_idx = 36000; i_idx < 37000; i_idx++) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
//    for (i_idx = 145700; i_idx < 146700; i_idx++) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
//    for (i_idx = -360; i_idx > -380; i_idx--) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//    }
//    for (i_idx = 0; i_idx > -1500; i_idx--) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
//    for (i_idx = -36000; i_idx > -37000; i_idx--) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
//    for (i_idx = -146000; i_idx > -147000; i_idx--) {
//        ts_date s_date = s_dt_util_day_to_date(i_idx);
//        ESP_LOGI(TAG, "s_dt_util_day_to_date: In:%d = %05d/%02d/%02d", i_idx, s_date.i_year, s_date.u8_month, s_date.u8_day);
//        vTaskDelay(1);
//    }
    ts_date_t s_date = s_dtm_day_to_date(-737425);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:    BC  2019/01/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-1 * DT_UTIL_DAYS_400YEARS);
    ESP_LOGI(TAG, "s_dt_util_day_to_date: -400Y -0399/01/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-1 * DT_UTIL_DAYS_400YEARS - 1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date: -100Y -0099/01/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-1 * DT_UTIL_DAYS_400YEARS);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:   -4Y -0003/01/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-367);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  -367 -0001/12/13:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-366);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  -366 -0000/01/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-365);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  -365 -0000/01/02:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-31);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:   -31 -0000/12/01:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(-1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:    -1 -0000/12/31:%05d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:    +1  0001/01/02:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(31);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:   +31  0001/02/01:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(364);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  +364  0001/12/31:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(365);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  +365  0002/01/01:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(366);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  +366  0002/01/02:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(DT_UTIL_DAYS_100YEARS - 1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  100Y- 0100/12/31:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(DT_UTIL_DAYS_100YEARS);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  100Y  0101/01/01:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(DT_UTIL_DAYS_400YEARS - 1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  400Y- 0400/12/31:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(DT_UTIL_DAYS_400YEARS);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  400Y  0401/01/01:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(DT_UTIL_DAYS_400YEARS + 1);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:  400Y+ 0401/01/02:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(737424);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:    AD  2019/01/01:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
    s_date = s_dtm_day_to_date(737338);
    ESP_LOGI(TAG, "s_dt_util_day_to_date:    AD  2019/10/07:%04d/%02d/%02d", s_date.i_year, s_date.i_month, s_date.i_day);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_date_time_02
 *
 * DESCRIPTION:Date Time Utilityの変換メソッドのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_date_time_02() {
    int i64_bef = esp_timer_get_time();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int i64_aft = esp_timer_get_time();
    ESP_LOGI(TAG, "High Resolution Timer 10msec %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i64_aft = esp_timer_get_time();
    ESP_LOGI(TAG, "High Resolution Timer 20msec %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    vTaskDelay(60 / portTICK_PERIOD_MS);
    i64_aft = esp_timer_get_time();
    ESP_LOGI(TAG, "High Resolution Timer 60msec %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = esp_timer_get_time();
    ESP_LOGI(TAG, "High Resolution Timer 0usec %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_wait_usec(100000);
    ESP_LOGI(TAG, "High Resolution Timer 100000us %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_wait_msec(100);
    ESP_LOGI(TAG, "High Resolution Timer 100msec  %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_delay_usec(100000);
    ESP_LOGI(TAG, "High Resolution Timer 100000us %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_delay_msec(100);
    ESP_LOGI(TAG, "High Resolution Timer 100msec  %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_delay_until_usec(i64_bef + 100000);
    ESP_LOGI(TAG, "High Resolution Timer 100000us %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
    i64_bef = esp_timer_get_time();
    i64_aft = i64_dtm_delay_until_msec((i64_bef / 1000) + 100);
    ESP_LOGI(TAG, "High Resolution Timer 100msec  %d -> %d = %d", (int)i64_bef, (int)i64_aft, (int)(i64_aft - i64_bef));
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_i2c_mst
 *
 * DESCRIPTION:I2Cのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_i2c_mst(void* args) {
    //==========================================================================
    // I2C Debug
    //==========================================================================
    // 提供された既存機能のテスト
    v_task_chk_com_i2c_mst_00();
    // I2Cの共通機能の動作確認テスト（正常系）
    v_task_chk_com_i2c_mst_01();
    // I2Cの共通機能の動作確認テスト（異常系）
    v_task_chk_com_i2c_mst_02();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_i2c_mst_00
 *
 * DESCRIPTION:既存のI2C機能のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_i2c_mst_00() {
    //==========================================================================
    // I2C Debug Start
    //==========================================================================
    // I2Cハンドル
    i2c_cmd_handle_t v_mst_cmd_hndl;
    // 結果ステータス
    esp_err_t sts_val;
    // 受信データ
    uint8_t u8_rx_data[16];

    //==========================================================================
    // I2C Debug Read
    // Note:RX8900を想定してテストコードを実装
    //==========================================================================
    //--------------------------------------------------------------------------
    // RAMにデータの書き込み
    //--------------------------------------------------------------------------
    // I2Cハンドル生成
    v_mst_cmd_hndl = i2c_cmd_link_create();
    // キューイング：Writeスタートコンディション
    sts_val = i2c_master_start(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.0 Error");
    }
    // アドレス書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, (0x32 << 1), true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.1 Error");
    }
    // レジスタアドレス(RAM)の書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, 0x07, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.2 Error");
    }
    // RAMにデータの書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, 0xAB, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.3 Error");
    }
    // ストップコンディション
    sts_val = i2c_master_stop(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.4 Error");
    }
    // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
    sts_val = i2c_master_cmd_begin(I2C_NUM_0, v_mst_cmd_hndl, 1000 / portTICK_PERIOD_MS);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.5 Error");
    }
    // I2Cリンクの削除
    i2c_cmd_link_delete(v_mst_cmd_hndl);

    //--------------------------------------------------------------------------
    // RAMデータの読み込み
    //--------------------------------------------------------------------------
    // I2Cハンドル生成
    v_mst_cmd_hndl = i2c_cmd_link_create();
    // キューイング：Writeスタートコンディション
    sts_val = i2c_master_start(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.6 Error");
    }
    // デバイスアドレス書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, (0x32 << 1), true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.7 Error");
    }
    // レジスタアドレス(RAM)の書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, 0x07, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.8 Error");
    }

    //--------------------------------------------------------------------------
    // キューイング：Readスタートコンディション
    //--------------------------------------------------------------------------
    // キューイング：Readスタートコンディション
    sts_val = i2c_master_start(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.9 Error");
    }
    // デバイスアドレス書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, ((0x32 << 1) | 0x01), true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.10 Error");
    }
    // データ読み込み
    // I2C_MASTER_ACK
    sts_val = i2c_master_read(v_mst_cmd_hndl, u8_rx_data, 8, I2C_MASTER_LAST_NACK);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.11 Error");
    }
    // ストップコンディション
    sts_val = i2c_master_stop(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.12 Error");
    }
    // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
    sts_val = i2c_master_cmd_begin(I2C_NUM_0, v_mst_cmd_hndl, 1000 / portTICK_PERIOD_MS);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 RX8900 No.13 Error");
    }
    // I2Cリンクの削除
    i2c_cmd_link_delete(v_mst_cmd_hndl);

    //==========================================================================
    // I2C Debug Write
    // Note:ST7032Iを想定してPINGテストコードを実装
    //==========================================================================
//    i64_dt_util_wait_usec(10);
//    // PING
//    ts_i2c_address s_address = {
//            .e_port_no = I2C_NUM_0,   // I2Cポート番号
//            .u16_address = 0x3E       // I2Cスレーブアドレス（10bit時：0b011110～）
//    };
//    sts_val = sts_i2c_mst_util_ping(s_address);
//    if (sts_val != ESP_OK) {
//        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.0 Error sts:%04X", sts_val);
//    }
    //==========================================================================
    // I2C Debug Write
    // Note:ST7032Iを想定してテストコードを実装
    //==========================================================================
    i64_dtm_wait_usec(10);
    // I2Cハンドル生成
    v_mst_cmd_hndl = i2c_cmd_link_create();
    // キューイング：スタートコンディション
    sts_val = i2c_master_start(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.0 Error");
    }
    // アドレス書き込み
    uint8_t u8_data = (0x3E << 1);
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, u8_data, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.1 Error");
    }
    // コマンド書き込み
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, 0x00, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.2 Error");
    }
    sts_val = i2c_master_write_byte(v_mst_cmd_hndl, 0x39, true);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.3 Error");
    }
    // ストップコンディション
    sts_val = i2c_master_stop(v_mst_cmd_hndl);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.4 Error");
    }
    // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
    sts_val = i2c_master_cmd_begin(I2C_NUM_0, v_mst_cmd_hndl, 1000 / portTICK_PERIOD_MS);
    if (sts_val != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 ST7032I No.5 Error");
    }
    // I2Cリンクの削除
    i2c_cmd_link_delete(v_mst_cmd_hndl);

    //==========================================================================
//    i2c_cmd_link_delete(v_mst_cmd_hndl);
//    v_mst_cmd_hndl = i2c_cmd_link_create();
    //==========================================================================
    // キューイング：スタートコンディション
//    sts_val = i2c_master_start(v_mst_cmd_hndl);
    // 一括書き込み
//    uint8_t u8_data_list[] = {(0x3E << 1), 0x00, 0x39};
//    i2c_master_write(v_mst_cmd_hndl, u8_data_list, 3, true);
    // ストップコンディション
//    sts_val = i2c_master_stop(v_mst_cmd_hndl);
    // キューイングされたI2C処理を実行、I2Cドライバの排他ロック取得を最大1秒待つ
//    sts_val = i2c_master_cmd_begin(I2C_NUM_0, v_mst_cmd_hndl, 1000 / portTICK_PERIOD_MS);
//    if (sts_val != ESP_OK) {
//        return;
//    }

    //==========================================================================
    // I2C Debug End
    //==========================================================================
    // I2Cリンクの削除
//    i2c_cmd_link_delete(v_mst_cmd_hndl);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_i2c_mst_01
 *
 * DESCRIPTION:I2C機能のテストケース関数（正常系）
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_i2c_mst_01() {
    // 実行結果
    esp_err_t sts_result;
    // アドレス
    ts_i2c_address_t s_address;
    // 送信データ
    uint8_t u8_tx_data[16];
    // 受信データ
    uint8_t u8_rx_data[16];
    //==========================================================================
    // I2C Debug Util Write
    // RX8900のRAMデータへの書き込みと読み込み
    //==========================================================================
    //--------------------------------------------------------------------------
    // Pingの送信
    //--------------------------------------------------------------------------
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x3E;
    sts_result = sts_io_i2c_mst_ping(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.0 Error sts:%X", sts_result);
    }

    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x32;
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.1 Error sts:%X", sts_result);
    }
    //--------------------------------------------------------------------------
    // レジスタ(RAM)への書き込み
    //--------------------------------------------------------------------------
    u8_tx_data[0] = 0x07;
    u8_tx_data[1] = 0xAB;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.2 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.3 Error sts:%X", sts_result);
    }
    //--------------------------------------------------------------------------
    // レジスタ(RAM)へのアドレス書き込み
    //--------------------------------------------------------------------------
    u8_tx_data[0] = 0x00;
    sts_result = sts_io_i2c_mst_write(u8_tx_data, 1, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.4 Error");
    }
    //--------------------------------------------------------------------------
    // 読み込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_read(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.5 Error");
    }
    //--------------------------------------------------------------------------
    // レジスタ(RAM)からの読み込み
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_read_stop(u8_rx_data, 10);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.6 Error");
    }
    //--------------------------------------------------------------------------
    // 読み込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_read(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.7 Error");
    }
    //--------------------------------------------------------------------------
    // レジスタ(RAM)からの読み込み
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_read_stop(u8_rx_data, 5);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c_01 No.8 Error");
    }

    //==========================================================================
    // I2C Debug Write
    // ST7032IのLCDドライバを想定
    //==========================================================================
    //--------------------------------------------------------------------------
    // Pingの送信
    //--------------------------------------------------------------------------
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x3E;
    sts_result = sts_io_i2c_mst_ping(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.0 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x3E;
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.0 Error");
    }
    //--------------------------------------------------------------------------
    // Function Set Default(IS=1)
    //--------------------------------------------------------------------------
    u8_tx_data[0] = 0x00;
    u8_tx_data[1] = 0x39;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.1 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.2 Error");
    }
    //--------------------------------------------------------------------------
    // Internal OSC frequency
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x14;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.3 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.4 Error");
    }
    //--------------------------------------------------------------------------
    // Display Contrast Lower set
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x70 | 0x08;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.5 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.6 Error");
    }
    //--------------------------------------------------------------------------
    // Power/ICON Control/Contrast Higher set
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x50 | 0x0E;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.7 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.8 Error");
    }
    //--------------------------------------------------------------------------
    // Follower Control
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x6C;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.9 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.10 Error");
    }
    //--------------------------------------------------------------------------
    // Function Set Default(IS=0)
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x38;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.11 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.12 Error");
    }
    //--------------------------------------------------------------------------
    // Display Switch On
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x08 | 0x04;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.13 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.14 Error");
    }
    //--------------------------------------------------------------------------
    // Clear Screen
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x01;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.15 Error");
    }
    // 次のコマンド実行までウェイト
    i64_dtm_delay_usec(1080);

    //==========================================================================
    // テストデータの書き込み
    //==========================================================================
    //--------------------------------------------------------------------------
    // トランザクション開始
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.16 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.17 Error");
    }
    //--------------------------------------------------------------------------
    // Set Cursor
    //--------------------------------------------------------------------------
    u8_tx_data[1] = 0x80;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 2, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.18 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.19 Error");
    }
    //--------------------------------------------------------------------------
    // 文字列の書き込み
    //--------------------------------------------------------------------------
    uint8_t u8_tx_string[] = "X0123456789";
    u8_tx_string[0] = 0x40;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_string, 11, true);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.20 Error");
    }
    //--------------------------------------------------------------------------
    // トランザクション終了
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_end();
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.21 Error %x", sts_result);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_com_i2c_mst_02
 *
 * DESCRIPTION:I2C機能のテストケース関数（異常系）
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_com_i2c_mst_02() {
    // 実行結果
    esp_err_t sts_result;
    // アドレス
    ts_i2c_address_t s_address;
    // 送信データ
    uint8_t u8_tx_data[16];
    // 受信データ
    uint8_t u8_rx_data[16];
    //==========================================================================
    // I2C Error Check
    // ST7032IのLCDドライバを想定した順序制御エラー
    //==========================================================================
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x3E;
    //--------------------------------------------------------------------------
    // データ読み込み
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_read(u8_rx_data, 8);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.1 Error");
    }
    //--------------------------------------------------------------------------
    // データ読み込み
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_read_stop(u8_rx_data, 8);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.2 Error");
    }
    //--------------------------------------------------------------------------
    // データ書き込み
    //--------------------------------------------------------------------------
    u8_tx_data[0] = 0x12;
    u8_tx_data[1] = 0x13;
    u8_tx_data[2] = 0x14;
    sts_result = sts_io_i2c_mst_write(u8_tx_data, 3, true);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.3 Error");
    }
    //--------------------------------------------------------------------------
    // データ書き込み
    //--------------------------------------------------------------------------
    u8_tx_data[0] = 0x22;
    u8_tx_data[1] = 0x23;
    u8_tx_data[2] = 0x24;
    sts_result = sts_io_i2c_mst_write_stop(u8_tx_data, 3, true);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.4 Error");
    }
    //==========================================================================
    // スタートコンディションの送信
    //==========================================================================
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_OK) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.5 Error");
    }
    //--------------------------------------------------------------------------
    // 初期化処理
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_init(I2C_NUM_0, I2C_FREQ_HZ_STD, GPIO_NUM_17, GPIO_NUM_16, GPIO_PULLUP_ENABLE);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.6 Error");
    }
    //--------------------------------------------------------------------------
    // トランザクション開始
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_begin();
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.7 Error");
    }
    //--------------------------------------------------------------------------
    // トランザクション終了
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_end();
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.8 Error");
    }
    //--------------------------------------------------------------------------
    // 読み込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_read(s_address);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.9 Error");
    }
    //--------------------------------------------------------------------------
    // 書き込みスタートコンディションの送信
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_start_write(s_address);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.10 Error");
    }
    //--------------------------------------------------------------------------
    // Ping
    //--------------------------------------------------------------------------
    sts_result = sts_io_i2c_mst_ping(s_address);
    if (sts_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "v_task_chk_i2c No.11 Error");
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_rx8900
 *
 * DESCRIPTION:RX8900のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_rx8900(void* args) {
    //==========================================================================
    // 初期処理・リセット処理
    //==========================================================================
    esp_err_t sts = sts_rx8900_init(I2C_NUM_0);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.2 Error");
    }
    sts = sts_rx8900_reset(I2C_NUM_0);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.3 Error");
    }
    //==========================================================================
    // データ読み込み
    //==========================================================================
    ts_rx8900_register_t s_register = s_rx8900_read(I2C_NUM_0);
    ts_rx8900_datetime_t ts_dt = s_register.ts_datetime;
    ESP_LOGI(TAG, "sts_drv_rx8900:%02d/%02d/%02d %02d:%02d:%02d",
             ts_dt.u8_year, ts_dt.u8_month, ts_dt.u8_day, ts_dt.u8_hour, ts_dt.u8_min, ts_dt.u8_sec);

    //==========================================================================
    // データ書き込み
    //==========================================================================
    // 日付書き込み
    s_register.ts_datetime.u8_year  = 20;
    s_register.ts_datetime.u8_month = 5;
    s_register.ts_datetime.u8_day   = 11;
    s_register.ts_datetime.u8_week  = DRV_RX8900_MONDAY;
    s_register.ts_datetime.u8_hour  = 5;
    s_register.ts_datetime.u8_min   = 10;
    s_register.ts_datetime.u8_sec   = 20;
    sts = sts_rx8900_write_datetime(I2C_NUM_0, s_register.ts_datetime);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.4 Error");
    }
    // アラーム書き込み（分）
    sts = sts_rx8900_write_alarm_min(I2C_NUM_0, 10, false);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.5 Error");
    }
    // アラーム書き込み（時）
    sts = sts_rx8900_write_alarm_hour(I2C_NUM_0, 10, false);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.6 Error");
    }
    // アラーム書き込み（曜日）
    uint8_t u8_week = DRV_RX8900_SUNDAY |
                      DRV_RX8900_MONDAY |
                      DRV_RX8900_THURSDAY |
                      DRV_RX8900_WEDNESDAY |
                      DRV_RX8900_TUESDAY |
                      DRV_RX8900_FRIDAY |
                      DRV_RX8900_SATURDAY;
    sts = sts_rx8900_write_alarm_week(I2C_NUM_0, u8_week, false);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.7 Error");
    }
    // カウンタ書き込み
    sts = sts_rx8900_write_counter(I2C_NUM_0, 0);
    if (sts != ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_rx8900 No.8 Error");
    }
    // レジスタ読み込み
    s_register = s_rx8900_read(I2C_NUM_0);
    ESP_LOGI(TAG, "sts_drv_rx8900 Alm:%02x:%02d:%02d Cnt:%d",
            s_register.u8_alarm_day_or_week, s_register.u8_alarm_hour, s_register.u8_alarm_min, s_register.u16_counter);
    int i_cnt;
    for (i_cnt = 0; i_cnt < 10; i_cnt++) {
        s_register = s_rx8900_read(I2C_NUM_0);
        ts_dt = s_register.ts_datetime;
        ESP_LOGI(TAG, "sts_drv_rx8900_init:%02d/%02d/%02d %02d:%02d:%02d",
                 ts_dt.u8_year, ts_dt.u8_month, ts_dt.u8_day, ts_dt.u8_hour, ts_dt.u8_min, ts_dt.u8_sec);
        ESP_LOGI(TAG, "sts_drv_rx8900_init:%d->%fC", s_register.u8_temperature, f_rx8900_celsius(s_register.u8_temperature));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_st7032i
 *
 * DESCRIPTION:ST7032Iのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_st7032i(void* args) {
    // 正常系
    v_task_chk_st7032i_00();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_st7032i_00
 *
 * DESCRIPTION:ST7032Iのテストケース関数（正常系）
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_st7032i_00() {
    //==========================================================================
    // ST7032I 初期化
    //==========================================================================
    // init
    esp_err_t sts_result = sts_st7032i_init(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_init OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_init Error");
    }
    // コントラスト設定
    sts_result = sts_st7032i_set_contrast(I2C_NUM_0, 0x28);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_contrast OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_contrast Error");
    }
    // カーソル設定
    te_st7032i_disp_sts_t e_disp_sts = DRV_ST7032I_DISP_ON
                                     | DRV_ST7032I_DISP_CURSOR
                                     | DRV_ST7032I_DISP_BLINK
                                     | DRV_ST7032I_DISP_ICON;
    sts_result = sts_st7032i_disp_control(I2C_NUM_0, e_disp_sts);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_disp_control OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_disp_control Error");
    }

    //==========================================================================
    // ST7032I アイコン表示
    //==========================================================================
    // アイコン書き込み
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x00, 0x1F);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_write_icon OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_write_icon Error");
    }
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x01, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x02, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x03, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x04, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x05, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x06, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x07, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x08, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x09, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0A, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0B, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0C, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0D, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0E, 0x1F);
    sts_result = sts_st7032i_write_icon(I2C_NUM_0, 0x0F, 0x1F);
    // 3秒待つ
    i64_dtm_delay_msec(3000);

    //==========================================================================
    // ST7032I 書き込み
    //==========================================================================
    // カーソル設定
    sts_result = sts_st7032i_set_cursor(I2C_NUM_0, 0, 0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor Error");
    }
    // 文字書き込み
    sts_result = sts_st7032i_write_char(I2C_NUM_0, '@');
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_write_char OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_write_char Error");
    }
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'T');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'E');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'S');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'T');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, ' ');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'O');
    sts_result = sts_st7032i_write_char(I2C_NUM_0, 'K');
    // カーソル設定
    sts_result = sts_st7032i_set_cursor(I2C_NUM_0, 1, 5);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor Error");
    }
    // 文字列書き込み
    sts_result = sts_st7032i_write_string(I2C_NUM_0, "ABCDE");
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_write_string OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_write_string Error");
    }
    // 3秒待つ
    i64_dtm_delay_msec(3000);

    //==========================================================================
    // ST7032I クリア
    //==========================================================================
    // ST7032Iクリアアイコン
    sts_result = sts_st7032i_clear_icon(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_clear_icon OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_clear_icon Error");
    }
    // 2秒待つ
    i64_dtm_delay_msec(3000);
    // ST7032Iクリアスクリーン
    sts_result = sts_st7032i_clear_screen(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_clear_screen OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_clear_screen Error");
    }
    // 3秒待つ
    i64_dtm_delay_msec(3000);

    //==========================================================================
    // ST7032I 書き込み
    //==========================================================================
    // カーソル設定
    sts_result = sts_st7032i_set_cursor(I2C_NUM_0, 0, 0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor Error");
    }
    // 文字列書き込み
    sts_result = sts_st7032i_write_string(I2C_NUM_0, "1234567890123456");
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_write_string OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_write_string Error");
    }
    // カーソル設定
    sts_result = sts_st7032i_set_cursor(I2C_NUM_0, 1, 0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor Error");
    }
    // 文字列書き込み
    sts_result = sts_st7032i_write_string(I2C_NUM_0, "1234567890123456");
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_write_string OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_write_string Error");
    }

    //==========================================================================
    // ST7032I カーソル制御
    //==========================================================================
    // 2秒待つ
    i64_dtm_delay_msec(2000);
    // カーソルをホームポジションに移動
    sts_result = sts_st7032i_return_home(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_return_home OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_return_home Error");
    }
    // 2秒待つ
    i64_dtm_delay_msec(2000);
    // カーソル設定
    sts_result = sts_st7032i_set_cursor(I2C_NUM_0, 1, 0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_set_cursor Error");
    }
    // 2秒待つ
    i64_dtm_delay_msec(2000);

    //--------------------------------------------------------------------------
    // カーソル右移動
    //--------------------------------------------------------------------------
    sts_result = sts_st7032i_cursor_shift_r(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_cursor_shift_r OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_cursor_shift_r Error");
    }
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_r(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_r(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_r(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_r(I2C_NUM_0);
    // 2秒待つ
    i64_dtm_delay_msec(2000);
    //--------------------------------------------------------------------------
    // カーソル左移動
    //--------------------------------------------------------------------------
    sts_result = sts_st7032i_cursor_shift_l(I2C_NUM_0);
    if (sts_result == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_st7032i:sts_st7032i_cursor_shift_l OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_st7032i:sts_st7032i_cursor_shift_l Error");
    }
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_l(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_l(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_l(I2C_NUM_0);
    // 0.5秒待つ
    i64_dtm_delay_msec(500);
    // カーソル右移動
    sts_result = sts_st7032i_cursor_shift_l(I2C_NUM_0);
}

/*******************************************************************************
 *
 * NAME: v_task_chk_adxl345
 *
 * DESCRIPTION:ADXL345のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_adxl345(void* args) {
    //==========================================================================
    // ADXL345初期化
    //==========================================================================
    ts_i2c_address_t s_address;
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = I2C_ADDR_ADXL345_H;
    esp_err_t sts = sts_adxl345_init(s_address, 0x0A);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_adxl345 | sts_adxl345_init OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_adxl345 | sts_adxl345_init Error");
    }
    //==========================================================================
    // 較正処理
    //==========================================================================
    sts = sts_adxl345_zeroing(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "v_task_chk_adxl345 | sts_adxl345_calibration OK");
    } else {
        ESP_LOGE(TAG, "v_task_chk_adxl345 | sts_adxl345_calibration Error");
    }
    //==========================================================================
    // 加速度読み込みテスト
    //==========================================================================
    sts_adxl345_set_offset(s_address, -1, -3, -60);
//    sts_adxl345_set_offset(s_address, 0, 0, 0);
//    sts_adxl345_zeroing(s_address);
//    sts_adxl345_set_data_format(s_address, DRV_ADXL345_4G, true, false, false);
    ts_adxl345_register_t s_register;
    sts_adxl345_read(s_address, &s_register);
    ESP_LOGI(TAG, "sts_adxl345_read O(%d,%d,%d)", s_register.i8_offset_x, s_register.i8_offset_y, s_register.i8_offset_z);
    ts_adxl345_axes_data_t s_gdata;
//    int8_t i8_offset = -128;
    int i_wait = 0;
    int i_cnt;
    for (i_cnt = 0; i_cnt < 100; i_cnt++) {
        i64_dtm_delay_msec(i_wait);
        i_wait = 500;
//        if ((i_cnt % 4) == 0) {
//            i8_offset++;
//            sts_adxl345_set_offset(s_address, i8_offset, i8_offset, i8_offset);
//            ESP_LOGI(TAG, "sts_adxl345_read O:f(%d,%d,%d)", i8_offset, i8_offset, i8_offset);
//        }
        sts_adxl345_read_g(s_address, &s_gdata);
        int16_t i16_g = i16_adxl345_conv_g_val(&s_gdata, false);
        ESP_LOGI(TAG, "sts_adxl345_read G:f(%d,%d,%d)=%d", s_gdata.i16_data_x, s_gdata.i16_data_y, s_gdata.i16_data_z, i16_g);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_lis3dh
 *
 * DESCRIPTION:LIS3DHのテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_lis3dh(void* args) {
    //==========================================================================
    // 初期設定
    //==========================================================================
    ts_i2c_address_t s_address;
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = 0x18;
    // レート設定
    ESP_LOGI(TAG, "LIS3DH set rate");
    esp_err_t sts = sts_lis3dh_set_rate(s_address, false, DRV_LIS3DH_RATE_LPW_1HZ);
    if (sts != ESP_OK) {
        return;
    }
    // 各軸の有効化
    ESP_LOGI(TAG, "LIS3DH enable axis");
    sts = sts_lis3dh_set_enable_axis(s_address, true, true, true);
    if (sts != ESP_OK) {
        return;
    }
    // 計測データ更新設定
    ESP_LOGI(TAG, "LIS3DH upd settings");
    sts = sts_lis3dh_set_upd_settings(s_address, false, false);
    if (sts != ESP_OK) {
        return;
    }
    // レンジ設定
    ESP_LOGI(TAG, "LIS3DH set range");
    sts = sts_lis3dh_set_range(s_address, DRV_LIS3DH_RANGE_2G, true);
    if (sts != ESP_OK) {
        return;
    }
    // FIFOモード
    ESP_LOGI(TAG, "LIS3DH set fifo mode");
//    sts = sts_lis3dh_set_fifo_mode(s_address, DRV_LIS3DH_MODE_STREAM);
    sts = sts_lis3dh_set_fifo_mode(s_address, DRV_LIS3DH_MODE_BYPASS);
    if (sts != ESP_OK) {
        return;
    }

    //==========================================================================
    // who am i
    //==========================================================================
    ESP_LOGI(TAG, "LIS3DH who am i");
    sts = sts_lis3dh_who_am_i(s_address);
    if (sts != ESP_OK) {
        return;
    }

    //==========================================================================
    // 加速度の読み込み
    //==========================================================================
    int i_cnt;
    for (i_cnt = 0; i_cnt < 100000; i_cnt++) {
        // FIFOカウント読み込み
        uint8_t u8_fifo_cnt;
        esp_err_t sts = sts_lis3dh_fifo_cnt(s_address, &u8_fifo_cnt);
        if (sts != ESP_OK) {
            return;
        }
        // 加速度読み込み
        ts_lis3dh_axes_data_t s_axes_data;
        sts = sts_lis3dh_acceleration(s_address, &s_axes_data);
        if (sts != ESP_OK) {
            return;
        }
        // ログ出力
        ESP_LOGI(TAG, "LIS3DH G:%d:%d:%d Cnt:%d", (int)s_axes_data.i16_data_x, (int)s_axes_data.i16_data_y, (int)s_axes_data.i16_data_z, u8_fifo_cnt);
//        v_lis3dh_read();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mpu6050
 *
 * DESCRIPTION:MPU6050のテストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *   void*          arg             R   パラメータ
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mpu6050(void* args) {
    // 正常系テストケース
    v_task_chk_mpu6050_00();
}

/*******************************************************************************
 *
 * NAME: v_task_chk_mpu6050_00
 *
 * DESCRIPTION:MPU6050の正常系テストケース関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_task_chk_mpu6050_00() {
    // アドレス
    ts_i2c_address_t s_address;
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = I2C_ADDR_MPU_6050_L;
    esp_err_t sts;
    //==========================================================================
    // 初期処理
    //==========================================================================
    sts = sts_mpu_6050_init(s_address, DRV_MPU_6050_ACCEL_RANGE_2G, DRV_MPU_6050_GYRO_RANGE_250);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_init:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_init:ERRROR");
    }
    //==========================================================================
    // 設定：ジャイロサンプリングレート
    //==========================================================================
    // サンプルレート = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Gyroscope Output Rate ＝8KHz(DLPFが有効の場合は1KHz)
    sts = sts_mpu_6050_set_smplrt_div(s_address, 0x00);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_smplrt_div:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_smplrt_div:ERRROR");
    }
    //==========================================================================
    // 設定：ローパスフィルター
    //==========================================================================
    // 加速度:260Hz以上をカット、ジャイロ:258Hz以上をカット
    sts = sts_mpu_6050_set_dlpf_cfg(s_address, DRV_MPU_6050_LPF_260_256);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_dlpf_cfg:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_dlpf_cfg:ERRROR");
    }
    //==========================================================================
    // 設定：ハイパスフィルター
    //==========================================================================
    sts = sts_mpu_6050_set_accel_hpf(s_address, DRV_MPU_6050_ACCEL_HPF_0P63HZ);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_accel_hpf:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_accel_hpf:ERRROR");
    }
    //==========================================================================
    // 設定：ジャイロレンジ
    //==========================================================================
    sts = sts_mpu_6050_set_gyro_range(s_address, DRV_MPU_6050_GYRO_RANGE_250);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_gyro_range:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_gyro_range:ERRROR");
    }
    //==========================================================================
    // 設定：加速度レンジ
    //==========================================================================
    sts = sts_mpu_6050_set_accel_range(s_address, DRV_MPU_6050_ACCEL_RANGE_2G);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_accel_range:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_accel_range:ERRROR");
    }
    //==========================================================================
    // 設定：FIFO有効無効設定
    //==========================================================================
    // FIFO無効化
    sts = sts_mpu_6050_set_fifo_enable(s_address, false, false, false, false, false);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:ERRROR");
    }
    //==========================================================================
    // クロック設定 ※内部オシレータ8MHz
    //==========================================================================
    sts = sts_mpu_6050_set_clock(s_address, DRV_MPU_6050_CLK_INTERNAL);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_clock:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_clock:ERRROR");
    }
    //==========================================================================
    // who am i
    //==========================================================================
    sts = sts_mpu_6050_who_am_i(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_who_am_i:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_who_am_i:ERRROR");
    }
    //==========================================================================
    // zeroing accel
    //==========================================================================
    sts = sts_mpu_6050_zeroing_accel(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_zeroing_accel:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_zeroing_accel:ERRROR");
    }
    //==========================================================================
    // zeroing gyro
    //==========================================================================
    sts = sts_mpu_6050_zeroing_gyro(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_zeroing_accel:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_zeroing_accel:ERRROR");
    }
    //==========================================================================
    // G
    //==========================================================================
    int i_cnt;
    for (i_cnt = 0; i_cnt < 10; i_cnt++) {
        v_mpu6050_read();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    //==========================================================================
    // zeroing clear
    //==========================================================================
    v_mpu_6050_zeroing_clear();
    ESP_LOGI(TAG, "MPU6050 v_mpu_6050_zeroing_clear:OK");
    //==========================================================================
    // G
    //==========================================================================
    for (i_cnt = 0; i_cnt < 10; i_cnt++) {
        v_mpu6050_read();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    //==========================================================================
    // FIFO有効無効設定
    //==========================================================================
    sts = sts_mpu_6050_set_fifo_enable(s_address, true, true, true, true, true);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:ERRROR");
    }
    //==========================================================================
    // FIFO Reset
    //==========================================================================
    sts = sts_mpu_6050_fifo_reset(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_fifo_reset:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_fifo_reset:ERRROR");
    }
    //==========================================================================
    // Read FIFO data count
    //==========================================================================
    // 5msecウェイト
    i64_dtm_delay_msec(5);
    int16_t i16_cnt;
    sts = sts_mpu_6050_fifo_cnt(s_address, &i16_cnt);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_fifo_cnt:OK cnt:%d", i16_cnt);
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_fifo_cnt:ERRROR");
    }
    //==========================================================================
    // Read FIFO data
    //==========================================================================
    v_mpu6050_fifo_read();

    //==========================================================================
    // FIFO有効無効設定
    //==========================================================================
    sts = sts_mpu_6050_set_fifo_enable(s_address, false, false, false, false, false);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_fifo_enable:ERRROR");
    }
    //==========================================================================
    // FIFO Reset
    //==========================================================================
    sts = sts_mpu_6050_fifo_reset(s_address);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_fifo_reset:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_fifo_reset:ERRROR");
    }
    //==========================================================================
    // Read FIFO data count
    //==========================================================================
    // ウェイト
    i64_dtm_delay_msec(10);
    sts = sts_mpu_6050_fifo_cnt(s_address, &i16_cnt);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_fifo_cnt:OK cnt:%d", i16_cnt);
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_fifo_cnt:ERRROR");
    }
    //==========================================================================
    // 加速度セルフテスト
    //==========================================================================
    sts = sts_mpu_6050_set_accel_self_test(s_address, true, true, true);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_accel_self_test:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_accel_self_test:ERRROR");
    }
    //==========================================================================
    // ジャイロセルフテスト設定
    //==========================================================================
    sts = sts_mpu_6050_set_gyro_self_test(s_address, true, true, true);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_gyro_self_test:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_gyro_self_test:ERRROR");
    }
    //==========================================================================
    // G
    //==========================================================================
    for (i_cnt = 0; i_cnt < 10; i_cnt++) {
        v_mpu6050_read();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    //==========================================================================
    // スリープサイクル設定
    //==========================================================================
    sts = sts_mpu_6050_set_sleep_cycle(s_address, DRV_MPU_6050_SLEEP_CYCLE_1000);
    if (sts == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 sts_mpu_6050_set_sleep_cycle:OK");
    } else {
        ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_set_sleep_cycle:ERRROR");
    }

}

/*******************************************************************************
 *
 * NAME: v_mpu6050_read
 *
 * DESCRIPTION:MPU6050の正常系テストケースの読み込み関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mpu6050_read() {
    // アドレス
    ts_i2c_address_t s_address;
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = I2C_ADDR_MPU_6050_L;
    //==========================================================================
    // 加速度読み込み
    //==========================================================================
    ts_mpu_6050_axes_data_t s_accel;
    esp_err_t sts = sts_mpu_6050_read_accel(s_address, &s_accel);
    if (sts != ESP_OK) {
        return;
    }
    /** ３軸の加速度の合成値 */
    int16_t i16_composite_value = i16_mpu_6050_composite_value(&s_accel, false);

    //==========================================================================
    // 温度読み込み
    //==========================================================================
    // 340 LSB/degrees and Offset 35 degrees and Difference -521
    // ((temperature + (35 * 340) - 521) / 340.0)
    float f_temp;
    sts = sts_mpu_6050_read_celsius(s_address, &f_temp);
    if (sts != ESP_OK) {
        return;
    }
    //==========================================================================
    // ジャイロ読み込み
    //==========================================================================
    // ジャイロ
    ts_mpu_6050_axes_data_t s_gyro;
    sts = sts_mpu_6050_read_gyro(s_address, &s_gyro);
    if (sts != ESP_OK) {
        return;
    }

    //==========================================================================
    // ログ出力
    //==========================================================================
//    ESP_LOGI(TAG, "MPU6050 A:%+06d:%+06d:%+06d G:%+06d:%+06d:%+06d T:%+d", i16_axis[0], i16_axis[1], i16_axis[2], i16_axis[4], i16_axis[5], i16_axis[6], i16_axis[3]);
    ESP_LOGI(TAG, "MPU6050 A:%+d:%+d:%+d G:%+d:%+d:%+d comp:%d T:%+f",
            s_accel.i16_data_x, s_accel.i16_data_y, s_accel.i16_data_z,
            s_gyro.i16_data_x, s_gyro.i16_data_y, s_gyro.i16_data_z,
            i16_composite_value, f_temp);
}

/*******************************************************************************
 *
 * NAME: v_mpu6050_read
 *
 * DESCRIPTION:MPU6050の正常系テストケースのFIFO読み込み処理関数
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mpu6050_fifo_read() {
    // アドレス
    ts_i2c_address_t s_address;
    s_address.e_port_no = I2C_NUM_0;
    s_address.u16_address = I2C_ADDR_MPU_6050_L;
    //==========================================================================
    // Read FIFO data
    //==========================================================================
    int16_t i16_data[7];
    int i_idx;
    for (i_idx = 0; i_idx < 7; i_idx++) {
        esp_err_t sts = sts_mpu_6050_fifo_data(s_address, &i16_data[i_idx]);
        if (sts != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 sts_mpu_6050_fifo_data:ERRROR");
            return;
        }
    }

    //==========================================================================
    // ログ出力
    //==========================================================================
    ESP_LOGI(TAG, "MPU6050 FIFO accel:%+d:%+d:%+d", i16_data[0], i16_data[1], i16_data[2]);
    ESP_LOGI(TAG, "MPU6050 FIFO temp :%f", f_mpu_6050_celsius(i16_data[3]));
    ESP_LOGI(TAG, "MPU6050 FIFO gyro :%+d:%+d:%+d", i16_data[4], i16_data[5], i16_data[6]);
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
