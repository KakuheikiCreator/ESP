/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common File Utility functions header file
 *
 * CREATED:2019/09/10 02:45:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:ファイルユーティリティ関係の関数群
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
#ifndef __NTFW_IO_FILE_UTIL_H__
#define __NTFW_IO_FILE_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdbool.h>
#include <esp_err.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <cJSON.h>
#include <driver/sdmmc_host.h>
#include <hal/spi_types.h>


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// SPIモードで接続も可能
//#define USE_SDMMC_HS2_MODE
//#define USE_SDMMC_HSPI_MODE
#define USE_SDMMC_VSPI_MODE

// HSPIモード
#ifdef USE_SDMMC_HSPI_MODE
  #ifndef PIN_NUM_SDMMC_MISO
  #define PIN_NUM_SDMMC_MISO GPIO_NUM_12
  #endif
  #ifndef PIN_NUM_SDMMC_MOSI
  #define PIN_NUM_SDMMC_MOSI GPIO_NUM_13
  #endif
  #ifndef PIN_NUM_SDMMC_CLK
  #define PIN_NUM_SDMMC_CLK  GPIO_NUM_14
  #endif
  #ifndef PIN_NUM_SDMMC_CS
  #define PIN_NUM_SDMMC_CS   GPIO_NUM_15
  #endif
#endif

// VSPIモード
#ifdef USE_SDMMC_VSPI_MODE
  #ifndef PIN_NUM_SDMMC_MISO
  #define PIN_NUM_SDMMC_MISO GPIO_NUM_19
  #endif
  #ifndef PIN_NUM_SDMMC_MOSI
  #define PIN_NUM_SDMMC_MOSI GPIO_NUM_23
  #endif
  #ifndef PIN_NUM_SDMMC_CLK
  #define PIN_NUM_SDMMC_CLK  GPIO_NUM_18
  #endif
  #ifndef PIN_NUM_SDMMC_CS
  #define PIN_NUM_SDMMC_CS   GPIO_NUM_5
  #endif
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 構造体：SDMMC情報 */
typedef struct {
    char c_speed[8];                                  // 接続速度
    char c_card_name[16];                             // カード名前
    char c_card_type[16];                             // カードタイプ
    char c_card_size[16];                             // カードサイズ
} ts_sdmmc_info_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** SDMMC情報：必要に応じてメモリ割り当て */
extern ts_sdmmc_info_t* ps_sdmmc_info;

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
//==============================================================================
// チェック関数
//==============================================================================
/** ファイルパスチェック */
extern bool b_futil_valid_path(const char* pc_path);

//==============================================================================
// 変換関数
//==============================================================================
/** 変換関数：ファイル名・ディレクトリ名→短縮ファイル名（変則的な8.3形式） */
extern bool b_futil_sfn(char* pc_snf, const char* pc_name, int i_num);
/** 変換関数：パス→短縮ファイル名形式（変則的な8.3形式） */
extern bool b_futil_sfn_path(char* pc_edit, const char* pc_path, int i_num);

//==============================================================================
// ファイル情報の取得関数
//==============================================================================
/** ファイルサイズの取得 */
extern long l_futil_file_size(const char* pc_path);
/** ファイル・ディレクトリの存在判定 */
extern bool b_futil_exist(const char* pc_path);
/** ファイルの存在判定 */
extern bool b_futil_file_exist(const char* pc_path);
/** ディレクトリの存在判定 */
extern bool b_futil_directory_exist(const char* pc_path);

//==============================================================================
// ファイル・ディレクトリの操作関数
//==============================================================================
/** ファイルのオープン（ディレクトリ自動作成） */
extern FILE* ps_futil_fopen(const char* pc_filename, const char* pc_mode);
/** ファイルのコピー */
extern bool b_futil_copy_file(const char* pc_src, const char* pc_dest);
/** ファイルの移動 */
extern bool b_futil_move_file(const char* pc_from, const char* pc_to);
/** ディレクトリの作成 */
extern bool b_futil_make_directory(const char* pc_path);
/** ディレクトリのコピー */
extern bool b_futil_copy_directory(const char* pc_src, const char* pc_dest);
/** ディレクトリの移動 */
extern bool b_futil_move_directory(const char* pc_from, const char* pc_to);
/** ディレクトリの削除 */
extern bool b_futil_remove_directory(const char* pc_path);
/** ディレクトリ内容のコピー */
extern bool b_futil_copy_member(const char* pc_src, const char* pc_dest);

//==============================================================================
// JSON関連関数
//==============================================================================
/** JSONファイルの読み込み処理 */
extern cJSON* ps_futil_cjson_parse_file(const char* pc_path, long l_max_size);
/** JSONファイルの書き込み処理 */
extern esp_err_t sts_futil_cjson_write_file(const char* pc_path, cJSON* ps_cjson);

//==============================================================================
// SDMMC関連関数
//==============================================================================
/** SDMMCカードのマウント（HS接続 4bit mode） */
extern sdmmc_card_t* ps_futil_sdmmc_hs_mount(char* pc_path,
                                              gpio_num_t e_gpio_num_cs,
                                              gpio_num_t e_gpio_num_cd,
                                              gpio_num_t e_gpio_num_wp,
                                              esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
/** SDMMCカードのマウント（HSPI接続） */
extern sdmmc_card_t* ps_futil_sdmmc_hspi_mount(char* pc_path,
                                                gpio_num_t e_gpio_num_cs,
                                                gpio_num_t e_gpio_num_cd,
                                                gpio_num_t e_gpio_num_wp,
                                                esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
/** SDMMCカードのマウント（VSPI接続） */
extern sdmmc_card_t* ps_futil_sdmmc_vspi_mount(char* pc_path,
                                                gpio_num_t e_gpio_num_cs,
                                                gpio_num_t e_gpio_num_cd,
                                                gpio_num_t e_gpio_num_wp,
                                                esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
/** SDMMCカードのアンマウント */
extern esp_err_t sts_futil_sdmmc_unmount();
/** SDMMCカードのアンマウント(card指定) */
extern esp_err_t sts_futil_sdmmc_unmount_card(sdmmc_card_t* ps_card);
/** SDMMCカードの情報編集 */
extern esp_err_t sts_futil_sdmmc_edit_info(ts_sdmmc_info_t* ps_info, sdmmc_card_t* ps_card);

#ifdef __cplusplus
}
#endif

#endif /* __NTFW_IO_FILE_UTIL_H__ */
