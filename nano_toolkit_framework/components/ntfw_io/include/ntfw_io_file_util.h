/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common File Utility functions header file
 *
 * CREATED:2019/09/10 02:45:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:ファイルユーティリティ関係の関数群
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
#ifndef __NTFW_IO_FILE_UTIL_H__
#define __NTFW_IO_FILE_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdbool.h>
#include <esp_system.h>
#include <esp_err.h>
#include <esp_vfs_fat.h>
#include <driver/gpio.h>
#include <driver/sdmmc_host.h>
#include <hal/spi_types.h>
#include <sdmmc_cmd.h>
#include <cJSON.h>

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 構造体：SDMMC情報 */
typedef struct {
    char c_speed[8];            // 接続速度
    char c_card_name[16];       // カード名前
    char c_card_type[16];       // カードタイプ
    char c_card_size[16];       // カードサイズ
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
// SDカード関連関数
//==============================================================================
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
/** SDカードのマウント（HS接続 4bit mode） */
extern sdmmc_card_t* ps_futil_sdmmc_mount(char* pc_path,
                                          sdmmc_slot_config_t* ps_slot_cfg,
                                          esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
#endif
/** SDカードのマウント（SPI接続） */
extern sdmmc_card_t* ps_futil_sdspi_mount(char* pc_path,
                                          sdspi_device_config_t* ps_device_cfg,
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
