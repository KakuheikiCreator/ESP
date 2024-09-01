/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common File Utility functions source file
 *
 * CREATED:2019/09/15 00:11:00
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

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_io_file_util.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <freertos/task.h>
#include <driver/sdspi_host.h>
#include "ntfw_com_mem_alloc.h"
#include "ntfw_com_value_util.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Block Time */
#ifndef COM_FUTIL_BLOCK_TIME
    // デフォルト値は無制限にウェイト
    #define COM_FUTIL_BLOCK_TIME    (portMAX_DELAY)
#endif

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 構造体：SDMMCマウント情報 */
typedef struct {
    char* pc_mnt_path;          // マウントパス
    sdmmc_card_t* ps_card;      // SDMMCカード情報
} ts_sdmmc_mount_info_t;

/** mutex init function */
typedef void (*tf_initialize)();

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス */
static SemaphoreHandle_t s_mutex = NULL;
/** SDMMCマウント情報 */
static ts_sdmmc_mount_info_t s_sdmmc_mnt_info_list[] = {
    {
        .pc_mnt_path = NULL,    // マウントパス
        .ps_card     = NULL     // SDMMCカード情報
    },
    {
        .pc_mnt_path = NULL,    // マウントパス
        .ps_card     = NULL     // SDMMCカード情報
    },
    {
        .pc_mnt_path = NULL,    // マウントパス
        .ps_card     = NULL     // SDMMCカード情報
    }
};

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** Mutex Initial processing */
static void v_mutex_init();
/** Dummy Mutex initial processing */
static void v_mutex_init_dummy();
/** Initialize function */
static tf_initialize pf_mutex_init = v_mutex_init;
/** ファイルコピー */
static bool b_file_copy(const char* pc_src, const char* pc_dest);
/** ディレクトリメンバーのコピー */
static bool b_member_copy(const char* pc_src, const char* pc_dest);
/** ディレクトリ作成 */
static bool b_make_directory(const char* pc_path);
/** テンポラリファイルのパス（動的確保）の生成 */
static char* pc_temp_file_path(const char* pc_path);
/** SDMMCのHSマウント処理 */
static sdmmc_card_t* ps_sdmmc_hs_mount(char* pc_path,
                                        gpio_num_t e_gpio_num_cs,
                                        gpio_num_t e_gpio_num_cd,
                                        gpio_num_t e_gpio_num_wp,
                                        esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
/** SDMMCのSPIマウント処理 */
static sdmmc_card_t* ps_sdmmc_spi_mount(char* pc_path,
                                         spi_host_device_t e_slot,
                                         gpio_num_t e_gpio_num_cs,
                                         gpio_num_t e_gpio_num_cd,
                                         gpio_num_t e_gpio_num_wp,
                                         esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg);
/** SDMMCアンマウント処理(card指定) */
static esp_err_t sts_sdmmc_unmount(ts_sdmmc_mount_info_t* ps_mount);
/** SDMMCアンマウント処理(card指定) */
static esp_err_t sts_sdmmc_edit_info(ts_sdmmc_info_t* ps_info, sdmmc_card_t* ps_card);


/** slotに対応したSDMMCマウント情報の取得 */
static ts_sdmmc_mount_info_t* ps_sdmmc_spi_mount_info(spi_host_device_t e_slot);
/** cardに対応したSDMMCマウント情報の取得 */
static ts_sdmmc_mount_info_t* ps_sdmmc_mount_info_card(sdmmc_card_t* ps_card);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
//==============================================================================
// チェック関数
//==============================================================================
/*******************************************************************************
 *
 * NAME: b_vutil_valid_path
 *
 * DESCRIPTION:ファイルパスチェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルパス
 *
 * RETURNS:
 *   bool true:有効なファイルパス文字列
 *
 ******************************************************************************/
bool b_futil_valid_path(const char* pc_path) {
    // NULLチェック
    if (pc_path == NULL) {
        return false;
    }
    // パスチェック（先端"/"である事）
    if (*pc_path != '/') {
        return false;
    }
    // 文字チェック
    pc_path++;
    char c_bef = '/';
    while (*pc_path != '\0') {
        // スプリット文字間隔チェック
        if (*pc_path == '/' && c_bef == '/') {
            return false;
        }
        // 禁則文字チェック
        if (i_vutil_index_of("\\:*?\"<>|", *pc_path) >= 0) {
            return false;
        }
        c_bef = *pc_path;
        pc_path++;
    }
    // 終端文字判定
    return (c_bef != '/');
}

//==============================================================================
// 変換関数
//==============================================================================

/*******************************************************************************
 *
 * NAME: b_futil_sfn_path
 *
 * DESCRIPTION:変換関数：ファイル名・ディレクトリ名→短縮ファイル名（変則的）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_snf          W   短縮ファイル名・ディレクトリ名
 *   char*          pc_name         R   元ファイル名・ディレクトリ名
 *   int            i_num           R   連番
 *
 * RETURNS:
 *   bool：ファイル名の短縮ファイル名への変換の有無
 *
 * NOTES:
 *   None.
 ******************************************************************************/
bool b_futil_sfn(char* pc_snf, const char* pc_name, int i_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pc_name == NULL || i_num <= 0) {
        // 変換有無を返却
        return false;
    }

    //==========================================================================
    // 対象外文字の置換
    //==========================================================================
    // 編集領域
    char c_edit[strlen(pc_name) + 1];
    // 短縮ファイル名への変換フラグ
    bool b_snf = false;
    // 直前のピリオドポインタ
    char c_ch;
    char* pc_period = &c_ch;
    int i_idx;
    for (i_idx = 0; (c_ch = pc_name[i_idx]) != '\0'; i_idx++) {
        c_edit[i_idx] = c_ch;
        // 最終ピリオド以外を"+"に置き換え
        if (c_ch == '.') {
            *pc_period = '+';
            pc_period = &c_edit[i_idx];
            continue;
        }
        // 禁則文字を"="に置き換え
        if (i_vutil_index_of("=+;,[] ", c_ch) >= 0) {
            // 禁則文字の置き換え
            c_edit[i_idx] = '_';
            b_snf = true;
        }
    }
    c_edit[i_idx] = '\0';

    //==========================================================================
    // SNF変換判定
    //==========================================================================
    int i_max  = 8;
    int i_cnt  = 0;
    int i_edit = 0;
    for (i_idx = 0; (c_ch = c_edit[i_idx]) != '\0'; i_idx++) {
        // 最終ピリオド以外の判定
        if (c_ch == '+') {
            b_snf = true;
            continue;
        }
        if (c_ch == '.') {
            c_edit[i_edit++] = c_ch;
            i_max = 3;
            i_cnt = 0;
            continue;
        }
        if (i_cnt >= i_max) {
            b_snf = true;
            continue;
        }
        c_edit[i_edit++] = c_ch;
        i_cnt++;
    }
    c_edit[i_edit] = '\0';

    //==========================================================================
    // SNF変換
    //==========================================================================
    // 数字文字列
    const char pc_num[] = "0123456789";
    bool b_wk_snf = b_snf;
    i_edit = 0;
    for (i_idx = 0; (c_ch = c_edit[i_idx]) != '\0'; i_idx++) {
        if (c_ch == '.' && b_wk_snf) {
            if (i_edit > 6) {
                i_edit = 6;
            }
            pc_snf[i_edit++] = '~';
            pc_snf[i_edit++] = pc_num[i_num];
            b_wk_snf = false;
        }
        // 文字を編集
        pc_snf[i_edit++] = c_ch;
    }
    // 終端に連番挿入
    if (b_wk_snf == true) {
        if (i_edit > 6) {
            i_edit = 6;
        }
        pc_snf[i_edit++] = '~';
        pc_snf[i_edit++] = pc_num[i_num];
    }
    // 終端文字を挿入
    pc_snf[i_edit] = '\0';
    // 変換有無を返却
    return b_snf;
}

/*******************************************************************************
 *
 * NAME: b_futil_sfn_path
 *
 * DESCRIPTION:変換関数：ファイルパス・ディレクトリパス→短縮ファイル名
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_out          W   変換結果ファイルパス
 *   char*          pc_path         R   変換対象ファイルパス
 *   int            i_num           R   連番
 *
 * RETURNS:
 *   bool：ファイル名の短縮ファイル名への変換の有無
 *
 * NOTES:
 *   None.
 ******************************************************************************/
bool b_futil_sfn_path(char* pc_out, const char* pc_path, int i_num) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (b_futil_valid_path(pc_path) == false || i_num <= 0) {
        // 変換有無を返却
        return false;
    }

    //==========================================================================
    // 編集用にコピー
    //==========================================================================
    char c_edit[strlen(pc_path)];
    strcpy(c_edit, pc_path);

    //==========================================================================
    // SNF変換処理
    //==========================================================================
    // 結果ステータス
    bool b_result = false;
    bool b_conv;
    char c_ch;
    int i_begin = 0;
    int i_out = 0;
    int i_idx;
    for (i_idx = 0; (c_ch = c_edit[i_idx]) != '\0'; i_idx++) {
        // セパレータ判定
        if (c_ch != '/') {
            continue;
        }
        if (i_begin < i_idx) {
            c_edit[i_idx] = '\0';
            b_conv = b_futil_sfn(&pc_out[i_out], &c_edit[i_begin], i_num);
            b_result = (b_result || b_conv);
            while (pc_out[++i_out] != '\0');
        }
        pc_out[i_out++] = c_ch;
        i_begin = i_idx + 1;
    }
    if (i_begin >= i_idx) {
        pc_out[i_out] = '\0';
    } else {
        b_conv = b_futil_sfn(&pc_out[i_out], &c_edit[i_begin], i_num);
        b_result = (b_result || b_conv);
    }
    return b_result;
}

/*******************************************************************************
 *
 * NAME: l_futil_file_size
 *
 * DESCRIPTION:ファイルサイズの取得
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルパス
 *
 * RETURNS:
 *   long:ファイルサイズ、ファイルが存在しない場合には-1
 *
 * NOTES:
 * None.
 ******************************************************************************/
long l_futil_file_size(const char* pc_path) {
    // パスチェック
    if (!b_futil_valid_path(pc_path)) {
        return -1;
    }
    // ファイルステータスを取得
    struct stat st;
    if (stat(pc_path, &st) != 0) {
        // ファイルステータスの取得が出来ない場合（ファイルが存在しない場合）
        return -1l;
    }
    return st.st_size;
}

/*******************************************************************************
 *
 * NAME: b_futil_exist
 *
 * DESCRIPTION:ファイル・ディレクトリの存在判定
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルパス
 *
 * RETURNS:
 *   true:ファイル・ディレクトリが存在する
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_exist(const char* pc_path) {
    // パスチェック
    if (b_futil_valid_path(pc_path) == false) {
        return false;
    }
    // 存在判定
    struct stat statBuf;
    return (stat(pc_path, &statBuf) == 0);
}

/*******************************************************************************
 *
 * NAME: b_futil_file_exist
 *
 * DESCRIPTION:ファイルの存在判定
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルパス
 *
 * RETURNS:
 *   true:ファイルが存在する
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_file_exist(const char* pc_path) {
    // パスチェック
    if (b_futil_valid_path(pc_path) == false) {
        return false;
    }
    // 存在判定
    struct stat st_stat;
    if (stat(pc_path, &st_stat) != 0) {
        return false;
    }
    return ((st_stat.st_mode & S_IFMT) == S_IFREG);
}

/*******************************************************************************
 *
 * NAME: b_futil_directory_exist
 *
 * DESCRIPTION:ディレクトリの存在判定
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ディレクトリパス
 *
 * RETURNS:
 *   true:ディレクトリが存在する
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_directory_exist(const char* pc_path) {
    // パスチェック
    if (b_futil_valid_path(pc_path) == false) {
        return false;
    }
    // 存在判定
    struct stat st_stat;
    if (stat(pc_path, &st_stat) != 0) {
        return false;
    }
    return ((st_stat.st_mode & S_IFMT) == S_IFDIR);
}

/*******************************************************************************
 *
 * NAME: ps_futil_fopen
 *
 * DESCRIPTION:ファイルのオープン（ディレクトリ自動作成）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ファイルパス
 *
 * RETURNS:
 *   true:ファイルが存在する
 *
 * NOTES:
 * None.
 ******************************************************************************/
FILE* ps_futil_fopen(const char* pc_filename, const char* pc_mode) {
    // 入力チェック
    if (b_futil_valid_path(pc_filename) == false) {
        return NULL;
    }
    // ディレクトリ有無の判定
    int i_last_idx = i_vutil_last_index_of(pc_filename, '/');
    if (i_last_idx > 0) {
        // ディレクトリパスの生成
        char pc_path[i_last_idx + 1];
        i_vutil_str_left(pc_path, pc_filename, i_last_idx);
        // ディレクトリの作成
        if (!b_futil_make_directory(pc_path)) {
            return NULL;
        }
    }
    // ファイルのオープン
    return fopen(pc_filename, pc_mode);
}

/*******************************************************************************
 *
 * NAME: b_futil_copy_file
 *
 * DESCRIPTION:ファイルのコピー
 *   コピー先にファイルが無い場合のみコピーする
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_src          R   コピー元ファイルパス
 *   char*          pc_dest         R   コピー先ファイルパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
 bool b_futil_copy_file(const char* pc_src, const char* pc_dest) {
    // コピー先ファイルの存在チェック
    if (b_futil_file_exist(pc_dest)) {
        return false;
    }
    // ファイルコピー
    return b_file_copy(pc_src, pc_dest);
}

/*******************************************************************************
 *
 * NAME: b_futil_move_file
 *
 * DESCRIPTION:ファイルの移動
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_from         R   移動元ファイルパス
 *   char*          pc_to           R   移動先ファイルパス
 *
 * RETURNS:
 *   true:移動成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_move_file(const char* pc_from, const char* pc_to) {
    // ファイルコピーする
    if (!b_futil_copy_file(pc_from, pc_to)) {
        // ファイルコピー失敗
        return false;
    }
    // コピー元ファイルを削除
    return (unlink(pc_from) == 0);
}

/*******************************************************************************
 *
 * NAME: b_futil_make_directory
 *
 * DESCRIPTION:ディレクトリの作成
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ディレクトリパス
 *
 * RETURNS:
 *   true:作成成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_make_directory(const char* pc_path) {
    // パスチェック
    if (!b_futil_valid_path(pc_path)) {
        return false;
    }
    // ディレクトリ作成
    return b_make_directory(pc_path);
}

/*******************************************************************************
 *
 * NAME: b_futil_copy_directory
 *
 * DESCRIPTION:ディレクトリのコピー
 *   コピー先のディレクトリが無い場合のみコピーする
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_src          R   コピー元ディレクトリパス
 *   char*          pc_dest         R   コピー先ディレクトリパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_copy_directory(const char* pc_src, const char* pc_dest) {
    // コピー元のディレクトリ存在チェック
    if (!b_futil_directory_exist(pc_src)) {
        return false;
    }
    // コピー先のディレクトリパスチェック
    if (!b_futil_valid_path(pc_dest)) {
        return false;
    }
    // コピー先のディレクトリパスの生成
    int i_last_idx = i_vutil_last_index_of(pc_src, '/');
    if (i_last_idx < 2) {
        return false;
    }
    char pc_wk_dest[strlen(pc_dest) + strlen(&pc_src[i_last_idx]) + 2];
    sprintf(pc_wk_dest, "%s%s", pc_dest, &pc_src[i_last_idx]);
    // コピー先ディレクトリの作成
    if (!b_make_directory(pc_wk_dest)) {
        return false;
    }
    // ディレクトリメンバーのコピー
    return b_member_copy(pc_src, pc_wk_dest);
}

/*******************************************************************************
 *
 * NAME: b_futil_copy_directory
 *
 * DESCRIPTION:ディレクトリの移動
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_from         R   移動元ディレクトリパス
 *   char*          pc_to           R   移動先ディレクトリパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_move_directory(const char* pc_from, const char* pc_to) {
    // ディレクトリをコピーする
    if (!b_futil_copy_directory(pc_from, pc_to)) {
        // ディレクトリコピー失敗
        return false;
    }
    // コピー元ディレクトリを削除
    return b_futil_remove_directory(pc_from);
}


/*******************************************************************************
 *
 * NAME: b_futil_remove_directory
 *
 * DESCRIPTION:ディレクトリの削除
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ディレクトリパス
 *
 * RETURNS:
 *   true:削除成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_remove_directory(const char* pc_path) {
    // パスチェック
    if (b_futil_valid_path(pc_path) == false) {
        return false;
    }
    // ディレクトリストリームのオープン
    DIR* pts_dir = opendir(pc_path);
    if (pts_dir == NULL) {
        return false;
    }
    // パス長
    int i_path_len = strlen(pc_path) + 2;
    char pc_tmp_path[i_path_len];
    strcpy(pc_tmp_path, pc_path);
    strcat(pc_tmp_path, "/");
    // ディレクトリ情報の参照
    struct dirent *pst_dir_info = readdir(pts_dir);
    // ディレクトリストリームのエントリ毎の処理
    while (pst_dir_info != NULL) {
        char pc_chk_path[i_path_len + strlen(pst_dir_info->d_name)];
        strcpy(pc_chk_path, pc_tmp_path);
        strcat(pc_chk_path, pst_dir_info->d_name);
        // エントリタイプ
        switch (pst_dir_info->d_type) {
        case DT_REG:
            // ファイルの削除
            unlink(pc_chk_path);
            break;
        case DT_DIR:
            // ディレクトリの削除
            // ディレクトリストリームのクローズ
            closedir(pts_dir);
            // サブディレクトリの削除
            b_futil_remove_directory(pc_chk_path);
            // 自ディレクトリの再オープン
            pts_dir = opendir(pc_path);
            break;
        }
        // エントリ情報の読み込み
        pst_dir_info = readdir(pts_dir);
    }
    // ディレクトリストリームのクローズ
    closedir(pts_dir);
    // 自ディレクトリの削除処理
    return (rmdir(pc_path) == 0);
}

/*******************************************************************************
 *
 * NAME: b_futil_copy_member
 *
 * DESCRIPTION:ディレクトリ内容のコピー
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_from         R   コピー元ファイルパス
 *   char*          pc_to           R   コピー先ファイルパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_futil_copy_member(const char* pc_src, const char* pc_dest) {
    // コピー元のディレクトリ存在チェック
    if (!b_futil_directory_exist(pc_src)) {
        return false;
    }
    // コピー先のディレクトリパスチェック
    if (!b_futil_directory_exist(pc_dest)) {
        return false;
    }
    // ディレクトリメンバーのコピー
    return b_member_copy(pc_src, pc_dest);
}

/*******************************************************************************
 *
 * NAME: ps_futil_cjson_parse_file
 *
 * DESCRIPTION:JSONファイルの読み込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   JSONファイルのパス
 *   long           l_max_size      R   最大ファイルサイズ
 *
 * RETURNS:
 *   cJSON*:JSONファイルを読み込んで生成したcJSON構造体
 *
 * NOTES:
 * None.
 ******************************************************************************/
cJSON* ps_futil_cjson_parse_file(const char* pc_path, long l_max_size) {
    // 入力チェック
    if (b_futil_valid_path(pc_path) == false || l_max_size <= 0) {
        return NULL;
    }
    // ファイル読み込み
    FILE* fp = fopen(pc_path, "r");
    if (fp == NULL) {
        return NULL;
    }
    // ファイルステータスを取得
    cJSON* s_cjson = NULL;
    do {
        struct stat st;
        if (stat(pc_path, &st) != 0) {
            // ファイルステータスの取得が出来ない場合（ファイルが存在しない場合）
            break;
        }
        // ファイルサイズ
        if (st.st_size > l_max_size) {
            break;
        }
        // JSONファイルの読み込み
        char c_json[st.st_size + 1];
        if (fread(c_json, 1, st.st_size, fp) == 0) {
            break;
        }
        c_json[st.st_size] = '\0';
        // cJSONを変換
        s_cjson = cJSON_Parse(c_json);
    } while (false);
    fclose(fp);
    return s_cjson;
}

/*******************************************************************************
 *
 * NAME: sts_futil_cjson_write_file
 *
 * DESCRIPTION:JSONファイルの書き込み処理
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   JSONファイルのパス
 *   cJSON*         ps_cjson        R   cJSONの要素構造体
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_futil_cjson_write_file(const char* pc_path, cJSON* ps_cjson) {
    // 入力チェック
    if (!b_futil_valid_path(pc_path) || ps_cjson == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // JSONテキスト変換
    char* pc_str = cJSON_Print(ps_cjson);
    if (pc_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // ファイルオープン（ディレクトリ自動作成）
    FILE* fp_wrt = ps_futil_fopen(pc_path, "w");
    if (fp_wrt == NULL) {
        // JSONテキスト解放
        free(pc_str);
        return ESP_ERR_INVALID_STATE;
    }
    // JSONファイルへ書き込み
    int i_size = strlen(pc_str);
    if (fwrite(pc_str, sizeof(char), i_size, fp_wrt) != i_size) {
        // ファイルをクローズする
        fclose(fp_wrt);
        // JSONテキスト解放
        free(pc_str);
        return ESP_ERR_INVALID_STATE;
    }
    // ファイルをクローズする
    fclose(fp_wrt);
    // JSONテキスト解放
    free(pc_str);
    // 結果返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_futil_sdmmc_hs_mount
 *
 * DESCRIPTION:SDMMCカードのマウント（HS接続 4bit mode）
 *
 * PARAMETERS:                          Name            RW  Usage
 * char*                                pc_path         R   マウントパス
 * gpio_num_t                           e_gpio_num_cs   R   チップセレクト
 * gpio_num_t                           e_gpio_num_cd   R   カード挿入ピン番号（未設定：SDMMC_SLOT_NO_CD）
 * gpio_num_t                           e_gpio_num_wp   R   ライトプロテクトピン番号（未設定：SDMMC_SLOT_NO_WP）
 * esp_vfs_fat_sdmmc_mount_config_t*    ps_mount_cfg    R   同時オープンファイル数
 *
 * RETURNS:
 *   sdmmc_card_t*: SDカード情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
sdmmc_card_t* ps_futil_sdmmc_hs_mount(char* pc_path,
                                      gpio_num_t e_gpio_num_cs,
                                      gpio_num_t e_gpio_num_cd,
                                      gpio_num_t e_gpio_num_wp,
                                      esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // SDMMCマウント
    //==========================================================================
    sdmmc_card_t* ps_card = ps_sdmmc_hs_mount(pc_path,
                                              e_gpio_num_cs,
                                              e_gpio_num_cd,
                                              e_gpio_num_wp,
                                              ps_mount_cfg);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ps_card;
}

/*******************************************************************************
 *
 * NAME: ps_futil_sdmmc_hspi_mount
 *
 * DESCRIPTION:SDMMCカードのマウント（HSPI接続）
 *
 * PARAMETERS:                          Name            RW  Usage
 * char*                                pc_path         R   マウントパス
 * gpio_num_t                           e_gpio_num_cs   R   チップセレクト
 * gpio_num_t                           e_gpio_num_cd   R   カード挿入ピン番号（未設定：SDMMC_SLOT_NO_CD）
 * gpio_num_t                           e_gpio_num_wp   R   ライトプロテクトピン番号（未設定：SDMMC_SLOT_NO_WP）
 * esp_vfs_fat_sdmmc_mount_config_t*    ps_mount_cfg    R   同時オープンファイル数
 *
 * RETURNS:
 *   sdmmc_card_t*: SDカード情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
sdmmc_card_t* ps_futil_sdmmc_hspi_mount(char* pc_path,
                                        gpio_num_t e_gpio_num_cs,
                                        gpio_num_t e_gpio_num_cd,
                                        gpio_num_t e_gpio_num_wp,
                                        esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // SDMMCマウント
    //==========================================================================
    sdmmc_card_t* ps_card = ps_sdmmc_spi_mount(pc_path,
                                               HSPI_HOST,
                                               e_gpio_num_cs,
                                               e_gpio_num_cd,
                                               e_gpio_num_wp,
                                               ps_mount_cfg);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ps_card;
}

/*******************************************************************************
 *
 * NAME: ps_futil_sdmmc_vspi_mount
 *
 * DESCRIPTION:SDMMCカードのマウント（VSPI接続）
 *
 * PARAMETERS:                          Name            RW  Usage
 * char*                                pc_path         R   マウントパス
 * gpio_num_t                           e_gpio_num_cs   R   チップセレクト
 * gpio_num_t                           e_gpio_num_cd   R   カード挿入ピン番号（未設定：SDMMC_SLOT_NO_CD）
 * gpio_num_t                           e_gpio_num_wp   R   ライトプロテクトピン番号（未設定：SDMMC_SLOT_NO_WP）
 * esp_vfs_fat_sdmmc_mount_config_t*    ps_mount_cfg    R   同時オープンファイル数
 *
 * RETURNS:
 *   sdmmc_card_t*: SDカード情報
 *
 * NOTES:
 * None.
 ******************************************************************************/
sdmmc_card_t* ps_futil_sdmmc_vspi_mount(char* pc_path,
                                        gpio_num_t e_gpio_num_cs,
                                        gpio_num_t e_gpio_num_cd,
                                        gpio_num_t e_gpio_num_wp,
                                        esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return NULL;
    }

    //==========================================================================
    // SDMMCマウント
    //==========================================================================
    sdmmc_card_t* ps_card = ps_sdmmc_spi_mount(pc_path,
                                               VSPI_HOST,
                                               e_gpio_num_cs,
                                               e_gpio_num_cd,
                                               e_gpio_num_wp,
                                               ps_mount_cfg);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ps_card;
}

/*******************************************************************************
 *
 * NAME: sts_futil_sdmmc_unmount
 *
 * DESCRIPTION:SDMMCカードのアンマウント
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_futil_sdmmc_unmount() {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // SDMMCアンマウント
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    ts_sdmmc_mount_info_t* ps_mount;
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < 3 && sts_val == ESP_OK; u8_idx++) {
        ps_mount = &s_sdmmc_mnt_info_list[u8_idx];
        if (ps_mount->pc_mnt_path != NULL) {
            sts_val = sts_sdmmc_unmount(ps_mount);
        }
    }

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_futil_sdmmc_unmount_card
 *
 * DESCRIPTION:SDMMCカードのアンマウント(card指定)
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_futil_sdmmc_unmount_card(sdmmc_card_t* ps_card) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // SDMMCアンマウント
    //==========================================================================
    esp_err_t sts_val = sts_sdmmc_unmount(ps_sdmmc_mount_info_card(ps_card));

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_futil_sdmmc_edit_info
 *
 * DESCRIPTION:SDMMCカードの情報編集
 *
 * PARAMETERS:      Name        RW  Usage
 * ts_sdmmc_info_t* ps_info     W   SDMMCカード情報の編集対象
 * sdmmc_card_t*    ps_card     R   SDMMCカードハンドル
 *
 * RETURNS:
 *   esp_err_t:処理ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
esp_err_t sts_futil_sdmmc_edit_info(ts_sdmmc_info_t* ps_info, sdmmc_card_t* ps_card) {
    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    // ミューテックスの初期化
    pf_mutex_init();
    // クリティカルセクション開始
    if (xSemaphoreTakeRecursive(s_mutex, COM_FUTIL_BLOCK_TIME) == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // SDMMCカード情報の編集
    //==========================================================================
    esp_err_t sts_val = sts_sdmmc_edit_info(ps_info, ps_card);

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 完了ステータス返却
    return sts_val;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_mutex_init
 *
 * DESCRIPTION:Mutex Initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mutex_init() {
    //==========================================================================
    // ミューテックスの初期化
    //==========================================================================
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    //==========================================================================
    // ミューテックス初期処理の切り替え
    //==========================================================================
    pf_mutex_init = v_mutex_init_dummy;
}

/*******************************************************************************
 *
 * NAME: v_mutex_init_dummy
 *
 * DESCRIPTION:Dummy Mutex initial processing
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
static void v_mutex_init_dummy() {
    return;
}

/*******************************************************************************
 *
 * NAME: b_file_copy
 *
 * DESCRIPTION:ファイルコピー処理
 *   コピー先にファイルが無い場合のみコピーする
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_src          R   コピー元ファイルパス
 *   char*          pc_dest         R   コピー先ファイルパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_file_copy(const char* pc_src, const char* pc_dest) {
    //==========================================================================
    // ファイルのオープン
    //==========================================================================
    // ファイルサイズの取得
    long l_size = l_futil_file_size(pc_src);
    // コピー元
    FILE* fpSrc = fopen(pc_src, "rb");
    if (fpSrc == NULL) {
        return false;
    }
    // コピー先
    FILE* fpDest = ps_futil_fopen(pc_dest, "wb");
    if (fpDest == NULL) {
        fclose(fpSrc);
        return false;
    }
    //==========================================================================
    // バッファ単位でコピー
    //==========================================================================
    char pc_buff[256];
    long l_max = l_size / 256;
    long l_cnt;
    for (l_cnt = 0; l_max > l_cnt; l_cnt++) {
        // １単位読み込み
        if (fread(pc_buff, 256, 1, fpSrc) != 1) {
            // EOFでは無いのでエラー
            fclose(fpDest);
            fclose(fpSrc);
            return false;
        }
        // コピー先ファイルへの書き込み
        if (fwrite(pc_buff, 256, 1, fpDest) != 1) {
            // 書き込みエラー
            fclose(fpDest);
            fclose(fpSrc);
            return false;
        }
        if ((l_cnt % 1024) == 1023) {
            vTaskDelay(1);
        }
    }
    //==========================================================================
    // 1バイト単位でコピー
    //==========================================================================
    l_max = l_size % 256;
    char c_buff;
    for (l_cnt = 0; l_max > l_cnt; l_cnt++) {
        // １単位読み込み
        if (fread(&c_buff, 1, 1, fpSrc) != 1) {
            // EOFでは無いのでエラー
            fclose(fpDest);
            fclose(fpSrc);
            return false;
        }
        // コピー先ファイルへの書き込み
        if (fwrite(&c_buff, 1, 1, fpDest) != 1) {
            // 書き込みエラー
            fclose(fpDest);
            fclose(fpSrc);
            return false;
        }
    }
    // ファイルクローズ
    return (fclose(fpSrc) != EOF && fclose(fpDest) != EOF);
}

/*******************************************************************************
 *
 * NAME: b_member_copy
 *
 * DESCRIPTION:ディレクトリメンバーのコピー処理
 *   コピー先のディレクトリが無い場合のみコピーする
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_src          R   コピー元ファイルパス
 *   char*          pc_dest         R   コピー先ファイルパス
 *
 * RETURNS:
 *   true:コピー成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_member_copy(const char* pc_src, const char* pc_dest) {
    // パス長
    char pc_from_path[strlen(pc_src) + 14];
    char pc_to_path[strlen(pc_dest) + 14];
    // 結果
    bool b_result = true;
    // コピー元ディレクトリのリスト
    DIR* pts_dir = opendir(pc_src);
    struct dirent *ent;
    while ((ent = readdir(pts_dir)) != NULL) {
        sprintf(pc_from_path, "%s/%s", pc_src, ent->d_name);
        sprintf(pc_to_path, "%s/%s", pc_dest, ent->d_name);
        if (ent->d_type == DT_REG) {
            // ファイルのコピー
            if (!b_file_copy(pc_from_path, pc_to_path)) {
                b_result = false;
                break;
            }
        } else if (ent->d_type == DT_DIR) {
            // サブディレクトリの作成
            if (mkdir(pc_to_path, (S_IRWXU)) != 0) {
                b_result = false;
                break;
            }
            // サブディレクトリのメンバーのコピー
            if (!b_member_copy(pc_from_path, pc_to_path)) {
                b_result = false;
                break;
            }
        }
    }
    closedir(pts_dir);
    // 実行結果を返却
    return b_result;
}

/*******************************************************************************
 *
 * NAME: b_make_directory
 *
 * DESCRIPTION:ディレクトリ作成
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   ディレクトリパス
 *
 * RETURNS:
 *   true:作成成功
 *
 * NOTES:
 * None.
 ******************************************************************************/
static bool b_make_directory(const char* pc_path) {
    // 編集元と編集先の領域を確保
    char pc_wk_path[strlen(pc_path) + 1];
    strcpy(pc_wk_path, pc_path);
    // 親ディレクトリから順番に作成
    char pc_edit_path[strlen(pc_path) + 1];
    pc_edit_path[0] = '\0';
    char* c_tkn = strtok(pc_wk_path, "/");
    bool b_result = false;
    do {
        strcat(pc_edit_path, "/");
        strcat(pc_edit_path, c_tkn);
        // ディレクトリの存在チェック
        if (b_futil_directory_exist(pc_edit_path)) {
            b_result = true;
        } else {
            // モード指定は無視されている様子なのでフルアクセスで作成
    //        b_result = (mkdir(pc_edit_path, (S_IRWXU | S_IRWXG | S_IRWXO)) == 0);
            b_result = (mkdir(pc_edit_path, (S_IRWXU)) == 0);
        }
        // 次のトークンを取得
        c_tkn = strtok(NULL, "/");
    } while(c_tkn != NULL);
    // 作成結果を返却
    return b_result;
}

/*******************************************************************************
 *
 * NAME: pc_temp_file_path
 *
 * DESCRIPTION:テンポラリファイルのパス（動的確保）の生成
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_path         R   作成先パス
 *
 * RETURNS:
 *   char*:生成したファイルのパス
 *
 * NOTES:
 *  取得したファイルパスは、l_com_mem_free関数の実行で解放する事
 ******************************************************************************/
static char* pc_temp_file_path(const char* pc_path) {
    // 有効なファイルパス（未作成のファイル）を探索
    char pc_name[] = "12345678.tmp";
    // ファイルパス(パス＋セパレータ＋ファイル名＋終端文字)のメモリ動的確保
    uint8_t u8_len = strlen(pc_path) + strlen(pc_name) + 2;
    char* pc_file_path = (char*)pv_mem_malloc(u8_len);
    FILE* fp;
    do {
        // ランダムなファイル名（英数８桁＋拡張子）のファイルパスを生成
        b_vutil_set_rand_upr_alphanumeric(pc_name, 8);
        sprintf(pc_file_path, "%s/%s", pc_path, pc_name);
        // ファイル存在チェック
        fp = fopen(pc_file_path, "r");
        if (fp == NULL) {
            break;
        }
        fclose(fp);
    } while(true);
    // 結果返却
    return pc_file_path;
}

/*******************************************************************************
 *
 * NAME: ps_sdmmc_hs_mount
 *
 * DESCRIPTION:SDMMCカードのマウント（HS接続 4bit mode）
 *
 * PARAMETERS:                          Name            RW  Usage
 * char*                                pc_path         R   マウント先のパス
 * gpio_num_t                           e_gpio_num_cs   R   チップセレクト
 * gpio_num_t                           e_gpio_num_cd   R   カード挿入ピン番号（未設定：SDMMC_SLOT_NO_CD）
 * gpio_num_t                           e_gpio_num_wp   R   ライトプロテクトピン番号（未設定：SDMMC_SLOT_NO_WP）
 * esp_vfs_fat_sdmmc_mount_config_t*    ps_mount_cfg    R   マウント設定
 *
 * RETURNS:
 *   sdmmc_card_t*:処理ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static sdmmc_card_t* ps_sdmmc_hs_mount(char* pc_path,
                                        gpio_num_t e_gpio_num_cs,
                                        gpio_num_t e_gpio_num_cd,
                                        gpio_num_t e_gpio_num_wp,
                                        esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // パス
    if (!b_futil_valid_path(pc_path)) {
        return NULL;
    }
    // スロット情報
    ts_sdmmc_mount_info_t* ps_mount = &s_sdmmc_mnt_info_list[0];
    if (ps_mount->pc_mnt_path != NULL) {
        // 既にマウント済み
        return NULL;
    }
    // チップセレクト
    if (!b_vutil_valid_gpio(e_gpio_num_cs) && e_gpio_num_cs != GPIO_NUM_NC) {
        return NULL;
    }
    // SDカードの挿入検出ピン
    if (!b_vutil_valid_gpio(e_gpio_num_cd) && e_gpio_num_cd != GPIO_NUM_NC) {
        return NULL;
    }
    // SDカードのライトプロテクトピン
    if (!b_vutil_valid_gpio(e_gpio_num_wp) && e_gpio_num_wp != GPIO_NUM_NC) {
        return NULL;
    }
    // マウント設定
    if (ps_mount_cfg == NULL) {
        return NULL;
    }

    //==========================================================================
    // HS接続のピン設定
    //==========================================================================
    // SDカードスロットに接続しているデフォルトピン（4bitモード）をプルアップ
    gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);  // CLK
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);  // CMD
    gpio_set_pull_mode(GPIO_NUM_2,  GPIO_PULLUP_ONLY);  // D0
    gpio_set_pull_mode(GPIO_NUM_4,  GPIO_PULLUP_ONLY);  // D1
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);  // D2
    gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);  // D3
    // SDカードの挿入検出ピン
    if (e_gpio_num_cd != GPIO_NUM_NC) {
        gpio_set_pull_mode(e_gpio_num_cd, GPIO_PULLUP_ONLY);   // CD
    }
    // SDカードのライトプロテクトピン
    if (e_gpio_num_wp != GPIO_NUM_NC) {
        gpio_set_pull_mode(e_gpio_num_wp, GPIO_PULLUP_ONLY);   // WP
    }
    // カードの挿入状態を取得
    if (e_gpio_num_cd != GPIO_NUM_NC) {
        if (gpio_get_level(e_gpio_num_cd) != 0x00) {
            // カード未入力
            return NULL;
        }
    }

    //==========================================================================
    // 指定されたパスにファイルシステムをマウントする
    //==========================================================================
    // SDMMCのホストコントローラ情報生成
    sdmmc_host_t s_host = SDMMC_HOST_DEFAULT();
    s_host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    // SDMMCカードのスロット設定を生成
    sdmmc_slot_config_t s_slot_cfg = SDMMC_SLOT_CONFIG_DEFAULT();
    // SDカード情報
    sdmmc_card_t* ps_card = NULL;
    esp_err_t sts_val = esp_vfs_fat_sdmmc_mount(pc_path, &s_host, &s_slot_cfg, ps_mount_cfg, &ps_card);
    // マウントの成否を判定
    if (sts_val != ESP_OK) {
        return NULL;
    }

    //==========================================================================
    // マウント情報を更新
    //==========================================================================
    // マウントパス
    ps_mount->pc_mnt_path = (char*)pv_mem_malloc(strlen(pc_path) + 1);
    strcpy(ps_mount->pc_mnt_path, pc_path);
    // カード情報
    ps_mount->ps_card = ps_card;

    // 完了ステータス返却
    return ps_card;
}

/*******************************************************************************
 *
 * NAME: ps_sdmmc_spi_mount
 *
 * DESCRIPTION:SDMMCカードのマウント（HS接続 4bit mode）
 *
 * PARAMETERS:                          Name            RW  Usage
 * char*                                pc_path         R   マウント先のパス
 * spi_host_device_t                    e_slot          R   スロット
 * gpio_num_t                           e_gpio_num_cs   R   チップセレクト
 * gpio_num_t                           e_gpio_num_cd   R   カード挿入ピン番号（未設定：SDMMC_SLOT_NO_CD）
 * gpio_num_t                           e_gpio_num_wp   R   ライトプロテクトピン番号（未設定：SDMMC_SLOT_NO_WP）
 * esp_vfs_fat_sdmmc_mount_config_t*    ps_mount_cfg    R   マウント設定
 *
 * RETURNS:
 *   sdmmc_card_t*:処理ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static sdmmc_card_t* ps_sdmmc_spi_mount(char* pc_path,
                                         spi_host_device_t e_slot,
                                         gpio_num_t e_gpio_num_cs,
                                         gpio_num_t e_gpio_num_cd,
                                         gpio_num_t e_gpio_num_wp,
                                         esp_vfs_fat_sdmmc_mount_config_t* ps_mount_cfg) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // パス
    if (!b_futil_valid_path(pc_path)) {
        return NULL;
    }
    // スロット情報
    ts_sdmmc_mount_info_t* ps_mount = ps_sdmmc_spi_mount_info(e_slot);
    if (ps_mount == NULL) {
        return NULL;
    }
    if (ps_mount->pc_mnt_path != NULL) {
        // 既にマウント済み
        return NULL;
    }
    // チップセレクト
    if (!b_vutil_valid_gpio(e_gpio_num_cs) && e_gpio_num_cs != GPIO_NUM_NC) {
        return NULL;
    }
    // SDカードの挿入検出ピン
    if (!b_vutil_valid_gpio(e_gpio_num_cd) && e_gpio_num_cd != GPIO_NUM_NC) {
        return NULL;
    }
    // SDカードのライトプロテクトピン
    if (!b_vutil_valid_gpio(e_gpio_num_wp) && e_gpio_num_wp != GPIO_NUM_NC) {
        return NULL;
    }
    // マウント設定
    if (ps_mount_cfg == NULL) {
        return NULL;
    }

    //==========================================================================
    // HS接続のピン設定
    //==========================================================================
    // SDカードの挿入検出ピン
    if (e_gpio_num_cd != GPIO_NUM_NC) {
        gpio_set_pull_mode(e_gpio_num_cd, GPIO_PULLUP_ONLY);   // CD
    }
    // SDカードのライトプロテクトピン
    if (e_gpio_num_wp != GPIO_NUM_NC) {
        gpio_set_pull_mode(e_gpio_num_wp, GPIO_PULLUP_ONLY);   // WP
    }
    // カードの挿入状態を取得
    if (e_gpio_num_cd != GPIO_NUM_NC) {
        if (gpio_get_level(e_gpio_num_cd) != 0x00) {
            // カード未入力
            return NULL;
        }
    }

    //==========================================================================
    // SPIマウント処理
    //==========================================================================
    // ホスト情報
    sdmmc_host_t s_host = SDSPI_HOST_DEFAULT();
    s_host.slot = e_slot;
    // デバイス設定
    sdspi_device_config_t s_device_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    s_device_cfg.host_id = e_slot;          // ホストID
    s_device_cfg.gpio_cs = e_gpio_num_cs;   // チップセレクト
    s_device_cfg.gpio_cd = e_gpio_num_cd;   // SDカードの挿入検出ライン
    s_device_cfg.gpio_wp = e_gpio_num_wp;   // SDカードのライトプロテクト
    // SDカードをマウント
    sdmmc_card_t* ps_card = NULL;
    esp_err_t sts_val = esp_vfs_fat_sdspi_mount(pc_path, &s_host, &s_device_cfg, ps_mount_cfg, &ps_card);
    if (sts_val != ESP_OK) {
        // マウント出来なかった場合
        return NULL;
    }

    //==========================================================================
    // マウント情報を更新
    //==========================================================================
    // マウントパス
    ps_mount->pc_mnt_path = (char*)pv_mem_malloc(strlen(pc_path) + 1);
    strcpy(ps_mount->pc_mnt_path, pc_path);
    // カード情報
    ps_mount->ps_card = ps_card;

    // カード情報を返却
    return ps_card;
}

/*******************************************************************************
 *
 * NAME: sts_sdmmc_unmount
 *
 * DESCRIPTION:SDMMCアンマウント処理(card指定)
 *
 * PARAMETERS:              Name        RW  Usage
 * ts_sdmmc_mount_info_t*   ps_mount    R   同時オープンファイル数
 *
 * RETURNS:
 *   esp_err_t:処理ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_sdmmc_unmount(ts_sdmmc_mount_info_t* ps_mount) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_mount == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ps_mount->pc_mnt_path == NULL) {
        // 既にアンマウント済み
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // SDMMCをアンマウント
    //==========================================================================
    esp_err_t sts_val = esp_vfs_fat_sdcard_unmount(ps_mount->pc_mnt_path, ps_mount->ps_card);
    if (sts_val != ESP_OK) {
        return sts_val;
    }

    //==========================================================================
    // マウント情報をクリア
    //==========================================================================
    // マウントパス
    l_mem_free(ps_mount->pc_mnt_path);
    ps_mount->pc_mnt_path = NULL;
    // カード情報
    ps_mount->ps_card = NULL;

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_sdmmc_edit_info
 *
 * DESCRIPTION:SDMMCアンマウント処理(card指定)
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_sdmmc_info_t*     ps_info     R   SDMMC情報
 * sdmmc_card_t*        ps_card     R   SDMMCカード情報
 *
 * RETURNS:
 *   esp_err_t:処理ステータス
 *
 * NOTES:
 * None.
 ******************************************************************************/
static esp_err_t sts_sdmmc_edit_info(ts_sdmmc_info_t* ps_info, sdmmc_card_t* ps_card) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 必須チェック
    if (ps_info == NULL || ps_card == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // カード情報ステータスチェック
    esp_err_t sts_val = sdmmc_get_status(ps_card);
    if (sts_val != ESP_OK) {
        return sts_val;
    }
    // マウント情報の有無
    ts_sdmmc_mount_info_t* ps_mount = ps_sdmmc_mount_info_card(ps_card);
    if (ps_mount == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ps_mount->pc_mnt_path == NULL) {
        // マウントされていない
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // SDMMCカード情報の編集
    //==========================================================================
    // テンポラリファイルにSDカード情報を書き込む
    char* pc_temp_path = pc_temp_file_path(ps_mount->pc_mnt_path);
    FILE* ps_tmp_file = fopen(pc_temp_path, "w+");
    // ファイルオープン不能な場合にはステータス返却して終了
    if (ps_tmp_file == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // SDMMCカード情報をテンポラリファイルに書き込み
    sdmmc_card_print_info(ps_tmp_file, ps_card);
    // テンポラリファイルからSDMMCカード情報を読み込む
    rewind(ps_tmp_file);
    char pc_line_buff[255];
    while (fgets(pc_line_buff, 255, ps_tmp_file) != NULL) {
        // 改行コード除去
        pc_line_buff[strlen(pc_line_buff) - 1] = '\0';
        // ２文字目で項目を判定
        switch (pc_line_buff[1]) {
        case 'a':
            // Name:カード名
            strcpy(ps_info->c_card_name, &pc_line_buff[6]);
            break;
        case 'y':
            // Type:カードタイプ
            strcpy(ps_info->c_card_type, &pc_line_buff[6]);
            break;
        case 'p':
            // Speed:接続速度
            strcpy(ps_info->c_speed, &pc_line_buff[7]);
            break;
        case 'i':
            // Size:カードサイズ
            strcpy(ps_info->c_card_size, &pc_line_buff[6]);
            break;
        default:
            break;
        }
    }
    // テンポラリファイルをクローズ
    fclose(ps_tmp_file);
    // テンポラリファイルを削除
    remove(pc_temp_path);
    // テンポラリファイルパスのメモリ解放
    l_mem_free(pc_temp_path);

    // 結果返信
    return ESP_OK;
}

/** SDMMCマウント情報の取得 */
static ts_sdmmc_mount_info_t* ps_sdmmc_spi_mount_info(spi_host_device_t e_slot) {
    // 対象を判別
    ts_sdmmc_mount_info_t* ps_mount;
    switch (e_slot) {
    case SPI2_HOST:
        ps_mount = &s_sdmmc_mnt_info_list[1];
        break;
    case SPI3_HOST:
        ps_mount = &s_sdmmc_mnt_info_list[2];
        break;
    default:
        return NULL;
    }
    // 対象無しの場合
    return ps_mount;
}

/** cardに対応したSDMMCマウント情報の取得 */
static ts_sdmmc_mount_info_t* ps_sdmmc_mount_info_card(sdmmc_card_t* ps_card) {
    // 入力チェック
    if (ps_card == NULL) {
        return NULL;
    }
    // 対象を判別
    ts_sdmmc_mount_info_t* ps_mount;
    if (s_sdmmc_mnt_info_list[0].ps_card == ps_card) {
        ps_mount = &s_sdmmc_mnt_info_list[0];
    } else if (s_sdmmc_mnt_info_list[1].ps_card == ps_card) {
        ps_mount = &s_sdmmc_mnt_info_list[1];
    } else if (s_sdmmc_mnt_info_list[2].ps_card == ps_card) {
        ps_mount = &s_sdmmc_mnt_info_list[2];
    } else {
        return NULL;
    }
    // 対象無しの場合
    return ps_mount;
}


/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
