/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common value utility functions source file
 *
 * CREATED:2019/09/10 02:46:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:値の変換等の値に関する基本的なユーティリティ関数群
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
/***      Macro Definitions                                                 ***/
/******************************************************************************/
#ifndef COM_VAL_INIT_SEED
#define COM_VAL_INIT_SEED (0x5F7F3D8B)
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include "ntfw_com_value_util.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <esp_system.h>
#include <esp_random.h>


/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/

/**
 * 乱数の生成関数
 */
typedef uint32_t (*v_com_value_random_t)();

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** 数字文字列（10進数） */
const char STR_DEC_NUMBER[] = "0123456789";
/** 数字文字列（16進数） */
const char STR_HEX_NUMBER[] = "0123456789ABCDEF";
/** 英小文字列 */
const char STR_LOWERCASE[] = "abcdefghijklmnopqrstuvwxyz";
/** 英大文字列 */
const char STR_UPPERCASE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
/** 英数小文字列 */
const char STR_LOWER_ALPHANUMERIC[] = "0123456789abcdefghijklmnopqrstuvwxyz";
/** 英数大文字列 */
const char STR_UPPER_ALPHANUMERIC[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
/** Base64文字列 */
const char STR_BASE64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/
//==============================================================================
// チェック関数
//==============================================================================
/*******************************************************************************
 *
 * NAME: b_vutil_valid_date
 *
 * DESCRIPTION:GPIO番号チェック
 *
 * PARAMETERS:      Name            RW  Usage
 *   gpio_num_t     gpio_no         R   GPIO番号
 *
 * RETURNS:
 *   bool TRUE:チェックOK（使用可能なピン番号）
 *
 ******************************************************************************/
bool b_vutil_valid_gpio(gpio_num_t gpio_no) {
    // 利用不可能なピン番号を判定
    return !(gpio_no < GPIO_NUM_3 || gpio_no > GPIO_NUM_39 ||
              gpio_no == 20 || gpio_no == 24 ||
              gpio_no == 28 || gpio_no == 29 ||
              gpio_no == 30 || gpio_no == 31);
}

//==============================================================================
// 情報関数
//==============================================================================

/*******************************************************************************
 *
 * NAME: i_vutil_dec_len_i
 *
 * DESCRIPTION:10進数の桁数（int）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_val           R   算出対象
 *
 * RETURNS:
 *   int：10進数の桁数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_dec_len_i(int i_val) {
    // 桁数算出
    uint32_t u32_wk_val = abs(i_val);
    int i_len = i_vutil_dec_len_u32(u32_wk_val);
    // マイナス判定
    if (i_val < 0) {
        i_len++;
    }
    // 桁数返信
    return i_len;
}

/*******************************************************************************
 *
 * NAME: i_vutil_dec_len_u32
 *
 * DESCRIPTION:10進数の桁数（uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t       u32_val         R   算出対象
 *
 * RETURNS:
 *   int：10進数の桁数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_dec_len_u32(uint32_t u32_val) {
    // 桁数算出
    int i_len = 1;
    uint32_t u32_wk_val = u32_val;
    while (u32_wk_val >= 10) {
        i_len++;
        u32_wk_val = u32_wk_val / 10;
    }
    // 桁数返信
    return i_len;
}

/*******************************************************************************
 *
 * NAME: i_vutil_hex_len_i
 *
 * DESCRIPTION:16進数の桁数（int）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_val           R   算出対象
 *
 * RETURNS:
 *   int：16進数の桁数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_hex_len_i(int i_val) {
    // 桁数算出
    uint32_t u32_wk_val = abs(i_val);
    int i_len = i_vutil_hex_len_u32(u32_wk_val);
    // マイナス判定
    if (i_val < 0) {
        i_len++;
    }
    // 桁数返信
    return i_len;
}

/*******************************************************************************
 *
 * NAME: i_vutil_hex_len_u32
 *
 * DESCRIPTION:16進数の桁数（uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t       u32_val         R   算出対象
 *
 * RETURNS:
 *   int：16進数の桁数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_hex_len_u32(uint32_t u32_val) {
    // 桁数算出
    int i_len = 1;
    uint32_t u32_wk_val = u32_val;
    while (u32_wk_val != 0) {
        i_len++;
        u32_wk_val = u32_wk_val >> 4;
    }
    // 桁数返信
    return i_len;
}

/*******************************************************************************
 *
 * NAME: i_base64_char_index
 *
 * DESCRIPTION:Base64の文字インデックスの取得処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   char           c_char      R   Base64文字
 *
 * RETURNS:
 * int:文字インデックス1、パディング文字（"="）は64、対象外文字の場合は-
 *
 ******************************************************************************/
int i_vutil_base64_char_index(char c_char) {
    // 英大文字の判定
    if (c_char >= 'A' && c_char <= 'Z') {
        return c_char - 'A';
    }
    // 英子文字の判定
    if (c_char >= 'a' && c_char <= 'z') {
      return c_char - 'a' + 26;
    }
    // 数字の判定
    if (c_char >= '0' && c_char <= '9') {
        return c_char - '0' + 52;
    }
    // 記号の判定
    if (c_char == '+') {
        return 62;
    }
    if (c_char == '/') {
        return 63;
    }
    // パディング文字の判定
    if (c_char == '=') {
        return 64;
    }
    // 対象外の文字
    return -1;
}

/*******************************************************************************
 *
 * NAME: i_vutil_base64_len_i
 *
 * DESCRIPTION:Base64変換後の桁数（int）
 *
 * PARAMETERS:      Name            RW  Usage
 *   int            i_len           R   変換前のバイト数
 *
 * RETURNS:
 *   int：Base64変換後の文字数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_base64_len_i(int i_len) {
    // 文字数算出
    return ((i_len * 8) + 4) / 6;
}

/*******************************************************************************
 *
 * NAME: i_vutil_base64_len_u32
 *
 * DESCRIPTION:Base64変換後の桁数（uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t       u32_len         R   変換前のバイト数
 *
 * RETURNS:
 *   int：Base64変換後の文字数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_base64_len_u32(uint32_t u32_len) {
    // 文字数算出
    return ((u32_len * 8) + 4) / 6;
}

/*******************************************************************************
 *
 * NAME: i_vutil_byte_len_base64
 *
 * DESCRIPTION:Byte配列に変換後の桁数（Base64）
 *
 * PARAMETERS:      Name        RW  Usage
 * char*            pc_src      R   Base64文字列
 * uint32_t         u32_len     R   Base64文字数
 *
 * RETURNS:
 *   int：byte配列に変換後の配列長、変換不能の場合は-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_byte_len_base64(const char* pc_src, const uint32_t u32_len) {
    //==========================================================================
    // 文字数チェック
    //==========================================================================
    if ((u32_len % 4) != 0) {
        return -1;
    }

    //==========================================================================
    // 文字種チェック
    //==========================================================================
    int i_base64_idx;
    int i_before_idx = 0;
    uint32_t u32_pad_len = 0;
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        // base64インデックス
        i_base64_idx = i_vutil_base64_char_index(pc_src[u32_idx]);
        if (i_base64_idx < 0) {
            return -1;
        }
        // パディング文字チェック
        if (i_base64_idx == 64) {
            u32_pad_len++;
        } else if (i_before_idx == 64) {
            // パディング文字以外の直前文字がパディング文字
            return -1;
        }
        i_before_idx = i_base64_idx;
    }
    // パディング文字数を算出
    if (u32_pad_len > 2) {
        // パディングが2文字以上はエラー
        return -1;
    }

    //==========================================================================
    // バイト数算出
    //==========================================================================
    return ((u32_len - u32_pad_len) * 6) / 8;
}

//==============================================================================
// 文字列関連関数
//==============================================================================
/*******************************************************************************
 *
 * NAME: i_vutil_strlen
 *
 * DESCRIPTION:NULL対応された文字列長取得(NULL値は0文字として扱う)
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_str      R   対象文字列
 *
 * RETURNS:
 *   int NULL値を0文字とした文字数
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_strlen(const char* pc_str) {
    // NULL判定
    if (pc_str == NULL) {
        return 0;
    }
    // 通常の文字列長算出
    return strlen(pc_str);
}

/*******************************************************************************
 *
 * NAME: pc_vutil_strcpy
 *
 * DESCRIPTION:NULL対応された文字列編集(NULL値はコピーしない)
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_str1     R   編集先文字列
 *   char*          pc_str2     R   編集元文字列
 *
 * RETURNS:
 *   char* pc_str1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
char* pc_vutil_strcpy(char* pc_str1, const char* pc_str2) {
    // NULL判定
    if (pc_str1 == NULL || pc_str2 == NULL) {
        return pc_str1;
    }
    // 通常の文字列比較
    return strcpy(pc_str1, pc_str2);
}

/*******************************************************************************
 *
 * NAME: i_vutil_strcmp
 *
 * DESCRIPTION:NULL対応された文字列比較(NULL値は最小値として扱う)
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_str1     R   比較対象文字列
 *   char*          pc_str2     R   比較対象文字列
 *
 * RETURNS:
 *   int NULL値を最小値としたstrcmpの結果値
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_strcmp(const char* pc_str1, const char* pc_str2) {
    // NULL判定
    if (pc_str1 == NULL) {
        if (pc_str2 == NULL) {
            return 0;
        }
        return -1;
    }
    if (pc_str2 == NULL) {
        return 1;
    }
    // 通常の文字列比較
    return strcmp(pc_str1, pc_str2);
}


/*******************************************************************************
 *
 * NAME: i_vutil_index_of
 *
 * DESCRIPTION:配列上の文字インデックス探索（先頭）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_str          R   探索対象文字列
 *   char           c_ch            R   探索文字
 *
 * RETURNS:
 *   int 見つかった文字の先頭インデックス、見つからない場合は-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_index_of(const char* pc_str, char c_ch) {
    // 入力チェック
    if (pc_str == NULL) {
        return -1;
    }
    // 文字探索
    int i_idx;
    for (i_idx = 0; *pc_str != '\0'; i_idx++) {
        if (*pc_str == c_ch) {
            return i_idx;
        }
        pc_str++;
    }
    return -1;
}

/*******************************************************************************
 *
 * NAME: i_vutil_last_index_of
 *
 * DESCRIPTION:配列上の文字インデックス探索（最終）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_str          R   探索対象文字列
 *   char           c_ch            R   探索文字
 *
 * RETURNS:
 *   int 見つかった文字の最終インデックス、見つからない場合は-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_last_index_of(const char* pc_str, char c_ch) {
    // 入力チェック
    if (pc_str == NULL) {
        return -1;
    }
    // 文字探索
    int i_idx = strlen(pc_str) - 1;
    while (i_idx >= 0) {
        if (pc_str[i_idx] == c_ch) {
            break;;
        }
        i_idx--;
    }
    return i_idx;
}

/*******************************************************************************
 *
 * NAME: b_vutil_replace_char
 *
 * DESCRIPTION:文字列の文字置き換え
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_str          RW  対象文字列
 *   char           c_ch            R   探索文字
 *   char           c_rep           R   置き換え文字
 *
 * RETURNS:
 *   true:文字置き換え有り
 *
 * NOTES:
 *   None.
 ******************************************************************************/
bool b_vutil_replace_char(char* pc_str, char c_ch, char c_rep) {
    char* pc_tgt = strchr(pc_str, c_ch);
    if (pc_tgt == NULL) {
        return false;
    }
    *pc_tgt = c_rep;
    return true;
}

/*******************************************************************************
 *
 * NAME: i_vutil_substr
 *
 * DESCRIPTION:文字列切り出し
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_to           W   編集先文字列
 *   char*          pc_from         R   編集元文字列
 *   uint32_t       u32_pos         R   ポジション
 *   uint32_t       u32_len         R   切り出し文字数
 *
 * RETURNS:
 *   int 切り出し文字数、切り出し位置が不正の場合には-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_substr(char* pc_to, const char* pc_from,
                     uint32_t u32_pos, uint32_t u32_len) {
    // 入力チェック
    if (pc_to == NULL || pc_from == NULL) {
        return -1;
    }
    uint32_t u32_from_len = (uint32_t)strlen(pc_from);
    if (u32_pos >= u32_from_len) {
        pc_to[0] = '\0';
        return -1;
    }
    // 文字列切り出し
    uint32_t u32_cpy_len = u32_len;
    if ((u32_pos + u32_len) > u32_from_len) {
        u32_cpy_len = u32_from_len - u32_pos;
    }
    strncpy(pc_to, &pc_from[u32_pos], u32_cpy_len);
    pc_to[u32_cpy_len] = '\0';
    // 正常終了
    return u32_cpy_len;
}

/*******************************************************************************
 *
 * NAME: i_vutil_str_left
 *
 * DESCRIPTION:先頭からの文字列切り出し
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_to           W   編集先文字列
 *   char*          pc_from         R   編集元文字列
 *   uint32_t       u32_len         R   切り出し文字数
 *
 * RETURNS:
 *   int 切り出し文字数、切り出し位置が不正の場合には-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_str_left(char* pc_to, const char* pc_from, uint32_t u32_len) {
    // 入力チェック
    if (pc_to == NULL || pc_from == NULL) {
        return -1;
    }
    // 文字数判定
    int i_len = strlen(pc_from);
    if (u32_len < i_len) {
        i_len = (int)u32_len;
    }
    strncpy(pc_to, pc_from, i_len);
    pc_to[i_len] = '\0';
    return i_len;
}

/*******************************************************************************
 *
 * NAME: i_vutil_str_rpad
 *
 * DESCRIPTION:文字充填（後方）
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_str      W   編集対象
 *   char           c_pad       R   充填文字
 *   uint32_t       u32_len     R   全体文字数
 *
 * RETURNS:
 *   int 充填した文字数、エラー時は-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_str_rpad(char* pc_str, const char c_pad, uint32_t u32_len) {
    // 入力チェック
    if (pc_str == NULL) {
        return -1;
    }
    // 文字の充填位置を探索
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        if (pc_str[u32_idx] == '\0') {
            break;
        }
    }
    // 充填文字数を算出
    int i_add = u32_len - u32_idx;
    // 文字を充填
    while (u32_idx < u32_len) {
        pc_str[u32_idx] = c_pad;
        u32_idx++;
    }
    // 終端文字を入れる
    pc_str[u32_idx] = '\0';
    // 結果返信
    return i_add;
}

/*******************************************************************************
 *
 * NAME: i_vutil_str_lpad
 *
 * DESCRIPTION:文字充填（前方）
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_str      W   編集対象
 *   char           c_pad       R   充填文字
 *   uint32_t       u32_len     R   全体文字数
 *
 * RETURNS:
 *   int 充填した文字数、エラー時は-1
 *
 * NOTES:
 *   None.
 ******************************************************************************/
int i_vutil_str_lpad(char* pc_str, const char c_pad, uint32_t u32_len) {
    // 入力チェック
    if (pc_str == NULL) {
        return -1;
    }
    // 終端文字を入れる
    pc_str[u32_len] = '\0';
    // 後方にシフト
    int i_len = strlen(pc_str);
    int i_from_idx = i_len - 1;
    int i_to_idx   = (int)u32_len - 1;
    while (i_from_idx >= 0) {
        pc_str[i_to_idx] = pc_str[i_from_idx];
        i_from_idx--;
        i_to_idx--;
    }
    // 充填文字数を算出
    int i_add = u32_len - i_len;
    // 文字を充填
    int i_idx;
    for (i_idx = 0; i_idx < i_add; i_idx++) {
        pc_str[i_idx] = c_pad;
    }
    // 結果返信
    return i_add;
}

/*******************************************************************************
 *
 * NAME: b_vutil_dec_string
 *
 * DESCRIPTION:10進数文字列判定
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_num          R   チェック対象
 *   uint32_t       u32_len         R   最大文字数
 *
 * RETURNS:
 *   bool: true(指定桁数以下の数字文字列)
 *
 * NOTES:
 *   None.
 ******************************************************************************/
bool b_vutil_dec_string(char* pc_num, uint32_t u32_max_len) {
    // 入力チェック
    if (pc_num == NULL) {
        return false;
    }
    // 文字数チェック
    int i_len = strlen(pc_num);
    if (i_len > u32_max_len) {
        return false;
    }
    // 文字種チェック
    int i_idx;
    for (i_idx = 0; i_idx < i_len; i_idx++) {
        if (strchr(STR_DEC_NUMBER, pc_num[i_idx]) == NULL) {
            break;
        }
    }
    // 結果判定
    return (i_idx >= i_len);
}

/*******************************************************************************
 *
 * NAME: b_vutil_hex_string
 *
 * DESCRIPTION:16進数文字列判定
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_num          R   チェック対象
 *   uint32_t       u32_len         R   最大文字数
 *
 * RETURNS:
 *   bool: true(指定桁数以下の数値文字列)
 *
 * NOTES:
 *   None.
 ******************************************************************************/
bool b_vutil_hex_string(char* pc_num, uint32_t u32_max_len) {
    // 入力チェック
    if (pc_num == NULL) {
        return false;
    }
    // 文字数チェック
    int i_len = strlen(pc_num);
    if (i_len > u32_max_len) {
        return false;
    }
    // 文字種チェック
    int i_idx;
    for (i_idx = 0; i_idx < i_len; i_idx++) {
        if (strchr(STR_HEX_NUMBER, pc_num[i_idx]) == NULL) {
            break;
        }
    }
    // 結果判定
    return (i_idx >= i_len);
}

//==============================================================================
// 生成関数
//==============================================================================
/*******************************************************************************
 *
 * NAME: u32_vutil_random
 *
 * DESCRIPTION:
 *   疑似乱数で強化された乱数の生成処理
 *   ESP32では、乱数の生成にWifiやBluetoothの無線回路のノイズを応用した
 *   メタステーブルを利用している様子なので、メタステーブルのエントロピー低下
 *   （起こらないと思いますが）を想定して、XorShiftで強化した乱数を生成する。
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   uint32_t 生成された乱数
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_random() {
    // create random
    uint32_t u32_random = esp_random();
    // update seed
    static uint32_t u32_entropy_src = COM_VAL_INIT_SEED;
    u32_entropy_src = ((u32_entropy_src >> 1) | (u32_entropy_src << 31)) ^ u32_random;
    srand(u32_entropy_src);
    // 結果返信
    return u32_random;
}

/*******************************************************************************
 *
 * NAME: b_vutil_set_u8_rand_array
 *
 * DESCRIPTION:生成関数（乱数配列）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint8_t*       pu8_rand_array  W   編集対象配列
 *   uint32_t       u32_len         R   編集サイズ
 *
 * RETURNS:
 *   成功：true
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_vutil_set_u8_rand_array(uint8_t* pu8_rand_array,
                               const uint32_t u32_len) {
    // 入力チェック
    if (pu8_rand_array == NULL) {
        return false;
    }
    // 乱数生成編集
    tu_type_converter_t converter;
    uint32_t u32_end = u32_len - (u32_len % 4);
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_end; u32_idx += 4) {
        converter.u32_values[0] = u32_vutil_random();
        pu8_rand_array[u32_idx] = converter.u8_values[0];
        pu8_rand_array[u32_idx + 1] = converter.u8_values[1];
        pu8_rand_array[u32_idx + 2] = converter.u8_values[2];
        pu8_rand_array[u32_idx + 3] = converter.u8_values[3];
    }
    if (u32_idx < u32_len) {
        converter.u32_values[0] = u32_vutil_random();
        do {
            pu8_rand_array[u32_idx] = converter.u8_values[u32_idx % 4];
            u32_idx++;
        } while (u32_idx < u32_len);
    }
    // 生成完了
    return true;
}

/*******************************************************************************
 *
 * NAME: b_vutil_set_u32_rand_array
 *
 * DESCRIPTION:生成関数（乱数配列）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t*      pu32_rand_array W   編集対象配列
 *   uint32_t       u32_len         R   編集サイズ
 *
 * RETURNS:
 *   成功：true
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_vutil_set_u32_rand_array(uint32_t* pu32_rand_array,
                                const uint32_t u32_len) {
    // 入力チェック
    if (pu32_rand_array == NULL) {
        return false;
    }
    // 乱数生成編集
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        pu32_rand_array[u32_idx] = u32_vutil_random();
    }
    // 生成完了
    return true;
}

/*******************************************************************************
 *
 * NAME: b_vutil_set_rand_string
 *
 * DESCRIPTION:生成関数（乱数文字列）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_rand_string  W   編集対象文字列
 *   char*          pc_src_string   R   編集元文字列
 *   uint32_t       u32_len         R   編集サイズ
 *
 * RETURNS:
 *   成功：true
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_vutil_set_rand_string(char* pc_rand_string,
                                 const char* pc_src_string,
                                 const uint32_t u32_len) {
    // 入力チェック
    if (pc_rand_string == NULL || pc_src_string == NULL) {
        return false;
    }
    // 乱数生成編集
    uint32_t u32_src_size = strlen(pc_src_string);
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        pc_rand_string[u32_idx] = pc_src_string[u32_vutil_random() % u32_src_size];
    }
    pc_rand_string[u32_idx] = '\0';
    // 生成完了
    return true;
}

/*******************************************************************************
 *
 * NAME: b_vutil_set_rand_lwr_alphanumeric
 *
 * DESCRIPTION:生成関数：乱数文字列（英数小文字）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_rand_string  W   編集対象文字列
 *   uint32_t       u32_len         R   編集サイズ
 *
 * RETURNS:
 *   成功：true
 *
 * NOTES:
 * None.
 ******************************************************************************/
extern bool b_vutil_set_rand_lwr_alphanumeric(char* pc_rand_string,
                                               const uint32_t u32_len) {
    return b_vutil_set_rand_string(pc_rand_string, STR_LOWER_ALPHANUMERIC, u32_len);
}

/*******************************************************************************
 *
 * NAME: b_vutil_set_rand_upr_alphanumeric
 *
 * DESCRIPTION:生成関数：乱数文字列（英数大文字）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char*          pc_rand_string  W   編集対象文字列
 *   uint32_t       u32_len         R   編集サイズ
 *
 * RETURNS:
 *   成功：true
 *
 * NOTES:
 * None.
 ******************************************************************************/
extern bool b_vutil_set_rand_upr_alphanumeric(char* pc_rand_string,
                                               const uint32_t u32_len) {
    return b_vutil_set_rand_string(pc_rand_string, STR_UPPER_ALPHANUMERIC, u32_len);
}

//==============================================================================
// 変換関数
//==============================================================================

/*******************************************************************************
 *
 * NAME:u32_vutil_binary_to_bcd
 *
 * DESCRIPTION:変換関数（数値→BCD形式）
 *
 * PARAMETERS:      Name            RW  Usage
 *    uint32_t      u32_val         R   変換対象
 * RETURNS:
 *    uint32:変換結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_binary_to_bcd(uint32_t u32_val) {
    uint32_t result = 0;	// 変換値
    uint8_t shift_cnt;
    for (shift_cnt = 0; shift_cnt < 32; shift_cnt += 4) {
        result = result | ((u32_val % 10) << shift_cnt);
        u32_val = u32_val / 10;
    }
    return result;
}

/*******************************************************************************
 *
 * NAME: u32_vutil_bcd_to_binary
 *
 * DESCRIPTION:変換関数（BCD形式→数値）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t       u32_val         R   変換対象
 * RETURNS:
 *   uint32_t:変換結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_bcd_to_binary(uint32_t u32_val) {
    uint32_t result = 0;        // 変換値
    uint32_t coefficient = 1;   // 係数
    uint8_t cnt;
    for (cnt = 0; cnt < 8; cnt++) {
        result += ((u32_val & 0x0000000F) % 10) * coefficient;
        u32_val = u32_val >> 4;
        coefficient = coefficient * 10;
    }
    return result;
}

/*******************************************************************************
 *
 * NAME: u32_vutil_u8_to_binary
 *
 * DESCRIPTION:変換関数（uint8_t→８桁の２進表現）
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint8_t        u8_val          R   変換対象
 *
 * RETURNS:
 *   uint32_t ８桁の２進数表現に変換した結果
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_u8_to_binary(uint8_t u8_val) {
    uint32_t u32_result = 0;
    uint32_t u32_addVal = 1;
    while (u8_val != 0) {
        if ((u8_val & 0x01) == 0x01) {
            u32_result = u32_result + u32_addVal;
        }
        u8_val = u8_val >> 1;
        u32_addVal = u32_addVal * 10;
    }
    return u32_result;
}

/*******************************************************************************
 *
 * NAME: v_vutil_u8_to_hex_string
 *
 * DESCRIPTION:変換関数（バイト配列から１６進文字列変換）
 *
 * PARAMETERS:      Name            RW  Usage
 * const uint8_t*   pu8_data        R   変換対象データ
 * uint32_t         u32_data_length R   データサイズ
 * char*            pc_string       W   編集先
 *
 * RETURNS:
 *
 * NOTES:
 * None.
 ******************************************************************************/
void v_vutil_u8_to_hex_string(const uint8_t* pu8_data, uint32_t u32_data_length, char* pc_string) {
    // 入力チェック
     if (pu8_data == NULL || pc_string == NULL) {
         return;
     }
     // 文字列編集
     uint8_t u8_data;
     uint32_t u32_pos = 0;
     uint32_t u32_idx;
     for (u32_idx = 0; u32_idx < u32_data_length; u32_idx++) {
         // 変換対象
         u8_data = pu8_data[u32_idx];
         pc_string[u32_pos++] = STR_HEX_NUMBER[u8_data >> 4];
         pc_string[u32_pos++] = STR_HEX_NUMBER[u8_data & 0x0F];
     }
     pc_string[u32_pos] = '\0';
}

/*******************************************************************************
 *
 * NAME: u32_vutil_to_numeric
 *
 * DESCRIPTION:変換関数（数字文字列→uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_str          R   変換対象
 *
 * RETURNS:
 *   uint32_t 変換結果、変換不能の場合には0を返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_to_numeric(const char* pc_str) {
    // 入力チェック
    if (pc_str == NULL) {
        return 0;
    }
    int i_len = strlen(pc_str);
    if (i_len <= 0) {
        return 0;
    }
    // 数値に変換
    uint32_t u32_result = 0;
    char c_val;
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < i_len; u8Idx++) {
        c_val = pc_str[u8Idx];
        if (c_val < '0' || c_val > '9') {
            return 0;
        }
        u32_result = u32_result * 10 + (uint32_t)(c_val - '0');
    }
    return u32_result;
}

/*******************************************************************************
 *
 * NAME: u64_vutil_to_numeric
 *
 * DESCRIPTION:変換関数（数字文字列→uint64_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_str          R   変換対象
 *
 * RETURNS:
 *   uint64_t 変換結果、変換不能の場合には0を返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint64_t u64_vutil_to_numeric(const char* pc_str) {
    // 入力チェック
    if (pc_str == NULL) {
        return 0;
    }
    int i_len = strlen(pc_str);
    if (i_len <= 0) {
        return 0;
    }
    // 数値に変換
    uint64_t u64_result = 0;
    char c_val;
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < i_len; u8Idx++) {
        c_val = pc_str[u8Idx];
        if (c_val < '0' || c_val > '9') {
            return 0;
        }
        u64_result = u64_result * 10 + (uint64_t)(c_val - '0');
    }
    return u64_result;
}

/*******************************************************************************
 *
 * NAME: u32_vutil_array_to_u32
 *
 * DESCRIPTION:変換関数（数字配列→uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_str          R   変換対象
 *   uint8_t        u8_begin        R   先頭文字インデックス
 *   uint8_t        u8_length       R   文字列長
 *
 * RETURNS:
 *   uint32_t 変換結果、変換不能の場合には0を返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint32_t u32_vutil_array_to_u32(const char* pc_str,
                                uint8_t u8_begin,
                                uint8_t u8_length) {
    // 入力チェック
     if (pc_str == NULL) {
        return 0;
    }
    int i_len = strlen(pc_str);
    if (i_len <= 0 || u8_begin >= i_len || u8_length == 0) {
        return 0;
    }
    // ファイル読み込み
    uint32_t u32_result = 0;
    char c_val;
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < u8_length; u8Idx++) {
        c_val = pc_str[u8_begin + u8Idx];
        if (c_val < '0' || c_val > '9') {
            return 0;
        }
        u32_result = u32_result * 10 + (uint32_t)(c_val - '0');
    }
    return u32_result;
}

/*******************************************************************************
 *
 * NAME: u64_vutil_array_to_u64
 *
 * DESCRIPTION:変換関数（数字文字列→uint32_t）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_str          R   変換対象
 *   uint8_t        u8_begin        R   先頭文字インデックス
 *   uint8_t        u8_length       R   文字列長
 *
 * RETURNS:
 *   uint64_t 変換結果、変換不能の場合には0を返却
 *
 * NOTES:
 * None.
 ******************************************************************************/
uint64_t u64_vutil_array_to_u64(const char* pc_str,
                                uint8_t u8_begin,
                                uint8_t u8_length) {
    // 入力チェック
    if (pc_str == NULL) {
        return 0;
    }
    int i_len = strlen(pc_str);
    if (i_len <= 0 || u8_begin >= i_len || u8_length == 0) {
        return 0;
    }
    // ファイル読み込み
    uint32_t u32_result = 0;
    char c_val;
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < u8_length; u8Idx++) {
        c_val = pc_str[u8_begin + u8Idx];
        if (c_val < '0' || c_val > '9') {
            return 0;
        }
        u32_result = u32_result * 10 + (uint32_t)(c_val - '0');
    }
    return u32_result;
}

/*******************************************************************************
 *
 * NAME: b_vutil_edit_dec_string
 *
 * DESCRIPTION:変換関数（数値から10進数文字列判定）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_num          R   編集先
 *   uint64_t       u64_val         R   変換対象
 *
 * RETURNS:
 *   bool 正常に編集完了時はtrue
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_vutil_edit_dec_string(char* pc_num, uint64_t u64_val) {
    // 入力チェック
    if (pc_num == NULL) {
        return false;
    }
    // 編集バッファ
    char c_num_buff[21];
    uint64_t u64_wk_val = u64_val;
    uint8_t u8_idx = 20;
    c_num_buff[u8_idx] = '\0';
    // 文字列変換
    do {
        u8_idx--;
        c_num_buff[u8_idx] = STR_DEC_NUMBER[u64_wk_val % 10];
        // 次サイクルへ
        u64_wk_val = u64_wk_val / 10;
    } while(u64_wk_val > 0);
    // 文字列を変換
    strcpy(pc_num, &c_num_buff[u8_idx]);
    // 変換完了
    return true;
}

/*******************************************************************************
 *
 * NAME: b_vutil_edit_hex_string
 *
 * DESCRIPTION:変換関数（数値から16進数文字列判定）
 *
 * PARAMETERS:      Name            RW  Usage
 *   char           pc_num          R   編集先
 *   uint64_t       u64_val         R   変換対象
 *
 * RETURNS:
 *   bool 正常に編集完了時はtrue
 *
 * NOTES:
 * None.
 ******************************************************************************/
bool b_vutil_edit_hex_string(char* pc_num, uint64_t u64_val) {
    // 入力チェック
    if (pc_num == NULL) {
        return false;
    }
    // 編集バッファ
    char c_num_buff[17];
    uint64_t u64_wk_val = u64_val;
    uint8_t u8_idx = 16;
    c_num_buff[u8_idx] = '\0';
    // 文字列変換
    do {
        u8_idx--;
        c_num_buff[u8_idx] = STR_HEX_NUMBER[u64_wk_val % 16];
        // 次サイクルへ
        u64_wk_val = u64_wk_val / 10;
    } while(u64_wk_val > 0);
    // 文字列を変換
    strcpy(pc_num, &c_num_buff[u8_idx]);
    // 変換完了
    return true;
}

/*******************************************************************************
 *
 * NAME: u8_vutil_upper_case
 *
 * DESCRIPTION:変換関数：大文字化
 *
 * PARAMETERS:      Name       RW  Usage
 *   char*          pc_dst     W   編集先
 *   char*          pc_src     R   元文字列
 *
 * RETURNS:
 *   uint32_t:大文字に変換した文字数
 *
 ******************************************************************************/
uint32_t u32_vutil_upper_case(char* pc_dst, const char* pc_src) {
    // 変換文字数
    uint32_t u32_cnt = 0;
    // 入力チェック
    if (pc_dst == NULL || pc_src == NULL) {
        return u32_cnt;
    }    // ファイル読み込み
    // 文字列長
    int i_len = strlen(pc_src);
    // 大文字化処理
    int i_add = 'A' - 'a';
    int i_idx;
    for (i_idx = 0; i_idx < i_len; i_idx++) {
        pc_dst[i_idx] = pc_src[i_idx];
        // 大文字化判定
        if (pc_dst[i_idx] >= 'a' && pc_dst[i_idx] <= 'z') {
            pc_dst[i_idx] += i_add;
            u32_cnt++;
        }
    }
    // 終端文字を挿入
    pc_dst[i_len] = '\0';
    // 変換文字数を返却
    return u32_cnt;
}

/*******************************************************************************
 *
 * NAME: u32_vutil_lower_case
 *
 * DESCRIPTION:変換関数：小文字化
 *
 * PARAMETERS:      Name       RW  Usage
 *   char*          pc_dst     W   編集先
 *   char*          pc_src     R   元文字列
 *
 * RETURNS:
 *   uint32_t:小文字に変換した文字数
 *
 ******************************************************************************/
uint32_t u32_vutil_lower_case(char* pc_dst, const char* pc_src) {
    // 入力チェック
    if (pc_dst == NULL || pc_src == NULL) {
        return 0;
    }
    // 文字列長
    int i_len = strlen(pc_src);
    // 変換文字数
    uint32_t u32_cnt = 0;
    // 小文字化処理
    int i_add = 'A' - 'a';
    int i_idx;
    for (i_idx = 0; i_idx < i_len; i_idx++) {
        pc_dst[i_idx] = pc_src[i_idx];
        // 大文字化判定
        if (pc_dst[i_idx] >= 'A' && pc_dst[i_idx] <= 'Z') {
            pc_dst[i_idx] -= i_add;
            u32_cnt++;
        }
    }
    // 終端文字を挿入
    pc_dst[i_len] = '\0';
    // 変換文字数を返却
    return u32_cnt;
}

/*******************************************************************************
 *
 * NAME: i_vutil_base64_encode
 *
 * DESCRIPTION:バイナリからBase64
 *
 * PARAMETERS:      Name        RW  Usage
 *   char*          pc_dst      W   編集先
 *   uint8_t*       pu8_src     R   元データ
 *   uint32_t       u32_len     R   元データ長
 *
 * RETURNS:
 *   int:変換後の文字数
 *
 ******************************************************************************/
int i_vutil_base64_encode(char* pc_dst, const uint8_t* pu8_src, const uint32_t u32_len) {
    // 3文字ずつ処理
    uint32_t u32_mod = u32_len % 3;
    uint32_t u32_mod_len = u32_len - u32_mod;
    uint32_t u32_buff;
    int i_dst_idx = 0;
    uint32_t u32_idx = 0;
    while (u32_idx < u32_mod_len) {
        u32_buff = pu8_src[u32_idx++];
        u32_buff = (u32_buff << 8) | pu8_src[u32_idx++];
        u32_buff = (u32_buff << 8) | pu8_src[u32_idx++];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff >> 18) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff >> 12) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff >> 6) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[u32_buff & 0x3F];
    }
    // 終端の処理
    if (u32_mod == 1) {
        pc_dst[i_dst_idx++] = STR_BASE64[(pu8_src[u32_idx] >> 2) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[(pu8_src[u32_idx] << 4) & 0x3F];
        pc_dst[i_dst_idx++] = '=';
        pc_dst[i_dst_idx++] = '=';
    } else if (u32_mod == 2) {
        u32_buff = pu8_src[u32_idx++];
        u32_buff = (u32_buff << 8) | pu8_src[u32_idx];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff >> 10) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff >> 4) & 0x3F];
        pc_dst[i_dst_idx++] = STR_BASE64[(u32_buff << 2) & 0x3F];
        pc_dst[i_dst_idx++] = '=';
    }
    // 終端文字を編集
    pc_dst[i_dst_idx] = '\0';
    // 変換後の文字数を返却
    return i_dst_idx;
}

/*******************************************************************************
 *
 * NAME: i_vutil_base64_decode
 *
 * DESCRIPTION:変換関数：Base64からバイナリ
 *
 * PARAMETERS:      Name       RW  Usage
 *   uint8_t*       pu8_dst    W   編集先
 *   char*          pc_src     R   元文字列
 *
 * RETURNS:
 *   int:変換後の文字数、変換不能時は-1
 *
 ******************************************************************************/
int i_vutil_base64_decode(uint8_t* pu8_dst, const char* pc_src) {
    //==========================================================================
    // 文字数チェック
    //==========================================================================
    if (pc_src == NULL) {
        return -1;
    }
    int i_len = strlen(pc_src);
    if (i_len == 0) {
        return 0;
    }
    if ((i_len % 4) != 0) {
        return -1;
    }

    //==========================================================================
    // デコード
    //==========================================================================
    uint32_t u32_buff;
    int i_dst_idx = 0;
    int i_base64_idx;
    uint32_t u32_idx;
    for (u32_idx = 0; pc_src[u32_idx] != '\0'; u32_idx += 4) {
        // base64インデックス
        i_base64_idx = i_vutil_base64_char_index(pc_src[u32_idx]);
        if (i_base64_idx < 0) {
            break;
        }
        u32_buff = i_base64_idx & 0x3F;
        // base64インデックス
        i_base64_idx = i_vutil_base64_char_index(pc_src[u32_idx + 1]);
        if (i_base64_idx < 0) {
            break;
        }
        u32_buff = (u32_buff << 6) | (i_base64_idx & 0x3F);
        // base64インデックス
        i_base64_idx = i_vutil_base64_char_index(pc_src[u32_idx + 2]);
        if (i_base64_idx < 0) {
            break;
        }
        u32_buff = (u32_buff << 6) | (i_base64_idx & 0x3F);
        // base64インデックス
        i_base64_idx = i_vutil_base64_char_index(pc_src[u32_idx + 3]);
        if (i_base64_idx < 0) {
            break;
        }
        u32_buff = (u32_buff << 6) | (i_base64_idx & 0x3F);

        // 結果編集
        pu8_dst[i_dst_idx++] = (u32_buff >> 16);
        // 終端判定
        if (pc_src[u32_idx + 3] != '=') {
            pu8_dst[i_dst_idx++] = (u32_buff >> 8);
            pu8_dst[i_dst_idx++] = u32_buff;
            continue;
        }
        if (pc_src[u32_idx + 2] != '=') {
            pu8_dst[i_dst_idx++] = (u32_buff >> 8);
        }
        // 終端と判断
        u32_idx += 4;
        break;
    }
    // 終端判定
    if (pc_src[u32_idx] != '\0') {
        // エラー
        return -1;
    }
    // 結果返却
    return i_dst_idx;
}

/*******************************************************************************
 *
 * NAME: u8_vutil_masking
 *
 * DESCRIPTION:マスキング処理
 *
 * PARAMETERS:      Name       RW  Usage
 *   uint8_t        u8_val     R   マスキング対象トークン
 *   uint8_t*       pu8_mask   R   マスクとなるトークン
 *   uint8_t        u8_len     R   マスキングサイズ
 *
 * RETURNS:
 *   uint8_t:マスキング結果
 *
 ******************************************************************************/
uint8_t u8_vutil_masking(uint8_t u8_val,
                           const uint8_t* pu8_mask,
                           uint8_t u8_len) {
    // 入力チェック
    if (pu8_mask == NULL) {
        return 0;
    }
    // マスキング処理
    uint8_t u8_wk_val = u8_val;
    uint8_t u8_idx;
    for (u8_idx = 0; u8_idx < u8_len; u8_idx++) {
        u8_wk_val = u8_wk_val ^ pu8_mask[u8_idx];
    }
    return u8_wk_val;
}

/*******************************************************************************
 *
 * NAME: u32_vutil_masking
 *
 * DESCRIPTION:マスキング処理
 *
 * PARAMETERS:      Name       RW  Usage
 *   uint32_t       u32_val    R   マスキング対象値
 *   uint8_t*       pu8_mask   R   マスクとなるトークン
 *   uint8_t        u8_len     R   マスキングサイズ
 *
 * RETURNS:
 *   uint32_t:マスキング結果
 *
 ******************************************************************************/
uint32_t u32_vutil_masking(uint32_t u32_val,
                             const uint8_t* pu8_mask,
                             uint8_t u8_len) {
    // 入力チェック
    if (pu8_mask == NULL) {
        return 0;
    }
    // マスキング処理
    uint32_t u32_wk_val = u32_val;
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < u8_len; u8Idx++) {
        u32_wk_val = u32_wk_val ^ ((uint32_t)pu8_mask[u8Idx] << (8 * (u8Idx % 4)));
    }
    return u32_wk_val;
}

/*******************************************************************************
 *
 * NAME: v_vutil_masking
 *
 * DESCRIPTION:マスキング処理
 *
 * PARAMETERS:      Name        RW  Usage
 *   uint8_t*       pu8_token   W   マスキング対象トークン
 *   uint8_t*       pu8_mask    R   マスクとなるトークン
 *   uint8_t        u8_len      R   マスキングサイズ
 *
 * RETURNS:
 *
 ******************************************************************************/
void v_vutil_masking(uint8_t* pu8_token,
                        const uint8_t* pu8_mask,
                        uint8_t u8_len) {
    // 入力チェック
    if (pu8_token == NULL || pu8_mask == NULL || u8_len == 0) {
        return;
    }
    // マスキング処理
    uint8_t u8Idx;
    for (u8Idx = 0; u8Idx < u8_len; u8Idx++) {
        pu8_token[u8Idx] = pu8_token[u8Idx] ^ pu8_mask[u8Idx];
    }
}

/*******************************************************************************
 *
 * NAME: d_vutil_entropy
 *
 * DESCRIPTION:エントロピー算出
 *   対象値を取得してカテゴリ分類し、該当カテゴリのカウンタをカウントアップする
 *   これを試算回数分繰り返して、カウンタ配列をカウントアップしたものから
 *   エントロピー数を算出する
 *
 * 例
 *   // １バイト単位の乱数のエントロピー算出例
 *   int i_list_size = 256;     // カテゴリ数（配列サイズ）は1バイト値の種類分
 *   int pi_list[i_list_size];  // 1バイト値の種類分のカウンタ配列
 *   int i_sample_size = 1000;  // 試算回数
 *   int i_cnt;
 *   for (i_cnt = 0; i_cnt < i_sample_size; i_cnt++) {
 *       pi_list[(uint8_t)u32_vutil_rand()]++;
 *   }
 *   double d_entropy = d_vutil_entropy(pi_list, i_list_size, i_sample_size);
 *
 * PARAMETERS:      Name            RW  Usage
 *   uint32_t*      u32_list        R   カテゴリ毎のカウンタ配列
 *   uint32_t       u32_list_size   R   カテゴリ数（配列サイズ）
 *   uint32_t       u32_sample_size R   試算回数
 *
 * RETURNS:
 *   0.0lf-1.0lfのエントロピー値、エラー時はマイナス値
 *
 ******************************************************************************/
double d_vutil_entropy(uint32_t* u32_list, uint32_t u32_list_size, uint32_t u32_sample_size) {
    // 入力チェック
    if (u32_list == NULL || u32_list_size == 0 || u32_sample_size == 0) {
        return -1.0f;
    }
    // エントロピー算出
    double d_entropy = 0;
    double d_val;
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt < u32_list_size; u32_cnt++) {
        d_val = (double)u32_list[u32_cnt] / u32_sample_size;
        if (d_val > 0) {
            d_entropy = d_entropy - d_val * log2(d_val);
        }
    }
    return d_entropy / 8;
}

//==============================================================================
// 計算関数
//==============================================================================

/*******************************************************************************
 *
 * NAME: u64_vutil_sqrt
 *
 * DESCRIPTION:計算関数：平方根（整数）
 *
 * PARAMETERS:      Name        RW  Usage
 *   uint64_t       u64_val     R   計算対象値
 *   bool           b_round_up  R   小数点以下切り上げ
 *
 * RETURNS:
 *   uint32_t:平方根の値
 *
 ******************************************************************************/
uint64_t u64_vutil_sqrt(uint64_t u64_val, bool b_round_up) {
    // 入力チェック
    if(u64_val == 0) {
        return 0;
    }
    // 平方根計算
    uint64_t u64_check = u64_val;
    uint64_t u64_sqrt;
    do {
        u64_sqrt  = u64_check;
        u64_check = (u64_check + (u64_val / u64_check)) / 2;
    } while (u64_sqrt > u64_check);
    // 小数点以下切り上げ
    if ((u64_sqrt != u64_check) && b_round_up) {
        u64_sqrt++;
    }
    // 結果返信
    return u64_sqrt;
}

/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
