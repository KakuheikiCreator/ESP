/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common value utility functions header file
 *
 * CREATED:2019/09/10 02:46:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:値の変換等の値に関する基本的なユーティリティ関数群
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
#ifndef  __NTFW_COM_VAL_UTIL_H__
#define  __NTFW_COM_VAL_UTIL_H__

#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <driver/gpio.h>


/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** bool:TRUE */
#define BOOL_TRUE        (1)
/** bool:FALSE */
#define BOOL_FALSE       (0)

/** max value:int */
#define MAX_VALUE_INT       (0x7FFFFFFF)
/** max value:long */
#define MAX_VALUE_LONG      (0x7FFFFFFFFFFFFFFF)
/** max value:int16_t */
#define MAX_VALUE_INT16     (0x7FFF)
/** max value:int32_t */
#define MAX_VALUE_INT32     (0x7FFFFFFF)
/** max value:int64_t */
#define MAX_VALUE_INT64     (0x7FFFFFFFFFFFFFFF)
/** max value:uint16_t */
#define MAX_VALUE_UINT16    (0xFFFF)
/** max value:uint32_t */
#define MAX_VALUE_UINT32    (0xFFFFFFFF)
/** max value:uint64_t */
#define MAX_VALUE_UINT64    (0xFFFFFFFFFFFFFFFF)

/** Conversion function(kilo) */
#define i_vutil_conv_to_kilo(val) (val * 1024)
/** Conversion function(mega) */
#define i_vutil_conv_to_mega(val) (val * 1024 * 1024)

/** 有効ピン判定 */
#define b_vutil_valid_pin(e_gpio_num) (e_gpio_num >= GPIO_NUM_0 && e_gpio_num < GPIO_NUM_MAX)
/** プルアップ設定値チェック */
#define b_vutil_valid_pullup(gpio_pullup) (gpio_pullup == GPIO_PULLUP_DISABLE || gpio_pullup == GPIO_PULLUP_ENABLE)

/** 変換関数：数値→BCD形式 */
#define u8_vutil_binary_to_bcd(u8_val) ((uint8_t)(((u8_val / 10) << 4) + (u8_val % 10)))
/** 変換関数：BCD形式→数値 */
#define u8_vutil_bcd_to_binary(u8_val) ((uint8_t)((u8_val >> 4) * 10 + (u8_val & 0x0F)))

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/** 共用体：値の読み替え用 */
typedef union {
    int64_t  i64_value;
    int32_t  i32_values[2];
    int16_t  i16_values[4];
    int8_t   i8_values[8];
    uint64_t u64_value;
    uint32_t u32_values[2];
    uint16_t u16_values[4];
    uint8_t  u8_values[8];
    int      i_values[2];
} tu_type_converter_t;

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/
/** 数字文字列（10進数） */
extern const char STR_DEC_NUMBER[];
/** 数字文字列（16進数） */
extern const char STR_HEX_NUMBER[];
/** 英小文字列 */
extern const char STR_LOWERCASE[];
/** 英大文字列 */
extern const char STR_UPPERCASE[];
/** 英数小文字列 */
extern const char STR_LOWER_ALPHANUMERIC[];
/** 英数大文字列 */
extern const char STR_UPPER_ALPHANUMERIC[];
/** Base64 */
extern const char STR_BASE64[];

/******************************************************************************/
/***      Exported Function Prototypes                                      ***/
/******************************************************************************/
//==============================================================================
// チェック関数
//==============================================================================
/** GPIO番号チェック */
extern bool b_vutil_valid_gpio(gpio_num_t gpio_no);

//==============================================================================
// 情報関数
//==============================================================================
/** 10進数の桁数（int） */
extern int i_vutil_dec_len_i(int i_val);
/** 10進数の桁数（uint32_t） */
extern int i_vutil_dec_len_u32(uint32_t u32_val);
/** 16進数の桁数（int） */
extern int i_vutil_hex_len_i(int i_val);
/** 16進数の桁数（uint32_t） */
extern int i_vutil_hex_len_u32(uint32_t u32_val);
/** Base64の文字インデックス */
extern int i_vutil_base64_char_index(char c_char);
/** Base64変換後の桁数（int） */
extern int i_vutil_base64_len_i(int i_len);
/** Base64変換後の桁数（uint32_t） */
extern int i_vutil_base64_len_u32(uint32_t u32_len);
/** Byte配列に変換後の桁数（Base64） */
extern int i_vutil_byte_len_base64(const char* pc_src, const uint32_t u32_len);

//==============================================================================
// 文字列関連関数
//==============================================================================
/** NULL対応された文字列長取得(NULL値は0文字として扱う) */
extern int i_vutil_strlen(const char* pc_str);
/** NULL対応された文字列編集(NULL値はコピーしない) */
extern char* pc_vutil_strcpy(char* pc_str1, const char* pc_str2);
/** NULL対応された文字列比較(NULL値は最小値として扱う) */
extern int i_vutil_strcmp(const char* pc_str1, const char* pc_str2);
/** 配列上の文字インデックス探索（先頭） */
extern int i_vutil_index_of(const char* pc_str, char c_ch);
/** 配列上の文字インデックス探索（最終） */
extern int i_vutil_last_index_of(const char* pc_str, char c_ch);
/** 文字列の文字置き換え */
extern bool b_vutil_replace_char(char* pc_str, char c_ch, char c_rep);
/** 文字列切り出し */
extern int i_vutil_substr(char* pc_to, const char* pc_from,
                           uint32_t u32_pos, uint32_t u32_len);
/** 先頭からの文字列切り出し */
extern int i_vutil_str_left(char* pc_to, const char* pc_from, uint32_t u32_len);
/** 文字充填（後方） */
extern int i_vutil_str_rpad(char* pc_str, const char c_pad, uint32_t u32_len);
/** 文字充填（前方） */
extern int i_vutil_str_lpad(char* pc_str, const char c_pad, uint32_t u32_len);
/** 10進数文字列判定 */
extern bool b_vutil_dec_string(char* pc_num, uint32_t u32_max_len);
/** 16進数文字列判定 */
extern bool b_vutil_hex_string(char* pc_num, uint32_t u32_max_len);

//==============================================================================
// 生成関数
//==============================================================================
/** 生成関数：疑似乱数の合成で強化された乱数の生成処理 */
extern uint32_t u32_vutil_random();
/** 生成関数：乱数配列（uint8） */
extern bool b_vutil_set_u8_rand_array(uint8_t* pu8_rand_array, uint32_t u32_len);
/** 生成関数：乱数配列（uint32） */
extern bool b_vutil_set_u32_rand_array(uint32_t* pu32_rand_array, uint32_t u32_len);
/** 生成関数：乱数文字列 */
extern bool b_vutil_set_rand_string(char* pc_rand_string,
                                     const char* pc_src_string,
                                     uint32_t u32_len);
/** 生成関数：乱数文字列（英数小文字） */
extern bool b_vutil_set_rand_lwr_alphanumeric(char* pc_rand_string,
                                               uint32_t u32_len);
/** 生成関数：乱数文字列（英数大文字） */
extern bool b_vutil_set_rand_upr_alphanumeric(char* pc_rand_string,
                                               uint32_t u32_len);

//==============================================================================
// 変換関数
//==============================================================================
/** 変換関数：数値→BCD形式 */
extern uint32_t u32_vutil_binary_to_bcd(uint32_t u32_val);
/** 変換関数：BCD形式→数値 */
extern uint32_t u32_vutil_bcd_to_binary(uint32_t u32_val);
/** 変換関数：uint8→二進表現 */
extern uint32_t u32_vutil_u8_to_binary(uint8_t u8_val);
/** 変換関数：バイト配列から１６進文字列変換 */
extern void v_vutil_u8_to_hex_string(const uint8_t* pu8_data,
                                       uint32_t u32_data_length,
                                       char* pc_string);
/** 変換関数：数字文字列から数値変換 */
extern uint32_t u32_vutil_to_numeric(const char* pc_str);
/** 変換関数：数字文字列から数値変換 */
extern uint64_t u64_vutil_to_numeric(const char* pc_str);
/** 変換関数：数字配列から数値変換 */
extern uint32_t u32_vutil_array_to_u32(const char* pc_str,
                                        uint8_t u8_begin,
                                        uint8_t u8_length);
/** 変換関数：数字配列から数値変換 */
extern uint64_t u64_vutil_array_to_u64(const char* pc_str,
                                        uint8_t u8_begin,
                                        uint8_t u8_length);
/** 変換関数：数値から10進数文字列判定 */
extern bool b_vutil_edit_dec_string(char* pc_num, uint64_t u64_val);
/** 変換関数：数値から16進数文字列判定 */
extern bool b_vutil_edit_hex_string(char* pc_num, uint64_t u64_val);
/** 変換関数：大文字変換 */
extern uint32_t u32_vutil_upper_case(char* pc_dst, const char* pc_src);
/** 変換関数：小文字変換 */
extern uint32_t u32_vutil_lower_case(char* pc_dst, const char* pc_src);
/** 変換関数：バイナリからBase64 */
extern int i_vutil_base64_encode(char* pc_dst, const uint8_t* pu8_src, const uint32_t u32_len);
/** 変換関数：Base64からバイナリ */
extern int i_vutil_base64_decode(uint8_t* pu8_dst, const char* pc_src);

/** 変換関数：マスキング処理(uint8) */
extern uint8_t u8_vutil_masking(uint8_t u8_val,
                                 const uint8_t* pu8_mask,
                                 uint8_t u8_size);
/** 変換関数：マスキング処理(uint32) */
extern uint32_t u32_vutil_masking(uint32_t u32_val,
                                   const uint8_t* pu8_mask,
                                   uint8_t u8_size);
/** 変換関数：配列のマスキング処理 */
extern void v_vutil_masking(uint8_t* pu8_token,
                             const uint8_t* pu8_mask,
                             uint8_t u8_size);

/** 変換関数：エントロピー（０．０～１．０の値）の算出 */
extern double d_vutil_entropy(uint32_t* u32_list,
                                uint32_t u32_list_size,
                                uint32_t u32_sample_size);

//==============================================================================
// 計算関数
//==============================================================================
/** 計算関数：平方根（整数） */
extern uint64_t u64_vutil_sqrt(uint64_t u64_val, bool b_round_up);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_COM_VAL_UTIL_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
