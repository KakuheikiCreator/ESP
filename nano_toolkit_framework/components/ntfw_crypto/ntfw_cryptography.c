/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :common cryptography library source file
 *
 * CREATED:2021/04/29 15:20:00
 * AUTHOR :Kakuheiki.Nakanohito
 *
 * DESCRIPTION:暗号に関する共通ライブラリ
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
#include "ntfw_cryptography.h"

#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include <esp_random.h>
#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>
#include <mbedtls/sha512.h>
#include <mbedtls/gcm.h>
#include "ntfw_com_mem_alloc.h"
#include "ntfw_com_value_util.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
// Personalization data length
#define CRYPTO_PERSONAL_DATA_LEN    (16)

//==============================================================================
// SHA
//==============================================================================
// hash size:SHA1
#define CRYPTO_SHA1_SIZE    (20)
// hash size:SHA224
#define CRYPTO_SHA224_SIZE  (28)
// hash size:SHA256
#define CRYPTO_SHA256_SIZE  (32)
// hash size:SHA512
#define CRYPTO_SHA384_SIZE  (48)
// hash size:SHA512
#define CRYPTO_SHA512_SIZE  (64)

//==============================================================================
// ECDH
//==============================================================================
// CURVE25519 client public key size
#define CRYPTO_CURVE25519_CLIENT_PUBLIC_KEY_LEN (36)
// CURVE25519 server public key size
#define CRYPTO_CURVE25519_SERVER_PUBLIC_KEY_LEN (33)

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
/**
 * 疑似乱数生成器のコンテキスト
 */
typedef struct {
    mbedtls_entropy_context s_entropy_ctx;      // エントロピーコンテキスト
    mbedtls_ctr_drbg_context s_ctr_drbg_ctx;    // カウンタ使用の疑似乱数生成器のコンテキスト
} ts_crypto_drbg_context_t;

/**
 * ライブラリ初期処理
 */
typedef void (*v_crypto_init_t)();


/**
 * データ受信関数
 */
typedef void (*tf_ble_rx_data_t)(TickType_t t_tick);

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Local Variables                                                   ***/
/******************************************************************************/
/** ミューテックス（乱数生成器等を排他制御） */
static SemaphoreHandle_t s_mutex = NULL;
/** drbg context */
static ts_crypto_drbg_context_t s_drbg_context;

/******************************************************************************/
/***      Local Function Prototypes                                         ***/
/******************************************************************************/
/** ライブラリ初期処理 */
static void v_crypto_init();
/** ライブラリ初期処理（ダミー関数） */
static void v_crypto_init_dmy();
/** 初期化関数 */
static v_crypto_init_t f_crypto_init = v_crypto_init;

/** drbg transaction start */
static esp_err_t sts_drbg_transaction_start();
/** drbg transaction end */
static void v_drbg_transaction_end();


/** Entropy source with hardware random numbers */
static int i_entropy_source_hw_random(void* pv_data, unsigned char* puc_output, size_t t_len, size_t* pt_outpu_len);

/******************************************************************************/
/***      Exported Functions                                                ***/
/******************************************************************************/

//==============================================================================
// 共通処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: ps_crypto_create_keyset
 *
 * DESCRIPTION:指定された文字セットから、ランダム文字列の生成処理
 *
 * PARAMETERS:  Name            RW  Usage
 * char*        pc_charset      R   文字セット
 * uint32_t     u32_len         R   生成文字数
 *
 * RETURNS:
 * ts_crypto_keyset*:生成された共通鍵へのポインタ
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_random_token(char* pc_charset, uint32_t u32_len) {
    if (pc_charset == NULL) {
        return NULL;
    }
    // 配列情報の生成
    ts_u8_array_t* ps_array = (ts_u8_array_t*)pv_mem_malloc(sizeof(ts_u8_array_t));
    if (ps_array == NULL) {
        return NULL;
    }
    ps_array->b_clone = true;
    ps_array->t_size  = u32_len;
    ps_array->pu8_values = pv_mem_malloc(u32_len);
    if (ps_array->pu8_values == NULL) {
        l_mem_free(ps_array);
        return NULL;
    }
    // 乱数配列生成
    uint8_t* pu8_value = ps_array->pu8_values;
    b_vutil_set_u8_rand_array( pu8_value, u32_len);
    // 乱数文字列編集
    uint32_t u32_ch_idx;
    uint32_t u32_str_len = strlen(pc_charset);
    uint32_t u32_idx;
    for (u32_idx = 0; u32_idx < u32_len; u32_idx++) {
        u32_ch_idx =  pu8_value[u32_idx] % u32_str_len;
        pu8_value[u32_idx] = pc_charset[u32_ch_idx];
    }
    // 結果返却
    return ps_array;

}

/*******************************************************************************
 *
 * NAME: ps_crypto_create_keyset
 *
 * DESCRIPTION:共通鍵の生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * ts_crypto_keyset*:生成された共通鍵へのポインタ
 *
 ******************************************************************************/
ts_crypto_keyset_t* ps_crypto_create_keyset() {
    // 共通鍵のメモリを確保
    ts_crypto_keyset_t* ps_keyset = pv_mem_malloc(sizeof(ts_crypto_keyset_t));
    if (ps_keyset == NULL) {
        return NULL;
    }
    // 生成した共通鍵を初期化
    ps_keyset->ps_key      = NULL;  // 共通鍵
    ps_keyset->ps_key_iv   = NULL;  // 初期ベクトル
    ps_keyset->ps_nonce    = NULL;  // ナンス
    ps_keyset->ps_auth_iv  = NULL;  // 認証タグ初期ベクトル
    // 生成した共通鍵を返却
    return ps_keyset;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_delete_keyset
 *
 * DESCRIPTION:共通鍵の削除処理
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_crypto_keyset_t*  ps_keyset   RW  生成された共通鍵へのポインタ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_delete_keyset(ts_crypto_keyset_t* ps_keyset) {
    // 入力チェック
    if (ps_keyset == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // プロパティをクリア
    sts_mdl_delete_u8_array(ps_keyset->ps_key);     // 共通鍵
    sts_mdl_delete_u8_array(ps_keyset->ps_key_iv);  // 初期ベクトル
    sts_mdl_delete_u8_array(ps_keyset->ps_nonce);   // ナンス
    sts_mdl_delete_u8_array(ps_keyset->ps_auth_iv); // 認証タグ初期ベクトル
    // 生成した共通鍵を解放
    l_mem_free(ps_keyset);
    // 結果を返信
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: u32_crypto_pkcs7_padded_length
 *
 * DESCRIPTION:パディング後のサイズ算出処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * uint32_t         u32_size        R   パディング前のサイズ
 * uint8_t          u8_block_size   R   パディング時のブロックサイズ
 *
 * RETURNS:
 * uint32_t:パディング後のサイズ
 *
 ******************************************************************************/
uint32_t u32_crypto_pkcs7_padded_length(uint32_t u32_size, uint8_t u8_block_size) {
    // 入力チェック
    if (u8_block_size == 0) {
        return 0;
    }
    // ブロック数を算出
    uint32_t u32_block_size = (uint32_t)u8_block_size;
    uint32_t u32_block_cnt  = (u32_size / u32_block_size) + 1;
    // パディング後のサイズを算出
    return u32_block_size * u32_block_cnt;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_pkcs7_padding
 *
 * DESCRIPTION:パディング処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * uint8_t*         pu8_edit        W   編集対象
 * ts_u8_array_t*   ps_data         R   パディング対象
 * uint8_t          u8_block_size   R   ブロックサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_pkcs7_padding(uint8_t* pu8_edit, ts_u8_array_t* ps_data, uint8_t u8_block_size) {
    // 入力チェック
    if (pu8_edit == NULL || u8_block_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // データを編集
    uint32_t u32_size = 0;
    if (ps_data != NULL) {
        u32_size = ps_data->t_size;
        memcpy(pu8_edit, ps_data->pu8_values, u32_size);
    }
    // パディングサイズ等を算出
    uint32_t u32_blocks   = (u32_size / (uint32_t)u8_block_size) + 1;
    uint32_t u32_new_size = u32_blocks * (uint32_t)u8_block_size;
    uint32_t u8_padding   = u32_new_size - u32_size;
    // パディング
    uint32_t u32_idx;
    for (u32_idx = u32_size; u32_idx < u32_new_size; u32_idx++) {
        pu8_edit[u32_idx] = u8_padding;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_pkcs7_padding
 *
 * DESCRIPTION:パディング処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t*   ps_data         R   パディング対象
 * uint8_t          u8_block_size   R   ブロックサイズ
 *
 * RETURNS:
 *   ts_u8_array_t*:結果データ
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_pkcs7_padding(ts_u8_array_t* ps_data, uint8_t u8_block_size) {
    // 入力チェック
    if (u8_block_size == 0) {
        return NULL;
    }
    // データを編集
    uint32_t u32_size = 0;
    uint8_t* pu8_values = NULL;
    if (ps_data != NULL) {
        u32_size   = ps_data->t_size;
        pu8_values = ps_data->pu8_values;
    }
    // パディングサイズ等を算出
    uint32_t u32_blocks   = (u32_size / (uint32_t)u8_block_size) + 1;
    uint32_t u32_new_size = u32_blocks * (uint32_t)u8_block_size;
    uint32_t u8_padding   = u32_new_size - ps_data->t_size;
    // バイト配列生成
    ts_u8_array_t* ps_new_array = ps_mdl_empty_u8_array(u32_new_size);
    if (ps_new_array == NULL) {
        return NULL;
    }
    // データをコピー
    uint8_t* pu8_new_data = ps_new_array->pu8_values;
    memcpy(pu8_new_data, pu8_values, u32_size);
    // パディング
    uint32_t u32_idx;
    for (u32_idx = ps_data->t_size; u32_idx < u32_new_size; u32_idx++) {
        pu8_new_data[u32_idx] = u8_padding;
    }
    // 編集結果を返却
    return ps_new_array;
}

/*******************************************************************************
 *
 * NAME: u32_crypto_pkcs7_unpated_length
 *
 * DESCRIPTION:アンパディング後のサイズ算出処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t*   ps_data         R   アンパディング対象
 * uint8_t          u8_block_size   R   ブロックサイズ
 *
 * RETURNS:
 * uint32_t:アンパディング後のサイズ
 *
 ******************************************************************************/
uint32_t u32_crypto_pkcs7_unpated_length(ts_u8_array_t* ps_data, uint8_t u8_block_size) {
    // 入力チェック
    if (ps_data == NULL || u8_block_size == 0) {
        return 0;
    }
    if (ps_data->t_size < u8_block_size || (ps_data->t_size % u8_block_size) != 0) {
        return ps_data->t_size;
    }
    // アンパディング後サイズ等を算出
    uint8_t  u8_padding   = ps_data->pu8_values[ps_data->t_size - 1];
    if (u8_padding > u8_block_size || u8_padding < ps_data->t_size) {
        return ps_data->t_size;
    }
    uint32_t u32_new_size = ps_data->t_size - u8_padding;
    // パディングのチェック
    uint32_t u32_idx;
    for (u32_idx = u32_new_size; u32_idx < ps_data->t_size; u32_idx++) {
        if (ps_data->pu8_values[u32_idx] != u8_padding) {
            // パディングエラー
            return ps_data->t_size;
        }
    }
    // アンパディング後のサイズを返却
    return u32_new_size;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_pkcs7_unpadding
 *
 * DESCRIPTION:アンパディング処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * uint8_t*         pu8_edit        W   結果編集対象
 * ts_u8_array_t*   ps_data         R   パディング対象
 * uint8_t          u8_block_size   R   ブロックサイズ
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_pkcs7_unpadding(uint8_t* pu8_edit, ts_u8_array_t* ps_data, uint8_t u8_block_size) {
    // 入力チェック
    if (pu8_edit == NULL || ps_data == NULL || u8_block_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ps_data->t_size < u8_block_size || (ps_data->t_size % u8_block_size) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // アンパディング後サイズ等を算出
    uint8_t  u8_padding   = ps_data->pu8_values[ps_data->t_size - 1];
    uint32_t u32_new_size = ps_data->t_size - u8_padding;
    // パディングのチェック
    uint32_t u32_idx;
    for (u32_idx = u32_new_size; u32_idx < ps_data->t_size; u32_idx++) {
        if (ps_data->pu8_values[u32_idx] != u8_padding) {
            // パディングエラー
            return ESP_ERR_INVALID_ARG;
        }
    }
    // アンパディング処理
    memcpy(pu8_edit, ps_data->pu8_values, u32_new_size);
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_pkcs7_unpadding
 *
 * DESCRIPTION:アンパディング処理（PKCS#7）
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t*   ps_data         R   アンパディング対象
 * uint8_t          u8_block_size   R   ブロックサイズ
 *
 * RETURNS:
 *   ts_u8_array_t*:結果データ、パディングされていない場合にはクローンを返却
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_pkcs7_unpadding(ts_u8_array_t* ps_data, uint8_t u8_block_size) {
    // 入力チェック
    if (ps_data == NULL || u8_block_size == 0) {
        return NULL;
    }
    if (ps_data->t_size < u8_block_size || (ps_data->t_size % u8_block_size) != 0) {
        return NULL;
    }
    // アンパディング後サイズ等を算出
    uint8_t  u8_padding   = ps_data->pu8_values[ps_data->t_size - 1];
    uint32_t u32_new_size = ps_data->t_size - u8_padding;
    // パディングのチェック
    uint32_t u32_idx;
    for (u32_idx = u32_new_size; u32_idx < ps_data->t_size; u32_idx++) {
        if (ps_data->pu8_values[u32_idx] != u8_padding) {
            // パディングエラー
            return NULL;
        }
    }
    // アンパディング済みデータを確保
    ts_u8_array_t* ps_array = ps_mdl_empty_u8_array(u32_new_size);
    if (ps_array == NULL) {
        return NULL;
    }
    // アンパディング処理
    memcpy(ps_array->pu8_values, ps_data->pu8_values, u32_new_size);
    // 編集結果を返却
    return ps_array;
}

//==============================================================================
// ハッシュ関数関連処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: sts_crypto_sha1
 *
 * DESCRIPTION:ハッシュ関数(SHA1)
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t    ps_token        R   ハッシュ関数パラメータ
 * uint32_t         u32_stretching  R   ストレッチング回数
 * uint8_t*         pu8_hash        W   ハッシュ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_sha1(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash) {
    // 入力チェック
    if (ps_token == NULL || pu8_hash == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ハッシュ処理コンテキスト
    mbedtls_sha1_context s_sha1_ctx;
    // コンテキストの初期化
    mbedtls_sha1_init(&s_sha1_ctx);
    // 対象トークン
    uint8_t* pu8_token = ps_token->pu8_values;
    size_t t_size = ps_token->t_size;
    // ハッシュ値
    uint8_t u8_wk_hash[CRYPTO_SHA1_SIZE];
    // ストレッチングのループ
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt <= u32_stretching; u32_cnt++) {
        // 計算の開始
        if (mbedtls_sha1_starts(&s_sha1_ctx) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の計算
        if (mbedtls_sha1_update(&s_sha1_ctx, pu8_token, t_size) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の書き出し
        if (mbedtls_sha1_finish(&s_sha1_ctx, u8_wk_hash) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 対象の更新
        pu8_token = u8_wk_hash;
        t_size = CRYPTO_SHA1_SIZE;
    }

    // コンテキストの解放
    mbedtls_sha1_free(&s_sha1_ctx);

    // 結果判定
    if (sts_val == ESP_OK) {
        memcpy(pu8_hash, u8_wk_hash, t_size);
    }

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_sha224
 *
 * DESCRIPTION:ハッシュ関数(SHA224)
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t    ps_token        R   ハッシュ関数パラメータ
 * uint32_t         u32_stretching  R   ストレッチング回数
 * uint8_t*         pu8_hash        W   ハッシュ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_sha224(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash) {
    // 入力チェック
    if (ps_token == NULL || pu8_hash == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ハッシュ処理コンテキスト
    mbedtls_sha256_context s_sha256_ctx;
    // コンテキストの初期化
    mbedtls_sha256_init(&s_sha256_ctx);
    // 対象トークン
    uint8_t* pu8_token = ps_token->pu8_values;
    size_t t_size = ps_token->t_size;
    // ハッシュ値
    uint8_t u8_wk_hash[CRYPTO_SHA224_SIZE];
    // ストレッチングのループ
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt <= u32_stretching; u32_cnt++) {
        // 計算の開始(SHA256)
        if (mbedtls_sha256_starts(&s_sha256_ctx, 1) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の計算
        if (mbedtls_sha256_update(&s_sha256_ctx, pu8_token, t_size) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の書き出し
        if (mbedtls_sha256_finish(&s_sha256_ctx, u8_wk_hash) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 対象の更新
        pu8_token = u8_wk_hash;
        t_size = CRYPTO_SHA224_SIZE;
    }

    // コンテキストの解放
    mbedtls_sha256_free(&s_sha256_ctx);

    // 結果判定
    if (sts_val == ESP_OK) {
        memcpy(pu8_hash, u8_wk_hash, t_size);
    }

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_sha256
 *
 * DESCRIPTION:ハッシュ関数(SHA256)
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t    ps_token        R   ハッシュ関数パラメータ
 * uint32_t         u32_stretching  R   ストレッチング回数
 * uint8_t*         pu8_hash        W   ハッシュ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_sha256(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash) {
    // 入力チェック
    if (ps_token == NULL || pu8_hash == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ハッシュ処理コンテキスト
    mbedtls_sha256_context s_sha256_ctx;
    // コンテキストの初期化
    mbedtls_sha256_init(&s_sha256_ctx);
    // 対象トークン
    uint8_t* pu8_token = ps_token->pu8_values;
    size_t t_size = ps_token->t_size;
    // ハッシュ値
    uint8_t u8_wk_hash[CRYPTO_SHA256_SIZE];
    // ストレッチングのループ
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt <= u32_stretching; u32_cnt++) {
        // 計算の開始(SHA256)
        if (mbedtls_sha256_starts(&s_sha256_ctx, 0) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の計算
        if (mbedtls_sha256_update(&s_sha256_ctx, pu8_token, t_size) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の書き出し
        if (mbedtls_sha256_finish(&s_sha256_ctx, u8_wk_hash) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 対象の更新
        pu8_token = u8_wk_hash;
        t_size = CRYPTO_SHA256_SIZE;
    }

    // コンテキストの解放
    mbedtls_sha256_free(&s_sha256_ctx);

    // 結果判定
    if (sts_val == ESP_OK) {
        memcpy(pu8_hash, u8_wk_hash, t_size);
    }

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_sha384
 *
 * DESCRIPTION:ハッシュ関数(SHA384)
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t    ps_token        R   ハッシュ関数パラメータ
 * uint32_t         u32_stretching  R   ストレッチング回数
 * uint8_t*         pu8_hash        W   ハッシュ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_sha384(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash) {
    // 入力チェック
    if (ps_token == NULL || pu8_hash == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ハッシュ処理コンテキスト
    mbedtls_sha512_context s_sha512_ctx;
    // コンテキストの初期化
    mbedtls_sha512_init(&s_sha512_ctx);
    // 対象トークン
    uint8_t* pu8_token = ps_token->pu8_values;
    size_t t_size = ps_token->t_size;
    // ハッシュ値
    uint8_t u8_wk_hash[CRYPTO_SHA384_SIZE];
    // ストレッチングのループ
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt <= u32_stretching; u32_cnt++) {
        // 計算の開始(SHA256)
        if (mbedtls_sha512_starts(&s_sha512_ctx, 1) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の計算
        if (mbedtls_sha512_update(&s_sha512_ctx, pu8_token, t_size) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の書き出し
        if (mbedtls_sha512_finish(&s_sha512_ctx, u8_wk_hash) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 対象の更新
        pu8_token = u8_wk_hash;
        t_size = CRYPTO_SHA384_SIZE;
    }

    // コンテキストの解放
    mbedtls_sha512_free(&s_sha512_ctx);

    // 結果判定
    if (sts_val == ESP_OK) {
        memcpy(pu8_hash, u8_wk_hash, t_size);
    }

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_sha512
 *
 * DESCRIPTION:ハッシュ関数(SHA512)
 *
 * PARAMETERS:      Name            RW  Usage
 * ts_u8_array_t    ps_token        R   ハッシュ関数パラメータ
 * uint32_t         u32_stretching  R   ストレッチング回数
 * uint8_t*         pu8_hash        W   ハッシュ値
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_sha512(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash) {
    // 入力チェック
    if (ps_token == NULL || pu8_hash == NULL) {
        return ESP_FAIL;
    }
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // ハッシュ処理コンテキスト
    mbedtls_sha512_context s_sha512_ctx;
    // コンテキストの初期化
    mbedtls_sha512_init(&s_sha512_ctx);
    // 対象トークン
    uint8_t* pu8_token = ps_token->pu8_values;
    size_t t_size = ps_token->t_size;
    // ハッシュ値
    uint8_t u8_wk_hash[CRYPTO_SHA512_SIZE];
    // ストレッチングのループ
    uint32_t u32_cnt;
    for (u32_cnt = 0; u32_cnt <= u32_stretching; u32_cnt++) {
        // 計算の開始(SHA256)
        if (mbedtls_sha512_starts(&s_sha512_ctx, 0) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の計算
        if (mbedtls_sha512_update(&s_sha512_ctx, pu8_token, t_size) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // ハッシュ値の書き出し
        if (mbedtls_sha512_finish(&s_sha512_ctx, u8_wk_hash) != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 対象の更新
        pu8_token = u8_wk_hash;
        t_size = CRYPTO_SHA512_SIZE;
    }

    // コンテキストの解放
    mbedtls_sha512_free(&s_sha512_ctx);

    // 結果判定
    if (sts_val == ESP_OK) {
        memcpy(pu8_hash, u8_wk_hash, t_size);
    }

    // 結果返信
    return sts_val;
}

//==============================================================================
// メッセージ認証符号
//==============================================================================

/*******************************************************************************
 *
 * NAME: sts_crypto_mac
 *
 * DESCRIPTION:MAC関数
 *
 * PARAMETERS:          Name            RW  Usage
 * mbedtls_md_type_t    e_type          R   ハッシュアルゴリズム
 * ts_u8_array_t*       ps_data         R   メッセージ
 * uint8_t*             pu8_digest      W   メッセージダイジェスト
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_mac(mbedtls_md_type_t e_type,
                         ts_u8_array_t* ps_data,
                         uint8_t* pu8_digest) {
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // メッセージダイジェストコンテキストを生成
    mbedtls_md_context_t md_ctx;
    mbedtls_md_init(&md_ctx);
    do {
        // メッセージダイジェスト情報を生成
        const mbedtls_md_info_t* s_md_info = mbedtls_md_info_from_type(e_type);
        if (s_md_info == NULL) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // コンテキストを初期化
        if (mbedtls_md_setup(&md_ctx, s_md_info, 0) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // メッセージダイジェストコンテキストにアルゴリズムを設定
        if (mbedtls_md_setup(&md_ctx, s_md_info, 1) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // メッセージ認証の開始処理
        if (mbedtls_md_starts(&md_ctx) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 入力データをメッセージ認証に反映
        uint8_t* pu8_data    = ps_data->pu8_values;
        size_t t_data_length = ps_data->t_size;
        if (mbedtls_md_update(&md_ctx, pu8_data, t_data_length) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // MACを生成
        if (mbedtls_md_finish(&md_ctx, pu8_digest) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
    } while(false);
    // コンテキストを解放
    mbedtls_md_free(&md_ctx);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_hmac
 *
 * DESCRIPTION:HMAC関数
 *
 * PARAMETERS:          Name            RW  Usage
 * mbedtls_md_type_t    e_type          R   ハッシュアルゴリズム
 * ts_u8_array_t*       ps_key          R   キー
 * ts_u8_array_t*       ps_data         R   メッセージ
 * uint8_t*             pu8_digest      W   メッセージダイジェスト
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_hmac(mbedtls_md_type_t e_type,
                          ts_u8_array_t* ps_key,
                          ts_u8_array_t* ps_data,
                          uint8_t* pu8_digest) {
    // 結果ステータス
    esp_err_t sts_val = ESP_OK;
    // メッセージダイジェストコンテキストを生成
    mbedtls_md_context_t md_ctx;
    mbedtls_md_init(&md_ctx);
    do {
        // メッセージダイジェスト情報を生成
        const mbedtls_md_info_t* s_md_info = mbedtls_md_info_from_type(e_type);
        if (s_md_info == NULL) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // コンテキストを初期化
        if (mbedtls_md_setup(&md_ctx, s_md_info, 1) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // メッセージダイジェストコンテキストにアルゴリズムを設定
        if (mbedtls_md_setup(&md_ctx, s_md_info, 1) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // メッセージ認証の開始処理
        uint8_t* pu8_key    = ps_key->pu8_values;
        size_t t_key_length = ps_key->t_size;
        if (mbedtls_md_hmac_starts(&md_ctx, pu8_key, t_key_length) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // 入力データをメッセージ認証に反映
        uint8_t* pu8_data    = ps_data->pu8_values;
        size_t t_data_length = ps_data->t_size;
        if (mbedtls_md_hmac_update(&md_ctx, pu8_data, t_data_length) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
        // HMACを生成
        if (mbedtls_md_hmac_finish(&md_ctx, pu8_digest) != 0) {
            // エラー終了
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }
    } while(false);
    // コンテキストを解放
    mbedtls_md_free(&md_ctx);

    // 結果返信
    return sts_val;
}

//==============================================================================
// 共通鍵暗号
//==============================================================================

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_ecb_enc
 *
 * DESCRIPTION:暗号化処理(AES)
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_u8_array_t*       ps_key      R   共通鍵
 * const ts_u8_array_t* ps_plane    R   平文
 *
 * RETURNS:
 *   ts_u8_array_t*:暗号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_ecb_enc(const ts_u8_array_t* ps_key, const ts_u8_array_t* ps_plane) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 鍵情報
    if (ps_key == NULL || ps_plane == NULL) {
        return NULL;
    }
    // データサイズチェック
    if ((ps_plane->t_size % AES_BLOCK_BYTES) != 0) {
        return NULL;
    }

    //==========================================================================
    // 暗号化
    //==========================================================================
    // 暗号データの編集先を生成
    ts_u8_array_t* ps_cipher = NULL;
    // AESコンテキスト
    mbedtls_aes_context s_aes_ctx;
    // コンテキストの初期処理
    mbedtls_aes_init(&s_aes_ctx);
    do {
        // 共通鍵の設定
        uint32_t u32_keybits = ps_key->t_size * 8;
        if (mbedtls_aes_setkey_enc(&s_aes_ctx, ps_key->pu8_values, u32_keybits) != 0) {
            // キー設定エラー
            break;
        }
        // 暗号データの編集先を生成
        ps_cipher = ps_mdl_empty_u8_array(ps_plane->t_size);
        if (ps_cipher == NULL) {
            // 生成エラー
            break;
        }
        // 暗号化
        uint8_t* pu8_plane  = ps_plane->pu8_values;
        uint8_t* pu8_cipher = ps_cipher->pu8_values;
        uint32_t u32_idx;
        for (u32_idx = 0; u32_idx < ps_plane->t_size; u32_idx += AES_BLOCK_BYTES) {
            if (mbedtls_aes_crypt_ecb(&s_aes_ctx, MBEDTLS_AES_ENCRYPT, &pu8_plane[u32_idx], &pu8_cipher[u32_idx]) != 0) {
                // 暗号化エラー
                sts_mdl_delete_u8_array(ps_cipher);
                ps_cipher = NULL;
                break;
            }
        }
    } while(false);
    // コンテキストの解放
    mbedtls_aes_free(&s_aes_ctx);

    // 結果を返却
    return ps_cipher;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_ecb_dec
 *
 * DESCRIPTION:復号処理(AES)
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_u8_array_t*       ps_key      R   共通鍵
 * const ts_u8_array_t* ps_cipher   R   暗号文
 *
 * RETURNS:
 *   ts_u8_array_t*:暗号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_ecb_dec(const ts_u8_array_t* ps_key, const ts_u8_array_t* ps_cipher) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 鍵情報
    if (ps_key == NULL || ps_cipher == NULL) {
        return NULL;
    }
    // データサイズチェック
    if ((ps_cipher->t_size % AES_BLOCK_BYTES) != 0) {
        return NULL;
    }

    //==========================================================================
    // 復号処理
    //==========================================================================
    // 平文データの編集先を生成
    ts_u8_array_t* ps_plane = NULL;
    // AESコンテキスト
    mbedtls_aes_context s_aes_ctx;
    // コンテキストの初期処理
    mbedtls_aes_init(&s_aes_ctx);
    do {
        // 共通鍵の設定
        uint32_t u32_keybits = ps_key->t_size * 8;
        if (mbedtls_aes_setkey_dec(&s_aes_ctx, ps_key->pu8_values, u32_keybits) != 0) {
            // 復号鍵の設定エラー
            break;
        }
        // 平文データの編集先を生成
        ts_u8_array_t* ps_plane = ps_mdl_empty_u8_array(ps_cipher->t_size);
        if (ps_plane == NULL) {
            break;
        }
        // 復号
        uint8_t* pu8_cipher = ps_cipher->pu8_values;
        uint8_t* pu8_plane  = ps_plane->pu8_values;
        uint32_t u32_idx;
        for (u32_idx = 0; u32_idx < ps_plane->t_size; u32_idx += AES_BLOCK_BYTES) {
            if (mbedtls_aes_crypt_ecb(&s_aes_ctx, MBEDTLS_AES_DECRYPT, &pu8_cipher[u32_idx], &pu8_plane[u32_idx]) != 0) {
                // 復号エラー
                sts_mdl_delete_u8_array(ps_plane);
                ps_plane = NULL;
                break;
            }
        }
    } while(false);
    // コンテキストの解放
    mbedtls_aes_free(&s_aes_ctx);

    // 結果を返却
    return ps_plane;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_cbc_enc
 *
 * DESCRIPTION:暗号化処理(AES)
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_crypto_keyset_t*  ps_keyset   R   共通鍵セット
 * const ts_u8_array_t* ps_plane    R   平文
 *
 * RETURNS:
 *   ts_u8_array_t*:暗号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_cbc_enc(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_plane) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 鍵情報
    if (ps_keyset == NULL || ps_plane == NULL) {
        return NULL;
    }
    // IV
    ts_u8_array_t* ps_iv = ps_keyset->ps_key_iv;
    if (ps_iv == NULL) {
        return NULL;
    }
    // IVサイズ
    if (ps_iv->t_size != AES_BLOCK_BYTES) {
        return NULL;
    }
    // データサイズチェック
    if ((ps_plane->t_size % AES_BLOCK_BYTES) != 0) {
        return NULL;
    }

    //==========================================================================
    // 暗号化
    //==========================================================================
    // 暗号データの編集先を生成
    ts_u8_array_t* ps_cipher = NULL;
    // AESコンテキスト
    mbedtls_aes_context s_aes_ctx;
    // コンテキストの初期処理
    mbedtls_aes_init(&s_aes_ctx);
    do {
        // 共通鍵の設定
        ts_u8_array_t* ps_key = ps_keyset->ps_key;
        uint32_t u32_keybits   = ps_key->t_size * 8;
        if (mbedtls_aes_setkey_enc(&s_aes_ctx, ps_key->pu8_values, u32_keybits) != 0) {
            // キー設定エラー
            break;
        }
        // 暗号データの編集先を生成
        size_t t_size = ps_plane->t_size;
        ps_cipher = ps_mdl_empty_u8_array(t_size);
        if (ps_cipher == NULL) {
            // 編集先の生成エラー
            break;
        }
        // 暗号化
        uint8_t* pu8_iv     = ps_iv->pu8_values;
        uint8_t* pu8_plane  = ps_plane->pu8_values;
        uint8_t* pu8_cipher = ps_cipher->pu8_values;
        if (mbedtls_aes_crypt_cbc(&s_aes_ctx, MBEDTLS_AES_ENCRYPT, t_size, pu8_iv, pu8_plane, pu8_cipher) != 0) {
            // 暗号化エラー
            sts_mdl_delete_u8_array(ps_cipher);
            ps_cipher = NULL;
            break;
        }
    } while(false);
    // コンテキストの解放
    mbedtls_aes_free(&s_aes_ctx);
    // 結果を返却
    return ps_cipher;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_cbc_dec
 *
 * DESCRIPTION:復号処理(AES)
 *
 * PARAMETERS:          Name        RW  Usage
 * ts_crypto_keyset_t*  ps_keyset   R   共通鍵セット
 * const ts_u8_array_t* ps_cipher   R   暗号文
 *
 * RETURNS:
 *   ts_u8_array_t*:復号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_cbc_dec(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_cipher) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 鍵情報
    if (ps_keyset == NULL || ps_cipher == NULL) {
        return NULL;
    }
    // IV
    ts_u8_array_t* ps_iv = ps_keyset->ps_key_iv;
    if (ps_iv == NULL) {
        return NULL;
    }
    // IVサイズ
    if (ps_iv->t_size != AES_BLOCK_BYTES) {
        return NULL;
    }
    // データサイズチェック
    if ((ps_cipher->t_size % AES_BLOCK_BYTES) != 0) {
        return NULL;
    }

    //==========================================================================
    // 復号処理
    //==========================================================================
    // 暗号データの編集先を生成
    ts_u8_array_t* ps_plane = NULL;
    // AESコンテキスト
    mbedtls_aes_context s_aes_ctx;
    // コンテキストの初期処理
    mbedtls_aes_init(&s_aes_ctx);
    do {
        // 共通鍵の設定
        ts_u8_array_t* ps_key = ps_keyset->ps_key;
        uint32_t u32_keybits   = ps_key->t_size * 8;
        if (mbedtls_aes_setkey_dec(&s_aes_ctx, ps_key->pu8_values, u32_keybits) != 0) {
            // キー設定エラー
            break;
        }
        // 復号データの編集先を生成
        size_t t_size = ps_cipher->t_size;
        ps_plane = ps_mdl_empty_u8_array(t_size);
        if (ps_plane == NULL) {
            // 編集先の生成エラー
            break;
        }
        // 暗号化
        uint8_t* pu8_iv     = ps_iv->pu8_values;
        uint8_t* pu8_cipher = ps_cipher->pu8_values;
        uint8_t* pu8_plane  = ps_plane->pu8_values;
        if (mbedtls_aes_crypt_cbc(&s_aes_ctx, MBEDTLS_AES_DECRYPT, t_size, pu8_iv, pu8_cipher, pu8_plane) != 0) {
            // 暗号化エラー
            sts_mdl_delete_u8_array(ps_plane);
            ps_plane = NULL;
            break;
        }
    } while(false);
    // コンテキストの解放
    mbedtls_aes_free(&s_aes_ctx);
    // 結果を返却
    return ps_plane;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_ctr
 *
 * DESCRIPTION:暗号化と複合処理(AES CTRモード)
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_u8_array_t*       ps_key          R   共通鍵セット
 * size_t*              pt_nc_off       RW  ナンスオフセット    ※ NULLの場合は０
 * uint8_t*             pu8_nonce_cnt   RW  ナンスカウンタ      ※ NULLの場合は内部で定義
 * uint8_t*             pu8_stream_blk  W   ストリームブロック  ※ NULLの場合は内部で定義
 * const ts_u8_array_t* ps_input        R   処理対象
 *
 * RETURNS:
 *   ts_u8_array_t*:暗号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_ctr(const ts_u8_array_t* ps_key,
                                 size_t* pt_nc_off,
                                 uint8_t* pu8_nonce_cnt,
                                 uint8_t* pu8_stream_blk,
                                 const ts_u8_array_t* ps_input) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // 鍵情報
    if (ps_key == NULL || ps_input == NULL) {
        return NULL;
    }
    // オフセット
    size_t t_nc_offset = 0;
    size_t* pt_noffset = &t_nc_offset;
    if (pt_nc_off != NULL) {
        pt_noffset = pt_nc_off;
    }
    // ナンスカウンター
    uint8_t u8_nonce_counter[AES_BLOCK_BYTES];
    uint8_t* pu8_ncnt = u8_nonce_counter;
    if (pu8_nonce_cnt != NULL) {
        pu8_ncnt = pu8_nonce_cnt;
    }
    // ストリームブロック
    uint8_t u8_stream_block[AES_BLOCK_BYTES];
    uint8_t* pu8_sblk = u8_stream_block;
    if (pu8_stream_blk != NULL) {
        pu8_sblk = pu8_stream_blk;
    }

    //==========================================================================
    // 暗号化・復号処理
    //==========================================================================
    // 出力結果
    ts_u8_array_t* ps_output = ps_mdl_empty_u8_array(ps_input->t_size);
    if (ps_output == NULL) {
        return NULL;
    }
    // コンテキストの初期処理
    mbedtls_aes_context s_aes_ctx;
    mbedtls_aes_init(&s_aes_ctx);
    do {
        // 共通鍵の設定
        uint8_t* pu8_key     = ps_key->pu8_values;
        uint32_t u32_keybits = ps_key->t_size * 8;
        if (mbedtls_aes_setkey_enc(&s_aes_ctx, pu8_key, u32_keybits) != 0) {
            // 出力データクリア
            sts_mdl_delete_u8_array(ps_output);
            ps_output = NULL;
            break;
        }
        //  入力データ
        uint8_t* pu8_input  = ps_input->pu8_values;
        size_t t_len        = ps_input->t_size;
        //  出力データ
        uint8_t* pu8_output = ps_output->pu8_values;
        // 第１引数：暗号処理のコンテキスト
        // 第２引数：入力データ長
        // 第３引数：処理対象の位置（ブロック番号）
        // 第４引数：暗号化に使用するナンスと、XORしたブロック数
        // 第５引数：暗号化もしくは復号化に利用したストリームブロック？
        // 第６引数：入力データ
        // 第７引数：出力データ
        int i_ret = mbedtls_aes_crypt_ctr(&s_aes_ctx,
                                          t_len,
                                          pt_noffset,
                                          pu8_ncnt,
                                          pu8_sblk,
                                          pu8_input,
                                          pu8_output);
        if (i_ret != 0) {
            // 出力データクリア
            sts_mdl_delete_u8_array(ps_output);
            ps_output = NULL;
            break;
        }
    } while(false);
    // コンテキストの解放
    mbedtls_aes_free(&s_aes_ctx);

    // 結果返信
    return ps_output;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_gcm_enc
 *
 * DESCRIPTION:暗号化処理(AES GCMモード)
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_crypto_keyset_t*  ps_keyset       R   共通鍵セット
 * const ts_u8_array_t* ps_plane        R   平文
 * ts_u8_array_t*       ps_auth_tag     W   認証タグ
 *
 * RETURNS:
 *   ts_u8_array_t*:暗号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_gcm_enc(const ts_crypto_keyset_t* ps_keyset,
                                     const ts_u8_array_t* ps_plane,
                                     ts_u8_array_t* ps_auth_tag) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // キーセット
    if (ps_keyset == NULL) {
        return NULL;
    }
    // 共通鍵
    ts_u8_array_t* ps_com_key = ps_keyset->ps_key;
    if (ps_com_key == NULL) {
        return NULL;
    }
    uint8_t* pu8_key  = ps_com_key->pu8_values;
    size_t t_key_size = ps_com_key->t_size;
    if (t_key_size != AES_128_KEY_BYTES &&
        t_key_size != AES_192_KEY_BYTES &&
        t_key_size != AES_256_KEY_BYTES) {
        return NULL;
    }
    // 初期ベクトル
    ts_u8_array_t* ps_iv = ps_keyset->ps_key_iv;
    if (ps_iv == NULL) {
        return NULL;
    }
    uint8_t* pu8_iv = ps_iv->pu8_values;
    size_t t_iv_size = ps_iv->t_size;
    if (ps_iv->t_size != IV_BYTES) {
        return NULL;
    }
    // 認証タグ初期ベクトル
    ts_u8_array_t* ps_add = ps_keyset->ps_auth_iv;
    uint8_t* pu8_add   = NULL;
    size_t t_add_size  = 0;
    if (ps_add != NULL) {
        t_add_size = ps_add->t_size;
        if (t_add_size > 0) {
            pu8_add = ps_add->pu8_values;
        }
    }
    // 平文
    if (ps_plane == NULL) {
        return NULL;
    }
    uint8_t* pu8_input       = ps_plane->pu8_values;
    uint32_t u32_input_size  = ps_plane->t_size;
    // 認証タグ
    if (ps_auth_tag == NULL) {
        return NULL;
    }
    uint8_t* pu8_auth_tag  = ps_auth_tag->pu8_values;
    size_t t_auth_tag_size = ps_auth_tag->t_size;
    if (t_auth_tag_size < 4) {
        return NULL;
    }

    //==========================================================================
    // 暗号化処理
    //==========================================================================
    // 暗号データ確保
    ts_u8_array_t* ps_cipher = ps_mdl_empty_u8_array(u32_input_size);
    if (ps_cipher == NULL) {
        return NULL;
    }
    uint8_t* pu8_output = ps_cipher->pu8_values;
    size_t t_output_size = ps_cipher->t_size;

    // init the context...
    // 暗号処理に必要なコンテキストの初期化
    mbedtls_gcm_context s_gcm_ctx;
    mbedtls_gcm_init(&s_gcm_ctx);
    do {
        // Set the key. This next line could have CAMELLIA or ARIA as our GCM mode cipher...
        // キー情報を設定、第三引数に鍵のバイト配列、第四引数に鍵のビット数を指定
        int i_ret = mbedtls_gcm_setkey(&s_gcm_ctx, MBEDTLS_CIPHER_ID_AES, pu8_key, t_key_size * 8);
        if (i_ret != 0) {
            break;
        }
        // Initialise the GCM cipher...
        // 暗号利用モードとしてGCMモードを選択し、暗号化を開始
        i_ret = mbedtls_gcm_starts(&s_gcm_ctx, MBEDTLS_GCM_ENCRYPT, pu8_iv, t_iv_size);
        if (i_ret != 0) {
            break;
        }
        // 入力バッファの設定
        i_ret = mbedtls_gcm_update_ad(&s_gcm_ctx, pu8_add, t_add_size);
        if (i_ret != 0) {
            break;
        }
        // Send the intialised cipher some data and store it...
        // GCMモードで暗号化、末尾以外はブロックサイズ（16Byte）単位
        size_t t_result_size = 0;
        i_ret = mbedtls_gcm_update(&s_gcm_ctx, pu8_input, u32_input_size, pu8_output, t_output_size, &t_result_size);
        if (i_ret != 0) {
            break;
        }
        // 認証タグの生成
        size_t t_output_length;
        mbedtls_gcm_finish(&s_gcm_ctx, NULL, 0, &t_output_length, pu8_auth_tag, t_auth_tag_size);

    } while(false);
    // GCMモードでのコンテキストの解放
    mbedtls_gcm_free(&s_gcm_ctx);

    // 結果を返却
    return ps_cipher;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_aes_gcm_dec
 *
 * DESCRIPTION:復号処理(AES GCMモード)
 *
 * PARAMETERS:          Name            RW  Usage
 * ts_crypto_keyset_t*  ps_keyset       R   共通鍵セット
 * const ts_u8_array_t* ps_cipher       R   暗号文
 * ts_u8_array_t*       ps_auth_tagg    W   認証タグ
 *
 * RETURNS:
 *   ts_u8_array_t*:復号文
 *
 ******************************************************************************/
ts_u8_array_t* ps_crypto_aes_gcm_dec(const ts_crypto_keyset_t* ps_keyset,
                                     const ts_u8_array_t* ps_cipher,
                                     ts_u8_array_t* ps_auth_tag) {
    //==========================================================================
    // 入力チェック
    //==========================================================================
    // キーセット
    if (ps_keyset == NULL) {
        return NULL;
    }
    // 共通鍵
    ts_u8_array_t* ps_com_key = ps_keyset->ps_key;
    if (ps_com_key == NULL) {
        return NULL;
    }
    uint8_t* pu8_key  = ps_com_key->pu8_values;
    size_t t_key_size = ps_com_key->t_size;
    if (t_key_size != AES_128_KEY_BYTES &&
        t_key_size != AES_192_KEY_BYTES &&
        t_key_size != AES_256_KEY_BYTES) {
        return NULL;
    }
    // 初期ベクトル
    ts_u8_array_t* ps_iv = ps_keyset->ps_key_iv;
    if (ps_iv == NULL) {
        return NULL;
    }
    uint8_t* pu8_iv = ps_iv->pu8_values;
    size_t t_iv_size = ps_iv->t_size;
    if (ps_iv->t_size != IV_BYTES) {
        return NULL;
    }
    // 認証タグ初期ベクトル
    ts_u8_array_t* ps_add = ps_keyset->ps_auth_iv;
    uint8_t* pu8_add   = NULL;
    size_t t_add_size  = 0;
    if (ps_add != NULL) {
        t_add_size = ps_add->t_size;
        if (t_add_size > 0) {
            pu8_add = ps_add->pu8_values;
        }
    }
    // 暗号
    if (ps_cipher == NULL) {
        return NULL;
    }
    uint8_t* pu8_input       = ps_cipher->pu8_values;
    uint32_t u32_input_size  = ps_cipher->t_size;
    // 認証タグ
    if (ps_auth_tag == NULL) {
        return NULL;
    }
    uint8_t* pu8_auth_tag  = ps_auth_tag->pu8_values;
    size_t t_auth_tag_size = ps_auth_tag->t_size;
    if (t_auth_tag_size < 4) {
        return NULL;
    }

    //==========================================================================
    // 暗号化処理
    //==========================================================================
    // 暗号データ確保
    ts_u8_array_t* ps_plane = ps_mdl_empty_u8_array(u32_input_size);
    if (ps_plane == NULL) {
        return NULL;
    }
    uint8_t* pu8_output = ps_plane->pu8_values;
    size_t t_output_size = ps_plane->t_size;

    // init the context...
    // 暗号処理に必要なコンテキストの初期化
    mbedtls_gcm_context s_gcm_ctx;
    mbedtls_gcm_init(&s_gcm_ctx);
    do {
        // Set the key. This next line could have CAMELLIA or ARIA as our GCM mode cipher...
        // キー情報を設定、第三引数に鍵のバイト配列、第四引数に鍵のビット数を指定
        int i_ret = mbedtls_gcm_setkey(&s_gcm_ctx, MBEDTLS_CIPHER_ID_AES, pu8_key, t_key_size * 8);
        if (i_ret != 0) {
            break;
        }
        // Initialise the GCM cipher...
        // 暗号利用モードとしてGCMモードを選択し、復号を開始
        i_ret = mbedtls_gcm_starts(&s_gcm_ctx, MBEDTLS_GCM_DECRYPT, pu8_iv, t_iv_size);
        if (i_ret != 0) {
            break;
        }
        // 入力バッファの設定
        i_ret = mbedtls_gcm_update_ad(&s_gcm_ctx, pu8_add, t_add_size);
        if (i_ret != 0) {
            break;
        }
        // Send the intialised cipher some data and store it...
        // GCMモードで暗号化、末尾以外はブロックサイズ（16Byte）単位
        size_t t_result_size = 0;
        i_ret = mbedtls_gcm_update(&s_gcm_ctx, pu8_input, u32_input_size, pu8_output, t_output_size, &t_result_size);
        if (i_ret != 0) {
            break;
        }
        // 認証タグの生成
        size_t t_output_length;
        mbedtls_gcm_finish(&s_gcm_ctx, NULL, 0, &t_output_length, pu8_auth_tag, t_auth_tag_size);
    } while(false);
    // GCMモードでのコンテキストの解放
    mbedtls_gcm_free(&s_gcm_ctx);

    // 結果を返却
    return ps_plane;
}


//==============================================================================
// ECDH関連処理
//==============================================================================

/*******************************************************************************
 *
 * NAME: ps_crypto_x25519_client_context
 *
 * DESCRIPTION:ECDHコンテキストの生成処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *   ts_crypto_ecdh_context_t*:ECDHコンテキスト
 *
 ******************************************************************************/
ts_crypto_x25519_context_t* ps_crypto_x25519_client_context() {
    //==========================================================================
    // 初期処理
    //==========================================================================
    v_crypto_init_t f_init = f_crypto_init;
    f_init();

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // 乱数生成器のトランザクション開始
    //==========================================================================
    if (sts_drbg_transaction_start() != ESP_OK) {
        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex);
        return NULL;
    }

    //==========================================================================
    // ECDHコンテキストの生成
    //==========================================================================
    ts_crypto_x25519_context_t* ps_ctx = NULL;
    do {
        //----------------------------------------------------------------------
        // X25519コンテキストの生成
        //----------------------------------------------------------------------
        // X25519コンテキストの確保
        ps_ctx = pv_mem_calloc(sizeof(ts_crypto_x25519_context_t));
        if (ps_ctx == NULL) {
            break;
        }
        // プロパティ初期化
        memset(ps_ctx->u8_svr_public_key, 0x00, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
        memset(ps_ctx->u8_key, 0x00, CRYPTO_X25519_KEY_SIZE);

        //----------------------------------------------------------------------
        // X25519コンテキストの初期化
        //----------------------------------------------------------------------
        // X25519コンテキストを初期化
        mbedtls_ecdh_init(&ps_ctx->s_ecdh_ctx);
        // X25519コンテキストに暗号方式を設定
        int i_ret = mbedtls_ecdh_setup(&ps_ctx->s_ecdh_ctx, MBEDTLS_ECP_DP_CURVE25519);
        if (i_ret != 0) {
            // コンテキストを解放
            v_crypto_x25519_delete_context(ps_ctx);
            ps_ctx = NULL;
            break;
        }

        //----------------------------------------------------------------------
        // X25519公開鍵を生成
        //----------------------------------------------------------------------
        // 公開鍵の生成
        size_t t_out_len;
        i_ret = mbedtls_ecdh_make_params(&ps_ctx->s_ecdh_ctx, &t_out_len,
                                         ps_ctx->u8_cli_public_key,
                                         CRYPTO_CURVE25519_CLIENT_PUBLIC_KEY_LEN,
                                         mbedtls_ctr_drbg_random,
                                         &s_drbg_context.s_ctr_drbg_ctx);
        if (i_ret != 0 || t_out_len != CRYPTO_CURVE25519_CLIENT_PUBLIC_KEY_LEN) {
            // コンテキストを解放
            v_crypto_x25519_delete_context(ps_ctx);
            ps_ctx = NULL;
            break;
        }
    } while(false);

    //==========================================================================
    // 乱数生成器のトランザクション終了
    //==========================================================================
    v_drbg_transaction_end();

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ps_ctx;
}

/*******************************************************************************
 *
 * NAME: ps_crypto_x25519_server_context
 *
 * DESCRIPTION:ECDHコンテキストの生成処理（サーバー側）
 *
 * PARAMETERS:      Name                RW  Usage
 * uint8_t*         pu8_client_pub_key  R   受信した公開鍵
 *
 * RETURNS:
 *   ts_crypto_ecdh_context_t*:ECDHコンテキスト
 *
 ******************************************************************************/
ts_crypto_x25519_context_t* ps_crypto_x25519_server_context(uint8_t* pu8_client_pub_key) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    v_crypto_init_t f_init = f_crypto_init;
    f_init();

    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (pu8_client_pub_key == NULL) {
        return NULL;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) != pdTRUE) {
        return NULL;
    }

    //==========================================================================
    // 乱数生成器のトランザクション開始
    //==========================================================================
    if (sts_drbg_transaction_start() != ESP_OK) {
        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex);
        return NULL;
    }

    //==========================================================================
    // X25519コンテキストの生成
    //==========================================================================
    ts_crypto_x25519_context_t* ps_ctx = NULL;
    do {
        //----------------------------------------------------------------------
        // X25519コンテキストの生成
        //----------------------------------------------------------------------
        // X25519コンテキストの確保
        ps_ctx = pv_mem_calloc(sizeof(ts_crypto_x25519_context_t));
        if (ps_ctx == NULL) {
            break;
        }
        // プロパティ初期化
        memset(ps_ctx->u8_svr_public_key, 0x00, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
        memset(ps_ctx->u8_key, 0x00, CRYPTO_X25519_KEY_SIZE);

        //----------------------------------------------------------------------
        // ECDHコンテキストの初期化
        //----------------------------------------------------------------------
        // ECDHコンテキストを初期化
        mbedtls_ecdh_init(&ps_ctx->s_ecdh_ctx);
        // 受信した公開鍵を読み込み
        const unsigned char* pc_client_pub_key = (const unsigned char*)pu8_client_pub_key;
        int i_ret = mbedtls_ecdh_read_params(&ps_ctx->s_ecdh_ctx,
                                             &pc_client_pub_key,
                                             pu8_client_pub_key + CRYPTO_CURVE25519_CLIENT_PUBLIC_KEY_LEN);
        if (i_ret != 0) {
            // コンテキストを解放
            v_crypto_x25519_delete_context(ps_ctx);
            ps_ctx = NULL;
            break;
        }

        //----------------------------------------------------------------------
        // X25519公開鍵を生成
        //----------------------------------------------------------------------
        size_t t_out_len;
        i_ret = mbedtls_ecdh_make_public(&ps_ctx->s_ecdh_ctx,
                                         &t_out_len,
                                         ps_ctx->u8_svr_public_key,
                                         CRYPTO_CURVE25519_SERVER_PUBLIC_KEY_LEN,
                                         mbedtls_ctr_drbg_random,
                                         &s_drbg_context.s_ctr_drbg_ctx);
        if (i_ret != 0 || t_out_len != CRYPTO_CURVE25519_SERVER_PUBLIC_KEY_LEN) {
            // コンテキストを解放
            v_crypto_x25519_delete_context(ps_ctx);
            ps_ctx = NULL;
            break;
        }

        //----------------------------------------------------------------------
        // 共通鍵の生成
        //----------------------------------------------------------------------
        i_ret = mbedtls_ecdh_calc_secret(&ps_ctx->s_ecdh_ctx,
                                         &t_out_len,
                                         ps_ctx->u8_key,
                                         CRYPTO_X25519_KEY_SIZE,
                                         mbedtls_ctr_drbg_random,
                                         &s_drbg_context.s_ctr_drbg_ctx);
        if (i_ret != 0) {
            // コンテキストを解放
            v_crypto_x25519_delete_context(ps_ctx);
            ps_ctx = NULL;
            break;
        }

        //----------------------------------------------------------------------
        // 受信した公開鍵を確保
        //----------------------------------------------------------------------
        memcpy(ps_ctx->u8_cli_public_key, pu8_client_pub_key, CRYPTO_CURVE25519_CLIENT_PUBLIC_KEY_LEN);

    } while(false);

    //==========================================================================
    // 乱数生成器のトランザクション終了
    //==========================================================================
    v_drbg_transaction_end();

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return ps_ctx;
}

/*******************************************************************************
 *
 * NAME: sts_crypto_x25519_client_secret
 *
 * DESCRIPTION:ECDHの共通鍵の生成処理
 *
 * PARAMETERS:                  Name                RW  Usage
 * ts_crypto_ecdh_context_t*    ps_client_ctx       W   ECDHコンテキスト（クライアント側）
 * uint8_t*                     pu8_server_pub_key  R   受信した公開鍵
 *
 * RETURNS:
 *   esp_err_t:結果ステータス
 *
 ******************************************************************************/
esp_err_t sts_crypto_x25519_client_secret(ts_crypto_x25519_context_t* ps_client_ctx,
                                          uint8_t* pu8_server_pub_key) {
    //==========================================================================
    // 初期処理
    //==========================================================================
    v_crypto_init_t f_init = f_crypto_init;
    f_init();

    //==========================================================================
    // 入力チェック
    //==========================================================================
    if (ps_client_ctx == NULL || pu8_server_pub_key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //==========================================================================
    // クリティカルセクション開始
    //==========================================================================
    if (xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    //==========================================================================
    // 乱数生成器のトランザクション開始
    //==========================================================================
    if (sts_drbg_transaction_start() != ESP_OK) {
        // クリティカルセクション終了
        xSemaphoreGiveRecursive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // ECDHの共通鍵の生成処理
    //==========================================================================
    esp_err_t sts_val = ESP_OK;
    do {
        //----------------------------------------------------------------------
        // ECDHコンテキストに受信した公開鍵を追加
        //----------------------------------------------------------------------
        int i_ret = mbedtls_ecdh_read_public(&ps_client_ctx->s_ecdh_ctx,
                                             pu8_server_pub_key,
                                             CRYPTO_CURVE25519_SERVER_PUBLIC_KEY_LEN);
        if (i_ret != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 共通鍵を生成して、ECDHコンテキストに設定
        //----------------------------------------------------------------------
        size_t t_out_len;
        i_ret = mbedtls_ecdh_calc_secret(&ps_client_ctx->s_ecdh_ctx,
                                         &t_out_len,
                                         ps_client_ctx->u8_key,
                                         CRYPTO_X25519_KEY_SIZE,
                                         mbedtls_ctr_drbg_random,
                                         &s_drbg_context.s_ctr_drbg_ctx);
        if (i_ret != 0) {
            sts_val = ESP_ERR_INVALID_STATE;
            break;
        }

        //----------------------------------------------------------------------
        // 受信した公開鍵を確保
        //----------------------------------------------------------------------
        memcpy(ps_client_ctx->u8_svr_public_key, pu8_server_pub_key, CRYPTO_CURVE25519_SERVER_PUBLIC_KEY_LEN);

    } while(false);

    //==========================================================================
    // 乱数生成器のトランザクション終了
    //==========================================================================
    v_drbg_transaction_end();

    //==========================================================================
    // クリティカルセクション終了
    //==========================================================================
    xSemaphoreGiveRecursive(s_mutex);

    // 結果返信
    return sts_val;
}

/*******************************************************************************
 *
 * NAME: v_crypto_x25519_delete_context
 *
 * DESCRIPTION:ECDHコンテキストの削除処理
 *
 * PARAMETERS:                  Name    RW  Usage
 * ts_crypto_ecdh_context_t*    ps_ctx  W   削除対象のコンテキスト
 *
 * RETURNS:
 *
 ******************************************************************************/
void v_crypto_x25519_delete_context(ts_crypto_x25519_context_t* ps_ctx) {
    // 入力チェック
    if (ps_ctx == NULL) {
        return;
    }
    // コンテキストの削除
    mbedtls_ecdh_free(&ps_ctx->s_ecdh_ctx);
    // プロパティクリア
    memset(ps_ctx->u8_cli_public_key, 0x00, CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE);
    memset(ps_ctx->u8_svr_public_key, 0x00, CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE);
    memset(ps_ctx->u8_key, 0x00, CRYPTO_X25519_KEY_SIZE);
    // メモリ解放
    l_mem_free(ps_ctx);
}


/******************************************************************************/
/***      Local Functions                                                   ***/
/******************************************************************************/

/*******************************************************************************
 *
 * NAME: v_crypto_init
 *
 * DESCRIPTION:ライブラリ初期処理
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * RETURNS:
 *
 ******************************************************************************/
static void v_crypto_init() {
    // ミューテックスを生成
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutex();
    }
    // 初期化関数を初期化
    f_crypto_init = v_crypto_init_dmy;
}

/*******************************************************************************
 *
 * NAME: v_crypto_init_dmy
 *
 * DESCRIPTION:ライブラリ初期処理（ダミー関数）
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * RETURNS:
 *
 ******************************************************************************/
static void v_crypto_init_dmy(){
}

/*******************************************************************************
 *
 * NAME: v_drbg_transaction_start
 *
 * DESCRIPTION:drbg transaction start
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * RETURNS:
 * esp_err_t:結果ステータス
 *
 ******************************************************************************/
static esp_err_t sts_drbg_transaction_start() {
    //==========================================================================
    // エントロピーソース設定
    //==========================================================================
    // エントロピーコンテキストの初期化
    mbedtls_entropy_init(&s_drbg_context.s_entropy_ctx);
    // エントロピーソース（ハードウェアソース）の設定
    int i_ret = mbedtls_entropy_add_source(&s_drbg_context.s_entropy_ctx,
                                           i_entropy_source_hw_random, NULL,
                                           32, MBEDTLS_ENTROPY_SOURCE_STRONG);
    if (i_ret != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    //==========================================================================
    // 乱数生成器の設定
    //==========================================================================
    // パーソナライズデータ（デバイス固有乱数データ）の生成
    uint8_t u8_personal_data[CRYPTO_PERSONAL_DATA_LEN];
    b_vutil_set_u8_rand_array(u8_personal_data,  CRYPTO_PERSONAL_DATA_LEN);
    // 疑似乱数生成器の初期処理
    mbedtls_ctr_drbg_init(&s_drbg_context.s_ctr_drbg_ctx);
    // 乱数シード生成
    i_ret = mbedtls_ctr_drbg_seed(&s_drbg_context.s_ctr_drbg_ctx,
                                  mbedtls_entropy_func,
                                  &s_drbg_context.s_entropy_ctx,
                                  u8_personal_data,
                                  CRYPTO_PERSONAL_DATA_LEN);
    if (i_ret != 0) {
        // エントロピーコンテキストを解放
        mbedtls_entropy_free(&s_drbg_context.s_entropy_ctx);
        // エラーステータス
        return ESP_ERR_INVALID_STATE;
    }
    // 正常終了
    return ESP_OK;
}

/*******************************************************************************
 *
 * NAME: v_drbg_transaction_end
 *
 * DESCRIPTION:drbg transaction end
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * RETURNS:
 *
 ******************************************************************************/
static void v_drbg_transaction_end() {
    // 乱数生成器のコンテキストを解放
    mbedtls_ctr_drbg_free(&s_drbg_context.s_ctr_drbg_ctx);
    // エントロピーコンテキストを解放
    mbedtls_entropy_free(&s_drbg_context.s_entropy_ctx);
}

/*******************************************************************************
 * NAME: i_entropy_source_hw_random
 *
 * DESCRIPTION:ハードウェア乱数によるエントロピーソース
 *
 * PARAMETERS:      Name            RW  Usage
 * void*            pv_data         R   ?
 * unsigned char*   puc_output      W   乱数の出力先
 * size_t           t_len           R   生成サイズ
 * size_t*          pt_outpu_len    R   生成サイズ
 *
 * RETURNS:
 * 0:正常終了
 *
 ******************************************************************************/
static int i_entropy_source_hw_random(void* pv_data,
                                       unsigned char* puc_output,
                                       size_t t_len,
                                       size_t* pt_outpu_len) {
    esp_fill_random(puc_output, t_len);
    *pt_outpu_len = t_len;
    return 0;
}

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
