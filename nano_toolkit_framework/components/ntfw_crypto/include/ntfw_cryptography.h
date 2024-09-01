/*******************************************************************************
 *
 * COMPONENT:Nano Toolkit Framework
 *
 * MODULE :cryptography library header file
 *
 * CREATED:2021/04/29 15:20:00
 * AUTHOR :Nakanohito
 *
 * DESCRIPTION:暗号に関する共通ライブラリ
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
#ifndef  __NTFW_CRYPTOGRAPHY_H__
#define  __NTFW_CRYPTOGRAPHY_H__


#if defined __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/
/** Curve25519 key size */
#define CRYPTO_X25519_KEY_SIZE                  (32)
/** Curve25519 public key size(Client) */
#define CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE    (36)
/** Curve25519 public key size(Server) */
#define CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE    (33)

/******************************************************************************/
/***      Include files                                                     ***/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <esp_system.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/md.h>
#include <mbedtls/aes.h>
#include <mbedtls/ecdh.h>
#include "ntfw_com_data_model.h"

/******************************************************************************/
/***      Macro Definitions                                                 ***/
/******************************************************************************/

/******************************************************************************/
/***      Type Definitions                                                  ***/
/******************************************************************************/
//==============================================================================
// 共通処理
//==============================================================================
/** 列挙型：暗号タイプ */
typedef enum {
    CRYPTO_TYPE_AES_128 = 0,    // AES 128bit
    CRYPTO_TYPE_AES_192,        // AES 192bit
    CRYPTO_TYPE_AES_256,        // AES 256bit
} te_crypto_type_t;

/** 構造体：共通鍵 */
typedef struct {
    ts_u8_array_t* ps_key;      // 共通鍵
    ts_u8_array_t* ps_key_iv;   // 初期ベクトル
    ts_u8_array_t* ps_nonce;    // ナンス
    ts_u8_array_t* ps_auth_iv;  // 認証タグ初期ベクトル
} ts_crypto_keyset_t;

/** 構造体：X25519鍵共有コンテキスト */
typedef struct {
    mbedtls_ecdh_context s_ecdh_ctx;    // ECDHコンテキスト
    uint8_t u8_cli_public_key[CRYPTO_X25519_CLIENT_PUBLIC_KEY_SIZE];  // 公開鍵（クライアント）
    uint8_t u8_svr_public_key[CRYPTO_X25519_SERVER_PUBLIC_KEY_SIZE];  // 公開鍵（サーバー）
    uint8_t u8_key[CRYPTO_X25519_KEY_SIZE];                           // 共有鍵
} ts_crypto_x25519_context_t;

//==============================================================================
// ハッシュ関数関連処理
//==============================================================================

/******************************************************************************/
/***      Exported Variables                                                ***/
/******************************************************************************/

/******************************************************************************/
/***      Exported Functions Prototypes                                     ***/
/******************************************************************************/

//==============================================================================
// 共通処理
//==============================================================================
/** 指定された文字セットの配列生成処理 */
extern ts_u8_array_t* ps_crypto_random_token(char* pc_charset, uint32_t u32_len);
/** 共通鍵の生成処理 */
extern ts_crypto_keyset_t* ps_crypto_create_keyset();
/** 共通鍵の削除処理 */
extern esp_err_t sts_crypto_delete_keyset(ts_crypto_keyset_t* ps_keyset);
/** パディング後のサイズ算出処理（PKCS#7） */
extern uint32_t u32_crypto_pkcs7_padded_length(uint32_t u32_size, uint8_t u8_block_size);
/** パディング処理（PKCS#7） */
extern esp_err_t sts_crypto_pkcs7_padding(uint8_t* pu8_edit, ts_u8_array_t* ps_data, uint8_t u8_block_size);
/** パディング処理（PKCS#7） */
extern ts_u8_array_t* ps_crypto_pkcs7_padding(ts_u8_array_t* ps_data, uint8_t u8_block_size);
/** アンパディング後のサイズ算出処理（PKCS#7） */
extern uint32_t u32_crypto_pkcs7_unpated_length(ts_u8_array_t* ps_data, uint8_t u8_block_size);
/** アンパディング処理（PKCS#7） */
extern esp_err_t sts_crypto_pkcs7_unpadding(uint8_t* pu8_edit, ts_u8_array_t* ps_data, uint8_t u8_block_size);
/** アンパディング処理（PKCS#7） */
extern ts_u8_array_t* ps_crypto_pkcs7_unpadding(ts_u8_array_t* ps_data, uint8_t u8_block_size);

//==============================================================================
// ハッシュ関数関連処理
//==============================================================================
/** ハッシュ関数(SHA1) */
extern esp_err_t sts_crypto_sha1(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash);
/** ハッシュ関数(SHA224) */
extern esp_err_t sts_crypto_sha224(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash);
/** ハッシュ関数(SHA256) */
extern esp_err_t sts_crypto_sha256(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash);
/** ハッシュ関数(SHA384) */
extern esp_err_t sts_crypto_sha384(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash);
/** ハッシュ関数(SHA512) */
extern esp_err_t sts_crypto_sha512(ts_u8_array_t* ps_token, uint32_t u32_stretching, uint8_t* pu8_hash);

//==============================================================================
// メッセージ認証符号
//==============================================================================
/** MAC関数 */
extern esp_err_t sts_crypto_mac(mbedtls_md_type_t e_type, ts_u8_array_t* ps_data, uint8_t* pu8_digest);
/** HMAC関数 */
extern esp_err_t sts_crypto_hmac(mbedtls_md_type_t e_type, ts_u8_array_t* ps_key, ts_u8_array_t* ps_data, uint8_t* pu8_digest);

//==============================================================================
// 共通鍵暗号
//==============================================================================
/** 暗号化処理(AES ECBモード) */
extern ts_u8_array_t* ps_crypto_aes_ecb_enc(const ts_u8_array_t* ps_key, const ts_u8_array_t* ps_plane);
/** 復号処理(AES ECBモード) */
extern ts_u8_array_t* ps_crypto_aes_ecb_dec(const ts_u8_array_t* ps_key, const ts_u8_array_t* ps_cipher);
/** 暗号化処理(AES CBCモード) */
extern ts_u8_array_t* ps_crypto_aes_cbc_enc(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_plane);
/** 復号処理(AES CBCモード) */
extern ts_u8_array_t* ps_crypto_aes_cbc_dec(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_cipher);
/** 暗号化処理(AES CTRモード) */
extern ts_u8_array_t* ps_crypto_aes_ctr(const ts_u8_array_t* ps_key,
                                         size_t* pt_nc_off,
                                         uint8_t* pu8_nonce_cnt,
                                         uint8_t* pu8_stream_blk,
                                         const ts_u8_array_t* ps_plane);
/** 暗号化処理(AES GCMモード) */
extern ts_u8_array_t* ps_crypto_aes_gcm_enc(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_plane, ts_u8_array_t* ps_auth_tag);
/** 復号処理(AES GCMモード) */
extern ts_u8_array_t* ps_crypto_aes_gcm_dec(const ts_crypto_keyset_t* ps_keyset, const ts_u8_array_t* ps_cipher, ts_u8_array_t* ps_auth_tag);

//==============================================================================
// 共通鍵共有(X25519)
//==============================================================================
/** X25519コンテキストの生成処理（クライアント側） */
extern ts_crypto_x25519_context_t* ps_crypto_x25519_client_context();
/** X25519コンテキストの生成処理（サーバー側） */
extern ts_crypto_x25519_context_t* ps_crypto_x25519_server_context(uint8_t* pu8_client_pub_key);
/** X25519の共通鍵の生成処理（クライアント側） */
extern esp_err_t sts_crypto_x25519_client_secret(ts_crypto_x25519_context_t* ps_client_ctx, uint8_t* pu8_server_pub_key);
/** X25519コンテキストの削除処理 */
extern void v_crypto_x25519_delete_context(ts_crypto_x25519_context_t* ps_ctx);

#if defined __cplusplus
}
#endif

#endif  /* __NTFW_CRYPTOGRAPHY_H__ */

/******************************************************************************/
/***      END OF FILE                                                       ***/
/******************************************************************************/
