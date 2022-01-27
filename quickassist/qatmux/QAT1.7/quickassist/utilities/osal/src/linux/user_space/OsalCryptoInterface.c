/**
 * @file OsalCryptoInterface.c (linux user space)
 *
 * @brief Osal interface to openssl crypto library.
 *
 * @par
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  version: QAT1.7.L.4.6.0-00025
 */

#include "Osal.h"
#ifdef USE_OPENSSL
#include <openssl/md5.h>
#include <openssl/sha.h>
#include <openssl/aes.h>
#else
#include "openssl/md5.h"
#include "openssl/sha.h"
#include "openssl/aes.h"
#endif

#ifdef USE_OPENSSL
#define INIT(TYPE) TYPE##_Init
#define TRANSFORM(TYPE) TYPE##_Transform
#define UPDATE(TYPE) TYPE##_Update
#define FINAL(TYPE) TYPE##_Final
#define OSAL_AES_SET_ENCRYPT AES_set_encrypt_key
#define OSAL_AES_ENCRYPT AES_encrypt
#else
#define INIT(TYPE) ossl_##TYPE##_Init
#define TRANSFORM(TYPE) ossl_##TYPE##_Transform
#define UPDATE(TYPE) ossl_##TYPE##_Update
#define FINAL(TYPE) ossl_##TYPE##_Final
#define OSAL_AES_SET_ENCRYPT ossl_AES_set_encrypt_key
#define OSAL_AES_ENCRYPT ossl_AES_encrypt
#endif

#define BYTE_TO_BITS_SHIFT 3

OSAL_STATUS
osalHashMD5(UINT8 *in, UINT8 *out)
{
    MD5_CTX ctx;
    if (!INIT(MD5)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(MD5)(&ctx, in);
    memcpy(out, &ctx, MD5_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashMD5Full(UINT8 *in, UINT8 *out, UINT32 len)
{
    MD5_CTX ctx;
    if (!INIT(MD5)(&ctx))
    {
        return OSAL_FAIL;
    }
    UPDATE(MD5)(&ctx, in, len);
    FINAL(MD5)(out, &ctx);
    memcpy(out, &ctx, MD5_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA1(UINT8 *in, UINT8 *out)
{
    SHA_CTX ctx;
    if (!INIT(SHA1)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(SHA1)(&ctx, in);
    memcpy(out, &ctx, SHA_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA1Full(UINT8 *in, UINT8 *out, UINT32 len)
{
    SHA_CTX ctx;
    UINT8 i = 0;
    if (!INIT(SHA1)(&ctx))
    {
        return OSAL_FAIL;
    }
    UPDATE(SHA1)(&ctx, in, len);
    FINAL(SHA1)(out, &ctx);
    memcpy(out, &ctx, SHA_DIGEST_LENGTH);
    /* Change output endianess for SHA1 algorithm */
    for (i = 0; i < (SHA_DIGEST_LENGTH >> 2); i++)
    {
        ((UINT32 *)(out))[i] = OSAL_HOST_TO_NW_32(((UINT32 *)(out))[i]);
    }
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA224(UINT8 *in, UINT8 *out)
{
    SHA256_CTX ctx;
    if (!INIT(SHA224)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(SHA256)(&ctx, in);
    memcpy(out, &ctx, SHA256_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA256(UINT8 *in, UINT8 *out)
{
    SHA256_CTX ctx;
    if (!INIT(SHA256)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(SHA256)(&ctx, in);
    memcpy(out, &ctx, SHA256_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA256Full(UINT8 *in, UINT8 *out, UINT32 len)
{
    SHA256_CTX ctx;
    UINT8 i = 0;
    if (!INIT(SHA256)(&ctx))
    {
        return OSAL_FAIL;
    }
    UPDATE(SHA256)(&ctx, in, len);
    FINAL(SHA256)(out, &ctx);
    memcpy(out, &ctx, SHA256_DIGEST_LENGTH);

    /* Change output endianess for SHA256 algorithm */
    for (i = 0; i < (SHA256_DIGEST_LENGTH >> 2); i++)
    {
        ((UINT32 *)(out))[i] = OSAL_HOST_TO_NW_32(((UINT32 *)(out))[i]);
    }
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA384(UINT8 *in, UINT8 *out)
{
    SHA512_CTX ctx;
    if (!INIT(SHA384)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(SHA512)(&ctx, in);
    memcpy(out, &ctx, SHA512_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA384Full(UINT8 *in, UINT8 *out, UINT32 len)
{
    /* We must use SHA512 for 384 context */
    SHA512_CTX ctx;
    UINT8 i = 0;

    if (!INIT(SHA384)(&ctx))
    {
        return OSAL_FAIL;
    }
    UPDATE(SHA384)(&ctx, in, len);
    FINAL(SHA384)(out, &ctx);
    memcpy(out, &ctx, SHA384_DIGEST_LENGTH);
    /* Change output endianess for SHA1 algorithm */
    for (i = 0; i < (SHA384_DIGEST_LENGTH >> 3); i++)
    {
        ((UINT64 *)(out))[i] = OSAL_HOST_TO_NW_64(((UINT64 *)(out))[i]);
    }
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA512(UINT8 *in, UINT8 *out)
{
    SHA512_CTX ctx;
    if (!INIT(SHA512)(&ctx))
    {
        return OSAL_FAIL;
    }
    TRANSFORM(SHA512)(&ctx, in);
    memcpy(out, &ctx, SHA512_DIGEST_LENGTH);
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalHashSHA512Full(UINT8 *in, UINT8 *out, UINT32 len)
{
    SHA512_CTX ctx;
    UINT16 i = 0;

    if (!INIT(SHA512)(&ctx))
    {
        return OSAL_FAIL;
    }
    UPDATE(SHA512)(&ctx, in, len);
    FINAL(SHA512)(out, &ctx);
    memcpy(out, &ctx, SHA512_DIGEST_LENGTH);
    /* Change output endianess for SHA512 algorithm */
    for (i = 0; i < (SHA512_DIGEST_LENGTH >> 3); i++)
    {
        ((UINT64 *)(out))[i] = OSAL_HOST_TO_NW_64(((UINT64 *)(out))[i]);
    }
    return OSAL_SUCCESS;
}

OSAL_STATUS
osalAESEncrypt(UINT8 *key, UINT32 keyLenInBytes, UINT8 *in, UINT8 *out)
{
    AES_KEY enc_key;
    INT32 status = OSAL_AES_SET_ENCRYPT(
        key, keyLenInBytes << BYTE_TO_BITS_SHIFT, &enc_key);
    if (status < 0)
    {
        return OSAL_FAIL;
    }
    OSAL_AES_ENCRYPT(in, out, &enc_key);
    return OSAL_SUCCESS;
}
