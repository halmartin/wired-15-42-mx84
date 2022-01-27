/******************************************************************************
 *
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
 *
 *****************************************************************************/

/**
 *****************************************************************************
 * @file lac_kpt.h
 *
 * @ingroupi Lac_Kpt
 *
 * @defgroup Lac_Kpt
 *
 * @description
 *           Secure Key Management Crypto Service -RSA, ECC and ECDSA Crypto
 *           service common include file
 *           This is the common include location for KPT crypto service
 *
 *****************************************************************************/
#ifndef __LAC_KPT_CRYPTO_QAT_COMMS_H__
#define __LAC_KPT_CRYPTO_QAT_COMMS_H__

#include "lac_mem_pools.h"

#define LAC_KPT_CIPHER_TEXT_SIZE_MAX LAC_BITS_TO_BYTES(LAC_MAX_OP_SIZE_IN_BITS)
#define LAC_KPT_HASH_TAG_SIZE_MAX (32)
#define LAC_KPT_SIZE_BYTES_MAX                                                 \
    (8 + LAC_KPT_HASH_TAG_SIZE_MAX + LAC_KPT_CIPHER_TEXT_SIZE_MAX)
/**< @ingroup LacKpt
 * Maximum KPT PKE operation in bits
 * In KPT mode, one PKE input data buffer is orgnized as below
 * -------------------------------------------------------------------------
 * |WPK Size(4B)|Reserved(4B)|      WPK Ciper Text(512B)   |      Tag(32B) |
 * -------------------------------------------------------------------------
 * The total size is 552 bytes
 */

#define LAC_KPT_POOL_SIZE_MAX (5)

#define LAC_KPT_ECDSA_POOL_SIZE (2)

/**< @ingroup LacKptCrypto
 * Macro to free all RSA Rep2 memory pool */

#define LacKptMemPoolArrayFree(poolArray, pool1, pool2, pool3, pool4, pool5)   \
    do                                                                         \
    {                                                                          \
        LacKptMemPoolFree(poolArray);                                          \
        LacKptMemPoolFree(pool1);                                              \
        LacKptMemPoolFree(pool2);                                              \
        LacKptMemPoolFree(pool3);                                              \
        LacKptMemPoolFree(pool4);                                              \
        LacKptMemPoolFree(pool5);                                              \
    } while (0);

#define LAC_KPT_EC_SIZE_BYTES_MAX (LAC_BITS_TO_BYTES(LAC_576_BITS) + 8)
/**< @ingroup LacKptCrypto
 * macro to define the max length of wrapped private key length in ecc case,
 * if the wrapping algorithm is gcm cbc case, private key will be padded to
 * a multiple of 16B */

/**
 ***************************************************************************
 * @ingroup LacKptCyQatComms
 *       Build a Kpt Crypto flat buffer which contains private key cipher
 * text and hash tag.
 *
 * @description
 *       This function build a kpt crypto input flat buffer which contains
 * wrapped private key' cipher text and hash tag, FW will paser it to get
 * necessary information.
 *
 * @param[in] pMempool       Pointer to a preallocated memory pool entry.
 * @param[in] wpkSize        Size of wrapped private key's cipher text..
 * @param[in] pUserFlatBuf   Pointer to a user input flat buffer which is a
 *                           combination of wpk cipher text and hash tag
 * @retval NULL
 ***************************************************************************/
void LacKpt_Crypto_BuildFlatBuf(Cpa8U *pMempool,
                                Cpa32U wpkSize,
                                const CpaFlatBuffer *pUserFlatBuf);

/**
 ***************************************************************************
 * @ingroup LacKptCyQatComms
 *       Allocate a memory pool entry from a spefic memory pool
 *
 * @description
 *       This function  apply for a memory pool entry from a pre-allocated
 * memory pool.
 *
 * @param[in]  poolID        Memory pool id of target memory pool.
 * @param[out] ppMempool     Pointer to the pointer of allocated memory entry.
 *
 *
 * @retval CPA_STATUS_SUCCESS     No error
 * @retval CPA_STATUS_RESOURCE    Resource error
 ***************************************************************************/
CpaStatus LacKptMemPoolMalloc(Cpa8U **ppMemPool, lac_memory_pool_id_t poolID);

/**
 ***************************************************************************
 * @ingroup LacKptCyQatComms
 *       Free a memory pool entry
 *
 * @description
 *       This function free a allocated memory pool entry.
 *
 * @param[in] pMempool     Pointer to allocated memory entry.
 *
 * @retval NULL
 ***************************************************************************/
void LacKptMemPoolFree(Cpa8U *pMemPool);

/**
******************************************************************************
* @ingroup LacKptCyQatComms
*      return the ECC service QW value for a given size.
*
* @description
*      This function will return the QWs value according to the size.
*
* @param[in]  size             input size
* @param[out] pMax             QW value returned according to size
*
* @retval CPA_STATUS_SUCCESS       Function executed successfully.
* @retval CPA_STATUS_INVALID_PARAM Size is invalid.  In this case, pMax will
*                                  be set to size
* @note   In kpt mode, private key is encrypted with a symmetric key, if
*         wrapping algorithm is AES-CBC, private key will be padded to a
*multiple of 16B, e.g. if size of clear private key is 65B, wrapped private key
*size will become 80B, so we enlarge the range of valid key size to
*LAC_KPT_EC_SIZE_BYTES_MAX(80B).
*****************************************************************************/
CpaStatus LacKptEc_GetRange(Cpa32U size, Cpa32U *pMax);

#endif
