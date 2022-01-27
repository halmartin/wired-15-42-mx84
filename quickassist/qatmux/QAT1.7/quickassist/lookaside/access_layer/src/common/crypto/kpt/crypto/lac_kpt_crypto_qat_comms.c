/***************************************************************************
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
 ***************************************************************************/

/**
 *****************************************************************************
 * @file lac_kpt_crypto_qat_comms.c
 *
 * @ingroup LacKptksp
 *
 * This file implements KPT crypto common function
 *
 *****************************************************************************/

/*
********************************************************************************
* Include public/global header files
********************************************************************************
*/
#include "cpa.h"

/*
****************************************************************************
* Include private header files
****************************************************************************
*/
/* SAL includes */
#include "lac_common.h"
#include "lac_mem_pools.h"
#include "lac_pke_utils.h"
#include "lac_kpt_crypto_qat_comms.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "sal_qat_cmn_msg.h"
#include "lac_ec.h"

/**
 ***************************************************************************
 * @ingroup LacKptCrypto
 *      Build a KPT input buffer according to wpk size and input buffer
 ***************************************************************************/
void LacKpt_Crypto_BuildFlatBuf(Cpa8U *pMemPool,
                                Cpa32U wpkSize,
                                const CpaFlatBuffer *pUserFlatBuf)
{
    Cpa8U *pTemp = pMemPool;
    Cpa32U padSize = 0;
    Cpa32U hashTagSize = 0;
    memset(pMemPool, 0, LAC_KPT_SIZE_BYTES_MAX);
    if (wpkSize > LAC_KPT_CIPHER_TEXT_SIZE_MAX)
    {
        wpkSize = LAC_KPT_CIPHER_TEXT_SIZE_MAX;
    }

    *((Cpa32U *)pTemp) = wpkSize;
    /*The first 4 bytes is wrapped private key size*/
    pTemp += 8;

    /*Memcpy private key cipher text to "cipher text field" */
    padSize = LAC_KPT_CIPHER_TEXT_SIZE_MAX - wpkSize;
    memcpy(pTemp + padSize, pUserFlatBuf->pData, wpkSize);
    /*Memcpy HASH to "HASH tag field" */
    if (pUserFlatBuf->dataLenInBytes > wpkSize)
    {
        hashTagSize = pUserFlatBuf->dataLenInBytes - wpkSize;
        if (hashTagSize > LAC_KPT_HASH_TAG_SIZE_MAX)
        {
            hashTagSize = LAC_KPT_HASH_TAG_SIZE_MAX;
        }
        pTemp += LAC_KPT_CIPHER_TEXT_SIZE_MAX;
        memcpy(pTemp, pUserFlatBuf->pData + wpkSize, hashTagSize);
    }
}

/**
 ***************************************************************************
 * @ingroup LacKptCrypto
 *      Allocate a Kpt memory pool according to pool id
 ***************************************************************************/
CpaStatus LacKptMemPoolMalloc(Cpa8U **ppMemPool, lac_memory_pool_id_t poolId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pMemPool = NULL;
    do
    {
        pMemPool = (Cpa8U *)Lac_MemPoolEntryAlloc(poolId);
        if (NULL == pMemPool)
        {
            LAC_LOG_ERROR("Cannot get mem pool entry");
            status = CPA_STATUS_RESOURCE;
        }
        else if ((void *)CPA_STATUS_RETRY == pMemPool)
        {
            osalYield();
        }
    } while ((void *)CPA_STATUS_RETRY == pMemPool);
    *ppMemPool = pMemPool;
    return status;
}

/**
 ***************************************************************************
 * @ingroup LacKptCrypto
 *     Free a Kpt memory pool
 ***************************************************************************/
void LacKptMemPoolFree(Cpa8U *pMemPool)
{
    if (NULL != pMemPool)
    {
        Lac_MemPoolEntryFree(pMemPool);
    }
}

CpaStatus LacKptEc_GetRange(Cpa32U size, Cpa32U *pMax)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    LAC_ASSERT_NOT_NULL(pMax);

    if (LAC_EC_SIZE_QW4_IN_BYTES >= size)
    {
        size = LAC_EC_SIZE_QW4_IN_BYTES;
    }
    else if (LAC_EC_SIZE_QW8_IN_BYTES >= size)
    {
        size = LAC_EC_SIZE_QW8_IN_BYTES;
    }
    else if (LAC_KPT_EC_SIZE_BYTES_MAX >= size)
    {
        size = LAC_EC_SIZE_QW9_IN_BYTES;
    }
    else
    {
        status = CPA_STATUS_INVALID_PARAM;
    }

    *pMax = size;

    return status;
}
