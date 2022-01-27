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
 ***************************************************************************
 * @file lac_kpt_dsa.c
 *
 * @ingroup Lac_KptDsa
 *
 * This file contains the implementation of DSA functions
 *
 ***************************************************************************/

/*
****************************************************************************
* Include public/global header files
****************************************************************************
*/

/* Include API files */
#include "cpa.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_kpt.h"

/*
****************************************************************************
* Include private header files
****************************************************************************
*/
/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp.h"
#include "icp_qat_fw_mmp_ids.h"

/* SAL includes */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "icp_adf_init.h"
#include "lac_sal.h"
#include "sal_service_state.h"
#include "lac_sal_ctrl.h"
#include "lac_pke_mmp.h"
#include "lac_common.h"
#include "lac_dsa.h"
#include "lac_hooks.h"
#include "lac_pke_qat_comms.h"
#include "lac_pke_utils.h"
#include "sal_statistics.h"
#include "lac_kpt_crypto_qat_comms.h"

#define KPT_DSA_RSSIGN_MASK (0x20)
/**< m ,k, p, q, g, x', b100000 */
#define KPT_DSA_SSIGN_MASK (0x10)
/**< m ,k, q, r, x', b10000 */
#define LAC_KPT_DSA_SIGNRS_NUM_OUT_ARGS (3)
/**< number of 'out' arguments in output arguments array for KPT DSA SIGNRS */
#define LAC_KPT_DSA_SIGNS_NUM_OUT_ARGS (2)
/**< number of 'out' arguments in output arguments array for KPT DSA SIGNS */
/*
****************************************************************************
* Global Variables
****************************************************************************
*/

/* Maps between operation sizes and PKE SIGN_S function ids */

static const Cpa32U lacKptDsaSignSSizeIdMap[][LAC_PKE_NUM_COLUMNS] = {
    {LAC_DSA_N_160, PKE_DSA_SIGN_S_160},
    {LAC_DSA_N_224, PKE_DSA_SIGN_S_224},
    {LAC_DSA_N_256, PKE_DSA_SIGN_S_256}};

/**<
 * Maps between operation size and PKE SIGN_RS function ids */

static const Cpa32U lacKptDsaSignRsSizeIdMap[][LAC_PKE_NUM_COLUMNS] = {
    {LAC_DSA_1024_160_PAIR, PKE_DSA_SIGN_R_S_1024_160},
    {LAC_DSA_2048_224_PAIR, PKE_DSA_SIGN_R_S_2048_224},
    {LAC_DSA_2048_256_PAIR, PKE_DSA_SIGN_R_S_2048_256},
    {LAC_DSA_3072_256_PAIR, PKE_DSA_SIGN_R_S_3072_256}};

/**< macro to initialize all DSA stats (stored in internal array of atomics) */
#define LAC_KPT_DSA_STATS_INIT(pCryptoService)                                 \
    do                                                                         \
    {                                                                          \
        Cpa32U i;                                                              \
                                                                               \
        for (i = 0; i < LAC_DSA_NUM_STATS; i++)                                \
        {                                                                      \
            osalAtomicSet(0, &(pCryptoService)->pLacDsaStatsArr[i]);           \
        }                                                                      \
    } while (0)

/* macro to increment a DSA stat (derives offset into array of atomics) */
#ifndef DISABLE_STATS
#define LAC_KPT_DSA_STAT_INC(statistic, pCryptoService)                        \
    do                                                                         \
    {                                                                          \
        if (CPA_TRUE ==                                                        \
            pCryptoService->generic_service_info.stats->bDsaStatsEnabled)      \
        {                                                                      \
            osalAtomicInc(                                                     \
                &(pCryptoService)                                              \
                     ->pLacDsaStatsArr[offsetof(CpaCyDsaStats64, statistic) /  \
                                       sizeof(Cpa64U)]);                       \
        }                                                                      \
    } while (0)
#else
#define LAC_KPT_DSA_STAT_INC(statistic, pCryptoService)
#endif

/*
****************************************************************************
* Define public/global function definitions
****************************************************************************
*/
#ifdef ICP_PARAM_CHECK
/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *     KPT DSA S Sign parameter check
 ***************************************************************************/
STATIC
CpaStatus LacKptDsaSSignParamCheck(const CpaCyDsaGenCbFunc pCb,
                                   const CpaCyDsaSSignOpData *pOpData,
                                   CpaBoolean *pProtocolStatus,
                                   CpaFlatBuffer *pS,
                                   CpaFlatBuffer *pKptUnwrapContext)
{
    /* check for valid callback function pointer */
    LAC_CHECK_NULL_PARAM(pCb);

    /* check for valid out buffer and data pointers */
    LAC_CHECK_NULL_PARAM(pS);
    LAC_CHECK_NULL_PARAM(pProtocolStatus);
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext);

    /* check parameters for null, 0 size, and LSB not set */
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->Z, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->K, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->Q, CHECK_NONE, 0);
    LAC_CHECK_ODD_PARAM(&pOpData->Q);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->R, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->X, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(pS, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(pKptUnwrapContext, CHECK_NONE, 0);

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      Given DSA the bit len of Q this function returns
 *       a value from the lac_dsa_n_values_t enum
 ***************************************************************************/

STATIC
lac_dsa_n_values_t LacKptDsa_GetN(Cpa32U sizeNBits)
{
    if (LAC_160_BITS == sizeNBits)
    {
        return LAC_DSA_N_160;
    }
    else if (LAC_224_BITS == sizeNBits)
    {
        return LAC_DSA_N_224;
    }
    else if (LAC_256_BITS == sizeNBits)
    {
        return LAC_DSA_N_256;
    }
    else
    {
        return LAC_DSA_N_INVALID;
    }
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      Given DSA {L,N} Pair (L is the bit len of P and N is the
 *       bit len of Q as defined in FIPS186-3) this function returns
 *       a value from the lac_dsa_ln_pair_t enum
 ***************************************************************************/

STATIC
lac_dsa_ln_pairs_t LacKptDsa_GetLNPair(Cpa32U opSizeLBits, Cpa32U opSizeNBits)
{
    if ((LAC_1024_BITS == opSizeLBits) && (LAC_160_BITS == opSizeNBits))
    {
        return LAC_DSA_1024_160_PAIR;
    }
    else if ((LAC_2048_BITS == opSizeLBits) && (LAC_224_BITS == opSizeNBits))
    {
        return LAC_DSA_2048_224_PAIR;
    }
    else if ((LAC_2048_BITS == opSizeLBits) && (LAC_256_BITS == opSizeNBits))
    {
        return LAC_DSA_2048_256_PAIR;
    }
    else if ((LAC_3072_BITS == opSizeLBits) && (LAC_256_BITS == opSizeNBits))
    {
        return LAC_DSA_3072_256_PAIR;
    }
    else
    {
        return LAC_DSA_INVALID_PAIR;
    }
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA S Sign internal callback
 ***************************************************************************/
STATIC
void LacKptDsaSSignCallback(CpaStatus status,
                            CpaBoolean protocolStatus,
                            CpaInstanceHandle instanceHandle,
                            lac_pke_op_cb_data_t *pCbData)
{
    CpaCyDsaGenCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyDsaSSignOpData *pOpData = NULL;
    CpaFlatBuffer *pS = NULL;
    Cpa8U *pMemPoolX = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCb = (CpaCyDsaGenCbFunc)LAC_CONST_PTR_CAST(pCbData->pClientCb);
    pCallbackTag = pCbData->pCallbackTag;
    pOpData = (CpaCyDsaSSignOpData *)LAC_CONST_PTR_CAST(pCbData->pClientOpData);
    pS = pCbData->pOutputData1;
    pMemPoolX = (Cpa8U *)(pCbData->pOpaqueData);

    LAC_ASSERT_NOT_NULL(pCb);
    LAC_ASSERT_NOT_NULL(pOpData);
    LAC_ASSERT_NOT_NULL(pS);
    LAC_ASSERT_NOT_NULL(pMemPoolX);

    LacKptMemPoolFree(pMemPoolX);

#ifndef DISABLE_STATS
    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_DSA_STAT_INC(numDsaSSignCompleted, pCryptoService);
    }
    else
    {
        LAC_KPT_DSA_STAT_INC(numDsaSSignCompletedErrors, pCryptoService);
    }
#endif
    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, protocolStatus, pS);
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA RS Sign internal callback
 ***************************************************************************/
STATIC
void LacKptDsaRSSignCallback(CpaStatus status,
                             CpaBoolean protocolStatus,
                             CpaInstanceHandle instanceHandle,
                             lac_pke_op_cb_data_t *pCbData)
{
    CpaCyDsaRSSignCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyDsaRSSignOpData *pOpData = NULL;
    CpaFlatBuffer *pR = NULL;
    CpaFlatBuffer *pS = NULL;
    Cpa8U *pMemPoolX = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCb = (CpaCyDsaRSSignCbFunc)LAC_CONST_PTR_CAST(pCbData->pClientCb);
    pCallbackTag = pCbData->pCallbackTag;
    pOpData =
        (CpaCyDsaRSSignOpData *)LAC_CONST_PTR_CAST(pCbData->pClientOpData);
    pR = pCbData->pOutputData1;
    pS = pCbData->pOutputData2;
    pMemPoolX = (Cpa8U *)(pCbData->pOpaqueData);

    LAC_ASSERT_NOT_NULL(pCb);
    LAC_ASSERT_NOT_NULL(pOpData);
    LAC_ASSERT_NOT_NULL(pR);
    LAC_ASSERT_NOT_NULL(pS);
    LAC_ASSERT_NOT_NULL(pMemPoolX);
    LacKptMemPoolFree(pMemPoolX);
#ifndef DISABLE_STATS
    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_DSA_STAT_INC(numDsaRSSignCompleted, pCryptoService);
    }
    else
    {
        LAC_KPT_DSA_STAT_INC(numDsaRSSignCompletedErrors, pCryptoService);
    }
#endif
    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, protocolStatus, pR, pS);
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA S Sign synchronous function
 ***************************************************************************/
STATIC CpaStatus LacKptDsaSignSSyn(const CpaInstanceHandle instanceHandle,
                                   const CpaCyDsaSSignOpData *pOpData,
                                   CpaBoolean *pProtocolStatus,
                                   CpaFlatBuffer *pS,
                                   CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_FAIL;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    /*
     * Call the async version of the function
     * with the sync callback function as a parameter.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = cpaCyKptDsaSignS(instanceHandle,
                                  LacSync_GenFlatBufVerifyCb,
                                  pSyncCallbackData,
                                  pOpData,
                                  pProtocolStatus,
                                  pS,
                                  pKptUnwrapContext);
    }
    else
    {
#ifndef DISABLE_STATS
        LAC_KPT_DSA_STAT_INC(numDsaSSignRequestErrors, pCryptoService);
#endif
        return status;
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus wCbStatus = CPA_STATUS_FAIL;
        wCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                            LAC_PKE_SYNC_CALLBACK_TIMEOUT,
                                            &status,
                                            pProtocolStatus);
        if (CPA_STATUS_SUCCESS != wCbStatus)
        {
#ifndef DISABLE_STATS
            /*
             * Inc stats only if the wait for callback failed.
             */
            LAC_KPT_DSA_STAT_INC(numDsaSSignCompletedErrors, pCryptoService);
#endif
            status = wCbStatus;
        }
    }
    else
    {
        /* As the Request was not sent the Callback will never
         * be called, so need to indicate that we're finished
         * with cookie so it can be destroyed. */
        LacSync_SetSyncCookieComplete(pSyncCallbackData);
    }
    LacSync_DestroySyncCookie(&pSyncCallbackData);
    return status;
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA S Sign API function
 ***************************************************************************/
CpaStatus cpaCyKptDsaSignS(const CpaInstanceHandle instanceHandle_in,
                           const CpaCyDsaGenCbFunc pCb,
                           void *pCallbackTag,
                           const CpaCyDsaSSignOpData *pOpData,
                           CpaBoolean *pProtocolStatus,
                           CpaFlatBuffer *pS,
                           CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_mmp_input_param_t in = {.flat_array = {0}};
    icp_qat_fw_mmp_output_param_t out = {.flat_array = {0}};
    Cpa32U inArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
    Cpa32U outArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};
    CpaBoolean internalMemIn[LAC_MAX_MMP_INPUT_PARAMS] = {CPA_FALSE};
    CpaBoolean internalMemOut[LAC_MAX_MMP_OUTPUT_PARAMS] = {CPA_FALSE};
    lac_pke_request_handle_t pRequestHandle = LAC_PKE_INVALID_HANDLE;
    lac_pke_op_cb_data_t cbData = {0};
    CpaInstanceHandle instanceHandle = NULL;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
#endif
    Cpa32U functionalityId = LAC_PKE_INVALID_FUNC_ID;
    Cpa32U nonceN = 0, opSizeInByte = 0;
#ifdef ICP_PARAM_CHECK
    Cpa32U byteLen = 0;
#endif
    lac_dsa_n_values_t opIndex = LAC_DSA_N_INVALID;

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    /* check for valid acceleration handle - can't update stats otherwise */
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    /* check LAC is initialised */
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    /* ensure this is a crypto or asym instance with pke enabled */
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    /* Check if the API has been called in sync mode */
    if (NULL == pCb)
    {
#ifdef ICP_TRACE
        status = LacKptDsaSignSSyn(
            instanceHandle, pOpData, pProtocolStatus, pS, pKptUnwrapContext);
        if (NULL != pProtocolStatus)
        {
            LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                     "0x%lx[%d], 0x%lx)\n",
                     (LAC_ARCH_UINT)instanceHandle_in,
                     (LAC_ARCH_UINT)pCb,
                     (LAC_ARCH_UINT)pCallbackTag,
                     (LAC_ARCH_UINT)pOpData,
                     (LAC_ARCH_UINT)pProtocolStatus,
                     *pProtocolStatus,
                     (LAC_ARCH_UINT)pS);
        }
        else
        {
            LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                     "0x%lx, 0x%lx)\n",
                     (LAC_ARCH_UINT)instanceHandle_in,
                     (LAC_ARCH_UINT)pCb,
                     (LAC_ARCH_UINT)pCallbackTag,
                     (LAC_ARCH_UINT)pOpData,
                     (LAC_ARCH_UINT)pProtocolStatus,
                     (LAC_ARCH_UINT)pS);
        }
        return status;
#endif
        return LacKptDsaSignSSyn(
            instanceHandle, pOpData, pProtocolStatus, pS, pKptUnwrapContext);
    }

#ifdef ICP_PARAM_CHECK
    /* check remaining parameters */
    status = LacKptDsaSSignParamCheck(
        pCb, pOpData, pProtocolStatus, pS, pKptUnwrapContext);
    if (CPA_STATUS_SUCCESS == status)
    {
#endif
        /* Fetch wpk size from unwrapcontext */
        pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
        pWpkSize = &(pUnwrapCxt->wpkSize);

#ifdef ICP_PARAM_CHECK
    }
#endif
#ifndef DISABLE_STATS
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LacPke_GetBitLen(&(pOpData->Q), &nonceN);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        opIndex = LacKptDsa_GetN(nonceN);
        opSizeInByte = LAC_ALIGN_POW2_ROUNDUP(LAC_BITS_TO_BYTES(nonceN),
                                              LAC_QUAD_WORD_IN_BYTES);
#ifdef ICP_PARAM_CHECK
        if (LAC_DSA_N_INVALID == opIndex)
        {
            LAC_INVALID_PARAM_LOG("Q not support size N "
                                  "Supported {L,N} = {1024, 160}, {2048, 224} "
                                  "{2048, 256}, {3072, 256}");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check byte len of z */
        byteLen = LacPke_GetMinBytes(&(pOpData->Z));
        if (byteLen > LAC_BITS_TO_BYTES(nonceN))
        {
            LAC_INVALID_PARAM_LOG("Z out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check 0 < k < q */
        if ((LacPke_CompareZero(&(pOpData->K), 0) == 0) ||
            (LacPke_Compare(&(pOpData->K), 0, &(pOpData->Q), 0) >= 0))
        {
            LAC_INVALID_PARAM_LOG("K out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check 0 < r < q */
        if ((LacPke_CompareZero(&(pOpData->R), 0) == 0) ||
            (LacPke_Compare(&(pOpData->R), 0, &(pOpData->Q), 0) >= 0))
        {
            LAC_INVALID_PARAM_LOG("R out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Ensure output buffer is the required size */
        if (pS->dataLenInBytes < LAC_BITS_TO_BYTES(nonceN))
        {
            LAC_INVALID_PARAM_LOG("Output Buffer has incorrect length");
            status = CPA_STATUS_INVALID_PARAM;
        }
#endif
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        Cpa8U *pMemPoolX = NULL;
        CpaFlatBuffer *pInBuffX = NULL;
        status = LacKptMemPoolMalloc(&pMemPoolX, pCryptoService->lac_kpt_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            /* If output buffer is larger than N we need to zero MS bytes */
            osalMemSet(
                pS->pData, 0, (pS->dataLenInBytes - LAC_BITS_TO_BYTES(nonceN)));

            functionalityId =
                LacPke_GetMmpId(opIndex,
                                lacKptDsaSignSSizeIdMap,
                                LAC_ARRAY_LEN(lacKptDsaSignSSizeIdMap));

            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolX, pWpkSize->wpkLenInBytes, &(pOpData->X));
            pInBuffX = (CpaFlatBuffer *)(pMemPoolX + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffX->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffX->pData = pMemPoolX;
            /* populate input parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_s_160.m, &pOpData->Z);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, m)] =
                opSizeInByte;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, m)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_s_160.k, &pOpData->K);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, k)] =
                opSizeInByte;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, k)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_s_160.q, &pOpData->Q);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, q)] =
                opSizeInByte;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, q)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_s_160.r, &pOpData->R);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, r)] =
                opSizeInByte;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, r)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_s_160.x, pInBuffX);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, x)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_s_input_t, x)] =
                CPA_TRUE;

            /* populate output parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(out.mmp_dsa_sign_s_160.s, pS);
            outArgSizeList[0] = opSizeInByte;
            internalMemOut[0] = CPA_FALSE;

            /* put the unwrap context into the output parameter list */
            out.flat_array[LAC_KPT_DSA_SIGNS_NUM_OUT_ARGS - 1] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            outArgSizeList[LAC_KPT_DSA_SIGNS_NUM_OUT_ARGS - 1] =
                sizeof(CpaCyKptUnwrapContext);
            internalMemOut[LAC_KPT_DSA_SIGNS_NUM_OUT_ARGS - 1] = CPA_FALSE;

            /* populate callback data */
            cbData.pClientCb = pCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pOpData;
            cbData.pOpaqueData = pMemPoolX;
            cbData.pOutputData1 = pS;

            status = LacPke_CreateRequest(&pRequestHandle,
                                          functionalityId,
                                          inArgSizeList,
                                          outArgSizeList,
                                          &in,
                                          &out,
                                          internalMemIn,
                                          internalMemOut,
                                          LacKptDsaSSignCallback,
                                          &cbData,
                                          instanceHandle);
            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the field mask */
                lac_pke_qat_req_data_t *pReqData =
                    (lac_pke_qat_req_data_t *)pRequestHandle;
                pReqData->u1.request.pke_hdr.kpt_mask = KPT_DSA_SSIGN_MASK;
                /* send request chain */
                status = LacPke_SendRequest(&pRequestHandle, instanceHandle);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LacKptMemPoolFree(pMemPoolX);
        }
    }

#ifndef DISABLE_STATS
    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_DSA_STAT_INC(numDsaSSignRequests, pCryptoService);
    }
    else
    {
        LAC_KPT_DSA_STAT_INC(numDsaSSignRequestErrors, pCryptoService);
    }
#endif
    return status;
}

#ifdef ICP_PARAM_CHECK
/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA RS Sign parameter check
 ***************************************************************************/
STATIC
CpaStatus LacKptDsaRSSignParamCheck(const CpaCyDsaRSSignCbFunc pCb,
                                    const CpaCyDsaRSSignOpData *pOpData,
                                    CpaBoolean *pProtocolStatus,
                                    CpaFlatBuffer *pR,
                                    CpaFlatBuffer *pS,
                                    CpaFlatBuffer *pKptUnwrapContext)
{

    /* check for valid callback function pointer */
    LAC_CHECK_NULL_PARAM(pCb);

    /* check for valid out buffer pointers */
    LAC_CHECK_NULL_PARAM(pProtocolStatus);
    LAC_CHECK_NULL_PARAM(pR);
    LAC_CHECK_NULL_PARAM(pS);
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext);

    /* check parameters for null, zero size and LSB not set */
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->Z, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->K, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->P, CHECK_NONE, 0);
    LAC_CHECK_ODD_PARAM(&pOpData->P);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->Q, CHECK_NONE, 0);
    LAC_CHECK_ODD_PARAM(&pOpData->Q);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->G, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->X, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(pR, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(pS, CHECK_NONE, 0);
    LAC_CHECK_FLAT_BUFFER_PARAM(pKptUnwrapContext, CHECK_NONE, 0);

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA RS Sign synchronous function
 ***************************************************************************/
STATIC CpaStatus LacKptDsaRSSignSyn(const CpaInstanceHandle instanceHandle,
                                    const CpaCyDsaRSSignOpData *pOpData,
                                    CpaBoolean *pProtocolStatus,
                                    CpaFlatBuffer *pR,
                                    CpaFlatBuffer *pS,
                                    CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_FAIL;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    /*
     * Call the async version of the function
     * with the sync callback function as a parameter.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = cpaCyKptDsaSignRS(instanceHandle,
                                   LacSync_GenDualFlatBufVerifyCb,
                                   pSyncCallbackData,
                                   pOpData,
                                   pProtocolStatus,
                                   pR,
                                   pS,
                                   pKptUnwrapContext);
    }
    else
    {
#ifndef DISABLE_STATS
        LAC_KPT_DSA_STAT_INC(numDsaRSSignRequestErrors, pCryptoService);
#endif
        return status;
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus wCbStatus = CPA_STATUS_FAIL;
        wCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                            LAC_PKE_SYNC_CALLBACK_TIMEOUT,
                                            &status,
                                            pProtocolStatus);
        if (CPA_STATUS_SUCCESS != wCbStatus)
        {
#ifndef DISABLE_STATS
            /*
             * Inc stats only if the wait for callback failed.
             */
            LAC_KPT_DSA_STAT_INC(numDsaRSSignCompletedErrors, pCryptoService);
#endif
            status = wCbStatus;
        }
    }
    else
    {
        /* As the Request was not sent the Callback will never
         * be called, so need to indicate that we're finished
         * with cookie so it can be destroyed. */
        LacSync_SetSyncCookieComplete(pSyncCallbackData);
    }
    LacSync_DestroySyncCookie(&pSyncCallbackData);
    return status;
}

/**
 ***************************************************************************
 * @ingroup Lac_KptDsa
 *      DSA RS Sign API function
 ***************************************************************************/
CpaStatus cpaCyKptDsaSignRS(const CpaInstanceHandle instanceHandle_in,
                            const CpaCyDsaRSSignCbFunc pCb,
                            void *pCallbackTag,
                            const CpaCyDsaRSSignOpData *pOpData,
                            CpaBoolean *pProtocolStatus,
                            CpaFlatBuffer *pR,
                            CpaFlatBuffer *pS,
                            CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_mmp_input_param_t in = {.flat_array = {0}};
    icp_qat_fw_mmp_output_param_t out = {.flat_array = {0}};
    Cpa32U inArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
    Cpa32U outArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};
    CpaBoolean internalMemIn[LAC_MAX_MMP_INPUT_PARAMS] = {CPA_FALSE};
    CpaBoolean internalMemOut[LAC_MAX_MMP_OUTPUT_PARAMS] = {CPA_FALSE};
    lac_pke_op_cb_data_t cbData = {0};
    CpaInstanceHandle instanceHandle = NULL;
    lac_pke_request_handle_t pRequestHandle = LAC_PKE_INVALID_HANDLE;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t *pCryptoService = NULL;
#endif
    Cpa32U functionalityId = LAC_PKE_INVALID_FUNC_ID;
    Cpa32U bitLenL = 0, nonceN = 0;
    Cpa32U opSizeNInBytes = 0;
#ifdef ICP_PARAM_CHECK
    Cpa32U byteLen = 0;
#endif
    lac_dsa_ln_pairs_t opIndex = LAC_DSA_INVALID_PAIR;

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    /* check for valid acceleration handle - can't update stats otherwise */
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    /* check LAC is initialised */
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    /* ensure this is a crypto or asym instance with pke enabled */
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    /* Check if the API has been called in sync mode */
    if (NULL == pCb)
    {
#ifdef ICP_TRACE
        status = LacKptDsaRSSignSyn(instanceHandle,
                                    pOpData,
                                    pProtocolStatus,
                                    pR,
                                    pS,
                                    pKptUnwrapContext);
        if (NULL != pProtocolStatus)
        {
            LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx[%d]"
                     ", 0x%lx, 0x%lx)\n",
                     (LAC_ARCH_UINT)instanceHandle_in,
                     (LAC_ARCH_UINT)pCb,
                     (LAC_ARCH_UINT)pCallbackTag,
                     (LAC_ARCH_UINT)pOpData,
                     *pProtocolStatus,
                     (LAC_ARCH_UINT)pR,
                     (LAC_ARCH_UINT)pS);
        }
        else
        {
            LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx"
                     ", 0x%lx, 0x%lx)\n",
                     (LAC_ARCH_UINT)instanceHandle_in,
                     (LAC_ARCH_UINT)pCb,
                     (LAC_ARCH_UINT)pCallbackTag,
                     (LAC_ARCH_UINT)pOpData,
                     (LAC_ARCH_UINT)pR,
                     (LAC_ARCH_UINT)pS);
        }
        return status;
#else
        return LacKptDsaRSSignSyn(instanceHandle,
                                  pOpData,
                                  pProtocolStatus,
                                  pR,
                                  pS,
                                  pKptUnwrapContext);
#endif
    }

#ifdef ICP_PARAM_CHECK
    /* check remaining parameters */
    status = LacKptDsaRSSignParamCheck(
        pCb, pOpData, pProtocolStatus, pR, pS, pKptUnwrapContext);

    if (CPA_STATUS_SUCCESS == status)
    {
#endif
        /* Fetch wpk size from unwrapcontext */
        pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
        pWpkSize = &(pUnwrapCxt->wpkSize);
#ifdef ICP_PARAM_CHECK
    }
#endif
#ifndef DISABLE_STATS
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
#endif
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LacPke_GetBitLen(&(pOpData->P), &bitLenL);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LacPke_GetBitLen(&(pOpData->Q), &nonceN);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        opIndex = LacKptDsa_GetLNPair(bitLenL, nonceN);
        opSizeNInBytes = LAC_ALIGN_POW2_ROUNDUP(LAC_BITS_TO_BYTES(nonceN),
                                                LAC_QUAD_WORD_IN_BYTES);
#ifdef ICP_PARAM_CHECK
        if (LAC_DSA_INVALID_PAIR == opIndex)
        {
            LAC_INVALID_PARAM_LOG("P, Q out of range "
                                  "Supported {L,N} = {1024, 160}, {2048, 224} "
                                  "{2048, 256}, {3072, 256}");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check byte len of z */
        byteLen = LacPke_GetMinBytes(&(pOpData->Z));
        if (byteLen > LAC_BITS_TO_BYTES(nonceN))
        {
            LAC_INVALID_PARAM_LOG("Z out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check 0 < k < q */
        if ((LacPke_CompareZero(&(pOpData->K), 0) == 0) ||
            (LacPke_Compare(&(pOpData->K), 0, &(pOpData->Q), 0) >= 0))
        {
            LAC_INVALID_PARAM_LOG("K out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {

        /* Check 1 < g < p */
        if ((LacPke_CompareZero(&(pOpData->G), -1) <= 0) ||
            (LacPke_Compare(&(pOpData->G), 0, &(pOpData->P), 0) >= 0))

        {
            LAC_INVALID_PARAM_LOG("G out of Range");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Ensure output buffers are the required size */
        if (pR->dataLenInBytes < LAC_BITS_TO_BYTES(nonceN))
        {
            LAC_INVALID_PARAM_LOG("Output Buffer R has incorrect length");
            status = CPA_STATUS_INVALID_PARAM;
        }

        if (pS->dataLenInBytes < LAC_BITS_TO_BYTES(nonceN))
        {
            LAC_INVALID_PARAM_LOG("Output Buffer S has incorrect length");
            status = CPA_STATUS_INVALID_PARAM;
        }
#endif
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaFlatBuffer *pInBuffX = NULL;
        Cpa8U *pMemPoolX = NULL;
        status = LacKptMemPoolMalloc(&pMemPoolX, pCryptoService->lac_kpt_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            /* Construct the kpt wpk buf */
            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolX, pWpkSize->wpkLenInBytes, &(pOpData->X));
            pInBuffX = (CpaFlatBuffer *)(pMemPoolX + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffX->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffX->pData = pMemPoolX;
            /* If output buffers are larger than N we need to zero MS bytes */
            osalMemSet(
                pR->pData, 0, (pR->dataLenInBytes - LAC_BITS_TO_BYTES(nonceN)));
            osalMemSet(
                pS->pData, 0, (pS->dataLenInBytes - LAC_BITS_TO_BYTES(nonceN)));

            functionalityId =
                LacPke_GetMmpId(opIndex,
                                lacKptDsaSignRsSizeIdMap,
                                LAC_ARRAY_LEN(lacKptDsaSignRsSizeIdMap));

            /* populate input parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.m,
                                          &pOpData->Z);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, m)] =
                opSizeNInBytes;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, m)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.k,
                                          &pOpData->K);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, k)] =
                opSizeNInBytes;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, k)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.p,
                                          &pOpData->P);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, p)] =
                LAC_BITS_TO_BYTES(bitLenL);
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, p)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.q,
                                          &pOpData->Q);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, q)] =
                opSizeNInBytes;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, q)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.g,
                                          &pOpData->G);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, g)] =
                LAC_BITS_TO_BYTES(bitLenL);
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, g)] =
                CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_dsa_sign_r_s_1024_160.x,
                                          pInBuffX);
            inArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, x)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemIn[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_input_t, x)] =
                CPA_TRUE;

            /* populate output parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(out.mmp_dsa_sign_r_s_1024_160.r, pR);
            outArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_output_t,
                                      r)] = opSizeNInBytes;
            internalMemOut[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_output_t,
                                      r)] = CPA_FALSE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(out.mmp_dsa_sign_r_s_1024_160.s, pS);
            outArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_output_t,
                                      s)] = opSizeNInBytes;
            internalMemOut[LAC_IDX_OF(icp_qat_fw_mmp_dsa_sign_r_s_output_t,
                                      s)] = CPA_FALSE;
            /* put the unwrap context into the output parameter list */
            out.flat_array[LAC_KPT_DSA_SIGNRS_NUM_OUT_ARGS - 1] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            outArgSizeList[LAC_KPT_DSA_SIGNRS_NUM_OUT_ARGS - 1] =
                sizeof(CpaCyKptUnwrapContext);
            internalMemOut[LAC_KPT_DSA_SIGNRS_NUM_OUT_ARGS - 1] = CPA_FALSE;

            /* populate callback data */
            cbData.pClientCb = pCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pOpData;
            cbData.pOpaqueData = pMemPoolX;
            cbData.pOutputData1 = pR;
            cbData.pOutputData2 = pS;

            status = LacPke_CreateRequest(&pRequestHandle,
                                          functionalityId,
                                          inArgSizeList,
                                          outArgSizeList,
                                          &in,
                                          &out,
                                          internalMemIn,
                                          internalMemOut,
                                          LacKptDsaRSSignCallback,
                                          &cbData,
                                          instanceHandle);
            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the field mask */
                lac_pke_qat_req_data_t *pReqData =
                    (lac_pke_qat_req_data_t *)pRequestHandle;
                pReqData->u1.request.pke_hdr.kpt_mask = KPT_DSA_RSSIGN_MASK;
                /* send request chain */
                status = LacPke_SendRequest(&pRequestHandle, instanceHandle);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LacKptMemPoolFree(pMemPoolX);
        }
    }

#ifndef DISABLE_STATS
    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_DSA_STAT_INC(numDsaRSSignRequests, pCryptoService);
    }
    else
    {
        LAC_KPT_DSA_STAT_INC(numDsaRSSignRequestErrors, pCryptoService);
    }
#endif
    return status;
}
