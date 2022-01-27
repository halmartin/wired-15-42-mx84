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
 *
 * @file lac_kpt_ecdsa.c
 *
 * @ingroup Lac_KptEc
 *
 * KPT Elliptic Curve Digital Signature Algorithm functions
 *
 * @lld_start
 *
 * @lld_overview
 * This file implements the KPT Elliptic Curve DSA apis. It implements
 * KPT Ecdsa API services: signature generation (s, rs),
 * Statistics are maintained per instance for each service.
 * The parameters supplied by the client are checked, and then input/output
 * argument lists are constructed before calling the PKE Comms layer to
 * create and send a request to the QAT.
 *
 * For KPT Ecdsa signRS,the number of input parameters to the function
 * is greater than the number of input parameters allowed for the PKE service
 * Therefore, for this function, we copy and concatenate all inputs(except
 * private key d) to 1 internal buffer and this is sent to QAT for processing.
 *
 * In all other cases the service implementations are a straightforward
 * marshalling of client-supplied parameters for the QAT. I.e. there is
 * minimal logic handled by this component.
 *
 * @lld_dependencies
 * - \ref LacAsymCommonQatComms "PKE QAT Comms" : For creating and sending
 * messages to the QAT
 * - \ref LacMem "Mem" : For memory allocation and freeing, and translating
 * between scalar and pointer types
 * - OSAL : For atomics and logging
 *
 * @note
 * The KPT ECDSA feature may be called in Asynchronous or Synchronous modes.
 * In Asynchronous mode the user supplies a Callback function to the API.
 * Control returns to the client after the message has been sent to the QAT
 * and the Callback gets invoked when the QAT completes the operation. There
 * is NO BLOCKING. This mode is preferred for maximum performance.
 * In Synchronous mode the client supplies no Callback function pointer (NULL)
 * and the point of execution is placed on a wait-queue internally, and this
 * is de-queued once the QAT completes the operation. Hence, Synchronous mode
 * is BLOCKING. So avoid using in an interrupt context. To achieve maximum
 * performance from the API Asynchronous mode is preferred.
 *
 * @performance
 *
 * @lld_initialisation
 * On initialization this component clears the stats.
 *
 * @lld_module_algorithms
 *
 * @lld_process_context
 *
 * @lld_end
 *
 ***************************************************************************/

/*
****************************************************************************
* Include public/global header files
****************************************************************************
*/

/* API Includes */
#include "cpa.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_cy_kpt.h"

/* OSAL Includes */
#include "Osal.h"

/* ADF Includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* QAT includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp.h"
#include "icp_qat_fw_mmp_ids.h"
#include "icp_qat_fw_pke.h"

/* Look Aside Includes */
#include "lac_log.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_pke_utils.h"
#include "lac_pke_qat_comms.h"
#include "lac_sync.h"
#include "lac_ec.h"
#include "lac_sym.h"
#include "lac_list.h"
#include "sal_service_state.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_kpt_crypto_qat_comms.h"

#define LAC_KPT_ECDSA_SIGNRS_NUM_OUT_ARGS (3)
/**< number of 'out' arguments in output arguments array for Sign RS */
#define LAC_KPT_ECDSA_SIGNRS_NUM_IN_QA_API (7)
/**< number of 'in' arguments in the arguments size list for Kpt Sign RS */
#define KPT_ECDSA_SIGNRS_MASK 0x2
/**< mask value to indicate which node in input array is wrapped,
 * {concatenation,d',k} b10 0x2*/
#define KPT_ECDSA_SIGNRS_RN_MASK 0x4
/**< mask value to indicate which node in input array is random number,
 * {concatenation,d',k} b100 0x4*/

#define LacKptEcdsaSignRSOpDataWrite(in_struct, out, pConcateBuff, pD, pR, pS) \
    do                                                                         \
    {                                                                          \
        /* populate input parameters */                                        \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in_struct.in, pConcateBuff);             \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in_struct.d, pD);                        \
                                                                               \
        /* populate output parameters */                                       \
        LAC_MEM_SHARED_WRITE_FROM_PTR(out.r, pR);                              \
        LAC_MEM_SHARED_WRITE_FROM_PTR(out.s, pS);                              \
    } while (0);
/* macro to populate Sign RS parameters */

#ifndef DISABLE_STATS
#define LAC_KPT_ECDSA_STAT_INC(statistic, pCryptoService)                      \
    do                                                                         \
    {                                                                          \
        if (CPA_TRUE ==                                                        \
            pCryptoService->generic_service_info.stats->bEccStatsEnabled)      \
        {                                                                      \
            osalAtomicInc(                                                     \
                &pCryptoService->pLacEcdsaStatsArr[offsetof(CpaCyEcdsaStats64, \
                                                            statistic) /       \
                                                   sizeof(Cpa64U)]);           \
        }                                                                      \
    } while (0)
/**< @ingroup Lac_KptEc
 * macro to increment a ECDSA stat (derives offset into array of atomics) */
#else
#define LAC_KPT_ECDSA_STAT_INC(statistic, pCryptoService)                      \
    (pCryptoService) = (pCryptoService)
#endif

#ifdef ICP_PARAM_CHECK
/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      KPT ECDSA Sign R & S parameter check
 ***************************************************************************/
STATIC
CpaStatus LacKptEcdsa_SignRSBasicParamCheck(
    const CpaInstanceHandle instanceHandle,
    const CpaCyKptEcdsaSignRSOpData *pOpData,
    CpaBoolean *pMultiplyStatus,
    CpaFlatBuffer *pR,
    CpaFlatBuffer *pS,
    CpaFlatBuffer *pKptUnwrapContext)
{
    /* check for NULL pointers */
    LAC_CHECK_NULL_PARAM(pMultiplyStatus);
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pR);
    LAC_CHECK_NULL_PARAM(pS);
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext);

    /* Check flat buffers in pOpData for NULL and dataLen of 0*/
    LAC_CHECK_NULL_PARAM(pOpData->a.pData);
    LAC_CHECK_SIZE(&(pOpData->a), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->b.pData);
    LAC_CHECK_SIZE(&(pOpData->b), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->q.pData);
    LAC_CHECK_SIZE(&(pOpData->q), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->xg.pData);
    LAC_CHECK_SIZE(&(pOpData->xg), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->yg.pData);
    LAC_CHECK_SIZE(&(pOpData->yg), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->n.pData);
    LAC_CHECK_SIZE(&(pOpData->n), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->m.pData);
    LAC_CHECK_SIZE(&(pOpData->m), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->d.pData);
    LAC_CHECK_SIZE(&(pOpData->d), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pR->pData);
    LAC_CHECK_SIZE(pR, CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pS->pData);
    LAC_CHECK_SIZE(pS, CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext->pData);
    LAC_CHECK_SIZE(pKptUnwrapContext, CHECK_NONE, 0);

    if (CPA_CY_EC_FIELD_TYPE_PRIME != pOpData->fieldType &&
        CPA_CY_EC_FIELD_TYPE_BINARY != pOpData->fieldType)
    {
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Check that q is odd */
    LAC_CHECK_ODD_PARAM(&(pOpData->q));

    /* Check that n is odd */
    LAC_CHECK_ODD_PARAM(&(pOpData->n));

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      return the size of the biggest number in CpaCyKptEcdsaSignRSOpData
 *
 * @description
 *      return the size of the biggest number in CpaCyKptEcdsaSignRSOpData,in
 *      KPT mode, private key's size is indicated by wpk_size
 *
 * @param[in]  pOpData      Pointer to a CpaCyKptEcdsaSignRSOpData structure
 *
 * @retval max  the size in bytes of the biggest number
 *
 ***************************************************************************/
STATIC Cpa32U
LacKptEcdsa_SignRSOpDataSizeGetMax(const CpaCyKptEcdsaSignRSOpData *pOpData,
                                   Cpa32U wpk_size)
{
    Cpa32U max = 0;
    /* need to find max size in bytes of number in input buffers */
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->xg)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->yg)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->m)), max);
    max = LAC_MAX(wpk_size, max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->q)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->n)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->a)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->b)), max);

    return max;
}

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *     Copies least significant len bytes from a flat buffer (if length of
 *     flat buffer is less than len padding will be added).
 *
 * @description
 *     This function copies data in a flat buffer to memory pointed to by
 *     ptr. This function performs no checks so it is assumed that there is
 *     enough memory allocated.
 *
 * @param[in/out]  ptr      Pointer to a pointer to an array of Cpa8U
 * @param[in]  pBuff        Pointer to a CpaFlatBuffer structure
 * @param[in]  len          Number of bytes to copy.
 *                          This is the amount by which *ptr is
 *                          incremented also.
 *
 ***************************************************************************/

STATIC void LacKptEc_FlatBuffToConcate(Cpa8U **ptr,
                                       const CpaFlatBuffer *pBuff,
                                       Cpa32U len)
{
    Cpa8U *pMem = NULL;

    pMem = (Cpa8U *)*ptr;

    if (pBuff->dataLenInBytes < len)
    {
        /* pad */
        osalMemSet(pMem, 0, (len - pBuff->dataLenInBytes));
        pMem = pMem + (len - pBuff->dataLenInBytes);
        /* copy all flat buffer */
        memcpy(pMem, pBuff->pData, pBuff->dataLenInBytes);
        pMem = pMem + pBuff->dataLenInBytes;
    }
    else
    {
        /* no padding is required and we may need to index into
           flatbuffer - only lsbs are copied */
        memcpy(pMem, &(pBuff->pData[pBuff->dataLenInBytes - len]), len);
        pMem = pMem + len;
    }

    *ptr = pMem;
}

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      KptECDSA Sign R & S synchronous function
 ***************************************************************************/
STATIC CpaStatus LacKptEcdsa_SignRSSyn(const CpaInstanceHandle instanceHandle,
                                       const CpaCyKptEcdsaSignRSOpData *pOpData,
                                       CpaBoolean *pMultiplyStatus,
                                       CpaFlatBuffer *pR,
                                       CpaFlatBuffer *pS,
                                       CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;

    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    /*
     * Call the asynchronous version of the function
     * with the generic synchronous callback function as a parameter.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = cpaCyKptEcdsaSignRS(instanceHandle,
                                     LacSync_GenDualFlatBufVerifyCb,
                                     pSyncCallbackData,
                                     pOpData,
                                     pMultiplyStatus,
                                     pR,
                                     pS,
                                     pKptUnwrapContext);
    }
    else
    {
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSRequestErrors, pCryptoService);
        return status;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus wCbStatus = CPA_STATUS_FAIL;
        wCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                            LAC_PKE_SYNC_CALLBACK_TIMEOUT,
                                            &status,
                                            pMultiplyStatus);

        if (CPA_STATUS_SUCCESS != wCbStatus)
        {
            LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSCompletedErrors,
                                   pCryptoService);
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
 * @ingroup Lac_KptEc
 *    KPT ECDSA Sign R & S internal callback
 ***************************************************************************/
STATIC
void LacKptEcdsa_SignRSCallback(CpaStatus status,
                                CpaBoolean multiplyStatus,
                                CpaInstanceHandle instanceHandle,
                                lac_pke_op_cb_data_t *pCbData)
{
    CpaCyEcdsaSignRSCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyKptEcdsaSignRSOpData *pOpData = NULL;
    CpaFlatBuffer *pR = NULL;
    CpaFlatBuffer *pS = NULL;
    Cpa8U **ppMemPoolArray = NULL;

    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;

    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCb = (CpaCyEcdsaSignRSCbFunc)pCbData->pClientCb;
    pCallbackTag = pCbData->pCallbackTag;
    pOpData = (CpaCyKptEcdsaSignRSOpData *)pCbData->pClientOpData;
    pR = pCbData->pOutputData1;
    pS = pCbData->pOutputData2;
    ppMemPoolArray = (Cpa8U **)(pCbData->pOpaqueData);

    LAC_ASSERT_NOT_NULL(pCb);
    LAC_ASSERT_NOT_NULL(pOpData);
    LAC_ASSERT_NOT_NULL(pR);
    LAC_ASSERT_NOT_NULL(pR->pData);
    LAC_ASSERT_NOT_NULL(pS);
    LAC_ASSERT_NOT_NULL(pS->pData);
    LAC_ASSERT_NOT_NULL(ppMemPoolArray);

    /* Free Mem Pool array */
    LacKptMemPoolFree(ppMemPoolArray[0]);
    ppMemPoolArray[0] = NULL;
    LacKptMemPoolFree(ppMemPoolArray[1]);
    ppMemPoolArray[1] = NULL;
    LacKptMemPoolFree((Cpa8U *)ppMemPoolArray);

    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSCompleted, pCryptoService);
    }
    else
    {
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSCompletedErrors, pCryptoService);
    }

    if ((CPA_FALSE == multiplyStatus) && (CPA_STATUS_SUCCESS == status))
    {
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSCompletedOutputInvalid,
                               pCryptoService);
    }

    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, multiplyStatus, pR, pS);
}

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *
 ***************************************************************************/
CpaStatus cpaCyKptEcdsaSignRS(const CpaInstanceHandle instanceHandle_in,
                              const CpaCyEcdsaSignRSCbFunc pCb,
                              void *pCallbackTag,
                              const CpaCyKptEcdsaSignRSOpData *pOpData,
                              CpaBoolean *pMultiplyStatus,
                              CpaFlatBuffer *pR,
                              CpaFlatBuffer *pS,
                              CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U dataOperationSizeBytes = 0;
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t *pCryptoService = NULL;
    lac_pke_request_handle_t pRequestHandle = LAC_PKE_INVALID_HANDLE;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;
#ifdef ICP_PARAM_CHECK
    Cpa32S compare = 0;
    Cpa32U bit_pos_q = 0, bit_pos_x = 0, bit_pos_y = 0;
    Cpa32U temp = 0;
    Cpa32U maxModLen = 0;
    CpaBoolean isZero = CPA_FALSE;
#endif
    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    /* instance checks - if fail, no inc stats just return */
    /* check for valid acceleration handle */
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    /* ensure LAC is initialised - return error if not */
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    /* ensure this is a crypto or asym instance with pke enabled */
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    /* Check if the API has been called in synchronous mode */
    if (NULL == pCb)
    {
#ifdef ICP_TRACE
#ifdef ICP_PARAM_CHECK
        /* Check for valid pointers */
        LAC_CHECK_NULL_PARAM(pMultiplyStatus);
#endif
        status = LacKptEcdsa_SignRSSyn(instanceHandle,
                                       pOpData,
                                       pMultiplyStatus,
                                       pR,
                                       pS,
                                       pKptUnwrapContext);

        LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                 "%d, 0x%lx, 0x%lx)\n",
                 (LAC_ARCH_UINT)instanceHandle_in,
                 (LAC_ARCH_UINT)pCb,
                 (LAC_ARCH_UINT)pCallbackTag,
                 (LAC_ARCH_UINT)pOpData,
                 *pMultiplyStatus,
                 (LAC_ARCH_UINT)pR,
                 (LAC_ARCH_UINT)pS);
        return status;
#else
        /* Call synchronous mode function */
        return LacKptEcdsa_SignRSSyn(instanceHandle,
                                     pOpData,
                                     pMultiplyStatus,
                                     pR,
                                     pS,
                                     pKptUnwrapContext);
#endif
    }

#ifdef ICP_PARAM_CHECK
    /* Basic Param Checking */
    status = LacKptEcdsa_SignRSBasicParamCheck(
        instanceHandle, pOpData, pMultiplyStatus, pR, pS, pKptUnwrapContext);

    /* Check that output buffers are big enough */
    if (CPA_STATUS_SUCCESS == status)
    {
        maxModLen = LacPke_GetMinBytes(&(pOpData->n));
        if ((pR->dataLenInBytes < maxModLen) ||
            (pS->dataLenInBytes < maxModLen))
        {
            LAC_INVALID_PARAM_LOG("Output buffer not big enough");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
#endif
        /* Fetch wpk size from unwrapcontext*/
        pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
        pWpkSize = &(pUnwrapCxt->wpkSize);
#ifdef ICP_PARAM_CHECK
    }
#endif
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Determine size */
        status = LacKptEc_GetRange(LacKptEcdsa_SignRSOpDataSizeGetMax(
                                       pOpData, pWpkSize->wpkLenInBytes),
                                   &dataOperationSizeBytes);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        if ((LAC_EC_SIZE_QW4_IN_BYTES == dataOperationSizeBytes) &&
            (CPA_CY_EC_FIELD_TYPE_BINARY == pOpData->fieldType))
        {
            /* Check if it is a NIST curve if not use 8QW */
            LacEc_CheckCurve4QWGF2(&dataOperationSizeBytes,
                                   &(pOpData->q),
                                   &(pOpData->a),
                                   &(pOpData->b),
                                   &(pOpData->n),
                                   NULL);
        }
    }

#ifdef ICP_PARAM_CHECK
    if ((CPA_STATUS_SUCCESS == status) &&
        (LAC_EC_SIZE_QW9_IN_BYTES == dataOperationSizeBytes))
    {
        /* 9QW checks */
        if (CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType)
        {
            /* Check if is is a NIST curve (if not it is an invalid param) */
            /* Also checks that xG and yG are less than 2^521 */
            status = LacEc_CheckCurve9QWGFP(&(pOpData->q),
                                            &(pOpData->a),
                                            &(pOpData->b),
                                            &(pOpData->n),
                                            NULL,
                                            &(pOpData->xg),
                                            &(pOpData->yg));
        }
        else
        {
            /* Check if is is a NIST curve (if not it is an invalid param) */
            /* Also checks that xG and yG are less than 571 bits */
            status = LacEc_CheckCurve9QWGF2(&(pOpData->q),
                                            &(pOpData->a),
                                            &(pOpData->b),
                                            &(pOpData->n),
                                            NULL,
                                            &(pOpData->xg),
                                            &(pOpData->yg));
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Ensure base point is not (0,0) */
        if ((0 == LacPke_CompareZero(&(pOpData->xg), 0)) &&
            (0 == LacPke_CompareZero(&(pOpData->yg), 0)))
        {
            LAC_INVALID_PARAM_LOG("Invalid base point");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* For GFP check q>3 and xg and yg less than q */
        if (CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType)
        {
            /* Ensure q > 3 */
            LacPke_GetBitPos(&(pOpData->q), &bit_pos_q, &temp, &isZero);
            if (bit_pos_q < LAC_EC_MIN_MOD_BIT_POS_GFP)
            {
                LAC_INVALID_PARAM_LOG("q is not > 3 as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
            /* Ensure xg < q */
            compare = LacPke_Compare(&(pOpData->xg), 0, &(pOpData->q), 0);
            if (compare >= 0)
            {
                LAC_INVALID_PARAM_LOG("xg is not < q as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
            /* Ensure yg < q */
            compare = LacPke_Compare(&(pOpData->yg), 0, &(pOpData->q), 0);
            if (compare >= 0)
            {
                LAC_INVALID_PARAM_LOG("yg is not < q as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
        }
        /* For GF2 4 and 8 QW check deg(q)>2 and deg(xg) and deg(yg) less
        than deg(q) (note: already checked for 9QW case) */
        if (((LAC_EC_SIZE_QW8_IN_BYTES == dataOperationSizeBytes) ||
             (LAC_EC_SIZE_QW4_IN_BYTES == dataOperationSizeBytes)) &&
            (CPA_CY_EC_FIELD_TYPE_BINARY == pOpData->fieldType))
        {
            LacPke_GetBitPos(&(pOpData->q), &bit_pos_q, &temp, &isZero);
            if (bit_pos_q < LAC_EC_MIN_MOD_BIT_POS_GF2)
            {
                LAC_INVALID_PARAM_LOG("deg(q) is not > 2 as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
            /* Ensure deg(xg) < deg(q) for non zero xg */
            LacPke_GetBitPos(&(pOpData->xg), &bit_pos_x, &temp, &isZero);
            if ((CPA_TRUE != isZero) && (bit_pos_x >= bit_pos_q))
            {
                LAC_INVALID_PARAM_LOG("deg(xg) is not < deg(q) as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
            /* Ensure deg(yg) < deg(q) for non zero yg */
            LacPke_GetBitPos(&(pOpData->yg), &bit_pos_y, &temp, &isZero);
            if ((CPA_TRUE != isZero) && (bit_pos_y >= bit_pos_q))
            {
                LAC_INVALID_PARAM_LOG("deg(yg) is not < deg(q) as required");
                status = CPA_STATUS_INVALID_PARAM;
            }
        }
    }
#endif

    if (CPA_STATUS_SUCCESS == status)
    {
        Cpa8U *pMemPoolConcate = NULL;
        Cpa8U *pMemPoolD = NULL;
        Cpa8U *pMemPoolArray = NULL;
        Cpa8U *pConcateTemp = NULL;
        Cpa8U **ppMemArray = NULL;
        CpaFlatBuffer *pInBuff = NULL;
        CpaFlatBuffer *pInBuffD = NULL;
        icp_qat_fw_mmp_input_param_t inRS = {.flat_array = {0}};
        icp_qat_fw_mmp_output_param_t outRS = {.flat_array = {0}};
        lac_pke_op_cb_data_t cbData = {0};

        /* Holding the calculated size of the input/output parameters */
        Cpa32U inArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
        Cpa32U outArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};

        CpaBoolean internalMemInList[LAC_MAX_MMP_INPUT_PARAMS] = {CPA_FALSE};
        CpaBoolean internalMemOutList[LAC_MAX_MMP_OUTPUT_PARAMS] = {CPA_FALSE};

        Cpa32U functionID = 0;

        /* clear output buffers */
        osalMemSet(pR->pData, 0, pR->dataLenInBytes);
        osalMemSet(pS->pData, 0, pS->dataLenInBytes);

        /* Need to concatenate user inputs - copy to ecc mempool memory */
        status = LacKptMemPoolMalloc(&pMemPoolArray,
                                     pCryptoService->lac_kpt_array_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            status =
                LacKptMemPoolMalloc(&pMemPoolD, pCryptoService->lac_kpt_pool);
            if (CPA_STATUS_SUCCESS == status)
            {
                status = LacKptMemPoolMalloc(&pMemPoolConcate,
                                             pCryptoService->lac_ec_pool);
            }
        }
        if (CPA_STATUS_SUCCESS == status)
        {
            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolD, pWpkSize->wpkLenInBytes, &(pOpData->d));
            pInBuffD = (CpaFlatBuffer *)(pMemPoolD + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffD->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffD->pData = pMemPoolD;

            /* Concatenate x,y, n, q, a, b, m*/
            pConcateTemp = pMemPoolConcate;
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->m), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->b), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->a), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->q), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->n), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->yg), dataOperationSizeBytes);
            LacKptEc_FlatBuffToConcate(
                &pConcateTemp, &(pOpData->xg), dataOperationSizeBytes);
            pInBuff = (CpaFlatBuffer *)pConcateTemp;
            pInBuff->dataLenInBytes =
                (dataOperationSizeBytes * LAC_KPT_ECDSA_SIGNRS_NUM_IN_QA_API);
            pInBuff->pData = pMemPoolConcate;

            ppMemArray = (Cpa8U **)pMemPoolArray;
            ppMemArray[0] = pMemPoolConcate;
            ppMemArray[1] = pMemPoolD;

            /* populate callback data */
            cbData.pClientCb = pCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pOpData;
            cbData.pOpaqueData = ppMemArray;
            cbData.pOutputData1 = pR;
            cbData.pOutputData2 = pS;

            /* Set the size for all parameters to be padded to */
            LAC_EC_SET_LIST_PARAMS(outArgSizeList,
                                   LAC_KPT_ECDSA_SIGNRS_NUM_OUT_ARGS,
                                   dataOperationSizeBytes);
            /* Output memory to QAT is externally allocated */
            LAC_EC_SET_LIST_PARAMS(internalMemOutList,
                                   LAC_KPT_ECDSA_SIGNRS_NUM_OUT_ARGS,
                                   CPA_FALSE);

            /* Populate input buffers and output buffers and set function IDs */
            if (CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType)
            {
                switch (dataOperationSizeBytes)
                {
                    case LAC_EC_SIZE_QW4_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gfp_l256,
                            outRS.mmp_kpt_ecdsa_sign_rs_gfp_l256,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GFP_L256;
                        break;
                    case LAC_EC_SIZE_QW8_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gfp_l512,
                            outRS.mmp_kpt_ecdsa_sign_rs_gfp_l512,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GFP_L512;
                        break;
                    case LAC_EC_SIZE_QW9_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gfp_521,
                            outRS.mmp_kpt_ecdsa_sign_rs_gfp_521,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GFP_521;
                        break;
                    default:
                        status = CPA_STATUS_INVALID_PARAM;
                        break;
                }
            }
            else
            {
                switch (dataOperationSizeBytes)
                {
                    case LAC_EC_SIZE_QW4_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gf2_l256,
                            outRS.mmp_kpt_ecdsa_sign_rs_gf2_l256,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GF2_L256;
                        break;
                    case LAC_EC_SIZE_QW8_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gf2_l512,
                            outRS.mmp_kpt_ecdsa_sign_rs_gf2_l512,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GF2_L512;
                        break;
                    case LAC_EC_SIZE_QW9_IN_BYTES:
                        LacKptEcdsaSignRSOpDataWrite(
                            inRS.mmp_kpt_ecdsa_sign_rs_gf2_571,
                            outRS.mmp_kpt_ecdsa_sign_rs_gf2_571,
                            pInBuff,
                            pInBuffD,
                            pR,
                            pS);
                        functionID = PKE_KPT_ECDSA_SIGN_RS_GF2_571;
                        break;
                    default:
                        status = CPA_STATUS_INVALID_PARAM;
                        break;
                }
            }

            /*Update size to real size, key cipher text + tag*/
            inArgSizeList[LAC_IDX_OF(
                icp_qat_fw_mmp_kpt_ecdsa_sign_rs_gf2_571_input_t, in)] =
                (LAC_KPT_ECDSA_SIGNRS_NUM_IN_QA_API * dataOperationSizeBytes);
            /* Input memory to QAT is internally allocated */
            internalMemInList[LAC_IDX_OF(
                icp_qat_fw_mmp_kpt_ecdsa_sign_rs_gf2_571_input_t, in)] =
                CPA_TRUE;

            inArgSizeList[LAC_IDX_OF(
                icp_qat_fw_mmp_kpt_ecdsa_sign_rs_gf2_571_input_t, d)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(
                icp_qat_fw_mmp_kpt_ecdsa_sign_rs_gf2_571_input_t, d)] =
                CPA_TRUE;

            /*Set the last node of output array is pKptUnwrapContext*/
            outRS.flat_array[LAC_KPT_ECDSA_SIGNRS_NUM_OUT_ARGS - 1] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            outArgSizeList[LAC_KPT_ECDSA_SIGNRS_NUM_OUT_ARGS - 1] =
                sizeof(CpaCyKptUnwrapContext);

            /* Send pke request */
            if (CPA_STATUS_SUCCESS == status)
            {
                /* create a PKE request to the QAT */
                status = LacPke_CreateRequest(&pRequestHandle,
                                              functionID,
                                              inArgSizeList,
                                              outArgSizeList,
                                              &inRS,
                                              &outRS,
                                              internalMemInList,
                                              internalMemOutList,
                                              LacKptEcdsa_SignRSCallback,
                                              &cbData,
                                              instanceHandle);
            }
            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the field mask */
                lac_pke_qat_req_data_t *pReqData =
                    (lac_pke_qat_req_data_t *)pRequestHandle;
                pReqData->u1.request.pke_hdr.kpt_mask = KPT_ECDSA_SIGNRS_MASK;
                pReqData->u1.request.pke_hdr.kpt_rn_mask =
                    KPT_ECDSA_SIGNRS_RN_MASK;
                /* send request chain */
                status = LacPke_SendRequest(&pRequestHandle, instanceHandle);
            }

            if (CPA_STATUS_SUCCESS != status)
            {
                LacKptMemPoolArrayFree(pMemPoolArray,
                                       pMemPoolConcate,
                                       pMemPoolD,
                                       NULL,
                                       NULL,
                                       NULL);
            }
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* increment stats */
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSRequests, pCryptoService);
    }
    else
    {
        /* increment stats */
        LAC_KPT_ECDSA_STAT_INC(numEcdsaSignRSRequestErrors, pCryptoService);
    }

    return status;
}
