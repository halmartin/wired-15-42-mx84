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
 ****************************************************************************
 *
 * @file lac_kpt_ec.c
 *
 * @ingroup Lac_KptEc
 *
 * Elliptic Curve functions
 *
 * @lld_start
 *
 * @lld_overview
 * This file implements KPT Elliptic Curve api funcitons.
 * @lld_dependencies
 * - \ref LacAsymCommonQatComms "PKE QAT Comms" : For creating and sending
 * messages to the QAT
 * - \ref LacMem "Mem" : For memory allocation and freeing, and translating
 * between scalar and pointer types
 * - OSAL : For atomics and logging
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
 ****************************************************************************/
/*

*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
/* API Includes */
#include "cpa.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_kpt.h"

/* OSAL Includes */
#include "Osal.h"

/* ADF Includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* QAT FW includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp.h"
#include "icp_qat_fw_mmp_ids.h"

/* Look Aside Includes */
#include "lac_log.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_pke_utils.h"
#include "lac_pke_qat_comms.h"
#include "lac_sync.h"
#include "lac_ec.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_service_state.h"
#include "sal_statistics.h"
#include "lac_kpt_crypto_qat_comms.h"

#define LAC_KPT_EC_POINT_MULTIPLY_NUM_IN_ARGS 7
/**< number of 'out' arguments in the arguments size list for point multiply */
#define LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS 2

#define KPT_ECCPOINT_MULTI_MASK (0x1)
/**< k' ,xg, yg, a, b, q, h, b0000001 */

#ifndef DISABLE_STATS
#define LAC_KPT_EC_STAT_INC(statistic, pCryptoService)                         \
    do                                                                         \
    {                                                                          \
        if (CPA_TRUE ==                                                        \
            pCryptoService->generic_service_info.stats->bEccStatsEnabled)      \
        {                                                                      \
            osalAtomicInc(                                                     \
                &pCryptoService                                                \
                     ->pLacEcStatsArr[offsetof(CpaCyEcStats64, statistic) /    \
                                      sizeof(Cpa64U)]);                        \
        }                                                                      \
    } while (0)
#else
#define LAC_KPT_EC_STAT_INC(statistic, pCryptoService)                         \
    (pCryptoService) = (pCryptoService)
#endif

#define LacKptEcPointMultiplyOpDataWrite(in, out, pOpData, pXk, pYk)           \
    do                                                                         \
    {                                                                          \
        /* populate input parameters */                                        \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.xg, &pOpData->xg);                    \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.yg, &pOpData->yg);                    \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.h, &pOpData->h);                      \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.q, &pOpData->q);                      \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.a, &pOpData->a);                      \
        LAC_MEM_SHARED_WRITE_FROM_PTR(in.b, &pOpData->b);                      \
                                                                               \
        /* populate output parameters */                                       \
        LAC_MEM_SHARED_WRITE_FROM_PTR(out.xk, pXk);                            \
        LAC_MEM_SHARED_WRITE_FROM_PTR(out.yk, pYk);                            \
    } while (0);

/*
********************************************************************************
* Static Variables
********************************************************************************
*/

/*
********************************************************************************
* Static Functions
********************************************************************************
*/

#ifdef ICP_PARAM_CHECK
/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      EC Point Multiply function to perform basic checks on the IN
 *      parameters (e.g. checks data buffers for NULL and 0 dataLen)
 ***************************************************************************/
STATIC CpaStatus
LacKptEc_PointMultiplyBasicParamCheck(const CpaInstanceHandle instanceHandle,
                                      const CpaCyEcPointMultiplyOpData *pOpData,
                                      const CpaBoolean *pMultiplyStatus,
                                      const CpaFlatBuffer *pXk,
                                      const CpaFlatBuffer *pYk,
                                      const CpaFlatBuffer *pKptUnwrapContext)
{
    /* check for null parameters */
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pMultiplyStatus);

    /* Check flat buffers in pOpData for NULL and dataLen of 0*/
    LAC_CHECK_NULL_PARAM(pOpData->a.pData);
    LAC_CHECK_SIZE(&(pOpData->a), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->b.pData);
    LAC_CHECK_SIZE(&(pOpData->b), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->k.pData);
    LAC_CHECK_SIZE(&(pOpData->k), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->q.pData);
    LAC_CHECK_SIZE(&(pOpData->q), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->xg.pData);
    LAC_CHECK_SIZE(&(pOpData->xg), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pOpData->yg.pData);
    LAC_CHECK_SIZE(&(pOpData->yg), CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pXk);
    LAC_CHECK_NULL_PARAM(pYk);
    LAC_CHECK_NULL_PARAM(pXk->pData);
    LAC_CHECK_SIZE(pXk, CHECK_NONE, 0);
    LAC_CHECK_NULL_PARAM(pYk->pData);
    LAC_CHECK_SIZE(pYk, CHECK_NONE, 0);

    /*Check unwrap context for NULL and dataLen of 0*/
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext);
    LAC_CHECK_NULL_PARAM(pKptUnwrapContext->pData);
    LAC_CHECK_SIZE(pKptUnwrapContext, CHECK_NONE, 0);

    /* Check Cofactor - pData of FlatBuffer can be NULL if dataLenInBytes=0 */
    if ((NULL == pOpData->h.pData) && (0 != pOpData->h.dataLenInBytes))
    {
        LAC_INVALID_PARAM_LOG("pOpData->h.pData is NULL and "
                              "pOpData->h.dataLenInBytes !=0");
        return CPA_STATUS_INVALID_PARAM;
    }
    if (NULL != pOpData->h.pData)
    {
        LAC_CHECK_SIZE(&(pOpData->h), CHECK_NONE, 0);
    }

    if (CPA_CY_EC_FIELD_TYPE_PRIME != pOpData->fieldType &&
        CPA_CY_EC_FIELD_TYPE_BINARY != pOpData->fieldType)
    {
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Check that q is odd */
    LAC_CHECK_ODD_PARAM(&(pOpData->q));

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *    KPT  EC Point Multiply internal callback
 ***************************************************************************/
STATIC void LacKptEc_PointMultiplyCallback(CpaStatus status,
                                           CpaBoolean multiplyStatus,
                                           CpaInstanceHandle instanceHandle,
                                           lac_pke_op_cb_data_t *pCbData)
{
    CpaCyEcPointMultiplyCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyEcPointMultiplyOpData *pOpData = NULL;
    CpaFlatBuffer *pXk = NULL;
    CpaFlatBuffer *pYk = NULL;
    Cpa8U *pMemPoolK = NULL;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;

    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCb = (CpaCyEcPointMultiplyCbFunc)pCbData->pClientCb;
    pCallbackTag = pCbData->pCallbackTag;
    pOpData = (CpaCyEcPointMultiplyOpData *)pCbData->pClientOpData;
    pXk = pCbData->pOutputData1;
    pYk = pCbData->pOutputData2;
    pMemPoolK = (Cpa8U *)(pCbData->pOpaqueData);

    LAC_ASSERT_NOT_NULL(pCb);
    LAC_ASSERT_NOT_NULL(pOpData);
    LAC_ASSERT_NOT_NULL(pXk);
    LAC_ASSERT_NOT_NULL(pYk);
    LAC_ASSERT_NOT_NULL(pMemPoolK);

    LacKptMemPoolFree(pMemPoolK);

    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyCompleted, pCryptoService);
    }
    else
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyCompletedError, pCryptoService);
    }

    if ((CPA_FALSE == multiplyStatus) && (CPA_STATUS_SUCCESS == status))
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyCompletedOutputInvalid,
                            pCryptoService);
    }

    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, multiplyStatus, pXk, pYk);
}

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      return the size in bytes of biggest number in CpaCyEcPointMultiplyOpData
 *
 * @description
 *      return the size of the biggest number in
 *      CpaCyEcPointMultiplyOpData.
 *
 * @param[in]  pOpData      Pointer to a CpaCyEcPointMultiplyOpData structure
 *
 * @retval max  the size in bytes of the biggest number
 *
 ***************************************************************************/
STATIC Cpa32U LacKptEc_PointMultiplyOpDataSizeGetMax(
    const CpaCyEcPointMultiplyOpData *pOpData,
    Cpa32U wpkSize)
{
    Cpa32U max = 0;

    /* need to find max size in bytes of number in input buffers */
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->xg)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->yg)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->h)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->q)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->a)), max);
    max = LAC_MAX(LacPke_GetMinBytes(&(pOpData->b)), max);
    max = LAC_MAX(wpkSize, max);

    return max;
}

/**
 ***************************************************************************
 * @ingroup Lac_KptEc
 *      EC Point Multiply synchronous function
 ***************************************************************************/
STATIC CpaStatus
LacKptEc_PointMultiplySyn(const CpaInstanceHandle instanceHandle,
                          const CpaCyEcPointMultiplyOpData *pOpData,
                          CpaBoolean *pMultiplyStatus,
                          CpaFlatBuffer *pXk,
                          CpaFlatBuffer *pYk,
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
        status = cpaCyKptEcPointMultiply(instanceHandle,
                                         LacSync_GenDualFlatBufVerifyCb,
                                         pSyncCallbackData,
                                         pOpData,
                                         pMultiplyStatus,
                                         pXk,
                                         pYk,
                                         pKptUnwrapContext);
    }
    else
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyRequestErrors, pCryptoService);
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
            LAC_KPT_EC_STAT_INC(numEcPointMultiplyCompletedError,
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
 *
 ***************************************************************************/
CpaStatus cpaCyKptEcPointMultiply(const CpaInstanceHandle instanceHandle_in,
                                  const CpaCyEcPointMultiplyCbFunc pCb,
                                  void *pCallbackTag,
                                  const CpaCyEcPointMultiplyOpData *pOpData,
                                  CpaBoolean *pMultiplyStatus,
                                  CpaFlatBuffer *pXk,
                                  CpaFlatBuffer *pYk,
                                  CpaFlatBuffer *pKptUnwrapContext)

{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U dataOperationSizeBytes = 0;
    CpaInstanceHandle instanceHandle = NULL;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;

#ifdef ICP_PARAM_CHECK
    Cpa32U bit_pos_q = 0;
    Cpa32U temp = 0;
    Cpa32U maxModLen = 0;
    CpaBoolean isZero = CPA_FALSE;
#endif
    sal_crypto_service_t *pCryptoService;

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
    /* ensure LAC is running - return error if not */
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
        status = LacKptEc_PointMultiplySyn(instanceHandle,
                                           pOpData,
                                           pMultiplyStatus,
                                           pXk,
                                           pYk,
                                           pKptUnwrapContext);
        LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                 "0x%lx, 0x%lx, 0x%lx)\n",
                 (LAC_ARCH_UINT)instanceHandle,
                 (LAC_ARCH_UINT)pCb,
                 (LAC_ARCH_UINT)pCallbackTag,
                 (LAC_ARCH_UINT)pOpData,
                 (LAC_ARCH_UINT)pXk,
                 (LAC_ARCH_UINT)pYk,
                 (LAC_ARCH_UINT)pKptUnwrapContext);
        return status;
#else
        /* Call synchronous mode function */
        return LacKptEc_PointMultiplySyn(instanceHandle,
                                         pOpData,
                                         pMultiplyStatus,
                                         pXk,
                                         pYk,
                                         pKptUnwrapContext);
#endif
    }

#ifdef ICP_PARAM_CHECK
    /* Basic NULL Param Checking  */
    status = LacKptEc_PointMultiplyBasicParamCheck(
        instanceHandle, pOpData, pMultiplyStatus, pXk, pYk, pKptUnwrapContext);
    /* Check that output buffers are big enough */
    if (CPA_STATUS_SUCCESS == status)
    {
        maxModLen = LacPke_GetMinBytes(&(pOpData->q));
        if ((pXk->dataLenInBytes < maxModLen) ||
            (pYk->dataLenInBytes < maxModLen))
        {
            LAC_INVALID_PARAM_LOG("Output buffers not big enough");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
#endif
        /* Fetch wpk size from unwrapcontext*/
        pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
        pWpkSize = &(pUnwrapCxt->wpkSize);
        /* Determine size - based on input numbers */
        status = LacKptEc_GetRange(LacKptEc_PointMultiplyOpDataSizeGetMax(
                                       pOpData, pWpkSize->wpkLenInBytes),
                                   &dataOperationSizeBytes);
#ifdef ICP_PARAM_CHECK
    }
#endif

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
                                   NULL,
                                   &(pOpData->h));
        }
    }

#ifdef ICP_PARAM_CHECK
    if ((CPA_STATUS_SUCCESS == status) &&
        (LAC_EC_SIZE_QW9_IN_BYTES == dataOperationSizeBytes))
    {
        /* 9QW checks */
        if (CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType)
        {
            /* Check if it is a NIST curve (if not, then  invalid param) */
            /* Also checks that xG and yG are less than 2^521 */
            status = LacEc_CheckCurve9QWGFP(&(pOpData->q),
                                            &(pOpData->a),
                                            &(pOpData->b),
                                            NULL,
                                            &(pOpData->h),
                                            &(pOpData->xg),
                                            &(pOpData->yg));
        }
        else
        {
            /*Check if it is a NIST curve (if not, invalid param) */
            /* Also checks that deg xG and yG are less than deg q */
            status = LacEc_CheckCurve9QWGF2(&(pOpData->q),
                                            &(pOpData->a),
                                            &(pOpData->b),
                                            NULL,
                                            &(pOpData->h),
                                            &(pOpData->xg),
                                            &(pOpData->yg));
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Ensure that h!=0 */
        /* This is invalid for all secure curves */
        /* pH=NULL is sent to PKE as h=0 which is understood by MMP program as
           h=1
           as required - therefore need to eliminate invalid h=0 case here */
        if (NULL != pOpData->h.pData)
        {
            if (0 == LacPke_CompareZero(&(pOpData->h), 0))
            {
                LAC_INVALID_PARAM_LOG("Cofactor == 0");
                status = CPA_STATUS_INVALID_PARAM;
            }
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check that q>3 for GFP (i.e. highest bit position needs to be greater
           than 1) or that deg(q)>2 for GF2 (i.e. highest bit position needs to
           be greater than 2) */
        LacPke_GetBitPos(&(pOpData->q), &bit_pos_q, &temp, &isZero);
        if (((CPA_CY_EC_FIELD_TYPE_BINARY == pOpData->fieldType) &&
             (bit_pos_q < LAC_EC_MIN_MOD_BIT_POS_GF2)) ||
            ((CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType) &&
             (bit_pos_q < LAC_EC_MIN_MOD_BIT_POS_GFP)))
        {
            LAC_INVALID_PARAM_LOG("q is not as required - too small");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }
#endif

    if (CPA_STATUS_SUCCESS == status)
    {
        icp_qat_fw_mmp_input_param_t in = {.flat_array = {0}};
        icp_qat_fw_mmp_output_param_t out = {.flat_array = {0}};
        lac_pke_op_cb_data_t cbData = {0};
        Cpa8U *pMemPoolK = NULL;
        CpaFlatBuffer *pInBuffK = NULL;
        pCryptoService = (sal_crypto_service_t *)instanceHandle;
        status = LacKptMemPoolMalloc(&pMemPoolK, pCryptoService->lac_kpt_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            /* Holding the calculated size of the input/output parameters */
            Cpa32U inArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
            Cpa32U outArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};

            CpaBoolean internalMemInList[LAC_MAX_MMP_INPUT_PARAMS] = {
                CPA_FALSE};
            CpaBoolean internalMemOutList[LAC_MAX_MMP_OUTPUT_PARAMS] = {
                CPA_FALSE};

            Cpa32U functionID = 0;

            /* Zero the output buffers */
            osalMemSet(pXk->pData, 0, pXk->dataLenInBytes);
            osalMemSet(pYk->pData, 0, pYk->dataLenInBytes);

            /* Construct the kpt wpk buf*/
            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolK, pWpkSize->wpkLenInBytes, &(pOpData->k));
            pInBuffK = (CpaFlatBuffer *)(pMemPoolK + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffK->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffK->pData = pMemPoolK;
            /* populate callback data */
            cbData.pClientCb = pCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pOpData;
            cbData.pOpaqueData = pMemPoolK;
            cbData.pOutputData1 = pXk;
            cbData.pOutputData2 = pYk;

            /* Set the size for all parameters to be padded to */
            LAC_EC_SET_LIST_PARAMS(inArgSizeList,
                                   LAC_KPT_EC_POINT_MULTIPLY_NUM_IN_ARGS,
                                   dataOperationSizeBytes);

            LAC_EC_SET_LIST_PARAMS(outArgSizeList,
                                   LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS,
                                   dataOperationSizeBytes);

            /* Set all input and output memory to externally allocated */
            LAC_EC_SET_LIST_PARAMS(internalMemInList,
                                   LAC_KPT_EC_POINT_MULTIPLY_NUM_IN_ARGS,
                                   CPA_FALSE);
            LAC_EC_SET_LIST_PARAMS(internalMemOutList,
                                   LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS,
                                   CPA_FALSE);
            /* Populate input and output buffers for MMP and set function ID */

            /* Note datalenInBytes of input flatbuffers can be greater than
               dataOperationSizeBytes */
            /* CreateRequest() allows for this
                          - pke buffer will offset into client buffer */
            if (CPA_CY_EC_FIELD_TYPE_PRIME == pOpData->fieldType)
            {
                switch (dataOperationSizeBytes)
                {
                    case LAC_EC_SIZE_QW4_IN_BYTES:
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gfp_l256,
                            out.maths_point_multiplication_gfp_l256,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GFP_L256;
                        break;
                    case LAC_EC_SIZE_QW8_IN_BYTES:
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gfp_l512,
                            out.maths_point_multiplication_gfp_l512,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GFP_L512;
                        break;
                    case LAC_EC_SIZE_QW9_IN_BYTES:
                    {
                        Cpa32U index = 0;
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gfp_521,
                            out.maths_point_multiplication_gfp_521,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GFP_521;
                        /* cofactor size is 1 qw */
                        index = LAC_IDX_OF(
                            icp_qat_fw_maths_point_multiplication_gfp_521_input_t,
                            h);
                        LAC_ASSERT(LAC_MAX_MMP_INPUT_PARAMS > index,
                                   "invalid cofactor index");
                        inArgSizeList[index] = LAC_QUAD_WORD_IN_BYTES;
                        break;
                    }
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
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gf2_l256,
                            out.maths_point_multiplication_gf2_l256,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GF2_L256;
                        break;
                    case LAC_EC_SIZE_QW8_IN_BYTES:
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gf2_l512,
                            out.maths_point_multiplication_gf2_l512,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GF2_L512;
                        break;
                    case LAC_EC_SIZE_QW9_IN_BYTES:
                    {
                        Cpa32U index = 0;
                        LacKptEcPointMultiplyOpDataWrite(
                            in.maths_point_multiplication_gf2_571,
                            out.maths_point_multiplication_gf2_571,
                            pOpData,
                            pXk,
                            pYk);
                        functionID = MATHS_POINT_MULTIPLICATION_GF2_571;
                        /* cofactor size is 1 qw */
                        index = LAC_IDX_OF(
                            icp_qat_fw_maths_point_multiplication_gf2_571_input_t,
                            h);
                        LAC_ASSERT(LAC_MAX_MMP_INPUT_PARAMS > index,
                                   "invalid cofactor index");
                        inArgSizeList[index] = LAC_QUAD_WORD_IN_BYTES;
                        break;
                    }
                    default:
                        status = CPA_STATUS_INVALID_PARAM;
                        break;
                }
            }

            /*Update size to real size, key cipher text + tag*/
            inArgSizeList[LAC_IDX_OF(
                icp_qat_fw_maths_point_multiplication_gfp_521_input_t, k)] =
                LAC_KPT_SIZE_BYTES_MAX;
            LAC_MEM_SHARED_WRITE_FROM_PTR(
                in.maths_point_multiplication_gfp_521.k, pInBuffK);
            internalMemInList[LAC_IDX_OF(
                icp_qat_fw_maths_point_multiplication_gfp_521_input_t, k)] =
                CPA_TRUE;

            /*Set the last node of output array is pKptUnwrapContext*/
            out.flat_array[LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            outArgSizeList[LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS] =
                sizeof(CpaCyKptUnwrapContext);
            internalMemOutList[LAC_KPT_EC_POINT_MULTIPLY_NUM_OUT_ARGS] =
                CPA_FALSE;

            if (CPA_STATUS_SUCCESS == status)
            {
                lac_pke_request_handle_t pRequestHandle =
                    LAC_PKE_INVALID_HANDLE;
                /* create a PKE request to the QAT */
                status = LacPke_CreateRequest(&pRequestHandle,
                                              functionID,
                                              inArgSizeList,
                                              outArgSizeList,
                                              &in,
                                              &out,
                                              internalMemInList,
                                              internalMemOutList,
                                              LacKptEc_PointMultiplyCallback,
                                              &cbData,
                                              instanceHandle);
                if (CPA_STATUS_SUCCESS == status)
                {
                    /* set the field mask */
                    lac_pke_qat_req_data_t *pReqData =
                        (lac_pke_qat_req_data_t *)pRequestHandle;
                    pReqData->u1.request.pke_hdr.kpt_mask =
                        KPT_ECCPOINT_MULTI_MASK;
                    /* send request chain */
                    status =
                        LacPke_SendRequest(&pRequestHandle, instanceHandle);
                }
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LacKptMemPoolFree(pMemPoolK);
        }
    }

    pCryptoService = (sal_crypto_service_t *)instanceHandle;

    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyRequests, pCryptoService);
    }
    else
    {
        LAC_KPT_EC_STAT_INC(numEcPointMultiplyRequestErrors, pCryptoService);
    }
    return status;
}
