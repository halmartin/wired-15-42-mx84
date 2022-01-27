/*************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
 * 
 *  version: QAT1.5.L.1.11.0-36
 *
 *****************************************************************************/

/**
 *****************************************************************************
 * @file lac_sym_nrbg_api.c
 *
 * @ingroup LacSym_Nrbg
 *
 * Implementation of the Non-Deterministic Random Bit Generation API
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"
#include "cpa_cy_nrbg.h"
#include "icp_sal_nrbg_ht.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "icp_adf_transport.h"
#ifdef DRBG_POLL_AND_WAIT
#include "icp_adf_poll.h"
#include "icp_sal_poll.h"
#endif
#include "sal_service_state.h"
#include "lac_sym.h"
#include "lac_sym_alg_chain.h"
#include "lac_sym_queue.h"
#include "lac_sym.h"
#include "lac_sym_qat_cipher.h"
#include "lac_buffer_desc.h"
#include "lac_mem_pools.h"
#include "lac_sal_types_crypto.h"
#include "lac_common.h"
#include "lac_mem.h"

/**
 *******************************************************************************
 * @ingroup LacSym_Nrbg
 *      This macro verifies that the device possesses non-deterministic random
 *      number generation capability.
 *
 * @param[in] instanceHandle       Instance handle
 *
 * @return CPA_STATUS_UNSUPPORTED  Capability is not supported
 *
 ******************************************************************************/
#define LAC_SYM_CHECK_NRBG_CAPABILITY(instanceHandle)                       \
do {                                                                        \
    sal_service_t* pGenericService = NULL;                                  \
    pGenericService = (sal_service_t*)instanceHandle;                       \
    if(!(pGenericService->capabilitiesMask &                                \
            ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER))                          \
    {                                                                       \
        LAC_LOG_ERROR("The device does not support NRBG capability");       \
        return CPA_STATUS_UNSUPPORTED;                                      \
    }                                                                       \
} while(0)                                                                  \

/**
 * LacSymNrbg_PerformSyncOp
 */
STATIC CpaStatus
LacSymNrbg_PerformSyncOp(CpaInstanceHandle instanceHandle,
                      void *pCallbackTag,
                      const CpaCyNrbgOpData *pOpData,
                      CpaFlatBuffer *pOutputData,
                      icp_qat_fw_la_cmd_id_t commandId);



/**
 ******************************************************************************
 * @ingroup LacSym_Nrbg
 *      NRBG Response Handler
 *
 * @description
 *      This function is called by QAT callback function to process
 *      the NRBG response messages.
 *
 * @param[in]  commandId             LA command id.
 * @param[in]  pRespData             Pointer to response message data
 * @param[in]  cmnRespFlags          Common response flags
 *
 * @return None
 *
 *****************************************************************************/
STATIC void
LacSymNrbg_ResponseHandler(icp_qat_fw_la_cmd_id_t commandId,
                        void *pRespData,
                        icp_qat_fw_comn_flags cmnRespFlags)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_sym_nrbg_cookie_t *pCookie = NULL;
    CpaCyGenFlatBufCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    void *pOpData = NULL;
    CpaFlatBuffer *pOutputData = NULL;
    Cpa32U *pContinuousRngTestFailures = NULL;

    LAC_ASSERT(pRespData,
               "LacSymNrbg_ResponseHandler() - pRespData is NULL\n");

    pCookie = (lac_sym_nrbg_cookie_t *)pRespData;

    /* Set response status */
    if (ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
            ICP_QAT_FW_COMN_RESP_CRYPTO_STAT_GET(cmnRespFlags))
    {
        status = CPA_STATUS_SUCCESS;
    }
    else
    {
        status = CPA_STATUS_FAIL;
    }

    pCb = pCookie->pCb;
    LAC_ASSERT_NOT_NULL(pCb);

    pCallbackTag = pCookie->pCallbackTag;
    pOpData = pCookie->pOpData;
    pOutputData = pCookie->pOutputData;

    if (ICP_QAT_FW_LA_CMD_TRNG_TEST == commandId &&
            CPA_STATUS_SUCCESS == status)
    {
        /* Set the pContinuousRngTestFailures */
        pContinuousRngTestFailures = (Cpa32U *)pOutputData->pData;

        *pContinuousRngTestFailures =
            pCookie->trngHTResult.test_status_fail_count;
    }

    Lac_MemPoolEntryFree(pCookie);

    pCb(pCallbackTag, status, pOpData, pOutputData);
}

/**
 ******************************************************************************
 * @ingroup LacSym_Nrbg
 *      NRBG Perform Operation
 *
 * @description
 *      This function performs the NRBG specific operation based on the LA
 *      command id.
 *
 * @param[in]  instanceHandle       Instance handle.
 * @param[in]  pCb                  Pointer to callback function to be invoked
 *                                  when the operation is complete. If this is
 *                                  set to a NULL value the function will
 *                                  operate synchronously.
 * @param[in]  pCallbackTag         Opaque User Data for this specific call. It
 *                                  will be returned unchanged in the callback.
 * @param[in]  pOpData              Structure containing all the data needed to
 *                                  perform operation.
 * @param[out] pOutputData          Pointer to memory allocated by the caller
 *                                  to which the output of the operation will
 *                                  be written.
 * @param[in]  commandId            LA command ID for this operation.
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully.
 * @retval CPA_STATUS_FAIL          Function failed.
 * @retval CPA_STATUS_RETRY         Resubmit the request.
 * @retval CPA_STATUS_RESOURCE      Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymNrbg_PerformOp(CpaInstanceHandle instanceHandle,
                  const CpaCyGenFlatBufCbFunc pCb,
                  void *pCallbackTag,
                  const CpaCyNrbgOpData *pOpData,
                  CpaFlatBuffer *pOutputData,
                  icp_qat_fw_la_cmd_id_t commandId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t *pService = (sal_crypto_service_t*) instanceHandle;
    lac_sym_nrbg_cookie_t *pCookie = NULL;
    icp_qat_fw_la_trng_req_t *pTrngReq = NULL;
    icp_qat_fw_comn_flags cmnReqFlags = 0;
    Cpa64U destDataAddr = 0;

    if (pService->executionEngine)
    {
        LAC_LOG_ERROR("Invalid Accelerator Number for NRBG Operations");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Synchronous mode */
    if (NULL == pCb)
    {
        return LacSymNrbg_PerformSyncOp(instanceHandle,
                                     pCallbackTag,
                                     pOpData,
                                     pOutputData,
                                     commandId);
    }

    /* Setup Cookie */
    pCookie = (lac_sym_nrbg_cookie_t*)
                Lac_MemPoolEntryAlloc(pService->lac_sym_cookie_pool);

    if ((NULL == pCookie) || ((void*)CPA_STATUS_RETRY == pCookie))
    {
        LAC_LOG_ERROR("Cannot get mem pool entry");
        return CPA_STATUS_RESOURCE;
    }

    pCookie->instanceHandle = instanceHandle;
    pCookie->pCb = pCb;
    pCookie->pCallbackTag = pCallbackTag;
    pCookie->pOpData = LAC_CONST_PTR_CAST(pOpData);
    pCookie->pOutputData = pOutputData;

    /* Initialise Request Message */
    pTrngReq = &(pCookie->trngReq);
    LAC_OS_BZERO(pTrngReq, sizeof(icp_qat_fw_la_trng_req_t));

    /* Setup NRBG Common Header */
    SalQatMsg_ReqTypePopulate(&(pTrngReq->comn_hdr.arch_if),
                              ICP_ARCH_IF_REQ_QAT_FW_LA,
                              pService->symLoResponseRingId);

    cmnReqFlags = ICP_QAT_FW_COMN_FLAGS_BUILD(
                               ICP_QAT_FW_COMN_ORD_FLAG_STRICT,
                               QAT_COMN_PTR_TYPE_FLAT,
                               ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,
                               QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                               QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                               QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                               QAT_COMN_CPR_SLICE_NOT_REQUIRED,
                               QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                               QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                               QAT_COMN_RND_SLICE_REQUIRED,
                               QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                               QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                               QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                               QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                               QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                               QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);

    pTrngReq->comn_hdr.comn_req_flags = cmnReqFlags;

    LAC_MEM_SHARED_WRITE_FROM_PTR(pTrngReq->comn_mid.opaque_data, pCookie);
    pTrngReq->comn_la_req.la_cmd_id = commandId;

    /* NRBG Get Random */
    if (ICP_QAT_FW_LA_CMD_TRNG_GET_RANDOM == commandId)
    {
        /* Entropy destination data address */
        destDataAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                LAC_OS_VIRT_TO_PHYS_EXTERNAL(pService->generic_service_info,
                                             pOutputData->pData));

        if (destDataAddr == 0)
        {
            LAC_LOG_ERROR("Unable to get the physical address of the"
            "destination address\n");
            return CPA_STATUS_FAIL;
        }

        pTrngReq->comn_mid.dest_data_addr = destDataAddr;

        /* Entropy length */
        pTrngReq->u.entropy_length = pOpData->lengthInBytes;
    }

    /* NRBG Health Test */
    if (ICP_QAT_FW_LA_CMD_TRNG_TEST == commandId)
    {
        /* Setup internal HT result buffer */
        LAC_OS_BZERO(&(pCookie->trngHTResult),
            sizeof(icp_qat_fw_la_trng_test_result_t));

        LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_INTERNAL(
            pTrngReq->comn_mid.dest_data_addr, &(pCookie->trngHTResult));
    }

    /* Send message */
    status = icp_adf_transPutMsg(pService->trans_handle_sym_tx_lo,
                                (void*)pTrngReq,
                                LAC_QAT_MSG_SZ_LW);

    if (CPA_STATUS_SUCCESS != status)
    {
        /* Clean up cookie memory */
        if (NULL != pCookie)
        {
            Lac_MemPoolEntryFree(pCookie);
        }
    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_Nrbg
 *      NRBG Perform Synchronous Operation
 *
 * @description
 *      This is a wrapper function to enable LacSymNrbg_PerformOp() to
 *      operate in synchronous mode.
 *
 * @param[in]  instanceHandle       Instance handle.
 * @param[in]  pCallbackTag         Opaque User Data for this specific call.
 * @param[in]  pOpData              Structure containing all the data needed
 *                                  to perform operation.
 * @param[out] pOutputData          Pointer to memory allocated by the client
 *                                  to which the output of the operation will
 *                                  be written.
 * @param[in]  commandId            LA command ID for this operation.
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully.
 * @retval CPA_STATUS_FAIL          Function failed.
 * @retval CPA_STATUS_RETRY         Resubmit the request.
 * @retval CPA_STATUS_RESOURCE      Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymNrbg_PerformSyncOp(CpaInstanceHandle instanceHandle,
                      void *pCallbackTag,
                      const CpaCyNrbgOpData *pOpData,
                      CpaFlatBuffer *pOutputData,
                      icp_qat_fw_la_cmd_id_t commandId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
#ifdef DRBG_POLL_AND_WAIT
    sal_crypto_service_t *pService = (sal_crypto_service_t*) instanceHandle;
    Cpa32U retries=0;
    Cpa32U drbgPollAndWaitTime=0;

    drbgPollAndWaitTime = pService->drbgPollAndWaitTime;
#endif

    status = LacSync_CreateSyncCookie(&pSyncCallbackData);

    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    status = LacSymNrbg_PerformOp(instanceHandle,
                               LacSync_GenFlatBufCb,
                               pSyncCallbackData,
                               pOpData,
                               pOutputData,
                               commandId);
    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus waitCbStatus = CPA_STATUS_FAIL;

#ifdef DRBG_POLL_AND_WAIT
        retries=0;
        do
        {
            waitCbStatus = icp_adf_pollInstance(
                               &(pService->trans_handle_sym_rx_lo), 1, 0);
            if (CPA_STATUS_FAIL == waitCbStatus)
            {
                LAC_LOG_ERROR("Poll Instance Failed");
                break;
            }
            waitCbStatus = LacSync_CheckForCallback(pSyncCallbackData,
                                        &status,
                                        NULL);
            if (CPA_STATUS_RETRY == waitCbStatus)
            {
                osalSleep(drbgPollAndWaitTime);
            }
            retries++;
        } while ((waitCbStatus != CPA_STATUS_SUCCESS)&&
                (retries<(LAC_SYM_NRBG_SYNC_CALLBACK_TIMEOUT /
                          drbgPollAndWaitTime)));

        if (CPA_STATUS_SUCCESS != waitCbStatus)
        {
            LAC_LOG_ERROR("Callback timed out");
            status = waitCbStatus;
        }
#else
        waitCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                        LAC_SYM_NRBG_SYNC_CALLBACK_TIMEOUT,
                                        &status,
                                        NULL);

        if (CPA_STATUS_SUCCESS != waitCbStatus)
        {
            LAC_LOG_ERROR("Callback timed out");
            status = waitCbStatus;
        }
#endif
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
 * cpaCyNrbgGetEntropy
 */
CpaStatus
cpaCyNrbgGetEntropy(const CpaInstanceHandle instanceHandle_in,
        const CpaCyGenFlatBufCbFunc pCb,
        void *pCallbackTag,
        const CpaCyNrbgOpData *pOpData,
        CpaFlatBuffer *pEntropy)
{
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pOpData,
            (LAC_ARCH_UINT)pEntropy);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
#endif

    /* check that the device supports random number capability */
    LAC_SYM_CHECK_NRBG_CAPABILITY(instanceHandle);

#ifdef ICP_PARAM_CHECK
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pEntropy);
    LAC_CHECK_NULL_PARAM(pEntropy->pData);

    /* Ensure lengthInBytes is non-zero */
    if (0 == pOpData->lengthInBytes)
    {
        LAC_INVALID_PARAM_LOG("pOpData->lengthInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Check for enough space in the flat buffer */
    if (pOpData->lengthInBytes > pEntropy->dataLenInBytes)
    {
        LAC_INVALID_PARAM_LOG("pEntropy->dataLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /* check crypto service is running otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO_A);

    return LacSymNrbg_PerformOp(instanceHandle,
                             pCb,
                             pCallbackTag,
                             pOpData,
                             pEntropy,
                             ICP_QAT_FW_LA_CMD_TRNG_GET_RANDOM);

}

/**
 * icp_sal_nrbgHealthTest
 */
CpaStatus
icp_sal_nrbgHealthTest(const CpaInstanceHandle instanceHandle_in,
                    Cpa32U *pContinuousRngTestFailures)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    CpaFlatBuffer *pTrngTestResult = NULL;

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
#endif

    /* check that the device supports random number capability */
    LAC_SYM_CHECK_NRBG_CAPABILITY(instanceHandle);

#ifdef ICP_PARAM_CHECK
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    LAC_CHECK_NULL_PARAM(pContinuousRngTestFailures);
#endif

    /* check crypto service is running otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO_A);

    status = LAC_OS_MALLOC(&pTrngTestResult, sizeof(CpaFlatBuffer));

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Cannot allocate test result buffer.");
        return status;
    }

    /* Setup TRNG test result buffer */
    pTrngTestResult->dataLenInBytes = sizeof(Cpa32U);
    pTrngTestResult->pData = (Cpa8U *)pContinuousRngTestFailures;

    status = LacSymNrbg_PerformOp(instanceHandle,
                                  NULL, NULL, NULL,
                                  pTrngTestResult,
                                  ICP_QAT_FW_LA_CMD_TRNG_TEST);

    LAC_OS_FREE(pTrngTestResult);

    return status;
}

/**
 * LacSymNrbg_Init
 */
void
LacSymNrbg_Init(void)
{
    /* NRBG Get Random */
    LacSymQat_RespHandlerRegister(
                                  ICP_QAT_FW_LA_CMD_TRNG_GET_RANDOM,
                                  LacSymNrbg_ResponseHandler);

    /* NRBG Test */
    LacSymQat_RespHandlerRegister(
                                  ICP_QAT_FW_LA_CMD_TRNG_TEST,
                                  LacSymNrbg_ResponseHandler);
}
