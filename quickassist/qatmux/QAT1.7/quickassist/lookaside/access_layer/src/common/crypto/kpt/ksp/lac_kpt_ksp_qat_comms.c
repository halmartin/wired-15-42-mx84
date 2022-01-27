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
 * @file lac_kpt_ksp_qat_comms.c
 *
 * @ingroup LacKptksp
 *
 * This file implements KSP functions for KPT.
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
#include "lac_kpt_ksp_qat_comms.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "sal_qat_cmn_msg.h"

/*
****************************************************************************
* Define static function definitions
****************************************************************************
*/

/**
 ***************************************************************************
 * @ingroup LacKptKsp
 *      Destroy Ksp requests
 ***************************************************************************/
void LacKpt_Ksp_DestroyRequest(lac_kpt_ksp_request_handle_t *pRequestHandle)
{
    lac_kpt_ksp_qat_req_data_t *pReqData = *pRequestHandle;
    *pRequestHandle = LAC_KPT_KSP_INVALID_HANDLE;
    if (NULL != pReqData->paramInfo.keyformat_viradd)
    {
        LAC_OS_CAFREE(pReqData->paramInfo.keyformat_viradd);
        pReqData->paramInfo.keyformat_viradd = NULL;
    }
    LAC_OS_FREE(pReqData);
}

/**
 ***************************************************************************
 * @ingroup LacKptKsp
 *      Create Ksp requests
 ***************************************************************************/
CpaStatus LacKpt_Ksp_CreateRequest(lac_kpt_ksp_request_handle_t *pRequestHandle,
                                   CpaInstanceHandle instanceHandle,
                                   Cpa8U cmdID,
                                   CpaCyKptHandle kpthandle,
                                   CpaCyKptWrappingFormat *pKptWrappingFormat,
                                   Cpa16U keyselflag,
                                   Cpa8U keyaction,
                                   CpaFlatBuffer *pOutputData,
                                   lac_kpt_ksp_op_cb_func_t pKspOpCbFunc,
                                   lac_kpt_ksp_op_cb_data_t *pCbData)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_kpt_ksp_qat_req_data_t *pReqData = NULL;
    icp_qat_kpt_ksp_key_format_t *pkeyformat = NULL;
    status = LAC_OS_MALLOC(&(pReqData), sizeof(lac_kpt_ksp_qat_req_data_t));
    if (CPA_STATUS_SUCCESS == status)
    {
        if (NULL == pReqData)
        {
            return CPA_STATUS_RESOURCE;
        }
        LAC_OS_BZERO(pReqData, sizeof(lac_kpt_ksp_qat_req_data_t));
        pReqData->paramInfo.keyformat_viradd = NULL;
        pReqData->pHeadReqData = pReqData;
        pReqData->cbinfo.cbFunc = pKspOpCbFunc;
        pReqData->cbinfo.pcbData = pCbData;
        pReqData->cbinfo.instanceHandle = instanceHandle;
        pReqData->u1.request.cmdID = cmdID;
        pReqData->u1.request.serviceType = LAC_KPT_SERVICE_TYPE;
        pReqData->u1.request.kpthandle_l =
            kpthandle & KPTKSP_HANDLE_LOW_DWORD_MASK;
        pReqData->u1.request.kpthandle_h =
            (kpthandle >> KPTKSP_HANDLE_DWORD_LENGTH_INBITS) &
            KPTKSP_HANDLE_LOW_DWORD_MASK;
        pReqData->u1.request.opaque_data = (LAC_ARCH_UINT)pReqData;
        pReqData->u1.request.valid = KPTKSP_REQUEST_VALID_MASK;
        if (NULL != pKptWrappingFormat)
        {
            sal_crypto_service_t *pCryptoService =
                (sal_crypto_service_t *)instanceHandle;
            status = LAC_OS_CAMALLOC(&(pkeyformat),
                                     KPT_KEY_FORMAT_LEN,
                                     LAC_64BYTE_ALIGNMENT,
                                     pCryptoService->nodeAffinity);
            if (CPA_STATUS_SUCCESS == status)
            {
                pkeyformat->alg = pKptWrappingFormat->wrappingAlgorithm;
                pkeyformat->hmac_type = pKptWrappingFormat->hmacType;
                pkeyformat->ic = pKptWrappingFormat->iterationCount;
                pkeyformat->action = keyaction;
                memcpy(pkeyformat->iv,
                       pKptWrappingFormat->iv,
                       CPA_CY_KPT_MAX_IV_LENGTH);
                memset(pkeyformat->state, 0, sizeof(pkeyformat->state));
                pReqData->u1.request.input_data =
                    LAC_OS_VIRT_TO_PHYS_INTERNAL(pkeyformat);
                pReqData->paramInfo.keyformat_viradd = pkeyformat;
            }
        }
        if ((NULL != pOutputData) && (NULL != pOutputData->pData))
        {
            pReqData->u1.request.output_data =
                LAC_OS_VIRT_TO_PHYS_INTERNAL(pOutputData->pData);
        }
        else
        {
            pReqData->u1.request.output_data = 0;
        }
        pReqData->u1.request.keysel_flags = keyselflag;
        *pRequestHandle = (lac_kpt_ksp_request_handle_t)pReqData;
    }
    return status;
}

/**
 ***************************************************************************
 * @ingroup LacKptKsp
 *      Kpt Ksp request create and send to QAT
 ***************************************************************************/
CpaStatus LacKpt_Ksp_SendSingleRequest(
    CpaInstanceHandle instanceHandle,
    Cpa8U cmdID,
    CpaCyKptHandle kpthandle,
    CpaCyKptWrappingFormat *pKptWrappingFormat,
    CpaCyKptKeySelectionFlags keyselflag,
    CpaCyKptKeyAction keyaction,
    CpaFlatBuffer *pOutputData,
    lac_kpt_ksp_op_cb_func_t pKspOpCbFunc,
    lac_kpt_ksp_op_cb_data_t *pCbData)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;
    lac_kpt_ksp_request_handle_t requestHandle = LAC_KPT_KSP_INVALID_HANDLE;
    status = LacKpt_Ksp_CreateRequest(&requestHandle,
                                      instanceHandle,
                                      cmdID,
                                      kpthandle,
                                      pKptWrappingFormat,
                                      keyselflag,
                                      keyaction,
                                      pOutputData,
                                      pKspOpCbFunc,
                                      pCbData);
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_ASSERT_NOT_NULL(requestHandle);
        lac_kpt_ksp_qat_req_data_t *pReqData =
            (lac_kpt_ksp_qat_req_data_t *)requestHandle;

        status = SalQatMsg_transPutMsg(pCryptoService->trans_handle_asym_tx,
                                       (void *)&(pReqData->u1.request),
                                       LAC_QAT_KPT_KSP_REQ_SZ_LW,
                                       LAC_LOG_MSG_KPT_KSP,
                                       NULL);
        if (CPA_STATUS_SUCCESS != status)
        {
            /* destroy the request (chain) */
            LacKpt_Ksp_DestroyRequest(&requestHandle);
        }
    }
    return status;
}
