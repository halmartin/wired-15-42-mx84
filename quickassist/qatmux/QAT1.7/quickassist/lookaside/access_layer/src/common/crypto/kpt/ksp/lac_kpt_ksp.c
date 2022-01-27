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
 * @file lac_kpt_ksp.c
 *
 * @ingroup LacKpt
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
#include "cpa_cy_kpt.h"

/* FW includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp_ids.h"
#include "icp_platform.h"

/* Include LAC files */
#include "lac_common.h"
#include "lac_kpt_ksp_qat_comms.h"
#include "lac_sync.h"
#include "sal_service_state.h"
#include "lac_sal_types_crypto.h"

/* ADF includes */
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_cfg.h"

/* SAL includes */
#include "icp_sal_kpt.h"

#define LAC_KPTKSP_SYNC_CALLBACK_TIMEOUT (2000) /* 2000ms */

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp client callback function
*
* @description
*        This function is called by async call back function when a response is
* received from the QAT to a Kpt Ksp request. cmdID,rspStatus, instanceHandle
* and pCbData are got from ksp response ring.
*
* @param[in] cmdID           Ksp command id
* @param[in] rspStatus       Ksp process result
* @param[in] instanceHandle  Ksp service instance handle
* @param[in] pCbData         Ksp client callback data
*
* @param[out] NULL
***************************************************************************/
STATIC
void LacKpt_ProcessKspCb(CpaStatus status,
                         Cpa8U cmdID,
                         Cpa16U rspStatus,
                         CpaInstanceHandle instanceHandle,
                         lac_kpt_ksp_op_cb_data_t *pCbData);
/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp sync client callback function
*
* @description
*        This function is called by ksp client call back function when a ksp
* response is received from the QAT to a Kpt Ksp request. This function will
* issue sync seamphore to transmit thread
*
* @param[in] pCallbackTag     Pointer to lac_sync_op_data_t structure
* @param[in] status           Client callback function process result
* @param[in] cmdID            Ksp service request command id
* @param[in] rspStatus        Ksp request process result
*
* @param[out] NULL
***************************************************************************/
STATIC
void LacKpt_KspSyncClientCb(void *pCallbackTag,
                            CpaStatus status,
                            Cpa8U cmdID,
                            Cpa16U rspStatus);
/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp sync mode packet sending function
*
* @description
*        This function is called by KPT KSP API to issue KSP request with
* sync up mode.
*
* @param[in] instanceHandle        KSP service instance handle
* @param[in] cmdID                 KSP command id
* @param[in] keyHandle             KPT key handle
* @param[in] pKptWrappingFormat    Pointer to CpaCyKptWrappingFormat
* @param[in] keySelFlag            Flag to indicate which kind of mode
*                                  will be loaded
* @param[in] keyAction             Whether HAMC authentication is needed
* @param[in] pOutputData           FlatBuffer pointer

* @param[out] status               KPT KSP packet send return result
***************************************************************************/
STATIC
CpaStatus LacKptKsp_PktSend(CpaInstanceHandle instanceHandle_in,
                            Cpa8U cmdID,
                            CpaCyKptHandle keyHandle,
                            CpaCyKptWrappingFormat *pKptWrappingFormat,
                            CpaCyKptKeySelectionFlags keySelFlag,
                            CpaCyKptKeyAction keyAction,
                            CpaFlatBuffer *pOutputData,
                            lac_kpt_ksp_op_cb_data_t *pCbData);
/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp client callback function
***************************************************************************/
STATIC
void LacKpt_ProcessKspCb(CpaStatus status,
                         Cpa8U cmdID,
                         Cpa16U rspStatus,
                         CpaInstanceHandle instanceHandle,
                         lac_kpt_ksp_op_cb_data_t *pCbData)
{
    lac_kpt_ksp_sync_cb pCb = NULL;
    void *pCallbackTag = NULL;
    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCallbackTag = (void *)pCbData->pCallbackTag;
    pCbData->cmdID = cmdID;
    pCbData->rspStatus = rspStatus;
    pCb = (lac_kpt_ksp_sync_cb)LAC_CONST_PTR_CAST(pCbData->pClientCb);
    LAC_ASSERT_NOT_NULL(pCb);
    /* invoke the user callback */
    pCb(pCallbackTag, status, cmdID, rspStatus);
}

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp sync mode client callback function
***************************************************************************/
STATIC
void LacKpt_KspSyncClientCb(void *pCallbackTag,
                            CpaStatus status,
                            Cpa8U cmdID,
                            Cpa16U rspStatus)
{
    LacSync_GenWakeupSyncCaller(pCallbackTag, status);
}

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*        Kpt Ksp sync mode packet sending function
***************************************************************************/
STATIC
CpaStatus LacKptKsp_PktSend(CpaInstanceHandle instanceHandle,
                            Cpa8U cmdID,
                            CpaCyKptHandle keyHandle,
                            CpaCyKptWrappingFormat *pKptWrappingFormat,
                            CpaCyKptKeySelectionFlags keyselflag,
                            CpaCyKptKeyAction keyaction,
                            CpaFlatBuffer *pOutputData,
                            lac_kpt_ksp_op_cb_data_t *pCbData)
{
    CpaStatus status = CPA_STATUS_FAIL;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create sync cookie");
        return status;
    }

    pCbData->pCallbackTag = (void *)pSyncCallbackData;
    status = LacKpt_Ksp_SendSingleRequest(instanceHandle,
                                          cmdID,
                                          keyHandle,
                                          pKptWrappingFormat,
                                          keyselflag,
                                          keyaction,
                                          pOutputData,
                                          LacKpt_ProcessKspCb,
                                          pCbData);
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LacSync_WaitForCallback(
            pSyncCallbackData, LAC_KPTKSP_SYNC_CALLBACK_TIMEOUT, &status, NULL);

        if (cmdID != pCbData->cmdID)
        {
            status = CPA_STATUS_FAIL;
            LAC_LOG_ERROR("Error, get a different cmdid from ksp response");
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
* @ingroup LacAsyKptKsp
*        Kpt Ksp register a symmetric wrapping key handle
***************************************************************************/
CpaStatus cpaCyKptRegisterKeyHandle(CpaInstanceHandle instanceHandle_in,
                                    CpaCyKptHandle keyHandle,
                                    CpaCyKptKeyManagementStatus *pkptstatus)
{
    CpaInstanceHandle instanceHandle = NULL;
    Cpa8U cmdID = KPTKSP_REGISTER_HANDLE_CMD;
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;
    if (pCryptoService->kpt_keyhandle_loaded >=
        pCryptoService->maxNumKptKeyHandle)
    {
        *pkptstatus = CPA_CY_KPT_REGISTER_HANDLE_FAIL_INSTANCE_QUOTA_EXCEEDED;
        LAC_LOG_ERROR1("Instance quota exceeded,registered number:%d",
                       pCryptoService->kpt_keyhandle_loaded);
        return CPA_STATUS_FAIL;
    }
    lac_kpt_ksp_op_cb_data_t cbdata = {0};

    /* Initial value of kptstatus is CPA_CY_KPT_FAILED */
    *pkptstatus = CPA_CY_KPT_FAILED;

    cbdata.pClientCb = (void *)LacKpt_KspSyncClientCb;
    cbdata.pCallbackTag = NULL;
    cbdata.cmdID = 0x0;
    cbdata.rspStatus = 0x0;

    status = LacKptKsp_PktSend(
        instanceHandle, cmdID, keyHandle, NULL, 0, 0, NULL, &cbdata);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    *pkptstatus = cbdata.rspStatus;
    switch (*pkptstatus)
    {
        case CPA_CY_KPT_SUCCESS:
        {
            break;
        }
        case CPA_CY_KPT_REGISTER_HANDLE_FAIL_RETRY:
        {
            LAC_LOG("Retry ksp request");
            break;
        }
        case CPA_CY_KPT_REGISTER_HANDLE_FAIL_DUPLICATE:
        {
            LAC_LOG_ERROR("Duplicate key handle");
            break;
        }
        default:
        {
            LAC_LOG_ERROR1("Register keyhandle failed,error code is %d \n",
                           cbdata.rspStatus);
            *pkptstatus = CPA_CY_KPT_FAILED;
            break;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*       Load SWK/WPK from CPM's mailbox to WKT
***************************************************************************/
CpaStatus cpaCyKptLoadKeys(CpaInstanceHandle instanceHandle_in,
                           CpaCyKptHandle keyHandle,
                           CpaCyKptWrappingFormat *pKptWrappingFormat,
                           CpaCyKptKeySelectionFlags keyselflag,
                           CpaCyKptKeyAction keyaction,
                           CpaFlatBuffer *pOutputData,
                           CpaCyKptKeyManagementStatus *pkptstatus)
{
    CpaInstanceHandle instanceHandle = NULL;
    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }
    LAC_CHECK_NULL_PARAM(pKptWrappingFormat);
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif
    Cpa8U cmdID = KPTKSP_LOAD_WKY_CMD;
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;
    lac_kpt_ksp_op_cb_data_t cbdata = {0};

    if (pKptWrappingFormat->wrappingAlgorithm <
            CPA_CY_KPT_WRAPPING_KEY_TYPE_AES128_GCM ||
        pKptWrappingFormat->wrappingAlgorithm >
            CPA_CY_KPT_WRAPPING_KEY_TYPE_AES256_CBC ||
        pKptWrappingFormat->iterationCount == 0 ||
        pKptWrappingFormat->iterationCount >
            KPTKSP_WRAPPED_KEY_MAXIMUM_ITERATION_COUNT ||
        pKptWrappingFormat->hmacType < CPA_CY_KPT_HMAC_TYPE_NULL ||
        pKptWrappingFormat->hmacType > CPA_CY_KPT_HMAC_TYPE_SHA3_512)
    {
        return CPA_STATUS_INVALID_PARAM;
    }

    *pkptstatus = CPA_CY_KPT_FAILED;

    cbdata.pClientCb = (void *)LacKpt_KspSyncClientCb;
    cbdata.pCallbackTag = NULL;
    cbdata.cmdID = 0x0;
    cbdata.rspStatus = 0x0;

    status = LacKptKsp_PktSend(instanceHandle,
                               cmdID,
                               keyHandle,
                               pKptWrappingFormat,
                               keyselflag,
                               keyaction,
                               pOutputData,
                               &cbdata);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    *pkptstatus = cbdata.rspStatus;
    switch (*pkptstatus)
    {
        case CPA_CY_KPT_SUCCESS:
        {
            osalAtomicInc(&(pCryptoService->kpt_keyhandle_loaded));
            break;
        }
        case CPA_CY_KPT_LOAD_KEYS_FAIL_INVALID_HANDLE:
        {
            LAC_LOG_ERROR("Load keys failed, invalid key handle ");
            break;
        }
        case CPA_CY_KPT_REGISTER_HANDLE_FAIL_WKT_FULL:
        {
            LAC_LOG_ERROR("WKT was full");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_HANDLE_NOT_REGISTERED:
        {
            LAC_LOG_ERROR("Key was not registered");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_CHECKSUM_ERROR:
        {
            LAC_LOG_ERROR("Checksum error");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_POSSIBLE_DOS_ATTACK:
        {
            LAC_LOG_ERROR("Possible DoS attack happend");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_INVALID_AC_SEND_HANDLE:
        {
            LAC_LOG_ERROR("Invalid handle got from PTT");
            break;
        }
        default:
        {
            LAC_LOG_ERROR1("Load keys failed,error code is %d \n",
                           cbdata.rspStatus);
            *pkptstatus = CPA_CY_KPT_FAILED;
            break;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*      Delete a SWK entry from wkt according to handle
***************************************************************************/
CpaStatus cpaCyKptDeleteKey(CpaInstanceHandle instanceHandle_in,
                            CpaCyKptHandle keyHandle,
                            CpaCyKptKeyManagementStatus *pkptstatus)
{
    CpaInstanceHandle instanceHandle = NULL;
    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    Cpa8U cmdID = KPTKSP_DEL_WKY_CMD;
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;
    lac_kpt_ksp_op_cb_data_t cbdata = {0};

    *pkptstatus = CPA_CY_KPT_FAILED;

    cbdata.pClientCb = (void *)LacKpt_KspSyncClientCb;
    cbdata.pCallbackTag = NULL;
    cbdata.cmdID = 0x0;
    cbdata.rspStatus = 0x0;

    status = LacKptKsp_PktSend(
        instanceHandle, cmdID, keyHandle, NULL, 0, 0, NULL, &cbdata);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    *pkptstatus = cbdata.rspStatus;
    switch (*pkptstatus)
    {
        case CPA_CY_KPT_SUCCESS:
        {
            osalAtomicDec(&(pCryptoService->kpt_keyhandle_loaded));
            break;
        }
        case CPA_CY_KPT_WKT_ENTRY_NOT_FOUND:
        {
            LAC_LOG_ERROR("Error, entry was not found in WKT");
            break;
        }
        default:
        {
            LAC_LOG_ERROR1("Delete wrapping key failed,error code is %d \n",
                           cbdata.rspStatus);
            *pkptstatus = CPA_CY_KPT_FAILED;
            break;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/**
***************************************************************************
* @ingroup LacAsyKptKsp
*      Perform ECDSA DRNG seed initialization/reseed function based on KPT
*      KSP and PTT function.
***************************************************************************/
CpaStatus icp_sal_kpt_drbg_reseed(const CpaInstanceHandle instanceHandle_in,
                                  PTTFunc pttFunc,
                                  CpaCyKptKeyManagementStatus *pKptStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U rval = 0;
    Cpa32U acHandle = 0;
    sal_crypto_service_t *pCryptoService = NULL;
    CpaInstanceHandle instanceHandle = instanceHandle_in;
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    LAC_CHECK_NULL_PARAM(pttFunc);
    LAC_CHECK_NULL_PARAM(pKptStatus);
#endif
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
#endif
    pCryptoService = (sal_crypto_service_t *)instanceHandle;
    if (!(pCryptoService->generic_service_info.capabilitiesMask &
          ICP_ACCEL_CAPABILITIES_KPT))
    {
        return CPA_STATUS_UNSUPPORTED;
    }

    lac_kpt_ksp_op_cb_data_t cbdata = {0};

    *pKptStatus = CPA_CY_KPT_FAILED;
    cbdata.pClientCb = (void *)LacKpt_KspSyncClientCb;
    cbdata.pCallbackTag = NULL;
    cbdata.cmdID = 0x0;
    cbdata.rspStatus = 0x0;

    status = LacKptKsp_PktSend(instanceHandle,
                               KPTKSP_REGISTER_SEED_HANDLE_CMD,
                               0,
                               NULL,
                               0,
                               0,
                               NULL,
                               &cbdata);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    *pKptStatus = cbdata.rspStatus;
    switch (*pKptStatus)
    {
        case CPA_CY_KPT_SUCCESS:
        {
            break;
        }
        case CPA_CY_KPT_REGISTER_HANDLE_FAIL_RETRY:
        {
            LAC_LOG("Retry ksp request");
            return status;
        }
        default:
        {
            LAC_LOG_ERROR1("Register random handle failed,error code is %d\n",
                           cbdata.rspStatus);
            *pKptStatus = CPA_CY_KPT_FAILED;
            return status;
        }
    }
    acHandle = icp_adf_get_kptAcHandle(pCryptoService->pkgID);
    rval = pttFunc((Cpa64U)acHandle);
    if (0 != rval)
    {
        return CPA_STATUS_FAIL;
    }
    *pKptStatus = CPA_CY_KPT_FAILED;
    cbdata.pClientCb = (void *)LacKpt_KspSyncClientCb;
    cbdata.pCallbackTag = NULL;
    cbdata.cmdID = 0x0;
    cbdata.rspStatus = 0x0;

    status = LacKptKsp_PktSend(instanceHandle,
                               KPTKSP_LOAD_WKY_CMD,
                               0,
                               NULL,
                               CPA_CY_KPT_RN_SEED,
                               0,
                               NULL,
                               &cbdata);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }
    *pKptStatus = cbdata.rspStatus;
    switch (*pKptStatus)
    {
        case CPA_CY_KPT_SUCCESS:
        {
            break;
        }
        case CPA_CY_KPT_LOAD_KEYS_FAIL_INVALID_HANDLE:
        {
            LAC_LOG_ERROR("Load keys failed, invalid key handle ");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_CHECKSUM_ERROR:
        {
            LAC_LOG_ERROR("Checksum error");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_POSSIBLE_DOS_ATTACK:
        {
            LAC_LOG_ERROR("Possible DoS attack happened");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_INVALID_AC_SEND_HANDLE:
        {
            LAC_LOG_ERROR("Invalid handle got from PTT");
            break;
        }
        case CPA_CY_KPT_LOADKEYS_FAIL_INVALID_DATA_OBJ:
        {
            LAC_LOG_ERROR("Invalid data object got from PTT");
            break;
        }
        default:
        {
            LAC_LOG_ERROR1("Load DRNG seed failed,error code is %d \n",
                           cbdata.rspStatus);
            *pKptStatus = CPA_CY_KPT_FAILED;
            return status;
        }
    }
    return status;
}
