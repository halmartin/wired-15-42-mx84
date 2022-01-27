/***************************************************************************
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
 ***************************************************************************/

/**
 *****************************************************************************
 * @file lac_sym_key.c
 *
 * @ingroup LacSymKey
 *
 * This file contains the implementation of all keygen functionality
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"
#include "cpa_cy_key.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_platform.h"
#include "lac_sym.h"
#include "lac_sym_qat_hash_defs_lookup.h"
#include "lac_sym_qat.h"
#include "lac_sym_key.h"
#include "lac_sal_types_crypto.h"
#include "sal_service_state.h"
#include "lac_sym_qat_key.h"
#include "lac_sym_hash_defs.h"
#include "sal_statistics.h"

/* Number of statistics */
#define LAC_KEY_NUM_STATS (sizeof(CpaCyKeyGenStats64) / sizeof(Cpa64U))

#ifndef DISABLE_STATS
#define LAC_KEY_STAT_INC(statistic, instanceHandle)                           \
do {                                                                          \
    sal_crypto_service_t* pService = NULL;                                    \
    LAC_ENSURE(instanceHandle != NULL,                                        \
                          "LAC_KEY_STAT_INC - instanceHandle NULL\n");        \
    pService = (sal_crypto_service_t*) instanceHandle;                        \
    if(CPA_TRUE == pService->generic_service_info.stats->bKeyGenStatsEnabled) \
    {                                                                         \
        osalAtomicInc(                                                        \
            &pService->pLacKeyStats[                                          \
            offsetof(CpaCyKeyGenStats64, statistic) / sizeof(Cpa64U)]);       \
    }                                                                         \
} while (0)
/**< macro to increment a Key stat (derives offset into array of atomics) */
#else
#define LAC_KEY_STAT_INC(statistic, instanceHandle)
#endif


#define LAC_KEY_STATS32_GET(keyStats, instanceHandle)                       \
do {                                                                        \
    int i;                                                                  \
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;\
    for (i = 0; i < LAC_KEY_NUM_STATS; i++)                                 \
    {                                                                       \
        ((Cpa32U *)&(keyStats))[i] =                                        \
                    (Cpa32U)osalAtomicGet(&pService->pLacKeyStats[i]);      \
    }                                                                       \
} while (0)
/**< macro to get all 32bit Key stats (from internal array of atomics) */

#define LAC_KEY_STATS64_GET(keyStats, instanceHandle)                       \
do {                                                                        \
    int i;                                                                  \
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;\
    for (i = 0; i < LAC_KEY_NUM_STATS; i++)                                 \
    {                                                                       \
        ((Cpa64U *)&(keyStats))[i] =                                        \
                            osalAtomicGet(&pService->pLacKeyStats[i]);      \
    }                                                                       \
} while (0)
/**< macro to get all 64bit Key stats (from internal array of atomics) */

/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      SSL/TLS stat type
 *
 * @description
 *      This enum determines which stat should be incremented
 *****************************************************************************/
typedef enum
{
    LAC_KEY_REQUESTS = 0,
    /**< Key requests sent */
    LAC_KEY_REQUEST_ERRORS,
    /**< Key requests errors */
    LAC_KEY_COMPLETED,
    /**< Key requests which received responses */
    LAC_KEY_COMPLETED_ERRORS
    /**< Key requests which received responses with errors */
}lac_key_stat_type_t;

/*** Local functions prototypes ***/
STATIC void
LacSymKey_MgfHandleResponse(
    icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pOpaqueData,
    icp_qat_fw_comn_flags cmnRespFlags);

STATIC CpaStatus
LacSymKey_MgfSync(const CpaInstanceHandle instanceHandle,
                  const CpaCyGenFlatBufCbFunc pKeyGenCb,
                  void *pCallbackTag,
                  const void *pKeyGenMgfOpData,
                  CpaFlatBuffer *pGeneratedMaskBuffer,
                  CpaBoolean bIsExtRequest);

STATIC void
LacSymKey_SslTlsHandleResponse(icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pOpaqueData,
    icp_qat_fw_comn_flags cmnRespFlags);

STATIC CpaStatus
LacSymKey_SslTlsSync(CpaInstanceHandle instanceHandle,
    const CpaCyGenFlatBufCbFunc pKeyGenCb,
    void *pCallbackTag,
    icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pKeyGenSslTlsOpData,
    CpaCySymHashAlgorithm hashAlgorithm,
    CpaFlatBuffer *pKeyGenOutpuData);


/*** Implementation ***/

/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Get the instance handle. Support single handle.
 * @param[in] instanceHandle_in        user supplied handle.
 * @retval    CpaInstanceHandle        the instance handle
 */
CpaInstanceHandle
LacKey_GetHandle(CpaInstanceHandle instanceHandle_in)
{
    CpaInstanceHandle instanceHandle = NULL;
    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }
    LAC_ENSURE_NOT_NULL(instanceHandle);
    return instanceHandle;
}

/**
*******************************************************************************
 * @ingroup LacSymKey
 *      Perform SSL/TLS key gen operation
 *
 * @description
 *      Perform SSL/TLS key gen operation
 *
 * @param[in] instanceHandle        QAT device handle.
 * @param[in] pKeyGenCb             Pointer to callback function to be invoked
 *                                  when the operation is complete.
 * @param[in] pCallbackTag          Opaque User Data for this specific call.
 * @param[in] lacCmdId              Lac command ID (identify SSL & TLS ops)
 * @param[in] pKeyGenSslTlsOpData   Structure containing all the data needed to
 *                                  perform the SSL/TLS key generation
 *                                  operation.
 * @param[in]  hashAlgorithm        Specifies the hash algorithm to use.
 *                                  According to RFC5246, this should be
 *                                  "SHA-256 or a stronger standard hash
 *                                  function."
 * @param[out] pKeyGenOutputData    pointer to where output result should be
 *                                  written
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_RETRY          Function should be retried.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE       Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymKey_KeyGenSslTls_GenCommon(CpaInstanceHandle instanceHandle,
                          const CpaCyGenFlatBufCbFunc pKeyGenCb,
                          void *pCallbackTag,
                          icp_qat_fw_la_cmd_id_t lacCmdId,
                          void *pKeyGenSslTlsOpData,
                          CpaCySymHashAlgorithm hashAlgorithm,
                          CpaFlatBuffer *pKeyGenOutputData);


/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Increment stat for TLS or SSL operation
 *
 * @description
 *      This is a generic function to update the stats for either a TLS or SSL
 *      operation.
 *
 * @param[in] lacCmdId          Indicate SSL or TLS operations
 * @param[in] statType          Statistics Type
 * @param[in] instanceHandle    Instance Handle
 *
 * @return None
 *
 *****************************************************************************/
STATIC void
LacKey_StatsInc(
    icp_qat_fw_la_cmd_id_t lacCmdId,
    lac_key_stat_type_t statType,
    CpaInstanceHandle instanceHandle)
{
    if (ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE == lacCmdId)
    {
        switch (statType)
        {
            case LAC_KEY_REQUESTS:
                LAC_KEY_STAT_INC(numSslKeyGenRequests, instanceHandle);
                break;
            case LAC_KEY_REQUEST_ERRORS:
                LAC_KEY_STAT_INC(numSslKeyGenRequestErrors, instanceHandle);
                break;
            case LAC_KEY_COMPLETED:
                LAC_KEY_STAT_INC(numSslKeyGenCompleted, instanceHandle);
                break;
            case LAC_KEY_COMPLETED_ERRORS:
                LAC_KEY_STAT_INC(numSslKeyGenCompletedErrors, instanceHandle);
                break;
            default:
                LAC_ENSURE(CPA_FALSE, "Invalid statistics type");
                break;
        }
    }
    else    /* TLS v1.0/1.1 and 1.2*/
    {
        switch (statType)
        {
            case LAC_KEY_REQUESTS:
                LAC_KEY_STAT_INC(numTlsKeyGenRequests, instanceHandle);
                break;
            case LAC_KEY_REQUEST_ERRORS:
                LAC_KEY_STAT_INC(numTlsKeyGenRequestErrors, instanceHandle);
                break;
            case LAC_KEY_COMPLETED:
                LAC_KEY_STAT_INC(numTlsKeyGenCompleted, instanceHandle);
                break;
            case LAC_KEY_COMPLETED_ERRORS:
                LAC_KEY_STAT_INC(numTlsKeyGenCompletedErrors, instanceHandle);
                break;
            default:
                LAC_ENSURE(CPA_FALSE, "Invalid statistics type");
                break;
        }
    }
}


void
LacKeygen_StatsShow(CpaInstanceHandle instanceHandle)
{
    CpaCyKeyGenStats64 keyStats = {0};

    LAC_KEY_STATS64_GET(keyStats, instanceHandle);

    osalLog64 (OSAL_LOG_LVL_USER,
               OSAL_LOG_DEV_STDOUT,
               SEPARATOR
               BORDER "                  Key Stats:                " BORDER "\n"
               SEPARATOR,
               0, 0, 0, 0, 0, 0, 0, 0);

    osalLog64 (OSAL_LOG_LVL_USER,
               OSAL_LOG_DEV_STDOUT,
               BORDER " SSL Key Requests:               %16llu " BORDER "\n"
               BORDER " SSL Key Request Errors:         %16llu " BORDER "\n"
               BORDER " SSL Key Completed               %16llu " BORDER "\n"
               BORDER " SSL Key Complete Errors:        %16llu " BORDER "\n"
               SEPARATOR,
               keyStats.numSslKeyGenRequests,
               keyStats.numSslKeyGenRequestErrors,
               keyStats.numSslKeyGenCompleted,
               keyStats.numSslKeyGenCompletedErrors,
               0, 0, 0, 0);

    osalLog64 (OSAL_LOG_LVL_USER,
               OSAL_LOG_DEV_STDOUT,
               BORDER " TLS Key Requests:               %16llu " BORDER "\n"
               BORDER " TLS Key Request Errors:         %16llu " BORDER "\n"
               BORDER " TLS Key Completed               %16llu " BORDER "\n"
               BORDER " TLS Key Complete Errors:        %16llu " BORDER "\n"
               SEPARATOR,
               keyStats.numTlsKeyGenRequests,
               keyStats.numTlsKeyGenRequestErrors,
               keyStats.numTlsKeyGenCompleted,
               keyStats.numTlsKeyGenCompletedErrors,
               0, 0, 0, 0);

    osalLog64 (OSAL_LOG_LVL_USER,
               OSAL_LOG_DEV_STDOUT,
               BORDER " MGF Key Requests:               %16llu " BORDER "\n"
               BORDER " MGF Key Request Errors:         %16llu " BORDER "\n"
               BORDER " MGF Key Completed               %16llu " BORDER "\n"
               BORDER " MGF Key Complete Errors:        %16llu " BORDER "\n"
               SEPARATOR,
               keyStats.numMgfKeyGenRequests,
               keyStats.numMgfKeyGenRequestErrors,
               keyStats.numMgfKeyGenCompleted,
               keyStats.numMgfKeyGenCompletedErrors,
               0, 0, 0, 0);
}

/** @ingroup LacSymKey */
CpaStatus
cpaCyKeyGenQueryStats(CpaInstanceHandle instanceHandle_in,
                        struct _CpaCyKeyGenStats *pSymKeyStats)
{
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pSymKeyStats);
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
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    LAC_CHECK_NULL_PARAM(pSymKeyStats);
#endif

    SAL_RUNNING_CHECK(instanceHandle);

    LAC_KEY_STATS32_GET(*pSymKeyStats, instanceHandle);

    return CPA_STATUS_SUCCESS;
}

/** @ingroup LacSymKey */
CpaStatus
cpaCyKeyGenQueryStats64(CpaInstanceHandle instanceHandle_in,
                        CpaCyKeyGenStats64 *pSymKeyStats)
{
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pSymKeyStats);
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
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    LAC_CHECK_NULL_PARAM(pSymKeyStats);
#endif

    SAL_RUNNING_CHECK(instanceHandle);

    LAC_KEY_STATS64_GET(*pSymKeyStats, instanceHandle);

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Return the size of the digest for a specific hash algorithm.
 * @description
 *      Return the expected digest size based on the sha algorithm submitted.
 *      The only supported value are sha256, sha384 and sha512.
 *
 * @param[in]  hashAlgorithm        either sha256, sha384 or sha512.
 * @return the expected size or 0 for an invalid hash.
 *
 *****************************************************************************/
STATIC Cpa32U
getDigestSizeFromHashAlgo(CpaCySymHashAlgorithm hashAlgorithm)
{
    switch(hashAlgorithm)
    {
        case CPA_CY_SYM_HASH_SHA256:
            return LAC_HASH_SHA256_DIGEST_SIZE;
        case CPA_CY_SYM_HASH_SHA384:
            return LAC_HASH_SHA384_DIGEST_SIZE;
        case CPA_CY_SYM_HASH_SHA512:
            return LAC_HASH_SHA512_DIGEST_SIZE;
        default:
            return 0;
    }
}
/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Key Generation MGF response handler
 *
 * @description
 *      Handles Key Generation MGF response messages from the QAT.
 *
 * @param[in] lacCmdId       Command id of the original request
 * @param[in] pOpaqueData    Pointer to opaque data that was in request
 * @param[in] cmnRespFlags   Indicates whether request succeeded
 *
 * @return void
 *
 *****************************************************************************/
STATIC void
LacSymKey_MgfHandleResponse(
    icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pOpaqueData,
    icp_qat_fw_comn_flags cmnRespFlags)
{
    CpaCyKeyGenMgfOpData *pMgfOpData = NULL;
    lac_sym_key_cookie_t *pCookie = NULL;
    CpaCyGenFlatBufCbFunc pKeyGenMgfCb = NULL;
    void *pCallbackTag = NULL;
    CpaFlatBuffer *pGeneratedKeyBuffer= NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean respStatusOk = (ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
                            ICP_QAT_FW_COMN_RESP_CRYPTO_STAT_GET(cmnRespFlags))
                              ? CPA_TRUE : CPA_FALSE;

    pCookie = (lac_sym_key_cookie_t *)pOpaqueData;

    if (CPA_TRUE == respStatusOk)
    {
        status = CPA_STATUS_SUCCESS;
        LAC_KEY_STAT_INC(numMgfKeyGenCompleted, pCookie->instanceHandle);
    }
    else
    {
        status = CPA_STATUS_FAIL;
        LAC_KEY_STAT_INC(numMgfKeyGenCompletedErrors, pCookie->instanceHandle);
    }

    pKeyGenMgfCb =
        (CpaCyGenFlatBufCbFunc)(pCookie->pKeyGenCb);

    LAC_ASSERT_NOT_NULL(pKeyGenMgfCb);

    pMgfOpData = pCookie->pKeyGenOpData;
    pCallbackTag = pCookie->pCallbackTag;
    pGeneratedKeyBuffer = pCookie->pKeyGenOutputData;

    Lac_MemPoolEntryFree(pCookie);

    (*pKeyGenMgfCb)(pCallbackTag, status, pMgfOpData, pGeneratedKeyBuffer);

}

/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Synchronous mode of operation wrapper function
 *
 * @description
 *      Wrapper function to implement synchronous mode of operation for
 *      cpaCyKeyGenMgf and cpaCyKeyGenMgfExt function.
 *
 * @param[in] instanceHandle       Instance handle
 * @param[in] pKeyGenCb            Internal callback function pointer
 * @param[in] pCallbackTag         Callback tag
 * @param[in] pKeyGenMgfOpData     Pointer to user provided Op Data structure
 * @param[in] pGeneratedMaskBuffer Pointer to a buffer where generated mask
 *                                 will be stored
 * @param[in] bIsExtRequest        Indicates origin of function call;
 *                                 if CPA_TRUE then the call comes from
 *                                 cpaCyKeyGenMgfExt function, otherwise
 *                                 from cpaCyKeyGenMgf
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_RETRY          Function should be retried.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE       Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymKey_MgfSync(const CpaInstanceHandle instanceHandle,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const void *pKeyGenMgfOpData,
               CpaFlatBuffer *pGeneratedMaskBuffer,
               CpaBoolean bIsExtRequest)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    lac_sync_op_data_t *pSyncCallbackData = NULL;

    status = LacSync_CreateSyncCookie(&pSyncCallbackData);

    if (CPA_STATUS_SUCCESS == status)
    {
        if(CPA_TRUE == bIsExtRequest)
        {
            status = cpaCyKeyGenMgfExt(instanceHandle, LacSync_GenFlatBufCb,
                            pSyncCallbackData,
                            (const CpaCyKeyGenMgfOpDataExt *)pKeyGenMgfOpData,
                            pGeneratedMaskBuffer);
        }
        else
        {
            status = cpaCyKeyGenMgf(instanceHandle, LacSync_GenFlatBufCb,
                            pSyncCallbackData,
                            (const CpaCyKeyGenMgfOpData*)pKeyGenMgfOpData,
                            pGeneratedMaskBuffer);
        }
    }
    else
    {
        /* Failure allocating sync cookie */
        LAC_KEY_STAT_INC(numMgfKeyGenRequestErrors, instanceHandle);
        return status;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus syncStatus = CPA_STATUS_SUCCESS;

        syncStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                             LAC_SYM_SYNC_CALLBACK_TIMEOUT,
                                             &status,
                                             NULL);

        /* If callback doesn't come back */
        if (CPA_STATUS_SUCCESS != syncStatus)
        {
            LAC_KEY_STAT_INC(numMgfKeyGenCompletedErrors, instanceHandle);
            LAC_LOG_ERROR("Callback timed out");
            status = syncStatus;
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
 ******************************************************************************
 * @ingroup LacSymKey
 *      Perform MGF key gen operation
 *
 * @description
 *      This function performs MGF key gen operation. It is common for requests
 *      coming from both cpaCyKeyGenMgf and cpaCyKeyGenMgfExt QAT API
 *      functions.
 *
 * @param[in] instanceHandle       Instance handle
 * @param[in] pKeyGenCb            Pointer to callback function to be invoked
 *                                 when the operation is complete.
 * @param[in] pCallbackTag         Opaque User Data for this specific call.
 * @param[in] pOpData              Pointer to the Op Data structure provided by
 *                                 the user in API function call. For calls
 *                                 originating from cpaCyKeyGenMgfExt it will
 *                                 point to CpaCyKeyGenMgfOpDataExt type of
 *                                 structure while for calls originating from
 *                                 cpaCyKeyGenMgf it will point to
 *                                 CpaCyKeyGenMgfOpData type of structure.
 * @param[in] pKeyGenMgfOpData     Pointer to the user provided
 *                                 CpaCyKeyGenMgfOpData structure. For calls
 *                                 originating from cpaCyKeyGenMgf it will
 *                                 point to the same structure as pOpData
 *                                 parameter; for calls originating from
 *                                 cpaCyKeyGenMgfExt it will point to the
 *                                 baseOpData member of the
 *                                 CpaCyKeyGenMgfOpDataExt structure passed in
 *                                 as a parameter to the API function call.
 * @param[in] pGeneratedMaskBuffer Pointer to a buffer where generated mask
 *                                 will be stored
 * @param[in] hashAlgorithm        Indicates which hash algorithm is to be used
 *                                 to perform MGF key gen operation. For calls
 *                                 originating from cpaCyKeyGenMgf it will
 *                                 always be CPA_CY_SYM_HASH_SHA1.
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_RETRY          Function should be retried.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE       Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymKey_MgfCommon(const CpaInstanceHandle instanceHandle,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const void *pOpData,
               const CpaCyKeyGenMgfOpData *pKeyGenMgfOpData,
               CpaFlatBuffer *pGeneratedMaskBuffer,
               CpaCySymHashAlgorithm hashAlgorithm)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    icp_qat_fw_la_key_gen_req_t keyGenReq;
    sal_qat_content_desc_info_t contentDescInfo = {0};
    lac_sym_key_cookie_t *pCookie = NULL;
    lac_sym_cookie_t *pSymCookie = NULL;
    sal_crypto_service_t* pService = NULL;
    Cpa64U inputPhysAddr = 0;
    Cpa64U outputPhysAddr = 0;
    /* Structure initializer is supported by C99, but it
     * is not supported by some old Intel compiler
     */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaCySymHashSetupData hashSetupData = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    Cpa32U hashBlkSizeInBytes = 0;
    lac_sym_qat_hash_alg_info_t *pHashAlgInfo = NULL;

    pService = (sal_crypto_service_t*) instanceHandle;
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
#endif

    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pKeyGenMgfOpData);
    LAC_CHECK_NULL_PARAM(pGeneratedMaskBuffer);
    LAC_CHECK_NULL_PARAM(pGeneratedMaskBuffer->pData);
    LAC_CHECK_NULL_PARAM(pKeyGenMgfOpData->seedBuffer.pData);

    LAC_ASSERT_NOT_NULL(pKeyGenCb);

    /* Maximum seed length for MGF1 request */
    if (pKeyGenMgfOpData->seedBuffer.dataLenInBytes >
                   ICP_QAT_FW_LA_MGF_SEED_LEN_MAX)
    {
        LAC_INVALID_PARAM_LOG("seedBuffer.dataLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Maximum mask length for MGF1 request */
    if (pKeyGenMgfOpData->maskLenInBytes > ICP_QAT_FW_LA_MGF_MASK_LEN_MAX)
    {
        LAC_INVALID_PARAM_LOG("maskLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* check for enough space in the flat buffer */
    if (pKeyGenMgfOpData->maskLenInBytes >
        pGeneratedMaskBuffer->dataLenInBytes)
    {
        LAC_INVALID_PARAM_LOG("pGeneratedMaskBuffer.dataLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /* Get hash alg info */
    LacSymQat_HashAlgLookupGet(instanceHandle, hashAlgorithm, &pHashAlgInfo);

    /* Allocate the cookie */
    pCookie =
    (lac_sym_key_cookie_t*)Lac_MemPoolEntryAlloc(pService->lac_sym_cookie_pool);
    if ((NULL == pCookie) || ((void*)CPA_STATUS_RETRY == pCookie))
    {
        LAC_LOG_ERROR("Cannot get mem pool entry");
        LAC_KEY_STAT_INC(numMgfKeyGenRequestErrors, instanceHandle);
        return CPA_STATUS_RESOURCE;
    }
    pSymCookie = (lac_sym_cookie_t *)pCookie;
    /* populate the cookie */
    pCookie->instanceHandle= instanceHandle;
    pCookie->pCallbackTag = pCallbackTag;
    pCookie->pKeyGenOpData =
            (void *) LAC_CONST_PTR_CAST(pOpData);
    pCookie->pKeyGenCb = pKeyGenCb;
    pCookie->pKeyGenOutputData = pGeneratedMaskBuffer;
    hashSetupData.hashAlgorithm = hashAlgorithm;
    hashSetupData.hashMode = CPA_CY_SYM_HASH_MODE_PLAIN;
    hashSetupData.digestResultLenInBytes = pHashAlgInfo->digestLength;

    /* Populate the Control block of the content descriptor */
    LacSymQat_HashContentDescInit(instanceHandle,
                &hashSetupData,
                /* control block first in content descriptor */
                (icp_qat_fw_auth_hdr_t *)pCookie->contentDesc,
                /* point to base of hw setup block */
                (Cpa8U *)pCookie->contentDesc + sizeof(icp_qat_fw_auth_hdr_t),
                LAC_SYM_KEY_NO_HASH_BLK_OFFSET_BYTES,
                ICP_QAT_FW_SLICE_NULL,
                ICP_QAT_HW_AUTH_MODE0,  /* just a plain hash */
                NULL,
                &hashBlkSizeInBytes);

    LacSymQat_KeyMgfRequestPopulate(
            &keyGenReq,
            pKeyGenMgfOpData->seedBuffer.dataLenInBytes,
            pKeyGenMgfOpData->maskLenInBytes,
            (Cpa8U)pHashAlgInfo->digestLength);

    contentDescInfo.pData = pCookie->contentDesc;
    contentDescInfo.dataPhys = LAC_MEM_CAST_PTR_TO_UINT64(
                pSymCookie->keyContentDescPhyAddr);
    contentDescInfo.hdrSzQuadWords =
            LAC_BYTES_TO_QUADWORDS(sizeof(icp_qat_fw_auth_hdr_t));
    contentDescInfo.hwBlkSzQuadWords =
            LAC_BYTES_TO_QUADWORDS(hashBlkSizeInBytes);

    /* Populate common request fields */
    inputPhysAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                LAC_OS_VIRT_TO_PHYS_EXTERNAL(pService->generic_service_info,
                                      pKeyGenMgfOpData->seedBuffer.pData));

    if (inputPhysAddr == 0)
    {
        LAC_LOG_ERROR("Unable to get the seed buffer physical address\n");
        status = CPA_STATUS_FAIL;
    }
    outputPhysAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                LAC_OS_VIRT_TO_PHYS_EXTERNAL(pService->generic_service_info,
                                             pGeneratedMaskBuffer->pData));
    if (outputPhysAddr == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the mask\n");
        status = CPA_STATUS_FAIL;
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        SalQatMsg_CmnMsgHdrPopulate((icp_qat_fw_la_bulk_req_t *)(&(keyGenReq)),
                                    &contentDescInfo,
                                    pService->keyCmnReqFlags,
                                    0);

        SalQatMsg_CmnMsgMidPopulate(&(keyGenReq.comn_mid),
                                    pCookie,
                                    inputPhysAddr,
                                    outputPhysAddr);

        SalQatMsg_ReqTypePopulate(&(keyGenReq.comn_hdr.arch_if),
                                    ICP_ARCH_IF_REQ_QAT_FW_LA,
                                   pService->symLoResponseRingId);

        /* Set footer - it is called to zero fields that are not used by key
             * requests */
        SalQatMsg_CmnFooterPopulate(&(keyGenReq.comn_ftr), 0);


         status = icp_adf_transPutMsg(pService->trans_handle_sym_tx_lo,
                                 (void*)&(keyGenReq),
                                 LAC_QAT_MSG_SZ_LW);
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        /* Update stats */
        LAC_KEY_STAT_INC(numMgfKeyGenRequests, instanceHandle);
    }
    else
    {
        LAC_KEY_STAT_INC(numMgfKeyGenRequestErrors, instanceHandle);
        /* clean up memory */
        if (NULL != pCookie)
        {
            Lac_MemPoolEntryFree(pCookie);
        }
    }
    return status;
}

/**
* cpaCyKeyGenMgf
*/
CpaStatus
cpaCyKeyGenMgf(const CpaInstanceHandle instanceHandle_in,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const CpaCyKeyGenMgfOpData *pKeyGenMgfOpData,
               CpaFlatBuffer *pGeneratedMaskBuffer)
{
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pKeyGenCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pKeyGenMgfOpData,
            (LAC_ARCH_UINT)pGeneratedMaskBuffer);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    /* If synchronous Operation */
    if (NULL == pKeyGenCb)
    {
        return LacSymKey_MgfSync(instanceHandle, pKeyGenCb, pCallbackTag,
                                 (const void*)pKeyGenMgfOpData,
                                 pGeneratedMaskBuffer, CPA_FALSE);
    }
    /* Asynchronous Operation */
    return LacSymKey_MgfCommon(instanceHandle, pKeyGenCb, pCallbackTag,
                             (const void*)pKeyGenMgfOpData,
                             pKeyGenMgfOpData,
                             pGeneratedMaskBuffer,
                             CPA_CY_SYM_HASH_SHA1);
}

/**
* cpaCyKeyGenMgfExt
*/
CpaStatus
cpaCyKeyGenMgfExt(const CpaInstanceHandle instanceHandle_in,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const CpaCyKeyGenMgfOpDataExt *pKeyGenMgfOpDataExt,
               CpaFlatBuffer *pGeneratedMaskBuffer)
{
    CpaInstanceHandle instanceHandle = NULL;

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pKeyGenCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pKeyGenMgfOpDataExt,
            (LAC_ARCH_UINT)pGeneratedMaskBuffer);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    /* If synchronous Operation */
    if (NULL == pKeyGenCb)
    {
        return LacSymKey_MgfSync(instanceHandle, pKeyGenCb, pCallbackTag,
                                 (const void*)pKeyGenMgfOpDataExt,
                                 pGeneratedMaskBuffer, CPA_TRUE);
    }

#ifdef ICP_PARAM_CHECK
    /* Param check specific for Ext function, rest of parameters validated
     * in LacSymKey_MgfCommon */
    LAC_CHECK_NULL_PARAM(pKeyGenMgfOpDataExt);
    if(CPA_CY_SYM_HASH_MD5 > pKeyGenMgfOpDataExt->hashAlgorithm
       || CPA_CY_SYM_HASH_SHA512 < pKeyGenMgfOpDataExt->hashAlgorithm)
    {
        LAC_INVALID_PARAM_LOG("hashAlgorithm");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /* Asynchronous Operation */
    return LacSymKey_MgfCommon(instanceHandle, pKeyGenCb, pCallbackTag,
                             (const void*)pKeyGenMgfOpDataExt,
                             &pKeyGenMgfOpDataExt->baseOpData,
                             pGeneratedMaskBuffer,
                             pKeyGenMgfOpDataExt->hashAlgorithm);
}

/**
 ******************************************************************************
 * @ingroup LacSymKey
 *      Key Generation SSL & TLS response handler
 *
 * @description
 *      Handles Key Generation SSL & TLS response messages from the QAT.
 *
 * @param[in] lacCmdId        Command id of the original request
 * @param[in] pOpaqueData     Pointer to opaque data that was in request
 * @param[in] cmnRespFlags    LA response flags
 *
 * @return void
 *
 *****************************************************************************/
STATIC void
LacSymKey_SslTlsHandleResponse(icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pOpaqueData,
    icp_qat_fw_comn_flags cmnRespFlags)
{
    void *pSslTlsOpData = NULL;
    CpaCyGenFlatBufCbFunc pKeyGenSslTlsCb = NULL;
    lac_sym_key_cookie_t *pCookie = NULL;
    void *pCallbackTag = NULL;
    CpaFlatBuffer *pGeneratedKeyBuffer= NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

    CpaBoolean respStatusOk = (ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
        ICP_QAT_FW_COMN_RESP_CRYPTO_STAT_GET(cmnRespFlags)) ?
                          CPA_TRUE : CPA_FALSE;

    pCookie = (lac_sym_key_cookie_t *)pOpaqueData;

    pSslTlsOpData = pCookie->pKeyGenOpData;

    if (CPA_TRUE == respStatusOk)
    {
        LacKey_StatsInc(lacCmdId,
                        LAC_KEY_COMPLETED,
                        pCookie->instanceHandle);
    }
    else
    {
        status = CPA_STATUS_FAIL;
        LacKey_StatsInc(lacCmdId,
                        LAC_KEY_COMPLETED_ERRORS,
                        pCookie->instanceHandle);
    }

    pKeyGenSslTlsCb = (CpaCyGenFlatBufCbFunc)(pCookie->pKeyGenCb);
    LAC_ASSERT_NOT_NULL(pKeyGenSslTlsCb);

    pCallbackTag = pCookie->pCallbackTag;
    pGeneratedKeyBuffer = pCookie->pKeyGenOutputData;

    Lac_MemPoolEntryFree(pCookie);

    (*pKeyGenSslTlsCb)(pCallbackTag, status,
                        pSslTlsOpData, pGeneratedKeyBuffer);
}


/**
*******************************************************************************
 * @ingroup LacSymKey
 *      Synchronous mode of operation function wrapper for performing SSL/TLS
 *      key gen operation
 *
 * @description
 *      Synchronous mode of operation function wrapper for performing SSL/TLS
 *      key gen operation
 *
 * @param[in] instanceHandle        QAT device handle.
 * @param[in] pKeyGenCb             Pointer to callback function to be invoked
 *                                  when the operation is complete.
 * @param[in] pCallbackTag          Opaque User Data for this specific call.
 * @param[in] lacCmdId              Lac command ID (identify SSL & TLS ops)
 * @param[in] pKeyGenSslTlsOpData   Structure containing all the data needed to
 *                                  perform the SSL/TLS key generation
 *                                  operation.
 * @param[in]  hashAlgorithm        Specifies the hash algorithm to use.
 *                                  According to RFC5246, this should be
 *                                  "SHA-256 or a stronger standard hash
 *                                  function."
 * @param[out] pKeyGenOutputData    pointer to where output result should be
 *                                  written
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_RETRY          Function should be retried.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE       Error related to system resources.
 *
 *****************************************************************************/
STATIC CpaStatus
LacSymKey_SslTlsSync(CpaInstanceHandle instanceHandle,
                    const CpaCyGenFlatBufCbFunc pKeyGenCb,
                    void *pCallbackTag,
                    icp_qat_fw_la_cmd_id_t lacCmdId,
                    void *pKeyGenSslTlsOpData,
                    CpaCySymHashAlgorithm hashAlgorithm,
                    CpaFlatBuffer *pKeyGenOutpuData)
{
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LacSymKey_KeyGenSslTls_GenCommon(instanceHandle,
                                           pKeyGenCb,
                                           pSyncCallbackData,
                                           lacCmdId,
                                           pKeyGenSslTlsOpData,
                                           hashAlgorithm,
                                           pKeyGenOutpuData);
    }
    else
    {
        /* Failure allocating sync cookie */
        LacKey_StatsInc(lacCmdId,
                        LAC_KEY_REQUEST_ERRORS,
                        instanceHandle);
        return status;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus syncStatus = CPA_STATUS_SUCCESS;

        syncStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                             LAC_SYM_SYNC_CALLBACK_TIMEOUT,
                                             &status,
                                             NULL);


        /* If callback doesn't come back */
        if (CPA_STATUS_SUCCESS != syncStatus)
        {
            LacKey_StatsInc(lacCmdId,
                            LAC_KEY_COMPLETED_ERRORS,
                            instanceHandle);
            LAC_LOG_ERROR("Callback timed out");
            status = syncStatus;
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

STATIC CpaStatus
computeHashKey(CpaFlatBuffer *secret, CpaFlatBuffer *hash, CpaCySymHashAlgorithm *hashAlgorithm)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    switch (*hashAlgorithm)
    {
    case CPA_CY_SYM_HASH_MD5:
        status = osalHashMD5Full(secret->pData,
                                 hash->pData,
                                 secret->dataLenInBytes);
        break;
    case CPA_CY_SYM_HASH_SHA1:
        status = osalHashSHA1Full(secret->pData,
                                  hash->pData,
                                  secret->dataLenInBytes);
        break;
    case CPA_CY_SYM_HASH_SHA256:
        status = osalHashSHA256Full(secret->pData,
                                    hash->pData,
                                    secret->dataLenInBytes);
        break;
    case CPA_CY_SYM_HASH_SHA384:
        status = osalHashSHA384Full(secret->pData,
                                    hash->pData,
                                    secret->dataLenInBytes);
        break;
    case CPA_CY_SYM_HASH_SHA512:
        status = osalHashSHA512Full(secret->pData,
                                    hash->pData,
                                    secret->dataLenInBytes);
        break;
    default:
        status = CPA_STATUS_FAIL;
    }
    return status;
}

STATIC CpaStatus
LacSymKey_KeyGenSslTls_GenCommon(CpaInstanceHandle instanceHandle,
                          const CpaCyGenFlatBufCbFunc pKeyGenCb,
                          void *pCallbackTag,
                          icp_qat_fw_la_cmd_id_t lacCmdId,
                          void *pKeyGenSslTlsOpData,
                          CpaCySymHashAlgorithm hashAlgorithm,
                          CpaFlatBuffer *pKeyGenOutputData)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean precompute = CPA_FALSE;
    icp_qat_fw_la_key_gen_req_t keyGenReq;
    lac_sym_key_cookie_t *pCookie = NULL;
    lac_sym_cookie_t *pSymCookie = NULL;
    Cpa64U inputPhysAddr = 0;
    Cpa64U outputPhysAddr = 0;
    /* Structure initializer is supported by C99, but it
     * is not supported by some old Intel compiler
     */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaCySymHashSetupData hashSetupData = {0};
    sal_qat_content_desc_info_t contentDescInfo = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    Cpa32U hashBlkSizeInBytes = 0;
    Cpa32U tlsPrefixLen = 0;
    icp_qat_hw_auth_mode_t qatHashMode = 0;
    CpaCySymHashNestedModeSetupData *pNestedModeSetupData =
            &(hashSetupData.nestedModeSetupData);
    CpaFlatBuffer inputSecret = {0};
    CpaFlatBuffer hashKeyOutput = {0};
    Cpa32U uSecretLen = 0;

	sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;

    /* If synchronous Operation */
    if (NULL == pKeyGenCb)
    {
       return LacSymKey_SslTlsSync(instanceHandle, LacSync_GenFlatBufCb,
                   pCallbackTag, lacCmdId, pKeyGenSslTlsOpData, hashAlgorithm,
                    pKeyGenOutputData);
    }

    /* Allocate the cookie */
    pCookie =
    (lac_sym_key_cookie_t*)Lac_MemPoolEntryAlloc(pService->lac_sym_cookie_pool);
    if ((NULL == pCookie) || ((void*)CPA_STATUS_RETRY == pCookie))
    {
        LAC_LOG_ERROR("Cannot get mem pool entry");
        /* Update stats */
        LacKey_StatsInc(lacCmdId,
                        LAC_KEY_REQUEST_ERRORS,
                        instanceHandle);
        return CPA_STATUS_RESOURCE;
    }
    pSymCookie = (lac_sym_cookie_t *)pCookie;
    if (ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE == lacCmdId)
    {
        qatHashMode = ICP_QAT_HW_AUTH_MODE0;
    }
    else /* TLS v1.0/1.1 or v1.2 */
    {
        qatHashMode = ICP_QAT_HW_AUTH_MODE2;
    }

    pCookie->instanceHandle = pService;
    pCookie->pCallbackTag = pCallbackTag;
    pCookie->pKeyGenCb = pKeyGenCb;
    pCookie->pKeyGenOpData = pKeyGenSslTlsOpData;
    pCookie->pKeyGenOutputData = pKeyGenOutputData;
    hashSetupData.hashMode = CPA_CY_SYM_HASH_MODE_NESTED;

    if (ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE == lacCmdId)
    {
        hashSetupData.hashAlgorithm = CPA_CY_SYM_HASH_SHA1;
        hashSetupData.digestResultLenInBytes = LAC_HASH_MD5_DIGEST_SIZE;
        pNestedModeSetupData->outerHashAlgorithm = CPA_CY_SYM_HASH_MD5;
        pNestedModeSetupData->pInnerPrefixData = NULL;
        pNestedModeSetupData->innerPrefixLenInBytes = 0;
        pNestedModeSetupData->pOuterPrefixData = NULL;
        pNestedModeSetupData->outerPrefixLenInBytes = 0;
    }
    else if (ICP_QAT_FW_LA_CMD_TLS_V1_2_KEY_DERIVE == lacCmdId)
    { /* TLS v1.2 */
        CpaCyKeyGenTlsOpData *pKeyGenTlsOpData =
                (CpaCyKeyGenTlsOpData *)pKeyGenSslTlsOpData;

        uSecretLen = pKeyGenTlsOpData->secret.dataLenInBytes;

        hashSetupData.hashAlgorithm = (CpaCySymHashAlgorithm) hashAlgorithm;
        hashSetupData.digestResultLenInBytes =
                            (Cpa32U)getDigestSizeFromHashAlgo(hashAlgorithm);

        pNestedModeSetupData->outerHashAlgorithm =
                            (CpaCySymHashAlgorithm) hashAlgorithm;
        if (CPA_CY_KEY_TLS_OP_MASTER_SECRET_DERIVE ==
                    pKeyGenTlsOpData->tlsOp)
        {
            switch (hashAlgorithm)
            {
                case CPA_CY_SYM_HASH_SHA256:
                    if (uSecretLen > ICP_QAT_FW_LA_TLS_V1_2_SECRET_LEN_MAX)
                    {
                        precompute = CPA_TRUE;
                    }
                    break;
                case CPA_CY_SYM_HASH_SHA384:
                case CPA_CY_SYM_HASH_SHA512:
                    if (uSecretLen > ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX)
                    {
                        precompute = CPA_TRUE;
                    }
                    break;
                default:
                    break;
            }
        }
        if (CPA_TRUE == precompute)  
        {
            /* Case when secret > algorithm block size */
            /* RFC 4868: For SHA-256 Block size is 512 bits, for SHA-384 and
             * SHA-512 Block size is 1024 bits */

            /* Initialize pointer where SHAxxx key will go. */
            hashKeyOutput.pData = &pCookie->hashKeyBuffer[0];
            hashKeyOutput.dataLenInBytes =
                    hashSetupData.digestResultLenInBytes;

            computeHashKey(&pKeyGenTlsOpData->secret,
                           &hashKeyOutput,
                           &hashSetupData.hashAlgorithm);

            /* outer prefix = secret , inner prefix = secret
             * secret < 64 bytes */
            pNestedModeSetupData->pInnerPrefixData =
                    hashKeyOutput.pData;
            pNestedModeSetupData->pOuterPrefixData =
                    hashKeyOutput.pData;
            pNestedModeSetupData->innerPrefixLenInBytes =
                    hashKeyOutput.dataLenInBytes;
            pNestedModeSetupData->outerPrefixLenInBytes =
                    hashKeyOutput.dataLenInBytes;
        }
        else
        {
            /* outer prefix = secret , inner prefix = secret
             * secret <= 64 bytes */
            pNestedModeSetupData->pInnerPrefixData =
                    pKeyGenTlsOpData->secret.pData;
            pNestedModeSetupData->pOuterPrefixData =
                    pKeyGenTlsOpData->secret.pData;
            pNestedModeSetupData->innerPrefixLenInBytes =
                    pKeyGenTlsOpData->secret.dataLenInBytes;
            pNestedModeSetupData->outerPrefixLenInBytes =
                    pKeyGenTlsOpData->secret.dataLenInBytes;
        }
    }
    else /* TLS v1.0/v1.1 */
    {
        CpaCyKeyGenTlsOpData *pKeyGenTlsOpData =
                (CpaCyKeyGenTlsOpData *)pKeyGenSslTlsOpData;

        hashSetupData.hashAlgorithm = CPA_CY_SYM_HASH_SHA1;
        hashSetupData.digestResultLenInBytes = LAC_HASH_MD5_DIGEST_SIZE;
        pNestedModeSetupData->outerHashAlgorithm = CPA_CY_SYM_HASH_MD5;

        uSecretLen = pKeyGenTlsOpData->secret.dataLenInBytes;
        /* We want to handle pre_master_secret > 128 bytes therefore we
         * only verify if the current operation is Master Secret Derive.
         * The other operations remain unchanged. */
        if ((uSecretLen > ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX) &&
            (CPA_CY_KEY_TLS_OP_MASTER_SECRET_DERIVE == pKeyGenTlsOpData->tlsOp))
        {
            /* secret = [s1 | s2 ]
             * s1 = outer prefix, s2 = inner prefix
             * length of s1 and s2 = ceil(secret_length / 2)
             * (secret length + 1)/2 will always give the ceil as division by 2
             * (>>1) will give the smallest integral value not less than arg */
            tlsPrefixLen =
                    (pKeyGenTlsOpData->secret.dataLenInBytes + 1) >> 1;
            inputSecret.dataLenInBytes = tlsPrefixLen;
            inputSecret.pData = pKeyGenTlsOpData->secret.pData;

            /* Since the pre_master_secret is > 128, we split the input
             * pre_master_secret in 2 halves and compute the MD5 of the
             * first half and the SHA1 on the second half. */
            hashAlgorithm = CPA_CY_SYM_HASH_MD5;

            /* Initialize pointer where MD5 key will go. */
            hashKeyOutput.pData = &pCookie->hashKeyBuffer[0];
            hashKeyOutput.dataLenInBytes = LAC_HASH_MD5_DIGEST_SIZE;
            computeHashKey(&inputSecret, &hashKeyOutput, &hashAlgorithm);

            pNestedModeSetupData->pOuterPrefixData =
                    &pCookie->hashKeyBuffer[0];
            pNestedModeSetupData->outerPrefixLenInBytes =
                    LAC_HASH_MD5_DIGEST_SIZE;

            /* Point to the second half of the pre_master_secret */
            inputSecret.pData = pKeyGenTlsOpData->secret.pData +
                    (pKeyGenTlsOpData->secret.dataLenInBytes - tlsPrefixLen);

            /* Compute the SHA1 on the second half of the pre_master_secret */
            hashAlgorithm = CPA_CY_SYM_HASH_SHA1;
            /* Initialize pointer where SHA1 key will go. */
            hashKeyOutput.pData =
                    &pCookie->hashKeyBuffer[LAC_HASH_MD5_DIGEST_SIZE];
            hashKeyOutput.dataLenInBytes = LAC_HASH_SHA1_DIGEST_SIZE;
            computeHashKey(&inputSecret, &hashKeyOutput, &hashAlgorithm);

            pNestedModeSetupData->pInnerPrefixData =
                    &pCookie->hashKeyBuffer[LAC_HASH_MD5_DIGEST_SIZE];
            pNestedModeSetupData->innerPrefixLenInBytes =
                    LAC_HASH_SHA1_DIGEST_SIZE;
        }
        else
        {
            /* secret = [s1 | s2 ]
             * s1 = outer prefix, s2 = inner prefix
             * length of s1 and s2 = ceil(secret_length / 2)
             * (secret length + 1)/2 will always give the ceil as division by 2
             * (>>1) will give the smallest integral value not less than arg */
            tlsPrefixLen =
                    (pKeyGenTlsOpData->secret.dataLenInBytes + 1) >> 1;

            /* last byte of s1 will be first byte of s2 if Length is odd */
            pNestedModeSetupData->pInnerPrefixData =
                    pKeyGenTlsOpData->secret.pData +
                    (pKeyGenTlsOpData->secret.dataLenInBytes - tlsPrefixLen);

            pNestedModeSetupData->pOuterPrefixData =
                    pKeyGenTlsOpData->secret.pData;
            pNestedModeSetupData->innerPrefixLenInBytes = tlsPrefixLen;
            pNestedModeSetupData->outerPrefixLenInBytes = tlsPrefixLen;
        }
    }
    /* note that following function doesn't look at inner/outer
     * prefix pointers in nested digest ctx */
    LacSymQat_HashContentDescInit(instanceHandle,
                &hashSetupData,
                /* control block is first in content descriptor */
                (icp_qat_fw_auth_hdr_t *)pCookie->contentDesc,
                /* pointer to base of hw setup block */
                pCookie->contentDesc + sizeof(icp_qat_fw_auth_hdr_t),
                LAC_SYM_KEY_NO_HASH_BLK_OFFSET_BYTES,
                ICP_QAT_FW_SLICE_NULL,
                qatHashMode,
                NULL, /* precompute data */
                &hashBlkSizeInBytes);

    if (ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE == lacCmdId)
    {
        CpaCyKeyGenSslOpData *pKeyGenSslOpData =
                (CpaCyKeyGenSslOpData *)pKeyGenSslTlsOpData;
        Cpa8U *pLabel = NULL;
        Cpa32U labelLen = 0;
        Cpa8U iterations = 0;
        Cpa64U labelPhysAddr = 0;

        /* iterations = ceiling of output required / output per iteration
         * Ceiling of a / b = (a + (b-1)) / b */
        iterations = (pKeyGenSslOpData->generatedKeyLenInBytes +
                            (LAC_SYM_QAT_KEY_SSL_BYTES_PER_ITERATION - 1))
                            >> LAC_SYM_QAT_KEY_SSL_ITERATIONS_SHIFT;

        if (CPA_CY_KEY_SSL_OP_USER_DEFINED == pKeyGenSslOpData->sslOp)
        {
            pLabel = pKeyGenSslOpData->userLabel.pData;
            labelLen = pKeyGenSslOpData->userLabel.dataLenInBytes;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                    pService->generic_service_info, pLabel);

            if (labelPhysAddr == 0)
            {
                LAC_LOG_ERROR("Unable to get the physical address of the"
                        " label\n");
                status = CPA_STATUS_FAIL;
            }
        }
        else
        {
            pLabel = pService->pSslLabel;
            /* calculate label length.
             * eg. 3 iterations is ABBCCC so length is 6 */
            labelLen = ((iterations * iterations) + iterations) >> 1;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_INTERNAL(pLabel);
        }

        LacSymQat_KeySslRequestPopulate(
                &keyGenReq,
                pKeyGenSslOpData->generatedKeyLenInBytes,
                labelLen,
                pKeyGenSslOpData->secret.dataLenInBytes,
                iterations);

        LacSymQat_KeySslKeyMaterialInputPopulate(
                &(pService->generic_service_info),
                &(pCookie->u.sslKeyInput),
                pKeyGenSslOpData->seed.pData,
                labelPhysAddr,
                pKeyGenSslOpData->secret.pData);

        inputPhysAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                pSymCookie->keySslKeyInputPhyAddr);

    }
    else /* TLS v1.0/1.1 or v1.2 */
    {
        CpaCyKeyGenTlsOpData *pKeyGenTlsOpData =
                (CpaCyKeyGenTlsOpData *)pKeyGenSslTlsOpData;
        lac_sym_qat_hash_state_buffer_info_t hashStateBufferInfo = {0};
        icp_qat_fw_auth_hdr_t *pHashControlBlock =
                (icp_qat_fw_auth_hdr_t *)pCookie->contentDesc;
        Cpa8U *pLabel = NULL;
        Cpa32U labelLen = 0;
        Cpa64U labelPhysAddr = 0;
        hashStateBufferInfo.pData = pCookie->hashStateBuffer;
        hashStateBufferInfo.pDataPhys = LAC_MEM_CAST_PTR_TO_UINT64(
                            pSymCookie->keyHashStateBufferPhyAddr);
        hashStateBufferInfo.stateStorageSzQuadWords = 0;
        /* Prefix sizes are block sizes of algorithms so are already
         * quad word multiples */
        hashStateBufferInfo.prefixAadSzQuadWords = LAC_BYTES_TO_QUADWORDS(
                    pHashControlBlock->u.inner_prefix_sz +
                    pHashControlBlock->outer_prefix_sz);
        /* Copy prefix data into hash state buffer */
        LacSymQat_HashStatePrefixAadBufferPopulate(
                &hashStateBufferInfo,
                (icp_qat_fw_auth_hdr_t *)pCookie->contentDesc,
                hashSetupData.nestedModeSetupData.pInnerPrefixData,
                pNestedModeSetupData->innerPrefixLenInBytes,
                hashSetupData.nestedModeSetupData.pOuterPrefixData,
                pNestedModeSetupData->outerPrefixLenInBytes);
        /* Firmware only looks at hash state buffer pointer and the
         * hash state buffer size so all other fields are set to 0 */
        LacSymQat_HashRequestParamsPopulate(
                &(pService->generic_service_info),
                &(pCookie->hashRequestParams),
                ICP_QAT_FW_SLICE_NULL,
                &hashStateBufferInfo, /* hash state prefix buffer */
                ICP_QAT_FW_LA_PARTIAL_NONE,
                0, /* hash result size */
                0, /* auth offset */
                0, /* auth len*/
                CPA_FALSE,
                NULL,
                CPA_CY_SYM_OP_NONE /* hash Algorithm */);
        /* Set up the labels and their length */
        if (CPA_CY_KEY_TLS_OP_USER_DEFINED == pKeyGenTlsOpData->tlsOp)
        {
            pLabel = pKeyGenTlsOpData->userLabel.pData;
            labelLen = pKeyGenTlsOpData->userLabel.dataLenInBytes;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                    pService->generic_service_info, pLabel);

            if (labelPhysAddr == 0)
            {
                LAC_LOG_ERROR("Unable to get the physical address of the"
                        " label\n");
                status = CPA_STATUS_FAIL;
            }
        }
        else if (CPA_CY_KEY_TLS_OP_MASTER_SECRET_DERIVE ==
                     pKeyGenTlsOpData->tlsOp)
        {
            pLabel = pService->pTlsLabel->masterSecret;
            labelLen = sizeof(LAC_SYM_KEY_TLS_MASTER_SECRET_LABEL) - 1;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_INTERNAL(pLabel);
        }
        else if (CPA_CY_KEY_TLS_OP_KEY_MATERIAL_DERIVE ==
                     pKeyGenTlsOpData->tlsOp)
        {
            pLabel = pService->pTlsLabel->keyMaterial;
            labelLen = sizeof(LAC_SYM_KEY_TLS_KEY_MATERIAL_LABEL) - 1;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_INTERNAL(pLabel);
        }
        else if (CPA_CY_KEY_TLS_OP_CLIENT_FINISHED_DERIVE ==
                     pKeyGenTlsOpData->tlsOp)
        {
            pLabel = pService->pTlsLabel->clientFinished;
            labelLen = sizeof(LAC_SYM_KEY_TLS_CLIENT_FIN_LABEL) - 1;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_INTERNAL(pLabel);
        }
        else
        {
            pLabel = pService->pTlsLabel->serverFinished;
            labelLen = sizeof(LAC_SYM_KEY_TLS_SERVER_FIN_LABEL) - 1;
            labelPhysAddr = LAC_OS_VIRT_TO_PHYS_INTERNAL(pLabel);
        }

        LacSymQat_KeyTlsRequestPopulate(
                &keyGenReq,
                &(pCookie->hashRequestParams),
                pKeyGenTlsOpData->generatedKeyLenInBytes,
                labelLen,
                pKeyGenTlsOpData->secret.dataLenInBytes,
                pKeyGenTlsOpData->seed.dataLenInBytes,
                lacCmdId);

        LacSymQat_KeyTlsKeyMaterialInputPopulate(
                &(pService->generic_service_info),
                &(pCookie->u.tlsKeyInput),
                pKeyGenTlsOpData->seed.pData,
                labelPhysAddr);
        inputPhysAddr = LAC_MEM_CAST_PTR_TO_UINT64(
                pSymCookie->keyTlsKeyInputPhyAddr);
    }
    outputPhysAddr = LAC_MEM_CAST_PTR_TO_UINT64(
          LAC_OS_VIRT_TO_PHYS_EXTERNAL(pService->generic_service_info,
                                        pKeyGenOutputData->pData));
    if (outputPhysAddr == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the"
                        " output buffer\n");
        status = CPA_STATUS_FAIL;
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        contentDescInfo.pData = pCookie->contentDesc;
        contentDescInfo.dataPhys = LAC_MEM_CAST_PTR_TO_UINT64(
            pSymCookie->keyContentDescPhyAddr);
        contentDescInfo.hdrSzQuadWords =
                    LAC_BYTES_TO_QUADWORDS(sizeof(icp_qat_fw_auth_hdr_t));
        contentDescInfo.hwBlkSzQuadWords =
                    LAC_BYTES_TO_QUADWORDS(hashBlkSizeInBytes);

        /* Populate common request header fields */

        SalQatMsg_CmnMsgHdrPopulate((icp_qat_fw_la_bulk_req_t *)(&(keyGenReq)),
                                    &contentDescInfo,
                                    pService->keyCmnReqFlags,
                                    0);

        /* Populate common request middle fields */
        SalQatMsg_CmnMsgMidPopulate(&(keyGenReq.comn_mid),
                                    pCookie,
                                    inputPhysAddr,
                                    outputPhysAddr);


        SalQatMsg_ReqTypePopulate(&(keyGenReq.comn_hdr.arch_if),
                                  ICP_ARCH_IF_REQ_QAT_FW_LA,
                                  pService->symLoResponseRingId);

       /* Set footer - it is called to zero fields that are not used by key
            * requests */
        SalQatMsg_CmnFooterPopulate(&(keyGenReq.comn_ftr), 0);

      /* Send message */
        status = icp_adf_transPutMsg(pService->trans_handle_sym_tx_lo,
                                 (void*)&(keyGenReq),
                                 LAC_QAT_MSG_SZ_LW);
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        /* Update stats */
        LacKey_StatsInc(lacCmdId,
                        LAC_KEY_REQUESTS,
                        pCookie->instanceHandle);
    }
    else
    {
        /* clean up cookie memory */
        if (NULL != pCookie)
        {
            LacKey_StatsInc(lacCmdId,
                            LAC_KEY_REQUEST_ERRORS,
                            pCookie->instanceHandle);
            Lac_MemPoolEntryFree(pCookie);
        }
    }
    return status;
}

/**
 * @ingroup LacSymKey
 *      Parameters check for TLS v1.0/1.1, v1.2 and SSL3
 * @description
 *      Check user parameters against the firmware/spec requirements.
 *
 * @param[in] pKeyGenOpData              Pointer to a structure containing all
 *                                       the data needed to perform the key
 *                                       generation operation.
 * @param[in]  hashAlgorithm             Specifies the hash algorithm to use.
 *                                       According to RFC5246, this should be
 *                                       "SHA-256 or a stronger standard hash
 *                                       function."
 * @param[in] pGeneratedKeyBuffer        User output buffers.
 * @param[in] cmdId                      Keygen operation to perform.
 */
#ifdef ICP_PARAM_CHECK
STATIC CpaStatus
LacSymKey_CheckParamSslTls(const void *pKeyGenOpData,
                CpaCySymHashAlgorithm hashAlgorithm,
                const CpaFlatBuffer *pGeneratedKeyBuffer,
                icp_qat_fw_la_cmd_id_t cmdId)
{
    /* Api max value */
    Cpa32U maxSecretLen = 0;
    Cpa32U maxSeedLen = 0;
    Cpa32U maxOutputLen = 0;

    /* User info */
    Cpa32U uSecretLen = 0;
    Cpa32U uSeedLen = 0;
    Cpa32U uOutputLen = 0;

    LAC_CHECK_NULL_PARAM(pKeyGenOpData);
    LAC_CHECK_NULL_PARAM(pGeneratedKeyBuffer);
    LAC_CHECK_NULL_PARAM(pGeneratedKeyBuffer->pData);

    if(ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE == cmdId)
    {
        CpaCyKeyGenSslOpData *opData = (CpaCyKeyGenSslOpData *) pKeyGenOpData;

        /* User info */
        uSecretLen = opData->secret.dataLenInBytes;
        uSeedLen = opData->seed.dataLenInBytes;
        uOutputLen = opData->generatedKeyLenInBytes;

        /* Api max value */
        maxSecretLen = ICP_QAT_FW_LA_SSL_SECRET_LEN_MAX;
        maxSeedLen = ICP_QAT_FW_LA_SSL_SEED_LEN_MAX;
        maxOutputLen = ICP_QAT_FW_LA_SSL_OUTPUT_LEN_MAX;

        /* Check user buffers */
        LAC_CHECK_NULL_PARAM(opData->secret.pData);
        LAC_CHECK_NULL_PARAM(opData->seed.pData);

        /* check operation*/
        if((Cpa32U) opData->sslOp > CPA_CY_KEY_SSL_OP_USER_DEFINED)
        {
            LAC_INVALID_PARAM_LOG("opData->sslOp");
            return CPA_STATUS_INVALID_PARAM;
        }
        if ((Cpa32U) opData->sslOp  == CPA_CY_KEY_SSL_OP_USER_DEFINED)
        {
            LAC_CHECK_NULL_PARAM(opData->userLabel.pData);
            /* Maximum label length for SSL Key Gen request */
            if (opData->userLabel.dataLenInBytes >
                       ICP_QAT_FW_LA_SSL_LABEL_LEN_MAX)
            {
                LAC_INVALID_PARAM_LOG("userLabel.dataLenInBytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }

        /* only seed length for SSL3 Key Gen request */
        if (maxSeedLen != uSeedLen)
        {
            LAC_INVALID_PARAM_LOG("seed.dataLenInBytes");
            return CPA_STATUS_INVALID_PARAM;
        }
    }
    else
    {
        CpaCyKeyGenTlsOpData *opData = (CpaCyKeyGenTlsOpData *) pKeyGenOpData;

        /* User info */
        uSecretLen = opData->secret.dataLenInBytes;
        uSeedLen = opData->seed.dataLenInBytes;
        uOutputLen = opData->generatedKeyLenInBytes;

        if(ICP_QAT_FW_LA_CMD_TLS_V1_1_KEY_DERIVE == cmdId)
        {
            /* Api max value */
            /* ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX needs to be multiplied
             * by 4 in order to verifiy the 512 conditions. We did not change
             * ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX as it represents
             * the max value tha firmware can handle.
             */
            maxSecretLen = ICP_QAT_FW_LA_TLS_V1_1_SECRET_LEN_MAX * 4;
        }
        else
        {
            /* Api max value */
            /* ICP_QAT_FW_LA_TLS_V1_2_SECRET_LEN_MAX needs to be multiplied
             * by 8 in order to verifiy the 512 conditions. We did not change
             * ICP_QAT_FW_LA_TLS_V1_2_SECRET_LEN_MAX as it represents
             * the max value tha firmware can handle.
             */
            maxSecretLen = ICP_QAT_FW_LA_TLS_V1_2_SECRET_LEN_MAX * 8;

            /* Check Hash algorithm */
            if(0 == getDigestSizeFromHashAlgo(hashAlgorithm))
            {
                LAC_INVALID_PARAM_LOG("hashAlgorithm");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
        maxSeedLen = ICP_QAT_FW_LA_TLS_SEED_LEN_MAX;
        maxOutputLen = ICP_QAT_FW_LA_TLS_OUTPUT_LEN_MAX;
        /* Check user buffers */
        LAC_CHECK_NULL_PARAM(opData->secret.pData);
        LAC_CHECK_NULL_PARAM(opData->seed.pData);

        /* check operation*/
        if((Cpa32U) opData->tlsOp > CPA_CY_KEY_TLS_OP_USER_DEFINED)
        {
            LAC_INVALID_PARAM_LOG("opData->tlsOp");
            return CPA_STATUS_INVALID_PARAM;
        }
        else if ((Cpa32U) opData->tlsOp == CPA_CY_KEY_TLS_OP_USER_DEFINED)
        {
            LAC_CHECK_NULL_PARAM(opData->userLabel.pData);
            /* Maximum label length for TLS Key Gen request */
            if (opData->userLabel.dataLenInBytes >
                       ICP_QAT_FW_LA_TLS_LABEL_LEN_MAX)
            {
                LAC_INVALID_PARAM_LOG("userLabel.dataLenInBytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }

        /* Maximum/only seed length for TLS Key Gen request */
        if(((Cpa32U) opData->tlsOp != CPA_CY_KEY_TLS_OP_MASTER_SECRET_DERIVE) ||
           ((Cpa32U) opData->tlsOp != CPA_CY_KEY_TLS_OP_KEY_MATERIAL_DERIVE))
        {
            if (uSeedLen > maxSeedLen)
            {
                LAC_INVALID_PARAM_LOG("seed.dataLenInBytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
        else
        {
            if (maxSeedLen != uSeedLen)
            {
                LAC_INVALID_PARAM_LOG("seed.dataLenInBytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
    }

    /* Maximum secret length for TLS/SSL3 Key Gen request */
    if (uSecretLen > maxSecretLen)
    {
        LAC_INVALID_PARAM_LOG("secret.dataLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Maximum output length for TLS/SSL3 Key Gen request */
    if (uOutputLen > maxOutputLen)
    {
        LAC_INVALID_PARAM_LOG("generatedKeyLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Check for enough space in the flat buffer */
    if (uOutputLen > pGeneratedKeyBuffer->dataLenInBytes)
    {
        LAC_INVALID_PARAM_LOG("pGeneratedKeyBuffer->dataLenInBytes");
        return CPA_STATUS_INVALID_PARAM;
    }
    return CPA_STATUS_SUCCESS;
}
#endif

/**
 *
 */
/**
 * @ingroup LacSymKey
 *      Common Keygen Code for TLS v1.0/1.1, v1.2 and SSL3.
 * @description
 *      Check user parameters and perform the required operation.
 *
 * @param[in] instanceHandle_in          Instance handle.
 * @param[in] pKeyGenCb                  Pointer to callback function to be
 *                                       invoked when the operation is complete.
 *                                       If this is set to a NULL value the
 *                                       function will operate synchronously.
 * @param[in] pCallbackTag               Opaque User Data for this specific
 *                                       call. Will be returned unchanged in the
 *                                       callback.
 * @param[in] pKeyGenOpData              Pointer to a structure containing all
 *                                       the data needed to perform the key
 *                                       generation operation.
 * @param[in]  hashAlgorithm             Specifies the hash algorithm to use.
 *                                       According to RFC5246, this should be
 *                                       "SHA-256 or a stronger standard hash
 *                                       function."
 * @param[out] pGeneratedKeyBuffer       User output buffer.
 * @param[in] cmdId                      Keygen operation to perform.
 */
STATIC CpaStatus
LacSymKey_KeyGenSslTls(const CpaInstanceHandle instanceHandle_in,
                const CpaCyGenFlatBufCbFunc pKeyGenCb,
                void *pCallbackTag,
                const void *pKeyGenOpData,
                CpaCySymHashAlgorithm hashAlgorithm,
                CpaFlatBuffer *pGeneratedKeyBuffer,
                icp_qat_fw_la_cmd_id_t cmdId)
{
#ifdef ICP_PARAM_CHECK
    CpaStatus status = CPA_STATUS_FAIL;
#endif
    CpaInstanceHandle instanceHandle = LacKey_GetHandle(instanceHandle_in);

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif

    SAL_RUNNING_CHECK(instanceHandle);

#ifdef ICP_PARAM_CHECK
    status = LacSymKey_CheckParamSslTls(pKeyGenOpData,
                    hashAlgorithm,
                    pGeneratedKeyBuffer,
                    cmdId);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }
#endif

    return LacSymKey_KeyGenSslTls_GenCommon(instanceHandle,
                    pKeyGenCb,
                    pCallbackTag,
                    cmdId,
                    LAC_CONST_PTR_CAST(pKeyGenOpData),
                    hashAlgorithm,
                    pGeneratedKeyBuffer);
}

/**
 * @ingroup LacSymKey
 *      SSL Key Generation Function.
 * @description
 *      This function is used for SSL key generation.  It implements the key
 *      generation function defined in section 6.2.2 of the SSL 3.0
 *      specification as described in
 *      http://www.mozilla.org/projects/security/pki/nss/ssl/draft302.txt.
 *
 *      The input seed is taken as a flat buffer and the generated key is
 *      returned to caller in a flat destination data buffer.
 *
 * @param[in] instanceHandle_in          Instance handle.
 * @param[in] pKeyGenCb                  Pointer to callback function to be
 *                                       invoked when the operation is complete.
 *                                       If this is set to a NULL value the
 *                                       function will operate synchronously.
 * @param[in] pCallbackTag               Opaque User Data for this specific
 *                                       call. Will be returned unchanged in the
 *                                       callback.
 * @param[in] pKeyGenSslOpData           Pointer to a structure containing all
 *                                       the data needed to perform the SSL key
 *                                       generation operation. The client code
 *                                       allocates the memory for this
 *                                       structure. This component takes
 *                                       ownership of the memory until it is
 *                                       returned in the callback.
 * @param[out] pGeneratedKeyBuffer       Caller MUST allocate a sufficient
 *                                       buffer to hold the key generation
 *                                       output. The data pointer SHOULD be
 *                                       aligned on an 8-byte boundary. The
 *                                       length field passed in represents the
 *                                       size of the buffer in bytes. The value
 *                                       that is returned is the size of the
 *                                       result key in bytes.
 *                                       On invocation the callback function
 *                                       will contain this parameter in the
 *                                       pOut parameter.
 *
 * @retval CPA_STATUS_SUCCESS            Function executed successfully.
 * @retval CPA_STATUS_FAIL               Function failed.
 * @retval CPA_STATUS_RETRY              Resubmit the request.
 * @retval CPA_STATUS_INVALID_PARAM      Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE           Error related to system resources.
 */
CpaStatus
cpaCyKeyGenSsl(const CpaInstanceHandle instanceHandle_in,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const CpaCyKeyGenSslOpData *pKeyGenSslOpData,
               CpaFlatBuffer *pGeneratedKeyBuffer)
{

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pKeyGenCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pKeyGenSslOpData,
            (LAC_ARCH_UINT)pGeneratedKeyBuffer);
#endif

    return LacSymKey_KeyGenSslTls(instanceHandle_in,
        pKeyGenCb,
        pCallbackTag,
        LAC_CONST_PTR_CAST(pKeyGenSslOpData),
        CPA_CY_SYM_OP_NONE, /* hashAlgorithm */
        pGeneratedKeyBuffer,
        ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE);
}

/**
 * @ingroup LacSymKey
 *      TLS Key Generation Function.
 * @description
 *      This function is used for TLS key generation.  It implements the
 *      TLS PRF (Pseudo Random Function) as defined by RFC2246 (TLS v1.0)
 *      and RFC4346 (TLS v1.1).
 *
 *      The input seed is taken as a flat buffer and the generated key is
 *      returned to caller in a flat destination data buffer.
 *
 * @param[in]  instanceHandle_in         Instance handle.
 * @param[in]  pKeyGenCb                 Pointer to callback function to be
 *                                       invoked when the operation is complete.
 *                                       If this is set to a NULL value the
 *                                       function will operate synchronously.
 * @param[in]  pCallbackTag              Opaque User Data for this specific
 *                                       call. Will be returned unchanged in the
 *                                       callback.
 * @param[in]  pKeyGenTlsOpData          Pointer to a structure containing all
 *                                       the data needed to perform the TLS key
 *                                       generation operation. The client code
 *                                       allocates the memory for this
 *                                       structure. This component takes
 *                                       ownership of the memory until it is
 *                                       returned in the callback.
 * @param[out] pGeneratedKeyBuffer       Caller MUST allocate a sufficient
 *                                       buffer to hold the key generation
 *                                       output. The data pointer SHOULD be
 *                                       aligned on an 8-byte boundary. The
 *                                       length field passed in represents the
 *                                       size of the buffer in bytes. The value
 *                                       that is returned is the size of the
 *                                       result key in bytes.
 *                                       On invocation the callback function
 *                                       will contain this parameter in the
 *                                       pOut parameter.
 *
 * @retval CPA_STATUS_SUCCESS            Function executed successfully.
 * @retval CPA_STATUS_FAIL               Function failed.
 * @retval CPA_STATUS_RETRY              Resubmit the request.
 * @retval CPA_STATUS_INVALID_PARAM      Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE           Error related to system resources.
 *
 */
CpaStatus
cpaCyKeyGenTls(const CpaInstanceHandle instanceHandle_in,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const CpaCyKeyGenTlsOpData *pKeyGenTlsOpData,
               CpaFlatBuffer *pGeneratedKeyBuffer)
{

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pKeyGenCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pKeyGenTlsOpData,
            (LAC_ARCH_UINT)pGeneratedKeyBuffer);
#endif

    return LacSymKey_KeyGenSslTls(instanceHandle_in,
                    pKeyGenCb,
                    pCallbackTag,
                    LAC_CONST_PTR_CAST(pKeyGenTlsOpData),
                    CPA_CY_SYM_OP_NONE, /* hashAlgorithm */
                    pGeneratedKeyBuffer,
                    ICP_QAT_FW_LA_CMD_TLS_V1_1_KEY_DERIVE);
}

/**
 * @ingroup LacSymKey
 * @description
 *      This function is used for TLS key generation.  It implements the
 *      TLS PRF (Pseudo Random Function) as defined by RFC5246 (TLS v1.2).
 *
 *      The input seed is taken as a flat buffer and the generated key is
 *      returned to caller in a flat destination data buffer.
 *
 * @param[in]  instanceHandle_in         Instance handle.
 * @param[in]  pKeyGenCb                 Pointer to callback function to be
 *                                       invoked when the operation is complete.
 *                                       If this is set to a NULL value the
 *                                       function will operate synchronously.
 * @param[in]  pCallbackTag              Opaque User Data for this specific
 *                                       call. Will be returned unchanged in the
 *                                       callback.
 * @param[in]  pKeyGenTlsOpData          Pointer to a structure containing all
 *                                       the data needed to perform the TLS key
 *                                       generation operation. The client code
 *                                       allocates the memory for this
 *                                       structure. This component takes
 *                                       ownership of the memory until it is
 *                                       returned in the callback.
 * @param[in]  hashAlgorithm             Specifies the hash algorithm to use.
 *                                       According to RFC5246, this should be
 *                                       "SHA-256 or a stronger standard hash
 *                                       function."
 * @param[out] pGeneratedKeyBuffer       Caller MUST allocate a sufficient
 *                                       buffer to hold the key generation
 *                                       output. The data pointer SHOULD be
 *                                       aligned on an 8-byte boundary. The
 *                                       length field passed in represents the
 *                                       size of the buffer in bytes. The value
 *                                       that is returned is the size of the
 *                                       result key in bytes.
 *                                       On invocation the callback function
 *                                       will contain this parameter in the
 *                                       pOut parameter.
 *
 * @retval CPA_STATUS_SUCCESS            Function executed successfully.
 * @retval CPA_STATUS_FAIL               Function failed.
 * @retval CPA_STATUS_RETRY              Resubmit the request.
 * @retval CPA_STATUS_INVALID_PARAM      Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE           Error related to system resources.
 */
CpaStatus
cpaCyKeyGenTls2(const CpaInstanceHandle instanceHandle_in,
               const CpaCyGenFlatBufCbFunc pKeyGenCb,
               void *pCallbackTag,
               const CpaCyKeyGenTlsOpData *pKeyGenTlsOpData,
               CpaCySymHashAlgorithm hashAlgorithm,
               CpaFlatBuffer *pGeneratedKeyBuffer)
{
#ifdef ICP_TRACE
    LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, %d, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pKeyGenCb,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pKeyGenTlsOpData,
            hashAlgorithm,
            (LAC_ARCH_UINT)pGeneratedKeyBuffer);
#endif

    return LacSymKey_KeyGenSslTls(instanceHandle_in,
                    pKeyGenCb,
                    pCallbackTag,
                    LAC_CONST_PTR_CAST(pKeyGenTlsOpData),
                    hashAlgorithm,
                    pGeneratedKeyBuffer,
                    ICP_QAT_FW_LA_CMD_TLS_V1_2_KEY_DERIVE);
}

/*
 * LacSymKey_Init
 */
CpaStatus
LacSymKey_Init(CpaInstanceHandle instanceHandle_in)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = LacKey_GetHandle(instanceHandle_in);
    sal_crypto_service_t* pService = NULL;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
#endif

    pService = (sal_crypto_service_t*) instanceHandle;

    status = LAC_OS_MALLOC(&(pService->pLacKeyStats),
            LAC_KEY_NUM_STATS * sizeof(OsalAtomic));

    if(CPA_STATUS_SUCCESS == status)
    {
        LAC_OS_BZERO((void *)pService->pLacKeyStats,
                LAC_KEY_NUM_STATS * sizeof(OsalAtomic));

        status = LAC_OS_CAMALLOC(&pService->pSslLabel,
                    ICP_QAT_FW_LA_SSL_LABEL_LEN_MAX,
                    LAC_8BYTE_ALIGNMENT,
                    pService->nodeAffinity);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        Cpa32U i = 0;
        Cpa32U offset = 0;

        /* Initialise SSL label ABBCCC..... */
        for (i = 0; i < ICP_QAT_FW_LA_SSL_ITERATES_LEN_MAX; i++)
        {
            osalMemSet(pService->pSslLabel + offset, 'A' + i, i + 1);
            offset += (i + 1);
        }

        /* allocate memory for TLS labels */
        status = LAC_OS_CAMALLOC(&pService->pTlsLabel,
                    sizeof(lac_sym_key_tls_labels_t),
                    LAC_8BYTE_ALIGNMENT,
                    pService->nodeAffinity);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_OS_BZERO(pService->pTlsLabel, sizeof(lac_sym_key_tls_labels_t));

        /* copy the TLS labels into the dynamically allocated structure  */
        memcpy(pService->pTlsLabel->masterSecret,
               LAC_SYM_KEY_TLS_MASTER_SECRET_LABEL,
               sizeof(LAC_SYM_KEY_TLS_MASTER_SECRET_LABEL) - 1);

        memcpy(pService->pTlsLabel->keyMaterial,
                LAC_SYM_KEY_TLS_KEY_MATERIAL_LABEL,
                sizeof(LAC_SYM_KEY_TLS_KEY_MATERIAL_LABEL) - 1);

        memcpy(pService->pTlsLabel->clientFinished,
                LAC_SYM_KEY_TLS_CLIENT_FIN_LABEL,
                sizeof(LAC_SYM_KEY_TLS_CLIENT_FIN_LABEL) - 1);

        memcpy(pService->pTlsLabel->serverFinished,
                LAC_SYM_KEY_TLS_SERVER_FIN_LABEL,
                sizeof(LAC_SYM_KEY_TLS_SERVER_FIN_LABEL) - 1);

        LacSymQat_RespHandlerRegister(
            ICP_QAT_FW_LA_CMD_SSL3_KEY_DERIVE,
            LacSymKey_SslTlsHandleResponse);

        LacSymQat_RespHandlerRegister(
            ICP_QAT_FW_LA_CMD_TLS_V1_1_KEY_DERIVE,
            LacSymKey_SslTlsHandleResponse);

        LacSymQat_RespHandlerRegister(
            ICP_QAT_FW_LA_CMD_TLS_V1_2_KEY_DERIVE,
            LacSymKey_SslTlsHandleResponse);

        LacSymQat_RespHandlerRegister(
            ICP_QAT_FW_LA_CMD_MGF1,
            LacSymKey_MgfHandleResponse);
    }

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_OS_FREE(pService->pLacKeyStats);
        LAC_OS_CAFREE(pService->pSslLabel);
        LAC_OS_CAFREE(pService->pTlsLabel);
    }

    return status;
}


/*
 * LacSymKey_Shutdown
 */
CpaStatus
LacSymKey_Shutdown(CpaInstanceHandle instanceHandle_in)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = LacKey_GetHandle(instanceHandle_in);
    sal_crypto_service_t* pService = NULL;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
#endif

    pService = (sal_crypto_service_t*) instanceHandle;

    if(NULL != pService->pLacKeyStats)
    {
        LAC_OS_FREE(pService->pLacKeyStats);
    }

    LAC_OS_CAFREE(pService->pSslLabel);
    LAC_OS_CAFREE(pService->pTlsLabel);

    return status;
}
