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
 * @file lac_kpt_rsa_decrypt.c
 *
 * @ingroup Lac_KptRsa
 *
 * This file implements data decrypto function for KPT RSA.
 *
 *****************************************************************************/

/*
********************************************************************************
* Include public/global header files
********************************************************************************
*/

#include "cpa.h"
#include "cpa_cy_kpt.h"
#include "cpa_cy_rsa.h"

/*
********************************************************************************
* Include private header files
********************************************************************************
*/

/* Osal include */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp_ids.h"

/* Include LAC files */
#include "lac_common.h"
#include "lac_pke_qat_comms.h"
#include "lac_pke_utils.h"
#include "lac_pke_mmp.h"
#include "lac_sym.h"
#include "lac_list.h"
#include "sal_service_state.h"
#include "lac_sal_types_crypto.h"
#include "lac_rsa_p.h"
#include "lac_rsa_stats_p.h"
#include "lac_kpt_crypto_qat_comms.h"

#define KPT_RSA_TYPE1_MASK (0x2)
/*Input array is {c, d', n}, mask value is b010*/

#define KPT_RSA_TYPE2_MASK (0x3e)
/*Input array is {c, p', q',dp',dq',qinv'}, mask value is b111110*/

#define LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS (2)
/**< number of 'out' arguments in output arguments array*/

/*
********************************************************************************
* Static Variables
********************************************************************************
*/
static const Cpa32U lacKptRsaDp1SizeIdMap[][LAC_PKE_NUM_COLUMNS] = {
    {LAC_512_BITS, PKE_RSA_DP1_512},
    {LAC_1024_BITS, PKE_RSA_DP1_1024},
    {LAC_1536_BITS, PKE_RSA_DP1_1536},
    {LAC_2048_BITS, PKE_RSA_DP1_2048},
    {LAC_3072_BITS, PKE_RSA_DP1_3072},
    {LAC_4096_BITS, PKE_RSA_DP1_4096}};
/**<
 *  Maps between operation sizes and PKE function ids */

static const Cpa32U lacKptRsaDp2SizeIdMap[][LAC_PKE_NUM_COLUMNS] = {
    {LAC_512_BITS, PKE_RSA_DP2_512},
    {LAC_1024_BITS, PKE_RSA_DP2_1024},
    {LAC_1536_BITS, PKE_RSA_DP2_1536},
    {LAC_2048_BITS, PKE_RSA_DP2_2048},
    {LAC_3072_BITS, PKE_RSA_DP2_3072},
    {LAC_4096_BITS, PKE_RSA_DP2_4096}};
/*
********************************************************************************
* Define static function definitions
********************************************************************************
*/

/*
 * This function checks the parameters for an RSA decrypt operation. It returns
 * the appropriate error in the case of null and invalid params and also
 * unsupported operations.
 */
#ifdef ICP_PARAM_CHECK
STATIC CpaStatus
LacKptRsa_DecryptParamsCheck(const CpaInstanceHandle instanceHandle,
                             const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                             const CpaCyRsaDecryptOpData *pDecryptData,
                             CpaFlatBuffer *pOutputData,
                             CpaFlatBuffer *pKptUnwrapContext);
#endif

/*
 * This function is called by the pke comms module after an RSAType1 Encrypt
 * message has been received from the QAT.
 */
STATIC void LacKptRsaType1_ProcessDecCb(CpaStatus status,
                                        CpaBoolean pass,
                                        CpaInstanceHandle instanceHandle,
                                        lac_pke_op_cb_data_t *pCbData);

/*
 * This function is called by the pke comms module after an KPT RSAType2
 * decrypt message has been received from the QAT.
 */
STATIC void LacKptRsaType2_ProcessDecCb(CpaStatus status,
                                        CpaBoolean pass,
                                        CpaInstanceHandle instanceHandle,
                                        lac_pke_op_cb_data_t *pCbData);

/*
 * This function performs RSA Decrypt for type 1 private keys.
 */
STATIC CpaStatus
LacKptRsa_Type1Decrypt(const CpaInstanceHandle instanceHandle,
                       const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                       void *pCallbackTag,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext);

/*
 * This function performs RSA Decrypt for type 2 private keys.
 */
STATIC CpaStatus
LacKptRsa_Type2Decrypt(const CpaInstanceHandle instanceHandle,
                       const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                       void *pCallbackTag,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext);

/*
 * This is the LAC RSA Decrypt synchronous function.
 */
STATIC CpaStatus
LacKptRsa_DecryptSynch(const CpaInstanceHandle instanceHandle,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext);

STATIC Cpa32U
LacKptRsa_GetPrivateKeyOpSize(const CpaCyRsaPrivateKey *pPrivateKey,
                              const CpaCyKptWpkSize *pWpkSize)
{
    Cpa32U sizeInBytes = 0;
    LAC_ASSERT_NOT_NULL(pPrivateKey);
    switch (pPrivateKey->privateKeyRepType)
    {
        case CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_1:
        {
            sizeInBytes +=
                LacPke_GetMinBytes(&(pPrivateKey->privateKeyRep1.modulusN));
        }
        break;

        case CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_2:
        {
            sizeInBytes = pWpkSize->rsaWpkSizeRep2.pLenInBytes;
            if (sizeInBytes != pWpkSize->rsaWpkSizeRep2.qLenInBytes)
            {
                LAC_LOG_ERROR(
                    "prime1P.dataLenInBytes != prime2Q.dataLenInBytes");
                sizeInBytes = 0;
            }
            else
            {
                /* should subtract HMAC part */
                sizeInBytes = sizeInBytes << 1;
            }
        }
        break;

        default:
        {
            /* Invalid Key Type */
            LAC_LOG_ERROR("Invalid Private Key Type.");
        }
    }

    return sizeInBytes;
}

#ifdef ICP_PARAM_CHECK
STATIC CpaStatus
LacKptRsa_DecryptParamsCheck(const CpaInstanceHandle instanceHandle,
                             const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                             const CpaCyRsaDecryptOpData *pDecryptData,
                             CpaFlatBuffer *pOutputData,
                             CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U opSizeInBytes = 0;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;

    /* Check user parameters */
    LAC_CHECK_NULL_PARAM(pDecryptData);
    LAC_CHECK_FLAT_BUFFER_PARAM(
        pKptUnwrapContext, CHECK_EQUALS, sizeof(CpaCyKptUnwrapContext));

    /* Check the Private Key is correct version, type and for NULL params */
    status = LacRsa_CheckPrivateKeyParam(pDecryptData->pRecipientPrivateKey);
    LAC_CHECK_STATUS(status);

    pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
    pWpkSize = &(pUnwrapCxt->wpkSize);
    /* Get the opSize */
    opSizeInBytes = LacKptRsa_GetPrivateKeyOpSize(
        pDecryptData->pRecipientPrivateKey, pWpkSize);
    if (CPA_FALSE == LacRsa_IsValidRsaSize(opSizeInBytes))
    {
        LAC_INVALID_PARAM_LOG(
            "Invalid Private Key Size - pDecryptData->pRecipientPrivateKey");
        return CPA_STATUS_INVALID_PARAM;
    }
    /* Check message and ciphertext buffers */
    LAC_CHECK_FLAT_BUFFER_PARAM_PKE(&(pDecryptData->inputData),
                                    CHECK_LESS_EQUALS,
                                    opSizeInBytes,
                                    CPA_FALSE);
    LAC_CHECK_FLAT_BUFFER_PARAM(
        pOutputData, CHECK_GREATER_EQUALS, opSizeInBytes);

    if (CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_1 ==
        pDecryptData->pRecipientPrivateKey->privateKeyRepType)
    {
        /* Check MSB and LSB of the modulus */
        LAC_CHECK_RSA_BUFFER_MSB_LSB(
            &(pDecryptData->pRecipientPrivateKey->privateKeyRep1.modulusN),
            opSizeInBytes,
            CPA_TRUE,
            CPA_TRUE);

        /* Standards based check: 0 < inputData < n */
        LAC_CHECK_NON_ZERO_PARAM(&(pDecryptData->inputData));
        if (LacPke_Compare(
                &(pDecryptData->inputData),
                0,
                &(pDecryptData->pRecipientPrivateKey->privateKeyRep1.modulusN),
                0) >= 0)
        {
            LAC_INVALID_PARAM_LOG("inputData must be < modulusN");
            return CPA_STATUS_INVALID_PARAM;
        }
    }
    return status;
}
#endif

STATIC CpaStatus
LacKptRsa_DecryptSynch(const CpaInstanceHandle instanceHandle,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_FAIL;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    /*
     * Call the async version of the function
     * with the sync callback function as a parameter.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = cpaCyKptRsaDecrypt(instanceHandle,
                                    LacSync_GenFlatBufCb,
                                    pSyncCallbackData,
                                    pDecryptData,
                                    pOutputData,
                                    pKptUnwrapContext);
    }
    else
    {
        LAC_RSA_STAT_INC(numRsaDecryptRequestErrors, instanceHandle);
        return status;
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus wCbStatus = CPA_STATUS_FAIL;
        wCbStatus = LacSync_WaitForCallback(
            pSyncCallbackData, LAC_PKE_SYNC_CALLBACK_TIMEOUT, &status, NULL);
        if (CPA_STATUS_SUCCESS != wCbStatus)
        {
            /*
             * Inc stats only if the wait for callback failed.
             */
            LAC_RSA_STAT_INC(numRsaDecryptCompletedErrors, instanceHandle);
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
 *****************************************************************************
 * @ingroup Lac_KptRsa
 *
 *****************************************************************************/
CpaStatus cpaCyKptRsaDecrypt(const CpaInstanceHandle instanceHandle_in,
                             const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                             void *pCallbackTag,
                             const CpaCyRsaDecryptOpData *pDecryptData,
                             CpaFlatBuffer *pOutputData,
                             CpaFlatBuffer *pKptUnwrapContext)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
             "0x%lx)\n",
             (LAC_ARCH_UINT)instanceHandle_in,
             (LAC_ARCH_UINT)pRsaDecryptCb,
             (LAC_ARCH_UINT)pCallbackTag,
             (LAC_ARCH_UINT)pDecryptData,
             (LAC_ARCH_UINT)pOutputData);
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
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif
    SAL_RUNNING_CHECK(instanceHandle);
#ifdef ICP_PARAM_CHECK
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    /* Check if the API has been called in sync mode */
    if (NULL == pRsaDecryptCb)
    {
        return LacKptRsa_DecryptSynch(
            instanceHandle, pDecryptData, pOutputData, pKptUnwrapContext);
    }
#ifdef ICP_PARAM_CHECK
    /* Check RSA Decrypt params and return an error if invalid */
    status = LacKptRsa_DecryptParamsCheck(instanceHandle,
                                          pRsaDecryptCb,
                                          pDecryptData,
                                          pOutputData,
                                          pKptUnwrapContext);
#endif
    if (CPA_STATUS_SUCCESS == status)
    {
        if (CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_1 ==
            pDecryptData->pRecipientPrivateKey->privateKeyRepType)
        {
            status = LacKptRsa_Type1Decrypt(instanceHandle,
                                            pRsaDecryptCb,
                                            pCallbackTag,
                                            pDecryptData,
                                            pOutputData,
                                            pKptUnwrapContext);
        }
        else /* Must be type2 key as param check has passed */
        {
            status = LacKptRsa_Type2Decrypt(instanceHandle,
                                            pRsaDecryptCb,
                                            pCallbackTag,
                                            pDecryptData,
                                            pOutputData,
                                            pKptUnwrapContext);
        }
    }

    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_RSA_STAT_INC(numRsaDecryptRequests, instanceHandle);
    }
    else
    {
        LAC_RSA_STAT_INC(numRsaDecryptRequestErrors, instanceHandle);
    }

    return status;
}

STATIC CpaStatus
LacKptRsa_Type1Decrypt(const CpaInstanceHandle instanceHandle,
                       const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                       void *pCallbackTag,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext)
{
    Cpa32U opSizeInBytes = 0;
    Cpa32U functionalityId = LAC_PKE_INVALID_FUNC_ID;
    Cpa32U pInArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
    Cpa32U pOutArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};
    CpaBoolean internalMemInList[LAC_MAX_MMP_INPUT_PARAMS] = {CPA_FALSE};
    CpaBoolean internalMemOutList[LAC_MAX_MMP_OUTPUT_PARAMS] = {CPA_FALSE};
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_pke_op_cb_data_t cbData = {0};
    icp_qat_fw_mmp_input_param_t in = {.flat_array = {0}};
    icp_qat_fw_mmp_output_param_t out = {.flat_array = {0}};
    lac_pke_request_handle_t pRequestHandle = LAC_PKE_INVALID_HANDLE;
    Cpa8U *pMemPoolD = NULL;
    CpaFlatBuffer *pInBuffD = NULL;
    sal_crypto_service_t *pCryptoService = NULL;
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;

    LAC_ASSERT_NOT_NULL(pDecryptData);
    LAC_ASSERT_NOT_NULL(pOutputData);
    LAC_ASSERT_NOT_NULL(pKptUnwrapContext);
    LAC_ASSERT_NOT_NULL(pKptUnwrapContext->pData);

    pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
    pWpkSize = &(pUnwrapCxt->wpkSize);

    opSizeInBytes = LacKptRsa_GetPrivateKeyOpSize(
        pDecryptData->pRecipientPrivateKey, pWpkSize);
    functionalityId = LacPke_GetMmpId(LAC_BYTES_TO_BITS(opSizeInBytes),
                                      lacKptRsaDp1SizeIdMap,
                                      LAC_ARRAY_LEN(lacKptRsaDp1SizeIdMap));
    if (LAC_PKE_INVALID_FUNC_ID == functionalityId)
    {
        LAC_INVALID_PARAM_LOG("Invalid Kpt Private Key Size - "
                              "pDecryptData->pRecipientPrivateKey");
        status = CPA_STATUS_INVALID_PARAM;
    }
    else
    {
        pCryptoService = (sal_crypto_service_t *)instanceHandle;
        status = LacKptMemPoolMalloc(&pMemPoolD, pCryptoService->lac_kpt_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            /* Zero ms bytes of output buffer */
            osalMemSet(pOutputData->pData,
                       0,
                       (pOutputData->dataLenInBytes - opSizeInBytes));

            /* populate input parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp1_1024.c,
                                          &(pDecryptData->inputData));
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, c)] =
                opSizeInBytes;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, c)] =
                CPA_FALSE;

            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolD,
                pWpkSize->wpkLenInBytes,
                &(pDecryptData->pRecipientPrivateKey->privateKeyRep1
                      .privateExponentD));

            pInBuffD = (CpaFlatBuffer *)(pMemPoolD + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffD->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffD->pData = pMemPoolD;

            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp1_1024.d, pInBuffD);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, d)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, d)] =
                CPA_TRUE;

            LAC_MEM_SHARED_WRITE_FROM_PTR(
                in.mmp_rsa_dp1_1024.n,
                &(pDecryptData->pRecipientPrivateKey->privateKeyRep1.modulusN));
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, n)] =
                opSizeInBytes;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_input_t, n)] =
                CPA_FALSE;

            /* populate output parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(out.mmp_rsa_dp1_1024.m, pOutputData);
            pOutArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_output_t, m)] =
                opSizeInBytes;
            internalMemOutList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp1_output_t, m)] =
                CPA_FALSE;

            /*Set the last node of output array is pKptUnwrapContext*/
            out.flat_array[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            pOutArgSizeList[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                sizeof(CpaCyKptUnwrapContext);
            internalMemOutList[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                CPA_FALSE;

            /* populate callback data */
            cbData.pClientCb = pRsaDecryptCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pDecryptData;
            cbData.pOutputData1 = pOutputData;
            cbData.pOpaqueData = pMemPoolD;
            /* create a PKE request to the QAT */
            status = LacPke_CreateRequest(&pRequestHandle,
                                          functionalityId,
                                          pInArgSizeList,
                                          pOutArgSizeList,
                                          &in,
                                          &out,
                                          internalMemInList,
                                          internalMemOutList,
                                          LacKptRsaType1_ProcessDecCb,
                                          &cbData,
                                          instanceHandle);
            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the field mask */
                lac_pke_qat_req_data_t *pReqData =
                    (lac_pke_qat_req_data_t *)pRequestHandle;
                pReqData->u1.request.pke_hdr.kpt_mask = KPT_RSA_TYPE1_MASK;
                /* send request chain */
                status = LacPke_SendRequest(&pRequestHandle, instanceHandle);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            /* Free Mem Pool */
            LacKptMemPoolFree(pMemPoolD);
        }
    }
    return status;
}

STATIC CpaStatus
LacKptRsa_Type2Decrypt(const CpaInstanceHandle instanceHandle,
                       const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                       void *pCallbackTag,
                       const CpaCyRsaDecryptOpData *pDecryptData,
                       CpaFlatBuffer *pOutputData,
                       CpaFlatBuffer *pKptUnwrapContext)
{
    Cpa32U opSizeInBytes = 0;
    Cpa32U functionalityId = LAC_PKE_INVALID_FUNC_ID;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U pInArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
    Cpa32U pOutArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};
    CpaBoolean internalMemInList[LAC_MAX_MMP_INPUT_PARAMS] = {CPA_FALSE};
    CpaBoolean internalMemOutList[LAC_MAX_MMP_OUTPUT_PARAMS] = {CPA_FALSE};
    lac_pke_op_cb_data_t cbData = {0};
    icp_qat_fw_mmp_input_param_t in = {.flat_array = {0}};
    icp_qat_fw_mmp_output_param_t out = {.flat_array = {0}};
    lac_pke_request_handle_t pRequestHandle = LAC_PKE_INVALID_HANDLE;
    sal_crypto_service_t *pCryptoService = NULL;
    /* In KPT feature, we need to reconstrct input buffer*/
    CpaCyKptUnwrapContext *pUnwrapCxt = NULL;
    CpaCyKptWpkSize *pWpkSize = NULL;

    LAC_ASSERT_NOT_NULL(pDecryptData);
    LAC_ASSERT_NOT_NULL(pOutputData);
    LAC_ASSERT_NOT_NULL(pKptUnwrapContext);
    LAC_ASSERT_NOT_NULL(pKptUnwrapContext->pData);

    pUnwrapCxt = (CpaCyKptUnwrapContext *)(pKptUnwrapContext->pData);
    pWpkSize = &(pUnwrapCxt->wpkSize);

    opSizeInBytes = LacKptRsa_GetPrivateKeyOpSize(
        pDecryptData->pRecipientPrivateKey, pWpkSize);
    functionalityId = LacPke_GetMmpId(LAC_BYTES_TO_BITS(opSizeInBytes),
                                      lacKptRsaDp2SizeIdMap,
                                      LAC_ARRAY_LEN(lacKptRsaDp2SizeIdMap));
    if (LAC_PKE_INVALID_FUNC_ID == functionalityId)
    {
        LAC_INVALID_PARAM_LOG("Invalid Kpt Private Key Size - "
                              "pDecryptData->pRecipientPrivateKey");
        status = CPA_STATUS_INVALID_PARAM;
    }
    else
    {
        Cpa8U *pMemPoolP = NULL;
        Cpa8U *pMemPoolQ = NULL;
        Cpa8U *pMemPoolDp = NULL;
        Cpa8U *pMemPoolDq = NULL;
        Cpa8U *pMemPoolQinv = NULL;
        Cpa8U *pMemPoolArray = NULL;
        CpaFlatBuffer *pInBuffP = NULL;
        CpaFlatBuffer *pInBuffQ = NULL;
        CpaFlatBuffer *pInBuffDp = NULL;
        CpaFlatBuffer *pInBuffDq = NULL;
        CpaFlatBuffer *pInBuffQinv = NULL;
        pCryptoService = (sal_crypto_service_t *)instanceHandle;

        status = LacKptMemPoolMalloc(&pMemPoolArray,
                                     pCryptoService->lac_kpt_array_pool);
        if (CPA_STATUS_SUCCESS == status)
        {
            status =
                LacKptMemPoolMalloc(&pMemPoolP, pCryptoService->lac_kpt_pool);
            if (CPA_STATUS_SUCCESS == status)
            {
                status = LacKptMemPoolMalloc(&pMemPoolQ,
                                             pCryptoService->lac_kpt_pool);
                if (CPA_STATUS_SUCCESS == status)
                {
                    status = LacKptMemPoolMalloc(&pMemPoolDp,
                                                 pCryptoService->lac_kpt_pool);
                    if (CPA_STATUS_SUCCESS == status)
                    {
                        status = LacKptMemPoolMalloc(
                            &pMemPoolDq, pCryptoService->lac_kpt_pool);
                        if (CPA_STATUS_SUCCESS == status)
                        {
                            status = LacKptMemPoolMalloc(
                                &pMemPoolQinv, pCryptoService->lac_kpt_pool);
                        }
                    }
                }
            }
        }
        if (CPA_STATUS_SUCCESS == status)
        {
            Cpa8U **ppMemArray = NULL;
            /* Zero ms bytes of output buffer */
            osalMemSet(pOutputData->pData,
                       0,
                       (pOutputData->dataLenInBytes - opSizeInBytes));
            /* populate input parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.c,
                                          &(pDecryptData->inputData));
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, c)] =
                opSizeInBytes;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, c)] =
                CPA_FALSE;

            /*p*/
            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolP,
                pWpkSize->rsaWpkSizeRep2.pLenInBytes,
                &(pDecryptData->pRecipientPrivateKey->privateKeyRep2.prime1P));
            pInBuffP = (CpaFlatBuffer *)(pMemPoolP + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffP->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffP->pData = pMemPoolP;
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.p, pInBuffP);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, p)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, p)] =
                CPA_TRUE;

            /*q*/
            LacKpt_Crypto_BuildFlatBuf(
                pMemPoolQ,
                pWpkSize->rsaWpkSizeRep2.qLenInBytes,
                &(pDecryptData->pRecipientPrivateKey->privateKeyRep2.prime2Q));
            pInBuffQ = (CpaFlatBuffer *)(pMemPoolQ + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffQ->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffQ->pData = pMemPoolQ;
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.q, pInBuffQ);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, q)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, q)] =
                CPA_TRUE;

            /*dp*/
            LacKpt_Crypto_BuildFlatBuf(pMemPoolDp,
                                       pWpkSize->rsaWpkSizeRep2.dpLenInBytes,
                                       &(pDecryptData->pRecipientPrivateKey
                                             ->privateKeyRep2.exponent1Dp));
            pInBuffDp = (CpaFlatBuffer *)(pMemPoolDp + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffDp->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffDp->pData = pMemPoolDp;
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.dp, pInBuffDp);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, dp)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, dp)] =
                CPA_TRUE;

            /*dq*/
            LacKpt_Crypto_BuildFlatBuf(pMemPoolDq,
                                       pWpkSize->rsaWpkSizeRep2.dqLenInBytes,
                                       &(pDecryptData->pRecipientPrivateKey
                                             ->privateKeyRep2.exponent2Dq));
            pInBuffDq = (CpaFlatBuffer *)(pMemPoolDq + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffDq->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffDq->pData = pMemPoolDq;
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.dq, pInBuffDq);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, dq)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, dq)] =
                CPA_TRUE;

            /*qInv*/
            LacKpt_Crypto_BuildFlatBuf(pMemPoolQinv,
                                       pWpkSize->rsaWpkSizeRep2.qinvLenInBytes,
                                       &(pDecryptData->pRecipientPrivateKey
                                             ->privateKeyRep2.coefficientQInv));
            pInBuffQinv =
                (CpaFlatBuffer *)(pMemPoolQinv + LAC_KPT_SIZE_BYTES_MAX);
            pInBuffQinv->dataLenInBytes = LAC_KPT_SIZE_BYTES_MAX;
            pInBuffQinv->pData = pMemPoolQinv;
            LAC_MEM_SHARED_WRITE_FROM_PTR(in.mmp_rsa_dp2_1024.qinv,
                                          pInBuffQinv);
            pInArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t, qinv)] =
                LAC_KPT_SIZE_BYTES_MAX;
            internalMemInList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_input_t,
                                         qinv)] = CPA_TRUE;

            /* populate output parameters */
            LAC_MEM_SHARED_WRITE_FROM_PTR(out.mmp_rsa_dp2_1024.m, pOutputData);
            pOutArgSizeList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_output_t, m)] =
                opSizeInBytes;
            internalMemOutList[LAC_IDX_OF(icp_qat_fw_mmp_rsa_dp2_output_t, m)] =
                CPA_FALSE;

            /*Set the last node of output array is pKptUnwrapContext*/
            out.flat_array[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                (LAC_ARCH_UINT)pKptUnwrapContext;
            pOutArgSizeList[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                sizeof(CpaCyKptUnwrapContext);
            internalMemOutList[LAC_KPT_RSA_DECRYPT_NUM_OUT_ARGS - 1] =
                CPA_FALSE;

            ppMemArray = (Cpa8U **)pMemPoolArray;
            ppMemArray[0] = pMemPoolP;
            ppMemArray[1] = pMemPoolQ;
            ppMemArray[2] = pMemPoolDp;
            ppMemArray[3] = pMemPoolDq;
            ppMemArray[4] = pMemPoolQinv;

            /* populate callback data */
            cbData.pClientCb = pRsaDecryptCb;
            cbData.pCallbackTag = pCallbackTag;
            cbData.pClientOpData = pDecryptData;
            cbData.pOutputData1 = pOutputData;
            cbData.pOpaqueData = ppMemArray;

            /* create a PKE request to the QAT */
            status = LacPke_CreateRequest(&pRequestHandle,
                                          functionalityId,
                                          pInArgSizeList,
                                          pOutArgSizeList,
                                          &in,
                                          &out,
                                          internalMemInList,
                                          internalMemOutList,
                                          LacKptRsaType2_ProcessDecCb,
                                          &cbData,
                                          instanceHandle);
            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the mask field */
                lac_pke_qat_req_data_t *pReqData =
                    (lac_pke_qat_req_data_t *)pRequestHandle;
                pReqData->u1.request.pke_hdr.kpt_mask = KPT_RSA_TYPE2_MASK;
                /* send request chain */
                status = LacPke_SendRequest(&pRequestHandle, instanceHandle);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LacKptMemPoolArrayFree(pMemPoolArray,
                                   pMemPoolP,
                                   pMemPoolQ,
                                   pMemPoolDp,
                                   pMemPoolDq,
                                   pMemPoolQinv);
        }
    }

    return status;
}

STATIC void LacKptRsaType1_ProcessDecCb(CpaStatus status,
                                        CpaBoolean pass,
                                        CpaInstanceHandle instanceHandle,
                                        lac_pke_op_cb_data_t *pCbData)
{
    CpaCyGenFlatBufCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyRsaDecryptOpData *pOpData = NULL;
    CpaFlatBuffer *pOutputData = NULL;
    Cpa8U *pMemPoolD = NULL;
    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCallbackTag = (void *)pCbData->pCallbackTag;

    pOpData =
        (CpaCyRsaDecryptOpData *)LAC_CONST_PTR_CAST(pCbData->pClientOpData);
    LAC_ASSERT_NOT_NULL(pOpData);

    pCb = (CpaCyGenFlatBufCbFunc)LAC_CONST_PTR_CAST(pCbData->pClientCb);
    LAC_ASSERT_NOT_NULL(pCb);

    pOutputData = pCbData->pOutputData1;
    LAC_ASSERT_NOT_NULL(pOutputData);

    pMemPoolD = (Cpa8U *)(pCbData->pOpaqueData);
    LAC_ASSERT_NOT_NULL(pMemPoolD);
    /* Free Mem Pool Entry */
    Lac_MemPoolEntryFree(pMemPoolD);

    /* increment stats */
    LAC_RSA_STAT_INC(numRsaDecryptCompleted, instanceHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_RSA_STAT_INC(numRsaDecryptCompletedErrors, instanceHandle);
    }

    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, pOutputData);
}

STATIC void LacKptRsaType2_ProcessDecCb(CpaStatus status,
                                        CpaBoolean pass,
                                        CpaInstanceHandle instanceHandle,
                                        lac_pke_op_cb_data_t *pCbData)
{
    CpaCyGenFlatBufCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyRsaDecryptOpData *pOpData = NULL;
    CpaFlatBuffer *pOutputData = NULL;
    Cpa8U **ppMemPoolArray = NULL;
    Cpa8U *pMemPool = NULL;
    Cpa32S i = 0;
    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCallbackTag = (void *)pCbData->pCallbackTag;

    pOpData =
        (CpaCyRsaDecryptOpData *)LAC_CONST_PTR_CAST(pCbData->pClientOpData);
    LAC_ASSERT_NOT_NULL(pOpData);

    pCb = (CpaCyGenFlatBufCbFunc)LAC_CONST_PTR_CAST(pCbData->pClientCb);
    LAC_ASSERT_NOT_NULL(pCb);

    pOutputData = pCbData->pOutputData1;
    LAC_ASSERT_NOT_NULL(pOutputData);

    ppMemPoolArray = (Cpa8U **)(pCbData->pOpaqueData);
    LAC_ASSERT_NOT_NULL(ppMemPoolArray);
    /* Free Mem Pool Entry */
    for (i = 0; i < 5; i++)
    {
        pMemPool = ppMemPoolArray[i];
        LAC_ASSERT_NOT_NULL(pMemPool);
        LacKptMemPoolFree(pMemPool);
        ppMemPoolArray[i] = NULL;
    }
    LacKptMemPoolFree((Cpa8U *)ppMemPoolArray);

    /* increment stats */
    LAC_RSA_STAT_INC(numRsaDecryptCompleted, instanceHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_RSA_STAT_INC(numRsaDecryptCompletedErrors, instanceHandle);
    }

    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, pOutputData);
}
