/******************************************************************************
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
 * @file lac_sym_drbg_general.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *     Implementation of the Deterministic Random Bit Generation API - general
 *     functions
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"
#include "cpa_cy_drbg.h"

#include "icp_sal_drbg_impl.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_poll.h"
#include "icp_sal_poll.h"
#include "icp_buffer_desc.h"
#include "sal_service_state.h"
#include "lac_sym.h"
#include "lac_sym_alg_chain.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_sym_drbg.h"

/* Authentication key used for AES CBC MAC operation in BCC function;
 * depending on implementation it is used either in full or only first
 * 128 bits are used */
STATIC Cpa8U bccKey[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      Updates symmetric session params
 *
 * @description
 *      This function updates symmetric cipher session params with actual
 *      cipher key, cipher algorithm and callback function that will be used in
 *      the next cipher request. This works under assumption that there is only
 *      one in-flight request allowed.
 *
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSessionDesc           Pointer to symmetric session descriptor
 * @param[in]  pSymCb                 Callback function that is to be
 *                                    registered with the session
 * @param[in]  cipherAlgorithm        Cipher algorithm that is to be used with
 *                                    the next cipher request; currently only
 *                                    AES in ECB, CBC and CTR mode is used
 * @param[in]  pKey                   Pointer to a key that will be used in a
 *                                    cipher request
 *
 * @return None
 *
 *****************************************************************************/
STATIC void
LacDrbg_ChangeSymSessionParams(lac_drbg_internal_state_t * pInternalState,
            lac_session_desc_t *pSessionDesc,
            CpaCySymCbFunc pSymCb,
            CpaCySymCipherAlgorithm cipherAlgorithm,
            Cpa8U* pKey)
{
    icp_qat_hw_cipher_mode_t mode = ICP_QAT_HW_CIPHER_ECB_MODE;
    icp_qat_hw_cipher_algo_t algorithm = ICP_QAT_HW_CIPHER_ALGO_NULL;
    icp_qat_hw_cipher_dir_t dir = ICP_QAT_HW_CIPHER_ENCRYPT;
    icp_qat_hw_cipher_convert_t key_convert = ICP_QAT_HW_CIPHER_NO_CONVERT;
    sal_qat_content_desc_info_t *pCdInfo = &(pSessionDesc->contentDescInfo);
    Cpa8U* pCtrlBlock = (Cpa8U *)pCdInfo->pData;

    Cpa32U ctrlBlockHdrSz = sizeof(icp_qat_fw_cipher_hdr_t);
    Cpa8U *pHwBlockBase =  pCtrlBlock + ctrlBlockHdrSz;

    icp_qat_hw_cipher_config_t *pCipherConfig =
                    (icp_qat_hw_cipher_config_t *) pHwBlockBase;
    Cpa8U *pCipherKey = (Cpa8U *)pHwBlockBase
                            + sizeof(icp_qat_hw_cipher_config_t);

    /* Set the ICP algorithm */
    if( LAC_DRBG_KEYLEN128_IN_BYTES == pInternalState->nImplKeyLen )
    {
        algorithm = ICP_QAT_HW_CIPHER_ALGO_AES128;
    }
    else
    {
        algorithm = ICP_QAT_HW_CIPHER_ALGO_AES256;
    }

    /* Check & set the block mode */
    if (LAC_CIPHER_IS_CTR_MODE(cipherAlgorithm))
    {
        mode = ICP_QAT_HW_CIPHER_CTR_MODE;
    }
    else if(LAC_CIPHER_IS_CBC_MODE(cipherAlgorithm))
    {
        mode = ICP_QAT_HW_CIPHER_CBC_MODE;
    }
    else
    {
        /* ECB mode */
        mode = ICP_QAT_HW_CIPHER_ECB_MODE;
    }

    /* Now, when everything is known, update session params */
    /* Set the symmetric callback function */
    pSessionDesc->pSymCb = pSymCb;
    /* Set the algorithm */
    pSessionDesc->cipherAlgorithm = cipherAlgorithm;

    pCipherConfig->val = ICP_QAT_HW_CIPHER_CONFIG_BUILD(
                             mode, algorithm, key_convert, dir);

    /* Copy the encryption key */
    osalMemCopy(pCipherKey, pKey, pInternalState->nImplKeyLen);

}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_StatsInit(sal_crypto_service_t* pService)
{
    Cpa32U numStats = sizeof(CpaCyDrbgStats64) / sizeof(Cpa64U);
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = LAC_OS_MALLOC(&pService->pLacDrbgStatsArr,
            numStats * sizeof(OsalAtomic));
    if(CPA_STATUS_SUCCESS == status)
    {
        LAC_OS_BZERO(
            (void *) LAC_CONST_PTR_CAST(pService->pLacDrbgStatsArr),
            numStats * sizeof(OsalAtomic));
    }
    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
void
LacDrbg_StatsFree(sal_crypto_service_t* pService)
{
    if(NULL != pService->pLacDrbgStatsArr)
    {
        LAC_OS_FREE(pService->pLacDrbgStatsArr);
    }
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_Encrypt(sal_crypto_service_t* pService,
               lac_drbg_internal_state_t * pInternalState,
               CpaCySymCbFunc pSymCb,
               lac_drbg_callback_data_t *pCbData,
               CpaCySymCipherAlgorithm algorithm,
               Cpa8U* pKey,
               Cpa8U* pIv,
               CpaBufferList *pSrcBuffer,
               CpaBufferList *pDstBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_session_desc_t * pSessionDesc = NULL;
    CpaCySymOpData *pOpData = NULL;
    Cpa32U index = 0;
    Cpa32U numRetries = 0;

    pSessionDesc = (lac_session_desc_t *)*(LAC_ARCH_UINT *)
                        ((Cpa8U*)pInternalState
                                + sizeof(lac_drbg_internal_state_t));

    /* prepare the opData */
    pOpData = &(pInternalState->buffers.opData);
    LAC_OS_BZERO(pOpData, sizeof(CpaCySymOpData));
    if( NULL != pIv )
    {
        pOpData->ivLenInBytes = LAC_DRBG_OUTLEN_IN_BYTES;
    }
    else
    {
        pOpData->ivLenInBytes = 0;
    }
    pOpData->pIv = pIv;
    pOpData->sessionCtx = (CpaCySymSessionCtx)((Cpa8U*)pInternalState
            + sizeof(lac_drbg_internal_state_t));
    pOpData->packetType = CPA_CY_SYM_PACKET_TYPE_FULL;

    for(index = 0; index < pSrcBuffer->numBuffers; index++ )
    {
        pOpData->messageLenToCipherInBytes +=
            pSrcBuffer->pBuffers[index].dataLenInBytes;
    }

    /* update session */
    LacDrbg_ChangeSymSessionParams(pInternalState,
                pSessionDesc,
                pSymCb,
                algorithm,
                pKey);

    /* call perform op */
    do
    {
    status = LacAlgChain_Perform((CpaInstanceHandle)pService,
                pSessionDesc,
                (void*)pCbData, /* callback tag */
                pOpData,
                pSrcBuffer,
                pDstBuffer,
                NULL);
    }
    while(CPA_STATUS_RETRY == status
            && LAC_DRBG_MAX_RETRIES > numRetries++);

    if( CPA_STATUS_RETRY == status )
    {
        LAC_LOG_ERROR("Too many retries happened");
        status = CPA_STATUS_FAIL;
    }

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_Update(sal_crypto_service_t *pService,
              lac_drbg_internal_state_t * pInternalState,
              Cpa8U* pData,
              LacDrbgNextOpCbFunc pNextOpCb,
              void * pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pTemp = NULL, *pKey = NULL, *pV = NULL;
    Cpa32U i=0;

    CpaBufferList *pSrcBuffer = NULL;
    Cpa32U bufferSizeUsed = 0;
    CpaCySymCbFunc pSymCb = NULL;
    lac_drbg_callback_data_t *pCbData = NULL;
#ifdef DRBG_POLL_AND_WAIT
    Cpa32U retries=0;
    Cpa32U drbgPollAndWaitTime=0;

    drbgPollAndWaitTime = pService->drbgPollAndWaitTime;
#endif

    pKey = pInternalState->workingState.key;
    pV = pInternalState->workingState.v;

    /* NIST SP 800-90 CRT_DRBG Update Process:
     * 1. temp = Null
     * 2. While(len(temp)<seedlen) do
     *     2.1 V = (V+1) mod 2^outlen
     *     2.2 output_block = Block_Encrypt(Key, V)
     *     2.3 temp = temp || output_block
     * 3. temp = Leftmost seedlen bits of temp
     * 4. temp = temp xor provided_data
     * 5. Key = Leftmost keylen bits of temp
     * 6. V = Rightmost outlen bits of temp
     * 7. Return the new values of Key and V
     */

    /* Carve the memory for BufferList that will contain data for encryption
     * Memory layout is as follows:
     * ------------------------------------------------------------
     * | CpaBufferList | CpaFlatBuffer | pData | pPrivateMetaData |
     * ------------------------------------------------------------
     */
    pSrcBuffer = (CpaBufferList *)pInternalState->buffers.buff;
    bufferSizeUsed += sizeof(CpaBufferList);

    pSrcBuffer->numBuffers = 1;
    pSrcBuffer->pBuffers = (CpaFlatBuffer *)(pSrcBuffer +1);
    bufferSizeUsed += sizeof(CpaFlatBuffer);

    /* Extra one block added at the beginning of encrypted data so
     * we don't need to increment counter manually. After encryption
     * discard first OUTLEN bytes */
    pSrcBuffer->pBuffers->dataLenInBytes =
        pInternalState->nImplSeedLen + LAC_DRBG_OUTLEN_IN_BYTES;
    pSrcBuffer->pBuffers->pData = (Cpa8U*)(pSrcBuffer->pBuffers + 1);
    bufferSizeUsed += pInternalState->nImplSeedLen + LAC_DRBG_OUTLEN_IN_BYTES;

    pSrcBuffer->pPrivateMetaData =
        pSrcBuffer->pBuffers->pData + pSrcBuffer->pBuffers->dataLenInBytes;
    bufferSizeUsed += pInternalState->maxBufferListMetaSize;

    /* Set the convenience pointer and set the data to be encrypted.
     * Step 2. is equivalent to a single CTR operation where V is the
     * counter and string of zeros is encrypted. The input data will be
     * then zeroed */
    pTemp = pSrcBuffer->pBuffers->pData;
    LAC_OS_BZERO(pTemp, pInternalState->nImplSeedLen
            + LAC_DRBG_OUTLEN_IN_BYTES);
    pTemp += LAC_DRBG_OUTLEN_IN_BYTES;

    /* Set the encrypt function operation parameters */
    pCbData = &pInternalState->buffers.cbDataForUpdate;

    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;

    if( NULL == pNextOpCb )
    {
        /* Synchronous call, coming from Instantiate */
        pSymCb = LacDrbg_SyncCb;
        status = LacSync_CreateSyncCookie(
                &pCbData->cbOpData.syncCbData.pSyncCallbackData);

        if(status != CPA_STATUS_SUCCESS)
        {
            LAC_LOG_ERROR("Sync cookie allocation error");
            return status;
        }
        pCbData->pNextOpCb = NULL;
    }
    else

    {
        /* Asynchronous call */
        pSymCb = LacDrbg_UpdateASyncCb;
        pCbData->cbOpData.asyncUpdateCbData.pData = pData;
        pCbData->pNextOpCb = pNextOpCb;
        pCbData->pNextOpCallbackTag = pCallbackTag;
    }

    status = LacDrbg_Encrypt(pService,
                            pInternalState,
                            pSymCb,
                            pCbData, /* Callback tag for sym callback */
                            CPA_CY_SYM_CIPHER_AES_CTR,
                            pKey,
                            pV,
                            pSrcBuffer,
                            pSrcBuffer);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Encrypt request failed");
        if( NULL == pNextOpCb )
        {
            /* As the Request was not sent the Callback will never
             * be called, so need to indicate that we're finished
             * with cookie so it can be destroyed. */
            LacSync_SetSyncCookieComplete(
                    pCbData->cbOpData.syncCbData.pSyncCallbackData);
            /* Destroy sync cookie */
            LacSync_DestroySyncCookie(
                    &pCbData->cbOpData.syncCbData.pSyncCallbackData);
        }
        return status;
    }

    /* Here check if we are operating in sync or async mode.
     * If sync mode - when called from Instantiate - than wait
     * for callback from Encrypt; else return;
     */
    if( NULL == pNextOpCb )
    {
        CpaStatus syncStatus = CPA_STATUS_SUCCESS;
        CpaBoolean opResult = CPA_FALSE;

#ifdef DRBG_POLL_AND_WAIT
        retries=0;
        do
        {
             /* Poll the low priority response ring */
             syncStatus = icp_adf_pollInstance(&(pService->trans_handle_sym_rx_lo), 1, 0);
             if (CPA_STATUS_FAIL == syncStatus)
             {
                 LAC_LOG_ERROR("Poll Instance Failed");
                 break;
             }
             syncStatus = LacSync_CheckForCallback(
                pCbData->cbOpData.syncCbData.pSyncCallbackData,
                &status,
                &opResult);
             if (CPA_STATUS_RETRY == syncStatus)
             {
                 osalSleep(drbgPollAndWaitTime);
             }
             retries++;
        } while ((syncStatus != CPA_STATUS_SUCCESS) &&
                (retries<(LAC_SYM_DRBG_SYNC_CALLBACK_TIMEOUT /
                          drbgPollAndWaitTime)));
#else
        syncStatus = LacSync_WaitForCallback(
                pCbData->cbOpData.syncCbData.pSyncCallbackData,
                LAC_SYM_DRBG_SYNC_CALLBACK_TIMEOUT,
                &status,
                &opResult);
#endif

        /* If callback doesn't come back */
        if (CPA_STATUS_SUCCESS != syncStatus)
        {
            LAC_LOG_ERROR("Callback timed out");
            status = syncStatus;
        }

        /* No point in continuing operation if cipher failed */
        if( CPA_STATUS_SUCCESS == status )
        {
            /* 4. temp = temp xor provided_data */
            for(i=0; i < pInternalState->nImplSeedLen; i++)
            {
                pTemp[i] ^= pData[i];
            }

            /* 5. Key = Leftmost keylen bits of temp */
            osalMemCopy(pKey, pTemp, pInternalState->nImplKeyLen);

            /* 6. V = Rightmost outlen bits of temp */
            osalMemCopy(pV,
                    pTemp + pInternalState->nImplKeyLen,
                    LAC_DRBG_OUTLEN_IN_BYTES);

            /* Indicate change of working state */
            pInternalState->isWorkingStateModified = CPA_TRUE;
        }

        /* Destroy sync cookie */
        LacSync_DestroySyncCookie(
                &pCbData->cbOpData.syncCbData.pSyncCallbackData);

    }

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_InitSymSession(sal_crypto_service_t *pService,
                      lac_drbg_internal_state_t * pInternalState,
                      CpaCySymSessionCtx pSessionCtx,
                      CpaBoolean isCipherSession)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_session_desc_t *pSessionDesc = NULL;
    CpaPhysicalAddr physAddress = 0;
    CpaPhysicalAddr physAddressAligned = 0;
    /* Structure initializer is supported by C99, but it is
     * not supported by some old Intel compiler
     */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaCySymSessionSetupData sessionSetupData = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    Cpa8U key[pInternalState->nImplKeyLen];

    /* Prepare session setup data */

    if( CPA_TRUE == isCipherSession )
    {
        sessionSetupData.symOperation = CPA_CY_SYM_OP_CIPHER;
        sessionSetupData.sessionPriority = CPA_CY_PRIORITY_NORMAL;
        sessionSetupData.cipherSetupData.cipherAlgorithm
                        = CPA_CY_SYM_CIPHER_AES_CTR;
        sessionSetupData.cipherSetupData.cipherDirection
                        = CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;
        sessionSetupData.cipherSetupData.cipherKeyLenInBytes
                            = pInternalState->nImplKeyLen;
        /* empty key, for initialization purposes only; actual key will
         * be injected into session descriptor before sending cipher request*/
        sessionSetupData.cipherSetupData.pCipherKey = key;
    }
    else
    {
        sessionSetupData.symOperation = CPA_CY_SYM_OP_HASH;
        sessionSetupData.sessionPriority = CPA_CY_PRIORITY_NORMAL;
        sessionSetupData.hashSetupData.digestResultLenInBytes =
            LAC_DRBG_OUTLEN_IN_BYTES;
        sessionSetupData.hashSetupData.hashMode =
            CPA_CY_SYM_HASH_MODE_AUTH;
        sessionSetupData.hashSetupData.hashAlgorithm =
            CPA_CY_SYM_HASH_AES_CBC_MAC;

        sessionSetupData.hashSetupData.authModeSetupData.authKey = bccKey;
        sessionSetupData.hashSetupData.authModeSetupData.authKeyLenInBytes =
            pInternalState->nImplKeyLen;
    }

    /* Re-align the session structure to 64 byte alignment */
    physAddress = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                        (*pService), (Cpa8U *)pSessionCtx + sizeof(void *));

    if (physAddress == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the session\n");
        return CPA_STATUS_FAIL;
    }

    physAddressAligned = LAC_ALIGN_POW2_ROUNDUP(
                        physAddress, LAC_64BYTE_ALIGNMENT);

    pSessionDesc = (lac_session_desc_t *)
    /* Move the session pointer by the physical offset
                        between aligned and unaligned memory */
           ((Cpa8U *) pSessionCtx + sizeof(void *)
            + (physAddressAligned-physAddress));

    /* save the aligned pointer in the first bytes (size of LAC_ARCH_UINT)
     * of the session memory */
    *((LAC_ARCH_UINT *)pSessionCtx) = (LAC_ARCH_UINT)pSessionDesc;

    /* Setup content descriptor info structure
     * assumption that content descriptor is the first field in
     * in the session descriptor */
    pSessionDesc->contentDescInfo.pData = (Cpa8U *)pSessionDesc;
    pSessionDesc->contentDescInfo.dataPhys = physAddressAligned;

    /* Set the Common Session Information */
    pSessionDesc->symOperation = sessionSetupData.symOperation;

    pSessionDesc->pSymCb = NULL;

    /* Set the session priority for QAT */
    pSessionDesc->qatSessionPriority = QAT_COMMS_PRIORITY_NORMAL;

    pSessionDesc->isAuthEncryptOp = CPA_FALSE;
    pSessionDesc->digestVerify = CPA_FALSE;
    pSessionDesc->isDPSession = CPA_FALSE;
    /* Mark session as internal so the symmetric statistics are not updated */
    pSessionDesc->internalSession = CPA_TRUE;

    status = LacAlgChain_SessionInit((CpaInstanceHandle)pService,
                &sessionSetupData, pSessionDesc);

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_Reseed(sal_crypto_service_t* pService,
              CpaCyDrbgSessionHandle sessionHandle,
              CpaFlatBuffer *pAdditionalInput,
              LacDrbgNextOpCbFunc pNextOpFunc,
              void *pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaFlatBuffer *pEntropyBuffer = NULL;
    Cpa32U *pEntropyLenReturned = NULL;
    Cpa32U minEntropy = 0;
    icp_sal_drbg_get_entropy_op_data_t *pEntropyOpData = NULL;
    lac_drbg_callback_data_t *pCbData = NULL;
    lac_drbg_callback_data_t *pCbDataForGetEntropy = NULL;
    lac_drbg_internal_state_t * pInternalState =
            LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    lac_drbg_op_type_t operationType = LAC_DRBG_OPERATION_NONE;

    LAC_ENSURE( NULL != pInternalState, "Internal state pointer is NULL");

    if( NULL == pInternalState->pGetEntropyInputFunc )
    {
        LAC_LOG_ERROR("Get_entropy_input function not registered");
        return CPA_STATUS_FAIL;
    }

    operationType = pInternalState->operationType;

    /* Setup the entropy input buffer */
    LAC_RBG_SEC_STRENGTH_TO_NUM(
            pInternalState->adminInfo.drbgSecurityStrength, minEntropy);

    pEntropyBuffer = &pInternalState->buffers.entropyInputBuffer;
    pEntropyBuffer->pData = pInternalState->buffers.entropyInput;
    pEntropyBuffer->dataLenInBytes = pInternalState->nImplSeedLen;

    pEntropyOpData = &pInternalState->buffers.getEntropyOpData;
    pEntropyOpData->sessionHandle = sessionHandle;
    pEntropyOpData->maxLength = pInternalState->nImplSeedLen;
    if( CPA_FALSE == pInternalState->isDFRequired )
    {
        pEntropyOpData->minLength = pInternalState->nImplSeedLen;
    }
    else
    {
        pEntropyOpData->minLength = LAC_DRBG_BITS_TO_BYTES(minEntropy);
    }
    pEntropyOpData->minEntropy = minEntropy;

    pEntropyLenReturned = &pInternalState->buffers.entropyLenReturned;

    pCbData = &pInternalState->buffers.cbDataForReseed;
    pCbData->pNextOpCb = pNextOpFunc;
    pCbData->pNextOpCallbackTag = pCallbackTag;
    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;

    pCbDataForGetEntropy =
        &pInternalState->buffers.cbDataForGetEntropyInput;
    pCbDataForGetEntropy->pService = pService;
    pCbDataForGetEntropy->pInternalState = pInternalState;
    pCbDataForGetEntropy->pNextOpCallbackTag = pCbData;

    pCbDataForGetEntropy->cbOpData
        .asyncGetEntropyInputCbData.pAdditionalInput = pAdditionalInput;

    /* Call 'Get entropy input' function - this call will is always
     * asynchronous here */
    status = pInternalState->pGetEntropyInputFunc(LacDrbg_GetEntropyInputCb,
                                pCbDataForGetEntropy,
                                pEntropyOpData,
                                pEntropyBuffer,
                                pEntropyLenReturned);

    if( CPA_STATUS_SUCCESS != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        if(LAC_DRBG_OPERATION_RESEED == pInternalState->operationType)
        {
            LAC_DRBG_STAT_INC(pInternalState, numReseedRequestErrors, pService);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }
    }
    else if(LAC_DRBG_OPERATION_RESEED == operationType)
    {
        /* Increase Reseed stats */
        LAC_DRBG_STAT_INC(pInternalState, numReseedRequests, pService);
    }

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_GenerateCommon(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t * pInternalState,
        Cpa32U numOfBytesRequested,
        CpaFlatBuffer *pPseudoRandomBits)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_callback_data_t *pCbData = NULL;
    CpaBufferList *pBufferList = NULL;
    Cpa32U numOfBuffersNeeded = 0;
    Cpa32U bufferCounter = 0;
    Cpa8U* pFreeBuffSpace = NULL;
    Cpa32U numOfFreeBufBytesToZero = 0;

    pFreeBuffSpace = pInternalState->buffers.buff;

    numOfBuffersNeeded =
        ( 0 == numOfBytesRequested % LAC_DRBG_OUTLEN_IN_BYTES ) ?
                LAC_DRBG_THREE_BUFFERS : LAC_DRBG_FOUR_BUFFERS;

    /* Setup buffer list
     * +-----------+---------+-------+-----------+-----------+-----------+
     * | CpaBuffer | CpaFlat | Meta  | pData for | pData for | pData for |
     * |    List   | Buffer[]| Data  |   buf0    |  mid buf  | last buf  |
     * +-----------+---------+-------+-----------+-----------+-----------+
     *
     * */
    pBufferList = (CpaBufferList *)pFreeBuffSpace;
    pFreeBuffSpace += sizeof(CpaBufferList);

    pBufferList->numBuffers = numOfBuffersNeeded;
    pBufferList->pBuffers = (CpaFlatBuffer*)pFreeBuffSpace;

    pFreeBuffSpace += (numOfBuffersNeeded * sizeof(CpaFlatBuffer));

    pBufferList->pPrivateMetaData = pFreeBuffSpace;

    pFreeBuffSpace += pInternalState->maxBufferListMetaSize;

    numOfFreeBufBytesToZero = LAC_DRBG_OUTLEN_IN_BYTES
                                + pInternalState->nImplSeedLen;
    if( LAC_DRBG_FOUR_BUFFERS == numOfBuffersNeeded )
    {
        numOfFreeBufBytesToZero += (LAC_DRBG_OUTLEN_IN_BYTES -
                (numOfBytesRequested % LAC_DRBG_OUTLEN_IN_BYTES));
    }
    LAC_OS_BZERO(pFreeBuffSpace, numOfFreeBufBytesToZero);

    /* Setup buffers */
    /* Buffer number 0 is added only to increment counter */
    pBufferList->pBuffers[bufferCounter].dataLenInBytes =
        LAC_DRBG_OUTLEN_IN_BYTES;
    pBufferList->pBuffers[bufferCounter].pData =
        pFreeBuffSpace;

    pFreeBuffSpace += LAC_DRBG_OUTLEN_IN_BYTES;
    bufferCounter++;

    /* First, user supplied buffer */
    pBufferList->pBuffers[bufferCounter].dataLenInBytes =
        numOfBytesRequested;
    pBufferList->pBuffers[bufferCounter].pData =
        pPseudoRandomBits->pData;

    LAC_OS_BZERO( pBufferList->pBuffers[bufferCounter].pData,
                pBufferList->pBuffers[bufferCounter].dataLenInBytes);
    bufferCounter++;

    /* Then, if necessary, middle buffer */
    if( LAC_DRBG_FOUR_BUFFERS == numOfBuffersNeeded )
    {
        pBufferList->pBuffers[bufferCounter].dataLenInBytes =
            LAC_DRBG_OUTLEN_IN_BYTES -
            (numOfBytesRequested % LAC_DRBG_OUTLEN_IN_BYTES);
        pBufferList->pBuffers[bufferCounter].pData =
            pFreeBuffSpace;

        pFreeBuffSpace +=
            pBufferList->pBuffers[bufferCounter].dataLenInBytes;
        bufferCounter++;
    }

    /* And the last buffer, which will be later xored with additional input
     * - it is a part of Update step*/
    pBufferList->pBuffers[bufferCounter].dataLenInBytes =
        pInternalState->nImplSeedLen;
    pBufferList->pBuffers[bufferCounter].pData =
        pFreeBuffSpace;

    pFreeBuffSpace += pInternalState->nImplSeedLen;

    /* Operating in asynchronous mode - this part will be reached
     * only if pAdditionalInput was NULL
     */
    pCbData = &pInternalState->buffers.cbDataForGenerate;
    pCbData->pNextOpCb = LacDrbg_GenerateNextOpAfterEncryptCb;
    pCbData->pNextOpCallbackTag = pCbData;

    pCbData->cbOpData.asyncGenerateCbData.pEncryptResult = pBufferList;

    /* This is reponsible for performing Steps 4-6 of
     * NIST SP 800-90, section 10.2.1.5.1 (when DF is not used)
     * or section 10.2.1.5.2 (when DF is used) */
    status = LacDrbg_Encrypt(pService,
                        pInternalState,
                        LacDrbg_ASyncCb,
                        pCbData,
                        CPA_CY_SYM_CIPHER_AES_CTR,
                        pInternalState->workingState.key,
                        pInternalState->workingState.v,
                        pBufferList,
                        pBufferList);

    return status;
}

/**
 * @ingroup icp_sal
 */
void
icp_sal_drbgGetInstance(CpaCyDrbgSessionHandle sessionHandle,
                        CpaInstanceHandle **pDrbgInstance)
{
    lac_drbg_internal_state_t *pInternalState = NULL;

    if(NULL == sessionHandle)
    {
        LAC_INVALID_PARAM_LOG("sessionHandle is NULL");
        return;
    }

    if(NULL == pDrbgInstance)
    {
        LAC_INVALID_PARAM_LOG("pDrbgInstance is NULL");
        return;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    if(NULL == pInternalState)
    {
        LAC_LOG_ERROR("Internal state pointer is NULL");
        return;
    }

    *pDrbgInstance = pInternalState->instanceHandle;

    return;
}
