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
 * @file lac_sym_drbg_df.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *    Implementation of the Deterministic Random Bit Generation API - functions
 *    to use with entropy sources that do require derivation function
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

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      AES-CBC-MAC function for use in BCC
 *
 * @description
 *      This function uses symmetric hash session set up during session
 *      initialization to calculate AES-CBC-MAC hash. This function always
 *      operates in asynchronous mode.
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSymCb                 Callback function used to indicate that
 *                                    symmetric hash request is finished and
 *                                    requested data is ready
 * @param[in]  pCbData                Pointer to callback data required to
 *                                    continue processing when hash request is
 *                                    finished. Passed to pSymCb as a
 *                                    pCallbackTag
 * @param[in]  pSrcBuffer             Buffer list containing input data
 * @param[out] pDigest                Buffer where the calculated hash will be
 *                                    stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_Hash(sal_crypto_service_t* pService,
             lac_drbg_internal_state_t *pInternalState,
             CpaCySymCbFunc pSymCb,
             lac_drbg_callback_data_t *pCbData,
             CpaBufferList *pSrcBuffer,
             Cpa8U* pDigest)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_session_desc_t * pSessionDesc = NULL;
    CpaCySymOpData *pOpData = NULL;
    Cpa32U index = 0;
    Cpa32U numRetries = 0;

    pSessionDesc = (lac_session_desc_t *)*(LAC_ARCH_UINT *)
                        ((Cpa8U*)pInternalState
                                + sizeof(lac_drbg_internal_state_t)
                                + LAC_SYM_SESSION_SIZE);

    /* Set the callback function */
    pSessionDesc->pSymCb = pSymCb;

    /* prepare the opData */
    pOpData = &(pInternalState->buffers.opData);
    LAC_OS_BZERO(pOpData, sizeof(CpaCySymOpData));

    pOpData->sessionCtx = (CpaCySymSessionCtx)((Cpa8U*)pInternalState
                + sizeof(lac_drbg_internal_state_t)
                + LAC_SYM_SESSION_SIZE);

    pOpData->packetType = CPA_CY_SYM_PACKET_TYPE_FULL;

    /* Calculate the length of message to hash */
    for(index = 0; index < pSrcBuffer->numBuffers; index++ )
    {
        pOpData->messageLenToHashInBytes +=
            pSrcBuffer->pBuffers[index].dataLenInBytes;
    }

    pOpData->pDigestResult = pDigest;

    /* call perform op */
    do
    {
        status = LacAlgChain_Perform((CpaInstanceHandle)pService,
                pSessionDesc,
                (void*)pCbData, /* callback tag */
                pOpData,
                pSrcBuffer,
                pSrcBuffer,
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
LacDrbg_Bcc(sal_crypto_service_t* pService,
        lac_drbg_internal_state_t *pInternalState,
        CpaBufferList *pSrcBuffer,
        Cpa8U* pOutputBlock,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void * pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaCySymCbFunc pSymCb = NULL;
    lac_drbg_callback_data_t *pCbData = NULL;
#ifdef DRBG_POLL_AND_WAIT
    Cpa32U retries = 0;
    Cpa32U drbgPollAndWaitTime = pService->drbgPollAndWaitTime;;
#endif

    /* Callback data */
    pCbData = &pInternalState->buffers.cbDataForBCC;

    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;


    if( NULL == pNextOpFunc )
    {
        pSymCb = LacDrbg_SyncCb;
        status = LacSync_CreateSyncCookie(
                    &pCbData->cbOpData.syncCbData.pSyncCallbackData);

        if( CPA_STATUS_SUCCESS != status )
        {
            LAC_LOG_ERROR("Sync cookie allocation error");
            return status;
        }
        pCbData->pNextOpCb = NULL;

    }
    else
    {
        pSymCb = LacDrbg_ASyncCb;

        pCbData->pNextOpCb = pNextOpFunc;
        pCbData->pNextOpCallbackTag = pCallbackTag;
    }

    status = LacDrbg_Hash(pService,
                      pInternalState,
                      pSymCb,
                      pCbData,
                      pSrcBuffer,
                      pOutputBlock);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Hash request failed");
        if( NULL == pNextOpFunc )
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

    /* If operating in sync mode, wait for callback
     * and destroy the sync cookie */
    if( NULL == pNextOpFunc )
    {
        CpaStatus syncStatus = CPA_STATUS_SUCCESS;
        CpaBoolean opResult = CPA_FALSE;

#ifdef DRBG_POLL_AND_WAIT
        retries=0;
        do
        {
             /* Poll the low priority response ring */
             syncStatus = icp_adf_pollInstance(
                              &(pService->trans_handle_sym_rx_lo), 1, 0);
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
                LAC_SYM_SYNC_CALLBACK_TIMEOUT,
                &status,
                &opResult);
#endif
        /* If callback doesn't come back */
        if (CPA_STATUS_SUCCESS != syncStatus)
        {
            LAC_LOG_ERROR("Callback timed out");
            status = syncStatus;
        }
        /* Destroy sync cookie */
        LacSync_DestroySyncCookie(
                &pCbData->cbOpData.syncCbData.pSyncCallbackData);

    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_Drbg
 *      DRBG Derivation Function
 *
 * @description
 *      This function implements the CTR_DRBG Derivation Function as described
 *      in NIST SP 800-90, section 10.4.2
 *
 * @param[in]  pService               Pointer to a crypto service structure
 * @param[in]  pInternalState         Pointer to a DRBG internal state
 * @param[in]  pSrcBuffer             Buffer list containing input data
 * @param[in]  pRequestedOutputData   Buffer where the output data will be
 *                                    stored
 * @param[in]  requestedOutputDataLen Amount of output data requested,
 *                                    in bytes. This is supposed to be equal
 *                                    to the length of seed for the
 *                                    implementation.
 * @param[in]  pNextOpFunc            Function that needs to be executed after
 *                                    Derivation Function process is completed
 *                                    when operating in asynchronous operation;
 *                                    NULL when operating synchronously
 * @param[in]  pCallbackTag           Pointer to opaque data that is passed to
 *                                    pNextOpFunc when working in asynchronous
 *                                    mode
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_BlockCipherDF(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t *pInternalState,
        CpaBufferList *pSrcBuffer,
        Cpa8U* pRequestedOutputData,
        Cpa32U requestedOutputDataLen,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void * pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus syncStatus = CPA_STATUS_SUCCESS;
    CpaBoolean opResult = CPA_FALSE;

    Cpa32U L = 0, N = 0, i = 0, index = 0;
    Cpa32U *pN = NULL, *pL = NULL;
    Cpa32U prefixLen = 0, suffixLen = 0;
    Cpa8U *pFreeBuff = NULL;
    Cpa8U *pTemp = NULL;
    Cpa8U *pK = NULL, *pIv = NULL;
    Cpa32U tempLen = 0;
    lac_drbg_callback_data_t *pCbData = NULL;
    LacDrbgNextOpCbFunc pNextFunc = NULL;
#ifdef DRBG_POLL_AND_WAIT
    Cpa32U retries = 0;
    Cpa32U drbgPollAndWaitTime = pService->drbgPollAndWaitTime;;
#endif

    for(index = 0; index < pSrcBuffer->numBuffers; index++ )
    {
        L += pSrcBuffer->pBuffers[index].dataLenInBytes;
    }

    N = requestedOutputDataLen;

    /* prefix buffer will contain L, N and IV */
    prefixLen = LAC_DRBG_OUTLEN_IN_BYTES + sizeof(L) + sizeof(N);
    /* Suffix will contain at least one byte - 0x80 */
    suffixLen = 1;

    /* But it might be longer if the total size is not a multiply of outlen */
    if( (prefixLen + L + suffixLen) % LAC_DRBG_OUTLEN_IN_BYTES != 0 )
    {
        suffixLen += LAC_DRBG_OUTLEN_IN_BYTES
            - ((prefixLen + L + suffixLen) % LAC_DRBG_OUTLEN_IN_BYTES);
    }

    /* Using pUserData field in pSrcBuffer to track free buff space */
    pFreeBuff = (Cpa8U*)pSrcBuffer->pUserData;

    /* Set prefix buffer */
    pSrcBuffer->pBuffers[0].dataLenInBytes = prefixLen;
    pSrcBuffer->pBuffers[0].pData = pFreeBuff;
    pFreeBuff += prefixLen;

    LAC_OS_BZERO(pSrcBuffer->pBuffers[0].pData, prefixLen);

    /* L and N are bitstrings (not little endian) so we need to swap */
    pL = (Cpa32U*)(pSrcBuffer->pBuffers[0].pData + LAC_DRBG_OUTLEN_IN_BYTES);
    LAC_MEM_SHARED_WRITE_SWAP(*pL, L);

    pN = pL + 1;
    LAC_MEM_SHARED_WRITE_SWAP(*pN, N);

    /* Insert suffix buffer */
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].dataLenInBytes = suffixLen;
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData = pFreeBuff;

    /* Zero the buffer BEFORE incrementing numBuffers */
    LAC_OS_BZERO(pFreeBuff, suffixLen);

    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData[0] =
        LAC_DRBG_DF_DATA_DELIMITER;

    pSrcBuffer->numBuffers++;

    pFreeBuff += suffixLen;

    pSrcBuffer->pPrivateMetaData = pFreeBuff;
    pFreeBuff += pInternalState->maxBufferListMetaSize;

    /* Store the pointer to the free buffer space in pUserData field */
    pSrcBuffer->pUserData = pFreeBuff;

    /* Align pTemp as IV for encrypt operation will be derived from it */
    pTemp = (Cpa8U*)LAC_ALIGN_POW2_ROUNDUP((LAC_ARCH_UINT)pFreeBuff,
            LAC_8BYTE_ALIGNMENT);

    if( NULL == pNextOpFunc )
    {
        /* Operating synchronously */
        pCbData = NULL;
        pNextFunc = NULL;
    }
    else
    {
        /* Operating asynchronously */
        pCbData = &pInternalState->buffers.cbDataForDF;
        pCbData->pService = pService;
        pCbData->pInternalState = pInternalState;
        pCbData->pNextOpCallbackTag = pCallbackTag;
        pCbData->pNextOpCb = pNextOpFunc;

        pCbData->cbOpData.asyncBlockCipherDFCbData.i = i;
        pCbData->cbOpData.asyncBlockCipherDFCbData.pTemp = pTemp;
        pCbData->cbOpData.asyncBlockCipherDFCbData.tempLen = tempLen;
        pCbData->cbOpData.asyncBlockCipherDFCbData.pSrcBuffer = pSrcBuffer;
        pCbData->cbOpData.asyncBlockCipherDFCbData.pRequestedOutputData =
            pRequestedOutputData;
        pCbData->cbOpData.asyncBlockCipherDFCbData.requestedOutputDataLen =
            requestedOutputDataLen;

        pNextFunc = LacDrbg_BccCb;
    }

    /* Call BCC function until desired length is achieved
     * NIST SP 800-90, Section 10.4.2 (step 9) */
    while( tempLen < LAC_DRBG_OUTLEN_IN_BYTES + pInternalState->nImplKeyLen )
    {
        /* pData is a bitstring (not little endian) so we need to swap */
        LAC_MEM_SHARED_WRITE_SWAP(*(Cpa32U *)pSrcBuffer->pBuffers[0].pData, i);

        if( NULL != pNextOpFunc )
        {
            pCbData->cbOpData.asyncBlockCipherDFCbData.i = i;
            pCbData->cbOpData.asyncBlockCipherDFCbData.tempLen = tempLen;
        }

        status = LacDrbg_Bcc(pService,
                             pInternalState,
                             pSrcBuffer,
                             pTemp + tempLen,
                             pNextFunc,
                             pCbData);

        if( CPA_STATUS_SUCCESS != status )
        {
            LAC_LOG_ERROR("Error calling BCC function");
            return status;
        }

        /* return if operating asynchronously */
        if( NULL != pNextOpFunc )
        {
            return status;
        }

        /* for synchronous operation increment tempLen and i; for asynchronous
         * this is done in a callback */
        tempLen += LAC_DRBG_OUTLEN_IN_BYTES;
        i++;
    }

    /* If we are here it means synchronous mode of operation */
    pK = pTemp;

    /* X = next outlen bits of temp (after keylen bits);
     * in this case X is used as IV for AES CBC encrypt operation */
    pIv = pK + pInternalState->nImplKeyLen;

    /* Using pRequestedOutputData as an input to AES CBC encrypt operation.
     * Assumption: length of pRequestedOutputData % outlen == 0
     * - this is always true for AES-128 and AES-256 based
     * DRBG implementations */
    LAC_OS_BZERO(pRequestedOutputData, requestedOutputDataLen);

    pCbData = &pInternalState->buffers.cbDataForDF;
    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;

    pCbData->pNextOpCb = NULL;

    /* Build the buffer - reusing input buffer as
     * it will not be used anymore */
    pSrcBuffer->numBuffers = 1;
    pSrcBuffer->pBuffers[0].pData = pRequestedOutputData;
    pSrcBuffer->pBuffers[0].dataLenInBytes = requestedOutputDataLen;

    /* Populate requested bits */
    status = LacSync_CreateSyncCookie(
            &pCbData->cbOpData.syncCbData.pSyncCallbackData);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Sync cookie allocation error");
        return status;
    }

    status = LacDrbg_Encrypt(pService,
                pInternalState,
                LacDrbg_SyncCb,
                pCbData,
                CPA_CY_SYM_CIPHER_AES_CBC,
                pK,
                pIv,
                pSrcBuffer,
                pSrcBuffer );

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Encrypt request failed");
        /* As the Request was not sent the Callback will never
         * be called, so need to indicate that we're finished
         * with cookie so it can be destroyed. */
        LacSync_SetSyncCookieComplete(
                pCbData->cbOpData.syncCbData.pSyncCallbackData);
        /* Destroy sync cookie */
        LacSync_DestroySyncCookie(
                &pCbData->cbOpData.syncCbData.pSyncCallbackData);
        return status;
    }

#ifdef DRBG_POLL_AND_WAIT
    retries=0;
    do
    {
         /* Poll the low priority response ring */
         syncStatus = icp_adf_pollInstance(
                          &(pService->trans_handle_sym_rx_lo), 1, 0);
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
                    LAC_SYM_SYNC_CALLBACK_TIMEOUT,
                    &status,
                    &opResult);
#endif

    /* If callback doesn't come back */
    if ( syncStatus != CPA_STATUS_SUCCESS )
    {
        LAC_LOG_ERROR("Callback timed out");
        status = syncStatus;
    }

    /* Destroy sync cookie */
    LacSync_DestroySyncCookie(
            &pCbData->cbOpData.syncCbData.pSyncCallbackData);

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_InstantiateDF(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t *pInternalState,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaFlatBuffer *pEntropyInputBuffer,
        CpaFlatBuffer *pNonceBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pSeedMaterial = NULL, *pFreeBuf = NULL;
    CpaBufferList *pSrcBuffer = NULL;

    /* NIST SP 800-90 CTR_DRBG instantiate algorithm when
     *     Derivation Function is used
     *     1. seed_material = entropy_input || nonce || personalization_string
     *     2. seed_material = Block_Cipher_df(seed_material, seedlen)
     *     3. Key = 0^keylen
     *     4. V = 0^outlen
     *     5. (Key,V) = Update(seed_material, Key, V)
     *     6. reseed_counter = 1
     *     7. Return V, Key and reseed_counter as the initial_working_state
     */

    /* Build the input buffer list */
    pFreeBuf = pInternalState->buffers.buff;

    pSrcBuffer = (CpaBufferList *)pFreeBuf;
    pFreeBuf += sizeof(CpaBufferList);

    /* Setup following buffers (if used): entropy input, nonce and
     * personalization string */
    pSrcBuffer->pBuffers = (CpaFlatBuffer *)pFreeBuf;

    /* Leaving space for 2 more buffers (so 5 in total)
     * as they will be set in DF*/
    pFreeBuf += LAC_DRBG_FIVE_BUFFERS * sizeof(CpaFlatBuffer);

    pSrcBuffer->numBuffers = 0;

    /* To avoid moving data in DF and as per comment above, leave the first
     * buffer empty */
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].dataLenInBytes = 0;
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData = NULL;
    pSrcBuffer->numBuffers++;


    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].dataLenInBytes =
        pEntropyInputBuffer->dataLenInBytes;
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData =
        pEntropyInputBuffer->pData;
    pSrcBuffer->numBuffers++;


    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].dataLenInBytes =
        pNonceBuffer->dataLenInBytes;
    pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData = pNonceBuffer->pData;
    pSrcBuffer->numBuffers++;


    if( NULL != pSetupData->personalizationString.pData )
    {
        pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].dataLenInBytes =
                            pSetupData->personalizationString.dataLenInBytes;
        pSrcBuffer->pBuffers[pSrcBuffer->numBuffers].pData =
            pSetupData->personalizationString.pData;
        pSrcBuffer->numBuffers++;
    }

    /* Store pFreeBuf at pUserData - it will be used in DF to determine free
     * space for further allocations   */
    pSrcBuffer->pUserData = pFreeBuf;

    pSeedMaterial = pInternalState->buffers.seedMaterial;

    status = LacDrbg_BlockCipherDF(pService,
                                  pInternalState,
                                  pSrcBuffer,
                                  pSeedMaterial,
                                  pInternalState->nImplSeedLen,
                                  NULL,
                                  NULL);

    if( CPA_STATUS_SUCCESS == status )
    {
        LAC_OS_BZERO(pInternalState->workingState.v,
                    LAC_DRBG_OUTLEN_IN_BYTES);
        LAC_OS_BZERO(pInternalState->workingState.key,
                pInternalState->nImplKeyLen);

        status = LacDrbg_Update(pService,
                                pInternalState,
                                pSeedMaterial,
                                NULL,
                                NULL);


        if( CPA_STATUS_SUCCESS == status)
        {
            /* Set reseedCounter to 1 as per NIST SP 800-90,
             * section 10.2.1.3.2 (Step 6) */
            pInternalState->workingState.reseedCounter = 1;
        }
    }

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_ReseedDF(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t *pInternalState,
        CpaFlatBuffer *pEntropyInput,
        CpaFlatBuffer *pAdditionalInput,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void *pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U* pFreeBuf = NULL;
    LacDrbgNextOpCbFunc pNextOp;
    lac_drbg_callback_data_t *pCbData;
    CpaBufferList *pSrcBuffer = NULL;
    Cpa8U* pSeedMaterial = NULL;
    Cpa32U counter = 0;

    /* Build the input buffer */
    pFreeBuf = pInternalState->buffers.buff;

    pSrcBuffer = (CpaBufferList *)pFreeBuf;
    pFreeBuf += sizeof(CpaBufferList);

    /* Setup flat buffers: entropy input and additional input (if provided) */
    pSrcBuffer->pBuffers = (CpaFlatBuffer *)pFreeBuf;

    /* To avoid moving data in DF and as per comment above leave the first
     * buffer empty */
    pSrcBuffer->pBuffers[counter].dataLenInBytes = 0;
    pSrcBuffer->pBuffers[counter].pData = NULL;
    counter++;

    pSrcBuffer->pBuffers[counter].dataLenInBytes =
        pEntropyInput->dataLenInBytes;
    pSrcBuffer->pBuffers[counter].pData = pEntropyInput->pData;
    counter++;

    if( NULL != pAdditionalInput->pData )
    {
        pSrcBuffer->pBuffers[counter].dataLenInBytes =
            pAdditionalInput->dataLenInBytes;
        pSrcBuffer->pBuffers[counter].pData =
            pAdditionalInput->pData;
        counter++;
    }

    pSrcBuffer->numBuffers = counter;

    /* Leaving space for 2 more buffers as they will be set in DF*/
    pFreeBuf += (counter + LAC_DRBG_TWO_BUFFERS) * sizeof(CpaFlatBuffer);

    /* Store pFreeBuf at pUserData - it will be used in DF to determine free
     * space for further allocations   */
    pSrcBuffer->pUserData = pFreeBuf;

    pSeedMaterial = pInternalState->buffers.seedMaterial;

    pCbData = &pInternalState->buffers.cbDataForReseedDF;
    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;
    pCbData->pNextOpCallbackTag = pCallbackTag;
    pCbData->pNextOpCb = pNextOpFunc;

    pNextOp = LacDrbg_ReseedDFCb;

    status = LacDrbg_BlockCipherDF(pService,
                                    pInternalState,
                                    pSrcBuffer,
                                    pSeedMaterial,
                                    pInternalState->nImplSeedLen,
                                    pNextOp,
                                    pCbData);

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_GenerateDF(sal_crypto_service_t *pService,
                    lac_drbg_internal_state_t *pInternalState,
                    Cpa32U numOfBytesRequested,
                    const CpaFlatBuffer *pAdditionalInput,
                    CpaFlatBuffer *pPseudoRandomBits)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U* pFreeBuf = NULL;
    CpaBufferList *pSrcBuffer = NULL;

    /* 1 */
    if(pInternalState->workingState.reseedCounter > LAC_DRBG_SEED_LIFE )
    {
        pInternalState->reseedRequired = CPA_TRUE;
        return CPA_STATUS_RETRY;
    }

    /* 2 */
    if( NULL != pAdditionalInput )
    {
        /* Build the input buffer */
        pFreeBuf = pInternalState->buffers.buff;

        pSrcBuffer = (CpaBufferList *)pFreeBuf;
        pFreeBuf += sizeof(CpaBufferList);

        /* Setup 1 flat buffer: additional input */
        pSrcBuffer->pBuffers = (CpaFlatBuffer *)pFreeBuf;

        /* Leaving space for 2 more buffers as they will be set in DF */
        pFreeBuf += LAC_DRBG_THREE_BUFFERS * sizeof(CpaFlatBuffer);

        /* To avoid moving data in DF leave the first buffer empty */
        pSrcBuffer->pBuffers[0].dataLenInBytes = 0;
        pSrcBuffer->pBuffers[0].pData = NULL;

        pSrcBuffer->pBuffers[1].dataLenInBytes =
            pAdditionalInput->dataLenInBytes;
        pSrcBuffer->pBuffers[1].pData =
            pAdditionalInput->pData;

        pSrcBuffer->numBuffers = LAC_DRBG_TWO_BUFFERS;

        /* Store pFreeBuf at pUserData - it will be used in DF
         * to determine free space for further allocations   */
        pSrcBuffer->pUserData = pFreeBuf;

        /* This function always operate in asynchronous mode, so call
         * derivation function and return - processing will continue
         * in the callbacks */
        status = LacDrbg_BlockCipherDF(pService,
                               pInternalState,
                               pSrcBuffer,
                               pInternalState->buffers.additionalInput,
                               pInternalState->nImplSeedLen,
                               LacDrbg_GenerateDFNextOpAfterDFCb,
                               &pInternalState->buffers.cbDataForGenerate);
        return status;

    }
    else
    {
        LAC_OS_BZERO(pInternalState->buffers.additionalInput,
                pInternalState->nImplSeedLen);
    }

    return LacDrbg_GenerateCommon(pService,
            pInternalState,
            numOfBytesRequested,
            pPseudoRandomBits);
}

