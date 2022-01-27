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
 * @file lac_sym_drbg_cb.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *     Implementation of the Deterministic Random Bit Generation API - callback
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
#include "icp_buffer_desc.h"
#include "sal_service_state.h"
#include "lac_sym.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_sym_drbg.h"


/**
 * @ingroup LacSym_Drbg
 *      Generic symmetric crypto callback used when drbg session is set up
 *      to operate synchronously
 */
void LacDrbg_SyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult)
{
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    LacSync_GenVerifyWakeupSyncCaller(
            (void *)pCbData->cbOpData.syncCbData.pSyncCallbackData,
            status,
            verifyResult);
}

/**
 * @ingroup LacSym_Drbg
 *      Generic symmetric crypto callback used when drbg session is set up
 *      to operate asynchronously
 */
void LacDrbg_ASyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult)
{
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
            status);
}

/**
 * @ingroup LacSym_Drbg
 *      Symmetric crypto callback which is being used to continue Update
 *      operation after AES CTR request is finished when drbg session is set up
 *      to operate asynchronously
 */
void LacDrbg_UpdateASyncCb(void *pCallbackTag,
        CpaStatus status,
        const CpaCySymOp operationType,
        void *pOpData,
        CpaBufferList *pDstBuffer,
        CpaBoolean verifyResult)
{
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    lac_drbg_internal_state_t *pInternalState = pCbData->pInternalState;

    /* Assumption - only one buffer in a list */
    Cpa8U* pTemp = pDstBuffer->pBuffers->pData;
    Cpa8U* pData = pCbData->cbOpData.asyncUpdateCbData.pData;
    Cpa8U* pNewKey = pInternalState->workingState.key;
    Cpa8U* pNewV = pInternalState->workingState.v;
    int i = 0;

    /* Discard first OUTLEN bytes as they were added only to
     * increment counter */
    pTemp += LAC_DRBG_OUTLEN_IN_BYTES;

    /* No point in continuing operation if cipher failed */
    if( CPA_STATUS_SUCCESS == status )
    {
        /* 4. temp = temp xor provided_data */
        for(i=0; i < pInternalState->nImplSeedLen; i++)
        {
            pTemp[i] ^= pData[i];
        }

        /* 5. Key = Leftmost keylen bits of temp */
        osalMemCopy(pNewKey, pTemp, pInternalState->nImplKeyLen);

        /* 6. V = Rightmost outlen bits of temp */
        osalMemCopy(pNewV,
                pTemp + pInternalState->nImplKeyLen,
                LAC_DRBG_OUTLEN_IN_BYTES);

        /* Indicate change of working state */
        pInternalState->isWorkingStateModified = CPA_TRUE;
    }

    /* Call the next operation in the chain */
    pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                       status);

}

/**
 * @ingroup LacSym_Drbg
 *      Callback function for implementation specific 'Get Entropy Input'
 *      function called from within LacDrbg_Reseed
 */
void
LacDrbg_GetEntropyInputCb(void *pCallbackTag,
        CpaStatus opStatus,
        void *pOpdata,
        Cpa32U lenReturned,
        CpaFlatBuffer *pEntropyBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    lac_drbg_internal_state_t *pInternalState = pCbData->pInternalState;
    CpaFlatBuffer* pAdditionalInput =
        pCbData->cbOpData.asyncGetEntropyInputCbData.pAdditionalInput;

    if( CPA_STATUS_SUCCESS == opStatus )
    {
        /* Update length of entropy data in the buffer */
        if( lenReturned < pEntropyBuffer->dataLenInBytes )
        {
            pEntropyBuffer->dataLenInBytes = lenReturned;
        }
        if( CPA_FALSE == pInternalState->isDFRequired )
        {
            status = LacDrbg_ReseedNoDF(pCbData->pService,
                        pInternalState,
                        pEntropyBuffer,
                        pAdditionalInput,
                        LacDrbg_ReseedCb,
                        pCbData->pNextOpCallbackTag);
        }
        else
        {
            status = LacDrbg_ReseedDF(pCbData->pService,
                        pInternalState,
                        pEntropyBuffer,
                        pAdditionalInput,
                        LacDrbg_ReseedCb,
                        pCbData->pNextOpCallbackTag);
        }
    }

    if( CPA_STATUS_SUCCESS != opStatus
            || CPA_STATUS_SUCCESS != status )
    {
        /* Get_entropy_input function was unable to provide requested entropy*/
        if( CPA_STATUS_SUCCESS != opStatus )
        {
            /* Enter the error state */
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
            osalAtomicSet(LAC_DRBG_ERROR_STATE,
                    &((pCbData->pService)->drbgErrorState));
            status = opStatus;
        }

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        if( LAC_DRBG_OPERATION_GENERATE == pInternalState->operationType )
        {
            LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                    pCbData->pService);

            pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
            if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
            {
                osalAtomicDec(&pInternalState->numInFlightRequests);
            }

            pInternalState->pGenCb(
                    pInternalState->buffers.cbDataForGenerate
                        .cbOpData.asyncGenerateCbData.pCallbackTag,
                    status,
                    pInternalState->buffers.cbDataForGenerate
                        .cbOpData.asyncGenerateCbData.pOpData,
                    pInternalState->buffers.cbDataForGenerate
                        .cbOpData.asyncGenerateCbData.pPseudoRandomBits);
        }
        else
        {
            LAC_DRBG_STAT_INC(pInternalState, numReseedCompletedErrors,
                            pCbData->pService);
            pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
            if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
            {
                osalAtomicDec(&pInternalState->numInFlightRequests);
            }

            pInternalState->pReseedCb(
                    pInternalState->buffers.cbUserDataForReseed
                        .pNextOpCallbackTag,
                    status,
                    pInternalState->buffers.cbUserDataForReseed
                        .cbOpData.asyncReseedCbData.pOpData);
        }
        return;
    }
}
/**
 * @ingroup LacSym_Drbg
 *      Callback function used to implement synchronous mode of operation.
 *      It is used when cpaCyDrbgReseed is called and session is setup
 *      in synchronous mode.
 */
void
LacDrbg_ReseedInternalCb(void *pCallbackTag,
        CpaStatus status,
        void *pOpData)
{
    LacSync_GenWakeupSyncCaller(pCallbackTag, status);
}

/**
 * @ingroup LacSym_Drbg
 *      Callback function for LacDrbg_Reseed function when API Reseed
 *      function is called in asynchronous mode - it updates reseed statistics
 *      and calls user callback
 */
void
LacDrbg_ReseedCallUserCb(void *pCallbackTag,
                     CpaStatus opStatus)
{
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData = NULL;

    pCbData = (lac_drbg_callback_data_t *)pCallbackTag;
    pInternalState = pCbData->pInternalState;

    if( CPA_STATUS_SUCCESS != opStatus )
    {
        LAC_DRBG_STAT_INC(pInternalState, numReseedCompletedErrors,
                pCbData->pService);
        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }
    }
    else
    {
        LAC_DRBG_STAT_INC(pInternalState, numReseedCompleted,
                pCbData->pService);
    }

    if(pInternalState->pReseedCb != LacDrbg_ReseedInternalCb)
    {
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
    }

    if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
    {
        osalAtomicDec(&pInternalState->numInFlightRequests);
    }

    pInternalState->pReseedCb(pCbData->pNextOpCallbackTag,
                              opStatus,
                              pCbData->cbOpData.asyncReseedCbData.pOpData);
}

/**
 * @ingroup LacSym_Drbg
 *      Callback function to continue reseed operation after Update request
 *      is finished - pairs with LacDrbg_ReseedDF and LacDrbg_ReseedNoDF
 */
void
LacDrbg_ReseedCb(void *pCallbackTag,
                     CpaStatus opStatus)
{
    lac_drbg_callback_data_t *pCbData = NULL;
    lac_drbg_internal_state_t *pInternalState = NULL;

    pCbData = (lac_drbg_callback_data_t *)pCallbackTag;
    pInternalState = pCbData->pInternalState;

    if( CPA_STATUS_SUCCESS == opStatus )
    {
        pInternalState->workingState.reseedCounter = 1;
    }

    /* Call the next operation in the chain */
    pCbData->pNextOpCb(pCbData->pNextOpCallbackTag, opStatus);
}

/**
 * @ingroup LacSym_Drbg
 *      This function continues Generate process after Encrypt is finished
 */
void
LacDrbg_GenerateNextOpAfterEncryptCb(void *pCallbackTag,
           CpaStatus opStatus)
{
    CpaBufferList *pBufferList = NULL;
    CpaCyDrbgGenOpData *pOpData = NULL;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;

    pInternalState = pCbData->pInternalState;
    pBufferList = pCbData->cbOpData.asyncGenerateCbData.pEncryptResult;
    pOpData =
        (CpaCyDrbgGenOpData *)pCbData->cbOpData.asyncGenerateCbData.pOpData;

    /* Continue only if cipher was successful */
    if( CPA_STATUS_SUCCESS == opStatus )
    {
        /* Complete the Update step - xor encrypted data with additional
         * input data */
        Cpa8U* pTemp =
            pBufferList->pBuffers[pBufferList->numBuffers - 1].pData;
        Cpa8U* pData = pInternalState->buffers.additionalInput;
        int i = 0;

        /* 4. temp = temp xor provided_data */
        for(i=0; i < pInternalState->nImplSeedLen; i++)
        {
            pTemp[i] ^= pData[i];
        }

        /* 5. Key = Leftmost keylen bits of temp */
        osalMemCopy(pInternalState->workingState.key,
                pTemp,
                pInternalState->nImplKeyLen);

        /* 6. V = Rightmost outlen bits of temp */
        osalMemCopy(pInternalState->workingState.v,
                pTemp + (pInternalState->nImplSeedLen
                        - LAC_DRBG_OUTLEN_IN_BYTES),
                        LAC_DRBG_OUTLEN_IN_BYTES);

        pInternalState->workingState.reseedCounter += 1;

        /* Indicate change of working state */
        pInternalState->isWorkingStateModified = CPA_TRUE;

        /* Update statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenCompleted,
                pCbData->pService);
    }
    else
    {
        /* Update statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }
    }

    pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
    if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
    {
        osalAtomicDec(&pInternalState->numInFlightRequests);
    }

    /* Call the user callback function */
    pInternalState->pGenCb(
        pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
        opStatus,
        (void *)pOpData,
        pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
}

/**
 * @ingroup LacSym_Drbg
 *      This function continues generate process after Update on
 *      additional input is completed
 */
void
LacDrbg_GenerateNextOpAfterUpdateCb(void *pCallbackTag,
           CpaStatus opStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    CpaCyDrbgGenOpData *pOpData = NULL;
    CpaFlatBuffer *pPseudoRandomBits = NULL;
    Cpa32U numOfBytesRequested = 0;

    pInternalState = pCbData->pInternalState;

    /* Get the Generate input parameters */
    pOpData =
        (CpaCyDrbgGenOpData *)pCbData->cbOpData.asyncGenerateCbData.pOpData;
    numOfBytesRequested = pOpData->lengthInBytes;
    pPseudoRandomBits =
        pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits;

    if( CPA_STATUS_SUCCESS != opStatus )
    {
        /* Previous operation - Update - failed */
        LAC_LOG_ERROR("Update function failed");

        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }

        /* Calling user callback */
        pInternalState->pGenCb(
                pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
                opStatus,
                (void *)pOpData,
                pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
        return;
    }

    status = LacDrbg_GenerateCommon(pCbData->pService,
                pCbData->pInternalState,
                numOfBytesRequested,
                pPseudoRandomBits);

    if( CPA_STATUS_SUCCESS != status )
    {
        status = CPA_STATUS_FAIL;
        LAC_LOG_ERROR("Call to Encrypt failed");

        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }

        /* Calling user callback */
        pInternalState->pGenCb(
                pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
                status,
                (void *)pOpData,
                pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
    }

}


/**
 * @ingroup LacSym_Drbg
 *      This callback continues Generate process after Reseed step
 */
void
LacDrbg_GenerateNextOpAfterReseedCb(void *pCallbackTag,
        CpaStatus opStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    CpaCyDrbgGenOpData *pOpData = NULL;

    pInternalState = pCbData->pInternalState;

    pOpData = (CpaCyDrbgGenOpData *)pCbData->cbOpData.
                                        asyncGenerateCbData.pOpData;

    /* Check what was the result of previous operation */
    if( CPA_STATUS_SUCCESS != opStatus )
    {
        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }

        /* Calling user callback */
        pInternalState->pGenCb(
                pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
                opStatus,
                (void *)pOpData,
                pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
        return;
    }

    pInternalState->reseedRequired = CPA_FALSE;

    /* Call Generate function */
    if( CPA_FALSE == pInternalState->isDFRequired )
    {
        status = LacDrbg_GenerateNoDF(pCbData->pService,
                            pCbData->pInternalState,
                            pOpData->lengthInBytes,
                            NULL, /* as reseed was performed, additional input
                                    is set to NULL - NIST SP 800-90,
                                    Section 9.3.1 (Step 7.4) */
                            pCbData->cbOpData.
                                    asyncGenerateCbData.pPseudoRandomBits);
    }
    else
    {
        status = LacDrbg_GenerateDF(pCbData->pService,
                              pCbData->pInternalState,
                              pOpData->lengthInBytes,
                              NULL,/* as reseed was performed, additional input
                                    is set to NULL - NIST SP 800-90,
                                    Section 9.3.1 (Step 7.4) */
                              pCbData->cbOpData.
                                      asyncGenerateCbData.pPseudoRandomBits);
    }


    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }

        /* Calling user callback */
        pInternalState->pGenCb(
                pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
                status,
                (void *)pOpData,
                pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
    }

}

/**
 * @ingroup LacSym_Drbg
 *      This callback continues derivation function operation after BCC request
 *      is completed. Paired with LacDrbg_Bcc
 */
void
LacDrbg_BccCb(void *pCallbackTag,
        CpaStatus opStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    lac_drbg_callback_data_t *pAsyncCbData = NULL;
    Cpa8U *pK = NULL, *pIv = NULL, *pRequestedOutputData = NULL;
    CpaBufferList *pSrcBuffer = NULL;

    if( CPA_STATUS_SUCCESS != opStatus )
    {
        /* BCC operation failed, propagate error */
        pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                           opStatus);
        return;
    }

    pInternalState = pCbData->pInternalState;
    pSrcBuffer = pCbData->cbOpData.asyncBlockCipherDFCbData.pSrcBuffer;

    /* Update length of data processed */
    pCbData->cbOpData.asyncBlockCipherDFCbData.tempLen +=
        LAC_DRBG_OUTLEN_IN_BYTES;

    /* Update counter */
    pCbData->cbOpData.asyncBlockCipherDFCbData.i++;

    /* Check if we have processed enough data */
    if( pCbData->cbOpData.asyncBlockCipherDFCbData.tempLen <
            LAC_DRBG_OUTLEN_IN_BYTES + pInternalState->nImplKeyLen )
    {
        /* Update first block, put 'i' in front */
        /* pData is a bitstring (not little endian) so we need to swap */
        LAC_MEM_SHARED_WRITE_SWAP(*(Cpa32U *)pSrcBuffer->pBuffers[0].pData,
                pCbData->cbOpData.asyncBlockCipherDFCbData.i);

        /* Call the BCC again */
        status = LacDrbg_Bcc(pCbData->pService,
                    pCbData->pInternalState,
                    pCbData->cbOpData.asyncBlockCipherDFCbData.pSrcBuffer,
                    pCbData->cbOpData.asyncBlockCipherDFCbData.pTemp
                        + pCbData->cbOpData.asyncBlockCipherDFCbData.tempLen,
                    LacDrbg_BccCb,
                    pCbData);

        if( status != CPA_STATUS_SUCCESS )
        {
            /* BCC operation failed, propagate error */
            pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                               status);
        }
        return;
    }
    /* There is enough data generated */

    /* K = leftmost keylen bits of temp */
    pK = pCbData->cbOpData.asyncBlockCipherDFCbData.pTemp;

    /* Using pRequestedOutputData as an input to AES CBC encrypt operation.
     * Assumption: length of pRequestedOutputData % outlen == 0
     * - this is always true for AES-128 and AES-256 based
     * DRBG implementations */
    pRequestedOutputData =
            pCbData->cbOpData.asyncBlockCipherDFCbData.pRequestedOutputData;
    LAC_OS_BZERO(pRequestedOutputData,
            pCbData->cbOpData.asyncBlockCipherDFCbData.requestedOutputDataLen);

    /* X = next outlen bits of temp (after keylen bits) */
    pIv = pK + pInternalState->nImplKeyLen;

    /* Build the buffer - reusing input buffer as
     * it is not be used anymore */
    pSrcBuffer->numBuffers = 1;
    pSrcBuffer->pBuffers[0].pData = pRequestedOutputData;
    pSrcBuffer->pBuffers[0].dataLenInBytes =
        pCbData->cbOpData.asyncBlockCipherDFCbData.requestedOutputDataLen;


    pAsyncCbData = &pInternalState->buffers.cbDataForEncrypt;
    pAsyncCbData->pService = pCbData->pService;
    pAsyncCbData->pInternalState = pCbData->pInternalState;
    pAsyncCbData->pNextOpCallbackTag = pCbData;
    pAsyncCbData->pNextOpCb = LacDrbg_BlockCipherDFEncryptCb;


    /* Call Encrypt to generate DF output */
    status = LacDrbg_Encrypt(pCbData->pService,
                pCbData->pInternalState,
                LacDrbg_ASyncCb,
                pAsyncCbData,
                CPA_CY_SYM_CIPHER_AES_CBC,
                pK,
                pIv,
                pSrcBuffer,
                pSrcBuffer );

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Call to encrypt failed, propagate error */
        LAC_LOG_ERROR("Encrypt request failed");
        pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                           status);
        return;

    }
}

/**
 * @ingroup LacSym_Drbg
 *      This callback continues derivation function operation after the
 *      Encrypt call
 */
void
LacDrbg_BlockCipherDFEncryptCb(void *pCallbackTag,
        CpaStatus opStatus)
{
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t*)pCallbackTag;

    /* Call the next operation in the chain */
    pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                       opStatus);

}

/**
 * @ingroup LacSym_Drbg
 *      This callback continues reseed process after the call to derivation
 *      function is completed
 */
void
LacDrbg_ReseedDFCb(void *pCallbackTag,
        CpaStatus opStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    Cpa8U* pSeedMaterial = NULL;

    pInternalState = pCbData->pInternalState;
    pSeedMaterial = pInternalState->buffers.seedMaterial;

    if( CPA_STATUS_SUCCESS != opStatus )
    {
        /* DF function failed, propagate error */
        pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                           opStatus);
        return;
    }

    status = LacDrbg_Update(pCbData->pService,
                            pCbData->pInternalState,
                            pSeedMaterial,
                            pCbData->pNextOpCb,
                            pCbData->pNextOpCallbackTag);

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Call to Update failed, propagate error */
        pCbData->pNextOpCb(pCbData->pNextOpCallbackTag,
                           status);
        return;
    }
}

/**
 * @ingroup LacSym_Drbg
 *      This callback continues Generate process after Derivation Function
 */
void
LacDrbg_GenerateDFNextOpAfterDFCb(void *pCallbackTag,
           CpaStatus opStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_callback_data_t *pCbData =
        (lac_drbg_callback_data_t *)pCallbackTag;
    CpaCyDrbgGenOpData *pOpData = NULL;

    pInternalState = pCbData->pInternalState;

    pOpData =
        (CpaCyDrbgGenOpData *)pCbData->cbOpData.asyncGenerateCbData.pOpData;

    if( CPA_STATUS_SUCCESS != opStatus )
    {
        LAC_LOG_ERROR("Derivation function failed");
        status = opStatus;

    }
    else
    {
        status = LacDrbg_Update(pCbData->pService,
                            pCbData->pInternalState,
                            pInternalState->buffers.additionalInput,
                            LacDrbg_GenerateNextOpAfterUpdateCb,
                            &pInternalState->buffers.cbDataForGenerate);

        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Update function failed");
        }
    }

    if( CPA_STATUS_SUCCESS != status
            || CPA_STATUS_SUCCESS != opStatus)
    {
        LAC_DRBG_STAT_INC(pInternalState, numGenCompletedErrors,
                pCbData->pService);

        /* Invalidate session if working state has previously changed */
        if(CPA_TRUE == pInternalState->isWorkingStateModified)
        {
            LAC_LOG_ERROR("Operation failed - session no longer valid");
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
        }

        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if ( pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }

        pInternalState->pGenCb(
                pCbData->cbOpData.asyncGenerateCbData.pCallbackTag,
                status,
                (void *)pOpData,
                pCbData->cbOpData.asyncGenerateCbData.pPseudoRandomBits);
    }

}

