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
 * @file lac_sym_drbg_api.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *     Implementation of the Deterministic Random Bit Generation API
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
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_debug.h"
#include "icp_buffer_desc.h"
#include "sal_service_state.h"
#include "lac_common.h"
#include "lac_sym.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_sym_drbg.h"
#include "lac_sym_drbg_ht.h"
#include "lac_sym_drbg_rdrand.h"
#include "lac_sal_types.h"
#ifdef DRBG_POLL_AND_WAIT
#include "icp_adf_poll.h"
#endif


/* Number of DRBG statistics */
#define LAC_DRBG_NUM_STATS (sizeof(CpaCyDrbgStats64) / sizeof(Cpa64U))

/* Implementation specific get entropy function pointer */
extern IcpSalDrbgGetEntropyInputFunc pGetEntropyInputFunc;

/* Implementation specific get nonce function pointer */
extern IcpSalDrbgGetNonceFunc pGetNonceFunc;

/* Implementation specific derivation function pointer */
extern IcpSalDrbgIsDFReqFunc pIsDFReqFunc;

/* Test specific get entropy function pointer */
extern IcpSalDrbgGetEntropyInputFunc pTestGetEntropyInputFunc;

/* Test specific get nonce function pointer */
extern IcpSalDrbgGetNonceFunc pTestGetNonceFunc;

/* Test specific derivation function pointer */
extern IcpSalDrbgIsDFReqFunc pTestIsDFReqFunc;


/**
 * @ingroup LacSym_Drbg
 *
 * @Description     Function to check if the device supports
 *                  random number capability
 */
CpaBoolean
LacDrbg_CheckRandomCapability(const CpaInstanceHandle instanceHandle)
{
    sal_service_t* pGenericService = NULL;
    pGenericService = (sal_service_t*)instanceHandle;

    if(pGenericService->capabilitiesMask &
           ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)
    {
        return CPA_TRUE;
    }
    return CPA_FALSE;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgSessionGetSize(const CpaInstanceHandle instanceHandle_in,
        const CpaCyDrbgSessionSetupData *pSetupData,
        Cpa32U *pSize)
{
    CpaInstanceHandle instanceHandle;
#ifdef ICP_TRACE
    CpaStatus status = CPA_STATUS_SUCCESS;
#endif

    if(instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
#ifdef ICP_TRACE
        status = LacDrbg_RdrandSessionGetSize(instanceHandle,
                    pSetupData,
                    pSize);
        if (NULL != pSize)
        {
            LAC_LOG4("Called with params (0x%lx, 0x%lx, 0x%lx[%d])\n",
                    (LAC_ARCH_UINT)instanceHandle_in,
                    (LAC_ARCH_UINT)pSetupData,
                    (LAC_ARCH_UINT)pSize,
                    *pSize);
        }
        else
        {
            LAC_LOG3("Called with params (0x%lx, 0x%lx, 0x%lx)\n",
                    (LAC_ARCH_UINT)instanceHandle_in,
                    (LAC_ARCH_UINT)pSetupData,
                    (LAC_ARCH_UINT)pSize);
        }
        return status;
#else
        return LacDrbg_RdrandSessionGetSize(instanceHandle,
                   pSetupData,
                   pSize);
#endif
    }

    LAC_CHECK_NULL_PARAM(pSetupData);
    LAC_CHECK_NULL_PARAM(pSize);

    if( NULL == pIsDFReqFunc )
    {
        LAC_LOG_ERROR("'Is DF Required' Function not registered");
        return CPA_STATUS_FAIL;
    }

    *pSize = sizeof(lac_drbg_internal_state_t) + LAC_SYM_SESSION_SIZE;

    /* In case the entropy source requires derivation function two
     * symmetric sessions need to be initialized - one for cipher
     * and one for hash operations (BCC function). Additional space for
     * hash session is then required  */
    if( CPA_TRUE == pIsDFReqFunc() )
    {
        *pSize += LAC_SYM_SESSION_SIZE;
    }

    /* Additional memory required for instantiate health test buffer */
    *pSize += LAC_DRBG_MAX_SEEDLEN_IN_BYTES;

    /* Additional memory required to store health test vector pointer */
    *pSize += sizeof(void*);

    /* Extra memory is needed to internally re-align the data. The pointer
     * to the algined data is stored at the start of the user allocated
     * memory hence the extra space for an LAC_ARCH_UINT */
    *pSize += LAC_8BYTE_ALIGNMENT + sizeof(LAC_ARCH_UINT);

#ifdef ICP_TRACE
    LAC_LOG4("Called with params (0x%lx, 0x%lx, 0x%lx[%d])\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pSetupData,
            (LAC_ARCH_UINT)pSize,
            *pSize);
#endif
    return CPA_STATUS_SUCCESS;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgInitSession(const CpaInstanceHandle instanceHandle_in,
        const CpaCyGenFlatBufCbFunc pGenCb,
        const CpaCyGenericCbFunc pReseedCb,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa32U* pSeedLen)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;

    if(instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        status = LacDrbg_RdrandInitSession(instanceHandle,
                     pGenCb,
                     pReseedCb,
                     pSetupData,
                     sessionHandle,
                     pSeedLen);
#ifdef ICP_TRACE
        if (NULL != pSeedLen)
        {
            LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                    "0x%lx, 0x%lx[%d])\n",
                    (LAC_ARCH_UINT)instanceHandle_in,
                    (LAC_ARCH_UINT)pGenCb,
                    (LAC_ARCH_UINT)pReseedCb,
                    (LAC_ARCH_UINT)pSetupData,
                    (LAC_ARCH_UINT)sessionHandle,
                    (LAC_ARCH_UINT)pSeedLen,
                    *pSeedLen);
        }
        else
        {
            LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                        "0x%lx, 0x%lx)\n",
                    (LAC_ARCH_UINT)instanceHandle_in,
                    (LAC_ARCH_UINT)pGenCb,
                    (LAC_ARCH_UINT)pReseedCb,
                    (LAC_ARCH_UINT)pSetupData,
                    (LAC_ARCH_UINT)sessionHandle,
                    (LAC_ARCH_UINT)pSeedLen);
        }
#endif
        return status;
    }

    status = LacDrbg_HTInitSession(instanceHandle_in,
                pSetupData,
                sessionHandle);

    if( CPA_STATUS_SUCCESS == status )
    {
        status = LacDrbg_InitSession(instanceHandle_in,
                    pGenCb,
                    pReseedCb,
                    pSetupData,
                    sessionHandle,
                    pSeedLen,
                    CPA_FALSE);
    }
    else if(CPA_STATUS_FAIL == status)
    {
        LAC_LOG_ERROR("Health test failed");
    }
#ifdef ICP_TRACE
    if (NULL != pSeedLen)
    {
        LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, "
                "0x%lx, 0x%lx, 0x%lx[%d])\n",
                (LAC_ARCH_UINT)instanceHandle_in,
                (LAC_ARCH_UINT)pGenCb,
                (LAC_ARCH_UINT)pReseedCb,
                (LAC_ARCH_UINT)pSetupData,
                (LAC_ARCH_UINT)sessionHandle,
                (LAC_ARCH_UINT)pSeedLen,
                *pSeedLen);
    }
    else
    {
        LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx, "
                "0x%lx, 0x%lx, 0x%lx)\n",
                (LAC_ARCH_UINT)instanceHandle_in,
                (LAC_ARCH_UINT)pGenCb,
                (LAC_ARCH_UINT)pReseedCb,
                (LAC_ARCH_UINT)pSetupData,
                (LAC_ARCH_UINT)sessionHandle,
                (LAC_ARCH_UINT)pSeedLen);
    }
#endif
    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_InitSession(const CpaInstanceHandle instanceHandle_in,
        const CpaCyGenFlatBufCbFunc pGenCb,
        const CpaCyGenericCbFunc pReseedCb,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa32U* pSeedLen,
        CpaBoolean testRun)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
    sal_crypto_service_t* pService = NULL;
    icp_sal_drbg_get_entropy_op_data_t *pEntropyOpData = NULL;
    Cpa32U secStren = 0;
    Cpa32U *pEntropyLenReturned = NULL;
    CpaInstanceHandle instanceHandle = NULL;
    CpaFlatBuffer * pEntropyBuffer = NULL;
    CpaFlatBuffer * pNonceBuffer = NULL;
    CpaPhysicalAddr physAddress = 0;
    CpaPhysicalAddr physAddressAligned = 0;

    CpaCySymSessionCtx pSessionCtx = NULL;

    if(instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);
    LAC_DRBG_CHECK_NULL_PARAM(pSetupData);
    LAC_DRBG_CHECK_NULL_PARAM(sessionHandle);
    LAC_DRBG_CHECK_NULL_PARAM(pSeedLen);
    if( 0 != pSetupData->personalizationString.dataLenInBytes
        && NULL == pSetupData->personalizationString.pData)
    {
        status = CPA_STATUS_INVALID_PARAM;
    }
    if(CPA_FALSE == testRun && CPA_STATUS_SUCCESS == status)
    {
        if( NULL == pIsDFReqFunc )
        {
            LAC_LOG_ERROR("'Is DF Required' Function not registered");
            status = CPA_STATUS_FAIL;
        }

        if( NULL == pGetEntropyInputFunc && CPA_STATUS_SUCCESS == status )
        {
            LAC_LOG_ERROR("'Get entropy' function not registered");
            status = CPA_STATUS_FAIL;
        }

        if( NULL == pGetNonceFunc && CPA_STATUS_SUCCESS == status )
        {
            LAC_LOG_ERROR("'Get nonce' function not registered");
            status = CPA_STATUS_FAIL;
        }
    }

    pService = (sal_crypto_service_t*) instanceHandle;

    if(CPA_STATUS_SUCCESS == status &&
            0 != osalAtomicGet(&pService->drbgErrorState))
    {
        LAC_LOG_ERROR("DRBG in error state");
        status = CPA_STATUS_FAIL;
    }

    /* Re-align the session structure to 8 byte alignment */
    physAddress = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                      (*pService), (Cpa8U *)sessionHandle + sizeof(void *));

    if (physAddress == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the session\n");
        status = CPA_STATUS_FAIL;
    }

    if( CPA_STATUS_SUCCESS == status )
    {
        physAddressAligned = LAC_ALIGN_POW2_ROUNDUP(
                                physAddress, LAC_8BYTE_ALIGNMENT);

        pInternalState = (lac_drbg_internal_state_t *)
                        /* Move the session pointer by the physical offset
                        between aligned and unaligned memory */
                        ((Cpa8U *) sessionHandle + sizeof(void *)
                        + (physAddressAligned-physAddress));

        /* Init the spinlock used to lock the access to numInFlightRequests */
        status = LAC_SPINLOCK_INIT(&pInternalState->sessionLock);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Spinlock init failed for sessionLock");
        }
    }

    if( CPA_STATUS_SUCCESS == status )
    {

        /* save the aligned pointer in the first bytes (size of unsigned long)
        * of the session memory */
        *((LAC_ARCH_UINT *)sessionHandle) = (LAC_ARCH_UINT)pInternalState;

        pInternalState->isTestRun = testRun;

        /* Store the instanceHandle in the internal state */
        pInternalState->instanceHandle = instanceHandle;

        /* Set the number of in flight requests to 0 */
        osalAtomicSet(0, &pInternalState->numInFlightRequests);

        /* Set the operation type to NONE */
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;

        /* Clear the error state flag */
        osalAtomicSet(0, &pInternalState->errorState);

        if( testRun == CPA_TRUE )
        {
            pInternalState->isDFRequired = pTestIsDFReqFunc();
            pInternalState->pGetEntropyInputFunc = pTestGetEntropyInputFunc;
            pInternalState->pGetNonceFunc = pTestGetNonceFunc;
        }
        else
        {
            pInternalState->isDFRequired = pIsDFReqFunc();
            pInternalState->pGetEntropyInputFunc = pGetEntropyInputFunc;
            pInternalState->pGetNonceFunc = pGetNonceFunc;
        }

        /* Check requested security strength */
        if( pSetupData->secStrength < CPA_CY_RBG_SEC_STRENGTH_112
            || pSetupData->secStrength > CPA_CY_RBG_SEC_STRENGTH_256
            || ( pSetupData->secStrength > CPA_CY_RBG_SEC_STRENGTH_128
                    && CPA_TRUE == pInternalState->isDFRequired))
        {

            if (CPA_CY_RBG_SEC_STRENGTH_192 == pSetupData->secStrength)
            {
                LAC_LOG_ERROR("Security strength 192 with prediction "
                    "resistance not supported\n");
            }
            if (CPA_CY_RBG_SEC_STRENGTH_256 == pSetupData->secStrength)
            {
                LAC_LOG_ERROR("Security strength 256 with prediction "
                    "resistance not supported\n");
            }
            status = CPA_STATUS_INVALID_PARAM;
        }
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        /* Set key length & seed length for the session */
        if( CPA_TRUE == pInternalState->isDFRequired )
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN128_IN_BYTES;
        }
        else
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN256_IN_BYTES;
        }

        pInternalState->nImplSeedLen = pInternalState->nImplKeyLen
                                            + LAC_DRBG_OUTLEN_IN_BYTES;

        if( CPA_FALSE == pInternalState->isDFRequired )
        {
            /* Check personalization string length */
            if( pSetupData->personalizationString.dataLenInBytes
                    > pInternalState->nImplSeedLen )
            {
                status = CPA_STATUS_INVALID_PARAM;
            }

        }
    }

    /***************** SYM SESSION SETUP **********************/
    if(CPA_STATUS_SUCCESS == status)
    {
        pSessionCtx = (CpaCySymSessionCtx)((Cpa8U*)pInternalState
                                    + sizeof(lac_drbg_internal_state_t));

        status = LacDrbg_InitSymSession(instanceHandle,
                                        pInternalState,
                                        pSessionCtx,
                                        CPA_TRUE);
        if( status != CPA_STATUS_SUCCESS)
        {
            LAC_LOG_ERROR("Cipher session init failed");
            status = CPA_STATUS_FAIL;
        }
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        pInternalState->symSessionSize = LAC_SYM_SESSION_SIZE;

        /* If entropy source requires derivation function, hash
         * session will be needed for BCC function. */
        if( CPA_TRUE == pInternalState->isDFRequired )
        {
            pSessionCtx = (CpaCySymSessionCtx)
                            ((Cpa8U*)pSessionCtx + LAC_SYM_SESSION_SIZE);

            status = LacDrbg_InitSymSession(instanceHandle,
                        pInternalState,
                        pSessionCtx,
                        CPA_FALSE);
            if( status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("Hash session init failed");
                status = CPA_STATUS_FAIL;
            }
            else
            {
                pInternalState->symSessionSize += LAC_SYM_SESSION_SIZE;
            }
        }
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        /* Additional memory required for instantiate health test buffer */
        pInternalState->symSessionSize += LAC_DRBG_MAX_SEEDLEN_IN_BYTES;

        /* Additional memory required for healt test vector pointer */
        pInternalState->symSessionSize += sizeof(void*);

        /**************** SYM SESSION SETUP END *******************/

        status = cpaCyBufferListGetMetaSize(instanceHandle,
                    LAC_DRBG_MAX_BUFFER_NO,
                    &pInternalState->maxBufferListMetaSize);
        if(CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Error while getting meta size");
        }
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        /* Obtain entropy input */
        LAC_RBG_SEC_STRENGTH_TO_NUM(pSetupData->secStrength, secStren);

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
            pEntropyOpData->minLength = LAC_DRBG_BITS_TO_BYTES(secStren);
        }
        pEntropyOpData->minEntropy = secStren;

        /* Get the pointer passed to 'Get Entropy Input' function that
         * will store returned entropy length */
        pEntropyLenReturned = &pInternalState->buffers.entropyLenReturned;

        status = pInternalState->pGetEntropyInputFunc(NULL,
                        NULL,
                        pEntropyOpData,
                        pEntropyBuffer,
                        pEntropyLenReturned);


        if( CPA_STATUS_SUCCESS != status )
        {
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pInternalState->errorState);
            osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);

            status = CPA_STATUS_FAIL;
        }
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        if( *pEntropyLenReturned < pEntropyBuffer->dataLenInBytes )
        {
            pEntropyBuffer->dataLenInBytes = *pEntropyLenReturned;
        }


        if( CPA_FALSE == pInternalState->isDFRequired )
        {
            status = LacDrbg_InstantiateNoDF(instanceHandle,
                                           pInternalState,
                                           pSetupData,
                                           pEntropyBuffer);

        }
        else
        {
            /* Obtain a nonce */
            pNonceBuffer = &pInternalState->buffers.nonceBuffer;
            pNonceBuffer->pData = pInternalState->buffers.nonce;
            pNonceBuffer->dataLenInBytes = pInternalState->nImplSeedLen;

            pEntropyOpData->sessionHandle = sessionHandle;
            pEntropyOpData->maxLength = pInternalState->nImplSeedLen;
            pEntropyOpData->minLength =
                LAC_DRBG_BITS_TO_BYTES(LAC_DRBG_NONCE_MIN_ENTROPY(secStren));
            pEntropyOpData->minEntropy = LAC_DRBG_NONCE_MIN_ENTROPY(secStren);

            pEntropyLenReturned = &pInternalState->buffers.entropyLenReturned;

            status = pInternalState->pGetNonceFunc(pEntropyOpData,
                                        pNonceBuffer,
                                        pEntropyLenReturned);


            if(status != CPA_STATUS_SUCCESS)
            {
                status = CPA_STATUS_FAIL;
            }

            if( CPA_STATUS_SUCCESS == status &&
                    (*pEntropyLenReturned < pNonceBuffer->dataLenInBytes) )
            {
                pNonceBuffer->dataLenInBytes = *pEntropyLenReturned;
            }

            if(CPA_STATUS_SUCCESS == status)
            {
                status = LacDrbg_InstantiateDF(instanceHandle,
                                          pInternalState,
                                          pSetupData,
                                          pEntropyBuffer,
                                          pNonceBuffer );
            }

        }
    }

    if( CPA_STATUS_SUCCESS == status )
    {
        /* Set prediction resistance flag */
        pInternalState->adminInfo.predictionResistanceRequired =
            pSetupData->predictionResistanceRequired;

        /* Set security strength */
        pInternalState->adminInfo.drbgSecurityStrength =
            pSetupData->secStrength;

        /* Set the callbacks */
        if(NULL == pGenCb)
        {
            pInternalState->pGenCb = LacSync_GenFlatBufCb;
        }
        else
        {
            pInternalState->pGenCb = pGenCb;
        }

        if(NULL == pReseedCb)
        {
            pInternalState->pReseedCb = LacDrbg_ReseedInternalCb;
        }
        else
        {
            pInternalState->pReseedCb = pReseedCb;
        }

        /* Set to indicate that state has not changed */
        pInternalState->isWorkingStateModified = CPA_FALSE;

        /* Return the seed length */
        *pSeedLen = pInternalState->nImplSeedLen;

        LAC_DRBG_STAT_INC(pInternalState, numSessionsInitialized, pService);
    }
    else
    {
        if (pInternalState != NULL)
        {
            LAC_DRBG_STAT_INC(pInternalState, numSessionErrors, pService);
        }
    }
    return status;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgGen(const CpaInstanceHandle instanceHandle_in,
        void *pCallbackTag,
        CpaCyDrbgGenOpData *pOpData,
        CpaFlatBuffer *pPseudoRandomBits)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    lac_drbg_internal_state_t *pInternalState = NULL;
    CpaFlatBuffer *pAdditionalInput = NULL;
    sal_crypto_service_t* pService = NULL;
#ifdef DRBG_POLL_AND_WAIT
    Cpa32U retries = 0;
    Cpa32U drbgPollAndWaitTime = 0;
#endif

#ifdef ICP_TRACE
    LAC_LOG4("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pOpData,
            (LAC_ARCH_UINT)pPseudoRandomBits);
#endif

    if( instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        status = LacDrbg_RdrandGen(instanceHandle,
                     pCallbackTag,
                     pOpData,
                     pPseudoRandomBits);
        return status;
    }

    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    LAC_DRBG_CHECK_NULL_PARAM(pOpData);
    LAC_DRBG_CHECK_NULL_PARAM(pPseudoRandomBits);
    LAC_DRBG_CHECK_NULL_PARAM(pPseudoRandomBits->pData);

    /* Step 1 of Generate Process, NIST SP 800-90, Section 9.3.1 */
    if((pPseudoRandomBits->dataLenInBytes < pOpData->lengthInBytes) ||
        (pOpData->lengthInBytes > LAC_DRBG_MAX_NUM_OF_BYTES ))
    {
        return CPA_STATUS_INVALID_PARAM;
    }

    LAC_DRBG_CHECK_NULL_PARAM(pOpData->sessionHandle);

    /* Check if additional input is valid; as this is an optional parameter
     * this might be have no data - in such case dataLenInBytes should
     * be set to 0 */
    if( 0 != pOpData->additionalInput.dataLenInBytes &&
        NULL == pOpData->additionalInput.pData )
    {
        return CPA_STATUS_INVALID_PARAM;
    }
    else
    {
        if( 0 == pOpData->additionalInput.dataLenInBytes
           || NULL == pOpData->additionalInput.pData)
        {
            pAdditionalInput = NULL;
        }
        else
        {
            pAdditionalInput = &pOpData->additionalInput;
        }
    }

    pInternalState =
        LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(pOpData->sessionHandle);
    LAC_ENSURE( NULL != pInternalState, "Internal state pointer is NULL");

    pService = (sal_crypto_service_t *)instanceHandle;

    /* Check if instance/session not in error state
     * (Step 2 of Generate Process, NIST SP 800-90, Section 9.3.1) */

    if( 0 != osalAtomicGet(&pInternalState->errorState)
        || 0 != osalAtomicGet(&pService->drbgErrorState))
    {
        LAC_LOG_ERROR("DRBG in error state");
        /* Increment statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenRequestErrors, pService);
        return CPA_STATUS_FAIL;
    }

    /* Lock the session to check if there are any other in flight requestes */
    LAC_SPINLOCK(&pInternalState->sessionLock);

    if( 0 != osalAtomicGet(&pInternalState->numInFlightRequests) )
    {
        /* A request is already being processed, return RETRY */
        LAC_SPINUNLOCK(&pInternalState->sessionLock);
        /* Increment statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenRequestErrors, pService);
        return CPA_STATUS_RETRY;
    }
    else
    {
        osalAtomicInc(&pInternalState->numInFlightRequests);
        pInternalState->operationType = LAC_DRBG_OPERATION_GENERATE;
    }
    LAC_SPINUNLOCK(&pInternalState->sessionLock);

    /* Step 3 of Generate Process, NIST SP 800-90, Section 9.3.1 */
    /* First check if secStrength within range */
    if(pOpData->secStrength < CPA_CY_RBG_SEC_STRENGTH_112 ||
       pOpData->secStrength > CPA_CY_RBG_SEC_STRENGTH_256)
    {
        return CPA_STATUS_INVALID_PARAM;
    }
    if(pOpData->secStrength > pInternalState->adminInfo.drbgSecurityStrength)
    {
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        osalAtomicDec(&pInternalState->numInFlightRequests);
        return CPA_STATUS_INVALID_PARAM;
    }

    /* 4 - for no DF case only - if DF is used an additional
     * input can't be too long*/
    if( CPA_FALSE == pInternalState->isDFRequired )
    {
        if(pOpData->additionalInput.dataLenInBytes
                > pInternalState->nImplSeedLen)
        {
            pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
            osalAtomicDec(&pInternalState->numInFlightRequests);
            return CPA_STATUS_INVALID_PARAM;
        }
    }

    /* 5 */
    if( CPA_TRUE == pOpData->predictionResistanceRequired
       && CPA_FALSE == pInternalState->adminInfo.predictionResistanceRequired )
    {
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        /* Increment statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenRequestErrors, pService);
        osalAtomicDec(&pInternalState->numInFlightRequests);
        return CPA_STATUS_FAIL;
    }

    pInternalState->buffers.cbDataForGenerate.pService =
        pService;
    pInternalState->buffers.cbDataForGenerate.pInternalState =
        pInternalState;
    pInternalState->buffers.cbDataForGenerate.cbOpData.
    asyncGenerateCbData.pPseudoRandomBits =
        pPseudoRandomBits;
    pInternalState->buffers.cbDataForGenerate.cbOpData.
    asyncGenerateCbData.pOpData =
        (void *)pOpData;
    pInternalState->buffers.cbDataForGenerate.cbOpData.
        asyncGenerateCbData.pCallbackTag =
            (void *)pCallbackTag;

    if( LacSync_GenFlatBufCb == pInternalState->pGenCb )
    {
        status = LacSync_CreateSyncCookie(&pInternalState->pSyncCallbackData);
        if( CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Problem when creating sync cookie");
            /* Increment statistics */
            LAC_DRBG_STAT_INC(pInternalState, numGenRequestErrors, pService);
            pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
            osalAtomicDec(&pInternalState->numInFlightRequests);
            return status;
        }

        pInternalState->buffers.cbDataForGenerate.cbOpData.
            asyncGenerateCbData.pCallbackTag =
                (void *)pInternalState->pSyncCallbackData;
    }

    /* 6 */
    pInternalState->reseedRequired = CPA_FALSE;

    /* Set to indicate that state has not changed since the beginning
     * of request processing */
    pInternalState->isWorkingStateModified = CPA_FALSE;

    /* 7 - 9 */
    do
    {
        /* 7 */
        if( CPA_TRUE == pInternalState->reseedRequired
                || CPA_TRUE == pOpData->predictionResistanceRequired )
        {
            status = LacDrbg_Reseed(instanceHandle,
                        pOpData->sessionHandle,
                        &pOpData->additionalInput,
                        LacDrbg_GenerateNextOpAfterReseedCb,
                        &pInternalState->buffers.cbDataForGenerate);

            if( CPA_STATUS_SUCCESS != status )
            {
                /* Do not decrease numInFlightRequests here as it was done
                 * in LacDrbg_Reseed */
                LAC_DRBG_STAT_INC(pInternalState,
                        numGenRequestErrors,
                        pService);
                if(pInternalState->pGenCb == LacSync_GenFlatBufCb)
                {
                    /* As the Request was not sent the Callback will never
                     * be called, so need to indicate that we're finished
                     * with cookie so it can be destroyed. */
                    LacSync_SetSyncCookieComplete(
                            pInternalState->pSyncCallbackData);
                    LacSync_DestroySyncCookie(
                            &pInternalState->pSyncCallbackData);
                    osalAtomicDec(&pInternalState->numInFlightRequests);
                }
                return CPA_STATUS_FAIL;
            }

            if(pInternalState->pGenCb != LacSync_GenFlatBufCb)
            {
                /* Do not decrease numInFlightRequests here as it will be done
                 * before calling user callback */
                LAC_DRBG_STAT_INC(pInternalState, numGenRequests, pService);
                return status;
            }
            else
            {
                /* Break the do..while loop and wait for callback.
                 * Next call to generate will be made from inside
                 * reseed callback (LacDrbg_GenerateNextOpAfterReseedCb. )*/
                break;
            }
        }

        /* 8
         * The calls below are made only if prediction resistance was not
         * requested else Generate function will be called
         * from inside LacDrbg_GenerateNextOpAfterReseedCb */
        if( CPA_FALSE == pInternalState->isDFRequired )
        {
            status = LacDrbg_GenerateNoDF(pService,
                        pInternalState,
                        pOpData->lengthInBytes,
                        pAdditionalInput,
                        pPseudoRandomBits);
        }
        else
        {
            status = LacDrbg_GenerateDF(pService,
                        pInternalState,
                        pOpData->lengthInBytes,
                        pAdditionalInput,
                        pPseudoRandomBits);
        }

    /* 9
     * If reseed is required, generate will return RETRY and
     * reseedRequired flag will be set. */
    }while( CPA_STATUS_RETRY == status
            && CPA_TRUE == pInternalState->reseedRequired );

    /* Lines below will be hit in following case:
     * - if session was set to operate synchronously and either prediction
     * resistance is required or seed has ended its life and reseed is required
     * - if prediction resistance is not requested and seed has not exceeded
     * its lifetime
     */

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Increment statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenRequestErrors, pService);
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        if(pInternalState->pGenCb != LacSync_GenFlatBufCb)
        {
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }
    }
    else
    {
        /* Increment statistics */
        LAC_DRBG_STAT_INC(pInternalState, numGenRequests, pService);
    }

    if(pInternalState->pGenCb == LacSync_GenFlatBufCb)
    {
        if( CPA_STATUS_SUCCESS == status )
        {
            CpaStatus syncStatus = CPA_STATUS_SUCCESS;
            CpaBoolean opResult = CPA_FALSE;

#ifdef DRBG_POLL_AND_WAIT
            drbgPollAndWaitTime = pService->drbgPollAndWaitTime;
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
                    pInternalState->pSyncCallbackData,
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
                    pInternalState->pSyncCallbackData,
                    LAC_SYM_DRBG_SYNC_CALLBACK_TIMEOUT,
                    &status,
                    &opResult);
#endif

            /* If callback doesn't come back */
            if ( syncStatus != CPA_STATUS_SUCCESS )
            {
                LAC_LOG_ERROR("Callback timed out");
                LAC_DRBG_STAT_INC(pInternalState,
                        numGenCompletedErrors,
                        pService);
                status = syncStatus;
            }
            LacSync_DestroySyncCookie(&pInternalState->pSyncCallbackData);
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }
        else
        {
            /* As the Request was not sent the Callback will never
             * be called, so need to indicate that we're finished
             * with cookie so it can be destroyed. */
            LacSync_SetSyncCookieComplete(pInternalState->pSyncCallbackData);
            LacSync_DestroySyncCookie(&pInternalState->pSyncCallbackData);
            osalAtomicDec(&pInternalState->numInFlightRequests);
        }
    }
    return status;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgReseed(const CpaInstanceHandle instanceHandle_in,
        void *pCallbackTag,
        CpaCyDrbgReseedOpData *pOpData)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    lac_drbg_internal_state_t *pInternalState = NULL;
    LacDrbgNextOpCbFunc pNextOpFunc = NULL;
    sal_crypto_service_t* pService = NULL;
    lac_drbg_callback_data_t *pCbData = NULL;
#ifdef DRBG_POLL_AND_WAIT
    Cpa32U retries = 0;
    Cpa32U drbgPollAndWaitTime = 0;
#endif

#ifdef ICP_TRACE
    LAC_LOG3("Called with params (0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pCallbackTag,
            (LAC_ARCH_UINT)pOpData);
#endif

    if( instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE )
    {
        instanceHandle = Lac_GetFirstHandle();
    }else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        LAC_LOG_ERROR("Reseed is not supported when using rdrand");
        return CPA_STATUS_UNSUPPORTED;
    }

    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    LAC_DRBG_CHECK_NULL_PARAM(pOpData);
    LAC_DRBG_CHECK_NULL_PARAM(pOpData->sessionHandle);


    if( NULL == pOpData->additionalInput.pData
            && 0 != pOpData->additionalInput.dataLenInBytes )
    {
        return CPA_STATUS_INVALID_PARAM;
    }


    pService = (sal_crypto_service_t*) instanceHandle;
    pInternalState =
        LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(pOpData->sessionHandle);
    LAC_ENSURE(NULL != pInternalState, "Internal state pointer is NULL");

    /* Check if instance not in error state */
    if( 0 != osalAtomicGet(&pInternalState->errorState)
        || 0 != osalAtomicGet(&pService->drbgErrorState))
    {
        LAC_LOG_ERROR("DRBG in error state");
        return CPA_STATUS_FAIL;
    }

    if( CPA_FALSE == pInternalState->isDFRequired )
    {
        if(pOpData->additionalInput.dataLenInBytes
                > pInternalState->nImplSeedLen )
        {
            return CPA_STATUS_INVALID_PARAM;
        }
    }

    /* Lock the session to check if there are any other in flight requestes */
    LAC_SPINLOCK(&pInternalState->sessionLock);

    if( 0 != osalAtomicGet(&pInternalState->numInFlightRequests) )
    {
        /* A request is already being processed, return RETRY */
        LAC_SPINUNLOCK(&pInternalState->sessionLock);
        return CPA_STATUS_RETRY;
    }
    else
    {
        osalAtomicInc(&pInternalState->numInFlightRequests);
        pInternalState->operationType = LAC_DRBG_OPERATION_RESEED;
    }
    LAC_SPINUNLOCK(&pInternalState->sessionLock);

    /* Set to indicate that state has not changed since the beginning
     * of request processing */
    pInternalState->isWorkingStateModified = CPA_FALSE;

    pNextOpFunc = LacDrbg_ReseedCallUserCb;
    pCbData = &pInternalState->buffers.cbUserDataForReseed;
    pCbData->pService = pService;
    pCbData->pInternalState = pInternalState;
    pCbData->pNextOpCallbackTag = pCallbackTag;
    pCbData->cbOpData.asyncReseedCbData.pOpData = pOpData;

    if( LacDrbg_ReseedInternalCb == pInternalState->pReseedCb )
    {
        status = LacSync_CreateSyncCookie(&pInternalState->pSyncCallbackData);
        if( CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Problem when creating sync cookie");
            return status;
        }
        pCbData->pNextOpCallbackTag = pInternalState->pSyncCallbackData;
    }

    status = LacDrbg_Reseed(pService,
                           pOpData->sessionHandle,
                           &pOpData->additionalInput,
                           pNextOpFunc,
                           pCbData);

    if(pInternalState->pReseedCb == LacDrbg_ReseedInternalCb)
    {
        if( CPA_STATUS_SUCCESS == status )
        {
            CpaStatus syncStatus = CPA_STATUS_SUCCESS;
            CpaBoolean opResult = CPA_FALSE;

#ifdef DRBG_POLL_AND_WAIT
            drbgPollAndWaitTime = pService->drbgPollAndWaitTime;;
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
                    pInternalState->pSyncCallbackData,
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
                            pInternalState->pSyncCallbackData,
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
        }
        else
        {
            /* As the Request was not sent the Callback will never
             * be called, so need to indicate that we're finished
             * with cookie so it can be destroyed. */
            LacSync_SetSyncCookieComplete(pInternalState->pSyncCallbackData);
        }
        pInternalState->operationType = LAC_DRBG_OPERATION_NONE;
        LacSync_DestroySyncCookie(&pInternalState->pSyncCallbackData);
        osalAtomicDec(&pInternalState->numInFlightRequests);
    }

    return status;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgRemoveSession(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle)
{
    lac_drbg_internal_state_t *pInternalState = NULL;
    CpaInstanceHandle instanceHandle;
#ifndef DISABLE_STATS
    sal_crypto_service_t* pService = NULL;
#endif

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)sessionHandle);
#endif
    if( instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE )
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        return LacDrbg_RdrandRemoveSession(instanceHandle,
                   sessionHandle);
    }

    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    LAC_DRBG_CHECK_NULL_PARAM(sessionHandle);

#ifndef DISABLE_STATS
    pService = (sal_crypto_service_t*) instanceHandle;
#endif
    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    LAC_ENSURE( NULL != pInternalState, "Internal state pointer is NULL");

    if( 0 != osalAtomicGet(&pInternalState->numInFlightRequests) )
    {
#ifndef DISABLE_STATS
        /* A request is already being processed, return RETRY */
        LAC_DRBG_STAT_INC(pInternalState, numSessionErrors, pService);
#endif
        return CPA_STATUS_RETRY;
    }

    LAC_SPINLOCK_DESTROY(&pInternalState->sessionLock);

    LAC_OS_BZERO(sessionHandle, sizeof(lac_drbg_internal_state_t)
                                    + pInternalState->symSessionSize
                                    + LAC_8BYTE_ALIGNMENT
                                    + sizeof(LAC_ARCH_UINT));

#ifndef DISABLE_STATS
    LAC_DRBG_STAT_INC(pInternalState, numSessionsRemoved, pService);
#endif
    return CPA_STATUS_SUCCESS;
}

/**
 * @ingroup cpaCyDrbg
 */
CpaStatus
cpaCyDrbgQueryStats64(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgStats64 *pStats)
{
    CpaInstanceHandle instanceHandle;
    sal_crypto_service_t* pService = NULL;
    /* Used for iterating through array of statistic atomic variables */
    Cpa32U i = 0;

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)instanceHandle_in,
            (LAC_ARCH_UINT)pStats);
#endif

    if( instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE )
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    LAC_DRBG_CHECK_NULL_PARAM(pStats);

    pService = (sal_crypto_service_t*) instanceHandle;

    for (i = 0; i < LAC_DRBG_NUM_STATS; i ++)
    {
        ((Cpa64U *)pStats)[i] = osalAtomicGet(&pService->pLacDrbgStatsArr[i]);
    }

    return CPA_STATUS_SUCCESS;
}
