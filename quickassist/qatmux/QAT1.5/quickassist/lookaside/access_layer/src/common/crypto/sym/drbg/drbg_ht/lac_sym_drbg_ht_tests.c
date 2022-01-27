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
 * @file lac_sym_drbg_ht_tests.c
 *
 * @ingroup LacSym_DrbgHT
 *
 * @description
 * Implementation of the Deterministic Random Bit Generation API - functions
 * used for health testing
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
#include "icp_sal_drbg_ht.h"
/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "icp_adf_transport.h"
#include "icp_buffer_desc.h"
#include "sal_service_state.h"
#include "lac_sym.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_sym_drbg.h"
#include "lac_sym_drbg_ht.h"
#include "lac_sym_drbg_ht_test_vectors.h"
#include "Osal.h"

/* Test specific get entropy function pointer */
IcpSalDrbgGetEntropyInputFunc pTestGetEntropyInputFunc = NULL;

/* Test specific get nonce function pointer */
IcpSalDrbgGetNonceFunc pTestGetNonceFunc = NULL;

/* Test specific derivation function pointer */
IcpSalDrbgIsDFReqFunc pTestIsDFReqFunc = NULL;

/* Implementation specific derivation function pointer */
extern IcpSalDrbgIsDFReqFunc pIsDFReqFunc;

static OsalMutex lock = NULL;

STATIC CpaStatus
LacDrbg_HTRemoveSession(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle);

int lacSymDrbgLock_init(void)
{
    return osalMutexInit(&lock);
}

void lacSymDrbgLock_exit(void)
{
    osalMutexDestroy(&lock);
}

int lacSymDrbgLock_lock(void)
{
    return osalMutexLock(&lock, OSAL_WAIT_FOREVER);
}

void lacSymDrbgLock_unlock(void)
{
    osalMutexUnlock(&lock);
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Get Entropy Input function used during Instantiate health tests
 *
 * @description
 *      Get Entropy Input function used during Instantiate health tests
 *
 * @param[in]  pCb              Callback that is to be called to notify that
 *                              entropy input is ready when operating in
 *                              asynchronous mode; if NULL then this is a
 *                              synchronous request and the entropy input
 *                              should be available upon returning from this
 *                              function
 * @param[in]  pCallbackTag     Opaque pointer to user data that is not to be
 *                              changed internally
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of entropy input requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              entropy input is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned entropy input is
 *                              to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_TestInstGetEntropyInputFunc(IcpSalDrbgGetEntropyInputCbFunc pCb,
        void * pCallbackTag,
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;

    LAC_ENSURE(pBuffer != NULL, "pBuffer is NULL");
    LAC_ENSURE(pOpData != NULL, "pOpData is NULL");
    LAC_ENSURE(pOpData->sessionHandle != NULL, "sessionHandle is NULL");

    pInstVector = (lac_drbg_ht_instantiate_vector_t *)
                        LAC_DRBG_HT_GET_VECTOR_POINTER(pOpData->sessionHandle);
    LAC_ENSURE(pInstVector != NULL, "Vector is NULL");

    if(pOpData->minLength > pInstVector->entropyInputLen
           || pOpData->maxLength < pInstVector->entropyInputLen)
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    if( NULL == pCb && NULL == pLengthReturned )
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    osalMemCopy(pBuffer->pData,
                pInstVector->entropyInput,
                pInstVector->entropyInputLen);

    if( NULL != pCb )
    {
        pCb(pCallbackTag,
                CPA_STATUS_SUCCESS,
                pOpData,
                pInstVector->entropyInputLen,
                pBuffer);
    }
    else
    {
        *pLengthReturned = pInstVector->entropyInputLen;
    }

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Negative scenario Get Entropy Input function used during
 *      health tests
 *
 * @description
 *      Negative scenario Get Entropy Input function used during
 *      health tests. Always returns CPA_STATUS_FAIL
 *
 * @param[in]  pCb              Callback that is to be called to notify that
 *                              entropy input is ready when operating in
 *                              asynchronous mode; if NULL then this is a
 *                              synchronous request and the entropy input
 *                              should be available upon returning from this
 *                              function
 * @param[in]  pCallbackTag     Opaque pointer to user data that is not to be
 *                              changed internally
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of entropy input requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              entropy input is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned entropy input is
 *                              to be stored
 *
 * @return CPA_STATUS_FAIL      Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_TestGetEntropyInputFailFunc(IcpSalDrbgGetEntropyInputCbFunc pCb,
        void * pCallbackTag,
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    return CPA_STATUS_FAIL;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Get Nonce function used during Instantiate health tests
 *
 * @description
 *      Get Nonce function used during Instantiate health tests
 *
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of nonce requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              nonce is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned nonce is to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_TestInstGetNonce(
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    lac_drbg_ht_instantiate_vector_t * pInstVector = NULL;

    LAC_ENSURE(pBuffer != NULL, "pBuffer is NULL");
    LAC_ENSURE(pLengthReturned != NULL, "pLengthReturned is NULL");
    LAC_ENSURE(pOpData != NULL, "pOpData is NULL");
    LAC_ENSURE(pOpData->sessionHandle != NULL, "sessionHandle is NULL");

    pInstVector = (lac_drbg_ht_instantiate_vector_t *)
                    LAC_DRBG_HT_GET_VECTOR_POINTER(pOpData->sessionHandle);
    LAC_ENSURE(pInstVector != NULL, "Vector is NULL");

    if( pOpData->minLength > pInstVector->nonceLen
           || pOpData->maxLength < pInstVector->nonceLen)
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    *pLengthReturned = pInstVector->nonceLen;
    osalMemCopy(pBuffer->pData,
                pInstVector->nonce,
                pInstVector->nonceLen);

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Is DF Required function used during Instantiate health tests for cases
 *      where DF is required
 *
 * @description
 *      Is DF Required function used during Instantiate health tests for cases
 *      where DF is required
 *
 * @return CPA_TRUE     Derivation function is used
 *
 *****************************************************************************/
STATIC CpaBoolean
LacDrbg_TestInstIsDFReqTrue(void)
{
    return CPA_TRUE;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Is DF Required function used during Instantiate health tests for cases
 *      where DF is not required
 *
 * @description
 *      Is DF Required function used during Instantiate health tests for cases
 *      where DF is not required
 *
 * @return CPA_FALSE     Derivation function is not used
 *
 *****************************************************************************/
STATIC CpaBoolean
LacDrbg_TestInstIsDFReqFalse(void)
{
    return CPA_FALSE;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Returns test vector that is to be used during Instantiate health tests
 *
 * @description
 *      Returns test vector that is to be used during Instantiate health tests
 *      based on the input parameters
 *
 * @param[in]  secStrength      Security strength for which DRBG is going to
 *                              be instantiated
 * @param[in]  predictionResistanceRequired     Flag indicating whether DRBG
 *                              is to be instantiated with prediction
 *                              resistance support turned on
 * @param[in]  isDFUsed         Flag indicating whether derivation function
 *                              is to be used in DRBG processing
 *
 * @return Pointer to a vector selected for testing if found
 * @return NULL if proper vector not found
 *
 *****************************************************************************/
STATIC lac_drbg_ht_instantiate_vector_t *
LacDrbg_HTGetInstantiateKATVector(CpaCyDrbgSecStrength secStrength,
        CpaBoolean predictionResistanceRequired,
        CpaBoolean isDFUsed)
{
    int index = 0;
    for(index = 0; index < LAC_DRBG_HT_NUM_INSTANTIATE_VECTORS; index++)
    {
        if( instantiateVectors[index].secStrength ==
                secStrength
            && instantiateVectors[index].predictionResistanceRequired ==
                predictionResistanceRequired
            && instantiateVectors[index].isDFUsed ==
                isDFUsed)
        {
            return instantiateVectors + index;
        }
    }
    return NULL;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Health tests for DRBG Instantiate functionality
 *
 * @description
 *      Health tests for DRBG Instantiate functionality where DRBG is tested
 *      with given vector. This function tests positive scenario and negative
 *      one as well (invalid params and failin 'Get Entropy Input' function)
 *
 * @param[in] instanceHandle_in Instance handle
 * @param[in] sessionHandle     Session Handle on which Instantiate
 *                              functionality is to be tested.
 * @param[in] pVector           Pointer to a vector Instantiate is to be
 *                              tested with.
 * @param[in] pBuffer           Pointer to a buffer that is to be used to
 *                              pass personalization string parameter to
 *                              Instantiate function. It must point to user
 *                              allocated physically contiguous memory
 *
 * @return CPA_STATUS_SUCCESS   Tests passed
 * @return CPA_STATUS_FAIL      Tests failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTInitSessionVector(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        lac_drbg_ht_instantiate_vector_t *pVector,
        Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t* pService = NULL;
    /* structure initializer is supported by C99, but
     * it is not supported by some old Intel compiler
     */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaCyDrbgSessionSetupData testSetupData = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif
    lac_drbg_internal_state_t *pInternalState = NULL;
    Cpa32U seedLen = 0;

    /* 1. Get the 'real' instance handle */
    if( instanceHandle_in == CPA_INSTANCE_HANDLE_SINGLE )
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);

    /* 2. Get the session memory pointer */
    pService = (sal_crypto_service_t *)instanceHandle;

    /* 2a.  Check if drbg not in error state */
    if(0 != osalAtomicGet(&pService->drbgErrorState))
    {
        return CPA_STATUS_FAIL;
    }

    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pVector);
    if(CPA_STATUS_SUCCESS != lacSymDrbgLock_lock())
    {
        return CPA_STATUS_FAIL;
    }

    /* 3a.  Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    if(CPA_TRUE ==pVector->isDFUsed)
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqTrue;
    }
    else
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqFalse;
    }

    /* 4. Run InitSession function - expect success */
    testSetupData.predictionResistanceRequired =
        pVector->predictionResistanceRequired;
    testSetupData.secStrength = pVector->secStrength;

    testSetupData.personalizationString.dataLenInBytes =
        pVector->persStringLen;
    testSetupData.personalizationString.pData = pBuffer;
    osalMemCopy(testSetupData.personalizationString.pData,
            pVector->persString,
            pVector->persStringLen);

    status = LacDrbg_InitSession(instanceHandle_in,
            NULL,
            NULL,
            &testSetupData,
            sessionHandle,
            &seedLen,
            CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);

    /* 5. Compare working state (Key, V, reseedCounter) with the values from
     * the vector */
    if( 0 != memcmp(pInternalState->workingState.key,
            pVector->key,
            pInternalState->nImplKeyLen))

    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    if( 0 != memcmp(pInternalState->workingState.v,
            pVector->v,
            LAC_DRBG_OUTLEN_IN_BYTES) )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    if( 1 != pInternalState->workingState.reseedCounter )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* 6. Negative test case - getEntropyInput failing (function returns FAIL
     *   and DRBG error flag is set in crypto_service structure) */
    pTestGetEntropyInputFunc = LacDrbg_TestGetEntropyInputFailFunc;
    status = LacDrbg_InitSession(instanceHandle_in,
            NULL,
            NULL,
            &testSetupData,
            sessionHandle,
            &seedLen,
            CPA_TRUE);

    if( CPA_STATUS_FAIL != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    if( 0 == osalAtomicGet(&pService->drbgErrorState) )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* 7. Clear error flag */
    osalAtomicSet(0, &pService->drbgErrorState);

    /* 8. Negative test cases - handling of input parameters */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;

    /* 8a.   Testing for NULL setup parameters pointer */
    status = LacDrbg_InitSession(instanceHandle_in,
            NULL,
            NULL,
            NULL,
            sessionHandle,
            &seedLen,
            CPA_TRUE);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* Error state should not be set */
    if( 0 != osalAtomicGet(&pService->drbgErrorState) )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* 8b.  Testing for NULL session handle */
    status = LacDrbg_InitSession(instanceHandle_in,
            NULL,
            NULL,
            &testSetupData,
            NULL,
            &seedLen,
            CPA_TRUE);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* Error state should not be set */
    if( 0 != osalAtomicGet(&pService->drbgErrorState) )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* 8c.  Testing for NULL seed length pointer */
    status = LacDrbg_InitSession(instanceHandle_in,
            NULL,
            NULL,
            &testSetupData,
            sessionHandle,
            NULL,
            CPA_TRUE);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* Error state should not be set */
    if( 0 != osalAtomicGet(&pService->drbgErrorState) )
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }

    /* Test Remove Session function */
    status = LacDrbg_HTRemoveSession(instanceHandle_in, sessionHandle);
    if(CPA_STATUS_SUCCESS != status)
    {
        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
        lacSymDrbgLock_unlock();
        return CPA_STATUS_FAIL;
    }
    lacSymDrbgLock_unlock();
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Health tests for DRBG Instantiate functionality
 *
 * @description
 *      Health tests for DRBG Instantiate functionality where DRBG is tested
 *      with given setup data
 *
 * @param[in]  instanceHandle_in  Instance handle.
 * @param[in]  pSetupData         Pointer to a setup data Instantiate is to be
 *                                tested with.
 * @param[in]  sessionHandle      Session Handle on which Instantiate
 *                                functionality is to be tested.
 *
 * @return CPA_STATUS_SUCCESS   Tests passed
 * @return CPA_STATUS_FAIL      Tests failed
 *
 *****************************************************************************/
CpaStatus
LacDrbg_HTInitSession(const CpaInstanceHandle instanceHandle_in,
            const CpaCyDrbgSessionSetupData *pSetupData,
            CpaCyDrbgSessionHandle sessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_ht_instantiate_vector_t *pVector = NULL;
    Cpa8U *pTestBuffer = NULL;
    CpaCyDrbgSessionHandle sessionHandleLocal = NULL;

    if( NULL == pSetupData )
    {
        LAC_LOG_ERROR("pSetupData is NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    if( NULL == sessionHandle )
    {
        LAC_LOG_ERROR("sessionHandle is NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    if( 0 != pSetupData->personalizationString.dataLenInBytes &&
                     NULL == pSetupData->personalizationString.pData )
    {
        LAC_LOG_ERROR("Invalid param - personalizationString");
        return CPA_STATUS_INVALID_PARAM;
    }


    if( NULL == pIsDFReqFunc )
    {
        LAC_LOG_ERROR("'Is DF Required' function not registered");
        return CPA_STATUS_FAIL;
    }

    if (CPA_TRUE == pIsDFReqFunc())
    {
        if (CPA_CY_RBG_SEC_STRENGTH_192 == pSetupData->secStrength)
        {
            LAC_LOG_ERROR("Security strength 192 with prediction "
                "resistance not supported\n");
            return CPA_STATUS_INVALID_PARAM;
        }
        if (CPA_CY_RBG_SEC_STRENGTH_256 == pSetupData->secStrength)
        {
            LAC_LOG_ERROR("Security strength 256 with prediction "
                "resistance not supported\n");
            return CPA_STATUS_INVALID_PARAM;
        }
    }

    /* Select the correct vector based on security strength and prediction
     * resistance flag from pSetupData */
    pVector = LacDrbg_HTGetInstantiateKATVector(pSetupData->secStrength,
            pSetupData->predictionResistanceRequired,
            pIsDFReqFunc());

    if( NULL == pVector )
    {
        LAC_LOG_ERROR("No test vector for requested session setup");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* First sizeof(void*) bytes of sessionHandle is reserve for a pointer
     * to a vector used while health testing. Adjust session handle to skip
     * these bytes */
    sessionHandleLocal = (CpaCyDrbgSessionHandle)
                            ((Cpa8U*)sessionHandle + sizeof(void*));

    /* Allocated session memory includes additional MAX_SEEDLEN bytes for
     * a buffer used during health testing - obtain pointer to this buffer */
    pTestBuffer = (Cpa8U*)sessionHandleLocal
                        + sizeof(lac_drbg_internal_state_t)
                        + LAC_SYM_SESSION_SIZE
                        + LAC_8BYTE_ALIGNMENT
                        + sizeof(LAC_ARCH_UINT);

    if(CPA_TRUE == pIsDFReqFunc())
    {
        pTestBuffer += LAC_SYM_SESSION_SIZE;
    }

    status = LacDrbg_HTInitSessionVector(instanceHandle_in,
                sessionHandleLocal,
                pVector,
                pTestBuffer);

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Get Entropy Input function used during Generate health tests
 *
 * @description
 *      Get Entropy Input function used during Generate health tests
 *
 * @param[in]  pCb              Callback that is to be called to notify that
 *                              entropy input is ready when operating in
 *                              asynchronous mode; if NULL then this is a
 *                              synchronous request and the entropy input
 *                              should be available upon returning from this
 *                              function
 * @param[in]  pCallbackTag     Opaque pointer to user data that is not to be
 *                              changed internally
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of entropy input requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              entropy input is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned entropy input is
 *                              to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid param provided
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_TestGenGetEntropyInput(IcpSalDrbgGetEntropyInputCbFunc pCb,
        void * pCallbackTag,
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    lac_drbg_ht_generate_vector_t * pGenVector = NULL;

    LAC_ENSURE(pBuffer != NULL, "pBuffer is NULL");
    LAC_ENSURE(pOpData != NULL, "pOpData is NULL");
    LAC_ENSURE(pOpData->sessionHandle != NULL, "sessionHandle is NULL");

    pGenVector = (lac_drbg_ht_generate_vector_t *)
                    LAC_DRBG_HT_GET_VECTOR_POINTER(pOpData->sessionHandle);
    LAC_ENSURE(pGenVector != NULL, "test vector is NULL");

    if( pOpData->minLength > pGenVector->entropyInputLen
           || pOpData->maxLength < pGenVector->entropyInputLen)
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    if( NULL == pCb && NULL == pLengthReturned )
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    osalMemCopy(pBuffer->pData,
                pGenVector->entropyInput,
                pGenVector->entropyInputLen);

    if( NULL != pCb )
    {
        pCb(pCallbackTag,
                CPA_STATUS_SUCCESS,
                pOpData,
                pGenVector->entropyInputLen,
                pBuffer);
    }
    else
    {
        *pLengthReturned = pGenVector->entropyInputLen;
    }

    return CPA_STATUS_SUCCESS;
}


/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Get Nonce function used during  Generate health tests
 *
 * @description
 *      Get Nonce function used during Generate health tests. This is only
 *      a stub, function is only required for DRBG session init stage
 *      of Generate function health tests
 *
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of nonce requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              nonce is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned nonce is to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 *
 *****************************************************************************/
CpaStatus
LacDrbg_TestGenGetNonce(
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    *pLengthReturned = pOpData->minLength;
    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Get Entropy Input function used during Reseed health tests
 *
 * @description
 *      Get Entropy Input function used during Reseed health tests
 *
 * @param[in]  pCb              Callback that is to be called to notify that
 *                              entropy input is ready when operating in
 *                              asynchronous mode; if NULL then this is a
 *                              synchronous request and the entropy input
 *                              should be available upon returning from this
 *                              function
 * @param[in]  pCallbackTag     Opaque pointer to user data that is not to be
 *                              changed internally
 * @param[in]  pOpData          Pointer to Op Data structure defining
 *                              parameters of entropy input requested
 * @param[in]  pBuffer          Pointer to a Flat Buffer where requested
 *                              entropy input is to be stored; this buffer is
 *                              supposed to store no more than maxLength bytes
 *                              as indicated in Op Data structure
 * @param[out] pLengthReturned  Pointer to a Cpa32U variable where the actual
 *                              length of returned entropy input is
 *                              to be stored
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Operation failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_TestReseedGetEntropyInput(IcpSalDrbgGetEntropyInputCbFunc pCb,
        void * pCallbackTag,
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    lac_drbg_ht_reseed_vector_t * pReseedVector = NULL;

    LAC_ENSURE(pBuffer != NULL, "pBuffer is NULL");
    LAC_ENSURE(pOpData != NULL, "pOpData is NULL");
    LAC_ENSURE(pOpData->sessionHandle != NULL, "sessionHandle is NULL");

    pReseedVector = (lac_drbg_ht_reseed_vector_t *)
                      LAC_DRBG_HT_GET_VECTOR_POINTER(pOpData->sessionHandle);
    LAC_ENSURE(pReseedVector != NULL, "test vector is NULL");

    if( pOpData->minLength > pReseedVector->entropyInputLen
           || pOpData->maxLength < pReseedVector->entropyInputLen)
    {
        LAC_LOG_ERROR("Invalid data received");
        return CPA_STATUS_FAIL;
    }
    if( NULL == pCb && NULL == pLengthReturned )
    {
        LAC_LOG_ERROR("Received incorrect data");
        return CPA_STATUS_FAIL;
    }

    osalMemCopy(pBuffer->pData,
                pReseedVector->entropyInput,
                pReseedVector->entropyInputLen);

    if( NULL != pCb )
    {
        pCb(pCallbackTag,
                CPA_STATUS_SUCCESS,
                pOpData,
                pReseedVector->entropyInputLen,
                pBuffer);
    }
    else
    {
        *pLengthReturned = pReseedVector->entropyInputLen;
    }

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      Health tests for DRBG Remove Session functionality
 *
 * @description
 *      Health tests for DRBG Remove Session functionality. Always run together
 *      with health tests for other DRBG functions
 *
 * @param[in]  instanceHandle_in Instance Handle
 * @param[in]  sessionHandle     Session Handle on which Remove Session
 *                               functionality is to be tested.
 *
 * @assumption  The sessionHandle has been adjusted in the health test function
 *              that this function is called from to skip sizeof(void*) bytes
 *              that store pointer to a known answer vector
 *
 * @return CPA_STATUS_SUCCESS   Tests passed
 * @return CPA_STATUS_FAIL      Tests failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTRemoveSession(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    /* Structure initializer is supported by C99, but
     * it is not supported by some old Intel compiler
     */
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(disable: 188)
#endif
    CpaCyDrbgSessionSetupData setupData = {0};
#if defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1300)
#pragma warning(enable)
#endif

    sal_crypto_service_t* pService = NULL;
    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;
    Cpa32U index = 0;
    Cpa32U seedLen = 0;
    Cpa32U memSizeToZero = 0;
    Cpa8U *pSessionMemory = NULL;

    /* Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqFalse;

    /* Select the instantiation vector */
    pInstVector = LacDrbg_HTGetInstantiateKATVector(
                        CPA_CY_RBG_SEC_STRENGTH_128,
                        CPA_TRUE,
                        CPA_FALSE);

    LAC_ENSURE( NULL != pInstVector, "No vector for instantiate" );
    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pInstVector);

    /* Init DRBG session (only to set up symmetric sessions) */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_TRUE;
    setupData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;

    status = LacDrbg_InitSession(instanceHandle_in,
                NULL,
                NULL,
                &setupData,
                sessionHandle,
                &seedLen,
                CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Failed to initialize DRBG session");
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);

    /* sessionHandle has previously been adjusted to skip vector pointer bytes,
     * so we need to make sure that during call to cpaCyDrbgRemoveSession we
     * don't overwrite bytes that do not belong to session memory. To do this
     * we need to decrement pInternalState->symSessionSize. */
    pInternalState->symSessionSize -= sizeof(void *);

    memSizeToZero = pInternalState->symSessionSize
                        + sizeof(lac_drbg_internal_state_t)
                        + LAC_8BYTE_ALIGNMENT
                        + sizeof(LAC_ARCH_UINT);

    pSessionMemory = (Cpa8U*)sessionHandle;

    /* Checking proper handling of NULL parameters */
    status = cpaCyDrbgRemoveSession(instanceHandle_in, NULL);
    if( CPA_STATUS_INVALID_PARAM != status )
    {
        status = CPA_STATUS_FAIL;
    }
    else
    {
        status = CPA_STATUS_SUCCESS;
    }

    /* Checking if internal state is being zeroized */
    if( CPA_STATUS_SUCCESS == status )
    {
        status = cpaCyDrbgRemoveSession(instanceHandle_in, sessionHandle);
    }
    if( CPA_STATUS_SUCCESS == status )
    {
        for( index = 0;
             index < memSizeToZero;
             index++, pSessionMemory++ )
        {
            if(*pSessionMemory != 0)
            {
                status = CPA_STATUS_FAIL;
                break;
            }
        }
    }

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Get the 'real' instance handle */
        if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
        {
            instanceHandle = Lac_GetFirstHandle();
        }
        else
        {
            instanceHandle = instanceHandle_in;
        }

        pService = (sal_crypto_service_t *)instanceHandle;

        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Generate health test function
 *
 * @description
 *      This function tests cpaCyDrbgGen function for health. Whether to use
 *      derivation function is indicated by bUseDF param.
 *
 * @param[in]  instanceHandle_in      Instance Handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  bUseDF                 Flag indicating whether derivation
 *                                    function is to be used during testing
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Generate function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTGenerateUseDF(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        CpaBoolean bUseDF,
        Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    CpaCyDrbgSessionSetupData setupData;
    CpaCyDrbgSecStrength secStrength;
    CpaCyDrbgGenOpData opData;
    CpaFlatBuffer randomBytes;

    lac_drbg_ht_generate_vector_t *pVectorList = NULL;
    lac_drbg_ht_generate_vector_t *pGenVector = NULL;
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;
    Cpa32U numVectors = 0;

    Cpa32U seedLen = 0;
    Cpa32U index = 0;

    lac_drbg_internal_state_t *pInternalState = NULL;

    if(bUseDF)
    {
        secStrength = CPA_CY_RBG_SEC_STRENGTH_128;
        pVectorList = generateDFVectors;
        numVectors = LAC_DRBG_HT_NUM_GENERATE_DF_VECTORS;
    }
    else
    {
        secStrength = CPA_CY_RBG_SEC_STRENGTH_256;
        pVectorList = generateNoDFVectors;
        numVectors = LAC_DRBG_HT_NUM_GENERATE_NODF_VECTORS;
    }

    /* Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    if(CPA_TRUE == bUseDF)
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqTrue;
    }
    else
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqFalse;
    }


    /* Select the instantiation vector */
    pInstVector = LacDrbg_HTGetInstantiateKATVector(
                    secStrength,
                    CPA_TRUE,
                    bUseDF);

    if( NULL == pInstVector )
    {
        LAC_LOG_ERROR("No vector for instantiate!");
        return CPA_STATUS_FAIL;
    }

    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pInstVector);

    /* Init DRBG session (only to set up symmetric sessions) */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_TRUE;
    setupData.secStrength = secStrength;

    status = LacDrbg_InitSession(instanceHandle_in,
                NULL,
                NULL,
                &setupData,
                sessionHandle,
                &seedLen,
                CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialize DRBG session");
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    pInternalState->pGetEntropyInputFunc = LacDrbg_TestGenGetEntropyInput;
    pInternalState->pGetNonceFunc = LacDrbg_TestGenGetNonce;
    pInternalState->isTestRun = CPA_TRUE;

    for(index = 0; index < numVectors; index++)
    {
        pGenVector = pVectorList + index;
        LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pGenVector);

        pInternalState->adminInfo.predictionResistanceRequired =
            pGenVector->predictionResistanceEnabled;
        pInternalState->isDFRequired = pGenVector->isDFUsed;

        if( CPA_TRUE == pInternalState->isDFRequired )
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN128_IN_BYTES;
            pInternalState->adminInfo.drbgSecurityStrength =
                CPA_CY_RBG_SEC_STRENGTH_128;
        }
        else
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN256_IN_BYTES;
            pInternalState->adminInfo.drbgSecurityStrength =
                            CPA_CY_RBG_SEC_STRENGTH_256;
        }
        pInternalState->nImplSeedLen =
            pInternalState->nImplKeyLen + LAC_DRBG_OUTLEN_IN_BYTES;

        osalMemCopy(pInternalState->workingState.key,
                    pGenVector->initialState.key,
                    pInternalState->nImplKeyLen);

        osalMemCopy(pInternalState->workingState.v,
                    pGenVector->initialState.v,
                    LAC_DRBG_OUTLEN_IN_BYTES);

        pInternalState->workingState.reseedCounter =
            pGenVector->initialState.reseedCounter;

        opData.sessionHandle = sessionHandle;
        opData.secStrength = pGenVector->requestedStrength;
        opData.predictionResistanceRequired =
            pGenVector->predictionResistanceRequired;
        opData.lengthInBytes = pGenVector->requestedBytesLen;

        opData.additionalInput.dataLenInBytes =
            pGenVector->additionalInputLen;
        opData.additionalInput.pData = pBuffer;

        osalMemCopy(opData.additionalInput.pData,
                    pGenVector->additionalInput,
                    pGenVector->additionalInputLen);

        randomBytes.pData = pBuffer + LAC_DRBG_MAX_SEEDLEN_IN_BYTES;
        randomBytes.dataLenInBytes = LAC_DRBG_OUTLEN_IN_BYTES;

        status = cpaCyDrbgGen(instanceHandle_in,
                    NULL,
                    &opData,
                    &randomBytes);

        if(status != pGenVector->expectedStatus)
        {
            status = CPA_STATUS_FAIL;
            break;
        }
        else
        {
            status = CPA_STATUS_SUCCESS;
        }

        if( CPA_STATUS_SUCCESS == pGenVector->expectedStatus )
        {
            if( 0 != memcmp(randomBytes.pData,
                       pGenVector->expectedBytes,
                       pGenVector->requestedBytesLen) )
            {
                status = CPA_STATUS_FAIL;
                break;
            }

            if( 0 != memcmp(pInternalState->workingState.key,
                       pGenVector->expectedState.key,
                       pInternalState->nImplKeyLen) )
            {
                status = CPA_STATUS_FAIL;
                break;
            }

            if( 0 != memcmp(pInternalState->workingState.v,
                       pGenVector->expectedState.v,
                       LAC_DRBG_OUTLEN_IN_BYTES) )
            {
                status = CPA_STATUS_FAIL;
                break;
            }

            if( pInternalState->workingState.reseedCounter !=
                    pGenVector->expectedState.reseedCounter )
            {
                status = CPA_STATUS_FAIL;
                break;
            }
        }
    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Generate health test function - error handling
 *
 * @description
 *      This function tests cpaCyDrbgGen function for health - error handling
 *      scenarios.
 *
 * @param[in]  instanceHandle_in      Instance Handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Generate function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTGenerateErrorHandling(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa8U *pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    CpaCyDrbgSessionSetupData setupData;
    CpaCyDrbgGenOpData opData;
    CpaFlatBuffer randomBytes;

    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;

    Cpa32U seedLen = 0;

    /* Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqTrue;

    /* Select the instantiation vector */
    pInstVector = LacDrbg_HTGetInstantiateKATVector(
                        CPA_CY_RBG_SEC_STRENGTH_128,
                        CPA_TRUE,
                        CPA_TRUE);


    LAC_ENSURE( NULL != pInstVector, "No vector for instantiate" );
    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pInstVector);

    /* Init DRBG session (only to set up symmetric sessions) */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_TRUE;
    setupData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;

    status = LacDrbg_InitSession(instanceHandle_in,
                NULL,
                NULL,
                &setupData,
                sessionHandle,
                &seedLen,
                CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Failed to initialize DRBG session");
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    pInternalState->pGetEntropyInputFunc = LacDrbg_TestGenGetEntropyInput;
    pInternalState->pGetNonceFunc = LacDrbg_TestGenGetNonce;
    pInternalState->isTestRun = CPA_TRUE;

    opData.sessionHandle = sessionHandle;
    opData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;
    opData.predictionResistanceRequired = CPA_FALSE;
    opData.lengthInBytes = LAC_DRBG_OUTLEN_IN_BYTES;
    opData.additionalInput.dataLenInBytes = 0;
    opData.additionalInput.pData = NULL;

    /* Testing proper handling of NULL parameters */
    status = cpaCyDrbgGen(instanceHandle_in,
                    NULL,
                    &opData,
                    NULL);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        LAC_LOG_ERROR("NULL parameter not handled correctly");
        return CPA_STATUS_FAIL;
    }

    status = cpaCyDrbgGen(instanceHandle_in,
                    NULL,
                    NULL,
                    &randomBytes);
    if( CPA_STATUS_INVALID_PARAM != status )
    {
        LAC_LOG_ERROR("NULL parameter not handled correctly");
        return CPA_STATUS_FAIL;
    }


    opData.sessionHandle = NULL;
    status = cpaCyDrbgGen(instanceHandle_in,
                        NULL,
                        &opData,
                        &randomBytes);
    if( CPA_STATUS_INVALID_PARAM != status )
    {
        LAC_LOG_ERROR("NULL parameter not handled correctly");
        return CPA_STATUS_FAIL;
    }
    opData.sessionHandle = sessionHandle;

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Generate health test function
 *
 * @description
 *      This function tests cpaCyDrbgGen function for health
 *
 * @param[in]  instanceHandle_in      Instance Handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Generate function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTGenerate(const CpaInstanceHandle instanceHandle_in,
                   CpaCyDrbgSessionHandle sessionHandle,
                   Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle;
    sal_crypto_service_t* pService = NULL;

    status = LacDrbg_HTGenerateUseDF(instanceHandle_in,
                 sessionHandle,
                 CPA_FALSE,
                 pBuffer);

    if( CPA_STATUS_SUCCESS == status )
    {
        status = LacDrbg_HTGenerateUseDF(instanceHandle_in,
                    sessionHandle,
                    CPA_TRUE,
                    pBuffer);
    }
    if( CPA_STATUS_SUCCESS == status )
    {
        status = LacDrbg_HTGenerateErrorHandling(instanceHandle_in,
                    sessionHandle,
                    pBuffer);
    }

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Get the 'real' instance handle */
        if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
            instanceHandle = Lac_GetFirstHandle();
        }else{
            instanceHandle = instanceHandle_in;
        }

        pService = (sal_crypto_service_t *)instanceHandle;

        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
    }
    else
    {
        status = LacDrbg_HTRemoveSession(instanceHandle_in, sessionHandle);
    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Reseed health test function
 *
 * @description
 *      This function tests cpaCyDrbgReseed function for health. Whether or not
 *      to use derivation function is indicated by bUseDF parameter.
 *
 * @param[in]  instanceHandle_in      Instance Handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  bUseDF                 Flag indicating whether derivation
 *                                    function is to be used during testing
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Reseed function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTReseedUseDF(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        CpaBoolean bUseDF,
        Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    CpaCyDrbgSessionSetupData setupData;
    CpaCyDrbgSecStrength secStrength;
    CpaCyDrbgReseedOpData opData;

    lac_drbg_ht_reseed_vector_t *pVectorList = NULL;
    lac_drbg_ht_reseed_vector_t *pReseedVector = NULL;
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;
    Cpa32U numVectors = 0;

    Cpa32U seedLen = 0;
    Cpa32U index = 0;

    lac_drbg_internal_state_t *pInternalState = NULL;

    if(bUseDF)
    {
        secStrength = CPA_CY_RBG_SEC_STRENGTH_128;
        pVectorList = reseedDFVectors;
        numVectors = LAC_DRBG_HT_NUM_RESEED_DF_VECTORS;
    }
    else
    {
        secStrength = CPA_CY_RBG_SEC_STRENGTH_256;
        pVectorList = reseedNoDFVectors;
        numVectors = LAC_DRBG_HT_NUM_RESEED_NODF_VECTORS;
    }

    /* Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    if(CPA_TRUE == bUseDF)
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqTrue;
    }
    else
    {
        pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqFalse;
    }

    /* Select the instantiation vector */
    pInstVector = LacDrbg_HTGetInstantiateKATVector(
                    secStrength,
                    CPA_TRUE,
                    bUseDF);

    LAC_ENSURE(pInstVector != NULL, "No vector for instantiate");
    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pInstVector);

    /* Init DRBG session (only to set up symmetric sessions) */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_TRUE;
    setupData.secStrength = secStrength;

    status = LacDrbg_InitSession(instanceHandle_in,
                NULL,
                NULL,
                &setupData,
                sessionHandle,
                &seedLen,
                CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Failed to initialize DRBG session");
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    pInternalState->pGetEntropyInputFunc = LacDrbg_TestReseedGetEntropyInput;
    pInternalState->pGetNonceFunc = LacDrbg_TestGenGetNonce;
    pInternalState->isTestRun = CPA_TRUE;

    for(index = 0; index < numVectors; index++)
    {
        pReseedVector = pVectorList + index;
        LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pReseedVector);

        pInternalState->adminInfo.predictionResistanceRequired =
            pReseedVector->predictionResistanceEnabled;
        pInternalState->isDFRequired = pReseedVector->isDFUsed;

        if( CPA_TRUE == pInternalState->isDFRequired )
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN128_IN_BYTES;
        }
        else
        {
            pInternalState->nImplKeyLen = LAC_DRBG_KEYLEN256_IN_BYTES;
        }

        pInternalState->adminInfo.drbgSecurityStrength =
                        pReseedVector->setupStrength;

        pInternalState->nImplSeedLen =
            pInternalState->nImplKeyLen + LAC_DRBG_OUTLEN_IN_BYTES;

        osalMemCopy(pInternalState->workingState.key,
                    pReseedVector->initialState.key,
                    pInternalState->nImplKeyLen);

        osalMemCopy(pInternalState->workingState.v,
                    pReseedVector->initialState.v,
                    LAC_DRBG_OUTLEN_IN_BYTES);

        pInternalState->workingState.reseedCounter =
            pReseedVector->initialState.reseedCounter;

        opData.sessionHandle = sessionHandle;

        opData.additionalInput.dataLenInBytes =
            pReseedVector->additionalInputLen;
        opData.additionalInput.pData = pBuffer;

        osalMemCopy(opData.additionalInput.pData,
                    pReseedVector->additionalInput,
                    pReseedVector->additionalInputLen);

        status = cpaCyDrbgReseed(instanceHandle_in,
                     NULL,
                     &opData);

        if(status != pReseedVector->expectedStatus)
        {
            status = CPA_STATUS_FAIL;
            break;
        }
        else
        {
            status = CPA_STATUS_SUCCESS;
        }

        if(CPA_STATUS_SUCCESS == pReseedVector->expectedStatus )
        {
            if( 0 != memcmp(pInternalState->workingState.key,
                       pReseedVector->expectedState.key,
                       pInternalState->nImplKeyLen) )
            {
                status = CPA_STATUS_FAIL;
                break;
            }

            if( 0 != memcmp(pInternalState->workingState.v,
                       pReseedVector->expectedState.v,
                       LAC_DRBG_OUTLEN_IN_BYTES) )
            {
                status = CPA_STATUS_FAIL;
                break;
            }

            if( pInternalState->workingState.reseedCounter !=
                    pReseedVector->expectedState.reseedCounter )
            {
                status = CPA_STATUS_FAIL;
                break;
            }
        }
    }

    return status;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Reseed health test function - error handling
 *
 * @description
 *      This function tests cpaCyDrbgReseed function for health
 *      - error handling scenarios.
 *
 * @param[in]  instanceHandle_in      Instance handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Reseed function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTReseedErrorHandling(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle;
    sal_crypto_service_t* pService = NULL;

    CpaCyDrbgSessionSetupData setupData;
    CpaCyDrbgReseedOpData opData;

    lac_drbg_internal_state_t *pInternalState = NULL;
    lac_drbg_ht_instantiate_vector_t *pInstVector = NULL;
    Cpa32U seedLen = 0;

    /* Set the 'test' implementation functions */
    pTestGetEntropyInputFunc = LacDrbg_TestInstGetEntropyInputFunc;
    pTestGetNonceFunc = LacDrbg_TestInstGetNonce;
    pTestIsDFReqFunc = LacDrbg_TestInstIsDFReqTrue;

    /* Select the instantiation vector */
    pInstVector = LacDrbg_HTGetInstantiateKATVector(
                        CPA_CY_RBG_SEC_STRENGTH_128,
                        CPA_TRUE,
                        CPA_TRUE);

    if( NULL == pInstVector )
    {
        LAC_LOG_ERROR("No vector for instantiate!");
        return CPA_STATUS_FAIL;
    }
    LAC_DRBG_HT_SET_VECTOR_POINTER(sessionHandle, pInstVector);

    /* Init DRBG session (only to set up symmetric sessions) */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_TRUE;
    setupData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;

    status = LacDrbg_InitSession(instanceHandle_in,
                NULL,
                NULL,
                &setupData,
                sessionHandle,
                &seedLen,
                CPA_TRUE);

    if( CPA_STATUS_SUCCESS != status )
    {
        LAC_LOG_ERROR("Failed to initialize DRBG session");
        return CPA_STATUS_FAIL;
    }

    pInternalState = LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(sessionHandle);
    pInternalState->pGetEntropyInputFunc = LacDrbg_TestGenGetEntropyInput;
    pInternalState->pGetNonceFunc = LacDrbg_TestGenGetNonce;
    pInternalState->isTestRun = CPA_TRUE;

    opData.sessionHandle = sessionHandle;
    opData.additionalInput.dataLenInBytes = 0;
    opData.additionalInput.pData = NULL;

    /* Testing proper handling of NULL parameters */
    status = cpaCyDrbgReseed(instanceHandle_in,
                    NULL,
                    NULL);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        LAC_LOG_ERROR("NULL parameter not handled correctly");
        return CPA_STATUS_FAIL;
    }

    opData.sessionHandle = NULL;
    status = cpaCyDrbgReseed(instanceHandle_in,
                        NULL,
                        &opData);

    if( CPA_STATUS_INVALID_PARAM != status )
    {
        LAC_LOG_ERROR("NULL parameter not handled correctly");
        return CPA_STATUS_FAIL;
    }
    opData.sessionHandle = sessionHandle;

    /* Testing error handling while obtaining entropy input */
    pInternalState->pGetEntropyInputFunc =
        LacDrbg_TestGetEntropyInputFailFunc;

    status = cpaCyDrbgReseed(instanceHandle_in,
                        NULL,
                        &opData);
    if( CPA_STATUS_FAIL != status )
    {
        LAC_LOG_ERROR("Get Entropy Input error not handled correctly");
        return CPA_STATUS_FAIL;
    }
    else
    {
        /* Get the 'real' instance handle */
        if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
            instanceHandle = Lac_GetFirstHandle();
        }else{
            instanceHandle = instanceHandle_in;
        }

        pService = (sal_crypto_service_t *)instanceHandle;

        if( LAC_DRBG_ERROR_STATE != osalAtomicGet(&pService->drbgErrorState) )
        {
            LAC_LOG_ERROR("Error state not set");
            return CPA_STATUS_FAIL;
        }

        osalAtomicSet(0, &pService->drbgErrorState);
    }

    return CPA_STATUS_SUCCESS;
}

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgHT
 *      DRBG Reseed health test function
 *
 * @description
 *      This function tests cpaCyDrbgReseed function for health
 *
 * @param[in]  instanceHandle_in      Instance handle
 * @param[in]  sessionHandle          DRBG session handle
 * @param[in]  pBuffer                Pointer to a buffer that is to be used to
 *                                    pass additional input parameter to
 *                                    Reseed function. It must point to
 *                                    user allocated physically contiguous
 *                                    memory and is supposed to store
 *                                    MAX_SEEDLEN + OUTLEN bytes max.
 *
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_FAIL            Health test failed
 *
 *****************************************************************************/
STATIC CpaStatus
LacDrbg_HTReseed(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa8U* pBuffer)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle;
    sal_crypto_service_t* pService = NULL;

    status = LacDrbg_HTReseedUseDF(instanceHandle_in,
                 sessionHandle,
                 CPA_TRUE,
                 pBuffer);

    if( CPA_STATUS_SUCCESS == status )
    {
        status = LacDrbg_HTReseedUseDF(instanceHandle_in,
                    sessionHandle,
                    CPA_FALSE,
                    pBuffer);

    }
    if( CPA_STATUS_SUCCESS == status )
    {
        status = LacDrbg_HTReseedErrorHandling(instanceHandle_in,
                    sessionHandle,
                    pBuffer);
    }

    if( CPA_STATUS_SUCCESS != status )
    {
        /* Get the 'real' instance handle */
        if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in){
            instanceHandle = Lac_GetFirstHandle();
        }else{
            instanceHandle = instanceHandle_in;
        }

        pService = (sal_crypto_service_t *)instanceHandle;

        osalAtomicSet(LAC_DRBG_ERROR_STATE, &pService->drbgErrorState);
    }
    else
    {
        status = LacDrbg_HTRemoveSession(instanceHandle_in, sessionHandle);
    }

    return status;
}

/**
 * @ingroup icp_sal
 */
CpaStatus
icp_sal_drbgHTGetTestSessionSize(CpaInstanceHandle instanceHandle_in,
                                 Cpa32U *pTestSessionSize)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U size = 0;
    CpaCyDrbgSessionSetupData setupData;
    CpaInstanceHandle instanceHandle = instanceHandle_in;

    LAC_DRBG_CHECK_NULL_PARAM(pTestSessionSize);

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        LAC_LOG_ERROR("Random HT is not supported for this instance!");
        return CPA_STATUS_UNSUPPORTED;
    }

    /* This is for initialization only. Values are not used during
     * cpaCyDrbgSessionGetSize call but need to be validated  */
    setupData.personalizationString.dataLenInBytes = 0;
    setupData.personalizationString.pData = NULL;
    setupData.predictionResistanceRequired = CPA_FALSE;
    setupData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;

    status = cpaCyDrbgSessionGetSize(instanceHandle, &setupData, &size);

    if( CPA_STATUS_SUCCESS == status )
    {
        /* If the current setup indicates that Derivation Function is not
         * required add additional size for the DF case */
        if( CPA_FALSE == pIsDFReqFunc() )
        {
            size += LAC_SYM_SESSION_SIZE;
        }

        /* Add space required to fit personalization string / additional input
         * and requested bits from vectors  */
        size += LAC_DRBG_MAX_SEEDLEN_IN_BYTES;
        size += LAC_DRBG_OUTLEN_IN_BYTES;

        *pTestSessionSize = size;
    }

    return status;
}

/**
 * @ingroup icp_sal
 */
CpaStatus
icp_sal_drbgHTInstantiate(const CpaInstanceHandle instanceHandle_in,
                            IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U index = 0;
    lac_drbg_ht_instantiate_vector_t *pVector;
    Cpa8U *pBuffer = NULL;
    CpaCyDrbgSessionHandle sessionHandle = NULL;
    CpaInstanceHandle instanceHandle = instanceHandle_in;

    LAC_DRBG_CHECK_NULL_PARAM(testSessionHandle);

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        LAC_LOG_ERROR("Random HT is not supported for this instance!");
        return CPA_STATUS_UNSUPPORTED;
    }

    pBuffer = (Cpa8U*)testSessionHandle;
    /* Moving by MAX_SEEDLEN only as this is what is only needed for
     * instantiate tests  */
    sessionHandle = (CpaCyDrbgSessionHandle)
        (pBuffer + LAC_DRBG_MAX_SEEDLEN_IN_BYTES);

    /* First sizeof(void*) bytes of sessionHandle is reserve for a pointer
     * to a vector used while health testing. Adjust session handle to skip
     * these bytes */
    sessionHandle = (CpaCyDrbgSessionHandle)
                            ((Cpa8U*)sessionHandle + sizeof(void*));

    for( index = 0; index < LAC_DRBG_HT_NUM_INSTANTIATE_VECTORS; index++ )
    {
        pVector = instantiateVectors + index;
        status = LacDrbg_HTInitSessionVector(instanceHandle,
                    sessionHandle,
                    pVector,
                    pBuffer);

        if(status != CPA_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

/**
 * @ingroup icp_sal
 */
CpaStatus
icp_sal_drbgHTGenerate(const CpaInstanceHandle instanceHandle_in,
                        IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pBuffer = NULL;
    CpaCyDrbgSessionHandle sessionHandle = NULL;
    CpaInstanceHandle instanceHandle = instanceHandle_in;

    LAC_CHECK_NULL_PARAM(testSessionHandle);

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        LAC_LOG_ERROR("Random HT is not supported for this instance!");
        return CPA_STATUS_UNSUPPORTED;
    }

    pBuffer = (Cpa8U*)testSessionHandle;
    sessionHandle = (CpaCyDrbgSessionHandle)
        (pBuffer + LAC_DRBG_MAX_SEEDLEN_IN_BYTES + LAC_DRBG_OUTLEN_IN_BYTES);

    /* First sizeof(void*) bytes of sessionHandle is reserve for a pointer
     * to a vector used while health testing. Adjust session handle to skip
     * these bytes */
    sessionHandle = (CpaCyDrbgSessionHandle)
                            ((Cpa8U*)sessionHandle + sizeof(void*));

    status = LacDrbg_HTGenerate(instanceHandle, sessionHandle, pBuffer);

    return status;
}

/**
 * @ingroup icp_sal
 */
CpaStatus
icp_sal_drbgHTReseed(const CpaInstanceHandle instanceHandle_in,
                        IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pBuffer = NULL;
    CpaCyDrbgSessionHandle sessionHandle = NULL;
    CpaInstanceHandle instanceHandle = instanceHandle_in;

    LAC_DRBG_CHECK_NULL_PARAM(testSessionHandle);

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
        instanceHandle = Lac_GetFirstHandle();
    }
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);

    if(CPA_TRUE != LacDrbg_CheckRandomCapability(instanceHandle))
    {
        LAC_LOG_ERROR("Random HT is not supported for this instance!");
        return CPA_STATUS_UNSUPPORTED;
    }

    pBuffer = (Cpa8U*)testSessionHandle;
    /* Moving by MAX_SEEDLEN only as this is what is only needed for
     * reseed tests  */
    sessionHandle = (CpaCyDrbgSessionHandle)
        (pBuffer + LAC_DRBG_MAX_SEEDLEN_IN_BYTES);

    /* First sizeof(void*) bytes of sessionHandle is reserve for a pointer
     * to a vector used while health testing. Adjust session handle to skip
     * these bytes */
    sessionHandle = (CpaCyDrbgSessionHandle)
                            ((Cpa8U*)sessionHandle + sizeof(void*));

    status = LacDrbg_HTReseed(instanceHandle, sessionHandle, pBuffer);

    return status;
}

