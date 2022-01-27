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
 * @file lac_sym_drbg_rdrand.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *     Implementation of the Deterministic Random Bit Generation using
 *     rdrand instruction
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
#include "lac_sym_drbg.h"
#include "lac_sym_drbg_rdrand.h"
#include "lac_sal_types.h"

/**
 * @ingroup LacSym_DrbgRdrand
 */
CpaStatus
LacDrbg_RdrandSessionGetSize(const CpaInstanceHandle instanceHandle,
        const CpaCyDrbgSessionSetupData *pSetupData,
        Cpa32U *pSize)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    LAC_CHECK_NULL_PARAM(pSetupData);
    LAC_CHECK_NULL_PARAM(pSize);
#endif

    /* Check for rdrand instruction support */
    status  = LacDrbg_RdrandIsSupported();
    LAC_CHECK_STATUS(status);

    *pSize = sizeof(lac_drbg_internal_state_t);

    /* The pointer to the data is stored at the start of the
     * user allocated memory hence the extra space for an LAC_ARCH_UINT */
    *pSize +=  sizeof(LAC_ARCH_UINT);

    return CPA_STATUS_SUCCESS;
}

/**
 * @ingroup LacSym_DrbgRdrand
 */
CpaStatus
LacDrbg_RdrandInitSession(const CpaInstanceHandle instanceHandle,
        const CpaCyGenFlatBufCbFunc pGenCb,
        const CpaCyGenericCbFunc pReseedCb,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa32U* pSeedLen)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t* pService = NULL;
#endif

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    LAC_CHECK_NULL_PARAM(pSetupData);
    LAC_CHECK_NULL_PARAM(sessionHandle);
    LAC_CHECK_NULL_PARAM(pSeedLen);
#endif

    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);
    SAL_RUNNING_CHECK(instanceHandle);

    /* Check for rdrand instruction support */
    status = LacDrbg_RdrandIsSupported();
    LAC_CHECK_STATUS(status);

#ifndef DISABLE_STATS
    pService = (sal_crypto_service_t*) instanceHandle;
#endif

    pInternalState = (lac_drbg_internal_state_t *)
                     ((Cpa8U *)sessionHandle + sizeof(LAC_ARCH_UINT));
    LAC_OS_BZERO(pInternalState, sizeof(lac_drbg_internal_state_t));

    /* save the internal state pointer in the first bytes
     * (size of unsigned long) of the session memory */
    *((LAC_ARCH_UINT *)sessionHandle) = (LAC_ARCH_UINT)pInternalState;

    /* Store the instanceHandle in the internal state */
    pInternalState->instanceHandle = instanceHandle;

    if(0 != pSetupData->personalizationString.dataLenInBytes
            || NULL != pSetupData->personalizationString.pData)
    {
        LAC_INVALID_PARAM_LOG("PersonalizationString is not "
            "supported for rdrand");
        status = CPA_STATUS_INVALID_PARAM;
    }

    if(CPA_TRUE == pSetupData->predictionResistanceRequired
            && CPA_STATUS_SUCCESS == status)
    {
        LAC_INVALID_PARAM_LOG("PredictionResistance is not supported "
            "for rdrand");
        status = CPA_STATUS_INVALID_PARAM;
    }

    if((pSetupData->secStrength < CPA_CY_RBG_SEC_STRENGTH_112
            || pSetupData->secStrength > CPA_CY_RBG_SEC_STRENGTH_128)
            && CPA_STATUS_SUCCESS == status)
    {
        LAC_INVALID_PARAM_LOG("Invalid secStrength. "
            "Supported secStrengths for rdrand are 112 and 128");
        status = CPA_STATUS_INVALID_PARAM;
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        /* Set the callback */
        pInternalState->pGenCb = pGenCb;

        /* Return the seed length */
        *pSeedLen = LAC_DRBG_RDRAND_SEEDLEN;

#ifndef DISABLE_STATS
        LAC_DRBG_RDRAND_STAT_INC(numSessionsInitialized, pService);
#endif
    }
    else
    {
#ifndef DISABLE_STATS
        LAC_DRBG_RDRAND_STAT_INC(numSessionErrors, pService);
#endif
    }
    return status;
}

/**
 * @ingroup LacSym_DrbgRdrand
 */
CpaStatus
LacDrbg_RdrandGen(const CpaInstanceHandle instanceHandle,
        void *pCallbackTag,
        CpaCyDrbgGenOpData *pOpData,
        CpaFlatBuffer *pPseudoRandomBits)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_drbg_internal_state_t *pInternalState = NULL;
#ifndef DISABLE_STATS
    sal_crypto_service_t* pService = NULL;
#endif

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pOpData->sessionHandle);
    LAC_CHECK_NULL_PARAM(pPseudoRandomBits);
    LAC_CHECK_NULL_PARAM(pPseudoRandomBits->pData);
#endif

    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    /* Check for rdrand instruction support */
    status = LacDrbg_RdrandIsSupported();
    LAC_CHECK_STATUS(status);

    if((pPseudoRandomBits->dataLenInBytes < pOpData->lengthInBytes)
        || (pOpData->lengthInBytes > LAC_DRBG_RDRAND_MAX_NUM_OF_BYTES ))
    {
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Additional Input not supported by RdRand */
    if( 0 != pOpData->additionalInput.dataLenInBytes ||
        NULL != pOpData->additionalInput.pData )
    {
        LAC_INVALID_PARAM_LOG("AdditionalInput is not supported for rdrand");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* predictionResistance not supported by RdRand */
    if(CPA_TRUE == pOpData->predictionResistanceRequired)
    {
        LAC_INVALID_PARAM_LOG("PredictionResistance is not supported "
            "for rdrand");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(pOpData->secStrength < CPA_CY_RBG_SEC_STRENGTH_112
            || pOpData->secStrength > CPA_CY_RBG_SEC_STRENGTH_128)
    {
        LAC_INVALID_PARAM_LOG("Invalid secStrength. "
            "Supported secStrengths for rdrand are 112 and 128");
        return CPA_STATUS_INVALID_PARAM;
    }

#ifndef DISABLE_STATS
    pService = (sal_crypto_service_t *)instanceHandle;
#endif

    pInternalState =
        LAC_DRBG_INTERNAL_STATE_FROM_HANDLE_GET(pOpData->sessionHandle);
    if(NULL == pInternalState)
    {
        LAC_LOG_ERROR("Internal state pointer is NULL");
        return CPA_STATUS_FAIL;
    }

    status = LacDrbg_RdrandGetBytes(pOpData->lengthInBytes,
                 pPseudoRandomBits->pData );

    if(CPA_STATUS_SUCCESS == status)
    {
#ifndef DISABLE_STATS
        LAC_DRBG_RDRAND_STAT_INC(numGenCompleted, pService);
#endif
        if(NULL!= pInternalState->pGenCb)
        {
            pInternalState->pGenCb(pCallbackTag,
                status,
                pOpData,
                pPseudoRandomBits);
        }
    }
    else
    {
#ifndef DISABLE_STATS
        LAC_DRBG_RDRAND_STAT_INC(numGenCompletedErrors, pService);
#endif
    }

    return status;
}

/**
 * @ingroup LacSym_DrbgRdrand
 */
CpaStatus
LacDrbg_RdrandRemoveSession(const CpaInstanceHandle instanceHandle,
        CpaCyDrbgSessionHandle sessionHandle)
{
#ifndef DISABLE_STATS
    sal_crypto_service_t* pService = NULL;
#endif

    SAL_CHECK_INSTANCE_TYPE(instanceHandle, SAL_SERVICE_TYPE_CRYPTO);

    /* check if LAC is initialised otherwise return an error */
    SAL_RUNNING_CHECK(instanceHandle);

    LAC_CHECK_NULL_PARAM(sessionHandle);

#ifndef DISABLE_STATS
    pService = (sal_crypto_service_t*) instanceHandle;
#endif

    LAC_OS_BZERO(sessionHandle, sizeof(lac_drbg_internal_state_t)
                              + sizeof(LAC_ARCH_UINT));

#ifndef DISABLE_STATS
    LAC_DRBG_RDRAND_STAT_INC(numSessionsRemoved, pService);
#endif
    return CPA_STATUS_SUCCESS;
}
