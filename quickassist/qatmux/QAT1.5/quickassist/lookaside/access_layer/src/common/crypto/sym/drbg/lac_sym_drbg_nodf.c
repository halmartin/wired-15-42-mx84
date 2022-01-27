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
 * @file lac_sym_drbg_nodf.c
 *
 * @ingroup LacSym_Drbg
 *
 * @description
 *    Implementation of the Deterministic Random Bit Generation API - functions
 *    to use with entropy sources that do not require derivation function
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
#include "icp_adf_debug.h"
#include "icp_adf_transport.h"
#include "icp_buffer_desc.h"
#include "sal_service_state.h"
#include "lac_sym.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"
#include "lac_sym_drbg.h"

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_GenerateNoDF(sal_crypto_service_t *pService,
                    lac_drbg_internal_state_t * pInternalState,
                    Cpa32U numOfBytesRequested,
                    const CpaFlatBuffer *pAdditionalInput,
                    CpaFlatBuffer *pPseudoRandomBits)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* 1 */
    if(pInternalState->workingState.reseedCounter > LAC_DRBG_SEED_LIFE )
    {
        pInternalState->reseedRequired = CPA_TRUE;
        return CPA_STATUS_RETRY;
    }

    /* 2 */
    if( NULL != pAdditionalInput )
    {
        osalMemCopy(pInternalState->buffers.additionalInput,
                    pAdditionalInput->pData,
                    pAdditionalInput->dataLenInBytes);

        /* Operating in asynchronous mode */
        status = LacDrbg_Update(pService,
                               pInternalState,
                               pInternalState->buffers.additionalInput,
                               LacDrbg_GenerateNextOpAfterUpdateCb,
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

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_ReseedNoDF(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t * pInternalState,
        CpaFlatBuffer *pEntropyInput,
        CpaFlatBuffer *pAdditionalInput,
        LacDrbgNextOpCbFunc pNextOpFunc,
        void *pCallbackTag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U* pSeedMaterial = NULL;
    Cpa32U index = 0;

    /* NIST SP 800-90
     * 1. temp = len(additional_input)
     * 2. If(temp<seedlen), then
     *          additional_input = additional_input || 0^(seedlen-temp)
     * 3. seed_material = entropy_input xor additional_input
     * 4. (Key,v)=Update(seed_material, Key,V)
     * 5. reseed_counter = 1
     * 6. Return V, Key and reseed_counter as the new_working_state
     */

    if(pAdditionalInput->dataLenInBytes < pInternalState->nImplSeedLen )
    {
        LAC_OS_BZERO(pInternalState->buffers.seedMaterial,
                pInternalState->nImplSeedLen);
    }

    if( NULL != pAdditionalInput->pData )
    {
        osalMemCopy(pInternalState->buffers.seedMaterial,
                    pAdditionalInput->pData,
                    pAdditionalInput->dataLenInBytes);
    }

    pSeedMaterial = pInternalState->buffers.seedMaterial;

    for(index = 0; index < pInternalState->nImplSeedLen; index++)
    {
        pSeedMaterial[index] ^= pEntropyInput->pData[index];
    }

    status = LacDrbg_Update(pService,
                           pInternalState,
                           pSeedMaterial,
                           pNextOpFunc,
                           pCallbackTag);

    return status;
}

/**
 * @ingroup LacSym_Drbg
 */
CpaStatus
LacDrbg_InstantiateNoDF(sal_crypto_service_t *pService,
        lac_drbg_internal_state_t * pInternalState,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaFlatBuffer *pEntropyInput)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U* pPersonalizationString = NULL, *pSeedMaterial = NULL;
    Cpa32U index = 0;

    /* NIST SP 800-90 CTR_DRBG_Instantiate_algorithm when
     *     Derivation Function is not used:
     *     1. temp = len(personalization_string)
     *     2. If(temp<seedlen), then personalization_string =
     *              personalization_string || 0^(seedlen - temp)
     *     3. seed_material = entropy_input xor personalization_string
     *     4. Key = 0^keylen
     *     5. V = 0^outlen
     *     6. (Key, V) = Update(seed_material, Key, V)
     *     7. reseed_counter = 1
     *     8. Return V, Key and reseed_counter as the initial_working_state
     */

    pPersonalizationString = pInternalState->buffers.persString;
    pSeedMaterial = pInternalState->buffers.seedMaterial;

    LAC_OS_BZERO(pPersonalizationString, pInternalState->nImplSeedLen);
    if( NULL != pSetupData->personalizationString.pData )
    {
        osalMemCopy(pPersonalizationString,
                    pSetupData->personalizationString.pData,
                    pSetupData->personalizationString.dataLenInBytes);
    }

    for(index = 0; index < pInternalState->nImplSeedLen; index++)
    {
        pSeedMaterial[index] = pEntropyInput->pData[index]
                                            ^ pPersonalizationString[index];
    }

    LAC_OS_BZERO(pInternalState->workingState.v, LAC_DRBG_OUTLEN_IN_BYTES);
    LAC_OS_BZERO(pInternalState->workingState.key,
            pInternalState->nImplKeyLen);

    status = LacDrbg_Update(pService,
                           pInternalState,
                           pSeedMaterial,
                           NULL,
                           NULL);

    if( CPA_STATUS_SUCCESS == status )
    {
        /* Set reseedCounter to 1 as per NIST SP 800-90,
         * section 10.2.1.3.1 (Step 7) */
        pInternalState->workingState.reseedCounter = 1;
    }

    return status;
}
