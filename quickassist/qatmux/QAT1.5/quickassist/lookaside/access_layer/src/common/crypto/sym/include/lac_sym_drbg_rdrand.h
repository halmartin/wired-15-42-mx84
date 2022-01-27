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
 * @file lac_sym_drbg_rdrand.h
 *
 * @defgroup LacSym_DrbgRdRand
 *
 * @ingroup LacSym
 *
 * @description
 *     Interfaces for DRBG implementation via rdrand
 *
 *****************************************************************************/
#define LAC_DRBG_RDRAND_MAX_NUM_OF_BYTES (0xFFFF)
/**< @ingroup LacSym_DrbgRdrand
 * Maximum number of bytes that might be requested in
 * a single Generate operation, as described in NIST SP 800-90,
 * section 10.2.1, Table 3 */

#define LAC_DRBG_RDRAND_SEEDLEN 256
/**< @ingroup LacSym_DrbgRdrand
 * Length of the seed in bits when using rdrand
 */

#ifndef DISABLE_STATS
#define LAC_DRBG_RDRAND_STAT_INC(statistic, pService)                    \
    do{                                                                  \
        if(CPA_TRUE ==                                                   \
                pService->generic_service_info.stats->bDrbgStatsEnabled) \
        {                                                                \
            osalAtomicInc(&pService->pLacDrbgStatsArr[                   \
                               offsetof(CpaCyDrbgStats64, statistic)     \
                               / sizeof(Cpa64U) ]);                      \
        }                                                                \
    }while(0)
#else
#define LAC_DRBG_RDRAND_STAT_INC(statistic, pService)
#endif
/**< @ingroup LacSym_Drbg_RdRand
 * Statistics incrementation macro.
 */

/* FUNCTIONS */
/**
 *****************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Returns the size (in bytes) of a DRBG session handle.
 *
 * @description
 *      This function is used by the client to determine the size of the
 *      memory it must allocate in order to store the DRBG session. This MUST
 *      be called before the client allocates the memory for the session
 *      and before the client calls the @ref cpaCyDrbgInitSession function.
 *
 * @param[in]  instanceHandle        Instance handle.
 * @param[in]  pSetupData            Pointer to session setup data which
 *                                   contains parameters which are static
 *                                   for a given DRBG session, such
 *                                   as security strength, etc.
 * @param[out] pSize                 The amount of memory in bytes required
 *                                   to hold the session.
 *
 * @return CPA_STATUS_SUCCESS        Function executed successfully.
 * @return CPA_STATUS_FAIL           Function failed.
 * @return CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @return CPA_STATUS_UNSUPPORTED    Error related to RdRand support.
 *
 *****************************************************************************/
CpaStatus
LacDrbg_RdrandSessionGetSize(const CpaInstanceHandle instanceHandle,
        const CpaCyDrbgSessionSetupData *pSetupData,
        Cpa32U *pSize);

/**
 ******************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Rdrand implementation of DRBG Init Session / Instantiate
 *
 * @description
 *      This function implements DRBG Init Session / Instantiate process. API
 *      function cpaCyDrbgInitSession, calls this function for initializing
 *      an rdrand based DRBG session
 *
 * @param[in]  instanceHandle         Instance handle.
 * @param[in]  pGenCb                 User callback for Generate operation. If
 *                                    this is set, cpaCyDrbgGen will operate
 *                                    asynchronously; if the pointer is NULL
 *                                    all calls to  cpaCyDrbgGen will be
 *                                    synchronous
 * @param[in]  pReseedCb              pReseed call back is not supported and
 *                                    will be ignored
 * @param[in]  pSetupData             Pointer to session setup data
 * @param[in]  sessionHandle          DRBG session handle
 * @param[out] pSeedLen               Default value for this implementation
 *                                    is 256
 * @return CPA_STATUS_SUCCESS         Operation successful
 * @return CPA_STATUS_INVALID_PARAM   Invalid parameter provided
 * @return CPA_STATUS_FAIL            Operation failed
 * @return CPA_STATUS_UNSUPPORTED     Error related to RdRand support.
 *
 *****************************************************************************/
CpaStatus
LacDrbg_RdrandInitSession(const CpaInstanceHandle instanceHandle,
        const CpaCyGenFlatBufCbFunc pGenCb,
        const CpaCyGenericCbFunc pReseedCb,
        const CpaCyDrbgSessionSetupData *pSetupData,
        CpaCyDrbgSessionHandle sessionHandle,
        Cpa32U* pSeedLen);

/**
 *****************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Generates pseudorandom bits.
 *
 * @description
 *      This function is used to request the generation of random bits.
 *      The generated data and the length of the data will be
 *      returned to the caller in an asynchronous callback function.
 *
 * @param[in]  instanceHandle        Instance handle.
 * @param[in]  pCallbackTag          Opaque User Data for this specific call.
 *                                   Will be returned unchanged in the callback.
 * @param[in]  pOpData               Structure containing all the data needed
 *                                   to perform the operation. The client code
 *                                   allocates the memory for this structure.
 *                                   This component takes ownership of the
 *                                   memory until it is returned in the
 *                                   callback.
 * @param[out] pPseudoRandomBits     Pointer to the memory allocated by the
 *                                   client where the random data will be
 *                                   written to. For optimal performance, the
 *                                   data pointed to SHOULD be 8-byte aligned.
 *                                   There is no endianness associated with
 *                                   the random data.On invocation the callback
 *                                   function will contain this parameter in
 *                                   its pOut parameter.
 *
 * @return CPA_STATUS_SUCCESS        Function executed successfully.
 * @return CPA_STATUS_FAIL           Function failed.
 * @return CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 * @return CPA_STATUS_UNSUPPORTED    Error related to RdRand support.
 *
 ******************************************************************************/
CpaStatus
LacDrbg_RdrandGen(const CpaInstanceHandle instanceHandle,
        void *pCallbackTag,
        CpaCyDrbgGenOpData *pOpData,
        CpaFlatBuffer *pPseudoRandomBits);

/**
 *****************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Removes a previously instantiated DRBG session, or instance.
 *
 * @description
 *      This function will remove a previously initialized DRBG session,
 *      or instance, and the installed callback handler function.
 *
 * @param[in]  instanceHandle       Instance handle.
 * @param[in]  sessionHandle        DRBG session handle to be removed.
 *
 * @return CPA_STATUS_SUCCESS       Function executed successfully.
 * @return CPA_STATUS_FAIL          Function failed.
 * @return CPA_STATUS_INVALID_PARAM Invalid parameter passed in.
 *
 *****************************************************************************/
CpaStatus
LacDrbg_RdrandRemoveSession(const CpaInstanceHandle instanceHandle_in,
        CpaCyDrbgSessionHandle sessionHandle);

/**
 *****************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Determines hardware support for rdrand execution.
 *
 * @description
 *      This function will query the cpuid and determine support for rdrand
 *
 * @return CPA_STATUS_SUCCESS       Function executed successfully.
 * @return CAP_STATUS_UNSUPPORTED   Operation is not supported
 *
 *****************************************************************************/
CpaStatus
LacDrbg_RdrandIsSupported(void);

/**
 *****************************************************************************
 * @ingroup LacSym_DrbgRdrand
 *      Generates pseudorandom bits via rdrand instruction
 *
 * @description
 *      This function is used to request the generation of random bits.
 *      The generated data and the length of the data will be returned
 *      to the caller in synchronous manner.
 *
 * @return CPA_STATUS_SUCCESS       Function executed successfully.
 * @return CPA_STATUS_FAIL          Function failed.
 *
 *****************************************************************************/
CpaStatus
LacDrbg_RdrandGetBytes(Cpa32U lengthInBytes, Cpa8U *pData);
