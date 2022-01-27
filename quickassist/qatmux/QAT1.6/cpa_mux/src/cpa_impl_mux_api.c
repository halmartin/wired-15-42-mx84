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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
 ***************************************************************************
 * @file cpa_mux.c
 *    Implementation of the mux/demux layer common API for mulitple
 *    implementations of the QA API.
 *
 * @ingroup cpaImplMux
 ***************************************************************************/

/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/
#include "cpa.h"
#include "cpa_mux_common.h"

/**
 *cpaCySymDpEnqueueOp:
 */
CpaStatus
cpaCySymDpEnqueueOp (CpaCySymDpOpData * pOpData,
                     const CpaBoolean performOpNow)
{
    if (NULL == pOpData)
    {
        MUX_ERROR("pOpData parameter cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    CPA_MUX_CALL_IMPL_FN(cpaCySymDpEnqueueOp,
                         pOpData->instanceHandle,
                         pOpData, performOpNow);
}

/**
 * cpaCySymDpEnqueueOpBatch:
 */
CpaStatus
cpaCySymDpEnqueueOpBatch (const Cpa32U numberRequests,
                          CpaCySymDpOpData * pOpData[],
                          const CpaBoolean performOpNow)
{
    if ((NULL == pOpData) || (NULL == pOpData[0]))
    {
        MUX_ERROR("pOpData param cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    CPA_MUX_CALL_IMPL_FN(cpaCySymDpEnqueueOpBatch,
                         pOpData[0]->instanceHandle,
                         numberRequests, pOpData, performOpNow);
}

/**
 * cpaDcGenerateHeader:
 */
CpaStatus
cpaDcGenerateHeader (CpaDcSessionHandle pSessionHandle,
                     CpaFlatBuffer * pDestBuff, Cpa32U * count)
{
    /**
     * This helper method is common across implementations, and does not access
     * driver-specific data, so just call the version in the first impl
     * we find.
     */
    CPA_MUX_CALL_IMPL_FN(cpaDcGenerateHeader,
                         CPA_INSTANCE_HANDLE_SINGLE,
                         pSessionHandle, pDestBuff, count);
}


/**
 * cpaDcGenerateFooter:
 */
CpaStatus
cpaDcGenerateFooter (CpaDcSessionHandle pSessionHandle,
                     CpaFlatBuffer * pDestBuff, CpaDcRqResults * pResults)
{
    /**
     * This helper method is common across implementations, and does not access
     * driver-specific data, so just call the version in the first impl
     * we find.
     */
    CPA_MUX_CALL_IMPL_FN(cpaDcGenerateFooter,
                         CPA_INSTANCE_HANDLE_SINGLE,
                         pSessionHandle, pDestBuff, pResults);
}


/**
 * cpaDcDpEnqueueOp:
 */
CpaStatus
cpaDcDpEnqueueOp (CpaDcDpOpData * pOpData, const CpaBoolean performOpNow)
{
    if (NULL == pOpData)
    {
        MUX_ERROR("pOpData parameter cannot be null");
        return CPA_STATUS_INVALID_PARAM;
    }

    CPA_MUX_CALL_IMPL_FN(cpaDcDpEnqueueOp,
                         pOpData->dcInstance,
                         pOpData, performOpNow);
}


/**
 * cpaDcDpEnqueueOpBatch:
 */
CpaStatus
cpaDcDpEnqueueOpBatch (const Cpa32U numberRequests, CpaDcDpOpData * pOpData[],
                       const CpaBoolean performOpNow)
{
    if ((NULL == pOpData) || (NULL == pOpData[0]))
    {
        MUX_ERROR("pOpData param value cannot be NULL");
        return CPA_STATUS_INVALID_PARAM;
    }

    CPA_MUX_CALL_IMPL_FN(cpaDcDpEnqueueOpBatch,
                         pOpData[0]->dcInstance,
                         numberRequests, pOpData, performOpNow);
}
/**
 * ========================================================================
 * The remaining functions take an instanceHandle as the first parameter
 * and all follow the same format.
 * ========================================================================
 */
/**
 * cpaCySymDpRegCbFunc:
 */
CpaStatus
cpaCySymDpRegCbFunc (const CpaInstanceHandle instanceHandle,
                     const CpaCySymDpCbFunc pSymNewCb)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpRegCbFunc,
                         instanceHandle, instanceHandle, pSymNewCb);
}

/**
 * cpaCySymDpSessionCtxGetSize:
 */
CpaStatus
cpaCySymDpSessionCtxGetSize (const CpaInstanceHandle instanceHandle,
                             const CpaCySymSessionSetupData *
                             pSessionSetupData,
                             Cpa32U * pSessionCtxSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpSessionCtxGetSize,
                         instanceHandle, instanceHandle,
                         pSessionSetupData, pSessionCtxSizeInBytes);
}

/**
 * cpaCySymDpSessionCtxGetDynamicSize:
 */
CpaStatus
cpaCySymDpSessionCtxGetDynamicSize (const CpaInstanceHandle instanceHandle,
                             const CpaCySymSessionSetupData *
                             pSessionSetupData,
                             Cpa32U * pSessionCtxSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpSessionCtxGetDynamicSize,
                         instanceHandle, instanceHandle,
                         pSessionSetupData, pSessionCtxSizeInBytes);
}

/**
 * cpaCySymDpInitSession:
 */
CpaStatus
cpaCySymDpInitSession (CpaInstanceHandle instanceHandle,
                       const CpaCySymSessionSetupData * pSessionSetupData,
                       CpaCySymDpSessionCtx sessionCtx)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpInitSession,
                         instanceHandle,instanceHandle,
                         pSessionSetupData, sessionCtx);
}

/**
 * cpaCySymDpRemoveSession:
 */
CpaStatus
cpaCySymDpRemoveSession (const CpaInstanceHandle instanceHandle,
                         CpaCySymDpSessionCtx sessionCtx)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpRemoveSession,
                         instanceHandle, instanceHandle, sessionCtx);
}

/**
 * cpaCySymDpPerformOpNow:
 */
CpaStatus
cpaCySymDpPerformOpNow (CpaInstanceHandle instanceHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymDpPerformOpNow,
                         instanceHandle, instanceHandle);
}

/**
 * cpaCySymInitSession:
 */
CpaStatus
cpaCySymInitSession (const CpaInstanceHandle instanceHandle,
                     const CpaCySymCbFunc pSymCb,
                     const CpaCySymSessionSetupData * pSessionSetupData,
                     CpaCySymSessionCtx sessionCtx)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymInitSession,
                         instanceHandle,instanceHandle,
                         pSymCb, pSessionSetupData, sessionCtx);
}

/**
 * cpaCySymRemoveSession:
 */
CpaStatus
cpaCySymRemoveSession (const CpaInstanceHandle instanceHandle,
                       CpaCySymSessionCtx pSessionCtx)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymRemoveSession,
                         instanceHandle,instanceHandle,
                         pSessionCtx);
}

/**
 * cpaCySymPerformOp:
 */
CpaStatus
cpaCySymPerformOp (const CpaInstanceHandle instanceHandle, void *pCallbackTag,
                   const CpaCySymOpData * pOpData,
                   const CpaBufferList * pSrcBuffer,
                   CpaBufferList * pDstBuffer, CpaBoolean * pVerifyResult)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymPerformOp,
                         instanceHandle, instanceHandle,
                         pCallbackTag, pOpData,
                         pSrcBuffer, pDstBuffer, pVerifyResult);
}

/**
 * cpaCySymQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCySymQueryStats (const CpaInstanceHandle instanceHandle,
                    struct _CpaCySymStats * pSymStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymQueryStats,
                         instanceHandle, instanceHandle,
                         pSymStats);
}

/**
 * cpaCySymQueryStats64:
 */
CpaStatus
cpaCySymQueryStats64 (const CpaInstanceHandle instanceHandle,
                      CpaCySymStats64 * pSymStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymQueryStats64,
                         instanceHandle, instanceHandle,
                         pSymStats);
}

/**
 * cpaCySymQueryCapabilities:
 */
CpaStatus
cpaCySymQueryCapabilities (const CpaInstanceHandle instanceHandle,
                           CpaCySymCapabilitiesInfo * pCapInfo)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymQueryCapabilities,
                         instanceHandle, instanceHandle,
                         pCapInfo);
}

/**
 * cpaCySymSessionCtxGetSize:
 */
CpaStatus
cpaCySymSessionCtxGetSize (const CpaInstanceHandle instanceHandle,
                           const CpaCySymSessionSetupData * pSessionSetupData,
                           Cpa32U * pSessionCtxSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymSessionCtxGetSize,
                         instanceHandle, instanceHandle,
                         pSessionSetupData, pSessionCtxSizeInBytes);
}

/**
 * cpaCySymSessionCtxGetDynamicSize:
 */
CpaStatus
cpaCySymSessionCtxGetDynamicSize (const CpaInstanceHandle instanceHandle,
                           const CpaCySymSessionSetupData * pSessionSetupData,
                           Cpa32U * pSessionCtxSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySymSessionCtxGetDynamicSize,
                         instanceHandle, instanceHandle,
                         pSessionSetupData, pSessionCtxSizeInBytes);
}


/**
 * cpaCyDrbgReseed:
 */
CpaStatus
cpaCyDrbgReseed (const CpaInstanceHandle instanceHandle, void *pCallbackTag,
                 CpaCyDrbgReseedOpData * pOpData)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgReseed,
                         instanceHandle, instanceHandle,
                         pCallbackTag, pOpData);
}

/**
 * cpaCyDrbgGen:
 */
CpaStatus
cpaCyDrbgGen (const CpaInstanceHandle instanceHandle, void *pCallbackTag,
              CpaCyDrbgGenOpData * pOpData, CpaFlatBuffer * pPseudoRandomBits)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgGen,
                         instanceHandle, instanceHandle,
                         pCallbackTag, pOpData, pPseudoRandomBits);
}

/**
 * cpaCyDrbgQueryStats64:
 */
CpaStatus
cpaCyDrbgQueryStats64 (const CpaInstanceHandle instanceHandle,
                       CpaCyDrbgStats64 * pStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDrbgQueryStats64,
                         instanceHandle, instanceHandle, pStats);
}

/**
 * icp_sal_drbgHTGetTestSessionSize:
 */
CpaStatus
icp_sal_drbgHTGetTestSessionSize (CpaInstanceHandle instanceHandle,
                                  Cpa32U * pTestSessionSize)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_drbgHTGetTestSessionSize,
                         instanceHandle, instanceHandle, pTestSessionSize);
}

/**
 * icp_sal_drbgHTInstantiate:
 */
CpaStatus
icp_sal_drbgHTInstantiate (const CpaInstanceHandle instanceHandle,
                           IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_drbgHTInstantiate,
                         instanceHandle, instanceHandle, testSessionHandle);
}

/**
 * icp_sal_drbgHTGenerate:
 */
CpaStatus
icp_sal_drbgHTGenerate (const CpaInstanceHandle instanceHandle,
                        IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_drbgHTGenerate,
                         instanceHandle, instanceHandle, testSessionHandle);
}

/**
 * icp_sal_drbgHTReseed:
 */
CpaStatus
icp_sal_drbgHTReseed (const CpaInstanceHandle instanceHandle,
                      IcpSalDrbgTestSessionHandle testSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_drbgHTReseed,
                         instanceHandle, instanceHandle, testSessionHandle);
}

/**
 * cpaCyNrbgGetEntropy:
 */
CpaStatus
cpaCyNrbgGetEntropy (const CpaInstanceHandle instanceHandle,
                     const CpaCyGenFlatBufCbFunc pCb, void *pCallbackTag,
                     const CpaCyNrbgOpData * pOpData,
                     CpaFlatBuffer * pEntropy)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyNrbgGetEntropy,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData, pEntropy);
}

/**
 * icp_sal_nrbgHealthTest:
 */
CpaStatus
icp_sal_nrbgHealthTest (const CpaInstanceHandle instanceHandle,
                        Cpa32U * pContinuousRngTestFailures)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_nrbgHealthTest,
                         instanceHandle, instanceHandle, pContinuousRngTestFailures);
}

/**
 * cpaCyDhKeyGenPhase1:
 */
CpaStatus
cpaCyDhKeyGenPhase1 (const CpaInstanceHandle instanceHandle,
                     const CpaCyGenFlatBufCbFunc pDhPhase1Cb,
                     void *pCallbackTag,
                     const CpaCyDhPhase1KeyGenOpData * pPhase1KeyGenData,
                     CpaFlatBuffer * pLocalOctetStringPV)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDhKeyGenPhase1,
                         instanceHandle, instanceHandle, pDhPhase1Cb, pCallbackTag,
                         pPhase1KeyGenData, pLocalOctetStringPV);
}

/**
 * cpaCyDhKeyGenPhase2Secret:
 */
CpaStatus
cpaCyDhKeyGenPhase2Secret (const CpaInstanceHandle instanceHandle,
                           const CpaCyGenFlatBufCbFunc pDhPhase2Cb,
                           void *pCallbackTag,
                           const CpaCyDhPhase2SecretKeyGenOpData *
                           pPhase2SecretKeyGenData,
                           CpaFlatBuffer * pOctetStringSecretKey)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDhKeyGenPhase2Secret,
                         instanceHandle, instanceHandle,
                         pDhPhase2Cb, pCallbackTag,
                         pPhase2SecretKeyGenData, pOctetStringSecretKey);
}

/**
 * cpaCyDhQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCyDhQueryStats (const CpaInstanceHandle instanceHandle,
                   struct _CpaCyDhStats * pDhStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDhQueryStats,
                         instanceHandle, instanceHandle, pDhStats);
}

/**
 * cpaCyDhQueryStats64:
 */
CpaStatus
cpaCyDhQueryStats64 (const CpaInstanceHandle instanceHandle,
                     CpaCyDhStats64 * pDhStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDhQueryStats64,
                         instanceHandle, instanceHandle, pDhStats);
}

/**
 * cpaCyKeyGenSsl:
 */
CpaStatus
cpaCyKeyGenSsl (const CpaInstanceHandle instanceHandle,
                const CpaCyGenFlatBufCbFunc pKeyGenCb, void *pCallbackTag,
                const CpaCyKeyGenSslOpData * pKeyGenSslOpData,
                CpaFlatBuffer * pGeneratedKeyBuffer)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenSsl,
                         instanceHandle, instanceHandle,
                         pKeyGenCb, pCallbackTag,
                         pKeyGenSslOpData, pGeneratedKeyBuffer);
}

/**
 * cpaCyKeyGenTls:
 */
CpaStatus
cpaCyKeyGenTls (const CpaInstanceHandle instanceHandle,
                const CpaCyGenFlatBufCbFunc pKeyGenCb, void *pCallbackTag,
                const CpaCyKeyGenTlsOpData * pKeyGenTlsOpData,
                CpaFlatBuffer * pGeneratedKeyBuffer)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenTls,
                         instanceHandle, instanceHandle,
                         pKeyGenCb, pCallbackTag,
                         pKeyGenTlsOpData, pGeneratedKeyBuffer);
}

/**
 * cpaCyKeyGenTls2:
 */
CpaStatus
cpaCyKeyGenTls2 (const CpaInstanceHandle instanceHandle,
                 const CpaCyGenFlatBufCbFunc pKeyGenCb, void *pCallbackTag,
                 const CpaCyKeyGenTlsOpData * pKeyGenTlsOpData,
                 CpaCySymHashAlgorithm hashAlgorithm,
                 CpaFlatBuffer * pGeneratedKeyBuffer)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenTls2,
                         instanceHandle, instanceHandle,
                         pKeyGenCb, pCallbackTag,
                         pKeyGenTlsOpData, hashAlgorithm,
                         pGeneratedKeyBuffer);
}

/**
 * cpaCyKeyGenMgf:
 */
CpaStatus
cpaCyKeyGenMgf (const CpaInstanceHandle instanceHandle,
                const CpaCyGenFlatBufCbFunc pKeyGenCb, void *pCallbackTag,
                const CpaCyKeyGenMgfOpData * pKeyGenMgfOpData,
                CpaFlatBuffer * pGeneratedMaskBuffer)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenMgf,
                         instanceHandle, instanceHandle,
                         pKeyGenCb, pCallbackTag,
                         pKeyGenMgfOpData, pGeneratedMaskBuffer);
}

/**
 * cpaCyKeyGenMgfExt:
 */
CpaStatus
cpaCyKeyGenMgfExt (const CpaInstanceHandle instanceHandle,
                   const CpaCyGenFlatBufCbFunc pKeyGenCb, void *pCallbackTag,
                   const CpaCyKeyGenMgfOpDataExt * pKeyGenMgfOpDataExt,
                   CpaFlatBuffer * pGeneratedMaskBuffer)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenMgfExt,
                         instanceHandle, instanceHandle,
                         pKeyGenCb, pCallbackTag,
                         pKeyGenMgfOpDataExt, pGeneratedMaskBuffer);
}

/**
 * cpaCyKeyGenQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCyKeyGenQueryStats (const CpaInstanceHandle instanceHandle,
                       struct _CpaCyKeyGenStats * pKeyGenStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenQueryStats,
                         instanceHandle, instanceHandle, pKeyGenStats);
}

/**
 * cpaCyKeyGenQueryStats64:
 */
CpaStatus
cpaCyKeyGenQueryStats64 (const CpaInstanceHandle instanceHandle,
                         CpaCyKeyGenStats64 * pKeyGenStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyKeyGenQueryStats64,
                         instanceHandle, instanceHandle, pKeyGenStats);
}

/**
 * cpaCyLnModExp:
 */
CpaStatus
cpaCyLnModExp (const CpaInstanceHandle instanceHandle,
               const CpaCyGenFlatBufCbFunc pLnModExpCb, void *pCallbackTag,
               const CpaCyLnModExpOpData * pLnModExpOpData,
               CpaFlatBuffer * pResult)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyLnModExp,
                         instanceHandle, instanceHandle,
                         pLnModExpCb, pCallbackTag,
                         pLnModExpOpData, pResult);
}

/**
 * cpaCyLnModInv:
 */
CpaStatus
cpaCyLnModInv (const CpaInstanceHandle instanceHandle,
               const CpaCyGenFlatBufCbFunc pLnModInvCb, void *pCallbackTag,
               const CpaCyLnModInvOpData * pLnModInvOpData,
               CpaFlatBuffer * pResult)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyLnModInv,
                         instanceHandle, instanceHandle,
                         pLnModInvCb, pCallbackTag,
                         pLnModInvOpData, pResult);
}

/**
 * cpaCyLnStatsQuery:
 */
CpaStatus CPA_DEPRECATED
cpaCyLnStatsQuery (const CpaInstanceHandle instanceHandle,
                   struct _CpaCyLnStats * pLnStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyLnStatsQuery,
                         instanceHandle, instanceHandle, pLnStats);
}

/**
 * cpaCyLnStatsQuery64:
 */
CpaStatus
cpaCyLnStatsQuery64 (const CpaInstanceHandle instanceHandle,
                     CpaCyLnStats64 * pLnStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyLnStatsQuery64,
                         instanceHandle, instanceHandle, pLnStats);
}

/**
 * cpaCyPrimeTest:
 */
CpaStatus
cpaCyPrimeTest (const CpaInstanceHandle instanceHandle,
                const CpaCyPrimeTestCbFunc pCb, void *pCallbackTag,
                const CpaCyPrimeTestOpData * pOpData,
                CpaBoolean * pTestPassed)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyPrimeTest,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pTestPassed);
}

/**
 * cpaCyPrimeQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCyPrimeQueryStats (const CpaInstanceHandle instanceHandle,
                      struct _CpaCyPrimeStats * pPrimeStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyPrimeQueryStats,
                         instanceHandle, instanceHandle, pPrimeStats);
}

/**
 * cpaCyPrimeQueryStats64:
 */
CpaStatus
cpaCyPrimeQueryStats64 (const CpaInstanceHandle instanceHandle,
                        CpaCyPrimeStats64 * pPrimeStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyPrimeQueryStats64,
                         instanceHandle, instanceHandle, pPrimeStats);
}

/**
 * cpaCyDsaGenPParam:
 */
CpaStatus
cpaCyDsaGenPParam (const CpaInstanceHandle instanceHandle,
                   const CpaCyDsaGenCbFunc pCb, void *pCallbackTag,
                   const CpaCyDsaPParamGenOpData * pOpData,
                   CpaBoolean * pProtocolStatus, CpaFlatBuffer * pP)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaGenPParam,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pProtocolStatus, pP);
}

/**
 * cpaCyDsaGenGParam:
 */
CpaStatus
cpaCyDsaGenGParam (const CpaInstanceHandle instanceHandle,
                   const CpaCyDsaGenCbFunc pCb, void *pCallbackTag,
                   const CpaCyDsaGParamGenOpData * pOpData,
                   CpaBoolean * pProtocolStatus, CpaFlatBuffer * pG)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaGenGParam,
                         instanceHandle, instanceHandle,
                         pCb, pCallbackTag, pOpData,
                         pProtocolStatus, pG);
}

/**
 * cpaCyDsaGenYParam:
 */
CpaStatus
cpaCyDsaGenYParam (const CpaInstanceHandle instanceHandle,
                   const CpaCyDsaGenCbFunc pCb, void *pCallbackTag,
                   const CpaCyDsaYParamGenOpData * pOpData,
                   CpaBoolean * pProtocolStatus, CpaFlatBuffer * pY)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaGenYParam,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pProtocolStatus, pY);
}

/**
 * cpaCyDsaSignR:
 */
CpaStatus
cpaCyDsaSignR (const CpaInstanceHandle instanceHandle,
               const CpaCyDsaGenCbFunc pCb, void *pCallbackTag,
               const CpaCyDsaRSignOpData * pOpData,
               CpaBoolean * pProtocolStatus, CpaFlatBuffer * pR)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaSignR,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pProtocolStatus, pR);
}

/**
 * cpaCyDsaSignS:
 */
CpaStatus
cpaCyDsaSignS (const CpaInstanceHandle instanceHandle,
               const CpaCyDsaGenCbFunc pCb, void *pCallbackTag,
               const CpaCyDsaSSignOpData * pOpData,
               CpaBoolean * pProtocolStatus, CpaFlatBuffer * pS)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaSignS,
                         instanceHandle, instanceHandle,
                         pCb, pCallbackTag, pOpData,
                         pProtocolStatus, pS);
}

/**
 * cpaCyDsaSignRS:
 */
CpaStatus
cpaCyDsaSignRS (const CpaInstanceHandle instanceHandle,
                const CpaCyDsaRSSignCbFunc pCb, void *pCallbackTag,
                const CpaCyDsaRSSignOpData * pOpData,
                CpaBoolean * pProtocolStatus, CpaFlatBuffer * pR,
                CpaFlatBuffer * pS)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaSignRS,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pProtocolStatus, pR, pS);
}

/**
 * cpaCyDsaVerify:
 */
CpaStatus
cpaCyDsaVerify (const CpaInstanceHandle instanceHandle,
                const CpaCyDsaVerifyCbFunc pCb, void *pCallbackTag,
                const CpaCyDsaVerifyOpData * pOpData,
                CpaBoolean * pVerifyStatus)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaVerify,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pVerifyStatus);
}

/**
 * cpaCyDsaQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCyDsaQueryStats (const CpaInstanceHandle instanceHandle,
                    struct _CpaCyDsaStats * pDsaStats)
{
   CPA_MUX_CALL_IMPL_FN(cpaCyDsaQueryStats,
                         instanceHandle, instanceHandle, pDsaStats);
}

/**
 * cpaCyDsaQueryStats64:
 */
CpaStatus
cpaCyDsaQueryStats64 (const CpaInstanceHandle instanceHandle,
                      CpaCyDsaStats64 * pDsaStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyDsaQueryStats64,
                         instanceHandle, instanceHandle, pDsaStats);
}

/**
 * cpaCyRsaGenKey:
 */
CpaStatus
cpaCyRsaGenKey (const CpaInstanceHandle instanceHandle,
                const CpaCyRsaKeyGenCbFunc pRsaKeyGenCb, void *pCallbackTag,
                const CpaCyRsaKeyGenOpData * pKeyGenOpData,
                CpaCyRsaPrivateKey * pPrivateKey,
                CpaCyRsaPublicKey * pPublicKey)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyRsaGenKey,
                         instanceHandle, instanceHandle,
                         pRsaKeyGenCb, pCallbackTag,
                         pKeyGenOpData, pPrivateKey, pPublicKey);
}

/**
 * cpaCyRsaEncrypt:
 */
CpaStatus
cpaCyRsaEncrypt (const CpaInstanceHandle instanceHandle,
                 const CpaCyGenFlatBufCbFunc pRsaEncryptCb,
                 void *pCallbackTag,
                 const CpaCyRsaEncryptOpData * pEncryptOpData,
                 CpaFlatBuffer * pOutputData)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyRsaEncrypt,
                         instanceHandle,instanceHandle,
                         pRsaEncryptCb, pCallbackTag,
                         pEncryptOpData, pOutputData);
}

/**
 * cpaCyRsaDecrypt:
 */
CpaStatus
cpaCyRsaDecrypt (const CpaInstanceHandle instanceHandle,
                 const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                 void *pCallbackTag,
                 const CpaCyRsaDecryptOpData * pDecryptOpData,
                 CpaFlatBuffer * pOutputData)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyRsaDecrypt,
                         instanceHandle, instanceHandle,
                         pRsaDecryptCb, pCallbackTag,
                         pDecryptOpData, pOutputData);
}

/**
 * cpaCyRsaQueryStats:
 */
CpaStatus CPA_DEPRECATED
cpaCyRsaQueryStats (const CpaInstanceHandle instanceHandle,
                    struct _CpaCyRsaStats * pRsaStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyRsaQueryStats,
                         instanceHandle, instanceHandle, pRsaStats);
}

/**
 * cpaCyRsaQueryStats64:
 */
CpaStatus
cpaCyRsaQueryStats64 (const CpaInstanceHandle instanceHandle,
                      CpaCyRsaStats64 * pRsaStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyRsaQueryStats64,
                         instanceHandle, instanceHandle, pRsaStats);
}

/**
 * cpaCyEcPointMultiply:
 */
CpaStatus
cpaCyEcPointMultiply (const CpaInstanceHandle instanceHandle,
                      const CpaCyEcPointMultiplyCbFunc pCb,
                      void *pCallbackTag,
                      const CpaCyEcPointMultiplyOpData * pOpData,
                      CpaBoolean * pMultiplyStatus, CpaFlatBuffer * pXk,
                      CpaFlatBuffer * pYk)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcPointMultiply,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pMultiplyStatus, pXk, pYk);
}

/**
 * cpaCyEcPointVerify:
 */
CpaStatus
cpaCyEcPointVerify (const CpaInstanceHandle instanceHandle,
                    const CpaCyEcPointVerifyCbFunc pCb, void *pCallbackTag,
                    const CpaCyEcPointVerifyOpData * pOpData,
                    CpaBoolean * pVerifyStatus)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcPointVerify,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pVerifyStatus);
}

/**
 * cpaCyEcQueryStats64:
 */
CpaStatus
cpaCyEcQueryStats64 (const CpaInstanceHandle instanceHandle,
                     CpaCyEcStats64 * pEcStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcQueryStats64,
                         instanceHandle, instanceHandle, pEcStats);
}

/**
 * cpaCyEcdhPointMultiply:
 */
CpaStatus
cpaCyEcdhPointMultiply (const CpaInstanceHandle instanceHandle,
                        const CpaCyEcdhPointMultiplyCbFunc pCb,
                        void *pCallbackTag,
                        const CpaCyEcdhPointMultiplyOpData * pOpData,
                        CpaBoolean * pMultiplyStatus, CpaFlatBuffer * pXk,
                        CpaFlatBuffer * pYk)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdhPointMultiply,
                         instanceHandle, instanceHandle, pCb, pCallbackTag,
                         pOpData, pMultiplyStatus, pXk, pYk);
}

/**
 * cpaCyEcdhQueryStats64:
 */
CpaStatus
cpaCyEcdhQueryStats64 (const CpaInstanceHandle instanceHandle,
                       CpaCyEcdhStats64 * pEcdhStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdhQueryStats64,
                         instanceHandle, instanceHandle, pEcdhStats);
}

/**
 * cpaCyEcdsaSignR:
 */
CpaStatus
cpaCyEcdsaSignR (const CpaInstanceHandle instanceHandle,
                 const CpaCyEcdsaGenSignCbFunc pCb, void *pCallbackTag,
                 const CpaCyEcdsaSignROpData * pOpData,
                 CpaBoolean * pSignStatus, CpaFlatBuffer * pR)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdsaSignR,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pSignStatus, pR);
}

/**
 * cpaCyEcdsaSignS:
 */
CpaStatus
cpaCyEcdsaSignS (const CpaInstanceHandle instanceHandle,
                 const CpaCyEcdsaGenSignCbFunc pCb, void *pCallbackTag,
                 const CpaCyEcdsaSignSOpData * pOpData,
                 CpaBoolean * pSignStatus, CpaFlatBuffer * pS)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdsaSignS,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pSignStatus, pS);
}

/**
 * cpaCyEcdsaSignRS:
 */
CpaStatus
cpaCyEcdsaSignRS (const CpaInstanceHandle instanceHandle,
                  const CpaCyEcdsaSignRSCbFunc pCb, void *pCallbackTag,
                  const CpaCyEcdsaSignRSOpData * pOpData,
                  CpaBoolean * pSignStatus, CpaFlatBuffer * pR,
                  CpaFlatBuffer * pS)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdsaSignRS,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pSignStatus, pR, pS);
}

/**
 * cpaCyEcdsaVerify:
 */
CpaStatus
cpaCyEcdsaVerify (const CpaInstanceHandle instanceHandle,
                  const CpaCyEcdsaVerifyCbFunc pCb, void *pCallbackTag,
                  const CpaCyEcdsaVerifyOpData * pOpData,
                  CpaBoolean * pVerifyStatus)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdsaVerify,
                         instanceHandle, instanceHandle, pCb,
                         pCallbackTag, pOpData,
                         pVerifyStatus);
}

/**
 * cpaCyEcdsaQueryStats64:
 */
CpaStatus
cpaCyEcdsaQueryStats64 (const CpaInstanceHandle instanceHandle,
                        CpaCyEcdsaStats64 * pEcdsaStats)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyEcdsaQueryStats64,
                         instanceHandle, instanceHandle, pEcdsaStats);
}

/**
 * cpaDcInitSession:
 */
CpaStatus
cpaDcInitSession (CpaInstanceHandle dcInstance,
                  CpaDcSessionHandle pSessionHandle,
                  CpaDcSessionSetupData * pSessionData,
                  CpaBufferList * pContextBuffer, CpaDcCallbackFn callbackFn)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcInitSession,
                         dcInstance, dcInstance, pSessionHandle, pSessionData,
                         pContextBuffer, callbackFn);
}

/**
 * cpaDcRemoveSession:
 */
CpaStatus
cpaDcRemoveSession (const CpaInstanceHandle dcInstance,
                    CpaDcSessionHandle pSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcRemoveSession,
                         dcInstance, dcInstance, pSessionHandle);
}

/**
 * cpaDcResetSession:
 */
CpaStatus
cpaDcResetSession (const CpaInstanceHandle dcInstance,
                   CpaDcSessionHandle pSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcResetSession,
                         dcInstance, dcInstance, pSessionHandle);
}

/**
 * cpaDcCompressData:
 */
CpaStatus
cpaDcCompressData (CpaInstanceHandle dcInstance,
                   CpaDcSessionHandle pSessionHandle,
                   CpaBufferList * pSrcBuff, CpaBufferList * pDestBuff,
                   CpaDcRqResults * pResults, CpaDcFlush flushFlag,
                   void *callbackTag)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcCompressData,
                         dcInstance, dcInstance, pSessionHandle, pSrcBuff,
                         pDestBuff, pResults, flushFlag, callbackTag);
}

/**
 * cpaDcDecompressData:
 */
CpaStatus
cpaDcDecompressData (CpaInstanceHandle dcInstance,
                     CpaDcSessionHandle pSessionHandle,
                     CpaBufferList * pSrcBuff, CpaBufferList * pDestBuff,
                     CpaDcRqResults * pResults, CpaDcFlush flushFlag,
                     void *callbackTag)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDecompressData,
                         dcInstance, dcInstance, pSessionHandle, pSrcBuff,
                         pDestBuff, pResults, flushFlag, callbackTag);
}

/**
 * cpaDcGetStats:
 */
CpaStatus
cpaDcGetStats (CpaInstanceHandle dcInstance, CpaDcStats * pStatistics)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcGetStats,
                         dcInstance, dcInstance, pStatistics);
}

/**
 * cpaDcDpGetSessionSize:
 */
CpaStatus
cpaDcDpGetSessionSize (CpaInstanceHandle dcInstance,
                       CpaDcSessionSetupData * pSessionData,
                       Cpa32U * pSessionSize)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDpGetSessionSize,
                         dcInstance, dcInstance,
                         pSessionData, pSessionSize);
}

/**
 * cpaDcDpInitSession:
 */
CpaStatus
cpaDcDpInitSession (CpaInstanceHandle dcInstance,
                    CpaDcSessionHandle pSessionHandle,
                    CpaDcSessionSetupData * pSessionData)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDpInitSession,
                         dcInstance, dcInstance,
                         pSessionHandle, pSessionData);
}

/**
 * cpaDcDpRemoveSession:
 */
CpaStatus
cpaDcDpRemoveSession (const CpaInstanceHandle dcInstance,
                      CpaDcSessionHandle pSessionHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDpRemoveSession,
                         dcInstance, dcInstance, pSessionHandle);
}

/**
 * cpaDcDpRegCbFunc:
 */
CpaStatus
cpaDcDpRegCbFunc (const CpaInstanceHandle dcInstance,
                  const CpaDcDpCallbackFn pNewCb)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDpRegCbFunc,
                         dcInstance, dcInstance, pNewCb);
}

/**
 * cpaDcDpPerformOpNow:
 */
CpaStatus
cpaDcDpPerformOpNow (CpaInstanceHandle dcInstance)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcDpPerformOpNow,
                         dcInstance, dcInstance);
}

/**
 * cpaCyBufferListGetMetaSize:
 */
CpaStatus
cpaCyBufferListGetMetaSize (const CpaInstanceHandle instanceHandle,
                            Cpa32U numBuffers, Cpa32U * pSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyBufferListGetMetaSize,
                         instanceHandle, instanceHandle,
                         numBuffers, pSizeInBytes);
}

/**
 * cpaCyGetStatusText:
 */
CpaStatus
cpaCyGetStatusText (const CpaInstanceHandle instanceHandle,
                    CpaStatus errStatus, Cpa8S * pStatusText)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyGetStatusText,
                         instanceHandle, instanceHandle,
                         errStatus, pStatusText);
}

/**
 * cpaCyInstanceGetInfo:
 */
CpaStatus CPA_DEPRECATED
cpaCyInstanceGetInfo (const CpaInstanceHandle instanceHandle,
                      struct _CpaInstanceInfo * pInstanceInfo)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyInstanceGetInfo,
                         instanceHandle, instanceHandle, pInstanceInfo);
}

/**
 * cpaCyInstanceGetInfo2:
 */
CpaStatus
cpaCyInstanceGetInfo2 (const CpaInstanceHandle instanceHandle,
                       CpaInstanceInfo2 * pInstanceInfo2)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyInstanceGetInfo2,
                         instanceHandle, instanceHandle,
                         pInstanceInfo2);
}

/**
 * cpaCyInstanceSetNotificationCb:
 */
CpaStatus
cpaCyInstanceSetNotificationCb (const CpaInstanceHandle instanceHandle,
                                const CpaCyInstanceNotificationCbFunc
                                pInstanceNotificationCb, void *pCallbackTag)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyInstanceSetNotificationCb,
                         instanceHandle, instanceHandle,
                         pInstanceNotificationCb, pCallbackTag);
}

/**
 * cpaCySetAddressTranslation:
 */
CpaStatus
cpaCySetAddressTranslation (const CpaInstanceHandle instanceHandle,
                            CpaVirtualToPhysical virtual2Physical)
{
    CPA_MUX_CALL_IMPL_FN(cpaCySetAddressTranslation,
                         instanceHandle, instanceHandle, virtual2Physical);
}

/**
 * cpaCyStartInstance:
 */
CpaStatus
cpaCyStartInstance (CpaInstanceHandle instanceHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyStartInstance,
                         instanceHandle, instanceHandle);
}

/**
 * cpaCyStopInstance:
 */
CpaStatus
cpaCyStopInstance (CpaInstanceHandle instanceHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyStopInstance,
                         instanceHandle, instanceHandle);
}

/**
 * cpaCyQueryCapabilities:
 */
CpaStatus
cpaCyQueryCapabilities (const CpaInstanceHandle instanceHandle,
                        CpaCyCapabilitiesInfo * pCapInfo)
{
    CPA_MUX_CALL_IMPL_FN(cpaCyQueryCapabilities,
                         instanceHandle, instanceHandle, pCapInfo);
}

/**
 * cpaDcGetNumIntermediateBuffers:
 */
CpaStatus
cpaDcGetNumIntermediateBuffers (CpaInstanceHandle instanceHandle,
                                Cpa16U *pNumBuffers)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcGetNumIntermediateBuffers,
                         instanceHandle, instanceHandle, pNumBuffers);
}

/**
 * cpaDcGetSessionSize:
 */
CpaStatus
cpaDcGetSessionSize (CpaInstanceHandle dcInstance,
                     CpaDcSessionSetupData * pSessionData,
                     Cpa32U * pSessionSize, Cpa32U * pContextSize)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcGetSessionSize,
                         dcInstance, dcInstance,
                         pSessionData, pSessionSize, pContextSize);
}

/**
 * cpaDcGetStatusText:
 */
CpaStatus
cpaDcGetStatusText (const CpaInstanceHandle dcInstance,
                    const CpaStatus errStatus, Cpa8S * pStatusText)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcGetStatusText,
                         dcInstance, dcInstance,
                         errStatus, pStatusText);
}

/**
 * cpaDcBufferListGetMetaSize:
 */
CpaStatus
cpaDcBufferListGetMetaSize (const CpaInstanceHandle instanceHandle,
                            Cpa32U numBuffers, Cpa32U * pSizeInBytes)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcBufferListGetMetaSize,
                         instanceHandle, instanceHandle,
                         numBuffers, pSizeInBytes);
}

/**
 * cpaDcInstanceGetInfo2:
 */
CpaStatus
cpaDcInstanceGetInfo2 (const CpaInstanceHandle instanceHandle,
                       CpaInstanceInfo2 * pInstanceInfo2)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcInstanceGetInfo2,
                         instanceHandle, instanceHandle, pInstanceInfo2);
}

/**
 * cpaDcInstanceSetNotificationCb:
 */
CpaStatus
cpaDcInstanceSetNotificationCb (const CpaInstanceHandle instanceHandle,
                                const CpaDcInstanceNotificationCbFunc
                                pInstanceNotificationCb, void *pCallbackTag)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcInstanceSetNotificationCb,
                         instanceHandle, instanceHandle,
                         pInstanceNotificationCb, pCallbackTag);
}

/**
 * cpaDcQueryCapabilities:
 */
CpaStatus
cpaDcQueryCapabilities (CpaInstanceHandle dcInstance,
                        CpaDcInstanceCapabilities * pInstanceCapabilities)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcQueryCapabilities,
                         dcInstance, dcInstance, pInstanceCapabilities);
}

/**
 * cpaDcSetAddressTranslation:
 */
CpaStatus
cpaDcSetAddressTranslation (const CpaInstanceHandle instanceHandle,
                            CpaVirtualToPhysical virtual2Physical)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcSetAddressTranslation,
                         instanceHandle, instanceHandle, virtual2Physical);
}

/**
 * cpaDcStartInstance:
 */
CpaStatus
cpaDcStartInstance (CpaInstanceHandle instanceHandle,
                    Cpa16U numBuffers,
                    CpaBufferList **pIntermediateBuffers)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcStartInstance,
                         instanceHandle, instanceHandle,
                         numBuffers, pIntermediateBuffers);
}

/**
 * cpaDcStopInstance:
 */
CpaStatus
cpaDcStopInstance (CpaInstanceHandle instanceHandle)
{
    CPA_MUX_CALL_IMPL_FN(cpaDcStopInstance,
                         instanceHandle, instanceHandle);
}

/**
 * icp_sal_CyPollInstance:
 */
CpaStatus
icp_sal_CyPollInstance (CpaInstanceHandle instanceHandle,
                        Cpa32U response_quota)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_CyPollInstance,
                         instanceHandle, instanceHandle, response_quota);
}

/**
 * icp_sal_CyPollDpInstance:
 */
CpaStatus
icp_sal_CyPollDpInstance (CpaInstanceHandle instanceHandle,
                          Cpa32U response_quota)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_CyPollDpInstance,
                         instanceHandle, instanceHandle, response_quota);
}

/**
 * icp_sal_DcPollInstance:
 */
CpaStatus
icp_sal_DcPollInstance (CpaInstanceHandle instanceHandle,
                        Cpa32U response_quota)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_DcPollInstance,
                         instanceHandle, instanceHandle, response_quota);
}

/**
 * icp_sal_DcPollDpInstance:
 */
CpaStatus
icp_sal_DcPollDpInstance (CpaInstanceHandle instanceHandle,
                          Cpa32U response_quota)
{
    CPA_MUX_CALL_IMPL_FN(icp_sal_DcPollDpInstance,
                         instanceHandle, instanceHandle, response_quota);
}

/**
 * icp_sal_iommu_get_remap_size:
 */
size_t
icp_sal_iommu_get_remap_size (size_t size)
{
    /**
     * This helper method is common across implementations, and does not access
     * driver-specific data, so just call the version in the first impl
     * we find.
     */
    CPA_MUX_CALL_IMPL_FN(icp_sal_iommu_get_remap_size,
                         CPA_INSTANCE_HANDLE_SINGLE,
                         size);
}

/**
 * icp_sal_iommu_map:
 */
CpaStatus icp_sal_iommu_map (Cpa64U phaddr,
                             Cpa64U iova,
                             size_t size)
{
    /**
     * This helper method is common across implementations, and does not access
     * driver-specific data, so just call the version in the first impl
     * we find.
     */
    CPA_MUX_CALL_IMPL_FN(icp_sal_iommu_map,
                         CPA_INSTANCE_HANDLE_SINGLE,
                         phaddr, iova, size);
}

/**
 * icp_sal_iommu_unmap:
 */
CpaStatus icp_sal_iommu_unmap (Cpa64U iova, size_t size)
{
    /**
     * This helper method is common across implementations, and does not access
     * driver-specific data, so just call the version in the first impl
     * we find.
     */
    CPA_MUX_CALL_IMPL_FN(icp_sal_iommu_unmap,
                         CPA_INSTANCE_HANDLE_SINGLE,
                         iova, size);
}
