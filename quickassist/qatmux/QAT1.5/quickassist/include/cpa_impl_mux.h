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
 *  version: QAT1.5.L.1.11.0-36
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file cpa_impl_mux.h
 *
 * @defgroup cpaImplMux CPA Implementation Mux
 *
 * Definition of the mux/demux shim layer for managing CPA API implementations
 *
 *****************************************************************************/


/*****************************************************************************/

#ifndef CPA_IMPL_MUX_H
#define CPA_IMPL_MUX_H

#include "cpa.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_common.h"
#include "cpa_cy_dh.h"
#include "cpa_cy_key.h"
#include "cpa_cy_prime.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_rsa.h"
#include "cpa_cy_ln.h"
#include "cpa_cy_im.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_ecdh.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_cy_drbg.h"
#include "cpa_cy_nrbg.h"
#include "cpa_cy_sym_dp.h"
#include "cpa_dc.h"
#include "cpa_dc_dp.h"
#include "icp_sal_versions.h"
#include "icp_sal_drbg_impl.h"
#include "icp_sal_drbg_ht.h"
#include "icp_sal_iommu.h"


/**
 *****************************************************************************
 * @ingroup cpaImplMux
 *      Driver type identifier for registration of CPA API implementations
 *
 * @description
 *      Enumeration which is used to provide driver-based identification of a
 *      library implementation of the CPA API during registration
 *
 * @see
 *      cpaMuxRegisterImpl()
 *
 *****************************************************************************/
typedef enum
{
    CPA_MUX_DRIVER_TYPE_QAT_1_6 = 0,
    /**< Indicates registration of a DH895XCC driver/library */
    CPA_MUX_DRIVER_TYPE_QAT_1_5,
    /**< Indicates registration of a DH89XXCC driver/library */
    CPA_MUX_DRIVER_TYPE_INVALID,
    /**< Indicates upper end of enumeration range */
} CpaMuxDriverType;


/**
 *****************************************************************************
 * @ingroup cpaImplMux
 *      Set of function pointers to be passed to cpaMuxRegisterImpl()
 *
 * @description
 *      Set of function pointers to be passed to cpaMuxRegisterImpl() upon
 *      registration of an implementation.  Applies to kernel-level code only.
 *
 * @see
 *      cpaMuxRegisterImpl()
 *
 *****************************************************************************/
typedef struct _CpaFuncPtrs
{
    /* Symbols for getting version information */
    CpaStatus (*icp_sal_getDevVersionInfo) (Cpa32U accelId,
                                            icp_sal_dev_version_info_t *
                                            pVerInfo);

    /* Symmetric Data Plane API symbols, declared in cpa_cy_sym_dp.h */
    CpaStatus (*cpaCySymDpEnqueueOp) (CpaCySymDpOpData * pOpData,
                                      const CpaBoolean performOpNow);
    CpaStatus (*cpaCySymDpEnqueueOpBatch) (const Cpa32U numberRequests,
                                           CpaCySymDpOpData * pOpData[],
                                           const CpaBoolean performOpNow);
    CpaStatus (*cpaCySymDpRegCbFunc) (const CpaInstanceHandle instanceHandle,
                                      const CpaCySymDpCbFunc pSymNewCb);
    CpaStatus (*cpaCySymDpSessionCtxGetSize) (const CpaInstanceHandle
                                              instanceHandle,
                                              const CpaCySymSessionSetupData *
                                              pSessionSetupData,
                                              Cpa32U *
                                              pSessionCtxSizeInBytes);
    CpaStatus (*cpaCySymDpSessionCtxGetDynamicSize) (
        const CpaInstanceHandle instanceHandle,
        const CpaCySymSessionSetupData *pSessionSetupData,
        Cpa32U *pSessionCtxSizeInBytes);
    
    CpaStatus (*cpaCySymDpInitSession) (CpaInstanceHandle instanceHandle,
                                        const CpaCySymSessionSetupData *
                                        pSessionSetupData,
                                        CpaCySymDpSessionCtx sessionCtx);
    CpaStatus (*cpaCySymDpRemoveSession) (const CpaInstanceHandle
                                          instanceHandle,
                                          CpaCySymDpSessionCtx sessionCtx);
    CpaStatus (*cpaCySymDpPerformOpNow) (CpaInstanceHandle instanceHandle);

    /* Symmetric Crypto API symbols, declared in cpa_cy_sym.h */
    CpaStatus (*cpaCySymInitSession) (const CpaInstanceHandle instanceHandle,
                                      const CpaCySymCbFunc pSymCb,
                                      const CpaCySymSessionSetupData *
                                      pSessionSetupData,
                                      CpaCySymSessionCtx sessionCtx);
    CpaStatus (*cpaCySymRemoveSession) (const CpaInstanceHandle
                                        instanceHandle,
                                        CpaCySymSessionCtx pSessionCtx);
    CpaStatus (*cpaCySymPerformOp) (const CpaInstanceHandle instanceHandle,
                                    void *pCallbackTag,
                                    const CpaCySymOpData * pOpData,
                                    const CpaBufferList * pSrcBuffer,
                                    CpaBufferList * pDstBuffer,
                                    CpaBoolean * pVerifyResult);
    CpaStatus CPA_DEPRECATED (*cpaCySymQueryStats) (const CpaInstanceHandle
                                                    instanceHandle,
                                                    struct _CpaCySymStats *
                                                    pSymStats);
    CpaStatus (*cpaCySymQueryStats64) (const CpaInstanceHandle
                                       instanceHandle,
                                       CpaCySymStats64 * pSymStats);
    CpaStatus (*cpaCySymQueryCapabilities) (const CpaInstanceHandle
                                            instanceHandle,
                                            CpaCySymCapabilitiesInfo *
                                            pCapInfo);
    CpaStatus (*cpaCySymSessionCtxGetSize) (const CpaInstanceHandle
                                            instanceHandle,
                                            const CpaCySymSessionSetupData *
                                            pSessionSetupData,
                                            Cpa32U *
                                            pSessionCtxSizeInBytes);
    CpaStatus (*cpaCySymSessionCtxGetDynamicSize) (
        const CpaInstanceHandle instanceHandle,
        const CpaCySymSessionSetupData *pSessionSetupData,
        Cpa32U *pSessionCtxSizeInBytes);

    /* DRBG API symbols, declared in cpa_cy_drbg.h */
    CpaStatus (*cpaCyDrbgInitSession) (const CpaInstanceHandle
                                       instanceHandle,
                                       const CpaCyGenFlatBufCbFunc pGenCb,
                                       const CpaCyGenericCbFunc pReseedCb,
                                       const CpaCyDrbgSessionSetupData *
                                       pSetupData,
                                       CpaCyDrbgSessionHandle sessionHandle,
                                       Cpa32U * pSeedLen);
    CpaStatus (*cpaCyDrbgReseed) (const CpaInstanceHandle instanceHandle,
                                  void *pCallbackTag,
                                  CpaCyDrbgReseedOpData * pOpData);
    CpaStatus (*cpaCyDrbgGen) (const CpaInstanceHandle instanceHandle,
                               void *pCallbackTag,
                               CpaCyDrbgGenOpData * pOpData,
                               CpaFlatBuffer * pPseudoRandomBits);
    CpaStatus (*cpaCyDrbgSessionGetSize) (const CpaInstanceHandle
                                          instanceHandle,
                                          const CpaCyDrbgSessionSetupData *
                                          pSetupData, Cpa32U * pSize);
    CpaStatus (*cpaCyDrbgRemoveSession) (const CpaInstanceHandle
                                         instanceHandle,
                                         CpaCyDrbgSessionHandle
                                         sessionHandle);
    CpaStatus (*cpaCyDrbgQueryStats64) (const CpaInstanceHandle
                                        instanceHandle,
                                        CpaCyDrbgStats64 * pStats);

    /* Additional DRBG API symbols, declared in icp_sal_drbg_impl.h */
    IcpSalDrbgGetEntropyInputFunc (*icp_sal_drbgGetEntropyInputFuncRegister)
        (IcpSalDrbgGetEntropyInputFunc func);
    IcpSalDrbgGetNonceFunc (*icp_sal_drbgGetNonceFuncRegister)
        (IcpSalDrbgGetNonceFunc func);
    IcpSalDrbgIsDFReqFunc (*icp_sal_drbgIsDFReqFuncRegister)
        (IcpSalDrbgIsDFReqFunc func);
    void (*icp_sal_drbgGetInstance) (CpaCyDrbgSessionHandle sessionHandle,
                                     CpaInstanceHandle **pDrbgInstance);

    /* Additional DRBG API symbols, declared in icp_sal_drbg_ht.h */
    CpaStatus (*icp_sal_drbgHTGetTestSessionSize) (CpaInstanceHandle
                                                   instanceHandle,
                                                   Cpa32U *
                                                   pTestSessionSize);
    CpaStatus (*icp_sal_drbgHTInstantiate) (const CpaInstanceHandle
                                            instanceHandle,
                                            IcpSalDrbgTestSessionHandle
                                            testSessionHandle);
    CpaStatus (*icp_sal_drbgHTGenerate) (const CpaInstanceHandle
                                         instanceHandle,
                                         IcpSalDrbgTestSessionHandle
                                         testSessionHandle);
    CpaStatus (*icp_sal_drbgHTReseed) (const CpaInstanceHandle
                                       instanceHandle,
                                       IcpSalDrbgTestSessionHandle
                                       testSessionHandle);

    /* NRBG API symbols, declared in cpa_cy_nrbg.h */
    CpaStatus (*cpaCyNrbgGetEntropy) (const CpaInstanceHandle
                                      instanceHandle,
                                      const CpaCyGenFlatBufCbFunc pCb,
                                      void *pCallbackTag,
                                      const CpaCyNrbgOpData * pOpData,
                                      CpaFlatBuffer * pEntropy);
    /* Additional NRBG API symbols, declared in icp_sal_nrbg_ht.h */
    CpaStatus (*icp_sal_nrbgHealthTest) (const CpaInstanceHandle
                                         instanceHandle,
                                         Cpa32U *
                                         pContinuousRngTestFailures);

    /* Diffie Hellman API symbols, declared in cpa_cy_dh.h */
    CpaStatus (*cpaCyDhKeyGenPhase1) (const CpaInstanceHandle
                                      instanceHandle,
                                      const CpaCyGenFlatBufCbFunc
                                      pDhPhase1Cb, void *pCallbackTag,
                                      const CpaCyDhPhase1KeyGenOpData *
                                      pPhase1KeyGenData,
                                      CpaFlatBuffer * pLocalOctetStringPV);
    CpaStatus (*cpaCyDhKeyGenPhase2Secret) (const CpaInstanceHandle
                                            instanceHandle,
                                            const CpaCyGenFlatBufCbFunc
                                            pDhPhase2Cb, void *pCallbackTag,
                                            const
                                            CpaCyDhPhase2SecretKeyGenOpData
                                            * pPhase2SecretKeyGenData,
                                            CpaFlatBuffer *
                                            pOctetStringSecretKey);
    CpaStatus CPA_DEPRECATED (*cpaCyDhQueryStats) (const CpaInstanceHandle
                                                   instanceHandle,
                                                   struct _CpaCyDhStats *
                                                   pDhStats);
    CpaStatus (*cpaCyDhQueryStats64) (const CpaInstanceHandle
                                      instanceHandle,
                                      CpaCyDhStats64 * pDhStats);

    /*Key Expansion and Generation API symbols, declared in cpa_cy_key.h */
    CpaStatus (*cpaCyKeyGenSsl) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyGenFlatBufCbFunc pKeyGenCb,
                                 void *pCallbackTag,
                                 const CpaCyKeyGenSslOpData *
                                 pKeyGenSslOpData,
                                 CpaFlatBuffer * pGeneratedKeyBuffer);
    CpaStatus (*cpaCyKeyGenTls) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyGenFlatBufCbFunc pKeyGenCb,
                                 void *pCallbackTag,
                                 const CpaCyKeyGenTlsOpData *
                                 pKeyGenTlsOpData,
                                 CpaFlatBuffer * pGeneratedKeyBuffer);
    CpaStatus (*cpaCyKeyGenTls2) (const CpaInstanceHandle instanceHandle,
                                  const CpaCyGenFlatBufCbFunc pKeyGenCb,
                                  void *pCallbackTag,
                                  const CpaCyKeyGenTlsOpData *
                                  pKeyGenTlsOpData,
                                  CpaCySymHashAlgorithm hashAlgorithm,
                                  CpaFlatBuffer * pGeneratedKeyBuffer);
    CpaStatus (*cpaCyKeyGenMgf) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyGenFlatBufCbFunc pKeyGenCb,
                                 void *pCallbackTag,
                                 const CpaCyKeyGenMgfOpData *
                                 pKeyGenMgfOpData,
                                 CpaFlatBuffer * pGeneratedMaskBuffer);
    CpaStatus (*cpaCyKeyGenMgfExt) (const CpaInstanceHandle instanceHandle,
                                    const CpaCyGenFlatBufCbFunc pKeyGenCb,
                                    void *pCallbackTag,
                                    const CpaCyKeyGenMgfOpDataExt *
                                    pKeyGenMgfOpDataExt,
                                    CpaFlatBuffer * pGeneratedMaskBuffer);
    CpaStatus CPA_DEPRECATED (*cpaCyKeyGenQueryStats) (const CpaInstanceHandle
                                                       instanceHandle,
                                                       struct
                                                       _CpaCyKeyGenStats *
                                                       pKeyGenStats);
    CpaStatus (*cpaCyKeyGenQueryStats64) (const CpaInstanceHandle
                                          instanceHandle,
                                          CpaCyKeyGenStats64 *
                                          pKeyGenStats);

    /* Large Number ModExp and ModInv API symbols, declared in cpa_cy_ln.h */
    CpaStatus (*cpaCyLnModExp) (const CpaInstanceHandle instanceHandle,
                                const CpaCyGenFlatBufCbFunc pLnModExpCb,
                                void *pCallbackTag,
                                const CpaCyLnModExpOpData * pLnModExpOpData,
                                CpaFlatBuffer * pResult);
    CpaStatus (*cpaCyLnModInv) (const CpaInstanceHandle instanceHandle,
                                const CpaCyGenFlatBufCbFunc pLnModInvCb,
                                void *pCallbackTag,
                                const CpaCyLnModInvOpData * pLnModInvOpData,
                                CpaFlatBuffer * pResult);
    CpaStatus CPA_DEPRECATED (*cpaCyLnStatsQuery) (const CpaInstanceHandle
                                                   instanceHandle,
                                                   struct _CpaCyLnStats *
                                                   pLnStats);
    CpaStatus (*cpaCyLnStatsQuery64) (const CpaInstanceHandle
                                      instanceHandle,
                                      CpaCyLnStats64 * pLnStats);

    /* Prime API symbols, declared in cpa_cy_prime.h */
    CpaStatus (*cpaCyPrimeTest) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyPrimeTestCbFunc pCb,
                                 void *pCallbackTag,
                                 const CpaCyPrimeTestOpData * pOpData,
                                 CpaBoolean * pTestPassed);
    CpaStatus CPA_DEPRECATED (*cpaCyPrimeQueryStats) (const CpaInstanceHandle
                                                      instanceHandle,
                                                      struct _CpaCyPrimeStats
                                                      * pPrimeStats);
    CpaStatus (*cpaCyPrimeQueryStats64) (const CpaInstanceHandle
                                         instanceHandle,
                                         CpaCyPrimeStats64 * pPrimeStats);

    /* DSA API symbols, declared in cpa_cy_dsa.h */
    CpaStatus (*cpaCyDsaGenPParam) (const CpaInstanceHandle instanceHandle,
                                    const CpaCyDsaGenCbFunc pCb,
                                    void *pCallbackTag,
                                    const CpaCyDsaPParamGenOpData * pOpData,
                                    CpaBoolean * pProtocolStatus,
                                    CpaFlatBuffer * pP);
    CpaStatus (*cpaCyDsaGenGParam) (const CpaInstanceHandle instanceHandle,
                                    const CpaCyDsaGenCbFunc pCb,
                                    void *pCallbackTag,
                                    const CpaCyDsaGParamGenOpData * pOpData,
                                    CpaBoolean * pProtocolStatus,
                                    CpaFlatBuffer * pG);
    CpaStatus (*cpaCyDsaGenYParam) (const CpaInstanceHandle instanceHandle,
                                    const CpaCyDsaGenCbFunc pCb,
                                    void *pCallbackTag,
                                    const CpaCyDsaYParamGenOpData * pOpData,
                                    CpaBoolean * pProtocolStatus,
                                    CpaFlatBuffer * pY);
    CpaStatus (*cpaCyDsaSignR) (const CpaInstanceHandle instanceHandle,
                                const CpaCyDsaGenCbFunc pCb,
                                void *pCallbackTag,
                                const CpaCyDsaRSignOpData * pOpData,
                                CpaBoolean * pProtocolStatus,
                                CpaFlatBuffer * pR);
    CpaStatus (*cpaCyDsaSignS) (const CpaInstanceHandle instanceHandle,
                                const CpaCyDsaGenCbFunc pCb,
                                void *pCallbackTag,
                                const CpaCyDsaSSignOpData * pOpData,
                                CpaBoolean * pProtocolStatus,
                                CpaFlatBuffer * pS);
    CpaStatus (*cpaCyDsaSignRS) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyDsaRSSignCbFunc pCb,
                                 void *pCallbackTag,
                                 const CpaCyDsaRSSignOpData * pOpData,
                                 CpaBoolean * pProtocolStatus,
                                 CpaFlatBuffer * pR, CpaFlatBuffer * pS);
    CpaStatus (*cpaCyDsaVerify) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyDsaVerifyCbFunc pCb,
                                 void *pCallbackTag,
                                 const CpaCyDsaVerifyOpData * pOpData,
                                 CpaBoolean * pVerifyStatus);
    CpaStatus CPA_DEPRECATED (*cpaCyDsaQueryStats) (const CpaInstanceHandle
                                                    instanceHandle,
                                                    struct _CpaCyDsaStats *
                                                    pDsaStats);
    CpaStatus (*cpaCyDsaQueryStats64) (const CpaInstanceHandle
                                       instanceHandle,
                                       CpaCyDsaStats64 * pDsaStats);

    /* RSA API symbols, declared in cpa_cy_rsa.h */
    CpaStatus (*cpaCyRsaGenKey) (const CpaInstanceHandle instanceHandle,
                                 const CpaCyRsaKeyGenCbFunc pRsaKeyGenCb,
                                 void *pCallbackTag,
                                 const CpaCyRsaKeyGenOpData * pKeyGenOpData,
                                 CpaCyRsaPrivateKey * pPrivateKey,
                                 CpaCyRsaPublicKey * pPublicKey);
    CpaStatus (*cpaCyRsaEncrypt) (const CpaInstanceHandle instanceHandle,
                                  const CpaCyGenFlatBufCbFunc pRsaEncryptCb,
                                  void *pCallbackTag,
                                  const CpaCyRsaEncryptOpData *
                                  pEncryptOpData,
                                  CpaFlatBuffer * pOutputData);
    CpaStatus (*cpaCyRsaDecrypt) (const CpaInstanceHandle instanceHandle,
                                  const CpaCyGenFlatBufCbFunc pRsaDecryptCb,
                                  void *pCallbackTag,
                                  const CpaCyRsaDecryptOpData *
                                  pDecryptOpData,
                                  CpaFlatBuffer * pOutputData);
    CpaStatus CPA_DEPRECATED (*cpaCyRsaQueryStats) (const CpaInstanceHandle
                                                    instanceHandle,
                                                    struct _CpaCyRsaStats *
                                                    pRsaStats);
    CpaStatus (*cpaCyRsaQueryStats64) (const CpaInstanceHandle
                                       instanceHandle,
                                       CpaCyRsaStats64 * pRsaStats);

    /* EC API symbols, declared in cpa_cy_ec.h */
    CpaStatus (*cpaCyEcPointMultiply) (const CpaInstanceHandle
                                       instanceHandle,
                                       const CpaCyEcPointMultiplyCbFunc pCb,
                                       void *pCallbackTag,
                                       const CpaCyEcPointMultiplyOpData *
                                       pOpData,
                                       CpaBoolean * pMultiplyStatus,
                                       CpaFlatBuffer * pXk,
                                       CpaFlatBuffer * pYk);
    CpaStatus (*cpaCyEcPointVerify) (const CpaInstanceHandle instanceHandle,
                                     const CpaCyEcPointVerifyCbFunc pCb,
                                     void *pCallbackTag,
                                     const CpaCyEcPointVerifyOpData *
                                     pOpData, CpaBoolean * pVerifyStatus);
    CpaStatus (*cpaCyEcQueryStats64) (const CpaInstanceHandle
                                      instanceHandle,
                                      CpaCyEcStats64 * pEcStats);

    /* ECDH API symbols, declared in cpa_cy_ecdh.h */
    CpaStatus (*cpaCyEcdhPointMultiply) (const CpaInstanceHandle
                                         instanceHandle,
                                         const CpaCyEcdhPointMultiplyCbFunc
                                         pCb, void *pCallbackTag,
                                         const CpaCyEcdhPointMultiplyOpData
                                         * pOpData,
                                         CpaBoolean * pMultiplyStatus,
                                         CpaFlatBuffer * pXk,
                                         CpaFlatBuffer * pYk);
    CpaStatus (*cpaCyEcdhQueryStats64) (const CpaInstanceHandle
                                        instanceHandle,
                                        CpaCyEcdhStats64 * pEcdhStats);

    /* ECDSA API symbols, declared in cpa_cy_ecdsa.h */
    CpaStatus (*cpaCyEcdsaSignR) (const CpaInstanceHandle instanceHandle,
                                  const CpaCyEcdsaGenSignCbFunc pCb,
                                  void *pCallbackTag,
                                  const CpaCyEcdsaSignROpData * pOpData,
                                  CpaBoolean * pSignStatus,
                                  CpaFlatBuffer * pR);
    CpaStatus (*cpaCyEcdsaSignS) (const CpaInstanceHandle instanceHandle,
                                  const CpaCyEcdsaGenSignCbFunc pCb,
                                  void *pCallbackTag,
                                  const CpaCyEcdsaSignSOpData * pOpData,
                                  CpaBoolean * pSignStatus,
                                  CpaFlatBuffer * pS);
    CpaStatus (*cpaCyEcdsaSignRS) (const CpaInstanceHandle instanceHandle,
                                   const CpaCyEcdsaSignRSCbFunc pCb,
                                   void *pCallbackTag,
                                   const CpaCyEcdsaSignRSOpData * pOpData,
                                   CpaBoolean * pSignStatus,
                                   CpaFlatBuffer * pR, CpaFlatBuffer * pS);
    CpaStatus (*cpaCyEcdsaVerify) (const CpaInstanceHandle instanceHandle,
                                   const CpaCyEcdsaVerifyCbFunc pCb,
                                   void *pCallbackTag,
                                   const CpaCyEcdsaVerifyOpData * pOpData,
                                   CpaBoolean * pVerifyStatus);
    CpaStatus (*cpaCyEcdsaQueryStats64) (const CpaInstanceHandle
                                         instanceHandle,
                                         CpaCyEcdsaStats64 * pEcdsaStats);

    /* DC Compression API symbols, declared in cpa_dc.h */
    CpaStatus (*cpaDcInitSession) (CpaInstanceHandle dcInstance,
                                   CpaDcSessionHandle pSessionHandle,
                                   CpaDcSessionSetupData * pSessionData,
                                   CpaBufferList * pContextBuffer,
                                   CpaDcCallbackFn callbackFn);
    CpaStatus (*cpaDcRemoveSession) (const CpaInstanceHandle dcInstance,
                                     CpaDcSessionHandle pSessionHandle);
    CpaStatus (*cpaDcResetSession) (const CpaInstanceHandle dcInstance,
                                    CpaDcSessionHandle pSessionHandle);
    CpaStatus (*cpaDcCompressData) (CpaInstanceHandle dcInstance,
                                    CpaDcSessionHandle pSessionHandle,
                                    CpaBufferList * pSrcBuff,
                                    CpaBufferList * pDestBuff,
                                    CpaDcRqResults * pResults,
                                    CpaDcFlush flushFlag,
                                    void *callbackTag);
    CpaStatus (*cpaDcDecompressData) (CpaInstanceHandle dcInstance,
                                      CpaDcSessionHandle pSessionHandle,
                                      CpaBufferList * pSrcBuff,
                                      CpaBufferList * pDestBuff,
                                      CpaDcRqResults * pResults,
                                      CpaDcFlush flushFlag,
                                      void *callbackTag);
    CpaStatus (*cpaDcGenerateHeader) (CpaDcSessionHandle pSessionHandle,
                                      CpaFlatBuffer * pDestBuff,
                                      Cpa32U * count);
    CpaStatus (*cpaDcGenerateFooter) (CpaDcSessionHandle pSessionHandle,
                                      CpaFlatBuffer * pDestBuff,
                                      CpaDcRqResults * pResults);
    CpaStatus (*cpaDcGetStats) (CpaInstanceHandle dcInstance,
                                CpaDcStats * pStatistics);

    /* DcDp Compression API symbols, declared in cpa_dc_dp.h */
    CpaStatus (*cpaDcDpGetSessionSize) (CpaInstanceHandle dcInstance,
                                        CpaDcSessionSetupData *
                                        pSessionData,
                                        Cpa32U * pSessionSize);
    CpaStatus (*cpaDcDpInitSession) (CpaInstanceHandle dcInstance,
                                     CpaDcSessionHandle pSessionHandle,
                                     CpaDcSessionSetupData * pSessionData);
    CpaStatus (*cpaDcDpRemoveSession) (const CpaInstanceHandle dcInstance,
                                       CpaDcSessionHandle pSessionHandle);
    CpaStatus (*cpaDcDpRegCbFunc) (const CpaInstanceHandle dcInstance,
                                   const CpaDcDpCallbackFn pNewCb);
    CpaStatus (*cpaDcDpEnqueueOp) (CpaDcDpOpData * pOpData,
                                   const CpaBoolean performOpNow);
    CpaStatus (*cpaDcDpEnqueueOpBatch) (const Cpa32U numberRequests,
                                        CpaDcDpOpData * pOpData[],
                                        const CpaBoolean performOpNow);
    CpaStatus (*cpaDcDpPerformOpNow) (CpaInstanceHandle dcInstance);

    /* Symbols for all of LAC, declared in cpa_cy_common.h */
    CpaStatus (*cpaCyBufferListGetMetaSize) (const CpaInstanceHandle
                                             instanceHandle,
                                             Cpa32U numBuffers,
                                             Cpa32U * pSizeInBytes);

    CpaStatus (*cpaCyGetInstances) (Cpa16U numInstances,
                                    CpaInstanceHandle * cyInstances);
    CpaStatus (*cpaCyGetNumInstances) (Cpa16U * pNumInstances);
    CpaStatus (*cpaCyGetStatusText) (const CpaInstanceHandle instanceHandle,
                                     CpaStatus errStatus,
                                     Cpa8S * pStatusText);
    CpaStatus CPA_DEPRECATED (*cpaCyInstanceGetInfo) (const CpaInstanceHandle
                                                      instanceHandle,
                                                      struct _CpaInstanceInfo
                                                      * pInstanceInfo);
    CpaStatus (*cpaCyInstanceGetInfo2) (const CpaInstanceHandle
                                        instanceHandle,
                                        CpaInstanceInfo2 * pInstanceInfo2);
    CpaStatus (*cpaCyInstanceSetNotificationCb) (const CpaInstanceHandle
                                                 instanceHandle,
                                                 const
                                                 CpaCyInstanceNotificationCbFunc
                                                 pInstanceNotificationCb,
                                                 void *pCallbackTag);
    CpaStatus (*cpaCySetAddressTranslation) (const CpaInstanceHandle
                                             instanceHandle,
                                             CpaVirtualToPhysical
                                             virtual2Physical);
    CpaStatus (*cpaCyStartInstance) (CpaInstanceHandle instanceHandle);
    CpaStatus (*cpaCyStopInstance) (CpaInstanceHandle instanceHandle);

    CpaStatus (*cpaCyQueryCapabilities) (const CpaInstanceHandle
                                         instanceHandle,
                                         CpaCyCapabilitiesInfo * pCapInfo);

    /* Additional DC Compression API symbols, declared in cpa_dc.h */
    CpaStatus (*cpaDcGetInstances) (Cpa16U numInstances,
                                    CpaInstanceHandle * dcInstances);
    CpaStatus (*cpaDcGetNumInstances) (Cpa16U * pNumInstances);
    CpaStatus (*cpaDcGetSessionSize) (CpaInstanceHandle dcInstance,
                                      CpaDcSessionSetupData * pSessionData,
                                      Cpa32U * pSessionSize,
                                      Cpa32U * pContextSize);
    CpaStatus (*cpaDcGetStatusText) (const CpaInstanceHandle dcInstance,
                                     const CpaStatus errStatus,
                                     Cpa8S * pStatusText);
    CpaStatus (*cpaDcBufferListGetMetaSize) (const CpaInstanceHandle
                                             instanceHandle,
                                             Cpa32U numBuffers,
                                             Cpa32U * pSizeInBytes);
    CpaStatus (*cpaDcInstanceGetInfo2) (const CpaInstanceHandle
                                        instanceHandle,
                                        CpaInstanceInfo2 * pInstanceInfo2);
    CpaStatus (*cpaDcInstanceSetNotificationCb) (const CpaInstanceHandle
                                                 instanceHandle,
                                                 const
                                                 CpaCyInstanceNotificationCbFunc
                                                 pInstanceNotificationCb,
                                                 void *pCallbackTag);
    CpaStatus (*cpaDcQueryCapabilities) (CpaInstanceHandle dcInstance,
                                         CpaDcInstanceCapabilities *
                                         pInstanceCapabilities);
    CpaStatus (*cpaDcSetAddressTranslation) (const CpaInstanceHandle
                                             instanceHandle,
                                             CpaVirtualToPhysical
                                             virtual2Physical);
    CpaStatus (*cpaDcGetNumIntermediateBuffers) (
                                             CpaInstanceHandle instanceHandle,
                                             Cpa16U *pNumBuffers);
    CpaStatus (*cpaDcStartInstance) (CpaInstanceHandle instanceHandle,
                                     Cpa16U numBuffers,
                                     CpaBufferList **pIntermediateBuffers);
    CpaStatus (*cpaDcStopInstance) (CpaInstanceHandle instanceHandle);

    /* Polling symbols, declared in icp_sal_poll.h */
    CpaStatus (*icp_sal_CyPollInstance) (CpaInstanceHandle instanceHandle,
                                         Cpa32U response_quota);
    CpaStatus (*icp_sal_CyPollDpInstance) (CpaInstanceHandle instanceHandle,
                                           Cpa32U response_quota);
    CpaStatus (*icp_sal_DcPollInstance) (CpaInstanceHandle instanceHandle,
                                         Cpa32U response_quota);
    CpaStatus (*icp_sal_DcPollDpInstance) (CpaInstanceHandle instanceHandle,
                                           Cpa32U response_quota);
#ifdef USER_SPACE
    /* User space process init/shutdown functions, declared in icp_sal_user.h */
    CpaStatus (*icp_sal_userStartMultiProcess) (const char *pProcessName,
                                                CpaBoolean limitDevAccess);
    CpaStatus (*icp_sal_userStart) (const char *pProcessName);
    CpaStatus (*icp_sal_userStop) (void);
    /* User space Dynamic instance API's */
    CpaStatus (*icp_sal_userCyGetAvailableNumDynInstances)(
                                                    Cpa32U *pNumCyInstances);
    CpaStatus (*icp_sal_userDcGetAvailableNumDynInstances)(
                                                    Cpa32U *pNumDcInstances);
    CpaStatus (*icp_sal_userCyInstancesAlloc)(Cpa32U numCyInstances,
                                            CpaInstanceHandle *pCyInstances);
    CpaStatus (*icp_sal_userDcInstancesAlloc)(Cpa32U numDcInstances,
                                            CpaInstanceHandle *pDcInstances);
    CpaStatus (*icp_sal_userCyFreeInstances)(Cpa32U numCyInstances,
                                            CpaInstanceHandle *pCyInstances);
    CpaStatus (*icp_sal_userDcFreeInstances)(Cpa32U numDcInstances,
                                            CpaInstanceHandle *pDcInstances);
    CpaStatus (*icp_sal_userCyGetAvailableNumDynInstancesByDevPkg) (
                                                    Cpa32U *pNumCyInstances,
                                                    Cpa32U devPkgID);
    CpaStatus (*icp_sal_userDcGetAvailableNumDynInstancesByDevPkg) (
                                                    Cpa32U *pNumCyInstances,
                                                    Cpa32U devPkgID);
    CpaStatus (*icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel) (
                                            Cpa32U *pNumCyInstances,
                                            Cpa32U devPkgID,
                                            Cpa32U accelerator_number);
    CpaStatus (*icp_sal_userCyInstancesAllocByPkgAccel) (Cpa32U numCyInstances,
                                            CpaInstanceHandle *pCyInstances,
                                            Cpa32U devPkgID,
                                            Cpa32U accelerator_number);
    CpaStatus (*icp_sal_userCyInstancesAllocByDevPkg)(Cpa32U numCyInstances,
                                            CpaInstanceHandle *pCyInstances,
                                            Cpa32U devPkgID);
    CpaStatus (*icp_sal_userDcInstancesAllocByDevPkg)(Cpa32U numCyInstances,
                                            CpaInstanceHandle *pCyInstances,
                                            Cpa32U devPkgID);
    /*HB API's*/
    CpaStatus (*icp_sal_check_device) (Cpa32U accelId);
    CpaStatus (*icp_sal_check_all_devices) (void);
   /*PF-VF comms API's*/
    CpaStatus (*icp_sal_userGetPfVfcommsStatus)(CpaBoolean *unreadMessage);
    CpaStatus (*icp_sal_userGetMsgFromPf)(Cpa32U accelId, Cpa32U *message,
                                   Cpa32U *messageCounter);
    CpaStatus (*icp_sal_userGetMsgFromVf)(Cpa32U accelId, Cpa32U vfNum,
                                   Cpa32U *message, Cpa32U *messageCounter);
    CpaStatus (*icp_sal_userSendMsgToPf)(Cpa32U accelId, Cpa32U message);
    CpaStatus (*icp_sal_userSendMsgToVf)(Cpa32U accelId, Cpa32U vfNum,
                                        Cpa32U message);
    /*Reset API*/
    CpaStatus (*icp_sal_reset_device)(Cpa32U accelId);       
    /* Thread-less Mode API's*/ 
    CpaStatus (*icp_sal_poll_device_events) (void);
    CpaStatus (*icp_sal_find_new_devices) (void);

    /* Event based poll API's */
    CpaStatus (*icp_sal_CyGetFileDescriptor)(CpaInstanceHandle instanceHandle_in,
                                       int *fd);
    CpaStatus (*icp_sal_CyPutFileDescriptor)(CpaInstanceHandle instanceHandle_in,
                                       int fd);
    CpaStatus (*icp_sal_DcGetFileDescriptor)(CpaInstanceHandle instanceHandle_in,
                                       int *fd);
    CpaStatus (*icp_sal_DcPutFileDescriptor)(CpaInstanceHandle instanceHandle_in,
                                       int fd);
#endif

#ifdef ONE_KO_RELEASE_PACKAGE
    /* Additional polling symbols, declared in icp_sal_poll.h */
    CpaStatus (*icp_sal_pollBank) (Cpa32U accelId, Cpa32U bank_number,
                                   Cpa32U response_quota);
    CpaStatus (*icp_sal_pollAllBanks) (Cpa32U accelId,
                                       Cpa32U response_quota);
#endif

    /* iommu wrapper symbols, declared in icp_sal_iommu.h */
    size_t (*icp_sal_iommu_get_remap_size) (size_t size);
    CpaStatus (*icp_sal_iommu_map) (Cpa64U phaddr, Cpa64U iova, size_t size);
    CpaStatus (*icp_sal_iommu_unmap) (Cpa64U iova, size_t size);

    /* ADF internal API symbol, declared in icp_adf_accel_mgr.h */
    CpaStatus (*icp_amgr_getNumInstances)(Cpa16U *pNumInstances);
} CpaFuncPtrs;

/**
 *****************************************************************************
 * @file cpa_impl_mux.h
 * @ingroup cpaImplMux
 *      Register an implementation of the CPA API
 *
 * @description
 *      This function accepts registration of an implementation of the CPA
 *      API or, more specifically, an implementation of the functions defined
 *      in @ref CpaFuncPtrs.
 *
 *      This function can be used in kernel space and user-space.  The parameter
 *      list and the specific behaviour of the function differs slightly in
 *      each case.  In all cases, a handle is returned if registration is
 *      successful.  This handle is needed to de-register the implementation
 *      at a later time.
 *
 *      In kernel-space, this function accepts a list of function pointers as
 *      input, via the pImplFuncts parameter.  pDynLibPath is ignored.
 *
 *      In user-space, this function accepts a pathname to a shared library
 *      containing symbols for all of the function pointers in @ref CpaFuncPtrs.
 *      Symbol addresses will be extracted automatically using libdl. The
 *      pImplFuncts parameter will be ignored in this case.
 *
 * @context
 *      This function MUST NOT be called from an interrupt context as it MAY
 *      sleep.
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      This function is synchronous and blocking.
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in]     driverType         Identifies the registered driver/library
 *                                   implementation, based on its supported
 *                                   driver type
 * @param[in]     pDynLibPath        Pathname of shared library implementation
 *                                   for the CPA API.  For user-space use only.
 * @param[in]     pImplFuncts        List of function pointers for implemetation
 *                                   of the CPA API.  For kernel-space use only.
 * @param[in,out] implHandle         Pointer to where a CPA Mux implementation
 *                                   registration handle will be written.
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 *
 * @pre
 *      None
 * @post
 *      None
 *
 * @see
 *      cpaMuxDeRegisterImpl
 *
 *****************************************************************************/
CpaStatus cpaMuxRegisterImpl (CpaMuxDriverType driverType,
                              char *pDynLibPath,
                              CpaFuncPtrs * pImplFuncts,
                              void **implHandle);

/**
 *****************************************************************************
 * @file cpa_impl_mux.h
 * @ingroup cpaImplMux
 *      Re-register an implementation of the CPA API
 *
 * @description
 *      This function de-registration a specified implementation of the CPA
 *      API which was previously registered through @ref cpaMuxRegisterImpl.
 *
 * @context
 *      This function MUST NOT be called from an interrupt context as it MAY
 *      sleep.
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      This function is synchronous and blocking.
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in,out] implHandle         CPA Mux implementation registration handle
 *                                   returned by @ref cpaMuxRegisterImpl.
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in.
 *
 * @pre
 *      None
 * @post
 *      None
 *
 * @see
 *      cpaMuxRegisterImpl
 *
 *****************************************************************************/
CpaStatus cpaMuxDeRegisterImpl (void *implHandle);

#endif
