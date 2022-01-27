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
 * @file lac_sym_alg_chain.c      Algorithm Chaining Perform
 *
 * @ingroup LacAlgChain
 ***************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/

#include "cpa.h"
#include "cpa_cy_sym.h"

#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/

#include "lac_mem.h"
#include "lac_log.h"
#include "lac_sym.h"
#include "lac_list.h"
#include "icp_qat_fw_la.h"
#include "lac_sal_types_crypto.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "lac_sym_alg_chain.h"
#include "lac_sym_cipher.h"
#include "lac_sym_cipher_defs.h"
#include "lac_sym_hash.h"
#include "lac_sym_hash_defs.h"
#include "lac_sym_qat_cipher.h"
#include "lac_sym_qat_hash.h"
#include "lac_sym_stats.h"
#include "lac_sym_queue.h"
#include "lac_sym_cb.h"
#include "sal_string_parse.h"
#include "lac_sym_auth_enc.h"
#include "lac_sym_qat.h"

/**
 * @ingroup LacAlgChain
 * This callback function will be invoked whenever a hash precompute
 * operation completes.  It will dequeue and send any QAT requests
 * which were queued up while the precompute was in progress.
 *
 * @param[in] callbackTag  Opaque value provided by user. This will
 *                         be a pointer to the session descriptor.
 *
 * @retval
 *     None
 *
 */
STATIC void LacSymAlgChain_HashPrecomputeDoneCb(void *callbackTag)
{
    LacSymCb_PendingReqsDequeue((lac_session_desc_t *)callbackTag);
}

/**
 * @ingroup LacAlgChain
 * Walk the buffer list and find the address for the given offset within
 * a buffer.
 *
 * @param[in] pBufferList   Buffer List
 * @param[in] packetOffset  Offset in the buffer list for which address
 *                          is to be found.
 * @param[out] ppDataPtr    This is where the sought pointer will be put
 * @param[out] pSpaceLeft   Pointer to a variable in which information about
 *                          available space from the given offset to the end
 *                          of the flat buffer it is located in will be returned
 *
 * @retval CPA_STATUS_SUCCESS Address with a given offset is found in the list
 * @retval CPA_STATUS_FAIL    Address with a given offset not found in the list.
 *
 */
STATIC CpaStatus
LacSymAlgChain_PtrFromOffsetGet(const CpaBufferList *pBufferList,
                                const Cpa32U packetOffset,
                                Cpa8U **ppDataPtr)
{
    Cpa32U currentOffset = 0;
    Cpa32U i = 0;

    for (i = 0; i < pBufferList->numBuffers; i++)
    {
        Cpa8U *pCurrData = pBufferList->pBuffers[i].pData;
        Cpa32U currDataSize = pBufferList->pBuffers[i].dataLenInBytes;

        /* If the offset is within the address space of the current buffer */
        if ((packetOffset >= currentOffset)
                && (packetOffset < (currentOffset + currDataSize)))
        {
            /* increment by offset of the address in the current buffer */
            *ppDataPtr = pCurrData + (packetOffset - currentOffset);
            return CPA_STATUS_SUCCESS;
        }

        /* Increment by the size of the buffer */
        currentOffset += currDataSize;
    }

    return CPA_STATUS_FAIL;
}

static void
LacAlgChain_CipherCDBuild_ForOptimisedCD(const CpaCySymCipherSetupData * pCipherData,
                                         lac_session_desc_t *pSessionDesc,
                                         icp_qat_fw_slice_t nextSlice,
                                         Cpa8U cipherOffsetInConstantsTable,
                                         Cpa8U *pOptimisedHwBlockBaseInDRAM,
                                         Cpa32U *pOptimisedHwBlockOffsetInDRAM)
{
    Cpa8U * pCipherKeyField = NULL;
    Cpa32U sizeInBytes = 0;
    pCipherKeyField = pOptimisedHwBlockBaseInDRAM;

    /* Need to build up the alternative CD for SHRAM Constants Table use with
     * an optimised content desc of 64 bytes for this case.
     * Cipher key will be in the Content desc in DRAM, The cipher config data 
     * is now in the SHRAM constants table. */

    LacSymQat_CipherHwBlockPopulateKeySetup(pCipherData,
                                            pCipherData->cipherKeyLenInBytes,
                                            pCipherKeyField,
                                            &sizeInBytes);

    LacSymQat_CipherCtrlBlockWrite(&(pSessionDesc->shramReqCacheFtr),
                                   pSessionDesc->cipherAlgorithm,
                                   pSessionDesc->cipherKeyLenInBytes,
                                   nextSlice,
                                   cipherOffsetInConstantsTable);

    *pOptimisedHwBlockOffsetInDRAM += sizeInBytes;
}

static void
LacAlgChain_CipherCDBuild_ForSHRAM(const CpaCySymCipherSetupData * pCipherData,
                                         lac_session_desc_t *pSessionDesc,
                                         icp_qat_fw_slice_t nextSlice,
                                         Cpa8U cipherOffsetInConstantsTable)
{
    Cpa32U sizeInBytes = 0;
    Cpa8U * pCipherKeyField = NULL;
    /* Need to build up the alternative CD for SHRAM Constants Table use
     * Cipher key will be in the Request, The cipher config data is now in the
     * SHRAM constants table. And nothing is now stored in the content desc */
    pCipherKeyField =
        (Cpa8U *)&(pSessionDesc->shramReqCacheHdr.cd_pars.s1.serv_specif_fields);

    LacSymQat_CipherHwBlockPopulateKeySetup(pCipherData,
                                            pCipherData->cipherKeyLenInBytes,
                                            pCipherKeyField,
                                            &sizeInBytes);

    LacSymQat_CipherCtrlBlockWrite(&(pSessionDesc->shramReqCacheFtr),
                                   pSessionDesc->cipherAlgorithm,
                                   pSessionDesc->cipherKeyLenInBytes,
                                   nextSlice,
                                   cipherOffsetInConstantsTable);

}

static void
LacAlgChain_CipherCDBuild(const CpaCySymCipherSetupData * pCipherData,
                          lac_session_desc_t *pSessionDesc,
                          icp_qat_fw_slice_t nextSlice,
                          Cpa8U cipherOffsetInConstantsTable,
                          icp_qat_fw_comn_flags *pCmnRequestFlags,
                          icp_qat_fw_serv_specif_flags *pLaCmdFlags,
                          Cpa8U *pHwBlockBaseInDRAM,
                          Cpa32U *pHwBlockOffsetInDRAM)
{
    Cpa8U * pCipherKeyField = NULL;
    Cpa8U cipherOffsetInReqQW = 0;
    Cpa32U sizeInBytes = 0;

    LAC_ENSURE_NOT_NULL(pHwBlockBaseInDRAM);
    LAC_ENSURE_NOT_NULL(pHwBlockOffsetInDRAM);

    /* Construct the ContentDescriptor in DRAM */
    cipherOffsetInReqQW = (*pHwBlockOffsetInDRAM / LAC_QUAD_WORD_IN_BYTES);
    ICP_QAT_FW_LA_CIPH_AUTH_CFG_OFFSET_FLAG_SET(*pLaCmdFlags,
                                   ICP_QAT_FW_CIPH_AUTH_CFG_OFFSET_IN_CD_SETUP);

    /* construct cipherConfig in CD in DRAM */
    LacSymQat_CipherHwBlockPopulateCfgData(pSessionDesc,
                                      pHwBlockBaseInDRAM + *pHwBlockOffsetInDRAM,
                                      &sizeInBytes);

    *pHwBlockOffsetInDRAM += sizeInBytes;

    /* Cipher key will be in CD in DRAM.
     * The Request contains a ptr to the CD.
     * This ptr will be copied into the request later once the CD is
     * fully constructed, but the flag is set here.  */
    pCipherKeyField = pHwBlockBaseInDRAM + *pHwBlockOffsetInDRAM;
    ICP_QAT_FW_COMN_CD_FLD_TYPE_SET(*pCmnRequestFlags,
                                    QAT_COMN_CD_FLD_TYPE_64BIT_ADR);

    LacSymQat_CipherHwBlockPopulateKeySetup(pCipherData,
                                            pCipherData->cipherKeyLenInBytes,
                                            pCipherKeyField,
                                            &sizeInBytes);
    /* update offset */
    *pHwBlockOffsetInDRAM += sizeInBytes;

    LacSymQat_CipherCtrlBlockWrite(&(pSessionDesc->reqCacheFtr),
                                   pSessionDesc->cipherAlgorithm,
                                   pSessionDesc->cipherKeyLenInBytes,
                                   nextSlice,
                                   cipherOffsetInReqQW);

}

static void
LacAlgChain_HashCDBuild(const CpaCySymHashSetupData *pHashData,
                        CpaInstanceHandle instanceHandle,
                        lac_session_desc_t *pSessionDesc,
                        icp_qat_fw_slice_t nextSlice,
                        Cpa8U hashOffsetInConstantsTable,
                        icp_qat_fw_comn_flags *pCmnRequestFlags,
                        icp_qat_fw_serv_specif_flags *pLaCmdFlags,
                        lac_sym_qat_hash_precompute_info_t *pPrecomputeData,
                        lac_sym_qat_hash_precompute_info_t *pPrecomputeDataOptimisedCd,
                        Cpa8U *pHwBlockBaseInDRAM,
                        Cpa32U *pHwBlockOffsetInDRAM,
                        Cpa8U * pOptimisedHwBlockBaseInDRAM,
                        Cpa32U *pOptimisedHwBlockOffsetInDRAM
                        )
{
    Cpa32U sizeInBytes = 0;
    Cpa32U hwBlockOffsetInQuadWords =
        *pHwBlockOffsetInDRAM/LAC_QUAD_WORD_IN_BYTES;

    /* build:
     * - the hash part of the ContentDescriptor in DRAM */
    /* - the hash part of the CD control block in the Request template */
    LacSymQat_HashContentDescInit(&(pSessionDesc->reqCacheFtr),
                                  instanceHandle,
                                  pHashData,
                                  pHwBlockBaseInDRAM,
                                  hwBlockOffsetInQuadWords,
                                  nextSlice,
                                  pSessionDesc->qatHashMode,
                                  CPA_FALSE,
                                  CPA_FALSE,
                                  pPrecomputeData,
                                  &sizeInBytes);

    /* Using DRAM CD so update offset */
    *pHwBlockOffsetInDRAM += sizeInBytes;

    sizeInBytes = 0;

    if (pSessionDesc->useOptimisedContentDesc )
    {
        LacSymQat_HashContentDescInit(&(pSessionDesc->shramReqCacheFtr),
                                      instanceHandle,
                                      pHashData,
                                      pOptimisedHwBlockBaseInDRAM,
                                      hashOffsetInConstantsTable,
                                      nextSlice,
                                      pSessionDesc->qatHashMode,
                                      CPA_TRUE,
                                      CPA_TRUE,
                                      pPrecomputeDataOptimisedCd,
                                      &sizeInBytes);

        *pOptimisedHwBlockOffsetInDRAM += sizeInBytes;
    }

    if (pSessionDesc->useSymConstantsTable && !pSessionDesc->useOptimisedContentDesc )
    {
        /* Need to build up the alternative CD for SHRAM Constants Table use */
        LacSymQat_HashContentDescInit(&(pSessionDesc->shramReqCacheFtr),
                                      instanceHandle,
                                      pHashData,
                                      pHwBlockBaseInDRAM,
                                      hashOffsetInConstantsTable,
                                      nextSlice,
                                      pSessionDesc->qatHashMode,
                                      CPA_TRUE,
                                      CPA_FALSE,
                                      pPrecomputeData,
                                      &sizeInBytes);
    }
}

/** @ingroup LacAlgChain */
CpaStatus
LacAlgChain_SessionInit(const CpaInstanceHandle instanceHandle,
        const CpaCySymSessionSetupData *pSessionSetupData,
        CpaBoolean disableShramConstants,
        lac_session_desc_t *pSessionDesc)
{
    CpaStatus stat, status = CPA_STATUS_SUCCESS;
    sal_qat_content_desc_info_t *pCdInfo = NULL;
    sal_qat_content_desc_info_t *pCdInfoOptimised = NULL;
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    Cpa8U *pHwBlockBaseInDRAM = NULL;
    Cpa8U *pOptimisedHwBlockBaseInDRAM = NULL;
    Cpa32U hwBlockOffsetInDRAM = 0;
    Cpa32U optimisedHwBlockOffsetInDRAM = 0;
    Cpa8U cipherOffsetInConstantsTable = 0;
    Cpa8U hashOffsetInConstantsTable = 0;
    icp_qat_fw_serv_specif_flags laCmdFlags = 0;
    icp_qat_fw_comn_flags cmnRequestFlags = 0;
    icp_qat_fw_comn_req_t *pMsg = NULL;
    icp_qat_fw_comn_req_t *pMsgS = NULL;
    const CpaCySymCipherSetupData * pCipherData;
    const CpaCySymHashSetupData *pHashData;
    Cpa16U proto = ICP_QAT_FW_LA_NO_PROTO; /* no CCM/GCM/Snow3G */
    CpaCySymAlgChainOrder chainOrder = 0;
    lac_sym_qat_hash_precompute_info_t precomputeData = {0};
    lac_sym_qat_hash_precompute_info_t precomputeDataOptimisedCd = {0};

    LAC_ENSURE_NOT_NULL(instanceHandle);
    LAC_ENSURE_NOT_NULL(pSessionSetupData);
    LAC_ENSURE_NOT_NULL(pSessionDesc);

    pCipherData = &(pSessionSetupData->cipherSetupData);
    pHashData = &(pSessionSetupData->hashSetupData);

    /*-------------------------------------------------------------------------
     * Populate session data
     *-----------------------------------------------------------------------*/

    /* Initialise Request Queue */
    stat = LAC_SPINLOCK_INIT(&pSessionDesc->requestQueueLock);
    if (CPA_STATUS_SUCCESS != stat)
    {
        LAC_LOG_ERROR("Spinlock init failed for sessionLock");
        return CPA_STATUS_RESOURCE;
    }

    pSessionDesc->pRequestQueueHead = NULL;
    pSessionDesc->pRequestQueueTail = NULL;
    pSessionDesc->nonBlockingOpsInProgress = CPA_TRUE;
    pSessionDesc->pInstance = instanceHandle;
    pSessionDesc->digestIsAppended = pSessionSetupData->digestIsAppended;
    pSessionDesc->digestVerify = pSessionSetupData->verifyDigest;

    /* Reset the pending callback counter */
    osalAtomicSet(0, &pSessionDesc->u.pendingCbCount);
    osalAtomicSet(0, &pSessionDesc->u.pendingDpCbCount);

    /* Partial state must be set to full, to indicate that next packet
       * expected on the session is a full packet or the start of a
       * partial packet. */
    pSessionDesc->partialState = CPA_CY_SYM_PACKET_TYPE_FULL;

    pSessionDesc->symOperation = pSessionSetupData->symOperation;
    switch(pSessionDesc->symOperation)
    {
    case CPA_CY_SYM_OP_CIPHER:
        pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_CIPHER;
        pSessionDesc->isCipher = TRUE;
        pSessionDesc->isAuth = FALSE;
        pSessionDesc->isAuthEncryptOp = CPA_FALSE;

        if (CPA_CY_SYM_CIPHER_SNOW3G_UEA2 ==
            pSessionSetupData->cipherSetupData.cipherAlgorithm)
        {
             proto = ICP_QAT_FW_LA_SNOW_3G_PROTO;
        }

        break;
    case CPA_CY_SYM_OP_HASH:
        pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_AUTH;
        pSessionDesc->isCipher = FALSE;
        pSessionDesc->isAuth = TRUE;
        pSessionDesc->isAuthEncryptOp = CPA_FALSE;

        if (CPA_CY_SYM_HASH_SNOW3G_UIA2 ==
            pSessionSetupData->hashSetupData.hashAlgorithm)
        {
             proto = ICP_QAT_FW_LA_SNOW_3G_PROTO;
        }

        break;
    case CPA_CY_SYM_OP_ALGORITHM_CHAINING:
        pSessionDesc->isCipher = TRUE;
        pSessionDesc->isAuth = TRUE;

        {
            /* set up some useful shortcuts */
            CpaCySymCipherAlgorithm cipherAlgorithm =
                    pSessionSetupData->cipherSetupData.cipherAlgorithm;
            CpaCySymCipherDirection cipherDir =
                    pSessionSetupData->cipherSetupData.cipherDirection;

            if (LAC_CIPHER_IS_CCM(cipherAlgorithm))
            {
                pSessionDesc->isAuthEncryptOp = CPA_TRUE;
                pSessionDesc->digestIsAppended = CPA_TRUE;
                proto = ICP_QAT_FW_LA_CCM_PROTO;

                /* Derive chainOrder from direction for isAuthEncryptOp cases */
                /* For CCM & GCM modes: force digest verify flag _TRUE
                   for decrypt and _FALSE for encrypt. For all other cases
                   use user defined value */

                if (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == cipherDir)
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER;
                    pSessionDesc->digestVerify = CPA_FALSE;
                }
                else
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH;
                    pSessionDesc->digestVerify = CPA_TRUE;
                }
            }
            else if (LAC_CIPHER_IS_GCM(cipherAlgorithm))
            {
                pSessionDesc->isAuthEncryptOp = CPA_TRUE;
                proto = ICP_QAT_FW_LA_GCM_PROTO;

                /* Derive chainOrder from direction for isAuthEncryptOp cases */
                /* For CCM & GCM modes: force digest verify flag _TRUE
                   for decrypt and _FALSE for encrypt. For all other cases
                   use user defined value */

                if (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == cipherDir)
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH;
                    pSessionDesc->digestVerify = CPA_FALSE;
                }
                else
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER;
                    pSessionDesc->digestVerify = CPA_TRUE;
                }
            }
            else
            {
                pSessionDesc->isAuthEncryptOp = CPA_FALSE;
                /* Use the chainOrder passed in */
                chainOrder = pSessionSetupData->algChainOrder;
#ifdef ICP_PARAM_CHECK
               if((chainOrder != CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER) &&
                  (chainOrder != CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH))
               {
                    LAC_INVALID_PARAM_LOG("algChainOrder");
                    return CPA_STATUS_INVALID_PARAM;
               }
#endif

                if (CPA_CY_SYM_HASH_SNOW3G_UIA2 ==
                    pSessionSetupData->hashSetupData.hashAlgorithm)
                {
                    proto = ICP_QAT_FW_LA_SNOW_3G_PROTO;
                }
            }

            if (CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH == chainOrder)
            {
                pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_CIPHER_HASH;
            }
            else if (CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER == chainOrder)
            {
                pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_HASH_CIPHER;
            }
        }
        break;
    default:break;
    }

    if (pSessionDesc->isCipher)
    {
        /* Populate cipher specific session data */

#ifdef ICP_PARAM_CHECK
            status = LacCipher_SessionSetupDataCheck(pCipherData);
#endif

        if (CPA_STATUS_SUCCESS == status)
        {
            pSessionDesc->cipherAlgorithm = pCipherData->cipherAlgorithm;
            pSessionDesc->cipherKeyLenInBytes = pCipherData->cipherKeyLenInBytes;
            pSessionDesc->cipherDirection = pCipherData->cipherDirection;

            /* ARC4 base key isn't added to the content descriptor, because
             * we don't need to pass it directly to the QAT engine. Instead
             * an initial cipher state & key matrix is derived from the
             * base key and provided to the QAT through the state pointer
             * in the request params. We'll store this initial state in
             * the session descriptor. */

            if (LAC_CIPHER_IS_ARC4(pSessionDesc->cipherAlgorithm))
            {
                LacSymQat_CipherArc4StateInit (
                        pCipherData->pCipherKey,
                        pSessionDesc->cipherKeyLenInBytes,
                        pSessionDesc->cipherARC4InitialState);

                pSessionDesc->cipherARC4InitialStatePhysAddr =
                        LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                pService->generic_service_info,
                                pSessionDesc->cipherARC4InitialState);

                if (0 == pSessionDesc->cipherARC4InitialStatePhysAddr)
                {
                    LAC_LOG_ERROR("Unable to get the physical address of "
                            "the initial state for ARC4\n");
                    status = CPA_STATUS_FAIL;
                }
            }
        }
    }

    if ((CPA_STATUS_SUCCESS == status) && pSessionDesc->isAuth)
    {
        /* Populate auth-specific session data */
        const CpaCySymHashSetupData *pHashData =
                                           &pSessionSetupData->hashSetupData;

#ifdef ICP_PARAM_CHECK
        status = LacHash_HashContextCheck(instanceHandle, pHashData);
#endif
        if (CPA_STATUS_SUCCESS == status)
        {
            pSessionDesc->hashResultSize = pHashData->digestResultLenInBytes;
            pSessionDesc->hashMode = pHashData->hashMode;
            pSessionDesc->hashAlgorithm = pHashData->hashAlgorithm;

            if (CPA_TRUE == pSessionDesc->isAuthEncryptOp ||
                    pHashData->hashAlgorithm == CPA_CY_SYM_HASH_SNOW3G_UIA2)
            {
                pSessionDesc->aadLenInBytes =
                        pHashData->authModeSetupData.aadLenInBytes;
            }

            /* Set the QAT hash mode */
            if ((pHashData->hashMode == CPA_CY_SYM_HASH_MODE_NESTED) ||
                (pHashData->hashMode == CPA_CY_SYM_HASH_MODE_PLAIN)  ||
                (pHashData->hashMode == CPA_CY_SYM_HASH_MODE_AUTH
                     && pHashData->hashAlgorithm == CPA_CY_SYM_HASH_AES_CBC_MAC))
            {
                pSessionDesc->qatHashMode = ICP_QAT_HW_AUTH_MODE0;
            }
            else /* CPA_CY_SYM_HASH_MODE_AUTH
                    && anything except CPA_CY_SYM_HASH_AES_CBC_MAC  */
            {
                if (IS_HMAC_ALG(pHashData->hashAlgorithm))
                {
                    /* HMAC Hash mode is determined by the config value */
                    pSessionDesc->qatHashMode = pService->qatHmacMode;
                }
                else
                {
                    pSessionDesc->qatHashMode = ICP_QAT_HW_AUTH_MODE1;
                }
            }
        }
    }

    /*-------------------------------------------------------------------------
     * build the message templates
     * create two content descriptors in the case we can support using SHRAM
     * constants and an optimised content descriptor. we have to do this in case
     * of partials.
     * 64 byte content desciptor is used in the SHRAM case for AES-128-HMAC-SHA1
     *-----------------------------------------------------------------------*/
    if (CPA_STATUS_SUCCESS == status)
    {
        pSessionDesc->useOptimisedContentDesc = CPA_FALSE;

        /* Check if SymConstantsTable can be used */
        if (disableShramConstants)
        {
            pSessionDesc->useSymConstantsTable = CPA_FALSE;
        }
        else
        {
            pSessionDesc->useSymConstantsTable =
                LacSymQat_UseSymConstantsTable(pSessionDesc,
                        &cipherOffsetInConstantsTable,
                        &hashOffsetInConstantsTable );
        }

        /* for a certain combination of Algorthm Chaining we want to *
         * use an optimised cd block */

        if(pSessionDesc->symOperation == CPA_CY_SYM_OP_ALGORITHM_CHAINING &&
                        pSessionDesc->useSymConstantsTable == CPA_TRUE)
        {
            pSessionDesc->useOptimisedContentDesc =
       	                LacSymQat_UseOptimisedContentDesc(pSessionDesc);
    	}

        /* setup some convenience pointers */
        pCdInfo = &(pSessionDesc->contentDescInfo);
        pHwBlockBaseInDRAM = (Cpa8U *)pCdInfo->pData;
        hwBlockOffsetInDRAM = 0;

        /* set up the pointer for the optimised content desc if this is possible
         * we still have to support both cd types in case of partials so 
         * we construct both */
        if (pSessionDesc->useOptimisedContentDesc == CPA_TRUE )
        {
            pCdInfoOptimised = &(pSessionDesc->contentDescOptimisedInfo);
            pOptimisedHwBlockBaseInDRAM = (Cpa8U *)pCdInfoOptimised->pData;
            optimisedHwBlockOffsetInDRAM = 0;
        }

        /*
         * Build the header flags with the default settings for this session.
         */
        if (pSessionDesc->isDPSession == CPA_TRUE)
        {
            cmnRequestFlags =
                    ICP_QAT_FW_COMN_FLAGS_BUILD(QAT_COMN_CD_FLD_TYPE_64BIT_ADR,
                            LAC_SYM_DP_QAT_PTR_TYPE);
        }
        else
        {
            cmnRequestFlags =
                    ICP_QAT_FW_COMN_FLAGS_BUILD(QAT_COMN_CD_FLD_TYPE_64BIT_ADR,
                            LAC_SYM_DEFAULT_QAT_PTR_TYPE);
        }

        LacSymQat_LaSetDefaultFlags(&laCmdFlags, pSessionDesc->symOperation);

        switch(pSessionDesc->symOperation)
        {
        case CPA_CY_SYM_OP_CIPHER:
        {
            LacAlgChain_CipherCDBuild(pCipherData,
                                      pSessionDesc,
                                      ICP_QAT_FW_SLICE_DRAM_WR,
                                      cipherOffsetInConstantsTable,
                                      &cmnRequestFlags,
                                      &laCmdFlags,
                                      pHwBlockBaseInDRAM,
                                      &hwBlockOffsetInDRAM);

            if (pSessionDesc->useSymConstantsTable)
            {
                LacAlgChain_CipherCDBuild_ForSHRAM(pCipherData,
                                                   pSessionDesc,
                                                   ICP_QAT_FW_SLICE_DRAM_WR,
                                                   cipherOffsetInConstantsTable);
            }
        }
            break;
        case CPA_CY_SYM_OP_HASH:
            LacAlgChain_HashCDBuild(pHashData,
                                    instanceHandle,
                                    pSessionDesc,
                                    ICP_QAT_FW_SLICE_DRAM_WR,
                                    hashOffsetInConstantsTable,
                                    &cmnRequestFlags,
                                    &laCmdFlags,
                                    &precomputeData,
                                    &precomputeDataOptimisedCd,
                                    pHwBlockBaseInDRAM,
                                    &hwBlockOffsetInDRAM,
                                    NULL,NULL);
            break;
        case CPA_CY_SYM_OP_ALGORITHM_CHAINING:
            /* For CCM/GCM, CPM firmware currently expects the cipher and hash
             * h/w setup blocks to be arranged according to the chain order
             * (Except for GCM/CCM, order doesn't actually matter as long as
             * the config offsets are set correctly in CD control blocks
             */
            if (CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER == chainOrder)
            {
                LacAlgChain_HashCDBuild(pHashData,
                                        instanceHandle,
                                        pSessionDesc,
                                        ICP_QAT_FW_SLICE_CIPHER,
                                        hashOffsetInConstantsTable,
                                        &cmnRequestFlags,
                                        &laCmdFlags,
                                        &precomputeData,
                                        &precomputeDataOptimisedCd,
                                        pHwBlockBaseInDRAM,
                                        &hwBlockOffsetInDRAM,
                                        pOptimisedHwBlockBaseInDRAM,
                                        &optimisedHwBlockOffsetInDRAM
                                        );

                LacAlgChain_CipherCDBuild(pCipherData,
                                          pSessionDesc,
                                          ICP_QAT_FW_SLICE_DRAM_WR,
                                          cipherOffsetInConstantsTable,
                                          &cmnRequestFlags,
                                          &laCmdFlags,
                                          pHwBlockBaseInDRAM,
                                          &hwBlockOffsetInDRAM);

                if (pSessionDesc->useOptimisedContentDesc)
                {
                    LacAlgChain_CipherCDBuild_ForOptimisedCD(pCipherData,
               	                          pSessionDesc,
                                          ICP_QAT_FW_SLICE_DRAM_WR,
                                          cipherOffsetInConstantsTable,
                                          pOptimisedHwBlockBaseInDRAM,
                                          &optimisedHwBlockOffsetInDRAM);
                }
            }
            else
            {
                LacAlgChain_CipherCDBuild(pCipherData,
                                          pSessionDesc,
                                          ICP_QAT_FW_SLICE_AUTH,
                                          cipherOffsetInConstantsTable,
                                          &cmnRequestFlags,
                                          &laCmdFlags,
                                          pHwBlockBaseInDRAM,
                                          &hwBlockOffsetInDRAM);

                if (pSessionDesc->useOptimisedContentDesc)
                {
                    LacAlgChain_CipherCDBuild_ForOptimisedCD(pCipherData,
                                          pSessionDesc,
                                          ICP_QAT_FW_SLICE_AUTH,
                                          cipherOffsetInConstantsTable,
                                          pOptimisedHwBlockBaseInDRAM,
                                          &optimisedHwBlockOffsetInDRAM);
                }

                LacAlgChain_HashCDBuild(pHashData,
                                        instanceHandle,
                                        pSessionDesc,
                                        ICP_QAT_FW_SLICE_DRAM_WR,
                                        hashOffsetInConstantsTable,
                                        &cmnRequestFlags,
                                        &laCmdFlags,
                                        &precomputeData,
                                        &precomputeDataOptimisedCd,
                                        pHwBlockBaseInDRAM,
                                        &hwBlockOffsetInDRAM,
                                        pOptimisedHwBlockBaseInDRAM,
                                        &optimisedHwBlockOffsetInDRAM);
            }
            break;
        default:
            LAC_LOG_ERROR("Invalid sym operation\n");
            status = CPA_STATUS_INVALID_PARAM;
        }
    }

    if ((CPA_STATUS_SUCCESS == status) && pSessionDesc->isAuth)
    {
        lac_sym_qat_hash_state_buffer_info_t *pHashStateBufferInfo =
            &(pSessionDesc->hashStateBufferInfo);
        CpaBoolean hashStateBuffer = CPA_TRUE;

        /* set up fields in both the cd_ctrl and reqParams which describe
         * the ReqParams block */
        LacSymQat_HashSetupReqParamsMetaData(&(pSessionDesc->reqCacheFtr),
                instanceHandle,
                pHashData,
                hashStateBuffer,
                pSessionDesc->qatHashMode,
                pSessionDesc->digestVerify);

        if (pSessionDesc->useSymConstantsTable)
        {
            /* Need to set up for SHRAM Constants Table use also */
            LacSymQat_HashSetupReqParamsMetaData(
                    &(pSessionDesc->shramReqCacheFtr),
                    instanceHandle,
                    pHashData,
                    hashStateBuffer,
                    pSessionDesc->qatHashMode,
                    pSessionDesc->digestVerify);
        }

        /* populate the hash state prefix buffer info structure
         * (part of user allocated session memory & the
         * buffer itself. For CCM/GCM the buffer is stored in the
         * cookie and is not initialised here) */
        if (CPA_FALSE == pSessionDesc->isAuthEncryptOp)
        {
#ifdef ICP_PARAM_CHECK
            LAC_CHECK_64_BYTE_ALIGNMENT(
                         &(pSessionDesc->hashStatePrefixBuffer[0]));
#endif
            status = LacHash_StatePrefixAadBufferInit(
                    &(pService->generic_service_info),
                    pHashData, 
                    &(pSessionDesc->reqCacheFtr),
                    pSessionDesc->qatHashMode,
                    pSessionDesc->hashStatePrefixBuffer,
                    pHashStateBufferInfo);
            /* SHRAM Constants Table not used for Auth-Enc */
        }

        if (CPA_STATUS_SUCCESS == status)
        {
            if (IS_HASH_MODE_1(pSessionDesc->qatHashMode))
            {
#ifdef ICP_PARAM_CHECK
                LAC_CHECK_64_BYTE_ALIGNMENT(
                             &(pSessionDesc->hashStatePrefixBuffer[0]));
#endif

                /* Block messages until precompute is completed */
                pSessionDesc->nonBlockingOpsInProgress = CPA_FALSE;
                status = LacHash_PrecomputeDataCreate(instanceHandle,
                            (CpaCySymSessionSetupData *)pSessionSetupData,
                            LacSymAlgChain_HashPrecomputeDoneCb,
                            pSessionDesc,
                            pSessionDesc->hashStatePrefixBuffer,
                            precomputeData.pState1,
                            precomputeData.pState2);
                if(pSessionDesc->useOptimisedContentDesc)
                {
                    status = LacHash_PrecomputeDataCreate(instanceHandle,
                            (CpaCySymSessionSetupData *)pSessionSetupData,
                            LacSymAlgChain_HashPrecomputeDoneCb,
                            pSessionDesc,
                            pSessionDesc->hashStatePrefixBuffer,
                            precomputeDataOptimisedCd.pState1,
                            precomputeDataOptimisedCd.pState2);
                }
            }
            else if(pHashData->hashAlgorithm
                        == CPA_CY_SYM_HASH_AES_CBC_MAC)
            {
                LAC_OS_BZERO(precomputeData.pState2,
                             precomputeData.state2Size);
                memcpy(precomputeData.pState2,
                    pHashData->authModeSetupData.authKey,
                    pHashData->authModeSetupData.authKeyLenInBytes);
            }
       }
       if (CPA_STATUS_SUCCESS == status)
       {

           if (pSessionDesc->digestVerify)
           {

               ICP_QAT_FW_LA_CMP_AUTH_SET(
                       laCmdFlags, ICP_QAT_FW_LA_CMP_AUTH_RES);
               ICP_QAT_FW_LA_RET_AUTH_SET(
                       laCmdFlags, ICP_QAT_FW_LA_NO_RET_AUTH_RES);
           }
           else
           {

               ICP_QAT_FW_LA_RET_AUTH_SET(
                       laCmdFlags, ICP_QAT_FW_LA_RET_AUTH_RES);
               ICP_QAT_FW_LA_CMP_AUTH_SET(
                       laCmdFlags, ICP_QAT_FW_LA_NO_CMP_AUTH_RES);
           }

       }
    }

    if (CPA_STATUS_SUCCESS == status)
    {

        /* Configure the ContentDescriptor field
            in the request if not done already */
        pCdInfo->hwBlkSzQuadWords = LAC_BYTES_TO_QUADWORDS(hwBlockOffsetInDRAM);
        pMsg = (icp_qat_fw_comn_req_t *)&(pSessionDesc->reqCacheHdr);
        SalQatMsg_ContentDescHdrWrite(
            (icp_qat_fw_comn_req_t *)pMsg,
            pCdInfo);

         pMsgS = (icp_qat_fw_comn_req_t *)&(pSessionDesc->shramReqCacheHdr);
        /*If we are using the optimised CD then
          we have to set this up correctly in the SHARM reqCache*/
        if (pSessionDesc->useOptimisedContentDesc)
        {
            pCdInfoOptimised->hwBlkSzQuadWords =
                           LAC_BYTES_TO_QUADWORDS(optimisedHwBlockOffsetInDRAM);
            SalQatMsg_ContentDescHdrWrite(
                (icp_qat_fw_comn_req_t *)pMsgS,
                pCdInfoOptimised);
        }

        /* Configure the common header */
        ICP_QAT_FW_LA_PROTO_SET(laCmdFlags, proto);

        /* set Append flag, if digest is appended */
        if (pSessionDesc->digestIsAppended)
        {
            ICP_QAT_FW_LA_DIGEST_IN_BUFFER_SET(laCmdFlags,
                    ICP_QAT_FW_LA_DIGEST_IN_BUFFER);
        }
        else
        {
            ICP_QAT_FW_LA_DIGEST_IN_BUFFER_SET(laCmdFlags,
                               ICP_QAT_FW_LA_NO_DIGEST_IN_BUFFER);
        }

        SalQatMsg_CmnHdrWrite(
            (icp_qat_fw_comn_req_t *)pMsg,
            ICP_QAT_FW_COMN_REQ_CPM_FW_LA,
            pSessionDesc->laCmdId,
            cmnRequestFlags,
            laCmdFlags);

        /* Need to duplicate if SHRAM Constants Table used */
        if (pSessionDesc->useSymConstantsTable)
        {
            ICP_QAT_FW_LA_CIPH_AUTH_CFG_OFFSET_FLAG_SET(
                laCmdFlags,
                ICP_QAT_FW_CIPH_AUTH_CFG_OFFSET_IN_SHRAM_CP);

            if (pSessionDesc->isCipher && !pSessionDesc->useOptimisedContentDesc )
            {
                ICP_QAT_FW_COMN_CD_FLD_TYPE_SET(
                    cmnRequestFlags,
                    QAT_COMN_CD_FLD_TYPE_16BYTE_DATA);
            }

            SalQatMsg_CmnHdrWrite(
                (icp_qat_fw_comn_req_t *)pMsgS,
                ICP_QAT_FW_COMN_REQ_CPM_FW_LA,
                pSessionDesc->laCmdId,
                cmnRequestFlags,
                laCmdFlags);
        }

    }

    return status;
}

/** @ingroup LacAlgChain */
CpaStatus
LacAlgChain_Perform(const CpaInstanceHandle instanceHandle,
    lac_session_desc_t *pSessionDesc,
    void *pCallbackTag,
    const CpaCySymOpData *pOpData,
    const CpaBufferList *pSrcBuffer,
    CpaBufferList *pDstBuffer,
    CpaBoolean *pVerifyResult)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    lac_sym_bulk_cookie_t * pCookie = NULL;
    lac_sym_cookie_t *pSymCookie = NULL;
    icp_qat_fw_la_bulk_req_t *pMsg = NULL;
    Cpa8U* pMsgDummy = NULL;
    Cpa8U* pCacheDummyHdr = NULL;
    Cpa8U* pCacheDummyFtr = NULL;
    Cpa32U qatPacketType = 0;
    CpaBufferList *pBufferList = NULL;
    Cpa8U *pDigestResult = NULL;
    Cpa64U srcAddrPhys = 0;
    Cpa64U dstAddrPhys = 0;
    icp_qat_fw_la_cmd_id_t laCmdId;
#ifdef ICP_PARAM_CHECK
    Cpa64U srcPktSize = 0;
#endif
    LAC_ENSURE_NOT_NULL(pSessionDesc);
    LAC_ENSURE_NOT_NULL(pOpData);

    /* Set the command id */
    laCmdId = pSessionDesc->laCmdId;

    if( CPA_TRUE == pSessionDesc->isAuthEncryptOp )
    {
        if( CPA_CY_SYM_HASH_AES_CCM == pSessionDesc->hashAlgorithm )
        {
#ifdef ICP_PARAM_CHECK
            status = LacSymAlgChain_CheckCCMData(pOpData->pAdditionalAuthData,
                            pOpData->pIv,
                            pOpData->messageLenToCipherInBytes,
                            pOpData->ivLenInBytes);
#endif
           if(CPA_STATUS_SUCCESS == status)
           {
               LacSymAlgChain_PrepareCCMData(pSessionDesc,
                            pOpData->pAdditionalAuthData,
                            pOpData->pIv,
                            pOpData->messageLenToCipherInBytes,
                            pOpData->ivLenInBytes);
           }

        }
        else if( CPA_CY_SYM_HASH_AES_GCM == pSessionDesc->hashAlgorithm)
        {
#ifdef ICP_PARAM_CHECK
           if(pSessionDesc->aadLenInBytes != 0 &&
                 pOpData->pAdditionalAuthData == NULL)
           {
               LAC_INVALID_PARAM_LOG("pAdditionalAuthData");
               status = CPA_STATUS_INVALID_PARAM;
           }
#endif
            if(CPA_STATUS_SUCCESS == status)
            {
                LacSymAlgChain_PrepareGCMData(pSessionDesc,
                      pOpData->pAdditionalAuthData);
            }
        }
    }

    /* allocate cookie (used by callback function) */
    if (CPA_STATUS_SUCCESS == status)
    {

        do{
            pSymCookie =
                    (lac_sym_cookie_t *)
                    Lac_MemPoolEntryAlloc(pService->lac_sym_cookie_pool);
            if(pSymCookie == NULL)
            {
                LAC_LOG_ERROR("Cannot allocate cookie - NULL");
                status = CPA_STATUS_RESOURCE;
            }
            else if ((void*)CPA_STATUS_RETRY == pSymCookie)
            {
#ifdef ICP_NONBLOCKING_PARTIALS_PERFORM
                status = CPA_STATUS_RETRY;
                break;
#else
                osalYield();
#endif
                }
            else
            {
                pCookie = &(pSymCookie->u.bulkCookie);
            }
        } while ((void*)CPA_STATUS_RETRY == pSymCookie);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* write the buffer descriptors */
        status = LacBuffDesc_BufferListDescWrite((CpaBufferList *)pSrcBuffer,
                                            &srcAddrPhys, CPA_FALSE,
                                            &(pService->generic_service_info));
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Unable to write src buffer descriptors");
        }

        /* For out of place operations */
        if ((pSrcBuffer != pDstBuffer) && (CPA_STATUS_SUCCESS == status))
        {
            status = LacBuffDesc_BufferListDescWrite(pDstBuffer,
                    &dstAddrPhys, CPA_FALSE,
                    &(pService->generic_service_info));
            if (CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("Unable to write dest buffer descriptors");
            }
        }
    }
#ifdef ICP_PARAM_CHECK
    LAC_ENSURE_NOT_NULL(pCookie);
#endif    
    if (CPA_STATUS_SUCCESS == status)
    {
        /* populate the cookie */
        pCookie->pCallbackTag = pCallbackTag;
        pCookie->sessionCtx = pOpData->sessionCtx;
        pCookie->pOpData = (const CpaCySymOpData *) pOpData;
        pCookie->pDstBuffer = pDstBuffer;
        pCookie->updateSessionIvOnSend = CPA_FALSE;
        pCookie->updateUserIvOnRecieve = CPA_FALSE;
        pCookie->updateKeySizeOnRecieve = CPA_FALSE;
        pCookie->pNext = NULL;
        pCookie->instanceHandle = pService;

        /* get the qat packet type for LAC packet type */
        LacSymQat_packetTypeGet(pOpData->packetType,
                pSessionDesc->partialState,
                &qatPacketType);
        /*
         * For XTS mode, the key size must be updated after
         * the first partial has been sent. Set a flag here so the
         * response knows to do this.
         */
        if ((laCmdId != ICP_QAT_FW_LA_CMD_AUTH) &&
                (CPA_CY_SYM_PACKET_TYPE_PARTIAL == pOpData->packetType) &&
                (LAC_CIPHER_IS_XTS_MODE(pSessionDesc->cipherAlgorithm)) &&
                (qatPacketType == ICP_QAT_FW_LA_PARTIAL_START))
        {
            pCookie->updateKeySizeOnRecieve = CPA_TRUE;
        }

        /*
        * Now create the Request.
        * Start by populating it from the cache in the session descriptor.
        */
        pMsg = &(pCookie->qatMsg);
        pMsgDummy = (Cpa8U*) pMsg;

        /* Normally, we want to use the SHRAM Constants Table if possible
         * for best performance (less DRAM accesses incurred by CPM).  But
         * we can't use it for partial-packet hash operations.  This is why
         * we build 2 versions of the message template at sessionInit,
         * one for SHRAM Constants Table usage and the other (default) for
         * Content Descriptor h/w setup data in DRAM.  And we chose between
         * them here on a per-request basis, when we know the packetType
         */
        if ((! pSessionDesc->useSymConstantsTable) ||
            (pSessionDesc->isAuth &&
             (CPA_CY_SYM_PACKET_TYPE_FULL != pOpData->packetType)))
        {

            pCacheDummyHdr = (Cpa8U*) &(pSessionDesc->reqCacheHdr);
            pCacheDummyFtr = (Cpa8U*) &(pSessionDesc->reqCacheFtr);
        }
        else
        {
            pCacheDummyHdr = (Cpa8U*) &(pSessionDesc->shramReqCacheHdr);
            pCacheDummyFtr = (Cpa8U*) &(pSessionDesc->shramReqCacheFtr);
        }
        osalMemCopy(pMsgDummy, pCacheDummyHdr,
                    (LAC_LONG_WORD_IN_BYTES * LAC_SIZE_OF_CACHE_HDR_IN_LW) );
        osalMemSet((pMsgDummy + (LAC_LONG_WORD_IN_BYTES * LAC_SIZE_OF_CACHE_HDR_IN_LW)), 0,
                    (LAC_LONG_WORD_IN_BYTES * LAC_SIZE_OF_CACHE_TO_CLEAR_IN_LW));
        osalMemCopy(pMsgDummy + (LAC_LONG_WORD_IN_BYTES * LAC_START_OF_CACHE_FTR_IN_LW),
                    pCacheDummyFtr,
                    (LAC_LONG_WORD_IN_BYTES * LAC_SIZE_OF_CACHE_FTR_IN_LW) );
        /*
         * Populate the comn_mid section
         */
        SalQatMsg_CmnMidWrite(pMsg,
                         pCookie,
                         LAC_SYM_DEFAULT_QAT_PTR_TYPE,
                         srcAddrPhys,
                         dstAddrPhys,
                         0,
                         0);
        /*
         * Populate the serv_specif_flags field of the Request header
         * Some of the flags are set up here.
         * Others are set up later when the RequestParams are set up.
         */
        LacSymQat_LaPacketCommandFlagSet(
            qatPacketType,
            laCmdId,
            pSessionDesc->cipherAlgorithm,
            &pMsg->comn_hdr.serv_specif_flags,
            pOpData->ivLenInBytes);

#ifdef ICP_PARAM_CHECK
        LacBuffDesc_BufferListTotalSizeGet(pSrcBuffer, &srcPktSize);
#endif

        /*
         * Populate the CipherRequestParams section of the Request
         */
        if (laCmdId != ICP_QAT_FW_LA_CMD_AUTH)
        {

            Cpa8U *pIvBuffer = NULL;


#ifdef ICP_PARAM_CHECK
            status = LacCipher_PerformParamCheck(
                        pSessionDesc->cipherAlgorithm,
                        pOpData,
                        srcPktSize);
            if (CPA_STATUS_SUCCESS != status)
            {
                /* free the cookie */
                if ((NULL != pCookie) && (((void*)CPA_STATUS_RETRY) != pCookie))
                {
                    Lac_MemPoolEntryFree(pCookie);
                }
                return status;
            }
#endif

            if (CPA_STATUS_SUCCESS == status)
            {
                /* align cipher IV */
                status = LacCipher_PerformIvCheck(
                    &(pService->generic_service_info), pCookie,
                    qatPacketType, &pIvBuffer);
            }
            /*populate the cipher request parameters */
            if (CPA_STATUS_SUCCESS == status)
            {
                Cpa64U ivBufferPhysAddr = 0;

                if (pIvBuffer != NULL)
                {
                    /* User OpData memory being used for IV buffer */
                    /* get the physical address */
                    ivBufferPhysAddr =
                            LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                    pService->generic_service_info,
                                    pIvBuffer);
                    if (0 == ivBufferPhysAddr)
                    {
                        LAC_LOG_ERROR("Unable to get the physical address "
                                "of the IV\n");
                        status = CPA_STATUS_FAIL;
                    }
                }

                if (status == CPA_STATUS_SUCCESS)
                {
                    status = LacSymQat_CipherRequestParamsPopulate(
                            pMsg,
                            pOpData->cryptoStartSrcOffsetInBytes,
                            pOpData->messageLenToCipherInBytes,
                            ivBufferPhysAddr,
                            pIvBuffer);
                }
            }
        }

        /*
         * Set up HashRequestParams part of Request
         */
        if ((status == CPA_STATUS_SUCCESS) &&
            (laCmdId != ICP_QAT_FW_LA_CMD_CIPHER))
        {
            Cpa32U authOffsetInBytes = pOpData->hashStartSrcOffsetInBytes;
            Cpa32U authLenInBytes = pOpData->messageLenToHashInBytes;

#ifdef ICP_PARAM_CHECK
            status = LacHash_PerformParamCheck(instanceHandle,
                       pSessionDesc, pOpData, srcPktSize, pVerifyResult);
            if(CPA_STATUS_SUCCESS != status)
            {
                /* free the cookie */
                if ((NULL != pCookie) && (((void*)CPA_STATUS_RETRY) != pCookie))
                {
                    Lac_MemPoolEntryFree(pCookie);
                }
                return status;
            }
#endif
            if (CPA_STATUS_SUCCESS == status)
            {
                /* Info structure for CCM/GCM */
                lac_sym_qat_hash_state_buffer_info_t hashStateBufferInfo = {0};
                lac_sym_qat_hash_state_buffer_info_t *pHashStateBufferInfo
                     = &(pSessionDesc->hashStateBufferInfo);

                if (CPA_TRUE == pSessionDesc->isAuthEncryptOp)
                {
                    icp_qat_fw_la_auth_req_params_t * pHashReqParams =
                        (icp_qat_fw_la_auth_req_params_t *)
                        ((Cpa8U*)&(pMsg->serv_specif_rqpars)+
                         ICP_QAT_FW_HASH_REQUEST_PARAMETERS_OFFSET);

                    hashStateBufferInfo.pData = pOpData->pAdditionalAuthData;
                    if (pOpData->pAdditionalAuthData == NULL)
                    {
                        hashStateBufferInfo.pDataPhys = 0;
                    }
                    else
                    {
                        hashStateBufferInfo.pDataPhys =
                            LAC_MEM_CAST_PTR_TO_UINT64(
                                LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                    pService->generic_service_info,
                                    pOpData->pAdditionalAuthData));
                    }

                    hashStateBufferInfo.stateStorageSzQuadWords = 0;
                    hashStateBufferInfo.prefixAadSzQuadWords =
                        LAC_BYTES_TO_QUADWORDS(pHashReqParams->u2.aad_sz);

                    /* Overwrite hash state buffer info structure pointer
                                    * with the one created for CCM/GCM */
                    pHashStateBufferInfo = &hashStateBufferInfo;

                    /* Aad buffer could be null in the GCM case */
                    if (0 == hashStateBufferInfo.pDataPhys &&
                       CPA_CY_SYM_HASH_AES_GCM !=
                                       pSessionDesc->hashAlgorithm &&
                       CPA_CY_SYM_HASH_AES_GMAC !=
                                       pSessionDesc->hashAlgorithm)
                    {
                        LAC_LOG_ERROR("Unable to get the physical address"
                            "of the AAD\n");
                        status = CPA_STATUS_FAIL;
                    }

                    /* for CCM/GCM the hash and cipher data regions
                                    * are equal */
                    authOffsetInBytes = pOpData->cryptoStartSrcOffsetInBytes;

                    if (pOpData->messageLenToCipherInBytes > 0)
                    {
                        authLenInBytes = pOpData->messageLenToCipherInBytes;
                    }

                }
                else if(CPA_CY_SYM_HASH_SNOW3G_UIA2 ==
                        pSessionDesc->hashAlgorithm)
                {
                    hashStateBufferInfo.pData =
                        pOpData->pAdditionalAuthData;
                    hashStateBufferInfo.pDataPhys =
                        LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                            pService->generic_service_info,
                            hashStateBufferInfo.pData);
                    hashStateBufferInfo.stateStorageSzQuadWords = 0;
                    hashStateBufferInfo.prefixAadSzQuadWords =
                       LAC_BYTES_TO_QUADWORDS(pSessionDesc->aadLenInBytes);

                    pHashStateBufferInfo = &hashStateBufferInfo;

                    if (0 == hashStateBufferInfo.pDataPhys)
                    {
                        LAC_LOG_ERROR("Unable to get the physical address"
                            "of the AAD\n");
                        status = CPA_STATUS_FAIL;
                    }
                }
                if(CPA_CY_SYM_HASH_AES_CCM == pSessionDesc->hashAlgorithm)
                {
                    if(CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT ==
                            pSessionDesc->cipherDirection)
                    {
                        /* On a decrypt path pSrcBuffer is used as this is
                         * where encrypted digest is located. Firmware
                         * uses encrypted digest for compare/verification*/
                        pBufferList = (CpaBufferList *)pSrcBuffer;
                    }
                    else
                    {
                        /* On an encrypt path pDstBuffer is used as this is
                         * where encrypted digest will be written */
                        pBufferList = (CpaBufferList *)pDstBuffer;
                    }
                    status = LacSymAlgChain_PtrFromOffsetGet(
                                    pBufferList,
                                    pOpData->cryptoStartSrcOffsetInBytes +
                                        pOpData->messageLenToCipherInBytes,
                                    &pDigestResult);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        LAC_LOG_ERROR("Cannot set digest pointer within the"
                              " buffer list - offset out of bounds");
                    }
                }
                else
                {
                    pDigestResult = pOpData->pDigestResult;
                }

                if (CPA_CY_SYM_OP_ALGORITHM_CHAINING
                                   == pSessionDesc->symOperation)
                {
                    /* In alg chaining mode, packets are not seen as partials
                     * for hash operations. Override to NONE.
                     */
                    qatPacketType = ICP_QAT_FW_LA_PARTIAL_NONE;
                }
#ifdef ICP_PARAM_CHECK
                if(CPA_TRUE == pSessionDesc->digestIsAppended)
                {
                    /*Check if the destination buffer can handle the digest
                     * if digestIsAppend is true*/
                    if(srcPktSize < (authOffsetInBytes + authLenInBytes +
                                    pSessionDesc->hashResultSize))
                    {
                        status = CPA_STATUS_INVALID_PARAM;
                    }
                }
#endif
                if (CPA_STATUS_SUCCESS == status)
                {
                    /* populate the hash request parameters */
                    status = LacSymQat_HashRequestParamsPopulate(
                          pMsg,
                          authOffsetInBytes,
                          authLenInBytes,
                          &(pService->generic_service_info),
                          pHashStateBufferInfo,
                          qatPacketType,
                          pSessionDesc->hashResultSize,
                          pSessionDesc->digestVerify,
                          pSessionDesc->digestIsAppended ? NULL : pDigestResult,
                          pSessionDesc->hashAlgorithm);
                }
            }
        }
    }

    /*
     * send the message to the QAT
     */
    if (CPA_STATUS_SUCCESS == status)
    {

        status = LacSymQueue_RequestSend(
                instanceHandle,
                pCookie,
                pSessionDesc);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        osalAtomicInc(&(pSessionDesc->u.pendingCbCount));
    }
    /* Case that will catch all error status's for this function */
    else
    {
        /* free the cookie */
        if (NULL != pSymCookie)
        {
            Lac_MemPoolEntryFree(pSymCookie);
        }
    }
    return status;
}
