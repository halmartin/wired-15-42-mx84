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

/* These #defines are used index to a slice array to build a chain of qat
 * requests. */
#define LAC_ALG_CHAIN_QAT_SLICE_INDEX0            0
#define LAC_ALG_CHAIN_QAT_SLICE_INDEX1            1
#define LAC_ALG_CHAIN_QAT_SLICE_INDEX2            2

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


static void LacSymQat_LaSetDefaultFlags(Cpa16U *laCmdFlags)
{
    ICP_QAT_FW_LA_PARTIAL_SET(*laCmdFlags, ICP_QAT_FW_LA_PARTIAL_NONE);
    ICP_QAT_FW_LA_WR_STATE_SET(*laCmdFlags, ICP_QAT_FW_LA_NO_UPDATE_STATE);
    ICP_QAT_FW_LA_CMP_AUTH_SET(*laCmdFlags, ICP_QAT_FW_LA_NO_CMP_AUTH_RES);
    ICP_QAT_FW_LA_DIGEST_IN_BUFFER_SET(*laCmdFlags,
                                        ICP_QAT_FW_LA_NO_DIGEST_IN_BUFFER);
    /* no CCM/GCM/Snow3G */
    ICP_QAT_FW_LA_PROTO_SET(*laCmdFlags, ICP_QAT_FW_LA_NO_PROTO);
    ICP_QAT_FW_LA_RET_AUTH_SET(*laCmdFlags, ICP_QAT_FW_LA_NO_RET_AUTH_RES);
}



/** @ingroup LacAlgChain */
CpaStatus
LacAlgChain_SessionInit(const CpaInstanceHandle instanceHandle,
        const CpaCySymSessionSetupData *pSessionSetupData,
        lac_session_desc_t *pSessionDesc)
{
    CpaStatus stat, status = CPA_STATUS_SUCCESS;
    sal_qat_content_desc_info_t *pCdInfo = NULL;
    sal_crypto_service_t* pService = (sal_crypto_service_t*) instanceHandle;
    Cpa32U ctrlBlockHdrSz = 0;

    Cpa8U *pCtrlBlock = NULL;
    Cpa8U *pHwBlockBase = NULL;
    Cpa32U ctrlBlockOffset = 0;
    Cpa32U hwBlockOffset = 0;
    Cpa32U i = 0;
    Cpa32U authResSize = 0;
    Cpa8U prefixAadSizeQW = 0;
    Cpa8U reqParamsSize = 0;
    Cpa16U laCmdFlags = 0;
    icp_qat_fw_comn_flags cmnReqFlags = pService->cmnReqFlags;
    Cpa16U proto = ICP_QAT_FW_LA_NO_PROTO; /* no CCM/GCM/Snow3G */
    /* check parameters */
    LAC_ENSURE_NOT_NULL(pSessionSetupData);
    LAC_ENSURE_NOT_NULL(pSessionDesc);
    switch(pSessionSetupData->symOperation)
    {
        case CPA_CY_SYM_OP_CIPHER:
        {
            pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_CIPHER;
            pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX0] =
                ICP_QAT_FW_SLICE_CIPHER;
            pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX1] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSz = sizeof(icp_qat_fw_cipher_hdr_t);

            break;
        }
        case CPA_CY_SYM_OP_HASH:
        {
            pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_AUTH;
            pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX0] =
                ICP_QAT_FW_SLICE_AUTH;
            pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX1] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSz = sizeof(icp_qat_fw_auth_hdr_t);

            break;
        }
        case CPA_CY_SYM_OP_ALGORITHM_CHAINING:
        {
            CpaCySymAlgChainOrder chainOrder = pSessionSetupData->algChainOrder;
            pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_CIPHER_HASH;
            if (pSessionDesc->isAuthEncryptOp)
            {
                /* Ensure algChainOrder is set correctly for CCM/GCM ops */
                CpaCySymCipherAlgorithm cipherAlgorithm =
                    pSessionSetupData->cipherSetupData.cipherAlgorithm;
                CpaCySymCipherDirection cipherDir =
                    pSessionSetupData->cipherSetupData.cipherDirection;

                if ((LAC_CIPHER_IS_CCM(cipherAlgorithm) &&
                     (CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT == cipherDir)) ||
                    (LAC_CIPHER_IS_GCM(cipherAlgorithm) &&
                     (CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT == cipherDir)))
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH;
                }
                else
                {
                    chainOrder = CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER;
                }
            }

            if (CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH == chainOrder)
            {
                pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX0] =
                    ICP_QAT_FW_SLICE_CIPHER;
                pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX1] =
                    ICP_QAT_FW_SLICE_AUTH;
            }
            else if (CPA_CY_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER == chainOrder)
            {
                pSessionDesc->laCmdId = ICP_QAT_FW_LA_CMD_HASH_CIPHER;
                pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX0] =
                    ICP_QAT_FW_SLICE_AUTH;
                pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX1] =
                    ICP_QAT_FW_SLICE_CIPHER;
            }
            else
            {
                LAC_INVALID_PARAM_LOG("algChainOrder");
                status = CPA_STATUS_INVALID_PARAM;
                break;
            }
            pSessionDesc->qatSlices[LAC_ALG_CHAIN_QAT_SLICE_INDEX2] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSz = sizeof(icp_qat_fw_cipher_hdr_t) +
                sizeof(icp_qat_fw_auth_hdr_t);
            break;
        }
        /* Unsupported operation. Return error */
        default:
        {
            LAC_INVALID_PARAM_LOG("symOperation");
            status = CPA_STATUS_INVALID_PARAM;
            break;
        }
    }
    /* setup some convenience pointers */
    pCdInfo = &(pSessionDesc->contentDescInfo);
    pCtrlBlock = (Cpa8U *)pCdInfo->pData;
    pHwBlockBase = pCtrlBlock + ctrlBlockHdrSz;
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

    /* Reset the pending callback counter */
    osalAtomicSet(0, &pSessionDesc->pendingCbCount);
    osalAtomicSet(0, &pSessionDesc->pendingDpCbCount);

    /* Partial state must be set to full, to indicate that next packet
       * expected on the session is a full packet or the start of a
       * partial packet. */
    pSessionDesc->partialState = CPA_CY_SYM_PACKET_TYPE_FULL;

    LacSymQat_LaSetDefaultFlags(&laCmdFlags);
    /* walk the QAT slice chain, excluding last (terminating) entry */
    /* Note: the content descriptor control block order is not fixed
     * it is determined by the operation being performed*/
    for (i = 0; (i < MAX_NUM_QAT_SLICES - 1) &&
                (CPA_STATUS_SUCCESS == status); i++)
    {
        if (ICP_QAT_FW_SLICE_CIPHER == pSessionDesc->qatSlices[i])
        {
            const CpaCySymCipherSetupData
                        *pCipherSetupData = &pSessionSetupData->cipherSetupData;
            icp_qat_fw_cipher_hdr_t *pCipherControlBlock =
                (icp_qat_fw_cipher_hdr_t *)(pCtrlBlock + ctrlBlockOffset);
            Cpa32U cipherHwBlkSizeBytes = 0;
            reqParamsSize += sizeof(icp_qat_fw_la_cipher_req_params_t);
#ifdef ICP_PARAM_CHECK
            status = LacCipher_SessionSetupDataCheck(pCipherSetupData);
#endif

            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the cipher session information */
                pSessionDesc->cipherAlgorithm =
                                            pCipherSetupData->cipherAlgorithm;
                pSessionDesc->cipherKeyLenInBytes =
                                          pCipherSetupData->cipherKeyLenInBytes;
                pSessionDesc->cipherDirection =
                                            pCipherSetupData->cipherDirection;

                /* populate the cipher section of the content descriptor
                 * - the key length from cipher setup data is also used as
                 *   target key length.  No key padding is required.
                 */
                LacSymQat_CipherContentDescPopulate(
                    pCipherSetupData,
                    pCipherSetupData->cipherKeyLenInBytes,
                    pCipherControlBlock,
                    pHwBlockBase,
                    hwBlockOffset,
                    pSessionDesc->qatSlices[i + 1],
                    &cipherHwBlkSizeBytes);

                /* update offsets */
                ctrlBlockOffset += sizeof(icp_qat_fw_cipher_hdr_t);
                hwBlockOffset += cipherHwBlkSizeBytes;

                /* ARC4 base key isn't added to the content descriptor, because
                 * we don't need to pass it directly to the QAT engine. Instead
                 * an initial cipher state & key matrix is derived from the
                 * base key and provided to the QAT through the state pointer
                 * in the request params. We'll store this initial state in
                 * the session descriptor. */
                if (LAC_CIPHER_IS_ARC4(pCipherSetupData->cipherAlgorithm))
                {

                    LacSymQat_CipherArc4StateInit (
                        pCipherSetupData->pCipherKey,
                        pCipherSetupData->cipherKeyLenInBytes,
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
                         break;
                    }
                }

                /*
                 * Save some of the request parameters into the
                 * session
                 */
                pSessionDesc->cipherRequestParamsQATCache =
                    ((pCipherControlBlock->next_id
                               << LAC_SYM_QAT_CIPHER_NEXT_ID_BIT_OFFSET) +
                     (pCipherControlBlock->curr_id
                               << LAC_SYM_QAT_CIPHER_CURR_ID_BIT_OFFSET) +
                     (pCipherControlBlock->state_sz
                               << LAC_SYM_QAT_CIPHER_STATE_SIZE_BIT_OFFSET));
            }
        }
        else if (ICP_QAT_FW_SLICE_AUTH == pSessionDesc->qatSlices[i])
        {
            const CpaCySymHashSetupData *pHashSetupData =
                                        &pSessionSetupData->hashSetupData;
            lac_sym_qat_hash_precompute_info_t precomputeData = {0};
            icp_qat_fw_auth_hdr_t *pHashControlBlock =
                (icp_qat_fw_auth_hdr_t *)(pCtrlBlock + ctrlBlockOffset);
            Cpa32U hashHwBlockSizeBytes = 0;
            icp_qat_hw_auth_mode_t qatHashMode = ICP_QAT_HW_AUTH_MODE0;
            lac_sym_qat_hash_state_buffer_info_t *pHashStateBufferInfo =
                 &(pSessionDesc->hashStateBufferInfo);
            reqParamsSize += sizeof(icp_qat_fw_la_auth_req_params_t);
#ifdef ICP_PARAM_CHECK
            status = LacHash_HashContextCheck(instanceHandle, pHashSetupData);
#endif

            if (CPA_STATUS_SUCCESS == status)
            {
                /* set the hash session information */
                pSessionDesc->hashResultSize =
                    pHashSetupData->digestResultLenInBytes;
                pSessionDesc->hashMode = pHashSetupData->hashMode;
                pSessionDesc->hashAlgorithm = pHashSetupData->hashAlgorithm;

                if (CPA_TRUE == pSessionDesc->isAuthEncryptOp ||
                 pHashSetupData->hashAlgorithm == CPA_CY_SYM_HASH_SNOW3G_UIA2)
                {
                    pSessionDesc->aadLenInBytes =
                        pHashSetupData->authModeSetupData.aadLenInBytes;
                }

                /* Set the QAT hash mode */
                if ((pHashSetupData->hashMode == CPA_CY_SYM_HASH_MODE_NESTED) ||
                    (pHashSetupData->hashMode == CPA_CY_SYM_HASH_MODE_PLAIN)  ||
                    (pHashSetupData->hashMode == CPA_CY_SYM_HASH_MODE_AUTH
                         && pHashSetupData->hashAlgorithm
                             == CPA_CY_SYM_HASH_AES_CBC_MAC))
                {
                    qatHashMode = ICP_QAT_HW_AUTH_MODE0;
                }
                else
                {
                    if (IS_HMAC_ALG(pHashSetupData->hashAlgorithm))
                    {
                        /* HMAC Hash mode is determined by the config value */
                        qatHashMode = pService->qatHmacMode;
                    }
                    else
                    {
                        qatHashMode = ICP_QAT_HW_AUTH_MODE1;
                    }
                }
                /* populate the hash section of the content descriptor */
                LacSymQat_HashContentDescInit(instanceHandle,
                    pHashSetupData,
                    pHashControlBlock,
                    pHwBlockBase,
                    hwBlockOffset,
                    pSessionDesc->qatSlices[i + 1],
                    qatHashMode,
                    &precomputeData,
                    &hashHwBlockSizeBytes);

                /* update offsets */
                ctrlBlockOffset += sizeof(icp_qat_fw_auth_hdr_t);
                hwBlockOffset += hashHwBlockSizeBytes;
    
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
                                pHashSetupData, pHashControlBlock, qatHashMode,
                                pSessionDesc->hashStatePrefixBuffer,
                                pHashStateBufferInfo);
                }
            }
            if (CPA_STATUS_SUCCESS == status)
            {
                if (IS_HASH_MODE_1(qatHashMode)) 
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
                }
                else if(pHashSetupData->hashAlgorithm
                            == CPA_CY_SYM_HASH_AES_CBC_MAC)
                {
                    LAC_OS_BZERO(precomputeData.pState2,
                                 precomputeData.state2Size);
                    memcpy(precomputeData.pState2,
                        pHashSetupData->authModeSetupData.authKey,
                        pHashSetupData->authModeSetupData.authKeyLenInBytes);
                }
           }
           if (CPA_STATUS_SUCCESS == status)
           {

                /* For CCM & GCM modes: force digest verify flag _TRUE
                   for decrypt and _FALSE for encrypt. For all other cases
                   use user defined value */
                if ((CPA_TRUE == pSessionDesc->isAuthEncryptOp)
                   && (pSessionSetupData->cipherSetupData.cipherDirection
                           == CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT))
                 {
                        pSessionDesc->digestVerify = CPA_TRUE;
                 }
                 else if ((CPA_TRUE == pSessionDesc->isAuthEncryptOp)
                         && (pSessionSetupData->cipherSetupData.cipherDirection
                            == CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT))
                 {
                     pSessionDesc->digestVerify = CPA_FALSE;
                 }

                if (pSessionDesc->digestVerify)
                {
                    authResSize = pSessionDesc->hashResultSize;
                    ICP_QAT_FW_LA_CMP_AUTH_SET(
                            laCmdFlags, ICP_QAT_FW_LA_CMP_AUTH_RES);
                    ICP_QAT_FW_LA_RET_AUTH_SET(
                            laCmdFlags, ICP_QAT_FW_LA_NO_RET_AUTH_RES);
                }
                else
                {
                    authResSize = 0;
                    ICP_QAT_FW_LA_RET_AUTH_SET(
                           laCmdFlags, ICP_QAT_FW_LA_RET_AUTH_RES);
                    ICP_QAT_FW_LA_CMP_AUTH_SET(
                            laCmdFlags, ICP_QAT_FW_LA_NO_CMP_AUTH_RES);
                }

                if (CPA_CY_SYM_HASH_SNOW3G_UIA2 == pSessionDesc->hashAlgorithm)
                {
                     /* For Snow3G aadSize is equal to ivSize */
                    prefixAadSizeQW =
                        LAC_BYTES_TO_QUADWORDS(ICP_QAT_HW_SNOW_3G_UEA2_IV_SZ);
                }
                else if (CPA_TRUE == pSessionDesc->isAuthEncryptOp)
                {
                     /* For CCM and GCM use aadsize from content descriptor */
                    prefixAadSizeQW =
                       LAC_BYTES_TO_QUADWORDS(pHashControlBlock->u.aad_sz);
                }
                else
                {
                    /* Value calculated by LacHash_StatePrefixAadBufferInit */
                    prefixAadSizeQW =
                        pHashStateBufferInfo->prefixAadSzQuadWords;
                }
                pSessionDesc->hashRequestParamsQATCache =
                    ((pHashControlBlock->next_id
                            << LAC_SYM_QAT_HASH_NEXT_ID_BIT_OFFSET)+
                    (pHashControlBlock->curr_id
                            << LAC_SYM_QAT_HASH_CURR_ID_BIT_OFFSET)+
                    (prefixAadSizeQW
                            << LAC_SYM_QAT_HASH_PREFIX_SIZE_BIT_OFFSET)+
                    authResSize);
            }
        }
        /* chain terminators */
        else if (ICP_QAT_FW_SLICE_DRAM_WR == pSessionDesc->qatSlices[i])
        {
            /* end of chain */
            break;
        }
    } /* for (i = 0; ... */

    if (CPA_STATUS_SUCCESS == status)
    {
        /* configure the common fields of the content descriptor */
        pCdInfo->hdrSzQuadWords = LAC_BYTES_TO_QUADWORDS(ctrlBlockOffset);
        pCdInfo->hwBlkSzQuadWords = LAC_BYTES_TO_QUADWORDS(hwBlockOffset);

        /* create the request header */
        if(QAT_COMMS_PRIORITY_HIGH == pSessionDesc->qatSessionPriority)
        {
            SalQatMsg_ReqTypePopulate(
                        &pSessionDesc->reqCache.comn_hdr.arch_if,
                        ICP_ARCH_IF_REQ_QAT_FW_LA,
                        pService->symHiResponseRingId);
        }
        else
        {
            SalQatMsg_ReqTypePopulate(
                        &pSessionDesc->reqCache.comn_hdr.arch_if,
                        ICP_ARCH_IF_REQ_QAT_FW_LA,
                        pService->symLoResponseRingId);
        }

        /* check for CCM/GCM/Snow3G protocols */
        if (CPA_TRUE == pSessionDesc->isAuthEncryptOp)
        {
            if (LAC_CIPHER_IS_CCM(pSessionDesc->cipherAlgorithm))
            {
                pSessionDesc->digestIsAppended = CPA_TRUE;
                proto = ICP_QAT_FW_LA_CCM_PROTO;
            }
            else if (LAC_CIPHER_IS_GCM(pSessionDesc->cipherAlgorithm))
            {
                proto = ICP_QAT_FW_LA_GCM_PROTO;
            }
        }
        else if (((CPA_CY_SYM_OP_CIPHER == pSessionDesc->symOperation) &&
                 (CPA_CY_SYM_CIPHER_SNOW3G_UEA2 ==
                   pSessionDesc->cipherAlgorithm)) ||
                 (!(CPA_CY_SYM_OP_CIPHER == pSessionDesc->symOperation) &&
                 (CPA_CY_SYM_HASH_SNOW3G_UIA2 == pSessionDesc->hashAlgorithm)))
        {
             proto = ICP_QAT_FW_LA_SNOW_3G_PROTO;
        }

        ICP_QAT_FW_LA_PROTO_SET(laCmdFlags, proto);

        if (pSessionDesc->isDPSession == CPA_TRUE)
        {
          /* For DP api we want to set flatbuffers by default in the reqCache */
            ICP_QAT_FW_COMN_PTR_TYPE_SET(
            cmnReqFlags, QAT_COMN_PTR_TYPE_FLAT);
        }

         /* set Append flag, if digest is appended */
        if (pSessionDesc->digestIsAppended)
        {
           ICP_QAT_FW_LA_DIGEST_IN_BUFFER_SET(
                 laCmdFlags, ICP_QAT_FW_LA_DIGEST_IN_BUFFER);
        }

        /*
         * Setup what we can from the request cache.
         */
        SalQatMsg_MsgParamsPopulate(
                &(pSessionDesc->reqCache),
                pCdInfo,
                NULL,   /* pCookie */
                0,      /* srcBuffer */
                0,      /* dstBuffer */
                pSessionDesc->laCmdId,
                0,      /* reqParamsPhyAddr */
                reqParamsSize,
                laCmdFlags,
                cmnReqFlags,
                0,      /* flowId */
                0);     /* next_addr */
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
    sal_qat_content_desc_info_t *pCdInfo = NULL;
    Cpa8U *pCtrlBlock = NULL;
    Cpa8U *pReqParams = NULL;
    Cpa8U reqParamsOffset = 0;
    icp_qat_fw_la_bulk_req_t *pMsg = NULL;
    Cpa32U qatPacketType = 0;
    icp_qat_fw_la_cmd_id_t laCmdId = ICP_QAT_FW_LA_CMD_CIPHER;
    CpaBufferList *pBufferList = NULL;
    Cpa8U *pDigestResult = NULL;
    icp_qat_fw_auth_hdr_t *pHashControlBlock = NULL;
    icp_qat_fw_slice_t nextSlice = ICP_QAT_FW_SLICE_NULL;
    icp_qat_fw_comn_req_hdr_t *hdr = NULL;
    Cpa64U srcAddrPhys = 0;
    Cpa64U dstAddrPhys = 0;
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
        else if(CPA_CY_SYM_HASH_AES_GCM == pSessionDesc->hashAlgorithm)
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

    if (CPA_CY_SYM_HASH_AES_GMAC == pSessionDesc->hashAlgorithm &&
        pOpData->messageLenToCipherInBytes == 0)
    {
#ifdef ICP_PARAM_CHECK
        if (pSessionDesc->aadLenInBytes != 0 ||
            pOpData->pAdditionalAuthData != NULL)
        {
            LAC_INVALID_PARAM_LOG(" For AES_GMAC, aadLenInBytes must be zero"
                                  " and pAdditionalAuthData must be NULL");
            status = CPA_STATUS_INVALID_PARAM;
        }
#endif
    }

    /* setup some convenience pointers */
    pCdInfo = &(pSessionDesc->contentDescInfo);
    pCtrlBlock = (Cpa8U *)pCdInfo->pData;

    /* hash control block is used only for CCM and GCM (HASH_CIPHER or
    * CIPHER_HASH), the pointer is NULL in all other cases*/
    if (laCmdId == ICP_QAT_FW_LA_CMD_HASH_CIPHER)
    {
        pHashControlBlock = (icp_qat_fw_auth_hdr_t *)pCtrlBlock;
    }
    else
    {
        pHashControlBlock = (icp_qat_fw_auth_hdr_t *)
                (pCtrlBlock + sizeof(icp_qat_fw_cipher_hdr_t));
    }

    /* allocate cookie (used by callback function) */
    if (CPA_STATUS_SUCCESS == status)
    {

        do{
            pCookie =
                (lac_sym_bulk_cookie_t *)
                   Lac_MemPoolEntryAlloc(pService->lac_sym_cookie_pool);
            if(pCookie == NULL)
            {
                LAC_LOG_ERROR("Cannot allocate cookie - NULL");
                status = CPA_STATUS_RESOURCE;
            }
            else if ((void*)CPA_STATUS_RETRY == pCookie)
            {
#ifdef ICP_NONBLOCKING_PARTIALS_PERFORM
                status = CPA_STATUS_RETRY;
                break;
#else
                osalYield();
#endif
            }
        } while ((void*)CPA_STATUS_RETRY == pCookie);
        pSymCookie = (lac_sym_cookie_t *)pCookie;
    }

    /* allocate memory for cipher and hash request parameters */
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
        LAC_ASSERT_NOT_NULL(pCookie);
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
        pCookie->pNext = NULL;
        pCookie->instanceHandle = pService;

        /* Cookie contains space for request params */
        pReqParams = pCookie->reqParamsBuffer;
        pMsg = &(pCookie->qatMsg);

        /*
        * Fill in the initial bytes of the ET ring message - cached from the
        * session descriptor.
        */
        osalMemCopy((void*)pMsg,(void*) &(pSessionDesc->reqCache),
            SAL_SESSION_REQUEST_CACHE_SIZE_IN_BYTES);

        if (instanceHandle != pSessionDesc->pInstance) {
            hdr = &pMsg->comn_hdr;

            /* change the slices for the new instance */
            hdr->comn_req_flags = pService->cmnReqFlags;

            /*
             * We have copied the @pSessionDesc->reqCache to the @pMsg,
             * and then we will send out this message with
             * LacSymQueue_RequestSend().
             *
             * But in the LacAlgChain_SessionInit(), we have called the
             * SalQatMsg_ReqTypePopulate() to set the response ring id in the
             * @pSessionDesc->reqCache.comn_hdr.arch_if.resp_pipe_id with
             * the init instance's response ring id. And now, we will use
             * a different instance, so we should update the
             * @pMsg.comn_hdr.arch_if.resp_pipe_id with the new
             * instance's response ring id.
             */
            hdr->arch_if.resp_pipe_id =
                ((QAT_COMMS_PRIORITY_HIGH == pSessionDesc->qatSessionPriority) ?
                            pService->symHiResponseRingId :
                            pService->symLoResponseRingId);
        }

        /* populate the fields that were not in the cache */
        pMsg->comn_mid.src_data_addr = srcAddrPhys;
        pMsg->comn_mid.dest_data_addr = (dstAddrPhys)? dstAddrPhys: srcAddrPhys;
        pMsg->req_params_addr = pSymCookie->bulkReqParamsBufferPhyAddr;
        pMsg->comn_ftr.next_request_addr = 0;

        /* Populates the QAT common request middle part of the message
        * (LW 6 to 11) */
        LAC_MEM_SHARED_WRITE_FROM_PTR(pMsg->comn_mid.opaque_data, pCookie);

        /* get the qat packet type for LAC packet type */
        LacSymQat_packetTypeGet(pOpData->packetType,
                                pSessionDesc->partialState,
                                &qatPacketType);

        /* set the command flags based on the packet type */
        LacSymQat_LaPacketCommandFlagSet(
            qatPacketType,
            laCmdId,
            &pMsg->comn_la_req.u.la_flags);

#ifdef ICP_PARAM_CHECK
        LacBuffDesc_BufferListTotalSizeGet(pSrcBuffer, &srcPktSize);
#endif
        /* the order of the request parameters is now fixed: Cipher then Hash*/
        if (laCmdId != ICP_QAT_FW_LA_CMD_AUTH)
        {
            icp_qat_fw_la_cipher_req_params_t *pCipherReqParams =
                (icp_qat_fw_la_cipher_req_params_t *)
                (pReqParams + reqParamsOffset);
            Cpa32U cipherOffsetInBytes =
                pOpData->cryptoStartSrcOffsetInBytes;
            Cpa32U cipherLenInBytes = pOpData->messageLenToCipherInBytes;
            Cpa8U *pIvBuffer = NULL;
            Cpa64U ivBufPhyAddr = 0;

            LAC_ENSURE_NOT_NULL(pCipherReqParams);

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
            if ((laCmdId == ICP_QAT_FW_LA_CMD_CIPHER) ||
                (laCmdId == ICP_QAT_FW_LA_CMD_HASH_CIPHER))
            {
                nextSlice = ICP_QAT_FW_SLICE_DRAM_WR;
            }
            else
            {
                nextSlice = ICP_QAT_FW_SLICE_AUTH;
            }

            if (CPA_STATUS_SUCCESS == status)
            {
                /* align cipher IV */
                status = LacCipher_PerformIvCheck(
                    &(pService->generic_service_info), pCookie,
                    qatPacketType, &pIvBuffer);
            }

            if (CPA_STATUS_SUCCESS == status)
            {
                /* populate the cipher request parameters */
                if(pIvBuffer != NULL)
                {
                    /* User OpData memory being used for IV */
                    ivBufPhyAddr =
                        LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                                       pService->generic_service_info,
                                       pIvBuffer);
                    if (0 == ivBufPhyAddr)
                    {
                        LAC_LOG_ERROR("Unable to get the physical address "
                                "of the IV\n");
                        status = CPA_STATUS_FAIL;
                    }
                }
            }
            if (CPA_STATUS_SUCCESS == status)
            {
                LacSymQat_CipherRequestParamsPopulate(
                    pCipherReqParams,
                    nextSlice,
                    pSessionDesc->cipherAlgorithm,
                    cipherOffsetInBytes,
                    cipherLenInBytes,
                    ivBufPhyAddr);

                /* update offset */
                reqParamsOffset += sizeof(*pCipherReqParams);

                /* For ECB-mode ciphers, IV is NULL so update-state flag
                 * must be unset (normally set for all partial packets) */
                if ((CPA_CY_SYM_PACKET_TYPE_PARTIAL == pOpData->packetType)
                    &&
                    LAC_CIPHER_IS_ECB_MODE(pSessionDesc->cipherAlgorithm))
                {
                    ICP_QAT_FW_LA_WR_STATE_SET(
                                    pMsg->comn_la_req.u.la_flags,
                                  ICP_QAT_FW_LA_NO_UPDATE_STATE);
                }
            }
        }
        if (laCmdId != ICP_QAT_FW_LA_CMD_CIPHER)
        {
            icp_qat_fw_la_auth_req_params_t *pHashReqParams =
                (icp_qat_fw_la_auth_req_params_t *)
                (pReqParams + reqParamsOffset);
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

                if (ICP_QAT_FW_LA_CMD_AUTH == laCmdId)
                {
                    if ((pSrcBuffer == pDstBuffer) &&
                        (pSessionDesc->digestIsAppended == CPA_FALSE))
                    {
                        /* performance optimisation for in place auth
                         * only operation */
                        nextSlice = ICP_QAT_FW_SLICE_NULL;
                    }
                    else
                    {
                        nextSlice = ICP_QAT_FW_SLICE_DRAM_WR;
                    }
                }
                else if (ICP_QAT_FW_LA_CMD_HASH_CIPHER == laCmdId)
                {
                    nextSlice = ICP_QAT_FW_SLICE_CIPHER;
                }
                else
                {
                    /* cipher hash case */
                    nextSlice = ICP_QAT_FW_SLICE_DRAM_WR;
                }

                if (CPA_TRUE == pSessionDesc->isAuthEncryptOp)
                {

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
                        LAC_BYTES_TO_QUADWORDS(pHashControlBlock->u.aad_sz);

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
                                    pHashControlBlock->final_sz))
                    {
                        status = CPA_STATUS_INVALID_PARAM;
                    }
                }
#endif
                if (CPA_STATUS_SUCCESS == status)
                {
                    /* populate the hash request parameters */
                    LacSymQat_HashRequestParamsPopulate(
                        &(pService->generic_service_info),
                        pHashReqParams,
                        nextSlice,
                        pHashStateBufferInfo,
                        qatPacketType,
                        pSessionDesc->hashResultSize,
                        authOffsetInBytes,
                        authLenInBytes,
                        pSessionDesc->digestVerify,
                        pSessionDesc->digestIsAppended ? NULL : pDigestResult,
                        pSessionDesc->hashAlgorithm);
                }
            }
            reqParamsOffset += sizeof(*pHashReqParams);
        }
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        if ((LAC_CIPHER_IS_GCM(pSessionDesc->cipherAlgorithm)) &&
            (LAC_CIPHER_IV_SIZE_GCM_12 == pOpData->ivLenInBytes))

        {
            ICP_QAT_FW_LA_GCM_IV_LEN_FLAG_SET(pMsg->comn_la_req.u.la_flags,
                                        ICP_QAT_FW_LA_GCM_IV_LEN_12_OCTETS);
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* send the message to the QAT */
        status = LacSymQueue_RequestSend(
                instanceHandle,
                pCookie,
                pSessionDesc);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        osalAtomicInc(&(pSessionDesc->pendingCbCount));
    }
    /* Case that will catch all error status's for this function */
    else
    {
        /* free the cookie */
        if ((NULL != pCookie) && (((void*)CPA_STATUS_RETRY) != pCookie))
        {
            Lac_MemPoolEntryFree(pCookie);
        }
    }
    return status;
}
