/*
 ***************************************************************************
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
 * @file  qat_init.c
 *
 * @ingroup qat_ctrl
 *
 * @description
 *        This file contains the functions to be used in the init of the
 *        AEs.
 *
 *****************************************************************************/

#include "cpa.h"

/* Osal includes. */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "Osal.h"
/* ADF includes. */
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"
/* FW includes. */
#include "icp_qat_fw_init.h"
#include "icp_qat_fw_admin.h"
#include "icp_qat_fw_la.h"
/* SAL includes. */
#include "lac_sym.h"
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_string_parse.h"
/* QAT includes. */
#include "qat_ctrl.h"
#include "qat_init.h"
#include "qat_init_defs.h"
#include "qat_rings.h"
#include "qat_admin.h"


/*
 * Response Handler function which in turn calls the appropriate
 * response function depending on the response type. This function
 * is set as the callback function when creating the init/admin rings.
 */
void
QatCtrl_ResponseMsgHandler(icp_comms_trans_handle trans_handle,
                           void *pNewMsg)
{
    icp_arch_if_request_t qatRespType=0;
    icp_arch_if_resp_hdr_t *pRespHeader = NULL;
    qatal_init_cb_info_t* pCbInfo = NULL;
    qatal_admin_cb_info_t* pCbInfoAdmin = NULL;
    icp_qat_fw_init_resp_t *pInitRespMsg = NULL;
    icp_qat_fw_admin_resp_t *pAdminRespMsg = NULL;
    sal_qat_service_t* qatInstance = NULL;

    LAC_ENSURE(pNewMsg != NULL,
               "QatCtrl_ResponseMsgHandler - pNewMsg is NULL\n");

    /* Discover the resp type of the message received. */
    pRespHeader = (icp_arch_if_resp_hdr_t *)pNewMsg;
    qatRespType = pRespHeader->resp_type;

    LAC_ENSURE(qatRespType <  QAT_NUM_CMD_TYPES,
               "QatCtrl_ResponseMsgHandler - incorrect request type\n");
    /*
     * Depending on the response type we either have an
     * init msg or an admin msg.
     */
    if(ICP_ARCH_IF_REQ_QAT_FW_INIT == qatRespType)
    {
        pInitRespMsg = (icp_qat_fw_init_resp_t *)pNewMsg;
        LAC_MEM_SHARED_READ_TO_PTR(pInitRespMsg->comn_resp.opaque_data,
                                   pCbInfo);
        LAC_MEM_SHARED_READ_TO_PTR(pCbInfo->instance,qatInstance);
    }
    else if(ICP_ARCH_IF_REQ_QAT_FW_ADMIN == qatRespType)
    {
        pAdminRespMsg = (icp_qat_fw_admin_resp_t *)pNewMsg;
        LAC_MEM_SHARED_READ_TO_PTR(pAdminRespMsg->comn_resp.opaque_data,
                                   pCbInfoAdmin);
        LAC_MEM_SHARED_READ_TO_PTR(pCbInfoAdmin->instance,qatInstance);
    }

    LAC_ENSURE(qatInstance != NULL,
               "QatCtrl_ResponseMsgHandler - qatInstance is NULL\n");
    qatInstance->qatRespHandlerTbl[qatRespType]((void *)pNewMsg,
                                                  qatRespType);
}

/*
 * Init response handler function. The response message is checked
 * and the callback called.
 */
void
QatCtrl_InitRespHandler(void *pRespMsg, icp_arch_if_request_t msgType)
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa8U opStatus = ICP_QAT_FW_COMN_STATUS_FLAG_OK;
    Cpa8U qatalCmdId = 0;
    icp_qat_fw_init_resp_t *pInitRespMsg = (icp_qat_fw_init_resp_t *)pRespMsg;
    qatal_init_cb_info_t *pCbInfo = NULL;
    QatCtrl_GenCbFunc pCb = NULL;
    void *pCallbackTag = NULL;

    if (NULL == pRespMsg)
    {
        LAC_LOG_ERROR("Qatal_InitRespHandler.. Response message is NULL\n");
        return;
    }
    qatalCmdId = pInitRespMsg->comn_resp.serv_cmd_id;

    if (qatalCmdId >= ICP_QAT_FW_INIT_CMD_DELIMITER)
    {
        LAC_LOG_ERROR("Qatal_InitRespHandler.. Invalid init command ID\n");
        return;
    }
    /* Check status of the response message */
    opStatus = pInitRespMsg->comn_resp.comn_status;

    if (ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
        ICP_QAT_FW_COMN_RESP_INIT_ADMIN_STAT_GET(opStatus))
    {
        status = CPA_STATUS_SUCCESS;
    }
    else
    {
        LAC_LOG_ERROR("QatCtrl_InitRespHandler. Response messages invalid\n");
    }

    LAC_MEM_SHARED_READ_TO_PTR(pInitRespMsg->comn_resp.opaque_data, pCbInfo);

    if (NULL == (void*)pCbInfo->pCb)
    {
        LAC_LOG_ERROR("Qatal_InitRespHandler. Callback pointer is NULL\n");
        return;
    }
    pCb = (QatCtrl_GenCbFunc)pCbInfo->pCb;
    pCallbackTag = (void *)pCbInfo->pCallbackTag;
    pCb(pCallbackTag, status);
    LAC_OS_CAFREE(pCbInfo);
}

/*
 *  Register a callback to handle message responses for a given request type
 */
void
QatCtrl_ResponseCbSet(sal_qat_service_t* qatInstance)
{
    /* Copy the address of the function to be called when a response of the
       specifed reqtype is received */
    qatInstance->qatRespHandlerTbl[ICP_ARCH_IF_REQ_QAT_FW_INIT] =
      QatCtrl_InitRespHandler;

    qatInstance->qatRespHandlerTbl[ICP_ARCH_IF_REQ_QAT_FW_ADMIN] =
      QatCtrl_AdminRespHandler;

    return;
}

/*
 * Init Sync Callback function.
 */
void
QatCtrl_InitSyncCb(void *pCallbackTag, CpaStatus status)
{
    LacSync_GenWakeupSyncCaller(pCallbackTag, status);
}

/*
 * This function takes care of building both the SET_AE_INFO
 * msg and the SET_RING_INFO msg. The messages are first built
 * and then sent.
 */
CpaStatus
QatCtrl_SendInitMsg(CpaInstanceHandle device,
                    sal_qat_service_t* qat_instance,
                    icp_qat_fw_init_cmd_id_t initCmdId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0, aeIndex = 0;
    Cpa32U numReqs =0;

    /* The number of requests to be sent */
    if(ICP_QAT_FW_INIT_CMD_SET_AE_INFO == initCmdId)
    {
        /* qatInstance0 sends all the SET_AE_INFO messages */
        numReqs = qat_instance->num_aes;
    }
    else
    {
        numReqs = qat_instance->num_aes_per_qat;
    }

    for(i = 0; i < numReqs; i++)
    {
         aeIndex = ((qat_instance->generic_service_info.instance *
                         qat_instance->num_aes_per_qat) + i);

         if(((ICP_QAT_FW_INIT_CMD_SET_AE_INFO == initCmdId) ||
             (ICP_QAT_FW_INIT_CMD_SET_RING_INFO == initCmdId))&&
            (qat_instance->serviceMask[i%qat_instance->num_aes_per_qat]))
         {
                status = QatCtrl_InitMsgSendSync(device,
                                                 qat_instance,
                                                 initCmdId, aeIndex);
                if(CPA_STATUS_SUCCESS != status)
                {
                    LAC_LOG_ERROR2("Failed to send Init msg %d to AE %d",
                                          initCmdId, aeIndex);
                    return status;
                }
         }
         else
         {
              /* Init TRNG if CY is enabled by the AE */
              if (((ICP_QAT_FW_INIT_CMD_TRNG_ENABLE == initCmdId) ||
                 (ICP_QAT_FW_INIT_CMD_TRNG_DISABLE == initCmdId)) &&
                 (SAL_SERVICE_TYPE_CRYPTO_A & qat_instance->serviceMask[i]))
              {
                    status =  QatCtrl_InitMsgSendSync(device,
                                                  qat_instance,
                                                  initCmdId, aeIndex);
                    if(CPA_STATUS_SUCCESS != status)
                    {
                        LAC_LOG_ERROR1("Failed to send TRNG msg to AE %d",
                                       aeIndex);
                        return status;
                    }
               }
         }
    }
    return status;
}

/*
 * Create the Sync Cookie and then build and send the
 * init message. Once the callback is received the
 * cookie is destroyed.
 */
CpaStatus
QatCtrl_InitMsgSendSync(CpaInstanceHandle instanceHandle,
                        sal_qat_service_t* qat_instance,
                        icp_qat_fw_init_cmd_id_t initCmdId,
                        Cpa32U aeIndex)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    CpaStatus waitCbStatus = CPA_STATUS_FAIL;

    /*
     * Create a sync cookie which will wait for
     * a wake up call from the response message before
     * continuing once LacSync_WaitForCallback is called.
     */
    status = LacSync_CreateSyncCookie(&pSyncCallbackData);
    if (CPA_STATUS_SUCCESS != status)
    {
        /* Failure allocating sync cookie */
        LAC_LOG_ERROR("Failed to create sync cookie for "
                      "init message\n");
        return status;
    }

    status = QatCtrl_BuildAndSendMsg(instanceHandle,
                                     qat_instance,
                                     initCmdId,
                                     aeIndex,
                                     QatCtrl_InitSyncCb,
                                     pSyncCallbackData);

    if (CPA_STATUS_SUCCESS == status)
    {
        waitCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                               LAC_INIT_MSG_CALLBACK_TIMEOUT,
                                               &status,
                                               NULL);
        if (CPA_STATUS_SUCCESS != waitCbStatus)
        {
            LAC_LOG_ERROR("Callback timed out");
            status = waitCbStatus;
        }
    }
    else
    {
        /* As the Request was not sent the Callback will never
         * be called, so need to indicate that we're finished
         * with cookie so it can be destroyed. */
        LacSync_SetSyncCookieComplete(pSyncCallbackData);
    }

    LacSync_DestroySyncCookie(&pSyncCallbackData);
    return status;
}

/*
 * Build and send the message to the Rings
 */
CpaStatus
QatCtrl_BuildAndSendMsg(CpaInstanceHandle instanceHandle,
                         sal_qat_service_t* qatInstanceHandle,
                         icp_qat_fw_init_cmd_id_t initCmdId,
                         Cpa32U aeIndex, QatCtrl_GenCbFunc pCb,
                         void *pCallbackTag)
{

    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_init_req_t initMsg;
    qatal_init_cb_info_t *pCbInfo=NULL;

    status = LAC_OS_CAMALLOC(&pCbInfo,
                             sizeof(qatal_init_cb_info_t),
                             LAC_64BYTE_ALIGNMENT,
                             qatInstanceHandle->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Error in allocation pCbInfo callback structure\n");
        return CPA_STATUS_RESOURCE;
    }
    pCbInfo->instance = (LAC_ARCH_UINT)qatInstanceHandle;
    pCbInfo->pCb = (LAC_ARCH_UINT)pCb;
    pCbInfo->pCallbackTag = (LAC_ARCH_UINT)pCallbackTag;

    /* Build the message associated with this command Id. */
    status = QatCtrl_InitMsgBuild(instanceHandle,
                                &initMsg,initCmdId,
                                aeIndex,
                                qatInstanceHandle,
                                pCbInfo);

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Send message to the Acceleration Engines */
        status = icp_adf_transPutMsg(
                         qatInstanceHandle->trans_handle_qat_admin_tx,
                         (Cpa32U *)&initMsg,
                         QAT_MSG_SZ_WORDS);
        /*
         * Note we cannot get retries here or fails so
         * message guaranteed to be sent.
         */
    }
    else
    {
        LAC_LOG_ERROR("Failed to send QAT init message");
        LAC_OS_CAFREE(pCbInfo);
        status = CPA_STATUS_FAIL;
    }
    return status;
}

/*
 *******************************************************************************
 * @ingroup qat_init
 *      QatCtrl_BuildShramMask
 *
 * @description
 *       Build the SHRAM mask for each Acceleration Engine.
 *       Assuming that there is 64KBytes of shared ram per QAT.
 *       Note: Only 3 services are currently supported.
 *
 * @param[out] ShramMask   BitMask of how the SHRAM will be divided
 * @param[in]  activeAes   Number of active AEs per QAT instance
 * @param[in]  aeIndex     AE Index we are initialising on this QAT
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully.
 *         CPA_STATUS_FAIL            Error occured.
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/
CpaStatus QatCtrl_BuildShramMask(Cpa64U *ShramMask,
                                 Cpa32U activeAes, Cpa32U aeIndex)
{
    *ShramMask = 0;
    /*
     * If there is only one AE all of the
     * shram is used by this AE.
     */
    if(NUM_AES_ONE == activeAes)
    {
        *ShramMask = ~(*ShramMask);
        return CPA_STATUS_SUCCESS;
    }
    /*
     * If there is two AEs each gets half of the shram.
     */
    if(NUM_AES_TWO == activeAes)
    {
        if(NUM_AES_ONE == aeIndex)
        {
            *ShramMask = ((~(*ShramMask)) & 0xFFFFFFFF);
        }
        else
        {
            *ShramMask = ((~(*ShramMask)) & 0xFFFFFFFF00000000ULL);
        }
        return CPA_STATUS_SUCCESS;
    }
    /*
     * If there is three the first two are crypto and they get
     * 23 pages each and the third compression gets 18.
     */
    if(NUM_AES_THREE == activeAes)
    {
        if(NUM_AES_ONE - 1 == aeIndex)
        {
            *ShramMask = ((~(*ShramMask)) & 0x7FFFFF);
        }
        else if(NUM_AES_TWO - 1 == aeIndex)
        {
            *ShramMask = ((~(*ShramMask)) & 0x3FFFFF800000ULL);
        }
        else
        {
            *ShramMask = ((~(*ShramMask)) & 0xFFFFC00000000000ULL);
        }
        return CPA_STATUS_SUCCESS;
    }

    /*
     * Currently four services are not supported
     */
    LAC_LOG_ERROR("Only three services are supported in current version\n");
    return CPA_STATUS_FAIL;
}

/*
 * Build the SliceMask for the SET_AE_INFO msg
 */
CpaStatus QatCtrl_BuildSliceMask(Cpa16U *SliceMask,
                                 Cpa32U serviceMask,
                                 Cpa32U shramInitFlag)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa16U cmnReqFlags =0;
    Cpa32U shram=0;

    if(shramInitFlag)
    {
        shram = QAT_COMN_SHRAM_INIT_REQUIRED;
    }
    else
    {
        shram = QAT_COMN_SHRAM_INIT_NOT_REQUIRED;
    }
    /*
     * For up to three services on 1 AE there are
     * seven possible combinations. They are covered
     * by the following if statements.
     */
    if(SAL_SERVICE_TYPE_CRYPTO_A & serviceMask)
    {
        cmnReqFlags |= CY0_ONLY(shram);
    }
    if(SAL_SERVICE_TYPE_CRYPTO_B & serviceMask)
    {
        cmnReqFlags |= CY1_ONLY(shram);
    }
    if(SAL_SERVICE_TYPE_COMPRESSION & serviceMask)
    {
        cmnReqFlags |= DC_ONLY(shram);
    }
    *SliceMask = cmnReqFlags;
    return status;
}

/*
 * Build the Init Msg. The message built is different depending
 * on whether it is a SET_AE_INFO, SET_RING_INFO, TRNG_ENABLE
 * or TRNG_DISABLE msg.
 */
CpaStatus QatCtrl_InitMsgBuild(CpaInstanceHandle instanceHandle,
                             icp_qat_fw_init_req_t *pInitReq,
                             icp_qat_fw_init_cmd_id_t initCmdId,
                             Cpa32U aeIndex,
                             sal_qat_service_t* qatInstance,
                             qatal_init_cb_info_t *pCbInfo)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U initShramFlag = 0;
    icp_qat_fw_init_set_ae_info_hdr_t *pAEReqHdr = NULL;
    icp_qat_fw_init_set_ae_info_t *pAEReqInfo = NULL;
    icp_qat_fw_init_set_ring_info_hdr_t *pRingReqHdr = NULL;
    icp_qat_fw_init_set_ring_info_t *pRingReqInfo = NULL;
    icp_qat_fw_init_trng_hdr_t *pTrngCmdHdr = NULL;
    icp_qat_fw_comn_flags cmnReqFlags = 0;
    icp_accel_dev_t* device = (icp_accel_dev_t*) instanceHandle;
    Cpa16U table_sz = 0;
    Cpa32U serviceMask = 0;
    Cpa32U ringClusterId = 0;
    Cpa32U qatTargetId = 0;

    LAC_CHECK_NULL_PARAM(pInitReq);

    /* serviceMask array has num_aes_per_qat entries, using aeIndex to
       access the correct entry */
    serviceMask = qatInstance->serviceMask[
                               aeIndex % qatInstance->num_aes_per_qat];

    /* Clear the message contents */
    osalMemSet(pInitReq, 0, sizeof(icp_qat_fw_init_req_t));

    /* Populate the Common Request Message Definition. */

    /* LW0 */
    SalQatMsg_ReqTypePopulate(&(pInitReq->comn_hdr.arch_if),
                              ICP_ARCH_IF_REQ_QAT_FW_INIT,
                              qatInstance->ringNumRx);

    /* LW 1 to 4 are not used and are not populated */


    /* Common middle part of the request (LW 6 to 11)*/
    SalQatMsg_CmnMsgMidPopulate(&(pInitReq->comn_mid),
                                  pCbInfo,0,0);

    if (initCmdId == ICP_QAT_FW_INIT_CMD_SET_RING_INFO)
    {
        pRingReqHdr = &(pInitReq->u.set_ring_info);
        pRingReqInfo = &(pInitReq->u1.set_ring_info);

        /* set the Init command ID */
        pRingReqHdr->init_cmd_id = initCmdId;
        /* set the AE target ID */
        pRingReqHdr->init_trgt_id = qatInstance->aeTargetIds[aeIndex];

        /* If device is started no need to rebuild the ring table
         * this is only update for a userspace or guest OS ring */
        if (!icp_adf_isDevStarted(device))
        {
            QatCtrl_buildKernelRingTable(device, qatInstance,
                             (aeIndex%qatInstance->num_aes_per_qat));
        }

        /* set the ring table size in bytes */
        table_sz = sizeof(icp_qat_fw_init_ring_table_t);
        pRingReqHdr->init_ring_tbl_sz = table_sz;
        /*
         * set the ring table address
         * The table that is sent down will depend on the
         * service assigned to that acceleration engine.
         */
        pRingReqInfo->init_ring_table_ptr = qatInstance->RingTableDmaAddr +
                            ((aeIndex % qatInstance->num_aes_per_qat) *
                                  sizeof(icp_qat_fw_init_ring_table_t));
    }
    else if (initCmdId == ICP_QAT_FW_INIT_CMD_SET_AE_INFO)
    {
        /* Initialize Shared RAM once per QAT */
        if ((aeIndex%(qatInstance->num_aes_per_qat)) == 0)
        {
            initShramFlag = 1;
        }

        /* Get the qatTargetId and ringClusterId */
        if( aeIndex >= qatInstance->num_aes_per_qat)
        {
           /* We are initialising the second QAT */
           qatTargetId = 1;
           ringClusterId = 1;
        }
        else
        {
           ringClusterId = 0;
           if(qatInstance->qatAeMask[0])
           {
                qatTargetId = 0;
           }
           else
           {
                qatTargetId = 1;
           }
        }

        pAEReqHdr = &(pInitReq->u.set_ae_info);
        pAEReqInfo = &(pInitReq->u1.set_ae_info);

        /* set the Init command ID */
        pAEReqHdr->init_cmd_id = initCmdId;
        /* set the AE target ID */
        pAEReqHdr->init_trgt_id = qatInstance->aeTargetIds[aeIndex];
        /* set the ring cluster ID */
        pAEReqHdr->init_ring_cluster_id = ringClusterId;
        /* set the QAT ID */
        pAEReqHdr->init_qat_id = qatTargetId;
        /* Configure the Slice Mask. */
        QatCtrl_BuildSliceMask(&(pAEReqHdr->init_slice_mask),
                               serviceMask,
                               initShramFlag);

        /* Configure the Shram Mask. */
        status = QatCtrl_BuildShramMask(&(pAEReqInfo->init_shram_mask),
                               qatInstance->active_aes_per_qat,
                               aeIndex%qatInstance->num_aes_per_qat);
        if(CPA_STATUS_SUCCESS != status)
        {
            return status;
        }

    }
    else if (initCmdId == ICP_QAT_FW_INIT_CMD_TRNG_ENABLE)
    {
        /* Set Common Request Flag */
        ICP_QAT_FW_COMN_RND_SLICE_SET(cmnReqFlags,QAT_COMN_RND_SLICE_REQUIRED);
        pInitReq->comn_hdr.comn_req_flags = cmnReqFlags;

        pTrngCmdHdr = &(pInitReq->u.init_trng);

        /* Set the Init Command ID */
        pTrngCmdHdr->init_cmd_id = initCmdId;
        /* set the AE target ID */
        pTrngCmdHdr->init_trgt_id = qatInstance->aeTargetIds[aeIndex];
    }
    else if (initCmdId == ICP_QAT_FW_INIT_CMD_TRNG_DISABLE)
    {
        /* Set Common Request Flag */
        ICP_QAT_FW_COMN_RND_SLICE_SET(cmnReqFlags,
                                      QAT_COMN_RND_SLICE_REQUIRED);
        pInitReq->comn_hdr.comn_req_flags = cmnReqFlags;

        pTrngCmdHdr = &(pInitReq->u.init_trng);

        /* set the Init command ID */
        pTrngCmdHdr->init_cmd_id = initCmdId;
        /* set the AE target ID */
        pTrngCmdHdr->init_trgt_id = qatInstance->aeTargetIds[aeIndex];
    }
    else
    {
        /* Unsupported message */
        LAC_LOG_ERROR("Invalid QAT init message ID\n");
        status = CPA_STATUS_INVALID_PARAM;
    }
    return status;
}

