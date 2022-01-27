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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file  qat_initadmin_dh895x.c
 *
 * @ingroup qat_ctrl
 *
 * @description
 *        Init/Admin messages construction, sending and response handling
 *        for dh895x.
 *
 *****************************************************************************/

#include "cpa.h"
/* Log and Osal includes. */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "Osal.h"

/* ADF includes */
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_init_admin.h"
#include "icp_qat_fw.h"

/* SAL includes */
#include "lac_sym.h"
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_string_parse.h"

#include "lac_sal_types_qat_ctrl.h"
#include "qat_admin_common.h"
#include "qat_initadmin_dh895x.h"
#ifndef ICP_DC_ONLY
#include "lac_sym_qat_constants_table.h"
#endif

CpaStatus
QatCtrl_SendSyncMsg(icp_comms_trans_handle trans_handle,
            icp_qat_fw_init_admin_req_t * p_req_msg,
            icp_qat_fw_init_admin_resp_t * p_resp_msg)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    status = icp_adf_transPutMsgSync(trans_handle, (Cpa32U *)p_req_msg,
            (Cpa32U *)p_resp_msg, QAT_DH895X_ADMIN_MSG_SZ);
    if (status != CPA_STATUS_SUCCESS)
    {
        if (status == CPA_STATUS_FAIL)
        {
            LAC_LOG_ERROR2("AE failed to respond to admin cmd %d, status:%d",
                                        p_req_msg->init_admin_cmd_id, status);
        }
    }
    else if (p_resp_msg->init_resp_hdr.status
                                != ICP_QAT_FW_INIT_RESP_STATUS_SUCCESS)
    {
        LAC_LOG_ERROR2("AE response received to admin cmd %d, Fail status: %d",
                p_req_msg->init_admin_cmd_id, p_resp_msg->init_resp_hdr.status);
        status = CPA_STATUS_FAIL;
    }

    return status;
}


CpaStatus QatCtrl_BuildMessage(Cpa8U commandId,
                               icp_qat_fw_init_admin_req_t *pReqMsg)
{
#ifndef ICP_DC_ONLY
    Cpa16U sizeInBytes = 0;
    Cpa64U pTable = 0;
#endif
    CpaStatus status = CPA_STATUS_SUCCESS;

    osalMemSet(pReqMsg, 0, sizeof(*pReqMsg));
    pReqMsg->init_admin_cmd_id = commandId;
    /* Put some arbitrary data in the field which FW loops back
     * to facilitate testing */
    pReqMsg->opaque_data = OPAQUE_DATA;

    switch (commandId)
    {
    case ICP_QAT_FW_INIT_ME:
#ifndef ICP_DC_ONLY
    case ICP_QAT_FW_TRNG_ENABLE:
    case ICP_QAT_FW_TRNG_DISABLE:
#endif
    case ICP_QAT_FW_STATUS_GET:
    case ICP_QAT_FW_COUNTERS_GET:
    case ICP_QAT_FW_LOOPBACK:
    case ICP_QAT_FW_HEARTBEAT_SYNC:
    case ICP_QAT_FW_HEARTBEAT_GET:
        break;
#ifndef ICP_DC_ONLY
    case ICP_QAT_FW_CONSTANTS_CFG:
        LacSymQat_ConstantsGetTableInfo(&pTable, &sizeInBytes);
        if (0 != pTable)
        {
            pReqMsg->init_cfg_sz = sizeInBytes;
            /* write the physical address */
            pReqMsg->init_cfg_ptr = pTable;
        }
        else
        {
            LAC_LOG_ERROR("SymConstantsTable not available,"
                    " Send loopback msg to FW instead");
            pReqMsg->init_admin_cmd_id = ICP_QAT_FW_LOOPBACK ;
        }
        break;
#endif
    default:
        LAC_LOG_ERROR1("Unrecognized init/admin msg type %d.", commandId);
        status = CPA_STATUS_FAIL;
        break;
    }
    return status;
}

CpaStatus QatCtrl_SendAdminCmd_dh895x(sal_qat_service_t *qatInstance, Cpa8U commandId)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_init_admin_req_t req_msg;
    icp_qat_fw_init_admin_resp_t resp_msg;
    icp_comms_trans_handle trans_handle;
    Cpa16U i=0;

    LAC_CHECK_NULL_PARAM(qatInstance);

    status = QatCtrl_BuildMessage(commandId, &req_msg);

    for(i = 0; i < qatInstance->num_aes; i++)
    {
        /* Retrieve the transport handle for the targeted AE */
        trans_handle = qatInstance->trans_handle_qat_admin_tx[i];

        LAC_LOG_DEBUG2("%d of %d num_aes",i,qatInstance->num_aes);

        LAC_CHECK_NULL_PARAM(trans_handle);

        /* zero the response buffer */
        memset(&resp_msg,0,sizeof(icp_qat_fw_init_admin_resp_t));

        /* Pass the message to the transport */
        status |= QatCtrl_SendSyncMsg(trans_handle, &req_msg, &resp_msg);
    }

    return status;
}


CpaStatus QatCtrl_FWCountGet_dh895x(sal_qat_service_t* qatInstance,
                                    qat_admin_stats_t *pQatStats)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_init_admin_req_t req_msg;
    icp_qat_fw_init_admin_resp_t resp_msg;
    icp_comms_trans_handle trans_handle;
    Cpa16U i;

    LAC_CHECK_NULL_PARAM(qatInstance);
    LAC_CHECK_NULL_PARAM(pQatStats);

    status = QatCtrl_BuildMessage(ICP_QAT_FW_COUNTERS_GET, &req_msg);

    for(i = 0; i < qatInstance->num_aes; i++)
    {
        /* Retrieve the transport handle for the targeted ME */
        trans_handle = qatInstance->trans_handle_qat_admin_tx[i];

        LAC_CHECK_NULL_PARAM(trans_handle);

        /* zero the response buffer */
        memset(&resp_msg,0,sizeof(icp_qat_fw_init_admin_resp_t));

        /* Pass the message to the transport */
        status = QatCtrl_SendSyncMsg(trans_handle, &req_msg, &resp_msg);
        if (status == CPA_STATUS_SUCCESS)
        {
            *(pQatStats->numRec+i) = resp_msg.init_resp_pars.u.s2.req_rec_count;
            *(pQatStats->numSent+i) = resp_msg.init_resp_pars.u.s2.resp_sent_count;
        }
    }
    return status;
}

CpaStatus QatCtrl_SendHeartbeat_dh895x(sal_qat_service_t *qatInstance,
                                       Cpa32U delay,
                                       CpaStatus *heartbeatStatus)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_init_admin_req_t req_msg_type1;
    icp_qat_fw_init_admin_req_t req_msg_type2;
    icp_qat_fw_init_admin_resp_t resp_msg;
    icp_comms_trans_handle trans_handle;
    Cpa16U i;
    Cpa8U heartbeatFWStatus = 0;

    LAC_CHECK_NULL_PARAM(qatInstance);

    *heartbeatStatus = CPA_STATUS_SUCCESS;

    status = QatCtrl_BuildMessage(ICP_QAT_FW_HEARTBEAT_SYNC, &req_msg_type1);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Build Heartbeat message SYNC failed.");
        return status;
    }

    status = QatCtrl_BuildMessage(ICP_QAT_FW_HEARTBEAT_GET, &req_msg_type2);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Build Heartbeat message GET failed.");
        return status;
    }

    /*
     * Acquire Heartbeat Mutex
     */
    if (osalMutexLock(&qatInstance->heartbeatLock,
                     QAT_DH895X_HEARTBEAT_MUTEX_TIMEOUT))
    {
        LAC_LOG_ERROR("Acquire Heartbeat Mutex failed.");
        return CPA_STATUS_FAIL;
    }

    if (CPA_FALSE == SalCtrl_getHeartbeatStatus(qatInstance))
    {
        /* Do not send any heartbeat and unlock the mutex */
        goto end_heartbeat;
    }

    for (i = 0; i < qatInstance->num_aes; i++)
    {
        /* Retrieve the transport handle for the targeted AE */
        trans_handle = qatInstance->trans_handle_qat_admin_tx[i];

        LAC_LOG_DEBUG2("%d of %d num_aes", i, qatInstance->num_aes);

        LAC_CHECK_NULL_PARAM(trans_handle);

        /* zero the response buffer */
        memset(&resp_msg, 0, sizeof(icp_qat_fw_init_admin_resp_t));

        /* Pass the message to the transport */
        status = QatCtrl_SendSyncMsg(trans_handle, &req_msg_type1, &resp_msg);
        if(CPA_STATUS_SUCCESS != status)
        {
            if (CPA_STATUS_FAIL == status)
            {
                LAC_LOG_ERROR("Send Heartbeat message SYNC failed.");
            }
            *heartbeatStatus = CPA_STATUS_FAIL;
            goto end_heartbeat;
        }
    }

    /* Wait (Heartbeat delay) */
    if (OSAL_SUCCESS != osalSleep(delay))
    {
        status = CPA_STATUS_RESOURCE;
        goto end_heartbeat;
    }

    for (i = 0; i < qatInstance->num_aes; i++)
    {
        /* Retrieve the transport handle for the targeted AE */
        trans_handle = qatInstance->trans_handle_qat_admin_tx[i];

        LAC_LOG_DEBUG2("%d of %d num_aes", i, qatInstance->num_aes);

        LAC_CHECK_NULL_PARAM(trans_handle);

        /* zero the response buffer */
        memset(&resp_msg, 0, sizeof(icp_qat_fw_init_admin_resp_t));

        /* Pass the message to the transport */
        status = QatCtrl_SendSyncMsg(trans_handle, &req_msg_type2, &resp_msg);
        if (CPA_STATUS_SUCCESS != status)
        {
            if (CPA_STATUS_FAIL == status)
            {
                LAC_LOG_ERROR("Send Heartbeat message GET failed.");
            }
            *heartbeatStatus = CPA_STATUS_FAIL;
            goto end_heartbeat;
        }

        heartbeatFWStatus = ICP_QAT_FW_COMN_HEARTBEAT_HDR_FLAG_GET(resp_msg.init_resp_hdr);
        if (ICP_QAT_FW_COMN_HEARTBEAT_BLOCKED == heartbeatFWStatus)
        {
            *heartbeatStatus = CPA_STATUS_FAIL;
            goto end_heartbeat;
        }
    }

end_heartbeat:
    /*
     * Release Heartbeat Mutex
     */
    osalMutexUnlock(&qatInstance->heartbeatLock);

    return status;
}
