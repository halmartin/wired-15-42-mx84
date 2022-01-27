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
 * @file sal_qat_cmn_msg.c
 *
 * @ingroup SalQatCmnMessage
 *
 * @description
 *    Implementation for populating the common (across services) QAT structures.
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "lac_common.h"
#include "icp_accel_devices.h"
#include "Osal.h"
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#include "icp_adf_transport.h"

#include "icp_qat_hw.h"
#include "icp_qat_fw.h"
#include "icp_qat_fw_la.h"

#include "sal_qat_cmn_msg.h"

/*****************************************************************************/

/* FlowId Field overloaded. If MSB is zero indicates to fw that value
 * contained is a  FlowId value.
 */
#define ENSURE_MSB_ZERO_MASK 0x7FFFFFFF

void
SalQatMsg_MsgParamsPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    const sal_qat_content_desc_info_t * pContentDescInfo, const void * pCookie,
    Cpa64U srcBuffer, Cpa64U dstBuffer, icp_qat_fw_la_cmd_id_t serviceCmdId,
    Cpa64U reqParamsPhyAddr, Cpa8U reqParamBlkSizeBytes, Cpa16U serviceCmdFlags,
    icp_qat_fw_comn_flags cmnReqFlags, Cpa32U flowID, Cpa64U next_addr)
{
    LAC_ENSURE_NOT_NULL(pBulkRq);

    SalQatMsg_CmnMsgHdrPopulate(pBulkRq,
                                pContentDescInfo,
                                cmnReqFlags,
                                flowID);

    SalQatMsg_ServiceCmdPopulate(pBulkRq, serviceCmdId, serviceCmdFlags);

    SalQatMsg_CmnMsgMidPopulate(&(pBulkRq->comn_mid),
                                pCookie,
                                srcBuffer,
                                dstBuffer);

    SalQatMsg_ReqParamsPopulate(pBulkRq,reqParamsPhyAddr,reqParamBlkSizeBytes);

    SalQatMsg_CmnFooterPopulate(&(pBulkRq->comn_ftr), next_addr);
}

void
SalQatMsg_ServiceCmdPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    icp_qat_fw_la_cmd_id_t serviceCmdId, Cpa16U serviceCmdFlags)
{
    /* set the LA command ID */
    pBulkRq->comn_la_req.la_cmd_id = serviceCmdId;

    /* set the LA command flags */
    pBulkRq->comn_la_req.u.la_flags = serviceCmdFlags;
}

void
SalQatMsg_ReqParamsPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    Cpa64U reqParamsPhyAddr, Cpa8U reqParamBlkSizeBytes)
{
    pBulkRq->req_params_addr = reqParamsPhyAddr;

    /* set the request params block size in quad words */
    pBulkRq->comn_la_req.u1.req_params_blk_sz =
        (reqParamBlkSizeBytes / LAC_QUAD_WORD_IN_BYTES);
}



void
SalQatMsg_CmnMsgHdrPopulate(icp_qat_fw_la_bulk_req_t * pComnReq,
                  const sal_qat_content_desc_info_t * pContentDescInfo,
                  icp_qat_fw_comn_flags cmnReqFlags,
                  Cpa32U flowID)
{
    icp_qat_fw_comn_req_hdr_t *pComnReqHdr = NULL;

    LAC_ENSURE_NOT_NULL(pComnReq);

    pComnReqHdr = &(pComnReq->comn_hdr);

    LAC_ENSURE_NOT_NULL(pComnReqHdr);

    LAC_ENSURE_NOT_NULL(pContentDescInfo);

    pComnReqHdr->content_desc_hdr_sz = pContentDescInfo->hdrSzQuadWords;

    pComnReqHdr->content_desc_params_sz = pContentDescInfo->hwBlkSzQuadWords;

    /* write common request flags into the message */
    pComnReqHdr->comn_req_flags = cmnReqFlags;

    pComnReqHdr->content_desc_addr = pContentDescInfo->dataPhys;

    pComnReq->flow_id = flowID;
}

void
SalQatMsg_CmnMsgMidPopulate(icp_qat_fw_comn_req_mid_t * pComnReqMid,
                            const void * pCookie,
                            Cpa64U srcBuffer,
                            Cpa64U dstBuffer)
{
    LAC_ENSURE_NOT_NULL(pComnReqMid);
    LAC_MEM_SHARED_WRITE_FROM_PTR(pComnReqMid->opaque_data, pCookie);
    pComnReqMid->src_data_addr = srcBuffer;

    /* In place */
    if (0 == dstBuffer)
    {
        pComnReqMid->dest_data_addr = srcBuffer;
    }
    /* Out of place */
    else
    {
        pComnReqMid->dest_data_addr = dstBuffer;
    }

}

void
SalQatMsg_ReqTypePopulate(icp_arch_if_req_hdr_t *pMsg,
    icp_arch_if_request_t msgType, Cpa32U responseRingId)
{
    LAC_ENSURE_NOT_NULL(pMsg);
    LAC_OS_BZERO(pMsg, sizeof(icp_arch_if_req_hdr_t));

    /* Call Macro build Flags */
    pMsg->flags = ICP_ARCH_IF_FLAGS_BUILD(ICP_ARCH_IF_REQ_VALID_SET,
                                          ICP_ARCH_IF_ET_RING_RESP,
                                          ICP_ARCH_IF_S_RESP);

    /* Set request type and response Ring */
    pMsg->req_type = msgType;
    pMsg->resp_pipe_id = responseRingId;
}

void
SalQatMsg_CmnFooterPopulate(icp_qat_fw_comn_req_ftr_t *pReqFtr,
                            Cpa64U next_addr)
{
    LAC_ENSURE_NOT_NULL(pReqFtr);

    pReqFtr->next_request_addr = next_addr;
}

Cpa32U
SalQatMsg_SetFlowId(CpaPhysicalAddr* pPhysAddress)
{
    CpaPhysicalAddr physAddressSession = 0;
    physAddressSession = *pPhysAddress >> LAC_64BYTE_ALIGNMENT_SHIFT;
    return (Cpa32U)(physAddressSession & ENSURE_MSB_ZERO_MASK);
}
