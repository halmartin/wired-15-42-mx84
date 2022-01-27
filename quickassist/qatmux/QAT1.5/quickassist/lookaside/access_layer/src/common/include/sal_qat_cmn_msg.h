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
 * @file sal_qat_cmn_msg.h
 *
 * @defgroup SalQatCmnMessage
 *
 * @ingroup SalQatCmnMessage
 *
 * Interfaces for populating the common QAT structures for a lookaside
 * operation.
 *
 *****************************************************************************/


/*****************************************************************************/

#ifndef SAL_QAT_CMN_MSG_H
#define SAL_QAT_CMN_MSG_H

/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/
#include "cpa.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "icp_accel_devices.h"
#include "icp_qat_fw_la.h"
#include "icp_qat_hw.h"
#include "lac_common.h"

/* length of the 5 long words of the request that are stored in the session
 * This is rounded up to 32 in order to use the fast memcopy function */
#define SAL_SESSION_REQUEST_CACHE_SIZE_IN_BYTES (32)

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      content descriptor info structure
 *
 * @description
 *      This structure contains generic information on the content descriptor
 *
 *****************************************************************************/
typedef struct sal_qat_content_desc_info_s
{
    CpaPhysicalAddr dataPhys;
    /**< Physical address of Content Descriptor */
    void *pData;
    /**< Virtual Pointer to Content Descriptor */
    Cpa8U hdrSzQuadWords;
    /**< Content desc header size in quad words */
    Cpa8U hwBlkSzQuadWords;
    /**< Hardware Setup Block size in quad words */
} sal_qat_content_desc_info_t;

/**
 *******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Lookaside response handler function type
 *
 * @description
 *      This type definition specifies the function prototype for handling the
 *      response messages for a specific symmetric operation
 *
 * @param[in] lacCmdId      Look Aside Command ID
 *
 * @param[in] pOpaqueData   Pointer to Opaque Data
 *
 * @param[in] cmnRespFlags  Common Response flags
 *
 * @return void
 *
 *****************************************************************************/
typedef void (*sal_qat_resp_handler_func_t)(
    icp_qat_fw_la_cmd_id_t lacCmdId,
    void *pOpaqueData,
    icp_qat_fw_comn_flags cmnRespFlags);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate the common architecture fields of the common request header in
 *      a message
 *
 * @description
 *      This function populates the QAT common request message (i.e. LW0).
 *
 *      Note: Memory must be allocated for the message prior to calling this
 *      function. As the contents of the message is copied onto the ring it can
 *      be allocated as a variable on the stack.
 *
 *
 * @param[in] pMsg                Pointer to the message
 * @param[in] msgType             Message type identifier
 * @param[in] responseRingId      Response ring number
 *
 * @return none
 *
 *****************************************************************************/
void SalQatMsg_ReqTypePopulate(icp_arch_if_req_hdr_t *pMsg,
    icp_arch_if_request_t msgType, Cpa32U responseRingId);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate LW1-4 of the common request message
 *
 * @description
 *      This function populates LW 1 to 4 of the message
 *
 *      Note: Memory must be allocated for the message prior to calling this
 *      function. As the contents of the message is copied onto the ring it can
 *      be allocated as a variable on the stack.
 *
 *
 * @param[out] pComnReq             Pointer to a common request
 * @param[in] pContentDescInfo      Pointer to content descriptor info structure
 * @param[in] cmnReqFlags           Common request flags
 * @param[in] flowID                Session unique id for stateful FW processing
 *                                  for stateless set flowId=0
 *
 * @return none
 *
 *****************************************************************************/
void
SalQatMsg_CmnMsgHdrPopulate(icp_qat_fw_la_bulk_req_t * pComnReq,
                            const sal_qat_content_desc_info_t * pContentDescInfo,
                            icp_qat_fw_comn_flags cmnReqFlags,
                            Cpa32U flowID);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate the fields of the middle part of the message (LW 6-11)
 *
 * @description
 *      This function populates the QAT common request middle part of the
 *      message (LW 6 to 11)
 *
 *      Note: Memory must be allocated for the message prior to calling this
 *      function. As the contents of the message is copied onto the ring it can
 *      be allocated as a variable on the stack.
 *
 *
 * @param[out] pComnReqMid          Pointer to a common request middle part
 * @param[in] pCookie               Pointer to cookie
 * @param[in] srcBuffer             source buffer. Physical address of
 *                                  a buffer or a CpaPhysBufferList.
 * @param[in] dstBuffer             destination buffer. Physical address
 *                                  a buffer or a CpaPhysBufferList.
 *
 * @return none
 *
 *****************************************************************************/
void
SalQatMsg_CmnMsgMidPopulate(icp_qat_fw_comn_req_mid_t * pComnReqMid,
                            const void * pCookie,
                            Cpa64U srcBuffer,
                            Cpa64U dstBuffer);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate the request parameters of a lookaside request message
 *
 * @description
 *      This function populates the QAT request parameters for a lookaside
 *       message (i.e. LW12-13 and some of LW5).
 *
 * @param[out] pBulkRq              Pointer to request structure (cast
 *                                     as a bulk message)
 * @param[in] reqParamsPhyAddr      Physical address of request params structure
 * @param[in] reqParamBlkSizeBytes  Size of request params structure
 *
 * @return none
 *
 *****************************************************************************/
void SalQatMsg_ReqParamsPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    Cpa64U reqParamsPhyAddr, Cpa8U reqParamBlkSizeBytes);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate LW1-15 of a lookaside request message
 *
 * @description
 *      This function populates the QAT message (except LW0).
 *
 * @param[out] pBulkRq              Pointer to request structure (cast as a
 *                                   bulk message)
 * @param[in] pContentDescInfo      Pointer to content descriptor info structure
 * @param[in] pCookie               Pointer to cookie
 * @param[in] srcBuffer             source buffer handle
 * @param[in] dstBuffer             destination buffer handle
 * @param[in] serviceCmdId          Service specific command ID for message
 * @param[in] reqParamsPhyAddr      Physical address of request params structure
 * @param[in] reqParamBlkSizeBytes  Size of request params structure
 * @param[in] serviceCmdFlags       Service specific command flags
 * @param[in] cmdReqFlags           common request flags
 * @param[in] flowID                flowID
 * @param[in] next_addr             address of next request in chain
 *
 * @return none
 *
 *****************************************************************************/
void SalQatMsg_MsgParamsPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    const sal_qat_content_desc_info_t * pContentDescInfo, const void * pCookie,
    Cpa64U srcBuffer, Cpa64U dstBuffer, icp_qat_fw_la_cmd_id_t serviceCmdId,
    Cpa64U reqParamsPhyAddr, Cpa8U reqParamBlkSizeBytes, Cpa16U serviceCmdFlags,
    icp_qat_fw_comn_flags cmnReqFlags, Cpa32U flowID, Cpa64U next_addr);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate the service command request fields of a lookaside request
 *      message
 *
 * @description
 *      This function populates the service command fields of request message
 *      some of LW5.
 *
 * @param[out] pBulkRq              Pointer to request structure (cast
 *                                   as a bulk message)
 * @param[in] serviceCmdId          Service specific command ID for message
 * @param[in] serviceCmdFlags       Service specific command flags
 *
 * @return none
 *
 *****************************************************************************/
void SalQatMsg_ServiceCmdPopulate(icp_qat_fw_la_bulk_req_t * pBulkRq,
    icp_qat_fw_la_cmd_id_t serviceCmdId, Cpa16U serviceCmdFlags);

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *      Populate the common footer request fields of a lookaside request
 *      message (i.e. LW14-15)
 *
 * @description
 *      This function populates the footer QAT of a lookaside message.
 *
 * @param[out] pReq                 Pointer to request footer
 * @param[in] next_addr             Address of next request in a chain or
 *                                  NULL if last request in a chain. This will
 *                                  be written to footer. Note to write src_len
 *                                  or dest_len instead another function will
 *                                  need to be used.
 * @return none
 *
 *****************************************************************************/
void
SalQatMsg_CmnFooterPopulate(icp_qat_fw_comn_req_ftr_t *pReqFtr,
                            Cpa64U next_addr) ;

/**
 ******************************************************************************
 * @ingroup SalQatCmnMessage
 *     Given the address of the session this function returns a session
 *     dependent 32 bit flowId
 *
 * @description
 *      This function will set the flow ID for a session
 *
 * @param[in] pPhysAddress           Physical address of session memory
 *
 * @return Cpa32U                    32 bit flow ID
 *
 *****************************************************************************/
Cpa32U
SalQatMsg_SetFlowId(CpaPhysicalAddr* pPhysAddress);

#endif /* SAL_QAT_CMN_MSG_H */
