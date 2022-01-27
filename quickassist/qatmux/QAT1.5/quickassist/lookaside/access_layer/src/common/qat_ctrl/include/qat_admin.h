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
 * @file qat_admin.h
 *
 * @ingroup qat_ctrl
 *
 * @description
 *      This file includes function prototypes and structures that are
 *      used to build the admin msg.
 *
 *****************************************************************************/

#ifndef QAT_ADMIN_H
#define QAT_ADMIN_H

/**
 *****************************************************************************
 * @ingroup qat_ctrl
 *
 * @description
 *      qat stats structure filled out by the QatCtrl_FWCountGet() function
 *
 *
 *****************************************************************************/
typedef struct qat_admin_stats_s
{
   Cpa64U *numSent;
   /**< array to hold num requests sent to AEs on this QAT */
   /**< array should be allocated by client, size = maxAesPerQat */
   Cpa64U *numRec;
   /**< array to hold num requests received from AEs on this QAT */
   /**< array should be allocated by client, size = maxAesPerQat */

   /* Note other info can similarly be added if required */
}qat_admin_stats_t;

/**
 *****************************************************************************
 * @ingroup qat_ctrl
 *
 * @description
 *      callback data for ADMIN messages
 *
 *****************************************************************************/
typedef struct qatal_admin_cb_info_s
{
    sal_qat_service_t* instance;
    /**< QAT instance the request is sent from */
    Cpa32U aeIndex;
    /**< AE index the request is sent to */
    CpaBoolean cbTimeout;
    /**< flag indicates if callback timeout happens */
    qat_admin_stats_t *pStats;
    /**< qat stats structure that is updated in callback */
    LAC_ARCH_UINT pCb;
    LAC_ARCH_UINT pCallbackTag;
} qatal_admin_cb_info_t;

/**
 *****************************************************************************
 * @ingroup icp_QatCtrl
 *      Definition of callback function for response messages
 *
 * @description
 *      This data structure specifies the prototype for the callback function.
 *      The callback function is registered by the application using
 *      the QatCtrl_ResponseCbSet() call.
 *
 * @context
 *      The function will be invoked from an interrupt bottom-half context
 *
 * @param pRespMsg      IN     A pointer to the 64-byte response message
 *
 * @param qatReqType    IN     QAT service type of the response message
 *
 * @retval
 *      None
 *
 * @see
 *      QatCtrl_ResponseCbSet(), QatCtrl_MsgSend()
 *
 *****************************************************************************/
typedef void (*qat_comms_cb_func_t)(void *pRespMsg,
                                        icp_arch_if_request_t qatReqType);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Get number of messages processes by the AEs
 *
 * @description
 *      This function calls the SendAdmin Msg function which
 *      sends debug messages down to the AEs and returns the
 *      number of messages processed by filling out the
 *      qat_admin_stats_t structure.
 *
 * @threadSafe
 *      Yes
 *
 * @param instanceHandle       IN  device handle
 * @param qatInstanceHandle    IN  qat handle
 * @param pQatStats            OUT Structure to be filled out
 *
 * @retval CPA_STATUS_SUCCESS  Successfully received stats
 * @retval CPA_STATUS_RESOURCE Resource error
 * @retval CPA_STATUS_RETRY    Retry
 * @retval CPA_STATUS_FAIL     General Fail
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_FWCountGet(CpaInstanceHandle instanceHandle,
                   void* qatInstanceHandle,
                   qat_admin_stats_t *pQatStats);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Send Admin msgs to AEs
 *
 * @description
 *      Send the Admin msgs to all the AEs
 *
 * @param instanceHandle       IN  service handle
 * @param qatInstanceHandle    IN  qat handle
 * @param adminCmdId           IN  The Admin command
 * @param pQatStats            IN  stats structure to be filled out in cb
 *
 * @retval CPA_STATUS_SUCCESS  Successfully sent message
 * @retval CPA_STATUS_RESOURCE Resource error
 * @retval CPA_STATUS_RETRY    Retry
 * @retval CPA_STATUS_FAIL     General Fail
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_SendAdminMsg(CpaInstanceHandle instanceHandle,
                     sal_qat_service_t* qat_instance,
                     icp_qat_fw_admin_cmd_id_t adminCmdId,
                     qat_admin_stats_t* pQatStats);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Synchronise the message sending
 *
 * @description
 *      Synchronise the message sending - setup callback
 *
 * @param instanceHandle       IN  service handle
 * @param qatInstanceHandle    IN  qat handle
 * @param adminCmdId           IN  The Admin command
 * @param aeIndex              IN  Ae index (range 0-MAX_AES)
 * @param pQatStats            IN  stats structure to be filled out in cb
 *
 * @retval CPA_STATUS_SUCCESS  Successfully sent message
 * @retval CPA_STATUS_RESOURCE Resource error
 * @retval CPA_STATUS_RETRY    Retry
 * @retval CPA_STATUS_FAIL     General Fail
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_AdminMsgSendSync(CpaInstanceHandle instanceHandle,
                         sal_qat_service_t* qat_instance,
                         icp_qat_fw_admin_cmd_id_t adminCmdId,
                         Cpa32U aeIndex,
                         qat_admin_stats_t* pQatStats);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Build and Send the Admin msg
 *
 * @description
 *      Prepare to build and then put the message onto the rings
 *
 * @param instanceHandle       IN  service handle
 * @param qatInstanceHandle    IN  qat handle
 * @param adminCmdId           IN  The Admin command
 * @param aeIndex              IN  Ae index
 * @param pQatStats            IN  stats structure to be filled out in cb
 * @param pCb                  IN  Callback function to call
 * @param pCallbackTag         IN  Callback data to use
 *
 * @retval CPA_STATUS_SUCCESS  Successfully built and sent message
 * @retval CPA_STATUS_RESOURCE Resource error
 * @retval CPA_STATUS_RETRY    Retry
 * @retval CPA_STATUS_FAIL     General Fail
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_BuildAndSendAdminMsg(CpaInstanceHandle instanceHandle,
                             sal_qat_service_t* qatInstanceHandle,
                             icp_qat_fw_admin_cmd_id_t adminCmdId,
                             Cpa32U aeIndex,
                             qatal_admin_cb_info_t* pCbInfo);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Build the Admin msg
 *
 * @description
 *      Build the admin message
 *
 * @param instanceHandle       IN  service handle
 * @param pAdminReq            IN  Admin request structure
 * @param adminCmdId           IN  The Admin command
 * @param aeIndex              IN  Ae index
 * @param qatInstanceHandle    IN  qat handle
 * @param pCbInfo              IN  Callback response structure
 *
 * @retval CPA_STATUS_SUCCESS  Successfully built message
 * @retval CPA_STATUS_FAIL     General Fail
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_AdminMsgBuild(CpaInstanceHandle instanceHandle,
                      icp_qat_fw_admin_req_t *pAdminReq,
                      icp_qat_fw_admin_cmd_id_t adminCmdId,
                      Cpa32U aeIndex,
                      sal_qat_service_t* qatInstance,
                      qatal_admin_cb_info_t *pCbInfo);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *
 * @description
 *      Admin response handler function. The message is read the callback
 *      function called.
 *
 * @param pRespMsg    IN  The response message
 * @param msgType     IN  The message type

 *
 *****************************************************************************/
void QatCtrl_AdminRespHandler(void *pRespMsg,
                            icp_arch_if_request_t msgType);

#endif
