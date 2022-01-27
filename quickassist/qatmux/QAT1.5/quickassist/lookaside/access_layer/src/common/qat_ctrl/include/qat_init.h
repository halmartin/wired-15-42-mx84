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
 * @file qat_init.h
 *
 * @ingroup qat_ctrl
 *
 * @description
 *      This file includes function prototypes and structures that are
 *      used to build the init msg.
 *
 *****************************************************************************/

#ifndef QAT_INIT_H
#define QAT_INIT_H

/*
 * Number of shared RAM pages to be distributed
 * between the AEs
 */
#define ICP_QAT_FW_SHRAM_NUM_PAGES  64
/* #defines for SHRAM masking. */
#define SHRAM_MASK_PER_QTR          0xFFFF
/*
 * Slice Mask Macros for configuring the AE. Altogether for up to three
 * services possible on one AE there are seven possible combinations.
 * All seven combinations can be deduced by suitable combinations of
 * the three below.
 */

#define CY0_ONLY(shram)                          \
        ICP_QAT_FW_COMN_FLAGS_BUILD(             \
        ICP_QAT_FW_COMN_ORD_FLAG_STRICT,         \
        QAT_COMN_PTR_TYPE_FLAT,                  \
        ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,       \
        shram,                                   \
        QAT_COMN_REGEX_SLICE_NOT_REQUIRED,       \
        QAT_COMN_XLAT_SLICE_NOT_REQUIRED,        \
        QAT_COMN_CPR_SLICE_NOT_REQUIRED,         \
        QAT_COMN_BULK_SLICE_NOT_REQUIRED,        \
        QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,     \
        QAT_COMN_RND_SLICE_REQUIRED,             \
        QAT_COMN_PKE1_SLICE_NOT_REQUIRED,        \
        QAT_COMN_PKE0_SLICE_REQUIRED,            \
        QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,       \
        QAT_COMN_AUTH0_SLICE_REQUIRED,           \
        QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,     \
        QAT_COMN_CIPHER0_SLICE_REQUIRED)

#define CY1_ONLY(shram)                          \
        ICP_QAT_FW_COMN_FLAGS_BUILD(             \
        ICP_QAT_FW_COMN_ORD_FLAG_STRICT,         \
        QAT_COMN_PTR_TYPE_FLAT,                  \
        ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,       \
        shram,                                   \
        QAT_COMN_REGEX_SLICE_NOT_REQUIRED,       \
        QAT_COMN_XLAT_SLICE_NOT_REQUIRED,        \
        QAT_COMN_CPR_SLICE_NOT_REQUIRED,         \
        QAT_COMN_BULK_SLICE_NOT_REQUIRED,        \
        QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,     \
        QAT_COMN_RND_SLICE_NOT_REQUIRED,         \
        QAT_COMN_PKE1_SLICE_REQUIRED,            \
        QAT_COMN_PKE0_SLICE_NOT_REQUIRED,        \
        QAT_COMN_AUTH1_SLICE_REQUIRED,           \
        QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,       \
        QAT_COMN_CIPHER1_SLICE_REQUIRED,         \
        QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED)



#define DC_ONLY(shram)                           \
        ICP_QAT_FW_COMN_FLAGS_BUILD(             \
        ICP_QAT_FW_COMN_ORD_FLAG_STRICT,         \
        QAT_COMN_PTR_TYPE_FLAT,                  \
        ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,       \
        shram,                                   \
        QAT_COMN_REGEX_SLICE_NOT_REQUIRED,       \
        QAT_COMN_XLAT_SLICE_REQUIRED,            \
        QAT_COMN_CPR_SLICE_REQUIRED,             \
        QAT_COMN_BULK_SLICE_NOT_REQUIRED,        \
        QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,     \
        QAT_COMN_RND_SLICE_NOT_REQUIRED,         \
        QAT_COMN_PKE1_SLICE_NOT_REQUIRED,        \
        QAT_COMN_PKE0_SLICE_NOT_REQUIRED,        \
        QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,       \
        QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,       \
        QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,     \
        QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED)

/**
 *****************************************************************************
 * @ingroup qat_ctrl
 *
 * @description
 *      callback data for INIT messages
 *
 *****************************************************************************/
typedef struct qatal_init_cb_info_s
{
    LAC_ARCH_UINT instance;                 /*address of qatInstance handle */
    LAC_ARCH_UINT pCb;
    LAC_ARCH_UINT pCallbackTag;
} qatal_init_cb_info_t;

/**
 *******************************************************************************
 * @ingroup qat_init
 *      QatCtrl_InitMsgBuild.
 *
 * @description
 *      Build the init message.
 *
 * @param instanceHandle  IN    device structure ptr
 * @param pInitReq        IN    Init request message
 * @param initCmdId       IN    Init message command Id
 * @param aeIndex         IN    Ae index (in range 0-MAX_AES)
 * @param qatInstance     IN    QAT instance struct ptr
 * @param pCbInfo         IN    Callback info struct ptr
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully.
 * @retval CPA_STATUS_FAIL            Function failed.
 * @retval CPA_STATUS_INVALID_PARAM   Function received invalid param.
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/
CpaStatus
QatCtrl_InitMsgBuild(CpaInstanceHandle instanceHandle,
                   icp_qat_fw_init_req_t *pInitReq,
                   icp_qat_fw_init_cmd_id_t initCmdId,
                   Cpa32U aeIndex, sal_qat_service_t* qatInstance,
                   qatal_init_cb_info_t *pCbInfo);

/**
 *******************************************************************************
 * @ingroup qat_init
 *      QatCtrl_BuildAndSendMsg.
 *
 * @description
 *      Call the function to build the message and then send the message to
 *      the request ring.
 *
 * @param instanceHandle  IN    device structure ptr
 * @param qatInstance     IN    QAT instance struct ptr
 * @param initCmdId       IN    Init message command Id
 * @param aeIndex         IN    Ae Index (in range 0-MAX_AES)
 * @param pCb             IN    Callback function ptr
 * @param pCbInfo         IN    Callback data ptr
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully.
 * @retval CPA_STATUS_FAIL            Function failed.
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/

CpaStatus
QatCtrl_BuildAndSendMsg(CpaInstanceHandle instanceHandle,
                         sal_qat_service_t* qatInstanceHandle,
                         icp_qat_fw_init_cmd_id_t initCmdId,
                         Cpa32U aeIndex,
                         QatCtrl_GenCbFunc pCb,
                         void *pCallbackTag);

/**
 *******************************************************************************
 * @ingroup qat_init
 *      QatCtrl_InitMsgSendSync
 *
 * @description
 *     Send the Init message making use of the lac sync framework.
 *
 * @param instanceHandle  IN    device structure ptr
 * @param qatInstance     IN    QAT instance struct ptr
 * @param initCmdId       IN    Init message command Id
 * @param aeIndex         IN    Ae Index
 *
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully.
 * @retval CPA_STATUS_FAIL            Function failed.
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/

CpaStatus
QatCtrl_InitMsgSendSync(CpaInstanceHandle instanceHandle,
                        sal_qat_service_t* qat_instance,
                        icp_qat_fw_init_cmd_id_t initCmdId,
                        Cpa32U aeIndex );

/**
 *******************************************************************************
 * @ingroup qat_init
 *      QatCtrl_SendInitMsg.
 *
 * @description
 *     Send the Init Msg to the request ring
 *
 * @param instanceHandle  IN    device structure ptr
 * @param qatInstance     IN    QAT instance struct ptr
 * @param initCmdId       IN    Init message command Id
 *
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully.
 * @retval CPA_STATUS_FAIL            Function failed.
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/
CpaStatus
QatCtrl_SendInitMsg(CpaInstanceHandle instanceHandle,
                     sal_qat_service_t* qat_instance,
                     icp_qat_fw_init_cmd_id_t initCmdId);

#endif/*QAT_INIT_H*/
