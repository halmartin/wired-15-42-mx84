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
 * @file qat_ctrl.h
 *
 * @ingroup qat_ctrl
 *
 * @description
 *      This file includes function prototypes that receive the callbacks as
 *      well as the functions used to get the stats on messages sent and
 *      received.
 *
 *****************************************************************************/


#ifndef QAT_CTRL_H
#define QAT_CTRL_H

/* The message size in words passed to ADF. */
#define QAT_MSG_SZ_WORDS (ICP_QAT_FW_RESP_DEFAULT_SZ >> 2)
/* AE Mask used to find the AE ID. */
#define AE_MASK 0x1
/* Number of AEs per SKU */
#define NUM_AES_ONE   1
#define NUM_AES_TWO   2
#define NUM_AES_THREE 3
#define NUM_AES_FOUR  4

/* Generic callback function for init & admin. */
typedef void (*QatCtrl_GenCbFunc)(void *pCallbackTag, CpaStatus status);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *
 * @description
 *      Register a callback to handle message responses
 *      for a given request type
 *
 * @param qatInstanceHandle    IN    The QAT instance handle
 *
 * @retval                     None
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/
void
QatCtrl_ResponseCbSet(sal_qat_service_t* qatInstanceHandle);


/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      QatCtrl_ResponseMsgHandler
 *
 * @description
 *      Response message handler function. This function in turn calls another
 *      function based on the message response type.
 *
 * @param trans_handle    IN    transport handle
 *
 * @param pNewMsg         IN    Response Message
 *
 *
 * @pre @ref QatCtrl_Init() must have been called already
 *
 *****************************************************************************/
void
QatCtrl_ResponseMsgHandler(icp_comms_trans_handle trans_handle,
                            void *pNewMsg);
#endif /*QAT_CTRL_H*/
