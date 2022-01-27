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
 *****************************************************************************
 * @file qat_initadmin_dh895x.h
 *
 * @ingroup qat_ctrl
 *
 * @description
 *      This file includes function prototypes and structures that are
 *      used to build the init msg for the DH895x device.
 *
 *****************************************************************************/

#ifndef QAT_INIT_ADMIN_DH895X_H
#define QAT_INIT_ADMIN_DH895X_H

/* #define admin interface message size */
#define QAT_DH895X_ADMIN_MSG_SZ               32
#define QAT_DH895X_HEARTBEAT_MUTEX_TIMEOUT    500
#define OPAQUE_DATA                           0x1122334455667788ULL
/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Send an admin message
 *
 * @description
 *      This function sends an admin message down to the AES.
 *      The message is constructed internally based on the
 *      command id supplied and is then sent to each AE.
 *
 * @threadSafe
 *      Yes
 *
 * @param qatInstanceHandle    IN  qat handle
 * @param commandId            IN  type of message
 *
 * @retval CPA_STATUS_SUCCESS  Successfully sent message and got a response.
 * @retval CPA_STATUS_FAIL     Message was not sent or didn't get a response.
 *
 *
 *****************************************************************************/
CpaStatus QatCtrl_SendAdminCmd_dh895x(sal_qat_service_t *qatInstance,
                                      Cpa8U commandId);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Get number of messages processes by the AEs
 *
 * @description
 *      This function calls the Send Sync Msg function which
 *      sends debug messages down to the AEs and returns the
 *      number of messages processed by filling out the
 *      qat_admin_stats_t structure.
 *
 * @threadSafe
 *      Yes
 *
 * @param qatInstance          IN  qat handle
 * @param pQatStats            OUT Structure to be filled out
 *
 * @retval CPA_STATUS_SUCCESS  Successfully received stats
 * @retval CPA_STATUS_FAIL     Failed to get stats
 *
 *
 *****************************************************************************/

CpaStatus QatCtrl_FWCountGet_dh895x(sal_qat_service_t* qatInstance,
                                    qat_admin_stats_t *pQatStats);

/**
 *******************************************************************************
 * @ingroup qat_ctrl
 *      Send an heartbeat query
 *
 * @description
 *      This function sends two admin messages down to the AEs,
 *      HEARTBEAT_SYNC and HEARTBEAT_GET to understand if the device is hung.
 *
 * @threadSafe
 *      Yes
 *
 * @param qatInstanceHandle    IN  qat handle
 * @param delay                IN  delay beween hearbeat SYNC and GET in ms
 * @param heartbeatStatus      OUT reports the result of an heartbeat query
 *
 * @retval CPA_STATUS_SUCCESS  Successfully sent message and got a response.
 * @retval CPA_STATUS_FAIL     Message was not sent or didn't get a response.
 *
 *
 *****************************************************************************/
CpaStatus QatCtrl_SendHeartbeat_dh895x(sal_qat_service_t *qatInstance,
                                       Cpa32U delay,
                                       CpaStatus *heartbeatStatus);
#endif/* QAT_INIT_ADMIN_DH895X_H */
