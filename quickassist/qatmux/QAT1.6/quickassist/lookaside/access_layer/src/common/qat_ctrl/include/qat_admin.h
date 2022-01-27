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
 *      callback data for ADMIN messages
 *
 *****************************************************************************/
typedef struct qatal_admin_cb_info_s
{
    sal_qat_service_t* instance;
    /**< QAT instance the request is sent from */
    Cpa32U aeIndex;
    /**< AE index the request is sent to */
    qat_admin_stats_t *pStats;
    /**< qat stats structure that is updated in callback */
    LAC_ARCH_UINT pCb;
    LAC_ARCH_UINT pCallbackTag;
} qatal_admin_cb_info_t;

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


#endif
