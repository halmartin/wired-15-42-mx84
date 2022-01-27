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
 * @file lac_sal_types_qat_ctrl.h
 *
 * @defgroup QAT Controller.
 *
 * @ingroup qat_ctrl
 *
 * @description
 *      This header stores the qat instance structure and associated variables
 *
 *****************************************************************************/

#ifndef LAC_SAL_TYPES_QAT_CTRL_H
#define LAC_SAL_TYPES_QAT_CTRL_H

/*
 * Used to monitor the response from the AEs
 * For the case of the init/admin the response
 * type is either 0, 1 or 2.
 */
#define QAT_NUM_CMD_TYPES   (3)

typedef void (*lac_qat_resp_handler_func_t)
     (void *pRespMsg, icp_arch_if_request_t msgType);

typedef struct sal_qat_service_s
{
    sal_service_t generic_service_info;
    /**< An instance of the Generic Service Container */
    icp_qat_fw_init_ring_table_t *pMasterRingTable;
    icp_qat_fw_init_ring_table_t *pCyARingTable;
    icp_qat_fw_init_ring_table_t *pCyBRingTable;
    icp_qat_fw_init_ring_table_t *pDcRingTable;
    Cpa64U RingTableDmaAddr;
    Cpa32U ringNumTx;
    Cpa32U ringNumRx;
    Cpa32U nodeId;
    Cpa32U max_qat;
    /**< maximum QAT for this SKU */

    Cpa32U max_aes;
    /**< maximum AEs for this SKU */

    Cpa32U num_aes;
    /**< number AEs in the system, will depend on the SKU */

    Cpa32U num_aes_per_qat;
    /**< number of AEs available to the qat instance
         (either all the AEs or half of them)*/

    Cpa32U active_aes_per_qat;
    /**< number of AEs in use by the qat instance, will
         depend on the SKU and the services enabled */

    Cpa32U *qatAeMask;
    /**< array describing the SKUing info */

    Cpa8U *aeTargetIds;
    /**< array containing valid AE target Ids, will depend on the SKU */

    Cpa8U *serviceMask;
    /**< array describing what services are enabled */

    lac_qat_resp_handler_func_t qatRespHandlerTbl[QAT_NUM_CMD_TYPES];

    OsalAtomic adminCallbackPending;
    /**< Atomic variable which manages in-flight admin messages. */

    icp_comms_trans_handle trans_handle_qat_admin_tx;
    icp_comms_trans_handle trans_handle_qat_admin_rx;

    sal_service_debug_t *debug;
    /**< Debug handler */
} sal_qat_service_t;

#endif /*LAC_SAL_TYPES_QAT_CTRL_H*/
