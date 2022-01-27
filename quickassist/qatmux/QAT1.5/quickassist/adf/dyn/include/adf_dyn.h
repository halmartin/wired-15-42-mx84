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

/******************************************************************************
 * @file adf_dyn.h
 *
 * @description
 *      This header file contains the function prototypes for dyn control
 *      layer.
 *
 *****************************************************************************/
#ifndef ADF_DYN_CP_H
#define ADF_DYN_CP_H

#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"

#define DYN_SUBSYSTEM_NAME          ("DYN")
#define DYN_CY_INST_NR_PARAM           ("NumberCyInstances")
#define DYN_CY_INST_NAME_PARAM         ("Cy%dName")
#define DYN_CY_INST_IS_POLLED_PARAM    ("Cy%dIsPolled")
#define DYN_CY_INST_ACCEL_NR_PARAM     ("Cy%dAcceleratorNumber")
#define DYN_CY_INST_EXEC_ENG_PARAM     ("Cy%dExecutionEngine")
#define DYN_CY_INST_BANK_NR_PARAM      ("Cy%dBankNumber")
#define DYN_CY_INST_ASYM_TX_PARAM      ("Cy%dRingAsymTx")
#define DYN_CY_INST_ASYM_RX_PARAM      ("Cy%dRingAsymRx")
#define DYN_CY_INST_SYM_TX_H_PARAM     ("Cy%dRingSymTxHi")
#define DYN_CY_INST_SYM_TX_L_PARAM     ("Cy%dRingSymTxLo")
#define DYN_CY_INST_SYM_RX_H_PARAM     ("Cy%dRingSymRxHi")
#define DYN_CY_INST_SYM_RX_L_PARAM     ("Cy%dRingSymRxLo")
#define DYN_CY_INST_SYM_REQUESTS_PARAM  ("Cy%dNumConcurrentSymRequests")
#define DYN_CY_INST_ASYM_REQUESTS_PARAM ("Cy%dNumConcurrentAsymRequests")

#define DYN_DC_INST_NR_PARAM           ("NumberDcInstances")
#define DYN_DC_INST_NAME_PARAM         ("Dc%dName")
#define DYN_DC_INST_IS_POLLED_PARAM    ("Dc%dIsPolled")
#define DYN_DC_INST_ACCEL_NR_PARAM     ("Dc%dAcceleratorNumber")
#define DYN_DC_INST_BANK_NR_PARAM      ("Dc%dBankNumber")
#define DYN_DC_INST_RING_TX_PARAM      ("Dc%dRingTx")
#define DYN_DC_INST_RING_RX_PARAM      ("Dc%dRingRx")
#define DYN_DC_INST_REQUESTS_PARAM     ("Dc%dNumConcurrentRequests")

/**
 * DYN cy instance
 * Instance can be only RX or TX
 */
typedef struct dyn_cy_instance_s{
    Cpa32U instance_nr;
    icp_accel_dev_t *dev;
    icp_comms_trans_handle *asym_tx;
    icp_comms_trans_handle *sym_tx_lo;
    icp_comms_trans_handle *sym_tx_hi;
    icp_comms_trans_handle *asym_rx;
    icp_comms_trans_handle *sym_rx_lo;
    icp_comms_trans_handle *sym_rx_hi;
    struct dyn_cy_instance_s *pNext;
    struct dyn_cy_instance_s *pPrev;
} dyn_cy_instance_t;

/**
 * DYN dc instance
 * Instance can be only RX or TX
 */
typedef struct dyn_dc_instance_s{
    Cpa32U instance_nr;
    icp_accel_dev_t *dev;
    icp_comms_trans_handle *dc_tx;
    icp_comms_trans_handle *dc_rx;
    struct dyn_dc_instance_s *pNext;
    struct dyn_dc_instance_s *pPrev;
} dyn_dc_instance_t;

/**
 *****************************************************************************
 * @description
 *      Register dyn event dispatcher into ADF
 *
 *****************************************************************************/
CpaStatus adf_dynRegister(void);

/**
 *****************************************************************************
 * @description
 *      Unregister dyn event dispatcher from ADF
 *
 *****************************************************************************/
CpaStatus adf_dynUnregister(void);

/**
 *****************************************************************************
 * @description
 *      Create dyn instance
 *
 *****************************************************************************/
CpaStatus adf_dynCreateInstance(icp_accel_dev_t* accel_dev);

/**
 *****************************************************************************
 * @description
 *      Delete dyn instance
 *
 *****************************************************************************/
CpaStatus adf_dynDeleteInstance(icp_accel_dev_t* accel_dev);

/**
 *****************************************************************************
 * @description
 *      Clear rings on exit and unset the ringsInUseMask bits on
 *      user-space application exit.
 *
 *****************************************************************************/
CpaStatus adf_dynResetRings(icp_accel_dev_t *pAccelDev,
                                 icp_et_ring_data_t *ringData);

/**
 *****************************************************************************
 * @description
 *      Init dyn instance for the crypto. In the function, we will find and
 *      set the accelerator information for the instances.
 *
 *****************************************************************************/
CpaStatus adf_dynCyInitInstance(icp_accel_dev_t *pAccelDev,
                                adf_instancemgr_t  *inst_info);

#endif /* ADF_DYN_CP_H */
