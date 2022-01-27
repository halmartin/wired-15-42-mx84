/*****************************************************************************
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

/*****************************************************************************
 * @file adf_vf2pf_ctrl.c
 *
 * @description
 *      VF2PF communication
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "adf_platform.h"
#include "adf_drv_sriov.h"

#define INIT_MAX_LOOPS  1000
#define INIT_SLEEP_TIME 2
#define SHUTDOWN_MAX_LOOPS  1000
#define SHUTDOWN_SLEEP_TIME 2

/*
 * Notify PF about VF init
 */
CpaStatus adf_VF2PF_init(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U *reg = NULL, val = 0, ctr = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U vf2pf_bar_id = 0;
    Cpa32U vf2pf_dbint_offset = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    hw_data = accel_dev->pHwDeviceData;
    vf2pf_bar_id = hw_data->getMiscBarId(hw_data);
    vf2pf_dbint_offset = hw_data->getPf2VfDbIntOffset(0);

    reg = (void*)
        accel_dev->pciAccelDev.pciBars[vf2pf_bar_id].virtAddr;

    /* Build a init message to PF */
    adf_vf2pf_set_msg_type(&val, ICP_PMISCBAR_VF2PFMSGTYPEINIT);
    adf_vf2pf_set_irq(&val);

    /* Send the message */
    ICP_ADF_CSR_WR(reg, vf2pf_dbint_offset, val);

    /* And wait for confirmation */
    do{
        ICP_MSLEEP(INIT_SLEEP_TIME);
        val = ICP_ADF_CSR_RD(reg, vf2pf_dbint_offset);
    } while((ICP_PMISCBAR_VF2PFMSGRINGINFOACK != val) &&
                                               ctr++ < INIT_MAX_LOOPS);
    /* Return success only if we got ACK from PF */
    status = (ICP_PMISCBAR_VF2PFMSGRINGINFOACK == val) ? CPA_STATUS_SUCCESS :
                                                         CPA_STATUS_FAIL;

    ADF_DEBUG("%s: status = %d\n", __FUNCTION__, status);

    return status;
}

/*
 * Notify PF about VF shutdown
 */
CpaStatus adf_VF2PF_shutdown(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U *reg = NULL, val = 0, ctr = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U vf2pf_bar_id = 0;
    Cpa32U vf2pf_dbint_offset = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    hw_data = accel_dev->pHwDeviceData;
    vf2pf_bar_id = hw_data->getMiscBarId(hw_data);
    vf2pf_dbint_offset = hw_data->getPf2VfDbIntOffset(0);

    reg = (void*)
        accel_dev->pciAccelDev.pciBars[vf2pf_bar_id].virtAddr;

    /* Build a init message to PF */
    adf_vf2pf_set_msg_type(&val, ICP_PMISCBAR_VF2PFMSGTYPESHUTDOWN);
    adf_vf2pf_set_irq(&val);

    /* Send the message */
    ICP_ADF_CSR_WR(reg, vf2pf_dbint_offset, val);
    /* And wait for confirmation */
    do{
        ICP_MSLEEP(SHUTDOWN_SLEEP_TIME);
        val = ICP_ADF_CSR_RD(reg, vf2pf_dbint_offset);
    } while((ICP_PMISCBAR_VF2PFMSGRINGINFOACK != val) &&
                                               ctr++ < SHUTDOWN_MAX_LOOPS);
    /* Return success only if we got ACK from PF */
    status = (ICP_PMISCBAR_VF2PFMSGRINGINFOACK == val) ? CPA_STATUS_SUCCESS :
                                                         CPA_STATUS_FAIL;
    ADF_DEBUG("%s: status = %d\n", __FUNCTION__, status);

    return status;
}

