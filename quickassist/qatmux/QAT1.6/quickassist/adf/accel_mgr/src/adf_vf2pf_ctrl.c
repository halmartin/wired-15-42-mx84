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
 *  version: QAT1.6.L.2.6.0-65
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
#include "icp_sal_versions.h"

#define CONFIRMATION_MAX_LOOPS  1000
#define CONFIRMATION_DELAY      2

/*
 * Notify PF about VF init
 */
CpaStatus adf_VF2PF_init(icp_accel_dev_t *accel_dev)
{
    Cpa32U val = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

    /* Build a message to PF */
    adf_vf2pf_set_msg_type(&val, ICP_VF2PF_MSGTYPE_INIT);
    adf_vf2pf_set_msg_origin(&val,ICP_PFVF_MSGORIGIN_SYSTEM);

    return accel_dev->pHwDeviceData->
        adf_systemMsg_VF2PF_putMsg(accel_dev, val);

}

/*
 * Notify PF about VF shutdown
 */
CpaStatus adf_VF2PF_shutdown(icp_accel_dev_t *accel_dev)
{
    Cpa32U  val = 0;
    
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

    /* Build a message to PF */
    adf_vf2pf_set_msg_type(&val, ICP_VF2PF_MSGTYPE_SHUTDOWN);
    adf_vf2pf_set_msg_origin(&val,ICP_PFVF_MSGORIGIN_SYSTEM);

    return accel_dev->pHwDeviceData->
        adf_systemMsg_VF2PF_putMsg(accel_dev, val);
    
}

/*
 * adf_VF2PF_RequestVersion
 * 
 * request the QAT driver version from the PF. so
 * we can understand if the VF driver version is supported or not 
 */
CpaStatus adf_VF2PF_RequestVersion(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U val = 0, ctr = 0;
    CpaBoolean respAvailable = CPA_FALSE;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa8U isVFSupported = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    hw_data = accel_dev->pHwDeviceData;
    
    /* Build a message to PF */
    adf_vf2pf_set_msg_type(&val, ICP_VF2PF_MSGTYPE_VERSION_REQ);
    adf_vf2pf_set_msg_origin(&val,ICP_PFVF_MSGORIGIN_SYSTEM);
    adf_vf2pf_set_version(&val, SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
            SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);
    
    /* clear response flag in case this was previously done */
    hw_data->icp_version.versionMsgAvailable = CPA_FALSE;

    /* Send the message */
    status = accel_dev->pHwDeviceData->
        adf_systemMsg_VF2PF_putMsg(accel_dev, val);

    if (CPA_STATUS_SUCCESS == status)
    {
        /* And wait for confirmation */
        do{
            ICP_MSLEEP(CONFIRMATION_DELAY);
            respAvailable = hw_data->icp_version.versionMsgAvailable;
        } while(respAvailable != CPA_TRUE && ctr++ < CONFIRMATION_MAX_LOOPS);

        if (CPA_TRUE == respAvailable)
        {
            isVFSupported = hw_data->icp_version.supported;

            switch(isVFSupported)
            {
            /*first find out if PF has decided compatibility */
            case ICP_PFVF_VF_NOTSUPPORTED:
            {
                ADF_ERROR("PF (%d.%d) and VF (%d.%d) QA drivers are "
                        "not compatible."
                        " Please upgrade VF to version supported by PF. See "
                        "PF version release notes for details\n",
                        hw_data->icp_version.pf_major,
                        hw_data->icp_version.pf_minor,
                        SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                        SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);
                status = CPA_STATUS_FAIL;
                break;
            }
            case ICP_PFVF_VF_SUPPORTED:
            {
                status = CPA_STATUS_SUCCESS;
                break;
            }
            case ICP_PFVF_VF_SUPPORTUNKNOWN:
            {
                /* the VF has to decide if versions are compatible.
                 * This only happens if the VF is a later version than the PF*/
                if ( hw_data->icp_version.pf_major
                      < hw_data->icp_version.device_lowestSupportMajorVersion)
                {
                    status = CPA_STATUS_FAIL;
                }

                if (CPA_STATUS_SUCCESS == status)
                {
                    if (hw_data->icp_version.pf_major ==
                        hw_data->icp_version.device_lowestSupportMajorVersion
                        &&
                        hw_data->icp_version.pf_minor <
                        hw_data->icp_version.device_lowestSupportMinorVersion)
                    {
                        status = CPA_STATUS_FAIL;
                    }
                    else
                    {
                        status = CPA_STATUS_SUCCESS;
                    }
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("PF (%d.%d) and VF (%d.%d) QA drivers are not "
                         "compatible."
                         " Please upgrade PF or downgrade VF to version "
                         "supported by PF. See release notes for details\n",
                            hw_data->icp_version.pf_major,
                            hw_data->icp_version.pf_minor,
                            SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                            SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER);
                }
                break;
            }
            default:
                ADF_ERROR("Invalid result received from PF. "
                        "Assume not compatible with VF.\n");
                status = CPA_STATUS_FAIL;
            }
        }
        else
        {
            ADF_ERROR("No response received from PF version check. "
                    "Assume not compatible with VF\n");
            status = CPA_STATUS_FAIL;
        }
    }
    else
    {
        ADF_ERROR("Failed to send version check msg to PF. "
                "           Assume not compatible with VF.\n");
    }
    ADF_DEBUG("%s: status = %d\n", __FUNCTION__, status);

    return status;
}
