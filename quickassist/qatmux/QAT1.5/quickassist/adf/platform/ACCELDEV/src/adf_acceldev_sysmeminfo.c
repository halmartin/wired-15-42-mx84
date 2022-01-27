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
 * @file adf_acceldev_sysmeminfo.c
 *
 * @description
 *      File contains platform specific sysMemInfo structure
 *      initialization functions
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_firml_interface.h"
#include "icp_accel_devices.h"
#include "adf_platform.h"
#include "adf_sysmeminfo.h"
#include "adf_init.h"

/*
 * adf_initSysMemInfo
 * Initialize the SysMemInfo structure with the information
 * required by firmware loader
 */
CpaStatus adf_initSysMemInfo(icp_accel_dev_t *accelDev,
                             icp_firml_sys_mem_info_t *sysMemInfo)
{
    icp_accel_pci_info_t  *pci_info   = NULL;
    adf_hw_device_data_t *hw_data = NULL;
    adf_esram_data_t esram_info = {0};
    Cpa32U i = 0;
    Cpa32U esramBarId = 0, miscBarId = 0;
    Cpa32U scratchOff = 0, scratchSize = 0;

    ICP_CHECK_FOR_NULL_PARAM(accelDev);
    ICP_CHECK_FOR_NULL_PARAM(accelDev->pHwDeviceData);
    ICP_CHECK_FOR_NULL_PARAM(sysMemInfo);

    pci_info = &accelDev->pciAccelDev;
    hw_data = accelDev->pHwDeviceData;

    /* Set instanceId deviceId and revId*/
    sysMemInfo->instanceId = accelDev->accelId;
    sysMemInfo->deviceId = accelDev->pciAccelDev.deviceId;
    sysMemInfo->revisionId = accelDev->pciAccelDev.revisionId;

    /* Set number of AEs and its mask */
    sysMemInfo->aeMask = accelDev->aeMask;
    sysMemInfo->numAe = hw_data->getNumAccelEngines(hw_data,
            sysMemInfo->aeMask);
    ADF_DEBUG("%s: aeMask=0x%X, numAe=%d\n",
        __FUNCTION__, sysMemInfo->aeMask, sysMemInfo->numAe);

    /* Set number of accelerators and its mask */
    sysMemInfo->qatMask = accelDev->accelMask;
    sysMemInfo->numQat = hw_data->getNumAccelerators(hw_data,
            sysMemInfo->qatMask);

    ADF_DEBUG("%s: qatMask=0x%X, numQat=%d\n",
        __FUNCTION__, sysMemInfo->qatMask, sysMemInfo->numQat);

    /* Set AE clock */
    sysMemInfo->aeClkMhz = hw_data->aeClockInMhz;

    /* Set the number of PCI BARs, its base and virtual address, and size */
    sysMemInfo->numBars = ICP_PCI_MAX_BARS;
    for (i = 0; i < ICP_PCI_MAX_BARS; i++)
    {
        sysMemInfo->pciBars[i].baseAddr = pci_info->pciBars[i].baseAddr;
        sysMemInfo->pciBars[i].virtAddr = pci_info->pciBars[i].virtAddr;
        sysMemInfo->pciBars[i].size = pci_info->pciBars[i].size;
    }

    /* Set the eSRAM base and virtual address, size, AE address, and offset */
    sysMemInfo->sramDesc.sramBaseAddr = 0;
    sysMemInfo->sramDesc.sramSize = 0;
    sysMemInfo->sramDesc.sramBaseAddr_v = 0;
    if (NOT_NULL_FUNCTION(hw_data->getSramBarId))
    {
        esramBarId = hw_data->getSramBarId(hw_data);
        sysMemInfo->sramDesc.sramBaseAddr =
                sysMemInfo->pciBars[esramBarId].baseAddr;
        sysMemInfo->sramDesc.sramSize =
                sysMemInfo->pciBars[esramBarId].size;
        sysMemInfo->sramDesc.sramBaseAddr_v =
                sysMemInfo->pciBars[esramBarId].virtAddr;

        hw_data->getEsramInfo(hw_data, &esram_info);
        sysMemInfo->sramDesc.sramAeAddr = esram_info.sramAeAddr;
        sysMemInfo->sramDesc.startOffset = 0;
    }

    /* Set the Scratch base and virtual address, and size */
    miscBarId = hw_data->getMiscBarId(hw_data);
    sysMemInfo->scratchDesc.scratchBaseAddr = 0;
    sysMemInfo->scratchDesc.scratchBaseAddr_v = 0;
    sysMemInfo->scratchDesc.scratchSize = 0;
    if (NOT_NULL_FUNCTION(hw_data->getScratchRamInfo))
    {
        hw_data->getScratchRamInfo(hw_data, &scratchOff, &scratchSize);

        sysMemInfo->scratchDesc.scratchBaseAddr =
                    sysMemInfo->pciBars[miscBarId].baseAddr + scratchOff;
        sysMemInfo->scratchDesc.scratchBaseAddr_v =
                    sysMemInfo->pciBars[miscBarId].virtAddr + scratchOff;
        sysMemInfo->scratchDesc.scratchSize = scratchSize;
    }

    /* Set the DRAM parameters to zero */
    sysMemInfo->numDramDesc = 0;
    sysMemInfo->dramDesc[0].dramBaseAddr = 0;
    sysMemInfo->dramDesc[0].dramBaseAddr_v = 0;
    sysMemInfo->dramDesc[0].dramSize = 0;
    sysMemInfo->dramDesc[1].dramBaseAddr = 0;
    sysMemInfo->dramDesc[1].dramBaseAddr_v = 0;
    sysMemInfo->dramDesc[1].dramSize = 0;

    /* Set the Revision ID */
    sysMemInfo->revisionId = (unsigned char)pci_info->revisionId;

    /* Set the PCIe Function 0 device pointer */
    sysMemInfo->dev = (void *)pci_info->pDev;


    /* Set the reload flag if the system is restarting*/
    if (BIT_IS_SET(accelDev->adfSubsystemStatus,
                           ADF_STATUS_SYSTEM_RESTARTING)) {
        sysMemInfo->reload = 1;
    }
    else {
        sysMemInfo->reload = 0;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cleanSysMemInfo
 * Clean the SysMemInfo structure
 */
CpaStatus adf_cleanSysMemInfo(icp_accel_dev_t *accelDev,
                                icp_firml_sys_mem_info_t *sysMemInfo)
{
    ICP_CHECK_FOR_NULL_PARAM(accelDev);
    ICP_CHECK_FOR_NULL_PARAM(sysMemInfo);
    return CPA_STATUS_SUCCESS;
}

