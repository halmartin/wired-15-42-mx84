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
 * @file  adf_esram.c
 *
 * @description
 *        This file contains the function for reading the eSRAM addr from the
 *        CSRs for the Acceleration Driver Framework (ADF).
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "icp_adf_esram.h"

/*
 * adf_esramSetConfigInfo
 * This function stores the eSRAM information to the device's
 * internal configuration table.
 */
CpaStatus adf_esramSetConfigInfo(icp_accel_dev_t *pAccelDev,
                                 Cpa64U physAddr,
                                 UARCH_INT virtAddr,
                                 Cpa32U size)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSection = NULL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    ADF_DEBUG("eSRAM: physAddr=0x%x, virtAddr=0x%x, size=%u\n",
                (UARCH_INT)physAddr, virtAddr, size);

    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    ICP_CHECK_STATUS_AND_LOG(status,
            "%s\n", "Device configuration not found");

    if (NULL == pCfgData->internal_section) {
        ADF_ERROR("Internal section does not exist\n");
        return CPA_STATUS_FAIL;
    }

    pSection = pCfgData->internal_section;

    /* Set physical address */
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(valStr, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                        "%llx", (long long unsigned int)physAddr);
    status = adf_cfgAddKeyValueParam(pAccelDev, pSection,
                        ADF_SRAM_PHYSICAL_ADDRESS, (void *)valStr, ADF_STR);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to set %s\n", ADF_SRAM_PHYSICAL_ADDRESS);

    /* Set virtual address */
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(valStr, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                        "%p", (void *)virtAddr);
    status = adf_cfgAddKeyValueParam(pAccelDev, pSection,
                        ADF_SRAM_VIRTUAL_ADDRESS, (void *)valStr, ADF_STR);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to set %s\n", ADF_SRAM_VIRTUAL_ADDRESS);

    /* Set size in bytes */
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(valStr, ADF_CFG_MAX_VAL_LEN_IN_BYTES,"%u", size);
    status = adf_cfgAddKeyValueParam(pAccelDev, pSection,
                        ADF_SRAM_SIZE_IN_BYTES, (void *)valStr, ADF_STR);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to set %s\n", ADF_SRAM_SIZE_IN_BYTES);

    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_esramGetAddress
 * This function retrieves the eSRAM information stored in the device's
 * internal configuration table.
 */
CpaStatus icp_adf_esramGetAddress(icp_accel_dev_t *accel_dev,
                                  Cpa32U accelNumber,
                                  Cpa64U *pPhysAddr,
                                  Cpa64U *pVirtAddr,
                                  Cpa32U *pSize)
{
    CpaStatus status = CPA_STATUS_FAIL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                INTERNAL_SEC, ADF_SRAM_PHYSICAL_ADDRESS, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to get %s value\n", ADF_SRAM_PHYSICAL_ADDRESS);

    *pPhysAddr = (Cpa64U)ICP_STRTOULL(valStr, NULL, ADF_CFG_BASE_HEX);

    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                INTERNAL_SEC, ADF_SRAM_VIRTUAL_ADDRESS, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to get %s value\n", ADF_SRAM_VIRTUAL_ADDRESS);

    *pVirtAddr = (Cpa64U)ICP_STRTOULL(valStr, NULL, ADF_CFG_BASE_HEX);

    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                INTERNAL_SEC, ADF_SRAM_SIZE_IN_BYTES, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to get %s value\n", ADF_SRAM_SIZE_IN_BYTES);

    *pSize = (Cpa32U)ICP_STRTOUL(valStr, NULL, ADF_CFG_BASE_DEC);

    *pSize = (*pSize) >> 1;
    if(1 == accelNumber)
    {
        *pPhysAddr += *pSize;
        *pVirtAddr += *pSize;
    }
    return CPA_STATUS_SUCCESS;
}
