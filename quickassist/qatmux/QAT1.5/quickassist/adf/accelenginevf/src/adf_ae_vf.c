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
 * @file adf_ae.c
 *
 * @description
 *      OS Agnostic Code to Manage / Control the Acceleration Engines
 *      Provide functions to:
 *      init/shutdown AE device
 *      map/patch/download/release microcode
 *      stop/start/reset Acceleration Engines
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_firml_interface.h"
#include "adf_ae.h"
#include "adf_ae_fw.h"
#include "adf_fw.h"
#include "adf_cfg.h"
#include "icp_accel_devices.h"
#include "adf_devmgr.h"
#include "icp_adf_ae.h"
#include "adf_sysmeminfo.h"
#include "adf_init.h"

#define TOOLS_VERSION "7.0.119 vf"
#define UOF_VERSION "0 . 5 . 0 vf"

/*
 * Type describing mmp firmware version
 */
typedef struct adf_mmp_version_s {
        Cpa8U ver_val[MMP_VERSION_LEN];
} adf_mmp_version_t;

CpaStatus icp_adf_aePatchSymbol(icp_accel_dev_t *pAccelDev,
                                Cpa32S aeNum,
                                char* pSymName,
                                Cpa32S symValue)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aefwGetVersion(icp_accel_dev_t *pAccelDev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    /* Find for which device this information will be
     * intended for. */
    stat = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't find config dev\n");
        return stat;
    }

    pSectionNode = adf_cfgSectionFind(pCfgData->config_section,
                                          GENERAL_SEC);
    if(NULL == pSectionNode)
    {
        ADF_ERROR("The %s section could not be found.\n", GENERAL_SEC);
        return CPA_STATUS_FAIL;
    }
    stat = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                   ICP_CFG_TOOLS_VER_KEY,
                                   TOOLS_VERSION, ADF_STR);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't add tools version to config\n");
        return stat;
    }

    stat = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                   ICP_CFG_UOF_VER_KEY,
                                   UOF_VERSION, ADF_STR);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't add uof version to config\n");
    }

    return stat;
}

CpaStatus adf_aeStart(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

/*
 * Frees the aligned MMP buffer which ADF has created
 */
CpaStatus adf_aeUcodeRelease(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus ret    = CPA_STATUS_SUCCESS;
    Cpa64U *pMmpVirtAddr = NULL;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    status = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                             ICP_CFG_MMP_ADDRESS_KEY, valStr);

    if(CPA_STATUS_SUCCESS == status)
    {
        status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("The device could not be found\n");
        }
        else
        {
            pSectionNode = adf_cfgSectionFind(pCfgData->internal_section,
                                          INTERNAL_SEC);
        }
        if (NULL == pSectionNode)
        {
            status = CPA_STATUS_FAIL;
            ADF_ERROR("The INTERNAL section could not be found\n");
        }
        else
        {
            pMmpVirtAddr = (Cpa64U *)ICP_STRTOUL(valStr, NULL,
                                             ADF_CFG_BASE_HEX);

            ICP_FREE(pMmpVirtAddr);
            pAccelDev->pMmpFirmwareLocation = NULL;

            status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                      ICP_CFG_MMP_ADDRESS_KEY);

            if(CPA_STATUS_SUCCESS != status)
            {
                ret = status;
                ADF_PRINT("Failed to delete key: %s\n",
                                      ICP_CFG_MMP_ADDRESS_KEY);
            }

            status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                      ICP_CFG_MMP_PHYS_ADDRESS_KEY);

            if (CPA_STATUS_SUCCESS != status)
            {
                ret = status;
                ADF_PRINT("Failed to delete key: %s\n",
                      ICP_CFG_MMP_PHYS_ADDRESS_KEY);
            }

            status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                     ICP_CFG_MMP_SIZE_BYTES_KEY);

            if (CPA_STATUS_SUCCESS != status)
            {
                ret = status;
                ADF_PRINT("Failed to delete key: %s\n",
                      ICP_CFG_MMP_SIZE_BYTES_KEY);
            }

            status = ret;
        }
    }
    return status;
}

void adf_freeMmpBuffer(icp_accel_dev_t *pAccelDev)
{
}

CpaStatus adf_aeInit(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aeShutdown(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aeStop(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

void adf_freeUofBuffer(icp_accel_dev_t *pAccelDev)
{
}

CpaStatus adf_aeUcodeDownload(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aeReleaseFirmware(icp_accel_dev_t *pAccelDev)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aefwLoadFirmware(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status, status_size, status_virt, status_phys = CPA_STATUS_FAIL;
    char mmp_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    void *pFwAddr = NULL;
    void *pNewFwAddr = NULL;
    Cpa64U pNewFwPhysAddr = 0, val = 0;
    Cpa32U fwSize = 0;
    adf_mmp_version_t mmp_ver = {{0}};
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    /* Find for which device this information will be
     * intended for. */
    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    pSectionNode = adf_cfgSectionFind(pCfgData->internal_section,
                                          INTERNAL_SEC);
    if(NULL == pSectionNode)
    {
        ADF_ERROR("The %s section could not be found.\n", INTERNAL_SEC);
        return CPA_STATUS_FAIL;
    }

    status = icp_adf_cfgGetParamValue(pAccelDev, GENERAL_SEC,
                                      ADF_AE_FW_MMP_NAME_KEY, mmp_name);

    ICP_CHECK_STATUS_AND_LOG(status,
            "Parameter %s is not configured.\n", ADF_AE_FW_MMP_NAME_KEY);

    ADF_PRINT("Calling request_firmware for MMP ..\n");

    status = adf_fw_request(pAccelDev, ADF_FW_MMP_TYPE, mmp_name);

    ICP_CHECK_STATUS_AND_LOG(status,
            "MMP Firmware not available, status = %d\n", status);

    /*
     * Set the ICP_ADF_CFG_PARAM_MMP_ADDRESS and
     * ICP_ADF_CFG_PARAM_MMP_SIZE_BYTES parameters in the
     * along with the new physical address of the MMP buffer
     * in the config table
     */
    status = adf_fw_get_info(pAccelDev, ADF_FW_MMP_TYPE, &pFwAddr, &fwSize);

    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to get MMP address and size. status = %d.\n", status);

    /*
    * Allocate 64 byte aligned memory, copy the mmp firmware
    * and add the size, virtual address and physical address
    * to the internal section of the config table
    */
    pNewFwAddr = ICP_MALLOC_GEN(fwSize);
    if(NULL == pNewFwAddr)
    {
        ADF_ERROR("Failed to alloc 64byte aligned mem for MMP firmware\n");
        return CPA_STATUS_FAIL;
    }

    pNewFwPhysAddr = ICP_VIRT_TO_PHYS(pNewFwAddr);
    ICP_MEMCPY(pNewFwAddr, pFwAddr, fwSize);

    /* get mmp version */

    ICP_MEMCPY(&mmp_ver, pNewFwAddr, MMP_VERSION_LEN);
    /*
     * Store physical mmp address as a string since
     * it is a 64bit value also on 32bit machine.
     */
    snprintf(mmp_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES, "0x%llX",
                                  (long long unsigned int)pNewFwPhysAddr);
    status_phys = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                     ICP_CFG_MMP_PHYS_ADDRESS_KEY,
                                     mmp_name, ADF_STR);

    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_phys,
            "Failed to set %s\n", ICP_CFG_MMP_PHYS_ADDRESS_KEY);

    status_virt = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                     ICP_CFG_MMP_ADDRESS_KEY,
                                     pNewFwAddr, ADF_HEX);

    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_virt,
            "Failed to set %s\n", ICP_CFG_MMP_ADDRESS_KEY);

    val = fwSize;
    status_size = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                     ICP_CFG_MMP_SIZE_BYTES_KEY,
                                     (void *)&val, ADF_DEC);

    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_size,
            "Failed to set %s\n", ICP_CFG_MMP_SIZE_BYTES_KEY);

    ICP_MEMSET(mmp_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    snprintf(mmp_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES, "%d.%d.%d",
                                             mmp_ver.ver_val[MMP_VER_0],
                                             mmp_ver.ver_val[MMP_VER_1],
                                             mmp_ver.ver_val[MMP_VER_2]);

    pSectionNode = adf_cfgSectionFind(pCfgData->config_section,
                                          GENERAL_SEC);
    if(NULL == pSectionNode)
    {
        ADF_ERROR("The %s section could not be found.\n", GENERAL_SEC);
        ICP_FREE(pNewFwAddr);
        return CPA_STATUS_FAIL;
    }

    status_size = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                     ICP_CFG_MMP_VER_KEY,
                                     mmp_name, ADF_STR);

    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_size,
            "Failed to set %s\n", ICP_CFG_MMP_VER_KEY);

    /* Free the memory we have allocated as config table
     * entries could not be created properly. Logging of
     * the problem will be done above.
     */
    if((status_size != CPA_STATUS_SUCCESS || status_virt != CPA_STATUS_SUCCESS
        || status_phys != CPA_STATUS_SUCCESS))
    {
        ICP_FREE(pNewFwAddr);
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_aeFwLoad(icp_accel_dev_t *pAccelDev)
{
    return adf_aefwLoadFirmware(pAccelDev);
}

