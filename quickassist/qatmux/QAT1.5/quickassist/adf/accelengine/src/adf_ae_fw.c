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
 * @file adf_ae_fw.c
 *
 * @description
 *        This file contains the firmware management code for the Acceleration
 *        Engine Driver.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_firml_interface.h"
#include "adf_ae_fw.h"
#include "adf_ae.h"
#include "adf_cfg.h"
#include "adf_fw.h"
#include "Osal.h"

/*
 * Type describing mmp firmware version
 */
typedef struct adf_mmp_version_s {
        Cpa8U ver_val[MMP_VERSION_LEN];
} adf_mmp_version_t;

/*
 *  Loads the firmware into memory for the specified accelerator.
 */
CpaStatus adf_aefwLoadFirmware(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status, status_size, status_virt, status_phys = CPA_STATUS_FAIL;
    char uof_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char mmp_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    void *pFwAddr = NULL;
    void *pNewFwAddr = NULL, *pUofFwAddr = NULL;
    Cpa64U pNewFwPhysAddr = 0, val = 0;
    Cpa32U fwSize = 0, fwRemappingSize = 0;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;
    adf_mmp_version_t mmp_ver = {{0}};
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    /* Find for which device this information will be
     * intended for. */
    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    status = icp_adf_cfgGetParamValue(pAccelDev, GENERAL_SEC,
                                      ADF_AE_FW_MOF_NAME_KEY, uof_name);

    ICP_CHECK_STATUS_AND_LOG(status,
            "Parameter %s is not configured.\n", ADF_AE_FW_MOF_NAME_KEY);
    status = adf_fw_request(pAccelDev, ADF_FW_UOF_TYPE, uof_name);
    ICP_CHECK_STATUS_AND_LOG(status,
            "UOF Firmware not available, status = %d\n", status);

    /* Get the FW Addr and Size. */
    status = adf_fw_get_info(pAccelDev, ADF_FW_UOF_TYPE, &pFwAddr, &fwSize);
    ICP_CHECK_STATUS_AND_LOG(status,
            "Failed to get UOF address and size. status = %d\n", status);

    /*
     * Create a buffer of memory capable of storing the UoF firmware image.
     * The local copy will then be used to download the FW to the AEs
     * and can be kept in memory in the event of a suspend/resume.
     * The FW can also be released as normal using OS specific commands
     * once the memory is copied.
     */

    /*
     * Allocate memory, copy the uof firmware, and add its size and address
     * to the internal section of the config table.
     */

    pUofFwAddr = ICP_VMALLOC(fwSize);
    if(NULL == pUofFwAddr)
    {
        ADF_ERROR("Failed to alloc 64byte aligned mem for local"
                  " copy of uof firmware\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMCPY(pUofFwAddr, pFwAddr, fwSize);

    /*
     * Add the local copies to the config table.
     * When the memory address is needed again it can be queried from
     * the config file.
     * The memory will be freed upon a system shutdown.
     */

    /* Add the FW address and size to the "INTERNAL" section of the
     * internal_section list. Find the pointer to that section. */

    pSectionNode = adf_cfgSectionFind(pCfgData->internal_section,
                                          INTERNAL_SEC);
    if(NULL == pSectionNode)
    {
        ADF_ERROR("The %s section could not be found.\n", INTERNAL_SEC);
        ICP_VFREE(pUofFwAddr);
        return CPA_STATUS_FAIL;
    }

    status_virt = adf_cfgAddKeyValueParam(pAccelDev,pSectionNode,
                                     ICP_CFG_UOF_ADDRESS_KEY,
                                     pUofFwAddr, ADF_HEX);
    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_virt,
            "Failed to set %s\n", ICP_CFG_UOF_ADDRESS_KEY);

    val = fwSize;
    status_size = adf_cfgAddKeyValueParam(pAccelDev,pSectionNode,
                                     ICP_CFG_UOF_SIZE_BYTES_KEY,
                                     (void *)&val, ADF_DEC);
    ICP_CHECK_STATUS_AND_LOG_NORETURN(status_size,
            "Failed to set %s\n", ICP_CFG_UOF_SIZE_BYTES_KEY);

    /* Free the memory we have allocated as config table
     * entries could not be created properly. Logging of
     * the problem will be done above.
     */
    if((status_size != CPA_STATUS_SUCCESS) ||
       (status_virt != CPA_STATUS_SUCCESS))
    {
        ICP_VFREE(pUofFwAddr);
        return CPA_STATUS_FAIL;
    }

    /* All MMP FW stuff from here on down. */

    status = icp_adf_cfgGetParamValue(pAccelDev, GENERAL_SEC,
                                      ADF_AE_FW_MMP_NAME_KEY, mmp_name);

    ICP_CHECK_STATUS_AND_LOG(status,
            "Parameter %s is not configured.\n", ADF_AE_FW_MMP_NAME_KEY);
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
    fwRemappingSize = osalIOMMUgetRemappingSize(fwSize);
    pNewFwAddr = ICP_MALLOC_GEN(fwRemappingSize);

    if(NULL == pNewFwAddr)
    {
        ADF_ERROR("Failed to alloc 64byte aligned mem for MMP firmware\n");
        return CPA_STATUS_FAIL;
    }

    pNewFwPhysAddr = ICP_VIRT_TO_PHYS(pNewFwAddr);
    if(osalIOMMUMap(pNewFwPhysAddr, pNewFwPhysAddr, fwRemappingSize))
    {
        ADF_ERROR("Failed to remap address of MMP firmware\n");
        return CPA_STATUS_FAIL;
    }

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

/*
 * Releases the resources used by the previously allocated
 * firmware for the specified accelerator.
 */
CpaStatus adf_aefwReleaseFirmware(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    if (pAccelDev->pUofFirmwareLocation)
    {
        /* Release the in-kernel firmware structure */
        status = adf_fw_release(pAccelDev, ADF_FW_UOF_TYPE);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("Failed to release UOF firmware.\n");
        }
        pAccelDev->pUofFirmwareLocation = NULL;
    }
    if (pAccelDev->pMmpFirmwareLocation)
    {
        status = adf_fw_release(pAccelDev, ADF_FW_MMP_TYPE);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("Failed to release MMP firmware.\n");
        }
        pAccelDev->pMmpFirmwareLocation = NULL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Returns the address and size of the requested firmware image,
 * for the specified accelerator.
 */
CpaStatus adf_aefwGetFirmware(icp_accel_dev_t *pAccelDev,
                              Cpa32S type,
                              void **pAddr,
                              Cpa32U *pSize)
{
    CpaStatus status_addr = CPA_STATUS_FAIL;
    CpaStatus status_size = CPA_STATUS_FAIL;
    char config_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES]={0};

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAddr);
    ICP_CHECK_FOR_NULL_PARAM(pSize);

    /* Query the config file for FW address and size. */
    switch(type)
    {
        case ADF_FW_UOF_TYPE:
            status_addr = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                                  ICP_CFG_UOF_ADDRESS_KEY, config_value);
            *pAddr = (void *)ICP_STRTOUL(config_value, NULL, ADF_CFG_BASE_HEX);
            status_size = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                                  ICP_CFG_UOF_SIZE_BYTES_KEY, config_value);
            *pSize = (Cpa32U)ICP_STRTOUL(config_value, NULL, ADF_CFG_BASE_DEC);
            break;
        case ADF_FW_MMP_TYPE:
            status_addr = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                                  ICP_CFG_MMP_ADDRESS_KEY, config_value);
            *pAddr = (void *)ICP_STRTOUL(config_value, NULL, ADF_CFG_BASE_HEX);
            status_size = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                                  ICP_CFG_MMP_SIZE_BYTES_KEY, config_value);
            *pSize = (Cpa32U)ICP_STRTOUL(config_value, NULL, ADF_CFG_BASE_DEC);
            break;
        default:
            ADF_ERROR("Firmware type %d not supported\n",type);
            return CPA_STATUS_FAIL;
    }
    if(CPA_STATUS_SUCCESS != status_addr || CPA_STATUS_SUCCESS != status_size)
    {
        ADF_ERROR("Failed to find the FW addr/size in the config table\n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Frees the aligned MMP buffer which ADF has created
 */
void adf_freeMmpBuffer(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa64U *pMmpVirtAddr = NULL;
    Cpa64U pMmpPhyAddr = 0;
    Cpa32U mmpSize = 0;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if (CPA_STATUS_SUCCESS != status)
    {
            ADF_ERROR("The device could not be found\n");
            return;
    }
    else
    {
        pSectionNode = adf_cfgSectionFind(pCfgData->internal_section,
                                          INTERNAL_SEC);
    }
    if (NULL == pSectionNode)
    {
        ADF_ERROR("The INTERNAL section could not be found\n");
        return;
    }

    status = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                             ICP_CFG_MMP_PHYS_ADDRESS_KEY, valStr);

    if(CPA_STATUS_SUCCESS == status)
    {
        pMmpPhyAddr = ICP_STRTOULL(valStr, NULL, ADF_CFG_BASE_HEX);
        status = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                             ICP_CFG_MMP_SIZE_BYTES_KEY, valStr);
        mmpSize = ICP_STRTOUL(valStr, NULL, ADF_CFG_BASE_DEC);
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        if(osalIOMMUUnmap((UINT64)pMmpPhyAddr,
                           osalIOMMUgetRemappingSize(mmpSize)))
        {
            ADF_ERROR("Failed to unmap iommu FW address\n");
        }
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        status = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                             ICP_CFG_MMP_ADDRESS_KEY, valStr);
        pMmpVirtAddr = (Cpa64U*)(UARCH_INT)ICP_STRTOULL(valStr, NULL,
                                                   ADF_CFG_BASE_HEX);
        ICP_FREE(pMmpVirtAddr);
        status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                      ICP_CFG_MMP_ADDRESS_KEY);

        if(CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to delete key: %s\n",
                                      ICP_CFG_MMP_ADDRESS_KEY);
        }

        status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                      ICP_CFG_MMP_PHYS_ADDRESS_KEY);

        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to delete key: %s\n",
                     ICP_CFG_MMP_PHYS_ADDRESS_KEY);
        }

        status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                     ICP_CFG_MMP_SIZE_BYTES_KEY);

        if (CPA_STATUS_SUCCESS != status)
        {
           ADF_ERROR("Failed to delete key: %s\n",
                      ICP_CFG_MMP_SIZE_BYTES_KEY);
        }
    }
}

/*
 * Frees the aligned UOF buffer which ADF has created
 */
void adf_freeUofBuffer(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    void *pUofAddr = NULL;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    status = icp_adf_cfgGetParamValue(pAccelDev, INTERNAL_SEC,
                             ICP_CFG_UOF_ADDRESS_KEY, valStr);

    if(CPA_STATUS_SUCCESS == status)
    {
        pUofAddr = (void *)ICP_STRTOUL(valStr, NULL,
                                        ADF_CFG_BASE_HEX);
        ICP_VFREE(pUofAddr);
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
            ADF_ERROR("The INTERNAL section could not be found\n");
        }
        else
        {
            status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                      ICP_CFG_UOF_ADDRESS_KEY);

            if(CPA_STATUS_SUCCESS != status)
            {
                ADF_ERROR("Failed to delete key: %s\n",
                                      ICP_CFG_UOF_ADDRESS_KEY);
            }

            status = adf_cfgDelKeyValueParam(pAccelDev, pSectionNode,
                     ICP_CFG_UOF_SIZE_BYTES_KEY);

            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_PRINT("Failed to delete key: %s\n",
                      ICP_CFG_UOF_SIZE_BYTES_KEY);
            }
        }
    }
}
