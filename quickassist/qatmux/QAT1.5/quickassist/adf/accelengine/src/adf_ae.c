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


/*
 *  Extracts firmware end tools version.
 */
CpaStatus adf_aefwGetVersion(icp_accel_dev_t *pAccelDev)
{
    adf_fw_loader_handle_t* ldr_handle = NULL;
    char *str = NULL, *ptr = NULL, *cpy = NULL;
    char version[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    const char *tag = "version";
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *pSectionNode = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);
    /* Find for which device this information will be
     * intended for. */
    stat = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't find config dev\n");
        return stat;
    }

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    str = icp_FirmLoader_GetAppMetaData(ldr_handle->firmLoaderHandle,
                                        ICP_FIRMLOADER_BADAE);
    if(NULL == str)
    {
        ADF_ERROR("Can't get firmware version\n");
        return CPA_STATUS_FAIL;
    }
    ADF_DEBUG("FirmLoader_GetAppMetaData returned: %s\n", str);
    pSectionNode = adf_cfgSectionFind(pCfgData->config_section,
                                          GENERAL_SEC);
    if(NULL == pSectionNode)
    {
        ADF_ERROR("The %s section could not be found.\n", GENERAL_SEC);
        return CPA_STATUS_FAIL;
    }
    /* Make a copy to manipulate on it */
    cpy = ICP_ZALLOC_GEN(strlen(str) + 1);
    ICP_MEMCPY(cpy, str, strlen(str) + 1);
    /* keep the original ptr to free it later */
    str = cpy;
    /* go to the first version str */
    if (strlen(tag) < strlen(cpy))
    {
        cpy = strstr(cpy, tag);
        if(!cpy)
        {
            ADF_ERROR("Invalid version format. %s\n", str);
            ICP_FREE(str);
            return CPA_STATUS_FAIL;
        }
    }
    else
    {
        ADF_ERROR("The tag length is longer than cpy length.\n", GENERAL_SEC);
        return CPA_STATUS_FAIL;
    }

    /* move it to the first digit */
    while(!ICP_ISDIGIT(*cpy))
    {
        cpy++;
    }

    ptr = cpy;
    /* and move till you find a space */
    while(!ICP_ISSPACE(*cpy))
    {
        cpy++;
    }
    /* this is tools version */
    *cpy = '\0';
    ICP_STRNCPY(version, ptr, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    stat = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                   ICP_CFG_TOOLS_VER_KEY,
                                   version, ADF_STR);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't add tools version to config\n");
        ICP_FREE(str);
        return stat;
    }

    ADF_DEBUG("tools version %s", ptr);
    cpy++;
    /* go to the second version str */
    cpy = strstr(cpy, tag);
    if(!cpy)
    {
        ADF_ERROR("Invalid version format. %s\n", str);
        ICP_FREE(str);
        return CPA_STATUS_FAIL;
    }
    /* and move it to the first digit */
    while(!ICP_ISDIGIT(*cpy))
    {
        cpy++;
    }
    /* this is uof version */
    ICP_MEMSET(version, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    ICP_STRNCPY(version, cpy, ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    stat = adf_cfgAddKeyValueParam(pAccelDev, pSectionNode,
                                   ICP_CFG_UOF_VER_KEY,
                                   version, ADF_STR);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Can't add uof version to config\n");
    }
    ADF_DEBUG("uof version %s", cpy);
    ICP_FREE(str);
    return stat;
}

/*
 * Wrapper function which loads the firmware, gets the firmware address
 * and size, and maps the object file
 */
CpaStatus adf_aeFwLoad(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    void* addr = NULL;
    Cpa32U size = 0;
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    status = adf_aefwLoadFirmware(pAccelDev);
    ICP_CHECK_STATUS(status);

    /* Get the UoF FW and Map the memory to the AEs */
    status = adf_aefwGetFirmware(pAccelDev, ADF_FW_UOF_TYPE, &addr, &size);
    ICP_CHECK_STATUS(status);
    /* load ucode for patching, ucode_map */
    status = adf_aeUcodeMap(pAccelDev, addr, size);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to Map the UoF firmware\n");
        return status;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Return error description.
 */
STATIC char* adf_aeUcodeError(int status)
{
    switch(status)
    {
        case ICP_FIRMLOADER_FAIL:
                return "the operation failed";
        case ICP_FIRMLOADER_INVALIDHANDLE:
                return "invalid handle";
        case ICP_FIRMLOADER_BADARG:
                return "bad function argument";
        case ICP_FIRMLOADER_DUPBKP:
                return "duplicate break point";
        case ICP_FIRMLOADER_NOUSTORE:
                return "not ustore available";
        case ICP_FIRMLOADER_BADADDR:
                return "bad address";
        case ICP_FIRMLOADER_BADLIB:
                return "bad debug library -- wasn't initialized";
        case ICP_FIRMLOADER_DISABLED:
                return "acceleration engine or interrupt disabled";
        case ICP_FIRMLOADER_ENABLED:
                return "acceleration engine enabled";
        case ICP_FIRMLOADER_RESET:
                return "acceleration engine is in reset";
        case ICP_FIRMLOADER_TIMEOUT:
                return "the operation exceed th etime limit";
        case ICP_FIRMLOADER_ISSET:
                return "condition/evaluation is set/true";
        case ICP_FIRMLOADER_NOTSET:
                return "condition/evaluation is not set/false";
        case ICP_FIRMLOADER_AEACTIVE:
                return "ae is running";
        case ICP_FIRMLOADER_MEMALLOC:
                return "memory allocation error";
        case ICP_FIRMLOADER_NEIGHAEACTIVE:
                return "neighbour ae is running";
        case ICP_FIRMLOADER_MEMFAIL:
                return "memory access fail";
        case ICP_FIRMLOADER_NOOBJ:
                return "no object";
        case ICP_FIRMLOADER_IMGNOTFND:
                return "image not found";
        case ICP_FIRMLOADER_SYMNOTFND:
                return "symbol not found";
        case ICP_FIRMLOADER_NEIGHNOTFND:
                return "neighbour AE not found";
        case ICP_FIRMLOADER_UOFINCOMPAT:
                return "UOF is incompatible with the chip type/revision";
        case ICP_FIRMLOADER_UOFVERINCOMPAT:
                return "UOF version incompatible with the loader"
                       " -- mismatched uof format";
        case ICP_FIRMLOADER_UNINITVAR:
                return "uninitialized import variable";
        case ICP_FIRMLOADER_EXPRFAIL:
                return "expression fail";
        case ICP_FIRMLOADER_EXCDDRAMSIZE:
                return "address exceed dram0 size";
        case ICP_FIRMLOADER_EXCDDRAM1SIZE:
                return "address exceed dram1 size";
        case ICP_FIRMLOADER_EXCDSRAMSIZE:
                return "address exceed sram size";
        case ICP_FIRMLOADER_EXCDSCRTHSIZE:
                return "address exceed scratch size";
        case ICP_FIRMLOADER_EXCDLMEMSIZE:
                return "address exceed local memory size";
        case ICP_FIRMLOADER_INVLDCTX:
                return "invalid context";
        case ICP_FIRMLOADER_EXCDUMEMSIZE:
                return "address exceed ustore memory size";
        case ICP_FIRMLOADER_ADDRNOTFOUND:
                return "address not found";
        case ICP_FIRMLOADER_PAGENOTFOUND:
                return "page not found";
        case ICP_FIRMLOADER_IVDWARN:
                return "unknown image or symbol defined";
        case ICP_FIRMLOADER_EXCDSHRAMSIZE:
                return "address exceed shared ram size";
        default:
                return "unknown error";
    }
}

/*
 *  Map the Uclo Object File (UOF) objects from the memory location
 *  described by the 'addr' and 'size' input parameters.
 */
CpaStatus adf_aeUcodeMap(icp_accel_dev_t *pAccelDev, void* addr, Cpa32U size)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_fw_loader_handle_t* ldr_handle = NULL;
    void *uof_addr = NULL;
    char uof_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    unsigned int uof_size = 0;
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;

    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    status = icp_adf_cfgGetParamValue(pAccelDev, GENERAL_SEC,
                                      ADF_AE_FW_UOF_NAME_KEY, uof_name);

    ICP_CHECK_STATUS_AND_LOG(status,
            "Parameter %s is not configured.\n", ADF_AE_FW_UOF_NAME_KEY);

    loader_status = icp_FirmLoader_MapMofAddr(ldr_handle->firmLoaderHandle,
                                              addr, size, uof_name,
                                              &uof_addr, &uof_size);

    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Mapping of MOF Firmware failed, status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));
        return CPA_STATUS_FAIL;
    }

    loader_status = icp_FirmLoader_MapObjAddr(ldr_handle->firmLoaderHandle,
                                              uof_addr, uof_size, 1);

    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Mapping of UOF Firmware failed, status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * Binds a value to the Ucode symbol in the UOF image.
 * This is a wrapper function to two internal functions
 * which patch the symbols and also store the symbols
 * in a config table.
 */
CpaStatus icp_adf_aePatchSymbol(icp_accel_dev_t *pAccelDev,
                                Cpa32S aeNum,
                                char* pSymName,
                                Cpa32S symValue)
{

    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pSymName);

    /*Patch the Symbols.*/
    status = adf_aePatchSymbol(pAccelDev, (unsigned char)aeNum,
                               pSymName, symValue);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to patch symbols\n");
        return CPA_STATUS_FAIL;
    }

    /*Store the symbols in for use in case of a suspend & resume.*/
    status = adf_aeStoreSymbols(pAccelDev, aeNum, pSymName, symValue);
    if(CPA_STATUS_SUCCESS != status)
    {
          ADF_ERROR("Failed to store the symbols into the config table\n");
          return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * Function which binds the symbols.
 */
CpaStatus adf_aePatchSymbol(icp_accel_dev_t *pAccelDev,
                                Cpa32S aeNum,
                                char* pSymName,
                                Cpa32S symValue)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;

    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    loader_status = icp_FirmLoader_AeBindSymbol(ldr_handle->firmLoaderHandle,
                                   (unsigned char)aeNum, pSymName, symValue);
    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Patching of symbol failed,status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}
/*
 * Place the Symbols into the  adf_cfg_device_data_t structure
 * in case a suspend is initiated.
 */
CpaStatus adf_aeStoreSymbols(icp_accel_dev_t *pAccelDev,
                             Cpa32S aeNum,
                             char* pSymName,
                             Cpa32S symValue)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_device_data_t *pCfgData = NULL;
    adf_cfg_section_t *section_symbols=NULL;
    Cpa64U val = (Cpa64U) symValue;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    section_symbols = ICP_ZALLOC_GEN(sizeof(adf_cfg_section_t));
    if(NULL == section_symbols)
    {
        ADF_ERROR("Unable to allocate memory for section_symbols.\n");
        return CPA_STATUS_FAIL;
    }

    ICP_STRNCPY(section_symbols->name, SYMBOLS_SEC, ICP_STRLEN(SYMBOLS_SEC));
    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to find the device while attempting to "
                  "patch symbols.\n");
        ICP_FREE(section_symbols);
        return CPA_STATUS_FAIL;
    }

    /* This information is intended for the symbol list
     * on this device. A section with the name "SYMBOLS"
     * is created within the symbol section.
     */
    status = adf_cfgSectionAdd(pAccelDev,
                                     &(pCfgData->symbol_section),
                                     section_symbols);
    if (status)
    {
        ADF_ERROR("Failed to add section %s.\n",
                      section_symbols->name);
        return status;
    }
    /* Add the key value pairs to the symbols section. */
    status = adf_cfgAddKeyValueParam(pAccelDev,
                                     pCfgData->symbol_section,
                                     pSymName ,
                                     (void *)&val,
                                     ADF_DEC);
    if (status)
    {
        ADF_ERROR("Failed to add key %s to section %s.\n",
                      pSymName, section_symbols->name);
        return status;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Removes the Symbols from the adf_cfg_device_data_t structure
 */
CpaStatus adf_aeRemoveSymbols(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_device_data_t *pCfgData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    status = adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to find the device while attempting to"
                    "patch symbols.\n");
        return CPA_STATUS_FAIL;
    }
    adf_cfgSectionDel(&pCfgData->symbol_section);
    return CPA_STATUS_SUCCESS;
}

/*
 * Writes all of the UOF images to the assigned AEs
 */
CpaStatus adf_aeUcodeDownload(icp_accel_dev_t *pAccelDev)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    loader_status =
                 icp_FirmLoader_WriteUimageAll(ldr_handle->firmLoaderHandle);
    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Writing of firmware to engines failed, status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Removes and frees all resource references associated with the handle.
 */
CpaStatus adf_aeUcodeRelease(icp_accel_dev_t *pAccelDev)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    /* Clean MMP and SHRAM if mem scrub is supported*/
    if ((CPA_TRUE == pAccelDev->pHwDeviceData->isMemScrubSupported) &&
             !BIT_IS_SET(pAccelDev->adfSubsystemStatus,
                         ADF_STATUS_SYSTEM_RESTARTING))
    {
        if (icp_FirmLoader_InitMemory(ldr_handle->firmLoaderHandle, ALL_TYPE)
            != ICP_FIRMLOADER_SUCCESS) {
            ADF_ERROR("Failed to init QAT memory\n");
        }
    }

    if (CPA_STATUS_SUCCESS != adf_aeReset(pAccelDev, 0, 0))
    {
        ADF_ERROR("Failed to reset the AEs\n");
    }

    loader_status = icp_FirmLoader_Deinit(ldr_handle->firmLoaderHandle);
    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Removing of firmware loader handle failed,"
                      " status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));
        ldr_handle->firmLoaderHandle = NULL;
        return CPA_STATUS_FAIL;
    }

    ldr_handle->firmLoaderHandle = NULL;
    return CPA_STATUS_SUCCESS;
}

/*
 * Release the Firmware.
 */
CpaStatus adf_aeReleaseFirmware(icp_accel_dev_t *pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    /* Release in-kernel copy of the firmware */
    if (adf_aefwReleaseFirmware(pAccelDev) != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Unable to release Ucode.\n");
    }
    return CPA_STATUS_SUCCESS;
}


/*
 * Start AE context from current program counter
 */
CpaStatus adf_aeStart(icp_accel_dev_t *pAccelDev)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;
    icp_firml_sys_mem_info_t *sys_mem_info = NULL;
    Cpa32U ae_ctr = 0, ae_num = 0;
    Cpa32U maxNumAccelEngines = 0;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);

    maxNumAccelEngines = GET_MAX_ACCELENGINES(pAccelDev);

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    sys_mem_info = (icp_firml_sys_mem_info_t *)ldr_handle->sysMemInfo;
    ICP_CHECK_FOR_NULL_PARAM(sys_mem_info);

    for (ae_num = 0, ae_ctr = 0;
                ae_num < maxNumAccelEngines && ae_ctr < sys_mem_info->numAe;
                        ae_num++)
    {
        if (sys_mem_info->aeMask & (1 << ae_num))
        {
            loader_status = icp_FirmLoader_Start(ldr_handle->firmLoaderHandle,
                                                ae_num, ADF_AE_CTX_ENABLE_MASK);
            if (ICP_FIRMLOADER_SUCCESS != loader_status)
            {
                ADF_ERROR("Failed to start AE %d status=0x%x \"%s\"\n",
                       ae_num, loader_status, adf_aeUcodeError(loader_status));
                return CPA_STATUS_FAIL;
            }
            ADF_PRINT("Started AE %d\n", ae_num);
            ae_ctr++;
        }
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * Disable the specified micro-thread(s)
 */
CpaStatus adf_aeStop(icp_accel_dev_t *pAccelDev)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;
    icp_firml_sys_mem_info_t *sys_mem_info = NULL;
    Cpa32U ae_ctr = 0, ae_num = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U maxNumAccelEngines = 0;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pUcodePrivData);

    hw_data = pAccelDev->pHwDeviceData;
    maxNumAccelEngines = hw_data->maxNumAccelEngines;

    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    ICP_CHECK_FOR_NULL_PARAM(ldr_handle->firmLoaderHandle);

    sys_mem_info = (icp_firml_sys_mem_info_t *)ldr_handle->sysMemInfo;
    ICP_CHECK_FOR_NULL_PARAM(sys_mem_info);

    for (ae_num = 0, ae_ctr = 0;
                ae_num < maxNumAccelEngines && ae_ctr < sys_mem_info->numAe;
                        ae_num++)
    {
        if (sys_mem_info->aeMask & (1 << ae_num))
        {
            loader_status = icp_FirmLoader_Stop(ldr_handle->firmLoaderHandle,
                                                ae_num, ADF_AE_CTX_ENABLE_MASK);
            if (ICP_FIRMLOADER_SUCCESS != loader_status)
            {
                ADF_ERROR("Failed to stop AE %dstatus=0x%x \"%s\"\n",
                       ae_num, loader_status, adf_aeUcodeError(loader_status));
                return CPA_STATUS_FAIL;
            }
            ADF_PRINT("Stopped AE %d\n", ae_num);
            ae_ctr++;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Reset the specified AccelEngines and conditionally clear the
 * registers to their power-up state. After this is done take
 * the specified AccelEngines out of reset and init AEs
 */
CpaStatus adf_aeReset(icp_accel_dev_t *pAccelDev,
                      Cpa32S aeNum,
                      Cpa32S ae_ClrRegs)
{
    int loader_status1 = ICP_FIRMLOADER_FAIL;
    int loader_status2 = ICP_FIRMLOADER_FAIL;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    loader_status1 = icp_FirmLoader_Reset(ldr_handle->firmLoaderHandle,
                                           ldr_handle->sysMemInfo->aeMask,
                                           ae_ClrRegs);
    loader_status2 = icp_FirmLoader_ClrReset(ldr_handle->firmLoaderHandle,
                                              ldr_handle->sysMemInfo->aeMask);
    if((ICP_FIRMLOADER_SUCCESS != loader_status1) ||
       (ICP_FIRMLOADER_SUCCESS != loader_status2))
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Initialize the AccelEngines and take them out of reset
 */
CpaStatus adf_aeInit(icp_accel_dev_t *pAccelDev)
{
    int loader_status = ICP_FIRMLOADER_FAIL;
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);

    if(NULL != pAccelDev->pUcodePrivData)
    {
        ADF_ERROR("Firmware Loader already initialised.\n");
        return CPA_STATUS_FAIL;
    }
    ldr_handle = (adf_fw_loader_handle_t *)
                        ICP_ZALLOC_GEN(sizeof(adf_fw_loader_handle_t));
    if(NULL == ldr_handle)
    {
        ADF_ERROR("Unable to allocate memory for ldr_handle.\n");
        return CPA_STATUS_FAIL;
    }
    ldr_handle->sysMemInfo = (icp_firml_sys_mem_info_t *)
                        ICP_ZALLOC_GEN(sizeof(icp_firml_sys_mem_info_t));
    if(NULL == ldr_handle->sysMemInfo)
    {
        ADF_ERROR("Unable to allocate memory for sysMemInfo.\n");
        ICP_FREE(ldr_handle);
        ldr_handle = NULL;
        return CPA_STATUS_FAIL;
    }

    /*
     * No need to check return value here as this cannot return
     * in this instance.
     */
    stat = adf_initSysMemInfo(pAccelDev, ldr_handle->sysMemInfo);
    loader_status = icp_FirmLoader_Init((ldr_handle->sysMemInfo),
                                       &ldr_handle->firmLoaderHandle);
    if (ICP_FIRMLOADER_SUCCESS != loader_status)
    {
        ADF_ERROR("Failed to initialise AE status=0x%x \"%s\"\n",
                       loader_status, adf_aeUcodeError(loader_status));

        ICP_FREE(ldr_handle->sysMemInfo);
        ldr_handle->sysMemInfo = NULL;
        ICP_FREE(ldr_handle);
        ldr_handle = NULL;
        return CPA_STATUS_FAIL;
    }
    pAccelDev->pUcodePrivData = (void*) ldr_handle;
    stat = adf_aeReset(pAccelDev, 0, 0);
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Failed to reset the AEs\n");
    }

    /* Clean MMP memory if mem scrub is supported*/
    if(CPA_TRUE == pAccelDev->pHwDeviceData->isMemScrubSupported)
    {
        if (icp_FirmLoader_InitMemory(ldr_handle->firmLoaderHandle, MMP_TYPE)
            != ICP_FIRMLOADER_SUCCESS) {
            ADF_ERROR("Failed to init QAT MMP memory\n");
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Shutdown the AccelEngines
 */
CpaStatus adf_aeShutdown(icp_accel_dev_t *pAccelDev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_fw_loader_handle_t* ldr_handle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    adf_aeRemoveSymbols(pAccelDev);
    ldr_handle = (adf_fw_loader_handle_t*) pAccelDev->pUcodePrivData;
    stat = adf_cleanSysMemInfo(pAccelDev, ldr_handle->sysMemInfo);
    if(CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Failed to clean sysMemInfo.\n");
    }
    ICP_FREE(ldr_handle->sysMemInfo);
    ldr_handle->sysMemInfo = NULL;
    ICP_FREE(ldr_handle);
    pAccelDev->pUcodePrivData = NULL;
    return stat;
}
