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
 * @file adf_cfg_dp.c
 *
 * @description
 *      This file contains Acceleration Driver Framework code for
 *      run-time configuration control of Acceleration Subsystem for data plain
 *
 *****************************************************************************/

#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_cfg.h"
#include "adf_cfg_types.h"

/*
 * icp_adf_cfgGetSharedMemSize
 *
 * This function is used to determine the size required for configuration data.
 * in shared memory. Client will allocate the memory buffer.
 */
Cpa32U icp_adf_cfgGetSharedMemSize(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int num_of_entries = 0, num_of_all_entries = 0,
        num_of_sections = 0;
    adf_cfg_device_data_t *config_data = NULL;
    adf_cfg_section_t *config_section = NULL;
    adf_cfg_key_val_t *config_value_list = NULL;

    if(NULL == accel_dev)
    {
        ADF_ERROR("Wrong parameter accel_dev\n");
        return 0;
    }
    status = adf_cfgDeviceFind(accel_dev->accelId, &config_data);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Unable to get configuration for device %d\n",
                                                accel_dev->accelId);
        return 0;
    }

    /* Add all config entries */
    config_section = config_data->config_section;
    while(config_section != NULL)
    {
        config_value_list = config_section->params;
        while (config_value_list != NULL)
        {
            config_value_list = config_value_list->pNext;
            num_of_entries++;
        }
        config_section = config_section->pNext;
        num_of_sections++;
        num_of_all_entries += num_of_entries;
    }

    config_section = config_data->internal_section;
    while(config_section != NULL)
    {
        config_value_list = config_section->params;
        while (config_value_list != NULL)
        {
            config_value_list = config_value_list->pNext;
            num_of_entries++;
        }
        config_section = config_section->pNext;
        num_of_sections++;
        num_of_all_entries += num_of_entries;
    }

    /* return size of all entries plus size of header */
    return sizeof(adf_dp_config_header_t) +
                   ((num_of_sections + num_of_all_entries + 1)
                                     * sizeof(adf_dp_config_entry_t));
}


/*
 * icp_adf_cfgExportData
 *
 * This function exports the configuration to shared memory buffer.
 * The buffer needs to be allocated by the client. Size of the buffer can by
 * determined using icp_adf_cfgGetSharedMemSize function.
 */
CpaStatus icp_adf_cfgExportData(icp_accel_dev_t *accel_dev,
                                                void *addr)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int num_of_entries = 0, num_of_sections = 0;
    adf_cfg_device_data_t *config_data = NULL;
    adf_cfg_section_t *config_section = NULL;
    adf_cfg_key_val_t *config_value_list = NULL;
    adf_dp_config_header_t *hdr = NULL;
    adf_dp_config_entry_t *entry = NULL;
    char *ptr = addr;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(addr);

    status = adf_cfgDeviceFind(accel_dev->accelId, &config_data);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Unable to get configuration for device %d\n",
                                                accel_dev->accelId);
        return CPA_STATUS_FAIL;
    }

    hdr = (adf_dp_config_header_t*) ptr;
    hdr->dev_id = accel_dev->accelId;
    ptr = ptr + sizeof(adf_dp_config_header_t);
    entry = (adf_dp_config_entry_t*) ptr;
    config_section = config_data->config_section;

    /* Export everything from config section */
    while(config_section != NULL)
    {
        adf_dp_config_entry_t *sec_entry = entry;
        num_of_entries = 0;
        config_value_list = config_section->params;
        entry->type = ADF_DP_ENTRY_TYPE_SECTION;
        strncpy((char *)entry->ADF_CFG_SECTION.name, config_section->name,
            ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        entry++;
        while (config_value_list != NULL)
        {
            num_of_entries++;
            entry->type = ADF_DP_ENTRY_TYPE_VALUE;
            strncpy((char *)entry->ADF_CFG_VALUE.name, config_value_list->key,
                ADF_CFG_MAX_KEY_LEN_IN_BYTES);
            strncpy((char *)entry->ADF_CFG_VALUE.value, config_value_list->val,
                ADF_CFG_MAX_VAL_LEN_IN_BYTES);
            entry++;
            config_value_list = config_value_list->pNext;
        }
        sec_entry->ADF_CFG_SECTION.number_of_values = num_of_entries;
        num_of_sections++;
        config_section = config_section->pNext;
    }

    /* Export everything from internal section */
    config_section = config_data->internal_section;
    while(config_section != NULL)
    {
        adf_dp_config_entry_t *sec_entry = entry;
        num_of_entries = 0;
        config_value_list = config_section->params;
        entry->type = ADF_DP_ENTRY_TYPE_SECTION;
        strncpy((char *)entry->ADF_CFG_SECTION.name, config_section->name,
            ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        entry++;
        while (config_value_list != NULL)
        {
            num_of_entries++;
            entry->type = ADF_DP_ENTRY_TYPE_VALUE;
            strncpy((char *)entry->ADF_CFG_VALUE.name, config_value_list->key,
                ADF_CFG_MAX_KEY_LEN_IN_BYTES);
            strncpy((char *)entry->ADF_CFG_VALUE.value, config_value_list->val,
                ADF_CFG_MAX_VAL_LEN_IN_BYTES);
            entry++;
            config_value_list = config_value_list->pNext;
        }
        sec_entry->ADF_CFG_SECTION.number_of_values = num_of_entries;
        num_of_sections++;
        config_section = config_section->pNext;
    }

    /* Mark config data as valid */
    entry->type = ADF_DP_ENTRY_TYPE_END;
    hdr->number_of_sections = num_of_sections;
    hdr->valid = ADF_CFG_HEADER_VERIFIER;
    return status;
}

