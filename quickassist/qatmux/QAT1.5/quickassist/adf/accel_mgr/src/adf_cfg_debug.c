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
 * @file adf_cfg_debug.c
 *
 * @description
 *      This file contains OS specific Acceleration Driver Framework code for
 *      run-time configuration debug of Acceleration Subsystem.
 *
 *****************************************************************************/
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_debug.h"
#include "adf_cfg.h"

/* Proc entry names */
#define ADF_DEBUG_CFG     "cfg_debug"
#define ADF_DEBUG_CFG_INT ".cfg_debug_int"
/* Max length of the line */
#define DEBUG_LINE_LEN 150
#define DEBUG_SECTION_LINE_LEN 70
#define DEBUG_FINENAME_LEN 15

typedef struct cfg_debug_info_s {
    debug_file_info_t cfg;
    char cfg_name[DEBUG_FINENAME_LEN];
    debug_file_info_t int_cfg;
    char int_cfg_name[DEBUG_FINENAME_LEN];
} cfg_debug_info_t;

/*
 * adf_debug_get_cfg_type
 * converts cfg_val_type to string
 */
STATIC const char* adf_debug_get_cfg_type(adf_cfg_val_type_t t)
{
    switch (t)
    {
        case ADF_DEC: return "DEC";
        case ADF_HEX: return "HEX";
        case ADF_STR: return "STR";
        default: break;
    }
    return "UNKNOWN";
}

/*
 * adf_cfgSectionList
 * List all sections and associated key-value pairs
 */
STATIC int adf_cfg_debug_print(void *prv, char* data,
                               int size, int offset)
{
    adf_cfg_device_data_t *dev_cfg = (adf_cfg_device_data_t*) prv;
    adf_cfg_section_t *section;
    adf_cfg_key_val_t *pKeyValueList;
    Cpa32U len = 0, index = 0;

    if(!dev_cfg || !data)
    {
        return 0;
    }

    section = dev_cfg->config_section;
    if(offset == 0)
    {
        len += snprintf(data+len, size-len,
                       "\n### Config Table ###\n");
    }
    while(section != NULL)
    {
        if(offset <= index)
        {
            if(size < DEBUG_SECTION_LINE_LEN + len)
            {
                return offset;
            }
            len += snprintf(data+len, size-len,
                       "\n[%s]\n", section->name);
            offset++;
        }
        index++;
        pKeyValueList = section->params;
        while (pKeyValueList != NULL)
        {
            if(size < DEBUG_LINE_LEN + len)
            {
                return offset;
            }
            if(offset <= index)
            {
                len += snprintf(data+len, size-len,
                     "%s = %s \t# %s\n",
                      pKeyValueList->key, pKeyValueList->val,
                      adf_debug_get_cfg_type(pKeyValueList->type));
                offset++;
            }
            index++;
            pKeyValueList = pKeyValueList->pNext;
        }
        section = section->pNext;
    }
    return 0;
}

/*
 * adf_cfgSectionList
 * List all sections and associated key-value pairs
 * from the internal section
 */
STATIC int adf_cfg_debug_print_int(void *prv, char* data,
                                   int size, int offset)
{
    adf_cfg_device_data_t *dev_cfg = (adf_cfg_device_data_t*) prv;
    adf_cfg_section_t *section;
    adf_cfg_key_val_t *pKeyValueList;
    Cpa32U len = 0, index = 0;

    if(!dev_cfg || !data)
    {
        return 0;
    }
    section = dev_cfg->internal_section;
    if(offset == 0)
    {
        len += snprintf(data+len, size-len,
                   "\n### Internal Table ###\n");
    }
    while(section != NULL)
    {
        if(offset <= index)
        {
            if(size < DEBUG_SECTION_LINE_LEN + len)
            {
                return offset;
            }
            len += snprintf(data+len, size-len,
                       "\n[%s]\n", section->name);
            offset++;
        }
        index++;
        pKeyValueList = section->params;
        while (pKeyValueList != NULL)
        {
            if(size < DEBUG_LINE_LEN + len)
            {
                return offset;
            }
            if(offset <= index)
            {
                len += snprintf(data+len, size-len,
                     "%s = %s \t# %s\n",
                      pKeyValueList->key, pKeyValueList->val,
                      adf_debug_get_cfg_type(pKeyValueList->type));
                offset++;
            }
            index++;
            pKeyValueList = pKeyValueList->pNext;
        }
        section = section->pNext;
    }
    return 0;
}

/*
 * adf_debug_create_cfg
 * Create a debug file for device config table
 */
CpaStatus adf_debug_create_cfg(icp_accel_dev_t *accel_dev,
                               adf_cfg_device_data_t *dev_cfg)
{
    debug_file_info_t *file;
    cfg_debug_info_t *cfg_info = ICP_ZALLOC_GEN(sizeof(cfg_debug_info_t));
    if(NULL == cfg_info)
    {
        ADF_ERROR("Failed to allocate memory for cfg debug file\n");
        return CPA_STATUS_FAIL;
    }
    file = &cfg_info->cfg;
    file->name = cfg_info->cfg_name;
    sprintf(file->name, ADF_DEBUG_CFG);
    file->parent = NULL;
    file->seq_read = adf_cfg_debug_print;
    file->private_data = dev_cfg;
    if(CPA_STATUS_SUCCESS != icp_adf_debugAddFile(accel_dev, file))
    {
        ADF_ERROR("Failed to create debug file for cfg\n");
        ICP_FREE(cfg_info);
        return CPA_STATUS_FAIL;
    }
    file = &cfg_info->int_cfg;
    file->name = cfg_info->int_cfg_name;
    sprintf(file->name, ADF_DEBUG_CFG_INT);
    file->parent = NULL;
    file->seq_read = adf_cfg_debug_print_int;
    file->private_data = dev_cfg;
    if(CPA_STATUS_SUCCESS != icp_adf_debugAddFile(accel_dev, file))
    {
        ADF_ERROR("Failed to create debug file for internal cfg\n");
        icp_adf_debugRemoveFile(&cfg_info->cfg);
        ICP_FREE(cfg_info);
        return CPA_STATUS_FAIL;
    }
    dev_cfg->debug_hndl = cfg_info;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_debug_remove_cfg
 * Remove debug file for device config table
 */
void adf_debug_remove_cfg(adf_cfg_device_data_t *dev_cfg)
{
    cfg_debug_info_t *cfg_info = dev_cfg->debug_hndl;
    if(cfg_info)
    {
        icp_adf_debugRemoveFile(&cfg_info->cfg);
        icp_adf_debugRemoveFile(&cfg_info->int_cfg);
        ICP_FREE(cfg_info);
        dev_cfg->debug_hndl = NULL;
    }
}

